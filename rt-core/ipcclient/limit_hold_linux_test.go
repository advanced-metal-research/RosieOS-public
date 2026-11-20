//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"math"
	"path/filepath"
	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"
)

// Compile a zero-excluding range in rad; compiler acceptance is part of the
// contract so Home cannot hide an exterior zero behind configuration rejection.
func limitHoldCore(t *testing.T, requireHome, qualified bool) fixture {
	t.Helper()
	machine, profile := limitHoldInputs(t, requireHome, qualified)
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	f, _ := coordinateModelDaemon(t, compiled, digest)
	return f
}

func limitHoldInputs(t *testing.T, requireHome, qualified bool) (map[string]any, map[string]any) {
	t.Helper()
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	machine, profile := compiledCoordinateInputs(t)
	axis := machine["axes"].([]any)[0].(map[string]any)
	axis["require_home"] = requireHome
	limits := axis["limits"].(map[string]any)
	limits["min"], limits["max"] = .01, .1
	if !qualified {
		delete(profile, "coordinate_evidence")
	}
	return machine, profile
}

func limitHoldResult(t *testing.T, c *Client, sequence uint64, err error) CommandResult {
	t.Helper()
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	result, err := c.WaitResult(ctx, sequence)
	var rejected *RejectedError
	if err != nil && !errors.As(err, &rejected) {
		t.Fatal(err)
	}
	return result
}

// Timeout reconciles a one-count tracking lag to trusted feedback while the
// inward jog is still below the minimum; arrest must preserve enabled hold.
func TestReviewLimitHoldTimeoutSnap(t *testing.T) {
	machine, profile := limitHoldInputs(t, false, false)
	// Exact fixture boundary: 1000 counts, with a 500-count initial pose.
	machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)["min"] = 1000 * (2 * math.Pi) / 131072
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	f, _ := coordinateModelDaemon(t, compiled, digest, "--sim-initial-position-counts", "500", "--sim-feedback-offset-counts", "-1")
	c := f.client
	arm(t, c, 1)
	wait(t, c, allOperationEnabled)
	seq, err := c.Jog(context.Background(), 1, []float64{.02}, 200*time.Millisecond)
	if r := limitHoldResult(t, c, seq, err); r.Result != ResultAccepted {
		t.Fatal(r)
	}
	lag := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		return d.ActiveJog == 1 && d.FeedbackPosCounts[0] >= 510 &&
			d.OutputTargetCounts[0]-d.FeedbackPosCounts[0] == 1
	})
	if lag.FeedbackPosCounts[0] >= 700 {
		t.Fatal(lag)
	}
	arrest := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		return d.LastStopCmdSeq == seq && d.LastStopReason == JOG_STOP_REASON_TIMEOUT && d.StopArrestMask == 1
	})
	if arrest.OutputTargetCounts[0] != arrest.FeedbackPosCounts[0] || arrest.OutputTargetVelocityCountsPerS[0] != 0 {
		t.Fatal(arrest)
	}
	waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		return d.LastStopCmdSeq == seq && d.StopArrestMask == 0
	})
	s := wait(t, c, func(s Status) bool { return s.TimeNS > arrest.SampleTimeNs })
	if s.Armed != 1 || s.AxisEnableMask != 1 || s.ExecutionFaultReasons != 0 ||
		s.Axes[0].PositionCounts < 200 || s.Axes[0].PositionCounts >= 700 {
		t.Fatal(s)
	}
}

// Equal logical samples stay stationary by construction. An outward change
// of one count, or a fraction of a count, must still refuse the whole prefix.
func TestReviewLimitHoldStationaryPrefix(t *testing.T) {
	c := limitHoldCore(t, false, false).client
	ctx := context.Background()
	for index, outward := range []float64{0, 1, .25} {
		points := make([]Point, 200)
		for sample := range points {
			points[sample] = Point{Time: time.Duration(sample) * time.Millisecond, Position: []float64{.007}}
		}
		// Exact fixture conversion: 131072 command counts per revolution.
		points[100].Position[0] -= outward * (2 * math.Pi) / 131072
		handle, err := c.PrepareTrajectory(ctx, uint64(131+index), 1, points)
		if outward == 0 {
			if err != nil || handle == 0 {
				t.Fatal(handle, err)
			}
		} else {
			var rejected *RejectedError
			if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonOutsideLimitsOutward {
				t.Fatal(handle, err)
			}
		}
	}
}

// Baseline: Home gates motion at an in-range zero, and tightening limits around
// that same zero invalidates an old anchor's identity without moving the axis.
func TestLimitHoldPreChangeGatingBaseline(t *testing.T) {
	machine, profile := compiledCoordinateInputs(t)
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "baseline-anchors")}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	f, stop := coordinateModelDaemon(t, compiled, digest)
	assertGated := func(c *Client, id uint64) {
		t.Helper()
		ctx := context.Background()
		s := wait(t, c, func(s Status) bool { return s.ConfigVerifiedMask == 1 })
		if s.HomeValidMask != 0 || s.Armed != 0 || s.AxisEnableMask != 0 ||
			s.Axes[0].PositionCounts != 0 || s.ExecutionFaultReasons != 0 || s.Axes[0].Readiness != "home_required" {
			t.Fatal(s)
		}
		seq, err := c.Enable(ctx, 1)
		if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
			t.Fatal(r)
		}
		seq, err = c.Arm(ctx)
		if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
			t.Fatal(r)
		}
		// Exact fixture request: one count/s and a one-count trajectory endpoint.
		oneCount := 2 * math.Pi / 131072
		seq, err = c.Jog(ctx, 1, []float64{oneCount}, 200*time.Millisecond)
		if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
			t.Fatal(r)
		}
		handle, err := c.PrepareTrajectory(ctx, id, 1, []Point{{Position: []float64{0}}, {Time: time.Second, Position: []float64{oneCount}}})
		if err != nil {
			t.Fatal(err)
		}
		_, err = c.StartTrajectory(ctx, handle)
		var rejected *RejectedError
		if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonNotReady {
			t.Fatal(err)
		}
		d := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool { return d.SampleTimeNs > s.TimeNS })
		if d.FeedbackPosCounts[0] != 0 || d.OutputTargetCounts[0] != 0 || d.OutputTargetVelocityCountsPerS[0] != 0 {
			t.Fatal(d)
		}
	}
	assertGated(f.client, 141)
	coordinateHome(t, f, 1)
	s := wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.Axes[0].LogicalValid })
	if s.Axes[0].PositionCounts != 0 || s.Axes[0].CoordinateCounts != 0 || s.Axes[0].HomeOffset != 0 {
		t.Fatal(s)
	}
	if err = store.Save(coordinateModelRecord(s)); err != nil {
		t.Fatal(err)
	}
	stop()
	// Exact tightened interval: [-100, 100] counts still contains the saved zero.
	limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
	limits["min"], limits["max"] = -100*(2*math.Pi)/131072, 100*(2*math.Pi)/131072
	revised, revisedDigest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil || revisedDigest == digest {
		t.Fatal(err, revisedDigest)
	}
	f, _ = coordinateModelDaemon(t, revised, revisedDigest)
	s = wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_ready" })
	if s.Axes[0].PositionCounts != 0 {
		t.Fatal(s)
	}
	coordinateModelRestore(t, f, true)
	s = wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 })
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].CoordinateCounts != 0 {
		t.Fatal(s)
	}
}

func TestLimitHoldHomeGate(t *testing.T) {
	for _, qualified := range []bool{false, true} {
		name := "plain"
		if qualified {
			name = "qualified"
		}
		t.Run(name, func(t *testing.T) {
			c := limitHoldCore(t, true, qualified).client
			ctx := context.Background()
			s := wait(t, c, func(s Status) bool { return s.ConfigVerifiedMask == 1 })
			if s.HomeValidMask != 0 || s.Armed != 0 || s.Axes[0].LogicalValid || s.Axes[0].Readiness != "home_required" {
				t.Fatal(s)
			}
			seq, err := c.Enable(ctx, 1)
			if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
				t.Fatal(r)
			}
			seq, err = c.Arm(ctx)
			if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
				t.Fatal(r)
			}
			seq, err = c.Jog(ctx, 1, []float64{-.01}, 100*time.Millisecond)
			if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonNotReady {
				t.Fatal(r)
			}
			handle, err := c.PrepareTrajectory(ctx, 31, 1, []Point{{Position: []float64{.01}}, {Time: time.Second, Position: []float64{.02}}})
			if err != nil {
				t.Fatal(err)
			}
			_, err = c.StartTrajectory(ctx, handle)
			var rejected *RejectedError
			if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonNotReady {
				t.Fatal(err)
			}
			seq, err = c.Home(ctx, 1)
			result := limitHoldResult(t, c, seq, err)
			if result.Result != ResultAccepted {
				t.Fatal(result)
			}
			wait(t, c, func(s Status) bool { return s.HomeValidMask == 1 })
			seq, err = c.Enable(ctx, 1)
			if r := limitHoldResult(t, c, seq, err); r.Result != ResultAccepted {
				t.Fatal(r)
			}
			seq, err = c.Arm(ctx)
			if r := limitHoldResult(t, c, seq, err); r.Result != ResultAccepted {
				t.Fatal(r)
			}
			s = wait(t, c, func(s Status) bool { return s.Axes[0].Readiness == "ready" })
			if s.HomeValidMask != 1 || s.ExecutionFaultReasons != 0 || s.Axes[0].PositionCounts != 0 || s.Axes[0].LogicalTarget != 0 {
				t.Fatal(s)
			}
			seq, err = c.Jog(ctx, 1, []float64{-.01}, 200*time.Millisecond)
			if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonOutsideLimitsOutward {
				t.Fatal(r)
			}
			handle, err = c.PrepareTrajectory(ctx, 32, 1, []Point{{Position: []float64{-.001}}, {Time: time.Second, Position: []float64{.01}}})
			if err != nil {
				t.Fatal(err)
			}
			_, err = c.StartTrajectory(ctx, handle)
			if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonOutsideLimitsOutward {
				t.Fatal(err)
			}
		})
	}
}

// Exterior bring-up stays still, rejects outward requests with a stable native
// reason, and only an explicit inward update can move the axis into its range.
func TestLimitHoldJogAndTrajectory(t *testing.T) {
	f := limitHoldCore(t, false, false)
	c := f.client
	ctx := context.Background()
	arm(t, c, 1)
	s := wait(t, c, func(s Status) bool { return s.Axes[0].Readiness == "ready" })
	if s.Axes[0].LogicalPosition != 0 || s.Axes[0].LogicalTarget != 0 || s.ExecutionFaultReasons != 0 {
		t.Fatal(s)
	}
	after := HostMonotonicNS()
	waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		if d.SampleTimeNs < after {
			return false
		}
		if d.OutputTargetCounts[0] != 0 || d.OutputTargetVelocityCountsPerS[0] != 0 {
			t.Fatal(d)
		}
		return d.SampleTimeNs >= after+100_000_000
	})
	seq, err := c.Jog(ctx, 1, []float64{-.02}, 200*time.Millisecond)
	if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonOutsideLimitsOutward {
		t.Fatal(r)
	}
	// A prepared exterior prefix is inert. Its first point is outward from the
	// current hold, so Start must refuse it without a discontinuity fault.
	handle, err := c.PrepareTrajectory(ctx, 81, 1, []Point{{Position: []float64{-.001}}, {Time: time.Second, Position: []float64{.01}}})
	if err != nil {
		t.Fatal(err)
	}
	_, err = c.StartTrajectory(ctx, handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonOutsideLimitsOutward {
		t.Fatal(err)
	}
	// At 131072 counts/revolution, -0.00004 rad rounds to -1 count.
	// Start must not truncate it to zero and accept it as continuous hold.
	handle, err = c.PrepareTrajectory(ctx, 84, 1, []Point{{Position: []float64{-.00004}}, {Time: time.Second, Position: []float64{.01}}})
	if err != nil {
		t.Fatal(err)
	}
	_, err = c.StartTrajectory(ctx, handle)
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonOutsideLimitsOutward {
		t.Fatal(err)
	}
	handle, err = c.PrepareTrajectory(ctx, 82, 1, []Point{{Position: []float64{0}}, {Time: time.Second, Position: []float64{-.001}}})
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonOutsideLimitsOutward {
		t.Fatal(handle, err)
	}
	id := GrantIdentity{CapabilityID: [16]byte{0x41}, Generation: 1}
	// Simulation-test authority supports the legacy path above; the fast lane
	// still requires its explicit mirrored application identity.
	if _, err = c.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if err = c.Close(); err != nil {
		t.Fatal(err)
	}
	f.options.SimulationTest = false
	reconnect, cancel := context.WithTimeout(ctx, 3*time.Second)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		c, err = Dial(reconnect, f.options)
		if err == nil {
			break
		}
		select {
		case <-reconnect.Done():
			t.Fatal(err)
		case <-ticker.C:
		}
	}
	t.Cleanup(func() { _ = c.Close() })
	if _, err = c.MirrorAcquire(ctx, id, HostMonotonicNS()+250_000_000); err != nil {
		t.Fatal(err)
	}
	stop := simfixture.Renew(ctx, 50*time.Millisecond, func() bool {
		_, err := c.MirrorRenew(ctx, id, HostMonotonicNS()+250_000_000)
		return err == nil
	})
	t.Cleanup(stop)
	arm(t, c, 1)
	wait(t, c, allOperationEnabled)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(ctx, id, 1, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	now = HostMonotonicNS()
	if err = c.UpdateLocalJog(ctx, session, 1, now, now+200_000_000, 1, []float64{-.02}); err != nil {
		t.Fatal(err)
	}
	o := waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if o.UpdateReason != JogReasonOutsideLimitsOutward || o.SourceSequence != 0 || o.HasInput != 0 {
		t.Fatal(o)
	}
	var next uint64
	sequence := uint64(1)
	s = wait(t, c, func(s Status) bool {
		if s.ExecutionFaultReasons != 0 || s.Armed != 1 {
			t.Fatal(s)
		}
		if s.Axes[0].PositionCounts >= 300 {
			return true
		}
		now := HostMonotonicNS()
		if now >= next {
			sequence++
			if err := c.UpdateLocalJog(ctx, session, sequence, now, now+200_000_000, 1, []float64{.02}); err != nil {
				t.Fatal(err)
			}
			next = now + 40_000_000
		}
		return false
	})
	if s.Axes[0].LogicalPosition < .01 || s.Axes[0].LogicalPosition > .1 {
		t.Fatal(s)
	}
	if err = c.EndLocalJog(ctx, session); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.Open == 0 })
}

func TestLimitHoldInwardTrajectory(t *testing.T) {
	c := limitHoldCore(t, false, false).client
	arm(t, c, 1)
	wait(t, c, allOperationEnabled)
	handle, err := c.PrepareTrajectory(context.Background(), 83, 1, []Point{{Position: []float64{0}}, {Time: time.Second, Position: []float64{.02}}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err = c.StartTrajectory(context.Background(), handle); err != nil {
		t.Fatal(err)
	}
	s := wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts >= 300 })
	if s.ExecutionFaultReasons != 0 || s.Armed != 1 || s.Axes[0].LogicalPosition < .01 {
		t.Fatal(s)
	}
}

// Tightened limits change machine identity while preserving coordinate identity.
// Matching absolute evidence at zero can restore Home under the revised limits.
func TestLimitHoldTightenedAnchorRestores(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	machine, profile := compiledCoordinateInputs(t)
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err)
	}
	store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "anchors")}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	f, stop := coordinateModelDaemon(t, compiled, digest)
	coordinateHome(t, f, 1)
	s := wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.Axes[0].LogicalValid })
	if err = store.Save(coordinateModelRecord(s)); err != nil {
		t.Fatal(err)
	}
	stop()
	limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
	limits["min"], limits["max"] = .01, .1
	revised, revisedDigest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil || revisedDigest == digest {
		t.Fatal(err, revisedDigest)
	}
	f, _ = coordinateModelDaemon(t, revised, revisedDigest)
	wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_ready" })
	coordinateModelRestore(t, f, true)
	s = wait(t, f.client, func(s Status) bool { return s.CommissioningPhase == 0 && s.HomeValidMask == 1 })
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].PositionCounts != 0 || !s.Axes[0].LogicalValid {
		t.Fatal(s)
	}
}

// A matching qualified anchor can restore an exterior pose without moving it;
// the tightened-range test above covers restoration after a limit change.
func TestLimitHoldMatchingAnchor(t *testing.T) {
	machine, profile := limitHoldInputs(t, true, true)
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err)
	}
	store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "anchors")}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	f, stop := coordinateModelDaemon(t, compiled, digest)
	coordinateHome(t, f, 1)
	s := wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.Axes[0].LogicalValid })
	if err = store.Save(coordinateModelRecord(s)); err != nil {
		t.Fatal(err)
	}
	stop()
	f, _ = coordinateModelDaemon(t, compiled, digest)
	wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_ready" })
	coordinateModelRestore(t, f, true)
	arm(t, f.client, 1)
	s = wait(t, f.client, func(s Status) bool { return s.Axes[0].Readiness == "ready" })
	if s.HomeValidMask != 1 || s.ExecutionFaultReasons != 0 || s.Axes[0].PositionCounts != 0 || s.Axes[0].LogicalTarget != 0 {
		t.Fatal(s)
	}
}

// Fault injection supplies a persistent 60-count error while the target starts
// outside the range. Recovery permission must not mask the following-error
// guard. Target lead beyond max_target_lead is a warning (JogAxis::advance):
// the same lag keeps the jog armed with no fault.
func TestLimitHoldTrackingGuards(t *testing.T) {
	for _, reason := range []uint32{16, 32} {
		name := "target_lead"
		if reason == 32 {
			name = "following_error"
		}
		t.Run(name, func(t *testing.T) {
			machine, profile := limitHoldInputs(t, false, false)
			limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
			if reason == 16 {
				limits["max_target_lead"] = .002
			} else {
				limits["following_error"] = .0002
			}
			compiled, digest, err := compileCoordinateJSON(t, machine, profile)
			if err != nil {
				t.Fatal(err)
			}
			f, _ := coordinateModelDaemon(t, compiled, digest, "--sim-feedback-offset-counts", "60")
			c := f.client
			arm(t, c, 1)
			wait(t, c, allOperationEnabled)
			if reason == 16 {
				seq, err := c.Jog(context.Background(), 1, []float64{.02}, 200*time.Millisecond)
				if r := limitHoldResult(t, c, seq, err); r.Result != ResultAccepted {
					t.Fatal(r)
				}
				// Unverified observation window: twice the 200 ms jog input.
				time.Sleep(400 * time.Millisecond)
				s := wait(t, c, func(Status) bool { return true })
				if s.ExecutionFaultReasons != 0 || s.SafetyFaultMask != 0 || s.Armed != 1 {
					t.Fatalf("target lead faulted or disarmed: %+v", s)
				}
				return
			}
			s := wait(t, c, func(s Status) bool { return s.ExecutionFaultReasons&reason != 0 })
			if s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].PositionCounts < 0 || s.Axes[0].PositionCounts > 100 {
				t.Fatalf("tracking guard: %+v", s)
			}
		})
	}
}
