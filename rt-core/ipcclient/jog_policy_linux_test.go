//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"
)

// Unverified simulation profile: 100,000 counts/m, 2,000 counts/s cap,
// 10,000 counts/s^2 acceleration, 200 ms arrest and 300 ms quick stop.
func jogPolicyCore(t *testing.T, cycle uint64, legacy bool, quickStopNS ...uint64) (*Client, GrantIdentity) {
	t.Helper()
	raw, err := os.ReadFile("../config/machines/simulation/simulation.json")
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]any
	if err := json.Unmarshal(raw, &machine); err != nil {
		t.Fatal(err)
	}
	machine["cycle_ns"] = cycle
	for _, value := range machine["axes"].([]any) {
		axis := value.(map[string]any)
		axis["type"], axis["wire_counts_per_rev"], axis["lead_m_per_rev"] = "linear", 100000, 1
		axis["jog"] = map[string]any{"arrest_ns": 200000000, "quick_stop_ns": 300000000}
		if len(quickStopNS) != 0 {
			axis["jog"].(map[string]any)["quick_stop_ns"] = quickStopNS[0]
		}
		limits := axis["limits"].(map[string]any)
		limits["min"], limits["max"], limits["max_velocity"] = -2, 2, .02
		limits["jog_acceleration"], limits["max_target_lead"], limits["following_error"] = .1, .05, .05
	}
	dir, err := os.MkdirTemp("../build", "jog-policy-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	raw, err = json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	path, err := filepath.Abs(filepath.Join(dir, "machine.json"))
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	root, err := filepath.Abs("../config")
	if err != nil {
		t.Fatal(err)
	}
	relative, err := filepath.Rel(root, path)
	if err != nil {
		t.Fatal(err)
	}
	c := startCoreWithRole(t, relative, legacy).client
	id := GrantIdentity{CapabilityID: [16]byte{0x71}, Generation: 1}
	if !legacy {
		if _, err := c.MirrorAcquire(context.Background(), id, HostMonotonicNS()+250_000_000); err != nil {
			t.Fatal(err)
		}
		stop := simfixture.Renew(context.Background(), 50*time.Millisecond, func() bool {
			_, err := c.MirrorRenew(context.Background(), id, HostMonotonicNS()+250_000_000)
			return err == nil
		})
		t.Cleanup(stop)
	}
	arm(t, c, 3)
	after := HostMonotonicNS()
	wait(t, c, func(s Status) bool { return s.TimeNS >= after && allOperationEnabled(s) })
	return c, id
}

// Consume the generated status decoder's last sample, so assertions use emitted
// motor counts and sample timestamps rather than inferred feedback velocities.
func jogPolicyDebug(c *Client) StatusJogDebugV1 {
	c.stateMu.Lock()
	defer c.stateMu.Unlock()
	return c.jogDebug
}

func waitJogPolicyDebug(t *testing.T, c *Client, predicate func(StatusJogDebugV1) bool) StatusJogDebugV1 {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		value := jogPolicyDebug(c)
		if value.SampleTimeNs != 0 && predicate(value) {
			return value
		}
		select {
		case <-ctx.Done():
			t.Fatalf("jog debug predicate: %+v, client error %v", value, c.Err())
		case <-ticker.C:
		}
	}
}

// Keep valid input current until the emitted velocity reaches the absolute cap;
// grant renewal is independent, so later input expiry cannot be grant expiry.
func jogPolicyMoving(t *testing.T, c *Client, id GrantIdentity, vector []float64) (LocalJogSession, StatusJogDebugV1) {
	t.Helper()
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), id, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	var sequence uint64
	var next uint64
	debug := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		if d.OutputTargetVelocityCountsPerS[0] == 2000 {
			return true
		}
		now := HostMonotonicNS()
		if now >= next {
			sequence++
			if err := c.UpdateLocalJog(context.Background(), session, sequence, now, now+200_000_000, 3, vector); err != nil {
				t.Fatal(err)
			}
			next = now + 40_000_000
		}
		return false
	})
	waitJog(t, c, func(o JogObservation) bool { return o.SourceSequence == sequence && o.HasInput == 1 })
	return session, debug
}

func TestEndJogRampsAtConfiguredAcceleration(t *testing.T) {
	for _, cycle := range []uint64{250_000, 1_000_000, 10_000_000} {
		t.Run(fmt.Sprint(cycle), func(t *testing.T) {
			c, id := jogPolicyCore(t, cycle, false)
			session, before := jogPolicyMoving(t, c, id, []float64{.02, .01})
			started := HostMonotonicNS()
			if err := c.EndLocalJog(context.Background(), session); err != nil {
				t.Fatal(err)
			}
			waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonRamping })
			previous := before
			samples := 0
			zero := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
				if d.SampleTimeNs <= previous.SampleTimeNs {
					return false
				}
				v, old := d.OutputTargetVelocityCountsPerS[0], previous.OutputTargetVelocityCountsPerS[0]
				// Status samples are 20 ms apart. One count covers the two rounded
				// endpoint velocities; one cycle covers the dispatch phase boundary.
				bound := 10000*float64(d.SampleTimeNs-previous.SampleTimeNs+cycle)/1e9 + 1
				if v < 0 || v > old || float64(old-v) > bound {
					t.Fatalf("acceleration: %d -> %d over %d ns, bound %.3f counts/s", old, v, d.SampleTimeNs-previous.SampleTimeNs, bound)
				}
				previous = d
				samples++
				return v == 0
			})
			if samples < 3 {
				t.Fatalf("only %d ramp velocity observations", samples)
			}
			if zero.SampleTimeNs-started > 300_000_000+cycle {
				t.Fatalf("halt exceeded 300000000 ns: %d", zero.SampleTimeNs-started)
			}
			closed := waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonClosed })
			if closed.NowHostNs-started > 300_000_000+cycle {
				t.Fatal("late close", closed.NowHostNs-started)
			}
			waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
				if d.SampleTimeNs <= zero.SampleTimeNs {
					return false
				}
				if d.OutputTargetVelocityCountsPerS[0] != 0 || d.OutputTargetCounts[0] != zero.OutputTargetCounts[0] {
					t.Fatalf("hold changed: %+v -> %+v", zero, d)
				}
				return d.SampleTimeNs >= zero.SampleTimeNs+100_000_000
			})
			state := wait(t, c, func(s Status) bool { return s.TimeNS >= zero.SampleTimeNs+100_000_000 })
			if state.Armed != 1 || state.ExecutionFaultReasons != 0 {
				t.Fatalf("qualified halt lost enabled hold: %+v", state)
			}
		})
	}
}

func TestExpiredJogRampsThenCloses(t *testing.T) {
	c, id := jogPolicyCore(t, 1_000_000, false)
	session, _ := jogPolicyMoving(t, c, id, []float64{.02, .01})
	input := waitJog(t, c, func(o JogObservation) bool { return o.HasInput == 1 })
	ramping := waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonRamping })
	if ramping.NowHostNs < input.InputDeadlineHostNs {
		t.Fatal("halt preceded input expiry")
	}
	now := HostMonotonicNS()
	if err := c.UpdateLocalJog(context.Background(), session, input.SourceSequence+1, now, now+200_000_000, 3, []float64{.02, .01}); err != nil {
		t.Fatal(err)
	}
	rejected := waitJog(t, c, func(o JogObservation) bool { return o.UpdateReason == JogReasonClosed })
	if rejected.SourceSequence != input.SourceSequence || rejected.Open != 0 {
		t.Fatal("late update changed generation", rejected)
	}
	closed := waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonExpired })
	if closed.NowHostNs-input.InputDeadlineHostNs > 301_000_000 {
		t.Fatal("expired input did not close within bound", closed)
	}
	zero := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
		return d.SampleTimeNs >= closed.NowHostNs && d.OutputTargetVelocityCountsPerS[0] == 0
	})
	state := wait(t, c, func(s Status) bool { return s.TimeNS >= zero.SampleTimeNs })
	if state.Armed != 1 || state.ExecutionFaultReasons != 0 {
		t.Fatal("expiry lost healthy hold", state)
	}
}

func TestStopDuringRampInhibitsImmediately(t *testing.T) {
	c, id := jogPolicyCore(t, 1_000_000, false)
	session, _ := jogPolicyMoving(t, c, id, []float64{.02, .01})
	if err := c.EndLocalJog(context.Background(), session); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonRamping })
	if _, err := c.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	stopped := waitJog(t, c, func(o JogObservation) bool {
		return o.NativeGeneration > session.Identity.NativeGeneration && o.Open == 0
	})
	debug := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool { return d.SampleTimeNs >= stopped.NowHostNs })
	if debug.OutputTargetVelocityCountsPerS[0] != 0 || debug.OutputTargetVelocityCountsPerS[1] != 0 {
		t.Fatal("nonzero counts/s after inhibited cycle", debug)
	}
	state := wait(t, c, func(s Status) bool { return s.TimeNS >= stopped.NowHostNs })
	if state.Armed != 0 || state.AxisEnableMask != 0 {
		t.Fatal("Stop retained enable", state)
	}
}

func TestOverspeedJogIsLimitedWithFeedback(t *testing.T) {
	c, id := jogPolicyCore(t, 1_000_000, false)
	session, debug := jogPolicyMoving(t, c, id, []float64{.08, .04})
	observation := waitJog(t, c, func(o JogObservation) bool { return o.UpdateReason == JogReasonLimited })
	if observation.VelocityScalePpm != 250_000 || observation.Open != 1 || observation.HasInput != 1 {
		t.Fatal("missing limiting feedback", observation)
	}
	if debug.OutputTargetVelocityCountsPerS[0] != 2000 || debug.OutputTargetVelocityCountsPerS[1] != -1000 {
		t.Fatal("vector did not scale uniformly to cap", debug)
	}
	if debug.FeedbackPosCounts[0] <= 0 || debug.FeedbackPosCounts[1] >= 0 {
		t.Fatal("limited vector did not move axes", debug)
	}
	if err := c.EndLocalJog(context.Background(), session); err != nil {
		t.Fatal(err)
	}
}

func TestLegacyJogArrestScalesWithCyclePeriod(t *testing.T) {
	for _, cycle := range []uint64{250_000, 10_000_000} {
		t.Run(fmt.Sprint(cycle), func(t *testing.T) {
			c, _ := jogPolicyCore(t, cycle, true)
			sequence, err := c.Jog(context.Background(), 3, []float64{.01, .005}, 100*time.Millisecond)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := c.WaitResult(context.Background(), sequence); err != nil {
				t.Fatal(err)
			}
			arrest := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool { return d.LastStopCmdSeq == sequence && d.StopArrestMask == 3 })
			if arrest.LastStopTimeNs == 0 {
				t.Fatal("arrest missing origin")
			}
			end := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool {
				return d.SampleTimeNs > arrest.SampleTimeNs && d.LastStopCmdSeq == sequence && d.StopArrestMask == 0
			})
			elapsed := end.SampleTimeNs - arrest.LastStopTimeNs
			// Measured publication contract: jog-debug samples occur every 20 ms.
			// The every-cycle C++ check separately asserts the one-cycle boundary.
			if elapsed < 200_000_000-cycle || elapsed > 230_000_000+cycle {
				t.Fatalf("legacy arrest took %d ns at cycle %d", elapsed, cycle)
			}
		})
	}
}

// EndJog uses the 200000000 ns acceleration bound, independent of the legacy
// 100000000 ns quick-stop window; a healthy arrest must retain enabled hold.
func TestJogHaltUsesAccelerationBoundWithoutQuickStopWindow(t *testing.T) {
	c, id := jogPolicyCore(t, 1_000_000, false, 100_000_000)
	session, _ := jogPolicyMoving(t, c, id, []float64{.02, .01})
	started := HostMonotonicNS()
	if err := c.EndLocalJog(context.Background(), session); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonRamping })
	closed := waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonClosed })
	// Unverified observation ceiling: 301000000 ns; deterministic C++ tests
	// enforce the physical 200000000 ns plus one cycle on every cycle period.
	if closed.NowHostNs-started <= 100_000_000 || closed.NowHostNs-started > 301_000_000 {
		t.Fatal("halt did not use its acceleration bound", closed.NowHostNs-started)
	}
	debug := waitJogPolicyDebug(t, c, func(d StatusJogDebugV1) bool { return d.SampleTimeNs >= closed.NowHostNs })
	if debug.OutputTargetVelocityCountsPerS[0] != 0 || debug.OutputTargetVelocityCountsPerS[1] != 0 {
		t.Fatal("completed halt retained velocity", debug)
	}
	state := wait(t, c, func(s Status) bool { return s.TimeNS >= closed.NowHostNs })
	if state.Armed != 1 || state.AxisEnableMask != 3 || state.ExecutionFaultReasons != 0 || state.SafetyFaultMask != 0 {
		t.Fatal("qualified halt must retain enabled hold", state)
	}
}

func TestJogDebugSignedCountCodec(t *testing.T) {
	// Wire measurements from StatusJogDebugV1: 360 bytes, velocity offset 296.
	var bytes [360]byte
	bytes[296], bytes[297], bytes[298], bytes[299] = 0xff, 0xff, 0xff, 0xff
	bytes[303] = 0x80
	debug, err := DecodeStatusJogDebugV1Frame(bytes[:])
	if err != nil || debug.OutputTargetVelocityCountsPerS[0] != -1 || debug.OutputTargetVelocityCountsPerS[1] != -2147483648 {
		t.Fatal("signed motor counts did not decode", debug, err)
	}
	if EncodeStatusJogDebugV1(debug) != bytes {
		t.Fatal("signed count bytes changed")
	}
	if _, err := DecodeStatusJogDebugV1Frame(bytes[:359]); err == nil {
		t.Fatal("truncated jog status accepted")
	}
}
