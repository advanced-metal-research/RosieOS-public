//go:build linux

package denseexec

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type resetClient struct {
	*fakeRTCore
	changedHome  bool
	stale        bool
	foreign      bool
	commandError error
	resetEntered chan struct{}
	// persists models the adapter and SDK on a submitted reset native refuses
	// (reset_linux.go, client.go remember): the session is retired, the SDK
	// forgets its fence, the fault stays, and a Stop on the retired session
	// is refused control_session_stale.
	persists bool
}

func (c *resetClient) Stop(ctx context.Context) (control.Grant, error) {
	c.mu.Lock()
	retired := c.persists && c.grant.Session == ""
	if retired {
		c.record("stop")
	}
	c.mu.Unlock()
	if retired {
		return control.Grant{}, &control.Rejected{Reason: "control_session_stale"}
	}
	return c.fakeRTCore.Stop(ctx)
}

func (c *resetClient) Status(ctx context.Context) (control.Status, error) {
	s, err := c.fakeRTCore.Status(ctx)
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.stale {
		c.status.TimeNS += 1000000
		c.status.Motion.TimeNS = c.status.TimeNS
	}
	s = c.status
	s.Grant = &ipcclient.GrantObservation{NowHostNs: s.TimeNS}
	if c.grant.Session != "" || c.foreign {
		s.Grant.GrantActive = 1
		s.Core.LeaseValid = 1
	} else {
		s.Core.LeaseValid = 0
	}
	return s, err
}
func (c *resetClient) Command(ctx context.Context, q *control.Request) (control.Response, error) {
	if q.Operation == "reset_fault" && c.resetEntered != nil {
		close(c.resetEntered)
		<-ctx.Done()
		return control.Response{}, ctx.Err()
	}
	if q.Operation == "reset_fault" && c.commandError != nil {
		return control.Response{}, c.commandError
	}
	if q.Operation == "reset_fault" && c.persists {
		c.mu.Lock()
		c.record("reset_fault")
		c.grant = control.Grant{}
		c.mu.Unlock()
		return control.Response{Operation: "reset_fault", Sequence: 1}, &control.Rejected{Reason: "fault_persists"}
	}
	result, err := c.fakeRTCore.Command(ctx, q)
	if q.Operation == "reset_fault" && err == nil {
		c.mu.Lock()
		c.grant = control.Grant{}
		if c.changedHome {
			c.status.Core.HomeEpoch++
		}
		c.mu.Unlock()
	}
	return result, err
}
func resetFixture(t *testing.T) (*RTCoreController, *resetClient) {
	r, f, _ := rtFake(t)
	pin := strings.Repeat("d", 64)
	r.cfg.Binding.ConfigurationSHA256 = pin
	f.mu.Lock()
	f.description.ConfigurationSHA256 = pin
	f.status.Core.ConfigurationSHA256 = pin
	f.status.Core.ConfigurationEpoch = 1
	f.status.Core.HomeEpoch = 7
	f.status.TimeNS = 1000000000
	f.status.Motion.TimeNS = f.status.TimeNS
	f.status.Motion.Done = true
	f.status.Core.SafetyFaultMask = 32
	f.status.Core.ExecutionFaultReasons = 512
	f.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Name: "joint_move_completion", Recovery: "reset_clears"}}}
	f.mu.Unlock()
	c := &resetClient{fakeRTCore: f}
	r.c = c
	return r, c
}
func TestStandaloneResetClearsFaultWithoutMotionAndReleases(t *testing.T) {
	r, c := resetFixture(t)
	result, err := r.ResetFault(context.Background())
	if err != nil || !result.Accepted {
		t.Fatal(result, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status.Core.HomeEpoch != 7 || c.status.Core.HomeValidMask != 511 || c.status.Core.Armed != 0 || c.status.Core.SafetyFaultMask != 0 || c.grant.Session != "" {
		t.Fatal("reset changed Home or retained authority")
	}
	resetCount := 0
	for _, call := range c.calls {
		if call == "reset_fault" {
			resetCount++
		}
		if call == "arm" || call == "enable" || call == "home" || call == "prepare_program" || call == "start_program" {
			t.Fatal("motion operation", c.calls)
		}
	}
	if resetCount != 1 {
		t.Fatal(c.calls)
	}
}

// AMR-720: native keeps trusted Home through a target-lead trip, so Clear
// faults resets it like any reset_after_condition_clears fault, without Home.
func TestStandaloneResetClearsTargetLeadWithoutHome(t *testing.T) {
	r, c := resetFixture(t)
	c.mu.Lock()
	c.status.Core.SafetyFaultMask = 4
	c.status.Core.ExecutionFaultReasons = 16
	c.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 4, Name: "target_lead", AxisMask: 4, Recovery: "reset_after_condition_clears"}}}
	c.mu.Unlock()
	result, err := r.ResetFault(context.Background())
	if err != nil || !result.Accepted {
		t.Fatal(result, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status.Core.HomeEpoch != 7 || c.status.Core.HomeValidMask != 511 || c.status.Core.ExecutionFaultReasons != 0 {
		t.Fatal("target-lead reset changed Home or retained the fault")
	}
	if rtCallIndex(c.calls, "reset_fault") < 0 || rtCallIndex(c.calls, "home") >= 0 {
		t.Fatal(c.calls)
	}
}

func recoveredReferenceFixture(t *testing.T) (*RTCoreController, *resetClient) {
	r, c := resetFixture(t)
	c.mu.Lock()
	c.status.Core.SafetyFaultMask = 4
	c.status.Core.ExecutionFaultReasons = 64
	c.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{
		Bit: 6, Name: "coordinate_reference", AxisMask: 4, Recovery: "rehome_required", ResetPreservesHome: true,
	}}}
	c.mu.Unlock()
	return r, c
}

func TestExplicitResetPreservesIndependentlyRecoveredHomeWithoutModel(t *testing.T) {
	r, c := recoveredReferenceFixture(t)
	// This fixture has no robot/model resources or planning resolver. Reset
	// requires only the native machine binding and its current recovery proof.
	result, err := r.ResetFault(context.Background())
	if err != nil || !result.Accepted {
		t.Fatal(result, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.status.Core.HomeEpoch != 7 || c.status.Core.HomeValidMask != 511 || c.status.Core.Armed != 0 || c.grant.Session != "" || c.resourceReads != 0 {
		t.Fatal("reset changed Home, acquired model resources, armed or retained authority")
	}
	for _, op := range []string{"enable", "arm", "home", "prepare_program", "start_program"} {
		if rtCallIndex(c.calls, op) >= 0 {
			t.Fatal(c.calls)
		}
	}
	if rtCallIndex(c.calls, "reset_fault") < 0 {
		t.Fatal(c.calls)
	}
}

func TestRecoveredReferenceStillRequiresExplicitResetBeforeArm(t *testing.T) {
	for _, arm := range []bool{false, true} {
		t.Run(map[bool]string{false: "automatic_reset", true: "arm"}[arm], func(t *testing.T) {
			r, c := recoveredReferenceFixture(t)
			var err error
			if arm {
				_, err = r.Arm(context.Background(), true)
			} else {
				_, err = r.resetFault(context.Background(), false)
			}
			if err == nil {
				t.Fatal("automatic reference reset admitted")
			}
			if arm && !strings.Contains(err.Error(), "use Clear fault") {
				t.Fatal(err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			for _, op := range []string{"reset_fault", "enable", "arm", "home"} {
				if rtCallIndex(c.calls, op) >= 0 {
					t.Fatal(c.calls)
				}
			}
		})
	}
}

func TestExplicitReferenceResetFailsClosedWithoutCompleteNativeProof(t *testing.T) {
	for _, name := range []string{"old_native", "home_missing", "rehome", "axis_unknown", "wrong_bit", "unclassified_fault", "restart", "unknown_recovery"} {
		t.Run(name, func(t *testing.T) {
			r, c := recoveredReferenceFixture(t)
			c.mu.Lock()
			s := &c.status
			switch name {
			case "old_native":
				s.Core.Recovery.Faults[0].ResetPreservesHome = false
			case "home_missing":
				s.Core.HomeValidMask &^= 4
			case "rehome":
				s.Core.Recovery.Faults[0].RehomeAxisMask = 4
			case "axis_unknown":
				s.Core.Recovery.Faults[0].AxisMask = 1 << 15
			case "wrong_bit":
				s.Core.Recovery.Faults[0].Bit = 5
			case "unclassified_fault":
				s.Core.ExecutionFaultReasons |= 1 << 31
			case "restart":
				s.Core.Recovery.Faults = append(s.Core.Recovery.Faults, ipcclient.FaultRecovery{Bit: 11, Name: "pdo_mapping_mismatch", Recovery: "restart_required"})
				s.Core.ExecutionFaultReasons |= 1 << 11
			case "unknown_recovery":
				s.Core.Recovery.Faults[0].Recovery = "unknown"
			}
			c.mu.Unlock()
			if result, err := r.ResetFault(context.Background()); err == nil || result.Accepted {
				t.Fatal(result, err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			if rtCallIndex(c.calls, "reset_fault") >= 0 {
				t.Fatal(c.calls)
			}
		})
	}
}

type changingReferenceResetClient struct{ *resetClient }

func (c *changingReferenceResetClient) Acquire(ctx context.Context, controller string, binding control.Binding) (control.Grant, error) {
	g, err := c.resetClient.Acquire(ctx, controller, binding)
	c.mu.Lock()
	c.status.Core.Recovery.Faults[0].ResetPreservesHome = false
	c.mu.Unlock()
	return g, err
}

func TestExplicitReferenceResetRechecksNativeProofAfterAcquire(t *testing.T) {
	r, c := recoveredReferenceFixture(t)
	r.c = &changingReferenceResetClient{c}
	if result, err := r.ResetFault(context.Background()); err == nil || result.Accepted {
		t.Fatal(result, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if rtCallIndex(c.calls, "reset_fault") >= 0 || c.grant.Session != "" {
		t.Fatal(c.calls, c.grant)
	}
}
func TestStandaloneResetRefusesUnsafeStateAndPolicy(t *testing.T) {
	for _, name := range []string{"armed", "enabled", "home", "jog", "moving", "restart", "rehome", "stale", "identity"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetFixture(t)
			c.mu.Lock()
			switch name {
			case "armed":
				c.status.Core.Armed = 1
			case "enabled":
				c.status.Core.AxisEnableMask = 1
			case "home":
				c.status.Core.CommissioningPhase = 1
			case "jog":
				c.status.Jog = &ipcclient.JogObservation{Open: 1}
			case "moving":
				c.status.Motion.Mode = 2
				c.status.Motion.Done = false
			case "restart":
				c.status.Core.Recovery.Faults[0].Recovery = "restart_required"
			case "rehome":
				c.status.Core.Recovery.Faults[0].Recovery = "rehome_required"
			case "stale":
				c.stale = true
			case "identity":
				c.status.Core.ConfigurationSHA256 = "wrong"
			}
			c.mu.Unlock()
			result, err := r.ResetFault(context.Background())
			if err == nil || result.Accepted {
				t.Fatal("unsafe reset accepted", result)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			if rtCallIndex(c.calls, "reset_fault") >= 0 {
				t.Fatal(c.calls)
			}
		})
	}
}
func TestStandaloneResetRefusalOrChangedHomeNeverClaimsSuccess(t *testing.T) {
	for _, name := range []string{"refused", "home_changed", "transport", "unsent"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetFixture(t)
			if name == "refused" {
				c.refuse = "reset_fault"
			} else if name == "transport" {
				c.commandError = errors.New("transport failed")
			} else if name == "unsent" {
				c.commandError = control.ErrConnectionUnsent
			} else {
				c.changedHome = true
			}
			result, err := r.ResetFault(context.Background())
			if err == nil || result.Accepted || result.Error == "" {
				t.Fatal(result, err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			if c.grant.Session != "" {
				t.Fatal("cleanup retained grant")
			}
			if rtCallIndex(c.calls, "enable") >= 0 || rtCallIndex(c.calls, "arm") >= 0 || rtCallIndex(c.calls, "home") >= 0 {
				t.Fatal(c.calls)
			}
		})
	}
}
func TestResetObservationFreshnessUsesSameHostProof(t *testing.T) {
	_, c := resetFixture(t)
	s, _ := c.Status(context.Background())
	pin := s.Core.ConfigurationSHA256
	if err := resetStatusAdmission(s, pin, time.Millisecond); err != nil {
		t.Fatal(err)
	}
	s.Grant.NowHostNs = s.TimeNS + 151000000
	if resetStatusAdmission(s, pin, time.Millisecond) == nil {
		t.Fatal("stale native observation admitted")
	}
	s.Grant = nil
	if resetStatusAdmission(s, pin, time.Millisecond) == nil {
		t.Fatal("missing clock evidence admitted")
	}
}

// Only the requested reads run in these cases. A background status poll must
// not consume the stale/fresh sequence intended for the reset operation.
type resetObservationClient struct {
	*resetClient
	reads     int
	observe   func(*control.Status, int)
	afterRead func(context.Context, *control.Status, int) error
}

func (c *resetObservationClient) Status(ctx context.Context) (control.Status, error) {
	s, err := c.resetClient.Status(ctx)
	c.reads++
	if c.observe != nil {
		c.observe(&s, c.reads)
	}
	if c.afterRead != nil {
		if readErr := c.afterRead(ctx, &s, c.reads); readErr != nil {
			return s, readErr
		}
	}
	return s, err
}

func resetObservationFixture(t *testing.T) (*RTCoreController, *resetObservationClient) {
	t.Helper()
	r, c := resetFixture(t)
	r.cancel()
	<-r.done
	observed := &resetObservationClient{resetClient: c}
	r.c = observed
	return r, observed
}

// The SDK's observation read retains current execution/safety evidence while
// avoiding historical handles. Recovery must use it on every retry instead of
// returning to the expensive full-status decoder.
type resetProjectedObservationClient struct {
	*resetObservationClient
	fullReads int
}

func (c *resetProjectedObservationClient) Status(context.Context) (control.Status, error) {
	c.fullReads++
	return control.Status{}, errors.New("reset used full status instead of observation")
}

func (c *resetProjectedObservationClient) StatusObservation(ctx context.Context) (control.Status, error) {
	return c.resetObservationClient.Status(ctx)
}

func TestResetReadUsesOptionalObservationForEveryRetry(t *testing.T) {
	for _, oldReads := range []int{0, 2} {
		name := "fresh"
		if oldReads != 0 {
			name = "old_then_fresh"
		}
		t.Run(name, func(t *testing.T) {
			r, observed := resetObservationFixture(t)
			projected := &resetProjectedObservationClient{resetObservationClient: observed}
			r.c = projected
			observed.observe = func(s *control.Status, read int) {
				if read <= oldReads {
					s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
				}
			}
			status, err := r.resetRead(context.Background(), nil)
			if err != nil || projected.fullReads != 0 || observed.reads != oldReads+1 {
				t.Fatalf("observation path not retained: full=%d observation=%d err=%v", projected.fullReads, observed.reads, err)
			}
			if status.Core.HomeEpoch != 7 || status.Core.SafetyFaultMask != 32 || status.Core.ExecutionFaultReasons != 512 {
				t.Fatalf("observation lost current Home/fault evidence: %+v", status.Core)
			}
		})
	}
}

func TestStandaloneResetRetriesOldPublicationWithoutMotion(t *testing.T) {
	r, c := resetObservationFixture(t)
	c.observe = func(s *control.Status, read int) {
		if read <= 2 {
			// The Nov 1 the 1430 cell failure: status is advancing, but a completed
			// read is already outside the unchanged 150 ms freshness limit.
			s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
		}
	}
	result, err := r.ResetFault(context.Background())
	if err != nil || !result.Accepted {
		t.Fatalf("fresh observation after old publications did not recover: %+v %v", result, err)
	}
	if c.reads < 3 {
		t.Fatal("reset accepted an old publication")
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	resets := 0
	for _, op := range c.calls {
		if op == "reset_fault" {
			resets++
		}
		if op == "arm" || op == "enable" || op == "home" || op == "prepare_program" || op == "start_program" {
			t.Fatalf("recovery issued motion operation: %v", c.calls)
		}
	}
	if resets != 1 || c.grant.Session != "" || c.status.Core.HomeEpoch != 7 || c.status.Core.HomeValidMask != 511 || c.status.Core.SafetyFaultMask != 0 {
		t.Fatalf("recovery did not preserve Home, clear once and retire authority: %v", c.calls)
	}
}

func TestResetReadPersistentOldPublicationStaysBounded(t *testing.T) {
	r, c := resetObservationFixture(t)
	c.observe = func(s *control.Status, _ int) {
		s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
	}
	started := time.Now()
	_, err := r.resetRead(context.Background(), nil)
	elapsed := time.Since(started)
	if err == nil || !strings.Contains(err.Error(), "rt_core_status_stale") {
		t.Fatalf("persistent stale status accepted: %v", err)
	}
	if c.reads < 2 || elapsed < 450*time.Millisecond || elapsed > 1500*time.Millisecond {
		t.Fatalf("fresh-status retry did not use a bounded 500 ms window: reads=%d elapsed=%s", c.reads, elapsed)
	}
	for _, op := range c.calls {
		if op == "acquire" || op == "reset_fault" {
			t.Fatalf("persistent stale status acquired authority: %v", c.calls)
		}
	}
}

func TestResetReadRetryDoesNotMaskChangedIdentityOrInhibitedState(t *testing.T) {
	for _, name := range []string{"configuration", "home", "daemon", "armed", "enabled", "moving", "jog"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetObservationFixture(t)
			baseline, err := c.resetClient.Status(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			want := "rt_core_identity_mismatch"
			if name == "armed" || name == "enabled" || name == "moving" || name == "jog" {
				want = "reset_requires_inhibited"
			}
			c.observe = func(s *control.Status, read int) {
				s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
				if read == 1 {
					return
				}
				switch name {
				case "configuration":
					s.Core.ConfigurationSHA256 = strings.Repeat("e", 64)
				case "home":
					s.Core.HomeEpoch++
				case "daemon":
					s.DaemonIncarnation = "successor"
				case "armed":
					s.Core.Armed = 1
				case "enabled":
					s.Core.AxisEnableMask = 511
				case "moving":
					s.Motion.Mode, s.Motion.Done = 2, false
				case "jog":
					s.Jog = &ipcclient.JogObservation{Open: 1}
				}
			}
			_, err = r.resetRead(context.Background(), &baseline)
			if err == nil || !strings.Contains(err.Error(), want) || c.reads != 2 {
				t.Fatalf("staleness masked %s: reads=%d err=%v", name, c.reads, err)
			}
		})
	}
}

func TestResetReadOldPublicationRetryHonorsCancellation(t *testing.T) {
	r, c := resetObservationFixture(t)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	c.observe = func(s *control.Status, _ int) {
		s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
		cancel()
	}
	started := time.Now()
	_, err := r.resetRead(ctx, nil)
	if err == nil || ctx.Err() != context.Canceled || c.reads != 1 || time.Since(started) > 250*time.Millisecond {
		t.Fatalf("cancelled recovery continued reading: reads=%d elapsed=%s err=%v", c.reads, time.Since(started), err)
	}
}

func TestResetReadDoesNotRetryMissingClockOrFuturePublication(t *testing.T) {
	for _, name := range []string{"missing_clock", "future_publication"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetObservationFixture(t)
			c.observe = func(s *control.Status, _ int) {
				if name == "missing_clock" {
					s.Grant = nil
				} else {
					s.Grant.NowHostNs = s.TimeNS - uint64(101*time.Millisecond)
				}
			}
			_, err := r.resetRead(context.Background(), nil)
			if err == nil || c.reads != 1 {
				t.Fatalf("invalid clock proof retried or admitted: reads=%d err=%v", c.reads, err)
			}
		})
	}
}

func TestResetReadOldStatusCannotMaskFutureMotionPublication(t *testing.T) {
	r, c := resetObservationFixture(t)
	c.observe = func(s *control.Status, _ int) {
		// The old status timestamp is retryable, but neither timestamp may
		// hide invalid clock evidence in the other publication.
		s.Grant.NowHostNs = s.TimeNS + uint64(151*time.Millisecond)
		s.Motion.TimeNS = s.Grant.NowHostNs + uint64(101*time.Millisecond)
	}
	_, err := r.resetRead(context.Background(), nil)
	if err == nil || !strings.Contains(err.Error(), "reset future publication") || c.reads != 1 {
		t.Fatalf("old status hid future motion evidence: reads=%d err=%v", c.reads, err)
	}
}

func TestStandaloneResetWaitsForFaultClearAndGrantRetirement(t *testing.T) {
	for _, name := range []string{"fault", "grant", "both"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetObservationFixture(t)
			postResetReads := 0
			c.observe = func(s *control.Status, _ int) {
				c.mu.Lock()
				resetSent := rtCallIndex(c.calls, "reset_fault") >= 0
				c.mu.Unlock()
				if !resetSent {
					return
				}
				postResetReads++
				if postResetReads > 2 {
					return
				}
				// Native acknowledgement and the independently published
				// fault/authority observations need not become visible together.
				if name == "fault" || name == "both" {
					s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons = 32, 512
				}
				if name == "grant" || name == "both" {
					s.Core.LeaseValid, s.Grant.GrantActive = 1, 1
				}
			}
			result, err := r.ResetFault(context.Background())
			if err != nil || !result.Accepted || postResetReads < 3 {
				t.Fatalf("reset acknowledgement was mistaken for final %s state: accepted=%v reads=%d err=%v", name, result.Accepted, postResetReads, err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			resets := 0
			for _, op := range c.calls {
				if op == "reset_fault" {
					resets++
				}
				if op == "arm" || op == "enable" || op == "home" {
					t.Fatalf("reset issued %s: %v", op, c.calls)
				}
			}
			if resets != 1 || c.grant.Session != "" || c.status.Core.HomeEpoch != 7 || c.status.Core.HomeValidMask != 511 {
				t.Fatalf("reset retried the command, changed Home or retained authority: %v", c.calls)
			}
		})
	}
}

func TestResetAfterNeverRetiredGrantStaysBounded(t *testing.T) {
	r, c := resetObservationFixture(t)
	before, err := c.resetClient.Status(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	c.observe = func(s *control.Status, _ int) {
		s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons = 0, 0
		s.Core.LeaseValid, s.Grant.GrantActive = 1, 1
	}
	started := time.Now()
	_, err = r.resetAfter(context.Background(), before)
	elapsed := time.Since(started)
	if err == nil || !strings.Contains(err.Error(), "reset_fault_cleanup_unverified") {
		t.Fatalf("unretired authority accepted: %v", err)
	}
	if c.reads < 2 || elapsed < 450*time.Millisecond || elapsed > 1500*time.Millisecond {
		t.Fatalf("authority observation did not use a bounded 500 ms window: reads=%d elapsed=%s", c.reads, elapsed)
	}
}

func TestResetAfterPendingGrantPreservesDeadlineButNotCancellation(t *testing.T) {
	for _, name := range []string{"observation_deadline", "parent_cancelled", "transport_error"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetObservationFixture(t)
			before, err := c.resetClient.Status(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			transportErr := errors.New("status transport failed")
			c.afterRead = func(readCtx context.Context, s *control.Status, read int) error {
				if read == 1 {
					s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons = 0, 0
					s.Core.LeaseValid, s.Grant.GrantActive = 1, 1
					return nil
				}
				if name == "transport_error" {
					return transportErr
				}
				if name == "parent_cancelled" {
					cancel()
				}
				// The last observed grant is still active when a later HTTP
				// status read blocks until its observation window expires.
				<-readCtx.Done()
				return readCtx.Err()
			}
			started := time.Now()
			_, err = r.resetAfter(ctx, before)
			elapsed := time.Since(started)
			if c.reads != 2 {
				t.Fatalf("unexpected number of status reads: %d (%v)", c.reads, err)
			}
			switch name {
			case "observation_deadline":
				if err == nil || !strings.Contains(err.Error(), "reset_fault_cleanup_unverified") || elapsed < 450*time.Millisecond || elapsed > 1500*time.Millisecond {
					t.Fatalf("deadline lost pending authority evidence: elapsed=%s err=%v", elapsed, err)
				}
			case "parent_cancelled":
				if !errors.Is(err, context.Canceled) || elapsed > 250*time.Millisecond {
					t.Fatalf("pending authority masked caller cancellation: elapsed=%s err=%v", elapsed, err)
				}
			case "transport_error":
				if !errors.Is(err, transportErr) {
					t.Fatalf("pending authority masked transport error: %v", err)
				}
			}
		})
	}
}

func TestResetAfterChangedHomeOrIdentityDoesNotWaitForRetirement(t *testing.T) {
	for _, name := range []string{"home", "configuration", "adapter", "daemon"} {
		t.Run(name, func(t *testing.T) {
			r, c := resetObservationFixture(t)
			before, err := c.resetClient.Status(context.Background())
			if err != nil {
				t.Fatal(err)
			}
			c.observe = func(s *control.Status, _ int) {
				s.Core.LeaseValid, s.Grant.GrantActive = 1, 1
				switch name {
				case "home":
					s.Core.HomeEpoch++
				case "configuration":
					s.Core.ConfigurationEpoch++
				case "adapter":
					s.AdapterIncarnation = "successor"
				case "daemon":
					s.DaemonIncarnation = "successor"
				}
			}
			_, err = r.resetAfter(context.Background(), before)
			if err == nil || !strings.Contains(err.Error(), "rt_core_identity_mismatch") || c.reads != 1 {
				t.Fatalf("changed %s was masked by pending retirement: reads=%d err=%v", name, c.reads, err)
			}
		})
	}
}

func TestStandaloneResetCleanNoopAndUnhomedReset(t *testing.T) {
	for _, clean := range []bool{true, false} {
		t.Run(map[bool]string{true: "clean", false: "unhomed"}[clean], func(t *testing.T) {
			r, c := resetFixture(t)
			c.status.Core.HomeEpoch = 0
			c.status.Core.HomeValidMask = 0
			if clean {
				c.status.Core.SafetyFaultMask = 0
				c.status.Core.ExecutionFaultReasons = 0
				c.status.Core.Recovery = nil
			}
			result, err := r.ResetFault(context.Background())
			if err != nil || !result.Accepted {
				t.Fatal(result, err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			if c.status.Core.HomeEpoch != 0 || c.status.Core.HomeValidMask != 0 || c.grant.Session != "" {
				t.Fatal("unhomed state or authority changed")
			}
			if clean && (rtCallIndex(c.calls, "acquire") >= 0 || rtCallIndex(c.calls, "reset_fault") >= 0) {
				t.Fatal(c.calls)
			}
		})
	}
}
func TestStandaloneResetForeignOrBusyDoesNotStopOwner(t *testing.T) {
	for _, foreign := range []bool{true, false} {
		t.Run(map[bool]string{true: "foreign", false: "busy"}[foreign], func(t *testing.T) {
			r, c := resetFixture(t)
			c.foreign = foreign
			if !foreign {
				c.acquireErr = &control.Rejected{Reason: "busy"}
			}
			result, err := r.ResetFault(context.Background())
			if err == nil || result.Accepted {
				t.Fatal(result, err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			for _, op := range []string{"reset_fault", "stop", "release", "enable", "arm", "home"} {
				if rtCallIndex(c.calls, op) >= 0 {
					t.Fatal(c.calls)
				}
			}
		})
	}
}
func TestStandaloneResetCleanupFailureRemainsExplicit(t *testing.T) {
	r, c := resetFixture(t)
	c.refuse = "reset_fault"
	c.releaseErr = errors.New("release unavailable")
	result, err := r.ResetFault(context.Background())
	if err == nil || result.Accepted || !strings.Contains(result.Error, "cleanup_unverified") {
		t.Fatal(result, err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if rtCallIndex(c.calls, "stop") < 0 || rtCallIndex(c.calls, "release") < 0 {
		t.Fatal(c.calls)
	}
}

func TestStandaloneResetStopCancelsPendingResetWithoutRearming(t *testing.T) {
	r, c := resetFixture(t)
	c.resetEntered = make(chan struct{})
	done := make(chan *ArmResult, 1)
	go func() { v, _ := r.ResetFault(context.Background()); done <- v }()
	select {
	case <-c.resetEntered:
	case <-time.After(3 * time.Second):
		t.Fatal("reset not submitted")
	}
	if _, err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	select {
	case v := <-done:
		if v.Accepted {
			t.Fatal("cancelled reset accepted")
		}
	case <-time.After(3 * time.Second):
		t.Fatal("reset ignored Stop")
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.grant.Session != "" || c.status.Core.Armed != 0 || c.status.Core.HomeEpoch != 7 || rtCallIndex(c.calls, "enable") >= 0 || rtCallIndex(c.calls, "arm") >= 0 {
		t.Fatal(c.calls)
	}
}

// driveAlarm latches what the 1430 cell showed after J5's Er41.1 (2026-10-11 12:19):
// drive_readiness on axis 5, recovered by a reset once the drive clears it.
func driveAlarm(c *resetClient) {
	c.status.Core.SafetyFaultMask = 16
	c.status.Core.ExecutionFaultReasons = 128
	c.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 7, Name: "drive_readiness", AxisMask: 16, Recovery: "reset_after_condition_clears", Outcome: "persists"}}}
}

// A reset native refuses (the drive alarm is still present) retires the
// session it was sent on. The failure's cleanup must not Stop that retired
// session into stop_uncertain: on the 1430 cell that refused every later Arm and Reset
// rt_core_inhibited until an OLP restart. Once the alarm clears, one Arm
// recovers.
func TestRefusedResetNeverLatchesStopUncertain(t *testing.T) {
	for _, path := range []string{"arm", "reset"} {
		t.Run(path, func(t *testing.T) {
			r, c := resetFixture(t)
			ctx := context.Background()
			c.mu.Lock()
			driveAlarm(c)
			c.persists = true
			c.mu.Unlock()
			var err error
			if path == "arm" {
				_, err = r.Arm(ctx, true)
			} else {
				_, err = r.ResetFault(ctx)
			}
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != "fault_persists" {
				t.Fatalf("refused reset must surface fault_persists: %v", err)
			}
			status := r.Status(ctx).RTCore
			if status.StopUncertain || status.Inhibited || status.RecoveryAction != "" {
				t.Fatalf("refused reset latched recovery: uncertain=%v inhibited=%v reason=%q", status.StopUncertain, status.Inhibited, status.Reason)
			}
			c.mu.Lock()
			if rtCallIndex(c.calls, "stop") >= 0 || rtCallIndex(c.calls, "enable") >= 0 || rtCallIndex(c.calls, "arm") >= 0 {
				c.mu.Unlock()
				t.Fatalf("stop on the retired session or motion after a refused reset: %v", c.calls)
			}
			c.persists = false
			c.mu.Unlock()
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatalf("Arm after the drive alarm cleared: %v", err)
			}
			c.mu.Lock()
			defer c.mu.Unlock()
			if c.status.Core.Armed != 1 || c.status.Core.SafetyFaultMask != 0 {
				t.Fatalf("Arm did not reset and arm: armed=%d faults=%#x calls=%v", c.status.Core.Armed, c.status.Core.SafetyFaultMask, c.calls)
			}
		})
	}
}

// A Stop the adapter refuses on a session it already retired is confirmed by a
// fresh observation that shows the robot disarmed with no lease and no grant.
// A foreign grant, an armed robot, an unreadable status, a transport failure or
// an acquisition of unknown outcome keeps it unconfirmed and blocking.
func TestRefusedStopConfirmedOnlyByFreshDisarmedGrantFreeObservation(t *testing.T) {
	for _, tc := range []struct {
		name      string
		change    func(*RTCoreController, *resetClient)
		confirmed bool
	}{
		{"retired_disarmed", nil, true},
		{"foreign_grant", func(_ *RTCoreController, c *resetClient) { c.foreign = true }, false},
		{"armed", func(_ *RTCoreController, c *resetClient) { c.status.Core.Armed = 1 }, false},
		{"enabled", func(_ *RTCoreController, c *resetClient) { c.status.Core.AxisEnableMask = 16 }, false},
		{"status_unreadable", func(_ *RTCoreController, c *resetClient) { c.statusErr = control.ErrTransport }, false},
		{"stop_transport", func(_ *RTCoreController, c *resetClient) { c.stopErr = control.ErrTransport }, false},
		{"acquisition_unknown", func(r *RTCoreController, _ *resetClient) { r.acquiring = true }, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, c := resetFixture(t)
			ctx := context.Background()
			if err := r.acquire(ctx); err != nil {
				t.Fatal(err)
			}
			c.mu.Lock()
			c.status.Core.SafetyFaultMask, c.status.Core.ExecutionFaultReasons, c.status.Core.Recovery = 0, 0, nil
			c.grant = control.Grant{}
			c.stopErr = &control.Rejected{Reason: "control_session_stale"}
			c.mu.Unlock()
			r.mu.Lock()
			c.mu.Lock()
			if tc.change != nil {
				tc.change(r, c)
			}
			c.mu.Unlock()
			r.mu.Unlock()
			result, err := r.Stop(ctx)
			status := r.statusSnapshot().RTCore
			if tc.confirmed {
				if err != nil || !result.OK || !result.Stopped || result.Note != "confirmed_by_observation" || status.StopUncertain || status.Inhibited {
					t.Fatalf("retired-session Stop not confirmed: note=%q ok=%v err=%v uncertain=%v reason=%q", result.Note, result.OK, err, status.StopUncertain, status.Reason)
				}
				return
			}
			if err == nil || !status.StopUncertain {
				t.Fatalf("unproven Stop confirmed: note=%q err=%v uncertain=%v", result.Note, err, status.StopUncertain)
			}
		})
	}
}
