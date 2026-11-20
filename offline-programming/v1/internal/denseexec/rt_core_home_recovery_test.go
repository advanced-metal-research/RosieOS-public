//go:build linux

package denseexec

import (
	"context"
	"errors"
	api "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
	"strings"
	"testing"
	"time"
)

type homeRecoveryClient struct {
	RTCoreClient
	entered    chan struct{}
	proceed    chan struct{}
	resetError error
	afterReset bool
}

func (c *homeRecoveryClient) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	if request.Operation == "reset_fault" {
		var response control.Response
		var err error
		if c.afterReset {
			response, err = c.RTCoreClient.Command(ctx, request)
		}
		close(c.entered)
		select {
		case <-c.proceed:
		case <-ctx.Done():
			return control.Response{}, ctx.Err()
		}
		if c.resetError != nil {
			return control.Response{}, c.resetError
		}
		if c.afterReset {
			return response, err
		}
	}
	return c.RTCoreClient.Command(ctx, request)
}
func referenceFault(f *fakeRTCore, axisMask uint32) {
	f.status.Core.HomeValidMask &^= axisMask
	f.status.Core.SafetyFaultMask = axisMask
	f.status.Core.ExecutionFaultReasons = 64
	f.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 6, Name: "coordinate_reference", AxisMask: axisMask, RehomeAxisMask: axisMask, Recovery: "rehome_required"}}}
}
func TestRTCoreHomeRecoversOnlyQualifiedReferenceFault(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	f.mu.Unlock()
	wrapped := &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: make(chan struct{})}
	r.c = wrapped
	finished := make(chan error, 1)
	go func() { finished <- r.HomeAxes(context.Background(), 4) }()
	select {
	case <-wrapped.entered:
	case err := <-finished:
		t.Fatalf("Home skipped coordinate recovery reset: %v", err)
	case <-time.After(time.Second):
		t.Fatal("reset not reached")
	}
	// The observer must not cancel the reset that resolves this already-known fault.
	if err := r.observe(context.Background()); err != nil {
		close(wrapped.proceed)
		t.Fatalf("existing recovery fault cancelled reset: %v", err)
	}
	close(wrapped.proceed)
	if err := <-finished; err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	calls := append([]string(nil), f.calls...)
	homed := f.status.Core.HomeValidMask
	masks := append([]uint32(nil), f.homeMasks...)
	f.mu.Unlock()
	reset, home := rtCallIndex(calls, "reset_fault"), rtCallIndex(calls, "home")
	if reset < 0 || home <= reset || rtCallIndex(calls[reset+1:home], "acquire") < 0 {
		t.Fatalf("reset, reacquire, Home order: %v", calls)
	}
	if len(masks) != 1 || masks[0] != 4 || homed&4 == 0 {
		t.Fatalf("Home scope/result: %v %#x", masks, homed)
	}
	if rtCallIndex(calls, "arm") >= 0 || rtCallIndex(calls, "enable") >= 0 {
		t.Fatalf("recovery enabled motion: %v", calls)
	}
}
func TestRTCoreHomeRecoveryPreservesNativeResetRefusal(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	f.mu.Unlock()
	ready := make(chan struct{})
	close(ready)
	r.c = &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: ready, resetError: &control.Rejected{Reason: "fault_persists: independent encoder proof unavailable"}}
	err := r.HomeAxes(context.Background(), 4)
	if err == nil || !strings.Contains(err.Error(), "independent encoder proof unavailable") {
		t.Fatalf("native reset diagnosis lost: %v", err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if rtCallIndex(f.calls, "home") >= 0 {
		t.Fatalf("Home sent after refused reset: %v", f.calls)
	}
}

func TestRTCoreHomeRecoveryRefusesUnrelatedOrUnsafeInitialFault(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*fakeRTCore)
	}{
		{"outside_requested_axes", func(f *fakeRTCore) { referenceFault(f, 8) }},
		{"armed", func(f *fakeRTCore) { f.status.Core.Armed = 1 }},
		{"enabled", func(f *fakeRTCore) { f.status.Core.AxisEnableMask = 4 }},
		{"missing_recovery", func(f *fakeRTCore) { f.status.Core.Recovery = nil }},
		{"different_fault", func(f *fakeRTCore) { f.status.Core.ExecutionFaultReasons |= 4 }},
		{"other_rehome_kind", func(f *fakeRTCore) { f.status.Core.Recovery.Faults[0].Name = "position_rate" }},
		// AMR-720: HM35 would re-zero the joint at its fault pose. Clear faults
		// or Arm resets target_lead; Home must never send its reset.
		{"target_lead", func(f *fakeRTCore) {
			f.status.Core.HomeValidMask |= 4
			f.status.Core.ExecutionFaultReasons = 16
			f.status.Core.Recovery.Faults[0] = ipcclient.FaultRecovery{Bit: 4, Name: "target_lead", AxisMask: 4, Recovery: "reset_after_condition_clears"}
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, f, _ := rtFake(t)
			f.mu.Lock()
			referenceFault(f, 4)
			tc.change(f)
			f.mu.Unlock()
			if err := r.HomeAxes(context.Background(), 4); err == nil {
				t.Fatal("unsafe/unrelated fault accepted")
			}
			f.mu.Lock()
			defer f.mu.Unlock()
			if rtCallIndex(f.calls, "reset_fault") >= 0 || rtCallIndex(f.calls, "home") >= 0 {
				t.Fatalf("recovery command sent: %v", f.calls)
			}
		})
	}
}

func TestRTCoreHomeRecoveryStillStopsOnNewFailure(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*fakeRTCore)
	}{
		{"execution_error", func(f *fakeRTCore) { f.status.Execution.Error = "new_authority_failure" }},
		{"reference_fault_spreads", func(f *fakeRTCore) { referenceFault(f, 12) }},
		{"new_fault", func(f *fakeRTCore) { f.status.Core.ExecutionFaultReasons |= 4 }},
		{"becomes_armed", func(f *fakeRTCore) { f.status.Core.Armed = 1 }},
		{"different_axis_faulted", func(f *fakeRTCore) { f.status.Axes[0].Readiness = "faulted" }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, f, _ := rtFake(t)
			f.mu.Lock()
			referenceFault(f, 4)
			f.mu.Unlock()
			wrapped := &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: make(chan struct{})}
			r.c = wrapped
			finished := make(chan error, 1)
			go func() { finished <- r.HomeAxes(context.Background(), 511) }()
			select {
			case <-wrapped.entered:
			case err := <-finished:
				t.Fatalf("reset not reached: %v", err)
			case <-time.After(time.Second):
				t.Fatal("reset not reached")
			}
			f.mu.Lock()
			tc.change(f)
			f.mu.Unlock()
			err := r.observe(context.Background())
			if err == nil {
				close(wrapped.proceed)
				<-finished
				t.Fatal("new failure hidden by recovery")
			}
			_ = r.fail(err)
			if err := <-finished; err == nil {
				t.Fatal("Home survived observer refusal")
			}
			f.mu.Lock()
			defer f.mu.Unlock()
			if rtCallIndex(f.calls, "home") >= 0 {
				t.Fatalf("Home sent after new failure: %v", f.calls)
			}
		})
	}
}

// Home's reference-recovery reset, and Arm's reset step (ResetFault, on
// the 1430 cell's 12:19 state), keep their grant while the adapter retires it before
// the Reset reply arrives; nothing proceeds until that reply. Arm no longer
// resets a fault that would leave a joint's Home invalid
// (TestArmNeverResetsWhatNeedsHome), so its case is the drive alarm.
func TestRTCoreHomeRecoveryRetirementBeforeResetReply(t *testing.T) {
	for _, action := range []string{"home", "arm"} {
		t.Run(action, func(t *testing.T) {
			r, f, _ := rtFake(t)
			f.mu.Lock()
			referenceFault(f, 4)
			f.mu.Unlock()
			var inner RTCoreClient = f
			if action == "arm" {
				var c *recoverClient
				r, c = recoverFixture(t)
				f, inner = c.fakeRTCore, c
				r.mu.Lock()
				r.stopUncertain, r.reason = false, ""
				r.mu.Unlock()
			}
			wrapped := &homeRecoveryClient{RTCoreClient: inner, entered: make(chan struct{}), proceed: make(chan struct{}), afterReset: true}
			r.c = wrapped
			finished := make(chan error, 1)
			go func() {
				if action == "arm" {
					_, err := r.Arm(context.Background(), true)
					finished <- err
				} else {
					finished <- r.HomeAxes(context.Background(), 4)
				}
			}()
			select {
			case <-wrapped.entered:
			case err := <-finished:
				t.Fatal(err)
			case <-time.After(time.Second):
				t.Fatal("reset not reached")
			}
			f.updates <- control.Renewal{Err: &control.Rejected{Reason: "control_session_stale"}}
			// Wait for the authority watcher to consume the explicit retirement, while
			// withholding the successful Reset reply. No Home may be sent yet.
			deadline := time.After(time.Second)
			for {
				r.mu.Lock()
				deferred := r.renewals == nil
				held := r.held
				r.mu.Unlock()
				if deferred {
					if !held {
						t.Fatal("retirement spuriously stopped reset")
					}
					break
				}
				select {
				case err := <-finished:
					t.Fatalf("reset cancelled before its reply: %v", err)
				case <-deadline:
					t.Fatal("retirement not observed")
				case <-time.After(time.Millisecond):
				}
			}
			f.mu.Lock()
			if rtCallIndex(f.calls, "home") >= 0 {
				t.Fatal("Home sent before Reset receipt")
			}
			f.mu.Unlock()
			close(wrapped.proceed)
			if err := <-finished; err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestRTCoreHomeRecoveryCanRetryAfterTerminalResetFailure(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	f.mu.Unlock()
	ready := make(chan struct{})
	close(ready)
	r.c = &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: ready, resetError: &control.Rejected{Reason: "fault_persists"}}
	if err := r.HomeAxes(context.Background(), 4); err == nil {
		t.Fatal("expected first reset refusal")
	}
	f.mu.Lock()
	f.status.Execution = control.Execution{State: "faulted", Error: "fault_persists"}
	f.mu.Unlock()
	r.c = f
	if err := r.HomeAxes(context.Background(), 4); err != nil {
		t.Fatalf("new acquisition could not retire historical failure: %v", err)
	}
}

func TestHomeReferenceRecoveryEventAndRenewalScope(t *testing.T) {
	_, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	s := f.status
	f.mu.Unlock()
	h := referenceRecoveryForHome(s, 4)
	event := control.Event{Type: "fault_cleared", TimeNS: s.TimeNS, AdapterIncarnation: s.AdapterIncarnation, DaemonIncarnation: s.DaemonIncarnation, Fault: &api.FaultEvent{Bit: 64, AxisMask: 4, RecoveryClass: "rehome_required"}}
	if !h.expectedClear(event) {
		t.Fatal("matching reference-clear event refused")
	}
	event.Type = "fault_latched"
	if h.expectedClear(event) {
		t.Fatal("new fault hidden")
	}
	event.Type = "fault_cleared"
	event.Fault.AxisMask = 8
	if h.expectedClear(event) {
		t.Fatal("other joint event hidden")
	}
	if !retiredHomeResetGrant(&control.Rejected{Reason: "control_session_stale"}) || retiredHomeResetGrant(control.ErrTransport) || retiredHomeResetGrant(control.ErrProtocol) || retiredHomeResetGrant(&control.Rejected{Reason: "expired"}) {
		t.Fatal("retirement scope is not exact")
	}
}

func TestRTCoreHomeRecoveryRetiredGrantDoesNotHideResetRefusal(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	f.mu.Unlock()
	wrapped := &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: make(chan struct{}), resetError: &control.Rejected{Reason: "fault_persists: encoder proof stale"}}
	r.c = wrapped
	finished := make(chan error, 1)
	go func() { finished <- r.HomeAxes(context.Background(), 4) }()
	select {
	case <-wrapped.entered:
	case err := <-finished:
		t.Fatal(err)
	case <-time.After(time.Second):
		t.Fatal("reset not reached")
	}
	f.updates <- control.Renewal{Err: &control.Rejected{Reason: "control_session_stale"}}
	deadline := time.After(time.Second)
	for {
		r.mu.Lock()
		deferred := r.renewals == nil
		r.mu.Unlock()
		if deferred {
			break
		}
		select {
		case err := <-finished:
			t.Fatalf("retirement hid reset result: %v", err)
		case <-deadline:
			t.Fatal("retirement not observed")
		case <-time.After(time.Millisecond):
		}
	}
	close(wrapped.proceed)
	err := <-finished
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "fault_persists" || !strings.Contains(failure.Detail, "Home recovery reset refused:") || !strings.Contains(failure.Detail, "encoder proof stale") {
		t.Fatalf("reset evidence lost: %v", err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if rtCallIndex(f.calls, "home") >= 0 {
		t.Fatalf("Home followed refused reset: %v", f.calls)
	}
}

func TestRTCoreHomeRecoveryTransportLossStillStops(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	f.mu.Unlock()
	wrapped := &homeRecoveryClient{RTCoreClient: f, entered: make(chan struct{}), proceed: make(chan struct{})}
	r.c = wrapped
	finished := make(chan error, 1)
	go func() { finished <- r.HomeAxes(context.Background(), 4) }()
	select {
	case <-wrapped.entered:
	case err := <-finished:
		t.Fatal(err)
	case <-time.After(time.Second):
		t.Fatal("reset not reached")
	}
	f.updates <- control.Renewal{Err: control.ErrTransport}
	select {
	case err := <-finished:
		if err == nil {
			t.Fatal("transport failure ignored")
		}
	case <-time.After(time.Second):
		t.Fatal("transport failure did not cancel reset")
	}
	// Cancellation wakes Home before its independent Stop owner has finished.
	deadline := time.After(time.Second)
	for {
		r.mu.Lock()
		stopped := r.stopping == 0
		r.mu.Unlock()
		if stopped {
			break
		}
		select {
		case <-deadline:
			t.Fatal("transport failure Stop did not finish")
		case <-time.After(time.Millisecond):
		}
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if rtCallIndex(f.calls, "stop") < 0 || rtCallIndex(f.calls, "home") >= 0 {
		t.Fatalf("wrong transport loss outcome: %v", f.calls)
	}
}
