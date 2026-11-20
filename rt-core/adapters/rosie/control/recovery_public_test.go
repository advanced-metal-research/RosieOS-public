//go:build linux

package control

import (
	"encoding/json"
	"net/http"
	"testing"

	"rosieos/rt-core/ipcclient"
)

func TestPublicRecoveryStatusListsLatchedFaults(t *testing.T) {
	f := startPublicReset(t, "simulation-home.json", 1,
		"--sim-send-fail-from-call", "5000", "--sim-send-fail-for-calls", "600")
	f.command(t, Request{Operation: "home", AxisMask: 1}, "")
	f.wait(t, func(s publicResetStatus) bool { return s.Core.HomeValidMask == 1 && s.Core.CommissioningPhase == 0 })
	f.command(t, Request{Operation: "release"}, "")
	faulted := f.wait(t, func(s publicResetStatus) bool { return s.Core.ExecutionFaultReasons&256 != 0 && s.Core.Armed == 0 })
	f.command(t, Request{Operation: "acquire", Controller: "recovery-status", Binding: f.binding}, "")
	response := f.command(t, Request{Operation: "recovery_status"}, "")
	var recovery ipcclient.RecoveryStatus
	if err := json.Unmarshal(response.Data, &recovery); err != nil {
		t.Fatal(err)
	}
	if len(recovery.Faults) != 1 || recovery.Faults[0].Bit != 8 || recovery.Faults[0].Name != "bus_transport" ||
		recovery.Faults[0].AxisMask != 1 || recovery.Faults[0].Recovery != "reset_after_condition_clears" {
		t.Fatalf("public recovery omitted native fault details: %+v", recovery)
	}
	f.wait(t, func(s publicResetStatus) bool {
		return s.Core.TimeNS > faulted.Core.TimeNS && s.Core.ConfigVerifiedMask == 1
	})
	response = f.command(t, Request{Operation: "reset_fault", AxisMask: 1}, "")
	requirePublicResetResult(t, response)
	if err := json.Unmarshal(response.Data, &recovery); err != nil {
		t.Fatal(err)
	}
	if recovery.ResetSequence != response.Sequence || recovery.HomeValidMask != 1 || len(recovery.Outcomes) != 1 || recovery.Outcomes[0].Outcome != "cleared" {
		t.Fatalf("public reset lacks correlated per-bit outcome: %+v", recovery)
	}
}

// Exercise the real public socket with an issued session but expired native
// grant; Stop never uses a native IPC shortcut or reacquires motion authority.
func TestStopAcceptedAfterGrantExpiry(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.ready(t, 511)
	f.mu.Lock()
	fence := f.fence
	f.fence = Fence{} // Disable fixture renewal without releasing the session.
	f.mu.Unlock()
	expired := f.wait(t, func(s publicResetStatus) bool { return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 })
	response := recoveryPublicRequest(t, f, Request{Operation: "stop", Fence: fence}, "")
	var grant Grant
	if err := json.Unmarshal(response.Data, &grant); err != nil {
		t.Fatal(err)
	}
	if grant.Session != "" {
		t.Fatalf("expired Stop reacquired authority: %+v", grant)
	}
	// Repeat with the same issued session: Stop remains an idempotent inhibition.
	recoveryPublicRequest(t, f, Request{Operation: "stop", Fence: fence}, "")
	last := expired.Core.TimeNS
	for sample := 0; sample < 6; sample++ {
		observed := f.wait(t, func(s publicResetStatus) bool { return s.Core.TimeNS > last })
		last = observed.Core.TimeNS
		if observed.Core.Armed != 0 || observed.Core.AxisEnableMask != 0 {
			t.Fatalf("expired Stop restored outputs: %+v", observed)
		}
		for axis := range observed.Core.Axes {
			if observed.Core.Axes[axis].PositionCounts-expired.Core.Axes[axis].PositionCounts != 0 {
				t.Fatalf("motion after expired Stop: %+v", observed)
			}
		}
	}
}

func TestStopRequiresSession(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.mu.Lock()
	issued := f.fence
	f.mu.Unlock()
	recoveryPublicRequest(t, f, Request{Operation: "stop"}, "control_session_stale")
	recoveryPublicRequest(t, f, Request{Operation: "stop", Fence: Fence{Session: "wrong-session", Generation: issued.Generation}}, "control_session_stale")
	recoveryPublicRequest(t, f, Request{Operation: "stop", Fence: Fence{Session: issued.Session, Generation: 0}}, "control_session_stale")
}

func TestStopAcceptedAfterGrantRevocation(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.mu.Lock()
	fence := f.fence
	f.mu.Unlock()
	f.command(t, Request{Operation: "release"}, "")
	recoveryPublicRequest(t, f, Request{Operation: "stop", Fence: fence}, "")
}

func TestAcquireAfterExpiredAuthorityHandoff(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.ready(t, 511)
	f.mu.Lock()
	previous := f.fence
	f.fence = Fence{}
	f.mu.Unlock()
	f.wait(t, func(s publicResetStatus) bool { return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 })
	response := f.command(t, Request{Operation: "acquire", Controller: "recovery-successor", Binding: f.binding}, "")
	var grant Grant
	if err := json.Unmarshal(response.Data, &grant); err != nil {
		t.Fatal(err)
	}
	if grant.Session == previous.Session || grant.Generation != 3 {
		t.Fatalf("expiry handoff did not issue the fenced successor: %+v", grant)
	}
}

func TestExpiredRecoveryHandoffCannotStopSuccessor(t *testing.T) {
	c := &Controller{grant: Grant{Session: "successor", Generation: 3}}
	if _, err := c.stopGrant(Fence{Session: "expired", Generation: 1}, true, true); err != nil {
		t.Fatal(err)
	}
	if c.grant.Session != "successor" || c.grant.Generation != 3 || c.stopping {
		t.Fatalf("old automatic expiry handoff changed the successor: %+v", c.grant)
	}
}

func TestPublicRecoveryStatusRequiresInhibited(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.ready(t, 511)
	f.command(t, Request{Operation: "recovery_status"}, "reset_requires_inhibited")
}

func TestPublicResetFaultPersistsReturnsNativeOutcome(t *testing.T) {
	f := startPublicReset(t, "simulation-home.json", 1,
		"--sim-send-fail-from-call", "3000", "--sim-send-fail-for-calls", "2000")
	f.command(t, Request{Operation: "release"}, "")
	faulted := f.wait(t, func(s publicResetStatus) bool { return s.Core.ExecutionFaultReasons&256 != 0 })
	f.command(t, Request{Operation: "acquire", Controller: "recovery-persist", Binding: f.binding}, "")
	f.mu.Lock()
	fence := f.fence
	f.fence = Fence{}
	f.mu.Unlock()
	response := recoveryPublicRequest(t, f, Request{Operation: "reset_fault", AxisMask: 1, Fence: fence}, "fault_persists")
	if response.NativeResult == nil || response.NativeResult.Sequence != response.Sequence || response.NativeResult.Result != ipcclient.ResultRejected {
		t.Fatalf("persistent reset omitted native rejection: %+v", response)
	}
	var recovery ipcclient.RecoveryStatus
	if err := json.Unmarshal(response.Data, &recovery); err != nil {
		t.Fatal(err)
	}
	if recovery.ResetSequence != response.Sequence || recovery.Reason != "fault_persists" || len(recovery.Outcomes) != 1 || recovery.Outcomes[0].Outcome != "persists" {
		t.Fatalf("persistent reset omitted per-bit outcome: %+v", recovery)
	}
	observed := f.wait(t, func(s publicResetStatus) bool { return s.Core.TimeNS > faulted.Core.TimeNS })
	if observed.Core.ExecutionFaultReasons != faulted.Core.ExecutionFaultReasons || observed.Core.SafetyFaultMask != faulted.Core.SafetyFaultMask {
		t.Fatalf("persistent public reset cleared fault state: %+v", observed)
	}
}

// Bypass only fixture fence substitution, retaining the actual authenticated
// public operation path for missing, revoked and expired session assertions.
func recoveryPublicRequest(t *testing.T, f *publicResetFixture, request Request, wantError string) publicResetResponse {
	t.Helper()
	request.Schema = "rosie.rt-control.request.v1"
	raw, err := json.Marshal(request)
	if err != nil {
		t.Fatal(err)
	}
	response, code, err := f.request(http.MethodPost, "/v1/control", raw, nil)
	wantCode := http.StatusOK
	if wantError != "" {
		wantCode = http.StatusConflict
	}
	if err != nil || code != wantCode || response.Error != wantError {
		t.Fatalf("%s: code=%d response=%+v error=%v", request.Operation, code, response, err)
	}
	return response
}
