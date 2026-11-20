//go:build linux

package main

import (
	"errors"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// The mini-bench fault fences generation before renewal fails; cleanup must
// retain the later native observation even outside interactive session mode.
func TestMoveFaultSurvivesRenewalRefusal(t *testing.T) {
	r := &runner{o: options{phase: "move"}, rows: []phaseResult{{Phase: "move", Reasons: []string{"reason 3"}}}}
	renew := errors.New("RTCore rejected application grant request (reason 3)")
	if r.retainObservedFault(renew) != renew {
		t.Fatal("invented a fault without an observation")
	}
	r.last = control.Status{Core: ipcclient.Status{
		SafetyFaultMask: 1, ExecutionFaultReasons: 4096,
		Recovery: &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 12, Name: "collision_watchdog", AxisMask: 1}}},
	}}
	want := "collision_watchdog (bit=12 axis_mask=1)"
	err := r.retainObservedFault(renew)
	if !errors.Is(err, renew) || !strings.Contains(err.Error(), want) || !strings.Contains(r.rows[0].Reasons[0], want) {
		t.Fatalf("lost native fault or renewal evidence: %v %+v", err, r.rows)
	}
	if err = r.ready(r.last, true); err == nil || !strings.Contains(err.Error(), want) {
		t.Fatalf("readiness hid fault behind epoch/authority checks: %v", err)
	}
}
