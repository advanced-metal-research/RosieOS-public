package denseexec

import (
	"strings"
	"testing"
)

// Each precondition of the one-press recovery decides exactly one next step,
// in the owner's order: confirm Stop, Reset, Arm (only confirmed), move.
// Persisting faults, Home and foreign holders stop it; nothing is routed
// around and nothing re-homes.
func TestRecoverToZeroDecisions(t *testing.T) {
	driveFault := recoverFault{Name: "drive_readiness", Axes: []string{"J5"}, Recovery: "reset_after_condition_clears"}
	er41 := recoverAlarm{Axis: "J5", Code: "Er41.1", Text: "Motor over-temperature due to locked- rotor"}
	for _, tc := range []struct {
		name   string
		facts  recoverFacts
		action string
		code   string
		words  string
	}{
		{"motion running", recoverFacts{Busy: "A joint move", StopLatched: true}, "refuse", "motion_active", "A joint move is running"},
		{"stop latched first", recoverFacts{StopLatched: true, Faults: []recoverFault{driveFault}}, "confirm_stop", "", ""},
		{"stop still latched after confirm", recoverFacts{StopLatched: true, Tried: map[string]bool{"confirm_stop": true}}, "refuse", "stop_unconfirmed", "press STOP"},
		{"foreign holder", recoverFacts{ForeignHolder: true, Faults: []recoverFault{driveFault}}, "refuse", "control_held_elsewhere", "Another controller holds the robot"},
		{"home invalid never re-homes", recoverFacts{HomeInvalid: []string{"J2", "J3"}, Faults: []recoverFault{driveFault}}, "refuse", "home_invalid", "Home is not valid on J2, J3. Recovery never re-homes"},
		{"position unknown", recoverFacts{PositionLost: []string{"J4"}}, "refuse", "position_unknown", "no valid position for J4"},
		{"rehome fault is not reset", recoverFacts{Faults: []recoverFault{driveFault, {Name: "coordinate_reference", Axes: []string{"J1"}, Recovery: "rehome_required"}}}, "refuse", "fault_not_resettable", "J1 coordinate_reference needs a new Home at the mechanical home pose"},
		{"restart fault is not reset", recoverFacts{Faults: []recoverFault{{Name: "watchdog", Recovery: "restart_required"}}}, "refuse", "fault_not_resettable", "Fault watchdog needs a controller restart"},
		{"unclassified fault is not reset", recoverFacts{Faults: []recoverFault{{Name: "fault mask 16/128", Recovery: "unknown"}}}, "refuse", "fault_not_resettable", "needs unknown"},
		{"resettable fault resets", recoverFacts{Faults: []recoverFault{driveFault}, Alarms: []recoverAlarm{er41}}, "reset", "", ""},
		{"reset refused names the alarm", recoverFacts{Faults: []recoverFault{driveFault}, Alarms: []recoverAlarm{er41}, Tried: map[string]bool{"reset": true}}, "refuse", "fault_persists",
			"J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again"},
		{"reset that would lose Home is not sent", recoverFacts{Faults: []recoverFault{{Name: "drive_readiness", Axes: []string{"J3"}, Recovery: "reset_after_condition_clears", Rehome: []string{"J3"}}}}, "refuse", "fault_not_resettable",
			"J3 drive_readiness lost the position reference on J3: it needs a new Home at the mechanical home pose (recovery never re-homes)"},
		{"drive in Fault with nothing latched resets", recoverFacts{Alarms: []recoverAlarm{er41}}, "reset", "", ""},
		{"reset refused without a drive word", recoverFacts{Faults: []recoverFault{{Name: "target_lead", Axes: []string{"J3"}, Recovery: "reset_after_condition_clears"}}, Tried: map[string]bool{"reset": true}}, "refuse", "fault_persists",
			"J3 target_lead still active after Reset — clear its cause, then press again"},
		{"at zero needs no arm", recoverFacts{AtZero: true}, "at_zero", "", ""},
		{"disarmed asks first", recoverFacts{}, "confirm_arm", "", ""},
		{"disarmed and confirmed arms", recoverFacts{ArmConfirmed: true}, "arm", "", ""},
		{"armed but not observed", recoverFacts{ArmConfirmed: true, Tried: map[string]bool{"arm": true}}, "refuse", "arm_unconfirmed", "does not report armed"},
		{"armed moves without asking", recoverFacts{Armed: true}, "move", "", ""},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if tc.facts.Tried == nil {
				tc.facts.Tried = map[string]bool{}
			}
			got := nextRecoverStep(tc.facts)
			if got.Action != tc.action || got.Code != tc.code || !strings.Contains(got.Detail, tc.words) {
				t.Fatalf("got %+v; want action %s code %s containing %q", got, tc.action, tc.code, tc.words)
			}
		})
	}
}

// A non-thermal drive alarm is not told to cool down.
func TestRecoverPersistingFaultRemedy(t *testing.T) {
	text := persistingFaultText(nil, []recoverAlarm{{Axis: "J2", Code: "Er25.0", Text: "Encoder communication"}}, "press again")
	if text != "J2 drive alarm Er25.0 (Encoder communication) still active — clear its cause at the drive, then press again" {
		t.Fatal(text)
	}
}

// Arm runs the recovery press's Stop confirmation and fault steps, in the same
// order, with Arm's words, and then arms; with nothing latched it is the Arm
// it always was. It never asks for a confirmation of its own.
func TestArmDecisions(t *testing.T) {
	driveFault := recoverFault{Name: "drive_readiness", Axes: []string{"J5"}, Recovery: "reset_after_condition_clears"}
	er41 := recoverAlarm{Axis: "J5", Code: "Er41.1", Text: "Motor over-temperature due to locked- rotor"}
	for _, tc := range []struct {
		name   string
		facts  recoverFacts
		action string
		code   string
		words  string
	}{
		{"nothing latched arms", recoverFacts{}, "arm", "", ""},
		{"nothing latched arms even while busy or foreign: Arm's own admission decides", recoverFacts{Busy: "A jog", ForeignHolder: true}, "arm", "", ""},
		{"home invalid is Home's step, not Arm's", recoverFacts{HomeInvalid: []string{"J2"}}, "arm", "", ""},
		{"unconfirmed stop first", recoverFacts{StopLatched: true, Faults: []recoverFault{driveFault}}, "confirm_stop", "", ""},
		{"stop confirmation not retried", recoverFacts{StopLatched: true, Tried: map[string]bool{"confirm_stop": true}}, "refuse", "stop_unconfirmed", "press STOP"},
		{"resettable fault resets", recoverFacts{Faults: []recoverFault{driveFault}, Alarms: []recoverAlarm{er41}}, "reset", "", ""},
		{"drive in Fault with nothing latched resets", recoverFacts{Alarms: []recoverAlarm{er41}}, "reset", "", ""},
		{"no reset while something runs", recoverFacts{Busy: "A joint move", Faults: []recoverFault{driveFault}}, "refuse", "motion_active", "then Arm again"},
		{"another controller's fault is not reset", recoverFacts{ForeignHolder: true, Faults: []recoverFault{driveFault}}, "refuse", "control_held_elsewhere", "the robot stays disarmed"},
		{"persisting alarm", recoverFacts{Faults: []recoverFault{driveFault}, Alarms: []recoverAlarm{er41}, Tried: map[string]bool{"reset": true}}, "refuse", "fault_persists",
			"J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again"},
		{"rehome fault", recoverFacts{Faults: []recoverFault{driveFault, {Name: "coordinate_reference", Axes: []string{"J1"}, Recovery: "rehome_required"}}}, "refuse", "fault_not_resettable",
			"J1 coordinate_reference needs a new Home at the mechanical home pose (Arm never re-homes), not a reset; the robot stays disarmed"},
		{"untrusted evidence", recoverFacts{Faults: []recoverFault{{Name: "bus_transport", Axes: []string{"J4"}, Recovery: "reset_after_condition_clears", Rehome: []string{"J4"}}}}, "refuse", "fault_not_resettable",
			"needs a new Home at the mechanical home pose (Arm never re-homes)"},
		{"restart fault", recoverFacts{Faults: []recoverFault{{Name: "pdo_mapping_mismatch", Axes: []string{"J4"}, Recovery: "restart_required"}}}, "refuse", "fault_not_resettable", "needs a controller restart"},
		{"settled faults arm", recoverFacts{Tried: map[string]bool{"reset": true}}, "arm", "", ""},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if tc.facts.Tried == nil {
				tc.facts.Tried = map[string]bool{}
			}
			got := nextArmStep(tc.facts)
			if got.Action != tc.action || got.Code != tc.code || !strings.Contains(got.Detail, tc.words) {
				t.Fatalf("got %+v; want action %s code %s containing %q", got, tc.action, tc.code, tc.words)
			}
		})
	}
}

// The confirmation names joints in degrees, Δ is the travel to zero, and a
// joint inside the table's zero tolerance stays.
func TestRecoverJointsAndStaleConfirmation(t *testing.T) {
	joints := recoverJoints([]string{"J1", "J2", "J3"}, []float64{0.5, -0.25, 0.0002})
	if joints[0].FromDeg < 28.64 || joints[0].FromDeg > 28.65 || joints[0].DeltaDeg != -joints[0].FromDeg || !joints[0].Moves || joints[2].Moves {
		t.Fatalf("%+v", joints)
	}
	if stale := recoverConfirmationStale(joints, joints); stale != "" {
		t.Fatal(stale)
	}
	moved := append([]RecoverJoint(nil), joints...)
	moved[1].FromDeg += 0.5
	if stale := recoverConfirmationStale(joints, moved); !strings.Contains(stale, "J2 by 0.50°") {
		t.Fatal(stale)
	}
	if stale := recoverConfirmationStale(nil, joints); !strings.Contains(stale, "J1 (not in the confirmation)") {
		t.Fatal(stale)
	}
	within := append([]RecoverJoint(nil), joints...)
	within[0].FromDeg += 0.05
	if stale := recoverConfirmationStale(joints, within); stale != "" {
		t.Fatal("a 0.05° reading change is the same pose:", stale)
	}
}
