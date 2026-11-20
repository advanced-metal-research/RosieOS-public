package rtctl

import (
	"strings"
	"testing"
)

// Explicit diagnostic ceilings, not claimed hardware qualification.
const transitionJSON = `{"max_displacement_counts":327,"max_velocity_counts_per_s":6553,"target_stationary_ms":100,"settling_ms":100,"timeout_ms":500}`

func transitionFixture() *fixture {
	f := fresh()
	f.put("profile.brake", `{"present":true,"hold_displacement_tolerance_counts":16}`)
	tx := asArray(f.get("profile.ethercat.tx_layout"))
	tx = append(tx, decode([]byte(`{"semantic":"velocity_actual","index":24684,"subindex":0,"bits":32}`)))
	f.put("profile.ethercat.tx_layout", canonical(tx))
	return f
}

func TestBrakeTransitionCompilerOptIn(t *testing.T) {
	f := transitionFixture()
	before, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(before.AxisProfiles, "brake_transition=") {
		t.Fatal("default opted in")
	}
	f.put("machine.axes.0.brake_transition_guard", transitionJSON)
	after, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(after.AxisProfiles, "brake_transition=327|6553|100|100|500\n") || before.MachineSHA256 == after.MachineSHA256 {
		t.Fatal("transition absent from native configuration or identity")
	}
	f.put("machine.axes.0.brake_transition_guard.max_displacement_counts", "328")
	changed, err := f.compile()
	if err != nil || changed.MachineSHA256 == after.MachineSHA256 {
		t.Fatal("cap identity lost", err)
	}
}

func TestBrakeTransitionCompilerRefusesPartialOrImplicitPolicy(t *testing.T) {
	for _, block := range []string{`null`, `{}`, `[]`, strings.Replace(transitionJSON, `327`, `0`, 1),
		strings.Replace(transitionJSON, `327`, `15`, 1), strings.Replace(transitionJSON, `6553`, `-1`, 1),
		strings.Replace(transitionJSON, `6553`, `1.5`, 1), strings.Replace(transitionJSON, `500`, `199`, 1),
		strings.Replace(transitionJSON, `500`, `2147483648`, 1), strings.Replace(transitionJSON, `"settling_ms":100,`, ``, 1),
		strings.Replace(transitionJSON, `"settling_ms"`, `"settle_typo"`, 1)} {
		f := transitionFixture()
		f.put("machine.axes.0.brake_transition_guard", block)
		if _, err := f.compile(); err == nil {
			t.Fatalf("accepted malformed transition %s", block)
		}
	}
	for _, mutation := range []struct{ key, value string }{
		{"profile.brake.present", "false"},
		{"profile.ethercat.tx_layout.3.bits", "16"},
		{"profile.brake_transition_guard", transitionJSON},
	} {
		f := transitionFixture()
		f.put("machine.axes.0.brake_transition_guard", transitionJSON)
		f.put(mutation.key, mutation.value)
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted", mutation.key)
		}
	}
}

func TestRobotBrakeTransitionIsNarrowMachineAxisPolicy(t *testing.T) {
	f := newRobotFixture(t, true)
	before, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	// A robot-bound machine declares the guard once in joint units for every
	// axis; a per-axis count block would be a per-joint number.
	f.setAxis("brake_transition_guard", decode([]byte(transitionJSON)))
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "robot_brake_guard_per_axis") {
		t.Fatalf("per-axis robot guard admitted: %v", err)
	}
	f.dropAxis("brake_transition_guard")
	f.machine.set("brake_transition_guard", decode([]byte(machineTransitionJSON)))
	after, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if before.MachineSHA256 == after.MachineSHA256 || !strings.Contains(after.AxisProfiles, "brake_transition=") {
		t.Fatal("robot machine-level policy lost")
	}
	// Existing mechanical brake override exclusion remains in force.
	f.setAxis("brake", decode([]byte(`{"present":false}`)))
	if _, err := f.compile(); err == nil {
		t.Fatal("general brake override admitted")
	}
}

// Opening is an explicit, identity-bound acquisition phase, never a new default.
const openingJSON = `{"max_displacement_counts":327,"max_velocity_counts_per_s":13107,"timeout_ms":500}`

func TestBrakeOpeningCompilerOptIn(t *testing.T) {
	f := transitionFixture()
	f.put("machine.axes.0.brake_transition_guard", transitionJSON)
	before, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(before.AxisProfiles, "brake_opening=") {
		t.Fatal("default opening enabled")
	}
	f.put("machine.axes.0.brake_opening_guard", openingJSON)
	after, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(after.AxisProfiles, "brake_opening=327|13107|500\n") || after.MachineSHA256 == before.MachineSHA256 {
		t.Fatal("opening identity absent")
	}
	for _, field := range []string{"max_displacement_counts", "max_velocity_counts_per_s", "timeout_ms"} {
		f.put("machine.axes.0.brake_opening_guard", openingJSON)
		f.put("machine.axes.0.brake_opening_guard."+field, "400")
		changed, err := f.compile()
		if err != nil || changed.MachineSHA256 == after.MachineSHA256 {
			t.Fatal("opening parameter not pinned", field, err)
		}
	}
}
func TestBrakeOpeningCompilerRefusesPartialPolicy(t *testing.T) {
	for _, block := range []string{`null`, `{}`, `[]`, strings.Replace(openingJSON, `327`, `15`, 1), strings.Replace(openingJSON, `13107`, `0`, 1), strings.Replace(openingJSON, `500`, `0`, 1), strings.Replace(openingJSON, `500`, `501`, 1), strings.Replace(openingJSON, `13107`, `1.5`, 1), strings.Replace(openingJSON, `"timeout_ms":500`, `"unknown":500`, 1)} {
		f := transitionFixture()
		f.put("machine.axes.0.brake_transition_guard", transitionJSON)
		f.put("machine.axes.0.brake_opening_guard", block)
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted malformed opening", block)
		}
	}
	for _, mutation := range []struct{ key, value string }{{"profile.brake_opening_guard", openingJSON}, {"machine.axes.0.brake_transition_guard", "null"}, {"profile.brake.present", "false"}} {
		f := transitionFixture()
		f.put("machine.axes.0.brake_transition_guard", transitionJSON)
		f.put("machine.axes.0.brake_opening_guard", openingJSON)
		f.put(mutation.key, mutation.value)
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted opening", mutation.key)
		}
	}
}
func TestRobotBrakeOpeningIsMachineAxisPolicy(t *testing.T) {
	f := newRobotFixture(t, true)
	f.machine.set("brake_transition_guard", decode([]byte(machineTransitionJSON)))
	f.setAxis("brake_opening_guard", decode([]byte(openingJSON)))
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "robot_brake_guard_per_axis") {
		t.Fatalf("per-axis robot opening admitted: %v", err)
	}
	f.dropAxis("brake_opening_guard")
	f.machine.set("brake_opening_guard", decode([]byte(machineOpeningJSON)))
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(compiled.AxisProfiles, "brake_opening=") {
		t.Fatal("robot opening policy lost")
	}
}

// Machine-level guards in joint units: 0.050 degrees, 1.0 and 2.0 degrees/s.
const (
	machineTransitionJSON = `{"max_displacement_rad":0.0008726646259971648,"max_velocity_rad_per_s":0.017453292519943295,"target_stationary_ms":100,"settling_ms":100,"timeout_ms":500}`
	machineOpeningJSON    = `{"max_displacement_rad":0.0008726646259971648,"max_velocity_rad_per_s":0.03490658503988659,"timeout_ms":500}`
)

// dropAxis removes key from the fixture's machine axis; an explicit null would
// be refused as a malformed guard rather than mean absence.
func (f robotFixture) dropAxis(key string) {
	kept := object{}
	for _, m := range f.axis() {
		if m.key != key {
			kept = append(kept, m)
		}
	}
	f.machine.set("axes", []any{kept})
}
