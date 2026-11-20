package rtctl

import (
	"strings"
	"testing"
)

func TestA6RobotUsesSoftwareGearRatio(t *testing.T) {
	f := newRobotFixture(t, true)
	f.bind(t)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	argv := strings.Join(compiled.Argv, " ")
	if !strings.Contains(argv, "--counts-per-rev 131072") ||
		!strings.Contains(argv, "--gear-ratio 18") {
		t.Fatalf("A6 robot did not use motor counts plus software gear: %s", argv)
	}
	if strings.Contains(compiled.AxisProfiles, "a6ec_rotation_mode") ||
		!strings.Contains(compiled.AxisProfiles,
			"a6ec_encoder_position_tracking_mode|u16|8192|8|1|power_cycle") {
		t.Fatalf("A6 startup posture:\n%s", compiled.AxisProfiles)
	}
}

func TestA6RobotKeepsBatterylessLinearEvidenceVolatile(t *testing.T) {
	f := newRobotFixture(t, true)
	joint := asObject(asArray(f.definition.get("joints"))[0])
	joint.set("encoder_battery", "absent")
	f.bind(t)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(compiled.AxisProfiles, "encoder_battery=absent") {
		t.Fatalf("battery absence did not reach the runtime profile:\n%s", compiled.AxisProfiles)
	}
	if !strings.Contains(compiled.AxisProfiles, "encoder_battery_absent_alarm=29445|3984") {
		t.Fatalf("battery-absent warning policy did not reach the runtime profile:\n%s", compiled.AxisProfiles)
	}
	if strings.Contains(compiled.AxisProfiles, "single_turn_absolute") {
		t.Fatalf("battery absence resurrected retired mode 3:\n%s", compiled.AxisProfiles)
	}
}
