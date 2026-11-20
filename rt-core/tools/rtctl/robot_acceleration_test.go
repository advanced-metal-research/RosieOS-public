package rtctl

import (
	"encoding/json"
	"math"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

func TestRobotAccelerationComesOnlyFromDescription(t *testing.T) {
	f := newRobotFixture(t, true)
	// Even a legacy drive descriptor cannot impose a second joint limit.
	profile := asObject(decode(readTestFile(t, "config/drives/a6ec.json")))
	profile.set("max_acceleration", json.Number("0.001"))
	writeTestFile(t, filepath.Join(f.profiles, "test.json"), canonical(profile), 0o644)
	before, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(before.AxisProfiles, "max_acceleration=1.0\n") || !strings.Contains(before.AxisProfiles, "tracking=1.0|") {
		t.Fatal("drive overrode robot plan/jog acceleration", before.AxisProfiles)
	}
	writeTestFile(t, filepath.Join(f.dir, "config.json"), `{"schema":"rosie.robot-config.v1","model_id":"fixture","planning":{"joints":{"J1":{"velocity_rad_s":0.25,"acceleration_rad_s2":2.5}}}}`, 0o644)
	f.pin(t)
	after, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(after.AxisProfiles, "max_acceleration=2.5\n") || !strings.Contains(after.AxisProfiles, "tracking=2.5|") || before.MachineSHA256 == after.MachineSHA256 {
		t.Fatal("robot acceleration did not reach native plan, jog and identity", after.AxisProfiles)
	}
}

func TestRobotAccelerationCannotFallBackToDrive(t *testing.T) {
	f := newRobotFixture(t, true)
	writeTestFile(t, filepath.Join(f.dir, "config.json"), `{"schema":"rosie.robot-config.v1","model_id":"fixture"}`, 0o644)
	f.pin(t)
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "robot_acceleration_required") {
		t.Fatalf("missing robot acceleration did not fail closed: %v", err)
	}
}

func TestRobotMotionLimitsCannotBeRedeclaredByMachine(t *testing.T) {
	for _, field := range []string{"max_velocity", "jog_acceleration"} {
		for _, value := range []string{"0.001", "1", "100", "null"} {
			t.Run(field+"/"+value, func(t *testing.T) {
				f := newRobotFixture(t, true)
				limits := asObject(f.axis().get("limits"))
				limits.set(field, decode([]byte(value)))
				f.setAxis("limits", limits)
				if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "robot_urdf_limits_required") {
					t.Fatalf("accepted %s=%s: %v", field, value, err)
				}
			})
		}
	}
	f := newRobotFixture(t, true)
	f.setAxis("max_acceleration", json.Number("100"))
	if _, err := f.compile(); err == nil {
		t.Fatal("accepted machine plan acceleration on a robot-bound axis")
	}
}

func TestRobotDriveSpeedCapUsesURDFAndMotorFrame(t *testing.T) {
	for _, nativeRatio := range []bool{false, true} {
		t.Run(strconv.FormatBool(nativeRatio), func(t *testing.T) {
			f := newRobotFixture(t, nativeRatio)
			check := func(velocity float64) {
				t.Helper()
				compiled, err := f.compile()
				if err != nil {
					t.Fatal(err)
				}
				// Synthetic fixture: 18 motor turns/joint turn and 131072 encoder
				// counts/motor turn, irrespective of electronic gearing mode.
				want := strconv.FormatUint(uint64(math.Round(velocity*18*131072/(2*math.Pi))), 10)
				for i, arg := range compiled.Argv {
					if arg == "--drive-speed-limit-counts-per-s" {
						if compiled.Argv[i+1] != want {
							t.Fatalf("drive cap %s, want %s", compiled.Argv[i+1], want)
						}
						return
					}
				}
				t.Fatal("missing explicit motor-frame speed cap")
			}
			check(0.5)
			writeTestFile(t, f.urdf, strings.ReplaceAll(robotFixtureURDF, `velocity="0.5"`, `velocity="0.4"`), 0o644)
			f.pin(t)
			check(0.4)
			for _, field := range []string{"max_motor_rpm", "drive_speed_limit_motor_rpm"} {
				f.machine.set(field, json.Number("3000"))
				if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "robot_urdf_limits_required") {
					t.Fatalf("machine motor cap accepted: %v", err)
				}
				f.machine = asObject(replace(f.machine, []string{field}, nil, true))
			}
		})
	}
}
