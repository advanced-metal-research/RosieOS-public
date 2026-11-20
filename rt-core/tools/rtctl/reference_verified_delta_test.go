package rtctl

import (
	"encoding/json"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

const verifiedDeltaPolicy = `{"method":"independent_absolute_delta_v1"}`

func TestVerifiedDeltaCompilerOptInAndIdentity(t *testing.T) {
	f := newRobotFixture(t, true)
	before, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	f.setAxis("reference_verified_delta_restore", decode([]byte(verifiedDeltaPolicy)))
	after, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	token := "reference_verified_delta_restore=independent_absolute_delta_v1\n"
	if strings.Count(after.AxisProfiles, token) != 1 || strings.Replace(after.AxisProfiles, token, "", 1) != before.AxisProfiles {
		t.Fatal("native policy delta not exact")
	}
	if !reflect.DeepEqual(before.CoordinateIdentitySHA256, after.CoordinateIdentitySHA256) {
		t.Fatal("immutable coordinate identities changed")
	}
	if before.SHA256 == after.SHA256 || before.MachineSHA256 == after.MachineSHA256 {
		t.Fatal("new admission not bound to configuration")
	}
}

// There is no delta cap. The removed maximum_delta_counts key is refused by
// name, so a capped configuration cannot silently compile as the new policy.
func TestVerifiedDeltaCompilerRejectsMalformedOrCappedAdmission(t *testing.T) {
	for _, policy := range []string{`null`, `{}`, `[]`, `{"method":"other"}`, `{"method":"independent_absolute_delta_v1","silent":true}`,
		`{"method":"independent_absolute_delta_v1","maximum_delta_counts":9102}`, `{"method":"independent_absolute_delta_v1","maximum_delta_counts":0}`} {
		f := newRobotFixture(t, true)
		f.setAxis("reference_verified_delta_restore", decode([]byte(policy)))
		if _, err := f.compile(); err == nil {
			t.Fatalf("accepted %s", policy)
		}
	}
	f := newRobotFixture(t, true)
	f.setAxis("reference_verified_delta_restore", decode([]byte(`{"method":"independent_absolute_delta_v1","maximum_delta_counts":9102}`)))
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "maximum_delta_counts was removed") {
		t.Fatal("capped admission not refused by name", err)
	}
}

// The current-pose zero path is removed: Home is established only by drive
// homing (HM35), so a machine or profile declaring it must not compile.
func TestCompilerRejectsCurrentPoseReferenceCapture(t *testing.T) {
	policy := `{"method":"independent_absolute_current_pose_v1","zero":"current","restore":"same_pose_only"}`
	f := newRobotFixture(t, true)
	f.setAxis("reference_capture", decode([]byte(policy)))
	if _, err := f.compile(); err == nil {
		t.Fatal("robot axis reference_capture accepted")
	}
	var machine map[string]any
	if err := json.Unmarshal(readTestFile(t, "config/machines/simulation/simulation-home.json"), &machine); err != nil {
		t.Fatal(err)
	}
	machine["axes"].([]any)[0].(map[string]any)["reference_capture"] = decode([]byte(policy))
	raw, _ := json.Marshal(machine)
	_, err := Compile(raw, filepath.Join(testRoot(t), "config/drives"))
	if err == nil || !strings.Contains(err.Error(), "establish Home with drive homing (HM35)") {
		t.Fatalf("flat axis reference_capture not refused with drive-homing reason: %v", err)
	}
}

func TestVerifiedDeltaRequiresPositiveIndependentPrerequisites(t *testing.T) {
	for _, trial := range []string{"missing_battery", "unknown_battery", "absent_battery", "source", "brake", "native_home"} {
		t.Run(trial, func(t *testing.T) {
			f := newRobotFixture(t, true)
			f.setAxis("reference_verified_delta_restore", decode([]byte(verifiedDeltaPolicy)))
			if strings.HasSuffix(trial, "battery") {
				joint := asObject(asArray(f.definition.get("joints"))[0])
				value := any(nil)
				if trial == "unknown_battery" {
					value = "unknown"
				}
				if trial == "absent_battery" {
					value = "absent"
				}
				joint.set("encoder_battery", value)
				f.bind(t)
				if _, err := f.compile(); err == nil {
					t.Fatal("invalid battery accepted")
				}
				return
			}
			path := filepath.Join(f.profiles, "test.json")
			raw, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			profile := asObject(decode(raw))
			if trial == "source" {
				block := asObject(profile.get("coordinate_evidence"))
				block.set("source", "pdo")
			}
			if trial == "brake" {
				block := asObject(profile.get("brake"))
				block.set("present", false)
			}
			if trial == "native_home" {
				profile.set("native_home", nil)
			}
			writeTestFile(t, path, canonical(profile), 0600)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted missing", trial)
			}
		})
	}
}
