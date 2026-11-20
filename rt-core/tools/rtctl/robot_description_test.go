package rtctl

import (
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/robotdesc"
)

// The fixture's description grows what a real one has beside its URDF: a
// config.json with caps and one mesh the URDF names, all registered.
func describeFixture(t *testing.T, f *robotFixture) string {
	t.Helper()
	urdf := strings.Replace(robotFixtureURDF, `<joint name="J1" type="revolute">`,
		`<link name="l1"><visual><geometry><mesh filename="package://fixture/meshes/base.dae"/></geometry></visual></link><joint name="J1" type="revolute">`, 1)
	writeTestFile(t, f.urdf, urdf, 0o644)
	writeTestFile(t, filepath.Join(f.dir, "config.json"), `{"schema":"rosie.robot-config.v1","model_id":"fixture","kinematic_correction_caps":{"xyz_m":0.002,"rpy_rad":0.0175},"planning":{"joints":{"J1":{"velocity_rad_s":0.25,"acceleration_rad_s2":1}}}}`, 0o644)
	writeTestFile(t, filepath.Join(f.dir, "meshes", "base.dae"), "<COLLADA/>", 0o644)
	f.pin(t)
	return f.dir
}

func resourcePaths(c Compiled) map[string]string {
	out := map[string]string{}
	if c.resources.Robot != nil {
		for _, resource := range c.resources.Robot.Resources {
			out[resource.Path] = resource.SHA256
		}
	}
	return out
}

// A CAD package label is not the robot id. The registered, identity-pinned
// mesh is still the only admitted resource; no package search is performed.
func TestRobotDescriptionMeshPackageLabel(t *testing.T) {
	for _, tc := range []struct {
		uri string
		ok  bool
	}{
		{"package://cad_export/meshes/base.dae", true},
		{"package://cad_export/meshes/missing.dae", false},
		{"package://cad_export/meshes/../meshes/base.dae", false},
		{"package://cad_export/robot.urdf", false},
		{"file:///meshes/base.dae", false},
		{"https://cad_export/meshes/base.dae", false},
	} {
		t.Run(tc.uri, func(t *testing.T) {
			f := newRobotFixture(t, false)
			describeFixture(t, &f)
			raw, err := os.ReadFile(f.urdf)
			if err != nil {
				t.Fatal(err)
			}
			writeTestFile(t, f.urdf, strings.ReplaceAll(string(raw), "package://fixture/meshes/base.dae", tc.uri), 0o644)
			f.pin(t)
			compiled, err := f.compile()
			if (err == nil) != tc.ok {
				t.Fatalf("compile %q: %v (want success %v)", tc.uri, err, tc.ok)
			}
			if tc.ok && resourcePaths(compiled)["meshes/base.dae"] == "" {
				t.Fatal("registered mesh missing from resources")
			}
		})
	}
}

// A machine that pins a description serves it whole: the manifest and every
// registered file, by description-relative path, plus the machine's
// machine_planning_calibration.json -- the empty document when the machine pins none.
func TestRobotDescriptionIsSnapshottedIntoTheResources(t *testing.T) {
	f := newRobotFixture(t, false)
	describeFixture(t, &f)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.HasPrefix(compiled.Robot.Description.RobotDescriptionSHA256, "sha256:") || compiled.Robot.Description.Path != "robot_description/robots/fixture" {
		t.Fatalf("binding = %+v", compiled.Robot)
	}
	paths := resourcePaths(compiled)
	for _, want := range []string{robotdesc.Manifest, robotdesc.Definition, "robot.urdf", "config.json", "meshes/base.dae", "machine_planning_calibration.json"} {
		if paths[want] == "" {
			t.Fatalf("resource %s missing; have %v", want, paths)
		}
	}
	if len(paths) != 6 {
		t.Fatalf("resources = %v", paths)
	}
	served := compiled.resources.Robot
	if served.ModelID != "fixture" || served.RobotDescriptionSHA256 != compiled.Robot.Description.RobotDescriptionSHA256 || served.MachinePlanningCalibrationSHA256 != compiled.Robot.MachinePlanningCalibrationSHA256 {
		t.Fatalf("described as %+v", served)
	}
	empty := string(compiled.resourceData[paths["machine_planning_calibration.json"]])
	if empty != string(robotdesc.EmptyMachinePlanningCalibrationBytes("fixture")) || compiled.Robot.MachinePlanningCalibrationSHA256 != robotdesc.MachinePlanningCalibrationSHA256([]byte(empty)) {
		t.Fatalf("machine_planning_calibration.json = %q, sha %s", empty, compiled.Robot.MachinePlanningCalibrationSHA256)
	}
}

// The description is checked, not trusted: a file that moved under the pin,
// a URDF the manifest does not register, a mesh the manifest does not
// register, a definition for another robot.
func TestRobotDescriptionRefusals(t *testing.T) {
	var refusal *RobotDefinitionError
	{
		f := newRobotFixture(t, false)
		dir := describeFixture(t, &f)
		writeTestFile(t, filepath.Join(dir, "config.json"), `{"schema":"rosie.robot-config.v1","model_id":"fixture"}`, 0o644)
		if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "robot_description_mismatch" {
			t.Fatalf("a description that moved under its pin compiled: %v", err)
		}
	}
	{
		f := newRobotFixture(t, false)
		describeFixture(t, &f)
		writeTestFile(t, filepath.Join(f.dir, robotdesc.Manifest), `{"schema":"`+robotdesc.ManifestSchema+`","model_id":"fixture","files":["config.json","meshes/base.dae","`+robotdesc.Definition+`"]}`, 0o644)
		identity, _ := robotdesc.RobotDescriptionSHA256(f.dir)
		ref := asObject(f.machine.get("robot"))
		setRobotDescriptionPin(ref, "sha256", identity)
		f.machine.set("robot", ref)
		if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "robot_description_mismatch" {
			t.Fatalf("a URDF outside the manifest compiled: %v", err)
		}
	}
	{
		f := newRobotFixture(t, false)
		describeFixture(t, &f)
		urdf := strings.Replace(robotFixtureURDF, `<joint name="J1" type="revolute">`,
			`<link name="l1"><visual><geometry><mesh filename="package://fixture/meshes/other.dae"/></geometry></visual></link><joint name="J1" type="revolute">`, 1)
		writeTestFile(t, f.urdf, urdf, 0o644)
		f.pin(t)
		if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "not a file the description registers") {
			t.Fatalf("a mesh outside the manifest compiled: %v", err)
		}
	}
	{
		f := newRobotFixture(t, false)
		f.definition.set("robot_id", "another")
		f.bind(t)
		if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "robot_description_mismatch" {
			t.Fatalf("a definition for another robot compiled: %v", err)
		}
	}
	{
		f := newRobotFixture(t, false)
		ref := asObject(f.machine.get("robot"))
		setRobotDescriptionPin(ref, "sha256", "sha256:"+strings.Repeat("0", 64))
		f.machine.set("robot", ref)
		if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "robot_description_mismatch" {
			t.Fatalf("a stale pin compiled: %v", err)
		}
	}
}

// A description with no geometry -- a bench robot -- is a description all
// the same: identity, definition, served whole; and there is nothing for a
// machine_planning_calibration.json to deviate from.
func TestRobotDescriptionWithoutGeometry(t *testing.T) {
	f := newRobotFixture(t, false)
	limits := asObject(f.axis().get("limits"))
	limits.set("max_velocity", json.Number("0.25"))
	limits.set("jog_acceleration", json.Number("1"))
	f.setAxis("limits", limits)
	if err := os.Remove(filepath.Join(f.dir, "config.json")); err != nil {
		t.Fatal(err)
	}
	joint := f.joint()
	joint.set("kind", "external")
	joint.set("cartesian_participates", false)
	joint.set("urdf_joint", nil)
	joint.set("mechanical_travel", object{{"min", json.Number("-3")}, {"max", json.Number("3")}, {"min_source", "unverified test"}, {"max_source", "unverified test"}})
	f.definition.set("joints", []any{joint})
	if err := os.Remove(f.urdf); err != nil {
		t.Fatal(err)
	}
	f.bind(t)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	paths := resourcePaths(compiled)
	if len(paths) != 3 || paths[robotdesc.Manifest] == "" || paths[robotdesc.Definition] == "" || paths["machine_planning_calibration.json"] == "" {
		t.Fatalf("resources = %v", paths)
	}
	writeTestFile(t, filepath.Join(f.root, "config", "cells", "bench.machine_planning_calibration.json"), `{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture"}`, 0o644)
	ref := asObject(f.machine.get("robot"))
	ref.set("machine_planning_calibration", object{{"path", "config/cells/bench.machine_planning_calibration.json"}, {"sha256", identityDigest(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture"}`)}, {"source", "s"}})
	f.machine.set("robot", ref)
	var refusal *RobotDefinitionError
	if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "machine_planning_calibration_invalid" {
		t.Fatalf("a machine_planning_calibration.json for a robot with no geometry compiled: %v", err)
	}
}

// The machine's machine_planning_calibration.json is judged by the contract's rules against the
// description it pins, and served beside it under its own identity.
func TestRobotMachinePlanningCalibrationIsJudgedAndServed(t *testing.T) {
	f := newRobotFixture(t, false)
	describeFixture(t, &f)
	narrowed := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture","limits":{"joints":{"J1":{"lower_rad":-1.5}}},"kinematics":{"J1":{"xyz_m":[0.001,0,0]}}}`
	writeTestFile(t, filepath.Join(f.root, "config", "cells", "bench.machine_planning_calibration.json"), narrowed, 0o644)
	ref := asObject(f.machine.get("robot"))
	ref.set("machine_planning_calibration", object{{"path", "config/cells/bench.machine_planning_calibration.json"}, {"sha256", identityDigest(narrowed)}, {"source", "unverified test cell"}})
	f.machine.set("robot", ref)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	paths := resourcePaths(compiled)
	if string(compiled.resourceData[paths["machine_planning_calibration.json"]]) != narrowed || compiled.Robot.MachinePlanningCalibrationSHA256 != robotdesc.MachinePlanningCalibrationSHA256([]byte(narrowed)) || compiled.resources.Robot.MachinePlanningCalibrationSHA256 != compiled.Robot.MachinePlanningCalibrationSHA256 {
		t.Fatalf("machine_planning_calibration.json not served as pinned: %s", compiled.Robot.MachinePlanningCalibrationSHA256)
	}

	var refusal *RobotDefinitionError
	widened := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture","limits":{"joints":{"J1":{"lower_rad":-3}}}}`
	writeTestFile(t, filepath.Join(f.root, "config", "cells", "bench.machine_planning_calibration.json"), widened, 0o644)
	ref.set("machine_planning_calibration", object{{"path", "config/cells/bench.machine_planning_calibration.json"}, {"sha256", identityDigest(widened)}, {"source", "s"}})
	f.machine.set("robot", ref)
	if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "machine_planning_calibration_invalid" {
		t.Fatalf("a widening machine_planning_calibration.json compiled: %v", err)
	}
	ref.set("machine_planning_calibration", object{{"path", "config/cells/bench.machine_planning_calibration.json"}, {"sha256", strings.Repeat("0", 64)}, {"source", "s"}})
	f.machine.set("robot", ref)
	if _, err := f.compile(); !errors.As(err, &refusal) || refusal.Reason != "robot_hash_mismatch" {
		t.Fatalf("a machine_planning_calibration.json that moved under its pin compiled: %v", err)
	}
}
