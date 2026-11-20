package robotconfig

import (
	"encoding/json"
	"math"
	"strings"
	"testing"

	"rosieos/robotdesc"
)

func compiled(t *testing.T, description Description, machinePlanningCalibration string) Context {
	t.Helper()
	var bytes []byte
	source := SourceEmpty
	if machinePlanningCalibration != "" {
		bytes, source = []byte(machinePlanningCalibration), SourceTarget
	}
	settings, err := Resolve(description, bytes, source)
	if err != nil {
		t.Fatalf("Resolve: %v", err)
	}
	context, err := Compile(description, settings)
	if err != nil {
		t.Fatalf("Compile: %v", err)
	}
	return context
}

func plannerOf(t *testing.T, context Context) PlannerDocument {
	t.Helper()
	var planner PlannerDocument
	if err := json.Unmarshal(context.Files[ContextPlanner], &planner); err != nil {
		t.Fatalf("planner.json: %v", err)
	}
	return planner
}

// The corrected URDF is the author's file with the cell's numbers on the
// joints the cell touched, and nothing else changed: the declaration, the
// comment, the joints the cell left alone, character for character.
func TestCompileRewritesOnlyWhatTheCellTouched(t *testing.T) {
	model := loadedModel(t, nil)
	context := compiled(t, model, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a",`+
		`"limits":{"speed_ceiling_rad_s":0.5,"joints":{"J1":{"lower_rad":-2,"velocity_rad_s":0.05}}},`+
		`"kinematics":{"J1":{"xyz_m":[0.001,0,0],"rpy_rad":[0,0,0.01]}}}`)

	urdf := string(context.Files[ContextURDF])
	for _, kept := range []string{
		`<?xml version="1.0"?>`,
		`<!-- a description, reduced to what the cell check reads -->`,
		`<limit lower="-1.0" upper="1.0" effort="50" velocity="0.5"/>`, // J2, untouched
		`<joint name="mount" type="fixed"/>`,
		`upper="3.0" effort="100"`, // J1's other side keeps the author's text
	} {
		if !strings.Contains(urdf, kept) {
			t.Fatalf("the rewrite lost %q:\n%s", kept, urdf)
		}
	}
	parsed, err := ParseURDF(urdf)
	if err != nil {
		t.Fatalf("the rewritten URDF does not parse: %v\n%s", err, urdf)
	}
	j1 := parsed.Joints["J1"]
	if j1.XYZMetres != [3]float64{0.001, 0, 1} || math.Abs(j1.RPYRadians[2]-0.01) > 1e-12 {
		t.Fatalf("J1 origin = %v %v", j1.XYZMetres, j1.RPYRadians)
	}
	// Planning 0.09 under the model's 1.0 ceiling; the cell enforces 0.05
	// under its 0.5 ceiling; the planning rate follows the enforced one down.
	if j1.Lower != -2 || j1.Upper != 3 || j1.Velocity != 0.05 {
		t.Fatalf("J1 limit = %v..%v at %v", j1.Lower, j1.Upper, j1.Velocity)
	}

	planner := plannerOf(t, context)
	if planner.Schema != ContextSchema || planner.ModelID != "cell_a" {
		t.Fatalf("planner = %+v", planner)
	}
	if planner.Limits["J1"] != (PlannerRate{VelocityRadS: 0.05, EnforcedVelocityRadS: 0.05, AccelerationRadS2: 2.1}) {
		t.Fatalf("J1 rates = %+v", planner.Limits["J1"])
	}
	if planner.Bounds["J1"] != (JointBounds{-2, 3}) || planner.Bounds["J2"] != (JointBounds{-1, 1}) {
		t.Fatalf("bounds = %+v", planner.Bounds)
	}
	if planner.SpeedCeilingRadS != 0.5 || len(planner.Driven) != 1 || planner.Driven[0] != "J1" ||
		planner.Held["J2"] != 0 || planner.ResetPose["J1"] != 0 {
		t.Fatalf("planner = %+v", planner)
	}
	if planner.Torch == nil || planner.Torch.ToolFrame != "tool0" {
		t.Fatalf("torch = %+v", planner.Torch)
	}
	if planner.JointCorrections["J1"].XYZMetres != [3]float64{0.001, 0, 0} {
		t.Fatalf("corrections = %+v", planner.JointCorrections)
	}
	if planner.Identity != context.Identity || planner.Identity.MachinePlanningCalibrationSource != SourceTarget {
		t.Fatalf("identity = %+v", planner.Identity)
	}

	var identity map[string]string
	if err := json.Unmarshal(context.Files[ContextIdentity], &identity); err != nil {
		t.Fatal(err)
	}
	if identity["robot_description_sha256"] != model.Identity || !strings.HasPrefix(identity["base_urdf_sha256"], "sha256:") {
		t.Fatalf("identity.json = %v", identity)
	}
	// The originals do not travel: a reader with config.json beside
	// planner.json would read it.
	for _, absent := range []string{"config.json", Manifest} {
		if _, present := context.Files[absent]; present {
			t.Fatalf("the context carries %s", absent)
		}
	}
}

// Without a cell, the context is the model: the URDF byte for byte, the
// planning rates as enforced rates, the model's own holds.
func TestCompileWithoutACellIsTheModel(t *testing.T) {
	model := loadedModel(t, nil)
	context := compiled(t, model, "")
	// The one thing the model alone changes: a driven axis's velocity is the
	// rate the planner may plan at, not the URDF's rating. Every other byte
	// is the author's.
	if want := strings.Replace(testURDF, `velocity="2.0"`, `velocity="0.09"`, 1); string(context.Files[ContextURDF]) != want {
		t.Fatalf("the empty document changed more than the planning rate:\n%s", context.Files[ContextURDF])
	}
	planner := plannerOf(t, context)
	if planner.Limits["J1"] != (PlannerRate{VelocityRadS: 0.09, EnforcedVelocityRadS: 0.09, AccelerationRadS2: 2.1}) {
		t.Fatalf("J1 rates = %+v", planner.Limits["J1"])
	}
	if planner.Identity.MachinePlanningCalibrationSource != SourceEmpty || planner.SpeedCeilingRadS != 0 {
		t.Fatalf("planner = %+v", planner)
	}
}

// A correction on a joint that states no <origin> gets one, composed onto
// the identity origin the URDF implies.
func TestCompileInsertsAnOriginTheJointLacks(t *testing.T) {
	model := loadedModel(t, nil)
	context := compiled(t, model, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a",`+
		`"kinematics":{"J2":{"xyz_m":[0.001,0,0]}}}`)
	parsed, err := ParseURDF(string(context.Files[ContextURDF]))
	if err != nil {
		t.Fatal(err)
	}
	if parsed.Joints["J2"].XYZMetres != [3]float64{0.001, 0, 0} {
		t.Fatalf("J2 origin = %v\n%s", parsed.Joints["J2"].XYZMetres, context.Files[ContextURDF])
	}
	if parsed.Joints["J1"].XYZMetres != [3]float64{0, 0, 1} {
		t.Fatal("J1 moved")
	}
}

// The cell may hold a positioner joint and never an arm joint: the arm
// solver has six unknowns, and that rule is judged where the plan is made.
func TestCompileRefusesAHeldArmJoint(t *testing.T) {
	urdf := `<robot name="cell_a">
  <link name="world"/><link name="l1"/><link name="tool0"/><link name="table"/>
  <joint name="J1" type="revolute"><parent link="world"/><child link="l1"/>
    <limit lower="-3" upper="3" velocity="2"/></joint>
  <joint name="flange" type="fixed"><parent link="l1"/><child link="tool0"/></joint>
  <joint name="J7" type="revolute"><parent link="world"/><child link="table"/>
    <limit lower="-3" upper="3" velocity="1"/></joint>
</robot>`
	config := `{"schema":"rosie.robot-config.v1","model_id":"cell_a",
  "frames":{"base":"world","tool":"tool0","work":"table"},
  "planning":{"joints":{"J1":{"velocity_rad_s":0.5,"acceleration_rad_s2":1}}},
  "axes":{"driven":["J1","J7"],"held":{}},"reset_pose":{"J1":0}}`
	model := loadedModel(t, map[string]string{"robot.urdf": urdf, "config.json": config})

	positioner := compiled(t, model, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"held":{"J7":0.25}}}`)
	planner := plannerOf(t, positioner)
	if planner.Held["J7"] != 0.25 || len(planner.Driven) != 1 || planner.Driven[0] != "J1" {
		t.Fatalf("a held positioner left driven = %v held = %v", planner.Driven, planner.Held)
	}

	settings, err := Resolve(model, []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"held":{"J1":0.0}}}`), SourceTarget)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := Compile(model, settings); err == nil || !strings.Contains(err.Error(), "only the positioner may be held") {
		t.Fatalf("a held arm joint compiled: %v", err)
	}
}

// A description nobody configured has no rates, no roles and no reset pose:
// refused here, in words, rather than at the planner's gate.
func TestCompileRefusesADescriptionWithoutConfig(t *testing.T) {
	model := loadedModel(t, map[string]string{
		"config.json": "",
		Manifest:      `{"schema":"` + ManifestSchema + `","model_id":"cell_a","files":["robot.urdf"]}`,
	})
	if _, err := Compile(model, Settings{Identity: Identity{ModelID: "cell_a"}}); err == nil ||
		!strings.Contains(err.Error(), "no config.json") {
		t.Fatalf("err = %v", err)
	}
}

// The registered files travel: meshes and spheres verbatim, by their
// description-relative path.
func TestCompileCarriesTheRegisteredFiles(t *testing.T) {
	model := loadedModel(t, map[string]string{
		"spheres.json":    `{"cell_id":"cell_a","links":{}}`,
		"meshes/base.dae": "<COLLADA/>",
		Manifest: `{"schema":"` + ManifestSchema + `","model_id":"cell_a",` +
			`"files":["config.json","meshes/base.dae","robot.urdf","spheres.json"]}`,
	})
	context := compiled(t, model, "")
	if string(context.Files["meshes/base.dae"]) != "<COLLADA/>" || string(context.Files["spheres.json"]) != `{"cell_id":"cell_a","links":{}}` {
		t.Fatalf("files = %v", keysOfBytes(context.Files))
	}
	wire := context.Wire()
	if files, ok := wire["files"].(map[string]string); !ok || files["meshes/base.dae"] == "" {
		t.Fatalf("wire = %v", wire)
	}
}

// The same pair compiles to the same bytes: a context can be diffed against
// another compiled from the same inputs.
func TestCompileIsDeterministic(t *testing.T) {
	model := loadedModel(t, nil)
	document := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-2}}}}`
	first, second := compiled(t, model, document), compiled(t, model, document)
	for path, data := range first.Files {
		if string(second.Files[path]) != string(data) {
			t.Fatalf("%s differs between two compiles", path)
		}
	}
}

// The origin arithmetic is the Python twin's (`weldplan.robot_description`):
// a correction along the joint's x moves the origin along the parent's y when
// the origin is turned 90 degrees, and the rotations stack.
func TestComposeOriginMatchesThePythonTwin(t *testing.T) {
	xyz, rpy := composeOrigin([3]float64{1, 0, 0}, [3]float64{0, 0, math.Pi / 2},
		[3]float64{0.01, 0, 0}, [3]float64{0, 0, 0.1})
	for i, want := range [3]float64{1, 0.01, 0} {
		if math.Abs(xyz[i]-want) > 1e-12 {
			t.Fatalf("xyz = %v", xyz)
		}
	}
	if math.Abs(rpy[0]) > 1e-12 || math.Abs(rpy[1]) > 1e-12 || math.Abs(rpy[2]-(math.Pi/2+0.1)) > 1e-12 {
		t.Fatalf("rpy = %v", rpy)
	}
	for _, angles := range [][3]float64{{0.3, -0.4, 1.2}, {-2.0, 1.0, 0.5}, {0.1, math.Pi / 2, 0.2}} {
		again := robotdesc.RPYMatrix(matrixRPY(robotdesc.RPYMatrix(angles)))
		want := robotdesc.RPYMatrix(angles)
		for i := 0; i < 3; i++ {
			for j := 0; j < 3; j++ {
				if math.Abs(again[i][j]-want[i][j]) > 1e-9 {
					t.Fatalf("rpy %v does not round-trip", angles)
				}
			}
		}
	}
}

func TestSetAttributeReplacesInPlaceOrAppends(t *testing.T) {
	for _, row := range []struct{ tag, key, value, want string }{
		{`<origin xyz="0 0 1" rpy="0 0 0"/>`, "xyz", "1 2 3", `<origin xyz="1 2 3" rpy="0 0 0"/>`},
		{`<limit lower='-1' upper='1'/>`, "upper", "0.5", `<limit lower='-1' upper='0.5'/>`},
		{`<limit lower="-1" upper="1"/>`, "velocity", "2", `<limit lower="-1" upper="1" velocity="2"/>`},
		{`<limit lower="-1">`, "upper", "1", `<limit lower="-1" upper="1">`},
	} {
		if got := setAttribute(row.tag, row.key, row.value); got != row.want {
			t.Fatalf("setAttribute(%q, %s) = %q, want %q", row.tag, row.key, got, row.want)
		}
	}
}

func keysOfBytes(files map[string][]byte) []string {
	out := make([]string, 0, len(files))
	for key := range files {
		out = append(out, key)
	}
	return out
}
