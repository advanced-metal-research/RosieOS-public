package robotdesc

import (
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

const testURDF = `<?xml version="1.0"?>
<!-- a description, reduced to what the cell check reads -->
<robot name="cell_a">
  <link name="world"/>
  <joint name="J1" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <limit lower="-3.0" upper="3.0" effort="100" velocity="2.0"/>
  </joint>
  <joint name="J2" type="revolute">
    <limit lower="-1.0" upper="1.0" effort="50" velocity="0.5"/>
  </joint>
  <joint name="mount" type="fixed"/>
</robot>
`

const testConfig = `{"schema":"rosie.robot-config.v1","model_id":"cell_a",
  "kinematic_correction_caps":{"xyz_m":0.002,"rpy_rad":0.0175},
  "planning":{"speed_ceiling_rad_s":1.0,
              "joints":{"J1":{"velocity_rad_s":0.09,"acceleration_rad_s2":2.1}}},
  "axes":{"driven":["J1"],"held":{"J2":0.0}},
  "reset_pose":{"J1":0.0},
  "torch":{"tool_frame":"tool0","electrode_axis":[1,0,0]}}`

// loadedModel writes a model whose files the test owns, so nothing here moves
// when a real robot is worked on.
func loadedModel(t *testing.T, files map[string]string) Description {
	t.Helper()
	root := t.TempDir()
	dir := filepath.Join(root, "robot_description", "cell_a")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	write := map[string]string{
		"robot.urdf":  testURDF,
		"config.json": testConfig,
		Manifest: `{"schema":"` + ManifestSchema + `","model_id":"cell_a",` +
			`"files":["config.json","robot.urdf"]}`,
	}
	for name, text := range files {
		write[name] = text
	}
	for name, text := range write {
		if text == "" {
			continue
		}
		target := filepath.Join(dir, filepath.FromSlash(name))
		if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(target, []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	loaded, err := At(filepath.Join(root, "robot_description")).Load("cell_a")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	return loaded
}

func TestLoadReadsTheURDFsJointsAndTheDescriptionsCaps(t *testing.T) {
	loaded := loadedModel(t, nil)
	if loaded.URDF.RobotName != "cell_a" || len(loaded.URDF.Joints) != 3 {
		t.Fatalf("urdf = %+v", loaded.URDF)
	}
	j1 := loaded.URDF.Joints["J1"]
	if !j1.Revolute() || j1.Lower != -3 || j1.Upper != 3 || j1.Velocity != 2 {
		t.Fatalf("J1 = %+v", j1)
	}
	if mount := loaded.URDF.Joints["mount"]; mount.Revolute() || mount.HasLimit {
		t.Fatalf("mount = %+v", mount)
	}
	if fastest, any := loaded.URDF.FastestRevolute(); !any || fastest != 2 {
		t.Fatalf("fastest = %v %v", fastest, any)
	}
	if loaded.Config.Caps.XYZMetres != 0.002 || loaded.Config.Caps.RPYRadians != 0.0175 {
		t.Fatalf("caps = %+v", loaded.Config.Caps)
	}
	if rate := loaded.Config.PlanningJoints["J1"]; rate.VelocityRadS != 0.09 {
		t.Fatalf("planning J1 = %+v", rate)
	}
	if !strings.HasPrefix(loaded.Identity, "sha256:") || len(loaded.Identity) != 71 {
		t.Fatalf("identity = %q", loaded.Identity)
	}
}

// A model id arrives from a browser. It names a directory and nothing else.
func TestLoadRefusesAModelIDThatIsAPath(t *testing.T) {
	for _, id := range []string{"", "../secrets", "cell_a/robot.urdf", "cell.a"} {
		if _, err := At(t.TempDir()).Load(id); err == nil {
			t.Fatalf("%q was loaded", id)
		}
	}
}

func TestLoadRefusesADescriptionWhoseURDFNamesAnotherRobot(t *testing.T) {
	root := t.TempDir()
	dir := filepath.Join(root, "robot_description", "cell_a")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		"robot.urdf": `<robot name="cell_b"/>`,
		Manifest: `{"schema":"` + ManifestSchema + `","model_id":"cell_a",` +
			`"files":["robot.urdf"]}`,
	} {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	_, err := At(filepath.Join(root, "robot_description")).Load("cell_a")
	if err == nil || !strings.Contains(err.Error(), "names robot") {
		t.Fatalf("err = %v", err)
	}
}

// A config.json that claims a rate the robot is not rated for would have the
// planner parametrize a motion the cell then refuses.
func TestLoadRefusesAPlanningRateAboveTheRating(t *testing.T) {
	root := t.TempDir()
	dir := filepath.Join(root, "robot_description", "cell_a")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		"robot.urdf": testURDF,
		"config.json": `{"schema":"rosie.robot-config.v1","model_id":"cell_a",` +
			`"planning":{"joints":{"J1":{"velocity_rad_s":9,"acceleration_rad_s2":1}}}}`,
		Manifest: `{"schema":"` + ManifestSchema + `","model_id":"cell_a",` +
			`"files":["config.json","robot.urdf"]}`,
	} {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	_, err := At(filepath.Join(root, "robot_description")).Load("cell_a")
	if err == nil || !strings.Contains(err.Error(), "never raise it") {
		t.Fatalf("err = %v", err)
	}
}

func TestEmptyMachinePlanningCalibrationBytesAreTheDaemonsOwn(t *testing.T) {
	want := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}` + "\n"
	if got := string(EmptyMachinePlanningCalibrationBytes("cell_a")); got != want {
		t.Fatalf("empty = %q", got)
	}
}

func TestResolveWithoutACellIsTheEmptyDocument(t *testing.T) {
	loaded := loadedModel(t, nil)
	settings, err := Resolve(loaded, nil, "")
	if err != nil {
		t.Fatal(err)
	}
	if settings.Identity.MachinePlanningCalibrationSource != SourceEmpty {
		t.Fatalf("source = %q", settings.Identity.MachinePlanningCalibrationSource)
	}
	if settings.Identity.MachinePlanningCalibrationSHA256 != MachinePlanningCalibrationSHA256(EmptyMachinePlanningCalibrationBytes("cell_a")) {
		t.Fatalf("machine_planning_calibration_sha256 = %q", settings.Identity.MachinePlanningCalibrationSHA256)
	}
	if settings.JointLimits != nil || settings.JointCorrections != nil || settings.Held != nil {
		t.Fatalf("the empty document settled something: %+v", settings)
	}
}

func TestResolveSettlesEveryNumberTheWorkerNeeds(t *testing.T) {
	loaded := loadedModel(t, nil)
	document := []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{
      "speed_ceiling_rad_s":0.4,
      "joints":{"J1":{"lower_rad":-2.0,"velocity_rad_s":0.05},"J2":{"upper_rad":0.5}},
      "held":{"J2":0.25}},
      "kinematics":{"J1":{"xyz_m":[0.001,0,-0.002],"rpy_rad":[0,0.01,0]}}}`)
	settings, err := Resolve(loaded, document, SourceTarget)
	if err != nil {
		t.Fatal(err)
	}
	if settings.Identity.MachinePlanningCalibrationSHA256 != MachinePlanningCalibrationSHA256(document) ||
		settings.Identity.MachinePlanningCalibrationSource != SourceTarget ||
		settings.Identity.RobotDescriptionSHA256 != loaded.Identity {
		t.Fatalf("identity = %+v", settings.Identity)
	}
	// A field the cell left out comes back as the URDF's own number: the
	// worker applies what it is given and has no defaults of its own.
	j1 := settings.JointLimits["J1"]
	if j1.LowerRad != -2 || j1.UpperRad != 3 || j1.VelocityRad == nil || *j1.VelocityRad != 0.05 {
		t.Fatalf("J1 = %+v", j1)
	}
	j2 := settings.JointLimits["J2"]
	if j2.LowerRad != -1 || j2.UpperRad != 0.5 || j2.VelocityRad != nil {
		t.Fatalf("J2 = %+v", j2)
	}
	if settings.Held["J2"] != 0.25 || settings.SpeedCeilingRadS != 0.4 {
		t.Fatalf("held/ceiling = %v %v", settings.Held, settings.SpeedCeilingRadS)
	}
	correction := settings.JointCorrections["J1"]
	if correction.XYZMetres != [3]float64{0.001, 0, -0.002} ||
		correction.RPYRadians != [3]float64{0, 0.01, 0} {
		t.Fatalf("correction = %+v", correction)
	}
}

// Every bounded-deviation rule, each as the refusal an operator reads.
func TestResolveRefusesAnythingButABoundedDeviation(t *testing.T) {
	loaded := loadedModel(t, nil)
	cases := map[string]struct{ document, says string }{
		"unknown top key":                  {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limit":{}}`, "unknown field"},
		"unknown limit key":                {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joint":{}}}`, "unknown field"},
		"unknown joint key":                {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower":1}}}}`, "unknown field"},
		"wrong schema":                     {`{"schema":"rosie.cell.v0","model_id":"cell_a"}`, "schema"},
		"another model":                    {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_b"}`, "is for model"},
		"not json":                         {`{`, "machine_planning_calibration.json"},
		"no such joint":                    {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J9":{"lower_rad":0}}}}`, "does not have"},
		"not revolute":                     {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"mount":{"lower_rad":0}}}}`, "not revolute"},
		"widens lower":                     {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-4}}}}`, "widens the URDF lower limit"},
		"widens upper":                     {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"upper_rad":4}}}}`, "widens the URDF upper limit"},
		"widens velocity":                  {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"velocity_rad_s":3}}}}`, "the URDF velocity"},
		"velocity not above zero":          {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"velocity_rad_s":0}}}}`, "the URDF velocity"},
		"crossed bounds":                   {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":1,"upper_rad":0}}}}`, "leaves lower"},
		"ceiling above rating":             {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"speed_ceiling_rad_s":3}}`, "fastest joint"},
		"ceiling not positive":             {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"speed_ceiling_rad_s":0}}`, "must be positive"},
		"held outside limits":              {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"held":{"J2":5}}}`, "outside the joint's limits"},
		"held outside the narrowed limits": {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J2":{"upper_rad":0.1}},"held":{"J2":0.5}}}`, "outside the joint's limits"},
		"correction over cap":              {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"xyz_m":[0.003,0,0]}}}`, "exceeds the description's cap"},
		"correction over rpy cap":          {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"rpy_rad":[0,0,0.02]}}}`, "exceeds the description's cap"},
		"correction wrong length":          {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"xyz_m":[0,0]}}}`, "three numbers"},
		"correction on no such joint":      {`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J9":{}}}`, "does not have"},
	}
	for name, test := range cases {
		t.Run(name, func(t *testing.T) {
			_, err := Resolve(loaded, []byte(test.document), SourceTarget)
			if err == nil {
				t.Fatalf("%s was accepted", name)
			}
			if !strings.Contains(err.Error(), test.says) {
				t.Fatalf("err = %v, want it to mention %q", err, test.says)
			}
		})
	}
}

// Without a config.json there is no agreed cap, so no correction is bounded.
func TestResolveRefusesEveryCorrectionWithoutCaps(t *testing.T) {
	root := t.TempDir()
	dir := filepath.Join(root, "robot_description", "cell_a")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		"robot.urdf": testURDF,
		Manifest: `{"schema":"` + ManifestSchema + `","model_id":"cell_a",` +
			`"files":["robot.urdf"]}`,
	} {
		target := filepath.Join(dir, filepath.FromSlash(name))
		if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(target, []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	loaded, err := At(filepath.Join(root, "robot_description")).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	if loaded.Config.Present {
		t.Fatal("a description with no config.json reported one")
	}
	_, err = Resolve(loaded, []byte(
		`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"xyz_m":[0.0001,0,0]}}}`),
		SourceTarget)
	if err == nil || !strings.Contains(err.Error(), "cap 0") {
		t.Fatalf("err = %v", err)
	}
}

// Saying a cell was read when it was not is the one thing machine_planning_calibration_source exists
// to prevent.
func TestResolveRefusesAnInconsistentSource(t *testing.T) {
	loaded := loadedModel(t, nil)
	if _, err := Resolve(loaded, nil, SourceTarget); err == nil {
		t.Fatal(`"target" without a machine_planning_calibration.json was accepted`)
	}
	if _, err := Resolve(loaded, nil, "cache"); err == nil {
		t.Fatal("an unknown source was accepted")
	}
}

func TestParseURDFRefusesWhatItCannotRead(t *testing.T) {
	for name, text := range map[string]string{
		"no root name":    `<robot/>`,
		"wrong root":      `<link name="x"/>`,
		"unclosed tag":    `<robot name="a"><joint name="J1" type="revolute"></robot>`,
		"unquoted value":  `<robot name=a/>`,
		"twice defined":   `<robot name="a"><joint name="J1" type="revolute"/><joint name="J1" type="revolute"/></robot>`,
		"bad limit":       `<robot name="a"><joint name="J1" type="revolute"><limit lower="x"/></joint></robot>`,
		"unterminated":    `<robot name="a"><!-- forever`,
		"joint with none": `<robot name="a"><joint type="revolute"/></robot>`,
	} {
		t.Run(name, func(t *testing.T) {
			if _, err := ParseURDF(text); err == nil {
				t.Fatalf("%s parsed", name)
			}
		})
	}
}

// A work surface is a point stated in its PARENT LINK's axes. The catalogue
// reports it in the base frame, so the offset turns with the link: a table
// yawed a quarter turn at the zero pose carries its surface point sideways,
// not forwards. The planner adds the same point as a fixed child of the link
// (weldplan.urdf.read_urdf), and the two must land on the same spot or a
// part authored from the catalogue is placed where the planner does not
// read it.
func TestCatalogueWorkSurfaceTurnsWithItsLink(t *testing.T) {
	loaded := loadedModel(t, map[string]string{
		"robot.urdf": `<robot name="cell_a">
  <link name="world"/><link name="table"/><link name="table_top"/>
  <joint name="world_to_table" type="fixed">
    <parent link="world"/><child link="table"/>
    <origin xyz="2 0 0.5" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="J1" type="revolute">
    <parent link="world"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <limit lower="-3.0" upper="3.0" effort="100" velocity="2.0"/>
  </joint>
</robot>`,
		"config.json": `{"schema":"rosie.robot-config.v1","model_id":"cell_a",
  "frames":{"base":"world","tool":"arm","work":"table_top",
            "work_surface":{"parent":"table","xyz_m":[1,0,0.1]}},
  "axes":{"driven":["J1"]}}`,
	})
	robot := loaded.Robot()
	// The table's +X points along world +Y after a +90 degree yaw, so one
	// metre along the table lands one metre along world Y, and the 0.1 m up
	// stays up.
	want := [3]float64{2, 1, 0.6}
	for axis := 0; axis < 3; axis++ {
		if got := robot.Frames.WorkWorldM[axis]; math.Abs(got-want[axis]) > 1e-9 {
			t.Fatalf("work_world_m = %v, want %v", robot.Frames.WorkWorldM, want)
		}
	}
}

// The store is a place: what is in it is what can be planned for, and a
// directory that is not a model is passed over rather than refused.
func TestListNamesEveryModelInTheStore(t *testing.T) {
	root := t.TempDir()
	for _, name := range []string{"rosie_b", "rosie_a"} {
		if err := os.MkdirAll(filepath.Join(root, name), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(root, name, "robot.urdf"), []byte(testURDF), 0o644); err != nil {
			t.Fatal(err)
		}
		// A model is its manifest; a directory of files with none is not one.
		if err := os.WriteFile(filepath.Join(root, name, Manifest), []byte(`{"schema":"`+ManifestSchema+`","model_id":"`+name+`","files":["robot.urdf"]}`), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	if err := os.WriteFile(filepath.Join(root, "rosie_b", Manifest), []byte(`{"schema":"`+ManifestSchema+`","model_id":"rosie_b","files":["robot.urdf"]}`), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(filepath.Join(root, "notes"), 0o755); err != nil {
		t.Fatal(err)
	}
	models, err := At(root).List()
	if err != nil {
		t.Fatal(err)
	}
	if len(models) != 2 || models[0] != "rosie_a" || models[1] != "rosie_b" {
		t.Fatalf("models = %v", models)
	}
	if _, err := At(filepath.Join(root, "nowhere")).List(); err == nil {
		t.Fatal("a store that is not there listed models")
	}
}

// A description without geometry -- a bench robot, only its manifest and
// rt-core definition -- is listed, loads with its identity, and says it
// cannot be planned against.
func TestADescriptionWithoutGeometryHasAnIdentityAndNothingToPlan(t *testing.T) {
	root := t.TempDir()
	dir := filepath.Join(root, "bench_one")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		Definition: `{"schema":"rosie.robot-definition.v1","robot_id":"bench_one"}`,
		Manifest:   `{"schema":"` + ManifestSchema + `","model_id":"bench_one","files":["` + Definition + `"]}`,
	} {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	models, err := At(root).List()
	if err != nil || len(models) != 1 || models[0] != "bench_one" {
		t.Fatalf("models = %v, %v", models, err)
	}
	description, err := At(root).Load("bench_one")
	if err != nil {
		t.Fatal(err)
	}
	identity, _ := RobotDescriptionSHA256(dir)
	if description.HasGeometry() || description.Identity != identity || description.ModelID != "bench_one" {
		t.Fatalf("loaded %+v", description)
	}
}
