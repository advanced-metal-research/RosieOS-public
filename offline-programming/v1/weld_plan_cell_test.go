package main

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/robotconfig"
	"rosieos/offline-programming/v1/internal/seam"
)

type seamWorkerFunc func(context.Context, []byte) (seam.Result, error)

func (function seamWorkerFunc) ExecuteJSON(ctx context.Context, request []byte) (seam.Result, error) {
	return function(ctx, request)
}

// A target that answers with the machine_planning_calibration.json a test hands it, or refuses, and
// serves the description directory it is pointed at.
type fakeCellTarget struct {
	reachable bool
	body      *denseexec.CellBody
	err       error
	// description is the directory served by `Description`; "" refuses the
	// way a daemon from before the context does (unknown kind).
	description string
	served      int
}

func (f *fakeCellTarget) Available() bool { return f.reachable }

func (f *fakeCellTarget) Cell(context.Context) (*denseexec.CellBody, error) {
	if f.err != nil {
		return nil, f.err
	}
	return f.body, nil
}

func (f *fakeCellTarget) Description(context.Context) (*denseexec.DescriptionBody, error) {
	f.served++
	if f.description == "" {
		return nil, errors.New("dense ingest: request_invalid: unknown kind description")
	}
	files := map[string][]byte{}
	err := filepath.WalkDir(f.description, func(path string, entry os.DirEntry, err error) error {
		if err != nil || entry.IsDir() {
			return err
		}
		relative, _ := filepath.Rel(f.description, path)
		data, err := os.ReadFile(path)
		files[filepath.ToSlash(relative)] = data
		return err
	})
	if err != nil {
		return nil, err
	}
	return &denseexec.DescriptionBody{Robot: f.body.Robot, Files: files}, nil
}

func targetServing(document string) *fakeCellTarget {
	return &fakeCellTarget{reachable: true, body: &denseexec.CellBody{
		Robot:                            denseexec.RobotCell{Configured: true, Valid: true, ModelID: "cell_a"},
		MachinePlanningCalibrationBase64: base64.StdEncoding.EncodeToString([]byte(document)),
		ExecutionLimits: robotconfig.ExecutionLimits{
			"J1": {PositionUnit: "rad", LowerRad: -3, UpperRad: 3, VelocityRadS: 2},
			"J2": {PositionUnit: "rad", LowerRad: -1, UpperRad: 1, VelocityRadS: 1},
		},
	}}
}

// A repo root with one description, manifest and all, so the identity is of a
// directory the test owns. `config.json` states the caps a correction must fit.
func cellRepo(t *testing.T) (root string, identity string) {
	t.Helper()
	root = t.TempDir()
	dir := filepath.Join(root, "robot_description", "robots", "cell_a")
	if err := os.MkdirAll(filepath.Join(dir, "meshes"), 0o755); err != nil {
		t.Fatal(err)
	}
	urdf := `<robot name="cell_a">` +
		`<joint name="J1" type="revolute">` +
		`<limit lower="-3" upper="3" effort="100" velocity="2"/></joint>` +
		`<joint name="J2" type="revolute">` +
		`<limit lower="-1" upper="1" effort="100" velocity="1"/></joint>` +
		`<joint name="base" type="fixed"/></robot>`
	config := `{"schema":"rosie.robot-config.v1","model_id":"cell_a",` +
		`"kinematic_correction_caps":{"xyz_m":0.002,"rpy_rad":0.0175},` +
		`"planning":{"joints":{"J1":{"velocity_rad_s":0.09,"acceleration_rad_s2":2.1}}},` +
		`"axes":{"driven":["J1","J2"],"held":{}},"reset_pose":{"J1":0.0},` +
		`"torch":{"tool_frame":"tool0","electrode_axis":[1,0,0]}}`
	for name, text := range map[string]string{
		"robot.urdf":      urdf,
		"config.json":     config,
		"meshes/base.dae": "<COLLADA/>",
		robotconfig.Manifest: `{"schema":"` + robotconfig.ManifestSchema + `","model_id":"cell_a",` +
			`"files":["config.json","meshes/base.dae","robot.urdf"]}`,
	} {
		if err := os.WriteFile(filepath.Join(dir, filepath.FromSlash(name)), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	identity, err := robotconfig.RobotDescriptionSHA256(dir)
	if err != nil {
		t.Fatal(err)
	}
	return root, identity
}

// packWith packs against a repo the test owns, returning what the seam worker
// was handed. Fetched descriptions land in a cache the test owns too.
func packWith(t *testing.T, root string, target CellTarget) (map[string]any, error) {
	t.Helper()
	var sent map[string]any
	server := NewServer(ServerOptions{RepoRoot: root, CellTarget: target,
		RobotDescriptionCache: filepath.Join(root, "cache"), Seams: seamWorkerFunc(
			func(_ context.Context, request []byte) (seam.Result, error) {
				if err := json.Unmarshal(request, &sent); err != nil {
					t.Fatal(err)
				}
				return seam.Result{JSON: []byte(`{"weldplan_base64":"` +
					base64.StdEncoding.EncodeToString([]byte("pk")) + `","sha256":"ab"}`)}, nil
			})})
	_, _, err := server.packWeldPlan(context.Background(), weldPlanRequest{
		CellID:     "cell_a",
		STEPBase64: base64.StdEncoding.EncodeToString([]byte("step")),
	})
	return sent, err
}

// contextOf is the planning context the worker was handed: its identity,
// and its members decoded. The worker is handed nothing else about the robot
// -- no URDF, no settings, no frames -- because one place composes the cell
// onto the description, and that place is here.
func contextOf(t *testing.T, sent map[string]any) (identity map[string]any, files map[string][]byte) {
	t.Helper()
	for _, gone := range []string{
		"machine_planning_calibration_base64", "robot_config_base64", "robot_description_sha256", "machine_planning_calibration_source",
		"cell_urdf_base64", "cell_settings", "robot_frames",
	} {
		if _, present := sent[gone]; present {
			t.Fatalf("the worker was still handed %q; it takes the planning context only", gone)
		}
	}
	context, ok := sent["planning_context"].(map[string]any)
	if !ok {
		t.Fatalf("no planning_context in %v", sent)
	}
	identity, _ = context["identity"].(map[string]any)
	encoded, _ := context["files"].(map[string]any)
	files = map[string][]byte{}
	for path, value := range encoded {
		data, err := base64.StdEncoding.DecodeString(value.(string))
		if err != nil {
			t.Fatalf("%s is not base64: %v", path, err)
		}
		files[path] = data
	}
	return identity, files
}

// The worker is handed the compiled robot, never the cell's document: one
// place reads machine_planning_calibration.json and composes it onto the description, and that place
// is here.
func TestPackWeldPlanSendsTheCompiledContext(t *testing.T) {
	root, identity := cellRepo(t)
	document := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{` +
		`"speed_ceiling_rad_s":0.5,"joints":{"J1":{"lower_rad":-2,"velocity_rad_s":0.05}},` +
		`"held":{"J2":0.25}},"kinematics":{"J2":{"xyz_m":[0.001,0,0]}}}`
	sent, err := packWith(t, root, targetServing(document))
	if err != nil {
		t.Fatal(err)
	}
	stamped, files := contextOf(t, sent)
	want := map[string]any{
		"model_id":                            "cell_a",
		"robot_description_sha256":            identity,
		"machine_planning_calibration_sha256": robotconfig.MachinePlanningCalibrationSHA256([]byte(document)),
		"machine_planning_calibration_source": "target",
	}
	got, _ := json.Marshal(stamped)
	expected, _ := json.Marshal(want)
	if string(got) != string(expected) {
		t.Fatalf("identity =\n%s\nwant\n%s", got, expected)
	}
	// A machine that says which configuration it runs has it recorded in the
	// plan's identity; one that does not leaves the identity as it was.
	described := targetServing(document)
	described.body.Robot.MachineConfigurationSHA256 = "sha256:" + strings.Repeat("9", 64)
	recorded, err := packWith(t, root, described)
	if err != nil {
		t.Fatal(err)
	}
	if identity, _ := contextOf(t, recorded); identity["machine_configuration_sha256"] != "sha256:"+strings.Repeat("9", 64) {
		t.Fatalf("identity = %v; want the machine's configuration recorded", identity)
	}
	for _, member := range []string{"robot.urdf", "planner.json", "identity.json", "meshes/base.dae"} {
		if _, present := files[member]; !present {
			t.Fatalf("the context has no %s; members are %v", member, files)
		}
	}
	if _, present := files["config.json"]; present {
		t.Fatal("the context carries config.json; the planner is not to read it")
	}
	// The cell is composed onto the URDF the worker will read: the narrowed
	// bound, the corrected origin, the planning rate.
	urdf, err := robotconfig.ParseURDF(string(files["robot.urdf"]))
	if err != nil {
		t.Fatal(err)
	}
	if urdf.Joints["J1"].Lower != -2 || urdf.Joints["J1"].Velocity != 0.05 ||
		urdf.Joints["J2"].XYZMetres != [3]float64{0.001, 0, 0} {
		t.Fatalf("the context's URDF is not the cell's: %+v", urdf.Joints)
	}
	var planner robotconfig.PlannerDocument
	if err := json.Unmarshal(files["planner.json"], &planner); err != nil {
		t.Fatal(err)
	}
	if planner.Held["J2"] != 0.25 || planner.SpeedCeilingRadS != 0.5 {
		t.Fatalf("planner.json = %+v", planner)
	}
}

// With no selected target, model-only offline planning remains available.
func TestPackWeldPlanWithoutAReachableTargetPlansAgainstTheEmptyDocument(t *testing.T) {
	root, identity := cellRepo(t)
	empty := robotconfig.MachinePlanningCalibrationSHA256(robotconfig.EmptyMachinePlanningCalibrationBytes("cell_a"))
	for name, target := range map[string]CellTarget{
		"no target":   nil,
		"unavailable": &fakeCellTarget{},
	} {
		t.Run(name, func(t *testing.T) {
			sent, err := packWith(t, root, target)
			if err != nil {
				t.Fatal(err)
			}
			identityOf, files := contextOf(t, sent)
			if identityOf["machine_planning_calibration_source"] != "empty" || identityOf["machine_planning_calibration_sha256"] != empty {
				t.Fatalf("identity = %v", identityOf)
			}
			if identityOf["robot_description_sha256"] != identity {
				t.Fatalf("robot_description_sha256 = %v, want %s", identityOf["robot_description_sha256"], identity)
			}
			var planner robotconfig.PlannerDocument
			if err := json.Unmarshal(files["planner.json"], &planner); err != nil {
				t.Fatal(err)
			}
			if len(planner.JointCorrections) != 0 || planner.SpeedCeilingRadS != 0 || len(planner.Held) != 0 {
				t.Fatalf("the empty document produced a deviation: %+v", planner)
			}
		})
	}
}

func TestPackWeldPlanDoesNotDropAnUnreachableSelectedMachinesLimits(t *testing.T) {
	root, _ := cellRepo(t)
	for _, target := range []*fakeCellTarget{
		{reachable: true, err: errors.New("dial: connection refused")},
		{reachable: true},
	} {
		sent, err := packWith(t, root, target)
		var refusal *seam.WorkerError
		if !errors.As(err, &refusal) || refusal.Code != seam.ErrorInvalidRequest ||
			!strings.Contains(refusal.Message, "Reconnect") || sent != nil {
			t.Fatalf("unreachable machine was allowed to plan without its limits: sent=%v err=%v", sent, err)
		}
	}
}

func TestPackWeldPlanCarriesNativeLimitsIntoPlannerAndURDF(t *testing.T) {
	root, _ := cellRepo(t)
	target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	acceleration := 0.5
	target.body.ExecutionLimits = robotconfig.ExecutionLimits{
		"J1": {PositionUnit: "rad", LowerRad: -1.5, UpperRad: 1.5, VelocityRadS: 0.04, AccelerationRadS2: &acceleration},
		"J2": {PositionUnit: "rad", LowerRad: -0.6, UpperRad: 0.6, VelocityRadS: 0.2, AccelerationRadS2: &acceleration},
	}
	sent, err := packWith(t, root, target)
	if err != nil {
		t.Fatal(err)
	}
	_, files := contextOf(t, sent)
	var planner robotconfig.PlannerDocument
	if err := json.Unmarshal(files["planner.json"], &planner); err != nil {
		t.Fatal(err)
	}
	urdf, err := robotconfig.ParseURDF(string(files["robot.urdf"]))
	if err != nil {
		t.Fatal(err)
	}
	if planner.Bounds["J1"].LowerRad != -1.5 || planner.Bounds["J2"].UpperRad != 0.6 ||
		planner.Limits["J1"].VelocityRadS != 0.04 || planner.Limits["J1"].AccelerationRadS2 != 0.5 ||
		urdf.Joints["J1"].Lower != -1.5 || urdf.Joints["J1"].Velocity != 0.04 || urdf.Joints["J2"].Upper != 0.6 ||
		planner.ExecutionLimitsSHA256 == "" {
		t.Fatalf("native constraints were lost: planner=%+v urdf=%+v", planner, urdf.Joints)
	}
}

func TestPackWeldPlanRejectsMissingNativeLimitsFromSelectedMachine(t *testing.T) {
	root, _ := cellRepo(t)
	for _, limits := range []robotconfig.ExecutionLimits{nil, {}} {
		target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
		target.body.ExecutionLimits = limits
		sent, err := packWith(t, root, target)
		var refusal *seam.WorkerError
		if !errors.As(err, &refusal) || refusal.Code != seam.ErrorInvalidRequest ||
			!strings.Contains(refusal.Message, "native planning limits") || sent != nil {
			t.Fatalf("missing native limits allowed model-only planning: sent=%v err=%v", sent, err)
		}
	}
}

// A machine_planning_calibration.json the description does not admit stops the pack, before anything
// is planned: the refusal is the caller's, so it is a 400.
func TestPackWeldPlanRefusesADeviationTheDescriptionDoesNotAdmit(t *testing.T) {
	root, _ := cellRepo(t)
	for name, document := range map[string]string{
		"widens a limit":     `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-4}}}}`,
		"correction too big": `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"xyz_m":[0.9,0,0]}}}`,
		"unknown key":        `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limit":{}}`,
		"another model":      `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_b"}`,
	} {
		t.Run(name, func(t *testing.T) {
			_, err := packWith(t, root, targetServing(document))
			var workerErr *seam.WorkerError
			if !errors.As(err, &workerErr) || workerErr.Code != seam.ErrorInvalidRequest {
				t.Fatalf("err = %v, want an invalid_request", err)
			}
			if status, _ := seamHTTPError(err); status != http.StatusBadRequest {
				t.Fatalf("status = %d", status)
			}
		})
	}
}

// The target is a different robot: refused here rather than planned and
// refused at the cell's own gate minutes later.
func TestPackWeldPlanRefusesATargetThatIsAnotherRobot(t *testing.T) {
	root, _ := cellRepo(t)
	target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	target.body.Robot.ModelID = "cell_b"
	_, err := packWith(t, root, target)
	if err == nil {
		t.Fatal("a plan for another robot was packed")
	}
	var workerErr *seam.WorkerError
	if !errors.As(err, &workerErr) || workerErr.Code != seam.ErrorInvalidRequest {
		t.Fatalf("err = %v, want an invalid_request", err)
	}
}

// The description this plan is for is the one the TARGET holds. A
// workstation whose copy hashes to what the cell advertises uses it; one
// whose copy does not fetches the cell's, verifies it, and keeps it by
// identity -- so a plan is never made against a robot the cell will refuse.
func TestPackWeldPlanPlansAgainstTheDescriptionTheTargetHolds(t *testing.T) {
	root, localIdentity := cellRepo(t)
	// The cell's revision: the same model with a second mesh registered.
	cellRoot, cellIdentity := cellRepo(t)
	cellDir := filepath.Join(cellRoot, "robot_description", "robots", "cell_a")
	if err := os.WriteFile(filepath.Join(cellDir, "meshes", "tool.dae"), []byte("<COLLADA/>"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(cellDir, robotconfig.Manifest), []byte(`{"schema":"`+robotconfig.ManifestSchema+
		`","model_id":"cell_a","files":["config.json","meshes/base.dae","meshes/tool.dae","robot.urdf"]}`), 0o644); err != nil {
		t.Fatal(err)
	}
	cellIdentity, err := robotconfig.RobotDescriptionSHA256(cellDir)
	if err != nil {
		t.Fatal(err)
	}
	if cellIdentity == localIdentity {
		t.Fatal("the test needs two revisions")
	}

	t.Run("the local copy is the cell's", func(t *testing.T) {
		target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
		target.body.Robot.RobotDescriptionSHA256 = localIdentity
		target.description = cellDir // would serve the other revision, if asked
		sent, err := packWith(t, root, target)
		if err != nil {
			t.Fatal(err)
		}
		identity, _ := contextOf(t, sent)
		if identity["robot_description_sha256"] != localIdentity || target.served != 0 {
			t.Fatalf("robot_description_sha256 = %v after %d fetches; want the local copy, unfetched", identity["robot_description_sha256"], target.served)
		}
	})

	t.Run("the local copy is another revision", func(t *testing.T) {
		target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
		target.body.Robot.RobotDescriptionSHA256 = cellIdentity
		target.description = cellDir
		sent, err := packWith(t, root, target)
		if err != nil {
			t.Fatal(err)
		}
		identity, files := contextOf(t, sent)
		if identity["robot_description_sha256"] != cellIdentity || target.served != 1 {
			t.Fatalf("robot_description_sha256 = %v after %d fetches; want the cell's, fetched once", identity["robot_description_sha256"], target.served)
		}
		if _, present := files["meshes/tool.dae"]; !present {
			t.Fatal("the context was compiled from the local copy, not the fetched one")
		}
		// Kept by identity: the next plan does not fetch again.
		if _, err := packWith(t, root, target); err != nil {
			t.Fatal(err)
		}
		if target.served != 1 {
			t.Fatalf("fetched %d times; the cache should have answered", target.served)
		}
	})

	t.Run("the cell serves something else", func(t *testing.T) {
		target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
		target.body.Robot.RobotDescriptionSHA256 = "sha256:" + strings.Repeat("d", 64)
		target.description = cellDir
		_, err := packWith(t, root, target)
		if err == nil || !strings.Contains(err.Error(), "refused") {
			t.Fatalf("a description that does not hash to the advertised identity was kept: %v", err)
		}
	})

	t.Run("an older daemon cannot serve", func(t *testing.T) {
		target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
		target.body.Robot.RobotDescriptionSHA256 = "sha256:" + strings.Repeat("e", 64)
		_, err := packWith(t, root, target)
		if err == nil || !strings.Contains(err.Error(), "could not serve its own") {
			t.Fatalf("want the mismatch named, got %v", err)
		}
	})
}

// No cell to ask, but a machine_planning_calibration.json in hand -- uploaded, or pulled earlier: the
// plan is made against it, labelled as the target's, and the cell that plays
// it compares the hash (docs/PLANNING-CONTEXT.md, Offline). A reachable cell
// still wins: its own file is the truth about it.
func TestPackWeldPlanUsesTheSuppliedMachinePlanningCalibrationOnlyWithoutACell(t *testing.T) {
	root, _ := cellRepo(t)
	uploaded := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-2}}}}`
	packOffline := func(target CellTarget) (map[string]any, error) {
		var sent map[string]any
		server := NewServer(ServerOptions{RepoRoot: root, CellTarget: target,
			RobotDescriptionCache: filepath.Join(root, "cache"), Seams: seamWorkerFunc(
				func(_ context.Context, request []byte) (seam.Result, error) {
					if err := json.Unmarshal(request, &sent); err != nil {
						t.Fatal(err)
					}
					return seam.Result{JSON: []byte(`{"weldplan_base64":"` +
						base64.StdEncoding.EncodeToString([]byte("pk")) + `","sha256":"ab"}`)}, nil
				})})
		_, _, err := server.packWeldPlan(context.Background(), weldPlanRequest{
			CellID: "cell_a", STEPBase64: base64.StdEncoding.EncodeToString([]byte("step")),
			MachinePlanningCalibrationBase64: base64.StdEncoding.EncodeToString([]byte(uploaded)),
		})
		return sent, err
	}

	sent, err := packOffline(nil)
	if err != nil {
		t.Fatal(err)
	}
	identity, files := contextOf(t, sent)
	if identity["machine_planning_calibration_source"] != "target" || identity["machine_planning_calibration_sha256"] != robotconfig.MachinePlanningCalibrationSHA256([]byte(uploaded)) {
		t.Fatalf("identity = %v; want the uploaded file, labelled as the target's", identity)
	}
	urdf, err := robotconfig.ParseURDF(string(files["robot.urdf"]))
	if err != nil || urdf.Joints["J1"].Lower != -2 {
		t.Fatalf("the uploaded narrowing did not reach the context: %v %+v", err, urdf.Joints["J1"])
	}

	// A widening in the upload is judged like the cell's own would be.
	widened := strings.Replace(uploaded, `-2`, `-4`, 1)
	if _, err := (func() (map[string]any, error) {
		uploaded = widened
		return packOffline(nil)
	})(); err == nil {
		t.Fatal("a widening uploaded as machine_planning_calibration.json was packed")
	}
	uploaded = `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"joints":{"J1":{"lower_rad":-2}}}}`

	// With a cell connected, its own file is what the plan is made against.
	own := `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","limits":{"speed_ceiling_rad_s":0.5}}`
	sent, err = packOffline(targetServing(own))
	if err != nil {
		t.Fatal(err)
	}
	identity, _ = contextOf(t, sent)
	if identity["machine_planning_calibration_sha256"] != robotconfig.MachinePlanningCalibrationSHA256([]byte(own)) {
		t.Fatalf("a connected cell's machine_planning_calibration.json was overridden by the upload: %v", identity)
	}
}
