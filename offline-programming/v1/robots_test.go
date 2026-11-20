package main

import (
	"encoding/base64"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/robotconfig"
)

// The catalogue is the store's answer, not a list anybody typed: a model
// appears because its directory is there.
func TestRobotsListsWhatTheStoreHolds(t *testing.T) {
	root := t.TempDir()
	writeModel(t, filepath.Join(root, "cell_a"))
	server := NewServer(ServerOptions{RobotDescriptionRoot: root})
	recorder := httptest.NewRecorder()
	server.robots(recorder, httptest.NewRequest(http.MethodGet, RobotsAPIPath, nil))
	if recorder.Code != http.StatusOK {
		t.Fatalf("status %d: %s", recorder.Code, recorder.Body)
	}
	var answer struct {
		Robots []struct {
			ModelID                string `json:"model_id"`
			RobotDescriptionSHA256 string `json:"robot_description_sha256"`
			Frames                 struct {
				Base       string     `json:"base"`
				Tool       string     `json:"tool"`
				Work       string     `json:"work"`
				WorkWorldM [3]float64 `json:"work_world_m"`
			} `json:"frames"`
			Axes struct {
				Driven []string           `json:"driven"`
				Held   map[string]float64 `json:"held"`
			} `json:"axes"`
		} `json:"robots"`
		Problems []string `json:"problems"`
	}
	if err := json.Unmarshal(recorder.Body.Bytes(), &answer); err != nil {
		t.Fatal(err)
	}
	if len(answer.Robots) != 1 || answer.Robots[0].ModelID != "cell_a" {
		t.Fatalf("robots = %+v", answer.Robots)
	}
	robot := answer.Robots[0]
	if robot.Frames.Base != "world" || robot.Frames.Tool != "tool0" || robot.Frames.Work != "table" {
		t.Fatalf("frames = %+v", robot.Frames)
	}
	// Walked from the URDF: 0.4 along the mount, then 0.1 up to the surface.
	want := [3]float64{0.4, 0, 0.1}
	for axis, value := range robot.Frames.WorkWorldM {
		if diff := value - want[axis]; diff > 1e-9 || diff < -1e-9 {
			t.Fatalf("work_world_m = %v, want %v", robot.Frames.WorkWorldM, want)
		}
	}
	if len(robot.Axes.Driven) != 1 || robot.Axes.Driven[0] != "J1" || robot.Axes.Held["J2"] != 0.5 {
		t.Fatalf("axes = %+v", robot.Axes)
	}
	if len(answer.Problems) != 0 {
		t.Fatalf("problems = %v", answer.Problems)
	}
}

// A description that will not load is named, and the rest still list.
func TestRobotsNamesADescriptionItCannotRead(t *testing.T) {
	root := t.TempDir()
	writeModel(t, filepath.Join(root, "cell_a"))
	broken := filepath.Join(root, "cell_b")
	if err := os.MkdirAll(broken, 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		"robot.urdf": `<robot name="cell_b"/>`,
		// Registered, and not there: the identity is unknown, so the model does not load.
		robotconfig.Manifest: `{"schema":"` + robotconfig.ManifestSchema + `","model_id":"cell_b","files":["robot.urdf","meshes/gone.dae"]}`,
	} {
		if err := os.WriteFile(filepath.Join(broken, name), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	recorder := httptest.NewRecorder()
	NewServer(ServerOptions{RobotDescriptionRoot: root}).robots(
		recorder, httptest.NewRequest(http.MethodGet, RobotsAPIPath, nil))
	var answer struct {
		Robots   []map[string]any `json:"robots"`
		Problems []string         `json:"problems"`
	}
	if err := json.Unmarshal(recorder.Body.Bytes(), &answer); err != nil {
		t.Fatal(err)
	}
	if len(answer.Robots) != 1 || len(answer.Problems) != 1 {
		t.Fatalf("robots %d problems %v", len(answer.Robots), answer.Problems)
	}
}

// A model with a positioner: the work frame is a point ON a link, and a
// mount between it and the base, so the walk has to compose both.
func writeModel(t *testing.T, dir string) {
	t.Helper()
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	urdf := `<?xml version="1.0"?>
<robot name="cell_a">
  <link name="world"/>
  <link name="bar"/>
  <link name="tool0"/>
  <joint name="mount" type="fixed">
    <parent link="world"/><child link="bar"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="J1" type="revolute">
    <parent link="world"/><child link="tool0"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="3.14"/>
  </joint>
  <joint name="J2" type="revolute">
    <parent link="bar"/><child link="tool0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="3.14"/>
  </joint>
</robot>
`
	config := `{"schema":"rosie.robot-config.v1","model_id":"cell_a",
  "frames":{"base":"world","tool":"tool0","work":"table",
            "work_surface":{"parent":"bar","xyz_m":[0,0,0.1]}},
  "kinematic_correction_caps":{"xyz_m":0.002,"rpy_rad":0.017},
  "planning":{"speed_ceiling_rad_s":0.2,"joints":{"J1":{"velocity_rad_s":0.09,"acceleration_rad_s2":2.0}}},
  "axes":{"driven":["J1"],"held":{"J2":0.5}},
  "reset_pose":{"J1":0.0},
  "torch":{"tool_frame":"tool0","electrode_axis":[1,0,0]}}`
	files := map[string]string{
		"robot.urdf":  urdf,
		"config.json": config,
		"robot_description_manifest.json": `{"schema":"rosie.robot-manifest.v1","model_id":"cell_a",` +
			`"files":["config.json","robot.urdf"]}`,
	}
	for name, text := range files {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
}
func TestRobotsIncludesSettingsSnapshot(t *testing.T) {
	root := t.TempDir()
	writeModel(t, filepath.Join(root, "cell_a"))
	recorder := httptest.NewRecorder()
	NewServer(ServerOptions{RobotDescriptionRoot: root}).robots(recorder, httptest.NewRequest(http.MethodGet, RobotsAPIPath, nil))
	var answer struct {
		Robots []struct {
			Settings struct {
				Joints []struct {
					Name          string   `json:"name"`
					RatedVelocity *float64 `json:"rated_velocity"`
				} `json:"joints"`
				Resources map[string]string `json:"resources"`
			} `json:"settings"`
		} `json:"robots"`
	}
	if err := json.Unmarshal(recorder.Body.Bytes(), &answer); err != nil {
		t.Fatal(err)
	}
	if len(answer.Robots) != 1 {
		t.Fatalf("robots = %+v", answer)
	}
	settings := answer.Robots[0].Settings
	if len(settings.Joints) != 3 || settings.Joints[0].Name != "J1" || settings.Joints[0].RatedVelocity == nil || *settings.Joints[0].RatedVelocity != 3.14 {
		t.Fatalf("settings = %+v", settings)
	}
	if len(settings.Resources) != 3 || len(settings.Resources["robot.urdf"]) != 71 {
		t.Fatalf("resources = %+v", settings.Resources)
	}
}

// A new program on a cell holding another description revision records its
// first waypoint against the cell's own description, fetched and verified the
// way planning fetches it; nothing the connected cell does not advertise is kept.
func TestCaptureModelFetchesTheConnectedCellsDescription(t *testing.T) {
	root, _ := cellRepo(t)
	cellRoot, _ := cellRepo(t)
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
	capture := func(target CellTarget, identity string) *httptest.ResponseRecorder {
		server := NewServer(ServerOptions{RepoRoot: root, CellTarget: target, RobotDescriptionCache: filepath.Join(root, "cache")})
		body, _ := json.Marshal(map[string]any{"model_id": "cell_a", "robot_description_sha256": identity, "machine_planning_calibration_base64": ""})
		recorder := httptest.NewRecorder()
		server.captureModel(recorder, httptest.NewRequest(http.MethodPost, RobotsAPIPath+"/capture-model", strings.NewReader(string(body))))
		return recorder
	}
	target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	target.body.Robot.RobotDescriptionSHA256 = cellIdentity
	target.description = cellDir
	if recorder := capture(target, cellIdentity); recorder.Code != http.StatusOK || target.served != 1 {
		t.Fatalf("status %d after %d fetches: %s", recorder.Code, target.served, recorder.Body)
	}
	other := "sha256:" + strings.Repeat("d", 64)
	if recorder := capture(target, other); recorder.Code != http.StatusBadRequest || target.served != 1 {
		t.Fatalf("captured %s, which the cell does not advertise: %d", other, recorder.Code)
	}
	target.reachable = false
	fresh := t.TempDir()
	server := NewServer(ServerOptions{RepoRoot: root, CellTarget: target, RobotDescriptionCache: fresh})
	body, _ := json.Marshal(map[string]any{"model_id": "cell_a", "robot_description_sha256": cellIdentity})
	recorder := httptest.NewRecorder()
	server.captureModel(recorder, httptest.NewRequest(http.MethodPost, RobotsAPIPath+"/capture-model", strings.NewReader(string(body))))
	if recorder.Code != http.StatusBadRequest {
		t.Fatalf("an unreachable cell's description was used: %d", recorder.Code)
	}
}

func TestCaptureModelUsesPlanningCalibration(t *testing.T) {
	root := t.TempDir()
	writeModel(t, filepath.Join(root, "cell_a"))
	server := NewServer(ServerOptions{RobotDescriptionRoot: root})
	description, err := server.robotStore.Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	calibration := []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a","kinematics":{"J1":{"xyz_m":[0.001,0,0],"rpy_rad":[0,0,0.01]}}}`)
	settings, err := robotconfig.Resolve(description, calibration, robotconfig.SourceTarget)
	if err != nil {
		t.Fatal(err)
	}
	compiled, err := robotconfig.Compile(description, settings)
	if err != nil {
		t.Fatal(err)
	}
	for _, identity := range []string{description.Identity, "sha256:" + strings.Repeat("0", 64)} {
		body, _ := json.Marshal(map[string]any{"model_id": "cell_a", "robot_description_sha256": identity, "machine_planning_calibration_base64": base64.StdEncoding.EncodeToString(calibration)})
		recorder := httptest.NewRecorder()
		server.captureModel(recorder, httptest.NewRequest(http.MethodPost, RobotsAPIPath+"/capture-model", strings.NewReader(string(body))))
		if identity != description.Identity {
			if recorder.Code != http.StatusBadRequest {
				t.Fatal("mismatched description accepted")
			}
			continue
		}
		if recorder.Code != http.StatusOK {
			t.Fatal(recorder.Body.String())
		}
		var answer struct {
			URDF     string               `json:"urdf"`
			Identity robotconfig.Identity `json:"identity"`
		}
		if err := json.Unmarshal(recorder.Body.Bytes(), &answer); err != nil {
			t.Fatal(err)
		}
		if answer.URDF != string(compiled.Files[robotconfig.ContextURDF]) || answer.Identity != compiled.Identity {
			t.Fatal("capture and planner use different robot geometry")
		}
	}
}
