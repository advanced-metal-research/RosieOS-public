package robotconfig

// The model the OLP's own tests build on: files the test owns, so nothing
// here moves when a real robot is worked on. The contract's own tests live
// with it in rosieos/robotdesc.

import (
	"os"
	"path/filepath"
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
