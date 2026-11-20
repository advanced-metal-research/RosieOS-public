//go:build linux

package control

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// A compiled set for a robot with geometry: the URDF, a config.json and the
// mesh the URDF names, all registered, beside the definition and the
// machine's machine_planning_calibration.json.
func describedResourceFixture(t *testing.T) (string, ipcclient.Description, ResourceManifest, map[string][]byte) {
	t.Helper()
	dir := remoteTestDir(t)
	robot, files := describedSet("fixture", `{"robot_id":"fixture","defaults":{"drive_profile":"test"},"joints":[{"name":"axis_0"}]}`,
		`{"schema":"rosie.machine-planning-calibration.v1","model_id":"fixture"}`+"\n", map[string]string{
			"robot.urdf":      `<robot name="fixture"><link name="l1"><visual><geometry><mesh filename="package://fixture/meshes/base.dae"/></geometry></visual></link><joint name="J1" type="revolute"><limit lower="-2" upper="2" velocity="0.5"/></joint></robot>`,
			"config.json":     `{"schema":"rosie.robot-config.v1","model_id":"fixture"}`,
			"meshes/base.dae": "<COLLADA/>",
		})
	identity := []byte(machineIdentity("fixture", robot.RobotDescriptionSHA256, ""))
	native := ipcclient.Description{ConfigurationSHA256: resourceDigest(identity), MachineSHA256: strings.Repeat("a", 64), Axes: []ipcclient.AxisDescription{{ID: "axis_0"}}}
	m := ResourceManifest{ConfigurationSHA256: native.ConfigurationSHA256, MachineSHA256: native.MachineSHA256, Robot: robot,
		Drives: []DriveDescription{{Axis: "axis_0", ConfigName: "test", ConfigSHA256: resourceDigest([]byte(`{}`))}}}
	if err := os.Mkdir(filepath.Join(dir, "resources"), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "configuration.identity"), identity, 0644); err != nil {
		t.Fatal(err)
	}
	for digest, data := range files {
		if err := os.WriteFile(filepath.Join(dir, "resources", digest), data, 0644); err != nil {
			t.Fatal(err)
		}
	}
	writeResourceManifest(t, dir, m)
	return dir, native, m, files
}

func TestResourceDescriptionIsAdmittedWhole(t *testing.T) {
	dir, native, m, files := describedResourceFixture(t)
	set, err := loadCompiledResources(dir, native)
	if err != nil {
		t.Fatalf("a described set was refused: %v", err)
	}
	for digest := range files {
		if set.data[digest] == nil {
			t.Fatalf("resource %s not served", digest)
		}
	}
	// A registered file missing, a mesh outside the manifest, the manifest
	// or definition gone, no machine_planning_calibration.json, or a file the manifest does not name:
	// each refused whole.
	without := func(path string) ResourceManifest {
		out := m
		out.Robot = &RobotDescription{ModelID: m.Robot.ModelID, RobotDescriptionSHA256: m.Robot.RobotDescriptionSHA256, MachinePlanningCalibrationSHA256: m.Robot.MachinePlanningCalibrationSHA256}
		for _, r := range m.Robot.Resources {
			if r.Path != path {
				out.Robot.Resources = append(out.Robot.Resources, r)
			}
		}
		return out
	}
	for _, path := range []string{"config.json", "meshes/base.dae", "machine_planning_calibration.json", "robot_description_manifest.json", "rtcore_definition.json", "robot.urdf"} {
		writeResourceManifest(t, dir, without(path))
		if _, err := loadCompiledResources(dir, native); err == nil {
			t.Fatalf("a set without %s was admitted", path)
		}
	}
	extra := m
	extra.Robot = &RobotDescription{ModelID: m.Robot.ModelID, RobotDescriptionSHA256: m.Robot.RobotDescriptionSHA256, MachinePlanningCalibrationSHA256: m.Robot.MachinePlanningCalibrationSHA256,
		Resources: append(append([]ResourceInfo{}, m.Robot.Resources...), ResourceInfo{Path: "notes.txt", SHA256: definitionSHA(m), Bytes: m.Robot.Resources[1].Bytes, MediaType: "application/json"})}
	writeResourceManifest(t, dir, extra)
	if _, err := loadCompiledResources(dir, native); err == nil {
		t.Fatal("a file the manifest does not register was admitted")
	}
	wrongCell := m
	wrongCell.Robot = &RobotDescription{ModelID: m.Robot.ModelID, RobotDescriptionSHA256: m.Robot.RobotDescriptionSHA256, MachinePlanningCalibrationSHA256: "sha256:" + strings.Repeat("c", 64), Resources: m.Robot.Resources}
	writeResourceManifest(t, dir, wrongCell)
	if _, err := loadCompiledResources(dir, native); err == nil {
		t.Fatal("a machine_planning_calibration.json served under another identity was admitted")
	}
	writeResourceManifest(t, dir, m)
	if _, err := loadCompiledResources(dir, native); err != nil {
		t.Fatal(err)
	}
}
