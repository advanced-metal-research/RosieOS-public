package robotdesc

import (
	"encoding/json"
	"path/filepath"
	"testing"
)

func TestCatalogueSettingsReflectRegisteredValuesWithoutChangingIdentity(t *testing.T) {
	dir := description(t, []string{"robot.urdf", Definition, "config.json", "meshes/base.dae"})
	write(t, dir, "robot.urdf", `<robot name="cell_a"><joint name="J1" type="revolute"><limit lower="0" upper="2" velocity="3.14"/></joint><joint name="tool" type="fixed"/></robot>`)
	write(t, dir, Definition, `{"schema":"rosie.robot-definition.v1","revision":7,"joints":[{"name":"motor1","urdf_joint":"J1","gear_ratio":{"numerator":100,"denominator":2}}]}`)
	write(t, dir, "config.json", `{"schema":"rosie.robot-config.v1","model_id":"cell_a","planning":{"speed_ceiling_rad_s":0.2,"joints":{"J1":{"velocity_rad_s":0.09,"acceleration_rad_s2":2}}},"torch":{"tool_frame":"tip","electrode_axis":[1,0,0]}}`)
	d, err := At(filepath.Dir(dir)).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	before := identity(t, dir)
	got := d.Robot().Settings
	if got == nil || got.Revision == nil || *got.Revision != 7 || len(got.Joints) != 2 {
		t.Fatalf("settings = %+v", got)
	}
	joint := got.Joints[0]
	if joint.Name != "J1" || joint.Lower == nil || *joint.Lower != 0 || joint.Upper == nil || *joint.Upper != 2 || joint.RatedVelocity == nil || *joint.RatedVelocity != 3.14 || joint.GearRatio == nil || *joint.GearRatio != 50 {
		t.Fatalf("joint = %+v", joint)
	}
	fixed := got.Joints[1]
	if fixed.Lower != nil || fixed.Upper != nil || fixed.RatedVelocity != nil || fixed.GearRatio != nil {
		t.Fatalf("invented fixed limits: %+v", fixed)
	}
	if got.Planning == nil || got.Planning.Joints["J1"].VelocityRadS != 0.09 || got.Torch == nil || got.Torch.ToolFrame != "tip" {
		t.Fatalf("planning/torch = %+v", got)
	}
	if len(got.Resources) != 5 || got.Resources["notes.md"] != "" {
		t.Fatalf("resources = %+v", got.Resources)
	}
	if after := identity(t, dir); after != before || d.Robot().RobotDescriptionSHA256 != before {
		t.Fatal("catalogue changed robot identity")
	}
	raw, err := json.Marshal(got)
	if err != nil {
		t.Fatal(err)
	}
	var wire map[string]any
	if err := json.Unmarshal(raw, &wire); err != nil {
		t.Fatal(err)
	}
	if _, ok := wire["updated_at"]; ok {
		t.Fatal("invented timestamp")
	}
	joints := wire["joints"].([]any)
	if joints[0].(map[string]any)["lower"] != float64(0) {
		t.Fatal("zero lower bound omitted")
	}
	if _, ok := joints[1].(map[string]any)["lower"]; ok {
		t.Fatal("fixed joint given a lower bound")
	}
}

func TestCatalogueSettingsMissingDefinitionAndPolicyStayUnknown(t *testing.T) {
	dir := description(t, []string{"robot.urdf"})
	write(t, dir, Definition, `{"schema":"rosie.robot-definition.v1","revision":99}`)
	d, err := At(filepath.Dir(dir)).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	got := d.Robot().Settings
	if got == nil || got.Revision != nil || got.Planning != nil || got.Torch != nil {
		t.Fatalf("unregistered metadata = %+v", got)
	}
}

func TestCatalogueSettingsDoNotMislabelChangedResources(t *testing.T) {
	dir := description(t, []string{"robot.urdf", "meshes/base.dae"})
	d, err := At(filepath.Dir(dir)).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	first := d.Robot().Settings
	if first == nil {
		t.Fatal("missing initial settings")
	}
	write(t, dir, "meshes/base.dae", "<COLLADA>new geometry</COLLADA>")
	if d.Robot().Settings != nil {
		t.Fatal("new resources reported under stale identity")
	}
	next, err := At(filepath.Dir(dir)).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	if next.Robot().Settings.Resources["meshes/base.dae"] == first.Resources["meshes/base.dae"] {
		t.Fatal("geometry change invisible")
	}
}

func TestCatalogueSettingsInvalidGearRatioStaysUnknown(t *testing.T) {
	dir := description(t, []string{"robot.urdf", Definition})
	write(t, dir, "robot.urdf", `<robot name="cell_a"><joint name="J1" type="revolute"><limit lower="-1" upper="1" velocity="2"/></joint></robot>`)
	write(t, dir, Definition, `{"schema":"rosie.robot-definition.v1","joints":[{"name":"J1","gear_ratio":{"numerator":100,"denominator":0}}]}`)
	d, err := At(filepath.Dir(dir)).Load("cell_a")
	if err != nil {
		t.Fatal(err)
	}
	if d.Robot().Settings.Joints[0].GearRatio != nil {
		t.Fatal("invalid ratio is not a known value")
	}
}
