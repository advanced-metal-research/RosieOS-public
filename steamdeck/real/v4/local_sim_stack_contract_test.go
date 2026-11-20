package main

import (
	"bytes"

	"os"

	"path/filepath"

	"strings"
	"testing"
)

func TestRobotV4RosieModelsOwnCoincidentEndEffectorToolFrames(t *testing.T) {
	root, err := repoRoot()
	if err != nil {
		t.Fatal(err)
	}
	rosie1400, err := os.ReadFile(
		filepath.Join(root, "robot_description", "robots", "rosie_1400_v3", "robot.urdf"))
	if err != nil {
		t.Fatal(err)
	}
	rosie1420, err := os.ReadFile(
		filepath.Join(root, "robot_description", "robots", "rosie_1420_v1", "robot.urdf"))
	if err != nil {
		t.Fatal(err)
	}
	for model, raw := range map[string][]byte{
		"rosie_1400_v3": rosie1400,
		"rosie_1420_v1": rosie1420,
	} {
		document := string(raw)
		for _, required := range []string{
			`<robot name="` + model + `"`,
			`<joint name="link_6_to_end_effector" type="fixed">`,
			`rpy="3.141592654 0.785398163 -3.141592654"`,
			`<joint name="end_effector_to_tool0" type="fixed">`,
			`<origin xyz="0 0 0" rpy="0 0 0"/>`,
		} {
			if !strings.Contains(document, required) {
				t.Fatalf("%s URDF missing frame contract %q", model, required)
			}
		}
	}
	if bytes.Equal(rosie1400, rosie1420) {
		t.Fatal("Rosie 1400 and Rosie 1420 must retain separate model-owned URDFs")
	}
	// Both models bolt the same mig_torch_v1 to the same place on link_6, so
	// they share one tip and one flange translation. That is measured, not
	// assumed: the two link_6.dae files carry one 867-point vertex set, and the
	// 1420's own Fusion export puts the torch at link_6's origin with a
	// bounding box matching it to 10 um.
	//
	// This used to require the two to DIFFER, and held the 1420 at
	// `0.090505385 -0.001675337 -0.406250448` -- the centre of the bottom face
	// of the torch's axis-aligned bounding box, which is the tip of a straight
	// tool and not of a swan neck. It put tool0 55.7 mm off the gun in free
	// air, and the test asserted it there. A separate file is what keeps the
	// models independent, which `bytes.Equal` above already checks; a separate
	// NUMBER is not, when the hardware is the same hardware.
	for _, model := range []struct {
		name     string
		document []byte
	}{
		{"Rosie 1400", rosie1400},
		{"Rosie 1420", rosie1420},
	} {
		if !strings.Contains(
			string(model.document),
			`xyz="0.004682724 0 -0.398382724"`) {
			t.Fatalf("%s is missing the measured torch-tip flange translation", model.name)
		}
	}
}

func TestRobotV4NativeMakefilesDefaultToCompleteBuild(t *testing.T) {
	for _, path := range []string{
		"../../../motion-server/v1/Makefile",
	} {
		raw, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if !strings.Contains(string(raw), ".DEFAULT_GOAL := all") {
			t.Fatalf("%s can select an included dependency as its default goal", path)
		}
	}
}
