package rtctl

import (
	"strings"
	"testing"
)

// TestRobotDefinitionsDocumentation keeps the deployment procedure discoverable
// beside migration, including identity changes that require a fresh Home.
func TestRobotDefinitionsDocumentation(t *testing.T) {
	doc := string(readTestFile(t, "docs/MIGRATION.md"))
	_, section, found := strings.Cut(doc, "### Robot definitions\n")
	if !found {
		t.Fatal("configuration guide has no Robot definitions section")
	}
	section, _, _ = strings.Cut(section, "\n## ")
	section = strings.Join(strings.Fields(section), " ")
	for _, contract := range []string{
		"rosie.robot-definition.v1", "robot_description/robots/", "rtcore_definition.json", "robot_description_manifest.json", "robot_description_sha256", "robot_id", "revision", "SHA-256",
		"geometry, mapped joint types and URDF limits", "joint mapping", "reductions", "encoder counts", "logical signs", "Cartesian participation", "default policies",
		"object descriptors", "position semantics", "startup defaults", "absent-joint declarations", "robot_joint", "slave_position", "narrowing overrides",
		"cannot override gearing, sign, wire counts or startup objects", "machine_sha256", "deployment_sha256", "configuration_sha256",
		"own epoch and anchor records", "Factory geometry/joints/limits", "Definition mechanics or policies", "Cell mapping/limits/deployment", "Home calibration",
		"coordinate_identity_sha256", "Existing v1 anchors require one Home", "subsequent non-coordinate changes preserve it", "HSS-40-100-I-D19", "HRP130KK-G18", "unverified",
		"J5 is settled per model", "27:2", "Rosie 1420 v1 uses 100/1", "owner confirmation on 2026-09-15",
		"physical pulley teeth on the current 1400 build remain unverified", "1420 J5 mechanism differs from that belt stage",
		"HRP85 variant's part-number ratio is unconfirmed", "1420 export has no component list", "17-bit motor-side absolute count model",
		"robot.robot_description", "robot.machine_planning_calibration", "GET /v1/resources/<sha256>", "tools/package-runtime.sh", "robot_description_unavailable", "robot_description_mismatch", "robot_hash_mismatch",
	} {
		if !strings.Contains(strings.ToLower(section), strings.ToLower(contract)) {
			t.Errorf("Robot definitions documentation omits %q", contract)
		}
	}
}

// TestRobotDefinitionDataflow keeps the compile inputs and packaged resource
// dependency visible, preventing the diagram from assigning mechanics to cells.
func TestRobotDefinitionDataflow(t *testing.T) {
	doc := string(readTestFile(t, "docs/DATAFLOW.md"))
	_, section, found := strings.Cut(doc, "## 2. Configuration:")
	if !found {
		t.Fatal("configuration dataflow section missing")
	}
	section, _, _ = strings.Cut(section, "\n## 3.")
	for _, dependency := range []string{"Runtime package", "Robot description", "robot_description/robots/", "rtcore_definition.json", "robot_description_manifest.json", "robot_description_sha256", "MODEL --> COMP", "MAN --> COMP", "PROF --> COMP", "CFG --> COMP", "URDF --> MODEL"} {
		if !strings.Contains(section, dependency) {
			t.Errorf("configuration dataflow omits %q", dependency)
		}
	}
	for _, line := range strings.Split(section, "\n") {
		if strings.HasPrefix(strings.TrimSpace(line), `CFG[`) && (strings.Contains(line, "counts per rev, sign") || strings.Contains(line, "startup values")) {
			t.Error("cell diagram still owns definition mechanics")
		}
	}
}
