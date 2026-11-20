package robotdesc

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// The layout, pinned to a number this package did not compute. The bytes
// below are fixed by the test itself, so this vector cannot go stale when
// somebody edits a real description; the daemon's C++ and the planner's
// Python are held to the same one, and three implementations agreeing with
// each other by construction would prove nothing.
const crossLanguageVector = "sha256:9c3105ce99e2cbbc138f7051596f69d4a14b71dab01d98732274c37207901483"

func TestDescriptionSHA256MatchesTheCrossLanguageVector(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "cell_a")
	if err := os.MkdirAll(filepath.Join(dir, "meshes"), 0o755); err != nil {
		t.Fatal(err)
	}
	write(t, dir, "robot.urdf", `<robot name="cell_a"/>`)
	write(t, dir, "meshes/base.dae", "<COLLADA/>")
	write(t, dir, "notes.md", "provenance, not the robot")
	write(t, dir, Manifest,
		`{"schema":"rosie.robot-manifest.v1","model_id":"cell_a","files":["meshes/base.dae","robot.urdf"]}`)
	if got := identity(t, dir); got != crossLanguageVector {
		t.Fatalf("identity = %s, the contract's vector is %s", got, crossLanguageVector)
	}
}

// And on the real descriptions, against the reference implementation rather
// than against a number: their contents move while the robot is worked on,
// and what must not move is the two implementations' agreement.
func TestDescriptionSHA256AgreesWithThePythonReference(t *testing.T) {
	python, err := exec.LookPath("python3")
	if err != nil {
		t.Skip("python3 is not installed")
	}
	// This package sits at <repository>/robot_description/go; the models are
	// its siblings under robots/. A standalone copy (the CI job copies
	// robot_description beside rt-core) has the models and not the planner,
	// so only a missing REFERENCE skips; a missing model is a broken tree.
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	for _, model := range []string{"rosie_1400_v3", "rosie_1420_v1"} {
		dir := filepath.Join(root, "robot_description", "robots", model)
		if _, err := os.Stat(filepath.Join(dir, Manifest)); err != nil {
			t.Fatalf("the repository's %s description is not at %s: %v", model, dir, err)
		}
		if _, err := os.Stat(filepath.Join(root, "weld_planner", "v1", "python", "weldplan", "robot_description.py")); err != nil {
			t.Skipf("the Python reference is not beside this copy: %v", err)
		}
		command := exec.Command(python, "-c",
			"import sys;from weldplan.robot_description import robot_description_sha256;"+
				"print(robot_description_sha256(sys.argv[1]))", dir)
		command.Env = append(os.Environ(),
			"PYTHONPATH="+filepath.Join(root, "weld_planner", "v1", "python"))
		out, err := command.CombinedOutput()
		if err != nil {
			t.Skipf("the reference implementation did not run: %v\n%s", err, out)
		}
		want := strings.TrimSpace(string(out))
		if got := identity(t, dir); got != want {
			t.Fatalf("%s: Go says %s, the reference says %s", model, got, want)
		}
	}
}

// A description directory with one registered file and one nobody registered.
func description(t *testing.T, files []string) string {
	t.Helper()
	dir := filepath.Join(t.TempDir(), "cell_a")
	if err := os.MkdirAll(filepath.Join(dir, "meshes"), 0o755); err != nil {
		t.Fatal(err)
	}
	write(t, dir, "robot.urdf", `<robot name="cell_a"/>`)
	write(t, dir, "meshes/base.dae", "<COLLADA/>")
	write(t, dir, "notes.md", "provenance, not the robot")
	// Present on disk, so registering it is refused for its NAME and not
	// for being missing.
	write(t, dir, MachinePlanningCalibration, `{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	manifest := map[string]any{"schema": ManifestSchema, "model_id": "cell_a", "files": files}
	raw, err := json.MarshalIndent(manifest, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	write(t, dir, Manifest, string(raw)+"\n")
	return dir
}

func write(t *testing.T, dir, rel, text string) {
	t.Helper()
	if err := os.WriteFile(filepath.Join(dir, filepath.FromSlash(rel)), []byte(text), 0o644); err != nil {
		t.Fatal(err)
	}
}

func identity(t *testing.T, dir string) string {
	t.Helper()
	got, err := RobotDescriptionSHA256(dir)
	if err != nil {
		t.Fatal(err)
	}
	return got
}

// What the identity does and does not move for. The first case is the whole
// reason this is a manifest: a CAD export or a README beside the robot must
// not make every plan on the cell stale.
func TestDescriptionSHA256CoversTheRegisteredFilesAndNothingElse(t *testing.T) {
	registered := []string{"meshes/base.dae", "robot.urdf"}
	dir := description(t, registered)
	before := identity(t, dir)

	write(t, dir, "notes.md", "edited, and still not the robot")
	write(t, dir, "meshes/spare.dae", "<COLLADA/>")
	if after := identity(t, dir); after != before {
		t.Fatal("an unregistered file changed the identity")
	}

	write(t, dir, "meshes/base.dae", "<COLLADA> </COLLADA>")
	edited := identity(t, dir)
	if edited == before {
		t.Fatal("editing a registered file did not change the identity")
	}

	// Dropping a name from the manifest is a change of identity, because the
	// manifest is hashed as an entry of its own.
	dropped := description(t, []string{"robot.urdf"})
	if identity(t, dropped) == identity(t, description(t, registered)) {
		t.Fatal("dropping a registered file did not change the identity")
	}
}

func TestDescriptionSHA256RefusesARegistryItCannotTrust(t *testing.T) {
	for name, files := range map[string][]string{
		"a file that is not here": {"robot.urdf", "meshes/missing.dae"},
		"a directory":             {"robot.urdf", "meshes"},
		"an absolute path":        {"/etc/passwd"},
		"an escape":               {"../robot.urdf"},
		"a duplicate":             {"robot.urdf", "robot.urdf"},
		"the manifest itself":     {"robot.urdf", Manifest},
		"the cell's own file":     {"robot.urdf", MachinePlanningCalibration},
		"nothing at all":          {},
	} {
		t.Run(name, func(t *testing.T) {
			if _, err := RobotDescriptionSHA256(description(t, files)); err == nil {
				t.Fatalf("%s was accepted", name)
			}
		})
	}

	dir := description(t, []string{"robot.urdf"})
	write(t, dir, Manifest, `{"schema":"rosie.robot-description.v0","model_id":"cell_a","files":["robot.urdf"]}`)
	if _, err := RobotDescriptionSHA256(dir); err == nil {
		t.Fatal("another schema was accepted")
	}
	write(t, dir, Manifest, `{"schema":"`+ManifestSchema+`","model_id":"cell_b","files":["robot.urdf"]}`)
	if _, err := RobotDescriptionSHA256(dir); err == nil {
		t.Fatal("a manifest for another model was accepted")
	}
	write(t, dir, Manifest, "{")
	if _, err := RobotDescriptionSHA256(dir); err == nil {
		t.Fatal("a manifest that is not JSON was accepted")
	}
	if err := os.Remove(filepath.Join(dir, Manifest)); err != nil {
		t.Fatal(err)
	}
	if _, err := RobotDescriptionSHA256(dir); err == nil {
		t.Fatal("a directory with no manifest has no identity")
	}
}

// The manifest is hashed like any other entry, so how it is written is part
// of the identity too: a description whose manifest lists the same files in
// another order is a different description, and the cell says so.
func TestDescriptionSHA256CoversTheRegistrysOwnBytes(t *testing.T) {
	forward := description(t, []string{"meshes/base.dae", "robot.urdf"})
	backward := description(t, []string{"robot.urdf", "meshes/base.dae"})
	if identity(t, forward) == identity(t, backward) {
		t.Fatal("rewriting the manifest left the identity alone")
	}
	// And it is stable: the same bytes hash the same way twice.
	if identity(t, forward) != identity(t, forward) {
		t.Fatal("the identity is not a function of the files")
	}
}
