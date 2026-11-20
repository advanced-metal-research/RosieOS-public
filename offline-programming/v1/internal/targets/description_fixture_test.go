//go:build linux

package targets

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"testing"

	"rosieos/robotdesc"
)

// stageDescription copies one of the repository's robot descriptions into a
// fixture repository with its rt-core definition edited, re-registers and
// re-pins it, and returns the `robot` block a machine config pins it with.
// The registered files come along whole, because the compiled configuration
// snapshots every one of them.
func stageDescription(t *testing.T, repository, fixtureRoot, model string, edit func(definition map[string]any)) map[string]any {
	t.Helper()
	source := filepath.Join(repository, "robot_description", "robots", model)
	dir := filepath.Join(fixtureRoot, "robot_description", "robots", model)
	files, err := robotdesc.DescriptionFiles(source)
	if err != nil {
		t.Fatal(err)
	}
	for _, rel := range files {
		data, err := os.ReadFile(filepath.Join(source, filepath.FromSlash(rel)))
		if err != nil {
			t.Fatal(err)
		}
		if rel == robotdesc.Definition {
			var definition map[string]any
			if err := json.Unmarshal(data, &definition); err != nil {
				t.Fatal(err)
			}
			edit(definition)
			if data, err = json.Marshal(definition); err != nil {
				t.Fatal(err)
			}
		}
		target := filepath.Join(dir, filepath.FromSlash(rel))
		if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(target, data, 0o644); err != nil {
			t.Fatal(err)
		}
	}
	sort.Strings(files)
	quoted := make([]string, len(files))
	for i, rel := range files {
		quoted[i] = strconv.Quote(rel)
	}
	manifest := `{"schema":"` + robotdesc.ManifestSchema + `","model_id":"` + model + `","files":[` + strings.Join(quoted, ",") + `]}`
	if err := os.WriteFile(filepath.Join(dir, robotdesc.Manifest), []byte(manifest), 0o644); err != nil {
		t.Fatal(err)
	}
	identity, err := robotdesc.RobotDescriptionSHA256(dir)
	if err != nil {
		t.Fatal(err)
	}
	return map[string]any{"robot_description": map[string]any{"path": "robot_description/robots/" + model, "sha256": identity}}
}
