//go:build linux

package targets

import (
	"os"
	"path/filepath"
	"testing"
)

// TestCellManifestNameReadsTheManifest keeps a cell's identity in its
// manifest: a cell is admitted by adding rt-core/config/cells/<id>.json, never
// by naming it in code, and an id from a catalogue or network discovery that
// is not a plain cell id never reaches a file path.
func TestCellManifestNameReadsTheManifest(t *testing.T) {
	root := t.TempDir()
	cells := filepath.Join(root, "rt-core/config/cells")
	if err := os.MkdirAll(cells, 0o755); err != nil {
		t.Fatal(err)
	}
	write := func(name, body string) {
		t.Helper()
		if err := os.WriteFile(filepath.Join(cells, name), []byte(body), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	write("example-cell-1.json", `{"name": "example-host", "nodes": []}`)
	write("no-host.json", `{"nodes": []}`)
	write("broken.json", `{"name": `)
	// A manifest outside rt-core/config/cells that a crafted id could reach.
	if err := os.WriteFile(filepath.Join(root, "outside.json"), []byte(`{"name": "outside"}`), 0o644); err != nil {
		t.Fatal(err)
	}

	name, raw, err := cellManifestName(root, "example-cell-1")
	if err != nil || name != "example-host" || len(raw) == 0 {
		t.Fatalf("a cell added as a manifest: name=%q err=%v", name, err)
	}
	for id, want := range map[string]string{
		"missing-cell":  "cell_id_unknown",
		"":              "cell_id_unknown",
		"../../outside": "cell_id_unknown",
		"a/b":           "cell_id_unknown",
		"Example-Cell":  "cell_id_unknown",
		"cell-":         "cell_id_unknown",
		"no-host":       "cell_manifest_invalid",
		"broken":        "cell_manifest_invalid",
	} {
		if name, _, err := cellManifestName(root, id); err == nil || err.Error() != want || name != "" {
			t.Errorf("cellManifestName(%q) = %q, %v; want %s", id, name, err, want)
		}
	}
	if _, err := ResolveCellConfiguration(root, "../../outside"); err == nil || err.Error() != "cell_id_unknown" {
		t.Fatalf("ResolveCellConfiguration let a crafted id reach a path: %v", err)
	}
}

// manifestHost is the host name cell id's manifest records, "" when it does
// not resolve: what the hard-coded cell list used to answer.
func manifestHost(root, id string) string {
	name, _, _ := cellManifestName(root, id)
	return name
}
