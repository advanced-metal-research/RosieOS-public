package robotconfig

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// filesOf is a description directory as a cell serves it: the manifest and
// every registered file, by description-relative path.
func filesOf(t *testing.T, dir string) map[string][]byte {
	t.Helper()
	registered, err := DescriptionFiles(dir)
	if err != nil {
		t.Fatal(err)
	}
	files := map[string][]byte{}
	for _, path := range append(registered, Manifest) {
		data, err := os.ReadFile(filepath.Join(dir, filepath.FromSlash(path)))
		if err != nil {
			t.Fatal(err)
		}
		files[path] = data
	}
	return files
}

// What a cell serves is kept only when it hashes to what the cell said: the
// cell is checked, not trusted.
func TestAdoptKeepsADescriptionThatHashesToItsIdentity(t *testing.T) {
	model := loadedModel(t, nil)
	cache := t.TempDir()
	if _, ok := Cached(cache, "cell_a", model.Identity); ok {
		t.Fatal("an empty cache answered")
	}
	adopted, err := Adopt(cache, "cell_a", model.Identity, filesOf(t, model.Dir))
	if err != nil {
		t.Fatalf("Adopt: %v", err)
	}
	if adopted.Identity != model.Identity || adopted.URDF.RobotName != "cell_a" {
		t.Fatalf("adopted = %+v", adopted)
	}
	cached, ok := Cached(cache, "cell_a", model.Identity)
	if !ok || cached.Dir != adopted.Dir {
		t.Fatalf("Cached = %v, %v; want the adopted directory", cached.Dir, ok)
	}
	if !strings.HasPrefix(adopted.Dir, filepath.Join(cache, strings.TrimPrefix(model.Identity, "sha256:"))) {
		t.Fatalf("kept at %s, not under the identity", adopted.Dir)
	}
	if entries, _ := os.ReadDir(cache); len(entries) != 1 {
		t.Fatalf("the staging directory was left behind: %v", entries)
	}
}

func TestAdoptRefusesWhatDoesNotHashToTheAdvertisedIdentity(t *testing.T) {
	model := loadedModel(t, nil)
	cache := t.TempDir()
	other := "sha256:" + strings.Repeat("f", 64)
	if _, err := Adopt(cache, "cell_a", other, filesOf(t, model.Dir)); err == nil || !strings.Contains(err.Error(), "refused") {
		t.Fatalf("err = %v", err)
	}
	if _, ok := Cached(cache, "cell_a", other); ok {
		t.Fatal("a refused description was kept")
	}
	for _, bad := range []string{"../escape", "/abs", "a//b"} {
		files := filesOf(t, model.Dir)
		files[bad] = []byte("x")
		if _, err := Adopt(cache, "cell_a", model.Identity, files); err == nil {
			t.Fatalf("path %q was written", bad)
		}
	}
	if _, err := Adopt(cache, "../cell", model.Identity, nil); err == nil {
		t.Fatal("a model id that is a path was accepted")
	}
	if _, err := Adopt(cache, "cell_a", "not-an-identity", nil); err == nil {
		t.Fatal("an identity that is not one was accepted")
	}
}

// A cached copy that no longer hashes to its name -- edited, or half
// deleted -- is not a copy.
func TestCachedRefusesACopyThatDrifted(t *testing.T) {
	model := loadedModel(t, nil)
	cache := t.TempDir()
	adopted, err := Adopt(cache, "cell_a", model.Identity, filesOf(t, model.Dir))
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(adopted.Dir, "config.json"), []byte(testConfig+"\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, ok := Cached(cache, "cell_a", model.Identity); ok {
		t.Fatal("a drifted copy answered for its identity")
	}
}
