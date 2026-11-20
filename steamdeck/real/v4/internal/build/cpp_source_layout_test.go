package build

import (
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
)

func buildContractRepositoryRoot(t *testing.T) string {
	t.Helper()
	_, source, _, ok := runtime.Caller(0)
	if !ok {
		t.Fatal("resolve internal build contract source path")
	}
	root := filepath.Clean(filepath.Join(
		filepath.Dir(source),
		"..",
		"..",
		"..",
		"..",
		"..",
	))
	if _, err := os.Stat(filepath.Join(root, "steamdeck", "real", "v4", "go.mod")); err != nil {
		t.Fatalf("resolve repository root from %s: %v", source, err)
	}
	return root
}

func TestNativeCXXBuildsUsePersistentTargetAwareObjectCachesAndArchives(t *testing.T) {
	root := buildContractRepositoryRoot(t)

	makefiles := []string{
		"motion-server/v1/Makefile",
	}
	for _, relative := range makefiles {
		relative := relative
		t.Run(strings.ReplaceAll(relative, "/", "_"), func(t *testing.T) {
			raw, readErr := os.ReadFile(filepath.Join(root, filepath.FromSlash(relative)))
			if readErr != nil {
				t.Fatal(readErr)
			}
			source := string(raw)
			for _, marker := range []string{
				"OBJECT_CACHE_DIR ?=",
				"target_triple=",
				"$(CXX) --version",
				"preprocessed",
				"-MMD -MP",
				"$(AR) rcs",
			} {
				if !strings.Contains(source, marker) {
					t.Fatalf("%s is missing cache/archive contract marker %q", relative, marker)
				}
			}
			cleanStart := strings.LastIndex(source, "\nclean:")
			if cleanStart < 0 {
				t.Fatalf("%s has no clean target", relative)
			}
			cleanRecipe := source[cleanStart:]
			if strings.Contains(cleanRecipe, "OBJECT_CACHE_DIR") || strings.Contains(cleanRecipe, "build-cache") {
				t.Fatalf("%s clean target removes its persistent object cache: %s", relative, cleanRecipe)
			}
		})
	}
}
