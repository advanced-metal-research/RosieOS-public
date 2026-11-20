package build

import (
	"fmt"
	"io/fs"
	"path/filepath"
	"sort"
	"strings"
)

// DaemonV1SourceInputPaths returns the production Rust inputs used to build
// daemon/v1. Test-only Rust modules are deliberately excluded from the
// deployable source identity.
func DaemonV1SourceInputPaths(repo string) ([]string, error) {
	required := []string{
		"daemon/v1/Cargo.toml",
		"daemon/v1/Cargo.lock",
		"daemon/v1/manifests/steamdeck-mm-client-only.json",
		"daemon/v1/metadata/client-only.json",
	}
	for _, relative := range required {
		if !pathExists(filepath.Join(repo, filepath.FromSlash(relative))) {
			return nil, fmt.Errorf("daemon/v1 deploy source input %s does not exist", relative)
		}
	}

	inputs := append([]string(nil), required...)
	sourceRoot := filepath.Join(repo, "daemon", "v1", "src")
	if !pathExists(sourceRoot) {
		return nil, fmt.Errorf("daemon/v1 deploy source root %s does not exist", sourceRoot)
	}
	err := filepath.WalkDir(sourceRoot, func(path string, entry fs.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if entry.IsDir() {
			if path != sourceRoot && filepath.Clean(path) == filepath.Join(sourceRoot, "main_tests") {
				return filepath.SkipDir
			}
			return nil
		}
		if filepath.Ext(path) != ".rs" || entry.Name() == "tests.rs" || strings.HasSuffix(entry.Name(), "_tests.rs") {
			return nil
		}
		relative, err := filepath.Rel(repo, path)
		if err != nil {
			return err
		}
		inputs = append(inputs, filepath.ToSlash(relative))
		return nil
	})
	if err != nil {
		return nil, fmt.Errorf("walk daemon/v1 source inputs: %w", err)
	}
	sort.Strings(inputs)
	return inputs, nil
}
