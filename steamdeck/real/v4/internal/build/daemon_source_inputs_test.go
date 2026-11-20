package build

import (
	"os"
	"path/filepath"
	"slices"
	"strings"
	"testing"
)

func TestDaemonV1SourceInputPathsExcludesTestOnlyRustSources(t *testing.T) {
	repo := t.TempDir()
	writeDaemonSourceInput(t, repo, "daemon/v1/Cargo.toml", "[package]\nname = \"fixture\"\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/Cargo.lock", "version = 4\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/manifests/steamdeck-mm-client-only.json", "{\"schema\":\"client-only\"}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/metadata/client-only.json", "{\"schema\":\"metadata\"}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/main.rs", "fn main() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/deploy/mod.rs", "pub fn deploy() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/main_runtime/self_test.rs", "pub fn self_test() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/main_tests/auto_apply.rs", "#[test]\nfn auto_apply() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/deploy/tests.rs", "#[test]\nfn deploy() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/install/tests.rs", "#[test]\nfn install() {}\n")
	writeDaemonSourceInput(t, repo, "daemon/v1/src/install/daemon_only_tests.rs", "#[test]\nfn daemon_only() {}\n")

	inputs, err := DaemonV1SourceInputPaths(repo)
	if err != nil {
		t.Fatalf("select daemon/v1 source inputs: %v", err)
	}
	want := []string{
		"daemon/v1/Cargo.lock",
		"daemon/v1/Cargo.toml",
		"daemon/v1/manifests/steamdeck-mm-client-only.json",
		"daemon/v1/metadata/client-only.json",
		"daemon/v1/src/deploy/mod.rs",
		"daemon/v1/src/main.rs",
		"daemon/v1/src/main_runtime/self_test.rs",
	}
	if !slices.Equal(inputs, want) {
		t.Fatalf("daemon/v1 source inputs = %v, want sorted production inputs %v", inputs, want)
	}
}

func TestDaemonV1SourceInputPathsRequiresBuildMetadataAndSourceRoot(t *testing.T) {
	t.Run("missing Cargo manifest", func(t *testing.T) {
		repo := t.TempDir()
		_, err := DaemonV1SourceInputPaths(repo)
		if err == nil || !strings.Contains(err.Error(), "daemon/v1/Cargo.toml does not exist") {
			t.Fatalf("missing Cargo.toml error = %v", err)
		}
	})

	t.Run("missing Cargo lock", func(t *testing.T) {
		repo := t.TempDir()
		writeDaemonSourceInput(t, repo, "daemon/v1/Cargo.toml", "[package]\nname = \"fixture\"\n")
		_, err := DaemonV1SourceInputPaths(repo)
		if err == nil || !strings.Contains(err.Error(), "daemon/v1/Cargo.lock does not exist") {
			t.Fatalf("missing Cargo.lock error = %v", err)
		}
	})

	t.Run("missing source root", func(t *testing.T) {
		repo := t.TempDir()
		writeDaemonSourceInput(t, repo, "daemon/v1/Cargo.toml", "[package]\nname = \"fixture\"\n")
		writeDaemonSourceInput(t, repo, "daemon/v1/Cargo.lock", "version = 4\n")
		writeDaemonSourceInput(t, repo, "daemon/v1/manifests/steamdeck-mm-client-only.json", "{}\n")
		writeDaemonSourceInput(t, repo, "daemon/v1/metadata/client-only.json", "{}\n")
		_, err := DaemonV1SourceInputPaths(repo)
		if err == nil || !strings.Contains(err.Error(), "daemon/v1 deploy source root") {
			t.Fatalf("missing source-root error = %v", err)
		}
	})
}

func writeDaemonSourceInput(t *testing.T, repo, relative, content string) {
	t.Helper()
	path := filepath.Join(repo, filepath.FromSlash(relative))
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatalf("create source directory for %s: %v", relative, err)
	}
	if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
		t.Fatalf("write source %s: %v", relative, err)
	}
}
