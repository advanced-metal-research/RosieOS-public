package main

import (
	"os"
	"path/filepath"
	"testing"
)

func TestRobotV4DaemonV1SourceBundleExcludesTestOnlyRustSources(t *testing.T) {
	repo := t.TempDir()
	writeSource := func(rel, content string) {
		t.Helper()
		path := filepath.Join(repo, filepath.FromSlash(rel))
		if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
			t.Fatalf("create source directory for %s: %v", rel, err)
		}
		if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
			t.Fatalf("write source %s: %v", rel, err)
		}
	}

	for rel, content := range map[string]string{
		"daemon/v1/Cargo.toml":                    "[package]\nname = \"fixture\"\n",
		"daemon/v1/Cargo.lock":                    "version = 4\n",
		"daemon/v1/src/main.rs":                   "fn main() {}\n",
		"daemon/v1/src/deploy/mod.rs":             "pub fn deploy() {}\n",
		"daemon/v1/src/main_runtime/self_test.rs": "pub fn self_test() {}\n",
		"daemon/v1/src/main_tests/auto_apply.rs":  "#[test]\nfn auto_apply() {}\n",
		"daemon/v1/src/deploy/tests.rs":           "#[test]\nfn deploy() {}\n",
		"daemon/v1/src/install/tests.rs":          "#[test]\nfn install() {}\n",
	} {
		writeSource(rel, content)
	}

	target := robotV4MeshDeployTarget{
		Peer:         "fixture-peer",
		ArtifactKind: "daemon-v1",
		BuildCommand: "cargo build --release --locked",
	}
	_, initialHash, _, err := robotV4ComponentSourceBundle(repo, target)
	if err != nil {
		t.Fatalf("build initial daemon/v1 source bundle: %v", err)
	}
	writeSource("daemon/v1/src/main_tests/auto_apply.rs", "#[test]\nfn changed_auto_apply() {}\n")
	writeSource("daemon/v1/src/deploy/tests.rs", "#[test]\nfn changed_deploy() {}\n")
	_, testOnlyHash, _, err := robotV4ComponentSourceBundle(repo, target)
	if err != nil {
		t.Fatalf("build daemon/v1 source bundle after test-only changes: %v", err)
	}
	if testOnlyHash != initialHash {
		t.Fatalf("test-only Rust changes altered release source hash: initial=%s changed=%s", initialHash, testOnlyHash)
	}
	writeSource("daemon/v1/src/deploy/mod.rs", "pub fn changed_deploy() {}\n")
	_, productionHash, _, err := robotV4ComponentSourceBundle(repo, target)
	if err != nil {
		t.Fatalf("build daemon/v1 source bundle after production change: %v", err)
	}
	if productionHash == initialHash {
		t.Fatalf("production Rust change did not alter release source hash: %s", productionHash)
	}
}
