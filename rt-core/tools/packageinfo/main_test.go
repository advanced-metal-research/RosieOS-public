package main

import (
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// runtimeDir uses the supplied POSIX test filesystem; Go's GOTMPDIR may have synthetic Windows modes.
func runtimeDir(t *testing.T) string {
	t.Helper()
	dir, err := os.MkdirTemp(os.Getenv("TMPDIR"), "package-test-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := os.RemoveAll(dir); err != nil {
			t.Error(err)
		}
	})
	return dir
}

// TestInventoryRejectsLinks prevents a package path from escaping its verification root.
func TestInventoryRejectsLinks(t *testing.T) {
	root := runtimeDir(t)
	if err := os.Symlink("/etc/passwd", filepath.Join(root, "linked")); err != nil {
		t.Fatal(err)
	}
	if _, err := inventory(root); err == nil {
		t.Fatal("symlink accepted")
	}
}

// TestInventoryDigestAndMode uses a known SHA-256 vector and absolute permission bits.
func TestInventoryDigestAndMode(t *testing.T) {
	root := runtimeDir(t)
	path := filepath.Join(root, "payload")
	if err := os.WriteFile(path, []byte("abc"), 0640); err != nil {
		t.Fatal(err)
	}
	files, err := inventory(root)
	if err != nil {
		t.Fatal(err)
	}
	if len(files) != 1 || files[0].Path != "payload" || files[0].Mode != "0640" || files[0].SHA256 != "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad" {
		t.Fatalf("unexpected inventory: %+v", files)
	}
}

// TestInstalledModeMatchesPackagedUnits pins the modes package-runtime.sh
// installs: a 0755 expectation for a 0644 timer failed every package build
// with "file integrity mismatch: host/rosie-rt-crl.timer" (2026-10-20).
func TestInstalledModeMatchesPackagedUnits(t *testing.T) {
	for path, want := range map[string]string{
		"host/rosie-rt-core.service": "0644",
		"host/rosie-rt-crl.service":  "0644",
		"host/rosie-rt-crl.timer":    "0644",
		"host/install.sh":            "0755",
		"host/run-core":              "0755",
		"bin/rosie-rt-core":          "0755",
		"tools/verify-package.sh":    "0755",
		"config/drives/a6ec.json":    "0644",
	} {
		if got := installedMode(path); got != want {
			t.Errorf("installedMode(%q) = %s, want %s", path, got, want)
		}
	}
}

// TestManifestRefusesUnknownContract prevents silently accepting an incompatible verifier schema.
func TestManifestRefusesUnknownContract(t *testing.T) {
	root := runtimeDir(t)
	m := manifest{Schema: schema, FormatVersion: 1, Backend: "simulation", Excluded: []string{"rt_package.json"}, Inputs: inputs{GitSHA: strings.Repeat("a", 40)}}
	b, err := json.Marshal(m)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(root, "rt_package.json")
	if err = os.WriteFile(path, b, 0644); err != nil {
		t.Fatal(err)
	}
	if _, err = readManifest(root); err != nil {
		t.Fatal(err)
	}
	for _, bad := range [][]byte{append(append([]byte{}, b...), []byte(` {}`)...), []byte(strings.Replace(string(b), `"format_version":1`, `"format_version":2`, 1)), []byte(strings.Replace(string(b), `"backend":"simulation"`, `"backend":"unknown"`, 1))} {
		if err = os.WriteFile(path, bad, 0644); err != nil {
			t.Fatal(err)
		}
		if _, err = readManifest(root); err == nil {
			t.Fatal("invalid manifest accepted")
		}
	}
}

// TestPrerequisiteRejectsMissingGLIBCTag checks the actual host provider, not a version-number heuristic.
func TestPrerequisiteRejectsMissingGLIBCTag(t *testing.T) {
	arch, err := command("", "uname", "-m")
	if err != nil {
		t.Fatal(err)
	}
	id, err := inspect("/bin/true")
	if err != nil {
		t.Fatal(err)
	}
	if !includes(id.Needed, "libc.so.6") {
		t.Fatal("glibc fixture has no libc dependency")
	}
	id.Versions["libc.so.6"] = []string{"GLIBC_9999.0"}
	m := manifest{Inputs: inputs{Architecture: arch}, Files: []artifact{{Path: "fixture", ELF: id}}}
	if err = prerequisites(runtimeDir(t), m); err == nil || !strings.Contains(err.Error(), "GLIBC_9999.0") {
		t.Fatalf("missing GLIBC tag accepted or wrong failure: %v", err)
	}
}
