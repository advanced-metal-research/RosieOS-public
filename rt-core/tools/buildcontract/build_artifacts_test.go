package buildcontract

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// The real build-module rule must exclude reviewer probes and old dependency
// caches from ./..., without hiding any source package or deleting artifacts.
func TestBuildArtifactsDoNotBecomeGoPackages(t *testing.T) {
	makefile, err := filepath.Abs("../../Makefile")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll("../../build", 0700); err != nil {
		t.Fatal(err)
	}
	root, err := os.MkdirTemp("../../build", "artifact-discovery-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(root) })
	for path, body := range map[string]string{
		"go.mod":                                  "module artifacttest\n\ngo 1.24\n",
		"source.go":                               "package fixture\n",
		"build/reviewer_cpu_test.go":              "package simfixture\n",
		"build/reviewer_status_test.go":           "package main\n",
		"build/go-mod-cache/old@v1.0.0/source.go": "package dependency\n",
	} {
		path = filepath.Join(root, path)
		if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte(body), 0600); err != nil {
			t.Fatal(err)
		}
	}
	run := func(binary string, args ...string) ([]byte, error) {
		cmd := exec.Command(binary, args...)
		cmd.Dir = root
		return cmd.CombinedOutput()
	}
	if output, err := run("go", "list", "./..."); err == nil || !strings.Contains(string(output), "found packages") {
		t.Fatalf("reviewer scratch did not reproduce package-discovery failure: %v\n%s", err, output)
	}
	if output, err := run("make", "--no-print-directory", "-f", makefile, "build/go.mod"); err != nil {
		t.Fatalf("build isolation rule: %v\n%s", err, output)
	}
	if output, err := run("go", "list", "./..."); err != nil || string(output) != "artifacttest\n" {
		t.Fatalf("source package discovery after isolation: %v\n%s", err, output)
	}
	if body, err := os.ReadFile(filepath.Join(root, "build/reviewer_cpu_test.go")); err != nil || string(body) != "package simfixture\n" {
		t.Fatalf("reviewer probe changed: %v", err)
	}
}
