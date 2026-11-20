//go:build linux

package simfixture

import (
	"os/exec"
	"path/filepath"
	"testing"
)

// CompileCPP compiles the operator example and test driver with warnings fatal,
// preventing integration from silently exercising a stale build/ binary.
func CompileCPP(t *testing.T, root, build, source, name string) string {
	t.Helper()
	binary := filepath.Join(build, name)
	cmd := exec.Command("g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-I"+filepath.Join(root, "clients/cpp/include"), "-I"+filepath.Join(root, "include"), "-I"+root, filepath.Join(root, source), "-pthread", "-o", binary, "-UNDEBUG")
	if source == "clients/cpp/examples/motion_server_sequence.cpp" {
		cmd.Args = append(cmd.Args, "-DNDEBUG")
	}
	raw, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("g++: %v %s", err, raw)
	}
	return binary
}
