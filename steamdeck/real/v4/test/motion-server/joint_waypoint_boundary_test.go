package motion_server

import (
	"os/exec"
	"path/filepath"
	"testing"
)

func TestJointWaypointBoundary(t *testing.T) {
	root := findRepoRoot(t)
	binary := filepath.Join(t.TempDir(), "joint-waypoint")
	cmd := exec.Command("c++", "-std=c++17", "-O0", "-pthread",
		filepath.Join(root, "motion-server/v1/tests/joint_waypoint_boundary_test.cpp"), "-o", binary)
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("compile: %v\n%s", err, out)
	}
	if out, err := exec.Command(binary).CombinedOutput(); err != nil {
		t.Fatalf("waypoint boundary: %v\n%s", err, out)
	}
}
