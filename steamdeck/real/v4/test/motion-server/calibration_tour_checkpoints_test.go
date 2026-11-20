package motion_server

import (
	"context"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

func TestRobotV4MotionServerCalibrationTourCheckpointRuntime(t *testing.T) {
	repoRoot := findRepoRoot(t)
	cxx := strings.TrimSpace(os.Getenv("CXX"))
	if cxx == "" {
		cxx = "c++"
	}
	motionServerDirectory := filepath.Join(
		repoRoot,
		"motion-server", "v1",
	)
	sourceDirectory := filepath.Join(motionServerDirectory, "src")
	testSource := filepath.Join(
		motionServerDirectory,
		"tests",
		"calibration_tour_checkpoints_runtime_test.cpp",
	)
	binary := filepath.Join(t.TempDir(), "calibration-tour-checkpoints-test")

	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	compile := exec.CommandContext(
		ctx,
		cxx,
		"-std=c++17",
		"-O2",
		"-Wall",
		"-Wextra",
		"-Wpedantic",
		"-I",
		sourceDirectory,
		"-o",
		binary,
		testSource,
	)
	if output, err := compile.CombinedOutput(); err != nil {
		t.Fatalf("compile calibration tour checkpoint runtime test: %v\n%s", err, output)
	}
	output, err := exec.CommandContext(ctx, binary).CombinedOutput()
	if err != nil {
		t.Fatalf("run calibration tour checkpoint runtime test: %v\n%s", err, output)
	}
	for _, required := range []string{
		"calibration_tour_checkpoints=pass",
		"checkpoint_count=8",
		"corner_and_home_rows=true",
		"exact_target_clamp=true",
		"failing_corner_holds=true",
		"converged_corner_releases=true",
		"no_progress_timeout_s=20",
		"penultimate_checkpoint_completion_blocked=true",
		"final_crossing_completion_blocked=true",
		"tolerance_rad=0.020",
	} {
		if !strings.Contains(string(output), required) {
			t.Fatalf("calibration tour checkpoint report missing %q: %s", required, output)
		}
	}
}
