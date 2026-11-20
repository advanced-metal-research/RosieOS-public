package motion_server

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

func TestRobotV4MotionServerConfiguredMissingPlannerURDFFailsClosed(
	t *testing.T,
) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "missing-authority-plan.json")
	inputPath := filepath.Join(
		repoRoot,
		"motion-server", "v1",
		"testdata",
		"spreadsheet-plan-input.json",
	)
	missingURDFPath := filepath.Join(t.TempDir(), "active.robot.urdf")

	cmd := exec.Command(
		binary,
		"--self-test",
		"spreadsheet-tesseract-plan",
		"--input",
		inputPath,
		"--output",
		outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	cmd.Env = append(
		os.Environ(),
		"ROBOT_V4_TESSERACT_URDF_PATH="+missingURDFPath,
	)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf(
			"motion-server missing-authority preview failed unexpectedly: %v\n%s",
			err,
			output,
		)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read missing-authority planner report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode missing-authority planner report: %v", err)
	}
	for key, want := range map[string]string{
		"collision_check_state": "unavailable_urdf_not_found",
		"limit_check_state":     "urdf_limits_unavailable",
		"limit_check_source":    "",
	} {
		if got := stringValue(report[key]); got != want {
			t.Fatalf(
				"missing configured URDF report %s = %q, want %q",
				key,
				got,
				want,
			)
		}
	}
	for _, key := range []string{
		"play_enabled",
		"runtime_preload_enabled",
		"collision_gate_clear",
	} {
		if boolValue(report[key]) {
			t.Fatalf(
				"missing configured URDF unexpectedly enabled %s",
				key,
			)
		}
	}
}
