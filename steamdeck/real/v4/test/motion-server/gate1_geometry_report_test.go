package motion_server

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

func TestRobotV4Gate1PlanReportsEverySampleFKResidual(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "gate1-plan.json")
	inputPath := filepath.Join(
		repoRoot,
		"motion-server", "v1", "testdata",
		"spreadsheet-plan-proportional-singularity-input.json",
	)
	cmd := exec.Command(
		binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("generate Gate 1 geometry plan: %v\n%s", err, output)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatal(err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode Gate 1 geometry plan: %v", err)
	}
	trajectory := anySlice(report["trajectory"])
	if len(trajectory) == 0 ||
		int(numericValue(report["fk_sample_count"])) != len(trajectory) {
		t.Fatalf("Gate 1 FK sample count does not cover the trajectory: %s", raw)
	}
	if !boolValue(report["all_sample_fk_within_tolerance"]) ||
		numericValue(report["max_fk_translation_residual_m"]) >
			numericValue(report["fk_translation_tolerance_m"]) ||
		numericValue(report["max_fk_orientation_residual_rad"]) >
			numericValue(report["fk_orientation_tolerance_rad"]) {
		t.Fatalf(
			"Gate 1 aggregate FK residual exceeded tolerance: translation=%g/%g m orientation=%g/%g rad all=%t",
			numericValue(report["max_fk_translation_residual_m"]),
			numericValue(report["fk_translation_tolerance_m"]),
			numericValue(report["max_fk_orientation_residual_rad"]),
			numericValue(report["fk_orientation_tolerance_rad"]),
			boolValue(report["all_sample_fk_within_tolerance"]),
		)
	}
	for index, sampleAny := range trajectory {
		sample, ok := sampleAny.(map[string]any)
		if !ok ||
			len(anySlice(sample["tcp_pose"])) != 6 ||
			len(anySlice(sample["fk_tcp_pose"])) != 6 ||
			!boolValue(sample["fk_within_tolerance"]) {
			t.Fatalf("Gate 1 FK sample %d is incomplete: %#v", index, sampleAny)
		}
	}
}
