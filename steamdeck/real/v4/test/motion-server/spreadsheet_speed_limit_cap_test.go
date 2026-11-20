package motion_server

import (
	"encoding/json"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

func TestRobotV4MotionServerSpreadsheetSpeedLimitCap(t *testing.T) {
	repoRoot := findRepoRoot(t)
	cases := []struct {
		name      string
		requested []float64
		want      float64
	}{
		{name: "absent defaults to hard limit", want: 0.2},
		{name: "higher request is capped", requested: []float64{0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45}, want: 0.2},
		{name: "lower request is preserved", requested: []float64{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}, want: 0.1},
	}

	for _, testCase := range cases {
		t.Run(testCase.name, func(t *testing.T) {
			request := map[string]any{
				"schema_version":      1,
				"program_id":          "speed-limit-cap",
				"program_revision":    1,
				"samples_per_segment": 2,
				"rows": []map[string]any{
					{
						"row_id": "start", "row_index": 0, "enabled": true,
						"row_type": "motion", "move_type": "joint_ptp",
						"target_frame_type":    "joint_positions",
						"target_joints_rad":    []float64{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						"joint_velocity_rad_s": 0.1,
						"tcp_velocity_m_s":     0.005,
						"playback_step_ms":     0,
					},
					{
						"row_id": "end", "row_index": 1, "enabled": true,
						"row_type": "motion", "move_type": "joint_ptp",
						"target_frame_type":    "joint_positions",
						"target_joints_rad":    []float64{0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
						"joint_velocity_rad_s": 0.1,
						"tcp_velocity_m_s":     0.005,
						"playback_step_ms":     0,
					},
				},
			}
			if testCase.requested != nil {
				request["global_joint_speed_limits_rad_s"] = testCase.requested
			}
			rawRequest, err := json.Marshal(request)
			if err != nil {
				t.Fatalf("encode speed-limit request: %v", err)
			}
			inputPath := filepath.Join(t.TempDir(), "input.json")
			outputPath := filepath.Join(t.TempDir(), "output.json")
			if err := os.WriteFile(inputPath, rawRequest, 0o600); err != nil {
				t.Fatalf("write speed-limit request: %v", err)
			}
			cmd := exec.Command(
				ensureMotionServerCartesianTestBinary(t, repoRoot),
				"--self-test", "spreadsheet-tesseract-plan",
				"--input", inputPath,
				"--output", outputPath,
			)
			cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
			if output, err := cmd.CombinedOutput(); err != nil {
				t.Fatalf("run speed-limit plan: %v\n%s", err, output)
			}
			rawReport, err := os.ReadFile(outputPath)
			if err != nil {
				t.Fatalf("read speed-limit report: %v", err)
			}
			var report map[string]any
			if err := json.Unmarshal(rawReport, &report); err != nil {
				t.Fatalf("decode speed-limit report: %v\n%s", err, rawReport)
			}
			limits := anySlice(report["joint_speed_limits_rad_s"])
			if len(limits) != 9 {
				t.Fatalf("reported %d speed limits, want nine: %s", len(limits), rawReport)
			}
			for axis, limit := range limits {
				if got := numericValue(limit); math.Abs(got-testCase.want) > 1e-9 {
					t.Fatalf("J%d speed limit = %.9f, want %.9f: %s",
						axis+1, got, testCase.want, rawReport)
				}
			}
		})
	}
}
