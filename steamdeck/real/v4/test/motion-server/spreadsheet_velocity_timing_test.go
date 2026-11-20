package motion_server

import (
	"encoding/json"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func runSpreadsheetVelocityTimingPlan(
	t *testing.T,
	repoRoot string,
	name string,
	rows []map[string]any,
) (map[string]any, []byte) {
	t.Helper()
	request := map[string]any{
		"schema_version":                  1,
		"program_id":                      "velocity-timing-" + name,
		"program_revision":                1,
		"samples_per_segment":             2,
		"global_joint_speed_limits_rad_s": []float64{1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5},
		"rows":                            rows,
	}
	rawRequest, err := json.Marshal(request)
	if err != nil {
		t.Fatalf("encode %s velocity timing request: %v", name, err)
	}
	inputPath := filepath.Join(t.TempDir(), name+"-input.json")
	outputPath := filepath.Join(t.TempDir(), name+"-output.json")
	if err := os.WriteFile(inputPath, rawRequest, 0o600); err != nil {
		t.Fatalf("write %s velocity timing request: %v", name, err)
	}
	cmd := exec.Command(
		ensureMotionServerCartesianTestBinary(t, repoRoot),
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("run %s velocity timing plan: %v\n%s", name, err, output)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read %s velocity timing report: %v", name, err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode %s velocity timing report: %v\n%s", name, err, raw)
	}
	return report, raw
}

func spreadsheetVelocityTimingLineRows(wireFeed, voltage float64) []map[string]any {
	return []map[string]any{
		{
			"row_id": "line-start", "row_index": 0, "enabled": true,
			"row_type": "motion", "move_type": "line_tcp",
			"target_frame_type":   "tcp_cartesian",
			"target_tcp_m":        []float64{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"tcp_velocity_m_s":    5.0,
			"playback_step_ms":    0,
			"wire_feed_m_per_min": wireFeed,
			"voltage_v":           voltage,
		},
		{
			"row_id": "line-end", "row_index": 1, "enabled": true,
			"row_type": "motion", "move_type": "line_tcp",
			"target_frame_type": "tcp_cartesian",
			"target_tcp_m":      []float64{1.0, 0.01, 0.0, 0.0, 0.0, 0.0},
			"tcp_velocity_m_s":  0.001,
			"playback_step_ms":  0,
		},
	}
}

func spreadsheetVelocityTimingJointRows() []map[string]any {
	return []map[string]any{
		{
			"row_id": "joint-start", "row_index": 0, "enabled": true,
			"row_type": "motion", "move_type": "joint_ptp",
			"target_frame_type":    "joint_positions",
			"target_joints_rad":    []float64{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"joint_velocity_rad_s": 5.0,
			"tcp_velocity_m_s":     0.005,
			"playback_step_ms":     0,
		},
		{
			"row_id": "joint-override", "row_index": 1, "enabled": true,
			"row_type": "motion", "move_type": "joint_ptp",
			"target_frame_type":    "joint_positions",
			"target_joints_rad":    []float64{0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"joint_velocity_rad_s": 0.0,
			"tcp_velocity_m_s":     0.0,
			"playback_step_ms":     2500,
		},
		{
			"row_id": "joint-end", "row_index": 2, "enabled": true,
			"row_type": "motion", "move_type": "joint_ptp",
			"target_frame_type":    "joint_positions",
			"target_joints_rad":    []float64{0.20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"joint_velocity_rad_s": 0.001,
			"playback_step_ms":     0,
		},
	}
}

func spreadsheetPlanFirstRowTime(t *testing.T, report map[string]any, row int) float64 {
	t.Helper()
	for _, entry := range anySlice(report["trajectory"]) {
		point, ok := entry.(map[string]any)
		if ok && int(numericValue(point["row_index"])) == row {
			return numericValue(point["time_s"])
		}
	}
	t.Fatalf("trajectory has no boundary for row %d: %#v", row, report["trajectory"])
	return 0
}

func spreadsheetPlanTimes(report map[string]any) []float64 {
	trajectory := anySlice(report["trajectory"])
	times := make([]float64, 0, len(trajectory))
	for _, entry := range trajectory {
		point, _ := entry.(map[string]any)
		times = append(times, numericValue(point["time_s"]))
	}
	return times
}

func TestRobotV4MotionServerNonLoopJointMoveEndsAtRequestedTarget(t *testing.T) {
	repoRoot := findRepoRoot(t)
	start := []float64{0.10, -0.05, 0.08, 0.02, -0.04, 0.06, 0.01, -0.02, 0.01}
	target := []float64{0.20, -0.10, 0.15, 0.05, -0.08, 0.12, 0.03, -0.04, 0.02}
	rows := []map[string]any{
		{
			"row_id": "move-start", "row_index": 0, "enabled": true,
			"row_type": "motion", "move_type": "joint_ptp",
			"target_frame_type":    "joint_positions",
			"target_joints_rad":    start,
			"target_tcp_m":         []float64{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"joint_velocity_rad_s": 0.5,
		},
		{
			"row_id": "move-target", "row_index": 1, "enabled": true,
			"row_type": "motion", "move_type": "joint_ptp",
			"target_frame_type":    "joint_positions",
			"target_joints_rad":    target,
			"target_tcp_m":         []float64{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			"joint_velocity_rad_s": 0.5,
		},
	}

	report, raw := runSpreadsheetVelocityTimingPlan(t, repoRoot, "non-loop-joint-target", rows)
	if boolValue(report["loop_command_present"]) {
		t.Fatalf("two-row joint move was incorrectly treated as a loop: %s", raw)
	}
	trajectory := anySlice(report["trajectory"])
	if len(trajectory) < 3 {
		t.Fatalf("joint move has %d samples, want a timed trajectory plus terminal hold: %s", len(trajectory), raw)
	}
	first, firstOK := trajectory[0].(map[string]any)
	last, lastOK := trajectory[len(trajectory)-1].(map[string]any)
	if !firstOK || !lastOK {
		t.Fatalf("joint move trajectory samples are malformed: %s", raw)
	}
	firstPositions := anySlice(first["positions_rad"])
	if len(firstPositions) != len(target) {
		t.Fatalf("joint move start has %d axes, want %d: %s", len(firstPositions), len(target), raw)
	}
	for axis, want := range start {
		if got := numericValue(firstPositions[axis]); math.Abs(got-want) > 1e-9 {
			t.Fatalf("joint move start J%d=%.9f, want %.9f: %s", axis+1, got, want, raw)
		}
	}
	lastPositions := anySlice(last["positions_rad"])
	if len(lastPositions) != len(target) {
		t.Fatalf("joint move endpoint has %d axes, want %d: %s", len(lastPositions), len(target), raw)
	}
	for axis, want := range target {
		if got := numericValue(lastPositions[axis]); math.Abs(got-want) > 1e-9 {
			t.Fatalf("joint move endpoint J%d=%.9f, want %.9f: %s", axis+1, got, want, raw)
		}
	}
	if numericValue(last["time_s"]) <= numericValue(first["time_s"]) {
		t.Fatalf("joint move has no positive elapsed time: %s", raw)
	}
}

func TestRobotV4MotionServerSpreadsheetVelocityTiming(t *testing.T) {
	repoRoot := findRepoRoot(t)

	t.Run("cartesian velocity and hard joint limit drive zero-step timing", func(t *testing.T) {
		report, raw := runSpreadsheetVelocityTimingPlan(
			t, repoRoot, "cartesian", spreadsheetVelocityTimingLineRows(5.5, 24.0))
		rows := anySlice(report["program_rows"])
		first, ok := rows[0].(map[string]any)
		last, lastOK := rows[1].(map[string]any)
		if !ok || !lastOK ||
			math.Abs(numericValue(first["tcp_velocity_m_s"])-0.5) > 1e-9 ||
			math.Abs(numericValue(last["tcp_velocity_m_s"])-0.005) > 1e-9 {
			t.Fatalf("TCP velocities were not clamped to Deck range 0.005..0.5 m/s: %s", raw)
		}
		if got := spreadsheetPlanFirstRowTime(t, report, 1); got <= 0.02+1e-6 {
			t.Fatalf("Cartesian row boundary time = %.9f, want slower than the unconstrained 0.02 s after the 0.2 rad/s joint cap: %s", got, raw)
		}
		limits := anySlice(report["joint_speed_limits_rad_s"])
		maxSpeeds := anySlice(report["max_joint_speeds_rad_s"])
		if len(limits) != 9 || len(maxSpeeds) != 9 {
			t.Fatalf("Cartesian plan did not report nine-axis speed-limit evidence: %s", raw)
		}
		for axis := range limits {
			if limit := numericValue(limits[axis]); math.Abs(limit-0.2) > 1e-9 ||
				numericValue(maxSpeeds[axis]) > limit+1e-6 {
				t.Fatalf("Cartesian plan J%d speed %.9f exceeds 0.2 rad/s limit %.9f: %s",
					axis+1, numericValue(maxSpeeds[axis]), limit, raw)
			}
		}
	})

	t.Run("joint velocity drives timing and explicit step overrides it", func(t *testing.T) {
		report, raw := runSpreadsheetVelocityTimingPlan(
			t, repoRoot, "joint", spreadsheetVelocityTimingJointRows())
		rows := anySlice(report["program_rows"])
		first, ok := rows[0].(map[string]any)
		last, lastOK := rows[2].(map[string]any)
		if !ok || !lastOK ||
			math.Abs(numericValue(first["joint_velocity_rad_s"])-1.5) > 1e-9 ||
			math.Abs(numericValue(last["joint_velocity_rad_s"])-0.01) > 1e-9 {
			t.Fatalf("joint velocities were not clamped to Deck range 0.01..1.5 rad/s: %s", raw)
		}
		firstSegmentEnd := spreadsheetPlanFirstRowTime(t, report, 1)
		secondSegmentEnd := spreadsheetPlanFirstRowTime(t, report, 2)
		if math.Abs(firstSegmentEnd-0.75) > 1e-6 {
			t.Fatalf("joint row boundary time = %.9f, want max joint delta 0.15 / hard limit 0.2 = 0.75 s: %s", firstSegmentEnd, raw)
		}
		if got := secondSegmentEnd - firstSegmentEnd; math.Abs(got-2.5) > 1e-6 {
			t.Fatalf("explicit playback step duration = %.9f, want 2.5 s regardless of zero row velocity: %s", got, raw)
		}
	})

	t.Run("weld metadata round trips without changing timing", func(t *testing.T) {
		firstReport, firstRaw := runSpreadsheetVelocityTimingPlan(
			t, repoRoot, "metadata-a", spreadsheetVelocityTimingLineRows(5.5, 24.0))
		secondReport, secondRaw := runSpreadsheetVelocityTimingPlan(
			t, repoRoot, "metadata-b", spreadsheetVelocityTimingLineRows(17.3, 31.7))
		firstRows := anySlice(firstReport["program_rows"])
		secondRows := anySlice(secondReport["program_rows"])
		firstRow, firstOK := firstRows[0].(map[string]any)
		secondRow, secondOK := secondRows[0].(map[string]any)
		if !firstOK || !secondOK ||
			math.Abs(numericValue(firstRow["wire_feed_m_per_min"])-5.5) > 1e-9 ||
			math.Abs(numericValue(firstRow["voltage_v"])-24.0) > 1e-9 ||
			math.Abs(numericValue(secondRow["wire_feed_m_per_min"])-17.3) > 1e-9 ||
			math.Abs(numericValue(secondRow["voltage_v"])-31.7) > 1e-9 {
			t.Fatalf("weld metadata did not round trip exactly:\nfirst=%s\nsecond=%s", firstRaw, secondRaw)
		}
		firstTimes := spreadsheetPlanTimes(firstReport)
		secondTimes := spreadsheetPlanTimes(secondReport)
		if len(firstTimes) != len(secondTimes) {
			t.Fatalf("metadata-only edit changed trajectory size: %d vs %d", len(firstTimes), len(secondTimes))
		}
		for index := range firstTimes {
			if math.Abs(firstTimes[index]-secondTimes[index]) > 1e-9 {
				t.Fatalf("metadata-only edit changed waypoint %d time: %.9f vs %.9f", index, firstTimes[index], secondTimes[index])
			}
		}
	})

	t.Run("unconfigured velocity-timed segment fails closed", func(t *testing.T) {
		rows := spreadsheetVelocityTimingLineRows(0.0, 0.0)
		rows[0]["tcp_velocity_m_s"] = 0.0
		request := map[string]any{
			"schema_version": 1,
			"program_id":     "velocity-timing-invalid",
			"rows":           rows,
		}
		rawRequest, err := json.Marshal(request)
		if err != nil {
			t.Fatal(err)
		}
		inputPath := filepath.Join(t.TempDir(), "invalid-input.json")
		outputPath := filepath.Join(t.TempDir(), "invalid-output.json")
		if err := os.WriteFile(inputPath, rawRequest, 0o600); err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(
			ensureMotionServerCartesianTestBinary(t, repoRoot),
			"--self-test", "spreadsheet-tesseract-plan",
			"--input", inputPath,
			"--output", outputPath,
		)
		cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
		output, err := cmd.CombinedOutput()
		if err == nil || !strings.Contains(string(output), "failed to generate a dense plan") {
			t.Fatalf("zero velocity-timed segment did not fail closed: err=%v output=%s", err, output)
		}
	})
}
