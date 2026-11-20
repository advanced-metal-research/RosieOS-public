package motion_server

import (
	"encoding/json"
	"math"

	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"testing"
)

const (
	joystickIntentFrameToolForMotionServerTest     = byte(1)
	joystickIntentFrameJointForMotionServerTest    = byte(2)
	jogFlagActiveForMotionServerTest               = uint32(1 << 0)
	jogFlagCountsPerSecondForMotionServerTest      = uint32(1 << 1)
	jogFlagCSVMoveModeCandidateForMotionServerTest = uint32(1 << 2)
)

func TestRobotV4MotionServerCartesianSolverDryRun(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "cartesian-io-report.json")
	inputPath := filepath.Join(repoRoot, "motion-server", "v1", "testdata", "cartesian-input.json")

	cmd := exec.Command(binary,
		"--self-test", "cartesian-io",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server cartesian solver dry-run failed: %v\n%s", err, string(output))
	}

	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read cartesian solver report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode cartesian solver report: %v\n%s", err, string(raw))
	}
	for key, want := range map[string]string{
		"name":                 "robot-v4-cartesiand",
		"self_test":            "cartesian-io",
		"input_source":         "cartesian_velocity_vector",
		"input_control_space":  "cartesian_tcp_velocity",
		"output_sink":          "rtcore_joint_velocity_lease",
		"rtcore_control_space": "joint_velocity_lease",
		"rtcore_msg_type":      "0x0130",
		"axis_mask":            "0x3f",
	} {
		if got := strings.TrimSpace(stringValue(report[key])); got != want {
			t.Fatalf("cartesian solver report %s = %q, want %q; report=%s", key, got, want, string(raw))
		}
	}
	if boolValue(report["accepted"]) != true {
		t.Fatalf("cartesian solver report accepted = %v, want true; report=%s", report["accepted"], string(raw))
	}
	if boolValue(report["hardware_access"]) != false {
		t.Fatalf("cartesian solver report hardware_access = %v, want false; report=%s", report["hardware_access"], string(raw))
	}
	if numericValue(report["timeout_ns"]) != 120000000 {
		t.Fatalf("cartesian solver timeout_ns = %v, want 120000000; report=%s", report["timeout_ns"], string(raw))
	}
	jointVelocity, ok := report["joint_velocity_rad_s"].([]any)
	if !ok || len(jointVelocity) != 6 {
		t.Fatalf("cartesian solver joint_velocity_rad_s = %#v, want six joints; report=%s", report["joint_velocity_rad_s"], string(raw))
	}
	maxAbs := math.Abs(numericValue(report["max_abs_qd_rad_s"]))
	if maxAbs <= 0 || maxAbs > 0.25 {
		t.Fatalf("cartesian solver max_abs_qd_rad_s = %.6f, want within safe dry-run range; report=%s", maxAbs, string(raw))
	}
}

func TestRobotV4MotionServerRejectsRolesInTransportIdentity(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	tests := []struct {
		name string
		args []string
		want string
	}{
		{
			name: "nats userinfo cannot hide a role",
			args: []string{"--nats-url", "nats://motion-server@other-relay:4222"},
			want: "NATS URL must use peer/role/service or a concrete host",
		},
		{
			name: "known source wrapper cannot replace peer",
			args: []string{"--nats-url", "daemon-v1://local:motion-server/motion-server/robot-v4-nats"},
			want: "NATS URL must use peer/role/service or a concrete host",
		},
		{
			name: "arbitrary source wrapper cannot replace peer",
			args: []string{"--nats-url", "daemon-v1://iroh-gossip-v1:realtime-core/motion-server/robot-v4-nats"},
			want: "NATS URL must use peer/role/service or a concrete host",
		},
		{
			name: "subject host must be a concrete peer",
			args: []string{"--nats-status-subject", "robot/v4/motion-server.iroh-gossip-v1:motion-server.status"},
			want: "subject must use the selected concrete motion-server peer",
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			args := []string{
				"--self-test", "solver-speed",
				"--iterations", "1",
				"--output", filepath.Join(t.TempDir(), "identity-rejection.json"),
			}
			args = append(args, tt.args...)
			cmd := exec.Command(binary, args...)
			cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
			cmd.Env = append(os.Environ(),
				"NO_REAL_MOVEMENT=1",
				"ROBOT_V4_SIMULATION_MODE=1",
			)
			output, err := cmd.CombinedOutput()
			if err == nil {
				t.Fatalf("motion-server accepted invalid transport identity %v\n%s", tt.args, string(output))
			}
			if !strings.Contains(string(output), tt.want) {
				t.Fatalf("motion-server rejection for %v did not contain %q\n%s", tt.args, tt.want, string(output))
			}
		})
	}
}

func TestRobotV4MotionServerKinematicsUseURDFSourceOfTruth(t *testing.T) {
	repoRoot := findRepoRoot(t)
	cpp := readMotionServerCartesianCompilationSource(t, repoRoot)
	for _, forbidden := range []string{
		"DHParam",
		"kGradient05DH",
		"standard_dh_transform",
		"standard_dh_forward_kinematics",
		"standard_dh_damped_least_squares",
		"kMotionServerFallbackArmURDFJoints",
		"kMotionServerFallbackEndEffectorURDFJoint",
		"static const std::array<Rosie1400URDFJoint, 6> joints = load_motion_server_arm_urdf_joints()",
		"static const Rosie1400URDFJoint joint = load_motion_server_end_effector_urdf_joint()",
	} {
		if strings.Contains(cpp, forbidden) {
			t.Fatalf("motion-server kinematics must not retain %q; URDF is the source of truth", forbidden)
		}
	}
	for _, required := range []string{
		"parse_urdf_joint",
		"resolve_spreadsheet_planner_urdf_path",
		"motion_server_kinematics_snapshot_for_use",
		"motion_server_authoritative_urdf_path",
		"motion_server_kinematics_binding_current",
		"rtcore_runtime_authority_identity_matches",
		"generated_urdf_sha256",
		"urdf_forward_kinematics_j1_base",
		"urdf_chain_damped_least_squares",
	} {
		if !strings.Contains(cpp, required) {
			t.Fatalf("motion-server URDF kinematics source missing %q", required)
		}
	}

	urdfPath := filepath.Join(repoRoot, "robot_description", "robots", "rosie_1400_v3", "robot.urdf")
	outputPath := filepath.Join(t.TempDir(), "kinematics-authority.json")
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	cmd := exec.Command(
		binary,
		"--self-test", "kinematics-authority",
		"--input", urdfPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server kinematics authority self-test failed: %v\n%s", err, output)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read motion-server kinematics authority report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode motion-server kinematics authority report: %v\n%s", err, raw)
	}
	for _, key := range []string{
		"unconfigured_path_rejected",
		"initial_sha_match_loaded",
		"tampered_same_authority_rejected",
		"invalid_snapshot_not_exposed",
		"stale_pair_rejected",
		"pair_switch_reloaded",
		"geometry_changed",
		"missing_rejected",
		"malformed_exact_hash_rejected",
		"passed",
	} {
		if !boolValue(report[key]) {
			t.Fatalf("kinematics authority report did not prove %s; report=%s", key, raw)
		}
	}
	if boolValue(report["compiled_fallback_used"]) {
		t.Fatalf("kinematics authority report used compiled fallback geometry; report=%s", raw)
	}
}

func TestRobotV4MotionServerSpreadsheetTesseractPlanDryRun(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "spreadsheet-tesseract-plan.json")
	inputPath := filepath.Join(repoRoot, "motion-server", "v1", "testdata", "spreadsheet-plan-input.json")

	cmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server spreadsheet Tesseract plan dry-run failed: %v\n%s", err, string(output))
	}

	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read spreadsheet Tesseract plan report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode spreadsheet Tesseract plan report: %v\n%s", err, string(raw))
	}
	for key, want := range map[string]string{
		"name":                             "robot-v4-cartesiand",
		"self_test":                        "spreadsheet-tesseract-plan",
		"planner_owner":                    "motion-server",
		"planner_backend":                  "motion_server_tesseract_trajectory_v1",
		"trajectory_method":                "linear-tcp-dense-csp",
		"integration":                      "motion_server_owned_planner_for_spreadsheet_playback",
		"program_source":                   "client-authored-program-rows",
		"jogging_solver":                   "direct_motion_server_cartesian_or_joint_solver",
		"playback_solver":                  "tesseract_dense_trajectory",
		"planner_status":                   "planned",
		"robotic-cell-type":                "rosie_1400_v3",
		"robot_model":                      "rosie_1400_v3",
		"tesseract_urdf_source":            "robot_description/robots/rosie_1400_v3/robot.urdf",
		"program_state":                    "planned",
		"limit_check_state":                "clear",
		"collision_joint_mapping":          "urdf_joints_equal_plan_positions_rad",
		"tesseract_collision_geometry":     "visual_meshes_as_collision_meshes",
		"tesseract_collision_mesh_package": "package://rosie_1400_v3/meshes",
	} {
		if got := strings.TrimSpace(stringValue(report[key])); got != want {
			t.Fatalf("spreadsheet Tesseract plan report %s = %q, want %q; report=%s", key, got, want, string(raw))
		}
	}
	requireHonestSpreadsheetPlanValidation(t, report, raw)
	if numericValue(report["database_schema_version"]) != 1 ||
		numericValue(report["program_revision"]) != 1 ||
		numericValue(report["source_row_count"]) != 4 ||
		numericValue(report["samples_per_segment"]) != 5 ||
		numericValue(report["planned_sample_count"]) < 16 ||
		numericValue(report["tool_path_length_m"]) < 1.49 ||
		numericValue(report["duration_s"]) <= 0 ||
		numericValue(report["max_tcp_step_m"]) > 0.11 ||
		numericValue(report["max_joint_step_rad"]) > 0.22 ||
		numericValue(report["max_tcp_fk_error_m"]) > 0.005 ||
		boolValue(report["smooth_dense_trajectory"]) != true ||
		boolValue(report["accepted"]) != true ||
		boolValue(report["hardware_access"]) != false {
		t.Fatalf("spreadsheet Tesseract plan metrics mismatch: %s", string(raw))
	}
	fields, ok := report["spreadsheet_database_fields"].([]any)
	if !ok ||
		!containsAnyString(fields, "program_name") ||
		!containsAnyString(fields, "robotic-cell-type") ||
		!containsAnyString(fields, "target_tcp_m") ||
		!containsAnyString(fields, "fixture_side") ||
		!containsAnyString(fields, "current_joints_rad") ||
		!containsAnyString(fields, "planned_sample_count") ||
		!containsAnyString(fields, "last_test_session_id") {
		t.Fatalf("spreadsheet Tesseract plan database fields missing expected concepts: %s", string(raw))
	}
	trajectory, ok := report["trajectory"].([]any)
	if !ok || len(trajectory) < 16 {
		t.Fatalf("spreadsheet Tesseract trajectory = %#v, want dense row-indexed samples; report=%s", report["trajectory"], string(raw))
	}
	first, ok := trajectory[0].(map[string]any)
	if !ok || numericValue(first["row_index"]) != 0 {
		t.Fatalf("first trajectory sample = %#v, want row_index 0", trajectory[0])
	}
	last, ok := trajectory[len(trajectory)-1].(map[string]any)
	if !ok || numericValue(last["row_index"]) != 3 {
		t.Fatalf("last trajectory sample = %#v, want row_index 3", trajectory[len(trajectory)-1])
	}
	// Weld timeline: input row 1 carries weld_on and row 2 weld_off, so the
	// arc must be on exactly for the samples of the segment departing row 1
	// and off before and after. Segment sampling is adaptive (TCP/joint step
	// limits may require more than samples_per_segment), so derive the expected
	// count from the row-boundary samples instead of hard-coding the minimum.
	weldOnSamples := 0
	rowOneBoundary := -1
	rowTwoBoundary := -1
	for index, entry := range trajectory {
		sample, ok := entry.(map[string]any)
		if !ok {
			t.Fatalf("trajectory sample missing object shape: %#v", entry)
		}
		if _, present := sample["weld_on"]; !present {
			t.Fatalf("trajectory sample missing weld_on field: %#v", sample)
		}
		if boolValue(sample["weld_on"]) {
			weldOnSamples++
		}
		row := int(numericValue(sample["row_index"]))
		if row == 1 && rowOneBoundary < 0 {
			rowOneBoundary = index
		}
		if row == 2 && rowTwoBoundary < 0 {
			rowTwoBoundary = index
		}
	}
	wantWeldOnSamples := rowTwoBoundary - rowOneBoundary
	if rowOneBoundary < 0 || rowTwoBoundary <= rowOneBoundary ||
		weldOnSamples != wantWeldOnSamples {
		t.Fatalf("weld_on sample count = %d, want adaptive row-1 segment count %d (boundaries %d..%d); report=%s",
			weldOnSamples, wantWeldOnSamples, rowOneBoundary, rowTwoBoundary, string(raw))
	}
	if boolValue(first["weld_on"]) || boolValue(last["weld_on"]) {
		t.Fatalf("weld arc must be off at plan start and end; report=%s", string(raw))
	}
}

func TestRobotV4MotionServerTableBoxSideBUsesJ8AndJ9Selector(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "table-box-side-b-plan.json")
	inputPath := filepath.Join(repoRoot, "motion-server", "v1", "testdata", "table-box-side-b-plan-request.json")
	cmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--manifest", filepath.Join(repoRoot, "rt-core", "config", "cells", "other-relay.json"),
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = repoRoot
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server B-side table-box plan failed: %v\n%s", err, string(output))
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read B-side table-box plan: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode B-side table-box plan: %v\n%s", err, raw)
	}
	if boolValue(report["accepted"]) ||
		!boolValue(report["harmonious_external_axis_valid"]) ||
		stringValue(report["harmonious_external_axis_joint"]) != "J8" ||
		stringValue(report["harmonious_active_side"]) != "B" ||
		stringValue(report["harmonious_selector_joint"]) != "J9" ||
		math.Abs(numericValue(report["harmonious_selector_rad"])-math.Pi) > 1e-5 ||
		stringValue(report["harmonious_work_frame"]) != "positioner_table_b" ||
		boolValue(report["program_structure_valid"]) ||
		int(numericValue(report["program_row_count"])) != 3 ||
		int(numericValue(report["motion_row_count"])) != 2 ||
		int(numericValue(report["calibration_section_count"])) != 0 ||
		int(numericValue(report["reconfiguration_row_count"])) != 0 ||
		!boolValue(report["starts_at_home"]) ||
		!boolValue(report["returns_home"]) ||
		!boolValue(report["loop_command_present"]) ||
		!boolValue(report["loop_command_valid"]) ||
		stringValue(report["validation_state"]) != "checks_unavailable" ||
		boolValue(report["motion_allowed"]) ||
		boolValue(report["play_enabled"]) ||
		boolValue(report["runtime_preload_enabled"]) ||
		boolValue(report["isolated_simulation_preload_enabled"]) ||
		!boolValue(report["simulation_preview_enabled"]) ||
		math.Abs(numericValue(report["harmonious_external_axis_min_rad"])) > 1e-9 ||
		math.Abs(numericValue(report["harmonious_external_axis_max_rad"])) > 1e-9 ||
		numericValue(report["max_workplane_relative_error_m"]) > 1e-9 ||
		numericValue(report["unreachable_point_count"]) != 0 {
		t.Fatalf("unsupported B-side canonical program did not fail closed: %s", raw)
	}
	rows := anySlice(report["program_rows"])
	trajectory := anySlice(report["trajectory"])
	if len(rows) != 3 || len(trajectory) < 2 || int(numericValue(report["planned_sample_count"])) != len(trajectory) {
		t.Fatalf("unsupported B-side sentinel preview mismatch rows=%d samples=%d: %s", len(rows), len(trajectory), raw)
	}
	wantLabels := []string{"StartUnsupportedSide", "ReturnStartUnsupportedSide", "LOOP"}
	for index, rowAny := range rows {
		row, ok := rowAny.(map[string]any)
		if !ok || stringValue(row["fixture_side"]) != "B" ||
			stringValue(row["work_frame"]) != "positioner_table_b" ||
			stringValue(row["label"]) != wantLabels[index] {
			t.Fatalf("unsupported B-side row %d lost sentinel/side/frame identity: %#v", index, rowAny)
		}
		if index < 2 {
			target := anySlice(row["target_joints_rad"])
			if stringValue(row["row_type"]) != "motion" ||
				stringValue(row["move_type"]) != "joint_ptp" ||
				len(target) != 9 ||
				math.Abs(numericValue(target[7])) > 1e-9 ||
				math.Abs(numericValue(target[8])-math.Pi) > 1e-5 {
				t.Fatalf("unsupported B-side sentinel row %d must hold J8 and select Table B with J9: %#v", index, rowAny)
			}
		} else if stringValue(row["row_type"]) != "command" ||
			stringValue(row["command"]) != "LOOP" ||
			int(numericValue(row["loop_target_row"])) != 0 ||
			int(numericValue(row["loop_max_iterations"])) != 1 ||
			!boolValue(row["loop_return_home"]) {
			t.Fatalf("unsupported B-side terminal LOOP lost its bounded fail-closed contract: %#v", rowAny)
		}
	}
	for index, sampleAny := range trajectory {
		sample, ok := sampleAny.(map[string]any)
		positions := anySlice(sample["positions_rad"])
		if !ok || len(positions) != 9 ||
			math.Abs(numericValue(positions[7])) > 1e-9 ||
			boolValue(sample["harmonious_workplane_active"]) {
			t.Fatalf("unsupported B-side preview sample %d moved J8 or activated the workplane trace: %#v", index, sampleAny)
		}
	}
	lastSample, ok := trajectory[len(trajectory)-1].(map[string]any)
	lastPositions := anySlice(lastSample["positions_rad"])
	if !ok || len(lastPositions) != 9 ||
		math.Abs(numericValue(lastPositions[7])) > 1e-9 ||
		math.Abs(numericValue(lastPositions[8])-math.Pi) > 1e-5 {
		t.Fatalf("unsupported B-side preview did not finish at held J8 and the J9 Table B selector: %#v", lastSample)
	}

	invalidOutputPath := filepath.Join(t.TempDir(), "table-box-invalid-side-plan.json")
	invalidInputPath := filepath.Join(repoRoot, "motion-server", "v1", "testdata", "table-box-invalid-side-mismatch-plan-request.json")
	invalidCmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--manifest", filepath.Join(repoRoot, "rt-core", "config", "cells", "other-relay.json"),
		"--input", invalidInputPath,
		"--output", invalidOutputPath,
	)
	invalidCmd.Dir = repoRoot
	if output, err := invalidCmd.CombinedOutput(); err != nil {
		t.Fatalf("run invalid A/B mismatch plan report: %v\n%s", err, output)
	}
	invalidRaw, err := os.ReadFile(invalidOutputPath)
	if err != nil {
		t.Fatalf("read invalid A/B mismatch plan report: %v", err)
	}
	var invalidReport map[string]any
	if err := json.Unmarshal(invalidRaw, &invalidReport); err != nil {
		t.Fatalf("decode invalid A/B mismatch report: %v\n%s", err, invalidRaw)
	}
	if boolValue(invalidReport["accepted"]) ||
		boolValue(invalidReport["harmonious_external_axis_valid"]) ||
		stringValue(invalidReport["harmonious_external_axis_validation_error"]) != "active_side_workplane_mismatch" {
		t.Fatalf("mismatched side/joint/frame tuple did not fail closed: %s", invalidRaw)
	}
}

func requireHonestSpreadsheetPlanValidation(t *testing.T, report map[string]any, raw []byte) {
	t.Helper()
	if len(stringValue(report["program_hash"])) != len("fnv1a64:")+16 ||
		!strings.HasPrefix(stringValue(report["program_hash"]), "fnv1a64:") ||
		int(numericValue(report["program_row_count"])) != len(anySlice(report["program_rows"])) {
		t.Fatalf("plan report missing stable program hash/row count identity: %s", string(raw))
	}
	unreachable := anySlice(report["unreachable_points"])
	if int(numericValue(report["unreachable_point_count"])) != len(unreachable) {
		t.Fatalf("unreachable_point_count %v does not match unreachable_points length %d; report=%s",
			report["unreachable_point_count"], len(unreachable), string(raw))
	}
	collisions := anySlice(report["collision_points"])
	if int(numericValue(report["collision_point_count"])) != len(collisions) {
		t.Fatalf("collision_point_count %v does not match collision_points length %d; report=%s",
			report["collision_point_count"], len(collisions), string(raw))
	}
	trajectory := anySlice(report["trajectory"])
	requireSpreadsheetPlanSafetyReport(t, report, trajectory, raw)
	for index, entry := range collisions {
		point, ok := entry.(map[string]any)
		_, hasRowIndex := point["row_index"]
		_, hasWaypointIndex := point["waypoint_index"]
		_, hasTime := point["time_s"]
		waypointIndex := int(numericValue(point["waypoint_index"]))
		if !ok || !hasRowIndex || !hasWaypointIndex || !hasTime ||
			stringValue(point["link_a"]) == "" || stringValue(point["link_b"]) == "" ||
			waypointIndex < 0 || waypointIndex >= len(trajectory) || numericValue(point["time_s"]) < 0 {
			t.Fatalf("collision point %d missing row/waypoint/time/link mapping: %#v", index, entry)
		}
		sample, sampleOK := trajectory[waypointIndex].(map[string]any)
		affectedJoints := anySlice(point["affected_joints"])
		if !sampleOK || int(numericValue(point["row_index"])) != int(numericValue(sample["row_index"])) ||
			math.Abs(numericValue(point["time_s"])-numericValue(sample["time_s"])) > 1e-6 {
			t.Fatalf("collision point %d does not map to its authoritative trajectory sample: point=%#v sample=%#v", index, point, sample)
		}
		if len(affectedJoints) == 0 {
			t.Fatalf("collision point %d does not name the actuated joints affected by %s<->%s: %#v",
				index, stringValue(point["link_a"]), stringValue(point["link_b"]), point)
		}
		for _, joint := range affectedJoints {
			name := stringValue(joint)
			if len(name) != 2 || name[0] != 'J' || name[1] < '1' || name[1] > '9' {
				t.Fatalf("collision point %d affected_joints contains invalid Robot v4 joint %q: %#v", index, name, point)
			}
		}
	}
	if numericValue(report["planning_time_s"]) <= 0 {
		t.Fatalf("planning_time_s must be measured, got %v; report=%s", report["planning_time_s"], string(raw))
	}
	validation := stringValue(report["validation_state"])
	state := stringValue(report["collision_check_state"])
	if state == "tesseract_discrete_contact_checked" {
		if stringValue(report["collision_checker"]) != "BulletDiscreteBVHManager" {
			t.Fatalf("collision_checker = %q, want BulletDiscreteBVHManager; report=%s",
				stringValue(report["collision_checker"]), string(raw))
		}
		if stringValue(report["collision_checker_library_version"]) == "" {
			t.Fatalf("collision_checker_library_version missing; report=%s", string(raw))
		}
		if int(numericValue(report["collision_checked_waypoint_count"])) != int(numericValue(report["planned_sample_count"])) {
			t.Fatalf("collision check covered %v waypoints, want every planned sample %v; report=%s",
				report["collision_checked_waypoint_count"], report["planned_sample_count"], string(raw))
		}
		wantValidation := "valid"
		if len(unreachable) > 0 || len(collisions) > 0 ||
			numericValue(report["singularity_hard_stop_point_count"]) > 0 {
			wantValidation = "invalid"
		} else if !boolValue(report["singularity_analysis_available"]) {
			wantValidation = "checks_unavailable"
		}
		if validation != wantValidation {
			t.Fatalf("validation_state = %q, want %q for %d unreachable and %d collision points; report=%s",
				validation, wantValidation, len(unreachable), len(collisions), string(raw))
		}
		for index, entry := range collisions {
			point, ok := entry.(map[string]any)
			if !ok ||
				stringValue(point["link_a"]) == "" ||
				stringValue(point["link_b"]) == "" ||
				numericValue(point["depth_m"]) <= 0 ||
				len(anySlice(point["tcp_pose"])) != 6 {
				t.Fatalf("collision point %d malformed: %#v; report=%s", index, entry, string(raw))
			}
		}
	} else {
		if len(collisions) > 0 {
			t.Fatalf("collision_points populated without a completed contact check (%q); report=%s", state, string(raw))
		}
		if len(unreachable) == 0 && numericValue(report["singularity_hard_stop_point_count"]) == 0 &&
			validation != "checks_unavailable" {
			t.Fatalf("validation_state = %q, want checks_unavailable when the collision check did not run (%q); report=%s",
				validation, state, string(raw))
		}
		t.Logf("MOTION_SERVER_COLLISION_WORKER_UNAVAILABLE state=%q error=%q (provision with `pixi install` under tesseract/v1)",
			state, stringValue(report["collision_check_error"]))
	}
}

func requireSpreadsheetPlanSafetyReport(t *testing.T, report map[string]any, trajectory []any, raw []byte) {
	t.Helper()
	requiredTopLevel := []string{
		"motion_safety_evidence_schema",
		"motion_allowed",
		"singularity_check_state",
		"singularity_analysis_available",
		"singularity_gate_clear",
		"singularity_warning_threshold",
		"singularity_hard_stop_threshold",
		"singularity_warning_point_count",
		"singularity_hard_stop_point_count",
		"minimum_singularity_score",
		"minimum_applied_velocity_scale",
		"velocity_scale_invariant",
		"singularity_points",
	}
	for _, field := range requiredTopLevel {
		if _, ok := report[field]; !ok {
			t.Fatalf("plan safety report missing top-level %s: %s", field, string(raw))
		}
	}
	if stringValue(report["motion_safety_evidence_schema"]) != "robot.v4.motion-safety-evidence.v1" {
		t.Fatalf("unexpected motion safety evidence schema %q: %s",
			stringValue(report["motion_safety_evidence_schema"]), string(raw))
	}
	if boolValue(report["motion_allowed"]) != boolValue(report["play_enabled"]) {
		t.Fatalf("motion_allowed must match the authoritative play gate: %s", string(raw))
	}
	if boolValue(report["singularity_analysis_available"]) &&
		stringValue(report["singularity_check_state"]) != "jacobian_svd_checked" {
		t.Fatalf("available singularity analysis must identify the Jacobian SVD check: %s", string(raw))
	}
	if stringValue(report["velocity_scale_invariant"]) != "scaled_velocity_i=requested_velocity_i*applied_velocity_scale" {
		t.Fatalf("unexpected common velocity-scale invariant %q: %s",
			stringValue(report["velocity_scale_invariant"]), string(raw))
	}
	warningThreshold := numericValue(report["singularity_warning_threshold"])
	hardThreshold := numericValue(report["singularity_hard_stop_threshold"])
	if hardThreshold < 0 || warningThreshold <= hardThreshold || warningThreshold > 1 {
		t.Fatalf("invalid singularity thresholds warning=%.9f hard=%.9f: %s", warningThreshold, hardThreshold, string(raw))
	}
	limits := anySlice(report["joint_speed_limits_rad_s"])
	if len(limits) != 9 {
		t.Fatalf("plan safety report has %d joint limits, want nine: %s", len(limits), string(raw))
	}
	if len(trajectory) < 2 {
		t.Fatalf("plan safety report requires a dense trajectory, got %d samples: %s", len(trajectory), string(raw))
	}
	warnings := 0
	hardStops := 0
	unavailable := 0
	jointPTPNotApplicable := 0
	minimumScore := 1.0
	minimumScale := 1.0
	for index, entry := range trajectory {
		sample, ok := entry.(map[string]any)
		if !ok {
			t.Fatalf("plan safety sample %d is not an object: %#v", index, entry)
		}
		for _, field := range []string{
			"jacobian_singular_values",
			"jacobian_condition_number",
			"jacobian_manipulability",
			"singularity_score",
			"singularity_class",
			"singularity_scale",
			"joint_limit_scale",
			"applied_velocity_scale",
			"limiting_joint",
			"singular_direction_joint",
			"requested_velocities_rad_s",
			"scaled_velocities_rad_s",
		} {
			if _, present := sample[field]; !present {
				t.Fatalf("plan safety sample %d missing %s: %#v", index, field, sample)
			}
		}
		singularValues := anySlice(sample["jacobian_singular_values"])
		requested := anySlice(sample["requested_velocities_rad_s"])
		scaled := anySlice(sample["scaled_velocities_rad_s"])
		if len(singularValues) != 6 || len(requested) != 9 || len(scaled) != 9 {
			t.Fatalf("plan safety sample %d shape singular_values=%d requested=%d scaled=%d, want 6/9/9: %#v",
				index, len(singularValues), len(requested), len(scaled), sample)
		}
		for axis := 1; axis < len(singularValues); axis++ {
			if numericValue(singularValues[axis]) > numericValue(singularValues[axis-1])+1e-6 {
				t.Fatalf("plan safety sample %d singular values are not descending: %#v", index, singularValues)
			}
		}
		score := numericValue(sample["singularity_score"])
		singularityScale := numericValue(sample["singularity_scale"])
		jointLimitScale := numericValue(sample["joint_limit_scale"])
		appliedScale := numericValue(sample["applied_velocity_scale"])
		if score < 0 || score > 1 || singularityScale < 0 || singularityScale > 1 ||
			jointLimitScale < 0 || jointLimitScale > 1 || appliedScale < 0 || appliedScale > 1 {
			t.Fatalf("plan safety sample %d has out-of-range score/scales: %#v", index, sample)
		}
		minimumScale = math.Min(minimumScale, appliedScale)
		for _, field := range []string{"limiting_joint", "singular_direction_joint"} {
			joint := stringValue(sample[field])
			if joint != "none" && (len(joint) != 2 || joint[0] != 'J' || joint[1] < '1' || joint[1] > '9') {
				t.Fatalf("plan safety sample %d has invalid %s %q: %#v", index, field, joint, sample)
			}
		}
		class := stringValue(sample["singularity_class"])
		switch class {
		case "clear":
			if score < warningThreshold-1e-6 {
				t.Fatalf("clear sample %d score %.9f is below warning %.9f", index, score, warningThreshold)
			}
		case "warning":
			warnings++
			if score <= hardThreshold || score >= warningThreshold {
				t.Fatalf("warning sample %d score %.9f is outside (%.9f, %.9f)", index, score, hardThreshold, warningThreshold)
			}
		case "hard_stop":
			hardStops++
			if score > hardThreshold+1e-6 || singularityScale != 0 {
				t.Fatalf("hard-stop sample %d did not fail to zero scale: %#v", index, sample)
			}
		case "unavailable":
			unavailable++
			if boolValue(report["singularity_analysis_available"]) {
				t.Fatalf("sample %d says singularity unavailable while report says available: %#v", index, sample)
			}
		case "not_applicable_joint_ptp":
			jointPTPNotApplicable++
			if stringValue(sample["move_type"]) != "joint_ptp" || singularityScale != 1 {
				t.Fatalf("sample %d may bypass the Cartesian singularity gate only for an explicit unscaled joint_ptp: %#v", index, sample)
			}
		default:
			t.Fatalf("plan safety sample %d has unknown singularity_class %q", index, class)
		}
		if class != "not_applicable_joint_ptp" {
			minimumScore = math.Min(minimumScore, score)
		}
		if index == 0 {
			continue
		}
		wantApplied := math.Min(singularityScale, jointLimitScale)
		if math.Abs(appliedScale-wantApplied) > 1e-6 {
			t.Fatalf("sample %d applied common scale %.9f != min(singularity %.9f, joint %.9f)",
				index, appliedScale, singularityScale, jointLimitScale)
		}
		for axis := 0; axis < 9; axis++ {
			wantScaled := numericValue(requested[axis]) * appliedScale
			gotScaled := numericValue(scaled[axis])
			if math.Abs(gotScaled-wantScaled) > 1e-5 {
				t.Fatalf("sample %d J%d scaled velocity %.9f != requested %.9f * common scale %.9f",
					index, axis+1, gotScaled, numericValue(requested[axis]), appliedScale)
			}
			if math.Abs(gotScaled) > numericValue(limits[axis])+1e-6 {
				t.Fatalf("sample %d J%d scaled velocity %.9f exceeds per-joint limit %.9f",
					index, axis+1, gotScaled, numericValue(limits[axis]))
			}
		}
	}
	if warnings != int(numericValue(report["singularity_warning_point_count"])) ||
		hardStops != int(numericValue(report["singularity_hard_stop_point_count"])) {
		t.Fatalf("singularity class counts warning=%d/%v hard=%d/%v disagree with trajectory: %s",
			warnings, report["singularity_warning_point_count"], hardStops,
			report["singularity_hard_stop_point_count"], string(raw))
	}
	if jointPTPNotApplicable > 0 && numericValue(report["reconfiguration_row_count"]) < 1 {
		t.Fatalf("trajectory contains %d joint_ptp singularity exemptions without reporting reconfiguration moves: %s",
			jointPTPNotApplicable, string(raw))
	}
	points := anySlice(report["singularity_points"])
	if len(points) != warnings+hardStops+unavailable {
		t.Fatalf("singularity point cloud has %d points, want non-clear count %d: %s",
			len(points), warnings+hardStops+unavailable, string(raw))
	}
	for index, entry := range points {
		point, ok := entry.(map[string]any)
		waypoint := int(numericValue(point["waypoint_index"]))
		if !ok || waypoint < 0 || waypoint >= len(trajectory) ||
			len(anySlice(point["tcp_pose"])) != 6 || len(anySlice(point["positions_rad"])) != 9 ||
			len(anySlice(point["jacobian_singular_values"])) != 6 {
			t.Fatalf("singularity point-cloud entry %d is not mapped to path/joints/Jacobian: %#v", index, entry)
		}
		sample, _ := trajectory[waypoint].(map[string]any)
		if stringValue(point["singularity_class"]) != stringValue(sample["singularity_class"]) ||
			math.Abs(numericValue(point["singularity_score"])-numericValue(sample["singularity_score"])) > 1e-6 {
			t.Fatalf("singularity point-cloud entry %d disagrees with trajectory waypoint %d: point=%#v sample=%#v",
				index, waypoint, point, sample)
		}
	}
	if math.Abs(minimumScore-numericValue(report["minimum_singularity_score"])) > 1e-6 ||
		math.Abs(minimumScale-numericValue(report["minimum_applied_velocity_scale"])) > 1e-6 {
		t.Fatalf("top-level minimum score/scale do not match trajectory score=%.9f scale=%.9f: %s",
			minimumScore, minimumScale, string(raw))
	}
}

func runSpreadsheetPlanSelfTest(t *testing.T, repoRoot, inputName string) (map[string]any, []byte) {
	t.Helper()
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), inputName+"-plan.json")
	inputPath := filepath.Join(repoRoot, "motion-server", "v1", "testdata", inputName)
	cmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server spreadsheet plan self-test (%s) failed: %v\n%s", inputName, err, string(output))
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read spreadsheet plan report (%s): %v", inputName, err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode spreadsheet plan report (%s): %v\n%s", inputName, err, string(raw))
	}
	return report, raw
}

func runSpreadsheetPlanSelfTestFromPath(t *testing.T, repoRoot, inputPath string) (map[string]any, []byte) {
	t.Helper()
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	outputPath := filepath.Join(t.TempDir(), "authoritative-edit-plan.json")
	cmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server authoritative spreadsheet plan self-test failed: %v\n%s", err, string(output))
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read authoritative spreadsheet plan report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode authoritative spreadsheet plan report: %v\n%s", err, string(raw))
	}
	return report, raw
}

func TestRobotV4MotionServerUsesExactHydratedCalibrationEdits(t *testing.T) {
	repoRoot := findRepoRoot(t)
	builtin, builtinRaw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-program-ref-input.json")
	rows := anySlice(builtin["program_rows"])
	const canonicalMotionRows = 15
	const canonicalProgramRows = canonicalMotionRows + 1
	if len(rows) != canonicalProgramRows ||
		int(numericValue(builtin["motion_row_count"])) != canonicalMotionRows ||
		int(numericValue(builtin["command_row_count"])) != 1 {
		t.Fatalf("builtin program returned an invalid editable Start..LOOP structure: rows=%d motion=%d command=%d want=%d/%d/1: %s",
			len(rows), int(numericValue(builtin["motion_row_count"])),
			int(numericValue(builtin["command_row_count"])),
			canonicalProgramRows, canonicalMotionRows, string(builtinRaw))
	}
	editedRowIndex := -1
	editedX := 0.0
	for index, entry := range rows {
		row, ok := entry.(map[string]any)
		if !ok {
			t.Fatalf("builtin row %d is not an object: %#v", index, entry)
		}
		row["revision"] = 1
		row["row_hash"] = "accepted-row-" + strconv.Itoa(index+1) + "-revision-1"
		if editedRowIndex < 0 && stringValue(row["motion_phase"]) == "calibration" {
			target := anySlice(row["target_tcp_m"])
			if len(target) != 6 {
				t.Fatalf("editable calibration row lacks six-axis target: %#v", row)
			}
			editedRowIndex = index
			editedX = numericValue(target[0]) + 0.001
			target[0] = editedX
			row["target_tcp_m"] = target
			row["revision"] = 2
			row["row_hash"] = "accepted-edited-row-revision-2"
		}
	}
	if editedRowIndex < 0 {
		t.Fatalf("builtin canonical program had no calibration row: %s", string(builtinRaw))
	}
	request := map[string]any{
		"schema_version":         1,
		"program_id":             "robot-v4-tour-calibration-box",
		"program_revision":       2,
		"base_program_revision":  1,
		"program_reference_only": false,
		"rows_omitted_by_client": false,
		"execution_mode":         "dry_run",
		"samples_per_segment":    2,
		"rows":                   rows,
	}
	requestRaw, err := json.Marshal(request)
	if err != nil {
		t.Fatalf("encode authoritative edit request: %v", err)
	}
	inputPath := filepath.Join(t.TempDir(), "authoritative-calibration-edit.json")
	if err := os.WriteFile(inputPath, requestRaw, 0o600); err != nil {
		t.Fatalf("write authoritative edit request: %v", err)
	}
	report, raw := runSpreadsheetPlanSelfTestFromPath(t, repoRoot, inputPath)
	if stringValue(report["program_source"]) !=
		"motion-server-authoritative-client-edited-program-rows" ||
		int(numericValue(report["program_revision"])) != 2 ||
		int(numericValue(report["accepted_program_revision"])) != 2 ||
		stringValue(report["program_revision_semantics"]) != "optimistic-client-base" ||
		stringValue(report["harmonious_trajectory_source"]) != "row_targets" ||
		stringValue(report["program_hash"]) == stringValue(builtin["program_hash"]) {
		t.Fatalf("edited canonical rows did not become the authoritative accepted program: %s", string(raw))
	}
	acceptedRows := anySlice(report["program_rows"])
	if len(acceptedRows) != len(rows) {
		t.Fatalf("accepted report returned %d rows, want exact %d-row edit: %s", len(acceptedRows), len(rows), string(raw))
	}
	acceptedEdit, ok := acceptedRows[editedRowIndex].(map[string]any)
	acceptedTarget := anySlice(acceptedEdit["target_tcp_m"])
	if !ok || len(acceptedTarget) != 6 ||
		math.Abs(numericValue(acceptedTarget[0])-editedX) > 1e-6 ||
		int(numericValue(acceptedEdit["revision"])) != 2 ||
		stringValue(acceptedEdit["row_hash"]) != "accepted-edited-row-revision-2" {
		t.Fatalf("motion-server did not retain the exact edited row: %#v", acceptedEdit)
	}
	terminal, ok := acceptedRows[len(acceptedRows)-1].(map[string]any)
	if !ok || stringValue(terminal["row_type"]) != "command" ||
		stringValue(terminal["command"]) != "LOOP" ||
		stringValue(terminal["motion_phase"]) != "loop" ||
		int(numericValue(terminal["calibration_section"])) != 0 ||
		int(numericValue(terminal["loop_target_row"])) != 0 ||
		!boolValue(terminal["loop_return_home"]) {
		t.Fatalf("accepted authoritative edit lost its protected terminal LOOP: %#v", terminal)
	}

	request["base_program_revision"] = 3
	staleRaw, err := json.Marshal(request)
	if err != nil {
		t.Fatalf("encode stale-base edit request: %v", err)
	}
	stalePath := filepath.Join(t.TempDir(), "stale-base-calibration-edit.json")
	if err := os.WriteFile(stalePath, staleRaw, 0o600); err != nil {
		t.Fatalf("write stale-base edit request: %v", err)
	}
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	cmd := exec.Command(binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", stalePath,
		"--output", filepath.Join(t.TempDir(), "stale-base-output.json"),
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	if output, err := cmd.CombinedOutput(); err == nil {
		t.Fatalf("motion-server accepted program_revision=2 over newer base=3: %s", string(output))
	}
}

func TestRobotV4MotionServerCanonicalPlanImportsIntoUnrealWithTableAOwnership(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("Robot v4 C++ motion-server integration runs on macOS/Linux")
	}
	repoRoot := findRepoRoot(t)
	_, raw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-program-ref-input.json")
	tempDir := t.TempDir()
	planPath := filepath.Join(tempDir, "robot-v4-canonical-plan.json")
	outputPath := filepath.Join(tempDir, "unreal-table-a-plan.json")
	if err := os.WriteFile(planPath, raw, 0o600); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command(
		"go", "run", ".", "unreal", "v1", "game", "import-robot-v4-plan",
		"--input", planPath, "--output", outputPath, "--json",
	)
	cmd.Dir = repoRoot
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("Unreal rejected the real C++ calibration plan: %v\n%s", err, output)
	}
	importedRaw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatal(err)
	}
	var imported map[string]any
	if err := json.Unmarshal(importedRaw, &imported); err != nil {
		t.Fatalf("decode Unreal Robot-v4 import: %v", err)
	}
	trajectory := anySlice(imported["trajectory"])
	if len(trajectory) < 200 {
		t.Fatalf("Unreal import returned only %d trajectory samples", len(trajectory))
	}
	sections := map[int]bool{}
	for index, value := range trajectory {
		sample, ok := value.(map[string]any)
		if !ok {
			t.Fatalf("Unreal sample %d is not an object", index)
		}
		section := int(numericValue(sample["calibration_section"]))
		if section == 0 {
			continue
		}
		sections[section] = true
		if stringValue(sample["path_parent"]) != "table_a" ||
			stringValue(sample["work_frame"]) != "positioner_table_a" ||
			stringValue(sample["fixture_side"]) != "A" ||
			len(anySlice(sample["target_local_xyz_m"])) != 3 {
			t.Fatalf("Unreal sample %d lost Table-A-local ownership: %#v", index, sample)
		}
	}
	if !sections[1] || !sections[2] || len(sections) != 2 {
		t.Fatalf("Unreal import calibration sections=%v, want exactly 1 and 2", sections)
	}
}

func TestRobotV4MotionServerProgramHashCanonicalizesSemanticPrecision(t *testing.T) {
	repoRoot := findRepoRoot(t)
	raw, err := os.ReadFile(filepath.Join(
		repoRoot,
		"motion-server", "v1",
		"src",
		"robot_v4_cartesian_plan_services.hpp",
	))
	if err != nil {
		t.Fatal(err)
	}
	source := string(raw)
	if !strings.Contains(source, "kSpreadsheetProgramHashUnitsPerWhole = 1000000.0") ||
		!strings.Contains(source, "std::llround(value * kSpreadsheetProgramHashUnitsPerWhole)") ||
		strings.Contains(source, "std::memcpy(&bits, &value, sizeof(bits))") {
		t.Fatal("Motion Server program hash does not use six-decimal semantic values")
	}
	report, reportRaw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-program-ref-input.json")
	got := stringValue(report["program_hash"])
	const canonicalHash = "fnv1a64:bda6c0aa57b6df88"
	if got != canonicalHash {
		t.Fatalf(
			"Motion Server canonical program hash = %q, want %q: %s",
			got,
			canonicalHash,
			string(reportRaw),
		)
	}
}

func TestRobotV4MotionServerResolvesCalibrationTourBoxFromProgramDatabase(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(t, repoRoot, "spreadsheet-plan-program-ref-input.json")
	if stringValue(report["program_id"]) != "robot-v4-tour-calibration-box" ||
		stringValue(report["program_source"]) != "motion-server-builtin-program-database" ||
		int(numericValue(report["program_revision"])) != 4 ||
		int(numericValue(report["source_row_count"])) != 15 {
		t.Fatalf("canonical program reference did not resolve from the motion-server database: %s", string(raw))
	}
	if !boolValue(report["tool_axis_constraint_present"]) ||
		!boolValue(report["tool_axis_constraint_valid"]) ||
		stringValue(report["tool_frame"]) != "end_effector" ||
		stringValue(report["tip_link"]) != "tool0" ||
		stringValue(report["world_frame"]) != "world" {
		t.Fatalf("canonical program did not apply its motion-server-owned tool constraint: %s", string(raw))
	}
	if !boolValue(report["harmonious_external_axis_present"]) ||
		!boolValue(report["harmonious_external_axis_valid"]) ||
		!boolValue(report["harmonious_external_axis_applied"]) ||
		stringValue(report["harmonious_external_axis_joint"]) != "J7" ||
		stringValue(report["harmonious_trajectory_source"]) != "row_targets" ||
		math.Abs(numericValue(report["harmonious_external_axis_min_rad"])) > 1e-9 ||
		math.Abs(numericValue(report["harmonious_external_axis_max_rad"])-math.Pi/4) > 1e-6 {
		t.Fatalf("canonical program did not coordinate the motion-server-owned J7 trajectory: %s", string(raw))
	}
	if !boolValue(report["harmonious_workplane_relative"]) ||
		stringValue(report["harmonious_work_frame"]) != "positioner_table_a" ||
		boolValue(report["harmonious_tool_axis_follows_workplane"]) ||
		numericValue(report["max_workplane_relative_error_m"]) > 1e-9 {
		t.Fatalf("canonical program did not keep its TCP path rigid to the rotating Table A workplane: %s", string(raw))
	}
	rows := anySlice(report["program_rows"])
	wantLabels := []string{
		"Start", "RetractBeforeL1", "TableCorner1", "TableCorner2",
		"TableCorner3", "L1SettleJ7Zero", "RetractAfterL1",
		"ReturnHomeAfterL1", "RetractBeforeL2",
		"SectionBStartC3", "TableCorner4", "SectionBEndC1",
		"L2SettleJ7Zero", "RetractAfterL2", "ReturnStart", "LOOP",
	}
	wantJ7ByLabel := map[string]float64{
		"TableCorner1":    0,
		"TableCorner2":    math.Pi / 4,
		"TableCorner3":    0,
		"SectionBStartC3": 0,
		"TableCorner4":    math.Pi / 4,
		"SectionBEndC1":   0,
	}
	for index, entry := range rows {
		row, ok := entry.(map[string]any)
		if !ok || index >= len(wantLabels) || stringValue(row["label"]) != wantLabels[index] ||
			stringValue(row["work_frame"]) != "positioner_table_a" {
			t.Fatalf("canonical program row %d is not authored in the Table A workplane: %#v", index, entry)
		}
		if index == len(rows)-1 {
			if stringValue(row["row_type"]) != "command" ||
				stringValue(row["command"]) != "LOOP" ||
				int(numericValue(row["loop_target_row"])) != 0 ||
				int(numericValue(row["loop_max_iterations"])) != 1 ||
				!boolValue(row["loop_return_home"]) {
				t.Fatalf("canonical terminal row is not the bounded return-home LOOP command: %#v", row)
			}
			continue
		}
		joints := anySlice(row["target_joints_rad"])
		if len(joints) != 9 {
			t.Fatalf("canonical program row %d does not expose all nine joint targets: %#v", index, entry)
		}
		if wantJ7, pinned := wantJ7ByLabel[stringValue(row["label"])]; pinned &&
			math.Abs(numericValue(joints[6])-wantJ7) > 1e-6 {
			t.Fatalf("canonical row %q J7=%v, want row target %v: %#v",
				stringValue(row["label"]), numericValue(joints[6]), wantJ7, row)
		}
		moveType := stringValue(row["move_type"])
		if moveType != "joint_ptp" && moveType != "line_tcp" {
			t.Fatalf("canonical motion row %d has unsupported move_type %q: %#v", index, moveType, entry)
		}
	}
	limits := anySlice(report["joint_speed_limits_rad_s"])
	maxSpeeds := anySlice(report["max_joint_speeds_rad_s"])
	if !boolValue(report["joint_speed_limits_applied"]) || len(limits) != 9 || len(maxSpeeds) != 9 {
		t.Fatalf("canonical program did not report nine-axis speed-limit retiming: %s", string(raw))
	}
	for axis := range limits {
		if numericValue(maxSpeeds[axis]) > numericValue(limits[axis])+1e-6 {
			t.Fatalf("canonical program axis J%d speed %.6f exceeds limit %.6f: %s",
				axis+1, numericValue(maxSpeeds[axis]), numericValue(limits[axis]), string(raw))
		}
	}
	trajectory := anySlice(report["trajectory"])
	if len(trajectory) < 13 {
		t.Fatalf("canonical program returned %d samples, want the dense segmented plan: %s", len(trajectory), string(raw))
	}
	if !boolValue(report["joint_angle_continuity_applied"]) ||
		numericValue(report["max_joint_step_rad"]) >= 0.25 ||
		numericValue(report["max_joint_ptp_step_rad"]) > 0.051 ||
		boolValue(report["hidden_start_transition"]) {
		t.Fatalf("canonical program retained a false +/-pi joint wrap discontinuity: %s", string(raw))
	}
	firstSample, firstSampleOK := trajectory[0].(map[string]any)
	lastSample, lastSampleOK := trajectory[len(trajectory)-1].(map[string]any)
	if !firstSampleOK || !lastSampleOK ||
		stringValue(firstSample["motion_phase"]) != "home" ||
		stringValue(lastSample["motion_phase"]) != "return_home" {
		t.Fatalf("canonical trajectory does not expose exact home boundaries: first=%#v last=%#v", firstSample, lastSample)
	}
	for _, sample := range []map[string]any{firstSample, lastSample} {
		positions := anySlice(sample["positions_rad"])
		if len(positions) != 9 {
			t.Fatalf("home boundary missing nine axes: %#v", sample)
		}
		for axis, position := range positions {
			if math.Abs(numericValue(position)) > 1e-9 {
				t.Fatalf("home boundary J%d = %.9f, want exact zero: %#v", axis+1, numericValue(position), sample)
			}
		}
	}
	middleHomeSeen := false
	for index := 1; index < len(trajectory); index++ {
		previous, previousOK := trajectory[index-1].(map[string]any)
		current, currentOK := trajectory[index].(map[string]any)
		previousPositions := anySlice(previous["positions_rad"])
		currentPositions := anySlice(current["positions_rad"])
		if !previousOK || !currentOK || len(previousPositions) != 9 || len(currentPositions) != 9 {
			t.Fatalf("canonical trajectory sample %d missing nine-axis positions: %s", index, string(raw))
		}
		for axis := range previousPositions {
			if math.Abs(numericValue(currentPositions[axis])-numericValue(previousPositions[axis])) >= 0.25 {
				t.Fatalf("canonical trajectory sample %d J%d retained >=0.25 rad discontinuity: %s", index, axis+1, string(raw))
			}
		}
		if stringValue(current["motion_phase"]) == "section_home" {
			exactHome := true
			for _, position := range currentPositions {
				exactHome = exactHome && math.Abs(numericValue(position)) <= 1e-9
			}
			middleHomeSeen = middleHomeSeen || exactHome
		}
		if stringValue(current["move_type"]) == "joint_ptp" &&
			math.Abs(numericValue(currentPositions[6])) > 0.001 {
			t.Fatalf("canonical reconfiguration sample %d moves J7 outside its explicit zero settle: %s", index, string(raw))
		}
	}
	if !middleHomeSeen {
		t.Fatalf("canonical two-L trajectory never reached the exact shared Home boundary: %s", string(raw))
	}
	worldCurveSeen := false
	firstEdgeWorldA := [3]float64{1.0256, -0.622, 0.2700625}
	var firstEdgeWorldB, firstEdgeWorldMid [3]float64
	bestFirstEdgeB := math.Inf(1)
	bestFirstEdgeMid := math.Inf(1)
	for index, entry := range trajectory {
		sample, ok := entry.(map[string]any)
		if !ok || !boolValue(sample["harmonious_workplane_active"]) {
			continue
		}
		positions := anySlice(sample["positions_rad"])
		tcp := anySlice(sample["tcp_pose"])
		if len(positions) != 9 || len(tcp) != 6 || stringValue(sample["path_parent"]) != "table_a" {
			t.Fatalf("table-owned calibration sample %d is incomplete: %#v", index, sample)
		}
		q := numericValue(positions[6])
		dx := numericValue(tcp[0]) - 1.1256
		dy := numericValue(tcp[1]) - 0.62200026
		dz := numericValue(tcp[2]) - 0.0700413
		c, s := math.Cos(q), math.Sin(q)
		refX := 1.1256 + c*dx - s*dz
		refY := 0.62200026 + dy
		refZ := 0.0700413 + s*dx + c*dz
		const epsilon = 2e-5
		if math.Abs(refZ-0.2700625) > epsilon {
			t.Fatalf("calibration sample %d left the +100mm Table-A workplane: reference=(%.9f,%.9f,%.9f) sample=%#v",
				index, refX, refY, refZ, sample)
		}
		section := int(numericValue(sample["calibration_section"]))
		onAB := math.Abs(refY+0.622) <= epsilon && refX >= 1.0256-epsilon && refX <= 1.2256+epsilon
		onBC := math.Abs(refX-1.2256) <= epsilon && refY >= -0.622-epsilon && refY <= 0.622+epsilon
		onCD := math.Abs(refY-0.622) <= epsilon && refX >= 1.0256-epsilon && refX <= 1.2256+epsilon
		onDA := math.Abs(refX-1.0256) <= epsilon && refY >= -0.622-epsilon && refY <= 0.622+epsilon
		if (section == 1 && !onAB && !onBC) || (section == 2 && !onCD && !onDA) ||
			(section != 1 && section != 2) {
			t.Fatalf("calibration sample %d is not on its authored L section: section=%d reference=(%.9f,%.9f,%.9f)",
				index, section, refX, refY, refZ)
		}
		worldDisplacement := math.Sqrt(
			math.Pow(numericValue(tcp[0])-refX, 2) +
				math.Pow(numericValue(tcp[1])-refY, 2) +
				math.Pow(numericValue(tcp[2])-refZ, 2))
		if q > 0.7 && worldDisplacement > 0.05 {
			worldCurveSeen = true
		}
		if section == 1 && onAB {
			world := [3]float64{
				numericValue(tcp[0]),
				numericValue(tcp[1]),
				numericValue(tcp[2]),
			}
			if distance := math.Abs(refX - 1.2256); distance < bestFirstEdgeB {
				bestFirstEdgeB = distance
				firstEdgeWorldB = world
			}
			if distance := math.Abs(refX - 1.1256); distance < bestFirstEdgeMid {
				bestFirstEdgeMid = distance
				firstEdgeWorldMid = world
			}
		}
	}
	if !worldCurveSeen {
		t.Fatalf("canonical two-L plan never produced a +45deg table-owned curved world sample: %s", string(raw))
	}
	if bestFirstEdgeB > 2e-5 || bestFirstEdgeMid > 0.002 {
		t.Fatalf("canonical first edge lacks endpoint/midpoint samples: B=%.9f midpoint=%.9f",
			bestFirstEdgeB, bestFirstEdgeMid)
	}
	firstEdgeChordMid := [3]float64{
		(firstEdgeWorldA[0] + firstEdgeWorldB[0]) / 2,
		(firstEdgeWorldA[1] + firstEdgeWorldB[1]) / 2,
		(firstEdgeWorldA[2] + firstEdgeWorldB[2]) / 2,
	}
	firstEdgeChordDeviation := math.Sqrt(
		math.Pow(firstEdgeWorldMid[0]-firstEdgeChordMid[0], 2) +
			math.Pow(firstEdgeWorldMid[1]-firstEdgeChordMid[1], 2) +
			math.Pow(firstEdgeWorldMid[2]-firstEdgeChordMid[2], 2))
	if firstEdgeChordDeviation <= 0.05 {
		t.Fatalf("canonical first edge collapsed to a world-space endpoint chord: midpoint deviation=%.9f m",
			firstEdgeChordDeviation)
	}
	if len(rows) != 16 || int(numericValue(report["motion_row_count"])) != 15 ||
		int(numericValue(report["command_row_count"])) != 1 ||
		int(numericValue(report["calibration_section_count"])) != 2 ||
		int(numericValue(report["reconfiguration_row_count"])) != 4 ||
		int(numericValue(report["home_row_count"])) != 3 ||
		!boolValue(report["starts_at_home"]) ||
		!boolValue(report["returns_home"]) ||
		!boolValue(report["program_structure_valid"]) ||
		!boolValue(report["loop_command_present"]) ||
		!boolValue(report["loop_command_valid"]) ||
		int(numericValue(report["loop_command_row_index"])) != 15 ||
		int(numericValue(report["loop_target_row"])) != 0 ||
		int(numericValue(report["loop_max_iterations"])) != 1 ||
		!boolValue(report["loop_return_home"]) {
		t.Fatalf("canonical program did not expose fifteen independently Home-bounded motion rows plus one bounded terminal LOOP command: %s", string(raw))
	}
	first, firstOK := rows[0].(map[string]any)
	returnStart, returnStartOK := rows[len(rows)-2].(map[string]any)
	loop, loopOK := rows[len(rows)-1].(map[string]any)
	if !firstOK || !returnStartOK || !loopOK ||
		stringValue(first["label"]) != "Start" ||
		stringValue(returnStart["label"]) != "ReturnStart" ||
		stringValue(loop["label"]) != "LOOP" ||
		stringValue(first["move_type"]) != "joint_ptp" ||
		stringValue(returnStart["move_type"]) != "joint_ptp" ||
		stringValue(first["revpi_io_action"]) != "none" ||
		stringValue(returnStart["revpi_io_action"]) != "none" ||
		int(numericValue(first["revpi_io_slot"])) != 5 ||
		len(anySlice(first["target_tcp_m"])) != 6 ||
		len(anySlice(first["target_joints_rad"])) != 9 {
		t.Fatalf("canonical program rows do not expose the thin-client render contract: first=%#v return_start=%#v loop=%#v", first, returnStart, loop)
	}
	wantTraceRows := map[string]int{
		"TableCorner2":  1,
		"TableCorner3":  1,
		"TableCorner4":  2,
		"SectionBEndC1": 2,
	}
	for _, entry := range rows {
		row, _ := entry.(map[string]any)
		if wantSection, ok := wantTraceRows[stringValue(row["label"])]; ok {
			if stringValue(row["move_type"]) != "line_tcp" ||
				int(numericValue(row["calibration_section"])) != wantSection {
				t.Fatalf("segmented trace row has the wrong section contract: %#v", row)
			}
			delete(wantTraceRows, stringValue(row["label"]))
		}
	}
	if len(wantTraceRows) != 0 {
		t.Fatalf("canonical program omitted segmented box trace rows: %#v", wantTraceRows)
	}
	state := stringValue(report["collision_check_state"])
	if state != "tesseract_discrete_contact_checked" {
		t.Skipf("tesseract collision worker unavailable (%q): run `pixi install` under tesseract/v1 to enable the genuine contact check", state)
	}
	if stringValue(report["validation_state"]) != "valid" ||
		numericValue(report["collision_point_count"]) != 0 ||
		!boolValue(report["collision_gate_clear"]) ||
		!boolValue(report["motion_allowed"]) ||
		!boolValue(report["play_enabled"]) ||
		!boolValue(report["runtime_preload_enabled"]) ||
		!boolValue(report["isolated_simulation_preload_enabled"]) ||
		boolValue(report["simulation_only_program"]) {
		t.Fatalf("canonical calibration-box loop must be collision-free and playable: %s", string(raw))
	}
}

func TestRobotV4MotionServerSpreadsheetPlanHonestyClear(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(t, repoRoot, "spreadsheet-plan-clear-input.json")
	requireHonestSpreadsheetPlanValidation(t, report, raw)
	if stringValue(report["constraint_owner"]) != "motion-server" ||
		stringValue(report["constraint_schema"]) != "robot.v4.tool-axis-constraint.v1" ||
		!boolValue(report["tool_axis_constraint_present"]) ||
		!boolValue(report["tool_axis_constraint_valid"]) ||
		!boolValue(report["tool_axis_constraint_checked"]) ||
		!boolValue(report["tool_axis_constraint_satisfied"]) ||
		numericValue(report["constraint_offset_deg"]) != 45.0 ||
		numericValue(report["constraint_tolerance_minus_deg"]) != 20.0 ||
		numericValue(report["constraint_tolerance_plus_deg"]) != 20.0 ||
		math.Abs(numericValue(report["min_measured_tool_axis_offset_deg"])-45.0) > 1.0 ||
		math.Abs(numericValue(report["max_measured_tool_axis_offset_deg"])-45.0) > 1.0 ||
		numericValue(report["max_tool_axis_error_deg"]) > 1.0 {
		t.Fatalf("motion-server did not authoritatively satisfy end_effector +X -> world -Z: report=%s", string(raw))
	}
	if stringValue(report["tool_frame"]) != "end_effector" ||
		stringValue(report["tip_link"]) != "tool0" ||
		stringValue(report["world_frame"]) != "world" ||
		boolValue(report["harmonious_external_axis_present"]) ||
		boolValue(report["harmonious_external_axis_applied"]) {
		t.Fatalf("collision-clear fixture must stay in-cell and must not imply an unrequested positioner workplane: report=%s", string(raw))
	}
	for index, entry := range anySlice(report["trajectory"]) {
		sample, ok := entry.(map[string]any)
		if !ok || !boolValue(sample["tool_axis_constraint_satisfied"]) ||
			math.Abs(numericValue(sample["measured_tool_axis_offset_deg"])-45.0) > 1.0 ||
			numericValue(sample["tool_axis_error_deg"]) > 1.0 {
			t.Fatalf("trajectory sample %d violates the authoritative tool-axis constraint: %#v", index, entry)
		}
		positions := anySlice(sample["positions_rad"])
		if len(positions) != 9 {
			t.Fatalf("trajectory sample %d has %d axes, want all 9 mapped drives: %#v", index, len(positions), entry)
		}
	}
	state := stringValue(report["collision_check_state"])
	if state != "tesseract_discrete_contact_checked" {
		t.Skipf("tesseract collision worker unavailable (%q): run `pixi install` under tesseract/v1 to enable the genuine contact check", state)
	}
	if stringValue(report["validation_state"]) != "valid" ||
		numericValue(report["unreachable_point_count"]) != 0 ||
		numericValue(report["collision_point_count"]) != 0 ||
		stringValue(report["limit_check_state"]) != "clear" ||
		!boolValue(report["collision_gate_clear"]) ||
		!boolValue(report["play_enabled"]) ||
		!boolValue(report["runtime_preload_enabled"]) ||
		!boolValue(report["isolated_simulation_preload_enabled"]) {
		t.Fatalf("raised rectangle must prove fully valid; report=%s", string(raw))
	}
}

func TestRobotV4MotionServerSpreadsheetPlanProportionalSingularityScale(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-proportional-singularity-input.json")
	trajectory := anySlice(report["trajectory"])
	requireSpreadsheetPlanSafetyReport(t, report, trajectory, raw)
	if !boolValue(report["singularity_analysis_available"]) ||
		!boolValue(report["singularity_gate_clear"]) ||
		numericValue(report["singularity_warning_point_count"]) == 0 ||
		numericValue(report["singularity_hard_stop_point_count"]) != 0 {
		t.Fatalf("warning-only singularity fixture did not stay clear of the hard-stop gate with proportional slowdown: %s", raw)
	}
	sawFractionalCommonScale := false
	for index, entry := range trajectory {
		if index == 0 {
			continue
		}
		sample, _ := entry.(map[string]any)
		scale := numericValue(sample["applied_velocity_scale"])
		if scale > 0 && scale < 1 {
			sawFractionalCommonScale = true
			break
		}
	}
	if !sawFractionalCommonScale {
		t.Fatalf("warning-only singularity fixture never applied a fractional common velocity scale: %s", raw)
	}
}

func TestRobotV4MotionServerResolvesRosie1420SixAxisNearHomeCalibrationBox(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	inputPath := filepath.Join(
		repoRoot, "motion-server", "v1", "testdata",
		"spreadsheet-plan-program-ref-rosie-1420-input.json",
	)
	outputPath := filepath.Join(t.TempDir(), "rosie-1420-near-home-plan.json")
	urdfPath := filepath.Join(
		repoRoot, "robot_description", "robots", "rosie_1420_v1", "robot.urdf",
	)
	collisionPython := os.Getenv("ROBOT_V4_LOCAL_SIM_TESSERACT_PYTHON")
	if collisionPython == "" {
		if home, homeErr := os.UserHomeDir(); homeErr == nil {
			candidate := filepath.Join(
				home, ".rosie", "robot-v4-tesseract-v1", "bin", "python3",
			)
			if _, statErr := os.Stat(candidate); statErr == nil {
				collisionPython = candidate
			}
		}
	}
	if collisionPython == "" {
		candidate := filepath.Join(
			repoRoot, "tesseract", "v1", ".pixi", "envs", "default", "bin", "python3",
		)
		if _, statErr := os.Stat(candidate); statErr == nil {
			collisionPython = candidate
		}
	}
	if collisionPython == "" {
		t.Skip("Rosie 1420 collision qualification requires the provisioned Tesseract Python worker")
	}
	collisionWorker := collisionPython + " " + filepath.Join(
		repoRoot, "tesseract", "v1", "python", "collision_check.py",
	)
	cmd := exec.Command(
		binary,
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	cmd.Env = append(
		os.Environ(),
		"ROBOT_V4_TESSERACT_URDF_PATH="+urdfPath,
		"ROBOT_V4_TESSERACT_COLLISION_WORKER="+collisionWorker,
	)
	output, runErr := cmd.CombinedOutput()
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read Rosie 1420 near-home plan after %v: %v\n%s", runErr, err, output)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode Rosie 1420 near-home plan: %v\n%s", err, raw)
	}
	if runErr != nil {
		t.Fatalf(
			"motion-server Rosie 1420 near-home plan failed: %v\n%s\nvalidation=%s structure=%t collision=%s contacts=%d unreachable=%d moving_axes=%d max_home_delta=%.6f",
			runErr, output,
			stringValue(report["validation_state"]),
			boolValue(report["program_structure_valid"]),
			stringValue(report["collision_check_state"]),
			int(numericValue(report["collision_point_count"])),
			int(numericValue(report["unreachable_point_count"])),
			int(numericValue(report["moving_arm_axis_count"])),
			numericValue(report["max_arm_home_delta_rad"]),
		)
	}
	for key, want := range map[string]string{
		"program_id":                       "robot-v4-tour-calibration-box-rosie-1420-v1",
		"robot_model":                      "rosie_1420_v1",
		"robotic-cell-type":                "rosie_1420_v1",
		"planner_owner":                    "motion-server",
		"planner_backend":                  "motion_server_tesseract_trajectory_v1",
		"trajectory_method":                "linear-tcp-dense-csp",
		"playback_solver":                  "tesseract_dense_trajectory",
		"collision_check_state":            "tesseract_discrete_contact_checked",
		"tesseract_collision_geometry":     "visual_meshes_as_collision_meshes",
		"tesseract_collision_mesh_package": "package://rosie_1420_v1/meshes",
	} {
		if got := stringValue(report[key]); got != want {
			t.Fatalf("Rosie 1420 report %s=%q, want %q: %s", key, got, want, raw)
		}
	}
	if !boolValue(report["program_structure_valid"]) ||
		!boolValue(report["motion_allowed"]) ||
		!boolValue(report["play_enabled"]) ||
		!boolValue(report["collision_gate_clear"]) ||
		!boolValue(report["all_sample_fk_within_tolerance"]) ||
		!boolValue(report["smooth_dense_trajectory"]) ||
		!boolValue(report["starts_at_home"]) ||
		!boolValue(report["returns_home"]) ||
		!boolValue(report["external_axes_stationary"]) ||
		!boolValue(report["near_home_joint_bound_satisfied"]) {
		t.Fatalf(
			"Rosie 1420 near-home safety contract failed: structure=%t motion=%t play=%t home=%t/%t external=%t near_home=%t validation=%s collision=%s unreachable=%d moving_axes=%d max_home_delta=%.6f",
			boolValue(report["program_structure_valid"]),
			boolValue(report["motion_allowed"]),
			boolValue(report["play_enabled"]),
			boolValue(report["starts_at_home"]),
			boolValue(report["returns_home"]),
			boolValue(report["external_axes_stationary"]),
			boolValue(report["near_home_joint_bound_satisfied"]),
			stringValue(report["validation_state"]),
			stringValue(report["collision_check_state"]),
			int(numericValue(report["unreachable_point_count"])),
			int(numericValue(report["moving_arm_axis_count"])),
			numericValue(report["max_arm_home_delta_rad"]),
		)
	}
	tesseractURDFSource := stringValue(report["tesseract_urdf_source"])
	if filepath.Clean(tesseractURDFSource) != filepath.Clean(urdfPath) {
		t.Fatalf(
			"Rosie 1420 Tesseract plan used URDF %q, want %q: %s",
			tesseractURDFSource, urdfPath, raw)
	}
	trajectory := anySlice(report["trajectory"])
	if checked := int(numericValue(
		report["collision_checked_waypoint_count"])); checked != len(trajectory) {
		t.Fatalf(
			"Tesseract checked %d Rosie 1420 waypoints, want all %d: %s",
			checked, len(trajectory), raw)
	}
	if stringValue(report["collision_checker"]) == "" ||
		stringValue(report["collision_checker_library_version"]) == "" ||
		numericValue(report["max_fk_translation_residual_m"]) > 0.005 ||
		numericValue(report["max_fk_orientation_residual_rad"]) > 0.05 {
		t.Fatalf(
			"Rosie 1420 Tesseract/FK proof is incomplete: %s", raw)
	}
	if int(numericValue(report["program_row_count"])) != 8 ||
		int(numericValue(report["motion_row_count"])) != 7 ||
		int(numericValue(report["calibration_section_count"])) != 1 ||
		int(numericValue(report["mapped_axis_mask"])) != 0x003f ||
		int(numericValue(report["moving_arm_axis_count"])) != 6 ||
		numericValue(report["max_external_axis_abs_rad"]) > 1e-9 ||
		numericValue(report["max_arm_home_delta_rad"]) > 0.25 ||
		!boolValue(report["loop_command_present"]) ||
		!boolValue(report["loop_command_valid"]) ||
		int(numericValue(report["loop_target_row"])) != 0 ||
		int(numericValue(report["loop_max_iterations"])) != 3 ||
		!boolValue(report["loop_return_home"]) {
		t.Fatalf("Rosie 1420 six-axis bounds are incomplete: %s", raw)
	}
	rows := anySlice(report["program_rows"])
	labels := map[string]bool{}
	for _, rowAny := range rows {
		row, _ := rowAny.(map[string]any)
		labels[stringValue(row["label"])] = true
	}
	for _, label := range []string{
		"NearHomeCorner1",
		"NearHomeCorner2",
		"NearHomeCorner3",
		"NearHomeCorner4",
		"ReturnStart",
		"LOOP",
	} {
		if !labels[label] {
			t.Fatalf("Rosie 1420 program is missing %s: %s", label, raw)
		}
	}
	for index, sampleAny := range trajectory {
		sample, _ := sampleAny.(map[string]any)
		positions := anySlice(sample["positions_rad"])
		if len(positions) != 9 {
			t.Fatalf("trajectory sample %d has %d positions, want shared nine-axis envelope", index, len(positions))
		}
		for axis := 6; axis < 9; axis++ {
			if math.Abs(numericValue(positions[axis])) > 1e-9 {
				t.Fatalf("trajectory sample %d moved external J%d: %#v", index, axis+1, positions)
			}
		}
	}
}

func TestRobotV4MotionServerSpreadsheetPlanHardSingularityFailsClosed(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-hard-singularity-input.json")
	trajectory := anySlice(report["trajectory"])
	requireSpreadsheetPlanSafetyReport(t, report, trajectory, raw)
	if !boolValue(report["singularity_analysis_available"]) ||
		boolValue(report["singularity_gate_clear"]) ||
		numericValue(report["singularity_hard_stop_point_count"]) == 0 ||
		stringValue(report["validation_state"]) != "invalid" ||
		boolValue(report["play_enabled"]) ||
		boolValue(report["runtime_preload_enabled"]) ||
		boolValue(report["isolated_simulation_preload_enabled"]) ||
		!boolValue(report["simulation_preview_enabled"]) {
		t.Fatalf("hard singularity did not fail closed while keeping its path previewable: %s", raw)
	}
	sawStoppedMotionSample := false
	for index, entry := range trajectory {
		if index == 0 {
			continue
		}
		sample, _ := entry.(map[string]any)
		if stringValue(sample["singularity_class"]) != "hard_stop" {
			continue
		}
		if numericValue(sample["applied_velocity_scale"]) != 0 {
			t.Fatalf("hard singularity sample %d retained nonzero common velocity scale: %#v", index, sample)
		}
		for axis, velocity := range anySlice(sample["scaled_velocities_rad_s"]) {
			if numericValue(velocity) != 0 {
				t.Fatalf("hard singularity sample %d J%d retained scaled velocity %.9f", index, axis+1, numericValue(velocity))
			}
		}
		sawStoppedMotionSample = true
	}
	if !sawStoppedMotionSample {
		t.Fatalf("hard singularity report has no stopped non-initial trajectory sample: %s", raw)
	}
}

func TestRobotV4MotionServerAcceleratedGate1HoldsEveryAcceptedSample(t *testing.T) {

	binary := ensureMotionServerCartesianTestBinary(t, findRepoRoot(t))
	output, err := exec.Command(binary, "--backend", "legacy_v4", "--udp-listen", "127.0.0.1:0").CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(string(output), "reason=backend_retired") {
		t.Fatalf("retired accelerated executor accepted: %v %s", err, output)
	}

}

func TestRobotV4MotionServerSpreadsheetPlanHonestyUnreachable(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(t, repoRoot, "spreadsheet-plan-unreachable-input.json")
	requireHonestSpreadsheetPlanValidation(t, report, raw)
	if stringValue(report["validation_state"]) != "invalid" {
		t.Fatalf("validation_state = %q, want invalid for an out-of-workspace waypoint; report=%s",
			stringValue(report["validation_state"]), string(raw))
	}
	unreachable := anySlice(report["unreachable_points"])
	if len(unreachable) == 0 {
		t.Fatalf("unreachable_points empty for an out-of-workspace waypoint; report=%s", string(raw))
	}
	sawIKFailure := false
	for index, entry := range unreachable {
		point, ok := entry.(map[string]any)
		if !ok || len(anySlice(point["tcp_pose"])) != 6 || numericValue(point["time_s"]) <= 0 {
			t.Fatalf("unreachable point %d malformed: %#v; report=%s", index, entry, string(raw))
		}
		reason := stringValue(point["reason"])
		if reason != "ik_no_solution" && reason != "joint_limit" {
			t.Fatalf("unreachable point %d reason = %q, want ik_no_solution or joint_limit; report=%s", index, reason, string(raw))
		}
		if reason == "ik_no_solution" && numericValue(anySlice(point["tcp_pose"])[0]) > 1.5 {
			sawIKFailure = true
		}
	}
	if !sawIKFailure {
		t.Fatalf("no ik_no_solution point recorded beyond the reachable envelope; report=%s", string(raw))
	}
	trajectory := anySlice(report["trajectory"])
	if len(trajectory) != int(numericValue(report["planned_sample_count"])) || len(trajectory) < 6 {
		t.Fatalf("diagnostic trajectory truncated to %d samples (planned_sample_count %v); the full map must survive IK failures; report=%s",
			len(trajectory), report["planned_sample_count"], string(raw))
	}
}

func TestRobotV4MotionServerSpreadsheetPlanHonestyCollision(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(t, repoRoot, "spreadsheet-plan-collision-input.json")
	requireHonestSpreadsheetPlanValidation(t, report, raw)
	state := stringValue(report["collision_check_state"])
	if state != "tesseract_discrete_contact_checked" {
		t.Skipf("tesseract collision worker unavailable (%q): run `pixi install` under tesseract/v1 to enable the genuine contact check", state)
	}
	if stringValue(report["validation_state"]) != "invalid" {
		t.Fatalf("validation_state = %q, want invalid for a path into the base column; report=%s",
			stringValue(report["validation_state"]), string(raw))
	}
	collisions := anySlice(report["collision_points"])
	if len(collisions) == 0 {
		t.Fatalf("collision_points empty for a path into the base column; report=%s", string(raw))
	}
	if boolValue(report["collision_gate_clear"]) || boolValue(report["play_enabled"]) ||
		boolValue(report["runtime_preload_enabled"]) ||
		!boolValue(report["simulation_preview_enabled"]) ||
		boolValue(report["isolated_simulation_preload_enabled"]) {
		t.Fatalf("colliding plan must block every preload/play path while remaining previewable in simulation; report=%s", string(raw))
	}
	firstCollision, _ := collisions[0].(map[string]any)
	firstRowIndex := int(numericValue(firstCollision["row_index"]))
	matchingRows := 0
	for _, entry := range anySlice(report["program_rows"]) {
		row, _ := entry.(map[string]any)
		if int(numericValue(row["row_index"])) == firstRowIndex {
			matchingRows++
		}
	}
	if matchingRows != 1 {
		t.Fatalf("first collision row %d maps to %d selectable program rows, want exactly one; report=%s", firstRowIndex, matchingRows, string(raw))
	}
	sawBaseContact := false
	maxDepth := 0.0
	for _, entry := range collisions {
		point, _ := entry.(map[string]any)
		linkA := stringValue(point["link_a"])
		linkB := stringValue(point["link_b"])
		if linkA == "link_1" || linkB == "link_1" || linkA == "floor" || linkB == "floor" {
			sawBaseContact = true
		}
		if numericValue(point["depth_m"]) > maxDepth {
			maxDepth = numericValue(point["depth_m"])
		}
	}
	if !sawBaseContact {
		t.Fatalf("no contact against the base column (link_1) or floor recorded; report=%s", string(raw))
	}
	if maxDepth <= 0 {
		t.Fatalf("collision points carry no positive penetration depth; report=%s", string(raw))
	}
	t.Logf("MOTION_SERVER_HONEST_COLLISION contacts=%d max_depth_m=%.4f checker=%s version=%s check_time_s=%.3f",
		len(collisions), maxDepth,
		stringValue(report["collision_checker"]),
		stringValue(report["collision_checker_library_version"]),
		numericValue(report["collision_check_time_s"]))
}

func TestRobotV4MotionServerCollisionRejectionDemoIsSimulationPreviewOnly(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report, raw := runSpreadsheetPlanSelfTest(
		t, repoRoot, "spreadsheet-plan-collision-demo-ref-input.json")
	if stringValue(report["program_id"]) != "robot-v4-tour-collision-rejection-demo" ||
		stringValue(report["program_source"]) != "motion-server-builtin-simulation-only-program-database" ||
		stringValue(report["execution_policy"]) != "simulation-preview-only" ||
		!boolValue(report["simulation_only_program"]) {
		t.Fatalf("collision rejection demo did not resolve as the motion-server-owned simulation fixture: %s", string(raw))
	}
	state := stringValue(report["collision_check_state"])
	if state != "tesseract_discrete_contact_checked" {
		t.Skipf("tesseract collision worker unavailable (%q): run `pixi install` under tesseract/v1 to enable the genuine contact check", state)
	}
	if stringValue(report["validation_state"]) != "invalid" ||
		numericValue(report["collision_point_count"]) <= 0 ||
		boolValue(report["collision_gate_clear"]) ||
		boolValue(report["motion_allowed"]) ||
		boolValue(report["play_enabled"]) ||
		boolValue(report["runtime_preload_enabled"]) ||
		boolValue(report["isolated_simulation_preload_enabled"]) ||
		!boolValue(report["simulation_preview_enabled"]) {
		t.Fatalf("collision rejection demo must expose contacts while every motion/preload gate stays blocked: %s", string(raw))
	}
	if len(anySlice(report["collision_points"])) == 0 || len(anySlice(report["trajectory"])) < 2 {
		t.Fatalf("collision rejection demo must carry visible trajectory and contact evidence: %s", string(raw))
	}
}

func TestRobotV4MotionServerNATSStatusCoalescesHotLoopWrites(t *testing.T) {
	source := readMotionServerCartesianCompilationSource(t, findRepoRoot(t))
	for _, want := range []string{
		"static std::atomic<std::int64_t> next_allowed_ns{0}",
		"now_ns < next_ns",
		"std::chrono::milliseconds(250)",
		"source timestamp plus remaining TTL has expired",
	} {
		if !strings.Contains(source, want) {
			t.Fatalf(
				"motion-server NATS status coalescing contract missing %q",
				want,
			)
		}
	}
}
