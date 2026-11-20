package planning

import (
	"encoding/json"
	"strings"
	"testing"
	"time"
)

func strictPlanReport(t *testing.T, request NormalizedPlanRequest, planRevision uint64) []byte {
	t.Helper()
	trajectory := []map[string]any{
		{
			"row_index": 0, "waypoint_index": 0, "time_s": 0.0,
			"move_type": "line_tcp", "motion_phase": "travel", "weld_on": false,
			"tcp_pose":               []float64{0, 0, 0, 0, 0, 0},
			"positions_rad":          []float64{0, 0, 0, 0, 0, 0, 0, 0, 0},
			"applied_velocity_scale": 1.0, "singularity_class": "clear",
		},
		{
			"row_index": 1, "waypoint_index": 1, "time_s": 1.0,
			"move_type": "line_tcp", "motion_phase": "weld", "weld_on": true,
			"tcp_pose":               []float64{0.1, 0, 0, 0, 0, 0},
			"positions_rad":          []float64{0.1, 0, 0, 0, 0, 0, 0, 0, 0},
			"applied_velocity_scale": 0.8, "singularity_class": "proportional",
			"limiting_joint": "J2",
		},
	}
	report := map[string]any{
		"plan_request_revision":          planRevision,
		"planner_owner":                  MotionServerPlannerOwner,
		"planner_backend":                MotionServerPlannerBackend,
		"planner_status":                 "planned",
		"program_id":                     request.ProgramID,
		"program_revision":               request.ProgramRevision,
		"accepted_program_revision":      request.ProgramRevision,
		"program_hash":                   "fnv1a64:1234",
		"robot_model":                    request.RobotModel,
		"validation_state":               "valid",
		"accepted":                       true,
		"play_enabled":                   true,
		"runtime_preload_enabled":        true,
		"collision_gate_clear":           true,
		"collision_check_state":          "tesseract_discrete_contact_checked",
		"collision_points":               []any{},
		"singularity_analysis_available": true,
		"singularity_gate_clear":         true,
		"singularity_check_state":        "jacobian_svd_checked",
		"singularity_points":             []any{},
		"limit_check_state":              "clear",
		"unreachable_points":             []any{},
		"planned_sample_count":           len(trajectory),
		"trajectory":                     trajectory,
		"hardware_access":                false,
	}
	report["collision_checked_waypoint_count"] = len(trajectory)
	raw, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func TestNormalizeMotionServerPlanResultRequiresCompleteCollisionCoverage(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 30, 0, 0, time.UTC)
	context := strictContext(now)
	tests := []struct {
		name   string
		mutate func(map[string]any)
		reason string
	}{
		{
			name: "coverage count missing",
			mutate: func(report map[string]any) {
				delete(report, "collision_checked_waypoint_count")
			},
			reason: "collision_checked_waypoint_count_missing",
		},
		{
			name: "coverage count zero",
			mutate: func(report map[string]any) {
				report["collision_checked_waypoint_count"] = float64(0)
			},
			reason: "collision_checked_waypoint_count_invalid",
		},
		{
			name: "coverage count does not cover every planned sample",
			mutate: func(report map[string]any) {
				report["collision_checked_waypoint_count"] = float64(1)
			},
			reason: "collision_checked_waypoint_count_mismatch",
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var report map[string]any
			if err := json.Unmarshal(
				strictPlanReport(t, context.Request, context.PlanRevision),
				&report,
			); err != nil {
				t.Fatal(err)
			}
			test.mutate(report)
			raw, err := json.Marshal(report)
			if err != nil {
				t.Fatal(err)
			}
			result, err := NormalizeMotionServerPlanResult(context, raw, 100)
			if err != nil {
				t.Fatalf("normalization should retain evidence: %v", err)
			}
			if result.RuntimePlayable() ||
				!containsReason(result.ReportReasons, test.reason) ||
				result.Evidence.State != PlanRejected {
				t.Fatalf("incomplete collision coverage did not fail closed: %+v", result)
			}
		})
	}
}

func strictContext(now time.Time) ReportContext {
	request := validPlanRequest(now)
	request.Target = TargetIdentity{ProfileID: "local-simulator"}
	return ReportContext{
		Request:      request,
		PlanID:       "local/plan-1",
		PlanRevision: 41,
		ReceivedAt:   now,
	}
}

func TestNormalizeMotionServerPlanResultRequiresCanonicalCompleteEvidence(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 30, 0, 0, time.UTC)
	context := strictContext(now)
	result, err := NormalizeMotionServerPlanResult(
		context,
		strictPlanReport(t, context.Request, context.PlanRevision),
		100,
	)
	if err != nil {
		t.Fatalf("normalize strict result: %v", err)
	}
	if !result.RuntimePlayable() || len(result.Trajectory) != 2 ||
		result.Trajectory[1].LimitingJoint != "J2" {
		t.Fatalf("strict result not playable: %+v", result)
	}
}

func TestNormalizeMotionServerPlanResultFailsClosedForMissingOrMismatchedProof(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 30, 0, 0, time.UTC)
	context := strictContext(now)
	tests := []struct {
		name   string
		mutate func(map[string]any)
		reason string
	}{
		{
			name: "revision mismatch",
			mutate: func(report map[string]any) {
				report["plan_request_revision"] = float64(context.PlanRevision + 1)
			},
			reason: "plan_request_revision_mismatch",
		},
		{
			name:   "noncanonical backend",
			mutate: func(report map[string]any) { report["planner_backend"] = "browser" },
			reason: "planner_backend_mismatch",
		},
		{
			name:   "hardware proof missing",
			mutate: func(report map[string]any) { delete(report, "hardware_access") },
			reason: "planner_hardware_access_proof_missing",
		},
		{
			name:   "hardware access enabled",
			mutate: func(report map[string]any) { report["hardware_access"] = true },
			reason: "planner_hardware_access_enabled",
		},
		{
			name:   "trajectory missing",
			mutate: func(report map[string]any) { delete(report, "trajectory") },
			reason: "trajectory_missing",
		},
		{
			name:   "sample count mismatch",
			mutate: func(report map[string]any) { report["planned_sample_count"] = float64(3) },
			reason: "planned_sample_count_mismatch",
		},
		{
			name: "noncanonical trajectory move type",
			mutate: func(report map[string]any) {
				report["trajectory"].([]any)[0].(map[string]any)["move_type"] = "browser_pose"
			},
			reason: "trajectory_move_type_invalid",
		},
		{
			name: "malformed clear singularity warning",
			mutate: func(report map[string]any) {
				report["singularity_points"] = []any{map[string]any{
					"row_index": 0, "waypoint_index": 0,
					"singularity_class":      "warning",
					"applied_velocity_scale": 2.0,
					"positions_rad":          []float64{0, 0, 0, 0, 0, 0, 0, 0, 0},
				}}
			},
			reason: "singularity_evidence_point_invalid",
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var report map[string]any
			if err := json.Unmarshal(
				strictPlanReport(t, context.Request, context.PlanRevision),
				&report,
			); err != nil {
				t.Fatal(err)
			}
			test.mutate(report)
			raw, err := json.Marshal(report)
			if err != nil {
				t.Fatal(err)
			}
			result, err := NormalizeMotionServerPlanResult(context, raw, 100)
			if err != nil {
				t.Fatalf("normalization should retain evidence: %v", err)
			}
			if result.RuntimePlayable() ||
				!containsReason(result.ReportReasons, test.reason) ||
				result.Evidence.State != PlanRejected {
				t.Fatalf("unsafe report did not fail closed: %+v", result)
			}
		})
	}
}

func TestNormalizeMotionServerPlanResultRetainsCanonicalBlockedEvidence(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 30, 0, 0, time.UTC)
	context := strictContext(now)
	var report map[string]any
	if err := json.Unmarshal(
		strictPlanReport(t, context.Request, context.PlanRevision),
		&report,
	); err != nil {
		t.Fatal(err)
	}
	report["accepted"] = false
	report["validation_state"] = "invalid"
	report["play_enabled"] = false
	report["runtime_preload_enabled"] = false
	report["collision_gate_clear"] = false
	report["collision_points"] = []map[string]any{{
		"row_index": 1, "waypoint_index": 1,
		"link_a": "link_2", "link_b": "fixture", "depth_m": 0.01,
		"affected_joints": []string{"J2"},
		"positions_rad":   []float64{0.1, 0, 0, 0, 0, 0, 0, 0, 0},
	}}
	raw, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	result, err := NormalizeMotionServerPlanResult(context, raw, 100)
	if err != nil {
		t.Fatal(err)
	}
	if result.RuntimePlayable() ||
		result.Evidence.Collision.State != CheckBlocked ||
		len(result.Evidence.Collision.Points) != 1 ||
		len(result.ReportReasons) != 0 {
		t.Fatalf("collision evidence was lost or accepted: %+v", result)
	}
}

func TestNormalizeMotionServerPlanResultRejectsMalformedTrajectory(t *testing.T) {
	now := time.Date(2026, 7, 30, 13, 30, 0, 0, time.UTC)
	context := strictContext(now)
	var report map[string]any
	if err := json.Unmarshal(
		strictPlanReport(t, context.Request, context.PlanRevision),
		&report,
	); err != nil {
		t.Fatal(err)
	}
	trajectory := report["trajectory"].([]any)
	point := trajectory[1].(map[string]any)
	point["time_s"] = -1.0
	point["positions_rad"] = []float64{0, 0}
	raw, _ := json.Marshal(report)
	result, err := NormalizeMotionServerPlanResult(context, raw, 100)
	if err != nil {
		t.Fatal(err)
	}
	if result.RuntimePlayable() ||
		!containsReason(result.ReportReasons, "trajectory_time_invalid") ||
		!containsReason(result.ReportReasons, "trajectory_joint_positions_invalid") {
		t.Fatalf("malformed trajectory accepted: %+v", result)
	}
}

func TestTagMotionServerReportRevisionIsExactAndIdempotent(t *testing.T) {
	raw := []byte(`{"planner_status":"planned"}`)
	tagged, err := TagMotionServerReportRevision(raw, 17)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(tagged), `"plan_request_revision":17`) {
		t.Fatalf("tagged report = %s", tagged)
	}
	again, err := TagMotionServerReportRevision(tagged, 17)
	if err != nil || !json.Valid(again) {
		t.Fatalf("idempotent tag failed: %v %s", err, again)
	}
	if _, err := TagMotionServerReportRevision(tagged, 18); err == nil {
		t.Fatal("mismatched existing revision was overwritten")
	}
}
