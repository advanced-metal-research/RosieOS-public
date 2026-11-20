package planning

import (
	"encoding/json"
	"strings"
	"testing"
	"time"
)

func validPlanRequest(now time.Time) NormalizedPlanRequest {
	return NormalizedPlanRequest{
		Schema:              NormalizedPlanRequestSchema,
		RequestID:           "request-17",
		ProjectID:           "project-dog-box",
		ProjectRevision:     7,
		ProgramID:           "program-dog-box",
		ProgramRevision:     11,
		BaseProgramRevision: 10,
		RobotModel:          RobotModelRosie1400V3,
		Target: TargetIdentity{
			ProfileID:   "bench-relay",
			ManifestRef: "rt-core/config/cells/bench-relay.json",
		},
		SubmittedAt: now.Add(-time.Second),
		Rows: []PlanRow{
			{RowID: "row-1", Revision: 11, Enabled: true, RowType: "motion"},
			{RowID: "row-2", Revision: 11, Enabled: true, RowType: "motion"},
		},
	}
}

func withRobotV4Program(t *testing.T, request NormalizedPlanRequest) NormalizedPlanRequest {
	t.Helper()
	rows := make([]map[string]any, len(request.Rows))
	for index, row := range request.Rows {
		rows[index] = map[string]any{
			"row_id":               row.RowID,
			"row_index":            index,
			"revision":             row.Revision,
			"row_hash":             "sha256:test-row",
			"enabled":              row.Enabled,
			"row_type":             row.RowType,
			"command":              row.Command,
			"move_type":            row.MoveType,
			"motion_phase":         row.MotionPhase,
			"tool_frame":           row.ToolFrame,
			"work_frame":           row.WorkFrame,
			"target_tcp_m":         row.TargetTCPM,
			"target_joints_rad":    row.TargetJointsRad,
			"tcp_velocity_m_s":     row.TCPVelocityMS,
			"joint_velocity_rad_s": row.JointVelocityRadS,
			"weld_enable":          row.WeldEnable,
			"arc_state":            row.ArcState,
			"test_unknown_field":   "preserve-me",
		}
	}
	raw, err := json.Marshal(map[string]any{
		"schema_version":        1,
		"program_id":            request.ProgramID,
		"program_name":          "Planning test",
		"program_revision":      request.ProgramRevision,
		"base_program_revision": request.BaseProgramRevision,
		"robot_model":           request.RobotModel,
		"robotic-cell-type":     request.RobotModel,
		"execution_mode":        "simulation",
		"rows":                  rows,
		"test_unknown_program":  map[string]any{"preserve": true},
	})
	if err != nil {
		t.Fatal(err)
	}
	request.Program = raw
	return request
}

func validMotionServerReport() string {
	return `{
		"planner_owner":"motion-server",
		"planner_backend":"tesseract",
		"program_id":"program-dog-box",
		"program_revision":11,
		"accepted_program_revision":11,
		"program_hash":"fnv1a64:1234",
		"robot_model":"rosie_1400_v3",
		"validation_state":"valid",
		"accepted":true,
		"play_enabled":true,
		"runtime_preload_enabled":true,
		"collision_gate_clear":true,
		"collision_check_state":"tesseract_discrete_contact_checked",
		"collision_points":[],
		"singularity_analysis_available":true,
		"singularity_gate_clear":true,
		"singularity_check_state":"jacobian_svd_checked",
		"singularity_points":[],
		"limit_check_state":"clear",
		"unreachable_points":[],
		"planned_sample_count":64,
		"future_report_field":"ignored_for_version_tolerance"
	}`
}

func TestNormalizeMotionServerReportRequiresExplicitEvidence(t *testing.T) {
	now := time.Date(2026, 7, 30, 7, 0, 0, 0, time.UTC)
	report := strings.Replace(validMotionServerReport(), `"collision_points":[]`, `"collision_point_count":0`, 1)
	evidence, err := NormalizeMotionServerReport(ReportContext{
		Request:      validPlanRequest(now),
		PlanID:       "plan-17",
		PlanRevision: 23,
		ReceivedAt:   now,
	}, []byte(report))
	if err != nil {
		t.Fatalf("normalize report: %v", err)
	}
	if evidence.Collision.State != CheckUnavailable {
		t.Fatalf("missing collision array became %q, want unavailable", evidence.Collision.State)
	}
	if evidence.State == PlanServerValidated || evidence.ServerValidated() {
		t.Fatal("report with missing collision evidence became executable")
	}
}

func TestNormalizeMotionServerReportPreservesEmptyAffectedJointArray(t *testing.T) {
	now := time.Date(2026, 7, 30, 7, 0, 0, 0, time.UTC)
	report := strings.Replace(
		validMotionServerReport(),
		`"singularity_points":[]`,
		`"singularity_points":[{
			"row_index":0,
			"waypoint_index":0,
			"state":"warning",
			"applied_velocity_scale":0.5,
			"positions_rad":[0,0,0,0,0,0,0,0,0]
		}]`,
		1,
	)
	evidence, err := NormalizeMotionServerReport(ReportContext{
		Request:      validPlanRequest(now),
		PlanID:       "plan-array-contract",
		PlanRevision: 1,
		ReceivedAt:   now,
	}, []byte(report))
	if err != nil {
		t.Fatal(err)
	}
	wire, err := json.Marshal(evidence)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(wire), `"affected_joints":[]`) {
		t.Fatalf("normalized singularity point emitted null affected_joints: %s", wire)
	}
}

func TestNormalizeMotionServerReportCorrelatesExactIdentity(t *testing.T) {
	now := time.Date(2026, 7, 30, 7, 0, 0, 0, time.UTC)
	evidence, err := NormalizeMotionServerReport(ReportContext{
		Request:      validPlanRequest(now),
		PlanID:       "plan-17",
		PlanRevision: 23,
		ReceivedAt:   now,
	}, []byte(validMotionServerReport()))
	if err != nil {
		t.Fatalf("normalize report: %v", err)
	}
	if !evidence.ServerValidated() || evidence.SampleCount != 64 {
		t.Fatalf("valid evidence not normalized as server validated: %+v", evidence)
	}

	mismatched := strings.Replace(validMotionServerReport(), `"robot_model":"rosie_1400_v3"`, `"robot_model":"rosie_1420_v1"`, 1)
	evidence, err = NormalizeMotionServerReport(ReportContext{
		Request:      validPlanRequest(now),
		PlanID:       "plan-17",
		PlanRevision: 23,
		ReceivedAt:   now,
	}, []byte(mismatched))
	if err != nil {
		t.Fatalf("normalize mismatched report: %v", err)
	}
	if evidence.State != PlanRejected || !containsReason(evidence.CorrelationReasons, "robot_model_mismatch") {
		t.Fatalf("model mismatch did not reject exact correlation: %+v", evidence)
	}
}

func containsReason(reasons []string, want string) bool {
	for _, reason := range reasons {
		if reason == want {
			return true
		}
	}
	return false
}
