package planning

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"strings"
)

const NormalizedPlanResultSchema = "offline-programming.normalized-plan-result.v1"

type TrajectorySample struct {
	RowIndex         int                 `json:"row_index"`
	WaypointIndex    int                 `json:"waypoint_index"`
	TimeS            float64             `json:"time_s"`
	MoveType         string              `json:"move_type"`
	MotionPhase      string              `json:"motion_phase"`
	WeldOn           bool                `json:"weld_on"`
	TCPPose          [6]float64          `json:"tcp_pose"`
	PositionsRad     [JointCount]float64 `json:"positions_rad"`
	VelocityScale    float64             `json:"applied_velocity_scale"`
	LimitingJoint    string              `json:"limiting_joint,omitempty"`
	SingularityClass string              `json:"singularity_class,omitempty"`
}

type NormalizedPlanResult struct {
	Schema        string                 `json:"schema"`
	Evidence      NormalizedPlanEvidence `json:"evidence"`
	Trajectory    []TrajectorySample     `json:"trajectory"`
	ReportReasons []string               `json:"report_reasons"`
	AcceptedPlan  *RobotV4AcceptedPlan   `json:"accepted_plan,omitempty"`
}

func (result NormalizedPlanResult) RuntimePlayable() bool {
	return result.Schema == NormalizedPlanResultSchema &&
		result.Evidence.ServerValidated() &&
		len(result.ReportReasons) == 0 &&
		result.Evidence.SampleCount > 0 &&
		len(result.Trajectory) == result.Evidence.SampleCount
}

type strictMotionServerReport struct {
	PlanRequestRevision           uint64                `json:"plan_request_revision"`
	PlannerStatus                 string                `json:"planner_status"`
	PlannerBackend                string                `json:"planner_backend"`
	HardwareAccess                *bool                 `json:"hardware_access"`
	CollisionCheckedWaypointCount *int                  `json:"collision_checked_waypoint_count"`
	Trajectory                    *[]rawTrajectoryPoint `json:"trajectory"`
}

type rawTrajectoryPoint struct {
	RowIndex         int       `json:"row_index"`
	WaypointIndex    int       `json:"waypoint_index"`
	TimeS            float64   `json:"time_s"`
	MoveType         string    `json:"move_type"`
	MotionPhase      string    `json:"motion_phase"`
	WeldOn           bool      `json:"weld_on"`
	TCPPose          []float64 `json:"tcp_pose"`
	PositionsRad     []float64 `json:"positions_rad"`
	VelocityScale    float64   `json:"applied_velocity_scale"`
	LimitingJoint    string    `json:"limiting_joint"`
	SingularityClass string    `json:"singularity_class"`
}

// NormalizeMotionServerPlanResult applies the stricter local-service contract
// on top of the version-tolerant evidence normalizer. A report is playable
// only when it carries the exact transport revision, canonical planner
// identity, explicit no-hardware proof, complete collision coverage, and a
// complete bounded trajectory.
func NormalizeMotionServerPlanResult(
	context ReportContext,
	raw []byte,
	maxTrajectorySamples int,
) (NormalizedPlanResult, error) {
	evidence, err := NormalizeMotionServerReport(context, raw)
	if err != nil {
		return NormalizedPlanResult{}, err
	}
	if maxTrajectorySamples < 1 {
		return NormalizedPlanResult{}, errors.New("planning: maximum trajectory samples must be positive")
	}
	var report strictMotionServerReport
	if err := decodeOneJSON(raw, &report); err != nil {
		return NormalizedPlanResult{}, fmt.Errorf("planning: decode strict motion-server report: %w", err)
	}

	result := NormalizedPlanResult{
		Schema:        NormalizedPlanResultSchema,
		Evidence:      evidence,
		Trajectory:    []TrajectorySample{},
		ReportReasons: []string{},
	}
	addReason := func(reason string) {
		if !containsString(result.ReportReasons, reason) {
			result.ReportReasons = append(result.ReportReasons, reason)
		}
	}
	if report.PlanRequestRevision == 0 {
		addReason("plan_request_revision_missing")
	} else if report.PlanRequestRevision != context.PlanRevision {
		addReason("plan_request_revision_mismatch")
	}
	if strings.TrimSpace(report.PlannerBackend) != MotionServerPlannerBackend {
		addReason("planner_backend_mismatch")
	}
	if strings.TrimSpace(report.PlannerStatus) != "planned" {
		addReason("planner_status_not_planned")
	}
	if report.HardwareAccess == nil {
		addReason("planner_hardware_access_proof_missing")
	} else if *report.HardwareAccess {
		addReason("planner_hardware_access_enabled")
	}
	if report.Trajectory == nil {
		addReason("trajectory_missing")
	} else if len(*report.Trajectory) > maxTrajectorySamples {
		addReason("trajectory_sample_limit_exceeded")
	} else {
		trajectory, reasons := normalizeTrajectory(context.Request, *report.Trajectory)
		result.Trajectory = trajectory
		for _, reason := range reasons {
			addReason(reason)
		}
	}
	if evidence.SampleCount <= 0 {
		addReason("planned_sample_count_invalid")
	}
	if report.Trajectory != nil && len(*report.Trajectory) != evidence.SampleCount {
		addReason("planned_sample_count_mismatch")
	}
	if report.CollisionCheckedWaypointCount == nil {
		addReason("collision_checked_waypoint_count_missing")
	} else if *report.CollisionCheckedWaypointCount <= 0 {
		addReason("collision_checked_waypoint_count_invalid")
	} else if *report.CollisionCheckedWaypointCount != evidence.SampleCount {
		addReason("collision_checked_waypoint_count_mismatch")
	}
	for _, reason := range validateCanonicalEvidence(context.Request, evidence) {
		addReason(reason)
	}

	if len(result.ReportReasons) > 0 {
		result.Evidence.State = PlanRejected
		result.Evidence.CorrelationReasons = appendUnique(
			result.Evidence.CorrelationReasons,
			result.ReportReasons...,
		)
	}
	return result, nil
}

// TagMotionServerReportRevision adds the transport correlation produced by the
// in-process Robot V4 plan handler to a canonical planner report.
// An existing, mismatched revision is rejected rather than overwritten.
func TagMotionServerReportRevision(raw []byte, planRevision uint64) ([]byte, error) {
	if planRevision == 0 {
		return nil, errors.New("planning: plan revision must be nonzero")
	}
	var report map[string]json.RawMessage
	if err := decodeOneJSON(raw, &report); err != nil {
		return nil, err
	}
	if existing, ok := report["plan_request_revision"]; ok {
		var revision uint64
		if err := json.Unmarshal(existing, &revision); err != nil || revision != planRevision {
			return nil, errors.New("planning: motion-server report revision is invalid or mismatched")
		}
		return append([]byte(nil), raw...), nil
	}
	encoded, err := json.Marshal(planRevision)
	if err != nil {
		return nil, err
	}
	report["plan_request_revision"] = encoded
	tagged, err := json.Marshal(report)
	if err != nil {
		return nil, fmt.Errorf("planning: tag motion-server report revision: %w", err)
	}
	return tagged, nil
}

func normalizeTrajectory(
	request NormalizedPlanRequest,
	raw []rawTrajectoryPoint,
) ([]TrajectorySample, []string) {
	out := make([]TrajectorySample, 0, len(raw))
	reasons := []string{}
	previousTime := -1.0
	for index, point := range raw {
		valid := true
		if point.WaypointIndex != index {
			reasons = append(reasons, "trajectory_waypoint_index_invalid")
			valid = false
		}
		if point.RowIndex < 0 || point.RowIndex >= len(request.Rows) {
			reasons = append(reasons, "trajectory_row_index_invalid")
			valid = false
		}
		if !finite(point.TimeS) || point.TimeS < 0 || point.TimeS < previousTime {
			reasons = append(reasons, "trajectory_time_invalid")
			valid = false
		}
		if len(point.TCPPose) != 6 || !finiteArray(point.TCPPose) {
			reasons = append(reasons, "trajectory_tcp_pose_invalid")
			valid = false
		}
		if len(point.PositionsRad) != JointCount || !finiteArray(point.PositionsRad) {
			reasons = append(reasons, "trajectory_joint_positions_invalid")
			valid = false
		}
		if !canonicalTrajectoryMoveType(point.MoveType) {
			reasons = append(reasons, "trajectory_move_type_invalid")
			valid = false
		}
		if strings.TrimSpace(point.MotionPhase) == "" {
			reasons = append(reasons, "trajectory_motion_phase_invalid")
			valid = false
		}
		if !finite(point.VelocityScale) || point.VelocityScale < 0 || point.VelocityScale > 1 {
			reasons = append(reasons, "trajectory_velocity_scale_invalid")
			valid = false
		}
		if !valid {
			continue
		}
		var tcp [6]float64
		var joints [JointCount]float64
		copy(tcp[:], point.TCPPose)
		copy(joints[:], point.PositionsRad)
		out = append(out, TrajectorySample{
			RowIndex:         point.RowIndex,
			WaypointIndex:    point.WaypointIndex,
			TimeS:            point.TimeS,
			MoveType:         strings.TrimSpace(point.MoveType),
			MotionPhase:      strings.TrimSpace(point.MotionPhase),
			WeldOn:           point.WeldOn,
			TCPPose:          tcp,
			PositionsRad:     joints,
			VelocityScale:    point.VelocityScale,
			LimitingJoint:    strings.TrimSpace(point.LimitingJoint),
			SingularityClass: strings.TrimSpace(point.SingularityClass),
		})
		previousTime = point.TimeS
	}
	return out, uniqueStrings(reasons)
}

func validateCanonicalEvidence(
	request NormalizedPlanRequest,
	evidence NormalizedPlanEvidence,
) []string {
	reasons := []string{}
	for _, point := range evidence.Collision.Points {
		if point.RowIndex < 0 ||
			point.RowIndex >= len(request.Rows) ||
			point.WaypointIndex < 0 ||
			(evidence.SampleCount > 0 && point.WaypointIndex >= evidence.SampleCount) ||
			strings.TrimSpace(point.LinkA) == "" ||
			strings.TrimSpace(point.LinkB) == "" ||
			!finite(point.DepthM) ||
			point.DepthM < 0 ||
			(len(point.PositionsRad) != 0 && len(point.PositionsRad) != JointCount) ||
			!finiteArray(point.PositionsRad) {
			reasons = append(reasons, "collision_evidence_point_invalid")
			break
		}
	}
	for _, point := range evidence.Singularity.Points {
		state := strings.TrimSpace(point.State)
		canonicalState := state == "warning" || state == "hard_stop" || state == "unavailable"
		if point.RowIndex < 0 ||
			point.RowIndex >= len(request.Rows) ||
			point.WaypointIndex < 0 ||
			(evidence.SampleCount > 0 && point.WaypointIndex >= evidence.SampleCount) ||
			!canonicalState ||
			(evidence.Singularity.State == CheckClear && state != "warning") ||
			!finite(point.AppliedVelocityScale) ||
			point.AppliedVelocityScale < 0 ||
			point.AppliedVelocityScale > 1 ||
			(evidence.Singularity.State == CheckClear && len(point.PositionsRad) != JointCount) ||
			(evidence.Singularity.State != CheckClear &&
				len(point.PositionsRad) != 0 &&
				len(point.PositionsRad) != JointCount) ||
			!finiteArray(point.PositionsRad) {
			reasons = append(reasons, "singularity_evidence_point_invalid")
			break
		}
	}
	for _, point := range evidence.Limits.Points {
		if point.RowIndex < 0 ||
			point.RowIndex >= len(request.Rows) ||
			point.WaypointIndex < 0 ||
			(evidence.SampleCount > 0 && point.WaypointIndex >= evidence.SampleCount) ||
			strings.TrimSpace(point.Reason) == "" ||
			(len(point.PositionsRad) != 0 && len(point.PositionsRad) != JointCount) ||
			!finiteArray(point.PositionsRad) {
			reasons = append(reasons, "limit_evidence_point_invalid")
			break
		}
	}
	return uniqueStrings(reasons)
}

func canonicalTrajectoryMoveType(value string) bool {
	switch strings.TrimSpace(value) {
	case "line_tcp", "curve_tcp", "joint_ptp":
		return true
	default:
		return false
	}
}

func decodeOneJSON(raw []byte, out any) error {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	if err := decoder.Decode(out); err != nil {
		return err
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return errors.New("multiple JSON values")
	}
	return nil
}

func finite(value float64) bool {
	return !math.IsNaN(value) && !math.IsInf(value, 0)
}

func appendUnique(values []string, additions ...string) []string {
	for _, addition := range additions {
		if !containsString(values, addition) {
			values = append(values, addition)
		}
	}
	return values
}

func uniqueStrings(values []string) []string {
	return appendUnique(nil, values...)
}

func containsString(values []string, wanted string) bool {
	for _, value := range values {
		if value == wanted {
			return true
		}
	}
	return false
}
