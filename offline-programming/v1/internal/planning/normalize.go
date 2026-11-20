package planning

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"strings"
	"time"
)

type ReportContext struct {
	Request      NormalizedPlanRequest
	PlanID       string
	PlanRevision uint64
	ReceivedAt   time.Time
}

type motionServerReport struct {
	PlannerOwner                   string                 `json:"planner_owner"`
	PlannerBackend                 string                 `json:"planner_backend"`
	ProgramID                      string                 `json:"program_id"`
	ProgramRevision                uint64                 `json:"program_revision"`
	AcceptedProgramRevision        uint64                 `json:"accepted_program_revision"`
	ProgramHash                    string                 `json:"program_hash"`
	RobotModel                     RobotModel             `json:"robot_model"`
	ValidationState                string                 `json:"validation_state"`
	Accepted                       *bool                  `json:"accepted"`
	PlayEnabled                    *bool                  `json:"play_enabled"`
	RuntimePreloadEnabled          *bool                  `json:"runtime_preload_enabled"`
	CollisionGateClear             *bool                  `json:"collision_gate_clear"`
	CollisionCheckState            string                 `json:"collision_check_state"`
	CollisionPoints                *[]CollisionPoint      `json:"collision_points"`
	SingularityAnalysisAvailable   *bool                  `json:"singularity_analysis_available"`
	SingularityGateClear           *bool                  `json:"singularity_gate_clear"`
	SingularityCheckState          string                 `json:"singularity_check_state"`
	SingularityPoints              *[]rawSingularityPoint `json:"singularity_points"`
	LimitCheckState                string                 `json:"limit_check_state"`
	UnreachablePoints              *[]UnreachablePoint    `json:"unreachable_points"`
	PlannedSampleCount             int                    `json:"planned_sample_count"`
	PlanningTimeS                  float64                `json:"planning_time_s"`
	CollisionCheckTimeS            float64                `json:"collision_check_time_s"`
	CollisionWorkerWallTimeS       float64                `json:"collision_worker_wall_time_s"`
	CollisionCacheHitWaypointCount int                    `json:"collision_cache_hit_waypoint_count"`
}

type rawSingularityPoint struct {
	RowIndex                  int       `json:"row_index"`
	WaypointIndex             int       `json:"waypoint_index"`
	State                     string    `json:"state"`
	SingularityClass          string    `json:"singularity_class"`
	AppliedVelocityScale      *float64  `json:"applied_velocity_scale"`
	ProportionalVelocityScale *float64  `json:"proportional_velocity_scale"`
	LimitingJoint             string    `json:"limiting_joint"`
	SingularDirectionJoint    string    `json:"singular_direction_joint"`
	AffectedJoints            []string  `json:"affected_joints"`
	PositionsRad              []float64 `json:"positions_rad"`
}

// NormalizeMotionServerReport converts a canonical motion-server report into
// explicit evidence. Missing fields remain unavailable; no default value is
// interpreted as a successful safety check.
func NormalizeMotionServerReport(context ReportContext, raw []byte) (NormalizedPlanEvidence, error) {
	if err := context.Request.Validate(); err != nil {
		return NormalizedPlanEvidence{}, fmt.Errorf("planning: invalid report context: %w", err)
	}
	if strings.TrimSpace(context.PlanID) == "" || context.PlanRevision == 0 || context.ReceivedAt.IsZero() {
		return NormalizedPlanEvidence{}, errors.New("planning: plan identity and receive time are required")
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	var report motionServerReport
	if err := decoder.Decode(&report); err != nil {
		return NormalizedPlanEvidence{}, fmt.Errorf("planning: decode motion-server report: %w", err)
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return NormalizedPlanEvidence{}, errors.New("planning: multiple JSON values in motion-server report")
	}

	programRevision := report.AcceptedProgramRevision
	if programRevision == 0 {
		programRevision = report.ProgramRevision
	}
	evidence := NormalizedPlanEvidence{
		Schema:          NormalizedPlanEvidenceSchema,
		RequestID:       context.Request.RequestID,
		GeneratedAt:     context.ReceivedAt.UTC(),
		PlannerOwner:    strings.TrimSpace(report.PlannerOwner),
		PlannerBackend:  strings.TrimSpace(report.PlannerBackend),
		State:           PlanUnvalidated,
		Accepted:        report.Accepted != nil && *report.Accepted,
		ValidationState: strings.TrimSpace(report.ValidationState),
		SampleCount:     report.PlannedSampleCount,
		Identity: PlanIdentity{
			ProjectID:       context.Request.ProjectID,
			ProjectRevision: context.Request.ProjectRevision,
			ProgramID:       strings.TrimSpace(report.ProgramID),
			ProgramRevision: programRevision,
			PlanID:          strings.TrimSpace(context.PlanID),
			PlanRevision:    context.PlanRevision,
			ProgramHash:     strings.TrimSpace(report.ProgramHash),
			RobotModel:      report.RobotModel,
			Target:          context.Request.Target,
		},
		Collision:   normalizeCollisionEvidence(report),
		Singularity: normalizeSingularityEvidence(report),
		Limits:      normalizeLimitEvidence(report),
		Runtime:     normalizeRuntimeEvidence(report),
		Timing: PlanTimingEvidence{
			PlanningTimeS:                  nonnegativeTiming(report.PlanningTimeS),
			CollisionCheckTimeS:            nonnegativeTiming(report.CollisionCheckTimeS),
			CollisionWorkerWallTime:        nonnegativeTiming(report.CollisionWorkerWallTimeS),
			CollisionCacheHitWaypointCount: report.CollisionCacheHitWaypointCount,
		},
	}
	evidence.CorrelationReasons = correlateReport(context.Request, report, evidence.Identity)

	if len(evidence.CorrelationReasons) > 0 ||
		(report.Accepted != nil && !*report.Accepted) ||
		evidence.ValidationState == "invalid" {
		evidence.State = PlanRejected
		return evidence, nil
	}
	if evidence.Accepted &&
		evidence.ValidationState == "valid" &&
		evidence.PlannerOwner == "motion-server" &&
		evidence.Collision.State == CheckClear &&
		evidence.Singularity.State == CheckClear &&
		evidence.Limits.State == CheckClear &&
		evidence.Runtime.State == CheckClear &&
		evidence.Runtime.Playable {
		evidence.State = PlanServerValidated
	}
	return evidence, nil
}

func nonnegativeTiming(value float64) float64 {
	if value < 0 {
		return 0
	}
	return value
}

func normalizeCollisionEvidence(report motionServerReport) CollisionEvidence {
	out := CollisionEvidence{
		State:      CheckUnavailable,
		CheckState: strings.TrimSpace(report.CollisionCheckState),
		Points:     []CollisionPoint{},
	}
	if report.CollisionPoints == nil || out.CheckState == "" || report.CollisionGateClear == nil {
		return out
	}
	for _, point := range *report.CollisionPoints {
		point.AffectedJoints = append([]string{}, point.AffectedJoints...)
		point.PositionsRad = append([]float64{}, point.PositionsRad...)
		out.Points = append(out.Points, point)
	}
	if *report.CollisionGateClear &&
		out.CheckState == "tesseract_discrete_contact_checked" &&
		len(out.Points) == 0 {
		out.State = CheckClear
	} else {
		out.State = CheckBlocked
	}
	return out
}

func normalizeSingularityEvidence(report motionServerReport) SingularityEvidence {
	out := SingularityEvidence{
		State:      CheckUnavailable,
		CheckState: strings.TrimSpace(report.SingularityCheckState),
		Points:     []SingularityPoint{},
	}
	if report.SingularityPoints == nil ||
		report.SingularityAnalysisAvailable == nil ||
		report.SingularityGateClear == nil ||
		out.CheckState == "" {
		return out
	}
	for _, point := range *report.SingularityPoints {
		state := strings.TrimSpace(point.State)
		if state == "" {
			state = strings.TrimSpace(point.SingularityClass)
		}
		scale := firstFloat(point.AppliedVelocityScale, point.ProportionalVelocityScale)
		limitingJoint := strings.TrimSpace(point.LimitingJoint)
		if limitingJoint == "" || strings.EqualFold(limitingJoint, "none") {
			limitingJoint = strings.TrimSpace(point.SingularDirectionJoint)
		}
		out.Points = append(out.Points, SingularityPoint{
			RowIndex:             point.RowIndex,
			WaypointIndex:        point.WaypointIndex,
			State:                state,
			AppliedVelocityScale: scale,
			LimitingJoint:        limitingJoint,
			AffectedJoints:       append([]string{}, point.AffectedJoints...),
			PositionsRad:         append([]float64{}, point.PositionsRad...),
		})
	}
	if *report.SingularityAnalysisAvailable &&
		*report.SingularityGateClear &&
		out.CheckState == "jacobian_svd_checked" &&
		!hasHardStop(out.Points) {
		out.State = CheckClear
	} else {
		out.State = CheckBlocked
	}
	return out
}

func normalizeLimitEvidence(report motionServerReport) LimitEvidence {
	out := LimitEvidence{
		State:      CheckUnavailable,
		CheckState: strings.TrimSpace(report.LimitCheckState),
		Points:     []UnreachablePoint{},
	}
	if report.UnreachablePoints == nil || out.CheckState == "" {
		return out
	}
	for _, point := range *report.UnreachablePoints {
		point.PositionsRad = append([]float64{}, point.PositionsRad...)
		out.Points = append(out.Points, point)
	}
	if out.CheckState == "clear" && len(out.Points) == 0 {
		out.State = CheckClear
	} else {
		out.State = CheckBlocked
	}
	return out
}

func normalizeRuntimeEvidence(report motionServerReport) RuntimeEvidence {
	out := RuntimeEvidence{State: CheckUnavailable}
	if report.PlayEnabled == nil || report.RuntimePreloadEnabled == nil {
		return out
	}
	out.Playable = *report.PlayEnabled && *report.RuntimePreloadEnabled
	if out.Playable {
		out.State = CheckClear
	} else {
		out.State = CheckBlocked
	}
	return out
}

func correlateReport(request NormalizedPlanRequest, report motionServerReport, identity PlanIdentity) []string {
	reasons := make([]string, 0, 6)
	if identity.ProgramID != request.ProgramID {
		reasons = append(reasons, "program_id_mismatch")
	}
	if identity.ProgramRevision != request.ProgramRevision {
		reasons = append(reasons, "program_revision_mismatch")
	}
	if identity.RobotModel != request.RobotModel {
		reasons = append(reasons, "robot_model_mismatch")
	}
	if identity.ProgramHash == "" {
		reasons = append(reasons, "program_hash_missing")
	}
	if strings.TrimSpace(report.PlannerOwner) != "motion-server" {
		reasons = append(reasons, "planner_owner_mismatch")
	}
	if report.ProgramRevision != 0 &&
		report.AcceptedProgramRevision != 0 &&
		report.ProgramRevision != report.AcceptedProgramRevision {
		reasons = append(reasons, "accepted_program_revision_mismatch")
	}
	return reasons
}

func firstFloat(values ...*float64) float64 {
	for _, value := range values {
		if value != nil {
			return *value
		}
	}
	return 0
}

func hasHardStop(points []SingularityPoint) bool {
	for _, point := range points {
		if point.State == "hard_stop" || point.State == "unavailable" {
			return true
		}
	}
	return false
}
