package connected

import (
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"
	"time"
)

func Correlate(
	now time.Time,
	expected ExpectedIdentity,
	status StatusObservation,
	plan PlanObservation,
) IdentityCorrelation {
	reasons := make([]string, 0, 20)
	check := func(passed bool, reason string) {
		if !passed {
			reasons = appendReason(reasons, reason)
		}
	}

	check(!now.IsZero(), "correlation_time_missing")
	check(expected.Target.ProfileID != "" && expected.Target.ManifestRef != "",
		"expected_target_identity_incomplete")
	check(status.Schema == StatusFrameSchema && status.Connected &&
		status.IdentityCorrelated && now.Before(status.FreshUntil),
		"status_unavailable_or_stale")
	check(plan.Schema == PlanFrameSchema && plan.IdentityCorrelated &&
		now.Before(plan.FreshUntil),
		"plan_unavailable_or_stale")
	check(status.Target == expected.Target, "status_target_mismatch")
	check(plan.Target == expected.Target, "plan_target_mismatch")
	check(status.Target == plan.Target, "status_plan_target_mismatch")
	check(plan.Identity.Complete(), "plan_identity_incomplete")
	check(plan.Identity.PlanID == expected.PlanID, "plan_id_mismatch")
	check(plan.Identity.PlanRevision == expected.PlanRevision, "plan_revision_mismatch")
	check(plan.Identity.ProgramID == expected.ProgramID, "program_id_mismatch")
	check(plan.Identity.ProgramRevision == expected.ProgramRevision, "program_revision_mismatch")
	check(plan.Identity.ProgramHash == expected.ProgramHash, "program_hash_mismatch")
	check(plan.Identity.RobotModel == expected.RobotModel, "robot_model_mismatch")
	// An accepted physical plan is intentionally not installed during review.
	// Exact runtime identity is proved after the separately fenced preload.
	// Requiring a preloaded plan here would collapse review and execution into
	// one unsafe step and would make preload confirmation meaningless.
	check(status.ServiceState == "running" ||
		status.ServiceState == "active" ||
		status.ServiceState == "ready",
		"motion_server_service_not_ready")
	check(status.Plan.PlaybackActive.Known &&
		!status.Plan.PlaybackActive.Value,
		"runtime_playback_must_be_inactive")
	check(plan.Accepted.Known && plan.Accepted.Value, "explicit_plan_acceptance_required")
	check(plan.Safety.Collision.Available && plan.Safety.Collision.Clear,
		"collision_evidence_not_clear")
	check(plan.Safety.Singularity.Available && plan.Safety.Singularity.Clear,
		"singularity_evidence_not_clear")
	check(plan.Safety.Limits.Available && plan.Safety.Limits.Clear,
		"limit_evidence_not_clear")
	check(plan.Safety.Runtime.Available && plan.Safety.Runtime.Clear,
		"runtime_evidence_not_clear")

	if strings.TrimSpace(expected.PlanID) == "" ||
		expected.PlanRevision == 0 ||
		strings.TrimSpace(expected.ProgramID) == "" ||
		expected.ProgramRevision == 0 ||
		strings.TrimSpace(expected.ProgramHash) == "" ||
		!robotmodel.IsArm(expected.RobotModel) {
		reasons = appendReason(reasons, "expected_plan_identity_incomplete")
	}
	return IdentityCorrelation{
		Schema:     CorrelationSchema,
		CheckedAt:  now.UTC(),
		Expected:   expected,
		Status:     cloneStatus(status),
		Plan:       clonePlan(plan),
		Correlated: len(reasons) == 0,
		Reasons:    reasons,
	}
}
