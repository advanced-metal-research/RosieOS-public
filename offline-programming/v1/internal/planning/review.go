package planning

import (
	"strings"
	"time"
)

const (
	ModeOfflinePreview = "offline_preview"
	ModeLocalSimulator = "local_simulator"
	ModeRealConnected  = "real_connected"
)

type StartPoseState string

const (
	StartPoseUnavailable StartPoseState = "unavailable"
	StartPoseConverged   StartPoseState = "converged"
	StartPoseMismatch    StartPoseState = "mismatch"
)

type RuntimeOutputState struct {
	Known               bool `json:"known"`
	EtherCATEnabled     bool `json:"ethercat_enabled"`
	MotionOutputEnabled bool `json:"motion_output_enabled"`
	ServosArmed         bool `json:"servos_armed"`
	EmergencyStopClear  bool `json:"emergency_stop_clear"`
	FaultFree           bool `json:"fault_free"`
	Stationary          bool `json:"stationary"`
	WeldOutputEnabled   bool `json:"weld_output_enabled"`
}

type RuntimePlanStatus struct {
	Connected       bool               `json:"connected"`
	StatusSequence  uint64             `json:"status_sequence"`
	ObservedAt      time.Time          `json:"observed_at"`
	FreshUntil      time.Time          `json:"fresh_until"`
	Target          TargetIdentity     `json:"target"`
	RobotModel      RobotModel         `json:"robot_model"`
	Plan            PlanIdentity       `json:"plan"`
	StartPose       StartPoseState     `json:"start_pose"`
	RuntimePlayable bool               `json:"runtime_playable"`
	Output          RuntimeOutputState `json:"output"`
}

type AuthorityProof struct {
	ControllerID      string    `json:"controller_id"`
	ControllerBootID  string    `json:"controller_boot_id"`
	TargetProfileID   string    `json:"target_profile_id"`
	LeaseID           string    `json:"lease_id"`
	FenceEpoch        uint64    `json:"fence_epoch"`
	StatusSequence    uint64    `json:"status_sequence"`
	FreshUntil        time.Time `json:"fresh_until"`
	TransitionPending bool      `json:"transition_pending"`
}

type ExecutionReviewInput struct {
	Now             time.Time              `json:"now"`
	Mode            string                 `json:"mode"`
	ProjectID       string                 `json:"project_id"`
	ProjectRevision uint64                 `json:"project_revision"`
	RobotModel      RobotModel             `json:"robot_model"`
	SelectedTarget  TargetIdentity         `json:"selected_target"`
	Evidence        NormalizedPlanEvidence `json:"evidence"`
	Runtime         RuntimePlanStatus      `json:"runtime"`
	Authority       AuthorityProof         `json:"authority"`
}

type ReviewCheck struct {
	Code   string `json:"code"`
	Passed bool   `json:"passed"`
	Detail string `json:"detail"`
}

type ExecutionReview struct {
	Schema             string         `json:"schema"`
	PrerequisitesReady bool           `json:"prerequisites_ready"`
	CanSubmit          bool           `json:"can_submit"`
	CanExecute         bool           `json:"can_execute"`
	Reasons            []string       `json:"reasons"`
	Checks             []ReviewCheck  `json:"checks"`
	Plan               PlanIdentity   `json:"plan"`
	Target             TargetIdentity `json:"target"`
	RobotModel         RobotModel     `json:"robot_model"`
}

// ReviewExecution is deliberately exhaustive and fail closed. Every check is
// positive evidence from one review input; absence is a failed check.
func ReviewExecution(input ExecutionReviewInput) ExecutionReview {
	review := ExecutionReview{
		Schema:     "offline-programming.execution-review.v1",
		Reasons:    []string{},
		Checks:     []ReviewCheck{},
		Plan:       input.Evidence.Identity,
		Target:     input.SelectedTarget,
		RobotModel: input.RobotModel,
	}
	add := func(code string, passed bool, detail string) {
		review.Checks = append(review.Checks, ReviewCheck{Code: code, Passed: passed, Detail: detail})
		if !passed {
			review.Reasons = append(review.Reasons, code)
		}
	}

	add("real_mode_required", input.Mode == ModeRealConnected, input.Mode)
	add("current_project_identity_required",
		strings.TrimSpace(input.ProjectID) != "" && input.ProjectRevision > 0,
		input.ProjectID)
	add("manifest_backed_target_required",
		strings.TrimSpace(input.SelectedTarget.ProfileID) != "" &&
			strings.TrimSpace(input.SelectedTarget.ManifestRef) != "",
		input.SelectedTarget.ProfileID)
	add("supported_robot_model_required", supportedRobotModel(input.RobotModel), string(input.RobotModel))
	add("server_validated_plan_required", input.Evidence.ServerValidated(), string(input.Evidence.State))
	add("plan_project_must_be_current",
		input.Evidence.Identity.ProjectID == input.ProjectID &&
			input.Evidence.Identity.ProjectRevision == input.ProjectRevision,
		input.Evidence.Identity.ProjectID)
	add("plan_target_must_match",
		input.Evidence.Identity.Target == input.SelectedTarget,
		input.Evidence.Identity.Target.ProfileID)
	add("plan_model_must_match",
		input.Evidence.Identity.RobotModel == input.RobotModel,
		string(input.Evidence.Identity.RobotModel))
	add("collision_evidence_clear", input.Evidence.Collision.State == CheckClear, string(input.Evidence.Collision.State))
	add("singularity_evidence_clear", input.Evidence.Singularity.State == CheckClear, string(input.Evidence.Singularity.State))
	add("limit_evidence_clear", input.Evidence.Limits.State == CheckClear, string(input.Evidence.Limits.State))
	add("runtime_playable_evidence_required",
		input.Evidence.Runtime.State == CheckClear && input.Evidence.Runtime.Playable,
		string(input.Evidence.Runtime.State))

	runtimeFresh := input.Runtime.Connected &&
		input.Runtime.StatusSequence > 0 &&
		!input.Runtime.ObservedAt.IsZero() &&
		!input.Runtime.FreshUntil.IsZero() &&
		!input.Now.IsZero() &&
		!input.Now.Before(input.Runtime.ObservedAt) &&
		input.Now.Before(input.Runtime.FreshUntil)
	add("fresh_runtime_status_required", runtimeFresh, input.Runtime.FreshUntil.UTC().Format(time.RFC3339Nano))
	add("runtime_target_must_match", input.Runtime.Target == input.SelectedTarget, input.Runtime.Target.ProfileID)
	add("runtime_model_must_match", input.Runtime.RobotModel == input.RobotModel, string(input.Runtime.RobotModel))
	add("runtime_plan_must_match", input.Runtime.Plan.Equal(input.Evidence.Identity), input.Runtime.Plan.PlanID)
	add("start_pose_must_converge", input.Runtime.StartPose == StartPoseConverged, string(input.Runtime.StartPose))
	add("runtime_must_be_playable", input.Runtime.RuntimePlayable, "")

	authorityFresh := strings.TrimSpace(input.Authority.ControllerID) != "" &&
		strings.TrimSpace(input.Authority.ControllerBootID) != "" &&
		input.Authority.TargetProfileID == input.SelectedTarget.ProfileID &&
		strings.TrimSpace(input.Authority.LeaseID) != "" &&
		input.Authority.FenceEpoch > 0 &&
		input.Authority.StatusSequence > 0 &&
		!input.Authority.TransitionPending &&
		!input.Now.IsZero() &&
		input.Now.Before(input.Authority.FreshUntil)
	add("fresh_server_authority_required", authorityFresh, input.Authority.ControllerID)

	output := input.Runtime.Output
	add("runtime_output_state_required", output.Known, "")
	add("ethercat_enabled_required", output.Known && output.EtherCATEnabled, "")
	add("motion_output_enabled_required", output.Known && output.MotionOutputEnabled, "")
	add("servos_armed_required", output.Known && output.ServosArmed, "")
	add("emergency_stop_clear_required", output.Known && output.EmergencyStopClear, "")
	add("fault_free_required", output.Known && output.FaultFree, "")
	add("stationary_start_required", output.Known && output.Stationary, "")
	add("weld_output_off_at_start", output.Known && !output.WeldOutputEnabled, "")

	review.PrerequisitesReady = len(review.Reasons) == 0
	review.CanSubmit = false
	review.CanExecute = false
	review.Reasons = append(review.Reasons, "real_execution_adapter_not_installed")
	return review
}
