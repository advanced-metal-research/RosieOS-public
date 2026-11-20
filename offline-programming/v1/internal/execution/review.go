package execution

import (
	"fmt"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/connected"
	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

func capabilityReport(now time.Time, config GatewayConfig) GatewayCapabilityReport {
	report := GatewayCapabilityReport{
		Schema:              CapabilitySchema,
		EvaluatedAt:         now.UTC(),
		Controller:          config.Bootstrap.Controller,
		Reasons:             []string{},
		Arm:                 false,
		WeldOn:              false,
		Jog:                 false,
		Teleop:              false,
		RawBrowserTransport: false,
	}
	add := func(passed bool, reason string) {
		if !passed {
			report.Reasons = appendUnique(report.Reasons, reason)
		}
	}

	report.GatewayEnabled = config.Features.GatewayEnabled
	report.ConnectedExecutionEnabled = config.Features.ConnectedExecutionEnabled
	report.ServerControllerAuthority = config.Features.ServerControllerAuthority
	add(report.GatewayEnabled, "gateway_feature_disabled")
	add(report.ConnectedExecutionEnabled, "connected_execution_feature_disabled")
	add(report.ServerControllerAuthority, "robot_v4_server_controller_authority_not_enabled")

	bootstrapValid := config.Bootstrap.Validate() == nil &&
		!now.IsZero() &&
		!now.Before(config.Bootstrap.BootstrappedAt)
	add(bootstrapValid, "controller_bootstrap_invalid")
	report.ManifestAllowlistInjected = config.Allowlist != nil
	report.AuthoritativeEvidenceInjected = config.Evidence != nil
	report.SessionObserverInjected = config.Observer != nil
	report.SessionMonitoring = config.Features.SessionMonitoringEnabled &&
		report.SessionObserverInjected
	report.ServerTransportInjected = config.Transport != nil
	report.LeaseAuthorityInjected = config.Authority != nil
	report.FenceValidationInjected = config.Authority != nil
	report.AuditSinkInjected = config.Audit != nil && config.Features.AuditRequired
	report.IdempotencyStoreInjected = config.Idempotency != nil
	add(report.ManifestAllowlistInjected, "manifest_allowlist_not_injected")
	add(report.AuthoritativeEvidenceInjected, "authoritative_evidence_verifier_not_injected")
	if config.Features.ConnectedExecutionEnabled &&
		!config.Features.AllowDeterministicTestTransport {
		add(report.SessionMonitoring, "active_session_monitoring_unavailable")
	}
	add(report.ServerTransportInjected, "server_transport_not_injected")
	add(report.LeaseAuthorityInjected, "lease_authority_not_injected")
	add(report.FenceValidationInjected, "fence_validator_not_injected")
	add(report.AuditSinkInjected, "required_audit_sink_not_injected")
	add(report.IdempotencyStoreInjected, "idempotency_store_not_injected")

	var transport ServerTransportCapabilities
	if config.Transport != nil {
		transport = config.Transport.Capabilities()
	}
	report.ServerOwnedTransport = transport.Available &&
		transport.ServerOwned &&
		transport.CommandSchema == RobotV4CommandSchema &&
		transport.LeaderProtocol == RobotV4LeaderProtocol &&
		!transport.RawEndpointExposed &&
		(!transport.TestOnly || config.Features.AllowDeterministicTestTransport)
	add(report.ServerOwnedTransport, "authoritative_server_transport_unavailable")
	report.ReviewTokens = bootstrapValid &&
		report.GatewayEnabled &&
		report.AuditSinkInjected
	report.PlanPreload = config.Features.PlanPreloadEnabled &&
		report.ServerOwnedTransport &&
		transport.PlanPreload
	report.ControlledStartArm = config.Features.ControlledStartArmEnabled &&
		report.ServerOwnedTransport &&
		transport.ControlledStartArm
	report.PlanPlay = config.Features.PlanPlayEnabled &&
		report.ServerOwnedTransport &&
		transport.PlanPlay
	report.ProtectiveStop = config.Features.ProtectiveTailEnabled &&
		report.ServerOwnedTransport &&
		transport.ProtectiveStop
	report.ProtectiveDisarm = config.Features.ProtectiveTailEnabled &&
		report.ServerOwnedTransport &&
		transport.ProtectiveDisarm
	report.ProtectiveWeldOff = config.Features.ProtectiveTailEnabled &&
		report.ServerOwnedTransport &&
		transport.ProtectiveWeldOff
	add(report.ReviewTokens, "review_token_issuer_unavailable")
	add(report.PlanPreload, "ordered_plan_preload_unavailable")
	add(report.ControlledStartArm, "controlled_start_arm_unavailable")
	add(report.PlanPlay, "ordered_plan_play_unavailable")
	add(report.ProtectiveStop, "protective_stop_unavailable")
	add(report.ProtectiveDisarm, "protective_disarm_unavailable")
	add(report.ProtectiveWeldOff, "protective_weld_off_unavailable")
	report.InfrastructureReady = len(report.Reasons) == 0
	return report
}

func reviewExecution(
	now time.Time,
	config GatewayConfig,
	input ExecutionReviewInput,
) ExecutionReview {
	capabilities := capabilityReport(now, config)
	expected := connected.ExpectedIdentity{
		Target:          input.Identity.Target.ConnectedIdentity(),
		PlanID:          input.Identity.PlanID,
		PlanRevision:    input.Identity.PlanRevision,
		ProgramID:       input.Identity.ProgramID,
		ProgramRevision: input.Identity.ProgramRevision,
		ProgramHash:     input.Identity.ProgramHash,
		RobotModel:      string(input.Identity.RobotModel),
	}
	correlation := connected.Correlate(now, expected, input.Status, input.RuntimePlan)
	review := ExecutionReview{
		Schema:       ReviewSchema,
		ReviewedAt:   now.UTC(),
		Checks:       []ReviewCheck{},
		Reasons:      []string{},
		Identity:     input.Identity,
		Correlation:  correlation,
		Capabilities: capabilities,
	}
	add := func(code string, passed bool, detail string) {
		review.Checks = append(review.Checks, ReviewCheck{
			Code:   code,
			Passed: passed,
			Detail: detail,
		})
		if !passed {
			review.Reasons = appendUnique(review.Reasons, code)
		}
	}

	add("gateway_infrastructure_ready", capabilities.InfrastructureReady,
		strings.Join(capabilities.Reasons, ","))
	add("execution_identity_complete", input.Identity.Complete(), input.Identity.PlanID)

	discoveryTarget := input.Discovery.Target
	discoveryExact := input.Discovery.Schema == targets.ConnectedTargetDiscoverySchema &&
		discoveryTarget.Kind == targets.TargetRealCell &&
		input.Discovery.ProfileID == input.Identity.Target.ProfileID &&
		input.Discovery.ManifestRef == input.Identity.Target.ManifestRef &&
		input.Discovery.CellID == input.Identity.Target.CellID &&
		discoveryTarget.ProfileID == input.Identity.Target.ProfileID &&
		discoveryTarget.ManifestRef == input.Identity.Target.ManifestRef &&
		discoveryTarget.CellID == input.Identity.Target.CellID
	add("exact_manifest_target_required", discoveryExact, input.Discovery.ProfileID)
	add("manifest_revision_allowlisted",
		config.Allowlist != nil && config.Allowlist.Allows(ManifestAllowance{
			Target:     input.Identity.Target,
			RobotModel: input.Identity.RobotModel,
		}),
		input.Identity.Target.ManifestRevision)
	add("server_mediated_motion_service_required",
		hasReadOnlyMotionService(input.Discovery.Services),
		input.Identity.Target.ProfileID)

	requestValid := input.PlanRequest.Validate() == nil
	add("normalized_plan_request_valid", requestValid, input.PlanRequest.RequestID)
	add("bounded_plan_request_identity",
		safeASCII(input.PlanRequest.RequestID, 128) &&
			safeASCII(input.Evidence.RequestID, 128),
		input.PlanRequest.RequestID)
	add("plan_request_identity_exact",
		requestMatchesIdentity(input.PlanRequest, input.Identity),
		input.PlanRequest.RequestID)
	add("raw_physical_io_actions_forbidden",
		noRawPhysicalIO(input.PlanRequest),
		input.PlanRequest.ProgramID)
	acceptedPlanValid := input.AcceptedPlan.Validate() == nil &&
		input.AcceptedPlan.AuthorityClassification ==
			planning.AcceptedPlanPhysicalTarget &&
		input.AcceptedPlan.Evidence.RuntimePlayable
	add("physical_target_accepted_plan_required",
		acceptedPlanValid,
		input.AcceptedPlan.PlanID)
	add("accepted_plan_identity_exact",
		acceptedPlanMatchesIdentity(input.AcceptedPlan, input.Identity),
		input.AcceptedPlan.TrajectoryDigest)
	add("accepted_plan_request_correlation",
		input.AcceptedPlan.RequestID != "" &&
			input.AcceptedPlan.RequestID == input.PlanRequest.RequestID,
		input.AcceptedPlan.RequestID)
	add("server_validated_plan_evidence_required",
		input.Evidence.ServerValidated() &&
			input.Evidence.PlannerOwner == "motion-server" &&
			strings.TrimSpace(input.Evidence.PlannerBackend) != "",
		string(input.Evidence.State))
	add("plan_evidence_identity_exact",
		evidenceMatchesIdentity(input.Evidence, input.Identity),
		input.Evidence.Identity.PlanID)
	add("plan_request_evidence_correlation",
		input.PlanRequest.RequestID != "" &&
			input.PlanRequest.RequestID == input.Evidence.RequestID,
		input.Evidence.RequestID)
	evidenceFresh := !now.IsZero() &&
		!input.Evidence.GeneratedAt.IsZero() &&
		!now.Before(input.Evidence.GeneratedAt) &&
		now.Sub(input.Evidence.GeneratedAt) <= config.Bootstrap.MaxPlanEvidenceAge &&
		!input.PlanRequest.SubmittedAt.IsZero() &&
		!input.Evidence.GeneratedAt.Before(input.PlanRequest.SubmittedAt)
	add("fresh_plan_evidence_required", evidenceFresh,
		input.Evidence.GeneratedAt.UTC().Format(time.RFC3339Nano))

	statusFresh := freshStatus(now, input.Status, config.Bootstrap.StatusFreshnessCeiling)
	planFresh := freshPlan(now, input.RuntimePlan, config.Bootstrap.StatusFreshnessCeiling)
	add("fresh_authoritative_status_required", statusFresh,
		input.Status.FreshUntil.UTC().Format(time.RFC3339Nano))
	add("fresh_authoritative_runtime_plan_required", planFresh,
		input.RuntimePlan.FreshUntil.UTC().Format(time.RFC3339Nano))
	add("exact_status_plan_target_correlation", correlation.Correlated,
		strings.Join(correlation.Reasons, ","))
	add("authoritative_target_ready_for_preload",
		authoritativeTargetReadyForPreload(input.Status, input.RuntimePlan),
		input.Status.Plan.PlanSessionID)
	add("safe_runtime_output_state",
		safeRuntimeOutput(input.Status),
		input.Status.ServiceState)

	review.Ready = len(review.Reasons) == 0
	return review
}

func requestMatchesIdentity(request planning.NormalizedPlanRequest, identity ExecutionIdentity) bool {
	return request.Schema == planning.NormalizedPlanRequestSchema &&
		request.ProjectID == identity.ProjectID &&
		request.ProjectRevision == identity.ProjectRevision &&
		request.ProgramID == identity.ProgramID &&
		request.ProgramRevision == identity.ProgramRevision &&
		request.RobotModel == identity.RobotModel &&
		request.Target.ProfileID == identity.Target.ProfileID &&
		request.Target.ManifestRef == identity.Target.ManifestRef
}

func acceptedPlanMatchesIdentity(
	plan planning.RobotV4AcceptedPlan,
	identity ExecutionIdentity,
) bool {
	return plan.PlanID == identity.PlanID &&
		plan.PlanRevision == identity.PlanRevision &&
		plan.TrajectoryDigest == identity.TrajectoryDigest &&
		plan.ProgramID == identity.ProgramID &&
		plan.ProgramRevision == identity.ProgramRevision &&
		plan.ProgramDigest == identity.ProgramDigest &&
		plan.Target.TargetID == identity.Target.ProfileID &&
		plan.Target.RobotModel == identity.RobotModel &&
		plan.Target.ManifestRevision == identity.Target.ManifestRevision
}

func evidenceMatchesIdentity(evidence planning.NormalizedPlanEvidence, identity ExecutionIdentity) bool {
	plan := evidence.Identity
	return plan.ProjectID == identity.ProjectID &&
		plan.ProjectRevision == identity.ProjectRevision &&
		plan.ProgramID == identity.ProgramID &&
		plan.ProgramRevision == identity.ProgramRevision &&
		plan.PlanID == identity.PlanID &&
		plan.PlanRevision == identity.PlanRevision &&
		plan.ProgramHash == identity.ProgramHash &&
		plan.RobotModel == identity.RobotModel &&
		plan.Target.ProfileID == identity.Target.ProfileID &&
		plan.Target.ManifestRef == identity.Target.ManifestRef
}

func freshStatus(now time.Time, status connected.StatusObservation, ceiling time.Duration) bool {
	return status.Schema == connected.StatusFrameSchema &&
		status.Connected &&
		status.IdentityCorrelated &&
		status.Sequence > 0 &&
		!status.ReceivedAt.IsZero() &&
		!status.SourceUpdatedAt.IsZero() &&
		!status.FreshUntil.IsZero() &&
		!now.IsZero() &&
		!now.Before(status.ReceivedAt) &&
		!now.Before(status.SourceUpdatedAt) &&
		now.Before(status.FreshUntil) &&
		status.FreshUntil.Sub(status.ReceivedAt) > 0 &&
		status.FreshUntil.Sub(status.ReceivedAt) <= ceiling
}

func freshPlan(now time.Time, plan connected.PlanObservation, ceiling time.Duration) bool {
	return plan.Schema == connected.PlanFrameSchema &&
		plan.IdentityCorrelated &&
		plan.Sequence > 0 &&
		!plan.ReceivedAt.IsZero() &&
		!plan.FreshUntil.IsZero() &&
		!now.IsZero() &&
		!now.Before(plan.ReceivedAt) &&
		now.Before(plan.FreshUntil) &&
		plan.FreshUntil.Sub(plan.ReceivedAt) > 0 &&
		plan.FreshUntil.Sub(plan.ReceivedAt) <= ceiling
}

func authoritativeTargetReadyForPreload(
	status connected.StatusObservation,
	plan connected.PlanObservation,
) bool {
	return status.ServiceState == "running" &&
		status.Plan.PlaybackActive.Known && !status.Plan.PlaybackActive.Value &&
		plan.PlannerOwner == "motion-server" &&
		strings.TrimSpace(plan.PlannerBackend) != "" &&
		plan.ValidationState == "valid" &&
		plan.Accepted.Known && plan.Accepted.Value &&
		plan.PlayEnabled.Known && plan.PlayEnabled.Value &&
		plan.RuntimePreload.Known && plan.RuntimePreload.Value &&
		plan.SampleCount >= 2 &&
		plan.Safety.Collision.Available && plan.Safety.Collision.Clear &&
		plan.Safety.Singularity.Available && plan.Safety.Singularity.Clear &&
		plan.Safety.Limits.Available && plan.Safety.Limits.Clear &&
		plan.Safety.Runtime.Available && plan.Safety.Runtime.Clear
}

func noRawPhysicalIO(request planning.NormalizedPlanRequest) bool {
	for _, row := range request.Rows {
		if row.RevPiIOSlot != nil || strings.TrimSpace(row.RevPiIOAction) != "" {
			return false
		}
	}
	return true
}

func safeRuntimeOutput(status connected.StatusObservation) bool {
	output := status.Output
	return output.CurrentZeroHold.Known && output.CurrentZeroHold.Value &&
		output.StaleZeroHold.Known && !output.StaleZeroHold.Value &&
		output.ArmRequested.Known && !output.ArmRequested.Value &&
		output.ActualServoArmed.Known && !output.ActualServoArmed.Value &&
		output.EtherCATEnabled.Known && output.EtherCATEnabled.Value &&
		output.MotionOutput.Known && output.MotionOutput.Value &&
		output.WeldOutput.Known && !output.WeldOutput.Value
}

func hasReadOnlyMotionService(services []targets.ConnectedTargetServiceDescriptor) bool {
	for _, service := range services {
		if service.Role == MotionServerService &&
			service.StatusStream &&
			service.PlanStream &&
			service.ReadOnly &&
			service.ServerMediated {
			return true
		}
	}
	return false
}

func appendUnique(values []string, value string) []string {
	for _, existing := range values {
		if existing == value {
			return values
		}
	}
	return append(values, value)
}

func reviewError(review ExecutionReview) error {
	if review.Ready {
		return nil
	}
	return fmt.Errorf("%w: %s", ErrReviewDenied, strings.Join(review.Reasons, ","))
}
