package connected

import (
	"time"

	"rosieos/offline-programming/v1/internal/targets"
)

type CapabilityReport struct {
	Schema                  string         `json:"schema"`
	Target                  TargetIdentity `json:"target"`
	ReportedAt              time.Time      `json:"reported_at"`
	ManifestBackedDiscovery bool           `json:"manifest_backed_discovery"`
	ReadOnlyStatus          bool           `json:"read_only_status"`
	ReadOnlyPlan            bool           `json:"read_only_plan"`
	IdentityCorrelated      bool           `json:"identity_correlated"`
	AuthorityObserved       bool           `json:"authority_observed"`
	AuthenticatedAuthority  bool           `json:"authenticated_authority"`
	CommandSubmission       bool           `json:"command_submission"`
	Arm                     bool           `json:"arm"`
	WeldOn                  bool           `json:"weld_on"`
	Play                    bool           `json:"play"`
	Jog                     bool           `json:"jog"`
	Teleop                  bool           `json:"teleop"`
	ProtectiveStop          bool           `json:"protective_stop"`
	ProtectiveDisarm        bool           `json:"protective_disarm"`
	ProtectiveWeldOff       bool           `json:"protective_weld_off"`
	RawBrowserTransport     bool           `json:"raw_browser_transport"`
	PhysicalExecution       bool           `json:"physical_execution"`
	Reasons                 []string       `json:"reasons"`
}

// ReportCapabilities describes only infrastructure proved by current
// observations. Physical and protective action capabilities remain false even
// when all read-only inputs correlate; future authenticated gateways must be a
// separate integration.
func ReportCapabilities(
	now time.Time,
	discovery *targets.ConnectedTargetDiscoveryDescriptor,
	status *StatusObservation,
	plan *PlanObservation,
	correlation *IdentityCorrelation,
) CapabilityReport {
	report := CapabilityReport{
		Schema:     CapabilityReportSchema,
		ReportedAt: now.UTC(),
		Reasons:    []string{},
	}
	if discovery != nil {
		report.Target = targetIdentity(*discovery)
		report.ManifestBackedDiscovery = discovery.Schema == targets.ConnectedTargetDiscoverySchema &&
			discovery.ProfileID != "" &&
			discovery.ManifestRef != ""
	}
	report.ReadOnlyStatus = status != nil &&
		status.Schema == StatusFrameSchema &&
		status.Connected &&
		!now.IsZero() &&
		now.Before(status.FreshUntil)
	report.ReadOnlyPlan = plan != nil &&
		plan.Schema == PlanFrameSchema &&
		!now.IsZero() &&
		now.Before(plan.FreshUntil)
	report.IdentityCorrelated = correlation != nil &&
		correlation.Schema == CorrelationSchema &&
		correlation.Correlated
	report.AuthorityObserved = report.ReadOnlyStatus &&
		status.Authority.Known &&
		status.Authority.LeaseFresh &&
		now.Before(status.Authority.FreshUntil)

	if !report.ManifestBackedDiscovery {
		report.Reasons = appendReason(report.Reasons, "manifest_backed_discovery_unavailable")
	}
	if !report.ReadOnlyStatus {
		report.Reasons = appendReason(report.Reasons, "fresh_read_only_status_unavailable")
	}
	if !report.ReadOnlyPlan {
		report.Reasons = appendReason(report.Reasons, "fresh_read_only_plan_unavailable")
	}
	if !report.IdentityCorrelated {
		report.Reasons = appendReason(report.Reasons, "status_plan_identity_not_correlated")
	}
	if discovery == nil || !discovery.RobotModelBound {
		report.Reasons = appendReason(report.Reasons, "robot_model_not_manifest_bound")
	}
	report.Reasons = appendReason(report.Reasons, "authenticated_authority_gateway_not_installed")
	report.Reasons = appendReason(report.Reasons, "command_gateway_not_installed")
	report.Reasons = appendReason(report.Reasons, "protective_action_gateway_not_installed")
	report.Reasons = appendReason(report.Reasons, "offline_programming_v1_physical_execution_disabled")
	return report
}
