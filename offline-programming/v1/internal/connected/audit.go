package connected

import (
	"errors"
	"fmt"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/targets"
)

type ExecutionAuditDecision struct {
	Action            string   `json:"action"`
	SubmissionAllowed bool     `json:"submission_allowed"`
	TransportUsed     bool     `json:"transport_used"`
	Reasons           []string `json:"reasons"`
}

type ExecutionAuditEnvelope struct {
	Schema       string                     `json:"schema"`
	EventID      string                     `json:"event_id"`
	RecordedAt   time.Time                  `json:"recorded_at"`
	Target       TargetIdentity             `json:"target"`
	Plan         PlanIdentity               `json:"plan"`
	Controller   targets.ControllerIdentity `json:"controller"`
	Authority    AuthorityObservation       `json:"authority"`
	Correlation  IdentityCorrelation        `json:"correlation"`
	Capabilities CapabilityReport           `json:"capabilities"`
	Decision     ExecutionAuditDecision     `json:"decision"`
}

type ExecutionAuditInput struct {
	EventID      string
	RecordedAt   time.Time
	Action       string
	Target       TargetIdentity
	Plan         PlanIdentity
	Controller   targets.ControllerIdentity
	Authority    AuthorityObservation
	Correlation  IdentityCorrelation
	Capabilities CapabilityReport
	Reasons      []string
}

// NewExecutionAuditEnvelope creates an immutable denied-at-boundary audit
// record. V1 cannot create an allowed submission envelope because no command
// transport exists in this package.
func NewExecutionAuditEnvelope(input ExecutionAuditInput) (ExecutionAuditEnvelope, error) {
	if !boundedID(input.EventID, 128) || input.RecordedAt.IsZero() {
		return ExecutionAuditEnvelope{}, errors.New("connected: audit event identity and time are required")
	}
	if !supportedAuditAction(input.Action) {
		return ExecutionAuditEnvelope{}, fmt.Errorf("connected: unsupported audit action %q", input.Action)
	}
	if input.Target.ProfileID == "" || input.Target.ManifestRef == "" {
		return ExecutionAuditEnvelope{}, errors.New("connected: audit target identity is incomplete")
	}
	reasons := make([]string, 0, MaxAuditReasons)
	for _, reason := range input.Reasons {
		reason = strings.TrimSpace(reason)
		if reason == "" || len(reason) > MaxAuditReasonLength {
			return ExecutionAuditEnvelope{}, errors.New("connected: audit reason is empty or exceeds bound")
		}
		if len(reasons) == MaxAuditReasons {
			return ExecutionAuditEnvelope{}, errors.New("connected: audit reason count exceeds bound")
		}
		reasons = append(reasons, reason)
	}
	reasons = appendReason(reasons, "offline_programming_v1_physical_execution_disabled")
	return ExecutionAuditEnvelope{
		Schema:       ExecutionAuditSchema,
		EventID:      input.EventID,
		RecordedAt:   input.RecordedAt.UTC(),
		Target:       input.Target,
		Plan:         input.Plan,
		Controller:   input.Controller,
		Authority:    input.Authority,
		Correlation:  input.Correlation,
		Capabilities: input.Capabilities,
		Decision: ExecutionAuditDecision{
			Action:            input.Action,
			SubmissionAllowed: false,
			TransportUsed:     false,
			Reasons:           reasons,
		},
	}, nil
}

func supportedAuditAction(action string) bool {
	switch action {
	case "review", "upload_plan", "play", "arm", "weld_on", "jog", "teleop",
		"stop", "disarm", "weld_off":
		return true
	default:
		return false
	}
}

func boundedID(value string, maximum int) bool {
	value = strings.TrimSpace(value)
	return value != "" && len(value) <= maximum
}
