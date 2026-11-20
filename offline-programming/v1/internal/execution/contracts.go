// Package execution implements the isolated, server-owned execution gate for
// Offline Programming V1. It defines no NATS, UDP, WebSocket, HTTP, daemon, or
// physical endpoint implementation.
package execution

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/connected"
	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

const (
	CapabilitySchema       = "offline-programming.execution-gateway-capabilities.v1"
	ReviewSchema           = "offline-programming.execution-gateway-review.v1"
	ReviewTokenSchema      = "offline-programming.execution-review-token.v1"
	TransportCommandSchema = "offline-programming.execution-transport-command.v1"
	TransportReceiptSchema = "offline-programming.execution-transport-receipt.v1"
	AuditSchema            = "offline-programming.execution-gateway-audit.v1"
	SessionSchema          = "offline-programming.execution-session.v1"
	RobotV4CommandSchema   = "robot.v4.robot-command.v1"
	RobotV4LeaderProtocol  = "robot.v4.leader-lease.v1"
	MotionServerService    = "motion-server"
)

var (
	ErrUnavailable       = errors.New("execution: gateway is unavailable")
	ErrReviewDenied      = errors.New("execution: execution review denied")
	ErrReviewToken       = errors.New("execution: review token is invalid, expired, stale, or consumed")
	ErrTransportReceipt  = errors.New("execution: transport receipt did not prove exact application")
	ErrSessionNotFound   = errors.New("execution: execution session not found")
	ErrIdempotencyReplay = errors.New("execution: command idempotency key was already committed")
)

type Clock interface {
	Now() time.Time
}

type systemClock struct{}

func (systemClock) Now() time.Time { return time.Now().UTC() }

type ControllerBootstrap struct {
	Controller             targets.ControllerIdentity `json:"controller"`
	CommandIDPrefix        string                     `json:"command_id_prefix"`
	BootstrappedAt         time.Time                  `json:"bootstrapped_at"`
	ReviewTTL              time.Duration              `json:"review_ttl"`
	LeaseTTL               time.Duration              `json:"lease_ttl"`
	MaxPlanEvidenceAge     time.Duration              `json:"max_plan_evidence_age"`
	StatusFreshnessCeiling time.Duration              `json:"status_freshness_ceiling"`
}

func (bootstrap ControllerBootstrap) Validate() error {
	if !bootstrap.Controller.Valid() ||
		!safeASCII(bootstrap.Controller.ControllerID, 63) ||
		!safeASCII(bootstrap.Controller.ControllerBootID, 63) {
		return errors.New("execution: concrete controller and per-process boot identities are required")
	}
	if !safeASCII(bootstrap.CommandIDPrefix, 24) {
		return errors.New("execution: bounded printable command id prefix is required")
	}
	if bootstrap.BootstrappedAt.IsZero() {
		return errors.New("execution: bootstrap time is required")
	}
	if bootstrap.ReviewTTL < time.Second || bootstrap.ReviewTTL > 30*time.Second {
		return errors.New("execution: review ttl must be between one and thirty seconds")
	}
	if bootstrap.LeaseTTL < 500*time.Millisecond || bootstrap.LeaseTTL > 2*time.Second {
		return errors.New("execution: Robot V4 lease ttl must be between 500ms and 2s")
	}
	if bootstrap.MaxPlanEvidenceAge < time.Second ||
		bootstrap.MaxPlanEvidenceAge > 24*time.Hour {
		return errors.New("execution: plan evidence age bound is invalid")
	}
	if bootstrap.StatusFreshnessCeiling < 100*time.Millisecond ||
		bootstrap.StatusFreshnessCeiling > connected.MaxObservationFreshness {
		return errors.New("execution: status freshness ceiling is invalid")
	}
	return nil
}

type FeatureGates struct {
	GatewayEnabled                  bool `json:"gateway_enabled"`
	ConnectedExecutionEnabled       bool `json:"connected_execution_enabled"`
	ServerControllerAuthority       bool `json:"server_controller_authority"`
	SessionMonitoringEnabled        bool `json:"session_monitoring_enabled"`
	PlanPreloadEnabled              bool `json:"plan_preload_enabled"`
	ControlledStartArmEnabled       bool `json:"controlled_start_arm_enabled"`
	PlanPlayEnabled                 bool `json:"plan_play_enabled"`
	ProtectiveTailEnabled           bool `json:"protective_tail_enabled"`
	AuditRequired                   bool `json:"audit_required"`
	AllowDeterministicTestTransport bool `json:"allow_deterministic_test_transport"`
}

type TargetRevisionIdentity struct {
	ProfileID        string `json:"profile_id"`
	ManifestRef      string `json:"manifest_ref"`
	CellID           string `json:"cell_id"`
	ManifestRevision string `json:"manifest_revision"`
}

func (target TargetRevisionIdentity) ConnectedIdentity() connected.TargetIdentity {
	return connected.TargetIdentity{
		ProfileID:   target.ProfileID,
		ManifestRef: target.ManifestRef,
		CellID:      target.CellID,
	}
}

func (target TargetRevisionIdentity) Complete() bool {
	return safeASCII(target.ProfileID, 128) &&
		safeASCII(target.ManifestRef, 512) &&
		safeASCII(target.CellID, 128) &&
		connected.ValidManifestRevision(target.ManifestRevision)
}

type ExecutionIdentity struct {
	ProjectID        string                 `json:"project_id"`
	ProjectRevision  uint64                 `json:"project_revision"`
	ProgramID        string                 `json:"program_id"`
	ProgramRevision  uint64                 `json:"program_revision"`
	PlanID           string                 `json:"plan_id"`
	PlanRevision     uint64                 `json:"plan_revision"`
	ProgramHash      string                 `json:"program_hash"`
	ProgramDigest    string                 `json:"program_digest"`
	TrajectoryDigest string                 `json:"trajectory_digest"`
	RobotModel       planning.RobotModel    `json:"robot_model"`
	Target           TargetRevisionIdentity `json:"target"`
}

func (identity ExecutionIdentity) Complete() bool {
	return safeASCII(identity.ProjectID, 128) &&
		identity.ProjectRevision > 0 &&
		safeASCII(identity.ProgramID, 128) &&
		identity.ProgramRevision > 0 &&
		safeASCII(identity.PlanID, 128) &&
		identity.PlanRevision > 0 &&
		safeASCII(identity.ProgramHash, 256) &&
		safeSHA256Digest(identity.ProgramDigest) &&
		safeSHA256Digest(identity.TrajectoryDigest) &&
		robotmodel.IsArm(string(identity.RobotModel)) &&
		identity.Target.Complete()
}

func (identity ExecutionIdentity) Equal(other ExecutionIdentity) bool {
	return identity == other
}

type ManifestAllowance struct {
	Target     TargetRevisionIdentity `json:"target"`
	RobotModel planning.RobotModel    `json:"robot_model"`
}

type ManifestAllowlist interface {
	Allows(ManifestAllowance) bool
}

type StaticManifestAllowlist struct {
	entries map[ManifestAllowance]struct{}
}

func NewStaticManifestAllowlist(entries []ManifestAllowance) (*StaticManifestAllowlist, error) {
	if len(entries) == 0 {
		return nil, errors.New("execution: manifest allowlist must not be empty")
	}
	allowlist := &StaticManifestAllowlist{entries: make(map[ManifestAllowance]struct{}, len(entries))}
	for _, entry := range entries {
		if !entry.Target.Complete() ||
			!robotmodel.IsArm(string(entry.RobotModel)) {
			return nil, errors.New("execution: manifest allowance is incomplete")
		}
		allowlist.entries[entry] = struct{}{}
	}
	return allowlist, nil
}

func (allowlist *StaticManifestAllowlist) Allows(entry ManifestAllowance) bool {
	if allowlist == nil {
		return false
	}
	_, ok := allowlist.entries[entry]
	return ok
}

type ServerTransportCapabilities struct {
	Available          bool   `json:"available"`
	ServerOwned        bool   `json:"server_owned"`
	TestOnly           bool   `json:"test_only"`
	CommandSchema      string `json:"command_schema"`
	LeaderProtocol     string `json:"leader_protocol"`
	PlanPreload        bool   `json:"plan_preload"`
	ControlledStartArm bool   `json:"controlled_start_arm"`
	PlanPlay           bool   `json:"plan_play"`
	ProtectiveStop     bool   `json:"protective_stop"`
	ProtectiveDisarm   bool   `json:"protective_disarm"`
	ProtectiveWeldOff  bool   `json:"protective_weld_off"`
	RawEndpointExposed bool   `json:"raw_endpoint_exposed"`
}

type ServerTransport interface {
	Capabilities() ServerTransportCapabilities
	Preload(context.Context, TransportCommand) (TransportReceipt, error)
	Arm(context.Context, TransportCommand) (TransportReceipt, error)
	Play(context.Context, TransportCommand) (TransportReceipt, error)
	Protect(context.Context, TransportCommand) (TransportReceipt, error)
}

type LeaseCancelRequest struct {
	TargetProfileID    string                     `json:"target_profile_id"`
	Controller         targets.ControllerIdentity `json:"controller"`
	RequestID          string                     `json:"request_id"`
	CampaignGeneration uint64                     `json:"campaign_generation"`
}

// LeaseAuthority models the existing Robot V4 acquire/renew/release/cancel
// campaign and a separate application-time fence validation. An observation
// of a lease is never treated as a validation.
type LeaseAuthority interface {
	Acquire(context.Context, targets.LeaseAcquireRequest) (targets.LeaseStatus, error)
	Renew(context.Context, targets.LeaseRenewRequest) (targets.LeaseStatus, error)
	Release(context.Context, targets.LeaseReleaseRequest) error
	Cancel(context.Context, LeaseCancelRequest) error
	ValidateFence(context.Context, targets.FenceValidationRequest) error
}

type GatewayAuditSink interface {
	Append(context.Context, GatewayAuditEnvelope) error
}

// AuthoritativeEvidenceVerifier compares a review input with the server's own
// connected-observation cache. Browser-posted status or plan JSON is never
// authoritative merely because it deserializes into ExecutionReviewInput.
type AuthoritativeEvidenceVerifier interface {
	Verify(context.Context, ExecutionReviewInput) error
}

// ActiveSessionObserver independently verifies the server-owned Motion Server
// and RTCore read-back for a running execution. An accepted command reply is
// never treated as live proof.
type ActiveSessionObserver interface {
	VerifyActive(context.Context, ExecutionSession, time.Time) error
}

type GatewayConfig struct {
	Bootstrap   ControllerBootstrap
	Features    FeatureGates
	Allowlist   ManifestAllowlist
	Evidence    AuthoritativeEvidenceVerifier
	Observer    ActiveSessionObserver
	Transport   ServerTransport
	Authority   LeaseAuthority
	Audit       GatewayAuditSink
	Idempotency targets.IdempotencyStore
	Clock       Clock
}

type GatewayCapabilityReport struct {
	Schema                        string                     `json:"schema"`
	EvaluatedAt                   time.Time                  `json:"evaluated_at"`
	Controller                    targets.ControllerIdentity `json:"controller"`
	GatewayEnabled                bool                       `json:"gateway_enabled"`
	ConnectedExecutionEnabled     bool                       `json:"connected_execution_enabled"`
	ServerControllerAuthority     bool                       `json:"server_controller_authority"`
	ManifestAllowlistInjected     bool                       `json:"manifest_allowlist_injected"`
	AuthoritativeEvidenceInjected bool                       `json:"authoritative_evidence_injected"`
	SessionObserverInjected       bool                       `json:"session_observer_injected"`
	SessionMonitoring             bool                       `json:"session_monitoring"`
	ServerTransportInjected       bool                       `json:"server_transport_injected"`
	ServerOwnedTransport          bool                       `json:"server_owned_transport"`
	LeaseAuthorityInjected        bool                       `json:"lease_authority_injected"`
	FenceValidationInjected       bool                       `json:"fence_validation_injected"`
	AuditSinkInjected             bool                       `json:"audit_sink_injected"`
	IdempotencyStoreInjected      bool                       `json:"idempotency_store_injected"`
	ReviewTokens                  bool                       `json:"review_tokens"`
	PlanPreload                   bool                       `json:"plan_preload"`
	ControlledStartArm            bool                       `json:"controlled_start_arm"`
	PlanPlay                      bool                       `json:"plan_play"`
	ProtectiveStop                bool                       `json:"protective_stop"`
	ProtectiveDisarm              bool                       `json:"protective_disarm"`
	ProtectiveWeldOff             bool                       `json:"protective_weld_off"`
	Arm                           bool                       `json:"arm"`
	WeldOn                        bool                       `json:"weld_on"`
	Jog                           bool                       `json:"jog"`
	Teleop                        bool                       `json:"teleop"`
	RawBrowserTransport           bool                       `json:"raw_browser_transport"`
	InfrastructureReady           bool                       `json:"infrastructure_ready"`
	Reasons                       []string                   `json:"reasons"`
}

type ExecutionReviewInput struct {
	Identity     ExecutionIdentity                          `json:"identity"`
	Discovery    targets.ConnectedTargetDiscoveryDescriptor `json:"discovery"`
	PlanRequest  planning.NormalizedPlanRequest             `json:"plan_request"`
	AcceptedPlan planning.RobotV4AcceptedPlan               `json:"accepted_plan"`
	Evidence     planning.NormalizedPlanEvidence            `json:"evidence"`
	Status       connected.StatusObservation                `json:"status"`
	RuntimePlan  connected.PlanObservation                  `json:"runtime_plan"`
}

type ReviewCheck struct {
	Code   string `json:"code"`
	Passed bool   `json:"passed"`
	Detail string `json:"detail"`
}

type ReviewToken struct {
	Schema              string                     `json:"schema"`
	TokenID             string                     `json:"token_id"`
	IssuedAt            time.Time                  `json:"issued_at"`
	ExpiresAt           time.Time                  `json:"expires_at"`
	Controller          targets.ControllerIdentity `json:"controller"`
	Identity            ExecutionIdentity          `json:"identity"`
	StatusSequence      uint64                     `json:"status_sequence"`
	RuntimePlanSequence uint64                     `json:"runtime_plan_sequence"`
	EvidenceRequestID   string                     `json:"evidence_request_id"`
}

type ExecutionReview struct {
	Schema       string                        `json:"schema"`
	ReviewedAt   time.Time                     `json:"reviewed_at"`
	Ready        bool                          `json:"ready"`
	Checks       []ReviewCheck                 `json:"checks"`
	Reasons      []string                      `json:"reasons"`
	Identity     ExecutionIdentity             `json:"identity"`
	Correlation  connected.IdentityCorrelation `json:"correlation"`
	Capabilities GatewayCapabilityReport       `json:"capabilities"`
	Token        *ReviewToken                  `json:"token,omitempty"`
}

type TransportAction string

const (
	ActionPreload TransportAction = "preload"
	ActionArm     TransportAction = "arm_for_reviewed_start"
	ActionPlay    TransportAction = "play"
	ActionStop    TransportAction = "stop"
	ActionDisarm  TransportAction = "disarm"
	ActionWeldOff TransportAction = "weld_off"
)

type CommandIdentity struct {
	CommandID        string    `json:"command_id"`
	IdempotencyKey   string    `json:"idempotency_key"`
	IntentSequence   uint64    `json:"intent_sequence"`
	ControllerBootID string    `json:"controller_boot_id"`
	CreatedAt        time.Time `json:"created_at"`
}

type TransportCommand struct {
	Schema             string                                 `json:"schema"`
	Action             TransportAction                        `json:"action"`
	SessionID          string                                 `json:"session_id"`
	ReviewTokenID      string                                 `json:"review_token_id"`
	Controller         targets.ControllerIdentity             `json:"controller"`
	Identity           ExecutionIdentity                      `json:"identity"`
	Command            CommandIdentity                        `json:"command"`
	CampaignGeneration uint64                                 `json:"campaign_generation"`
	Credential         *targets.LeaseCredential               `json:"credential,omitempty"`
	AcceptedPlan       *planning.RobotV4AcceptedPlanReference `json:"accepted_plan,omitempty"`
	Reason             string                                 `json:"reason"`
}

func (command TransportCommand) Validate(now time.Time) error {
	if command.Schema != TransportCommandSchema || !supportedAction(command.Action) {
		return errors.New("execution: unsupported transport command")
	}
	if !safeASCII(command.SessionID, 128) ||
		!safeASCII(command.ReviewTokenID, 128) ||
		!command.Controller.Valid() ||
		!command.Identity.Complete() ||
		command.CampaignGeneration == 0 {
		return errors.New("execution: transport command binding is incomplete")
	}
	if !safeASCII(command.Command.CommandID, 63) ||
		command.Command.IntentSequence == 0 ||
		command.Command.ControllerBootID != command.Controller.ControllerBootID ||
		command.Command.CreatedAt.IsZero() ||
		now.Before(command.Command.CreatedAt) {
		return errors.New("execution: command identity is invalid")
	}
	expected := targets.CommandIdempotencyKey(
		command.Identity.Target.ProfileID,
		MotionServerService,
		command.Command.CommandID,
	)
	if command.Command.IdempotencyKey != expected {
		return fmt.Errorf("execution: idempotency key must be %q", expected)
	}
	if command.Action == ActionPreload {
		if command.AcceptedPlan == nil ||
			command.AcceptedPlan.Validate() != nil {
			return errors.New("execution: preload requires the exact accepted plan reference")
		}
	} else if command.AcceptedPlan != nil {
		return errors.New("execution: only preload may contain an accepted plan reference")
	}
	if command.Action == ActionPreload ||
		command.Action == ActionArm ||
		command.Action == ActionPlay {
		if command.Credential == nil ||
			command.Credential.ControllerBootID != command.Controller.ControllerBootID ||
			command.Credential.LeaseID == "" ||
			command.Credential.FenceEpoch == 0 {
			return errors.New("execution: energizing command requires an exact lease credential")
		}
	}
	return nil
}

type TransportReceipt struct {
	Schema         string            `json:"schema"`
	Action         TransportAction   `json:"action"`
	CommandID      string            `json:"command_id"`
	IdempotencyKey string            `json:"idempotency_key"`
	Accepted       bool              `json:"accepted"`
	Applied        bool              `json:"applied"`
	Duplicate      bool              `json:"duplicate"`
	ServerOwned    bool              `json:"server_owned"`
	RecordedAt     time.Time         `json:"recorded_at"`
	Identity       ExecutionIdentity `json:"identity"`
	Result         string            `json:"result"`
}

type StartRequest struct {
	RequestID   string               `json:"request_id"`
	ReviewToken string               `json:"review_token"`
	Review      ExecutionReviewInput `json:"review"`
}

type ExecutionSession struct {
	Schema             string                     `json:"schema"`
	SessionID          string                     `json:"session_id"`
	StartedAt          time.Time                  `json:"started_at"`
	Identity           ExecutionIdentity          `json:"identity"`
	Controller         targets.ControllerIdentity `json:"controller"`
	ReviewTokenID      string                     `json:"review_token_id"`
	CampaignGeneration uint64                     `json:"campaign_generation"`
	Lease              targets.LeaseStatus        `json:"lease"`
	PreloadReceipt     TransportReceipt           `json:"preload_receipt"`
	ArmReceipt         TransportReceipt           `json:"arm_receipt"`
	PlayReceipt        TransportReceipt           `json:"play_receipt"`
}

type StopRequest struct {
	RequestID string `json:"request_id"`
	SessionID string `json:"session_id"`
	Reason    string `json:"reason"`
}

type GatewayAuditEnvelope struct {
	Schema        string                     `json:"schema"`
	EventID       string                     `json:"event_id"`
	RecordedAt    time.Time                  `json:"recorded_at"`
	Phase         string                     `json:"phase"`
	Action        string                     `json:"action"`
	Outcome       string                     `json:"outcome"`
	TransportUsed bool                       `json:"transport_used"`
	SessionID     string                     `json:"session_id,omitempty"`
	ReviewTokenID string                     `json:"review_token_id,omitempty"`
	Controller    targets.ControllerIdentity `json:"controller"`
	Identity      ExecutionIdentity          `json:"identity"`
	Command       *CommandIdentity           `json:"command,omitempty"`
	Lease         *targets.LeaseStatus       `json:"lease,omitempty"`
	Reasons       []string                   `json:"reasons"`
}

func supportedAction(action TransportAction) bool {
	switch action {
	case ActionPreload, ActionArm, ActionPlay, ActionStop, ActionDisarm, ActionWeldOff:
		return true
	default:
		return false
	}
}

func safeASCII(value string, maximum int) bool {
	if value == "" || len(value) > maximum {
		return false
	}
	for _, char := range []byte(value) {
		if char < 0x21 || char > 0x7e {
			return false
		}
	}
	return true
}

func safeSHA256Digest(value string) bool {
	const prefix = "sha256:"
	if !strings.HasPrefix(value, prefix) || len(value) != len(prefix)+64 {
		return false
	}
	_, err := hex.DecodeString(strings.TrimPrefix(value, prefix))
	return err == nil
}
