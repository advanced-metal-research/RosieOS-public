// Package connected defines the isolated, read-only Robot V4 observation
// boundary for Offline Programming V1. It contains no network dialer,
// command publisher, or physical execution implementation.
package connected

import (
	"context"
	"errors"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"time"

	"rosieos/offline-programming/v1/internal/targets"
)

const (
	StatusFrameSchema       = "offline-programming.connected-status.v1"
	PlanFrameSchema         = "offline-programming.connected-plan.v1"
	CorrelationSchema       = "offline-programming.connected-correlation.v1"
	CapabilityReportSchema  = "offline-programming.connected-capabilities.v1"
	ExecutionAuditSchema    = "offline-programming.execution-audit.v1"
	MaxStatusFrameBytes     = 256 * 1024
	MaxPlanFrameBytes       = 4 * 1024 * 1024
	MaxObservationFreshness = 30 * time.Second
	MaxCorrelationReasons   = 32
	MaxAuditReasons         = 32
	MaxAuditReasonLength    = 160
)

var (
	ErrFrameTooLarge      = errors.New("connected: observation frame exceeds bounded size")
	ErrStaleFrameSequence = errors.New("connected: observation sequence is stale or repeated")
	ErrAuditSinkDisabled  = errors.New("connected: execution audit sink is disabled")
)

type TargetIdentity struct {
	ProfileID   string `json:"profile_id"`
	ManifestRef string `json:"manifest_ref"`
	CellID      string `json:"cell_id"`
}

func targetIdentity(discovery targets.ConnectedTargetDiscoveryDescriptor) TargetIdentity {
	return TargetIdentity{
		ProfileID:   discovery.ProfileID,
		ManifestRef: discovery.ManifestRef,
		CellID:      discovery.CellID,
	}
}

type BooleanFact struct {
	Known bool `json:"known"`
	Value bool `json:"value"`
}

type ObservationContext struct {
	Target          targets.ConnectedManifestTarget
	SourcePeer      string
	SourceSubject   string
	Sequence        uint64
	ReceivedAt      time.Time
	FreshnessWindow time.Duration
}

type RuntimePlanFacts struct {
	Preloaded                BooleanFact `json:"preloaded"`
	LoadedProgramAvailable   BooleanFact `json:"loaded_program_available"`
	LoadedProgramPlayable    BooleanFact `json:"loaded_program_playable"`
	ServerValidated          BooleanFact `json:"server_validated"`
	PlayAuthorized           BooleanFact `json:"play_authorized"`
	PlaybackActive           BooleanFact `json:"playback_active"`
	LoadedProgramRevision    uint64      `json:"loaded_program_revision"`
	PlanRevision             uint64      `json:"plan_revision"`
	PlanSessionID            string      `json:"plan_session_id"`
	AcceptedPlanID           string      `json:"accepted_plan_id"`
	AcceptedProgramDigest    string      `json:"accepted_program_digest"`
	AcceptedTrajectoryDigest string      `json:"accepted_trajectory_digest"`
	AcceptedManifestRevision string      `json:"accepted_manifest_revision"`
}

type RuntimeOutputFacts struct {
	ArmRequested     BooleanFact `json:"arm_requested"`
	CurrentZeroHold  BooleanFact `json:"current_zero_hold"`
	StaleZeroHold    BooleanFact `json:"stale_zero_hold"`
	ActualServoArmed BooleanFact `json:"actual_servo_armed"`
	EtherCATEnabled  BooleanFact `json:"ethercat_enabled"`
	MotionOutput     BooleanFact `json:"motion_output_enabled"`
	WeldOutput       BooleanFact `json:"weld_output_enabled"`
}

type RTCoreOutputObservation struct {
	Schema          string             `json:"schema"`
	Target          TargetIdentity     `json:"target"`
	SourcePeer      string             `json:"source_peer"`
	SourceSubject   string             `json:"source_subject"`
	Sequence        uint64             `json:"sequence"`
	ReceivedAt      time.Time          `json:"received_at"`
	SourceUpdatedAt time.Time          `json:"source_updated_at"`
	FreshUntil      time.Time          `json:"fresh_until"`
	ServiceName     string             `json:"service_name"`
	ServiceState    string             `json:"service_state"`
	Output          RuntimeOutputFacts `json:"output"`
	Reasons         []string           `json:"reasons"`
}

type AuthorityObservation struct {
	Known              bool          `json:"known"`
	ControllerID       string        `json:"controller_id"`
	ControllerBootID   string        `json:"controller_boot_id"`
	LeaseID            string        `json:"lease_id"`
	FenceEpoch         uint64        `json:"fence_epoch"`
	CampaignGeneration uint64        `json:"campaign_generation"`
	LeaseFresh         bool          `json:"lease_fresh"`
	TransitionPending  bool          `json:"transition_pending"`
	ExpiresIn          time.Duration `json:"expires_in"`
	FreshUntil         time.Time     `json:"fresh_until"`
}

type StatusObservation struct {
	Schema             string                   `json:"schema"`
	Target             TargetIdentity           `json:"target"`
	SourcePeer         string                   `json:"source_peer"`
	SourceSubject      string                   `json:"source_subject"`
	Sequence           uint64                   `json:"sequence"`
	ReceivedAt         time.Time                `json:"received_at"`
	SourceUpdatedAt    time.Time                `json:"source_updated_at"`
	FreshUntil         time.Time                `json:"fresh_until"`
	Connected          bool                     `json:"connected"`
	IdentityCorrelated bool                     `json:"identity_correlated"`
	ServiceName        string                   `json:"service_name"`
	ServiceState       string                   `json:"service_state"`
	Plan               RuntimePlanFacts         `json:"plan"`
	Output             RuntimeOutputFacts       `json:"output"`
	RTCoreOutput       *RTCoreOutputObservation `json:"rtcore_output,omitempty"`
	Authority          AuthorityObservation     `json:"authority"`
	CorrelationReasons []string                 `json:"correlation_reasons"`
}

type PlanIdentity struct {
	PlanID          string `json:"plan_id"`
	PlanRevision    uint64 `json:"plan_revision"`
	ProgramID       string `json:"program_id"`
	ProgramRevision uint64 `json:"program_revision"`
	ProgramHash     string `json:"program_hash"`
	RobotModel      string `json:"robot_model"`
}

func (identity PlanIdentity) Complete() bool {
	return identity.PlanID != "" &&
		identity.PlanRevision > 0 &&
		identity.ProgramID != "" &&
		identity.ProgramRevision > 0 &&
		identity.ProgramHash != "" &&
		robotmodel.IsArm(identity.RobotModel)
}

type SafetyEvidence struct {
	Available  bool   `json:"available"`
	Clear      bool   `json:"clear"`
	State      string `json:"state"`
	PointCount int    `json:"point_count"`
}

type PlanSafetyFacts struct {
	Collision   SafetyEvidence `json:"collision"`
	Singularity SafetyEvidence `json:"singularity"`
	Limits      SafetyEvidence `json:"limits"`
	Runtime     SafetyEvidence `json:"runtime"`
}

type PlanObservation struct {
	Schema             string          `json:"schema"`
	Target             TargetIdentity  `json:"target"`
	SourcePeer         string          `json:"source_peer"`
	SourceSubject      string          `json:"source_subject"`
	Sequence           uint64          `json:"sequence"`
	ReceivedAt         time.Time       `json:"received_at"`
	FreshUntil         time.Time       `json:"fresh_until"`
	Identity           PlanIdentity    `json:"identity"`
	PlannerOwner       string          `json:"planner_owner"`
	PlannerBackend     string          `json:"planner_backend"`
	ValidationState    string          `json:"validation_state"`
	Accepted           BooleanFact     `json:"accepted"`
	PlayEnabled        BooleanFact     `json:"play_enabled"`
	RuntimePreload     BooleanFact     `json:"runtime_preload_enabled"`
	SampleCount        int             `json:"sample_count"`
	Safety             PlanSafetyFacts `json:"safety"`
	IdentityCorrelated bool            `json:"identity_correlated"`
	CorrelationReasons []string        `json:"correlation_reasons"`
}

type ExpectedIdentity struct {
	Target          TargetIdentity `json:"target"`
	PlanID          string         `json:"plan_id"`
	PlanRevision    uint64         `json:"plan_revision"`
	ProgramID       string         `json:"program_id"`
	ProgramRevision uint64         `json:"program_revision"`
	ProgramHash     string         `json:"program_hash"`
	RobotModel      string         `json:"robot_model"`
}

type IdentityCorrelation struct {
	Schema     string            `json:"schema"`
	CheckedAt  time.Time         `json:"checked_at"`
	Expected   ExpectedIdentity  `json:"expected"`
	Status     StatusObservation `json:"status"`
	Plan       PlanObservation   `json:"plan"`
	Correlated bool              `json:"correlated"`
	Reasons    []string          `json:"reasons"`
}

type AuditSink interface {
	Append(context.Context, ExecutionAuditEnvelope) error
}

type DisabledAuditSink struct{}

func (DisabledAuditSink) Append(context.Context, ExecutionAuditEnvelope) error {
	return ErrAuditSinkDisabled
}
