// Package simulator defines the isolated, PC-native local planning and
// simulation lifecycle for Offline Programming V1.
package simulator

import (
	"errors"
	"fmt"
	"net"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

const (
	ServiceStatusSchema = "offline-programming.local-simulator-status.v1"
	SafetyProofSchema   = "offline-programming.local-simulator-safety-proof.v1"
	AvailabilitySchema  = "offline-programming.local-simulator-availability.v1"

	PlatformPCNative       = "pc_native"
	PlanTransportInProcess = planning.MotionServerPlanTransport

	HTTPStatusPath = "/api/offline-programming/v1/local-simulator/status"
	HTTPStartPath  = "/api/offline-programming/v1/local-simulator/start"
	HTTPStopPath   = "/api/offline-programming/v1/local-simulator/stop"
	HTTPPlansPath  = "/api/offline-programming/v1/local-simulator/plans"
)

type LifecyclePhase string

const (
	PhaseStopped  LifecyclePhase = "stopped"
	PhaseStarting LifecyclePhase = "starting"
	PhaseReady    LifecyclePhase = "ready"
	PhaseStopping LifecyclePhase = "stopping"
	PhaseFailed   LifecyclePhase = "failed"
)

type AvailabilityCode string

const (
	AvailabilityReady                 AvailabilityCode = "available"
	AvailabilityExplicitOptInRequired AvailabilityCode = "explicit_opt_in_required"
	AvailabilityProcessContainment    AvailabilityCode = "process_containment_unavailable"
	AvailabilityCanonicalBinary       AvailabilityCode = "canonical_binary_unavailable"
	AvailabilityCollisionRuntime      AvailabilityCode = "collision_runtime_unavailable"
	AvailabilityCanonicalAssets       AvailabilityCode = "canonical_assets_unavailable"
	AvailabilityProbeFailed           AvailabilityCode = "canonical_probe_failed"
)

// BackendAvailability is a typed, public-safe answer to whether the canonical
// local planner can be started. Paths and process arguments remain server-side.
type BackendAvailability struct {
	Schema            string           `json:"schema"`
	Available         bool             `json:"available"`
	Code              AvailabilityCode `json:"code"`
	Reason            string           `json:"reason"`
	CanonicalPlanner  bool             `json:"canonical_planner"`
	ProcessBounded    bool             `json:"process_bounded"`
	PersistentProcess bool             `json:"persistent_process"`
	ExplicitOptIn     bool             `json:"explicit_opt_in"`
}

// SafetyProof must be supplied by the running backend, not inferred from a
// selected UI mode. Safe proves the complete local invariant positively.
type SafetyProof struct {
	Schema                   string    `json:"schema"`
	ProfileID                string    `json:"profile_id"`
	RuntimeBootID            string    `json:"runtime_boot_id"`
	StatusSequence           uint64    `json:"status_sequence"`
	ObservedAt               time.Time `json:"observed_at"`
	FreshUntil               time.Time `json:"fresh_until"`
	Simulation               bool      `json:"simulation"`
	NoRealMovement           bool      `json:"no_real_movement"`
	LoopbackOnly             bool      `json:"loopback_only"`
	EtherCATOutputEnabled    bool      `json:"ethercat_output_enabled"`
	MotionOutputEnabled      bool      `json:"motion_output_enabled"`
	HardwareAccess           bool      `json:"hardware_access"`
	PhysicalServicesStarted  bool      `json:"physical_services_started"`
	CommandSubmissionEnabled bool      `json:"command_submission_enabled"`
}

func (proof SafetyProof) Validate(now time.Time, profileID string) error {
	switch {
	case proof.Schema != SafetyProofSchema:
		return fmt.Errorf("simulator: safety proof schema must be %q", SafetyProofSchema)
	case strings.TrimSpace(profileID) == "" || proof.ProfileID != profileID:
		return errors.New("simulator: safety proof profile is missing or mismatched")
	case strings.TrimSpace(proof.RuntimeBootID) == "":
		return errors.New("simulator: safety proof runtime boot identity is required")
	case proof.StatusSequence == 0:
		return errors.New("simulator: safety proof status sequence must be nonzero")
	case proof.ObservedAt.IsZero() || proof.FreshUntil.IsZero() || now.IsZero():
		return errors.New("simulator: safety proof timestamps are required")
	case now.Before(proof.ObservedAt) || !now.Before(proof.FreshUntil):
		return errors.New("simulator: safety proof is stale or from the future")
	case !proof.Simulation:
		return errors.New("simulator: simulation must be explicitly true")
	case !proof.NoRealMovement:
		return errors.New("simulator: no real movement must be explicitly true")
	case !proof.LoopbackOnly:
		return errors.New("simulator: loopback-only proof must be explicitly true")
	case proof.EtherCATOutputEnabled:
		return errors.New("simulator: EtherCAT output must be false")
	case proof.MotionOutputEnabled:
		return errors.New("simulator: motion output must be false")
	case proof.HardwareAccess:
		return errors.New("simulator: hardware access must be false")
	case proof.PhysicalServicesStarted:
		return errors.New("simulator: physical services started must be false")
	case proof.CommandSubmissionEnabled:
		return errors.New("simulator: command submission enabled must be false")
	default:
		return nil
	}
}

type RuntimeSnapshot struct {
	ProfileID      string              `json:"profile_id"`
	RobotModel     planning.RobotModel `json:"robot_model"`
	PlannerReady   bool                `json:"planner_ready"`
	SimulatorReady bool                `json:"simulator_ready"`
	PlanTransport  string              `json:"plan_transport"`
	Safety         SafetyProof         `json:"safety"`
}

func (snapshot RuntimeSnapshot) Validate(
	now time.Time,
	topology Topology,
	maxProofTTL time.Duration,
) error {
	if snapshot.ProfileID != topology.ProfileID {
		return errors.New("simulator: runtime profile is mismatched")
	}
	if snapshot.RobotModel != topology.RobotModel {
		return errors.New("simulator: runtime robot model is mismatched")
	}
	if !snapshot.PlannerReady || !snapshot.SimulatorReady {
		return errors.New("simulator: planner and simulator must both be ready")
	}
	if snapshot.PlanTransport != PlanTransportInProcess {
		return errors.New("simulator: plan transport must be in-process")
	}
	if maxProofTTL <= 0 {
		return errors.New("simulator: maximum proof TTL must be positive")
	}
	if err := snapshot.Safety.Validate(now, topology.ProfileID); err != nil {
		return err
	}
	if now.Sub(snapshot.Safety.ObservedAt) > maxProofTTL ||
		snapshot.Safety.FreshUntil.Sub(now) > maxProofTTL {
		return errors.New("simulator: safety proof exceeds the configured freshness window")
	}
	return nil
}

// Topology is intentionally smaller than the Steam Deck stack. The PC service
// needs no Steam, Gamescope, SSH, controller authority, NATS, RTCore, or
// EtherCAT process. It invokes the canonical planner through a persistent,
// contained local process and returns data for browser playback.
type Topology struct {
	Platform             string              `json:"platform"`
	ProfileID            string              `json:"profile_id"`
	RobotModel           planning.RobotModel `json:"robot_model"`
	LoopbackHost         string              `json:"loopback_host"`
	PlanTransport        string              `json:"plan_transport"`
	PhysicalServiceStart bool                `json:"physical_service_start"`
	CommandTransport     bool                `json:"command_transport"`
}

func NewPCTopology(
	profileID string,
	robotModel planning.RobotModel,
	loopbackHost string,
) Topology {
	return Topology{
		Platform:             PlatformPCNative,
		ProfileID:            strings.TrimSpace(profileID),
		RobotModel:           robotModel,
		LoopbackHost:         strings.TrimSpace(loopbackHost),
		PlanTransport:        PlanTransportInProcess,
		PhysicalServiceStart: false,
		CommandTransport:     false,
	}
}

func (topology Topology) Validate() error {
	if topology.Platform != PlatformPCNative {
		return errors.New("simulator: topology platform must be pc_native")
	}
	if strings.TrimSpace(topology.ProfileID) == "" {
		return errors.New("simulator: topology profile identity is required")
	}
	if !robotmodel.IsArm(string(topology.RobotModel)) {
		return errors.New("simulator: topology robot model is unsupported")
	}
	if !isLoopbackHost(topology.LoopbackHost) {
		return fmt.Errorf("simulator: host %q is not loopback", topology.LoopbackHost)
	}
	if topology.PlanTransport != PlanTransportInProcess {
		return errors.New("simulator: topology plan transport must be in-process")
	}
	if topology.PhysicalServiceStart {
		return errors.New("simulator: topology cannot start physical services")
	}
	if topology.CommandTransport {
		return errors.New("simulator: topology cannot expose a command transport")
	}
	return nil
}

type ResourceLimits struct {
	MaxPlanRequestBytes  int           `json:"max_plan_request_bytes"`
	MaxPlanReportBytes   int           `json:"max_plan_report_bytes"`
	MaxTrajectorySamples int           `json:"max_trajectory_samples"`
	MaxConcurrentPlans   int           `json:"max_concurrent_plans"`
	PlanTimeout          time.Duration `json:"plan_timeout"`
}

func DefaultResourceLimits() ResourceLimits {
	return ResourceLimits{
		MaxPlanRequestBytes:  8 << 20,
		MaxPlanReportBytes:   64 << 20,
		MaxTrajectorySamples: 250_000,
		MaxConcurrentPlans:   1,
		// The first Tesseract process on a PC can spend well over a minute
		// loading its Python/native collision environment before performing
		// the waypoint contact check. The application pays that cost once while
		// warming its persistent worker; keep startup and requests bounded.
		PlanTimeout: 4 * time.Minute,
	}
}

func (limits ResourceLimits) Validate() error {
	if limits.MaxPlanRequestBytes < 1 ||
		limits.MaxPlanReportBytes < 1 ||
		limits.MaxTrajectorySamples < 1 ||
		limits.MaxConcurrentPlans != 1 ||
		limits.PlanTimeout <= 0 {
		return errors.New("simulator: resource limits must be positive and allow exactly one concurrent plan")
	}
	if limits.MaxPlanRequestBytes > 64<<20 ||
		limits.MaxPlanReportBytes > 256<<20 ||
		limits.MaxTrajectorySamples > 1_000_000 ||
		limits.PlanTimeout > 10*time.Minute {
		return errors.New("simulator: resource limits exceed the local safety ceiling")
	}
	return nil
}

// SafetyPolicy is an invariant requested at start. It is deliberately not a
// SafetyProof: the backend must independently observe and return a fresh proof
// with its own boot identity and status sequence.
type SafetyPolicy struct {
	Simulation               bool `json:"simulation"`
	NoRealMovement           bool `json:"no_real_movement"`
	LoopbackOnly             bool `json:"loopback_only"`
	EtherCATOutputEnabled    bool `json:"ethercat_output_enabled"`
	MotionOutputEnabled      bool `json:"motion_output_enabled"`
	HardwareAccess           bool `json:"hardware_access"`
	PhysicalServicesStarted  bool `json:"physical_services_started"`
	CommandSubmissionEnabled bool `json:"command_submission_enabled"`
}

type StartRequest struct {
	Topology Topology       `json:"topology"`
	Limits   ResourceLimits `json:"limits"`
	Policy   SafetyPolicy   `json:"policy"`
}

type Readiness struct {
	Ready   bool     `json:"ready"`
	Reasons []string `json:"reasons"`
}

type Status struct {
	Schema         string                `json:"schema"`
	ServiceID      string                `json:"service_id"`
	StatusSequence uint64                `json:"status_sequence"`
	Phase          LifecyclePhase        `json:"phase"`
	ObservedAt     time.Time             `json:"observed_at"`
	Availability   BackendAvailability   `json:"availability"`
	Topology       Topology              `json:"topology"`
	Readiness      Readiness             `json:"readiness"`
	Safety         SafetyProof           `json:"safety"`
	PlanInFlight   bool                  `json:"plan_in_flight"`
	LastPlan       planning.PlanIdentity `json:"last_plan"`
	LastErrorCode  ErrorCode             `json:"last_error_code,omitempty"`
	LastError      string                `json:"last_error,omitempty"`
}

type StatusEnvelope struct {
	Status Status `json:"status"`
}

type PlanEnvelope struct {
	Result planning.NormalizedPlanResult `json:"result"`
}

func isLoopbackHost(host string) bool {
	host = strings.TrimSpace(strings.Trim(host, "[]"))
	if strings.EqualFold(host, "localhost") {
		return true
	}
	address := net.ParseIP(host)
	return address != nil && address.IsLoopback()
}
