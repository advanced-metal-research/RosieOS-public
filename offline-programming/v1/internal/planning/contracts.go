// Package planning defines transport-independent planning and execution
// readiness contracts for Offline Programming V1.
package planning

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"
	"time"
)

const (
	NormalizedPlanRequestSchema  = "offline-programming.normalized-plan-request.v1"
	NormalizedPlanEvidenceSchema = "offline-programming.normalized-plan-evidence.v1"
	JointCount                   = 9
)

type RobotModel string

const (
	RobotModelRosie1400V3 RobotModel = "rosie_1400_v3"
	RobotModelRosie1420V1 RobotModel = "rosie_1420_v1"
)

type PlanValidationState string

const (
	PlanUnvalidated     PlanValidationState = "unvalidated"
	PlanServerValidated PlanValidationState = "server_validated"
	PlanRejected        PlanValidationState = "rejected"
	PlanStale           PlanValidationState = "stale"
)

type CheckState string

const (
	CheckUnavailable CheckState = "unavailable"
	CheckClear       CheckState = "clear"
	CheckBlocked     CheckState = "blocked"
)

type TargetIdentity struct {
	ProfileID   string `json:"profile_id"`
	ManifestRef string `json:"manifest_ref,omitempty"`
}

type PlanIdentity struct {
	ProjectID       string         `json:"project_id"`
	ProjectRevision uint64         `json:"project_revision"`
	ProgramID       string         `json:"program_id"`
	ProgramRevision uint64         `json:"program_revision"`
	PlanID          string         `json:"plan_id"`
	PlanRevision    uint64         `json:"plan_revision"`
	ProgramHash     string         `json:"program_hash"`
	RobotModel      RobotModel     `json:"robot_model"`
	Target          TargetIdentity `json:"target"`
}

func (identity PlanIdentity) Complete() bool {
	return strings.TrimSpace(identity.ProjectID) != "" &&
		identity.ProjectRevision > 0 &&
		strings.TrimSpace(identity.ProgramID) != "" &&
		identity.ProgramRevision > 0 &&
		strings.TrimSpace(identity.PlanID) != "" &&
		identity.PlanRevision > 0 &&
		strings.TrimSpace(identity.ProgramHash) != "" &&
		supportedRobotModel(identity.RobotModel) &&
		strings.TrimSpace(identity.Target.ProfileID) != ""
}

func (identity PlanIdentity) Equal(other PlanIdentity) bool {
	return identity.ProjectID == other.ProjectID &&
		identity.ProjectRevision == other.ProjectRevision &&
		identity.ProgramID == other.ProgramID &&
		identity.ProgramRevision == other.ProgramRevision &&
		identity.PlanID == other.PlanID &&
		identity.PlanRevision == other.PlanRevision &&
		identity.ProgramHash == other.ProgramHash &&
		identity.RobotModel == other.RobotModel &&
		identity.Target == other.Target
}

type PlanRow struct {
	RowID             string              `json:"row_id"`
	Revision          uint64              `json:"revision"`
	Enabled           bool                `json:"enabled"`
	RowType           string              `json:"row_type"`
	Command           string              `json:"command,omitempty"`
	MoveType          string              `json:"move_type"`
	MotionPhase       string              `json:"motion_phase"`
	ToolFrame         string              `json:"tool_frame"`
	WorkFrame         string              `json:"work_frame"`
	TargetTCPM        [6]float64          `json:"target_tcp_m"`
	TargetJointsRad   [JointCount]float64 `json:"target_joints_rad"`
	TCPVelocityMS     float64             `json:"tcp_velocity_m_s"`
	JointVelocityRadS float64             `json:"joint_velocity_rad_s"`
	WeldEnable        bool                `json:"weld_enable"`
	ArcState          string              `json:"arc_state"`
	RevPiIOSlot       *int                `json:"revpi_io_slot,omitempty"`
	RevPiIOAction     string              `json:"revpi_io_action"`
}

type NormalizedPlanRequest struct {
	Schema              string         `json:"schema"`
	RequestID           string         `json:"request_id"`
	ProjectID           string         `json:"project_id"`
	ProjectRevision     uint64         `json:"project_revision"`
	ProgramID           string         `json:"program_id"`
	ProgramRevision     uint64         `json:"program_revision"`
	BaseProgramRevision uint64         `json:"base_program_revision"`
	RobotModel          RobotModel     `json:"robot_model"`
	Target              TargetIdentity `json:"target"`
	SubmittedAt         time.Time      `json:"submitted_at"`
	Rows                []PlanRow      `json:"rows"`
	// Program is the complete shared Robot V4 authoring document received
	// from Offline Programming or SteamDeck. It is passed byte-for-byte to
	// robot-v4-cartesiand; Rows is a validated index used only for correlation
	// and safety review.
	Program json.RawMessage `json:"-"`
}

func (request NormalizedPlanRequest) Validate() error {
	if request.Schema != NormalizedPlanRequestSchema {
		return fmt.Errorf("planning: schema must be %q", NormalizedPlanRequestSchema)
	}
	if strings.TrimSpace(request.RequestID) == "" ||
		strings.TrimSpace(request.ProjectID) == "" ||
		strings.TrimSpace(request.ProgramID) == "" {
		return errors.New("planning: request, project, and program identities are required")
	}
	if request.ProjectRevision == 0 || request.ProgramRevision == 0 {
		return errors.New("planning: project and program revisions must be nonzero")
	}
	if request.BaseProgramRevision > request.ProgramRevision {
		return errors.New("planning: base program revision exceeds program revision")
	}
	if !supportedRobotModel(request.RobotModel) {
		return fmt.Errorf("planning: unsupported robot model %q", request.RobotModel)
	}
	if strings.TrimSpace(request.Target.ProfileID) == "" {
		return errors.New("planning: target profile identity is required")
	}
	if request.SubmittedAt.IsZero() {
		return errors.New("planning: submitted timestamp is required")
	}
	if len(request.Program) > 0 {
		if _, err := validateRobotV4ProgramDocument(request.Program, request); err != nil {
			return err
		}
	}
	if len(request.Rows) < 2 {
		return errors.New("planning: at least two program rows are required")
	}
	seen := make(map[string]struct{}, len(request.Rows))
	enabledMotionRows := 0
	for index, row := range request.Rows {
		if strings.TrimSpace(row.RowID) == "" || row.Revision == 0 {
			return fmt.Errorf("planning: row %d has incomplete identity", index)
		}
		if _, duplicate := seen[row.RowID]; duplicate {
			return fmt.Errorf("planning: duplicate row id %q", row.RowID)
		}
		seen[row.RowID] = struct{}{}
		if row.Enabled && row.RowType == "motion" {
			enabledMotionRows++
		}
		if !finiteArray(row.TargetTCPM[:]) || !finiteArray(row.TargetJointsRad[:]) {
			return fmt.Errorf("planning: row %q contains non-finite targets", row.RowID)
		}
		if !finiteNonnegative(row.TCPVelocityMS) || !finiteNonnegative(row.JointVelocityRadS) {
			return fmt.Errorf("planning: row %q contains invalid velocity", row.RowID)
		}
	}
	if enabledMotionRows < 2 {
		return errors.New("planning: at least two enabled motion rows are required")
	}
	return nil
}

type CollisionPoint struct {
	RowIndex       int       `json:"row_index"`
	WaypointIndex  int       `json:"waypoint_index"`
	LinkA          string    `json:"link_a"`
	LinkB          string    `json:"link_b"`
	DepthM         float64   `json:"depth_m"`
	AffectedJoints []string  `json:"affected_joints"`
	PositionsRad   []float64 `json:"positions_rad,omitempty"`
}

type SingularityPoint struct {
	RowIndex             int       `json:"row_index"`
	WaypointIndex        int       `json:"waypoint_index"`
	State                string    `json:"state"`
	AppliedVelocityScale float64   `json:"applied_velocity_scale"`
	LimitingJoint        string    `json:"limiting_joint,omitempty"`
	AffectedJoints       []string  `json:"affected_joints"`
	PositionsRad         []float64 `json:"positions_rad,omitempty"`
}

type UnreachablePoint struct {
	RowIndex      int       `json:"row_index"`
	WaypointIndex int       `json:"waypoint_index"`
	Reason        string    `json:"reason"`
	Joint         string    `json:"joint,omitempty"`
	PositionsRad  []float64 `json:"positions_rad,omitempty"`
}

type CollisionEvidence struct {
	State      CheckState       `json:"state"`
	CheckState string           `json:"check_state"`
	Points     []CollisionPoint `json:"points"`
}

type SingularityEvidence struct {
	State      CheckState         `json:"state"`
	CheckState string             `json:"check_state"`
	Points     []SingularityPoint `json:"points"`
}

type LimitEvidence struct {
	State      CheckState         `json:"state"`
	CheckState string             `json:"check_state"`
	Points     []UnreachablePoint `json:"points"`
}

type RuntimeEvidence struct {
	State    CheckState `json:"state"`
	Playable bool       `json:"playable"`
}

type PlanTimingEvidence struct {
	PlanningTimeS                  float64 `json:"planning_time_s"`
	CollisionCheckTimeS            float64 `json:"collision_check_time_s"`
	CollisionWorkerWallTime        float64 `json:"collision_worker_wall_time_s"`
	CollisionCacheHitWaypointCount int     `json:"collision_cache_hit_waypoint_count"`
}

type NormalizedPlanEvidence struct {
	Schema             string              `json:"schema"`
	RequestID          string              `json:"request_id"`
	Identity           PlanIdentity        `json:"identity"`
	GeneratedAt        time.Time           `json:"generated_at"`
	PlannerOwner       string              `json:"planner_owner"`
	PlannerBackend     string              `json:"planner_backend"`
	State              PlanValidationState `json:"state"`
	Accepted           bool                `json:"accepted"`
	ValidationState    string              `json:"validation_state"`
	SampleCount        int                 `json:"sample_count"`
	Collision          CollisionEvidence   `json:"collision"`
	Singularity        SingularityEvidence `json:"singularity"`
	Limits             LimitEvidence       `json:"limits"`
	Runtime            RuntimeEvidence     `json:"runtime"`
	Timing             PlanTimingEvidence  `json:"timing"`
	CorrelationReasons []string            `json:"correlation_reasons"`
}

func (evidence NormalizedPlanEvidence) ServerValidated() bool {
	return evidence.Schema == NormalizedPlanEvidenceSchema &&
		evidence.State == PlanServerValidated &&
		evidence.Accepted &&
		evidence.Identity.Complete() &&
		len(evidence.CorrelationReasons) == 0 &&
		evidence.Collision.State == CheckClear &&
		evidence.Singularity.State == CheckClear &&
		evidence.Limits.State == CheckClear &&
		evidence.Runtime.State == CheckClear &&
		evidence.Runtime.Playable
}

// supportedRobotModel reports whether model is an arm this tree describes
// (robotmodel: the description directory is the record of a model).
func supportedRobotModel(model RobotModel) bool { return robotmodel.IsArm(string(model)) }

func finiteArray(values []float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func finiteNonnegative(value float64) bool {
	return !math.IsNaN(value) && !math.IsInf(value, 0) && value >= 0
}
