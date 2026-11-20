package planning

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"regexp"
	"strings"
	"time"
)

const (
	RobotV4AcceptedPlanSchema          = "robot.v4.accepted-plan.v1"
	RobotV4AcceptedPlanReferenceSchema = "robot.v4.accepted-plan-reference.v1"
)

var sha256DigestPattern = regexp.MustCompile(`^sha256:[0-9a-f]{64}$`)

type AcceptedPlanAuthority string

const (
	AcceptedPlanSimulationOnly AcceptedPlanAuthority = "simulation_only"
	AcceptedPlanPhysicalTarget AcceptedPlanAuthority = "physical_target"
)

type AcceptedPlanPlannerIdentity struct {
	Owner            string `json:"owner"`
	Backend          string `json:"backend"`
	TrajectoryMethod string `json:"trajectory_method"`
	BuildDigest      string `json:"build_digest"`
	RequestSubject   string `json:"request_subject,omitempty"`
}

type AcceptedPlanTargetIdentity struct {
	TargetID             string             `json:"target_id"`
	RobotModel           RobotModel         `json:"robot_model"`
	JointOrder           [JointCount]string `json:"joint_order"`
	ManifestRevision     string             `json:"manifest_revision"`
	CellConfigRevision   string             `json:"cell_config_revision"`
	CalibrationDigest    string             `json:"calibration_digest"`
	URDFDigest           string             `json:"urdf_digest"`
	MeshDigest           string             `json:"mesh_digest,omitempty"`
	CollisionWorldDigest string             `json:"collision_world_digest"`
	SettingsRevision     uint64             `json:"settings_revision"`
	ToolFrame            string             `json:"tool_frame,omitempty"`
	WorkFrame            string             `json:"work_frame,omitempty"`
}

type AcceptedPlanEvidence struct {
	PlanningContractAccepted bool               `json:"planning_contract_accepted"`
	Collision                string             `json:"collision"`
	Singularity              string             `json:"singularity"`
	Limits                   string             `json:"limits"`
	StartPose                string             `json:"start_pose"`
	RuntimePlayable          bool               `json:"runtime_playable"`
	CollisionPoints          []CollisionPoint   `json:"collision_points"`
	SingularityPoints        []SingularityPoint `json:"singularity_points"`
}

type AcceptedPlanSample struct {
	SampleIndex       int                 `json:"sample_index"`
	RowID             string              `json:"row_id"`
	RowIndex          int                 `json:"row_index"`
	TimeS             float64             `json:"time_s"`
	TCPPoseM          [6]float64          `json:"tcp_pose_m"`
	JointPositionsRad [JointCount]float64 `json:"joint_positions_rad"`
	WeldOn            bool                `json:"weld_on"`
}

type RobotV4AcceptedPlan struct {
	Schema                  string                      `json:"schema"`
	AuthorityClassification AcceptedPlanAuthority       `json:"authority_classification"`
	PlanID                  string                      `json:"plan_id"`
	PlanRevision            uint64                      `json:"plan_revision"`
	TrajectoryDigest        string                      `json:"trajectory_digest"`
	ProgramID               string                      `json:"program_id"`
	ProgramRevision         uint64                      `json:"program_revision"`
	ProgramDigest           string                      `json:"program_digest"`
	Planner                 AcceptedPlanPlannerIdentity `json:"planner"`
	Target                  AcceptedPlanTargetIdentity  `json:"target"`
	Evidence                AcceptedPlanEvidence        `json:"evidence"`
	RequestID               string                      `json:"request_id,omitempty"`
	Trajectory              []AcceptedPlanSample        `json:"trajectory"`
	CreatedAt               time.Time                   `json:"created_at"`
}

type RobotV4AcceptedPlanReference struct {
	Schema           string     `json:"schema"`
	PlanID           string     `json:"plan_id"`
	PlanRevision     uint64     `json:"plan_revision"`
	TrajectoryDigest string     `json:"trajectory_digest"`
	ProgramID        string     `json:"program_id"`
	ProgramRevision  uint64     `json:"program_revision"`
	ProgramDigest    string     `json:"program_digest"`
	TargetID         string     `json:"target_id"`
	RobotModel       RobotModel `json:"robot_model"`
	ManifestRevision string     `json:"manifest_revision"`
}

func (plan RobotV4AcceptedPlan) Reference() RobotV4AcceptedPlanReference {
	return RobotV4AcceptedPlanReference{
		Schema:           RobotV4AcceptedPlanReferenceSchema,
		PlanID:           plan.PlanID,
		PlanRevision:     plan.PlanRevision,
		TrajectoryDigest: plan.TrajectoryDigest,
		ProgramID:        plan.ProgramID,
		ProgramRevision:  plan.ProgramRevision,
		ProgramDigest:    plan.ProgramDigest,
		TargetID:         plan.Target.TargetID,
		RobotModel:       plan.Target.RobotModel,
		ManifestRevision: plan.Target.ManifestRevision,
	}
}

func (reference RobotV4AcceptedPlanReference) Validate() error {
	if reference.Schema != RobotV4AcceptedPlanReferenceSchema ||
		strings.TrimSpace(reference.PlanID) == "" ||
		reference.PlanRevision == 0 ||
		!sha256DigestPattern.MatchString(reference.TrajectoryDigest) ||
		strings.TrimSpace(reference.ProgramID) == "" ||
		reference.ProgramRevision == 0 ||
		!sha256DigestPattern.MatchString(reference.ProgramDigest) ||
		strings.TrimSpace(reference.TargetID) == "" ||
		!supportedRobotModel(reference.RobotModel) ||
		strings.TrimSpace(reference.ManifestRevision) == "" {
		return errors.New("planning: accepted plan reference is incomplete")
	}
	return nil
}

// Validate checks the immutable accepted-plan artifact without relying on
// browser state. It deliberately validates the digest of the exact trajectory
// representation carried by the artifact: a transport cannot alter samples
// while retaining the target-issued identity.
func (plan RobotV4AcceptedPlan) Validate() error {
	if plan.Schema != RobotV4AcceptedPlanSchema {
		return fmt.Errorf(
			"planning: accepted plan schema must be %q",
			RobotV4AcceptedPlanSchema,
		)
	}
	if plan.AuthorityClassification != AcceptedPlanSimulationOnly &&
		plan.AuthorityClassification != AcceptedPlanPhysicalTarget {
		return errors.New("planning: accepted plan authority classification is invalid")
	}
	if strings.TrimSpace(plan.PlanID) == "" ||
		plan.PlanRevision == 0 ||
		strings.TrimSpace(plan.ProgramID) == "" ||
		plan.ProgramRevision == 0 ||
		plan.CreatedAt.IsZero() {
		return errors.New("planning: accepted plan identity is incomplete")
	}
	for name, digest := range map[string]string{
		"trajectory":      plan.TrajectoryDigest,
		"program":         plan.ProgramDigest,
		"planner build":   plan.Planner.BuildDigest,
		"calibration":     plan.Target.CalibrationDigest,
		"URDF":            plan.Target.URDFDigest,
		"collision world": plan.Target.CollisionWorldDigest,
	} {
		if !sha256DigestPattern.MatchString(digest) {
			return fmt.Errorf("planning: accepted plan %s digest is invalid", name)
		}
	}
	if plan.Target.MeshDigest != "" &&
		!sha256DigestPattern.MatchString(plan.Target.MeshDigest) {
		return errors.New("planning: accepted plan mesh digest is invalid")
	}
	if plan.Planner.Owner != MotionServerPlannerOwner ||
		strings.TrimSpace(plan.Planner.Backend) == "" ||
		strings.TrimSpace(plan.Planner.TrajectoryMethod) == "" ||
		strings.TrimSpace(plan.Target.TargetID) == "" ||
		!supportedRobotModel(plan.Target.RobotModel) ||
		strings.TrimSpace(plan.Target.ManifestRevision) == "" ||
		strings.TrimSpace(plan.Target.CellConfigRevision) == "" ||
		plan.Target.SettingsRevision == 0 {
		return errors.New("planning: accepted plan planner or target identity is incomplete")
	}
	if plan.Target.JointOrder != robotV4JointOrder() {
		return errors.New("planning: accepted plan joint order is not canonical J1-J9")
	}
	if !plan.Evidence.PlanningContractAccepted ||
		plan.Evidence.Collision != "clear" ||
		(plan.Evidence.Singularity != "clear" &&
			plan.Evidence.Singularity != "warning") ||
		plan.Evidence.Limits != "clear" {
		return errors.New("planning: accepted plan safety evidence is not executable")
	}
	if plan.AuthorityClassification == AcceptedPlanPhysicalTarget &&
		!plan.Evidence.RuntimePlayable {
		return errors.New("planning: physical accepted plan is not runtime playable")
	}
	if plan.AuthorityClassification == AcceptedPlanSimulationOnly &&
		plan.Evidence.RuntimePlayable {
		return errors.New("planning: simulation-only accepted plan cannot be runtime playable")
	}
	if len(plan.Trajectory) < 2 {
		return errors.New("planning: accepted plan trajectory requires at least two samples")
	}
	previousTime := -1.0
	for index, sample := range plan.Trajectory {
		if sample.SampleIndex != index ||
			sample.RowIndex < 0 ||
			strings.TrimSpace(sample.RowID) == "" ||
			math.IsNaN(sample.TimeS) ||
			math.IsInf(sample.TimeS, 0) ||
			sample.TimeS < 0 ||
			sample.TimeS < previousTime ||
			!finiteArray(sample.TCPPoseM[:]) ||
			!finiteArray(sample.JointPositionsRad[:]) {
			return fmt.Errorf(
				"planning: accepted plan trajectory sample %d is invalid",
				index,
			)
		}
		previousTime = sample.TimeS
	}
	if AcceptedTrajectoryDigest(plan.Trajectory) != plan.TrajectoryDigest {
		return errors.New("planning: accepted plan trajectory digest does not match its samples")
	}
	return nil
}

// ValidateForRequest binds a target-issued artifact to the exact saved
// program request and selected target. It is the promotion boundary between a
// returned plan-shaped document and an accepted physical plan.
func (plan RobotV4AcceptedPlan) ValidateForRequest(
	request NormalizedPlanRequest,
	authority AcceptedPlanAuthority,
) error {
	if err := request.Validate(); err != nil {
		return err
	}
	if err := plan.Validate(); err != nil {
		return err
	}
	if plan.AuthorityClassification != authority {
		return errors.New("planning: accepted plan authority does not match the request")
	}
	if plan.ProgramID != request.ProgramID ||
		plan.ProgramRevision != request.ProgramRevision ||
		plan.RequestID != request.RequestID ||
		plan.Target.TargetID != request.Target.ProfileID ||
		plan.Target.RobotModel != request.RobotModel {
		return errors.New("planning: accepted plan identity does not match the saved program request")
	}
	expectedProgramDigest, err := RobotV4ProgramDigest(request.Program)
	if err != nil {
		return err
	}
	if plan.ProgramDigest != expectedProgramDigest {
		return errors.New("planning: accepted plan program digest does not match the saved program")
	}
	for _, sample := range plan.Trajectory {
		if sample.RowIndex >= len(request.Rows) ||
			request.Rows[sample.RowIndex].RowID != sample.RowID {
			return errors.New("planning: accepted plan row identity does not match the saved program")
		}
	}
	return nil
}

// RobotV4ProgramDigest returns the contract digest carried by a finalized
// shared program. If the document has not been finalized, it falls back to a
// digest of its exact JSON bytes; callers must therefore use the same saved
// bytes when correlating a plan.
func RobotV4ProgramDigest(raw json.RawMessage) (string, error) {
	if len(raw) == 0 || !json.Valid(raw) {
		return "", errors.New("planning: shared Robot V4 program JSON is invalid")
	}
	var metadata struct {
		ProgramDigest string `json:"program_digest"`
	}
	if err := json.Unmarshal(raw, &metadata); err != nil {
		return "", fmt.Errorf("planning: decode shared program digest: %w", err)
	}
	if sha256DigestPattern.MatchString(strings.TrimSpace(metadata.ProgramDigest)) {
		return strings.TrimSpace(metadata.ProgramDigest), nil
	}
	return sha256Digest(raw), nil
}

// AcceptedPlanAssets are supplied by the process that actually invoked the
// planner. A physical plan must receive these values from the selected target;
// Offline Programming cannot invent them from browser state.
type AcceptedPlanAssets struct {
	PlannerBuildDigest   string
	TrajectoryMethod     string
	ManifestRevision     string
	CellConfigRevision   string
	CalibrationDigest    string
	URDFDigest           string
	MeshDigest           string
	CollisionWorldDigest string
	SettingsRevision     uint64
}

func MintRobotV4AcceptedPlan(
	request NormalizedPlanRequest,
	result NormalizedPlanResult,
	assets AcceptedPlanAssets,
	authority AcceptedPlanAuthority,
	createdAt time.Time,
) (RobotV4AcceptedPlan, error) {
	if !result.RuntimePlayable() {
		return RobotV4AcceptedPlan{}, errors.New(
			"planning: only a complete server-validated result can become an accepted plan",
		)
	}
	if authority != AcceptedPlanSimulationOnly &&
		authority != AcceptedPlanPhysicalTarget {
		return RobotV4AcceptedPlan{}, errors.New(
			"planning: accepted plan authority classification is invalid",
		)
	}
	if createdAt.IsZero() {
		return RobotV4AcceptedPlan{}, errors.New(
			"planning: accepted plan creation time is required",
		)
	}
	for name, digest := range map[string]string{
		"planner build":   assets.PlannerBuildDigest,
		"calibration":     assets.CalibrationDigest,
		"URDF":            assets.URDFDigest,
		"collision world": assets.CollisionWorldDigest,
	} {
		if !sha256DigestPattern.MatchString(digest) {
			return RobotV4AcceptedPlan{}, fmt.Errorf(
				"planning: %s digest is invalid",
				name,
			)
		}
	}
	if assets.MeshDigest != "" &&
		!sha256DigestPattern.MatchString(assets.MeshDigest) {
		return RobotV4AcceptedPlan{}, errors.New(
			"planning: mesh digest is invalid",
		)
	}
	program, err := decodeAcceptedPlanProgramMetadata(request.Program)
	if err != nil {
		return RobotV4AcceptedPlan{}, err
	}
	cellConfigRevision := strings.TrimSpace(program.CellConfigRevision)
	if cellConfigRevision == "" {
		cellConfigRevision = strings.TrimSpace(assets.CellConfigRevision)
	}
	settingsRevision := program.Settings.Revision
	if settingsRevision == 0 {
		settingsRevision = assets.SettingsRevision
	}
	if strings.TrimSpace(assets.ManifestRevision) == "" ||
		cellConfigRevision == "" ||
		settingsRevision == 0 ||
		strings.TrimSpace(assets.TrajectoryMethod) == "" {
		return RobotV4AcceptedPlan{}, errors.New(
			"planning: accepted plan target/build revision identity is incomplete",
		)
	}
	programDigest := strings.TrimSpace(program.ProgramDigest)
	if !sha256DigestPattern.MatchString(programDigest) {
		programDigest = sha256Digest(request.Program)
	}
	trajectory := make([]AcceptedPlanSample, len(result.Trajectory))
	for index, sample := range result.Trajectory {
		if sample.RowIndex < 0 || sample.RowIndex >= len(request.Rows) {
			return RobotV4AcceptedPlan{}, errors.New(
				"planning: accepted trajectory row identity is invalid",
			)
		}
		trajectory[index] = AcceptedPlanSample{
			SampleIndex:       index,
			RowID:             request.Rows[sample.RowIndex].RowID,
			RowIndex:          sample.RowIndex,
			TimeS:             sample.TimeS,
			TCPPoseM:          sample.TCPPose,
			JointPositionsRad: sample.PositionsRad,
			WeldOn:            sample.WeldOn,
		}
	}
	singularity := "clear"
	if len(result.Evidence.Singularity.Points) > 0 {
		singularity = "warning"
	}
	targetID := request.Target.ProfileID
	target := AcceptedPlanTargetIdentity{
		TargetID:             targetID,
		RobotModel:           request.RobotModel,
		JointOrder:           robotV4JointOrder(),
		ManifestRevision:     assets.ManifestRevision,
		CellConfigRevision:   cellConfigRevision,
		CalibrationDigest:    assets.CalibrationDigest,
		URDFDigest:           assets.URDFDigest,
		MeshDigest:           assets.MeshDigest,
		CollisionWorldDigest: assets.CollisionWorldDigest,
		SettingsRevision:     settingsRevision,
		ToolFrame:            program.Defaults.DefaultToolFrame,
		WorkFrame:            program.Defaults.DefaultWorkFrame,
	}
	return RobotV4AcceptedPlan{
		Schema:                  RobotV4AcceptedPlanSchema,
		AuthorityClassification: authority,
		PlanID:                  result.Evidence.Identity.PlanID,
		PlanRevision:            result.Evidence.Identity.PlanRevision,
		TrajectoryDigest:        AcceptedTrajectoryDigest(trajectory),
		ProgramID:               request.ProgramID,
		ProgramRevision:         request.ProgramRevision,
		ProgramDigest:           programDigest,
		Planner: AcceptedPlanPlannerIdentity{
			Owner:            MotionServerPlannerOwner,
			Backend:          result.Evidence.PlannerBackend,
			TrajectoryMethod: assets.TrajectoryMethod,
			BuildDigest:      assets.PlannerBuildDigest,
		},
		Target: target,
		Evidence: AcceptedPlanEvidence{
			PlanningContractAccepted: true,
			Collision:                "clear",
			Singularity:              singularity,
			Limits:                   "clear",
			StartPose:                "unavailable",
			RuntimePlayable:          authority == AcceptedPlanPhysicalTarget,
			CollisionPoints:          append([]CollisionPoint{}, result.Evidence.Collision.Points...),
			SingularityPoints:        append([]SingularityPoint{}, result.Evidence.Singularity.Points...),
		},
		RequestID:  request.RequestID,
		Trajectory: trajectory,
		CreatedAt:  createdAt.UTC(),
	}, nil
}

// AcceptedTrajectoryDigest is the cross-language digest used by Robot V4,
// Offline Programming, and SteamDeck. It hashes a versioned, length-delimited
// IEEE-754 representation rather than JSON text, so equivalent samples cannot
// acquire different identities because two runtimes format a float
// differently.
func AcceptedTrajectoryDigest(trajectory []AcceptedPlanSample) string {
	hash := sha256.New()
	_, _ = hash.Write([]byte("robot.v4.accepted-trajectory.v1\x00"))
	writeAcceptedDigestUint64(hash, uint64(len(trajectory)))
	for _, sample := range trajectory {
		writeAcceptedDigestUint64(hash, uint64(sample.SampleIndex))
		writeAcceptedDigestUint64(hash, uint64(sample.RowIndex))
		writeAcceptedDigestUint64(hash, uint64(len(sample.RowID)))
		_, _ = hash.Write([]byte(sample.RowID))
		writeAcceptedDigestFloat64(hash, sample.TimeS)
		for _, value := range sample.TCPPoseM {
			writeAcceptedDigestFloat64(hash, value)
		}
		for _, value := range sample.JointPositionsRad {
			writeAcceptedDigestFloat64(hash, value)
		}
		if sample.WeldOn {
			_, _ = hash.Write([]byte{1})
		} else {
			_, _ = hash.Write([]byte{0})
		}
	}
	return "sha256:" + hex.EncodeToString(hash.Sum(nil))
}

func writeAcceptedDigestUint64(hash interface{ Write([]byte) (int, error) }, value uint64) {
	var raw [8]byte
	binary.BigEndian.PutUint64(raw[:], value)
	_, _ = hash.Write(raw[:])
}

func writeAcceptedDigestFloat64(hash interface{ Write([]byte) (int, error) }, value float64) {
	writeAcceptedDigestUint64(hash, math.Float64bits(value))
}

type acceptedPlanProgramMetadata struct {
	ProgramDigest      string `json:"program_digest"`
	CellConfigRevision string `json:"cell_config_revision"`
	Settings           struct {
		Revision uint64 `json:"revision"`
	} `json:"settings"`
	Defaults struct {
		DefaultToolFrame string `json:"default_tool_frame"`
		DefaultWorkFrame string `json:"default_work_frame"`
	} `json:"defaults"`
}

func decodeAcceptedPlanProgramMetadata(
	raw json.RawMessage,
) (acceptedPlanProgramMetadata, error) {
	var metadata acceptedPlanProgramMetadata
	if err := json.Unmarshal(raw, &metadata); err != nil {
		return metadata, fmt.Errorf(
			"planning: decode accepted plan program metadata: %w",
			err,
		)
	}
	return metadata, nil
}

func sha256Digest(raw []byte) string {
	sum := sha256.Sum256(raw)
	return "sha256:" + hex.EncodeToString(sum[:])
}

func robotV4JointOrder() [JointCount]string {
	var order [JointCount]string
	for index := range order {
		order[index] = fmt.Sprintf("J%d", index+1)
	}
	return order
}
