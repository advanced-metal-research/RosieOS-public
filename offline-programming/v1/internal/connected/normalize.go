package connected

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"rosieos/offline-programming/v1/internal/robotmodel"
	"strings"
	"sync"
	"time"
)

type webSocketEnvelope struct {
	Type         string          `json:"type"`
	State        string          `json:"state"`
	Subject      string          `json:"subject"`
	ReceivedAtMS int64           `json:"received_at_ms"`
	Status       json.RawMessage `json:"status"`
	Plan         json.RawMessage `json:"plan"`
}

type rawStatus struct {
	Name                     string `json:"name"`
	ServiceName              string `json:"service_name"`
	ServiceState             string `json:"service_state"`
	State                    string `json:"state"`
	UpdatedAt                string `json:"updated_at"`
	PlanPreloaded            *bool  `json:"plan_preloaded"`
	LoadedProgramAvailable   *bool  `json:"loaded_program_available"`
	LoadedProgramRevision    uint64 `json:"loaded_program_revision"`
	LoadedProgramPlayable    *bool  `json:"loaded_program_playable"`
	PlanRevision             uint64 `json:"plan_revision"`
	PlanSessionID            string `json:"plan_session_id"`
	PlanPlaybackActive       *bool  `json:"plan_playback_active"`
	PlanServerValidated      *bool  `json:"plan_server_validated"`
	PlanPlayAuthorized       *bool  `json:"plan_play_authorized"`
	AcceptedPlanID           string `json:"accepted_plan_id"`
	AcceptedProgramDigest    string `json:"accepted_program_digest"`
	AcceptedTrajectoryDigest string `json:"accepted_trajectory_digest"`
	AcceptedManifestRevision string `json:"accepted_manifest_revision"`
	ServosArmedRequested     *bool  `json:"servos_armed_requested"`
	CurrentOutputZeroHold    *bool  `json:"current_output_zero_hold"`
	StaleZeroHold            *bool  `json:"stale_zero_hold"`
	LeaderID                 string `json:"leader_id"`
	LeaderBootID             string `json:"leader_boot_id"`
	LeaderLeaseID            string `json:"leader_lease_id"`
	LeaderFenceEpoch         uint64 `json:"leader_fence_epoch"`
	LeaderCampaignGeneration uint64 `json:"leader_campaign_generation"`
	LeaderLeaseFresh         *bool  `json:"leader_lease_fresh"`
	LeaderExpiresInMS        int64  `json:"leader_expires_in_ms"`
	LeaderTransitionPending  *bool  `json:"leader_transition_pending"`
}

type rawPlan struct {
	PlanID                       string             `json:"plan_id"`
	PlanSessionID                string             `json:"plan_session_id"`
	PlanRevision                 uint64             `json:"plan_revision"`
	PlannedPathRevision          uint64             `json:"planned_path_revision"`
	PlannerOwner                 string             `json:"planner_owner"`
	PlannerBackend               string             `json:"planner_backend"`
	ProgramID                    string             `json:"program_id"`
	ProgramRevision              uint64             `json:"program_revision"`
	AcceptedProgramRevision      uint64             `json:"accepted_program_revision"`
	ProgramHash                  string             `json:"program_hash"`
	RobotModel                   string             `json:"robot_model"`
	ValidationState              string             `json:"validation_state"`
	Accepted                     *bool              `json:"accepted"`
	PlayEnabled                  *bool              `json:"play_enabled"`
	RuntimePreloadEnabled        *bool              `json:"runtime_preload_enabled"`
	CollisionGateClear           *bool              `json:"collision_gate_clear"`
	CollisionCheckState          string             `json:"collision_check_state"`
	CollisionPoints              *[]json.RawMessage `json:"collision_points"`
	SingularityAnalysisAvailable *bool              `json:"singularity_analysis_available"`
	SingularityGateClear         *bool              `json:"singularity_gate_clear"`
	SingularityCheckState        string             `json:"singularity_check_state"`
	SingularityPoints            *[]json.RawMessage `json:"singularity_points"`
	LimitCheckState              string             `json:"limit_check_state"`
	UnreachablePoints            *[]json.RawMessage `json:"unreachable_points"`
	PlannedSampleCount           int                `json:"planned_sample_count"`
	AcceptedPlan                 *struct {
		PlanID          string `json:"plan_id"`
		PlanRevision    uint64 `json:"plan_revision"`
		ProgramID       string `json:"program_id"`
		ProgramRevision uint64 `json:"program_revision"`
		Target          struct {
			RobotModel string `json:"robot_model"`
		} `json:"target"`
	} `json:"accepted_plan"`
}

// NormalizeNATSStatusFrame accepts the direct JSON published on the exact
// manifest status subject. It reuses the same normalizer as the browser-safe
// WebSocket envelope so both observation routes produce one contract.
func NormalizeNATSStatusFrame(
	context ObservationContext,
	raw []byte,
) (StatusObservation, error) {
	if len(raw) == 0 || len(raw) > MaxStatusFrameBytes {
		return StatusObservation{}, ErrFrameTooLarge
	}
	envelope, err := json.Marshal(webSocketEnvelope{
		Type:    "motion-server-status",
		Subject: context.SourceSubject,
		Status:  append(json.RawMessage(nil), raw...),
	})
	if err != nil {
		return StatusObservation{}, err
	}
	return NormalizeStatusFrame(context, envelope)
}

// NormalizeNATSPlanFrame accepts the direct planner report published on the
// selected Motion Server plan subject.
func NormalizeNATSPlanFrame(
	context ObservationContext,
	raw []byte,
) (PlanObservation, error) {
	if len(raw) == 0 || len(raw) > MaxPlanFrameBytes {
		return PlanObservation{}, ErrFrameTooLarge
	}
	envelope, err := json.Marshal(webSocketEnvelope{
		Type:         "motion-server-plan",
		Subject:      context.SourceSubject,
		ReceivedAtMS: context.ReceivedAt.UnixMilli(),
		Plan:         append(json.RawMessage(nil), raw...),
	})
	if err != nil {
		return PlanObservation{}, err
	}
	return NormalizePlanFrame(context, envelope)
}

func NormalizeStatusFrame(context ObservationContext, raw []byte) (StatusObservation, error) {
	if len(raw) > MaxStatusFrameBytes {
		return StatusObservation{}, ErrFrameTooLarge
	}
	if err := validateContext(context, context.Target.Binding.MotionServerPeer,
		context.Target.Binding.MotionServerStatusSubject); err != nil {
		return StatusObservation{}, err
	}
	envelope, err := decodeEnvelope(raw)
	if err != nil {
		return StatusObservation{}, err
	}
	if envelope.Type != "motion-server-status" || len(envelope.Status) == 0 {
		return StatusObservation{}, errors.New("connected: expected motion-server-status frame")
	}
	if envelope.Subject != "" && envelope.Subject != context.SourceSubject {
		return StatusObservation{}, errors.New("connected: status frame subject does not match source context")
	}
	var status rawStatus
	if err := decodeSingleJSON(envelope.Status, &status); err != nil {
		return StatusObservation{}, fmt.Errorf("connected: decode motion-server status: %w", err)
	}

	reasons := make([]string, 0, 8)
	sourceUpdatedAt, timestampOK := parseSourceTime(status.UpdatedAt, context.ReceivedAt)
	if !timestampOK || !context.ReceivedAt.Before(sourceUpdatedAt.Add(context.FreshnessWindow)) {
		reasons = appendReason(reasons, "status_source_timestamp_missing_or_stale")
		timestampOK = false
	}
	serviceState := strings.TrimSpace(status.ServiceState)
	if serviceState == "" {
		serviceState = strings.TrimSpace(status.State)
	}
	serviceReady := serviceState == "running" || serviceState == "active" || serviceState == "ready"
	serviceName := strings.TrimSpace(status.ServiceName)
	if serviceName == "" {
		serviceName = strings.TrimSpace(status.Name)
	}
	if serviceName != "robot-v4-cartesiand" {
		reasons = appendReason(reasons, "motion_server_service_identity_mismatch")
	}
	if !serviceReady {
		reasons = appendReason(reasons, "motion_server_service_not_ready")
	}
	freshUntil := time.Time{}
	if timestampOK {
		freshUntil = sourceUpdatedAt.Add(context.FreshnessWindow)
	}

	authority := normalizeAuthority(status, context, freshUntil)
	return StatusObservation{
		Schema:             StatusFrameSchema,
		Target:             targetIdentity(context.Target.Discovery),
		SourcePeer:         context.SourcePeer,
		SourceSubject:      context.SourceSubject,
		Sequence:           context.Sequence,
		ReceivedAt:         context.ReceivedAt.UTC(),
		SourceUpdatedAt:    sourceUpdatedAt,
		FreshUntil:         freshUntil,
		Connected:          len(reasons) == 0,
		IdentityCorrelated: true,
		ServiceName:        serviceName,
		ServiceState:       serviceState,
		Plan: RuntimePlanFacts{
			Preloaded:                booleanFact(status.PlanPreloaded),
			LoadedProgramAvailable:   booleanFact(status.LoadedProgramAvailable),
			LoadedProgramPlayable:    booleanFact(status.LoadedProgramPlayable),
			ServerValidated:          booleanFact(status.PlanServerValidated),
			PlayAuthorized:           booleanFact(status.PlanPlayAuthorized),
			PlaybackActive:           booleanFact(status.PlanPlaybackActive),
			LoadedProgramRevision:    status.LoadedProgramRevision,
			PlanRevision:             status.PlanRevision,
			PlanSessionID:            strings.TrimSpace(status.PlanSessionID),
			AcceptedPlanID:           strings.TrimSpace(status.AcceptedPlanID),
			AcceptedProgramDigest:    strings.TrimSpace(status.AcceptedProgramDigest),
			AcceptedTrajectoryDigest: strings.TrimSpace(status.AcceptedTrajectoryDigest),
			AcceptedManifestRevision: strings.TrimSpace(status.AcceptedManifestRevision),
		},
		Output: RuntimeOutputFacts{
			ArmRequested:    booleanFact(status.ServosArmedRequested),
			CurrentZeroHold: booleanFact(status.CurrentOutputZeroHold),
			StaleZeroHold:   booleanFact(status.StaleZeroHold),
			// Motion-server status does not prove these RTCore/output facts.
			ActualServoArmed: BooleanFact{},
			EtherCATEnabled:  BooleanFact{},
			MotionOutput:     BooleanFact{},
			WeldOutput:       BooleanFact{},
		},
		Authority:          authority,
		CorrelationReasons: reasons,
	}, nil
}

func NormalizePlanFrame(context ObservationContext, raw []byte) (PlanObservation, error) {
	if len(raw) > MaxPlanFrameBytes {
		return PlanObservation{}, ErrFrameTooLarge
	}
	if err := validateContext(context, context.Target.Binding.MotionServerPeer,
		context.Target.Binding.MotionServerPlanSubject); err != nil {
		return PlanObservation{}, err
	}
	envelope, err := decodeEnvelope(raw)
	if err != nil {
		return PlanObservation{}, err
	}
	if envelope.Type != "motion-server-plan" || len(envelope.Plan) == 0 {
		return PlanObservation{}, errors.New("connected: expected motion-server-plan frame")
	}
	if envelope.Subject != context.SourceSubject {
		return PlanObservation{}, errors.New("connected: plan frame requires the exact manifest subject")
	}
	envelopeTime := time.UnixMilli(envelope.ReceivedAtMS).UTC()
	if envelope.ReceivedAtMS <= 0 ||
		envelopeTime.After(context.ReceivedAt.Add(time.Second)) ||
		context.ReceivedAt.Sub(envelopeTime) > context.FreshnessWindow {
		return PlanObservation{}, errors.New("connected: plan frame receive time is missing, future, or stale")
	}
	var plan rawPlan
	if err := decodeSingleJSON(envelope.Plan, &plan); err != nil {
		return PlanObservation{}, fmt.Errorf("connected: decode motion-server plan: %w", err)
	}

	reasons := make([]string, 0, 16)
	revision := plan.PlannedPathRevision
	if revision == 0 {
		revision = plan.PlanRevision
	}
	planID := strings.TrimSpace(plan.PlanID)
	if planID == "" {
		planID = strings.TrimSpace(plan.PlanSessionID)
	}
	programRevision := plan.AcceptedProgramRevision
	if programRevision == 0 {
		programRevision = plan.ProgramRevision
	}
	if plan.AcceptedPlan != nil {
		if strings.TrimSpace(plan.AcceptedPlan.PlanID) != "" {
			planID = strings.TrimSpace(plan.AcceptedPlan.PlanID)
		}
		if plan.AcceptedPlan.PlanRevision > 0 {
			revision = plan.AcceptedPlan.PlanRevision
		}
		if strings.TrimSpace(plan.AcceptedPlan.ProgramID) != "" {
			plan.ProgramID = strings.TrimSpace(plan.AcceptedPlan.ProgramID)
		}
		if plan.AcceptedPlan.ProgramRevision > 0 {
			programRevision = plan.AcceptedPlan.ProgramRevision
		}
		if strings.TrimSpace(plan.AcceptedPlan.Target.RobotModel) != "" {
			plan.RobotModel = strings.TrimSpace(
				plan.AcceptedPlan.Target.RobotModel,
			)
		}
	}
	identity := PlanIdentity{
		PlanID:          planID,
		PlanRevision:    revision,
		ProgramID:       strings.TrimSpace(plan.ProgramID),
		ProgramRevision: programRevision,
		ProgramHash:     strings.TrimSpace(plan.ProgramHash),
		RobotModel:      strings.TrimSpace(plan.RobotModel),
	}
	if identity.PlanID == "" {
		reasons = appendReason(reasons, "plan_id_unavailable")
	}
	if identity.PlanRevision == 0 {
		reasons = appendReason(reasons, "plan_revision_unavailable")
	}
	if identity.ProgramID == "" {
		reasons = appendReason(reasons, "program_id_unavailable")
	}
	if identity.ProgramRevision == 0 {
		reasons = appendReason(reasons, "program_revision_unavailable")
	}
	if plan.ProgramRevision > 0 && plan.AcceptedProgramRevision > 0 &&
		plan.ProgramRevision != plan.AcceptedProgramRevision {
		reasons = appendReason(reasons, "accepted_program_revision_mismatch")
	}
	if identity.ProgramHash == "" {
		reasons = appendReason(reasons, "program_hash_unavailable")
	}
	if !robotmodel.IsArm(identity.RobotModel) {
		reasons = appendReason(reasons, "robot_model_unavailable_or_unsupported")
	}
	if strings.TrimSpace(plan.PlannerOwner) != "motion-server" {
		reasons = appendReason(reasons, "planner_owner_mismatch")
	}
	if plan.Accepted == nil || !*plan.Accepted {
		reasons = appendReason(reasons, "explicit_plan_acceptance_unavailable")
	}

	safety := PlanSafetyFacts{
		Collision:   normalizeCollision(plan),
		Singularity: normalizeSingularity(plan),
		Limits:      normalizeLimits(plan),
		Runtime:     normalizeRuntime(plan),
	}
	if !safety.Collision.Available || !safety.Collision.Clear {
		reasons = appendReason(reasons, "collision_evidence_unavailable_or_blocked")
	}
	if !safety.Singularity.Available || !safety.Singularity.Clear {
		reasons = appendReason(reasons, "singularity_evidence_unavailable_or_blocked")
	}
	if !safety.Limits.Available || !safety.Limits.Clear {
		reasons = appendReason(reasons, "limit_evidence_unavailable_or_blocked")
	}
	if !safety.Runtime.Available || !safety.Runtime.Clear {
		reasons = appendReason(reasons, "runtime_evidence_unavailable_or_blocked")
	}
	accepted := booleanFact(plan.Accepted)
	return PlanObservation{
		Schema:             PlanFrameSchema,
		Target:             targetIdentity(context.Target.Discovery),
		SourcePeer:         context.SourcePeer,
		SourceSubject:      context.SourceSubject,
		Sequence:           context.Sequence,
		ReceivedAt:         envelopeTime,
		FreshUntil:         envelopeTime.Add(context.FreshnessWindow),
		Identity:           identity,
		PlannerOwner:       strings.TrimSpace(plan.PlannerOwner),
		PlannerBackend:     strings.TrimSpace(plan.PlannerBackend),
		ValidationState:    strings.TrimSpace(plan.ValidationState),
		Accepted:           accepted,
		PlayEnabled:        booleanFact(plan.PlayEnabled),
		RuntimePreload:     booleanFact(plan.RuntimePreloadEnabled),
		SampleCount:        plan.PlannedSampleCount,
		Safety:             safety,
		IdentityCorrelated: len(reasons) == 0,
		CorrelationReasons: reasons,
	}, nil
}

type ReadOnlyStreams struct {
	mu             sync.RWMutex
	statusSequence uint64
	planSequence   uint64
	status         *StatusObservation
	plan           *PlanObservation
}

// AcceptStatus and AcceptPlan retain only the latest normalized frame. This
// mirrors the Robot V4 WebSocket backpressure policy: old observational
// frames are replaced, never queued without a bound.
func (streams *ReadOnlyStreams) AcceptStatus(observation StatusObservation) error {
	streams.mu.Lock()
	defer streams.mu.Unlock()
	if observation.Sequence == 0 || observation.Sequence <= streams.statusSequence {
		return ErrStaleFrameSequence
	}
	copy := cloneStatus(observation)
	streams.statusSequence = observation.Sequence
	streams.status = &copy
	return nil
}

func (streams *ReadOnlyStreams) AcceptPlan(observation PlanObservation) error {
	streams.mu.Lock()
	defer streams.mu.Unlock()
	if observation.Sequence == 0 || observation.Sequence <= streams.planSequence {
		return ErrStaleFrameSequence
	}
	copy := clonePlan(observation)
	streams.planSequence = observation.Sequence
	streams.plan = &copy
	return nil
}

func (streams *ReadOnlyStreams) Latest() (*StatusObservation, *PlanObservation) {
	streams.mu.RLock()
	defer streams.mu.RUnlock()
	var status *StatusObservation
	var plan *PlanObservation
	if streams.status != nil {
		copy := cloneStatus(*streams.status)
		status = &copy
	}
	if streams.plan != nil {
		copy := clonePlan(*streams.plan)
		plan = &copy
	}
	return status, plan
}

func validateContext(context ObservationContext, expectedPeer, expectedSubject string) error {
	if context.Sequence == 0 || context.ReceivedAt.IsZero() {
		return errors.New("connected: observation sequence and receive time are required")
	}
	if context.FreshnessWindow <= 0 || context.FreshnessWindow > MaxObservationFreshness {
		return errors.New("connected: observation freshness window is invalid")
	}
	if context.Target.Discovery.Schema != "offline-programming.connected-target-discovery.v1" ||
		context.SourcePeer == "" || context.SourcePeer != expectedPeer ||
		context.SourceSubject == "" || context.SourceSubject != expectedSubject {
		return errors.New("connected: observation source is not bound to the selected manifest")
	}
	return nil
}

func decodeEnvelope(raw []byte) (webSocketEnvelope, error) {
	var envelope webSocketEnvelope
	if err := decodeSingleJSON(raw, &envelope); err != nil {
		return webSocketEnvelope{}, fmt.Errorf("connected: decode WebSocket frame: %w", err)
	}
	return envelope, nil
}

func decodeSingleJSON(raw []byte, destination any) error {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	if err := decoder.Decode(destination); err != nil {
		return err
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return errors.New("multiple JSON values")
	}
	return nil
}

func parseSourceTime(value string, receivedAt time.Time) (time.Time, bool) {
	parsed, err := time.Parse(time.RFC3339Nano, strings.TrimSpace(value))
	if err != nil || parsed.After(receivedAt.Add(time.Second)) {
		return time.Time{}, false
	}
	return parsed.UTC(), true
}

func normalizeAuthority(status rawStatus, context ObservationContext, statusFreshUntil time.Time) AuthorityObservation {
	present := !statusFreshUntil.IsZero() &&
		status.LeaderLeaseFresh != nil &&
		*status.LeaderLeaseFresh &&
		strings.TrimSpace(status.LeaderID) != "" &&
		status.LeaderID != "none" &&
		strings.TrimSpace(status.LeaderBootID) != "" &&
		status.LeaderBootID != "none" &&
		strings.TrimSpace(status.LeaderLeaseID) != "" &&
		status.LeaderLeaseID != "none" &&
		status.LeaderFenceEpoch > 0 &&
		status.LeaderCampaignGeneration > 0 &&
		status.LeaderExpiresInMS > 0 &&
		(status.LeaderTransitionPending == nil || !*status.LeaderTransitionPending)
	if !present {
		return AuthorityObservation{}
	}
	leaseFreshUntil := context.ReceivedAt.Add(time.Duration(status.LeaderExpiresInMS) * time.Millisecond)
	if !statusFreshUntil.IsZero() && statusFreshUntil.Before(leaseFreshUntil) {
		leaseFreshUntil = statusFreshUntil
	}
	return AuthorityObservation{
		Known:              true,
		ControllerID:       strings.TrimSpace(status.LeaderID),
		ControllerBootID:   strings.TrimSpace(status.LeaderBootID),
		LeaseID:            strings.TrimSpace(status.LeaderLeaseID),
		FenceEpoch:         status.LeaderFenceEpoch,
		CampaignGeneration: status.LeaderCampaignGeneration,
		LeaseFresh:         true,
		TransitionPending:  false,
		ExpiresIn:          time.Duration(status.LeaderExpiresInMS) * time.Millisecond,
		FreshUntil:         leaseFreshUntil,
	}
}

func normalizeCollision(plan rawPlan) SafetyEvidence {
	available := plan.CollisionPoints != nil &&
		plan.CollisionGateClear != nil &&
		strings.TrimSpace(plan.CollisionCheckState) != ""
	count := sliceLength(plan.CollisionPoints)
	return SafetyEvidence{
		Available: available,
		Clear: available && *plan.CollisionGateClear &&
			plan.CollisionCheckState == "tesseract_discrete_contact_checked" &&
			count == 0,
		State:      strings.TrimSpace(plan.CollisionCheckState),
		PointCount: count,
	}
}

func normalizeSingularity(plan rawPlan) SafetyEvidence {
	available := plan.SingularityPoints != nil &&
		plan.SingularityAnalysisAvailable != nil &&
		plan.SingularityGateClear != nil &&
		strings.TrimSpace(plan.SingularityCheckState) != ""
	count := sliceLength(plan.SingularityPoints)
	return SafetyEvidence{
		Available: available,
		Clear: available && *plan.SingularityAnalysisAvailable &&
			*plan.SingularityGateClear &&
			plan.SingularityCheckState == "jacobian_svd_checked",
		State:      strings.TrimSpace(plan.SingularityCheckState),
		PointCount: count,
	}
}

func normalizeLimits(plan rawPlan) SafetyEvidence {
	available := plan.UnreachablePoints != nil && strings.TrimSpace(plan.LimitCheckState) != ""
	count := sliceLength(plan.UnreachablePoints)
	return SafetyEvidence{
		Available:  available,
		Clear:      available && plan.LimitCheckState == "clear" && count == 0,
		State:      strings.TrimSpace(plan.LimitCheckState),
		PointCount: count,
	}
}

func normalizeRuntime(plan rawPlan) SafetyEvidence {
	available := plan.PlayEnabled != nil && plan.RuntimePreloadEnabled != nil
	clear := available && *plan.PlayEnabled && *plan.RuntimePreloadEnabled
	state := "unavailable"
	if available {
		state = "blocked"
		if clear {
			state = "clear"
		}
	}
	return SafetyEvidence{Available: available, Clear: clear, State: state}
}

func sliceLength(value *[]json.RawMessage) int {
	if value == nil {
		return 0
	}
	return len(*value)
}

func booleanFact(value *bool) BooleanFact {
	if value == nil {
		return BooleanFact{}
	}
	return BooleanFact{Known: true, Value: *value}
}

func appendReason(reasons []string, reason string) []string {
	if reason == "" || len(reasons) >= MaxCorrelationReasons {
		return reasons
	}
	for _, existing := range reasons {
		if existing == reason {
			return reasons
		}
	}
	return append(reasons, reason)
}

func cloneStatus(observation StatusObservation) StatusObservation {
	observation.CorrelationReasons = append([]string(nil), observation.CorrelationReasons...)
	if observation.RTCoreOutput != nil {
		copy := *observation.RTCoreOutput
		copy.Reasons = append([]string(nil), copy.Reasons...)
		observation.RTCoreOutput = &copy
	}
	return observation
}

func clonePlan(observation PlanObservation) PlanObservation {
	observation.CorrelationReasons = append([]string(nil), observation.CorrelationReasons...)
	return observation
}
