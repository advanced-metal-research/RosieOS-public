package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/connected"
	"rosieos/offline-programming/v1/internal/execution"
	"rosieos/offline-programming/v1/internal/targets"
)

const connectedTargetsSchema = "offline-programming.connected-targets.v1"
const connectedStatusSchema = "offline-programming.connected-target-status.v1"

type ConnectedRegistry struct {
	mu      sync.RWMutex
	targets map[string]*connectedTarget
	issues  []connectedManifestIssue
}

type connectedTarget struct {
	manifest            targets.ConnectedManifestTarget
	streams             connected.ReadOnlyStreams
	outputMu            sync.Mutex
	motionStatus        *connected.StatusObservation
	rtcoreOutput        *connected.RTCoreOutputObservation
	fusedStatusSequence uint64
	statusHistory       []connected.StatusObservation
	planHistory         []connected.PlanObservation
}

type connectedManifestIssue struct {
	ManifestRef string `json:"manifest_ref"`
	Code        string `json:"code"`
	Reason      string `json:"reason"`
}

type connectedDiscoveryEnvelope struct {
	Schema  string                                       `json:"schema"`
	Targets []targets.ConnectedTargetDiscoveryDescriptor `json:"targets"`
	Issues  []connectedManifestIssue                     `json:"issues"`
}

type publicConnectedStatus struct {
	Schema            string                                     `json:"schema"`
	Target            targets.ConnectedTargetDiscoveryDescriptor `json:"target"`
	State             string                                     `json:"state"`
	ObservedAt        time.Time                                  `json:"observed_at"`
	StatusAvailable   bool                                       `json:"status_available"`
	PlanAvailable     bool                                       `json:"plan_available"`
	Connected         bool                                       `json:"connected"`
	ServiceName       string                                     `json:"service_name,omitempty"`
	ServiceState      string                                     `json:"service_state,omitempty"`
	Plan              *connected.RuntimePlanFacts                `json:"plan,omitempty"`
	Output            *connected.RuntimeOutputFacts              `json:"output,omitempty"`
	Authority         *connected.AuthorityObservation            `json:"authority,omitempty"`
	PlanIdentity      *connected.PlanIdentity                    `json:"plan_identity,omitempty"`
	PlanSafety        *connected.PlanSafetyFacts                 `json:"plan_safety,omitempty"`
	StatusObservation *connected.StatusObservation               `json:"status_observation,omitempty"`
	PlanObservation   *connected.PlanObservation                 `json:"plan_observation,omitempty"`
	Reasons           []string                                   `json:"reasons"`
}

func LoadConnectedRegistry(repoRoot string) *ConnectedRegistry {
	registry := &ConnectedRegistry{
		targets: make(map[string]*connectedTarget),
		issues:  []connectedManifestIssue{},
	}
	pattern := filepath.Join(repoRoot, "rt-core", "config", "cells", "*.json")
	paths, err := filepath.Glob(pattern)
	if err != nil {
		registry.issues = append(registry.issues, connectedManifestIssue{
			ManifestRef: "rt-core/config/cells",
			Code:        "manifest_scan_failed",
			Reason:      "connected manifest directory is unavailable",
		})
		return registry
	}
	sort.Strings(paths)
	for _, path := range paths {
		ref, err := filepath.Rel(repoRoot, path)
		if err != nil {
			continue
		}
		ref = filepath.ToSlash(ref)
		raw, err := os.ReadFile(path)
		if err != nil {
			registry.addIssue(ref, "manifest_read_failed", "manifest could not be read")
			continue
		}
		// The Steam Deck wake inventory shares the cells directory but is not a
		// cell; it says so in its schema, so it is passed over by schema rather
		// than by file name.
		var schema struct {
			Version string `json:"schema_version"`
		}
		if json.Unmarshal(raw, &schema) == nil && schema.Version == "robot_v4_steamdeck_wake_inventory_v1" {
			continue
		}
		manifest, err := targets.ParseConnectedTargetManifest(ref, raw, repoRoot)
		if err != nil {
			registry.addIssue(ref, "manifest_invalid", "manifest failed public-safe connected-target validation")
			continue
		}
		registry.targets[manifest.Discovery.ProfileID] = &connectedTarget{manifest: manifest}
	}
	return registry
}

func (registry *ConnectedRegistry) Discovery() connectedDiscoveryEnvelope {
	envelope := connectedDiscoveryEnvelope{
		Schema:  connectedTargetsSchema,
		Targets: []targets.ConnectedTargetDiscoveryDescriptor{},
		Issues:  []connectedManifestIssue{},
	}
	if registry == nil {
		envelope.Issues = append(envelope.Issues, connectedManifestIssue{
			ManifestRef: "rt-core/config/cells",
			Code:        "registry_unavailable",
			Reason:      "connected target registry is unavailable",
		})
		return envelope
	}
	registry.mu.RLock()
	defer registry.mu.RUnlock()
	profiles := make([]string, 0, len(registry.targets))
	for profile := range registry.targets {
		profiles = append(profiles, profile)
	}
	sort.Strings(profiles)
	for _, profile := range profiles {
		envelope.Targets = append(envelope.Targets, registry.targets[profile].manifest.Discovery)
	}
	envelope.Issues = append(envelope.Issues, registry.issues...)
	return envelope
}

func (registry *ConnectedRegistry) Status(
	profileID string,
	now time.Time,
) (publicConnectedStatus, bool) {
	target, ok := registry.target(profileID)
	if !ok {
		return publicConnectedStatus{}, false
	}
	status, plan := target.streams.Latest()
	reasons := []string{}
	statusAvailable := status != nil && now.Before(status.FreshUntil)
	planAvailable := plan != nil && now.Before(plan.FreshUntil)
	if !statusAvailable {
		reasons = append(reasons, "fresh_read_only_status_unavailable")
	}
	if !planAvailable {
		reasons = append(reasons, "fresh_read_only_plan_unavailable")
	}
	result := publicConnectedStatus{
		Schema:          connectedStatusSchema,
		Target:          target.manifest.Discovery,
		State:           "unavailable",
		ObservedAt:      now.UTC(),
		StatusAvailable: statusAvailable,
		PlanAvailable:   planAvailable,
		Connected:       statusAvailable && status.Connected,
		Reasons:         reasons,
	}
	if statusAvailable {
		result.State = "read_only"
		result.ServiceName = status.ServiceName
		result.ServiceState = status.ServiceState
		planFacts := status.Plan
		outputFacts := status.Output
		authority := status.Authority
		result.Plan = &planFacts
		result.Output = &outputFacts
		result.Authority = &authority
		result.StatusObservation = status
	}
	if planAvailable {
		identity := plan.Identity
		safety := plan.Safety
		result.PlanIdentity = &identity
		result.PlanSafety = &safety
		result.PlanObservation = plan
	}
	return result, true
}

// AcceptNATSStatus and AcceptNATSPlan are the only network-observation write
// boundary. They bind direct NATS JSON to the exact manifest peer and subject
// before it can enter the browser-safe cache or execution verifier.
func (registry *ConnectedRegistry) AcceptNATSStatus(
	profileID string,
	raw []byte,
	sequence uint64,
	receivedAt time.Time,
) error {
	target, ok := registry.target(profileID)
	if !ok {
		return fmt.Errorf("connected target %q was not found", profileID)
	}
	freshness := targetFreshness(target.manifest)
	observation, err := connected.NormalizeNATSStatusFrame(
		connected.ObservationContext{
			Target:          target.manifest,
			SourcePeer:      target.manifest.Binding.MotionServerPeer,
			SourceSubject:   target.manifest.Binding.MotionServerStatusSubject,
			Sequence:        sequence,
			ReceivedAt:      receivedAt.UTC(),
			FreshnessWindow: freshness,
		},
		raw,
	)
	if err != nil {
		return err
	}
	target.outputMu.Lock()
	target.motionStatus = &observation
	err = target.acceptFusedStatusLocked(receivedAt.UTC())
	target.outputMu.Unlock()
	return err
}

func (registry *ConnectedRegistry) AcceptNATSRTCoreStatus(
	profileID string,
	raw []byte,
	sequence uint64,
	receivedAt time.Time,
) error {
	target, ok := registry.target(profileID)
	if !ok {
		return fmt.Errorf("connected target %q was not found", profileID)
	}
	observation, err := connected.NormalizeNATSRTCoreStatusFrame(
		connected.ObservationContext{
			Target:          target.manifest,
			SourcePeer:      target.manifest.Binding.RealtimeCorePeer,
			SourceSubject:   target.manifest.Binding.RealtimeCoreStatusSubject,
			Sequence:        sequence,
			ReceivedAt:      receivedAt.UTC(),
			FreshnessWindow: targetFreshness(target.manifest),
		},
		raw,
	)
	if err != nil {
		return err
	}
	target.outputMu.Lock()
	target.rtcoreOutput = &observation
	err = target.acceptFusedStatusLocked(receivedAt.UTC())
	target.outputMu.Unlock()
	return err
}

func (target *connectedTarget) acceptFusedStatusLocked(now time.Time) error {
	if target.motionStatus == nil {
		return nil
	}
	status := *target.motionStatus
	status.CorrelationReasons = append(
		[]string(nil),
		target.motionStatus.CorrelationReasons...,
	)
	target.fusedStatusSequence++
	status.Sequence = target.fusedStatusSequence
	if output := target.rtcoreOutput; output != nil &&
		now.Before(output.FreshUntil) &&
		output.Target == status.Target {
		copy := *output
		copy.Reasons = append([]string(nil), output.Reasons...)
		status.RTCoreOutput = &copy
		status.Output.ArmRequested = output.Output.ArmRequested
		status.Output.ActualServoArmed = output.Output.ActualServoArmed
		status.Output.EtherCATEnabled = output.Output.EtherCATEnabled
		status.Output.MotionOutput = output.Output.MotionOutput
		status.Output.WeldOutput = output.Output.WeldOutput
		if output.FreshUntil.Before(status.FreshUntil) {
			status.FreshUntil = output.FreshUntil
		}
		for _, reason := range output.Reasons {
			status.CorrelationReasons = appendConnectedReason(
				status.CorrelationReasons,
				reason,
			)
		}
	} else {
		status.CorrelationReasons = appendConnectedReason(
			status.CorrelationReasons,
			"fresh_rtcore_output_observation_unavailable",
		)
	}
	if err := target.streams.AcceptStatus(status); err != nil {
		return err
	}
	target.statusHistory = appendStatusHistory(target.statusHistory, status)
	return nil
}

func appendConnectedReason(reasons []string, reason string) []string {
	for _, existing := range reasons {
		if existing == reason {
			return reasons
		}
	}
	return append(reasons, reason)
}

const connectedEvidenceHistoryLimit = 8

func appendStatusHistory(
	history []connected.StatusObservation,
	observation connected.StatusObservation,
) []connected.StatusObservation {
	observation.CorrelationReasons = append(
		[]string(nil),
		observation.CorrelationReasons...,
	)
	if observation.RTCoreOutput != nil {
		copy := *observation.RTCoreOutput
		copy.Reasons = append([]string(nil), copy.Reasons...)
		observation.RTCoreOutput = &copy
	}
	history = append(history, observation)
	if len(history) > connectedEvidenceHistoryLimit {
		history = append(
			[]connected.StatusObservation(nil),
			history[len(history)-connectedEvidenceHistoryLimit:]...,
		)
	}
	return history
}

func appendPlanHistory(
	history []connected.PlanObservation,
	observation connected.PlanObservation,
) []connected.PlanObservation {
	observation.CorrelationReasons = append(
		[]string(nil),
		observation.CorrelationReasons...,
	)
	history = append(history, observation)
	if len(history) > connectedEvidenceHistoryLimit {
		history = append(
			[]connected.PlanObservation(nil),
			history[len(history)-connectedEvidenceHistoryLimit:]...,
		)
	}
	return history
}

func containsStatusObservation(
	history []connected.StatusObservation,
	wanted connected.StatusObservation,
) bool {
	for _, observation := range history {
		if reflect.DeepEqual(observation, wanted) {
			return true
		}
	}
	return false
}

func containsPlanObservation(
	history []connected.PlanObservation,
	wanted connected.PlanObservation,
) bool {
	for _, observation := range history {
		if reflect.DeepEqual(observation, wanted) {
			return true
		}
	}
	return false
}

func (registry *ConnectedRegistry) AcceptNATSPlan(
	profileID string,
	raw []byte,
	sequence uint64,
	receivedAt time.Time,
) error {
	target, ok := registry.target(profileID)
	if !ok {
		return fmt.Errorf("connected target %q was not found", profileID)
	}
	freshness := targetFreshness(target.manifest)
	observation, err := connected.NormalizeNATSPlanFrame(
		connected.ObservationContext{
			Target:          target.manifest,
			SourcePeer:      target.manifest.Binding.MotionServerPeer,
			SourceSubject:   target.manifest.Binding.MotionServerPlanSubject,
			Sequence:        sequence,
			ReceivedAt:      receivedAt.UTC(),
			FreshnessWindow: freshness,
		},
		raw,
	)
	if err != nil {
		return err
	}
	if err := target.streams.AcceptPlan(observation); err != nil {
		return err
	}
	target.outputMu.Lock()
	target.planHistory = appendPlanHistory(target.planHistory, observation)
	target.outputMu.Unlock()
	return nil
}

// Verify implements execution.AuthoritativeEvidenceVerifier. Equality with
// the server-owned cache is required; posting a valid-looking browser object
// cannot make it authoritative.
func (registry *ConnectedRegistry) Verify(
	_ context.Context,
	input execution.ExecutionReviewInput,
) error {
	target, ok := registry.target(input.Identity.Target.ProfileID)
	if !ok {
		return errors.New("connected: selected execution target is unavailable")
	}
	target.outputMu.Lock()
	statusKnown := containsStatusObservation(
		target.statusHistory,
		input.Status,
	)
	planKnown := containsPlanObservation(
		target.planHistory,
		input.RuntimePlan,
	)
	target.outputMu.Unlock()
	if !statusKnown || !planKnown {
		return errors.New("connected: review evidence does not equal the server-owned observation cache")
	}
	if target.manifest.Binding.ManifestRevision !=
		input.Identity.Target.ManifestRevision {
		return errors.New("connected: review manifest revision is stale")
	}
	return nil
}

// BindLatestExecutionEvidence replaces browser-supplied observation JSON with
// the latest server-owned normalized target evidence. The exact result is
// retained in a bounded history, so a concurrent NATS refresh cannot
// invalidate an in-flight HTTP request.
func (registry *ConnectedRegistry) BindLatestExecutionEvidence(
	input *execution.ExecutionReviewInput,
) error {
	if input == nil {
		return errors.New("connected: execution review input is unavailable")
	}
	target, ok := registry.target(input.Identity.Target.ProfileID)
	if !ok {
		return errors.New("connected: selected execution target is unavailable")
	}
	status, plan := target.streams.Latest()
	if status == nil || plan == nil {
		return errors.New(
			"connected: fresh target status and plan evidence are unavailable",
		)
	}
	input.Status = *status
	input.RuntimePlan = *plan
	return nil
}

// VerifyActive implements execution.ActiveSessionObserver. It requires fresh,
// independently received Motion Server and RTCore evidence for the exact
// session, accepted plan, manifest revision, controller lease, and safe live
// output state.
func (registry *ConnectedRegistry) VerifyActive(
	ctx context.Context,
	session execution.ExecutionSession,
	now time.Time,
) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	target, ok := registry.target(session.Identity.Target.ProfileID)
	if !ok {
		return errors.New("connected: active execution target is unavailable")
	}
	if target.manifest.Binding.ManifestRevision !=
		session.Identity.Target.ManifestRevision {
		return errors.New("connected: active execution manifest changed")
	}
	status, plan := target.streams.Latest()
	if status == nil || plan == nil ||
		now.IsZero() ||
		!now.Before(status.FreshUntil) ||
		!now.Before(plan.FreshUntil) ||
		status.ReceivedAt.IsZero() ||
		now.Before(status.ReceivedAt) ||
		now.Sub(status.ReceivedAt) > 3*time.Second ||
		!status.Connected ||
		!status.IdentityCorrelated ||
		!plan.IdentityCorrelated {
		return errors.New("connected: active execution evidence is stale or disconnected")
	}
	expectedTarget := session.Identity.Target.ConnectedIdentity()
	if status.Target != expectedTarget ||
		plan.Target != expectedTarget ||
		status.RTCoreOutput == nil ||
		status.RTCoreOutput.Target != expectedTarget ||
		!now.Before(status.RTCoreOutput.FreshUntil) ||
		status.RTCoreOutput.ReceivedAt.IsZero() ||
		now.Before(status.RTCoreOutput.ReceivedAt) ||
		now.Sub(status.RTCoreOutput.ReceivedAt) > 3*time.Second ||
		len(status.RTCoreOutput.Reasons) != 0 {
		return errors.New("connected: active RTCore output proof is unavailable")
	}
	runtime := status.Plan
	identity := session.Identity
	if !runtime.Preloaded.Known || !runtime.Preloaded.Value ||
		!runtime.LoadedProgramAvailable.Known ||
		!runtime.LoadedProgramAvailable.Value ||
		!runtime.LoadedProgramPlayable.Known ||
		!runtime.LoadedProgramPlayable.Value ||
		!runtime.ServerValidated.Known || !runtime.ServerValidated.Value ||
		!runtime.PlayAuthorized.Known || !runtime.PlayAuthorized.Value ||
		!runtime.PlaybackActive.Known || !runtime.PlaybackActive.Value ||
		// Robot V4 names this field "loaded_program_revision", but the
		// Motion Server assigns it the opaque plan-request revision used as
		// the accepted plan revision. The editable program revision remains
		// separately proved by the plan observation below.
		runtime.LoadedProgramRevision != identity.PlanRevision ||
		runtime.PlanRevision != identity.PlanRevision ||
		runtime.PlanSessionID != identity.PlanID ||
		runtime.AcceptedPlanID != identity.PlanID ||
		runtime.AcceptedProgramDigest != identity.ProgramDigest ||
		runtime.AcceptedTrajectoryDigest != identity.TrajectoryDigest ||
		runtime.AcceptedManifestRevision != identity.Target.ManifestRevision {
		return errors.New("connected: active Motion Server plan identity changed")
	}
	if plan.Identity.PlanID != identity.PlanID ||
		plan.Identity.PlanRevision != identity.PlanRevision ||
		plan.Identity.ProgramID != identity.ProgramID ||
		plan.Identity.ProgramRevision != identity.ProgramRevision ||
		plan.Identity.ProgramHash != identity.ProgramHash ||
		plan.Identity.RobotModel != string(identity.RobotModel) ||
		!plan.Accepted.Known || !plan.Accepted.Value ||
		!plan.PlayEnabled.Known || !plan.PlayEnabled.Value ||
		!plan.RuntimePreload.Known || !plan.RuntimePreload.Value {
		return errors.New("connected: active accepted-plan observation changed")
	}
	authority := status.Authority
	if !authority.Known ||
		!authority.LeaseFresh ||
		authority.TransitionPending ||
		!now.Before(authority.FreshUntil) ||
		authority.ControllerID != session.Controller.ControllerID ||
		authority.ControllerBootID !=
			session.Controller.ControllerBootID ||
		authority.LeaseID != session.Lease.Credential.LeaseID ||
		authority.FenceEpoch != session.Lease.Credential.FenceEpoch ||
		authority.CampaignGeneration != session.CampaignGeneration {
		return errors.New("connected: active controller lease or fence changed")
	}
	output := status.Output
	if !output.ArmRequested.Known || !output.ArmRequested.Value ||
		!output.ActualServoArmed.Known || !output.ActualServoArmed.Value ||
		!output.EtherCATEnabled.Known || !output.EtherCATEnabled.Value ||
		!output.MotionOutput.Known || !output.MotionOutput.Value ||
		!output.WeldOutput.Known || output.WeldOutput.Value ||
		!output.StaleZeroHold.Known || output.StaleZeroHold.Value ||
		!output.CurrentZeroHold.Known || output.CurrentZeroHold.Value {
		return errors.New("connected: active RTCore output state is unsafe")
	}
	return nil
}

func targetFreshness(target targets.ConnectedManifestTarget) time.Duration {
	freshness := time.Duration(
		target.Discovery.PeerStaleAfterSeconds,
	) * time.Second
	if freshness <= 0 || freshness > connected.MaxObservationFreshness {
		return connected.MaxObservationFreshness
	}
	return freshness
}

func (registry *ConnectedRegistry) Capabilities(
	profileID string,
	now time.Time,
) (connected.CapabilityReport, bool) {
	target, ok := registry.target(profileID)
	if !ok {
		return connected.CapabilityReport{}, false
	}
	status, plan := target.streams.Latest()
	report := connected.ReportCapabilities(
		now,
		&target.manifest.Discovery,
		status,
		plan,
		nil,
	)
	return report, true
}

// PlanningTarget is the server-only targetplanning.TargetSource
// implementation. The returned binding includes private subjects and daemon
// service refs and must never be serialized to the browser.
func (registry *ConnectedRegistry) PlanningTarget(
	profileID string,
) (targets.ConnectedManifestTarget, bool) {
	target, ok := registry.target(profileID)
	if !ok {
		return targets.ConnectedManifestTarget{}, false
	}
	return target.manifest, true
}

func (registry *ConnectedRegistry) target(profileID string) (*connectedTarget, bool) {
	if registry == nil {
		return nil, false
	}
	registry.mu.RLock()
	defer registry.mu.RUnlock()
	target, ok := registry.targets[strings.TrimSpace(profileID)]
	return target, ok
}

func (registry *ConnectedRegistry) addIssue(ref, code, reason string) {
	registry.issues = append(registry.issues, connectedManifestIssue{
		ManifestRef: ref,
		Code:        code,
		Reason:      boundedPublicReason(reason),
	})
}

func boundedPublicReason(value string) string {
	value = strings.TrimSpace(value)
	if value == "" {
		return "connected manifest unavailable"
	}
	if len(value) > 240 {
		return fmt.Sprintf("%s…", value[:239])
	}
	return value
}
