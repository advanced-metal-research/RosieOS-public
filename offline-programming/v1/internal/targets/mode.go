package targets

import (
	"errors"
	"strings"
	"time"
)

type ModePhase string

const (
	PhaseOffline        ModePhase = "offline"
	PhaseSimulatorReady ModePhase = "simulator_ready"
	PhaseConnecting     ModePhase = "connecting"
	PhaseConnected      ModePhase = "connected"
	PhaseBlocked        ModePhase = "blocked"
)

type SimulatorSafetyProof struct {
	ProfileID           string    `json:"profile_id"`
	StatusSequence      uint64    `json:"status_sequence"`
	ObservedAt          time.Time `json:"observed_at"`
	FreshUntil          time.Time `json:"fresh_until"`
	NoRealMovement      bool      `json:"no_real_movement"`
	EtherCATEnabled     bool      `json:"ethercat_enabled"`
	MotionOutputEnabled bool      `json:"motion_output_enabled"`
}

type TargetObservation struct {
	ProfileID      string    `json:"profile_id"`
	ManifestRef    string    `json:"manifest_ref"`
	StatusSequence uint64    `json:"status_sequence"`
	ObservedAt     time.Time `json:"observed_at"`
	FreshUntil     time.Time `json:"fresh_until"`
	Connected      bool      `json:"connected"`
}

type ModeState struct {
	Revision    uint64             `json:"revision"`
	Mode        TargetKind         `json:"mode"`
	Phase       ModePhase          `json:"phase"`
	Target      TargetDescriptor   `json:"target"`
	Observation *TargetObservation `json:"observation,omitempty"`
	BlockedBy   []string           `json:"blocked_by"`
	ChangedAt   time.Time          `json:"changed_at"`
}

type ModeMachine struct {
	state ModeState
}

func NewModeMachine(now time.Time) *ModeMachine {
	return &ModeMachine{state: ModeState{
		Revision:  1,
		Mode:      TargetOfflinePreview,
		Phase:     PhaseOffline,
		Target:    OfflineDescriptor(),
		BlockedBy: []string{},
		ChangedAt: now.UTC(),
	}}
}

func (machine *ModeMachine) State() ModeState {
	state := machine.state
	state.Target.Axes = append([]AxisDescriptor(nil), state.Target.Axes...)
	state.BlockedBy = append([]string(nil), state.BlockedBy...)
	if state.Observation != nil {
		observation := *state.Observation
		state.Observation = &observation
	}
	return state
}

func (machine *ModeMachine) Select(
	now time.Time,
	target TargetDescriptor,
	simulatorProof *SimulatorSafetyProof,
) (ModeState, error) {
	if machine == nil {
		return ModeState{}, errors.New("targets: nil mode machine")
	}
	if now.IsZero() {
		return machine.failClosed(now, "selection_time_missing"), errors.New("targets: selection time is required")
	}
	switch target.Kind {
	case TargetOfflinePreview:
		machine.set(now, TargetOfflinePreview, PhaseOffline, OfflineDescriptor(), nil, nil)
		return machine.State(), nil
	case TargetLocalSimulator:
		if target.Schema != TargetDescriptorSchema ||
			!target.Capabilities.Simulation ||
			target.Capabilities.RealExecution ||
			simulatorProof == nil ||
			simulatorProof.ProfileID != target.ProfileID ||
			simulatorProof.StatusSequence == 0 ||
			simulatorProof.ObservedAt.IsZero() ||
			now.Before(simulatorProof.ObservedAt) ||
			!now.Before(simulatorProof.FreshUntil) ||
			!simulatorProof.NoRealMovement ||
			simulatorProof.EtherCATEnabled ||
			simulatorProof.MotionOutputEnabled {
			return machine.failClosed(now, "local_simulator_safety_proof_invalid"),
				errors.New("targets: local simulator safety proof is missing, stale, or unsafe")
		}
		machine.set(now, TargetLocalSimulator, PhaseSimulatorReady, target, nil, nil)
		return machine.State(), nil
	case TargetRealCell:
		if target.Schema != TargetDescriptorSchema ||
			strings.TrimSpace(target.ProfileID) == "" ||
			strings.TrimSpace(target.ManifestRef) == "" ||
			target.Capabilities.RealExecution ||
			target.Capabilities.Teleop ||
			target.Capabilities.RawBrowserTransport {
			return machine.failClosed(now, "real_target_descriptor_invalid"),
				errors.New("targets: real target descriptor is incomplete or enables an unavailable capability")
		}
		machine.set(now, TargetRealCell, PhaseConnecting, target, nil, nil)
		return machine.State(), nil
	default:
		return machine.failClosed(now, "unsupported_target_mode"), errors.New("targets: unsupported target mode")
	}
}

func (machine *ModeMachine) Observe(now time.Time, observation TargetObservation) (ModeState, error) {
	if machine == nil {
		return ModeState{}, errors.New("targets: nil mode machine")
	}
	if machine.state.Mode != TargetRealCell {
		return machine.State(), errors.New("targets: real observation received outside real mode")
	}
	valid := observation.Connected &&
		observation.ProfileID == machine.state.Target.ProfileID &&
		observation.ManifestRef == machine.state.Target.ManifestRef &&
		observation.StatusSequence > 0 &&
		!observation.ObservedAt.IsZero() &&
		!now.Before(observation.ObservedAt) &&
		now.Before(observation.FreshUntil)
	if !valid {
		machine.set(now, TargetRealCell, PhaseBlocked, machine.state.Target, &observation,
			[]string{"real_target_status_stale_or_mismatched"})
		return machine.State(), errors.New("targets: real target status is stale or mismatched")
	}
	machine.set(now, TargetRealCell, PhaseConnected, machine.state.Target, &observation, nil)
	return machine.State(), nil
}

func (machine *ModeMachine) Disconnect(now time.Time, reason string) ModeState {
	if machine == nil {
		return ModeState{}
	}
	if strings.TrimSpace(reason) == "" {
		reason = "target_disconnected"
	}
	if machine.state.Mode == TargetRealCell {
		machine.set(now, TargetRealCell, PhaseBlocked, machine.state.Target, nil, []string{reason})
	} else {
		machine.failClosed(now, reason)
	}
	return machine.State()
}

func (machine *ModeMachine) set(
	now time.Time,
	mode TargetKind,
	phase ModePhase,
	target TargetDescriptor,
	observation *TargetObservation,
	blocked []string,
) {
	machine.state = ModeState{
		Revision:    machine.state.Revision + 1,
		Mode:        mode,
		Phase:       phase,
		Target:      target,
		Observation: observation,
		BlockedBy:   append([]string(nil), blocked...),
		ChangedAt:   now.UTC(),
	}
}

func (machine *ModeMachine) failClosed(now time.Time, reason string) ModeState {
	machine.set(now, TargetOfflinePreview, PhaseBlocked, OfflineDescriptor(), nil, []string{reason})
	return machine.State()
}
