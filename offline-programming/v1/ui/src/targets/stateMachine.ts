import {
  type SimulatorSafetyProof,
  type TargetDescriptor,
  type TargetMode,
  type TargetObservation,
  isTargetDescriptor,
  offlineTargetDescriptor,
} from "./contracts.ts";

const MAX_EVIDENCE_FUTURE_SKEW_MS = 1_000;

export type TargetPhase =
  | "offline"
  | "simulator_ready"
  | "connecting"
  | "connected"
  | "blocked";

export interface TargetState {
  revision: number;
  mode: TargetMode;
  phase: TargetPhase;
  target: TargetDescriptor;
  observation: TargetObservation | null;
  blockedBy: string[];
  changedAtMs: number;
}

export type TargetEvent =
  | { type: "select_offline"; atMs: number }
  | {
      type: "select_local_simulator_unavailable";
      atMs: number;
      target: TargetDescriptor;
      reason: string;
    }
  | { type: "select_local_simulator"; atMs: number; target: TargetDescriptor; proof: SimulatorSafetyProof }
  | { type: "select_real_target"; atMs: number; target: TargetDescriptor }
  | { type: "observe_real_target"; atMs: number; observation: TargetObservation }
  | { type: "disconnect"; atMs: number; reason: string };

export function initialTargetState(atMs: number): TargetState {
  return {
    revision: 1,
    mode: "offline_preview",
    phase: "offline",
    target: offlineTargetDescriptor(),
    observation: null,
    blockedBy: [],
    changedAtMs: atMs,
  };
}

export function reduceTargetState(state: TargetState, event: TargetEvent): TargetState {
  if (!validTime(event.atMs)) return failClosed(state, 0, "event_time_invalid");
  switch (event.type) {
    case "select_offline":
      return nextState(state, event.atMs, {
        mode: "offline_preview",
        phase: "offline",
        target: offlineTargetDescriptor(),
      });
    case "select_local_simulator_unavailable":
      if (!validLocalDescriptor(event.target)) {
        return failClosed(state, event.atMs, "local_simulator_descriptor_invalid");
      }
      return nextState(state, event.atMs, {
        mode: "local_simulator",
        phase: "blocked",
        target: event.target,
        blockedBy: [event.reason.trim() || "local_simulator_unavailable"],
      });
    case "select_local_simulator":
      if (!validSimulatorSelection(event.target, event.proof, event.atMs)) {
        return failClosed(state, event.atMs, "local_simulator_safety_proof_invalid");
      }
      return nextState(state, event.atMs, {
        mode: "local_simulator",
        phase: "simulator_ready",
        target: event.target,
      });
    case "select_real_target":
      if (!validRealDescriptor(event.target)) {
        return failClosed(state, event.atMs, "real_target_descriptor_invalid");
      }
      return nextState(state, event.atMs, {
        mode: "real_connected",
        phase: "connecting",
        target: event.target,
      });
    case "observe_real_target": {
      if (state.mode !== "real_connected") return state;
      if (!validRealObservation(state.target, event.observation, event.atMs)) {
        return nextState(state, event.atMs, {
          mode: "real_connected",
          phase: "blocked",
          target: state.target,
          observation: event.observation,
          blockedBy: ["real_target_status_stale_or_mismatched"],
        });
      }
      return nextState(state, event.atMs, {
        mode: "real_connected",
        phase: "connected",
        target: state.target,
        observation: event.observation,
      });
    }
    case "disconnect":
      if (state.mode === "real_connected") {
        return nextState(state, event.atMs, {
          mode: "real_connected",
          phase: "blocked",
          target: state.target,
          blockedBy: [event.reason.trim() || "target_disconnected"],
        });
      }
      return failClosed(state, event.atMs, event.reason.trim() || "target_disconnected");
  }
}

export function targetContextIsFresh(state: TargetState, atMs: number): boolean {
  const observation = state.observation;
  return state.mode === "real_connected" &&
    state.phase === "connected" &&
    observation !== null &&
    validRealObservation(state.target, observation, atMs);
}

function validSimulatorSelection(
  target: TargetDescriptor,
  proof: SimulatorSafetyProof,
  atMs: number,
): boolean {
  return isTargetDescriptor(target) &&
    target.kind === "local_simulator" &&
    target.capabilities.simulation &&
    !target.capabilities.realExecution &&
    !target.capabilities.teleop &&
    !target.capabilities.rawBrowserTransport &&
    proof.profileId === target.profileId &&
    positiveInteger(proof.statusSequence) &&
    validEvidenceTime(proof.observedAtMs, proof.freshUntilMs, atMs) &&
    proof.noRealMovement &&
    !proof.ethercatEnabled &&
    !proof.motionOutputEnabled;
}

function validLocalDescriptor(target: TargetDescriptor): boolean {
  return isTargetDescriptor(target) &&
    target.kind === "local_simulator" &&
    target.capabilities.simulation &&
    !target.capabilities.realExecution &&
    !target.capabilities.teleop &&
    !target.capabilities.rawBrowserTransport;
}

function validRealDescriptor(target: TargetDescriptor): boolean {
  return isTargetDescriptor(target) &&
    target.kind === "real_cell" &&
    nonempty(target.profileId) &&
    nonempty(target.manifestRef) &&
    !target.capabilities.realExecution &&
    !target.capabilities.teleop &&
    !target.capabilities.rawBrowserTransport;
}

function validRealObservation(
  target: TargetDescriptor,
  observation: TargetObservation,
  atMs: number,
): boolean {
  return observation.connected &&
    observation.profileId === target.profileId &&
    observation.manifestRef === target.manifestRef &&
    positiveInteger(observation.statusSequence) &&
    nonempty(observation.robotModel) &&
    validEvidenceTime(observation.observedAtMs, observation.freshUntilMs, atMs);
}

function failClosed(state: TargetState, atMs: number, reason: string): TargetState {
  return nextState(state, atMs, {
    mode: "offline_preview",
    phase: "blocked",
    target: offlineTargetDescriptor(),
    blockedBy: [reason],
  });
}

function nextState(
  current: TargetState,
  atMs: number,
  patch: Pick<TargetState, "mode" | "phase" | "target"> &
    Partial<Pick<TargetState, "observation" | "blockedBy">>,
): TargetState {
  return {
    revision: current.revision + 1,
    mode: patch.mode,
    phase: patch.phase,
    target: structuredClone(patch.target),
    observation: patch.observation ? structuredClone(patch.observation) : null,
    blockedBy: [...(patch.blockedBy ?? [])],
    changedAtMs: atMs,
  };
}

function validEvidenceTime(observedAtMs: number, freshUntilMs: number, atMs: number): boolean {
  return validTime(observedAtMs) &&
    validTime(freshUntilMs) &&
    observedAtMs <= atMs + MAX_EVIDENCE_FUTURE_SKEW_MS &&
    atMs < freshUntilMs;
}

function validTime(value: number): boolean {
  return Number.isFinite(value) && value >= 0;
}

function positiveInteger(value: number): boolean {
  return Number.isSafeInteger(value) && value > 0;
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}
