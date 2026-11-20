import {
  connectedStatusIsFresh,
  type ConnectedBooleanFact,
  type ConnectedStatusObservation,
} from "./targets/connectedStreams.ts";

export type ProgramExecutionMode = "simulation" | "dry_run" | "production";

export interface ExecutionPosture { backend: string; simulation?: boolean }

/** Only a described simulation backend permits a no-output claim; a selected
 * physical or unknown target must never inherit the browser's program mode. */
export function executionPosturePill(target: ExecutionPosture): SessionStatusPill {
  if (target.backend === "simulation" && target.simulation === true) {
    return { label: "SIMULATION", className: "is-offline", title: "Selected target is simulation. No physical robot output." };
  }
  if (target.backend === "ethercat" || target.simulation === false) {
    return { label: "REAL · PHYSICAL OUTPUT", className: "is-real", title: "Real machine selected. Explicit ARM and execution actions can produce physical motion." };
  }
  return { label: "OUTPUT UNKNOWN", className: "is-real", title: "Selected target posture is unavailable. Physical output cannot be ruled out." };
}

export interface SessionStatusPill {
  label: string;
  className: string;
  title: string;
}

export interface RobotLiveEvidence {
  connected: boolean;
  state: "unavailable" | "read_only" | null;
  observation: ConnectedStatusObservation | null;
  nowMs: number;
}

/**
 * Fail-closed: a physical robot is live only when read-only status says it is
 * connected and the observation is still fresh.
 */
export function robotTargetIsLive(evidence: RobotLiveEvidence): boolean {
  if (evidence.state !== "read_only" || !evidence.connected) {
    return false;
  }
  if (!evidence.observation) return false;
  return connectedStatusIsFresh(evidence.observation, evidence.nowMs);
}

/** Fail-closed: unknown or missing servo facts are disarmed. */
export function drivesAreArmed(fact: ConnectedBooleanFact | null | undefined): boolean {
  return fact?.known === true && fact.value === true;
}

export function sessionModeStatusPill(
  executionMode: ProgramExecutionMode,
  robotLive: boolean,
  target: ExecutionPosture | null = null,
): SessionStatusPill {
  if (target) return executionPosturePill(target);
  if (executionMode === "simulation") {
    return {
      label: "SIMULATION",
      className: "is-offline",
      title: "Browser preview is simulation. No execution target is selected.",
    };
  }
  if (executionMode === "dry_run") {
    return {
      label: "DRY RUN",
      className: "",
      title: "Browser preview is dry run. No execution target is selected.",
    };
  }
  if (robotLive) {
    return {
      label: "CONNECTED",
      className: "is-connected",
      title: "Production mode with a live connected robot target.",
    };
  }
  return {
    label: "NO ROBOT",
    className: "",
    title: "Production mode is selected, but no live robot target is available.",
  };
}

export function driveArmStatusPill(armed: boolean): SessionStatusPill {
  if (armed) {
    return {
      label: "ARMED",
      className: "is-armed",
      title: "Robot servos report armed. Physical motion may be live.",
    };
  }
  return {
    label: "DISARMED",
    className: "is-disarmed",
    title: "Connected jog status does not report armed drives.",
  };
}

// The header no longer carries a simulator ARM toggle: the browser
// simulator's arming lives with the Cartesian pad in the Jog panel, and the
// one machine pill comes from execution/machineFlow.ts.
