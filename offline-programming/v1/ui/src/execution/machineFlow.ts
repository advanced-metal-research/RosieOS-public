/**
 * The machine path as one derived view: Choose → Connect → Home → Arm →
 * Drive, each step done, current, blocked or not yet possible, with the
 * server's typed reason turned into words the operator can act on.
 *
 * This module is pure. It reads what the OLP server reported (the cell
 * catalogue, the selected target, capabilities and the last status) and
 * says which step is next and why the others are not. It never decides
 * that something is safe: a step is "done" only when the server's own
 * status says so, and an unknown posture is treated as real.
 *
 * What it exists to prevent: an operator hunting across pills, captions,
 * a feedback strip and a footer to learn why ARM did nothing, and arming a
 * real cell while believing it was the simulator.
 *
 * The armed state is one fact (`ArmFact`), derived here once and rendered by
 * every surface (header pill, footer, posture banner, Arm step, Drive
 * gating, Jog caption, run row) from the same value. It has exactly five
 * states: DISARMED, ARMING, ARMED, DISARMING (or STOPPING) and UNKNOWN, and
 * UNKNOWN is reserved for the one situation where the server has no fresh
 * observation of the machine. A server refusal, a "busy" result or a
 * request that never left the workstation does not make a known state
 * unknown: on the LA bench the header read ARMED while the Arm step read
 * "arm state is unknown" after a busy Arm, and that split is the defect.
 */

import { unknownRobotModelReason, type CellTarget } from "../targets/cells.ts";
import { rtCoreReasonText } from "../targets/rtCoreStatus.ts";
import { controllerFaultEventMessage, controlGrantMessage } from "../targets/operatorFeedback.ts";
import { executionPosturePill, type ExecutionPosture, type SessionStatusPill } from "../sessionStatusPills.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { denseExecutionState } from "./denseClient.ts";

export type MachineStepID = "choose" | "connect" | "home" | "arm" | "drive";
export type MachineStepState = "done" | "current" | "blocked" | "pending";
export type MachinePosture = "none" | "simulation" | "real" | "unknown";

/** Use the machine's observed posture for the pad heading, preventing a
 * selected physical target from inheriting the disconnected preview label. */
export function cartesianMachineHeading(view: MachineFlowView | null): string {
  if (!view?.connected) return "Drives the browser preview. Arm preview to jog.";
  const posture = view.posture === "simulation" ? "simulation, no physical output" : view.posture === "real" ? "REAL, motion is physical" : "output unknown, treated as real";
  return `Drives the selected machine, ${view.cellLabel} (${posture}).${view.arm.jogNote ? ` ${view.arm.jogNote}` : ""}`;
}

/** Status freshness bound for the armed state. Eight status poll periods
 *  (`DENSE_STATUS_POLL_MS`, 250 ms, in densePanel.ts) and well inside the
 *  server's 5 s heartbeat deadline, so the operator reads STATUS STALE
 *  before the server stops the machine for silence. Chosen, not measured. */
export const STATUS_FRESHNESS_MS = 2000;

export type ArmState = "disarmed" | "arming" | "armed" | "disarming" | "unknown";
/** A request this panel sends to the machine, by what it does to the arm state. */
export type MachineRequest = "arm" | "disarm" | "stop" | "other";

export interface ArmFact {
  state: ArmState;
  /** The large words at the top of the Arm step. */
  headline: string;
  /** The short token the header pill, footer and run row carry. */
  token: string;
  /** One or two sentences: why the state is what it is. */
  detail: string;
  /** The request in flight while arming, disarming or stopping. */
  pending: "arm" | "disarm" | "stop" | null;
  /** This server holds the grant, so ARMED is also drivable from here. */
  controlled: boolean;
  /** The last request never reached the machine and left its state alone. */
  transportRetry: string | null;
  /** The Jog panel's caption about the joints, or empty when jog may proceed. */
  jogNote: string;
}

export interface MachineStep {
  id: MachineStepID;
  title: string;
  state: MachineStepState;
  /** One line in plain words: what is true now, or what to do next. */
  summary: string;
  /** The typed server code behind a blocked or refused state, verbatim. */
  code?: string;
  /** The Arm step's state in large words, rendered before the summary. */
  headline?: string;
}

export interface MachineFlowInput {
  /** The server offers a cell catalogue (null: the single joint-daemon target). */
  catalogue: CellTarget[] | null;
  catalogueError: string | null;
  /** The cell currently highlighted in the picker, connected or not. */
  highlighted: CellTarget | null;
  /** The cell the server remembers as selected, and its label. */
  connectedCell: string;
  connectedLabel: string;
  modelId: string;
  /** A target is selected on this panel (status poll and heartbeat run). */
  connected: boolean;
  /** The picker points at a cell other than the connected one. */
  cellPending: boolean;
  /** The last Connect refusal, as the server's code. */
  connectError: string | null;
  inFlight: boolean;
  posture: ExecutionPosture | null;
  capabilities: DenseCapabilities | null;
  status: DenseStatus | null;
  /** A REAL or unknown target asked to arm and awaits the operator's confirmation. */
  armConfirmPending: boolean;
  /** Milliseconds since the server last answered the status poll (null: never). */
  statusAgeMs: number | null;
  /** This panel's request in flight, when there is one. */
  pendingRequest: MachineRequest | null;
  /** This panel's last completed request and how it ended (error null: answered). */
  lastRequest: { request: MachineRequest; error: string | null } | null;
  /** The active maintenance notice's words, when this client knows of one. */
  maintenance?: string | null;
}

export interface MachineFlowView {
  steps: MachineStep[];
  current: MachineStepID | null;
  posture: MachinePosture;
  /** The wording the header pill and the panel banner share for the posture. */
  postureLabel: string;
  cellLabel: string;
  modelId: string;
  connected: boolean;
  legacy: boolean;
  /** The one armed fact every surface renders. */
  arm: ArmFact;
  /** `arm.state` as a boolean for gates: true ARMED, false DISARMED, null otherwise. */
  armed: boolean | null;
  /** A program is playing or paused on the machine (only with a fresh status). */
  live: boolean;
  executionState: string;
  /** Described axis ids that report no valid Home. */
  homeRequired: string[];
  /** The machine offers native Home (rt-core) rather than a saved-pose GO HOME. */
  nativeHome: boolean;
  fault: string | null;
  canConnect: boolean;
  canHome: boolean;
  canArm: boolean;
  /** Why Arm is refused, or will fail, in the server's observation: one line
   *  per standing cause, shown until it clears (armBlockers). */
  armBlockers: string[];
  /** Arming needs the operator's second, deliberate click. */
  armConfirmRequired: boolean;
  canDrive: boolean;
  stopEnabled: boolean;
  disarmEnabled: boolean;
}

// Plain words for the typed reasons the server and rt-core return along
// this path. The code stays visible beside the words: support reads codes.
// Unverified operator wording; codes mirror the server's cell_* and rt_core
// refusal tables.
const machineReasonWords: Record<string, string> = {
  home_required: "Home is required before loading. Review the Home step to establish the current joint position references.",
  cell_configuration_mismatch: "The cell's deployed configuration no longer matches this catalogue entry. Re-provision the cell or update the catalogue; the server refuses until they agree.",
  cell_configuration_missing: "No native configuration is provisioned for this cell yet, so it cannot be connected.",
  cell_manifest_binding_missing: "The cell's manifest does not carry a complete rt-core identity.",
  cell_endpoint_mismatch: "The catalogue's endpoint disagrees with the cell's manifest.",
  cell_pair_mismatch: "The catalogue's pair id disagrees with the cell's manifest.",
  cell_pair_revision_mismatch: "The catalogue's pair revision disagrees with the cell's manifest.",
  cell_model_mismatch: "The chosen robot model is not the one this cell is configured for.",
  cell_mode_mismatch: "The cell's live backend is not what the catalogue requests (simulation against real, or the reverse).",
  cell_switch_while_playing: "A program is playing on the connected machine. STOP it before switching.",
  cell_switch_busy: "Another switch is already in progress. Try again in a moment.",
  cell_previous_stop_failed: "The previous machine did not acknowledge Stop, so its selection is kept. Use the hardware stop and check that cell.",
  cell_unreachable: "The cell did not answer. Check the tunnel and that its listener is up.",
  unreachable: "The cell did not answer when the catalogue was read. Check the tunnel and that its listener is up.",
  rt_core_transport_lost: "The connection to the cell's rt-core was lost (network, TLS alert or a revoked client). Nothing is commanded until it is described again.",
  rt_core_connection_reopened: "The connection was retried. No request reached the adapter; retry the operation when the connection is available.",
  rt_core_request_slow: "The machine is still answering the last request.",
  busy: "The machine is still serving a request.",
  session_principal_mismatch: "This client's certificate is not issued for the cell's pair. Use the credentials provisioned for it.",
  target_changed: "The machine selection changed while the request was in flight. Nothing was sent; check the selection and try again.",
  rt_core_inhibited: "New work is refused until the last Stop is acknowledged and control is released.",
  rt_core_closed: "The controller has shut down. Restart the offline-programming server.",
  control_session_stale: "Control was lost or superseded (a Stop, renewal loss or a newer session). Arm again to continue.",
  control_already_owned: "Another controller holds this machine (the pendant or another workstation). One controller is admitted at a time.",
  in_use_by_other_controller: "Another controller is using the robot; the notice says who. Nothing was sent. Stop still works.",
  // A pendant's own identity (OLP client_identity.go): a hidden pendant on a
  // Deck read the same controller as the visible one and armed another robot.
  client_not_focused: "This pendant was in the background (its window did not have input focus), so nothing was sent. Bring it to the front and try again. Stop always works.",
  client_cell_mismatch: "This pendant was started for another cell than the one this OLP server drives. Nothing was sent. Start the pendant's launcher for this cell. Stop always works.",
  no_grant: "No control grant is held. Arm acquires it.",
  ui_heartbeat_lost: "This browser stopped heartbeating, so the server stopped the machine.",
  operator_stop: "Stopped by the operator.",
  capability_unimplemented: "This machine does not implement that operation.",
  "rt_core status unavailable": "The machine has not reported a valid status yet.",
  "rt_core description identity is incompatible": "The cell's rt-core speaks a different contract or capabilities digest than this app was built against. Update the app or the cell so they match; nothing is commanded until they do.",
  rt_core_fault: "The controller reports a latched fault. Home requests encoder-reference recovery before homing; Arm resets it first only when every latched fault's recovery is a reset and no joint would lose Home. The controller must confirm recovery before either action can continue.",
  jog_mode_conflict: "Another motion is active on this machine. Release it first.",
  // POST /play on a loaded, disarmed robot (denseexec Play); the session stays loaded.
  play_requires_arm: "Arm the robot to play. The program stays loaded.",
  // POST /load of a plan made before a Home, configuration or controller change.
  planning_start_stale: "Home, the configuration or the controller changed since this plan was made. Plan again from the robot's current position, then Load.",
  dense_blob_not_found: "The weld planner no longer holds this plan and no kept copy was accepted. Plan again.",
};

// Reasons that mean the machine will refuse or cannot be reached: the path
// shows them as blocked. Every other reported reason (a Stop result, a lost
// grant, a refused jog) is the outcome of the last action; Arm again is the
// recovery, so those are shown as a note and the server stays the authority.
// Unverified classification; the server refuses regardless of this list.
const blockingReasons = new Set([
  "rt_core_inhibited", "rt_core_closed", "rt_core_transport_lost", "rt_core status unavailable",
  "rt_core_event", "control_already_owned", "session_principal_mismatch", "target_changed",
]);
// rt_core_fault is deliberately not blocking: the server's Arm resets faults
// whose recovery is a reset first, so Arm is the recovery, not a dead end.

export function isBlockingReason(reason: string): boolean {
  const code = reason.split(":", 1)[0]!.trim();
  return blockingReasons.has(code) || blockingReasons.has(reason.trim());
}

// Reasons under which the server holds no usable observation of the machine.
// With one of these the arm state is UNKNOWN, never DISARMED: a lost
// connection says nothing about the drives.
const noObservationReasons = new Set(["rt_core_transport_lost", "rt_core_closed", "rt_core status unavailable"]);
// Reasons that mean a request is still being served: the arm state is in
// transition (ARMING or DISARMING), not unknown. `busy` is the adapter's
// word (rt-core ReasonBusy); `rt_core_request_slow` is the server's.
const inFlightReasons = new Set(["busy", "rt_core_request_slow"]);
// A request the server proved unsent: the machine never saw it, so its arm
// state did not change. The bench showed these as raw dial errors
// ("write tcp ...: broken pipe") under an "arm state is unknown" line.
const transportUnsentReasons = new Set(["rt_core_connection_reopened"]);

/** A reason under which the server holds no observation of the machine: the
 *  cell is unreachable, not merely reporting a result. Every other reason
 *  (a Stop, a Disarm, a refused jog) is the outcome of the last action and
 *  proves the machine answered. */
export function isNoObservationReason(reason: string): boolean {
  const code = reason.split(":", 1)[0]!.trim();
  return noObservationReasons.has(code) || noObservationReasons.has(reason.trim());
}

/** FAULT in the title bar: a latched rt-core fault or a blocking reason.
 *  "disarm" and "operator_stop" are the results of the operator's own action
 *  and leave the pill with the drives' word. */
export function isMachineFault(reason: string): boolean {
  const code = reason.split(":", 1)[0]!.trim();
  return code === "rt_core_fault" || isBlockingReason(reason);
}
const transportUnsentPattern = /rt_core_connection_reopened|broken pipe|connection refused|connection reset|unsent/i;

/** A failed request whose bytes never reached the machine (a closed or
 * refused connection). Unverified pattern beyond the server's typed code. */
export function isTransportUnsent(message: string): boolean {
  const code = message.split(":", 1)[0]!.trim();
  return transportUnsentReasons.has(code) || transportUnsentPattern.test(message);
}

function reasonCode(reason: string | null): string {
  return reason ? reason.split(":", 1)[0]!.trim() : "";
}

function requestName(request: MachineRequest | null): string {
  return request === "arm" ? "Arm" : request === "disarm" ? "Disarm" : request === "stop" ? "Stop" : "the request";
}

/**
 * The armed state as one fact. The observation is the drives' own word
 * (`core.armed` on rt-core, the daemon's `armed` on the legacy backend),
 * fresh within `STATUS_FRESHNESS_MS` and not superseded by a reason that
 * says the server cannot see the machine. Over that, a request in flight
 * (this panel's, or the server's `busy`) names the transition. Anything
 * else is a note beside the state, never a substitute for it.
 */
function deriveArmFact(input: MachineFlowInput, legacy: boolean, real: boolean, cellLabel: string, fault: string | null): ArmFact {
  const { status } = input;
  const code = reasonCode(fault);
  const flags = status?.rtCoreFlags ?? null;
  const pending = input.pendingRequest === "arm" || input.pendingRequest === "disarm" || input.pendingRequest === "stop" ? input.pendingRequest : null;
  const last = input.lastRequest;
  const machine = real ? cellLabel : `the simulation ${cellLabel}`;

  let transportRetry: string | null = null;
  if (last?.error && isTransportUnsent(last.error)) {
    transportRetry = `Request did not reach the machine; retry ${requestName(last.request)}.`;
  } else if (transportUnsentReasons.has(code)) {
    transportRetry = `Request did not reach the machine; retry ${requestName(pending ?? last?.request ?? null)}.`;
  }

  const observed: boolean | null = status === null ? null : legacy ? status.armed : status.rtCore?.coreArmed ?? null;
  const fresh = observed !== null && input.statusAgeMs !== null && input.statusAgeMs <= STATUS_FRESHNESS_MS && !noObservationReasons.has(code);
  const controlled = legacy ? observed === true : status?.session?.leaseHeld === true;
  const uncertain = flags?.stopUncertain ? " The server reports the last Stop outcome unconfirmed." : "";

  if (!fresh) {
    const why = status === null
      ? "The machine has not reported a status yet."
      : noObservationReasons.has(code) || observed === null
        ? (fault ? machineReasonText(fault) : "The last status carried no servo observation.")
        : `The last status is ${((input.statusAgeMs ?? 0) / 1000).toFixed(1)} s old; the bound is ${STATUS_FRESHNESS_MS / 1000} s.`;
    return {
      state: "unknown", headline: "UNKNOWN: no fresh status from the machine", token: "STATUS STALE",
      detail: `${why} Whether the drives on ${cellLabel} are energised cannot be told from here; STOP stays available.`,
      pending, controlled, transportRetry, jogNote: "Arm state UNKNOWN: no fresh status from the machine.",
    };
  }

  const busy = inFlightReasons.has(code);
  const lastDirection = last?.request === "arm" || last?.request === "disarm" || last?.request === "stop" ? last.request : null;
  let transition: "arm" | "disarm" | "stop" | null = null;
  if (pending === "arm" && !observed) transition = "arm";
  else if ((pending === "disarm" || pending === "stop") && observed) transition = pending;
  else if (busy) {
    // The server says a request is being served but this panel has none in
    // flight: the direction is the last one sent from here, else the only
    // one that changes the observed state.
    transition = observed
      ? (lastDirection === "stop" ? "stop" : "disarm")
      : (lastDirection === "disarm" || lastDirection === "stop" ? lastDirection : "arm");
  }
  const via = busy ? " (the machine reports busy)" : "";
  if (transition === "arm") {
    return {
      state: "arming", headline: "ARMING…", token: "ARMING",
      detail: `Arm request in flight${via}: acquiring control and enabling the drives on ${machine}. Wait for ARMED.`,
      pending: transition, controlled, transportRetry, jogNote: "Arming; wait for ARMED before jogging.",
    };
  }
  if (transition) {
    const stop = transition === "stop";
    return {
      state: "disarming", headline: stop ? "STOPPING…" : "DISARMING…", token: stop ? "STOPPING" : "DISARMING",
      detail: `${stop ? "Stop" : "Disarm"} request in flight${via}: stopping and releasing control of ${machine}. Wait for DISARMED.${uncertain}`,
      pending: transition, controlled, transportRetry, jogNote: `${stop ? "Stopping" : "Disarming"}; wait for DISARMED.`,
    };
  }
  if (observed) {
    const other = controlled ? "" : " Another controller holds the machine; this workstation has no grant, so it cannot drive from here.";
    return {
      state: "armed", headline: "ARMED", token: "ARMED",
      detail: `${real ? `Motors on ${cellLabel} are energised.` : `Armed on the simulation ${cellLabel}.`}${other}${uncertain}`,
      pending: null, controlled, transportRetry, jogNote: controlled ? "" : "Armed by another controller; this workstation holds no grant.",
    };
  }
  return {
    state: "disarmed", headline: "DISARMED", token: "DISARMED",
    detail: `${real ? `Motors on ${cellLabel} are off.` : `Drives on the simulation ${cellLabel} are off.`}${uncertain}`,
    pending: null, controlled, transportRetry, jogNote: "Arm it first (step 4).",
  };
}

/** Words for a typed reason, with the code kept beside them. The code may
 * arrive as `code: detail`; only the code selects the wording. A detail that
 * already repeats the code (the HTTP `{error, detail}` pair joined, then
 * prefixed again) is stripped so the operator reads `code: text` once. */
export function machineReasonText(reason: string): string {
  const trimmed = reason.trim();
  if (!trimmed) return "";
  const grant = controlGrantMessage(trimmed.split(":", 1)[0]!);
  if (grant) return grant;
  const controllerFault = controllerFaultEventMessage(trimmed);
  if (controllerFault) return `${controllerFault} [${trimmed}]`;
  const code = trimmed.split(":", 1)[0]!.trim();
  const words = machineReasonWords[code] ?? machineReasonWords[trimmed];
  if (words) return `${words} [${code}]`;
  let detail = trimmed.slice(code.length + 1).trim();
  while (detail === code || detail.startsWith(`${code}:`)) {
    detail = detail === code ? "" : detail.slice(code.length + 1).trim();
  }
  // rtCoreReasonText already keeps the code beside its label.
  return rtCoreReasonText(code, detail);
}

function postureOf(posture: ExecutionPosture | null, connected: boolean): { posture: MachinePosture; label: string } {
  if (!posture && !connected) return { posture: "none", label: "NO MACHINE" };
  if (!posture) return { posture: "unknown", label: "OUTPUT UNKNOWN" };
  const pill = executionPosturePill(posture);
  if (pill.label === "SIMULATION") return { posture: "simulation", label: pill.label };
  if (pill.label === "OUTPUT UNKNOWN") return { posture: "unknown", label: pill.label };
  return { posture: "real", label: pill.label };
}

function axisList(ids: readonly string[]): string {
  if (ids.length <= 3) return ids.join(", ");
  return `${ids.slice(0, 3).join(", ")} and ${ids.length - 3} more`;
}

/**
 * Every standing reason Arm is refused, will fail or will do more than arm,
 * from the server's last status, in words that name the action. On a
 * Rosie 1430 arm (2026-10-11 12:19) J5's drive tripped Er41.1 and OLP latched "last Stop
 * unconfirmed"; each Arm press on the Deck and in the browser was refused
 * with a generic line or none, and nothing said J5 had a drive alarm. The list
 * stays on screen until the cause clears: an unconfirmed Stop or release (the
 * server's Arm confirms it from the robot's state first), a drive alarm on a
 * named axis (the server's Arm resets it first when every latched fault's
 * recovery is a reset; owner 2026-10-11: "re-arming the robot should clear
 * faults too if they exist"), and the maintenance notice. A controller
 * holding the robot has its own notice (controlNotice).
 */
export function armBlockers(status: DenseStatus | null, maintenance?: string | null): string[] {
  const out: string[] = [];
  if (maintenance) out.push(`Maintenance: ${maintenance}. Arming is refused until it ends.`);
  const flags = status?.rtCoreFlags;
  if (flags?.stopUncertain) {
    out.push("The last Stop is unconfirmed. Arm confirms it from the robot's state first (disarmed, no lease, no grant); if the robot does not show that, press STOP.");
  } else if (flags?.inhibited) {
    out.push("Stop was acknowledged but releasing control was not. Arm confirms the release from the robot's state first; if it cannot, press STOP again.");
  }
  const axes = status?.rtCore?.axes ?? [];
  const faults = status?.rtCore?.operatorFeedback?.reset.faults ?? [];
  const resettable = faults.every(fault => (fault.recovery === "reset_clears" || fault.recovery === "reset_after_condition_clears") && !fault.rehome_axis_mask);
  for (const feedback of status?.rtCore?.operatorFeedback?.axes ?? []) {
    if (!feedback.alarmText || axes[feedback.index]?.readiness !== "faulted") continue;
    const check = "Check that joint (a blockage, a cable or hose binding, a brake that did not release, a hot motor)";
    out.push(resettable && feedback.alarmShort
      ? `${feedback.alarmText}. Arm will reset ${feedback.alarmShort} first; if the drive still reports it, Arm is refused with the reason. ${check}.`
      : `${feedback.alarmText}. ${check}.`);
  }
  return out;
}

/** Arm's own refusals from its first steps (denseexec settleForArm): the
 *  server's sentence names the joint, the alarm and what to do. */
const armSettleCodes = new Set(["fault_persists", "fault_not_resettable", "motion_active", "control_held_elsewhere", "arm_incomplete"]);

/** The words for a refused Arm: the server's sentence for Arm's own fault
 *  steps, else the standing causes when there are any (the refusal's code is
 *  then their consequence and stays beside them), else the refusal itself. */
export function armRefusalText(blockers: readonly string[], error: string): string {
  const code = reasonCode(error);
  const detail = error.slice(error.indexOf(":") + 1).trim();
  if (armSettleCodes.has(code) && error.includes(":") && detail) return `${detail} [${code}]`;
  return blockers.length ? `${blockers.join(" ")}${code ? ` [${code}]` : ""}` : machineReasonText(error);
}

/** A restart-required native fault cannot be recovered by Home. Preserve the
 * supported reference-recovery Home path for other recovery policies. */
export function nativeHomeRecoveryRefusal(status: DenseStatus | null): string | null {
  const faults = status?.rtCore?.operatorFeedback?.reset.faults.filter(fault => fault.recovery === "restart_required") ?? [];
  return faults.length ? `Home unavailable: ${faults.map(fault => fault.name).join(", ")} requires controller restart and recovery.` : null;
}

/** Derive the five steps from server-reported state only. */
export function deriveMachineFlow(input: MachineFlowInput): MachineFlowView {
  const { status, capabilities } = input;
  const modelReason = unknownRobotModelReason(input.modelId, status?.target?.model_id);
  const legacy = capabilities !== null && capabilities.backend !== "rt_core";
  const nativeHome = capabilities?.backend === "rt_core" && capabilities.rtCore?.nativeHome === true;
  const { posture, label: postureLabel } = postureOf(input.posture, input.connected);
  const real = posture === "real" || posture === "unknown";
  const cellLabel = input.connectedLabel || input.highlighted?.label || capabilities?.cell || "selected target";
  const executionState = denseExecutionState(status);
  const fault = input.connected ? status?.daemonError ?? null : null;
  const code = reasonCode(fault);
  const arm = deriveArmFact(input, legacy, real, cellLabel, fault);
  // A stale status cannot vouch for a playing program any more than for the drives.
  const live = input.connected && arm.state !== "unknown" && status?.session !== null && (executionState === "playing" || executionState === "paused");
  const armed = input.connected ? arm.state === "armed" ? true : arm.state === "disarmed" ? false : null : null;
  // The drives are observed energised (armed, or a stop still in flight).
  const motorsOn = input.connected && (arm.state === "armed" || arm.state === "disarming");
  const blocking = fault !== null && isBlockingReason(fault);
  // A refusal is a note beside the state. A request in flight or one proven
  // unsent is not a result at all: the fact already says ARMING, or carries
  // the retry line, and "Last result: busy" is the wording the bench showed
  // under a state it had no business calling unknown.
  const note = fault && !blocking && !inFlightReasons.has(code) && !transportUnsentReasons.has(code) ? ` Last result: ${machineReasonText(fault)}` : "";
  const retry = arm.transportRetry ? ` ${arm.transportRetry}` : "";
  const recovery = status?.rtCoreFlags?.recoveryAction ? ` ${status.rtCoreFlags.recoveryAction}` : "";
  const axes = capabilities?.rtCore?.axes ?? [];
  const homeRequired = input.connected && status?.rtCore
    ? status.rtCore.axes.flatMap((axis, index) => axis.homeValid ? [] : [axes[index]?.id ?? `J${index + 1}`])
    : [];
  const homeLoss = status?.rtCore?.operatorFeedback.axes.find(axis => axis.homeLoss)?.homeLoss ?? "";

  const steps: MachineStep[] = [];

  // 1. Choose.
  let choose: MachineStep;
  if (input.catalogueError) {
    choose = { id: "choose", title: "Choose a machine", state: "blocked", summary: `The machine catalogue could not be read (${input.catalogueError}); retrying. The server keeps its selection; nothing was changed on the machine.` };
  } else if (input.catalogue === null) {
    choose = input.connected || input.highlighted
      ? { id: "choose", title: "Choose a machine", state: "done", summary: capabilities?.cell ? `Joint daemon for ${capabilities.cell}` : "Joint daemon" }
      : { id: "choose", title: "Choose a machine", state: "current", summary: capabilities?.available === false
        ? `No joint daemon is configured on this server${capabilities.reason ? `: ${capabilities.reason}` : ""}`
        : "Pick the joint daemon to work on a machine. Without one, nothing here moves a robot." };
  } else if (input.connected && !input.cellPending) {
    choose = { id: "choose", title: "Choose a machine", state: "done", summary: `${cellLabel} with ${input.modelId}` };
  } else if (input.highlighted?.reason) {
    choose = { id: "choose", title: "Choose a machine", state: "blocked", summary: `${input.highlighted.label} is unavailable. ${machineReasonText(input.highlighted.reason)}`, code: input.highlighted.reason };
  } else if (input.highlighted) {
    choose = { id: "choose", title: "Choose a machine", state: "done", summary: `${input.highlighted.label} with ${input.modelId}, ${input.highlighted.requested_mode === "real" ? "a real machine" : "a simulation"}` };
  } else {
    choose = { id: "choose", title: "Choose a machine", state: "current", summary: "Pick a machine from the list. Choosing grants no control." };
  }
  steps.push(choose);

  // 2. Connect.
  let connect: MachineStep;
  const chooseDone = choose.state === "done" || (choose.state === "blocked" && input.highlighted !== null && input.highlighted.reason !== "cell_configuration_missing");
  if (input.catalogue === null) {
    connect = input.connected
      ? { id: "connect", title: "Connect", state: "done", summary: capabilities?.daemonReachable === false ? "Joint daemon selected but not reachable" : "Joint daemon reachable" }
      : { id: "connect", title: "Connect", state: chooseDone ? "current" : "pending", summary: chooseDone ? "Select the joint daemon to start its status poll." : "Choose first." };
  } else if (!chooseDone) {
    connect = { id: "connect", title: "Connect", state: "pending", summary: "Choose an available machine first." };
  } else if (input.connectError) {
    connect = { id: "connect", title: "Connect", state: "blocked", summary: `Connect was refused. ${machineReasonText(input.connectError)}`, code: input.connectError.split(":", 1)[0] };
  } else if (input.inFlight && !input.connected) {
    connect = { id: "connect", title: "Connect", state: "current", summary: "Describing the machine and checking its identity." };
  } else if (input.connected && !input.cellPending) {
    const backend = input.posture?.backend ?? "unknown backend";
    connect = { id: "connect", title: "Connect", state: "done", summary: `Identity verified over mutual TLS. ${backend}${real ? ". This is a real machine." : ""}` };
  } else if (input.connected && input.cellPending) {
    connect = { id: "connect", title: "Connect", state: "current", summary: `Connect to switch to ${input.highlighted?.label ?? "the highlighted machine"}. ${input.connectedLabel || "The current machine"} stays connected until then.` };
  } else {
    connect = { id: "connect", title: "Connect", state: "current", summary: input.highlighted?.requested_mode === "real"
      ? "Connect describes the cell over mutual TLS and verifies its identity. It grants no control and moves nothing."
      : "Connect describes the simulation and verifies its identity. It grants no control." };
  }
  steps.push(connect);

  // Steps 3 to 5 concern the connected machine; a pending switch keeps them
  // pointed at the connected one until Connect succeeds.
  const connectedNow = input.connected && !input.cellPending;
  const statusReady = connectedNow && status !== null;

  // 3. Home.
  const homeRecoveryRefusal = nativeHomeRecoveryRefusal(status);
  let home: MachineStep;
  if (legacy) {
    home = { id: "home", title: "Home", state: connectedNow ? "done" : "pending", summary: connectedNow ? "This backend uses saved-pose GO HOME, a motion: available here after Arm." : "Connect first." };
  } else if (!connectedNow) {
    home = { id: "home", title: "Home", state: "pending", summary: "Connect first." };
  } else if (homeRecoveryRefusal) {
    home = { id: "home", title: "Home", state: "blocked", summary: homeRecoveryRefusal, code: "restart_required" };
  } else if (fault && blocking) {
    home = { id: "home", title: "Home", state: "blocked", summary: machineReasonText(fault), code: fault.split(":", 1)[0] };
  } else if (!statusReady || !status?.rtCore) {
    home = { id: "home", title: "Home", state: "pending", summary: "Waiting for the machine's first status." };
  } else if (homeRequired.length === 0) {
    home = { id: "home", title: "Home", state: "done", summary: axes.length ? `Home valid on ${axisList(axes.map(axis => axis.id))}.` : "Home valid." };
  } else if (status.rtCore.axes.some(axis => axis.calibrationValid === true && axis.positionState !== "trusted")) {
    home = { id: "home", title: "Home", state: "pending", summary: homeLoss || "Calibration retained; waiting for verified position recovery while disarmed and stationary." };
  } else if (motorsOn) {
    home = { id: "home", title: "Home", state: "blocked", summary: `${axisList(homeRequired)} report Home required. DISARM, then Home. Home briefly energises the drives; settling movement may occur.` };
  } else if (!nativeHome) {
    home = { id: "home", title: "Home", state: "blocked", summary: `${axisList(homeRequired)} report Home required, and this machine does not offer native Home.` };
  } else {
    home = { id: "home", title: "Home", state: "current", summary: `${axisList(homeRequired)} report Home required${homeLoss ? ` (${homeLoss.toLowerCase()})` : ""}. Start disarmed. Home establishes the joint position references, briefly energising the drives; settling movement may occur.` };
  }
  steps.push(home);

  // 4. Arm. The state comes first, in the fact's words; the summary is the
  // explanation and the next action.
  let armStep: MachineStep;
  // Only a known Home requirement holds Arm back; before the first status
  // the server decides, as it always did.
  const homeKnownRequired = home.state === "current" || home.state === "blocked";
  const inTransition = arm.state === "arming" || arm.state === "disarming";
  const blockers = connectedNow ? armBlockers(status, input.maintenance) : [];
  // Only maintenance refuses Arm here. An unconfirmed Stop and a drive alarm
  // leave Arm available (the server's Arm confirms the Stop and resets the
  // alarm first) and lead its summary.
  const armRefused = connectedNow && !!input.maintenance;
  const alarmNote = blockers.length && !armRefused ? `${blockers.join(" ")} ` : "";
  if (modelReason) {
    armStep = { id: "arm", title: "Arm", state: "blocked", summary: modelReason, headline: arm.headline };
  } else if (!connectedNow) {
    armStep = { id: "arm", title: "Arm", state: "pending", summary: "Connect first." };
  } else if (armRefused && !inTransition) {
    armStep = { id: "arm", title: "Arm", state: "blocked", summary: `${blockers.join(" ")}${recovery}`, headline: arm.headline, code: "maintenance" };
  } else if (fault && blocking) {
    armStep = { id: "arm", title: "Arm", state: "blocked", summary: `${machineReasonText(fault)}${recovery}`, code: code, headline: arm.headline };
  } else if (arm.state === "armed" && arm.controlled) {
    armStep = { id: "arm", title: "Arm", state: "done", summary: `${arm.detail}${retry}`, headline: arm.headline };
  } else if (inTransition) {
    armStep = { id: "arm", title: "Arm", state: "current", summary: `${arm.detail}${retry}`, headline: arm.headline };
  } else if (arm.state === "armed") {
    armStep = { id: "arm", title: "Arm", state: "current", summary: `${arm.detail} Arm here acquires control if the machine admits a second controller.${retry}${note}`, headline: arm.headline };
  } else if (!legacy && homeKnownRequired) {
    armStep = { id: "arm", title: "Arm", state: homeRecoveryRefusal ? "blocked" : "pending",
      summary: homeRecoveryRefusal || "Home first.", headline: arm.headline };
  } else if (input.armConfirmPending) {
    armStep = { id: "arm", title: "Arm", state: "current", summary: `Confirm to energise the motors on ${cellLabel}. Cancel leaves them off.`, headline: arm.headline };
  } else if (arm.state === "unknown") {
    armStep = { id: "arm", title: "Arm", state: "current", summary: `${alarmNote}${arm.detail} Arm acquires control and enables the drives${real ? " on a real machine" : ""}; the server decides.${retry}${note}`, headline: arm.headline };
  } else {
    armStep = { id: "arm", title: "Arm", state: "current", summary: alarmNote + (real
      ? `Arm acquires control and energises the motors on ${cellLabel}. A second click confirms.`
      : `Arm acquires control and enables the simulated drives. No physical output.`) + retry + note, headline: arm.headline };
  }
  steps.push(armStep);

  // 5. Drive. Opens only on ARMED with this server's grant: the same fact,
  // so it can never say "Arm first" beside a pill that says ARMED.
  let drive: MachineStep;
  const ready = status?.rtCore?.ready;
  if (modelReason) {
    drive = { id: "drive", title: "Drive", state: "blocked", summary: modelReason };
  } else if (!connectedNow) {
    drive = { id: "drive", title: "Drive", state: "pending", summary: "Connect and arm first." };
  } else if (fault && blocking) {
    drive = { id: "drive", title: "Drive", state: "blocked", summary: machineReasonText(fault), code: code };
  } else if (arm.state !== "armed" || !arm.controlled) {
    drive = { id: "drive", title: "Drive", state: "pending", summary:
      arm.state === "arming" ? "Arming; Drive opens at ARMED."
      : arm.state === "disarming" ? `${arm.pending === "stop" ? "Stopping" : "Disarming"}; Arm again afterwards.`
      : arm.state === "unknown" ? "Arm state UNKNOWN: no fresh status from the machine."
      : arm.state === "armed" ? "Armed by another controller; Arm here to take control."
      : "Arm first." };
  } else if (live) {
    drive = { id: "drive", title: "Drive", state: "current", summary: executionState === "paused" ? "Program paused on the machine." : "Program playing on the machine. STOP ends it." };
  } else if (!legacy && ready !== true) {
    const unready = status?.rtCore?.axes.map((axis, index) => axis.readiness !== "ready" ? `${axes[index]?.id ?? `J${index + 1}`} ${axis.readiness}` : "").filter(Boolean) ?? [];
    drive = { id: "drive", title: "Drive", state: "blocked", summary: unready.length
      ? `Not ready: ${axisList(unready)}. Jog and Play wait for every axis to report ready.`
      : "Not ready: waiting for valid feedback from every axis." };
  } else {
    drive = { id: "drive", title: "Drive", state: "current", summary: `Jog joints, move by angle or run the program${real ? ` on ${cellLabel}. Motion is physical.` : " on the simulation."}` };
  }
  steps.push(drive);

  const current = steps.find(step => step.state === "current")?.id ?? null;
  return {
    steps, current, posture, postureLabel, cellLabel, modelId: input.modelId,
    connected: input.connected, legacy, arm, armed, live, executionState, homeRequired, nativeHome, fault,
    canConnect: connect.state === "current" && !input.inFlight,
    canHome: home.state === "current" && !input.inFlight && !inTransition,
    canArm: capabilities?.backend === "rt_core" && capabilities.available && armStep.state === "current" && !input.inFlight && !inTransition,
    armBlockers: blockers,
    armConfirmRequired: real,
    canDrive: capabilities?.backend === "rt_core" && capabilities.available && drive.state === "current" && !input.inFlight,
    stopEnabled: input.connected,
    disarmEnabled: input.connected && (legacy ? armed !== false && !input.inFlight : true),
  };
}

export interface LegacyPillContext {
  executionMode: "simulation" | "dry_run" | "production";
  robotLive: boolean;
  connectedArmed: boolean;
}

/** The one header pill about the machine. Real posture is red whether or
 * not it is armed; only a described simulation is green. Without a machine
 * the pill says so instead of borrowing the program's preview mode. The
 * arm token is the fact's: while it is UNKNOWN the pill says STATUS STALE
 * and claims neither ARMED nor DISARMED. */
export function machineStatePill(view: MachineFlowView, legacy: LegacyPillContext): SessionStatusPill {
  if (!view.connected) {
    if (legacy.executionMode === "production" && legacy.robotLive) {
      return {
        label: `CONNECTED · READ-ONLY · ${legacy.connectedArmed ? "ARMED" : "DISARMED"}`,
        className: "is-connected",
        title: "A manifest-backed robot is observed read-only. This page does not arm it; the pill mirrors its servo fact.",
      };
    }
    return { label: "NO MACHINE", className: "is-none", title: "No machine connected. The browser preview cannot move a robot. Choose and connect a machine under Cells." };
  }
  const motion = view.live ? view.executionState.toUpperCase() : view.arm.token;
  const label = `${view.posture === "simulation" ? "SIMULATION" : view.posture === "unknown" ? "OUTPUT UNKNOWN" : "REAL"} · ${view.cellLabel} · ${motion}`;
  const stale = view.arm.state === "unknown";
  const energised = view.arm.state === "armed" || view.arm.state === "disarming" || view.live;
  if (view.posture === "simulation") {
    return {
      label,
      className: energised ? "is-sim-armed" : stale ? "is-offline is-stale" : "is-offline",
      title: stale
        ? `${view.cellLabel}: no fresh status from the machine, so whether it is armed is unknown. STOP stays available.`
        : `${view.cellLabel} is a described simulation. No physical robot output.`,
    };
  }
  return {
    label,
    className: energised ? "is-real" : stale ? "is-real-idle is-stale" : "is-real-idle",
    title: stale
      ? `${view.cellLabel}: no fresh status from the machine, so whether its drives are energised is unknown. STOP stays available.`
      : view.posture === "unknown"
        ? `${view.cellLabel}: posture unavailable, so physical output cannot be ruled out.`
        : `${view.cellLabel} is a real machine. ${energised ? "Motors are energised; explicit actions move metal." : "Arm energises its motors."}`,
  };
}
