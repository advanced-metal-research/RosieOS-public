// The pendant's view of the selected machine, derived by OLP's own code from
// the OLP server's replies: normalisation, the machine flow (steps, gates and
// the one armed fact), joint gates, home status, speed captions, Cartesian
// twists and every operator reason text. The Qt side only transports replies
// and times requests; it never decides whether motion is allowed.
import {
  denseExecutionState, normalizeDenseCapabilities, normalizeDenseStatus, normalizeRobotCell,
  type DenseCapabilities, type DenseStatus,
} from "../../../../offline-programming/v1/ui/src/execution/denseClient.ts";
import { armBlockers, armRefusalText as armRefusalWords, deriveMachineFlow, machineReasonText, type MachineRequest } from "../../../../offline-programming/v1/ui/src/execution/machineFlow.ts";
import {
  JOINT_JOG_FRACTIONS, JOINT_ZERO_TOLERANCE_DEG, jointDescribedRangeRefusal, jointJogDisabled, jointJogSpeedNote,
} from "../../../../offline-programming/v1/ui/src/execution/jointJog.ts";
import { deriveHomeStatus } from "../../../../offline-programming/v1/ui/src/execution/homeStatus.ts";
import {
  normalizeRecoverResult, recoverConfirmation, recoverFailureText, recoverOutcomeText, recoverRequest, recoverToZeroDisabled,
  type RecoverJogSpeeds,
} from "../../../../offline-programming/v1/ui/src/execution/recoverToZero.ts";
import {
  cartesianCancelText, cartesianDefaultMode, cartesianHoldTwist, cartesianMoveBody, cartesianStepLabels,
  requireCartesianSteps, type CartesianMode,
} from "../../../../offline-programming/v1/ui/src/execution/cartesianMove.ts";
import { createOperatorFence } from "../../../../offline-programming/v1/ui/src/targets/operatorFeedback.ts";
import { cellSettingsOf, machineReachable, teachObservationOf, type TeachStill } from "../../../../offline-programming/v1/ui/src/execution/teach.ts";
import { rtCoreReasonText } from "../../../../offline-programming/v1/ui/src/targets/rtCoreStatus.ts";
import type { CellTarget } from "../../../../offline-programming/v1/ui/src/targets/cells.ts";
import { setConnectedRobot } from "../../../../offline-programming/v1/ui/src/domain/robots.ts";

import { TeachSettle } from "./teach_settle.ts";
const teachSettle = new TeachSettle();
export function teachDiagnostics() { return teachSettle.diagnostics; }

let status: DenseStatus | null = null;
let capabilities: DenseCapabilities | null = null;
let still: TeachStill | null = null;
let nativeObservation = "";
let lastNativeTimeNs = 0;
const fence = createOperatorFence();

export function observeCapabilities(payload: unknown): void {
  capabilities = normalizeDenseCapabilities(payload);
}

/**
 * The Dense panel's #teachBusy: the Qt side's request in flight, held jog or
 * move (`local`), or a playing or paused program on the selected machine
 * (its robotMoving). No stillness accrues and no capture is taken while busy.
 */
function teachBusy(local: boolean): boolean {
  const state = denseExecutionState(status);
  return local || state === "playing" || state === "paused";
}

/**
 * As the Dense panel applies a status: a reply older than one already applied
 * in the same native clock domain is dropped, so a delayed success cannot
 * replace a newer refusal; the teach stillness window follows every reply.
 * `busy`: the Qt side's request in flight, held jog or move (teachBusy adds
 * a playing program).
 */
export function observeStatus(payload: unknown, busy: boolean): void {
  const next = normalizeDenseStatus(payload, fence);
  const target = next.target ?? status?.target;
  const identity = `${target?.cell_id ?? ""}:${target?.selection_generation ?? ""}:${next.rtCore?.observationIdentity}`;
  if (next.rtCore && identity === nativeObservation && next.rtCore.timeNs < lastNativeTimeNs) return;
  status = { ...next, target };
  still = teachSettle.observe(status, capabilities, identity, teachBusy(busy));
  if (status.rtCore && (identity !== nativeObservation || status.rtCore.timeNs > lastNativeTimeNs)) {
    nativeObservation = identity;
    lastNativeTimeNs = status.rtCore.timeNs;
  }
}

/** The measured pose a recording is made from, or OLP's reason there is none. */
export function teachObservation(input: { selected: boolean; busy: boolean; statusAgeMs: number | null; cellModel: string; cellId: string }) {
  const reachable = machineReachable(input.selected, status, input.statusAgeMs);
  const observation = teachObservationOf({
    reachable, busy: teachBusy(input.busy), status, capabilities, still, settings: cellSettingsOf(status, capabilities, reachable),
    observedAtMs: Date.now() - (input.statusAgeMs ?? 0), cellModel: input.cellModel, cellId: input.cellId,
  });
  if (!still) throw new Error("Release jog and wait for fresh stationary position samples before recording");
  return observation;
}

/** The settings plus the machine's own calibration, from a GET /cell reply (Dense panel fetchCellSettings). */
export function fetchedCellSettings(selected: boolean, statusAgeMs: number | null, payload: unknown) {
  const settings = cellSettingsOf(status, capabilities, machineReachable(selected, status, statusAgeMs));
  if (!settings) return null;
  const record = payload && typeof payload === "object" ? payload as Record<string, unknown> : {};
  const robot = normalizeRobotCell(record.robot_cell);
  if (!robot) throw new Error("the target named no robot identity");
  if (!robot.valid) throw new Error(`robot_cell_unavailable${robot.error ? `: ${robot.error}` : ""}`);
  return {
    ...settings, modelId: robot.modelId, robotDescriptionSha256: robot.robotDescriptionSha256,
    machinePlanningCalibrationSha256: robot.machinePlanningCalibrationSha256, cellFetchedAt: new Date().toISOString(),
    machinePlanningCalibrationBase64: typeof record.machine_planning_calibration_base64 === "string" ? record.machine_planning_calibration_base64 || null : null,
    machinePlanningCalibrationSource: "target" as const,
  };
}

/** The server did not answer: identity is kept, no servo fact survives. */
export function observeSilence(error: string): void {
  still = null; teachSettle.reset();
  status = status ? { ...status, rtCore: null, armed: null, daemonError: error } : null;
}

/** The measured joint positions by axis id, as the Jog table reads them:
 *  rt-core's logical position in the axis unit, or null when the machine
 *  has not reported one. Cheap enough for every status of a fast poll. */
export function livePose(): { id: string; position: number | null }[] {
  return (capabilities?.rtCore?.axes ?? []).map((axis, index) => ({
    id: axis.id, position: status?.rtCore?.axes[index]?.position ?? null,
  }));
}

/** The last applied status and capabilities, for the Run and Telemetry facades. */
export function machineStatus(): DenseStatus | null {
  return status;
}

export function machineCapabilities(): DenseCapabilities | null {
  return capabilities;
}

export function forgetMachine(): void {
  teachSettle.reset();
  status = null;
  capabilities = null;
  // A stillness window or clock domain never outlives its connection.
  still = null;
  nativeObservation = "";
  lastNativeTimeNs = 0;
}

export interface MachineInput {
  catalogue: CellTarget[] | null;
  catalogueError: string | null;
  highlighted: CellTarget | null;
  connectedCell: string;
  connectedLabel: string;
  modelId: string;
  connected: boolean;
  cellPending: boolean;
  connectError: string | null;
  inFlight: boolean;
  posture: CellTarget | null;
  armConfirmPending: boolean;
  statusAgeMs: number | null;
  pendingRequest: MachineRequest | null;
  lastRequest: { request: MachineRequest; error: string | null } | null;
  jogBusy: boolean;
  fraction: number;
  /** The maintenance notice's words while one is up (Machine::setMaintenance). */
  maintenance?: string | null;
}

/** Everything the pages paint about the machine, in OLP's words. */
export function machineView(input: MachineInput) {
  // As main.ts syncConnectedRobot: drift and adoption compare against the
  // connected machine's description while one is connected.
  const settings = cellSettingsOf(status, capabilities, machineReachable(input.connected, status, input.statusAgeMs));
  setConnectedRobot(input.connected && settings?.robotModelId && settings.robotDescriptionSha256
    ? { modelId: settings.robotModelId, robotDescriptionSha256: settings.robotDescriptionSha256 } : null);
  const flow = deriveMachineFlow({ ...input, posture: input.posture as never, capabilities, status });
  const described = capabilities?.rtCore?.axes ?? [];
  const home = deriveHomeStatus({ status, capabilities, connected: input.connected, statusAgeMs: input.statusAgeMs });
  const live = status?.axes ?? [];
  const joints = described.map((axis, position) => {
    const index = axis.index ?? position;
    const observed = live.find((candidate) => candidate.index === index);
    const homed = home.axes.find((candidate) => candidate.index === index);
    // As the Jog table reads it: rt-core's logical position in the axis unit,
    // falling back to the legacy daemon's q_rad.
    const logical = status?.rtCore?.axes[position]?.position;
    return {
      index, id: axis.id, unit: axis.position_unit,
      positionRad: logical ?? observed?.qRad ?? null, minRad: axis.min_position, maxRad: axis.max_position,
      maxVelocityRad: axis.max_velocity, home: homed?.label ?? "", homeState: homed?.state ?? "",
    };
  });
  return {
    flow,
    target: status?.target ?? null,
    session: status?.session ?? null,
    leaseValid: input.connected && flow.arm.state !== "unknown" ? status?.rtCore?.coreLeaseValid ?? null : null,
    cell: capabilities?.cell ?? "",
    executionState: denseExecutionState(status),
    roundTripMs: status?.roundTripMS ?? null,
    cartesianDefaultMode: cartesianDefaultMode(status?.roundTripMS),
    jogDisabled: jointJogDisabled(input.connected, capabilities, status, input.jogBusy, flow.arm, input.modelId),
    // All joints to 0° recovers a disarmed or stuck robot, so it has its own gate.
    zeroDisabled: recoverToZeroDisabled(input.connected, capabilities, status, input.jogBusy, input.modelId) ||
      (capabilities?.recoverZeroJogSpeeds ? "" : "Update the cell software to return to zero at the selected jog speeds"),
    speedNote: jointJogSpeedNote(input.fraction, described),
    fractions: [...JOINT_JOG_FRACTIONS],
    joints,
    cursor: status?.cursor ?? null,
  };
}

/** OLP's "All joints to 0°": one move of every rotary joint not already within
 *  the zero tolerance, by its native logical position. Empty when all are home. */
export function zeroMoves(): { axis: number; degrees: number }[] {
  const moves: { axis: number; degrees: number }[] = [];
  (capabilities?.rtCore?.axes ?? []).forEach((axis, index) => {
    if (axis?.position_unit !== "rad") return;
    const position = status?.rtCore?.axes[index]?.position;
    if (position == null || !Number.isFinite(position)) return;
    const degrees = position * 180 / Math.PI;
    if (Math.abs(degrees) < JOINT_ZERO_TOLERANCE_DEG) return;
    moves.push({ axis: index, degrees: -degrees });
  });
  return moves;
}

/** The body of one All joints to 0° press (OLP's recovery,
 *  `recoverToZero.ts`): the first asks; `confirmed` is the answer the
 *  operator confirmed, whose joints go back for the server to check. */
export function recoverBody(fraction: number, confirmed?: unknown, speeds?: RecoverJogSpeeds): Record<string, unknown> {
  return recoverRequest(fraction, confirmed === undefined || confirmed === null ? undefined : normalizeRecoverResult(confirmed), speeds);
}

/** One recovery reply in OLP's words: `confirm_arm` carries the confirmation
 *  to show and the result to send back; anything else carries the line to
 *  show (a refusal is the server's sentence, after what cleared). */
export function recoverAnswer(payload: unknown, ok: boolean, code = "", detail = "", request?: unknown) {
  if (!ok) return { state: "refused", text: recoverFailureText(code, detail, payload ?? undefined) };
  const result = normalizeRecoverResult(payload);
  if (result.state === "confirm_arm") return { state: result.state, text: recoverOutcomeText(result), confirmation: recoverConfirmation(result, request), result };
  return { state: result.state, text: recoverOutcomeText(result) };
}

/** Refuse a move-by-angle whose endpoint leaves the described range, in OLP's words, before it is sent. */
export function moveRefusal(index: number, degrees: number): string {
  const axis = (capabilities?.rtCore?.axes ?? [])[index];
  const origin = status?.rtCore?.axes[index]?.position ?? null;
  const refusal = axis ? jointDescribedRangeRefusal({ id: axis.id, position_unit: axis.position_unit,
    min_position: axis.min_position, max_position: axis.max_position, origin_rad: origin }, degrees) : null;
  return refusal?.message ?? "";
}

/** The twist a Cartesian hold sends, from OLP's hold rule and the operator's boxes. */
export function holdTwist(axis: number, direction: number, distanceMM: number, angleDegrees: number, fraction: number): number[] {
  return cartesianHoldTwist(axis, direction, requireCartesianSteps({ distanceMM, angleDegrees }), fraction);
}

export function stepBody(intent: { target_id: string; session_id: string; revision: number; axis: number; degrees: number; fraction: number }, frame: string) {
  return cartesianMoveBody(intent, frame);
}

export function stepLabels(mode: CartesianMode) {
  return cartesianStepLabels(mode);
}

/** A typed server or rt-core code in operator words; unknown codes stay visible. */
export function reasonText(code: string, detail = ""): string {
  return cartesianCancelText(code) ?? (machineReasonText(code) || rtCoreReasonText(code, detail));
}

/** A refused Arm in the Machine panel's words: the standing causes from the
 *  last status (drive alarm, unconfirmed Stop, maintenance), else the code. */
export function armRefusalText(code: string, detail = "", maintenance = ""): string {
  return `Arm refused: ${armRefusalWords(armBlockers(status, maintenance || null), detail && detail !== code ? `${code}: ${detail}` : code)}`;
}

/** Go to plan start's moves: every axis to an absolute position, each checked
 *  against its described range as OLP's multi-axis move checks it. */
export function absoluteMoves(positions: number[]): { moves: { axis: number; degrees: number; position_rad: number }[]; refusal: string } {
  const axes = capabilities?.rtCore?.axes ?? [];
  if (positions.length !== axes.length || positions.some((q, i) => !Number.isFinite(q) || axes[i]?.position_unit !== "rad"))
    return { moves: [], refusal: "Plan start does not match the machine's rotary axes" };
  let refusal = "";
  const moves = positions.map((position_rad, axis) => {
    const origin = status?.rtCore?.axes[axis]?.position ?? null;
    const described = axes[axis]!;
    const degrees = (position_rad - (origin ?? Number.NaN)) * 180 / Math.PI;
    const refused = jointDescribedRangeRefusal({ id: described.id, position_unit: described.position_unit,
      min_position: described.min_position, max_position: described.max_position, origin_rad: origin }, degrees);
    if (refused && !refusal) refusal = refused.message;
    return { axis, degrees: 0, position_rad };
  });
  return { moves, refusal };
}
