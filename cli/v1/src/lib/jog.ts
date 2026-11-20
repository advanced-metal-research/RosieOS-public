// Bounded jogs, through OLP only, with the browser's own requests; shared by
// the CLI and the MCP server:
//
// - Move joints: Move by angle / All joints to 0° / Go to plan start
//   (POST dense-execution/move; ui/src/execution/jointJog.ts JointAngleMove,
//   olp-core machine.ts zeroMoves and absoluteMoves).
// - Cartesian step: one straight TCP step (POST dense-execution/cartesian/move;
//   cartesianMove.ts cartesianMoveBody, olp-core stepBody).
// - Timed hold: the hold-to-run buttons with a timer for a hand
//   (jog/{begin,update,end}, cartesian/{start,intent,halt}; jointJog.ts
//   JointJogHold, cartesianJog.ts cartesianJogAPI).
//
// An agent has no hand on a trigger, so every call is bounded here before
// OLP sees it: travel per call (config.jog), speed fraction and hold
// duration, on top of OLP's and rt-core's own limits. A hold always ends with
// End/Halt from a finally block; once OLP accepted the press, Stop follows
// when that End fails or the hold met an error, and a refused press gets End
// only. Every motion request is beaten around (keeper.around). OLP's refusals
// (jog_mode_conflict while a program plays or a loop runs, no_grant,
// joint_move_limits, ...) come back in its words and are never retried. Each
// call is admitted by robot.ts `admit` and fenced with that read's selection.
import type { Context } from "./context.js";
import { refusalText, type HttpReply } from "./http.js";
import { DENSE, type DenseStatus, type TargetIdentity } from "./olp.js";
import { admit, refusalOf, replyBody, RobotRefusal } from "./robot.js";
import { errorMessage, record } from "./text.js";

// OLP's jointMoveVelocityReserve (benchdrive, measured 2026-09-14): a move or
// joint hold at fraction f runs at f x 0.75 x the described maximum velocity.
const VELOCITY_RESERVE = 0.75;
// The browser's cadence (jointJog.ts JOINT_JOG_CADENCE_MS, unverified 50 ms)
// and its in-flight bound when OLP reports pipelined_intents
// (JOINT_JOG_MAX_IN_FLIGHT: one 250 ms input lifetime of cadence).
export const CADENCE_MS = 50;
const MAX_IN_FLIGHT = 5;
// An intent older than a Rosie 1430 cell's 250 ms input lifetime is useless;
// one that
// takes a second is a lost link, and the hold ends on it.
const INTENT_TIMEOUT_MS = 1000;
// OLP's JOINT_ZERO_TOLERANCE_DEG: an absolute target nearer than this is
// already reached (two hundredths of a degree, under the display's decimals).
export const AT_TARGET_DEG = 0.02;
// A Cartesian step owns its execution in OLP whatever this request does
// (CartesianMove runs WithoutCancel), so its wait only bounds this call.
const STEP_TIMEOUT_MS = 300_000;

export const CARTESIAN_AXES = ["x", "y", "z", "rx", "ry", "rz"] as const;
export type CartesianAxis = (typeof CARTESIAN_AXES)[number];

export interface DescribedAxis {
  index?: number;
  id: string;
  position_unit: string;
  min_position: number;
  max_position: number;
  max_velocity?: number;
  jog_acceleration?: number;
  max_acceleration?: number;
}

/** GET jog/state and cartesian/state (denseexec.JointJogState); fields read here. */
interface JogState {
  active: boolean;
  revision: number;
  reason?: string;
  move?: { axis: number; state: string; reason: string; revision: number } | null;
  pipelined_intents?: boolean;
  last_end?: Record<string, unknown> | null;
}

/** What one jog call needs from OLP, read fresh at its start. */
interface Machine {
  target: TargetIdentity;
  status: DenseStatus;
  /** capabilities.cell: the rt-core binding's pair id, which OLP checks as target_id (not target.cell_id). */
  cell: string;
  sessionId: string;
  axes: DescribedAxis[];
  /** Live logical position per described axis (rad), null when not valid. */
  positions: Array<number | null>;
}

const finite = (value: unknown): value is number => typeof value === "number" && Number.isFinite(value);
export const toDeg = (rad: number) => (rad * 180) / Math.PI;
export const fmt = (deg: number) => `${deg.toFixed(2)}°`;

/** OLP's refusal code, in the alphabet an end_cause accepts (jogEndCause.ts refusalEndCause). */
function codeOf(reply: HttpReply): string {
  const code = record(reply.body).error;
  return typeof code === "string" && /^[a-z0-9_.:-]{1,64}$/.test(code) ? code : "client_error";
}

/** OLP answers jog_mode_conflict for every "something else moves" case; say which one the status shows. */
export function conflictHint(reply: HttpReply | null, status: DenseStatus): string {
  if (!reply || codeOf(reply) !== "jog_mode_conflict") return "";
  const loop = record(status.loop);
  if (loop.state === "running" || loop.state === "finishing") return " A program loop is running: OLP refuses jogging until it ends (Finish cycle, or Stop).";
  if (status.session?.state === "playing") return " A program is playing: OLP refuses jogging until it is paused or stopped.";
  return " Another motion (a hold, move or program) is active on this cell.";
}

export function describedAxes(capabilities: Record<string, unknown>, status: DenseStatus): DescribedAxis[] {
  const fromCaps = record(record(capabilities.rt_core).description).axes;
  const fromStatus = record(record(status.rt_core).description).axes;
  const axes = Array.isArray(fromCaps) ? fromCaps : Array.isArray(fromStatus) ? fromStatus : [];
  return axes.map(record).map((axis, index) => ({
    index: finite(axis.index) ? axis.index : index,
    id: typeof axis.id === "string" && axis.id ? axis.id : `J${index + 1}`,
    position_unit: String(axis.position_unit ?? ""),
    min_position: Number(axis.min_position),
    max_position: Number(axis.max_position),
    max_velocity: finite(axis.max_velocity) ? axis.max_velocity : undefined,
    jog_acceleration: finite(axis.jog_acceleration) ? axis.jog_acceleration : undefined,
    max_acceleration: finite(axis.max_acceleration) ? axis.max_acceleration : undefined,
  }));
}

/** rt-core's logical position per axis, as the Jog table reads it (rtCoreStatus.ts mapRTCoreStatus). */
export function livePositions(status: DenseStatus, count: number): Array<number | null> {
  const live = record(record(status.rt_core).status).axes;
  const axes = Array.isArray(live) ? live.map(record) : [];
  return Array.from({ length: count }, (_, i) => {
    const axis = axes[i];
    return axis && axis.logical_valid === true && finite(axis.logical_position) ? axis.logical_position : null;
  });
}

function jointIndex(axes: DescribedAxis[], name: string): number | string {
  const wanted = name.trim().toLowerCase();
  const index = axes.findIndex((axis) => axis.id.toLowerCase() === wanted);
  return index >= 0 ? index : `no described joint '${name}' (this cell has ${axes.map((axis) => axis.id).join(", ") || "none"})`;
}

/**
 * A rest-to-rest move's duration, bounded above: each joint alone at its
 * speed and acceleration at the fraction (OLP newMoveJoint), the slowest
 * governing all (one shared progress). OLP stops the move when this request
 * is cancelled (denseJointMove), so the wait must outlast it.
 */
function moveTimeoutMs(moves: Array<{ index: number; deltaRad: number }>, axes: DescribedAxis[], fraction: number): number {
  let longest = 0;
  for (const move of moves) {
    const axis = axes[move.index]!;
    const velocity = (axis.max_velocity ?? NaN) * fraction * VELOCITY_RESERVE;
    const acceleration = Math.min(axis.jog_acceleration ?? NaN, axis.max_acceleration ?? Infinity) * fraction;
    if (!(velocity > 0) || !(acceleration > 0)) return 600_000;
    longest = Math.max(longest, Math.abs(move.deltaRad) / velocity + velocity / acceleration);
  }
  return Math.ceil(Math.min(900_000, 30_000 + 2_000 * longest));
}

/** Admission, then the capabilities and live positions the request is built from. */
async function machine(ctx: Context, action: string): Promise<Machine> {
  const { status, target } = await admit(ctx, action);
  if (!status.session?.id) throw new RobotRefusal(`${action} refused: OLP has no execution session on ${target.cell_id}. Arm first.`, { status });
  const capabilities = await ctx.olp.get(`${DENSE}/capabilities`);
  if (!capabilities.ok) throw refusalOf("capabilities", capabilities);
  const caps = record(capabilities.body);
  const cell = typeof caps.cell === "string" ? caps.cell : "";
  if (!cell) throw new RobotRefusal(`${action} refused: OLP's capabilities name no cell binding (target_id); describe the selected rt-core target first.`, caps);
  const axes = describedAxes(caps, status);
  return { target, status, cell, sessionId: status.session.id, axes, positions: livePositions(status, axes.length) };
}

/** The hold/move lane's state; a live hold or move refuses here in the browser's words. */
async function laneState(ctx: Context, route: string, m: Machine, action: string): Promise<JogState> {
  const reply = await ctx.olp.get(`${DENSE}/${route}`);
  if (!reply.ok) throw refusalOf(`${action} (reading ${route})`, reply);
  const state = reply.body as JogState;
  if (!Number.isSafeInteger(state.revision) || typeof state.active !== "boolean") throw new RobotRefusal(`OLP answered an invalid ${route}: ${JSON.stringify(reply.body).slice(0, 300)}`);
  if (state.active || state.move?.state === "preparing" || state.move?.state === "moving") {
    throw new RobotRefusal(`${action} refused: jog_mode_conflict: Another target motion is active on ${m.target.cell_id}.`, { state });
  }
  return state;
}

/** Correlated native completion, as the browser accepts it: done, this axis, a revision past the intent's. */
function completed(reply: HttpReply, axis: number, revision: number): boolean {
  const data = record(reply.body);
  return data.state === "done" && data.axis === axis && Number.isSafeInteger(data.revision) && Number(data.revision) > revision;
}

async function lastJogEnd(ctx: Context, route: string): Promise<Record<string, unknown> | null> {
  try {
    const status = await ctx.olp.status();
    if (status.last_jog_end) return status.last_jog_end;
    const state = await ctx.olp.get(`${DENSE}/${route}`);
    return state.ok ? (record(state.body).last_end as Record<string, unknown> | null) ?? null : null;
  } catch {
    return null;
  }
}

function refusedWithHint(action: string, reply: HttpReply, status: DenseStatus): RobotRefusal {
  const base = refusalOf(action, reply);
  return new RobotRefusal(`${base.message}${conflictHint(reply, status)}`, base.data);
}

export interface PlannedJoint { index: number; id: string; from: number | null; deltaDeg: number; toDeg: number | null; target?: number }

export interface MoveResult {
  /** Nothing was sent: every listed joint already within AT_TARGET_DEG of its target. */
  nothingToMove: boolean;
  fraction: number;
  planned: PlannedJoint[];
  /** Live positions (rad) after the move, per described axis. */
  now: Array<number | null>;
  request?: Record<string, unknown>;
  reply?: unknown;
  revision?: unknown;
}

/** One synchronised joint-space move: relative (signed travel) or absolute (target angles), bounded before anything is sent. */
export async function moveJoints(ctx: Context, input: { mode: "relative" | "absolute"; joints: { joint: string; deg: number }[]; fraction: number }): Promise<MoveResult> {
  const limits = ctx.config.jog;
  const variable = `${ctx.config.variablePrefix}JOG_MAX_DEG`;
  const { mode: kind, joints, fraction } = input;
  if (fraction < 0.01 || fraction > limits.maxMoveFraction) throw new RobotRefusal(`Move joints refused: fraction ${fraction} is outside 0.01-${limits.maxMoveFraction} (${ctx.config.variablePrefix}JOG_MAX_FRACTION)`);
  if (kind === "relative" && joints.some((j) => j.deg === 0)) throw new RobotRefusal("relative moves need a nonzero signed angle for every joint");
  const m = await machine(ctx, "Move joints");
  const planned: PlannedJoint[] = [];
  for (const { joint, deg } of joints) {
    const index = jointIndex(m.axes, joint);
    if (typeof index === "string") throw new RobotRefusal(`Move joints refused: ${index}`);
    if (planned.some((p) => p.index === index)) throw new RobotRefusal(`Move joints refused: ${joint} is listed twice`);
    const axis = m.axes[index]!;
    if (axis.position_unit !== "rad") throw new RobotRefusal(`Move joints refused: ${axis.id} is not a rotary axis described in radians (joint_move_units)`);
    const origin = m.positions[index] ?? null;
    if (kind === "absolute") {
      if (origin === null) throw new RobotRefusal(`Move joints refused: ${axis.id} has no valid live position, so the travel to ${fmt(deg)} cannot be bounded`);
      planned.push({ index, id: axis.id, from: toDeg(origin), deltaDeg: deg - toDeg(origin), toDeg: deg, target: (deg * Math.PI) / 180 });
    } else {
      planned.push({ index, id: axis.id, from: origin === null ? null : toDeg(origin), deltaDeg: deg, toDeg: origin === null ? null : toDeg(origin) + deg });
    }
  }
  const overCap = planned.filter((p) => Math.abs(p.deltaDeg) > limits.maxJointDeg);
  if (overCap.length) {
    throw new RobotRefusal(
      `Move joints refused: ${overCap.map((p) => `${p.id} would travel ${fmt(p.deltaDeg)}`).join(", ")}; the limit is ${limits.maxJointDeg}° per joint per call (${variable}). Nothing was sent. Split the move, or ask the owner to change the limit.`,
      { planned, max_joint_deg: limits.maxJointDeg },
    );
  }
  for (const p of planned) {
    const axis = m.axes[p.index]!;
    if (p.toDeg === null || !finite(axis.min_position) || !finite(axis.max_position)) continue;
    const end = (p.toDeg * Math.PI) / 180;
    if (end < axis.min_position || end > axis.max_position) {
      throw new RobotRefusal(`Move joints refused: joint_move_limits: ${p.id} would end at ${fmt(p.toDeg)}, outside its described range ${fmt(toDeg(axis.min_position))} to ${fmt(toDeg(axis.max_position))}. Nothing was sent.`, { planned });
    }
  }
  const moving = kind === "absolute" ? planned.filter((p) => Math.abs(p.deltaDeg) >= AT_TARGET_DEG) : planned;
  if (!moving.length) return { nothingToMove: true, fraction, planned, now: m.positions };

  const state = await laneState(ctx, "jog/state", m, "Move joints");
  const moves = moving.map((p) => (p.target !== undefined ? { axis: p.index, degrees: 0, position_rad: p.target } : { axis: p.index, degrees: p.deltaDeg }));
  const first = moves[0]!;
  // One relative joint is the GO button (no list); several, or any
  // absolute target, is beginMany's list with the first entry repeated.
  const listed = kind === "absolute" || moves.length > 1;
  const intent = { target_id: m.cell, session_id: m.sessionId, revision: state.revision, axis: first.axis, degrees: first.degrees, fraction, ...(listed ? { axes: moves } : {}) };
  const timeoutMs = moveTimeoutMs(moving.map((p) => ({ index: p.index, deltaRad: (p.deltaDeg * Math.PI) / 180 })), m.axes, fraction);
  const reply = await ctx.keeper.around(
    () => ctx.olp.post(`${DENSE}/move`, intent, { target: m.target, timeoutMs }),
    (answer) => answer.ok && completed(answer, first.axis, state.revision),
  );
  if (!reply.ok) throw refusedWithHint("joint move", reply, m.status);
  if (!completed(reply, first.axis, state.revision)) throw new RobotRefusal(`Move completion was not observed (rt_core_protocol_error): OLP answered ${JSON.stringify(reply.body).slice(0, 300)}`, replyBody(reply));
  const after = await ctx.olp.status().catch(() => null);
  return { nothingToMove: false, fraction, planned: moving, now: after ? livePositions(after, m.axes.length) : [], request: intent, reply: reply.body, revision: record(reply.body).revision };
}

/** One straight TCP step from the machine's held pose (never a pose sent from here); returns on correlated completion. */
export async function cartesianStep(ctx: Context, input: { frame: "base" | "tool"; axis: CartesianAxis; amount: number; fraction: number }): Promise<{ request: Record<string, unknown>; reply: unknown; unit: string }> {
  const limits = ctx.config.jog;
  const { frame, axis, amount, fraction } = input;
  const index = CARTESIAN_AXES.indexOf(axis);
  if (index < 0) throw new RobotRefusal(`Cartesian step: axis is one of ${CARTESIAN_AXES.join(", ")}`);
  const linear = index < 3;
  const cap = linear ? limits.maxCartesianMm : limits.maxCartesianDeg;
  const unit = linear ? "mm" : "°";
  if (amount === 0) throw new RobotRefusal("Cartesian step needs a nonzero signed amount");
  if (fraction < 0.01 || fraction > limits.maxMoveFraction) throw new RobotRefusal(`Cartesian step refused: fraction ${fraction} is outside 0.01-${limits.maxMoveFraction}`);
  if (Math.abs(amount) > cap) {
    throw new RobotRefusal(`Cartesian step refused: ${axis} ${amount} ${unit} exceeds ${cap} ${unit} per call (${ctx.config.variablePrefix}CARTESIAN_MAX_${linear ? "MM" : "DEG"}). Nothing was sent.`, { axis, amount, cap });
  }
  const m = await machine(ctx, "Cartesian step");
  const state = await laneState(ctx, "cartesian/state", m, "Cartesian step");
  const request = { target_id: m.cell, session_id: m.sessionId, revision: state.revision, frame, axis: index, distance_mm: linear ? amount : 0, angle_degrees: linear ? 0 : amount, fraction };
  const reply = await ctx.keeper.around(
    () => ctx.olp.post(`${DENSE}/cartesian/move`, request, { target: m.target, timeoutMs: STEP_TIMEOUT_MS }),
    (answer) => answer.ok && completed(answer, index, state.revision),
  );
  if (!reply.ok) throw refusedWithHint("Cartesian step", reply, m.status);
  if (!completed(reply, index, state.revision)) throw new RobotRefusal(`Cartesian completion was not observed (rt_core_protocol_error): OLP answered ${JSON.stringify(reply.body).slice(0, 300)}`, replyBody(reply));
  return { request, reply: reply.body, unit };
}

export interface HoldInput {
  kind: "joint" | "cartesian";
  joint?: string;
  axis?: CartesianAxis;
  frame: "base" | "tool";
  direction: "positive" | "negative";
  durationS: number;
  fraction: number;
  /** Cartesian speed at 100 % (the browser's boxes): mm/s and °/s. */
  speedMmS: number;
  speedDegS: number;
}

export interface HoldOutcome {
  ok: boolean;
  label: string;
  summary: string;
  updates: number;
  heldMs: number;
  endCause: string;
  endText: string;
  stopText: string;
  /** OLP's classified jog end (code, text, re-pull). */
  lastJogEnd: Record<string, unknown> | null;
  /** The jog_mode_conflict hint, when a refusal was one. */
  hint: string;
  request: Record<string, unknown>;
  routes: { state: string; begin: string; update: string; end: string };
}

/** A hold-to-run jog with a timer for a hand, bounded in duration, fraction and commanded travel. THE ROBOT MOVES for durationS. */
export async function jogHold(ctx: Context, input: HoldInput): Promise<HoldOutcome> {
  const limits = ctx.config.jog;
  const prefix = ctx.config.variablePrefix;
  const direction = input.direction === "positive" ? 1 : -1;
  if (input.kind === "joint" && !input.joint) throw new RobotRefusal("a joint hold needs the joint (e.g. 'J3')");
  if (input.kind === "cartesian" && !input.axis) throw new RobotRefusal("a Cartesian hold needs the axis (x, y, z, rx, ry or rz)");
  if (input.durationS < 0.05 || input.durationS > limits.maxHoldS) throw new RobotRefusal(`Jog hold refused: duration ${input.durationS} s is outside 0.05-${limits.maxHoldS} s (${prefix}JOG_HOLD_MAX_S)`);
  if (input.fraction < 0.01 || input.fraction > limits.maxHoldFraction) throw new RobotRefusal(`Jog hold refused: fraction ${input.fraction} is outside 0.01-${limits.maxHoldFraction} (${prefix}JOG_HOLD_MAX_FRACTION)`);
  const cartesianIndex = input.axis ? CARTESIAN_AXES.indexOf(input.axis) : -1;
  if (input.kind === "cartesian") {
    const linear = cartesianIndex < 3;
    const travel = (linear ? input.speedMmS : input.speedDegS) * input.fraction * input.durationS;
    const cap = linear ? limits.maxCartesianMm : limits.maxCartesianDeg;
    if (travel > cap) {
      throw new RobotRefusal(`Jog hold refused: ${input.axis} at ${linear ? input.speedMmS : input.speedDegS} ${linear ? "mm/s" : "°/s"} x ${input.fraction} for ${input.durationS} s commands ${travel.toFixed(1)} ${linear ? "mm" : "°"}, over ${cap} per call. Nothing was sent.`);
    }
  }
  const m = await machine(ctx, "Jog hold");
  let label: string;
  let intent: Record<string, unknown>;
  let routes: { state: string; begin: string; update: string; end: string };
  if (input.kind === "joint") {
    const index = jointIndex(m.axes, input.joint!);
    if (typeof index === "string") throw new RobotRefusal(`Jog hold refused: ${index}`);
    const axis = m.axes[index]!;
    if (!(axis.max_velocity! > 0)) throw new RobotRefusal(`Jog hold refused: ${axis.id} describes no max_velocity, so the hold's travel cannot be bounded`);
    const travel = toDeg(axis.max_velocity! * input.fraction * VELOCITY_RESERVE) * input.durationS;
    if (travel > limits.maxJointDeg) {
      throw new RobotRefusal(`Jog hold refused: ${axis.id} at fraction ${input.fraction} for ${input.durationS} s commands ${fmt(travel)}, over ${limits.maxJointDeg}° per call (${prefix}JOG_MAX_DEG). Nothing was sent.`);
    }
    label = `${axis.id} ${direction > 0 ? "+" : "-"}`;
    routes = { state: "jog/state", begin: "jog/begin", update: "jog/update", end: "jog/end" };
    intent = { axis: index, direction };
  } else {
    // cartesianHoldTwist: the box is the speed at 100 % in SI; the resolver applies the fraction.
    const twist = [0, 0, 0, 0, 0, 0];
    twist[cartesianIndex] = cartesianIndex < 3 ? (direction * input.speedMmS) / 1000 : (direction * input.speedDegS * Math.PI) / 180;
    label = `${input.axis} ${direction > 0 ? "+" : "-"} (${input.frame})`;
    routes = { state: "cartesian/state", begin: "cartesian/start", update: "cartesian/intent", end: "cartesian/halt" };
    intent = { frame: input.frame, twist };
  }
  const state = await laneState(ctx, routes.state, m, "Jog hold");
  const hold = { target_id: m.cell, session_id: m.sessionId, revision: state.revision, ...intent, fraction: input.fraction };
  const outcome = await ctx.keeper.around(() => runHold(ctx, hold, routes, m, state, input.durationS), (result) => result.ok);
  const end = await lastJogEnd(ctx, routes.state);
  return { ...outcome, label, lastJogEnd: end, hint: conflictHint(outcome.problem, m.status).trim(), request: hold, routes };
}

interface RunOutcome {
  ok: boolean;
  summary: string;
  updates: number;
  heldMs: number;
  endCause: string;
  endText: string;
  stopText: string;
  problem: HttpReply | null;
}

/**
 * One hold, as JointJogHold runs it: the cadence starts with the press, an
 * intent refused jog_session_stale before the press is answered is not the
 * hold's (the lane may not be open yet), any later refusal or transport
 * failure ends the hold at once. End/Halt goes out from `finally` whatever
 * happened, without waiting for intents still in flight (OLP drops them:
 * End advances the revision); Stop follows only a hold that began (press
 * accepted, or its answer lost) when End did not end it or the hold met an
 * error. A refused press started nothing and is never Stopped.
 */
async function runHold(ctx: Context, hold: Record<string, unknown>, routes: { begin: string; update: string; end: string }, m: Machine, state: JogState, durationS: number): Promise<RunOutcome> {
  const { olp, keeper } = ctx;
  const limit = state.pipelined_intents === true ? MAX_IN_FLIGHT : 1;
  const run = { starting: true, ended: false, inFlight: 0, updates: 0, cause: "", words: "", problem: null as HttpReply | null, refusedBy: "" as "" | "press" | "intent" | "client" };
  let wake = () => {};
  const failed = new Promise<void>((resolve) => (wake = resolve));
  const fail = (by: "press" | "intent" | "client", endCause: string, words: string, reply: HttpReply | null) => {
    if (run.cause) return;
    Object.assign(run, { cause: endCause, words, problem: reply, refusedBy: by });
    wake();
  };
  const drained: Array<Promise<void>> = [];
  const tick = () => {
    if (run.ended || run.inFlight >= limit) return;
    const early = run.starting;
    run.inFlight++;
    run.updates++;
    drained.push(
      olp
        .post(`${DENSE}/${routes.update}`, hold, { target: m.target, timeoutMs: INTENT_TIMEOUT_MS })
        .then((reply) => {
          if (reply.ok || run.ended || (early && codeOf(reply) === "jog_session_stale")) return;
          fail("intent", `intent_refused:${codeOf(reply)}`, `OLP refused an intent (HTTP ${reply.status}): ${refusalText(reply)}`, reply);
        })
        .catch((error: unknown) => {
          if (!run.ended) fail("client", "intent_refused:client_error", `an intent failed: ${errorMessage(error)}`, null);
        })
        .finally(() => {
          run.inFlight--;
        }),
    );
  };
  const timer = setInterval(tick, CADENCE_MS);
  let heldMs = 0;
  let endCause = "button_released";
  let endText = "";
  let endOk = false;
  let stopText = "";
  /** OLP accepted the press, or its answer was lost after it may have: only then can there be motion to Stop. */
  let began = false;
  try {
    const begin = await olp.post(`${DENSE}/${routes.begin}`, hold, { target: m.target });
    run.starting = false;
    if (!begin.ok) {
      fail("press", `intent_refused:${codeOf(begin)}`, `OLP refused the press (HTTP ${begin.status}): ${refusalText(begin)}`, begin);
    } else {
      began = true;
      if (!run.cause) {
        const pressed = Date.now();
        let timeout: NodeJS.Timeout | undefined;
        await Promise.race([new Promise<void>((resolve) => (timeout = setTimeout(resolve, durationS * 1000))), failed]);
        clearTimeout(timeout);
        heldMs = Date.now() - pressed;
      }
    }
  } catch (error) {
    // The press may have been accepted before its answer was lost.
    began = true;
    fail("client", "intent_refused:client_error", `the press failed: ${errorMessage(error)}`, null);
  } finally {
    run.ended = true;
    clearInterval(timer);
    if (run.cause) endCause = run.cause;
    try {
      const end = await olp.post(`${DENSE}/${routes.end}`, { ...hold, end_cause: endCause }, { target: m.target });
      // A refusal OLP already acted on retired this revision: End then meets jog_session_stale and nothing is left to end.
      const retired = !end.ok && codeOf(end) === "jog_session_stale" && (run.refusedBy === "press" || run.refusedBy === "intent");
      endOk = end.ok || retired;
      endText = end.ok ? "accepted" : `refused (HTTP ${end.status}): ${refusalText(end)}${retired ? " (OLP had already ended the hold)" : ""}`;
    } catch (error) {
      endText = `failed: ${errorMessage(error)}`;
    }
    // A refused press started nothing: End only, whatever End answers (OLP
    // refuses a press with jog_mode_conflict while a program plays or a
    // loop runs, and a Stop would end that motion). A hold that began gets
    // Stop when End did not land or the hold failed.
    if (began && (!endOk || run.cause)) {
      keeper.stop("STOP sent after a failed jog hold");
      try {
        const stop = await olp.post(`${DENSE}/stop`, {});
        const data = replyBody(stop);
        stopText = stop.ok
          ? `sent because ${!endOk ? "End did not end the hold" : "the hold failed"}; ${data.stopped ? "confirmed" : "NOT confirmed"} after ${String(data.attempts ?? "?")} attempt(s). The session and grant are released: Arm again before the next motion.`
          : `NOT confirmed (HTTP ${stop.status}): ${refusalText(stop)}. Use the hardware E-stop if the robot may still be moving.`;
      } catch (error) {
        stopText = `FAILED: ${errorMessage(error)}. Use the hardware E-stop if the robot may still be moving.`;
      }
    }
    await Promise.allSettled(drained);
  }
  const ok = !run.cause && endOk && !stopText;
  const summary = ok ? "completed and released" : run.refusedBy === "press" ? `not started: ${run.words}` : run.words ? `ended early: ${run.words}` : `End failed: ${endText}`;
  return { ok, summary, updates: run.updates, heldMs, endCause, endText, stopText, problem: run.problem };
}
