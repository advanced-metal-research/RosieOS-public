import { DENSE_EXECUTION_PATH, type DenseCapabilities, type DenseStatus } from "./denseClient.ts";
import { previewJointRadForAxis } from "./livePose.ts";
import { jointJogBrowserStopText, jointMoveLimitsText, rtCoreReasonText } from "../targets/rtCoreStatus.ts";
import { markBusy } from "../ui/busy.ts";
import { unknownRobotModelReason } from "../targets/cells.ts";
import type { ArmFact } from "./machineFlow.ts";
import { deriveHomeStatus } from "./homeStatus.ts";
import { pointerEndCause, refusalEndCause, windowEndCause, type JogEndCause } from "./jogEndCause.ts";
import {
  RECOVER_TO_ZERO_PATH, normalizeRecoverResult, recoverConfirmation, recoverFailureText, recoverOutcomeText, recoverRequest, recoverToZeroDisabled,
  type RecoverConfirmation, type RecoverResult,
} from "./recoverToZero.ts";

// Unverified browser cadence: 50 ms; server expires input after 100 ms.
export const JOINT_JOG_CADENCE_MS = 50;
// Unverified operator policy: fractions of each joint's described maximum
// velocity, up to the whole of it (owner decision 2026-09-21); the server puts
// its measured 0.75 drive reserve under every fraction, so 100 % never
// commands the limit itself.
export const JOINT_JOG_FRACTIONS = [0.05, 0.1, 0.25, 0.5, 0.75, 1] as const;
// The reserve the server applies (jointMoveVelocityReserve, measured 2026-09-14).
export const JOINT_JOG_VELOCITY_RESERVE = 0.75;

/** What the Speed fraction is a fraction of, for the note beside the select:
 *  each rotary joint's described maximum velocity, in degrees per second, and
 *  the resulting jog speed. One number when every joint shares a maximum; a
 *  range naming the slowest and fastest joints otherwise; at 100 % only the
 *  maximum, since the result would repeat it (owner 2026-09-21: the note read
 *  "(2.86°/s to 11.46°/s): 2.86°/s to 11.46°/s"). Empty when the description
 *  carries no maximum. Decimals shrink as the numbers grow: 0.57, 11.5, 180. */
export function jointJogSpeedNote(fraction: number, axes: readonly { id?: string; position_unit?: string; max_velocity?: number }[]): string {
  const rated = axes.filter(a => a.position_unit === "rad" && typeof a.max_velocity === "number" && Number.isFinite(a.max_velocity) && a.max_velocity > 0)
    .map(a => ({ id: a.id ?? "", deg: a.max_velocity! * 180 / Math.PI }));
  if (rated.length === 0) return "";
  const deg = (v: number) => `${v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v.toFixed(2)}°/s`;
  const lo = rated.reduce((a, b) => (b.deg < a.deg ? b : a)), hi = rated.reduce((a, b) => (b.deg > a.deg ? b : a));
  const whole = fraction >= 1;
  if (hi.deg - lo.deg < 0.005) {
    return whole ? `of each joint's described maximum ${deg(hi.deg)}` : `of each joint's described maximum ${deg(hi.deg)}: ${deg(hi.deg * fraction)}`;
  }
  const named = (x: { id: string; deg: number }) => (x.id ? `${deg(x.deg)} (${x.id})` : deg(x.deg));
  const range = `${named(lo)} to ${named(hi)}`;
  return whole ? `of each joint's described maximum, ${range}` : `of each joint's described maximum, ${range}: ${deg(lo.deg * fraction)} to ${deg(hi.deg * fraction)}`;
}

interface Intent {
  target_id: string; session_id: string; revision: number;
  axis: number; direction: number; fraction: number;
  /** End only: why this client ended the hold (the server journals and
   *  classifies it; `jogEndCause.ts`). */
  end_cause?: string;
}
/** One joint of a several-axis move: a signed delta in degrees. */
export interface MoveAxis { axis: number; degrees: number; position_rad?: number }
/** The described range and live origin for a client-side joint_move_limits
 *  check: min/max are the axis description's min_position/max_position in rad. */
export interface DescribedJoint {
  id: string;
  position_unit: string;
  min_position: number;
  max_position: number;
  origin_rad: number | null;
}

/** Refuse a signed step whose endpoint is outside the described range, in the
 *  same words the row will show, so a hopeless GO never reaches the server. */
export function jointDescribedRangeRefusal(axis: DescribedJoint | undefined, degrees: number): JointJogError | null {
  if (!axis || axis.position_unit !== "rad") return null;
  const min = axis.min_position, max = axis.max_position, origin = axis.origin_rad;
  if (!Number.isFinite(min) || !Number.isFinite(max) || min >= max || origin == null || !Number.isFinite(origin)) return null;
  const delta = degrees * Math.PI / 180;
  if (origin < min || origin > max || origin + delta < min || origin + delta > max || Math.abs(delta) > max - min) {
    return new JointJogError("joint_move_limits", jointMoveLimitsText(axis.id, min, max));
  }
  return null;
}
/** `axes`, when present, names every joint of one synchronised move; `axis` and
 * `degrees` are then its first entry (the server plans them as one trajectory). */
interface MoveIntent { target_id: string; session_id: string; revision: number; axis: number; degrees: number; fraction: number; axes?: MoveAxis[] }
interface MoveState { axis: number; state: string; reason: string; revision: number; detail?: string; axes?: number[] }
interface JogState {
  active: boolean; revision: number; reason: string; move?: MoveState;
  /** Path holds (cell advertises path_jog): fraction may change within a Cartesian hold. */
  fraction_updates?: boolean;
  stopping?: boolean; governing_limit?: string; stop_reason?: string; path_speed?: number;
  /** The server orders intents that arrive together (an overtaken one is
   *  dropped), so several may be in flight. Absent: one at a time. */
  pipelined_intents?: boolean;
}
type TargetIdentity = DenseStatus["target"];
export interface JointJogAPI {
  state(target?: TargetIdentity): Promise<JogState>;
  /** `signal` aborts an intent the hold no longer needs (its release). */
  send(action: "begin" | "update" | "end", intent: Intent, target?: TargetIdentity, signal?: AbortSignal): Promise<JogState>;
  move?(intent: MoveIntent, signal: AbortSignal, target?: TargetIdentity): Promise<MoveState>;
  /** One press of the recovery (`recoverToZero.ts`); a refusal throws a
   *  JointJogError whose message is the operator's sentence. */
  recover?(body: Record<string, unknown>, signal: AbortSignal, target?: TargetIdentity): Promise<RecoverResult>;
}

export class JointJogError extends Error {
  readonly code: string;
  constructor(code: string, detail: string) { super(detail); this.code = code; }
}

/** The reason code a move row carries when the failure was thrown in this
 * browser rather than typed by the server; `detail` then holds the words. */
export const JOINT_JOG_CLIENT_ERROR = "client_error";

/** Operator text for a failed jog request. A typed server refusal keeps its
 * code and label. Anything else was thrown in this browser (a bug, a rejected
 * fetch) and is shown as `client error: <message>`; it is never dressed as
 * `rt_core_transport_lost`, which is the server's word for losing the machine
 * and turns the arm state UNKNOWN in the Machine panel. The LA bench showed
 * `rt_core_transport_lost: Illegal invocation` under the joints while the
 * machine was armed and answering: a TypeError from this file, not the cell. */
export function jointJogFailureText(error: unknown): string {
  if (error instanceof JointJogError) return rtCoreReasonText(error.code, error.message);
  return `client error: ${error instanceof Error ? error.message : String(error)}`;
}

/** Preserve every typed server refusal; network errors disable the held input
 * instead of replaying requests when connectivity returns. */
export function jointJogAPI(fetcher: typeof fetch = globalThis.fetch.bind(globalThis)): JointJogAPI {
  const headers = (target: TargetIdentity) => ({
    "Content-Type": "application/json",
    ...(target ? { "X-RT-Target-Generation": target.selection_generation, "X-RT-Target-Cell": target.cell_id } : {}),
  });
  const request = async (action: string, intent?: Intent, target?: TargetIdentity, signal?: AbortSignal): Promise<JogState> => {
    const response = await fetcher(`${DENSE_EXECUTION_PATH}/jog/${action}`, {
      method: intent ? "POST" : "GET", credentials: "same-origin", redirect: "error", cache: "no-store", signal,
      headers: headers(target), body: intent ? JSON.stringify(intent) : undefined,
      keepalive: action === "end",
    });
    const body = await response.json();
    if (!response.ok) throw new JointJogError(body.error || "rt_core_failure", body.detail || "Joint jog refused");
    if (!Number.isSafeInteger(body.revision) || body.revision < 0 || typeof body.active !== "boolean") {
      throw new JointJogError("rt_core_protocol_error", "Invalid joint jog state");
    }
    return body as JogState;
  };
  return {
    state: target => request("state", undefined, target), send: (action, intent, target, signal) => request(action, intent, target, signal),
    move: async (intent, signal, target) => {
      const response = await fetcher(`${DENSE_EXECUTION_PATH}/move`, {
        method: "POST", credentials: "same-origin", redirect: "error", cache: "no-store", signal,
        headers: headers(target), body: JSON.stringify(intent),
      });
      const body = await response.json();
      if (!response.ok) throw new JointJogError(body.error || "rt_core_failure", body.detail || "Joint move refused");
      if (body.state !== "done" || body.axis !== intent.axis || !Number.isSafeInteger(body.revision) || body.revision <= intent.revision) {
        throw new JointJogError("rt_core_protocol_error", "Move completion was not observed");
      }
      return body as MoveState;
    },
    recover: async (body, signal, target) => {
      const response = await fetcher(`${DENSE_EXECUTION_PATH}${RECOVER_TO_ZERO_PATH}`, {
        method: "POST", credentials: "same-origin", redirect: "error", cache: "no-store", signal,
        headers: headers(target), body: JSON.stringify(body),
      });
      const payload = await response.json().catch(() => null);
      if (!response.ok) {
        const record = (payload ?? {}) as { error?: string; detail?: string };
        throw new JointJogError(record.error || "rt_core_failure", recoverFailureText(record.error ?? "", record.detail ?? "", payload ?? undefined));
      }
      return normalizeRecoverResult(payload);
    },
  };
}

/** A GO pins the displayed selection before awaiting state and owns one revision.
 * Cancellation prevents late receipts from publishing done or starting a move. */
export class JointAngleMove {
  readonly #api: JointJogAPI;
  readonly #changed: () => void;
  #run: AbortController | null = null;
  state: MoveState = { axis: -1, state: "", reason: "", revision: 0 };
  constructor(api: JointJogAPI, changed: () => void) { this.#api = api; this.#changed = changed; }
  get busy(): boolean { return this.#run !== null; }
  async begin(target: string, session: string, axis: number, degrees: number, fraction: number, identity?: TargetIdentity, described?: DescribedJoint): Promise<void> {
    return this.#start(target, session, [{ axis, degrees }], false, fraction, identity, described ? [described] : undefined);
  }
  /** One move of several joints at once: the server plans them as one
   * trajectory that starts and finishes together. */
  async beginMany(target: string, session: string, moves: MoveAxis[], fraction: number, identity?: TargetIdentity, described?: readonly (DescribedJoint | undefined)[]): Promise<void> {
    return this.#start(target, session, moves, true, fraction, identity, described);
  }
  async #start(target: string, session: string, moves: MoveAxis[], listed: boolean, fraction: number, identity?: TargetIdentity, described?: readonly (DescribedJoint | undefined)[]): Promise<void> {
    if (this.busy) return;
    const pinned = identity ? { ...identity } : undefined;
    const run = new AbortController(); this.#run = run;
    const first = moves[0] ?? { axis: -1, degrees: NaN };
    const axes = listed ? moves.map(m => m.axis) : undefined;
    this.state = { axis: first.axis, state: "preparing", reason: "", revision: 0, ...(axes ? { axes } : {}) }; this.#changed();
    try {
      if (moves.length === 0 || moves.some(m => m.position_rad !== undefined ? !Number.isFinite(m.position_rad) || m.degrees !== 0 : !Number.isFinite(m.degrees) || m.degrees === 0)) throw new JointJogError("joint_move_invalid", "Enter a finite absolute position or a nonzero signed angle");
      for (const [i, move] of moves.entries()) {
        const joint = described?.[i];
        const degrees = move.position_rad === undefined ? move.degrees : (move.position_rad - (joint?.origin_rad ?? NaN)) * 180 / Math.PI;
        const refusal = jointDescribedRangeRefusal(joint, degrees);
        if (refusal) throw refusal;
      }
      const state = await this.#api.state(pinned);
      if (this.#run !== run) return;
      if (state.active || state.move?.state === "preparing" || state.move?.state === "moving") throw new JointJogError("jog_mode_conflict", "Another target motion is active");
      if (!this.#api.move) throw new JointJogError("capability_unimplemented", "Move by angle is unavailable");
      this.state = { ...this.state, state: "moving", revision: state.revision + 1 }; this.#changed();
      const intent: MoveIntent = { target_id: target, session_id: session, revision: state.revision, axis: first.axis, degrees: first.degrees, fraction, ...(listed ? { axes: moves } : {}) };
      const result = await this.#api.move(intent, run.signal, pinned);
      if (this.#run === run) this.state = { ...result, ...(axes && !result.axes ? { axes } : {}) };
    } catch (error) {
      if (this.#run === run) {
        this.state = error instanceof JointJogError
          ? { ...this.state, state: "refused", reason: error.code, detail: error.message }
          : { ...this.state, state: "refused", reason: JOINT_JOG_CLIENT_ERROR, detail: jointJogFailureText(error) };
      }
    } finally { if (this.#run === run) this.#run = null; this.#changed(); }
  }
  cancel(cause = "operator_stop"): void {
    if (!this.#run) return;
    const run = this.#run; this.#run = null;
    this.state = { ...this.state, state: "stopped", reason: cause };
    run.abort(cause); this.#changed();
  }
}

/** One hold's requests: `inFlight` counts its unanswered press and intents;
 *  `starting` stays true until the press is answered. */
interface Hold { intent: Intent | null; target: TargetIdentity; timer: ReturnType<typeof setInterval> | null; inFlight: number; limit: number; starting: boolean; intents: AbortController }
interface JogTimers {
  every(fn: () => void, ms: number): ReturnType<typeof setInterval>;
  clear(id: ReturnType<typeof setInterval>): void;
}

/** The browser's interval timers, called as free functions. The previous
 * default, `{ every: setInterval, clear: clearInterval }`, invoked
 * `window.setInterval` as a method of a plain object, so its `this` was that
 * object and Chrome refused the call with `TypeError: Illegal invocation` on
 * the first hold; Node's timers ignore `this`, so no unit test saw it. */
export function browserJogTimers(): JogTimers {
  return { every: (fn, ms) => setInterval(fn, ms), clear: id => clearInterval(id) };
}

// The cadence keeps running while a press or an intent waits on a slow reply:
// on a Rosie 1430 arm (2026-10-07 15:33) it kept one request in flight and skipped ticks, so one
// slow round trip left 135/221/>=250 ms gaps between intents at OLP and the
// 250 ms deadman stopped a 400 mm/s hold. At most one input lifetime of
// cadence (250 ms / 50 ms, that arm's Describe max_jog_input_age_ns) is in flight
// when the server reports pipelined_intents:
// intents reach OLP every cadence up to a ~250 ms round trip, and at most
// round trip - 200 ms apart beyond it (200 ms at 400 ms). The bound leaves one
// of the browser's six HTTP/1.1 connections per origin for status, and release
// aborts the hold's in-flight intents, so End never queues behind them.
export const JOINT_JOG_MAX_IN_FLIGHT = 5;

/** One pointer hold pins its displayed selection and owns one fenced intent.
 * Release wins over pending replies; no late response can restart its cadence.
 * Intents are sent only while the hold is held: release clears the cadence
 * and sends End at once, whatever is still in flight. */
export class JointJogHold {
  readonly #api: JointJogAPI;
  readonly #changed: () => void;
  readonly #timers: JogTimers;
  #hold: Hold | null = null;
  #pending = 0;
  #starting = 0;
  #ending: Promise<void> | null = null;
  #axis = -1;
  reason = "";
  constructor(api: JointJogAPI, changed: () => void, timers: JogTimers = browserJogTimers()) {
    this.#api = api; this.#changed = changed; this.#timers = timers;
  }
  get active(): boolean { return this.#hold !== null; }
  get busy(): boolean { return this.active || this.#pending > 0; }
  /** A new press is refused only while a hold is held or a press is still
   *  unanswered. A release still in flight does not refuse it: the server
   *  holds the press until that stop completes (1de38926). */
  get pressBusy(): boolean { return this.active || this.#starting > 0; }
  /** The axis of the current or last hold; -1 before the first begin. */
  get axis(): number { return this.#axis; }

  async begin(target: string, session: string, axis: number, direction: number, fraction: number, identity?: TargetIdentity): Promise<void> {
    if (this.pressBusy) { this.reason = rtCoreReasonText("jog_mode_conflict"); this.#changed(); return; }
    this.#axis = axis;
    const hold: Hold = { intent: null, target: identity ? { ...identity } : undefined, timer: null, inFlight: 1, limit: 1, starting: true, intents: new AbortController() };
    this.#hold = hold; this.reason = ""; this.#pending++; this.#starting++; this.#changed();
    try {
      let state = await this.#api.state(hold.target);
      if (this.#hold !== hold) return;
      const ending = this.#ending;
      if (state.active && ending) {
        // The previous release may not have reached the server yet.
        await ending;
        state = await this.#api.state(hold.target);
        if (this.#hold !== hold) return;
      }
      if (state.active) throw new JointJogError("jog_mode_conflict", "Another joint is held");
      // A server that cannot order intents arriving together (before
      // OLP's pipelined_intents) refuses the older one jog_invalid_timing: one
      // in flight there, as before.
      hold.limit = state.pipelined_intents === true ? JOINT_JOG_MAX_IN_FLIGHT : 1;
      hold.intent = { target_id: target, session_id: session, revision: state.revision, axis, direction, fraction };
      hold.timer = this.#timers.every(() => { void this.#tick(hold); }, JOINT_JOG_CADENCE_MS);
      await this.#api.send("begin", hold.intent, hold.target);
    } catch (error) {
      if (this.#hold === hold) { this.#failure(error); void this.end(refusalEndCause(error)); }
    } finally {
      hold.inFlight--; hold.starting = false; this.#pending--; this.#starting--; this.#changed();
    }
  }

  /** A path hold accepts a new speed within the hold (joystick, speed
   *  select); the next cadence intent carries it. Joint jogs and cells
   *  without fraction_updates end the hold instead. */
  setFraction(fraction: number): boolean {
    const hold = this.#hold;
    if (!hold?.intent || !(fraction > 0 && fraction <= 1)) return false;
    hold.intent = { ...hold.intent, fraction };
    return true;
  }

  async #tick(hold: Hold): Promise<void> {
    if (this.#hold !== hold || !hold.intent || hold.inFlight >= hold.limit) return;
    const early = hold.starting;
    hold.inFlight++;
    try { await this.#api.send("update", hold.intent, hold.target, hold.intents.signal); }
    catch (error) {
      // Before the press is answered the server may not hold this revision
      // yet (an ordinary Begin is still opening the lane): that intent's stale
      // refusal is not the hold's. The press's own reply decides.
      if (early && error instanceof JointJogError && error.code === "jog_session_stale") return;
      if (this.#hold === hold) { this.#failure(error); void this.end(refusalEndCause(error)); }
    }
    finally { hold.inFlight--; }
  }

  /** `cause` says why (JOG_END_CAUSES); an empty cause is sent as none, which
   *  the server reads as a release, as from an older client. */
  async end(cause: JogEndCause | "" = ""): Promise<void> {
    const hold = this.#hold;
    if (!hold) return;
    this.#hold = null;
    if (hold.timer !== null) this.#timers.clear(hold.timer);
    hold.intents.abort();
    this.#pending++; this.#changed();
    const ending = (async () => {
      try { if (hold.intent) await this.#api.send("end", cause ? { ...hold.intent, end_cause: cause } : hold.intent, hold.target); }
      catch (error) {
        // A refusal/deadman may already have retired this revision; preserve its
        // original operator reason while still showing any transport failure.
        if (!(error instanceof JointJogError && error.code === "jog_session_stale" && this.reason)) this.#failure(error);
      } finally { this.#pending--; this.#changed(); }
    })();
    this.#ending = ending;
    try { await ending; } finally { if (this.#ending === ending) this.#ending = null; }
  }

  observeReason(reason: string, revision?: number): void {
    if (!reason) return;
    if (revision !== undefined && this.#hold && (!this.#hold.intent || revision <= this.#hold.intent.revision)) return;
    this.reason = rtCoreReasonText(reason);
    void this.end(`server_reason:${reason}`); this.#changed();
  }
  /** A request threw while this hold was live: end it and show why, in the
   * server's words for a typed refusal and as a client error for anything else. */
  observeFailure(error: unknown): void {
    this.reason = jointJogFailureText(error);
    void this.end("state_poll_failed"); this.#changed();
  }
  #failure(error: unknown): void {
    this.reason = jointJogFailureText(error);
  }
}

/** The words for a move row: a browser-originated stop in its own words, the
 *  server's reason by code (joint_move_limits carries the described range when
 *  the detail has it), a browser failure by its own text, else the state. */
function moveReasonText(move: MoveState): string {
  if (!move.reason) return move.state;
  if (move.reason === JOINT_JOG_CLIENT_ERROR) return move.detail ?? "client error: unknown";
  const browser = jointJogBrowserStopText(move.reason);
  if (browser) return browser;
  return rtCoreReasonText(move.reason, move.reason === "joint_move_limits" ? move.detail : "");
}

/** Prefer this pane's latest attempt over a polled older move, so a
 *  joint_move_limits on J7 cannot leave J1's operator_stop on screen. */
function selectJogMove(local: MoveState, remote: MoveState, busy: boolean): MoveState {
  if (busy || !remote.state) return local;
  if (!local.state) return remote;
  if (local.revision >= remote.revision) return local;
  if (["refused", "stopped"].includes(local.state) && local.axis !== remote.axis && !(remote.axes?.includes(local.axis) ?? false)) return local;
  return remote;
}

/** Why the joint controls are off, in the order the machine path resolves
 * it: no machine, no description, a refusal, no grant, disarmed, not ready,
 * busy. The pill's derived arm fact prevents retained armed status from
 * admitting a new hold or move after observations go stale. */
export function jointJogDisabled(selected: boolean, caps: DenseCapabilities | null, status: DenseStatus | null, busy: boolean, arm: Pick<ArmFact, "state" | "controlled" | "jogNote"> | null, model = status?.target?.model_id): string {
  const modelReason = unknownRobotModelReason(model, status?.target?.model_id);
  if (modelReason) return modelReason;
  if (!selected) return "No machine connected: choose and connect one under Cells to jog its joints";
  if (caps?.backend !== "rt_core" || !caps.rtCore || !caps.daemonReachable || !caps.cell) return "Describe the selected rt-core target before jogging";
  if (status?.daemonError) return rtCoreReasonText(status.daemonError);
  if (!status?.session?.leaseHeld) return "No grant; ARM the machine (step 4) before jogging";
  if (status.armed !== true) return "Jog not ready; ARM the machine (step 4) before jogging";
  if (arm?.state !== "armed" || !arm.controlled) return arm?.jogNote || "Arm state UNKNOWN: no fresh status from the machine.";
  if (status.rtCore?.ready !== true) return "Jog not ready; wait for valid target feedback";
  // Between a loop's cycles the session is not "playing" for moments; the
  // server refuses a jog there too (jog_mode_conflict).
  if (status.loop?.state === "running" || status.loop?.state === "finishing") return "A program loop is running; STOP it (or Finish cycle) before jogging";
  if (busy || status.session.state === "playing") return "Another target operation is active";
  return "";
}

/** Bind all release paths to the same idempotent End; pointer capture alone
 * cannot keep motion alive after leaving a button or losing window focus. */
export function wireJointJogButton(button: HTMLButtonElement, begin: () => void, end: (cause: JogEndCause) => void): void {
  button.addEventListener("pointerdown", event => {
    if (button.disabled || event.button !== 0 || event.isPrimary === false) return;
    event.preventDefault(); begin();
  });
  for (const event of ["pointerup", "pointerleave", "pointercancel", "lostpointercapture"]) button.addEventListener(event, () => end(pointerEndCause(event)));
}

/** Title on the Preview column: the live Position cell is still the machine. */
export const PREVIEW_COLUMN_TITLE =
  "Joint angle of the previewed plan at the playback cursor; the Position column stays the machine's";

/** Joints of a preview sample, in the model's jointOrder. Kept structural so
 *  this file does not import the simulation contracts. */
export interface PreviewJoints {
  jointOrder: readonly string[];
  jointsRad: readonly number[];
}

/** Render the described target axes without replacing held DOM nodes during
 * status polling. POS uses public logical units, preventing simulator feedback. */
/** What the Joints section can ask its host for beyond the jog lane: the
 *  native Home transaction (the Deploy pane's HOME, gated the same way). */
export interface JointJogActions {
  home?: () => void;
  /** The native Home on one axis, under the same gate as `home`. */
  homeAxis?: (index: number) => void;
  /** The reason Home is unavailable now, or null when it may run. */
  homeDisabled?: () => string | null;
  /** Ask the operator before the recovery arms: resolves true to arm and
   *  move. Absent, a recovery that needs Arm stops at the question. */
  confirmRecover?: (confirmation: RecoverConfirmation) => Promise<boolean>;
}

/** Below this the joint is at zero for the zero-all sequence: two hundredths
 *  of a degree, under the table's own two-decimal display. */
export const JOINT_ZERO_TOLERANCE_DEG = 0.02;

/** The button's tooltip: what one press does when the robot is not ready. */
export const ZERO_BUTTON_TITLE =
  "Move every joint to 0° at the selected speed, all together in one move. Disarmed or stuck, it first confirms an unconfirmed Stop and resets faults whose recovery is a reset, then asks before it arms";

export class JointJogPanel {
  readonly #host: HTMLElement;
  readonly #hold: JointJogHold;
  readonly #api: JointJogAPI;
  readonly #move: JointAngleMove;
  #remoteMove: MoveState = { axis: -1, state: "", reason: "", revision: 0 };
  #caps: DenseCapabilities | null = null;
  #status: DenseStatus | null = null;
  #selected = false;
  #disabled = "";
  #key = "";
  #polling = false;
  #fraction = 0.1;
  #buttons: HTMLButtonElement[] = [];
  #positions: HTMLElement[] = [];
  #previews: HTMLElement[] = [];
  /** The note beside the Speed select: what the fraction is a fraction of. */
  #speedNote: HTMLElement | null = null;
  /** The previewed plan at the playback cursor, or null when stopped. */
  #previewSample: PreviewJoints | null = null;
  #goButtons: HTMLButtonElement[] = [];
  #angles: HTMLInputElement[] = [];
  #rowStates: HTMLElement[] = [];

  #loggedRefusal = "";
  readonly #refusal: (text: string) => void;
  readonly #actions: JointJogActions;
  /** The zero-all sequence in progress: the axis being moved, or -1. */
  #zeroing: number[] = [];
  #homeButton: HTMLButtonElement | null = null;
  #homeAxisButtons: HTMLButtonElement[] = [];
  #homeStates: HTMLElement[] = [];
  #zeroButton: HTMLButtonElement | null = null;
  /** Why All joints to 0° (the recovery) cannot start: recoverToZeroDisabled. */
  #recoverDisabled = "";
  /** The recovery request in flight; STOP and leaving the page abort it. */
  #recoverRun: AbortController | null = null;
  /** A recovery press is running, its confirmation included. Its own Reset
   *  and Arm change the session and the jog gate under it; only STOP, a
   *  selection change or leaving the page end it. */
  #recovering = false;
  /** #gateNote holds the last recovery's outcome (cleared with the note). */
  #recoveryNote = false;
  /** Latest operator-facing gate from a silent zeroAll return. */
  #gateNote = "";
  /** A stop this browser sent: poll must not replace it with operator_stop. */
  #browserStop = false;
  /** Per-row text for a hold this browser ended; cleared on the next attempt. */
  #rowReasons: string[] = [];
  constructor(host: HTMLElement, api: JointJogAPI = jointJogAPI(), refusal: (text: string) => void = () => {}, actions: JointJogActions = {}) {
    this.#refusal = refusal;
    this.#actions = actions;
    this.#host = host; this.#api = api;
    this.#hold = new JointJogHold(api, () => this.#paint());
    this.#move = new JointAngleMove(api, () => this.#paint());
    for (const event of ["blur", "pointerup", "pointercancel"] as const) window.addEventListener(event, () => this.#release(event));
    window.addEventListener("pagehide", () => this.#release("pagehide"));
  }
  end(cause: JogEndCause | "" = ""): void { void this.#hold.end(cause); }
  stop(cause: JogEndCause = "operator_stop"): void {
    this.end(cause); this.#move.cancel(); this.#recovering = false; this.#recoverRun?.abort(cause); this.#zeroing = []; this.#paint();
  }
  /** End a hold on pointer release, and a running step when the page loses
   *  focus. pointerup does not cancel a GO: that event is the click that
   *  started it. */
  #release(event: "blur" | "pointerup" | "pointercancel" | "pagehide" | "pointer_release", endCause?: JogEndCause): void {
    const cause = event === "blur" || event === "pagehide" ? "page_blur" : "pointer_release";
    const stopText = jointJogBrowserStopText(cause);
    if (this.#hold.active && stopText) {
      this.#browserStop = true;
      this.#hold.reason = stopText;
      if (this.#hold.axis >= 0) this.#rowReasons[this.#hold.axis] = stopText;
    }
    void this.#hold.end(endCause ?? windowEndCause(event));
    if ((event === "blur" || event === "pagehide") && this.#move.busy) {
      this.#browserStop = true;
      this.#gateNote = "";
      this.#move.cancel(cause);
    }
    if ((event === "blur" || event === "pagehide") && this.#recoverRun) {
      this.#browserStop = true;
      this.#recovering = false;
      this.#recoverRun.abort(cause);
    }
    if (event === "pagehide") this.#zeroing = [];
    this.#paint();
  }
  #describedJoint(index: number): DescribedJoint | undefined {
    const axis = this.#caps?.rtCore?.axes[index];
    if (!axis) return undefined;
    return {
      id: axis.id,
      position_unit: axis.position_unit,
      min_position: axis.min_position,
      max_position: axis.max_position,
      origin_rad: this.#status?.rtCore?.axes[index]?.position ?? null,
    };
  }
  #displayedMove(): MoveState {
    return selectJogMove(this.#move.state, this.#remoteMove, this.#move.busy);
  }
  #clearAttempt(): void {
    this.#browserStop = false;
    this.#hold.reason = "";
    this.#gateNote = "";
    this.#recoveryNote = false;
    this.#rowReasons = [];
  }

  /** All joints to 0°, which is also the one-press recovery
   *  (`recoverToZero.ts`): the server confirms an unconfirmed Stop, resets
   *  faults whose recovery is a reset, and -- only after the operator
   *  confirms the named action with every joint's travel -- arms; then every
   *  joint off zero moves to 0° in one move, one trajectory that starts and
   *  finishes together at the selected speed. It stops at the first step
   *  that does not clear, and the reason line says why. STOP, or leaving
   *  the page, aborts the request and the server stops the move. (One joint
   *  at a time was fragile: on the LA bench the sequence stopped after the
   *  first joint when a transient between moves cancelled the next one.) */
  async zeroAll(fraction = this.#fraction): Promise<void> {
    const blocked = this.#blockRecover();
    if (blocked) { this.#gateNote = blocked; this.#paint(); return; }
    if (!this.#api.recover) { this.#gateNote = "Recover to zero is unavailable"; this.#paint(); return; }
    this.#clearAttempt();
    const target = this.#status?.target ? { ...this.#status.target } : undefined;
    const rotary = (this.#caps!.rtCore?.axes ?? []).flatMap((axis, index) => (axis?.position_unit === "rad" ? [index] : []));
    this.#recovering = true;
    const press = async (body: Record<string, unknown>): Promise<RecoverResult> => {
      const run = new AbortController(); this.#recoverRun = run;
      this.#zeroing = rotary; this.#paint();
      try { return await this.#api.recover!(body, run.signal, target); } finally {
        if (this.#recoverRun === run) this.#recoverRun = null;
        this.#zeroing = []; this.#paint();
      }
    };
    try {
      let result = await press(recoverRequest(fraction));
      if (result.state === "confirm_arm") {
        const confirmed = this.#actions.confirmRecover ? await this.#actions.confirmRecover(recoverConfirmation(result)) : false;
        if (!this.#recovering) { this.#gateNote = "All joints to 0° stopped before it armed"; return; }
        if (!confirmed) { this.#gateNote = "All joints to 0° cancelled: nothing was armed or moved"; return; }
        result = await press(recoverRequest(fraction, result));
      }
      this.#gateNote = recoverOutcomeText(result);
    } catch (error) {
      this.#gateNote = this.#browserStop ? jointJogBrowserStopText("page_blur") || "Stopped: the page lost focus"
        : error instanceof JointJogError ? error.message
        : error instanceof DOMException && error.name === "AbortError" ? "All joints to 0° stopped"
        : jointJogFailureText(error);
      this.#refusal(`All joints to 0°: ${this.#gateNote}`);
    } finally {
      this.#recovering = false; this.#recoveryNote = true; this.#zeroing = []; this.#paint();
    }
  }
  /** The joints of the zero-all move in progress; empty when none. */
  get zeroing(): readonly number[] { return this.#zeroing; }
  /** Uses native held positions to reach exact plan endpoints, even for tiny offsets. */
  async goToPositions(positions: readonly number[]): Promise<void> {
    const blocked = this.#blockZeroAll();
    if (blocked) throw new Error(blocked);
    const axes = this.#caps!.rtCore!.axes;
    if (positions.length !== axes.length || positions.some((q, i) => !Number.isFinite(q) || axes[i]?.position_unit !== "rad")) throw new Error("Plan start does not match the machine's rotary axes");
    this.#clearAttempt();
    const moves = positions.map((position_rad, axis) => ({ axis, degrees: 0, position_rad }));
    await this.#move.beginMany(this.#caps!.cell, this.#status!.session!.id, moves, this.#fraction, this.#status!.target, moves.map(m => this.#describedJoint(m.axis)));
    if (this.#move.state.state !== "done") throw new Error(moveReasonText(this.#move.state));
  }
  #blockZeroAll(): string | null {
    if (this.#zeroing.length > 0) return "All joints to 0° is already running";
    if (!this.#selected || !this.#caps) return "Connect a machine first";
    if (this.#disabled) return this.#disabled;
    if (this.#hold.active || this.#move.busy || ["preparing", "moving"].includes(this.#displayedMove().state)) return "Another target motion is active";
    if (!this.#status?.session) return "No grant; ARM the machine (step 4) before jogging";
    return null;
  }
  /** Why the recovery cannot start now: the gate for a robot that is not
   *  armed or not answering cleanly, which is what it recovers. */
  #blockRecover(): string | null {
    if (this.#zeroing.length > 0) return "All joints to 0° is already running";
    if (!this.#selected || !this.#caps) return "Connect a machine first";
    if (this.#recoverDisabled) return this.#recoverDisabled;
    if (this.#hold.active || this.#move.busy || ["preparing", "moving"].includes(this.#displayedMove().state)) return "Another target motion is active";
    return null;
  }
  /** Why the zero-all cannot start now, or null; the Cartesian view shares the button. */
  zeroGate(): string | null {
    if (this.#zeroing.length > 0) return null;
    return this.#blockRecover();
  }
  zeroLabel(): string {
    if (this.#zeroing.length === 0) return "All joints to 0°";
    if (this.#zeroing.length === 1) return `Zeroing ${this.#caps?.rtCore?.axes[this.#zeroing[0]!]?.id ?? "one joint"}…`;
    return `Zeroing ${this.#zeroing.length} joints…`;
  }
  get active(): boolean { return this.#hold.active; }
  /** A GO, zero-all or plan-start move of this panel is in flight. */
  get moveBusy(): boolean { return this.#move.busy; }
  /** The Speed % this table shows: Go to plan start and Run program move at it. */
  get speedFraction(): number { return this.#fraction; }
  update(selected: boolean, caps: DenseCapabilities | null, status: DenseStatus | null, busy: boolean, arm: ArmFact, model: string, statusAgeMs: number | null = null, pendingHomeAxes: readonly number[] | null = null): void {
    if (caps?.cell !== this.#caps?.cell || status?.session?.id !== this.#status?.session?.id ||
        status?.target?.cell_id !== this.#status?.target?.cell_id || status?.target?.selection_generation !== this.#status?.target?.selection_generation || !selected) {
      const sessionOnly = selected && status?.session?.id !== this.#status?.session?.id && caps?.cell === this.#caps?.cell &&
        status?.target?.selection_generation === this.#status?.target?.selection_generation;
      // The recovery's own Reset and Arm replace the session under it.
      if (sessionOnly && this.#recovering) { this.end("session_changed"); this.#move.cancel("session_changed"); }
      else this.stop(sessionOnly ? "session_changed" : "selection_changed");
      this.#remoteMove = { axis: -1, state: "", reason: "", revision: 0 };
    }
    this.#caps = caps; this.#status = status; this.#selected = selected;
    this.#disabled = jointJogDisabled(selected, caps, status, busy, arm, model);
    this.#recoverDisabled = recoverToZeroDisabled(selected, caps, status, busy, model);
    if (this.#disabled && this.#recovering) { this.end("jog_disabled"); this.#move.cancel("jog_disabled"); }
    else if (this.#disabled) this.stop("jog_disabled");
    const axes = caps?.rtCore?.axes ?? [];
    const key = JSON.stringify([caps?.cell, axes]);
    if (key !== this.#key) {
      this.end("ui_navigation"); this.#key = key; this.#rowReasons = []; this.#host.replaceChildren();
      const caption = document.createElement("p"); caption.className = "field-help joint-jog-caption"; caption.dataset.jogCaption = "";
      const label = document.createElement("label"); label.className = "joint-jog-speed"; label.textContent = "Speed ";
      const select = document.createElement("select"); select.setAttribute("aria-label", "Joint jog velocity fraction");
      for (const fraction of JOINT_JOG_FRACTIONS) { const o = document.createElement("option"); o.value = String(fraction); o.textContent = `${fraction*100}%`; select.append(o); }
      select.value = String(this.#fraction);
      select.addEventListener("change", () => { this.end("speed_change"); this.#fraction = Number(select.value); this.#paint(); }); label.append(select);
      const speedNote = document.createElement("span"); speedNote.className = "joint-jog-speed-note"; speedNote.dataset.jogSpeedNote = "";
      speedNote.title = `Moves run at ${JOINT_JOG_VELOCITY_RESERVE} of this, the measured drive reserve; rt-core enforces each joint's maximum regardless`;
      label.append(speedNote); this.#speedNote = speedNote;
      // The toolbar: speed at the left, the two whole-arm actions at the right.
      const toolbar = document.createElement("div"); toolbar.className = "joint-jog-toolbar";
      toolbar.append(label);
      const actions = document.createElement("div"); actions.className = "joint-jog-actions";
      if (this.#actions.home) {
        const home = document.createElement("button"); home.type = "button"; home.className = "mini-button joint-jog-home"; home.textContent = "Home joints";
        home.addEventListener("click", () => { if (!home.disabled) this.#actions.home?.(); });
        actions.append(home); this.#homeButton = home;
      }
      const zero = document.createElement("button"); zero.type = "button"; zero.className = "mini-button joint-jog-zero"; zero.textContent = "All joints to 0°";
      zero.addEventListener("click", () => void this.zeroAll());
      actions.append(zero); this.#zeroButton = zero;
      toolbar.append(actions);
      const table = document.createElement("table"); table.className = "dense-motor-table joint-jog-table";
      table.innerHTML = `<thead><tr><th>Axis</th><th>Position</th><th title="${PREVIEW_COLUMN_TITLE}">Preview</th><th>Hold to jog</th><th>Move by angle</th><th>Home</th></tr></thead>`;
      const body = document.createElement("tbody"); this.#buttons = []; this.#positions = []; this.#previews = []; this.#homeAxisButtons = []; this.#homeStates = [];
      this.#goButtons = []; this.#angles = []; this.#rowStates = [];
      axes.forEach((axis, index) => {
        const row = document.createElement("tr"), name = document.createElement("th"), position = document.createElement("td"), preview = document.createElement("td"), controls = document.createElement("td");
        name.textContent = axis.id; position.dataset.jogPosition = ""; preview.dataset.jogPreview = "";
        this.#positions.push(position); this.#previews.push(preview);
        for (const direction of [-1, 1]) {
          const button = document.createElement("button"); button.type = "button"; button.className = "mini-button joint-jog-hold";
          button.textContent = direction < 0 ? "−" : "+";
          button.setAttribute("aria-label", `Jog ${caps?.cell ?? "selected robot"} ${axis.id} ${direction < 0 ? "negative" : "positive"}`);
          wireJointJogButton(button, () => {
            if (!button.disabled && this.#selected && this.#status?.session && this.#caps) {
              this.#clearAttempt();
              void this.#hold.begin(this.#caps.cell, this.#status.session.id, index, direction, this.#fraction, this.#status.target);
            }
          }, cause => this.#release("pointer_release", cause));
          this.#buttons.push(button); controls.append(button);
        }
        const move = document.createElement("td"), angle = document.createElement("input"), go = document.createElement("button"), result = document.createElement("span");
        angle.type = "number"; angle.step = "any"; angle.placeholder = "± deg"; angle.className = "joint-move-angle";
        angle.setAttribute("aria-label", `Move ${axis.id} by signed degrees`);
        go.type = "button"; go.className = "mini-button joint-move-go"; go.textContent = "GO";
        go.setAttribute("aria-label", `Move ${caps?.cell ?? "selected robot"} ${axis.id} by angle`);
        go.addEventListener("click", () => {
          if (!go.disabled && this.#selected && this.#caps && this.#status?.session) {
            this.#clearAttempt();
            void this.#move.begin(this.#caps.cell, this.#status.session.id, index, Number(angle.value), this.#fraction, this.#status.target, this.#describedJoint(index));
          }
        });
        result.className = "joint-move-state"; result.setAttribute("aria-live", "polite");
        this.#goButtons.push(go); this.#angles.push(angle); this.#rowStates.push(result);
        // One line per joint: the field, GO and the move's last word side by
        // side in a wrapper (a <td> stays a table cell), the word in a slot of
        // fixed width so a message never resizes the row under a held button.
        const moveControls = document.createElement("div"); moveControls.className = "joint-move-controls";
        moveControls.append(angle, go, result); move.className = "joint-move-cell"; move.append(moveControls);
        controls.className = "joint-hold-cell";
        row.append(name, position, preview, controls, move);
        const homeCell = document.createElement("td"); homeCell.className = "joint-home-cell";
        const homeControls = document.createElement("div"); homeControls.className = "joint-home-controls";
        homeCell.append(homeControls);
        const homeState = document.createElement("div"); homeState.className = "field-help joint-home-status";
        homeState.dataset.jogHomeState = "unavailable";
        homeState.setAttribute("aria-live", "polite");
        homeControls.append(homeState); this.#homeStates.push(homeState);
        if (this.#actions.homeAxis) {
          const home = document.createElement("button"); home.type = "button"; home.className = "mini-button joint-home-axis"; home.textContent = "Home";
          home.setAttribute("aria-label", `Home ${axis.id}`);
          home.addEventListener("click", () => { if (!home.disabled) this.#actions.homeAxis?.(index); });
          homeControls.append(home); this.#homeAxisButtons.push(home);
        }
        row.append(homeCell);
        body.append(row);
      });
      table.append(body);
      // The table scrolls sideways on its own in a narrow column, so the
      // notes and buttons above it keep the column's width.
      const scroller = document.createElement("div"); scroller.className = "joint-jog-table-scroll"; scroller.append(table);
      const reason = document.createElement("p"); reason.className = "field-help joint-jog-reason"; reason.dataset.jogReason = ""; reason.setAttribute("aria-live", "polite");
      this.#host.append(caption, toolbar, scroller, reason);
    }
    this.#positions.forEach((position, i) => {
      const value = status?.rtCore?.axes[i]?.position;
      const rotary = axes[i]?.position_unit === "rad";
      position.textContent = value == null ? "—" : rotary ? `${(value * 180 / Math.PI).toFixed(2)}°` : `${value.toFixed(4)} ${axes[i]?.position_unit ?? ""}`;
    });
    const home = deriveHomeStatus({ status, capabilities: caps, connected: selected, statusAgeMs, pendingAxes: pendingHomeAxes });
    this.#homeStates.forEach((element, i) => {
      const fact = home.axes.find(axis => axis.index === (axes[i]?.index ?? i)) ?? home;
      // The row's word, short enough to share the line with Home: the
      // verified state is the common one and reads as one word; the full
      // label and the machine's reason are the tooltip.
      element.textContent = fact.label === "Calibrated · position verified" ? "Verified" : fact.label;
      element.title = fact.detail ? `${fact.label}: ${fact.detail}` : fact.label;
      element.dataset.jogHomeState = fact.state;
    });
    this.#paintPreview();
    this.#paint();
  }
  /** The previewed plan's joints at the playback cursor, or null when the
   *  preview is stopped. Paints the Preview column; hold, GO and zero-all
   *  keep using the machine. */
  previewSample(sample: PreviewJoints | null): void {
    this.#previewSample = sample;
    this.#paintPreview();
  }
  #paintPreview(): void {
    const axes = this.#caps?.rtCore?.axes ?? [];
    this.#previews.forEach((cell, i) => {
      const rad = previewJointRadForAxis(this.#previewSample, axes[i]?.id, i);
      cell.textContent = rad === null ? "—" : `${(rad * 180 / Math.PI).toFixed(2)}°`;
    });
  }
  async poll(): Promise<void> {
    if (this.#polling || !this.#selected || this.#caps?.backend !== "rt_core") return;
    this.#polling = true;
    const session = this.#status?.session?.id;
    const target = this.#status?.target;
    try {
      const state = await this.#api.state(target);
      if (session === this.#status?.session?.id && target?.cell_id === this.#status?.target?.cell_id && target?.selection_generation === this.#status?.target?.selection_generation) {
        if (state.reason && !this.#browserStop) this.#hold.observeReason(state.reason, state.revision);
        if (state.move && state.move.revision >= this.#remoteMove.revision) {
          const local = this.#move.state;
          const staleInFlight = this.#browserStop && ["preparing", "moving"].includes(state.move.state) && state.move.revision <= local.revision;
          if (!staleInFlight) this.#remoteMove = state.move;
        }
        this.#paint();
      }
    } catch (error) {
      if (this.#hold.active) this.#hold.observeFailure(error);
    } finally { this.#polling = false; }
  }
  #paint(): void {
    const move = this.#displayedMove();
    const moving = this.#move.busy || ["preparing", "moving"].includes(move.state);
    const caption = this.#host.querySelector<HTMLElement>("[data-jog-caption]");
    const robot = this.#status?.robotCell;
    const label = robot?.configured && robot.valid ? robot.modelId : this.#caps?.cell;
    if (caption) caption.textContent = `Jogs the selected robot: ${this.#selected ? label || "unavailable" : "none connected"}`;
    for (const button of this.#buttons) { button.disabled = !!this.#disabled || moving; button.title = this.#disabled || (moving ? "A joint move is active; STOP or wait for done" : "Hold to jog this joint; release to stop"); }
    // Choosing a speed is not a motion: it stays available while disarmed or
    // disconnected and is locked only while a hold or move is running.
    const select = this.#host.querySelector("select"); if (select) select.disabled = this.#hold.busy || moving;
    if (this.#speedNote) this.#speedNote.textContent = jointJogSpeedNote(this.#fraction, this.#caps?.rtCore?.axes ?? []);
    this.#goButtons.forEach((button, index) => {
      const units = this.#caps?.rtCore?.axes[index]?.position_unit !== "rad";
      button.disabled = !!this.#disabled || this.#hold.busy || moving || units;
      button.title = this.#disabled || (units ? "Move by angle requires a rotary axis described in radians" : this.#hold.busy || moving ? "Another target motion is active" : "Move this joint by the signed angle; STOP cancels");
      this.#angles[index]!.disabled = button.disabled;
      const mine = move.axis === index || (move.axes?.includes(index) ?? false);
      this.#rowStates[index]!.textContent = mine ? moveReasonText(move) : this.#rowReasons[index] ?? "";
      this.#rowStates[index]!.title = this.#rowStates[index]!.textContent ?? "";
      markBusy(this.#rowStates[index]!, mine && (move.state === "preparing" || move.state === "moving"));
    });
    if (this.#homeButton) {
      const why = this.#actions.homeDisabled?.() ?? null;
      this.#homeButton.disabled = why !== null || this.#zeroing.length > 0 || moving;
      this.#homeButton.title = why ?? (moving ? "A joint move is active; wait for done" : "Run the machine's Home on every joint (the drives must be off)");
    }
    for (const [index, button] of this.#homeAxisButtons.entries()) {
      const why = this.#actions.homeDisabled?.() ?? null;
      button.disabled = why !== null || this.#zeroing.length > 0 || moving;
      button.title = why ?? (moving ? "A joint move is active; wait for done" : `Run the machine's Home on ${this.#caps?.rtCore?.axes[index]?.id ?? "this joint"} alone (the drives must be off)`);
    }
    if (this.#zeroButton) {
      const zeroing = this.#zeroing.length > 0;
      this.#zeroButton.disabled = !!this.#recoverDisabled || this.#hold.active || (moving && !zeroing);
      this.#zeroButton.textContent = this.zeroLabel();
      markBusy(this.#zeroButton, zeroing);
      this.#zeroButton.title = this.#recoverDisabled || (zeroing ? "Recovering and moving every off-zero joint to 0° together; STOP ends it" : ZERO_BUTTON_TITLE);
    }
    const reason = this.#host.querySelector<HTMLElement>("[data-jog-reason]");
    const moveText = move.state === "refused" || move.state === "stopped" ? moveReasonText(move) : "";
    if (reason) {
      // A recovery's outcome or refusal is the answer to the press: it comes
      // before the jog gate, which while disarmed always reads "No grant".
      reason.textContent = this.#recoveryNote ? [this.#gateNote, this.#disabled].filter(Boolean).join(". ") : this.#disabled || this.#gateNote || moveText || this.#hold.reason;
      reason.title = reason.textContent;
    }
    const refusal = [this.#hold.reason, move.state === "refused" || move.state === "stopped" ? moveReasonText(move) : ""].filter(Boolean).join("; ");
    if (refusal && refusal !== this.#loggedRefusal) this.#refusal(refusal);
    this.#loggedRefusal = refusal;
  }
}
