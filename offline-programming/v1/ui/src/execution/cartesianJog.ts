import { DENSE_EXECUTION_PATH, type DenseCapabilities, type DenseStatus } from "./denseClient.ts";
import { JointAngleMove, JointJogError, JointJogHold, jointJogDisabled, type JointJogAPI } from "./jointJog.ts";
import { CARTESIAN_INPUT_INVALID, CARTESIAN_STEP_DEFAULTS, cartesianCancelText, cartesianDefaultMode, cartesianHoldTwist, cartesianMoveBody, readCartesianSteps, requireCartesianSteps, type CartesianCancelCause, type CartesianMode, type CartesianSteps } from "./cartesianMove.ts";
import { rtCoreReasonText } from "../targets/rtCoreStatus.ts";
import { cartesianMachineHeading, type MachineFlowView } from "./machineFlow.ts";
import { robotModelClass, unknownRobotModelReason } from "../targets/cells.ts";
import { windowEndCause, type JogEndCause } from "./jogEndCause.ts";

function cancelEndCause(cause: CartesianCancelCause): JogEndCause {
  return cause === "disarm" ? "operator_disarm" : cause === "stop" ? "operator_stop" : cause === "selection_changed" || cause === "session_changed" ? cause : "operator_stop";
}

// Unverified presentation policy: a bench never offers an arm motion control.
export const BENCH_CARTESIAN_UNAVAILABLE = "Not available on a bench without an arm model";

/** Route machine requests with the pinned selection. A refused request stays on
 * this route; it cannot fall back to preview or rotate TOOL input in the browser. */
export function cartesianJogAPI(frame: () => string, fetcher: typeof fetch = globalThis.fetch.bind(globalThis), steps: () => CartesianSteps = readCartesianSteps): JointJogAPI {
  const request: JointJogAPI["send"] = async (action, intent, target, signal) => {
    const twist = cartesianHoldTwist(intent.axis, intent.direction, steps(), intent.fraction);
    const route = { begin: "start", update: "intent", end: "halt" }[action];
    return send(route, { target_id: intent.target_id, session_id: intent.session_id, revision: intent.revision, frame: frame(), twist, fraction: intent.fraction,
      ...(intent.end_cause ? { end_cause: intent.end_cause } : {}) }, target, signal);
  };
  const send = async (action: string, body: unknown, target: DenseStatus["target"], signal?: AbortSignal) => {
    const response = await fetcher(`${DENSE_EXECUTION_PATH}/cartesian/${action}`, {
      method: body ? "POST" : "GET", credentials: "same-origin", redirect: "error", cache: "no-store", signal,
      headers: { "Content-Type": "application/json", ...(target ? { "X-RT-Target-Generation": target.selection_generation, "X-RT-Target-Cell": target.cell_id } : {}) },
      body: body ? JSON.stringify(body) : undefined, keepalive: action === "halt",
    });
    const result = await response.json();
    if (!response.ok) throw new JointJogError(result.error || "cartesian_resolver_unavailable", result.detail || "Cartesian jog refused");
    if (!Number.isSafeInteger(result.revision) || result.revision < 0 || typeof result.active !== "boolean") throw new JointJogError("rt_core_protocol_error", "Invalid Cartesian hold state");
    return result;
  };
  return { state: target => send("state", undefined, target), send: request,
    move: async (intent, signal, target) => {
      const response = await fetcher(`${DENSE_EXECUTION_PATH}/cartesian/move`, {
        method: "POST", credentials: "same-origin", redirect: "error", cache: "no-store", signal,
        headers: { "Content-Type": "application/json", ...(target ? { "X-RT-Target-Generation": target.selection_generation, "X-RT-Target-Cell": target.cell_id } : {}) },
        body: JSON.stringify(cartesianMoveBody(intent, frame())),
      });
      const result = await response.json();
      if (!response.ok) throw new JointJogError(result.error || "cartesian_resolver_unavailable", result.detail || "Cartesian move refused");
      if (result.state !== "done" || result.axis !== intent.axis || !Number.isSafeInteger(result.revision) || result.revision <= intent.revision) throw new JointJogError("rt_core_protocol_error", "Cartesian completion was not observed");
      return result;
    },
  };
}

/** Selected-machine Cartesian holds share joint jog's release and expiry
 * protocol. Selection, authority and status changes end the pinned hold. */
export class MachineCartesianJog {
  readonly #hold: JointJogHold;
  readonly #api: JointJogAPI;
  readonly #move: JointAngleMove;
  #mode: CartesianMode | null = null;
  readonly #changed: () => void;
  #frame = "base";
  #selected = false;
  #caps: DenseCapabilities | null = null;
  #status: DenseStatus | null = null;
  #identity: DenseStatus["target"];
  #sessionID: string | undefined;
  #stale = false;
  #remoteMove: JointAngleMove["state"] = { axis: -1, state: "", reason: "", revision: 0 };
  #disabled = "";
  #polling = false;
  #loggedRefusal = "";
  #steps: CartesianSteps = { ...CARTESIAN_STEP_DEFAULTS };
  #inputError = "";
  #fractionUpdates = false;
  constructor(changed: () => void, fetcher: typeof fetch = globalThis.fetch.bind(globalThis), refusal: (text: string) => void = () => {}) {
    this.#changed = () => {
      changed();
      const local = this.#move?.state;
      const move = local && (this.#move.busy || local.revision >= this.#remoteMove.revision) ? local : this.#remoteMove;
      const text = [this.#inputError, this.#hold?.reason, move.state === "refused" ? move.detail ?? rtCoreReasonText(move.reason) : ""].filter(Boolean).join("; ");
      if (text && text !== this.#loggedRefusal) refusal(text);
      this.#loggedRefusal = text;
    };
    this.#api = cartesianJogAPI(() => this.#frame, fetcher, () => this.#steps);
    this.#hold = new JointJogHold(this.#api, this.#changed);
    this.#move = new JointAngleMove(this.#api, this.#changed);
    for (const event of ["blur", "pagehide", "pointerup", "pointercancel"]) window.addEventListener(event, () => this.end(windowEndCause(event)));
    document.addEventListener("visibilitychange", () => { if (document.hidden) this.end("visibility_hidden"); });
  }
  get stepBusy(): boolean { return this.#move.busy || ["preparing", "moving"].includes(this.#remoteMove.state); }
  get busy(): boolean { return this.#hold.busy || this.stepBusy; }
  get mode(): CartesianMode { return this.#mode ?? cartesianDefaultMode(this.#status?.roundTripMS); }
  setMode(mode: CartesianMode): void { if (this.busy) return; this.end("mode_switch"); this.#inputError = ""; this.#mode = mode; this.#changed(); }
  get disabled(): string { return this.#disabled; }
  get reason(): string {
    if (this.#inputError) return this.#inputError;
    if (this.stepBusy && this.#stale) return "waiting for the machine";
    const local = this.#move.state;
    const move = this.#move.busy || cartesianCancelText(local.reason) || local.revision >= this.#remoteMove.revision ? local : this.#remoteMove;
    return cartesianCancelText(move.reason) ?? (move.reason ? move.detail ?? rtCoreReasonText(move.reason) : move.state || this.#hold.reason || this.#disabled);
  }
  /** `cause` is the End's end_cause (jogEndCause.ts). */
  end(cause: JogEndCause | "" = ""): void { void this.#hold.end(cause); }
  /** Speed change during a hold: a path hold (the server reports
   *  fraction_updates) keeps its line and changes speed; otherwise release. */
  setFraction(fraction: number): void {
    if (this.#fractionUpdates && this.#hold.setFraction(fraction)) return;
    this.end("speed_change");
  }
  stop(cause: CartesianCancelCause): void { this.end(cancelEndCause(cause)); this.#remoteMove = { axis: -1, state: "", reason: "", revision: 0 }; this.#move.cancel(cause); }
  /** Only confirmed identities cancel a step. Missing observations disable new
   * input and end holds, preventing status polling from aborting accepted work. */
  update(selected: boolean, caps: DenseCapabilities | null, status: DenseStatus | null, busy: boolean, fresh = true, model = status?.target?.model_id): void {
    const target = status?.target;
    if (target?.cell_id && target.selection_generation) {
      if (this.#identity && (target.cell_id !== this.#identity.cell_id || target.selection_generation !== this.#identity.selection_generation)) {
        this.stop("selection_changed"); this.#mode = null; this.#sessionID = undefined;
      }
      this.#identity = { ...target };
    }
    if (status?.session?.id) {
      if (this.#sessionID && status.session.id !== this.#sessionID) this.stop("session_changed");
      this.#sessionID = status.session.id;
    }
    this.#selected = selected; this.#caps = caps; this.#status = status;
    this.#stale = !fresh || !selected || !status?.target || !status.session || !status.rtCore || !!status.daemonError;
    if (status?.target && status.roundTripMS !== undefined) this.#mode ??= cartesianDefaultMode(status.roundTripMS);
    this.#disabled = unknownRobotModelReason(model, status?.target?.model_id) ||
      (robotModelClass(model ?? "") === "bench" || robotModelClass(status?.target?.model_id ?? "") === "bench"
        ? BENCH_CARTESIAN_UNAVAILABLE : jointJogDisabled(selected, caps, status, busy, {
          state: fresh && status?.armed === true ? "armed" : "unknown",
          controlled: status?.session?.leaseHeld === true, jogNote: "Arm state UNKNOWN: no fresh status from the machine.",
        }, model));
    if (this.#disabled) this.end("jog_disabled");
  }
  begin(frame: string, axes: number[], fraction: number, steps = readCartesianSteps()): void {
    // A path release still in flight does not refuse a hold press: the server
    // holds the press until that stop completes. A step still needs quiet.
    const busy = this.mode === "step" ? this.busy : this.#hold.pressBusy || this.stepBusy;
    if (!this.#selected || this.#disabled || busy || !this.#caps || !this.#status?.session) return;
    const axis = axes.findIndex(value => value !== 0);
    if (axes.length !== 6 || axis < 0 || axes.filter(value => value !== 0).length !== 1 || !["base", "tool"].includes(frame)) return;
    const direction = axes[axis]! < 0 ? -1 : axes[axis]! > 0 ? 1 : 0;
    if (direction === 0) return;
    this.#inputError = "";
    try { requireCartesianSteps(steps); } catch (error) {
      this.#inputError = error instanceof JointJogError ? error.message : CARTESIAN_INPUT_INVALID;
      this.#changed();
      return;
    }
    this.#frame = frame;
    this.#steps = { distanceMM: steps.distanceMM, angleDegrees: steps.angleDegrees };
    if (this.mode === "step") {
      void this.#move.begin(this.#caps.cell, this.#status.session.id, axis, direction * (axis < 3 ? steps.distanceMM : steps.angleDegrees), fraction, this.#status.target);
      return;
    }
    void this.#hold.begin(this.#caps.cell, this.#status.session.id, axis, direction, fraction, this.#status.target);
  }
  async poll(): Promise<void> {
    if (this.#polling || !this.#selected) return;
    this.#polling = true;
    const target = this.#identity, session = this.#sessionID;
    try {
      const result = await this.#api.state(target);
      if (session === this.#sessionID && target?.cell_id === this.#identity?.cell_id && target?.selection_generation === this.#identity?.selection_generation) {
        this.#fractionUpdates = result.fraction_updates === true;
        if (result.reason) this.#hold.observeReason(result.reason, result.revision);
        if (result.move && result.move.revision >= this.#remoteMove.revision) this.#remoteMove = result.move;
      }
    } catch (error) { this.#stale = true; if (this.#hold.active) this.#hold.observeFailure(error); }
    finally { this.#polling = false; this.#changed(); }
  }
}

/** A connected machine owns the pad even when disarmed or unavailable, so a
 * failed connection cannot silently change the operator's motion destination. */
export function cartesianPadPresentation(view: MachineFlowView | null, model = view?.modelId) {
  const reason = unknownRobotModelReason(model, view?.modelId);
  if (reason) return { tag: "UNKNOWN MODEL", heading: reason, previewArmVisible: false };
  if (robotModelClass(model ?? "") === "bench") return {
    tag: "BENCH CELL", heading: BENCH_CARTESIAN_UNAVAILABLE, previewArmVisible: false,
  };
  return {
    tag: view?.connected ? "SELECTED MACHINE" : "BROWSER PREVIEW",
    heading: cartesianMachineHeading(view),
    previewArmVisible: !view?.connected,
  };
}
