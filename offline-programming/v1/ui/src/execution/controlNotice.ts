/**
 * "Another controller is using the robot": what the browser pop-up and the
 * Steam Deck overlay show, decided from the OLP server's control view
 * (control.go) -- the `control` member of the maintenance route both clients
 * already poll, and of every 409 `in_use_by_other_controller` refusal.
 *
 * The server enforces the rule; this only words it. It never offers Take
 * control unless the server said it would accept it, and it never offers
 * Stop for a holder this server cannot stop (another server's grant). An
 * older server sends no `control`: nothing is shown.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no Object.entries.
 */
import { maintenanceAgo } from "./maintenanceSessions.ts";

export const CONTROL_REFUSAL = "in_use_by_other_controller";
export const CONTROL_API = "/api/offline-programming/v1/control";
export const CONTROL_TAKE_API = "/api/offline-programming/v1/control/take";
/** One per browser tab and one per pendant process (control.go). */
export const CLIENT_ID_HEADER = "X-Rosie-Client-Id";

export type ControlState = "free" | "yours" | "other" | "other_server";

export interface ControlOwnerView {
  kind: string;
  label: string;
  address: string;
  loopback: boolean;
  since: string;
  lastAction: string;
  lastActionAt: string;
  running: string;
  idleMs: number;
}

export interface ServoHolderView {
  controller: string;
  remote: boolean;
  identity: string;
  since: string;
}

export interface ControlView {
  state: ControlState;
  owner: ControlOwnerView | null;
  armed: boolean;
  leaseHeld: boolean;
  sessionState: string;
  servoHolder: ServoHolderView | null;
  takeControl: boolean;
  takeControlNote: string;
  stopAvailable: boolean;
  detail: string;
  now: string;
}

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? value as Record<string, unknown> : {};
}

function str(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

/** The server's control view, or null when it sent none (an older server). */
export function normalizeControl(value: unknown): ControlView | null {
  const body = record(value);
  const state = str(body.state);
  if (state !== "free" && state !== "yours" && state !== "other" && state !== "other_server") return null;
  const owner = record(body.owner);
  const servo = record(body.servo_holder);
  return {
    state,
    owner: str(owner.label) ? {
      kind: str(owner.kind), label: str(owner.label), address: str(owner.address), loopback: owner.loopback === true,
      since: str(owner.since), lastAction: str(owner.last_action), lastActionAt: str(owner.last_action_at),
      running: str(owner.running), idleMs: typeof owner.idle_ms === "number" ? owner.idle_ms : 0,
    } : null,
    armed: body.armed === true,
    leaseHeld: body.lease_held === true,
    sessionState: str(body.session_state),
    servoHolder: str(servo.controller) ? {
      controller: str(servo.controller), remote: servo.remote === true, identity: str(servo.identity), since: str(servo.since),
    } : null,
    takeControl: body.take_control === true,
    takeControlNote: str(body.take_control_note),
    stopAvailable: body.stop_available !== false,
    detail: str(body.detail),
    now: str(body.now),
  };
}

/** The control view a refusal carries: `{error: in_use_by_other_controller, control}`. */
export function controlFromRefusal(body: unknown): ControlView | null {
  const value = record(body);
  return value.error === CONTROL_REFUSAL ? normalizeControl(value.control) : null;
}

export interface ControlNotice {
  /** Another controller is using the robot. */
  active: boolean;
  /** Shown as the full card; false: the slim bar after Wait. */
  expanded: boolean;
  kicker: string;
  title: string;
  who: string;
  what: string;
  since: string;
  /** What Take control would do, or why it is not offered. */
  note: string;
  canTake: boolean;
  /** This server's Stop reaches the robot. */
  stopAvailable: boolean;
  /** When it does not: whose Stop does. */
  stopNote: string;
  /** The slim bar's one line. */
  summary: string;
  /** Identifies the situation: a new holder, or a new refusal, re-expands. */
  key: string;
}

const HIDDEN: ControlNotice = {
  active: false, expanded: false, kicker: "", title: "", who: "", what: "", since: "", note: "",
  canTake: false, stopAvailable: true, stopNote: "", summary: "", key: "",
};

function clock(iso: string): string {
  const ms = Date.parse(iso);
  if (!Number.isFinite(ms)) return "";
  const d = new Date(ms);
  const two = (n: number) => (n < 10 ? "0" : "") + n;
  return `${two(d.getHours())}:${two(d.getMinutes())}:${two(d.getSeconds())}`;
}

function sinceText(iso: string, nowMs: number): string {
  const ago = maintenanceAgo(iso, nowMs);
  if (!ago) return "";
  const at = clock(iso);
  return at ? `since ${ago} (${at})` : `since ${ago}`;
}

function activity(view: ControlView, nowMs: number): string {
  const parts: string[] = [];
  if (view.armed) parts.push("ARMED");
  else if (view.leaseHeld) parts.push("holds the control session (disarmed)");
  const owner = view.owner;
  if (owner && owner.running) parts.push(`running ${owner.running}`);
  else if (owner && owner.lastAction && owner.lastAction !== "take") {
    const at = maintenanceAgo(owner.lastActionAt, nowMs);
    parts.push(`last request ${owner.lastAction}${at ? ` ${at}` : ""}`);
  }
  if (view.sessionState === "playing") parts.push("program playing");
  else if (view.sessionState === "loaded") parts.push("program loaded");
  return parts.join(" · ") || "connected";
}

/**
 * The notice for one control view. `dismissedKey`: the situation the operator
 * pressed Wait on, shown as the slim bar until it changes. `nowMs`: the
 * server's clock as the client estimates it.
 */
export function controlNoticeOf(view: ControlView | null, nowMs: number, dismissedKey = ""): ControlNotice {
  if (!view || (view.state !== "other" && view.state !== "other_server")) return HIDDEN;
  if (view.state === "other_server" && view.servoHolder) {
    const holder = view.servoHolder;
    const key = `server:${holder.controller}:${holder.since}`;
    const where = holder.remote ? `remote ${holder.identity || "client"}` : "local socket on the servo host";
    return {
      active: true, expanded: key !== dismissedKey,
      kicker: "ROBOT IN USE · ANOTHER SERVER",
      title: "Another controller is using the robot",
      who: `${holder.controller} · ${where}`,
      what: "holds the servo host's control grant",
      since: sinceText(holder.since, nowMs),
      note: view.takeControlNote || "Control cannot be taken from another server; its operator releases it by disarming.",
      canTake: false,
      stopAvailable: false,
      stopNote: "This server cannot stop it. Use that controller's Stop or the hardware stop.",
      summary: `Robot in use by ${holder.controller} (another server)`,
      key,
    };
  }
  const owner = view.owner;
  if (!owner) return HIDDEN;
  const key = `client:${owner.kind}:${owner.label}:${owner.address}:${owner.since}`;
  return {
    active: true, expanded: key !== dismissedKey,
    kicker: view.armed ? "ROBOT IN USE · ARMED" : "ROBOT IN USE",
    title: "Another controller is using the robot",
    who: `${owner.label} · ${owner.loopback ? `from ${owner.address} (this host or a tunnel)` : `from ${owner.address}`}`,
    what: activity(view, nowMs),
    since: sinceText(owner.since, nowMs),
    note: view.takeControlNote,
    canTake: view.takeControl,
    stopAvailable: view.stopAvailable,
    stopNote: "",
    summary: `Robot in use by ${owner.label}${view.armed ? " · ARMED" : ""}`,
    key,
  };
}

/**
 * The client's memory between polls: the last control view (from the poll or
 * a refusal, whichever is newer) and the situation the operator waited on.
 * A refusal re-expands even a waited-on situation: the operator just tried
 * to act.
 */
export class ControlTracker {
  #view: ControlView | null = null;
  #dismissed = "";
  #offsetMs = 0;

  /** A poll's answer: the maintenance route's body. */
  observe(body: unknown, nowMs: number): void {
    const value = record(body);
    this.#view = normalizeControl(value.control);
    this.#clock(this.#view, nowMs);
    if (!this.#view || this.#view.state === "free" || this.#view.state === "yours") this.#dismissed = "";
  }

  /** A refusal's body; true when it was a control refusal. */
  refused(body: unknown, nowMs: number): boolean {
    const view = controlFromRefusal(body);
    if (!view) return false;
    this.#view = view;
    this.#clock(view, nowMs);
    this.#dismissed = "";
    return true;
  }

  /** Take control was answered with the requester's view. */
  took(body: unknown, nowMs: number): void {
    const view = normalizeControl(body);
    if (view) {
      this.#view = view;
      this.#clock(view, nowMs);
    }
  }

  /** Wait: collapse to the slim bar until the situation changes. */
  wait(nowMs: number): void {
    this.#dismissed = this.notice(nowMs).key;
  }

  /** Details: the full card again. */
  expand(): void {
    this.#dismissed = "";
  }

  get view(): ControlView | null {
    return this.#view;
  }

  notice(nowMs: number): ControlNotice {
    return controlNoticeOf(this.#view, nowMs + this.#offsetMs, this.#dismissed);
  }

  #clock(view: ControlView | null, nowMs: number): void {
    const server = view ? Date.parse(view.now) : Number.NaN;
    if (Number.isFinite(server)) this.#offsetMs = server - nowMs;
  }
}
