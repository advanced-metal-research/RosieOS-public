/**
 * Cartesian / tool jog in Offline Programming. This is the Motion Server
 * pendant pathway (`robot-v4-cartesiand`), not Tesseract planning.
 *
 * UI hold buttons → POST /local-simulator/jog/intent ~10 Hz → OLP streams
 * joystick UDP to an isolated cartesiand in ROBOT_V4_SIMULATION_MODE.
 * A classic program's Plan & simulate stays on /local-simulator/plans
 * (Tesseract); a rails program's Plan does not go near it.
 */

import type { Description, ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { mapRTCoreStatus } from "../targets/rtCoreStatus.ts";
import { createOperatorFence, type OperatorFeedback } from "../targets/operatorFeedback.ts";

export const LIVE_JOG_API = "/api/offline-programming/v1/local-simulator/jog";
export const TESSERACT_PLAN_API = "/api/offline-programming/v1/local-simulator/plans";

export const CARTESIAN_JOG_DISABLED_TITLE =
  "Cartesian / tool-frame jog needs the Motion Server jog runtime (robot-v4-cartesiand).";
export const CARTESIAN_JOG_DISARMED_TITLE =
  "Browser simulator jog is disarmed. Click Arm preview to allow it. The machine arms from the Machine panel, not here.";
// unverified UI wording; the jog endpoint controls simulation only.
export const CARTESIAN_JOG_ENABLED_TITLE =
  "Hold to jog the browser simulator; never a physical robot.";

export const JOG_START_MAX_ATTEMPTS = 8;
export const JOG_POLL_MS = 100;
export const JOG_HOLD_MS = 100;

// unverified UI preference key; shared by the guarded reader and writer.
export const JOG_PAD_STORAGE_KEY = "rosie.offline-programming.v1.jog-pad-visible";

/** Default closed if browser storage cannot be read, so boot still renders. */
export function readJogPadVisible(
  storage: () => Pick<Storage, "getItem"> = () => window.localStorage,
): boolean {
  try {
    return storage().getItem(JOG_PAD_STORAGE_KEY) === "true";
  } catch {
    return false;
  }
}

/** Remember visibility when possible; denied storage must not block toggling. */
export function writeJogPadVisible(
  visible: boolean,
  storage: () => Pick<Storage, "setItem"> = () => window.localStorage,
): void {
  try {
    storage().setItem(JOG_PAD_STORAGE_KEY, String(visible));
  } catch {
    // Keep the in-memory choice when browser storage is unavailable.
  }
}

/**
 * Rotary jog speed as a fraction of the daemon's MAX_JOG_ANGULAR_RAD_S (π
 * rad/s). Full stick spins the tool 180°/s, which makes tool-frame rotation
 * sloppy: the UI re-rotates the stick by the live tool orientation only per
 * 10 Hz send, so at 180°/s the commanded axis lags the tool by up to 18° and
 * the TCP wanders (measured ~16 cm drift per revolution at full stick vs
 * <1 cm per 90° at quarter stick). Linear axes stay at full stick — 0.2 m/s
 * is already a comfortable jog speed with no staleness problem.
 */
export const ROTARY_JOG_SCALE = 0.25;

/**
 * Scale a raw ±1 button axes vector into the intent the UI streams:
 * linear (x, y, z) at full stick, rotary (rx, ry, rz) at ROTARY_JOG_SCALE.
 */
export function jogButtonAxes(raw: readonly number[]): number[] {
  return raw.map((value, index) => (index < 3 ? value : value * ROTARY_JOG_SCALE));
}

export type LiveJogPath = "start" | "stop" | "intent" | "state" | "halt";

export type JogState = {
  operatorFeedback?: OperatorFeedback | null;
  running: boolean;
  jointsRad?: number[] | null;
  axisNames?: string[] | null;
  activeIntent?: boolean;
  lastCommandAge?: string;
  daemonAddress?: string;
  error?: string;
};

export type JogResponse = {
  ok: boolean;
  error: string;
  state: JogState | null;
};

export type CartesianJogView = {
  live: boolean;
  armed: boolean;
  frameDisabled: boolean;
  gridDisabled: boolean;
  buttonDisabled: boolean;
  title: string;
};

export function liveJogUrl(path: LiveJogPath): string {
  return `${LIVE_JOG_API}/${path}`;
}

export function jogUsesMotionServerNotTesseract(url: string = LIVE_JOG_API): boolean {
  return url.startsWith(LIVE_JOG_API) &&
    !url.includes("tesseract") &&
    !url.startsWith(TESSERACT_PLAN_API);
}

export function jogSessionIsLive(response: JogResponse | null | undefined): boolean {
  return response?.ok === true && response.state?.running === true;
}

export function cartesianJogDisabledTitle(error?: string): string {
  const detail = error?.trim() ?? "";
  return detail || CARTESIAN_JOG_DISABLED_TITLE;
}

/**
 * BASE/TOOL hold buttons require BOTH a live jog session and an explicit
 * user ARM. Fail closed: a live session with disarmed drives never jogs.
 */
export function cartesianJogView(live: boolean, armed: boolean, error?: string): CartesianJogView {
  const enabled = live && armed;
  const title = !live
    ? cartesianJogDisabledTitle(error)
    : armed
      ? CARTESIAN_JOG_ENABLED_TITLE
      : CARTESIAN_JOG_DISARMED_TITLE;
  return {
    live,
    armed,
    frameDisabled: !enabled,
    gridDisabled: !enabled,
    buttonDisabled: !enabled,
    title,
  };
}

export function parseJogResponse(payload: unknown, fence = createOperatorFence()): JogResponse {
  if (payload === null || typeof payload !== "object" || Array.isArray(payload)) {
    return { ok: false, error: "invalid jog response", state: null };
  }
  const record = payload as Record<string, unknown>;
  const state = parseJogState(record.state, fence);
  const error = typeof record.error === "string" ? record.error : state?.error ?? "";
  return {
    ok: record.ok === true,
    error,
    state,
  };
}

export function parseJogState(payload: unknown, fence = createOperatorFence()): JogState | null {
  if (payload === null || typeof payload !== "object" || Array.isArray(payload)) {
    return null;
  }
  const record = payload as Record<string, unknown>;
  let operatorFeedback: OperatorFeedback | null = null;
  if (record.rtCore && typeof record.rtCore === "object") {
    const observation = record.rtCore as {status: ProcessStatus; description: Description; reason?: string};
    try { operatorFeedback = mapRTCoreStatus(observation.status, [], observation.reason ?? "", observation.description, undefined, undefined, "jog", fence).operatorFeedback; } catch { /* Missing public evidence stays unavailable. */ }
  }
  return {
    operatorFeedback,
    running: record.running === true,
    jointsRad: numberArray(record.jointsRad),
    axisNames: stringArray(record.axisNames),
    activeIntent: record.activeIntent === true,
    lastCommandAge: typeof record.lastCommandAge === "string" ? record.lastCommandAge : "",
    daemonAddress: typeof record.daemonAddress === "string" ? record.daemonAddress : "",
    error: typeof record.error === "string" ? record.error : "",
  };
}

export function nextJogRetryDelayMs(attempt: number): number {
  const delays = [400, 800, 1500, 2500, 4000, 5000, 5000];
  const index = Math.max(0, Math.min(attempt, delays.length - 1));
  return delays[index]!;
}

export function shouldKeepTryingJogStart(live: boolean, attempt: number): boolean {
  return !live && attempt < JOG_START_MAX_ATTEMPTS;
}

function numberArray(value: unknown): number[] | null {
  if (!Array.isArray(value)) return null;
  return value.map((entry) => typeof entry === "number" && Number.isFinite(entry) ? entry : 0);
}

function stringArray(value: unknown): string[] | null {
  if (!Array.isArray(value)) return null;
  return value.map((entry) => typeof entry === "string" ? entry : "");
}
