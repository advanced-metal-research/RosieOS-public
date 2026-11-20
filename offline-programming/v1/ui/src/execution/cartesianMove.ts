import { JointJogError } from "./jointJog.ts";

// Unverified LAN round-trip bound: 20 ms, matching controller timing selection.
export const CARTESIAN_LAN_RTT_MS = 20;
// Unverified operator defaults: 100 mm (or mm/s) translation and 15 degrees (or deg/s) rotation.
export const CARTESIAN_STEP_DEFAULTS = { distanceMM: 100, angleDegrees: 15 };
// Unverified operator bounds: 1000 mm or mm/s linear, 180 degrees or deg/s angular.
export const CARTESIAN_LINEAR_LIMIT = 1000;
export const CARTESIAN_ANGULAR_LIMIT = 180;
export const CARTESIAN_INPUT_INVALID = "Enter a distance from 0 to 1000 mm (or mm/s) and an angle from 0 to 180 degrees (or deg/s), excluding zero";
// Unverified browser preference namespace; carries no machine authority.
const STORAGE_KEY = "offline-programming.cartesian-step";
export type CartesianMode = "hold" | "step";
export type CartesianSteps = typeof CARTESIAN_STEP_DEFAULTS;
export type CartesianCancelCause = "stop" | "disarm" | "selection_changed" | "session_changed";

/** Name the cancelling action without attributing status loss or another
 * window's selection to the operator pressing STOP. */
export function cartesianCancelText(reason: string): string | undefined {
  switch (reason) {
    case "stop": return "Move cancelled: STOP pressed";
    case "disarm": return "Move cancelled: DISARM pressed";
    case "selection_changed": return "Move cancelled: selection changed in another window";
    case "session_changed": return "Move cancelled: machine session changed";
    case "client_disconnected": return "Move connection lost: client disconnected; read machine state before retrying";
    default: return undefined;
  }
}

/** Unknown latency selects Step; only a measured LAN path defaults to Hold,
 * preventing a remote connection from depending on pointer cadence. */
export function cartesianDefaultMode(roundTripMS: number | undefined): CartesianMode {
  return roundTripMS !== undefined && Number.isFinite(roundTripMS) && roundTripMS >= 0 && roundTripMS <= CARTESIAN_LAN_RTT_MS ? "hold" : "step";
}

/** Persist only finite positive operator inputs; corrupt storage cannot send
 * a zero, reversed, or unbounded displacement to the machine. */
export function readCartesianSteps(storage?: Pick<Storage, "getItem">) {
  try {
    storage ??= globalThis.localStorage;
    const value = JSON.parse(storage?.getItem(STORAGE_KEY) ?? "null");
    if (validSteps(value)) return { distanceMM: value.distanceMM, angleDegrees: value.angleDegrees };
  } catch { /* Browser storage is optional. */ }
  return { ...CARTESIAN_STEP_DEFAULTS };
}
function validSteps(value: CartesianSteps | null): value is CartesianSteps {
  return !!value && Number.isFinite(value.distanceMM) && value.distanceMM > 0 && value.distanceMM <= CARTESIAN_LINEAR_LIMIT &&
    Number.isFinite(value.angleDegrees) && value.angleDegrees > 0 && value.angleDegrees <= CARTESIAN_ANGULAR_LIMIT;
}
export function writeCartesianSteps(value: CartesianSteps, storage?: Pick<Storage, "setItem">): boolean {
  if (!validSteps(value)) return false;
  try { (storage ?? globalThis.localStorage)?.setItem(STORAGE_KEY, JSON.stringify(value)); } catch { /* Keep current inputs when storage is unavailable. */ }
  return true;
}

/** Refuse zero, negative, NaN and out-of-range operator numbers before a hold
 * or step can put motion on the wire. */
export function requireCartesianSteps(value: CartesianSteps): CartesianSteps {
  if (!validSteps(value)) throw new JointJogError("cartesian_input_invalid", CARTESIAN_INPUT_INVALID);
  return value;
}

/** Hold twist in SI: the box is the speed at 100 % (mm/s → m/s on axes 0..2,
 * deg/s → rad/s on axes 3..5). Speed % is NOT applied here: the Cartesian
 * resolver multiplies the twist by the fraction it receives
 * (motion-server/v1/src/cartesian_resolver.hpp, velocity_vector = twist *
 * fraction), so 100 mm/s at 25 % is 25 mm/s on the wire to the joints; scaling
 * in the browser as well made it 6.25 mm/s. The pad supplies only the sign, and
 * the fraction travels beside the twist as both that scale and the per-joint
 * ceiling. */
export function cartesianHoldTwist(axis: number, direction: number, steps: CartesianSteps, fraction: number): number[] {
  requireCartesianSteps(steps);
  if (!Number.isInteger(axis) || axis < 0 || axis > 5 || (direction !== 1 && direction !== -1) ||
      !Number.isFinite(fraction) || fraction <= 0 || fraction > 1) {
    throw new JointJogError("cartesian_input_invalid", CARTESIAN_INPUT_INVALID);
  }
  const twist = [0, 0, 0, 0, 0, 0];
  twist[axis] = axis < 3 ? direction * steps.distanceMM / 1000 : direction * steps.angleDegrees * Math.PI / 180;
  return twist;
}

/** Labels and help name the unit the current mode gives the shared boxes. */
export function cartesianStepLabels(mode: CartesianMode): { linear: string; angular: string; help: string } {
  if (mode === "hold") {
    return {
      linear: "Speed at 100% (mm/s)",
      angular: "Rate at 100% (°/s)",
      help: "Hold moves at Speed % of these boxes while a button is held: 100 mm/s at 25% is 25 mm/s on X, Y and Z, and Rate scales the same way on RX, RY and RZ; no joint exceeds Speed % of its described maximum. Stop or Disarm cancels it.",
    };
  }
  return {
    linear: "Distance (mm)",
    angular: "Angle (degrees)",
    help: "Step sends one move of this distance or angle per press; Stop or Disarm cancels it. Hold moves while a button is held and suits a local link. Step is chosen for you on a slow link.",
  };
}

/** Build exactly one signed displacement; the chosen frame is resolved from
 * server feedback, so no browser TCP pose can replace the machine's origin. */
export function cartesianMoveBody(intent: { target_id: string; session_id: string; revision: number; axis: number; degrees: number; fraction: number }, frame: string) {
  const amount = intent.degrees;
  if (!["base", "tool"].includes(frame) || !Number.isInteger(intent.axis) || intent.axis < 0 || intent.axis > 5 ||
      !Number.isFinite(amount) || amount === 0 || Math.abs(amount) > (intent.axis < 3 ? CARTESIAN_LINEAR_LIMIT : CARTESIAN_ANGULAR_LIMIT)) {
    throw new JointJogError("cartesian_input_invalid", CARTESIAN_INPUT_INVALID);
  }
  return { target_id: intent.target_id, session_id: intent.session_id, revision: intent.revision, frame, axis: intent.axis,
    distance_mm: intent.axis < 3 ? amount : 0, angle_degrees: intent.axis >= 3 ? amount : 0, fraction: intent.fraction };
}
