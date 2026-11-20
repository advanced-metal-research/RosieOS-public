import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { unknownRobotModelReason } from "../targets/cells.ts";

/** All joints to 0° is also the one-press recovery (owner 2026-10-11: "we
 *  should also be able to recover by pressing the return all joints to zero
 *  button"). The OLP server runs it (POST .../recover-to-zero,
 *  `internal/denseexec/recover_zero_plan.go`): confirm an unconfirmed Stop from
 *  a fresh status, Reset faults whose recovery is a reset, Arm only after the
 *  operator confirms, then the unchanged All joints to 0° move. It stops at
 *  the first step that does not clear and says why. This file is the words
 *  and the confirmation, shared by the browser and the Deck (olp-core). */
export const RECOVER_TO_ZERO_ACTION = "Arm and move all joints to zero";
export const RECOVER_TO_ZERO_PATH = "/recover-to-zero";

export interface RecoverJoint { axis: number; id: string; from_deg: number; to_deg: number; delta_deg: number; moves: boolean }
export interface RecoverStep { step: string; outcome: string; detail?: string }
export interface RecoverRefusal { step: string; code: string; detail: string }
export type RecoverState = "done" | "at_zero" | "confirm_arm" | "refused";
export interface RecoverResult {
  state: RecoverState; action: string; steps: RecoverStep[]; joints: RecoverJoint[]; refusal: RecoverRefusal | null;
}

const isRecord = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
const num = (v: unknown): number => (typeof v === "number" && Number.isFinite(v) ? v : NaN);
const str = (v: unknown): string => (typeof v === "string" ? v : "");

/** The server's answer, or the `recovery` member of its 409. */
export function normalizeRecoverResult(raw: unknown): RecoverResult {
  const r = isRecord(raw) && isRecord(raw.recovery) ? raw.recovery : isRecord(raw) ? raw : {};
  const state = (["done", "at_zero", "confirm_arm", "refused"] as const).find(s => s === r.state) ?? "refused";
  const joints = Array.isArray(r.joints) ? r.joints.filter(isRecord).map(j => ({
    axis: num(j.axis), id: str(j.id), from_deg: num(j.from_deg), to_deg: num(j.to_deg), delta_deg: num(j.delta_deg), moves: j.moves === true,
  })) : [];
  const steps = Array.isArray(r.steps) ? r.steps.filter(isRecord).map(s => ({ step: str(s.step), outcome: str(s.outcome), detail: str(s.detail) })) : [];
  const refusal = isRecord(r.refusal) ? { step: str(r.refusal.step), code: str(r.refusal.code), detail: str(r.refusal.detail) } : null;
  return { state, action: str(r.action) || RECOVER_TO_ZERO_ACTION, steps, joints, refusal };
}

/** The request: the first press asks; the confirmation sends back the joints
 *  the operator saw, which the server checks against a fresh status. */
export interface RecoverJogSpeeds { linearSpeedMMs: number; angularSpeedDegS: number }

export function recoverRequest(fraction: number, confirmed?: RecoverResult, speeds?: RecoverJogSpeeds): Record<string, unknown> {
  if (speeds && (!Number.isFinite(speeds.linearSpeedMMs) || speeds.linearSpeedMMs <= 0 ||
    !Number.isFinite(speeds.angularSpeedDegS) || speeds.angularSpeedDegS <= 0)) {
    throw new Error("Choose positive tool travel and rotation speeds before returning to zero");
  }
  const request = confirmed
    ? { fraction, confirm_arm: true, confirmed: confirmed.joints.map(j => ({ axis: j.axis, id: j.id, from_deg: j.from_deg })) }
    : { fraction, confirm_arm: false };
  return speeds ? { ...request, linear_speed_mm_s: speeds.linearSpeedMMs, angular_speed_deg_s: speeds.angularSpeedDegS } : request;
}

/** Why the button cannot start a recovery now, or "". Unlike jogging it does
 *  not need the machine armed, a grant or a clean status: clearing those is
 *  what it is for. */
export function recoverToZeroDisabled(selected: boolean, caps: DenseCapabilities | null, status: DenseStatus | null, busy: boolean, model = status?.target?.model_id): string {
  const modelReason = unknownRobotModelReason(model, status?.target?.model_id);
  if (modelReason) return modelReason;
  if (!selected) return "Connect a machine first";
  if (caps?.backend !== "rt_core" || !caps.rtCore || !caps.cell) return "Describe the selected rt-core target first";
  if (!status) return "Waiting for the machine's first status";
  if (busy || status.session?.state === "playing") return "Another target operation is active";
  return "";
}

const deg = (v: number) => `${v.toFixed(2)}°`;
const signed = (v: number) => `${v > 0 ? "+" : v < 0 ? "−" : "±"}${Math.abs(v).toFixed(2)}°`;

export interface RecoverConfirmation { eyebrow?: string; title: string; intro: string; joints: { id: string; from: string; to: string; delta: string; moves: boolean }[]; lines: string[]; warning: string; confirm: string; cancel: string }

/** The confirmation before Arm: the action by name, each joint's angle, its
 *  target and travel, and the sweep warning. The operator decides. */
export function recoverConfirmation(result: RecoverResult, request?: unknown): RecoverConfirmation {
  const joints = result.joints.map(j => ({ id: j.id, from: deg(j.from_deg), to: deg(j.to_deg), delta: j.moves ? `Δ ${signed(j.delta_deg)}` : "stays", moves: j.moves }));
  const done = recoverStepsText(result.steps);
  const rates = isRecord(request) ? request : {};
  const fraction = num(rates.fraction), linear = num(rates.linear_speed_mm_s), angular = num(rates.angular_speed_deg_s);
  const bounded = fraction > 0 && fraction <= 1 && linear > 0 && angular > 0;
  const rateText = bounded ? ` (tool up to ${Number((linear * fraction).toPrecision(6))} mm/s and ${Number((angular * fraction).toPrecision(6))}°/s)` : "";
  return {
    title: result.action || RECOVER_TO_ZERO_ACTION,
    intro: `${done ? `${done}. ` : ""}The robot is disarmed. Confirming arms it and moves every joint to 0° at the selected speed${rateText}:`,
    joints,
    lines: joints.map(j => `${j.id}  ${j.from} → ${j.to}  ${j.delta}`),
    warning: "All joints move together along one straight line in joint space, not a straight tool path: the tool may sweep through the cell. Check the whole path is clear. STOP ends the move.",
    confirm: "Arm and move", cancel: "Cancel",
  };
}

const STEP_WORDS: Record<string, string> = { confirm_stop: "Stop confirmed", reset: "faults reset", arm: "armed", move: "moved to 0°" };

/** The steps that cleared, in operator words ("Stop confirmed · faults reset"). */
export function recoverStepsText(steps: readonly RecoverStep[]): string {
  return steps.filter(s => s.outcome === "done" && STEP_WORDS[s.step] && !(s.step === "move" && s.detail?.includes("already"))).map(s => STEP_WORDS[s.step]).join(" · ");
}

/** The outcome line under the button. A refusal is the server's sentence,
 *  prefixed with what cleared before it. */
export function recoverOutcomeText(result: RecoverResult): string {
  const done = recoverStepsText(result.steps);
  switch (result.state) {
    case "done": return `All joints at 0° (${done})`;
    case "at_zero": return done ? `Every joint is already at 0° (${done}; not armed)` : "Every joint is already at 0°";
    case "confirm_arm": return "Waiting for the confirmation to arm";
    case "refused": return `${done ? `${done}; then ` : ""}${result.refusal?.detail || "Recovery refused"}`;
  }
}

/** A refused request (HTTP error) in operator words: the server's detail for
 *  its own refusals and for the maintenance and control gates. */
export function recoverFailureText(code: string, detail: string, raw?: unknown): string {
  const result = raw !== undefined ? normalizeRecoverResult(raw) : null;
  if (result?.refusal) return recoverOutcomeText(result);
  return detail || code || "Recovery refused";
}
