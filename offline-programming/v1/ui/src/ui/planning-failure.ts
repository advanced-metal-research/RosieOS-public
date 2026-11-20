import { planRefusalFrom, planRefusalText } from "./progress-dialog.ts";
import { PLAN_START_MOVING, planMotionInstruction } from "../execution/runControls.ts";

export interface PlanningFailure {
  message: string;
  details?: string;
  action?: { key: "review-robot-settings" | "plan-from-program-start"; label: string };
}

/** The alternative when the robot's measured start could not be verified:
 *  plan from the program's own start, then Go to plan start before Load and
 *  Play. Offered, never taken: the operator presses it. */
export const PLAN_FROM_PROGRAM_START_ACTION = { key: "plan-from-program-start", label: "Plan from program start" } as const;

/** Explain known refusals without losing the planner's exact evidence.
 *  This does not adopt settings, change motion, or retry a plan.
 *  Only an unverified measured start offers program-start planning. Changing
 *  the start does not repair an internal MoveL, collision or service failure. */
export function planningFailure(error: unknown, device: "pendant" | "desktop" = "desktop", measuredStart = false): PlanningFailure {
  const failure = explained(error, device);
  const unverifiedStart = !!error && typeof error === "object" && (error as { code?: unknown }).code === "planning_start_unverified";
  return measuredStart && unverifiedStart && !failure.action ? { ...failure, action: { ...PLAN_FROM_PROGRAM_START_ACTION } } : failure;
}

function explained(error: unknown, device: "pendant" | "desktop"): PlanningFailure {
  const refusal = planRefusalFrom(error && typeof error === "object" && "reason" in error
    ? { dense_error: error } : error);
  const text = planRefusalText(refusal);
  // A jog or move still under way: one instruction, the server's words below.
  if (error && typeof error === "object" && (error as { code?: unknown }).code === PLAN_START_MOVING) {
    return { message: planMotionInstruction(device), details: `${PLAN_START_MOVING}: ${text}` };
  }
  if (/description moved under the program/i.test(text) ||
      /program is written for[\s\S]*at description[\s\S]*this request packs[\s\S]*sha256:/i.test(text)) {
    return {
      message: "Robot settings have changed since this program was created. Review and accept the current settings in Workspace, then plan again.",
      details: text,
      action: { key: "review-robot-settings", label: "Review robot settings" },
    };
  }
  if ((typeof refusal !== "string" && refusal.reason === "seam_not_planned") || /\bseam_not_planned\b/.test(text)) {
    const detail = typeof refusal === "string" ? text.replace(/\s*·?\s*seam_not_planned\b/, "").trim() : refusal.detail.trim();
    const seam = /\bseam\s+([^\s:;()]+)/i.exec(detail)?.[1] ?? (/^seam_[\w-]+$/.test(detail) ? detail : null);
    const locationOnly = !detail || /^(?:seam\s+[\w-]+|seam_[\w-]+|s\d+)$/.test(detail);
    return {
      message: `Planning stopped: no candidate motion path was found${seam ? ` for ${seam}` : " for this seam"}. ` +
        (locationOnly ? "The planner did not report why. Review the seam definition and torch orientation before planning again." :
          "Review the planner's explanation in Technical details before planning again."),
      details: text,
    };
  }
  if ((typeof refusal !== "string" && refusal.reason === "motion_not_verified") ||
      /\bmotion_not_verified\b|requires native PASS/i.test(text)) {
    return { message: unverifiedMotionMessage(text), details: text };
  }
  return { message: text };
}

/** Aggregate limit violations count checks, not distinct axes or position limits. */
function unverifiedMotionMessage(text: string): string {
  const seam = /\bseam\s+([^\s:;()]+)/i.exec(text)?.[1];
  const collisions = Number(/\b(\d+)\s+collisions?\b/i.exec(text)?.[1] ?? 0);
  const penetrating = Number(/\b(\d+)\s+penetrating\b/i.exec(text)?.[1] ?? 0);
  const limits = Number(/\b(\d+)\s+limit violations?\b/i.exec(text)?.[1] ?? 0);
  const counts: string[] = [];
  if (collisions > 0) counts.push(`${collisions} collision${collisions === 1 ? "" : "s"}` +
    (penetrating > 0 ? ` (${penetrating} overlap${penetrating === 1 ? "" : "s"})` : ""));
  if (limits > 0) counts.push(`${limits} limit-check failure${limits === 1 ? "" : "s"}`);
  const message = [`Planning stopped: no verified motion path is available${seam ? ` for ${seam}` : ""}.`];
  if (counts.length) message.push(`Checks reported ${counts.join(" and ")}.`);
  const collision = collisions > 0 || /contact or overlap between|collision on|collision between/i.test(text);
  const rate = /(?:velocity|acceleration) (?:magnitude|limit)|travel[ _]speed/i.test(text);
  const position = /\bposition\s+[^;]*\boutside\b/i.test(text);
  const singular = /\bsingularity\b|\bsingular\b|cannot move along the weld direction/i.test(text);
  if (collision) message.push("Review the seam, torch pose and part placement for collisions.");
  if (rate) message.push("Review the requested weld speed and the reported speed or acceleration limits.");
  if (position) message.push("Review the reported joint positions and limits.");
  if (singular) message.push("Review the torch orientation and robot posture near the reported singularity.");
  if (limits > 0 && !rate && !position && !singular) message.push("The limit checks can cover position, speed, acceleration or other motion constraints; they do not count distinct joints.");
  message.push("See Technical details for the failed checks before planning again.");
  return message.join(" ");
}
