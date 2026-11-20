/**
 * Which robot a plan was made for, against which robot the target is
 * (docs/ROBOT-DESCRIPTION-CONTRACT.md).
 *
 * The daemon is the judge: it refuses a plan whose header names another
 * description or machine_planning_calibration.json, at load and again at play. This module only
 * says so before the button is pressed, in words, and turns the daemon's
 * refusal codes into sentences an operator can act on.
 */

import type { DenseRobotRef } from "../domain/project.ts";
import type { DenseRobotCell } from "./denseClient.ts";

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);

/** The header's (or the plan summary's) `robot` block, or null when it has
 *  none or it is not whole. */
export function robotRefFrom(value: unknown): DenseRobotRef | null {
  if (!isRecord(value)) return null;
  const { model_id, robot_description_sha256, machine_planning_calibration_sha256, machine_planning_calibration_source } = value;
  if (typeof model_id !== "string" || typeof robot_description_sha256 !== "string" ||
    typeof machine_planning_calibration_sha256 !== "string" || (machine_planning_calibration_source !== "target" && machine_planning_calibration_source !== "empty")) {
    return null;
  }
  const recorded = value.machine_configuration_sha256;
  return {
    modelId: model_id, robotDescriptionSha256: robot_description_sha256,
    machinePlanningCalibrationSha256: machine_planning_calibration_sha256, machinePlanningCalibrationSource: machine_planning_calibration_source,
    ...(typeof recorded === "string" && recorded ? { machineConfigurationSha256: recorded } : {}),
  };
}

/** The `robot` block of a dense blob's header, read without decoding the
 *  samples. Null when the blob has none or is not readable. */
export function robotRefFromBlob(blob: Uint8Array): DenseRobotRef | null {
  if (blob.byteLength < 4) return null;
  const length = new DataView(blob.buffer, blob.byteOffset, blob.byteLength).getUint32(0, false);
  if (4 + length > blob.byteLength) return null;
  try {
    const header: unknown = JSON.parse(new TextDecoder().decode(blob.subarray(4, 4 + length)));
    return isRecord(header) ? robotRefFrom(header.robot_cell) : null;
  } catch {
    return null;
  }
}

const short = (value: string): string => value.replace(/^sha256:/, "").slice(0, 12) || "none";

/**
 * Why the target will refuse this plan, or null when it will not (or cannot
 * say: a daemon from before the contract reports no identity, and judges by
 * its own rules). Matches the daemon's checks, in its order.
 */
export function robotMismatch(plan: DenseRobotRef | null | undefined, target: DenseRobotCell | null): string | null {
  if (!target) return null;
  // The target first, as the server judges it: a machine that cannot say which
  // robot it is refuses every plan, and "re-plan it" would be the wrong advice.
  if (!target.configured) return "the target's daemon has no robot description, so it refuses every plan";
  if (!target.valid) return `the target's robot identity is invalid: ${target.error ?? "no reason given"}`;
  if (!plan) return "this plan names no robot description or cell (planned before the cell checked) — re-plan it";
  const differs: string[] = [];
  if (plan.modelId !== target.modelId) differs.push(`model ${plan.modelId} ≠ ${target.modelId}`);
  if (plan.robotDescriptionSha256 !== target.robotDescriptionSha256) {
    differs.push(`description ${short(plan.robotDescriptionSha256)} ≠ ${short(target.robotDescriptionSha256)}`);
  }
  if (plan.machinePlanningCalibrationSha256 !== target.machinePlanningCalibrationSha256) {
    differs.push(`machine_planning_calibration.json ${short(plan.machinePlanningCalibrationSha256)} ≠ ${short(target.machinePlanningCalibrationSha256)}`);
  }
  if (differs.length === 0) return null;
  return `planned against another robot or cell — re-fetch cell settings and re-plan (${differs.join(", ")})`;
}

/**
 * What to say when the plan records another machine configuration than the
 * target has now, or null. Not a refusal: that digest also moves for changes
 * no plan depends on (a CPU number, a bus timeout), so the fence stays the
 * description and the planning calibration, and this is only worth knowing.
 */
export function machineConfigurationNote(plan: DenseRobotRef | null | undefined, target: DenseRobotCell | null): string | null {
  const planned = plan?.machineConfigurationSha256 ?? "";
  const now = target?.machineConfigurationSha256 ?? "";
  if (!planned || !now || planned === now) return null;
  return `the machine's configuration changed since this plan was made (${short(planned)} → ${short(now)}); the description and planning calibration still match`;
}

/** An operator's sentence for a robot-identity refusal code, or null for any
 *  other code. */
export function robotRefusalSentence(code: string): string | null {
  switch (code) {
    case "robot_cell_missing":
      return "the plan does not say which robot it was made for; re-plan it";
    case "robot_cell_unavailable":
      return "the robot cannot say which robot it is (no description, or its machine_planning_calibration.json is invalid)";
    case "robot_cell_mismatch":
      return "planned against another robot or cell — re-fetch cell settings and re-plan";
    case "robot_cell_changed":
      return "the robot's description or machine_planning_calibration.json changed during playback; stopped — re-fetch cell settings and re-plan";
    case "robot_cell_invalid":
      return "the plan's robot identity block is malformed";
    default:
      return null;
  }
}

/** Short form of the target's identity for the runtime line. */
export function describeRobotCell(identity: DenseRobotCell): string {
  if (!identity.configured) return "no robot description";
  if (!identity.valid) return `robot identity INVALID: ${identity.error ?? "?"}`;
  return `${identity.modelId} · description ${short(identity.robotDescriptionSha256)} · machine_planning_calibration.json ${identity.machinePlanningCalibrationPresent ? short(identity.machinePlanningCalibrationSha256) : "none"}`;
}

/**
 * What a plan says about the cell it was made for, when nobody has asked the
 * cell. `machine_planning_calibration_source: "empty"` means the planner could not reach a target and
 * planned against the model's empty document: no narrowed limit, no measured
 * correction. A cell that states none will play it; a cell that states any
 * will refuse it, which is the same refusal as any other mismatch and not a
 * worse one. Null when the plan did read a cell.
 */
export function plannedWithoutACell(plan: DenseRobotRef | null | undefined): string | null {
  if (!plan || plan.machinePlanningCalibrationSource !== "empty") return null;
  return "planned without a cell: no target was read, so no narrowed limit or measured "
    + "correction is in it. A cell that states any will refuse it — pull its settings and plan again";
}
