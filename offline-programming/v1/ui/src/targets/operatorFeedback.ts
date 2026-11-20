import type { AxisDescription, BrakeHoldDiagnostic, Description, DriveSupervisorDiagnostic, ProcessStatus, FaultRecovery, Event } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { operatorAcceptanceClears } from "./operatorKinds.ts";

// Unverified display policy: 10% gives early context, 2% emphasizes the boundary.
// These fractions are not stopping envelopes and never authorize motion.
const approachFraction = 0.10, nearFraction = 0.02;

export interface OperatorRejection { unattributed?: boolean; sourceSequence: number; commandKind: string; code: string; text: string; axisMask: number }
export interface OperatorLimit { lowerDistance: number; upperDistance: number; lowerFraction: number; upperFraction: number; band: string; text: string }
export interface OperatorReset { admissible: boolean; text: string; reason: string; sequence: number; faults: FaultRecovery[] }
export interface OperatorAxis { index: number; rejection: OperatorRejection | null; limit: OperatorLimit | null; homeLoss: string; reset: OperatorReset; alarmText: string;
  /** The faulted drive's alarm in short, "J5 Er41.1" (what Arm resets first); empty when the drive is not in Fault. */
  alarmShort?: string;
  /** Latest native brake-hold warning for this axis; empty when none. */
  brakeWarning?: string;
  /** Why this axis ended the last Arm attempt; present until the next Arm request. */
  armEnded?: string;
  /** Latest disarm-motion warning for this axis; empty when none. */
  supervisorWarning?: string }
export interface OperatorFeedback { stale?: OperatorRejection; rejection: OperatorRejection | null; reset: OperatorReset; axes: OperatorAxis[];
  /** Additional display context (preview state or an observed execution fault).
   * Never changes rejection codes or recovery permissions. */
  note?: string;
  /** Non-blocking warnings (brake hold, disarm motion). Never a refusal, fault or Arm gate. */
  warnings?: string[];
  /** Reasons the last Arm attempt ended (drive-state supervisor). Retryable:
   * nothing is latched and no reset is needed. */
  armEnded?: string[] }

// Guard causes that are not brake take-up evidence. Current natives report
// them through the drive-state supervisor instead; older natives labelled
// them as brake warnings, so they are worded here without a brake remedy.
const nonBrakeCauses: Record<string, string> = {
  opening_timeout: "the drive did not finish enabling within the opening bound",
  frame_changed: "its coordinate frame changed",
  stale_feedback: "drive feedback was stale",
  observation_time: "drive feedback timing was invalid",
  drive_state: "the drive left its expected state",
  opening_proof: "the enable command receipt was missing",
  invalid_configuration: "the brake configuration is invalid",
};

/** Leads every suspected-slip warning, so renderers can set it apart. */
export const brakeSlipPrefix = "Brake may be slipping:";

/** Operator text for a native brake-hold warning. The drive manages its brake,
 * so brake evidence is never a fault, reset or Arm blocker and the native
 * message names the drive delay to adjust. A slip warning instead names
 * repeated same-direction movement within one braked period, which no delay
 * setting explains, and asks for the brake to be inspected. A non-brake cause
 * from an older native names no brake remedy and does not claim Arm is
 * unaffected. Degrees appear only for an axis described in radians. */
export function brakeWarningText(axisId: string, w: BrakeHoldDiagnostic | undefined,
    axis?: Pick<AxisDescription, "position_unit" | "counts_per_unit" | "sign">): string {
  if (!w || w.severity !== "warning" || !(w.warning_sequence > 0)) return "";
  const nonBrake = nonBrakeCauses[w.cause];
  if (nonBrake) return `${axisId} drive check: ${nonBrake} (${w.cause} during ${w.phase}). This is not a brake delay setting; if motion is not ready, Stop and retry Arm.`;
  const perRad = axis?.position_unit === "rad" && Number.isFinite(axis.counts_per_unit) && axis.counts_per_unit > 0
    ? axis.counts_per_unit / (axis.sign === -1 ? -1 : 1) : 0;
  const degrees = (counts: number) => (counts / perRad * 180 / Math.PI).toFixed(4);
  if (w.kind === "slip") {
    const total = w.same_direction_displacement_counts;
    return `${brakeSlipPrefix} ${axisId} moved ${w.same_direction_count} times in the same direction ` +
      `(total ${total} counts${perRad ? `, ${degrees(total)} deg` : ""}) while the brake was applied - inspect the brake. ` +
      "Not a fault; Arm remains available.";
  }
  const angle = (counts: number, suffix = "") => perRad ? ` (${degrees(counts)}°${suffix})` : "";
  const torqueOn = w.transition === "torque_on";
  const message = w.message || (torqueOn
    ? "Brake appears to release after torque on - adjust drive brake release delay (C05.13)."
    : "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).");
  const moved = `moved ${w.displacement_counts} counts${angle(w.displacement_counts)}`;
  const peak = w.peak_displacement_counts !== w.displacement_counts
    ? `, peak ${w.peak_displacement_counts} counts${angle(w.peak_displacement_counts)}` : "";
  const speed = w.peak_velocity_counts_per_s ? `, peak ${w.peak_velocity_counts_per_s} counts/s${angle(w.peak_velocity_counts_per_s, "/s")}` : "";
  const rest = w.settled ? `; settled ${w.settled_displacement_counts} counts${angle(w.settled_displacement_counts)} from the anchor` : "; settling";
  const effect = torqueOn
    ? "Warning only: the drive's release delay still gates motion; nothing to reset."
    : "Warning only: nothing to reset, and Arm is not blocked.";
  return `${axisId} brake warning: ${message} Detected ${w.cause} during ${w.phase}: ${moved}${peak}${speed}${rest}. ${effect}`;
}

/** Operator text for a native drive-state supervisor event, split by effect:
 * an Arm-ending event is shown only while it is the reason for the last Arm
 * attempt; disarm motion is a warning. Neither is a fault or needs reset. */
export function driveSupervisorText(axisId: string, d: DriveSupervisorDiagnostic | undefined): { armEnded?: string; warning?: string } {
  if (!d || !(d.event_sequence > 0)) return {};
  const fallback: Record<string, string> = {
    enable_timeout: `${axisId} did not reach Operation Enabled in time. Arm ended; retry Arm.`,
    release_timeout: `${axisId} brake did not report released in time. Arm ended; retry Arm.`,
    left_operation_enabled: `${axisId} left Operation Enabled while armed. Arm ended; retry Arm.`,
    arm_cancelled: `${axisId} Arm ended because ${d.cause.replaceAll("_", " ")}; retry Arm.`,
    disarm_motion: `${axisId} moved while disarming (drive still energized).`,
  };
  const message = d.message || fallback[d.kind] || `${axisId} drive supervisor: ${d.kind}.`;
  if (d.arm_ended) return d.current ? { armEnded: message } : {};
  return { warning: `${message} Warning only: nothing latched; check the axis before the next Arm.` };
}

export interface OperatorCommandFence { generation: number; observed: number; accepted: number }
export interface OperatorFence { identity?: string; kinds: Record<string, OperatorCommandFence>; stale?: OperatorRejection; rejection?: OperatorRejection | null; receiptAfter?: number; after: number }

/** Own receipt fences per consumer; missing input must not erase command history. */
export function createOperatorFence(): OperatorFence { return { kinds: {}, after: 0 }; }

/** UI explanation for plan admission, separate from the shared native reason labels. */
export function processIOAdmissionMessage(reason: string): string | undefined {
  if (reason.split(":", 1)[0]!.trim() !== "process_io_executor_not_qualified") return undefined;
  return "This plan requests welding or process outputs that this machine is not configured to execute. Select Dry run, then load again to test the motion with welding output off; no replanning is needed, or use a machine with qualified process outputs to weld.";
}

/** Event wording is UI-only; unknown events retain their original detail. */
export function controllerFaultEventMessage(reason: string): string | undefined {
  const parts = reason.split(":").map(part => part.trim());
  if (parts.length < 2 || parts.at(-1) !== "fault_latched" || parts.slice(0, -1).some(part => part !== "rt_core_event")) return undefined;
  return "The controller stopped the run because a fault was latched.";
}

/** Native grant reason 3 is a generation mismatch, not a Home or joint-limit fault. */
export function controlGrantMessage(reason: string): string | undefined {
  if (!/^RTCore rejected application grant request \d+ \(reason 3\)$/.test(reason.trim())) return undefined;
  return "The control request used an out-of-date controller state and was refused. Reconnect to the machine, then review Home and Arm before retrying.";
}

export function operatorRejection(code: string, axisMask = 0, commandKind = "unknown", sourceSequence = 0): OperatorRejection {
  const home = code.split(":", 1)[0]!.trim() === "home_required"
    ? "Home is required before loading. Review the Home step to establish the current joint position references." : undefined;
  return { sourceSequence, commandKind, code, text: home ?? controlGrantMessage(code) ?? processIOAdmissionMessage(code) ?? operatorReasonLabels[code] ?? "Unknown rt-core refusal", axisMask };
}

/** Apply the shared kind table; unknown refusals must have a recovery path. */
export function acceptOperatorRejection(reason: OperatorRejection | null, kind: string): OperatorRejection | null {
  return reason && operatorAcceptanceClears(reason.commandKind, kind) ? null : reason;
}

// Generated jog reasons 0..23, mirrored with the native/bridge reason tables.
const jogCodes = ["jog_none", "jog_closed", "jog_mode_conflict", "jog_not_ready", "jog_wrong_identity", "jog_invalid_generation", "jog_stale_sequence", "jog_invalid_vector", "jog_invalid_timing", "jog_expired", "jog_authority_lost", "jog_clock_reset", "jog_generation_exhausted", "jog_ramping", "jog_limited", "outside_limits_outward",
  "jog_path_start_mismatch", "jog_path_capacity", "jog_path_out_of_order", "jog_path_invalid_sample", "jog_path_ended", "jog_path_fault", "jog_path_unavailable", "jog_path_invalid_config"];

/** Fence public receipts by kind and source sequence; arrival time cannot
 * make an older refusal override accepted input. */
export function reduceOperatorRejection(s: ProcessStatus, events: readonly Event[], reason: OperatorRejection | null, after: number, fence = createOperatorFence()): {reason: OperatorRejection | null; after: number; stale?: OperatorRejection} {
  if (s.adapter_incarnation && s.daemon_incarnation) {
    const identity = `${s.adapter_incarnation}:${s.daemon_incarnation}`;
    if (fence.identity && fence.identity !== identity) { fence.kinds = {}; delete fence.stale; fence.after = 0; after = 0; }
    fence.identity = identity;
  }
  after = Math.max(after, fence.after);
  let command = fence.kinds.jog ?? { generation: 0, observed: 0, accepted: 0 };
  const previousGeneration = command.generation;
  if (s.jog && s.jog.JogGeneration >= command.generation) {
    if (s.jog.JogGeneration > command.generation) command = { generation: s.jog.JogGeneration, observed: 0, accepted: 0 };
    if (s.jog.SourceSequence) command.observed = Math.max(command.observed, s.jog.SourceSequence);
    if (s.jog.ObservedSourceSequence && (s.jog.UpdateReason === 0 || s.jog.UpdateReason === 14)) command.accepted = Math.max(command.accepted, s.jog.ObservedSourceSequence);
    fence.kinds.jog = command;
  }
  if (s.jog?.HasInput && previousGeneration && command.generation > previousGeneration && (s.jog.UpdateReason === 0 || s.jog.UpdateReason === 14)) reason = acceptOperatorRejection(reason, "jog");
  const jog = (code: number, mask: number) => operatorRejection(jogCodes[code] ?? `unknown_jog_reason_${code}`, mask, "jog");
  for (const e of events) {
    if (e.time_ns <= after) continue;
    if (e.events_dropped) { reason = operatorRejection("event_cursor_lost"); after = e.time_ns; }
    if (e.jog && (e.type === "jog_refused" || e.type === "jog_expired")) { reason = jog(e.jog.reason, e.jog.axis_mask); after = e.time_ns; }
  }
  // Ignored arrivals advance deduplication without hiding acceptance in this observation.
  let acceptanceAfter = after;
  const latest = s.jog_ingress?.latest_refusal;
  if (latest?.reason && latest.now_host_ns > after) {
    after = latest.now_host_ns;
    if (!latest.source_sequence || latest.source_sequence > command.accepted) {
      const receipt = jog(latest.reason, 0); receipt.sourceSequence = latest.source_sequence;
      if (!latest.source_sequence || latest.source_sequence !== command.observed) {
        receipt.unattributed = true; fence.stale = receipt;
      } else { reason = receipt; acceptanceAfter = latest.now_host_ns; }
    }
  }
  if (s.jog?.HasInput && s.jog.JogGeneration === command.generation && (s.jog.UpdateReason === 0 || s.jog.UpdateReason === 14) &&
      ((reason?.commandKind === "jog" && reason.sourceSequence > 0 && s.jog.ObservedSourceSequence >= reason.sourceSequence) ||
       ((!reason || !reason.sourceSequence) && s.jog.FirstObservedHostNs > acceptanceAfter))) {
    reason = acceptOperatorRejection(reason, "jog"); after = Math.max(after, s.jog.FirstObservedHostNs);
  }
  fence.after = after;
  fence.rejection = reason;
  return { reason, after, ...(fence.stale ? { stale: fence.stale } : {}) };
}

/** Project public observations only; missing limits or recovery stay unavailable
 * so incomplete feedback cannot claim readiness or reset permission. */
/** The drive's own alarm as rt-core decoded it from the A6-EC manual's table
 * (rt-core/docs/drives/a6ec/a6ec_manual_codes.json), e.g. "J5 drive alarm
 * Er41.1 (Motor over-temperature due to locked-rotor)". An alarm on an axis
 * native holds faulted is a drive alarm; one on a ready axis is a warning. An
 * undecoded 0x603F code on a faulted axis is still named, never hidden. */
export function driveAlarmText(id: string, axis: Pick<ProcessStatus["axes"][number], "drive_alarm" | "drive_alarm_text" | "readiness">, native?: { error_code?: number }): string {
  const faulted = axis.readiness === "faulted";
  const code = (axis.drive_alarm ?? "").trim();
  const words = (axis.drive_alarm_text ?? "").split(/\s+/).filter(Boolean).join(" ");
  if (code) return `${id} drive ${faulted ? "alarm" : "warning"} ${code}${words ? ` (${words})` : ""}`;
  const error = native?.error_code ?? 0;
  return faulted && error ? `${id} drive fault 0x${`0000${error.toString(16).toUpperCase()}`.slice(-4)}` : "";
}

/** A faulted drive's alarm in short, "J5 Er41.1" or "J5 0x7121"; "" when the
 *  drive is not in Fault or reports nothing. */
export function driveAlarmShort(id: string, axis: Pick<ProcessStatus["axes"][number], "drive_alarm" | "readiness">, native?: { error_code?: number }): string {
  if (axis.readiness !== "faulted") return "";
  const code = (axis.drive_alarm ?? "").trim();
  const error = native?.error_code ?? 0;
  return code ? `${id} ${code}` : error ? `${id} 0x${`0000${error.toString(16).toUpperCase()}`.slice(-4)}` : "";
}

export function deriveOperatorFeedback(s: ProcessStatus, d: Pick<Description, "axes">, rejection: OperatorRejection | null = null, stale?: OperatorRejection): OperatorFeedback {
  const r: OperatorReset = { admissible: false, text: "Reset unavailable: recovery observation unavailable", reason: "", sequence: 0, faults: [] };
  const recovery = s.core.recovery;
  if (recovery) {
    r.reason = recovery.reason ?? ""; r.sequence = recovery.reset_sequence ?? 0; r.faults = recovery.faults ?? [];
    r.admissible = s.time_ns > 0 && s.core.armed === 0 && s.core.commissioning_phase === 0 && s.core.native_home_active_axis_mask === 0 &&
      s.motion?.time_ns > 0 && (s.motion.done || (s.motion.mode !== 2 && s.motion.mode !== 3)) && s.execution.state !== "executing" && !!s.grant?.GrantActive;
    r.text = r.admissible ? "Reset fault available; cause must be cleared" : "Reset unavailable: inhibit motion and acquire control";
    if (r.reason && r.reason !== "none") r.text += ": " + operatorRejection(r.reason).text;
    for (const f of r.faults) r.text += `; ${f.name} (${f.recovery})`;
  }
  const referenceMask = r.faults.filter(f => f.name === "coordinate_reference").reduce((mask, f) => mask | f.axis_mask, 0);
  const referenceAxes = s.axes.flatMap((_, i) => referenceMask & (1 << i) ? [d.axes.find(axis => axis.index === i)?.id || `J${i + 1}`] : []);
  const separated = s.axes.every(a => typeof a.calibration_valid === "boolean");
  const referenceRecovered = referenceAxes.length > 0 && r.faults.filter(f => f.name === "coordinate_reference").every(f =>
    f.reset_preserves_home === true && f.rehome_axis_mask === 0 && (s.core.home_valid_mask & f.axis_mask) === f.axis_mask);
  const note = referenceRecovered ? `${referenceAxes.join(", ")}: saved Home has recovered. Use Clear fault while disarmed, then arm explicitly. No new Home is needed.`
    : referenceAxes.length && separated ? `${referenceAxes.join(", ")} lost position trust. Motion stopped; saved calibration is retained. Read-only recovery requires disarmed, stationary drives. Reset and Arm remain explicit.`
    : referenceAxes.length ? `${referenceAxes.join(", ")} position-reference evidence is not yet qualified. Keep the drives disarmed and inspect recovery status; do not establish a new Home just to clear this fault.` : undefined;
  // Polls may retain only the event code after the original request detail is
  // gone. The recovery observation still identifies this fault, including in
  // consumers that display/log rejection.text without calling the renderer.
  if (note && rejection?.code === "rt_core_event") rejection = { ...rejection, text: "The controller stopped motion because joint position evidence was rejected." };
  const axes = s.axes.map((a, i) => {
    const described = d.axes.find(axis => axis.index === i);
    const brakeWarning = brakeWarningText(described?.id || `J${i + 1}`, a.brake_hold_diagnostic, described);
    const supervisor = driveSupervisorText(described?.id || `J${i + 1}`, a.drive_supervisor);
    const alarmShort = driveAlarmShort(described?.id || `J${i + 1}`, a, s.core.axes?.[i]);
    const out: OperatorAxis = { index: i, rejection: rejection && (rejection.axisMask & (1 << i)) ? rejection : null,
      limit: null, homeLoss: "", reset: { ...r, faults: r.faults.filter(f => f.axis_mask === 0 || (f.axis_mask & (1 << i))) },
      alarmText: driveAlarmText(described?.id || `J${i + 1}`, a, s.core.axes?.[i]),
      ...(alarmShort ? { alarmShort } : {}),
      ...(brakeWarning ? { brakeWarning } : {}),
      ...(supervisor.armEnded ? { armEnded: supervisor.armEnded } : {}),
      ...(supervisor.warning ? { supervisorWarning: supervisor.warning } : {}) };
    if (typeof a.calibration_valid === "boolean") {
      const n = s.core.axes?.[i];
      if (!a.calibration_valid) out.homeLoss = `Calibration missing/mismatched${n?.anchor_identity_changed_field ? `: ${n.anchor_identity_changed_field}` : ""}; position check ${a.position_reason ?? "unknown"}`;
      else if (a.position_state !== "trusted" || !a.logical_valid || !a.home_valid)
        out.homeLoss = `Calibrated · ${a.position_state === "recovering" ? "recovering position" : "position recovery required"}; position check ${a.position_reason ?? "unknown"}; audit check ${a.audit_reason ?? "unknown"}`;
    } else if (!a.home_valid) {
      out.homeLoss = "Home required: never homed";
      const n = s.core.axes?.[i];
      if (n) {
        switch (n.anchor_reason ?? "") {
          case "anchor_ready": out.homeLoss = operatorRejection(n.anchor_reason).text; break;
          case "anchor_missing": out.homeLoss = "Home required: anchor missing"; break;
          case "anchor_identity_mismatch": out.homeLoss = n.anchor_identity_changed_field === "anchor.source"
            ? "Home required: the encoder reference source changed. The saved position reference cannot be reused; establish Home again."
            : "Home required: anchor identity changed: " + (n.anchor_identity_changed_field || "unknown field"); break;
          case "": case "none": if (s.core.home_epoch) out.homeLoss = "Home required: validity lost"; break;
          default: out.homeLoss = "Home required: restore refused: " + operatorRejection(n.anchor_reason ?? "").text;
        }
      }
      if (referenceMask & (1 << i)) out.homeLoss = `Position reference lost; Home is invalid${out.homeLoss === "Home required: never homed" || out.homeLoss === "Home required: validity lost" ? "" : `; ${out.homeLoss}`}`;
    }
    const limit = d.axes[i], range = limit ? limit.max_position - limit.min_position : NaN;
    if (limit?.index === i && a.logical_valid && Number.isFinite(a.logical_position) && Number.isFinite(range) && range > 0) {
      const lowerDistance = a.logical_position - limit.min_position, upperDistance = limit.max_position - a.logical_position;
      const lowerFraction = lowerDistance / range, upperFraction = upperDistance / range;
      const nearest = Math.min(lowerFraction, upperFraction);
      const l: OperatorLimit = { lowerDistance, upperDistance, lowerFraction, upperFraction, band: "clear", text: "Limits clear" };
      if (nearest <= approachFraction) { l.band = "approaching"; l.text = "Approaching soft limit"; }
      if (nearest <= nearFraction) { l.band = "near"; l.text = "Near soft limit"; }
      if (nearest <= 0) { l.band = "at_limit"; l.text = "At soft limit"; }
      // Generated native jog reason 14 is Limited; it is observation, not rejection.
      if (s.jog?.Open && (s.jog.AxisMask & (1 << i)) && s.jog.StateReason === 14 &&
          (nearest <= 0 || a.logical_target <= limit.min_position || a.logical_target >= limit.max_position)) {
        l.band = "clamped"; l.text = "At limit, clamped";
      }
      out.limit = l;
    }
    return out;
  });
  const warnings = axes.flatMap(axis => [axis.brakeWarning, axis.supervisorWarning].filter((w): w is string => !!w));
  // An Arm-ending reason is history once the controller is armed again.
  const armEnded = s.core.armed ? [] : axes.flatMap(axis => axis.armEnded ? [axis.armEnded] : []);
  return { ...(stale ? { stale } : {}), ...(note ? { note } : {}), ...(warnings.length ? { warnings } : {}),
    ...(armEnded.length ? { armEnded } : {}), rejection, reset: r, axes };
}

// The browser preview's local cartesiand reports these as the jog session's
// reason while nothing is wrong: `leader_transition` is its leader broker
// state before the first hold, `rt_core_stop_fence` the fence after a hold
// ends normally. Rendered as "Rejected (group): Unknown rt-core refusal"
// they read as a fault on a pad that is merely idle. UI-side only: the
// bridge and pendant reason tables (checked by the parity fixtures) do not
// carry them, and `operatorReasonLabels` must stay equal to those tables.
export const previewJogIdleLines: Record<string, string> = {
  leader_transition: "Preview jog idle. Hold a button to move the browser preview.",
  rt_core_stop_fence: "Preview hold ended.",
};

/** The browser preview's feedback: an idle-state reason becomes a neutral
 * note and is never shown as a rejection. Other feedback passes unchanged;
 * a real machine's strip does not go through here. */
export function previewOperatorFeedback(feedback: OperatorFeedback | null): OperatorFeedback | null {
  const code = feedback?.rejection?.code ?? "";
  const note = previewJogIdleLines[code];
  if (!feedback || !note) return feedback;
  return { ...feedback, rejection: null, note, axes: feedback.axes.map(axis => axis.rejection?.code === code ? { ...axis, rejection: null } : axis) };
}

/**
 * What the browser preview's strip says. The per-axis home, limit and reset
 * dump describes a real cell's fence; on the preview it listed nine simulated
 * axes at 50 % of range under "Reset unavailable" on every program load
 * (owner 2026-09-21: why is this here?). The preview says nothing while idle
 * or after a hold ends (the pad's own caption covers that) and one line for a
 * refusal the preview itself made, such as a soft limit.
 */
export function previewFeedbackLine(feedback: OperatorFeedback | null): string {
  const rejection = feedback?.rejection;
  if (!rejection || rejection.code in previewJogIdleLines) return "";
  return `${rejection.text} [${rejection.code}]`;
}

/** Use text nodes for public reasons, preventing diagnostic text becoming markup. */
export function renderOperatorFeedback(host: HTMLElement, feedback: OperatorFeedback | null, requestReason = ""): void {
  host.replaceChildren();
  if (!feedback) { host.textContent = "Operator feedback unavailable"; return; }
  const processAdmission = processIOAdmissionMessage(requestReason) ?? processIOAdmissionMessage(feedback.rejection?.code ?? "");
  const controllerFault = controllerFaultEventMessage(requestReason) ?? controllerFaultEventMessage(feedback.rejection?.code ?? "");
  // A plan requiring unsupported process outputs does not call for fault Reset.
  // Retain actual recovery faults, lost Home, alarms and limit warnings; omit
  // only the unrelated healthy-axis and Reset-availability inventory.
  const resetRelevant = feedback.reset.faults.length > 0 || !!feedback.reset.reason && feedback.reset.reason !== "none";
  const lines = !processAdmission || resetRelevant ? [feedback.reset.text] : [];
  if (feedback.stale) lines.unshift(`Stale (unattributed): ${feedback.stale.text} [${feedback.stale.code}], sequence ${feedback.stale.sourceSequence}`);
  if (feedback.rejection && !(controllerFault && feedback.rejection.code.split(":", 1)[0]!.trim() === "rt_core_event") && (!processAdmission || !processIOAdmissionMessage(feedback.rejection.code))) lines.unshift(`Rejected (${feedback.rejection.axisMask ? "axes mask " + feedback.rejection.axisMask : "group"}): ${feedback.rejection.text} [${feedback.rejection.code}]`);
  if (controllerFault) lines.unshift(`${controllerFault} [${requestReason || feedback.rejection!.code}]`);
  if (processAdmission) lines.unshift(`Load refused: ${processAdmission} [process_io_executor_not_qualified]`);
  if (feedback.note) lines.unshift(feedback.note);
  // The reason the last Arm attempt ended leads: it explains why Arm dropped.
  for (const reason of feedback.armEnded ?? []) {
    const row = document.createElement("div");
    row.className = "operator-arm-ended";
    row.textContent = `Arm ended: ${reason}`;
    host.append(row);
  }
  // Warnings are rendered apart from refusals and recovery: they gate nothing.
  for (const warning of feedback.warnings ?? []) {
    const row = document.createElement("div");
    row.className = warning.startsWith(brakeSlipPrefix) ? "operator-warning operator-warning-slip" : "operator-warning";
    row.textContent = `Warning: ${warning}`;
    host.append(row);
  }
  for (const a of feedback.axes) {
    const fields = processAdmission
      ? [a.homeLoss, a.limit && a.limit.band !== "clear" ? a.limit.text : "", a.alarmText]
      : [a.homeLoss || "Home valid", a.limit ? `${a.limit.text}; lower ${(a.limit.lowerFraction * 100).toFixed(1)}%, upper ${(a.limit.upperFraction * 100).toFixed(1)}% of range` : "Limits unavailable", a.alarmText];
    const text = fields.filter(Boolean).join("; ");
    if (text) lines.push(`Axis ${a.index + 1}: ${text}`);
  }
  for (const line of lines) { const row = document.createElement("div"); row.textContent = line; host.append(row); }
}

// Mirrored from the bridge/pendant reasons table; fixture tests check wording.
export const operatorReasonLabels: Record<string, string> = {
  "anchor_store_io": "Anchor storage unavailable",
  "anchor_ready": "Home restored",
  "soft_limit": "Soft limit; correct the trajectory",
  "no_grant": "No grant; acquire control again",
  "wrong_generation": "Stale generation; release and begin a new jog",
  "inhibited": "Inhibited; clear fault or Stop and re-arm",
  "invalid_telemetry_cursor": "Invalid telemetry cursor",
  "telemetry_cursor_ahead": "Telemetry cursor ahead",
  "telemetry_busy": "Telemetry busy",
  "telemetry_unavailable": "Telemetry unavailable",
  "telemetry_label_invalid": "Telemetry label invalid",
  "axis_count_mismatch": "Axis count mismatch",
  "blob_length_mismatch": "Blob length mismatch",
  "block_layout_invalid": "Block layout invalid",
  "block_sha256_mismatch": "Block sha256 mismatch",
  "dense_schema_mismatch": "Dense schema mismatch",
  "duration_mismatch": "Duration mismatch",
  "header_json_invalid": "Header json invalid",
  "header_truncated": "Header truncated",
  "kind_invalid": "Kind invalid",
  "limits_invalid": "Limits invalid",
  "nonfinite_sample": "Nonfinite sample",
  "q_step_exceeded": "Q step exceeded",
  "qd_limit_exceeded": "Qd limit exceeded",
  "reserved_flags_set": "Reserved flags set",
  "sample_count_overflow": "Sample count overflow",
  "sample_encoding_mismatch": "Sample encoding mismatch",
  "segment_index_out_of_order": "Segment index out of order",
  "segment_too_short": "Segment too short",
  "segments_empty": "Segments empty",
  "time_grid_invalid": "Time grid invalid",
  "torch_outside_weld": "Torch outside weld",
  "total_sample_count_mismatch": "Total sample count mismatch",
  "trajectory_digest_mismatch": "Trajectory digest mismatch",
  "trajectory_identity_mismatch": "Trajectory identity mismatch",
  "event_cursor_lost": "Event cursor lost",
  "application_generation_exhausted": "Application generation exhausted",
  "authority_binding_mismatch": "Authority binding mismatch",
  "body_too_large": "Body too large",
  "bound path is not a socket": "Bound path is not a socket",
  "boundary_q_discontinuity": "Boundary q discontinuity",
  "boundary_qd_nonzero": "Boundary qd nonzero",
  "busy": "Busy",
  "capability_unimplemented": "Capability unimplemented",
  "continuous_weld_discontinuity": "Continuous weld boundary mismatch",
  "control payload too large": "Control payload too large",
  "control_already_owned": "Control already owned",
  "control_session_stale": "Control session stale",
  "invalid_requested_lease": "Invalid requested lease; check link profile",
  "session_principal_mismatch": "Session principal mismatch",
  "daemon_restarted": "Daemon restarted",
  "dense_axis_map_requires_nine_ids": "Dense axis map requires nine ids",
  "duplicate_request_field": "Duplicate request field",
  "expired": "Expired",
  "fault_persists": "Fault persists",
  "pdo_mapping_mismatch": "Drive PDO mapping does not match configuration",
  "fence": "Fence",
  "fragmentation rejected": "Fragmentation rejected",
  "handle_active": "Handle active",
  "handle_consumed": "Handle consumed",
  "handle_discarded": "Handle discarded",
  "handle_retired": "Retired handle; prepare the motion again",
  "handle_started": "Handle started",
  "handle_superseded": "Handle superseded",
  "hijacking unavailable": "Hijacking unavailable",
  "independent_jog_unavailable": "Independent jog unavailable",
  "invalid UTF-8": "Invalid utf-8",
  "invalid close code": "Invalid close code",
  "invalid close payload": "Invalid close payload",
  "invalid close reason": "Invalid close reason",
  "invalid frame flags": "Invalid frame flags",
  "invalid opcode": "Invalid opcode",
  "invalid remote CRL PEM": "Invalid remote crl pem",
  "invalid remote pair SAN": "Invalid remote pair san",
  "invalid remote principal SAN": "Invalid remote principal san",
  "invalid websocket upgrade": "Invalid websocket upgrade",
  "invalid_control_generation": "Invalid control generation",
  "invalid_dense_axis_map": "Invalid dense axis map",
  "invalid_pair_configuration_binding": "Invalid pair configuration binding",
  "invalid_request_envelope": "Invalid request envelope",
  "invalid_request_id": "Invalid request id",
  "jog_clock_arithmetic_range": "Jog clock arithmetic range",
  "jog_clock_calibration_expired": "Jog clock calibration expired",
  "jog_clock_exchange_inconsistent": "Jog clock exchange inconsistent",
  "jog_clock_incarnation_mismatch": "Jog clock incarnation mismatch",
  "jog_clock_invalid_budget_or_exchange": "Jog clock invalid budget or exchange",
  "jog_clock_mapping_generation_mismatch": "Jog clock mapping generation mismatch",
  "jog_clock_moved_backwards": "Jog clock moved backwards",
  "jog_clock_uncertainty_exceeded": "Jog clock uncertainty exceeded",
  "jog_clock_unqualified": "Jog clock unqualified",
  "jog_input_deadline_expired": "Jog input has expired",
  "jog_input_entirely_future": "Jog input entirely future",
  "jog_input_too_old": "Jog input too old",
  "jog_invalid_frame": "Jog invalid frame",
  "jog_publisher_busy": "Jog publisher busy",
  "jog_session_or_sequence_stale": "Jog session or sequence stale",
  "jog_session_stale": "Jog session stale",
  "jog_stream_idle": "Jog stream idle",
  "manifest_revision_mismatch": "Manifest revision mismatch",
  "mode_conflict": "Mode conflict",
  "native_execution_failed": "Native execution failed; check status and events, reset_fault if faulted, then prepare and start again",
  "native_limit_exceeded": "Native limit exceeded",
  "outside_limits_outward": "Outside limits; jog inward (opposite way)",
  "native_rejected": "Native motion request was refused",
  "native_segment_rate_exceeded": "Native segment rate exceeded",
  "noncanonical length": "Noncanonical length",
  "not_ready": "Not ready",
  "payload too large": "Payload too large",
  "process start time unavailable": "Process start time unavailable",
  "io_not_configured": "I/O is not configured",
  "io_not_armed": "I/O is not armed",
  "io_fast_input_unsatisfied": "Required I/O input missing",
  "io_readback_disagreement": "I/O readback disagreement",
  "io_torch_unqualified": "Torch I/O is not qualified",
  "io_marker_late": "I/O marker arrived late",
  "io_exchange_lost": "I/O exchange lost",
  "io_wait_timeout": "Sensor wait timed out; check the program input",
  "io_wait_input_invalid": "Sensor wait stopped: input unavailable",
  "io_marker_invalid": "Invalid I/O marker",
  "process_io_executor_not_qualified": "Process io executor not qualified",
  "program_already_executing": "Program already executing",
  "program_identity_mismatch": "Program identity mismatch",
  "program_identity_missing": "Program identity missing",
  "recovery_observation_unavailable": "Recovery observation unavailable",
  "remote CA must be a self-signed component CA": "Remote ca must be a self-signed component ca",
  "remote CA must contain one CA certificate": "Remote ca must contain one ca certificate",
  "remote CA, certificate, key and CRL are required": "Remote ca, certificate, key and crl are required",
  "remote CRL not current": "Remote crl not current",
  "remote client must be issued directly by component CA": "Remote client must be issued directly by component ca",
  "remote client revoked": "Remote client revoked",
  "remote identity SAN must be an opaque token": "Remote identity san must be an opaque token",
  "remote pair-id required": "Remote pair-id required",
  "remote principal/pair binding mismatch": "Remote principal/pair binding mismatch",
  "request_envelope_changed": "Request envelope changed",
  "request_id_conflict": "Request id conflict",
  "reset_requires_inhibited": "Reset requires inhibited",
  "resource_unknown": "Requested resource unavailable",
  "resource_artifact_invalid": "Robot resource artifact invalid",
  "schema_mismatch": "Schema mismatch",
  "segment_boundary_discontinuous": "Segment boundary discontinuous",
  "socket lock ownership cannot be verified": "Socket lock ownership cannot be verified",
  "socket lock path changed": "Socket lock path changed",
  "socket owned by a live process": "Socket owned by a live process",
  "socket ownership cannot be verified": "Socket ownership cannot be verified",
  "trailing_request_data": "Trailing request data",
  "unknown_endpoint": "Unknown endpoint",
  "unknown_handle": "Unknown handle",
  "unknown_operation": "Unknown operation",
  "unmapped_dense_axis": "Unmapped dense axis",
  "unsupported critical CRL extension": "Unsupported critical crl extension",
  "unsupported socket ownership transport": "Unsupported socket ownership transport",
  "anchor_missing": "Anchor missing",
  "anchor_identity_mismatch": "Anchor identity mismatch",
  "anchor_source_invalid": "Anchor source invalid",
  "anchor_disagrees": "Anchor disagrees",
  "invalid_axis_mask": "Invalid axis mask",
  "event_cursor_ahead": "Event cursor ahead",
  "invalid_event_cursor": "Invalid event cursor",
  "path_jog_unavailable": "Path jog unavailable; cell needs jog_jerk",
  "path_queue_full": "Path queue full; resend after acks",
  "path_chunk_stale": "Path chunk stale; begin a new path jog",
  "jog_none": "Jog none",
  "jog_closed": "Jog closed",
  "jog_mode_conflict": "Jog mode conflict",
  "jog_not_ready": "Jog not ready",
  "jog_wrong_identity": "Jog wrong identity",
  "jog_invalid_generation": "Jog invalid generation",
  "jog_stale_sequence": "Jog stale sequence",
  "jog_invalid_vector": "Jog invalid vector",
  "jog_invalid_timing": "Jog invalid timing",
  "jog_expired": "Jog expired",
  "jog_authority_lost": "Jog authority lost",
  "jog_clock_reset": "Jog clock reset",
  "jog_generation_exhausted": "Jog generation exhausted",
  "jog_ramping": "Jog ramping",
  "jog_limited": "Jog limited",
  "jog_path_start_mismatch": "Path start is off the held target",
  "jog_path_capacity": "Path buffer full; retry",
  "jog_path_out_of_order": "Path chunk out of order",
  "jog_path_invalid_sample": "Path sample invalid",
  "jog_path_ended": "Path jog already ended",
  "jog_path_fault": "Path jog fault; clear fault and re-arm",
  "jog_path_unavailable": "Path jog unavailable; cell needs jog_jerk",
  "jog_path_invalid_config": "Path jog configuration invalid",
  "none": "None",
  "invalid_command": "Invalid command",
  "capacity": "Capacity",
  "invalid_trajectory": "Invalid trajectory"
};
