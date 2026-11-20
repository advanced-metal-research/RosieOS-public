import type { Description, Event, PlanCursor, ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import * as reasons from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { acceptOperatorRejection, createOperatorFence, deriveOperatorFeedback, operatorRejection, processIOAdmissionMessage, reduceOperatorRejection, type OperatorFence } from "./operatorFeedback.ts";

/** Preserve the adapter code alongside operator wording. Unknown reasons stay
 * visible refusals; a missing label must never turn a failed jog into success. */
export function rtCoreReasonText(code: string, detail = ""): string {
  const processAdmission = processIOAdmissionMessage(code);
  if (processAdmission) return `${code}: ${processAdmission}`;
  // Unverified operator labels shared by public status and selected-target jog.
  const labels: Record<string, string> = {
    cartesian_resolver_unavailable: "Cartesian resolver unavailable; release the pad and check the local Motion Server installation",
    cartesian_resolver_invalid: "Invalid Cartesian resolver response; motion stopped",
    cartesian_axes_invalid: "Cartesian jog requires described J1–J6 axes in radians with valid limits",
    cartesian_pose_stale: "Live joint pose is stale; motion stopped",
    cartesian_input_invalid: "Invalid Cartesian frame, twist or speed; release the pad",
    ik_no_solution: "No joint motion solves this Cartesian direction at the current pose",
    joint_limit: "Cartesian motion would cross a joint limit; release and choose another direction",
    jacobian_gate: "Cartesian motion stopped near a singular pose; reposition with joint jog",
    cartesian_hold_unavailable: "This cell needs rt-core path jog for Cartesian holds; use Step mode",
    cartesian_resolver_too_slow: "Cartesian resolver too slow for a path hold at this speed; lower the speed or use Step mode",
    cartesian_reach: "Cartesian line leaves the reachable workspace; stopped on the line",
    ik_discontinuity: "Cartesian line reaches a joint flip or near-singular jump; stopped on the line",
    path_buffer_underrun: "Cartesian path geometry arrived late; stopped on the line",
    path_fault: "Cartesian path lane fault; motion stopped",
    no_grant: "No grant; acquire control again",
    // The OLP server's 503 while a maintenance notice is active (server maintenance.go).
    maintenance: "Update in progress; robot actions are refused until it ends (Stop, Disarm and Disconnect still work)",
    jog_not_ready: "Jog not ready; clear fault or Stop and re-arm",
    jog_mode_conflict: "Jog mode conflict; release the current hold first",
    jog_stale_sequence: "Jog stale sequence; release and begin a new jog",
    jog_session_stale: "Jog session stale; release and begin a new jog",
    jog_input_deadline_expired: "Jog input has expired; release and begin a new jog",
    jog_input_too_old: "Jog input too old; release and begin a new jog",
    jog_limited: "Jog limited; release input and inspect limits",
    outside_limits_outward: "Outside limits; jog inward (opposite way)",
    joint_move_invalid: "Enter a nonzero signed angle and a speed fraction above 0 and at most 50%",
    joint_move_limits: "Angle exceeds the described joint range or soft position limits",
    joint_move_units: "Move by angle requires a rotary axis described in radians",
    joint_move_description_invalid: "Described motion limits are missing or invalid",
    joint_move_too_many_points: "Move exceeds the described trajectory capacity",
    joint_move_interrupted: "Joint move interrupted; inspect status before another GO",
    joint_move_prepare_timeout: "Trajectory upload did not complete within the link budget; the machine stays armed, press again",
    joint_move_prepare_busy: "The cell is still decoding a previous upload; the machine stays armed, press again",
    joint_move_feedback_invalid: "Joint position is unavailable; inspect target feedback",
    operator_stop: "Stopped by operator",
  };
  const known = Object.entries(reasons).some(([name, value]) => name.startsWith("Reason") && value === code);
  // A joint_move_limits detail names the joint and its described range in
  // degrees; without it the generic label still refuses rather than looking
  // like success.
  const label = (code === "joint_move_limits" || code === "native_limit_exceeded" || code === "native_segment_rate_exceeded") && detail || labels[code] || (known ? code.replaceAll("_", " ").replace(/^./, c => c.toUpperCase()) : detail || "Release input; inspect public status before retrying");
  return `${code}: ${label}`;
}

/** Operator text for a joint_move_limits refusal that names the joint and the
 *  described range in degrees. minRad/maxRad are the axis description's
 *  min_position/max_position (rad); the operator reads degrees. */
export function jointMoveLimitsText(axisId: string, minRad: number, maxRad: number): string {
  return `Angle exceeds ${axisId}'s described range (${(minRad * 180 / Math.PI).toFixed(1)}° to ${(maxRad * 180 / Math.PI).toFixed(1)}°)`;
}

/** A stop this browser sent because the page lost focus or the pointer was
 *  released, not because the operator pressed STOP. The server's
 *  operator_stop label stays for that button. */
export function jointJogBrowserStopText(cause: string): string | undefined {
  switch (cause) {
    case "page_blur":
    case "pagehide":
      return "Stopped: the page lost focus while the move ran";
    case "pointer_release":
      return "Stopped: the pointer was released while the move ran";
    default:
      return undefined;
  }
}

/** A native clock is host uptime, not trajectory elapsed time. An inactive
 * handle/UINT32_MAX cursor does not identify an executing sample at all. */
export function activeRTCoreCursor(cursor: PlanCursor | null | undefined): PlanCursor | null {
  if (!cursor || !Number.isInteger(cursor.active_handle) || cursor.active_handle <= 0 ||
      !Number.isInteger(cursor.sample_index) || cursor.sample_index < 0 || cursor.sample_index >= 0xffffffff) return null;
  return cursor;
}

/** Explain the observed fault without replacing the raw reason or recovery
 * gates. Bit 0 is the native execution policy's start_discontinuity flag. */
export function rtCoreExecutionDiagnostic(status: ProcessStatus): string | null {
  const startMismatch = status.core.recovery?.faults?.some(fault => fault.name === "start_discontinuity") ||
    (Number.isInteger(status.core.execution_fault_reasons) && (status.core.execution_fault_reasons & 1) !== 0);
  return startMismatch
    ? "The robot is not at this plan’s starting position, so motion was refused. Arm the robot to recover, then use Go to plan start, Load to robot and Play. Home does not move the robot to the plan start."
    : null;
}

/** Map observed logical state only. Invalid coordinates stay unavailable, and
 * faults and history loss disable readiness rather than preserving old readiness. */
export function mapRTCoreStatus(status: ProcessStatus, events: readonly Event[] = [], reason = "", description: Pick<Description, "axes"> = { axes: [] }, refusalCode = reason.split(":", 1)[0] ?? "", afterNS = refusalCode ? status.time_ns : 0, refusalKind = "unknown", fence: OperatorFence = createOperatorFence()) {
  if (!status || !Array.isArray(status.axes) || !status.core || !status.execution ||
      !(status.time_ns > 0) || !status.adapter_incarnation || !status.daemon_incarnation) {
    throw new Error("rt_core status unavailable");
  }
  const eventFailure = events.find(e => e.type === "events_dropped" || e.fault || e.type === "grant_expired" || e.type === "grant_released");
  const nativeFault = status.core.safety_fault_mask ? `rt_core_fault: ${status.core.safety_fault_mask}` : "";
  // Stop/Disarm history cannot hide a fault observed in the current native
  // publication. Keep explicit transport/refusal reasons and feedback receipts.
  const retainedStop = reason === "operator_stop" || reason === "disarm";
  const failure = (nativeFault && (!reason || retainedStop) ? nativeFault : "") || reason || status.execution.error || nativeFault ||
    (eventFailure ? `rt_core_event: ${eventFailure.type}` : "");
  if (fence.identity && fence.identity !== `${status.adapter_incarnation}:${status.daemon_incarnation}`) {
    fence.rejection = null; fence.receiptAfter = 0;
  }
  // Keep the active result when deduplication consumes its receipt; only a new
  // matching acceptance or explicit refusal may replace that display state.
  let rejection = refusalCode ? operatorRejection(refusalCode, 0, refusalKind) : fence.rejection ?? null;
  if (!refusalCode && afterNS > (fence.receiptAfter ?? 0)) rejection = acceptOperatorRejection(rejection, refusalKind);
  fence.receiptAfter = Math.max(fence.receiptAfter ?? 0, afterNS);
  const receipt = reduceOperatorRejection(status, events, rejection, afterNS, fence);
  const executionDiagnostic = rtCoreExecutionDiagnostic(status);
  const operatorFeedback = deriveOperatorFeedback(status, description, receipt.reason, receipt.stale);
  if (executionDiagnostic) operatorFeedback.note = executionDiagnostic;
  return {
    operatorFeedback,
    executionDiagnostic,
    armed: !failure && status.core.armed === 1,
    // The drives' own word, before any reason is folded in. `armed` above
    // is the drivable compound; this is the observation the Machine panel
    // renders as ARMED or DISARMED, so a refusal cannot turn a known state
    // into "unknown".
    coreArmed: status.core.armed === 1,
    // Observe the native grant even before this client has an OLP session.
    // Missing or invalid values cannot be presented as a released lease.
    coreLeaseValid: status.core.lease_valid === 1 ? true : status.core.lease_valid === 0 ? false : null,
    coreEnableMask: status.core.axis_enable_mask,
    coreFaultMask: status.core.safety_fault_mask,
    homeValidMask: status.core.home_valid_mask,
    /** Daemon host time of this observation (monotonic on the cell); the
     *  telemetry strip's time base. */
    timeNs: status.time_ns,
    observationIdentity: `${status.adapter_incarnation}:${status.daemon_incarnation}`,
    // Position numbers are meaningful only in this coordinate generation.
    // A Home/configuration change must invalidate cached plan-start proof.
    coordinateIdentity: typeof status.core.configuration_sha256 === "string" && status.core.configuration_sha256.length > 0 &&
      Number.isSafeInteger(status.core.configuration_epoch) && status.core.configuration_epoch > 0 &&
      Number.isSafeInteger(status.core.home_epoch) && status.core.home_epoch > 0
      ? `${status.core.configuration_sha256}:${status.core.configuration_epoch}:${status.core.home_epoch}` : null,
    ready: !failure && status.axes.length > 0 && status.axes.every(axis => axis.home_valid && axis.logical_valid && axis.readiness === "ready"),
    reason: failure || null,
    execution: status.execution,
    cursor: activeRTCoreCursor(status.plan_cursor ?? status.core.plan_cursor),
    axes: status.axes.map((axis, index) => ({
      position: axis.logical_valid && Number.isFinite(axis.logical_position) ? axis.logical_position : null,
      target: axis.logical_valid && Number.isFinite(axis.logical_target) ? axis.logical_target : null,
      velocity: axis.logical_valid ? axis.logical_velocity ?? null : null,
      homeValid: axis.home_valid,
      calibrationValid: axis.calibration_valid,
      positionState: axis.position_state,
      positionReason: axis.position_reason,
      auditReason: axis.audit_reason,
      auditStatus: axis.audit_status,
      auditLastVerifiedNs: axis.audit_last_verified_ns,
      homeActive: ((status.core.native_home_active_axis_mask ?? 0) & (1 << index)) !== 0,
      coordinateValid: axis.logical_valid === true,
      readiness: axis.readiness,
      statusword: axis.statusword,
      positionCounts: axis.position_counts,
      // The drive's own measurements, raw: counts/s, counts and per-mille of
      // rated torque. The telemetry strip converts them with the description.
      velocityActualCountsPerS: axis.velocity_actual_counts_per_s,
      velocityActualValid: axis.velocity_actual_valid === true,
      followingErrorCounts: axis.following_error_counts,
      followingErrorValid: axis.following_error_valid === true,
      torqueRaw: axis.torque_raw,
    })),
  };
}
