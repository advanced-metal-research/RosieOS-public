import assert from "node:assert/strict";
import test from "node:test";
import { brakeSlipPrefix, brakeWarningText, createOperatorFence, deriveOperatorFeedback, driveAlarmText, operatorReasonLabels, operatorRejection, previewFeedbackLine, previewJogIdleLines, previewOperatorFeedback, renderOperatorFeedback } from "./operatorFeedback.ts";
import { parseJogState } from "../simulation/live-jog.ts";
import type { ProcessStatus } from "../../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import { isMachineFault, machineReasonText } from "../execution/machineFlow.ts";
import { mapRTCoreStatus } from "./rtCoreStatus.ts";

class Host {
  rows: string[] = [];
  textContent = "";
  replaceChildren(): void { this.rows = []; this.textContent = ""; }
  append(row: { textContent: string }): void { this.rows.push(row.textContent); }
}

const status = (): ProcessStatus => ({
  time_ns: 1000, adapter_incarnation: "adapter", daemon_incarnation: "daemon",
  motion: { time_ns: 1000, mode: 0, done: true }, grant: { GrantActive: 1 }, execution: { state: "idle", error: "" },
  core: { armed: 0, commissioning_phase: 0, native_home_active_axis_mask: 0, safety_fault_mask: 0, axes: [], recovery: { reason: "none", reset_sequence: 0, faults: [] } },
  axes: [{ home_valid: true, logical_valid: true, readiness: "ready", logical_position: 0, logical_target: 0, statusword: 0, position_counts: 0 }],
} as unknown as ProcessStatus);
const description = { axes: [{ index: 0, id: "J1", position_unit: "rad", min_position: -5, max_position: 5 }] } as never;

test("native grant generation mismatch is distinct from invalid Home and other grant reasons", () => {
  const reason = "RTCore rejected application grant request 4 (reason 3)";
  assert.match(operatorRejection(reason).text, /out-of-date controller state/);
  assert.match(machineReasonText(`${reason}: ${reason}`), /Reconnect to the machine/);
  assert.doesNotMatch(operatorRejection(reason).text, /joint limit|encoder source|Unknown/);
  assert.equal(operatorRejection("RTCore rejected application grant request 4 (reason 5)").text, "Unknown rt-core refusal");
});

test("changed encoder source explains why the saved Home reference cannot be reused", () => {
  assert.match(operatorRejection("home_required").text, /Home is required before loading/);
  assert.match(machineReasonText("home_required: establish reference"), /Review the Home step/);
  const s = status(); s.axes[0]!.home_valid = false;
  s.core.axes = [{ anchor_reason: "anchor_identity_mismatch", anchor_identity_changed_field: "anchor.source" }] as typeof s.core.axes;
  const feedback = deriveOperatorFeedback(s, description);
  assert.match(feedback.axes[0]!.homeLoss, /encoder reference source changed.*saved position reference cannot be reused; establish Home again/);
  assert.equal(feedback.reset.admissible, true, "wording does not change recovery authorization");
});

// The owner saw "Rejected (group): Unknown rt-core refusal [leader_transition]"
// under an idle preview pad, and "rt_core_stop_fence" after a normal release.
// Both are the local cartesiand's state, not refusals.
test("the browser preview's idle reasons render as a state line, never as a rejection", (t) => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  for (const [code, line] of Object.entries(previewJogIdleLines)) {
    const jog = parseJogState({ running: true, rtCore: { status: status(), description, reason: code } }, createOperatorFence());
    assert.equal(jog?.operatorFeedback?.rejection?.code, code, `${code} arrives as the session's rejection`);
    const host = new Host();
    renderOperatorFeedback(host as unknown as HTMLElement, previewOperatorFeedback(jog?.operatorFeedback ?? null));
    assert.equal(host.rows[0], line, code);
    assert.doesNotMatch(host.rows.join("\n"), /Rejected|Unknown rt-core refusal|leader_transition|rt_core_stop_fence/, code);
    assert.match(host.rows.join("\n"), /Axis 1: Home valid/, `${code}: the rest of the strip stays`);
  }
  assert.equal(previewJogIdleLines.leader_transition, "Preview jog idle. Hold a button to move the browser preview.");
  assert.equal(previewJogIdleLines.rt_core_stop_fence, "Preview hold ended.");
});

test("preview mapping leaves real refusals, empty feedback and the shared label table alone", () => {
  const refused = deriveOperatorFeedback(status(), description, operatorRejection("soft_limit", 1, "jog", 3));
  assert.equal(previewOperatorFeedback(refused), refused);
  assert.equal(previewOperatorFeedback(null), null);
  const clean = deriveOperatorFeedback(status(), description);
  assert.equal(previewOperatorFeedback(clean), clean);
  assert.equal(clean.note, undefined);
  // The Go and C++ parity tests compare this table verbatim; the preview's
  // idle lines live beside it, not in it.
  for (const code of Object.keys(previewJogIdleLines)) assert.equal(code in operatorReasonLabels, false, code);
});

// The owner opened a new program and the Jog heading carried nine lines of
// "Axis n: Home valid; Limits clear; lower 50.0%, upper 50.0%" under "Reset
// unavailable": the simulator's fence, not anything about the browser preview.
test("the browser preview's strip is empty while idle and one line for a refusal it made", () => {
  for (const code of Object.keys(previewJogIdleLines)) {
    const idle = deriveOperatorFeedback(status(), description, operatorRejection(code, 0, "jog", 1));
    assert.equal(previewFeedbackLine(idle), "", code);
  }
  assert.equal(previewFeedbackLine(deriveOperatorFeedback(status(), description)), "");
  assert.equal(previewFeedbackLine(null), "");
  const refused = deriveOperatorFeedback(status(), description, operatorRejection("soft_limit", 1, "jog", 3));
  assert.equal(previewFeedbackLine(refused), "Soft limit; correct the trajectory [soft_limit]");
  assert.doesNotMatch(previewFeedbackLine(refused), /Axis 1|Reset/);
});

test("process-output admission explains the plan requirement without Reset or healthy-axis inventory", t => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const code = "process_io_executor_not_qualified";
  const s = status(); s.core.armed = 1;
  const feedback = deriveOperatorFeedback(s, description, operatorRejection(code));
  const host = new Host(); renderOperatorFeedback(host as unknown as HTMLElement, feedback);
  assert.equal(host.rows.length, 1);
  assert.match(host.rows[0]!, /^Load refused:.*plan requests welding or process outputs.*machine is not configured to execute/);
  assert.doesNotMatch(host.rows.join("\n"), /Reset|Home|Axis 1|50\.0%/);
  assert.match(host.rows[0]!, /Select Dry run, then load again.*no replanning is needed/);
  assert.doesNotMatch(host.rows[0]!, /Plan a dry run/);
  assert.equal(isMachineFault(code), false);
  assert.match(machineReasonText(`${code}: Process io executor not qualified`), /plan requests welding or process outputs/);
  assert.equal(operatorReasonLabels[code], "Process io executor not qualified", "native/pendant parity table remains unchanged");
  assert.equal(feedback.axes.length, 1, "rendering does not discard machine observations");
});

test("a process-output refusal retains independent faults, lost Home, alarm and limit observations", t => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const s = status(); s.axes[0]!.home_valid = false; s.axes[0]!.logical_position = 5;
  s.core.recovery!.reason = "fault_persists";
  s.core.recovery!.faults = [{ bit: 4, name: "drive_fault", axis_mask: 1, recovery: "reset", outcome: "", rehome_axis_mask: 0 }];
  const feedback = deriveOperatorFeedback(s, description, operatorRejection("soft_limit", 1, "jog", 3));
  feedback.axes[0]!.alarmText = "Drive alarm: following error";
  const host = new Host(); renderOperatorFeedback(host as unknown as HTMLElement, feedback, "process_io_executor_not_qualified: request rejected");
  const text = host.rows.join("\n");
  assert.match(text, /Load refused:.*welding or process outputs/);
  assert.match(text, /Soft limit; correct the trajectory \[soft_limit\]/);
  assert.match(text, /Reset fault available.*drive_fault/);
  assert.match(text, /Home required: never homed; At soft limit; Drive alarm: following error/);
  assert.doesNotMatch(text, /of range/);
});

test("a latched coordinate fault explains the affected joint without changing recovery permissions", t => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const s = status();
  s.axes = Array.from({ length: 6 }, () => ({ ...s.axes[0]! }));
  s.axes[4]!.home_valid = false;
  s.core.recovery!.faults = [{ bit: 16, name: "coordinate_reference", axis_mask: 16, recovery: "rehome", outcome: "", rehome_axis_mask: 16 }];
  s.core.recovery!.reset_sequence = 12;
  const rejection = operatorRejection("rt_core_event");
  const before = structuredClone(s);
  for (const armed of [0, 1]) {
    s.core.armed = armed;
    const feedback = deriveOperatorFeedback(s, description, rejection);
    const host = new Host(); renderOperatorFeedback(host as unknown as HTMLElement, feedback, "rt_core_event: fault_latched");
    const text = host.rows.join("\n");
    assert.match(text, /J5 position-reference evidence is not yet qualified/);
    assert.match(text, /controller stopped the run because a fault was latched.*\[rt_core_event: fault_latched\]/);
    assert.match(text, /Axis 5: Position reference lost; Home is invalid/);
    assert.doesNotMatch(text, /Unknown rt-core refusal|never homed|J[12346] lost/);
    assert.match(text, /coordinate_reference \(rehome\)/, "raw fault and recovery remain visible");
    assert.deepEqual(feedback.rejection, { ...rejection, text: "The controller stopped motion because joint position evidence was rejected." });
    renderOperatorFeedback(host as unknown as HTMLElement, feedback);
    assert.doesNotMatch(host.rows.join("\n"), /Unknown rt-core refusal/, "a later poll without the original event detail still explains the authoritative fault");
    assert.equal(feedback.reset.admissible, armed === 0);
    assert.equal(feedback.reset.sequence, 12);
    assert.deepEqual(feedback.reset.faults, before.core.recovery!.faults);
  }
  s.core.armed = before.core.armed;
  assert.deepEqual(s, before, "deriving/rendering feedback issues no recovery action or status mutation");
  assert.equal(operatorReasonLabels.rt_core_event, undefined, "shared parity table stays unchanged");
});

test("native recovered-reference proof directs the operator to Clear fault, never new Home", () => {
  const s = status();
  s.core.home_valid_mask = 1;
  s.core.recovery!.faults = [{ bit:6, name:"coordinate_reference", axis_mask:1, recovery:"rehome_required", outcome:"persists", rehome_axis_mask:0, reset_preserves_home:true }];
  assert.match(deriveOperatorFeedback(s, description).note!, /saved Home has recovered.*Clear fault.*No new Home is needed/);
  s.core.recovery!.faults[0]!.reset_preserves_home = false;
  assert.doesNotMatch(deriveOperatorFeedback(s, description).note!, /saved Home has recovered|No new Home is needed/);
});

test("missing or unrelated fault evidence does not invent a lost joint reference", () => {
  const s = status(); s.axes[0]!.home_valid = false;
  assert.equal(deriveOperatorFeedback(s, description).note, undefined);
  for (const [name, mask] of [["drive_fault", 1], ["coordinate_reference", 0]] as const) {
    s.core.recovery!.faults = [{ bit: 16, name, axis_mask: mask, recovery: "reset", outcome: "", rehome_axis_mask: 0 }];
    const feedback = deriveOperatorFeedback(s, description);
    assert.equal(feedback.note, undefined);
    assert.equal(feedback.axes[0]!.homeLoss, "Home required: never homed");
  }
});


test("current position loss takes precedence over historical anchor refusal", () => {
  const s = status();
  Object.assign(s.axes[0]!, { calibration_valid: true, position_state: "lost", position_reason: "discontinuity",
    audit_reason: "torn", home_valid: false, logical_valid: false });
  s.core.axes = [{ anchor_reason: "anchor_disagrees" }] as typeof s.core.axes;
  const result = deriveOperatorFeedback(s, description).axes[0]!.homeLoss;
  assert.match(result, /Calibrated.*position recovery required.*discontinuity/);
  assert.doesNotMatch(result, /Home required|restore refused|Anchor disagrees/);
});

// Recorded the 1430 cell cases: J4 closing 24 counts at 20479 counts/s, J5 opening -8
// counts at -10922 counts/s. J4/J5 scaling: 6553.6 counts per degree. A brake
// warning names the drive delay and is never a refusal, fault, reset or Arm gate.
test("brake-hold warnings render as non-blocking warnings with the drive delay to adjust", (t) => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const countsPerRad = 6553.6 * 180 / Math.PI;
  const d = { axes: [{ index: 0, id: "J4", position_unit: "rad", counts_per_unit: countsPerRad, sign: 1, min_position: -5, max_position: 5 },
    { index: 1, id: "J5", position_unit: "rad", counts_per_unit: countsPerRad, sign: 1, min_position: -5, max_position: 5 }] } as never;
  const s = status();
  s.axes.push({ ...s.axes[0]! });
  s.axes[0]!.brake_hold_diagnostic = { severity: "warning", warning_sequence: 1, transition: "torque_off",
    message: "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).", cause: "velocity_cap",
    phase: "closing", displacement_counts: 24, peak_displacement_counts: 24, peak_velocity_counts_per_s: 20479,
    settled: true, settled_displacement_counts: 24 } as never;
  // An older native without the message still names the release delay.
  s.axes[1]!.brake_hold_diagnostic = { severity: "warning", warning_sequence: 1, transition: "torque_on", message: "",
    cause: "velocity_cap", phase: "opening", displacement_counts: -8, peak_displacement_counts: -8,
    peak_velocity_counts_per_s: -10922, settled: false, settled_displacement_counts: 0 } as never;
  const feedback = deriveOperatorFeedback(s, d);
  assert.equal(feedback.warnings?.length, 2);
  assert.match(feedback.warnings![0]!, /^J4 brake warning: Brake appears to latch after torque off - adjust drive brake hold delay \(C05\.10\)\. /);
  assert.match(feedback.warnings![0]!, /moved 24 counts \(0\.0037°\), peak 20479 counts\/s \(3\.1248°\/s\); settled 24 counts/);
  assert.match(feedback.warnings![1]!, /^J5 brake warning: Brake appears to release after torque on - adjust drive brake release delay \(C05\.13\)\. /);
  assert.match(feedback.warnings![1]!, /moved -8 counts \(-0\.0012°\), peak -10922 counts\/s \(-1\.6666°\/s\); settling/);
  assert.match(feedback.warnings![0]!, /\. Warning only: nothing to reset, and Arm is not blocked\.$/);
  assert.match(feedback.warnings![1]!, /\. Warning only: the drive's release delay still gates motion; nothing to reset\.$/);
  assert.equal(feedback.rejection, null);
  assert.deepEqual(feedback.reset.faults, []);
  assert.equal(feedback.reset.admissible, true, "a warning needs no reset and blocks none");
  const host = new Host();
  renderOperatorFeedback(host as unknown as HTMLElement, feedback);
  assert.match(host.rows[0]!, /^Warning: J4 brake warning/);
  assert.match(host.rows[1]!, /^Warning: J5 brake warning/);
  const mapped = mapRTCoreStatus(s, [], "", d);
  assert.equal(mapped.reason, null);
  assert.equal(mapped.ready, true, "readiness is unchanged by a brake warning");
  assert.equal(mapped.operatorFeedback.warnings?.length, 2);
  // No warning, or an axis without a brake, adds nothing.
  s.axes[0]!.brake_hold_diagnostic = { severity: "none", warning_sequence: 0 } as never;
  delete (s.axes[1] as { brake_hold_diagnostic?: unknown }).brake_hold_diagnostic;
  const quiet = deriveOperatorFeedback(s, d);
  assert.equal(quiet.warnings, undefined);
  assert.equal("brakeWarning" in quiet.axes[0]!, false);
});

// Suspected slip: the third consecutive same-direction warning in ONE braked
// period (J4, 17-count steps, 51 counts total at 6553.6 counts per degree). No
// delay setting explains it, so the text asks for a brake inspection; it is
// rendered apart from settle warnings and still gates nothing.
test("suspected brake slip renders a distinct inspect-the-brake warning that blocks nothing", (t) => {
  Object.defineProperty(globalThis, "document", { configurable: true,
    value: { createElement: () => ({ textContent: "", className: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const countsPerRad = 6553.6 * 180 / Math.PI;
  const d = { axes: [{ index: 0, id: "J4", position_unit: "rad", counts_per_unit: countsPerRad, sign: 1, min_position: -5, max_position: 5 },
    { index: 1, id: "J5", position_unit: "rad", counts_per_unit: countsPerRad, sign: 1, min_position: -5, max_position: 5 }] } as never;
  const s = status();
  s.axes.push({ ...s.axes[0]! });
  const slip = { severity: "warning", warning_sequence: 3, kind: "slip", transition: "torque_off",
    message: "Brake may be slipping: J4 moved 3 times in the same direction (total 51 counts, 0.0078 deg) while the brake was applied - inspect the brake.",
    cause: "disabled_displacement", phase: "held", displacement_counts: 17, peak_displacement_counts: 17,
    peak_velocity_counts_per_s: 0, settled: false, settled_displacement_counts: 0,
    same_direction_count: 3, same_direction_displacement_counts: 51 };
  s.axes[0]!.brake_hold_diagnostic = slip as never;
  // One settle on the other axis keeps the drive-delay advice.
  s.axes[1]!.brake_hold_diagnostic = { severity: "warning", warning_sequence: 1, kind: "settle", transition: "torque_off",
    message: "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).", cause: "transition_corridor",
    phase: "settling", displacement_counts: 17, peak_displacement_counts: 17, peak_velocity_counts_per_s: 0,
    settled: false, settled_displacement_counts: 0, same_direction_count: 1, same_direction_displacement_counts: 17 } as never;
  const feedback = deriveOperatorFeedback(s, d);
  assert.equal(feedback.warnings?.length, 2);
  assert.equal(feedback.warnings![0], "Brake may be slipping: J4 moved 3 times in the same direction " +
    "(total 51 counts, 0.0078 deg) while the brake was applied - inspect the brake. Not a fault; Arm remains available.");
  assert.match(feedback.warnings![1]!, /^J5 brake warning: Brake appears to latch after torque off - adjust drive brake hold delay \(C05\.10\)\. /);
  assert.doesNotMatch(feedback.warnings![1]!, /slipping/);
  assert.equal(feedback.rejection, null);
  assert.deepEqual(feedback.reset.faults, []);
  assert.equal(feedback.reset.admissible, true, "slip needs no reset and blocks none");
  const rows: { textContent: string; className: string }[] = [];
  const host = { replaceChildren: () => { rows.length = 0; }, append: (row: { textContent: string; className: string }) => rows.push(row) };
  renderOperatorFeedback(host as unknown as HTMLElement, feedback);
  assert.equal(rows[0]!.className, "operator-warning operator-warning-slip");
  assert.ok(rows[0]!.textContent.startsWith(`Warning: ${brakeSlipPrefix} J4`));
  assert.equal(rows[1]!.className, "operator-warning");
  const mapped = mapRTCoreStatus(s, [], "", d);
  assert.equal(mapped.reason, null);
  assert.equal(mapped.ready, true, "readiness is unchanged by suspected slip");
  // Without a radian description the total is reported in counts only.
  assert.equal(brakeWarningText("J4", slip as never), "Brake may be slipping: J4 moved 3 times in the same direction " +
    "(total 51 counts) while the brake was applied - inspect the brake. Not a fault; Arm remains available.");
});
// An older native labelled non-brake guard causes (opening timeout, frame
// change, missing wire receipt, ...) as brake warnings. They name no brake
// delay and do not claim Arm is unaffected: such an Arm may not have enabled.
test("non-brake guard causes from an older native name no brake remedy", () => {
  for (const [cause, reason] of [["opening_timeout", "the drive did not finish enabling within the opening bound"],
    ["frame_changed", "its coordinate frame changed"], ["opening_proof", "the enable command receipt was missing"],
    ["stale_feedback", "drive feedback was stale"], ["observation_time", "drive feedback timing was invalid"],
    ["drive_state", "the drive left its expected state"], ["invalid_configuration", "the brake configuration is invalid"]] as const) {
    const s = status();
    s.axes[0]!.brake_hold_diagnostic = { severity: "warning", warning_sequence: 1, transition: "torque_on",
      message: "Brake appears to release after torque on - adjust drive brake release delay (C05.13).", cause,
      phase: "opening", displacement_counts: 0, peak_displacement_counts: 0, peak_velocity_counts_per_s: 0,
      settled: false, settled_displacement_counts: 0 } as never;
    const warning = deriveOperatorFeedback(s, description).warnings![0]!;
    assert.equal(warning, `J1 drive check: ${reason} (${cause} during opening). This is not a brake delay setting; if motion is not ready, Stop and retry Arm.`);
    assert.doesNotMatch(warning, /C05|Arm remains available|Arm is not blocked/);
  }
});

// Drive-state supervisor (every axis, independent of brake configuration). An
// Arm-ending reason is shown while it explains the last Arm attempt and leads
// the strip; disarm motion is a warning. Neither is a fault or needs a reset.
test("drive-supervisor reasons render as ended Arm attempts or disarm warnings", (t) => {
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ textContent: "" }) } });
  t.after(() => Reflect.deleteProperty(globalThis, "document"));
  const d = { axes: [{ index: 0, id: "J4", position_unit: "rad", min_position: -5, max_position: 5 },
    { index: 1, id: "J5", position_unit: "rad", min_position: -5, max_position: 5 }] } as never;
  const s = status();
  s.axes.push({ ...s.axes[0]! });
  const ended = "J4 did not reach Operation Enabled within 500 ms (statusword 0x0233). Arm ended; retry Arm.";
  const moved = "J5 moved 1.200 deg while disarming (drive still energized); bound 1.146 deg.";
  s.axes[0]!.drive_supervisor = { severity: "arm_ended", event_sequence: 1, kind: "enable_timeout", cause: "none",
    message: ended, statusword: 0x0233, arm_ended: true, current: true, elapsed_ns: 500_000_000, bound_ns: 500_000_000,
    displacement_counts: 0, bound_counts: 0, observed_ns: 1 } as never;
  s.axes[1]!.drive_supervisor = { severity: "warning", event_sequence: 1, kind: "disarm_motion", cause: "none",
    message: moved, statusword: 0x0250, arm_ended: false, current: false, elapsed_ns: 49_000_000, bound_ns: 0,
    displacement_counts: 7864, bound_counts: 7510, observed_ns: 1 } as never;
  const feedback = deriveOperatorFeedback(s, d);
  assert.deepEqual(feedback.armEnded, [ended]);
  assert.deepEqual(feedback.warnings, [`${moved} Warning only: nothing latched; check the axis before the next Arm.`]);
  assert.equal(feedback.rejection, null);
  assert.deepEqual(feedback.reset.faults, []);
  assert.equal(feedback.reset.admissible, true, "an ended Arm attempt needs no reset");
  const host = new Host();
  renderOperatorFeedback(host as unknown as HTMLElement, feedback);
  assert.equal(host.rows[0], `Arm ended: ${ended}`);
  assert.equal(host.rows[1], `Warning: ${moved} Warning only: nothing latched; check the axis before the next Arm.`);
  // A cancelled Arm stage from a native without message text still says why.
  s.axes[0]!.drive_supervisor = { ...s.axes[0]!.drive_supervisor!, kind: "arm_cancelled", cause: "frame_changed", message: "" } as never;
  assert.deepEqual(deriveOperatorFeedback(s, d).armEnded, ["J4 Arm ended because frame changed; retry Arm."]);
  // Historical once a later Arm was requested, and never shown while armed.
  s.axes[0]!.drive_supervisor = { ...s.axes[0]!.drive_supervisor!, current: false } as never;
  assert.equal(deriveOperatorFeedback(s, d).armEnded, undefined);
  s.axes[0]!.drive_supervisor = { ...s.axes[0]!.drive_supervisor!, current: true } as never;
  s.core.armed = 1;
  assert.equal(deriveOperatorFeedback(s, d).armEnded, undefined);
  // No event adds nothing.
  delete (s.axes[0] as { drive_supervisor?: unknown }).drive_supervisor;
  delete (s.axes[1] as { drive_supervisor?: unknown }).drive_supervisor;
  const quiet = deriveOperatorFeedback(s, d);
  assert.equal(quiet.armEnded, undefined);
  assert.equal(quiet.warnings, undefined);
});

// the 1430 cell 2026-10-11 12:47: rt-core published J5's decoded drive alarm and the
// UI showed nothing (alarmText was never filled).
test("driveAlarmText names the axis and the drive's decoded alarm, a warning on a ready axis as a warning", () => {
  assert.equal(driveAlarmText("J5", { drive_alarm: "Er41.1", drive_alarm_text: "Motor over-temperature due to locked- rotor", readiness: "faulted" }),
    "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor)");
  assert.equal(driveAlarmText("J3", { drive_alarm: "AL.600", drive_alarm_text: "", readiness: "ready" }), "J3 drive warning AL.600");
  assert.equal(driveAlarmText("J5", { drive_alarm: "", drive_alarm_text: "", readiness: "faulted" }, { error_code: 0x7121 }), "J5 drive fault 0x7121");
  assert.equal(driveAlarmText("J5", { drive_alarm: "", drive_alarm_text: "", readiness: "faulted" }, { error_code: 0 }), "");
  assert.equal(driveAlarmText("J1", { drive_alarm: "", drive_alarm_text: "", readiness: "ready" }, { error_code: 0x7121 }), "");
});
