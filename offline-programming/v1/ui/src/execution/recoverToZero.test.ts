import assert from "node:assert/strict";
import test from "node:test";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import {
  normalizeRecoverResult, recoverConfirmation, recoverFailureText, recoverOutcomeText, recoverRequest, recoverStepsText, recoverToZeroDisabled,
} from "./recoverToZero.ts";

const refused409 = {
  error: "fault_persists",
  detail: "J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again",
  recovery: {
    state: "refused", action: "Arm and move all joints to zero",
    steps: [{ step: "start", outcome: "done" }, { step: "confirm_stop", outcome: "done" }, { step: "reset", outcome: "refused", detail: "fault_persists: ..." }],
    joints: [{ axis: 4, id: "J5", from_deg: 37.4, to_deg: 0, delta_deg: -37.4, moves: true }],
    refusal: { step: "reset", code: "fault_persists", detail: "J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again" },
  },
};

test("a refusal reads as what cleared, then the server's sentence", () => {
  assert.equal(recoverFailureText("fault_persists", refused409.detail, refused409),
    "Stop confirmed; then J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again");
  // The maintenance and control gates answer without a recovery member: their detail.
  assert.equal(recoverFailureText("maintenance", "Update in progress: OLP update. Robot actions are refused until it ends; Stop, Disarm and Disconnect still work.", { error: "maintenance" }),
    "Update in progress: OLP update. Robot actions are refused until it ends; Stop, Disarm and Disconnect still work.");
  assert.equal(recoverFailureText("in_use_by_other_controller", "The robot is in use by Browser (Chrome on macOS) ... Stop always works."),
    "The robot is in use by Browser (Chrome on macOS) ... Stop always works.");
});

test("the confirmation names the action, every joint's angle → 0° and travel, and the sweep", () => {
  const c = recoverConfirmation(normalizeRecoverResult({ state: "confirm_arm", action: "Arm and move all joints to zero",
    steps: [{ step: "reset", outcome: "done" }, { step: "arm", outcome: "needs_confirmation" }],
    joints: [{ axis: 4, id: "J5", from_deg: 37.4, to_deg: 0, delta_deg: -37.4, moves: true }, { axis: 1, id: "J2", from_deg: -11.46, to_deg: 0, delta_deg: 11.46, moves: true }] }));
  assert.equal(c.title, "Arm and move all joints to zero");
  assert.deepEqual(c.lines, ["J5  37.40° → 0.00°  Δ −37.40°", "J2  -11.46° → 0.00°  Δ +11.46°"]);
  assert.match(c.intro, /^faults reset\. The robot is disarmed\. Confirming arms it/);
  assert.equal(c.confirm, "Arm and move");
});

test("outcomes, requests and the gate", () => {
  assert.equal(recoverStepsText([{ step: "confirm_stop", outcome: "done" }, { step: "reset", outcome: "done" }, { step: "arm", outcome: "done" }, { step: "move", outcome: "done" }]),
    "Stop confirmed · faults reset · armed · moved to 0°");
  assert.equal(recoverOutcomeText(normalizeRecoverResult({ state: "at_zero", steps: [{ step: "reset", outcome: "done" }, { step: "move", outcome: "done", detail: "every joint is already at 0°" }] })),
    "Every joint is already at 0° (faults reset; not armed)");
  assert.deepEqual(recoverRequest(0.1), { fraction: 0.1, confirm_arm: false });
  const caps = { backend: "rt_core", cell: "example-arm-1420", daemonReachable: true, rtCore: { axes: [] } } as unknown as DenseCapabilities;
  const stuck = { armed: false, session: null, daemonError: "operator_stop: control_session_stale", target: { model_id: "rosie_1420_v1" } } as unknown as DenseStatus;
  assert.equal(recoverToZeroDisabled(true, caps, stuck, false), "", "disarmed, no grant and a latched Stop are what it recovers");
  assert.equal(recoverToZeroDisabled(false, caps, stuck, false), "Connect a machine first");
  assert.equal(recoverToZeroDisabled(true, caps, { ...stuck, session: { id: "s", state: "playing" } } as unknown as DenseStatus, false), "Another target operation is active");
});

test("return-to-zero carries base jog rates and scales only the displayed effective limits", () => {
  const speeds = { linearSpeedMMs: 400, angularSpeedDegS: 30 };
  const request = recoverRequest(.25, undefined, speeds);
  assert.deepEqual(request, { fraction: .25, confirm_arm: false, linear_speed_mm_s: 400, angular_speed_deg_s: 30 });
  const result = normalizeRecoverResult({ state: "confirm_arm", joints: [{ axis: 0, id: "J1", from_deg: 10, to_deg: 0, delta_deg: -10, moves: true }] });
  assert.match(recoverConfirmation(result, request).intro, /tool up to 100 mm\/s and 7\.5°\/s/);
  assert.deepEqual(recoverRequest(.25, result, speeds), { ...request, confirm_arm: true, confirmed: [{ axis: 0, id: "J1", from_deg: 10 }] });
  for (const value of [0, -1, NaN, Infinity]) {
    assert.throws(() => recoverRequest(.25, undefined, { ...speeds, linearSpeedMMs: value }), /positive tool/);
    assert.throws(() => recoverRequest(.25, undefined, { ...speeds, angularSpeedDegS: value }), /positive tool/);
  }
});
