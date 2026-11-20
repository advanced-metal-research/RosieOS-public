import assert from "node:assert/strict";
import test from "node:test";
import { CapabilitiesDigest, ContractVersion } from "../../../../rt-core/clients/ts/rt_control_api_generated.ts";
import * as machine from "./machine.ts";

// The Dense panel's teach fixture (densePanel.test.ts): six measured joints,
// no velocity PDOs, so stillness is two native observations of one pose.
const description = { contract_version: ContractVersion, capabilities_digest: CapabilitiesDigest, machine_sha256: "a".repeat(64), deployment_sha256: "b".repeat(64),
  axes: ["J1", "J2", "J3", "J4", "J5", "J6"].map((id, index) => ({ index, id, position_unit: "rad", min_position: -5, max_position: 5 })), capabilities: [] };
const positions = [0.0498, 0.1937, 0, 0, 0, 0];
function status(timeNs: number, state = "loaded") {
  return {
    backend: "rt_core", available: true,
    // The OLP server's execution session: what denseExecutionState reads.
    session: { id: "ds-1", state },
    robot_cell: { valid: true, model_id: "rosie_1420_v1", robot_description_sha256: "sha256:" + "1".repeat(64) },
    target: { selection_generation: "g1", cell_id: "la-bench", model_id: "rosie_1420_v1", label: "Rosie 1420 bench", backend: "ethercat", host: "example-bench-relay" },
    rt_core: { description, status: {
      time_ns: timeNs, adapter_incarnation: "adapter", daemon_incarnation: "daemon", execution: { state },
      core: { armed: 1, safety_fault_mask: 0 },
      axes: positions.map(position => ({ home_valid: true, logical_valid: true, calibration_valid: true, position_state: "trusted", readiness: "ready", logical_position: position, logical_target: position, statusword: 0, position_counts: 0 })),
    } },
  };
}
function connect() {
  machine.forgetMachine();
  machine.observeCapabilities({ backend: "rt_core", available: true, daemon_reachable: true, rt_core: { description } });
}
const teach = (busy = false) => machine.teachObservation({ selected: true, busy, statusAgeMs: 0, cellModel: "rosie_1420_v1", cellId: "la-bench" });

test("lease display uses fresh native status, independently of an OLP session", () => {
  connect();
  const input: machine.MachineInput = {
    catalogue: null, catalogueError: null, highlighted: null, connectedCell: "la-bench", connectedLabel: "Rosie 1420 bench",
    modelId: "rosie_1420_v1", connected: true, cellPending: false, connectError: null, inFlight: false,
    posture: { backend: "ethercat", simulation: false } as never, armConfirmPending: false, statusAgeMs: 0,
    pendingRequest: null, lastRequest: null, jogBusy: false, fraction: 0.1,
  };
  let time = 1_000_000_000;
  for (const [lease, expected] of [[0, false], [1, true], [undefined, null], [7, null]] as const) {
    const s = status(time++);
    (s as Record<string, unknown>).session = null;
    (s.rt_core.status.core as Record<string, unknown>).lease_valid = lease;
    machine.observeStatus(s, false);
    assert.equal(machine.machineView(input).leaseValid, expected);
    assert.equal(machine.machineView({ ...input, statusAgeMs: 3000 }).leaseValid, null);
    assert.equal(machine.machineView({ ...input, connected: false }).leaseValid, null);
  }
  machine.observeSilence("connection failed");
  assert.equal(machine.machineView(input).leaseValid, null);
});

test("two idle observations of one pose admit a capture", () => {
  connect();
  machine.observeStatus(status(1_000_000_000), false);
  machine.observeStatus(status(1_250_000_000), false);
  assert.deepEqual(teach().pose.jointsRad.slice(0, 6), positions);
});

test("statuses taken while the pendant is busy never count toward stillness", () => {
  connect();
  // A held jog or a move spans these polls; the pose reads the same because
  // velocity PDOs are absent, which is exactly when the window must not accrue.
  machine.observeStatus(status(1_000_000_000), true);
  machine.observeStatus(status(1_250_000_000), true);
  machine.observeStatus(status(1_500_000_000), false);
  assert.throws(() => teach(), /stationary position samples/, "the first idle status starts the window");
  machine.observeStatus(status(1_750_000_000), false);
  assert.deepEqual(teach().pose.jointsRad.slice(0, 6), positions);
});

test("a playing or paused program is busy for teach, as the Dense panel's robotMoving", () => {
  for (const state of ["playing", "paused"]) {
    connect();
    machine.observeStatus(status(1_000_000_000, state), false);
    machine.observeStatus(status(1_250_000_000, state), false);
    assert.throws(() => teach(), /fresh stationary feedback/, state);
  }
});

test("a stillness window does not survive forgetting the machine", () => {
  connect();
  machine.observeStatus(status(1_000_000_000), false);
  machine.observeStatus(status(1_250_000_000), false);
  connect();
  machine.observeStatus(status(1_500_000_000), false);
  assert.throws(() => teach(), /stationary position samples/);
});

// Teaching on the Program page (owner, 2026-10-12): armed with a plan loaded
// and not playing, the jog gate is open; a playing program or a running loop
// (between cycles too) closes it with words the jog strip shows.
test("the jog gate: open with a plan loaded, closed while it plays or a loop runs", () => {
  const caps = () => machine.observeCapabilities({ backend: "rt_core", available: true, daemon_reachable: true, recover_zero_jog_speeds: true, cell: "la-bench", rt_core: { description } });
  const at = (state: string, loop?: Record<string, unknown>) => {
    machine.forgetMachine();
    caps();
    const s = status(1_000_000_000, state) as Record<string, unknown>;
    (s.session as Record<string, unknown>).lease = { held: true };
    s.armed = true;
    if (loop) s.loop = loop;
    machine.observeStatus(s, false);
    return machine.machineView({ catalogue: null, catalogueError: null, highlighted: null, connectedCell: "la-bench", connectedLabel: "Rosie 1420 bench",
      modelId: "rosie_1420_v1", connected: true, cellPending: false, connectError: null, inFlight: false,
      posture: { backend: "ethercat", simulation: false } as never, armConfirmPending: false, statusAgeMs: 0, pendingRequest: null,
      lastRequest: null, jogBusy: false, fraction: 0.1 }).jogDisabled;
  };
  assert.equal(at("loaded"), "");
  assert.match(at("playing"), /Another target operation/);
  assert.match(at("done", { mode: "loop", count: 0, cycles: 2, state: "running" }), /program loop is running; STOP it/);
  assert.equal(at("done", { mode: "count", count: 3, cycles: 3, state: "ended", end_reason: "ran 3 cycles" }), "");
});

// cell A 2026-10-11 12:19: J5 Er41.1 and OLP's unconfirmed Stop. Every Arm press
// on the Deck was refused with the generic rt_core_inhibited line or nothing.
function incident(stopUncertain: boolean) {
  const s = status(2_000_000_000, "stopped");
  const st = s.rt_core.status as Record<string, unknown> & { core: Record<string, unknown>; axes: Record<string, unknown>[] };
  st.core = { armed: 0, safety_fault_mask: 16, execution_fault_reasons: 128, axis_enable_mask: 0,
    recovery: { reason: "fault_persists", reset_sequence: 140, faults: [{ bit: 7, name: "drive_readiness", axis_mask: 16, recovery: "reset_after_condition_clears", outcome: "persists", rehome_axis_mask: 0 }] },
    axes: positions.map((_, i) => ({ error_code: i === 4 ? 0x7121 : 0 })) };
  st.axes[4] = { ...st.axes[4], readiness: "faulted", drive_alarm: "Er41.1", drive_alarm_text: "Motor over-temperature due to locked- rotor" };
  return { ...s, rt_core: { ...s.rt_core, stop_uncertain: stopUncertain, inhibited: false, reason: "fault_persists: control_session_stale" } };
}

// Owner 2026-10-11: "re-arming the robot should clear faults too if they
// exist". The Deck's A stays Arm: OLP confirms the Stop and resets J5 first.
test("in the incident state A stays Arm, the view says Arm resets J5 first, and a hot J5 is the server's words", () => {
  connect();
  machine.observeStatus(incident(true), false);
  const view = machine.machineView({ catalogue: null, catalogueError: null, highlighted: null, connectedCell: "la-bench", connectedLabel: "Rosie 1420 bench",
    modelId: "rosie_1420_v1", connected: true, cellPending: false, connectError: null, inFlight: false,
    posture: { backend: "ethercat", simulation: false } as never, armConfirmPending: false, statusAgeMs: 0, pendingRequest: null,
    lastRequest: null, jogBusy: false, fraction: 0.1 });
  assert.equal(view.flow.canArm, true);
  assert.equal(view.flow.armBlockers.length, 2);
  assert.match(view.flow.armBlockers[0]!, /^The last Stop is unconfirmed\. Arm confirms it from the robot's state first/);
  assert.match(view.flow.armBlockers[1]!, /^J5 drive alarm Er41\.1 \(Motor over-temperature due to locked- rotor\)\. Arm will reset J5 Er41\.1 first/);
  // The Stop could not be confirmed: the standing causes lead Arm's refusal.
  const refusal = machine.armRefusalText("rt_core_inhibited", "The last Stop is still unconfirmed: the servo host still shows a lease or grant");
  assert.match(refusal, /^Arm refused: The last Stop is unconfirmed\..*J5 drive alarm Er41\.1 .*\[rt_core_inhibited\]$/);
  // J5 still hot: the server's sentence, not a generic line.
  machine.observeStatus(incident(false), false);
  assert.equal(machine.armRefusalText("fault_persists", "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again"),
    "Arm refused: J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again [fault_persists]");
});

// The owner's one-press recovery on the Deck (2026-10-11): in the incident
// state ALL JOINTS TO 0° stays pressable while jogging is refused, the first
// answer becomes OLP's confirmation, and a still-hot J5 is the server's words.
test("All joints to 0° recovers the incident state: its own gate, the confirmation and the refusal", () => {
  machine.forgetMachine();
  machine.observeCapabilities({ backend: "rt_core", available: true, daemon_reachable: true, recover_zero_jog_speeds: true, cell: "la-bench", rt_core: { description } });
  machine.observeStatus(incident(true), false);
  const view = machine.machineView({ catalogue: null, catalogueError: null, highlighted: null, connectedCell: "la-bench", connectedLabel: "Rosie 1420 bench",
    modelId: "rosie_1420_v1", connected: true, cellPending: false, connectError: null, inFlight: false,
    posture: { backend: "ethercat", simulation: false } as never, armConfirmPending: false, statusAgeMs: 0, pendingRequest: null,
    lastRequest: null, jogBusy: false, fraction: 0.1 });
  assert.notEqual(view.jogDisabled, "");
  assert.equal(view.zeroDisabled, "", "disarmed with an unconfirmed Stop is what the button recovers");
  assert.deepEqual(machine.recoverBody(0.1), { fraction: 0.1, confirm_arm: false });
  const asked = machine.recoverAnswer({ state: "confirm_arm", action: "Arm and move all joints to zero",
    steps: [{ step: "confirm_stop", outcome: "done" }, { step: "reset", outcome: "done" }, { step: "arm", outcome: "needs_confirmation" }],
    joints: [{ axis: 4, id: "J5", from_deg: 37.4, to_deg: 0, delta_deg: -37.4, moves: true }, { axis: 2, id: "J3", from_deg: 0, to_deg: 0, delta_deg: 0, moves: false }] }, true);
  assert.equal(asked.state, "confirm_arm");
  assert.equal(asked.confirmation!.title, "Arm and move all joints to zero");
  assert.deepEqual(asked.confirmation!.lines, ["J5  37.40° → 0.00°  Δ −37.40°", "J3  0.00° → 0.00°  stays"]);
  assert.deepEqual(machine.recoverBody(0.1, asked.result), { fraction: 0.1, confirm_arm: true, confirmed: [{ axis: 4, id: "J5", from_deg: 37.4 }, { axis: 2, id: "J3", from_deg: 0 }] });
  const hot = "J5 Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again";
  const refused = machine.recoverAnswer({ error: "fault_persists", detail: hot, recovery: { state: "refused", steps: [{ step: "confirm_stop", outcome: "done" }],
    refusal: { step: "reset", code: "fault_persists", detail: hot } } }, false, "fault_persists", hot);
  assert.deepEqual(refused, { state: "refused", text: `Stop confirmed; then ${hot}` });
  assert.equal(machine.recoverAnswer({ state: "done", steps: [{ step: "arm", outcome: "done" }, { step: "move", outcome: "done" }] }, true).text, "All joints at 0° (armed · moved to 0°)");
});

test("return-to-zero refuses a server that cannot enforce the selected jog rates", () => {
  machine.forgetMachine();
  const caps = { backend: "rt_core", available: true, daemon_reachable: true, cell: "la-bench", rt_core: { description } };
  const input: machine.MachineInput = { catalogue: null, catalogueError: null, highlighted: null, connectedCell: "la-bench", connectedLabel: "bench",
    modelId: "rosie_1420_v1", connected: true, cellPending: false, connectError: null, inFlight: false, posture: null,
    armConfirmPending: false, statusAgeMs: 0, pendingRequest: null, lastRequest: null, jogBusy: false, fraction: .25 };
  machine.observeStatus(incident(true), false);
  for (const support of [undefined, false, "true", true]) {
    machine.observeCapabilities({ ...caps, recover_zero_jog_speeds: support });
    assert.equal(machine.machineView(input).zeroDisabled, support === true ? "" : "Update the cell software to return to zero at the selected jog speeds");
  }
  const request = machine.recoverBody(.25, undefined, { linearSpeedMMs: 400, angularSpeedDegS: 30 });
  assert.equal(request.linear_speed_mm_s, 400);
  assert.equal(request.angular_speed_deg_s, 30);
  const answer = machine.recoverAnswer({ state: "confirm_arm", joints: [] }, true, "", "", request);
  assert.match(answer.confirmation!.intro, /100 mm\/s and 7\.5°\/s/);
});

test("pendant capture tolerates bounded servo settling, never changing commands or untrusted motion", () => {
  connect();
  machine.observeCapabilities({ backend: "rt_core", available: true, rt_core: { description: { ...description, axes: description.axes.map(a => ({ ...a, completion_tolerance: .0006 })) } } });
  const frame = (ms: number, delta: number, target = 0) => {
    const s = status(1e9 + ms * 1e6);
    s.rt_core.status.axes[5]!.logical_position = delta;
    s.rt_core.status.axes[5]!.logical_target = target;
    return s;
  };
  for (let ms=0;ms<=300;ms+=50) machine.observeStatus(frame(ms, ms % 100 ? .00027 : -.00027),false);
  assert.ok(Math.abs(teach().pose.jointsRad[5]!) === .00027, "save latest measured pose, not averaged or target pose");
  assert.equal(machine.teachDiagnostics().ready, true);
  machine.observeStatus(frame(350,.0008),false);
  assert.throws(() => teach(), /stationary position samples/);
  // A changed command is not servo dither; even a mild target change resets readiness.
  for (let ms=400;ms<=700;ms+=50) machine.observeStatus(frame(ms,.0001,(ms-400)*.000001),false);
  assert.throws(() => teach(), /stationary position samples/);
  machine.observeStatus(frame(750,0),true);
  machine.observeStatus(frame(800,0),false);
  assert.throws(() => teach(), /stationary position samples/);
  for (let ms=850;ms<=1100;ms+=50) machine.observeStatus(frame(ms,0),false);
  assert.doesNotThrow(() => teach());
  machine.observeSilence("lost");
  machine.observeStatus(frame(1150,0),false);
  assert.throws(() => teach(), /stationary position samples/);
  const invalid = frame(1400,0); invalid.rt_core.status.axes[5]!.home_valid = false;
  machine.observeStatus(invalid,false);
  assert.throws(() => teach(), /verified position/);
});

test("reported zero velocities cannot bypass pendant target settling", () => {
  connect();
  const s = status(1e9);
  for (const a of s.rt_core.status.axes) (a as Record<string,unknown>).logical_velocity = 0;
  machine.observeStatus(s,false);
  assert.throws(() => teach(), /stationary position samples/);
  s.rt_core.status.time_ns += 250e6;
  machine.observeStatus(s,false);
  assert.doesNotThrow(() => teach());
  s.rt_core.status.time_ns += 50e6;
  s.rt_core.status.axes[5]!.logical_target = .01;
  machine.observeStatus(s,false);
  assert.throws(() => teach(), /stationary position samples/);
});


test("a stationary following offset records measured pose rather than refusing or substituting target", () => {
  connect();
  const s = status(1e9);
  s.rt_core.status.axes[5]!.logical_target = .001;
  machine.observeStatus(s,false);
  s.rt_core.status.time_ns += 250e6;
  machine.observeStatus(s,false);
  assert.equal(teach().pose.jointsRad[5],0);
});
