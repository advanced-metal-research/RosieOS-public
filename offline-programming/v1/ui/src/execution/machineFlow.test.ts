import assert from "node:assert/strict";
import test from "node:test";
import { armBlockers, armRefusalText, deriveMachineFlow, isMachineFault, isTransportUnsent, machineReasonText, machineStatePill, STATUS_FRESHNESS_MS, type ArmState, type MachineFlowInput } from "./machineFlow.ts";
import { normalizeDenseCapabilities, normalizeDenseStatus, type DenseCapabilities, type DenseStatus } from "./denseClient.ts";
import type { CellTarget } from "../targets/cells.ts";

const bench: CellTarget = {
  cell_id: "example-bench-1420", label: "Test bench", role: "TEST BENCH / BENCH-RELAY",
  models: ["rosie_1420_v1"], requested_mode: "real", backend: "ethercat", simulation: false, host: "127.0.0.1:8443",
};
const local: CellTarget = {
  cell_id: "local-simulation", label: "Local simulation", role: "LOCAL SIMULATION",
  models: ["rosie_1400_v3"], requested_mode: "simulation", backend: "simulation", simulation: true, host: "localhost",
};

function caps(overrides: Partial<DenseCapabilities> = {}): DenseCapabilities {
  return {
    backend: "rt_core", available: true, cell: "bench", ingestAddr: "", natsUrl: "", rtcoreHost: "", controllerId: "olp",
    daemonReachable: true, reason: null,
    rtCore: { nativeHome: true, axes: [{ index: 0, id: "J1", position_unit: "rad" }, { index: 1, id: "J2", position_unit: "rad" }] } as unknown as DenseCapabilities["rtCore"],
    ...overrides,
  };
}

// `armed` here is the server's compound (`core.armed && held && reason == ""`,
// null under any reason); `coreArmed` is the drives' own word, which is what
// the fact renders. `motors` sets the drives' word independently for the
// cases where the server's compound and the observation differ.
function status(overrides: { armed?: boolean | null; motors?: boolean; held?: boolean; home?: boolean[]; ready?: boolean; state?: string; error?: string | null; session?: boolean;
  flags?: Partial<NonNullable<DenseStatus["rtCoreFlags"]>> } = {}): DenseStatus {
  const home = overrides.home ?? [true, true];
  const ready = overrides.ready ?? true;
  const motors = overrides.motors ?? overrides.armed === true;
  return {
    backend: "rt_core", available: true, daemon: null, cursor: null, axes: null,
    session: overrides.session === false ? null : { id: "s", trajectoryDigest: "", planId: "", state: overrides.state ?? "idle", leaseHeld: overrides.held ?? true, leaseExpiresInMs: null, heartbeatAgeMs: null, heartbeatDeadlineMs: null, stopReason: null, lastError: null },
    daemonError: overrides.error ?? null,
    armed: overrides.armed === undefined ? false : overrides.armed,
    rtCoreFlags: { inhibited: false, stopUncertain: false, recoveryAction: null, note: null, ...overrides.flags },
    rtCore: {
      armed: overrides.armed === true, coreArmed: motors, ready: ready && home.every(Boolean), reason: null,
      execution: { state: overrides.state ?? "idle" },
      axes: home.map((valid, index) => ({ position: 0, target: 0, velocity: 0, homeValid: valid, readiness: valid && ready ? "ready" : valid ? "not_ready" : "home_required", statusword: 0, positionCounts: index })),
      operatorFeedback: { rejection: null, reset: { admissible: false, text: "", reason: "", sequence: 0, faults: [] },
        axes: home.map((valid, index) => ({ index, rejection: null, limit: null, homeLoss: valid ? "" : "Home required: never homed", reset: { admissible: false, text: "", reason: "", sequence: 0, faults: [] }, alarmText: "" })) },
    } as unknown as DenseStatus["rtCore"],
  };
}

function input(overrides: Partial<MachineFlowInput> = {}): MachineFlowInput {
  return {
    catalogue: [local, bench], catalogueError: null, highlighted: null, connectedCell: "", connectedLabel: "", modelId: "rosie_1420_v1",
    connected: false, cellPending: false, connectError: null, inFlight: false, posture: null, capabilities: null, status: null, armConfirmPending: false,
    // A status handed in (here or spread in later) is fresh unless a test
    // says otherwise; with no status the fact is UNKNOWN whatever the age.
    statusAgeMs: 0, pendingRequest: null, lastRequest: null,
    ...overrides,
  };
}

const states = (view: ReturnType<typeof deriveMachineFlow>) => view.steps.map(step => `${step.id}:${step.state}`);
const noLegacy = { executionMode: "simulation" as const, robotLive: false, connectedArmed: false };
const onBench = (overrides: Partial<MachineFlowInput> = {}) => input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
  posture: { backend: "ethercat", simulation: false }, capabilities: caps(), ...overrides });

test("unknown picker or status model blocks Arm and Drive even with fresh ready status", () => {
  for (const armed of [false, true]) {
    for (const [modelId, reported] of [["bench_unknown", "rosie_1420_v1"], ["rosie_1420_v1", "bench_unknown"]]) {
      const observed = status({ armed });
      observed.target = { model_id: reported } as DenseStatus["target"];
      const view = deriveMachineFlow(onBench({ modelId, status: observed }));
      assert.equal(view.arm.state, armed ? "armed" : "disarmed");
      assert.equal(view.canArm, false); assert.equal(view.canDrive, false);
      for (const step of view.steps.slice(3)) {
        assert.equal(step.state, "blocked");
        assert.equal(step.summary, "Unknown robot model bench_unknown: this app does not know how to drive it");
      }
      assert.equal(view.stopEnabled, true); assert.equal(view.disarmEnabled, true);
    }
  }
});

test("nothing chosen: Choose is the only current step and the pill says NO MACHINE", () => {
  const view = deriveMachineFlow(input());
  assert.deepEqual(states(view), ["choose:current", "connect:pending", "home:pending", "arm:pending", "drive:pending"]);
  assert.equal(view.current, "choose");
  assert.equal(view.posture, "none");
  assert.equal(view.stopEnabled, false);
  const pill = machineStatePill(view, { executionMode: "simulation", robotLive: false, connectedArmed: false });
  assert.equal(pill.label, "NO MACHINE");
  assert.equal(pill.className, "is-none");
});

test("a highlighted unavailable cell blocks Choose with the typed reason in words and keeps the code", () => {
  const view = deriveMachineFlow(input({ highlighted: { ...bench, reason: "cell_configuration_mismatch" } }));
  const choose = view.steps[0]!;
  assert.equal(choose.state, "blocked");
  assert.equal(choose.code, "cell_configuration_mismatch");
  assert.match(choose.summary, /deployed configuration no longer matches/);
  assert.match(choose.summary, /\[cell_configuration_mismatch\]/);
  // The server is the authority: Connect remains the next action.
  assert.equal(view.steps[1]!.state, "current");
  assert.equal(view.canConnect, true);
  const missing = deriveMachineFlow(input({ highlighted: { ...bench, reason: "cell_configuration_missing" } }));
  assert.equal(missing.steps[1]!.state, "pending");
  assert.equal(missing.canConnect, false);
});

test("a highlighted real cell makes Connect current and names it a real machine", () => {
  const view = deriveMachineFlow(input({ highlighted: bench }));
  assert.deepEqual(states(view).slice(0, 2), ["choose:done", "connect:current"]);
  assert.match(view.steps[0]!.summary, /a real machine/);
  assert.match(view.steps[1]!.summary, /mutual TLS/);
  assert.equal(view.canConnect, true);
});

test("a refused Connect is blocked with words and the raw code", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connectError: "cell_mode_mismatch" }));
  assert.equal(view.steps[1]!.state, "blocked");
  assert.equal(view.steps[1]!.code, "cell_mode_mismatch");
  assert.match(view.steps[1]!.summary, /live backend is not what the catalogue requests/);
});

test("connected real cell without status: Home waits, Arm stays the server's call, Drive pending", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps() }));
  assert.deepEqual(states(view), ["choose:done", "connect:done", "home:pending", "arm:current", "drive:pending"]);
  assert.equal(view.posture, "real");
  assert.match(view.steps[1]!.summary, /This is a real machine/);
  assert.equal(view.stopEnabled, true);
  // No status yet is the one honest UNKNOWN; the pill says so rather than picking a side.
  assert.equal(view.arm.state, "unknown");
  assert.equal(view.steps[3]!.headline, "UNKNOWN: no fresh status from the machine");
  assert.match(view.steps[3]!.summary, /has not reported a status yet/);
  assert.equal(machineStatePill(view, noLegacy).label, "REAL · Test bench · STATUS STALE");
});

test("Home required puts Home current, Arm pending; native Home needs disarmed drives", () => {
  const base = input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps() });
  const required = deriveMachineFlow({ ...base, status: status({ home: [true, false] }) });
  assert.deepEqual(states(required), ["choose:done", "connect:done", "home:current", "arm:pending", "drive:pending"]);
  assert.deepEqual(required.homeRequired, ["J2"]);
  assert.match(required.steps[2]!.summary, /J2 report Home required/);
  assert.match(required.steps[2]!.summary, /briefly energising the drives/);
  assert.match(required.steps[2]!.summary, /settling movement may occur/);
  assert.doesNotMatch(required.steps[2]!.summary, /nothing moves/);
  assert.equal(required.canHome, true);
  assert.equal(required.canArm, false);

  const armedFirst = deriveMachineFlow({ ...base, status: status({ home: [true, false], armed: true }) });
  assert.equal(armedFirst.steps[2]!.state, "blocked");
  assert.match(armedFirst.steps[2]!.summary, /DISARM, then Home/);

  const noNative = deriveMachineFlow({ ...base, capabilities: caps({ rtCore: { nativeHome: false, axes: [] } as unknown as DenseCapabilities["rtCore"] }), status: status({ home: [true, false] }) });
  assert.equal(noNative.steps[2]!.state, "blocked");
  assert.match(noNative.steps[2]!.summary, /does not offer native Home/);
});

test("Home valid and disarmed: Arm is the one current step and a real cell requires confirmation", () => {
  const base = input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps(), status: status() });
  const view = deriveMachineFlow(base);
  assert.deepEqual(states(view), ["choose:done", "connect:done", "home:done", "arm:current", "drive:pending"]);
  assert.match(view.steps[2]!.summary, /Home valid on J1, J2/);
  assert.match(view.steps[3]!.summary, /energises the motors on Test bench/);
  assert.equal(view.armConfirmRequired, true);
  assert.equal(view.canArm, true);
  const confirming = deriveMachineFlow({ ...base, armConfirmPending: true });
  assert.match(confirming.steps[3]!.summary, /Confirm to energise/);
  const pill = machineStatePill(view, { executionMode: "simulation", robotLive: false, connectedArmed: false });
  assert.equal(pill.label, "REAL · Test bench · DISARMED");
  assert.equal(pill.className, "is-real-idle");
});

test("a simulation cell arms without confirmation and its pill stays green until armed", () => {
  const base = input({ highlighted: local, connected: true, connectedCell: local.cell_id, connectedLabel: local.label, modelId: "rosie_1400_v3",
    posture: { backend: "simulation", simulation: true }, capabilities: caps({ cell: "local" }), status: status() });
  const view = deriveMachineFlow(base);
  assert.equal(view.posture, "simulation");
  assert.equal(view.armConfirmRequired, false);
  assert.match(view.steps[3]!.summary, /No physical output/);
  let pill = machineStatePill(view, { executionMode: "production", robotLive: true, connectedArmed: true });
  assert.equal(pill.label, "SIMULATION · Local simulation · DISARMED");
  assert.equal(pill.className, "is-offline");
  const armed = deriveMachineFlow({ ...base, status: status({ armed: true }) });
  assert.deepEqual(states(armed), ["choose:done", "connect:done", "home:done", "arm:done", "drive:current"]);
  assert.match(armed.steps[4]!.summary, /on the simulation/);
  pill = machineStatePill(armed, { executionMode: "simulation", robotLive: false, connectedArmed: false });
  assert.equal(pill.label, "SIMULATION · Local simulation · ARMED");
  assert.equal(pill.className, "is-sim-armed");
});

test("armed real cell: Drive current, pill red and filled; playing shows the state", () => {
  const base = input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps() });
  const armed = deriveMachineFlow({ ...base, status: status({ armed: true }) });
  assert.deepEqual(states(armed), ["choose:done", "connect:done", "home:done", "arm:done", "drive:current"]);
  assert.equal(armed.steps[3]!.headline, "ARMED");
  assert.match(armed.steps[3]!.summary, /Motors on Test bench are energised/);
  assert.match(armed.steps[4]!.summary, /Motion is physical/);
  assert.equal(armed.canDrive, true);
  assert.equal(machineStatePill(armed, { executionMode: "simulation", robotLive: false, connectedArmed: false }).className, "is-real");
  const playing = deriveMachineFlow({ ...base, status: status({ armed: true, state: "playing" }) });
  assert.equal(playing.live, true);
  assert.match(playing.steps[4]!.summary, /playing/);
  assert.equal(machineStatePill(playing, { executionMode: "simulation", robotLive: false, connectedArmed: false }).label, "REAL · Test bench · PLAYING");
});

test("armed but not ready blocks Drive and names the axis", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps(), status: status({ armed: true, ready: false }) }));
  assert.equal(view.steps[4]!.state, "blocked");
  assert.match(view.steps[4]!.summary, /Not ready: J1 not_ready, J2 not_ready/);
  assert.equal(view.canDrive, false);
});

test("a typed rt-core refusal blocks Home, Arm and Drive with the same words and code", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps(), status: status({ armed: null, error: "rt_core_inhibited: acknowledged Stop and Release required" }) }));
  for (const step of view.steps.slice(2)) {
    assert.equal(step.state, "blocked", step.id);
    assert.equal(step.code, "rt_core_inhibited");
    assert.match(step.summary, /until the last Stop is acknowledged/);
  }
  assert.equal(view.fault, "rt_core_inhibited: acknowledged Stop and Release required");
  // The refusal blocks the path but the drives' own word is still fresh:
  // DISARMED, not unknown. The server's recovery text rides on the Arm step.
  assert.equal(view.arm.state, "disarmed");
  assert.equal(machineStatePill(view, noLegacy).label, "REAL · Test bench · DISARMED");
  const recovering = deriveMachineFlow(onBench({ status: status({ armed: null, error: "rt_core_inhibited: acknowledged Stop and Release required",
    flags: { inhibited: true, recoveryAction: "Retry POST /dense-execution/stop." } }) }));
  assert.match(recovering.steps[3]!.summary, /Retry POST \/dense-execution\/stop\.$/);
});

test("a Stop result is a note on Arm, not a lock: the operator arms again", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps(), status: status({ armed: null, error: "operator_stop" }) }));
  assert.deepEqual(states(view), ["choose:done", "connect:done", "home:done", "arm:current", "drive:pending"]);
  assert.match(view.steps[3]!.summary, /Last result: Stopped by the operator\. \[operator_stop\]/);
  assert.equal(view.canArm, true);
  const stale = deriveMachineFlow(input({ highlighted: bench, connected: true, connectedCell: bench.cell_id, connectedLabel: bench.label,
    posture: { backend: "ethercat", simulation: false }, capabilities: caps(), status: status({ armed: null, error: "control_session_stale" }) }));
  assert.equal(stale.steps[3]!.state, "current");
  assert.match(stale.steps[3]!.summary, /Arm again to continue/);
});

test("before the first status, Arm stays available and Home waits", () => {
  const view = deriveMachineFlow(input({ catalogue: null, connected: true, posture: { backend: "unknown" }, capabilities: caps({ rtCore: null }), status: null }));
  assert.deepEqual(states(view), ["choose:done", "connect:done", "home:pending", "arm:current", "drive:pending"]);
  assert.match(view.steps[2]!.summary, /Waiting for the machine's first status/);
  assert.equal(view.canArm, true);
});

test("unknown posture is treated as real: confirmation required, pill red", () => {
  const view = deriveMachineFlow(input({ catalogue: null, connected: true, posture: { backend: "unknown" }, capabilities: caps({ rtCore: null }), status: status() }));
  assert.equal(view.posture, "unknown");
  assert.equal(view.armConfirmRequired, true);
  assert.equal(view.steps[0]!.state, "done");
  assert.equal(machineStatePill(view, { executionMode: "simulation", robotLive: false, connectedArmed: false }).label, "OUTPUT UNKNOWN · bench · DISARMED");
});

test("a pending switch keeps the connected machine's steps and makes Connect current again", () => {
  const view = deriveMachineFlow(input({ highlighted: bench, connected: true, cellPending: true, connectedCell: local.cell_id, connectedLabel: local.label,
    posture: { backend: "simulation", simulation: true }, capabilities: caps(), status: status({ armed: true }) }));
  assert.equal(view.steps[0]!.state, "done");
  assert.equal(view.steps[1]!.state, "current");
  assert.match(view.steps[1]!.summary, /switch to Test bench/);
  assert.match(view.steps[1]!.summary, /Local simulation stays connected/);
  assert.equal(view.steps[3]!.state, "pending");
});

test("retired backend observations cannot enable motion", () => {
  const capabilities = normalizeDenseCapabilities({ backend: "legacy_v4", available: true });
  const legacyStatus = normalizeDenseStatus({ backend: "legacy_v4", available: true, armed: true });
  const view = deriveMachineFlow(input({ catalogue: null, connected: true, capabilities, status: legacyStatus }));
  assert.equal(capabilities.reason, "backend_retired");
  assert.equal(legacyStatus.daemonError, "backend_retired");
  assert.equal(view.canArm, false);
  assert.equal(view.canDrive, false);
});

test("production read-only legacy observation is the only non-machine pill besides NO MACHINE", () => {
  const view = deriveMachineFlow(input());
  const pill = machineStatePill(view, { executionMode: "production", robotLive: true, connectedArmed: true });
  assert.equal(pill.label, "CONNECTED · READ-ONLY · ARMED");
  assert.equal(machineStatePill(view, { executionMode: "production", robotLive: false, connectedArmed: true }).label, "NO MACHINE");
});

// The defect this guards: on the LA bench the header pill read ARMED while
// the Arm step read "arm state is unknown", the Drive step "Arm first" and
// the run row "busy". One fact, rendered by every surface, in each state.
test("one armed fact: pill, Arm headline, Drive gate and Jog caption agree in every state", () => {
  const cases: Array<{ name: string; input: MachineFlowInput; state: ArmState; token: string; headline: string; drive: RegExp; jog: RegExp }> = [
    { name: "disarmed", input: onBench({ status: status() }), state: "disarmed", token: "DISARMED", headline: "DISARMED", drive: /^Arm first\.$/, jog: /Arm it first \(step 4\)/ },
    { name: "arming (this panel's request)", input: onBench({ status: status(), inFlight: true, pendingRequest: "arm" }),
      state: "arming", token: "ARMING", headline: "ARMING…", drive: /Arming; Drive opens at ARMED/, jog: /Arming; wait for ARMED/ },
    { name: "arming (server busy after our Arm)", input: onBench({ status: status({ armed: null, error: "busy: Busy" }), lastRequest: { request: "arm", error: null } }),
      state: "arming", token: "ARMING", headline: "ARMING…", drive: /Arming; Drive opens at ARMED/, jog: /Arming; wait for ARMED/ },
    { name: "armed", input: onBench({ status: status({ armed: true }) }), state: "armed", token: "ARMED", headline: "ARMED", drive: /Motion is physical/, jog: /^$/ },
    { name: "stopping", input: onBench({ status: status({ armed: true }), inFlight: true, pendingRequest: "stop" }),
      state: "disarming", token: "STOPPING", headline: "STOPPING…", drive: /Stopping; Arm again afterwards/, jog: /Stopping; wait for DISARMED/ },
    { name: "disarming", input: onBench({ status: status({ armed: true }), inFlight: true, pendingRequest: "disarm" }),
      state: "disarming", token: "DISARMING", headline: "DISARMING…", drive: /Disarming; Arm again afterwards/, jog: /Disarming; wait for DISARMED/ },
    { name: "disarming (server busy after our Stop)", input: onBench({ status: status({ armed: null, motors: true, error: "busy: Busy" }), lastRequest: { request: "stop", error: null } }),
      state: "disarming", token: "STOPPING", headline: "STOPPING…", drive: /Stopping; Arm again afterwards/, jog: /Stopping; wait for DISARMED/ },
    { name: "unknown (no status yet)", input: onBench({ status: null }), state: "unknown", token: "STATUS STALE",
      headline: "UNKNOWN: no fresh status from the machine", drive: /UNKNOWN: no fresh status/, jog: /UNKNOWN: no fresh status/ },
    { name: "unknown (status older than the bound)", input: onBench({ status: status({ armed: true }), statusAgeMs: STATUS_FRESHNESS_MS + 1 }), state: "unknown", token: "STATUS STALE",
      headline: "UNKNOWN: no fresh status from the machine", drive: /UNKNOWN: no fresh status/, jog: /UNKNOWN: no fresh status/ },
    { name: "unknown (connection to rt-core lost)", input: onBench({ status: status({ armed: null, motors: true, error: "rt_core_transport_lost: EOF" }) }), state: "unknown", token: "STATUS STALE",
      headline: "UNKNOWN: no fresh status from the machine", drive: /until the last Stop|lost/, jog: /UNKNOWN: no fresh status/ },
  ];
  for (const c of cases) {
    const view = deriveMachineFlow(c.input);
    assert.equal(view.arm.state, c.state, c.name);
    assert.equal(view.arm.headline, c.headline, c.name);
    assert.equal(view.steps[3]!.headline, c.headline, `${c.name}: Arm step headline`);
    assert.equal(view.arm.token, c.token, c.name);
    const pill = machineStatePill(view, noLegacy);
    assert.equal(pill.label, `REAL · Test bench · ${c.token}`, `${c.name}: header pill`);
    assert.match(view.steps[4]!.summary, c.drive, `${c.name}: Drive step`);
    assert.equal(view.steps[4]!.state === "current", c.state === "armed", `${c.name}: Drive opens only on ARMED`);
    assert.equal(view.canDrive, c.state === "armed", `${c.name}: canDrive`);
    assert.match(view.arm.jogNote, c.jog, `${c.name}: Jog caption`);
    assert.equal(view.armed, c.state === "armed" ? true : c.state === "disarmed" ? false : null, `${c.name}: boolean gate`);
    assert.equal(view.stopEnabled, true, `${c.name}: STOP stays available`);
    // Only UNKNOWN may say stale, and UNKNOWN must never claim a side.
    assert.equal(pill.className.includes("is-stale"), c.state === "unknown", `${c.name}: stale class`);
    if (c.state === "unknown") assert.doesNotMatch(pill.label, /\bARMED\b|\bDISARMED\b/, `${c.name}: pill picks no side`);
    // A request in flight is a transition, never "Last result: busy".
    if (c.state === "arming" || c.state === "disarming") {
      assert.doesNotMatch(view.steps[3]!.summary, /Last result|unknown/i, `${c.name}: in flight is not a result`);
      assert.equal(view.canArm, false, `${c.name}: no second Arm while one is in flight`);
      assert.equal(view.canHome, false, `${c.name}: no Home while in flight`);
    }
  }
  // The pill is filled red while the drives are energised, edged otherwise.
  assert.equal(machineStatePill(deriveMachineFlow(cases[4]!.input), noLegacy).className, "is-real");
  assert.equal(machineStatePill(deriveMachineFlow(cases[6]!.input), noLegacy).className, "is-real");
  assert.equal(machineStatePill(deriveMachineFlow(cases[0]!.input), noLegacy).className, "is-real-idle");
  assert.equal(machineStatePill(deriveMachineFlow(cases[7]!.input), noLegacy).className, "is-real-idle is-stale");
  assert.match(machineStatePill(deriveMachineFlow(cases[7]!.input), noLegacy).title, /STOP stays available/);
});

test("a request that never reached the machine is a retry line, not a state change", () => {
  const brokenPipe = "write tcp 127.0.0.1:34492->127.0.0.1:8443: write: broken pipe";
  assert.equal(isTransportUnsent(brokenPipe), true);
  assert.equal(isTransportUnsent("rt_core_connection_reopened: dial tcp: connection refused"), true);
  assert.equal(isTransportUnsent("control_session_stale: cancelled Arm"), false);
  // The failed Arm from this panel: DISARMED stays DISARMED and the pill does not move.
  const failed = deriveMachineFlow(onBench({ status: status(), lastRequest: { request: "arm", error: brokenPipe } }));
  assert.equal(failed.arm.state, "disarmed");
  assert.equal(failed.arm.transportRetry, "Request did not reach the machine; retry Arm.");
  assert.equal(failed.steps[3]!.headline, "DISARMED");
  assert.match(failed.steps[3]!.summary, /Request did not reach the machine; retry Arm\./);
  assert.doesNotMatch(failed.steps[3]!.summary, /unknown|broken pipe/i);
  assert.equal(machineStatePill(failed, noLegacy).label, "REAL · Test bench · DISARMED");
  assert.equal(failed.canArm, true);
  // The same, reported by the server as its typed code in the status.
  const reopened = deriveMachineFlow(onBench({ status: status({ armed: null, error: `rt_core_connection_reopened: ${brokenPipe}` }), lastRequest: { request: "arm", error: null } }));
  assert.equal(reopened.arm.state, "disarmed");
  assert.match(reopened.steps[3]!.summary, /retry Arm\./);
  assert.doesNotMatch(reopened.steps[3]!.summary, /Last result/);
  // A background read's reconnect note cannot say the operator's request was unsent.
  const noted = deriveMachineFlow(onBench({ status: status({ armed: true, flags: { note: "rt_core_connection_reopened" } }), lastRequest: { request: "stop", error: null } }));
  assert.equal(noted.arm.state, "armed");
  assert.equal(noted.arm.transportRetry, null);
  assert.doesNotMatch(noted.steps[3]!.summary, /Request did not reach/);
  assert.equal(machineStatePill(noted, noLegacy).label, "REAL · Test bench · ARMED");
});

test("drives armed by another controller read ARMED but do not open Drive here", () => {
  const view = deriveMachineFlow(onBench({ status: status({ armed: null, motors: true, held: false }) }));
  assert.equal(view.arm.state, "armed");
  assert.equal(view.arm.controlled, false);
  assert.equal(view.steps[3]!.state, "current");
  assert.match(view.steps[3]!.summary, /Another controller holds the machine/);
  assert.equal(view.steps[4]!.state, "pending");
  assert.match(view.steps[4]!.summary, /Armed by another controller/);
  assert.equal(machineStatePill(view, noLegacy).label, "REAL · Test bench · ARMED");
  assert.match(view.arm.jogNote, /holds no grant/);
});

// the 1430 cell 2026-10-11 12:19: the server refused every Arm while the last Stop
// was unconfirmed. Arm now confirms it from the robot's state first (owner
// 2026-10-11), so the step stays available and says so; STOP stays the way
// out when the robot cannot show it.
test("an unconfirmed Stop leaves Arm available: Arm confirms it first", () => {
  const view = deriveMachineFlow(onBench({ status: status({ armed: null, error: "fault_persists: control_session_stale", flags: { stopUncertain: true } }) }));
  assert.equal(view.arm.state, "disarmed");
  assert.match(view.arm.detail, /last Stop outcome unconfirmed/);
  assert.equal(view.steps[3]!.state, "current");
  assert.match(view.steps[3]!.summary, /^The last Stop is unconfirmed\. Arm confirms it from the robot's state first .*press STOP\./);
  assert.equal(view.canArm, true);
  assert.equal(view.stopEnabled, true);
  assert.equal(view.armBlockers.length, 1);
  // The server could not confirm it: Arm's refusal keeps the standing cause.
  assert.match(armRefusalText(view.armBlockers, "rt_core_inhibited: The last Stop is still unconfirmed: the servo host still shows a lease or grant"), /^The last Stop is unconfirmed\..*\[rt_core_inhibited\]$/);
});

function driveAlarmStatus(flags: Partial<NonNullable<DenseStatus["rtCoreFlags"]>> = {}, recovery = "reset_after_condition_clears", rehome = 0): DenseStatus {
  const observed = status({ armed: null, error: "fault_persists", flags });
  const rtCore = observed.rtCore as unknown as { axes: { readiness: string }[]; operatorFeedback: { reset: { faults: unknown[] }; axes: { alarmText: string; alarmShort?: string }[] } };
  rtCore.axes[1]!.readiness = "faulted";
  rtCore.operatorFeedback.axes[1]!.alarmText = "J2 drive alarm Er41.1 (Motor over-temperature due to locked-rotor)";
  rtCore.operatorFeedback.axes[1]!.alarmShort = "J2 Er41.1";
  rtCore.operatorFeedback.reset.faults = [{ bit: 7, name: "drive_readiness", axis_mask: 2, recovery, outcome: "persists", rehome_axis_mask: rehome }];
  return observed;
}

// A drive alarm stays on screen until it clears, names the axis and the
// drive's code, and leaves Arm available: the server's Arm resets it first
// (owner 2026-10-11: "re-arming the robot should clear faults too if they
// exist"); a drive that still reports it refuses Arm with the server's words.
test("a drive alarm leaves Arm available and says Arm will reset it first", () => {
  const view = deriveMachineFlow(onBench({ status: driveAlarmStatus() }));
  assert.equal(view.steps[3]!.state, "current");
  assert.equal(view.canArm, true);
  assert.match(view.steps[3]!.summary, /^J2 drive alarm Er41\.1 \(Motor over-temperature due to locked-rotor\)\. Arm will reset J2 Er41\.1 first; if the drive still reports it, Arm is refused with the reason\. Check that joint/);
  assert.equal(view.armBlockers.length, 1);
  assert.equal(armRefusalText(view.armBlockers, "fault_persists: J2 drive alarm Er41.1 (Motor over-temperature due to locked-rotor) still active — let it cool, then Arm again"),
    "J2 drive alarm Er41.1 (Motor over-temperature due to locked-rotor) still active — let it cool, then Arm again [fault_persists]");
  assert.equal(armRefusalText(view.armBlockers, "fault_not_resettable: J1 coordinate_reference needs a new Home at the mechanical home pose (Arm never re-homes), not a reset; the robot stays disarmed"),
    "J1 coordinate_reference needs a new Home at the mechanical home pose (Arm never re-homes), not a reset; the robot stays disarmed [fault_not_resettable]");
  // A fault that needs a new Home or a restart: no promise of a reset.
  for (const [recovery, rehome] of [["rehome_required", 2], ["reset_after_condition_clears", 2], ["restart_required", 0]] as const) {
    const unpromised = deriveMachineFlow(onBench({ status: driveAlarmStatus({}, recovery, rehome) }));
    assert.doesNotMatch(unpromised.armBlockers.join(" "), /Arm will reset/, `${recovery} rehome=${rehome}`);
    assert.match(unpromised.armBlockers[0]!, /^J2 drive alarm Er41\.1 .*Check that joint/);
  }
  // With the Stop also unconfirmed both causes stay listed and Arm stays available.
  const both = deriveMachineFlow(onBench({ status: driveAlarmStatus({ stopUncertain: true }) }));
  assert.equal(both.canArm, true);
  assert.equal(both.armBlockers.length, 2);
  assert.match(both.steps[3]!.summary, /Arm confirms it from the robot's state first.*Arm will reset J2 Er41\.1 first/);
  // A cleared alarm (axis ready again) leaves nothing listed.
  const cleared = driveAlarmStatus();
  (cleared.rtCore as unknown as { axes: { readiness: string }[] }).axes[1]!.readiness = "ready";
  assert.deepEqual(armBlockers(cleared), []);
  assert.equal(armRefusalText([], "rt_core_inhibited"), machineReasonText("rt_core_inhibited"));
  assert.deepEqual(armBlockers(status(), "Update in progress"), ["Maintenance: Update in progress. Arming is refused until it ends."]);
});

test("machineReasonText keeps the code and falls back to the rt-core reason table", () => {
  assert.match(machineReasonText("session_principal_mismatch"), /not issued for the cell's pair.*\[session_principal_mismatch\]$/);
  assert.match(machineReasonText("target_changed: selection generation 4"), /\[target_changed\]/);
  assert.equal(machineReasonText("jog_not_ready"), "jog_not_ready: Jog not ready; clear fault or Stop and re-arm");
  // Load and Play refusals the Steam Deck showed as "Release input; inspect
  // public status before retrying" on simulated the 1430 cell (2026-10-11).
  assert.match(machineReasonText("play_requires_arm"), /^Arm the robot to play\..*\[play_requires_arm\]$/);
  assert.match(machineReasonText("planning_start_stale"), /Plan again from the robot's current position.*\[planning_start_stale\]$/);
  assert.match(machineReasonText("dense_blob_not_found"), /Plan again\. \[dense_blob_not_found\]$/);
  // A hidden pendant on steamdeck-pendant armed the 1430 cell from the controller (2026-10-11).
  assert.match(machineReasonText("client_not_focused"), /^This pendant was in the background.*Stop always works\. \[client_not_focused\]$/);
  assert.match(machineReasonText("client_cell_mismatch"), /started for another cell.*Nothing was sent.*\[client_cell_mismatch\]$/);
  assert.match(machineReasonText("rt_core status unavailable"), /has not reported a valid status/);
  assert.equal(machineReasonText(""), "");
  // The HTTP pair is `{error: "rt_core_event", detail: "rt_core_event: fault_cleared"}`;
  // DenseExecutionError joins them, and rtCoreReasonText would prefix the code again.
  assert.equal(machineReasonText("rt_core_event: rt_core_event: fault_cleared"), "rt_core_event: fault_cleared");
  assert.equal(machineReasonText("rt_core_event: fault_cleared"), "rt_core_event: fault_cleared");
});

test("latched controller events explain an aborted run and preserve the diagnostic code", () => {
  for (const reason of ["rt_core_event: fault_latched", "rt_core_event: rt_core_event: fault_latched"]) {
    assert.equal(machineReasonText(reason), `The controller stopped the run because a fault was latched. [${reason}]`);
    assert.doesNotMatch(machineReasonText(reason), /J5|Home|Reset/, "the event alone cannot identify a joint or authorize recovery");
  }
  assert.equal(machineReasonText("rt_core_event: future_event"), "rt_core_event: future_event");
  assert.equal(machineReasonText("rt_core_event: fault_latched: future_detail"), "rt_core_event: fault_latched: future_detail");
});


test("restart-required fault blocks Home while reference recovery remains available", () => {
  const observed = status({ home: [false, false], held: false });
  observed.rtCore!.operatorFeedback.reset.faults = [{ bit: 11, name: "pdo_mapping_mismatch", axis_mask: 8, recovery: "restart_required", outcome: "persists", rehome_axis_mask: 8 }];
  const selected = input({ connected: true, capabilities: caps(), status: observed });
  const blocked = deriveMachineFlow(selected);
  assert.equal(blocked.canHome, false);
  assert.match(blocked.steps.find(step => step.id === "home")!.summary, /pdo_mapping_mismatch.*restart/);
  observed.rtCore!.operatorFeedback.reset.faults = [{ bit: 4, name: "encoder_reference", axis_mask: 1, recovery: "rehome", outcome: "persists", rehome_axis_mask: 1 }];
  assert.equal(deriveMachineFlow(selected).canHome, true);
});


test("current restart-required fault outranks Stop history without enabling recovery", () => {
  const raw = {
    time_ns: 100, adapter_incarnation: "adapter", daemon_incarnation: "daemon",
    execution: { state: "completed", error: "operator_stop" },
    core: { armed: 0, safety_fault_mask: 8, execution_fault_reasons: 2048,
      home_valid_mask: 63, commissioning_phase: 0, native_home_active_axis_mask: 0,
      recovery: { faults: [{ name: "pdo_mapping_mismatch", axis_mask: 8, recovery: "restart_required" }] } },
    axes: Array.from({ length: 6 }, () => ({ home_valid: true, logical_valid: true, readiness: "not_ready" })),
  };
  for (const reason of ["operator_stop", "disarm", ""]) {
    const normalized = normalizeDenseStatus({ backend: "rt_core", available: true, rt_core: { status: raw, reason } });
    const view = deriveMachineFlow(onBench({ status: normalized }));
    assert.equal(view.fault, "rt_core_fault: 8");
    assert.equal(isMachineFault(view.fault!), true, "top badge must expose FAULT");
    assert.equal(view.arm.state, "disarmed", "retain the actual drive state");
    assert.equal(view.canArm, false);
    assert.equal(view.canHome, false);
    assert.equal(view.canDrive, false);
    assert.match(view.steps[3]!.summary, /pdo_mapping_mismatch.*requires controller restart/);
    assert.equal(normalized.rtCore?.operatorFeedback.reset.admissible, false);
  }
  // A fault-free acknowledged Stop still permits explicit re-arm, while a
  // transport-loss report keeps the existing unknown-state gate.
  raw.core.safety_fault_mask = 0;
  raw.core.execution_fault_reasons = 0;
  raw.core.recovery.faults = [];
  const stopped = normalizeDenseStatus({ backend: "rt_core", available: true, rt_core: { status: raw, reason: "operator_stop" } });
  assert.equal(stopped.daemonError, "operator_stop");
  assert.equal(deriveMachineFlow(onBench({ status: stopped })).canArm, true);
  raw.core.safety_fault_mask = 8;
  const lost = normalizeDenseStatus({ backend: "rt_core", available: true, rt_core: { status: raw, reason: "rt_core_transport_lost" } });
  assert.equal(lost.daemonError, "rt_core_transport_lost");
  assert.equal(deriveMachineFlow(onBench({ status: lost })).arm.state, "unknown");
});
