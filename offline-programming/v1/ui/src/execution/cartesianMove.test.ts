import assert from "node:assert/strict";
import test from "node:test";
import { CARTESIAN_INPUT_INVALID, cartesianDefaultMode, cartesianHoldTwist, cartesianMoveBody, cartesianStepLabels, readCartesianSteps, writeCartesianSteps } from "./cartesianMove.ts";
import { cartesianJogAPI, MachineCartesianJog } from "./cartesianJog.ts";
import { JointAngleMove, JointJogError } from "./jointJog.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { setImmediate } from "node:timers/promises";

test("Cartesian inputs default to 100 mm and 15 degrees and persist per browser", () => {
  let saved = "";
  const storage = { getItem: () => saved, setItem: (_key: string, value: string) => { saved = value; } };
  assert.deepEqual(readCartesianSteps(storage), { distanceMM: 100, angleDegrees: 15 });
  assert.equal(writeCartesianSteps({ distanceMM: 25, angleDegrees: 7.5 }, storage), true);
  assert.deepEqual(readCartesianSteps(storage), { distanceMM: 25, angleDegrees: 7.5 });
  assert.equal(writeCartesianSteps({ distanceMM: -1, angleDegrees: 15 }, storage), false);
  saved = '{"distanceMM":0,"angleDegrees":15}';
  assert.deepEqual(readCartesianSteps(storage), { distanceMM: 100, angleDegrees: 15 });
});

test("Hold/Step defaults use the absolute LAN round-trip bound and fail closed", () => {
  for (const rtt of [0, 10, 20]) assert.equal(cartesianDefaultMode(rtt), "hold");
  for (const rtt of [20.001, 250, undefined, NaN, -1]) assert.equal(cartesianDefaultMode(rtt), "step");
});

test("Twelve Cartesian directions build exactly one displacement with server-owned origin", () => {
  for (let axis = 0; axis < 6; axis++) for (const direction of [-1, 1]) {
    const body = cartesianMoveBody({ target_id: "bench", session_id: "s", revision: 2, axis, degrees: direction*(axis < 3 ? 100 : 15), fraction: .25 }, "tool");
    assert.equal(body.distance_mm, axis < 3 ? direction*100 : 0);
    assert.equal(body.angle_degrees, axis >= 3 ? direction*15 : 0);
    assert.equal(body.frame, "tool"); assert.equal(body.fraction, .25);
    assert.equal("pose" in body, false);
  }
  for (const amount of [0, NaN, Infinity, 1001]) {
    assert.throws(() => cartesianMoveBody({ target_id: "b", session_id: "s", revision: 0, axis: 0, degrees: amount, fraction: .1 }, "base"),
      error => error instanceof JointJogError && error.code === "cartesian_input_invalid" && error.message === CARTESIAN_INPUT_INVALID);
  }
});

test("Hold twist for 100 mm/s and 15 deg/s is 0.1 m/s and 0.2617993877991494 rad/s at every Speed %; the resolver applies the fraction", () => {
  const steps = { distanceMM: 100, angleDegrees: 15 };
  assert.deepEqual(cartesianHoldTwist(1, 1, steps, 1), [0, 0.1, 0, 0, 0, 0]);
  assert.deepEqual(cartesianHoldTwist(4, 1, steps, 1), [0, 0, 0, 0, 0.2617993877991494, 0]);
  assert.deepEqual(cartesianHoldTwist(1, -1, steps, 1), [0, -0.1, 0, 0, 0, 0]);
  assert.deepEqual(cartesianHoldTwist(4, -1, steps, 1), [0, 0, 0, 0, -0.2617993877991494, 0]);
  // The wire carries the box speed unscaled at any fraction; the resolver
  // multiplies by the fraction, so 100 mm/s at 25 % becomes 25 mm/s there.
  assert.deepEqual(cartesianHoldTwist(1, 1, steps, 0.25), [0, 0.1, 0, 0, 0, 0]);
  assert.deepEqual(cartesianHoldTwist(4, 1, steps, 0.25), [0, 0, 0, 0, 0.2617993877991494, 0]);
  assert.throws(() => cartesianHoldTwist(1, 1, steps, 0), /cartesian_input_invalid|Enter a distance/);
  assert.throws(() => cartesianHoldTwist(1, 1, steps, 1.01), /cartesian_input_invalid|Enter a distance/);
});

test("Step still sends 100 mm on axis 0 and 15 degrees on axis 3 from the same boxes", () => {
  const steps = { distanceMM: 100, angleDegrees: 15 };
  const linear = cartesianMoveBody({ target_id: "bench", session_id: "s", revision: 2, axis: 0, degrees: steps.distanceMM, fraction: .25 }, "base");
  assert.equal(linear.distance_mm, 100);
  assert.equal(linear.angle_degrees, 0);
  assert.equal(linear.fraction, .25);
  const angular = cartesianMoveBody({ target_id: "bench", session_id: "s", revision: 2, axis: 3, degrees: steps.angleDegrees, fraction: .25 }, "base");
  assert.equal(angular.distance_mm, 0);
  assert.equal(angular.angle_degrees, 15);
  assert.equal(angular.fraction, .25);
});

test("Cartesian box labels follow Hold and Step and name the unit", () => {
  const hold = cartesianStepLabels("hold");
  assert.equal(hold.linear, "Speed at 100% (mm/s)");
  assert.equal(hold.angular, "Rate at 100% (°/s)");
  assert.match(hold.help, /100 mm\/s at 25% is 25 mm\/s/);
  assert.match(hold.help, /Speed % of its described maximum/);
  const step = cartesianStepLabels("step");
  assert.equal(step.linear, "Distance (mm)");
  assert.equal(step.angular, "Angle (degrees)");
  assert.match(step.help, /distance or angle per press/);
});

test("Cartesian boxes refuse zero, negative, NaN and out-of-range values in both modes", () => {
  const invalid = [
    { distanceMM: 0, angleDegrees: 15 },
    { distanceMM: -1, angleDegrees: 15 },
    { distanceMM: NaN, angleDegrees: 15 },
    { distanceMM: 1001, angleDegrees: 15 },
    { distanceMM: 100, angleDegrees: 0 },
    { distanceMM: 100, angleDegrees: -1 },
    { distanceMM: 100, angleDegrees: 181 },
    { distanceMM: 100, angleDegrees: NaN },
  ];
  const storage = { setItem() { throw new Error("must not persist invalid Cartesian steps"); } };
  for (const steps of invalid) {
    assert.equal(writeCartesianSteps(steps, storage), false);
    assert.throws(() => cartesianHoldTwist(1, 1, steps, 1),
      error => error instanceof JointJogError && error.code === "cartesian_input_invalid" && error.message === CARTESIAN_INPUT_INVALID);
  }
});

test("Cartesian press sends one bounded move and reports its correlated completion", async () => {
  const calls: { url: string; init: RequestInit }[] = [];
  const api = cartesianJogAPI(() => "base", (async (url, init) => {
    calls.push({ url: String(url), init: init! });
    return new Response(JSON.stringify(String(url).endsWith("state") ? { active: false, revision: 4 } : { axis: 0, state: "done", revision: 5 }));
  }) as typeof fetch);
  const move = new JointAngleMove(api, () => {});
  await move.begin("bench", "s", 0, 100, .1, { cell_id: "bench", selection_generation: "7", model_id: "rosie_1420_v1", label: "Bench", host: "localhost", backend: "simulation" });
  assert.equal(move.state.state, "done");
  assert.deepEqual(calls.map(c => c.init.method), ["GET", "POST"]);
  assert.ok(calls[1]!.url.endsWith("/cartesian/move"));
  assert.equal((calls[1]!.init.headers as Record<string, string>)["X-RT-Target-Generation"], "7");
  assert.equal(JSON.parse(String(calls[1]!.init.body)).distance_mm, 100);
});

test("Hold/Step toggle preserves a step across pointer release and Stop cancels it", async t => {
  t.mock.timers.enable({ apis: ["setInterval"] });
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const calls: string[] = [];
  let entered!: () => void;
  const moving = new Promise<void>(resolve => { entered = resolve; });
  let signal: AbortSignal | null | undefined;
  const machine = new MachineCartesianJog(() => {}, (async (url, init) => {
    calls.push(String(url));
    if (String(url).endsWith("/state")) return Response.json({ revision: 2, active: false });
    signal = init?.signal; entered();
    return new Promise<Response>((_resolve, reject) => signal?.addEventListener("abort", () => reject(new Error("cancelled"))));
  }) as typeof fetch);
  machine.update(true, { backend: "rt_core", rtCore: {}, daemonReachable: true, cell: "bench" } as DenseCapabilities,
    { armed: true, rtCore: { ready: true }, roundTripMS: 250, target: { cell_id: "bench", selection_generation: "2" }, session: { id: "s", leaseHeld: true, state: "idle" } } as DenseStatus, false);
  assert.equal(machine.mode, "step");
  machine.setMode("hold"); assert.equal(machine.mode, "hold");
  machine.setMode("step");
  machine.begin("base", [1,0,0,0,0,0], .1, { distanceMM: 100, angleDegrees: 15 });
  await moving;
  window.dispatchEvent(new Event("pointerup"));
  t.mock.timers.tick(1000);
  assert.equal(machine.busy, true); assert.equal(signal?.aborted, false);
  assert.equal(calls.filter(url => url.endsWith("/move")).length, 1);
  assert.equal(calls.length, 2);
  machine.stop("stop"); assert.equal(signal?.aborted, true);
  assert.equal(signal?.reason, "stop");
  assert.equal(machine.reason, "Move cancelled: STOP pressed");
});

for (const cause of ["done", "selection_changed", "cell_changed", "session_changed", "disarm"] as const) {
  test(`Cartesian step survives incomplete status and records ${cause}`, async t => {
    for (const [name, value] of Object.entries({ window: new EventTarget(), document: Object.assign(new EventTarget(), { hidden: true }) })) {
      const previous = Object.getOwnPropertyDescriptor(globalThis, name);
      Object.defineProperty(globalThis, name, { configurable: true, value });
      t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
    }
    let entered!: () => void, complete!: (value: Response) => void;
    const moving = new Promise<void>(resolve => { entered = resolve; });
    let signal: AbortSignal | null | undefined, pollFailure = "";
    const calls: string[] = [];
    const machine = new MachineCartesianJog(() => {}, (async (url, init) => {
      calls.push(String(url));
      if (String(url).endsWith("/state")) {
        if (pollFailure === "failed") throw new TypeError("Failed to fetch");
        if (pollFailure) return Response.json({ error: pollFailure }, { status: 409 });
        return Response.json({ revision: 2, active: false });
      }
      signal = init?.signal; entered();
      return new Promise<Response>((resolve, reject) => {
        complete = resolve;
        signal?.addEventListener("abort", () => reject(new Error(String(signal?.reason))));
      });
    }) as typeof fetch);
    const caps = { backend: "rt_core", rtCore: {}, daemonReachable: true, cell: "bench" } as DenseCapabilities;
    const status = { armed: true, rtCore: { ready: true }, roundTripMS: 240, target: { cell_id: "bench", selection_generation: "2" }, session: { id: "s", leaseHeld: true, state: "idle" } } as DenseStatus;
    machine.update(true, caps, status, false);
    machine.begin("base", [-1, 0, 0, 0, 0, 0], .05);
    await moving;
    for (const observation of [{ ...status, target: undefined }, { ...status, session: null }, null,
      { ...status, rtCore: null, armed: null, daemonError: "rt_core_status_slow" }]) {
      machine.update(true, caps, observation, true);
      assert.equal(signal?.aborted, false);
      assert.equal(machine.busy, true);
      assert.equal(machine.reason, "waiting for the machine");
    }
    machine.update(false, null, null, false);
    assert.equal(signal?.aborted, false);
    machine.update(true, caps, status, false, false);
    assert.equal(machine.reason, "waiting for the machine");
    for (const failure of ["failed", "rt_core_status_slow"]) {
      pollFailure = failure; await machine.poll();
      assert.equal(signal?.aborted, false);
      assert.equal(machine.reason, "waiting for the machine");
    }
    for (const event of ["blur", "pagehide", "pointerup", "pointercancel"]) window.dispatchEvent(new Event(event));
    document.dispatchEvent(new Event("visibilitychange"));
    machine.update(true, caps, status, false);
    assert.equal(signal?.aborted, false);
    assert.equal(machine.reason, "moving");
    if (cause === "done") {
      complete(Response.json({ axis: 0, state: "done", reason: "", revision: 3 }));
      await setImmediate();
      assert.equal(machine.reason, "done");
      assert.equal(signal?.aborted, false);
    } else {
      if (cause === "disarm") machine.stop("disarm");
      else if (cause === "session_changed") machine.update(true, caps, { ...status, session: { ...status.session!, id: "replacement" } }, false);
      else machine.update(true, caps, { ...status, target: { ...status.target!, ...(cause === "cell_changed" ? { cell_id: "other" } : { selection_generation: "3" }) } }, false);
      assert.equal(signal?.aborted, true);
      assert.equal(signal?.reason, cause === "cell_changed" ? "selection_changed" : cause);
      assert.equal(machine.reason, cause === "disarm" ? "Move cancelled: DISARM pressed" : cause === "session_changed" ? "Move cancelled: machine session changed" : "Move cancelled: selection changed in another window");
      await setImmediate();
      assert.doesNotMatch(machine.reason, /Stopped by operator|done/);
    }
    assert.equal(machine.busy, false);
    assert.equal(calls.filter(url => url.endsWith("/move")).length, 1);
    assert.equal(calls.filter(url => url.endsWith("/halt")).length, 0);
  });
}

test("Machine Cartesian Hold and Step use the same boxes in millimetres, degrees and SI twist", async t => {
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const bodies: { url: string; body: Record<string, unknown> }[] = [];
  const machine = new MachineCartesianJog(() => {}, (async (url, init) => {
    bodies.push({ url: String(url), body: init?.body ? JSON.parse(String(init.body)) : {} });
    const path = String(url);
    if (path.endsWith("/move")) return Response.json({ axis: JSON.parse(String(init?.body)).axis, state: "done", revision: 3 });
    if (path.endsWith("/state") || path.endsWith("/halt")) return Response.json({ active: false, revision: 2 });
    return Response.json({ active: true, revision: 2 });
  }) as typeof fetch);
  const caps = { backend: "rt_core", rtCore: {}, daemonReachable: true, cell: "bench" } as DenseCapabilities;
  const status = { armed: true, rtCore: { ready: true }, roundTripMS: 1, target: { cell_id: "bench", selection_generation: "2" }, session: { id: "s", leaseHeld: true, state: "idle" } } as DenseStatus;
  machine.update(true, caps, status, false);
  const steps = { distanceMM: 100, angleDegrees: 15 };
  machine.setMode("hold");
  machine.begin("base", [0, 1, 0, 0, 0, 0], .25, steps);
  await setImmediate();
  const holdLinear = bodies.find(call => call.url.endsWith("/start"))!.body;
  assert.deepEqual(holdLinear.twist, [0, 0.1, 0, 0, 0, 0], "the box speed travels unscaled; the resolver applies the fraction");
  assert.equal(holdLinear.fraction, .25);
  machine.end();
  await setImmediate();
  bodies.length = 0;
  machine.begin("base", [0, 0, 0, 0, 1, 0], .25, steps);
  await setImmediate();
  assert.deepEqual(bodies.find(call => call.url.endsWith("/start"))!.body.twist, [0, 0, 0, 0, 0.2617993877991494, 0]);
  machine.end();
  await setImmediate();
  bodies.length = 0;
  machine.setMode("step");
  machine.begin("base", [1, 0, 0, 0, 0, 0], .25, steps);
  await setImmediate();
  const stepLinear = bodies.find(call => call.url.endsWith("/move"))!.body;
  assert.equal(stepLinear.distance_mm, 100);
  assert.equal(stepLinear.angle_degrees, 0);
  assert.equal(stepLinear.fraction, .25);
  bodies.length = 0;
  machine.begin("base", [0, 0, 0, 1, 0, 0], .25, steps);
  await setImmediate();
  const stepAngular = bodies.find(call => call.url.endsWith("/move"))!.body;
  assert.equal(stepAngular.distance_mm, 0);
  assert.equal(stepAngular.angle_degrees, 15);
  assert.equal(stepAngular.fraction, .25);
});

test("Invalid Cartesian boxes are refused before a hold or step is sent", async t => {
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  let requests = 0;
  const machine = new MachineCartesianJog(() => {}, async () => { requests++; return Response.json({ active: false, revision: 0 }); });
  const caps = { backend: "rt_core", rtCore: {}, daemonReachable: true, cell: "bench" } as DenseCapabilities;
  const status = { armed: true, rtCore: { ready: true }, roundTripMS: 1, target: { cell_id: "bench", selection_generation: "2" }, session: { id: "s", leaseHeld: true, state: "idle" } } as DenseStatus;
  machine.update(true, caps, status, false);
  for (const mode of ["hold", "step"] as const) {
    machine.setMode(mode);
    machine.begin("base", [1, 0, 0, 0, 0, 0], .1, { distanceMM: 0, angleDegrees: 15 });
    assert.equal(machine.reason, CARTESIAN_INPUT_INVALID);
    machine.begin("base", [0, 0, 0, 1, 0, 0], .1, { distanceMM: 100, angleDegrees: 181 });
    assert.equal(machine.reason, CARTESIAN_INPUT_INVALID);
  }
  assert.equal(requests, 0);
});

test("Cartesian state restores the completion after reconnecting", async t => {
  for (const name of ["window", "document"]) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value: new EventTarget() });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  const machine = new MachineCartesianJog(() => {}, (async () => Response.json({ active: false, revision: 7, move: { axis: 0, state: "done", reason: "", revision: 7 } })) as typeof fetch);
  machine.update(true, null, null, false);
  await machine.poll();
  assert.equal(machine.reason, "done");
});

test("Cartesian hold: a press while the previous path halt is still in flight is sent, not refused locally", async t => {
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  // OLP's path End returns only after the stop completes; the server holds
  // the next press until then (1de38926), so the pad must send it.
  const calls: string[] = [];
  let revision = 2, halted!: () => void;
  const halt = new Promise<void>(resolve => { halted = resolve; });
  const machine = new MachineCartesianJog(() => {}, (async (url: string, init?: RequestInit) => {
    const route = String(url).split("/").at(-1)!;
    calls.push(route);
    if (route === "state") return Response.json({ revision, active: false });
    if (route === "halt") { revision++; await halt; return Response.json({ revision, active: false }); }
    const body = JSON.parse(String(init?.body));
    return Response.json({ revision: body.revision, active: true });
  }) as typeof fetch);
  machine.update(true, { backend: "rt_core", rtCore: {}, daemonReachable: true, cell: "bench" } as DenseCapabilities,
    { armed: true, rtCore: { ready: true }, roundTripMS: 10, target: { cell_id: "bench", selection_generation: "2" }, session: { id: "s", leaseHeld: true, state: "idle" } } as DenseStatus, false);
  assert.equal(machine.mode, "hold");
  machine.begin("base", [1, 0, 0, 0, 0, 0], .5);
  await setImmediate(); await setImmediate();
  machine.end();
  await setImmediate();
  assert.equal(machine.busy, true, "the halt is in flight");
  machine.begin("base", [-1, 0, 0, 0, 0, 0], .5);
  await setImmediate(); await setImmediate();
  assert.deepEqual(calls.filter(c => c !== "intent"), ["state", "start", "halt", "state", "start"]);
  assert.equal(machine.reason, "");
  halted(); await setImmediate();
  machine.end(); await setImmediate();
});
