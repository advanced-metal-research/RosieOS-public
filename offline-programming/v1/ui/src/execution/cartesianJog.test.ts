import assert from "node:assert/strict";
import test from "node:test";
import { cartesianJogAPI, cartesianPadPresentation, MachineCartesianJog } from "./cartesianJog.ts";
import { JointJogError, JointJogHold } from "./jointJog.ts";
import type { MachineFlowView } from "./machineFlow.ts";
import type { DenseCapabilities, DenseStatus } from "./denseClient.ts";
import { rtCoreReasonText } from "../targets/rtCoreStatus.ts";

test("Hold twist uses the operator boxes as mm/s and deg/s, unscaled; the fraction rides beside it", async () => {
  const bodies: Record<string, unknown>[] = [];
  const api = cartesianJogAPI(() => "base", (async (_url, init) => {
    if (init?.body) bodies.push(JSON.parse(String(init.body)));
    return Response.json({ active: true, revision: 1 });
  }) as typeof fetch, () => ({ distanceMM: 100, angleDegrees: 15 }));
  const intent = { target_id: "bench", session_id: "s", revision: 1, axis: 1, direction: 1, fraction: 0.25 };
  await api.send("begin", intent);
  assert.deepEqual(bodies[0], { target_id: "bench", session_id: "s", revision: 1, frame: "base", twist: [0, 0.1, 0, 0, 0, 0], fraction: 0.25 });
  bodies.length = 0;
  await api.send("begin", { ...intent, axis: 4 });
  assert.deepEqual(bodies[0]!.twist, [0, 0, 0, 0, 0.2617993877991494, 0]);
  assert.equal(bodies[0]!.fraction, 0.25);
  bodies.length = 0;
  await api.send("begin", { ...intent, direction: -1 });
  assert.deepEqual(bodies[0]!.twist, [0, -0.1, 0, 0, 0, 0]);
  assert.equal(bodies[0]!.fraction, 0.25);
});

test("Cartesian requests pin the machine and preserve TOOL twist through release", async () => {
  const calls: { url: string; init: RequestInit }[] = [];
  const api = cartesianJogAPI(() => "tool", (async (url, init) => {
    calls.push({ url: String(url), init: init! });
    return new Response(JSON.stringify({ active: true, revision: 7 }));
  }) as typeof fetch);
  const target = { cell_id: "bench", selection_generation: "19" } as DenseStatus["target"];
  const intent = { target_id: "bench", session_id: "session", revision: 7, axis: 0, direction: 1, fraction: .1 };
  await api.state(target);
  for (const action of ["begin", "update", "end"] as const) await api.send(action, intent, target);
  assert.deepEqual(calls.map(c => c.url.split("/").slice(-2).join("/")), ["cartesian/state", "cartesian/start", "cartesian/intent", "cartesian/halt"]);
  for (const call of calls) {
    assert.equal((call.init.headers as Record<string, string>)["X-RT-Target-Cell"], "bench");
    assert.equal((call.init.headers as Record<string, string>)["X-RT-Target-Generation"], "19");
    assert.equal(call.init.redirect, "error");
  }
  assert.deepEqual(JSON.parse(String(calls[2]!.init.body)), { target_id: "bench", session_id: "session", revision: 7, frame: "tool", twist: [.1, 0, 0, 0, 0, 0], fraction: .1 });
  assert.equal(calls[3]!.init.keepalive, true);
});

test("Cartesian refusal is typed and never retries on the preview route", async () => {
  const paths: string[] = [];
  const api = cartesianJogAPI(() => "base", (async url => {
    paths.push(String(url));
    return new Response(JSON.stringify({ error: "jacobian_gate", detail: "Singular pose" }), { status: 409 });
  }) as typeof fetch);
  await assert.rejects(api.send("update", { target_id: "bench", session_id: "s", revision: 0, axis: 4, direction: -1, fraction: .1 }), error => error instanceof JointJogError && error.code === "jacobian_gate");
  assert.equal(paths.length, 1);
  assert.ok(paths[0]!.endsWith("/dense-execution/cartesian/intent"));
  assert.match(rtCoreReasonText("jacobian_gate"), /singular pose/);
});

test("Cartesian hold release sends one halt with its original identity", async () => {
  const paths: string[] = [];
  const identities: string[] = [];
  const api = cartesianJogAPI(() => "base", (async (url, init) => {
    paths.push(String(url)); identities.push((init!.headers as Record<string, string>)["X-RT-Target-Cell"]!);
    return new Response(JSON.stringify({ active: false, revision: 0 }));
  }) as typeof fetch);
  const hold = new JointJogHold(api, () => {}, { every: () => 1 as unknown as ReturnType<typeof setInterval>, clear: () => {} });
  const target = { cell_id: "first", selection_generation: "2" } as NonNullable<DenseStatus["target"]>;
  await hold.begin("first", "s", 0, 1, .1, target);
  target.cell_id = "second";
  await hold.end(); await hold.end();
  assert.equal(paths.filter(p => p.endsWith("/halt")).length, 1);
  assert.deepEqual(identities, ["first", "first", "first"]);
});

test("A path hold changes speed within the hold: the next intent carries the new fraction on the same line", async () => {
  const bodies: Record<string, unknown>[] = [];
  const api = cartesianJogAPI(() => "base", (async (url, init) => {
    if (init?.body) bodies.push({ route: String(url).split("/").pop(), ...JSON.parse(String(init.body)) });
    return Response.json({ active: false, revision: 3, fraction_updates: true });
  }) as typeof fetch, () => ({ distanceMM: 100, angleDegrees: 15 }));
  let tick = () => {};
  const hold = new JointJogHold(api, () => {}, { every: fn => { tick = fn; return 1 as unknown as ReturnType<typeof setInterval>; }, clear: () => {} });
  await hold.begin("bench", "s", 0, 1, .25);
  assert.equal(hold.setFraction(.75), true);
  tick(); await new Promise(resolve => setTimeout(resolve, 0));
  assert.deepEqual(bodies.map(b => [b.route, b.fraction, (b.twist as number[])[0]]), [["start", .25, .1], ["intent", .75, .1]]);
  assert.equal(hold.setFraction(1.5), false);
  assert.equal(hold.setFraction(0), false);
  await hold.end();
  assert.equal(hold.setFraction(.5), false);
  assert.match(rtCoreReasonText("cartesian_hold_unavailable"), /path jog/);
  assert.match(rtCoreReasonText("path_buffer_underrun"), /stopped on the line/);
});

test("Cartesian heading, tag and preview arming follow connection state", () => {
  assert.deepEqual(cartesianPadPresentation(null), { tag: "BROWSER PREVIEW", heading: "Drives the browser preview. Arm preview to jog.", previewArmVisible: true });
  for (const posture of ["real", "simulation", "unknown"] as const) {
    const view = { connected: true, cellLabel: "LA bench", posture, arm: { jogNote: "Arm it first (step 4)." } } as MachineFlowView;
    const pad = cartesianPadPresentation(view);
    assert.equal(pad.tag, "SELECTED MACHINE");
    assert.equal(pad.previewArmVisible, false);
    assert.match(pad.heading, /Drives the selected machine, LA bench/);
    assert.match(pad.heading, /Arm it first \(step 4\)/);
    assert.ok(pad.heading.includes(posture === "simulation" ? "simulation, no physical output" : posture === "real" ? "REAL, motion is physical" : "output unknown, treated as real"));
  }
});

test("bench Cartesian section stays disabled before and after Connect and Arm", t => {
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  let requests = 0;
  const machine = new MachineCartesianJog(() => {}, async () => { requests++; return Response.json({}); });
  for (const model of ["bench_one_motor_1to1", "bench_nine_motors_1to1"] as const) {
    for (const connected of [false, true]) {
      const view = { modelId: model, connected } as MachineFlowView;
      assert.deepEqual(cartesianPadPresentation(view), { tag: "BENCH CELL", heading: "Not available on a bench without an arm model", previewArmVisible: false });
      machine.update(connected, null, { armed: true } as DenseStatus, false, true, model);
      assert.equal(machine.disabled, "Not available on a bench without an arm model");
      machine.begin("base", [1, 0, 0, 0, 0, 0], .1);
    }
  }
  assert.equal(requests, 0);
});

test("unknown picker or status models block ready Cartesian steps and holds without requests", t => {
  for (const [name, value] of Object.entries({ window: new EventTarget(), document: new EventTarget() })) {
    const previous = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => previous ? Object.defineProperty(globalThis, name, previous) : Reflect.deleteProperty(globalThis, name));
  }
  let requests = 0;
  const machine = new MachineCartesianJog(() => {}, async () => { requests++; return Response.json({}); });
  const caps = { backend: "rt_core", cell: "machine", daemonReachable: true, rtCore: { axes: [] } } as unknown as DenseCapabilities;
  for (const [picker, reported] of [["bench_unknown", "rosie_1420_v1"], ["rosie_1420_v1", "bench_unknown"], ["bench_unknown", "bench_unknown"]]) {
    const status = { armed: true, target: { cell_id: "machine", selection_generation: "1", model_id: reported }, session: { id: "s", leaseHeld: true, state: "idle" }, rtCore: { ready: true } } as DenseStatus;
    machine.update(true, caps, status, false, true, picker);
    const reason = "Unknown robot model bench_unknown: this app does not know how to drive it";
    assert.equal(machine.disabled, reason);
    assert.deepEqual(cartesianPadPresentation({ modelId: reported, connected: true } as MachineFlowView, picker), { tag: "UNKNOWN MODEL", heading: reason, previewArmVisible: false });
    for (const mode of ["step", "hold"] as const) {
      machine.setMode(mode); machine.begin("base", [1, 0, 0, 0, 0, 0], .1);
    }
  }
  assert.equal(requests, 0);
});
