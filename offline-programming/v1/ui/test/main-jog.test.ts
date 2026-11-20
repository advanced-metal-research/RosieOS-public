import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { setImmediate } from "node:timers/promises";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";
import { MachineCartesianJog, cartesianPadPresentation } from "../src/execution/cartesianJog.ts";
import { type DenseCapabilities, type DenseStatus } from "../src/execution/denseClient.ts";
import { robotModelClass } from "../src/targets/cells.ts";
import { cartesianStepLabels, readCartesianSteps, writeCartesianSteps } from "../src/execution/cartesianMove.ts";
import { jogButtonAxes } from "../src/simulation/live-jog.ts";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const names = ["holdJogIntent", "stopJogHold", "toggleJogPad", "syncSpreadsheetCartesianJog", "wireSpreadsheetCartesianJog", "dockPaneShown"];
const handlers = ast.statements.filter(node => ts.isFunctionDeclaration(node) && names.includes(node.name?.text ?? ""));
assert.equal(handlers.length, 6);
const script = ts.transpileModule(handlers.map(node => node.getText(ast)).join("\n"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

/** Exercise the toolbar's real handlers and native Cartesian client, preventing
 * disconnected preview or a stale selection from becoming a second jog route. */
function harness(t: import("node:test").TestContext) {
  t.mock.timers.enable({ apis: ["setInterval", "setTimeout"] });
  for (const name of ["window", "document"]) {
    const old = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value: new EventTarget() });
    t.after(() => old ? Object.defineProperty(globalThis, name, old) : Reflect.deleteProperty(globalThis, name));
  }
  function element() {
    const attributes = new Map<string, string>();
    return {
      hidden: false, disabled: false, title: "", textContent: "", value: "base",
      dataset: { jogAxes: "" }, attributes,
      classList: { toggle() {} },
      setAttribute: (key: string, value: string) => attributes.set(key, value),
      addEventListener() {},
      onpointerdown: (_event: object) => {}, onpointerup: () => {},
      onpointerleave: () => {}, onpointercancel: () => {},
    };
  }
  const buttons = [...source.matchAll(/data-jog-axes="([^"]+)"/g)].map(match => {
    const button = element(); button.dataset.jogAxes = match[1]!; return button;
  });
  assert.equal(buttons.length, 12);
  const grid = { ...element(), querySelectorAll: () => buttons };
  const elements = new Map<string, ReturnType<typeof element>>(
    ["#spreadsheet-jog", "#jog-panel", "#jog-frame", "#jog-operator-feedback", "#cartesian-jog-tag", "#cartesian-jog-target",
      "#sim-arm-toggle", "#cartesian-speed-label", "#cartesian-speed", "#cartesian-machine-reason", "#cartesian-step-controls",
      "#cartesian-mode", "#cartesian-distance", "#cartesian-angle", "#cartesian-distance-label", "#cartesian-angle-label",
      "#cartesian-step-help", "#cartesian-zero"].map(id => [id, element()]),
  );
  elements.set("#spreadsheet-cartesian-grid", grid);
  elements.get("#cartesian-speed")!.value = "0.1";
  const calls: { route: string; body: Record<string, unknown>; headers: Headers; hidden: boolean; stored: number }[] = [];
  const stored: boolean[] = [], logs: string[] = [];
  let selected = false, model = "rosie_1400_v3", sync = () => {};
  let response: ((route: string) => Response | Promise<Response>) | null = null;
  const status = {
    backend: "rt_core", armed: true, roundTripMS: 1,
    target: { cell_id: "local-simulation", selection_generation: "3", model_id: model, backend: "simulation" },
    session: { id: "session", state: "idle", leaseHeld: true },
    rtCore: { ready: true },
  } as DenseStatus;
  const caps = { backend: "rt_core", cell: "local-dev", daemonReachable: true, rtCore: { axes: [] } } as unknown as DenseCapabilities;
  const machine = new MachineCartesianJog(() => sync(), async (url, init) => {
    assert.ok(String(url).startsWith("/api/offline-programming/v1/dense-execution/cartesian/"));
    const route = String(url).split("/").at(-1)!;
    const body = init?.body ? JSON.parse(String(init.body)) : {};
    calls.push({ route, body, headers: new Headers(init?.headers), hidden: elements.get("#spreadsheet-jog")!.hidden, stored: stored.length });
    if (response) return response(route);
    return Response.json(route === "move"
      ? { state: "done", axis: body.axis, revision: 8 }
      : { active: false, revision: 7 });
  }, text => logs.push(text));
  const jogAction = { checked: false };
  const state = {
    densePanel: { targetSelected: () => selected, cellModel: () => model, cartesianJog: () => machine,
      endJogHolds: () => machine.end(), flowView: () => null,
      zeroJointsGate: () => "Connect a machine first", zeroJointsLabel: () => "All joints to 0°", zeroAllJoints: () => Promise.resolve() },
    jogAction, stageActions: { spreadsheet: { checked: true } }, applySpreadsheetShown: () => {}, applyTelemetryVisible: () => {},
    robotModelClass, cartesianPadPresentation, cartesianStepLabels, readCartesianSteps, writeCartesianSteps, jogButtonAxes,
    jogPadVisible: false, telemetryVisible: false, jogHoldTimer: null, jogLive: true, simArmed: true,
    writeJogPadVisible: (value: boolean) => stored.push(value),
    query: (id: string) => { assert.ok(elements.has(id), id); return elements.get(id); },
    document: { querySelector: (id: string) => elements.get(id) },
    // Any obsolete preview call is a failure, including after a native refusal.
    jogRequest: () => { throw new Error("preview jog bypass"); },
  };
  const context = createContext(state);
  const run = (code: string) => runInContext(code, context);
  run(script);
  sync = () => run("syncSpreadsheetCartesianJog()");
  run("wireSpreadsheetCartesianJog()");
  const update = () => { machine.update(selected, caps, status, false, true, model); sync(); };
  t.after(() => machine.end());
  return {
    machine, status, calls, stored, logs, buttons, grid, elements, jogAction, run, update,
    connect() { selected = true; update(); },
    disconnect() { selected = false; update(); },
    model(value: string) { model = value; status.target!.model_id = value; update(); },
    refuse(next: typeof response) { response = next; },
    show() { run("toggleJogPad()"); },
    /** Press a pad key by its axes (the pad's layout is a controller, not a
     *  row, so position says nothing) or by index for sweeps. */
    press(which: number | string = "1,0,0,0,0,0") {
      const button = typeof which === "number" ? buttons[which] : buttons.find(b => b.dataset.jogAxes === which);
      grid.onpointerdown({ button: 0, target: { closest: () => button }, preventDefault() {} });
    },
    async hold() { this.connect(); this.show(); machine.setMode("hold"); this.press(); await setImmediate(); },
  };
}

test("Jog Action persists its pressed state; offline stays disabled even when preview is armed", t => {
  const h = harness(t);
  assert.equal(h.elements.get("#spreadsheet-jog")!.hidden, true);
  h.show(); assert.equal(h.jogAction.checked, true);
  h.press();
  assert.equal(h.calls.length, 0);
  assert.equal(h.buttons.filter(b => b.disabled).length, 12);
  assert.equal(h.elements.get("#spreadsheet-jog")!.title, "Connect and arm a cell to jog");
  h.show(); assert.equal(h.jogAction.checked, false);
  assert.deepEqual(h.stored, [true, false]);
});

for (const model of ["bench_one_motor_1to1", "bench_nine_motors_1to1", "bench_unknown"]) {
  test(model + " refuses every wired Cartesian direction without preview fallback", t => {
    const h = harness(t); h.connect(); h.model(model); h.show();
    assert.equal(h.buttons.filter(b => b.disabled).length, 12);
    for (let index = 0; index < 12; index++) h.press(index);
    h.run('holdJogIntent({mode:"base",axes:[1,0,0,0,0,0]})');
    assert.equal(h.calls.length, 0);
    assert.match(h.elements.get("#spreadsheet-jog")!.title, /bench|Unknown robot model/);
  });
}

test("the selected disarmed cell never falls back to armed browser preview", t => {
  const h = harness(t); h.connect(); h.status.armed = false; h.update(); h.show(); h.press();
  assert.equal(h.buttons.filter(b => b.disabled).length, 12);
  assert.equal(h.calls.length, 0);
  assert.match(h.elements.get("#cartesian-machine-reason")!.textContent, /ARM/);
});

test("clearing a bench leaves the offline pad disabled", t => {
  const h = harness(t); h.connect(); h.model("bench_one_motor_1to1"); h.show();
  h.disconnect(); h.model("rosie_1400_v3"); h.press();
  assert.equal(h.buttons.filter(b => b.disabled).length, 12);
  assert.equal(h.calls.length, 0);
});

for (const edge of ["onpointerup", "onpointerleave", "onpointercancel"] as const) {
  test(edge + " ends the native hold once and retires its cadence", async t => {
    const h = harness(t); await h.hold();
    t.mock.timers.tick(50); await setImmediate();
    h.grid[edge](); h.grid.onpointerup(); h.grid.onpointerleave(); await setImmediate();
    t.mock.timers.tick(100); await setImmediate();
    assert.deepEqual(h.calls.map(c => c.route), ["state", "start", "intent", "halt"]);
    for (const call of h.calls) {
      assert.equal(call.headers.get("X-RT-Target-Generation"), "3");
      assert.equal(call.headers.get("X-RT-Target-Cell"), "local-simulation");
    }
  });
}

test("idle exits send zero native halt requests", t => {
  const h = harness(t); h.connect(); h.show();
  h.grid.onpointerup(); h.grid.onpointerleave(); h.grid.onpointercancel();
  assert.equal(h.calls.length, 0);
});

for (const edge of ["disarm", "session loss", "hide", "target change"]) {
  test(edge + " releases native input once without replay", async t => {
    const h = harness(t); await h.hold();
    if (edge === "disarm") h.status.armed = false;
    if (edge === "session loss") h.status.session = null;
    if (edge === "target change") h.status.target!.selection_generation = "4";
    if (edge === "hide") h.show(); else h.update();
    await setImmediate(); h.grid.onpointerup();
    t.mock.timers.tick(100); await setImmediate();
    assert.deepEqual(h.calls.map(c => c.route), ["state", "start", "halt"]);
    assert.equal(h.calls[2]!.headers.get("X-RT-Target-Generation"), "3");
    if (edge === "hide") {
      assert.equal(h.calls[2]!.hidden, false);
      assert.equal(h.calls[2]!.stored, 1);
      assert.deepEqual(h.stored, [true, false]);
    }
  });
}

for (const frame of ["base", "tool"]) {
  test(frame + " Step sends one displacement and reports native completion", async t => {
    const h = harness(t); h.connect(); h.show(); h.machine.setMode("step");
    h.elements.get("#jog-frame")!.value = frame;
    h.elements.get("#cartesian-distance")!.value = "1";
    h.press(); await setImmediate(); h.grid.onpointerup();
    assert.deepEqual(h.calls.map(c => c.route), ["state", "move"]);
    assert.equal(h.calls[1]!.body.frame, frame);
    assert.equal(h.calls[1]!.body.axis, 0);
    assert.equal(h.calls[1]!.body.distance_mm, 1);
    assert.equal(h.calls[1]!.body.angle_degrees, 0);
    assert.equal(h.machine.reason, "done");
  });
}

test("Step RZ sends the Angle box in degrees", async t => {
  const h = harness(t); h.connect(); h.show(); h.machine.setMode("step");
  h.elements.get("#cartesian-angle")!.value = "15";
  h.press("0,0,0,0,0,1"); await setImmediate(); h.grid.onpointerup();
  assert.deepEqual(h.calls.map(c => c.route), ["state", "move"]);
  assert.equal(h.calls[1]!.body.axis, 5);
  assert.equal(h.calls[1]!.body.distance_mm, 0);
  assert.equal(h.calls[1]!.body.angle_degrees, 15);
});

test("TOOL holds keep the frame for native resolution and never use a browser pose", async t => {
  const h = harness(t); h.connect(); h.show(); h.machine.setMode("hold");
  h.elements.get("#jog-frame")!.value = "tool"; h.press("0,0,0,1,0,0"); await setImmediate();
  t.mock.timers.tick(50); await setImmediate(); h.grid.onpointerup(); await setImmediate();
  assert.equal(h.calls[2]!.body.frame, "tool");
  assert.deepEqual(h.calls[2]!.body.twist, [0, 0, 0, 0.2617993877991494, 0, 0]);
});

test("Hold +Y and +Rz send the operator boxes as SI twist with fraction", async t => {
  const h = harness(t); h.connect(); h.show(); h.machine.setMode("hold");
  h.elements.get("#cartesian-distance")!.value = "100";
  h.elements.get("#cartesian-angle")!.value = "15";
  h.press("0,1,0,0,0,0"); await setImmediate();
  const y = h.calls.find(call => call.route === "start")!;
  assert.deepEqual(y.body.twist, [0, 0.1, 0, 0, 0, 0], "the box speed travels unscaled; the resolver applies the 10 %");
  assert.equal(y.body.fraction, 0.1);
  h.grid.onpointerup(); await setImmediate();
  h.press("0,0,0,0,0,1"); await setImmediate();
  const rz = h.calls.filter(call => call.route === "start").at(-1)!;
  assert.deepEqual(rz.body.twist, [0, 0, 0, 0, 0, 0.2617993877991494]);
  assert.equal(rz.body.fraction, 0.1);
});

test("Cartesian labels name the unit for Hold and Step", t => {
  const h = harness(t); h.connect(); h.show();
  h.machine.setMode("step");
  assert.equal(h.elements.get("#cartesian-distance-label")!.textContent, "Distance (mm)");
  assert.equal(h.elements.get("#cartesian-angle-label")!.textContent, "Angle (degrees)");
  assert.match(h.elements.get("#cartesian-step-help")!.textContent, /distance or angle per press/);
  assert.equal(h.elements.get("#cartesian-distance")!.disabled, false);
  h.machine.setMode("hold");
  assert.equal(h.elements.get("#cartesian-distance-label")!.textContent, "Speed at 100% (mm/s)");
  assert.equal(h.elements.get("#cartesian-angle-label")!.textContent, "Rate at 100% (°/s)");
  assert.match(h.elements.get("#cartesian-step-help")!.textContent, /25 mm\/s/);
  assert.equal(h.elements.get("#cartesian-distance")!.disabled, false);
});

for (const failure of ["configuration mismatch", "stale generation", "transport lost"]) {
  test(failure + " is visible, copied to Logs and never replayed through preview", async t => {
    const h = harness(t); await h.hold();
    h.refuse(route => {
      if (route !== "intent") return Response.json({ active: false, revision: 7 });
      if (failure === "transport lost") throw new Error("transport lost");
      return Response.json({ error: failure === "stale generation" ? "target_changed" : "cell_configuration_mismatch", detail: failure }, { status: 409 });
    });
    t.mock.timers.tick(50); await setImmediate(); h.grid.onpointerup();
    t.mock.timers.tick(100); await setImmediate();
    assert.deepEqual(h.calls.map(c => c.route), ["state", "start", "intent", "halt"]);
    assert.ok(h.machine.reason.length > 0);
    assert.equal(h.elements.get("#jog-operator-feedback")!.hidden, false);
    assert.ok(h.logs.length >= 1);
  });
}

for (const fail of ["refused", "lost"]) {
  test("a " + fail + " release is reported without retrying or restarting input", async t => {
    const h = harness(t); await h.hold();
    h.refuse(() => {
      if (fail === "lost") throw new Error("release connection lost");
      return Response.json({ error: "jog_session_stale", detail: "release refused" }, { status: 409 });
    });
    h.grid.onpointerup(); await setImmediate(); h.grid.onpointerleave();
    t.mock.timers.tick(100); await setImmediate();
    assert.deepEqual(h.calls.map(c => c.route), ["state", "start", "halt"]);
    assert.ok(h.machine.reason.length > 0);
    assert.ok(h.logs.length >= 1);
  });
}
