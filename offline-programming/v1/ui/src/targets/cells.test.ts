import assert from "node:assert/strict";
import test from "node:test";
import { armModels, CellCatalogueError, CellCatalogueUnreachableError, CellTargetsClient, cellTargetText, cellModelPresentation, renderCellModel, robotModelClass, type CellTarget } from "./cells.ts";
import { canonicalRobotModel, robotModelFor } from "../viewer/models.ts";
import { robotCatalogue, setRobotCatalogue } from "../domain/robots.ts";

test("machine selection sends only cell/model intent and preserves a typed refusal", async () => {
  const calls: { url: string; init: RequestInit }[] = [];
  const client = new CellTargetsClient(async (url, init) => {
    calls.push({ url: String(url), init: init! });
    return Response.json({ error: "cell_mode_mismatch" }, { status: 409 });
  });
  await assert.rejects(client.select({ cell_id: "bench-relay", model_id: "rosie_1420_v1" }), /cell_mode_mismatch/);
  assert.equal(calls.length, 1);
  assert.equal(calls[0]!.url, "/api/offline-programming/v1/targets/select");
  assert.deepEqual(JSON.parse(String(calls[0]!.init.body)), { cell_id: "bench-relay", model_id: "rosie_1420_v1" });
  assert.equal(calls[0]!.init.redirect, "error");
});

test("connected machine text contains label, model, backend and host only", () => {
  const cell = { label: "Detached bench", backend: "simulation", host: "127.0.0.1:8443", key_file: "/private/key.pem" } as unknown as CellTarget;
  assert.equal(cellTargetText(cell, "rosie_1420_v1"), "Detached bench · rosie_1420_v1 · simulation · 127.0.0.1:8443");
});

test("bench models replace the viewport and disable the model select; arm paths remain identical", t => {
  const note = { textContent: "", hidden: true };
  const panes = [{ style: { visibility: "" } }, { style: { visibility: "" } }];
  let option: { value: string; textContent: string } | null = null;
  const select = { disabled: false, value: "", querySelector: () => option ? { remove: () => { option = null; } } : null,
    append: (value: typeof option) => { option = value; } };
  const previous = Object.getOwnPropertyDescriptor(globalThis, "document");
  Object.defineProperty(globalThis, "document", { configurable: true, value: { createElement: () => ({ dataset: {} }) } });
  t.after(() => previous ? Object.defineProperty(globalThis, "document", previous) : Reflect.deleteProperty(globalThis, "document"));
  for (const model of ["bench_one_motor_1to1", "bench_nine_motors_1to1"] as const) {
    assert.equal(robotModelClass(model), "bench");
    assert.equal(cellModelPresentation(model).armModel, null);
    renderCellModel(select as unknown as HTMLSelectElement, note as HTMLElement, panes as HTMLElement[], model);
    assert.equal(select.disabled, true);
    assert.equal(select.value, model);
    assert.equal(option!.textContent, model === "bench_one_motor_1to1" ? "Mini bench (one motor, 1:1)" : "Bench (nine motors, 1:1)");
    assert.equal(note.hidden, false);
    assert.equal(note.textContent, "Bench cell: no robot model. Live joint table below.");
    assert.deepEqual(panes.map(pane => pane.style.visibility), ["hidden", "hidden"]);
  }
  for (const model of ["rosie_1400_v3", "rosie_1420_v1"] as const) {
    const before = JSON.stringify(canonicalRobotModel(model));
    assert.equal(robotModelClass(model), "arm");
    const arm = cellModelPresentation(model).armModel;
    assert.equal(arm, model);
    assert.equal(JSON.stringify(canonicalRobotModel(arm!)), before);
    renderCellModel(select as unknown as HTMLSelectElement, note as HTMLElement, panes as HTMLElement[], model);
    assert.equal(select.disabled, false);
    assert.equal(select.value, model);
    assert.equal(option, null);
    assert.equal(note.hidden, true);
    assert.deepEqual(panes.map(pane => pane.style.visibility), ["", ""]);
  }
  assert.equal(robotModelClass("bench_unknown"), "unknown");
  renderCellModel(select as unknown as HTMLSelectElement, note as HTMLElement, panes as HTMLElement[], "bench_unknown");
  assert.equal(select.disabled, true);
  assert.equal(select.value, "bench_unknown");
  assert.equal(note.hidden, false);
  assert.equal(note.textContent, "Unknown robot model bench_unknown: this app does not know how to drive it");
  assert.deepEqual(panes.map(pane => pane.style.visibility), ["hidden", "hidden"]);
});

test("catalogue validation rejects malformed targets and inconsistent selections", async () => {
  const cell = { cell_id: "bench", label: "Bench", role: "bench", models: ["rosie_1420_v1"], requested_mode: "simulation", backend: "simulation", host: "localhost" };
  const changed = (change: object) => ({ targets: [{ ...cell, ...change }], selected: null });
  for (const [body, reason] of [
    [null, "response is not an object"],
    [{}, "targets is not a list"],
    [{ targets: [] }, "selected is not a cell_id/model_id pair"],
    [{ targets: [null], selected: null }, "targets[0] is not an object"],
    [changed({ models: [] }), "targets[0].models is not a non-empty list"],
    [changed({ models: [1] }), "targets[0].models has an empty or non-string model"],
    [changed({ models: [""] }), "targets[0].models has an empty or non-string model"],
    [changed({ host: null }), "targets[0].host null is not a non-empty string"],
    [changed({ backend: "other" }), 'targets[0].backend "other" is not simulation, ethercat or unreachable'],
    [changed({ requested_mode: "other" }), 'targets[0].requested_mode "other" is not simulation or real'],
    [changed({ simulation: "true" }), 'targets[0].simulation "true" is not a boolean'],
    [{ targets: [cell, cell], selected: null }, "cell_id bench appears twice"],
    [{ targets: [cell], selected: { cell_id: "missing", model_id: "rosie_1420_v1" } }, "selected missing/rosie_1420_v1 is not a catalogued cell and model"],
    [{ targets: [cell], selected: { cell_id: "bench", model_id: "bench_unknown" } }, "selected bench/bench_unknown is not a catalogued cell and model"],
  ] as const) {
    await assert.rejects(new CellTargetsClient(async () => Response.json(body)).list(),
      (error: Error) => error instanceof CellCatalogueError && error.message === `Invalid machine catalogue: ${reason}`);
  }
  const selected = { cell_id: "bench", model_id: "rosie_1420_v1" };
  await assert.rejects(new CellTargetsClient(async () => Response.json({ target: cell, selected: { ...selected, model_id: "rosie_1400_v3" } })).select(selected),
    { message: "Invalid machine catalogue: selected bench/rosie_1400_v3 is not a catalogued cell and model" });
  assert.deepEqual(await new CellTargetsClient(async () => Response.json({ targets: [cell], selected })).list(), { targets: [cell], selected });
});

test("a dead proxy is reported as unreachable with its status, not as an invalid catalogue", async () => {
  // Vite's dev proxy answers a refused upstream connection with an empty 500.
  const dead = new CellTargetsClient(async () => new Response("", { status: 500 }));
  for (const call of [() => dead.list(), () => dead.select({ cell_id: "bench", model_id: "rosie_1420_v1" })]) {
    await assert.rejects(call(), (error: Error) => error instanceof CellCatalogueUnreachableError
      && error instanceof CellCatalogueError && error.message === "OLP API unreachable: HTTP 500 with no JSON body");
  }
  const html = new CellTargetsClient(async () => new Response("<!doctype html>", { status: 200, headers: { "Content-Type": "text/html" } }));
  await assert.rejects(html.list(), { message: "Invalid machine catalogue: HTTP 200 body is not JSON" });
});

test("unknown catalogue identity stays unknown and selection posts zero requests", async () => {
  const cell = { cell_id: "bench", label: "Bench", role: "bench", models: ["bench_unknown"], requested_mode: "simulation", backend: "simulation", host: "localhost" };
  let posts = 0;
  const client = new CellTargetsClient(async (_url, init) => {
    if (init?.method === "POST") posts++;
    return Response.json({ targets: [cell], selected: null });
  });
  const list = await client.list();
  assert.equal(robotModelClass(list.targets[0]!.models[0]!), "unknown");
  await assert.rejects(client.select({ cell_id: "bench", model_id: "bench_unknown" }), /Unknown robot model bench_unknown: this app does not know how to drive it/);
  assert.equal(posts, 0);
});

test("an arm the catalogue describes is offered and drawn by its family, with no id in code", t => {
  const previous = robotCatalogue();
  t.after(() => setRobotCatalogue(previous));
  const template = previous.find(entry => entry.modelId === "rosie_1420_v1")!;
  setRobotCatalogue([...previous,
    { ...template, modelId: "example_1430_arm", urdfPackage: "rosie_1430_v1" },
    { ...template, modelId: "example_other_arm", urdfPackage: "example_other_v1" }]);
  assert.equal(robotModelClass("example_1430_arm"), "arm");
  assert.ok(armModels().includes("example_1430_arm"));
  const family = robotModelFor("example_1430_arm");
  assert.equal(family.displayName, "Rosie 1430 · example_1430_arm");
  assert.equal(family.urdfURL, "robot/example_1430_arm/robot.urdf");
  assert.deepEqual(family.packageMap, { rosie_1430_v1: "robot/example_1430_arm" });
  assert.deepEqual(family.cameras, canonicalRobotModel("rosie_1420_v1").cameras, "a Rosie 1430 is framed like the 1420");
  // A family this app has no chrome for still draws, from its own files.
  assert.deepEqual(robotModelFor("example_other_arm").packageMap, { example_other_arm: "robot/example_other_arm" });
  // A model no description backs is unknown, so it gains no arm renderer.
  assert.equal(robotModelClass("example_missing_arm"), "unknown");
  assert.ok(!armModels().includes("example_missing_arm"));
});
