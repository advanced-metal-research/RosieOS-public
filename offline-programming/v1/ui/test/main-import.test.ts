import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { setImmediate } from "node:timers/promises";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const names = ["importSTEP", "importStepBytes", "openStepExampleById", "importRecentPart"];
const handlers = ast.statements.filter(node => ts.isFunctionDeclaration(node) && names.includes(node.name?.text ?? ""));
assert.equal(handlers.length, names.length);
const script = ts.transpileModule(handlers.map(node => node.getText(ast)).join("\n"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

function deferred<T>() {
  let resolve!: (value: T) => void;
  let reject!: (error: Error) => void;
  const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function harness() {
  const bytes = deferred<Uint8Array>();
  const topology = deferred<{ edges: unknown[] }>();
  const mesh = deferred<void>();
  const renders: (string | null)[] = [];
  const statuses: string[] = [];
  const buttons = new Map<string, { disabled: boolean; textContent: string; classList: { add(): void; remove(): void } }>();
  let extracts = 0, meshes = 0;
  const previous = { identity: { name: "Previous" }, cad: { fingerprint: "old" } };
  const state = {
    stepImportInFlight: false, importingPartFilename: null as string | null,
    workpieceSwitchInFlight: false, stepImportKind: "weld", importingKind: "weld", importingWorkpieceId: null,
    editingLockReason: () => null,
    projectWorkpieces: () => [], activeWorkpieceId: () => "active",
    railsMode: { reset() {} }, setSelection() {},
    viewer: { workpieceGizmoEnabled: false }, railsWeldsCard: { shown: false }, waypointWeldsCard: { shown: false },
    project: previous, activeCADSource: null,
    query: (id: string) => {
      if (!buttons.has(id)) buttons.set(id, { disabled: false, textContent: "", classList: { add() {}, remove() {} } });
      return buttons.get(id)!;
    },
    renderProgramTree: () => renders.push(state.importingPartFilename),
    markBusy() {}, setCADStatus: (text: string) => statuses.push(text),
    cadClient: { extractTopology: () => { extracts++; return topology.promise; } },
    appendCadQueryTopology: () => ({ identity: { name: "" }, cad: { fingerprint: "new" } }),
    repository: {
      putSource: () => Promise.resolve(),
      getSource: async () => ({ bytes: await bytes.promise }),
    }, refreshRecentParts() {},
    recentParts: [{ fingerprint: "recent", filename: "recent.step" }],
    history: { push() {} }, resetWeldAuthoringUiState() {},
    changed: () => state.renderProgramTree(),
    loadWorkpieceMesh: () => { meshes++; return mesh.promise; },
    stepExamples: [{ id: "demo", filename: "demo.step" }],
    examplesUrl: (name: string) => name,
    fetch: async () => ({ ok: true, arrayBuffer: () => bytes.promise }),
    message: (error: Error) => error.message,
  };
  const context = createContext(state);
  runInContext(script, context);
  const file = { name: "bracket.step", arrayBuffer: () => bytes.promise };
  const input = { files: [file], value: "bracket.step" };
  Object.assign(state, { event: { currentTarget: input } });
  return { state, bytes, topology, mesh, renders, statuses, previous, input,
    counts: () => ({ extracts, meshes }),
    run: (code = "importSTEP(event)") => runInContext(code, context) as Promise<void> };
}

test("filename appears before reading bytes and spinner lasts through topology and surface loading", async () => {
  const h = harness();
  const importing = h.run();
  assert.deepEqual(h.renders, ["bracket.step"]);
  assert.equal(h.input.value, "");
  assert.equal(h.state.project, h.previous);
  assert.equal(h.counts().extracts, 0);
  await h.run("importStepBytes('second.step', new Uint8Array())");
  assert.equal(h.state.importingPartFilename, "bracket.step");
  h.bytes.resolve(new Uint8Array([1]));
  await setImmediate();
  assert.equal(h.counts().extracts, 1);
  assert.equal(h.state.importingPartFilename, "bracket.step");
  h.topology.resolve({ edges: [] });
  await setImmediate();
  assert.equal(h.counts().meshes, 1);
  assert.equal(h.state.stepImportInFlight, true);
  assert.equal(h.renders.at(-1), "bracket.step");
  h.mesh.resolve();
  await importing;
  assert.equal(h.state.project.identity.name, "bracket");
  assert.equal(h.renders.at(-1), null);
  assert.equal(h.state.stepImportInFlight, false);
  assert.equal(h.state.query("#import-step").disabled, false);
});

for (const failure of ["file read", "topology"] as const) {
  test(`${failure} failure clears the spinner, preserves the previous part and allows retry`, async () => {
    const h = harness();
    const importing = h.run();
    if (failure === "file read") h.bytes.reject(new Error("read failed"));
    else {
      h.bytes.resolve(new Uint8Array());
      await setImmediate();
      h.topology.reject(new Error("invalid STEP"));
    }
    await importing;
    assert.equal(h.state.project, h.previous);
    assert.equal(h.renders.at(-1), null);
    assert.equal(h.state.stepImportInFlight, false);
    assert.equal(h.state.query("#import-step").disabled, false);
    assert.match(h.statuses.at(-1)!, /STEP import blocked: (read failed|invalid STEP)/);
  });
}

test("demo download shows its filename before network completion and clears on failure", async () => {
  const h = harness();
  const importing = h.run("openStepExampleById('demo')");
  assert.equal(h.renders.at(-1), "demo.step");
  await setImmediate();
  h.bytes.reject(new Error("network unavailable"));
  await importing;
  assert.equal(h.renders.at(-1), null);
  assert.match(h.statuses.at(-1)!, /network unavailable/);
});

test("recent part shows its filename while the stored source is read", async () => {
  const h = harness();
  const importing = h.run("importRecentPart('recent')");
  assert.equal(h.renders.at(-1), "recent.step");
  h.bytes.reject(new Error("storage unavailable"));
  await importing;
  assert.equal(h.renders.at(-1), null);
  assert.equal(h.state.project, h.previous);
  assert.match(h.statuses.at(-1)!, /storage unavailable/);
});
