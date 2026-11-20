import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const functions = ast.statements.filter(node => ts.isFunctionDeclaration(node)
  && ["renderWeldDrawerPanel", "syncEdgeSelectionState"].includes(node.name?.text ?? ""));
const edgeSelection = ast.statements.filter(ts.isVariableStatement)
  .flatMap(node => [...node.declarationList.declarations])
  .find(node => node.name.getText(ast) === "edgeSelection")!;
const script = ts.transpileModule([
  ...functions.map(node => node.getText(ast)),
  `const edgeSelection = ${edgeSelection.initializer!.getText(ast)};`,
].join("\n"), { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None } }).outputText;

function harness() {
  const drawer = { hidden: false };
  const calls: string[] = [];
  let callbacks: { onEdgeClick(id: string, point: number[]): void; onScenePointClick(point: number[]): void };
  let picking = false;
  const state = {
    project: { railsAuthoring: true, robot: { model: "rosie_1400_v3" } },
    weldDraft: null, edgeSelectionMode: true,
    query: () => drawer, viewer: {}, cellWorkFrame: () => ({ worldM: [0, 0, 0] }),
    EdgeSelectionController: class {
      constructor(options: { callbacks: typeof callbacks }) { callbacks = options.callbacks; }
      setState(value: { mode: boolean }) { picking = value.mode; }
    },
    railsMode: {
      enabled: false,
      renderPanel: () => { calls.push("rails panel"); return ""; },
      renderParameterSections: () => ({}),
      onEdgeClicked: () => { calls.push("rails edge"); return false; },
      onScenePointClicked: () => { calls.push("rails point"); return false; },
    },
    railsWeldsCard: { setSummaries() {} }, renderPlanPresets() {},
    renderWeldDrawer: () => { drawer.hidden = false; calls.push("classic panel"); },
    projectWelds: () => [],
    draftWithEdge: () => { calls.push("classic draft"); return null; },
    addControlPointAtWorld: () => calls.push("classic point"),
    createWeldFromDraft() {}, deleteWeldFromProgram() {},
  };
  const context = createContext(state);
  runInContext(script, context);
  return { state, drawer, calls, picking: () => picking,
    render: () => runInContext("renderWeldDrawerPanel(); syncEdgeSelectionState();", context),
    click: () => { callbacks.onEdgeClick("edge", [0, 0, 0]); callbacks.onScenePointClick([0, 0, 0]); },
  };
}

test("rails loading, ready and failed states never open the classic edge drawer", () => {
  const h = harness();
  for (const enabled of [false, true, false]) {
    h.state.railsMode.enabled = enabled;
    h.render();
    assert.equal(h.drawer.hidden, true);
    assert.equal(h.picking(), enabled, "stale classic selection must not enable picking while rails loads");
  }
  assert.deepEqual(h.calls, ["rails panel"]);
  h.state.project.railsAuthoring = false;
  h.render();
  assert.equal(h.drawer.hidden, false);
  assert.equal(h.calls.at(-1), "classic panel");
});

test("rails clicks cannot fall through into classic drafts even when the runtime declines them", () => {
  const h = harness();
  h.click();
  h.state.railsMode.enabled = true;
  h.click();
  assert.deepEqual(h.calls, ["rails edge", "rails point", "rails edge", "rails point"]);
  h.calls.length = 0;
  h.state.project.railsAuthoring = false;
  h.state.railsMode.enabled = false;
  h.click();
  assert.deepEqual(h.calls, ["classic draft", "classic panel", "classic point"]);
});
