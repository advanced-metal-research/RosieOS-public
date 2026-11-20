import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import ts from "typescript";
import { Action } from "../src/ui/action.ts";

const sourceText = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const source = ts.createSourceFile("main.ts", sourceText, ts.ScriptTarget.Latest, true);
let callback: ts.Expression | undefined;
function visit(node: ts.Node): void {
  if (ts.isCallExpression(node) && node.expression.getText(source) === "createPlannedPathController") {
    const deps = node.arguments[0];
    if (deps && ts.isObjectLiteralExpression(deps)) {
      const show = deps.properties.find(property => ts.isPropertyAssignment(property) && property.name.getText(source) === "show");
      if (show && ts.isPropertyAssignment(show)) callback = show.initializer;
    }
  }
  ts.forEachChild(node, visit);
}
visit(source);
assert.ok(callback, "main supplies the planned path display callback");
const executable = ts.transpileModule(`const show = ${callback.getText(source)};`, {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

type Outcome = { ok: boolean };
function deferred<T>() {
  let resolve!: (value: T) => void;
  let reject!: (error: unknown) => void;
  const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function harness(initial: Promise<Outcome>) {
  const installed: unknown[] = [];
  const traced: unknown[] = [];
  const viewer = {
    modelId: "robot",
    tracePlannedPath: async (runs: unknown) => { traced.push(runs); return runs; },
    setPlannedPath: (runs: unknown) => installed.push(runs),
  };
  const build = new Function("viewer", "densePlannedPathRuns", "projectPlannedPath", "initial", `
    let viewerProjectReady = initial;
    ${executable}
    return { show, setReady: next => { viewerProjectReady = next; } };
  `);
  const hooks = build(viewer, () => [["dense-route"]], () => ["classic-route"], initial) as {
    show(project: { robot: { model: string } }, dense: object | null, isCurrent: () => boolean): Promise<void>;
    setReady(next: Promise<Outcome>): void;
  };
  return { ...hooks, installed, traced };
}
const project = { robot: { model: "robot" } };

test("planned path waits for the latest model synchronization when an earlier load is superseded", async () => {
  const old = deferred<Outcome>();
  const latest = deferred<Outcome>();
  const h = harness(old.promise);
  const showing = h.show(project, {}, () => true);
  h.setReady(latest.promise);
  old.resolve({ ok: false });
  await Promise.resolve();
  assert.deepEqual(h.installed, []);
  latest.resolve({ ok: true });
  await showing;
  assert.deepEqual(h.installed, [[["dense-route"]]]);
});

test("changing the active plan while model geometry loads cannot trace or install old motion", async () => {
  const ready = deferred<Outcome>();
  const h = harness(ready.promise);
  let current = true;
  const showing = h.show(project, {}, () => current);
  current = false;
  ready.resolve({ ok: true });
  await showing;
  assert.deepEqual(h.traced, []);
  assert.deepEqual(h.installed, []);
});

test("an obsolete readiness failure does not hide a plan whose latest geometry loads", async () => {
  const old = deferred<Outcome>();
  const h = harness(old.promise);
  const showing = h.show(project, {}, () => true);
  h.setReady(Promise.resolve({ ok: true }));
  old.reject(new Error("obsolete overlay load failed"));
  await showing;
  assert.deepEqual(h.installed, [[["dense-route"]]]);
});

test("fresh classic planned samples install after model readiness without dense FK", async () => {
  const h = harness(Promise.resolve({ ok: true }));
  await h.show(project, null, () => true);
  assert.deepEqual(h.traced, []);
  assert.deepEqual(h.installed, [[["classic-route"]]]);
});

test("the viewport menu exposes TCP path enabled by default and toggles only its visibility", () => {
  let definition: ts.Expression | undefined;
  let connection: ts.CallExpression | undefined;
  let menuCall: ts.CallExpression | undefined;
  function find(node: ts.Node): void {
    if (ts.isPropertyAssignment(node) && node.name.getText(source) === "tcpPath") definition = node.initializer;
    if (ts.isCallExpression(node)) {
      if (node.expression.getText(source) === "stageActions.tcpPath.triggered.connect") connection = node;
      if (node.expression.getText(source) === "viewMenu.addActions" && node.arguments.some(argument =>
        ts.isArrayLiteralExpression(argument) && argument.elements.some(element => element.getText(source) === "stageActions.tcpPath"))) menuCall = node;
    }
    ts.forEachChild(node, find);
  }
  find(source);
  assert.ok(definition && connection && menuCall, "TCP path action is connected to the viewport menu");
  const action = new Function("Action", "Spline", `return ${definition.getText(source)};`)(Action, null) as Action;
  const visibility: boolean[] = [];
  const stageActions = { tcpPath: action, labels: {}, tcpFrame: {} };
  let items: unknown[] = [];
  const wiring = ts.transpileModule(`${connection.getText(source)}; ${menuCall.getText(source)};`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
  }).outputText;
  // A camera/pose call would fail: this adapter offers only visibility.
  new Function("stageActions", "viewer", "viewMenu", wiring)(stageActions,
    { setPlannedPathVisible: (value: boolean) => visibility.push(value) },
    { addActions: (actions: unknown[]) => { items = actions; } });
  assert.equal(action.checked, true);
  assert.ok(items.includes(action));
  action.trigger();
  action.trigger();
  assert.deepEqual(visibility, [false, true]);
  action.dispose();
});
