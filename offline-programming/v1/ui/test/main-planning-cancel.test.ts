import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const handler = ast.statements.find(node => ts.isFunctionDeclaration(node) && node.name?.text === "planRailsProgram")!;
const script = ts.transpileModule(handler.getText(ast), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

test("Cancel aborts planning, releases the busy state and never loads a simulation", async () => {
  let cancel: (() => void) | undefined, signal: AbortSignal | undefined;
  let cancelled = 0, rendered = 0;
  const statuses: string[] = [];
  const state = {
    AbortController, project: {}, projectWorkpieces: () => [{}], offlineSimulationInFlight: false,
    densePanel: { connectedCell: () => null }, PLANNED_FROM_PROGRAM_START: "planned from the program's start",
    stopRailsPreview() {}, renderTargetAndReview() {}, render() { rendered++; },
    setStorageStatus: (text: string) => statuses.push(text),
    planProgress: {
      start(_title: string, action: () => void) { cancel = action; },
      cancelled() { cancelled++; }, stage() {}, setCancelAction() {},
    },
    railsMode: { planWeldTrajectory(_report: unknown, requestSignal: AbortSignal) {
      signal = requestSignal;
      return new Promise((_resolve, reject) => requestSignal.addEventListener("abort", () => reject(requestSignal.reason), { once: true }));
    } },
    loadRailsSimulation() { throw new Error("cancelled planning must not load simulation"); },
    showPlanningFailure() { throw new Error("cancellation is not a planning failure"); },
  };
  const context = createContext(state); runInContext(script, context);
  const pending = runInContext("planRailsProgram()", context) as Promise<void>;
  assert.equal(state.offlineSimulationInFlight, true);
  assert.ok(cancel);
  cancel();
  assert.equal(signal?.aborted, true);
  await pending;
  assert.equal(state.offlineSimulationInFlight, false);
  assert.equal(cancelled, 1);
  assert.equal(rendered, 1);
  assert.equal(statuses.at(-1), "Planning cancelled");
});

test("completed planning stops accepting cancellation before saving its result", async () => {
  let cancellable = true, loaded = false;
  const state = {
    AbortController, project: {}, projectWorkpieces: () => [{}], offlineSimulationInFlight: false,
    densePanel: { connectedCell: () => null }, PLANNED_FROM_PROGRAM_START: "planned from the program's start",
    stopRailsPreview() {}, renderTargetAndReview() {}, render() {}, setStorageStatus() {},
    planProgress: {
      start() {}, stage() {}, finish() {}, setCancelAction(action: null) { assert.equal(action, null); cancellable = false; },
    },
    railsMode: { async planWeldTrajectory(report: (stage: unknown) => void) {
      report({ index: 3, count: 4, label: "Keeping the plan" });
      assert.equal(cancellable, false);
      return true;
    } },
    async loadRailsSimulation() { loaded = true; return { state: "not-loaded" }; },
    showPlanningFailure(error: unknown) { throw error; },
  };
  const context = createContext(state); runInContext(script, context);
  await runInContext("planRailsProgram()", context);
  assert.equal(loaded, true);
  assert.equal(state.offlineSimulationInFlight, false);
});
