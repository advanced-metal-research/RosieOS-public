import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";
import { denseTrajectoryRecordOf } from "../src/rails/planRequest.ts";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
let installer: ts.Expression | undefined;
function visit(node: ts.Node): void {
  if (ts.isCallExpression(node) && ts.isIdentifier(node.expression) && node.expression.text === "createRailsMode") {
    const deps = node.arguments[0];
    assert.ok(deps && ts.isObjectLiteralExpression(deps));
    const property = deps.properties.find(item => ts.isPropertyAssignment(item) && item.name.getText(ast) === "applyPlan");
    assert.ok(property && ts.isPropertyAssignment(property));
    installer = property.initializer;
  }
  ts.forEachChild(node, visit);
}
visit(ast);
assert.ok(installer, "rails dependency must supply its plan installer");
const script = ts.transpileModule(`const applyPlan = ${installer.getText(ast)};`, {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

function harness() {
  let resolve!: () => void;
  const pending = new Promise<void>(done => { resolve = done; });
  const previous = { identity: { revision: 5 }, activeWorkpieceId: "part-a", denseTrajectory: null };
  const recorded: unknown[] = [], history: unknown[] = [], persisted: unknown[] = [], statuses: string[] = [];
  let renders = 0;
  let flushes = 0;
  const state = {
    project: previous as Record<string, unknown>, console, atob, Uint8Array, denseTrajectoryRecordOf,
    autosave: { flush: async () => { flushes += 1; } },
    repository: { putTrajectory: (record: unknown) => { persisted.push(record); return pending; } },
    recordPlan: (project: Record<string, unknown>) => { recorded.push(project); return { ...project, planRecorded: true }; },
    history: { push: (project: unknown) => history.push(project) },
    changed: (text: string) => statuses.push(text), render: () => { renders += 1; },
    result: {
      dense: {
        schema: "dense", trajectory_digest: "trajectory-a", request_sha256: "request-a", plan_id: "plan-a",
        program_id: "program-a", program_digest: "program-digest-a", manifest_revision: 1, plan_revision: 2,
        created_at: "2026-09-22", dt_s: 0.001, total_sample_count: 3, total_duration_s: 0.003,
        bytes: 3, segments: [{ index: 0, kind: "weld", source_id: "weld-a", sample_count: 3, duration_s: 0.003 }],
      },
      program_digest: "source-a", dense_blob_base64: "AQID",
    },
  };
  const context = createContext(state); runInContext(script, context);
  return { state, previous, recorded, history, persisted, statuses, resolve,
    get renders() { return renders; },
    get flushes() { return flushes; },
    run: () => runInContext("applyPlan(result)", context) as Promise<void>,
  };
}

test("switching parts during trajectory storage cannot install a stale plan or alter history", async () => {
  const h = harness(), installing = h.run();
  assert.equal(h.persisted.length, 1);
  assert.equal(h.state.project, h.previous, "the current project stays untouched while storage is pending");
  assert.equal(h.previous.denseTrajectory, null);
  const replacement = { identity: { revision: 6 }, activeWorkpieceId: "part-b", denseTrajectory: null };
  h.state.project = replacement; h.resolve(); await installing;
  assert.equal(h.state.project, replacement);
  assert.equal(replacement.denseTrajectory, null);
  assert.equal(h.previous.denseTrajectory, null);
  assert.equal(h.recorded.length, 0); assert.equal(h.history.length, 0);
  assert.equal(h.statuses.length, 0); assert.equal(h.renders, 0);
  assert.equal(h.flushes, 0);
});

test("an unchanged project receives the completed trajectory and one history entry", async () => {
  const h = harness(), installing = h.run();
  assert.equal(h.state.project, h.previous); assert.equal(h.recorded.length, 0);
  h.resolve(); await installing;
  const installed: Record<string, unknown> = h.state.project;
  assert.notEqual(installed, h.previous);
  assert.equal(installed.planRecorded, true);
  assert.equal(h.recorded.length, 1); assert.equal(h.history.length, 1); assert.equal(h.history[0], h.previous);
  const trajectory = installed.denseTrajectory as Record<string, unknown>;
  assert.equal(trajectory.trajectoryDigest, "trajectory-a");
  assert.equal(trajectory.sourceProjectRevision, 5); assert.equal(trajectory.sourceProgramDigest, "source-a");
  assert.equal(h.previous.denseTrajectory, null, "history retains the unplanned snapshot");
  assert.deepEqual(h.statuses, ["Program planned"]); assert.equal(h.renders, 1);
  assert.equal(h.flushes, 1, "the installed plan is flushed before returning");
});
