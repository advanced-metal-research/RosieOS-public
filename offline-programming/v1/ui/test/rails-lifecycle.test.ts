import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { setImmediate } from "node:timers/promises";
import { createContext, runInContext } from "node:vm";
import * as THREE from "three";
import ts from "typescript";
import { hasWorkpieceGeometry } from "../src/domain/placement.ts";
import { effectiveNodes } from "../src/program-v2/nodes.ts";
import { evaluate } from "../src/rails/scalar-function.ts";
import { railsPayloadOf, railsProvenance, seamWeldNodeFor } from "../src/program-v2/rails-node.ts";
import { decideWeld, measureRunPiece, runIndexAt, runPiecePayload, searchSeam } from "../src/rails/authoring.ts";
const source = await readFile(new URL("../src/rails/index.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("index.ts", source, ts.ScriptTarget.ES2022, true);
const factory = ast.statements.find(node => ts.isFunctionDeclaration(node) && node.name?.text === "createRailsMode")!;
const script = ts.transpileModule(factory.getText(ast).replace("export function", "function"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;
function deferred<T = any>() {
  let resolve!: (value: T) => void, reject!: (error: Error) => void;
  const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}
const seam = { id: "same-seam", weld_joint_ref: "same-joint", span: { start_s: 0, end_s: 1 }, reference: {} };
const joint = { id: "same-joint", weld_line: {} };
const joints = { seams: [seam], weld_joints: [joint] };
const frames = { frames: [{ s: 0, point_xyz_m: [0, 0, 0] }, { s: 1, point_xyz_m: [1, 0, 0] }] };
/** Real lifecycle, real Three groups, deferred worker responses; presentation
 * dependencies are stubbed so the race is reproducible without WebGL/network. */
function harness() {
  let project = { cad: { fingerprint: "A" }, seams: [], edgeCatalog: [{ samplesM: [[0, 0, 0], [1, 0, 0]] }], nodes: [{ kind: "weld", geometry: { kind: "manual_pose_wp" } }] as any[], robot: {}, program: {}, denseTrajectory: { trajectoryDigest: "previous-plan" } };
  const parent = new THREE.Group();
  const statuses: string[] = [], writes: unknown[] = [];
  const detection: ReturnType<typeof deferred>[] = [], searches: ReturnType<typeof deferred>[] = [], frameRequests: ReturnType<typeof deferred>[] = [];
  let deferFrames = false, renderCount = 0;
  let detailCallbacks: { onDecide(choice: "accept" | "trim"): void } | null = null;
  const planned = deferred();
  let planSignal: AbortSignal | undefined;
  const client = {
    encodeSTEP: () => "step",
    planWeldTrajectory: (_body: unknown, signal?: AbortSignal) => { planSignal = signal; return planned.promise; },
    detectJoints() { const call = deferred(); detection.push(call); return call.promise; },
    sampleSeamFrames() { if (!deferFrames) return Promise.resolve(frames); const call = deferred(); frameRequests.push(call); return call.promise; },
    planTorchPath() { const call = deferred(); searches.push(call); return call.promise; },
    checkTorchFits() { throw new Error("stale search must never check clearance"); },
  };
  const context = createContext({
    THREE, client, console, structuredClone, evaluate, effectiveNodes, hasWorkpieceGeometry,
    isRailsWeld: () => true, isManualPoseWeld: () => false, railsPayloadOf, railsProvenance, seamWeldNodeFor,
    decideWeld, measureRunPiece, runIndexAt, runPiecePayload, searchSeam,
    buildRailsOverlay: () => new THREE.Group(), tickOverlayFlow() {},
    lowerManualPoseWelds: (value: unknown) => value, toProgramV2Document: () => ({}),
    computeProgramV2Digest: async () => "digest",
    // rails/planRequest.ts, lifted out of the closure: packing stays asynchronous.
    plannedNodes: (value: { nodes: any[] }) => effectiveNodes(value.nodes), packPlanRequest: async () => ({}),
    measuredStartUnsupported: () => "", hasMotionNodes: () => true, PLANNED_FROM_PROGRAM_START: "planned from the program's start",
    MATCH_TOLERANCE_M: 0.003, POINT_TOLERANCE_M: 0.025, RAILS_AMBER: "orange",
    document: { body: { appendChild() {}, append() {} } }, requestAnimationFrame() {},
    buildDetailDialog: () => ({ element: {}, close() {}, show(_entry: unknown, callbacks: typeof detailCallbacks) { detailCallbacks = callbacks; } }),
    createParameterDialog: () => ({ element: {} }), createPresetLibrary: () => ({}),
    createParameterView: () => ({ ready: async () => {}, values: () => ({}), set: { preset: {} } }),
    SEARCH_SCHEMA: [], SEARCH_SCOPE: "", SHIPPED_SEARCH_PRESETS: [],
    TORCH_SCHEMA: [], TORCH_SCOPE: "", SHIPPED_TORCH_PRESETS: [], torchProblems() {}, torchRequest: () => ({}),
    projectWorkpiecePlacement: () => ({ originM: [0, 0, 0], rotationRpyRad: [0, 0, 0] }),
    disposeOverlay: (group: THREE.Group) => group.removeFromParent(),
    catalogEdgeById: () => ({ samplesM: [[0, 0, 0], [1, 0, 0]] }),
    deps: {
      parent, getFixtures: () => [], getProject: () => project,
      getSource: () => ({ kind: "bytes", filename: project.cad.fingerprint, bytes: new Uint8Array([1]) }),
      status: (text: string) => statuses.push(text), requestPanelRender: () => { renderCount += 1; },
      upsertWeldNode: (node: unknown) => writes.push(node), removeWeldNode: (node: unknown) => writes.push(node),
      applyPlan: async (plan: unknown) => writes.push(plan), presetStore: {},
    },
  });
  runInContext(script + "\nglobalThis.mode = createRailsMode(deps);", context);
  const mode = context.mode as import("../src/rails/index.ts").RailsMode;
  return {
    mode, parent, detection, searches, frameRequests, statuses, writes, planned,
    get planSignal() { return planSignal; },
    get detailCallbacks() { return detailCallbacks; },
    get project() { return project; },
    get renders() { return renderCount; }, set deferFrames(value: boolean) { deferFrames = value; },
    switchPart(name: string) { mode.reset(); project = { ...project, cad: { fingerprint: name } }; },
    async ready() { const pending = mode.enable(); detection.at(-1)!.resolve({ joints }); await pending; },
  };
}
test("reset releases loading and late detection cannot replace a newer part", async () => {
  const h = harness(), first = h.mode.enable();
  assert.equal(h.mode.enable(), first, "repeated enable shares the analysis promise");
  assert.equal(h.mode.busy, true);
  h.switchPart("B"); assert.equal(h.mode.busy, false);
  const next = h.mode.enable(); h.detection[1]!.resolve({ joints }); await next;
  const renders = h.renders, statuses = h.statuses.length;
  h.detection[0]!.resolve({ joints }); await first;
  assert.equal(h.mode.fingerprint, "B"); assert.equal(h.mode.enabled, true); assert.equal(h.mode.busy, false);
  assert.equal(h.renders, renders); assert.equal(h.statuses.length, statuses);
  assert.equal(h.parent.children[0]!.children.length, 1);
});

function authoredWeld(id = "rails-same-seam") {
  return {
    id, op: "weld", enabled: true, label: "My weld", comment: "trimmed to 111 mm",
    geometry: { kind: "seam", id: seam.id, curve: {}, span: { start_s: 0.2, end_s: 0.8 },
      direction: "forward", reference: {}, orientation: {
        work_angle_deg: { kind: "constant", value: 0 }, travel_angle_deg: { kind: "constant", value: 10 },
      } },
    travel_speed_mm_s: { kind: "constant", value: 4 }, weave: { shape: "none" },
    extensions: { rails: { sha256: "A", joint_id: joint.id, joint,
      payload: { id, seam_ids: [seam.id], preset_id: null, overrides: {} } } },
  };
}

test("program deletion removes the Rails entry and overlay, and undo/reload restores it", async () => {
  const h = harness();
  const node = authoredWeld(), other = authoredWeld("another-run");
  h.project.nodes = [node, other];
  await h.ready();
  assert.equal(h.mode.entries.length, 2, "saved authored nodes populate Rails on reload");
  const removedOverlay = h.mode.entries[0]!.overlay!;
  const otherOverlay = h.mode.entries[1]!.overlay!;
  h.project.nodes = [other];
  h.mode.syncProgramWelds();
  assert.equal(h.mode.entries.length, 1);
  assert.equal(h.mode.entries[0]!.id, other.id);
  assert.equal(removedOverlay.parent, null);
  assert.equal(otherOverlay.parent, h.parent.children[0]);
  assert.equal(h.writes.length, 0, "reconciliation never edits the program");
  h.project.nodes = [node, other];
  h.mode.syncProgramWelds();
  assert.equal(h.mode.entries.length, 2, "undo restores the removed weld");
  const restored = h.mode.entries.find(entry => entry.id === node.id)!;
  assert.equal(JSON.stringify(restored.span), JSON.stringify(node.geometry.span));
  assert.equal(restored.note, node.comment);
  assert.ok(restored.overlay?.parent);
  h.project.nodes = [];
  h.mode.syncProgramWelds();
  assert.equal(h.mode.entries.length, 0, "redo removes both authored rows without ghosts");
});

test("program edits update Rails payload and preserve unaccepted proposals", async () => {
  const h = harness(); await h.ready();
  h.mode.onEdgeClicked("edge"); await setImmediate();
  const proposed = h.mode.entries[0]!;
  h.mode.syncProgramWelds();
  assert.equal(h.mode.entries[0], proposed, "a draft without a program node is still a valid proposal");
  const node = authoredWeld();
  h.project.nodes = [node]; h.mode.syncProgramWelds();
  const edited = structuredClone(node);
  edited.label = "Edited weld"; edited.geometry.span.end_s = 0.5;
  edited.geometry.orientation.travel_angle_deg.value = 20;
  h.project.nodes = [edited]; h.mode.syncProgramWelds();
  assert.equal(h.mode.entries[0]!.seam.label, "Edited weld");
  assert.equal(h.mode.entries[0]!.span.end_s, 0.5);
  assert.equal((h.mode.entries[0]!.payload!.overrides.orientation as any).travel_angle_deg.value, 20);
  h.project.nodes = []; h.mode.syncProgramWelds();
  for (const call of h.searches) call.resolve({ direction: "forward", plan: { clear_fraction: 1 } });
  await setImmediate();
  assert.equal(h.mode.entries.length, 0, "late search cannot resurrect a deleted weld");
  assert.equal(h.writes.length, 0);
  assert.equal(h.mode.busy, false);
});

test("cancelling a plan prevents even a late successful response from installing", async () => {
  const h = harness(); await h.ready();
  const previous = h.project;
  const savedPlan = h.project.denseTrajectory;
  const cancellation = new AbortController();
  const planning = h.mode.planWeldTrajectory(undefined, cancellation.signal);
  await setImmediate();
  assert.equal(h.planSignal, cancellation.signal);
  cancellation.abort();
  h.planned.resolve({ dense: null });
  await assert.rejects(planning, { name: "AbortError" });
  assert.equal(h.writes.length, 0);
  assert.equal(h.project, previous);
  assert.equal(h.project.denseTrajectory, savedPlan, "cancellation preserves the previous saved plan");
  assert.equal(h.mode.busy, false);
  assert.equal(h.statuses.at(-1), "Rails: Planning cancelled");
  assert.equal(await h.mode.planWeldTrajectory(), true, "a fresh plan is accepted after cancellation");
  assert.equal(h.writes.length, 1);
});

test("cancelling while packing prevents the planner request", async () => {
  const h = harness(); await h.ready();
  const cancellation = new AbortController();
  const planning = h.mode.planWeldTrajectory(undefined, cancellation.signal);
  cancellation.abort();
  await assert.rejects(planning, { name: "AbortError" });
  assert.equal(h.planSignal, undefined);
  assert.equal(h.writes.length, 0);
  assert.equal(h.mode.busy, false);
});
test("late guides cannot enter a new part cache or clear its busy state", async () => {
  const h = harness(); h.deferFrames = true;
  const first = h.mode.enable(); h.detection[0]!.resolve({ joints }); await setImmediate();
  assert.equal(h.frameRequests.length, 1);
  h.switchPart("A"); // Identical fingerprints still represent different instances.
  const second = h.mode.enable(); h.detection[1]!.resolve({ joints }); await setImmediate();
  h.frameRequests[0]!.resolve(frames); await first;
  assert.equal(h.mode.busy, true); assert.equal(h.mode.fingerprint, null);
  assert.equal(h.parent.children[0]!.children.length, 0);
  h.frameRequests[1]!.resolve(frames); await second;
  assert.equal(h.mode.enabled, true); assert.equal(h.parent.children[0]!.children.length, 1);
});
test("late angle searches cannot add entries, check clearance or write nodes", async () => {
  const h = harness(); await h.ready(); assert.equal(h.mode.onEdgeClicked("edge"), true); await setImmediate();
  assert.equal(h.searches.length, 2); assert.equal(h.mode.entries.length, 1);
  h.switchPart("B"); await h.ready(); const renders = h.renders, statuses = h.statuses.length;
  for (const call of h.searches) call.resolve({ direction: "forward", plan: { clear_fraction: 1 } });
  await setImmediate();
  assert.equal(h.mode.entries.length, 0); assert.equal(h.mode.fingerprint, "B");
  assert.equal(h.renders, renders); assert.equal(h.statuses.length, statuses); assert.equal(h.writes.length, 0);
});
test("disable cancels startup and suppresses late errors", async () => {
  const h = harness(), pending = h.mode.enable(); h.mode.disable(); const statuses = h.statuses.length;
  h.detection[0]!.reject(new Error("old STEP failed")); await pending;
  assert.equal(h.mode.enabled, false); assert.equal(h.mode.busy, false); assert.equal(h.statuses.length, statuses);
});

test("a plan returned after switching parts is never installed", async () => {
  const h = harness(); await h.ready();
  const planning = h.mode.planWeldTrajectory(); await setImmediate();
  assert.equal(h.mode.busy, true);
  h.switchPart("B"); await h.ready();
  const renders = h.renders, statuses = h.statuses.length;
  h.planned.resolve({ dense: null });
  assert.equal(await planning, false);
  assert.equal(h.writes.length, 0); assert.equal(h.renders, renders);
  assert.equal(h.statuses.length, statuses); assert.equal(h.mode.fingerprint, "B");
});

test("frames finishing after the replacement is ready cannot poison matching", async () => {
  const h = harness(); h.deferFrames = true;
  const first = h.mode.enable(); h.detection[0]!.resolve({ joints }); await setImmediate();
  h.switchPart("B"); h.deferFrames = false; await h.ready();
  h.frameRequests[0]!.resolve({ frames: [
    { s: 0, point_xyz_m: [100, 0, 0] }, { s: 1, point_xyz_m: [101, 0, 0] },
  ] });
  await first;
  h.mode.onEdgeClicked("edge"); await setImmediate();
  assert.equal(h.searches.length, 2, "matching must still use B's frames at the origin");
  assert.equal(h.mode.fingerprint, "B");
  h.mode.reset(); // Cancel the deliberately pending searches.
  for (const call of h.searches) call.reject(new Error("cancelled old search"));
  await setImmediate();
});

test("failed analysis releases readiness and can be retried", async () => {
  const h = harness(); const failed = h.mode.enable();
  h.detection[0]!.reject(new Error("worker unavailable")); await failed;
  assert.equal(h.mode.busy, false); assert.equal(h.mode.enabled, false);
  assert.equal(h.mode.fingerprint, null);
  assert.match(h.statuses.at(-1)!, /worker unavailable/);
  await h.ready();
  assert.equal(h.detection.length, 2); assert.equal(h.mode.enabled, true);
  assert.equal(h.mode.fingerprint, "A");
});


test("authored motion edits invalidate old measurements but unrelated project clones do not", async () => {
  const h = harness(); const node = authoredWeld(); h.project.nodes = [node]; await h.ready();
  const original = h.mode.entries[0]!;
  original.plan = { knot_fractions: [0, 1], work_path_deg: [0, 0], travel_path_deg: [10, 10], direction: "forward", clear_fraction: 1 } as any;
  original.clearance = { old: true } as any;
  original.summary = { trimSpan: { start_s: 0.25, end_s: 0.75 } } as any;
  const plan = original.plan;
  h.project.nodes = [structuredClone(node)]; h.mode.syncProgramWelds();
  assert.equal(h.mode.entries[0], original);
  assert.equal(h.mode.entries[0]!.plan, plan, "unrelated immutable project changes preserve measurement");
  const edited = structuredClone(node); edited.geometry.direction = "reverse";
  edited.geometry.span.end_s = 0.5;
  h.project.nodes = [edited]; h.mode.syncProgramWelds();
  const replacement = h.mode.entries[0]!;
  assert.notEqual(replacement, original);
  assert.equal(original.overlay, null);
  assert.equal(replacement.plan, null);
  assert.equal(replacement.clearance, null);
  assert.equal(replacement.summary, null);
  assert.equal(replacement.seam.direction, "reverse");
  assert.equal(replacement.span.end_s, 0.5);
});

test("late angle search cannot overwrite an externally edited authored entry", async () => {
  const h = harness(); const node = authoredWeld(); h.project.nodes = [node]; await h.ready();
  h.mode.onEdgeClicked("edge"); await setImmediate();
  const searching = h.mode.entries[0]!;
  assert.equal(searching.state, "searching");
  const edited = structuredClone(node); edited.geometry.orientation.travel_angle_deg.value = 35;
  h.project.nodes = [edited]; h.mode.syncProgramWelds();
  const authoritative = h.mode.entries[0]!;
  assert.notEqual(authoritative, searching);
  for (const call of h.searches) call.resolve({ direction: "forward", plan: { clear_fraction: 1 } });
  await setImmediate();
  assert.equal(h.mode.entries[0], authoritative);
  assert.equal(authoritative.state, "accepted");
  assert.equal((authoritative.payload!.overrides.orientation as any).travel_angle_deg.value, 35);
  assert.equal(authoritative.plan, null);
  assert.equal(h.writes.length, 0);
  assert.equal(h.mode.busy, false);
});

test("reaccepting preserves authored non-Rails fields and the just-accepted measurements", async () => {
  const h = harness();
  const node = { ...authoredWeld(), enabled: false, standoff_mm: { kind: "constant", value: 22 }, weld_parameters: { current_a: 120 },
    extensions: { ...authoredWeld().extensions, owner_note: { value: "retain" } } };
  h.project.nodes = [node]; await h.ready();
  const entry = h.mode.entries[0]!;
  entry.plan = { knot_fractions: [0, 1], work_path_deg: [0, 0], travel_path_deg: [10, 10], direction: "forward", clear_fraction: 1 } as any;
  const plan = entry.plan;
  assert.equal(h.mode.openEntryForSeam(seam.id), true);
  h.detailCallbacks!.onDecide("accept");
  const saved = h.writes.at(-1) as typeof node;
  assert.equal(saved.enabled, false);
  assert.deepEqual(saved.standoff_mm, node.standoff_mm);
  assert.deepEqual(saved.weld_parameters, node.weld_parameters);
  assert.deepEqual(saved.extensions.owner_note, node.extensions.owner_note);
  h.project.nodes = [structuredClone(saved)]; h.mode.syncProgramWelds();
  assert.equal(h.mode.entries[0], entry);
  assert.equal(entry.plan, plan, "accept's immediate immutable node update does not invalidate its own measurement");
});
