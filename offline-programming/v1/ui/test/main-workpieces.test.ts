import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createContext, runInContext } from "node:vm";
import ts from "typescript";

import { appendCadQueryTopology } from "../src/adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../src/adapters/weld-authoring.ts";
import type { CadSTEPSource, CadWorkpieceMesh } from "../src/cad/contracts.ts";
import { createProject, reviseProject, type AuthoringChange, type OfflineProgrammingProject } from "../src/domain/project.ts";
import { activeWorkpieceId, activateWorkpiece, environmentFixtures, projectWorkpieces } from "../src/domain/workpieces.ts";
import { newStepFixture, workcellFixtures } from "../src/domain/workcell.ts";
import { workpieceTopology } from "./support/workpiece-fixtures.ts";

const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
const ast = ts.createSourceFile("main.ts", source, ts.ScriptTarget.ES2022, true);
const names = ["importSTEP", "importStepBytes", "selectWorkpiece"];
const handlers = ast.statements.filter(node => ts.isFunctionDeclaration(node) && names.includes(node.name?.text ?? ""));
assert.equal(handlers.length, names.length);
const script = ts.transpileModule(handlers.map(node => node.getText(ast)).join("\n"), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.None },
}).outputText;

type StoredSource = { filename: string; fingerprint: string; bytes: Uint8Array };

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>(yes => { resolve = yes; });
  return { promise, resolve };
}

function harness(initial = createProject({ robotModel: "rosie_1420_v1" })) {
  const sources = new Map<string, StoredSource>();
  const statuses: string[] = [], storageStatuses: string[] = [], loaded: CadSTEPSource[] = [];
  const events: string[] = [], history: OfflineProgrammingProject[] = [];
  const buttons = new Map<string, { disabled: boolean; textContent: string; classList: { add(): void; remove(): void } }>();
  const failures = { storage: false, topology: new Set<string>(), tessellation: false };
  const state = {
    project: initial, stepImportInFlight: false, workpieceSwitchInFlight: false,
    stepImportKind: "weld", importingKind: "weld", importingPartFilename: null as string | null,
    importingWorkpieceId: null as string | null, activeCADSource: null as CadSTEPSource | null,
    editingLockReason: () => null,
    appendCadQueryTopology, activeWorkpieceId, activateWorkpiece, projectWorkpieces,
    newStepFixture, workcellFixtures, workcellFixtureMeshes: new Map<string, CadWorkpieceMesh>(),
    repository: {
      putSource: async (value: StoredSource) => {
        events.push(`persist:${value.filename}`);
        if (failures.storage) throw new Error("source storage unavailable");
        sources.set(value.fingerprint, value);
      },
      getSource: async (fingerprint: string) => sources.get(fingerprint),
    },
    cadClient: {
      extractTopology: async ({ source }: { source: CadSTEPSource }) => {
        const filename = source.filename!;
        events.push(`extract:${filename}`);
        if (failures.topology.has(filename)) throw new Error(`invalid STEP ${filename}`);
        const fingerprint = Buffer.from(filename).toString("hex").padEnd(64, "0").slice(0, 64);
        return workpieceTopology(filename, fingerprint);
      },
      tessellate: async ({ source }: { source: CadSTEPSource }) => {
        if (failures.tessellation) throw new Error("fixture tessellation unavailable");
        return { fingerprint: "f".repeat(64), filename: source.filename } as CadWorkpieceMesh;
      },
    },
    query: (id: string) => {
      if (!buttons.has(id)) buttons.set(id, { disabled: false, textContent: "", classList: { add() {}, remove() {} } });
      return buttons.get(id)!;
    },
    markBusy() {}, renderProgramTree() {}, refreshRecentParts() {}, resetWeldAuthoringUiState() {},
    setCADStatus: (text: string) => statuses.push(text),
    setStorageStatus: (text: string) => storageStatuses.push(text),
    history: { push: (value: OfflineProgrammingProject) => history.push(value) },
    changed: () => events.push(`commit:${state.project.cad.filename}`),
    mutate: (change: AuthoringChange, edit: (project: OfflineProgrammingProject) => void) => {
      state.project = reviseProject(state.project, change, edit);
      events.push(`mutate:${change}`);
    },
    selection: { kind: "part" }, setSelection: (value: { kind: string }) => { state.selection = value; },
    railsMode: { reset: () => events.push("rails-reset") }, railsStartup: Promise.resolve(),
    syncRailsModeToProject: () => events.push("rails-sync"),
    viewer: { workpieceGizmoEnabled: false }, railsWeldsCard: { shown: false }, waypointWeldsCard: { shown: false },
    setPartGizmo: () => events.push("close-gizmo"),
    setRailsWeldsOpen: () => events.push("close-rails-card"),
    setWaypointWeldsOpen: () => events.push("close-waypoints-card"),
    programTree: { select() {} }, treeIdOf: () => "part",
    loadWorkpieceMesh: async (source: CadSTEPSource) => {
      loaded.push(source);
      events.push(`mesh:${source.filename}`);
    },
    stepExamples: [], message: (error: Error) => error.message,
  };
  const context = createContext(state);
  runInContext(script, context);
  return {
    state, sources, failures, statuses, storageStatuses, loaded, history, events,
    import: async (filename: string, kind = "weld") => {
      Object.assign(context, { filename, kind });
      return await runInContext("importStepBytes(filename, new Uint8Array([1, 2]), kind)", context) as boolean;
    },
    files: async (files: { name: string; arrayBuffer(): Promise<ArrayBuffer> }[]) => {
      const input = { files, value: "selected" };
      Object.assign(context, { event: { currentTarget: input } });
      await runInContext("importSTEP(event)", context);
      return input;
    },
    select: async (id: string): Promise<boolean> => {
      Object.assign(context, { selectedId: id });
      return await runInContext("selectWorkpiece(selectedId)", context);
    },
  };
}

test("real import handlers append parts and persist their source before committing while retaining first-part welds", async () => {
  const h = harness();
  await h.import("first.step");
  h.state.project = createWeldsFromRecommendations(h.state.project);
  h.state.project.fixtures[0]!.translationM = [1, 2, 3];
  const original = structuredClone(projectWorkpieces(h.state.project)[0]!);
  await h.import("second.step");
  const parts = projectWorkpieces(h.state.project);
  assert.equal(parts.length, 2);
  assert.deepEqual(parts[0], original);
  assert.equal(h.state.project.identity.name, "first");
  assert.equal(h.sources.size, 2);
  assert.deepEqual(h.loaded.map(source => source.filename), ["first.step", "second.step"]);
  assert.equal(h.history.length, 2);
  assert.ok(h.events.indexOf("persist:second.step") < h.events.indexOf("commit:second.step"));
  assert.deepEqual(environmentFixtures(h.state.project).map(fixture => fixture.name), ["Stand", "first.step"]);
});

test("a multi-file selection continues after an invalid STEP and retains both successful files", async () => {
  const h = harness();
  h.failures.topology.add("broken.step");
  const input = await h.files(["first.step", "broken.step", "last.step"].map(name => ({
    name, arrayBuffer: async () => new Uint8Array([1]).buffer,
  })));
  assert.equal(input.value, "");
  assert.deepEqual(projectWorkpieces(h.state.project).map(part => part.cad.filename), ["first.step", "last.step"]);
  assert.ok(h.statuses.some(text => text.includes("invalid STEP broken.step")));
  assert.match(h.statuses.at(-1)!, /Imported 2 of 3 files\. Could not import: broken\.step/);
  assert.equal(h.state.stepImportInFlight, false);
  assert.equal(h.state.importingPartFilename, null);
});

test("adding a part closes tools belonging to the previous part before committing the new one", async () => {
  const h = harness();
  await h.import("first.step");
  h.events.length = 0;
  h.state.viewer.workpieceGizmoEnabled = true;
  h.state.railsWeldsCard.shown = true;
  h.state.waypointWeldsCard.shown = true;
  assert.equal(await h.import("second.step"), true);
  const committed = h.events.indexOf("commit:second.step");
  for (const closed of ["close-gizmo", "close-rails-card", "close-waypoints-card"]) {
    assert.ok(h.events.indexOf(closed) >= 0 && h.events.indexOf(closed) < committed, closed);
  }
});

for (const kind of ["weld", "fixture"]) {
  test(`${kind} import cannot add to a different program opened while the file is being read`, async () => {
    const h = harness();
    const bytes = deferred<ArrayBuffer>();
    h.state.stepImportKind = kind;
    const importing = h.files([{ name: "pending.step", arrayBuffer: () => bytes.promise }]);
    assert.equal(h.state.stepImportInFlight, true);
    const replacement = createProject({ name: "Different program" });
    h.state.project = replacement;
    bytes.resolve(new ArrayBuffer(1));
    await importing;
    assert.equal(h.state.project, replacement);
    assert.equal(projectWorkpieces(h.state.project).length, 0);
    assert.equal(h.loaded.length, 0);
    assert.equal(h.history.length, 0);
    assert.match(h.statuses.at(-1)!, /open program changed during import/);
    assert.equal(h.state.stepImportInFlight, false);
  });
}

test("a pending part switch cannot install a previous program's part into a newly opened program", async () => {
  const h = harness();
  await h.import("first.step");
  const first = projectWorkpieces(h.state.project)[0]!;
  const stored = h.sources.get(first.cad.fingerprint)!;
  await h.import("second.step");
  const source = deferred<StoredSource | undefined>();
  h.state.repository.getSource = () => source.promise;
  const selecting = h.select(first.id);
  assert.equal(h.state.workpieceSwitchInFlight, true);
  const replacement = createProject({ name: "Different program" });
  h.state.project = replacement;
  h.state.activeCADSource = null;
  source.resolve(stored);
  assert.equal(await selecting, false);
  assert.equal(h.state.project, replacement);
  assert.equal(h.state.activeCADSource, null);
  assert.equal(h.loaded.length, 2);
  assert.equal(h.state.workpieceSwitchInFlight, false);
});

test("opening another program during a batch cancels the remaining files instead of importing into it", async () => {
  const h = harness();
  const bytes = deferred<ArrayBuffer>();
  let secondRead = false;
  const importing = h.files([
    { name: "pending.step", arrayBuffer: () => bytes.promise },
    { name: "next.step", arrayBuffer: async () => { secondRead = true; return new ArrayBuffer(1); } },
  ]);
  const replacement = createProject({ name: "Different program" });
  h.state.project = replacement;
  bytes.resolve(new ArrayBuffer(1));
  await importing;
  assert.equal(h.state.project, replacement);
  assert.equal(secondRead, false);
  assert.equal(projectWorkpieces(h.state.project).length, 0);
  assert.equal(h.state.stepImportInFlight, false);
});

test("unknown part selection and selection during an import cannot mutate the program", async () => {
  const h = harness();
  await h.import("first.step");
  const previous = h.state.project;
  assert.equal(await h.select("missing"), false);
  h.state.stepImportInFlight = true;
  assert.equal(await h.select(activeWorkpieceId(previous)), false);
  assert.equal(h.state.project, previous);
  assert.match(h.storageStatuses.at(-1)!, /Wait for the current part/);
  assert.equal(h.state.workpieceSwitchInFlight, false);
});

test("fixture import uses the existing workspace fixture model and leaves the active weld program intact", async () => {
  const h = harness();
  await h.import("first.step");
  h.state.project = createWeldsFromRecommendations(h.state.project);
  const before = structuredClone(projectWorkpieces(h.state.project));
  const activeSource = h.state.activeCADSource;
  await h.import("clamp.step", "fixture");
  assert.deepEqual(projectWorkpieces(h.state.project), before);
  assert.equal(h.state.activeCADSource, activeSource);
  assert.equal(h.state.project.workcellFixtures.length, 2);
  assert.equal(h.state.project.workcellFixtures[0]!.id, "stand");
  assert.equal(h.state.project.workcellFixtures[1]!.shape.kind, "step");
  assert.equal(h.state.selection.kind, "fixture");
  assert.equal(h.loaded.length, 1);
  assert.equal(h.state.workcellFixtureMeshes.size, 1);
});

test("selecting an older part restores its source, welds, placement and surrounding geometry", async () => {
  const h = harness();
  await h.import("first.step");
  h.state.project = createWeldsFromRecommendations(h.state.project);
  h.state.project.fixtures[0]!.translationM = [0.7, 0.1, 1.5];
  const first = structuredClone(projectWorkpieces(h.state.project)[0]!);
  await h.import("second.step");
  assert.equal(await h.select(first.id), true);
  assert.equal(activeWorkpieceId(h.state.project), first.id);
  assert.deepEqual(h.state.project.nodes, first.nodes);
  assert.deepEqual(h.state.project.fixtures, first.fixtures);
  assert.equal(h.state.activeCADSource!.filename, "first.step");
  assert.equal(h.loaded.at(-1)!.filename, "first.step");
  assert.deepEqual(environmentFixtures(h.state.project).map(fixture => fixture.name), ["Stand", "second.step"]);
  assert.equal(h.state.workpieceSwitchInFlight, false);
});

for (const kind of ["weld", "fixture"]) {
  test(`${kind} source-storage failure does not replace or add to the current program`, async () => {
    const h = harness();
    await h.import("first.step");
    const previous = h.state.project;
    const activeSource = h.state.activeCADSource;
    h.failures.storage = true;
    await h.import("failed.step", kind);
    assert.equal(h.state.project, previous);
    assert.equal(h.state.activeCADSource, activeSource);
    assert.equal(h.state.workcellFixtureMeshes.size, 0);
    assert.match(h.statuses.at(-1)!, /source storage unavailable/);
    assert.equal(h.state.stepImportInFlight, false);
  });
}

test("selecting a part with a missing stored source leaves the current part and source intact", async () => {
  const h = harness();
  await h.import("first.step");
  const first = projectWorkpieces(h.state.project)[0]!;
  await h.import("second.step");
  h.sources.delete(first.cad.fingerprint);
  const previous = h.state.project;
  const activeSource = h.state.activeCADSource;
  assert.equal(await h.select(first.id), false);
  assert.equal(h.state.project, previous);
  assert.equal(h.state.activeCADSource, activeSource);
  assert.match(h.storageStatuses.at(-1)!, /missing from this browser/);
  assert.equal(h.state.workpieceSwitchInFlight, false);
});
