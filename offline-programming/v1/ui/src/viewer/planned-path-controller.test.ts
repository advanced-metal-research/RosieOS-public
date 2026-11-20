import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { createBlankSpreadsheetProject } from "../domain/spreadsheet.ts";
import { cloneProject, cellSettingsKey, type OfflineProgrammingProject } from "../domain/project.ts";
import { decodeDenseTrajectory, type DenseTrajectory } from "../execution/denseTrajectoryBlob.ts";
import { createPlannedPathController } from "./planned-path-controller.ts";

const bytes = new Uint8Array(await readFile(new URL(
  "../../../../../motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt", import.meta.url,
)));
const decoded = decodeDenseTrajectory(bytes);

function planned(): OfflineProgrammingProject {
  const project = createBlankSpreadsheetProject({ name: "Planned path", railsAuthoring: true });
  project.denseTrajectory = {
    schema: decoded.schema, trajectoryDigest: decoded.trajectoryDigest,
    requestSha256: "request", planId: "plan", programId: project.identity.id,
    programDigest: "program", manifestRevision: 1, planRevision: 1,
    createdAt: "2026-09-22T00:00:00Z", dtS: decoded.dtS,
    totalSampleCount: decoded.totalSampleCount, totalDurationS: 1,
    bytes: bytes.length, segments: [], sourceProjectRevision: project.identity.revision,
  };
  return project;
}

function deferred<T>() {
  let resolve!: (value: T) => void;
  let reject!: (error: unknown) => void;
  const promise = new Promise<T>((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

function harness(read: () => Promise<Uint8Array | ArrayBuffer | null> = async () => bytes) {
  const shown: { project: OfflineProgrammingProject; dense: DenseTrajectory | null }[] = [];
  const errors: unknown[] = [];
  let clears = 0;
  let reads = 0;
  const controller = createPlannedPathController({
    readTrajectory: () => { reads++; return read(); },
    clear: () => { clears++; },
    show: (project, dense) => { shown.push({ project, dense }); },
    onError: (error) => { errors.push(error); },
  });
  return { controller, shown, errors, clears: () => clears, reads: () => reads };
}

test("the active dense plan loads every segment once; duplicate sync shares work", async () => {
  const result = deferred<Uint8Array>();
  const h = harness(() => result.promise);
  const project = planned();
  const first = h.controller.sync(project);
  assert.equal(h.controller.sync(project), first);
  assert.equal(h.clears(), 1, "old scene is cleared before storage returns");
  result.resolve(bytes);
  await first;
  assert.deepEqual(h.shown[0]!.dense!.segments, decoded.segments);
  const selected = cloneProject(project);
  selected.simulation.selectedRowId = "another-row";
  await h.controller.sync(selected);
  assert.equal(h.reads(), 1, "selection changes do not reload or reset the view");
  assert.equal(h.shown.length, 1);
});

test("changing projects while storage is pending cannot install the old plan", async () => {
  const result = deferred<Uint8Array>();
  const h = harness(() => result.promise);
  const old = h.controller.sync(planned());
  const next = planned();
  next.denseTrajectory = null;
  await h.controller.sync(next);
  result.resolve(bytes);
  await old;
  assert.equal(h.shown.length, 0);
});

test("deleting the active plan and disposal invalidate asynchronous viewer preparation", async () => {
  for (const dispose of [false, true]) {
    const ready = deferred<void>();
    const started = deferred<void>();
    const installed: string[] = [];
    const controller = createPlannedPathController({
      readTrajectory: async () => bytes, clear() {},
      async show(project, _dense, isCurrent) {
        started.resolve();
        await ready.promise;
        if (isCurrent()) installed.push(project.identity.id);
      },
    });
    const project = planned();
    const pending = controller.sync(project);
    await started.promise;
    if (dispose) controller.dispose();
    else await controller.sync({ ...project, denseTrajectory: null });
    ready.resolve();
    await pending;
    assert.deepEqual(installed, []);
  }
});

test("missing, corrupt and mismatched stored bytes keep the previous path cleared", async () => {
  for (const data of [null, new Uint8Array([1, 2]), bytes.buffer.slice(0)]) {
    const h = harness(async () => data);
    const project = planned();
    if (data instanceof ArrayBuffer) project.denseTrajectory!.trajectoryDigest = "different";
    await h.controller.sync(project);
    assert.equal(h.shown.length, 0);
    assert.ok(h.clears() >= 1);
    assert.equal(h.errors.length, data === null ? 0 : 1);
  }
});

test("an old storage failure cannot clear a newly selected plan", async () => {
  const oldRead = deferred<Uint8Array>();
  let count = 0;
  const h = harness(() => ++count === 1 ? oldRead.promise : Promise.resolve(bytes));
  const old = h.controller.sync(planned());
  await h.controller.sync(planned());
  const clears = h.clears();
  oldRead.reject(new Error("old store failure"));
  await old;
  assert.equal(h.shown.length, 1);
  assert.equal(h.clears(), clears);
  assert.deepEqual(h.errors, []);
});

test("stale revisions and incompatible robot identities do not read or display dense motion", async () => {
  for (const reason of ["revision", "model", "description"] as const) {
    const h = harness();
    const project = planned();
    if (reason === "revision") project.identity.revision++;
    else project.denseTrajectory!.robot = {
      modelId: reason === "model" ? "other" : project.robot.model,
      robotDescriptionSha256: reason === "description" ? "other" : project.robot.description!.robotDescriptionSha256,
      machinePlanningCalibrationSha256: "", machinePlanningCalibrationSource: "empty",
    };
    await h.controller.sync(project);
    assert.equal(h.reads(), 0);
    assert.deepEqual(h.shown, []);
  }
});

test("only fresh classic trajectories are handed to the viewer without a dense lookup", async () => {
  const h = harness();
  const project = planned();
  project.denseTrajectory = null;
  project.plan = {
    ...project.plan, id: "classic", state: "server_validated", revision: 1,
    sourceProjectRevision: project.identity.revision, sourceRobotModel: project.robot.model,
    sourceCell: cellSettingsKey(project.robot.cell),
    trajectory: [{ index: 0, phase: "travel", timeS: 0, tcpPose: [0, 0, 0, 0, 0, 0], jointPositionsRad: [0, 0, 0, 0, 0, 0, 0, 0, 0], weldOn: false, seamId: null, rowIndex: 0 }],
  };
  await h.controller.sync(project);
  assert.equal(h.reads(), 0);
  assert.equal(h.shown.length, 1);
  assert.equal(h.shown[0]!.dense, null);
  for (const state of ["stale", "rejected", "missing"] as const) {
    project.plan = { ...project.plan, state };
    await h.controller.sync(project);
  }
  assert.equal(h.shown.length, 1);
});
