import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { fileURLToPath } from "node:url";

import { createRailsSimulation } from "../src/execution/railsSimulation.ts";
import { decodeDenseTrajectory } from "../src/execution/denseTrajectoryBlob.ts";
import { createBlankSpreadsheetProject } from "../src/domain/spreadsheet.ts";
import type {
  DenseTrajectoryRecord,
  DenseTrajectorySegment,
  OfflineProgrammingProject,
} from "../src/domain/project.ts";
import type { SimulationSample } from "../src/simulation/contracts.ts";

/** The same reference blob the decoder and the Go twin answer to. */
async function referenceBlob(): Promise<Uint8Array> {
  return new Uint8Array(
    await readFile(
      fileURLToPath(
        new URL(
          "../../../../motion-server/joint-trajectory/v1/testdata/dense_joint_trajectory_reference.rdt",
          import.meta.url,
        ),
      ),
    ),
  );
}

function record(blob: Uint8Array, overrides: Partial<DenseTrajectoryRecord> = {}): DenseTrajectoryRecord {
  const decoded = decodeDenseTrajectory(blob);
  return {
    schema: decoded.schema,
    trajectoryDigest: decoded.trajectoryDigest,
    requestSha256: "sha256:request",
    planId: "plan-1",
    programId: "program-1",
    programDigest: "sha256:request",
    manifestRevision: 1,
    planRevision: 1,
    createdAt: "2026-09-10T09:00:00.000Z",
    dtS: decoded.dtS,
    totalSampleCount: decoded.totalSampleCount,
    totalDurationS: 1.0,
    bytes: blob.byteLength,
    segments: decoded.segments.map((segment) => ({
      index: segment.index,
      kind: segment.kind as DenseTrajectorySegment["kind"],
      sourceId: segment.sourceId,
      sampleCount: segment.samples.length,
      durationS: 0,
    })),
    sourceProjectRevision: 1,
    sourceProgramDigest: "sha256:planned",
    ...overrides,
  };
}

function planned(blob: Uint8Array, overrides?: Partial<DenseTrajectoryRecord>): OfflineProgrammingProject {
  const project = createBlankSpreadsheetProject({ name: "Rails", railsAuthoring: true });
  project.denseTrajectory = record(blob, overrides);
  return project;
}

function harness(blob: Uint8Array | null, openDigest = "sha256:planned") {
  const shown: (SimulationSample | null)[] = [];
  const changes: unknown[] = [];
  let nowMs = 0;
  const pending: (() => void)[] = [];
  const simulation = createRailsSimulation({
    show: (sample) => shown.push(sample),
    onChange: (snapshot) => changes.push(snapshot),
    readTrajectory: async () => blob,
    programDigest: async () => openDigest,
    now: () => nowMs,
    schedule: (step) => pending.push(step),
    cancel: () => pending.splice(0, pending.length),
  });
  return {
    simulation, shown, changes,
    advance(seconds: number) {
      nowMs += seconds * 1000;
      for (const step of pending.splice(0, pending.length)) step();
    },
  };
}

test("a planned program loads the bytes it planned and reports their provenance", async () => {
  const blob = await referenceBlob();
  const { simulation, shown } = harness(blob);
  const outcome = await simulation.load(planned(blob));

  assert.equal(outcome.state, "loaded");
  assert.equal(simulation.loaded, true);
  assert.ok(simulation.durationS > 0, "a loaded trajectory has a duration to scrub");
  assert.equal(simulation.provenance!.plannedAt, "2026-09-10T09:00:00.000Z");
  assert.equal(simulation.provenance!.programDigest, "sha256:planned");
  assert.equal(simulation.provenance!.current, true);
  // The viewer is posed from the blob's own joints -- what the robot plays.
  assert.equal(shown.at(-1)!.timeS, 0);
  assert.ok(shown.at(-1)!.jointsRad.length > 0);
});

test("a trajectory planned from another program is loaded, and says so", async () => {
  // Loaded rather than hidden: last plan's motion is still worth watching.
  // Believing it is the current program's is the failure to prevent.
  const blob = await referenceBlob();
  const { simulation } = harness(blob, "sha256:edited-since");
  await simulation.load(planned(blob));
  assert.equal(simulation.provenance!.current, false);
  assert.equal(simulation.loaded, true);
});

test("recheck answers again after the program moves under the trajectory", async () => {
  const blob = await referenceBlob();
  let openDigest = "sha256:planned";
  const shown: (SimulationSample | null)[] = [];
  const simulation = createRailsSimulation({
    show: (sample) => shown.push(sample),
    onChange: () => {},
    readTrajectory: async () => blob,
    programDigest: async () => openDigest,
    now: () => 0,
    schedule: () => 0,
    cancel: () => {},
  });
  const project = planned(blob);
  await simulation.load(project);
  assert.equal(simulation.provenance!.current, true);

  openDigest = "sha256:edited";
  await simulation.recheck(project);
  assert.equal(simulation.provenance!.current, false);
});

test("an unplanned program has nothing to simulate, and is not an error", async () => {
  const { simulation } = harness(null);
  const project = createBlankSpreadsheetProject({ name: "Rails", railsAuthoring: true });
  assert.equal((await simulation.load(project)).state, "unplanned");
  assert.equal(simulation.loaded, false);
});

test("a planned program whose bytes are gone reports missing, not empty", async () => {
  // The summary lives in the project and the bytes in a separate store; a
  // project restored where the store is not says so rather than showing a
  // Simulate button that does nothing.
  const blob = await referenceBlob();
  const { simulation } = harness(null);
  const outcome = await simulation.load(planned(blob));
  assert.equal(outcome.state, "missing");
  assert.equal(simulation.loaded, false);
});

test("bytes that do not decode are refused, never played", async () => {
  const blob = await referenceBlob();
  const truncated = blob.subarray(0, blob.byteLength - 200);
  const project = planned(blob);
  const { simulation } = harness(truncated);
  const outcome = await simulation.load(project);
  assert.equal(outcome.state, "unreadable");
  assert.equal(simulation.loaded, false);
});

test("playing walks the trajectory against the wall clock", async () => {
  const blob = await referenceBlob();
  const { simulation, shown, advance } = harness(blob);
  await simulation.load(planned(blob));
  shown.length = 0;
  simulation.play();
  assert.equal(simulation.playing, true);
  advance(0.05);
  assert.ok(shown.at(-1)!.timeS >= 0.04, "50 ms of wall clock should be 50 ms of trajectory");
  simulation.pause();
  assert.equal(simulation.playing, false);
});

test("seeking by fraction lands inside the trajectory", async () => {
  const blob = await referenceBlob();
  const { simulation, shown } = harness(blob);
  await simulation.load(planned(blob));
  shown.length = 0;
  simulation.seekFraction(0.5);
  const at = shown.at(-1)!.timeS;
  assert.ok(at > 0 && at <= simulation.durationS);
  // Out of range is clamped, not thrown: a range input can send either end.
  simulation.seekFraction(2);
  assert.ok(shown.at(-1)!.timeS <= simulation.durationS);
});

test("clearing forgets the trajectory and the provenance together", async () => {
  const blob = await referenceBlob();
  const { simulation } = harness(blob);
  await simulation.load(planned(blob));
  simulation.clear();
  assert.equal(simulation.loaded, false);
  assert.equal(simulation.provenance, null);
});
