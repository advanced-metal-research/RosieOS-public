import assert from "node:assert/strict";
import test from "node:test";

import {
  canStepPlayback,
  moveBoundaryTimes,
  previewTrajectory,
  rowIdAtPlaybackTime,
  rowIdForTrajectorySample,
  sampleAtTime,
  stepPlaybackTime,
  trajectoryMotionSummary,
} from "../src/domain/playback.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

test("Dog Box browser preview is dense, timed, and weld-state preserving", () => {
  const project = createDogBoxProject();
  const trajectory = previewTrajectory(project.rows);
  assert.ok(trajectory.length > project.rows.length);
  assert.ok((trajectory.at(-1)?.timeS ?? 0) > 1);
  assert.ok(trajectory.some((sample) => sample.weldOn));
  assert.ok(trajectory.some((sample) => !sample.weldOn));
});

test("sampleAtTime interpolates TCP and reports completion", () => {
  const project = createDogBoxProject();
  const trajectory = previewTrajectory(project.rows.slice(0, 2));
  const endTime = trajectory.at(-1)!.timeS;
  const middle = sampleAtTime(trajectory, endTime / 2);
  assert.equal(middle.done, false);
  assert.ok(middle.sample);
  const done = sampleAtTime(trajectory, endTime + 1);
  assert.equal(done.done, true);
  assert.deepEqual(done.sample, trajectory.at(-1));
});

test("empty playback fails safely with no sample", () => {
  assert.deepEqual(sampleAtTime([], 4), { sample: null, done: true });
});

test("playback clock maps sample rowIndex to spreadsheet row id", () => {
  const project = createDogBoxProject();
  const rows = project.rows.slice(0, 3);
  const trajectory = previewTrajectory(rows);
  const startId = rowIdAtPlaybackTime(rows, trajectory, 0);
  assert.equal(startId, rows[0]!.id);

  const endTime = trajectory.at(-1)!.timeS;
  const endId = rowIdAtPlaybackTime(rows, trajectory, endTime);
  assert.equal(endId, rows[rows.length - 1]!.id);

  const midSample = sampleAtTime(trajectory, endTime / 2).sample;
  assert.ok(midSample);
  assert.equal(
    rowIdForTrajectorySample(rows, midSample),
    rows[midSample.rowIndex]!.id,
  );

  assert.equal(rowIdForTrajectorySample(rows, null), null);
  assert.equal(rowIdAtPlaybackTime(rows, [], 1), null);
  assert.equal(
    rowIdForTrajectorySample(rows, { ...midSample, rowIndex: 99 }),
    null,
  );
});

test("motion summary measures joint motion rather than a changing play state", () => {
  const project = createDogBoxProject();
  const rows = structuredClone(project.rows.slice(0, 2));
  rows[0]!.targetTcpPose = [1, 0, 0.8, 0, 0, 0];
  rows[1]!.targetTcpPose = [1, 0, 0.8, 0, 0, 0];
  rows[0]!.targetJointsRad = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  rows[1]!.targetJointsRad = [0.8, 0, 0, 0, 0, 0, 0, 0, 0];

  const moving = trajectoryMotionSummary(previewTrajectory(rows));
  assert.equal(moving.robotMoves, true);
  assert.ok(moving.jointTravelRad >= 0.8);

  rows[1]!.targetJointsRad = [...rows[0]!.targetJointsRad];
  const staticRobot = trajectoryMotionSummary(previewTrajectory(rows));
  assert.equal(staticRobot.robotMoves, false);
});

test("move boundaries follow distinct rowIndex runs", () => {
  const project = createDogBoxProject();
  const rows = project.rows.slice(0, 3);
  const trajectory = previewTrajectory(rows);
  const boundaries = moveBoundaryTimes(trajectory);
  assert.ok(boundaries.length >= 2);
  assert.equal(boundaries[0], trajectory[0]!.timeS);

  const rowIndexes = boundaries.map((timeS) => sampleAtTime(trajectory, timeS).sample?.rowIndex);
  assert.equal(new Set(rowIndexes).size, rowIndexes.length);
  assert.deepEqual(moveBoundaryTimes([]), []);
});

test("step forward/back targets neighboring move ends for animated execution", () => {
  const project = createDogBoxProject();
  const rows = project.rows.slice(0, 3);
  const trajectory = previewTrajectory(rows);
  const boundaries = moveBoundaryTimes(trajectory);
  assert.ok(boundaries.length >= 2);
  const endTime = trajectory.at(-1)!.timeS;

  assert.equal(stepPlaybackTime(trajectory, 0, "backward"), null);
  assert.equal(canStepPlayback(trajectory, 0, "backward"), false);

  // Forward from the first move start runs that move to the next boundary.
  const firstStep = stepPlaybackTime(trajectory, 0, "forward");
  assert.equal(firstStep, boundaries[1]);
  assert.equal(canStepPlayback(trajectory, 0, "forward"), true);
  assert.ok(firstStep! > 0, "step target must leave room to animate, not teleport");

  const midTime = (boundaries[0]! + boundaries[1]!) / 2;
  assert.equal(stepPlaybackTime(trajectory, midTime, "backward"), boundaries[0]);
  assert.equal(stepPlaybackTime(trajectory, midTime, "forward"), boundaries[1]);

  // Last move: forward goes to the planned trajectory end (not a no-op).
  const lastStart = boundaries.at(-1)!;
  assert.equal(stepPlaybackTime(trajectory, lastStart, "forward"), endTime);
  assert.equal(canStepPlayback(trajectory, lastStart, "forward"), endTime > lastStart);

  if (boundaries.length >= 2) {
    assert.equal(
      stepPlaybackTime(trajectory, lastStart, "backward"),
      boundaries.at(-2),
    );
  }

  if (endTime > lastStart) {
    assert.equal(stepPlaybackTime(trajectory, endTime, "backward"), lastStart);
    assert.equal(stepPlaybackTime(trajectory, endTime, "forward"), null);
  }

  assert.equal(stepPlaybackTime([], 1, "forward"), null);
});
