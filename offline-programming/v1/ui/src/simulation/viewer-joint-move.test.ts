import assert from "node:assert/strict";
import test from "node:test";

import { createDogBoxProject } from "../examples/dog-box.ts";
import type { JointVector, PlanTrajectorySample } from "../domain/project.ts";
import { toRobotV4Program } from "../adapters/robot-v4.ts";
import {
  createViewerJointMoveProject,
  homeJointTarget,
  jointsMatch,
  validateViewerJointMoveTrajectory,
} from "./viewer-joint-move.ts";

const START: JointVector = [0.6, -0.4, 0.3, 0.2, -0.1, 0.7, 0, 0, 0];
const ZERO = homeJointTarget();

function sample(
  index: number,
  timeS: number,
  joints: JointVector,
): PlanTrajectorySample {
  return {
    index,
    rowIndex: index === 0 ? 0 : 1,
    seamId: null,
    phase: "travel",
    weldOn: false,
    timeS,
    tcpPose: [0, 0, 0, 0, 0, 0],
    jointPositionsRad: [...joints],
  };
}

test("viewer Home is authored as a transient two-row Robot V4 JOINT move", () => {
  const source = createDogBoxProject("2026-07-31T00:00:00.000Z");
  const sourceRowsBefore = structuredClone(source.rows);
  const transient = createViewerJointMoveProject(source, {
    purpose: "home",
    startJointsRad: START,
    targetJointsRad: ZERO,
    localProfileId: "local-simulator",
  });

  assert.notEqual(transient, source);
  assert.equal(transient.target.mode, "local_simulator");
  assert.equal(transient.program.executionMode, "simulation");
  assert.equal(transient.rows.length, 2);
  assert.deepEqual(
    transient.rows.map((row) => row.moveType),
    ["JOINT", "JOINT"],
  );
  assert.deepEqual(transient.rows[0]!.targetJointsRad, START);
  assert.deepEqual(transient.rows[1]!.targetJointsRad, ZERO);
  assert.equal(transient.rows.every((row) => row.weldEnabled === false), true);
  assert.deepEqual(
    toRobotV4Program(transient).rows.map((row) => row.playback_step_ms),
    [0, 0],
  );
  assert.deepEqual(source.rows, sourceRowsBefore);
});

test("viewer joint motion accepts only timed Tesseract trajectories with exact endpoints", () => {
  const middle: JointVector = START.map((value) => value / 2) as JointVector;
  const trajectory = [
    sample(0, 0, START),
    sample(1, 1.5, middle),
    sample(2, 3, ZERO),
  ];
  assert.doesNotThrow(() =>
    validateViewerJointMoveTrajectory(trajectory, START, ZERO)
  );
  assert.equal(jointsMatch(trajectory[1]!.jointPositionsRad, START), false);
  assert.throws(
    () => validateViewerJointMoveTrajectory(
      [sample(0, 0, START), sample(1, 0, ZERO)],
      START,
      ZERO,
    ),
    /positive duration/,
  );
  assert.throws(
    () => validateViewerJointMoveTrajectory(
      [sample(0, 0, ZERO), sample(1, 1, ZERO)],
      START,
      ZERO,
    ),
    /does not start/,
  );
});
