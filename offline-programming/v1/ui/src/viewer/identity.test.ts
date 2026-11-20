import test from "node:test";
import assert from "node:assert/strict";
import type { RobotPoseSnapshot } from "../simulation/contracts.ts";
import {
  PoseLatch,
  validateLoadedUrdfIdentity,
  validatePoseIdentity,
} from "./identity.ts";
import {
  CANONICAL_ROBOT_MODELS,
  CANONICAL_JOINT_ORDER,
  canonicalRobotModel,
} from "./models.ts";

const descriptor = canonicalRobotModel("rosie_1400_v3");

function pose(overrides: Partial<RobotPoseSnapshot> = {}): RobotPoseSnapshot {
  return {
    modelId: "rosie_1400_v3",
    jointOrder: CANONICAL_JOINT_ORDER,
    jointsRad: CANONICAL_JOINT_ORDER.map(() => 0),
    source: "live",
    observedAtMs: 1000,
    ...overrides,
  };
}

test("loaded URDF identity requires canonical name, joints, and frames", () => {
  const joints = Object.fromEntries(descriptor.urdfJointNames.map((name) => [name, {}]));
  const valid = validateLoadedUrdfIdentity(descriptor, {
    robotName: descriptor.urdfRobotName,
    joints,
    frames: { link_6: {}, tool0: {} },
  });
  assert.equal(valid.ok, true);

  const invalid = validateLoadedUrdfIdentity(descriptor, {
    robotName: "wrong",
    joints: {},
    frames: {},
  });
  assert.equal(invalid.ok, false);
  assert.ok(invalid.issues.some((issue) => issue.code === "robot-name-mismatch"));
  assert.ok(invalid.issues.some((issue) => issue.code === "missing-joint"));
  assert.ok(invalid.issues.some((issue) => issue.code === "missing-frame"));
});

test("pose identity rejects model, order, count, and finite-value mismatches", () => {
  const invalid = validatePoseIdentity(descriptor, pose({
    modelId: "rosie_1420_v1",
    jointOrder: [...CANONICAL_JOINT_ORDER].reverse(),
    jointsRad: [Number.NaN],
  }));
  const codes = invalid.issues.map((issue) => issue.code);
  assert.deepEqual(
    new Set(codes),
    new Set([
      "model-mismatch",
      "joint-order-mismatch",
      "joint-count-mismatch",
      "non-finite-joint",
    ]),
  );
});

test("pose latch falls back home, holds last known offline, and rejects bad poses", () => {
  const latch = new PoseLatch(descriptor);
  assert.equal(latch.resolve().mode, "home");
  assert.deepEqual(latch.resolve().pose.jointsRad, descriptor.homePoseRad);

  assert.equal(latch.accept(pose({ jointsRad: [1, 0, 0, 0, 0, 0, 0, 0, 0] })).ok, true);
  assert.equal(latch.resolve(1100).mode, "active");
  latch.markOffline();
  const offline = latch.resolve(1200);
  assert.equal(offline.mode, "last-known");
  assert.equal(offline.pose.jointsRad[0], 1);

  assert.equal(latch.accept(pose({ modelId: "wrong" })).ok, false);
  assert.equal(latch.resolve().pose.jointsRad[0], 1);
  latch.clear();
  assert.equal(latch.resolve().mode, "home");
});

test("every canonical robot Home pose is the exact J1-J9 zero position", () => {
  for (const model of Object.values(CANONICAL_ROBOT_MODELS)) {
    assert.deepEqual(model.poseJointOrder, CANONICAL_JOINT_ORDER);
    assert.deepEqual(model.homePoseRad, CANONICAL_JOINT_ORDER.map(() => 0));
  }
});

test("1420 descriptor preserves project J1-J9 identity but only applies URDF J1-J6", () => {
  const model = canonicalRobotModel("rosie_1420_v1");
  assert.deepEqual(model.poseJointOrder, CANONICAL_JOINT_ORDER);
  assert.deepEqual(model.urdfJointNames, CANONICAL_JOINT_ORDER.slice(0, 6));
});
