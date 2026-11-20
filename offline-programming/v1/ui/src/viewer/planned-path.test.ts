import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { URDFRobot, URDFJoint, URDFLink } from "urdf-loader/src/URDFClasses.js";
import { Window } from "happy-dom";
import { createProject } from "../domain/project.ts";
import { plannerPoseToWorld } from "../domain/placement.ts";
import type { DenseTrajectory } from "../execution/denseTrajectoryBlob.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import { densePlannedPathRuns, projectPlannedPath, tracePlannedTcpPath } from "./planned-path.ts";
import { canonicalRobotModel, CANONICAL_JOINT_ORDER } from "./models.ts";

function robotFixture() {
  const world = new THREE.Group();
  world.position.set(10, 20, 30);
  const robot = new URDFRobot();
  robot.position.set(1, 2, 3);
  const joint = new URDFJoint();
  joint.name = joint.urdfName = "J1";
  joint.jointType = "revolute";
  joint.axis.set(0, 0, 1);
  joint.limit = { lower: -0.5, upper: 0.5, effort: 100, velocity: 1 };
  const tcp = new URDFLink();
  tcp.name = tcp.urdfName = "tool0";
  tcp.position.set(2, 0, 0);
  robot.add(joint);
  joint.add(tcp);
  robot.joints = { J1: joint };
  robot.links = { tool0: tcp };
  world.add(robot);
  robot.setJointValue("J1", 0.2);
  world.updateMatrixWorld(true);
  return { robot, joint, tcp };
}

function sample(q: number, motion: "travel" | "weld" = "travel"): SimulationSample {
  return { modelId: "rosie_1400_v3", jointOrder: ["J1"], jointsRad: [q], source: "preview", timeS: 0, motion };
}

test("dense path keeps start, freespace, approach, weld, retract and end samples as separate runs", () => {
  const project = createProject();
  const kinds = ["freespace", "approach", "weld", "retract", "freespace"];
  const dense: DenseTrajectory = {
    schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: "test", dtS: 1, totalSampleCount: 15,
    segments: kinds.map((kind, index) => ({ index, kind, sourceId: `segment-${index}`, samples: [0, 0.5, 1].map((tS, i) => ({
      tS, qRad: new Float64Array([index * 10 + i, 0, 0, 0, 0, 0, 0, 0, 0]),
      qdRadS: new Float64Array(9), torchOn: kind === "weld",
    })) })),
  };
  const runs = densePlannedPathRuns(project, dense);
  assert.deepEqual(runs.map(run => run.length), [3, 3, 3, 3, 3]);
  assert.equal(runs[0]![0]!.jointsRad[0], 0);
  assert.equal(runs.at(-1)!.at(-1)!.jointsRad[0], 42);
  assert.equal(runs[0]!.at(-1)!.jointsRad[0], 2);
  assert.equal(runs[1]![0]!.jointsRad[0], 10, "separate segments do not invent connecting samples");
  assert.deepEqual(runs.map(run => run[0]!.motion), ["travel", "travel", "weld", "travel", "travel"]);
  assert.equal(runs.at(-1)!.at(-1)!.timeS, 5);
});

test("dense TCP uses every joint sample's curved FK in world coordinates without moving the visible robot", async () => {
  const { robot, joint, tcp } = robotFixture();
  const before = tcp.getWorldPosition(new THREE.Vector3()).clone();
  const runs = await tracePlannedTcpPath(robot, "tool0", ["J1"], [
    [sample(0), sample(Math.PI / 4), sample(Math.PI / 2, "weld")],
    [sample(Math.PI)],
  ]);
  assert.deepEqual(runs.map(run => run.length), [3, 1]);
  assert.deepEqual(runs[0]![0]!.positionM, [13, 22, 33]);
  assert.ok(Math.abs(runs[0]![1]!.positionM[0] - (11 + Math.SQRT2)) < 1e-10);
  assert.ok(Math.abs(runs[0]![1]!.positionM[1] - (22 + Math.SQRT2)) < 1e-10);
  assert.deepEqual(runs[0]![2]!.positionM, [11, 24, 33]);
  assert.equal(runs[0]![2]!.motion, "weld");
  assert.ok(Math.abs(runs[1]![0]!.positionM[0] - 9) < 1e-10, "FK must not clamp the plan to the viewer's joint limit");
  assert.equal(joint.jointValue[0], 0.2);
  assert.equal(joint.ignoreLimits, false);
  assert.ok(tcp.getWorldPosition(new THREE.Vector3()).equals(before));
});

test("long paths yield without dropping samples, and cancellation returns no partial path", async () => {
  const { robot } = robotFixture();
  const samples = Array.from({ length: 2050 }, (_, index) => sample(index / 2050));
  let yields = 0;
  const runs = await tracePlannedTcpPath(robot, "tool0", ["J1"], [samples], {
    yieldToUI: async () => { yields++; },
  });
  assert.equal(yields, 2);
  assert.equal(runs[0]!.length, samples.length);
  let current = true;
  const cancelled = await tracePlannedTcpPath(robot, "tool0", ["J1"], [samples], {
    isCurrent: () => current, yieldToUI: async () => { current = false; },
  });
  assert.deepEqual(cancelled, []);
});

test("missing frames and mismatched joint samples cannot produce a misleading partial path", async () => {
  const { robot } = robotFixture();
  await assert.rejects(tracePlannedTcpPath(robot, "missing", ["J1"], [[sample(0)]]), /cannot find robot frame/);
  await assert.rejects(tracePlannedTcpPath(robot, "tool0", ["missing"], [[sample(0)]]), /cannot find robot joint/);
  await assert.rejects(tracePlannedTcpPath(robot, "tool0", ["J1"], [[sample(0), sample(Number.NaN)]]), /does not match/);
});

test("classic planned path includes every accepted TCP sample in planner world coordinates", () => {
  const project = createProject();
  project.plan.trajectory = [0, 1, 2].map(index => ({
    index, phase: "travel", timeS: index, jointPositionsRad: [0, 0, 0, 0, 0, 0, 0, 0, 0], tcpPose: [index, index * 2, index * 3, 0, 0, 0],
    weldOn: index === 1, rowIndex: index, seamId: null,
  }));
  const path = projectPlannedPath(project);
  assert.equal(path.length, 3);
  assert.deepEqual(path.map(point => point.motion), ["travel", "weld", "travel"]);
  assert.deepEqual(path[0]!.positionM, plannerPoseToWorld(project, project.plan.trajectory[0]!.tcpPose).slice(0, 3));
  assert.deepEqual(path[2]!.positionM, plannerPoseToWorld(project, project.plan.trajectory[2]!.tcpPose).slice(0, 3));
});

for (const model of ["rosie_1400_v3", "rosie_1420_v1"] as const) {
  test(`${model} real URDF traces canonical nine-axis samples while preserving the viewer pose`, async () => {
    const browser = new Window();
    const keys = ["Document", "Element", "DOMParser"] as const;
    const before = keys.map(key => Object.getOwnPropertyDescriptor(globalThis, key));
    for (const key of keys) Object.defineProperty(globalThis, key, { configurable: true, value: browser[key] });
    try {
      const loader = new URDFLoader();
      loader.parseVisual = false;
      loader.parseCollision = false;
      const xml = await readFile(new URL(`../../../../../robot_description/robots/${model}/robot.urdf`, import.meta.url), "utf8");
      const robot = loader.parse(xml);
      const descriptor = canonicalRobotModel(model);
      const samples: SimulationSample[] = [0, 0.1, 0.2, 0.3].map(q => ({
        modelId: model, jointOrder: CANONICAL_JOINT_ORDER,
        jointsRad: [q, q / 2, -q / 2, 0, 0, q / 3, 0, 0, 0],
        source: "preview", timeS: q, motion: q === 0.2 ? "weld" : "travel",
      }));
      const original = Object.values(robot.joints).map(joint => [...joint.jointValue]);
      const path = await tracePlannedTcpPath(robot, descriptor.cameras.tcp.frame, descriptor.urdfJointNames, [samples]);
      assert.equal(path[0]!.length, 4);
      assert.notDeepEqual(path[0]![0]!.positionM, path[0]![3]!.positionM);
      const reference = robot.clone();
      for (let index = 0; index < samples.length; index++) {
        for (const name of descriptor.urdfJointNames) reference.setJointValue(name, samples[index]!.jointsRad[CANONICAL_JOINT_ORDER.indexOf(name as typeof CANONICAL_JOINT_ORDER[number])]!);
        reference.updateMatrixWorld(true);
        const expected = reference.links[descriptor.cameras.tcp.frame]!.getWorldPosition(new THREE.Vector3());
        const actual = new THREE.Vector3(...path[0]![index]!.positionM);
        assert.ok(actual.distanceTo(expected) < 1e-10);
      }
      assert.deepEqual(Object.values(robot.joints).map(joint => joint.jointValue), original);
    } finally {
      for (let index = 0; index < keys.length; index++) {
        if (before[index]) Object.defineProperty(globalThis, keys[index]!, before[index]!);
        else Reflect.deleteProperty(globalThis, keys[index]!);
      }
      void browser.happyDOM.close();
    }
  });
}
