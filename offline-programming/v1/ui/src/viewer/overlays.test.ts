import assert from "node:assert/strict";
import test from "node:test";
import * as THREE from "three";

import type { Rpy } from "../domain/project.ts";
import { torchGuideVectors, torchOrientationRpy } from "../domain/weld.ts";
import {
  applyTcpPose,
  createCurveCenterGroup,
  createMotionPathGroup,
} from "./overlays.ts";

test("spreadsheet path targets remain visible even for a single loaded row", () => {
  const group = createMotionPathGroup([{
    positionM: [0.8, 0.1, 0.7],
    motion: "travel",
  }]);
  const markers = group.children.find((child) =>
    child.userData.spreadsheetTargets === true
  );
  assert.ok(markers);
  assert.equal(markers.type, "Points");
  assert.equal(group.children.some((child) => child.type === "Line"), false);
});

test("TCP pose viewer decodes URDF RPY and aligns triad +X with the torch direction", () => {
  const torch = {
    workAngleDeg: 45,
    travelAngleDeg: 10,
    spinAngleDeg: 0,
  };
  const guide = torchGuideVectors([1, 0, 0], torch);
  const rpy = torchOrientationRpy([1, 0, 0], torch);
  assert.ok(guide);
  assert.ok(rpy);

  const object = new THREE.Object3D();
  applyTcpPose(object, [0, 0, 0, ...rpy]);
  const renderedToolX = new THREE.Vector3(1, 0, 0)
    .applyQuaternion(object.quaternion)
    .toArray();
  closeVec(renderedToolX, guide.torchUp.map((component) => -component));
});

test("URDF placeholder RPY preserves a multi-axis fixture's rendered rotation", () => {
  const fixtureRotation = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(0.31, -0.47, 0.83, "XYZ"),
  );
  const placeholder = new THREE.Euler().setFromQuaternion(fixtureRotation, "ZYX");
  const object = new THREE.Object3D();
  applyTcpPose(object, [0, 0, 0, placeholder.x, placeholder.y, placeholder.z]);
  assert.ok(object.quaternion.angleTo(fixtureRotation) < 1e-7);
});

test("accepted-plan URDF TCP poses use the same viewer boundary without double conversion", () => {
  const rpy: Rpy = [0.27, -0.38, 0.91];
  const object = new THREE.Object3D();
  applyTcpPose(object, [1, 2, 3, ...rpy]);
  const expected = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(...rpy, "ZYX"),
  );
  assert.ok(object.quaternion.angleTo(expected) < 1e-7);
  assert.deepEqual(object.position.toArray(), [1, 2, 3]);
});

test("curve centers render as distinct non-draggable handle markers", () => {
  const group = createCurveCenterGroup([{
    id: "move-curve-1",
    positionM: [0.4, 0.2, 0.7],
    selected: true,
  }]);
  assert.equal(group.children.length, 1);
  assert.equal(group.children[0]!.userData.curveCenter, true);
  assert.equal(group.children[0]!.userData.moveId, "move-curve-1");
  assert.deepEqual(group.children[0]!.position.toArray(), [0.4, 0.2, 0.7]);
});

function closeVec(
  actual: readonly number[],
  expected: readonly number[],
  tolerance = 1e-12,
): void {
  actual.forEach((value, index) => {
    assert.ok(Math.abs(value - expected[index]!) <= tolerance);
  });
}
