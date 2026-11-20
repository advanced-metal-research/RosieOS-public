import assert from "node:assert/strict";
import test from "node:test";
import { Euler, Quaternion, Vector3 } from "three";
import { createDogBoxProject } from "../examples/dog-box.ts";
import {
  placementFromWorldAnchorPose,
  setWorkpiecePlacement,
  transformWorkpiecePoint,
  workpieceAnchorM,
  workpieceAnchorWorldM,
  workpiecePlacement,
  workpieceWorldRotationRpy,
} from "./placement.ts";
import { reconcileCellPin, type Rpy, type Vec3 } from "./project.ts";

function close(actual: readonly number[], expected: readonly number[], tol = 1e-9): void {
  assert.equal(actual.length, expected.length);
  for (const [i, value] of expected.entries()) {
    assert.ok(Math.abs(actual[i]! - value) < tol, `[${i}] ${actual[i]} vs ${value}`);
  }
}

// The gizmo drags a pivot at the anchor in the world and hands back its pose.
// Feeding that pose through placementFromWorldAnchorPose has to reproduce
// the placement the part already has -- otherwise releasing the mouse
// without moving would nudge the part.
for (const model of ["rosie_1400_v3", "rosie_1420_v1"] as const) {
  test(`${model}: a gizmo pose read back from the current placement is a no-op`, () => {
    const project = createDogBoxProject("2026-09-13T00:00:00.000Z");
    project.robot.model = model;
    reconcileCellPin(project);
    const authored = { offsetFromWorkFrameM: [0.25, -0.1, 0.02] as Vec3, rotationRpyRad: [0.4, -0.7, 1.1] as Rpy };
    assert.equal(setWorkpiecePlacement(project, authored), true);

    const anchorWorld = workpieceAnchorWorldM(project);
    assert.ok(anchorWorld);
    // The viewer's rotation, in the same "ZYX" reading the mesh applies.
    const rpy = workpieceWorldRotationRpy(project);
    const q = new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], "ZYX"));

    const back = placementFromWorldAnchorPose(project, anchorWorld, [q.x, q.y, q.z, q.w]);
    close(back.offsetFromWorkFrameM, authored.offsetFromWorkFrameM, 1e-9);
    close(back.rotationRpyRad, authored.rotationRpyRad, 1e-9);
  });
}

test("a translated gizmo pose moves the anchor by exactly the drag", () => {
  const project = createDogBoxProject("2026-09-13T00:00:00.000Z");
  project.robot.model = "rosie_1420_v1";
  reconcileCellPin(project);
  setWorkpiecePlacement(project, { offsetFromWorkFrameM: [1, 1, 0], rotationRpyRad: [0, 0, -Math.PI / 6] });
  const before = workpieceAnchorWorldM(project)!;
  const rpy = workpieceWorldRotationRpy(project);
  const q = new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
  const dragged: Vec3 = [before[0] + 0.1, before[1] - 0.05, before[2] + 0.2];
  setWorkpiecePlacement(project, placementFromWorldAnchorPose(project, dragged, [q.x, q.y, q.z, q.w]));
  close(workpieceAnchorWorldM(project)!, dragged, 1e-9);
  // The 1420's work frame is the world, so the offset is the anchor itself.
  close(workpiecePlacement(project)!.offsetFromWorkFrameM, dragged, 1e-9);
});

test("a rotated gizmo pose turns the part about the anchor, not the CAD origin", () => {
  const project = createDogBoxProject("2026-09-13T00:00:00.000Z");
  project.robot.model = "rosie_1400_v3";
  reconcileCellPin(project);
  setWorkpiecePlacement(project, { offsetFromWorkFrameM: [0.1, 0.2, 0], rotationRpyRad: [0, 0, 0] });
  const anchorWorld = workpieceAnchorWorldM(project)!;
  const anchorLocal = workpieceAnchorM(project)!;
  const turn = new Quaternion().setFromEuler(new Euler(0.3, -0.2, 0.9, "ZYX"));
  setWorkpiecePlacement(project, placementFromWorldAnchorPose(project, anchorWorld, [turn.x, turn.y, turn.z, turn.w]));
  // The anchor stays put...
  close(transformWorkpiecePoint(project, anchorLocal), anchorWorld, 1e-9);
  // ...and a point 10 cm along CAD +x from it lands where the turn puts it.
  const probe: Vec3 = [anchorLocal[0] + 0.1, anchorLocal[1], anchorLocal[2]];
  const expected = new Vector3(0.1, 0, 0).applyQuaternion(turn).add(new Vector3(...anchorWorld));
  close(transformWorkpiecePoint(project, probe), expected.toArray(), 1e-9);
});
