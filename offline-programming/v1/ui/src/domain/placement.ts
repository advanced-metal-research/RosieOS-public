import {
  Euler,
  Quaternion,
  Vector3,
} from "three";

import { armBaseWorldM } from "./robots.ts";
import {
  cellWorkFrame,
  type CellWorkFrame,
  type FixturePlacement,
  type OfflineProgrammingProject,
  type Rpy,
  type TcpPose,
  type Vec3,
} from "./project.ts";

/*
 * Project types remain the source of truth for each cell's work frame.
 * This module owns only placement math and migration.
 */

export interface WorkpiecePlacement {
  /**
   * The part's pose in the CELL'S WORK FRAME -- the 1400's table top, the
   * 1420's floor. Named for the frame and not for a table, because a cell
   * without one placed its parts 1.913 m from where the planner read them.
   */
  offsetFromWorkFrameM: Vec3;
  rotationRpyRad: Rpy;
}

interface RigidTransform {
  translation: Vector3;
  rotation: Quaternion;
}

export function hasWorkpieceGeometry(project: OfflineProgrammingProject): boolean {
  return anchorSourcePoints(project).length > 0;
}

/**
 * CAD-local points that define the part's placement anchor. The persisted
 * edge catalog is preferred because it covers the whole part and exists from
 * the moment of import (P2.12 imports have no seams yet); legacy projects
 * without a catalog keep their seam-derived anchor.
 */
function anchorSourcePoints(project: OfflineProgrammingProject): Vec3[] {
  const catalog = Array.isArray(project.edgeCatalog) ? project.edgeCatalog : [];
  const catalogPoints = catalog.flatMap((edge) => edge.samplesM);
  if (catalogPoints.length > 0) return catalogPoints;
  return project.seams.flatMap((seam) => seam.geometry.pointsM);
}

/**
 * The source-CAD point that is placed at the table centre by default:
 * horizontal bounds centre and the lowest sampled Z point.
 */
export function workpieceAnchorM(project: OfflineProgrammingProject): Vec3 | null {
  const points = anchorSourcePoints(project);
  if (points.length === 0) return null;
  // A running min/max, not Math.min(...xs): spreading every sampled point
  // as call arguments overflows the stack on a large import (a 65536-row
  // program's worth of seams, 2026-09-07).
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity;
  for (const point of points) {
    if (!Number.isFinite(point[0]) || !Number.isFinite(point[1]) || !Number.isFinite(point[2])) {
      return null;
    }
    if (point[0] < minX) minX = point[0];
    if (point[0] > maxX) maxX = point[0];
    if (point[1] < minY) minY = point[1];
    if (point[1] > maxY) maxY = point[1];
    if (point[2] < minZ) minZ = point[2];
  }
  return [(minX + maxX) / 2, (minY + maxY) / 2, minZ];
}

export function workpiecePlacement(
  project: OfflineProgrammingProject,
): WorkpiecePlacement | null {
  const fixture = project.fixtures[0];
  const anchor = workpieceAnchorM(project);
  if (!fixture || !anchor) return null;
  const rotatedAnchor = rotatePoint(anchor, fixture.rotationRpyRad);
  return {
    offsetFromWorkFrameM: addVec3(fixture.translationM, rotatedAnchor),
    rotationRpyRad: [...fixture.rotationRpyRad],
  };
}

/**
 * Set the workpiece pose relative to the CELL'S WORK FRAME. The raw CAD
 * geometry remains unchanged; only the fixture placement is authored.
 */
export function setWorkpiecePlacement(
  project: OfflineProgrammingProject,
  placement: WorkpiecePlacement,
): boolean {
  const anchor = workpieceAnchorM(project);
  if (!anchor) return false;
  const work = ensureCellWorkFrame(project);
  const fixture = ensurePrimaryFixture(project);
  const rotation: Rpy = finiteRpy(placement.rotationRpyRad);
  const offset = finiteVec3(placement.offsetFromWorkFrameM);
  const rotatedAnchor = rotatePoint(anchor, rotation);
  fixture.frameId = work.id;
  fixture.translationM = subtractVec3(offset, rotatedAnchor);
  fixture.rotationRpyRad = rotation;
  return true;
}

/** Where the placement anchor sits in the world: the point the viewer's
 *  drag gizmo pivots on, so a rotation turns the part about its own base
 *  centre and not about the CAD origin, which can be a metre away. */
export function workpieceAnchorWorldM(project: OfflineProgrammingProject): Vec3 | null {
  const anchor = workpieceAnchorM(project);
  if (!anchor || !project.fixtures[0]) return null;
  return transformWorkpiecePoint(project, anchor);
}

/**
 * The placement that puts the anchor at `anchorWorldM` with the part rotated
 * by `rotationXyzw` in the world. This is the gizmo's read-back: the viewer
 * drags a pivot in the world, and the fixture is authored in the cell's work
 * frame, so the pose is carried through the inverse of that frame. Rotation
 * components are written as fixed-axis RPY, which is what `rigidTransform`
 * reads and what the planner is sent.
 */
export function placementFromWorldAnchorPose(
  project: OfflineProgrammingProject,
  anchorWorldM: Vec3,
  rotationXyzw: readonly [number, number, number, number],
): WorkpiecePlacement {
  const work = cellWorkFrame(project.robot.model);
  const frame = resolveFrameTransform(project, work.id);
  const local = composeTransforms(invertTransform(frame), {
    translation: new Vector3(...finiteVec3(anchorWorldM)),
    rotation: new Quaternion(...rotationXyzw).normalize(),
  });
  const euler = new Euler().setFromQuaternion(local.rotation, "ZYX");
  return {
    offsetFromWorkFrameM: vectorTuple(local.translation),
    rotationRpyRad: [cleanNumber(euler.x), cleanNumber(euler.y), cleanNumber(euler.z)],
  };
}

/** Put the part at the cell's work-frame origin: the table centre on the
 *  1400, the floor origin on a cell that fixtures to the floor. */
export function centreWorkpieceOnWorkFrame(project: OfflineProgrammingProject): boolean {
  return setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0, 0, 0],
    rotationRpyRad: [0, 0, 0],
  });
}

export function transformWorkpiecePoint(
  project: OfflineProgrammingProject,
  pointM: Vec3,
): Vec3 {
  const fixture = project.fixtures[0];
  if (!fixture) return [...pointM];
  const frame = resolveFrameTransform(project, fixture.frameId);
  const local = rigidTransform(fixture.translationM, fixture.rotationRpyRad);
  const world = composeTransforms(frame, local);
  return vectorTuple(new Vector3(...pointM).applyQuaternion(world.rotation).add(world.translation));
}

export function worldPointToPlanner(
  project: OfflineProgrammingProject,
  pointM: Vec3,
): Vec3 {
  return subtractVec3(pointM, plannerOriginWorldM(project));
}

export function plannerPointToWorld(
  project: OfflineProgrammingProject,
  pointM: Vec3,
): Vec3 {
  return addVec3(pointM, plannerOriginWorldM(project));
}

export function plannerPoseToWorld(
  project: OfflineProgrammingProject,
  pose: Readonly<TcpPose>,
): TcpPose {
  const xyz = plannerPointToWorld(project, [pose[0], pose[1], pose[2]]);
  return [xyz[0], xyz[1], xyz[2], pose[3], pose[4], pose[5]];
}

export function workpieceWorldRotationRpy(
  project: OfflineProgrammingProject,
): Rpy {
  const fixture = project.fixtures[0];
  if (!fixture) return [0, 0, 0];
  const frame = resolveFrameTransform(project, fixture.frameId);
  const local = rigidTransform(fixture.translationM, fixture.rotationRpyRad);
  const world = composeTransforms(frame, local);
  // Row/TCP poses use URDF fixed-axis RPY (Rz · Ry · Rx), the same
  // convention the fixture is stored in; "ZYX" reads it back.
  const euler = new Euler().setFromQuaternion(world.rotation, "ZYX");
  return [cleanNumber(euler.x), cleanNumber(euler.y), cleanNumber(euler.z)];
}

/**
 * Re-hang the workpiece on the cell's own work frame, KEEPING IT WHERE IT IS
 * in the world. Returns whether anything moved.
 *
 * The frame a part hangs off is a property of the cell, and a project can
 * outlive the cell it was authored against: switching the robot changes which
 * link a workpiece is fixtured to, and every project saved before
 * `CELL_WORK_FRAME` existed hangs off the 1400's table whatever robot it
 * names. Both want the same repair.
 *
 * The WORLD pose is what is preserved, not the offset, because the world pose
 * is what was authored -- the part was dragged to where it looked right beside
 * the arm the viewer was drawing. Preserving the offset instead would teleport
 * it by the distance between the two cells' work frames, 1.913 m on x between
 * the 1400's table and the 1420's floor.
 */
export function alignWorkpieceToCellWorkFrame(
  project: OfflineProgrammingProject,
): boolean {
  const fixture = project.fixtures[0];
  if (!fixture) {
    ensureCellWorkFrame(project);
    return false;
  }
  const before = composeTransforms(
    resolveFrameTransform(project, fixture.frameId),
    rigidTransform(fixture.translationM, fixture.rotationRpyRad),
  );
  const work = ensureCellWorkFrame(project);
  const local = composeTransforms(
    invertTransform(resolveFrameTransform(project, work.id)),
    before,
  );
  const translation = vectorTuple(local.translation);
  // Written in the one convention `rigidTransform` reads: fixed-axis RPY,
  // Three.js "ZYX".
  const euler = new Euler().setFromQuaternion(local.rotation, "ZYX");
  const rotation: Rpy = [cleanNumber(euler.x), cleanNumber(euler.y), cleanNumber(euler.z)];
  const moved = fixture.frameId !== work.id ||
    !closeVec3(fixture.translationM, translation) ||
    !closeVec3(fixture.rotationRpyRad, rotation);
  fixture.frameId = work.id;
  fixture.translationM = translation;
  fixture.rotationRpyRad = rotation;
  return moved;
}

/** Is the workpiece hung off a frame that is not this cell's work frame? */
export function needsCellWorkFrameRehang(project: OfflineProgrammingProject): boolean {
  const fixture = project.fixtures[0];
  return fixture !== undefined && fixture.frameId !== cellWorkFrame(project.robot.model).id;
}

export function needsWorkFrameMigration(project: OfflineProgrammingProject): boolean {
  const fixture = project.fixtures[0];
  const work = cellWorkFrame(project.robot.model);
  const frame = project.frames.find((candidate) => candidate.id === work.id);
  return hasWorkpieceGeometry(project) &&
    fixture !== undefined &&
    (
      (nearZero(fixture.translationM) && nearZero(fixture.rotationRpyRad)) ||
      frame === undefined ||
      !closeVec3(frame.translationM, work.worldM) ||
      !nearZero(frame.rotationRpyRad)
    );
}

function ensurePrimaryFixture(project: OfflineProgrammingProject): FixturePlacement {
  const existing = project.fixtures[0];
  if (existing) return existing;
  const fixture: FixturePlacement = {
    id: "cad-workpiece",
    name: project.cad.filename || "Workpiece",
    frameId: cellWorkFrame(project.robot.model).id,
    sourceCadFingerprint: project.cad.fingerprint,
    translationM: [0, 0, 0],
    rotationRpyRad: [0, 0, 0],
  };
  project.fixtures.push(fixture);
  return fixture;
}

// Motion Server's Robot V4 TCP contract is expressed in the arm-base planning
// frame and the URDF viewer in the workstation world frame. Keep the
// conversion at those boundaries so authored rows are sent byte-for-byte to
// the same planner contract the physical controller uses while overlays still
// line up with the loaded workstation model. The offset is the robot's own,
// walked out of its URDF and served with the catalogue.
function plannerOriginWorldM(project: OfflineProgrammingProject): Vec3 {
  return armBaseWorldM(project.robot.model);
}

function ensureCellWorkFrame(project: OfflineProgrammingProject): CellWorkFrame {
  const work = cellWorkFrame(project.robot.model);
  const existing = project.frames.find((frame) => frame.id === work.id);
  if (existing) {
    // `world` is its own root and has no parent to name. Writing one would
    // make it its own ancestor, and `resolveFrameTransform` answers a cycle
    // with the identity instead of saying anything.
    existing.parentId = work.id === "world" ? null : "world";
    existing.translationM = [...work.worldM];
    existing.rotationRpyRad = [0, 0, 0];
    return work;
  }
  project.frames.push({
    id: work.id,
    parentId: work.id === "world" ? null : "world",
    translationM: [...work.worldM],
    rotationRpyRad: [0, 0, 0],
  });
  return work;
}

function resolveFrameTransform(
  project: OfflineProgrammingProject,
  frameId: string,
  visiting = new Set<string>(),
): RigidTransform {
  const frame = project.frames.find((candidate) => candidate.id === frameId);
  if (!frame || visiting.has(frame.id)) return identityTransform();
  const local = rigidTransform(frame.translationM, frame.rotationRpyRad);
  if (!frame.parentId) return local;
  visiting.add(frame.id);
  const parent = resolveFrameTransform(project, frame.parentId, visiting);
  visiting.delete(frame.id);
  return composeTransforms(parent, local);
}

/**
 * Every RPY triple in a project -- a fixture's, a frame's -- is URDF
 * fixed-axis roll-pitch-yaw, `R = Rz(yaw) · Ry(pitch) · Rx(roll)`. That is
 * what the weld planner reads the exported `placement.rpy_rad` as
 * (`seam_worker/robot/kinematics.py rpy_matrix`), and Three.js's "ZYX"
 * Euler order composes exactly that matrix from the same three numbers.
 *
 * It used to be Three.js "XYZ" (`Rx · Ry · Rz`). The two agree about any
 * rotation about one axis, which is why it held for so long, and disagree
 * about every compound one: a part turned about two axes was planned
 * against somewhere it was not drawn.
 */
function rigidTransform(translationM: Vec3, rotationRpyRad: Rpy): RigidTransform {
  return {
    translation: new Vector3(...finiteVec3(translationM)),
    rotation: new Quaternion().setFromEuler(new Euler(...finiteRpy(rotationRpyRad), "ZYX")),
  };
}

function invertTransform(transform: RigidTransform): RigidTransform {
  const rotation = transform.rotation.clone().invert();
  return {
    rotation,
    translation: transform.translation.clone().applyQuaternion(rotation).negate(),
  };
}

function identityTransform(): RigidTransform {
  return {
    translation: new Vector3(),
    rotation: new Quaternion(),
  };
}

function composeTransforms(parent: RigidTransform, local: RigidTransform): RigidTransform {
  return {
    translation: local.translation.clone().applyQuaternion(parent.rotation).add(parent.translation),
    rotation: parent.rotation.clone().multiply(local.rotation),
  };
}

function rotatePoint(point: Vec3, rotationRpyRad: Rpy): Vec3 {
  return vectorTuple(
    new Vector3(...point).applyEuler(new Euler(...finiteRpy(rotationRpyRad), "ZYX")),
  );
}

function vectorTuple(vector: Vector3): Vec3 {
  return [cleanNumber(vector.x), cleanNumber(vector.y), cleanNumber(vector.z)];
}

function addVec3(left: Vec3, right: Vec3): Vec3 {
  return [
    cleanNumber(left[0] + right[0]),
    cleanNumber(left[1] + right[1]),
    cleanNumber(left[2] + right[2]),
  ];
}

function subtractVec3(left: Vec3, right: Vec3): Vec3 {
  return [
    cleanNumber(left[0] - right[0]),
    cleanNumber(left[1] - right[1]),
    cleanNumber(left[2] - right[2]),
  ];
}

function finiteVec3(value: readonly number[]): Vec3 {
  return [
    finite(value[0]),
    finite(value[1]),
    finite(value[2]),
  ];
}

function finiteRpy(value: readonly number[]): Rpy {
  return [
    finite(value[0]),
    finite(value[1]),
    finite(value[2]),
  ];
}

function finite(value: number | undefined): number {
  return typeof value === "number" && Number.isFinite(value) ? cleanNumber(value) : 0;
}

function cleanNumber(value: number): number {
  return Math.abs(value) < 1e-12 ? 0 : value;
}

function nearZero(values: readonly number[]): boolean {
  return values.every((value) => Number.isFinite(value) && Math.abs(value) < 1e-12);
}

function closeVec3(left: Vec3, right: Vec3): boolean {
  return left.every((value, index) =>
    Number.isFinite(value) && Math.abs(value - right[index]!) < 1e-9
  );
}
