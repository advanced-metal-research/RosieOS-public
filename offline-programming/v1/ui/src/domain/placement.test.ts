import assert from "node:assert/strict";
import test from "node:test";

import {
  ensureCadPathRows,
  rowsFromSeams,
} from "../adapters/cadquery.ts";
import {
  alignWorkpieceToCellWorkFrame,
  centreWorkpieceOnWorkFrame,
  needsCellWorkFrameRehang,
  needsWorkFrameMigration,
  setWorkpiecePlacement,
  transformWorkpiecePoint,
  worldPointToPlanner,
  workpieceAnchorM,
  workpiecePlacement,
} from "./placement.ts";
import {
  createProject,
  needsCellPinReconcile,
  reconcileCellPin,
  type OfflineProgrammingProject,
  type Rpy,
  type Vec3,
} from "./project.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import { torchGuideVectors } from "./weld.ts";

test("CAD workpiece is centred on the table with its lowest point on the surface", () => {
  const project = placementProject();
  assert.equal(needsWorkFrameMigration(project), true);
  assert.equal(centreWorkpieceOnWorkFrame(project), true);
  assert.equal(needsWorkFrameMigration(project), false);
  assert.deepEqual(workpiecePlacement(project), {
    offsetFromWorkFrameM: [0, 0, 0],
    rotationRpyRad: [0, 0, 0],
  });
  closeVec(
    transformWorkpiecePoint(project, [0, 0, 0.5]),
    [1.913, 0.6858, 0.6509],
  );
  closeVec(
    transformWorkpiecePoint(project, [1, 2, 1.5]),
    [2.913, 2.6858, 1.6509],
  );
});

test("workpiece offsets and rotation move CAD geometry around table centre", () => {
  const project = placementProject();
  assert.equal(setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0.1, -0.2, 0.03],
    rotationRpyRad: [0, 0, Math.PI / 2],
  }), true);
  const placement = workpiecePlacement(project);
  assert.ok(placement);
  closeVec(placement.offsetFromWorkFrameM, [0.1, -0.2, 0.03]);
  closeVec(
    transformWorkpiecePoint(project, [0, 0, 0.5]),
    [2.013, 0.4858, 0.6809],
  );
  closeVec(
    transformWorkpiecePoint(project, [1, 0, 0.5]),
    [2.013, 1.4858, 0.6809],
  );
});

test("CAD rows use the Motion Server base frame while preserving exact world placement", () => {
  const project = placementProject();
  centreWorkpieceOnWorkFrame(project);
  const centred = rowsFromSeams(project);
  const firstWeld = centred.find((row) => row.phase === "weld");
  assert.ok(firstWeld);
  assert.equal(firstWeld.workFrameId, "world");
  closeVec(
    firstWeld.targetTcpPose.slice(0, 3),
    worldPointToPlanner(project, transformWorkpiecePoint(project, [-1, -2, 0.5])),
  );

  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0.2, 0, 0],
    rotationRpyRad: [0, 0, 0],
  });
  const moved = rowsFromSeams(project);
  const movedFirstWeld = moved.find((row) => row.phase === "weld");
  assert.ok(movedFirstWeld);
  assert.ok(Math.abs(movedFirstWeld.targetTcpPose[0] - firstWeld.targetTcpPose[0] - 0.2) < 1e-9);
});

test("CAD weld rows carry torch RPY and approach/retract inherit adjacent frames", () => {
  const project = placementProject();
  project.seams[0]!.geometry.pointsM = [
    [-1, -2, 0.5],
    [0, -2, 0.5],
    [1, -2, 0.5],
  ];
  project.processParameters[0]!.torchWorkAngleDeg = 30;
  project.processParameters[0]!.torchTravelAngleDeg = 20;
  project.processParameters[0]!.torchSpinAngleDeg = 15;
  centreWorkpieceOnWorkFrame(project);

  const rows = rowsFromSeams(project);
  const welds = rows.filter((row) => row.phase === "weld");
  const approach = rows.find((row) => row.phase === "approach");
  const retract = rows.find((row) => row.phase === "retract");
  assert.equal(welds.length, 2);
  assert.ok(approach);
  assert.ok(retract);
  assert.deepEqual(approach.targetTcpPose.slice(3), welds[0]!.targetTcpPose.slice(3));
  assert.deepEqual(retract.targetTcpPose.slice(3), welds.at(-1)!.targetTcpPose.slice(3));
  assert.equal(approach.torchWorkAngleDeg, 30);
  assert.equal(retract.torchTravelAngleDeg, 20);

  const expected = torchGuideVectors([1, 0, 0], {
    workAngleDeg: 30,
    travelAngleDeg: 20,
    spinAngleDeg: 15,
  });
  assert.ok(expected);
  closeVec(
    rotateLocalAxis(welds[0]!.targetTcpPose.slice(3) as Rpy, [1, 0, 0]),
    expected.torchUp.map((component) => -component) as Vec3,
  );
});

test("vertical CAD seam tangents retain the workpiece placeholder RPY", () => {
  // A genuinely vertical seam (non-zero length) must fail closed — this is a
  // distinct property from the zero-length collapse below and both stay
  // covered (closure condition 1, 2026-08-04).
  const project = placementProject();
  project.seams[0]!.geometry.pointsM = [
    [0, 0, 0.5],
    [0, 0, 1.5],
  ];
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0, 0, 0],
    rotationRpyRad: [0, 0, Math.PI / 4],
  });
  const rows = rowsFromSeams(project);

  assert.ok(rows.length >= 3);
  for (const row of rows) {
    closeVec(row.targetTcpPose.slice(3), [0, 0, Math.PI / 4]);
  }
});

test("zero-length CAD seam collapse retains the workpiece placeholder RPY", () => {
  const project = placementProject();
  project.seams[0]!.geometry.pointsM = [
    [0, 0, 0.5],
    [0, 0, 0.5],
  ];
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0, 0, 0],
    rotationRpyRad: [0, 0, Math.PI / 4],
  });
  const rows = rowsFromSeams(project);

  assert.ok(rows.length >= 3);
  for (const row of rows) {
    closeVec(row.targetTcpPose.slice(3), [0, 0, Math.PI / 4]);
  }
});

test("multi-axis fixture placeholders use the same URDF RPY convention as weld rows", () => {
  const project = placementProject();
  project.seams[0]!.geometry.pointsM = [
    [0, 0, 0.5],
    [0, 0, 0.5],
  ];
  const fixtureRpy: Rpy = [0.31, -0.47, 0.83];
  setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0, 0, 0],
    rotationRpyRad: fixtureRpy,
  });
  const rows = rowsFromSeams(project);
  const placeholderRpy = rows[0]!.targetTcpPose.slice(3) as Rpy;
  // The fixture IS fixed-axis RPY, so the row's placeholder is the same
  // rotation read in the same convention: the same three numbers.
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]] as Vec3[]) {
    closeVec(
      rotateLocalAxis(placeholderRpy, axis),
      rotateLocalAxis(fixtureRpy, axis),
    );
  }
});

test("a compound fixture rotation turns the part the way the planner will read it", () => {
  // The planner applies `placement.rpy_rad` as URDF fixed-axis RPY,
  // Rz·Ry·Rx (seam_worker/robot/kinematics.py). The part drawn here must
  // turn by that same matrix, or the seams are planned against a part
  // rotated somewhere else -- which is what happened under Three.js "XYZ":
  // right for any single axis, tens of degrees off for [0.31, -0.47, 0.83].
  const project = placementProject();
  const rpy: Rpy = [0.31, -0.47, 0.83];
  setWorkpiecePlacement(project, { offsetFromWorkFrameM: [0, 0, 0], rotationRpyRad: rpy });
  const anchor = workpieceAnchorM(project)!;
  const origin = transformWorkpiecePoint(project, anchor);
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]] as Vec3[]) {
    const shifted: Vec3 = [anchor[0] + axis[0], anchor[1] + axis[1], anchor[2] + axis[2]];
    const tip = transformWorkpiecePoint(project, shifted);
    closeVec(
      [tip[0] - origin[0], tip[1] - origin[1], tip[2] - origin[2]],
      rotateLocalAxis(rpy, axis),
    );
  }
});

test("CAD row RPY is continuous across seams including approach and retract inheritance", () => {
  const rows = rowsFromSeams(createDogBoxProject());
  const motion = rows.filter((row) => (row.rowType ?? "motion") === "motion");
  for (let index = 1; index < motion.length; index += 1) {
    motion[index]!.targetTcpPose.slice(3).forEach((value, axis) => {
      assert.ok(
        Math.abs(value - motion[index - 1]!.targetTcpPose[axis + 3]!) <= Math.PI + 1e-12,
        `rows ${index - 1}/${index} axis ${axis} exceeded pi`,
      );
    });
  }
});

test("disabled CAD paths remain editable as disabled spreadsheet rows", () => {
  const project = placementProject();
  project.seams[0]!.enabled = false;

  const rows = rowsFromSeams(project);

  assert.ok(rows.length >= 3);
  assert.ok(rows.every((row) => row.seamId === "seam-1"));
  assert.ok(rows.every((row) => row.enabled === false));
  assert.ok(rows.some((row) => row.phase === "weld"));
});

test("legacy projects restore missing disabled CAD path rows", () => {
  const project = placementProject();
  project.seams[0]!.enabled = false;
  project.rows = [];

  assert.equal(ensureCadPathRows(project), true);
  assert.ok(project.rows.length >= 3);
  assert.ok(project.rows.every((row) => row.enabled === false));
  assert.equal(ensureCadPathRows(project), false);
});

function placementProject(
  robotModel: OfflineProgrammingProject["robot"]["model"] = "rosie_1400_v3",
): OfflineProgrammingProject {
  const project = createProject({ now: "2026-07-31T00:00:00.000Z", robotModel });
  project.seams = [{
    id: "seam-1",
    label: "WELD PATH 1",
    enabled: true,
    sourceEdgeId: "edge-1",
    partId: "part-1",
    geometryType: "LINE",
    trim: { startOffsetM: 0, endOffsetM: 0 },
    reversed: false,
    geometry: {
      kind: "polyline",
      pointsM: [
        [-1, -2, 0.5],
        [1, 2, 1.5],
      ],
    },
    workFrameId: "world",
    processParameterId: "process-default",
  }];
  project.sequence = [{
    id: "sequence-1",
    seamId: "seam-1",
    order: 0,
    approachOffsetM: 0.05,
    retractOffsetM: 0.05,
  }];
  return project;
}

// 0.1 mm. The work frame is the SERVER'S now -- walked out of the URDF at the
// zero pose -- where these expectations were written against the browser's
// rounded copy of it (1.913, 0.6858, 0.6509 for a frame that is really
// 1.9129999999999998, 0.68580026, 0.6508788). The two agree to 21 um, which is
// three orders of magnitude under anything these tests are about, and the
// arithmetic itself still has to land exactly.
const PLACEMENT_TOLERANCE_M = 1e-4;

function closeVec(
  actual: readonly number[],
  expected: readonly number[],
): void {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, index) => {
    assert.ok(
      Math.abs(value - expected[index]!) < PLACEMENT_TOLERANCE_M,
      `${value} != ${expected[index]} at ${index}`,
    );
  });
}

function rotateLocalAxis(rpy: Rpy, local: Vec3): Vec3 {
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    cy * cp * local[0] + (cy * sp * sr - sy * cr) * local[1] +
      (cy * sp * cr + sy * sr) * local[2],
    sy * cp * local[0] + (sy * sp * sr + cy * cr) * local[1] +
      (sy * sp * cr - cy * sr) * local[2],
    -sp * local[0] + cp * sr * local[1] + cp * cr * local[2],
  ];
}



test("a cell with no table places its part against the floor, not the 1400's table", () => {
  // `rosie_1420_v1` is an arm on a pedestal and the part is fixtured to the
  // floor, so the weld planner's work frame is the `world` link itself. The
  // offset authored here is the offset the planner reads; placing against the
  // 1400's table instead put every 1420 part 1.913 m along x from where the
  // planner looked for it, which is behind the robot.
  const project = placementProject("rosie_1420_v1");
  assert.equal(centreWorkpieceOnWorkFrame(project), true);
  // [0, 0, 0.5] is this part's anchor -- horizontal centre, lowest sampled
  // point -- so centring lands it ON the work-frame origin. The 1400 test
  // above is the same assertion against [1.913, 0.6858, 0.6509].
  closeVec(transformWorkpiecePoint(project, [0, 0, 0.5]), [0, 0, 0]);

  assert.equal(setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [1.0, 0, 0.3],
    rotationRpyRad: [0, 0, 0],
  }), true);
  // The part's lowest sampled point is its anchor, so the offset IS where that
  // point lands in world.
  closeVec(transformWorkpiecePoint(project, [0, 0, 0.5]), [1.0, 0, 0.3]);

  const placement = workpiecePlacement(project);
  assert.ok(placement);
  closeVec(placement.offsetFromWorkFrameM, [1.0, 0, 0.3]);
});

test("the work frame the fixture hangs off is the cell's own", () => {
  const table = placementProject("rosie_1400_v3");
  assert.equal(centreWorkpieceOnWorkFrame(table), true);
  // The description names the table's SURFACE point, which is the frame the
  // planner measures from; the browser used to name the link it hangs off.
  assert.equal(table.fixtures[0]?.frameId, "positioner_table_a_top");

  const floor = placementProject("rosie_1420_v1");
  assert.equal(centreWorkpieceOnWorkFrame(floor), true);
  assert.equal(floor.fixtures[0]?.frameId, "world");
  // `world` is the root. Given a parent it would be its own ancestor, and
  // `resolveFrameTransform` answers a cycle with the identity rather than
  // saying anything -- so the part would silently land at the origin.
  assert.equal(floor.frames.find((frame) => frame.id === "world")?.parentId, null);
  assert.equal(floor.frames.filter((frame) => frame.id === "world").length, 1);
});


test("switching the cell keeps the part where it is and re-measures the offset", () => {
  // The part is dragged to where it looks right beside the arm the viewer is
  // drawing, so the WORLD pose is what was authored. Preserving the offset
  // instead would move the part by the distance between the two cells' work
  // frames -- 1.913 m on x -- the moment the robot was switched.
  const project = placementProject("rosie_1400_v3");
  assert.equal(setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [-0.5, -0.299, 0.1],
    rotationRpyRad: [0, 0, Math.PI / 4],
  }), true);
  const anchorWorldBefore = transformWorkpiecePoint(project, [0, 0, 0.5]);
  closeVec(anchorWorldBefore, [1.413, 0.3868, 0.7509]);

  assert.equal(needsCellWorkFrameRehang(project), false);
  project.robot.model = "rosie_1420_v1";
  assert.equal(needsCellWorkFrameRehang(project), true);
  assert.equal(alignWorkpieceToCellWorkFrame(project), true);
  assert.equal(needsCellWorkFrameRehang(project), false);

  closeVec(transformWorkpiecePoint(project, [0, 0, 0.5]), anchorWorldBefore);
  assert.equal(project.fixtures[0]?.frameId, "world");
  // The 1420's work frame is the world origin, so the offset the planner now
  // reads IS the part's world position.
  const placement = workpiecePlacement(project);
  assert.ok(placement);
  closeVec(placement.offsetFromWorkFrameM, anchorWorldBefore);
  closeVec(placement.rotationRpyRad, [0, 0, Math.PI / 4]);
});

test("re-hanging an already-aligned part changes nothing", () => {
  const project = placementProject("rosie_1400_v3");
  assert.equal(setWorkpiecePlacement(project, {
    offsetFromWorkFrameM: [0.2, 0.3, 0.4],
    rotationRpyRad: [0, 0, 0],
  }), true);
  assert.equal(alignWorkpieceToCellWorkFrame(project), false);
  const placement = workpiecePlacement(project);
  assert.ok(placement);
  closeVec(placement.offsetFromWorkFrameM, [0.2, 0.3, 0.4]);
});


test("a project whose stored robot outran its program link is repaired", () => {
  // The state the selector cannot reach: the project already holds the 1420,
  // so the select renders at 1420 and picking it again fires no `change`.
  // The program link keeps the 1400 it was created with. The document itself
  // is written from `robot.model`, so it cannot fall behind the same way.
  const project = placementProject("rosie_1400_v3");
  project.robot.model = "rosie_1420_v1";
  assert.equal(project.program.roboticCellType, "rosie_1400_v3");
  assert.equal(needsCellPinReconcile(project), true);

  assert.equal(reconcileCellPin(project), true);
  assert.equal(project.program.roboticCellType, "rosie_1420_v1");

  assert.equal(needsCellPinReconcile(project), false);
  assert.equal(reconcileCellPin(project), false);
});

test("a consistent project is left alone", () => {
  const project = placementProject("rosie_1420_v1");
  assert.equal(needsCellPinReconcile(project), false);
  assert.equal(reconcileCellPin(project), false);
  assert.equal(project.program.roboticCellType, "rosie_1420_v1");
});
