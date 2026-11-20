import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { Euler, Quaternion } from "three";

import {
  parseRobotV4ProgramV2,
  programV2Problems,
  type ScalarFunction,
  type SeamGeometry,
  type WeldNode,
} from "../contracts/robot-v4-program-v2.ts";
import { applyAngles } from "../rails/frame-math.ts";
import {
  pathSegments,
  segmentArcLengthM,
  segmentDerivative,
  segmentPoint,
  type Continuity,
  type Quat,
  type V3,
  type Waypoint,
} from "../waypoints/model.ts";
import { isManualPoseWeld, manualPoseWeldNode, quatToRpy, rpyToQuat, waypointWeldOf } from "../waypoints/node.ts";
import { lowerManualPoseWeld, lowerManualPoseWelds, rotateByQuat, slerp } from "./manual-pose-seam.ts";

// Judged in degrees and metres against the planner's own forward math
// (SEAM-MODEL section 5, `rails/frame-math.ts`), never against the sampling
// the lowering chose.

const DEG = Math.PI / 180;

function quatFromAxisAngle(axis: V3, angleRad: number): Quat {
  const q = new Quaternion().setFromAxisAngle({ x: axis[0], y: axis[1], z: axis[2] } as never, angleRad);
  return [q.x, q.y, q.z, q.w];
}

function multiply(a: Quat, b: Quat): Quat {
  const q = new Quaternion(...a).multiply(new Quaternion(...b));
  return [q.x, q.y, q.z, q.w];
}

const DOWN = quatFromAxisAngle([1, 0, 0], Math.PI);

function waypoint(id: string, positionM: V3, quaternion: Quat, continuity: Continuity, standoffMm = 17): Waypoint {
  return { id, positionM, quaternion, continuity, standoffMm };
}

/** A torch that tilts and turns as it goes round an out-of-plane S, with
 *  each continuity once. */
function twisted(): Waypoint[] {
  const tilt = (deg: number) => multiply(quatFromAxisAngle([0, 1, 0], deg * DEG), DOWN);
  return [
    waypoint("a", [0, 0, 0], tilt(0), "c1"),
    waypoint("b", [0.05, 0, 0], tilt(20), "c1"),
    waypoint("c", [0.08, 0.03, 0.01], multiply(quatFromAxisAngle([0, 0, 1], 40 * DEG), tilt(20)), "c2"),
    waypoint("d", [0.1, 0.07, 0.01], tilt(-15), "c0", 22),
    waypoint("e", [0.14, 0.07, 0.0], tilt(-15), "c1", 22),
  ];
}

const PROCESS = { travelSpeedMmS: 8, weave: { shape: "none" } };

function evaluate(fn: ScalarFunction, s: number): number {
  assert.equal(fn.kind, "piecewise_linear");
  const knots = (fn as { knots: [number, number][] }).knots;
  for (let i = 1; i < knots.length; i += 1) {
    if (s <= knots[i]![0]) {
      const [s0, v0] = knots[i - 1]!;
      const [s1, v1] = knots[i]!;
      return v0 + ((s - s0) / (s1 - s0)) * (v1 - v0);
    }
  }
  return knots.at(-1)![1];
}

function angleDeg(a: V3, b: V3): number {
  const dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return Math.acos(Math.min(1, Math.max(-1, dot / (Math.hypot(...a) * Math.hypot(...b))))) / DEG;
}

function unit(a: V3): V3 {
  const n = Math.hypot(...a);
  return [a[0] / n, a[1] / n, a[2] / n];
}

/** The electrode direction the planner rebuilds at a point of the seam. */
function plannerElectrode(seam: SeamGeometry, s: number, tangent: V3): V3 {
  const d = (seam.reference as { fixed_direction_xyz: V3 }).fixed_direction_xyz;
  const along = d[0] * tangent[0] + d[1] * tangent[1] + d[2] * tangent[2];
  const r = unit([d[0] - along * tangent[0], d[1] - along * tangent[1], d[2] - along * tangent[2]]);
  const u = applyAngles(tangent, r, evaluate(seam.orientation.work_angle_deg, s), evaluate(seam.orientation.travel_angle_deg, s));
  return [-u[0], -u[1], -u[2]];
}

test("URDF roll-pitch-yaw and quaternions agree with three's ZYX order", () => {
  for (const rpy of [[0.3, -0.7, 1.1], [Math.PI, 0, 0], [-2.9, 0.4, -0.2]] as V3[]) {
    const expected = new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
    const q = rpyToQuat(rpy);
    assert.ok(Math.abs(Math.abs(q[0] * expected.x + q[1] * expected.y + q[2] * expected.z + q[3] * expected.w) - 1) < 1e-12);
    const back = rpyToQuat(quatToRpy(q));
    assert.ok(Math.abs(Math.abs(back[0] * q[0] + back[1] * q[1] + back[2] * q[2] + back[3] * q[3]) - 1) < 1e-12);
  }
});

test("a waypoint weld survives the node round trip, standoffs included", () => {
  const points = twisted();
  const node = manualPoseWeldNode("W", points, PROCESS);
  assert.ok(isManualPoseWeld(node));
  const back = waypointWeldOf(node).waypoints;
  assert.deepEqual(back.map((point) => point.id), points.map((point) => point.id));
  back.forEach((point, index) => {
    const original = points[index]!;
    assert.deepEqual(point.positionM, original.positionM);
    assert.equal(point.continuity, original.continuity);
    assert.ok(Math.abs(point.standoffMm - original.standoffMm) < 1e-9, `${point.standoffMm} vs ${original.standoffMm}`);
    assert.ok(angleDeg(rotateByQuat(point.quaternion, [0, 0, 1]), rotateByQuat(original.quaternion, [0, 0, 1])) < 1e-6);
  });
  assert.deepEqual(node.geometry.kind, "manual_pose_wp");
  assert.equal(node.standoff_mm.kind, "piecewise_linear");
});

test("the seam's curve is the waypoint path, exactly", () => {
  const points = twisted();
  const seam = lowerManualPoseWeld(manualPoseWeldNode("W", points, PROCESS) as never).geometry as SeamGeometry;
  const segments = pathSegments(points);
  assert.equal(seam.curve.segments.length, segments.length);
  seam.curve.segments.forEach((piece, index) => {
    const { degree, control_points_xyz_m: control } = piece as { degree: number; control_points_xyz_m: V3[] };
    for (const u of [0, 0.2, 0.5, 0.9, 1]) {
      const w = degree === 1 ? [1 - u, u] : [(1 - u) ** 3, 3 * u * (1 - u) ** 2, 3 * u * u * (1 - u), u ** 3];
      const point = [0, 1, 2].map((k) => w.reduce((sum, weight, j) => sum + weight * control[j]![k]!, 0)) as V3;
      const expected = segmentPoint(segments[index]!, u);
      assert.ok(Math.hypot(point[0] - expected[0], point[1] - expected[1], point[2] - expected[2]) < 1e-12);
    }
  });
  assert.deepEqual(
    seam.curve.segments.map((segment) => (segment as { continuity_to_next?: string }).continuity_to_next),
    ["C1", "C2", "C0", undefined],
  );
  // Only the gap from the C0 corner to the end is straight, so only it is a
  // line, where the planner's tracking bound is exact rather than a curve's.
  assert.deepEqual(seam.curve.segments.map((segment) => (segment as { degree: number }).degree), [3, 3, 3, 1]);
  const corner = [
    waypoint("a", [0, 0, 0], DOWN, "c1"),
    waypoint("b", [0.06, 0, 0], DOWN, "c0"),
    waypoint("c", [0.06, 0.04, 0], DOWN, "c0"),
  ];
  const lines = lowerManualPoseWeld(manualPoseWeldNode("L", corner, PROCESS) as never).geometry as SeamGeometry;
  assert.deepEqual(lines.curve.segments.map((segment) => (segment as { degree: number }).degree), [1, 1]);
});

test("the planner rebuilds each waypoint's electrode direction from the angles", () => {
  const points = twisted();
  const seam = lowerManualPoseWeld(manualPoseWeldNode("W", points, PROCESS) as never).geometry as SeamGeometry;
  const segments = pathSegments(points);
  const lengths = segments.map((segment) => segmentArcLengthM(segment));
  const total = lengths.reduce((sum, length) => sum + length, 0);
  let before = 0;
  segments.forEach((segment, index) => {
    // Each segment's two ends, with that segment's tangent: at the C0 corner
    // the two sides are two different knots.
    const start = before / total + (index > 0 && points[index]!.continuity === "c0" ? 1e-6 : 0);
    const end = (before + lengths[index]!) / total;
    for (const [s, u, q] of [[start, 0, points[index]!.quaternion], [end, 1, points[index + 1]!.quaternion]] as const) {
      const rebuilt = plannerElectrode(seam, Math.min(1, s), unit(segmentDerivative(segment, u)));
      const authored = rotateByQuat(q, [0, 0, 1]);
      assert.ok(angleDeg(rebuilt, authored) < 1e-6, `segment ${index} u=${u}: ${angleDeg(rebuilt, authored)}°`);
    }
    before += lengths[index]!;
  });
});

test("between knots the planner's torch stays within 0.2° of the authored slerp", () => {
  // A 90° swing in 20 mm, the steepest a hand-placed pair is likely to be.
  const points = [
    waypoint("a", [0, 0, 0], DOWN, "c1"),
    // About an axis off every frame direction, so work and travel both move
    // and neither moves linearly.
    waypoint("b", [0.02, 0, 0], multiply(quatFromAxisAngle(unit([1, 1, 1]), 90 * DEG), DOWN), "c1"),
  ];
  const seam = lowerManualPoseWeld(manualPoseWeldNode("W", points, PROCESS) as never).geometry as SeamGeometry;
  const [segment] = pathSegments(points);
  const length = segmentArcLengthM(segment!);
  let worst = 0;
  for (let k = 0; k <= 400; k += 1) {
    const u = k / 400;
    const along = segmentArcLengthM(segment!, u);
    const authored = rotateByQuat(slerp(points[0]!.quaternion, points[1]!.quaternion, along / length), [0, 0, 1]);
    const rebuilt = plannerElectrode(seam, along / length, unit(segmentDerivative(segment!, u)));
    worst = Math.max(worst, angleDeg(rebuilt, authored));
  }
  // 0.078° measured at 2 mm knots; 4 mm knots give 0.29° and fail here.
  assert.ok(worst < 0.2, `worst ${worst.toFixed(3)}°`);
});

test("a C0 corner is a jump between two knots, and the reference is never along the path", () => {
  const points = twisted();
  const seam = lowerManualPoseWeld(manualPoseWeldNode("W", points, PROCESS) as never).geometry as SeamGeometry;
  const knots = (seam.orientation.work_angle_deg as { knots: [number, number][] }).knots;
  for (let i = 1; i < knots.length; i += 1) assert.ok(knots[i]![0] > knots[i - 1]![0], `knot ${i} does not advance`);
  assert.equal(knots[0]![0], 0);
  assert.equal(knots.at(-1)![0], 1);
  const gaps = knots.slice(1).map((knot, i) => knot[0] - knots[i]![0]);
  assert.equal(gaps.filter((gap) => gap < 1e-5).length, 1, "one corner, one straddling pair");
  const d = (seam.reference as { fixed_direction_xyz: V3 }).fixed_direction_xyz;
  for (const segment of pathSegments(points)) {
    for (let k = 0; k <= 20; k += 1) {
      const t = unit(segmentDerivative(segment, k / 20));
      assert.ok(angleDeg(d, t) > 30 && angleDeg(d, t) < 150, `reference ${angleDeg(d, t)}° from the tangent`);
    }
  }
});

test("an arc lowers to the exact circle, and the planner rebuilds its torch at every knot", () => {
  const r = 0.04;
  const at = (deg: number): V3 => [r * Math.cos((deg * Math.PI) / 180), r * Math.sin((deg * Math.PI) / 180), 0];
  const tilt = (deg: number) => multiply(quatFromAxisAngle([1, 0, 0], deg * DEG), DOWN);
  const points = [
    waypoint("a", at(-60), tilt(0), "c0"),
    waypoint("b", at(40), tilt(15), "c0", 20),
    waypoint("c", at(170), tilt(-10), "c0", 14),
  ];
  const node = manualPoseWeldNode("A", points, PROCESS, { tool: "arc" });
  assert.equal((node.geometry as { shape?: string }).shape, "arc");
  const lowered = lowerManualPoseWeld(node as never);
  const seam = lowered.geometry as SeamGeometry;
  // 230° in pieces of at most 90°, each the rational conic of its half-angle.
  assert.equal(seam.curve.segments.length, 3);
  for (const piece of seam.curve.segments as { degree: number; weights: number[]; control_points_xyz_m: V3[] }[]) {
    assert.equal(piece.degree, 2);
    const [p0, p1, p2] = piece.control_points_xyz_m as [V3, V3, V3];
    const [w0, w1, w2] = piece.weights as [number, number, number];
    for (const u of [0.1, 0.5, 0.83]) {
      const b = [(1 - u) ** 2 * w0, 2 * u * (1 - u) * w1, u * u * w2];
      const sum = b[0]! + b[1]! + b[2]!;
      const q = [0, 1, 2].map((k) => (b[0]! * p0[k]! + b[1]! * p1[k]! + b[2]! * p2[k]!) / sum) as V3;
      assert.ok(Math.abs(Math.hypot(...q) - r) < 1e-12, "a point of the piece is off the circle");
    }
  }
  // The middle point is a knot of every profile, at its share of the sweep.
  const through = 100 / 230;
  const knots = (seam.orientation.work_angle_deg as { knots: [number, number][] }).knots;
  assert.ok(knots.some(([s]) => Math.abs(s - through) < 1e-12));
  // At each of the three points the planner's own rotations give the
  // authored electrode, with the circle's tangent there.
  for (const [s, deg, q] of [[0, -60, points[0]!.quaternion], [through, 40, points[1]!.quaternion], [1, 170, points[2]!.quaternion]] as const) {
    const a = (deg * Math.PI) / 180;
    const tangent: V3 = [-Math.sin(a), Math.cos(a), 0];
    const rebuilt = plannerElectrode(seam, s, tangent);
    assert.ok(angleDeg(rebuilt, rotateByQuat(q, [0, 0, 1])) < 1e-6, `at ${deg}°`);
  }
  // Standoffs sit on the same knots.
  assert.deepEqual((lowered.standoff_mm as { knots: [number, number][] }).knots.map(([s, v]) => [Number(s.toFixed(12)), v]),
    [[0, 17], [Number(through.toFixed(12)), 20], [1, 14]]);
});

test("an arc through three points in a row stops the plan", () => {
  const points = [
    waypoint("a", [0, 0, 0], DOWN, "c0"),
    waypoint("b", [0.01, 0, 0], DOWN, "c0"),
    waypoint("c", [0.02, 0, 0], DOWN, "c0"),
  ];
  assert.throws(() => lowerManualPoseWeld(manualPoseWeldNode("A", points, PROCESS, { tool: "arc" }) as never),
    /on a line/);
});

test("waypoints left on top of each other stop the plan, naming them", () => {
  const points = twisted();
  points[2] = { ...points[1]!, id: "copy" };
  assert.throws(() => lowerManualPoseWeld(manualPoseWeldNode("W", points, PROCESS) as never), /P2 and P3 are on top of each other/);
});

test("the lowered program is one the planner's validators accept", async () => {
  const fixture = parseRobotV4ProgramV2(JSON.parse(await readFile(
    new URL("../../../../../steamdeck/real/v4/contracts/programs/fixtures/complete-program-v2.json", import.meta.url),
    "utf8",
  )));
  assert.ok(fixture.nodes.some(isManualPoseWeld));
  const lowered = lowerManualPoseWelds(fixture);
  assert.ok(!lowered.nodes.some(isManualPoseWeld));
  assert.deepEqual(programV2Problems(lowered), []);
  const weld = lowered.nodes.find((node) => node.id === "weld_manual_corner") as WeldNode;
  assert.equal(weld.geometry.kind, "seam");
  // The node's own standoff and speed ride along unchanged.
  const original = fixture.nodes.find((node) => node.id === "weld_manual_corner") as WeldNode;
  assert.deepEqual(weld.standoff_mm, original.standoff_mm);
  assert.deepEqual(weld.travel_speed_mm_s, original.travel_speed_mm_s);
  // The fixture's L: straight legs, torch straight down, so no tilt anywhere.
  const travel = (weld.geometry as SeamGeometry).orientation.travel_angle_deg as { knots: [number, number][] };
  assert.ok(travel.knots.every(([, value]) => Math.abs(value) < 1e-9));
  // And the input document is untouched.
  assert.ok(fixture.nodes.some(isManualPoseWeld));
});
