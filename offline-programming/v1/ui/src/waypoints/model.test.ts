import assert from "node:assert/strict";
import test from "node:test";

import {
  TORCH_DOWN,
  arcThrough,
  pathSegments,
  samplePath,
  segmentArcLengthM,
  sharedContinuity,
  waypointAfter,
  waypointArcFractions,
  type Continuity,
  type V3,
  type Waypoint,
} from "./model.ts";

// The path is judged in metres and radians, never against its own sampling:
// the derivatives below are divided by the parameter step in metres, and the
// thresholds are fixed numbers.

const STEPS = 2000;

function path(points: V3[], continuity: Continuity): Waypoint[] {
  return points.map((positionM, i) => ({
    id: `p${i}`,
    positionM,
    quaternion: [...TORCH_DOWN],
    continuity,
    standoffMm: 17,
  }));
}

function sub(a: V3, b: V3): V3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function norm(a: V3): number {
  return Math.hypot(...a);
}

function chordLength(points: V3[], i: number): number {
  return norm(sub(points[i + 1]!, points[i]!));
}

/** First and second derivative (per metre of parameter) just before and just
 *  after waypoint `k`, from one-sided differences inside each segment. */
function derivativesAt(samples: V3[], points: V3[], k: number) {
  const j = k * STEPS;
  const dtLeft = chordLength(points, k - 1) / STEPS;
  const dtRight = chordLength(points, k) / STEPS;
  const at = (i: number) => samples[i]!;
  const first = (a: V3, b: V3, dt: number): V3 => sub(b, a).map((v) => v / dt) as V3;
  const second = (a: V3, b: V3, c: V3, dt: number): V3 =>
    [0, 1, 2].map((x) => (a[x]! - 2 * b[x]! + c[x]!) / (dt * dt)) as V3;
  return {
    velocityIn: first(at(j - 1), at(j), dtLeft),
    velocityOut: first(at(j), at(j + 1), dtRight),
    accelIn: second(at(j - 2), at(j - 1), at(j), dtLeft),
    accelOut: second(at(j), at(j + 1), at(j + 2), dtRight),
  };
}

function angleRad(a: V3, b: V3): number {
  const cos = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm(a) * norm(b));
  return Math.acos(Math.min(1, Math.max(-1, cos)));
}

// An L with uneven legs, then a turn out of plane: nothing symmetric to hide
// behind.
const POINTS: V3[] = [
  [0, 0, 0],
  [0.1, 0, 0],
  [0.1, 0.05, 0],
  [0.14, 0.05, 0.03],
];

test("a C0 path is the polyline through its waypoints", () => {
  const samples = samplePath(path(POINTS, "c0"), STEPS);
  assert.equal(samples.length, 3 * STEPS + 1);
  for (let i = 0; i + 1 < POINTS.length; i += 1) {
    const a = POINTS[i]!;
    const b = POINTS[i + 1]!;
    for (const k of [0, STEPS / 4, STEPS / 2, (3 * STEPS) / 4]) {
      const s = k / STEPS;
      const expected = [0, 1, 2].map((x) => a[x]! + (b[x]! - a[x]!) * s) as V3;
      assert.ok(norm(sub(samples[i * STEPS + k]!, expected)) < 1e-12, `segment ${i} at ${s}`);
    }
  }
  // The corner is really a corner: 90° at the first bend.
  const { velocityIn, velocityOut } = derivativesAt(samples, POINTS, 1);
  assert.ok(Math.abs(angleRad(velocityIn, velocityOut) - Math.PI / 2) < 1e-3);
});

test("every path passes through its waypoints", () => {
  for (const continuity of ["c0", "c1", "c2"] as const) {
    const samples = samplePath(path(POINTS, continuity), STEPS);
    POINTS.forEach((point, k) => {
      assert.ok(norm(sub(samples[k * STEPS]!, point)) < 1e-12, `${continuity} waypoint ${k}`);
    });
  }
});

test("C1 shares the tangent direction at a waypoint but not the curvature", () => {
  const samples = samplePath(path(POINTS, "c1"), STEPS);
  for (const k of [1, 2]) {
    const d = derivativesAt(samples, POINTS, k);
    assert.ok(angleRad(d.velocityIn, d.velocityOut) < 0.002, `tangent kink at ${k}`);
  }
  const d = derivativesAt(samples, POINTS, 1);
  // Metres per metre²: a C1 bend this tight jumps by tens of 1/m.
  assert.ok(norm(sub(d.accelIn, d.accelOut)) > 5, "C1 is not secretly C2");
});

test("C2 shares tangent and curvature at a waypoint", () => {
  const samples = samplePath(path(POINTS, "c2"), STEPS);
  for (const k of [1, 2]) {
    const d = derivativesAt(samples, POINTS, k);
    assert.ok(norm(sub(d.velocityIn, d.velocityOut)) < 0.005, `velocity jump at ${k}`);
    // Differences carry O(dt) error times the third derivative; 0.5 1/m is
    // a hundredth of the jump C1 shows at the same waypoint.
    assert.ok(norm(sub(d.accelIn, d.accelOut)) < 0.5, `curvature jump at ${k}`);
  }
});

test("a mixed path keeps each waypoint's own continuity", () => {
  const waypoints = path(POINTS, "c2");
  waypoints[1]!.continuity = "c0";
  const samples = samplePath(waypoints, STEPS);
  const corner = derivativesAt(samples, POINTS, 1);
  assert.ok(Math.abs(angleRad(corner.velocityIn, corner.velocityOut) - Math.PI / 2) < 1e-3);
  // The first segment runs between an end and a corner: straight.
  const mid = samples[STEPS / 2]!;
  assert.ok(norm(sub(mid, [0.05, 0, 0])) < 1e-12);
  const smooth = derivativesAt(samples, POINTS, 2);
  assert.ok(norm(sub(smooth.accelIn, smooth.accelOut)) < 0.5);
});

test("collinear C2 waypoints make a straight path", () => {
  const line: V3[] = [[0, 0, 0], [0.02, 0.01, 0], [0.1, 0.05, 0], [0.12, 0.06, 0]];
  for (const sample of samplePath(path(line, "c2"), 50)) {
    // Distance from the line through the origin along (2, 1, 0).
    const cross = sample[0] * 1 - sample[1] * 2;
    assert.ok(Math.abs(cross) / Math.sqrt(5) < 1e-12);
    assert.ok(Math.abs(sample[2]) < 1e-12);
  }
});

test("a new waypoint is an exact copy of the one it follows", () => {
  const points = path([[0, 0, 0], [0, 0.1, 0]], "c2");
  points[1]!.quaternion = [0, 0, 0.6, 0.8];
  points[1]!.standoffMm = 22;
  const next = waypointAfter(points, 1, [9, 9, 9], 17);
  assert.notEqual(next.id, points[1]!.id);
  assert.deepEqual(next.positionM, [0, 0.1, 0]);
  assert.deepEqual(next.quaternion, [0, 0, 0.6, 0.8]);
  assert.equal(next.continuity, "c2");
  assert.equal(next.standoffMm, 22);
  // A copy, not an alias: moving the new one leaves the old one put.
  next.positionM[0] = 1;
  assert.deepEqual(points[1]!.positionM, [0, 0.1, 0]);

  const first = waypointAfter([], -1, [0.2, 0, 0.05], 17);
  assert.deepEqual(first.positionM, [0.2, 0, 0.05]);
  assert.deepEqual(first.quaternion, TORCH_DOWN);
  assert.equal(first.standoffMm, 17);
  assert.equal(waypointAfter([], -1, [0, 0, 0], 500).standoffMm, 100);
});

test("a waypoint copied onto its predecessor draws a finite path", () => {
  const points = path([[0, 0, 0], [0.05, 0, 0], [0.05, 0, 0], [0.05, 0.05, 0]], "c1");
  for (const continuity of ["c0", "c1", "c2"] as const) {
    for (const point of points) point.continuity = continuity;
    for (const sample of samplePath(points, 8)) {
      assert.ok(sample.every(Number.isFinite), `${continuity}: ${sample}`);
    }
  }
});

test("the shared continuity is the interior waypoints', or none", () => {
  const points = path(POINTS, "c2");
  points[0]!.continuity = "c0";
  assert.equal(sharedContinuity(points), "c2");
  points[2]!.continuity = "c1";
  assert.equal(sharedContinuity(points), null);
  assert.equal(sharedContinuity(points.slice(0, 2)), null);
});

test("arc length is measured, not the chord", () => {
  // A C0 L: the path is its legs, 60 mm and 40 mm.
  const corner = path([[0, 0, 0], [0.06, 0, 0], [0.06, 0.04, 0]], "c0");
  assert.deepEqual(waypointArcFractions(corner).map((s) => Number(s.toFixed(12))), [0, 0.6, 1]);
  // A C1 quarter-turn is longer than its chords: each leg bows outward.
  const bent = path([[0, 0, 0], [0.06, 0, 0], [0.06, 0.04, 0]], "c1");
  const total = pathSegments(bent).reduce((sum, segment) => sum + segmentArcLengthM(segment), 0);
  assert.ok(total > 0.1 + 1e-4, `${total} m should exceed the 0.1 m of chord`);
  // Against a fine polyline of the same path, in absolute units.
  let polyline = 0;
  const samples = samplePath(bent, 4096);
  for (let i = 1; i < samples.length; i += 1) polyline += norm(sub(samples[i]!, samples[i - 1]!));
  assert.ok(Math.abs(total - polyline) < 1e-9, `quadrature ${total} vs polyline ${polyline}`);
});

test("an arc is the circle through its three points, the long way round when they say so", () => {
  const r = 0.03;
  const center: V3 = [0.1, -0.02, 0.05];
  // A tilted plane, and a 250° sweep: past a half turn, so the middle point
  // decides the direction and not the shorter way between the ends.
  const e1: V3 = [1 / Math.sqrt(2), 0, 1 / Math.sqrt(2)];
  const e2: V3 = [0, 1, 0];
  const at = (deg: number): V3 => {
    const a = (deg * Math.PI) / 180;
    return [0, 1, 2].map((k) => center[k]! + r * (Math.cos(a) * e1[k]! + Math.sin(a) * e2[k]!)) as V3;
  };
  const points = path([at(0), at(100), at(250)], "c1");
  const arc = arcThrough(points)!;
  assert.ok(norm(sub(arc.center, center)) < 1e-12);
  assert.ok(Math.abs(arc.radius - r) < 1e-12);
  assert.ok(Math.abs(arc.sweep - (250 * Math.PI) / 180) < 1e-12);
  assert.ok(Math.abs(arc.throughAngle - (100 * Math.PI) / 180) < 1e-12);
  assert.deepEqual(waypointArcFractions(points, "arc").map((s) => Number(s.toFixed(12))), [0, 0.4, 1]);
  for (const sample of samplePath(points, 8, "arc")) {
    assert.ok(Math.abs(norm(sub(sample, center)) - r) < 1e-12);
  }
  // The path starts and ends where the points are.
  const samples = samplePath(points, 8, "arc");
  assert.ok(norm(sub(samples[0]!, at(0))) < 1e-12 && norm(sub(samples.at(-1)!, at(250))) < 1e-12);
  // Three points in a row have no circle.
  assert.equal(arcThrough(path([[0, 0, 0], [0.01, 0.01, 0], [0.02, 0.02, 0]], "c1")), null);
});
