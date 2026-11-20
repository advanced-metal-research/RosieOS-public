/**
 * Hand-placed poses lowered to the seam the weld planner plans.
 *
 * One way only. A `manual_pose_wp` weld stays that in the program, where it
 * is edited; just before a plan request is packed, `lowerManualPoseWelds`
 * swaps each one for a `seam` weld the seam search reads. The weld planner
 * refuses a request that still carries one (`weldplan/plan_request.py`).
 *
 * What the seam carries:
 *
 * - curve: the path exactly, one curve segment per waypoint gap with the
 *   waypoint's continuity as `continuity_to_next`. A straight gap (both end
 *   tangents along its chord: between two C0 waypoints, or an end and a C0)
 *   is a degree-1 line; any other is the cubic Bezier of its Hermite. The
 *   difference is not cosmetic: the planner's tracking certificate is exact
 *   on a degree-1 line and only bounded on a curve
 *   (`weld_planner/v1/docs/issues/0008`), and before that bound existed a
 *   cubic emitted for a straight gap came back unverified.
 * - reference: a fixed vector. The planner builds r = d - (d.t)t from it and
 *   nothing else (it has no rotation-minimizing frame), so the angles below
 *   are measured against exactly that. d is the candidate furthest from
 *   parallel to the tangent anywhere on the path, since r is undefined where
 *   they meet.
 * - orientation: work and travel angle profiles, piecewise linear, sampled
 *   every `ANGLE_SAMPLE_STEP_M` from the schema's orientation rule (slerp by
 *   each segment's arc-length fraction) and inverted through SEAM-MODEL
 *   section 5. No limits: an unbanded seam is followed as authored. Spin is
 *   not carried -- the planner holds a weld to 5 DOF and picks its own.
 * - standoff, travel speed and weave: the node's, unchanged. They are
 *   functions of the same normalized arc length.
 */

import {
  MANUAL_POSE_MIN_CHORD_M,
  type ManualPoseGeometry,
  type RobotV4ProgramV2,
  type ScalarFunction,
  type SeamGeometry,
  type WeldNode,
} from "../contracts/robot-v4-program-v2.ts";
import {
  add,
  arcPoint,
  arcTangent,
  arcThrough,
  norm,
  pathSegments,
  scale,
  segmentArcLengthM,
  segmentDerivative,
  sub,
  type PathSegment,
  type Quat,
  type V3,
  type Waypoint,
} from "../waypoints/model.ts";
import { coincidentWaypoints, isManualPoseWeld, shapeOf, waypointWeldOf } from "../waypoints/node.ts";

/** How far apart the angle knots are along the weld: 2 mm. A torch turned
 *  90° about a skew axis between two waypoints 20 mm apart is read between
 *  knots within 0.078° of the slerp (0.29° at 4 mm, 0.020° at 1 mm;
 *  `manual-pose-seam.test.ts`, 2026-09-17). */
export const ANGLE_SAMPLE_STEP_M = 0.002;
/** At least this many knots per segment, however short. */
const MIN_SAMPLES_PER_SEGMENT = 4;
/** A C0 corner's angles jump; the jump is two knots this far apart in s. */
const CORNER_GAP_S = 1e-6;

type ManualPoseWeld = WeldNode & { geometry: ManualPoseGeometry };

/** The document with every manual_pose_wp weld lowered to a seam. */
export function lowerManualPoseWelds(document: RobotV4ProgramV2): RobotV4ProgramV2 {
  if (!document.nodes.some(isManualPoseWeld)) return document;
  return {
    ...document,
    nodes: document.nodes.map((node) => (isManualPoseWeld(node) ? lowerManualPoseWeld(node) : node)),
  };
}

interface Sample {
  s: number;
  tangent: V3;
  quaternion: Quat;
}

export function lowerManualPoseWeld(node: ManualPoseWeld): WeldNode {
  const weld = waypointWeldOf(node);
  const points = weld.waypoints;
  if (points.length < 2) throw new Error(`weld ${JSON.stringify(node.label)} needs at least two waypoints to plan`);
  const coincident = coincidentWaypoints(points, MANUAL_POSE_MIN_CHORD_M)[0];
  if (coincident !== undefined) {
    throw new Error(
      `weld ${JSON.stringify(node.label)}: P${coincident} and P${coincident + 1} are on top of each other — move one before planning`,
    );
  }
  const { samples, segments, total } = shapeOf(weld.tool) === "arc"
    ? arcSamples(node.label, points)
    : splineSamples(node.label, points);
  const electrode = samples.map((sample) => rotateByQuat(sample.quaternion, [0, 0, 1]));
  const reference = referenceDirection(samples, electrode);
  const workKnots: [number, number][] = [];
  const travelKnots: [number, number][] = [];
  let previousWork: number | null = null;
  samples.forEach((sample, index) => {
    const t = sample.tangent;
    const r = unit(sub(reference, scale(t, dot(reference, t))));
    const b = cross(t, r);
    // u points from the weld toward the torch body: against the electrode.
    const u = scale(electrode[index]!, -1);
    const travel = Math.asin(clamp(dot(u, t))) * DEG;
    let work = Math.atan2(dot(u, b), dot(u, r)) * DEG;
    // Unwrapped, so a torch that swings past ±180° is not read as flipping.
    if (previousWork !== null) work += 360 * Math.round((previousWork - work) / 360);
    previousWork = work;
    workKnots.push([sample.s, work]);
    travelKnots.push([sample.s, travel]);
  });

  const geometry: SeamGeometry = {
    kind: "seam",
    id: node.id,
    label: node.label,
    weld_joint_ref: null,
    curve: {
      segments,
      closed: false,
      length_m: total,
    },
    span: { start_s: 0, end_s: 1 },
    direction: "forward",
    reference: { mode: "fixed_vector", fixed_direction_xyz: reference, semantics: "custom" },
    orientation: {
      work_angle_deg: profile(workKnots),
      travel_angle_deg: profile(travelKnots),
    },
  } as SeamGeometry;
  return { ...structuredClone(node), geometry };
}

/** Both end tangents along the chord: the Hermite is the chord itself, at
 *  uniform speed, so the degree-1 line is the same curve, parameter and all. */
function isStraight(segment: PathSegment): boolean {
  const chord = scale(sub(segment.p1, segment.p0), 1 / segment.h);
  return norm(sub(segment.m0, chord)) < STRAIGHT_TOL && norm(sub(segment.m1, chord)) < STRAIGHT_TOL;
}

/** How far a unit tangent may sit from its unit chord and still be read as
 *  straight: 1e-9 rad. The C0 rule gives the chord exactly; the spline solve
 *  at a natural end next to a C0 gives it to rounding. */
const STRAIGHT_TOL = 1e-9;

interface Lowered {
  samples: Sample[];
  segments: CurveSegment[];
  total: number;
}

type CurveSegment = SeamGeometry["curve"]["segments"][number];

/** The waypoint rule: one Hermite per gap, a line where the gap is straight. */
function splineSamples(label: string, points: readonly Waypoint[]): Lowered {
  const segments = pathSegments(points);
  const lengths = segments.map((segment) => segmentArcLengthM(segment));
  const total = lengths.reduce((sum, length) => sum + length, 0);
  if (!(total > 0)) throw new Error(`weld ${JSON.stringify(label)} has no length to weld`);
  const samples: Sample[] = [];
  let before = 0;
  segments.forEach((segment, index) => {
    const length = lengths[index]!;
    const count = Math.max(MIN_SAMPLES_PER_SEGMENT, Math.ceil(length / ANGLE_SAMPLE_STEP_M));
    const q0 = points[index]!.quaternion;
    const q1 = points[index + 1]!.quaternion;
    const corner = index > 0 && points[index]!.continuity === "c0";
    for (let k = 0; k <= count; k += 1) {
      // The junction is the previous segment's last sample, unless the path
      // turns a corner there: then this side's tangent needs its own knot.
      if (k === 0 && index > 0 && !corner) continue;
      const u = k / count;
      const along = k === 0 ? 0 : k === count ? length : segmentArcLengthM(segment, u);
      let s = (before + along) / total;
      if (k === 0 && corner) s += CORNER_GAP_S;
      samples.push({
        s,
        tangent: unit(segmentDerivative(segment, u)),
        quaternion: slerp(q0, q1, length > 0 ? along / length : 0),
      });
    }
    before += length;
  });
  finish(samples);
  return {
    samples,
    total,
    segments: segments.map((segment, index) => ({
      ...(isStraight(segment)
        ? { degree: 1, control_points_xyz_m: [segment.p0, segment.p1], knots_u: [0, 0, 1, 1] }
        : {
          degree: 3,
          control_points_xyz_m: [
            segment.p0,
            add(segment.p0, scale(segment.m0, segment.h / 3)),
            sub(segment.p1, scale(segment.m1, segment.h / 3)),
            segment.p1,
          ],
          knots_u: [0, 0, 0, 0, 1, 1, 1, 1],
        }),
      length_m: lengths[index]!,
      ...(index + 1 < segments.length
        ? { continuity_to_next: points[index + 1]!.continuity.toUpperCase() as "C0" | "C1" | "C2" }
        : {}),
    })),
  };
}

/**
 * The arc rule: the circle through the three points, exactly, as rational
 * quadratic pieces of at most 90 degrees (weight `cos(half the piece)`, the
 * textbook conic). Arc length is the angle times the radius, so `s` is the
 * angle's share of the sweep and the second point sits at a knot of every
 * profile, where its orientation is authored.
 */
function arcSamples(label: string, points: readonly Waypoint[]): Lowered {
  const arc = arcThrough(points);
  if (!arc) throw new Error(`arc ${JSON.stringify(label)}: its three points are on a line — move P2 off it before planning`);
  const total = arc.radius * arc.sweep;
  const samples: Sample[] = [];
  const legs: [number, number, Quat, Quat][] = [
    [0, arc.throughAngle, points[0]!.quaternion, points[1]!.quaternion],
    [arc.throughAngle, arc.sweep, points[1]!.quaternion, points[2]!.quaternion],
  ];
  legs.forEach(([from, to, q0, q1], leg) => {
    const count = Math.max(MIN_SAMPLES_PER_SEGMENT, Math.ceil((arc.radius * (to - from)) / ANGLE_SAMPLE_STEP_M));
    for (let k = leg === 0 ? 0 : 1; k <= count; k += 1) {
      const angle = from + ((to - from) * k) / count;
      samples.push({
        s: angle / arc.sweep,
        tangent: arcTangent(arc, angle),
        quaternion: slerp(q0, q1, k / count),
      });
    }
  });
  finish(samples);
  const pieces = Math.max(1, Math.ceil(arc.sweep / (Math.PI / 2) - 1e-9));
  const step = arc.sweep / pieces;
  const segments: CurveSegment[] = Array.from({ length: pieces }, (_, index) => {
    const a = index * step;
    const b = a + step;
    const middle = add(arc.center, scale(
      add(scale(arc.u, Math.cos(a + step / 2)), scale(arc.v, Math.sin(a + step / 2))),
      arc.radius / Math.cos(step / 2),
    ));
    return {
      degree: 2,
      control_points_xyz_m: [arcPoint(arc, a), middle, arcPoint(arc, b)],
      weights: [1, Math.cos(step / 2), 1],
      knots_u: [0, 0, 0, 1, 1, 1],
      length_m: arc.radius * step,
      ...(index + 1 < pieces ? { continuity_to_next: "C2" as const } : {}),
    };
  });
  return { samples, segments, total };
}

function finish(samples: Sample[]): void {
  samples[0]!.s = 0;
  samples[samples.length - 1]!.s = 1;
}

function profile(knots: [number, number][]): ScalarFunction {
  return { kind: "piecewise_linear", knots };
}

/** The candidate least parallel to the tangent anywhere on the path: the
 *  mean torch-body direction first (the natural 0° for a torch that stays
 *  roughly put), then the part's axes. */
function referenceDirection(samples: readonly Sample[], electrode: readonly V3[]): V3 {
  const mean = electrode.reduce<V3>((sum, e) => sub(sum, e), [0, 0, 0]);
  const candidates: V3[] = [];
  if (norm(mean) > 1e-9) candidates.push(unit(mean));
  candidates.push([0, 0, 1], [1, 0, 0], [0, 1, 0]);
  let best = candidates[0]!;
  let bestClearance = -1;
  for (const candidate of candidates) {
    const clearance = Math.min(...samples.map((sample) => norm(cross(candidate, sample.tangent))));
    if (clearance > bestClearance + 1e-9) {
      best = candidate;
      bestClearance = clearance;
    }
  }
  return best;
}

const DEG = 180 / Math.PI;

function dot(a: V3, b: V3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a: V3, b: V3): V3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function unit(a: V3): V3 {
  const length = norm(a);
  return length > 0 ? scale(a, 1 / length) : [0, 0, 0];
}

function clamp(value: number): number {
  return Math.min(1, Math.max(-1, value));
}

/** Shortest-arc spherical interpolation, the schema's orientation rule. */
export function slerp(a: Quat, b: Quat, t: number): Quat {
  let [bx, by, bz, bw] = b;
  let cos = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
  if (cos < 0) {
    [bx, by, bz, bw] = [-bx, -by, -bz, -bw];
    cos = -cos;
  }
  let wa: number;
  let wb: number;
  if (cos > 0.9995) {
    wa = 1 - t;
    wb = t;
  } else {
    const angle = Math.acos(cos);
    const sin = Math.sin(angle);
    wa = Math.sin((1 - t) * angle) / sin;
    wb = Math.sin(t * angle) / sin;
  }
  const out: Quat = [wa * a[0] + wb * bx, wa * a[1] + wb * by, wa * a[2] + wb * bz, wa * a[3] + wb * bw];
  const length = Math.hypot(...out) || 1;
  return [out[0] / length, out[1] / length, out[2] / length, out[3] / length];
}

/** `v` rotated by the unit quaternion `q` (x y z w). */
export function rotateByQuat([x, y, z, w]: Quat, v: V3): V3 {
  // v + 2w(q × v) + 2 q × (q × v)
  const qv: V3 = [x, y, z];
  const c1 = cross(qv, v);
  const c2 = cross(qv, c1);
  return [
    v[0] + 2 * (w * c1[0] + c2[0]),
    v[1] + 2 * (w * c1[1] + c2[1]),
    v[2] + 2 * (w * c1[2] + c2[2]),
  ];
}
