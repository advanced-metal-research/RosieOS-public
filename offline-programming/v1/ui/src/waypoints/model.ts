/**
 * The waypoint weld: one weld per program, authored as torch poses the
 * operator places by hand rather than found on a CAD edge. Each waypoint is
 * a work point on the part, the torch orientation there and the standoff
 * the torch is held at, all in the PART's own frame, so moving the part
 * carries its waypoints with it; each interior waypoint says how the path
 * passes through it -- C0, C1 or C2.
 *
 * This module is the data and the path through it. The program node it is
 * stored as is `node.ts`; the seam a plan receives is
 * `program-v2/manual-pose-seam.ts`.
 */

import { DEFAULT_TORCH_STANDOFF_MM } from "../domain/project.ts";

export type V3 = [number, number, number];
/** A unit quaternion, x y z w -- three's order. */
export type Quat = [number, number, number, number];

/**
 * How the path passes through a waypoint.
 *
 * - `c0`: a corner. The segments either side are straight toward their
 *   neighbours; only position is shared.
 * - `c1`: a smooth turn. The tangent direction is shared (the bisector of the
 *   two chords); curvature may jump.
 * - `c2`: a spline knot. Tangent and curvature are both shared with any
 *   neighbouring C2 knot, solved together as one cubic spline.
 *
 * The two end waypoints have no continuity of their own: the path simply
 * starts and stops there.
 */
export type Continuity = "c0" | "c1" | "c2";
export const CONTINUITIES: readonly Continuity[] = ["c0", "c1", "c2"];

export interface Waypoint {
  id: string;
  /** The work point the torch aims at, metres, part frame. The path runs
   *  through these. */
  positionM: V3;
  /** Torch orientation, part frame. +Z points from the gun into the work;
   *  the torch sits back along -Z from the work point. */
  quaternion: Quat;
  continuity: Continuity;
  /** Contact tip to work point, along the torch axis. Per waypoint, so a
   *  weld can close in or back off along its length. */
  standoffMm: number;
}

/** A waypoint's standoff when nothing else says: the cell's one default. */
export const DEFAULT_WAYPOINT_STANDOFF_MM = DEFAULT_TORCH_STANDOFF_MM;

/** The standoff range a waypoint accepts: the rails torch sheet's
 *  (`rails/parameters.ts`), the widest either fitted gun is held at. */
export const STANDOFF_MIN_MM = 0.5;
export const STANDOFF_MAX_MM = 100;

export function clampStandoffMm(value: number): number {
  return Math.min(STANDOFF_MAX_MM, Math.max(STANDOFF_MIN_MM, value));
}

/**
 * How a weld's points make its path. `spline` is the waypoint rule of
 * `pathSegments`, and a line is that rule with two points. `arc` is the
 * circle through exactly three points, from the first through the second to
 * the third.
 */
export type WeldShape = "spline" | "arc";

/** The circle through three points: `center`, `radius`, the in-plane axes
 *  `u` (toward the first point) and `v` (toward the direction of travel),
 *  the total `sweep` and the angle at which it passes the second point. */
export interface Arc {
  center: V3;
  radius: number;
  u: V3;
  v: V3;
  sweep: number;
  throughAngle: number;
}

/** Shortest distance from the middle point to the chord of the outer two
 *  below which three points are read as a line and not an arc: 1 um, the
 *  document's coincidence threshold (`MANUAL_POSE_MIN_CHORD_M`). */
export const ARC_MIN_SAGITTA_M = 1e-6;

/**
 * The arc through three points, or null when they are (nearly) collinear or
 * the outer two coincide -- no circle, or no way to say which way round.
 */
export function arcThrough(points: readonly Pick<Waypoint, "positionM">[]): Arc | null {
  if (points.length !== 3) return null;
  const [a, b, c] = points.map((point) => point.positionM) as [V3, V3, V3];
  const ab = sub(b, a);
  const ac = sub(c, a);
  const normal = cross(ab, ac);
  const area = norm(normal);
  const chord = norm(ac);
  if (chord <= ARC_MIN_SAGITTA_M || area / chord <= ARC_MIN_SAGITTA_M) return null;
  // Circumcentre: a + (|ab|^2 (ac x n) + |ac|^2 (n x ab)) / (2 |n|^2).
  const n2 = area * area;
  const offset = scale(
    add(scale(cross(ac, normal), dot(ab, ab)), scale(cross(normal, ab), dot(ac, ac))),
    1 / (2 * n2),
  );
  const center = add(a, offset);
  const radius = norm(offset);
  const u = scale(sub(a, center), 1 / radius);
  const axis = scale(normal, 1 / area);
  const v = cross(axis, u);
  const angleOf = (p: V3): number => {
    const d = sub(p, center);
    const angle = Math.atan2(dot(d, v), dot(d, u));
    return angle < 0 ? angle + 2 * Math.PI : angle;
  };
  return { center, radius, u, v, sweep: angleOf(c), throughAngle: angleOf(b) };
}

/** The point on an arc at `angle` from its first point. */
export function arcPoint(arc: Arc, angle: number): V3 {
  return add(arc.center, add(scale(arc.u, arc.radius * Math.cos(angle)), scale(arc.v, arc.radius * Math.sin(angle))));
}

/** The unit direction of travel on an arc at `angle`. */
export function arcTangent(arc: Arc, angle: number): V3 {
  return add(scale(arc.u, -Math.sin(angle)), scale(arc.v, Math.cos(angle)));
}

export function dot(a: V3, b: V3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function cross(a: V3, b: V3): V3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

/** Pointing straight down in the part frame: tool Z = part -Z. */
export const TORCH_DOWN: Quat = [1, 0, 0, 0];

export function newId(prefix: string): string {
  const random = Math.random().toString(36).slice(2, 8);
  return `${prefix}-${Date.now().toString(36)}-${random}`;
}

/**
 * A waypoint added after `afterIndex`: an exact copy of that waypoint's
 * pose, standoff and continuity under a new id -- the operator moves it off
 * with the handle from where the last one was left. With nothing to copy it
 * starts at `origin`, pointing down, at `standoffMm`.
 */
export function waypointAfter(
  points: readonly Waypoint[],
  afterIndex: number,
  origin: V3,
  standoffMm: number,
): Waypoint {
  const base = points[afterIndex];
  if (!base) {
    return {
      id: newId("wp"),
      positionM: [...origin],
      quaternion: [...TORCH_DOWN],
      continuity: "c1",
      standoffMm: clampStandoffMm(standoffMm),
    };
  }
  return {
    id: newId("wp"),
    positionM: [...base.positionM],
    quaternion: [...base.quaternion],
    continuity: base.continuity,
    standoffMm: base.standoffMm,
  };
}

/** The continuity every interior waypoint shares, or null when they differ
 *  (or there is no interior waypoint to have one). */
export function sharedContinuity(points: readonly Waypoint[]): Continuity | null {
  const interior = points.slice(1, -1);
  const first = interior[0]?.continuity;
  if (!first) return null;
  return interior.every((point) => point.continuity === first) ? first : null;
}

/**
 * One piece of the path, between waypoints `index` and `index + 1`: a cubic
 * Hermite in u in [0, 1], `p(u) = H(p0, h*m0, p1, h*m1; u)`, with `m0` and
 * `m1` the unit tangents the rule below gives at its ends and `h` its chord.
 */
export interface PathSegment {
  index: number;
  p0: V3;
  m0: V3;
  p1: V3;
  m1: V3;
  h: number;
}

/**
 * The path through the waypoints, one segment per consecutive pair.
 *
 * Chord-length parametrised cubic Hermite segments. The tangent at each
 * waypoint (a unit vector: the parameter is roughly arc length) is:
 *
 * - C0: one-sided -- the chord on each side, which makes a segment between
 *   two C0 waypoints exactly the straight line between them;
 * - C1: the bisector of the two unit chords;
 * - C2 and the two ends: unknowns of the standard cubic-spline system, with
 *   curvature continuity at C2 waypoints and a natural (zero curvature) end.
 *   A C0 or C1 neighbour enters that system as a known tangent.
 *
 * This is the rule program.v2 pins for a `manual_pose_wp` weld; the seam a
 * plan receives is built from these same segments.
 *
 * Coincident waypoints (a zero-length chord) make a segment that is a point:
 * there is no direction to be continuous with. The document refuses them.
 */
export function pathSegments(waypoints: readonly Pick<Waypoint, "positionM" | "continuity">[]): PathSegment[] {
  const n = waypoints.length;
  if (n < 2) return [];
  const p = waypoints.map((point) => point.positionM);
  const chords: V3[] = [];
  const h: number[] = [];
  for (let i = 0; i + 1 < n; i += 1) {
    chords.push(sub(p[i + 1]!, p[i]!));
    h.push(Math.max(norm(chords[i]!), 1e-9));
  }
  const unitChord = (i: number): V3 => scale(chords[i]!, 1 / h[i]!);
  // Known tangents: [incoming, outgoing] at each waypoint, or null for an
  // unknown of the spline system.
  const known: ([V3, V3] | null)[] = waypoints.map((point, i) => {
    if (i === 0 || i === n - 1) return null;
    if (point.continuity === "c0") return [unitChord(i - 1), unitChord(i)];
    if (point.continuity === "c1") {
      const bisector = add(unitChord(i - 1), unitChord(i));
      const length = norm(bisector);
      // A full reversal has no bisector; the path turns on the spot.
      const direction = length > 1e-9 ? scale(bisector, 1 / length) : unitChord(i);
      return [direction, direction];
    }
    return null;
  });
  const tangents = solveTangents(n, h, chords, known);
  const out: PathSegment[] = [];
  for (let i = 0; i + 1 < n; i += 1) {
    out.push({
      index: i,
      p0: p[i]!,
      m0: known[i]?.[1] ?? tangents[i]!,
      p1: p[i + 1]!,
      m1: known[i + 1]?.[0] ?? tangents[i + 1]!,
      h: h[i]!,
    });
  }
  return out;
}

/** The point at `u` on a segment. */
export function segmentPoint(segment: PathSegment, u: number): V3 {
  return hermite(segment.p0, segment.m0, segment.p1, segment.m1, segment.h, u);
}

/** dp/du at `u` on a segment: the tangent, not normalized. */
export function segmentDerivative(segment: PathSegment, u: number): V3 {
  const u2 = u * u;
  const d00 = 6 * u2 - 6 * u;
  const d10 = 3 * u2 - 4 * u + 1;
  const d01 = -6 * u2 + 6 * u;
  const d11 = 3 * u2 - 2 * u;
  const { p0, m0, p1, m1, h } = segment;
  return [0, 1, 2].map((k) =>
    d00 * p0[k]! + d10 * h * m0[k]! + d01 * p1[k]! + d11 * h * m1[k]!
  ) as V3;
}

/** Gauss-Legendre, five nodes on [0, 1]: exact for a degree-9 polynomial,
 *  and |p'(u)| of a cubic is smooth enough that eight panels of it put a
 *  100 mm S-bend within a nanometre of a 4096-panel trapezoid sum. */
const GAUSS_NODES = [0.0469100770306680, 0.2307653449471585, 0.5, 0.7692346550528415, 0.9530899229693320];
const GAUSS_WEIGHTS = [0.1184634425280945, 0.2393143352496832, 0.2844444444444444, 0.2393143352496832, 0.1184634425280945];
const ARC_PANELS = 8;

/** Arc length of a segment from u = 0 to `u`, metres. */
export function segmentArcLengthM(segment: PathSegment, u = 1): number {
  let total = 0;
  const width = u / ARC_PANELS;
  for (let panel = 0; panel < ARC_PANELS; panel += 1) {
    for (let k = 0; k < GAUSS_NODES.length; k += 1) {
      total += GAUSS_WEIGHTS[k]! * width * norm(segmentDerivative(segment, (panel + GAUSS_NODES[k]!) * width));
    }
  }
  return total;
}

/** Where each waypoint sits along the path, as normalized arc length: 0 at
 *  the first, 1 at the last. The s every scalar function on the weld node is
 *  a function of. Evenly spread when the path has no length. */
export function waypointArcFractions(
  waypoints: readonly Pick<Waypoint, "positionM" | "continuity">[],
  shape: WeldShape = "spline",
): number[] {
  if (waypoints.length === 0) return [];
  if (waypoints.length === 1) return [0];
  if (shape === "arc") {
    const arc = arcThrough(waypoints);
    if (arc) return [0, arc.throughAngle / arc.sweep, 1];
  }
  const lengths = pathSegments(waypoints).map((segment) => segmentArcLengthM(segment));
  const total = lengths.reduce((sum, length) => sum + length, 0);
  const out = [0];
  let running = 0;
  for (const length of lengths) {
    running += length;
    out.push(total > 0 ? running / total : out.length / lengths.length);
  }
  out[out.length - 1] = 1;
  return out;
}

/**
 * Points along the path through the waypoints, `samplesPerSegment` per
 * segment plus the last waypoint, positions only. See `pathSegments` for
 * the rule.
 */
export function samplePath(
  waypoints: readonly Pick<Waypoint, "positionM" | "continuity">[],
  samplesPerSegment = 16,
  shape: WeldShape = "spline",
): V3[] {
  if (waypoints.length === 0) return [];
  if (waypoints.length === 1) return [[...waypoints[0]!.positionM]];
  if (shape === "arc") {
    const arc = arcThrough(waypoints);
    if (arc) {
      const count = Math.max(2, Math.ceil((arc.sweep / (Math.PI / 2)) * samplesPerSegment));
      return Array.from({ length: count + 1 }, (_, k) => arcPoint(arc, (k / count) * arc.sweep));
    }
  }
  const out: V3[] = [];
  const steps = Math.max(1, Math.floor(samplesPerSegment));
  for (const segment of pathSegments(waypoints)) {
    for (let k = 0; k < steps; k += 1) out.push(segmentPoint(segment, k / steps));
  }
  out.push([...waypoints[waypoints.length - 1]!.positionM]);
  return out;
}

/**
 * Tangents at the unknown waypoints, by the Thomas algorithm on the
 * tridiagonal spline system (one per coordinate; the matrix is shared).
 *
 *   interior C2 i:  h[i] m[i-1] + 2(h[i-1] + h[i]) m[i] + h[i-1] m[i+1]
 *                     = 3 (h[i] d[i-1] + h[i-1] d[i])
 *   start:          2 m[0] + m[1]   = 3 d[0]
 *   end:            m[n-2] + 2 m[n-1] = 3 d[n-2]
 *
 * with d[i] the unit chord. A known neighbour's term moves to the right-hand
 * side (using the tangent on the side that faces this waypoint), and a known
 * waypoint's own row is the identity, so a C0 or C1 waypoint splits the
 * system into independent runs.
 */
function solveTangents(
  n: number,
  h: readonly number[],
  chords: readonly V3[],
  known: readonly ([V3, V3] | null)[],
): V3[] {
  const d = chords.map((chord, i) => scale(chord, 1 / h[i]!));
  const lower = new Array<number>(n).fill(0);
  const diag = new Array<number>(n).fill(1);
  const upper = new Array<number>(n).fill(0);
  const rhs: V3[] = Array.from({ length: n }, () => [0, 0, 0] as V3);
  for (let i = 0; i < n; i += 1) {
    const fixed = known[i];
    if (fixed) {
      rhs[i] = [...fixed[1]];
      continue;
    }
    let a = 0;
    let b: number;
    let c = 0;
    let r: V3;
    if (i === 0) {
      b = 2;
      c = 1;
      r = scale(d[0]!, 3);
    } else if (i === n - 1) {
      a = 1;
      b = 2;
      r = scale(d[n - 2]!, 3);
    } else {
      a = h[i]!;
      b = 2 * (h[i - 1]! + h[i]!);
      c = h[i - 1]!;
      r = add(scale(d[i - 1]!, 3 * h[i]!), scale(d[i]!, 3 * h[i - 1]!));
    }
    const left = i > 0 ? known[i - 1] : null;
    if (left) {
      r = sub(r, scale(left[1], a));
      a = 0;
    }
    const right = i < n - 1 ? known[i + 1] : null;
    if (right) {
      r = sub(r, scale(right[0], c));
      c = 0;
    }
    lower[i] = a;
    diag[i] = b;
    upper[i] = c;
    rhs[i] = r;
  }
  for (let i = 1; i < n; i += 1) {
    const w = lower[i]! / diag[i - 1]!;
    diag[i] = diag[i]! - w * upper[i - 1]!;
    rhs[i] = sub(rhs[i]!, scale(rhs[i - 1]!, w));
  }
  const m: V3[] = new Array(n);
  m[n - 1] = scale(rhs[n - 1]!, 1 / diag[n - 1]!);
  for (let i = n - 2; i >= 0; i -= 1) {
    m[i] = scale(sub(rhs[i]!, scale(m[i + 1]!, upper[i]!)), 1 / diag[i]!);
  }
  return m;
}

function hermite(p0: V3, m0: V3, p1: V3, m1: V3, h: number, s: number): V3 {
  const s2 = s * s;
  const s3 = s2 * s;
  const h00 = 2 * s3 - 3 * s2 + 1;
  const h10 = s3 - 2 * s2 + s;
  const h01 = -2 * s3 + 3 * s2;
  const h11 = s3 - s2;
  return [0, 1, 2].map((k) =>
    h00 * p0[k]! + h10 * h * m0[k]! + h01 * p1[k]! + h11 * h * m1[k]!
  ) as V3;
}

export function add(a: V3, b: V3): V3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function sub(a: V3, b: V3): V3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function scale(a: V3, k: number): V3 {
  return [a[0] * k, a[1] * k, a[2] * k];
}

export function norm(a: V3): number {
  return Math.hypot(a[0], a[1], a[2]);
}
