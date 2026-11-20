/**
 * The line a weld node runs along, as points in the workpiece frame -- for
 * drawing it, not for planning it.
 *
 * A seam weld is its curve (rational B-spline segments) cut to its span; a
 * hand-placed weld is its waypoint path. A classic posed polyline is in the
 * world frame, which this does not draw in, so it has no path here.
 *
 * Arc length is read off a dense polyline of each segment rather than solved
 * exactly: the span's ends land within a chord's sag of where the planner
 * puts them, which is invisible at the scale anything is drawn.
 */

import type { Curve, CurveSegment, ProgramV2Node } from "../contracts/robot-v4-program-v2.ts";
import { samplePath, type V3 } from "../waypoints/model.ts";
import { isManualPoseWeld, shapeOf, waypointWeldOf } from "../waypoints/node.ts";

/** Points per curve segment. A 90 degree conic at 64 is within 0.1 mm of
 *  its circle at a 150 mm radius. */
const SEGMENT_SAMPLES = 64;

export function weldPathPoints(node: ProgramV2Node): V3[] | null {
  if (node.op !== "weld") return null;
  if (isManualPoseWeld(node)) {
    const weld = waypointWeldOf(node);
    return weld.waypoints.length >= 2 ? samplePath(weld.waypoints, 16, shapeOf(weld.tool)) : null;
  }
  if (node.geometry.kind === "seam") {
    const span = node.geometry.span ?? {};
    return curveSpan(node.geometry.curve, span.start_s ?? 0, span.end_s ?? 1);
  }
  return null;
}

/** The curve between normalized arc lengths `start` and `end`, wrapping
 *  through s = 0 when `end < start` (a closed curve's span may). */
export function curveSpan(curve: Curve, start: number, end: number): V3[] {
  const points = sampleCurve(curve);
  if (points.length < 2) return points;
  const cumulative = [0];
  for (let i = 1; i < points.length; i += 1) {
    cumulative.push(cumulative[i - 1]! + distance(points[i]!, points[i - 1]!));
  }
  const total = cumulative.at(-1)!;
  if (!(total > 0)) return points;
  const cut = (lo: number, hi: number): V3[] => {
    const out: V3[] = [pointAt(lo)];
    for (let i = 0; i < points.length; i += 1) {
      const s = cumulative[i]! / total;
      if (s > lo && s < hi) out.push(points[i]!);
    }
    out.push(pointAt(hi));
    return out;
  };
  const pointAt = (s: number): V3 => {
    const target = s * total;
    let i = 1;
    while (i < cumulative.length - 1 && cumulative[i]! < target) i += 1;
    const a = cumulative[i - 1]!;
    const b = cumulative[i]!;
    const t = b > a ? (target - a) / (b - a) : 0;
    const p = points[i - 1]!;
    const q = points[i]!;
    return [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t];
  };
  if (end >= start) return cut(start, end);
  return [...cut(start, 1), ...cut(0, end).slice(1)];
}

/** Every segment, evaluated at evenly spaced parameters, joined end to end. */
export function sampleCurve(curve: Curve): V3[] {
  const out: V3[] = [];
  curve.segments.forEach((segment, index) => {
    const knots = segment.knots_u;
    const u0 = knots[segment.degree]!;
    const u1 = knots[knots.length - segment.degree - 1]!;
    for (let k = index === 0 ? 0 : 1; k <= SEGMENT_SAMPLES; k += 1) {
      out.push(evaluate(segment, u0 + ((u1 - u0) * k) / SEGMENT_SAMPLES));
    }
  });
  return out;
}

/** de Boor on homogeneous control points, and the divide. */
export function evaluate(segment: CurveSegment, u: number): V3 {
  const p = segment.degree;
  const knots = segment.knots_u;
  const points = segment.control_points_xyz_m;
  const weights = segment.weights ?? points.map(() => 1);
  const n = points.length;
  let span = p;
  while (span < n - 1 && u >= knots[span + 1]!) span += 1;
  const d = Array.from({ length: p + 1 }, (_, j) => {
    const w = weights[span - p + j]!;
    const c = points[span - p + j]!;
    return [c[0] * w, c[1] * w, c[2] * w, w];
  });
  for (let r = 1; r <= p; r += 1) {
    for (let j = p; j >= r; j -= 1) {
      const lo = knots[j + span - p]!;
      const hi = knots[j + 1 + span - r]!;
      const alpha = hi > lo ? (u - lo) / (hi - lo) : 0;
      d[j] = d[j]!.map((v, k) => (1 - alpha) * d[j - 1]![k]! + alpha * v);
    }
  }
  const [x, y, z, w] = d[p]!;
  return [x! / w!, y! / w!, z! / w!];
}

function distance(a: V3, b: V3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
