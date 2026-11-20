// @ts-nocheck — verbatim port from weld_planner/v1/ui/src; that tree
// typechecks it under its own strict tsconfig (and scalar-function is
// pinned against python/weld_programmer/scalar.py by tests). Checking
// it again under this tree's noUncheckedIndexedAccess would force a
// fork full of `!`s that could silently drift from the original.
// Ported verbatim from weld_planner/v1/ui/src/scalar-function.ts —
// `evaluate` mirrors python/weld_programmer/scalar.py, pinned there.
/**
 * Scalar functions of normalized arc length, and the model behind their editor.
 *
 * A weld parameter is rarely one number: travel speed drops into a corner, the
 * work angle rolls as a fillet climbs a curved flange. The contract has always
 * allowed that -- constant, piecewise_linear, bspline, samples -- and the UI
 * only ever wrote constants.
 *
 * The editor works in ONE representation, a list of `[s, value]` control points
 * with the first at s=0 and the last at s=1, and converts at the boundary:
 *
 *     constant          -> two points at the same height
 *     piecewise_linear  -> its knots, as they are
 *     bspline, samples  -> not editable here; shown and left alone
 *
 * On the way out a flat profile becomes a `constant` again rather than two
 * identical knots. "This does not vary" is worth saying in the document, and a
 * planner reading `constant` can skip an evaluation it would otherwise have to
 * do per sample.
 *
 * Linear interpolation between points, deliberately. A smooth spline through
 * the same points would overshoot between them, and an overshoot in a work
 * angle is the torch leaning somewhere nobody asked for. If a smooth profile is
 * ever wanted the schema already has `bspline` for it, and it should be an
 * explicit choice rather than the default.
 *
 * `evaluate` mirrors `python/weld_programmer/scalar.py`, pinned by `tests/programmer/test_scalar.py`.
 */

export type ScalarFunction =
  | { kind: "constant"; value: number }
  | { kind: "piecewise_linear"; knots: number[][] }
  | { kind: "bspline"; degree: number; knots_u: number[]; control_values: number[] }
  | { kind: "samples"; s: number[]; values: number[]; interpolation?: string };

/**
 * The same function over a SUB-RANGE, renormalised back onto [0, 1].
 *
 * `restrict(f, lo, hi)(x) === f(lo + x * (hi - lo))`.
 *
 * Needed the moment a weld is split. A profile is a function of fraction along
 * the arc it belongs to, so handing the parent's profile to a half-length
 * child does not keep the angles -- it COMPRESSES the whole ramp into half the
 * metal. A 20 to 10 degree sweep over 110 mm silently becomes two 20 to 10
 * sweeps over 55 mm each, which is a different weld that still looks like a
 * profile.
 *
 * `bspline` and `samples` are returned unchanged rather than resampled. They
 * are not editable here either, and quietly converting one into a
 * piecewise_linear approximation would change the document's own statement
 * about what it is. Both are reported by `restrictionProblems` instead.
 */
export function restrict(fn: ScalarFunction, lo: number, hi: number): ScalarFunction {
  const span = hi - lo;
  if (fn.kind === "constant") return { ...fn };
  if (Math.abs(span) < 1e-12) return { kind: "constant", value: evaluate(fn, lo) };
  // A bspline or a samples table comes back WHOLE, and `restrictionProblems`
  // says the profile was left stretched. A bspline needs knot insertion to
  // narrow correctly; `samples` looks easy and is not, because with
  // `interpolation: "step"` the function is discontinuous and `evaluate` holds
  // the PREVIOUS value, so sampling the endpoints lands on the wrong side of a
  // jump. A narrowing that is subtly wrong at the boundary is worse than one
  // that did not happen and said so.
  if (fn.kind !== "piecewise_linear") return { ...fn };

  const at = (x: number) => (x - lo) / span;
  const inside = fn.knots
    .filter(([s]) => s > Math.min(lo, hi) + 1e-9 && s < Math.max(lo, hi) - 1e-9)
    .map(([s, value]) => [at(s), value] as number[]);
  const knots = [
    [0, evaluate(fn, lo)],
    ...(span > 0 ? inside : inside.slice().reverse()),
    [1, evaluate(fn, hi)],
  ];
  // A restriction that turns out flat is written back as a constant, the same
  // rule `fromPoints` follows: "this does not vary" is worth saying.
  const values = knots.map(([, v]) => v);
  if (Math.max(...values) - Math.min(...values) < 1e-12) {
    return { kind: "constant", value: values[0] };
  }
  return { kind: "piecewise_linear", knots: knots.map(([s, v]) => [Number(s), Number(v)]) };
}

/** Why `restrict` could not narrow this, if it could not. Empty for everything
 *  the UI authors; a `bspline` is the one kind left stretched, and that is
 *  named rather than left to be discovered. */
export const restrictionProblems = (fn: ScalarFunction): string[] => {
  if (fn.kind === "bspline") {
    return [
      "a bspline profile cannot be narrowed onto a shorter arc without knot" +
        " insertion, so it is carried across whole and now spans the piece" +
        " rather than the original weld",
    ];
  }
  if (fn.kind === "samples") {
    return [
      "a sampled profile is carried across whole and now spans the piece rather" +
        " than the original weld: with step interpolation it is discontinuous," +
        " and narrowing it by resampling lands on the wrong side of a jump",
    ];
  }
  return [];
};

/** A control point in the editor: normalized arc length, and the value there. */
export interface Point {
  s: number;
  value: number;
}

const clamp01 = (v: number) => (v < 0 ? 0 : v > 1 ? 1 : v);

/** The value at `s`, clamped to [0, 1]. Mirrors `scalar.evaluate`. */
export function evaluate(fn: ScalarFunction, s: number): number {
  const at = clamp01(s);
  if (fn.kind === "constant") return fn.value;

  if (fn.kind === "piecewise_linear") {
    const knots = fn.knots;
    if (at <= knots[0][0]) return knots[0][1];
    for (let i = 1; i < knots.length; i += 1) {
      const [s0, v0] = knots[i - 1];
      const [s1, v1] = knots[i];
      if (at <= s1) {
        const span = s1 - s0;
        // A zero-width span is a step; taking the right-hand value keeps the
        // function right-continuous, as the span arithmetic elsewhere is.
        return span <= 0 ? v1 : v0 + ((v1 - v0) * (at - s0)) / span;
      }
    }
    return knots[knots.length - 1][1];
  }

  if (fn.kind === "samples") {
    const { s: positions, values, interpolation } = fn;
    if (at <= positions[0]) return values[0];
    for (let i = 1; i < positions.length; i += 1) {
      if (at <= positions[i]) {
        if (interpolation === "step") return values[i - 1];
        const span = positions[i] - positions[i - 1];
        return span <= 0
          ? values[i]
          : values[i - 1] + ((values[i] - values[i - 1]) * (at - positions[i - 1])) / span;
      }
    }
    return values[values.length - 1];
  }

  return deBoor(fn, at);
}

function deBoor(fn: Extract<ScalarFunction, { kind: "bspline" }>, s: number): number {
  const { degree, knots_u: knots, control_values: values } = fn;
  const lo = knots[degree];
  const hi = knots[knots.length - degree - 1];
  const u = lo + (hi - lo) * s;

  let span = degree;
  for (let i = degree; i < knots.length - degree - 1; i += 1) {
    if (u >= knots[i + 1]) span = i + 1;
  }
  span = Math.min(span, values.length - 1);

  const d: number[] = [];
  for (let i = 0; i <= degree; i += 1) d.push(values[span - degree + i]);
  for (let r = 1; r <= degree; r += 1) {
    for (let i = degree; i >= r; i -= 1) {
      const left = knots[i + span - degree];
      const right = knots[i + 1 + span - r];
      const alpha = right === left ? 0 : (u - left) / (right - left);
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }
  return d[degree];
}

/** Can the point editor handle this function, or only display it? */
export const isEditable = (fn: ScalarFunction | undefined): boolean =>
  fn?.kind === "constant" || fn?.kind === "piecewise_linear";

/** Editor points for a function. Always at least two, spanning s=0 to s=1. */
export function toPoints(fn: ScalarFunction | undefined, fallback = 0): Point[] {
  if (!fn || fn.kind === "constant") {
    const value = fn?.kind === "constant" ? fn.value : fallback;
    return [
      { s: 0, value },
      { s: 1, value },
    ];
  }
  if (fn.kind === "piecewise_linear") {
    return fn.knots.map(([s, value]) => ({ s, value }));
  }
  // Not editable: represent it by its sampled shape so the caller can draw
  // something honest, and refuse to write it back.
  return [0, 0.25, 0.5, 0.75, 1].map((s) => ({ s, value: evaluate(fn, s) }));
}

/** Points back to a function. Flat becomes a constant. */
export function fromPoints(points: Point[], tolerance = 1e-9): ScalarFunction {
  const sorted = normalize(points);
  const values = sorted.map((p) => p.value);
  const flat = Math.max(...values) - Math.min(...values) <= tolerance;
  if (flat) return { kind: "constant", value: values[0] };
  return { kind: "piecewise_linear", knots: sorted.map((p) => [p.s, p.value]) };
}

/**
 * Sorted, endpoints pinned to 0 and 1, no two points at the same s.
 *
 * The schema requires strictly increasing knots from 0 to 1, and JSON Schema
 * cannot express that -- `python/scalar.problems` catches it server-side, and
 * this stops the editor from producing it in the first place.
 */
export function normalize(points: Point[], minGap = 1e-4): Point[] {
  const sorted = [...points].sort((a, b) => a.s - b.s);
  if (sorted.length < 2) {
    const value = sorted[0]?.value ?? 0;
    return [
      { s: 0, value },
      { s: 1, value },
    ];
  }
  sorted[0] = { ...sorted[0], s: 0 };
  sorted[sorted.length - 1] = { ...sorted[sorted.length - 1], s: 1 };
  const out: Point[] = [sorted[0]];
  for (let i = 1; i < sorted.length; i += 1) {
    const previous = out[out.length - 1];
    const s = Math.min(1, Math.max(previous.s + minGap, clamp01(sorted[i].s)));
    if (i === sorted.length - 1) out.push({ ...sorted[i], s: 1 });
    else if (s < 1) out.push({ ...sorted[i], s });
  }
  return out;
}

/** Add a control point, keeping the list ordered and legal. */
export const addPoint = (points: Point[], point: Point): Point[] =>
  normalize([...points, { s: clamp01(point.s), value: point.value }]);

/** Remove one, unless it is an endpoint -- the span has to stay [0, 1]. */
export function removePoint(points: Point[], index: number): Point[] {
  if (index <= 0 || index >= points.length - 1) return points;
  return normalize(points.filter((_, i) => i !== index));
}

/** Move one. Endpoints keep their s and move only in value. */
export function movePoint(points: Point[], index: number, next: Point): Point[] {
  const edge = index === 0 || index === points.length - 1;
  const updated = points.map((point, i) =>
    i === index ? { s: edge ? point.s : clamp01(next.s), value: next.value } : point,
  );
  return normalize(updated);
}
