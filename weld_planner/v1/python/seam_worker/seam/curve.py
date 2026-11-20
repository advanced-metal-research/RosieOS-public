#!/usr/bin/env python
"""Rational B-spline evaluation, arc length, and curvature.

The piece everything geometric depends on. `SEAM-MODEL.md` R4 requires curves
stay analytic because the torch frame needs C1 and curvature needs C2, and
finite differences over a polyline are noisiest exactly at tight corners --
which is where they matter. This module is what makes that pay off.

Pure Python plus numpy: no OCCT, so it is fast, importable anywhere, and
testable without a CAD environment. That also makes OCCT an *independent
oracle* for the tests rather than the implementation, which is the point --
`tests/programmer/test_curve.py` checks every evaluated point against OCCT's own
evaluation of the same edge.

Two things it exists to provide:

**`u` is a dial, `s` is a distance.** A B-spline's parameter carries no metric
meaning; on the dog_box loop, treating them as interchangeable puts a waypoint
62.5 mm out of place. Everything in the weld program contract is defined on
normalized arc length, so converting is mandatory, not optional.

**Curvature.** Needed for adaptive sampling and speed scheduling, and only
available from a C2 curve.

## Scalar evaluation, batched evaluation

`de_boor` evaluates one parameter; `de_boor_many` carries the same recursion
across an array of them, and everything arc-length goes through the batched
form. That is not a micro-optimisation: arc length is quadrature, quadrature is
thousands of evaluations known up front, and building one segment's `s -> u`
table called the scalar form about 12,000 times at interpreter overhead each.
Measured, that was **59% of a torch clearance check** -- a module with no rays
in it dominating a module that is nothing but rays. Constructing pipe_bracket's
longest weld line and its rail went from 568 ms to 18 ms.

Both forms are kept, and `tests/programmer/test_curve.py` checks them against each other on
every edge of two real workpieces, so the fast one cannot drift from the one the
OCCT parity tests pin.
"""

from __future__ import annotations

import argparse
import bisect
import itertools
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from seam_worker import paths

# Gauss-Legendre nodes for arc-length quadrature. 16 is exact for polynomials
# up to degree 31, which comfortably covers |C'(u)| for the degrees in use.
_GAUSS_NODES, _GAUSS_WEIGHTS = np.polynomial.legendre.leggauss(16)

# Subintervals per knot span for arc-length quadrature. Gauss-Legendre is exact
# for smooth integrands; |C'(u)| is smooth only WITHIN a span.
_PANELS_PER_SPAN = 4

# Samples in the s -> u lookup table, per segment.
DEFAULT_LUT_SAMPLES = 64

NEWTON_TOL = 1e-12
NEWTON_MAX_STEPS = 24

# A position this close to a segment join, in NORMALIZED arc length, is ON that
# join. See `Curve.joins_s` for why it is not zero: the joins a caller asks for
# are exact, but only against the DECLARED lengths, and a sum in a different
# order is a few ulps away from the same number. 1e-12 is 0.19 nm on a 190 mm
# weld line -- four orders below the ~0.06 mm the reachability bisection in
# `access.py` resolves a boundary to, so nothing that is genuinely near a corner
# can reach it, and eight orders above the noise it exists to absorb.
JOIN_TOL_S = 1e-12


# --------------------------------------------------------------------------
# de Boor
# --------------------------------------------------------------------------


def find_span(u: float, degree: int, knots: np.ndarray) -> int:
    """Index of the knot span containing `u`, clamped to the valid range."""
    n = len(knots) - degree - 2
    if u >= knots[n + 1]:
        return n
    if u <= knots[degree]:
        return degree
    return max(degree, min(n, bisect.bisect_right(knots, u) - 1))


def find_span_many(us: np.ndarray, degree: int, knots: np.ndarray) -> np.ndarray:
    """`find_span` for a whole array at once. Same clamping, same edge cases."""
    n = len(knots) - degree - 2
    spans = np.clip(np.searchsorted(knots, us, side="right") - 1, degree, n)
    spans = np.where(us >= knots[n + 1], n, spans)
    return np.where(us <= knots[degree], degree, spans)


def de_boor(u: float, degree: int, knots: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Evaluate a (possibly homogeneous) B-spline at `u`.

    Dimension-agnostic: pass 3D control points for a non-rational curve, or 4D
    homogeneous points for a rational one.
    """
    if degree == 0:
        return points[find_span(u, degree, knots)].copy()
    span = find_span(u, degree, knots)
    d = [points[span - degree + i].astype(float).copy() for i in range(degree + 1)]
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            left = knots[span + j - degree]
            right = knots[span + j - r + 1]
            alpha = 0.0 if right == left else (u - left) / (right - left)
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
    return d[degree]


def de_boor_many(
    us: np.ndarray, degree: int, knots: np.ndarray, points: np.ndarray
) -> np.ndarray:
    """`de_boor` evaluated at every `u` in one pass.

    The same triangle, with the recursion carried across an axis of parameters
    instead of one at a time. It exists because arc length is quadrature and
    quadrature is thousands of evaluations that are known up front: building one
    segment's arc-length table called `de_boor` about 12,000 times, at Python
    call overhead each, which was 59% of a torch clearance check. Batched, the
    same table is two array operations.

    Kept beside the scalar form rather than replacing it: `evaluate` is a
    genuinely one-point path, and `tests/programmer/test_curve.py` checks the two against
    each other so the fast one cannot drift.
    """
    us = np.asarray(us, dtype=float)
    if degree == 0:
        return points[find_span_many(us, 0, knots)].astype(float)
    spans = find_span_many(us, degree, knots)
    d = points[spans[:, None] - degree + np.arange(degree + 1)[None, :]].astype(float)
    for r in range(1, degree + 1):
        for j in range(degree, r - 1, -1):
            left = knots[spans + j - degree]
            denom = knots[spans + j - r + 1] - left
            # A zero span is a repeated knot; the scalar form takes alpha = 0
            # there, and the guard keeps the divide from warning about it.
            alpha = np.where(denom == 0.0, 0.0, (us - left) / np.where(denom == 0.0, 1.0, denom))
            a = alpha[:, None]
            d[:, j, :] = (1.0 - a) * d[:, j - 1, :] + a * d[:, j, :]
    return d[:, degree, :]


def derivative_spline(
    degree: int, knots: np.ndarray, points: np.ndarray
) -> tuple[int, np.ndarray, np.ndarray]:
    """Control points and knots of the derivative curve.

    q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1}), on the interior knots.
    """
    if degree < 1 or len(points) < 2:
        return 0, knots[1:-1] if len(knots) > 2 else knots, np.zeros((1, points.shape[1]))
    q = np.zeros((len(points) - 1, points.shape[1]))
    for i in range(len(points) - 1):
        denom = knots[i + degree + 1] - knots[i + 1]
        if denom > 0:
            q[i] = degree * (points[i + 1] - points[i]) / denom
    return degree - 1, knots[1:-1], q


class Segment:
    """One clamped rational B-spline segment of a weld line or rail."""

    def __init__(self, spec: dict[str, Any]):
        self.degree = int(spec["degree"])
        self.knots = np.asarray(spec["knots_u"], dtype=float)
        points = np.asarray(spec["control_points_xyz_m"], dtype=float)
        weights = spec.get("weights")
        self.rational = weights is not None
        self.weights = (
            np.asarray(weights, dtype=float)
            if self.rational
            else np.ones(len(points), dtype=float)
        )
        # Homogeneous control points: rational evaluation is just de Boor in 4D
        # followed by a projective divide.
        self.homogeneous = np.hstack([points * self.weights[:, None], self.weights[:, None]])
        self.u0 = float(self.knots[self.degree])
        self.u1 = float(self.knots[len(self.knots) - self.degree - 1])
        self.declared_length = float(spec.get("length_m", 0.0))

        d1 = derivative_spline(self.degree, self.knots, self.homogeneous)
        d2 = derivative_spline(*d1)
        self._d1 = d1
        self._d2 = d2
        self._lut: tuple[np.ndarray, np.ndarray] | None = None

    # -- evaluation ------------------------------------------------------

    def _homogeneous_at(self, u: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        a = de_boor(u, self.degree, self.knots, self.homogeneous)
        a1 = de_boor(u, self._d1[0], self._d1[1], self._d1[2])
        a2 = de_boor(u, self._d2[0], self._d2[1], self._d2[2])
        return a, a1, a2

    def evaluate(self, u: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(point, first derivative, second derivative) at parameter `u`.

        Rational derivatives come from the quotient rule on C = A/w:
            C'  = (A' - w' C) / w
            C'' = (A'' - 2 w' C' - w'' C) / w
        Differencing the projected curve instead would lose the exactness the
        rational representation exists to provide.
        """
        u = min(max(u, self.u0), self.u1)
        a, a1, a2 = self._homogeneous_at(u)
        w, w1, w2 = a[3], a1[3], a2[3]
        point = a[:3] / w
        first = (a1[:3] - w1 * point) / w
        second = (a2[:3] - 2.0 * w1 * first - w2 * point) / w
        return point, first, second

    def point(self, u: float) -> np.ndarray:
        return self.evaluate(u)[0]

    def speed(self, u: float) -> float:
        return float(np.linalg.norm(self.evaluate(u)[1]))

    def speed_many(self, us: np.ndarray) -> np.ndarray:
        """`|C'(u)|` at every `u` at once -- the quadrature integrand.

        Only the first derivative, where `evaluate` also computes the second.
        Arc length never wants C'', and on a rational segment computing it costs
        a third de Boor pass plus the quotient rule.
        """
        us = np.clip(np.asarray(us, dtype=float), self.u0, self.u1)
        a = de_boor_many(us, self.degree, self.knots, self.homogeneous)
        a1 = de_boor_many(us, self._d1[0], self._d1[1], self._d1[2])
        w = a[:, 3:4]
        point = a[:, :3] / w
        return np.linalg.norm((a1[:, :3] - a1[:, 3:4] * point) / w, axis=1)

    def _quadrature(self, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
        """Gauss-Legendre over many panels at once. `lo`/`hi` are arrays."""
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        nodes = mid[:, None] + half[:, None] * _GAUSS_NODES[None, :]
        speeds = self.speed_many(nodes.ravel()).reshape(nodes.shape)
        return half * (speeds * _GAUSS_WEIGHTS[None, :]).sum(axis=1)

    def curvature(self, u: float) -> float:
        _, d1, d2 = self.evaluate(u)
        cross = np.cross(d1, d2)
        denom = float(np.linalg.norm(d1)) ** 3
        return float(np.linalg.norm(cross) / denom) if denom > 1e-30 else 0.0

    # -- arc length ------------------------------------------------------

    def _panels(self, lo: float, hi: float) -> list[tuple[float, float]]:
        """Integration panels that never straddle an interior knot.

        A curve is only C1 across an interior knot, so |C'(u)| has a kink
        there and Gauss-Legendre -- which assumes a smooth integrand -- loses
        most of its accuracy. Integrating one panel across the whole span of a
        270 degree arc gave a length 4.4e-5 mm wrong and, worse, an s->u map
        that bunched samples near segment boundaries by 25%.
        """
        breaks = sorted({lo, hi} | {float(k) for k in self.knots if lo < k < hi})
        panels: list[tuple[float, float]] = []
        for a, b in itertools.pairwise(breaks):
            for i in range(_PANELS_PER_SPAN):
                t0 = a + (b - a) * i / _PANELS_PER_SPAN
                t1 = a + (b - a) * (i + 1) / _PANELS_PER_SPAN
                panels.append((t0, t1))
        return panels

    def arc_length(self, u0: float | None = None, u1: float | None = None) -> float:
        """Gauss-Legendre quadrature of |C'(u)|, panelled at interior knots.

        Arc length of a polynomial curve has no closed form in general -- it is
        an elliptic integral even in simple cases -- so quadrature is
        unavoidable. Only straight lines give s proportional to u for free.
        """
        lo = self.u0 if u0 is None else max(self.u0, u0)
        hi = self.u1 if u1 is None else min(self.u1, u1)
        if hi <= lo:
            return 0.0
        panels = np.asarray(self._panels(lo, hi), dtype=float)
        return float(self._quadrature(panels[:, 0], panels[:, 1]).sum())

    def _build_lut(self, samples: int = DEFAULT_LUT_SAMPLES) -> tuple[np.ndarray, np.ndarray]:
        if self._lut is None:
            # Interior knots are inserted as LUT breakpoints for the same
            # reason quadrature panels at them: the curve is only C1 there.
            # `lengths` must be sized from `us`, not from `samples` -- sizing it
            # from `samples` silently truncated the table by however many knots
            # were inserted, which lost 1.495 mm on a 270 degree arc and bunched
            # s -> u samples near every segment boundary.
            interior = [float(k) for k in self.knots if self.u0 < k < self.u1]
            us = np.unique(
                np.concatenate([np.linspace(self.u0, self.u1, samples), np.array(interior)])
            )
            # Every panel of every step in ONE quadrature call. The panels are
            # exactly the ones the step-by-step `arc_length` would have used --
            # no interior knot can fall inside a step, because they are all
            # breakpoints above -- so this changes the cost and not the number.
            frac = np.arange(_PANELS_PER_SPAN + 1, dtype=float) / _PANELS_PER_SPAN
            width = (us[1:] - us[:-1])[:, None]
            lo = us[:-1, None] + width * frac[None, :-1]
            hi = us[:-1, None] + width * frac[None, 1:]
            per_step = self._quadrature(lo.ravel(), hi.ravel()).reshape(lo.shape).sum(axis=1)
            self._lut = (us, np.concatenate([[0.0], np.cumsum(per_step)]))
        return self._lut

    @property
    def length(self) -> float:
        return float(self._build_lut()[1][-1])

    def u_at_length(self, target: float) -> float:
        """Invert arc length to parameter: LUT bracket, then Newton polish.

        Newton converges fast because the derivative it needs, |C'(u)|, is the
        quadrature integrand already in hand.
        """
        us, lengths = self._build_lut()
        total = lengths[-1]
        if total <= 0.0:
            return self.u0
        target = min(max(target, 0.0), total)
        index = int(np.searchsorted(lengths, target))
        index = min(max(index, 1), len(us) - 1)
        lo, hi = us[index - 1], us[index]
        u = lo + (hi - lo) * (
            (target - lengths[index - 1]) / max(1e-30, lengths[index] - lengths[index - 1])
        )
        for _ in range(NEWTON_MAX_STEPS):
            error = lengths[index - 1] + self.arc_length(lo, u) - target
            if abs(error) <= NEWTON_TOL:
                break
            speed = self.speed(u)
            if speed <= 1e-30:
                break
            u = min(max(u - error / speed, self.u0), self.u1)
        return float(u)


class Curve:
    """A composite curve: segments joined end to end.

    Normalized arc length `s` in [0, 1] spans the whole composite, which is the
    canonical parameter for everything in the weld program contract.
    """

    def __init__(self, spec: dict[str, Any]):
        self.segments = [Segment(s) for s in spec["segments"]]
        self.closed = bool(spec.get("closed", False))
        self.lengths = [s.length for s in self.segments]
        self.total = float(sum(self.lengths))
        self.cumulative = np.concatenate([[0.0], np.cumsum(self.lengths)])
        self.joins_s = self._joins_s(spec)

    def _joins_s(self, spec: dict[str, Any]) -> np.ndarray:
        """Where the segment joins sit in `s`, on the segments' DECLARED lengths.

        Not `self.cumulative / self.total`, and the difference is the whole
        point. Everything that hands this class a position at a join computes it
        the same way -- `continuity.junction_positions_s` sums `length_m` and
        divides by their total, and every corner-derived seam boundary in the
        committed corpus is bit-identical to a value in this array. Quadrature
        answers the same question independently and lands up to 0.9 nm away on
        `dog_box`'s rounded lobes, which is a real disagreement between two
        measurements of one arc and not something to resolve by rounding.

        So joins are compared on the scale they were AUTHORED on, and arc length
        is left to do the thing only it can: place a position that is not a join.
        Falls back to the quadrature cumulative where a segment declares no
        length, which is the only scale available then.
        """
        declared = [float(s.get("length_m", 0.0)) for s in spec["segments"]]
        if declared and all(d > 0.0 for d in declared):
            joins = np.concatenate([[0.0], np.cumsum(declared)])
            return joins / joins[-1]
        return self.cumulative / self.total if self.total > 0.0 else self.cumulative

    def at_join(self, s: float, left_limit: bool = False) -> tuple[int, float] | None:
        """`(segment index, u)` when `s` lands on a join, else None.

        A weld line corners at a join, so the tangent there is two-valued and
        the two answers differ by the whole corner angle. Which one is meant is
        not a property of the position -- it is a property of the caller's
        direction of travel, so it is asked for rather than inferred:

        - **right limit** (the default): the segment BEGINNING here. Intervals
          are half-open `[start, end)`, so this is where a seam departing its
          first point travels.
        - **left limit**: the segment ENDING here, which is how a seam arrives
          at its last point. At `s = 0` of a CLOSED curve that is the last
          segment -- the loop's closure point is arrived at from the far end of
          the parametrisation, and a span cut at a corner genuinely ends there.

        Both limits clamp at `s = 1`: it is the top of the range this function
        answers in, and a caller that means "departing the closure point" says
        `s = 0`, which is the same point.
        """
        last = len(self.segments) - 1
        index = int(np.argmin(np.abs(self.joins_s - s)))
        if abs(float(self.joins_s[index]) - s) > JOIN_TOL_S:
            return None
        if left_limit:
            if index > 0:
                return index - 1, self.segments[index - 1].u1
            return (last, self.segments[last].u1) if self.closed else (0, self.segments[0].u0)
        if index > last:
            return last, self.segments[last].u1
        return index, self.segments[index].u0

    def locate(self, s: float, left_limit: bool = False) -> tuple[int, float]:
        """Normalized arc length -> (segment index, local parameter u)."""
        s = min(max(s, 0.0), 1.0)
        at_join = self.at_join(s, left_limit)
        if at_join is not None:
            return at_join
        target = s * self.total
        index = int(np.searchsorted(self.cumulative, target, side="right")) - 1
        index = min(max(index, 0), len(self.segments) - 1)
        return index, self.segments[index].u_at_length(target - self.cumulative[index])

    def evaluate(
        self, s: float, left_limit: bool = False
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        index, u = self.locate(s, left_limit)
        return self.segments[index].evaluate(u)

    def point(self, s: float) -> np.ndarray:
        return self.evaluate(s)[0]

    def tangent(self, s: float, left_limit: bool = False) -> np.ndarray:
        d1 = self.evaluate(s, left_limit)[1]
        norm = float(np.linalg.norm(d1))
        return d1 / norm if norm > 1e-30 else np.array([1.0, 0.0, 0.0])

    def curvature(self, s: float, left_limit: bool = False) -> float:
        index, u = self.locate(s, left_limit)
        return self.segments[index].curvature(u)

    def sample_uniform(self, count: int) -> list[tuple[float, np.ndarray]]:
        """Points at equal arc-length spacing -- genuinely equal, not equal in
        parameter. Sampling uniformly in `u` is the defect that put one waypoint
        on an R ~ 7 mm corner in the prior CAD path."""
        return [(s, self.point(s)) for s in np.linspace(0.0, 1.0, max(2, count))]

    def sample_adaptive(
        self, max_chord_m: float, max_step_m: float, min_step_m: float = 1e-5
    ) -> list[float]:
        """Arc lengths chosen by curvature: dense where the curve bends, sparse
        where it does not. Chord deviation of a circular arc of radius R over a
        step L is about L^2 / (8R), which inverts to L = sqrt(8 R d)."""
        out = [0.0]
        s = 0.0
        while s < 1.0:
            kappa = self.curvature(s)
            radius = 1.0 / kappa if kappa > 1e-9 else float("inf")
            step = max_step_m if math.isinf(radius) else math.sqrt(8.0 * radius * max_chord_m)
            step = min(max(step, min_step_m), max_step_m)
            s = min(1.0, s + step / max(1e-30, self.total))
            out.append(s)
        return out


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    """Checks against closed forms, where an error has nowhere to hide."""
    quarter = {
        "segments": [
            {
                "degree": 2,
                "control_points_xyz_m": [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                "weights": [1.0, math.cos(math.pi / 4), 1.0],
                "knots_u": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            }
        ]
    }
    curve = Curve(quarter)
    assert abs(curve.total - math.pi / 2) < 1e-9, f"quarter circle length {curve.total}"
    for s in (0.0, 0.25, 0.5, 0.75, 1.0):
        point = curve.point(s)
        assert abs(float(np.linalg.norm(point)) - 1.0) < 1e-9, "point off the unit circle"
        assert abs(curve.curvature(s) - 1.0) < 1e-7, "curvature of a unit circle must be 1"
        angle = math.atan2(point[1], point[0])
        assert abs(angle - s * math.pi / 2) < 1e-7, "arc length must be uniform on a circle"
    print(
        f"curve self-test ok: quarter circle length {curve.total:.12f} "
        f"(pi/2 = {math.pi / 2:.12f}), curvature 1.0, arc length uniform"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--curve", default="", help="JSON file holding a curve object")
    parser.add_argument("--samples", type=int, default=16)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.curve:
        raise ValueError("--curve is required unless --self-test is set")
    curve = Curve(json.loads(Path(ns.curve).read_text(encoding="utf-8")))
    rows = [
        {
            "s": round(s, 6),
            "point_xyz_m": [round(float(c), 9) for c in point],
            "curvature_1_per_m": round(curve.curvature(s), 6),
        }
        for s, point in curve.sample_uniform(ns.samples)
    ]
    print(json.dumps({"length_m": curve.total, "samples": rows}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"curve: {exc}", file=sys.stderr)
        raise SystemExit(1)
