#!/usr/bin/env python
"""Rational B-splines, in torch, differentiably.

The weld program carries its geometry as exact rational B-splines, and the
optimizer differentiates through them: the seam constraint is
`FK(q(s_i))` against `P(s_i)`, so `P` has to live inside the autodiff graph
alongside the robot. That is the whole reason this exists rather than a call
into the authoring side's evaluator.

## What is reimplemented, and what that costs

de Boor on homogeneous control points, the quotient rule for rational
derivatives, panelled Gauss-Legendre for arc length, and Newton for the
`s -> u` inversion. Every one of those is a convention with a way to be
subtly wrong, so `data/motion/oracles/seams.*.json` pins all of them against
`seam_worker.seam.curve` -- see `tests/motion_planner/test_curves.py`.

## Batched over samples, looped over segments

A composite curve's segments have different knot vectors and different
control-point counts, so they cannot be one tensor without padding. They are
also FEW -- eight for the dogbone perimeter, four for a pipe fillet -- while
samples number in the thousands and seeds multiply that again. So the loop
is over segments and the batch is over positions: `evaluate` masks the
sample set per segment and gathers. Kernel launches go as the segment count,
which is the small number.

## Float64 by default, float32 on request

`ARCHITECTURE.md` puts fp32 on the GPU because a consumer card runs fp64 at
1/64 rate, and the search only needs to land in the right basin. But the
parity tests, the verifier and the polish all want fp64, and a module that
silently used fp32 would make the oracle comparison meaningless. So the
dtype is explicit and defaults to the accurate one; the optimizer asks for
fp32 deliberately.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from typing import Any

import torch

# Gauss-Legendre on [-1, 1], 8 nodes: exact through degree 15, which covers
# the integrand |C'(u)| for any degree this format produces once the interval
# is panelled. Panels do the accuracy work; the order only has to be enough
# that each panel is cheap.
_GL_NODES = (
    -0.9602898564975363,
    -0.7966664774136267,
    -0.5255324099163290,
    -0.1834346424956498,
    0.1834346424956498,
    0.5255324099163290,
    0.7966664774136267,
    0.9602898564975363,
)
_GL_WEIGHTS = (
    0.1012285362903763,
    0.2223810344533745,
    0.3137066458778873,
    0.3626837833783620,
    0.3626837833783620,
    0.3137066458778873,
    0.2223810344533745,
    0.1012285362903763,
)

# Panels per knot span for the arc-length lookup. Sixteen puts the table's
# own interpolation error far below the Newton tolerance that follows it, and
# the table is built once per seam.
PANELS_PER_SPAN = 16
NEWTON_STEPS = 6
NEWTON_TOL_M = 1e-13

# A position this close to a segment join, in NORMALIZED arc length, is ON that
# join -- so the side of it is chosen by `Curve.locate`'s stated rule rather
# than by which way a float landed. The authoring side sets the same constant
# for the same reason and `Curve.joins_s` explains it; 1e-12 is 0.19 nm on a
# 190 mm weld line, four orders below the coarsest boundary anything upstream
# measures and eight above the noise it absorbs.
JOIN_TOL_S = 1e-12


class CurveError(ValueError):
    """A curve this refuses to evaluate, with the reason."""


def _derivative(degree: int, knots: torch.Tensor, points: torch.Tensor):
    """The B-spline whose value is this one's first derivative.

    In homogeneous coordinates, so it is the derivative of `A(u)`, not of the
    projected curve. Projecting first and differencing would give up exactly
    the exactness the rational form exists to provide.
    """
    if degree < 1:
        zero = torch.zeros_like(points[:1])
        return 0, knots[1:-1], zero
    span = knots[degree + 1 : len(points) + degree] - knots[1 : len(points)]
    safe = torch.where(span.abs() < 1e-30, torch.ones_like(span), span)
    delta = degree * (points[1:] - points[:-1]) / safe.unsqueeze(-1)
    return degree - 1, knots[1:-1], delta


class Segment:
    """One clamped rational B-spline segment."""

    def __init__(self, spec: dict[str, Any], *, dtype=torch.float64, device=None):
        self.degree = int(spec["degree"])
        self.dtype, self.device = dtype, device
        knots = torch.as_tensor(spec["knots_u"], dtype=dtype, device=device)
        points = torch.as_tensor(spec["control_points_xyz_m"], dtype=dtype, device=device)
        weights = spec.get("weights")
        w = (
            torch.as_tensor(weights, dtype=dtype, device=device)
            if weights is not None
            else torch.ones(len(points), dtype=dtype, device=device)
        )
        if len(knots) != len(points) + self.degree + 1:
            raise CurveError(
                f"a degree-{self.degree} segment with {len(points)} control points needs "
                f"{len(points) + self.degree + 1} knots, not {len(knots)}"
            )
        self.knots = knots
        # Homogeneous: rational evaluation is de Boor in 4D and a divide.
        self.homogeneous = torch.cat([points * w.unsqueeze(-1), w.unsqueeze(-1)], dim=-1)
        self.u0 = float(knots[self.degree])
        self.u1 = float(knots[len(knots) - self.degree - 1])

        self._d1 = _derivative(self.degree, self.knots, self.homogeneous)
        self._d2 = _derivative(*self._d1)
        self._lut_u: torch.Tensor | None = None
        self._lut_s: torch.Tensor | None = None

    # -- de Boor ---------------------------------------------------------

    def _de_boor(self, u: torch.Tensor, degree: int, knots: torch.Tensor, points: torch.Tensor):
        """Batched de Boor. `u` is (...,); the result is (..., 4)."""
        if degree == 0:
            index = torch.clamp(
                torch.searchsorted(knots.contiguous(), u.contiguous(), right=True) - 1,
                0,
                len(points) - 1,
            )
            return points[index]
        n = len(points)
        span = torch.clamp(
            torch.searchsorted(knots.contiguous(), u.contiguous(), right=True) - 1, degree, n - 1
        )
        # d[j] = P[span - degree + j], gathered per sample.
        offsets = torch.arange(degree + 1, device=u.device)
        rows = points[(span.unsqueeze(-1) - degree + offsets).clamp(0, n - 1)]
        # A PYTHON list of tensors, not slice assignment into one. Writing
        # `d[..., j, :] = ...` is an in-place update of a view, and autograd
        # refuses to differentiate through it -- the seam constraint needs
        # dP/ds, so a curve that evaluates correctly and cannot be
        # differentiated is no use here. Descending `j` still reads the
        # previous round's `d[j-1]`, which is what makes this de Boor.
        d = [rows[..., j, :] for j in range(degree + 1)]
        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                lo = knots[(j + span - degree).clamp(0, len(knots) - 1)]
                hi = knots[(j + 1 + span - r).clamp(0, len(knots) - 1)]
                width = hi - lo
                safe = torch.where(width.abs() < 1e-30, torch.ones_like(width), width)
                alpha = torch.where(
                    width.abs() < 1e-30, torch.zeros_like(width), (u - lo) / safe
                ).unsqueeze(-1)
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]
        return d[degree]

    def evaluate(self, u: torch.Tensor):
        """`(point, C', C'')` at parameter `u`, batched.

        Rational derivatives by the quotient rule on `C = A / w`:
            C'  = (A' - w' C) / w
            C'' = (A'' - 2 w' C' - w'' C) / w
        """
        u = torch.clamp(u, self.u0, self.u1)
        a = self._de_boor(u, self.degree, self.knots, self.homogeneous)
        a1 = self._de_boor(u, *self._d1)
        a2 = self._de_boor(u, *self._d2)
        w, w1, w2 = a[..., 3:4], a1[..., 3:4], a2[..., 3:4]
        point = a[..., :3] / w
        first = (a1[..., :3] - w1 * point) / w
        second = (a2[..., :3] - 2.0 * w1 * first - w2 * point) / w
        return point, first, second

    def speed(self, u: torch.Tensor) -> torch.Tensor:
        """`|C'(u)|`: the arc-length integrand, without paying for C''."""
        u = torch.clamp(u, self.u0, self.u1)
        a = self._de_boor(u, self.degree, self.knots, self.homogeneous)
        a1 = self._de_boor(u, *self._d1)
        w = a[..., 3:4]
        point = a[..., :3] / w
        return torch.linalg.norm((a1[..., :3] - a1[..., 3:4] * point) / w, dim=-1)

    def curvature(self, u: torch.Tensor) -> torch.Tensor:
        """`|C' x C''| / |C'|^3`, from the analytic derivatives.

        Never from finite differences over a polyline -- GOAL.md
        non-negotiable 4, and the reason C'' is computed exactly above.
        """
        _, d1, d2 = self.evaluate(u)
        speed = torch.linalg.norm(d1, dim=-1)
        cross = torch.linalg.norm(torch.cross(d1, d2, dim=-1), dim=-1)
        return torch.where(speed > 1e-30, cross / speed.clamp_min(1e-30) ** 3, torch.zeros_like(speed))

    # -- arc length ------------------------------------------------------

    def _build_lut(self) -> None:
        """Cumulative arc length at panel boundaries across the whole segment."""
        breaks = torch.unique(
            torch.clamp(self.knots[self.degree : len(self.knots) - self.degree], self.u0, self.u1)
        )
        edges = [breaks[0]]
        for lo, hi in itertools.pairwise(breaks):
            step = (hi - lo) / PANELS_PER_SPAN
            edges.extend(lo + step * (k + 1) for k in range(PANELS_PER_SPAN))
        us = torch.stack(edges)
        nodes = torch.tensor(_GL_NODES, dtype=self.dtype, device=self.device)
        weights = torch.tensor(_GL_WEIGHTS, dtype=self.dtype, device=self.device)
        lo, hi = us[:-1], us[1:]
        mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        sampled = self.speed((mid.unsqueeze(-1) + half.unsqueeze(-1) * nodes).reshape(-1)).reshape(
            len(lo), len(nodes)
        )
        panels = half * (sampled * weights).sum(dim=-1)
        self._lut_u = us
        self._lut_s = torch.cat(
            [torch.zeros(1, dtype=self.dtype, device=self.device), torch.cumsum(panels, dim=0)]
        )

    @property
    def length(self) -> float:
        if self._lut_s is None:
            self._build_lut()
        return float(self._lut_s[-1])

    def arc_length_to(self, u: torch.Tensor) -> torch.Tensor:
        """Arc length from `u0` to `u`, batched."""
        if self._lut_s is None:
            self._build_lut()
        u = torch.clamp(u, self.u0, self.u1)
        index = torch.clamp(
            torch.searchsorted(self._lut_u.contiguous(), u.contiguous(), right=True) - 1,
            0,
            len(self._lut_u) - 2,
        )
        lo = self._lut_u[index]
        nodes = torch.tensor(_GL_NODES, dtype=self.dtype, device=self.device)
        weights = torch.tensor(_GL_WEIGHTS, dtype=self.dtype, device=self.device)
        mid, half = 0.5 * (lo + u), 0.5 * (u - lo)
        sampled = self.speed(mid.unsqueeze(-1) + half.unsqueeze(-1) * nodes)
        return self._lut_s[index] + half * (sampled * weights).sum(dim=-1)

    def u_at_length(self, target: torch.Tensor) -> torch.Tensor:
        """Invert `arc_length_to`: the parameter at a given arc length.

        Table lookup for the bracket, then Newton, whose derivative is the
        speed and is therefore free. Bisection alone would need ~45
        iterations for the same tolerance; Newton needs a handful because
        arc length is monotone and smooth in `u`.
        """
        if self._lut_s is None:
            self._build_lut()
        target = torch.clamp(target, torch.zeros_like(target), torch.full_like(target, self.length))
        index = torch.clamp(
            torch.searchsorted(self._lut_s.contiguous(), target.contiguous(), right=True) - 1,
            0,
            len(self._lut_s) - 2,
        )
        lo_u, hi_u = self._lut_u[index], self._lut_u[index + 1]
        lo_s, hi_s = self._lut_s[index], self._lut_s[index + 1]
        width = (hi_s - lo_s).clamp_min(1e-30)
        u = lo_u + (hi_u - lo_u) * ((target - lo_s) / width)
        for _ in range(NEWTON_STEPS):
            error = self.arc_length_to(u) - target
            # `detach` because this is a CONVERGENCE test, not a value: without
            # it, reading the scalar drags a grad-tracking tensor into Python
            # and warns about it on every call.
            if float(error.detach().abs().max()) < NEWTON_TOL_M:
                break
            u = torch.clamp(u - error / self.speed(u).clamp_min(1e-30), self.u0, self.u1)
        return u


class Curve:
    """A composite curve: segments joined end to end, parametrized by `s`.

    `s` in [0, 1] is normalized arc length over the WHOLE composite, which is
    the contract's canonical parameter (`SEAM-MODEL.md` §3).
    """

    def __init__(self, spec: dict[str, Any], *, dtype=torch.float64, device=None):
        segments = spec.get("segments")
        if not segments:
            raise CurveError("a curve needs at least one segment")
        self.segments = [Segment(s, dtype=dtype, device=device) for s in segments]
        self.closed = bool(spec.get("closed", False))
        self.dtype, self.device = dtype, device
        lengths = [seg.length for seg in self.segments]
        self.total = float(sum(lengths))
        self.cumulative = torch.tensor(
            [0.0, *torch.cumsum(torch.tensor(lengths, dtype=dtype), dim=0).tolist()],
            dtype=dtype,
            device=device,
        )
        self.joins_s = self._joins_s(segments)
        # The segments' own end parameters, gathered so that a position landing
        # on a join can be answered exactly instead of Newton-solving its way
        # back to a value it already knows.
        self._u_start = torch.tensor([seg.u0 for seg in self.segments], dtype=dtype, device=device)
        self._u_end = torch.tensor([seg.u1 for seg in self.segments], dtype=dtype, device=device)

    def _joins_s(self, segments: list[dict[str, Any]]) -> torch.Tensor:
        """Where the segment joins sit in `s`, on the segments' DECLARED lengths.

        Not `cumulative / total`, and the difference is what makes a corner
        land. A position handed to this class at a join was computed by the
        authoring side as a running sum of `length_m` over its total, so on
        that scale it is exact; quadrature measures the same arc independently
        and answers up to 0.9 nm differently on `dog_box`'s rounded lobes.
        Against the quadrature cumulative a seam boundary sitting exactly on a
        corner therefore falls an unpredictable few ulps to one side of it, and
        `locate` reads the tangent off the wrong segment -- 90 degrees wrong,
        at the one position where a weld line's tangent is two-valued.

        Joins are compared on the scale they were authored on; arc length keeps
        the job only it can do, placing a position that is not a join. Where a
        segment declares no length there is no other scale, so quadrature it is.
        """
        declared = [float(s.get("length_m", 0.0)) for s in segments]
        if declared and all(d > 0.0 for d in declared):
            joins = [0.0]
            for length in declared:
                joins.append(joins[-1] + length)
            return torch.tensor(
                [j / joins[-1] for j in joins], dtype=self.dtype, device=self.device
            )
        return self.cumulative / max(self.total, 1e-30)

    def _as_tensor(self, s: Any) -> torch.Tensor:
        if isinstance(s, torch.Tensor):
            return s.to(dtype=self.dtype, device=self.device)
        return torch.as_tensor(s, dtype=self.dtype, device=self.device)

    def locate(self, s: Any, left_limit: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        """Normalized arc length to (segment index, local parameter u).

        `left_limit` decides which segment a position lands in when it falls
        on a join, and that is not a nicety: segment joins are where a weld
        line corners, so the tangent there is genuinely two-valued and the two
        answers differ by the whole corner angle.

        The default is the right limit -- intervals are half-open
        `[start, end)`, so a position on a join belongs to the segment
        beginning there. `Seam.frames` asks for the left limit at the end of
        a span, because a seam's last point is ARRIVED at: the direction of
        travel there is the one coming in, not the one the next segment
        would have departed along. On a CLOSED curve the left limit at `s = 0`
        is the last segment: the loop's closure point is arrived at from the
        far end of the parametrisation, and a span cut at a corner there --
        `bracket_b`'s second seam is one -- genuinely ends on it.

        Whether a position IS on a join is decided in `joins_s`, on the
        authoring side's own arc-length scale, and not by comparing the
        quadrature cumulative against `s * total`. That comparison is the bug
        this rule used to be defeated by: a boundary lying exactly on a corner
        misses the quadrature's answer for it by an ulp or a nanometre, in a
        direction nothing controls, and the stated limit then resolves against
        a join the position is no longer on.
        """
        s = torch.clamp(self._as_tensor(s), 0.0, 1.0)
        target = s * self.total
        knots = self.cumulative.contiguous()
        index = torch.searchsorted(knots, target.contiguous(), right=True) - 1
        index = torch.clamp(index, 0, len(self.segments) - 1)
        within = target - self.cumulative[index]
        u = torch.zeros_like(target)
        for i, segment in enumerate(self.segments):
            mask = index == i
            if bool(mask.any()):
                u[mask] = segment.u_at_length(within[mask])

        on_join, join_index, join_u = self._at_join(s, left_limit)
        if bool(on_join.any()):
            index = torch.where(on_join, join_index, index)
            u = torch.where(on_join, join_u, u)
        return index, u

    def _at_join(
        self, s: torch.Tensor, left_limit: torch.Tensor | None
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """`(on a join, segment index, u)` for a batch of positions.

        The index and `u` are only meaningful where the mask is set; the answer
        is exact there rather than Newton-solved, because a join's parameter is
        a segment's own `u0` or `u1` and nothing about it needs measuring.
        """
        last = len(self.segments) - 1
        gap = (s.unsqueeze(-1) - self.joins_s.to(s.device)).abs()
        join = gap.argmin(dim=-1)
        on_join = gap.gather(-1, join.unsqueeze(-1)).squeeze(-1) <= JOIN_TOL_S

        left = (
            torch.zeros_like(on_join)
            if left_limit is None
            else left_limit.to(device=on_join.device, dtype=torch.bool).expand_as(on_join)
        )
        # Arriving: the segment ENDING at the join, wrapping to the last one at
        # s=0 of a closed loop. Departing: the one beginning there. Both clamp
        # at s=1, the top of the range this answers in.
        at_the_start = left & (join == 0)
        wrapped = at_the_start if self.closed else torch.zeros_like(on_join)
        clamped_start = torch.zeros_like(on_join) if self.closed else at_the_start
        index = torch.where(left, join - 1, torch.clamp(join, max=last))
        index = torch.where(wrapped, torch.full_like(join, last), index)
        index = torch.clamp(index, 0, last)
        at_end = (left & ~clamped_start) | (~left & (join > last))
        u = torch.where(at_end, self._u_end.to(s.device)[index], self._u_start.to(s.device)[index])
        return on_join, index, u

    def evaluate(self, s: Any, left_limit: torch.Tensor | None = None):
        """`(point, C', C'')` at normalized arc length, batched."""
        index, u = self.locate(s, left_limit)
        shape = (*u.shape, 3)
        point = torch.zeros(shape, dtype=self.dtype, device=self.device)
        first = torch.zeros(shape, dtype=self.dtype, device=self.device)
        second = torch.zeros(shape, dtype=self.dtype, device=self.device)
        for i, segment in enumerate(self.segments):
            mask = index == i
            if not bool(mask.any()):
                continue
            p, d1, d2 = segment.evaluate(u[mask])
            point[mask], first[mask], second[mask] = p, d1, d2
        return point, first, second

    def point(self, s: Any) -> torch.Tensor:
        return self.evaluate(s)[0]

    def tangent(self, s: Any, left_limit: torch.Tensor | None = None) -> torch.Tensor:
        d1 = self.evaluate(s, left_limit)[1]
        norm = torch.linalg.norm(d1, dim=-1, keepdim=True)
        fallback = torch.zeros_like(d1)
        fallback[..., 0] = 1.0
        return torch.where(norm > 1e-30, d1 / norm.clamp_min(1e-30), fallback)

    def curvature(self, s: Any, left_limit: torch.Tensor | None = None) -> torch.Tensor:
        index, u = self.locate(s, left_limit)
        out = torch.zeros_like(u)
        for i, segment in enumerate(self.segments):
            mask = index == i
            if bool(mask.any()):
                out[mask] = segment.curvature(u[mask])
        return out

    def s_within_segment(
        self, s: Any, left_limit: torch.Tensor | None = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """(segment index, fraction along that segment by arc length).

        What per-segment rail correspondence needs: `SEAM-MODEL.md` §5 maps
        rail segment `i` to seam segment `i` by normalized arc length WITHIN
        the segment, which is not the same as by `s` over the whole curve
        unless the segments happen to be equal in length.
        """
        index, u = self.locate(s, left_limit)
        within = torch.zeros_like(u)
        for i, segment in enumerate(self.segments):
            mask = index == i
            if bool(mask.any()):
                within[mask] = segment.arc_length_to(u[mask]) / max(segment.length, 1e-30)
        return index, within

    def point_at_segment_fraction(self, index: torch.Tensor, fraction: torch.Tensor) -> torch.Tensor:
        """The inverse direction: a point from (segment, fraction within it)."""
        out = torch.zeros((*fraction.shape, 3), dtype=self.dtype, device=self.device)
        for i, segment in enumerate(self.segments):
            mask = index == i
            if bool(mask.any()):
                u = segment.u_at_length(fraction[mask] * segment.length)
                out[mask] = segment.evaluate(u)[0]
        return out


def self_test() -> int:
    """Closed forms, because an oracle can only say two implementations
    agree. A circle and a straight line have arc lengths and curvatures
    nobody has to compute."""
    import math

    # A unit circle as a degree-2 rational B-spline: the standard nine-point,
    # four-arc form. Exactly the construction the extractor emits for a fillet
    # round, and the one that would break if weights were ignored.
    r2 = math.sqrt(2.0) / 2.0
    circle = {
        "segments": [
            {
                "degree": 2,
                "knots_u": [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1],
                "control_points_xyz_m": [
                    [1, 0, 0], [1, 1, 0], [0, 1, 0], [-1, 1, 0], [-1, 0, 0],
                    [-1, -1, 0], [0, -1, 0], [1, -1, 0], [1, 0, 0],
                ],
                "weights": [1, r2, 1, r2, 1, r2, 1, r2, 1],
            }
        ],
        "closed": True,
    }
    curve = Curve(circle)
    assert abs(curve.total - 2.0 * math.pi) < 1e-9, f"circumference {curve.total}"

    s = torch.linspace(0.0, 1.0, 17, dtype=torch.float64)
    points = curve.point(s)
    radii = torch.linalg.norm(points, dim=-1)
    assert float((radii - 1.0).abs().max()) < 1e-12, f"not on the unit circle: {radii}"
    assert float((curve.curvature(s) - 1.0).abs().max()) < 1e-8, "curvature of a unit circle is 1"

    # Equal arc length means equal ANGLE on a circle, which is the property a
    # parameter-uniform sampler fails and the reason it is banned. Measured
    # over the open interval: on a closed curve s=1 IS s=0, so including both
    # ends would be measuring the wrap rather than the spacing.
    open_points = curve.point(torch.linspace(0.0, 1.0, 17, dtype=torch.float64)[:-1])
    angles = torch.atan2(open_points[:, 1], open_points[:, 0])
    steps = torch.diff(torch.where(angles < -1e-12, angles + 2 * math.pi, angles))
    assert float(steps.max() - steps.min()) < 1e-9, f"uneven arc-length spacing: {steps}"
    assert abs(float(steps.mean()) - 2.0 * math.pi / 16.0) < 1e-9

    # A line: length is exact, curvature is zero, and the tangent is constant.
    line = {
        "segments": [
            {
                "degree": 1,
                "knots_u": [0, 0, 1, 1],
                "control_points_xyz_m": [[0, 0, 0], [3, 4, 0]],
            }
        ]
    }
    straight = Curve(line)
    assert abs(straight.total - 5.0) < 1e-12
    assert float(straight.curvature(s).abs().max()) < 1e-12
    assert torch.allclose(
        straight.tangent(s), torch.tensor([0.6, 0.8, 0.0], dtype=torch.float64).expand(17, 3)
    )
    half = straight.point(torch.tensor([0.5], dtype=torch.float64))
    assert torch.allclose(half, torch.tensor([[1.5, 2.0, 0.0]], dtype=torch.float64), atol=1e-12)

    # Differentiable end to end, which is the point of doing this in torch:
    # d|P(s)|/ds along a unit circle is zero, and the gradient must flow.
    su = torch.tensor([0.3], dtype=torch.float64, requires_grad=True)
    torch.linalg.norm(curve.point(su)).backward()
    assert su.grad is not None and float(su.grad.abs()) < 1e-6, su.grad

    # float32 still lands in the right place, which is all the search needs.
    cheap = Curve(circle, dtype=torch.float32)
    assert abs(cheap.total - 2.0 * math.pi) < 1e-3, cheap.total

    print(
        "curves self-test ok: a rational circle has the right circumference, radius, curvature "
        "and even spacing; a line is exact; gradients flow; float32 lands close"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose seams to measure")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise CurveError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read

    request = read(ns.read)
    joints = request.joints_by_id
    out = []
    for seam in request.seams:
        spec = (joints.get(seam.get("weld_joint_ref")) or {}).get("weld_line")
        if not spec:
            continue
        curve = Curve(spec)
        out.append(
            {
                "seam_id": seam.get("id"),
                "segments": len(curve.segments),
                "closed": curve.closed,
                "joint_length_m": round(curve.total, 9),
                "max_curvature_1_m": round(
                    float(curve.curvature(torch.linspace(0, 1, 201, dtype=torch.float64)).max()), 6
                ),
            }
        )
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"curves: {exc}", file=sys.stderr)
        raise SystemExit(1)
