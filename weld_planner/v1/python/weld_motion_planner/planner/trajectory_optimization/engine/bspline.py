#!/usr/bin/env python
"""Uniform clamped B-splines as CONSTANT matrices, for the free-space stage.

## Why a matrix and not de Boor

For a fixed degree, control-point count and sample count, the map from control
points to sampled states is linear and does not depend on the control points:

    q = B  @ P        v = B1 @ P        a = B2 @ P        j = B3 @ P

So it is four matrices, built once and reused for every motion of that shape.
De Boor's corner cut is the right algorithm when the curve is evaluated at
arbitrary parameters -- which is what `geometry/curves.py` does for the seam's
rational B-splines -- but here the sample grid is fixed and the same for every
row of the batch, so the whole evaluation is one `matmul` per derivative.
That is also what a CUDA graph wants: fixed shapes, no control flow.

## Why this exists at all

`docs/issues/0015`. The free-space trajectory used to be a cubic Hermite whose
tangents were centred differences of its own knots -- a Catmull-Rom spline,
which is `C1`. Acceleration was therefore DISCONTINUOUS at every knot, measured
at 1.73 rad/s^2 on one transit, 0.83 of this cell's own limit, from the seam
between two segments alone. `verify.check_limits` differences the waypoints it
is handed, so it samples the knots and steps over the jump.

A cubic B-spline is `C2` at simple knots: acceleration is continuous by
construction, and the jump is gone rather than penalised. Measured here, the
acceleration either side of an interior knot agrees to 6e-06. Jerk stays
discontinuous, which is exactly right for degree 3 and is what the jerk term
is for.

## The derivatives are analytic

Nothing here is finite-differenced. `B2` is the second derivative of the basis,
so the acceleration it returns is the curve's, at any sample density. This is
the property that makes a limit check mean something: a finite difference on a
coarse grid reads a smaller acceleration than the motion has, and on a fine one
reads the jump of a `C1` curve as unbounded.
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np
import torch

#: Cubic. Degree 3 is the lowest that is `C2`, and `C2` is the whole point.
#: Higher degrees are smoother still but pull the curve further from its
#: control polygon, which costs reach near the ends where a motion is clamped.
DEGREE = 3


def clamped_knots(degree: int, n_control: int) -> np.ndarray:
    """Uniform knot vector, clamped at both ends.

    Clamped so the curve BEGINS at its first control point and ENDS at its
    last: a free-space motion's endpoints are data (a weld pose, the reset
    pose) and must be hit exactly, not approached. Length is
    `n_control + degree + 1`, which is what the Cox-de Boor recurrence needs.
    """
    if n_control < degree + 1:
        raise ValueError(f"a degree-{degree} B-spline needs at least "
                         f"{degree + 1} control points, got {n_control}")
    inner = n_control - degree - 1
    return np.concatenate([
        np.zeros(degree + 1),
        np.arange(1, inner + 1, dtype=float) / (inner + 1),
        np.ones(degree + 1),
    ])


def _one_basis(u: np.ndarray, i: int, degree: int, knots: np.ndarray,
               n_control: int) -> np.ndarray:
    """Cox-de Boor for basis `i`, with the closing endpoint written down.

    The recurrence's support is half-open, so at `u = 1` every basis evaluates
    to zero -- including the last, which should be one. A clamped curve has to
    END on its last control point, so that single case is stated rather than
    left to the limit: the degree-0 function closed at `u = 1` is the one on
    the LAST NON-EMPTY knot interval.

    Picked by the knots and not by `n_control`, because `basis` evaluates a
    derivative through the degree-lower basis with one MORE function over the
    same knots. Chosen as `n_control - 1` of that count, the closed function
    was one whose interval is empty, and every derivative read zero at
    `u = 1` -- a motion's end velocity, acceleration and jerk, invisible to
    the cost and wrong in the wire wherever the end was not pinned
    (found 2026-09-17: 51 rad/s against zero on an unpinned end).
    """
    if degree == 0:
        inside = (knots[i] <= u) & (u < knots[i + 1])
        if knots[i] < knots[i + 1] and knots[i + 1] == knots[-1]:
            inside = inside | np.isclose(u, knots[-1])
        return inside.astype(float)
    out = np.zeros_like(u)
    left = knots[i + degree] - knots[i]
    right = knots[i + degree + 1] - knots[i + 1]
    # A zero denominator means that term's basis has empty support, so the term
    # is ABSENT -- not infinite. Clamped knot vectors repeat knots at the ends,
    # so this is the normal case there, not an edge case.
    if left > 0:
        out = out + (u - knots[i]) / left * _one_basis(
            u, i, degree - 1, knots, n_control)
    if right > 0:
        out = out + (knots[i + degree + 1] - u) / right * _one_basis(
            u, i + 1, degree - 1, knots, n_control)
    return out


def basis(u: np.ndarray, degree: int, knots: np.ndarray, n_control: int,
          order: int = 0) -> np.ndarray:
    """`(len(u), n_control)` basis, or its `order`-th derivative in `u`.

    The derivative recurrence is the standard one,

        d/du N_{i,p} = p * ( N_{i,p-1}/(t_{i+p} - t_i)
                           - N_{i+1,p-1}/(t_{i+p+1} - t_{i+1}) )

    which lowers the degree by one and needs one more basis function, over the
    SAME knot vector. Above the degree every derivative is zero.
    """
    u = np.atleast_1d(np.asarray(u, dtype=float))
    if order > degree:
        return np.zeros((len(u), n_control))
    if order == 0:
        return np.stack([_one_basis(u, i, degree, knots, n_control)
                         for i in range(n_control)], axis=1)
    lower = basis(u, degree - 1, knots, n_control + 1, order - 1)
    out = np.zeros((len(u), n_control))
    for i in range(n_control):
        left = knots[i + degree] - knots[i]
        right = knots[i + degree + 1] - knots[i + 1]
        if left > 0:
            out[:, i] += degree * lower[:, i] / left
        if right > 0:
            out[:, i] -= degree * lower[:, i + 1] / right
    return out


@lru_cache(maxsize=64)
def _u_matrices(degree: int, n_control: int, n_samples: int
                ) -> tuple[np.ndarray, ...]:
    """`(B, B1, B2, B3)` in the PARAMETER `u`, cached by shape.

    Kept free of `duration` so the cache key is the shape alone: `retime`
    changes the horizon many times over one motion and the basis does not
    depend on it. Dividing by `duration**order` at use turns these into time
    derivatives.
    """
    knots = clamped_knots(degree, n_control)
    u = np.linspace(0.0, 1.0, n_samples)
    return tuple(basis(u, degree, knots, n_control, order=k) for k in range(4))


class Basis:
    """The four matrices for one `(n_control, n_samples)` shape, on a device.

    `duration` is applied here rather than baked in, so one instance serves a
    motion through every horizon `retime` tries.
    """

    __slots__ = ("degree", "n_control", "n_samples", "position",
                 "_d1", "_d2", "_d3")

    def __init__(self, n_control: int, n_samples: int, *,
                 device: torch.device, degree: int = DEGREE,
                 dtype: torch.dtype = torch.float64):
        mats = _u_matrices(degree, n_control, n_samples)
        self.degree = degree
        self.n_control = n_control
        self.n_samples = n_samples
        self.position, self._d1, self._d2, self._d3 = (
            torch.as_tensor(m, dtype=dtype, device=device) for m in mats)

    def states(self, control: torch.Tensor, duration: torch.Tensor | float
               ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """`(q, v, a, j)` for control points `(B, n_control, dof)`.

        `duration` may be a scalar or `(B, 1, 1)`, which is how a batch of
        motions on different horizons stays one tensor problem.
        """
        if control.shape[-2] != self.n_control:
            raise ValueError(
                f"expected {self.n_control} control points, "
                f"got {control.shape[-2]}")
        return (self.position @ control,
                (self._d1 @ control) / duration,
                (self._d2 @ control) / duration ** 2,
                (self._d3 @ control) / duration ** 3)
