"""The free-space trajectory's basis: what it guarantees, in absolute terms.

`docs/issues/0015` is the reason this module exists, so the assertions here are
the properties that issue turns on -- `C2`, exact endpoints, analytic
derivatives -- and every threshold is a number, not a quantity derived from the
sampling the curve happens to use.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

from weld_motion_planner.planner.trajectory_optimization.engine.bspline import (
    DEGREE,
    Basis,
    basis,
    clamped_knots,
)

DOF = 6


def _control(n_control: int = 16, seed: int = 0) -> np.ndarray:
    return np.random.default_rng(seed).normal(size=(n_control, DOF))


def test_the_basis_is_a_partition_of_unity_and_its_derivatives_sum_to_zero():
    """Every sample is a weighted AVERAGE of the control points.

    If the weights did not sum to one the curve would drift off its own control
    polygon under a rigid translation of the polygon, which is not a spline.
    The derivative rows summing to zero is the same statement differentiated:
    translating every control point moves the curve and changes no velocity.
    """
    knots = clamped_knots(DEGREE, 16)
    u = np.linspace(0.0, 1.0, 257)
    position = basis(u, DEGREE, knots, 16, order=0)
    assert np.abs(position.sum(axis=1) - 1.0).max() < 1e-12
    assert (position >= -1e-12).all(), "a B-spline basis is non-negative"
    for order in (1, 2, 3):
        derivative = basis(u, DEGREE, knots, 16, order=order)
        assert np.abs(derivative.sum(axis=1)).max() < 1e-9


def test_the_curve_starts_and_ends_exactly_on_its_end_control_points():
    """A free-space motion's ends are DATA -- a weld pose, the reset pose -- so
    the clamped knot vector has to hit them exactly, not approach them."""
    control = _control()
    knots = clamped_knots(DEGREE, len(control))
    ends = basis(np.array([0.0, 1.0]), DEGREE, knots, len(control)) @ control
    np.testing.assert_allclose(ends[0], control[0], atol=1e-12)
    np.testing.assert_allclose(ends[-1], control[-1], atol=1e-12)


def test_the_derivatives_are_the_curves_own_and_not_a_finite_difference():
    """Central differences of the CURVE converge to what the matrices report.

    Second order in `h`, which is the signature of the difference being the
    approximation and the matrix being exact. Sampled between knots: at a knot
    the third derivative jumps, and a central difference across a jump measures
    the jump.
    """
    control = _control()
    knots = clamped_knots(DEGREE, len(control))

    def evaluate(u, order=0):
        return basis(np.asarray(u), DEGREE, knots, len(control), order) @ control

    inside = np.array([0.13, 0.27, 0.41, 0.58, 0.73, 0.88])
    errors = []
    for h in (1e-2, 1e-3, 1e-4):
        difference = (evaluate(inside + h) - evaluate(inside - h)) / (2 * h)
        errors.append(np.abs(difference - evaluate(inside, 1)).max())
    # Each tenfold refinement cuts the error a hundredfold, give or take.
    assert errors[1] < errors[0] / 50.0
    assert errors[2] < errors[1] / 50.0
    assert errors[2] < 1e-4


def test_acceleration_is_continuous_across_every_interior_knot():
    """**The property `docs/issues/0015` is about.**

    The Catmull-Rom curve this replaced jumped 1.73 rad/s^2 at a knot -- 0.83
    of this cell's own acceleration limit -- and `check_limits` never saw it,
    because it differences the waypoints it is handed and the jump lives
    between them. A cubic B-spline is `C2`, so there is nothing to see.

    Jerk is asserted DISCONTINUOUS in the same breath: degree 3 is `C2` and not
    `C3`, and a test that demanded otherwise would be wrong about the basis.
    """
    control = _control()
    knots = clamped_knots(DEGREE, len(control))
    interior = np.unique(knots[DEGREE + 1:-(DEGREE + 1)])
    assert len(interior) > 0, "this shape has no interior knot to test"

    def evaluate(u, order):
        return basis(np.asarray(u), DEGREE, knots, len(control), order) @ control

    step = 1e-9
    acceleration_jump = 0.0
    jerk_jump = 0.0
    for knot in interior:
        left, right = evaluate([knot - step], 2), evaluate([knot + step], 2)
        acceleration_jump = max(acceleration_jump, np.abs(left - right).max())
        left, right = evaluate([knot - step], 3), evaluate([knot + step], 3)
        jerk_jump = max(jerk_jump, np.abs(left - right).max())

    assert acceleration_jump < 1e-4, (
        f"acceleration jumps {acceleration_jump:.2e} rad/s^2 across a knot; "
        "the curve is not C2 and issue 0015 is back")
    assert jerk_jump > 1.0, (
        "jerk is continuous, so this is not the cubic basis it claims to be")


def test_the_horizon_scales_the_derivatives_and_not_the_shape():
    """Twice the horizon is half the velocity and a quarter the acceleration.

    The same control points on a longer horizon are the same PATH flown more
    slowly. `retime` shrinks a horizon repeatedly, so a basis that folded the
    duration into its shape would change the geometry every time it did.
    """
    control = torch.as_tensor(_control(), dtype=torch.float64)
    shaped = Basis(len(control), 129, device=torch.device("cpu"))
    slow_q, slow_v, slow_a, slow_j = shaped.states(control, 4.0)
    fast_q, fast_v, fast_a, fast_j = shaped.states(control, 2.0)
    torch.testing.assert_close(slow_q, fast_q)
    torch.testing.assert_close(slow_v * 2.0, fast_v)
    torch.testing.assert_close(slow_a * 4.0, fast_a)
    torch.testing.assert_close(slow_j * 8.0, fast_j)


def test_a_batch_of_motions_may_each_have_their_own_horizon():
    """One tensor problem, many horizons: `plan_moves` folds every motion into
    one batch and `retime` gives each its own clock."""
    control = torch.as_tensor(
        np.stack([_control(seed=1), _control(seed=2)]), dtype=torch.float64)
    shaped = Basis(control.shape[1], 65, device=torch.device("cpu"))
    duration = torch.tensor([[[2.0]], [[4.0]]], dtype=torch.float64)
    _, velocity, _, _ = shaped.states(control, duration)
    _, first, _, _ = shaped.states(control[0], 2.0)
    _, second, _, _ = shaped.states(control[1], 4.0)
    torch.testing.assert_close(velocity[0], first)
    torch.testing.assert_close(velocity[1], second)


def test_a_shape_too_small_for_the_degree_is_refused():
    with pytest.raises(ValueError):
        clamped_knots(DEGREE, DEGREE)


def test_the_derivatives_hold_at_the_closing_end_too():
    """At `u = 1` the half-open recurrence reads zero unless the closing
    function is written down, and for a derivative that function lives in a
    basis with one MORE function than the curve's. Every derivative there,
    against scipy's B-spline on the same knots and polygon, not just the
    position -- the position was always right, which is how the derivatives
    went unnoticed."""
    from scipy.interpolate import BSpline

    control = _control(seed=5)
    knots = clamped_knots(DEGREE, 16)
    reference = BSpline(knots, control, DEGREE)
    u = np.array([0.0, 0.37, 1.0])
    for order in range(DEGREE + 1):
        ours = basis(u, DEGREE, knots, 16, order=order) @ control
        theirs = reference.derivative(order)(u) if order else reference(u)
        assert np.abs(ours - theirs).max() < 1e-9 * max(1.0, np.abs(theirs).max()), order
