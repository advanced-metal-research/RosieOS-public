"""The rate cost's contract: free inside the envelope, dt-invariant outside.

These are the two properties `docs/issues/0015` turns on. The old term had
neither -- it charged for moving and never bounded acceleration at all -- so
each one is asserted directly rather than inferred from a planned motion.
"""

from __future__ import annotations

import torch

from weld_motion_planner.planner.trajectory_optimization.engine.rates import (
    ACTIVATION,
    Limits,
    RateCost,
    W_REG_ACCELERATION,
    bound_hinge,
    squared_regularization,
)

DOF = 3


def _limits() -> Limits:
    ones = torch.ones(DOF, dtype=torch.float64)
    return Limits(position_lower=-3.0 * ones, position_upper=3.0 * ones,
                  velocity=2.0 * ones, acceleration=1.0 * ones,
                  jerk=10.0 * ones)


def test_using_the_envelope_is_free_and_leaving_it_is_not():
    """A hinge, not a preference.

    The term this replaced was `(v/vmax)^2`, cheapest at zero, so it priced
    MOVING: a motion that used its limits paid for it and the optimizer bought
    slowness it was never asked for. Nothing inside the bound may cost
    anything, or that returns.
    """
    lower = torch.full((DOF,), -1.0, dtype=torch.float64)
    upper = torch.full((DOF,), 1.0, dtype=torch.float64)
    inside = torch.tensor([[[0.0, 0.5, 0.99]]], dtype=torch.float64)
    assert bound_hinge(inside, lower, upper, 100.0).item() == 0.0

    # Outside: the excess is a FRACTION of the limit range and the sum is a
    # MEAN, so the number is dimensionless and comparable with the collision
    # hinge. 0.5 * 100 * (0.5/2)^2 / 3 axes.
    outside = torch.tensor([[[1.5, 0.0, 0.0]]], dtype=torch.float64)
    expected = 0.5 * 100.0 * (0.5 / 2.0) ** 2 / DOF
    assert abs(bound_hinge(outside, lower, upper, 100.0).item() - expected) < 1e-12

    below = torch.tensor([[[-1.5, 0.0, 0.0]]], dtype=torch.float64)
    assert abs(bound_hinge(below, lower, upper, 100.0).item() - expected) < 1e-12


def test_the_hinge_engages_inside_the_limit_so_the_boundary_has_a_gradient():
    """A hinge that started exactly at the limit would have zero gradient
    there, and the optimizer would settle with the rate resting on the
    boundary -- over it as soon as the curve is sampled any finer."""
    cost = RateCost(_limits())
    assert bool((cost.velocity_upper < _limits().velocity).all())
    at_limit = torch.full((1, 4, DOF), 2.0, dtype=torch.float64)
    zero = torch.zeros_like(at_limit)
    duration = torch.ones(1, 1, 1, dtype=torch.float64)
    assert cost(zero, at_limit, zero, zero, duration).item() > 0.0


def test_retiming_halves_the_horizon_bias_rather_than_removing_it():
    """What `duration**n` actually does, asserted because the claim it is often
    given -- cuRobo's kernel says "invariant to dt choice" -- is not true.

    Fly the SAME path over `T`: acceleration goes as `1/T^2`, so an un-retimed
    squared sum goes as `T^-4`. The `T^2` factor leaves `T^-2`. Invariance
    would need `T^4`. Halving the exponent is the contract; a test that
    demanded invariance would fail against correct code, which is how this one
    was written the first time.
    """
    acceleration = torch.tensor([[[0.4, -0.2, 0.1]]], dtype=torch.float64)

    def retimed(horizon: float) -> float:
        # Same path, so acceleration falls as 1/T^2 relative to T = 1.
        scaled = acceleration / horizon ** 2
        return squared_regularization(
            scaled, W_REG_ACCELERATION * horizon ** 2).item()

    one, two, four = retimed(1.0), retimed(2.0), retimed(4.0)
    assert one > 0.0
    # T^-2: doubling the horizon quarters the term.
    assert abs(two / one - 0.25) < 1e-12
    assert abs(four / two - 0.25) < 1e-12
    # And NOT invariant, which is the misreading this pins down.
    assert two != one


def test_ratios_are_reported_against_the_stated_limit_not_the_inset_one():
    """The verifier asks about the cell's limit. A ratio measured against the
    shrunken bound the hinge uses would answer a friendlier question."""
    cost = RateCost(_limits())
    velocity = torch.full((1, 3, DOF), 2.0, dtype=torch.float64)
    zero = torch.zeros_like(velocity)
    v_ratio, _, _ = cost.worst_ratios(velocity, zero, zero)
    assert abs(v_ratio.item() - 1.0) < 1e-12
    assert ACTIVATION > 0.0


def test_every_rate_is_bounded_including_acceleration():
    """Acceleration used to have no bound term at all -- only a squared price
    -- and measured, the planner came back at 3.35x the limit."""
    cost = RateCost(_limits())
    zero = torch.zeros(1, 5, DOF, dtype=torch.float64)
    duration = torch.ones(1, 1, 1, dtype=torch.float64)
    baseline = cost(zero, zero, zero, zero, duration).item()
    assert baseline == 0.0
    for over in ("velocity", "acceleration", "jerk"):
        args = {"q": zero, "v": zero, "a": zero, "j": zero}
        args[{"velocity": "v", "acceleration": "a", "jerk": "j"}[over]] = (
            torch.full_like(zero, 100.0))
        assert cost(args["q"], args["v"], args["a"], args["j"],
                    duration).item() > 0.0, f"{over} is not bounded"
