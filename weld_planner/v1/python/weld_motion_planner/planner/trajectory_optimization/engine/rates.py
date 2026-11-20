#!/usr/bin/env python
"""What a free-space motion's rates cost: a bound hinge and a regularizer.

## Two terms, not one, and why

`docs/issues/0015` measured this planner's free-space output at 3.35x its own
acceleration limit while cuRobo sat at 0.99 on the same twelve problems, the
same robot and the same limits. The difference is not a clip -- cuRobo has none
either (`warp_bound_util.py: aggregate_bound_cost` prices the excess and
returns early inside the bounds). It is that cuRobo prices two DIFFERENT things
and this module used to price one:

- **A bound hinge.** Zero inside the limit, squared outside. Nothing is bought
  by being slow, so the optimizer spends the whole envelope; what it may not do
  is leave it.
- **An always-on squared regularizer.** Small, and about smoothness rather than
  legality: it is what keeps a curve from using the whole envelope in a
  sawtooth when a smooth ride costs the same in time.

The old term was neither. `W_VEL * (v/vmax)^2` is cheapest at zero, so it
priced MOVING -- it pushed the motion slow and still let acceleration through,
because acceleration had no bound term at all.

## The activation distance

The hinge engages a little INSIDE the limit -- `ACTIVATION` of the full range,
cuRobo's `activation_distance: 0.01`. A hinge that engaged exactly at the limit
would have zero gradient there, so the optimizer would converge with the rate
resting on the boundary and the first sampling refinement would find it over.
Engaging early gives the boundary a gradient to be pushed off.

## dt retiming, and what it actually buys

Every regularizer weight is scaled by `duration**n` -- `n = 1, 2, 3` for
velocity, acceleration, jerk. This is cuRobo's `retime_weights`.

**It does not make the cost horizon-invariant, and its kernel comment saying
so is approximate.** Fly the SAME path over `T`: velocity goes as `1/T`,
acceleration as `1/T^2`, jerk as `1/T^3`, so an un-retimed squared sum goes as
`T^-2n`. Multiplying by `T^n` leaves `T^-n`. Invariance would need `T^2n`.
Derived, and asserted below.

What it buys is that the remaining bias is HALF the exponent it was. That
matters because the horizon is chosen outside this term -- by `_horizon_for`
and then by `retime` -- and an un-retimed regularizer fights that choice hard
enough to win: at `T^-4` a motion is paid sixteen times over to double its
clock, and the answer comes back slow. `retime`'s own docstring already named
the symptom, "a seed's cost on a 3.8-second horizon and an answer's on a
1.4-second one are not the same quantity", and worked around it by never
comparing the two.

The bias is deliberately not removed altogether. Some pull toward a longer
horizon is what keeps the optimizer from riding the bound hinge everywhere;
the hinge says what is legal, this says what is comfortable, and `retime`
arbitrates.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

#: Fraction of the full limit range at which a hinge starts to pull. cuRobo's
#: `activation_distance` for every cspace term in `lbfgs_bspline_trajopt.yml`.
ACTIVATION = 0.01

#: Bound weights. Position dominates -- leaving the box is a different kind of
#: wrong from being brisk -- then velocity, then acceleration and jerk. The
#: ordering and the spread are cuRobo's `cost_cfg.cspace_cfg.weight`.
W_BOUND_POSITION = 10000.0
W_BOUND_VELOCITY = 10000.0
W_BOUND_ACCELERATION = 100.0
W_BOUND_JERK = 50.0

#: Always-on regularization. Acceleration is weighted an order above velocity:
#: the goal is a smooth ride, not a slow one. cuRobo's
#: `squared_l2_regularization_weight`.
W_REG_VELOCITY = 1000.0
W_REG_ACCELERATION = 10000.0
W_REG_JERK = 5.0

#: How far it GOES, against how it is flown. An L1 sum of joint travel in the
#: time metric, carried over from `freespace.W_TRAVEL`.
#:
#: L1 and not squared, for the reason a detour is a length and not an energy:
#: the gradient of a squared term grows with the excursion, so a path that has
#: to go round something would be pushed hardest exactly where it has least
#: room.
#:
#: **It does not on its own recover a detour -- measured twice.** Before the
#: straightening cycle existed, `bracket_b_2x` rewound came back 1.320 with
#: this term and 1.320 without, to five figures; after `_shortcut_cycle`, the
#: same corpus transit came back 1.2641 at 25 AND at 50, again to five
#: figures. A detour lives in which basin the path occupies, and no weight
#: here moves a path between basins -- the shortcut screen does. This weight
#: only prices length WITHIN a basin, and 25 does that job.
W_TRAVEL = 25.0

#: Collision, cuRobo's way: ONE weighted hinge, no multipliers.
#:
#: `lbfgs_bspline_trajopt.yml` prices the scene at 100000 and self-collision at
#: 10000, against cspace bound weights of 10000 -- so collision is the term
#: that wins, by construction, and nothing about it is iterative.
#:
#: The augmented Lagrangian this replaces (`collision/cost.py`) exists to drive
#: a hard constraint to satisfaction by raising multipliers between outer
#: steps. That machinery earns its keep when there is a constraint to satisfy
#: exactly; here there is no projection step and no feasibility guarantee to
#: preserve, so all it added was a second scale nobody had matched to the rate
#: terms -- measured on this cell, collision contributed 2.13e4 against the
#: rates' 1.17e8, a factor of 5500, and the optimizer ignored it.
W_COLLIDE_ENV = 100000.0
W_COLLIDE_SELF = 10000.0

#: Distance at which the collision hinge starts to pull, metres. cuRobo's
#: `scene_collision_cfg.activation_distance: 0.0025`, widened because this
#: planner's motions are metres long and a 2.5 mm skin gives the optimizer no
#: room to turn before it is already inside.
#:
#: Swept on the corpus, judged by the two contracts that pull against each
#: other -- the T2 verifier (no penetration) and the 1.2 detour bound:
#:
#:   0.0500  detour 1.264  certified      the skin is a corridor tax
#:   0.0150  detour 1.264  certified      smaller bow, same basin
#:   0.0100  detour 1.264  certified      still taxed into the bow
#:   0.0075  detour < 1.2  certified      <- both hold
#:   0.0050  detour < 1.2  certified
#:   0.0025  detour < 1.2  PENETRATES     cuRobo's value; 80 Adam steps get
#:                                        no gradient until too late
#:
#: 0.0075 keeps 3x the distance to the penetration cliff. The old solver's
#: augmented Lagrangian ran margin ZERO and survived because its multipliers
#: grew without bound; a fixed-weight hinge needs the skin, just not a wide
#: one.
COLLIDE_ACTIVATION_M = 0.0075


def collision_hinge(clearance: torch.Tensor, weight: float,
                    activation: float = COLLIDE_ACTIVATION_M) -> torch.Tensor:
    """`0.5 * w * (activation - clearance)^2` where closer than `activation`.

    Flat beyond it, so nothing is charged for room the motion already has --
    the same shape as the rate hinges, and the same reason.
    """
    if weight <= 0.0:
        return clearance.new_zeros(clearance.shape[0])
    # Also a fraction and a mean, matching the rate terms: `activation` is the
    # natural scale here, so a hinge fully engaged is 1.
    deficit = torch.relu(activation - clearance) / activation
    return 0.5 * weight * deficit.pow(2).mean(dim=1)


def shrink(lower: torch.Tensor, upper: torch.Tensor,
           activation: float = ACTIVATION) -> tuple[torch.Tensor, torch.Tensor]:
    """The bounds a hinge actually uses: pulled in by `activation` of the range."""
    inset = activation * (upper - lower)
    return lower + inset, upper - inset


def bound_hinge(value: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor,
                weight: float) -> torch.Tensor:
    """`0.5 * w * excess^2`, MEANED over samples and axes. Zero inside.

    One-sided on each side and flat between, so nothing is charged for using
    the envelope -- only for leaving it.

    **Meaned, not summed, and the excess is a FRACTION of the limit.** Both
    are about being comparable with the collision term, which is metres of
    clearance over a handful of spheres: a raw rad/s^2 excess summed over 161
    samples and 7 axes came out at 1e8 against collision's 1e4, and the
    optimizer spent every step on rates while the arm sat 40 mm inside the
    part. Dimensionless and averaged, the two terms are the same size and the
    weights below mean what they say.
    """
    if weight <= 0.0:
        return value.new_zeros(value.shape[0])
    span = torch.clamp(upper - lower, min=1e-9)
    excess = (torch.relu(value - upper) + torch.relu(lower - value)) / span
    return 0.5 * weight * excess.pow(2).mean(dim=(1, 2))


def squared_regularization(value: torch.Tensor, weight: torch.Tensor | float,
                           scale: torch.Tensor | None = None) -> torch.Tensor:
    """`0.5 * w * (v/scale)^2`, MEANED. `weight` may be `(B, 1, 1)` after retiming.

    Normalised by the axis's own limit and averaged, for the reason
    `bound_hinge` gives: a term that is not dimensionless cannot be weighed
    against one that is.
    """
    if isinstance(weight, float) and weight <= 0.0:
        return value.new_zeros(value.shape[0])
    if scale is not None:
        value = value / torch.clamp(scale, min=1e-9)
    return 0.5 * (weight * value.pow(2)).mean(dim=(1, 2))


@dataclass
class Limits:
    """Per-axis limits, `(dof,)` each, as the cell states them."""

    position_lower: torch.Tensor
    position_upper: torch.Tensor
    velocity: torch.Tensor
    acceleration: torch.Tensor
    jerk: torch.Tensor


class RateCost:
    """The rate half of the objective, for a batch of sampled trajectories.

    Collision is priced elsewhere (`collision/cost.py`, an augmented Lagrangian
    against fields) and deliberately stays there: this module is about what the
    ROBOT can do, that one is about where it may be.
    """

    def __init__(self, limits: Limits, *, activation: float = ACTIVATION):
        self.limits = limits
        self.position_lower, self.position_upper = shrink(
            limits.position_lower, limits.position_upper, activation)
        self.velocity_upper = limits.velocity * (1.0 - 2.0 * activation)
        self.acceleration_upper = limits.acceleration * (1.0 - 2.0 * activation)
        self.jerk_upper = limits.jerk * (1.0 - 2.0 * activation)

    def __call__(self, q: torch.Tensor, v: torch.Tensor, a: torch.Tensor,
                 j: torch.Tensor, duration: torch.Tensor) -> torch.Tensor:
        """`(B,)`. `duration` is `(B, 1, 1)` so a batch may hold many horizons.

        The retiming factor is the horizon, not the sample spacing: the samples
        are a fixed count over `duration`, so `dt` and `duration` differ by a
        constant and the constant is absorbed into the weights.
        """
        total = bound_hinge(q, self.position_lower, self.position_upper,
                            W_BOUND_POSITION)
        total = total + bound_hinge(v, -self.velocity_upper,
                                    self.velocity_upper, W_BOUND_VELOCITY)
        total = total + bound_hinge(a, -self.acceleration_upper,
                                    self.acceleration_upper,
                                    W_BOUND_ACCELERATION)
        total = total + bound_hinge(j, -self.jerk_upper, self.jerk_upper,
                                    W_BOUND_JERK)
        total = total + squared_regularization(
            v, W_REG_VELOCITY * duration, self.limits.velocity)
        total = total + squared_regularization(
            a, W_REG_ACCELERATION * duration ** 2, self.limits.acceleration)
        total = total + squared_regularization(
            j, W_REG_JERK * duration ** 3, self.limits.jerk)
        # Travel is a distance, so it is retimed once: `|v| * dt` is `|dq|`,
        # and summing that is the path's own length whatever clock it is on.
        total = total + W_TRAVEL * (
            (v.abs() / self.limits.velocity) * duration).mean(dim=(1, 2))
        return total

    def worst_ratios(self, v: torch.Tensor, a: torch.Tensor, j: torch.Tensor
                     ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """`(velocity, acceleration, jerk)` as fractions of the TRUE limits.

        Against the stated limits, not the shrunken ones the hinge uses: this
        is what the verifier will ask, and a number that flattered itself by
        measuring against an inset bound would answer a different question.
        """
        return (
            (v.abs() / self.limits.velocity).amax(dim=(1, 2)),
            (a.abs() / self.limits.acceleration).amax(dim=(1, 2)),
            (j.abs() / self.limits.jerk).amax(dim=(1, 2)),
        )
