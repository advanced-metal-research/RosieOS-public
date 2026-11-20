#!/usr/bin/env python
"""M6: the moves between the welds, so a plan stops being a set of islands.

M5 emits one certified trajectory per seam and nothing that reaches them. This
builds the rest: the arm leaves the cell's reset pose, arrives at the first
seam ALREADY MOVING at the speed that seam starts with, hops seam to seam, and
comes home. Same solver every time -- only the endpoints change.

What a caller has to know:

- **Timing is free here and nowhere else in this planner.** A weld's clock is
  its travel speed, a process parameter (`plan.Candidate`), so M5 may not
  redistribute it. A transit has no such contract: `duration_for` picks the
  first horizon and `retime` earns it back.
- **Endpoint VELOCITY is clamped, not just position.** A weld-side end is
  clamped to REST by HOLDING `PIN_KNOTS` knots there: the arc starts and stops
  on a still torch, so the joined trajectory deliberately breaks C1 at exactly
  those handovers and nowhere else.
- Knots are cubic Hermite `(q, q̇)`, so the T2 verifier certifies the CURVE
  with the same call M5 uses. The smoothness stack, the box and pairwise-rate
  clips and the monotone accept/reject loop are M5's.
- The collision term is M5's machinery with the margin SPLIT: the augmented
  Lagrangian on `clearance >= 0`, and the 50 mm standoff as a fixed-weight
  buffer beside it. See `collision_cost` and `buffers`.

Measured across the corpus, three plan requests at verification margin 0:
transit time 12.8 -> 5.6 s, 14.1 -> 8.5 and 19.9 -> 11.1, and motions free of
contact 4/4, 3/4, 2/4 -> 4/4 everywhere. `BENCHMARKS.md` 12.5 has the tables
and what each part bought alone.

`IMPLEMENTATION.md` M6 asks for L-BFGS. This is Adam, because the rate CLIP is
a projection and moves the iterate outside the objective's logic on any step
that saturates. L-BFGS stays a measurement to take.

**The trust-region ratchet fixed here (`TRUST_GROWTH`) is still in
`weld_trajopt.py` and is NOT touched.** M5's loop is the same one-way scale with
a seam projection inside it, and nobody has measured how that interacts with a
recovering step size. `docs/issues/0011`.

Run: `pixi run -e motion python -m weld_motion_planner.planner.trajectory_optimization.engine.freespace <request.weldplan>`.
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, replace
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
import torch

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.planner.trajectory_optimization.engine.roadmap import (
    SAMPLES as roadmap_samples,
)
from weld_motion_planner.planner.trajectory_optimization.engine.roadmap import Roadmap
from weld_motion_planner.planner.trajectory_optimization.engine.roadmap import (
    path as roadmap_path,
)
from weld_motion_planner.planner.trajectory_optimization.engine.weld_trajopt import (
    VERIFY_MARGIN_M,
    W_ACC,
    W_JERK,
    W_VEL,
    _forget_moments,
)
from weld_motion_planner.robot_cell.collision.cost import (
    DEFAULT_BUFFER_WEIGHT,
    CollisionCost,
)
from weld_motion_planner.robot_cell.robot import fk, ik
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.verifier.hermite_rates import (
    HermiteRateError,
    judge_hermite_rates,
)
from weld_motion_planner.verifier.hermite_wire import hermite_wire_degrees
from weld_motion_planner.verifier.verify import (
    FreeSpaceMotion,
    Verdict,
    verify_free_space,
)
from weld_motion_planner.verifier.verify import kinematics as verify_kinematics

# Knots roughly this far apart in time: fine enough that the Hermite resolves a
# swing the arm makes, coarse enough that a two-second transit is twenty
# variables and not two hundred.
KNOT_SPACING_S = 0.1
MIN_KNOTS = 5
MAX_KNOTS = 40

# Headroom on `duration_for`, whose formula is exact only for a rest-to-rest
# cubic. Keeps the first iterate inside the limits rather than relying on the
# clip to drag it there.
DURATION_SLACK = 1.4

INNER_STEPS = 80
OUTER_STEPS = 3
ADAM_LR = 5e-3

# Below this many inner steps a CUDA graph cannot pay for itself: the price of
# one is two real warm-up steps on a side stream plus the recording pass.
CAPTURE_MIN_STEPS = 16

# The standoff the collision term prefers, and the narrower one it settles for
# near a weld. 50 mm mid-flight gives descent a signal well before contact. At
# the ends it is unachievable -- a motion starts or finishes AT a weld pose, 3.9 mm
# off the work on this cell -- so the buffer there is the pose's own clearance
# and ramps to the full value over `BUFFER_RAMP_KNOTS`. Measured on
# `bracket_b_2x`'s approach, the knots inside 50 mm are the last five of thirty.
BUFFER_M = 0.05
BUFFER_SELF_M = 0.02
BUFFER_RAMP_KNOTS = 4

# The handover to a weld is a STOP, and the stop is STRUCTURAL: the last
# PIN_KNOTS knots of a weld-touching end hold one configuration, so the arc
# starts and stops on a still torch. The C1 handover to the weld's own clock
# is deliberately given up at exactly these joints.
#
# Pinned knots rather than a rate cap on the run-up, which is what this used
# to be. A cap is an upper bound on each knot-to-knot step, so it buys the
# crawl by spending the horizon's step BUDGET -- and `retime` shrinks the
# horizon without re-checking that budget, which left the re-paced seed short
# of its own endpoint. `_clamp_ends` then wrote the endpoint in anyway, and
# the gap became one segment: measured on a 1.2 rad approach, a 161x
# over-cap lunge landing exactly at the weld. The tighter the crawl, the
# sooner the budget ran out and the bigger that lunge -- asking for a slower
# contact made contact faster. A pin cannot do that: it spends no budget.
#
# Two knots is the minimum that makes the verifier's first difference zero.
# Three makes the SECOND difference zero too, so the acceleration it measures
# at the handover is zero rather than an artefact of where a ramp stopped.
#
# The run-up is left to `W_ACC` rather than a fixed-length ramp -- a squared
# penalty spreads a deceleration over as many knots as it needs, and a
# fixed-length ramp forced 1.9x the limit and the verifier failed the motion.
PIN_KNOTS = 3

# The share of an axis's own acceleration limit the braking envelope above a
# pinned band is allowed to use (`_move_caps`). Half, not all: the verifier
# judges the same finite differences the envelope is written in, and one AT
# the limit sits on the boundary it is judged against.
BRAKE_FRACTION = 0.5

# The augmented Lagrangian's starting penalty, not `cost.py`'s default 50. With
# no multiplier accumulated the AL term weighs `rho / 2`, so at 50 a penetration
# is priced at 25 against the buffer's 1000. Measured on `bracket_a_2x`'s first
# transit, the tight one, with everything else held:
#
#     rho        50   ->  -5.24 mm
#     rho     2 000   ->  -3.59 mm
#     rho    20 000   ->  -0.55 mm
#     rho    60 000   ->  -0.18 mm
#
# Below a millimetre is fitting noise: the self-collision backward uses atomics,
# so the same run twice differs by ~0.06 mm, and the ESDF's own worst-case error
# against the exact mesh is 1.6 mm.
COLLIDE_RHO = 60000.0

# How much a second of full-rate joint travel costs. THE UNITS ARE SECONDS:
# `sum |dq| / vmax` over the knots, the same ruler `detour_of` reports in.
#
# The smoothness stack prices smoothness and not distance, so a wide swing can
# beat a direct move with a corner in it. Measured on `bracket_b_2x`'s approach,
# every seed descended and straightened: the straight path came out at 560
# degrees of travel and cost 18.93, the `J1`-bowed one at 590 degrees and cost
# 14.71, and the bow won. 25 is on a plateau, not an edge: at 10 the bow still
# wins, at 25 and at 50 the answer is the same 560-degree path. A weld pass has
# no such term and must not -- its path is pinned by the seam.
W_TRAVEL = 25.0

# What an accepted step does to the per-row trust scale; it only ever halved on
# a rejection before. Measured: the scale falls from 0.25 to 0.016 at the first
# augmented-Lagrangian multiplier growth, then stays there for the remaining two
# thirds of the budget at a 100% accept rate, which is why raising penalty
# weights and adding cost terms both changed the answer by under 1%.
#
# Capped at 1.0: the scale multiplies Adam's own step, and caps of 8 and 64 drop
# the accept rate to 60% and end WORSE than the ratchet did.
TRUST_GROWTH = 1.3

# How many times to alternate straightening with descent. Steep then flat: on
# `bracket_b_2x`'s approach the detour goes 1.49, 1.12, 1.02, 1.00 and stops.
SHORTCUT_CYCLES = 3

# The shortcut pass, coarse to fine. Six levels covers `MAX_KNOTS`: the whole
# motion first, spans of two segments last -- below that there is nothing to
# straighten, since a single segment IS the straight line between its knots.
SHORTCUT_LEVELS = 6
# How finely a candidate straight line is screened: radians of the largest axis
# between samples, a step SIZE and not a count, because a span here may cross
# the whole cell while the roadmap's fixed 8 interior points only ever validate
# an edge between k-nearest neighbours. 3 degrees is a third of the roadmap's
# node margin at this arm's reach.
SHORTCUT_STEP_RAD = 0.05

# Solves the retime loop is allowed, counting refusals. An accepted shrink is
# followed by a smaller one and a refused shrink by a bisection, so three is the
# shortest budget in which either sequence gets a second chance.
RETIME_ATTEMPTS = 3
# Outer steps for a retime RE-SOLVE -- not the fresh solve's OUTER_STEPS. A
# retime starts from a trajectory that is already clear and already shaped, so
# it does not need the multiplier ladder a fresh solve climbs. Measured on
# `bracket_a_2x`, outer 3 against 1: every duration the same or SHORTER --
# two shrinks the pressured re-solve refused (3.15 s and 3.24 s motions) are
# accepted at 2.66 and 3.05 -- and the join drops 3.34 s -> 2.73 s. The
# accept test, not the outer count, is what guards clearance and rates.
RETIME_OUTER_STEPS = 1
# What fraction of the binding discrete rate budget a retimed motion may reach.
# Not 1.0: a motion aimed exactly at the limit lands over it as soon as the
# re-solve moves a knot. The final verifier separately judges exact Hermite
# extrema; this heuristic does not replace that gate.
RETIME_TARGET = 0.9
# Never shrink a horizon by more than this in one attempt. Handing a trajectory
# a third of its time at once means re-solving a problem that no longer
# resembles the one that was solved.
RETIME_FLOOR = 0.4
# How much screened clearance a retime may spend on going faster: almost none.
# Measured, `bracket_b_2x`'s approach: it retimed 2.94 s -> 2.65 s by shaving
# clearance from 3.9 mm to 0.1 mm, passed a "still above zero" accept test on
# the strength of that 0.1, and was then failed by the verifier. The 1 mm of
# give is for the screen's own wobble -- see `COLLIDE_RHO`.
RETIME_GIVE_M = 0.001

#: Which free-space solver `plan_moves` uses.
#:
#:   "bspline"  the control-point solver of `docs/issues/0015` -- a C2 B-spline
#:              whose rates are checked on the curve rather than on a knot grid
#:              the excess hides between. The default. Named for what the
#:              caller chooses -- the curve family -- rather than for the
#:              decision variable. (It was briefly "control"; the old
#:              spelling is REJECTED, deliberately -- an alias lets two names
#:              for one solver drift through configs and logs.)
#:   "curobo"   cuRobo's motion planner, on the same robot and the same
#:              obstacles (`curobo_backend`). For comparison: it brings its own
#:              optimizer (L-BFGS, many seeds, a graph planner) and its own
#:              world, so a disagreement says which half is responsible.
#:   "legacy"   DEPRECATED. The Catmull-Rom path this repository shipped: C1,
#:              certified on its knot grid, and measured at 3.35x its own
#:              acceleration limit between the knots. Kept runnable so old
#:              results can be reproduced; do not plan new work with it.
#:
#: `FREE_SPACE_BACKEND` may also be set from the environment, so a server can
#: be started on one without editing code.
FREE_SPACE_BACKEND = os.environ.get("WELD_PLANNER_FREE_SPACE_BACKEND", "bspline")

# How many ways to try the same move, at the cost of one batched tensor. The
# approach is why: the arm leaves home with its wrist at zero and must arrive
# with it flipped past 180 degrees, so the straight line in joint space sweeps
# the forearm through the table -- measured, 29.2 mm of penetration that a local
# optimizer started inside never lost.
SEEDS = 8
# The bump amplitude, in radians, spread across the seeds. Big enough to clear
# a fixture, small enough that the smoothness term can still pull it back.
SEED_SWING_RAD = 0.9

# How far back along the electrode to put the standoff waypoint, metres. Three
# rather than one because the right distance is a property of the fixture and
# nothing here knows it -- 80 mm clears a plate, 250 mm clears a jig -- and
# seeds are batched, so the choice is free. Measured on `bracket_b_2x seam_0001`:
# 3.9 mm of clearance at the weld itself, then 30.3 / 67.5 / 141.7 mm at these.
RETREATS_M = (0.08, 0.15, 0.25)

# Roadmap nodes a terminal may attach to, higher than `roadmap.ATTACH` because a
# weld pose sits millimetres off the work and most of its near neighbours fail
# edge validation: on `bracket_a_2x seam_0001`, 0 of the nearest 24 and 24 of
# the nearest 256. It also shortens the routes found -- `hitch_2x`'s approach
# goes from 8 waypoints to 5.
REWIND_ATTACH = 256


class FreeSpaceError(RuntimeError):
    """A connecting motion this refuses to plan, with the reason."""


@dataclass
class Move:
    """One free-space move, in the same shape a seam's trajectory arrives in."""

    kind: FreeSpaceMotion
    from_id: str | None
    to_id: str | None
    knots: np.ndarray              # (L, dof) radians: arm then positioner
    knot_velocity: np.ndarray      # (L, dof) rad/s -- Hermite data
    times_s: np.ndarray
    # The worst gap anywhere on the sampled path. `clearance_m` is the worse of
    # the two, which are also kept apart: "the torch is in the plate" and "the
    # forearm is in the wrist" are different failures with different fixes.
    clearance_m: float
    env_clearance_m: float = float("nan")
    self_clearance_m: float = float("nan")
    verdict: Verdict | None = None
    # Smoothness in and out, BOTH on the horizon this motion ended up with. Against
    # different horizons the two are not the same quantity -- see `retime`.
    cost_before: float = 0.0
    cost_after: float = 0.0
    # Which of the batched seeds won: a motion that only works from the standoff
    # seed depends on something worth knowing about.
    seed: str = ""
    # Joint travel in the time metric over the straight-line distance between the
    # clamped endpoints. 1.0 is the geodesic and cannot be beaten; 1.5 means half
    # the move is detour, and 3.0 is a motion to look at rather than trust.
    detour: float = 1.0
    # What the horizon was before `retime` shrank it.
    duration_before_s: float = 0.0
    # The held axes, so a serialisation of this motion is playable without the
    # profile that planned it.
    held: dict[str, float] = field(default_factory=dict)
    # The axis order `knots` is in. Carried rather than assumed: a reader that
    # guessed it would play the move on the wrong joints.
    names: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def duration_s(self) -> float:
        return float(self.times_s[-1] - self.times_s[0])


@dataclass
class MoveSpec:
    """One connecting motion's endpoints and seeds, before it joins the batch.

    What `connect` knows about a motion before anything is solved: where it
    starts and ends (position AND velocity -- the endpoint rates are the
    module's contract), the standoff waypoints to seed through, and the
    roadmap route if one was found. `MoveSolver` folds a list of these into
    one tensor problem, the same way M5's `Synthesizer` folds seams.
    """

    kind: FreeSpaceMotion
    from_id: str | None
    to_id: str | None
    start: np.ndarray
    start_rate: np.ndarray
    end: np.ndarray
    end_rate: np.ndarray
    via: list[np.ndarray] = field(default_factory=list)
    route: np.ndarray | None = None
    # Which ends touch a weld: that end holds one configuration over its last
    # (or first) PIN_KNOTS knots, so it arrives and leaves stopped. The caller
    # also passes REST as that end's rate, which the pin then makes true
    # rather than merely declared.
    pin_start: bool = False
    pin_end: bool = False

    def pinned(self) -> tuple[int, int]:
        """`(start, end)` knots held at the clamped configuration, this row.

        One is the plain clamp every motion has -- the endpoint is data. A
        weld-touching end holds `PIN_KNOTS` of them.
        """
        return (PIN_KNOTS if self.pin_start else 1,
                PIN_KNOTS if self.pin_end else 1)


def _step_caps(count: int, duration: float, vmax: np.ndarray) -> np.ndarray:
    """`(count - 1, dof)` largest legal knot-to-knot step, radians.

    The rate limit and nothing else -- `_move_caps` adds what a pinned band
    needs on top.
    """
    dt_row = duration / max(count - 1, 1)
    return np.tile(vmax * dt_row, (max(count - 1, 1), 1))


def _move_caps(spec: MoveSpec, count: int, duration: float,
               vmax: np.ndarray, amax: np.ndarray) -> np.ndarray:
    """`_step_caps` with the pinned bands zeroed and a braking envelope above.

    A pinned segment may not move at all, so its budget is nothing. That is
    what makes `_repaced` lay the path out inside the MOVING span and park on
    the endpoint through the pin, instead of walking to the last knot and
    having `_clamp_ends` tear the tail back.

    Above the band the cap is what it takes to STOP there: `k` segments from
    the pin, an axis may move no faster than it can shed in `k` steps at
    `BRAKE_FRACTION` of its own acceleration limit. Half, not all, of `amax`:
    the verifier judges the same finite differences, and an envelope AT the
    limit sits on the boundary it is judged against.

    This is not the old crawl. That capped the run-up at an ABSOLUTE
    `0.05 rad/s` over a fixed count of segments -- a number unrelated to
    stopping, which spent budget the horizon then did not have for the path
    and put a 161x lunge at the weld. An envelope spends only what physics
    already requires: a motion that cannot afford it could not have stopped.
    Priced-only deceleration was not enough on its own -- measured, the
    approach came back at 1.11 of the acceleration limit and the verifier
    failed it.
    """
    caps = _step_caps(count, duration, vmax)
    pin_start, pin_end = spec.pinned()
    n = len(caps)
    dt_row = duration / max(count - 1, 1)
    brake = (BRAKE_FRACTION * np.asarray(amax, dtype=np.float64)) * dt_row * dt_row
    head, tail = pin_start - 1, pin_end - 1
    if tail > 0:
        stop = max(n - tail, 0)                  # first pinned segment
        caps[stop:] = 0.0
        away = (stop - np.arange(stop))          # segments left to shed speed in
        caps[:stop] = np.minimum(caps[:stop], away[:, None] * brake[None, :])
    if head > 0:
        start = min(head, n)                     # first moving segment
        caps[:start] = 0.0
        away = (np.arange(start, n) - start + 1)
        caps[start:] = np.minimum(caps[start:], away[:, None] * brake[None, :])
    return caps


def _carries(spec: MoveSpec, count: int, duration: float, vmax: np.ndarray,
             amax: np.ndarray, need_s: float) -> bool:
    """Can this horizon's caps walk a path of `need_s` seconds at full rate?

    The budget is summed in the roadmap's own time metric -- the binding axis
    of each segment -- which is the metric `need_s` is measured in.
    """
    scale = np.maximum(vmax, 1e-9)
    dt_row = duration / max(count - 1, 1)
    caps = _move_caps(spec, count, duration, vmax, amax)
    usable_s = float((caps / (scale * dt_row)).min(axis=1).sum()) * dt_row
    return usable_s >= need_s + 1e-9


def _horizon_for(spec: MoveSpec, vmax: np.ndarray,
                 amax: np.ndarray) -> tuple[float, int]:
    """`(duration, knots)` whose caps can actually carry the motion.

    `duration_for` prices the STRAIGHT line at full rate; a motion that pins
    knots at its ends, or follows a roadmap route round an obstacle, needs
    more than that -- measured, a 2.58 s straight-line horizon left a 1.25 s
    route 50 mm short of its endpoint. So the horizon is grown until the
    summed per-segment caps cover the longest path this motion might be
    seeded with, axis by axis, with 20% to spare. `retime` earns the surplus
    back afterwards -- down to `_carry_floor`, which is this same budget.

    The pinned segments carry no travel and the braking envelope above them
    carries less than full rate, so neither is whole budget -- `_carries`
    sums what the caps actually allow rather than assuming `vmax` throughout.
    """
    scale = np.maximum(vmax, 1e-9)
    paths = [np.stack([spec.start, spec.end])]
    for via in spec.via:
        paths.append(np.stack([spec.start, via, spec.end]))
    if spec.route is not None and len(spec.route) >= 2:
        paths.append(np.asarray(spec.route, dtype=np.float64))
    # Seconds at full rate, the binding axis of each leg -- the same time
    # metric the roadmap searches in. Per-axis radian sums undercount: a path
    # that moves one axis and then another wastes every segment's budget on
    # the axes standing still.
    need_s = max(float(np.abs(np.diff(p, axis=0) / scale).max(axis=1).sum())
                 for p in paths)
    duration = duration_for(np.asarray(spec.end) - np.asarray(spec.start),
                            vmax, amax)
    count = MIN_KNOTS
    for _ in range(24):
        count = int(np.clip(round(duration / KNOT_SPACING_S) + 1,
                            MIN_KNOTS, MAX_KNOTS))
        if _carries(spec, count, duration, vmax, amax, need_s * 1.2):
            break
        duration *= 1.25
    return float(duration), count


def _carry_floor(spec: MoveSpec, knots: np.ndarray, dt: float,
                 vmax: np.ndarray, amax: np.ndarray) -> float:
    """The smallest horizon multiplier whose caps still carry `knots`.

    `retime` shrinks a horizon by `k` and keeps the knot COUNT, so every
    segment's budget shrinks with it while the path it has to walk does not.
    Below this ratio the re-paced seed cannot reach its own endpoint,
    `_clamp_ends` writes the endpoint in regardless, and the shortfall
    becomes one segment at the weld -- the lunge `PIN_KNOTS` describes.

    Without this the demanded scale was always under the floor for a motion
    that stops at its ends, so the first attempt of every retime started from
    a broken seed and was refused on a solve of it.

    BISECTED rather than divided: the rate part of a cap scales with `dt` but
    the braking envelope scales with `dt^2`, so the budget is not linear in
    the multiplier and a ratio would sit under the true floor exactly where
    the envelope binds. Budget rises with the horizon, so bisection is sound.
    """
    knots = np.asarray(knots, dtype=np.float64)
    count = len(knots)
    if count < 2 or dt <= 0.0:
        return 1.0
    scale = np.maximum(vmax, 1e-9)
    need_s = float(np.abs(np.diff(knots, axis=0) / scale).max(axis=1).sum())
    duration = dt * (count - 1)
    if _carries(spec, count, duration * RETIME_FLOOR, vmax, amax, need_s):
        return RETIME_FLOOR
    if not _carries(spec, count, duration, vmax, amax, need_s):
        return 1.0
    lo, hi = RETIME_FLOOR, 1.0
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if _carries(spec, count, duration * mid, vmax, amax, need_s):
            hi = mid
        else:
            lo = mid
    return float(hi)


def _repaced(path: np.ndarray, caps_rad: np.ndarray) -> np.ndarray:
    """The same polyline, walked at the pace `caps_rad` allows.

    Knot `i` stands where a walker from knot `i - 1` runs out of that
    segment's per-axis budget. Seeds are laid out with no regard for the
    caps, and the clip cannot repair that -- it only pulls a knot toward its
    anchor, which drags the shape OFF the seed's own path: measured on
    `bracket_a_2x`'s 0006 -> 0001 transit, a route seed with +3.7 mm of
    clearance left the projection at -22.6 mm. Walking the same path at the
    legal pace keeps the geometry and spends the horizon.

    If the budget runs out before the path does, the remaining knots hold the
    farthest point reached; the end clamp and `cap_excess_rad` make that
    visible rather than hidden. `_horizon_for` sizes the horizon so it cannot
    happen on a first solve and `_carry_floor` bounds `retime` so it cannot
    happen on a later one -- both are that guarantee, not this walker's.
    """
    n = len(path)
    out = np.empty_like(path, dtype=np.float64)
    out[0] = path[0]
    seg, p = 0, path[0].astype(np.float64).copy()
    for i in range(1, n):
        budget = caps_rad[i - 1].astype(np.float64).copy()
        while seg < n - 1:
            delta = path[seg + 1] - p
            need = np.abs(delta)
            if need.max() <= 1e-12:
                seg += 1
                continue
            share = float(np.min(np.where(need > 1e-12,
                                          budget / np.maximum(need, 1e-12),
                                          np.inf)))
            if share >= 1.0:
                budget = budget - need
                p = path[seg + 1].astype(np.float64).copy()
                seg += 1
                continue
            p = p + delta * share
            break
        out[i] = p
    return out


@dataclass
class _Solved:
    """One motion's winner, out of the batch: what `retime` and `Move` need."""

    knots: np.ndarray              # (n, dof), the pad already trimmed
    qdot: np.ndarray               # (n, dof)
    times_s: np.ndarray            # (n,)
    cost_before: float
    cost_after: float
    clearance_m: float
    env_clearance_m: float
    self_clearance_m: float
    seed: str
    # Worst knot-to-knot step over its cap, radians; positive means the clip
    # could not satisfy the caps -- a horizon too short to carry the path.
    # `_carry_floor` is what keeps it at zero; `retime` refuses a shrink that
    # leaves it positive, which is now a backstop and not the usual case.
    cap_excess_rad: float = 0.0


def duration_for(delta: np.ndarray, vmax: np.ndarray, amax: np.ndarray) -> float:
    """How long a move of `delta` needs, from the rate limits alone.

    For a cubic from rest to rest over `d` in time `T` the extremes are
    `1.5 d / T` and `6 d / T^2`, so `T >= 1.5|d|/vmax` and
    `T >= sqrt(6|d|/amax)`, per axis. The binding axis wins, and the result is
    padded by `DURATION_SLACK`.
    """
    delta = np.abs(np.asarray(delta, dtype=np.float64))
    by_velocity = float((1.5 * delta / np.maximum(vmax, 1e-9)).max())
    by_acceleration = float(np.sqrt(6.0 * delta / np.maximum(amax, 1e-9)).max())
    return max(by_velocity, by_acceleration, 1e-3) * DURATION_SLACK


def rate_ratios(knots: np.ndarray, dt: float, vmax: np.ndarray,
                amax: np.ndarray) -> tuple[float, float]:
    """Discrete optimizer rate ratios, used only to guide retiming.

    These are first and second waypoint differences, not extrema of the emitted
    Hermite curve. The exact downstream verifier owns the latter. Above 1.0
    means over this optimizer budget; unlike `time_scale`, there is no clamp.
    """
    knots = np.asarray(knots, dtype=np.float64)
    if len(knots) < 2:
        return 0.0, 0.0
    velocity = np.diff(knots, axis=0) / dt
    v_ratio = float(np.abs(velocity / np.maximum(vmax, 1e-9)).max())
    if len(knots) < 3:
        return v_ratio, 0.0
    acceleration = np.diff(velocity, axis=0) / dt
    return v_ratio, float(np.abs(acceleration / np.maximum(amax, 1e-9)).max())


def time_scale(knots: np.ndarray, dt: float, vmax: np.ndarray, amax: np.ndarray, *,
               target: float = RETIME_TARGET) -> float:
    """What to multiply a horizon by so the binding axis reaches `target`.

    On a horizon `k` times shorter every velocity divides by `k` and every
    acceleration by `k^2`, so an axis at `v` of its limit demands
    `k >= v / target` and one at `a` demands `k >= sqrt(a / target)`. The larger
    demand wins, capped at 1.0: this only ever makes a motion faster.
    """
    v_ratio, a_ratio = rate_ratios(knots, dt, vmax, amax)
    demand = max(v_ratio / max(target, 1e-9),
                 float(np.sqrt(a_ratio / max(target, 1e-9))))
    return float(min(1.0, max(demand, RETIME_FLOOR)))


def detour_of(knots: np.ndarray, vmax: np.ndarray) -> float:
    """Joint travel divided by the straight-line distance, in the time metric.

    The metric is the roadmap's -- seconds at full rate, the binding axis of each
    step -- and not radians: a radian of the base is a slower move than a radian
    of the wrist. 1.0 is the geodesic. REPORTED and never a constraint: a motion
    that has to go round something has a detour above 1 and is correct.
    """
    knots = np.asarray(knots, dtype=np.float64)
    if len(knots) < 2:
        return 1.0
    scale = np.maximum(vmax, 1e-9)
    travelled = float(np.abs(np.diff(knots, axis=0) / scale).max(axis=1).sum())
    direct = float(np.abs((knots[-1] - knots[0]) / scale).max())
    return travelled / direct if direct > 1e-12 else 1.0


def _hermite_batch(q0, q1, d0, d1, h, u):
    """(..., dof) cubic Hermite at parameter `u`, torch or numpy alike."""
    u2, u3 = u * u, u * u * u
    return ((2 * u3 - 3 * u2 + 1) * q0 + h * (u3 - 2 * u2 + u) * d0
            + (-2 * u3 + 3 * u2) * q1 + h * (u3 - u2) * d1)


# --------------------------------------------------------------------------
# Winding


def urdf_position_limits(request: Any, names: list[str]) -> tuple[np.ndarray, np.ndarray] | None:
    """The bounds stated by the URDF the cell descriptor names, or `None`.

    `None` when the file cannot be found or does not cover every axis -- a
    partial answer would be worse than the descriptor's, because the caller
    would not know which half it had.
    """
    import xml.etree.ElementTree as ET

    source = ((request.cell.get("source") or {}).get("filename") or "").strip()
    cell_id = str(request.cell.get("id") or "")
    if not source or not cell_id:
        return None
    root = Path(__file__).resolve().parents[7] / "robot_description" / "robots"
    found = [c for c in sorted(root.rglob(source)) if c.parent.name == cell_id]
    if not found:
        return None
    bounds = {}
    for joint in ET.parse(found[0]).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None:
            bounds[joint.get("name")] = (float(limit.get("lower")),
                                         float(limit.get("upper")))
    if any(name not in bounds for name in names):
        return None
    return (np.array([bounds[n][0] for n in names]),
            np.array([bounds[n][1] for n in names]))


def real_position_limits(request: Any, names: list[str]
                         ) -> tuple[np.ndarray, np.ndarray]:
    """The TIGHTER of the cell descriptor's bounds and the URDF's.

    `docs/issues/0014`: the corpus bundles carry a cell captured before the arm
    was given its real bounds, so they state `J4 -6.3..6.3` where the mechanism
    has `-pi..pi` and `J5 -6.3..6.3` where it has `-3.37..1.3`. A plan made
    against the bundle emits joints the arm refuses -- 2,356 `joint_limit`
    rejections from the Tesseract checker on the OLP export gate, and one J5
    knot with no representative inside the real bounds at all.

    Taking the tighter of the two is the conservative reading and needs no
    re-bundle: where the descriptor is already right it changes nothing, and
    where it is stale the URDF wins. The bundles should still be refreshed
    (`tools/refresh_corpus_cells.py`); this stops the planner designing an arm
    that does not exist in the meantime.
    """
    lower, upper = stated_position_limits(request.cell, names)
    from_urdf = urdf_position_limits(request, names)
    if from_urdf is None:
        return lower, upper
    return np.maximum(lower, from_urdf[0]), np.minimum(upper, from_urdf[1])


def stated_position_limits(cell_doc: dict[str, Any],
                           names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    """`(lower, upper)` radians per axis of `names`, from the cell document.

    A bound the document does not state is infinite, so a clamp against it is
    a no-op. `ik.limits_of` is the strict reader that refuses instead.
    """
    limits = {str(joint.get("name")): (joint.get("limits_rad") or {})
              for joint in cell_doc.get("joints") or []}
    lower = np.array([float((limits.get(n) or {}).get("lower", -np.inf))
                      for n in names])
    upper = np.array([float((limits.get(n) or {}).get("upper", np.inf))
                      for n in names])
    return lower, upper


def _windings(request: PlanRequest, knots: np.ndarray, here: np.ndarray, *,
              names: list[str] | None = None,
              cell: Any = None) -> list[np.ndarray]:
    """Whole-turn offsets this seam could be commanded on, nearest first.

    The nearest choice per axis, then the single-axis deviations from it. NOT the
    full product: seven axes with three turns each is 2187 windings of the same
    weld, and both cases in `docs/issues/0009` differ from the nearest on one
    wrist axis. A winding that needs two axes moved at once is not offered.
    """
    if names is None:
        names = (cell or RobotCell.of(request.cell)).moved_axes
    turn = 2.0 * np.pi
    lower, upper = stated_position_limits(request.cell, list(names))
    legal, cost = [], []
    for i, (lo, hi) in enumerate(zip(lower, upper, strict=True)):
        column = knots[:, i]
        # A winding is chosen at a HANDOVER: `here` must reach the seam's
        # first command. Using the whole seam's midpoint can cross pi on an
        # unrelated wrist axis and make a needless full turn the base choice;
        # because alternatives vary one axis from that base, the actually
        # reachable single-axis winding then disappears from the candidate
        # set. Whole-seam min/max still decides legality below.
        entry = column[0]
        # Over the WHOLE seam: a winding that clips at one knot is not
        # available at any of them.
        options = [k for k in (-1, 0, 1)
                   if lo <= column.min() + k * turn and column.max() + k * turn <= hi]
        options.sort(key=lambda k: abs(entry + k * turn - here[i]))
        legal.append(options or [0])
        cost.append({k: abs(entry + k * turn - here[i]) for k in (options or [0])})

    base = [options[0] for options in legal]
    out = [np.array(base, dtype=np.float64) * turn]
    extra = []
    for i, options in enumerate(legal):
        for k in options[1:]:
            alt = list(base)
            alt[i] = k
            extra.append((cost[i][k] - cost[i][base[i]],
                          np.array(alt, dtype=np.float64) * turn))
    out.extend(offset for _penalty, offset in sorted(extra, key=lambda p: p[0]))
    return out


def rewound(request: PlanRequest, reports: list[Any], start: np.ndarray, *,
            names: list[str] | None = None, roadmap: Roadmap | None = None,
            planner_collision_model=None, device=None,
            held: dict[str, float] | None = None,
            attach: int = REWIND_ATTACH, cell: Any = None) -> list[Any]:
    """The same welds, commanded by the short way round, walking the plan in
    order from `start`.

    A revolute axis at `t` and at `t - 2 pi` is the same cell: same forward
    kinematics, same spheres, same clearances -- measured on `bracket_b_2x`,
    0.18 microns of difference across three seams -- and the same weld. Only the
    number the controller is handed differs, and with it how far the arm travels.

    **The reports handed back REPLACE the originals.** They carry different joint
    numbers for the same poses, so serialising the pre-rewind ones next to these
    motions puts a 360-degree jump at every handover.

    M4 picks a winding per seam (`Graph.unwound`) with no view of the plan the
    seam will sit in, so the arm can be told to reach `J5 = +199.5` while it
    stands at `J5 = 0` and `-160.5` is the identical pose. On this cell that is
    worse than a 360-degree detour: the roadmap finds NO clear configuration with
    `J5` between 44 and 151 degrees, at any sampling density, so the approach to
    `+199.5` does not exist. Rewound, every motion in the plan routes.

    **Nearest is not the same as reachable, so where a roadmap is supplied this
    asks it.** `bracket_a_2x seam_0001`'s `J5 = +165` is nearer to home than
    `-195` and has NO route to it, while the other turn routes in five waypoints.
    Candidates from `_windings` are tried nearest first and the first one the
    roadmap reaches wins; with no roadmap the nearest is taken.

    Velocities are untouched: a constant offset has no derivative.
    """
    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    if names is None:
        names = cell.moved_axes
    here = np.asarray(start, dtype=np.float64).copy()
    out = []
    for report in reports:
        knots = np.asarray(report.knots, dtype=np.float64)
        candidates = _windings(request, knots, here, names=names, cell=cell)
        offset = candidates[0]
        if roadmap is not None and planner_collision_model is not None:
            for candidate in candidates:
                if roadmap_path(roadmap, planner_collision_model, device, here,
                                knots[0] + candidate, held=held or {},
                                attach=attach) is not None:
                    offset = candidate
                    break
        moved = replace(report, knots=knots + offset)
        out.append(moved)
        here = np.asarray(moved.knots[-1], dtype=np.float64)
    return out


def order_seams(reports: list[Any], start: np.ndarray, *,
                vmax: np.ndarray, amax: np.ndarray) -> list[Any]:
    """The same welds, visited in the order that keeps the transits short.

    `connect` joins the reports in the order it is handed, and until now that
    was the order the REQUEST wrote them in -- which on a real part sends the
    arm across the cell to a far seam while one sits beside it. The cost of
    an order is the closed-form `duration_for` between one weld's last knot
    and the next's first, taken MODULO whole turns per axis because `rewound`
    commands the short way round afterwards; the tour is open -- reset pose
    in, reset pose out, both legs priced.

    Nearest-neighbour from the reset pose, then single-seam relocation passes
    until no move helps. Deterministic, and exact would be overkill: the
    surrogate itself is exact only for a rest-to-rest cubic along the
    straight line, so polishing its last percent buys nothing real.

    ORDERING ONLY: each weld keeps its own direction, knots and clock. A weld
    that would be cheaper run BACKWARDS is a different trajectory and a
    different question, and it is not asked here.
    """
    if len(reports) < 3:
        return list(reports)
    turn = 2.0 * np.pi
    home = np.asarray(start, dtype=np.float64)
    starts = np.stack([np.asarray(r.knots[0], dtype=np.float64) for r in reports])
    ends = np.stack([np.asarray(r.knots[-1], dtype=np.float64) for r in reports])

    def cost(a_end: np.ndarray, b_start: np.ndarray) -> float:
        return duration_for((b_start - a_end + np.pi) % turn - np.pi, vmax, amax)

    n = len(reports)
    remaining = set(range(n))
    order: list[int] = []
    here = home
    while remaining:
        nearest = min(remaining, key=lambda j: cost(here, starts[j]))
        order.append(nearest)
        remaining.remove(nearest)
        here = ends[nearest]

    def total(sequence: list[int]) -> float:
        legs = cost(home, starts[sequence[0]])
        for a, b in pairwise(sequence):
            legs += cost(ends[a], starts[b])
        return legs + cost(ends[sequence[-1]], home)

    score = total(order)
    improved = True
    while improved:
        improved = False
        for i in range(n):
            for at in range(n):
                if at == i:
                    continue
                candidate = order[:i] + order[i + 1:]
                candidate.insert(at, order[i])
                value = total(candidate)
                if value + 1e-9 < score:
                    order, score, improved = candidate, value, True
    return [reports[i] for i in order]


def winding_reach(request: PlanRequest, reports: list[Any], start: np.ndarray, *,
                  names: list[str] | None = None,
                  cell: Any = None) -> np.ndarray:
    """Every configuration any candidate winding could put the arm at.

    The roadmap's sampling box is cut to what is being joined, so it has to be
    told about the windings not yet chosen -- otherwise the box is drawn around
    the nearest turn and the alternative `rewound` exists to find is outside it.
    Costs a wider box and nothing else.
    """
    cell = cell or RobotCell.of(request.cell)
    if names is None:
        names = cell.moved_axes
    here = np.asarray(start, dtype=np.float64).copy()
    reach = [here.copy()]
    for report in reports:
        knots = np.asarray(report.knots, dtype=np.float64)
        for offset in _windings(request, knots, here, names=names, cell=cell):
            reach.append(knots[0] + offset)
            reach.append(knots[-1] + offset)
        here = knots[-1] + _windings(request, knots, here, names=names,
                                     cell=cell)[0]
    return np.vstack(reach)


# --------------------------------------------------------------------------
# The standoff waypoint


def standoff_configs(request: PlanRequest, q: np.ndarray, device, *,
                     retreats_m=RETREATS_M, names: list[str] | None = None,
                     cell: Any = None) -> list[np.ndarray]:
    """The weld pose, backed straight off along the electrode, as joint angles.

    A torch approaches its weld ALONG its own axis, from above, and leaves the
    same way. The straight line from the reset pose arrives from whatever side
    the joint-space interpolation sweeps through, and on `bracket_b_2x` that side
    is through the plate -- twenty-four clean knots and a dive in the last seven
    (`docs/issues/0009`). Descent cannot change which side a path comes from; a
    seed can start on the right one.

    `tool0`'s rotation carries the tool asset's `electrode_axis` onto `-u`, so
    `+u`, out of the work, is `-(R @ axis)`. The tool ORIGIN moves along it, the
    rotation is kept exactly, and IK is SEEDED AT THE WELD CONFIGURATION ITSELF
    -- so the answer is a few degrees away in the same branch, no wrist flips
    between standoff and weld, and the plunge is a straight retreat rather than a
    re-posture. Measured, `seam_0001`: 15.5 degrees of largest joint travel for
    100 mm of retreat, position error 1 micron.

    The positioner is NOT moved: backing the gun off a stationary part is the
    move.

    Returns one configuration per entry of `retreats_m`, SKIPPING any the IK did
    not reach -- so the list may be shorter than `retreats_m`, or empty.
    """
    return standoff_batch(request, [np.asarray(q, dtype=np.float64)], device,
                          retreats_m=retreats_m, names=names, cell=cell)[0]


def standoff_batch(request: PlanRequest, ends: list[np.ndarray], device, *,
                   retreats_m=RETREATS_M, names: list[str] | None = None,
                   cell: Any = None) -> list[list[np.ndarray]]:
    """`standoff_configs` for MANY weld poses, in ONE iterative-IK call.

    Every (pose, retreat) pair is one row of a single batched solve, because
    the solver's cost is per CALL and not per row -- twenty poses solved one
    retreat at a time were 2.3 s of a 4.6 s join, and one batch of sixty rows
    is one solve. The answers, the seeding, and the skip-on-non-convergence
    are `standoff_configs`' exactly.
    """
    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    if names is None:
        names = cell.moved_axes
    if not ends:
        return []
    tool = str((cell.descriptor.get("frames") or {}).get("tool") or "")
    index = cell.arm.chain.link_names.index(tool)
    order = [list(names).index(n) for n in cell.arm.chain.axis_names]
    stacked = np.stack([np.asarray(q, dtype=np.float64) for q in ends])
    seeds = torch.as_tensor(stacked[:, order], dtype=torch.float64, device=device)
    rot, pos = fk.torch_link_transform(cell.arm.chain, index, seeds)
    axis = torch.as_tensor(np.asarray(cell.arm.tool.electrode_axis),
                           dtype=torch.float64, device=device)
    away = -(rot @ axis)                       # `+u`, out of the work
    # float64 for the same reason the seed is: the limits clamp the solve, and
    # float32 bounds would round the box this branch is searched inside. Read
    # from the descriptor -- casting the cell's float32 tensors up keeps the
    # float32 rounding.
    lower, upper = ik.limits_of(cell.descriptor, cell.arm.chain, device,
                                torch.float64)

    reach = torch.as_tensor(list(retreats_m), dtype=torch.float64,
                            device=device).repeat(len(ends)).view(-1, 1)
    rows = len(retreats_m)
    solution = ik.solve(cell.arm.chain, index,
                        rot.repeat_interleave(rows, 0),
                        pos.repeat_interleave(rows, 0)
                        + reach * away.repeat_interleave(rows, 0),
                        seeds.repeat_interleave(rows, 0).clone(),
                        lower=lower, upper=upper)
    converged = solution.converged.cpu().numpy()
    answers = solution.q.cpu().numpy()

    out: list[list[np.ndarray]] = []
    for i, q in enumerate(stacked):
        vias = []
        for r in range(rows):
            if not bool(converged[i * rows + r]):
                continue
            via = q.copy()
            via[order] = answers[i * rows + r]
            vias.append(via)
        out.append(vias)
    return out


class MoveSolver:
    """Every connecting motion of a plan, solved as ONE tensor problem.

    A row is a (motion, seed) pair -- M5's `Synthesizer` row model with a
    motion where it has a seam. Rows share the cell, its limits and the
    collision model; everything a motion owns (its endpoints, its horizon,
    its clock, its seed pool) rides along per row. Endpoints clamped,
    interiors free -- unchanged from when this solved one motion at a time.

    Why one problem: the loop is launch-bound (`BENCHMARKS.md` 11.3), so
    eleven motions solved separately cost eleven times the launches for the
    same arithmetic. Batched, the whole plan's connecting motions cost what
    one used to -- the exact move 11.3a measured on M5's seams.

    **Rows are padded to the longest motion and the pad is inert.** Short
    rows repeat their clamped end so the pad holds still; every sum is
    masked; the difference operators and the end clamps know each row's own
    last index rather than assuming `-1`. A pad that leaked into the
    collision term would be charged for extra copies of a near-weld stance,
    and one that leaked into the acceleration term would price a stop that
    is not in the motion -- both hazards 11.3a already paid for once.
    """

    def __init__(self, request: PlanRequest, planner_collision_model, device, *,
                 moves: list[MoveSpec], held: dict[str, float],
                 seeds_count: int = SEEDS,
                 durations: list[float] | None = None,
                 seed_trajs: list[np.ndarray] | None = None,
                 cell: Any = None):
        if not moves:
            raise FreeSpaceError("nothing to join")
        self.request = request
        self.planner_collision_model = planner_collision_model
        self.device = device
        self.held = held
        self.moves = moves
        self.M = len(moves)

        # Assembled once by the caller. Building the profile and the axis order
        # here is how two stages came to disagree about the same cell.
        self.cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
        limits = getattr(self.cell.profile, "limits", None) or {}
        self.names = list(self.cell.moved_axes)
        # The scene's own checker: `moved_axes` in, the scene's chain out,
        # held axes filled -- one home for the mapping, shared with M4 and M5.
        self.pose_check = (None if planner_collision_model is None else
                           planner_collision_model.checker(self.names, held))
        self.vmax = np.array([float((limits.get(n) or {}).get("velocity_rad_s") or 1e9)
                              for n in self.names])
        self.amax = np.array([float((limits.get(n) or {}).get("acceleration_rad_s2") or 1e9)
                              for n in self.names])
        box_lower, box_upper = stated_position_limits(request.cell, self.names)

        # `durations` is passed only by `retime`, which has trajectories in
        # hand, and the knot COUNTS then come from those trajectories rather
        # than from the spacing rule: a retime that also resampled would change
        # two things at once and the accept test could not say which one it was
        # judging. Its pool is that one trajectory per motion -- a retime
        # refines a solve that has already settled which way round an obstacle
        # to go, so the seeds below are not offered again on a horizon too
        # short to answer the question.
        if durations is None:
            sized = [_horizon_for(spec, self.vmax, self.amax) for spec in moves]
            durations = [d for d, _ in sized]
            counts = [n for _, n in sized]
            pools = [self._pool(spec, n, d, seeds_count)
                     for spec, n, d in zip(moves, counts, durations, strict=True)]
        else:
            if seed_trajs is None:
                raise FreeSpaceError(
                    "a horizon was given with no trajectory to put on it; "
                    "`durations` is `retime`'s argument and it always has one")
            counts = [len(np.asarray(t)) for t in seed_trajs]
            pools = [([np.asarray(t, dtype=np.float64).copy()], ["retimed"])
                     for t in seed_trajs]
        self.durations = [float(d) for d in durations]
        self.counts = counts
        self.times = [np.arange(n) * (d / (n - 1))
                      for n, d in zip(counts, self.durations, strict=True)]

        # Every seed is RE-PACED before it is packed: the same polyline,
        # walked at the pace its motion's caps allow -- see `_repaced` for
        # what skipping this did to a clear route seed.
        move_caps = [_move_caps(spec, n, d, self.vmax, self.amax)
                     for spec, n, d in zip(moves, counts, self.durations,
                                           strict=True)]
        pools = [([_repaced(seed, move_caps[m]) for seed in seeds], labels)
                 for m, (seeds, labels) in enumerate(pools)]

        # The batch: a row per (motion, seed). `row_move` is what lets every
        # per-motion quantity below be gathered rather than assumed equal.
        self.row_move = [m for m, (seeds, _) in enumerate(pools) for _ in seeds]
        self.row_seed = [name for _, names in pools for name in names]
        self.B = len(self.row_move)
        self.L = max(counts)
        self._lengths_list = [counts[m] for m in self.row_move]
        self.lengths = torch.as_tensor(self._lengths_list, device=device)
        index = torch.arange(self.L, device=device)
        self.mask = index.unsqueeze(0) < self.lengths.unsqueeze(1)       # (B, L)
        self._index = index.view(1, -1, 1)
        self.dt = torch.as_tensor(
            [self.durations[m] / (counts[m] - 1) for m in self.row_move],
            dtype=torch.float64, device=device).view(-1, 1, 1)
        # Knots then midpoints -- `_sampled`'s own layout -- so one mask keeps
        # the pad out of the collision sum.
        mid = index[:-1].unsqueeze(0) < (self.lengths - 1).unsqueeze(1)
        self.sample_mask = torch.cat([self.mask, mid], dim=1).to(torch.float64)

        packed = np.zeros((self.B, self.L, len(self.names)))
        row = 0
        for seeds, _ in pools:
            for seed in seeds:
                n = len(seed)
                packed[row, :n] = seed
                packed[row, n:] = seed[-1]
                row += 1
        self.x = torch.tensor(packed, dtype=torch.float64, device=device,
                              requires_grad=True)

        def per_row(pick) -> torch.Tensor:
            return torch.as_tensor(
                np.stack([np.asarray(pick(moves[m]), dtype=np.float64)
                          for m in self.row_move]),
                dtype=torch.float64, device=device)

        self.t_vmax = torch.as_tensor(self.vmax, dtype=torch.float64, device=device)
        self.t_amax = torch.as_tensor(self.amax, dtype=torch.float64, device=device)
        self.t_lower = torch.as_tensor(box_lower, dtype=torch.float64, device=device)
        self.t_upper = torch.as_tensor(box_upper, dtype=torch.float64, device=device)
        self.t_start = per_row(lambda s: s.start)                       # (B, dof)
        self.t_end = per_row(lambda s: s.end)
        self.t_start_rate = per_row(lambda s: s.start_rate)
        self.t_end_rate = per_row(lambda s: s.end_rate)
        # How many knots each end holds at its clamped value -- 1 for a plain
        # endpoint, `PIN_KNOTS` for one that touches a weld.
        self.t_pin_start = torch.as_tensor(
            [moves[m].pinned()[0] for m in self.row_move], device=device
        ).view(-1, 1, 1)
        self.t_pin_end = torch.as_tensor(
            [moves[m].pinned()[1] for m in self.row_move], device=device
        ).view(-1, 1, 1)

        # The rate cap per SEGMENT, radians, pinned bands zeroed --
        # `_move_caps` is the one place that is written down, shared with the
        # seed re-pacing above. Pad segments get `vmax` and are inert: their
        # deltas are zero. `_clip` clamps against this.
        cap = np.zeros((self.B, max(self.L - 1, 1), len(self.names)))
        for row, m in enumerate(self.row_move):
            n = counts[m]
            cap[row] = self.vmax * (self.durations[m] / (n - 1))
            cap[row, :n - 1] = move_caps[m]
        self.step_cap = torch.as_tensor(cap, dtype=torch.float64, device=device)
        # Knots the clip may move: strictly inside a row's PINNED bands. The
        # clip used to move the ends and `_clamp_ends` put them back, which
        # left the boundary SEGMENTS over their caps -- harmless while only
        # `vmax` capped them (never binding at half-speed), wrong once a band
        # is held. A pinned knot moved here and restored there would be the
        # same bug one knot further in.
        here = index.view(1, -1)
        last = (self.lengths - 1).view(-1, 1)
        self._clip_movable = ((here >= self.t_pin_start.view(-1, 1))
                              & (here <= last - self.t_pin_end.view(-1, 1))
                              ).unsqueeze(-1)

        # Seeds are PROJECTED before anything is scored. The monotone loop
        # reverts a step whose cost got worse -- including a step whose only
        # change was the clip -- so feasibility has to be an invariant of every
        # stored iterate, the seeds first of all, or the loop keeps the
        # infeasible incumbent and the caps never bite. Measured before this:
        # the shortcut's straight line flew a held band at 0.89 of `vmax`,
        # every clipped step rejected as costlier.
        self._clamp_ends()
        self._clip()

    @torch.no_grad()
    def buffers(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Per-pose standoff preferences, `(B * (2L - 1),)` each.

        The full `BUFFER_M` in mid-flight, the CLAMPED END's own clearance at the
        ends, ramped between over `BUFFER_RAMP_KNOTS`. A uniform 50 mm asks for
        something unachievable at exactly the two places the optimizer cannot
        move, and then pushes on every knot near them to compensate, bending the
        plunge away from the weld it is trying to reach.

        The ends are MEASURED and not assumed -- per ROW, because every row has
        its own clamped endpoints now: a different fixture, gun or placement
        gives a different number, and so does a different motion.

        The layout matches `_sampled`: `L` knots, then the `L - 1` midpoints,
        with midpoint `j` at position `j + 0.5`. The ramp measures distance to
        each row's OWN last real knot; pad entries can hold anything, because
        `sample_mask` keeps their poses out of the cost.
        """
        span = 2 * self.L - 1
        if self.planner_collision_model is None:
            flat = torch.zeros(self.B * span, dtype=torch.float64, device=self.device)
            return flat + BUFFER_M, flat + BUFFER_SELF_M

        ends = torch.cat([self.t_start, self.t_end])                    # (2B, dof)
        full = self._full_q(ends)
        env_end = self.planner_collision_model.clearance(full).min(dim=1).values.double()
        own = self.planner_collision_model.self_clearance(full)
        self_end = (own.min(dim=1).values.double() if own.numel()
                    else torch.full((2 * self.B,), BUFFER_SELF_M, dtype=torch.float64,
                                    device=self.device))

        position = torch.cat([
            torch.arange(self.L, dtype=torch.float64, device=self.device),
            torch.arange(self.L - 1, dtype=torch.float64, device=self.device) + 0.5,
        ]).unsqueeze(0)                                                 # (1, span)
        last = (self.lengths.to(torch.float64) - 1.0).view(-1, 1)
        from_start = position.expand(self.B, -1)
        from_end = last - position
        nearer_is_start = from_start <= from_end
        from_nearer = torch.minimum(from_start, from_end)
        reach = torch.clamp(from_nearer / max(BUFFER_RAMP_KNOTS, 1e-9),
                            min=0.0, max=1.0)

        def ramp(at_ends: torch.Tensor, wide: float) -> torch.Tensor:
            starts = at_ends[:self.B].view(-1, 1)
            finals = at_ends[self.B:].view(-1, 1)
            near = torch.where(nearer_is_start, starts, finals)
            near = near.clamp(min=0.0, max=wide)
            return (near + (wide - near) * reach).reshape(-1)

        return ramp(env_end, BUFFER_M), ramp(self_end, BUFFER_SELF_M)

    def _pool(self, spec: MoveSpec, count: int, duration: float,
              seeds_count: int) -> tuple[list[np.ndarray], list[str]]:
        """Every way to try the same motion, as curves through the clamped ends.

        Seed 0 is the Hermite through the clamped endpoints. The rest are that
        same curve bowed sideways, one per arm axis and in both directions, by a
        half-sine that vanishes at both ends so every seed still meets the clamps
        exactly. Deterministic, not random: determinism is a stated property of
        this planner, and an RNG would plan the same request differently twice.

        Per MOTION and in numpy, on purpose: seeds are built once at setup and
        the batch is what the optimizer sees, so nothing here is on the clock
        the batching exists to protect.
        """
        start = np.asarray(spec.start, dtype=np.float64)
        end = np.asarray(spec.end, dtype=np.float64)
        start_rate = np.asarray(spec.start_rate, dtype=np.float64)
        end_rate = np.asarray(spec.end_rate, dtype=np.float64)
        u = np.linspace(0.0, 1.0, count).reshape(-1, 1)
        straight = _hermite_batch(start, end, start_rate, end_rate, duration, u)
        bump = np.sin(np.pi * u)                              # (L, 1), zero at both ends
        seeds = [straight]
        names = ["straight"]

        # TWO STAGED SEEDS. The approach must take the wrist from zero to
        # whatever the sweep unwound the seam's first knot to -- 199.5 degrees on
        # `J5` for `bracket_b_2x seam_0001` -- and a straight line in joint space
        # makes that swing while the body is beside the part. Measured: every
        # sinusoidal perturbation of the straight line still penetrated by 30 mm,
        # because the ORDER of the motion is what has to change. So one seed
        # turns the wrist first and then the body, one the reverse; each is a
        # different homotopy class, which descent cannot cross.
        arm_dof = self.cell.arm.chain.num_dof
        wrist = np.zeros(len(self.names))     # `self.names` order: arm first
        wrist[arm_dof - 3:arm_dof] = 1.0
        body = 1.0 - wrist                    # positioner included, as before
        ramp = np.clip(2.0 * u, 0.0, 1.0)                    # done by halfway
        late = np.clip(2.0 * u - 1.0, 0.0, 1.0)              # starts at halfway
        for name, (first, second) in (("wrist-first", (wrist, body)),
                                      ("body-first", (body, wrist))):
            staged = start + (end - start) * (ramp * first + late * second)
            staged[0], staged[-1] = start, end
            seeds.append(staged)
            names.append(name)

        # THE STANDOFF SEEDS: these arrive at the weld from ABOVE it, along the
        # electrode, which no bow or reordering of the straight line does. The
        # waypoint is the weld pose pushed back along `+u` -- `standoff_configs`.
        for k, waypoint in enumerate(spec.via or []):
            seeds.append(self._through(np.asarray(waypoint, dtype=np.float64),
                                       u, duration, start, end,
                                       start_rate, end_rate))
            names.append(f"standoff-{k}")

        # THE ROADMAP SEED, the only one found rather than shaped: a route through
        # configurations a global search screened clear. When the free space is in
        # two pieces, no amount of shaping a curve in one of them reaches the
        # other (`docs/issues/0009`).
        if spec.route is not None and len(spec.route) >= 2:
            seeds.append(self._along(spec.route, u, duration,
                                     start, end, start_rate, end_rate))
            names.append("roadmap")

        while len(seeds) < seeds_count:
            k = len(seeds)
            axis = (k - 1) % arm_dof             # arm axes; the positioner carries the part
            sign = 1.0 if (k - 1) // arm_dof % 2 == 0 else -1.0
            offset = np.zeros(len(self.names))
            offset[axis] = sign * SEED_SWING_RAD
            seeds.append(straight + bump * offset)
            names.append(f"bow-J{axis + 1}{'+' if sign > 0 else '-'}")
        return seeds, names

    def _through(self, via: np.ndarray, u: np.ndarray, duration: float,
                 start: np.ndarray, end: np.ndarray,
                 start_rate: np.ndarray, end_rate: np.ndarray) -> np.ndarray:
        """The seed that goes start -> `via` -> end, as two Hermites.

        The horizon is split by the TIME each leg needs -- joint travel over the
        rate limits, worst axis -- and not evenly: on an approach the waypoint
        sits a hand's width from the weld, so an even split would seed a crawl
        into the part and a lunge across the cell.

        The seed stops at the waypoint, so both legs meet C1 by having no corner
        rather than by a fit. Only the SEED stops: the waypoint is not clamped and
        the smoothness term pays for a needless halt, so a pass-through survives
        the loop if a pass-through is cheaper. The clamped endpoints stay exact --
        at `u = 0` the first leg is `start`, at `u = 1` the second is `end`.
        """
        scale = np.maximum(self.vmax, 1e-9)
        near = float((np.abs(via - start) / scale).max())
        far = float((np.abs(end - via) / scale).max())
        # Clipped so neither leg is starved: a waypoint 100 mm from the weld is a
        # fraction of a percent of the flight by this measure.
        split = float(np.clip(near / max(near + far, 1e-12), 0.15, 0.85))

        rest = np.zeros_like(via)
        first = _hermite_batch(start, via, start_rate, rest,
                               split * duration, np.clip(u / split, 0.0, 1.0))
        second = _hermite_batch(via, end, rest, end_rate,
                                (1.0 - split) * duration,
                                np.clip((u - split) / (1.0 - split), 0.0, 1.0))
        return np.where(u <= split, first, second)

    def _along(self, route: np.ndarray, u: np.ndarray, duration: float,
               start: np.ndarray, end: np.ndarray,
               start_rate: np.ndarray, end_rate: np.ndarray) -> np.ndarray:
        """The seed that follows a whole roadmap route.

        Roadmap waypoints come with no timing and no velocities, so they are laid
        out along the horizon by the time metric the roadmap searched in and each
        interior one is given a centred difference of its neighbours --
        Catmull-Rom. Resting at every waypoint, the way `_through` rests at its
        single standoff, would seed a stop-start crawl through a dozen poses.

        The clamped endpoint velocities are used as-is on the first and last legs,
        so this seed meets the same contract as every other one.
        """
        route = np.asarray(route, dtype=np.float64)
        scale = np.maximum(self.vmax, 1e-9)
        leg = np.abs(np.diff(route, axis=0) / scale).max(axis=1)
        if not leg.sum():
            return _hermite_batch(start, end, start_rate, end_rate, duration, u)
        edge = np.concatenate([[0.0], np.cumsum(leg) / leg.sum()])   # (W,) in [0, 1]

        rate = np.zeros_like(route)
        rate[0], rate[-1] = start_rate, end_rate
        if len(route) > 2:
            gap = ((edge[2:] - edge[:-2]) * duration).reshape(-1, 1)
            rate[1:-1] = (route[2:] - route[:-2]) / np.maximum(gap, 1e-9)

        out = np.empty((len(u), route.shape[1]))
        for i in range(len(route) - 1):
            lo, hi = edge[i], edge[i + 1]
            span = max(hi - lo, 1e-12)
            here = (u[:, 0] >= lo) & (u[:, 0] <= hi) if i else (u[:, 0] <= hi)
            local = np.clip((u[here] - lo) / span, 0.0, 1.0)
            out[here] = _hermite_batch(route[i], route[i + 1], rate[i], rate[i + 1],
                                       span * duration, local)
        return out

    # ----------------------------------------------------------------- rates

    def velocities(self, x: torch.Tensor) -> torch.Tensor:
        """(B, L, dof): central differences inside, the DECLARED rate at an
        unpinned end.

        A weld-side end arrives at and leaves from REST, and the pin is what
        makes that so: across `PIN_KNOTS` identical knots the central
        difference IS zero, so nothing has to be written over it. Imposing a
        rate there used to hide the one thing worth seeing -- a trajectory
        that lunged into the weld reported the rate it had been promised.

        The declared rate is still imposed at an end that is NOT pinned, which
        is `plan_move`'s contract for a caller splicing onto a clock of its
        own. A row's end is its OWN last real knot, `lengths - 1`, not the
        padded `-1`; written at the pad, the end rate would land on a knot the
        motion does not have and the real last knot would keep half its true
        one-sided velocity. The pad's rate is zero -- it does not move.
        """
        qdot = torch.zeros_like(x)
        qdot[:, 1:-1] = (x[:, 2:] - x[:, :-2]) / (2 * self.dt)
        last = (self.lengths - 1).view(-1, 1, 1)
        loose_start = (self._index == 0) & (self.t_pin_start == 1)
        loose_end = (self._index == last) & (self.t_pin_end == 1)
        qdot = torch.where(loose_start, self.t_start_rate.unsqueeze(1), qdot)
        qdot = torch.where(loose_end, self.t_end_rate.unsqueeze(1), qdot)
        return qdot * (self._index <= last)

    # ----------------------------------------------------------------- costs

    def smoothness(self, x: torch.Tensor) -> torch.Tensor:
        """(B,) -- per row, because the point of running several is to
        choose between them.

        M5's three smoothness terms, plus a fourth this module adds: the L1 joint
        travel, in seconds at full rate. The three price how a path is flown, the
        fourth how far it goes -- see `W_TRAVEL`. Each difference is masked to
        the rows that actually have that many knots: the pad repeats the clamped
        end so it holds still, but the step from the last real velocity down to
        the pad's zero would read as a violent deceleration, and the
        acceleration term would price a stop that is not in the motion.
        """
        step = (x[:, 1:] - x[:, :-1])
        vel = step / self.dt
        acc = (vel[:, 1:] - vel[:, :-1]) / self.dt
        jerk = (acc[:, 1:] - acc[:, :-1]) / self.dt
        return (W_VEL * ((vel / self.t_vmax).pow(2) * self._span(1)).sum(dim=(1, 2))
                + W_ACC * ((acc / self.t_amax).pow(2) * self._span(2)).sum(dim=(1, 2))
                + W_JERK * ((jerk / (self.t_amax / self.dt)).pow(2)
                            * self._span(3)).sum(dim=(1, 2))
                + W_TRAVEL * ((step / self.t_vmax).abs() * self._span(1)).sum(dim=(1, 2)))

    def _span(self, order: int) -> torch.Tensor:
        """(B, L-order, 1): true where a difference of this order is real."""
        index = torch.arange(self.L - order, device=self.device)
        return (index.unsqueeze(0) < (self.lengths - order).unsqueeze(1)
                ).to(torch.float64).unsqueeze(-1)

    def _full_q(self, x: torch.Tensor) -> torch.Tensor:
        """(N, dof) for the collision planner_collision_model, held axes
        reattached -- through the scene's own `checker`."""
        return self.pose_check.q_full(x.reshape(-1, x.shape[-1]))

    def _sampled(self, x: torch.Tensor, qdot: torch.Tensor) -> torch.Tensor:
        """The knots AND the midpoints between them.

        Collision has to be pushed against where the arm actually is, which
        between two knots is neither of them. The optimizer may sample here; the
        verifier that judges the result may not, and does not.
        """
        mid = (0.5 * (x[:, :-1] + x[:, 1:])
               + 0.125 * self.dt * (qdot[:, :-1] - qdot[:, 1:]))
        return torch.cat([x, mid], dim=1)

    # ------------------------------------------------------------------- run

    def run(self, collision: CollisionCost | None, *,
            inner_steps: int = INNER_STEPS,
            outer_steps: int = OUTER_STEPS,
            shortcut_levels: int = SHORTCUT_LEVELS) -> list[_Solved]:
        """Solve every row at once and hand back each motion's winner.

        Descend, then alternate straightening with descent (`SHORTCUT_CYCLES`),
        then rank. A shortcut is a DISCRETE jump between paths and descent is a
        continuous deformation, so the two find different things and each cycle
        hands the other a better starting point.

        Straightening happens BEFORE the ranking, so the seeds are compared as the
        shapes they will actually be flown as.

        Winners are per MOTION, chosen the way `weld_trajopt_main.WeldTrajoptModule`
        chooses a candidate: a seed that PENETRATES loses to every seed that does
        not, whatever its smoothness says. Among the clear ones, lowest cost.
        """
        self._clamp_ends()
        with torch.no_grad():
            cost0 = self.smoothness(self.x)

        self._descend(collision, inner_steps, outer_steps)
        for _ in range(SHORTCUT_CYCLES if shortcut_levels else 0):
            if not self.shortcut(levels=shortcut_levels):
                break
            # The straight line is projected before it is scored, for the same
            # invariant the seeds hold: no stored iterate over its caps.
            self._clamp_ends()
            self._clip()
            self._descend(collision, inner_steps, 1)

        with torch.no_grad():
            knots = self.x.detach()
            qdot = self.velocities(knots)
            cost1 = self.smoothness(knots)
            env = torch.full((self.B,), float("inf"), device=self.device,
                             dtype=torch.float64)
            mine = env.clone()
            if collision is not None:
                env, mine = self._clearance(knots, qdot)
            # ONE transfer off the device for the whole batch, at the end.
            # How far each row's steps sit over their caps -- the projection is
            # approximate at the pinned bands, and a row on a horizon too
            # short to carry its path cannot satisfy them at all.
            steps = (knots[:, 1:] - knots[:, :-1]).abs()
            over = ((steps - self.step_cap).clamp_(min=0.0)
                    * self.mask[:, 1:].unsqueeze(-1)).amax(dim=(1, 2))
            bulk = {name: t.cpu().numpy() for name, t in
                    (("knots", knots), ("qdot", qdot), ("cost0", cost0),
                     ("cost1", cost1), ("env", env), ("self", mine),
                     ("over", over))}
        gap = np.minimum(bulk["env"], bulk["self"])

        out: list[_Solved] = []
        for m in range(self.M):
            rows = [r for r in range(self.B) if self.row_move[r] == m]
            # A seed that penetrates loses to every seed that does not. When NONE
            # is clear the tie-break has to be clearance and not cost: ranking
            # failures by smoothness picked a 30 mm penetration over a 4 mm one.
            win = min(rows,
                      key=lambda r: ((float(gap[r]) < 0.0, -float(gap[r]))
                                     if float(gap[r]) < 0.0
                                     else (False, float(bulk["cost1"][r]))))
            n = self.counts[m]                 # the pad is not a trajectory
            out.append(_Solved(
                knots=bulk["knots"][win][:n],
                qdot=bulk["qdot"][win][:n],
                times_s=self.times[m],
                cost_before=float(bulk["cost0"][win]),
                cost_after=float(bulk["cost1"][win]),
                clearance_m=float(gap[win]),
                env_clearance_m=float(bulk["env"][win]),
                self_clearance_m=float(bulk["self"][win]),
                seed=self.row_seed[win],
                cap_excess_rad=float(bulk["over"][win])))
        return out

    def _cost(self, collision: CollisionCost | None) -> torch.Tensor:
        """(B,) the whole objective: smoothness, travel, collision, buffer.

        The collision sum is masked with `sample_mask`: the pad repeats a real
        near-weld stance, so unmasked it would be charged -- and the augmented
        Lagrangian would grow multipliers on poses no motion contains.
        """
        value = self.smoothness(self.x)
        if collision is not None:
            per_pose = collision.value(
                self._full_q(self._sampled(self.x, self.velocities(self.x))))
            value = value + (per_pose.view(self.B, -1) * self.sample_mask).sum(dim=1)
        return value

    def _descend(self, collision: CollisionCost | None, inner_steps: int,
                 outer_steps: int) -> None:
        """The monotone Adam loop, on every seed at once.

        The outer loop is where the augmented Lagrangian's multipliers grow, the
        only mechanism here that can drive a penetration to exactly zero rather
        than to `1/w`. The rest of what it does -- re-baseline the incumbent,
        throw away Adam's moments, restart the optimizer -- is bookkeeping for an
        objective that genuinely changed underneath the iterate.

        It is also where the trust scale used to die: re-baselining makes the next
        few steps look bad and the ratchet halved on each of them
        (`docs/issues/0011` has the trace). `TRUST_GROWTH` is what makes the outer
        loop survivable rather than something to remove.
        """
        capturable = self.device.type == "cuda"
        opt = torch.optim.Adam([self.x], lr=ADAM_LR, capturable=capturable)
        scale = torch.ones(self.B, 1, 1, dtype=torch.float64, device=self.device)
        with torch.no_grad():
            best = self._cost(collision).clone()
        steps = max(int(inner_steps), 0)
        graph, warmed = ((None, 0) if not steps
                         else self._capture(opt, collision, scale, best, steps))
        for outer in range(max(int(outer_steps), 1)):
            # The capture's warm-up runs were real steps, taken out of the
            # FIRST outer's budget and nobody else's.
            for _ in range(steps - (warmed if outer == 0 else 0)):
                if graph is not None:
                    graph.replay()
                else:
                    self._step(opt, collision, scale, best)
            if collision is not None and steps:
                collision.step_multipliers(
                    self._full_q(self._sampled(self.x.detach(),
                                               self.velocities(self.x.detach()))),
                    grow=True)
                with torch.no_grad():
                    best.copy_(self._cost(collision))
                    # A fresh optimizer, without becoming a fresh OBJECT: the
                    # graph replays against this one's state tensors, so the
                    # moments and the step count are zeroed in place instead.
                    # `rho` and the multipliers are device tensors updated in
                    # place by `step_multipliers` for the same reason.
                    for state in opt.state.values():
                        for moment in state.values():
                            if torch.is_tensor(moment):
                                moment.zero_()

    def _capture(self, opt: torch.optim.Optimizer,
                 collision: CollisionCost | None, scale: torch.Tensor,
                 best: torch.Tensor, steps: int) -> tuple[Any, int]:
        """`(graph, warm steps taken)`: one `_step` recorded as a CUDA graph.

        The step is launch-bound, not compute-bound -- ~1,900 kernel launches
        against microsecond kernels (`BENCHMARKS.md` 11.3) -- and a replay
        costs about the GPU time alone; probed on this cell, 5.5 ms eager
        against 1.0 ms replayed. One recording serves the WHOLE descent: the
        outer step's multiplier growth and optimizer reset are in-place
        tensor updates the recorded kernels read live.

        The warm-up runs on a side stream because capture refuses to start
        while ordinary work is pending. Its two runs are REAL steps and the
        caller counts them; the recording pass executes nothing.
        """
        if self.device.type != "cuda" or steps < CAPTURE_MIN_STEPS:
            return None, 0
        try:
            stream = torch.cuda.Stream()
            stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(stream):
                for _ in range(2):
                    self._step(opt, collision, scale, best)
            torch.cuda.current_stream().wait_stream(stream)
            graph = torch.cuda.CUDAGraph()
            with torch.cuda.graph(graph):
                self._step(opt, collision, scale, best)
        except Exception:  # noqa: BLE001 -- whatever refused, eager answers
            # Capture is an optimisation, exactly as `robot/fk.py` treats it:
            # the iterate is whatever the warm-up left, and the eager loop
            # finishes the job.
            return None, 2
        return graph, 2

    def _step(self, opt: torch.optim.Optimizer, collision: CollisionCost | None,
              scale: torch.Tensor, best: torch.Tensor) -> None:
        """One monotone step, in place on `(x, moments, scale, best)`.

        Every quantity it touches is a pre-existing tensor updated in place
        and every decision a mask -- no device-to-host read anywhere. That is
        what makes this the unit a CUDA graph could replay, and it is also
        what keeps the loop from synchronising once per step.
        """
        snapshot = self.x.detach().clone()
        opt.zero_grad(set_to_none=False)
        self._cost(collision).sum().backward()
        opt.step()
        with torch.no_grad():
            self.x.data.copy_(snapshot + scale * (self.x.data - snapshot))
        self._clamp_ends()
        self._clip()
        with torch.no_grad():
            now = self._cost(collision)
            worse = now > best + 1e-9
            self.x.data.copy_(torch.where(worse.view(-1, 1, 1),
                                          snapshot, self.x.data))
            _forget_moments(opt, self.x, worse)
            # Halve on a rejection, RECOVER on an acceptance; the cap
            # keeps recovery at Adam's own step. See `TRUST_GROWTH`.
            scale.mul_(torch.where(worse, 0.5, TRUST_GROWTH).view(-1, 1, 1))
            scale.clamp_(min=5e-4, max=1.0)
            best.copy_(torch.minimum(best, now))

    # ---------------------------------------------------------- straightening

    @torch.no_grad()
    def shortcut(self, *, levels: int = SHORTCUT_LEVELS,
                 step_rad: float = SHORTCUT_STEP_RAD) -> bool:
        """Replace what can be replaced by the straight line through it.

        A span `(i, j)` is a candidate, the straight line from `x[i]` to `x[j]`
        replaces its interior, and it is taken if that line screens clear. This is
        cuRobo's `PathPruner.prune_path_with_shortcuts` on a trajectory's knots
        instead of a graph path.

        Coarse to fine, not all pairs: all pairs of a 40-knot motion is 780
        candidates whose screens total `O(L^3)` configurations, seconds per motion
        for a pass that runs on every seed. So the whole motion first, then each
        half, then each quarter, with the spans at each level overlapping by half
        so nothing straddling a boundary is invisible -- `O(L log L)`, biggest
        saving tested first.

        A candidate is sampled at a fixed angular STEP, not a fixed count, and
        every sample must clear both fields. That is a SCREEN and not a proof,
        exactly as `roadmap.py` says of itself; what comes out is judged by the T2
        verifier, which certifies the curve.

        Only the interior moves, so the clamped endpoints and the endpoint
        VELOCITIES are untouched by construction, and since velocities are central
        differences of the knots the trajectory stays C1 through a shortcut's ends
        with nothing to refit.

        Returns whether anything was taken.
        """
        if self.L < 4:
            return False
        touched = False
        width = self.L - 1
        for _ in range(max(int(levels), 1)):
            spans = self._spans(width)
            if not spans:
                break
            keep = self._clear_spans(spans, step_rad)
            # Left to right, never two that share an interior knot: a candidate
            # was screened against the path as it stood, so an overlapping
            # application would fly a line nothing looked at.
            for (i, j), rows in self._accept(spans, keep):
                self._straighten(rows, i, j)
                touched = True
            if width <= 2:
                break
            width = max(2, width // 2)
        return touched

    def _spans(self, width: int) -> list[tuple[int, int]]:
        """Candidate `(i, j)` at one level, overlapping by half a width."""
        step = max(1, width // 2)
        out = []
        for i in range(0, self.L - width, step):
            out.append((i, i + width))
        last = self.L - 1 - width
        if last >= 0 and (not out or out[-1][0] != last):
            out.append((last, last + width))
        return [(i, j) for i, j in out if j - i >= 2]

    def _straighten(self, rows: list[int], i: int, j: int) -> None:
        """`x[rows, i+1:j]` becomes the straight line from `x[i]` to `x[j]`."""
        index = torch.as_tensor(rows, device=self.device)
        a = self.x.data[index, i].unsqueeze(1)
        b = self.x.data[index, j].unsqueeze(1)
        t = torch.linspace(0.0, 1.0, j - i + 1, dtype=self.x.dtype,
                           device=self.device)[1:-1].view(1, -1, 1)
        self.x.data[index, i + 1:j] = a * (1.0 - t) + b * t

    def _clear_spans(self, spans: list[tuple[int, int]],
                     step_rad: float) -> np.ndarray:
        """`(len(spans), K)` -- which seeds could take which span.

        One screen for every span of every seed, in a single call: a per-span loop
        would put back the launch overhead everything else here is batched to
        avoid.
        """
        rows: list[torch.Tensor] = []
        counts: list[int] = []
        for i, j in spans:
            a = self.x.data[:, i]                                    # (B, dof)
            b = self.x.data[:, j]
            reach = float((b - a).abs().max())
            n = int(np.clip(round(reach / max(step_rad, 1e-6)), 2, 512))
            t = torch.linspace(0.0, 1.0, n + 2, dtype=self.x.dtype,
                               device=self.device)[1:-1].view(1, -1, 1)
            rows.append(a.unsqueeze(1) * (1.0 - t) + b.unsqueeze(1) * t)   # (B, n, dof)
            counts.append(n)
        if self.planner_collision_model is None:
            # Nothing to hit, so every span is clear and the motion collapses to
            # the straight line.
            return np.ones((len(spans), self.B), dtype=bool)

        packed = torch.cat(rows, dim=1)                              # (B, sum n, dof)
        env, mine = self._screen(packed)
        out = np.zeros((len(spans), self.B), dtype=bool)
        at = 0
        for k, n in enumerate(counts):
            gap = torch.minimum(env[:, at:at + n], mine[:, at:at + n]).min(dim=1).values
            out[k] = (gap > 0.0).cpu().numpy()
            at += n
        return out

    def _screen(self, points: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """`(env, self)` clearance at `(B, N, dof)` configurations, `(B, N)` each."""
        flat = self._full_q(points)
        env = self.planner_collision_model.clearance(flat).min(dim=1).values
        own = self.planner_collision_model.self_clearance(flat)
        mine = (own.min(dim=1).values if own.numel()
                else torch.full_like(env, float("inf")))
        shape = points.shape[:2]
        return env.view(shape).double(), mine.view(shape).double()

    def _accept(self, spans: list[tuple[int, int]],
                keep: np.ndarray) -> list[tuple[tuple[int, int], list[int]]]:
        """Per span, which rows may take it -- overlaps dropped, per row.

        The bookkeeping is per ROW and not per level: rows are independent
        problems that happen to share a tensor, and one may take the whole
        motion while another takes only its first half.

        A span is also refused to any row it does not FIT: spans are laid out
        on the padded length, and a span whose far end is past a row's own
        last knot would hand `_straighten` that row's REAL endpoint as
        interior -- overwriting a clamped end with a point on somebody's
        straight line.
        """
        # Per row, the last interior knot already rewritten. A span may start
        # there -- it does not move its own first knot -- and no earlier.
        claimed: dict[int, int] = {}
        out = []
        for index, (i, j) in enumerate(spans):
            rows = [int(k) for k in np.nonzero(keep[index])[0]
                    if i >= claimed.get(int(k), 0)
                    and j <= self._lengths_list[int(k)] - 1]
            if rows:
                for k in rows:
                    claimed[k] = j - 1
                out.append(((i, j), rows))
        return out

    def collision_cost(self) -> CollisionCost | None:
        """The term this module descends on: the AL on `clearance >= 0`, and the
        50 mm buffer beside it as a fixed hinge.

        **`margin_m` is zero.** A 50 mm margin hands the augmented Lagrangian a
        target the arm cannot reach, because a motion BEGINS at a clamped weld pose
        millimetres off the work, so its multipliers accumulate forever. Measured
        after six outer steps on `bracket_b_2x`'s approach: nothing in collision,
        118 constraint entries still under growing pressure, `rho` at 14074,
        multipliers at 1257, every bit of it spent on standoff.

        Margin zero with NO buffer is the opposite failure -- the term is silent
        until the arm is already inside, and three of four motions penetrated. So
        the buffer stays as a fixed weight, masked narrower near the weld where
        50 mm is not on offer (`buffers`).

        The AL keeps the hard constraint because only it finishes: at fixed weight
        the same motion plateaus at -2.50 mm and is still at -0.06 mm with the weight
        a hundred times higher. The sweep is in `cost.py`'s docstring.
        """
        if self.planner_collision_model is None:
            return None
        env, own = self.buffers()
        return CollisionCost(scene=self.planner_collision_model,
                             batch=self.B * (2 * self.L - 1),
                             margin_m=0.0, self_margin_m=0.0, rho=COLLIDE_RHO,
                             buffer_m=env, buffer_self_m=own,
                             buffer_weight=DEFAULT_BUFFER_WEIGHT)

    # --------------------------------------------------------------- helpers

    @torch.no_grad()
    def _clearance(self, knots: torch.Tensor,
                   qdot: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """`(env, self)` worst gap per row, `(B,)` each, the pad excluded.

        BOTH fields. A ranking that reads only the environment field scores a path
        that folded the forearm into the wrist as perfectly clear: measured on
        `bracket_b_2x`, the retract's straight seed ranked first at +7.7 mm of
        environment clearance while the verifier found four segments of `link_4`
        against `link_6` and `end_effector` against `link_3`.

        Kept apart rather than minimised into one number because they fail for
        different reasons and are fixed by different things (`docs/issues/0009`).
        """
        sampled = self._full_q(self._sampled(knots, qdot))
        rows = knots.shape[0]
        pad = ~self.sample_mask.bool()

        def worst(values):
            per_pose = values.min(dim=1).values.view(rows, -1)
            return per_pose.masked_fill(pad, float("inf")).min(dim=1).values.double()

        mine = self.planner_collision_model.self_clearance(sampled)
        return (worst(self.planner_collision_model.clearance(sampled)),
                worst(mine) if mine.numel()
                else torch.full((rows,), float("inf"), dtype=torch.float64,
                                device=knots.device))

    @torch.no_grad()
    def _clamp_ends(self) -> None:
        """The endpoints are DATA, not variables. Adam moves every element it is
        given, so they are written back after every step rather than being trusted
        to stay: a motion whose ends drifted would hand the weld an arm in the
        wrong place. A row's end is at its OWN `lengths - 1`, and everything past
        it is pinned to the same value -- which is what keeps the pad a repeat of
        the clamped end rather than whatever the optimizer walked it to.

        A weld-touching end holds `PIN_KNOTS` of them and not one. That band is
        what makes the handover a stop the VERIFIER can see: its first
        difference across the band is exactly zero, so the endpoint rate is a
        property of the knots instead of a number written over them."""
        last = (self.lengths - 1).view(-1, 1, 1)
        head = self._index < self.t_pin_start
        tail = self._index >= last - (self.t_pin_end - 1)
        self.x.data.copy_(torch.where(head, self.t_start.unsqueeze(1),
                                      self.x.data))
        self.x.data.copy_(torch.where(tail, self.t_end.unsqueeze(1),
                                      self.x.data))

    @torch.no_grad()
    def _clip(self) -> None:
        """Boxes, then the pairwise velocity clamp, forward then backward.

        The two sweeps are an approximate projection onto the rate set -- a
        forward pass alone shovels all the correction onto later knots. The
        verifier re-measures afterwards, so approximation costs iterations,
        not truth. The cap comes from `step_cap`, per SEGMENT: the rate limit
        per axis, `dt` per ROW, and zero across a pinned band.
        The pinned bands themselves are never moved (`_clip_movable`), so
        `_clamp_ends` runs BEFORE this and nothing has to be put back after.
        """
        self.x.data.clamp_(self.t_lower, self.t_upper)
        for indices, direction in ((range(1, self.L), 1),
                                   (range(self.L - 2, -1, -1), -1)):
            for i in indices:
                anchor = i - direction        # the knot already fixed this sweep
                cap = self.step_cap[:, min(i, anchor)]
                delta = self.x.data[:, i] - self.x.data[:, anchor]
                moved = self.x.data[:, anchor] + delta.clamp(min=-cap, max=cap)
                self.x.data[:, i] = torch.where(self._clip_movable[:, i],
                                                moved, self.x.data[:, i])


def retime(solver: MoveSolver, solved: list[_Solved], *,
           attempts: int = RETIME_ATTEMPTS, target: float = RETIME_TARGET,
           inner_steps: int = INNER_STEPS,
           outer_steps: int = RETIME_OUTER_STEPS
           ) -> tuple[list[_Solved], list[list[str]]]:
    """Shrink each horizon until an axis is near its limit, and re-solve on it.

    `duration_for` is a floor -- exact only for a cubic from rest to rest along
    the STRAIGHT line, then padded by `DURATION_SLACK` -- so once the optimizer
    has moved the interior the knots reach about half the rate limits. Measured
    on `bracket_b_2x`, worst velocity 0.46-0.54 of the limit and worst
    acceleration 0.29-0.48 across all four motions: every one was flown at half
    speed.

    A re-solve and not a rescale, because the endpoint velocities are the WELD's
    and do not scale with the horizon. Shrink it and the same imposed rate belongs
    to a shorter first segment, which changes the shape near both ends -- so the
    optimizer is asked again, seeded with the trajectory it just produced. This is
    cuRobo's `finetune_dt_scale` loop (`solver_trajopt.py`).

    **The rounds are LOCKSTEP over the batch.** Attempts are sequential within a
    motion -- each seeds from the last answer -- but the motions are independent,
    so round `k` re-solves every motion that still wants shrinking as one batch.
    A motion is done when its rates ask for no shrink (`scale > 0.98`); a refusal
    bisects that motion's scale toward 1 and it rides the next round. Solved
    one at a time this was 2.68x of the whole join, all of it at batch 1 on a
    launch-bound loop (`BENCHMARKS.md` 12.5).

    A shrink is kept only if the result is clear -- or, when the slow motion was
    not clear either, only if it is no worse. Nothing here can turn a certified
    motion into an uncertified one.

    **A shrink is bounded by `_carry_floor` before it is tried at all.** The
    knot COUNT is held here, so shrinking the horizon shrinks every segment's
    step budget while the path it has to walk stays the length it was. Past
    that floor the re-paced seed cannot reach its own endpoint and the
    shortfall lands as one segment at the weld. Rate demand alone always asked
    for less than the floor on a motion that stops at its ends, so every
    retime used to spend its first attempt solving a broken seed and then
    refusing the result.

    **The cost pair travels with the horizon.** Smoothness is measured against
    `dt`, so a seed's cost on a 3.8-second horizon and an answer's on a
    1.4-second one are not the same quantity: comparing them once read as a
    motion leaving three times rougher when it had left in a third of the time.
    An accepted shrink keeps BOTH costs from its own solve.
    """
    notes: list[list[str]] = [[] for _ in solved]

    def wanted(m: int, s: _Solved) -> float:
        dt = float(s.times_s[1] - s.times_s[0])
        ask = time_scale(s.knots, dt, solver.vmax, solver.amax, target=target)
        floor = _carry_floor(solver.moves[m], s.knots, dt, solver.vmax,
                             solver.amax)
        return float(min(1.0, max(ask, floor)))

    scales = [wanted(m, s) for m, s in enumerate(solved)]
    for _attempt in range(max(int(attempts), 0)):
        todo = [m for m, scale in enumerate(scales) if scale <= 0.98]
        if not todo:
            break
        durations = [float(solved[m].times_s[-1]) * scales[m] for m in todo]
        faster = MoveSolver(
            solver.request, solver.planner_collision_model, solver.device,
            moves=[solver.moves[m] for m in todo], held=solver.held,
            durations=durations, seed_trajs=[solved[m].knots for m in todo],
            cell=solver.cell)
        # A fresh cost object, multipliers at zero. That coupling mattered while
        # the AL carried the 50 mm buffer: a retime given nothing accumulated cut
        # 22.7 mm into the part at 1.98 s on a path that had been 3.9 mm clear.
        # With the buffer on a fixed weight the pressure is there from the first
        # step, so there is nothing left to inherit.
        #
        # No shortcut pass either: the geometry is the one already screened and
        # straightened, and only the clock changed.
        answers = faster.run(faster.collision_cost(), inner_steps=inner_steps,
                             outer_steps=outer_steps, shortcut_levels=0)

        for slot, m in enumerate(todo):
            fast = answers[slot]
            was = solved[m].clearance_m
            now = fast.clearance_m
            # Clear enough and within the optimizer's discrete rate budget. The
            # emitted Hermite curve is judged exactly downstream; this heuristic
            # can reject a bad shrink but cannot certify the final curve.
            dt_fast = float(fast.times_s[1] - fast.times_s[0])
            over = max(rate_ratios(fast.knots, dt_fast, solver.vmax, solver.amax))
            refusal = None
            if fast.cap_excess_rad > 1e-9:
                refusal = (f"it would break the crawl by "
                           f"{fast.cap_excess_rad:.4f} rad a step")
            elif np.isinf(was) and np.isinf(now):
                # No collision model: `_clearance` reports `inf` because there
                # is nothing to be clear OF, and a clearance test against a
                # quantity that does not exist has no opinion. Requiring a
                # FINITE `was` refused every shrink instead -- a cell planned
                # without a collision model kept its first horizon whole, and
                # the motion came back at 1.87x the time it needed.
                pass
            elif not (np.isfinite(was) and now >= was - RETIME_GIVE_M and now >= 0.0):
                refusal = (f"it would leave {now * 1000:.1f} mm where "
                           f"{was * 1000:.1f} mm was clear")
            elif over > 1.0:
                refusal = f"it would reach {over:.2f} of a rate limit"
            if refusal is not None:
                # BISECT rather than give up: the horizon the rates asked for is
                # too short, and the answer is a longer one, not the one the
                # motion arrived with. Measured on `bracket_b_2x`'s approach,
                # giving up on the first refusal cost the whole saving -- the
                # truth was 1.81 s.
                notes[m].append(f"refused {durations[slot]:.2f} s: {refusal}")
                scales[m] = 0.5 * (scales[m] + 1.0)
                continue
            notes[m].append(
                f"retimed {solved[m].times_s[-1]:.2f} s -> {durations[slot]:.2f} s")
            # The seed that won the SOLVE, not the retime's placeholder: which
            # of the pool a motion only works from is worth knowing about, and
            # every accepted retime would otherwise report "retimed".
            fast.seed = solved[m].seed
            solved[m] = fast
            scales[m] = wanted(m, fast)
    return solved, notes


def plan_moves(
        request: PlanRequest, planner_collision_model, device, *,
        specs: list[MoveSpec], held: dict[str, float],
        verifier_collision_model: Any = None,
        verify_margin_m: float = VERIFY_MARGIN_M,
        inner_steps: int = INNER_STEPS, outer_steps: int = OUTER_STEPS,
        retime_attempts: int = RETIME_ATTEMPTS,
        shortcut_levels: int = SHORTCUT_LEVELS, backend: str | None = None,
        cell: Any = None,
        progress: Any = None) -> list[Move]:
    """Every motion in `specs`, planned as ONE batch, straightened, retimed
    and judged.

    The batch is as many motions as the caller has -- there is no cap here.
    The biggest tensor is the self-collision screen, batch x 1663 pairs, and
    a whole 22-seam program's motions keep it in the hundreds of megabytes;
    a caller with enough motions to change that arithmetic should split the
    list, not this function.
    """
    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    if progress is not None:
        progress("optimize")
    solver = MoveSolver(request, planner_collision_model, device,
                        moves=specs, held=held, cell=cell)
    # The REQUEST's choice wins; the module global is the default a caller who
    # does not care inherits.
    chosen = (backend or FREE_SPACE_BACKEND or "bspline").strip().lower()
    if chosen not in ("bspline", "curobo", "legacy"):
        raise FreeSpaceError(
            f"unknown free-space backend {chosen!r}; "
            "expected 'bspline', 'curobo' or 'legacy'")
    if chosen == "curobo":
        from weld_motion_planner.planner.trajectory_optimization.engine import (
            curobo_backend as _curobo,
        )
        solved, notes, _raw = _curobo.solve_batch(
            request, planner_collision_model, device, specs=specs, held=held,
            cell=cell)
        slow = [float(s.times_s[-1]) for s in solved]
        return _finish_moves(request, specs, solved, notes, slow, solver, cell,
                             held, verifier_collision_model, verify_margin_m,
                             progress)
    if chosen == "bspline":
        # `docs/issues/0015`: the control-point solver, whose curve is C2 and
        # whose limits are checked on the curve rather than on a knot grid the
        # excess hides between. `solver` is still built because `plan_moves`
        # reads `solver.vmax` and `solver.names` for the detour and the
        # verifier's axis order; it is not asked to solve.
        from weld_motion_planner.planner.trajectory_optimization.engine import (
            control_solver as _control,
        )
        solved, notes, _raw = _control.solve_batch_as_freespace(
            request, planner_collision_model, device, specs=specs, held=held,
            cell=cell, inner_steps=inner_steps, outer_steps=outer_steps,
            retime_attempts=retime_attempts)
        slow = [float(s.times_s[-1]) for s in solved]
        return _finish_moves(request, specs, solved, notes, slow, solver, cell,
                             held, verifier_collision_model, verify_margin_m,
                             progress)
    solved = solver.run(solver.collision_cost(), inner_steps=inner_steps,
                        outer_steps=outer_steps, shortcut_levels=shortcut_levels)
    slow = [float(s.times_s[-1]) for s in solved]
    solved, notes = retime(solver, solved, attempts=retime_attempts,
                           inner_steps=inner_steps)

    return _finish_moves(request, specs, solved, notes, slow, solver,
                         cell, held, verifier_collision_model,
                         verify_margin_m, progress)


def retime_rest_to_rest_wire(names, knots, rates, times, limits):
    """Lengthen a stopped free-space move to fit its exact emitted rates.

    Optimizer samples can miss an acceleration peak between them, and its
    bounded re-solve budget can finish above a limit. Timing is free for these
    connecting moves. Scaling every interval and its tangent inversely leaves
    every point of the Hermite path unchanged, including collision clearance.
    Moving endpoints keep their velocity contract and are left for the normal
    verifier to accept or refuse. Weld trajectories never call this function.
    """
    exact = judge_hermite_rates(names, knots, rates, times, limits)
    if not exact["violations"] or np.any(rates[[0, -1]] != 0.0):
        return times, rates, 1.0
    scale = max(
        max(axis["velocity"]["ratio"], np.sqrt(axis["acceleration"]["ratio"]))
        for axis in exact["per_joint"]
    )
    # Leave binary64 arithmetic headroom; this changes the trajectory clock,
    # never the declared limits or the verifier's strict comparison.
    scale *= 1.0 + 1e-6
    if not np.isfinite(scale):
        raise HermiteRateError("free-space retiming needs a finite duration scale")
    slowed_times = times[0] + (times - times[0]) * scale
    slowed_rates = rates / scale
    checked = judge_hermite_rates(names, knots, slowed_rates, slowed_times, limits)
    if checked["verdict"] != "PASS":
        raise HermiteRateError("free-space retiming did not satisfy exact rate limits")
    return slowed_times, slowed_rates, scale


def _finish_moves(request, specs, solved, notes, slow, solver, cell, held,
                  verifier_collision_model, verify_margin_m, progress):
    """Assemble `Move`s, verify them, and return them.

    Extracted so both free-space solvers share one reporting path: the
    representation changed, what a caller is handed did not.
    """
    out: list[Move] = []
    jobs = []
    reports = []
    kin = None if verifier_collision_model is None else verify_kinematics(request.cell)
    for m, (spec, s) in enumerate(zip(specs, solved, strict=True)):
        times, qdot, clock_scale = retime_rest_to_rest_wire(
            solver.names, s.knots, s.qdot, s.times_s,
            getattr(cell.profile, "limits", {}) or {})
        move_notes = list(notes[m])
        if clock_scale > 1.0:
            move_notes.append(
                f"exact rate retime {s.times_s[-1]:.6f} s -> {times[-1]:.6f} s")
        move = Move(kind=spec.kind, from_id=spec.from_id, to_id=spec.to_id,
                    knots=s.knots, knot_velocity=qdot, times_s=times,
                    clearance_m=s.clearance_m,
                    env_clearance_m=s.env_clearance_m,
                    self_clearance_m=s.self_clearance_m,
                    cost_before=s.cost_before, cost_after=s.cost_after,
                    seed=s.seed, detour=detour_of(s.knots, solver.vmax),
                    duration_before_s=slow[m], held=dict(held),
                    names=list(cell.moved_axes), notes=move_notes)

        if kin is not None:
            waypoints = np.zeros((len(s.knots), len(kin.axis_names)))
            rates = np.zeros_like(waypoints)
            order = {n: i for i, n in enumerate(solver.names)}
            for i, name in enumerate(kin.axis_names):
                if name in order:
                    waypoints[:, i] = s.knots[:, order[name]]
                    rates[:, i] = qdot[:, order[name]]
                elif name in held:
                    waypoints[:, i] = held[name]
            # T1 is a weld-process check. M6 marks it not applicable, while the
            # emitted Hermite collision and rate checks remain mandatory.
            span_times = getattr(s, "span_times_s", None)
            if span_times is not None:
                span_times = times[0] + (span_times - s.times_s[0]) * clock_scale
            keep = certified_samples(times, span_times)
            report = report_samples(len(times), keep)
            reports.append(report)
            jobs.append((spec.kind, request.cell, verifier_collision_model, waypoints[keep],
                         {"limits": getattr(cell.profile, "limits", {}) or {}},
                         times[keep], rates[keep], verify_margin_m, waypoints[report]))
        out.append(move)

    if jobs:
        if progress is not None:
            progress("verify")
        # ONE pool for the whole batch. The judge is serial CPU -- 58k coal
        # queries at ~100 us for this plan's motions -- and the motions are
        # independent, so they fan out; but a pool per MOTION pays its own
        # startup back out (measured: workers 4 took 0.40 s to 0.34 s on one
        # motion). The geometry pickles in a millisecond, so the fan-out is
        # all win once the pool outlives one task.
        if len(jobs) > 1:
            # Fanned by MOTION, one pool for the whole batch, and measured
            # to be the right grain twice over: cutting motions into pieces
            # moved nothing (2.63 s -> 2.63 s on the 22-seam corpus -- the
            # pool was already full), and lending each motion's segments to
            # the pool (`verify`'s `pool=`) made it 2.7x WORSE, because its
            # work items carry the kinematics and geometry per SEGMENT and
            # twenty-three lenders are a pickle storm. The wall time here is
            # total verify CPU over the cores: ~420 ms serial per 161-sample
            # motion, of which the certify walk is 314 and the clearance
            # report 106.
            with ProcessPoolExecutor(
                    max_workers=min(len(jobs), os.cpu_count() or 1)) as pool:
                verdicts = list(pool.map(_judge, jobs))
        else:
            verdicts = [_judge(jobs[0])]
        for move, verdict, report in zip(out, verdicts, reports, strict=True):
            # The report read a subset of the wire; say where on the wire.
            if verdict.worst_at is not None and "waypoint" in verdict.worst_at:
                verdict.worst_at["waypoint"] = int(report[verdict.worst_at["waypoint"]])
            move.verdict = verdict
    return out


def _judge(args: tuple) -> Verdict:
    """One M6 verdict -- module-level so a worker process can be handed it."""
    motion, cell_doc, geometry, waypoints, profile, times_s, rates, margin_m, wire = args
    return verify_free_space(
        cell_doc,
        geometry,
        waypoints,
        motion=motion,
        profile=profile,
        times_s=times_s,
        velocities=rates,
        margin_m=margin_m,
        clearance_waypoints=wire,
    )


#: How many wire samples the clearance REPORT reads per motion, at most. The
#: report is the operator's "worst clearance" line and nothing passes or fails
#: on it (`verify.report_clearance`: every instant is the certificate's
#: business), and it costs a screened distance per body pair per sample --
#: read along the whole 10 ms wire it was 12.6 s of an 8-weld request's
#: serial verify against 2.3 s for the certificate (2026-09-17). 161 is the
#: count the wire had before it was sampled by time, which is the density the
#: report was read at when that figure was first published.
REPORT_SAMPLES = 161


def report_samples(count: int, certified: np.ndarray) -> np.ndarray:
    """Wire indices the clearance report reads: evenly spread, at most
    `REPORT_SAMPLES`, plus every sample the certificate kept."""
    spread = np.round(np.linspace(0, count - 1, min(count, REPORT_SAMPLES))).astype(int)
    return np.union1d(spread, certified)


#: How far a wire sample may sit past a span boundary and still count as on
#: it, as a share of the motion's duration. The samples are a `linspace` and
#: the boundaries a product, so a sample that IS a boundary can miss it by a
#: few ulps; the pieces either side agree there in value and two derivatives
#: (a cubic B-spline is C2 at a simple knot), so which side claims it moves
#: the certified curve by nothing measurable.
SPAN_TOL = 1e-12


def certified_samples(times_s: np.ndarray, span_times_s: np.ndarray | None) -> np.ndarray:
    """Which wire samples the certificate needs: indices into `times_s`.

    The controller replays the Hermite through consecutive wire samples, and
    the wire's `(q, qdot)` are the B-spline's own values -- so a wire interval
    that lies inside one of the curve's cubic pieces IS that piece, and so is
    the Hermite between the first and last wire samples of the piece: a cubic
    is fixed by its two end values and slopes. Only an interval that straddles
    a boundary is a different curve, and it stays as it is. Certifying the
    kept samples' Hermites is therefore certifying exactly the curve the
    controller runs, on ~30 segments instead of one per 10 ms: 7.7 s of the
    8-weld `bracket_b_2x` request's 9.2 s of free space was the verifier
    walking wire intervals (`tools/bench_plan.py`, 2026-09-17).

    Every sample when the solver states no pieces (the other backends).
    """
    count = len(times_s)
    if span_times_s is None or count < 3:
        return np.arange(count)
    times = np.asarray(times_s, dtype=np.float64)
    bounds = np.asarray(span_times_s, dtype=np.float64)
    tol = SPAN_TOL * max(float(times[-1] - times[0]), 1e-30)
    piece = np.searchsorted(bounds, times[:-1] + tol, side="right") - 1
    top = bounds[np.clip(piece + 1, 0, len(bounds) - 1)]
    inside = (piece >= 0) & (piece + 1 < len(bounds)) & (times[1:] <= top + tol)
    piece = np.where(inside, piece, -1)
    keep = np.ones(count, dtype=bool)
    same = (piece[1:] == piece[:-1]) & (piece[1:] >= 0)
    keep[1:-1] = ~same
    return np.nonzero(keep)[0]


def plan_move(
        request: PlanRequest, planner_collision_model, device, *,
        kind: FreeSpaceMotion,
        start, start_rate, end, end_rate, held: dict[str, float],
        from_id: str | None = None, to_id: str | None = None,
        via: list[np.ndarray] | None = None, route: np.ndarray | None = None,
        verifier_collision_model: Any = None,
        verify_margin_m: float = VERIFY_MARGIN_M,
        inner_steps: int = INNER_STEPS, outer_steps: int = OUTER_STEPS,
        retime_attempts: int = RETIME_ATTEMPTS,
        shortcut_levels: int = SHORTCUT_LEVELS, cell: Any = None) -> Move:
    """One motion, planned, shortened, retimed and judged: `plan_moves` for a
    caller who has one question."""
    spec = MoveSpec(kind=kind, from_id=from_id, to_id=to_id,
                    start=np.asarray(start, dtype=np.float64),
                    start_rate=np.asarray(start_rate, dtype=np.float64),
                    end=np.asarray(end, dtype=np.float64),
                    end_rate=np.asarray(end_rate, dtype=np.float64),
                    via=list(via or []), route=route)
    return plan_moves(request, planner_collision_model, device, specs=[spec],
                      held=held, verifier_collision_model=verifier_collision_model,
                      verify_margin_m=verify_margin_m, inner_steps=inner_steps,
                      outer_steps=outer_steps, retime_attempts=retime_attempts,
                      shortcut_levels=shortcut_levels, cell=cell)[0]


def unplanned_welds(seams: list[dict[str, Any]], planned: list[Any]) -> list[str]:
    """The program's welds that no planned report stands for, in program order."""
    have = {str(report.seam_id) for report in planned}
    return [str(seam.get("id")) for seam in seams if str(seam.get("id")) not in have]


def connect(request: PlanRequest, reports: list[Any], *, planner_collision_model, device,
            roadmap: Roadmap | None = None,
            verifier_collision_model: Any = None, verify_margin_m: float = VERIFY_MARGIN_M,
            inner_steps: int = INNER_STEPS,
            outer_steps: int = OUTER_STEPS,
            retime_attempts: int = RETIME_ATTEMPTS,
            shortcut_levels: int = SHORTCUT_LEVELS,
            backend: str | None = None,
            cell: Any = None, progress: Any = None) -> list[Move]:
    """Every move BETWEEN the welds, in plan order.

    `reports` are M5's, in the order they are to be executed. What comes back
    interleaves with them: approach, seam, transit, seam, ..., retract -- and
    each motion's endpoints are the neighbouring seams' own first and last
    knots in POSITION, at REST in velocity, with a held stop on the
    weld-touching side (`PIN_KNOTS`). The handover to a weld is a stop --
    the arc starts and ends on a still torch -- not a C1 splice onto the
    weld's own clock.

    Every motion is planned in ONE batch (`plan_moves`): the endpoints are all
    known before anything is solved -- a motion's ends are the neighbouring
    welds', not the neighbouring MOTIONS' -- so nothing here is sequential, and
    solved one at a time the join was launch-bound at batch ~10
    (`BENCHMARKS.md` 12.5). Only the roadmap queries and the standoff IK stay
    per motion, at setup: milliseconds beside the solve.

    This joins the reports it is HANDED and does not rewind them. Callers that
    want rewinding run `rewound` first and must carry ITS reports downstream too
    -- the CLI below does.
    """
    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    if not cell.profile.reset_pose:
        raise FreeSpaceError(
            f"cell {request.cell.get('id')!r} states no reset pose; there is "
            "nowhere for an approach to start (see `cell/profile.py`)")
    home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])
    rest = np.zeros_like(home)
    held = dict(cell.profile.held)

    ordered = [r for r in reports if len(r.knots) >= 2]
    if not ordered:
        return []

    def route_for(start, end):
        return (roadmap_path(roadmap, planner_collision_model, device,
                             start, end, held=held, attach=REWIND_ATTACH)
                if roadmap is not None else None)

    # The standoff waypoints come from the pose at the WELD end of the motion.
    # The reset pose has no electrode axis to back off from, so nothing is
    # tried there. Every seam's first and last knot in ONE batched IK call --
    # solved endpoint by endpoint this was half the join (`standoff_batch`).
    table = standoff_batch(request,
                           [r.knots[0] for r in ordered]
                           + [r.knots[-1] for r in ordered], device, cell=cell)
    arrive = table[:len(ordered)]
    depart = table[len(ordered):]

    # The moves are the PROGRAM's, not this stage's. It used to read the weld
    # count and produce one approach, one transit per gap and one retract from
    # it, which is the last place a planner invented motion nobody asked for:
    # D23 made the alternation the validity test and D25 took the inventing
    # away. What is left here is planning the moves it is handed.
    # A weld with no path first. Its moves are stated -- the program is fine --
    # but they have no end to start or finish at, and comparing the move list
    # against only the welds that DID plan blamed the list: "the program states
    # approach, transit, transit, transit, retract between 2 welds" about four
    # welds of which two crossed nothing.
    missing = unplanned_welds(request.seams, ordered)
    if missing:
        raise FreeSpaceError(
            f"{len(missing)} of {len(request.seams)} welds have no path "
            f"({', '.join(missing)}), so the moves to and from them have no end "
            "to plan between"
        )

    stated = [
        node.get("op") for node in request.freespace_nodes
        if node.get("op") in ("approach", "transit", "retract")
    ]
    wanted = ["approach"] + ["transit"] * (len(ordered) - 1) + ["retract"]
    if stated != wanted:
        raise FreeSpaceError(
            "the program states "
            + (", ".join(stated) or "no moves")
            + f" between {len(ordered)} welds, and this stage plans the moves it is "
            "given rather than the moves it would have chosen: "
            + ", ".join(wanted)
        )

    # Every end is a held STOP -- the weld-side ends because the arc starts and
    # stops on a still torch, and the HOME ends because the arm leaves the reset
    # pose stationary and comes back stationary. Home used to be a stop by
    # declaration alone (`velocities` wrote REST over whatever the knots did),
    # which read as rest and was not: on the control-point solver, whose rates
    # are the curve's own, the approach left home at 0.28 rad/s.
    # M5's `knot_velocity` is NOT imposed on the motions any more: the arc
    # starts and stops on a still torch, and the weld keeps its own clock.
    specs = [MoveSpec("approach", None, ordered[0].seam_id,
                      home, rest, ordered[0].knots[0], rest,
                      via=arrive[0],
                      route=route_for(home, ordered[0].knots[0]),
                      pin_start=True, pin_end=True)]
    # Transits get BOTH ends' standoffs as separate seeds -- one that leaves up
    # the electrode, one that arrives down it -- which only add candidates to
    # rank.
    for i, (before, after) in enumerate(pairwise(ordered)):
        specs.append(MoveSpec(
            "transit", before.seam_id, after.seam_id,
            before.knots[-1], rest,
            after.knots[0], rest,
            via=depart[i] + arrive[i + 1],
            route=route_for(before.knots[-1], after.knots[0]),
            pin_start=True, pin_end=True))
    specs.append(MoveSpec("retract", ordered[-1].seam_id, None,
                          ordered[-1].knots[-1], rest,
                          home, rest, via=depart[-1],
                          route=route_for(ordered[-1].knots[-1], home),
                          pin_start=True, pin_end=True))

    return plan_moves(request, planner_collision_model, device, specs=specs,
                      backend=backend,
                      held=held, verifier_collision_model=verifier_collision_model,
                      verify_margin_m=verify_margin_m, inner_steps=inner_steps,
                      outer_steps=outer_steps, retime_attempts=retime_attempts,
                      shortcut_levels=shortcut_levels, cell=cell,
                      progress=progress)


def move_json(move: Move) -> dict[str, Any]:
    """One free-space move, on the wire — `weld_trajopt.trajectory_json`'s shape.

    The knots and their velocities, in degrees: the pair IS the Hermite, so a
    reader that interpolates it plays the move the planner emitted. `axis_names`
    and `held` ride along so the motion is playable on its own. Both clearances are
    kept apart -- "the torch is in the plate" and "the forearm is in the wrist"
    are different failures with different fixes.
    """
    return {
        "kind": move.kind,
        "from": move.from_id,
        "to": move.to_id,
        "axis_names": list(move.names),
        "held": {k: float(np.rad2deg(v)) for k, v in move.held.items()},
        **hermite_wire_degrees(
            move.knots,
            move.knot_velocity,
            move.times_s,
        ),
        "duration_s": move.duration_s,
        "seed": move.seed,
        "clearance_mm": {
            "env": (None if not np.isfinite(move.env_clearance_m)
                    else round(move.env_clearance_m * 1000.0, 2)),
            "self": (None if not np.isfinite(move.self_clearance_m)
                     else round(move.self_clearance_m * 1000.0, 2)),
        },
        "verdict": None if move.verdict is None else move.verdict.summary(),
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--seams", type=int, default=3,
                        help="how many seams to join, in plan order")
    parser.add_argument("--verify-margin-mm", type=float, default=VERIFY_MARGIN_M * 1000.0)
    parser.add_argument("--samples", type=int, default=roadmap_samples,
                        help="roadmap samples; 0 plans without one")
    parser.add_argument("--no-rewind", action="store_true",
                        help="command the welds on the turn M4 chose, detour and all")
    parser.add_argument("--retime", type=int, default=RETIME_ATTEMPTS,
                        help="horizon-shrinking attempts per motion; 0 keeps the "
                             "closed-form duration `duration_for` picked")
    parser.add_argument("--shortcut-levels", type=int, default=SHORTCUT_LEVELS,
                        help="coarse-to-fine straightening levels; 0 flies the "
                             "seed's own shape, detour and all")
    args = parser.parse_args(argv)

    from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
    from weld_motion_planner.planner.planner_main import (
        MotionPlanningOptions,
        plan_motion,
    )
    from weld_motion_planner.planner.trajectory_optimization.engine import (
        roadmap as roadmap_module,
    )
    from weld_motion_planner.planner.trajectory_optimization.options import (
        WeldTrajoptOptions,
    )
    from weld_motion_planner.robot_cell.collision import model as collision_model
    from weld_motion_planner.verifier import model as verifier_model

    request = read_request(args.request)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    reports = plan_motion(
        request,
        MotionPlanningOptions(run_weld_trajopt=True, run_connecting_trajopt=False,
                              seam_search=SeamSearchOptions(k_best=1),
                              weld_trajopt=WeldTrajoptOptions(verify=False)),
        device=device).weld_trajectories[: args.seams]
    cell = RobotCell.of(request.cell, device=device)
    planner_collision_model = collision_model.for_planner(request, cell)
    verifier_collision_model = verifier_model.for_verifier(request)
    home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])

    # The roadmap is built FIRST and over every winding still on the table
    # (`winding_reach`): a box drawn around the turns already chosen could not see
    # the alternative it exists to find.
    graph = None
    if args.samples:
        graph = roadmap_module.build(
            request, planner_collision_model, device,
            held=dict(cell.profile.held),
            focus=winding_reach(request, reports, home, cell=cell),
            samples=args.samples, cell=cell)
        print("roadmap:", graph.summary())

    if not args.no_rewind:
        turned = rewound(request, reports, home, roadmap=graph,
                         planner_collision_model=planner_collision_model,
                         device=device, held=dict(cell.profile.held), cell=cell)
        for before, after in zip(reports, turned, strict=True):
            delta = np.rad2deg(after.knots[0] - before.knots[0]).round(0)
            if np.abs(delta).max() > 1e-9:
                print(f"rewound {before.seam_id}: "
                      + ", ".join(f"{n}{int(d):+d}"
                                  for n, d in zip(cell.moved_axes, delta) if d))
        reports = turned

    moves = connect(request, reports,
                    planner_collision_model=planner_collision_model,
                    device=device, roadmap=graph,
                    verifier_collision_model=verifier_collision_model,
                    verify_margin_m=args.verify_margin_mm / 1000.0,
                    retime_attempts=args.retime,
                    shortcut_levels=args.shortcut_levels, cell=cell)
    total = sum(move.duration_s for move in moves)
    weld = sum(float(r.times_s[-1] - r.times_s[0]) for r in reports)
    for item in moves:
        verdict = item.verdict.summary() if item.verdict else {}
        print(f"{item.kind:9s} {item.from_id or 'reset'!s:11s} -> "
              f"{item.to_id or 'reset'!s:11s} "
              f"{len(item.knots):3d} knots  "
              f"{item.duration_before_s:5.2f} -> {item.duration_s:5.2f} s  "
              f"detour {item.detour:4.2f}  "
              f"env {item.env_clearance_m * 1000:6.1f} mm  "
              f"self {item.self_clearance_m * 1000:6.1f} mm  "
              f"cost {item.cost_before:7.2f} -> {item.cost_after:6.2f}  "
              f"seed {item.seed:12s} {verdict.get('verdict', '-')}"
              f" pen {verdict.get('penetrating', '-')}")
    print(f"\n{len(reports)} seams welded in {weld:.1f} s, "
          f"{len(moves)} moves between welds in {total:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
