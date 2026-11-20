#!/usr/bin/env python
"""Choosing the angles: a lattice, a dynamic program, and no OCCT anywhere.

Given a table of "how much arc is clear at this position with these angles",
this picks the work and travel profile that welds the most of the seam. The
table comes from `clearance.py`; nothing here knows how it was measured, which
is the same discipline `curve.py` keeps -- OCCT is an oracle, not a dependency
-- and it is why this module's tests run in milliseconds on hand-built
lattices instead of minutes on a B-rep.

## The objective is lexicographic, not weighted

    J1   + mm of clear arc
    J2   - deg-mm away from the authored profile
    J3   - total variation of the profile

Compared as a tuple, in that order. There is no epsilon to tune and no way for
a thousand millimetres of smoothness to buy one millimetre of coverage.

This is sound as a dynamic program for a reason worth writing down: **lexicographic
order on R^k is translation invariant** -- if x > y then x + z > y + z -- so the
best prefix reaching a state stays best after any common suffix, which is
exactly the exchange argument Bellman optimality needs. A weighted sum would
also work, but only after someone picked the weights, and nobody can say what a
degree of deviation is worth in millimetres of weld.

## Two grids, and why they are not the same grid

The clearance oracle samples every 2 mm, because a 3 mm plate seen edge-on
between two samples is a collision nobody reported. The DECISION grid is much
coarser -- a knot every 10 mm by default -- and that is not an optimisation, it
is what makes the rate limit mean anything:

    rate 0.5 deg/mm x  2 mm step  =  1.0 deg  = 0.5 of a 2 deg work bin
    rate 0.5 deg/mm x 10 mm step  =  5.0 deg  = 2.5 work bins

At the oracle's own spacing the limit permits less than one bin, so no
transition is ever legal, the path is frozen at the authored profile and the
optimiser looks broken while reporting success. Angles are smooth things; they
do not need a knot every 2 mm. Each decision segment's cell value is the arc
cleared over all the oracle samples it covers, so coverage is still measured at
2 mm.

## What the bins are

Anchored on zero -- `k * step` for integer k inside the band -- so 0 is always
representable and a symmetric band gives an odd, symmetric set. Work at 2 deg
because the measured admissible window on a 90 degree fillet is about +/-6
degrees: a cliff, not a landscape, and a 10 degree bin steps straight over it.
Travel at 5 deg, where the geometry is far less sensitive.

The admissible band is a **hard mask** on the bins, never a penalty. A profile
outside its limits is not a worse weld, it is a different weld, and an
optimiser that could buy its way out of a declared limit with coverage would be
answering a question nobody asked.

## What is NOT here

Standoff. It is an outer loop in the caller, not a third lattice axis: stickout
is held constant along a weld -- varying it varies the current -- so a profile
that ramps it is not a procedure anyone runs. Computationally that makes it a
handful of independent runs over slices of one table rather than a dimension
multiplying it.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from typing import Any

SCHEMA = "amr-weld-planner-v1.orientation-plan.v1"

# Bin widths. Work is fine because the admissible window is narrow; travel is
# coarse because the geometry barely notices it.
DEFAULT_WORK_BIN_DEG = 2.0
DEFAULT_TRAVEL_BIN_DEG = 5.0

# Distance between decision knots. See the module docstring: at the oracle's own
# 2 mm this would make every rate limit prohibitive.
DEFAULT_KNOT_SPACING_MM = 10.0

# What a millimetre of stickout is worth in degrees of angle, so J2 can compare
# them. Mirrors `settings.optimizer.standoff_cost_deg_per_mm`, which is what
# actually reaches this module -- the constant here is the floor for a caller
# that has no settings, not a second opinion about the value.
DEFAULT_STANDOFF_COST_DEG_PER_MM = 3.0

# Ties in the objective are broken toward the LOWEST bin index, deterministically,
# so two runs over the same table give the same path. Compared with a strict >,
# never >=, which is what makes that true.
_EPS = 1e-9


class OrientError(ValueError):
    """A lattice this optimiser refuses to search, with the reason."""


def bin_values(lo: float, hi: float, step: float) -> list[float]:
    """Bin centres inside [lo, hi], on the grid anchored at zero.

    Anchoring on zero rather than on `lo` is what guarantees 0 is representable
    and that a band symmetric about 0 produces a symmetric, odd-sized set. A
    grid starting at `lo` would put the bins at -14.7, -12.7, ... for a band of
    -14.7 degrees, and "work angle 0" -- the overwhelmingly common answer --
    would not be among the choices.
    """
    if step <= 0:
        raise OrientError(f"bin step must be positive, got {step}")
    if hi < lo:
        raise OrientError(f"empty band: min {lo} > max {hi}")
    first = math.ceil(lo / step - _EPS)
    last = math.floor(hi / step + _EPS)
    if last < first:
        # The band is narrower than one bin and falls between two of them.
        # Snapping to the nearest bin would silently move outside a declared
        # limit, so the caller is told instead.
        raise OrientError(
            f"the band [{lo}, {hi}] contains no multiple of {step}; "
            "widen it or use a finer bin"
        )
    return [k * step for k in range(first, last + 1)]


def optimise(
    *,
    clear_mm: list[list[list[float]]],
    work_bins: list[float],
    travel_bins: list[float],
    arc_mm: list[float],
    spacing_mm: list[float],
    authored_work: list[float],
    authored_travel: list[float],
    work_limits: dict[str, Any] | None = None,
    travel_limits: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """The best path through the lattice, lexicographically.

    `clear_mm[i][a][b]` is millimetres of clear arc in decision segment `i`
    when the work angle is `work_bins[a]` and the travel angle is
    `travel_bins[b]`. `arc_mm[i]` is how much arc that segment speaks for, and
    `spacing_mm[i]` the distance from knot `i` to knot `i+1`.

    `authored_*` is the profile to stay near when coverage ties. It must be the
    profile UNDERNEATH any previous optimiser output -- pass a previous result
    back in and each run drifts toward the last one rather than toward what a
    person actually authored.
    """
    n = len(clear_mm)
    if n == 0:
        raise OrientError("nothing to optimise: the lattice has no segments")
    for name, seq, want in (
        ("arc_mm", arc_mm, n),
        ("authored_work", authored_work, n),
        ("authored_travel", authored_travel, n),
        ("spacing_mm", spacing_mm, max(0, n - 1)),
    ):
        if len(seq) != want:
            raise OrientError(f"{name} has {len(seq)} entries, expected {want}")

    allowed = [
        _allowed_bins(work_bins, work_limits, i, "work") for i in range(n)
    ]
    allowed_t = [
        _allowed_bins(travel_bins, travel_limits, i, "travel") for i in range(n)
    ]
    rate_w = _rate_of(work_limits)
    rate_t = _rate_of(travel_limits)

    def step_cost(i: int, a: int, b: int) -> tuple[float, float, float]:
        deviation = arc_mm[i] * (
            abs(work_bins[a] - authored_work[i]) + abs(travel_bins[b] - authored_travel[i])
        )
        return (clear_mm[i][a][b], -deviation, 0.0)

    # best[(a, b)] -> (objective tuple, back-pointer)
    best: dict[tuple[int, int], tuple[tuple[float, float, float], tuple[int, int] | None]] = {}
    for a in allowed[0]:
        for b in allowed_t[0]:
            best[(a, b)] = (step_cost(0, a, b), None)
    trail: list[dict[tuple[int, int], tuple[int, int] | None]] = [
        {state: None for state in best}
    ]

    for i in range(1, n):
        gap = spacing_mm[i - 1]
        reach_w = None if rate_w is None else rate_w * gap + _EPS
        reach_t = None if rate_t is None else rate_t * gap + _EPS
        nxt: dict[tuple[int, int], tuple[tuple[float, float, float], tuple[int, int] | None]] = {}
        for a in allowed[i]:
            for b in allowed_t[i]:
                here = step_cost(i, a, b)
                champion: tuple[float, float, float] | None = None
                from_state: tuple[int, int] | None = None
                for (pa, pb), (score, _) in best.items():
                    move_w = abs(work_bins[a] - work_bins[pa])
                    move_t = abs(travel_bins[b] - travel_bins[pb])
                    if reach_w is not None and move_w > reach_w:
                        continue
                    if reach_t is not None and move_t > reach_t:
                        continue
                    total = (
                        score[0] + here[0],
                        score[1] + here[1],
                        score[2] - move_w - move_t,
                    )
                    # Strict, so the first predecessor in iteration order wins a
                    # tie and the answer is reproducible.
                    if champion is None or total > champion:
                        champion = total
                        from_state = (pa, pb)
                if champion is not None:
                    nxt[(a, b)] = (champion, from_state)
        if not nxt:
            raise OrientError(
                f"no path reaches segment {i}: the rate limit is tighter than the "
                f"bins can express over {gap:.1f} mm. Widen the limit, or space the "
                "decision knots further apart."
            )
        best = nxt
        trail.append({state: value[1] for state, value in nxt.items()})

    end = max(best.items(), key=lambda kv: kv[1][0])
    # `max` is not enough on its own: it returns the first maximum, and dict
    # order here is insertion order over ascending bins, so the tie-break is the
    # lowest work bin then the lowest travel bin. Deterministic by construction.
    state: tuple[int, int] | None = end[0]
    objective = end[1][0]

    path: list[tuple[int, int]] = []
    for i in range(n - 1, -1, -1):
        assert state is not None
        path.append(state)
        state = trail[i][state]
    path.reverse()

    return {
        "work_deg": [work_bins[a] for a, _ in path],
        "travel_deg": [travel_bins[b] for _, b in path],
        "work_bin_index": [a for a, _ in path],
        "travel_bin_index": [b for _, b in path],
        "clear_mm": round(objective[0], 9),
        "deviation_deg_mm": round(-objective[1], 9),
        "variation_deg": round(-objective[2], 9),
    }


def _rate_of(limits: dict[str, Any] | None) -> float | None:
    if not limits:
        return None
    rate = limits.get("max_rate_deg_per_mm")
    if rate is None:
        return None
    if not isinstance(rate, int | float) or isinstance(rate, bool) or float(rate) < 0:
        raise OrientError(f"max_rate_deg_per_mm must be a non-negative number, got {rate!r}")
    return float(rate)


def _allowed_bins(
    values: list[float], limits: dict[str, Any] | None, index: int, axis: str
) -> list[int]:
    """Bin indices inside the band at this segment. A hard mask, not a penalty."""
    lo = _at(limits, "min", index, -math.inf)
    hi = _at(limits, "max", index, math.inf)
    if lo > hi:
        raise OrientError(f"{axis} limits cross at segment {index}: min {lo} > max {hi}")
    out = [i for i, v in enumerate(values) if lo - _EPS <= v <= hi + _EPS]
    if not out:
        raise OrientError(
            f"{axis} band [{lo}, {hi}] at segment {index} admits none of the "
            f"{len(values)} bins; widen the limit or use a finer bin"
        )
    return out


def _at(limits: dict[str, Any] | None, key: str, index: int, fallback: float) -> float:
    if not limits or key not in limits or limits[key] is None:
        return fallback
    series = limits[key]
    if isinstance(series, int | float):
        return float(series)
    return float(series[index]) if index < len(series) else fallback


# --------------------------------------------------------------------------
# lattice path -> a scalar function the contract can carry
# --------------------------------------------------------------------------


def simplify(fractions: list[float], values: list[float], tolerance: float) -> list[int]:
    """Douglas-Peucker over (fraction, value). Returns the indices to keep.

    The deviation measured is **vertical, in degrees** -- not the perpendicular
    distance the textbook algorithm uses. That is not a shortcut, it is the only
    measure that means anything here: `x` is a dimensionless fraction and `y` is
    degrees, so a perpendicular distance in that plane is a length in no unit at
    all, and dividing by the chord's own length makes the effective tolerance
    depend on the chord's slope.

    It cost four percentage points of coverage on the first real seam this was
    run against. A travel path of [20, 10, 10, 10, 10, 10, 10, 0, 10] degrees
    collapsed to a straight 20 -> 10 ramp under a 2.5 degree tolerance, because
    the perpendicular distance at the first knot came out as 0.87 -- the chord
    fell 10 degrees over a unit of `x`, so its length was 10.05 and every real
    deviation was divided by it. Vertically that same knot is 8.75 degrees off
    the chord, which is what an operator would see and what the torch would do.

    The endpoints are always kept, so a profile still spans s=0 to s=1 -- which
    the schema requires and a consumer assumes.
    """
    if len(fractions) < 3:
        return list(range(len(fractions)))
    keep = {0, len(fractions) - 1}

    def walk(lo: int, hi: int) -> None:
        if hi <= lo + 1:
            return
        x0, y0, x1, y1 = fractions[lo], values[lo], fractions[hi], values[hi]
        span = x1 - x0
        worst, at = 0.0, -1
        for i in range(lo + 1, hi):
            chord = y0 if span == 0 else y0 + (y1 - y0) * (fractions[i] - x0) / span
            d = abs(values[i] - chord)
            if d > worst:
                worst, at = d, i
        if worst > tolerance and at > 0:
            keep.add(at)
            walk(lo, at)
            walk(at, hi)

    walk(0, len(fractions) - 1)
    return sorted(keep)


def to_profile(
    fractions: list[float],
    values: list[float],
    tolerance: float,
    length_mm: float = 0.0,
    max_rate_deg_per_mm: float | None = None,
) -> dict[str, Any]:
    """A lattice path as a `scalar_function`, reduced to the knots that matter.

    Flat comes out as a `constant`, so the optimiser and the spline editor write
    the same document for the same shape -- otherwise a profile the operator
    never touched would show as edited the moment it was optimised.

    **Douglas-Peucker cannot break the rate limit**, which is worth stating
    because it looks like it should. The chord it draws between two kept knots
    has the AVERAGE slope of the steps it replaced:

        |v_j - v_i| = |sum of the steps| <= sum of |steps| <= rate * (s_j - s_i)

    so the merged segment is never steeper than the limit the path already
    satisfied. The check below is therefore an assertion about this code, not a
    correction applied to its output -- if it ever fires, the path handed in was
    already illegal.
    """
    if not fractions:
        raise OrientError("a profile needs at least one point")
    if max(values) - min(values) <= tolerance:
        mean = sum(values) / len(values)
        return {"kind": "constant", "value": round(mean, 6)}

    keep = simplify(fractions, values, tolerance)
    knots = [[round(fractions[i], 6), round(values[i], 6)] for i in keep]
    # Pin the ends: the schema requires s to run from 0 to 1, and a profile that
    # started at 0.06 would be silently extended by every consumer's clamp.
    knots[0][0] = 0.0
    knots[-1][0] = 1.0

    if max_rate_deg_per_mm is not None and length_mm > 0:
        for (s0, v0), (s1, v1) in pairwise(knots):
            span = (s1 - s0) * length_mm
            if span > 0 and abs(v1 - v0) / span > max_rate_deg_per_mm + 1e-6:
                raise OrientError(
                    f"simplified profile changes at {abs(v1 - v0) / span:.4f} deg/mm "
                    f"between s={s0} and s={s1}, over the limit of "
                    f"{max_rate_deg_per_mm}. The path handed in already broke it."
                )
    return {"kind": "piecewise_linear", "knots": knots}


# --------------------------------------------------------------------------
# the outer loop: standoff, and the profiles that come out
# --------------------------------------------------------------------------


def _bin_width(values: list[float]) -> float:
    return min((b - a for a, b in pairwise(values)), default=0.0)


def _nearest(values: list[float], target: float) -> int:
    return min(range(len(values)), key=lambda i: (abs(values[i] - target), i))


def mirror_clear(
    clear_mm: list[list[list[float]]], work_bins: list[float], travel_bins: list[float]
) -> list[list[list[float]]]:
    """The same table read as if the seam were welded the other way round.

    Reversing travel negates the tangent, and `apply_angles` turns that into
    exactly

        u(-t, work, travel) == u(t, -work, -travel)

    -- an identity, verified to zero error over random frames rather than
    argued from the algebra. The torch points somewhere different for the same
    two numbers, and somewhere IDENTICAL for their negations. So the reversed
    seam's clearance is this table indexed at the negated angles, and reversal
    costs no rays at all. The same trick standoff gets: a second outer loop
    over one oracle, not a second oracle.

    The bins must be symmetric about zero, or the negated angle is not in the
    table; `clearance.search_inputs(both_directions=True)` arranges that. It is
    checked rather than assumed, because a silently truncated mirror would
    return a table that is merely wrong instead of one that is missing.

    What does NOT mirror is the authored profile or the limits. Those are in
    the welder's numbers, and 10 degrees of drag is 10 degrees of drag whichever
    way round the seam is run -- the schema is explicit that reversing keeps
    positive travel meaning drag. So the DP compares like with like: same
    target, same window, a different table underneath.
    """
    for name, bins in (("work", work_bins), ("travel", travel_bins)):
        for lo, hi in zip(bins, reversed(bins)):
            if abs(lo + hi) > 1e-9:
                raise OrientError(
                    f"the {name} bins are not symmetric about zero, so a reversed seam's "
                    f"angles are not in the table; build the lattice with both_directions"
                )
    return [
        [list(reversed(row)) for row in reversed(knot)]
        for knot in clear_mm
    ]


def search(
    lattice: dict[str, Any],
    *,
    authored_work: list[float],
    authored_travel: list[float],
    authored_standoff_mm: float,
    work_limits: dict[str, Any] | None = None,
    travel_limits: dict[str, Any] | None = None,
    standoff_cost_deg_per_mm: float = DEFAULT_STANDOFF_COST_DEG_PER_MM,
    directions: tuple[str, ...] = ("forward",),
    authored_direction: str = "forward",
) -> dict[str, Any]:
    """One DP per candidate standoff, and the best answer as scalar functions.

    Standoff is an outer loop over slices of one table, which costs nothing:
    `clearance.build_lattice` already answered for every standoff out of the
    same rays. So this is N runs of a millisecond DP, not N oracles.

    **Its departure is priced, not free.** Electrode extension sets the
    resistive preheat of the wire -- a longer stickout drops current and
    penetration, and past roughly 20-25 mm the gas coverage goes -- so an
    optimiser that quietly added 6 mm to clear a gusset would have changed the
    weld, not just the way the torch got there. `standoff_cost_deg_per_mm`
    converts millimetres into the degrees J2 already speaks, and it is a
    declared settings field rather than a constant buried here: without it
    there is no way to state whether 1 mm of stickout is worth 2 degrees of
    work angle, and hiding the choice inside a tier ordering would not make it
    go away. The departure is also REPORTED, in millimetres, whatever it costs.

    The caller supplies the candidate standoffs by building the lattice with
    them; an empty band means one candidate and this degenerates to a single DP.
    """
    standoffs = list(lattice["standoffs_mm"])
    if not standoffs:
        raise OrientError("the lattice has no standoffs to choose between")
    if standoff_cost_deg_per_mm < 0:
        raise OrientError(
            f"standoff_cost_deg_per_mm must not be negative, got {standoff_cost_deg_per_mm}"
        )
    home = _nearest(standoffs, authored_standoff_mm)
    if abs(standoffs[home] - authored_standoff_mm) > 1e-6:
        # The authored standoff has to BE a candidate, or "never worse than what
        # was authored" is being measured against a weld nobody authored.
        raise OrientError(
            f"the authored standoff {authored_standoff_mm} mm is not among the "
            f"lattice's candidates {standoffs}"
        )

    work_bins = lattice["work_bins"]
    travel_bins = lattice["travel_bins"]
    arc_mm = lattice["arc_mm"]
    total_arc = sum(arc_mm)

    # What the authored profile is worth in this same table, so the comparison
    # is like with like. Snapped to the nearest bin the band allows, which is an
    # approximation -- and the reason the caller re-verifies the winner with the
    # fine fan rather than treating any lattice number as final.
    home_w = [_nearest(work_bins, v) for v in authored_work]
    home_t = [_nearest(travel_bins, v) for v in authored_travel]
    authored_clear = sum(
        lattice["clear_mm"][home][k][home_w[k]][home_t[k]] for k in range(len(arc_mm))
    )

    # Nearest-to-authored first, so an exact tie in the objective keeps the
    # standoff the operator asked for rather than whichever came first in a list.
    order = sorted(range(len(standoffs)), key=lambda d: (abs(standoffs[d] - authored_standoff_mm), d))

    # Direction is the other free outer loop, and free is meant literally: the
    # reversed table is the forward one re-indexed, so this costs a millisecond
    # DP per direction and no rays. Authored direction FIRST, so a tie leaves
    # the weld running the way the operator set it up -- flipping a weld to win
    # nothing is churn, and it is the kind of churn that is hard to notice in a
    # list of thirty.
    wanted = [authored_direction] + [d for d in directions if d != authored_direction]
    tables = {"forward": lattice["clear_mm"]}
    if "reverse" in wanted:
        tables["reverse"] = [
            mirror_clear(slice_mm, work_bins, travel_bins) for slice_mm in lattice["clear_mm"]
        ]

    # Per direction, and only the CHOSEN one is reported. `standoffs_tried` is
    # the standoff ladder -- what each candidate was worth -- and listing every
    # standoff twice, once per direction, would make it a different thing that
    # every reader has to filter first.
    tried_by_direction: dict[str, list[dict[str, Any]]] = {}
    per_direction: dict[str, float] = {}
    winners: dict[str, tuple[tuple[float, float, float], int, dict[str, Any]]] = {}
    for direction in wanted:
        table = tables[direction]
        tried = tried_by_direction.setdefault(direction, [])
        best: tuple[tuple[float, float, float], int, dict[str, Any]] | None = None
        for d in order:
            result = optimise(
                clear_mm=table[d],
                work_bins=work_bins,
                travel_bins=travel_bins,
                arc_mm=arc_mm,
                spacing_mm=lattice["spacing_mm"],
                authored_work=authored_work,
                authored_travel=authored_travel,
                work_limits=work_limits,
                travel_limits=travel_limits,
            )
            departure = abs(standoffs[d] - authored_standoff_mm)
            penalty = standoff_cost_deg_per_mm * departure * total_arc
            score = (
                result["clear_mm"],
                -(result["deviation_deg_mm"] + penalty),
                -result["variation_deg"],
            )
            previous = per_direction.get(direction)
            if previous is None or result["clear_mm"] > previous:
                per_direction[direction] = result["clear_mm"]
            tried.append(
                {
                    "standoff_mm": standoffs[d],
                    "departure_mm": round(departure, 6),
                    "clear_mm": result["clear_mm"],
                    "clear_fraction": (
                        round(result["clear_mm"] / total_arc, 6) if total_arc else 0.0
                    ),
                    "deviation_deg_mm": result["deviation_deg_mm"],
                    "standoff_penalty_deg_mm": round(penalty, 9),
                }
            )
            if best is None or score > best[0]:
                best = (score, d, result)
        assert best is not None
        winners[direction] = best

    # A FULL plan per direction, not just the winner's.
    #
    # The choice used to be made here, on the lattice's own numbers -- 17 rays
    # at 10 mm knots. That is the wrong resolution to decide it at: two
    # directions can tie coarsely and separate at the fine fan, and the tie
    # went to whichever was authored, so a genuinely better reverse could be
    # thrown away before anyone measured it properly.
    #
    # So both are emitted and the caller verifies both, from the same lattice
    # and the same config, and decides on what it measures. `chosen` here is
    # only a provisional pick so a caller that does not verify still gets an
    # answer.
    def assemble(direction: str) -> dict[str, Any]:
        _, index, path = winners[direction]
        tried_here = tried_by_direction[direction]
        for row in tried_here:
            row["chosen"] = row["standoff_mm"] == standoffs[index]
        tried_here.sort(key=lambda row: row["standoff_mm"])
        return {
            "direction": direction,
            "direction_changed": direction != authored_direction,
            "standoff_mm": standoffs[index],
            "standoff_departure_mm": round(abs(standoffs[index] - authored_standoff_mm), 6),
            "standoff_cost_deg_per_mm": standoff_cost_deg_per_mm,
            "work_angle_deg": to_profile(
                knots, path["work_deg"], _bin_width(work_bins) / 2.0,
                length_mm, _rate_of(work_limits),
            ),
            "travel_angle_deg": to_profile(
                knots, path["travel_deg"], _bin_width(travel_bins) / 2.0,
                length_mm, _rate_of(travel_limits),
            ),
            "knot_fractions": knots,
            "work_path_deg": path["work_deg"],
            "travel_path_deg": path["travel_deg"],
            "seam_length_mm": round(length_mm, 6),
            "clear_mm": path["clear_mm"],
            "clear_fraction": round(path["clear_mm"] / total_arc, 6) if total_arc else 0.0,
            "authored_clear_mm": round(authored_clear, 9),
            "authored_clear_fraction": (
                round(authored_clear / total_arc, 6) if total_arc else 0.0
            ),
            "deviation_deg_mm": path["deviation_deg_mm"],
            "variation_deg": path["variation_deg"],
            "standoffs_tried": tried_here,
        }

    length_mm = float(lattice["seam_length_mm"])
    knots = lattice["knot_fractions"]
    by_direction = {name: assemble(name) for name in wanted}

    # Provisional, on the coarse numbers, with the authored direction winning a
    # tie. A verifying caller overrules this; a caller that does not gets the
    # behaviour that was here before.
    provisional = max(
        wanted, key=lambda name: (round(winners[name][0][0], 6), name == authored_direction)
    )
    return {
        "schema": SCHEMA,
        **by_direction[provisional],
        # Every direction that was tried, each a complete plan. Reported whether
        # or not the answer changed: "we tried both and this way round won" and
        # "we never looked" are different facts.
        "by_direction": by_direction,
        "direction_clear_mm": {k: round(v, 6) for k, v in sorted(per_direction.items())},
    }


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def self_test() -> int:
    work = bin_values(-6.0, 6.0, 2.0)
    travel = [10.0]
    assert work == [-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0], work
    assert 0.0 in work, "zero must be representable"

    # A corridor that SWINGS: clear only at -4 and below early, only at +4 and
    # above later. Covering both ends means moving 8 degrees between them, so
    # the rate limit is what decides whether the coverage is reachable.
    #
    # A limit on the RATE bounds change, not value -- a profile parked at +4 the
    # whole way has zero rate and breaks nothing. That is why this fixture needs
    # two opposite corridors rather than one: with a single corridor the path
    # simply sits inside it and no rate limit ever binds.
    n = 5
    def cell(i: int, v: float) -> float:
        if i == 1:
            return 10.0 if v <= -4.0 else 0.0
        if i == 3:
            return 10.0 if v >= 4.0 else 0.0
        return 10.0

    clear = [[[cell(i, v)] for v in work] for i in range(n)]
    common = {
        "clear_mm": clear,
        "work_bins": work,
        "travel_bins": travel,
        "arc_mm": [10.0] * n,
        "spacing_mm": [10.0] * (n - 1),
        "authored_work": [0.0] * n,
        "authored_travel": [10.0] * n,
    }

    # 0.5 deg/mm over 10 mm knots is 5 degrees a step: -4 to +4 in two steps.
    loose = optimise(**common, work_limits={"max_rate_deg_per_mm": 0.5})
    assert loose["clear_mm"] == 50.0, loose
    assert loose["work_deg"][1] <= -4.0 and loose["work_deg"][3] >= 4.0, loose["work_deg"]

    # 0.1 deg/mm is 1 degree a step -- 4 degrees across the whole seam -- so both
    # corridors cannot be reached and exactly one segment is given up.
    tight = optimise(**common, work_limits={"max_rate_deg_per_mm": 0.1})
    assert tight["clear_mm"] == 40.0, tight
    assert loose["clear_mm"] > tight["clear_mm"], "the rate limit has to cost something"

    # A band is a hard mask: forbid the positive corridor and it is lost, however
    # much coverage it was worth.
    masked = optimise(
        **common,
        work_limits={"max": 0.0, "max_rate_deg_per_mm": 0.5},
    )
    assert masked["clear_mm"] == 40.0, masked
    assert max(masked["work_deg"]) <= 0.0, masked["work_deg"]

    # Ties go to the authored profile: with everything clear, nothing moves.
    flat_lattice = [[[10.0] for _ in work] for _ in range(n)]
    lazy = optimise(**{**common, "clear_mm": flat_lattice}, work_limits={"max_rate_deg_per_mm": 0.5})
    assert lazy["work_deg"] == [0.0] * n, lazy["work_deg"]
    assert lazy["deviation_deg_mm"] == 0.0 and lazy["variation_deg"] == 0.0, lazy

    # The standoff outer loop. Two candidates over one table: at 10 mm the torch
    # cannot reach the middle segment at any angle, at 14 mm it can.
    near = [[[0.0 if i == 2 else 10.0] for _ in work] for i in range(n)]
    far = [[[10.0] for _ in work] for _ in range(n)]
    lattice = {
        "knot_fractions": [i / (n - 1) for i in range(n)],
        "spacing_mm": [10.0] * (n - 1),
        "arc_mm": [10.0] * n,
        "seam_length_mm": 50.0,
        "work_bins": work,
        "travel_bins": travel,
        "standoffs_mm": [10.0, 14.0],
        "clear_mm": [near, far],
    }
    common_search = {
        "authored_work": [0.0] * n,
        "authored_travel": [10.0] * n,
        "authored_standoff_mm": 10.0,
        "work_limits": {"max_rate_deg_per_mm": 0.5},
    }
    dear = search(lattice, **common_search, standoff_cost_deg_per_mm=3.0)
    assert dear["standoff_mm"] == 14.0, "coverage is J1 -- no price buys it back"
    assert dear["clear_mm"] == 50.0 and dear["standoff_departure_mm"] == 4.0
    assert dear["authored_clear_mm"] == 40.0, "the baseline is the authored standoff"
    # The cost can only ever break a TIE in coverage, which is what makes it
    # safe to declare: it cannot trade weld for procedure. Give both standoffs
    # the same coverage and the priced run comes home.
    tie = {**lattice, "clear_mm": [far, far]}
    assert search(tie, **common_search, standoff_cost_deg_per_mm=3.0)["standoff_mm"] == 10.0
    assert search(tie, **common_search, standoff_cost_deg_per_mm=0.0)["standoff_mm"] == 10.0, (
        "an exact tie keeps the authored standoff"
    )
    assert [row["standoff_mm"] for row in dear["standoffs_tried"]] == [10.0, 14.0]
    assert sum(1 for row in dear["standoffs_tried"] if row["chosen"]) == 1

    flat = to_profile([0.0, 0.5, 1.0], [3.0, 3.0, 3.0], 1.0)
    assert flat == {"kind": "constant", "value": 3.0}, flat
    ramp = to_profile([0.0, 0.5, 1.0], [0.0, 5.0, 10.0], 0.5)
    assert ramp["kind"] == "piecewise_linear"
    assert ramp["knots"][0] == [0.0, 0.0] and ramp["knots"][-1] == [1.0, 10.0]
    assert len(ramp["knots"]) == 2, "a straight ramp needs no middle knot"

    print(
        "orient self-test ok: swing corridor covered "
        f"{loose['clear_mm']:.0f} mm, rate limit cost "
        f"{loose['clear_mm'] - tight['clear_mm']:.0f} mm, mask cost "
        f"{loose['clear_mm'] - masked['clear_mm']:.0f} mm"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--lattice", default="", help="JSON with the optimise() arguments")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.lattice:
        raise ValueError("one of --lattice or --self-test is required")

    from pathlib import Path

    problem = json.loads(Path(ns.lattice).read_text(encoding="utf-8"))
    print(json.dumps(optimise(**problem), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"orient: {exc}", file=sys.stderr)
        raise SystemExit(1)
