#!/usr/bin/env python
"""Scalar functions of normalized arc length: evaluating them, and checking them.

A weld parameter is rarely one number. Travel speed drops into a corner, the
work angle rolls as a fillet climbs a curved flange, current tapers into a
crater. The contract has said so since the first commit -- `scalar_function` is
a `oneOf` over constant, piecewise_linear, bspline and samples -- but nothing
in this module could evaluate one, and the UI only ever wrote constants.

Two jobs here:

**Evaluate.** One function, four kinds, `s` clamped to [0, 1]. The planner will
sample these at whatever density it chooses; R3 forbids this module from baking
that choice in, so nothing here samples anything by default.

**Check what JSON Schema cannot.** The schema says knots are "s strictly
increasing, first 0.0, last 1.0" -- in a *description*, because JSON Schema has
no way to express it. So a document with knots at [[0.5, 10], [0.2, 20]] passes
validation and then means whatever the reader guesses. `problems()` is where
that gets caught, and `server._validate` runs it over every scalar in a program.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from typing import Any

SCHEMA = "amr-weld-planner-v1.scalar.v1"

KINDS = ("constant", "piecewise_linear", "bspline", "samples")


def evaluate(fn: dict[str, Any], s: float) -> float:
    """The value at normalized arc length `s`, clamped to [0, 1]."""
    s = min(max(float(s), 0.0), 1.0)
    kind = fn.get("kind")

    if kind == "constant":
        return float(fn["value"])

    if kind == "piecewise_linear":
        knots = fn["knots"]
        if s <= knots[0][0]:
            return float(knots[0][1])
        for (s0, v0), (s1, v1) in pairwise(knots):
            if s <= s1:
                span = s1 - s0
                # A zero-width span is a step. Taking the right-hand value makes
                # the function right-continuous, which is the same convention
                # the span arithmetic elsewhere uses.
                return float(v1) if span <= 0.0 else float(v0 + (v1 - v0) * (s - s0) / span)
        return float(knots[-1][1])

    if kind == "samples":
        values = fn["values"]
        positions = fn["s"]
        interpolation = fn.get("interpolation", "linear")
        if s <= positions[0]:
            return float(values[0])
        for index in range(1, len(positions)):
            if s <= positions[index]:
                s0, s1 = positions[index - 1], positions[index]
                v0, v1 = values[index - 1], values[index]
                if interpolation == "step":
                    return float(v0)
                span = s1 - s0
                return float(v1) if span <= 0.0 else float(v0 + (v1 - v0) * (s - s0) / span)
        return float(values[-1])

    if kind == "bspline":
        return _bspline(fn, s)

    raise ValueError(f"unknown scalar function kind {kind!r}")


def _bspline(fn: dict[str, Any], s: float) -> float:
    """de Boor on a scalar-valued spline.

    Same algorithm as `curve.py` uses for points, in one dimension. Duplicated
    rather than shared because that one carries rational weights and homogeneous
    coordinates, and a parameter profile has neither.
    """
    degree = int(fn["degree"])
    knots = [float(k) for k in fn["knots_u"]]
    values = [float(v) for v in fn["control_values"]]
    lo, hi = knots[degree], knots[len(knots) - degree - 1]
    u = lo + (hi - lo) * s

    span = degree
    for index in range(degree, len(knots) - degree - 1):
        if u >= knots[index + 1]:
            span = index + 1
    span = min(span, len(values) - 1)

    d = [values[span - degree + i] for i in range(degree + 1)]
    for r in range(1, degree + 1):
        for i in range(degree, r - 1, -1):
            left = knots[i + span - degree]
            right = knots[i + 1 + span - r]
            alpha = 0.0 if right == left else (u - left) / (right - left)
            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]
    return float(d[degree])


def problems(fn: Any) -> list[str]:
    """What is wrong with this scalar function, in sentences. Empty means fine.

    Everything here is something the schema permits and a reader could not
    resolve: knots out of order, endpoints that do not span [0, 1], a control
    count that does not match the knot vector, a NaN.
    """
    if not isinstance(fn, dict):
        return ["not an object"]
    kind = fn.get("kind")
    if kind not in KINDS:
        return [f"unknown kind {kind!r}"]

    def finite(values: list[Any], what: str) -> list[str]:
        bad = [v for v in values if not isinstance(v, int | float) or not math.isfinite(float(v))]
        return [f"{what} contains a non-finite value"] if bad else []

    if kind == "constant":
        return finite([fn.get("value")], "value")

    if kind == "piecewise_linear":
        knots = fn.get("knots") or []
        found = finite([k[0] for k in knots], "knot positions")
        found += finite([k[1] for k in knots], "knot values")
        if found:
            return found
        positions = [float(k[0]) for k in knots]
        if positions[0] != 0.0 or positions[-1] != 1.0:
            found.append(
                f"knots must run from s=0 to s=1, not s={positions[0]} to s={positions[-1]}"
            )
        if any(b <= a for a, b in pairwise(positions)):
            # Not pedantry: the evaluator walks the list in order, so an
            # out-of-order knot silently makes a stretch of the seam
            # unreachable by any s.
            found.append(f"knot positions must strictly increase, got {positions}")
        return found

    if kind == "samples":
        positions = [float(v) for v in fn.get("s") or []]
        values = fn.get("values") or []
        found = finite(positions, "s") + finite(values, "values")
        if len(positions) != len(values):
            found.append(f"{len(positions)} positions but {len(values)} values")
        if positions and any(b < a for a, b in pairwise(positions)):
            found.append("sample positions must not decrease")
        return found

    degree = fn.get("degree", 0)
    knots = fn.get("knots_u") or []
    values = fn.get("control_values") or []
    found = finite(knots, "knots_u") + finite(values, "control_values")
    if len(knots) != len(values) + degree + 1:
        found.append(
            f"a degree {degree} spline over {len(values)} control values needs"
            f" {len(values) + degree + 1} knots, got {len(knots)}"
        )
    return found


# How finely a profile is sampled when its shape does not admit an exact
# answer. 401 points over a seam is finer than any profile anyone authors by
# hand and finer than the optimizer's own 2 mm lattice, so a violation this
# misses is smaller than the thing that produced the profile.
_SAMPLE_COUNT = 401


def breakpoints(fn: dict[str, Any]) -> list[float]:
    """The positions where a profile can turn. Where extremes live.

    A piecewise linear function's extreme relative to another piecewise linear
    function is always at one of their breakpoints, so checking a band this way
    is EXACT rather than sampled -- which matters, because a limit check that
    only samples will eventually pass a profile that pokes out between two
    sample points and then fail somewhere downstream that does not sample.
    """
    kind = fn.get("kind")
    if kind == "piecewise_linear":
        return [float(k[0]) for k in fn.get("knots") or []]
    if kind == "samples":
        return [float(s) for s in fn.get("s") or []]
    if kind == "bspline":
        knots = [float(k) for k in fn.get("knots_u") or []]
        degree = int(fn.get("degree", 1))
        if len(knots) <= 2 * degree:
            return []
        lo, hi = knots[degree], knots[len(knots) - degree - 1]
        width = hi - lo
        return sorted({(k - lo) / width for k in knots if lo <= k <= hi}) if width > 0 else []
    return []


def _is_piecewise(fn: dict[str, Any]) -> bool:
    """Is this function linear between its breakpoints?

    Only these admit an exact answer; everything else is sampled and says so.
    """
    return fn.get("kind") == "constant" or (
        fn.get("kind") == "piecewise_linear"
        or (fn.get("kind") == "samples" and fn.get("interpolation", "linear") == "linear")
    )


def _check_positions(*fns: dict[str, Any] | None) -> list[float]:
    positions = {0.0, 1.0}
    exact = True
    for fn in fns:
        if not isinstance(fn, dict):
            continue
        positions.update(b for b in breakpoints(fn) if 0.0 <= b <= 1.0)
        exact = exact and _is_piecewise(fn)
    if not exact:
        positions.update(i / (_SAMPLE_COUNT - 1) for i in range(_SAMPLE_COUNT))
    return sorted(positions)


def max_rate_deg_per_mm(fn: dict[str, Any], length_mm: float) -> tuple[float, bool]:
    """`(steepest |d value / d arc length|, is it exact)`.

    **Per millimetre, not per unit s.** `s` is normalized, so the same number
    would mean rates eighty times apart on a 10 mm seam and an 800 mm one, and
    merging two pieces into one weld would silently change what a limit means.

    Exact for anything linear between its breakpoints; sampled, and flagged as
    such, for a spline or a stepped sample set.
    """
    if length_mm <= 0.0:
        return 0.0, False
    kind = fn.get("kind")
    if kind == "constant":
        return 0.0, True

    if kind == "samples" and fn.get("interpolation") == "step":
        # A step has infinite rate at the step, and saying anything else would
        # be a comfortable lie about a profile that jumps.
        values = [float(v) for v in fn.get("values") or []]
        return (math.inf if len(set(values)) > 1 else 0.0), True

    positions = _check_positions(fn)
    values = [evaluate(fn, s) for s in positions]
    worst = 0.0
    for (s0, v0), (s1, v1) in pairwise(list(zip(positions, values, strict=True))):
        gap = (s1 - s0) * length_mm
        if gap > 0.0:
            worst = max(worst, abs(v1 - v0) / gap)
    return worst, _is_piecewise(fn)


def restriction_problems(fn: dict[str, Any]) -> list[str]:
    """Why `restrict` could not narrow this, if it could not.

    Empty for everything the UI authors. A `bspline` is the one kind that comes
    back unchanged, and a profile silently left stretched across a cut weld is
    exactly the failure `restrict` exists to prevent -- so it is named rather
    than left to be discovered.
    """
    kind = fn.get("kind")
    if kind == "bspline":
        return [
            (
                "a bspline profile cannot be narrowed onto a shorter arc without "
                "knot insertion, so it is carried across whole and now spans the "
                "piece rather than the original weld"
            )
        ]
    if kind == "samples":
        return [
            (
                "a sampled profile is carried across whole and now spans the piece "
                "rather than the original weld: with step interpolation it is "
                "discontinuous, and narrowing it by resampling lands on the wrong "
                "side of a jump"
            )
        ]
    return []


def restrict(fn: dict[str, Any], lo: float, hi: float) -> dict[str, Any]:
    """The same function over a SUB-RANGE, renormalised back onto [0, 1].

    `restrict(f, lo, hi)(x) == f(lo + x * (hi - lo))`.

    Needed the moment a weld is split. A profile is a function of fraction
    along the arc it belongs to, so handing a parent's profile to a half-length
    child does not keep the angles -- it COMPRESSES the whole ramp into half the
    metal. A 20 to 10 degree sweep over 110 mm silently becomes two 20 to 10
    sweeps over 55 mm each, which is a different weld that still looks like a
    profile.

    `bspline` and `samples` come back unchanged rather than resampled: quietly
    turning one into a piecewise_linear approximation would change the
    document's own statement about what it is.
    """
    kind = fn.get("kind")
    span = float(hi) - float(lo)
    if kind == "constant":
        return dict(fn)
    if abs(span) < 1e-12:
        return {"kind": "constant", "value": evaluate(fn, lo)}
    if kind != "piecewise_linear":
        # A bspline or a samples table. Both come back WHOLE, and
        # `restriction_problems` says the profile was left stretched.
        #
        # A bspline needs knot insertion to narrow correctly, and approximating
        # it as a polyline would change the document's own statement about what
        # it is. `samples` looks easy and is not: with `interpolation: "step"`
        # the function is discontinuous and `evaluate` holds the PREVIOUS
        # value, so sampling the endpoints lands on the wrong side of a jump --
        # narrowing [5, 9, 3] onto its second half produced [5, 9], which reads
        # 5 everywhere the parent reads 9. A narrowing that is subtly wrong at
        # the boundary is worse than one that did not happen and said so.
        return dict(fn)

    inside = [
        [(float(s) - lo) / span, float(v)]
        for s, v in fn.get("knots", [])
        if min(lo, hi) + 1e-9 < float(s) < max(lo, hi) - 1e-9
    ]
    if span < 0:
        inside.reverse()
    knots = [[0.0, evaluate(fn, lo)], *inside, [1.0, evaluate(fn, hi)]]
    values = [v for _, v in knots]
    if max(values) - min(values) < 1e-12:
        return {"kind": "constant", "value": values[0]}
    return {"kind": "piecewise_linear", "knots": knots}


def problems_against_limits(
    fn: dict[str, Any],
    limits: dict[str, Any] | None,
    length_mm: float = 0.0,
    label: str = "value",
) -> list[str]:
    """Does this profile stay inside the band the weld declares for it?

    The band is authored intent, like a tolerance: a profile outside it is not a
    worse weld, it is a DIFFERENT weld. So this reports rather than clamps, and
    the optimizer treats the same band as a hard mask on its lattice rather than
    as a penalty.

    Three checks, all of them things JSON Schema cannot express:

    - `min <= max` everywhere, because two independent scalar functions can
      cross;
    - the profile inside the band, reported with the position and both numbers,
      since "work angle out of range" without a place to look is not actionable;
    - the rate limit, in degrees per millimetre of arc.
    """
    if not isinstance(limits, dict) or not limits:
        return []
    lower = limits.get("min") if isinstance(limits.get("min"), dict) else None
    upper = limits.get("max") if isinstance(limits.get("max"), dict) else None

    found: list[str] = []
    for name, bound in (("min", lower), ("max", upper)):
        for problem in problems(bound) if bound is not None else []:
            found.append(f"{label} limits.{name}: {problem}")
    if found:
        # A malformed bound cannot be evaluated, and guessing what it meant
        # would report violations against a band nobody wrote.
        return found

    positions = _check_positions(fn, lower, upper)

    if lower is not None and upper is not None:
        crossed = [s for s in positions if evaluate(lower, s) > evaluate(upper, s)]
        if crossed:
            s = crossed[0]
            found.append(
                f"{label} limits cross at s={s:.4f}: min {evaluate(lower, s):.3f} "
                f"> max {evaluate(upper, s):.3f}"
            )
            return found

    worst: tuple[float, float, str] | None = None
    for s in positions:
        value = evaluate(fn, s)
        if lower is not None:
            gap = evaluate(lower, s) - value
            if gap > 1e-9 and (worst is None or gap > worst[0]):
                worst = (gap, s, f"below its minimum {evaluate(lower, s):.3f}")
        if upper is not None:
            gap = value - evaluate(upper, s)
            if gap > 1e-9 and (worst is None or gap > worst[0]):
                worst = (gap, s, f"above its maximum {evaluate(upper, s):.3f}")
    if worst is not None:
        _, s, why = worst
        found.append(f"{label} is {evaluate(fn, s):.3f} at s={s:.4f}, {why}")

    limit = limits.get("max_rate_deg_per_mm")
    if isinstance(limit, int | float) and not isinstance(limit, bool):
        if length_mm <= 0.0:
            # R-UI-1: a rate limit that could not be checked is reported as
            # unchecked, never passed over in silence.
            found.append(
                f"{label} has a rate limit of {float(limit)} deg/mm but the seam length "
                "is unknown, so it could not be checked"
            )
        else:
            rate, exact = max_rate_deg_per_mm(fn, length_mm)
            if rate > float(limit) + 1e-9:
                how = "" if exact else f" (measured over {_SAMPLE_COUNT} samples)"
                found.append(
                    f"{label} changes at up to {rate:.4f} deg/mm{how}, over its limit "
                    f"of {float(limit)} deg/mm"
                )
    return found


def is_constant(fn: dict[str, Any], tolerance: float = 1e-12) -> bool:
    """Does this function actually vary? A flat profile should be a constant."""
    if fn.get("kind") == "constant":
        return True
    if fn.get("kind") == "piecewise_linear":
        values = [float(k[1]) for k in fn["knots"]]
        return max(values) - min(values) <= tolerance
    return False


def scalars_in(seam: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """Every scalar function in a seam, with a dotted path. For validation."""
    found: list[tuple[str, dict[str, Any]]] = []

    def walk(node: Any, path: str) -> None:
        if isinstance(node, dict):
            if node.get("kind") in KINDS:
                found.append((path, node))
                return
            for key, value in node.items():
                walk(value, f"{path}.{key}" if path else key)
        elif isinstance(node, list):
            for index, value in enumerate(node):
                walk(value, f"{path}[{index}]")

    walk(seam, "")
    return found


def self_test() -> int:
    flat = {"kind": "constant", "value": 12.0}
    ramp = {"kind": "piecewise_linear", "knots": [[0.0, 0.0], [0.5, 10.0], [1.0, 0.0]]}

    assert evaluate(flat, 0.37) == 12.0
    assert evaluate(ramp, 0.0) == 0.0
    assert evaluate(ramp, 0.25) == 5.0
    assert evaluate(ramp, 0.5) == 10.0
    assert evaluate(ramp, 0.75) == 5.0
    assert evaluate(ramp, 1.0) == 0.0
    assert evaluate(ramp, -1.0) == 0.0, "s clamps rather than extrapolating"

    assert problems(flat) == []
    assert problems(ramp) == []
    assert problems({"kind": "piecewise_linear", "knots": [[0.0, 1.0], [0.5, 2.0]]})
    assert problems({"kind": "piecewise_linear", "knots": [[0.0, 1.0], [0.7, 2.0], [0.3, 3.0]]})
    assert is_constant({"kind": "piecewise_linear", "knots": [[0.0, 5.0], [1.0, 5.0]]})
    assert not is_constant(ramp)

    band = {
        "min": {"kind": "constant", "value": -5.0},
        "max": {"kind": "constant", "value": 5.0},
        "max_rate_deg_per_mm": 0.5,
    }
    assert problems_against_limits({"kind": "constant", "value": 0.0}, band, 100.0) == []
    assert problems_against_limits({"kind": "constant", "value": 9.0}, band, 100.0)
    # 10 degrees over 100 mm is 0.1 deg/mm, inside the limit; over 10 mm it is
    # 1.0, outside it. Same profile, same limit, different seam -- which is the
    # whole reason the rate is per millimetre and not per unit s.
    assert problems_against_limits(ramp, {"max_rate_deg_per_mm": 0.5}, 100.0) == []
    assert problems_against_limits(ramp, {"max_rate_deg_per_mm": 0.5}, 10.0)
    assert max_rate_deg_per_mm(ramp, 100.0) == (0.2, True)
    assert problems_against_limits(
        {"kind": "constant", "value": 0.0},
        {"min": {"kind": "constant", "value": 5.0}, "max": {"kind": "constant", "value": -5.0}},
    ), "crossed limits"

    print("scalar self-test ok: constant, piecewise_linear, clamping, knots, and limit bands")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--function", default="", help="a scalar function as JSON")
    parser.add_argument("--at", type=float, default=0.5)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.function:
        raise ValueError("one of --function or --self-test is required")

    fn = json.loads(ns.function)
    found = problems(fn)
    print(json.dumps({"problems": found, "value_at": ns.at, "value": evaluate(fn, ns.at)}, indent=2))
    return 1 if found else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"scalar: {exc}", file=sys.stderr)
        raise SystemExit(1)
