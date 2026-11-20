#!/usr/bin/env python
"""Seam frames: the `{P, t, r, b}` a viewer or planner needs at each point.

Implements `SEAM-MODEL.md` section 5. The frame construction lives here, on the
server, and nowhere else -- `WEBUI.md` section 8 explains why:

    If the client computes torch orientation in TypeScript and the server
    computes it in Python when saving, they can DIVERGE, and a sign-convention
    divergence is exactly the bug class this design exists to avoid. But
    round-tripping to the server on every slider drag is unusable.

    So: the server sends frames; the client applies angles.

Frames do not depend on the work or travel angle, so a slider drag needs no
round trip. The subtle parts -- orthogonalisation against the tangent, rail
correspondence, per-segment matching where the bisector flips at a corner --
happen once, here, against tests.

The two rotations the client is trusted with are ten lines and are given in
`apply_angles` so both sides can be checked against the same reference.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from seam_worker import paths
from seam_worker.seam.curve import Curve

SCHEMA = "amr-weld-planner-v1.seam-frames.v1"

DEFAULT_SAMPLES = 96

# Two backstops on chord refinement, neither of them a budget: at the tolerances
# anything real asks for, a joint lands in the low hundreds of frames. They exist
# so that a curve with a curvature spike -- or a caller asking for a nanometre --
# cannot subdivide until the payload is one no browser wants. Hitting either is
# reported in `warnings`, because a frame table quietly coarser than it was asked
# for is exactly what shows up later as an unexplained facet.
MAX_FRAMES = 4000
MIN_STEP_M = 1e-5

# `Q - P` this close to parallel with the tangent leaves no frame to build.
DEGENERATE_DOT = 0.999


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Right-handed rotation matrix, Rodrigues form."""
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm < 1e-30:
        return np.eye(3)
    a = a / norm
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def apply_angles(
    t: np.ndarray, r: np.ndarray, work_deg: float, travel_deg: float
) -> np.ndarray:
    """Torch reference direction `u` from the frame and the two angles.

        r' = R(t,  work) r          roll within the plane perpendicular to travel
        b' = t x r'
        u  = R(b', -travel) r'      tilt along travel

    Work first, then travel: that matches how a welder sets up, and is what
    makes travel angle mean "tilt along travel" whatever work angle is already
    applied. Positive travel is DRAG -- the torch leans back over completed
    weld. Verified numerically in `tests/programmer/test_frames.py` rather than asserted.

    `u` points from the weld toward the torch body; the electrode axis is `-u`.
    """
    r_prime = rotation(t, math.radians(work_deg)) @ r
    b_prime = np.cross(t, r_prime)
    return rotation(b_prime, -math.radians(travel_deg)) @ r_prime


def _rail_point(rail: Curve, seam: Curve, s: float, left_limit: bool = False) -> np.ndarray:
    """Rail point corresponding to seam arc length `s`.

    Per SEGMENT when the counts match, whole-curve otherwise. Per-segment is
    what makes a bisector rail work: the bisector genuinely flips at a corner,
    so the rail steps there, and matching over the whole curve would smear that
    discontinuity across the corner instead of respecting it.

    `left_limit` is the seam's, not the rail's: the rail steps where the seam
    corners, so taking the two on different sides of one join would read the
    work-angle datum a segment early.
    """
    if len(rail.segments) != len(seam.segments):
        return rail.point(s)
    index, u = seam.locate(s, left_limit)
    segment = seam.segments[index]
    within = segment.arc_length(segment.u0, u) / max(1e-30, segment.length)
    rail_segment = rail.segments[index]
    return rail_segment.point(rail_segment.u_at_length(within * rail_segment.length))


def _too_coarse(
    a: np.ndarray, m: np.ndarray, b: np.ndarray, max_chord_m: float, max_angle_rad: float
) -> bool:
    """Does the chord `a`->`b` need splitting at `m`?

    The mesher's two criteria, because these are the mesher's two criteria:
    `tessellate.py` discretises every CAD edge with `GCPnts_TangentialDeflection`
    under a linear AND an angular deflection, and the weld line is drawn from
    exactly that polyline. The angular one is what does the work at a corner --
    at 0.05 rad the chord sag is R/3200, so 3 um on a 10 mm radius and 16 um on a
    50 mm one, where the 0.05 mm linear bound alone would have allowed 50 um at
    both. A rail held only to the linear bound is therefore an order of magnitude
    coarser than the weld line beside it at every corner in the part, which is
    the whole visible complaint.
    """
    if float(np.linalg.norm(m - 0.5 * (a + b))) > max_chord_m:
        return True
    u, v = m - a, b - m
    return float(np.arctan2(np.linalg.norm(np.cross(u, v)), float(np.dot(u, v)))) > max_angle_rad


def _needs_split(
    rail: Curve | None, lo: float, hi: float, at: Any, max_chord_m: float, max_angle_rad: float
) -> bool:
    """Is the interval `lo`..`hi` too coarse for the seam or for the rail?

    The rail is the one that decides it. A frame table is sampled in the SEAM's
    arc length, but the rail does not advance at the seam's rate: correspondence
    is by fraction WITHIN a segment (D6), so on a corner whose rail segment is
    2.4x its seam segment -- a 10 mm radius offset 20 mm along a bisector at 45
    degrees, which is `pipe_bracket`'s every corner -- a uniform step in seam arc
    length walks the rail 2.4x as far. The rail is sampled most coarsely exactly
    where it is longest and most curved, and measuring only the seam never sees
    it: on that joint the seam's own chords are within tolerance everywhere the
    rail's are worst.

    The drawn rail is `P + offset * r` with `r` a unit vector, which is the rail
    point itself wherever the offset was achieved exactly; where it was only
    approximated (D20) the two differ by that stated deviation, the same bound
    either way. Sampling the rail curve is sampling what gets drawn.
    """
    mid = 0.5 * (lo + hi)
    p_lo, p_hi, p_mid = at(lo), at(hi), at(mid)
    if _too_coarse(p_lo[0], p_mid[0], p_hi[0], max_chord_m, max_angle_rad):
        return True
    if rail is None:
        return False
    return _too_coarse(p_lo[1], p_mid[1], p_hi[1], max_chord_m, max_angle_rad)


def _positions(
    seam: Curve,
    rail: Curve | None,
    samples: int | None,
    max_chord_m: float | None,
    max_angle_rad: float | None,
) -> tuple[list[float], list[str], dict[float, tuple[np.ndarray, np.ndarray, np.ndarray | None]]]:
    """Where to put the frames: the uniform grid, refined where a curve bends.

    **The uniform grid is a floor, and refinement only ever adds to it.**
    Replacing it was the obvious reading and it decimates: `Curve.sample_adaptive`
    caps its step at an eighth of the curve, so a straight seam comes back with
    nine frames. Nine are enough to DRAW a straight -- two would be -- but this
    table is also what the comb is spaced along and what the playhead steps
    through, and neither survives nine of them.

    **Bisection, not marching.** `sample_adaptive` reads curvature at the point
    it has reached and steps from there, so a 26 mm step taken along a straight
    steps clean over a 16 mm corner without ever evaluating inside it. That is
    not hypothetical: it is what happens on `pipe_bracket`'s joint 3, whose
    corners are a twelfth of its length. Splitting intervals instead means every
    interval is measured at its own midpoint and nothing can be skipped.

    **Corners are left exactly as they were.** An interval spanning a segment
    boundary is passed through untouched: the bisector genuinely steps there
    (D6), both curves are smooth only WITHIN a segment, and a sag measured
    across the step would be measuring the step. Refining it would chase a
    discontinuity down to the floor and change what the viewport asserts at a
    corner, which is a separate decision from this one.
    """
    grid = [float(s) for s in np.linspace(0.0, 1.0, max(2, samples or DEFAULT_SAMPLES))]
    if not max_chord_m and not max_angle_rad:
        return grid, [], {}
    # Either bound alone is a bound; an absent one is infinity, not zero.
    chord = max_chord_m if max_chord_m else float("inf")
    angle = max_angle_rad if max_angle_rad else float("inf")

    # Everything an evaluated position costs, kept so that the emit loop can
    # skip re-deriving it. The expense here is not the de Boor evaluation, it is
    # `u_at_length`: a Newton solve over quadrature, run once to place `s` on the
    # seam and again to place it on the rail. Refinement asks for positions the
    # emit loop is about to ask for again, and that -- not the tolerance -- was
    # what made the first version of this three times the cost of no refinement.
    cache: dict[float, tuple[np.ndarray, np.ndarray, np.ndarray | None]] = {}

    def at(s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """`(point, d1, rail point)` at `s`, evaluated at most once."""
        hit = cache.get(s)
        if hit is None:
            point, d1, _ = seam.evaluate(s)
            hit = (point, d1, None if rail is None else _rail_point(rail, seam, s))
            cache[s] = hit
        return hit

    def straight(index: int) -> bool:
        """Can no interval inside this segment fail either bound?

        Only when both curves are line segments there, and `degree == 1` on two
        poles IS a line segment -- no tolerance, no probing. Worth the check
        because straight runs are most of a real weld line and testing them is
        pure cost: a line's chord IS the line.
        """
        pair = [seam.segments[index]] + ([] if rail is None else [rail.segments[index]])
        return all(s.degree == 1 and len(s.homogeneous) == 2 for s in pair)

    out = [grid[0]]
    floored = False
    capped = False
    for lo, hi in itertools.pairwise(grid):
        index = seam.locate(lo)[0]
        if index != seam.locate(hi)[0] or straight(index):
            out.append(hi)
            continue
        stack = [(lo, hi)]
        while stack:
            a, b = stack.pop()
            if len(out) + len(stack) >= MAX_FRAMES:
                capped = True
                out.append(b)
                continue
            if (b - a) * seam.total <= MIN_STEP_M:
                floored = True
                out.append(b)
                continue
            if not _needs_split(rail, a, b, at, chord, angle):
                out.append(b)
                continue
            mid = 0.5 * (a + b)
            stack.append((mid, b))
            stack.append((a, mid))

    asked = f"max_chord_m = {max_chord_m}, max_angle_rad = {max_angle_rad}"
    warnings = []
    if floored:
        warnings.append(f"refinement hit the {MIN_STEP_M * 1000:g} mm step floor, short of {asked}")
    if capped:
        warnings.append(f"refinement stopped at the {MAX_FRAMES}-frame cap, short of {asked}")
    return out, warnings, cache


def frame_at(
    seam: Curve,
    s: float,
    rail: Curve | None = None,
    fixed_vec: Any = None,
    previous_r: Any = None,
    evaluated: tuple[Any, Any, Any] | None = None,
    left_limit: bool = False,
) -> tuple[Any, Any, Any, str | None]:
    """`(point, t, r, warning)` at one position. The body of the sampling loop.

    Extracted because a caller that bisects toward a boundary needs a frame at
    an arbitrary `s`, not a table on a grid -- `clearance.py` refines its
    blocked-span edges the way `access.py` refines its own. Keeping one
    implementation matters more than usual here: `r` is the datum the work
    angle is measured from, so a second version of this arithmetic would be a
    second definition of what a work angle means.

    `evaluated` is `(point, d1, rail point)` for this `s` when a caller already
    has them -- `_positions` computes exactly these to decide where to sample,
    and they cost a Newton solve apiece. It is a way of not paying twice, never
    a way of supplying different values: pass what this would have computed, or
    pass nothing.

    `left_limit` picks the side of a corner, and only matters when `s` lands
    exactly on one -- which is where seams are SPLIT, so a common case rather
    than a corner case. A seam arriving at its last point travels along the
    segment it came in on; everywhere else the direction of travel is the one
    being departed along. See `Curve.at_join`.
    """
    s = float(min(max(s, 0.0), 1.0))
    point, d1, rail_point = (
        evaluated if evaluated is not None else (*seam.evaluate(s, left_limit)[:2], None)
    )
    speed = float(np.linalg.norm(d1))
    t = d1 / speed if speed > 1e-30 else np.array([1.0, 0.0, 0.0])

    if rail is not None:
        direction = (
            _rail_point(rail, seam, s, left_limit) if rail_point is None else rail_point
        ) - point
    elif previous_r is not None:
        # Double-reflection would be the full method; projecting the last frame
        # forward is its first-order form and is stable here.
        direction = previous_r
    else:
        direction = np.array([0.0, 0.0, 1.0]) if fixed_vec is None else fixed_vec

    along = float(np.dot(direction, t))
    r = np.asarray(direction, dtype=float) - along * t
    norm = float(np.linalg.norm(r))
    warning = None
    if norm < 1e-12 or abs(along) / max(1e-30, float(np.linalg.norm(direction))) > DEGENERATE_DOT:
        # SEAM-MODEL section 5: the construction fails only here, and it is
        # detectable rather than silently producing a bad frame.
        warning = f"degenerate frame at s={s:.4f}: reference is parallel to the tangent"
        r = np.array([0.0, 0.0, 1.0]) - t[2] * t
        norm = float(np.linalg.norm(r)) or 1.0
    return point, t, r / norm, warning


def seam_frames(
    seam_curve: dict[str, Any],
    reference: dict[str, Any],
    samples: int | None = None,
    max_chord_m: float | None = None,
    max_angle_rad: float | None = None,
) -> dict[str, Any]:
    """Sample a seam and emit `{P, t, r, b}` plus curvature at each point.

    `samples` sets the uniform grid; `max_chord_m` and `max_angle_rad` add
    positions on top of it wherever the seam or the rail bends enough to need
    them. See `_positions`.
    """
    seam = Curve(seam_curve)
    mode = reference.get("mode", "rail_curve")
    rail = Curve(reference["rail"]) if mode == "rail_curve" and reference.get("rail") else None

    positions, limits, evaluated = _positions(seam, rail, samples, max_chord_m, max_angle_rad)

    fixed = reference.get("fixed_direction_xyz") or reference.get("seed_direction_xyz")
    fixed_vec = np.asarray(fixed, dtype=float) if fixed else np.array([0.0, 0.0, 1.0])

    rows: list[dict[str, Any]] = []
    warnings: list[str] = list(limits)
    previous_r: np.ndarray | None = None

    for s in positions:
        s = float(min(max(s, 0.0), 1.0))
        carry = previous_r if mode == "rotation_minimizing" else None
        point, t, r, warning = frame_at(seam, s, rail, fixed_vec, carry, evaluated.get(s))
        if warning:
            warnings.append(warning)
        previous_r = r

        rows.append(
            {
                "s": round(s, 9),
                "point_xyz_m": [round(float(c), 9) for c in point],
                "tangent_xyz": [round(float(c), 9) for c in t],
                "reference_xyz": [round(float(c), 9) for c in r],
                "binormal_xyz": [round(float(c), 9) for c in np.cross(t, r)],
                "curvature_1_per_m": round(seam.curvature(s), 6),
            }
        )

    return {
        "schema": SCHEMA,
        "length_m": seam.total,
        "closed": bool(seam_curve.get("closed", False)),
        "reference_mode": mode,
        "samples": len(rows),
        "frames": rows,
        "warnings": warnings,
    }


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    from seam_worker.seam.detect import detect_joints
    from seam_worker.cad.topology import extract_topology

    step = paths.PIPE_BRACKET
    if not step.exists():
        print(f"frames self-test: missing fixture {step}", file=sys.stderr)
        return 1
    joint = next(
        j
        for j in detect_joints(extract_topology(step))["weld_joints"]
        if j["id"] == "weld_joint_0003"
    )
    result = seam_frames(
        joint["weld_line"],
        {"mode": "rail_curve", "rail": joint["bisector_rail"]["curve"]},
        samples=64,
    )
    assert not result["warnings"], result["warnings"]
    worst = 0.0
    for row in result["frames"]:
        t = np.array(row["tangent_xyz"])
        r = np.array(row["reference_xyz"])
        b = np.array(row["binormal_xyz"])
        worst = max(
            worst,
            abs(float(np.dot(t, r))),
            abs(float(np.dot(t, b))),
            abs(float(np.dot(r, b))),
            abs(float(np.linalg.norm(t)) - 1.0),
        )
    print(
        f"frames self-test ok: {result['samples']} frames over "
        f"{result['length_m'] * 1000:.3f} mm, worst orthonormality error {worst:.2e}"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seam", default="", help="JSON with {curve, reference}")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.seam:
        raise ValueError("--seam is required unless --self-test is set")
    spec = json.loads(Path(ns.seam).read_text(encoding="utf-8"))
    print(json.dumps(seam_frames(spec["curve"], spec["reference"], ns.samples), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"frames: {exc}", file=sys.stderr)
        raise SystemExit(1)
