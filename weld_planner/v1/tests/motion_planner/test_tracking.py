"""The tracking certificate: proving the tip stays on the seam BETWEEN knots.

The claim under test is not "the error was small at the points we looked at"
-- that is what `weld_trajopt.py`'s midpoint quadrature already reports, and this
file measures how badly it can be wrong. The claim is that the sup over a
whole continuum is bounded, which no amount of sampling establishes.

Most of what is proved here needs no GPU and no fixture: a one-axis arm
sweeping a chord has a tracking error of exactly `1 - cos(half the sweep)`,
worked by hand, and a certificate that cannot get that right has no business
judging a real cell. A curved weld line has no such closed form, so its half
is held the only way a bound can be held from outside: by the mean value
theorem, against difference quotients of the target itself, on curves drawn
at random (`docs/issues/0008`). The fixture tests then hold the same
machinery against `bracket_b_2x` and `hitch_2x`, where the answers are not
analytic but a dense sweep can still say whether the bound was a bound.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import torch
from weld_motion_planner.geometry.seam_frame import Seam
from weld_motion_planner.verifier.verify import (
    SeamTarget,
    _curvature_bound,
    _hermite,
    _velocity_extreme,
    certify_tracking,
    chain_reach,
    kinematics,
)

from .committed import committed_plan

ROOT = Path(__file__).resolve().parents[2] / "data" / "motion"
TOLERANCE_M = 0.5e-3


# --------------------------------------------------------------------------
# The arithmetic the certificate is built out of


def test_the_velocity_extremes_are_exact_and_not_sampled():
    """A cubic Hermite's derivative is a quadratic, so its extremes on an
    interval are two ends and at most one vertex -- all arithmetic. This is
    the half of the bound that makes the tip's Lipschitz constant exact, so
    it is held against a dense numerical maximum: not merely >= it, but equal
    to it, because a bound that is loose here pays for it in bisection depth
    on every interval of every seam."""
    rng = np.random.default_rng(20260813)
    worst_ratio = 0.0
    for _ in range(120):
        q0, q1, d0, d1 = (rng.normal(size=4) for _ in range(4))
        h = float(abs(rng.normal())) + 0.05
        lo = float(rng.uniform(0.0, 0.6))
        hi = lo + float(rng.uniform(0.02, 0.4))
        us = np.linspace(lo, hi, 3001)
        path = np.stack([_hermite(q0, q1, d0, d1, h, u) for u in us])
        numeric = np.abs(np.gradient(path, us * h, axis=0)).max(axis=0)
        closed = _velocity_extreme(q0, q1, d0, d1, h, lo, hi)
        assert (numeric <= closed + 1e-9).all(), "the closed form missed a real extreme"
        worst_ratio = max(worst_ratio, float((numeric / closed).min()))
    assert worst_ratio > 0.99, "the closed form is loose; it should be the exact maximum"


def test_chain_reach_answers_zero_for_a_joint_that_moves_nothing():
    """Zero is a fact, not an omission -- a joint off the chain to a point
    genuinely cannot move it, and charging it anyway would make every bound
    on the cell useless. The trap on the other side is worse: forgetting the
    offset from the last joint to the point makes the tip look welded to the
    wrist axis, and the certificate would then pass a swing it never bounded."""
    cell, _ = _rod_cell(reach_m=1.0)
    kin = kinematics(cell)
    assert chain_reach(kin, "J1", "tip", 0.0) == pytest.approx(1.0)
    # The rod's own origin sits ON J1's axis: the joint turns it without
    # moving it.
    assert chain_reach(kin, "J1", "rod", 0.0) == pytest.approx(0.0)
    # A point 50 mm off that origin does move, by exactly its own offset.
    assert chain_reach(kin, "J1", "rod", 0.05) == pytest.approx(0.05)
    assert chain_reach(kin, "J1", "world", 0.0) == pytest.approx(0.0)


# --------------------------------------------------------------------------
# A cell whose answer is known by hand


def _rod_cell(reach_m: float):
    """A rod of length `reach_m` on one revolute axis, tip frame at its end."""
    cell = {
        "id": "rod_demo",
        "frames": {"base": "world", "tool": "tip", "work": "world"},
        "links": [{"name": n} for n in ("world", "rod", "tip")],
        "joints": [
            {"name": "J1", "parent": "world", "child": "rod", "type": "revolute",
             "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
             "limits_rad": {"lower": -3.2, "upper": 3.2}},
            {"name": "tip_fixed", "parent": "rod", "child": "tip", "type": "fixed",
             "xyz_m": [reach_m, 0, 0], "rpy_rad": [0, 0, 0]},
        ],
    }
    return cell, reach_m


def _chord_seam(half_sweep: float, reach_m: float, degree: int = 1):
    """The straight line joining the two ends of the rod's swing.

    The rod's tip travels an ARC and the seam is its CHORD, so the tracking
    error is the sagitta and its maximum is `reach * (1 - cos(half_sweep))`
    exactly, at the middle of the sweep. Nothing about that number is
    measured, which is what makes it a test.
    """
    a, b = -half_sweep, half_sweep
    p0 = [reach_m * math.cos(a), reach_m * math.sin(a), 0.0]
    p1 = [reach_m * math.cos(b), reach_m * math.sin(b), 0.0]
    if degree == 1:
        segment = {"degree": 1, "knots_u": [0, 0, 1, 1], "control_points_xyz_m": [p0, p1]}
    else:
        middle = [0.5 * (p0[0] + p1[0]), 0.0, 0.0]
        segment = {"degree": 2, "knots_u": [0, 0, 0, 1, 1, 1],
                   "control_points_xyz_m": [p0, middle, p1]}
    return {
        "id": "chord",
        "curve": {"segments": [segment]},
        # A datum perpendicular to travel, so the frame is well defined, and
        # angles of zero so the torch direction is that datum exactly.
        "reference": {"mode": "fixed", "fixed_direction_xyz": [0.0, 0.0, 1.0]},
        "orientation": {"work_angle_deg": {"kind": "constant", "value": 0.0},
                        "travel_angle_deg": {"kind": "constant", "value": 0.0}},
        # An authored ZERO, not an absence: the sagitta argument needs the
        # target ON the weld line, and nothing states a stickout to fall
        # back to on a synthetic rod.
        "process": {"standoff_mm": {"kind": "constant", "value": 0.0}},
    }


def _sweep(half_sweep: float, reach_m: float, knots: int = 5, degree: int = 1):
    """Certify the rod sweeping its chord, standoff zero so the target is the
    weld point itself and the whole answer is the sagitta."""
    cell, _ = _rod_cell(reach_m)
    kin = kinematics(cell)
    target = SeamTarget(Seam(_chord_seam(half_sweep, reach_m, degree), dtype=torch.float64),
                        np.eye(4))
    duration = 1.0
    times = np.linspace(0.0, duration, knots)
    angles = np.linspace(-half_sweep, half_sweep, knots)
    waypoints = angles.reshape(-1, 1)
    # Constant joint rate: with matching Hermite velocities the interpolant
    # is the straight line through the knots, so the arc really is what the
    # tip traces and no spline overshoot is hiding in the answer.
    rates = np.full((knots, 1), 2.0 * half_sweep / duration)
    fractions = np.linspace(0.0, 1.0, knots)
    return certify_tracking(kin, cell, target, waypoints, rates, times, fractions,
                            tolerance_m=TOLERANCE_M)


def test_a_sweep_inside_tolerance_is_certified_clear():
    half = 0.02
    sagitta = 1.0 - math.cos(half)          # 0.20 mm, under the 0.5 mm tolerance
    assert sagitta < TOLERANCE_M
    cert = _sweep(half, 1.0)
    assert cert.ok, cert.summary()
    assert cert.bound_m <= TOLERANCE_M
    assert cert.bound_m >= sagitta, \
        "an upper bound below the error it is bounding is not an upper bound"


def test_a_sweep_outside_tolerance_is_caught_with_the_sagitta_as_the_witness():
    half = 0.1
    sagitta = 1.0 - math.cos(half)          # 5.0 mm, ten times the tolerance
    assert sagitta > TOLERANCE_M
    # FOUR knots, so the middle of the sweep -- where the sagitta is -- is the
    # midpoint of an interval rather than a knot, and the witness can land on
    # it exactly. With five it lands on a knot the verifier never evaluates, which
    # is a fine thing for the BOUND to cover and a bad thing to assert about
    # the witness.
    cert = _sweep(half, 1.0, knots=4)
    assert not cert.clear and cert.violations, cert.summary()
    assert cert.worst_witnessed_m == pytest.approx(sagitta, rel=1e-6)
    assert cert.worst_witnessed_m <= sagitta + 1e-9, \
        "witnessed more error than the geometry has; the target has drifted"
    # And with FIVE, where the worst instant is a knot and every point the
    # verifier looks at is better than it, the bound must still cover the truth.
    assert _sweep(half, 1.0, knots=5).bound_m >= sagitta


def test_sampling_the_knots_alone_would_have_passed_the_bad_sweep():
    """The whole argument for a certificate, in one comparison. The chord's
    two ends ARE tip poses, so a single-interval sweep between them is exact
    at every knot it has -- and 5 mm out in the middle. A verifier that checked
    waypoints would sign it off."""
    half = 0.1
    cell, reach = _rod_cell(1.0)
    kin = kinematics(cell)
    target = SeamTarget(Seam(_chord_seam(half, reach), dtype=torch.float64), np.eye(4))
    for angle, fraction in ((-half, 0.0), (half, 1.0)):
        frames = kin.transforms([angle])
        aim = frames["world"][:3, :3] @ target.local(fraction)[0] + frames["world"][:3, 3]
        assert float(np.linalg.norm(frames["tip"][:3, 3] - aim)) < 1e-12, \
            "the knots were meant to be exact"
    cert = _sweep(half, 1.0, knots=2)
    assert not cert.clear, "and the certificate must fail it anyway"
    assert cert.worst_witnessed_m == pytest.approx(1.0 - math.cos(half), rel=1e-6)


# --------------------------------------------------------------------------
# A curved weld line: the bound on the target's own speed


def _cubic(p0, p1, p2, p3, weights=None) -> dict:
    out = {"degree": 3, "knots_u": [0, 0, 0, 0, 1, 1, 1, 1],
           "control_points_xyz_m": [[float(v) for v in p] for p in (p0, p1, p2, p3)]}
    if weights is not None:
        out["weights"] = [float(w) for w in weights]
    return out


def _random_curve(rng, segments: int, scale: float, rational: bool = False) -> dict:
    """Cubic pieces joined end to end, each bent well away from its chord,
    and weighted when `rational` -- a CAD arc is a weighted spline."""
    out, start = [], rng.normal(size=3) * scale
    for _ in range(segments):
        end = start + rng.normal(size=3) * scale
        bend = rng.normal(size=(2, 3)) * scale
        weights = [1.0, *rng.uniform(0.4, 2.5, 2), 1.0] if rational else None
        out.append(_cubic(start, start + bend[0], end + bend[1], end, weights))
        start = end
    return {"segments": out}


def _profile(rng, lo: float, hi: float, knots: int) -> dict:
    s = np.concatenate([[0.0], np.sort(rng.uniform(0.05, 0.95, knots - 2)), [1.0]])
    return {"kind": "piecewise_linear",
            "knots": [[float(a), float(b)] for a, b in zip(s, rng.uniform(lo, hi, knots))]}


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    return v / np.linalg.norm(v)


def _random_curved_seam(rng, reference: str) -> dict:
    """A seam where everything turns at once: a bent weld line (weighted, for
    `rational`), a datum that is a fixed vector or a rail (read per segment,
    or at the shared `s` when the segment counts differ), both angles and the
    standoff as profiles."""
    segments = int(rng.integers(1, 4))
    curve = _random_curve(rng, segments, 0.05, rational=reference == "rational")
    if reference in ("fixed", "rational"):
        datum = {"mode": "fixed_vector", "fixed_direction_xyz": _unit(rng.normal(size=3)).tolist()}
    elif reference == "rail_per_segment":
        lift = np.array([0.0, 0.0, 0.03])
        rail = {"segments": [
            _cubic(*(np.asarray(p) + lift + rng.normal(size=3) * 0.004
                     for p in segment["control_points_xyz_m"]))
            for segment in curve["segments"]]}
        datum = {"mode": "rail_curve", "rail": rail}
    else:
        datum = {"mode": "rail_curve", "rail": _random_curve(rng, segments + 1, 0.05)}
    start = float(rng.uniform(0.0, 0.3))
    return {
        "id": f"random_{reference}",
        "curve": curve,
        "span": {"start_s": start, "end_s": float(rng.uniform(start + 0.4, 1.0))},
        "direction": "reverse" if rng.random() < 0.5 else "forward",
        "reference": datum,
        "orientation": {"work_angle_deg": _profile(rng, -40.0, 40.0, 4),
                        "travel_angle_deg": _profile(rng, -20.0, 20.0, 3)},
        "process": {"standoff_mm": _profile(rng, 10.0, 25.0, 3)},
    }


def _pieces(target: SeamTarget) -> list[tuple[float, float]]:
    cuts = sorted({0.0, 1.0, *target.breakpoints()})
    return list(zip(cuts[:-1], cuts[1:]))


@pytest.mark.parametrize("reference", ["fixed", "rational", "rail_per_segment", "rail_shared_s"])
def test_the_curved_bound_is_a_bound_on_the_target_speed(reference):
    """The soundness claim for `docs/issues/0008`, held without any trust in
    its derivation: `|T(b) - T(a)| <= sup|T'| (b - a)` inside a piece, by the
    mean value theorem, so no difference quotient of the target may exceed
    the bound -- on curves bent hard enough that the tangent, the datum and
    both angles all turn at once.

    Stretches the certificate refuses are allowed (a random datum can pass
    near the tangent, a random bend can cusp) and counted; a batch that
    refused everything would prove nothing, so most must be bounded."""
    seed = {"fixed": 1, "rational": 4, "rail_per_segment": 2, "rail_shared_s": 3}[reference]
    rng = np.random.default_rng(seed)
    bounded = refused = 0
    for _ in range(12):
        target = SeamTarget(Seam(_random_curved_seam(rng, reference), dtype=torch.float64),
                            np.eye(4))
        for piece_lo, piece_hi in _pieces(target):
            for _ in range(2):
                # Stretches like the ones the certificate asks about: a knot
                # interval is a small share of a piece, and the datum's
                # Lipschitz margin is paid over the stretch's half-width.
                width = (piece_hi - piece_lo) * rng.uniform(0.01, 0.2)
                a = rng.uniform(piece_lo, piece_hi - width)
                b = a + width
                bound = target.lipschitz_f(float(a), float(b))
                if bound is None:
                    refused += 1
                    continue
                bounded += 1
                f = np.linspace(a, b, 257)
                points = target.local(f)
                quotient = np.linalg.norm(np.diff(points, axis=0), axis=1) / np.diff(f)
                assert quotient.max() <= bound * (1.0 + 1e-9), (
                    f"{reference}: the target moved at {quotient.max():.6f} m per unit "
                    f"fraction on [{a:.6f}, {b:.6f}], above a bound of {bound:.6f}")
    assert bounded >= 3 * refused and bounded > 20, (bounded, refused)


def test_the_curvature_bound_holds_and_tightens_as_the_range_does():
    """`sup |C''| / inf |C'|^2` from Bezier control points, against the
    segment's own analytic curvature. A degree-5 spline with interior knots,
    so the per-span stretches are exercised rather than one polynomial."""
    from weld_motion_planner.geometry.curves import Segment

    rng = np.random.default_rng(20260917)
    steps = rng.normal(size=(9, 3)) * 0.02 + np.array([0.03, 0.0, 0.0])
    segment = Segment({"degree": 5, "control_points_xyz_m": np.cumsum(steps, axis=0).tolist(),
                       "knots_u": [0] * 6 + [0.3, 0.5, 0.7] + [1] * 6}, dtype=torch.float64)
    whole = _curvature_bound(segment, 0.0, 1.0)
    part = _curvature_bound(segment, 0.52, 0.68)
    assert whole is not None and part is not None
    for lo, hi, bound in ((0.0, 1.0, whole), (0.52, 0.68, part)):
        kappa = segment.curvature(torch.linspace(lo, hi, 4001, dtype=torch.float64))
        assert float(kappa.max()) <= bound, (lo, hi, float(kappa.max()), bound)
    assert part <= whole


def test_a_rational_arc_is_bounded_by_its_own_curvature():
    """A CAD arc is a WEIGHTED spline -- `hitch_2x seam_0008` is the
    standard seven-point circle -- so the bound has to go through the quotient
    rule rather than refuse it. A circle's curvature is `1 / r` everywhere:
    the bound must sit at or above that, and not absurdly far above it on a
    short stretch, where the hull has closed in on the arc."""
    from weld_motion_planner.geometry.curves import Segment

    radius = 0.04
    r2 = 0.5
    third = [[1, 0], [1, math.sqrt(3)], [-0.5, math.sqrt(3) / 2], [-2, 0],
             [-0.5, -math.sqrt(3) / 2], [1, -math.sqrt(3)], [1, 0]]
    circle = Segment({
        "degree": 2,
        "knots_u": [0, 0, 0, 1, 1, 2, 2, 3, 3, 3],
        "control_points_xyz_m": [[radius * x, radius * y, 0.0] for x, y in third],
        "weights": [1, r2, 1, r2, 1, r2, 1],
    }, dtype=torch.float64)
    kappa = circle.curvature(torch.linspace(0.0, 3.0, 3001, dtype=torch.float64))
    assert float((kappa - 1.0 / radius).abs().max()) < 1e-6, "the fixture is not a circle"
    whole = _curvature_bound(circle, 0.0, 3.0)
    short = _curvature_bound(circle, 1.2, 1.25)
    assert whole is not None and short is not None
    assert whole >= 1.0 / radius and short >= 1.0 / radius
    assert short <= 1.2 / radius, f"{short * radius:.3f} x the true curvature"


def _arc_seam(half_sweep: float, reach_m: float) -> dict:
    """The rod tip's own arc as a non-rational cubic: the classic
    `4/3 tan(theta/4)` handles, within microns of the circle over this sweep,
    so what the certificate measures is the arm leaving a CURVED line."""
    handle = 4.0 / 3.0 * math.tan(half_sweep / 2.0) * reach_m
    a, b = -half_sweep, half_sweep
    p0 = np.array([math.cos(a), math.sin(a), 0.0]) * reach_m
    p3 = np.array([math.cos(b), math.sin(b), 0.0]) * reach_m
    p1 = p0 + handle * np.array([-math.sin(a), math.cos(a), 0.0])
    p2 = p3 - handle * np.array([-math.sin(b), math.cos(b), 0.0])
    seam = _chord_seam(half_sweep, reach_m)
    seam["id"] = "arc"
    seam["curve"] = {"segments": [_cubic(p0, p1, p2, p3)]}
    return seam


def _arc_sweep(reach_m: float, arc_reach_m: float, knots: int):
    half = 0.2
    cell, _ = _rod_cell(reach_m)
    kin = kinematics(cell)
    target = SeamTarget(Seam(_arc_seam(half, arc_reach_m), dtype=torch.float64), np.eye(4))
    cert = certify_tracking(kin, cell, target,
                            np.linspace(-half, half, knots).reshape(-1, 1),
                            np.full((knots, 1), 2.0 * half),
                            np.linspace(0.0, 1.0, knots), np.linspace(0.0, 1.0, knots),
                            tolerance_m=TOLERANCE_M)
    return kin, target, half, cert


def test_a_curved_weld_line_is_certified_when_the_arm_follows_it():
    """The case `docs/issues/0008` was open for. The rod sweeps the arc and
    the seam IS that arc, so the error is only the cubic's microns and the
    certificate must say PASS -- where it used to say REFUSED -- with a bound
    that still covers what a dense sweep finds."""
    kin, target, half, cert = _arc_sweep(0.3, 0.3, knots=9)
    assert cert.ok, cert.summary()
    assert not cert.unverifiable
    worst = 0.0
    for u in np.linspace(0.0, 1.0, 1001):
        frames = kin.transforms([-half + 2.0 * half * u])
        worst = max(worst, float(np.linalg.norm(frames["tip"][:3, 3] - target.local(u)[0])))
    assert worst <= cert.bound_m, (worst, cert.bound_m)


def test_a_curved_line_with_the_arm_off_it_still_fails():
    """Certifying a curve must not mean passing one: the same arc swept 3 mm
    further out is out of tolerance everywhere and has to say so."""
    _, _, _, cert = _arc_sweep(0.303, 0.3, knots=5)
    assert not cert.clear and cert.violations, cert.summary()
    assert cert.worst_witnessed_m == pytest.approx(0.003, rel=1e-2)


# --------------------------------------------------------------------------
# The real cell


cuda = pytest.mark.skipif(not torch.cuda.is_available(),
                          reason="synthesis is a batched torch path")


@pytest.fixture(scope="module")
def synthesized():
    from weld_motion_planner.io.plan_request import read

    from .pipeline import search_then_plan_weld_trajectories

    request = read(committed_plan(ROOT / "bracket_b_2x.weldplan"))
    return {
        seam_id: search_then_plan_weld_trajectories(
            request, only_seam=seam_id, inner_steps=30, outer_steps=2).trajectories[0]
        for seam_id in ("seam_0003", "seam_0007")
    }


@cuda
def test_the_smooth_seam_is_certified_over_its_whole_length(synthesized):
    cert = synthesized["seam_0003"].tracking
    assert cert is not None and cert.ok, cert.summary()
    assert cert.bound_m <= cert.tolerance_m
    assert not cert.unverifiable, "the placed fixture must be fully verifiable"


@cuda
def test_the_certificate_bounds_what_a_dense_sweep_finds(synthesized):
    """The soundness claim, checked the only way it can be checked from
    outside: sample far more finely than the verifier did and confirm nothing
    exceeds the bound. Ninety-seven samples per interval, deliberately not a
    power of two, so the sweep and the bisection share no instants."""
    from weld_motion_planner.io.plan_request import read

    request = read(committed_plan(ROOT / "bracket_b_2x.weldplan"))
    for seam_id, report in synthesized.items():
        worst = _dense_worst(request, seam_id, report, samples=97)
        assert worst <= report.tracking.bound_m + 1e-12, (
            f"{seam_id}: sampled {worst * 1e3:.4f} mm above a bound of "
            f"{report.tracking.bound_m * 1e3:.4f} mm")


@cuda
def test_the_curved_seam_is_decided_on_the_real_cell_too():
    """`hitch_2x seam_0008`, a 40 mm rational circle and the corpus's one
    curved weld line, the seam `docs/issues/0008` was named for.

    It used to come back REFUSED -- "no derivative bound" -- whatever its
    trajectory did. With the curvature bound the certificate has to DECIDE,
    and decide what the optimizer's own between-knot measure says: on the
    current cell that trajectory leaves the circle by about 1.6 mm between
    knots (measured 2026-09-17: optimizer midpoints 1.597 mm, certificate
    witness 1.707 mm against a 0.5 mm tolerance), so the verdict is FAIL, and
    a trajectory that tracked it would be PASS. Either way a dense sweep has
    to stay under the bound the certificate reports.
    """
    from weld_motion_planner.io.plan_request import read

    from .pipeline import search_then_plan_weld_trajectories

    request = read(committed_plan(ROOT / "hitch_2x.weldplan"))
    report = search_then_plan_weld_trajectories(
        request, only_seam="seam_0008", k_best=1).trajectories[0]
    cert = report.tracking
    assert cert is not None
    assert not cert.unverifiable, cert.summary()
    assert cert.bound_m < float("inf") and not cert.undecided, cert.summary()
    out = report.midpoint_tracking_m > cert.tolerance_m
    assert cert.ok is not out, (cert.summary(), report.midpoint_tracking_m)
    assert report.midpoint_tracking_m <= cert.bound_m, \
        "the optimizer measured more error than the certificate bounded"
    worst = _dense_worst(request, "seam_0008", report, samples=97)
    assert worst <= cert.bound_m + 1e-12


def _dense_worst(request, seam_id: str, report, *, samples: int) -> float:
    from .pipeline import plan_seam_search
    from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix
    from weld_motion_planner.robot_cell.robot import fk

    cell = request.cell
    tool = str(cell["frames"]["tool"])
    work = str(cell["frames"]["work"])
    kin = kinematics(cell)
    arm = fk.build(cell, [tool])
    order = {name: i for i, name in enumerate([*arm.axis_names, "J7"])}

    planned = next(s for s in plan_seam_search(request, SeamSearchOptions(k_best=1)).seams
                   if s.id == seam_id)
    # The LATTICE Hermite: the curve the certificate was computed on. The
    # report's `knots` are the same curve re-sampled at the controller's
    # clock (`weld_trajopt.resample_wire`), 25x the segments; sweeping that
    # would check the same cubic at the same instants for 25x the FK.
    knots, velocity, times_s, fractions = (
        report.lattice_knots, report.lattice_velocity,
        report.lattice_times_s, report.lattice_fractions)
    length = len(knots)
    waypoints = np.zeros((length, kin.num_dof))
    rates = np.zeros_like(waypoints)
    for i, name in enumerate(kin.axis_names):
        if name in order:
            waypoints[:, i] = knots[:, order[name]]
            rates[:, i] = velocity[:, order[name]]
        elif name in planned.held:
            waypoints[:, i] = planned.held[name]

    joints = request.joints_by_id
    spec = next(s for s in request.seams if str(s["id"]) == seam_id)
    target = SeamTarget(
        Seam(spec, joints.get(str(spec.get("weld_joint_ref"))), dtype=torch.float64),
        placement_matrix(request.placement))

    worst = 0.0
    for k in range(length - 1):
        h = float(times_s[k + 1] - times_s[k])
        f0, f1 = float(fractions[k]), float(fractions[k + 1])
        for u in np.linspace(0.0, 1.0, samples):
            q = _hermite(waypoints[k], waypoints[k + 1], rates[k], rates[k + 1], h, u)
            frames = kin.transforms(q)
            aim = (frames[work][:3, :3] @ target.local(f0 + (f1 - f0) * u)[0]
                   + frames[work][:3, 3])
            worst = max(worst, float(np.linalg.norm(frames[tool][:3, 3] - aim)))
    return worst
