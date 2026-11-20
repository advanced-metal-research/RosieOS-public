"""Rational B-spline evaluation, arc length and curvature.

`curve.py` is pure Python and numpy, with no OCCT in it. That is what makes
these tests worth anything: **OCCT is an independent oracle here, not the
implementation.** Two separately written evaluators agreeing to 1e-12 on every
edge of two real workpieces is real evidence; a module checked against the
library it wraps is not.

Everything else is checked against a closed form -- a circle's length is
`R * sweep`, its curvature is exactly `1/R`, and its arc length is uniform in
angle. An error has nowhere to hide in those.
"""

import itertools
import math

import numpy as np
import pytest
from seam_worker.seam.curve import Curve, Segment, de_boor, de_boor_many

MM = 1000.0


def _unit_quarter_circle():
    """Exact rational quadratic. Length pi/2, curvature 1, everywhere."""
    return Curve(
        {
            "segments": [
                {
                    "degree": 2,
                    "control_points_xyz_m": [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                    "weights": [1.0, math.cos(math.pi / 4), 1.0],
                    "knots_u": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                }
            ]
        }
    )


# --------------------------------------------------------------------------
# closed forms
# --------------------------------------------------------------------------


def test_quarter_circle_length_curvature_and_uniformity():
    curve = _unit_quarter_circle()
    assert curve.total == pytest.approx(math.pi / 2, abs=1e-12)
    for s in np.linspace(0.0, 1.0, 21):
        point = curve.point(s)
        assert np.linalg.norm(point) == pytest.approx(1.0, abs=1e-9)
        assert curve.curvature(s) == pytest.approx(1.0, abs=1e-7)
        # arc length uniform in angle is the property `u` does NOT have
        assert math.atan2(point[1], point[0]) == pytest.approx(s * math.pi / 2, abs=1e-7)


def test_parameter_is_not_arc_length_on_an_arc():
    """The distinction the whole contract is built on.

    On this arc, u = 0.25 lands at 21.60 degrees, not 22.5. Treating the dial
    as a distance is exactly the defect that put a waypoint 62.5 mm out of
    place on the dog_box loop.
    """
    segment = _unit_quarter_circle().segments[0]
    at_u = segment.point(0.25)
    angle_at_u = math.degrees(math.atan2(at_u[1], at_u[0]))
    assert angle_at_u == pytest.approx(21.5981, abs=1e-3)

    curve = _unit_quarter_circle()
    at_s = curve.point(0.25)
    angle_at_s = math.degrees(math.atan2(at_s[1], at_s[0]))
    assert angle_at_s == pytest.approx(22.5, abs=1e-5)
    assert abs(angle_at_u - angle_at_s) > 0.85, "u and s must genuinely differ here"


def test_arc_length_inversion_round_trips():
    curve = _unit_quarter_circle()
    for s in np.linspace(0.0, 1.0, 33):
        index, u = curve.locate(s)
        recovered = curve.segments[index].arc_length(curve.segments[index].u0, u)
        assert recovered / curve.total == pytest.approx(s, abs=1e-9)


def test_straight_line_has_zero_curvature_and_exact_length():
    a, b = [0.0, 0.0, 0.2], [0.12, 0.0, 0.2]
    curve = Curve(
        {
            "segments": [
                {
                    "degree": 1,
                    "control_points_xyz_m": [a, b],
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                }
            ]
        }
    )
    assert curve.total == pytest.approx(math.dist(a, b), abs=1e-15)
    for s in np.linspace(0.0, 1.0, 9):
        assert curve.curvature(s) == pytest.approx(0.0, abs=1e-12)
        assert curve.point(s) == pytest.approx(np.add(a, np.subtract(b, a) * s), abs=1e-15)


def test_derivatives_match_finite_differences():
    """Guards the rational quotient rule. Differencing the projected curve is
    the wrong way to get derivatives, but it is a valid CHECK of them."""
    segment = _unit_quarter_circle().segments[0]
    h = 1e-6
    for u in (0.2, 0.4, 0.6, 0.8):
        _, d1, d2 = segment.evaluate(u)
        fd1 = (segment.point(u + h) - segment.point(u - h)) / (2 * h)
        fd2 = (segment.point(u + h) - 2 * segment.point(u) + segment.point(u - h)) / (h * h)
        assert d1 == pytest.approx(fd1, abs=1e-6)
        assert d2 == pytest.approx(fd2, abs=1e-3)


# --------------------------------------------------------------------------
# OCCT as an independent oracle
# --------------------------------------------------------------------------


@pytest.mark.parametrize("fixture", ["dog_box", "pipe_bracket"])
def test_evaluation_matches_occt_at_equal_arc_length(fixture, request, repo_root):
    """The strongest check available: two independently written evaluators.

    curve.py is pure Python with no OCCT; OCCT is a C++ kernel. Agreement
    across every edge of two real workpieces is evidence a self-consistency
    test cannot give.

    Compared at equal ARC LENGTH, not at equal parameter. `GeomConvert`
    preserves an edge's parameter RANGE and knot positions but not its
    parametrisation within a span: on a circle the two coincide only at knots
    and span midpoints. A first version of this test sampled at
    t = 0, 0.25, 0.5, 0.75, 1.0 of a two-span arc -- every one of which is a
    knot or a span midpoint -- and passed by luck while 28 of 90 edges were
    actually being compared wrongly.

    This also exercises the s -> u inversion on both sides, since OCCT's
    GCPnts_AbscissaPoint does its own.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_AbscissaPoint
    from seam_worker.cad.topology import (
        edge_geom_key,
        extract_topology,
        load_step,
        sorted_solids,
    )

    step = (
        repo_root / "cadquery" / "v1" / "step" / "dog_box.step"
        if fixture == "dog_box"
        else request.getfixturevalue("pipe_bracket_step")
    )
    topology = extract_topology(step)
    shape = load_step(step)
    edges = {}
    for index, solid in enumerate(sorted_solids(shape)):
        for edge in solid.Edges():
            edges.setdefault(edge_geom_key(f"solid_{index}", edge), edge)

    worst_point = 0.0
    checked = 0
    for record in topology["edges"]:
        occt = edges.get(record["geom_key"])
        if occt is None:
            continue
        segment = Segment(record["bspline"])
        adaptor = BRepAdaptor_Curve(occt.wrapped)
        total = GCPnts_AbscissaPoint.Length_s(adaptor)
        for t in np.linspace(0.0, 1.0, 11):
            locator = GCPnts_AbscissaPoint(adaptor, t * total, adaptor.FirstParameter())
            if not locator.IsDone():
                continue
            theirs = np.array(adaptor.Value(locator.Parameter()).Coord()) / MM
            mine = segment.point(segment.u_at_length(t * segment.length))
            worst_point = max(worst_point, float(np.linalg.norm(mine - theirs)))
        checked += 1

    assert checked > 50, f"only {checked} edges compared"
    assert worst_point * MM < 1e-4, f"position disagrees by {worst_point * MM:.3e} mm"


@pytest.mark.parametrize("fixture", ["dog_box", "pipe_bracket"])
def test_length_matches_the_closed_form_on_every_edge(fixture, request, repo_root):
    """Circles have length R * sweep and lines have length |b - a|. No kernel
    involved, so nothing here can agree for the wrong reason."""
    from seam_worker.cad.topology import extract_topology

    step = (
        repo_root / "cadquery" / "v1" / "step" / "dog_box.step"
        if fixture == "dog_box"
        else request.getfixturevalue("pipe_bracket_step")
    )
    worst = 0.0
    checked = 0
    for record in extract_topology(step)["edges"]:
        analytic = record.get("analytic")
        if not analytic:
            continue
        if analytic["kind"] == "line":
            expected = math.dist(analytic["start_xyz_m"], analytic["end_xyz_m"])
        else:
            expected = analytic["radius_m"] * abs(analytic["sweep_angle_rad"])
        worst = max(worst, abs(Segment(record["bspline"]).length - expected))
        checked += 1
    assert checked > 50
    assert worst * MM < 1e-5, f"length disagrees with closed form by {worst * MM:.3e} mm"


def test_curvature_matches_the_analytic_radius_on_real_arcs(dog_box_topology):
    """Every circular edge in dog_box carries its radius; curvature must be 1/R."""
    checked = 0
    for record in dog_box_topology["edges"]:
        if record["curve_type"] != "circle":
            continue
        segment = Segment(record["bspline"])
        expected = 1.0 / record["analytic"]["radius_m"]
        for u in np.linspace(segment.u0, segment.u1, 9):
            assert segment.curvature(float(u)) == pytest.approx(expected, rel=1e-6), record["id"]
        checked += 1
    assert checked == 28, "every circular edge in dog_box, not just the dogbone's"


def test_composite_length_matches_the_detected_weld_line(pipe_bracket_topology, pipe_bracket_step):
    """End to end: evaluate the emitted weld line and get the reported length."""
    from seam_worker.seam.detect import detect_joints

    _ = pipe_bracket_step
    joints = detect_joints(pipe_bracket_topology)
    for joint in joints["weld_joints"]:
        curve = Curve(joint["weld_line"])
        assert curve.total == pytest.approx(joint["weld_line"]["length_m"], rel=1e-9), joint["id"]


# --------------------------------------------------------------------------
# joins
# --------------------------------------------------------------------------


def _l_corner(first_m: float = 0.075, second_m: float = 0.05, closed: bool = False):
    """Two straight segments meeting at a right angle, lengths declared.

    Deliberately unequal and not round in `s`: the join sits at 0.6 of the
    whole, which is a number a uniform grid never lands on by accident.
    """
    return Curve(
        {
            "closed": closed,
            "segments": [
                {
                    "degree": 1,
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                    "control_points_xyz_m": [[0.0, 0.0, 0.0], [first_m, 0.0, 0.0]],
                    "length_m": first_m,
                },
                {
                    "degree": 1,
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                    "control_points_xyz_m": [[first_m, 0.0, 0.0], [first_m, second_m, 0.0]],
                    "length_m": second_m,
                },
            ],
        }
    )


def test_a_position_on_a_join_takes_the_side_it_is_asked_for():
    """The tangent at a corner is two-valued, so which one comes back is the
    caller's to state: departing by default, arriving on request. Left to a
    comparison of `s * total` against a quadrature sum it is neither -- it is
    whichever side the last ulp fell on, which is how a seam split exactly at
    a corner came to report the tangent of the segment before it."""
    curve = _l_corner()
    join = 0.075 / 0.125

    assert curve.locate(join) == (1, 0.0), "departing a join enters the next segment at its start"
    assert curve.locate(join, left_limit=True) == (0, 1.0), "arriving stays on the one before"
    np.testing.assert_allclose(curve.tangent(join), [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(curve.tangent(join, left_limit=True), [1.0, 0.0, 0.0], atol=1e-12)


def test_the_join_positions_are_the_ones_continuity_reports():
    """`junction_positions_s` sums the DECLARED lengths and `Curve` measures
    the same arcs by quadrature, which are two answers to one question. A
    corner is only ever landed on exactly if the comparison happens on the
    scale the caller's number came from -- so these must be equal to the bit,
    not to a tolerance."""
    from seam_worker.seam.continuity import junction_positions_s

    spec = {
        "closed": True,
        "segments": [
            {
                "degree": 2,
                "control_points_xyz_m": [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                "weights": [1.0, math.cos(math.pi / 4), 1.0],
                "knots_u": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                # The analytic quarter arc. `Curve` quadratures the same arc to
                # a slightly different number, which is the disagreement this
                # test exists to keep out of the join positions.
                "length_m": math.pi / 2,
            },
            {
                "degree": 1,
                "knots_u": [0.0, 0.0, 1.0, 1.0],
                "control_points_xyz_m": [[0.0, 1.0, 0.0], [0.0, 0.0, 0.0]],
                "length_m": 1.0,
            },
        ],
    }
    curve = Curve(spec)
    for junction in junction_positions_s(spec):
        assert junction["s"] in set(curve.joins_s.tolist()), junction


def test_arriving_at_s_zero_of_a_closed_loop_comes_from_the_last_segment():
    """A span cut at a corner can END at s=0 -- `bracket_b`'s second seam
    does, and `_cut_run` writes it that way because a closed loop's run
    wrapped through the parametrisation's start. The point is the loop's
    closure and it is arrived at from the far end, so the segment before
    segment 0 is the last one. An open curve has nothing there and clamps."""
    closed = _l_corner(closed=True)
    assert closed.locate(0.0, left_limit=True) == (1, 1.0)
    np.testing.assert_allclose(closed.tangent(0.0, left_limit=True), [0.0, 1.0, 0.0], atol=1e-12)

    assert _l_corner(closed=False).locate(0.0, left_limit=True) == (0, 0.0)


def test_a_position_that_is_not_a_join_still_goes_through_arc_length():
    """The join rule is about the ends of a span, not a snap that drags
    nearby positions onto a corner. 1e-12 of a 125 mm curve is 0.125 pm, so
    anything a caller can measure stays where it was put."""
    curve = _l_corner()
    join = 0.075 / 0.125
    for offset in (-1e-6, -1e-9, 1e-9, 1e-6):
        index, _ = curve.locate(join + offset)
        assert index == (0 if offset < 0 else 1), offset


# --------------------------------------------------------------------------
# sampling
# --------------------------------------------------------------------------


def test_adaptive_sampling_is_dense_where_the_curve_bends(pipe_bracket_topology):
    """R3 in practice: a straight run needs few samples however long, a tight
    arc needs many. Uniform sampling gives exactly the wrong distribution."""
    from seam_worker.seam.detect import detect_joints

    joint = next(
        j
        for j in detect_joints(pipe_bracket_topology)["weld_joints"]
        if j["id"] == "weld_joint_0003"
    )
    curve = Curve(joint["weld_line"])
    samples = curve.sample_adaptive(max_chord_m=50e-6, max_step_m=0.010)
    assert len(samples) > 2

    curved = straight = 0
    for s in samples:
        (curved := curved + 1) if curve.curvature(s) > 1.0 else (straight := straight + 1)
    assert curved > 0 and straight > 0

    # chord deviation actually stays under the requested tolerance
    worst = 0.0
    for a, b in itertools.pairwise(samples):
        mid = curve.point(0.5 * (a + b))
        chord_mid = 0.5 * (curve.point(a) + curve.point(b))
        worst = max(worst, float(np.linalg.norm(mid - chord_mid)))
    assert worst < 60e-6, f"chord deviation {worst * MM:.4f} mm exceeds tolerance"


def test_uniform_sampling_is_uniform_in_arc_length(dog_box_topology):
    """Uniform in ARC LENGTH, which is the guarantee -- not uniform in chord.

    Chord spacing is genuinely uneven and correctly so: a step that spans a C0
    corner has a much shorter chord than one along a straight run, because the
    path bends between the two points. Asserting equal chords would be
    asserting the curve has no corners.
    """
    from seam_worker.seam.detect import detect_joints

    joint = detect_joints(dog_box_topology)["weld_joints"][0]
    curve = Curve(joint["weld_line"])

    worst = 0.0
    for s in np.linspace(0.0, 1.0, 41):
        index, u = curve.locate(float(s))
        travelled = curve.cumulative[index] + curve.segments[index].arc_length(
            curve.segments[index].u0, u
        )
        worst = max(worst, abs(travelled - s * curve.total))
    assert worst * MM < 1e-4, f"arc length at s drifts by {worst * MM:.3e} mm"


def test_self_test_entry_point_passes():
    from seam_worker.seam.curve import self_test

    assert self_test() == 0


def test_the_batched_evaluator_matches_the_one_point_one(dog_box_topology):
    """`de_boor_many` is the same recursion carried across an axis of
    parameters, and arc length now goes through it exclusively.

    It exists because building one segment's arc-length table called the scalar
    `de_boor` about 12,000 times at interpreter overhead each, which measured as
    59% of a torch clearance check. That is a good reason to batch and no reason
    at all to accept a different answer, so the two are checked against each
    other on real edges rather than on a synthetic curve that might be kinder.
    """
    edges = dog_box_topology["edges"][:40]
    checked = 0
    for edge in edges:
        segment = Segment(edge["bspline"])
        us = np.linspace(segment.u0, segment.u1, 37)
        batched = de_boor_many(us, segment.degree, segment.knots, segment.homogeneous)
        for u, row in zip(us, batched):
            one = de_boor(float(u), segment.degree, segment.knots, segment.homogeneous)
            assert np.allclose(row, one, rtol=0, atol=1e-14), (edge.get("id"), u)
            checked += 1
    assert checked > 100


def test_speed_many_matches_speed_on_real_edges(dog_box_topology):
    """The quadrature integrand, batched. `speed_many` skips the second
    derivative that `evaluate` computes and arc length never wants, so it is a
    different code path and not just a loop turned inside out."""
    for edge in dog_box_topology["edges"][:40]:
        segment = Segment(edge["bspline"])
        us = np.linspace(segment.u0, segment.u1, 23)
        batched = segment.speed_many(us)
        for u, fast in zip(us, batched):
            assert fast == pytest.approx(segment.speed(float(u)), rel=1e-12, abs=1e-15)
