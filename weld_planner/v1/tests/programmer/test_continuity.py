"""Junction continuity: does the weld line turn, and by how much.

The claim under test is narrow and worth stating: chaining guarantees the curve
does not JUMP, this guarantees nothing about whether it TURNS, and a torch cares
about the second. Everything here is measured against real geometry from the two
fixtures rather than against a constructed spline, because the interesting cases
-- an undercut dogbone relief, a rounded column whose arcs are tangent to its
flats -- are exactly the ones nobody would think to construct.
"""

import math

import pytest
from seam_worker.seam.continuity import (
    TANGENT_TOL_DEG,
    annotate_curve,
    break_positions_s,
    junction,
)
from seam_worker.seam.detect import detect_joints


@pytest.fixture(scope="module")
def pipe_bracket_joints(pipe_bracket_topology):
    return detect_joints(pipe_bracket_topology)


@pytest.fixture(scope="module")
def dog_box_joints(dog_box_topology):
    return detect_joints(dog_box_topology)


def _all_junctions(joints_doc):
    out = []
    for joint in joints_doc["weld_joints"]:
        for measured in annotate_curve(joint["weld_line"]):
            out.append((joint["id"], measured))
    return out


# --------------------------------------------------------------------------
# the two shapes that matter
# --------------------------------------------------------------------------


def test_a_dogbone_relief_meets_its_edge_at_a_corner(dog_box_joints):
    """dog_box's lap perimeters are rectangles with 270 degree corner reliefs.

    The relief sweeps PAST tangency, so the arc leaves the straight edge
    perpendicular to it. Eight segments, eight square corners, and the arc must
    stop at every one of them.
    """
    joint = next(j for j in dog_box_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    measured = annotate_curve(joint["weld_line"])

    assert len(measured) == 8, "a closed 8-segment loop has 8 junctions, including the wrap"
    assert all(m["klass"] == "C0" for m in measured)
    for m in measured:
        assert m["tangent_angle_deg"] == pytest.approx(90.0, abs=1e-9)


def test_a_rounded_column_is_tangent_at_every_corner(pipe_bracket_joints):
    """pipe_bracket's column is a rounded square: arcs blended into flats.

    Same segment count as the dogbone, same closed loop, opposite answer. This
    is the pair that shows segment count and curve type cannot classify a
    junction -- only the tangents can.
    """
    joint = next(j for j in pipe_bracket_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    measured = annotate_curve(joint["weld_line"])

    assert len(measured) == 8
    assert all(m["klass"] == "G1" for m in measured)
    for m in measured:
        assert m["tangent_angle_deg"] < TANGENT_TOL_DEG

    # G1 but NOT curvature-continuous: a straight flat has kappa = 0 and the
    # blend arc does not. That jump is the reason the measurement is reported
    # as a number instead of being folded into the class.
    assert any(m["curvature_relative_jump"] > 0.5 for m in measured)


def test_the_gusset_rectangles_are_all_corners(pipe_bracket_joints):
    sharp = [
        j
        for j in pipe_bracket_joints["weld_joints"]
        if len(j["weld_line"]["segments"]) == 4
    ]
    assert len(sharp) == 4, "four gusset-to-plate and gusset-to-column rectangles"
    for joint in sharp:
        measured = annotate_curve(joint["weld_line"])
        assert [m["klass"] for m in measured] == ["C0"] * 4


# --------------------------------------------------------------------------
# what licenses the tolerance
# --------------------------------------------------------------------------


def test_the_distribution_is_bimodal_so_the_tolerance_is_not_a_judgement_call(
    pipe_bracket_joints, dog_box_joints
):
    """Nothing lands between 1e-3 and 1 degree, on either fixture.

    That gap is what makes TANGENT_TOL_DEG uncontroversial: any threshold across
    six orders of magnitude classifies identically. The day a genuinely marginal
    fixture arrives this fails, which is the point -- better a loud failure than
    a classification decided by where the constant happened to sit.
    """
    angles = [m["tangent_angle_deg"] for _, m in _all_junctions(pipe_bracket_joints)]
    angles += [m["tangent_angle_deg"] for _, m in _all_junctions(dog_box_joints)]
    assert angles, "the fixtures must actually have junctions"

    in_the_gap = [a for a in angles if TANGENT_TOL_DEG < a < 1.0]
    assert in_the_gap == [], f"marginal junctions need a deliberate threshold: {in_the_gap}"


def test_a_junction_is_never_labelled_C1_or_C2(pipe_bracket_joints, dog_box_joints):
    """Parametric continuity is not a claim this code can support.

    Every segment carries its source edge's own knot vector out of OCCT, so
    derivative MAGNITUDES across a junction differ for reasons unrelated to the
    shape -- rescaling a knot vector, which moves no point, would change the
    answer. Only the geometric statement is emitted.
    """
    for doc in (pipe_bracket_joints, dog_box_joints):
        for joint in doc["weld_joints"]:
            annotate_curve(joint["weld_line"])
            for segment in joint["weld_line"]["segments"]:
                assert segment.get("continuity_to_next") in {"C0", "G1", None}


def test_tangent_angle_survives_the_small_angle_regime():
    """acos would lose half its digits here; atan2 does not.

    A 1e-6 degree turn is far below the tolerance and must still be measured
    rather than rounded to zero -- otherwise the bimodality test above could
    pass by numerical accident.
    """
    theta = math.radians(1e-6)
    line = {
        "degree": 1,
        "knots_u": [0.0, 0.0, 1.0, 1.0],
        "control_points_xyz_m": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        "length_m": 1.0,
    }
    turned = {
        "degree": 1,
        "knots_u": [0.0, 0.0, 1.0, 1.0],
        "control_points_xyz_m": [[1.0, 0.0, 0.0], [1.0 + math.cos(theta), math.sin(theta), 0.0]],
        "length_m": 1.0,
    }
    measured = junction(line, turned)
    assert measured["tangent_angle_deg"] == pytest.approx(1e-6, rel=1e-6)
    assert measured["klass"] == "G1"
    assert measured["position_gap_m"] == pytest.approx(0.0, abs=1e-12)


def test_parametrisation_does_not_change_the_answer():
    """The same two segments, one with its knots rescaled 100x.

    The geometry is identical, so the classification must be identical. Under a
    C1 test it would not be, which is the whole argument for reporting G1.
    """
    a = {
        "degree": 1,
        "knots_u": [0.0, 0.0, 1.0, 1.0],
        "control_points_xyz_m": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        "length_m": 1.0,
    }
    b_slow = {
        "degree": 1,
        "knots_u": [0.0, 0.0, 1.0, 1.0],
        "control_points_xyz_m": [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        "length_m": 1.0,
    }
    b_fast = dict(b_slow, knots_u=[0.0, 0.0, 100.0, 100.0])

    assert junction(a, b_slow)["klass"] == junction(a, b_fast)["klass"] == "G1"
    assert junction(a, b_slow)["tangent_angle_deg"] == pytest.approx(
        junction(a, b_fast)["tangent_angle_deg"], abs=1e-12
    )


# --------------------------------------------------------------------------
# the open/closed spec decision
# --------------------------------------------------------------------------


def test_an_open_curve_has_no_continuity_on_its_last_segment():
    """Absent, not "C0". There is no next segment to be discontinuous with."""
    curve = {
        "closed": False,
        "segments": [
            {
                "degree": 1,
                "knots_u": [0.0, 0.0, 1.0, 1.0],
                "control_points_xyz_m": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                "length_m": 1.0,
            },
            {
                "degree": 1,
                "knots_u": [0.0, 0.0, 1.0, 1.0],
                "control_points_xyz_m": [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
                "length_m": 1.0,
            },
        ],
    }
    measured = annotate_curve(curve)
    assert len(measured) == 1
    assert curve["segments"][0]["continuity_to_next"] == "C0"
    assert "continuity_to_next" not in curve["segments"][1]


def test_a_closed_curve_wraps_to_segment_zero(pipe_bracket_joints):
    joint = next(j for j in pipe_bracket_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 4)
    assert joint["weld_line"]["closed"] is True
    measured = annotate_curve(joint["weld_line"])
    assert measured[-1]["from_segment_index"] == 3
    assert measured[-1]["to_segment_index"] == 0
    assert measured[-1]["position_gap_m"] < 1e-9, "the wrap junction must actually close"


def test_a_single_segment_loop_has_no_junctions(dog_box_joints):
    """dog_box's two circular joints are one segment each. A circle meeting
    itself is not a junction between two segments, and inventing one would put
    a spurious break at s=0."""
    circle = next(j for j in dog_box_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 1)
    assert annotate_curve(circle["weld_line"]) == []
    assert break_positions_s(circle["weld_line"], 5.0) == []


# --------------------------------------------------------------------------
# break positions -- what seams.py will cut on
# --------------------------------------------------------------------------


def test_break_positions_land_on_the_declared_arc_length_scale(dog_box_joints):
    """Positions must come from the segments' DECLARED length_m sums.

    That is the scale access.py places blocked spans on. Recomputing lengths
    through curve.Curve would be more precise and would put these cuts on a
    subtly different scale than the runs they compose with.
    """
    joint = next(j for j in dog_box_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    breaks = break_positions_s(joint["weld_line"], 5.0)

    segments = joint["weld_line"]["segments"]
    total = sum(s["length_m"] for s in segments)
    expected, travelled = [], 0.0
    for segment in segments[:-1]:
        travelled += segment["length_m"]
        expected.append(travelled / total)
    # the wrap junction is at s=0, not at s=1
    expected = sorted({0.0, *expected})

    assert breaks == pytest.approx(expected, abs=1e-12)


def test_a_tangent_loop_yields_no_breaks(pipe_bracket_joints):
    joint = next(j for j in pipe_bracket_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    assert break_positions_s(joint["weld_line"], 5.0) == []
    # ...but every junction is still a break at a threshold below the tolerance
    assert len(break_positions_s(joint["weld_line"], -1.0)) == 8


def test_the_threshold_is_a_dial(dog_box_joints):
    joint = next(j for j in dog_box_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    assert len(break_positions_s(joint["weld_line"], 5.0)) == 8
    assert break_positions_s(joint["weld_line"], 89.9) == pytest.approx(
        break_positions_s(joint["weld_line"], 5.0)
    )
    assert break_positions_s(joint["weld_line"], 90.1) == [], "nothing turns by more than 90 deg"


# --------------------------------------------------------------------------
# the detector carries it forward
# --------------------------------------------------------------------------


def test_detection_records_junctions_as_evidence(pipe_bracket_joints):
    """`evidence` is the one open object in the schema, which is why the
    measurements live there and only the enum lands on the segment."""
    for joint in pipe_bracket_joints["weld_joints"]:
        junctions = joint["evidence"]["junctions"]
        segments = joint["weld_line"]["segments"]
        assert len(junctions) == len(segments), "closed loops: one junction per segment"
        for measured in junctions:
            assert measured["klass"] in {"C0", "G1"}
            assert measured["position_gap_m"] < 1e-9
        for segment in segments:
            assert segment["continuity_to_next"] in {"C0", "G1"}


def test_continuity_is_not_the_same_claim_as_rail_continuity(pipe_bracket_joints):
    """Groove-normal continuity and weld-line tangent continuity correlate on
    these fixtures and are logically independent. Keeping both means neither
    quietly stands in for the other -- see test_frames.py's rail test."""
    column = next(j for j in pipe_bracket_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 8)
    gusset = next(j for j in pipe_bracket_joints["weld_joints"] if len(j["weld_line"]["segments"]) == 4)

    assert all(s["continuity_to_next"] == "G1" for s in column["weld_line"]["segments"])
    assert column["bisector_rail"]["continuous"] is True

    assert all(s["continuity_to_next"] == "C0" for s in gusset["weld_line"]["segments"])
    assert gusset["bisector_rail"]["continuous"] is False
