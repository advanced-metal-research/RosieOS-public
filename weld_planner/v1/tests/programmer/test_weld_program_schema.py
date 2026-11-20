"""The weld program contract: schema validity, curve invariants, and frame math.

The frame construction in SEAM-MODEL.md section 5 is executed here rather than
trusted. In particular the travel-angle sign convention (positive == drag) is
verified numerically, because it is a coin flip that is cheap to get backwards
and expensive to discover later.

The fixture is now GENERATED from dog_box.step by topology.py + detect.py, so
its numbers are extracted rather than authored. It was previously a hand-made
750 mm rectangle, which hid two real bugs: detect.py emitted weld-line segments
unchained, and the rail's continuity was asserted rather than measured. A
rectangle is chained by construction, so neither could surface.
"""

import itertools
import json
import math

import numpy as np
import pytest
from jsonschema import Draft202012Validator

SCHEMA_FILE = "amr-weld-planner-v1.weld-program.v1.schema.json"
EXAMPLE_FILE = "examples/dog_box_lap_perimeter.json"


@pytest.fixture(scope="module")
def schema(schema_dir):
    return json.loads((schema_dir / SCHEMA_FILE).read_text())


@pytest.fixture(scope="module")
def program(schema_dir):
    return json.loads((schema_dir / EXAMPLE_FILE).read_text())


def test_schema_is_valid_draft_2020_12(schema):
    Draft202012Validator.check_schema(schema)


def test_example_validates(schema, program):
    errors = sorted(Draft202012Validator(schema).iter_errors(program), key=lambda e: list(e.path))
    assert not errors, "\n".join(f"{list(e.path)}: {e.message}" for e in errors[:5])


# --------------------------------------------------------------------------
# curve invariants the JSON Schema cannot express
# --------------------------------------------------------------------------


def _check_curve(curve, name):
    total = 0.0
    for i, seg in enumerate(curve["segments"]):
        n, p = len(seg["control_points_xyz_m"]), seg["degree"]
        knots = seg["knots_u"]
        assert len(knots) == n + p + 1, f"{name}[{i}] knot count"
        assert all(b >= a for a, b in itertools.pairwise(knots)), f"{name}[{i}] non-decreasing"
        assert knots[: p + 1] == [knots[0]] * (p + 1), f"{name}[{i}] clamped start"
        assert knots[-(p + 1) :] == [knots[-1]] * (p + 1), f"{name}[{i}] clamped end"
        if "weights" in seg:
            assert len(seg["weights"]) == n, f"{name}[{i}] weight count"
        if p == 1 and n == 2:
            a, b = seg["control_points_xyz_m"]
            assert math.dist(a, b) == pytest.approx(seg["length_m"], abs=1e-9), f"{name}[{i}] len"
        total += seg.get("length_m", 0.0)
    if "length_m" in curve:
        assert total == pytest.approx(curve["length_m"], abs=1e-9), f"{name} total length"
    return total


def test_curve_invariants(program):
    joint = program["weld_joints"][0]
    assert _check_curve(joint["weld_line"], "weld_line") == pytest.approx(0.821239, abs=1e-6)
    _check_curve(joint["bisector_rail"]["curve"], "bisector_rail")
    _check_curve(program["seams"][0]["reference"]["rail"], "seam.rail")


def test_weld_line_is_a_chained_closed_wire(program):
    """REGRESSION. A curve is "segments joined end to end". detect.py emitted
    them in face-edge exploration order, which is neither traversal order nor
    consistently oriented: every consecutive gap was 35.36 mm while the segment
    count and the 821.239 mm total were both correct."""
    weld_line = program["weld_joints"][0]["weld_line"]
    segs = weld_line["segments"]
    assert weld_line["closed"] is True
    for i, seg in enumerate(segs):
        end = seg["control_points_xyz_m"][-1]
        nxt = segs[(i + 1) % len(segs)]["control_points_xyz_m"][0]
        assert end == pytest.approx(nxt, abs=1e-9), f"gap after segment {i}"


def test_reversed_segments_keep_their_analytic_annotation_consistent(program):
    """Chaining reverses some segments. Poles, knots and the analytic block all
    have to follow, or the annotation contradicts its own spline."""
    for seg in program["weld_joints"][0]["weld_line"]["segments"]:
        analytic = seg.get("analytic")
        poles = seg["control_points_xyz_m"]
        if analytic and analytic["kind"] == "line":
            assert analytic["start_xyz_m"] == pytest.approx(poles[0], abs=1e-9), seg["id"]
            assert analytic["end_xyz_m"] == pytest.approx(poles[-1], abs=1e-9), seg["id"]
        elif analytic and analytic["kind"] in ("arc", "circle"):
            centre = np.array(analytic["center_xyz_m"])
            for pole in (poles[0], poles[-1]):
                radius = np.linalg.norm(np.array(pole) - centre)
                assert radius == pytest.approx(analytic["radius_m"], abs=1e-9), seg["id"]
            assert seg["length_m"] == pytest.approx(
                analytic["radius_m"] * abs(analytic["sweep_angle_rad"]), rel=1e-9
            ), seg["id"]


def test_rail_lies_along_the_groove_face_bisector(program):
    """Enforces the contact-pair vs groove-faces distinction by data.

    Checked at segment ENDPOINTS, which lie exactly on a clamped B-spline. A
    per-segment "pure translation" check only works when every groove face is
    planar; on the real dogbone four of the eight are cylinders, whose normal
    rotates along the arc, so the rail is a concentric offset rather than a
    translation.
    """
    joint = program["weld_joints"][0]
    offset = joint["bisector_rail"]["offset_m"]
    seam_segs = joint["weld_line"]["segments"]
    rail_segs = joint["bisector_rail"]["curve"]["segments"]
    assert len(seam_segs) == len(rail_segs)

    for i, (seam, rail) in enumerate(zip(seam_segs, rail_segs)):
        for which in (0, -1):
            delta = np.subtract(
                rail["control_points_xyz_m"][which], seam["control_points_xyz_m"][which]
            )
            distance = np.linalg.norm(delta)
            assert distance == pytest.approx(offset, abs=1e-9), f"rail segment {i}"
            groove = joint["groove_faces"][i]
            bisector = np.add(
                groove["face_a"]["outward_normal_xyz"], groove["face_b"]["outward_normal_xyz"]
            )
            bisector = bisector / np.linalg.norm(bisector)
            # planar grooves have a constant normal, so the stored one is exact
            if groove["face_a"]["surface_type"] == "plane":
                assert delta / distance == pytest.approx(bisector, abs=1e-6), f"segment {i}"


def test_rail_is_intentionally_discontinuous_at_corners(program):
    """Not a defect. Where the groove normal flips, the bisector flips and the
    rail steps by offset * |b1 - b2| -- 20 mm at a 90 degree flip on a 20 mm
    offset. This is why correspondence is per-segment, and why `closed` and
    `continuous` are measured rather than assumed."""
    rail = program["weld_joints"][0]["bisector_rail"]
    segs = rail["curve"]["segments"]
    assert rail["continuous"] is False
    assert rail["curve"]["closed"] is False
    gaps = [
        float(
            np.linalg.norm(
                np.subtract(
                    seg["control_points_xyz_m"][-1],
                    segs[(i + 1) % len(segs)]["control_points_xyz_m"][0],
                )
            )
        )
        for i, seg in enumerate(segs)
    ]
    assert max(gaps) == pytest.approx(rail["offset_m"], abs=1e-6)


# --------------------------------------------------------------------------
# SEAM-MODEL.md section 5, executed
# --------------------------------------------------------------------------


def _rot(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def seam_frame(point, rail_point, tangent, work_deg, travel_deg):
    """t, r, b = t x r; then r' = R(t, work) r and u = R(b', -travel) r'."""
    t = np.asarray(tangent, float)
    t = t / np.linalg.norm(t)
    d = np.subtract(rail_point, point)
    r = d - np.dot(d, t) * t
    r = r / np.linalg.norm(r)
    rp = _rot(t, math.radians(work_deg)) @ r
    bp = np.cross(t, rp)
    u = _rot(bp, -math.radians(travel_deg)) @ rp
    return t, r, u


def test_positive_travel_angle_is_drag():
    """t=+x, r=+z, work 0, travel +15 deg. The doc's own worked example."""
    t, _r, u = seam_frame([0, 0, 0], [0, 0, 1], [1, 0, 0], 0.0, 15.0)
    want = [math.sin(math.radians(15)), 0.0, math.cos(math.radians(15))]
    assert u == pytest.approx(want, abs=1e-12)
    assert np.dot(u, t) > 0, "torch must sit ahead of the weld point (drag/backhand)"


def test_weave_axis_is_perpendicular_to_travel_and_electrode():
    """b' carries the weave displacement, and must be orthogonal to both the
    travel direction and the electrode axis for ANY work and travel angle --
    otherwise weave would modulate standoff or travel speed."""
    worst = 0.0
    for work in range(-60, 61, 15):
        for travel in range(-30, 31, 10):
            t, r, _ = seam_frame([0, 0, 0], [0, 0, 1], [1, 0, 0], work, travel)
            rp = _rot(t, math.radians(work)) @ r
            bp = np.cross(t, rp)
            u = _rot(bp, -math.radians(travel)) @ rp
            worst = max(worst, abs(np.dot(bp, t)), abs(np.dot(bp, u)))
    assert worst < 1e-12, f"b' not orthogonal (worst {worst:.2e})"


def test_fixture_frame_is_a_45_degree_bisector_on_every_segment(program):
    """Dihedral is 90 degrees all the way round, so the rail direction sits at
    45 degrees out of the seam plane on every segment -- including the arcs,
    where the horizontal component rotates but the vertical stays at 1/sqrt(2)."""
    joint = program["weld_joints"][0]
    seam = program["seams"][0]
    work = seam["orientation"]["work_angle_deg"]["value"]
    travel = seam["orientation"]["travel_angle_deg"]["value"]

    for i, (seg, rail) in enumerate(
        zip(joint["weld_line"]["segments"], joint["bisector_rail"]["curve"]["segments"])
    ):
        p = seg["control_points_xyz_m"][0]
        q = rail["control_points_xyz_m"][0]
        # Tangent at u=0 of a clamped B-spline is along P1 - P0. The chord
        # from first to last pole is NOT the tangent -- on a 270 degree arc it
        # is nowhere near it, and using it distorts the orthogonalised rail
        # direction to sqrt(2/3) instead of sqrt(1/2).
        tangent = np.subtract(seg["control_points_xyz_m"][1], p)
        if np.linalg.norm(tangent) < 1e-9:
            continue
        t, r, u = seam_frame(p, q, tangent, work, travel)
        assert r[2] == pytest.approx(math.sqrt(0.5), abs=1e-6), f"segment {i} not 45 deg"
        assert np.dot(u, t) > 0, f"segment {i}: 10 deg travel should be drag"
        assert u[2] > 0, f"segment {i}: torch must sit above the work"


def test_spin_is_free_by_default(program):
    """R2: a weld constrains 5 DOF. Anything else needs an explicit decision."""
    for seam in program["seams"]:
        spin = seam["orientation"].get("spin", {})
        assert spin.get("mode", "free") == "free", seam["id"]


def test_no_field_smuggles_robot_state(program):
    """R1: the contract must not carry motion. Guards against a well-meaning
    future addition of joint values or poses."""
    banned = ("joint_values", "positions_rad", "quaternion", "pose_", "tcp_", "_rad_per_s")
    blob = json.dumps(program)
    for token in banned:
        assert token not in blob, f"weld program contains robot state: {token!r}"
