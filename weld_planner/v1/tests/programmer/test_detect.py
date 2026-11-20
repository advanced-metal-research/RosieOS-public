"""Stage 2 weld joint detection, against the real dog_box.step geometry.

Five joints, found by face adjacency alone -- antiparallel outward normals,
zero plane separation, overlapping extents. Nothing here recognises shapes.

Two of these tests are regressions for bugs that shipped in the first draft and
were caught only by checking the numbers:

- `test_rail_follows_the_true_local_bisector` -- the rail was translated by one
  bisector frozen at the groove face's uv midpoint. On a 270 degree corner
  relief that put it 26.1 mm out at the arc endpoints, on a 20 mm offset.
- `test_full_cylinder_normal_sign_is_resolvable` -- the convex/concave sign was
  taken from the face centroid, which for a full 360 degree cylinder lies
  exactly on the axis, silently dropping two joints.
"""

import itertools
import json
import math

import numpy as np
import pytest
from jsonschema import Draft202012Validator
from seam_worker.seam.detect import (
    ANTIPARALLEL_DOT,
    _cylinder_normal_sign,
    analyze_pair,
    candidate_pairs,
    detect_joints,
)

MM = 1000.0
DOGBONE_JOINT = "weld_joint_0002"  # solid_0 / solid_2, the top plate


@pytest.fixture(scope="module")
def joints(dog_box_topology):
    return detect_joints(dog_box_topology)


@pytest.fixture(scope="module")
def faces(dog_box_topology):
    return {f["id"]: f for f in dog_box_topology["faces"]}


def true_bisector(faces, point, groove):
    """Groove bisector at a specific point, computed independently of detect.py's
    rail construction. The cylinder case must evaluate the radial direction at
    `point` while taking the convex/concave sign as a property of the face."""
    face_a = faces[f"{groove['face_a']['solid_id']}:{groove['face_a']['face_id']}"]
    n_base = np.array(groove["face_b"]["outward_normal_xyz"])
    if face_a["surface_type"] == "plane":
        n_a = np.array(face_a["outward_normal_xyz"])
    else:
        cyl = face_a["cylinder"]
        axis = np.array(cyl["axis_dir_xyz"])
        origin = np.array(cyl["axis_origin_xyz_m"])
        radial = (point - origin) - np.dot(point - origin, axis) * axis
        n_a = _cylinder_normal_sign(face_a) * radial / np.linalg.norm(radial)
    bis = n_a + n_base
    return bis / np.linalg.norm(bis)


# --------------------------------------------------------------------------
# detection
# --------------------------------------------------------------------------


def test_five_joints_detected(joints):
    s = joints["summary"]
    assert s["joint_count"] == 5
    assert s["near_miss_count"] == 0
    # dog_box has curved faces, and none of them is a contact this cannot
    # handle: every pair of solids that touches produces a joint, so the
    # solid-pair test in `detect_joints` suppresses all of them. A cell that
    # reported "could not analyse" about a contact it DID weld would be worse
    # than silence, because it would be untrue.
    assert s["unsupported_count"] == 0
    # 36, where this was 22. The prefilter no longer drops curved faces before
    # `analyze_pair` can see them -- which is what makes a tube through a hole
    # reportable at all instead of vanishing. `joint_count` is unchanged, and
    # that is the point of pinning both: considering more pairs must not invent
    # joints, only let the detector say what it could not work out.
    assert s["candidate_pairs"] == 36


def test_joint_members_are_the_five_real_contacts(joints):
    pairs = sorted(
        tuple(sorted(m["solid_id"] for m in j["members"])) for j in joints["weld_joints"]
    )
    assert pairs == [
        ("solid_0", "solid_1"),
        ("solid_0", "solid_2"),
        ("solid_0", "solid_3"),
        ("solid_0", "solid_4"),
        ("solid_0", "solid_5"),
    ]


def test_all_joints_are_laps_with_the_block_as_base(joints):
    """Every branch here is 3 mm thin along its contact normal, so all five are
    plates and discs lying flat on the block rather than standing on it."""
    for joint in joints["weld_joints"]:
        assert joint["type"] == "lap", joint["id"]
        base = next(m for m in joint["members"] if m["role"] == "base")
        assert base["solid_id"] == "solid_0", joint["id"]
        assert joint["evidence"]["thickness_ratio"] < 0.25, joint["id"]
        assert joint["contact"]["kind"] == "coincident"
        assert joint["contact"]["gap_mm"]["value"] == pytest.approx(0.0, abs=1e-9)


def test_no_false_joints_between_non_touching_solids(joints):
    """The three plates never touch each other, only the block. A detector keying
    on proximity rather than contact would invent joints here."""
    for joint in joints["weld_joints"]:
        solids = {m["solid_id"] for m in joint["members"]}
        assert "solid_0" in solids, f"{joint['id']} does not involve the block"


# --------------------------------------------------------------------------
# weld line: exact curves, carried through
# --------------------------------------------------------------------------


def test_weld_line_carries_exact_curves(joints):
    """The dogbone stays 4 arcs + 4 lines. Nothing is resampled into a polyline."""
    joint = next(j for j in joints["weld_joints"] if j["id"] == DOGBONE_JOINT)
    segments = joint["weld_line"]["segments"]
    assert len(segments) == 8
    assert joint["weld_line"]["length_m"] * MM == pytest.approx(821.239, abs=1e-3)

    arcs = [s for s in segments if s.get("analytic", {}).get("kind") in ("arc", "circle")]
    lines = [s for s in segments if s.get("analytic", {}).get("kind") == "line"]
    assert len(arcs) == 4 and len(lines) == 4

    for arc in arcs:
        assert arc["degree"] == 2 and "weights" in arc, "arc must stay an exact rational spline"
        assert arc["analytic"]["radius_m"] * MM == pytest.approx(25.0, abs=1e-9)
        assert math.degrees(arc["analytic"]["sweep_angle_rad"]) == pytest.approx(270.0, abs=1e-6)


def test_cylinder_joints_are_single_closed_circles(joints):
    for jid in ("weld_joint_0004", "weld_joint_0005"):
        joint = next(j for j in joints["weld_joints"] if j["id"] == jid)
        assert len(joint["weld_line"]["segments"]) == 1
        assert joint["weld_line"]["length_m"] * MM == pytest.approx(math.pi * 100.0, abs=1e-6)


# --------------------------------------------------------------------------
# groove faces: the distinction that is easy to get silently wrong
# --------------------------------------------------------------------------


def test_groove_faces_are_not_the_contact_faces(joints, faces):
    """The contact pair classifies the joint; the groove faces orient the torch.

    On the top plate the contact is plate-bottom against block-top -- horizontal
    on horizontal. The groove at the weld line is the plate's SIDE face against
    the block's TOP face -- vertical meeting horizontal. Conflating them yields a
    dihedral of 180 degrees and a meaningless rail.
    """
    joint = next(j for j in joints["weld_joints"] if j["id"] == DOGBONE_JOINT)
    branch_contact = next(m for m in joint["members"] if m["role"] == "branch")
    base_contact = next(m for m in joint["members"] if m["role"] == "base")

    for groove in joint["groove_faces"]:
        # face_a is the branch's exposed side face, never its contact face
        assert groove["face_a"]["face_id"] not in branch_contact["face_ids"]
        assert groove["face_a"]["solid_id"] == branch_contact["solid_id"]
        # face_b IS the base's contact face -- the block's top plane is both
        assert groove["face_b"]["face_id"] in base_contact["face_ids"]

        n_a = groove["face_a"]["outward_normal_xyz"]
        n_b = groove["face_b"]["outward_normal_xyz"]
        assert abs(np.dot(n_a, n_b)) < 0.01, "groove normals must be roughly perpendicular"


def test_groove_faces_include_the_cylindrical_reliefs(joints):
    joint = next(j for j in joints["weld_joints"] if j["id"] == DOGBONE_JOINT)
    kinds = sorted(g["face_a"]["surface_type"] for g in joint["groove_faces"])
    assert kinds == ["cylinder"] * 4 + ["plane"] * 4


def test_dihedral_is_constant_ninety(joints):
    """Constant even though half the groove faces are cylinders: the relief axes
    are parallel to the block's top normal, so the rotating radial normal stays
    90 degrees from it all the way round. Constant dihedral, rotating bisector."""
    for joint in joints["weld_joints"]:
        dihedral = joint["dihedral_angle_deg"]
        assert dihedral["kind"] == "constant", joint["id"]
        assert dihedral["value"] == pytest.approx(90.0, abs=1e-6), joint["id"]


# --------------------------------------------------------------------------
# the rail
# --------------------------------------------------------------------------


def test_rail_follows_the_true_local_bisector(joints, faces):
    """REGRESSION. The first draft translated each segment by a single bisector
    sampled at the groove face's uv midpoint. On a 270 degree relief that is
    135 degrees away from the bisector at the arc endpoints, putting the rail
    26.1 mm out on a 20 mm offset -- further wrong than it was long.

    Clamped B-spline endpoints lie exactly on the curve, so checking there is
    exact and needs no evaluator.
    """
    for joint in joints["weld_joints"]:
        offset = joint["bisector_rail"]["offset_m"]
        pairs = zip(
            joint["weld_line"]["segments"],
            joint["bisector_rail"]["curve"]["segments"],
            joint["groove_faces"],
        )
        for seam, rail, groove in pairs:
            for which in (0, -1):
                point = np.array(seam["control_points_xyz_m"][which])
                rail_point = np.array(rail["control_points_xyz_m"][which])
                delta = rail_point - point
                distance = np.linalg.norm(delta)
                assert distance == pytest.approx(offset, abs=1e-9), joint["id"]
                expected = true_bisector(faces, point, groove)
                assert delta / distance == pytest.approx(expected, abs=1e-7), joint["id"]


def test_rail_on_a_cylindrical_groove_is_an_exact_concentric_arc(joints, faces):
    """Not a fit. A rational B-spline circle offsets exactly when its control
    points are scaled radially about the axis, so degree, weights and knots all
    survive untouched."""
    joint = next(j for j in joints["weld_joints"] if j["id"] == DOGBONE_JOINT)
    offset = joint["bisector_rail"]["offset_m"]
    checked = 0
    for seam, rail, groove in zip(
        joint["weld_line"]["segments"],
        joint["bisector_rail"]["curve"]["segments"],
        joint["groove_faces"],
    ):
        face = faces[f"{groove['face_a']['solid_id']}:{groove['face_a']['face_id']}"]
        if face["surface_type"] != "cylinder":
            continue
        cyl = face["cylinder"]
        axis = np.array(cyl["axis_dir_xyz"])
        origin = np.array(cyl["axis_origin_xyz_m"])
        sign = _cylinder_normal_sign(face)
        expected_radius = cyl["radius_m"] + sign * offset / math.sqrt(2.0)

        assert seam["degree"] == rail["degree"] == 2
        assert rail["knots_u"] == seam["knots_u"], "knot vector must be untouched"
        assert rail["weights"] == seam["weights"], "weights must be untouched"

        endpoint = np.array(rail["control_points_xyz_m"][0]) - origin
        radial = endpoint - np.dot(endpoint, axis) * axis
        assert np.linalg.norm(radial) == pytest.approx(expected_radius, abs=1e-9)
        assert rail["length_m"] == pytest.approx(
            seam["length_m"] * expected_radius / cyl["radius_m"], rel=1e-9
        )
        checked += 1
    assert checked == 4


def test_full_cylinder_normal_sign_is_resolvable(faces):
    """REGRESSION. Taking the convex/concave sign from the face centroid fails on
    a full 360 degree cylinder, whose centroid lies exactly on the axis. That
    silently dropped both cylinder joints. Comparing the normal against the
    position it was sampled at has no such degeneracy."""
    cylinders = [f for f in faces.values() if f["surface_type"] == "cylinder"]
    assert cylinders
    for face in cylinders:
        assert "normal_sample_xyz_m" in face, face["id"]
        assert _cylinder_normal_sign(face) in (-1.0, 1.0), face["id"]

    full = [f for f in cylinders if f["solid_id"] in ("solid_4", "solid_5")]
    assert full, "expected the two full cylinders"
    for face in full:
        cyl = face["cylinder"]
        axis = np.array(cyl["axis_dir_xyz"])
        rel = np.array(face["centroid_xyz_m"]) - np.array(cyl["axis_origin_xyz_m"])
        radial = rel - np.dot(rel, axis) * axis
        assert np.linalg.norm(radial) < 1e-9, "centroid should be on the axis -- the trap"
        assert _cylinder_normal_sign(face) == 1.0


def test_rail_segment_count_matches_the_weld_line(joints):
    """Per-segment correspondence requires equal counts -- SEAM-MODEL.md 5."""
    for joint in joints["weld_joints"]:
        assert len(joint["bisector_rail"]["curve"]["segments"]) == len(
            joint["weld_line"]["segments"]
        )


# --------------------------------------------------------------------------
# diagnostics
# --------------------------------------------------------------------------


def test_rejections_carry_a_readable_reason(dog_box_topology):
    """"Not a joint" is useless for manual authoring. WEBUI.md section 9."""
    verdicts = [analyze_pair(a, b) for a, b in candidate_pairs(dog_box_topology)]
    rejected = [v for v in verdicts if v["verdict"] == "rejected"]
    assert rejected
    for verdict in rejected:
        assert verdict["reason"], verdict
        assert any(
            token in verdict["reason"]
            for token in ("normals", "apart", "overlap", "sliver", "same solid")
        ), verdict["reason"]


def test_antiparallel_threshold_rejects_perpendicular_faces(dog_box_topology):
    verdicts = [analyze_pair(a, b) for a, b in candidate_pairs(dog_box_topology)]
    for verdict in verdicts:
        if verdict.get("normal_dot") is not None and verdict["verdict"] == "contact":
            assert verdict["normal_dot"] <= ANTIPARALLEL_DOT


def test_near_miss_channel_exists_and_is_empty_here(joints):
    """dog_box mates exactly, so nothing lands in the gap band. The channel is
    still asserted so the shape stays part of the contract."""
    assert joints["near_misses"] == []
    assert joints["unsupported"] == []


# --------------------------------------------------------------------------
# the point of all of it: output drops straight into the contract
# --------------------------------------------------------------------------


def test_detected_joints_validate_against_the_weld_program_schema(joints, schema_dir):
    """Stage 2 output must be usable as `weld_joints[]` with no massaging."""
    schema = json.loads(
        (schema_dir / "amr-weld-planner-v1.weld-program.v1.schema.json").read_text()
    )
    program = {
        "schema": "amr-weld-planner-v1.weld-program.v1",
        "program_id": "detected_dog_box",
        "source": joints["source"],
        "workpiece": {"frame_id": "dog_box_origin"},
        "weld_joints": joints["weld_joints"],
        "seams": [
            {
                "id": "seam_0001",
                "weld_joint_ref": joints["weld_joints"][0]["id"],
                "curve": {"from_weld_joint": True},
                "reference": {
                    "mode": "rail_curve",
                    "rail": joints["weld_joints"][0]["bisector_rail"]["curve"],
                    "correspondence": "normalized_arc_length",
                    "semantics": "bisector",
                },
                "orientation": {
                    "work_angle_deg": {"kind": "constant", "value": 0.0},
                    "travel_angle_deg": {"kind": "constant", "value": 10.0},
                    "spin": {"mode": "free"},
                },
                "process": {
                    "method": "GMAW",
                    "travel_speed_mm_s": {"kind": "constant", "value": 8.0},
                },
            }
        ],
    }
    errors = sorted(Draft202012Validator(schema).iter_errors(program), key=lambda e: list(e.path))
    assert not errors, "\n".join(f"{list(e.path)}: {e.message}" for e in errors[:5])


def test_self_test_entry_point_passes():
    from seam_worker.seam.detect import self_test

    assert self_test() == 0


# --------------------------------------------------------------------------
# boundary loops: a contact face with a hole in it
# --------------------------------------------------------------------------
#
# A pipe standing on a plate touches it through an annulus, so the contact
# region has TWO boundaries: the outer circle, which is the fillet weld, and the
# bore. Before this, `chain_edges` returned None for the pair and every weld on
# an exhaust manifold -- five of them, all real, all exactly coincident -- was
# reported as unsupported with a message about groove faces that had never been
# reached.


def _line_edge(edge_id, start, end):
    """The fields `chain_loops` and `_loop_bbox` read, and nothing else."""
    return {
        "id": edge_id,
        "start_xyz_m": list(start),
        "end_xyz_m": list(end),
        "length_m": math.dist(start, end),
        "bspline": {"control_points_xyz_m": [list(start), list(end)]},
    }


def _square_loop(prefix, half):
    corners = [(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)]
    return [
        _line_edge(f"{prefix}_{i}", corners[i], corners[(i + 1) % 4]) for i in range(4)
    ]


def test_two_boundaries_come_back_as_two_loops():
    from seam_worker.seam.detect import chain_loops

    outer = _square_loop("outer", 0.050)
    inner = _square_loop("inner", 0.020)
    # Interleaved and mid-loop first: exploration order is neither traversal
    # order nor grouped by loop, which is the whole reason for chaining.
    shuffled = [inner[2], outer[1], outer[3], inner[0], outer[0], inner[3], outer[2], inner[1]]

    loops = chain_loops(shuffled)
    assert len(loops) == 2, [[e["id"] for e, _ in c] for c, _ in loops]
    for chain, closed in loops:
        assert closed, [e["id"] for e, _ in chain]
        assert len(chain) == 4
        # Consecutive segments actually meet, which is what a bag of edges with
        # the right total length would not do.
        for (edge, flipped), (nxt, nxt_flipped) in itertools.pairwise(chain):
            tail = edge["start_xyz_m"] if flipped else edge["end_xyz_m"]
            head = nxt["end_xyz_m"] if nxt_flipped else nxt["start_xyz_m"]
            assert math.dist(tail, head) < 1e-9


def test_the_hole_is_labelled_inner_and_the_perimeter_outer():
    from seam_worker.seam.detect import _boundary_kinds, chain_loops

    loops = chain_loops(_square_loop("outer", 0.050) + _square_loop("inner", 0.020))
    kinds = dict(
        zip(
            [round(sum(e["length_m"] for e, _ in chain), 6) for chain, _ in loops],
            _boundary_kinds(loops),
        )
    )
    assert kinds[round(4 * 0.100, 6)] == "outer"
    assert kinds[round(4 * 0.040, 6)] == "inner"


def test_side_by_side_boundaries_are_both_outer():
    """Two contact patches on one face are two boundaries, not a hole in one.
    Containment is the test, so neither encloses the other and neither is a bore."""
    from seam_worker.seam.detect import _boundary_kinds, chain_loops

    left = _square_loop("left", 0.010)
    right = [
        _line_edge(e["id"] + "_r", [c + 0.5 for c in e["start_xyz_m"]], [c + 0.5 for c in e["end_xyz_m"]])
        for e in _square_loop("right", 0.010)
    ]
    assert _boundary_kinds(chain_loops(left + right)) == ["outer", "outer"]


def test_a_single_loop_is_still_labelled_single_and_chains_alone():
    """The regression guard for every workpiece that has no holes: dog_box must
    behave exactly as it did, ids and labels included."""
    from seam_worker.seam.detect import _boundary_kinds, chain_edges, chain_loops

    single = _square_loop("only", 0.050)
    assert _boundary_kinds(chain_loops(single)) == ["single"]
    chained = chain_edges(single)
    assert chained is not None and chained[1] is True

    # And the documented contract of `chain_edges` itself is unchanged: it is
    # what continuity.py, span.py and seams.py are written against.
    assert chain_edges(single + _square_loop("hole", 0.020)) is None
    assert chain_edges([]) is None


def test_dog_box_joints_are_all_single_boundary(joints):
    """No dog_box contact face has a hole, so the multi-loop path must not have
    changed anything: five joints, no boundary suffix in any label."""
    assert len(joints["weld_joints"]) == 5
    for joint in joints["weld_joints"]:
        assert joint["evidence"]["contact_boundary"] == "single", joint["id"]
        assert "boundary" not in joint["label"], joint["label"]


def test_a_failed_loop_reports_what_actually_failed(dog_box_topology):
    """The reason a pair produced nothing has to name the step that failed.

    The old code had four `return None` paths behind one hardcoded sentence
    about groove faces, and it cost real time: a chaining failure on a manifold
    was investigated in `groove_faces_for_edge`, which was working correctly."""
    from seam_worker.seam.detect import build_joints

    faces = {f["id"]: f for f in dog_box_topology["faces"]}
    branch = next(iter(faces))
    stripped = {
        **dog_box_topology,
        "faces": [
            {**f, "edge_uses": []} if f["id"] == branch else f
            for f in dog_box_topology["faces"]
        ],
    }
    built, problems = build_joints(
        stripped, {"branch": branch, "base": branch, "verdict": "contact"}, 1, 0.020
    )
    assert built == []
    assert problems and "non-seam edges" in problems[0], problems


# --------------------------------------------------------------------------
# non-circular seams: a rail for any groove face
# --------------------------------------------------------------------------
#
# The rail is the work-angle reference, and it used to be buildable only against
# a planar groove face (constant normal) or a cylinder whose axis is parallel to
# the base normal (closed-form concentric arc). Anything else -- a cone, a torus,
# the `extrusion` surface OCCT reports for an elliptical tube -- returned None,
# and `build_joints` then discarded the whole joint, weld line included. An
# elliptical tube standing on a plate produced zero welds.
#
# There is no exact answer available: the offset of an ellipse is not an ellipse
# and not a rational curve of the same degree. So the general path approximates
# and MEASURES, refining by knot insertion until the error is inside
# `RAIL_TOLERANCE_DEG` of induced work angle.


@pytest.fixture(scope="module")
def ellipse_tube_topology(tmp_path_factory):
    """A plate with an elliptical tube standing on it: planar contact, and a
    groove face that is neither a plane nor a cylinder."""
    import cadquery as cq
    from seam_worker.cad.topology import extract_topology

    plate = cq.Workplane("XY").box(200, 120, 10, centered=(True, True, False)).val()
    tube = cq.Workplane("XY").ellipse(30, 20).ellipse(27, 17).extrude(60).translate((0, 0, 10)).val()
    path = tmp_path_factory.mktemp("ellipse") / "ellipse_tube.step"
    cq.exporters.export(cq.Compound.makeCompound([plate, tube]), str(path))
    return extract_topology(path)


def test_an_elliptical_seam_is_detected(ellipse_tube_topology):
    """The regression for the whole feature: this was 0 joints and 2 unsupported."""
    found = detect_joints(ellipse_tube_topology)
    assert found["summary"]["joint_count"] == 2, found["unsupported"]
    assert found["summary"]["unsupported_count"] == 0

    outer = found["weld_joints"][0]
    assert outer["type"] == "tee"
    # The weld line is the exact ellipse, carried as the rational spline it is --
    # the approximation is in the RAIL, never in the weld path.
    assert outer["weld_line"]["closed"] is True
    assert outer["weld_line"]["length_m"] * MM == pytest.approx(158.7, abs=0.5)
    assert outer["dihedral_angle_deg"] == {"kind": "constant", "value": 90.0}


def test_the_elliptical_rail_states_and_meets_its_tolerance(ellipse_tube_topology):
    """An approximate rail is only usable because its error is measured. Pole
    offsetting alone measured 5.97 mm out on a 20 mm offset -- a 17 degree work
    angle -- so the refinement is what makes this honest rather than the
    reporting."""
    import math

    from seam_worker.seam.detect import RAIL_TOLERANCE_DEG

    found = detect_joints(ellipse_tube_topology)
    budget_mm = 20.0 * math.tan(math.radians(RAIL_TOLERANCE_DEG))
    for joint in found["weld_joints"]:
        rail = joint["bisector_rail"]
        # On the rail, not on its segments: segments are the shared curve
        # definition and a weld line must not carry rail-only fields.
        assert rail["approximate"] is True
        assert rail["within_tolerance"] is True, rail["max_deviation_mm"]
        assert rail["max_deviation_mm"] <= budget_mm, rail["max_deviation_mm"]
        for segment in rail["curve"]["segments"]:
            assert "max_deviation_mm" not in segment
            # Refinement happened: an unrefined ellipse has a handful of poles.
            assert len(segment["control_points_xyz_m"]) > 20


def test_knot_insertion_does_not_move_the_curve(ellipse_tube_topology):
    """The property the whole refinement rests on. Boehm insertion adds poles and
    must leave the curve identical; if it did not, the rail would be offset from
    a curve that is no longer the weld line."""
    from seam_worker.seam.curve import Segment
    from seam_worker.seam.detect import _refine_once

    joint = detect_joints(ellipse_tube_topology)["weld_joints"][0]
    original = joint["weld_line"]["segments"][0]
    refined = _refine_once(original)

    assert len(refined["control_points_xyz_m"]) > len(original["control_points_xyz_m"])
    before, after = Segment(original), Segment(refined)
    for i in range(21):
        u = before.u0 + (before.u1 - before.u0) * i / 20
        assert math.dist(list(before.point(u)), list(after.point(u))) < 1e-9, u


def test_the_exact_rail_paths_are_still_exact(joints):
    """dog_box is planes and cylinders, both of which have closed forms. None of
    it may be routed through the approximation -- an exact rail is never traded
    for a measured one."""
    for joint in joints["weld_joints"]:
        rail = joint["bisector_rail"]
        assert "approximate" not in rail, joint["id"]
        assert "max_deviation_mm" not in rail, joint["id"]
        for segment in rail["curve"]["segments"]:
            assert "max_deviation_mm" not in segment, joint["id"]


@pytest.fixture(scope="module")
def spline_boss_topology(tmp_path_factory):
    """A plate with a lofted boss whose side face is a genuine B-spline surface.

    Not an extrusion of a spline outline -- that is a `SurfaceOfExtrusion`, which
    the elliptical fixture already covers. A loft between two DIFFERENT closed
    splines gives a real `bspline` surface, and its bottom edge a `bspline` curve,
    so this is the case where neither the groove face nor the weld line has any
    closed form at all.
    """
    import cadquery as cq
    from cadquery.occ_impl.shapes import Edge, Solid, Wire
    from seam_worker.cad.topology import extract_topology

    def ring(points, z):
        return Wire.assembleEdges(
            [Edge.makeSpline([cq.Vector(x, y, z) for x, y in points], periodic=True)]
        )

    plate = cq.Workplane("XY").box(200, 120, 10, centered=(True, True, False)).val()
    boss = Solid.makeLoft(
        [
            ring([(35, 0), (12, 26), (-20, 22), (-33, 0), (-14, -25), (18, -24)], 10.0),
            ring([(26, 0), (9, 19), (-15, 16), (-24, 0), (-10, -18), (13, -17)], 70.0),
        ],
        ruled=False,
    )
    path = tmp_path_factory.mktemp("spline") / "spline_boss.step"
    cq.exporters.export(cq.Compound.makeCompound([plate, boss]), str(path))
    return extract_topology(path)


def test_a_spline_seam_on_a_spline_surface_is_detected(spline_boss_topology):
    """The hardest shape available: no analytic curve, no analytic surface.

    `surface_types` here is ['bspline', 'plane'] and `curve_types` ['bspline',
    'line'], so nothing in this joint has a closed form except the plate it sits
    on. It still yields a weld line, a dihedral and a rail whose error is stated.
    """
    assert spline_boss_topology["summary"]["surface_types"] == ["bspline", "plane"]

    found = detect_joints(spline_boss_topology)
    assert found["summary"]["joint_count"] == 1, found["unsupported"]
    assert found["summary"]["unsupported_count"] == 0

    joint = found["weld_joints"][0]
    assert joint["groove_faces"][0]["face_a"]["surface_type"] == "bspline"
    assert joint["weld_line"]["length_m"] * MM == pytest.approx(194.4, abs=1.0)
    assert joint["bisector_rail"]["approximate"] is True
    assert joint["bisector_rail"]["within_tolerance"] is True


def test_every_surface_type_the_extractor_emits_is_in_the_contract(schema_dir):
    """The other half of the same bug. A `bezier`, `extrusion`, `offset` or
    `revolution` face validated nowhere, because the enum named seven of the
    eleven kinds cadquery can report -- so an elliptical tube failed the schema
    even once detection handled it."""
    from cadquery.occ_impl.shape_protocols import geom_LUT_FACE

    schema = json.loads((schema_dir / "amr-weld-planner-v1.weld-program.v1.schema.json").read_text())
    allowed = set(schema["$defs"]["groove_face"]["properties"]["surface_type"]["enum"])
    emitted = {name.lower() for name in geom_LUT_FACE.values()}
    assert emitted <= allowed, f"the extractor can emit {sorted(emitted - allowed)}"
