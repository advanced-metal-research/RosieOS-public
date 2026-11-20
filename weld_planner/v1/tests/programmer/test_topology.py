"""Stage 1 topology extraction, against the real dog_box.step geometry.

Every expected number here was read out of the STEP file, not assumed. The
hand-authored weld-program fixture originally claimed a 750 mm rectangular
perimeter for the top plate; the real face is a dogbone with 270 degree corner
reliefs and an 821.239 mm perimeter. These tests exist so that kind of drift
fails loudly.
"""

import math

import numpy as np
import pytest

MM = 1000.0  # metres -> millimetres, for readable expectations

# Deterministic ids come from a volume-descending, centroid-ascending sort.
# solid_1, _2 and _3 share a volume, so centroid breaks the tie: the y=-76.5
# plate, then the top plate at y=0, then the y=+76.5 plate.
EXPECTED_SOLIDS = {
    "solid_0": {"volume_mm3": 6750000, "faces": 6, "bbox": (-150, -75, 0, 150, 75, 150)},
    "solid_1": {"volume_mm3": 62671.5, "faces": 10, "bbox": (-125, -78, 14, 125, -75, 139)},
    "solid_2": {"volume_mm3": 62671.5, "faces": 10, "bbox": (-125, -62.5, 150, 125, 62.5, 153)},
    "solid_3": {"volume_mm3": 62671.5, "faces": 10, "bbox": (-125, 75, 14, 125, 78, 139)},
    "solid_4": {"volume_mm3": 23561.9, "faces": 3, "bbox": (-153, -50, 25, -150, 50, 125)},
    "solid_5": {"volume_mm3": 23561.9, "faces": 3, "bbox": (150, -50, 25, 153, 50, 125)},
}

TOP_PLATE_BOTTOM_FACE = "solid_2:face_0004"


def test_solid_inventory(dog_box_topology):
    solids = {s["id"]: s for s in dog_box_topology["solids"]}
    assert set(solids) == set(EXPECTED_SOLIDS)
    for sid, want in EXPECTED_SOLIDS.items():
        got = solids[sid]
        assert got["volume_m3"] * 1e9 == pytest.approx(want["volume_mm3"], rel=1e-4), sid
        assert len(got["face_ids"]) == want["faces"], sid
        bbox = [v * MM for v in got["bbox_min_xyz_m"] + got["bbox_max_xyz_m"]]
        assert bbox == pytest.approx(list(want["bbox"]), abs=1e-6), sid


def test_summary_matches_contents(dog_box_topology):
    s = dog_box_topology["summary"]
    assert s["solid_count"] == len(dog_box_topology["solids"]) == 6
    assert s["face_count"] == len(dog_box_topology["faces"]) == 42
    assert s["edge_count"] == len(dog_box_topology["edges"]) == 90
    assert s["surface_types"] == ["cylinder", "plane"]
    assert s["curve_types"] == ["circle", "line"]


def test_extraction_is_deterministic(dog_box_topology, repo_root):
    """Identifiers must be stable across runs. OCCT traversal order is not
    guaranteed, so ids are assigned from sorted geometry rather than iteration
    order, and every entity carries a content hash."""
    from seam_worker.cad.topology import extract_topology

    again = extract_topology(repo_root / "cadquery" / "v1" / "step" / "dog_box.step")
    for key in ("solids", "faces", "edges"):
        assert [e["id"] for e in again[key]] == [e["id"] for e in dog_box_topology[key]]
        assert [e["geom_key"] for e in again[key]] == [e["geom_key"] for e in dog_box_topology[key]]


def test_geom_keys_are_unique(dog_box_topology):
    for key in ("solids", "faces", "edges"):
        keys = [e["geom_key"] for e in dog_box_topology[key]]
        assert len(set(keys)) == len(keys), f"{key} has colliding geom_keys"


# --------------------------------------------------------------------------
# curves stay analytic (GOAL.md R4)
# --------------------------------------------------------------------------


def test_knot_vector_invariant(dog_box_topology):
    """Clamped B-spline: len(knots) == n_control_points + degree + 1."""
    for edge in dog_box_topology["edges"]:
        bs = edge["bspline"]
        n, p = len(bs["control_points_xyz_m"]), bs["degree"]
        assert len(bs["knots_u"]) == n + p + 1, edge["id"]
        assert all(b >= a for a, b in zip(bs["knots_u"], bs["knots_u"][1:])), edge["id"]
        assert bs["knots_u"][: p + 1] == [bs["knots_u"][0]] * (p + 1), edge["id"]
        assert bs["knots_u"][-(p + 1) :] == [bs["knots_u"][-1]] * (p + 1), edge["id"]
        if "weights" in bs:
            assert len(bs["weights"]) == n, edge["id"]
            assert all(w > 0 for w in bs["weights"]), edge["id"]


def test_lines_are_degree_one_with_two_poles(dog_box_topology):
    for edge in dog_box_topology["edges"]:
        if edge["curve_type"] != "line":
            continue
        bs = edge["bspline"]
        assert bs["degree"] == 1 and len(bs["control_points_xyz_m"]) == 2, edge["id"]
        assert "weights" not in bs, f"{edge['id']} line should be non-rational"
        a, b = bs["control_points_xyz_m"]
        span = math.dist(a, b)
        assert span == pytest.approx(edge["length_m"], rel=1e-12), edge["id"]


def test_circles_are_exact_rational_quadratics(dog_box_topology):
    """A circle must come out exact, not tessellated: length == R * sweep to
    machine precision. This is the property a polyline representation loses."""
    seen = 0
    for edge in dog_box_topology["edges"]:
        if edge["curve_type"] != "circle":
            continue
        bs, an = edge["bspline"], edge["analytic"]
        assert bs["degree"] == 2, edge["id"]
        assert "weights" in bs, f"{edge['id']} circle must be rational"
        expected = an["radius_m"] * abs(an["sweep_angle_rad"])
        assert edge["length_m"] == pytest.approx(expected, rel=1e-12), edge["id"]
        seen += 1
    assert seen > 0


def test_analytic_annotation_agrees_with_endpoints(dog_box_topology):
    """start/end, the analytic block and the B-spline poles must all be in the
    same direction -- the underlying curve's.

    52 of 62 line edges in this file are TopAbs_REVERSED, where cadquery's
    startPoint() returns the curve's LAST point. Using it would make the record
    contradict its own spline.
    """
    for edge in dog_box_topology["edges"]:
        an = edge.get("analytic")
        if not an or an["kind"] != "line":
            continue
        assert an["start_xyz_m"] == pytest.approx(edge["start_xyz_m"], abs=1e-12), edge["id"]
        assert an["end_xyz_m"] == pytest.approx(edge["end_xyz_m"], abs=1e-12), edge["id"]
        poles = edge["bspline"]["control_points_xyz_m"]
        assert poles[0] == pytest.approx(edge["start_xyz_m"], abs=1e-9), edge["id"]
        assert poles[-1] == pytest.approx(edge["end_xyz_m"], abs=1e-9), edge["id"]


# --------------------------------------------------------------------------
# surfaces: the data the prior extractor never fetched
# --------------------------------------------------------------------------


def test_every_face_carries_surface_parameters(dog_box_topology):
    for face in dog_box_topology["faces"]:
        kind = face["surface_type"]
        assert kind in ("plane", "cylinder"), face["id"]
        assert kind in face, f"{face['id']} missing its {kind} parameter block"
        assert len(face["uv_bounds"]) == 4
    planes = [f for f in dog_box_topology["faces"] if f["surface_type"] == "plane"]
    assert len(planes) == 28


def test_outward_normals_point_away_from_solid(dog_box_topology):
    """The critical correctness property for stage 2.

    A face carrying TopAbs_REVERSED uses the opposite sense to its underlying
    surface. Getting that wrong silently inverts every dihedral angle
    downstream, so this checks the result geometrically rather than trusting
    the orientation flag: for these convex-hull-ish solids, an outward normal
    must have a positive dot with (face centroid - solid centroid).
    """
    solids = {s["id"]: s for s in dog_box_topology["solids"]}
    checked = 0
    for face in dog_box_topology["faces"]:
        if face["surface_type"] != "plane":
            continue
        n = face["outward_normal_xyz"]
        assert math.isclose(math.dist(n, (0, 0, 0)), 1.0, abs_tol=1e-9), face["id"]
        sc = solids[face["solid_id"]]["centroid_xyz_m"]
        fc = face["centroid_xyz_m"]
        outward = sum(ni * (fi - si) for ni, fi, si in zip(n, fc, sc))
        assert outward > 0, f"{face['id']} normal points inward (dot {outward:+.6f})"
        checked += 1
    assert checked == 28


def test_normal_sample_point_lies_on_its_face(dog_box_topology):
    """The normal is stored with the point it was evaluated at.

    A normal without its location is ambiguous on any curved surface, and stage
    2 needs the pair to decide whether a cylindrical face is convex or concave.
    The face centroid cannot answer that: a full 360 degree cylinder has its
    centroid exactly on the axis.
    """
    for face in dog_box_topology["faces"]:
        sample = face["normal_sample_xyz_m"]
        assert len(sample) == 3, face["id"]
        lo = face["bbox_min_xyz_m"]
        hi = face["bbox_max_xyz_m"]
        assert all(a - 1e-9 <= s <= b + 1e-9 for s, a, b in zip(sample, lo, hi)), face["id"]

        if face["surface_type"] == "cylinder":
            cyl = face["cylinder"]
            axis = np.array(cyl["axis_dir_xyz"])
            rel = np.array(sample) - np.array(cyl["axis_origin_xyz_m"])
            radial = rel - np.dot(rel, axis) * axis
            assert np.linalg.norm(radial) == pytest.approx(cyl["radius_m"], abs=1e-9), face["id"]
        elif face["surface_type"] == "plane":
            plane = face["plane"]
            offset = np.dot(
                np.array(sample) - np.array(plane["origin_xyz_m"]), np.array(plane["normal_xyz"])
            )
            assert abs(offset) < 1e-9, face["id"]


def test_plane_normal_matches_face_normal_up_to_sign(dog_box_topology):
    for face in dog_box_topology["faces"]:
        if face["surface_type"] != "plane":
            continue
        pn, fn = face["plane"]["normal_xyz"], face["outward_normal_xyz"]
        dot = sum(a * b for a, b in zip(pn, fn))
        assert abs(abs(dot) - 1.0) < 1e-9, face["id"]


# --------------------------------------------------------------------------
# adjacency: what joint detection consumes
# --------------------------------------------------------------------------


def test_edge_face_adjacency_is_symmetric(dog_box_topology):
    faces = {f["id"]: f for f in dog_box_topology["faces"]}
    edges = {e["id"]: e for e in dog_box_topology["edges"]}
    for face in faces.values():
        for eid in face["edge_ids"]:
            assert face["id"] in edges[eid]["face_ids"], f"{eid} !-> {face['id']}"
    for edge in edges.values():
        assert edge["face_ids"], f"{edge['id']} bounds no face"
        for fid in edge["face_ids"]:
            assert edge["id"] in faces[fid]["edge_ids"], f"{fid} !-> {edge['id']}"


def test_every_edge_has_exactly_two_uses(dog_box_topology):
    """A closed solid is 2-manifold: every edge is used exactly twice.

    Not "bounds two distinct faces". A seam edge on a periodic surface is used
    twice by the SAME face, once forward and once reversed -- correct topology,
    not an open shell. cadquery's Face.Edges() hides this by deduplicating, so
    the extractor explores raw OCCT instead.
    """
    for edge in dog_box_topology["edges"]:
        assert edge["use_count"] == 2, f"{edge['id']} has {edge['use_count']} uses"
        assert len(edge["face_ids"]) == 2, edge["id"]


def test_seam_edges_are_flagged(dog_box_topology):
    """The two cylinders each carry one seam line. Stage 2 must skip these when
    tracing real boundaries -- a cylinder's seam is a parametrization artifact,
    not a physical edge."""
    seams = [e for e in dog_box_topology["edges"] if e["is_seam"]]
    assert {e["solid_id"] for e in seams} == {"solid_4", "solid_5"}
    for edge in seams:
        assert edge["curve_type"] == "line", edge["id"]
        assert len(set(edge["face_ids"])) == 1, f"{edge['id']} seam should be used twice by one face"
    non_seam = [e for e in dog_box_topology["edges"] if not e["is_seam"]]
    for edge in non_seam:
        assert len(set(edge["face_ids"])) == 2, f"{edge['id']} should bound two distinct faces"


def test_edge_uses_carry_orientation(dog_box_topology):
    """Stage 2 needs orientation to order edges into a directed wire."""
    for face in dog_box_topology["faces"]:
        assert face["edge_uses"], face["id"]
        for use in face["edge_uses"]:
            assert use["orientation"] in ("forward", "reversed"), face["id"]
    cyl = next(
        f
        for f in dog_box_topology["faces"]
        if f["solid_id"] == "solid_4" and f["surface_type"] == "cylinder"
    )
    assert len(cyl["edge_uses"]) == 4, "cylinder face uses its seam edge twice"
    assert sum(1 for u in cyl["edge_uses"] if u["seam"]) == 2


def test_coincident_cross_solid_face_pairs(dog_box_topology):
    """The five weld joints in the assembly, found from first principles:
    antiparallel normals, zero plane separation, overlapping extents.

    Nothing about shape recognition is involved. This is what the existing
    extractor cannot do -- not because it fails at it, but because it never
    reads surface geometry at all.
    """
    planes = [f for f in dog_box_topology["faces"] if f["surface_type"] == "plane"]
    pairs = []
    for i, a in enumerate(planes):
        for b in planes[i + 1 :]:
            if a["solid_id"] == b["solid_id"]:
                continue
            na, nb = a["outward_normal_xyz"], b["outward_normal_xyz"]
            if sum(x * y for x, y in zip(na, nb)) > -0.999:
                continue
            oa, ob = a["plane"]["origin_xyz_m"], b["plane"]["origin_xyz_m"]
            if abs(sum(n * (x - y) for n, x, y in zip(na, oa, ob))) > 1e-9:
                continue
            amin, amax = a["bbox_min_xyz_m"], a["bbox_max_xyz_m"]
            bmin, bmax = b["bbox_min_xyz_m"], b["bbox_max_xyz_m"]
            if any(lo > hi + 1e-9 for lo, hi in zip(amin, bmax)):
                continue
            if any(lo > hi + 1e-9 for lo, hi in zip(bmin, amax)):
                continue
            pairs.append(tuple(sorted((a["solid_id"], b["solid_id"]))))

    assert sorted(pairs) == [
        ("solid_0", "solid_1"),
        ("solid_0", "solid_2"),
        ("solid_0", "solid_3"),
        ("solid_0", "solid_4"),
        ("solid_0", "solid_5"),
    ]


def test_top_plate_contact_face_is_a_dogbone(dog_box_topology):
    """Locks in the real geometry.

    The weld-program fixture was hand-authored from bounding boxes and claimed a
    750 mm rectangular perimeter. The actual face is a dogbone: four 270 degree
    reliefs at R=25 mm plus four lines, 821.239 mm around, 20890.5 mm^2.

    Note this face IS 4 arcs + 4 lines, which is exactly the shape the existing
    extractor's dogbone rule matches. It would find this outline -- as a
    standalone shape, with no knowledge that it is a joint and no dihedral.
    """
    face = next(f for f in dog_box_topology["faces"] if f["id"] == TOP_PLATE_BOTTOM_FACE)
    edges = {e["id"]: e for e in dog_box_topology["edges"]}
    bounding = [edges[eid] for eid in face["edge_ids"]]

    assert face["area_m2"] * 1e6 == pytest.approx(20890.5, abs=0.1)
    assert face["outward_normal_xyz"] == [-0.0, -0.0, -1.0]

    kinds = sorted(e["curve_type"] for e in bounding)
    assert kinds == ["circle"] * 4 + ["line"] * 4

    perimeter = sum(e["length_m"] for e in bounding) * MM
    assert perimeter == pytest.approx(821.239, abs=1e-3)

    arcs = [e for e in bounding if e["curve_type"] == "circle"]
    for arc in arcs:
        assert arc["analytic"]["radius_m"] * MM == pytest.approx(25.0, abs=1e-9)
        assert math.degrees(arc["analytic"]["sweep_angle_rad"]) == pytest.approx(270.0, abs=1e-6)

    lines = sorted(e["length_m"] * MM for e in bounding if e["curve_type"] == "line")
    assert lines == pytest.approx([25.0, 25.0, 150.0, 150.0], abs=1e-9)


def test_self_test_entry_point_passes():
    from seam_worker.cad.topology import self_test

    assert self_test() == 0
