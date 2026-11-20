"""Display mesh: face provenance, world placement, and winding.

The mesh is a pick target for manual authoring, so the checks that matter are
the ones that would let a click resolve to the wrong CAD face, or put geometry
in the wrong place:

- vertices must be in WORLD space. OCCT stores each face's triangulation in a
  local frame and `TopLoc_Location` is not identity on this file -- node 1 of
  the top plate reads z=0 for a face that lives at z=150.
- normals must match topology's outward normals, and triangle winding must
  agree with them, or back-face culling hides the wrong side.
- face ranges must tile the index buffer exactly, or a raycast hit resolves to
  a neighbouring face.
"""


import numpy as np
import pytest
from seam_worker.cad.tessellate import tessellate

MM = 1000.0


@pytest.fixture(scope="module")
def mesh(dog_box_topology, repo_root):
    step = repo_root / "cadquery" / "v1" / "step" / "dog_box.step"
    sidecar, payload = tessellate(step, dog_box_topology)
    return sidecar, payload


def _read(sidecar, payload, name):
    spec = sidecar["buffers"][name]
    dtype = np.float32 if spec["type"] == "float32" else np.uint32
    array = np.frombuffer(payload, dtype=dtype, count=spec["count"] * spec["components"],
                          offset=spec["byte_offset"])
    return array.reshape(spec["count"], spec["components"])


# --------------------------------------------------------------------------
# provenance
# --------------------------------------------------------------------------


def test_every_cad_face_and_edge_is_represented(mesh, dog_box_topology):
    sidecar, _ = mesh
    assert sidecar["counts"]["unmatched_faces"] == 0
    assert sidecar["counts"]["faces"] == len(dog_box_topology["faces"]) == 42
    assert sidecar["counts"]["edges"] == len(dog_box_topology["edges"]) == 90

    topo_faces = {f["id"] for f in dog_box_topology["faces"]}
    topo_edges = {e["id"] for e in dog_box_topology["edges"]}
    assert {r["face_id"] for r in sidecar["face_ranges"]} == topo_faces
    assert {r["edge_id"] for r in sidecar["edge_ranges"]} == topo_edges


def test_face_ranges_tile_the_index_buffer_exactly(mesh):
    """No gaps, no overlaps. A gap means a triangle resolves to no face; an
    overlap means it resolves to the wrong one."""
    sidecar, _ = mesh
    ranges = sorted(sidecar["face_ranges"], key=lambda r: r["tri_start"])
    cursor = 0
    for record in ranges:
        assert record["tri_start"] == cursor, f"gap or overlap at {record['face_id']}"
        assert record["tri_count"] > 0, record["face_id"]
        cursor += record["tri_count"]
    assert cursor == sidecar["counts"]["triangles"]


def test_edge_ranges_tile_the_line_buffer_exactly(mesh):
    sidecar, _ = mesh
    ranges = sorted(sidecar["edge_ranges"], key=lambda r: r["seg_start"])
    cursor = 0
    for record in ranges:
        assert record["seg_start"] == cursor, f"gap or overlap at {record['edge_id']}"
        cursor += record["seg_count"]
    assert cursor == sidecar["counts"]["line_segments"]


def test_buffer_layout_is_self_describing(mesh):
    sidecar, payload = mesh
    total = 0
    for spec in sidecar["buffers"].values():
        assert spec["byte_offset"] == total
        total += spec["byte_length"]
    assert total == len(payload)


# --------------------------------------------------------------------------
# world placement -- the TopLoc_Location trap
# --------------------------------------------------------------------------


def test_mesh_is_in_world_space(mesh):
    """REGRESSION GUARD. Each face's triangulation lives in a local frame and
    the location is NOT identity on this file. Skipping the transform silently
    collapses parts onto the origin."""
    sidecar, payload = mesh
    positions = _read(sidecar, payload, "positions")
    lo = positions.min(axis=0) * MM
    hi = positions.max(axis=0) * MM
    assert lo == pytest.approx([-153.0, -78.0, 0.0], abs=0.05)
    assert hi == pytest.approx([153.0, 78.0, 153.0], abs=0.05)
    # sidecar carries float64; the buffer is float32, so compare at float32 precision
    assert sidecar["bbox_min_xyz_m"] == pytest.approx(lo / MM, abs=1e-6)
    assert sidecar["bbox_max_xyz_m"] == pytest.approx(hi / MM, abs=1e-6)


def test_vertices_lie_on_the_face_they_are_attributed_to(mesh, dog_box_topology):
    """Strongest provenance check: geometry attributed to a face must actually
    satisfy that face's surface equation."""
    sidecar, payload = mesh
    positions = _read(sidecar, payload, "positions")
    indices = _read(sidecar, payload, "indices")
    faces = {f["id"]: f for f in dog_box_topology["faces"]}

    worst_plane = worst_cylinder = 0.0
    for record in sidecar["face_ranges"]:
        face = faces[record["face_id"]]
        tris = indices[record["tri_start"] : record["tri_start"] + record["tri_count"]]
        verts = positions[np.unique(tris.reshape(-1))]
        if face["surface_type"] == "plane":
            origin = np.array(face["plane"]["origin_xyz_m"])
            normal = np.array(face["plane"]["normal_xyz"])
            worst_plane = max(worst_plane, float(np.abs((verts - origin) @ normal).max()))
        else:
            cyl = face["cylinder"]
            axis = np.array(cyl["axis_dir_xyz"])
            rel = verts - np.array(cyl["axis_origin_xyz_m"])
            radial = rel - np.outer(rel @ axis, axis)
            deviation = np.abs(np.linalg.norm(radial, axis=1) - cyl["radius_m"])
            worst_cylinder = max(worst_cylinder, float(deviation.max()))

    assert worst_plane * MM < 1e-4, f"planar vertices off-plane by {worst_plane * MM:.2e} mm"
    # bounded by the tessellation chord deviation, not by float noise
    assert worst_cylinder * MM < 0.02, f"cylinder vertices off-surface by {worst_cylinder * MM:.4f} mm"


# --------------------------------------------------------------------------
# normals and winding
# --------------------------------------------------------------------------


def test_planar_normals_match_topology_outward_normals(mesh, dog_box_topology):
    sidecar, payload = mesh
    positions = _read(sidecar, payload, "positions")
    normals = _read(sidecar, payload, "normals")
    indices = _read(sidecar, payload, "indices")
    faces = {f["id"]: f for f in dog_box_topology["faces"]}

    checked = 0
    for record in sidecar["face_ranges"]:
        face = faces[record["face_id"]]
        if face["surface_type"] != "plane":
            continue
        tris = indices[record["tri_start"] : record["tri_start"] + record["tri_count"]]
        vertex_normals = normals[np.unique(tris.reshape(-1))]
        expected = np.array(face["outward_normal_xyz"])
        assert vertex_normals == pytest.approx(
            np.tile(expected, (len(vertex_normals), 1)), abs=1e-6
        ), record["face_id"]
        checked += 1
    assert checked == 28
    assert len(positions) == len(normals)


def test_triangle_winding_agrees_with_normals(mesh):
    """Counter-clockwise when viewed from outside. If winding and normals
    disagree, back-face culling hides the visible side and picking breaks on
    exactly the faces the operator is trying to click."""
    sidecar, payload = mesh
    positions = _read(sidecar, payload, "positions")
    normals = _read(sidecar, payload, "normals")
    indices = _read(sidecar, payload, "indices")

    a = positions[indices[:, 0]]
    b = positions[indices[:, 1]]
    c = positions[indices[:, 2]]
    geometric = np.cross(b - a, c - a)
    lengths = np.linalg.norm(geometric, axis=1)
    keep = lengths > 1e-12
    geometric = geometric[keep] / lengths[keep, None]

    shading = normals[indices[:, 0]][keep]
    alignment = np.einsum("ij,ij->i", geometric, shading)
    assert alignment.min() > 0.5, f"worst winding/normal alignment {alignment.min():.4f}"


# --------------------------------------------------------------------------
# edges stay curvature-adaptive (GOAL.md R3)
# --------------------------------------------------------------------------


def test_edge_discretisation_is_curvature_adaptive(mesh, dog_box_topology):
    """A straight line needs 1 segment however long it is; a tight arc needs
    many. Uniform sampling would give the opposite -- which is exactly the
    defect that put one waypoint on an R~7mm corner in the prior CAD path."""
    sidecar, _ = mesh
    edges = {e["id"]: e for e in dog_box_topology["edges"]}
    ranges = {r["edge_id"]: r for r in sidecar["edge_ranges"]}

    lines = [(edges[k]["length_m"], v["seg_count"]) for k, v in ranges.items()
             if edges[k]["curve_type"] == "line"]
    arcs = [(edges[k]["length_m"], v["seg_count"]) for k, v in ranges.items()
            if edges[k]["curve_type"] == "circle"]
    assert lines and arcs

    assert max(n for _, n in lines) == 1, "a straight edge needs exactly one segment"

    long_line = max(length for length, _ in lines)
    short_arc = min(length for length, _ in arcs)
    arc_segments = min(n for _, n in arcs)
    assert short_arc < long_line, "expected an arc shorter than the longest line"
    assert arc_segments > 1, "arcs must be subdivided by curvature, not by length"


def test_arc_chord_error_is_bounded(mesh, dog_box_topology):
    """Sagitta of each polyline chord against the true arc must stay within the
    requested deflection."""
    from seam_worker.cad.tessellate import EDGE_LINEAR_DEFLECTION

    sidecar, payload = mesh
    line_positions = _read(sidecar, payload, "line_positions")
    line_indices = _read(sidecar, payload, "line_indices")
    edges = {e["id"]: e for e in dog_box_topology["edges"]}

    worst = 0.0
    for record in sidecar["edge_ranges"]:
        edge = edges[record["edge_id"]]
        if edge["curve_type"] != "circle":
            continue
        radius = edge["analytic"]["radius_m"]
        segs = line_indices[record["seg_start"] : record["seg_start"] + record["seg_count"]]
        chords = np.linalg.norm(
            line_positions[segs[:, 1]] - line_positions[segs[:, 0]], axis=1
        )
        half = np.clip(chords / (2.0 * radius), -1.0, 1.0)
        sagitta = radius * (1.0 - np.cos(np.arcsin(half)))
        worst = max(worst, float(sagitta.max()))
    assert worst * MM <= EDGE_LINEAR_DEFLECTION * 1.05, f"chord error {worst * MM:.4f} mm"


def test_declared_display_only(mesh):
    """Not collision geometry. `visual_meshes_as_collision_meshes` is an
    existing smell in this repo and must not be repeated."""
    sidecar, _ = mesh
    assert sidecar["purpose"] == "display_only"


def test_self_test_entry_point_passes():
    from seam_worker.cad.tessellate import self_test

    assert self_test() == 0
