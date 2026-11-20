#!/usr/bin/env python
"""Display mesh for the authoring UI: triangles with CAD face provenance.

Emits `amr-weld-planner-v1.mesh.v1` -- a binary buffer plus a JSON sidecar.

**Display only.** This is not collision geometry. The existing verdict path in
this repo reports `visual_meshes_as_collision_meshes`, which is a smell that
must not be repeated here: a display tessellation is tuned for looking right,
not for being conservative.

Two design points, both from WEBUI.md:

- **Face RANGES, not per-triangle ids.** Faces are tessellated one at a time and
  recorded as `(face_id, tri_start, tri_count)`. 42 entries for dog_box.step
  rather than thousands, and it maps directly onto `BufferGeometry.groups` in
  three.js, so per-face highlighting is a material swap rather than a geometry
  rebuild. Selection speaks CAD face ids; triangles never leave the renderer.
- **Identifiers come from content, not from ordering.** Faces and edges are
  mapped back to topology ids through `topology.geom_key`. Replicating the
  ordering logic here would be a second place for the two modules to drift.

Edges are discretised with `GCPnts_TangentialDeflection`, which is
curvature-adaptive. Nothing is sampled uniformly -- see GOAL.md R3.

Self-contained. No imports from `cadquery/v1/` or `tesseract/v1/`.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Any

from seam_worker import paths
from seam_worker.cad.topology import (
    DEFAULT_UNIT_SCALE_TO_M,
    edge_geom_key,
    face_geom_key,
    load_step,
    sorted_solids,
)

SCHEMA = "amr-weld-planner-v1.mesh.v1"

# Chord deviation for surface tessellation, in SOURCE units (mm). Tight,
# because the mesh is a pick target for manual authoring and precision near
# face boundaries matters. 0.01 mm is negligible at these sizes.
DEFAULT_DEFLECTION = 0.01
DEFAULT_ANGULAR_DEFLECTION = 0.5

# Edge discretisation, source units and radians.
EDGE_LINEAR_DEFLECTION = 0.05
EDGE_ANGULAR_DEFLECTION = 0.05


def _pack(values: list[float], fmt: str = "<f") -> bytes:
    return b"".join(struct.pack(fmt, v) for v in values)


def _pack_u32(values: list[int]) -> bytes:
    return b"".join(struct.pack("<I", v) for v in values)


def _face_normals_from_uv(face: Any, triangulation: Any, reversed_face: bool) -> list[float]:
    """Per-node normals evaluated on the real surface, via the triangulation's
    UV nodes.

    OCCT does not populate normals here (`HasNormals()` is False), and a
    per-triangle geometric normal would facet every cylinder. The surface knows
    its own normal, so ask it.

    A TopAbs_REVERSED face uses the opposite sense to its surface, exactly as in
    `topology.outward_normal` -- and the triangle winding must be flipped to
    match, or back-face culling hides the wrong side.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.gp import gp_Pnt, gp_Vec

    adaptor = BRepAdaptor_Surface(face.wrapped)
    out: list[float] = []
    for i in range(1, triangulation.NbNodes() + 1):
        uv = triangulation.UVNode(i)
        point, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
        adaptor.D1(uv.X(), uv.Y(), point, du, dv)
        n = du.Crossed(dv)
        length = n.Magnitude()
        if length < 1e-12:
            out.extend([0.0, 0.0, 1.0])
            continue
        nx, ny, nz = n.X() / length, n.Y() / length, n.Z() / length
        if reversed_face:
            nx, ny, nz = -nx, -ny, -nz
        out.extend([nx, ny, nz])
    return out


def tessellate(
    step_path: Path,
    topology: dict[str, Any],
    deflection: float = DEFAULT_DEFLECTION,
    unit_scale_to_m: float = DEFAULT_UNIT_SCALE_TO_M,
) -> tuple[dict[str, Any], bytes]:
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GCPnts import GCPnts_TangentialDeflection
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopLoc import TopLoc_Location

    scale = unit_scale_to_m
    shape = load_step(step_path)
    BRepMesh_IncrementalMesh(shape.wrapped, deflection, False, DEFAULT_ANGULAR_DEFLECTION, True)

    face_by_key = {f["geom_key"]: f for f in topology["faces"]}
    edge_by_key = {e["geom_key"]: e for e in topology["edges"]}

    positions: list[float] = []
    normals: list[float] = []
    indices: list[int] = []
    face_ranges: list[dict[str, Any]] = []
    unmatched_faces: list[str] = []

    for solid in sorted_solids(shape):
        for face in solid.Faces():
            location = TopLoc_Location()
            triangulation = BRep_Tool.Triangulation_s(face.wrapped, location)
            if triangulation is None or triangulation.NbTriangles() == 0:
                continue

            key = face_geom_key(face)
            record = face_by_key.get(key)
            if record is None:
                unmatched_faces.append(key)
                continue

            reversed_face = face.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
            # The triangulation is stored in the face's LOCAL frame. On
            # dog_box.step the location is not identity -- node 1 of the top
            # plate reads z=0 for a face that lives at z=150 -- so skipping this
            # puts the whole mesh in the wrong place.
            transform = location.Transformation()
            base_vertex = len(positions) // 3

            for i in range(1, triangulation.NbNodes() + 1):
                node = triangulation.Node(i).Transformed(transform)
                positions.extend([node.X() * scale, node.Y() * scale, node.Z() * scale])

            # NOT transformed. The two are in different frames and it is easy
            # to "fix" this into a bug: `triangulation.Node()` is in the face's
            # LOCAL frame and needs the location applied, but
            # `BRepAdaptor_Surface` already accounts for the location, so its
            # normals are world-space already. Rotating them again turns a
            # [0,0,1] face normal into [0,-1,0] while the positions stay
            # correct -- which no position-based test catches.
            normals.extend(_face_normals_from_uv(face, triangulation, reversed_face))

            tri_start = len(indices) // 3
            for i in range(1, triangulation.NbTriangles() + 1):
                a, b, c = triangulation.Triangle(i).Get()
                if reversed_face:
                    a, c = c, a
                indices.extend(
                    [base_vertex + a - 1, base_vertex + b - 1, base_vertex + c - 1]
                )
            face_ranges.append(
                {
                    "face_id": record["id"],
                    "solid_id": record["solid_id"],
                    "surface_type": record["surface_type"],
                    "tri_start": tri_start,
                    "tri_count": len(indices) // 3 - tri_start,
                }
            )

    line_positions: list[float] = []
    line_indices: list[int] = []
    edge_ranges: list[dict[str, Any]] = []
    seen_edges: set[str] = set()

    for solid_index, solid in enumerate(sorted_solids(shape)):
        solid_id = f"solid_{solid_index}"
        for edge in solid.Edges():
            record = edge_by_key.get(edge_geom_key(solid_id, edge))
            if record is None or record["id"] in seen_edges:
                continue
            seen_edges.add(record["id"])

            adaptor = BRepAdaptor_Curve(edge.wrapped)
            discretiser = GCPnts_TangentialDeflection(
                adaptor, EDGE_ANGULAR_DEFLECTION, EDGE_LINEAR_DEFLECTION
            )
            count = discretiser.NbPoints()
            if count < 2:
                continue
            base_vertex = len(line_positions) // 3
            for i in range(1, count + 1):
                p = discretiser.Value(i)
                line_positions.extend([p.X() * scale, p.Y() * scale, p.Z() * scale])
            seg_start = len(line_indices) // 2
            for i in range(count - 1):
                line_indices.extend([base_vertex + i, base_vertex + i + 1])
            edge_ranges.append(
                {
                    "edge_id": record["id"],
                    "solid_id": record["solid_id"],
                    "curve_type": record["curve_type"],
                    "seg_start": seg_start,
                    "seg_count": count - 1,
                }
            )

    blobs = [
        ("positions", _pack(positions), "float32", 3, len(positions) // 3),
        ("normals", _pack(normals), "float32", 3, len(normals) // 3),
        ("indices", _pack_u32(indices), "uint32", 3, len(indices) // 3),
        ("line_positions", _pack(line_positions), "float32", 3, len(line_positions) // 3),
        ("line_indices", _pack_u32(line_indices), "uint32", 2, len(line_indices) // 2),
    ]
    buffers: dict[str, Any] = {}
    payload = bytearray()
    for name, blob, dtype, components, count in blobs:
        buffers[name] = {
            "byte_offset": len(payload),
            "byte_length": len(blob),
            "type": dtype,
            "components": components,
            "count": count,
        }
        payload.extend(blob)

    xs = positions[0::3] or [0.0]
    ys = positions[1::3] or [0.0]
    zs = positions[2::3] or [0.0]

    sidecar = {
        "schema": SCHEMA,
        "purpose": "display_only",
        "source": topology.get("source", {}),
        "tessellation": {
            "linear_deflection_source_units": deflection,
            "angular_deflection_rad": DEFAULT_ANGULAR_DEFLECTION,
            "edge_linear_deflection_source_units": EDGE_LINEAR_DEFLECTION,
            "edge_angular_deflection_rad": EDGE_ANGULAR_DEFLECTION,
            "unit_scale_to_m": scale,
        },
        "counts": {
            "vertices": len(positions) // 3,
            "triangles": len(indices) // 3,
            "faces": len(face_ranges),
            "line_vertices": len(line_positions) // 3,
            "line_segments": len(line_indices) // 2,
            "edges": len(edge_ranges),
            "unmatched_faces": len(unmatched_faces),
        },
        "buffers": buffers,
        "bbox_min_xyz_m": [min(xs), min(ys), min(zs)],
        "bbox_max_xyz_m": [max(xs), max(ys), max(zs)],
        "face_ranges": face_ranges,
        "edge_ranges": edge_ranges,
    }
    return sidecar, bytes(payload)


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology

    step = repo_root() / "cadquery" / "v1" / "step" / "dog_box.step"
    if not step.exists():
        print(f"tessellate self-test: missing fixture {step}", file=sys.stderr)
        return 1
    topology = extract_topology(step)
    sidecar, payload = tessellate(step, topology)
    counts = sidecar["counts"]
    assert counts["faces"] == len(topology["faces"]), counts
    assert counts["edges"] == len(topology["edges"]), counts
    assert counts["unmatched_faces"] == 0, counts
    assert len(payload) == sum(b["byte_length"] for b in sidecar["buffers"].values())
    print(
        f"tessellate self-test ok: {counts['triangles']} triangles over "
        f"{counts['faces']} face ranges, {counts['line_segments']} line segments over "
        f"{counts['edges']} edge ranges, {len(payload) / 1024:.1f} KiB"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="")
    parser.add_argument("--out", default="", help="path for the .json sidecar; .bin sits beside it")
    parser.add_argument("--deflection", type=float, default=DEFAULT_DEFLECTION)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)

    if ns.self_test:
        return self_test()
    if not ns.step:
        raise ValueError("--step is required unless --self-test is set")

    from seam_worker.cad.topology import extract_topology

    step = Path(ns.step)
    sidecar, payload = tessellate(step, extract_topology(step), ns.deflection)
    if ns.out:
        out = Path(ns.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        binary = out.with_suffix(".bin")
        binary.write_bytes(payload)
        sidecar["binary"] = binary.name
        out.write_text(json.dumps(sidecar, indent=2) + "\n", encoding="utf-8")
        print(f"wrote mesh: {out} + {binary} ({len(payload) / 1024:.1f} KiB)")
    if ns.summary or not ns.out:
        print(json.dumps(sidecar["counts"] if ns.summary else sidecar, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"tessellate: {exc}", file=sys.stderr)
        raise SystemExit(1)
