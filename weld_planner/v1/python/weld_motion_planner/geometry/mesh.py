#!/usr/bin/env python
"""The part's surfaces, tessellated by the planner rather than handed to it.

The plan request ships a STEP and no mesh, on purpose: `MOTION.md` argues that
the consumer picks its own tolerance, because the authoring side's display
mesh is chosen to look right at screen resolution and this one is chosen to
resolve millimetre weld clearance. Two different questions, two different
deflections.

That means an OCCT on this side. It is not a crack in the boundary -- the
rule is that `weld_motion_planner` never imports `seam_worker`, not that it stays
small -- and `tests/motion_planner/test_boundary.py` still enforces the rule that
matters.

## One mesh, in the workpiece frame

The triangles come out in the STEP's own coordinates, which IS the workpiece
frame: `GOAL.md` non-negotiable 5 puts the program's geometry there, and the
placement says where that frame sits in the cell. So the distance field baked
from these triangles rides the positioner for free, which is the whole reason
`ARCHITECTURE.md` splits the fields by frame rather than baking one world
grid.

Units: the extractor works in metres and so does everything here. A STEP in
millimetres is converted on import by the same `unit scale` the program
records, so the mesh and the seam agree without anybody rescaling twice.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# The deflection the bake asks OCCT for. Well below the 1-2 mm voxel the part
# grid uses, so the triangulation is not what limits the field's accuracy --
# and not so fine that a plate becomes a million triangles for no gain.
DEFAULT_DEFLECTION_M = 0.0002
DEFAULT_ANGULAR_DEFLECTION_RAD = 0.3


class MeshError(ValueError):
    """A STEP this refuses to tessellate, with the reason."""


@dataclass(frozen=True)
class Mesh:
    """Triangles in the workpiece frame."""

    vertices: np.ndarray  # (V, 3) float64, metres
    faces: np.ndarray  # (F, 3) int32

    @property
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def summary(self) -> dict:
        lo, hi = self.bounds
        return {
            "vertices": len(self.vertices),
            "triangles": len(self.faces),
            "bounds_min_m": [round(float(v), 6) for v in lo],
            "bounds_max_m": [round(float(v), 6) for v in hi],
            "size_mm": [round(float(v) * 1000.0, 3) for v in (hi - lo)],
        }


def tessellate(
    step_bytes: bytes,
    *,
    deflection_m: float = DEFAULT_DEFLECTION_M,
    angular_deflection_rad: float = DEFAULT_ANGULAR_DEFLECTION_RAD,
) -> Mesh:
    """A STEP's solids as one triangle soup, in metres.

    `BRepMesh_IncrementalMesh` KEEPS any triangulation already on the shape,
    so a deflection asked for after somebody else has meshed is accepted and
    quietly ignored -- the authoring side documents hitting exactly this in
    `occlusion.py`. The shape here is freshly imported and meshed once, so
    the deflection asked for is the deflection got.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as handle:
        handle.write(step_bytes)
        path = Path(handle.name)
    try:
        reader = STEPControl_Reader()
        if reader.ReadFile(str(path)) != IFSelect_RetDone:
            raise MeshError("OCCT could not read this STEP")
        reader.TransferRoots()
        shape = reader.OneShape()
    finally:
        path.unlink(missing_ok=True)

    # STEP is millimetres by convention and OCCT keeps those units. The weld
    # program is metres, so the scale is applied here rather than left for a
    # caller to remember -- a mesh a thousand times too big does not look
    # wrong, it looks like the part is somewhere else.
    scale = 0.001

    BRepMesh_IncrementalMesh(shape, deflection_m / scale, False, angular_deflection_rad, True)

    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        explorer.Next()
        if triangulation is None:
            continue
        transform = location.Transformation()
        count = triangulation.NbNodes()
        block = np.empty((count, 3), dtype=np.float64)
        for i in range(1, count + 1):
            point = triangulation.Node(i).Transformed(transform)
            block[i - 1] = (point.X() * scale, point.Y() * scale, point.Z() * scale)
        indices = np.empty((triangulation.NbTriangles(), 3), dtype=np.int32)
        for i in range(1, triangulation.NbTriangles() + 1):
            a, b, c = triangulation.Triangle(i).Get()
            indices[i - 1] = (a - 1 + offset, b - 1 + offset, c - 1 + offset)
        # Reversed faces come out with their winding flipped, and the sign of
        # the distance field is decided by winding. Left as OCCT gives it and
        # handled by the winding-number sign in `esdf.py`, which does not
        # care about per-face orientation -- see the note there.
        vertices.append(block)
        faces.append(indices)
        offset += count

    if not faces:
        raise MeshError("this STEP tessellated to no triangles at all")
    return Mesh(np.concatenate(vertices), np.concatenate(faces))


def self_test() -> int:
    """Against a solid whose surface area and bounds are known exactly."""
    import cadquery as cq

    box = cq.Workplane("XY").box(100.0, 50.0, 25.0)  # millimetres
    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as handle:
        path = Path(handle.name)
    try:
        cq.exporters.export(box, str(path))
        data = path.read_bytes()
    finally:
        path.unlink(missing_ok=True)

    mesh = tessellate(data)
    lo, hi = mesh.bounds
    size = hi - lo
    assert np.allclose(size, [0.1, 0.05, 0.025], atol=1e-9), f"box came out {size} m"
    assert len(mesh.faces) >= 12, mesh.faces.shape

    # Area, because bounds alone would pass on a mesh with holes in it.
    a = mesh.vertices[mesh.faces[:, 0]]
    b = mesh.vertices[mesh.faces[:, 1]]
    c = mesh.vertices[mesh.faces[:, 2]]
    area = float(0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())
    exact = 2.0 * (0.1 * 0.05 + 0.1 * 0.025 + 0.05 * 0.025)
    assert abs(area - exact) < 1e-9, f"surface area {area} m^2, expected {exact}"

    print(
        f"mesh self-test ok: a 100x50x25 mm box tessellates to {len(mesh.faces)} triangles, "
        f"{area * 1e6:.1f} mm^2 of surface, in metres"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose part to tessellate")
    parser.add_argument("--deflection-mm", type=float, default=DEFAULT_DEFLECTION_M * 1000.0)
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise MeshError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read

    request = read(ns.read)
    mesh = tessellate(request.step_bytes, deflection_m=ns.deflection_mm / 1000.0)
    print(json.dumps({"source": request.step_filename, **mesh.summary()}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"mesh: {exc}", file=sys.stderr)
        raise SystemExit(1)
