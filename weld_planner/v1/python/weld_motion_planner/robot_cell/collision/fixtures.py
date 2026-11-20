"""Workcell fixtures the operator defined in the OLP, as triangles in the WORLD frame.

A stand under the part, a clamp, a table: bodies the arm must not hit that
are not in the robot's URDF and are not the part. Until 2026-09-20 the only
such body was `seam_worker.robot.cell.CELL_FIXTURES`, a placeholder box the
planner added on its own that nothing else drew or could edit, so the OLP
placed the part in clear air while the planner avoided a stand nobody could
see (`rosie_1420_v1`, owner review of a failed plan). Now the OLP carries the
fixtures in the `.weldplan` (`fixtures.json`) and this module turns them into
meshes; the planner's screen and the verifier read the same list.

One entry is::

    {"id": "stand", "name": "Stand",
     "shape": {"kind": "box", "size_m": [0.5, 0.5, 1.0]}
           | {"kind": "mesh", "vertices": [...], "faces": [...], "filename": "stand.step"},
     "xyz_m": [1.0, 1.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]}

A box stands on its bottom-centre: `xyz_m` is the middle of its footprint on
the floor and it rises `size_m[2]` from there, so a stand placed on the floor
has z = 0 and a part on it sits at z = size_m[2]. A mesh is placed by its own
origin, the way a STEP the OLP tessellated is. `rpy_rad` is fixed-axis roll,
pitch, yaw (Rz Ry Rx), the URDF convention every placement here uses.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from weld_motion_planner.geometry.mesh import Mesh


class FixtureError(ValueError):
    """A fixture entry this refuses to build, with the reason."""


# The twelve triangles of a unit box, over the eight corners listed in
# `_box_vertices`; outward-facing so a signed test reads the same as a mesh
# from a STEP.
_BOX_FACES = np.array(
    [
        [0, 2, 1], [0, 3, 2],  # bottom (z = 0), facing down
        [4, 5, 6], [4, 6, 7],  # top, facing up
        [0, 1, 5], [0, 5, 4],  # y = -sy/2
        [1, 2, 6], [1, 6, 5],  # x = +sx/2
        [2, 3, 7], [2, 7, 6],  # y = +sy/2
        [3, 0, 4], [3, 4, 7],  # x = -sx/2
    ],
    dtype=np.int32,
)


def _box_vertices(size_m: tuple[float, float, float]) -> np.ndarray:
    sx, sy, sz = size_m
    hx, hy = sx / 2.0, sy / 2.0
    return np.array(
        [
            [-hx, -hy, 0.0], [hx, -hy, 0.0], [hx, hy, 0.0], [-hx, hy, 0.0],
            [-hx, -hy, sz], [hx, -hy, sz], [hx, hy, sz], [-hx, hy, sz],
        ],
        dtype=np.float64,
    )


def rotation_rpy(rpy_rad: tuple[float, float, float]) -> np.ndarray:
    """Fixed-axis roll, pitch, yaw as a matrix: Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = (float(v) for v in rpy_rad)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _triple(value: Any, what: str, fixture_id: str) -> tuple[float, float, float]:
    try:
        x, y, z = (float(v) for v in value)
    except (TypeError, ValueError) as error:
        raise FixtureError(f"fixture {fixture_id!r}: {what} must be three numbers") from error
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise FixtureError(f"fixture {fixture_id!r}: {what} must be finite")
    return x, y, z


def fixture_mesh(fixture: dict[str, Any]) -> Mesh:
    """One fixture entry as triangles in the world frame."""
    fixture_id = str(fixture.get("id") or "?")
    shape = fixture.get("shape") or {}
    kind = shape.get("kind")
    if kind == "box":
        size = _triple(shape.get("size_m"), "shape.size_m", fixture_id)
        if min(size) <= 0.0:
            raise FixtureError(f"fixture {fixture_id!r}: a box needs three positive sides, got {size}")
        vertices, faces = _box_vertices(size), _BOX_FACES.copy()
    elif kind == "mesh":
        try:
            vertices = np.asarray(shape.get("vertices"), dtype=np.float64).reshape(-1, 3)
            faces = np.asarray(shape.get("faces"), dtype=np.int32).reshape(-1, 3)
        except (TypeError, ValueError) as error:
            raise FixtureError(
                f"fixture {fixture_id!r}: a mesh needs flat vertex triples and face index triples"
            ) from error
        if len(vertices) < 3 or len(faces) < 1:
            raise FixtureError(f"fixture {fixture_id!r}: the mesh is empty")
        if faces.min() < 0 or faces.max() >= len(vertices):
            raise FixtureError(f"fixture {fixture_id!r}: a face names a vertex the mesh does not have")
        if not np.isfinite(vertices).all():
            raise FixtureError(f"fixture {fixture_id!r}: the mesh has a non-finite vertex")
    else:
        raise FixtureError(f"fixture {fixture_id!r}: unknown shape kind {kind!r}; expected box or mesh")
    xyz = _triple(fixture.get("xyz_m", (0.0, 0.0, 0.0)), "xyz_m", fixture_id)
    rpy = _triple(fixture.get("rpy_rad", (0.0, 0.0, 0.0)), "rpy_rad", fixture_id)
    placed = vertices @ rotation_rpy(rpy).T + np.asarray(xyz, dtype=np.float64)
    return Mesh(vertices=placed, faces=faces)


def fixture_meshes(fixtures: list[dict[str, Any]] | None) -> list[Mesh]:
    """Every fixture of a plan request as a world-frame mesh; none when the
    request carries no list (then the cell's own placeholder, if any, stands)."""
    if not fixtures:
        return []
    if not isinstance(fixtures, list):
        raise FixtureError("fixtures must be a list of fixture entries")
    return [fixture_mesh(entry) for entry in fixtures if isinstance(entry, dict)]


def concatenate(meshes: list[Mesh]) -> Mesh | None:
    """One mesh from several, face indices offset; None for none."""
    parts = [m for m in meshes if m is not None]
    if not parts:
        return None
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    base = 0
    for mesh in parts:
        v = np.asarray(mesh.vertices, dtype=np.float64)
        vertices.append(v)
        faces.append(np.asarray(mesh.faces, dtype=np.int32) + base)
        base += len(v)
    return Mesh(vertices=np.concatenate(vertices), faces=np.concatenate(faces))
