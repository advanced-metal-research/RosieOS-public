#!/usr/bin/env python3
"""Tessellate one STEP document into a triangle mesh for the Offline
Programming workstation viewport.

Companion to cadquery/v1/python/extract_weld_edges.py: same stdin/stdout JSON
contract, same millimetre wire unit, same STEP SHA-256 fingerprint identity, so
the bounded Go worker in offline-programming/v1/internal/cad can cache mesh
responses under the same fingerprint as topology responses.

Surface triangles come from OCCT's BRepMesh_IncrementalMesh, reached through
CadQuery's Shape.tessellate — the same engine cadquery/v1/python/step_to_urdf.py
already uses for its STL exports.

Request (stdin, one JSON object):
    {"operation": "tessellate", "filename": "part.step",
     "step_base64": "...", "linear_deflection_mm": 0.5,
     "angular_deflection_rad": 0.5, "max_triangles": 400000}

Response (stdout, one JSON object): see mesh_payload().
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
TOOL = "rosie cadquery v1 workpiece mesh"
DEFAULT_LINEAR_DEFLECTION_MM = 0.5
DEFAULT_ANGULAR_DEFLECTION_RAD = 0.5
MIN_LINEAR_DEFLECTION_MM = 0.01
MAX_LINEAR_DEFLECTION_MM = 10.0
MIN_ANGULAR_DEFLECTION_RAD = 0.05
MAX_ANGULAR_DEFLECTION_RAD = 1.5
DEFAULT_MAX_TRIANGLES = 400_000
MAX_MAX_TRIANGLES = 2_000_000
# Positions are emitted at micrometre precision: finer than any weld process
# cares about, and roughly 40% smaller than full float repr.
POSITION_DECIMALS = 3
NORMAL_DECIMALS = 4


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_shape(step_path: Path):
    """Import a STEP document as a single CadQuery shape."""
    import cadquery as cq

    imported = cq.importers.importStep(str(step_path))
    return imported.val()


def solids_of(shape) -> list:
    """Mirror step_to_urdf.solids_from_shape: per-solid meshing keeps each
    solid's triangulation independent and lets a part index be reported."""
    for attribute in ("Solids", "solids"):
        accessor = getattr(shape, attribute, None)
        if accessor is None:
            continue
        try:
            solids = list(accessor())
        except Exception:
            continue
        if solids:
            return solids
    return [shape]


def tessellate_solid(
    solid,
    linear_deflection_mm: float,
    angular_deflection_rad: float,
) -> tuple[list, list]:
    """Return (vertices, triangles) for one solid.

    CadQuery's Shape.tessellate drives BRepMesh_IncrementalMesh and returns
    OCCT gp_Pnt-like vertices plus index triples local to this solid.
    """
    vertices, triangles = solid.tessellate(
        linear_deflection_mm,
        angular_deflection_rad,
    )
    return list(vertices), list(triangles)


def vertex_xyz(vertex) -> tuple[float, float, float]:
    """CadQuery returns Vector objects; OCP returns gp_Pnt. Support both."""
    for names in (("x", "y", "z"), ("X", "Y", "Z")):
        first = getattr(vertex, names[0], None)
        if first is None:
            continue
        if callable(first):
            return (
                float(getattr(vertex, names[0])()),
                float(getattr(vertex, names[1])()),
                float(getattr(vertex, names[2])()),
            )
        return (
            float(getattr(vertex, names[0])),
            float(getattr(vertex, names[1])),
            float(getattr(vertex, names[2])),
        )
    point = tuple(float(value) for value in vertex)
    if len(point) != 3:
        raise ValueError("tessellation vertex is not a 3-tuple")
    return point  # type: ignore[return-value]


def triangle_normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def accumulate_normals(
    positions: list[float],
    indices: list[int],
) -> list[float]:
    """Area-weighted vertex normals.

    OCCT gives per-face triangulations that already split along sharp edges, so
    averaging shared vertices does not round off part corners. Computed here
    rather than in the browser to keep first-frame cost off the main thread.
    """
    normals = [0.0] * len(positions)
    for offset in range(0, len(indices), 3):
        ia, ib, ic = indices[offset], indices[offset + 1], indices[offset + 2]
        a = (positions[ia * 3], positions[ia * 3 + 1], positions[ia * 3 + 2])
        b = (positions[ib * 3], positions[ib * 3 + 1], positions[ib * 3 + 2])
        c = (positions[ic * 3], positions[ic * 3 + 1], positions[ic * 3 + 2])
        nx, ny, nz = triangle_normal(a, b, c)
        for index in (ia, ib, ic):
            normals[index * 3] += nx
            normals[index * 3 + 1] += ny
            normals[index * 3 + 2] += nz
    for index in range(0, len(normals), 3):
        nx, ny, nz = normals[index], normals[index + 1], normals[index + 2]
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        if length <= 1e-12:
            normals[index], normals[index + 1], normals[index + 2] = 0.0, 0.0, 1.0
            continue
        normals[index] = round(nx / length, NORMAL_DECIMALS)
        normals[index + 1] = round(ny / length, NORMAL_DECIMALS)
        normals[index + 2] = round(nz / length, NORMAL_DECIMALS)
    return normals


def mesh_payload(
    step_path: Path,
    fingerprint: str,
    linear_deflection_mm: float,
    angular_deflection_rad: float,
    max_triangles: int,
) -> dict[str, Any]:
    shape = load_shape(step_path)
    solids = solids_of(shape)

    positions: list[float] = []
    indices: list[int] = []
    bounds_min = [math.inf, math.inf, math.inf]
    bounds_max = [-math.inf, -math.inf, -math.inf]
    solid_count = 0
    truncated = False

    for solid in solids:
        if len(indices) // 3 >= max_triangles:
            truncated = True
            break
        try:
            vertices, triangles = tessellate_solid(
                solid,
                linear_deflection_mm,
                angular_deflection_rad,
            )
        except Exception:
            # One unmeshable solid must not lose the rest of the part.
            continue
        if not vertices or not triangles:
            continue
        base = len(positions) // 3
        for vertex in vertices:
            x, y, z = vertex_xyz(vertex)
            positions.append(round(x, POSITION_DECIMALS))
            positions.append(round(y, POSITION_DECIMALS))
            positions.append(round(z, POSITION_DECIMALS))
            bounds_min[0] = min(bounds_min[0], x)
            bounds_min[1] = min(bounds_min[1], y)
            bounds_min[2] = min(bounds_min[2], z)
            bounds_max[0] = max(bounds_max[0], x)
            bounds_max[1] = max(bounds_max[1], y)
            bounds_max[2] = max(bounds_max[2], z)
        vertex_count = len(vertices)
        for triangle in triangles:
            if len(indices) // 3 >= max_triangles:
                truncated = True
                break
            first, second, third = (int(value) for value in triangle[:3])
            if not (
                0 <= first < vertex_count
                and 0 <= second < vertex_count
                and 0 <= third < vertex_count
            ):
                continue
            indices.append(base + first)
            indices.append(base + second)
            indices.append(base + third)
        solid_count += 1

    if not indices:
        raise ValueError("STEP document produced no surface triangles.")

    normals = accumulate_normals(positions, indices)
    return {
        "schema_version": SCHEMA_VERSION,
        "tool": TOOL,
        "model_id": f"cadquery-step-{fingerprint[:16]}",
        "filename": step_path.name,
        "fingerprint": fingerprint,
        "unit": "mm",
        "mesh": {
            "positions": positions,
            "indices": indices,
            "normals": normals,
            "vertex_count": len(positions) // 3,
            "triangle_count": len(indices) // 3,
            "bounds_min": [round(value, POSITION_DECIMALS) for value in bounds_min],
            "bounds_max": [round(value, POSITION_DECIMALS) for value in bounds_max],
            "linear_deflection_mm": linear_deflection_mm,
            "angular_deflection_rad": angular_deflection_rad,
            "solid_count": solid_count,
            "truncated": truncated,
        },
    }


def decode_step_payload(payload: dict[str, Any]) -> tuple[str, bytes]:
    """Accept the same filename/step_base64 shape as extract_weld_edges.py."""
    filename = str(payload.get("filename") or "upload.step").strip() or "upload.step"
    raw_base64 = str(payload.get("step_base64") or payload.get("stepBase64") or "").strip()
    if "," in raw_base64:
        raw_base64 = raw_base64.split(",", 1)[1]
    if not raw_base64:
        raise ValueError("Field 'step_base64' is required.")
    try:
        step_bytes = base64.b64decode(raw_base64, validate=True)
    except Exception as exc:
        raise ValueError("Field 'step_base64' is not valid base64.") from exc
    if not step_bytes:
        raise ValueError("STEP payload is empty.")
    return filename, step_bytes


def decode_mesh_options(payload: dict[str, Any]) -> tuple[float, float, int]:
    linear = float(
        payload.get(
            "linear_deflection_mm",
            payload.get("linearDeflectionMm", DEFAULT_LINEAR_DEFLECTION_MM),
        )
    )
    angular = float(
        payload.get(
            "angular_deflection_rad",
            payload.get("angularDeflectionRad", DEFAULT_ANGULAR_DEFLECTION_RAD),
        )
    )
    triangles = int(
        payload.get("max_triangles", payload.get("maxTriangles", DEFAULT_MAX_TRIANGLES))
    )
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError("Deflection values must be finite.")
    return (
        clamp(linear, MIN_LINEAR_DEFLECTION_MM, MAX_LINEAR_DEFLECTION_MM),
        clamp(angular, MIN_ANGULAR_DEFLECTION_RAD, MAX_ANGULAR_DEFLECTION_RAD),
        int(clamp(float(triangles), 1_000.0, float(MAX_MAX_TRIANGLES))),
    )


def tessellate_from_stdin() -> int:
    payload = json.load(sys.stdin)
    if not isinstance(payload, dict):
        raise ValueError("JSON object required.")
    operation = str(payload.get("operation") or "tessellate")
    if operation != "tessellate":
        raise ValueError(f"Unsupported operation {operation!r}.")
    filename, step_bytes = decode_step_payload(payload)
    linear, angular, max_triangles = decode_mesh_options(payload)
    fingerprint = hashlib.sha256(step_bytes).hexdigest()
    suffix = Path(filename).suffix.lower()
    if suffix not in {".step", ".stp"}:
        suffix = ".step"
    handle = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
    try:
        with handle:
            handle.write(step_bytes)
        result = mesh_payload(
            Path(handle.name),
            fingerprint,
            linear,
            angular,
            max_triangles,
        )
        result["filename"] = filename
        print(json.dumps(result, separators=(",", ":")))
        return 0
    finally:
        try:
            os.remove(handle.name)
        except OSError:
            pass


def tessellate_from_path(args: argparse.Namespace) -> int:
    step_path = Path(args.step).expanduser().resolve()
    if not step_path.exists():
        raise FileNotFoundError(f"STEP file not found: {step_path}")
    fingerprint = hashlib.sha256(step_path.read_bytes()).hexdigest()
    linear = clamp(
        args.linear_deflection_mm,
        MIN_LINEAR_DEFLECTION_MM,
        MAX_LINEAR_DEFLECTION_MM,
    )
    angular = clamp(
        args.angular_deflection_rad,
        MIN_ANGULAR_DEFLECTION_RAD,
        MAX_ANGULAR_DEFLECTION_RAD,
    )
    result = mesh_payload(step_path, fingerprint, linear, angular, args.max_triangles)
    summary = result["mesh"]
    print(
        f"{summary['triangle_count']} triangles, "
        f"{summary['vertex_count']} vertices, "
        f"{summary['solid_count']} solids, "
        f"truncated={summary['truncated']}",
        file=sys.stderr,
    )
    print(json.dumps(result, separators=(",", ":")))
    return 0


def self_test() -> int:
    """Exercise every path that does not need OCCT, so the request contract and
    the normal/index math stay checkable without the CAD environment."""
    filename, step_bytes = decode_step_payload({
        "filename": "x.step",
        "step_base64": base64.b64encode(b"STEP").decode("ascii"),
    })
    assert filename == "x.step"
    assert step_bytes == b"STEP"

    linear, angular, triangles = decode_mesh_options({})
    assert linear == DEFAULT_LINEAR_DEFLECTION_MM
    assert angular == DEFAULT_ANGULAR_DEFLECTION_RAD
    assert triangles == DEFAULT_MAX_TRIANGLES

    linear, angular, triangles = decode_mesh_options({
        "linear_deflection_mm": 0.0001,
        "angular_deflection_rad": 99.0,
        "max_triangles": 10,
    })
    assert linear == MIN_LINEAR_DEFLECTION_MM
    assert angular == MAX_ANGULAR_DEFLECTION_RAD
    assert triangles == 1_000

    assert vertex_xyz((1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)

    class Vector:
        x, y, z = 4.0, 5.0, 6.0

    assert vertex_xyz(Vector()) == (4.0, 5.0, 6.0)

    class Point:
        def X(self):  # noqa: N802 - OCP naming
            return 7.0

        def Y(self):  # noqa: N802
            return 8.0

        def Z(self):  # noqa: N802
            return 9.0

    assert vertex_xyz(Point()) == (7.0, 8.0, 9.0)

    assert triangle_normal((0, 0, 0), (1, 0, 0), (0, 1, 0)) == (0.0, 0.0, 1.0)
    assert triangle_normal((0, 0, 0), (1, 0, 0), (2, 0, 0)) == (0.0, 0.0, 0.0)

    # Two coplanar triangles sharing an edge: every normal is +Z.
    positions = [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0]
    indices = [0, 1, 2, 0, 2, 3]
    normals = accumulate_normals(positions, indices)
    assert normals == [0, 0, 1] * 4, normals

    assert clamp(5.0, 0.0, 1.0) == 1.0
    print("tessellate_step self-test passed", file=sys.stderr)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stdin", action="store_true", help="read one JSON request from stdin")
    parser.add_argument("--step", help="tessellate a STEP file directly")
    parser.add_argument("--linear-deflection-mm", type=float, default=DEFAULT_LINEAR_DEFLECTION_MM)
    parser.add_argument(
        "--angular-deflection-rad",
        type=float,
        default=DEFAULT_ANGULAR_DEFLECTION_RAD,
    )
    parser.add_argument("--max-triangles", type=int, default=DEFAULT_MAX_TRIANGLES)
    parser.add_argument("--self-test", action="store_true", help="run environment-free checks")
    args = parser.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.stdin:
        return tessellate_from_stdin()
    if args.step:
        return tessellate_from_path(args)
    parser.error("one of --stdin, --step, or --self-test is required")
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # noqa: BLE001 - stderr is the worker's error channel
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        sys.exit(1)
