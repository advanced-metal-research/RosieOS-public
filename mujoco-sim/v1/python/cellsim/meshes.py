"""Collada to OBJ, once per mesh.

The cell's link meshes live beside the URDF as `.dae` and MuJoCo reads only
STL/OBJ/MSH. This converts each Collada file to an OBJ next to a cache
directory, keyed by the source's mtime and size, so a rebuilt mesh is picked
up and an unchanged one costs a stat.

pycollada rather than trimesh: the weld planner's verifier reads the same
files with pycollada (`verifier/verify.py`), and this host's trimesh is not
installed anywhere the sim runs. The cell's files are metres and Z-up
(checked on 2026-08-29: every `rosie_1400_v3` mesh reports `unitmeter=1.0`,
`upaxis=Z_UP`); the unit scale is honoured anyway so a millimetre export
would not silently arrive a thousand times too large.
"""

from __future__ import annotations

import pathlib

import numpy as np


class MeshError(ValueError):
    pass


def load_collada(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray]:
    """Triangles of one Collada file as (vertices (N,3) float64 m, faces (M,3) int)."""
    import collada

    doc = collada.Collada(
        str(path),
        ignore=[collada.common.DaeUnsupportedError, collada.common.DaeBrokenRefError],
    )
    scale = float(doc.assetInfo.unitmeter or 1.0)
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for geometry in doc.scene.objects("geometry"):
        for primitive in geometry.primitives():
            if not hasattr(primitive, "vertex_index"):
                try:
                    primitive = primitive.triangleset()
                except Exception:  # noqa: BLE001, S112 -- lines/points carry no surface
                    continue
            v = np.asarray(primitive.vertex, dtype=np.float64) * scale
            f = np.asarray(primitive.vertex_index, dtype=np.int64).reshape(-1, 3)
            if len(v) == 0 or len(f) == 0:
                continue
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
    if not vertices:
        raise MeshError(f"{path}: no triangles")
    return np.vstack(vertices), np.vstack(faces)


def write_obj(path: pathlib.Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


def _stamp(path: pathlib.Path) -> str:
    st = path.stat()
    return f"{st.st_mtime_ns}:{st.st_size}"


def convert(src: pathlib.Path, cache: pathlib.Path) -> pathlib.Path:
    """The OBJ for one `.dae`, converting only when the source changed."""
    cache.mkdir(parents=True, exist_ok=True)
    dst = cache / (src.stem + ".obj")
    stamp = cache / (src.stem + ".stamp")
    want = _stamp(src)
    if dst.exists() and stamp.exists() and stamp.read_text() == want:
        return dst
    vertices, faces = load_collada(src)
    write_obj(dst, vertices, faces)
    stamp.write_text(want)
    return dst


def convert_all(mesh_dir: pathlib.Path, cache: pathlib.Path) -> dict[str, pathlib.Path]:
    return {p.stem: convert(p, cache) for p in sorted(mesh_dir.glob("*.dae"))}
