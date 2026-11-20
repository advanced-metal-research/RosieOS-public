#!/usr/bin/env python
"""The debug bundle: what a human looks at, and what nothing depends on.

`IMPLEMENTATION.md` MV. A stage whose output can only be judged by reading
numbers is a stage that will be judged wrong, so every milestone writes what
it computed into a bundle and the dev viewer draws it.

## Explicitly not a contract

Schema `motion-debug.v0`, and the `v0` is load-bearing: this may break at any
commit, nothing downstream may read it, and the response artifact of M8
remains the only thing the planner promises. Keeping that line sharp is what
lets the bundle carry whatever a panel happens to need -- the moment
something depends on it, it stops being a debug bundle and becomes a second
contract to maintain.

## A directory, not a zip

The plan request is a zip because it crosses a process boundary and has to be
one file with one digest. This crosses no boundary; it is read by a page on
the same machine, over the dev server. A directory means the browser fetches
`field.bin` directly and streams it, with no unzip library in the viewer and
no base64 inflating an 8 MB volume to 11 MB of JSON.

## Downsampled on the way out

The part field is ~8M voxels, which is 30 MiB of float32 and more than a
page should hold to draw slices with. It is reduced by strided sampling to
a few million at most -- min-pooled, not point-sampled, so a thin feature
between two sample planes still shows as near-zero clearance rather than
disappearing. What the viewer draws is then conservative in the right
direction: it never claims more room than the field says.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch

SCHEMA = "motion-debug.v0"

# Where the dev viewer looks. Under `ui/public`, so Vite serves it at
# `/debug/<name>/` with no route of its own and the production build simply
# does not include it (it is gitignored).
DEFAULT_ROOT = Path(__file__).resolve().parents[3] / "ui" / "public" / "debug"

# Cap on what a slice viewer has to hold. 4M float32 is 16 MB, which streams
# in well under a second on localhost and is far more resolution than a
# screen can show.
MAX_FIELD_VOXELS = 4_000_000


class Bundle:
    """One run's worth of things to look at, written as it goes."""

    def __init__(self, name: str, root: Path | None = None):
        self.root = (root or DEFAULT_ROOT) / name
        if self.root.exists():
            shutil.rmtree(self.root)
        self.root.mkdir(parents=True)
        self.manifest: dict[str, Any] = {
            "schema": SCHEMA,
            "name": name,
            "note": "Dev output. Not a contract; may break at any commit.",
            "panels": {},
        }
        self._started = time.time()

    # -- panels ----------------------------------------------------------

    def add_request(self, request: Any, gaps: dict[str, list[str]]) -> None:
        self.manifest["request"] = {
            "request_id": request.request_id,
            "program_id": request.program.get("program_id"),
            "source": request.step_filename,
            "cell": request.cell.get("id"),
            "seams": len(request.seams),
            "placement": request.placement,
            "gaps": gaps,
        }

    def add_mesh(self, mesh: Any) -> None:
        """Triangle soup as raw float32, for three.js to upload directly."""
        vertices = mesh.vertices[mesh.faces.reshape(-1)].astype(np.float32)
        (self.root / "mesh.bin").write_bytes(vertices.tobytes())
        self.manifest["panels"]["mesh"] = {
            "path": "mesh.bin",
            "layout": "float32 xyz per vertex, three per triangle",
            "triangles": len(mesh.faces),
            **mesh.summary(),
        }

    def add_field(self, field: Any, label: str = "part") -> None:
        """A distance field, downsampled conservatively."""
        values = field.values
        stride = 1
        while values.numel() / stride**3 > MAX_FIELD_VOXELS:
            stride += 1
        if stride > 1:
            # MIN over each block, not a sample of its corner: a 1 mm rib
            # between two sample planes must not vanish, and clearance that
            # reads smaller than it is can only ever make the viewer
            # pessimistic -- which is the safe direction for a picture whose
            # job is to show where the torch nearly touches.
            trimmed = values[
                : values.shape[0] // stride * stride,
                : values.shape[1] // stride * stride,
                : values.shape[2] // stride * stride,
            ]
            blocks = trimmed.reshape(
                trimmed.shape[0] // stride, stride,
                trimmed.shape[1] // stride, stride,
                trimmed.shape[2] // stride, stride,
            )
            values = blocks.amin(dim=(1, 3, 5))

        data = values.detach().cpu().to(torch.float32).contiguous()
        (self.root / f"field.{label}.bin").write_bytes(data.numpy().tobytes())
        self.manifest["panels"].setdefault("fields", {})[label] = {
            "path": f"field.{label}.bin",
            "layout": "float32, x-major (index = (i*ny + j)*nz + k), metres, negative inside",
            "shape": [int(v) for v in data.shape],
            "voxel_m": field.voxel_m * stride,
            "origin_m": [float(v) for v in field.origin_pos_m],
            "frame": field.frame,
            "downsample_stride": stride,
            "full_shape": list(field.shape),
            "min_m": float(data.min()),
            "max_m": float(data.max()),
        }

    def add_seams(self, seams: list[dict[str, Any]]) -> None:
        (self.root / "seams.json").write_text(json.dumps(seams, indent=1))
        self.manifest["panels"]["seams"] = {
            "path": "seams.json",
            "count": len(seams),
            "layout": "per seam: sampled points, frame triads, torch direction, clearance",
        }

    def add_timing(self, timing: dict[str, float]) -> None:
        self.manifest["timing_s"] = {k: round(v, 4) for k, v in timing.items()}

    def close(self) -> Path:
        self.manifest["total_s"] = round(time.time() - self._started, 4)
        (self.root / "manifest.json").write_text(json.dumps(self.manifest, indent=2))
        return self.root


def build(
    path: Path, *, name: str | None = None, root: Path | None = None, voxel_mm: float = 1.5
) -> Path:
    """Everything M0 through M2 computed, in one directory."""
    from weld_motion_planner.geometry.esdf import _ensure_kernel, bake
    from weld_motion_planner.geometry.mesh import tessellate
    from weld_motion_planner.geometry.seam_frame import Seam, apply_angles
    from weld_motion_planner.io.plan_request import gaps, read
    from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell

    _ensure_kernel()
    request = read(path)
    bundle = Bundle(name or Path(path).stem, root)
    bundle.add_request(request, gaps(request))

    timing: dict[str, float] = {}
    mark = time.time()
    mesh = tessellate(request.step_bytes)
    timing["tessellate"] = time.time() - mark
    bundle.add_mesh(mesh)

    mark = time.time()
    field = bake(mesh, voxel_m=voxel_mm / 1000.0)
    timing["bake_part_field"] = time.time() - mark
    bundle.add_field(field, "part")

    profile = profile_for_cell(request.cell)
    if profile is not None:
        bundle.manifest["cell_profile"] = {
            "driven": profile.driven,
            "held": profile.held,
            "notes": profile.notes,
        }

    mark = time.time()
    joints = request.joints_by_id
    rows: list[dict[str, Any]] = []
    for spec in request.seams:
        seam = Seam(spec, joints.get(spec.get("weld_joint_ref")))
        # One sample per ~2 mm, which is the density the graph will use and
        # dense enough that a corner shows as a corner rather than a chord.
        count = int(max(16, min(600, round(seam.length_m * 1000.0 / 2.0))))
        fractions = torch.linspace(0.0, 1.0, count, dtype=torch.float64)
        got = seam.frames(fractions)
        orientation = spec.get("orientation") or {}
        weave = (spec.get("process") or {}).get("weave")
        work = _profile_at(orientation.get("work_angle_deg"), fractions)
        travel = _profile_at(orientation.get("travel_angle_deg"), fractions)
        u = apply_angles(got["t"], got["r"], work, travel)
        clearance = field.sample(got["pos_m"].to(torch.float32))
        rows.append(
            {
                "id": seam.id,
                "weld_joint_ref": spec.get("weld_joint_ref"),
                "length_mm": round(seam.length_m * 1000.0, 3),
                "reversed": seam.reversed,
                "weave": weave,
                # Stated, not inferred. `{"shape": "none"}` is a truthy dict,
                # so every reader that tested the weave object for presence
                # would report a stringer bead as weaving -- which is how the
                # first read of this bundle concluded all 26 seams weave.
                "weaves": str((weave or {}).get("shape") or "none") != "none",
                "has_band": bool(orientation.get("limits")),
                "work_angle_deg": [round(v, 4) for v in work.tolist()],
                "travel_angle_deg": [round(v, 4) for v in travel.tolist()],
                "points_m": _round(got["pos_m"]),
                "t": _round(got["t"]),
                "r": _round(got["r"]),
                "b": _round(got["b"]),
                "u": _round(u),
                "curvature_1_m": [round(v, 4) for v in got["curvature_1_m"].tolist()],
                # Distance from the weld point to the nearest surface. It is
                # ~0 by construction -- the seam lies ON the part -- so it is
                # here as a CHECK: a seam that reads far from every surface
                # is a seam in the wrong frame.
                "field_at_point_mm": [round(v * 1000.0, 3) for v in clearance.tolist()],
                "degenerate": [bool(v) for v in got["degenerate"].tolist()],
            }
        )
    timing["sample_seams"] = time.time() - mark
    bundle.add_seams(rows)
    bundle.add_timing(timing)
    return bundle.close()


def _round(tensor: torch.Tensor, places: int = 6) -> list[list[float]]:
    return [[round(v, places) for v in row] for row in tensor.tolist()]


def _profile_at(profile: dict[str, Any] | None, fractions: torch.Tensor) -> torch.Tensor:
    """A scalar function of normalized arc length, evaluated.

    Only the two kinds the fixtures use. A profile this does not understand
    reads as zero and SAYS so rather than silently flattening -- a torch
    drawn at 0 degrees when the program said 20 is a picture that lies.
    """
    if not profile:
        return torch.zeros_like(fractions)
    kind = profile.get("kind")
    if kind == "constant":
        return torch.full_like(fractions, float(profile.get("value", 0.0)))
    if kind == "piecewise_linear":
        knots = profile.get("knots") or []
        xs = torch.tensor([float(k[0]) for k in knots], dtype=fractions.dtype)
        ys = torch.tensor([float(k[1]) for k in knots], dtype=fractions.dtype)
        index = torch.clamp(torch.searchsorted(xs, fractions) - 1, 0, len(xs) - 2)
        lo, hi = xs[index], xs[index + 1]
        alpha = ((fractions - lo) / (hi - lo).clamp_min(1e-12)).clamp(0.0, 1.0)
        return ys[index] + alpha * (ys[index + 1] - ys[index])
    raise ValueError(f"the viewer cannot draw a {kind!r} angle profile yet")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", nargs="?", default="", help="a .weldplan to visualise")
    parser.add_argument("--name", default="")
    parser.add_argument("--out", default="")
    parser.add_argument("--voxel-mm", type=float, default=1.5)
    ns = parser.parse_args(argv)
    if not ns.request:
        parser.error("a .weldplan is required")
    where = build(
        Path(ns.request),
        name=ns.name or None,
        root=Path(ns.out) if ns.out else None,
        voxel_mm=ns.voxel_mm,
    )
    manifest = json.loads((where / "manifest.json").read_text())
    print(f"wrote {where}")
    print(f"  open the viewer at /viz.html?bundle={manifest['name']}")
    for key, value in (manifest.get("timing_s") or {}).items():
        print(f"  {key}: {value}s")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"debug: {exc}", file=sys.stderr)
        raise SystemExit(1)
