#!/usr/bin/env python
"""Which positioner angles put the part into the room, precomputed.

## Why this is one number and not a collision check

The workpiece group and the world group both move -- the part swinging into
the H-frame, the table dipping toward the floor -- and neither is spheres.
But their RELATIVE pose depends only on the driven positioner axes, which
during a weld is J7 alone. Whether the part hits the room is a function of
one angle, whatever the arm is doing.

So it is answered once, at bake time, rather than per query: sweep the axis
over its range, carry a surface sampling of the workpiece group through each
angle, ask the world field how close it came, and keep the angles that clear
by a margin. The result is a boolean over a 1-D grid, and it enters the
pipeline where masks already live -- node generation refuses bins outside
it, refinement clips to it as a box constraint, and a held axis gets the
same question asked once as a scalar.

Two driven positioner axes would make this a 2-D cell grid and still cost
milliseconds. It does not generalize to the arm, and it is not meant to: the
arm's pose is six numbers, not one.

## The margin is the sweep's own error, not a safety factor

Between two sampled angles the part keeps turning, and a point at
perpendicular distance `R` from the axis moves `R * step` of arc. Clearing
by less than that at both neighbours says nothing about what happened in
between. So the margin is `R_max * step` computed from the geometry being
swept -- shrink the step and the margin shrinks with it, which is the
property a discretization error should have and a safety factor should not.

## It is a search device, not the safety argument

The mask keeps the optimizer out of angles that were never going to work.
What certifies a trajectory is the exact-mesh CCD verifier, which re-checks the
winner including positioner motion. A mask that is slightly too permissive
costs a wasted search; the verifier is what makes it not cost more.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry import esdf
from weld_motion_planner.geometry.mesh import Mesh
from weld_motion_planner.robot_cell.robot import cell as cellmod

DEFAULT_STEP_RAD = np.deg2rad(0.5)
# Target spacing of the surface sampling carried through the sweep. Coarser
# than the field it is queried against on purpose: the field already knows
# the geometry exactly, and these points only have to find where the group
# comes closest.
DEFAULT_SPACING_M = 0.02


class PositionerError(ValueError):
    """A sweep this refuses to run, with the reason."""


@dataclass(frozen=True)
class Mask:
    """Which values of one axis clear the world, on a 1-D grid."""

    axis: str
    angles_rad: np.ndarray  # (N,)
    clearance_m: np.ndarray  # (N,) the closest the group came, at each angle
    margin_m: float
    step_rad: float

    @property
    def allowed(self) -> np.ndarray:
        return self.clearance_m > self.margin_m

    def allows(self, value_rad: float) -> bool:
        """Is this angle inside the mask?

        Nearest-sample rather than interpolated: the grid IS the resolution
        of the answer, and pretending to more of it between samples would be
        inventing the thing the margin exists to account for.
        """
        if not len(self.angles_rad):
            return False
        return bool(self.allowed[int(np.abs(self.angles_rad - float(value_rad)).argmin())])

    def bins(self) -> list[tuple[float, float]]:
        """Contiguous allowed intervals, as (lo, hi) in radians.

        What node generation wants: a positioner bin is admissible if it
        lies inside one of these, and the graph never has to look at the
        grid itself.
        """
        out: list[tuple[float, float]] = []
        start = None
        for i, ok in enumerate(self.allowed):
            if ok and start is None:
                start = i
            elif not ok and start is not None:
                out.append((float(self.angles_rad[start]), float(self.angles_rad[i - 1])))
                start = None
        if start is not None:
            out.append((float(self.angles_rad[start]), float(self.angles_rad[-1])))
        return out

    def summary(self) -> dict[str, Any]:
        allowed = self.allowed
        return {
            "axis": self.axis,
            "samples": int(len(self.angles_rad)),
            "step_deg": round(float(np.rad2deg(self.step_rad)), 4),
            "margin_mm": round(self.margin_m * 1000.0, 2),
            "allowed": int(allowed.sum()),
            "allowed_fraction": round(float(allowed.mean()), 4) if len(allowed) else 0.0,
            "bins_deg": [[round(float(np.rad2deg(a)), 2), round(float(np.rad2deg(b)), 2)]
                         for a, b in self.bins()],
            "worst_clearance_mm": round(float(self.clearance_m.min()) * 1000.0, 2)
            if len(self.clearance_m) else None,
        }


def surface_points(mesh: Mesh, spacing_m: float = DEFAULT_SPACING_M) -> np.ndarray:
    """A deterministic sampling of a mesh's surface, roughly `spacing` apart.

    Deterministic and not random: `ARCHITECTURE.md` asks the whole pipeline
    to be reproducible from a seed, and a mask that shifts by a sample
    between runs would move the graph's admissible bins with it.

    Each triangle gets a barycentric lattice sized to its own longest edge,
    so a large face is covered and a sliver costs three points.
    """
    tri = mesh.vertices[mesh.faces]  # (F, 3, 3)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    longest = np.maximum.reduce([
        np.linalg.norm(b - a, axis=1),
        np.linalg.norm(c - b, axis=1),
        np.linalg.norm(a - c, axis=1),
    ])
    steps = np.clip(np.ceil(longest / max(spacing_m, 1e-9)).astype(int), 1, 64)

    out: list[np.ndarray] = []
    for n in np.unique(steps):
        sel = steps == n
        us, vs = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = (us + vs) <= n
        fu = (us[keep] / n)[None, :, None]
        fv = (vs[keep] / n)[None, :, None]
        block = a[sel][:, None, :] + fu * (b[sel] - a[sel])[:, None, :] + fv * (c[sel] - a[sel])[:, None, :]
        out.append(block.reshape(-1, 3))
    if not out:
        raise PositionerError("this mesh has no triangles to sample")
    return np.unique(np.round(np.concatenate(out), 9), axis=0)


def sweep(
    cell: dict[str, Any],
    env_field: esdf.DistanceField,
    workpiece_points: np.ndarray,
    *,
    axis: str,
    work_to_piece: np.ndarray,
    held: dict[str, float] | None = None,
    step_rad: float = DEFAULT_STEP_RAD,
    limits_rad: tuple[float, float] | None = None,
    device: torch.device | None = None,
) -> Mask:
    """Sweep one positioner axis and record where the group clears the world.

    `workpiece_points` are in the WORKPIECE frame, which is where the part's
    own geometry lives; `work_to_piece` is the placement that relates it to
    the work frame the axis drives.
    """
    joints = {str(j.get("name")): j for j in (cell.get("joints") or [])}
    if axis not in joints:
        raise PositionerError(f"this cell has no joint named {axis!r}")
    work = (cell.get("frames") or {}).get("work")
    if not work:
        raise PositionerError("this cell names no work frame, so nothing rides the positioner")

    if limits_rad is None:
        raw = joints[axis].get("limits_rad") or {}
        lower = raw.get("lower")
        upper = raw.get("upper")
        limits_rad = (
            float(lower) if lower is not None else -np.pi,
            float(upper) if upper is not None else np.pi,
        )
    lo, hi = float(limits_rad[0]), float(limits_rad[1])
    if not hi > lo:
        raise PositionerError(f"{axis} has an empty range: {lo} to {hi}")
    count = max(2, int(np.ceil((hi - lo) / step_rad)) + 1)
    angles = np.linspace(lo, hi, count)
    step = float(angles[1] - angles[0])

    device = device or env_field.values.device
    base = dict(cellmod.zero_pose(cell).values)
    base.update(held or {})

    # Points into the WORK frame once: the placement does not move with the
    # axis, so carrying it through every angle would be the same multiply
    # repeated `count` times.
    piece = np.concatenate([workpiece_points, np.ones((len(workpiece_points), 1))], axis=1)
    in_work = (piece @ work_to_piece.T)[:, :3]

    # The margin is the arc a point sweeps between samples, so it needs the
    # distance from the AXIS LINE, not from the origin.
    joint = joints[axis]
    direction = np.asarray(joint.get("axis_xyz", (0.0, 0.0, 1.0)), dtype=np.float64)
    direction = direction / (np.linalg.norm(direction) or 1.0)
    along = in_work @ direction
    radius = float(np.linalg.norm(in_work - along[:, None] * direction[None, :], axis=1).max())
    margin = radius * step

    world = np.empty((count, len(in_work), 3), dtype=np.float32)
    for i, value in enumerate(angles):
        matrix = cellmod.link_transforms(cell, cellmod.Pose({**base, axis: float(value)}))[str(work)]
        world[i] = in_work @ matrix[:3, :3].T + matrix[:3, 3]

    points = torch.as_tensor(world, dtype=torch.float32, device=device)
    with torch.no_grad():
        distance = env_field.sample_fused(points.reshape(-1, 3)).reshape(count, -1)
        clearance = distance.min(dim=1).values.cpu().numpy()

    return Mask(axis=axis, angles_rad=angles, clearance_m=clearance,
                margin_m=margin, step_rad=step)


def self_test() -> int:
    if not torch.cuda.is_available():
        print("positioner: no CUDA; the field is a GPU path, nothing to check")
        return 0
    device = torch.device("cuda", torch.cuda.current_device())
    from weld_motion_planner.robot_cell.collision.scene import (
        box_mesh,
        placement_matrix,
    )

    # A table on a post, with a wall on one side. Turning the table swings a
    # long part; half a turn puts it through the wall and half does not, so
    # the mask has a right answer that can be read off the geometry.
    cell = {
        "frames": {"base": "world", "tool": "tool", "work": "table"},
        "links": [{"name": n} for n in ("world", "floor", "table", "tool")],
        "joints": [
            {"name": "world_to_floor", "parent": "world", "child": "floor", "type": "fixed",
             "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
            {"name": "J7", "parent": "floor", "child": "table", "type": "revolute",
             "xyz_m": [0, 0, 0.5], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
             "limits_rad": {"lower": -np.pi, "upper": np.pi}},
            {"name": "Jt", "parent": "floor", "child": "tool", "type": "fixed",
             "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        ],
    }
    # A wall at x = +0.60, and a part reaching 0.68 out along +x of the
    # table -- PAST the wall, so pointing at it is a real collision. The
    # first version of this reached only 0.50 and cleared at every angle,
    # which is a sweep with nothing in it: the mask came back 100% allowed
    # and said nothing about whether the sweep works.
    wall = box_mesh([0.60, -1.0, 0.0], [0.70, 1.0, 1.2])
    part = box_mesh([0.30, -0.05, -0.05], [0.68, 0.05, 0.05])
    field = esdf.bake(wall, voxel_m=0.01, margin_m=0.30, frame="world", device=device)

    points = surface_points(part, spacing_m=0.02)
    mask = sweep(cell, field, points, axis="J7",
                 work_to_piece=placement_matrix({}), device=device)
    info = mask.summary()
    print(f"positioner: {info['samples']} samples at {info['step_deg']} deg, "
          f"margin {info['margin_mm']} mm, {len(points)} surface points")
    print(f"  allowed {info['allowed']}/{info['samples']} = {info['allowed_fraction']:.0%}, "
          f"worst clearance {info['worst_clearance_mm']} mm")
    print(f"  bins (deg): {info['bins_deg']}")

    # Pointing at the wall (0 rad) must be refused; pointing away (pi) must
    # be allowed. If those two come out the same the sweep is not turning
    # anything.
    if mask.allows(0.0):
        print("positioner: FAIL, the part points into the wall at 0 and the mask allows it")
        return 1
    if not mask.allows(np.pi):
        print("positioner: FAIL, the part points away from the wall at pi and the mask refuses it")
        return 1
    if not (0.3 < info["allowed_fraction"] < 0.95):
        print(f"positioner: FAIL, {info['allowed_fraction']:.0%} allowed is not a sweep, "
              "it is an answer that ignored the geometry")
        return 1

    # The margin is a discretization error, so a finer step must shrink it.
    fine = sweep(cell, field, points, axis="J7", work_to_piece=placement_matrix({}),
                 step_rad=np.deg2rad(0.125), device=device)
    print(f"  at 0.125 deg: margin {fine.margin_m * 1000:.2f} mm "
          f"(was {mask.margin_m * 1000:.2f} mm at {info['step_deg']} deg)")
    if not fine.margin_m < mask.margin_m * 0.5:
        print("positioner: FAIL, the margin did not shrink with the step; "
              "it is behaving as a safety factor rather than as the sweep's own error")
        return 1

    print("positioner: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"positioner: {exc}", file=sys.stderr)
        raise SystemExit(1)
