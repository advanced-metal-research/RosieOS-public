#!/usr/bin/env python
"""The collision stage end to end: arm spheres against the cell's ESDF.

`bench_esdf.py` measured the field on its own -- bake time, query
throughput, accuracy against cuRoboV2. This measures the thing the
optimizer actually calls: forward kinematics over the arm, the authored
sphere model carried along by it, and one fused field query per sphere per
timestep, with a gradient that has to come back through all of it to the
joint angles.

## What is a field and what is a sphere

`ARCHITECTURE.md`'s rigid-group rule, made concrete for `rosie_1400_v3`:

    spheres   link_1..link_6, end_effector, tool0   the body being checked
    field     floor, h_frame, positioner_table_b    world-static
    field     positioner_table_a + part + fixtures  rides J7, its own frame

Only the first field is baked here. The second is the workpiece frame's,
and it is a different measurement -- it moves with the positioner, so the
arm's distance to it depends on J7 and the J7 feasibility mask is what
handles workpiece-versus-world. Mixing them into one grid would bake a
pose into a field that is supposed to outlive poses.

## What the numbers mean

A trajectory is on the order of a hundred timesteps and the optimizer runs
hundreds of iterations, so the quantity that matters is not one query but
`timesteps x spheres` queries per iteration, differentiated. That is the
number this reports.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import zipfile
from pathlib import Path
from typing import Any

import numpy as np
import torch

V1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V1 / "python"))

from weld_motion_planner.geometry import esdf  # noqa: E402
from weld_motion_planner.geometry.mesh import Mesh  # noqa: E402
from weld_motion_planner.robot_cell.collision import (
    pairs,  # noqa: E402
    spheres,  # noqa: E402
)
from weld_motion_planner.robot_cell.robot import cell as cellmod  # noqa: E402
from weld_motion_planner.robot_cell.robot import fk as fkmod  # noqa: E402

# Same convention as `seam_worker.paths.CELL_ROOT`, restated rather than
# imported: this is a bench tool, but the import boundary that keeps
# `weld_motion_planner` free of the authoring stack is easier to keep if tools do
# not casually reach across it either.
CELL_ROOT = Path(os.environ.get("WELD_CELL_ROOT") or (V1.parents[1] / "robot_description" / "robots"))

# World-static geometry. `positioner_table_a` is deliberately absent: it
# rides J7 and belongs to the workpiece frame's field.
ENV_LINKS = ("floor", "h_frame", "positioner_table_b")


def read_cell(plan: Path) -> dict[str, Any]:
    with zipfile.ZipFile(plan) as archive:
        return json.loads(archive.read("cell.json"))


def load_visual(link: str, cell_id: str) -> Mesh | None:
    """A link's visual mesh, in its own frame.

    Read directly rather than through trimesh, which wants `pycollada` for
    this format. These files are STL converted by the cell exporter: one
    float array of positions, one triangle list, a single input at offset
    zero. Twenty lines of ElementTree covers that exactly, and keeps this
    tool runnable in the `motion` environment rather than only in `bench`.

    No up-axis conversion. The files declare Z_UP and the URDF's frames are
    already Z-up, so the loader-side rotation that `ui/spheres.ts` has to
    undo never happens here.
    """
    import xml.etree.ElementTree as ET

    path = CELL_ROOT / cell_id / "meshes" / f"{link}.dae"
    if not path.is_file():
        return None
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(path).getroot()
    verts: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for geometry in root.iter(f"{{{ns['c']}}}mesh"):
        array = geometry.find("c:source/c:float_array", ns)
        if array is None or not array.text:
            continue
        points = np.fromstring(array.text, sep=" ", dtype=np.float64).reshape(-1, 3)
        indices: list[int] = []
        for block in geometry.findall("c:triangles", ns):
            inputs = block.findall("c:input", ns)
            stride = max((int(i.get("offset") or 0) for i in inputs), default=0) + 1
            p = block.find("c:p", ns)
            if p is None or not p.text:
                continue
            flat = np.fromstring(p.text, sep=" ", dtype=np.int64)
            indices.extend(flat[::stride].tolist())
        if not indices:
            continue
        verts.append(points)
        faces.append(np.asarray(indices, dtype=np.int32).reshape(-1, 3) + offset)
        offset += len(points)
    if not faces:
        return None
    return Mesh(np.concatenate(verts), np.concatenate(faces))


def env_mesh(cell: dict[str, Any], pose: cellmod.Pose) -> Mesh:
    """The world-static geometry as one triangle soup, in world coordinates."""
    places = cellmod.link_transforms(cell, pose)
    verts: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for name in ENV_LINKS:
        mesh = load_visual(name, str(cell.get("id")))
        if mesh is None:
            continue
        matrix = places[name]
        moved = mesh.vertices @ matrix[:3, :3].T + matrix[:3, 3]
        verts.append(moved)
        faces.append(mesh.faces + offset)
        offset += len(moved)
    if not faces:
        raise SystemExit("none of the env links had a mesh; check WELD_CELL_ROOT")
    return Mesh(np.concatenate(verts), np.concatenate(faces))


# --------------------------------------------------------------------------
# differentiable forward kinematics over the arm
# --------------------------------------------------------------------------


class ArmFK:
    """Torch FK for the sphere-bearing chain.

    Written here rather than in `weld_motion_planner.robot_cell.robot.cell` on purpose: `cell.py` is
    the torch-free structural reader that the fast smoke checks run, and it
    stays that way until M4 needs FK in the optimizer for real. What this
    proves is that the chain composes and differentiates; where it lives is
    a later decision.
    """

    def __init__(self, cell: dict[str, Any], links: list[str], device: torch.device):
        self.device = device
        self.links = links
        parents = {str(j.get("child")): j for j in (cell.get("joints") or [])}
        base = str((cell.get("frames") or {}).get("base"))

        # For each sphere-bearing link, the chain of joints from the base.
        self.chains: dict[str, list[dict[str, Any]]] = {}
        for link in links:
            chain: list[dict[str, Any]] = []
            node = link
            while node in parents and node != base:
                joint = parents[node]
                chain.append(joint)
                node = str(joint.get("parent"))
            self.chains[link] = list(reversed(chain))

        # Every revolute joint that appears on any of those chains, in a
        # stable order -- this is the vector the optimizer will own.
        seen: list[str] = []
        for link in links:
            for joint in self.chains[link]:
                name = str(joint.get("name"))
                if joint.get("type") == "revolute" and name not in seen:
                    seen.append(name)
        self.axis_names = seen
        self.index = {name: i for i, name in enumerate(seen)}

        self.origins: dict[str, torch.Tensor] = {}
        self.axes: dict[str, torch.Tensor] = {}
        for link in links:
            for joint in self.chains[link]:
                name = str(joint.get("name"))
                if name in self.origins:
                    continue
                self.origins[name] = torch.tensor(
                    cellmod.transform(joint.get("xyz_m", (0, 0, 0)), joint.get("rpy_rad", (0, 0, 0))),
                    dtype=torch.float32, device=device,
                )
                axis = np.asarray(joint.get("axis_xyz", (0.0, 0.0, 1.0)), dtype=np.float64)
                norm = float(np.linalg.norm(axis)) or 1.0
                self.axes[name] = torch.tensor(axis / norm, dtype=torch.float32, device=device)

    def _rotation(self, name: str, q: torch.Tensor) -> torch.Tensor:
        """(B,4,4) for one revolute joint at angles `q` of shape (B,)."""
        axis = self.axes[name]
        batch = q.shape[0]
        kx, ky, kz = axis[0], axis[1], axis[2]
        zero = torch.zeros((), dtype=q.dtype, device=q.device)
        cross = torch.stack([
            torch.stack([zero, -kz, ky]),
            torch.stack([kz, zero, -kx]),
            torch.stack([-ky, kx, zero]),
        ])
        eye = torch.eye(3, dtype=q.dtype, device=q.device)
        sin = torch.sin(q).reshape(batch, 1, 1)
        cos = torch.cos(q).reshape(batch, 1, 1)
        rot = eye + sin * cross + (1.0 - cos) * (cross @ cross)
        out = torch.zeros(batch, 4, 4, dtype=q.dtype, device=q.device)
        out[:, :3, :3] = rot
        out[:, 3, 3] = 1.0
        return out

    def transforms(self, q: torch.Tensor) -> dict[str, torch.Tensor]:
        """World transform per sphere-bearing link, for (B, A) joint values."""
        batch = q.shape[0]
        eye = torch.eye(4, dtype=q.dtype, device=q.device).expand(batch, 4, 4)
        cache: dict[tuple[str, ...], torch.Tensor] = {(): eye}
        out: dict[str, torch.Tensor] = {}
        for link in self.links:
            key: tuple[str, ...] = ()
            matrix = eye
            for joint in self.chains[link]:
                name = str(joint.get("name"))
                key = key + (name,)
                if key in cache:
                    matrix = cache[key]
                    continue
                step = self.origins[name].expand(batch, 4, 4)
                if joint.get("type") == "revolute":
                    step = step @ self._rotation(name, q[:, self.index[name]])
                matrix = matrix @ step
                cache[key] = matrix
            out[link] = matrix
        return out


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--plan", default=str(V1 / "data" / "motion" / "bracket_a.weldplan"))
    parser.add_argument("--voxel-mm", type=float, default=5.0, help="env field resolution")
    parser.add_argument("--timesteps", type=int, default=128, help="poses per batch, as a trajectory would have")
    parser.add_argument("--iters", type=int, default=50, help="timed optimizer iterations")
    ns = parser.parse_args(argv)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    cell = read_cell(Path(ns.plan))
    cell_id = str(cell.get("id"))
    pose = cellmod.zero_pose(cell)

    model = spheres.load(cell_id)
    if model is None:
        raise SystemExit(f"no sphere model for {cell_id}; author one with tools/sphere_tool")
    # The environment is what is NOT on the arm chain -- not "whatever has
    # no spheres". `end_effector` and `tool0` are arm links that nobody has
    # covered yet, and calling them environment made `link_6` adjacent to
    # the environment and dropped it from the check.
    arm_chain = cellmod.sphere_links(cell)
    on_arm = [name for name in arm_chain if model.links.get(name)]
    env_names = [str(link.get("name")) for link in (cell.get("links") or [])
                 if str(link.get("name")) not in arm_chain]
    # A link bolted to environment geometry is dropped from the environment
    # check: its cover includes the mating face by construction, so it reads
    # as a permanent collision. See `weld_motion_planner.robot_cell.collision.pairs`.
    covered = pairs.env_checked(cell, on_arm, env_names)
    dropped = [n for n in on_arm if n not in covered]
    print(f"cell {cell_id} on {device}")
    print(f"  sphere links: {', '.join(covered)}")
    print(f"  spheres: {sum(len(model.links[n]) for n in covered)} over {len(covered)} links"
          + (f"  (dropped {', '.join(dropped)}: adjacent to the environment)" if dropped else ""))

    # ---- bake the world-static field -------------------------------------
    mesh = env_mesh(cell, pose)
    print(f"  env mesh: {len(mesh.faces)} triangles from {', '.join(ENV_LINKS)}")
    start = time.perf_counter()
    field = esdf.bake(mesh, voxel_m=ns.voxel_mm / 1000.0, margin_m=0.10, frame="world", device=device)
    torch.cuda.synchronize() if device.type == "cuda" else None
    bake_s = time.perf_counter() - start
    info = field.summary()
    print(f"  bake: {bake_s * 1000:.0f} ms for {info['shape']} = {info['voxels'] / 1e6:.1f}M voxels, "
          f"{info['memory_mib']} MiB, {info['voxel_mm']} mm")

    # ---- the arm, and its spheres ----------------------------------------
    fk = ArmFK(cell, covered, device)
    print(f"  driven axes: {', '.join(fk.axis_names)}")

    local: dict[str, torch.Tensor] = {}
    radii_parts: list[torch.Tensor] = []
    for link in covered:
        entries = model.links[link]
        local[link] = torch.tensor([c for c, _ in entries], dtype=torch.float32, device=device)
        radii_parts.append(torch.tensor([r for _, r in entries], dtype=torch.float32, device=device))
    radii = torch.cat(radii_parts)
    total = int(radii.numel())

    # The fused path: one kernel from joint angles to world sphere centres,
    # replacing the few dozen small torch launches the chain used to cost.
    fused_fk, fused_spec, fused_radii, _, _ = fkmod.make(cell, covered, model.links, device)

    def torch_points(q: torch.Tensor) -> torch.Tensor:
        places = fk.transforms(q)
        return torch.cat([
            torch.einsum("bij,nj->bni", places[link][:, :3, :3], local[link])
            + places[link][:, None, :3, 3]
            for link in covered
        ], dim=1)

    def clearance(q: torch.Tensor, fused: bool = True) -> torch.Tensor:
        """(B, S) distance from each sphere's SURFACE to the env geometry."""
        if fused:
            return field.sample_fused(fused_fk(q)) - fused_radii
        return field.sample_fused(torch_points(q)) - radii

    # ---- one honest pose first -------------------------------------------
    axes = len(fk.axis_names)
    q0 = torch.zeros(1, axes, device=device)
    with torch.no_grad():
        d0 = clearance(q0)
    print(f"\n  at the zero pose: min clearance {float(d0.min()) * 1000:.1f} mm, "
          f"{int((d0 < 0).sum())} of {total} spheres in collision")

    # ---- gradient parity against finite differences ----------------------
    #
    # Two objectives, because they fail differently. The plain sum of
    # clearances is smooth apart from the field's own trilinear kinks; the
    # hinge is what the optimizer will actually use and adds a kink at every
    # sphere sitting exactly on the margin. Comparing both separates "the
    # kernel's derivative is wrong" from "central differences across a kink
    # are not the derivative".
    #
    # And a sweep over the step size, because that is the tell. A genuine
    # error is flat in eps; a kink artefact shrinks with it, since a smaller
    # step is less likely to straddle one.

    def objective(qq: torch.Tensor, hinge: bool) -> torch.Tensor:
        d = clearance(qq)
        return torch.relu(0.05 - d).pow(2).sum() if hinge else d.sum()

    # The field itself, fused against the reference path, on exactly the
    # points the arm puts there. If these disagree, nothing above is worth
    # reading -- and unlike a synthetic point cloud, this covers the cases
    # the arm actually produces: centres deep inside the floor, and centres
    # outside the grid box where the conservative extension takes over.
    start_pose = (torch.rand(1, axes, device=device) - 0.5) * 0.8
    with torch.no_grad():
        places = fk.transforms(start_pose)
        pts = torch.cat([
            torch.einsum("bij,nj->bni", places[link][:, :3, :3], local[link]) + places[link][:, None, :3, 3]
            for link in covered
        ], dim=1).reshape(-1, 3)
    lower, upper = field.origin_pos_m, field.upper_pos_m
    outside = int(((pts < lower) | (pts > upper)).any(dim=-1).sum())
    a = pts.clone().requires_grad_(True)
    va = field.sample_fused(a)
    va.sum().backward()
    b = pts.clone().requires_grad_(True)
    vb = field.sample(b)
    vb.sum().backward()
    print(f"\n  field parity on {len(pts)} arm-sphere centres ({outside} outside the grid box):")
    print(f"    value   max |fused - reference| = {float((va - vb).detach().abs().max()) * 1000:.4f} mm")
    print(f"    gradient max |fused - reference| = {float((a.grad - b.grad).abs().max().detach()):.4f} "
          f"(a unit vector's worth is 1.0)")

    # Who is actually pushing on the cost, and are any of them in the
    # extension? The extension is `max(boundary, exit distance)`, which is
    # C0 -- continuous, with a kink on the surface where the two terms
    # cross. Every sphere in the ACTIVE set that sits in the extension is a
    # place central differences and the derivative are entitled to disagree.
    with torch.no_grad():
        d_here = clearance(start_pose).reshape(-1)
        out_mask = ((pts < lower) | (pts > upper)).any(dim=-1)
        active = d_here < 0.05
    print(f"    of {int(active.sum())} spheres inside the 50 mm margin, "
          f"{int((active & out_mask).sum())} are in the extension")

    print()
    for hinge in (False, True):
        q = start_pose.clone().requires_grad_(True)
        objective(q, hinge).backward()
        analytic = q.grad.clone().reshape(-1)
        row = []
        for eps in (1e-2, 1e-3, 1e-4):
            numeric = torch.zeros_like(analytic)
            with torch.no_grad():
                for a in range(axes):
                    step = torch.zeros(1, axes, device=device)
                    step[0, a] = eps
                    plus = objective(start_pose + step, hinge)
                    minus = objective(start_pose - step, hinge)
                    numeric[a] = (plus - minus) / (2 * eps)
            scale = max(float(numeric.abs().max()), 1e-9)
            row.append(f"eps {eps:g}: {float((analytic - numeric).abs().max()) / scale:.1e}")
        label = "hinge cost" if hinge else "sum of clearances"
        print(f"  gradient vs central differences, {label:<18} {'   '.join(row)}")

    # ---- throughput, as the optimizer would run it -----------------------
    batch = ns.timesteps
    traj = (torch.rand(batch, axes, device=device) - 0.5) * 1.2
    traj.requires_grad_(True)
    for _ in range(3):  # warm up
        torch.relu(0.05 - clearance(traj)).pow(2).sum().backward()
        traj.grad = None
    if device.type == "cuda":
        torch.cuda.synchronize()
    start = time.perf_counter()
    for _ in range(ns.iters):
        loss = torch.relu(0.05 - clearance(traj)).pow(2).sum()
        loss.backward()
        traj.grad = None
    if device.type == "cuda":
        torch.cuda.synchronize()
    elapsed = time.perf_counter() - start

    # Where the time goes. Doubling the timestep count barely moves the
    # per-iteration cost, which says the field query is not the bottleneck
    # -- the arm chain is, in launch overhead for a few dozen tiny matmuls.
    # Worth knowing before anyone optimizes the kernel that is already fast.
    def timed(fn, reps: int) -> float:
        for _ in range(3):
            fn()
        if device.type == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(reps):
            fn()
        if device.type == "cuda":
            torch.cuda.synchronize()
        return (time.perf_counter() - t0) / reps

    with torch.no_grad():
        fk_torch = timed(lambda: torch_points(traj.detach()), ns.iters)
        fk_fused = timed(lambda: fused_fk(traj.detach()), ns.iters)
        pts_flat = torch_points(traj.detach()).reshape(-1, 3).contiguous()
        query_only = timed(lambda: field.sample_fused(pts_flat), ns.iters)

    # And the whole iteration through the torch chain, for the comparison
    # the fused kernel exists to win.
    traj.grad = None
    for _ in range(3):
        torch.relu(0.05 - clearance(traj, fused=False)).pow(2).sum().backward()
        traj.grad = None
    if device.type == "cuda":
        torch.cuda.synchronize()
    t0 = time.perf_counter()
    for _ in range(ns.iters):
        torch.relu(0.05 - clearance(traj, fused=False)).pow(2).sum().backward()
        traj.grad = None
    if device.type == "cuda":
        torch.cuda.synchronize()
    torch_iter = (time.perf_counter() - t0) / ns.iters

    per_iter = elapsed / ns.iters
    queries = batch * total
    print(f"\n  forward only, {batch * total:,} points:"
          f"  FK torch {fk_torch * 1000:.2f} ms"
          f"  FK fused {fk_fused * 1000:.2f} ms"
          f"  field query {query_only * 1000:.2f} ms")
    print(f"  full iteration (fwd+bwd):  torch chain {torch_iter * 1000:.2f} ms"
          f"  fused chain {per_iter * 1000:.2f} ms"
          f"  ({torch_iter / per_iter:.1f}x)")
    print(f"\n  trajectory: {batch} timesteps x {total} spheres = {queries:,} queries per iteration")
    print(f"  forward + backward: {per_iter * 1000:.2f} ms per iteration"
          f"  ({1.0 / per_iter:.0f} iterations/s)")
    print(f"  throughput: {queries / per_iter / 1e6:.1f}M sphere-queries/s, differentiated")

    # A trajectory optimizer wants hundreds of iterations inside a second or
    # two; state the budget that implies rather than leaving it as a ratio.
    print(f"  200 iterations would take {200 * per_iter * 1000:.0f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
