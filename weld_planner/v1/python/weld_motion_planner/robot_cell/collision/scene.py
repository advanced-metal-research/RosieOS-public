#!/usr/bin/env python
"""Everything the arm must not hit, in the frame each thing is rigid in.

`esdf.py` bakes one field. This assembles the several a real cell needs and
answers the one question the optimizer asks: for this joint vector, how far
is every sphere from anything, differentiably.

## Why more than one field

`ARCHITECTURE.md`'s rule: **spheres are the body being checked, fields are
everything it must not hit** -- and a field can only be baked in a frame the
things inside it are rigid in. That splits the cell:

- **world group** -- floor, pedestal, H-frame, the second bay. Rigid in world
  once the held axes are held. One coarse field.
- **workpiece group** -- the part, and the table and fixtures bolted to the
  same positioner. Rigid with respect to the part by construction, so they
  ride the positioner for free. TWO fields in that one frame, because one
  resolution cannot serve both: fine for the part, coarse for the table.

Distance to a union is the min of the samples, so a second field in the same
frame costs one extra lookup and nothing else.

## The trap this module exists to avoid

The arm's world position does not depend on the positioner. Its position
**in the workpiece frame** depends on nothing else. So `d(clearance)/d(J7)`
is zero against the world field and nonzero against the workpiece fields,
and the entire J7 term arrives through the frame transform.

Compute that transform from a buffer -- say, the fused kernel's link
transforms -- and it carries no grad_fn. Everything still runs. The
positioner just silently stops being a degree of freedom, and the optimizer
never discovers the rotation that would have made the seam reachable. So the
work frame is composed in torch by `fk.torch_link_transform`, and
`self_test` asserts both halves of the asymmetry: nonzero against the
workpiece, exactly zero against the world.

## What it does not do

No self-collision -- `pairs.self_ignore` has the mask, the sphere-vs-sphere
test is not built. No J7 feasibility mask. Both are named in M2's remainder.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from functools import cached_property
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry import esdf
from weld_motion_planner.geometry.mesh import Mesh
from weld_motion_planner.robot_cell.collision import pairs, positioner
from weld_motion_planner.robot_cell.robot import cell as cellmod
from weld_motion_planner.robot_cell.robot import fk as fkmod
from weld_motion_planner.utils.device import warp_stream

# A stand-in for "this group does not check this sphere". Large enough that
# it never wins a min, finite so that no NaN can be built from it.
UNCHECKED_M = 1.0e6


class SceneError(ValueError):
    """A scene this refuses to assemble, with the reason."""


_kernel = None


def _make_kernel():
    import warp as wp

    @wp.kernel
    def self_gap(
        points: wp.array(dtype=wp.vec3),   # (B*S,)
        radii: wp.array(dtype=float),      # (S,)
        pair_a: wp.array(dtype=int),       # (P,)
        pair_b: wp.array(dtype=int),       # (P,)
        num_pairs: int,
        num_spheres: int,
        out: wp.array(dtype=float),        # (B*P,)
    ):
        tid = wp.tid()
        b = tid / num_pairs
        k = tid - b * num_pairs
        i = pair_a[k]
        j = pair_b[k]
        delta = points[b * num_spheres + i] - points[b * num_spheres + j]
        out[tid] = wp.length(delta) - radii[i] - radii[j]

    @wp.kernel
    def self_gap_backward(
        grad_out: wp.array(dtype=float),   # (B*P,)
        points: wp.array(dtype=wp.vec3),
        pair_a: wp.array(dtype=int),
        pair_b: wp.array(dtype=int),
        num_pairs: int,
        num_spheres: int,
        grad_points: wp.array(dtype=wp.vec3),  # (B*S,)
    ):
        tid = wp.tid()
        # Most pairs are nowhere near each other, and a hinge cost gives
        # them a gradient of exactly zero. Two atomic adds each, on an arm
        # where every sphere sits in ~167 pairs, is the whole cost of this
        # kernel -- so the ones that cannot move anything leave first.
        g = grad_out[tid]
        if g == 0.0:
            return
        b = tid / num_pairs
        k = tid - b * num_pairs
        i = pair_a[k]
        j = pair_b[k]
        ia = b * num_spheres + i
        ib = b * num_spheres + j
        delta = points[ia] - points[ib]
        length = wp.length(delta)
        if length > 1.0e-9:
            # d|p_i - p_j| / dp_i is the unit vector between them, and the
            # negative of it for p_j. Coincident centres have no direction
            # and no gradient to give; the value is still correct.
            step = g / length * delta
            wp.atomic_add(grad_points, ia, step)
            wp.atomic_add(grad_points, ib, -step)

    return self_gap, self_gap_backward


def _ensure_kernel():
    global _kernel
    if _kernel is None:
        import warp as wp

        wp.init()
        _kernel = _make_kernel()
    return _kernel


_sweep_kernels = None


def _make_sweep_kernel():
    """The swept-sphere kernels: cuRobo's `swept_sphere_obstacle_collision`
    walk, on this scene's fields, with one honesty its kernel does not have.

    Thread = (pose, sphere, group). From the sphere's centre at its sample,
    walk the CHORD toward the previous and the next sample's centre -- half
    of each, the neighbour walks the other half -- reading the field as
    you go and advancing by the clearance you just read (conservative
    advancement: nothing can be closer than that within that distance), by
    at least a radius so the walk always moves, at most `steps` reads per
    half chord. Far from anything the first read clears the half chord and
    the walk costs one query; only near the part does it walk in small
    steps. That is what a torch tensor version cannot do: it reads every
    point whether or not the chord is anywhere near a wall.

    Recorded per thread: the least clearance met, the chord weight of the
    current endpoint where it was met, which neighbour, and the field
    gradient there in WORLD. Backward scatters that gradient to BOTH
    endpoints of the chord, weighted `t` and `1 - t` -- cuRobo attributes
    the whole of it to the current endpoint and relies on the neighbour's
    thread to balance it, which is right on average and wrong per point.
    """
    import warp as wp

    @wp.func
    def _trilinear(values: wp.array(dtype=float), nx: int, ny: int, nz: int,
                   origin: wp.vec3, voxel: float, p: wp.vec3):
        # `esdf.sample_with_grad`, as a function: the same interpolation,
        # the same conservative extension outside the box, the same
        # gradient. The parity test on that kernel is the authority on
        # both; keep them identical.
        upper = wp.vec3(origin[0] + float(nx - 1) * voxel,
                        origin[1] + float(ny - 1) * voxel,
                        origin[2] + float(nz - 1) * voxel)
        cx = wp.clamp(p[0], origin[0], upper[0])
        cy = wp.clamp(p[1], origin[1], upper[1])
        cz = wp.clamp(p[2], origin[2], upper[2])
        dx = p[0] - cx
        dy = p[1] - cy
        dz = p[2] - cz
        outside = wp.sqrt(dx * dx + dy * dy + dz * dz)
        fx = (cx - origin[0]) / voxel
        fy = (cy - origin[1]) / voxel
        fz = (cz - origin[2]) / voxel
        ix = wp.min(int(wp.floor(fx)), nx - 2)
        iy = wp.min(int(wp.floor(fy)), ny - 2)
        iz = wp.min(int(wp.floor(fz)), nz - 2)
        tx = fx - float(ix)
        ty = fy - float(iy)
        tz = fz - float(iz)
        base = (ix * ny + iy) * nz + iz
        c000 = values[base]
        c001 = values[base + 1]
        c010 = values[base + nz]
        c011 = values[base + nz + 1]
        c100 = values[base + ny * nz]
        c101 = values[base + ny * nz + 1]
        c110 = values[base + ny * nz + nz]
        c111 = values[base + ny * nz + nz + 1]
        c00 = c000 * (1.0 - tz) + c001 * tz
        c01 = c010 * (1.0 - tz) + c011 * tz
        c10 = c100 * (1.0 - tz) + c101 * tz
        c11 = c110 * (1.0 - tz) + c111 * tz
        c0 = c00 * (1.0 - ty) + c01 * ty
        c1 = c10 * (1.0 - ty) + c11 * ty
        value = c0 * (1.0 - tx) + c1 * tx
        gx = (c1 - c0) / voxel
        gy = ((c01 - c00) * (1.0 - tx) + (c11 - c10) * tx) / voxel
        gz = (((c001 - c000) * (1.0 - ty) + (c011 - c010) * ty) * (1.0 - tx)
              + ((c101 - c100) * (1.0 - ty) + (c111 - c110) * ty) * tx) / voxel
        if outside > 0.0:
            if dx != 0.0:
                gx = 0.0
            if dy != 0.0:
                gy = 0.0
            if dz != 0.0:
                gz = 0.0
            if outside >= value:
                return outside, wp.vec3(dx / outside, dy / outside, dz / outside)
            return value, wp.vec3(gx, gy, gz)
        return value, wp.vec3(gx, gy, gz)

    @wp.kernel
    def sweep_forward(
        centers: wp.array(dtype=wp.vec3),
        radii: wp.array(dtype=float),
        group_spheres: wp.array(dtype=wp.int32),
        n_g: int, num_spheres: int, samples: int, is_workpiece: int,
        rot: wp.array(dtype=wp.mat33), pos: wp.array(dtype=wp.vec3),
        values: wp.array(dtype=float), nx: int, ny: int, nz: int,
        origin: wp.vec3, voxel: float, steps: int,
        out_min: wp.array(dtype=float), out_t: wp.array(dtype=float),
        out_nbr: wp.array(dtype=wp.int32), out_grad: wp.array(dtype=wp.vec3),
    ):
        tid = wp.tid()
        p = tid / n_g
        k = tid - p * n_g
        s = group_spheres[k]
        h = p - (p / samples) * samples
        r = radii[s]
        cur = centers[p * num_spheres + s]
        R = wp.identity(n=3, dtype=float)
        t0 = wp.vec3(0.0, 0.0, 0.0)
        if is_workpiece == 1:
            R = rot[p]
            t0 = pos[p]
        Rt = wp.transpose(R)
        lcur = Rt * (cur - t0)
        best = float(1.0e6)
        best_t = float(1.0)
        best_nbr = int(-1)
        best_g = wp.vec3(0.0, 0.0, 0.0)
        for direction in range(2):
            nbr = int(-1)
            if direction == 0:
                if h > 0:
                    nbr = p - 1
            else:
                if h < samples - 1:
                    nbr = p + 1
            if nbr >= 0:
                lnb = Rt * (centers[nbr * num_spheres + s] - t0)
                half = wp.length(lnb - lcur) * 0.5
                inv_half = 1.0 / wp.max(half, 0.001)
                jump = float(0.0)
                for _ in range(steps):
                    if jump >= half:
                        break
                    t = 1.0 - 0.5 * jump * inv_half
                    pt = t * lcur + (1.0 - t) * lnb
                    d, g = _trilinear(values, nx, ny, nz, origin, voxel, pt)
                    clear = d - r
                    if clear < best:
                        best = clear
                        best_t = t
                        best_nbr = nbr
                        best_g = R * g
                    # Advance by the clearance (nothing is closer than that
                    # within that distance), and by at least a VOXEL so the
                    # walk always moves. cuRobo's floor is the sphere radius;
                    # at this arm's 20-40 mm spheres that skipped a thin part
                    # feature between two reads and left seam_0011 ->
                    # seam_0016 penetrating by one waypoint at any cap. The
                    # field cannot resolve below a voxel, so a voxel is the
                    # honest floor.
                    if clear > 0.0:
                        jump += wp.max(clear, voxel)
                    else:
                        jump += voxel
        out_min[tid] = best
        out_t[tid] = best_t
        out_nbr[tid] = best_nbr
        out_grad[tid] = best_g

    @wp.kernel
    def sweep_backward(
        grad_out: wp.array(dtype=float), best_t: wp.array(dtype=float),
        best_nbr: wp.array(dtype=wp.int32), best_g: wp.array(dtype=wp.vec3),
        num_spheres: int, grad_centers: wp.array(dtype=wp.vec3),
    ):
        i = wp.tid()
        nbr = best_nbr[i]
        if nbr < 0:
            return
        g = grad_out[i] * best_g[i]
        t = best_t[i]
        s = i - (i / num_spheres) * num_spheres
        wp.atomic_add(grad_centers, i, g * t)
        wp.atomic_add(grad_centers, nbr * num_spheres + s, g * (1.0 - t))

    return sweep_forward, sweep_backward


def _ensure_sweep_kernel():
    global _sweep_kernels
    if _sweep_kernels is None:
        _sweep_kernels = _make_sweep_kernel()
    return _sweep_kernels


class _SweptGap(torch.autograd.Function):
    """The torch face of the swept-sphere kernels: `(P, spheres)` least
    clearance met along each sphere's half chords to its neighbouring
    samples, `UNCHECKED_M` where it had none. Groups are merged here, per
    sphere, so backward is one kernel over the winning reads."""

    @staticmethod
    def forward(ctx, world: torch.Tensor, scene: "Scene", q: torch.Tensor,
                samples: int, steps: int) -> torch.Tensor:
        import warp as wp

        forward_kernel, _ = _ensure_sweep_kernel()
        P, ns, _ = world.shape
        device = world.device
        flat = world.detach().reshape(-1, 3).contiguous()
        centers_wp = wp.from_torch(flat, dtype=wp.vec3, requires_grad=False)
        radii = scene.radii.detach().to(torch.float32).contiguous()
        radii_wp = wp.from_torch(radii, requires_grad=False)
        launch = ({"stream": stream} if (stream := warp_stream(device)) is not None
                  else {"device": wp.device_from_torch(device)})

        best_min = torch.full((P, ns), UNCHECKED_M, dtype=torch.float32, device=device)
        best_t = torch.ones((P, ns), dtype=torch.float32, device=device)
        best_nbr = torch.full((P, ns), -1, dtype=torch.int32, device=device)
        best_g = torch.zeros((P, ns, 3), dtype=torch.float32, device=device)

        rot = pos = None
        if any(g.frame == "workpiece" for g in scene.groups):
            with torch.no_grad():
                r, t = scene.workpiece_frame_from_fused(q.detach())
            rot = r.to(torch.float32).contiguous()
            pos = t.to(torch.float32).contiguous()
        dummy_rot = torch.zeros((1, 3, 3), dtype=torch.float32, device=device)
        dummy_pos = torch.zeros((1, 3), dtype=torch.float32, device=device)

        for group in scene.groups:
            idx = group.spheres.to(torch.int64)
            idx32 = idx.to(torch.int32).contiguous()
            n_g = int(idx.numel())
            values_wp, _flat, origin = group.field._fused_launch_args
            nx, ny, nz = group.field.shape
            is_wp = 1 if group.frame == "workpiece" else 0
            out_min = torch.empty(P * n_g, dtype=torch.float32, device=device)
            out_t = torch.empty(P * n_g, dtype=torch.float32, device=device)
            out_nbr = torch.empty(P * n_g, dtype=torch.int32, device=device)
            out_g = torch.empty(P * n_g, 3, dtype=torch.float32, device=device)
            wp.launch(
                forward_kernel, dim=P * n_g,
                inputs=[
                    centers_wp, radii_wp,
                    wp.from_torch(idx32, requires_grad=False),
                    n_g, ns, int(samples), is_wp,
                    wp.from_torch(rot if is_wp else dummy_rot, dtype=wp.mat33,
                                  requires_grad=False),
                    wp.from_torch(pos if is_wp else dummy_pos, dtype=wp.vec3,
                                  requires_grad=False),
                    values_wp, int(nx), int(ny), int(nz), origin,
                    float(group.field.voxel_m), int(steps),
                    wp.from_torch(out_min, requires_grad=False),
                    wp.from_torch(out_t, requires_grad=False),
                    wp.from_torch(out_nbr, requires_grad=False),
                    wp.from_torch(out_g, dtype=wp.vec3, requires_grad=False),
                ],
                **launch,
            )
            om = out_min.view(P, n_g)
            cur = best_min.index_select(1, idx)
            better = om < cur
            best_min.index_copy_(1, idx, torch.where(better, om, cur))
            best_t.index_copy_(1, idx, torch.where(
                better, out_t.view(P, n_g), best_t.index_select(1, idx)))
            best_nbr.index_copy_(1, idx, torch.where(
                better, out_nbr.view(P, n_g), best_nbr.index_select(1, idx)))
            best_g.index_copy_(1, idx, torch.where(
                better.unsqueeze(-1), out_g.view(P, n_g, 3),
                best_g.index_select(1, idx)))
        ctx.save_for_backward(best_t, best_nbr, best_g)
        ctx.shape = (P, ns)
        return best_min

    @staticmethod
    def backward(ctx, grad_output: torch.Tensor):
        import warp as wp

        _, backward_kernel = _ensure_sweep_kernel()
        best_t, best_nbr, best_g = ctx.saved_tensors
        P, ns = ctx.shape
        device = best_t.device
        grad_centers = torch.zeros(P * ns, 3, dtype=torch.float32, device=device)
        wp.launch(
            backward_kernel, dim=P * ns,
            inputs=[
                wp.from_torch(grad_output.contiguous().reshape(-1).to(torch.float32),
                              requires_grad=False),
                wp.from_torch(best_t.reshape(-1), requires_grad=False),
                wp.from_torch(best_nbr.reshape(-1), requires_grad=False),
                wp.from_torch(best_g.reshape(-1, 3), dtype=wp.vec3,
                              requires_grad=False),
                ns,
                wp.from_torch(grad_centers, dtype=wp.vec3, requires_grad=False),
            ],
            **({"stream": stream} if (stream := warp_stream(device)) is not None
               else {"device": wp.device_from_torch(device)}),
        )
        return grad_centers.view(P, ns, 3), None, None, None, None


def placement_matrix(placement: dict[str, Any]) -> np.ndarray:
    """Work frame to workpiece frame.

    `offline-programming/v1/ui/src/domain/placement.ts` states the convention
    and this must agree with it exactly, or the planner plans against a part
    the authoring side drew somewhere else:

        T_workpiece = T_work(joints) . Translate(offset) . Rpy(rpy) . Translate(-anchor)

    `T_work` is the descriptor's work link. On rosie_1400_v3 that is the middle
    of the table's top face, the OLP's table frame -- not the table link's own
    origin on J7's axis, which put every OLP part 622 mm along the axis and
    100 mm down, inside the bar.

    The anchor translation is INSIDE the rotation, so turning the part spins
    it about its own anchor rather than swinging it around the table centre.
    """
    anchor = placement.get("anchor_xyz_m") or (0.0, 0.0, 0.0)
    offset = placement.get("offset_xyz_m") or (0.0, 0.0, 0.0)
    rpy = placement.get("rpy_rad") or (0.0, 0.0, 0.0)
    return cellmod.transform(offset, rpy) @ cellmod.transform(
        [-float(anchor[0]), -float(anchor[1]), -float(anchor[2])], (0.0, 0.0, 0.0)
    )


@dataclass(frozen=True)
class Group:
    """One rigid group's field, the frame it lives in, and who checks it."""

    label: str
    field: esdf.DistanceField
    frame: str  # "world" or "workpiece"
    spheres: torch.Tensor  # (n,) int64 indices into the scene's sphere array

    def summary(self) -> dict[str, Any]:
        return {"label": self.label, "frame": self.frame,
                "spheres": int(self.spheres.numel()), **self.field.summary()}


@dataclass
class Scene:
    """The assembled collision model for one cell and one placement."""

    fk: Any  # callable q -> (B, S, 3), from fk.make
    spec: fkmod.ChainSpec
    radii: torch.Tensor  # (S,)
    groups: list[Group]
    work_index: int | None  # index into spec.link_names, or None
    work_to_piece: torch.Tensor  # (4, 4) the placement, work -> workpiece
    sphere_links: list[str] = dataclass_field(default_factory=list)
    cell: dict[str, Any] = dataclass_field(default_factory=dict)
    # A surface sampling of the workpiece group, in workpiece coordinates.
    # Kept because the positioner sweep needs exactly this and nothing else
    # in the pipeline has it -- the fields know the geometry but cannot be
    # asked to hand back points on it.
    workpiece_points: np.ndarray = dataclass_field(default_factory=lambda: np.zeros((0, 3)))
    # Sphere index pairs the robot must keep apart from itself, precomputed
    # from `pairs.self_ignore`. Empty when every link is adjacent to every
    # other, which is a two-link arm and not a mistake.
    self_pairs: torch.Tensor | None = None

    @property
    def num_spheres(self) -> int:
        return int(self.radii.numel())

    @cached_property
    def _self_gap_launch_args(self) -> tuple[Any, Any, Any, tuple[torch.Tensor, ...]]:
        """The self-collision kernel's static arguments, built once.

        The pair columns were cast to int32 and wrapped on EVERY launch --
        two device allocations and five wrapper builds per step, the same
        per-launch overhead `robot/fk.py` documents building its wrappers
        once. The source tensors ride along so the memory the wrappers point
        at cannot be collected.
        """
        import warp as wp

        radii = self.radii.contiguous()
        pair_a = self.self_pairs[:, 0].contiguous().to(torch.int32)
        pair_b = self.self_pairs[:, 1].contiguous().to(torch.int32)
        return (wp.from_torch(radii), wp.from_torch(pair_a), wp.from_torch(pair_b),
                (radii, pair_a, pair_b))

    def workpiece_frame(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """The workpiece frame's world pose, differentiably, as (rot, pos).

        Separate from `world_to_workpiece` so that `clearance` composes it
        ONCE and hands it to every workpiece group -- it does not depend on
        which spheres are being asked about, and computing it per group was
        paying for the chain walk twice.
        """
        if self.work_index is None:
            raise SceneError("this cell has no work frame, so nothing is in the workpiece frame")
        rot, pos = fkmod.torch_link_transform(self.spec, self.work_index, q)
        lrot = self.work_to_piece[:3, :3].to(rot)
        lpos = self.work_to_piece[:3, 3].to(pos)
        return rot @ lrot, (rot @ lpos) + pos

    def workpiece_frame_from_fused(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """`workpiece_frame`, read off the fused FK's buffers instead of a
        second chain walk. Only valid right after `fk(q)` ran for this same
        `q` at this batch size -- which is the swept term's situation, and
        the reason it exists: the torch walk it replaces was ~100 launches
        per descent step for a frame the fused kernel had already written.
        Falls back to the walk when the fused callable cannot answer."""
        if self.work_index is None:
            raise SceneError("this cell has no work frame, so nothing is in the workpiece frame")
        last = getattr(self.fk, "last_link_poses", None)
        if last is None:
            return self.workpiece_frame(q)
        lrot, lpos = last(q.shape[0])
        rot = lrot[:, self.work_index].to(q.dtype)
        pos = lpos[:, self.work_index].to(q.dtype)
        lrot_w = self.work_to_piece[:3, :3].to(rot)
        lpos_w = self.work_to_piece[:3, 3].to(pos)
        return rot @ lrot_w, (rot @ lpos_w) + pos

    def world_to_workpiece(self, points: torch.Tensor, q: torch.Tensor,
                           frame: tuple[torch.Tensor, torch.Tensor] | None = None) -> torch.Tensor:
        """(B, n, 3) world points into the workpiece frame, differentiably."""
        trot, tpos = frame if frame is not None else self.workpiece_frame(q)
        # `(p - t) @ R` is `R^T (p - t)` written so that it is a matmul.
        # einsum would read better and its backward dispatches through
        # triton, which this environment cannot compile.
        return (points - tpos[:, None, :]) @ trot

    def self_clearance(self, q: torch.Tensor) -> torch.Tensor:
        """(B, P) gap between each pair of spheres the arm must keep apart.

        A pair is included when the two links are neither the same nor
        adjacent -- `pairs.self_ignore`. Two links across a bearing overlap
        at the mating face by construction, so checking them would report a
        collision at every pose the robot can reach.

        Fused, because the plain-torch version was measured first and lost.
        Gathering both endpoints materialises two `(B, P, 3)` tensors -- 38
        MB at 128 poses, 307 MB at 1024 -- and the arithmetic on them is a
        subtract and a norm. It cost 0.558 ms forward-and-back at batch 128,
        more than the entire rest of the collision term, and went superlinear
        from there. The kernel reads two points and writes one float.
        """
        if self.self_pairs is None or not self.self_pairs.numel():
            return torch.zeros(q.shape[0], 0, device=q.device, dtype=torch.float32)
        return _SelfGap.apply(self.fk(q), self)

    def clearances(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """`(env, self)` -- both fields from ONE forward-kinematics pass.

        `clearance` and `self_clearance` each run the fused FK, and every
        cost asks for the pair, so calling them separately pays the FK twice
        for the same `q`. Worse than waste: the fused FK reuses one scratch
        buffer per batch size, and inside a CUDA graph capture the recorded
        backward of a loss holding TWO fk nodes loses one branch's gradient
        to the aliasing -- measured, the self term's contribution vanished
        from the replay entirely while every single-fk loss recorded
        faithfully. One world, two fields, one fk node.
        """
        world = self.fk(q)
        env = self._clearance_of(world, q)
        if self.self_pairs is None or not self.self_pairs.numel():
            return env, torch.zeros(q.shape[0], 0, device=q.device,
                                    dtype=torch.float32)
        return env, _SelfGap.apply(world, self)

    def clearances_and_world(self, q: torch.Tensor
                             ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """`clearances`, plus the world sphere centres the fields were read at.

        For a caller that wants to read the environment field at points it
        builds FROM those centres -- the chord between two samples' spheres,
        for a swept term -- without a second forward-kinematics node: the
        fused FK's scratch buffer is one per batch size, and two fk nodes in a
        captured backward alias it (see `clearances`). Everything derived from
        `world` shares its single node.
        """
        world = self.fk(q)
        env = self._clearance_of(world, q)
        if self.self_pairs is None or not self.self_pairs.numel():
            return env, torch.zeros(q.shape[0], 0, device=q.device,
                                    dtype=torch.float32), world
        return env, _SelfGap.apply(world, self), world

    def clearance_at(self, world: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        """(N, S) environment clearance at GIVEN world centres, `q` only for
        the workpiece frame. A plain read of the field at points a caller
        built; `chord_clearance` is the swept read."""
        return self._clearance_of(world, q)

    def chord_clearance(self, world: torch.Tensor, q: torch.Tensor,
                        samples: int, steps: int = 3) -> torch.Tensor:
        """`(P, spheres)` the least environment clearance met on the chords
        from each sample's sphere centre to its neighbours' -- the swept
        collision read, `_make_sweep_kernel`'s contract. `world` is the fused
        FK's output so the gradient flows on into the kinematics; `q` sets
        the workpiece frame per pose; `samples` is the poses per motion
        (poses are laid out motion-major, so neighbours are `p +- 1` inside
        a motion). `UNCHECKED_M` where a sphere had no chord."""
        return _SweptGap.apply(world, self, q, int(samples), int(steps))

    def clearance(self, q: torch.Tensor) -> torch.Tensor:
        """(B, S) distance from each sphere's SURFACE to the nearest thing.

        The min across groups is the distance to their union, which is what
        a union of rigid bodies means and why a second field in the same
        frame costs only a lookup.
        """
        return self._clearance_of(self.fk(q), q)

    def _clearance_of(self, world: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
        batch = world.shape[0]
        frame = (self.workpiece_frame(q)
                 if any(g.frame == "workpiece" for g in self.groups) else None)
        stacked = []
        for group in self.groups:
            points = world.index_select(1, group.spheres)
            if group.frame == "workpiece":
                points = self.world_to_workpiece(points, q, frame)
            elif group.frame != "world":
                raise SceneError(f"group {group.label!r} is in unknown frame {group.frame!r}")
            near = group.field.sample_fused(points) - self.radii.index_select(0, group.spheres)
            # Out of place: an in-place scatter into a tensor that requires
            # grad is the kind of thing autograd refuses at the worst moment.
            full = torch.full(
                (batch, self.num_spheres), UNCHECKED_M, dtype=near.dtype, device=near.device
            )
            stacked.append(full.index_copy(1, group.spheres, near))
        if not stacked:
            raise SceneError("this scene has no groups; there is nothing to collide with")
        return torch.stack(stacked, dim=0).min(dim=0).values

    def positioner_mask(self, axis: str, **kwargs) -> positioner.Mask:
        """Which values of a positioner axis keep the workpiece out of the room.

        Answered once against the world field, because the relative pose of
        the two groups depends on this axis and nothing else -- see
        `weld_motion_planner.robot_cell.collision.positioner`.
        """
        world = next((g for g in self.groups if g.frame == "world"), None)
        if world is None:
            raise SceneError("this scene has no world field to sweep against")
        if not len(self.workpiece_points):
            raise SceneError("this scene carries no workpiece geometry to sweep")
        return positioner.sweep(
            self.cell, world.field, self.workpiece_points, axis=axis,
            work_to_piece=self.work_to_piece.detach().cpu().numpy().astype(np.float64),
            **kwargs,
        )

    def checker(self, axis_names: list[str],
                held: dict[str, Any] | None = None) -> PoseCheck:
        """A batched collision check for poses named by the CALLER's axes.

        See `PoseCheck`. Bound once, because the axis mapping and the held
        values are per-planning-problem, not per-batch.
        """
        return PoseCheck(self, axis_names, held)

    def summary(self) -> dict[str, Any]:
        return {
            "spheres": self.num_spheres,
            "sphere_links": list(self.sphere_links),
            "axes": list(self.spec.axis_names),
            "work_frame": None if self.work_index is None else self.spec.link_names[self.work_index],
            "self_pairs": 0 if self.self_pairs is None else int(self.self_pairs.shape[0]),
            "groups": [g.summary() for g in self.groups],
        }


class PoseCheck:
    """One home for "my q, this scene's chain": every planning stage names its
    poses by its OWN axes -- the lattice by arm-then-positioner, M5 and M6 by
    `cell.moved_axes` -- while the scene drives the whole cell's chain in its
    own order. Four stages each hand-scattered columns by name, and they had
    already drifted: the lattice screen left the held axes at the ZERO a fresh
    tensor holds, where M5 filled them with the profile's values.

    `held` pins the axes the caller does not move: a float, or a `(N,)` tensor
    for a batch whose rows are pinned differently (M5, where each seam may sit
    on its own station).

    DIFFERENTIABLE on purpose: nothing here detaches, so the same object
    serves the screen -- which wraps it in `no_grad` and chunks -- and an
    optimizer that backpropagates through `clearance`.
    """

    def __init__(self, scene: Scene, axis_names: list[str],
                 held: dict[str, Any] | None = None):
        self.scene = scene
        column_of = {name: i for i, name in enumerate(scene.spec.axis_names)}
        # (own column, scene column) for the axes this scene actually drives;
        # an axis it does not know is baked into its frames and has no column.
        self.columns = [(i, column_of[name])
                        for i, name in enumerate(axis_names)
                        if name in column_of]
        self.held = {column_of[name]: value for name, value in (held or {}).items()
                     if name in column_of}

    def q_full(self, q: torch.Tensor) -> torch.Tensor:
        """`(N, len(axis_names))` in the caller's order -> `(N, num_dof)` in the
        scene's, held axes filled, at the scene's own dtype."""
        dtype = self.scene.radii.dtype
        full = torch.zeros(q.shape[0], self.scene.spec.num_dof,
                           dtype=dtype, device=q.device)
        for own, column in self.columns:
            full[:, column] = q[:, own].to(dtype)
        for column, value in self.held.items():
            full[:, column] = (value.to(dtype) if torch.is_tensor(value)
                               else float(value))
        return full

    def clearance(self, q: torch.Tensor) -> torch.Tensor:
        """`(N,)` the worst gap ANYWHERE at each pose -- fields and the arm
        against itself merged, because "is this pose in trouble" has one
        answer. A caller that needs the two apart (`roadmap`, where they are
        different failures with different fixes) reads the scene directly.

        THE ARM AGAINST ITSELF is not optional. The lattice screen once
        skipped `self_clearance`: re-judging screened-feasible nodes on
        `bracket_a_placed` seam 1 with exact meshes found 60% were collisions,
        the largest category `link_4` against `link_6` -- the arm folded into
        itself, which no world geometry can see."""
        full = self.q_full(q)
        gap = self.scene.clearance(full).min(dim=-1).values
        if self.scene.self_pairs is not None and self.scene.self_pairs.numel():
            gap = torch.minimum(gap, self.scene.self_clearance(full).min(dim=-1).values)
        return gap


class _SelfGap(torch.autograd.Function):
    """The torch face of the self-collision kernel.

    Backward scatters into the sphere positions with atomics -- each sphere
    sits in about 167 pairs on this arm -- and the gradient flows on into
    the kinematics from there, because the input it is given is the fused
    FK's output.
    """

    @staticmethod
    def forward(ctx, world: torch.Tensor, scene: Scene) -> torch.Tensor:
        import warp as wp

        forward_kernel, _ = _ensure_kernel()
        batch, spheres, _ = world.shape
        pairs_n = int(scene.self_pairs.shape[0])
        flat = world.detach().reshape(-1, 3).contiguous()
        out = torch.empty(batch * pairs_n, dtype=torch.float32, device=world.device)
        radii_wp, pair_a_wp, pair_b_wp, _keepalive = scene._self_gap_launch_args
        wp.launch(
            forward_kernel,
            dim=batch * pairs_n,
            inputs=[
                # `requires_grad=False`: the Function carries its own gradient,
                # and `wp.from_torch` would otherwise allocate a warp grad
                # buffer per call -- illegal inside CUDA graph capture.
                wp.from_torch(flat, dtype=wp.vec3, requires_grad=False),
                radii_wp,
                pair_a_wp,
                pair_b_wp,
                pairs_n, spheres,
                wp.from_torch(out, requires_grad=False),
            ],
            # Torch's CURRENT stream, not warp's default: unordered on a side
            # stream, silently absent from a CUDA graph capture otherwise.
            **({"stream": stream} if (stream := warp_stream(world.device)) is not None
               else {"device": wp.device_from_torch(world.device)}),
        )
        ctx.save_for_backward(flat)
        ctx.scene = scene
        ctx.shape = (batch, spheres, pairs_n)
        return out.reshape(batch, pairs_n)

    @staticmethod
    def backward(ctx, grad_output: torch.Tensor):
        import warp as wp

        _, backward_kernel = _ensure_kernel()
        (flat,) = ctx.saved_tensors
        scene = ctx.scene
        batch, spheres, pairs_n = ctx.shape
        grad_points = torch.zeros_like(flat)
        _radii_wp, pair_a_wp, pair_b_wp, _keepalive = scene._self_gap_launch_args
        wp.launch(
            backward_kernel,
            dim=batch * pairs_n,
            inputs=[
                wp.from_torch(grad_output.contiguous().reshape(-1),
                              requires_grad=False),
                wp.from_torch(flat, dtype=wp.vec3, requires_grad=False),
                pair_a_wp,
                pair_b_wp,
                pairs_n, spheres,
                wp.from_torch(grad_points, dtype=wp.vec3, requires_grad=False),
            ],
            **({"stream": stream} if (stream := warp_stream(flat.device)) is not None
               else {"device": wp.device_from_torch(flat.device)}),
        )
        return grad_points.reshape(batch, spheres, 3), None


def cell_obstacles(cell: dict[str, Any], mesh_root: Any, *,
                   held: dict[str, float] | None = None,
                   extra_static: list[Mesh] | None = None,
                   ) -> tuple[Mesh | None, Mesh | None, str | None]:
    """`(env, fixtures, fixtures_link)`: the cell's own structure, as obstacles.

    `extra_static` are bodies in the WORLD frame that are not in the cell's
    descriptor: the operator's fixtures from the plan request (a stand under
    the part), built by `collision.fixtures`. They join the env group, moved
    into the base frame with the same transform as the cell's static links.

    Everything the arm can hit that is not the workpiece -- the floor, the
    positioner's frame, the table the part is bolted to. `build` has taken
    these since it was written and no caller ever passed them, so the screen
    ran against the part alone and called postures feasible that put the
    forearm through the table. `docs/issues/0006`.

    The split is by FRAME and it is not cosmetic. Bodies that ride the work
    link turn with the positioner, so they must be baked in the work frame
    where their field turns with them; bodies that do not are baked in world.
    Getting this backwards bakes the weld table at one positioner angle and
    screens every other angle against a table that is not there -- which is
    exactly the shape of the bug this function exists to fix, one level down.

    A body driven by a HELD axis counts as world: `held` is what the lattice
    pins, so an axis that never moves makes everything beyond it static. The
    caller passes the same dictionary it passed to `nodes.build`.

    Meshes are loaded through `verify.load_dae` -- imported here rather than at
    module scope because `verify` is the verifier and this is the screen, and the
    screen has no business depending on the verifier for anything but a file
    reader.
    """
    from pathlib import Path

    from weld_motion_planner.verifier.verify import kinematics, load_dae

    root = Path(mesh_root)
    kin = kinematics(cell)
    frames = kin.transforms([float((held or {}).get(a, 0.0)) for a in kin.axis_names])
    work = str((cell.get("frames") or {}).get("work") or "")

    # Which links move with an axis the lattice is free to turn. Walking the
    # joints is what makes this a property of the CELL rather than a list of
    # names somebody has to keep up to date.
    moving: set[str] = set()
    free = {str(j["name"]) for j in (cell.get("joints") or [])
            if str(j.get("type") or "fixed") != "fixed"} - set(held or {})
    for joint in kin.joints:
        parent, child = str(joint["parent"]), str(joint["child"])
        if parent in moving or str(joint.get("name")) in free:
            moving.add(child)

    tool = str((cell.get("frames") or {}).get("tool") or "")
    arm: set[str] = set()
    walk = tool
    while walk:
        arm.add(walk)
        walk = kin.parent_of.get(walk, "")
    arm -= {kin.base}

    def merge(names: list[str], into: str) -> Mesh | None:
        if not names:
            return None
        inverse = np.linalg.inv(frames[into])
        vertices: list[np.ndarray] = []
        faces: list[np.ndarray] = []
        base = 0
        for name in names:
            path = root / f"{name}.dae"
            if not path.is_file():
                continue
            mesh = load_dae(path)
            place = inverse @ frames[name]
            moved = np.asarray(mesh.vertices) @ place[:3, :3].T + place[:3, 3]
            vertices.append(moved)
            faces.append(np.asarray(mesh.faces) + base)
            base += len(moved)
        if not vertices:
            return None
        return Mesh(vertices=np.concatenate(vertices), faces=np.concatenate(faces))

    # Ride the body the work frame is BOLTED to, not the work link itself. On
    # rosie_1400_v3 the work frame is a fixed marker on the table's top face
    # (`seam_worker.robot.cell.WORK_SURFACES`), and the table is the marker's
    # PARENT: riding the marker dropped the table's own mesh from every screen.
    # NOT the arm, the same exclusion `static` makes one line down. On
    # `rosie_1400_v3` the arm does not ride the table and `_rides` drops it for
    # free; on a cell whose work frame IS its base -- `rosie_1420_v1`, an arm on
    # a pedestal with the part fixtured to the floor -- everything rides the
    # carrier, so the arm's own meshes were baked into the fixture field at the
    # held pose. The screen then measured the arm against a frozen copy of
    # itself and called EVERY pose a collision, at every placement.
    carrier = _carrier(cell, work) if work else ""
    on_work = [n for n in kin.links if n != work and work and n in moving
               and n not in arm and _rides(kin, n, carrier)]
    # A link that does not MOVE is static, whatever it sits on the chain to.
    # This excluded everything on the chain to the tool as well, and that is
    # not the same set: the fixed mount BELOW the first joint is on that chain
    # and never moves. `rosie_1400_v3`'s `floor` was dropped by it -- the very
    # body `env_checked` is written about, whose header measures link_1's
    # 355 mm clearance to everything else so that link_1 can be excused from a
    # field with the floor in it. There was no floor in the field. On
    # `rosie_1420_v1` the same rule dropped `pedestal` and `base`, and its arm
    # reaches 0.0 mm into both over 400 random postures, so the screen could
    # not see the mount the robot is bolted to.
    static = [n for n in kin.links if n not in moving and n != kin.base]
    env = merge(static, kin.base)
    if extra_static:
        from weld_motion_planner.robot_cell.collision.fixtures import concatenate

        inverse = np.linalg.inv(frames[kin.base])
        moved = [
            Mesh(vertices=np.asarray(m.vertices) @ inverse[:3, :3].T + inverse[:3, 3],
                 faces=np.asarray(m.faces))
            for m in extra_static
        ]
        env = concatenate(([env] if env is not None else []) + moved)
    return (env,
            merge([work] + on_work if work else [], work) if work else None,
            work or None)


def _carrier(cell: dict[str, Any], link: str) -> str:
    """The body `link` is rigid with: up through the cell's FIXED joints only.

    The cell's own joint types and not the kinematics' -- `kinematics` bakes a
    held axis into a fixed joint, and climbing through J7 when it is held would
    carry the climb to the floor and put the arm on the work frame.
    """
    into = {str(j["child"]): j for j in (cell.get("joints") or [])}
    walk = link
    while walk in into and str(into[walk].get("type") or "fixed") == "fixed":
        walk = str(into[walk]["parent"])
    return walk


def _rides(kin, name: str, link: str) -> bool:
    """Is `name` at or below `link` in the tree?"""
    walk = name
    while walk:
        if walk == link:
            return True
        walk = kin.parent_of.get(walk, "")
    return False


def build(
    cell: dict[str, Any],
    model_links: dict[str, list],
    *,
    device: torch.device,
    placement: dict[str, Any] | None = None,
    part: Mesh | None = None,
    fixtures: Mesh | None = None,
    fixtures_link: str | None = None,
    env: Mesh | None = None,
    part_voxel_m: float = esdf.DEFAULT_VOXEL_M,
    coarse_voxel_m: float = 0.005,
    margin_m: float = esdf.DEFAULT_MARGIN_M,
    sample_spacing_m: float = positioner.DEFAULT_SPACING_M,
) -> Scene:
    """Assemble the scene.

    Meshes are passed in rather than loaded, so this module holds no policy
    about where a cell's assets live -- the part comes from the plan request's
    STEP, the rest from the cell bundle, and both callers already know how
    to get them.

    `fixtures` and `env` arrive in the WORK and WORLD frames respectively,
    which is where a cell descriptor's link transforms put them. The fixture
    mesh is moved into the workpiece frame here, since that is the frame its
    field has to be baked in.
    """
    frames = cell.get("frames") or {}
    work = frames.get("work")
    arm_chain = cellmod.sphere_links(cell)
    on_arm = [name for name in arm_chain if model_links.get(name)]
    if not on_arm:
        raise SceneError("no arm link carries spheres; author a sphere model first")

    # The work link joins the FK spec even though it carries no spheres:
    # its transform is the workpiece frame, and putting it here is what puts
    # the positioner axis into `spec.axis_names`.
    links = list(on_arm)
    work_index = None
    if work and work not in links:
        links.append(str(work))
        work_index = len(links) - 1
    elif work:
        work_index = links.index(str(work))

    fn, spec, radii, sphere_link, _local = fkmod.make(cell, links, model_links, device)
    order = {name: i for i, name in enumerate(links)}
    sphere_link_names = [links[int(i)] for i in sphere_link]

    def indices(names: list[str]) -> torch.Tensor:
        keep = [i for i, name in enumerate(sphere_link_names) if name in set(names)]
        return torch.tensor(keep, dtype=torch.long, device=device)

    every = torch.arange(len(sphere_link), dtype=torch.long, device=device)
    lmat = placement_matrix(placement or {})
    groups: list[Group] = []

    if env is not None:
        env_names = [
            str(link.get("name")) for link in (cell.get("links") or [])
            if str(link.get("name")) not in arm_chain
        ]
        checked = pairs.env_checked(cell, on_arm, env_names)
        idx = indices(checked)
        if idx.numel():
            # THE ENV BOX HAS TO CLEAR THE BIGGEST SPHERE, and `margin_m` alone
            # does not. Outside the grid `DistanceField.sample` answers
            # `max(boundary value, distance to the box)` -- a lower bound, and
            # deliberately so -- but both terms are about `margin_m` just
            # outside a face, while a sphere's radius is not. `rosie_1420_v1`
            # stands on a pedestal, so its env box wraps the mount alone: at
            # the default 50 mm margin, link_2's 80 mm sphere sits 98 mm above
            # the box and reads 18.0 mm at EVERY pose, where the exact meshes
            # are 88.9 mm apart. Nothing could clear the 20 mm floor and the
            # roadmap refused all 4096 of its samples.
            #
            # `margin_m + the largest radius` puts the geometry at least a
            # radius from every face, so a sphere just outside reads positive
            # and grows outward from there. Measured on that cell: 18.0 mm
            # becomes 90.0 mm, which is the exact-mesh answer. The 1400's arm
            # was already inside its own env box -- its floor spans the cell --
            # so this only costs it a bigger grid, 43.3M voxels to 76.0M.
            env_margin = float(margin_m) + float(radii.max()) if radii.numel() else margin_m
            groups.append(Group("env", esdf.bake(
                env, voxel_m=coarse_voxel_m, margin_m=env_margin, frame="world",
                device=device,
            ), "world", idx))

    if part is not None:
        groups.append(Group("part", esdf.bake(
            part, voxel_m=part_voxel_m, margin_m=margin_m, frame="workpiece", device=device,
        ), "workpiece", every))

    if fixtures is not None:
        inverse = np.linalg.inv(lmat)
        moved = Mesh(fixtures.vertices @ inverse[:3, :3].T + inverse[:3, 3], fixtures.faces)
        groups.append(Group("fixtures", esdf.bake(
            moved, voxel_m=coarse_voxel_m, margin_m=margin_m, frame="workpiece", device=device,
        ), "workpiece", every))

    # The workpiece group's surface, in workpiece coordinates, for the
    # positioner sweep.
    #
    # The fixture body is dropped from the SAMPLING when it is adjacent to
    # environment geometry -- the table hangs off the H-frame through the
    # positioner joint, so it touches it at the bearing at every angle and
    # would make the mask empty for a reason that is not about the part.
    # Measured on `rosie_1400_v3`: the table reads -0.00 mm against the
    # H-frame at every angle, and +409 mm against the floor and bay B, so
    # nothing is lost by leaving it out here.
    #
    # Its FIELD stays. The arm still has to not hit the table; it is only
    # the sweep, which asks a question about the positioner, that the
    # bearing contact is meaningless for. A cell whose table can swing into
    # the floor needs the world field split instead, and this is where that
    # would go.
    piece_meshes: list[Mesh] = []
    if part is not None:
        piece_meshes.append(part)
    if fixtures is not None:
        adjacent = pairs.adjacency(cell)
        env_names = {
            str(link.get("name")) for link in (cell.get("links") or [])
            if str(link.get("name")) not in arm_chain
        }
        touches_env = fixtures_link is not None and any(
            frozenset((fixtures_link, name)) in adjacent for name in env_names
        )
        if not touches_env:
            inverse = np.linalg.inv(lmat)
            piece_meshes.append(
                Mesh(fixtures.vertices @ inverse[:3, :3].T + inverse[:3, 3], fixtures.faces)
            )
    sampled = [positioner.surface_points(m, spacing_m=sample_spacing_m) for m in piece_meshes]
    piece_points = np.concatenate(sampled) if sampled else np.zeros((0, 3))

    if any(g.frame == "workpiece" for g in groups) and work_index is None:
        raise SceneError(
            "a workpiece-frame field was asked for but the cell names no work frame, "
            "so there is nowhere to fixture a part"
        )

    # Sphere pairs that are neither on the same link nor on adjacent ones.
    mask = pairs.self_ignore(cell, on_arm)
    slot = {name: i for i, name in enumerate(on_arm)}
    owner = np.array([slot[name] for name in sphere_link_names], dtype=np.int64)
    allowed = np.asarray(mask, dtype=bool)[owner[:, None], owner[None, :]]
    # Upper triangle only: a pair is one constraint, not two.
    ia, ib = np.nonzero(np.triu(allowed, k=1))
    pair_index = (
        torch.tensor(np.stack([ia, ib], axis=1), dtype=torch.long, device=device)
        if len(ia) else torch.zeros((0, 2), dtype=torch.long, device=device)
    )

    return Scene(
        fk=fn, spec=spec, radii=radii, groups=groups, work_index=work_index,
        work_to_piece=torch.tensor(lmat, dtype=torch.float32, device=device),
        sphere_links=sphere_link_names, cell=cell, workpiece_points=piece_points,
        self_pairs=pair_index,
    )


def box_mesh(lo, hi) -> Mesh:
    """An axis-aligned box, wound outward. For tests, and for a fixture a
    cell descriptor gives dimensions for rather than geometry."""
    lo = np.asarray(lo, dtype=np.float64)
    hi = np.asarray(hi, dtype=np.float64)
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    # Indices are (x, y, z) bits: 0b_xyz. Every normal points OUT, and the
    # first version of this had them all pointing in -- which reads as a
    # solid describing the complement, exactly the whole-surface inversion
    # `esdf.py` says the winding number cannot rescue. Nine of the ten tests
    # around it passed anyway, because a sign-flipped field is still a
    # field; only asking what the distance is at the centre caught it.
    # `test_box_mesh_is_wound_outward` now asks.
    f = np.array([
        [0, 1, 3], [0, 3, 2],   # x = lo
        [4, 7, 5], [4, 6, 7],   # x = hi
        [0, 5, 1], [0, 4, 5],   # y = lo
        [2, 3, 7], [2, 7, 6],   # y = hi
        [0, 2, 6], [0, 6, 4],   # z = lo
        [1, 7, 3], [1, 5, 7],   # z = hi
    ], dtype=np.int32)
    return Mesh(v, f)


def _demo_cell() -> dict[str, Any]:
    """A two-link arm on a floor, with a positioner table beside it."""
    return {
        "frames": {"base": "world", "tool": "link_3", "work": "table"},
        "links": [{"name": n} for n in ("world", "floor", "link_1", "link_2", "link_3", "table")],
        "joints": [
            {"name": "world_to_floor", "parent": "world", "child": "floor", "type": "fixed",
             "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
            {"name": "J1", "parent": "floor", "child": "link_1", "type": "revolute",
             "xyz_m": [0, 0, 0.30], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1]},
            {"name": "J2", "parent": "link_1", "child": "link_2", "type": "revolute",
             "xyz_m": [0.30, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 1, 0]},
            # Within the arm's reach on purpose: a demo where the arm
            # cannot get near the part exercises only the conservative
            # extension, and the first version of this did exactly that --
            # every number it printed came from outside the grid.
            # link_3 exists so that link_1 and link_3 are NON-adjacent and
            # the self-collision path has a pair to check. Two links are all
            # adjacent to each other, which is a valid arm and a useless test.
            {"name": "J3", "parent": "link_2", "child": "link_3", "type": "revolute",
             "xyz_m": [0.30, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 1, 0]},
            {"name": "J7", "parent": "floor", "child": "table", "type": "revolute",
             "xyz_m": [0.67, 0, 0.30], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 1, 0]},
        ],
    }


def self_test() -> int:
    if not torch.cuda.is_available():
        print("scene: no CUDA; the fields are a GPU path, nothing to check")
        return 0
    device = torch.device("cuda", torch.cuda.current_device())
    cell = _demo_cell()
    model = {
        "link_1": [((0.15, 0.0, 0.0), 0.06)],
        "link_2": [((0.10, 0.0, 0.0), 0.05), ((0.25, 0.0, 0.0), 0.05)],
        "link_3": [((0.10, 0.0, 0.0), 0.05)],
    }
    scene = build(
        cell, model, device=device,
        placement={"anchor_xyz_m": [0, 0, 0], "offset_xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        part=box_mesh([-0.08, -0.08, 0.0], [0.08, 0.08, 0.16]),
        fixtures=box_mesh([-0.25, -0.25, -0.04], [0.25, 0.25, 0.0]),
        env=box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
        part_voxel_m=0.004, coarse_voxel_m=0.01, margin_m=0.12,
    )
    info = scene.summary()
    print(f"scene: {info['spheres']} spheres, axes {info['axes']}, work frame {info['work_frame']!r}")
    for g in info["groups"]:
        print(f"  {g['label']:<9} {g['frame']:<9} {g['spheres']} spheres  "
              f"{g['shape']}  {g['memory_mib']} MiB  {g['voxel_mm']} mm")

    if [g["label"] for g in info["groups"]] != ["env", "part", "fixtures"]:
        print("scene: FAIL, groups are not env + part + fixtures")
        return 1
    if "J7" not in scene.spec.axis_names:
        print("scene: FAIL, the positioner axis is not in the FK spec, so nothing can move the part")
        return 1

    index = {name: i for i, name in enumerate(scene.spec.axis_names)}
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, index["J2"]] = -0.6      # reach the arm out toward the table
    with torch.no_grad():
        d = scene.clearance(q)
    print(f"  clearance at the test pose: min {float(d.min()) * 1000:.1f} mm over {d.numel()} spheres")
    if not torch.isfinite(d).all() or float(d.max()) > UNCHECKED_M / 2:
        print("scene: FAIL, some sphere was left unchecked by every group")
        return 1

    # Outside every grid, `sample` returns the conservative extension -- a
    # distance to the BOX, not to the geometry -- so a pose that never
    # enters a grid checks nothing about the field. The first version of
    # this demo put the table beyond the arm's reach and every number it
    # printed was the extension.
    reached = 0
    with torch.no_grad():
        world = scene.fk(q)
        for group in scene.groups:
            pts = world.index_select(1, group.spheres)
            if group.frame == "workpiece":
                pts = scene.world_to_workpiece(pts, q)
            lo, hi = group.field.origin_pos_m, group.field.upper_pos_m
            n = int((~((pts < lo) | (pts > hi)).any(dim=-1)).sum())
            print(f"  {group.label:<9} {n} of {pts.shape[1]} spheres inside the grid")
            reached += n
    if reached == 0:
        print("scene: FAIL, no sphere is inside any grid -- this pose only tests the extension")
        return 1

    # The asymmetry `ARCHITECTURE.md` warns about, asserted in both
    # directions. Against the world field the positioner cannot matter;
    # against the workpiece fields it is the ONLY way it matters, and a
    # detached work transform would zero it without any other symptom.
    def grad_wrt(labels: list[str]) -> torch.Tensor:
        sub = Scene(fk=scene.fk, spec=scene.spec, radii=scene.radii,
                    groups=[g for g in scene.groups if g.label in labels],
                    work_index=scene.work_index, work_to_piece=scene.work_to_piece,
                    sphere_links=scene.sphere_links)
        qa = q.clone().requires_grad_(True)
        sub.clearance(qa).sum().backward()
        return qa.grad[0].clone()

    world_g = grad_wrt(["env"])
    piece_g = grad_wrt(["part", "fixtures"])
    j7 = index["J7"]
    print(f"  d(sum clearance)/d(J7):  world field {float(world_g[j7]):+.3e}   "
          f"workpiece fields {float(piece_g[j7]):+.3e}")
    if abs(float(world_g[j7])) > 1e-9:
        print("scene: FAIL, the world field moved with the positioner; the frames are crossed")
        return 1
    if abs(float(piece_g[j7])) < 1e-6:
        print("scene: FAIL, the workpiece fields did not move with the positioner -- "
              "the work transform is detached and J7 is invisible to the optimizer")
        return 1

    # Self-collision: the pairs, and the hand-written backward against a
    # reference that is not itself.
    pair_count = int(scene.self_pairs.shape[0])
    print(f"  self-collision pairs: {pair_count} "
          f"(all pairs would be {scene.num_spheres * (scene.num_spheres - 1) // 2})")
    if pair_count == 0:
        print("scene: FAIL, no self-collision pairs -- every link is adjacent to every other")
        return 1

    # A pose worked out rather than guessed. Both elbows at pi fold the arm
    # straight back: link_3's sphere lands at x = 0.10 and link_1's sits at
    # x = 0.15, both at z = 0.30, so their centres are 50 mm apart against
    # radii summing to 110 mm -- a 60 mm overlap. A "folded" pose that
    # happened not to touch would test the arithmetic and not the answer.
    folded = torch.zeros(1, scene.spec.num_dof, device=device)
    folded[0, index["J2"]] = float(np.pi)
    folded[0, index["J3"]] = float(np.pi)
    qa = folded.clone().requires_grad_(True)
    weight = torch.randn(1, pair_count, device=device, generator=torch.Generator(device=device).manual_seed(3))
    (scene.self_clearance(qa) * weight).sum().backward()

    qb = folded.clone().requires_grad_(True)
    world = scene.fk(qb)
    a, b = scene.self_pairs[:, 0], scene.self_pairs[:, 1]
    delta = world.index_select(1, a) - world.index_select(1, b)
    ref = torch.linalg.vector_norm(delta, dim=-1) - (
        scene.radii.index_select(0, a) + scene.radii.index_select(0, b))
    (ref * weight).sum().backward()

    scale = max(float(qb.grad.abs().max()), 1e-12)
    err = float((qa.grad - qb.grad).abs().max()) / scale
    with torch.no_grad():
        gap = scene.self_clearance(folded)
    print(f"  folded pose: min self gap {float(gap.min()) * 1000:.1f} mm, "
          f"{int((gap < 0).sum())} pairs overlapping")
    if int((gap < 0).sum()) == 0:
        print("scene: FAIL, the folded pose does not self-collide, so nothing here "
              "checks that a self-collision is detected at all")
        return 1
    if abs(float(gap.min()) + 0.060) > 0.002:
        print(f"scene: FAIL, the overlap is {float(gap.min()) * 1000:.1f} mm and the "
              "geometry says -60.0 mm")
        return 1
    print(f"  self-collision gradient vs a torch reference: worst relative {err:.2e}")
    if err > 1e-4:
        print("scene: FAIL, the fused self-collision backward disagrees with plain torch")
        return 1

    print("scene: ok")
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
        print(f"scene: {exc}", file=sys.stderr)
        raise SystemExit(1)
