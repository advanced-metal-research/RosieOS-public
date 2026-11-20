#!/usr/bin/env python
"""Forward kinematics as one kernel: joint angles straight to world spheres.

## Why this exists

`bench_collision.py` measured the collision stage and found the field was
not the bottleneck -- the arm was:

    forward only, 128 timesteps x 300 spheres
      FK (torch ops)      0.64 ms
      field query         0.08 ms

The FLOP count is nothing: 128 poses times eight links times a 3x3 multiply.
What costs is that torch spends it across several dozen kernel launches with
an allocation apiece, and the tell is that doubling the timesteps did not
change the per-iteration time at all. That is launch overhead, and the only
way to remove launch overhead is to launch less.

So: one kernel from `q` to world sphere centres, which is the same move
cuRobo makes (`KinematicsFusedFunction` emits `batch_robot_spheres` directly
rather than link transforms for someone else to apply).

## The shape of it

Two stages, cuRobo's split. One thread per (pose, LINK) walks that link's
chain and writes its world transform; one thread per (pose, SPHERE) reads
that transform and applies it. Cost is `poses x (links x depth + spheres)`
rather than `poses x spheres x depth`, and the link poses fall out as a
first-class result -- the tool pose and the seam frames want them too.

The first version had one thread per sphere walk its own chain, on the
argument that redundant arithmetic is free next to a launch. True at one
trajectory and false past a couple of thousand poses, where it went
compute-bound at 3.1G sphere-updates a second while cuRobo's split was
still launch-bound at 8.4G. Measured, then changed.

The backward pass is analytic. For a revolute joint `j`, the world centre
moves as

    dp/dq_j = a_j x (p - o_j) = a_j x p - a_j x o_j

so the second term is the same for every sphere on that pose. Stage one
stores the axis and that moment PER JOINT -- a few kilobytes -- and
backward reconstructs each column from them. The first version stored a
full Jacobian per sphere instead: 177 MB at 16k poses, written in forward
and read back in backward, for six numbers a thread can rebuild from two.

## Where the time actually goes

At these kernel sizes the driver is the cost, not the GPU. Batch 1 and
batch 16384 took the same 0.08 ms before any of this, which is the
signature of an overhead-bound launch. Two things follow, and both are
worth more than any arithmetic:

- The warp array wrappers are built once, not per launch. `wp.from_torch`
  validates and constructs a Python object, and at four arrays a launch it
  was a measurable share of a kernel that takes tens of microseconds.
- Both stages are captured into a CUDA graph and replayed. Three trips
  through the driver become one.

Together those took the forward from 0.085 ms to 0.010 ms at batch 128.

## What it is not

Not a general kinematics library. It handles the fixed and revolute joints
this cell's URDF has, in a tree, with the base at identity. Prismatic axes,
mimic joints, and closed chains are absent because nothing here has them,
and each would be a new branch in the innermost loop.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import Any

import numpy as np

from weld_motion_planner.robot_cell.robot import cell as cellmod
from weld_motion_planner.utils.device import warp_stream


class FKError(ValueError):
    """A chain this refuses to compile, with the reason."""


# `fk_backward` folds each sphere's Jacobian columns into named registers --
# `acc0` through `acc5` -- because a kernel cannot index a local array, and
# the accumulator is chosen by the joint's POSITION in the chain. Six covers
# a six-axis arm exactly. A seventh revolute joint on one chain would land in
# the final `else`, add itself to `acc5`, and return a silently wrong
# gradient, so the limit is checked here rather than discovered there.
MAX_CHAIN_DOF = 6


@dataclass(frozen=True)
class ChainSpec:
    """The arm, flattened for the GPU.

    One entry per (link, joint-on-its-chain), duplicated across links that
    share a prefix. The duplication is deliberate: an indexed walk up a
    parent array costs a dependent load per step, and this arm's chains are
    six deep.
    """

    link_names: list[str]
    axis_names: list[str]
    start: np.ndarray        # (L,) int32, first chain entry for link
    count: np.ndarray        # (L,) int32, entries in its chain
    origin_rot: np.ndarray   # (E, 3, 3) float32
    origin_pos: np.ndarray   # (E, 3) float32
    axis: np.ndarray         # (E, 3) float32, unit, meaningless where dof < 0
    dof: np.ndarray          # (E,) int32, index into q, or -1 for a fixed joint
    max_depth: int
    # Exactly one chain entry per degree of freedom is flagged to WRITE that
    # joint's world frame. Every link whose chain contains a joint computes
    # the identical frame for it, so letting them all write would be a
    # benign race -- but one writer is one write, and at 16k poses that is
    # the difference between a store and five.
    owner: np.ndarray        # (E,) uint8
    # The degrees of freedom on each link's chain, flattened. Backward walks
    # this rather than the full chain, since a fixed joint contributes no
    # column.
    jstart: np.ndarray       # (L,) int32
    jcount: np.ndarray       # (L,) int32
    jdof: np.ndarray         # (K,) int32
    # For the single-kernel path: which link index each chain entry ends at,
    # or -1 when that entry's child link carries no spheres.
    chain_link: np.ndarray   # (E,) int32

    @property
    def num_dof(self) -> int:
        return len(self.axis_names)


def build(cell: dict[str, Any], links: list[str]) -> ChainSpec:
    """Flatten each link's chain from the base, in base-first order."""
    parents = {str(j.get("child")): j for j in (cell.get("joints") or [])}
    base = str((cell.get("frames") or {}).get("base") or "")

    chains: list[list[dict[str, Any]]] = []
    for link in links:
        chain: list[dict[str, Any]] = []
        node = link
        seen: set[str] = set()
        while node in parents and node != base:
            if node in seen:
                raise FKError(f"link {link!r} is its own ancestor; this cell is not a tree")
            seen.add(node)
            chain.append(parents[node])
            node = str(parents[node].get("parent"))
        chains.append(list(reversed(chain)))

    # Everything that is not `revolute` is folded into the fixed origin below,
    # which is right for `fixed` and WRONG for anything else that moves. A
    # prismatic or continuous joint would be silently frozen: the kernel would
    # report a dof count the verifier disagrees with, and the planner would
    # optimise over an arm one axis shorter than the one that exists. Refused
    # loudly instead, because there is no cell in this repo with either and a
    # cell that grows one should fail at build rather than at a weld.
    MOVES = {"revolute", "continuous", "prismatic", "planar", "floating"}
    unsupported = sorted({
        f"{joint.get('name')} ({joint.get('type')})"
        for chain in chains for joint in chain
        if str(joint.get("type") or "fixed") in MOVES - {"revolute"}
    })
    if unsupported:
        raise FKError(
            "this kinematics models revolute joints only, and folding these into "
            f"the fixed origin would freeze axes the cell actually has: {unsupported}")

    axis_names: list[str] = []
    for chain in chains:
        for joint in chain:
            name = str(joint.get("name"))
            if joint.get("type") == "revolute" and name not in axis_names:
                axis_names.append(name)
    index = {name: i for i, name in enumerate(axis_names)}

    start = np.zeros(len(links), dtype=np.int32)
    count = np.zeros(len(links), dtype=np.int32)
    origin_rot: list[np.ndarray] = []
    origin_pos: list[np.ndarray] = []
    axis: list[np.ndarray] = []
    dof: list[int] = []
    for i, chain in enumerate(chains):
        start[i] = len(dof)
        count[i] = len(chain)
        for joint in chain:
            matrix = cellmod.transform(
                joint.get("xyz_m", (0.0, 0.0, 0.0)), joint.get("rpy_rad", (0.0, 0.0, 0.0))
            )
            origin_rot.append(matrix[:3, :3])
            origin_pos.append(matrix[:3, 3])
            if joint.get("type") == "revolute":
                raw = np.asarray(joint.get("axis_xyz", (0.0, 0.0, 1.0)), dtype=np.float64)
                norm = float(np.linalg.norm(raw))
                if norm < 1e-12:
                    raise FKError(f"joint {joint.get('name')!r} has a zero axis")
                axis.append(raw / norm)
                dof.append(index[str(joint.get("name"))])
            else:
                axis.append(np.array([0.0, 0.0, 1.0]))
                dof.append(-1)

    # A chain with NO joint on it is legal, and the reason is a real cell:
    # `rosie_1420_v1` is an arm on a pedestal whose work frame IS the base, so
    # the chain to the work frame is empty and the workpiece does not move.
    # This used to refuse it, and the refusal reached `RobotCell.of` as
    # "no joints on any of these links' chains" -- a message about kinematics
    # for a cell whose only unusual property is that it has no positioner.
    #
    # An empty chain is not a missing chain: `torch_link_transform` walks
    # `count[i]` entries and returns the identity for zero of them, which is
    # the right answer. What a cell may NOT have is a static chain to its TOOL
    # frame, and `RobotCell.of` is what says so, because only the caller knows
    # which role it asked for.
    dof_arr = np.asarray(dof, dtype=np.int32)
    # One writer per degree of freedom: the first entry that mentions it,
    # which by construction belongs to the shortest chain containing it.
    owner = np.zeros(len(dof_arr), dtype=np.uint8)
    claimed: set[int] = set()
    for e, d in enumerate(dof_arr):
        if d >= 0 and int(d) not in claimed:
            claimed.add(int(d))
            owner[e] = 1

    jstart = np.zeros(len(links), dtype=np.int32)
    jcount = np.zeros(len(links), dtype=np.int32)
    jdof: list[int] = []
    for i in range(len(links)):
        jstart[i] = len(jdof)
        for k in range(int(count[i])):
            d = int(dof_arr[int(start[i]) + k])
            if d >= 0:
                jdof.append(d)
        jcount[i] = len(jdof) - jstart[i]

    # A chain entry ends at the child link of its joint. Record its index
    # when that link is one we track, so the single-kernel path can write
    # every link transform its walk passes through.
    order = {name: i for i, name in enumerate(links)}
    chain_link = np.full(len(dof_arr), -1, dtype=np.int32)
    for i, chain in enumerate(chains):
        for k, joint in enumerate(chain):
            child = str(joint.get("child"))
            if child in order:
                chain_link[int(start[i]) + k] = order[child]

    deepest = int(jcount.max()) if len(jcount) else 0
    if deepest > MAX_CHAIN_DOF:
        raise FKError(
            f"link {links[int(jcount.argmax())]!r} has {deepest} revolute joints on its "
            f"chain and the backward kernel folds at most {MAX_CHAIN_DOF}; widen the "
            f"accumulator ladder in `fk_backward` before using a chain this deep"
        )

    return ChainSpec(
        link_names=list(links),
        axis_names=axis_names,
        start=start,
        count=count,
        # Reshaped, not just cast: with no chain entries at all `np.asarray([])`
        # is `(0,)` and everything downstream indexes these as `(E, 3, 3)` and
        # `(E, 3)`. An empty array of the WRONG shape survives until something
        # slices it.
        origin_rot=np.asarray(origin_rot, dtype=np.float32).reshape(-1, 3, 3),
        origin_pos=np.asarray(origin_pos, dtype=np.float32).reshape(-1, 3),
        axis=np.asarray(axis, dtype=np.float32).reshape(-1, 3),
        dof=dof_arr,
        max_depth=int(count.max()) if len(count) else 0,
        owner=owner,
        jstart=jstart,
        jcount=jcount,
        jdof=np.asarray(jdof, dtype=np.int32),
        chain_link=chain_link,
    )


# Uploaded chain constants, per (spec, device, dtype). Keyed by id and
# holding the spec alive, so an id cannot be reused under the cache.
_UPLOADED: dict[tuple[int, str, Any], tuple[Any, ...]] = {}


def _chain_tensors(spec: ChainSpec, device, dtype):
    """The chain's constant matrices, on device, built once.

    `torch.as_tensor` on a numpy array is a HOST-TO-DEVICE COPY, and doing
    it per call made this function 0.285 ms against 0.013 ms for the entire
    fused kinematics -- twenty-two times the cost of the thing it supports,
    for a chain three joints long. Nothing here depends on `q`.
    """
    import torch

    key = (id(spec), str(device), dtype)
    hit = _UPLOADED.get(key)
    if hit is not None:
        return hit[1:]
    origin_rot = torch.as_tensor(spec.origin_rot, device=device, dtype=dtype)
    origin_pos = torch.as_tensor(spec.origin_pos, device=device, dtype=dtype)
    # `.reshape(-1, 3, 3)` for the same reason `build` reshapes its arrays: a
    # chain with no entries makes `np.asarray([])` a `(0,)`, and these are
    # indexed per entry.
    skew = [_skew(a) for a in spec.axis]
    cross = torch.as_tensor(np.asarray(skew).reshape(-1, 3, 3), device=device, dtype=dtype)
    cross2 = torch.as_tensor(np.asarray([k @ k for k in skew]).reshape(-1, 3, 3),
                             device=device, dtype=dtype)
    eye = torch.eye(3, device=device, dtype=dtype)
    built = (origin_rot, origin_pos, cross, cross2, eye)
    _UPLOADED[key] = (spec, *built)
    return built


def _skew(axis) -> np.ndarray:
    """The cross-product matrix of a unit axis, as plain numpy.

    Constant per joint, so it never belongs in the autograd graph.
    """
    x, y, z = (float(v) for v in axis)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def torch_link_transform(spec: ChainSpec, link_index: int, q):
    """One link's world transform, in plain torch, differentiably.

    The fused kernel writes link transforms into a buffer, and a buffer is
    not part of the autograd graph. That is fine for the spheres, whose
    positions come out of `Fused.apply` and carry a grad_fn -- but the
    WORKPIECE frame is a link transform, and the arm's distance to the part
    depends on the positioner only through it.

    `ARCHITECTURE.md` names this trap: a detached `T_work` silently zeroes
    `d(cost)/d(J7)`. Everything still runs, the positioner just stops being
    a degree of freedom the optimizer can see. So the work frame is composed
    here instead, in torch, where the chain is one link deep in practice and
    the gradient is torch's problem.

    Returns `(rot, pos)` shaped `(B, 3, 3)` and `(B, 3)`.
    """
    import torch

    batch = q.shape[0]
    device, dtype = q.device, q.dtype
    origin_rot, origin_pos, cross_all, cross2_all, eye = _chain_tensors(spec, device, dtype)

    rot = eye.repeat(batch, 1, 1)
    pos = torch.zeros(batch, 3, device=device, dtype=dtype)
    for k in range(int(spec.count[link_index])):
        e = int(spec.start[link_index]) + k
        # `rot @ v` and not `einsum`: einsum's BACKWARD dispatches through
        # triton, which cannot compile in this environment (no `cuda.h`) and
        # aborts the process rather than raising. Matmul is the same
        # arithmetic and does not.
        pos = pos + (rot @ origin_pos[e])
        rot = rot @ origin_rot[e]
        d = int(spec.dof[e])
        if d >= 0:
            # `cross` depends only on the joint AXIS, which is constant, so
            # it is built in numpy and uploaded rather than assembled from
            # scalar tensors. Not just tidier: `torch.stack` of 0-d tensors
            # on CUDA sends this through a JIT that cannot run in the motion
            # environment (no `cuda.h`), and it aborts the process rather
            # than raising. The only q-dependent operations left are sin,
            # cos and matmul.
            angle = q[:, d].reshape(batch, 1, 1)
            rot = rot @ (eye + torch.sin(angle) * cross_all[e]
                         + (1.0 - torch.cos(angle)) * cross2_all[e])
    return rot, pos


def reference(spec: ChainSpec, q, sphere_link, sphere_local):
    """The same answer in plain torch, as the parity target.

    Kept because a kernel that is only checked against itself is checked
    against nothing, and because a CPU field still needs a path.
    """
    import torch

    batch = q.shape[0]
    device, dtype = q.device, q.dtype
    origin_rot = torch.as_tensor(spec.origin_rot, device=device, dtype=dtype)
    origin_pos = torch.as_tensor(spec.origin_pos, device=device, dtype=dtype)

    # `repeat` and not `expand`: an expanded tensor has stride zero, and
    # feeding one into a chain of matmuls that autograd then differentiates
    # aborted the process here rather than raising. Materialising it costs a
    # few hundred bytes and this is the reference path, not the fast one.
    rot_by_link: list[torch.Tensor] = []
    pos_by_link: list[torch.Tensor] = []
    eye = torch.eye(3, device=device, dtype=dtype)
    for li in range(len(spec.link_names)):
        rot = eye.repeat(batch, 1, 1)
        pos = torch.zeros(batch, 3, device=device, dtype=dtype)
        for k in range(int(spec.count[li])):
            e = int(spec.start[li]) + k
            pos = pos + (rot @ origin_pos[e])
            rot = rot @ origin_rot[e]
            d = int(spec.dof[e])
            if d >= 0:
                k = _skew(spec.axis[e])
                cross = torch.as_tensor(k, device=device, dtype=dtype)
                cross2 = torch.as_tensor(k @ k, device=device, dtype=dtype)
                angle = q[:, d].reshape(batch, 1, 1)
                turn = eye + torch.sin(angle) * cross + (1.0 - torch.cos(angle)) * cross2
                rot = rot @ turn
        rot_by_link.append(rot)
        pos_by_link.append(pos)

    link_index = torch.as_tensor(sphere_link, device=device, dtype=torch.long)
    rot_all = torch.stack(rot_by_link, dim=1)  # (B, L, 3, 3)
    pos_all = torch.stack(pos_by_link, dim=1)  # (B, L, 3)
    rot = rot_all[:, link_index]               # (B, S, 3, 3)
    pos = pos_all[:, link_index]               # (B, S, 3)
    local = torch.as_tensor(sphere_local, device=device, dtype=dtype)
    return (rot @ local.unsqueeze(-1)).squeeze(-1) + pos


# --------------------------------------------------------------------------
# the kernel
# --------------------------------------------------------------------------

_kernel = None


def _make_kernel():
    import warp as wp

    @wp.func
    def _turn(axis: wp.vec3, angle: float) -> wp.mat33:
        return wp.quat_to_matrix(wp.quat_from_axis_angle(axis, angle))

    @wp.kernel
    def fk_links(
        q: wp.array2d(dtype=float),             # (B, D)
        start: wp.array(dtype=int),             # (L,)
        count: wp.array(dtype=int),             # (L,)
        origin_rot: wp.array(dtype=wp.mat33),   # (E,)
        origin_pos: wp.array(dtype=wp.vec3),    # (E,)
        axis: wp.array(dtype=wp.vec3),          # (E,)
        dof: wp.array(dtype=int),               # (E,)
        owner: wp.array(dtype=int),             # (E,)
        num_links: int,
        link_rot: wp.array2d(dtype=wp.mat33),   # (B, L)
        link_pos: wp.array2d(dtype=wp.vec3),    # (B, L)
        joint_axis: wp.array2d(dtype=wp.vec3),  # (B, D) world axis
        joint_moment: wp.array2d(dtype=wp.vec3),  # (B, D) axis x origin
    ):
        tid = wp.tid()
        b = tid / num_links
        li = tid - b * num_links
        base = start[li]
        n = count[li]

        rot = wp.identity(n=3, dtype=float)
        pos = wp.vec3(0.0, 0.0, 0.0)
        for k in range(n):
            e = base + k
            pos = pos + rot * origin_pos[e]
            rot = rot * origin_rot[e]
            d = dof[e]
            if d >= 0:
                # The joint frame is (rot, pos) right here, before its own
                # rotation is applied -- which is what makes `a x (x - o)`
                # the right column: the axis and the point it turns about.
                if owner[e] == 1:
                    a = rot * axis[e]
                    joint_axis[b, d] = a
                    joint_moment[b, d] = wp.cross(a, pos)
                rot = rot * _turn(axis[e], q[b, d])
        link_rot[b, li] = rot
        link_pos[b, li] = pos

    @wp.kernel
    def fk_single(
        q: wp.array2d(dtype=float),
        start: wp.array(dtype=int),
        count: wp.array(dtype=int),
        origin_rot: wp.array(dtype=wp.mat33),
        origin_pos: wp.array(dtype=wp.vec3),
        axis: wp.array(dtype=wp.vec3),
        dof: wp.array(dtype=int),
        write_link: wp.array(dtype=int),        # (S,) link this sphere reports, or -1
        write_dof: wp.array(dtype=int),         # (S,) dof this sphere reports, or -1
        sphere_link: wp.array(dtype=int),
        sphere_local: wp.array(dtype=wp.vec3),
        num_spheres: int,
        out: wp.array(dtype=wp.vec3),
        link_rot: wp.array2d(dtype=wp.mat33),
        link_pos: wp.array2d(dtype=wp.vec3),
        joint_axis: wp.array2d(dtype=wp.vec3),
        joint_moment: wp.array2d(dtype=wp.vec3),
    ):
        """Both stages in one launch, for when the launch is the cost.

        Below a few thousand poses the kernels take microseconds and a CUDA
        graph node costs about five, so two nodes lose to one even though
        this walks the chain per sphere instead of per link. Above that the
        redundant walking dominates and `fk_links` + `fk_spheres` wins. The
        crossover is measured, not guessed.

        The link transforms and joint frames still come out, but only from
        one designated thread per link -- letting all three hundred spheres
        write them was correct (every writer stores identical bits) and cost
        more than the walk it was riding along with.
        """
        tid = wp.tid()
        b = tid / num_spheres
        s = tid - b * num_spheres
        li = sphere_link[s]
        base = start[li]
        n = count[li]

        wl = write_link[s]
        wd = write_dof[s]
        rot = wp.identity(n=3, dtype=float)
        pos = wp.vec3(0.0, 0.0, 0.0)
        for k in range(n):
            e = base + k
            pos = pos + rot * origin_pos[e]
            rot = rot * origin_rot[e]
            d = dof[e]
            if d >= 0:
                if d == wd:
                    a = rot * axis[e]
                    joint_axis[b, d] = a
                    joint_moment[b, d] = wp.cross(a, pos)
                rot = rot * _turn(axis[e], q[b, d])
        if wl >= 0:
            link_rot[b, wl] = rot
            link_pos[b, wl] = pos
        out[tid] = pos + rot * sphere_local[s]

    @wp.kernel
    def fk_spheres(
        link_rot: wp.array2d(dtype=wp.mat33),
        link_pos: wp.array2d(dtype=wp.vec3),
        sphere_link: wp.array(dtype=int),       # (S,)
        sphere_local: wp.array(dtype=wp.vec3),  # (S,)
        num_spheres: int,
        out: wp.array(dtype=wp.vec3),           # (B*S,)
    ):
        tid = wp.tid()
        b = tid / num_spheres
        s = tid - b * num_spheres
        li = sphere_link[s]
        out[tid] = link_pos[b, li] + link_rot[b, li] * sphere_local[s]

    @wp.kernel
    def fk_backward(
        grad_out: wp.array(dtype=wp.vec3),      # (B*S,)
        point: wp.array(dtype=wp.vec3),         # (B*S,) the forward result
        joint_axis: wp.array2d(dtype=wp.vec3),
        joint_moment: wp.array2d(dtype=wp.vec3),
        jstart: wp.array(dtype=int),            # (L,)
        jcount: wp.array(dtype=int),            # (L,)
        jdof: wp.array(dtype=int),              # (K,)
        sphere_link: wp.array(dtype=int),
        num_spheres: int,
        per_thread: int,
        grad_q: wp.array2d(dtype=float),        # (B, D)
    ):
        tid = wp.tid()
        groups = (num_spheres + per_thread - 1) / per_thread
        b = tid / groups
        gidx = tid - b * groups
        s0 = gidx * per_thread

        # Fold `per_thread` spheres into registers before touching global
        # memory. One thread per sphere put six atomic adds per sphere onto
        # a handful of `grad_q` slots -- 29M of them at 16k poses -- and the
        # contention, not the arithmetic, was the cost.
        acc0 = float(0.0)
        acc1 = float(0.0)
        acc2 = float(0.0)
        acc3 = float(0.0)
        acc4 = float(0.0)
        acc5 = float(0.0)
        d0 = int(-1)
        d1 = int(-1)
        d2 = int(-1)
        d3 = int(-1)
        d4 = int(-1)
        d5 = int(-1)
        for t in range(per_thread):
            s = s0 + t
            if s < num_spheres:
                li = sphere_link[s]
                idx = b * num_spheres + s
                g = grad_out[idx]
                x = point[idx]
                base = jstart[li]
                n = jcount[li]
                for k in range(n):
                    d = jdof[base + k]
                    # a x (x - o) = a x x - a x o, and the second term is
                    # the same for every sphere on this pose. Keeping the
                    # axis and that moment per JOINT rather than a Jacobian
                    # per sphere is what removed the 177 MB buffer.
                    col = wp.cross(joint_axis[b, d], x) - joint_moment[b, d]
                    v = wp.dot(g, col)
                    # A chain is at most six deep on this arm, and `k` is
                    # its position, so the slot is known without a search.
                    if k == 0:
                        acc0 += v
                        d0 = d
                    elif k == 1:
                        acc1 += v
                        d1 = d
                    elif k == 2:
                        acc2 += v
                        d2 = d
                    elif k == 3:
                        acc3 += v
                        d3 = d
                    elif k == 4:
                        acc4 += v
                        d4 = d
                    else:
                        acc5 += v
                        d5 = d
        if d0 >= 0:
            wp.atomic_add(grad_q, b, d0, acc0)
        if d1 >= 0:
            wp.atomic_add(grad_q, b, d1, acc1)
        if d2 >= 0:
            wp.atomic_add(grad_q, b, d2, acc2)
        if d3 >= 0:
            wp.atomic_add(grad_q, b, d3, acc3)
        if d4 >= 0:
            wp.atomic_add(grad_q, b, d4, acc4)
        if d5 >= 0:
            wp.atomic_add(grad_q, b, d5, acc5)

    return fk_links, fk_spheres, fk_backward, fk_single


def _ensure_kernel():
    global _kernel
    if _kernel is None:
        import warp as wp

        wp.init()
        _kernel = _make_kernel()
    return _kernel


class _FusedFK:
    """Uploaded chain plus reusable buffers, so a hot loop allocates nothing.

    cuRobo pre-allocates its forward and backward buffers for exactly this
    reason (`KinematicsFusedFunction.create_buffers`). Allocation inside the
    optimizer's inner loop is a synchronisation point wearing a disguise.
    """

    def __init__(self, spec: ChainSpec, sphere_link, sphere_local, device, graph: bool = True):
        import torch
        import warp as wp

        _ensure_kernel()
        self.spec = spec
        # `torch.device("cuda")` carries index None, and warp indexes its
        # device list with it. Resolve to the current device rather than
        # making every caller remember to write "cuda:0".
        if device.type == "cuda" and device.index is None:
            device = torch.device("cuda", torch.cuda.current_device())
        self.device = device
        self.warp_device = wp.device_from_torch(device)

        def up(array, dtype):
            return wp.array(np.ascontiguousarray(array), dtype=dtype, device=self.warp_device)

        self.start = up(spec.start, wp.int32)
        self.count = up(spec.count, wp.int32)
        self.origin_rot = up(spec.origin_rot, wp.mat33)
        self.origin_pos = up(spec.origin_pos, wp.vec3)
        self.axis = up(spec.axis, wp.vec3)
        self.dof = up(spec.dof, wp.int32)
        self.owner = up(spec.owner.astype(np.int32), wp.int32)
        self.jstart = up(spec.jstart, wp.int32)
        self.jcount = up(spec.jcount, wp.int32)
        self.jdof = up(spec.jdof, wp.int32)
        self.num_links = len(spec.link_names)
        self.sphere_link_np = np.asarray(sphere_link, dtype=np.int32)
        self.sphere_link = up(self.sphere_link_np, wp.int32)
        self.sphere_local = up(np.asarray(sphere_local, dtype=np.float32), wp.vec3)
        self.num_spheres = len(self.sphere_link_np)
        # One designated reporter per link, and per degree of freedom: the
        # first sphere on the link that owns it. Falls back to the two-stage
        # path if any link or axis has no reporter, which is how a tree that
        # branches away from the sphere set stays correct.
        write_link = np.full(self.num_spheres, -1, dtype=np.int32)
        write_dof = np.full(self.num_spheres, -1, dtype=np.int32)
        seen_link: set[int] = set()
        for si, li in enumerate(self.sphere_link_np):
            if int(li) not in seen_link:
                seen_link.add(int(li))
                write_link[si] = int(li)
        owner_link: dict[int, int] = {}
        for li in range(self.num_links):
            for k in range(int(spec.count[li])):
                d = int(spec.dof[int(spec.start[li]) + k])
                if d >= 0 and d not in owner_link:
                    owner_link[d] = li
        for d, li in owner_link.items():
            for si, sl in enumerate(self.sphere_link_np):
                if int(sl) == li:
                    write_dof[si] = d
                    break
        self.single_ok = (
            len(seen_link) == self.num_links and len(owner_link) == spec.num_dof
        )
        self.write_link = up(write_link, wp.int32)
        self.write_dof = up(write_dof, wp.int32)
        # Each backward thread folds several spheres before touching global
        # memory. One thread per sphere meant six atomic adds per sphere all
        # landing on the same handful of `grad_q` slots -- 29M of them at
        # 16k poses, into 98k slots.
        self.use_graph = graph
        # Measured on a 5090 with 300 spheres: one launch wins while the
        # graph node dominates, two once the redundant chain walk does. The
        # crossover sits around a hundred thousand sphere-updates.
        self.single_kernel_limit = 150_000
        self.spheres_per_thread = 8
        self.spheres_per_thread_dim = (
            self.num_spheres + self.spheres_per_thread - 1
        ) // self.spheres_per_thread
        self._buffers: dict[int, tuple] = {}
        self._torch = torch

    def buffers(self, batch: int):
        """Per-batch-size scratch, allocated once and captured into a graph.

        No Jacobian here. The first version kept one per sphere -- 177 MB at
        16k poses, written in forward and read back in backward -- for six
        numbers reconstructible from the joint's axis and moment, which are
        per POSE and fit in a few kilobytes.

        The warp array wrappers are built once too. `wp.from_torch` is not
        free: it validates and constructs a Python object per call, and at
        four arrays a launch it was a measurable share of a kernel that
        takes tens of microseconds.
        """
        import warp as wp

        torch = self._torch
        if batch in self._buffers:
            return self._buffers[batch]

        n = batch * self.num_spheres
        dof = self.spec.num_dof
        links = self.num_links
        dev = self.device
        f32 = torch.float32
        t = {
            "q": torch.zeros(batch, dof, dtype=f32, device=dev),
            "out": torch.empty(n, 3, dtype=f32, device=dev),
            "lrot": torch.empty(batch, links, 3, 3, dtype=f32, device=dev),
            "lpos": torch.empty(batch, links, 3, dtype=f32, device=dev),
            "jaxis": torch.empty(batch, dof, 3, dtype=f32, device=dev),
            "jmom": torch.empty(batch, dof, 3, dtype=f32, device=dev),
            "gout": torch.zeros(n, 3, dtype=f32, device=dev),
            "gq": torch.zeros(batch, dof, dtype=f32, device=dev),
        }
        w = {
            "q": wp.from_torch(t["q"]),
            "out": wp.from_torch(t["out"], dtype=wp.vec3),
            "lrot": wp.from_torch(t["lrot"], dtype=wp.mat33),
            "lpos": wp.from_torch(t["lpos"], dtype=wp.vec3),
            "jaxis": wp.from_torch(t["jaxis"], dtype=wp.vec3),
            "jmom": wp.from_torch(t["jmom"], dtype=wp.vec3),
            "gout": wp.from_torch(t["gout"], dtype=wp.vec3),
            "gq": wp.from_torch(t["gq"]),
        }

        fk_links, fk_spheres, fk_backward, fk_single = _ensure_kernel()

        # One launch or two. Below the crossover the graph node is the cost
        # and the redundant chain walk is free; above it that reverses.
        single = self.single_ok and batch * self.num_spheres <= self.single_kernel_limit

        record_stream = None

        def target():
            # Torch's CURRENT stream, per call and not per closure: the point
            # is that these launches follow whoever is running them -- a side
            # stream, or a CUDA graph being captured -- rather than warp's
            # default. See `utils.device.warp_stream`. While warp is recording
            # its OWN graphs below, the launches go to the stream being
            # recorded instead: warp's capture only accepts a stream it owns.
            if record_stream is not None:
                return {"stream": record_stream}
            stream = warp_stream(self.device)
            return ({"stream": stream} if stream is not None
                    else {"device": self.warp_device})

        def launch_forward_single():
            wp.launch(
                fk_single,
                dim=batch * self.num_spheres,
                inputs=[w["q"], self.start, self.count, self.origin_rot, self.origin_pos,
                        self.axis, self.dof, self.write_link, self.write_dof,
                        self.sphere_link, self.sphere_local, self.num_spheres,
                        w["out"], w["lrot"], w["lpos"], w["jaxis"], w["jmom"]],
                **target(),
            )

        def launch_forward_two():
            wp.launch(
                fk_links,
                dim=batch * self.num_links,
                inputs=[w["q"], self.start, self.count, self.origin_rot, self.origin_pos,
                        self.axis, self.dof, self.owner, self.num_links,
                        w["lrot"], w["lpos"], w["jaxis"], w["jmom"]],
                **target(),
            )
            wp.launch(
                fk_spheres,
                dim=batch * self.num_spheres,
                inputs=[w["lrot"], w["lpos"], self.sphere_link, self.sphere_local,
                        self.num_spheres, w["out"]],
                **target(),
            )

        launch_forward = launch_forward_single if single else launch_forward_two

        def launch_backward():
            # The TORCH tensor, not the warp wrapper: `w["gq"].zero_()` is a
            # warp memset on warp's own stream, which a torch CUDA graph
            # capture cannot record -- the replayed backward then never
            # zeroes, and the atomic adds return exact multiples of the true
            # gradient, one per replay. Measured before this line changed.
            t["gq"].zero_()
            wp.launch(
                fk_backward,
                dim=batch * self.spheres_per_thread_dim,
                inputs=[w["gout"], w["out"], w["jaxis"], w["jmom"],
                        self.jstart, self.jcount, self.jdof, self.sphere_link,
                        self.num_spheres, self.spheres_per_thread, w["gq"]],
                **target(),
            )

        # Capture both into CUDA graphs. Two launches plus a zero-fill are
        # three trips through the driver; replaying a graph is one, and at
        # this kernel size the driver is most of the cost -- batch 1 and
        # batch 16384 took the same 0.08 ms before this, which is the
        # signature of an overhead-bound launch rather than a busy GPU.
        graphs = None
        if self.use_graph and self.device.type == "cuda":
            try:
                # A warp-OWNED stream: warp's capture refuses a wrapped torch
                # stream ("unknown capture stream"), and the default stream
                # cannot capture at all. `target()` routes the launches onto
                # it while `record_stream` is set. The torch `zero_()` inside
                # `launch_backward` cannot join a warp recording, so the
                # replay path zeroes `gq` itself before every launch.
                record_stream = wp.Stream(self.warp_device)
                with wp.ScopedCapture(stream=record_stream) as cap_f:
                    launch_forward()
                with wp.ScopedCapture(stream=record_stream) as cap_b:
                    launch_backward()
                graphs = (cap_f.graph, cap_b.graph)
            except Exception:  # noqa: BLE001 - capture is an optimisation
                graphs = None
            finally:
                record_stream = None

        entry = (t, launch_forward, launch_backward, graphs)
        self._buffers[batch] = entry
        return entry

def make(cell: dict[str, Any], links: list[str], model_links, device, graph: bool = True):
    """A callable `q -> (B, S, 3)` world sphere centres, differentiable.

    `model_links` maps link name to its list of `(centre, radius)`, as
    `spheres.SphereModel.links` gives it.

    `graph` captures the launches into a CUDA graph, which is where most of
    the speed is at these kernel sizes. Pass False to measure the eager path
    or to debug -- a captured graph replays against fixed addresses, so a
    stray allocation inside it fails loudly rather than quietly.

    THE RESULT ALIASES AN INTERNAL BUFFER. It is valid until the next call
    at the same batch size, which is what lets a hot loop allocate nothing.
    Clone it if you need two results alive at once. cuRobo's buffers behave
    the same way for the same reason.
    """
    import torch
    import warp as wp

    spec = build(cell, links)
    order = {name: i for i, name in enumerate(links)}
    sphere_link: list[int] = []
    sphere_local: list[tuple[float, float, float]] = []
    radii: list[float] = []
    for name in links:
        for centre, radius in model_links.get(name, []):
            sphere_link.append(order[name])
            sphere_local.append(centre)
            radii.append(radius)
    if not sphere_link:
        raise FKError("none of these links carry spheres")

    state = _FusedFK(spec, sphere_link, sphere_local, device, graph=graph)
    _ensure_kernel()

    class Fused(torch.autograd.Function):
        @staticmethod
        def forward(ctx, q: torch.Tensor) -> torch.Tensor:
            batch = q.shape[0]
            t, launch_forward, _lb, graphs = state.buffers(batch)
            # Copy into the captured buffer rather than wrapping the caller's
            # tensor: a graph replays against fixed addresses, so the input
            # has to live at one.
            t["q"].copy_(q.detach())
            # Replayed on torch's CURRENT stream: a graph launched on warp's
            # default would run unordered against the copy above on a side
            # stream. And not replayed at all while an OUTER graph is being
            # captured -- a graph launch cannot be recorded into a capturing
            # stream, so the eager launches stand in and get recorded
            # individually instead.
            if graphs is not None and not torch.cuda.is_current_stream_capturing():
                wp.capture_launch(graphs[0], stream=warp_stream(state.device))
            else:
                launch_forward()
            ctx.batch = batch
            return t["out"].reshape(batch, state.num_spheres, 3)

        @staticmethod
        def backward(ctx, grad_output: torch.Tensor):
            batch = ctx.batch
            t, _lf, launch_backward, graphs = state.buffers(batch)
            t["gout"].copy_(grad_output.reshape(-1, 3))
            if graphs is not None and not torch.cuda.is_current_stream_capturing():
                # The zero lives OUTSIDE the recorded graph -- it is a torch
                # op, and warp's recording cannot hold it -- so it runs here,
                # or the atomic adds accumulate onto the last call's answer.
                t["gq"].zero_()
                wp.capture_launch(graphs[1], stream=warp_stream(state.device))
            else:
                launch_backward()
            return t["gq"].clone()

    def link_poses(q: torch.Tensor):
        """World rotation and translation per link, as (B, L, 3, 3), (B, L, 3).

        Free: stage one already computes and stores them, and the collision
        stage is not their only consumer -- the tool pose and the seam frames
        come out of the same walk.
        """
        Fused.apply(q)
        t, _lf, _lb, _g = state.buffers(q.shape[0])
        return t["lrot"], t["lpos"]

    radii_t = torch.tensor(radii, dtype=torch.float32, device=device)
    def fn(q: torch.Tensor) -> torch.Tensor:
        return Fused.apply(q)

    # `Fused.apply` is a bound method and will not take an attribute, so the
    # callable handed back is a plain function with the link poses attached
    # to it -- one object for the two things a caller wants from a walk.
    def last_link_poses(batch: int):
        """The link poses the LAST call at this batch size left in the
        buffers -- no launch. Valid until the next call at that batch size,
        like every result here; for a caller that has just asked for the
        spheres of the same `q` and wants the frames the same walk produced
        without paying for the walk again (the swept term's workpiece frame
        was a second, torch chain walk per descent step before this)."""
        t, _lf, _lb, _g = state.buffers(batch)
        return t["lrot"], t["lpos"]

    fn.link_poses = link_poses  # type: ignore[attr-defined]
    fn.last_link_poses = last_link_poses  # type: ignore[attr-defined]
    return fn, spec, radii_t, np.asarray(sphere_link, dtype=np.int32), np.asarray(sphere_local, dtype=np.float32)


def self_test() -> int:
    """Parity against the torch reference, and the gradient against autograd."""
    import torch

    if not torch.cuda.is_available():
        print("fk: no CUDA; the fused path is a GPU path, nothing to check")
        return 0

    device = torch.device("cuda")
    # A three-link chain with perpendicular axes: enough that a wrong axis
    # or a wrong compose order cannot hide.
    cell = {
        "frames": {"base": "base", "tool": "c"},
        "links": [{"name": n} for n in ("base", "a", "b", "c")],
        "joints": [
            {"name": "J1", "parent": "base", "child": "a", "type": "revolute",
             "xyz_m": [0.0, 0.0, 0.20], "rpy_rad": [0.0, 0.0, 0.0], "axis_xyz": [0, 0, 1]},
            {"name": "J2", "parent": "a", "child": "b", "type": "revolute",
             "xyz_m": [0.10, 0.0, 0.05], "rpy_rad": [0.0, 0.3, 0.0], "axis_xyz": [0, 1, 0]},
            {"name": "J3", "parent": "b", "child": "c", "type": "fixed",
             "xyz_m": [0.0, 0.04, 0.30], "rpy_rad": [0.1, 0.0, 0.0]},
        ],
    }
    links = ["a", "b", "c"]
    model = {
        "a": [((0.01, 0.0, 0.02), 0.05), ((0.0, 0.03, 0.0), 0.04)],
        "b": [((0.0, 0.0, 0.10), 0.06)],
        "c": [((0.02, 0.0, 0.0), 0.03), ((0.0, 0.0, 0.05), 0.02)],
    }
    fused, spec, radii, sphere_link, sphere_local = make(cell, links, model, device)

    torch.manual_seed(7)
    q = (torch.rand(5, spec.num_dof, device=device) - 0.5) * 2.0

    # The reference runs on the CPU in float64. Two reasons, and both are
    # about it being a REFERENCE: double precision makes it the arbiter
    # rather than a second opinion at the same precision, and the CPU path
    # keeps this check working in an environment where torch cannot reach
    # triton -- which is this one, where the CUDA headers are absent and the
    # first GPU autograd op that wants a JIT dies compiling `cuda.h`.
    q_cpu = q.detach().double().cpu()
    got = fused(q)
    want = reference(spec, q_cpu, sphere_link, sphere_local)
    err = float((got.detach().double().cpu() - want).abs().max())
    print(f"fk: value vs reference max {err * 1000:.6f} mm over {got.numel() // 3} spheres")
    if err > 1e-5:
        print("fk: FAIL, the fused kernel and the reference disagree")
        return 1

    # Gradient: autograd through the reference is the target, since the
    # reference is plain torch and torch differentiates it for us.
    weight = torch.randn_like(got)
    weight_cpu = weight.detach().double().cpu()
    qa = q.clone().requires_grad_(True)
    (fused(qa) * weight).sum().backward()
    qb = q_cpu.clone().requires_grad_(True)
    (reference(spec, qb, sphere_link, sphere_local) * weight_cpu).sum().backward()
    scale = max(float(qb.grad.abs().max()), 1e-9)
    gerr = float((qa.grad.double().cpu() - qb.grad).abs().max()) / scale
    print(f"fk: gradient vs autograd, worst relative {gerr:.2e}")
    if gerr > 1e-4:
        print("fk: FAIL, the analytic Jacobian disagrees with autograd")
        return 1

    print("fk: ok")
    return 0


def main(argv: list[str]) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"fk: {exc}", file=sys.stderr)
        raise SystemExit(1)
