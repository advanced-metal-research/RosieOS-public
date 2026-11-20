#!/usr/bin/env python
"""How fast the tip can move in a given direction, per unit of joint speed.

## Why not a manipulability number

The obvious index is Yoshikawa's `w = sqrt(det(J J^T))`, and it is the wrong
one here. It measures the VOLUME of the velocity ellipsoid, so it stays
healthy while the ellipsoid goes paper-thin in one direction -- and a weld
only ever needs one direction at a time. Over 4096 poses of this very arm
`self_test` finds one where `w` is **125% of its median** while mobility along
`+x` is **16% of its own**. A screen on `w` waves that posture through as
better than typical.

So the index is DIRECTIONAL:

    mu_d(q) = 1 / || J^+ d ||_2

the largest Cartesian speed along the unit direction `d` that one unit of
joint speed can buy. Its reciprocal is the more useful object -- `J^+ d` is
the minimum-norm joint velocity that achieves unit Cartesian speed along `d`,
so the PER-JOINT form

    | [J^+ d]_i | * v  <=  qdot_max_i

is a hard constraint with no norm hiding inside it, and it is strictly
stronger than the scalar version. `WEAVE-AND-VERIFICATION.md` section 4 argues
both; this module computes both.

## What this is for now that weave is out

That document introduces the index as `mu_b'`, along the weave's transverse
axis `b'`, to keep an arm weave-capable. **Weave is out of scope**
(`ARCHITECTURE.md`), and this module does not reinstate it.

What survives the removal is the constraint the WELD ITSELF imposes. A seam is
run at a programmed travel speed along the tangent `t`, and near a singularity
in that direction the joints have to slew hard to deliver a modest tip speed.
That is a real infeasibility, it belongs to the centreline rather than to the
oscillation, and it is the same arithmetic with `d = t`. So the direction is
an ARGUMENT here, not a constant: pass `t` for travel, and pass `b'` too on
the day someone takes weave back on.

## Numerics

Via SVD of the 3xn linear block rather than an explicit pseudoinverse.
`J^+ = J^T (J J^T)^-1` needs a 3x3 inverse that blows up exactly where the
answer matters, and the blow-up is the signal -- `mu -> 0` at a singularity is
correct, `nan` is not. With `J = U S V^T`,

    || J^+ d ||^2 = sum_i (u_i . d)^2 / s_i^2

which degrades smoothly: a direction ORTHOGONAL to the collapsing singular
vector keeps its mobility, which is the whole point of a directional index and
the thing `det` cannot express.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from weld_motion_planner.robot_cell.robot.fk import ChainSpec, FKError, _chain_tensors

# Below this, a singular value is reported as collapsed rather than divided by.
# 1e-9 m/rad is nine orders below this arm's reach; anything at that scale is
# a singularity and not a small number.
SINGULAR_FLOOR = 1.0e-9


class MobilityError(ValueError):
    """A chain or a direction this refuses, with the reason."""


#: `(id(spec), device, dtype) -> (spec, axis)` -- the spec reference keeps the
#: id stable, exactly `fk._UPLOADED`'s pattern and for `fk._chain_tensors`'
#: reason: `torch.as_tensor` on a numpy array is a host-to-device copy, per
#: call it was a synchronise in the middle of the descent step, and inside a
#: CUDA graph capture it is an error outright ("Cannot copy between CPU and
#: CUDA tensors during CUDA graph capture") -- the failure that silently sent
#: the weld trajopt back to the eager loop.
_AXIS_UPLOADED: dict = {}


def _axis_tensor(spec: ChainSpec, device, dtype) -> torch.Tensor:
    key = (id(spec), str(device), dtype)
    hit = _AXIS_UPLOADED.get(key)
    if hit is None:
        hit = (spec, torch.as_tensor(np.asarray(spec.axis),
                                     device=device, dtype=dtype))
        _AXIS_UPLOADED[key] = hit
    return hit[1]


def jacobian(spec: ChainSpec, link_index: int, q: torch.Tensor, *,
             tip_local: Any = None) -> torch.Tensor:
    """The geometric Jacobian at a link's tip. `(B, 6, dof)`, differentiable.

    Built from the same chain walk `torch_link_transform` does, capturing each
    revolute joint's world axis `a_j` and origin `o_j` on the way past. The
    columns are then the textbook ones:

        linear[:, j]  = a_j x (p_tip - o_j)
        angular[:, j] = a_j

    which is worth stating because it is the SAME identity `fk.py`'s backward
    kernel uses -- `dp/dq_j = a_j x (p - o_j)` -- read as a matrix instead of
    as a gradient. Columns for joints not on this link's chain stay zero:
    a wrist joint does not move the shoulder, and the pseudoinverse must see
    that as "cannot help" rather than as "no data".
    """
    batch = q.shape[0]
    device, dtype = q.device, q.dtype
    origin_rot, origin_pos, cross_all, cross2_all, eye = _chain_tensors(spec, device, dtype)

    rot = eye.repeat(batch, 1, 1)
    pos = torch.zeros(batch, 3, device=device, dtype=dtype)
    axes: list[torch.Tensor] = []
    origins: list[torch.Tensor] = []
    columns: list[int] = []

    axis_const = _axis_tensor(spec, device, dtype)
    for k in range(int(spec.count[link_index])):
        e = int(spec.start[link_index]) + k
        pos = pos + (rot @ origin_pos[e])
        rot = rot @ origin_rot[e]
        d = int(spec.dof[e])
        if d >= 0:
            # Captured HERE: after the fixed origin and before the joint's own
            # rotation, which is the frame the axis is expressed in.
            axes.append(rot @ axis_const[e])
            origins.append(pos)
            columns.append(d)
            angle = q[:, d].reshape(batch, 1, 1)
            rot = rot @ (eye + torch.sin(angle) * cross_all[e]
                         + (1.0 - torch.cos(angle)) * cross2_all[e])

    tip = pos
    if tip_local is not None:
        offset = torch.as_tensor(np.asarray(tip_local, dtype=np.float64),
                                 device=device, dtype=dtype)
        tip = pos + (rot @ offset)

    if not axes:
        return torch.zeros(batch, 6, spec.num_dof, device=device, dtype=dtype)

    # Stacked once and scattered once, rather than two indexed writes per
    # joint -- 2*K fewer kernel launches per call, and this is called sixty
    # times per IK solve.
    #
    # Worth 5 ms of an 87 ms solve at batch 256 and no more, which is recorded
    # because it was expected to be worth much more. That solve costs the same
    # 82 ms at batch 256 as at batch 2048, so it is paying the DRIVER and not
    # the GPU -- and what remains is the two chain walks per iteration, not
    # these writes. The fix for that is graph capture, as `fk.py` already
    # does; it is named in `BENCHMARKS.md` and not built here.
    axis_stack = torch.stack(axes, dim=-1)                       # (B, 3, K)
    origin_stack = torch.stack(origins, dim=-1)                  # (B, 3, K)
    linear = torch.linalg.cross(axis_stack, tip.unsqueeze(-1) - origin_stack, dim=1)
    packed = torch.cat([linear, axis_stack], dim=1)              # (B, 6, K)

    if len(columns) == spec.num_dof and columns == list(range(spec.num_dof)):
        return packed
    jac = torch.zeros(batch, 6, spec.num_dof, device=device, dtype=dtype)
    # Cached like `_axis_tensor`, and for its reason: `columns` is a function
    # of `(spec, link_index)` alone, and uploading it per call was a
    # host-to-device copy inside the captured step.
    key = (id(spec), link_index, str(device))
    hit = _AXIS_UPLOADED.get(key)
    if hit is None:
        hit = (spec, torch.as_tensor(columns, device=device, dtype=torch.long))
        _AXIS_UPLOADED[key] = hit
    return jac.index_copy(2, hit[1], packed)


def tip(spec: ChainSpec, link_index: int, q: torch.Tensor, *,
        tip_local: Any = None) -> torch.Tensor:
    """Where the Jacobian was taken, so a caller can check it agrees with FK."""
    from weld_motion_planner.robot_cell.robot.fk import torch_link_transform

    rot, pos = torch_link_transform(spec, link_index, q)
    if tip_local is None:
        return pos
    offset = torch.as_tensor(np.asarray(tip_local, dtype=np.float64),
                             device=q.device, dtype=q.dtype)
    return pos + (rot @ offset)


@dataclass(frozen=True)
class Mobility:
    """What one direction costs, per pose."""

    mu: torch.Tensor              # (B,) m/s of tip per unit ||qdot||
    joint_velocity: torch.Tensor  # (B, dof) SIGNED qdot for 1 m/s along d
    joint_rates: torch.Tensor     # (B, dof) its magnitude, which is what limits bound
    singular: torch.Tensor        # (B,) bool: the direction is unreachable
    # (B,) the ellipsoid's volume in the subspace the arm can actually reach:
    # the product of the singular values ABOVE the floor. Not over all three,
    # because an arm that cannot leave a plane has a zero singular value at
    # every pose, and `prod` would then report zero manipulability everywhere
    # and make the comparison this module exists to draw impossible to state.
    yoshikawa: torch.Tensor

    def summary(self) -> dict[str, Any]:
        return {
            "mu_min": round(float(self.mu.min()), 6),
            "mu_median": round(float(self.mu.median()), 6),
            "singular": int(self.singular.sum()),
            "worst_joint_rate": round(float(self.joint_rates.max()), 4),
        }


def directional(jac: torch.Tensor, direction: torch.Tensor) -> Mobility:
    """`mu_d` and the per-joint rates, for a unit direction per pose.

    `direction` is `(B, 3)` in the WORLD frame -- the caller maps the seam
    tangent through the workpiece placement, because which frame `t` lives in
    is a fact about the program and not about the arm.
    """
    if jac.ndim != 3 or jac.shape[1] != 6:
        raise MobilityError(f"expected a (B, 6, dof) Jacobian, got {tuple(jac.shape)}")
    if direction.shape[:1] != jac.shape[:1] or direction.shape[-1] != 3:
        raise MobilityError(
            f"direction must be (B, 3) to match the batch, got {tuple(direction.shape)}")

    unit = direction / direction.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    linear = jac[:, :3, :]                                   # (B, 3, dof)
    u, s, vh = torch.linalg.svd(linear, full_matrices=False)  # u (B,3,3), s (B,3)

    # (u_i . d) / s_i, with the collapsed directions held out rather than
    # divided by. `torch.where` on BOTH branches: a nan in the discarded
    # branch still propagates through the backward pass.
    alive = s > SINGULAR_FLOOR
    proj = torch.einsum("bij,bj->bi", u.transpose(1, 2), unit)  # (B, 3)
    safe = torch.where(alive, s, torch.ones_like(s))
    contrib = torch.where(alive, proj / safe, torch.zeros_like(proj))

    # A direction with weight on a collapsed singular vector is not slow, it
    # is unreachable: no joint velocity produces it at all.
    lost = torch.where(alive, torch.zeros_like(proj), proj.abs())
    singular = lost.max(dim=-1).values > 1e-6

    norm = contrib.norm(dim=-1)
    mu = torch.where(singular, torch.zeros_like(norm), 1.0 / norm.clamp_min(1e-12))

    # The minimum-norm joint velocity for 1 m/s along d, per joint. Returned
    # SIGNED as well as absolute: a caller that wants to check `J qdot == d`
    # -- the one test that catches a transpose or a sign, which every
    # norm-based check passes -- cannot do it from magnitudes.
    qdot = torch.einsum("bji,bj->bi", vh, contrib)  # V @ contrib
    volume = torch.where(alive, s, torch.ones_like(s)).prod(dim=-1)
    return Mobility(mu=mu, joint_velocity=qdot, joint_rates=qdot.abs(),
                    singular=singular, yoshikawa=volume)


def travel_margin(mob: Mobility, travel_speed_m_s: torch.Tensor,
                  qdot_max: torch.Tensor | None) -> dict[str, torch.Tensor]:
    """Whether the programmed travel speed is actually deliverable.

    The scalar form is `travel_speed / mu <= ||qdot_max||`, and the per-joint
    form is `rate_i * travel_speed <= qdot_max_i`. The second is stricter and
    is the one reported as the verdict; the first is what a cost term would
    descend on.

    `qdot_max` of `None` is not a pass. It comes back as an all-false
    `checked` mask, and the caller is expected to carry that into the
    verdict's `unverifiable` list the way `verify.check_limits` does -- this
    cell's profile has no rate limits at all.
    """
    speed = torch.as_tensor(travel_speed_m_s, device=mob.mu.device, dtype=mob.mu.dtype)
    needed = mob.joint_rates * speed.reshape(-1, 1)
    out: dict[str, torch.Tensor] = {"needed_rad_per_s": needed}
    if qdot_max is None:
        out["checked"] = torch.zeros_like(mob.mu, dtype=torch.bool)
        out["feasible"] = torch.zeros_like(mob.mu, dtype=torch.bool)
        out["headroom"] = torch.full_like(mob.mu, float("nan"))
        return out
    limits = torch.as_tensor(qdot_max, device=mob.mu.device, dtype=mob.mu.dtype)
    ratio = needed / limits.clamp_min(1e-12)
    out["checked"] = torch.ones_like(mob.mu, dtype=torch.bool)
    out["feasible"] = (~mob.singular) & (ratio.max(dim=-1).values <= 1.0)
    # 1.0 is "exactly at the limit", 0.0 is "the limit is already broken".
    out["headroom"] = 1.0 - ratio.max(dim=-1).values
    return out


# --------------------------------------------------------------------------


def self_test() -> int:
    if not torch.cuda.is_available():
        print("mobility: no CUDA; this path is batched torch on device")
        return 0
    from pathlib import Path

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.robot import fk

    device = torch.device("cuda", torch.cuda.current_device())
    root = Path(__file__).resolve().parents[4] / "data" / "motion"
    cell = read(root / "bracket_b_2x.weldplan").cell
    tool = str((cell.get("frames") or {}).get("tool") or "")
    spec = fk.build(cell, [tool])
    link = spec.link_names.index(tool)
    print(f"mobility: {tool}, {spec.num_dof} axes {spec.axis_names}")

    # 1. The Jacobian against finite differences of the FK it claims to
    #    differentiate. This is the whole module; if the columns are wrong
    #    every number below is decoration.
    torch.manual_seed(0)
    batch = 64
    q = torch.rand(batch, spec.num_dof, device=device, dtype=torch.float64) * 2.0 - 1.0
    jac = jacobian(spec, link, q)
    eps = 1e-6
    worst = 0.0
    for j in range(spec.num_dof):
        bump = torch.zeros_like(q)
        bump[:, j] = eps
        fd = (tip(spec, link, q + bump) - tip(spec, link, q - bump)) / (2 * eps)
        worst = max(worst, float((fd - jac[:, :3, j]).abs().max()))
    print(f"  jacobian vs finite differences: worst {worst:.3e} m/rad")
    if worst > 1e-6:
        print("mobility: FAIL, the Jacobian does not differentiate the kinematics")
        return 1

    # 2. The argument the module opens with, as a measurement rather than a
    #    claim: find a pose where Yoshikawa looks fine and the direction that
    #    matters does not.
    q = torch.rand(4096, spec.num_dof, device=device, dtype=torch.float64) * 3.0 - 1.5
    jac = jacobian(spec, link, q)
    direction = torch.zeros(len(q), 3, device=device, dtype=torch.float64)
    direction[:, 0] = 1.0
    mob = directional(jac, direction)
    w_med, mu_med = mob.yoshikawa.median(), mob.mu.median()
    healthy = mob.yoshikawa > 0.8 * w_med
    if not bool(healthy.any()):
        print("mobility: FAIL, no pose in the sample has a healthy Yoshikawa index")
        return 1
    worst_at = int(mob.mu.masked_fill(~healthy, float("inf")).argmin())
    ratio_w = float(mob.yoshikawa[worst_at] / w_med)
    ratio_mu = float(mob.mu[worst_at] / mu_med)
    print(f"  over {len(q)} poses, along +x: mu median {float(mu_med):.4f} m per rad")
    print(f"    the worst pose whose Yoshikawa is healthy: "
          f"w = {ratio_w * 100:.0f}% of median, mu = {ratio_mu * 100:.1f}% of median")
    if ratio_mu > 0.5:
        print("mobility: FAIL, the sample found no posture where the two disagree; "
              "the directional index is not earning its place here")
        return 1

    # 3. The per-joint form must be stricter than the scalar one, and the
    #    comparison has to be made where the limit actually BITES. A welding
    #    speed against a 3.14 rad/s limit leaves two orders of headroom, and
    #    at that setting both forms accept everything and agree perfectly
    #    while proving nothing. So the speed is swept until the boundary is
    #    somewhere inside the sample.
    limits = torch.full((spec.num_dof,), 3.14, device=device, dtype=torch.float64)
    print(f"  {'tip speed':>10}  {'scalar ok':>10}  {'per-joint ok':>13}  stricter by")
    stricter_anywhere = 0
    for speed_m_s in (0.008, 0.5, 1.0, 2.0, 4.0):
        speed = torch.full((len(q),), speed_m_s, device=device, dtype=torch.float64)
        margin = travel_margin(mob, speed, limits)
        scalar_ok = (speed / mob.mu.clamp_min(1e-12)) <= float(limits.norm())
        if bool((margin["feasible"] & ~scalar_ok).any()):
            print("mobility: FAIL, a pose passed the per-joint form and failed the "
                  "scalar one; the per-joint form is supposed to be the stricter")
            return 1
        stricter = int((scalar_ok & ~margin["feasible"]).sum())
        stricter_anywhere = max(stricter_anywhere, stricter)
        print(f"  {speed_m_s * 1000:7.0f}mm/s  {int(scalar_ok.sum()):10d}  "
              f"{int(margin['feasible'].sum()):13d}  {stricter:11d}")
    if stricter_anywhere == 0:
        print("mobility: FAIL, the per-joint form never refused anything the scalar "
              "form accepted, at any speed -- the comparison is vacuous")
        return 1

    # 4. Missing limits are not a pass.
    absent = travel_margin(mob, speed, None)
    if bool(absent["feasible"].any()) or bool(absent["checked"].any()):
        print("mobility: FAIL, a missing rate limit was reported as feasible")
        return 1
    print(f"  with no rate limits: checked {int(absent['checked'].sum())}, "
          f"feasible {int(absent['feasible'].sum())}")

    print("mobility: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (MobilityError, FKError) as exc:
        print(f"mobility: {exc}", file=sys.stderr)
        raise SystemExit(1)
