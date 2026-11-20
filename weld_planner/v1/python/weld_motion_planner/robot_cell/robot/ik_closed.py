#!/usr/bin/env python
"""Inverse kinematics without iterating, for arms shaped like this one.

`ik.py` solves by damped least squares: about thirty-five iterations, each with
a Jacobian, a six-by-six linear solve and two chain walks. On a
wrist-partitioned elbow arm none of that is necessary. The answer is
trigonometry, all eight postures come out at once, and there is no such thing
as "failed to converge" -- either the geometry admits a solution or it does
not, and the difference is the arccosine of something outside `[-1, 1]` rather
than an iteration budget running out.

## Why it is exact here, and every premise is CHECKED

Three facts about the arm, measured in `form_of` rather than assumed, because
an arm that broke any of them would get a confident wrong answer from this file
and a correct one from the iterative solver:

- **The last three axes meet at a point.** Then the wrist centre depends on the
  first three joints alone, which splits one six-dimensional problem into two
  three-dimensional ones.
- **The second and third axes are parallel and perpendicular to the first.**
  Then placing the wrist centre is a two-link planar problem in the plane those
  two axes turn in.
- **The side offsets are known.** `J2` sits 125.5 mm to one side of the first
  axis and the wrist centre 125.5 mm back, so they nearly cancel and `J1` is
  nearly an `atan2`. NEARLY is not a closed form. This arm's two numbers are
  `-0.125500489` and `+0.125499999`, which miss by 490 nm, and a solver that
  assumes they cancel is solving an idealised arm 490 nm away from this one --
  which is exactly the error it then shows against forward kinematics. The
  offset is carried instead: `J1 = phi - asin(c / r)` with `c` the leftover and
  `r` the wrist centre's distance from the first axis, which reduces to the
  `atan2` when `c` is zero and is exact when it is not.

## The eight

    branch = shoulder * 4 + elbow * 2 + flip

    shoulder   two values of J1, a turn apart
    elbow      two roots of the planar triangle
    flip       two of the wrist's Euler decomposition

The same eight the iterative solver finds from eight structured seeds -- but
labelled BY CONSTRUCTION rather than by which basin a seed happened to fall
into. `docs/issues/0002` is the request for exactly that.

## Parallelism

Better than the iterative path, not worse. No loop, no linear solve, no
data-dependent iteration count: one elementwise pass of trigonometry over the
whole batch with the eight branches as a leading dimension. The iterative
solver's `torch.linalg.solve` is a batched LU factorisation and the least
GPU-shaped thing in the pipeline; there is nothing like it here.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

# The premises, as tolerances. Loose enough for a chain assembled in float32,
# tight enough that a genuinely different arm misses by orders of magnitude.
AXIS_TOL = 1e-6
# The leftover side offset is CARRIED rather than bounded, so there is no
# tolerance on it any more. What remains is the geometric fact that the wrist
# centre cannot be closer to the first axis than that offset -- a target inside
# that cylinder has no solution, and the `asin` says so by going out of range.
CANCEL_TOL_M = 2.0e-6   # kept for callers that still report it

BRANCHES = 8
CANONICAL_AXES = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))


class ClosedFormError(ValueError):
    """An arm this cannot solve in closed form, with the reason."""


@dataclass
class ArmForm:
    """What the closed form needs, all of it measured off the cell.

    The elbow's plane is carried as complex numbers: a turn of `t` about the
    shared second/third axis acts on that plane as multiplication by
    `exp(-i t)`, which makes the two-link problem one line of algebra instead
    of a matrix chain.
    """

    base_xyz: np.ndarray         # first joint's origin, in the base frame
    shoulder: complex            # first origin to second, in the elbow's plane
    upper_m: float               # second origin to third
    fore: complex                # third origin to the wrist centre, same plane
    wrist_in_tool: np.ndarray    # (3,) the wrist centre in the tool's own frame
    tool_in_six: np.ndarray      # (3, 3) the tool's rotation on the sixth link
    cancel_error_m: float        # the leftover side offset, carried as a term

    @property
    def fore_m(self) -> float:
        return float(abs(self.fore))


def form_of(cell: dict[str, Any], axis_names: list[str],
            tool_rot0: np.ndarray, tool_pos0: np.ndarray) -> ArmForm | None:
    """Measure the closed form, or `None` if this arm does not have one.

    `None` and not an exception: the iterative solver answers every arm, so one
    this cannot describe should be slower rather than refused. `tool_rot0` and
    `tool_pos0` are the tool frame at the ZERO posture, which is how the tool's
    mounting is learned without this module knowing how many fixed joints lie
    between the sixth link and the torch.
    """
    joints = {str(j["name"]): j for j in (cell.get("joints") or [])}
    if len(axis_names) != 6 or any(n not in joints for n in axis_names):
        return None
    named = [joints[n] for n in axis_names]
    if any(np.abs(np.asarray(j.get("rpy_rad") or (0, 0, 0), float)).max() > AXIS_TOL
           for j in named):
        return None
    axes = [np.asarray(j["axis_xyz"], float) for j in named]
    axes = [a / np.linalg.norm(a) for a in axes]
    if any(np.abs(a - np.asarray(w, float)).max() > AXIS_TOL
           for a, w in zip(axes, CANONICAL_AXES)):
        return None

    origin = np.cumsum(np.stack([np.asarray(j["xyz_m"], float) for j in named]), axis=0)

    # The wrist centre is the point on all three of the last axes. Solved for
    # and then CHECKED to be on each, so an arm whose wrist merely nearly meets
    # falls through to the iterative solver instead of getting a wrong answer.
    a_mat, b_vec = np.zeros((3, 3)), np.zeros(3)
    for i in (3, 4, 5):
        block = np.eye(3) - np.outer(axes[i], axes[i])
        a_mat += block
        b_vec += block @ origin[i]
    try:
        wrist = np.linalg.solve(a_mat, b_vec)
    except np.linalg.LinAlgError:
        return None
    for i in (3, 4, 5):
        block = np.eye(3) - np.outer(axes[i], axes[i])
        if np.linalg.norm(block @ (wrist - origin[i])) > 1e-6:
            return None

    shared = axes[1]
    shoulder = origin[1] - origin[0]
    fore = wrist - origin[2]
    # The leftover, which used to be a premise and is now a term. Nothing is
    # refused for it: an arm with a real shoulder offset is solved by the same
    # algebra, and one with none gets `asin(0)` and the plain `atan2`.
    cancel = float(shoulder @ shared + fore @ shared)

    plane = lambda v: complex(float(v[0]), float(v[2]))   # noqa: E731 -- XZ, since the shared axis is Y
    return ArmForm(
        base_xyz=origin[0],
        shoulder=plane(shoulder),
        upper_m=float(np.linalg.norm(origin[2] - origin[1])),
        fore=plane(fore),
        wrist_in_tool=tool_rot0.T @ (wrist - tool_pos0),
        tool_in_six=tool_rot0,       # the sixth link's rotation is identity at zero
        cancel_error_m=cancel,
    )


def _rot_y(angle: torch.Tensor) -> torch.Tensor:
    cos, sin, zero, one = (torch.cos(angle), torch.sin(angle),
                           torch.zeros_like(angle), torch.ones_like(angle))
    return torch.stack([torch.stack([cos, zero, sin], -1),
                        torch.stack([zero, one, zero], -1),
                        torch.stack([-sin, zero, cos], -1)], -2)


def _rot_z(angle: torch.Tensor) -> torch.Tensor:
    cos, sin, zero, one = (torch.cos(angle), torch.sin(angle),
                           torch.zeros_like(angle), torch.ones_like(angle))
    return torch.stack([torch.stack([cos, -sin, zero], -1),
                        torch.stack([sin, cos, zero], -1),
                        torch.stack([zero, zero, one], -1)], -2)


def solve(form: ArmForm, target_rot: torch.Tensor, target_pos: torch.Tensor
          ) -> tuple[torch.Tensor, torch.Tensor]:
    """`(8, N, 6)` joint values and `(8, N)` bool for which of them exist.

    Branch order is `shoulder * 4 + elbow * 2 + flip`, so the index names the
    posture family. A branch that does not exist carries whatever the
    arithmetic left there and must be read through the mask.
    """
    device, dtype = target_pos.device, target_pos.dtype
    as_t = lambda v: torch.as_tensor(np.asarray(v, np.float64), device=device, dtype=dtype)  # noqa: E731

    centre = target_pos + (target_rot @ as_t(form.wrist_in_tool))
    rel = centre - as_t(form.base_xyz)

    # --- the shoulder ------------------------------------------------------
    # After the first turn the wrist centre must sit `cancel` to the side, so
    #     -u_x sin(t) + u_y cos(t) = cancel,  i.e.  r sin(phi - t) = cancel.
    # Two roots, and they are `phi` and `phi - pi` when the offset vanishes.
    # A wrist centre closer to the axis than the offset has neither, which the
    # `asin` reports by leaving its range -- and that is a real unreachable
    # cylinder around the shoulder, not a numerical accident.
    radius = torch.sqrt(rel[:, 0] * rel[:, 0] + rel[:, 1] * rel[:, 1])
    phi = torch.atan2(rel[:, 1], rel[:, 0])
    ratio = torch.as_tensor(form.cancel_error_m, device=device, dtype=dtype) / radius.clamp_min(1e-12)
    reachable = ratio.abs() <= 1.0
    lean = torch.arcsin(ratio.clamp(-1.0, 1.0))
    first = torch.stack([phi - lean, phi - np.pi + lean], dim=0)       # (2, N)

    # The wrist centre with the first turn undone, in the elbow's plane.
    x = torch.cos(first) * rel[:, 0] + torch.sin(first) * rel[:, 1]
    z = rel[:, 2].unsqueeze(0).expand_as(x)
    zx = x - float(form.shoulder.real)
    zz = z - float(form.shoulder.imag)

    # --- the elbow: the planar triangle ------------------------------------
    upper, fore_len, fore_arg = form.upper_m, form.fore_m, float(np.angle(form.fore))
    span = torch.sqrt(zx * zx + zz * zz)
    cosine = (span * span - upper * upper - fore_len * fore_len) / (2.0 * upper * fore_len)
    exists = (cosine.abs() <= 1.0) & reachable.unsqueeze(0)
    inner = torch.arccos(cosine.clamp(-1.0, 1.0))
    third = torch.stack([inner, -inner], dim=0) - (np.pi / 2 - fore_arg)   # (2, 2, N)

    # The pair of links has to point where the wrist centre is: the second turn
    # is the difference between the direction the pair makes on its own and the
    # direction it must end up in.
    made = torch.atan2(upper + fore_len * torch.sin(-third + fore_arg),
                       fore_len * torch.cos(-third + fore_arg))
    second = made - torch.atan2(zz, zx).unsqueeze(0)

    first = first.unsqueeze(0).expand_as(third)
    exists = exists.unsqueeze(0).expand_as(third)

    # --- the wrist: R36 = R03^T (R_target R_tool^T), as XYZ Euler ----------
    # `R03 = Rz(t1) Ry(t2) Ry(t3) = Rz(t1) Ry(t2 + t3)`, which is why the two
    # elbow joints never need separate matrices.
    six = target_rot @ as_t(form.tool_in_six).T                         # (N, 3, 3)
    wrist_rot = (_rot_y(-(second + third))
                 @ _rot_z(-first)
                 @ six.reshape(1, 1, -1, 3, 3))                         # (2, 2, N, 3, 3)
    fifth = torch.arcsin(wrist_rot[..., 0, 2].clamp(-1.0, 1.0))
    fifth = torch.stack([fifth, np.pi - fifth], dim=0)                  # (2, 2, 2, N)
    cos_fifth = torch.cos(fifth)
    safe = cos_fifth.abs() > 1e-9
    guard = torch.where(safe, cos_fifth, torch.ones_like(cos_fifth))
    rot = wrist_rot.unsqueeze(0)
    fourth = torch.atan2(-rot[..., 1, 2] / guard, rot[..., 2, 2] / guard)
    sixth = torch.atan2(-rot[..., 0, 1] / guard, rot[..., 0, 0] / guard)

    shape = fifth.shape                                                 # (2, 2, 2, N)
    grow = lambda v: v.unsqueeze(0).expand(shape)                       # noqa: E731
    stacked = torch.stack([grow(first), grow(second), grow(third),
                           fourth, fifth, sixth], dim=-1)
    # (flip, elbow, shoulder, N, 6) -> branch = shoulder*4 + elbow*2 + flip
    stacked = stacked.permute(2, 1, 0, 3, 4).reshape(BRANCHES, -1, 6)
    live = (grow(exists) & safe).permute(2, 1, 0, 3).reshape(BRANCHES, -1)
    return stacked, live


def self_test() -> int:
    print("ik_closed: checked against the iterative solver in "
          "tests/motion_planner/test_ik_closed.py")
    return 0


def main(argv: list[str]) -> int:
    argparse.ArgumentParser(description=__doc__.splitlines()[0]).parse_args(argv)
    return self_test()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
