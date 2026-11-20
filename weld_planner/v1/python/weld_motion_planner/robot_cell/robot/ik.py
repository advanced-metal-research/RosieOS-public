#!/usr/bin/env python
"""Given a pose for the torch, what joint angles put it there.

The redundancy graph is built out of these: one solve per node, hundreds of
thousands of them per seam, so this is the bulk of the graph's cost and it is
written to be one batched tensor computation rather than a loop over poses.

## The method

Damped least squares, iterated. At a guess `q`, the difference between where
the torch is and where it should be is a six-number error `e` -- three of
position, three of rotation. The Jacobian `J` says how that error moves when
each joint moves. The step is

    dq = J^T (J J^T + lambda^2 I)^-1 e

Undamped (`lambda = 0`) this is the plain Newton step and it is the fastest
thing there is until the arm is near a posture where some direction costs
almost nothing to want and almost everything to deliver -- there the inverse
blows up and the step flies off. The damping bounds it: `lambda` trades exact
descent for a bounded step, and since a solve that overshoots is a solve that
never converges, that trade is free here.

`lambda` is adapted per pose rather than fixed. A step that reduced the error
earns a smaller `lambda` (toward Newton, fast); one that made it worse is
undone and earns a larger one (toward gradient descent, safe). Per pose and
not per batch, because a batch contains poses at wildly different distances
from their targets and one shared `lambda` would be tuned for none of them.

## The rotation error

`R_err = R_target R_current^T` as an axis and an angle, packed as one vector
of length `angle` pointing along `axis`. The formula through `acos` of the
trace loses precision badly near zero angle -- which is exactly where a
converging solve spends most of its iterations -- so this uses `atan2` of the
skew part against the trace, which does not.

## What "converged" means

Position and orientation are checked SEPARATELY, against tolerances in metres
and radians. A single blended residual would let a solve trade 2 mm of
position for a degree of angle and call itself done, and those two are not
interchangeable in a weld.

Joint limits are enforced by clamping after every step, which makes the
answer feasible by construction. A pose only reachable outside the limits then
fails to converge, which is the correct verdict rather than a solution the
robot cannot execute.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from weld_motion_planner.robot_cell.robot.fk import ChainSpec, torch_link_transform
from weld_motion_planner.robot_cell.robot.mobility import jacobian

DEFAULT_ITERS = 60
DEFAULT_POS_TOL_M = 1.0e-4      # 0.1 mm, well under the weld tolerances above it
DEFAULT_ROT_TOL_RAD = 1.0e-3    # 0.057 deg
DEFAULT_LAMBDA = 0.05
# How often the solve drops converged poses out of its working set. Every step
# would spend more on gathers than it saves; never is what the loop did before,
# and it ground 52% of the batch for forty iterations after they were done.
RETIRE_EVERY = 5
LAMBDA_MIN = 1.0e-6
LAMBDA_MAX = 1.0e3


class IKError(ValueError):
    """A request this refuses to solve, with the reason."""


def rotation_error(current: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """`(B, 3)`: the rotation taking `current` to `target`, as axis times angle.

    Via `atan2` and not `acos(trace)`. The trace formula's derivative is
    infinite at zero angle, so it loses most of its significant digits exactly
    where an iterating solve lives; `atan2` of the skew magnitude against the
    trace is well conditioned across the whole range.
    """
    rel = target @ current.transpose(-1, -2)
    skew = torch.stack([
        rel[..., 2, 1] - rel[..., 1, 2],
        rel[..., 0, 2] - rel[..., 2, 0],
        rel[..., 1, 0] - rel[..., 0, 1],
    ], dim=-1)
    sin2 = torch.linalg.norm(skew, dim=-1)                    # 2 sin(angle)
    cos2 = rel[..., 0, 0] + rel[..., 1, 1] + rel[..., 2, 2] - 1.0  # 2 cos(angle)
    angle = torch.atan2(sin2, cos2)
    # Near zero the axis is `skew / sin2` with both going to zero together, so
    # the ratio is finite but computed as 0/0. `angle / sin2` is the same
    # limit and stays finite: at small angle it tends to 1/2.
    scale = torch.where(sin2 > 1e-12, angle / sin2.clamp_min(1e-30),
                        torch.full_like(sin2, 0.5))
    return skew * scale.unsqueeze(-1)


def pose_error(spec: ChainSpec, link_index: int, q: torch.Tensor,
               target_rot: torch.Tensor, target_pos: torch.Tensor, *,
               tip_local: Any = None) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """`(position error, rotation error, current rotation)` at `q`."""
    rot, pos = torch_link_transform(spec, link_index, q)
    if tip_local is not None:
        offset = torch.as_tensor(np.asarray(tip_local, dtype=np.float64),
                                 device=q.device, dtype=q.dtype)
        pos = pos + (rot @ offset)
    return target_pos - pos, rotation_error(rot, target_rot), rot


@dataclass
class Solution:
    """What came back, per pose."""

    q: torch.Tensor              # (B, dof)
    converged: torch.Tensor      # (B,) bool
    position_error_m: torch.Tensor   # (B,)
    rotation_error_rad: torch.Tensor  # (B,)
    iterations: int
    # (B,) bool: the solve ended pressed against a joint limit. Not a failure
    # on its own -- plenty of good welds are made near a stop -- but the graph
    # wants it, because a node at a limit has no room to be refined and its
    # edges are worth less.
    at_limit: torch.Tensor

    def summary(self) -> dict[str, Any]:
        n = int(self.q.shape[0])
        ok = self.converged
        return {
            "poses": n,
            "converged": int(ok.sum()),
            "rate": round(float(ok.float().mean()), 4),
            "worst_position_mm": round(float(self.position_error_m[ok].max()) * 1000.0, 4)
            if bool(ok.any()) else None,
            "worst_rotation_deg": round(float(torch.rad2deg(self.rotation_error_rad[ok].max())), 4)
            if bool(ok.any()) else None,
            "at_limit": int(self.at_limit.sum()),
            "iterations": self.iterations,
        }


def limits_of(cell: dict[str, Any], spec: ChainSpec, device, dtype
              ) -> tuple[torch.Tensor, torch.Tensor]:
    """Position limits in the chain's own axis order, refused if any is absent.

    Not defaulted to +-pi. An axis whose limit is missing would silently gain
    or lose travel, and every solve after that would be answering a question
    about a different robot.
    """
    bounds = {}
    for joint in cell.get("joints") or []:
        limit = joint.get("limits_rad") or {}
        if "lower" in limit and "upper" in limit:
            bounds[str(joint["name"])] = (float(limit["lower"]), float(limit["upper"]))
    missing = [n for n in spec.axis_names if n not in bounds]
    if missing:
        raise IKError(f"these axes have no position limit in the cell: {missing}")
    lo = torch.tensor([bounds[n][0] for n in spec.axis_names], device=device, dtype=dtype)
    hi = torch.tensor([bounds[n][1] for n in spec.axis_names], device=device, dtype=dtype)
    return lo, hi


def solve(spec: ChainSpec, link_index: int, target_rot: torch.Tensor,
          target_pos: torch.Tensor, seed: torch.Tensor, *,
          lower: torch.Tensor | None = None, upper: torch.Tensor | None = None,
          tip_local: Any = None,
          iters: int = DEFAULT_ITERS,
          pos_tol_m: float = DEFAULT_POS_TOL_M,
          rot_tol_rad: float = DEFAULT_ROT_TOL_RAD) -> Solution:
    """Batched damped least squares. One pose per row, all rows at once."""
    if target_rot.shape[0] != target_pos.shape[0] or target_rot.shape[0] != seed.shape[0]:
        raise IKError("targets and seeds must agree on the batch size")
    if seed.shape[1] != spec.num_dof:
        raise IKError(f"seed has {seed.shape[1]} axes, this chain has {spec.num_dof}")

    q = seed.clone()
    device, dtype = q.device, q.dtype
    batch = q.shape[0]
    lam = torch.full((batch,), DEFAULT_LAMBDA, device=device, dtype=dtype)
    eye = torch.eye(6, device=device, dtype=dtype)

    # THE WORKING SET SHRINKS AS POSES FINISH.
    #
    # The loop used to stop only when EVERY pose had converged, which on a real
    # lattice never happens: about 45% of poses are unreachable and iterate
    # forever by definition, so all of them ran the full budget. Measured on
    # `dog_box` seam 1 at 32x32 bins, 32768 poses x 8 branches: 52.3% converged
    # by iteration 20 and then kept being ground for another 40.
    #
    # A converged pose's answer cannot change, so dropping it is exact -- the
    # same argument the search uses to compact away infeasible nodes. `live`
    # indexes back into the caller's batch and every per-pose array below is
    # compact; results are written out as rows retire.
    live = torch.arange(batch, device=device)
    tgt_rot, tgt_pos = target_rot, target_pos
    out_q = seed.clone()
    out_dp = torch.zeros(batch, 3, device=device, dtype=dtype)
    out_dr = torch.zeros(batch, 3, device=device, dtype=dtype)

    def residual(qq):
        dp, dr, _ = pose_error(spec, link_index, qq, tgt_rot, tgt_pos,
                               tip_local=tip_local)
        return dp, dr

    dp, dr = residual(q)
    # The scalar the damping adapts on. Position in metres and rotation in
    # radians are not the same unit, and this sum is a HEURISTIC for "did the
    # step help" -- never a convergence test, which is why the two are
    # checked separately below.
    score = dp.pow(2).sum(-1) + dr.pow(2).sum(-1)

    done = 0
    for step in range(iters):
        done = step + 1
        error = torch.cat([dp, dr], dim=-1).unsqueeze(-1)            # (B, 6, 1)
        jac = jacobian(spec, link_index, q, tip_local=tip_local)     # (B, 6, dof)
        jjt = jac @ jac.transpose(1, 2)
        damped = jjt + (lam.pow(2).reshape(-1, 1, 1) * eye)
        step_q = (jac.transpose(1, 2) @ torch.linalg.solve(damped, error)).squeeze(-1)

        trial = q + step_q
        if lower is not None:
            trial = torch.clamp(trial, lower, upper)
        tp, tr = residual(trial)
        trial_score = tp.pow(2).sum(-1) + tr.pow(2).sum(-1)

        # Per pose, not per batch: a batch holds poses at very different
        # distances from their targets and one shared damping would suit none.
        better = trial_score < score
        keep = better.unsqueeze(-1)
        q = torch.where(keep, trial, q)
        dp = torch.where(keep, tp, dp)
        dr = torch.where(keep, tr, dr)
        score = torch.where(better, trial_score, score)
        lam = torch.where(better, (lam * 0.5).clamp_min(LAMBDA_MIN),
                          (lam * 3.0).clamp_max(LAMBDA_MAX))

        arrived = ((torch.linalg.norm(dp, dim=-1) < pos_tol_m)
                   & (torch.linalg.norm(dr, dim=-1) < rot_tol_rad))
        if bool(arrived.all()):
            out_q[live] = q
            out_dp[live] = dp
            out_dr[live] = dr
            live = live[:0]
            break
        # Retiring costs a handful of gathers, so it is not worth doing every
        # step for the one or two poses that finished. `RETIRE_EVERY` is small
        # enough that the freshly converged do not ride along for long and large
        # enough that the gathers stay noise.
        if bool(arrived.any()) and (step + 1) % RETIRE_EVERY == 0:
            keep = ~arrived
            out_q[live[arrived]] = q[arrived]
            out_dp[live[arrived]] = dp[arrived]
            out_dr[live[arrived]] = dr[arrived]
            live = live[keep]
            q, dp, dr = q[keep], dp[keep], dr[keep]
            score, lam = score[keep], lam[keep]
            tgt_rot, tgt_pos = tgt_rot[keep], tgt_pos[keep]

    if len(live):
        out_q[live] = q
        out_dp[live] = dp
        out_dr[live] = dr
    q, dp, dr = out_q, out_dp, out_dr
    pos_norm = torch.linalg.norm(dp, dim=-1)
    rot_norm = torch.linalg.norm(dr, dim=-1)
    at_limit = torch.zeros(batch, dtype=torch.bool, device=device)
    if lower is not None:
        span = (upper - lower).clamp_min(1e-9)
        at_limit = (((q - lower) / span < 1e-6) | ((upper - q) / span < 1e-6)).any(dim=-1)
    return Solution(
        q=q,
        converged=(pos_norm < pos_tol_m) & (rot_norm < rot_tol_rad),
        position_error_m=pos_norm,
        rotation_error_rad=rot_norm,
        iterations=done,
        at_limit=at_limit,
    )


# --------------------------------------------------------------------------
# Seeds


def branch_seeds(q: torch.Tensor) -> torch.Tensor:
    """The eight postures a wrist-partitioned 6R arm can hold one pose in.

    `(B, dof) -> (8, B, dof)`. Shoulder forward or reversed, elbow up or down,
    wrist flipped or not. These are seeds and not answers -- the
    transformations are exact only for a textbook arm with no offsets, and
    this one has them -- but each lands in the basin of a genuinely different
    solution, which is all a seed has to do.

    Why not random restarts: at eight structured seeds the branches are found
    reliably and REPRODUCIBLY, and `ARCHITECTURE.md` asks the whole graph to
    be deterministic from a seed. Random restarts would need far more tries to
    cover the same eight basins and would not give the same answer twice.
    """
    if q.shape[1] < 6:
        raise IKError("branch seeds assume a 6-axis arm; this chain is shorter")
    pi = float(np.pi)
    out = []
    for shoulder in (0, 1):
        for elbow in (0, 1):
            for wrist in (0, 1):
                v = q.clone()
                if shoulder:
                    v[:, 0] = v[:, 0] + pi
                    v[:, 1] = -v[:, 1]
                    v[:, 2] = -v[:, 2]
                if elbow:
                    v[:, 2] = -v[:, 2]
                    v[:, 1] = v[:, 1] + 0.6   # nudge off the shared start
                if wrist:
                    v[:, 3] = v[:, 3] + pi
                    v[:, 4] = -v[:, 4]
                    v[:, 5] = v[:, 5] + pi
                out.append(v)
    return torch.stack(out, dim=0)


def random_seeds(lower: torch.Tensor, upper: torch.Tensor, count: int, batch: int,
                 *, generator: torch.Generator | None = None) -> torch.Tensor:
    """`(count, batch, dof)` uniform within the joint limits.

    Measured against `branch_seeds` on 2048 poses of this arm, both find about
    95% of solutions at eight seeds -- the structured branches are not better
    per seed, they are REPRODUCIBLE, which is what `ARCHITECTURE.md` asks of
    the graph. Random restarts are for closing the last few percent, where
    more tries is the only lever: 16 seeds reach 99.2% and 24 reach 99.7%.
    """
    dof = lower.shape[0]
    span = (upper - lower)
    draw = torch.rand(count, batch, dof, device=lower.device, dtype=lower.dtype,
                      generator=generator)
    return draw * span + lower


def solve_seeds(spec: ChainSpec, link_index: int, target_rot: torch.Tensor,
                target_pos: torch.Tensor, seeds: torch.Tensor, **kwargs) -> list[Solution]:
    """One solve per seed, all seeds and all poses in ONE batched call.

    `seeds` is `(K, B, dof)`. Returns K `Solution`s in seed order rather than
    a deduplicated set: which distinct postures exist is the GRAPH's question,
    and it answers it by joint distance between layers rather than by identity
    within one.
    """
    if seeds.ndim != 3:
        raise IKError(f"seeds must be (K, batch, dof), got {tuple(seeds.shape)}")
    branches, batch, dof = seeds.shape
    flat = solve(spec, link_index,
                 target_rot.repeat(branches, 1, 1),
                 target_pos.repeat(branches, 1),
                 seeds.reshape(branches * batch, dof), **kwargs)
    return [
        Solution(
            q=flat.q[i * batch:(i + 1) * batch],
            converged=flat.converged[i * batch:(i + 1) * batch],
            position_error_m=flat.position_error_m[i * batch:(i + 1) * batch],
            rotation_error_rad=flat.rotation_error_rad[i * batch:(i + 1) * batch],
            iterations=flat.iterations,
            at_limit=flat.at_limit[i * batch:(i + 1) * batch],
        )
        for i in range(branches)
    ]


def solve_multi(spec: ChainSpec, link_index: int, target_rot: torch.Tensor,
                target_pos: torch.Tensor, seed: torch.Tensor, **kwargs) -> list[Solution]:
    """`solve_seeds` over the eight structured branches. The default."""
    return solve_seeds(spec, link_index, target_rot, target_pos,
                       branch_seeds(seed), **kwargs)


def best_of(solutions: list[Solution]) -> Solution:
    """Collapse a seed sweep to one answer per pose: the first that converged.

    First and not closest-to-anything, so the result does not depend on which
    order the seeds happened to be generated in beyond the order given.
    """
    if not solutions:
        raise IKError("no solutions to choose from")
    q = solutions[0].q.clone()
    ok = solutions[0].converged.clone()
    pos = solutions[0].position_error_m.clone()
    rot = solutions[0].rotation_error_rad.clone()
    lim = solutions[0].at_limit.clone()
    for other in solutions[1:]:
        take = other.converged & ~ok
        q = torch.where(take.unsqueeze(-1), other.q, q)
        pos = torch.where(take, other.position_error_m, pos)
        rot = torch.where(take, other.rotation_error_rad, rot)
        lim = torch.where(take, other.at_limit, lim)
        ok = ok | other.converged
    return Solution(q=q, converged=ok, position_error_m=pos, rotation_error_rad=rot,
                    iterations=solutions[0].iterations, at_limit=lim)


def distinct(solutions: list[Solution], *, tol_rad: float = 0.1) -> torch.Tensor:
    """`(B,)` how many genuinely different postures were found per pose.

    Counted by joint distance, which is the same measure the graph's edges
    use -- two solutions closer than the edge threshold are one node's worth
    of choice, however differently they were seeded.
    """
    stack = torch.stack([s.q for s in solutions], dim=0)          # (K, B, dof)
    ok = torch.stack([s.converged for s in solutions], dim=0)     # (K, B)
    k = stack.shape[0]
    counts = torch.zeros(stack.shape[1], dtype=torch.long, device=stack.device)
    for i in range(k):
        fresh = ok[i].clone()
        for j in range(i):
            same = (stack[i] - stack[j]).abs().max(dim=-1).values < tol_rad
            fresh &= ~(same & ok[j])
        counts += fresh.long()
    return counts


# --------------------------------------------------------------------------


def self_test() -> int:
    if not torch.cuda.is_available():
        print("ik: no CUDA; this path is batched torch on device")
        return 0
    from pathlib import Path

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.robot import fk

    device = torch.device("cuda", torch.cuda.current_device())
    root = Path(__file__).resolve().parents[4] / "data" / "motion"
    cell = read(root / "bracket_b_2x.weldplan").cell
    tool = str(cell["frames"]["tool"])
    spec = fk.build(cell, [tool])
    link = spec.link_names.index(tool)
    lower, upper = limits_of(cell, spec, device, torch.float64)
    print(f"ik: {tool}, {spec.num_dof} axes, limits "
          f"{[round(float(v), 2) for v in lower]} .. {[round(float(v), 2) for v in upper]}")

    # Targets that are reachable BY CONSTRUCTION: take random joint values,
    # run them forward, and ask for the pose that came out. A target invented
    # in Cartesian space would mix "the solver failed" with "no arm could do
    # that", and the first is the only thing under test here.
    torch.manual_seed(0)
    batch = 2048
    truth = (torch.rand(batch, spec.num_dof, device=device, dtype=torch.float64)
             * (upper - lower) + lower)
    target_rot, target_pos = torch_link_transform(spec, link, truth)

    seed = torch.zeros_like(truth)
    seed[:, 1] = -0.8
    seed[:, 2] = 0.8
    import time

    torch.cuda.synchronize()
    t0 = time.perf_counter()
    single = solve(spec, link, target_rot, target_pos, seed, lower=lower, upper=upper)
    torch.cuda.synchronize()
    ms = (time.perf_counter() - t0) * 1000.0
    print(f"  one seed:     {single.summary()}")
    print(f"                {ms:.1f} ms for {batch} poses, {ms / batch * 1000:.1f} us each")

    t0 = time.perf_counter()
    many = solve_multi(spec, link, target_rot, target_pos, seed, lower=lower, upper=upper)
    torch.cuda.synchronize()
    ms = (time.perf_counter() - t0) * 1000.0
    any_ok = torch.stack([s.converged for s in many], dim=0).any(dim=0)
    counts = distinct(many)
    print(f"  eight seeds:  {int(any_ok.sum())}/{batch} poses solved by at least one "
          f"({float(any_ok.float().mean()) * 100:.1f}%)")
    print(f"                {ms:.1f} ms, distinct postures per pose: "
          f"mean {float(counts.float().mean()):.2f}, max {int(counts.max())}")

    if float(any_ok.float().mean()) < 0.95:
        print("ik: FAIL, poses generated FROM the arm's own kinematics should be "
              "solvable; a miss here is the solver's and not the geometry's")
        return 1
    if float(counts.float().mean()) < 1.5:
        print("ik: FAIL, the branch seeds are collapsing to one posture -- the graph "
              "would have no islands to choose between")
        return 1

    # Every returned solution must actually be a solution. Checked by running
    # the answer forward, not by trusting the residual the loop carried.
    for label, sol in (("one seed", single), ("branch 0", many[0])):
        ok = sol.converged
        if not bool(ok.any()):
            continue
        rot, pos = torch_link_transform(spec, link, sol.q[ok])
        dp = torch.linalg.norm(target_pos[ok] - pos, dim=-1).max()
        dr = torch.linalg.norm(rotation_error(rot, target_rot[ok]), dim=-1).max()
        print(f"  {label} re-checked forward: {float(dp) * 1000:.5f} mm, "
              f"{float(torch.rad2deg(dr)):.5f} deg")
        if float(dp) > DEFAULT_POS_TOL_M or float(dr) > DEFAULT_ROT_TOL_RAD:
            print("ik: FAIL, a solution reported converged does not reproduce its target")
            return 1

    # Limits are enforced, not suggested.
    for sol in (single, *many):
        if bool((sol.q < lower - 1e-9).any()) or bool((sol.q > upper + 1e-9).any()):
            print("ik: FAIL, a solution left the joint limits")
            return 1
    print("  every solution is inside the joint limits")

    print("ik: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except IKError as exc:
        print(f"ik: {exc}", file=sys.stderr)
        raise SystemExit(1)
