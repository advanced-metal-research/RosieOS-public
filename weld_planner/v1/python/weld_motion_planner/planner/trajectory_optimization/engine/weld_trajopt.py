#!/usr/bin/env python
"""M5, first cut: a discrete candidate in, a continuous trajectory out.

`docs/motion/IMPLEMENTATION.md` M5 is the spec this implements; the shape in
one paragraph: the variables are the KNOTS -- one per lattice sample, arm
axes plus positioner -- and the loop is Adam inside an augmented-Lagrangian
outer, with the constraints each handled by their own mechanism. Tracking is
PROJECTED (DLS Newton onto the seam after every step, spin untouched),
boxes and rates are CLIPPED, collision is the AL. The cost is cuRobo's
smoothness stack -- velocity, acceleration, jerk squared, each normalised by
its own limit so the axes are comparable -- plus a mid-interval tracking
quadrature as insurance against what the knots cannot see.

The output carries knot VELOCITIES too: the tip-velocity projection
`q̇ = q̇_fd + J⁺(ṗ_des − J·q̇_fd)` makes the pair `(q, q̇)` cubic-Hermite
data, which is what drops between-knot tracking error two orders below the
position-only spline (`IMPLEMENTATION.md` has the arithmetic).

## What this first cut deliberately defers, and why it is safe to

- **Serial, one trajectory at a time** -- the K candidates and the seams are
  loops, not all of them in one tensor. Correctness first; the batching is a
  reshape, not a redesign.
- **Angle attraction**: no committed plan request authors deviation bands yet,
  so the term would run against nothing. The clip hooks are where it slots.
- **The SQP polish** is its own task.

Two things that WERE on that list and are not any more.

The certified tracking bound: every trajectory carries a `TrackingCertificate`
from `verify.certify_tracking`, which bounds the worst tip-to-seam error over
the whole continuum rather than at chosen instants. Read that one and not
`midpoint_tracking_m` when the question is whether a seam is weldable -- the
midpoint number is the optimizer's own quadrature, and on the corner seams it
understates the truth by up to 4.75x.

The K-candidate re-ranking: `weld_trajopt_main.WeldTrajoptModule` works
every way M4 found across a seam and picks by THIS stage's objective, because
M4 ranks by accumulated L1 joint motion and the two rulers disagree. Measured
on `seam_0005`, they disagree completely -- the sweep's own winner scores last
of five here. Only the winner is certified; the losers travel with it as
numbers so the choice can be argued with.

The stage -- the `PlannerModule`, the `plan_*` entry points and the CLI --
lives in `weld_trajopt_main.py`, one level up.

Run: `pixi run -e motion python -m weld_motion_planner.planner.trajectory_optimization.weld_trajopt_main <request.weldplan>`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry.seam_frame import Seam, apply_angles
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.planner.dp_seam_search.engine import nodes
from weld_motion_planner.planner.trajectory_optimization.engine.control_solver import (
    INTERPOLATION_STEPS,
    SWEEP_STEPS,  # noqa: F401 -- re-exported: `weld_trajopt_main` builds the cost from HERE
    WIRE_DT_S,
    WIRE_MIN_INTERVALS,
)
from weld_motion_planner.robot_cell.collision.cost import CollisionCost
from weld_motion_planner.robot_cell.collision.scene import placement_matrix
from weld_motion_planner.robot_cell.robot import fk
from weld_motion_planner.robot_cell.robot.ik import limits_of, pose_error
from weld_motion_planner.robot_cell.robot.mobility import jacobian
from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.verifier.hermite_wire import hermite_wire_degrees
from weld_motion_planner.verifier.verify import (
    DEFAULT_VERIFY_MARGIN_M,
    SeamTarget,
    TrackingCertificate,
    Verdict,
    certify_tracking,
    check_limits,
    replace_limit_findings,
    verify,
)
from weld_motion_planner.verifier.verify import (
    kinematics as verify_kinematics,
)

# cuRobo's smoothness ratios (velocity : acceleration : jerk = 1000 : 10000 :
# 5), renormalised to velocity = 1. The terms themselves are dimensionless --
# each derivative is divided by its own limit before squaring -- so these
# ratios are comparing like with like.
W_VEL = 1.0
W_ACC = 10.0
W_JERK = 0.005

# How many times `_fit_rate_set` may pull knot velocities toward their chords
# before giving up. The curve is what the controller runs and what T2 walks, and
# straightening a segment costs tracking, so this is small on purpose: measured
# on the corpus, an offending trajectory settles in one or two rounds and the
# budget only bounds a pathological one.
RATE_FIT_ROUNDS = 8
# Interior tracking: per knot span, the MEAN over its `INTERPOLATION_STEPS - 1`
# interior grid samples of (error / tolerance)^2, so the weight means the same
# thing at every grid -- at two steps a span it is exactly the midpoint term
# this loop carried for its first year.
W_TRACK_MID = 10.0

# The cost grid, the swept collision read and the wire are FREE SPACE's
# numbers, imported rather than restated so the two stages judge and emit at
# one resolution (`control_solver` carries the measurement behind each):
#
# - `INTERPOLATION_STEPS`: samples of the Hermite each knot span contributes
#   to the grid the collision term and the tracking quadrature are judged on.
#   The knots here are the lattice's own samples, `nodes.DEFAULT_STEP_S`
#   apart, so at 4 the grid reads the curve every 62.5 ms.
# - `SWEEP_STEPS`: cap on the chord walk between consecutive grid samples
#   (`scene._make_sweep_kernel`) -- what a wall between two samples costs
#   the optimizer instead of the verifier.
# - `WIRE_DT_S` / `WIRE_MIN_INTERVALS`: spacing of the emitted `(q, qdot, t)`,
#   the same Hermite re-evaluated at the controller's clock (`resample_wire`).

# The T2 verifier's own safety pad, restated here so the weld pass declares which
# number it is being judged against rather than inheriting one. It is a
# POLICY, not a measurement: `verify.py` set it because the cell meshes are
# the visual meshes and the fixtures are hand-measured. A weld pass is the
# case that strains it -- the gun stands off the work by 20 mm and its own
# body then clears the part by 1.3-2.0 mm on this fixture, so at the verifier's
# default every good seam reports a collision it does not have. Whether the
# pad or the standoff is the wrong number is an engineering call and not the
# planner's, so nothing here is quietly loosened; the number is merely made
# visible and the clearance is always reported beside the verdict.
VERIFY_MARGIN_M = DEFAULT_VERIFY_MARGIN_M

# How many of M4's ways across a seam this stage re-ranks. Five is what the
# operator UI already asks the sweep for, so the two agree about what "the
# alternatives" means without a second number to keep in step.
K_BEST = 5

DLS_DAMPING = 1e-4
PROJECT_ITERS = 2
INNER_STEPS = 60
OUTER_STEPS = 4

#: Below this many inner steps a CUDA graph capture cannot pay for its own
#: warm-up and recording. `freespace.CAPTURE_MIN_STEPS`, unchanged.
CAPTURE_MIN_STEPS = 16

#: The synthesis loop's working precision. float32 for the control solver's
#: reason: the FK constants and both collision fields are float32 already,
#: the quality gates are millimetres (`midpoint_tracking_m < 0.5e-3` where
#: float32 noise is ~1e-7 m), and the deployment target runs FP64 at 1/64 of
#: FP32. The T2 verifier's own copy of the contract stays float64 -- it is
#: the judge, and the judge does not adopt the defendant's precision.
WELD_DTYPE = torch.float64
ADAM_LR = 2e-3

# What an ACCEPTED step does to a row's trust scale, and 1.0 means nothing --
# which makes the scale below a RATCHET: it halves on every rejection and can
# never climb back. The value is named rather than left as a literal because it
# is known to be wrong and is deliberately not being changed here.
#
# `freespace.py` had the identical loop and the freeze it causes is measured in
# `docs/issues/0011`: at the augmented-Lagrangian multiplier growth the
# objective legitimately jumps, a few steps are correctly rejected, the scale
# falls by four halvings, and the remaining two thirds of the budget then runs
# at a 100% accept rate and a sixteenth of the step. Recovery capped at 1.0
# fixed it there.
#
# Why not here, yet: this loop runs a hard seam projection every step, which
# moves the iterate outside the objective's logic in a way free space has no
# equivalent of, and what a recovering step size does in the presence of it has
# not been measured. M5's numbers are pinned in `BENCHMARKS.md` 11 and any
# change to them has to be made knowingly rather than inherited from a
# measurement taken on a different problem.
TRUST_GROWTH = 1.0


class GenerateError(RuntimeError):
    """A synthesis this refuses to run, with the reason."""


# --------------------------------------------------------------------------
# The output


@dataclass
class SeamTrajectory:
    """What one seam's synthesis did, in numbers a reader can act on."""

    seam_id: str
    # THE WIRE: the emitted Hermite sampled at `WIRE_DT_S`, `(N, dof)` -- the
    # arm's axes, then the positioner LAST -- with the curve's own velocity
    # at each sample and the seam fraction each sample tracks. The lattice
    # knots the stage optimized are a subset of these samples (every
    # `step_s / WIRE_DT_S`-th), and `lattice_samples` says how many.
    knots: np.ndarray             # (N, dof) radians
    knot_velocity: np.ndarray     # (N, dof) rad/s
    times_s: np.ndarray
    # Tracking at the knots (exact, from pose_error) and at the cost grid's
    # interior samples (through the Hermite; REPORTED, not certified).
    knot_tracking_m: float
    midpoint_tracking_m: float
    # The smoothness cost and the worst normalised acceleration, before and
    # after -- "did the corner get better" as numbers.
    cost_before: float
    cost_after: float
    acc_ratio_before: float       # max |acc| / limit over axes and knots
    acc_ratio_after: float
    clearance_before_m: float
    clearance_after_m: float
    limit_violations: list[dict[str, Any]] = field(default_factory=list)
    limit_unverifiable: list[str] = field(default_factory=list)
    # The verifier's answers, which are different sentences from the numbers
    # above them: `midpoint_tracking_m` is the worst error at points the
    # optimizer LOOKED at, `tracking` is a bound on the worst error anywhere,
    # and `verdict` is the collision verdict over the emitted CURVE rather than
    # over the polyline through its knots.
    tracking: TrackingCertificate | None = None
    verdict: Verdict | None = None
    # The ranking. `objective` is what the K candidates were compared on,
    # `chosen_from` how many there were, and `rejected` what the losers
    # scored -- so the choice can be argued with rather than trusted.
    objective: float = 0.0
    chosen_from: int = 1
    # Where the winner sat in M4's order. Anything but 0 means the two
    # stages disagreed, which is the whole reason this stage re-ranks.
    sweep_rank: int = 0
    sweep_cost_rad: float = 0.0
    rejected: list[dict[str, Any]] = field(default_factory=list)
    fractions: np.ndarray | None = None      # (N,) seam fraction per sample
    lattice_samples: int = 0                 # knots the synthesis ran on
    # The same curve as the LATTICE Hermite: the knots the synthesis moved,
    # their velocities, clock and fractions. What the certificates are
    # computed on -- the verifier bisects every segment it is handed, so it
    # is handed the coarsest exact description of the curve rather than the
    # wire (25x the segments of the same cubic: measured 2.4 s -> 42.7 s of
    # T2 on the 22-seam corpus for a certificate over the identical path).
    lattice_knots: np.ndarray | None = None
    lattice_velocity: np.ndarray | None = None
    lattice_times_s: np.ndarray | None = None
    lattice_fractions: np.ndarray | None = None


def trajectory_json(traj: SeamTrajectory) -> dict[str, Any]:
    """One seam's continuous trajectory, on the wire.

    `(q, qdot, t)` at `WIRE_DT_S` -- the controller's clock, and free space's
    wire (`control_solver.WIRE_DT_S`). The pair at each sample is cubic
    Hermite data, so a reader that interpolates it is playing the SAME curve
    the optimizer shaped and the verifier certified: the samples are exact
    evaluations of that Hermite, not a resampling somebody chose the
    resolution of, and a Hermite through them reproduces it to float64
    (`resample_wire`). Nine lines of arithmetic on the other side;
    `ui/src/panes/plan/motion-panel.ts` holds them.

    Degrees, matching `plan.as_json`, because every reader of either prints
    degrees. Free space is NOT here: these are per-seam trajectories with no
    approach, retract or transit between them (M6), so a player must treat
    each as its own clock and not run one into the next.
    """
    tracking = traj.tracking
    verdict = traj.verdict
    return {
        **hermite_wire_degrees(
            traj.knots,
            traj.knot_velocity,
            traj.times_s,
        ),
        "lattice_samples": traj.lattice_samples,
        # Which of the sweep's candidates this came from, and what the ones
        # not chosen scored -- the picker shows the sweep's order, so a
        # trajectory that came from candidate 4 has to say so or it will look
        # like it belongs to candidate 0.
        "from_candidate": traj.sweep_rank,
        "chosen_from": traj.chosen_from,
        "rejected": traj.rejected,
        "objective": round(traj.objective, 4),
        "smoothness": {"before": round(traj.cost_before, 4),
                       "after": round(traj.cost_after, 4)},
        "acc_ratio": {"before": round(traj.acc_ratio_before, 4),
                      "after": round(traj.acc_ratio_after, 4)},
        "limit_violations": traj.limit_violations,
        "limit_unverifiable": traj.limit_unverifiable,
        # Two different sentences, and the UI must not merge them: the
        # midpoint number is the optimizer's own quadrature at points it
        # chose, the certificate bounds every instant. On the corner seams
        # they differ by a factor of nearly five.
        "sampled_tracking_mm": {
            "knots": round(traj.knot_tracking_m * 1000.0, 4),
            "midpoints": round(traj.midpoint_tracking_m * 1000.0, 4)},
        "tracking": None if tracking is None else tracking.summary(),
        "verdict": None if verdict is None else verdict.summary(),
    }


@dataclass
class SynthesisRun:
    """Every candidate of every seam, synthesized and NOT yet certified.

    What `WeldTrajoptModule.plan` reduces to winners and this keeps. The K candidates
    per seam are all synthesized -- they share the batch, so having them costs
    nothing extra -- and then the winner is taken and the rest are discarded
    down to a few numbers. That is right when the only question is which way
    across a seam is smoothest. It is wrong when something DOWNSTREAM has a
    view: which candidate a seam ends on decides where the arm stands when the
    next link starts, and on this cell that can decide whether the link exists
    at all (`freespace.rewound` -- no clear pose with `J5` in 50-145 degrees).

    So this hands the pool over intact and leaves certification to whoever
    chooses. Certification stays winner-only either way: it costs more than the
    synthesis that produced it.

    **Nothing chooses differently today, and that was measured rather than
    assumed.** A Viterbi over (seam, candidate) with the transit cost as the
    edge weight is the obvious consumer, and on this corpus it is worth 1% of
    transit time with every quality bar removed, 0.2% with one anyone would
    defend -- because a transit's length is set by where the seams are and not
    by which way the arm crossed them (`BENCHMARKS.md` 12.6: the edge varies
    2.3% across all 25 candidate combinations of a pair). This exists as the
    seam that chooser plugs into, with the number that would justify plugging
    something in.
    """

    seam_ids: list[str]
    candidates: list[list[SeamTrajectory]]     # per seam, best objective first
    synthesizer: Synthesizer
    jobs: list[SeamJob]
    verifier_collision_model: Any = None

    def certify(self, index: int, traj: SeamTrajectory, *,
                verify_margin_m: float = VERIFY_MARGIN_M) -> SeamTrajectory:
        """Judge the chosen candidate of seam `index`."""
        return self.synthesizer.certify(self.jobs[index], traj,
                                        verifier_collision_model=self.verifier_collision_model,
                                        verify_margin_m=verify_margin_m)


# --------------------------------------------------------------------------
# Every trajectory in the plan request, as one tensor problem


@dataclass
class SeamJob:
    """One seam's whole synthesis problem, before it is folded into a batch."""

    seam: Seam
    spec: dict[str, Any]
    joint: dict[str, Any] | None
    candidates_rad: np.ndarray        # (K, L, 7)
    times_s: np.ndarray
    fractions: np.ndarray
    held: dict[str, float]


def seam_local_targets(request: PlanRequest, seam: Seam, fractions, device,
                       dtype=torch.float64):
    """`(T, u)` at each fraction in the PLACED WORKPIECE frame -- `T` is the
    tool0 target, the seam's authored standoff already spent along `u`.

    Everything in the target composition that does not depend on the
    positioner: seam frames, the authored angles and standoff, and the
    placement. The
    fractions are fixed for the life of a synthesis -- the lattice chose them
    and the optimizer moves joints, not samples -- so this is loop-invariant
    and its caller computes it once.

    That it was NOT once cost 93% of the stage. Profiled: `targets` was 61% of
    a synthesis run and `Curve.locate` alone a third of that, re-solving `s ->
    u` by Newton for the same fractions on every Adam step. The tell was that
    a nine-knot seam took as long as a fifty-six-knot one -- the work was per
    ITERATION, not per sample.
    """
    fr = seam.frames(torch.as_tensor(fractions, dtype=dtype, device=device))
    work_deg, travel_deg = nodes.authored_angles(seam, np.asarray(fractions))
    u = apply_angles(fr["t"], fr["r"],
                     torch.as_tensor(work_deg, dtype=dtype, device=device),
                     torch.as_tensor(travel_deg, dtype=dtype, device=device))
    place = torch.as_tensor(placement_matrix(request.placement), dtype=dtype,
                            device=device)
    standoff = torch.as_tensor(
        nodes.authored_standoff_m(seam, np.asarray(fractions)),
        dtype=dtype, device=device)
    placed_u = u @ place[:3, :3].T
    return (fr["pos_m"] @ place[:3, :3].T + place[:3, 3]
            + standoff.unsqueeze(-1) * placed_u,
            placed_u)


class Synthesizer:
    """Every trajectory in one plan request, synthesized as ONE tensor problem.

    A row is a (seam, candidate) pair. Rows share a cell -- the same arm, the
    same collision model, the same limits -- and that is all they need to
    share, because everything a seam owns (its curve, its fractions, its
    clock, its tolerance) rides along per row.

    Why not one seam at a time: the loop is launch-bound, not compute-bound
    (`BENCHMARKS.md` 11.3 -- 190 ms of GPU work inside 533 ms of CPU, ~1 900
    kernel launches a step). Seven seams run separately cost seven times the
    launches for the same arithmetic. Batched, the whole plan request costs what
    one seam used to.

    **Rows are padded to the longest seam and the pad is inert.** Each short
    row repeats its last real knot, so the pad holds still, and every SUM is
    masked so the pad contributes nothing. Repeating alone would not do: the
    step from the last real velocity down to the pad's zero reads as a
    violent deceleration, and the acceleration term would price a stop that
    is not in the trajectory.
    """

    def __init__(self, request: PlanRequest, jobs: list[SeamJob],
                 planner_collision_model, device, cell: Any = None):
        if not jobs:
            raise GenerateError("nothing to plan")
        self.request = request
        self.jobs = jobs
        self.device = device
        self.planner_collision_model = planner_collision_model
        # Assembled by the caller. Building the chains here is how two stages
        # came to disagree about the same cell.
        self.cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
        tool = str((self.cell.descriptor.get("frames") or {}).get("tool") or "")
        self.arm = self.cell.arm.chain
        self.tool_index = self.arm.link_names.index(tool)
        self.tool_asset = self.cell.arm.tool

        # The knot layout: the arm's axes then the positioner -- which is
        # `cell.moved_axes`' own order, CHECKED rather than assumed, because
        # every `[..., :arm_dof]` below silently reads the wrong columns if the
        # two ever drift.
        moved_axes = list(self.cell.moved_axes)
        if moved_axes[:self.arm.num_dof] != list(self.arm.axis_names):
            raise GenerateError(
                f"the cell's moved axes {moved_axes} do not start with the arm's "
                f"own {list(self.arm.axis_names)}; the knot layout depends on it")
        self.arm_dof = self.arm.num_dof
        self.dof = len(moved_axes)
        # `moved_axes` puts the positioner LAST, and holds only the arm when the
        # cell has none -- `rosie_1420_v1`, whose part is fixtured to the floor.
        # `None` rather than a column the knots do not have, so every read of it
        # goes through `positioner_of` and cannot silently address J6 instead.
        self.positioner_column = self.dof - 1 if self.dof > self.arm_dof else None

        # The boxes the clip holds the knots inside. The arm's come off the
        # cell's own limits, at this stage's float64; the positioner is held to
        # the plan's +-90 cap from the SAME `positioner_plan_limits` the lattice
        # binned inside -- this was `clamp_(-pi/2, pi/2)` on a hand-written axis
        # index for a while, one policy spelled twice.
        self.arm_lower, self.arm_upper = limits_of(self.cell.descriptor, self.arm,
                                                   device, WELD_DTYPE)
        self.positioner_lower = self.positioner_upper = None
        if self.positioner_column is not None:
            self.positioner_lower, self.positioner_upper = nodes.positioner_plan_limits(
                *nodes.positioner_hardware_limits(self.cell.descriptor,
                                                  self.cell.positioner_axis),
                plan_span_rad=np.pi)

        # A row per (seam, candidate). `row_job` is what lets every per-seam
        # quantity below be gathered rather than assumed equal -- `dt` and the
        # tolerance happen to match across this corpus, and a count-mode plan
        # gives every seam its own clock (`plan.edge_rule`). One global value
        # where a per-seam one belongs is a bug this repository has already
        # paid for once.
        self.row_job = [i for i, job in enumerate(jobs)
                        for _ in range(len(job.candidates_rad))]
        self.B = len(self.row_job)
        self.L = max(job.candidates_rad.shape[1] for job in jobs)
        rows = torch.as_tensor(self.row_job, device=device)
        self.lengths = torch.as_tensor(
            [jobs[j].candidates_rad.shape[1] for j in self.row_job], device=device)
        index = torch.arange(self.L, device=device)
        self.mask = index.unsqueeze(0) < self.lengths.unsqueeze(1)      # (B, L)
        # The cost GRID: every knot plus `INTERPOLATION_STEPS - 1` samples of
        # the Hermite inside each span, `(L - 1) * n + 1` per row and in
        # TIME ORDER -- the swept read relies on that: a grid sample's chords
        # run to `s +- 1`. The pad's spans are zero-length (the pad repeats
        # a knot and carries no velocity), so their samples coincide with
        # the last real one and `sample_mask` keeps them out of every sum.
        self.steps_per_span = max(int(INTERPOLATION_STEPS), 1)
        self.S = (self.L - 1) * self.steps_per_span + 1
        grid_index = torch.arange(self.S, device=device)
        self.sample_mask = (grid_index.unsqueeze(0)
                            <= ((self.lengths - 1) * self.steps_per_span).unsqueeze(1))
        self.sample_pad = self.sample_mask.to(WELD_DTYPE)
        u = (torch.arange(1, self.steps_per_span, device=device, dtype=WELD_DTYPE)
             / self.steps_per_span)
        # Hermite basis at the interior parameters: `(n - 1, 4)` for
        # `(q0, h d0, q1, h d1)`.
        self.hermite_weights = torch.stack(
            [2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u,
             -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2], dim=-1)
        self.dt = torch.as_tensor(
            [float(jobs[j].times_s[1] - jobs[j].times_s[0]) for j in self.row_job],
            dtype=WELD_DTYPE, device=device).view(-1, 1, 1)
        self.tolerance_m = torch.as_tensor(
            [float((jobs[j].spec.get("tolerance") or {}).get("position_mm") or 0.5) / 1000.0
             for j in self.row_job], dtype=WELD_DTYPE, device=device).view(-1, 1)
        limits = getattr(self.cell.profile, "limits", None) or {}
        names = list(self.cell.moved_axes)
        self.vmax = torch.tensor(
            [float((limits.get(n) or {}).get("velocity_rad_s") or 1e9) for n in names],
            dtype=WELD_DTYPE, device=device)
        self.amax = torch.tensor(
            [float((limits.get(n) or {}).get("acceleration_rad_s2") or 1e9) for n in names],
            dtype=WELD_DTYPE, device=device)

        # The knots: arm then positioner, float64 -- the whole trajectory is a
        # few thousand numbers and the projection's Newton steps deserve the
        # precision. Short rows repeat their last real knot into the pad, so
        # the pad holds still; the masks above are what keep it out of the
        # sums.
        packed = np.zeros((self.B, self.L, self.dof))
        row = 0
        for job in jobs:
            for candidate in job.candidates_rad:
                n = len(candidate)
                packed[row, :n] = candidate
                packed[row, n:] = candidate[-1]
                row += 1
        self.x = torch.tensor(packed, dtype=WELD_DTYPE, device=device,
                              requires_grad=True)

        # The loop-invariant half of the target, evaluated once per seam and
        # then gathered per row. There are exactly two fraction sets per seam
        # and neither ever changes: the knots the lattice chose, and the
        # grid's interior samples between them. See `seam_local_targets` for
        # what recomputing them every step cost.
        work = str((self.cell.descriptor.get("frames") or {}).get("work") or "")
        self.work_chain = self.cell.work_chain
        self.work_index = self.work_chain.link_names.index(work)
        self._local = {
            where: self._pack_targets(where, rows) for where in ("knots", "interior")
        }
        # The work chain's held axes, per row, so a seam pinned to a different
        # station does not inherit its neighbour's.
        column_of = {n: i for i, n in enumerate(self.work_chain.axis_names)}
        base = torch.zeros(self.B, self.work_chain.num_dof, dtype=WELD_DTYPE,
                           device=device)
        for r, j in enumerate(self.row_job):
            for name, value in jobs[j].held.items():
                if name in column_of:
                    base[r, column_of[name]] = float(value)
        self.work_base = base
        # The pad repeats a real pose, so it is a legitimate pose and the
        # collision term would happily charge for it -- fifteen extra copies
        # of a 41-knot row's last stance. Harmless only while that stance is
        # clear, which is a coincidence and not a guarantee; `sample_pad`
        # above is what keeps it out.
        #
        # The scene's own checker, bound to this batch: `cell.moved_axes` in,
        # the scene's chain out. Held axes are per SEAM, so they go in as one
        # value per GRID SAMPLE and not once for the batch -- two seams
        # pinned to different stations would otherwise be screened against
        # one another's model.
        self.pose_check = None
        if planner_collision_model is not None:
            per_pose_held = {
                name: torch.as_tensor(
                    [float(jobs[j].held.get(name, 0.0)) for j in self.row_job],
                    dtype=WELD_DTYPE, device=device).repeat_interleave(self.S)
                for name in {n for job in jobs for n in job.held}}
            self.pose_check = planner_collision_model.checker(
                list(self.cell.moved_axes), per_pose_held)
        # ZERO is a cell, not a disagreement: the work chain of a cell with no
        # positioner has no axis for a seam to drive. More than one is still
        # refused -- the knots carry a single positioner column, so two driven
        # work axes would silently pose the second one at its held value.
        driven = {n for j in jobs for n in self.work_chain.axis_names if n not in j.held}
        if len(driven) > 1:
            raise GenerateError(
                f"the seams disagree about which work axis is driven: {sorted(driven)}")
        if bool(driven) != (self.positioner_column is not None):
            raise GenerateError(
                f"the cell's moved axes say the positioner is "
                f"{self.cell.positioner_axis!r} and the seams drive {sorted(driven)}; "
                "the knot layout and the work chain have to agree")
        self.work_driven = column_of[driven.pop()] if driven else None

    def _pack_targets(self, where: str, rows: torch.Tensor):
        """`(B, n, 3)` seam point and torch direction, padded like the knots.

        `"interior"` is the grid's samples between the knots, span-major:
        `(L - 1) * (steps_per_span - 1)` fractions, each span's interior at
        `u = i / steps_per_span`, linear in the span's own fraction range --
        the same rule the tracking certificate interpolates by, so the
        optimizer and its judge aim at the same seam point at each instant.
        """
        interior = self.steps_per_span - 1
        width = self.L if where == "knots" else (self.L - 1) * interior
        point = torch.zeros(len(self.jobs), width, 3, dtype=WELD_DTYPE,
                            device=self.device)
        direction = torch.zeros_like(point)
        if width == 0:
            return point[rows], direction[rows]
        for j, job in enumerate(self.jobs):
            fractions = np.asarray(job.fractions, dtype=np.float64)
            if where == "interior":
                u = np.arange(1, self.steps_per_span, dtype=np.float64) / self.steps_per_span
                fractions = (fractions[:-1, None]
                             + (fractions[1:] - fractions[:-1])[:, None] * u[None, :]
                             ).reshape(-1)
            p, u = seam_local_targets(self.request, job.seam, fractions, self.device)
            n = len(p)
            point[j, :n], direction[j, :n] = p, u
            point[j, n:], direction[j, n:] = p[-1], u[-1]
        return point[rows], direction[rows]

    # ------------------------------------------------------------- targets

    def positioner_of(self, knots):
        """The positioner's column of a knot tensor, or a zero stand-in.

        A cell with no positioner has no such column, and every caller here
        wants the same thing from it: the angle to drive the work chain to.
        Zero is that angle -- the work chain has no joint to turn, so
        `torch_link_transform` walks its fixed offsets and ignores the value --
        and it is a CONSTANT, carrying no gradient, because there is no degree
        of freedom for a gradient to reach.

        Taking the whole knot tensor and dropping its last axis, rather than
        letting each site build its own zeros, is what keeps `targets`' `(b, n)`
        contract: the five callers slice the knots differently -- live, detached,
        the interior grid, a report's certified curve -- and each needs the
        leading shape of whatever IT sliced.
        """
        if self.positioner_column is not None:
            return knots[..., self.positioner_column]
        return torch.zeros(knots.shape[:-1], dtype=knots.dtype,
                           device=knots.device)

    def targets(self, where: str, theta):
        """`(tool0 target, u)` -- the seam's authored standoff already applied
        by `seam_local_targets`, because the thing being projected is the TOOL
        frame, not the electrode's ideal. Projecting the tip onto the weld
        POINT was this module's first bug -- it slammed the gun 20 mm into the
        part on the very first projection and everything after was divergence
        dressed as optimization.

        `where` names a cached fraction set rather than passing the fractions,
        so a caller cannot quietly ask for a set that was never precomputed.
        `theta` is `(K, n)` over candidates and knots; the result matches.
        """
        b, n = theta.shape
        point, u_local = self._local[where]
        qpos = self.work_base.unsqueeze(1).expand(b, n, -1).reshape(b * n, -1).clone()
        # Nothing to drive on a cell with no positioner: `qpos` is then
        # `(b * n, 0)` and the walk to the work frame is its fixed offsets.
        if self.work_driven is not None:
            qpos[:, self.work_driven] = theta.reshape(-1)
        wrot, wpos = fk.torch_link_transform(self.work_chain, self.work_index, qpos)
        # `(R * v).sum(-1)` and not `R @ v.unsqueeze(-1)`: the batched 3x1
        # bmm's BACKWARD dispatches through triton, which this environment
        # cannot compile -- the same trap `robot/fk.py` documents for einsum.
        flat_p = point.reshape(-1, 3)
        flat_u = u_local.reshape(-1, 3)
        p = (wrot * flat_p.unsqueeze(-2)).sum(-1) + wpos
        u = (wrot * flat_u.unsqueeze(-2)).sum(-1)
        return p.view(b, n, 3), u.view(b, n, 3)

    def tool_pose(self, q_arm):
        """`(rot, pos)` for `(K, n, 6)` of arm angles, shaped to match.

        The chain functions all take one leading batch dimension, so the
        candidate and knot axes are folded together for the call and unfolded
        after. That fold IS the batching: fourteen candidate-runs of 41 poses
        launch fourteen times the kernels that one run of 574 does, for the
        same arithmetic (`BENCHMARKS.md` 11.3).
        """
        k, n = q_arm.shape[:2]
        rot, pos = fk.torch_link_transform(self.arm, self.tool_index,
                                           q_arm.reshape(-1, self.arm_dof))
        return rot.view(k, n, 3, 3), pos.view(k, n, 3)

    # ---------------------------------------------------------- projection

    @torch.no_grad()
    def project(self, iters: int = PROJECT_ITERS) -> None:
        """DLS Newton: tip to `P`, electrode to `-u`, spin untouched."""
        for _ in range(iters):
            q = self.x[..., :self.arm_dof]
            target_p, target_u = self.targets(
                "knots", self.positioner_of(self.x))
            rot, _pos = self.tool_pose(q)
            # Cached: per call this was a host-to-device copy of a constant,
            # a synchronise in eager and a capture-invalidating one recorded.
            axis = getattr(self, "_electrode_axis_dev", None)
            if axis is None or axis.dtype != rot.dtype:
                axis = torch.as_tensor(
                    np.asarray(self.tool_asset.electrode_axis),
                    dtype=rot.dtype, device=rot.device)
                self._electrode_axis_dev = axis
            flat_q = q.reshape(-1, self.arm_dof)
            current_axis = rot.reshape(-1, 3, 3) @ axis
            correction = _align_rotation(current_axis, -target_u.reshape(-1, 3))
            target_rot = correction @ rot.reshape(-1, 3, 3)
            e_pos, e_rot, _ = pose_error(self.arm, self.tool_index, flat_q,
                                         target_rot, target_p.reshape(-1, 3))
            error = torch.cat([e_pos, e_rot], dim=-1).unsqueeze(-1)  # (K*L, 6, 1)
            jac = jacobian(self.arm, self.tool_index, flat_q)         # (K*L, 6, 6)
            jjt = jac @ jac.transpose(-1, -2)
            eye = torch.eye(6, dtype=jjt.dtype, device=jjt.device) * DLS_DAMPING
            # `solve_ex`, not `solve`: the plain form reads the factorization's
            # info flag back to the HOST to raise on singularity, and that
            # synchronise invalidates a CUDA graph capture mid-recording.
            # The system is damped (`+ eye`), so it is never singular and
            # the check never had anything to find.
            step = (jac.transpose(-1, -2)
                    @ torch.linalg.solve_ex(jjt + eye, error)[0])
            self.x.data[..., :self.arm_dof] += (
                step.squeeze(-1).view(self.B, self.L, self.arm_dof))

    @torch.no_grad()
    def knot_velocity(self) -> torch.Tensor:
        """(L, dof) rad/s: finite-difference filled into the tip constraint.

        `q̇ = q̇_fd + J⁺(ṗ_des − J q̇_fd)` -- the natural motion projected onto
        the affine set of velocities whose tip matches the target's own rate.
        Linear, one damped solve, exact.
        """
        x = self.x.detach()
        qdot = _central_difference(x, self.dt, self.lengths)
        target_p, _ = self.targets("knots", self.positioner_of(x))
        pdot = _central_difference(target_p, self.dt, self.lengths)

        jac = jacobian(self.arm, self.tool_index,
                       x[..., :self.arm_dof].reshape(-1, self.arm_dof)
                       )[:, :3, :]                             # (K*L, 3, arm)
        flat_qdot = qdot[..., :self.arm_dof].reshape(-1, self.arm_dof)
        residual = pdot.reshape(-1, 3) - (jac @ flat_qdot.unsqueeze(-1)).squeeze(-1)
        jjt = jac @ jac.transpose(-1, -2)
        eye = torch.eye(3, dtype=jjt.dtype, device=jjt.device) * DLS_DAMPING
        # `solve_ex` for `project`'s reason: damped, never singular, and the
        # plain form's host read of the info flag kills the graph capture.
        fix = (jac.transpose(-1, -2)
               @ torch.linalg.solve_ex(jjt + eye, residual.unsqueeze(-1))[0])
        qdot[..., :self.arm_dof] += (
            fix.squeeze(-1).view(self.B, self.L, self.arm_dof))
        return qdot

    # --------------------------------------------------------------- costs

    def smoothness(self, x: torch.Tensor) -> torch.Tensor:
        """(B,) -- PER ROW, because the point of batching is to rank them, and
        one summed scalar would make every trajectory in the plan request a single
        problem.

        Each difference is masked to the rows that actually have that many
        knots. Repeating the last knot into the pad makes the pad hold still,
        but the step from the last real VELOCITY down to the pad's zero would
        read as a violent deceleration -- so the acceleration and jerk terms
        would price a stop that is not in the trajectory.
        """
        vel = (x[:, 1:] - x[:, :-1]) / self.dt
        acc = (vel[:, 1:] - vel[:, :-1]) / self.dt
        jerk = (acc[:, 1:] - acc[:, :-1]) / self.dt
        v = ((vel / self.vmax).pow(2) * self._span(1)).sum(dim=(1, 2))
        a = ((acc / self.amax).pow(2) * self._span(2)).sum(dim=(1, 2))
        j = ((jerk / (self.amax / self.dt)).pow(2) * self._span(3)).sum(dim=(1, 2))
        return W_VEL * v + W_ACC * a + W_JERK * j

    def _span(self, order: int) -> torch.Tensor:
        """(B, L-order, 1): true where a difference of this order is real."""
        index = torch.arange(self.L - order, device=self.device)
        return (index.unsqueeze(0) < (self.lengths - order).unsqueeze(1)
                ).to(WELD_DTYPE).unsqueeze(-1)

    def grid(self, x: torch.Tensor, qdot: torch.Tensor) -> torch.Tensor:
        """(B, S, dof): the emitted curve at the cost grid -- every knot and
        the cubic Hermite at `steps_per_span - 1` points inside each span.

        The velocities are `knot_velocity`'s, a linear solve outside the
        autodiff graph, so a grid sample's gradient reaches the knots through
        the Hermite's position weights alone. That is the bias `_objective`
        explains and its monotone gate exists for; the collision and tracking
        terms read this grid and inherit both.
        """
        n = self.steps_per_span
        q0, q1 = x[:, :-1], x[:, 1:]
        if n < 2:
            return x
        d0, d1 = qdot[:, :-1] * self.dt, qdot[:, 1:] * self.dt
        w = self.hermite_weights.view(1, 1, -1, 4, 1)
        inner = (w[..., 0, :] * q0.unsqueeze(2) + w[..., 1, :] * d0.unsqueeze(2)
                 + w[..., 2, :] * q1.unsqueeze(2) + w[..., 3, :] * d1.unsqueeze(2))
        # The pad's FIRST span runs from the last real knot -- which has a
        # real velocity -- to its motionless copy, and a Hermite between two
        # equal points with a nonzero start rate bows OUT. Those samples are
        # masked from every sum, but the swept read walks the chord from the
        # last real sample toward them, and that is a pad tax on a real
        # constraint (measured: a short seam's descent changed by 2x when
        # batched with a long one). Every pad span is held at the knot, so
        # the pad's chords are zero-length and the walk never reads them.
        inner = torch.where(self._span(1).unsqueeze(-1).bool(), inner,
                            q0.unsqueeze(2))
        spans = torch.cat([q0.unsqueeze(2), inner], dim=2)          # (B, L-1, n, dof)
        return torch.cat([spans.reshape(self.B, (self.L - 1) * n, self.dof),
                          x[:, -1:]], dim=1)

    @torch.no_grad()
    def grid_now(self) -> torch.Tensor:
        """The grid at the current iterate, velocities freshly solved."""
        return self.grid(self.x.detach(), self.knot_velocity())

    def interior_tracking(self, grid: torch.Tensor) -> torch.Tensor:
        """(B, L-1, n-1): tip-to-seam distance at each interior grid sample --
        the optimizer's own quadrature of what happens BETWEEN the knots.
        Zero-width on the pad; empty at one step a span."""
        n = self.steps_per_span
        if n < 2:
            return torch.zeros(self.B, self.L - 1, 0, dtype=grid.dtype,
                               device=grid.device)
        inner = grid[:, :-1].view(self.B, self.L - 1, n, self.dof)[:, :, 1:]
        theta = self.positioner_of(inner).reshape(self.B, -1)
        target_p, _ = self.targets("interior", theta)
        _, pos = self.tool_pose(inner[..., :self.arm_dof].reshape(self.B, -1, self.arm_dof))
        return ((pos - target_p).norm(dim=-1).view(self.B, self.L - 1, n - 1)
                * self._span(1))

    def tracking_term(self, grid: torch.Tensor) -> torch.Tensor:
        """(B,) `W_TRACK_MID` times, per span, the mean squared normalised
        interior error -- so the weight means the same at every grid."""
        error = self.interior_tracking(grid)
        if not error.shape[-1]:
            return torch.zeros(self.B, dtype=grid.dtype, device=grid.device)
        return W_TRACK_MID * (error / self.tolerance_m.view(-1, 1, 1)
                              ).pow(2).mean(dim=2).sum(dim=1)

    def _objective(self, collision: CollisionCost | None) -> torch.Tensor:
        """The PROJECTED objective the monotone gate judges, `(B,)`.

        The grid terms treat the knot velocities as constants (they come
        from a linear solve, not the autodiff graph), so their gradient is
        biased and does not vanish at the optimum -- left to run free, that
        bias walks a converged trajectory uphill: measured, a seam at 0.05 mm
        midpoint error drifted to 6.9 mm while every individual step thought
        it was helping. So a step only stands if this number, re-evaluated
        AFTER clip and projection with fresh velocities, got better;
        otherwise the step is reverted and the trust rate halves. Descent
        becomes a guarantee instead of a hope, and a seam that is already
        right stays exactly where it is.

        Every quantity is `(B,)` and every decision a mask. The scalar
        version read `float(value)` once a step, and each read was a
        GPU-to-CPU synchronise in the middle of the loop -- which is also
        what would have made the step uncapturable as the CUDA graph
        `_capture` now records.
        """
        with torch.no_grad():
            value = self.smoothness(self.x)
            grid = self.grid(self.x, self.knot_velocity())
            value = value + self.tracking_term(grid)
            if collision is not None:
                value = value + (collision.value(self._full_q(grid))
                                 .view(self.B, self.S) * self.sample_pad).sum(dim=1)
            return value

    def _step(self, opt: torch.optim.Optimizer,
              collision: CollisionCost | None, scale: torch.Tensor,
              best: torch.Tensor) -> None:
        """One monotone step, in place on `(x, moments, scale, best)`.

        Every quantity it touches is a pre-existing tensor updated in place
        and every decision a mask -- no device-to-host read anywhere, which
        is what makes this the unit a CUDA graph can replay.
        """
        with torch.no_grad():
            self._snapshot.copy_(self.x.detach())
        opt.zero_grad(set_to_none=False)
        loss = self.smoothness(self.x)
        grid = self.grid(self.x, self.knot_velocity())
        loss = loss + self.tracking_term(grid)
        if collision is not None:
            # At the GRID, swept: the cost's constraint is the least
            # clearance each sphere meets at a sample and along the chords
            # to its neighbours (`CollisionCost.sweep_samples`), so a wall
            # between two samples is priced here and not found by T2.
            loss = loss + (collision.value(self._full_q(grid))
                           .view(self.B, self.S) * self.sample_pad).sum(dim=1)
        # The candidates share no term, so one backward over their sum
        # gives each exactly the gradient it would have had alone.
        loss.sum().backward()
        opt.step()
        with torch.no_grad():
            self.x.data.copy_(self._snapshot
                              + scale * (self.x.data - self._snapshot))
        self._clip()
        self.project(iters=1)
        now = self._objective(collision)
        with torch.no_grad():
            worse = now > best + 1e-9
            self.x.data.copy_(torch.where(worse.view(-1, 1, 1),
                                          self._snapshot, self.x.data))
            _forget_moments(opt, self.x, worse)
            scale.mul_(torch.where(worse, 0.5, TRUST_GROWTH).view(-1, 1, 1))
            scale.clamp_(min=5e-4)
            best.copy_(torch.minimum(best, now))

    def _capture(self, opt: torch.optim.Optimizer,
                 collision: CollisionCost | None, scale: torch.Tensor,
                 best: torch.Tensor, steps: int) -> tuple[Any, int]:
        """`(graph, warm steps taken)`: one `_step` recorded as a CUDA graph.

        `freespace.MoveSolver._capture`'s contract, ported unchanged: warm-up
        on a side stream because capture refuses to start while ordinary work
        is pending, the two warm runs are REAL steps the caller counts, and
        whatever refuses to record falls back to the eager loop with the
        iterate the warm-up left.
        """
        if self.device.type != "cuda" or steps < CAPTURE_MIN_STEPS:
            return None, 0
        try:
            stream = torch.cuda.Stream()
            stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(stream):
                for _ in range(2):
                    self._step(opt, collision, scale, best)
            torch.cuda.current_stream().wait_stream(stream)
            graph = torch.cuda.CUDAGraph()
            with torch.cuda.graph(graph):
                self._step(opt, collision, scale, best)
        except Exception:  # noqa: BLE001 -- whatever refused, eager answers
            import os
            if os.environ.get("WELD_PLANNER_CAPTURE_DEBUG"):
                import traceback
                traceback.print_exc()
            return None, 2
        return graph, 2

    # ----------------------------------------------------------------- run

    def run(self, collision: CollisionCost | None, *,
            inner_steps: int = INNER_STEPS,
            outer_steps: int = OUTER_STEPS) -> dict[int, list[SeamTrajectory]]:
        """Reports keyed by job index, each list in the sweep's own order."""
        # Baselines before anything moves.
        self.project()
        with torch.no_grad():
            cost0 = self.smoothness(self.x)
            acc0 = self._acc_ratio()
            clear0 = self._clearance(collision)

        # MONOTONE, by accept/reject on the PROJECTED objective -- the
        # contract is `_objective`'s docstring, the unit is `_step`, and one
        # `_step` recorded as a CUDA graph serves the whole descent because
        # every decision in it is a device-side mask.
        capturable = self.device.type == "cuda"
        opt = torch.optim.Adam([self.x], lr=ADAM_LR, capturable=capturable)
        # Per candidate, because one of them backtracking must not slow the
        # rest down. Adam's own lr stays put and the step it proposes is
        # scaled on the way out, which is the same arithmetic per candidate
        # and lets them all share one optimizer.
        scale = torch.ones(self.B, 1, 1, dtype=WELD_DTYPE, device=self.device)
        best = self._objective(collision).clone()
        # A persistent buffer: the recorded kernels read THIS tensor, so the
        # revert baseline must live at one address for the life of the graph.
        self._snapshot = self.x.detach().clone()
        steps = max(int(inner_steps), 0)
        graph, warmed = ((None, 0) if not steps else
                         self._capture(opt, collision, scale, best, steps))
        for outer in range(outer_steps):
            # The capture's warm-up runs were real steps, taken out of the
            # FIRST outer's budget and nobody else's.
            for _ in range(steps - (warmed if outer == 0 else 0)):
                if graph is not None:
                    graph.replay()
                else:
                    self._step(opt, collision, scale, best)
            if collision is not None:
                collision.step_multipliers(self._full_q(self.grid_now()), grow=True)
                # The multipliers just moved the landscape; the incumbent
                # objective and Adam's moments both describe the old one.
                # A fresh optimizer, without becoming a fresh OBJECT: the
                # graph replays against this one's state tensors, so the
                # moments and the step count are zeroed in place instead.
                # (`rho` and the multipliers are device tensors updated in
                # place by `step_multipliers` for the same reason.)
                with torch.no_grad():
                    best.copy_(self._objective(collision))
                    for state in opt.state.values():
                        for moment in state.values():
                            if torch.is_tensor(moment):
                                moment.zero_()

        self.project(iters=PROJECT_ITERS + 1)
        qdot = self._fit_rate_set(self.knot_velocity())
        return self._reports(qdot, collision, cost0, acc0, clear0)

    @torch.no_grad()
    def _fit_rate_set(self, qdot: torch.Tensor) -> torch.Tensor:
        """Knot velocities whose CURVE stays inside the joint rate limits.

        `_clip` bounds the knot-to-knot step, which is the chord. What ships is
        the cubic Hermite through those knots, and between two knots it bulges
        past its own chord: on `rosie_1420_v1` seam_0008 the chords sat at 0.936
        of J1's limit and the curve peaked at 1.086 in ONE segment of 131, which
        T2 refused. `_clip` could not see it and ran clean; running it more
        times changed nothing, because the thing it bounds was never the thing
        that was over.

        The derivative on a segment is a quadratic in `u` through `m0` and `m1`
        whose mean is the chord, so its peak is closed form -- `hermite_peak`,
        which agrees with a 2001-sample search to 1e-8 rad/s. Pulling `m0` and
        `m1` toward the chord shrinks the deviation LINEARLY and reaches the
        chord itself, where the segment is straight and the peak is `|c|` --
        already inside the limit because `_clip` put it there. So a pull always
        exists; the iteration is only because a knot's velocity is shared with
        the segment on the other side of it.
        """
        limit = self.vmax.view(1, 1, -1)
        h = self.dt.view(-1, 1, 1)
        for _ in range(RATE_FIT_ROUNDS):
            chord = (self.x.data[:, 1:] - self.x.data[:, :-1]) / h
            peak = hermite_peak(qdot[:, :-1], qdot[:, 1:], chord)
            # The pad repeats the last real knot, so its chord is zero and its
            # peak cannot offend; no mask is needed to keep it out.
            if not bool((peak > limit).any()):
                break
            # How far each segment's velocities may stand off their chord and
            # still fit: |c| + s * deviation <= limit, by the triangle
            # inequality, so the same `s` bounds the endpoints and the vertex.
            spread = (peak - chord.abs()).clamp_min(1e-12)
            room = (limit - chord.abs()).clamp_min(0.0)
            scale = (room / spread).clamp(max=1.0)
            # A knot belongs to two segments and takes the TIGHTER of them,
            # pulling toward that segment's chord. The first and last knot have
            # one segment each; `inf` on the missing side lets the minimum pick
            # the real one without a special case.
            infinite = torch.full_like(qdot, float("inf"))
            before, after = infinite.clone(), infinite.clone()
            before[:, 1:] = scale            # the segment ending at this knot
            after[:, :-1] = scale            # the segment starting at it
            chord_before, chord_after = torch.zeros_like(qdot), torch.zeros_like(qdot)
            chord_before[:, 1:] = chord
            chord_after[:, :-1] = chord
            tighter_before = before < after
            per_knot = torch.minimum(before, after).clamp(max=1.0)
            reference = torch.where(tighter_before, chord_before, chord_after)
            qdot = reference + per_knot * (qdot - reference)
        return qdot

    @torch.no_grad()
    def _reports(self, qdot: torch.Tensor, collision: CollisionCost | None,
                 cost0: torch.Tensor, acc0: torch.Tensor, clear0: torch.Tensor
                 ) -> dict[int, list[SeamTrajectory]]:
        """Every row measured and written up, in one device-to-host transfer.

        The `objective` written here is the number the K candidates are ranked
        against. NOT the loop's own `best`, which carries the augmented-
        Lagrangian term and therefore the multiplier state of one particular run
        -- two candidates' multipliers are not comparable quantities. This is a
        function of the trajectory alone: smoothness plus tracking, the two
        things M5 is for.
        """
        knots = self.x.detach()
        target_p, _ = self.targets("knots", self.positioner_of(knots))
        _, pos = self.tool_pose(knots[..., :self.arm_dof])
        knot_err = (pos - target_p).norm(dim=-1).amax(dim=1)
        grid = self.grid(knots, qdot)
        interior = self.interior_tracking(grid)
        worst_interior = (interior.amax(dim=(1, 2)) if interior.shape[-1]
                          else torch.zeros(self.B, device=self.device))
        cost1 = self.smoothness(knots)
        score = cost1 + self.tracking_term(grid)
        # ONE transfer off the device for the whole batch, at the end.
        bulk = {name: t.double().cpu().numpy() for name, t in
                (("knots", knots), ("qdot", qdot), ("knot_err", knot_err),
                 ("mid_err", worst_interior), ("cost0", cost0),
                 ("cost1", cost1), ("acc0", acc0),
                 ("acc1", self._acc_ratio()), ("clear0", clear0),
                 ("clear1", self._clearance(collision)), ("score", score))}

        out: dict[int, list[SeamTrajectory]] = {}
        for i, j in enumerate(self.row_job):
            job = self.jobs[j]
            n = job.candidates_rad.shape[1]        # the pad is not a trajectory
            # The WIRE: the Hermite through the knots, re-evaluated at the
            # controller's clock in float64 on the CPU.
            wire_q, wire_v, wire_t, wire_f = resample_wire(
                bulk["knots"][i][:n], bulk["qdot"][i][:n],
                np.asarray(job.times_s, dtype=np.float64),
                np.asarray(job.fractions, dtype=np.float64))
            out.setdefault(j, []).append(SeamTrajectory(
                seam_id=job.seam.id,
                knots=wire_q,
                knot_velocity=wire_v,
                times_s=wire_t,
                fractions=wire_f,
                lattice_samples=n,
                lattice_knots=bulk["knots"][i][:n],
                lattice_velocity=bulk["qdot"][i][:n],
                lattice_times_s=np.asarray(job.times_s, dtype=np.float64),
                lattice_fractions=np.asarray(job.fractions, dtype=np.float64),
                knot_tracking_m=float(bulk["knot_err"][i]),
                midpoint_tracking_m=float(bulk["mid_err"][i]),
                cost_before=float(bulk["cost0"][i]),
                cost_after=float(bulk["cost1"][i]),
                acc_ratio_before=float(bulk["acc0"][i]),
                acc_ratio_after=float(bulk["acc1"][i]),
                clearance_before_m=float(bulk["clear0"][i]),
                clearance_after_m=float(bulk["clear1"][i]),
                objective=float(bulk["score"][i]),
                chosen_from=len(job.candidates_rad),
                sweep_rank=len(out.get(j, [])),
            ))
        return out

    def certify(self, job: SeamJob, traj: SeamTrajectory, *,
                verifier_collision_model: Any = None,
                verify_margin_m: float = VERIFY_MARGIN_M) -> SeamTrajectory:
        """Judge a finished trajectory. Winner only, by design.

        Certification costs more than the synthesis that produced it -- the
        tracking bisection and the T2 verifier are both CPU walks over the whole
        continuum -- and judging every candidate to then discard all but one
        would spend that on trajectories nobody will run.

        Split in two on purpose: `certify_payload` gathers everything the
        judging needs -- it touches the GPU (`world_tangents`) and this
        object, so it runs HERE -- and `certify_cpu` is the judging itself,
        rebuilt from documents, so a caller with many winners can fan it out
        across worker processes instead of walking them one at a time.
        """
        return apply_certification(
            traj, certify_cpu(self.certify_payload(
                job, traj, verifier_collision_model=verifier_collision_model,
                verify_margin_m=verify_margin_m)))

    def certify_payload(self, job: SeamJob, traj: SeamTrajectory, *,
                        verifier_collision_model: Any = None,
                        verify_margin_m: float = VERIFY_MARGIN_M) -> dict[str, Any]:
        """Everything `certify_cpu` needs, as plain documents and arrays.

        Judged on the LATTICE Hermite, not the wire: the two are one curve
        (`resample_wire`), and the verifier's cost is per segment it is
        handed, so it is handed the fewest segments that describe the curve
        exactly. `SeamTrajectory.lattice_knots` says why in numbers.
        """
        kin = verify_kinematics(self.request.cell)
        profile = profile_for_cell(self.request.cell)
        knots, rates_in, times_s, fractions = _certified_curve(traj, job)
        n = len(knots)
        waypoints = np.zeros((n, len(kin.axis_names)))
        rates = np.zeros_like(waypoints)
        column_of = {n: i for i, n in enumerate(self.cell.moved_axes)}
        for i, name in enumerate(kin.axis_names):
            if name in column_of:
                waypoints[:, i] = knots[:, column_of[name]]
                rates[:, i] = rates_in[:, column_of[name]]
            elif name in job.held:
                waypoints[:, i] = job.held[name]
        # The velocities are the point. Without them the verifier certifies
        # the polyline through the knots, and the curve this stage emits
        # leaves that polyline by degrees of joint angle on a corner.
        #
        # T1's inputs go in too. They are not extras: `check_travel`
        # reports "not checked" without them, and an unchecked mobility
        # margin keeps the whole verdict out of PASS -- which is the
        # correct behaviour and the reason to supply them rather than
        # live with it. Both are already in hand here, the seam tangent
        # from the same composition the projection targets and the speed
        # from the process the program authored.
        return {
            "cell": self.request.cell,
            "limits": getattr(profile, "limits", {}) or {},
            "spec": job.spec, "joint": job.joint,
            "placement": placement_matrix(self.request.placement),
            "waypoints": waypoints, "rates": rates,
            "times_s": times_s, "fractions": fractions,
            "tolerance_m": float((job.spec.get("tolerance") or {})
                                 .get("position_mm") or 0.5) / 1000.0,
            "geometry": verifier_collision_model,
            "margin_m": verify_margin_m,
            "directions": (None if verifier_collision_model is None
                           else self.world_tangents(job, traj)),
            "speeds": np.full(n, nodes.travel_speed_m_s(job.seam)),
        }

    @torch.no_grad()
    def world_tangents(self, job: SeamJob, traj: SeamTrajectory) -> np.ndarray:
        """(L, 3) seam travel direction in WORLD at each knot of `traj`.

        The direction T1 measures mobility along. Taken through the same
        placement-then-work-chain composition the targets use, at the
        positioner angle the trajectory actually holds -- a tangent read in
        the part's own frame would describe a seam that is not where this
        cell put it.

        Runs at float64 on purpose while the loop runs float32: the seam's
        own curve is float64, this is once per REPORT rather than per step,
        and a judge-side quantity keeps the judge's precision.

        Takes the REPORT and not `self.x`: one synthesizer now holds every
        candidate, and reading the live tensor would answer for whichever row
        happened to be first rather than for the one being judged.
        """
        knots, _, _, fractions = _certified_curve(traj, job)
        theta = self.positioner_of(
            torch.as_tensor(knots, dtype=torch.float64, device=self.device))
        fr = job.seam.frames(torch.as_tensor(fractions, dtype=torch.float64,
                                             device=self.device))
        place = torch.as_tensor(placement_matrix(self.request.placement),
                                dtype=torch.float64, device=self.device)
        tangent = fr["t"] @ place[:3, :3].T
        chain = self.work_chain
        column_of = {n: i for i, n in enumerate(chain.axis_names)}
        qpos = torch.zeros(len(theta), chain.num_dof, dtype=torch.float64,
                           device=self.device)
        for name, value in job.held.items():
            if name in column_of:
                qpos[:, column_of[name]] = float(value)
        if self.work_driven is not None:
            qpos[:, self.work_driven] = theta
        wrot, _ = fk.torch_link_transform(chain, self.work_index, qpos)
        return (wrot * tangent.unsqueeze(-2)).sum(-1).cpu().numpy()

    def seam_target(self, job: SeamJob) -> SeamTarget:
        """The verifier's own copy of the contract: CPU, float64, rebuilt from the
        program documents rather than handed the optimizer's live seam."""
        return SeamTarget(Seam(job.spec, job.joint, dtype=torch.float64),
                          placement_matrix(self.request.placement))

    # ------------------------------------------------------------- helpers

    @torch.no_grad()
    def _clip(self) -> None:
        """Boxes, then the pairwise velocity clamp, forward then backward.

        The two sweeps are an approximate projection onto the rate set -- a
        forward pass alone shovels all the correction onto later knots. The
        verifier re-measures afterwards, so approximation costs iterations, not
        truth.
        """
        self.x.data[..., :self.arm_dof].clamp_(self.arm_lower, self.arm_upper)
        if self.positioner_column is not None:
            self.x.data[..., self.positioner_column].clamp_(self.positioner_lower,
                                                            self.positioner_upper)

        # The knot index still walks in Python -- the sweeps are sequential by
        # construction, each knot leaning on the one already fixed -- but it
        # now moves every candidate at once, so the launches divide by K.
        # `cap` is (B, 1, dof): the rate limit is per axis and `dt` is per ROW,
        # so a seam on a slower clock gets a tighter cap without anyone
        # writing that down twice. Slices keep their knot dimension for the
        # same broadcasting reason `_central_difference` explains.
        cap = self.vmax * self.dt.view(-1, 1, 1)
        for indices, direction in ((range(1, self.L), 1),
                                   (range(self.L - 2, -1, -1), -1)):
            for i in indices:
                anchor = i - direction        # the knot already fixed this sweep
                delta = self.x.data[:, i:i + 1] - self.x.data[:, anchor:anchor + 1]
                self.x.data[:, i:i + 1] = (self.x.data[:, anchor:anchor + 1]
                                           + delta.clamp(-cap, cap))

    def _full_q(self, x: torch.Tensor) -> torch.Tensor:
        """(K*L, dof) for the collision model, through the scene's own
        `checker` -- one home for the axis mapping and the held axes."""
        if self.pose_check is None:
            return x
        return self.pose_check.q_full(x.reshape(-1, x.shape[-1]))

    @torch.no_grad()
    def _acc_ratio(self) -> torch.Tensor:
        """(B,) worst normalised acceleration, per row, pad excluded."""
        vel = (self.x[:, 1:] - self.x[:, :-1]) / self.dt
        acc = (vel[:, 1:] - vel[:, :-1]) / self.dt
        ratio = (acc.abs() / self.amax) * self._span(2)
        return ratio.amax(dim=2).amax(dim=1)

    @torch.no_grad()
    def _clearance(self, collision: CollisionCost | None) -> torch.Tensor:
        """(K,) worst clearance, per candidate, over the cost grid and its
        chords -- the same read the cost descends on, so "clear" here means
        what the optimizer was asked for. Environment and self merged: a
        candidate that penetrates, or folds into itself, must lose on its
        own account and not drag the batch down with it."""
        if collision is None:
            return torch.full((self.B,), float("nan"), device=self.device)
        env, own = collision.constraints(self._full_q(self.grid_now()))
        clear = (env + collision.margin_m).min(dim=1).values
        if own.numel():
            clear = torch.minimum(
                clear, (own + collision.self_margin_m).min(dim=1).values)
        return (clear.view(self.B, self.S)
                .masked_fill(~self.sample_mask, float('inf')).min(dim=1).values)


def certify_limits_and_tracking(payload: dict[str, Any]) -> dict[str, Any]:
    """Limits and the tracking certificate, from documents and arrays alone.

    Module-level and torch-free on the GPU side, so a worker process can be
    handed it: the winners are independent and this is serial CPU work --
    measured, more than half of M5's stage time -- so `weld_trajopt_main`
    fans it out across one pool instead of walking the seams one at a time.
    """
    kin = verify_kinematics(payload["cell"])
    violations, unverifiable = check_limits(
        kin, payload["cell"], {"limits": payload["limits"]},
        payload["waypoints"], payload["times_s"],
        knot_velocities=payload["rates"])
    tracking = certify_tracking(
        kin, payload["cell"],
        SeamTarget(Seam(payload["spec"], payload["joint"], dtype=torch.float64),
                   payload["placement"]),
        payload["waypoints"], payload["rates"], payload["times_s"],
        payload["fractions"], tolerance_m=payload["tolerance_m"])
    return {"limit_violations": violations, "limit_unverifiable": unverifiable,
            "tracking": tracking}


def certify_t2(payload: dict[str, Any], *, pool: Any = None) -> Any:
    """The T2 verdict over the emitted curve, or `None` with no geometry.

    Runs where the CALLER stands, not in a worker, so a lent `pool` can carry
    the segment fan-out: a seam's segments are the cost (measured, one
    120-segment seam was 3.9 s of a 4.5 s certification stage) and they are
    independent, where the seam itself is one indivisible verdict.
    """
    if payload["geometry"] is None:
        return None
    return verify(
        payload["cell"], payload["geometry"], payload["waypoints"],
        profile={"limits": payload["limits"]},
        times_s=payload["times_s"], velocities=payload["rates"],
        margin_m=payload["margin_m"], directions=payload["directions"],
        travel_speed_m_s=payload["speeds"], pool=pool)


def certify_cpu(payload: dict[str, Any]) -> dict[str, Any]:
    """The judging half of `Synthesizer.certify`: limits, the tracking
    certificate, and the T2 verdict, for a caller with one seam and no pool."""
    judged = certify_limits_and_tracking(payload)
    judged["verdict"] = certify_t2(payload)
    return judged


def apply_certification(traj: SeamTrajectory,
                        judged: dict[str, Any]) -> SeamTrajectory:
    """`certify_cpu`'s answers, written onto the trajectory they judge."""
    traj.limit_violations = judged["limit_violations"]
    traj.limit_unverifiable = judged["limit_unverifiable"]
    traj.tracking = judged["tracking"]
    if judged["verdict"] is not None:
        # T2 owns collision and weld-travel findings. This stage owns exact
        # rate findings for the emitted M5 Hermite, so compose those surfaces
        # before the native/UI verdict is exposed.
        traj.verdict = replace_limit_findings(
            judged["verdict"],
            judged["limit_violations"],
            judged["limit_unverifiable"],
        )
    return traj


# --------------------------------------------------------------------------
# The wire


def hermite_peak(m0: torch.Tensor, m1: torch.Tensor,
                 chord: torch.Tensor) -> torch.Tensor:
    """The largest speed a cubic Hermite segment reaches, per axis.

    Its derivative is `a u^2 + b u + m0`, a quadratic through the two knot
    velocities whose mean over the segment is the chord -- that mean is what
    makes `a` and `b` follow from `(m0, m1, chord)` alone, with no `h` in them.
    The peak is at an endpoint or at the vertex, and the vertex only counts
    when it falls strictly inside the segment.
    """
    a = 3.0 * (m0 + m1) - 6.0 * chord
    b = -4.0 * m0 - 2.0 * m1 + 6.0 * chord
    ends = torch.maximum(m0.abs(), m1.abs())
    safe = torch.where(a == 0.0, torch.ones_like(a), a)
    u = -b / (2.0 * safe)
    vertex = (m0 - b * b / (4.0 * safe)).abs()
    inside = (a != 0.0) & (u > 0.0) & (u < 1.0)
    return torch.where(inside, torch.maximum(ends, vertex), ends)


def _certified_curve(traj: SeamTrajectory, job: SeamJob
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """`(knots, velocities, times, fractions)` the certificates are computed
    on: the lattice Hermite when the trajectory carries it, else what it
    holds -- a trajectory built by hand, or one whose knots ARE its wire."""
    if traj.lattice_knots is None:
        fractions = job.fractions if traj.fractions is None else traj.fractions
        return (np.asarray(traj.knots, dtype=np.float64),
                np.asarray(traj.knot_velocity, dtype=np.float64),
                np.asarray(traj.times_s, dtype=np.float64),
                np.asarray(fractions, dtype=np.float64))
    return (traj.lattice_knots, traj.lattice_velocity,
            traj.lattice_times_s, traj.lattice_fractions)


def resample_wire(knots: np.ndarray, qdot: np.ndarray, times_s: np.ndarray,
                  fractions: np.ndarray, *, dt_s: float = WIRE_DT_S,
                  min_intervals: int = WIRE_MIN_INTERVALS
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """`(q, qdot, t, f)` at the controller's clock, off the Hermite the knots
    and their velocities define -- the SAME curve the T2 verifier and the
    tracking certificate walk (`verify._hermite`), so what is emitted is
    what was judged.

    `round(T / dt_s)` intervals spread evenly, never fewer than
    `min_intervals`: the spacing is the target, exactly it when the seam's
    clock divides (the lattice's 0.25 s does). Each sample carries the
    cubic's own derivative, which is what makes the wire Hermite data in its
    own right: a Hermite through these samples is this curve to float64, not
    an approximation of it. The fraction is linear in each span's time, the
    rule the certificate interpolates by. The ends are the knots' own data,
    written rather than left to the last bit.
    """
    knots = np.asarray(knots, dtype=np.float64)
    qdot = np.asarray(qdot, dtype=np.float64)
    times_s = np.asarray(times_s, dtype=np.float64)
    fractions = np.asarray(fractions, dtype=np.float64)
    if len(times_s) < 2 or times_s[-1] <= times_s[0]:
        return knots, qdot, times_s, fractions
    duration = float(times_s[-1] - times_s[0])
    count = max(round(duration / dt_s), int(min_intervals)) + 1
    t = np.linspace(times_s[0], times_s[-1], count)
    k = np.clip(np.searchsorted(times_s, t, side="right") - 1, 0, len(times_s) - 2)
    h = (times_s[k + 1] - times_s[k])[:, None]
    u = ((t - times_s[k]) / h[:, 0])[:, None]
    u2, u3 = u * u, u * u * u
    q0, q1, d0, d1 = knots[k], knots[k + 1], qdot[k], qdot[k + 1]
    q = ((2 * u3 - 3 * u2 + 1) * q0 + h * (u3 - 2 * u2 + u) * d0
         + (-2 * u3 + 3 * u2) * q1 + h * (u3 - u2) * d1)
    v = ((6 * u2 - 6 * u) * q0 + h * (3 * u2 - 4 * u + 1) * d0
         + (-6 * u2 + 6 * u) * q1 + h * (3 * u2 - 2 * u) * d1) / h
    f = fractions[k] + (fractions[k + 1] - fractions[k]) * u[:, 0]
    q[0], q[-1] = knots[0], knots[-1]
    v[0], v[-1] = qdot[0], qdot[-1]
    f[0], f[-1] = fractions[0], fractions[-1]
    return q, v, t, f


# --------------------------------------------------------------------------
# The loop's pieces


@torch.no_grad()
def _forget_moments(opt: torch.optim.Optimizer, x: torch.Tensor,
                    rejected: torch.Tensor) -> None:
    """Zero Adam's running moments for the candidates whose step was undone.

    The unbatched loop threw the whole optimizer away on a reject, for the
    reason that survives batching: after a revert the moments describe a path
    the iterate did not take, and letting them drive the next step is how a
    backtrack turns into a lurch. Batched, only the rejected ROWS may be
    cleared -- rebuilding the optimizer would flush the candidates that are
    making progress.
    """
    state = opt.state.get(x)
    if not state:
        return
    mask = rejected.view(-1, 1, 1)
    for key in ("exp_avg", "exp_avg_sq", "max_exp_avg_sq"):
        moment = state.get(key)
        if moment is not None:
            moment.masked_fill_(mask, 0.0)


def _central_difference(values: torch.Tensor, dt: torch.Tensor,
                        lengths: torch.Tensor) -> torch.Tensor:
    """d/dt along the KNOT axis of `(B, L, ...)`, one-sided at each row's ends.

    Written once because the knot velocities and the target's own rate are the
    same arithmetic, and because the axis moved when the candidates were
    batched -- two hand-inlined copies would have been two chances to keep
    differencing over candidates instead of over time.

    `lengths` is what makes it safe once rows are padded. A row's last REAL
    knot is at `n-1`, not at `L-1`, and left to the generic central rule it
    would difference against the pad -- returning half the true one-sided
    velocity at exactly the knot where the trajectory ends.
    """
    # Every slice keeps its knot dimension so the per-row `dt` of shape
    # (B, 1, 1) broadcasts against it. Dropping to `values[:, 0]` makes the
    # difference (B, C) and the division then broadcasts the BATCH axis
    # against the channel axis, silently producing a (B, B, C) tensor.
    out = torch.zeros_like(values)
    out[:, 1:-1] = (values[:, 2:] - values[:, :-2]) / (2 * dt)
    out[:, 0:1] = (values[:, 1:2] - values[:, 0:1]) / dt

    trailing = torch.zeros_like(values)
    trailing[:, 1:] = (values[:, 1:] - values[:, :-1]) / dt
    index = torch.arange(values.shape[1], device=values.device).unsqueeze(0)
    out = torch.where((index == (lengths - 1).unsqueeze(1)).unsqueeze(-1),
                      trailing, out)
    # The pad does not move, so nothing there has a rate.
    return out * (index < lengths.unsqueeze(1)).unsqueeze(-1)


def _align_rotation(current_axis: torch.Tensor, target_axis: torch.Tensor) -> torch.Tensor:
    """(B, 3, 3) minimal rotation carrying `current_axis` onto `target_axis`.

    The 5-DOF trick: composing this with the CURRENT tool rotation yields a
    6-DOF target whose rotation-about-the-electrode error is zero by
    construction -- so a plain 6-DOF DLS solve projects onto the seam while
    leaving spin exactly where the optimizer put it. Spin is the redundancy;
    the projection must not spend it.
    """
    a = current_axis / current_axis.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    b = target_axis / target_axis.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    v = torch.linalg.cross(a, b, dim=-1)
    c = (a * b).sum(-1, keepdim=True).unsqueeze(-1)
    zero = torch.zeros_like(v[..., 0])
    k = torch.stack([
        torch.stack([zero, -v[..., 2], v[..., 1]], dim=-1),
        torch.stack([v[..., 2], zero, -v[..., 0]], dim=-1),
        torch.stack([-v[..., 1], v[..., 0], zero], dim=-1),
    ], dim=-2)
    eye = torch.eye(3, dtype=a.dtype, device=a.device).expand_as(k)
    # Rodrigues with the (1 - c) / |v|^2 form; safe at c -> 1 (identity) and
    # undefined only at c = -1, a half-turn the projection never asks for.
    denom = (v.norm(dim=-1).pow(2)).clamp_min(1e-12).unsqueeze(-1).unsqueeze(-1)
    return eye + k + (k @ k) * (1.0 - c) / denom


if __name__ == "__main__":
    raise SystemExit(
        "the CLI moved with the stage: python -m "
        "weld_motion_planner.planner.trajectory_optimization.weld_trajopt_main")
