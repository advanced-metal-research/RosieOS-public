#!/usr/bin/env python
"""M5's stage: the weld synthesis engine, wired into the pipeline.

The engine lives in `engine/weld_trajopt.py` and knows nothing about the
pipeline; this file is what `planner_main.py` calls, and everything in it is
plumbing -- options in, models handed over, the winner certified. Read the
engine for how a seam becomes a curve.

Run: `pixi run -e motion python -m weld_motion_planner.planner.trajectory_optimization.weld_trajopt_main <request.weldplan>`.
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry.seam_frame import Seam
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.planner.base_types import PlannerContext, PlannerModule
from weld_motion_planner.planner.trajectory_optimization.engine.weld_trajopt import (
    K_BEST,
    SWEEP_STEPS,
    VERIFY_MARGIN_M,
    SeamJob,
    SeamTrajectory,
    SynthesisRun,
    Synthesizer,
    apply_certification,
    certify_cpu,
    certify_limits_and_tracking,
    certify_t2,
)
from weld_motion_planner.planner.trajectory_optimization.options import (
    WeldTrajoptOptions,
    WeldTrajoptResult,
)
from weld_motion_planner.robot_cell.collision.cost import CollisionCost

# Per-segment T2 cost varies 6x across the corpus -- ~5 ms on a plate seam,
# ~33 ms on `hitch_2x`'s curved seam close to the part -- so segment count
# alone cannot name the outlier that will pin the pool. What it can do is
# bound the loss: below this, even at the expensive rate a whole seam stays
# near the pool's other jobs, and splitting was MEASURED a loss (ten
# 65-segment plate seams split cost ~1 s over leaving them whole); above it,
# splitting won 1.6-2.4 s on the seams that had the expensive rate and was
# noise on the ones that did not.
SPLIT_SEGMENTS = 100


class WeldTrajoptModule(PlannerModule):
    """M5: each weld smoothed from the sweep's samples into a continuous curve.

    `upstream` is M4's `SeamSearchResult`. This synthesizes every candidate of
    every seam and picks each seam's winner by M5's own objective, looking at
    that seam alone -- it cannot prefer a candidate because it would shorten
    the transit to the NEXT seam. A caller that wants to choose with the whole
    plan in view calls `plan_weld_candidates`, which does the same synthesis
    and returns all the candidates with no winner picked.
    """

    name = "weld_trajopt"
    Options = WeldTrajoptOptions
    Result = WeldTrajoptResult

    def plan(self, request: PlanRequest, upstream: Any,
             options: WeldTrajoptOptions,
             context: PlannerContext) -> WeldTrajoptResult:
        run = plan_weld_candidates(
            request, upstream, options,
            planner_collision_model=context.planner_collision_model,
            verifier_collision_model=context.verifier_collision_model,
            device=context.device, cell=context.cell)
        winners: list[SeamTrajectory] = []
        payloads: list[dict[str, Any]] = []
        for index, candidates in enumerate(run.candidates):
            winner = candidates[0]
            winner.rejected = [
                {"sweep_rank": other.sweep_rank,
                 "sweep_cost_rad": round(other.sweep_cost_rad, 4),
                 "objective": round(other.objective, 3),
                 "midpoint_tracking_mm": round(other.midpoint_tracking_m * 1000.0, 3),
                 "acc_ratio": round(other.acc_ratio_after, 3)}
                for other in candidates[1:]]
            winners.append(winner)
            payloads.append(run.synthesizer.certify_payload(
                run.jobs[index], winner,
                verifier_collision_model=run.verifier_collision_model,
                verify_margin_m=options.verify_margin_m))
        # ONE pool for every winner's judging. Certification is serial CPU --
        # the tracking bisection and the T2 walk -- and it was more than half
        # the stage: the seams are independent, so they fan out, and the GPU
        # half of the prep (`world_tangents`) already ran in `certify_payload`.
        #
        # A LONG seam's T2 fans out by segment chunk rather than by seam.
        # Fanned by seam alone, the wall time is the one long seam --
        # measured, a 120-segment seam held 3.9 s of a 4.5 s stage while
        # eighteen finished ones idled -- so a seam past `SPLIT_SEGMENTS`
        # sends its tracking to a worker (geometry stripped: that job never
        # opens it) and verifies HERE, lending the pool its segments in
        # `POOL_TASKS` slices; the thread only orchestrates. A short seam
        # stays ONE worker job, tracking and verdict together: splitting
        # those too was measured 0.8-1.5 s slower on a 22-short-seam plan,
        # the tracking jobs filling the pool ahead of every verdict.
        if len(payloads) > 1:
            split = [len(p["waypoints"]) - 1 >= SPLIT_SEGMENTS for p in payloads]
            with ProcessPoolExecutor(
                    max_workers=min(len(payloads), os.cpu_count() or 1)) as judges:
                whole = {i: judges.submit(certify_cpu, payload)
                         for i, payload in enumerate(payloads) if not split[i]}
                tracked = {i: judges.submit(certify_limits_and_tracking,
                                            {**payload, "geometry": None})
                           for i, payload in enumerate(payloads) if split[i]}
                with ThreadPoolExecutor(max_workers=max(1, sum(split))) as t2:
                    lent = {i: t2.submit(certify_t2, payload, pool=judges)
                            for i, payload in enumerate(payloads) if split[i]}
                judged = [{**tracked[i].result(), "verdict": lent[i].result()}
                          if split[i] else whole[i].result()
                          for i in range(len(payloads))]
        else:
            judged = [certify_cpu(payload) for payload in payloads]
        return WeldTrajoptResult(trajectories=[
            apply_certification(winner, answer)
            for winner, answer in zip(winners, judged, strict=True)])


def plan_weld_candidates(request: PlanRequest, result: Any,
                         options: WeldTrajoptOptions | None = None, *,
                         planner_collision_model: Any = None,
                         verifier_collision_model: Any = None,
                         device: Any = None, cell: Any = None) -> SynthesisRun:
    """Every seam's every candidate, synthesized, ranked, and left unjudged.

    The two models are HANDED IN, never built here. A flag still outranks one:
    `avoid_collisions=False` that weighted collision anyway would be worse than
    the honest cost.
    """
    if options is None:
        options = WeldTrajoptOptions()
    if device is None:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    planner_collision_model = planner_collision_model if options.avoid_collisions else None
    exact = verifier_collision_model if options.verify else None
    joints = request.joints_by_id

    # EVERY seam and EVERY candidate in one tensor. Rows share a cell and
    # nothing else; each seam's curve, clock, fractions and tolerance ride
    # along per row. Seven seams run separately cost seven times the kernel
    # launches for the same arithmetic (`BENCHMARKS.md` 11.3).
    jobs: list[SeamJob] = []
    planned_for: list[Any] = []
    for planned in result.seams:
        if options.only_seam and planned.id != options.only_seam:
            continue
        if not planned.candidates:
            continue
        spec = next(s for s in request.seams if str(s.get("id")) == planned.id)
        joint = joints.get(str(spec.get("weld_joint_ref")))
        jobs.append(SeamJob(
            seam=Seam(spec, joint, device=device), spec=spec, joint=joint,
            candidates_rad=np.stack([c.joints_rad for c in planned.candidates]),
            times_s=planned.candidates[0].times_s,
            fractions=planned.fractions, held=planned.held))
        planned_for.append(planned)
    if not jobs:
        return SynthesisRun(seam_ids=[], candidates=[],
                            synthesizer=None, jobs=[])  # type: ignore[arg-type]

    synth = Synthesizer(request, jobs, planner_collision_model, device, cell=cell)
    collision = None
    if planner_collision_model is not None:
        # MARGIN ZERO, deliberately unlike the cost module's default. Its
        # 50 mm margin exists to give descent a gradient before contact when
        # diving into free space; a weld holds the gun a few millimetres from
        # the work BY DEFINITION, so any positive margin reads the weld
        # itself as a violation and the AL spends the whole loop fighting the
        # projection -- measured: it took a seam with 0.05 mm midpoint error
        # to 6.7 mm while "improving" it. At zero, the multipliers stay silent
        # until something actually penetrates, which is the guard synthesis
        # wants.
        #
        # The batch is one pose per SAMPLE of the padded cost grid, so every
        # trajectory owns its own multipliers and no two are pooled; and the
        # read is swept along that grid, free space's term (`SWEEP_STEPS`).
        collision = CollisionCost(scene=planner_collision_model,
                                  batch=synth.B * synth.S,
                                  margin_m=0.0, self_margin_m=0.0,
                                  sweep_samples=synth.S, sweep_steps=SWEEP_STEPS)
    produced = synth.run(collision, inner_steps=options.inner_steps,
                         outer_steps=options.outer_steps)

    pools: list[list[SeamTrajectory]] = []
    for index, planned in enumerate(planned_for):
        run = produced[index]
        for traj, candidate in zip(run, planned.candidates, strict=True):
            traj.sweep_cost_rad = float(candidate.cost_rad)
        # A candidate that PENETRATES loses to every one that does not,
        # whatever its objective says -- the smoothness terms are a
        # preference and a clearance below zero is a fact. Otherwise the
        # order is this stage's own objective.
        run.sort(key=lambda r: (r.clearance_after_m < 0.0, r.objective))
        pools.append(run)
    return SynthesisRun(seam_ids=[p.id for p in planned_for], candidates=pools,
                        synthesizer=synth, jobs=jobs, verifier_collision_model=exact)


# --------------------------------------------------------------------------
# CLI


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--seam", default=None)
    parser.add_argument("--no-collide", action="store_true")
    parser.add_argument("--no-verify", action="store_true",
                        help="skip the T2 certificate over the emitted curve")
    parser.add_argument("--verify-margin-mm", type=float,
                        default=VERIFY_MARGIN_M * 1000.0,
                        help="the verifier's safety pad; 0 certifies non-penetration alone")
    parser.add_argument("-k", "--k-best", type=int, default=K_BEST,
                        help="how many of the sweep's ways across to re-rank")
    args = parser.parse_args(argv)

    # M4 then M5 is `plan_motion`'s job, asked for by flags; this CLI only
    # renders M5's half of the result.
    from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
    from weld_motion_planner.planner.planner_main import (
        MotionPlanningOptions,
        plan_motion,
    )

    result = plan_motion(
        read_request(args.request),
        MotionPlanningOptions(
            run_weld_trajopt=True, run_connecting_trajopt=False,
            seam_search=SeamSearchOptions(
                k_best=max(1, args.k_best),
                screen_collisions=not args.no_collide),
            weld_trajopt=WeldTrajoptOptions(
                avoid_collisions=not args.no_collide,
                verify=not args.no_verify,
                verify_margin_m=args.verify_margin_mm / 1000.0,
                only_seam=args.seam))).weld_trajopt_result
    for r in result.trajectories:
        acc = f"acc {r.acc_ratio_before:.2f} -> {r.acc_ratio_after:.2f} of limit"
        track = (f"track: knots {r.knot_tracking_m * 1e6:.1f} um, "
                 f"midpoints {r.midpoint_tracking_m * 1e3:.3f} mm")
        broken = sum(1 for v in r.limit_violations if v["kind"] != "position")
        if r.chosen_from > 1:
            moved = "" if r.sweep_rank == 0 else "   <-- the sweep ranked it below"
            print(f"{r.seam_id}: chose sweep candidate {r.sweep_rank} of "
                  f"{r.chosen_from}, objective {r.objective:.2f}{moved}")
            for other in r.rejected:
                print(f"      rejected #{other['sweep_rank']}: objective "
                      f"{other['objective']}, midpoints "
                      f"{other['midpoint_tracking_mm']} mm, acc "
                      f"{other['acc_ratio']} of limit")
        print(f"{r.seam_id}: cost {r.cost_before:.2f} -> {r.cost_after:.2f}, {acc}, "
              f"{track}, clearance {r.clearance_before_m * 1e3:.1f} -> "
              f"{r.clearance_after_m * 1e3:.1f} mm, rate violations {broken}")
        if r.tracking is not None:
            print(f"    tracking certificate: {r.tracking.summary()}")
        if r.verdict is not None:
            print(f"    T2 verifier over the curve: {r.verdict.summary()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
