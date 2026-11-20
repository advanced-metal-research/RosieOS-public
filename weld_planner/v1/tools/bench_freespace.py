#!/usr/bin/env python
"""M6 join, timed: the number the batching work is measured against.

Runs the same pipeline as `freespace.main` -- M4+M5 upstream at k=1, roadmap,
rewind, connect -- but with a CUDA-synced stopwatch around each phase and the
verifier OFF, because the budget under measurement is the planning itself
(the verdicts are downstream judging and are timed elsewhere). `connect` is
the number that matters; the rest is printed so a regression in the setup
cannot hide inside it.

Run: `pixi run -e motion python tools/bench_freespace.py data/motion/bracket_a_2x.weldplan`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
from weld_motion_planner.planner.planner_main import MotionPlanningOptions, plan_motion
from weld_motion_planner.planner.trajectory_optimization.engine import (
    roadmap as roadmap_module,
)
from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
    connect,
    rewound,
    winding_reach,
)
from weld_motion_planner.planner.trajectory_optimization.options import (
    WeldTrajoptOptions,
)
from weld_motion_planner.robot_cell.collision import model as collision_model
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.utils.timing import Stopwatch, cuda_sync


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--seams", type=int, default=0,
                        help="how many seams to join; 0 means every one the sweep crossed")
    args = parser.parse_args(argv)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    watch = Stopwatch(sync=cuda_sync(device))

    request = read_request(args.request)
    with watch.stage("upstream (M4+M5, k=1)"):
        reports = plan_motion(
            request,
            MotionPlanningOptions(run_weld_trajopt=True,
                                  run_connecting_trajopt=False,
                                  seam_search=SeamSearchOptions(k_best=1),
                                  weld_trajopt=WeldTrajoptOptions(verify=False)),
            device=device).weld_trajectories
        if args.seams:
            reports = reports[: args.seams]
    cell = RobotCell.of(request.cell, device=device)
    with watch.stage("collision model"):
        planner_collision_model = collision_model.for_planner(request, cell)
    held = dict(cell.profile.held)
    home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])

    with watch.stage("roadmap"):
        graph = roadmap_module.build(
            request, planner_collision_model, device, held=held,
            focus=winding_reach(request, reports, home, cell=cell), cell=cell)
    with watch.stage("rewound"):
        reports = rewound(request, reports, home, roadmap=graph,
                          planner_collision_model=planner_collision_model,
                          device=device, held=held, cell=cell)
    with watch.stage("connect"):
        moves = connect(request, reports,
                        planner_collision_model=planner_collision_model,
                        device=device, roadmap=graph,
                        verifier_collision_model=None, cell=cell)

    for move in moves:
        print(f"{move.kind:9s} {move.from_id or 'reset'!s:11s} -> "
              f"{move.to_id or 'reset'!s:11s} {len(move.knots):3d} knots  "
              f"{move.duration_before_s:5.2f} -> {move.duration_s:5.2f} s  "
              f"env {move.env_clearance_m * 1000:6.1f} mm  "
              f"self {move.self_clearance_m * 1000:6.1f} mm  "
              f"seed {move.seed}")
    print(f"\n{len(reports)} seams, {len(moves)} connecting motions")
    for name, ms in watch.ms(total=True).items():
        print(f"  {name:24s} {ms / 1000.0:8.2f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
