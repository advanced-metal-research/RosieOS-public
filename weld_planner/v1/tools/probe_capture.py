#!/usr/bin/env python
"""Can the M6 inner step be captured as a CUDA graph, and what blocks it?

`BENCHMARKS.md` 11.3b probed capture for M5 and found two blockers: every
cuSOLVER path, and a host copy inside the collision term. M6's step has no
solve in it, so after the copy fixes (`esdf._fused_launch_args`,
`scene._self_gap_launch_args`) capture should be within reach -- this probes
it on the real cell rather than assuming, and stays in the tree so the next
person re-probes instead of re-guessing.

Three stages, coarse to fine: capture the whole `MoveSolver._step`, and when
that fails, capture the forward cost alone, then the collision term alone --
so the error names a piece rather than a step. A successful capture is
replayed and timed against the eager step.

The solver under probe is a synthetic two-motion batch from the cell's reset
pose, so nothing upstream (M4, M5, roadmap) runs: the step's arithmetic does
not care whether the endpoints came from a weld.

Run: `pixi run -e motion python tools/probe_capture.py data/motion/bracket_a_2x.weldplan`.
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import torch
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
    ADAM_LR,
    MoveSolver,
    MoveSpec,
)
from weld_motion_planner.robot_cell.collision import model as collision_model
from weld_motion_planner.robot_cell.robot_cell import RobotCell


def attempt(label: str, build) -> bool:
    """Warm up on a side stream, capture, replay once. Reports, never raises."""
    fn = build()
    stream = torch.cuda.Stream()
    stream.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(stream):
        for _ in range(3):
            fn()
    torch.cuda.current_stream().wait_stream(stream)
    graph = torch.cuda.CUDAGraph()
    try:
        with torch.cuda.graph(graph):
            fn()
        graph.replay()
        torch.cuda.synchronize()
    except Exception as error:  # noqa: BLE001 -- the point is the message
        torch.cuda.synchronize()
        print(f"  {label:24s} BLOCKED   {type(error).__name__}: {error}")
        if "-v" in sys.argv:
            traceback.print_exc()
        return False
    print(f"  {label:24s} captures")
    return graph


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--steps", type=int, default=100,
                        help="replay count for the timing comparison")
    args = parser.parse_args([a for a in argv if a != "-v"])

    if not torch.cuda.is_available():
        print("no CUDA device; there is nothing to probe")
        return 1
    device = torch.device("cuda")
    request = read_request(args.request)
    cell = RobotCell.of(request.cell, device=device)
    scene = collision_model.for_planner(request, cell)
    held = dict(cell.profile.held)
    home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])
    away = home + np.linspace(0.2, -0.2, len(home))
    rest = np.zeros_like(home)

    specs = [MoveSpec("transit", None, None, home, rest, away, rest),
             MoveSpec("transit", None, None, away, rest, home, rest)]
    solver = MoveSolver(request, scene, device, moves=specs, held=held, cell=cell)
    collision = solver.collision_cost()

    print("pieces first, so a failure above names one:")
    attempt("smoothness", lambda: (lambda: solver.smoothness(solver.x).sum()))
    attempt("collision.value", lambda: (lambda: collision.value(
        solver._full_q(solver._sampled(solver.x.detach(),
                                       solver.velocities(solver.x.detach())))).sum()))
    attempt("cost fwd+bwd", lambda: (lambda: (
        solver.x.grad is not None and solver.x.grad.zero_(),
        solver._cost(collision).sum().backward())[-1]))

    print("the whole step:")
    opt = torch.optim.Adam([solver.x], lr=ADAM_LR, capturable=True)
    scale = torch.ones(solver.B, 1, 1, dtype=torch.float64, device=device)
    with torch.no_grad():
        best = solver._cost(collision).clone()

    graph = attempt("MoveSolver._step",
                    lambda: (lambda: solver._step(opt, collision, scale, best)))
    if not graph:
        return 1

    torch.cuda.synchronize()
    tick = time.perf_counter()
    for _ in range(args.steps):
        solver._step(opt, collision, scale, best)
    torch.cuda.synchronize()
    eager = (time.perf_counter() - tick) / args.steps

    tick = time.perf_counter()
    for _ in range(args.steps):
        graph.replay()
    torch.cuda.synchronize()
    replay = (time.perf_counter() - tick) / args.steps
    print(f"\n  eager  {eager * 1000:7.2f} ms/step")
    print(f"  replay {replay * 1000:7.2f} ms/step   ({eager / max(replay, 1e-9):.1f}x)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
