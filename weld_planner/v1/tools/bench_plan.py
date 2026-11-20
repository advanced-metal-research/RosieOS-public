#!/usr/bin/env python
"""The standing benchmark report: one COLD run, then warm runs, averaged.

One number per stage is how a regression hides: the first run of a process
pays capture warm-ups, warp JIT and per-batch-shape FK graph recordings that
no later run pays -- measured, a 22-seam request's `dp_seam_search` read
2,817 ms cold-of-shapes and ~600 ms warm -- so a single-run figure is
whichever of those the runner happened to take. This prints both, and the
warm side as mean over several runs with the spread, because a stage that
wobbles is a finding too.

Substages come from wrapping the solvers' own entry points in-process, and
the verification share from a warm verify-off pass subtracted from the warm
mean. Run it through pixi, like everything else:

    pixi run -e motion python tools/bench_plan.py data/motion/bracket_b_2x.weldplan
    pixi run -e motion python tools/bench_plan.py plan_a.weldplan plan_b.weldplan --runs 5
"""

from __future__ import annotations

import argparse
import collections
import statistics
import sys
import time
from pathlib import Path

import numpy as np  # noqa: F401  -- imported for side-effect parity with the server
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "motion_planner"))


def _clock(fn):
    torch.cuda.synchronize()
    start = time.perf_counter()
    out = fn()
    torch.cuda.synchronize()
    return (time.perf_counter() - start) * 1000.0, out


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plans", nargs="+", help=".weldplan files to benchmark")
    parser.add_argument("--runs", type=int, default=5, help="warm runs to average")
    parser.add_argument("--k-best", type=int, default=5)
    args = parser.parse_args(argv)

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.planner.trajectory_optimization.engine import (
        control_solver,
        freespace,
        weld_trajopt,
    )
    from weld_motion_planner.robot_cell.collision import model as collision_model
    from weld_motion_planner.robot_cell.robot_cell import RobotCell
    from weld_motion_planner.verifier import model as verifier_model
    from pipeline import search_then_plan_weld_trajectories

    device = torch.device("cuda")

    # In-process substage taps. `_judge` stays module-level and unwrapped, so
    # the verification pool's pickling never sees a closure.
    taps = collections.Counter()

    def wrap(obj, name, label):
        orig = getattr(obj, name)

        def timed(*a, **k):
            torch.cuda.synchronize()
            start = time.perf_counter()
            out = orig(*a, **k)
            torch.cuda.synchronize()
            taps[label] += (time.perf_counter() - start) * 1000.0
            return out

        setattr(obj, name, timed)

    synthesizer = next(c for c in vars(weld_trajopt).values()
                       if isinstance(c, type) and hasattr(c, "run")
                       and hasattr(c, "_reports"))
    wrap(synthesizer, "run", "weld:descent")
    wrap(synthesizer, "_capture", "weld:capture")
    wrap(control_solver.ControlSolver, "run", "conn:descent")
    wrap(control_solver.ControlSolver, "_capture", "conn:capture")
    wrap(freespace, "standoff_batch", "conn:standoff")

    for path in args.plans:
        request = read(path)
        cell = RobotCell.of(request.cell, device=device)

        def stage_pass(verify: bool):
            taps.clear()
            times: dict[str, float] = {}
            times["collision_model"], scene = _clock(
                lambda: collision_model.for_planner(request, cell))
            vcm = verifier_model.for_verifier(request) if verify else None
            times["weld_trajopt(M4+M5)"], result = _clock(
                lambda: search_then_plan_weld_trajectories(
                    request, k_best=args.k_best, verify=verify, device=device))
            times["connecting_trajopt"], _ = _clock(
                lambda: freespace.connect(
                    request, result.trajectories,
                    planner_collision_model=scene, device=device,
                    verifier_collision_model=vcm, cell=cell))
            times["total"] = sum(v for k, v in times.items() if k != "total")
            return times, dict(taps)

        cold, cold_taps = stage_pass(verify=True)
        warms, warm_taps = [], []
        for _ in range(max(1, args.runs)):
            t, sub = stage_pass(verify=True)
            warms.append(t)
            warm_taps.append(sub)
        no_verify, _ = stage_pass(verify=False)

        def mean(key, rows):
            return statistics.mean(r.get(key, 0.0) for r in rows)

        def spread(key, rows):
            values = [r.get(key, 0.0) for r in rows]
            return (max(values) - min(values)) / 2.0

        name = Path(path).name
        print(f"\n{'=' * 66}\n{name}  --  cold 1 + warm {len(warms)} (mean +/- half-range), ms")
        print(f"{'stage':30} {'cold':>9} {'warm':>9} {'+/-':>7}")
        order = ["total", "weld_trajopt(M4+M5)", "connecting_trajopt",
                 "collision_model"]
        for key in order:
            print(f"  {key:28} {cold[key]:9.0f} {mean(key, warms):9.0f} "
                  f"{spread(key, warms):7.0f}")
        weld_verify = mean("weld_trajopt(M4+M5)", warms) - no_verify["weld_trajopt(M4+M5)"]
        conn_verify = mean("connecting_trajopt", warms) - no_verify["connecting_trajopt"]
        print("  substages (warm):")
        for label in ("weld:descent", "weld:capture", "conn:descent",
                      "conn:capture", "conn:standoff"):
            print(f"    {label:26} {cold_taps.get(label, 0.0):9.0f} "
                  f"{mean(label, warm_taps):9.0f} {spread(label, warm_taps):7.0f}")
        print(f"    {'weld:verify (delta)':26} {'':9} {weld_verify:9.0f}")
        print(f"    {'conn:verify (delta)':26} {'':9} {conn_verify:9.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
