#!/usr/bin/env python
"""The lattice and the search: what they cost, and what each fix bought.

M4's two stages on the real corpus. The question this answers is not "is it
fast" but "where does the time go and what happens when the grid gets finer",
because the grid is the one knob that has to move when the edge threshold
tightens -- `docs/issues/0004` -- and every gain below was found by asking that.

Four measurements:

- **Stages**, so the next optimisation goes where the time is rather than where
  it is easy. The answer has been "inverse kinematics" every time.
- **Scaling in bins**, which is quadratic in the poses screened and would have
  been quartic in the search's memory before it was chunked.
- **What each fix bought**, run with the fix off where a flag allows it and
  cited from its commit where not. A speedup nobody can reproduce is a rumour.
- **The whole corpus** at the shipped configuration, which is the number that
  belongs in an end-to-end budget.

Run: `pixi run -e motion python tools/bench_graph.py`.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "motion"


def _clock(torch):
    torch.cuda.synchronize()
    return time.perf_counter()


def _seam(read, Seam, torch, stem: str, index: int = 0):
    plan = read(DATA / f"{stem}.weldplan")
    joints = {str(j.get("id")): j for j in (plan.program.get("weld_joints") or [])}
    spec = plan.seams[index]
    return plan, Seam(spec, joints.get(str(spec.get("weld_joint_ref"))),
                      device=torch.device("cuda"))


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--threshold-deg", type=float, default=25.0)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args(argv)

    import torch

    if not torch.cuda.is_available():
        print("bench-graph: needs CUDA")
        return 0

    from weld_motion_planner.geometry.seam_frame import Seam
    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.planner.dp_seam_search.engine import graph as graphmod
    from weld_motion_planner.planner.dp_seam_search.engine import nodes
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix
    from weld_motion_planner.robot_cell.robot_cell import RobotCell

    limit = np.deg2rad(args.threshold_deg)
    device = torch.cuda.get_device_properties(0)
    print(f"{device.name}, {device.total_memory / 2**30:.1f} GiB, "
          f"{device.multi_processor_count} SMs")
    print(f"{args.samples} samples, {args.threshold_deg:.0f} deg edge threshold, "
          f"median of {args.repeats}\n")

    plan, seam = _seam(read, Seam, torch, "dog_box")
    place = placement_matrix(plan.placement)
    common = dict(samples=args.samples, work_to_piece=place)

    def run(bins, **extra):
        kw = {**common, "spin_bins": bins, "positioner_bins": bins, **extra}
        nodes.build(RobotCell.of(plan.cell), seam, **kw)              # warm
        builds, searches, peaks = [], [], []
        for _ in range(args.repeats):
            torch.cuda.empty_cache()
            torch.cuda.reset_peak_memory_stats()
            t0 = _clock(torch)
            lattice = nodes.build(RobotCell.of(plan.cell), seam, **kw)
            t1 = _clock(torch)
            result = graphmod.search(lattice, threshold_rad=limit, k_best=2)
            t2 = _clock(torch)
            builds.append(t1 - t0)
            searches.append(t2 - t1)
            peaks.append(torch.cuda.max_memory_allocated() / 2**30)
        return (np.median(builds), np.median(searches), max(peaks),
                lattice, result)

    print("## Bins")
    print(f"{'bins':>6}{'gap':>8}{'poses':>10}{'live/layer':>12}"
          f"{'build':>9}{'search':>9}{'peak':>10}{'crossed':>9}")
    for bins in (15, 32, 64, 96, 128):
        build_s, search_s, peak, lattice, result = run(bins)
        poses = args.samples * bins * bins
        print(f"{bins:>6}{360 / bins:>7.1f}°{poses:>10,}"
              f"{int(lattice.per_layer().max()):>12,}"
              f"{build_s:>8.2f}s{search_s:>8.3f}s{peak:>7.2f} GiB"
              f"{('yes' if result.crossed else 'NO'):>9}")

    print("\n## What each fix bought, at 64 bins")
    base_build, base_search, base_peak, _, _ = run(64)
    rows = [("shipped", base_build, base_search, base_peak)]
    for label, extra in (("float64 screen", {"dtype": torch.float64}),
                         ("one transition slice", {})):
        if label == "one transition slice":
            kw = {**common, "spin_bins": 64, "positioner_bins": 64}
            lattice = nodes.build(RobotCell.of(plan.cell), seam, **kw)
            graphmod.search(lattice, threshold_rad=limit, k_best=2, chunk_bytes=1 << 45)
            ts, pk = [], []
            for _ in range(args.repeats):
                torch.cuda.reset_peak_memory_stats()
                held = torch.cuda.memory_allocated()
                t0 = _clock(torch)
                graphmod.search(lattice, threshold_rad=limit, k_best=2,
                                chunk_bytes=1 << 45)
                ts.append(_clock(torch) - t0)
                pk.append((torch.cuda.max_memory_allocated() - held) / 2**30)
            rows.append((label, base_build, float(np.median(ts)), max(pk)))
            continue
        if label == "float64 screen":
            plan64, seam64 = _seam(read, Seam, torch, "dog_box")
            kw = {**common, "spin_bins": 64, "positioner_bins": 64, **extra}
            nodes.build(plan64.cell, seam64, **kw)
            ts = []
            for _ in range(args.repeats):
                t0 = _clock(torch)
                nodes.build(plan64.cell, seam64, **kw)
                ts.append(_clock(torch) - t0)
            rows.append((label, float(np.median(ts)), base_search, float("nan")))
            continue
        b, s, p, _, _ = run(64, **extra)
        rows.append((label, b, s, p))
    print(f"{'':>22}{'build':>9}{'search':>9}{'peak':>10}{'vs shipped':>12}")
    for label, b, s, p in rows:
        total = b + s
        ratio = "" if label == "shipped" else f"{total / (base_build + base_search):.2f}x"
        peak = "" if not np.isfinite(p) else f"{p:.2f} GiB"
        print(f"{label:>22}{b:>8.2f}s{s:>8.3f}s{peak:>10}{ratio:>12}")

    print("\n## Transition chunk size, at 96 bins")
    kw = {**common, "spin_bins": 96, "positioner_bins": 96}
    lattice = nodes.build(RobotCell.of(plan.cell), seam, **kw)
    live = int(lattice.per_layer().max())
    print(f"{'budget':>10}{'slices':>8}{'elements/slice':>17}{'time':>9}{'peak':>10}")
    for name, budget in (("1 MiB", 1 << 20), ("64 MiB", 64 << 20),
                         ("256 MiB", 256 << 20), ("1 GiB", 1 << 30),
                         ("one slice", 1 << 45)):
        wide = min(live, max(1, budget // (live * 152)))
        try:
            graphmod.search(lattice, threshold_rad=limit, k_best=2, chunk_bytes=budget)
            ts, pk = [], []
            for _ in range(args.repeats):
                torch.cuda.reset_peak_memory_stats()
                held = torch.cuda.memory_allocated()
                t0 = _clock(torch)
                graphmod.search(lattice, threshold_rad=limit, k_best=2, chunk_bytes=budget)
                ts.append(_clock(torch) - t0)
                pk.append((torch.cuda.max_memory_allocated() - held) / 2**30)
        except torch.cuda.OutOfMemoryError:
            # Not a failure of the bench: the unsliced row is the one this
            # whole column exists to replace, and it stopped fitting when the
            # closed form doubled the live-node count. Reported as the wall it
            # is rather than crashing the run.
            torch.cuda.empty_cache()
            print(f"{name:>10}{-(-live // wide):>8}{live * wide * 7:>17,}"
                  f"{'out of memory':>17}")
            continue
        print(f"{name:>10}{-(-live // wide):>8}{live * wide * 7:>17,}"
              f"{np.median(ts):>8.3f}s{max(pk):>7.2f} GiB")
    del lattice
    torch.cuda.empty_cache()

    print("\n## The whole corpus, shipped configuration")
    bins = graphmod.bins_for(limit)
    crossed, failed, total_build, total_search, worst = 0, [], 0.0, 0.0, 0.0
    for path in sorted(DATA.glob("*.weldplan")):
        each = read(path)
        joints = {str(j.get("id")): j for j in (each.program.get("weld_joints") or [])}
        matrix = placement_matrix(each.placement)
        for spec in each.seams[:3]:
            one = Seam(spec, joints.get(str(spec.get("weld_joint_ref"))),
                       device=torch.device("cuda"))
            t0 = _clock(torch)
            lattice = nodes.build(each.cell, one, samples=args.samples,
                                  spin_bins=bins, positioner_bins=bins,
                                  work_to_piece=matrix)
            t1 = _clock(torch)
            result = graphmod.search(lattice, threshold_rad=limit, k_best=2)
            t2 = _clock(torch)
            total_build += t1 - t0
            total_search += t2 - t1
            worst = max(worst, t2 - t0)
            if result.crossed:
                crossed += 1
            else:
                failed.append(f"{path.stem}/{spec['id']}@{result.frontier}")
    n = crossed + len(failed)
    print(f"{n} seams at {bins} bins: {crossed} crossed"
          + (f", failures {failed}" if failed else ""))
    print(f"  build {total_build:.2f}s, search {total_search:.2f}s, "
          f"total {total_build + total_search:.2f}s")
    print(f"  {(total_build + total_search) / n * 1000:.0f} ms a seam on average, "
          f"{worst * 1000:.0f} ms worst")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
