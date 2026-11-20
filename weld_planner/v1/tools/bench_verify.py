#!/usr/bin/env python
"""The T2 verifier on the real cell: what it costs, and how it scales.

`IMPLEMENTATION.md`'s M3 asks for verifier throughput measured against
`WEAVE-AND-VERIFICATION.md` section 5's waypoint-count table. That table sizes
a plan at 5 600 to 22 800 waypoints if collision is checked by SAMPLING at a
gap fine enough to be trustworthy. The point of this bench is that the verifier
does not sample, so the row of that table it should be compared against is not
the one with the same waypoint count -- it is the one with the same
TRAJECTORY, at whatever waypoint count the planner actually emits.

Two numbers matter:

- **Cost per segment**, which sets the verifier's place in the end-to-end budget.
- **How that cost moves with clearance**, because conservative advancement
  bisects toward tight geometry. A verifier that is cheap in open air and
  unaffordable near the workpiece is not a verifier you can run on a weld.

Run: `pixi run -e motion python tools/bench_verify.py`.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MESH_ROOT = ROOT.parents[1] / "robot_description" / "robots" / "rosie_1400_v3" / "meshes"
REQUEST = ROOT / "data" / "motion" / "bracket_a_placed.weldplan"


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--segments", type=int, default=64)
    parser.add_argument("--workers", type=int, default=0)
    args = parser.parse_args(argv)

    from weld_motion_planner.geometry import mesh as meshmod
    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix
    from weld_motion_planner.verifier import verify as V

    request = read(REQUEST)
    cell = request.cell
    part = meshmod.tessellate(request.step_bytes)
    geom = V.from_cell(cell, mesh_root=MESH_ROOT, workpiece=part,
                       work_to_piece=placement_matrix(request.placement))
    kin = V.kinematics(cell)
    radii = V.reach_radii(kin, geom)

    print(f"bench-verify: {len(geom.meshes)} bodies, {len(geom.pairs)} checked pairs, "
          f"{sum(len(m.faces) for m in geom.meshes.values())} triangles")
    print("  reach radii (mm): " + ", ".join(
        f"{name} {radii[name] * 1000:.0f}" for name in kin.axis_names))

    t0 = time.perf_counter()
    geom.objects()
    print(f"  BVH build: {(time.perf_counter() - t0) * 1000:.1f} ms\n")

    rng = np.random.default_rng(0)
    index = {n: i for i, n in enumerate(kin.axis_names)}

    def arm(scale: float, n: int) -> np.ndarray:
        """A wander around a pose that reaches over the table, at `scale`
        radians per step. Small steps are the weld itself; large ones are the
        transit moves between seams."""
        q = np.zeros((n + 1, kin.num_dof))
        q[:, index["J2"]] = -1.0
        q[:, index["J3"]] = 0.8
        q += rng.normal(0.0, scale, size=q.shape).cumsum(axis=0) * 0.0
        step = rng.normal(0.0, scale, size=(n, kin.num_dof))
        q[1:] += step.cumsum(axis=0)
        return q

    print(f"  {'step':>10}  {'ms/segment':>11}  {'queries/seg':>12}  "
          f"{'verdict':>9}  worst clearance")
    for label, scale in (("1 mrad", 0.001), ("10 mrad", 0.01),
                         ("100 mrad", 0.1), ("500 mrad", 0.5)):
        waypoints = arm(scale, args.segments)
        t0 = time.perf_counter()
        verdict = V.verify(cell, geom, waypoints, workers=args.workers)
        elapsed = time.perf_counter() - t0
        summary = verdict.summary()
        print(f"  {label:>10}  {elapsed / verdict.segments * 1000:11.2f}  "
              f"{verdict.queries / verdict.segments:12.1f}  "
              f"{summary['verdict']:>9}  {summary['worst_clearance_mm']} mm")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
