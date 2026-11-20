#!/usr/bin/env python
"""Write the sweep's own table into the shape `ui/dp.html` draws.

`graph.json` carries the ANSWER: which nodes screening kept, and which of them
the chosen path runs through. It carries no account of the search, because the
search does not keep one -- the dynamic program needs only the cost row it just
wrote, so `graph.search` overwrites a single tensor and the table is gone by the
time it returns.

That makes the one interesting property of the method invisible. "The best way
to reach this node" is not a fact about the node; it is a fact the sweep
*derives* from the same claim about the previous layer, and the whole reason to
use a dynamic program rather than a search with a seed is that this derivation
never has to look further back than one layer. Drawn, that is a wave of cost
spreading left to right and a back-pointer chain you can follow with a mouse.
Written down as `best_cost: 0.3396` it is a number.

So this asks for `search(trace=True)` and writes what comes back:

- the cost-to-come at every node of every layer, which is the table itself;
- the back-pointers, so the page can walk any node's chain to layer 0;
- the second-best slot's cost beside the best, because where they are equal the
  answer is a coin flip and the K-best set is not as diverse as its count says;
- three pair counts per transition -- all, what the positioner band left, what
  the threshold admitted -- which is the only place the two exact prunings can
  be seen doing their work.

Writes `ui/public/debug/<plan request>/dp.json`, beside `graph.json` and under the
same rules: schema `v0`, nothing may depend on it, may break at any commit.

Run: `pixi run -e motion python tools/make_dp_demo.py`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT_ROOT = ROOT / "ui" / "public" / "debug"
SCHEMA = "motion-dp.v0"

# COSTS GO OUT AS WHOLE MILLIRADIANS, and that is a file-size decision rather
# than a numerical one. The table is the bundle -- two seams of `dog_box` came
# to 8 MiB before this, and 2.5 of those were the cost columns alone, because
# `0.3396488428115845` rounded to four decimals still serialises as nineteen
# characters of JSON. As an integer it is four, and a milliradian is 0.057
# degrees of accumulated joint motion, which is finer than the page prints and
# far finer than a float32 sweep is entitled to claim.
#
# `-1` is unreachable. `null` would be honest and costs two more characters per
# node; the sentinel is safe because a cost-to-come is a sum of absolute joint
# differences and cannot be negative.
MILLIRAD = 1000.0
UNREACHABLE = -1


def one_seam(cell, seam, placement, *, step_s, spin_bins, positioner_bins,
             scene=None, threshold_rad=None, derived=False,
             k_best=2) -> dict[str, Any]:
    from weld_motion_planner.planner.dp_seam_search.engine import graph as graphmod
    from weld_motion_planner.planner.dp_seam_search.engine import nodes
    from weld_motion_planner.robot_cell.robot_cell import RobotCell

    started = time.perf_counter()
    lattice = nodes.build(RobotCell.of(cell), seam, step_s=step_s, spin_bins=spin_bins,
                          positioner_bins=positioner_bins,
                          work_to_piece=placement, scene=scene)
    built = time.perf_counter()
    result = graphmod.search(lattice, threshold_rad=threshold_rad, k_best=k_best,
                             derived=derived, trace=True)
    searched = time.perf_counter()
    trace = result.trace
    assert trace is not None

    L, S, P, B = lattice.shape
    layers = []
    for layer in range(trace.layers()):
        cost = trace.cost[layer]
        live = np.isfinite(cost[:, 0])
        best = np.where(live, np.rint(cost[:, 0] * MILLIRAD), UNREACHABLE)
        # THE RUNNER-UP AS A GAP, not as its own cost. It is the same
        # information given the column beside it, it is the quantity anyone
        # actually asks about -- "how much worse is the second way of getting
        # here" -- and it is a small number where the absolute cost is a large
        # one, which matters when both are being written out per node per
        # layer. Where the gap is zero the two ways are tied and the K-best set
        # is less diverse than its count suggests.
        if cost.shape[1] > 1:
            both = live & np.isfinite(cost[:, 1])
            gap = np.where(both, np.rint((cost[:, 1] - cost[:, 0]) * MILLIRAD),
                           UNREACHABLE)
        else:
            gap = np.full(len(cost), UNREACHABLE)
        entry: dict[str, Any] = {
            # Lattice node index per slot, so the page can put each slot on the
            # same row axis `graph.html` uses. Without it the rows are sweep
            # order, which is sorted by positioner bin and differs layer to
            # layer -- a heatmap drawn in that order shows a shape that is an
            # artefact of the sort.
            "alive": [int(v) for v in trace.alive[layer]],
            "cost": [int(v) for v in best],
            "gap": [int(v) for v in gap],
        }
        if layer:
            # EVERY SLOT, flattened row-major, not just the best one. The chain
            # backwards from a node's best path may arrive at the previous
            # layer through that node's RUNNER-UP -- `back_slot` is a separate
            # array precisely because it can -- so a bundle carrying only slot
            # 0's pointers would let the page draw a chain that quietly changes
            # which path it is following. Costs about 500 KiB a seam at k=2.
            entry["back_node"] = [int(v) for v in trace.back_node[layer].reshape(-1)]
            entry["back_slot"] = [int(v) for v in trace.back_slot[layer].reshape(-1)]
            # The transition INTO this layer, so index `layer` of the counts is
            # index `layer - 1` of the trace's per-transition arrays.
            entry["pairs_all"] = int(trace.pairs_all[layer - 1])
            entry["pairs_kept"] = int(trace.pairs_kept[layer - 1])
            entry["edges"] = int(trace.edges[layer - 1])
        layers.append(entry)

    return {
        "id": seam.id,
        "length_mm": round(seam.length_m * 1000.0, 2),
        "shape": [L, S, P, B],
        "fractions": [round(float(v), 6) for v in lattice.fractions],
        "spin_deg": [round(float(np.rad2deg(v)), 3) for v in lattice.spin_rad],
        "positioner_deg": [round(float(np.rad2deg(v)), 3)
                           for v in lattice.positioner_rad],
        "positioner_axis": lattice.positioner_axis,
        "axis_names": lattice.axis_names,
        "k_best": k_best,
        "cost_unit": "milliradian of accumulated joint motion; -1 is unreachable",
        "layers": layers,
        "graph": {
            **result.summary(),
            "paths": result.paths.tolist(),
            "costs": [None if not np.isfinite(c) else round(float(c), 5)
                      for c in result.costs],
            "reachable_per_layer": [int(v) for v in result.reachable],
        },
        "build_s": round(built - started, 4),
        "search_s": round(searched - built, 4),
    }


def build(path: Path, *, step_s: float, spin_bins: int, positioner_bins: int,
          limit: int, threshold_rad=None, derived: bool = False,
          k_best: int = 2, screen_collisions: bool = True) -> dict[str, Any]:
    import torch
    from weld_motion_planner.geometry.seam_frame import Seam
    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix

    request = read(path)
    cell = request.cell
    placement = placement_matrix(request.placement)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    joints = {str(j.get("id")): j for j in (request.program.get("weld_joints") or [])}

    # The same scene the graph bundle screens with -- borrowed, like the
    # geometry the page borrows, so the two bundles cannot disagree about
    # what the arm may touch.
    scene = None
    if screen_collisions:
        from make_graph_demo import _scene_for
        scene = _scene_for(request, cell, device)

    seams = []
    for spec in request.seams[:limit]:
        one = Seam(spec, joints.get(str(spec.get("weld_joint_ref"))), device=device)
        try:
            seams.append(one_seam(cell, one, placement, step_s=step_s,
                                  spin_bins=spin_bins,
                                  positioner_bins=positioner_bins,
                                  scene=scene,
                                  threshold_rad=threshold_rad, derived=derived,
                                  k_best=k_best))
        except Exception as exc:  # noqa: BLE001 - a bad seam should not lose the rest
            seams.append({"id": one.id, "error": f"{type(exc).__name__}: {exc}"})
        last = seams[-1]
        print(f"  {last['id']}: "
              + (last["error"] if "error" in last
                 else f"{last['graph']['crossed'] and 'crossed' or 'stopped'}, "
                      f"{len(last['layers'])} layers traced, "
                      f"{last['search_s'] * 1000:.0f} ms"))

    return {
        "schema": SCHEMA,
        "name": path.stem,
        "cell_id": cell.get("id"),
        "note": "Debug output. Nothing may depend on this; it may break at any commit.",
        "seams": seams,
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step-ms", type=float, default=250.0,
                        help="milliseconds of travel between samples")
    parser.add_argument("--spin", type=int, default=None)
    parser.add_argument("--positioner", type=int, default=None)
    # TWO by default where `make_graph_demo` takes six. That page carries a
    # screening verdict per node -- one small integer; this one carries a cost
    # and a back-pointer per node per layer, which is about 2 MiB a seam at 25
    # bins. Six of those a plan request is 60 MiB of dev bundle to look at one
    # seam at a time.
    parser.add_argument("--seams", type=int, default=2, help="per plan request")
    parser.add_argument("--threshold-deg", type=float, default=25.0,
                        help="edge threshold, used only if the profile has no rates")
    # TWO, not four. The table is the file, and every extra slot is another
    # float per node per layer for a column the page draws as one number --
    # "how much worse is the runner-up". Four slots doubled `dog_box` to no
    # visible gain.
    parser.add_argument("--k-best", type=int, default=2)
    parser.add_argument("--only", nargs="*", default=None)
    parser.add_argument("--stated-threshold", action="store_true",
                        help="use --threshold-deg instead of the profile's rates")
    # ON by default, same reasoning and same day as `make_graph_demo`'s: a
    # page playing back a path the cell cannot run is a wrong demo, not a
    # light one.
    parser.add_argument("--collide", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="screen against the whole cell (--no-collide to skip)")
    args = parser.parse_args(argv)

    from weld_motion_planner.planner.dp_seam_search.engine.graph import (
        bins_for,
        threshold_from_limits,
    )
    from weld_motion_planner.robot_cell.robot.profile import load as load_profile

    profile = load_profile("rosie_1400_v3")
    axes = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
    rule = threshold_from_limits(profile, axes, args.step_ms / 1000.0)
    from_cell = rule is not None and not args.stated_threshold
    if from_cell:
        print("edge rule derived from the cell profile: "
              + ", ".join(f"{a} {np.rad2deg(v):.1f}deg" for a, v in zip(axes, rule)))
    else:
        rule = np.full(len(axes), np.deg2rad(args.threshold_deg))
    needed = bins_for(float(rule[-1]))
    if args.spin is None:
        args.spin = needed
    if args.positioner is None:
        # 90 over the capped 180 degree span -- 2 degrees apart. Finer than
        # the threshold's floor on purpose: the positioner is the redundancy
        # whose bin IS the state, and the resolution here is what the pages
        # show as genuinely different ways to hold the work.
        args.positioner = 90
    print(f"spin {args.spin}, positioner {args.positioner} "
          f"({360.0 / needed:.1f} deg apart), {args.step_ms:.0f} ms a sample")

    requests = sorted((ROOT / "data" / "motion").glob("*.weldplan"))
    if args.only:
        requests = [p for p in requests if p.stem in args.only]
    if not requests:
        print("make-dp-demo: no plan requests matched", file=sys.stderr)
        return 1

    for path in requests:
        print(f"{path.stem}:")
        try:
            payload = build(path, step_s=args.step_ms / 1000.0, spin_bins=args.spin,
                            positioner_bins=args.positioner, limit=args.seams,
                            threshold_rad=rule, derived=from_cell,
                            k_best=args.k_best, collide=args.collide)
        except Exception as exc:  # noqa: BLE001
            print(f"  skipped: {type(exc).__name__}: {exc}")
            continue
        out = OUT_ROOT / path.stem
        out.mkdir(parents=True, exist_ok=True)
        target = out / "dp.json"
        target.write_text(json.dumps(payload))
        print(f"  -> {target.relative_to(ROOT)} ({target.stat().st_size / 1024:.0f} KiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
