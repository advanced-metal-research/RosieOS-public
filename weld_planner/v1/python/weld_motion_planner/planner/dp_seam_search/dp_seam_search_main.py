#!/usr/bin/env python
"""**How each weld is welded** (M4). A `.weldplan` in, candidate paths out.

    plan request  ->  cell + program + part + placement    io.plan_request
    cell      ->  how far a joint may move in one step    graph.threshold_from_limits
    cell      ->  obstacles, split by rigid frame         collision.scene
    seam      ->  lattice: every way to weld each sample  nodes.build
    lattice   ->  the K best ways across                  graph.search
    path      ->  the trajectory a controller runs        Graph.unwound

Three things a caller has to know:

- The step limit ("edge rule") is one number per joint: that joint's velocity
  limit times the time between samples. A cell that states no rates gets a
  stated number and the result says `derived: False`.
- Screening RAISES when the sphere model or the cell meshes are missing. A
  screen that lost the cell would keep answering "screened" while passing
  postures through the fixtures.
- A seam the search could not cross reports its `frontier` instead of a path.
  The costs stay finite -- they are the best partial ways there -- but the
  trajectory is NaN past that sample.

Run: `pixi run -e motion python -m weld_motion_planner.planner.dp_seam_search.dp_seam_search_main <request.weldplan>`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np

from weld_motion_planner.geometry.seam_frame import Seam
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.plan_request import read as read_request

# Re-exported so callers can import the shapes from the stage that fills them in.
from weld_motion_planner.io.results import (  # noqa: F401
    Candidate,
    PlanError,
    SeamPlan,
    SeamSearchResult,
)
from weld_motion_planner.planner.base_types import PlannerContext, PlannerModule
from weld_motion_planner.planner.dp_seam_search.engine import graph as graphmod
from weld_motion_planner.planner.dp_seam_search.engine import nodes
from weld_motion_planner.planner.dp_seam_search.engine.graph import (
    bins_for,
    threshold_from_limits,
)
from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
from weld_motion_planner.robot_cell.collision import scene as scenemod
from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell
from weld_motion_planner.robot_cell.robot.profile import moved_axes
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.utils.timing import Stopwatch, cuda_sync, report
from weld_motion_planner.verifier.verify import check_limits
from weld_motion_planner.verifier.verify import kinematics as verify_kinematics


class DpSeamSearchModule(PlannerModule):
    """M4: every way to weld every sample, swept for the K best ways across.

    `upstream` is `None` -- this is the first module, and there is nothing before
    it to be handed.
    """

    name = "dp_seam_search"
    Options = SeamSearchOptions
    Result = SeamSearchResult

    def plan(self, request: PlanRequest, upstream: Any,
             options: SeamSearchOptions,
             context: PlannerContext) -> SeamSearchResult:
        import torch

        device = context.device
        if device is None:
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        step_s = options.step_s if options.step_s is not None else nodes.DEFAULT_STEP_S
        cell = (context.cell or RobotCell.of(request.cell, device=device)).to(device)
        watch = Stopwatch(cuda_sync(device))
        # Reported at the DEFAULT step; the per-seam rules used in count mode are
        # this scaled by each seam's own dt.
        rule, derived = edge_rule(request.cell, step_s, options)

        # The flag outranks the handed model, or `screen_collisions=False` would
        # screen anyway. Never built here -- see the adapter below.
        planner_collision_model = (context.planner_collision_model
                                   if options.screen_collisions else None)
        with watch.stage("seams"):
            seams = [plan_seam(request, spec, options, cell,
                               planner_collision_model=planner_collision_model,
                               step_s=step_s, device=device)
                     for spec in request.seams]
        out = SeamSearchResult(
            cell_id=str(request.cell.get("id") or ""),
            screened=planner_collision_model is not None,
            threshold_rad=rule,
            threshold_derived_from_limits=derived,
            k_best=options.k_best,
            seams=seams,
            timings_ms=watch.ms(),
        )
        if context.log:
            for line in log_lines(out):
                print(line, file=sys.stderr, flush=True)
        return out


# ---- What the module runs, per seam --------------------------------------------


def edge_rule(cell: dict[str, Any], step_s: float,
              options: SeamSearchOptions) -> tuple[np.ndarray, bool]:
    """`(step limit per axis, derived)`. The cell's rates when it states them.

    Per AXIS, not one minimum: a single number would hold the wrist to the
    positioner's 60 deg/s when it can do 240.

    `step_s` must be the time the samples are ACTUALLY spaced by -- per seam in
    count mode. Passing the default instead admitted steps sized for 250 ms
    against declared times of 40, so every candidate on a short seam broke the
    limit it was searched under."""
    axes = moved_axes(cell)
    rule = threshold_from_limits(profile_for_cell(cell), axes, step_s)
    if rule is None:
        stated = np.deg2rad(options.stated_step_limit_deg)
        return np.full(len(axes), stated), False
    return rule, True


def plan_seam(request: PlanRequest, spec: dict[str, Any],
              options: SeamSearchOptions, cell: RobotCell, *,
              planner_collision_model, step_s: float, device) -> SeamPlan:
    """Lattice, screen, sweep -- for one seam.

    The sample spacing, and so the step limit, is PER SEAM: `step_s` in time
    mode, `length / speed / intervals` in count mode -- which differs on every
    seam because their lengths do."""
    watch = Stopwatch(cuda_sync(device))
    joints = request.joints_by_id
    with watch.stage("seam"):
        seam = Seam(spec, joints.get(str(spec.get("weld_joint_ref"))), device=device)
        if options.samples_per_seam:
            dt = ((seam.length_m / nodes.travel_speed_m_s(seam))
                  / max(options.samples_per_seam - 1, 1))
        else:
            dt = step_s
        rule, derived = edge_rule(request.cell, dt, options)
    lattice = None
    try:
        with watch.stage("lattice"):
            lattice = nodes.build(
                cell, seam,
                **({"samples": options.samples_per_seam} if options.samples_per_seam
                   else {"step_s": step_s}),
                spin_bins=bins_for(float(rule[-1])),
                positioner_bins=options.positioner_bins,
                work_to_piece=scenemod.placement_matrix(request.placement),
                scene=planner_collision_model)
        with watch.stage("search"):
            result = graphmod.search(lattice, threshold_rad=rule, k_best=options.k_best,
                                     derived=derived)
    except Exception as exc:  # noqa: BLE001 - one bad seam keeps its name
        # WITH its timings, or the run's total loses whatever the failure cost.
        return SeamPlan(id=seam.id, length_m=seam.length_m, crossed=False,
                        error=f"{type(exc).__name__}: {exc}",
                        timings_ms=_seam_timings(watch, lattice))

    out = SeamPlan(
        id=seam.id,
        length_m=seam.length_m,
        crossed=result.crossed,
        frontier=result.frontier,
        frontier_causes=result.frontier_causes or {},
        frontier_demand=result.frontier_demand,
        feasible_rate=float(lattice.feasible.float().mean()),
        axis_names=lattice.state_axes,
        held=dict(lattice.held),
        fractions=np.asarray(lattice.fractions),
    )
    # The sample clock. By time the lattice knows its own step; by count the
    # samples are equal fractions of a seam run at the authored speed.
    L = lattice.q.shape[0]
    if lattice.step_s:
        dt = float(lattice.step_s)
    else:
        speed = nodes.travel_speed_m_s(seam)
        dt = (seam.length_m / speed) / max(L - 1, 1)
    times = np.arange(L, dtype=np.float64) * dt

    # Two `np.diff`s, so cheap enough to run per candidate here. Waypoints are
    # built in the VERIFIER's axis order, held axes included, so this is the same
    # arithmetic T2 will run rather than a lookalike.
    with watch.stage("limits"):
        kin = verify_kinematics(request.cell)
        profile = profile_for_cell(request.cell)
        profile_doc = {"limits": getattr(profile, "limits", None) or {}}
        order = {name: i for i, name in enumerate(out.axis_names)}
        for k in range(options.k_best if result.crossed else 0):
            if not np.isfinite(result.costs[k]):
                continue
            joints_rad = result.unwound(lattice, k)
            waypoints = np.zeros((L, len(kin.axis_names)))
            for i, name in enumerate(kin.axis_names):
                if name in order:
                    waypoints[:, i] = joints_rad[:, order[name]]
                elif name in out.held:
                    waypoints[:, i] = out.held[name]
            violations, unverifiable = check_limits(kin, request.cell, profile_doc,
                                                    waypoints, times)
            out.candidates.append(Candidate(cost_rad=float(result.costs[k]),
                                            joints_rad=joints_rad,
                                            times_s=times,
                                            limit_violations=violations,
                                            limit_unverifiable=unverifiable))
    out.timings_ms = _seam_timings(watch, lattice)
    return out


def _seam_timings(watch: Stopwatch, lattice: Any) -> dict[str, float]:
    """The seam's stages, with the lattice's own substituted for its total.

    LEAVES only, so a breakdown drawn over the dict adds up: `nodes.build`
    reports `setup`, `targets`, `ik`, `screen`, `grid`, and keeping `lattice`
    beside them would double-count. The aggregate survives only when the build
    raised before reporting any of them.
    """
    inner = dict(getattr(lattice, "timings_ms", None) or {})
    if inner:
        flat: dict[str, float] = {}
        for name, ms in watch.stages.items():
            if name == "lattice":
                flat.update(inner)
            else:
                flat[name] = ms
        watch.stages = flat
    return watch.ms()


def log_lines(result: SeamSearchResult) -> list[str]:
    """The breakdown, as text. Printed to stderr: stdout is where a CLI's ANSWER
    goes, and `... > plan.json` has to stay a plan."""
    lines = [
        f"plan {result.cell_id}: {len(result.seams)} seams, "
        f"{result.crossed} crossed, k={result.k_best}"
    ]
    lines += report(result.timings_ms)
    for seam in result.seams:
        stages = seam.timings_ms
        if not stages:
            continue
        # One line per seam, biggest stage first -- enough to spot an outlier.
        inline = " ".join(
            f"{name} {ms:.0f}"
            for name, ms in sorted(stages.items(), key=lambda kv: -kv[1])
            if name != "total"
        )
        lines.append(f"  {seam.id:<14} {stages.get('total', 0):8.0f} ms  {inline}")
    return lines


# ---- CLI --------------------------------------------------------------------


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--k", type=int, default=SeamSearchOptions.k_best)
    parser.add_argument("--samples-per-seam", type=int, default=None)
    parser.add_argument("--no-collide", action="store_true")
    args = parser.parse_args(argv)

    # Running M4 alone is `plan_motion`'s job, asked for by flags; this CLI
    # only renders the result.
    from weld_motion_planner.planner.planner_main import (
        MotionPlanningOptions,
        plan_motion,
    )

    result = plan_motion(
        read_request(args.request),
        MotionPlanningOptions(
            run_weld_trajopt=False, run_connecting_trajopt=False,
            seam_search=SeamSearchOptions(
                k_best=args.k, samples_per_seam=args.samples_per_seam,
                screen_collisions=not args.no_collide))).seam_search_result
    print(f"{result.cell_id}: {len(result.seams)} seams, "
          f"{'screened' if result.screened else 'UNSCREENED'}, "
          f"per-joint step limit "
          f"{'from the cell rates' if result.threshold_derived_from_limits else 'STATED, not from the cell'}")
    for s in result.seams:
        if s.error:
            print(f"  {s.id:12} error: {s.error}")
        elif s.crossed:
            best = min(c.cost_rad for c in s.candidates)
            broken = sum(1 for c in s.candidates if c.limit_violations)
            unv = s.candidates[0].limit_unverifiable if s.candidates else []
            print(f"  {s.id:12} crossed, {len(s.candidates)} candidates, "
                  f"best {np.rad2deg(best):.1f} deg"
                  + (f", {broken} break rate limits" if broken else ", limits clean")
                  + (f", unverifiable: {'; '.join(unv)}" if unv else ""))
        else:
            causes = ", ".join(f"{k} {v}" for k, v in sorted(s.frontier_causes.items()))
            print(f"  {s.id:12} stopped at sample {s.frontier} -- {causes}")
    print(f"{result.crossed} of {len(result.seams)} crossed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
