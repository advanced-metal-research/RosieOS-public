"""Test-only shortcut: M4 then M5, the flag combination on `plan_motion`.

The product code has no one-call "search then synthesize" -- a caller sets the
stage flags on `planner_main.plan_motion` -- but these tests want exactly that
combination dozens of times, so the options assembly lives here once. Quiet on
purpose: `log=False`.
"""

from __future__ import annotations

from typing import Any

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.results import SeamSearchResult
from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
from weld_motion_planner.planner.planner_main import MotionPlanningOptions, plan_motion
from weld_motion_planner.planner.trajectory_optimization.options import (
    WeldTrajoptOptions,
    WeldTrajoptResult,
)


def plan_seam_search(request: PlanRequest,
                     options: SeamSearchOptions | None = None, *,
                     device: Any = None) -> SeamSearchResult:
    """Run M4 alone, quietly, and hand back its result."""
    return plan_motion(
        request,
        MotionPlanningOptions(run_weld_trajopt=False, run_connecting_trajopt=False,
                              seam_search=options or SeamSearchOptions()),
        device=device, log=False).seam_search_result


def search_then_plan_weld_trajectories(
        request: PlanRequest, *, only_seam: str | None = None,
        screen_collisions: bool = True, samples_per_seam: int | None = None,
        inner_steps: int = WeldTrajoptOptions.inner_steps,
        outer_steps: int = WeldTrajoptOptions.outer_steps,
        verify: bool = True,
        verify_margin_m: float = WeldTrajoptOptions.verify_margin_m,
        k_best: int = 5, device: Any = None) -> WeldTrajoptResult:
    """Run M4, then M5 on its result, and hand back M5's result alone."""
    return plan_motion(
        request,
        MotionPlanningOptions(
            run_weld_trajopt=True, run_connecting_trajopt=False,
            seam_search=SeamSearchOptions(
                k_best=max(1, k_best), samples_per_seam=samples_per_seam,
                screen_collisions=screen_collisions),
            weld_trajopt=WeldTrajoptOptions(
                avoid_collisions=screen_collisions, verify=verify,
                verify_margin_m=verify_margin_m, only_seam=only_seam,
                inner_steps=inner_steps, outer_steps=outer_steps)),
        device=device, log=False).weld_trajopt_result
