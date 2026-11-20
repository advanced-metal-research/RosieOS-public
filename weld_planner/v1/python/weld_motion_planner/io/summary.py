"""Human-readable CLI summary for a completed native motion plan."""

from __future__ import annotations

from typing import Any


def print_answer(plan: Any) -> None:
    """Print only the answer; planner timing narration stays on stderr."""
    found = plan.seam_search_result
    if found is not None:
        print(
            f"{found.cell_id}: {found.crossed} of {len(found.seams)} seams "
            f"crossed, {'screened' if found.screened else 'UNSCREENED'}"
        )
    if plan.weld_trajectories:
        print(f"{len(plan.weld_trajectories)} synthesized:")
        for trajectory in plan.weld_trajectories:
            verdict = (
                "not checked"
                if trajectory.verdict is None
                else trajectory.verdict.summary()["verdict"]
            )
            worst = getattr(trajectory.tracking, "bound_m", None)
            print(
                f"  {trajectory.seam_id:14} {verdict}"
                + (
                    f", tracking <= {worst * 1000:.3f} mm"
                    if worst
                    else ""
                )
            )
    if plan.connecting_trajectories is not None:
        held = sum(
            1 for move in plan.connecting_trajectories if move.get("held")
        )
        print(
            f"{len(plan.connecting_trajectories)} connecting moves, "
            f"{held} with the work held still"
        )
    if plan.connecting_trajopt_error:
        print(f"connecting FAILED: {plan.connecting_trajopt_error}")
