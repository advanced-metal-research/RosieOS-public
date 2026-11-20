"""What the discrete search is asked for (M4).

Beside `dp_seam_search_main.py` because these are its settings and nobody
else's. `MotionPlanningOptions` holds one of these; nothing here imports torch, so a
caller can build the options without the GPU stack.

"""

from __future__ import annotations

from dataclasses import dataclass

from weld_motion_planner.planner.base_types import PlannerModuleOptions


@dataclass
class SeamSearchOptions(PlannerModuleOptions):
    """The settings that stop at M4. `plan_seam` takes exactly these, per seam."""

    # Ways across each seam to return. Five because the runner-up is routinely
    # the best path's wrist-flipped twin (`docs/issues/0002`).
    k_best: int = 5

    # Space the lattice by sample COUNT instead of by time. `None` is by time,
    # which is what a weld's own travel speed implies.
    samples_per_seam: int | None = None

    # Screen each posture against the collision model. OFF means UNCHECKED, not
    # safe, and the result says which (`SeamSearchResult.screened`). A flag at
    # all because a cell with no sphere model authored yet cannot screen.
    screen_collisions: bool = True

    # 2 degrees a bin over the positioner's capped 180. Finer than the step
    # limit's floor on purpose; band pruning keeps the sweep from paying
    # quadratically for the extra bins.
    positioner_bins: int = 90

    # Step limit for a cell whose profile states no velocity limits. Reported as
    # `derived: False` so nobody reads it as physics.
    stated_step_limit_deg: float = 25.0

    # Seconds between samples in TIME mode. `None` means the lattice's own
    # default: it lives on `nodes` as `DEFAULT_STEP_S` and naming it here would
    # pull torch into a file whose point is that it needs nothing.
    step_s: float | None = None
