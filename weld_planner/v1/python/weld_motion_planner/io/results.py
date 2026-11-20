"""What a plan IS, as opposed to how it is made: the seam search's vocabulary.

Read `planner/dp_seam_search/dp_seam_search_main.py` for what fills these in --
this file declares, in the order the search builds them: `Candidate` is one way
across a seam, `SeamPlan` is what the sweep concluded about that seam, and
`SeamSearchResult` is every seam of one plan request. `PlanError` is the fourth
ending: a request the planner refuses, and it is planner-level rather than
search-level -- the collision model is built before the search runs and
`collision/model.py` raises it from there.

A file of its own rather than a header on the search, because the search is
read for its ORDER, and eighty lines of field comments in front of that order
is eighty lines between the reader and the first stage. The comments on the
fields are the point: a field named `times_s` is a mystery about where the
clock came from; the comment beside it says the clock was DECLARED, and that
is a fact about the process that no amount of reading the search recovers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from weld_motion_planner.planner.base_types import PlannerModuleResult


@dataclass
class Candidate:
    """One of the K best ways across a seam, ready to execute or to draw."""

    # Radians of accumulated weighted joint motion -- the sweep's own cost.
    cost_rad: float
    # (samples, dof + 1) radians: the arm's axes then the positioner, UNWOUND
    # -- the values a controller would run, not the lattice's canonical ones.
    joints_rad: np.ndarray
    # (samples,) seconds. DECLARED, not invented: a weld move's timing was
    # never free -- travel speed is a process parameter, so the samples were
    # placed `step_s` of travel apart when the lattice was built and this
    # merely writes that fact down. It is what makes the verifier's velocity and
    # acceleration checks computable at all; cuRobo scales dt until limits
    # hold because ITS tasks may arrive late, and a weld may not.
    times_s: np.ndarray | None = None
    # `check_limits`' verdict on this candidate, when the plan ran it: how
    # many violations, and what stayed unverifiable.
    limit_violations: list[dict[str, Any]] = field(default_factory=list)
    limit_unverifiable: list[str] = field(default_factory=list)


@dataclass
class SeamPlan:
    """What the planner concluded about one seam."""

    id: str
    length_m: float
    crossed: bool
    candidates: list[Candidate] = field(default_factory=list)
    # Present only when the sweep stopped: the first sample nothing reached,
    # why every node there was unavailable, and what the seam demanded.
    frontier: int | None = None
    frontier_causes: dict[str, int] = field(default_factory=dict)
    frontier_demand: dict[str, Any] | None = None
    feasible_rate: float = 0.0
    axis_names: list[str] = field(default_factory=list)
    held: dict[str, float] = field(default_factory=dict)
    fractions: np.ndarray | None = None
    error: str | None = None
    # Milliseconds per stage, in pipeline order: `seam`, then the lattice's own
    # `setup`/`targets`/`ik`/`screen`/`grid`, then `search` and `limits`, then
    # `total` for the whole seam. Present on a seam that FAILED too, and that is
    # the case it earns its keep on -- eleven seconds of lattice ending in an
    # exception is the most interesting line in the log, and a timing dict that
    # only survived success would drop it.
    timings_ms: dict[str, float] = field(default_factory=dict)


@dataclass
class SeamSearchResult(PlannerModuleResult):
    """Every seam of one plan request, and how it was planned."""

    cell_id: str
    screened: bool
    threshold_rad: np.ndarray
    threshold_derived_from_limits: bool
    k_best: int
    seams: list[SeamPlan]
    # `timings_ms` comes from `PlannerModuleResult`: `seams` summed, and `total`.
    # No `collision_model`: the module screens against the model it is handed and
    # never builds one, so that cost is the assembler's and is reported there.

    @property
    def crossed(self) -> int:
        return sum(1 for s in self.seams if s.crossed)


class PlanError(RuntimeError):
    """A plan this refuses to run, with the reason."""
