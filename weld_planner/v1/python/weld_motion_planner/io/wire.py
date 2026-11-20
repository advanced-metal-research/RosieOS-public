#!/usr/bin/env python
"""The bytes boundary: a plan request in, a JSON-shaped dict out.

Three functions, and between them they are the whole of what a caller outside
this package exchanges with it:

    request_from_bytes   a `.weldplan`'s bytes -> the request every stage takes
    as_json              a finished `SeamSearchResult` -> the shape a reader consumes
    finite               make any of that legal JSON

`weld_motion_planner.server` is the only caller that needs any of them, and it
needs all three -- it is handed bytes and must answer with JSON. The CLI in
`planner/` uses none: it reads a PATH and prints a summary a person reads, which
is why the wire shape being here costs it nothing. Nothing here plans anything,
and that is the argument for the split: a file whose docstring says "read this to
understand what the planner does" should not also be where the serialisation
lives.

`finite` sits here rather than under `server/` for the narrower version of the
same argument: it exists to fix up what `as_json` produces, so it belongs beside
the thing it fixes up. It spent a while in `server/response.py` next to a copy of
the M6 pipeline, and when that moved to `planner/trajectory_optimization/` what
was left was a
27-line file holding one function that had nothing to do with HTTP.

**Why a dict and not the dataclasses.** `results`, beside this file, declares
what a plan IS, and those declarations are for readers INSIDE the package. What crosses the wire is
plain data in degrees, because every consumer of it prints degrees, and because
`trajectory_optimization.engine.weld_trajopt` imports the planner -- the arrow may not turn
around, so the wire shape cannot depend on M5's report types.

**Additive by construction.** `as_json` takes M5's trajectories keyed by seam id
and a seam GAINS a `trajectory` key without losing anything, so a reader that
only knows about candidates keeps working. The same rule holds the timings: they
are emitted in pipeline insertion order and JSON preserves it, so a stage added
to the planner appears in the drawer with no second edit anywhere.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.results import SeamSearchResult


def request_from_bytes(data: bytes) -> PlanRequest:
    """Open a plan request's bytes into the request every stage takes.

    A server that also wants the trajectories needs the REQUEST as well as
    the plan -- M5 takes the request, not the plan -- so this is separate from
    `plan` rather than folded into it. Opening the plan request twice to get both
    would parse the STEP twice.
    """
    from weldplan.plan_request import read_request

    from weld_motion_planner.io.plan_request import _context_of

    opened = read_request(data)
    return PlanRequest(manifest=opened["manifest"], program=opened["program"],
                       placement=opened["placement"], cell=opened["cell"],
                       step_bytes=opened["step_bytes"], path=None,
                       fixtures=opened.get("fixtures"),
                       context=_context_of(opened), start_state=opened.get("start_state"),
                       io_outputs=opened.get("io_outputs"))


def as_json(result: SeamSearchResult, trajectories: dict[str, dict[str, Any]] | None = None
            ) -> dict[str, Any]:
    """The wire shape. Degrees, because every reader of it prints degrees.

    `trajectories` are M5's, keyed by seam id, and they are ADDITIVE: a seam
    gains a `trajectory` key and loses nothing, so a reader that only knows
    about candidates keeps working. Plain dicts rather than reports, because
    `weld_trajopt` imports this module and the arrow may not turn around.
    """
    trajectories = trajectories or {}
    return {
        "cell_id": result.cell_id,
        "screened": result.screened,
        "k_best": result.k_best,
        "threshold_deg": [round(float(np.rad2deg(v)), 1) for v in result.threshold_rad],
        "threshold_derived_from_limits": result.threshold_derived_from_limits,
        # Where the wall clock went. On the wire because the UI draws it: a
        # number the operator can only get by reading the server's terminal is
        # a number they will never read, and "why did that take forty seconds"
        # is asked in front of the drawer, not in front of the log.
        #
        # Insertion order is PIPELINE order, and JSON preserves it, so the
        # breakdown needs no separate statement of what follows what -- a stage
        # added in the planner appears in the drawer with no second edit.
        "timings_ms": result.timings_ms,
        "seams": [
            {
                "id": s.id,
                "length_mm": round(s.length_m * 1000.0, 2),
                "timings_ms": s.timings_ms,
                **({"error": s.error} if s.error else {}),
                "crossed": s.crossed,
                "frontier": s.frontier,
                "frontier_causes": s.frontier_causes or None,
                "frontier_demand": s.frontier_demand,
                "feasible_rate": round(s.feasible_rate, 4),
                "axis_names": s.axis_names,
                "held": {k: round(float(np.rad2deg(v)), 3) for k, v in sorted(s.held.items())},
                "fractions": [round(float(v), 6) for v in (s.fractions if s.fractions is not None else [])],
                "candidates": [
                    {"cost_deg": round(float(np.rad2deg(c.cost_rad)), 2),
                     "joints_deg": np.round(np.rad2deg(c.joints_rad), 3).tolist(),
                     "times_s": (None if c.times_s is None
                                 else np.round(c.times_s, 4).tolist()),
                     "limit_violations": c.limit_violations,
                     "limit_unverifiable": c.limit_unverifiable}
                    for c in s.candidates
                ],
                **({"trajectory": trajectories[s.id]} if s.id in trajectories else {}),
            }
            for s in result.seams
        ],
    }


def finite(value: Any) -> Any:
    """The same tree with `NaN` and `inf` replaced by `null`.

    Neither is JSON, and one of them anywhere in the tree kills the WHOLE
    response rather than the field it is in -- so a single blocked seam takes
    the twenty-one good ones with it.

    They arrive through `frontier_demand`: a seam the search could not cross
    reports how far short it fell, and on a corner where the frontier sits at the
    first sample that is a mean over zero of them. Applied by the caller rather
    than folded into `as_json` because the caller has more than `as_json`'s output
    to clean -- the moves between welds are built by M6 and carry their own
    clearances.
    """
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [finite(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value
