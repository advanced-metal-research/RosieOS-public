#!/usr/bin/env python
"""What the two continuous stages are asked for, and what they hand back.

Both stages in one file because they are one folder and one subject: M5 optimises
the welding motion, M6 the motion between welds, and their settings are the same
three knobs read by two different solvers. The defaults are written once, on the
M5 object, and referenced by the M6 one.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from weld_motion_planner.planner.base_types import (
    PlannerModuleOptions,
    PlannerModuleResult,
)
from weld_motion_planner.verifier.verify import DEFAULT_VERIFY_MARGIN_M


@dataclass
class WeldTrajoptOptions(PlannerModuleOptions):
    """M5: each weld smoothed from the sweep's samples into a continuous curve."""

    # Keep the curve out of contact with a collision term in the optimizer's
    # cost. `avoid` and not `screen`: M4 throws a colliding lattice node away,
    # this pushes a curve and rejects nothing.
    avoid_collisions: bool = True

    # Check the emitted curve against exact meshes and the joint limits, over the
    # curve rather than the polyline through its knots (T2).
    verify: bool = True

    # The pad that check allows. Wider than a weld pass has room for --
    # `docs/issues/0007` is the argument -- and the default is the verifier's own
    # so nothing here quietly loosens it.
    verify_margin_m: float = DEFAULT_VERIFY_MARGIN_M

    # The descent budget. Inner steps per trust-region round, and rounds.
    #: 40, measured down from 60: the synthesis is seeded by the search's own
    #: candidates and converges before the old budget ran out -- at 40 every
    #: quality gate holds (midpoint < 0.5 mm, certification, the no-harm
    #: bound) and the stage is 12% faster. NOTE the engine's INNER_STEPS
    #: default never reaches the server; THIS number does.
    inner_steps: int = 40
    outer_steps: int = 4

    # Plan ONE seam and skip the rest, by id. For looking at a single weld.
    only_seam: str | None = None


@dataclass
class ConnectingTrajoptOptions(PlannerModuleOptions):
    """M6: the approach, the transits between welds, and the retract.

    The same three knobs as M5 and the same defaults, referenced rather than
    repeated. A separate object because it is a separate stage a caller can run
    on its own -- see `planner_main.plan_motion`.
    """

    avoid_collisions: bool = WeldTrajoptOptions.avoid_collisions
    verify: bool = WeldTrajoptOptions.verify
    verify_margin_m: float = WeldTrajoptOptions.verify_margin_m
    #: Which free-space solver runs this stage.
    #:
    #:   "bspline"  the control-point B-spline solver (`control_solver`);
    #:              "control" is accepted as a silent alias for requests that
    #:              predate the rename
    #:   "curobo"   cuRobo's motion planner (`curobo_backend`), same robot and
    #:              same obstacles -- for comparison, and optional at runtime
    #:   "legacy"   DEPRECATED -- the Catmull-Rom path, kept only to reproduce
    #:              old results
    #:
    #: A REQUEST-level choice rather than a module global, so a caller can put
    #: two backends beside each other without restarting the server.
    backend: str = "bspline"
    # Visit the welds in the order that keeps the transits SHORT, instead of
    # the order the request wrote them in. The welds themselves are untouched
    # -- only which one the arm walks to next -- and the surrogate is the
    # closed-form `duration_for` between one weld's last knot and the next's
    # first, modulo whole turns (`freespace.order_seams`). Off, the authored
    # order is executed as written.
    #
    # OFF by default (owner, 2026-09-22): the order in the program is the
    # order the operator arranged in the spreadsheet, and a plan that quietly
    # visits the welds another way is a plan that does not match its program
    # -- distortion sequencing, tack order and which side is welded first are
    # process decisions the planner has no standing to overrule. A caller
    # that wants the short tour asks for it.
    order_seams: bool = False


@dataclass
class WeldTrajoptResult(PlannerModuleResult):
    """M5's output: one trajectory per weld that got one."""

    # In the order M4 planned the seams. Each is a `weld_trajopt.SeamTrajectory`.
    trajectories: list[Any] = field(default_factory=list)


@dataclass
class ConnectingTrajoptResult(PlannerModuleResult):
    """M6's output, and it is TWO things -- the second is easy to miss.

    Connecting re-decides which turn of a wrist axis each weld is commanded on,
    so it hands back REWOUND welds beside the moves. Serialising the originals
    the caller passed in would put a 360-degree jump at every handover, which is
    why they travel in the result rather than being left to the caller to
    remember.
    """

    # The moves themselves, as wire dicts, in execution order.
    connecting_trajectories: list[dict[str, Any]] = field(default_factory=list)
    # The welds AS REWOUND. Use these, not the ones handed in.
    weld_trajectories: list[Any] = field(default_factory=list)
