#!/usr/bin/env python
"""How the arm gets from one weld to the next (M6).

Three steps over the welds M5 has already smoothed:

    winding_reach   which turns of a wrist axis are worth considering
    rewound         commit each weld to one of those turns
    connect         plan the approach, the transits, and the retract

The middle step is the one to know about. A wrist axis can reach the same pose
at several full turns of itself, and M4 picked one per weld without knowing what
came before or after -- so a plan straight out of M5 can have a weld commanded
at +180 degrees followed by one at -180. Physically the same pose; on the wire, a
360-degree jump that both a chart reader and a controller take at face value.
`rewound` re-decides those turns for the plan as a whole.

So this returns the REWOUND welds alongside the moves, and the caller must
serialise those rather than the originals it passed in.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from weld_motion_planner.planner.base_types import PlannerContext, PlannerModule
from weld_motion_planner.planner.trajectory_optimization.options import (
    ConnectingTrajoptOptions,
    ConnectingTrajoptResult,
)


class ConnectingTrajoptModule(PlannerModule):
    """M6: rewind every weld onto a consistent turn, then connect them.

    `upstream` is M5's weld trajectories. The result carries the REWOUND welds
    beside the moves -- see `ConnectingTrajoptResult`.
    """

    name = "connecting_trajopt"
    Options = ConnectingTrajoptOptions
    Result = ConnectingTrajoptResult

    def plan(self, request: Any, upstream: list[Any],
             options: ConnectingTrajoptOptions,
             context: PlannerContext) -> ConnectingTrajoptResult:
        from weld_motion_planner.planner.trajectory_optimization.engine import (
            roadmap as roadmap_module,
        )
        from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
            connect,
            move_json,
            order_seams,
            rewound,
            winding_reach,
        )
        from weld_motion_planner.robot_cell.robot_cell import RobotCell

        # The cell's machine, off the context. Built here only for a caller that
        # assembled a context without one -- every axis order, rate limit and
        # standoff below is read off this ONE model, because two stages that
        # assemble their own have already disagreed about the same cell.
        cell = context.cell or RobotCell.of(request.cell, device=context.device)
        held = dict(cell.profile.held)
        home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])
        from weldplan.measured_start import refuse_io
        from .engine.taught import connect_taught_program, in_program_order, start_configuration
        # Every path, before any solve: the weld-only connector below reads
        # only approach/transit/retract and used to drop an I/O row unsaid.
        refuse_io(request.program["nodes"], getattr(request, "io_outputs", None))

        if in_program_order(request):
            # Authored waypoints, or a measured start, fix program order: do
            # not reorder welds around them. Windings are chosen walking from
            # the plan's real first configuration, and the roadmap (built only
            # when there are welds to reach) seeds every free-space leg as it
            # does for the weld-only connector; with no welds there is nothing
            # for it to route to and no cost is paid.
            start = start_configuration(request, cell)
            graph = None
            if upstream:
                if context.progress is not None:
                    context.progress("freespace seed")
                graph = roadmap_module.build(
                    request, context.planner_collision_model, context.device, held=held,
                    focus=winding_reach(request, upstream, start, cell=cell), cell=cell)
            turned = rewound(request, upstream, start, roadmap=graph,
                             planner_collision_model=context.planner_collision_model,
                             device=context.device, held=held, cell=cell)
            return ConnectingTrajoptResult(
                connecting_trajectories=connect_taught_program(request, turned, options, context, cell,
                                                               roadmap=graph),
                weld_trajectories=turned)

        if context.progress is not None:
            context.progress("freespace seed")

        # The visit ORDER comes first: `rewound` walks the plan in order to
        # choose windings, so it has to see the order that will be flown.
        if options.order_seams and len(upstream) > 2:
            limits = getattr(cell.profile, "limits", None) or {}
            upstream = order_seams(
                upstream, home,
                vmax=np.array([float((limits.get(n) or {}).get("velocity_rad_s") or 1e9)
                               for n in cell.moved_axes]),
                amax=np.array([float((limits.get(n) or {}).get("acceleration_rad_s2") or 1e9)
                               for n in cell.moved_axes]))

        # The box of windings the roadmap is allowed to choose from has to hold the
        # candidates it is choosing between, or the choice is made before it runs
        # (`docs/issues/0009`).
        graph = roadmap_module.build(
            request, context.planner_collision_model, context.device, held=held,
            focus=winding_reach(request, upstream, home, cell=cell), cell=cell)
        turned = rewound(request, upstream, home, roadmap=graph,
                         planner_collision_model=context.planner_collision_model,
                         device=context.device, held=held, cell=cell)
        joined = connect(request, turned,
                         planner_collision_model=context.planner_collision_model,
                         device=context.device, roadmap=graph,
                         verifier_collision_model=context.verifier_collision_model,
                         verify_margin_m=options.verify_margin_m, cell=cell,
                         backend=options.backend,
                         progress=context.progress)
        return ConnectingTrajoptResult(
            connecting_trajectories=[move_json(move) for move in joined],
            weld_trajectories=turned)



