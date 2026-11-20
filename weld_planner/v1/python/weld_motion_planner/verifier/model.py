#!/usr/bin/env python
"""The verifier's collision model: the same cell as real triangles.

Nothing baked and nothing approximated -- the judge's evidence, deliberately
NOT the model the planner searches against. `collision/model.py` builds that
one, and holds the argument for keeping the two different.
"""

from __future__ import annotations

from pathlib import Path

from weld_motion_planner.geometry import mesh as meshmod
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.results import PlanError
from weld_motion_planner.robot_cell.collision import scene as scenemod
from weld_motion_planner.robot_cell.robot.profile import mesh_root as mesh_root_of


def for_verifier(request: PlanRequest, *, mesh_root: Path | None = None):
    """The verifier's model: real triangles, nothing baked, nothing approximated."""
    # Imported here and not at module scope, so the judge's stack loads only
    # when a verifier model is actually built.
    from weld_motion_planner.verifier.verify import from_cell

    # The request's OWN cell. A judge reading another cell's meshes is worse
    # than one reading none: it returns a verdict.
    mesh_root = (mesh_root_of(request.cell.get("id") or "") if mesh_root is None
                 else Path(mesh_root))
    if not Path(mesh_root).is_dir():
        raise PlanError(
            f"no cell meshes at {mesh_root}; the verifier refuses to judge a "
            "trajectory against a cell it cannot see")
    from weld_motion_planner.robot_cell.collision.fixtures import fixture_meshes

    return from_cell(request.cell, mesh_root=Path(mesh_root),
                     workpiece=None if request.step_bytes is None else meshmod.tessellate(request.step_bytes),
                     work_to_piece=scenemod.placement_matrix(request.placement),
                     extra_static=fixture_meshes(request.fixtures))
