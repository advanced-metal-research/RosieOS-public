#!/usr/bin/env python
"""The planner's collision model: one request in, what every search queries out.

A `.weldplan` carries the part as CAD and the cell as a descriptor with its
meshes stripped. Neither is something a search can query. `for_planner` turns a
request into what is: the arm as spheres, everything it must not hit as
signed-distance fields. APPROXIMATE, fast and differentiable, which is what
M4's lattice, M5's optimizer and M6's free-space search all need.

It is half of a deliberate pair. `verifier/model.py` builds the other half --
the same cell as real triangles, nothing baked and nothing approximated --
**and the two disagreeing is why the verifier exists.** Sphere-versus-field
error is the same order as the millimetre clearances a weld runs at -- measured
on this cell, 0.023 mm mean but 1.6 mm worst against the exact mesh -- so the
approximate model serves the SEARCH and the exact one serves the PROOF. Note
what is not shared even in construction: `cell_obstacles` bakes the static
bodies at the held pose, because a distance field has to be built once and then
ridden; the verifier keeps every link a link and turns them all itself.

**It raises rather than returning a smaller model.** Every missing piece here
is a setup problem with a name -- author a sphere model, point at the meshes --
and a planner that quietly plans around one has produced a plan whose single
guarantee is the part that went missing. A screen that has lost the cell keeps
answering "screened" while passing postures straight through the fixtures.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from weld_motion_planner.geometry import mesh as meshmod
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.results import PlanError
from weld_motion_planner.robot_cell.collision import scene as scenemod
from weld_motion_planner.robot_cell.context import mesh_root_for
from weld_motion_planner.robot_cell.robot_cell import RobotCell


def for_planner(request: PlanRequest, cell: RobotCell, device: Any = None, *,
                mesh_root: Path | None = None):
    """The planner's model: the arm against the part, the cell, and itself.

    The CELL half comes from `cell` -- the arm's spheres, and which axes are held --
    so the field the arm is screened against is baked at the pose the lattice
    says it is held at. Baking at a different one is `docs/issues/0003` in
    another form: the world says one thing, the search assumes another, and the
    output looks the same either way.

    The rest is per REQUEST and stays here: the part, where it is fixtured, and
    the cell meshes around it.
    """
    if cell.arm.spheres is None:
        raise PlanError(
            f"no sphere model for cell {cell.descriptor.get('id')!r}; author one with "
            "`pixi run motion-spheres`, or plan with screen_collisions=False on purpose")
    # THIS cell's meshes, not a constant naming one of them. The default is
    # resolved here rather than in the signature because it depends on the
    # argument beside it.
    mesh_root = mesh_root_for(cell.descriptor) if mesh_root is None else Path(mesh_root)
    if not Path(mesh_root).is_dir():
        raise PlanError(
            f"no cell meshes at {mesh_root}; refusing to screen against the "
            "part alone, which would report 'screened' for a cell missing "
            "the table the arm stands over")
    # The operator's own bodies ride along in the request; the cell's
    # placeholder, when the request carries a list, is already gone from
    # the descriptor (`seam_worker.robot.cell.read_urdf`).
    from weld_motion_planner.robot_cell.collision.fixtures import fixture_meshes

    env, fixtures, fixtures_link = scenemod.cell_obstacles(
        cell.descriptor, mesh_root, held=dict(cell.profile.held),
        extra_static=fixture_meshes(request.fixtures))
    return scenemod.build(cell.descriptor, cell.arm.spheres.links,
                          device=device or cell.device,
                          placement=request.placement,
                          part=None if request.step_bytes is None else meshmod.tessellate(request.step_bytes),
                          env=env, fixtures=fixtures, fixtures_link=fixtures_link)
