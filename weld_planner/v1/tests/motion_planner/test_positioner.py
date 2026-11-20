"""The positioner feasibility mask.

Geometry with an answer that can be read off it rather than trusted: a rod
on a turntable and a wall on one side. The rod reaches past the wall, so the
angles that point at it are refused and the angles that point away are
allowed, and where the boundary falls is arithmetic.

The property worth guarding beyond correctness is that the MARGIN is a
discretization error and not a safety factor -- halve the step and it must
halve. A margin that stayed put would be a fudge quietly deciding which
angles the planner may use.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch
from weld_motion_planner.geometry import esdf
from weld_motion_planner.robot_cell.collision.positioner import (
    Mask,
    PositionerError,
    surface_points,
    sweep,
)
from weld_motion_planner.robot_cell.collision.scene import box_mesh, placement_matrix

pytestmark = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="the distance field is a GPU path"
)

# A turntable at z = 0.5 with a vertical axis, and a wall at x = 0.60..0.70.
CELL = {
    "frames": {"base": "world", "tool": "tool", "work": "table"},
    "links": [{"name": n} for n in ("world", "floor", "table", "tool")],
    "joints": [
        {"name": "world_to_floor", "parent": "world", "child": "floor", "type": "fixed",
         "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        {"name": "J7", "parent": "floor", "child": "table", "type": "revolute",
         "xyz_m": [0, 0, 0.5], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
         "limits_rad": {"lower": -np.pi, "upper": np.pi}},
        {"name": "Jt", "parent": "floor", "child": "tool", "type": "fixed",
         "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
    ],
}
REACH = 0.68
WALL_X = 0.60


@pytest.fixture(scope="module")
def mask() -> Mask:
    device = torch.device("cuda", torch.cuda.current_device())
    field = esdf.bake(box_mesh([WALL_X, -1.0, 0.0], [0.70, 1.0, 1.2]),
                      voxel_m=0.01, margin_m=0.30, frame="world", device=device)
    rod = box_mesh([0.30, -0.05, -0.05], [REACH, 0.05, 0.05])
    return sweep(CELL, field, surface_points(rod, spacing_m=0.02), axis="J7",
                 work_to_piece=placement_matrix({}), device=device)


def test_the_sweep_finds_a_real_obstruction(mask):
    """A mask that allows everything has not been tested by anything.

    The first version of this fixture had the rod stop short of the wall, so
    every angle cleared and the sweep would have passed with the rotation
    removed entirely.
    """
    assert 0.05 < float(mask.allowed.mean()) < 0.95
    assert float(mask.clearance_m.min()) < 0


def test_pointing_at_the_wall_is_refused_and_away_is_allowed(mask):
    assert not mask.allows(0.0)
    assert mask.allows(np.pi)
    assert mask.allows(-np.pi / 2) or mask.allows(np.pi / 2)


def test_the_blocked_window_matches_the_arithmetic(mask):
    """The rod's tip clears the wall when `cos(theta) < wall / reach`, so the
    half-window is `arccos(0.60/0.68)` widened by the rod's half-width and
    the margin. Anything much outside that is a sweep measuring the wrong
    thing."""
    blocked = np.abs(mask.angles_rad[~mask.allowed])
    ideal = np.arccos(WALL_X / REACH)
    assert blocked.max() < ideal + np.deg2rad(15)
    assert blocked.max() > ideal


def test_margin_is_a_discretization_error_not_a_safety_factor(mask):
    """Quarter the step, quarter the margin. A margin that stays put is a
    fudge deciding which angles the planner may use."""
    device = torch.device("cuda", torch.cuda.current_device())
    field = esdf.bake(box_mesh([WALL_X, -1.0, 0.0], [0.70, 1.0, 1.2]),
                      voxel_m=0.01, margin_m=0.30, frame="world", device=device)
    rod = box_mesh([0.30, -0.05, -0.05], [REACH, 0.05, 0.05])
    fine = sweep(CELL, field, surface_points(rod, spacing_m=0.02), axis="J7",
                 work_to_piece=placement_matrix({}), step_rad=np.deg2rad(0.125), device=device)
    assert fine.margin_m == pytest.approx(mask.margin_m / 4, rel=0.05)


def test_the_answer_is_stable_across_step_sizes(mask):
    """Halving the step must not move the allowed fraction much -- if it
    does, the sweep is aliasing rather than resolving."""
    device = torch.device("cuda", torch.cuda.current_device())
    field = esdf.bake(box_mesh([WALL_X, -1.0, 0.0], [0.70, 1.0, 1.2]),
                      voxel_m=0.01, margin_m=0.30, frame="world", device=device)
    rod = box_mesh([0.30, -0.05, -0.05], [REACH, 0.05, 0.05])
    fine = sweep(CELL, field, surface_points(rod, spacing_m=0.02), axis="J7",
                 work_to_piece=placement_matrix({}), step_rad=np.deg2rad(0.25), device=device)
    assert abs(float(fine.allowed.mean()) - float(mask.allowed.mean())) < 0.03


def test_bins_cover_exactly_the_allowed_samples(mask):
    """What node generation consumes. A bin that included a refused angle
    would let the graph put a node somewhere the sweep said it could not."""
    for lo, hi in mask.bins():
        inside = (mask.angles_rad >= lo - 1e-12) & (mask.angles_rad <= hi + 1e-12)
        assert mask.allowed[inside].all()
    covered = sum(int(((mask.angles_rad >= lo) & (mask.angles_rad <= hi)).sum())
                  for lo, hi in mask.bins())
    assert covered == int(mask.allowed.sum())


def test_surface_points_are_deterministic():
    """A mask that shifted between runs would move the graph's admissible
    bins with it, and `ARCHITECTURE.md` asks for reproducibility from a seed."""
    rod = box_mesh([0.0, 0.0, 0.0], [0.3, 0.1, 0.1])
    a = surface_points(rod, spacing_m=0.02)
    b = surface_points(rod, spacing_m=0.02)
    assert a.shape == b.shape
    assert np.array_equal(a, b)


def test_surface_points_land_on_the_surface():
    lo, hi = np.array([0.0, 0.0, 0.0]), np.array([0.3, 0.1, 0.1])
    pts = surface_points(box_mesh(lo, hi), spacing_m=0.02)
    assert len(pts) > 50
    assert (pts >= lo - 1e-9).all() and (pts <= hi + 1e-9).all()
    # Every point is on a face, i.e. touching at least one bound.
    on_face = np.isclose(pts, lo).any(axis=1) | np.isclose(pts, hi).any(axis=1)
    assert on_face.all()


def test_an_unknown_axis_is_refused():
    device = torch.device("cuda", torch.cuda.current_device())
    field = esdf.bake(box_mesh([0, 0, 0], [0.1, 0.1, 0.1]), voxel_m=0.02, device=device)
    with pytest.raises(PositionerError, match="no joint named"):
        sweep(CELL, field, np.zeros((1, 3)), axis="J99",
              work_to_piece=placement_matrix({}), device=device)


def test_a_cell_with_no_work_frame_is_refused():
    device = torch.device("cuda", torch.cuda.current_device())
    field = esdf.bake(box_mesh([0, 0, 0], [0.1, 0.1, 0.1]), voxel_m=0.02, device=device)
    cell = {**CELL, "frames": {"base": "world", "tool": "tool"}}
    with pytest.raises(PositionerError, match="no work frame"):
        sweep(cell, field, np.zeros((1, 3)), axis="J7",
              work_to_piece=placement_matrix({}), device=device)
