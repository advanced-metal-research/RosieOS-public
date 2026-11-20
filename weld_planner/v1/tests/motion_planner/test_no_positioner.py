"""A cell with no positioner: one bin, and a state as wide as the arm.

`rosie_1400_v3` welds over a two-axis positioner and every stage of the search
was written against that shape -- `moved_axes` refused a cell that did not have
exactly one live positioner axis, `fk.build` refused the jointless chain such a
cell's work frame produces, and the graph's state was the arm's joints plus a
column that had to exist.

`rosie_1420_v1` is an arm on a pedestal and the part is bolted to the floor. Its
work frame IS its base, so the chain to it has no joints and nothing turns the
weld under the torch. The redundancy at a weld point is SPIN alone.

These hold the shape of that cell without needing its assets: the descriptors
here are the smallest thing with the same structure, which is what the failures
were about -- none of them were about this arm's dimensions.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch
from weld_motion_planner.planner.dp_seam_search.engine import graph as graphmod
from weld_motion_planner.planner.dp_seam_search.engine.nodes import FEASIBLE, Lattice
from weld_motion_planner.robot_cell.robot import fk
from weld_motion_planner.robot_cell.robot.cell import axis_roles

# An arm on a fixed mount, welding parts fixtured to the floor. Two joints
# rather than six: what is under test is the WORK chain being empty, and the
# arm's length has nothing to do with it.
MOUNTED = {
    "id": "test_mounted",
    "frames": {"base": "world", "tool": "tool0", "work": "world"},
    "links": [{"name": n} for n in ("world", "pedestal", "l1", "l2", "tool0")],
    "joints": [
        {"name": "mount", "type": "fixed", "parent": "world", "child": "pedestal",
         "xyz_m": [0.0, 0.0, 0.5]},
        {"name": "J1", "type": "revolute", "parent": "pedestal", "child": "l1",
         "axis_xyz": [0.0, 0.0, 1.0], "limits_rad": {"lower": -3.0, "upper": 3.0}},
        {"name": "J2", "type": "revolute", "parent": "l1", "child": "l2",
         "xyz_m": [0.0, 0.0, 0.4],
         "axis_xyz": [0.0, 1.0, 0.0], "limits_rad": {"lower": -2.0, "upper": 2.0}},
        {"name": "gun", "type": "fixed", "parent": "l2", "child": "tool0",
         "xyz_m": [0.3, 0.0, 0.0]},
    ],
}


def test_the_work_chain_has_no_axes_when_the_part_is_on_the_floor():
    roles = axis_roles(MOUNTED)
    assert roles["arm"] == ["J1", "J2"]
    assert roles["work"] == []


def test_a_chain_with_no_joint_builds_and_is_the_identity():
    """`fk.build` used to refuse this, and the refusal reached `RobotCell.of` as
    "no joints on any of these links' chains" -- a message about kinematics for
    a cell whose only unusual property is that its work frame is its base."""
    chain = fk.build(MOUNTED, ["world"])
    assert chain.num_dof == 0
    rot, pos = fk.torch_link_transform(chain, 0, torch.zeros(3, 0))
    assert torch.allclose(rot, torch.eye(3).expand(3, 3, 3))
    assert torch.allclose(pos, torch.zeros(3, 3))


def test_a_static_work_frame_off_the_base_carries_its_fixed_offsets():
    """Not special-cased to the identity: a table BOLTED to the floor is also a
    cell with no positioner, and the offsets on the way to it are real."""
    bolted = {**MOUNTED, "frames": {**MOUNTED["frames"], "work": "pedestal"}}
    chain = fk.build(bolted, ["pedestal"])
    assert chain.num_dof == 0
    _rot, pos = fk.torch_link_transform(chain, 0, torch.zeros(1, 0))
    assert pos[0].tolist() == pytest.approx([0.0, 0.0, 0.5])


def _lattice(*, positioner: str | None) -> Lattice:
    """A two-layer lattice over a two-axis arm, every node feasible.

    `P` is 1 either way: what differs is whether that bin is an ANGLE of a real
    axis or the one pose a fixed workpiece has.
    """
    L, S, P, B = 2, 3, 1, 1
    arm = ["J1", "J2"]
    limits = [[-3.0, 3.0], [-2.0, 2.0]]
    if positioner is not None:
        limits = [*limits, [-1.0, 1.0]]
    return Lattice(
        fractions=np.linspace(0.0, 1.0, L),
        spin_rad=np.linspace(-np.pi, np.pi, S, endpoint=False),
        positioner_rad=np.zeros(P),
        positioner_axis=positioner,
        axis_names=arm,
        held={},
        limits_rad=np.asarray(limits, dtype=float),
        step_s=0.25,
        work_deg=np.zeros(L), travel_deg=np.zeros(L), standoff_mm=np.full(L, 17.0),
        branches=B,
        q=torch.zeros(L, S, P, B, len(arm)),
        cause=torch.full((L, S, P, B), FEASIBLE, dtype=torch.int8),
        position_error_m=torch.zeros(L, S, P, B),
        clearance_m=None,
    )


def test_state_axes_is_the_arm_alone_without_a_positioner():
    assert _lattice(positioner=None).state_axes == ["J1", "J2"]
    assert _lattice(positioner="J7").state_axes == ["J1", "J2", "J7"]


def test_the_searched_state_has_no_positioner_column():
    """A constant column would be harmless arithmetic and a lie about the cell:
    every edge would price a zero move on an axis that does not exist, and
    `limits_rad` would need a row describing its range."""
    q, bounds, _winds = graphmod._state_with_positioner(_lattice(positioner=None))
    assert q.shape[-1] == 2
    assert bounds.shape == (2, 2)


def test_the_sweep_runs_and_reports_paths_as_wide_as_the_arm():
    lattice = _lattice(positioner=None)
    result = graphmod.search(lattice, threshold_rad=np.full(2, 1.0), k_best=2)
    assert result.crossed
    assert result.joints(lattice).shape == (2, 2)
    # Spin is the only binned axis left, so it is the only one that can be
    # reported pinned -- and reading a positioner threshold off a two-axis
    # state would be an index past the end rather than a wrong answer.
    assert set(graphmod.pinned_axes(lattice, np.full(2, 0.01))) <= {"spin"}


def test_the_env_box_clears_the_largest_sphere():
    """The rule the env field's margin has to satisfy, on a synthetic cell.

    Outside the grid `DistanceField.sample` answers `max(boundary value,
    distance to the box)` -- a lower bound, deliberately. Both terms are about
    the margin just outside a face, so a sphere whose radius exceeds the margin
    reads as contact wherever its centre leaves the box, however far the
    geometry actually is.

    `rosie_1420_v1` is the cell that found it: it stands on a pedestal, so its
    env box wraps the mount alone, and at the default 50 mm margin link_2's
    80 mm sphere read 18.0 mm at EVERY pose against exact meshes 88.9 mm apart.
    Nothing cleared the 20 mm floor and the roadmap refused all 4096 samples.
    """
    pytest.importorskip("warp")
    if not torch.cuda.is_available():
        pytest.skip("the distance fields are a GPU path")
    from weld_motion_planner.robot_cell.collision import scene as scenemod

    cell = scenemod._demo_cell()
    device = torch.device("cuda", torch.cuda.current_device())
    # One fat sphere, far bigger than the 50 mm default margin.
    model = {"link_1": [((0.15, 0.0, 0.0), 0.12)], "link_2": [((0.10, 0.0, 0.0), 0.05)]}
    built = scenemod.build(
        cell, model, device=device,
        placement={"anchor_xyz_m": [0, 0, 0], "offset_xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        part=scenemod.box_mesh([-0.08, -0.08, 0.0], [0.08, 0.08, 0.16]),
        env=scenemod.box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
        part_voxel_m=0.006, coarse_voxel_m=0.02, margin_m=0.05,
    )
    env = next(group for group in built.groups if group.label == "env")
    summary = env.field.summary()
    span = np.asarray(summary["upper_pos_m"]) - np.asarray(summary["origin_m"])
    # The env mesh is 3.0 x 3.0 x 0.05; the box has to stand off it by the
    # margin PLUS the largest sphere on every side.
    assert span[0] == pytest.approx(3.0 + 2 * (0.05 + 0.12), abs=0.021)
    assert span[2] == pytest.approx(0.05 + 2 * (0.05 + 0.12), abs=0.021)
