"""Which collision pairs are ignored, and why the rule is structural.

The cases here are the ones that were got wrong on the way in, which is the
only reason a test of a twenty-line module earns its place: environment
defined as "no spheres" rather than "off the arm chain", and an
environment-to-environment adjacency splitting a group it has no business
splitting.
"""

from __future__ import annotations

import pytest
from weld_motion_planner.robot_cell.collision import pairs
from weld_motion_planner.robot_cell.robot.cell import sphere_links


def test_adjacency_includes_fixed_joints(plan_cell):
    adjacent = pairs.adjacency(plan_cell)
    names = {tuple(sorted(pair)) for pair in adjacent}
    # `end_effector` hangs off `link_6` by a FIXED joint. Their covers
    # overlap exactly as much as two links across a bearing do; that the
    # pose between them never changes makes the overlap permanent, not
    # acceptable.
    assert ("end_effector", "link_6") in names
    assert ("link_1", "link_2") in names


def test_self_mask_is_symmetric_with_no_diagonal(plan_cell):
    links = [n for n in sphere_links(plan_cell)]
    mask = pairs.self_ignore(plan_cell, links)
    n = len(links)
    assert all(mask[i][i] == 0 for i in range(n))
    assert all(mask[i][j] == mask[j][i] for i in range(n) for j in range(n))
    # Neighbours off, next-but-one on: a joint apart is a bearing, two
    # joints apart is a pose the planner has to be able to rule out.
    index = {name: i for i, name in enumerate(links)}
    if "link_2" in index and "link_3" in index and "link_4" in index:
        assert mask[index["link_2"]][index["link_3"]] == 0
        assert mask[index["link_2"]][index["link_4"]] == 1


def test_environment_is_off_chain_not_uncovered(plan_cell):
    """The bug this exists for.

    `end_effector` and `tool0` are arm links that carry no spheres yet.
    Calling anything without spheres "environment" made `link_6` adjacent to
    the environment and silently dropped it from the collision check --
    fifty spheres on the wrist, invisible, with nothing to say so.
    """
    arm_chain = sphere_links(plan_cell)
    every = [str(link.get("name")) for link in (plan_cell.get("links") or [])]
    env = [name for name in every if name not in arm_chain]
    assert "end_effector" not in env
    assert "floor" in env

    checked = pairs.env_checked(plan_cell, arm_chain, env)
    assert "link_6" in checked
    # `link_1` is `floor`'s child, and `floor` is environment.
    assert "link_1" not in checked


def test_environment_adjacency_does_not_drop_anything(plan_cell):
    """`h_frame` hangs off `floor` and both are obstacles.

    Two obstacles touching each other is nobody's problem, and it must not
    remove an arm link from the check.
    """
    arm_chain = sphere_links(plan_cell)
    every = [str(link.get("name")) for link in (plan_cell.get("links") or [])]
    env = [name for name in every if name not in arm_chain]
    checked = pairs.env_checked(plan_cell, arm_chain, env)
    assert set(checked) == {name for name in arm_chain if name != "link_1"}


def test_a_joint_without_a_child_is_refused():
    with pytest.raises(pairs.PairError):
        pairs.adjacency({"joints": [{"name": "J1", "parent": "a"}]})


@pytest.fixture
def plan_cell(motion_fixture_path):
    import json
    import zipfile

    with zipfile.ZipFile(motion_fixture_path) as archive:
        return json.loads(archive.read("cell.json"))


def test_a_fixed_mount_is_one_body_with_the_link_jointed_to_it():
    """The pair rule follows the BODY, not the joint's stated parent.

    `rosie_1420_v1` mounts its arm `world -fixed- pedestal -fixed- base -J1-
    link_1`, so J1's parent is `base` while the casting `link_1` actually sits
    inside is `pedestal`. Measured on the real meshes: the exact triangles are
    0.0 mm apart at every J1 over the full turn, while the nearest VERTICES are
    35 mm apart -- the spigot goes down inside the mount. Reading the stated
    parent alone left that pair checked, and the verifier called every waypoint
    of every weld on that cell a penetrating collision.
    """
    mounted = {
        "frames": {"base": "world", "tool": "tool0"},
        "joints": [
            {"name": "world_to_pedestal", "parent": "world", "child": "pedestal",
             "type": "fixed"},
            {"name": "pedestal_to_base", "parent": "pedestal", "child": "base",
             "type": "fixed"},
            {"name": "J1", "parent": "base", "child": "link_1", "type": "revolute"},
            {"name": "J2", "parent": "link_1", "child": "link_2", "type": "revolute"},
        ],
    }
    adjacent = {tuple(sorted(pair)) for pair in pairs.adjacency(mounted)}
    assert ("link_1", "pedestal") in adjacent
    assert ("link_1", "world") in adjacent
    # The mount's own parts are one body with each other.
    assert ("base", "pedestal") in adjacent
    # And the collapse stops at the first REVOLUTE joint: `link_2` is a joint
    # further out and can genuinely swing into the mount, so it stays checked.
    assert ("link_2", "pedestal") not in adjacent


def test_collapsing_fixed_joints_costs_the_shipping_cell_one_far_apart_pair(plan_cell):
    """What the collapse gives up on `rosie_1400_v3`, stated rather than assumed.

    Six pairs stop being checked that the stated-parent rule checked, and five
    of them carry no geometry at all: `tool0` and `world` are bare frames and
    `positioner_table_a_top` is the fixed marker on the table's top face, so a
    pair naming one of them has nothing to collide.

    The sixth is real: `link_5` against `end_effector`, the contact tip bolted
    through the whole torch to `link_6`. Measured over 120 random postures
    inside the cell's own limits, those two never come within 396 mm -- the
    torch lies between them -- so nothing the screen could have caught is given
    up for it. The verifier's own query count says the same: 2475 pair queries
    on `bracket_a_2x` seam 5 became 2367, with every clearance and every
    verdict unchanged.
    """
    stated = {frozenset((str(j["parent"]), str(j["child"])))
              for j in plan_cell["joints"]}
    gained = {tuple(sorted(pair)) for pair in pairs.adjacency(plan_cell) - stated}
    assert gained == {
        ("end_effector", "link_5"),
        ("link_5", "tool0"), ("link_6", "tool0"),
        ("link_1", "world"), ("h_frame", "world"),
        ("h_frame", "positioner_table_a_top"),
    }
