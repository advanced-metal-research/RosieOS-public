#!/usr/bin/env python
"""Which pairs the collision stage is allowed to ignore, from the tree.

## Why this is not a hand-written list

Two links joined by a joint touch. That is what a joint IS -- a bearing, a
flange, a mating face -- and their sphere covers overlap there by
construction, because a cover that stopped short of the mating face would
leave the real link uncovered. So an adjacent pair reads as a collision at
every pose including the reset pose, and a feasibility test that says
"always infeasible" is not a test.

`rosie_1400_v3` at the zero pose: 18 of 300 spheres in collision at -42.3
mm, all of them `link_1` against `floor`, which is its parent through J1.
Correct arithmetic, useless answer.

The rule is structural, so it is derived rather than listed. A list would be
a second description of the tree, and the one that goes stale.

## Two different mechanisms, because there are two kinds of pair

**Robot against robot** is a sphere-versus-sphere test and the ignore is a
mask over link pairs: adjacent, and each link against itself.

**Robot against the environment** cannot work that way. The environment is
baked into distance FIELDS, and a field does not know which mesh each voxel
came from -- so "link_1 ignores floor" is not expressible as a mask over one
field that has floor in it.

The precise answer is to bake the environment in groups, one per thing a
link might need to ignore, and mask which groups each link is queried
against. On this cell that is two groups, and the memory is a wash: 121 +
67 MiB against 198 for the union at 5 mm, because the union's bounding box
is mostly the empty space between a wide flat floor and a tall narrow
frame. What it is not free in is QUERIES -- every other link would pay two
where it now pays one, and the field round trip is already the largest
piece of the collision term.

So the shortcut: drop the adjacent link from the environment check
altogether. `link_1` turns about a vertical axis at the base and its
closest approach to anything other than the floor is 355 mm -- measured by
sweeping J1 x J9 x J8 over 13 824 configurations against the exact meshes,
not assumed. The group split stays the answer for a cell where that number
comes out small.
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any


class PairError(ValueError):
    """A tree this refuses to read, with the reason."""


def _link_names(cell: dict[str, Any]) -> list[str]:
    """Every link this descriptor mentions, `links` list or not.

    The joints are read as well as the link list because a descriptor written
    for a test states the tree and not always the links, and a pair rule that
    silently saw no links would return an EMPTY ignore set -- every adjacent
    pair checked, which is the failure this module exists to prevent.
    """
    seen: list[str] = []
    for name in ([str(link["name"]) for link in (cell.get("links") or [])]
                 + [str(j[end]) for j in (cell.get("joints") or [])
                    for end in ("parent", "child") if j.get(end)]):
        if name not in seen:
            seen.append(name)
    return seen


def _ends(joint: dict[str, Any]) -> tuple[str, str]:
    """A joint's `(parent, child)`, refused rather than skipped when either is
    missing: a joint with no child names no pair, and a pair rule that quietly
    dropped it would leave two touching links checked."""
    parent, child = joint.get("parent"), joint.get("child")
    if not parent or not child:
        raise PairError(f"joint {joint.get('name')!r} is missing a parent or a child")
    return str(parent), str(child)


def rigid_groups(cell: dict[str, Any]) -> dict[str, str]:
    """Each link, mapped to the body it is RIGID with: fixed joints collapsed.

    A fixed joint does not make two links neighbours, it makes them one body.
    The distinction is invisible on a cell that bolts its fixed parts to the
    same link the joints attach to, and decisive on one that does not:
    `rosie_1420_v1` mounts the arm as `world -fixed- pedestal -fixed- base -J1-
    link_1`, so `link_1`'s parent is `base` and the body it actually sits
    inside is `pedestal`. The arm's spigot goes down into the mount, so the
    exact triangles are 0.0 mm apart at EVERY J1 (measured over the full turn;
    the nearest VERTICES are 35 mm apart, which is how a vertex-distance check
    misses it). Un-collapsed, the verifier called every waypoint of every weld
    a penetrating collision and the cell could not pass a single trajectory.
    """
    root: dict[str, str] = {}

    def find(name: str) -> str:
        root.setdefault(name, name)
        while root[name] != name:
            root[name] = root[root[name]]
            name = root[name]
        return name

    for name in _link_names(cell):
        find(name)
    for joint in cell.get("joints") or []:
        stated_parent, stated_child = _ends(joint)
        if str(joint.get("type") or "fixed") != "fixed":
            continue
        parent, child = find(stated_parent), find(stated_child)
        if parent != child:
            root[parent] = child
    return {name: find(name) for name in list(root)}


def adjacency(cell: dict[str, Any]) -> set[frozenset[str]]:
    """Every pair of links that touch by construction: jointed, or one body.

    Fixed joints count, and they count TWICE OVER. `end_effector` is bolted to
    `link_6` and their covers overlap exactly as much as two links across a
    bearing do; that the pose never changes between them makes the overlap
    permanent rather than acceptable. And because a fixed joint welds two links
    into one body, a link jointed to EITHER of them is jointed to both --
    `rigid_groups` is the argument, with the measurement.

    On `rosie_1400_v3` collapsing adds exactly one pair, `link_5` against
    `end_effector`, which never comes within 396 mm over 120 random postures:
    the whole torch lies between them, so nothing is given up for it.
    """
    body = rigid_groups(cell)
    jointed: set[frozenset[str]] = set()
    for joint in cell.get("joints") or []:
        parent, child = _ends(joint)
        jointed.add(frozenset((body[parent], body[child])))

    names = _link_names(cell)
    out: set[frozenset[str]] = set()
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if body[a] == body[b] or frozenset((body[a], body[b])) in jointed:
                out.add(frozenset((a, b)))
    return out


def self_ignore(cell: dict[str, Any], links: list[str]) -> list[list[int]]:
    """A mask over sphere-bearing link pairs: 1 to check, 0 to ignore.

    Symmetric, with the diagonal off. A link against itself is the same
    body, and its own spheres overlap wherever the cover needed them to.
    """
    adjacent = adjacency(cell)
    index = {name: i for i, name in enumerate(links)}
    n = len(links)
    mask = [[1] * n for _ in range(n)]
    for i in range(n):
        mask[i][i] = 0
    for pair in adjacent:
        names = tuple(pair)
        if len(names) != 2:
            continue
        a, b = names
        if a in index and b in index:
            mask[index[a]][index[b]] = 0
            mask[index[b]][index[a]] = 0
    return mask


def env_checked(cell: dict[str, Any], sphere_links: list[str], env_links: list[str]) -> list[str]:
    """Which sphere-bearing links are checked against the environment field.

    A link adjacent to environment geometry is dropped from the check
    entirely, rather than the environment being split so that link can skip
    one piece of it.

    Splitting is the more precise answer and it is not free: it doubles the
    field queries for every OTHER link, and the field round trip is already
    the largest piece of the collision term. On this cell the only such link
    is `link_1`, which turns about a vertical axis at the base and stays
    inside the robot's own footprint -- measured over a 13 824-configuration
    sweep of J1 x J9 x J8 against the exact meshes, its closest approach to
    the H-frame and to bay B is 355 mm, so nothing is lost by making it
    invisible to a field it was never going to reach.

    That measurement is the licence for this shortcut, and it is a fact
    about THIS cell. A cell whose first link can swing into a fixture wants
    the split instead, which is why the adjacency above is computed rather
    than assumed.
    """
    adjacent = adjacency(cell)
    return [
        link for link in sphere_links
        if not any(frozenset((link, env)) in adjacent for env in env_links)
    ]


def self_test() -> int:
    cell = {
        "frames": {"base": "world", "tool": "tool0"},
        "joints": [
            {"name": "world_to_floor", "parent": "world", "child": "floor", "type": "fixed"},
            {"name": "J1", "parent": "floor", "child": "link_1", "type": "revolute"},
            {"name": "J2", "parent": "link_1", "child": "link_2", "type": "revolute"},
            {"name": "J3", "parent": "link_2", "child": "link_3", "type": "revolute"},
            {"name": "J9", "parent": "floor", "child": "h_frame", "type": "revolute"},
            {"name": "J8", "parent": "h_frame", "child": "positioner_table_b", "type": "revolute"},
        ],
    }
    links = ["link_1", "link_2", "link_3"]
    mask = self_ignore(cell, links)
    if mask != [[0, 0, 1], [0, 0, 0], [1, 0, 0]]:
        print(f"pairs: FAIL, self mask is {mask}")
        return 1

    # link_1 hangs off `floor`, so it leaves the environment check.
    # `h_frame` is adjacent to `floor` too, but both are environment -- a
    # pair of obstacles touching each other is nobody's problem.
    checked = env_checked(cell, links, ["floor", "h_frame", "positioner_table_b"])
    if checked != ["link_2", "link_3"]:
        print(f"pairs: FAIL, env-checked links are {checked}")
        return 1

    # A FIXED chain above the first joint is one body, so the link jointed to
    # its far end is adjacent to all of it. `rosie_1420_v1` mounts its arm this
    # way and `link_1`'s spigot sits inside `pedestal` at every J1; reading the
    # joint's stated parent alone leaves that pair checked and the verifier
    # refuses every trajectory the cell can run.
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
    adjacent = adjacency(mounted)
    for pair in (("link_1", "pedestal"), ("link_1", "world"), ("pedestal", "base")):
        if frozenset(pair) not in adjacent:
            print(f"pairs: FAIL, {pair} is not adjacent across a fixed mount")
            return 1
    if frozenset(("link_2", "pedestal")) in adjacent:
        print("pairs: FAIL, link_2 is two joints from the mount and was masked "
              "anyway; collapsing fixed joints must not walk through a revolute one")
        return 1

    print("pairs: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose pairs to describe")
    ns = parser.parse_args(argv)
    if ns.self_test or not ns.read:
        return self_test()

    import zipfile
    from pathlib import Path

    from weld_motion_planner.robot_cell.robot.cell import (
        sphere_links as sphere_link_names,
    )

    with zipfile.ZipFile(Path(ns.read)) as archive:
        cell = json.loads(archive.read("cell.json"))
    spheres = sphere_link_names(cell)
    everything = [str(link.get("name")) for link in (cell.get("links") or [])]
    env = [name for name in everything if name not in spheres]
    print(json.dumps({
        "sphere_links": spheres,
        "self_ignore_pairs": sorted(
            sorted(pair) for pair in adjacency(cell)
            if all(name in spheres for name in pair)
        ),
        "env_links": env,
        "env_checked": env_checked(cell, spheres, env),
    }, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"pairs: {exc}", file=sys.stderr)
        raise SystemExit(1)
