#!/usr/bin/env python
"""The cell as the planner sees it: which axes are its to move, and where
everything is when they take a value.

## Why this is not just forward kinematics

The authoring side has forward kinematics too, and it is the same arithmetic.
What it does not have -- because it never needed it -- is the distinction
that decides the shape of the planner's search: an axis the planner DRIVES,
an axis it is merely GIVEN, and an axis that belongs to the cell but not to
this weld.

`rosie_1400_v3` makes the distinction concrete. Its work frame hangs off two
revolute joints:

    floor --J9--> h_frame --J7--> positioner_table_a
                        \\--J8--> positioner_table_b

J7 tilts the table the part is clamped to: a live positioner axis, and the
second dimension of the null space the redundancy graph searches. J9 turns
the whole H-frame, swapping which bay faces the robot -- an INDEX, held for
the length of a weld, because the other bay is where an operator is
fixturing the next part. J8 tilts that other bay and is not on the work
chain at all.

Three roles, and only two of them are derivable from the descriptor:

    arm         on the chain from base to tool                  DRIVEN
    work chain  on the chain from base to work                  DRIVEN or HELD
    off chain   on neither                                      HELD

The chains are structure and are computed here. Which work-chain axes are
live and which are indices is a fact about how the cell is USED, and lives
in the cell profile. Defaulting is not an option: guess that J9 is live and
the planner will happily rotate a bay through the operator; guess that J7 is
held and it throws away the redundancy that makes seams reachable.

## Held is not the same as absent

The temptation is to drop held axes and plan over the seven that move. That
is wrong twice. The work frame's pose depends on J9, so a plan built with it
at zero puts the part somewhere it is not. And bay B -- with a part clamped
to it -- is geometry the arm must not hit, so J8's value has to reach the
collision stage even though nothing will ever optimize it.

So a pose here is always over EVERY movable axis. What the planner varies is
a subset, named by `driven`; the rest are pinned by the profile and travel
alongside.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


class CellError(ValueError):
    """A cell this refuses to pose, with the reason."""


# --------------------------------------------------------------------------
# transforms
# --------------------------------------------------------------------------


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw: `R = Rz(yaw) Ry(pitch) Rx(roll)`.

    Fixed-axis and not Euler, which is the same matrix read in the other
    order and a sign error waiting to happen. Held to the authoring side's
    `kinematics.rpy_matrix` by an exported oracle rather than by agreement.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_matrix(axis: Any, angle: float) -> np.ndarray:
    """Rodrigues rotation about `axis` by `angle` radians, right-handed."""
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise CellError("a joint axis of zero length has no rotation")
    x, y, z = vector / norm
    c, s = float(np.cos(angle)), float(np.sin(angle))
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )


def transform(xyz: Any, rpy: Any) -> np.ndarray:
    """A 4x4 from a translation and a fixed-axis rpy triple."""
    out = np.eye(4)
    out[:3, :3] = rpy_matrix(*[float(v) for v in rpy])
    out[:3, 3] = [float(v) for v in xyz]
    return out


def invert(matrix: np.ndarray) -> np.ndarray:
    """The inverse of a rigid transform, by transpose rather than by solve."""
    out = np.eye(4)
    rotation = matrix[:3, :3]
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ matrix[:3, 3]
    return out


def joint_transform(joint: dict[str, Any], value: float) -> np.ndarray:
    """Parent to child, including the joint's own rotation."""
    origin = transform(joint.get("xyz_m", (0.0, 0.0, 0.0)), joint.get("rpy_rad", (0.0, 0.0, 0.0)))
    if joint.get("type") != "revolute":
        return origin
    turn = np.eye(4)
    turn[:3, :3] = axis_angle_matrix(joint.get("axis_xyz", (0.0, 0.0, 1.0)), float(value))
    return origin @ turn


# --------------------------------------------------------------------------
# structure
# --------------------------------------------------------------------------


def _parents(cell: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Child link name to the joint that reaches it."""
    return {str(j.get("child")): j for j in (cell.get("joints") or [])}


def chain_to(cell: dict[str, Any], link: str) -> list[dict[str, Any]]:
    """Every joint from the base down to `link`, base end first.

    Raises rather than returning a partial chain: a chain that quietly stops
    short describes a robot with a limb missing, and the pose it computes is
    wrong in a way nothing downstream can detect.
    """
    parents = _parents(cell)
    base = (cell.get("frames") or {}).get("base")
    walk: list[dict[str, Any]] = []
    node = str(link)
    seen: set[str] = set()
    while node in parents and node not in seen:
        seen.add(node)
        joint = parents[node]
        walk.append(joint)
        node = str(joint.get("parent"))
    if base and node != base:
        raise CellError(
            f"link {link!r} does not connect back to the base frame {base!r}; "
            f"the walk stopped at {node!r}"
        )
    return list(reversed(walk))


def axis_roles(cell: dict[str, Any]) -> dict[str, list[str]]:
    """Which movable axes reach the tool, which reach the work, which neither.

    Structure only. Whether a work-chain axis is a live positioner or a held
    index is not in here and cannot be -- see the module docstring.
    """
    frames = cell.get("frames") or {}
    movable = {
        str(j.get("name")) for j in (cell.get("joints") or []) if j.get("type") == "revolute"
    }

    def axes_of(role: str) -> list[str]:
        link = frames.get(role)
        if not link:
            return []
        return [
            str(j["name"]) for j in chain_to(cell, str(link)) if j.get("type") == "revolute"
        ]

    arm = axes_of("tool")
    work = axes_of("work")
    ordered = [str(j["name"]) for j in (cell.get("joints") or []) if j.get("type") == "revolute"]
    off = [name for name in ordered if name not in set(arm) | set(work)]
    shared = sorted(set(arm) & set(work))
    if shared:
        # A cell where moving the arm moves the table. Physically possible --
        # a positioner mounted on the arm -- and not something this planner
        # models, so it is refused rather than silently mis-planned.
        raise CellError(
            f"axes {', '.join(shared)} are on both the tool and the work chain; "
            "this planner assumes the part and the torch move independently"
        )
    return {"arm": arm, "work": work, "off_chain": off, "movable": sorted(movable)}


# --------------------------------------------------------------------------
# the posed cell
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    """One value per movable axis, driven and held alike.

    A dict and not a vector, because a vector is only meaningful beside the
    ordering that produced it and the two get separated. `vector()` produces
    one when the optimizer needs it, against an explicit list of names.
    """

    values: dict[str, float]

    def vector(self, names: list[str]) -> np.ndarray:
        missing = [name for name in names if name not in self.values]
        if missing:
            raise CellError(f"no value for {', '.join(missing)}")
        return np.array([self.values[name] for name in names], dtype=float)

    def with_values(self, updates: dict[str, float]) -> Pose:
        return Pose({**self.values, **updates})


def link_transforms(cell: dict[str, Any], pose: Pose | dict[str, float]) -> dict[str, np.ndarray]:
    """A 4x4 world transform for every link the base can reach."""
    values = pose.values if isinstance(pose, Pose) else dict(pose)
    frames = cell.get("frames") or {}
    parents = _parents(cell)
    links = [str(link.get("name")) for link in (cell.get("links") or [])]
    base = frames.get("base")
    if base and base not in links:
        raise CellError(f"the base frame names link {base!r}, which this cell does not have")

    out: dict[str, np.ndarray] = {}

    def place(name: str, seen: frozenset[str]) -> np.ndarray:
        if name in out:
            return out[name]
        joint = parents.get(name)
        if joint is None:
            out[name] = np.eye(4)
            return out[name]
        parent = str(joint.get("parent"))
        if parent in seen:
            raise CellError(f"link {name!r} is its own ancestor; this cell is not a tree")
        matrix = place(parent, seen | {name}) @ joint_transform(
            joint, float(values.get(str(joint.get("name")), 0.0))
        )
        out[name] = matrix
        return matrix

    for name in links:
        place(name, frozenset())
    return out


def frame(cell: dict[str, Any], role: str, pose: Pose | dict[str, float]) -> np.ndarray:
    """The world transform of a named frame -- `base`, `tool` or `work`."""
    named = (cell.get("frames") or {}).get(role)
    if not named:
        raise CellError(f"this cell names no {role} frame")
    placed = link_transforms(cell, pose)
    if named not in placed:
        raise CellError(f"the {role} frame names link {named!r}, which this cell does not have")
    return placed[named]


def sphere_links(cell: dict[str, Any], driven: list[str] | None = None) -> list[str]:
    """Links that need a sphere cover: the body being CHECKED.

    `ARCHITECTURE.md`: spheres are for the moving body the planner checks,
    fields are for everything it must not hit. That makes this the chain
    from the base to the TOOL, minus whatever does not actually move --
    and both halves of that sentence have a trap in them.

    `floor` is on the tool chain and is bolted to `world` by a fixed joint.
    It never moves, so it is a field; covering a three-metre base plate in
    spheres would be both wrong and enormous.

    `positioner_table_a` DOES move -- J7 drives it -- and is still not
    spheres, because it is not the body being checked: it is rigid with
    respect to the workpiece and bakes into that frame's field, riding the
    positioner for free. A rule of "everything that moves" put it here, and
    would have had the planner checking the table against the part it is
    bolted to.

    And `end_effector` and `tool0` hang off `link_6` by FIXED joints, so a
    rule of "children of driven joints" would drop them -- while they move
    exactly as much as the link they are bolted to, and carry the torch.

    `ui/spheres.ts` states the same rule in TypeScript: the tool must agree
    with the reader about which links it is asking an operator to cover.
    """
    parents = _parents(cell)
    live = set(driven) if driven is not None else {
        str(j["name"]) for j in (cell.get("joints") or []) if j.get("type") == "revolute"
    }
    frames = cell.get("frames") or {}
    base = frames.get("base")
    tool = frames.get("tool")
    if not tool:
        return []

    path: list[tuple[str, dict[str, Any]]] = []
    node = str(tool)
    seen: set[str] = set()
    while node in parents and node not in seen and node != base:
        seen.add(node)
        joint = parents[node]
        path.append((node, joint))
        node = str(joint.get("parent"))

    out: list[str] = []
    moving = False
    for name, joint in reversed(path):
        if str(joint.get("name")) in live:
            moving = True
        if moving:
            out.append(name)
    return out


def zero_pose(cell: dict[str, Any]) -> Pose:
    """Every movable axis at zero. A starting point, never a plan."""
    return Pose(
        {
            str(j["name"]): 0.0
            for j in (cell.get("joints") or [])
            if j.get("type") == "revolute"
        }
    )


def self_test() -> int:
    """A two-bay cell in miniature: an arm, an index axis, a live table, and
    a second table that belongs to nobody's plan."""
    cell = {
        "id": "two_bay",
        "frames": {"base": "floor", "tool": "tool0", "work": "table_a"},
        "links": [
            {"name": n}
            for n in ("floor", "link_1", "link_2", "tool0", "h_frame", "table_a", "table_b")
        ],
        "joints": [
            {
                "name": "J1", "type": "revolute", "parent": "floor", "child": "link_1",
                "xyz_m": [0, 0, 0.5], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
            },
            {
                "name": "J2", "type": "revolute", "parent": "link_1", "child": "link_2",
                "xyz_m": [0, 0, 1.0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 1, 0],
            },
            {
                "name": "fixed_tool", "type": "fixed", "parent": "link_2", "child": "tool0",
                "xyz_m": [0.2, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
            },
            {
                "name": "J9", "type": "revolute", "parent": "floor", "child": "h_frame",
                "xyz_m": [1.0, 0, 0.3], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
            },
            {
                "name": "J7", "type": "revolute", "parent": "h_frame", "child": "table_a",
                "xyz_m": [0, 0.5, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 1, 0],
            },
            {
                "name": "J8", "type": "revolute", "parent": "h_frame", "child": "table_b",
                "xyz_m": [0, -0.5, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, -1, 0],
            },
        ],
    }

    roles = axis_roles(cell)
    assert roles["arm"] == ["J1", "J2"], roles
    assert roles["work"] == ["J9", "J7"], roles
    assert roles["off_chain"] == ["J8"], roles

    # The trap the module exists to name: J9 takes no decision and still moves
    # the part. A plan that dropped it would put the work frame somewhere it
    # is not.
    held = zero_pose(cell)
    turned = held.with_values({"J9": np.pi / 2})
    a = frame(cell, "work", held)[:3, 3]
    b = frame(cell, "work", turned)[:3, 3]
    assert np.linalg.norm(a - b) > 0.5, f"the index axis did not move the work frame: {a} {b}"

    # And J8 moves neither the tool nor the work frame, which is exactly why it
    # must not be searched -- and exactly why it still has to reach collision.
    other = held.with_values({"J8": 1.0})
    assert np.allclose(frame(cell, "work", other), frame(cell, "work", held))
    assert not np.allclose(
        link_transforms(cell, other)["table_b"], link_transforms(cell, held)["table_b"]
    )

    # Arm axes move the tool and leave the part alone. The independence the
    # whole two-frame collision split rests on.
    reached = held.with_values({"J1": 0.3, "J2": -0.2})
    assert not np.allclose(frame(cell, "tool", reached), frame(cell, "tool", held))
    assert np.allclose(frame(cell, "work", reached), frame(cell, "work", held))

    # Inverse and forward agree, which is what the workpiece-frame map needs.
    work = frame(cell, "work", turned)
    assert np.allclose(invert(work) @ work, np.eye(4), atol=1e-12)

    # A positioner hung off the arm: the H-frame reparented onto link_2, so
    # every arm axis is also a work axis. Physically buildable, not modelled
    # here, and refused rather than planned wrongly.
    on_the_arm = {
        **cell,
        "joints": [
            j if j["name"] != "J9" else {**j, "parent": "link_2"} for j in cell["joints"]
        ],
    }
    try:
        axis_roles(on_the_arm)
    except CellError as exc:
        assert "both the tool and the work chain" in str(exc), exc
    else:
        raise AssertionError("a positioner mounted on the arm was accepted")

    print(
        "cell self-test ok: arm, work chain and off-chain axes told apart; a held index "
        "axis still moves the part and a second bay still moves"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose cell to describe")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise CellError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read

    cell = read(Path(ns.read)).cell
    roles = axis_roles(cell)
    print(
        json.dumps(
            {
                "id": cell.get("id"),
                "frames": cell.get("frames"),
                "axes": roles,
                "note": (
                    "which work-chain axes are live and which are held indices is a fact "
                    "about how the cell is used, and belongs in the cell profile"
                ),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"cell: {exc}", file=sys.stderr)
        raise SystemExit(1)
