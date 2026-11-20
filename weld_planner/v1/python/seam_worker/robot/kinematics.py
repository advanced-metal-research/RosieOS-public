#!/usr/bin/env python
"""Where every link of a cell is, for a given set of joint values.

Forward kinematics and nothing else. No solver, no IK, no reachability -- those
belong to a planner, and `GOAL.md` draws the line at the weld program. What
placement needs is narrower and unavoidable: **the table moves**. A workpiece
fixtured to `positioner_table_a` sits somewhere different for every value of
J7, J8 and J9, so a placement stored without kinematics is correct only at
zero.

## The convention, stated once

URDF is metres, radians, right-handed, Z-up -- the same as `weld-program.v1`,
so nothing is converted. A joint's `origin` is the FIXED transform from parent
to the joint frame, and the joint's own rotation is applied AFTER it, about
`axis` expressed in that frame:

    T_parent_child = Translate(xyz) . Rpy(rpy) . AxisAngle(axis, q)

`rpy` is fixed-axis roll-pitch-yaw applied X then Y then Z, which composes as

    R = Rz(yaw) . Ry(pitch) . Rx(roll)

That ordering is the single most reusable way to get a robot pointing somewhere
plausible and wrong, which is why it lives in one function here, is mirrored by
exactly one function in `ui/src/kinematics.ts`, and is pinned across the two by
`tests/programmer/test_kinematics_parity.py` over a grid that includes non-commuting
angles. Two rotations that commute prove nothing about their order.

## Matrices, not quaternions

A 4x4 homogeneous matrix per link, composed by multiplication. Quaternions plus
a vector would be fewer numbers and the same arithmetic, but a matrix is what
three.js wants at the other end and what a mesh transform is, so converting
twice would add a place to be wrong for no gain. `numpy` is already a
dependency of every other module here.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any

import numpy as np

SCHEMA = "amr-weld-planner-v1.pose.v1"


class KinematicsError(ValueError):
    """A cell this refuses to pose, with the reason."""


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw as a 3x3 rotation.

    `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`. Written out rather than composed from
    three matrix products because this is the one function whose ORDER matters
    and the product form invites someone to "tidy" the operands.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def axis_angle_matrix(axis: Any, angle: float) -> np.ndarray:
    """Rodrigues rotation about `axis` by `angle` radians, right-handed.

    A zero-length axis is an identity rather than a division by zero: URDF
    allows `<axis>` to be omitted on a fixed joint, and a NaN propagating into
    a transform would put a link at no position at all -- which three.js culls
    silently, so the limb would simply not be there.
    """
    vector = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = vector / norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=float,
    )


def transform(xyz: Any, rpy: Any) -> np.ndarray:
    """A 4x4 from a translation and a fixed-axis rpy triple."""
    out = np.eye(4)
    out[:3, :3] = rpy_matrix(*(float(v) for v in rpy))
    out[:3, 3] = [float(v) for v in xyz]
    return out


def joint_transform(joint: dict[str, Any], value: float) -> np.ndarray:
    """Parent to child, including the joint's own rotation.

    The joint value is applied AFTER the fixed origin, about the axis expressed
    in the joint frame. Putting it before -- rotating the parent and then
    offsetting -- is the other plausible reading of a URDF and gives a limb
    that swings around the wrong pivot.
    """
    fixed = transform(joint["xyz_m"], joint["rpy_rad"])
    if joint["type"] == "fixed":
        return fixed
    moving = np.eye(4)
    moving[:3, :3] = axis_angle_matrix(joint["axis_xyz"], float(value))
    return fixed @ moving


def clamp_to_limits(cell: dict[str, Any], values: dict[str, float]) -> dict[str, float]:
    """Joint values held inside their declared limits.

    Clamped rather than refused. A slider dragged one degree past a limit is
    not an error to argue with -- it is the limit doing its job -- and the
    viewport should show the pose the robot would actually hold. A value for a
    joint the cell does not have IS dropped, because that is a different
    mistake and silently posing nothing would hide it.
    """
    limits = {
        j["name"]: j.get("limits_rad")
        for j in cell.get("joints") or []
        if j["type"] != "fixed"
    }
    out: dict[str, float] = {}
    for name, value in values.items():
        if name not in limits:
            continue
        span = limits[name]
        number = float(value)
        if span:
            number = min(max(number, float(span["lower"])), float(span["upper"]))
        out[name] = number
    return out


def link_transforms(
    cell: dict[str, Any], values: dict[str, float] | None = None
) -> dict[str, np.ndarray]:
    """A 4x4 world transform for every link, keyed by link name.

    Walks parents rather than recursing over children, so a cell whose joints
    are listed in any order still resolves -- URDF does not promise topological
    order and `rosie_1400_v3` really does declare J9 after the arm's tool.

    A link that cannot reach the base is LEFT OUT, not placed at the origin. In
    the middle of a cell the origin is a plausible-looking spot, so a limb
    dropped there reads as geometry rather than as a missing joint; absent, the
    caller can say which link it could not place. `cell.problems` reports the
    same thing at load time.
    """
    joints = {j["child"]: j for j in cell.get("joints") or []}
    posed = clamp_to_limits(cell, values or {})
    names = [link["name"] for link in cell.get("links") or []]

    out: dict[str, np.ndarray] = {}

    def resolve(name: str, seen: frozenset[str]) -> np.ndarray | None:
        if name in out:
            return out[name]
        joint = joints.get(name)
        if joint is None:
            # A root. The base is the identity; any OTHER root is a limb adrift
            # and is not placed.
            return np.eye(4) if name == _base_of(cell, names, joints) else None
        if name in seen:
            raise KinematicsError(f"the joint chain loops at link {name!r}")
        parent = resolve(joint["parent"], seen | {name})
        if parent is None:
            return None
        local = joint_transform(joint, posed.get(joint["name"], 0.0))
        out[name] = parent @ local
        return out[name]

    for name in names:
        placed = resolve(name, frozenset())
        if placed is not None:
            out[name] = placed
    return out


def _base_of(cell: dict[str, Any], names: list[str], joints: dict[str, Any]) -> str | None:
    """The declared base, or the root of the biggest tree. Mirrors `cell.problems`."""
    declared = (cell.get("frames") or {}).get("base")
    if declared in names:
        return declared
    roots = sorted(set(names) - set(joints))

    def root_of(name: str) -> str:
        seen: set[str] = set()
        while name in joints and name not in seen:
            seen.add(name)
            name = joints[name]["parent"]
        return name

    return max(
        roots, key=lambda r: (sum(1 for n in names if root_of(n) == r), r), default=None
    )


def frame(cell: dict[str, Any], role: str, values: dict[str, float] | None = None) -> np.ndarray:
    """The world transform of a NAMED frame -- `base`, `tool` or `work`.

    By role, never by link name. `positioner_table_a` is a fact about one CAD
    export; "the frame a workpiece is fixtured to" is a fact about welding, and
    only the second is safe for a caller to depend on.
    """
    link = (cell.get("frames") or {}).get(role)
    if not link:
        raise KinematicsError(
            f"cell {cell.get('id', '?')!r} has no {role} frame, so nothing can be "
            "measured against it"
        )
    placed = link_transforms(cell, values)
    if link not in placed:
        raise KinematicsError(
            f"the {role} frame is link {link!r}, which is not connected to the base"
        )
    return placed[link]


def rpy_from_matrix(matrix: np.ndarray) -> list[float]:
    """The inverse of `rpy_matrix`: fixed-axis roll-pitch-yaw from a rotation.

    For `R = Rz(yaw) . Ry(pitch) . Rx(roll)`:

        R[2][0] = -sin(pitch)
        roll    = atan2(R[2][1], R[2][2])
        yaw     = atan2(R[1][0], R[0][0])

    At pitch = +/-90 degrees the first and third rotations act about the same
    axis and only their SUM is recoverable -- the classic gimbal lock. Rather
    than return whatever the arctangents make of a degenerate matrix, roll is
    pinned to zero and the whole turn is attributed to yaw. That is one of the
    infinitely many correct answers, it is stable under small changes, and it
    round-trips exactly, which is what a drag handle needs.
    """
    r = np.asarray(matrix, dtype=float)[:3, :3]
    sin_pitch = -float(r[2, 0])
    sin_pitch = min(1.0, max(-1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    if abs(sin_pitch) > 1.0 - 1e-9:
        return [0.0, pitch, math.atan2(-float(r[0, 1]), float(r[1, 1]))]
    return [
        math.atan2(float(r[2, 1]), float(r[2, 2])),
        pitch,
        math.atan2(float(r[1, 0]), float(r[0, 0])),
    ]


def invert(matrix: np.ndarray) -> np.ndarray:
    """The inverse of a RIGID transform: `[R|t]` -> `[R^T | -R^T t]`.

    Not a general inverse. Every transform in this module is a rotation and a
    translation, so the transpose is exact and free where a general inversion
    would introduce error and hide a scale nobody meant to have.
    """
    m = np.asarray(matrix, dtype=float)
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out


def compose(*transforms: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    for item in transforms:
        out = out @ item
    return out


def apply(matrix: np.ndarray, point: Any) -> list[float]:
    """A point through a 4x4. Metres in, metres out."""
    vector = np.asarray([*(float(v) for v in point), 1.0], dtype=float)
    return [float(v) for v in (matrix @ vector)[:3]]


def pose_document(
    cell: dict[str, Any], values: dict[str, float] | None = None
) -> dict[str, Any]:
    """Every link's transform, as row-major 4x4 lists a browser can consume."""
    placed = link_transforms(cell, values)
    missing = sorted(
        {link["name"] for link in cell.get("links") or []} - set(placed)
    )
    return {
        "schema": SCHEMA,
        "cell": cell.get("id"),
        "joints": clamp_to_limits(cell, values or {}),
        "links": {name: [float(v) for v in matrix.flatten()] for name, matrix in placed.items()},
        # Named, not silently absent: a limb missing from a viewport looks like
        # geometry that was never there.
        "unplaced": missing,
    }


def self_test() -> int:
    # A two-joint arm whose answer is obvious by hand: J1 turns 90 degrees
    # about Z, then a 1 m offset along X ends up along +Y.
    cell = {
        "id": "t",
        "frames": {"base": "world", "tool": "tip", "work": "world"},
        "links": [{"name": "world"}, {"name": "arm"}, {"name": "tip"}],
        "joints": [
            {
                "name": "J1", "type": "revolute", "parent": "world", "child": "arm",
                "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
                "limits_rad": {"lower": -3.2, "upper": 3.2},
            },
            {
                "name": "fix", "type": "fixed", "parent": "arm", "child": "tip",
                "xyz_m": [1, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
            },
        ],
    }
    at_zero = link_transforms(cell)
    assert np.allclose(apply(at_zero["tip"], [0, 0, 0]), [1, 0, 0]), at_zero["tip"]

    turned = link_transforms(cell, {"J1": math.pi / 2})
    assert np.allclose(apply(turned["tip"], [0, 0, 0]), [0, 1, 0], atol=1e-12)

    # The limit does its job rather than raising: a slider past the end shows
    # the pose the robot would actually hold.
    clamped = link_transforms(cell, {"J1": 99.0})
    assert np.allclose(apply(clamped["tip"], [0, 0, 0]), apply(link_transforms(cell, {"J1": 3.2})["tip"], [0, 0, 0]))

    # rpy ORDER, asserted against the composition itself rather than against a
    # vector worked out by hand. The first version of this checked
    # `Rz(0).Ry(90).Rx(90) @ [0,0,1] == [1,0,0]`, which is simply the wrong
    # answer -- it is [0,-1,0] -- so the test failed while the code was right.
    # Comparing to the product is the claim actually being made, and it cannot
    # be got wrong by arithmetic.
    def rot_x(a):
        return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])

    def rot_y(a):
        return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])

    def rot_z(a):
        return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    roll, pitch, yaw = 0.3, -0.7, 1.1
    got = rpy_matrix(roll, pitch, yaw)
    assert np.allclose(got, rot_z(yaw) @ rot_y(pitch) @ rot_x(roll), atol=1e-12), got
    # And the reversed order is genuinely different, or the assertion above
    # would hold for either convention and prove nothing.
    assert not np.allclose(got, rot_x(roll) @ rot_y(pitch) @ rot_z(yaw), atol=1e-6)

    # A joint value is applied AFTER its origin, so this link swings about the
    # offset pivot rather than about the parent.
    offset = {
        **cell,
        "joints": [
            {**cell["joints"][0], "xyz_m": [2, 0, 0]},
            cell["joints"][1],
        ],
    }
    swung = link_transforms(offset, {"J1": math.pi / 2})
    assert np.allclose(apply(swung["tip"], [0, 0, 0]), [2, 1, 0], atol=1e-12)

    # An adrift limb is left out, not put at the origin.
    lonely = {**cell, "links": [*cell["links"], {"name": "orphan"}]}
    assert "orphan" not in link_transforms(lonely)
    assert pose_document(lonely)["unplaced"] == ["orphan"]

    # rpy round-trips, including through the gimbal-locked pole where only the
    # SUM of roll and yaw is recoverable.
    for triple in [
        (0.0, 0.0, 0.0), (0.3, -0.7, 1.1), (-2.9, 0.2, 0.05),
        (0.0, math.pi / 2, 0.0), (0.4, -math.pi / 2, 0.9),
    ]:
        back = rpy_from_matrix(rpy_matrix(*triple))
        assert np.allclose(rpy_matrix(*back), rpy_matrix(*triple), atol=1e-9), (triple, back)

    # And a rigid inverse really is one.
    m = transform([1.0, -2.0, 0.5], [0.3, -0.7, 1.1])
    assert np.allclose(invert(m) @ m, np.eye(4), atol=1e-12)

    print("kinematics self-test ok: 90° about Z sends +X to +Y, and a limit clamps")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cell", default="", help="a cell descriptor JSON")
    parser.add_argument("--urdf", default="", help="a robot.urdf to read first")
    parser.add_argument("--joints", default="{}", help='{"J1": 0.5}')
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()

    from pathlib import Path

    if ns.urdf:
        from seam_worker.robot.cell import frames_beside, read_urdf

        document = read_urdf(Path(ns.urdf), frames_beside(Path(ns.urdf)))
    elif ns.cell:
        document = json.loads(Path(ns.cell).read_text(encoding="utf-8"))
    else:
        raise ValueError("one of --cell, --urdf or --self-test is required")
    print(json.dumps(pose_document(document, json.loads(ns.joints)), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"kinematics: {exc}", file=sys.stderr)
        raise SystemExit(1)
