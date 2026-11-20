#!/usr/bin/env python
"""The work cell: a URDF read as DATA, reduced to what placement needs.

A weld program says what to weld on a workpiece. It says nothing about where
the workpiece SITS -- and it must not, because the same program should run on
whichever table is free. Placement is the missing half, and placement needs a
cell: a robot, a positioner, a table to put the part on, and enough kinematics
to know where that table is when the positioner turns.

## Read as data, not imported

`offline-programming/v1` ships a URDF bundle and a viewer for it. GOAL.md
forbids importing that module's CODE, and this does not: it parses the URDF
with `xml.etree` from the standard library, the same way `server.py` reads
`cadquery/v1/step/dog_box.step` as a fixture. Nothing here calls into their
TypeScript, and their file could be replaced by any other URDF tomorrow.

The parser is deliberately small and refuses what it does not understand. A
URDF feature this does not implement -- a prismatic joint, a mimic, a
`<geometry>` that is a primitive rather than a mesh -- is reported as a
`problem` rather than silently dropped, because a link quietly missing from a
cell is a collision nobody sees coming.

## What comes out

`amr-weld-planner-v1.cell.v1`: links with their mesh references, joints with
their origin, axis and limits, and three NAMED frames that the rest of the
application asks for by role rather than by link name:

    base    the frame everything is measured in            (`world`)
    tool    where the torch is mounted                     (`tool0`)
    work    the surface a workpiece is placed on           (`positioner_table_a_top`)

Those roles are the whole reason this document exists rather than the UI
reading the URDF directly. `positioner_table_a` is a fact about one Fusion
export; "the frame a workpiece is fixtured to" is a fact about welding, and
only the second is safe to write into a placement record.

## Units and conventions

URDF is metres and radians, right-handed, Z-up -- the same as
`weld-program.v1`, so nothing is converted here. `rpy` is the URDF convention:
fixed-axis roll-pitch-yaw, applied X then Y then Z, which composes as
`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`. `kinematics.py` owns that arithmetic and
its TypeScript twin is checked against it, because a rotation convention
duplicated across two languages is exactly how a robot ends up pointing
somewhere plausible and wrong.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from weldplan.urdf import (  # noqa: F401 -- re-exported for the tools and tests that read a checkout
    CELL_FIXTURES,
    SCHEMA,
    SUPPORTED_JOINTS,
    CellError,
    frames_beside,
    read_urdf,
)


def problems(cell: dict[str, Any]) -> list[str]:
    """Everything wrong with a descriptor, as sentences. Empty means usable."""
    out = list(cell.get("problems") or [])
    names = {link["name"] for link in cell.get("links") or []}
    children = [j["child"] for j in cell.get("joints") or []]

    if len(children) != len(set(children)):
        out.append("a link has two parents, so this is not a tree")
    for joint in cell.get("joints") or []:
        for role in ("parent", "child"):
            if joint[role] not in names:
                out.append(f"joint {joint['name']!r} names an unknown {role} {joint[role]!r}")

    base = (cell.get("frames") or {}).get("base")
    if base and base not in names:
        out.append(f"the base frame names link {base!r}, which this cell does not have")
        base = None

    # Falling back to the URDF's own root -- a link that is never a child --
    # rather than skipping the check. A cell with no role mapping is exactly
    # the one nobody has looked at yet, so it is the one that most needs its
    # connectivity checked; keying this on `frames.base` meant an unmapped cell
    # was silently checked for nothing at all.
    parents = {j["child"]: j["parent"] for j in cell.get("joints") or []}

    def root_of(name: str) -> str:
        seen: set[str] = set()
        while name in parents and name not in seen:
            seen.add(name)
            name = parents[name]
        return name

    if not base:
        roots = sorted(names - set(children))
        # The BIGGEST tree is the cell; anything else is a stray limb. Reporting
        # "several roots" instead would be true and useless -- what the operator
        # needs is which link is adrift, not that the graph has a shape.
        base = max(roots, key=lambda r: (sum(1 for n in names if root_of(n) == r), r), default=None)

    # Every link must be reachable from the base, or it renders at the origin
    # in the middle of the cell -- which looks like geometry rather than like a
    # missing joint.
    if base:
        for name in sorted(names):
            if root_of(name) != base:
                out.append(f"link {name!r} is not connected to the base frame")
    return out


def find_cells(root: Path) -> list[Path]:
    """Every `robot.urdf` under `root`, in a stable order.

    Absence is not an error. The bundle lives in a sibling module that may not
    be checked out, and a missing cell directory means placement has nothing to
    offer -- which the UI says, rather than inventing a robot.
    """
    if not root.is_dir():
        return []
    return sorted(root.glob("*/robot.urdf"))


def load_cells(root: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for path in find_cells(root):
        try:
            cell = read_urdf(path, frames_beside(path))
        except CellError as exc:
            # One unreadable bundle must not hide the others.
            out.append(
                {
                    "schema": SCHEMA,
                    "id": path.parent.name,
                    "source": {"filename": path.name},
                    "frames": {},
                    "links": [],
                    "joints": [],
                    "axes": [],
                    "problems": [str(exc)],
                }
            )
            continue
        cell["problems"] = problems(cell)
        out.append(cell)
    return out


def summarise(cell: dict[str, Any]) -> str:
    """One line an operator can act on."""
    meshes = sum(len(link["visuals"]) for link in cell["links"])
    return (
        f"{cell['id']}: {len(cell['links'])} links, {len(cell['axes'])} axes, "
        f"{meshes} meshes"
        + (f", work frame {cell['frames']['work']}" if cell["frames"].get("work") else ", NO work frame")
        + (f" — {len(cell['problems'])} problems" if cell["problems"] else "")
    )


def self_test() -> int:
    import tempfile

    urdf = """<?xml version="1.0"?>
    <robot name="rosie_1400_v3">
      <material name="grey"><color rgba="0.2 0.3 0.4 1"/></material>
      <link name="world"/>
      <link name="floor">
        <visual><origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry><mesh filename="package://rosie_1400_v3/meshes/floor.dae"/></geometry>
          <material name="grey"/></visual>
      </link>
      <link name="tool0"/>
      <link name="positioner_table_a"/>
      <link name="slider"/>
      <joint name="world_to_floor" type="fixed">
        <parent link="world"/><child link="floor"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </joint>
      <joint name="J1" type="revolute">
        <parent link="floor"/><child link="tool0"/>
        <origin xyz="0.7874 0.6858 0.48" rpy="0 0 1.5"/>
        <axis xyz="0 0 1"/><limit lower="-6.3" upper="6.3"/>
      </joint>
      <joint name="J7" type="revolute">
        <parent link="floor"/><child link="positioner_table_a"/>
        <origin xyz="1.9 0.68 0.65" rpy="0 0 0"/>
        <axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14"/>
      </joint>
      <joint name="J99" type="prismatic">
        <parent link="floor"/><child link="slider"/>
        <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
      </joint>
    </robot>
    """
    # The frames the description states, the way the OLP hands them over.
    frames = {"base": "world", "tool": "tool0", "work": "positioner_table_a_top",
              "work_surface": {"parent": "positioner_table_a", "xyz_m": [0.0, -0.622, 0.1]}}
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "robot.urdf"
        path.write_text(urdf, encoding="utf-8")
        cell = read_urdf(path, frames)
        # And a description that states none: a cell with no work frame says
        # so rather than placing a part at the origin.
        assert "no work frame" in " ".join(read_urdf(path)["problems"])
        assert frames_beside(path) is None

    assert cell["id"] == "rosie_1400_v3"
    assert cell["axes"] == ["J1", "J7"], cell["axes"]
    assert cell["frames"] == {"base": "world", "tool": "tool0", "work": "positioner_table_a_top"}
    # The work frame is the table's top face, carried as a fixed child of the
    # table so it rides J7 -- and is not an axis.
    top = next(j for j in cell["joints"] if j["child"] == "positioner_table_a_top")
    assert (top["type"], top["parent"], top["xyz_m"]) == ("fixed", "positioner_table_a", [0.0, -0.622, 0.1])

    # The package prefix is a ROS locator and means nothing to a browser.
    floor = next(link for link in cell["links"] if link["name"] == "floor")
    assert floor["visuals"][0]["mesh"] == "meshes/floor.dae", floor["visuals"][0]["mesh"]
    assert floor["visuals"][0]["rgba"] == [0.2, 0.3, 0.4, 1.0]

    # rpy is read, not dropped -- a mount rotation silently lost would point a
    # torch somewhere plausible and wrong.
    j1 = next(j for j in cell["joints"] if j["name"] == "J1")
    assert j1["rpy_rad"] == [0.0, 0.0, 1.5]
    assert j1["limits_rad"] == {"lower": -6.3, "upper": 6.3}

    # The prismatic joint is REPORTED, and its child is then unreachable --
    # both said out loud rather than leaving a limb at the origin.
    found = problems(cell)
    assert any("prismatic" in p for p in found), found
    assert any("'slider' is not connected" in p for p in found), found

    print("cell self-test ok: " + summarise({**cell, "problems": found}))
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--urdf", default="", help="a robot.urdf to read")
    parser.add_argument("--root", default="", help="a directory of <cell>/robot.urdf bundles")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if ns.root:
        for cell in load_cells(Path(ns.root)):
            print(summarise(cell))
            for problem in cell["problems"]:
                print(f"    - {problem}")
        return 0
    if not ns.urdf:
        raise ValueError("one of --urdf, --root or --self-test is required")
    cell = read_urdf(Path(ns.urdf), frames_beside(Path(ns.urdf)))
    cell["problems"] = problems(cell)
    print(json.dumps(cell, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"cell: {exc}", file=sys.stderr)
        raise SystemExit(1)
