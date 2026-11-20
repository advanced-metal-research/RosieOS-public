#!/usr/bin/env python
"""Where the torch is, on the robot that carries it.

Which direction the electrode runs in the tool frame is a fact about a
particular robot with a particular gun bolted to it. It is not a convention
this package gets to choose, and it is not the same on the next cell.

So it is part of the robot's DESCRIPTION -- `robot_description/<model>/config.json`,
beside the URDF whose `link_6_to_end_effector` it is talking about -- and it is
**hand-authored once and committed** rather than derived. Deriving it is not
possible from what the model ships: the evidence is a sentence. `rosie_1400_v3`
states it in
`end_effector_frame_check.json` as

    "TCP +X points from the wrist toward the torch tip / into the workpiece
     side; the 45-degree work angle is baked into the fixed
     link_6->end_effector rotation from CAD."

which a person reads and writes down, the same way the workpiece placement in
`bracket_a_placed.weldplan` is a judgement made once by dragging a part in the
authoring UI. A generator that pretended to extract it would be inventing
confidence.

## Why this is not a default with a fallback

It was, for two commits: `+X`, taken from the one cell in this repo. A default
that happens to be right for the only robot present is indistinguishable from
a correct implementation until the second robot arrives, and then it is wrong
silently -- inverse kinematics converges onto the rotated pose, the lattice
fills with green, and the only symptom is a torch pointing somewhere odd.

The same argument the description already makes about velocity limits:
"deliberately EMPTY rather than guessed... inventing them would produce a
trajectory that verifies against numbers nobody measured." So a model with no
torch in its `config.json` is REFUSED, with the path it should be written to.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from weld_motion_planner.robot_cell.robot import description


class ToolError(ValueError):
    """A tool asset this refuses, with the reason."""


@dataclass(frozen=True)
class ToolFrame:
    """How the gun sits in a cell's tool frame."""

    cell_id: str
    # Unit, in the tool frame: the direction from the tool origin toward the
    # electrode tip. `torch_pose` carries this onto `-u`.
    electrode_axis: tuple[float, float, float]
    tool_frame: str | None = None
    source: str | None = None
    path: Path | None = None

    @classmethod
    def from_json(cls, document: dict[str, Any], path: Path | None = None,
                  cell_id: str = "") -> ToolFrame:
        """The `torch` section of a model's `config.json`."""
        raw = document.get("electrode_axis")
        if not (isinstance(raw, list) and len(raw) == 3):
            raise ToolError(f"{path or 'asset'}: electrode_axis must be three numbers, got {raw!r}")
        vector = np.asarray(raw, dtype=np.float64)
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ToolError(f"{path or 'asset'}: electrode_axis has no direction: {raw!r}")
        unit = vector / norm
        return cls(
            cell_id=str(document.get("cell_id") or cell_id),
            electrode_axis=(float(unit[0]), float(unit[1]), float(unit[2])),
            tool_frame=document.get("tool_frame"),
            source=document.get("source"),
            path=path,
        )

    def summary(self) -> dict[str, Any]:
        return {
            "cell_id": self.cell_id,
            "tool_frame": self.tool_frame,
            "electrode_axis": [round(v, 6) for v in self.electrode_axis],
            "source": self.source,
        }


def load(cell_id: str, root: Path | None = None) -> ToolFrame | None:
    """The torch of a model's description, or None if nobody has authored one."""
    found = description.config(str(cell_id), root)
    if found is None:
        return None
    return ToolFrame.from_json(found.torch, description.config_path(str(cell_id), root), cell_id)


def require(cell_id: str, root: Path | None = None) -> ToolFrame:
    """The torch, or a refusal that says where to write it."""
    found = load(cell_id, root)
    if found is None:
        where = description.config_path(str(cell_id), root)
        raise ToolError(
            f"cell {cell_id!r} has no torch in its description, so which way its electrode "
            f"points is unknown. Write the `torch` section of {where}, taking the direction "
            "from the model's own frame documentation -- guessing it turns every target "
            "orientation by the angle between the guess and the truth, and nothing "
            "downstream notices.")
    return found


def self_test() -> int:
    found = load("rosie_1400_v3")
    if found is None:
        print("tool: FAIL, rosie_1400_v3 has no committed tool asset")
        return 1
    print(f"tool: {found.summary()}")

    axis = np.asarray(found.electrode_axis)
    if abs(float(np.linalg.norm(axis)) - 1.0) > 1e-9:
        print("tool: FAIL, the loaded axis is not a unit vector")
        return 1

    try:
        require("no_such_cell")
    except ToolError as exc:
        if "no torch in its description" not in str(exc):
            print(f"tool: FAIL, the refusal does not say what is missing: {exc}")
            return 1
    else:
        print("tool: FAIL, a cell with no asset was not refused")
        return 1

    print("tool: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("cell_id", nargs="?")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.cell_id:
        print(json.dumps(require(args.cell_id).summary(), indent=2))
        return 0
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except ToolError as exc:
        print(f"tool: {exc}", file=sys.stderr)
        raise SystemExit(1)
