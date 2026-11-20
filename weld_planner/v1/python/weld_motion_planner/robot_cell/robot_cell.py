#!/usr/bin/env python
"""One work cell, assembled once and handed down.

A cell is the whole station: the arm, the positioner it welds over, the tables,
the floor and the H-frame. `cell.json` describes all of it, and the arm is one
part. So the container is named for the cell and the robot sits inside it.

    RobotCell.descriptor    cell.json: every link and joint of the station
    RobotCell.arm           the robot -- its chain, limits, body and gun
    RobotCell.work_chain    base to the table: the positioner. EMPTY on a cell
                            whose part is fixtured to the floor
    RobotCell.profile       rates, held axes, reset pose. Spans BOTH, so it
                            belongs to the cell rather than to either
    RobotCell.moved_axes    the axes a search moves, positioner last

Fourteen functions were assembling those separately. That is not a cost problem
-- the whole assembly measures 0.18 ms -- but a CONSISTENCY one: two of them can
assemble differently, and that class of divergence has already produced a run
that screened the welds against one set of meshes and the moves between them
against another.

## The device tag

A `ChainSpec` is numpy and belongs to no card; only the joint limits are tensors.
So the tag lives on `Arm`, beside them, and is READ OFF them rather than
remembered from the request -- asking for `cuda` and landing on `cuda:0` cannot
make it disagree. `__post_init__` still refuses a hand-set tag.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, replace
from typing import Any

import torch

from weld_motion_planner.robot_cell import context as contextmod
from weld_motion_planner.robot_cell.robot import fk, ik
from weld_motion_planner.robot_cell.robot.profile import CellProfile
from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell
from weld_motion_planner.robot_cell.robot.profile import moved_axes as moved_axes_of
from weld_motion_planner.robot_cell.robot.tool import ToolFrame


class RobotCellError(ValueError):
    """A cell this refuses to model, with the reason."""


@dataclass(frozen=True)
class Arm:
    """The robot: what moves the torch, and what collides doing it."""

    # base to the tool frame. Its axes are the arm's joints, J1..J6 here.
    chain: fk.ChainSpec
    # Position limits in `chain`'s axis order.
    joint_limits: tuple[torch.Tensor, torch.Tensor]
    # The body being CHECKED, as spheres on the arm's own links. `None` when
    # nobody has authored one: the cell can still be planned unscreened, and
    # `collision.model.for_planner` is what refuses, being the thing that needs
    # them.
    spheres: Any
    # The gun bolted to it: which way the electrode runs, how far off the work
    # the planner holds it.
    tool: ToolFrame
    device: Any
    dtype: Any

    def __post_init__(self) -> None:
        lower, _ = self.joint_limits
        if lower.device != self.device or lower.dtype != self.dtype:
            raise RobotCellError(
                f"this arm says {self.device}/{self.dtype} and its limits are on "
                f"{lower.device}/{lower.dtype}; use `RobotCell.of()` or `to()`")

    def to(self, device: Any = None, dtype: Any = None) -> Arm:
        device = self.device if device is None else device
        dtype = self.dtype if dtype is None else dtype
        if torch.device(device) == self.device and dtype == self.dtype:
            return self
        limits = tuple(t.to(device=device, dtype=dtype) for t in self.joint_limits)
        return replace(self, joint_limits=limits,
                       device=limits[0].device, dtype=limits[0].dtype)


@dataclass(frozen=True)
class RobotCell:
    """The station a plan is planned for."""

    descriptor: dict[str, Any]
    arm: Arm
    # base to the work frame. Its axes are the positioner and the station index.
    work_chain: fk.ChainSpec
    profile: CellProfile
    moved_axes: list[str]
    # The planning context the request carried: the corrected URDF and the
    # meshes, for a reader that needs a path (cuRobo, the cell obstacles).
    context: contextmod.PlanningContext

    @classmethod
    def of(cls, descriptor: dict[str, Any], *, device: Any = None,
           dtype: Any = torch.float32) -> RobotCell:
        """Read the cell's documents and flatten its chains.

        The sphere model, the torch and the URDF come from the planning
        context the request carried -- what the OLP compiled for THIS pair
        (docs/PLANNING-CONTEXT.md). A descriptor with no context is refused:
        there is no longer a description to look those up in by model name.
        """
        cell_id = str(descriptor.get("id") or "")
        compiled = contextmod.of(descriptor)
        if compiled is None:
            raise RobotCellError(
                f"cell {cell_id!r} arrived without a planning context, so its spheres, torch "
                "and rates are unknown; the request predates docs/PLANNING-CONTEXT.md -- re-pack it")
        profile = profile_for_cell(descriptor)
        frames = descriptor.get("frames") or {}
        if device is None:
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        chain = fk.build(descriptor, [str(frames.get("tool") or "")])
        # The WORK chain may be static -- a cell with no positioner is a cell,
        # and `fk.build` allows it for that reason. The TOOL chain may not: an
        # arm nothing moves is a frames declaration that names the wrong link,
        # and the failure downstream would be an inverse kinematics with no
        # unknowns rather than anything that reads as a bad descriptor.
        if chain.num_dof == 0:
            raise RobotCellError(
                f"cell {cell_id!r} has no movable joint between its base and its tool "
                f"frame {str(frames.get('tool') or '')!r}; nothing can pose the torch")
        limits = ik.limits_of(descriptor, chain, device, dtype)
        return cls(
            descriptor=descriptor,
            arm=Arm(chain=chain, joint_limits=limits, spheres=compiled.spheres(),
                    tool=compiled.tool(), device=limits[0].device, dtype=limits[0].dtype),
            work_chain=fk.build(descriptor, [str(frames.get("work") or "")]),
            profile=profile,
            moved_axes=moved_axes_of(descriptor),
            context=compiled,
        )

    def to(self, device: Any = None, dtype: Any = None) -> RobotCell:
        """The same cell with its arm on another card, or in another precision.

        Only the limits move; everything else is numpy or a document. The screen
        wants float32 and the verifier float64, which is why this takes a dtype.
        """
        moved = self.arm.to(device, dtype)
        return self if moved is self.arm else replace(self, arm=moved)

    @property
    def device(self) -> Any:
        return self.arm.device

    @property
    def dtype(self) -> Any:
        return self.arm.dtype

    @property
    def positioner_axis(self) -> str | None:
        """The one work-chain axis this cell welds through, or `None` when the
        work does not move.

        `moved_axes` is the arm's axes followed by the positioner's, so what
        makes this `None` is the two lists being the same length -- derived from
        the chains rather than remembered, the same argument the module header
        makes about the device tag.

        A cell with no positioner is not a degenerate one: `rosie_1420_v1` is an
        arm on a pedestal and the part is bolted to the floor. Its redundancy at
        a weld point is SPIN alone, so the lattice has one positioner bin and
        the search's state is the six arm joints.
        """
        arm_axes = len(self.arm.chain.axis_names)
        return self.moved_axes[-1] if len(self.moved_axes) > arm_axes else None

    def summary(self) -> dict[str, Any]:
        return {
            "cell_id": str(self.descriptor.get("id") or ""),
            "moved_axes": list(self.moved_axes),
            "arm_axes": list(self.arm.chain.axis_names),
            "work_axes": list(self.work_chain.axis_names),
            "sphere_links": sorted(self.arm.spheres.links) if self.arm.spheres else None,
            "held": dict(self.profile.held),
            "device": str(self.device),
            "dtype": str(self.dtype),
        }


def self_test() -> int:
    from pathlib import Path

    from weld_motion_planner.io.plan_request import read

    root = Path(__file__).resolve().parents[3] / "data" / "motion"
    found = sorted(root.glob("*.weldplan"))
    if not found:
        print("robot cell: no committed plan request to model a cell from")
        return 1
    cell = RobotCell.of(read(found[0]).cell, device=torch.device("cpu"))
    print(f"robot cell: {json.dumps(cell.summary())}")

    if cell.positioner_axis is not None and cell.moved_axes[-1] != cell.positioner_axis:
        print("robot cell: FAIL, the positioner is not the last moved axis")
        return 1
    if cell.positioner_axis is None and len(cell.moved_axes) != len(cell.arm.chain.axis_names):
        print("robot cell: FAIL, no positioner was reported and the moved axes "
              "hold more than the arm")
        return 1
    if cell.to() is not cell:
        print("robot cell: FAIL, a move to where it already is rebuilt it")
        return 1

    moved = cell.to(dtype=torch.float64)
    if moved.arm.joint_limits[0].dtype != torch.float64 or moved.dtype != torch.float64:
        print("robot cell: FAIL, to(dtype=) did not move the limits")
        return 1
    if cell.dtype != torch.float32:
        print("robot cell: FAIL, to() mutated the cell it was called on")
        return 1

    try:
        replace(cell.arm, device=torch.device("meta"))
    except RobotCellError:
        pass
    else:
        print("robot cell: FAIL, a tag that disagrees with the tensors was accepted")
        return 1

    print("robot cell: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (RobotCellError, ValueError) as exc:
        print(f"robot cell: {exc}", file=sys.stderr)
        raise SystemExit(1)
