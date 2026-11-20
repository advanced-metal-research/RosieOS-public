#!/usr/bin/env python
"""The cell profile: what a URDF does not say and a weld program must not.

Between the two documents the planner is handed there is a hole. The cell
descriptor describes a robot -- links, joints, axes, position limits -- and
says nothing about how it is USED. `program.json` describes a weld and is
forbidden from mentioning a robot at all. Neither can say that J9 swaps bays
rather than welding, that this arm decelerates at 8 rad/s^2, or that a reset
pose exists and is where every seam starts.

So a profile is COMPUTED, from the two documents that are allowed to say it:
the model's own `config.json` and, on top of it, the target's `machine_planning_calibration.json` --
by the OLP, once, into the planning context's `planner.json`
(docs/PLANNING-CONTEXT.md), which `for_cell` reads. It used to be a third
file of its own, `data/motion/cells/<cell>.profile.json`, beside the planner
rather than beside the robot, and then a merge this side re-derived on every
plan from files it found by model name -- both the arrangement the Robot
Description Contract exists to end: a robot with two descriptions has no
description. `load` still reads a model's `config.json` from a checkout, for
the tools and tests that check a description on disk.

## What belongs in the description, and the test for it

A fact belongs in `config.json` when it is true of THIS ROBOT and is not in
the URDF. Axis roles pass: nothing in a URDF distinguishes an index axis from
a positioner. Acceleration passes: it is nowhere in the format. The reset pose
passes: it is a choice, not a measurement. The planning velocity passes as a
POLICY -- the URDF's `<limit velocity>` is a rating, and what the planner
parametrizes against is allowed to be lower, never higher.

The arm's axis list FAILS the test, and deliberately. It is derivable --
`cell.axis_roles` walks the chain to the tool -- so restating it would create
a second description that drifts from the first. A description OVERRIDES
structure; it never repeats it.

## Two velocities, and which document owns each

`velocity_rad_s` is what the planner may parametrize against, from the model's
`config.json`. `enforced_velocity_rad_s` is what the CELL clamps at -- the
ceiling the dense header declares -- and that is a fact about one cell, so it
comes from its `machine_planning_calibration.json` and falls back to the planning rate when the cell
says nothing. The gap between them is the joint daemon's follower authority to
close a lag.

## Missing is a state, not an error

A model that has never been fitted out still loads. `problems` says what is
absent and which milestone needs it, because "this cell has no acceleration
limits yet" is a task, and a reader that raised on it would make every early
run look like a broken configuration instead.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from weld_motion_planner.robot_cell.robot import description
from weld_motion_planner.robot_cell.robot.cell import CellError, Pose, axis_roles

# Where a model's assets are, re-exported so every caller that already asks
# this module for the meshes keeps working: the description owns the layout.
URDF_ROOT = description.DESCRIPTION_ROOT
mesh_root = description.mesh_root


@dataclass(frozen=True)
class CellProfile:
    """One cell's setup. Every field optional; absence is reported, not raised."""

    cell_id: str
    driven: list[str] = field(default_factory=list)
    held: dict[str, float] = field(default_factory=dict)
    limits: dict[str, dict[str, float]] = field(default_factory=dict)
    reset_pose: dict[str, float] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    path: Path | None = None

    def pose_from(self, driven_values: dict[str, float]) -> Pose:
        """A full pose: the axes the planner drove, plus the ones it did not.

        The single place held axes get reattached. Everything downstream --
        forward kinematics, collision, the verifier -- takes a pose over
        EVERY movable axis, so that a held index cannot be silently absent
        and default to zero.
        """
        unknown = sorted(set(driven_values) - set(self.driven))
        if unknown:
            raise CellError(f"{', '.join(unknown)} is not a driven axis of {self.cell_id!r}")
        missing = sorted(set(self.driven) - set(driven_values))
        if missing:
            raise CellError(f"no value for driven axis {', '.join(missing)}")
        return Pose({**self.held, **driven_values})


def load(model_id: str, root: Path | None = None) -> CellProfile | None:
    """The MODEL's setup, merged from its description, or None when that model
    has no `config.json`.

    `root` is the description root (`robot_description/`), not a profile directory: there
    is no profile file any more. What comes back is the model alone -- no cell
    has been applied -- so `enforced_velocity_rad_s` is the planning rate, the
    only honest stand-in for a number the model cannot know. `for_cell` puts
    the target's own on top.
    """
    found = description.config(str(model_id), root)
    if found is None:
        return None
    limits = {
        axis: {
            "velocity_rad_s": spec["velocity_rad_s"],
            "acceleration_rad_s2": spec["acceleration_rad_s2"],
            "enforced_velocity_rad_s": spec["velocity_rad_s"],
        }
        for axis, spec in found.planning.items()
    }
    return CellProfile(
        cell_id=str(model_id),
        driven=list(found.driven),
        held=dict(found.held),
        limits=limits,
        reset_pose=dict(found.reset_pose),
        path=description.config_path(str(model_id), root),
    )


def for_cell(cell: dict[str, Any], root: Path | None = None) -> CellProfile:
    """The profile of the cell a plan is FOR: the model's own numbers, narrowed
    by the cell's own, as the OLP compiled them into the planning context's
    `planner.json` (docs/PLANNING-CONTEXT.md).

    Every planner reads rates and holds through here rather than `load`,
    because a cell that lowered J2 to 0.05 rad/s and was honoured by the screen
    but not by the smoother would plan a motion the daemon then refuses -- or
    worse, one whose header ceiling was the model's. Position limits and joint
    origins are not here: they are already on the descriptor's joints, which
    were parsed from the context's corrected URDF.

    The merge itself -- the enforced rate the cell's own, the planning rate
    never above it or the ceiling, a hold only ever a positioner joint -- is
    the compiler's (`internal/robotconfig.Compile`) and is tested there. This
    side confirms the hold against the descriptor's chains, since a lattice
    built over a held arm axis fails three stages later. A request with no
    context is refused: its rates are unknown, and `root` no longer names a
    place to look them up.
    """
    from weld_motion_planner.robot_cell import context as contextmod

    compiled = contextmod.of(cell)
    if compiled is None:
        raise CellError(
            f"cell {cell.get('id')!r} arrived without a planning context, so its rates and "
            "holds are unknown; the request predates docs/PLANNING-CONTEXT.md -- re-pack it")
    return compiled.profile(cell)


def problems(profile: CellProfile | None, cell: dict[str, Any]) -> list[str]:
    """What is wrong or missing, as sentences naming the milestone that needs it.

    Checked against the descriptor rather than in isolation, because every
    interesting failure is a disagreement between the two: an axis the
    profile drives that the cell does not have, a work-chain axis it forgot
    to say anything about, a reset pose outside a joint's own limits.
    """
    out: list[str] = []
    cell_id = str(cell.get("id") or "?")
    if profile is None:
        return [
            (
                f"cell {cell_id!r} has no profile; write "
                f"{description.config_path(cell_id)} (axis roles, planning rates, reset pose, torch)"
            )
        ]
    if profile.cell_id != cell_id:
        out.append(f"the profile is for cell {profile.cell_id!r} and this request carries {cell_id!r}")

    roles = axis_roles(cell)
    movable = set(roles["movable"])
    driven = set(profile.driven)
    held = set(profile.held)

    for name in sorted(driven - movable):
        out.append(f"the profile drives {name!r}, which is not a movable axis of this cell")
    for name in sorted(held - movable):
        out.append(f"the profile holds {name!r}, which is not a movable axis of this cell")
    both = sorted(driven & held)
    if both:
        out.append(f"{', '.join(both)} is both driven and held; it can only be one")

    unaccounted = sorted(movable - driven - held)
    if unaccounted:
        # The dangerous omission. An axis nobody mentioned gets no value, and
        # the cheapest wrong answer is to call it zero -- which for an index
        # axis means planning against a bay that is not facing the robot.
        out.append(
            f"{', '.join(unaccounted)} is neither driven nor held, so it would have no value; "
            "an unmentioned axis is how a part ends up somewhere it is not"
        )

    missing_arm = sorted(set(roles["arm"]) - driven)
    if missing_arm:
        out.append(
            f"the arm axes {', '.join(missing_arm)} are not driven; the torch cannot be posed "
            "without them"
        )
    # Only when there IS a work-chain axis to drive. A cell with none has no
    # positioner at all -- `rosie_1420_v1` is an arm on a pedestal welding
    # parts fixtured to the floor -- and calling that a problem made every
    # profile written for such a cell report a fault it can never clear.
    if roles["work"] and not (driven & set(roles["work"])):
        out.append(
            "no work-chain axis is driven, so the positioner is fixed and the planner has one "
            "degree of redundancy instead of two"
        )

    limits = {name: profile.limits.get(name) or {} for name in sorted(driven)}
    without_speed = [name for name, spec in limits.items() if not spec.get("velocity_rad_s")]
    without_accel = [
        name for name, spec in limits.items() if not spec.get("acceleration_rad_s2")
    ]
    if without_speed:
        out.append(
            f"no velocity limit for {', '.join(without_speed)}; M7 parametrizes "
            "against it and T2 checks it"
        )
    if without_accel:
        out.append(
            f"no acceleration limit for {', '.join(without_accel)}; acceleration "
            "binds before velocity on a weld and T2 checks it"
        )

    if not profile.reset_pose:
        out.append("no reset pose; every seam is planned from one and back to one (M6)")
    else:
        stated = set(profile.reset_pose)
        if not driven <= stated:
            out.append(
                f"the reset pose does not give a value for {', '.join(sorted(driven - stated))}"
            )
        out.extend(_outside_limits(cell, profile.reset_pose))

    out.extend(_outside_limits(cell, profile.held, what="the held value"))
    return out


def _outside_limits(
    cell: dict[str, Any], values: dict[str, float], what: str = "the reset pose"
) -> list[str]:
    """Values a joint's own declared range does not permit."""
    out: list[str] = []
    for joint in cell.get("joints") or []:
        name = str(joint.get("name"))
        if name not in values:
            continue
        span = joint.get("limits_rad") or {}
        if not span:
            continue
        value = values[name]
        if not math.isfinite(value):
            out.append(f"{what} for {name!r} is not a finite number")
        elif not (float(span["lower"]) - 1e-9 <= value <= float(span["upper"]) + 1e-9):
            out.append(
                f"{what} puts {name!r} at {value:.4f} rad, outside its limits "
                f"[{span['lower']:.4f}, {span['upper']:.4f}]"
            )
    return out


def moved_axes(cell: dict[str, Any]) -> list[str]:
    """The axes a planner MOVES on this cell, arm first and the positioner LAST.

    Derived, not stated: `axis_roles` says which movable axes reach the tool and
    which reach the work, and this cell's profile says which of the work ones are
    HELD -- a station index the robot does not weld through. What is left of the
    work chain is the positioner.

    Last is load-bearing: M4 sizes its spin bins from the last entry's step
    limit. `cell["axes"]` cannot be used instead -- on `rosie_1400_v3` it lists
    nine, held ones included, and puts J9 before J7.

    **ZERO is a cell, not a misconfiguration.** `rosie_1420_v1` is an arm on a
    pedestal and the part is fixtured to the floor, so there is no positioner to
    move and this returns the arm alone. What stays refused is TWO or more:
    which of them a search sweeps is not decidable from the cell, and guessing
    is how a screen over one axis reads as a screen over the whole positioner
    (`docs/issues/0003`).
    """
    roles = axis_roles(cell)
    held = for_cell(cell).held
    live = [name for name in roles["work"] if name not in held]
    if len(live) > 1:
        raise CellError(
            f"this cell has {len(live)} live positioner axes ({live}); a planner "
            "sweeps at most one, so which is not decidable from the cell alone")
    return [*roles["arm"], *live]


def self_test() -> int:
    cell = {
        "id": "c",
        "frames": {"base": "floor", "tool": "tool0", "work": "table_a"},
        "links": [{"name": n} for n in ("floor", "l1", "tool0", "h", "table_a", "table_b")],
        "joints": [
            {"name": "J1", "type": "revolute", "parent": "floor", "child": "l1",
             "limits_rad": {"lower": -3.0, "upper": 3.0}},
            {"name": "fix", "type": "fixed", "parent": "l1", "child": "tool0"},
            {"name": "J9", "type": "revolute", "parent": "floor", "child": "h",
             "limits_rad": {"lower": -3.2, "upper": 3.2}},
            {"name": "J7", "type": "revolute", "parent": "h", "child": "table_a",
             "limits_rad": {"lower": -3.0, "upper": 3.0}},
            {"name": "J8", "type": "revolute", "parent": "h", "child": "table_b",
             "limits_rad": {"lower": -3.0, "upper": 3.0}},
        ],
    }
    complete = CellProfile(
        cell_id="c",
        driven=["J1", "J7"],
        held={"J9": 0.0, "J8": 0.5},
        limits={
            "J1": {"velocity_rad_s": 3.1, "acceleration_rad_s2": 12.0},
            "J7": {"velocity_rad_s": 1.0, "acceleration_rad_s2": 4.0},
        },
        reset_pose={"J1": 0.0, "J7": 0.0},
    )
    assert problems(complete, cell) == [], problems(complete, cell)

    # The pose a driven-only dict expands into carries the held axes too.
    posed = complete.pose_from({"J1": 0.4, "J7": -0.2})
    assert posed.values == {"J1": 0.4, "J7": -0.2, "J9": 0.0, "J8": 0.5}, posed.values

    # The omission that matters: an axis nobody mentioned.
    forgetful = replace(complete, held={"J9": 0.0})
    assert any("neither driven nor held" in p for p in problems(forgetful, cell))

    out_of_range = replace(complete, held={"J9": 9.0, "J8": 0.0})
    assert any("outside its limits" in p for p in problems(out_of_range, cell))

    bare = replace(complete, held={"J9": 0.0, "J8": 0.0}, limits={}, reset_pose={})
    said = " ".join(problems(bare, cell))
    assert "velocity limit" in said and "acceleration limit" in said and "reset pose" in said

    assert problems(None, cell)[0].startswith("cell 'c' has no profile")

    try:
        complete.pose_from({"J1": 0.0})
    except CellError as exc:
        assert "no value for driven axis J7" in str(exc), exc
    else:
        raise AssertionError("a pose missing a driven axis was accepted")

    print(
        "cell profile self-test ok: roles reattach held axes, an unmentioned axis is caught, "
        "and what is merely missing is reported rather than raised"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose cell profile to check")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise CellError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read

    cell = read(Path(ns.read)).cell
    found = for_cell(cell)
    print(
        json.dumps(
            {
                "cell_id": cell.get("id"),
                "profile": str(found.path) if found else None,
                "driven": found.driven if found else [],
                "held": found.held if found else {},
                "problems": problems(found, cell),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"profile: {exc}", file=sys.stderr)
        raise SystemExit(1)
