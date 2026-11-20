#!/usr/bin/env python
"""What the planner was handed, and what it still cannot plan from.

`weldplan.plan_request` already refuses to BUILD a request that is malformed --
no placement, a program in the wrong frame, a placement authored against a
different cell. This module asks the other question, from the other side:
given a request that is perfectly well-formed, is there enough here for THIS
planner to work with?

The two are different in kind and both are needed. The producer cannot know
what a consumer requires -- that is the whole point of a contract narrow
enough that two planners can read it and disagree about method. So the
consumer states its own preconditions, once, in one place, and says them as
sentences rather than failing later with a stack trace three stages in.

## Three kinds of gap, deliberately separated

`blocking` -- the request cannot be planned by anybody. No kinematic chain,
no axes, a seam with no orientation profile. These are wrong.

`assets` -- the request is fine and this cell has not been fitted out yet.
Joint velocity and acceleration limits, the sphere model, the reset pose:
none of them belong in the plan request (they are facts about the robot and its
planner, not about the weld or the placement), and all of them must exist
before a trajectory can be emitted. `ARCHITECTURE.md` calls these per-cell
assets. Listing them apart from the blocking problems is what stops "this
cell has never been set up" reading as "this weld program is broken".

`notes` -- true, worth knowing, not an obstacle. A seam that weaves is
planned as its centreline, because this planner does not handle weave; that
is a real approximation and it is said out loud, but it does not stop a
plan.

## The URDF has no velocity limits, and that is not an oversight here

`cell.json` carries `limits_rad` with `lower` and `upper` only. Position
limits screen the graph; velocity and acceleration limits are what the
timing stage parametrizes against and what T2 checks. So every cell needs a
limits asset before M7, and this module is where that becomes visible on day
one rather than at the end of the pipeline.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from weldplan.plan_request import EXTENSION, PlanRequestError, read_request

# Five constraints come from the weld and one spin DOF is free, so six movable
# axes is the point below which a cell cannot even express a weld pose, let
# alone have redundancy to spend on reach and collision.
MINIMUM_AXES = 6


@dataclass(frozen=True)
class PlanRequest:
    """One `.weldplan`, opened and verified, with its parts named.

    A frozen dataclass rather than the raw dict `read_request` returns,
    because every stage downstream reaches for the same five documents and
    `opened["program"]` spelled out forty times is forty chances to reach for
    `opened["programs"]` instead. The documents themselves stay plain dicts:
    they are the contract, and wrapping them in types of our own would mean
    maintaining a second description of somebody else's schema.
    """

    manifest: dict[str, Any]
    program: dict[str, Any]
    placement: dict[str, Any]
    cell: dict[str, Any]
    step_bytes: bytes | None
    path: Path | None = None
    # The operator's workcell bodies (`fixtures.json`), or None when the
    # request says nothing about them; see `robot_cell.collision.fixtures`.
    fixtures: list[dict[str, Any]] | None = None
    # The planning context the container carried, registered so every stage
    # that holds only the descriptor finds it by identity
    # (`robot_cell.context.of`). None on a request from before the context.
    context: Any = None
    start_state: dict[str, Any] | None = None
    # The connected cell's outputs (`io_outputs.json`, weldplan.io_schedule),
    # when the program switches any.
    io_outputs: dict[str, Any] | None = None

    @property
    def request_id(self) -> str:
        return str(self.manifest.get("request_id") or "")

    @property
    def seams(self) -> list[dict[str, Any]]:
        """The seams to weld, in the order the program states them.

        program.v2 is a list of NODES, and a seam is the geometry of a weld
        node. Two things are put back together here rather than at the forty
        places downstream that read a seam. `process` is reassembled from the
        node, because D23 moved `travel_speed_mm_s`, `standoff_mm` and
        `weave` up there -- the planner times the weld from the first, places
        the tip from the second and reserves mobility for the third, so all
        three belong to the weld and not to the shape of its line.

        Reassembled EXHAUSTIVELY, which is the point. This rebuilt the block
        from two of the three, so `dp_seam_search.engine.nodes` refused every
        seam for authoring no standoff -- while the document carried one it
        could not see. A plan came back 200 with every seam errored and no
        trajectory in it.
        And a `posed_polyline` weld is skipped: the seam search wants a 5-DOF
        band, not a 6-DOF pose list, and that flow plans elsewhere.
        """
        out: list[dict[str, Any]] = []
        for node in self.program.get("nodes") or []:
            if not isinstance(node, dict) or node.get("op") != "weld":
                continue
            geometry = node.get("geometry")
            if not isinstance(geometry, dict) or geometry.get("kind") != "seam":
                continue
            seam = {k: v for k, v in geometry.items() if k != "kind"}
            seam["process"] = {
                "travel_speed_mm_s": node.get("travel_speed_mm_s"),
                "weave": node.get("weave"),
                "standoff_mm": node.get("standoff_mm"),
            }
            out.append(seam)
        return out

    @property
    def freespace_nodes(self) -> list[dict[str, Any]]:
        """Every move between the welds, as the program states them.

        The planner does not decide that an approach or a transit exists. D23
        made the alternation the validity test -- `home? approach weld
        (transit weld)* retract home?` -- and D25 took the last of the
        inventing away from M6, which used to read a weld count and produce a
        move list from it. A gap the producer did not state is a document
        problem, not a hole for a planner to fill: the arm is not to make a
        move nobody asked for.
        """
        return [
            node
            for node in self.program.get("nodes") or []
            if isinstance(node, dict)
            and node.get("op") in ("approach", "transit", "retract", "home")
        ]

    @property
    def weld_joints(self) -> list[dict[str, Any]]:
        """The joints the seams name. Under `workpiece` since D24, where the
        weld program keeps them at top level: a joint is an objective fact
        about the part, which is what `workpiece` holds."""
        return list((self.program.get("workpiece") or {}).get("weld_joints") or [])

    @property
    def joints_by_id(self) -> dict[str, dict[str, Any]]:
        """`weld_joints`, indexed the way every caller wanted it. A seam takes
        its line from here when it carries no curve of its own."""
        return {str(joint.get("id")): joint for joint in self.weld_joints}

    @property
    def axes(self) -> list[str]:
        """The movable joints, in the order an operator reads them as J1..Jn."""
        return [str(name) for name in (self.cell.get("axes") or [])]

    @property
    def step_filename(self) -> str:
        return str((self.manifest.get("source") or {}).get("filename") or "part.step")


def read(path: str | Path) -> PlanRequest:
    """Open a `.weldplan`. Every declared digest is checked on the way in."""
    where = Path(path)
    opened = read_request(where.read_bytes())
    return PlanRequest(
        manifest=opened["manifest"],
        program=opened["program"],
        placement=opened["placement"],
        cell=opened["cell"],
        step_bytes=opened["step_bytes"],
        path=where,
        fixtures=opened.get("fixtures"),
        context=_context_of(opened),
        start_state=opened.get("start_state"),
        io_outputs=opened.get("io_outputs"),
    )


def _context_of(opened: dict[str, Any]) -> Any:
    from weld_motion_planner.robot_cell import context as contextmod

    return contextmod.from_members(opened.get("context"))


def _chain_problems(cell: dict[str, Any]) -> list[str]:
    """Whether forward kinematics can run at all on this descriptor."""
    out: list[str] = []
    frames = cell.get("frames") or {}
    links = {str(link.get("name")) for link in (cell.get("links") or [])}
    joints = cell.get("joints") or []

    for role in ("base", "tool", "work"):
        named = frames.get(role)
        if not named:
            out.append(
                f"the cell names no {role} frame, so there is no {'chain to the torch' if role == 'tool' else 'frame to plan against'}"
            )
        elif named not in links:
            out.append(f"the {role} frame names link {named!r}, which this cell does not have")

    parents = {str(j.get("child")): str(j.get("parent")) for j in joints}
    base = frames.get("base")
    for role in ("tool", "work"):
        target = frames.get(role)
        if not (base and target and target in links and base in links):
            continue
        seen: set[str] = set()
        node = str(target)
        while node in parents and node not in seen:
            seen.add(node)
            node = parents[node]
        if node != base:
            out.append(
                f"the {role} frame {target!r} does not connect back to the base frame "
                f"{base!r}, so its pose is undefined"
            )
    return out


def _limits_asset_note(cell: dict[str, Any]) -> str | None:
    """The velocity and acceleration limits a URDF `<limit>` usually omits."""
    joints = [j for j in (cell.get("joints") or []) if j.get("type") == "revolute"]
    unlimited = [str(j.get("name")) for j in joints if not (j.get("limits_rad") or {})]
    if unlimited:
        return (
            f"{len(unlimited)} of {len(joints)} axes carry no position limits "
            f"({', '.join(unlimited[:3])}{'...' if len(unlimited) > 3 else ''}); "
            "the graph cannot screen a joint whose range it does not know"
        )
    return None


def gaps(request: PlanRequest) -> dict[str, list[str]]:
    """What stands between this request and a trajectory, sorted by kind.

    Returns `blocking`, `assets` and `notes`. Empty `blocking` means the
    request is plannable in principle; empty `assets` too means this cell has
    been fitted out and a plan can actually be emitted.
    """
    blocking: list[str] = []
    assets: list[str] = []
    notes: list[str] = []

    blocking.extend(_chain_problems(request.cell))

    axes = request.axes
    if len(axes) < MINIMUM_AXES:
        blocking.append(
            f"the cell has {len(axes)} movable axes and a weld pose constrains five degrees "
            f"of freedom; {MINIMUM_AXES} is the minimum that can hold one"
        )

    for seam in request.seams:
        sid = seam.get("id", "?")
        orientation = seam.get("orientation") or {}
        for axis in ("work_angle_deg", "travel_angle_deg"):
            if not orientation.get(axis):
                blocking.append(
                    f"seam {sid!r} has no {axis} profile, so the electrode direction is undefined"
                )
        weave = (seam.get("process") or {}).get("weave") or {}
        if weave and weave.get("shape") not in (None, "none"):
            notes.append(
                f"seam {sid!r} weaves {weave.get('shape')} at {weave.get('amplitude_mm')} mm / "
                f"{weave.get('frequency_hz')} Hz, which this planner does not expand, "
                f"reserve mobility for, or verify -- it plans the centreline"
            )
    # Counted rather than listed. Fourteen seams with no band is one fact about
    # the program; fourteen identical sentences is a wall an operator scrolls
    # past, and the one line that differs gets lost in it.
    unbanded = [
        seam.get("id", "?")
        for seam in request.seams
        if not ((seam.get("orientation") or {}).get("limits") or {})
    ]
    if unbanded:
        notes.append(
            f"{len(unbanded)} of {len(request.seams)} seams declare no admissible band, so their "
            "angles are pinned to the authored profile and there is nothing to trade for reach"
        )

    # ---- per-cell assets. Not in the plan request, and deliberately so.
    limits = _limits_asset_note(request.cell)
    if limits:
        blocking.append(limits)
    cell_id = request.cell.get("id", "?")
    assets.append(
        f"cell {cell_id!r} needs joint velocity and acceleration limits; a URDF <limit> carries "
        "position only, and stage 7 parametrizes against the other two"
    )
    assets.append(f"cell {cell_id!r} needs a sphere model for its links, and one for the torch")
    assets.append(f"cell {cell_id!r} needs a reset pose, which every seam is planned from and to")

    return {"blocking": blocking, "assets": assets, "notes": notes}


def describe(request: PlanRequest) -> dict[str, Any]:
    """A summary an operator can read, and the CLI prints."""
    weaved = sum(
        1
        for seam in request.seams
        if ((seam.get("process") or {}).get("weave") or {}).get("shape") not in (None, "none")
    )
    banded = sum(1 for seam in request.seams if (seam.get("orientation") or {}).get("limits"))
    return {
        "request_id": request.request_id,
        "program_id": request.program.get("program_id"),
        "source": request.step_filename,
        "step_bytes": len(request.step_bytes or b""),
        "cell": request.cell.get("id"),
        "axes": request.axes,
        "frames": request.cell.get("frames") or {},
        "seams": len(request.seams),
        "seams_with_weave": weaved,
        "seams_with_bands": banded,
        "gaps": gaps(request),
    }


def self_test() -> int:
    """Built on a hand-made request, so this stays milliseconds and needs no
    fixture. The fixture plan requests are exercised by `tests/motion_planner/`."""
    from weldplan import program_v2
    from weldplan.plan_request import write_request

    # A minimal program.v2 with its one weld turned into a seam. `_minimal`
    # writes a `posed_polyline`, which is the classic flow's weld and not this
    # planner's: `seams` skips it, and a request with no seam is unplannable.
    program = program_v2._minimal()
    weld = next(node for node in program["nodes"] if node["op"] == "weld")
    weld["geometry"] = {
        "kind": "seam",
        "id": "seam_0001",
        "curve": {
            "segments": [
                {
                    "degree": 1,
                    "control_points_xyz_m": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                }
            ]
        },
        "reference": {"mode": "rail"},
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
        },
    }
    placement = program["placement"]
    joints = [
        {
            "name": f"J{i}",
            "type": "revolute",
            "parent": f"l{i - 1}",
            "child": f"l{i}",
            "limits_rad": {"lower": -3.0, "upper": 3.0},
        }
        for i in range(1, 7)
    ]
    joints.append(
        {"name": "table", "type": "fixed", "parent": "l0", "child": "t", "limits_rad": None}
    )
    cell = {
        "id": "c",
        "frames": {"base": "l0", "tool": "l6", "work": "t"},
        # `spare` is a link no joint reaches: the detached case below points a
        # frame at it. Every other link hangs off `l0`.
        "links": [
            {"name": n, "visuals": []}
            for n in ("l0", "l1", "l2", "l3", "l4", "l5", "l6", "t", "spare")
        ],
        "joints": joints,
        "axes": [f"J{i}" for i in range(1, 7)],
    }
    data = write_request(
        request_id="r",
        program=program,
        cell=cell,
        step_bytes=b"ISO-10303-21;",
        step_filename="part.step",
    )

    import io
    import zipfile

    opened = read_request(data)
    request = PlanRequest(
        manifest=opened["manifest"],
        program=opened["program"],
        placement=opened["placement"],
        cell=opened["cell"],
        step_bytes=opened["step_bytes"],
    )
    found = gaps(request)
    assert not found["blocking"], found["blocking"]
    assert any("velocity" in note for note in found["assets"]), found["assets"]
    assert any("admissible band" in note for note in found["notes"]), found["notes"]

    # A cell whose tool frame is a link no joint reaches. Well-formed as an
    # plan request -- the producer has no reason to object -- and unplannable,
    # which is exactly the class of thing this module exists to catch.
    adrift = {**cell, "frames": {**cell["frames"], "tool": "spare"}}
    detached = PlanRequest(
        manifest={}, program=program, placement=placement, cell=adrift,
        step_bytes=b"",
    )
    assert any("does not connect" in p for p in gaps(detached)["blocking"]), gaps(detached)

    bare = {**weld, "geometry": {"kind": "seam", "id": "s", "orientation": {}}}
    no_angles = {**program, "nodes": [bare]}
    blind = PlanRequest(
        manifest={}, program=no_angles, placement=placement, cell=cell,
        step_bytes=b"",
    )
    assert len([p for p in gaps(blind)["blocking"] if "electrode direction" in p]) == 2

    assert zipfile.ZipFile(io.BytesIO(data)).namelist()[0] == "mimetype"
    print(
        "plan request self-test ok: a well-formed plan request this planner can use, "
        "one it cannot, and the per-cell assets named apart from both"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help=f"a {EXTENSION} to describe")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise PlanRequestError("--read or --self-test")
    print(json.dumps(describe(read(ns.read)), indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"request: {exc}", file=sys.stderr)
        raise SystemExit(1)
