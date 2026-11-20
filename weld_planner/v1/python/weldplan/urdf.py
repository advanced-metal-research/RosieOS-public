#!/usr/bin/env python
"""A URDF read as DATA into a cell descriptor, and the descriptor a planning
context yields.

`amr-weld-planner-v1.cell.v1`: links with their mesh references, joints with
their origin, axis and limits, and three NAMED frames that the rest of the
application asks for by role rather than by link name:

    base    the frame everything is measured in            (`world`)
    tool    where the torch is mounted                     (`tool0`)
    work    the surface a workpiece is placed on           (`positioner_table_a_top`)

Those roles are the whole reason this document exists rather than a reader
opening the URDF directly. `positioner_table_a` is a fact about one Fusion
export; "the frame a workpiece is fixtured to" is a fact about welding, and
only the second is safe to write into a placement record.

## Where it is read from

The PLANNING CONTEXT (`docs/PLANNING-CONTEXT.md`). The OLP compiles the robot
once -- the cell's corrections and narrowed limits already on the URDF, the
frames beside it in `planner.json`, the identity in `identity.json` -- and
`descriptor_of` parses that URDF and stamps that identity. Nothing here
applies a deviation: what is composed was composed before the file got here,
and a parser that also corrected would be a second answer to "which robot".

The seam worker's tools (`seam_worker.robot.cell`) read a checkout's URDFs
through the same parser, for browsing; the planning path never does.

## Read as data, not imported

`offline-programming/v1` ships a URDF bundle and a viewer for it. GOAL.md
forbids importing that module's CODE, and this does not: it parses the URDF
with `xml.etree` from the standard library. Nothing here calls into their
TypeScript, and their file could be replaced by any other URDF tomorrow.

The parser is deliberately small and refuses what it does not understand. A
URDF feature this does not implement -- a prismatic joint, a mimic, a
`<geometry>` that is a primitive rather than a mesh -- is reported as a
`problem` rather than silently dropped, because a link quietly missing from a
cell is a collision nobody sees coming.

## Units and conventions

URDF is metres and radians, right-handed, Z-up -- the same as the weld
program, so nothing is converted here. `rpy` is the URDF convention:
fixed-axis roll-pitch-yaw, applied X then Y then Z, which composes as
`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

SCHEMA = "amr-weld-planner-v1.cell.v1"

# Which link plays which role -- what everything is measured in, where the
# torch is, and what a workpiece is fixtured to. The URDF has no vocabulary
# for "the table a workpiece goes on", so the description states it in
# `config.json` under `frames`, and it ARRIVES HERE: the caller reads the
# description (the OLP does, once) and passes it in. This module resolves
# nothing by cell id, because a name resolved twice is a name that disagrees
# with itself.
#
# `work_surface` is a work frame that is a point ON a link rather than the
# link's own origin. It is added to the descriptor as a fixed child so
# everything that walks joints -- FK, the verifier, the positioner sweep --
# carries it without knowing. `positioner_table_a`'s origin is on J7's axis,
# the bar's centreline, while a part is placed from the middle of its TOP
# FACE; a work frame at the link origin read every offset 622 mm along the
# axis and 100 mm down, which put the part inside the steel and refused every
# pose.

# Bodies a cell has that its URDF does not carry, as fixed children of a link
# it does. The workpiece stands on something, and on a cell with no positioner
# that something is not in the robot's own model: `rosie_1420_v1` is an arm on a
# pedestal and its URDF describes the arm and the mount, nothing else. Without a
# fixture the planner believes there is clear air under the part and will route
# the forearm straight through whatever is really holding it.
#
# PLACEHOLDER, and it says so in the name. Nobody has measured the real
# fixture; this is the box the owner asked for on 2026-09-13 -- 0.5 m square in
# plan, 1.0 m tall, standing on the floor under where the part is placed. Its
# top face is at z = 1.000 m, which is where that part's underside sits, and its
# centre is 0.84 m horizontally from the arm's J1 origin.
#
# It is added to the DESCRIPTOR and not to `robot.urdf` on purpose. That file is
# embedded byte for byte in the RTCore nats bridge and its sha256 is pinned in
# the pair config, so a link added there desyncs the pair's home profile key and
# the cell stops arming. A body the planner must avoid is not a claim about the
# robot's kinematics, and this is the seam where the two are allowed to differ.
#
# Since 2026-09-20 the OLP carries the same stand as an editable workcell
# fixture (`fixtures.json` in the .weldplan, drawn in its viewport, changed or
# replaced by a STEP by the operator), and a request that carries that list
# gets NO placeholder from here: `read_urdf(..., fixtures=[...])`. This table
# stands only for a request that says nothing about fixtures, and its box
# must stay equal to the OLP's default (`ui/src/domain/workcell.ts`).
CELL_FIXTURES: dict[str, list[dict[str, Any]]] = {
    "rosie_1420_v1": [
        {
            "link": "fixture_placeholder",
            "parent": "world",
            "xyz_m": [1.0, 1.0, 0.0],
        },
    ],
}

# Joint types this understands. `prismatic` is absent because no cell here has
# one and an untested branch is worse than a refusal; `continuous` is absent
# for the same reason, though it would be two lines.
SUPPORTED_JOINTS = ("revolute", "fixed")


class CellError(ValueError):
    """A cell this refuses to read, with the reason."""


def _floats(text: str | None, count: int, fallback: tuple[float, ...]) -> list[float]:
    if not text:
        return list(fallback)
    parts = text.replace(",", " ").split()
    if len(parts) != count:
        raise CellError(f"expected {count} numbers, got {text!r}")
    return [float(p) for p in parts]


def _origin(node: ET.Element | None) -> dict[str, list[float]]:
    """A URDF `<origin>`, defaulting to identity when absent."""
    element = None if node is None else node.find("origin")
    return {
        "xyz_m": _floats(None if element is None else element.get("xyz"), 3, (0.0, 0.0, 0.0)),
        "rpy_rad": _floats(None if element is None else element.get("rpy"), 3, (0.0, 0.0, 0.0)),
    }


def _mesh_name(filename: str) -> str:
    """`package://rosie_1400_v3/meshes/link_1.dae` -> `meshes/link_1.dae`.

    The package prefix is a ROS resource locator and means nothing to a
    browser. What survives is the path RELATIVE TO THE BUNDLE, which is what
    the asset mount serves -- so a descriptor stays valid however the bundle is
    hosted.
    """
    tail = filename.split("://", 1)[-1]
    # Drop the package name, which is the bundle directory itself.
    return tail.split("/", 1)[1] if "/" in tail else tail


def frames_beside(path: Path) -> dict[str, Any] | None:
    """The `frames` of the description this URDF belongs to, for a caller that
    is browsing the checkout rather than answering a request.

    The planning path does NOT come through here: the OLP reads the
    description once and passes the block in. This is for the tools that walk
    `robot_description/` themselves.
    """
    config = path.parent / "config.json"
    if not config.is_file():
        return None
    try:
        return (json.loads(config.read_text(encoding="utf-8")) or {}).get("frames")
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


def read_urdf(path: Path | bytes, frames: dict[str, Any] | None = None,
              filename: str = "robot.urdf", *,
              fixtures: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Parse a URDF into a cell descriptor. Unsupported features are reported.

    `path` is a file, or the file's bytes (a context member) with `filename`
    for the messages and the descriptor's `source`.

    `frames` is the description's own `config.json` block: which link is the
    base, which is the tool, and what a workpiece is fixtured to. Without it a
    cell parses and has no work frame, which `problems` says out loud --
    the same answer as a cell whose description never stated one.

    Never raises for a feature it cannot model -- those become `problems`, so a
    cell with one odd joint still shows its other twelve links rather than
    failing whole. It raises only when the file is not a URDF at all.
    """
    if isinstance(path, bytes | bytearray):
        source, fallback_id = bytes(path), ""
    else:
        source, filename, fallback_id = Path(path).read_bytes(), Path(path).name, Path(path).parent.name
    try:
        root = ET.fromstring(source)
    except ET.ParseError as exc:
        raise CellError(f"{filename} is not parseable XML: {exc}") from exc
    if root.tag != "robot":
        raise CellError(f"{filename} has root <{root.tag}>, expected <robot>")

    cell_id = root.get("name") or fallback_id
    problems: list[str] = []

    materials: dict[str, list[float]] = {}
    for node in root.findall("material"):
        colour = node.find("color")
        name = node.get("name")
        if name and colour is not None:
            materials[name] = _floats(colour.get("rgba"), 4, (0.8, 0.8, 0.8, 1.0))

    links: list[dict[str, Any]] = []
    for node in root.findall("link"):
        name = node.get("name")
        if not name:
            problems.append("a <link> has no name and was skipped")
            continue
        visuals: list[dict[str, Any]] = []
        for visual in node.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None or not mesh.get("filename"):
                # A box or a cylinder. Legal URDF, not modelled here, and the
                # link would render as nothing -- so it is named rather than
                # left as a hole in the cell.
                problems.append(f"link {name!r} has a non-mesh visual, which is not drawn")
                continue
            material = visual.find("material")
            visuals.append(
                {
                    "mesh": _mesh_name(mesh.get("filename", "")),
                    **_origin(visual),
                    "rgba": materials.get(
                        (material.get("name") if material is not None else "") or "",
                        [0.75, 0.78, 0.82, 1.0],
                    ),
                }
            )
        links.append({"name": name, "visuals": visuals})

    joints: list[dict[str, Any]] = []
    for node in root.findall("joint"):
        name = node.get("name") or ""
        kind = node.get("type") or ""
        # `is None`, NOT `or`. An ElementTree element with no children is
        # FALSY, so `node.find("parent") or fallback` discards a perfectly good
        # `<parent link="floor"/>` -- every joint in the real bundle vanished
        # and the parser reported twelve links "not connected to the base"
        # while looking like it had read the file. The synthetic fixture had
        # the same shape and failed the same way, which is the only reason it
        # was caught before the viewport drew a robot in a heap at the origin.
        parent_node = node.find("parent")
        child_node = node.find("child")
        parent = None if parent_node is None else parent_node.get("link")
        child = None if child_node is None else child_node.get("link")
        if not (name and parent and child):
            problems.append(f"joint {name or '?'!r} is missing a name, parent or child")
            continue
        if kind not in SUPPORTED_JOINTS:
            # Reported, and the child link is then unreachable from the base --
            # which `problems` says out loud rather than leaving a limb to
            # render at the origin.
            problems.append(
                f"joint {name!r} is {kind!r}; only {', '.join(SUPPORTED_JOINTS)} are modelled, "
                f"so {child!r} and anything below it will not be placed"
            )
            continue
        limit = node.find("limit")
        joints.append(
            {
                "name": name,
                "type": kind,
                "parent": parent,
                "child": child,
                **_origin(node),
                "axis_xyz": _floats(
                    (node.find("axis").get("xyz") if node.find("axis") is not None else None),
                    3,
                    (0.0, 0.0, 1.0),
                ),
                "limits_rad": (
                    None
                    if limit is None
                    else {
                        "lower": float(limit.get("lower", "0") or 0.0),
                        "upper": float(limit.get("upper", "0") or 0.0),
                    }
                ),
                # The design rate the URDF states. Carried because a reader of
                # a descriptor should not have to open the URDF to learn what
                # the arm is RATED for -- what a given cell enforces is a
                # different number and arrives with that cell's settings.
                **(
                    {"velocity_rad_s": float(limit.get("velocity"))}
                    if limit is not None and limit.get("velocity")
                    else {}
                ),
            }
        )

    surface = frames.get("work_surface") if frames else None
    if surface is not None and any(link["name"] == surface["parent"] for link in links):
        links.append({"name": frames["work"], "visuals": []})
        joints.append(
            {
                "name": f"{frames['work']}_fixed",
                "type": "fixed",
                "parent": surface["parent"],
                "child": frames["work"],
                "xyz_m": list(surface["xyz_m"]),
                "rpy_rad": [0.0, 0.0, 0.0],
                "axis_xyz": [0.0, 0.0, 1.0],
                "limits_rad": None,
            }
        )

    # A request that names its fixtures gets none of the placeholders.
    placeholders = () if fixtures is not None else CELL_FIXTURES.get(cell_id, ())
    for fixture in placeholders:
        if not any(link["name"] == fixture["parent"] for link in links):
            problems.append(
                f"the fixture {fixture['link']!r} hangs off {fixture['parent']!r}, "
                "which this cell does not have"
            )
            continue
        links.append({"name": fixture["link"], "visuals": []})
        joints.append(
            {
                "name": f"{fixture['link']}_fixed",
                "type": "fixed",
                "parent": fixture["parent"],
                "child": fixture["link"],
                "xyz_m": list(fixture["xyz_m"]),
                "rpy_rad": [0.0, 0.0, 0.0],
                "axis_xyz": [0.0, 0.0, 1.0],
                "limits_rad": None,
            }
        )

    roles = {role: str(name) for role, name in (frames or {}).items()
             if role in ("base", "tool", "work") and name}
    known = {link["name"] for link in links}
    for role, link_name in list(roles.items()):
        if link_name not in known:
            problems.append(
                f"the {role} frame names link {link_name!r}, which this cell does not have"
            )
            roles.pop(role)
    if "work" not in roles:
        problems.append(
            "no work frame: this cell does not say which link a workpiece is "
            "fixtured to, so nothing can be placed on it"
        )

    document = {
        "schema": SCHEMA,
        "id": cell_id,
        "source": {"filename": filename},
        "frames": roles,
        "links": links,
        "joints": joints,
        # Only the movable ones, in file order, which is the order an operator
        # reads them as J1..J9.
        "axes": [j["name"] for j in joints if j["type"] == "revolute"],
        "problems": problems,
    }
    return document


# The members a planning context is read from (`docs/PLANNING-CONTEXT.md`).
CONTEXT_URDF = "robot.urdf"
CONTEXT_PLANNER = "planner.json"
CONTEXT_IDENTITY = "identity.json"
_IDENTITY_KEYS = ("model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "machine_planning_calibration_source")

# Carry machine provenance and the native execution-limit snapshot into the
# descriptor; the latter distinguishes the planner's registered contexts.
_OPTIONAL_IDENTITY_KEYS = ("machine_configuration_sha256", "execution_limits_sha256")


def descriptor_of(context: dict[str, bytes], *,
                  fixtures: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """The cell descriptor a planning context yields: its URDF parsed with the
    frames its `planner.json` names, stamped with the identity its
    `identity.json` carries -- the pair the dense header repeats and the joint
    daemon fences on.

    The URDF is already the CELL's: origins composed, limits narrowed, by the
    OLP. So the joints read here are the corrected joints, and every stage
    that walks them plans the corrected robot without a second application.

    `fixtures` is the request's own workcell list; when it is given, even
    empty, the descriptor carries none of `CELL_FIXTURES`' placeholders.
    """
    for member in (CONTEXT_URDF, CONTEXT_PLANNER, CONTEXT_IDENTITY):
        if member not in context:
            raise CellError(f"the planning context has no {member}")
    try:
        planner = json.loads(context[CONTEXT_PLANNER])
        identity = json.loads(context[CONTEXT_IDENTITY])
    except ValueError as exc:
        raise CellError(f"the planning context's numbers are not JSON: {exc}") from exc
    if not isinstance(planner, dict) or not isinstance(identity, dict):
        raise CellError("planner.json and identity.json must be objects")
    for key in _IDENTITY_KEYS:
        if not isinstance(identity.get(key), str) or not identity[key]:
            raise CellError(f"identity.json.{key} must be a non-empty string")
    frames = planner.get("frames")
    cell = read_urdf(context[CONTEXT_URDF], frames if isinstance(frames, dict) else None,
                     fixtures=fixtures)
    if cell["id"] != identity["model_id"]:
        raise CellError(
            f"the context's URDF names robot {cell['id']!r} and its identity says "
            f"{identity['model_id']!r}")
    cell["robot_cell"] = {key: str(identity[key]) for key in _IDENTITY_KEYS}
    for key in _OPTIONAL_IDENTITY_KEYS:
        if isinstance(identity.get(key), str) and identity[key]:
            cell["robot_cell"][key] = identity[key]
    return cell
