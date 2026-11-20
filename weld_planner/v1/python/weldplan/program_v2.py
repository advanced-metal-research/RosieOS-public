"""`robot.v4.program.v2`: the pre-compile weld program, and what it refuses.

## What this document is

One weld program for both authoring flows. Offline Programming's classic flow
and this module's rails flow used to meet only at `robot.v4.program.v1` rows,
which are already post-planning: one row per waypoint, TCP poses baked, or
every solved joint knot. program.v2 sits one level up. It is an ordered list
of NODES -- move, weld, approach, transit, retract, home, dwell, io -- one
spreadsheet row per node. Taught moves hold a destination and its captured
observation; sampled trajectories and solved paths remain plan artifacts. A
planner lowers it. The schema is `robot/v4/contracts/programs/robot.v4.program.v2.schema.json`
and D23 in `docs/DECISIONS.md` is the reasoning.

## The one rule about motion

Weld blocks retain explicit connecting moves:

    approach -> weld -> (transit -> weld)* -> retract

Taught moves may precede, follow or separate these blocks, or stand alone.
An optional Home remains at the start or end.

The planner never inserts a freespace move. A gap the producer did not state
is a validation error, not a hint -- the document says every move the arm
makes, and the planner decides only HOW. Event nodes (`io`, `dwell`) sit on
the boundaries of that line and are transparent to the check: they execute
with the arm at rest, between the motion nodes around them.

## What this module is, and is not

The semantic half of the contract: the checks JSON Schema cannot express
(the alternation, the deny list, curve invariants), as sentences an operator
can act on. The structural half is the schema itself, closed everywhere, and
`tests/shared/test_program_v2.py` runs both and proves they agree. This
module deliberately does not import `jsonschema`: `weldplan` is stdlib-only
so that a planner can open a program without this module's environment, and
the Go and TypeScript validators beside it have no schema library at all, so
everything here is also checked by hand there.

## The deny list

`motion_server_submission.go` refuses a program carrying `planned_path`; this
does the same, recursively, for every key that would smuggle a solution in.
Two lists, because the seam vocabulary legitimately uses `knots` (a
`piecewise_linear` angle profile) and the classic geometry legitimately uses
`points` with `rpy_rad`: those are banned only under freespace nodes and
`defaults`, where they can only ever be a path.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

SCHEMA = "robot.v4.program.v2"

MOTION_OPS = ("weld", "approach", "transit", "retract", "home", "move")
FREESPACE_OPS = ("approach", "transit", "retract", "home")
EVENT_OPS = ("io", "dwell")

# Anywhere in the document. Each is the name of a solved thing in one of the
# repo's other contracts: program.v1 rows, the accepted plan, the dense
# trajectory, the motion-plan result.
FORBIDDEN_KEYS = frozenset(
    {
        "joints",
        "joint_positions_rad",
        "target_joints_rad",
        "q_rad",
        "qd_rad_s",
        "target_tcp_m",
        "tcp_pose_m",
        "trajectory",
        "planned_path",
        "accepted_plan",
        "preloaded_plan",
        "waypoints",
        "rows",
    }
)

# Under freespace nodes and `defaults` only. Legitimate seam or classic
# vocabulary elsewhere; under a freespace node they can only be a path.
FREESPACE_FORBIDDEN_KEYS = frozenset(
    {
        "knots",
        "knots_u",
        "times_s",
        "control_points_xyz_m",
        "segments",
        "samples",
        "points",
        "poses",
        "rpy_rad",
        "path",
    }
)

POLICY_KEYS = ("clearance_min_mm", "speed_scale", "torch_policy", "positioner_policy")


class ProgramV2Error(ValueError):
    """A program that cannot be accepted, with every reason."""


def problems(doc: dict[str, Any]) -> list[str]:
    """What is wrong with a program.v2 document, as sentences.

    Collected rather than raised on the first, so a document with three
    things wrong takes one round trip to fix. Structural shape (types,
    required keys, closed objects) is the schema's job and is not repeated
    here; what is here is everything the schema cannot say.
    """
    out: list[str] = []
    if doc.get("schema") != SCHEMA:
        out.append(f"schema must be {SCHEMA!r}, got {doc.get('schema')!r}")

    nodes = doc.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        out.append("nodes is empty, so there is nothing to plan")
        nodes = []

    # The robot the program is for, and that the placement names the same
    # one: the offsets are a record in that description's work frame
    # (docs/PLANNING-CONTEXT.md). The identity's shape is the schema's to
    # check; that the two halves agree is not.
    robot = doc.get("robot")
    placement = doc.get("placement")
    if (isinstance(robot, dict) and isinstance(placement, dict)
            and robot.get("model_id") != placement.get("cell_id")):
        out.append(
            f"placement.cell_id names {placement.get('cell_id')!r} and the program is for "
            f"{robot.get('model_id')!r}; the offsets are a record in one description's work frame")

    # The joint index a seam's `weld_joint_ref` resolves against. Duplicated
    # ids are reported here rather than silently shadowed, because the loser
    # of a shadow is whichever one the writer meant half the time.
    joints: dict[str, dict[str, Any]] = {}
    for entry in (doc.get("workpiece") or {}).get("weld_joints") or []:
        if not isinstance(entry, dict):
            continue
        jid = entry.get("id")
        if not isinstance(jid, str) or not jid:
            continue
        if jid in joints:
            out.append(f"workpiece.weld_joints has two joints with id {jid!r}")
            continue
        joints[jid] = entry

    owns = doc.get("planner_owns")
    if not isinstance(owns, list) or not owns:
        out.append("planner_owns is empty; the absences are decisions and must be stated")

    created = doc.get("created_utc")
    if isinstance(created, str) and _rfc3339(created) is None:
        out.append(f"created_utc {created!r} is not an RFC 3339 timestamp")

    out.extend(_finite_problems(doc, "$"))
    out.extend(_forbidden_problems(doc, "$", scoped=False))
    defaults = doc.get("defaults")
    if isinstance(defaults, dict):
        out.extend(_forbidden_problems(defaults, "$.defaults", scoped=True))

    from .taught_weld import problems as taught_weld_problems
    out.extend(taught_weld_problems(nodes))

    seen: set[str] = set()
    motion: list[tuple[int, str, str]] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            out.append(f"nodes[{index}] is not an object")
            continue
        nid = node.get("id", f"nodes[{index}]")
        op = node.get("op")
        if nid in seen:
            out.append(f"node id {nid!r} is used twice; one row, one id")
        seen.add(str(nid))
        if op in MOTION_OPS:
            motion.append((index, str(nid), str(op)))
        elif op not in EVENT_OPS:
            out.append(f"node {nid!r} has unknown op {op!r}")
            continue
        if op == "io":
            action = node.get("action", "set")
            if action not in ("set", "pulse", "wait", "monitor", "end_monitor"):
                out.append(f"node {nid!r} has unknown I/O action")
            for key in ("duration_s", "timeout_s", "stable_s"):
                if key in node and (type(node[key]) not in (int, float) or not math.isfinite(node[key]) or not (0 if key=="stable_s" else .001) <= node[key] <= 3600):
                    out.append(f"node {nid!r} {key} is outside its allowed range")
            if action=="pulse" and (node.get("state") is not True or not node.get("duration_s")):
                out.append(f"node {nid!r} pulse requires ON and duration")
            timeout, stable = node.get("timeout_s"), node.get("stable_s", 0)
            if action=="wait" and (type(timeout) not in (int, float) or type(stable) not in (int, float) or not timeout or stable>timeout):
                out.append(f"node {nid!r} wait requires a timeout at least as long as stability")
        if op == "move":
            out.extend(_move_problems(node, str(nid)))
        elif op == "weld":
            out.extend(_weld_problems(node, str(nid), joints, (doc.get("workpiece") or {}).get("frame_id")))
        elif op in FREESPACE_OPS:
            out.extend(_forbidden_problems(node, f"node {nid!r}", scoped=True))
            overrides = node.get("overrides")
            if isinstance(overrides, dict):
                for key in overrides:
                    if key not in POLICY_KEYS:
                        out.append(
                            f"node {nid!r} overrides {key!r}, which is not a defaults key"
                        )
        elif op == "dwell" and (
            not _is_number(node.get("duration_s")) or node["duration_s"] < 0
        ):
            out.append(f"dwell {nid!r} needs a non-negative duration_s")
        if op != "weld" and "weld_parameters" in node:
            out.append(f"node {nid!r} carries weld_parameters but is not a weld")
        if op not in FREESPACE_OPS and (
            "overrides" in node or "depart_standoff" in node or "arrive_standoff" in node
        ):
            out.append(f"node {nid!r} carries freespace policy but is not a freespace node")
        if op in ("approach", "home") and "depart_standoff" in node:
            out.append(f"{op} {nid!r} carries depart_standoff, but nothing is welded before it")
        if op in ("retract", "home") and "arrive_standoff" in node:
            out.append(f"{op} {nid!r} carries arrive_standoff, but nothing is welded after it")

    loop_returns = frozenset(
        str(node.get("id")) for node in nodes
        if isinstance(node, dict) and node.get("op") == "home"
        and isinstance(node.get("extensions"), dict) and node["extensions"].get("rosie.loop_return") is True
    )
    out.extend(_alternation_problems(motion, loop_returns))
    return out


def _move_problems(node: dict[str, Any], nid: str) -> list[str]:
    out = []
    for key in ("target", "via", "capture.pose"):
        pose = (node.get("capture") or {}).get("pose") if key == "capture.pose" else node.get(key)
        if key == "via" and pose is None:
            continue
        if not isinstance(pose, dict):
            out.append(f"move {nid!r} needs a {key} pose")
            continue
        names, values = pose.get("joint_names", []), pose.get("joint_values_rad", [])
        if not names or len(names) != len(values) or len(set(names)) != len(names):
            out.append(f"move {nid!r} {key} needs unique joint names and matching values")
    capture = node.get("capture") or {}
    if capture.get("source") == "machine" and not capture.get("cell_id"):
        out.append(f"move {nid!r} machine capture needs a cell_id")
    return out


def _weld_problems(node: dict[str, Any], nid: str,
                   joints: dict[str, dict[str, Any]],
                   workpiece_frame: Any = None) -> list[str]:
    out: list[str] = []
    params = node.get("weld_parameters")
    if params is None:
        out.append(f"weld {nid!r} has no weld_parameters; the reserved block must be present")
    elif params != {}:
        out.append(
            f"weld {nid!r} fills weld_parameters ({', '.join(sorted(params))}); "
            "the block is reserved and must be empty in program.v2"
        )
    geometry = node.get("geometry")
    if not isinstance(geometry, dict):
        out.append(f"weld {nid!r} has no geometry")
        return out
    kind = geometry.get("kind")
    if kind == "seam":
        out.extend(_seam_problems(geometry, nid, joints))
    elif kind == "posed_polyline":
        points = geometry.get("points")
        if not isinstance(points, list) or len(points) < 2:
            out.append(f"weld {nid!r} posed_polyline needs at least two points")
        else:
            for i, point in enumerate(points):
                for key in ("xyz_m", "rpy_rad"):
                    value = point.get(key) if isinstance(point, dict) else None
                    if not (isinstance(value, list) and len(value) == 3):
                        out.append(f"weld {nid!r} point {i} {key} must be 3 numbers")
    elif kind == "manual_pose_wp":
        out.extend(_manual_pose_problems(geometry, nid, workpiece_frame))
    else:
        out.append(
            f"weld {nid!r} geometry kind {kind!r} is not seam, posed_polyline or manual_pose_wp"
        )
    speed = node.get("travel_speed_mm_s")
    if isinstance(speed, dict):
        out.extend(_scalar_problems(speed, f"weld {nid!r} travel_speed_mm_s"))
    # Never defaulted here. An absent standoff is not zero -- zero puts the tip
    # in the part -- and the planner refuses a seam that authors none, so the
    # document has to carry it.
    standoff = node.get("standoff_mm")
    if not isinstance(standoff, dict):
        out.append(f"weld {nid!r} standoff_mm is required")
    else:
        out.extend(_scalar_problems(standoff, f"weld {nid!r} standoff_mm"))
        out.extend(_positive_scalar_problems(standoff, f"weld {nid!r} standoff_mm"))
    return out


# The shortest chord two consecutive manual_pose_wp points may have: a
# micrometre. Below it the chord has no direction to be continuous with and the
# arc-length parameter collapses (the schema's rule).
MANUAL_POSE_MIN_CHORD_M = 1e-6


def _manual_pose_problems(geometry: dict[str, Any], nid: str,
                          workpiece_frame: Any) -> list[str]:
    out: list[str] = []
    # The workpiece frame by definition: a hand-placed pose rides with the part.
    frame = geometry.get("frame_id")
    if not isinstance(frame, str) or not frame or frame != workpiece_frame:
        out.append(
            f"weld {nid!r} manual_pose_wp frame_id {frame!r} is not the workpiece "
            f"frame {workpiece_frame!r}"
        )
    shape = geometry.get("shape", "spline")
    if shape not in ("spline", "arc"):
        out.append(f"weld {nid!r} manual_pose_wp shape must be spline or arc")
    points = geometry.get("points")
    if not isinstance(points, list) or len(points) < 2:
        out.append(f"weld {nid!r} manual_pose_wp needs at least two points")
        return out
    if shape == "arc" and len(points) != 3:
        out.append(f"weld {nid!r} manual_pose_wp arc needs exactly three points")
        return out
    previous: list[float] | None = None
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            out.append(f"weld {nid!r} point {i} must be an object")
            previous = None
            continue
        for key in ("xyz_m", "rpy_rad"):
            value = point.get(key)
            if not (isinstance(value, list) and len(value) == 3):
                out.append(f"weld {nid!r} point {i} {key} must be 3 numbers")
        if point.get("continuity") not in ("C0", "C1", "C2"):
            out.append(f"weld {nid!r} point {i} continuity must be C0, C1 or C2")
        xyz = point.get("xyz_m")
        if not (isinstance(xyz, list) and len(xyz) == 3
                and all(isinstance(v, (int, float)) for v in xyz)):
            previous = None
            continue
        if previous is not None and math.dist(xyz, previous) <= MANUAL_POSE_MIN_CHORD_M:
            out.append(
                f"weld {nid!r} points {i - 1} and {i} coincide; "
                "consecutive points must be distinct"
            )
        previous = [float(v) for v in xyz]
    if shape == "arc":
        try:
            a, b, c = ([float(v) for v in point["xyz_m"]] for point in points)
        except (KeyError, TypeError, ValueError):
            return out
        ab = [b[i] - a[i] for i in range(3)]
        ac = [c[i] - a[i] for i in range(3)]
        normal = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
                  ab[0] * ac[1] - ab[1] * ac[0])
        chord = math.hypot(*ac)
        if chord <= MANUAL_POSE_MIN_CHORD_M or math.hypot(*normal) / chord <= MANUAL_POSE_MIN_CHORD_M:
            out.append(
                f"weld {nid!r} manual_pose_wp arc points lie on a line; "
                "no circle passes through them in order"
            )
    return out


def _seam_problems(geometry: dict[str, Any], nid: str,
                   joints: dict[str, dict[str, Any]]) -> list[str]:
    out: list[str] = []
    curve = geometry.get("curve")
    closed = False
    # A named joint has to EXIST, curve or no curve. The reference resolves
    # inside this document because `workpiece.weld_joints[]` is here (D24);
    # before that it pointed at a list program.v2 did not carry, so a document
    # could name a joint that existed nowhere and nothing would say so.
    ref = geometry.get("weld_joint_ref")
    named = isinstance(ref, str) and bool(ref)
    if named and ref not in joints:
        out.append(
            f"weld {nid!r} names weld_joint_ref {ref!r}, which is not in "
            "workpiece.weld_joints"
        )
    if not isinstance(curve, dict) or "segments" not in curve:
        # A seam may take its line from its joint instead of carrying one.
        # That is what `weld_joint_ref` is for, and it is what the planner
        # does: `Seam.__init__` reads `seam["curve"] or joint["weld_line"]`
        # and refuses when neither is there.
        if not named:
            out.append(
                f"weld {nid!r} seam geometry has no inline curve and no "
                "weld_joint_ref to take one from"
            )
        elif ref in joints and not isinstance(joints[ref].get("weld_line"), dict):
            out.append(
                f"weld {nid!r} has no curve and joint {ref!r} has no weld_line, "
                "so the seam has no line at all"
            )
    else:
        closed = bool(curve.get("closed", False))
        out.extend(_curve_problems(curve, f"weld {nid!r} curve"))
    span = geometry.get("span")
    if isinstance(span, dict):
        start = span.get("start_s", 0.0)
        end = span.get("end_s", 1.0)
        if _is_number(start) and _is_number(end) and start > end and not closed:
            out.append(
                f"weld {nid!r} span start_s {start} > end_s {end} wraps through s=0, "
                "which is only valid on a closed curve"
            )
    out.extend(_search_provenance_problems(geometry.get("search_provenance"), nid))

    orientation = geometry.get("orientation")
    if isinstance(orientation, dict):
        for axis in ("work_angle_deg", "travel_angle_deg"):
            profile = orientation.get(axis)
            if isinstance(profile, dict):
                out.extend(_scalar_problems(profile, f"weld {nid!r} {axis}"))
        limits = orientation.get("limits")
        if isinstance(limits, dict):
            for axis, band in limits.items():
                if not isinstance(band, dict):
                    continue
                lo, hi = band.get("min"), band.get("max")
                if (
                    isinstance(lo, dict)
                    and isinstance(hi, dict)
                    and lo.get("kind") == "constant"
                    and hi.get("kind") == "constant"
                    and _is_number(lo.get("value"))
                    and _is_number(hi.get("value"))
                    and lo["value"] > hi["value"]
                ):
                    out.append(f"weld {nid!r} {axis} band has min above max")
    return out


def _search_provenance_problems(block: Any, nid: str) -> list[str]:
    """A search that could not have moved what it says it was free to move.

    The window is the authored profile plus or minus the deviation, and the
    bin is how finely that window was walked. A bin wider than the window
    leaves one value in it, so the search chose the profile it was handed and
    the deviation described a freedom nothing exercised. Schema cannot say
    this: each number is fine on its own and only the pair is wrong.
    """
    if block is None:
        return []
    if not isinstance(block, dict):
        return [f"weld {nid!r} search_provenance must be an object"]
    out: list[str] = []
    for axis in ("work", "travel"):
        deviation = block.get(f"{axis}_deviation_deg")
        binned = block.get(f"{axis}_bin_deg")
        if not _is_number(deviation) or not _is_number(binned):
            continue
        if deviation > 0 and binned > 2 * deviation:
            out.append(
                f"weld {nid!r} searched {axis} over +/-{deviation} degrees in "
                f"{binned}-degree bins, which is one bin: the deviation names a "
                "freedom the bin spacing did not allow"
            )
    return out


def _curve_problems(curve: dict[str, Any], where: str) -> list[str]:
    """The invariants `tests/programmer/test_weld_program_schema.py` asserts
    by hand on the weld program: a clamped B-spline's knot vector has exactly
    n + p + 1 entries, is non-decreasing, and is clamped at both ends."""
    out: list[str] = []
    segments = curve.get("segments")
    if not isinstance(segments, list) or not segments:
        out.append(f"{where} has no segments")
        return out
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            out.append(f"{where} segment {i} is not an object")
            continue
        poles = seg.get("control_points_xyz_m")
        degree = seg.get("degree")
        knots = seg.get("knots_u")
        if not (isinstance(poles, list) and isinstance(knots, list) and isinstance(degree, int)):
            out.append(f"{where} segment {i} needs control_points_xyz_m, degree and knots_u")
            continue
        n, p = len(poles), degree
        if len(knots) != n + p + 1:
            out.append(
                f"{where} segment {i} knots_u has {len(knots)} entries, "
                f"needs {n} + {p} + 1 = {n + p + 1}"
            )
            continue
        if any(b < a for a, b in itertools.pairwise(knots)):
            out.append(f"{where} segment {i} knots_u is not non-decreasing")
        if knots[: p + 1] != [knots[0]] * (p + 1) or knots[-(p + 1) :] != [knots[-1]] * (p + 1):
            out.append(f"{where} segment {i} knots_u is not clamped")
        weights = seg.get("weights")
        if weights is not None and len(weights) != n:
            out.append(f"{where} segment {i} has {len(weights)} weights for {n} poles")
    return out


def _positive_scalar_problems(fn: dict[str, Any], where: str) -> list[str]:
    """Every value a standoff profile can take has to be positive: the number is a distance from the work, and zero or less is the tip inside the part. Checked here rather than left to the planner, because a document that says it is welding at 0 mm is wrong before anything is planned."""
    kind = fn.get("kind")
    if kind == "constant":
        values = [fn.get("value")]
    elif kind == "piecewise_linear":
        values = [k[1] for k in fn.get("knots") or [] if isinstance(k, list) and len(k) == 2]
    elif kind == "samples":
        values = list(fn.get("values") or [])
    elif kind == "bspline":
        values = list(fn.get("control_values") or [])
    else:
        return []
    for value in values:
        if not isinstance(value, int | float) or isinstance(value, bool) or value <= 0:
            return [f"{where} must be positive everywhere, got {value!r}"]
    return []


def _scalar_problems(fn: dict[str, Any], where: str) -> list[str]:
    out: list[str] = []
    kind = fn.get("kind")
    if kind == "piecewise_linear":
        knots = fn.get("knots")
        if isinstance(knots, list) and len(knots) >= 2:
            s = [k[0] for k in knots if isinstance(k, list) and len(k) == 2]
            if len(s) != len(knots) or s[0] != 0.0 or s[-1] != 1.0:
                out.append(f"{where} piecewise_linear knots must run from s=0 to s=1")
            elif any(b <= a for a, b in itertools.pairwise(s)):
                out.append(f"{where} piecewise_linear knots must have strictly increasing s")
    elif kind == "samples":
        s, values = fn.get("s"), fn.get("values")
        if isinstance(s, list) and isinstance(values, list) and len(s) != len(values):
            out.append(f"{where} samples has {len(s)} s values and {len(values)} values")
    elif kind == "bspline":
        knots, values, degree = fn.get("knots_u"), fn.get("control_values"), fn.get("degree")
        if (
            isinstance(knots, list)
            and isinstance(values, list)
            and isinstance(degree, int)
            and len(knots) != len(values) + degree + 1
        ):
            out.append(f"{where} bspline knot count does not match control_values + degree + 1")
    return out


def _alternation_problems(motion: list[tuple[int, str, str]], loop_returns: frozenset[str] = frozenset()) -> list[str]:
    """The single line. `home` at either end, then approach / weld / transit
    / weld ... / retract, strictly alternating weld and freespace."""
    out: list[str] = []
    if not motion:
        out.append("the program has no motion nodes")
        return out
    ops = [op for _, _, op in motion]
    # A loop-return Home (OLP loop mode) ends the program after its own last
    # node, a Home included: the other nodes are judged as if it were absent.
    end = len(motion)
    while end > 1 and motion[end - 1][2] == "home" and motion[end - 1][1] in loop_returns:
        end -= 1
    for k, (index, nid, op) in enumerate(motion):
        prev_op = ops[k - 1] if k > 0 else None
        next_op = ops[k + 1] if k + 1 < end else None
        if k >= end:
            continue
        if op == "home":
            if k not in (0, end - 1):
                out.append(
                    f"home {nid!r} sits in the middle of the program; home may only start or end it"
                )
        elif op == "weld":
            if prev_op not in ("approach", "transit"):
                out.append(
                    f"weld {nid!r} is not preceded by an approach or transit"
                    + (" (an approach is missing)" if prev_op in (None, "home") else "")
                )
            if next_op not in ("retract", "transit"):
                out.append(
                    f"weld {nid!r} is not followed by a retract or transit"
                    + (" (a transit is missing before the next weld)" if next_op == "weld" else "")
                )
        elif op == "approach":
            if next_op != "weld":
                out.append(f"approach {nid!r} is not followed by a weld")
            if prev_op not in (None, "home", "move", "retract"):
                out.append(
                    f"approach {nid!r} follows {prev_op!r}; only a home or nothing may precede it"
                )
        elif op == "retract":
            if prev_op != "weld":
                out.append(f"retract {nid!r} is not preceded by a weld")
            if next_op not in (None, "home", "move", "approach"):
                out.append(
                    f"retract {nid!r} is followed by {next_op!r}; only a home or nothing may follow it"
                )
        elif op == "transit" and (prev_op != "weld" or next_op != "weld"):
            out.append(f"transit {nid!r} must sit between two welds")
    return out


def _forbidden_problems(value: Any, where: str, *, scoped: bool) -> list[str]:
    """Recursive, exact key match. A substring match would trip on
    `weld_joint_ref`; a top-level-only match would miss a path one object
    down, which is where a smuggled solution would sit."""
    out: list[str] = []
    banned = FORBIDDEN_KEYS | (FREESPACE_FORBIDDEN_KEYS if scoped else frozenset())
    if isinstance(value, dict):
        for key, child in value.items():
            if key in banned:
                out.append(
                    f"{where} carries {key!r}, which is a solved quantity; "
                    "program.v2 states intent only"
                )
            else:
                out.extend(_forbidden_problems(child, f"{where}.{key}", scoped=scoped))
    elif isinstance(value, list):
        for i, child in enumerate(value):
            out.extend(_forbidden_problems(child, f"{where}[{i}]", scoped=scoped))
    return out


def _finite_problems(value: Any, where: str) -> list[str]:
    """Python's json accepts NaN and Infinity; Go's and Node's do not. The
    three validators must agree, so the odd one out checks by hand."""
    out: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            out.extend(_finite_problems(child, f"{where}.{key}"))
    elif isinstance(value, list):
        for i, child in enumerate(value):
            out.extend(_finite_problems(child, f"{where}[{i}]"))
    elif isinstance(value, float) and not math.isfinite(value):
        out.append(f"{where} is {value}, which is not a finite number")
    return out


def _is_number(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _rfc3339(text: str) -> datetime | None:
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None


def load(path: Path) -> dict[str, Any]:
    """Read a document and refuse it with every problem, or return it."""
    doc = json.loads(path.read_text(encoding="utf-8"))
    found = problems(doc)
    if found:
        raise ProgramV2Error("; ".join(found))
    return doc


def _minimal() -> dict[str, Any]:
    point = {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]}

    def node(nid: str, op: str, **own: Any) -> dict[str, Any]:
        return {
            "id": nid,
            "op": op,
            "enabled": True,
            "label": nid,
            "comment": "",
            "row_hash": "sha256:" + "0" * 64,
            **own,
        }

    weld = {
        "geometry": {
            "kind": "posed_polyline",
            "frame_id": "workpiece",
            "points": [point, {**point, "xyz_m": [0.1, 0.0, 0.0]}],
        },
        "travel_speed_mm_s": {"kind": "constant", "value": 8.0},
        "standoff_mm": {"kind": "constant", "value": 17.0},
        "weave": {"shape": "none"},
        "weld_parameters": {},
    }
    return {
        "schema": SCHEMA,
        "program_id": "self_test",
        "program_name": "self test",
        "program_revision": 1,
        "base_program_revision": 0,
        "program_state": "saved_unplanned",
        "execution_mode": "simulation",
        "created_utc": "2026-09-03T00:00:00Z",
        "units": {
            "length": "m",
            "angle": "rad",
            "time": "s",
            "joint_order": ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"],
        },
        "producer": {
            "component": "weld-planner-v1",
            "source_revision": "self-test",
            "inputs": [{"kind": "weld_program", "ref": "p", "sha256": "0" * 64}],
        },
        "workpiece": {"frame_id": "workpiece"},
        "placement": {
            "cell_id": "c",
            "anchor_xyz_m": [0.0, 0.0, 0.0],
            "offset_xyz_m": [0.0, 0.0, 0.0],
            "rpy_rad": [0.0, 0.0, 0.0],
        },
        "robot": {"model_id": "c", "robot_description_sha256": "sha256:" + "0" * 64},
        "defaults": {
            "clearance_min_mm": 20.0,
            "speed_scale": 1.0,
            "torch_policy": "free",
            "positioner_policy": "free",
        },
        "nodes": [
            node("home_0", "home"),
            node("approach_1", "approach"),
            node("io_1", "io", channel="revpi:3", state=True),
            node("weld_1", "weld", **weld),
            node("transit_1", "transit"),
            node("weld_2", "weld", **weld),
            node("dwell_2", "dwell", duration_s=0.5),
            node("retract_2", "retract"),
            node("home_3", "home"),
        ],
        "planner_owns": ["everything a plan is"],
    }


def self_test() -> int:
    import copy

    doc = _minimal()
    assert problems(doc) == [], problems(doc)

    def expect(mutate, phrase: str) -> None:
        bad = copy.deepcopy(doc)
        mutate(bad)
        found = " ".join(problems(bad))
        assert phrase in found, f"expected {phrase!r} in: {found!r}"

    # two welds with nothing between them: the planner does not fill gaps
    expect(lambda d: d["nodes"].pop(4), "transit is missing")
    # a weld first: no approach
    expect(lambda d: d["nodes"].__delitem__(slice(0, 2)), "approach is missing")
    # home in the middle
    expect(lambda d: d["nodes"].insert(4, {**d["nodes"][0], "id": "home_mid"}), "middle")
    # a solved path on a transit
    expect(lambda d: d["nodes"][4].__setitem__("trajectory", []), "'trajectory'")
    # classic points under a freespace node
    expect(
        lambda d: d["nodes"][4].__setitem__("points", [{"xyz_m": [0, 0, 0]}]),
        "'points'",
    )
    # dense joints on a weld
    expect(lambda d: d["nodes"][3].__setitem__("q_rad", [[0] * 9]), "'q_rad'")
    # weld parameters filled
    expect(
        lambda d: d["nodes"][3]["weld_parameters"].__setitem__("voltage_v", 24.0),
        "reserved",
    )
    # an override that is not a defaults key
    expect(lambda d: d["nodes"][4].__setitem__("overrides", {"max_qd": 1}), "not a defaults key")
    # a standoff on the wrong side of the node it belongs to
    lift = {"direction": "workpiece_z", "distance_mm": 10.0}
    expect(lambda d: d["nodes"][1].__setitem__("depart_standoff", lift), "nothing is welded before")
    expect(lambda d: d["nodes"][7].__setitem__("arrive_standoff", lift), "nothing is welded after")
    ok_lift = copy.deepcopy(doc)
    ok_lift["nodes"][4]["depart_standoff"] = lift
    ok_lift["nodes"][4]["arrive_standoff"] = lift
    assert problems(ok_lift) == [], problems(ok_lift)
    # The standoff. Absent is not zero -- zero is the tip inside the part --
    # so both are refused here, at authoring, rather than arriving later as a
    # refused seam inside a 200 from the motion server.
    expect(lambda d: d["nodes"][3].pop("standoff_mm"), "standoff_mm is required")
    expect(
        lambda d: d["nodes"][3].__setitem__("standoff_mm", {"kind": "constant", "value": 0.0}),
        "positive everywhere",
    )
    expect(
        lambda d: d["nodes"][3].__setitem__(
            "standoff_mm", {"kind": "piecewise_linear", "knots": [[0.0, 12.0], [1.0, 0.0]]},
        ),
        "positive everywhere",
    )
    ok_standoff = copy.deepcopy(doc)
    ok_standoff["nodes"][3]["standoff_mm"] = {
        "kind": "piecewise_linear",
        "knots": [[0.0, 12.0], [1.0, 9.0]],
    }
    assert problems(ok_standoff) == [], problems(ok_standoff)
    # NaN, which json.loads would let through
    expect(lambda d: d["defaults"].__setitem__("clearance_min_mm", float("nan")), "finite")
    # a piecewise_linear profile on a weld is legitimate seam vocabulary
    ok = copy.deepcopy(doc)
    ok["nodes"][3]["travel_speed_mm_s"] = {
        "kind": "piecewise_linear",
        "knots": [[0.0, 8.0], [1.0, 6.0]],
    }
    assert problems(ok) == [], problems(ok)
    # but one that runs backwards is not
    expect(
        lambda d: d["nodes"][3].__setitem__(
            "travel_speed_mm_s",
            {"kind": "piecewise_linear", "knots": [[0.0, 8.0], [0.5, 6.0], [0.4, 6.0], [1.0, 6.0]]},
        ),
        "strictly increasing",
    )
    # a seam whose knot vector is one short
    seam = {
        "kind": "seam",
        "id": "seam_0001",
        "curve": {
            "segments": [
                {"degree": 1, "control_points_xyz_m": [[0, 0, 0], [1, 0, 0]], "knots_u": [0, 0, 1]}
            ]
        },
        "reference": {"mode": "fixed_vector", "fixed_direction_xyz": [0, 0, 1]},
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 0.0},
        },
    }
    expect(lambda d: d["nodes"][3].__setitem__("geometry", seam), "knots_u has 3 entries")
    # a wrapping span on an open curve
    seam_ok = copy.deepcopy(seam)
    seam_ok["curve"]["segments"][0]["knots_u"] = [0, 0, 1, 1]
    seam_ok["span"] = {"start_s": 0.8, "end_s": 0.2}
    expect(lambda d: d["nodes"][3].__setitem__("geometry", seam_ok), "only valid on a closed curve")

    print("program.v2 self-test ok: the line must alternate, gaps are refused, solutions are refused")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("path", nargs="?", default="", help="a program.v2 JSON document to check")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.path:
        raise ProgramV2Error("a path or --self-test")
    doc = json.loads(Path(ns.path).read_text(encoding="utf-8"))
    found = problems(doc)
    for line in found:
        print(line)
    if not found:
        ops = [n.get("op") for n in doc.get("nodes", [])]
        print(f"ok: {doc.get('program_id')} with {len(ops)} nodes, {ops.count('weld')} welds")
    return 1 if found else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"program_v2: {exc}", file=sys.stderr)
        raise SystemExit(1)
