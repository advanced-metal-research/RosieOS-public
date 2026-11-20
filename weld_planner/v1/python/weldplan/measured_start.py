"""Immutable native observation used as the first configuration, never reset.

This validates a captured request, not live readiness: the OLP owns capture
and must compare the same native identity and current joints again at Play.

Every motion node plans from it (2026-10-11, owner: "this is a general
purpose robot, we must be able to program it"): taught moves, Home, the weld
block's approach/transit/retract and the welds between them. The program is
flown in authored order from this pose; the welds keep their own verified
trajectories and only the free-space legs connect from it. A Home first is a
collision-checked transit to the reset pose, not an assumption that the arm
is already there. An `io` node is scheduled against the cell's outputs
(`io_schedule.py`) or refused by name; an output is never silently omitted.
"""
import math

SCHEMA = "rosie.measured-planning-start.v1"

MOTION_OPS = ("move", "weld", "approach", "transit", "retract", "home")


def validate(start, cell, program, io_outputs=None):
    if start is None:
        return None
    if not isinstance(start, dict) or start.get("schema") != SCHEMA:
        raise ValueError("invalid measured planning start schema")
    names, values = start.get("joint_names"), start.get("joint_values_rad")
    if not isinstance(names, list) or not all(isinstance(n, str) and n for n in names) or names != cell.get("axes") or len(set(names)) != len(names):
        raise ValueError("measured start must name every cell joint in its declared order")
    if not isinstance(values, list) or len(values) != len(names) or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        raise ValueError("measured start requires finite joint positions")
    for key in ("model_id", "robot_description_sha256", "machine_planning_calibration_sha256"):
        if not (start.get("robot_cell") or {}).get(key) or start["robot_cell"][key] != (cell.get("robot_cell") or {}).get(key):
            raise ValueError("measured start robot/calibration does not match the planning cell")
    for key in ("coordinate_identity", "observation_identity", "observed_time_ns"):
        if not isinstance(start.get(key), str) or not start[key]:
            raise ValueError("measured start lacks native coordinate/observation identity")
    if not start["observed_time_ns"].isdigit() or int(start["observed_time_ns"]) <= 0:
        raise ValueError("measured start has no native observation time")
    nodes = [n for n in program.get("nodes", []) if n.get("enabled", True)]
    if not any(n.get("op") in MOTION_OPS for n in nodes):
        raise ValueError("measured start requires at least one motion node (waypoint, Home or weld)")
    refuse_io(nodes, io_outputs)
    return start


def in_program_order(program, start_state):
    """Whether the program is flown node by node in authored order.

    A taught waypoint fixes the order, and so does a measured start: the
    first leg leaves the robot's observed pose, so every later leg leaves
    wherever the previous node ended (a weld-only program included, which
    the model-reset planner may reorder and always starts at reset). The
    connector (`connecting_trajopt_main`), the result's `program_order` and
    the dense node mapping all follow this one predicate."""
    return start_state is not None or any(
        n.get("op") in ("move", "io") and n.get("enabled", True) for n in program.get("nodes", []))


def refuse_io(nodes, io_outputs=None):
    """An enabled `io` node that cannot be scheduled is refused by name, on
    every planning path (`weldplan.io_schedule.io_problems`): with no cell
    outputs in the request, with a channel the cell does not have, or a torch
    output the cell has not qualified. Before 2026-10-11 a weld-only program
    dropped its I/O rows without a word, because only the taught connector
    looked."""
    from .io_schedule import io_problems
    problems = io_problems(nodes, io_outputs)
    if problems:
        raise ValueError("; ".join(problems))


def _node_name(node):
    op, label = str(node.get("op")), str(node.get("label") or "").strip()
    return op.upper() if not label or label.lower() == op.lower() else f"{label} ({op})"
