#!/usr/bin/env python
"""Generic no-ROS Cartesian planning worker for `rosie tesseract v1`.

The worker intentionally knows nothing about Robot V7. Callers provide URDF/SRDF,
joint names, a start seed, target points, and optional tip offset. The result is
planner diagnostics; consumers decide how, or whether, to execute anything.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

import ur10_demo as demo

PLAN_SCHEMA = "tesseract-v1.cartesian-plan.v1"
RESULT_SCHEMA = "tesseract-v1.cartesian-plan-result.v1"


def require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value.strip()


def float_vector(value: Any, label: str, length: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} must contain exactly {length} numbers")
    try:
        return np.asarray([float(item) for item in value], dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} contains a non-numeric value") from exc


def string_list(value: Any, label: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{label} must be a non-empty string array")
    out = [require_string(item, label) for item in value]
    if len(set(out)) != len(out):
        raise ValueError(f"{label} contains duplicate names")
    return out


def load_plan(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    plan = require_mapping(payload, "plan")
    if plan.get("schema") != PLAN_SCHEMA:
        raise ValueError(f"unsupported plan schema {plan.get('schema')!r}")
    return plan


def target_points(plan: dict[str, Any]) -> list[dict[str, Any]]:
    """Parse targets into {label, xyz | local} entries.

    `pose.xyz_m` is a world-frame point. `pose.local_xyz_m` is expressed in
    the plan's `workpiece_frame.link` frame and resolved against the live
    Tesseract environment after the frame joints are baked into the start
    state.
    """
    raw_targets = plan.get("targets")
    if not isinstance(raw_targets, list) or len(raw_targets) < 2:
        raise ValueError("targets must contain at least two waypoints")
    points: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_targets):
        target = require_mapping(raw, f"targets[{index}]")
        pose = require_mapping(target.get("pose"), f"targets[{index}].pose")
        entry: dict[str, Any] = {"label": str(target.get("label") or f"target_{index}")}
        if pose.get("local_xyz_m") is not None:
            entry["local"] = float_vector(pose.get("local_xyz_m"), f"targets[{index}].pose.local_xyz_m", 3)
        else:
            entry["xyz"] = float_vector(pose.get("xyz_m"), f"targets[{index}].pose.xyz_m", 3)
        overrides = target.get("joint_values_rad")
        if overrides is not None:
            overrides = require_mapping(overrides, f"targets[{index}].joint_values_rad")
            entry["frame_joints"] = {str(name): float(value) for name, value in overrides.items()}
        points.append(entry)
    return points


def parse_workpiece_frame(plan: dict[str, Any], joint_names: list[str]) -> tuple[str, dict[str, float]]:
    """Return (frame link, frame joint values) from plan.workpiece_frame."""
    workpiece = plan.get("workpiece_frame")
    if workpiece is None:
        return "", {}
    workpiece = require_mapping(workpiece, "workpiece_frame")
    link = require_string(workpiece.get("link"), "workpiece_frame.link")
    raw_joints = workpiece.get("joint_values_rad") or {}
    raw_joints = require_mapping(raw_joints, "workpiece_frame.joint_values_rad")
    frame_joints: dict[str, float] = {}
    for joint_name, value in raw_joints.items():
        if joint_name not in joint_names:
            raise ValueError(f"workpiece_frame joint {joint_name!r} not in robot.joint_names")
        frame_joints[str(joint_name)] = float(value)
    return link, frame_joints


def workpiece_frame_transform(env: Any, joint_names: list[str], start: np.ndarray, link: str) -> np.ndarray:
    """4x4 world transform of the workpiece link at the start state."""
    env.setState(joint_names, start)
    transform = env.getLinkTransform(link)
    matrix = np.eye(4)
    matrix[:3, :3] = demo.transform_matrix(transform)[:3, :3]
    matrix[:3, 3] = demo.transform_translation(transform)
    return matrix


def active_tcp_xyz(env: Any, joint_names: list[str], q: np.ndarray, tip_link: str, tip_offset: np.ndarray) -> np.ndarray:
    env.setState(joint_names, q)
    transform = env.getLinkTransform(tip_link)
    matrix = demo.transform_matrix(transform)
    xyz = demo.transform_translation(transform)
    return xyz + matrix[:3, :3] @ tip_offset


def solve_tip_offset_ik(
    env: Any,
    joint_names: list[str],
    seed: np.ndarray,
    target_xyz: np.ndarray,
    tip_link: str,
    tip_offset: np.ndarray,
    tolerance_m: float,
) -> np.ndarray:
    q = np.asarray(seed, dtype=float).copy()
    limits = demo.limits_for_arm_joints(joint_names)
    eps = 1e-4
    damping = 0.055
    for _ in range(180):
        current = active_tcp_xyz(env, joint_names, q, tip_link, tip_offset)
        error = target_xyz - current
        if float(np.linalg.norm(error)) <= tolerance_m:
            break
        jac = np.zeros((3, len(joint_names)), dtype=float)
        for col in range(len(joint_names)):
            step = np.zeros(len(joint_names), dtype=float)
            step[col] = eps
            plus = active_tcp_xyz(env, joint_names, q + step, tip_link, tip_offset)
            minus = active_tcp_xyz(env, joint_names, q - step, tip_link, tip_offset)
            jac[:, col] = (plus - minus) / (2.0 * eps)
        lhs = jac @ jac.T + (damping * damping) * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, error)
        q += np.clip(dq, -0.14, 0.14)
        q = np.clip(q, limits[:, 0], limits[:, 1])
    return q


def tool_axis_world(env: Any, joint_names: list[str], q: np.ndarray, tip_link: str, tool_axis: np.ndarray) -> np.ndarray:
    env.setState(joint_names, q)
    matrix = demo.transform_matrix(env.getLinkTransform(tip_link))
    return matrix[:3, :3] @ tool_axis


def axis_alignment_error(axis: np.ndarray, world_axis: np.ndarray, weight: float) -> np.ndarray:
    """Axis-angle orientation error scaled by weight (meters per radian).

    Unlike the raw cross product this has no spurious zero at anti-parallel:
    there the magnitude is pi along an arbitrary perpendicular direction.
    """
    cross = np.cross(axis, world_axis)
    dot = float(np.clip(np.dot(axis, world_axis), -1.0, 1.0))
    angle = math.acos(dot)
    norm = float(np.linalg.norm(cross))
    if norm > 1e-9:
        direction = cross / norm
    elif dot < 0.0:
        reference = np.array([1.0, 0.0, 0.0]) if abs(float(axis[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
        direction = np.cross(axis, reference)
        direction = direction / float(np.linalg.norm(direction))
    else:
        return np.zeros(3)
    return weight * angle * direction


def solve_tip_offset_ik_axis(
    env: Any,
    joint_names: list[str],
    seed: np.ndarray,
    target_xyz: np.ndarray,
    tip_link: str,
    tip_offset: np.ndarray,
    tolerance_m: float,
    tool_axis: np.ndarray,
    world_axis: np.ndarray,
    weight: float,
    angle_tolerance_rad: float,
    posture_targets: np.ndarray | None = None,
    posture_weights: np.ndarray | None = None,
) -> np.ndarray:
    """Position IK plus a tool-axis alignment constraint (5 DOF: rotation
    about the tool axis stays free, suiting axially symmetric tools).

    Optional posture bias: weak extra error rows pull selected joints toward
    preferred values through the remaining redundancy, e.g. keeping J2 near
    vertical while the free tool-axis yaw absorbs the difference.
    """
    q = np.asarray(seed, dtype=float).copy()
    limits = demo.limits_for_arm_joints(joint_names)
    eps = 1e-4
    damping = 0.055
    use_posture = posture_targets is not None and posture_weights is not None

    def error_vector(qv: np.ndarray) -> np.ndarray:
        position = active_tcp_xyz(env, joint_names, qv, tip_link, tip_offset)
        axis = tool_axis_world(env, joint_names, qv, tip_link, tool_axis)
        parts = [target_xyz - position, axis_alignment_error(axis, world_axis, weight)]
        if use_posture:
            parts.append(posture_weights * (posture_targets - qv))
        return np.concatenate(parts)

    for _ in range(360):
        error = error_vector(q)
        axis = tool_axis_world(env, joint_names, q, tip_link, tool_axis)
        angle = math.acos(float(np.clip(np.dot(axis, world_axis), -1.0, 1.0)))
        if float(np.linalg.norm(error[:3])) <= tolerance_m and angle <= angle_tolerance_rad:
            break
        rows = len(error)
        jac = np.zeros((rows, len(joint_names)), dtype=float)
        for col in range(len(joint_names)):
            step = np.zeros(len(joint_names), dtype=float)
            step[col] = eps
            plus = error_vector(q + step)
            minus = error_vector(q - step)
            jac[:, col] = (plus - minus) / (2.0 * eps)
        # jac differentiates the error, so the Newton step is negated.
        lhs = jac @ jac.T + (damping * damping) * np.eye(rows)
        dq = -jac.T @ np.linalg.solve(lhs, error)
        q += np.clip(dq, -0.14, 0.14)
        q = np.clip(q, limits[:, 0], limits[:, 1])
    return q


def run_trajopt(mods: dict[str, object], env: Any, joint_names: list[str], waypoints: list[np.ndarray], working_frame: str, tip_link: str) -> list[dict[str, Any]]:
    program = build_joint_program(mods, waypoints, joint_names, working_frame, tip_link)
    request_cls = mods["PlannerRequest"]
    planner_cls = mods["TrajOptMotionPlanner"]

    request = request_cls()
    request.env = env
    request.instructions = program
    request.profiles = demo.make_profiles(mods)
    request.verbose = False

    planner = planner_cls(demo.PLANNER_NAME)
    if hasattr(planner, "checkRequest") and not planner.checkRequest(request):
        raise RuntimeError("Tesseract rejected the Cartesian plan request")
    response = planner.solve(request)
    if not demo.planner_response_successful(response):
        raise RuntimeError(f"TrajOpt failed: {demo.planner_response_message(response)}")
    return demo.extract_positions(mods, response.results)


def build_joint_program(mods: dict[str, object], waypoints: list[np.ndarray], joint_names: list[str], working_frame: str, tip_link: str):
    composite_instruction = mods["CompositeInstruction"]
    default_profile_key = mods["DEFAULT_PROFILE_KEY"]
    joint_waypoint = mods["JointWaypoint"]
    joint_waypoint_wrap = mods["JointWaypointPoly_wrap_JointWaypoint"]
    manipulator_info = mods["ManipulatorInfo"]
    move_instruction = mods["MoveInstruction"]
    move_instruction_wrap = mods["MoveInstructionPoly_wrap_MoveInstruction"]
    move_type = mods["MoveInstructionType_FREESPACE"]

    manip = manipulator_info()
    manip.manipulator = "manipulator"
    manip.working_frame = working_frame
    manip.tcp_frame = tip_link

    program = composite_instruction(default_profile_key)
    program.setManipulatorInfo(manip)
    for q in waypoints:
        waypoint = joint_waypoint(joint_names, q)
        instruction = move_instruction(joint_waypoint_wrap(waypoint), move_type, default_profile_key)
        program.appendMoveInstruction(move_instruction_wrap(instruction))
    return program


def enrich_rows(
    env: Any,
    rows: list[dict[str, Any]],
    joint_names: list[str],
    targets: list[np.ndarray],
    tip_link: str,
    tip_offset: np.ndarray,
    tcp_speed_m_s: float,
) -> tuple[float, float]:
    max_tracking_error = 0.0
    path_length = 0.0
    previous: np.ndarray | None = None
    elapsed = 0.0
    for index, row in enumerate(rows):
        target = targets[min(index, len(targets) - 1)]
        q = np.asarray(row["positions_rad"], dtype=float)
        tcp = active_tcp_xyz(env, joint_names, q, tip_link, tip_offset)
        if previous is not None:
            segment = float(np.linalg.norm(target - targets[min(index - 1, len(targets) - 1)]))
            path_length += segment
            elapsed += segment / max(0.001, tcp_speed_m_s)
        previous = target
        tracking_error = float(np.linalg.norm(tcp - target))
        max_tracking_error = max(max_tracking_error, tracking_error)
        row["index"] = index
        row["time_s"] = elapsed
        row["target_xyz_m"] = [float(x) for x in target]
        row["tip_link"] = tip_link
        row["tip_offset_xyz_m"] = [float(x) for x in tip_offset]
        row["active_tcp_xyz_m"] = [float(x) for x in tcp]
        row["tracking_error_m"] = tracking_error
        row["tcp_speed_m_s"] = float(tcp_speed_m_s)
    for row in rows:
        row["tool_path_length_m"] = path_length
    return max_tracking_error, path_length


def plan_cartesian(plan: dict[str, Any]) -> dict[str, Any]:
    robot = require_mapping(plan.get("robot"), "robot")
    urdf_path = Path(require_string(robot.get("urdf"), "robot.urdf"))
    srdf_path = Path(require_string(robot.get("srdf"), "robot.srdf"))
    joint_names = string_list(robot.get("joint_names"), "robot.joint_names")
    working_frame = str(robot.get("working_frame") or "base_link")
    base_link = str(robot.get("base_link") or working_frame)
    tip_link = str(robot.get("tip_link") or robot.get("tcp_frame") or "tool0")
    start = float_vector(plan.get("start_joint_rad"), "start_joint_rad", len(joint_names))
    target_entries = target_points(plan)
    frame_link, frame_joints = parse_workpiece_frame(plan, joint_names)
    if not frame_link and any("local" in entry for entry in target_entries):
        raise ValueError("targets use pose.local_xyz_m but the plan has no workpiece_frame")
    # The frame joints (external positioner axes) are baked into the start
    # state so IK seeding, TrajOpt, and collision checking all see the true
    # workpiece pose.
    for joint_name, value in frame_joints.items():
        start[joint_names.index(joint_name)] = value
    planner = require_mapping(plan.get("planner", {}), "planner")
    tolerance_m = float(planner.get("position_tolerance_m", 0.006))
    tcp_speed_m_s = float(plan.get("tcp_speed_m_s", 0.005))
    tip_offset = float_vector(robot.get("tip_offset_xyz_m", [0, 0, 0]), "robot.tip_offset_xyz_m", 3)

    axis_constraint = plan.get("tool_axis_constraint")
    tool_axis = np.zeros(3)
    world_axis = np.zeros(3)
    axis_weight = 0.5
    axis_tolerance_rad = math.radians(1.0)
    if axis_constraint is not None:
        axis_constraint = require_mapping(axis_constraint, "tool_axis_constraint")
        tool_axis = float_vector(axis_constraint.get("tool_axis_xyz"), "tool_axis_constraint.tool_axis_xyz", 3)
        tool_axis = tool_axis / float(np.linalg.norm(tool_axis))
        world_axis = float_vector(axis_constraint.get("world_axis_xyz"), "tool_axis_constraint.world_axis_xyz", 3)
        world_axis = world_axis / float(np.linalg.norm(world_axis))
        axis_weight = float(axis_constraint.get("weight", 0.5))
        axis_tolerance_rad = math.radians(float(axis_constraint.get("angle_tolerance_deg", 1.0)))

    posture_bias = plan.get("posture_bias")
    posture_targets = None
    posture_weights = None
    if posture_bias is not None:
        posture_bias = require_mapping(posture_bias, "posture_bias")
        biased = require_mapping(posture_bias.get("joints"), "posture_bias.joints")
        posture_targets = np.zeros(len(joint_names))
        posture_weights = np.zeros(len(joint_names))
        default_weight = float(posture_bias.get("weight", 0.08))
        for joint_name, preferred in biased.items():
            if joint_name not in joint_names:
                raise ValueError(f"posture_bias joint {joint_name!r} not in robot.joint_names")
            index = joint_names.index(joint_name)
            posture_targets[index] = float(preferred)
            posture_weights[index] = default_weight

    mods = demo.load_tesseract_modules()
    env = demo.make_environment(mods, start, joint_names, urdf_path, srdf_path)
    frame_matrix = None
    if frame_link:
        frame_matrix = workpiece_frame_transform(env, joint_names, start, frame_link)
    targets: list[np.ndarray] = []
    target_frame_joints: list[dict[str, float]] = []
    for index, entry in enumerate(target_entries):
        # Per-target external-axis overrides let one plan sweep a positioner
        # joint through the path (harmonious workpiece co-motion).
        merged = dict(frame_joints)
        for joint_name, value in (entry.get("frame_joints") or {}).items():
            if joint_name not in joint_names:
                raise ValueError(f"targets[{index}].joint_values_rad joint {joint_name!r} not in robot.joint_names")
            merged[joint_name] = float(value)
        target_frame_joints.append(merged)
        if "local" in entry:
            state = start.copy()
            for joint_name, value in merged.items():
                state[joint_names.index(joint_name)] = value
            matrix = workpiece_frame_transform(env, joint_names, state, frame_link)
            targets.append(matrix[:3, :3] @ entry["local"] + matrix[:3, 3])
        else:
            targets.append(entry["xyz"])
    q = start.copy()
    seeds: list[np.ndarray] = []
    for target, target_joints in zip(targets, target_frame_joints):
        for joint_name, value in target_joints.items():
            q[joint_names.index(joint_name)] = value
        if axis_constraint is not None:
            q = solve_tip_offset_ik_axis(
                env, joint_names, q, target, tip_link, tip_offset, tolerance_m,
                tool_axis, world_axis, axis_weight, axis_tolerance_rad,
                posture_targets, posture_weights).copy()
        else:
            q = solve_tip_offset_ik(env, joint_names, q, target, tip_link, tip_offset, tolerance_m).copy()
        # The external axes are position-commanded, not IK variables: pin the
        # requested values exactly so TrajOpt receives the prescribed sweep.
        for joint_name, value in target_joints.items():
            q[joint_names.index(joint_name)] = value
        seeds.append(q.copy())
    rows = run_trajopt(mods, env, joint_names, seeds, base_link, tip_link)
    max_tracking_error, path_length = enrich_rows(env, rows, joint_names, targets, tip_link, tip_offset, tcp_speed_m_s)
    max_axis_error_rad = 0.0
    if axis_constraint is not None:
        for row in rows:
            q_row = np.asarray(row["positions_rad"], dtype=float)
            axis = tool_axis_world(env, joint_names, q_row, tip_link, tool_axis)
            angle = math.acos(float(np.clip(np.dot(axis, world_axis), -1.0, 1.0)))
            row["tool_axis_error_deg"] = math.degrees(angle)
            max_axis_error_rad = max(max_axis_error_rad, angle)
    if frame_link:
        # Local coordinates are echoed against each row's OWN joint state so
        # harmonious positioner sweeps stay attached to the moving workpiece.
        for row in rows:
            q_row = np.asarray(row["positions_rad"], dtype=float)
            matrix = workpiece_frame_transform(env, joint_names, q_row, frame_link)
            target = np.asarray(row["target_xyz_m"], dtype=float)
            local = matrix[:3, :3].T @ (target - matrix[:3, 3])
            row["path_parent_link"] = frame_link
            row["target_local_xyz_m"] = [float(x) for x in local]
    goal = np.asarray(rows[-1]["positions_rad"], dtype=float)
    deltas = np.abs(goal - start)
    moving_joint_names = [name for name, delta in zip(joint_names, deltas) if float(delta) > 1e-6]
    success = max_tracking_error <= tolerance_m
    if axis_constraint is not None and max_axis_error_rad > axis_tolerance_rad:
        success = False
    return {
        "schema": RESULT_SCHEMA,
        "trajectory_schema": "urdf-viewer-trajectory-v1",
        "backend": "tesseract_v1_trajopt_joint_from_numeric_ik",
        "planner": "TrajOptMotionPlanner",
        "planner_task": demo.PLANNER_NAME,
        "success": success,
        "robot": robot.get("label", "generic"),
        "source_urdf": demo.repo_relative_path(urdf_path),
        "source_srdf": demo.repo_relative_path(srdf_path),
        "base_link": base_link,
        "tip_link": tip_link,
        "working_frame": working_frame,
        "workpiece_frame": {
            "link": frame_link,
            "joint_values_rad": frame_joints,
            "world_transform": [[float(v) for v in row] for row in frame_matrix],
        } if frame_link else None,
        "joint_names": joint_names,
        "start_rad": [float(x) for x in start],
        "goal_rad": [float(x) for x in goal],
        "summary": {
            "success": success,
            "waypoints": len(rows),
            "base_waypoints": len(seeds),
            "max_joint_delta_rad": float(np.max(deltas)),
            "moving_joint_count": len(moving_joint_names),
            "moving_joint_names": moving_joint_names,
            "tool_path_length_m": path_length,
            "max_tracking_error_m": max_tracking_error,
            "max_axis_error_deg": math.degrees(max_axis_error_rad) if axis_constraint is not None else None,
            "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
            "has_positions": True,
            "has_velocities": False,
            "has_accelerations": False,
            "has_torques": False,
        },
        "trajectory": rows,
        "errors": [] if success else [f"max tracking error {max_tracking_error:.6f}m exceeds tolerance {tolerance_m:.6f}m"],
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Run a generic Tesseract Cartesian plan request.")
    parser.add_argument("--self-test", action="store_true", help="run a small built-in planner smoke test")
    parser.add_argument("--spec", default="", help="tesseract-v1.cartesian-plan.v1 request JSON")
    parser.add_argument("--out", default="", help="optional path to write result JSON")
    parser.add_argument("--json", action="store_true", help="print result JSON to stdout")
    ns = parser.parse_args(argv)
    if ns.self_test:
        result = self_test()
        print(f"self-test: Cartesian planner produced {result['summary']['waypoints']} waypoints")
        return 0
    if not ns.spec:
        raise ValueError("--spec is required unless --self-test is set")
    result = plan_cartesian(load_plan(Path(ns.spec)))
    if ns.out:
        out_path = Path(ns.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if ns.json or not ns.out:
        print(json.dumps(result, indent=2))
    else:
        print(f"wrote Cartesian plan result: {ns.out}")
    return 0


def self_test() -> dict[str, Any]:
    return plan_cartesian(
        {
            "schema": PLAN_SCHEMA,
            "robot": {
                "label": "UR-10 self-test",
                "urdf": str(demo.asset_path("ur10_no_mesh.urdf")),
                "srdf": str(demo.asset_path("ur10.srdf")),
                "joint_names": demo.ARM_JOINT_NAMES,
                "base_link": "base_link",
                "working_frame": "base_link",
                "tip_link": "tool0",
                "tip_offset_xyz_m": [0, 0, 0],
            },
            "start_joint_rad": demo.DEFAULT_START,
            "targets": [
                {"pose": {"xyz_m": [0.40, -0.10, 0.25]}},
                {"pose": {"xyz_m": [0.42, -0.08, 0.25]}},
                {"pose": {"xyz_m": [0.44, -0.10, 0.25]}},
            ],
            "tcp_speed_m_s": 0.02,
            "planner": {"position_tolerance_m": 0.02},
        }
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker should print concise failures.
        print(f"cartesian-plan: {exc}", file=sys.stderr)
        raise SystemExit(1)
