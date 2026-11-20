#!/usr/bin/env python
"""No-ROS UR-10 TrajOpt demo for `rosie tesseract v1`.

The Go CLI owns the command surface. This worker keeps the Tesseract Python
bindings isolated behind a small JSON-producing entry point.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
HARMONIOUS_JOINT_NAMES = ["turntable_joint", *ARM_JOINT_NAMES]
ROSIE_ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ROSIE_HARMONIOUS_JOINT_NAMES = ["turntable_joint", *ROSIE_ARM_JOINT_NAMES]
ROSIE_V3_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]

DEFAULT_START = [0.0, -1.2, 1.6, -0.4, 1.2, 0.0]
DEFAULT_GOAL = [0.35, -0.85, 1.15, -0.25, 1.0, 0.45]
ROSIE_DEFAULT_START = [0.0, -1.05, 1.35, -0.45, 1.15, -0.2]
ROSIE_DEFAULT_GOAL = [0.18, -0.9, 1.25, -0.25, 0.95, 0.2]
PLANNER_NAME = "TrajOptMotionPlannerTask"
TURNTABLE_CENTER_XYZ = np.asarray([0.85, 0.0, 0.0], dtype=float)
TURNTABLE_TRACK_POINT_LOCAL_XYZ = np.asarray([0.0, 0.32, 0.065], dtype=float)
TCP_ABOVE_TRACK_POINT_M = 0.275
ARM_LIMITS = np.asarray(
    [
        [-6.283185307, 6.283185307],
        [-6.283185307, 6.283185307],
        [-3.1415926536, 3.1415926536],
        [-6.283185307, 6.283185307],
        [-6.283185307, 6.283185307],
        [-6.283185307, 6.283185307],
    ],
    dtype=float,
)
ARM_LIMITS_BY_NAME = {name: ARM_LIMITS[index] for index, name in enumerate(ARM_JOINT_NAMES)}
ROSIE_ARM_LIMITS_BY_NAME = {
    "joint1": np.asarray([-6.3, 6.3], dtype=float),
    "joint2": np.asarray([-1.9, 1.9], dtype=float),
    "joint3": np.asarray([-4.2, 1.53], dtype=float),
    "joint4": np.asarray([-6.3, 6.3], dtype=float),
    "joint5": np.asarray([-6.3, 6.3], dtype=float),
    "joint6": np.asarray([-10.0, 10.0], dtype=float),
}
PATH_PRESETS = {
    ("reach", 6): (DEFAULT_START, DEFAULT_GOAL),
    ("wrist-sweep", 6): ([0.0, -1.05, 1.35, -0.45, 1.15, -0.65], [0.08, -1.0, 1.28, 0.55, 0.72, 0.85]),
    ("vertical-arc", 6): ([-0.2, -1.3, 1.8, -0.55, 1.35, -0.3], [0.25, -0.72, 0.92, 0.18, 0.9, 0.42]),
    ("xy-square", 6): (DEFAULT_START, DEFAULT_START),
    ("reach", 7): ([0.0, *DEFAULT_START], [0.25, *DEFAULT_GOAL]),
    ("turntable-spin", 7): ([0.0, -0.05, -1.1, 1.45, -0.35, 1.05, -0.2], [math.pi, 0.05, -1.0, 1.3, -0.15, 0.9, 0.2]),
    ("harmonious", 7): ([-math.pi, *DEFAULT_START], [math.pi, *DEFAULT_GOAL]),
}
PATH_CHOICES = sorted({name for name, _ in PATH_PRESETS})
TRAJECTORY_METHOD_CHOICES = ["waypoints", "dense-csp", "spline-time", "cartesian-track"]
DEFAULT_TCP_SPEED_M_S = 0.08
DEFAULT_CSP_SAMPLE_PERIOD_S = 0.1
DEFAULT_SPLINE_SAMPLES = 81
DEFAULT_CARTESIAN_SAMPLES = 81
DEFAULT_SQUARE_CENTER_FRAME_XYZ = np.asarray([0.5, 0.0, 0.2], dtype=float)
DEFAULT_SQUARE_SIZE_M = 0.1
XY_SQUARE_SCHEMA = "tesseract-v1.xy-square-plane.v1"


def parse_joint_names(raw: str) -> list[str]:
    names = [p.strip() for p in raw.split(",") if p.strip()]
    if not names:
        raise ValueError("--joint-names must contain at least one joint name")
    if len(set(names)) != len(names):
        raise ValueError("--joint-names contains duplicate names")
    return names


def parse_joint_vector(raw: str, label: str, expected_len: int) -> np.ndarray:
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) != expected_len:
        raise ValueError(f"{label} must contain exactly {expected_len} comma-separated joint values")
    try:
        values = [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError(f"{label} contains a non-numeric joint value") from exc
    return np.asarray(values, dtype=float)


def parse_xyz_vector(raw: str, label: str) -> np.ndarray:
    return parse_joint_vector(raw, label, 3)


def preset_vectors(path_kind: str, joint_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    if path_kind == "harmonious" and joint_names == ROSIE_HARMONIOUS_JOINT_NAMES:
        return np.asarray([-math.pi, *ROSIE_DEFAULT_START], dtype=float), np.asarray([math.pi, *ROSIE_DEFAULT_GOAL], dtype=float)
    if joint_names == ROSIE_V3_JOINT_NAMES:
        rosie_presets = {
            "reach": (
                [*ROSIE_DEFAULT_START, 0.0, 0.0, 0.0],
                [*ROSIE_DEFAULT_GOAL, 0.0, 0.0, 0.0],
            ),
            "wrist-sweep": (
                [0.0, -1.05, 1.35, -0.45, 1.15, -0.65, 0.0, 0.0, 0.0],
                [0.08, -1.0, 1.28, 0.55, 0.72, 0.85, 0.0, 0.0, 0.0],
            ),
            "vertical-arc": (
                [-0.2, -1.3, 1.8, -0.55, 1.35, -0.3, 0.0, 0.0, 0.0],
                [0.25, -0.72, 0.92, 0.18, 0.9, 0.42, 0.0, 0.0, 0.0],
            ),
            "harmonious": (
                [*ROSIE_DEFAULT_START, -math.pi, 0.0, 0.0],
                [*ROSIE_DEFAULT_GOAL, math.pi, 0.0, 0.0],
            ),
        }
        if path_kind in rosie_presets:
            start, goal = rosie_presets[path_kind]
            return np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    key = (path_kind, len(joint_names))
    if key not in PATH_PRESETS:
        available = ", ".join(name for name, width in PATH_PRESETS if width == len(joint_names))
        raise ValueError(f"--path {path_kind!r} is not available for {len(joint_names)} joints; choices: {available}")
    start, goal = PATH_PRESETS[key]
    return np.asarray(start, dtype=float), np.asarray(goal, dtype=float)


def harmonious_arm_joint_names(joint_names: list[str]) -> list[str] | None:
    if len(joint_names) == 7 and joint_names[0] == "turntable_joint":
        return joint_names[1:]
    return None


def limits_for_arm_joints(arm_joint_names: list[str]) -> np.ndarray:
    limits: list[np.ndarray] = []
    for name in arm_joint_names:
        if name in ROSIE_ARM_LIMITS_BY_NAME:
            limits.append(ROSIE_ARM_LIMITS_BY_NAME[name])
        elif name in ARM_LIMITS_BY_NAME:
            limits.append(ARM_LIMITS_BY_NAME[name])
        else:
            limits.append(np.asarray([-6.283185307, 6.283185307], dtype=float))
    return np.asarray(limits, dtype=float)


def default_vector(values: Iterable[float]) -> str:
    return ",".join(f"{v:g}" for v in values)


def load_tesseract_modules():
    from tesseract_robotics.tesseract_common import (  # type: ignore
        FilesystemPath,
        GeneralResourceLocator,
        ManipulatorInfo,
    )
    from tesseract_robotics.tesseract_command_language import (  # type: ignore
        CompositeInstruction,
        DEFAULT_PROFILE_KEY,
        InstructionPoly_as_MoveInstructionPoly,
        JointWaypoint,
        JointWaypointPoly_wrap_JointWaypoint,
        MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction,
        MoveInstructionType_FREESPACE,
        ProfileDictionary,
        WaypointPoly_as_JointWaypointPoly,
        WaypointPoly_as_StateWaypointPoly,
    )
    from tesseract_robotics.tesseract_environment import Environment  # type: ignore
    from tesseract_robotics.tesseract_motion_planners import PlannerRequest  # type: ignore
    from tesseract_robotics.tesseract_motion_planners_trajopt import (  # type: ignore
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultPlanProfile,
        TrajOptMotionPlanner,
    )

    return {
        "CompositeInstruction": CompositeInstruction,
        "DEFAULT_PROFILE_KEY": DEFAULT_PROFILE_KEY,
        "Environment": Environment,
        "FilesystemPath": FilesystemPath,
        "GeneralResourceLocator": GeneralResourceLocator,
        "InstructionPoly_as_MoveInstructionPoly": InstructionPoly_as_MoveInstructionPoly,
        "JointWaypoint": JointWaypoint,
        "JointWaypointPoly_wrap_JointWaypoint": JointWaypointPoly_wrap_JointWaypoint,
        "ManipulatorInfo": ManipulatorInfo,
        "MoveInstruction": MoveInstruction,
        "MoveInstructionPoly_wrap_MoveInstruction": MoveInstructionPoly_wrap_MoveInstruction,
        "MoveInstructionType_FREESPACE": MoveInstructionType_FREESPACE,
        "PlannerRequest": PlannerRequest,
        "ProfileDictionary": ProfileDictionary,
        "TrajOptDefaultCompositeProfile": TrajOptDefaultCompositeProfile,
        "TrajOptDefaultPlanProfile": TrajOptDefaultPlanProfile,
        "TrajOptMotionPlanner": TrajOptMotionPlanner,
        "WaypointPoly_as_JointWaypointPoly": WaypointPoly_as_JointWaypointPoly,
        "WaypointPoly_as_StateWaypointPoly": WaypointPoly_as_StateWaypointPoly,
    }


def asset_path(name: str) -> Path:
    return Path(__file__).resolve().parents[1] / "assets" / name


def repo_relative_path(path: Path) -> str:
    resolved = path.resolve()
    for root in [Path.cwd().resolve(), *Path.cwd().resolve().parents]:
        if (root / "go.mod").is_file():
            try:
                return resolved.relative_to(root).as_posix()
            except ValueError:
                break
    return str(path)


def materialize_urdf_with_absolute_mesh_paths(
        urdf_path: Path, temp_dir: Path | None = None) -> Path:
    """Tesseract's no-ROS loader does not resolve URDF-relative mesh paths."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    changed = False
    if "xmlns:tesseract" not in root.attrib:
        root.set("xmlns:tesseract", "http://tesseract-robotics.github.io/tesseract")
        changed = True
    if "tesseract:make_convex" not in root.attrib:
        root.set("tesseract:make_convex", "false")
        changed = True
    for mesh in root.iter("mesh"):
        filename = (mesh.get("filename") or "").strip()
        if not filename:
            continue
        if filename.startswith("package://"):
            package_prefix = f"package://{urdf_path.parent.name}/"
            if filename.startswith(package_prefix):
                mesh.set("filename", (urdf_path.parent / filename[len(package_prefix):]).resolve().as_posix())
                changed = True
            continue
        if "://" in filename or Path(filename).is_absolute():
            continue
        mesh.set("filename", (urdf_path.parent / filename).resolve().as_posix())
        changed = True
    if not changed:
        return urdf_path

    output_dir = temp_dir or (Path(tempfile.gettempdir()) / "rosieos-tesseract")
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"{urdf_path.stem}_absolute_mesh.urdf"
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def transform_translation(transform) -> np.ndarray:
    value = transform.translation
    if callable(value):
        value = value()
    return np.asarray(value, dtype=float).flatten()


def transform_matrix(transform) -> np.ndarray:
    value = transform.matrix
    if callable(value):
        value = value()
    return np.asarray(value, dtype=float)


def planner_response_successful(response) -> bool:
    value = response.successful
    if callable(value):
        value = value()
    return bool(value)


def planner_response_message(response) -> str:
    value = response.message
    if callable(value):
        value = value()
    return str(value)


def parse_xyz_attr(raw: str | None, default: Iterable[float]) -> np.ndarray:
    if not raw:
        return np.asarray(list(default), dtype=float)
    return np.asarray([float(part) for part in raw.split()], dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(x) for x in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rpy_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.eye(3, dtype=float)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    v = 1 - c
    return np.asarray(
        [
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ],
        dtype=float,
    )


class URDFKinematics:
    def __init__(self, urdf_path: Path, base_link: str = "base_link", tip_link: str = "tool0"):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        children: dict[str, list[dict[str, object]]] = {}
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            origin = joint.find("origin")
            axis = joint.find("axis")
            item = {
                "name": joint.get("name", ""),
                "type": joint.get("type", "fixed"),
                "parent": parent.get("link", ""),
                "child": child.get("link", ""),
                "xyz": parse_xyz_attr(origin.get("xyz") if origin is not None else None, [0, 0, 0]),
                "rpy": parse_xyz_attr(origin.get("rpy") if origin is not None else None, [0, 0, 0]),
                "axis": parse_xyz_attr(axis.get("xyz") if axis is not None else None, [0, 0, 1]),
            }
            children.setdefault(str(item["parent"]), []).append(item)
        chain = self._find_chain(children, base_link, tip_link)
        if chain is None:
            raise RuntimeError(f"URDF has no chain from {base_link!r} to {tip_link!r}: {urdf_path}")
        self.chain = chain

    def _find_chain(self, children: dict[str, list[dict[str, object]]], link: str, tip: str) -> list[dict[str, object]] | None:
        if link == tip:
            return []
        for joint in children.get(link, []):
            suffix = self._find_chain(children, str(joint["child"]), tip)
            if suffix is not None:
                return [joint, *suffix]
        return None

    def tool_transform(self, joint_names: list[str], q: np.ndarray) -> np.ndarray:
        values = {name: float(value) for name, value in zip(joint_names, q)}
        transform = np.eye(4, dtype=float)
        for joint in self.chain:
            transform = transform @ origin_transform(np.asarray(joint["xyz"], dtype=float), np.asarray(joint["rpy"], dtype=float))
            if joint["type"] in {"revolute", "continuous"}:
                rotation = np.eye(4, dtype=float)
                rotation[:3, :3] = axis_angle_matrix(np.asarray(joint["axis"], dtype=float), values.get(str(joint["name"]), 0.0))
                transform = transform @ rotation
        return transform

    def tool_xyz(self, joint_names: list[str], q: np.ndarray) -> np.ndarray:
        return self.tool_transform(joint_names, q)[:3, 3]


def solve_urdf_position_ik(model: URDFKinematics, target_xyz: np.ndarray, seed: np.ndarray, joint_names: list[str]) -> np.ndarray:
    q = np.asarray(seed, dtype=float).copy()
    limits = limits_for_arm_joints(joint_names)
    eps = 1e-4
    damping = 0.055
    for _ in range(180):
        current = model.tool_xyz(joint_names, q)
        error = target_xyz - current
        if float(np.linalg.norm(error)) < 0.006:
            break
        jac = np.zeros((3, len(joint_names)), dtype=float)
        for col in range(len(joint_names)):
            step = np.zeros(len(joint_names), dtype=float)
            step[col] = eps
            plus = model.tool_xyz(joint_names, q + step)
            minus = model.tool_xyz(joint_names, q - step)
            jac[:, col] = (plus - minus) / (2.0 * eps)
        lhs = jac @ jac.T + (damping * damping) * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, error)
        q += np.clip(dq, -0.14, 0.14)
        q = np.clip(q, limits[:, 0], limits[:, 1])
    return q


def run_urdf_xy_square_demo(
    start: np.ndarray,
    joint_names: list[str],
    urdf_path: Path,
    srdf_path: Path,
    robot_label: str,
    viewer_model: str,
    working_frame: str,
    square_center_xyz: np.ndarray,
    square_size_m: float,
) -> dict[str, object]:
    model = URDFKinematics(urdf_path)
    targets = square_points_frame(square_center_xyz, square_size_m)
    q = np.asarray(start, dtype=float).copy()
    rows: list[dict[str, object]] = []
    previous_tool: np.ndarray | None = None
    path_length = 0.0
    max_tracking_error = 0.0
    for index, target_frame in enumerate(targets):
        target_ur = frame_xyz_to_ur_base_xyz(target_frame)
        q = solve_urdf_position_ik(model, target_ur, q, joint_names)
        tool_frame = ur_base_xyz_to_frame_xyz(model.tool_xyz(joint_names, q))
        if previous_tool is not None:
            path_length += float(np.linalg.norm(tool_frame - previous_tool))
        previous_tool = tool_frame
        tracking_error = float(np.linalg.norm(tool_frame - target_frame))
        max_tracking_error = max(max_tracking_error, tracking_error)
        rows.append(
            {
                "index": index,
                "time_s": float(index),
                "positions_rad": [float(x) for x in q],
                "positions_deg": [float(math.degrees(x)) for x in q],
                "target_xyz_m": [float(x) for x in target_frame],
                "tool0_xyz_m": [float(x) for x in tool_frame],
                "tracking_error_m": tracking_error,
                "path_kind": "xy-plane-square",
            }
        )
    for row in rows:
        row["tool_path_length_m"] = path_length
    deltas = np.abs(np.asarray(rows[-1]["positions_rad"], dtype=float) - np.asarray(rows[0]["positions_rad"], dtype=float))
    moving_joint_names = [name for name, delta in zip(joint_names, deltas) if float(delta) > 1e-6]
    return {
        "schema": XY_SQUARE_SCHEMA,
        "trajectory_schema": "urdf-viewer-trajectory-v1",
        "backend": "urdf_numeric_ik",
        "planner": "URDFNumericIK",
        "planner_task": "URDFXYSquarePath",
        "robot": robot_label,
        "source": "tesseract/v1 demo --path xy-square",
        "source_urdf": repo_relative_path(urdf_path),
        "source_srdf": repo_relative_path(srdf_path),
        "viewer_model": viewer_model,
        "path_kind": "xy-square",
        "trajectory_method": "waypoints",
        "interpolation": "urdf_numeric_ik_waypoints",
        "servo_mode": "none",
        "method_notes": "URDF-based square path: target points are in the requested working frame; IK is solved against the URDF chain.",
        "sample_period_s": 0.0,
        "tcp_speed_m_s": DEFAULT_TCP_SPEED_M_S,
        "working_frame": working_frame,
        "frame": working_frame,
        "units": "meters",
        "ros": False,
        "joint_names": joint_names,
        "start_rad": rows[0]["positions_rad"],
        "goal_rad": rows[-1]["positions_rad"],
        "description": "Small square target in the X/Y plane, centered 500 mm in +X and 200 mm in +Z from the origin.",
        "square_center_xyz_m": [float(x) for x in square_center_xyz],
        "square_size_m": float(square_size_m),
        "summary": {
            "waypoints": len(rows),
            "base_waypoints": len(rows),
            "max_joint_delta_rad": float(np.max(deltas)),
            "moving_joint_count": len(moving_joint_names),
            "moving_joint_names": moving_joint_names,
            "tool_path_length_m": path_length,
            "max_tracking_error_m": max_tracking_error,
            "duration_s": 0.0,
            "has_positions": True,
            "has_velocities": False,
            "has_accelerations": False,
            "has_torques": False,
        },
        "trajectory": rows,
    }


def make_environment(mods: dict[str, object], start: np.ndarray, joint_names: list[str], urdf_path: Path, srdf_path: Path):
    environment_cls = mods["Environment"]
    filesystem_path = mods["FilesystemPath"]
    locator_cls = mods["GeneralResourceLocator"]

    env = environment_cls()
    tesseract_urdf_path = materialize_urdf_with_absolute_mesh_paths(urdf_path)
    ok = env.init(filesystem_path(str(tesseract_urdf_path)), filesystem_path(str(srdf_path)), locator_cls())
    if not ok:
        raise RuntimeError(f"Tesseract failed to initialize UR-10 assets: {urdf_path}, {srdf_path}")
    env.setState(joint_names, start)
    return env


def build_program(mods: dict[str, object], waypoints: list[np.ndarray], joint_names: list[str], working_frame: str):
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
    manip.tcp_frame = "tool0"

    program = composite_instruction(default_profile_key)
    program.setManipulatorInfo(manip)
    for q in waypoints:
        waypoint = joint_waypoint(joint_names, q)
        instruction = move_instruction(joint_waypoint_wrap(waypoint), move_type, default_profile_key)
        program.appendMoveInstruction(move_instruction_wrap(instruction))
    return program


def make_profiles(mods: dict[str, object]):
    default_profile_key = mods["DEFAULT_PROFILE_KEY"]
    profile_dictionary = mods["ProfileDictionary"]
    composite_profile_cls = mods["TrajOptDefaultCompositeProfile"]
    plan_profile_cls = mods["TrajOptDefaultPlanProfile"]

    composite_profile = composite_profile_cls()
    composite_profile.smooth_velocities = True
    composite_profile.smooth_accelerations = True
    composite_profile.smooth_jerks = False
    composite_profile.collision_cost_config.enabled = True
    composite_profile.collision_constraint_config.enabled = True

    profiles = profile_dictionary()
    profiles.addProfile(PLANNER_NAME, default_profile_key, composite_profile)
    profiles.addProfile(PLANNER_NAME, default_profile_key, plan_profile_cls())
    return profiles


def interpolate_waypoints(start: np.ndarray, goal: np.ndarray, steps: int) -> list[np.ndarray]:
    return [start + (goal - start) * (index / (steps - 1)) for index in range(steps)]


def turntable_track_point_xyz(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    x, y, z = TURNTABLE_TRACK_POINT_LOCAL_XYZ
    return TURNTABLE_CENTER_XYZ + np.asarray([c * x - s * y, s * x + c * y, z], dtype=float)


def tcp_target_above_track_point(track_point_xyz: np.ndarray) -> np.ndarray:
    return track_point_xyz + np.asarray([0.0, 0.0, TCP_ABOVE_TRACK_POINT_M], dtype=float)


def arm_tool_xyz(env, arm_q: np.ndarray, arm_joint_names: list[str]) -> np.ndarray:
    env.setState(arm_joint_names, arm_q)
    return transform_translation(env.getLinkTransform("tool0"))


def solve_arm_position_ik(env, target_xyz: np.ndarray, seed: np.ndarray, arm_joint_names: list[str]) -> np.ndarray:
    q = np.asarray(seed, dtype=float).copy()
    limits = limits_for_arm_joints(arm_joint_names)
    eps = 1e-4
    damping = 0.055
    for _ in range(140):
        current = arm_tool_xyz(env, q, arm_joint_names)
        error = target_xyz - current
        if float(np.linalg.norm(error)) < 0.006:
            break
        jac = np.zeros((3, len(arm_joint_names)), dtype=float)
        for col in range(len(arm_joint_names)):
            step = np.zeros(len(arm_joint_names), dtype=float)
            step[col] = eps
            plus = arm_tool_xyz(env, q + step, arm_joint_names)
            minus = arm_tool_xyz(env, q - step, arm_joint_names)
            jac[:, col] = (plus - minus) / (2.0 * eps)
        lhs = jac @ jac.T + (damping * damping) * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, error)
        q += np.clip(dq, -0.14, 0.14)
        q = np.clip(q, limits[:, 0], limits[:, 1])
    return q


def harmonious_tracking_waypoints(
    env,
    seed: np.ndarray,
    steps: int,
    arm_joint_names: list[str],
) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    default_seed = ROSIE_DEFAULT_START if arm_joint_names == ROSIE_ARM_JOINT_NAMES else DEFAULT_START
    arm_seed = np.asarray(seed[1:] if len(seed) == 7 else default_seed, dtype=float)
    waypoints: list[np.ndarray] = []
    tcp_targets: list[np.ndarray] = []
    track_points: list[np.ndarray] = []
    for index in range(steps):
        ratio = index / (steps - 1)
        theta = -math.pi + 2.0 * math.pi * ratio
        track_point = turntable_track_point_xyz(theta)
        tcp_target = tcp_target_above_track_point(track_point)
        arm_seed = solve_arm_position_ik(env, tcp_target, arm_seed, arm_joint_names)
        # Wrist 3 mostly spins about the TCP axis, so use it to make all seven
        # axes visibly participate without disturbing the tracked point.
        arm_seed = arm_seed.copy()
        arm_seed[5] = -0.35 + 0.7 * ratio
        waypoints.append(np.asarray([theta, *arm_seed], dtype=float))
        tcp_targets.append(tcp_target)
        track_points.append(track_point)
    return waypoints, tcp_targets, track_points


def extract_positions(mods: dict[str, object], results) -> list[dict[str, object]]:
    as_move = mods["InstructionPoly_as_MoveInstructionPoly"]
    as_state = mods["WaypointPoly_as_StateWaypointPoly"]
    as_joint = mods["WaypointPoly_as_JointWaypointPoly"]

    rows: list[dict[str, object]] = []
    for index, instruction in enumerate(results):
        move_instruction = as_move(instruction)
        waypoint = move_instruction.getWaypoint()
        time_s = 0.0
        if waypoint.isStateWaypoint():
            state_waypoint = as_state(waypoint)
            position = state_waypoint.getPosition().flatten()
            time_s = float(state_waypoint.getTime())
        elif waypoint.isJointWaypoint():
            joint_waypoint = as_joint(waypoint)
            position = joint_waypoint.getPosition().flatten()
        else:
            raise RuntimeError(f"planner returned unsupported waypoint at index {index}")
        rows.append(
            {
                "index": index,
                "time_s": time_s,
                "positions_rad": [float(x) for x in position],
                "positions_deg": [float(math.degrees(x)) for x in position],
            }
        )
    return rows


def enrich_tool_poses(env, rows: list[dict[str, object]], joint_names: list[str]) -> None:
    previous_xyz: np.ndarray | None = None
    path_length = 0.0
    for row in rows:
        q = np.asarray(row["positions_rad"], dtype=float)
        env.setState(joint_names, q)
        transform = env.getLinkTransform("tool0")
        matrix = transform_matrix(transform)
        xyz = transform_translation(transform)
        if previous_xyz is not None:
            path_length += float(np.linalg.norm(xyz - previous_xyz))
        previous_xyz = xyz
        row["tool0_xyz_m"] = [float(x) for x in xyz]
        row["tool0_matrix"] = [[float(v) for v in matrix[r, :]] for r in range(4)]
    for row in rows:
        row["tool_path_length_m"] = path_length


def rows_from_positions(positions: list[np.ndarray], times: list[float]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for index, (position, time_s) in enumerate(zip(positions, times)):
        rows.append(
            {
                "index": index,
                "time_s": float(time_s),
                "positions_rad": [float(x) for x in position],
                "positions_deg": [float(math.degrees(x)) for x in position],
            }
        )
    return rows


def add_derivatives(rows: list[dict[str, object]]) -> None:
    if len(rows) < 2:
        return
    positions = np.asarray([row["positions_rad"] for row in rows], dtype=float)
    times = np.asarray([float(row.get("time_s", 0.0)) for row in rows], dtype=float)
    if not np.all(np.diff(times) > 0):
        return

    velocities = np.zeros_like(positions)
    accelerations = np.zeros_like(positions)
    for index in range(len(rows)):
        if index == 0:
            dt = times[1] - times[0]
            velocities[index] = (positions[1] - positions[0]) / dt
        elif index == len(rows) - 1:
            dt = times[-1] - times[-2]
            velocities[index] = (positions[-1] - positions[-2]) / dt
        else:
            dt = times[index + 1] - times[index - 1]
            velocities[index] = (positions[index + 1] - positions[index - 1]) / dt

    for index in range(len(rows)):
        if index == 0:
            dt = times[1] - times[0]
            accelerations[index] = (velocities[1] - velocities[0]) / dt
        elif index == len(rows) - 1:
            dt = times[-1] - times[-2]
            accelerations[index] = (velocities[-1] - velocities[-2]) / dt
        else:
            dt = times[index + 1] - times[index - 1]
            accelerations[index] = (velocities[index + 1] - velocities[index - 1]) / dt

    for row, velocity, acceleration in zip(rows, velocities, accelerations):
        row["velocities_rad_s"] = [float(x) for x in velocity]
        row["accelerations_rad_s2"] = [float(x) for x in acceleration]


def piecewise_linear_positions(control_positions: list[np.ndarray], sample_count: int) -> list[np.ndarray]:
    if sample_count <= len(control_positions):
        return [np.asarray(q, dtype=float).copy() for q in control_positions]
    segment_count = len(control_positions) - 1
    samples: list[np.ndarray] = []
    for sample_index in range(sample_count):
        u = sample_index / (sample_count - 1)
        scaled = u * segment_count
        lower = min(int(math.floor(scaled)), segment_count - 1)
        blend = scaled - lower
        samples.append(control_positions[lower] + (control_positions[lower + 1] - control_positions[lower]) * blend)
    return samples


def cubic_hermite_positions(control_positions: list[np.ndarray], sample_count: int) -> list[np.ndarray]:
    controls = [np.asarray(q, dtype=float) for q in control_positions]
    if sample_count <= len(controls):
        return [q.copy() for q in controls]
    tangents: list[np.ndarray] = []
    for index, q in enumerate(controls):
        if index == 0:
            tangents.append(controls[1] - q)
        elif index == len(controls) - 1:
            tangents.append(q - controls[index - 1])
        else:
            tangents.append((controls[index + 1] - controls[index - 1]) * 0.5)

    segment_count = len(controls) - 1
    samples: list[np.ndarray] = []
    for sample_index in range(sample_count):
        u = sample_index / (sample_count - 1)
        scaled = u * segment_count
        lower = min(int(math.floor(scaled)), segment_count - 1)
        t = scaled - lower
        t2 = t * t
        t3 = t2 * t
        h00 = 2.0 * t3 - 3.0 * t2 + 1.0
        h10 = t3 - 2.0 * t2 + t
        h01 = -2.0 * t3 + 3.0 * t2
        h11 = t3 - t2
        samples.append(h00 * controls[lower] + h10 * tangents[lower] + h01 * controls[lower + 1] + h11 * tangents[lower + 1])
    return samples


def uniform_times(sample_count: int, duration_s: float) -> list[float]:
    if sample_count <= 1:
        return [0.0]
    return [duration_s * (index / (sample_count - 1)) for index in range(sample_count)]


def assign_harmonious_tracking_fields(rows: list[dict[str, object]]) -> float:
    max_tracking_error = 0.0
    for row in rows:
        position = np.asarray(row["positions_rad"], dtype=float)
        if len(position) == 0:
            continue
        track_point = turntable_track_point_xyz(float(position[0]))
        target = tcp_target_above_track_point(track_point)
        row["target_xyz_m"] = [float(x) for x in target]
        row["track_point_xyz_m"] = [float(x) for x in track_point]
        row["tcp_above_track_point_m"] = TCP_ABOVE_TRACK_POINT_M
        if "tool0_xyz_m" in row:
            tool_xyz = np.asarray(row["tool0_xyz_m"], dtype=float)
            tracking_error = float(np.linalg.norm(tool_xyz - target))
            row["tracking_error_m"] = tracking_error
            max_tracking_error = max(max_tracking_error, tracking_error)
    return max_tracking_error


def make_cartesian_tracking_positions(env, sample_count: int, arm_joint_names: list[str]) -> list[np.ndarray]:
    arm_seed = np.asarray(ROSIE_DEFAULT_START if arm_joint_names == ROSIE_ARM_JOINT_NAMES else DEFAULT_START, dtype=float)
    positions: list[np.ndarray] = []
    for index in range(sample_count):
        ratio = index / (sample_count - 1)
        theta = -math.pi + 2.0 * math.pi * ratio
        track_point = turntable_track_point_xyz(theta)
        tcp_target = tcp_target_above_track_point(track_point)
        arm_seed = solve_arm_position_ik(env, tcp_target, arm_seed, arm_joint_names).copy()
        arm_seed[5] = -0.35 + 0.7 * ratio
        positions.append(np.asarray([theta, *arm_seed], dtype=float))
    return positions


def frame_xyz_to_ur_base_xyz(point: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(point, dtype=float)
    return np.asarray([-y, x, z], dtype=float)


def ur_base_xyz_to_frame_xyz(point: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(point, dtype=float)
    return np.asarray([y, -x, z], dtype=float)


def square_points_frame(center_xyz: np.ndarray, size_m: float) -> list[np.ndarray]:
    half = float(size_m) / 2.0
    cx, cy, cz = np.asarray(center_xyz, dtype=float)
    return [
        np.asarray([cx - half, cy - half, cz], dtype=float),
        np.asarray([cx + half, cy - half, cz], dtype=float),
        np.asarray([cx + half, cy + half, cz], dtype=float),
        np.asarray([cx - half, cy + half, cz], dtype=float),
        np.asarray([cx - half, cy - half, cz], dtype=float),
    ]


def square_tracking_waypoints(
    env,
    center_frame_xyz: np.ndarray,
    size_m: float,
    seed: np.ndarray,
    arm_joint_names: list[str],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    arm_seed = np.asarray(seed, dtype=float).copy()
    waypoints: list[np.ndarray] = []
    targets_frame = square_points_frame(center_frame_xyz, size_m)
    for target_frame in targets_frame:
        target_ur_base = frame_xyz_to_ur_base_xyz(target_frame)
        arm_seed = solve_arm_position_ik(env, target_ur_base, arm_seed, arm_joint_names).copy()
        waypoints.append(arm_seed.copy())
    return waypoints, targets_frame


def assign_xy_square_fields(rows: list[dict[str, object]], targets_frame: list[np.ndarray]) -> float:
    max_tracking_error = 0.0
    for row in rows:
        index = min(int(row["index"]), len(targets_frame) - 1)
        target = targets_frame[index]
        row["target_xyz_m"] = [float(x) for x in target]
        if "tool0_xyz_m" in row:
            tool_frame = ur_base_xyz_to_frame_xyz(np.asarray(row["tool0_xyz_m"], dtype=float))
            row["tool0_xyz_m"] = [float(x) for x in tool_frame]
            tracking_error = float(np.linalg.norm(tool_frame - target))
            row["tracking_error_m"] = tracking_error
            max_tracking_error = max(max_tracking_error, tracking_error)
        row["path_kind"] = "xy-plane-square"
    return max_tracking_error


def trajectory_duration_s(tool_path_length: float, tcp_speed_m_s: float) -> float:
    speed = max(0.001, float(tcp_speed_m_s))
    return max(1.0, float(tool_path_length) / speed)


def run_demo(
    start: np.ndarray,
    goal: np.ndarray,
    steps: int,
    joint_names: list[str] | None = None,
    urdf_path: Path | None = None,
    srdf_path: Path | None = None,
    robot_label: str = "UR-10",
    viewer_model: str = "",
    path_kind: str = "reach",
    working_frame: str = "",
    trajectory_method: str = "waypoints",
    csp_sample_period_s: float = DEFAULT_CSP_SAMPLE_PERIOD_S,
    tcp_speed_m_s: float = DEFAULT_TCP_SPEED_M_S,
    spline_samples: int = DEFAULT_SPLINE_SAMPLES,
    cartesian_samples: int = DEFAULT_CARTESIAN_SAMPLES,
    square_center_xyz: np.ndarray | None = None,
    square_size_m: float = DEFAULT_SQUARE_SIZE_M,
) -> dict[str, object]:
    if steps < 3:
        raise ValueError("--steps must be at least 3 because TrajOpt acceleration smoothing needs three states")
    if trajectory_method not in TRAJECTORY_METHOD_CHOICES:
        raise ValueError(f"--trajectory-method must be one of {', '.join(TRAJECTORY_METHOD_CHOICES)}")

    resolved_joint_names = joint_names or ARM_JOINT_NAMES
    if len(start) != len(resolved_joint_names) or len(goal) != len(resolved_joint_names):
        raise ValueError("start, goal, and joint_names must have matching lengths")

    resolved_urdf = urdf_path or asset_path("ur10_no_mesh.urdf")
    resolved_srdf = srdf_path or asset_path("ur10.srdf")
    resolved_working_frame = working_frame or "base_link"
    if path_kind == "xy-square":
        return run_urdf_xy_square_demo(
            start,
            resolved_joint_names,
            resolved_urdf,
            resolved_srdf,
            robot_label,
            viewer_model,
            resolved_working_frame,
            square_center_xyz if square_center_xyz is not None else DEFAULT_SQUARE_CENTER_FRAME_XYZ,
            square_size_m,
        )
    mods = load_tesseract_modules()
    env = make_environment(mods, start, resolved_joint_names, resolved_urdf, resolved_srdf)
    harmonious_arm_names = harmonious_arm_joint_names(resolved_joint_names)
    target_positions: list[np.ndarray] = []
    track_point_positions: list[np.ndarray] = []
    square_targets_frame: list[np.ndarray] = []
    if path_kind == "xy-square":
        waypoints, square_targets_frame = square_tracking_waypoints(
            env,
            square_center_xyz if square_center_xyz is not None else DEFAULT_SQUARE_CENTER_FRAME_XYZ,
            square_size_m,
            start,
            resolved_joint_names,
        )
        start = waypoints[0]
        goal = waypoints[-1]
        steps = len(waypoints)
    elif path_kind == "harmonious" and harmonious_arm_names is not None:
        waypoints, target_positions, track_point_positions = harmonious_tracking_waypoints(env, start, steps, harmonious_arm_names)
        start = waypoints[0]
        goal = waypoints[-1]
    else:
        waypoints = interpolate_waypoints(start, goal, steps)
    program = build_program(mods, waypoints, resolved_joint_names, resolved_working_frame)
    profiles = make_profiles(mods)

    request_cls = mods["PlannerRequest"]
    planner_cls = mods["TrajOptMotionPlanner"]

    request = request_cls()
    request.env = env
    request.instructions = program
    request.profiles = profiles
    request.verbose = False

    planner = planner_cls(PLANNER_NAME)
    if hasattr(planner, "checkRequest") and not planner.checkRequest(request):
        raise RuntimeError("Tesseract rejected the TrajOpt planner request")
    response = planner.solve(request)
    if not planner_response_successful(response):
        raise RuntimeError(f"TrajOpt failed: {planner_response_message(response)}")

    base_rows = extract_positions(mods, response.results)
    enrich_tool_poses(env, base_rows, resolved_joint_names)
    base_positions = [np.asarray(row["positions_rad"], dtype=float) for row in base_rows]
    base_tool_path_length = float(base_rows[0]["tool_path_length_m"]) if base_rows else 0.0
    duration_s = trajectory_duration_s(base_tool_path_length, tcp_speed_m_s)

    method_notes = ""
    servo_mode = "none"
    interpolation = "tesseract_trajopt_waypoints"
    sample_period_s = 0.0
    if trajectory_method == "waypoints":
        rows = base_rows
        method_notes = "TrajOpt waypoints with viewer-side joint-linear interpolation between rows."
    elif trajectory_method == "dense-csp":
        sample_period_s = max(0.001, float(csp_sample_period_s))
        sample_count = max(len(base_positions), int(math.ceil(duration_s / sample_period_s)) + 1)
        positions = piecewise_linear_positions(base_positions, sample_count)
        rows = rows_from_positions(positions, uniform_times(len(positions), duration_s))
        add_derivatives(rows)
        enrich_tool_poses(env, rows, resolved_joint_names)
        interpolation = "joint_linear_resampled"
        servo_mode = "CSP"
        method_notes = "Dense Cyclic Synchronous Position example: joint positions are resampled onto a fixed sample period."
    elif trajectory_method == "spline-time":
        sample_count = max(len(base_positions), int(spline_samples))
        positions = cubic_hermite_positions(base_positions, sample_count)
        rows = rows_from_positions(positions, uniform_times(len(positions), duration_s))
        add_derivatives(rows)
        enrich_tool_poses(env, rows, resolved_joint_names)
        interpolation = "joint_cubic_hermite_time_parameterized"
        servo_mode = "CSP"
        method_notes = "Spline/time-parameterized example: cubic Hermite joint interpolation with time, velocity, and acceleration fields."
    else:
        if path_kind != "harmonious" or harmonious_arm_names is None:
            raise ValueError("--trajectory-method cartesian-track currently requires --path harmonious with a 7-axis turntable joint set")
        sample_count = max(3, int(cartesian_samples))
        positions = make_cartesian_tracking_positions(env, sample_count, harmonious_arm_names)
        rows = rows_from_positions(positions, uniform_times(len(positions), duration_s))
        add_derivatives(rows)
        enrich_tool_poses(env, rows, resolved_joint_names)
        interpolation = "cartesian_target_resolved_ik"
        servo_mode = "CSP"
        method_notes = "Cartesian tracking example: the turntable angle defines the weld target and each sample is resolved with IK."

    max_tracking_error = 0.0
    if path_kind == "harmonious" and harmonious_arm_names is not None:
        max_tracking_error = assign_harmonious_tracking_fields(rows)
    if path_kind == "xy-square":
        max_tracking_error = assign_xy_square_fields(rows, square_targets_frame)

    deltas = np.abs(goal - start)
    max_joint_delta = float(np.max(deltas))
    moving_joint_names = [name for name, delta in zip(resolved_joint_names, deltas) if float(delta) > 1e-6]
    tool_path_length = float(rows[0]["tool_path_length_m"]) if rows else 0.0
    result = {
        "trajectory_schema": "urdf-viewer-trajectory-v1",
        "backend": "tesseract_robotics.tesseract_motion_planners_trajopt",
        "planner": "TrajOptMotionPlanner",
        "planner_task": PLANNER_NAME,
        "collision_cost_enabled": True,
        "collision_constraint_enabled": True,
        "collision_prevention_model": "rosie_1400_v3_mesh_urdf"
        if robot_label == "rosie_1400_v3"
        else "tesseract_environment_collision_geometry",
        "robot": robot_label,
        "source_urdf": repo_relative_path(resolved_urdf),
        "source_srdf": repo_relative_path(resolved_srdf),
        "viewer_model": viewer_model,
        "path_kind": path_kind,
        "trajectory_method": trajectory_method,
        "interpolation": interpolation,
        "servo_mode": servo_mode,
        "method_notes": method_notes,
        "sample_period_s": sample_period_s,
        "tcp_speed_m_s": float(tcp_speed_m_s),
        "working_frame": resolved_working_frame,
        "ros": False,
        "joint_names": resolved_joint_names,
        "start_rad": [float(x) for x in start],
        "goal_rad": [float(x) for x in goal],
        "summary": {
            "waypoints": len(rows),
            "base_waypoints": len(base_rows),
            "max_joint_delta_rad": max_joint_delta,
            "moving_joint_count": len(moving_joint_names),
            "moving_joint_names": moving_joint_names,
            "tool_path_length_m": tool_path_length,
            "max_tracking_error_m": max_tracking_error,
            "duration_s": duration_s if trajectory_method != "waypoints" else 0.0,
            "has_positions": True,
            "has_velocities": trajectory_method != "waypoints",
            "has_accelerations": trajectory_method != "waypoints",
            "has_torques": False,
        },
        "trajectory": rows,
    }
    if path_kind == "xy-square":
        center = square_center_xyz if square_center_xyz is not None else DEFAULT_SQUARE_CENTER_FRAME_XYZ
        result.update(
            {
                "schema": XY_SQUARE_SCHEMA,
                "source": "tesseract/v1 demo --path xy-square",
                "frame": resolved_working_frame,
                "units": "meters",
                "description": "Small square target in the X/Y plane, centered 500 mm in +X and 200 mm in +Z from the origin.",
                "square_center_xyz_m": [float(x) for x in center],
                "square_size_m": float(square_size_m),
            }
        )
    return result


def print_human(result: dict[str, object]) -> None:
    summary = result["summary"]
    print("UR-10 TrajOpt demo (no ROS)")
    print(f"planner: {result['planner']} via {result['backend']}")
    summary_line = {**summary, "path_kind": result["path_kind"], "trajectory_method": result["trajectory_method"]}
    print(
        "path: {path_kind}  method: {trajectory_method}  waypoints: {waypoints}  moving_axes: {moving_joint_count}  max_joint_delta_rad: {max_joint_delta_rad:.4f}  "
        "tool_path_length_m: {tool_path_length_m:.4f}".format(**summary_line)
    )
    print("")
    print(f"joints: {', '.join(result['joint_names'])}")
    print("")
    print("idx  q_deg                                             tool0_xyz_m")
    rows = list(result["trajectory"])
    if len(rows) > 60:
        display_rows: list[dict[str, object] | None] = [*rows[:28], None, *rows[-12:]]
    else:
        display_rows = rows
    for row in display_rows:
        if row is None:
            print(f"...  {len(rows) - 40} rows omitted from table; JSON output contains all rows")
            continue
        q = row["positions_deg"]
        xyz = row["tool0_xyz_m"]
        q_text = " ".join(f"{v:7.2f}" for v in q)
        xyz_text = " ".join(f"{v:8.4f}" for v in xyz)
        print(f"{row['index']:>3}  {q_text}    {xyz_text}")


def command_demo(args: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="ur10_demo.py demo")
    parser.add_argument("--path", choices=PATH_CHOICES, default="reach", help="named path preset")
    parser.add_argument("--joint-names", default=",".join(ARM_JOINT_NAMES), help="comma-separated joint names matching start/goal")
    parser.add_argument("--start", default="", help="comma-separated start joint angles in radians")
    parser.add_argument("--goal", default="", help="comma-separated goal joint angles in radians")
    parser.add_argument("--steps", type=int, default=9, help="number of TrajOpt waypoints")
    parser.add_argument("--trajectory-method", choices=TRAJECTORY_METHOD_CHOICES, default="waypoints", help="viewer/servo trajectory method")
    parser.add_argument("--csp-sample-period", type=float, default=DEFAULT_CSP_SAMPLE_PERIOD_S, help="dense CSP example sample period in seconds")
    parser.add_argument("--tcp-speed", type=float, default=DEFAULT_TCP_SPEED_M_S, help="nominal TCP speed used for time-parameterized examples")
    parser.add_argument("--spline-samples", type=int, default=DEFAULT_SPLINE_SAMPLES, help="spline/time example sample count")
    parser.add_argument("--cartesian-samples", type=int, default=DEFAULT_CARTESIAN_SAMPLES, help="Cartesian tracking example sample count")
    parser.add_argument("--square-center", default=default_vector(DEFAULT_SQUARE_CENTER_FRAME_XYZ), help="square center XYZ in meters for --path xy-square")
    parser.add_argument("--square-size", type=float, default=DEFAULT_SQUARE_SIZE_M, help="square edge length in meters for --path xy-square")
    parser.add_argument("--json", nargs="?", const="-", default="", help="print JSON, or write JSON to this path")
    parser.add_argument("--out", default="", help="optional path to write the JSON trajectory")
    parser.add_argument("--urdf", default="", help="optional URDF path")
    parser.add_argument("--srdf", default="", help="optional SRDF path")
    parser.add_argument("--robot-label", default="UR-10", help="robot label for the JSON payload")
    parser.add_argument("--viewer-model", default="", help="URDF viewer model key for the JSON payload")
    parser.add_argument("--working-frame", default="", help="Manipulator working frame")
    ns = parser.parse_args(args)

    joint_names = parse_joint_names(ns.joint_names)
    preset_start, preset_goal = preset_vectors(ns.path, joint_names)
    start = parse_joint_vector(ns.start, "--start", len(joint_names)) if ns.start else preset_start
    goal = parse_joint_vector(ns.goal, "--goal", len(joint_names)) if ns.goal else preset_goal
    urdf_path = Path(ns.urdf) if ns.urdf else None
    srdf_path = Path(ns.srdf) if ns.srdf else None
    square_center = parse_xyz_vector(ns.square_center, "--square-center")
    if ns.square_size <= 0:
        raise ValueError("--square-size must be positive")
    result = run_demo(
        start,
        goal,
        ns.steps,
        joint_names,
        urdf_path,
        srdf_path,
        ns.robot_label,
        ns.viewer_model,
        ns.path,
        ns.working_frame,
        ns.trajectory_method,
        ns.csp_sample_period,
        ns.tcp_speed,
        ns.spline_samples,
        ns.cartesian_samples,
        square_center,
        ns.square_size,
    )

    json_path = ns.out or ("" if ns.json == "-" else ns.json)
    if json_path:
        out_path = Path(json_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    if ns.json == "-":
        print(json.dumps(result, indent=2))
    else:
        print_human(result)
        if json_path:
            print("")
            print(f"wrote JSON trajectory: {json_path}")
    return 0


def self_test() -> int:
    result = run_demo(np.asarray(DEFAULT_START), np.asarray(DEFAULT_GOAL), 5)
    trajectory = result["trajectory"]
    assert len(trajectory) == 5
    np.testing.assert_allclose(trajectory[0]["positions_rad"], DEFAULT_START, atol=1e-8)
    np.testing.assert_allclose(trajectory[-1]["positions_rad"], DEFAULT_GOAL, atol=1e-8)
    assert result["summary"]["tool_path_length_m"] > 0
    print("self-test: UR-10 TrajOpt demo produced 5 no-ROS waypoints")
    harmonious_start, harmonious_goal = preset_vectors("harmonious", HARMONIOUS_JOINT_NAMES)
    steamdeck_assets = Path(__file__).resolve().parents[3] / "urdf" / "v1" / "ui" / "public" / "robot" / "steamdeck_ur10"
    if steamdeck_assets.exists():
        seven_axis = run_demo(
            harmonious_start,
            harmonious_goal,
            5,
            HARMONIOUS_JOINT_NAMES,
            steamdeck_assets / "planning.urdf",
            steamdeck_assets / "robot.srdf",
            "SteamDeck UR-10 + turntable",
            "steamdeck_ur10",
            "harmonious",
            "base_link",
        )
        assert seven_axis["summary"]["moving_joint_count"] == 7
        print("self-test: TrajOpt accepted a 7-axis turntable+UR-10 harmonious plan")
        dense = run_demo(
            harmonious_start,
            harmonious_goal,
            5,
            HARMONIOUS_JOINT_NAMES,
            steamdeck_assets / "planning.urdf",
            steamdeck_assets / "robot.srdf",
            "SteamDeck UR-10 + turntable",
            "steamdeck_ur10",
            "harmonious",
            "base_link",
            "dense-csp",
            0.5,
            DEFAULT_TCP_SPEED_M_S,
            9,
            9,
        )
        assert len(dense["trajectory"]) > len(seven_axis["trajectory"])
        assert "velocities_rad_s" in dense["trajectory"][0]
        print("self-test: dense CSP example produced timed position/velocity rows")
    rosie_assets = Path(__file__).resolve().parents[3] / "robot_description" / "robots" / "rosie_1400_v3"
    if rosie_assets.exists():
        rosie_joint_names = ROSIE_V3_JOINT_NAMES
        rosie_start = np.zeros(len(rosie_joint_names), dtype=float)
        rosie_goal = np.asarray([0.18, -0.9, 1.25, -0.25, 0.95, 0.2, 0.0, 0.0, 0.0], dtype=float)
        rosie = run_demo(
            rosie_start,
            rosie_goal,
            5,
            rosie_joint_names,
            rosie_assets / "robot.urdf",
            rosie_assets / "robot.srdf",
            "rosie_1400_v3",
            "rosie_1400_v3",
            "harmonious",
            "world",
            "dense-csp",
            0.5,
            DEFAULT_TCP_SPEED_M_S,
            9,
            9,
        )
        assert rosie["source_urdf"].endswith("robot_description/robots/rosie_1400_v3/robot.urdf")
        assert rosie["source_srdf"].endswith("robot_description/robots/rosie_1400_v3/robot.srdf")
        assert rosie["collision_cost_enabled"] is True
        assert rosie["collision_constraint_enabled"] is True
        assert rosie["collision_prevention_model"] == "rosie_1400_v3_mesh_urdf"
        assert rosie["summary"]["moving_joint_count"] >= 6
        assert rosie["summary"]["tool_path_length_m"] > 0
        assert rosie["summary"]["max_tracking_error_m"] < 0.25
        print("self-test: rosie_1400_v3 mesh URDF initialized and produced a mesh-collision dense CSP path")
    return 0


def main(argv: list[str]) -> int:
    if len(argv) == 1 and argv[0] == "--self-test":
        return self_test()
    if not argv or argv[0] in {"help", "-h", "--help"}:
        print("usage: ur10_demo.py demo [--path name] [--trajectory-method method] [--joint-names names] [--start q...] [--goal q...] [--steps n] [--square-center x,y,z] [--json [file]] [--out file]")
        print("       ur10_demo.py --self-test")
        return 0
    command, rest = argv[0], argv[1:]
    if command == "demo":
        return command_demo(rest)
    print(f"unknown command {command!r}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:
        print(f"ur10_demo.py: {exc}", file=sys.stderr)
        raise SystemExit(1)
