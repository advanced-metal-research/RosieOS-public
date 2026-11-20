#!/usr/bin/env python
"""The robot description's identity, and its `config.json`.

The contract is `docs/ROBOT-DESCRIPTION-CONTRACT.md`. A plan is made against a
robot description (a directory, identified by the files its `robot_description_manifest.json`
registers) and a cell's `machine_planning_calibration.json` (identified by the sha256 of its bytes),
and the joint daemon refuses a plan whose two hashes are not its own.

## Who decides what, and where

The OLP is the one place that READS a `machine_planning_calibration.json` and judges it, and since the
planning context (`docs/PLANNING-CONTEXT.md`) the one place that APPLIES it:
the corrected URDF and the merged numbers arrive compiled, and nothing on the
planning side composes a correction or narrows a limit. What this package owns
is the identity layout, because three languages compute it and they have to
agree byte for byte, and the reader for `config.json`, which the description's
tools and tests use to check a model on disk.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA = "rosie.machine-planning-calibration.v1"
CONFIG_SCHEMA = "rosie.robot-config.v1"
CELL_SOURCES = ("target", "empty")

# Numbers and names only. Prose belongs in the description's README: a note
# here would be hashed into the robot's identity, so a typo in it would refuse
# every plan made before the fix.
_CONFIG_KEYS = {"schema", "model_id", "frames", "kinematic_correction_caps", "planning",
                "axes", "reset_pose", "torch"}
_PLANNING_KEYS = {"speed_ceiling_rad_s", "joints"}
_PLANNING_JOINT_KEYS = {"velocity_rad_s", "acceleration_rad_s2"}
_AXES_KEYS = {"driven", "held"}
_CAP_KEYS = {"xyz_m", "rpy_rad"}
_TORCH_KEYS = {"tool_frame", "electrode_axis"}
_IDENTITY_KEYS = ("model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "machine_planning_calibration_source")
_HEX = frozenset("0123456789abcdef")


class CellDocumentError(ValueError):
    """A description document, or the settings handed to this package, is not
    shaped the way the contract says."""


@dataclass(frozen=True)
class RobotConfig:
    """`config.json`: what the URDF cannot hold about one robot model.

    Never a second copy of a URDF value. `planning` is the planner's own rate
    policy, bounded above by the URDF's rating; the cell's ENFORCED rate is a
    per-cell fact and lives in that cell's `machine_planning_calibration.json`.

    `kinematic_correction_caps` is checked for shape here and not carried:
    what it bounds is a cell's correction, and that judgement is the OLP's and
    the daemon's, on either side of this package.
    """

    model_id: str
    speed_ceiling_rad_s: float
    planning: dict[str, dict[str, float]]
    driven: list[str]
    held: dict[str, float]
    reset_pose: dict[str, float]
    torch: dict[str, Any]


def parse_config(document: dict[str, Any], *, model_id: str,
                 velocities: dict[str, float]) -> RobotConfig:
    """A validated `config.json`, or `CellDocumentError` naming the first rule
    it breaks.

    `velocities` is the URDF's own rating per revolute joint -- the ceiling a
    planning rate is checked against, and the set of joints this file may speak
    about at all. A planning rate above the rating would have the planner
    parametrize a motion the robot is not rated for, and a joint the URDF does
    not have is a typo that would otherwise silently do nothing.
    """
    if not isinstance(document, dict):
        raise CellDocumentError("config.json is not an object")
    _only(document, _CONFIG_KEYS, "config.json")
    if document.get("schema") != CONFIG_SCHEMA:
        raise CellDocumentError(
            f"config.json schema {document.get('schema')!r} is not {CONFIG_SCHEMA!r}")
    if document.get("model_id") != model_id:
        raise CellDocumentError(
            f"config.json is for model {document.get('model_id')!r}, the directory is {model_id!r}")

    caps_raw = document.get("kinematic_correction_caps")
    if caps_raw is None:
        raise CellDocumentError("config.json has no kinematic_correction_caps")
    _object(caps_raw, "kinematic_correction_caps")
    _only(caps_raw, _CAP_KEYS, "kinematic_correction_caps")
    _at_least_zero(caps_raw.get("xyz_m", 0.0), "kinematic_correction_caps.xyz_m")
    _at_least_zero(caps_raw.get("rpy_rad", 0.0), "kinematic_correction_caps.rpy_rad")

    planning = document.get("planning")
    if planning is None:
        raise CellDocumentError("config.json has no planning section")
    _object(planning, "planning")
    _only(planning, _PLANNING_KEYS, "planning")
    fastest = max(velocities.values(), default=0.0)
    ceiling = _positive(planning.get("speed_ceiling_rad_s", fastest), "planning.speed_ceiling_rad_s")
    if ceiling > fastest:
        raise CellDocumentError(
            f"planning.speed_ceiling_rad_s {ceiling} is above the URDF's fastest joint ({fastest})")
    joints = planning.get("joints")
    _object(joints if joints is not None else {}, "planning.joints")
    rates: dict[str, dict[str, float]] = {}
    for name, spec in (joints or {}).items():
        where = f"planning.joints.{name}"
        if name not in velocities:
            raise CellDocumentError(f"{where}: the URDF has no revolute joint {name!r}")
        _object(spec, where)
        _only(spec, _PLANNING_JOINT_KEYS, where)
        velocity = _positive(spec.get("velocity_rad_s"), f"{where}.velocity_rad_s")
        if velocity > velocities[name]:
            raise CellDocumentError(
                f"{where}.velocity_rad_s {velocity} is above the URDF's rating "
                f"({velocities[name]}); a description may lower the rate it plans at, never raise it")
        rates[str(name)] = {
            "velocity_rad_s": min(velocity, ceiling),
            "acceleration_rad_s2": _positive(spec.get("acceleration_rad_s2"),
                                             f"{where}.acceleration_rad_s2"),
        }

    axes = document.get("axes")
    if axes is None:
        raise CellDocumentError("config.json has no axes section")
    _object(axes, "axes")
    _only(axes, _AXES_KEYS, "axes")
    driven = [str(name) for name in (axes.get("driven") or [])]
    held = {str(name): _number(value, f"axes.held.{name}")
            for name, value in (axes.get("held") or {}).items()}
    for name in [*driven, *held]:
        if name not in velocities:
            raise CellDocumentError(f"axes: the URDF has no revolute joint {name!r}")
    both = sorted(set(driven) & set(held))
    if both:
        raise CellDocumentError(f"axes: {both[0]!r} is both driven and held; it can only be one")

    reset = document.get("reset_pose") or {}
    _object(reset, "reset_pose")
    reset_pose = {str(name): _number(value, f"reset_pose.{name}") for name, value in reset.items()}
    for name in reset_pose:
        if name not in velocities:
            raise CellDocumentError(f"reset_pose: the URDF has no revolute joint {name!r}")

    torch = document.get("torch")
    if torch is None:
        raise CellDocumentError("config.json has no torch section")
    _object(torch, "torch")
    _only(torch, _TORCH_KEYS, "torch")
    axis = torch.get("electrode_axis")
    if not (isinstance(axis, list) and len(axis) == 3):
        raise CellDocumentError("torch.electrode_axis must be three numbers")
    length = math.sqrt(sum(_number(v, "torch.electrode_axis") ** 2 for v in axis))
    if length < 1e-9:
        raise CellDocumentError("torch.electrode_axis has no direction")
    if not isinstance(torch.get("tool_frame"), str) or not torch["tool_frame"]:
        raise CellDocumentError("torch.tool_frame must name a link")

    return RobotConfig(model_id=model_id, speed_ceiling_rad_s=ceiling, planning=rates,
                       driven=driven, held=held, reset_pose=reset_pose, torch=dict(torch))


MANIFEST = "robot_description_manifest.json"
MANIFEST_SCHEMA = "rosie.robot-manifest.v1"
IDENTITY_DOMAIN = b"rosie.robot-description.identity.v1\x00"
# The cell serves its own deviation under this name beside the description's
# files, so a description may not register one: the two would collide in the
# served set and the adapter would refuse the whole set at start. The Go twin
# (`robotdesc.DescriptionFiles`) refuses the same name.
MACHINE_PLANNING_CALIBRATION = "machine_planning_calibration.json"


class DescriptionError(ValueError):
    """The description does not say what it is made of, or a file it registers
    is not there."""


def description_files(directory: Path) -> list[str]:
    """The paths `robot_description_manifest.json` registers, validated, without the manifest
    itself. Raises `DescriptionError` when the manifest is missing, malformed,
    or names something a description may not contain."""
    directory = Path(directory)
    path = directory / MANIFEST
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise DescriptionError(f"{path} is missing: a description says which files it is made of") from error
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DescriptionError(f"{path} is not JSON: {error}") from error
    if not isinstance(document, dict) or document.get("schema") != MANIFEST_SCHEMA:
        raise DescriptionError(f"{path} schema is not {MANIFEST_SCHEMA!r}")
    if document.get("model_id") != directory.name:
        raise DescriptionError(
            f"{path} is for model {document.get('model_id')!r}, the directory is {directory.name!r}")
    files = document.get("files")
    if not isinstance(files, list) or not files:
        raise DescriptionError(f"{path}: files must be a non-empty list")
    seen: set[str] = set()
    for entry in files:
        if not isinstance(entry, str) or not entry:
            raise DescriptionError(f"{path}: a registered path is not a string")
        if entry.startswith("/") or "\\" in entry or entry in seen or entry == MANIFEST:
            raise DescriptionError(f"{path}: {entry!r} is not a description-relative path")
        if entry == MACHINE_PLANNING_CALIBRATION:
            raise DescriptionError(
                f"{path}: {entry!r} is the cell's own file, served beside the description; "
                "a description may not register one")
        if any(part in ("", ".", "..") for part in entry.split("/")):
            raise DescriptionError(f"{path}: {entry!r} escapes the description")
        seen.add(entry)
        if not (directory / entry).is_file():
            raise DescriptionError(f"{path} registers {entry!r}, which is not a file here")
    return sorted(seen)


def robot_description_sha256(directory: Path) -> str:
    """The description's identity: sha256 over its manifest and every file the
    manifest names (`docs/ROBOT-DESCRIPTION-CONTRACT.md`).

    The manifest is hashed as an entry of its own, so dropping a file from it
    is itself a change of identity. A file nobody registered -- a CAD export, a
    report, a README -- is not part of the robot and does not make a plan
    stale, which is why this is a manifest and not a hash of the directory.

    Third implementation after the daemon's C++ and the OLP's Go, all three
    held to the same vectors.
    """
    directory = Path(directory)
    return hash_files(directory, [MANIFEST, *description_files(directory)])


def mesh_sha256(directory: Path) -> str:
    """The same hash over the registered meshes alone: what a sphere fit was
    made from, so `spheres.json` can carry it and a refit that never happened
    is a test failure rather than a surprise on the cell."""
    directory = Path(directory)
    meshes = [f for f in description_files(directory) if f.startswith("meshes/")]
    return hash_files(directory, meshes)


def hash_files(directory: Path, entries: list[str]) -> str:
    """The identity layout: the domain, the count, then each path and the
    sha256 of its bytes, in path order."""
    digest = hashlib.sha256()
    digest.update(IDENTITY_DOMAIN)
    digest.update(len(entries).to_bytes(8, "big"))
    for entry in sorted(entries):
        raw = entry.encode("utf-8")
        digest.update(len(raw).to_bytes(8, "big"))
        digest.update(raw)
        digest.update(hashlib.sha256((Path(directory) / entry).read_bytes()).digest())
    return "sha256:" + digest.hexdigest()


def empty_cell_bytes(model_id: str) -> bytes:
    """The document a cell with no `machine_planning_calibration.json` serves and hashes -- byte for
    byte what the daemon synthesises, so an offline plan and an unconfigured
    cell agree."""
    return ('{"schema":"rosie.machine-planning-calibration.v1","model_id":"' + model_id + '"}\n').encode("utf-8")


def machine_planning_calibration_sha256(data: bytes) -> str:
    return "sha256:" + hashlib.sha256(data).hexdigest()


def is_robot_description_sha256(value: Any) -> bool:
    return (isinstance(value, str) and value.startswith("sha256:") and len(value) == 71
            and set(value[7:]) <= _HEX)


def header_identity(descriptor: dict[str, Any]) -> dict[str, str] | None:
    """The dense header's `robot` block for a descriptor, or None when the
    descriptor was never stamped (a request from before the contract).

    Identity only: the settings beside it are the planner's business, and the
    daemon fences on the pair, not on what the pair permits."""
    robot = descriptor.get("robot_cell")
    if not isinstance(robot, dict):
        return None
    block = {key: str(robot[key]) for key in ("model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "machine_planning_calibration_source")}
    # The machine the plan was made for, when there was one: a record the
    # cell never refuses over (it moves for changes no plan depends on).
    recorded = robot.get("machine_configuration_sha256")
    if isinstance(recorded, str) and recorded:
        block["machine_configuration_sha256"] = recorded
    return block


def _only(value: dict[str, Any], allowed: set[str], where: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise CellDocumentError(f"{where} has unknown key {unknown[0]!r}")


def _object(value: Any, where: str) -> None:
    if not isinstance(value, dict):
        raise CellDocumentError(f"{where} must be an object")


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CellDocumentError(f"{where} must be a finite number")
    return float(value)


def _at_least_zero(value: Any, where: str) -> float:
    number = _number(value, where)
    if number < 0:
        raise CellDocumentError(f"{where} must not be negative")
    return number


def _positive(value: Any, where: str) -> float:
    number = _number(value, where)
    if number <= 0:
        raise CellDocumentError(f"{where} must be positive")
    return number
