#!/usr/bin/env python
"""The robot description: one directory, one robot.

`docs/ROBOT-DESCRIPTION-CONTRACT.md`. Everything intrinsic to a robot MODEL
lives in `robot_description/<model>/`: `robot.urdf` (kinematics, meshes, design
limits), `meshes/`, `spheres.json` (the collision model, with the hashes it was
fitted from) and `config.json` (what the URDF cannot hold -- acceleration, axis
roles, the reset pose, the planner's rate policy, the torch convention, and the
caps a cell's kinematic corrections must stay inside).

Its identity is not a hash of the directory but of the files
`robot_description_manifest.json` REGISTERS (`weldplan.robot_description.robot_description_sha256`),
so the CAD exports, scripts and reports that also sit there are provenance and
not part of the robot. Nothing here depends on where the directory is; this
module is the one place that says.

## Why the planner reads it rather than keeping its own copy

It used to keep three: `<model>.profile.json`, `<model>.tool.json` and
`<model>.spheres.json` under `data/motion/cells/`, beside the planner. The
proposal's whole case is that a robot with two descriptions has no
description -- the cell's URDF and the workstation's had already drifted -- and
the same argument applies inside this repo. So the assets moved next to the
URDF they describe, and what the planner keeps is a READER.

## Model from disk, cell from the request

What is deliberately NOT here: how a particular cell deviates from the model.
That is `machine_planning_calibration.json`; it lives on the cell, the OLP reads it and judges it
against this description, and the settled numbers arrive inside the plan
request (`descriptor.robot`). So there is exactly one reader per kind of fact:
static model data is read from git by whoever needs it -- this module, the
motion server, the tools -- and a cell's own settings are read once, by the
OLP, from the cell.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

from weldplan.robot_description import CellDocumentError, RobotConfig, parse_config

# Beside the components, not inside this one: the same directory the OLP, the
# simulators and the joint daemon read, so there is one description per robot.
DESCRIPTION_ROOT = Path(__file__).resolve().parents[6] / "robot_description" / "robots"


def path(model_id: str, root: Path | None = None) -> Path:
    """The description directory for a model."""
    return (root or DESCRIPTION_ROOT) / str(model_id)


def urdf_path(model_id: str, root: Path | None = None) -> Path:
    return path(model_id, root) / "robot.urdf"


def mesh_root(model_id: str, root: Path | None = None) -> Path:
    """Where THIS model's link meshes are.

    This was one module constant naming `rosie_1400_v3`, and every screen and
    every verifier read the 1400's meshes whatever cell it was planning. On
    `rosie_1420_v1` that is not a near miss: `cell_obstacles` looks each link up
    by name, the 1420's `pedestal` and `base` are not in that folder and its
    `link_3` is a different body, so the arm was screened against a table it
    does not stand at and a forearm it does not have.
    """
    return path(model_id, root) / "meshes"


def spheres_path(model_id: str, root: Path | None = None) -> Path:
    return path(model_id, root) / "spheres.json"


def config_path(model_id: str, root: Path | None = None) -> Path:
    return path(model_id, root) / "config.json"


def urdf_velocities(model_id: str, root: Path | None = None) -> dict[str, float]:
    """Each revolute joint's URDF velocity RATING, which is the ceiling a
    planning rate and a cell's enforced rate are both checked against.

    Read from the URDF rather than restated anywhere, which is the rule that
    makes `config.json` a set of overrides instead of a second description.
    """
    source = urdf_path(model_id, root)
    if not source.is_file():
        return {}
    out: dict[str, float] = {}
    for joint in ET.parse(source).getroot().findall("joint"):
        limit = joint.find("limit")
        name = joint.get("name")
        if not name or joint.get("type") != "revolute" or limit is None:
            continue
        rating = limit.get("velocity")
        if rating:
            out[name] = float(rating)
    return out


def config(model_id: str, root: Path | None = None) -> RobotConfig | None:
    """The model's `config.json`, validated against its URDF, or None when the
    model has none (an early model is a state, not an error -- `profile.problems`
    says what is missing and which milestone needs it)."""
    source = config_path(model_id, root)
    if not source.is_file():
        return None
    return _config_of(source, str(model_id))


@lru_cache(maxsize=8)
def _cached(source: str, stamp: tuple[float, int], model_id: str) -> RobotConfig:
    document = json.loads(Path(source).read_text(encoding="utf-8"))
    return parse_config(document, model_id=model_id,
                        velocities=urdf_velocities(model_id, Path(source).parent.parent))


def _config_of(source: Path, model_id: str) -> RobotConfig:
    # Keyed by mtime and size so an edited file is re-read: these are read on
    # every plan, and a stale cap or rate is exactly what the contract exists
    # to stop.
    stat = source.stat()
    try:
        return _cached(str(source), (stat.st_mtime, stat.st_size), model_id)
    except CellDocumentError as error:
        raise CellDocumentError(f"{source}: {error}") from error
