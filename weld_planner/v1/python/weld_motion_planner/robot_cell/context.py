#!/usr/bin/env python
"""The planning context: the robot a plan is for, as the OLP compiled it.

`docs/PLANNING-CONTEXT.md`. The OLP reads the description and the cell's
`machine_planning_calibration.json`, judges the deviation, composes it ONCE -- corrected origins and
narrowed limits into `robot.urdf`, the rate policy, holds, reset pose, frames
and torch into `planner.json` -- and packs the result under `context/` in the
plan request, with the sphere model and the meshes beside it. This module is
the reader. It parses `planner.json` into the profile every stage already
reads, hands the sphere model and the torch to `RobotCell`, and writes the
URDF and meshes to a directory for the readers that need a path (cuRobo, the
cell obstacle meshes).

## What this replaces

Four lookups by MODEL NAME on the planner's own disk: `_find_urdf` globbing
`robot_description/` for a `robot.urdf` under a directory called after the
cell, `profile.load` reading its `config.json`, `spheres.load` its
`spheres.json`, `tool.require` its torch. A name is not an identity; the
container inlines its descriptor precisely so a cell is never resolved by
name, and then the planner did it anyway one directory down. With a context
there is nothing to resolve: what arrived is what the plan is for, and the
header's `robot_cell` block says which pair it was compiled from.

## Found by identity, not threaded

Every stage takes the descriptor and asks `RobotCell.of(descriptor)`; the
context is looked up from the descriptor's `robot_cell` identity -- the same
pair the header carries and the daemon fences on -- rather than passed down
through forty signatures. The key also binds any native execution-limit
snapshot, so the same robot on two machines retains each machine's bounds.
The registry is bounded and lives
for the process: it is a cache of what a request carried, never a store.

A descriptor with no registered context is a request from before the context,
and every reader refuses it: there is no description on this machine to fall
back to by name, and that was the point.
"""

from __future__ import annotations

import json
import tempfile
from collections import OrderedDict
from pathlib import Path
from typing import Any

SCHEMA = "rosie.planning-context.v1"

# Members every context has; everything else is the description's registered
# files under their own paths (`spheres.json`, `meshes/...`, `robot.srdf`).
URDF = "robot.urdf"
PLANNER = "planner.json"
IDENTITY = "identity.json"

_IDENTITY_KEYS = ("model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "machine_planning_calibration_source")

# The machine configuration is recorded, not fenced on. The optional native
# execution-limit digest distinguishes planning contexts for that same robot.
_OPTIONAL_IDENTITY_KEYS = ("machine_configuration_sha256", "execution_limits_sha256")

# Contexts kept, by identity. One plan needs one; a server that alternates
# between two cells keeps both warm; more than four means something is
# leaking, not that a fifth cell arrived.
_KEEP = 4


class ContextError(ValueError):
    """A context this refuses to read, with the reason."""


class PlanningContext:
    """One compiled robot: its members, its numbers, and a directory on demand."""

    __slots__ = ("_dir", "files", "identity", "planner")

    def __init__(self, files: dict[str, bytes]) -> None:
        for required in (URDF, PLANNER, IDENTITY):
            if required not in files:
                raise ContextError(f"the planning context has no {required}")
        self.files = dict(files)
        try:
            planner = json.loads(files[PLANNER])
            identity = json.loads(files[IDENTITY])
        except ValueError as error:
            raise ContextError(f"the planning context's numbers are not JSON: {error}") from error
        if not isinstance(planner, dict) or planner.get("schema") != SCHEMA:
            raise ContextError(
                f"planner.json schema {planner.get('schema') if isinstance(planner, dict) else planner!r} "
                f"is not {SCHEMA!r}")
        if not isinstance(identity, dict):
            raise ContextError("identity.json must be an object")
        for key in _IDENTITY_KEYS:
            if not isinstance(identity.get(key), str) or not identity[key]:
                raise ContextError(f"identity.json.{key} must be a non-empty string")
        stamped = planner.get("identity")
        if not isinstance(stamped, dict) or any(stamped.get(k) != identity[k] for k in _IDENTITY_KEYS):
            raise ContextError("planner.json and identity.json name different pairs")
        execution = identity.get("execution_limits_sha256")
        if execution is not None and (not isinstance(execution, str) or
                not execution.startswith("sha256:") or len(execution) != 71 or
                any(c not in "0123456789abcdef" for c in execution[7:])):
            raise ContextError("identity.json.execution_limits_sha256 must be a SHA256 digest")
        if planner.get("execution_limits_sha256") != execution:
            raise ContextError("planner.json and identity.json name different execution limits")
        self.identity: dict[str, str] = {key: str(identity[key]) for key in _IDENTITY_KEYS}
        for key in _OPTIONAL_IDENTITY_KEYS:
            if isinstance(identity.get(key), str) and identity[key]:
                self.identity[key] = identity[key]
        self.planner: dict[str, Any] = planner
        self._dir: tempfile.TemporaryDirectory | None = None

    # ---- what the stages read

    @property
    def model_id(self) -> str:
        return self.identity["model_id"]

    @property
    def key(self) -> str:
        return key_of(self.identity)

    def profile(self, cell: dict[str, Any] | None = None) -> Any:
        """The merged profile, exactly the shape `profile.for_cell` produced.

        A hold may only name a positioner joint -- the arm solver has six
        unknowns -- and the OLP already refused a cell that holds an arm
        joint; with a descriptor to check against it is confirmed here, since
        a lattice built over a held arm axis fails three stages later.
        """
        from weld_motion_planner.robot_cell.robot.cell import CellError, axis_roles
        from weld_motion_planner.robot_cell.robot.profile import CellProfile

        held = {str(k): float(v) for k, v in (self.planner.get("held") or {}).items()}
        driven = [str(axis) for axis in (self.planner.get("driven") or [])]
        if cell is not None and held:
            arm = set(axis_roles(cell)["arm"])
            for axis in held:
                if axis in arm:
                    raise CellError(
                        f"the cell holds {axis!r}, which is not a positioner joint of "
                        f"{self.model_id!r}; only the positioner may be held")
        limits = {
            str(axis): {
                "velocity_rad_s": float(spec["velocity_rad_s"]),
                "acceleration_rad_s2": float(spec["acceleration_rad_s2"]),
                "enforced_velocity_rad_s": float(spec["enforced_velocity_rad_s"]),
            }
            for axis, spec in (self.planner.get("limits") or {}).items()
        }
        return CellProfile(
            cell_id=self.model_id,
            driven=driven,
            held=held,
            limits=limits,
            reset_pose={str(k): float(v) for k, v in (self.planner.get("reset_pose") or {}).items()},
            path=None,
        )

    def spheres(self) -> Any:
        """The sphere model, or None when the description has none."""
        from weld_motion_planner.robot_cell.collision.spheres import SphereModel

        data = self.files.get("spheres.json")
        if data is None:
            return None
        return SphereModel.from_json(json.loads(data), None)

    def tool(self) -> Any:
        """The torch, or a refusal naming the description that lacks one."""
        from weld_motion_planner.robot_cell.robot.tool import ToolError, ToolFrame

        torch = self.planner.get("torch")
        if not isinstance(torch, dict):
            raise ToolError(
                f"cell {self.model_id!r} has no torch in its description, so which way its "
                "electrode points is unknown. Write the `torch` section of its config.json.")
        return ToolFrame.from_json(torch, None, self.model_id)

    # ---- what needs a path

    def materialize(self) -> Path:
        """The members on disk, once: `robot.urdf` beside `meshes/`, the layout
        cuRobo's `asset_root_path` and the cell obstacle loader both expect.
        Deleted with the context; nothing else may keep the path."""
        if self._dir is None:
            self._dir = tempfile.TemporaryDirectory(prefix="planning-context-")
            root = Path(self._dir.name)
            for path, data in self.files.items():
                target = root / path
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
        return Path(self._dir.name)

    def urdf_path(self) -> Path:
        return self.materialize() / URDF

    def mesh_root(self) -> Path:
        return self.materialize() / "meshes"

    def close(self) -> None:
        if self._dir is not None:
            self._dir.cleanup()
            self._dir = None


def key_of(identity: dict[str, Any]) -> str:
    key = "|".join(str(identity.get(field) or "") for field in _IDENTITY_KEYS)
    # The same robot and calibration may run on machines with different native
    # envelopes. Keep those contexts separate while preserving old offline keys.
    execution = identity.get("execution_limits_sha256")
    return f"{key}|execution:{execution}" if execution else key


_REGISTRY: OrderedDict[str, PlanningContext] = OrderedDict()


def register(context: PlanningContext) -> PlanningContext:
    """Make `context` the one `of()` answers with for its identity."""
    key = context.key
    old = _REGISTRY.pop(key, None)
    if old is not None and old is not context:
        old.close()
    _REGISTRY[key] = context
    while len(_REGISTRY) > _KEEP:
        _, evicted = _REGISTRY.popitem(last=False)
        evicted.close()
    return context


def of(cell: dict[str, Any] | None) -> PlanningContext | None:
    """The context registered for this descriptor's identity, or None."""
    if not isinstance(cell, dict):
        return None
    robot = cell.get("robot_cell")
    if not isinstance(robot, dict):
        return None
    found = _REGISTRY.get(key_of(robot))
    if found is not None:
        _REGISTRY.move_to_end(key_of(robot))
    return found


def from_members(files: dict[str, bytes] | None) -> PlanningContext | None:
    """A context from what `read_request` returned, registered; None when the
    request carried none."""
    if not files:
        return None
    return register(PlanningContext(files))


def mesh_root_for(cell: dict[str, Any]) -> Path:
    """Where THIS cell's meshes are: the context's. A descriptor with no
    context has none to screen against, and says so."""
    found = of(cell)
    if found is None:
        raise ContextError(
            f"cell {cell.get('id')!r} arrived without a planning context, so its meshes are "
            "unknown; the request predates docs/PLANNING-CONTEXT.md -- re-pack it")
    return found.mesh_root()
