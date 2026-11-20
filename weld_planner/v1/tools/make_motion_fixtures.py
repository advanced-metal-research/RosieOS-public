#!/usr/bin/env python
"""Build the `.weldplan` plan requests the motion planner's tests read.

Lives in `tools/` and not in either package, because it is the one program
that legitimately stands on both sides of the boundary: it imports
`seam_worker` to run the authoring pipeline, and it writes files that
`weld_motion_planner` consumes. Putting it inside `weld_motion_planner` would
have made the planner import the authoring side; putting it inside
`seam_worker` would have made the authoring side carry a dependency on the
planner's fixture needs. A script that is neither is the honest description.

Run it when the authoring pipeline changes what it emits, not on every test
run: the output is COMMITTED, so a planner test failing after an authoring
change is a diff a human reads rather than a mystery that appears in CI.
Plan requests are deterministic bytes by contract, so re-running with no
change upstream rewrites the same file and `git status` stays clean.

## What this replaced, and why

The committed corpus was `amr-weld-planner-v1.weld-program.v1` and had been
since before program.v2 existed. `PlanRequest.seams` reads weld NODES, so
every one of those files answered with zero seams and zero freespace nodes --
and the tests parametrized over them ran, passed, and proved nothing about a
weld. That is how the planner came to refuse every real weld for authoring no
standoff while the suite stayed green.

The old generator could not have fixed it: it imported
`seam_worker.program.build_program`, which went with the authoring server in
D24, so it had not been runnable for some time either.

## The placement is the part that cannot be generated

`default_placement` is the anchor plus a ZERO offset and rpy, which puts the
part at the work frame's ORIGIN. On `rosie_1400_v3` that origin is inside the
H-frame: the part reads -41.6 mm against it at every positioner angle, and
the feasibility mask correctly comes back empty. Nothing downstream of M2 can
be exercised from a generated placement.

So the placements are DATA here, carried in `PLACEMENTS` below. They were
measured by a human dragging each part onto the table in the authoring UI,
they are the only reason these fixtures reach the seam search at all, and
they were rescued out of the v1 corpus before it was replaced.

## The seams are deliberately not all default

A program whose every seam has no weave and no admissible band would let the
weave expansion and the rescue pass be "tested" against data that never
triggers them. So one seam per fixture is given real weave parameters and a
deliberately tight band -- authored intent is exactly what those fields are,
and an operator would have set them in the UI.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path
from typing import Any

from olp_context import compiled_context
from seam_worker import paths
from seam_worker.cad.topology import extract_topology
from seam_worker.reach.access import annotate
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.seams import DEFAULT_STANDOFF_MM, default_seams
from weldplan.plan_request import EXTENSION, write_request
from weldplan.urdf import descriptor_of

OUT_DIR = paths.DATA_DIR / "motion"
# Where the corpus parts live: the same STEPs the OLP offers as examples, so
# a fixture and what an operator opens are the same geometry.
EXAMPLES = paths.REPO_ROOT / "offline-programming" / "v1" / "ui" / ".staged-public" / "examples"

# Fixed so the bytes are, which is the property the format promises and the
# tests lean on. A wall clock here would make every rebuild a diff.
CREATED_UTC = "2026-01-01T00:00:00Z"

# Measured by hand in the authoring UI, rescued from the v1 corpus. See the
# module docstring: a generated placement cannot reach the seam search.
#
# Offsets are from the middle of the table's top face, the work frame the cell
# descriptor names (`seam_worker.robot.cell.WORK_SURFACES`) and the frame the
# OLP authors in. The v1 corpus stored them from J7's axis, and they are that
# number minus [0, -0.622, +0.100]: two of the three parts were simply centred
# on the table.
PLACEMENTS: dict[str, dict[str, Any]] = {
    "bracket_a_2x": {
        "anchor_xyz_m": [0.15, -0.09, -0.02],
        "offset_xyz_m": [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
    },
    "bracket_b_2x": {
        "anchor_xyz_m": [0.0, 0.0, 0.0],
        "offset_xyz_m": [0.0, 0.0037726178, 0.0],
        "rpy_rad": [2.3390285425286377e-16, 2.2172773506981978e-16, 1.5718609776386132],
    },
    "hitch_2x": {
        "anchor_xyz_m": [0.08000066070087375, 0.0, 0.0],
        "offset_xyz_m": [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
    },
}

# What an operator would have dialled in for a 3 mm fillet: light weave, and
# five degrees of work angle to trade.
WEAVE = {
    "shape": "sine",
    "amplitude_mm": 1.5,
    "frequency_hz": 3.0,
    "corner_policy": "ramp_out",
    "ramp_in_mm": 5.0,
    "ramp_out_mm": 5.0,
}
BAND = {
    "work_angle_deg": {
        "min": {"kind": "constant", "value": -5.0},
        "max": {"kind": "constant", "value": 5.0},
        "max_rate_deg_per_mm": 0.5,
    },
    "travel_angle_deg": {
        "min": {"kind": "constant", "value": 0.0},
        "max": {"kind": "constant", "value": 20.0},
        "max_rate_deg_per_mm": 0.5,
    },
}

# The gun these are measured with, and where it is held. `standoff_mm` is
# required on every weld node and never defaulted: zero puts the tip in the
# part. Where it is held is the cell's one default, so the corpus is planned at
# the standoff a newly detected seam is authored at.
TOOLING = {
    "id": "mig_gas_nozzle_16",
    "label": "MIG, Ø16 gas nozzle",
    "body_radius_mm": 8.0,
    "body_length_mm": 150.0,
    "default_standoff_mm": DEFAULT_STANDOFF_MM,
}
STANDOFF_MM = DEFAULT_STANDOFF_MM
TRAVEL_SPEED_MM_S = 4.0

PLANNER_OWNS = [
    "positioner joint values -- prescribed by the planner, not by this document",
    "collision meshes for the CELL -- the cell is named and hashed here, its "
    "descriptor is the planner's asset",
    "torch orientation within the admissible bands each seam-geometry weld declares",
    "sampling density and where waypoints land on a weld",
    "the joint-space solution of every freespace node -- this document states "
    "constraints, never a path",
    "time parametrization and dynamics",
]

SEAM_KEYS = (
    "id", "label", "weld_joint_ref", "origin", "pass", "curve", "span",
    "direction", "reference", "orientation", "weld_geometry", "lead",
    "tolerance", "sampling_hint",
)


def _node(op: str, nid: str, comment: str, **own: Any) -> dict[str, Any]:
    return {
        "id": nid,
        "op": op,
        "enabled": True,
        "label": nid,
        "comment": comment,
        "row_hash": f"pending:{nid}",
        **own,
    }


def _weld_node(seam: dict[str, Any], joint: dict[str, Any], decorate: bool) -> dict[str, Any]:
    geometry = {"kind": "seam", **{k: seam[k] for k in SEAM_KEYS if k in seam}}
    # A default seam names its curve rather than carrying one: `default_seams`
    # emits `{"from_weld_joint": true}` and leaves resolving it to whoever
    # accepts the seam (the OLP does this in `rails-node.ts`). Resolve it here
    # too, or the document loses the curve's `closed` flag and every span that
    # wraps through s=0 -- legitimate on a joint loop -- is refused as
    # wrapping on an open curve.
    curve = geometry.get("curve")
    if not isinstance(curve, dict) or curve.get("from_weld_joint"):
        geometry["curve"] = joint["weld_line"]
    if decorate:
        geometry.setdefault("orientation", {})["limits"] = {
            axis: dict(spec) for axis, spec in BAND.items()
        }
    return _node(
        "weld",
        f"weld_{seam['id']}",
        "one pass along the detected seam",
        geometry=geometry,
        travel_speed_mm_s={"kind": "constant", "value": TRAVEL_SPEED_MM_S},
        standoff_mm={"kind": "constant", "value": STANDOFF_MM},
        weave=dict(WEAVE) if decorate else {"shape": "none"},
        weld_parameters={},
    )


def _node_line(seams: list[dict[str, Any]],
               joints_by_id: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    """`home approach weld (transit weld)* retract home`.

    The alternation IS the validity test (D23), and D25 took the last of the
    inventing away from the planner: a gap here is a document problem, not a
    hole for M6 to fill. So every move between the welds is stated.
    """
    nodes: list[dict[str, Any]] = [_node("home", "home_start", "leave the cell's reset pose")]
    for index, seam in enumerate(seams):
        if index == 0:
            nodes.append(_node("approach", f"approach_{seam['id']}", "arrive at the first weld"))
        else:
            nodes.append(_node("transit", f"transit_{seam['id']}", "cross to the next weld"))
        joint = joints_by_id.get(seam.get("weld_joint_ref"))
        if joint is None:
            raise SystemExit(f"seam {seam['id']!r} names a joint the document does not carry")
        nodes.append(_weld_node(seam, joint, decorate=index == 0))
    nodes.append(_node("retract", "retract_end", "leave the last weld"))
    nodes.append(_node("home", "home_end", "return to the cell's reset pose"))
    return nodes


def build(step: Path, *, cell_id: str, label: str) -> bytes:
    placement = PLACEMENTS.get(step.stem)
    if placement is None:
        raise SystemExit(
            f"no measured placement for {step.stem!r}. A generated one puts the part at "
            "the work frame origin, which on rosie_1400_v3 is inside the H-frame and "
            "makes the feasibility mask empty -- place it in the authoring UI and add "
            "the numbers to PLACEMENTS."
        )

    topology = extract_topology(step)
    joints = annotate(step, topology, detect_joints(topology))
    seams = default_seams(joints)["seams"]
    if not seams:
        raise SystemExit(f"{step.name} produced no seams to weld")

    # The robot as the OLP compiles it (docs/PLANNING-CONTEXT.md): the one
    # compiler, so the fixture carries the context a real plan request does.
    context = compiled_context(cell_id, root=paths.CELL_ROOT)
    cell = descriptor_of(context)

    program = {
        "schema": "robot.v4.program.v2",
        "program_id": f"{step.stem}_fixture",
        "program_name": label,
        "program_revision": 1,
        "base_program_revision": 0,
        "program_state": "saved_unplanned",
        "execution_mode": "simulation",
        "created_utc": CREATED_UTC,
        "units": {"length": "m", "angle": "rad"},
        "producer": {"tool": "tools/make_motion_fixtures.py", "version": "0.1.0"},
        "source": {
            "kind": "step",
            "path": str(step),
            "sha256": hashlib.sha256(step.read_bytes()).hexdigest(),
            "unit_scale_to_m": 0.001,
            "importer": "amr-weld-planner topology.v1 + detect.v1",
        },
        "workpiece": {
            "frame_id": "workpiece",
            "weld_joints": joints.get("weld_joints") or [],
        },
        "placement": {"cell_id": cell_id, **placement},
        "cell": {"id": cell_id, "sha256": "0" * 64},
        "tool": None,
        "defaults": {
            "clearance_min_mm": 20.0,
            "speed_scale": 1.0,
            "torch_policy": "free",
            "positioner_policy": "free",
        },
        "nodes": _node_line(seams, {j["id"]: j for j in joints.get("weld_joints") or []}),
        "planner_owns": list(PLANNER_OWNS),
        "extensions": {},
    }

    return write_request(
        request_id=f"{step.stem}_fixture",
        program=program,
        cell=cell,
        step_bytes=step.read_bytes(),
        step_filename=step.name,
        tooling=dict(TOOLING),
        label=label,
        created_utc=CREATED_UTC,
        context=context,
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("steps", nargs="*", type=Path, help="STEP files; default is the corpus")
    parser.add_argument("--cell", default="rosie_1400_v3")
    parser.add_argument("--out", type=Path, default=OUT_DIR)
    args = parser.parse_args(argv)

    steps = args.steps or [EXAMPLES / f"{stem}.step" for stem in sorted(PLACEMENTS)]
    args.out.mkdir(parents=True, exist_ok=True)
    for step in steps:
        if not step.is_file():
            print(f"missing {step}", file=sys.stderr)
            return 1
        payload = build(step, cell_id=args.cell, label=step.stem.replace("_", " "))
        target = args.out / f"{step.stem}{EXTENSION}"
        target.write_bytes(payload)
        print(f"{target} ({len(payload)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
