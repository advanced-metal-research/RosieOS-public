#!/usr/bin/env python
"""Turn a saved `.weldproj` into the `.weldplan` plan request the planner eats.

`make_motion_fixtures.py` builds plan requests from a bare STEP and has to invent
the two things a STEP cannot know: which seams an operator wants welded, and
where the part sits on the table. It invents them badly on purpose --
`default_placement` drops the part at the work frame's origin, which on
`rosie_1400_v3` is inside the H-frame -- and its own comment says a real
placement is a human judgement made by dragging in the UI.

A `.weldproj` is that judgement, saved. It carries the STEP, the operator's
weld list with every override they dialled in, and a `cell.json` that IS the
placement. So this script runs the same authoring pipeline and then uses the
project's answers instead of defaults, which is the only difference between
the two files and the whole reason this one exists.

    pixi run -e default python tools/project_to_plan_request.py part.weldproj -o out.weldplan

Analysis is re-run rather than trusted from the bundle, because the project
format deliberately stores no derived geometry -- its own manifest says
opening recomputes, so that what is shown cannot disagree with the pipeline
that showed it. The same guarantee is worth having here.
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import zipfile
from pathlib import Path
from typing import Any

from olp_context import compiled_context
from seam_worker import paths
from seam_worker.cad.topology import extract_topology
from seam_worker.program import build_program
from seam_worker.reach.access import annotate
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.seams import default_seams
from seam_worker.store.settings import resolve
from weldplan.plan_request import write_request
from weldplan.urdf import descriptor_of


def read_project(path: Path, workdir: Path) -> dict[str, Any]:
    """`{manifest, welds, placement, step_path, settings}` from the bundle."""
    with zipfile.ZipFile(path) as bundle:
        names = set(bundle.namelist())
        for required in ("manifest.json", "welds.json", "cell.json"):
            if required not in names:
                raise SystemExit(f"{path.name} carries no {required}; is it a .weldproj?")
        manifest = json.loads(bundle.read("manifest.json"))
        welds = json.loads(bundle.read("welds.json"))
        # `cell.json` in a project is the PLACEMENT -- anchor, offset, rpy and
        # the cell it was authored against. The name is the authoring side's;
        # the planner calls the same document a placement.
        placement = json.loads(bundle.read("cell.json"))
        step_name = next(n for n in names if n.startswith("source/") and not n.endswith("/"))
        step_path = workdir / Path(step_name).name
        step_path.write_bytes(bundle.read(step_name))
    return {"manifest": manifest, "welds": welds, "placement": placement,
            "step": step_path,
            "settings": (manifest.get("settings") or {}).get("defaults")}


def build(project: dict[str, Any], *, request_id: str, created_utc: str) -> bytes:
    placement = project["placement"]
    cell_id = str(placement.get("cell_id") or "")
    if not cell_id:
        raise SystemExit("the project's placement names no cell; it cannot be planned")

    step = project["step"]
    topology = extract_topology(step)
    joints = annotate(step, topology, detect_joints(topology))
    joints["seams"] = default_seams(joints)["seams"]

    # The operator's OWN weld list, overrides and all -- not a default
    # selection of every seam the detector found. That list is the difference
    # between "what could be welded" and "what this program welds".
    program = build_program(
        joints, topology,
        program_id=str(project["manifest"].get("project_id") or step.stem),
        label=project["manifest"].get("label"),
        source_filename=step.name,
        welds=project["welds"],
        created_utc=created_utc,
    )

    # The robot as the OLP compiles it (docs/PLANNING-CONTEXT.md), so a
    # request built here is the request the OLP would have built.
    context = compiled_context(cell_id, root=paths.CELL_ROOT)

    return write_request(
        request_id=request_id,
        program=program,
        cell=descriptor_of(context),
        step_bytes=step.read_bytes(),
        step_filename=step.name,
        tooling=resolve(None).get("torch"),
        label=project["manifest"].get("label"),
        created_utc=created_utc,
        context=context,
    )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("project", type=Path)
    parser.add_argument("-o", "--out", type=Path, default=None)
    parser.add_argument("--created-utc", default="2026-01-01T00:00:00Z",
                        help="fixed by default, so the bytes are")
    parser.add_argument("--request-id", default=None,
                        help="defaults to the output stem; it travels IN the "
                             "plan request, so two names give two byte streams")
    args = parser.parse_args(argv)

    out = args.out or paths.DATA_DIR / "motion" / f"{args.project.stem}.weldplan"
    out.parent.mkdir(parents=True, exist_ok=True)

    # The STEP is unpacked to a temporary directory and not beside the output.
    # It is already inside the plan request this writes; a second copy in the data
    # directory would be an untracked file that looks like a fixture.
    with tempfile.TemporaryDirectory() as scratch:
        project = read_project(args.project, Path(scratch))
        payload = build(project, request_id=args.request_id or out.stem,
                        created_utc=args.created_utc)
    out.write_bytes(payload)

    placement = project["placement"]
    print(f"{out}  ({len(payload) / 1024:.0f} KiB)")
    print(f"  cell      {placement.get('cell_id')}")
    print(f"  offset    {[round(v, 4) for v in placement.get('offset_xyz_m') or []]} m")
    print(f"  rpy       {[round(v, 4) for v in placement.get('rpy_rad') or []]} rad")
    print(f"  welds     {len(project['welds'])} authored")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
