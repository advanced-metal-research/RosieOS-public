#!/usr/bin/env python
"""Scale a bundled STEP fixture uniformly about the origin and write a new STEP.

Why a tool and not a hand-edited file: the `_2x` fixtures in `data/fixtures/`
are DERIVED from their originals, and a derived file with no recorded recipe is
a file nobody can correct. When `bracket_a.step` is re-exported from CAD,
`bracket_a_2x.step` is stale and there is no way to tell by looking at it. Run
this and the pair is in step again.

    python tools/scale_fixture.py --step data/fixtures/bracket_a.step --factor 2

The output name is the input stem plus the factor -- `bracket_a` at 2 becomes
`bracket_a_2x` -- written beside the input unless `--out` says otherwise.

## Scaling about the origin, not about the centre

`gp_Trsf.SetScaleFactor` scales about the world origin, so a part's distance
from the origin doubles along with the part. That is the wanted behaviour here:
these fixtures are authored in the workpiece frame the placement stage later
positions, and scaling about the centroid instead would have moved the part
relative to its own origin -- the same solid, in a different place, which reads
as a placement bug rather than as a bigger part.

Uniform scale keeps planes planar and cylinders cylindrical, so the topology
stage sees the same surface and curve types it saw before, with the same face
and edge counts. That is the check `--verify` runs.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any

# OCCT's `write.step.schema`; 4 is AP214 IS, which is what the hand-authored
# fixtures beside these were exported as.
_AP214_IS = 4

# The write stamps wall-clock time into the header, which would make every
# re-run a diff even when the geometry is byte-identical -- the same trap
# `make_motion_fixtures.py` avoids by emitting deterministic bytes. A fixed
# stamp says what is true of a derived file: its date is the date of its
# source, not of the last time somebody ran the tool.
_FIXED_STAMP = "1970-01-01T00:00:00"


def _read_step(path: Path) -> Any:
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader

    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError(f"OCCT could not read {path}")
    reader.TransferRoots()
    return reader.OneShape()


def _write_step(shape: Any, path: Path) -> None:
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer

    # Millimetres and AP214, matching what the hand-authored fixtures carry --
    # `topology.py` reads these files as millimetres by default and a unit
    # change here would silently shrink the part by a thousand.
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    Interface_Static.SetIVal_s("write.step.schema", _AP214_IS)
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise ValueError(f"OCCT could not write {path}")
    _freeze_header(path)


def _freeze_header(path: Path) -> None:
    """Replace the written wall-clock stamp with `_FIXED_STAMP`, in the header only."""
    text = path.read_text(encoding="utf-8")
    head, sep, body = text.partition("ENDSEC;")
    if not sep:
        raise ValueError(f"no header section in {path}")
    head = re.sub(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}", _FIXED_STAMP, head, count=1)
    path.write_text(head + sep + body, encoding="utf-8")


def _bbox_mm(shape: Any) -> tuple[float, ...]:
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return box.Get()


def scale_step(source: Path, target: Path, factor: float) -> dict[str, Any]:
    """Write `source` scaled by `factor` to `target`; return the two bounding boxes."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf

    if factor <= 0.0:
        raise ValueError(f"factor must be positive, got {factor}")

    shape = _read_step(source)
    trsf = gp_Trsf()
    trsf.SetScaleFactor(factor)
    scaled = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    _write_step(scaled, target)
    return {"before_mm": list(_bbox_mm(shape)), "after_mm": list(_bbox_mm(scaled))}


def _summary(path: Path) -> dict[str, Any]:
    from seam_worker.cad.topology import extract_topology

    return dict(extract_topology(path)["summary"])


def default_target(source: Path, factor: float) -> Path:
    tag = f"{factor:g}".replace(".", "p")
    return source.with_name(f"{source.stem}_{tag}x{source.suffix}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", required=True, help="STEP file to scale")
    parser.add_argument("--factor", type=float, default=2.0)
    parser.add_argument("--out", default="", help="output STEP (default: <stem>_<n>x.step)")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="re-extract topology from both files and require the summaries to match",
    )
    ns = parser.parse_args(argv)

    source = Path(ns.step)
    target = Path(ns.out) if ns.out else default_target(source, ns.factor)
    target.parent.mkdir(parents=True, exist_ok=True)
    boxes = scale_step(source, target, ns.factor)
    print(f"wrote {target}")
    print(f"  bbox mm before {['%.3f' % v for v in boxes['before_mm']]}")
    print(f"  bbox mm after  {['%.3f' % v for v in boxes['after_mm']]}")

    if ns.verify:
        before, after = _summary(source), _summary(target)
        if before != after:
            print(f"topology drifted:\n  before {before}\n  after  {after}", file=sys.stderr)
            return 1
        print(
            f"  topology unchanged: {after['solid_count']} solids, "
            f"{after['face_count']} faces, {after['edge_count']} edges"
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"scale_fixture: {exc}", file=sys.stderr)
        raise SystemExit(1)
