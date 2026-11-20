#!/usr/bin/env python
"""The robot's collision model: spheres on the moving body.

`ARCHITECTURE.md` splits collision by what a body IS during a weld. Spheres
are for the body being checked -- the arm chain and the torch -- because a
sphere is the one shape whose distance to a field is exact in closed form:
`d = f(centre) - r`, one query. Everything else in the cell is rigid within
some frame once the held axes are held, and is a distance field instead.

This module is the reader for that asset. It lives in the robot's own
description -- `robot_description/<model>/spheres.json`, beside the meshes it was fitted
to -- and is authored by `tools/sphere_tool` or `tools/fit_spheres.py`, which
fit spheres to each link's mesh and report what the fit missed.

`provenance` carries the sha256 of `robot.urdf` and the git tree hash of
`meshes/` the fit was made from. A refit merged without its meshes, or meshes
replaced without a refit, is then a test failure rather than a collision model
that quietly describes a different arm (`tests/motion_planner/test_spheres.py`).

## Two of the three sphere sets have different provenances

- **arm links** -- fitted from the cell's meshes, by hand-checked automatic
  fit. Committed per cell.
- **the torch** -- derivable from `tooling.json`, which carries the gun as
  analytic dimensions. `torch_spheres` builds it rather than asking anyone
  to draw it, because the plan request already states the geometry and a
  hand-drawn second copy would drift from the clearance answer the
  authoring side computed with the first.

## Sloppy spheres cost speed, not safety

Under-cover a link and the search misses a contact -- but the CCD verifier
re-checks the winner against exact meshes, so what ships is still sound;
what suffers is the search wasting itself in basins that fail at the end.
Over-cover and plans get conservative. Neither is a safety argument, which
is why the coverage report travels WITH the asset: `problems` reads it and
says which links are worth another pass.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCHEMA = "amr-weld-planner-v1.sphere-model.v1"

# A link left blind by more than this is called out. A weld runs at
# millimetre clearances, so a centimetre-deep hole in the model is a real
# gap; below that the CCD verifier absorbs it.
WORST_UNCOVERED_M = 0.010


class SphereError(ValueError):
    """A sphere model this refuses to read, with the reason."""


@dataclass(frozen=True)
class SphereModel:
    """Spheres per link, each in that link's own frame, metres."""

    cell_id: str
    links: dict[str, list[tuple[tuple[float, float, float], float]]] = field(default_factory=dict)
    coverage: dict[str, dict[str, float]] = field(default_factory=dict)
    # What the fit was made from: `robot_urdf_sha256` and `meshes_sha256`, plus
    # whatever prose the author left. Empty on a model fitted before the
    # description contract.
    provenance: dict[str, Any] = field(default_factory=dict)
    path: Path | None = None

    @property
    def count(self) -> int:
        return sum(len(v) for v in self.links.values())

    @classmethod
    def from_json(cls, document: dict[str, Any], path: Path | None = None) -> SphereModel:
        if document.get("schema") != SCHEMA:
            raise SphereError(f"{path or 'model'}: unknown schema {document.get('schema')!r}")
        links: dict[str, list[tuple[tuple[float, float, float], float]]] = {}
        for name, spheres in (document.get("links") or {}).items():
            built = []
            for entry in spheres:
                centre = entry.get("center_m")
                radius = entry.get("radius_m")
                if not (isinstance(centre, list) and len(centre) == 3):
                    raise SphereError(f"{name}: center_m must be three numbers, got {centre!r}")
                if not isinstance(radius, int | float) or radius <= 0 or not math.isfinite(radius):
                    raise SphereError(f"{name}: radius_m must be a positive number, got {radius!r}")
                built.append(((float(centre[0]), float(centre[1]), float(centre[2])), float(radius)))
            links[str(name)] = built
        return cls(
            cell_id=str(document.get("cell_id") or ""),
            links=links,
            coverage={str(k): dict(v) for k, v in (document.get("coverage") or {}).items()},
            provenance=dict(document.get("provenance") or {}),
            path=path,
        )


def load(cell_id: str, root: Path | None = None) -> SphereModel | None:
    """The sphere model from a model's description, or None if nobody has
    authored one. `root` is the description root (`robot_description/`)."""
    from weld_motion_planner.robot_cell.robot import description

    path = description.spheres_path(str(cell_id), root)
    if not path.is_file():
        return None
    return SphereModel.from_json(json.loads(path.read_text(encoding="utf-8")), path)


def torch_spheres(tooling: dict[str, Any] | None, spacing_m: float = 0.006) -> list[tuple[tuple[float, float, float], float]]:
    """The gun, as spheres along its axis, from the plan request's own numbers.

    Built rather than drawn. `tooling.json` states the torch analytically --
    that is what the authoring side answered clearance with -- so a second,
    hand-made description would be a second torch, and the two would differ
    on the day somebody changed a nozzle.

    Local frame: +Z along the electrode, away from the weld, origin at the
    tip. That is `SEAM-MODEL.md`'s `-u` direction, so placing this set means
    aligning +Z with `-electrode` and putting the origin on `P(s)`.
    """
    if not tooling:
        return []
    reach = float(tooling.get("reach_mm") or tooling.get("body_length_mm") or 120.0) / 1000.0
    body_r = float(tooling.get("body_radius_mm") or 20.0) / 1000.0
    nozzle_r = float(tooling.get("nozzle_radius_mm") or tooling.get("gas_nozzle_radius_mm") or body_r * 0.6) / 1000.0
    stickout = float(tooling.get("stickout_mm") or tooling.get("electrode_extension_mm") or 15.0) / 1000.0
    wire_r = float(tooling.get("wire_radius_mm") or 0.6) / 1000.0

    out: list[tuple[tuple[float, float, float], float]] = []
    # The wire, from the tip to where the nozzle starts. Small spheres, and
    # they matter more than their size suggests: this is the part that goes
    # INTO the groove.
    steps = max(1, round(stickout / max(spacing_m * 0.5, 1e-6)))
    for i in range(steps + 1):
        z = stickout * i / steps
        out.append(((0.0, 0.0, z), wire_r))
    # The nozzle, then the body. Centres run from `first + r` to `last - r`
    # so the UNION spans exactly the declared extent: stepping centres to
    # `reach` itself would push the model a radius past the gun it is
    # describing -- conservative, but a torch 8 mm longer than the one the
    # authoring side answered clearance with is a different torch.
    nozzle_len = min(0.03, max(0.0, reach - stickout))
    for lo, hi, radius in (
        (stickout, stickout + nozzle_len, nozzle_r),
        (stickout + nozzle_len, reach, body_r),
    ):
        if hi - lo <= 1e-9:
            continue
        first = lo + radius
        last = hi - radius
        if last <= first:
            # A section shorter than its own diameter: one sphere centred in
            # it. It still bulges past the ends radially, which is the
            # direction that keeps the model conservative.
            out.append(((0.0, 0.0, 0.5 * (lo + hi)), radius))
            continue
        steps = max(1, math.ceil((last - first) / max(spacing_m, radius * 0.9)))
        for i in range(steps + 1):
            out.append(((0.0, 0.0, first + (last - first) * i / steps), radius))
    return out


def problems(model: SphereModel | None, cell: dict[str, Any], driven: list[str] | None = None) -> list[str]:
    """What is wrong or thin about a sphere model, as sentences.

    The arm chain is what MUST be covered -- it is the body being checked.
    Links that are neither on that chain nor named by the model are not
    faults: they are fields, by design.
    """
    out: list[str] = []
    cell_id = str(cell.get("id") or "?")
    if model is None:
        return [
            (
                f"cell {cell_id!r} has no sphere model; author one with tools/sphere_tool and "
                f"save it as robot_description/{cell_id}/spheres.json"
            )
        ]
    if model.cell_id != cell_id:
        out.append(f"the sphere model is for cell {model.cell_id!r} and this request carries {cell_id!r}")

    known = {str(link.get("name")) for link in (cell.get("links") or [])}
    for name in sorted(set(model.links) - known):
        out.append(f"the model has spheres for {name!r}, which this cell does not have")

    if driven:
        # Only the arm chain needs spheres. Which links those are comes from
        # the caller, because it is `cell.axis_roles` that knows, and this
        # module must not need a kinematic model to read a file.
        missing = [name for name in driven if not model.links.get(name)]
        if missing:
            out.append(
                f"no spheres on {', '.join(missing)}; a link on the arm chain with no spheres "
                "is invisible to collision search"
            )

    for name, report in sorted(model.coverage.items()):
        worst = float(report.get("worst_uncovered_m") or 0.0)
        if worst > WORST_UNCOVERED_M:
            out.append(
                f"{name} is uncovered by up to {worst * 1000:.1f} mm; the search will miss "
                "contacts there and the CCD verifier will reject plans it thought were clear"
            )
    return out


def self_test() -> int:
    cell = {"id": "c", "links": [{"name": "l1"}, {"name": "l2"}, {"name": "table"}]}
    document = {
        "schema": SCHEMA,
        "cell_id": "c",
        "links": {
            "l1": [{"center_m": [0, 0, 0.1], "radius_m": 0.08}],
            "l2": [{"center_m": [0, 0, 0.3], "radius_m": 0.05}],
        },
        "coverage": {"l1": {"worst_uncovered_m": 0.002}, "l2": {"worst_uncovered_m": 0.001}},
    }
    model = SphereModel.from_json(document)
    assert model.count == 2
    assert problems(model, cell, driven=["l1", "l2"]) == []

    # A link on the arm chain with no spheres is invisible, and that is the
    # failure this asset exists to prevent.
    thin = SphereModel.from_json({**document, "links": {"l1": document["links"]["l1"]}})
    assert any("invisible to collision search" in p for p in problems(thin, cell, driven=["l1", "l2"]))

    # A gap the CCD verifier would catch later, reported now.
    holed = SphereModel.from_json({**document, "coverage": {"l1": {"worst_uncovered_m": 0.03}}})
    assert any("uncovered by up to 30.0 mm" in p for p in problems(holed, cell, driven=["l1", "l2"]))

    assert any("has no sphere model" in p for p in problems(None, cell))
    assert any("does not have" in p for p in problems(
        SphereModel.from_json({**document, "links": {**document["links"], "ghost": []}}), cell
    ))

    try:
        SphereModel.from_json({**document, "links": {"l1": [{"center_m": [0, 0, 0], "radius_m": -1}]}})
    except SphereError as exc:
        assert "positive" in str(exc), exc
    else:
        raise AssertionError("a negative radius was accepted")

    # The torch builds from analytic dimensions, tip first, wire thinnest.
    gun = torch_spheres({"reach_mm": 120.0, "body_radius_mm": 20.0, "nozzle_radius_mm": 9.0, "stickout_mm": 15.0})
    assert gun, "no torch spheres were built"
    assert gun[0][0][2] == 0.0, "the first sphere is not at the tip"
    assert gun[0][1] < 0.002, "the wire is not the thin part"
    assert max(r for _, r in gun) <= 0.02 + 1e-9
    assert max(c + r for (_, _, c), r in gun) <= 0.120 + 1e-9, "the model runs past the gun"
    # The tip sphere straddles z=0 by its own small radius, and that is
    # correct: the wire tip is a point, the sphere approximating it reaches
    # a fraction of a millimetre into the weld pool, and being generous
    # THERE is the conservative direction.
    assert min(c for (_, _, c), _ in gun) == 0.0
    assert not torch_spheres(None)

    print(
        f"sphere model self-test ok: reads and refuses, names an uncovered link and a bald one, "
        f"and builds {len(gun)} torch spheres from the plan request's own dimensions"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help="a .weldplan whose cell's model to check")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise SphereError("--read or --self-test")

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.robot.cell import sphere_links
    from weld_motion_planner.robot_cell.robot.profile import (
        for_cell as profile_for_cell,
    )

    request = read(ns.read)
    cell = request.cell
    profile = profile_for_cell(cell)
    arm_links = sphere_links(cell, profile.driven if profile else None)
    model = load(str(cell.get("id")))
    print(
        json.dumps(
            {
                "cell_id": cell.get("id"),
                "model": str(model.path) if model else None,
                "spheres": model.count if model else 0,
                "arm_links": arm_links,
                "problems": problems(model, cell, driven=arm_links),
                "profile_driven": profile.driven if profile else [],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"spheres: {exc}", file=sys.stderr)
        raise SystemExit(1)
