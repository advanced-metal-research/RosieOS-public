"""A fixture is judged against what MOVES, and against the part.

`verify.from_cell` pairs the operator's fixtures "with the arm and the
workpiece only". The arm was read as every link between the tool and the base
frame, which on `rosie_1420_v1` includes `base` and `pedestal`: both hang off
`world` by fixed joints. The cell's own placeholder stand overlaps the pedestal
mesh by a 75 x 50 mm corner, so every segment of every seam reported a
collision between two bodies that never move (`f7a12ed2`, 2026-09-21: 122 of
122), and no hand-placed weld on that cell could be planned.
"""

from __future__ import annotations

import json
from pathlib import Path

from weld_motion_planner.robot_cell.collision.fixtures import fixture_meshes
from weld_motion_planner.verifier import verify
from weldplan.urdf import read_urdf

ROBOT = Path(__file__).resolve().parents[4] / "robot_description" / "robots" / "rosie_1420_v1"

# The OLP's default stand for this cell (`ui/src/domain/workcell.ts`).
STAND = [{"id": "stand", "name": "Stand", "rpy_rad": [0, 0, 0], "xyz_m": [1, 1, 0],
          "shape": {"kind": "box", "size_m": [0.5, 0.5, 1]}}]


def _fixture_partners() -> set[str]:
    frames = json.loads((ROBOT / "config.json").read_text())["frames"]
    cell = read_urdf(ROBOT / "robot.urdf", frames, fixtures=STAND)
    assert cell["frames"]["tool"] == "tool0", "without a tool frame there is no arm to pair with"
    geometry = verify.from_cell(cell, mesh_root=ROBOT / "meshes",
                                extra_static=fixture_meshes(STAND))
    partners = set()
    for a, b in geometry.pairs:
        if a.startswith("fixture_") or b.startswith("fixture_"):
            partners.add(b if a.startswith("fixture_") else a)
    return partners


def test_a_fixture_is_not_paired_with_what_never_moves():
    partners = _fixture_partners()
    assert "pedestal" not in partners and "base" not in partners, partners


def test_a_fixture_is_still_paired_with_every_link_that_moves():
    partners = _fixture_partners()
    for link in ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6"):
        assert link in partners, (link, partners)
