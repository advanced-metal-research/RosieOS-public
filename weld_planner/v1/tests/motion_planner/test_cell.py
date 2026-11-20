"""The cell: whose axes are whose, and where everything is when they move.

The parity test here is the pattern the whole boundary rests on.
`weld_motion_planner` implements forward kinematics itself -- it may not import
`seam_worker` -- so the two would be free to drift on a rotation
convention, and a drift of that kind is invisible until a torch is a
centimetre off a seam. `data/motion/oracles/fk.<cell>.json` is the authoring
side's answer, generated once and committed; this file holds ours to it.

That is the same arrangement `tests/programmer/test_frame_parity.py` uses to
hold the TypeScript client to `frames.py`, for the same reason: two
implementations of one convention need a referee that is neither of them.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from weld_motion_planner.robot_cell.robot.cell import (
    CellError,
    Pose,
    axis_roles,
    chain_to,
    frame,
    invert,
    link_transforms,
    zero_pose,
)

ORACLES = Path(__file__).resolve().parents[2] / "data" / "motion" / "oracles"


@pytest.fixture(scope="module")
def fk_oracle():
    found = sorted(ORACLES.glob("fk.*.json"))
    if not found:
        pytest.skip(f"no FK oracle in {ORACLES}; run tools/make_motion_fixtures.py")
    return json.loads(found[0].read_text())


# --------------------------------------------------------------------------
# parity
# --------------------------------------------------------------------------


def test_forward_kinematics_matches_the_authoring_side(plan_request, fk_oracle):
    """Every link, every case, to 1e-9 m.

    Loose enough to survive float formatting in the oracle and far tighter
    than any convention error could hide in: a transposed rpy order or a
    left-handed rotation is wrong by millimetres at these joint values, not
    by nanometres.
    """
    if plan_request.cell["id"] != fk_oracle["cell_id"]:
        pytest.skip(f"oracle is for {fk_oracle['cell_id']}")

    worst = 0.0
    for case in fk_oracle["cases"]:
        placed = link_transforms(plan_request.cell, Pose(dict(case["values"])))
        for name, flat in case["links"].items():
            expected = np.asarray(flat, dtype=float).reshape(4, 4)
            assert name in placed, f"{name} was not placed at all"
            worst = max(worst, float(np.max(np.abs(placed[name] - expected))))
    assert worst < 1e-9, f"forward kinematics differs from the oracle by {worst:.3e}"


def test_the_oracle_is_not_trivially_satisfiable(fk_oracle):
    """A guard on the guard. If every case were the identity -- an oracle
    generated at zero, say -- the parity test above would pass against an
    implementation that returns identity for everything."""
    moved = 0
    for case in fk_oracle["cases"]:
        for flat in case["links"].values():
            if float(np.max(np.abs(np.asarray(flat).reshape(4, 4) - np.eye(4)))) > 1e-6:
                moved += 1
    assert moved > 10, "the oracle barely moves anything, so it proves little"


# --------------------------------------------------------------------------
# roles
# --------------------------------------------------------------------------


def test_the_arm_and_the_work_chain_are_told_apart(plan_request):
    """The structural half of the axis question, and the half a descriptor
    can answer."""
    roles = axis_roles(plan_request.cell)
    assert roles["arm"], "no axis reaches the tool"
    assert roles["work"], "no axis reaches the work frame"
    assert not set(roles["arm"]) & set(roles["work"])


def test_the_second_bay_is_visible_and_is_nobodys_variable(plan_request):
    """`rosie_1400_v3` specifically: J8 tilts the bay an operator is loading
    while this weld runs. It must not be searched, and it must still be
    posed -- it is geometry the arm can hit."""
    if plan_request.cell["id"] != "rosie_1400_v3":
        pytest.skip("cell-specific")
    roles = axis_roles(plan_request.cell)
    assert roles["arm"] == ["J1", "J2", "J3", "J4", "J5", "J6"], roles["arm"]
    assert roles["work"] == ["J9", "J7"], roles["work"]
    assert roles["off_chain"] == ["J8"], roles["off_chain"]

    placed = link_transforms(plan_request.cell, zero_pose(plan_request.cell))
    assert "positioner_table_b" in placed, "the other bay is not in the posed cell at all"


def test_a_held_index_axis_still_moves_the_part(plan_request):
    """The trap. J9 takes no planning decision and the work frame hangs off
    it, so a plan that dropped it would put the part somewhere it is not."""
    if plan_request.cell["id"] != "rosie_1400_v3":
        pytest.skip("cell-specific")
    cell = plan_request.cell
    rest = zero_pose(cell)
    indexed = rest.with_values({"J9": 1.2})
    moved = float(np.linalg.norm(frame(cell, "work", rest)[:3, 3] - frame(cell, "work", indexed)[:3, 3]))
    assert moved > 0.01, f"turning the H-frame moved the work frame by {moved} m"


def test_an_arm_axis_does_not_move_the_part(plan_request):
    """The independence the two-frame collision split rests on: the part's
    distance field rides the positioner, so it must not ride the arm."""
    cell = plan_request.cell
    roles = axis_roles(cell)
    rest = zero_pose(cell)
    reached = rest.with_values({roles["arm"][0]: 0.8})
    assert np.allclose(frame(cell, "work", reached), frame(cell, "work", rest))
    assert not np.allclose(frame(cell, "tool", reached), frame(cell, "tool", rest))


# --------------------------------------------------------------------------
# refusals
# --------------------------------------------------------------------------


def test_a_link_off_the_tree_is_refused_rather_than_placed_at_the_origin(plan_request):
    """Silently returning identity would draw a limb in the middle of the
    cell, which reads as geometry rather than as a missing joint -- and
    collision-checks against a robot that is not there."""
    cell = {**plan_request.cell, "links": [*plan_request.cell["links"], {"name": "adrift"}]}
    with pytest.raises(CellError, match="does not connect"):
        chain_to(cell, "adrift")


def test_a_pose_missing_an_axis_is_refused(plan_request):
    """Defaulting an unnamed axis to zero is how a held index quietly
    becomes wrong."""
    roles = axis_roles(plan_request.cell)
    short = Pose({name: 0.0 for name in roles["arm"]})
    with pytest.raises(CellError, match="no value for"):
        short.vector(roles["arm"] + roles["work"])


def test_inversion_round_trips(plan_request):
    """The workpiece-frame map is `invert(work) @ tool`, so this is the one
    piece of arithmetic every seam constraint runs through."""
    work = frame(plan_request.cell, "work", zero_pose(plan_request.cell))
    assert np.allclose(invert(work) @ work, np.eye(4), atol=1e-12)


def test_self_test_entry_point_passes():
    from weld_motion_planner.robot_cell.robot.cell import self_test

    assert self_test() == 0
