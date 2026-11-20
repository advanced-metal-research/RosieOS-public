"""The robot's collision model, and the torch built from the plan request.

The asset itself is authored in the browser (`tools/sphere_tool`); what is
tested here is the reader, the refusals, and the one part that is DERIVED
rather than drawn -- the torch, whose dimensions the plan request already
carries and which must therefore not be hand-copied.
"""

from __future__ import annotations

import pytest
from weld_motion_planner.robot_cell.collision.spheres import (
    SCHEMA,
    SphereError,
    SphereModel,
    problems,
    torch_spheres,
)
from weld_motion_planner.robot_cell.robot.cell import axis_roles, sphere_links

DOCUMENT = {
    "schema": SCHEMA,
    "cell_id": "c",
    "links": {
        "l1": [{"center_m": [0.0, 0.0, 0.1], "radius_m": 0.08}],
        "l2": [{"center_m": [0.0, 0.0, 0.3], "radius_m": 0.05}],
    },
    "coverage": {"l1": {"worst_uncovered_m": 0.002}, "l2": {"worst_uncovered_m": 0.001}},
}
CELL = {"id": "c", "links": [{"name": "l1"}, {"name": "l2"}, {"name": "table"}]}


def test_a_well_formed_model_reads_clean():
    model = SphereModel.from_json(DOCUMENT)
    assert model.count == 2
    assert problems(model, CELL, driven=["l1", "l2"]) == []


def test_a_bald_arm_link_is_caught():
    """The failure this asset exists to prevent: a link on the checked body
    with no spheres is simply invisible to the search."""
    thin = SphereModel.from_json({**DOCUMENT, "links": {"l1": DOCUMENT["links"]["l1"]}})
    assert any("invisible to collision search" in p for p in problems(thin, CELL, driven=["l1", "l2"]))


def test_a_coverage_gap_is_reported_with_its_depth():
    """Under-covering costs search time rather than safety -- the CCD verifier
    still rejects what it missed -- so this is a report, not a refusal."""
    holed = SphereModel.from_json({**DOCUMENT, "coverage": {"l1": {"worst_uncovered_m": 0.03}}})
    said = " ".join(problems(holed, CELL, driven=["l1", "l2"]))
    assert "30.0 mm" in said and "CCD verifier" in said


def test_links_the_cell_does_not_have_are_caught():
    ghost = SphereModel.from_json({**DOCUMENT, "links": {**DOCUMENT["links"], "ghost": []}})
    assert any("does not have" in p for p in problems(ghost, CELL))


def test_a_negative_radius_is_refused_rather_than_stored():
    with pytest.raises(SphereError, match="positive"):
        SphereModel.from_json({**DOCUMENT, "links": {"l1": [{"center_m": [0, 0, 0], "radius_m": -1}]}})


def test_an_unknown_schema_is_refused():
    with pytest.raises(SphereError, match="unknown schema"):
        SphereModel.from_json({"schema": "something.else", "cell_id": "c"})


def test_an_unwritten_model_says_what_to_do():
    said = problems(None, CELL)
    # The tool that writes it, and where in the model's description it goes.
    assert len(said) == 1 and "tools/sphere_tool" in said[0]
    assert "robot_description/c/spheres.json" in said[0]


# --------------------------------------------------------------------------
# the torch
# --------------------------------------------------------------------------


def _tooling(path) -> dict:
    """The package's `tooling.json`, opened the way the authoring tool opens it."""
    from weldplan.plan_request import read_request

    return read_request(path.read_bytes())["tooling"]


def test_the_torch_is_built_from_the_requests_own_dimensions(motion_fixture_path):
    """Not drawn. `tooling.json` is what the authoring side answered clearance
    with, so a second hand-made description would be a second torch -- and they
    would disagree the day a nozzle changed.

    Read straight out of the package: this builder AUTHORS the sphere model
    (`tools/fit_spheres.py`), and the planner that reads the model never touches
    the document."""
    gun = torch_spheres(_tooling(motion_fixture_path))
    assert gun, "the fixture carries a torch and no spheres were built"
    assert gun[0][0][2] == 0.0, "the first sphere is not at the tip"
    assert gun[0][1] < 0.002, "the wire is not the thinnest part"
    radii = [r for _, r in gun]
    assert min(radii) < max(radii), "the torch came out a uniform tube"


def test_the_torch_model_stays_within_its_declared_reach(motion_fixture_path):
    """The COVERED extent, not the centres. An earlier builder stepped
    centres up to the declared length, which put the union a radius past it
    -- a torch 8 mm longer than the one the authoring side answered
    clearance with, which is a different torch."""
    tooling = _tooling(motion_fixture_path) or {}
    reach = float(tooling.get("reach_mm") or tooling.get("body_length_mm") or 120.0) / 1000.0
    gun = torch_spheres(tooling)
    assert max(c[2] + r for c, r in gun) <= reach + 1e-9
    # The tip sphere straddles z=0 by its own radius on purpose: the wire
    # tip is a point, and a model that reaches a fraction of a millimetre
    # into the weld pool errs conservatively where contact is intended.
    assert min(c[2] for c, _ in gun) == 0.0


def test_no_tooling_means_no_torch_spheres():
    assert torch_spheres(None) == []


def test_which_links_need_covering_is_derived_and_has_three_traps(plan_request):
    """`sphere_links` is the body being CHECKED, and each exclusion here was
    a wrong rule that got written first.

    - `floor` is on the tool chain and fixed to world: never moves, so it is
      a field. "Everything on the tool chain" put a three-metre base plate
      in the sphere model.
    - `positioner_table_a` moves under J7 and is still a field: it is rigid
      with respect to the workpiece and bakes into that frame. "Everything
      that moves" put it here, which would have had the planner checking
      the table against the part bolted to it.
    - `end_effector` and `tool0` reach `link_6` through FIXED joints and
      must be covered anyway -- they move with it and they carry the torch.
      "Children of driven joints" dropped them.
    """
    if plan_request.cell["id"] != "rosie_1400_v3":
        pytest.skip("cell-specific")
    covered = set(sphere_links(plan_request.cell))
    assert {"link_1", "link_6", "end_effector", "tool0"} <= covered
    assert "floor" not in covered, "a fixed base plate is a field, not spheres"
    assert "positioner_table_a" not in covered, "the workpiece group is a field"
    assert "positioner_table_b" not in covered, "the second bay is a field"


def test_the_arm_axes_still_agree_with_axis_roles(plan_request):
    """The two derivations answer different questions -- which AXES drive the
    tool, which LINKS need covering -- and must not disagree about the arm."""
    roles = axis_roles(plan_request.cell)
    children = {j["child"] for j in plan_request.cell["joints"] if j.get("name") in set(roles["arm"])}
    assert children <= set(sphere_links(plan_request.cell))


def test_self_test_entry_point_passes():
    from weld_motion_planner.robot_cell.collision.spheres import self_test

    assert self_test() == 0
