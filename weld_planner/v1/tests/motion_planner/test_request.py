"""The planner's front door: what it accepts, and what it says is missing.

These read the COMMITTED fixture plan requests rather than building them, which
is the point of committing them -- no OCCT, no STEP import, milliseconds. The
authoring pipeline that produced them is exercised by `tests/programmer/`; if it
changes what it emits, `tools/make_motion_fixtures.py` is re-run and the diff
is read by a human.
"""

from __future__ import annotations

import pytest
from weld_motion_planner.io.plan_request import PlanRequest, gaps, read


def test_a_fixture_request_opens_with_every_document_present(plan_request):
    """All five, because a planner that got four would discover the fifth
    missing three stages in."""
    assert plan_request.program["schema"] == "robot.v4.program.v2"
    assert plan_request.placement["cell_id"] == plan_request.cell["id"]
    assert plan_request.step_bytes.startswith(b"ISO-10303-21;")
    assert plan_request.seams


def test_the_geometry_is_in_the_workpiece_frame(plan_request):
    """GOAL.md non-negotiable 5. Everything this planner does with the seam is
    evaluated in that frame, so a program in any other one is a different
    document wearing the same schema."""
    assert plan_request.program["workpiece"]["frame_id"] == "workpiece"


def test_the_cell_arrives_with_kinematics_and_without_meshes(plan_request):
    """The split `plan_request.py` argues for: the descriptor pins the joints and
    frames the placement depends on, and the collision geometry is the
    planner's own."""
    assert plan_request.cell["meshes_omitted"] is True
    assert all(link["visuals"] == [] for link in plan_request.cell["links"])
    assert plan_request.cell["joints"], "no joints: nothing to run forward kinematics on"
    assert plan_request.cell["frames"]["work"]


def test_no_joint_value_reached_the_planner(plan_request):
    """The positioner is prescribed HERE and nowhere upstream. A request that
    carried one would be a plan already made, and this stage would be
    ratifying it rather than making it."""
    import json

    blob = json.dumps({"program": plan_request.program, "placement": plan_request.placement})
    for token in ("joint_values", "q_start", "seed_pose", "positioner_deg"):
        assert token not in blob


def test_the_fixtures_are_plannable_in_principle(plan_request):
    """Empty `blocking` is the M0 bar: nothing about these requests is wrong.
    What is missing is per-cell setup, which is a different list on purpose."""
    found = gaps(plan_request)
    assert found["blocking"] == [], found["blocking"]
    assert found["assets"], "a cell with no limits, spheres or reset pose should say so"


def test_the_per_cell_assets_are_named_rather_than_assumed(plan_request):
    """A URDF `<limit>` carries position only. Velocity and acceleration are
    what stage 7 parametrizes against and what T2 checks, so their absence
    has to be visible on day one rather than at the timing stage."""
    assets = " ".join(gaps(plan_request)["assets"])
    assert "velocity and acceleration" in assets
    assert "sphere model" in assets
    assert "reset pose" in assets


def test_the_fixture_SET_exercises_a_band():
    """M4's rescue pass and M5's angle attraction both need a seam with an
    admissible band, and the corpus has one again.

    This asserted the ABSENCE for a while, and said so: the corpus had become
    three plan requests exported from real projects, no operator had authored
    a band in any of them, and the docstring ended "the day a band is authored
    this test fails, and the fix is to flip it back to demanding one".

    That day came with the program.v2 rebuild. `make_motion_fixtures` authors
    a band on the first seam of every fixture again, deliberately: a corpus
    whose every seam has no band would let the rescue pass and the angle
    attraction be "tested" against data that never triggers them.

    So it demands one, and names the fixtures that carry it -- the flip the
    old docstring asked for.
    """
    from pathlib import Path

    from weld_motion_planner.io.plan_request import read

    from .committed import committed_plans

    root = Path(__file__).resolve().parents[2] / "data" / "motion"
    banded = []
    for path in committed_plans(root):
        request = read(path)
        for seam in request.seams:
            limits = (seam.get("orientation") or {}).get("limits")
            if limits and limits.get("work_angle_deg", {}).get("max_rate_deg_per_mm", 0) > 0:
                banded.append(path.stem)
                break
    assert banded, (
        "no fixture authors an admissible band, so the rescue pass and the "
        "angle attraction have nothing to bite on -- rebuild the corpus with "
        "tools/make_motion_fixtures.py")


def test_the_planner_does_not_read_the_tooling_document(plan_request):
    """The torch collides, but its geometry is the CELL's, not the weld
    program's: the screen places the spheres authored on `link_6`, into whose
    mesh the gun is baked. The standoff is the seam's own authored
    `process.standoff_mm` (or its wire's stickout), not this document's.

    So `tooling.json` is not opened on any planning path, and this used to
    BLOCK on its absence -- a refusal whose stated reason ("the torch is what
    collides") was true of the sphere model and not of the document. Pinned
    both ways: the field is gone, and the spheres are there.
    """
    import dataclasses

    from weld_motion_planner.robot_cell.collision import spheres as spheremod

    assert "tooling" not in {f.name for f in dataclasses.fields(PlanRequest)}, \
        "the planner reads the tooling document again; the gun is the cell's"
    model = spheremod.load(str(plan_request.cell["id"]))
    assert model is not None and model.links.get("link_6"), \
        "no spheres on the link the gun is baked into, so nothing models the torch"


def test_a_tampered_request_is_refused(motion_fixture_path):
    """Digests are checked on read. A request edited after it was built would
    otherwise be acted on without anyone having validated it."""
    import io
    import json
    import zipfile

    from weldplan.plan_request import PlanRequestError

    original = zipfile.ZipFile(motion_fixture_path.open("rb"))
    swapped = io.BytesIO()
    with zipfile.ZipFile(swapped, "w") as out:
        for item in original.infolist():
            payload = original.read(item.filename)
            if item.filename == "program.json":
                # The placement travels INSIDE the program now (D23), so the
                # document to tamper with is the program itself.
                document = json.loads(payload)
                document["placement"]["offset_xyz_m"] = [9, 9, 9]
                payload = json.dumps(document).encode()
            out.writestr(item.filename, payload)
    with pytest.raises(PlanRequestError, match="digest"):
        from weldplan.plan_request import read_request

        read_request(swapped.getvalue())


def test_both_fixtures_open(motion_fixture_path):
    """Parametrized over every committed plan request, so a fixture added later is
    covered without editing this file."""
    assert read(motion_fixture_path).seams


def test_self_test_entry_point_passes():
    from weld_motion_planner.io.plan_request import self_test

    assert self_test() == 0
