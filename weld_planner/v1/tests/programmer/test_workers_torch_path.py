"""`plan_torch_path` in the reverse direction answers with a reversed weld.

The reverse search runs forwards over a mirrored table, and it used to return
that table's numbers verbatim: `direction: forward`, `direction_changed: false`
and negative travel. The OLP turns a weld round only when `direction_changed`
says so, so a winning reverse search was welded forwards with the arc pushing.
"""

from __future__ import annotations

import base64

import pytest

from seam_worker import paths
from seam_worker.seam.scalar import evaluate
from seam_worker.workers import run

BRACKET_B = paths.FIXTURES_DIR / "bracket_b_2x.step"
# The thin nozzle, because it is the torch that bends seam_0010's travel angle
# hard at its blocked end -- a flat profile would pass a sign test by accident.
NOZZLE = {"id": "mig_gas_nozzle_16", "label": "MIG, Ø16 gas nozzle", "body_radius_mm": 8.0,
          "body_length_mm": 150.0, "standoff_range_mm": {"min": 6.0, "max": 25.0}}
SEARCH = {"work_deviation_deg": 15.0, "travel_deviation_deg": 40.0, "standoff_cost_deg_per_mm": 3.0}


@pytest.fixture(scope="module")
def weld():
    if not BRACKET_B.exists():
        pytest.skip(f"missing fixture {BRACKET_B}")
    step = base64.b64encode(BRACKET_B.read_bytes()).decode()
    joints = run({"operation": "detect_joints", "step_base64": step})["joints"]
    seam = next(s for s in joints["seams"] if s["id"] == "seam_0010")
    joint = next(j for j in joints["weld_joints"] if j["id"] == seam["weld_joint_ref"])
    plan = run({
        "operation": "plan_torch_path", "step_base64": step, "joint": joint, "seam": seam,
        "torch": NOZZLE, "search": SEARCH, "default_standoff_mm": 17.0, "direction": "reverse",
    })["plan"]
    return step, joint, seam, plan


def _clearance(step, joint, seam, torch, standoff_mm):
    return run({"operation": "check_torch_fits", "step_base64": step, "joint": joint,
                "seam": seam, "torch": torch, "options": {"standoff_mm": standoff_mm}})["clearance"]


def test_a_reverse_search_says_the_weld_turned(weld):
    _, _, seam, plan = weld
    assert seam["direction"] == "forward"
    assert (plan["direction"], plan["direction_changed"]) == ("reverse", True)


def test_the_reversed_weld_keeps_the_authored_drag(weld):
    """Authored +10 deg of travel is drag either way round. The unconverted
    answer started this weld at -10: the same pose, labelled as a push."""
    _, _, seam, plan = weld
    authored = evaluate(seam["orientation"]["travel_angle_deg"], 0.0)
    assert authored == 10.0
    assert evaluate(plan["travel_angle_deg"], 0.0) == pytest.approx(authored)


def test_the_reversed_weld_is_the_pose_that_was_searched(weld):
    """Same torch poses as the forward-frame answer, so the same clearance, run
    by run -- a reversal that moved the torch would show up here."""
    step, joint, seam, plan = weld
    negate = {"kind": "piecewise_linear"}
    reversed_weld = {**seam, "direction": "reverse", "orientation": {
        **seam["orientation"],
        "work_angle_deg": plan["work_angle_deg"], "travel_angle_deg": plan["travel_angle_deg"]}}

    def flipped(profile):
        if profile["kind"] == "constant":
            return {"kind": "constant", "value": -profile["value"]}
        return {**negate, "knots": [[s, -v] for s, v in profile["knots"]]}

    as_searched = {**seam, "direction": "forward", "orientation": {
        **seam["orientation"],
        "work_angle_deg": flipped(plan["work_angle_deg"]),
        "travel_angle_deg": flipped(plan["travel_angle_deg"])}}
    a = _clearance(step, joint, reversed_weld, NOZZLE, plan["standoff_mm"])
    b = _clearance(step, joint, as_searched, NOZZLE, plan["standoff_mm"])
    assert a["clear_fraction"] == b["clear_fraction"]
    assert a["clear_runs"] == b["clear_runs"]
