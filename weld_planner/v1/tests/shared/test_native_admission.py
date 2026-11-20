"""The admission gate says what the failing report measured.

`motion_not_verified: ... tracking: FAIL` could not be told apart from a miss
of a tenth of a millimetre, and the person reading it had no way to get the
number: the report is inside a result the OLP never shows. The gate repeats
the report's own figures and computes none.
"""

from __future__ import annotations

import pytest
import math

from weldplan.native_admission import MotionAdmissionError, require_native_motion

_PASS = {"verdict": "PASS", "collisions": 0, "penetrating": 0,
         "limit_violations": 0, "unverifiable": []}


def _result(tracking: dict) -> dict:
    return {"seams": [{"id": "s1", "crossed": True,
                       "trajectory": {"verdict": dict(_PASS), "tracking": tracking}}]}


def test_a_failed_tracking_refusal_carries_the_measured_miss():
    tracking = {"verdict": "FAIL", "tolerance_mm": 0.5, "bound_mm": 2.31,
                "worst_seen_mm": 1.707, "segments_out": 14, "undecided": 0,
                "unverifiable": []}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_result(tracking))
    assert refused.value.reason == "motion_not_verified"
    detail = refused.value.detail
    assert detail.startswith("seam s1 tracking: FAIL (")
    assert detail.endswith("; requires native PASS")
    for said in ("worst seen 1.707 mm", "tolerance 0.5 mm", "14 segments out of tolerance"):
        assert said in detail, detail
    assert "undecided" not in detail, "a zero count is not a finding"


def test_a_refused_tracking_refusal_says_why_it_could_not_decide():
    tracking = {"verdict": "REFUSED", "tolerance_mm": 0.5, "bound_mm": float("inf"),
                "segments_out": 0, "undecided": 0,
                "unverifiable": ["tracking: no derivative bound", "another"]}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_result(tracking))
    detail = refused.value.detail
    assert "unverifiable: tracking: no derivative bound (+1 more)" in detail
    assert "inf" not in detail, "a bound that does not exist is not a number to print"


def test_a_report_with_no_figures_reads_as_before():
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_result({"verdict": "FAIL"}))
    assert refused.value.detail == "seam s1 tracking: FAIL; requires native PASS"


def test_a_passing_result_is_admitted():
    require_native_motion(_result({"verdict": "PASS", "tolerance_mm": 0.5, "bound_mm": 0.2,
                                   "segments_out": 0, "undecided": 0, "unverifiable": []}))


def test_uncrossed_seam_retains_sampled_search_frontier_and_causes():
    seam = {"id": "seam_0011", "crossed": False, "length_mm": 111.07,
            "frontier": 21, "fractions": [i / 112 for i in range(113)],
            "frontier_causes": {"joint limit": 12, "collision": 8, "duplicate": 4},
            "candidates": []}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion({"seams": [seam]})
    assert refused.value.reason == "seam_not_planned"
    for text in ("seam seam_0011", "no complete path", "sample 22 of 113",
                 "18.75%", "20.8256 mm", "12 poses outside joint travel limits",
                 "8 poses rejected by sphere collision screening", "4 duplicate poses",
                 "not a continuous mesh collision certificate"):
        assert text in refused.value.detail
    assert "penetration" not in refused.value.detail


def test_uncrossed_seam_preserves_error_without_inventing_frontier_evidence():
    seam = {"id": "s1", "crossed": False, "error": "tool frame is missing",
            "frontier": 50, "fractions": [0, 1],
            "frontier_causes": {"collision": -1, "joint limit": float("nan")}}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion({"seams": [seam]})
    assert "tool frame is missing" in refused.value.detail
    for text in ("sample 51", "collision", "joint travel", "nan", "certificate"):
        assert text not in refused.value.detail


def test_uncrossed_seam_reports_unreachable_and_transition_failures_separately():
    seam = {"id": "s1", "crossed": False,
            "frontier_causes": {"no ik": 6, "no edge": 2}}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion({"seams": [seam]})
    assert "6 poses unable to reach" in refused.value.detail
    assert "2 poses with no allowed transition" in refused.value.detail


def _failed_motion(report: dict, **extra) -> dict:
    return {"seams": [{"id": "seam_0011", "crossed": True,
                       "trajectory": {"verdict": report, **extra}}]}


def test_collision_and_exact_limit_evidence_survive_admission_refusal():
    report = {**_PASS, "verdict": "FAIL", "collisions": 1, "penetrating": 1,
              "limit_violations": 2,
              "collision_details": [{"pair": ["link_4", "workpiece"],
                                     "reason": "collision", "penetrating": True,
                                     "worst_clearance_m": -1, "segment": 8}],
              "limit_details": [
                  {"axis": "J2", "kind": "position", "value_rad": math.radians(110),
                   "limit_rad": list(map(math.radians, [-109, 109])), "index": 3},
                  {"axis": "J3", "kind": "velocity", "value": math.radians(-40),
                   "limit": math.radians(30), "segment": 4, "time_s": .125}]}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_failed_motion(report))
    detail = refused.value.detail
    assert "Contact or overlap between link_4 and workpiece (segment 8)" in detail
    assert "J2 position 110° outside [-109°, 109°] (waypoint 3)" in detail
    assert "J3 velocity magnitude 40°/s exceeds 30°/s (segment 4, 0.125 s)" in detail
    assert "1000" not in detail, "the -1 penetration sentinel is not a measured depth"
    assert refused.value.reason == "motion_not_verified"


def test_margin_and_unresolved_clearance_are_not_reported_as_contact():
    report = {**_PASS, "verdict": "FAIL", "collisions": 2,
              "collision_details": [
                  {"pair": ["torch", "part"], "penetrating": False,
                   "worst_clearance_m": .001, "margin_m": .002},
                  {"pair": ["link_1", "fixture_0"], "reason": "undecided"}]}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_failed_motion(report))
    assert "torch and part: 1 mm; required 2 mm" in refused.value.detail
    assert "Clearance could not be verified between link_1 and fixture_0" in refused.value.detail
    assert "Contact or overlap" not in refused.value.detail


def test_legacy_limit_records_and_non_position_failures_remain_distinct():
    findings = [
        {"kind": "singular", "index": 1},
        {"kind": "travel_speed", "index": 2},
        {"axis": "J6", "kind": "acceleration", "signed_rad_s2": -math.pi,
         "limit_rad_s2": math.pi / 2},
        {"axis": "J1", "kind": "position", "value_rad": float("nan"),
         "limit_rad": [-1, 1]},
    ]
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_failed_motion({**_PASS, "verdict": "FAIL", "limit_violations": 4},
                                            limit_violations=findings))
    detail = refused.value.detail
    assert "cannot move along the weld direction" in detail
    assert "travel speed exceeds the available joint speeds" in detail
    assert "J6 acceleration magnitude 180°/s² exceeds 90°/s²" in detail
    assert "nan" not in detail


def test_detailed_evidence_is_bounded_and_does_not_change_pass_gate():
    finding = {"axis": "J1", "kind": "position", "value_rad": 2, "limit_rad": [-1, 1]}
    report = {**_PASS, "verdict": "FAIL", "limit_violations": 20,
              "limit_details": [finding] * 16}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_failed_motion(report))
    assert refused.value.detail.count("J1 position") == 6
    assert "14 more limit findings" in refused.value.detail
    report["verdict"] = "PASS"
    with pytest.raises(MotionAdmissionError, match="PASS requires limit_violations=0"):
        require_native_motion(_failed_motion(report))


def test_connecting_move_failure_also_carries_evidence():
    report = {**_PASS, "verdict": "FAIL", "collisions": 1,
              "collision_details": [{"pair": ["link_6", "fixture_0"], "penetrating": True}]}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion({"connecting_trajectories": [{"kind": "approach", "from": "home",
                                                           "to": "s1", "verdict": report}]})
    assert "approach move 0 (home -> s1)" in refused.value.detail
    assert "Contact or overlap between link_6 and fixture_0" in refused.value.detail


def test_small_limit_excess_is_not_rounded_away():
    report = {**_PASS, "verdict": "FAIL", "limit_violations": 1,
              "limit_details": [{"axis": "J6", "kind": "position",
                                 "value_rad": math.radians(120.000001),
                                 "limit_rad": list(map(math.radians, [-120, 120]))}]}
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion(_failed_motion(report))
    assert "120.000001° outside [-120°, 120°]" in refused.value.detail
