from weld_motion_planner.verifier.verify import Verdict
from weldplan.native_admission import MotionAdmissionError, require_native_motion
import pytest


def test_summary_retains_bounded_collision_and_limit_evidence_without_changing_counts():
    collisions = [{"pair": ["link_4", "workpiece"], "segment": i, "penetrating": True}
                  for i in range(12)]
    limits = [{"axis": "J2", "kind": "position", "index": i,
               "value_rad": 2, "limit_rad": [-1, 1]} for i in range(20)]
    verdict = Verdict(passed=False, collisions=collisions, limit_violations=limits)
    report = verdict.summary()
    assert report["verdict"] == "FAIL"
    assert report["collisions"] == report["penetrating"] == 12
    assert report["limit_violations"] == 20
    assert report["collision_details"] == collisions[:8]
    assert report["limit_details"] == limits[:16]
    assert len(verdict.collisions) == 12
    assert len(verdict.limit_violations) == 20
    with pytest.raises(MotionAdmissionError) as refused:
        require_native_motion({"connecting_trajectories": [{"kind": "transit", "from": "s1",
                                                           "to": "s2", "verdict": report}]})
    assert "Contact or overlap between link_4 and workpiece" in refused.value.detail
    assert "J2 position" in refused.value.detail
    assert "9 more collision findings" in refused.value.detail
    assert "14 more limit findings" in refused.value.detail
