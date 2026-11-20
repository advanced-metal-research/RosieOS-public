"""The join names the welds that have no path, instead of blaming the move list.

On 2026-09-11 a four-weld program where two welds crossed nothing was refused
with "the program states approach, transit, transit, transit, retract between
2 welds" -- a correct program, measured against only the welds that planned.
"""

from types import SimpleNamespace

from weld_motion_planner.planner.trajectory_optimization.engine.freespace import unplanned_welds


def _report(seam_id):
    return SimpleNamespace(seam_id=seam_id, knots=[0, 1])


def test_the_welds_with_no_path_are_named_in_program_order():
    seams = [{"id": "seam_0006"}, {"id": "seam_0013"}, {"id": "seam_0010"}, {"id": "seam_0014"}]
    planned = [_report("seam_0010"), _report("seam_0014")]
    assert unplanned_welds(seams, planned) == ["seam_0006", "seam_0013"]


def test_a_program_whose_welds_all_planned_has_none():
    seams = [{"id": "a"}, {"id": "b"}]
    assert unplanned_welds(seams, [_report("b"), _report("a")]) == []
