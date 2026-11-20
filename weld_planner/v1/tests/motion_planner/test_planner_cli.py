"""CPU-only checks for the planner CLI's report and boundary behavior."""

from __future__ import annotations

from types import SimpleNamespace

from weld_motion_planner.io.summary import print_answer


class _PassedVerdict:
    def summary(self) -> dict[str, str]:
        return {"verdict": "PASS"}


def test_verified_trajectory_does_not_print_not_checked(capsys):
    trajectory = SimpleNamespace(
        seam_id="seam-1",
        verdict=_PassedVerdict(),
        tracking=None,
    )
    plan = SimpleNamespace(
        seam_search_result=None,
        weld_trajectories=[trajectory],
        connecting_trajectories=None,
        connecting_trajopt_error=None,
    )

    print_answer(plan)

    output = capsys.readouterr().out
    assert "seam-1         PASS" in output
    assert "not checked" not in output
