"""The solved connecting path must fit limits between optimizer samples too."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from weld_motion_planner.planner.trajectory_optimization.engine import freespace
from weld_motion_planner.verifier.hermite_rates import HermiteRateError, judge_hermite_rates
from weld_motion_planner.verifier.verify import _hermite, check_limits


def refused_retract():
    doc = json.loads((Path(__file__).parent / "fixtures" / "retract_rate_overshoot.json").read_text())
    return (doc["axis_names"], np.array(doc["knots_rad"]),
            np.array(doc["knot_velocity_rad_s"]), np.array(doc["times_s"]), doc["limits"])


def test_actual_retract_fits_unchanged_limits_without_changing_its_path():
    names, q, v, t, limits = refused_retract()
    before = judge_hermite_rates(names, q, v, t, limits)
    assert before["violations"][0]["kind"] == "acceleration"
    assert before["violations"][0]["max_abs_rad_s2"] == pytest.approx(0.50328113487826)
    times, rates, scale = freespace.retime_rest_to_rest_wire(names, q, v, t, limits)
    assert 1.003 < scale < 1.004
    assert times[-1] - t[-1] < 0.02
    assert judge_hermite_rates(names, q, rates, times, limits)["verdict"] == "PASS"
    assert np.array_equal(rates[[0, -1]], np.zeros((2, 1)))
    # Compare off-grid locations in every segment, not just identical knots.
    for k, dt in enumerate(np.diff(t)):
        for u in (0.13, 0.5, 0.87):
            original = _hermite(q[k], q[k + 1], v[k], v[k + 1], dt, u)
            slowed = _hermite(q[k], q[k + 1], rates[k], rates[k + 1],
                              times[k + 1] - times[k], u)
            np.testing.assert_allclose(slowed, original, rtol=0, atol=1e-14)
    assert limits["J2"]["acceleration_rad_s2"] == 0.5


def test_interior_velocity_extremum_also_sets_the_clock():
    q, v, t = np.array([[0.0], [1.0]]), np.zeros((2, 1)), np.array([0.0, 1.0])
    limits = {"J1": {"velocity_rad_s": 1.0, "acceleration_rad_s2": 100.0}}
    times, rates, scale = freespace.retime_rest_to_rest_wire(["J1"], q, v, t, limits)
    assert scale == pytest.approx(1.5 * (1 + 1e-6))
    assert judge_hermite_rates(["J1"], q, rates, times, limits)["verdict"] == "PASS"


def test_already_legal_motion_keeps_its_clock():
    names, q, v, t, limits = refused_retract()
    t, v = t * 2, v / 2
    times, rates, scale = freespace.retime_rest_to_rest_wire(names, q, v, t, limits)
    assert scale == 1.0
    assert times is t and rates is v


def test_moving_endpoint_contract_is_not_scaled_away():
    names, q, v, t, limits = refused_retract()
    v[0, 0] = 0.01
    times, rates, scale = freespace.retime_rest_to_rest_wire(names, q, v, t, limits)
    assert scale == 1.0
    assert times is t and rates is v
    assert judge_hermite_rates(names, q, rates, times, limits)["verdict"] == "FAIL"


def test_retiming_does_not_hide_an_interior_position_violation():
    q, v, t = np.zeros((3, 1)), np.array([[0.0], [3.0], [0.0]]), np.array([0.0, 1.0, 2.0])
    limits = {"J1": {"velocity_rad_s": 1.0, "acceleration_rad_s2": 1.0}}
    times, rates, scale = freespace.retime_rest_to_rest_wire(["J1"], q, v, t, limits)
    assert scale > 1.0
    findings, missing = check_limits(
        SimpleNamespace(axis_names=["J1"]),
        {"joints": [{"name": "J1", "limits_rad": {"lower": -0.1, "upper": 0.1}}]},
        {"limits": limits}, q, times, knot_velocities=rates)
    assert not missing
    assert findings and all(f["kind"] == "position" for f in findings)


@pytest.mark.parametrize("bad", [None, 0.0, float("nan"), float("inf")])
def test_invalid_limit_cannot_be_used_to_retime(bad):
    names, q, v, t, limits = refused_retract()
    limits["J2"]["acceleration_rad_s2"] = bad
    with pytest.raises(HermiteRateError):
        freespace.retime_rest_to_rest_wire(names, q, v, t, limits)


def test_nonfinite_wire_is_refused():
    names, q, v, t, limits = refused_retract()
    q[4, 0] = float("nan")
    with pytest.raises(HermiteRateError):
        freespace.retime_rest_to_rest_wire(names, q, v, t, limits)


def test_finish_verifies_and_emits_the_same_retimed_curve(monkeypatch):
    names, q, v, t, limits = refused_retract()
    solved = SimpleNamespace(
        knots=q, qdot=v, times_s=t, span_times_s=np.linspace(0.0, t[-1], 14),
        clearance_m=0.002, env_clearance_m=0.002, self_clearance_m=0.004,
        cost_before=2.0, cost_after=1.0, seed="control")
    monkeypatch.setattr(freespace, "verify_kinematics", lambda _: SimpleNamespace(axis_names=names))
    calls = []
    verdict = SimpleNamespace(worst_at=None)

    def verify(_cell, _geometry, waypoints, **kwargs):
        exact = judge_hermite_rates(names, waypoints, kwargs["velocities"],
                                    kwargs["times_s"], kwargs["profile"]["limits"])
        assert exact["verdict"] == "PASS"
        calls.append(kwargs)
        return verdict

    monkeypatch.setattr(freespace, "verify_free_space", verify)
    moves = freespace._finish_moves(
        SimpleNamespace(cell={}), [SimpleNamespace(kind="retract", from_id="seam_0010", to_id=None)],
        [solved], [[]], [t[-1]], SimpleNamespace(names=names, vmax=np.array([3.0])),
        SimpleNamespace(moved_axes=names, profile=SimpleNamespace(limits=limits)),
        {}, object(), 0.0, None)
    assert len(calls) == 1
    assert moves[0].verdict is verdict
    assert moves[0].times_s[-1] > t[-1]
    assert calls[0]["times_s"][-1] == moves[0].times_s[-1]
    assert moves[0].knots is q
    assert any("exact rate retime" in note for note in moves[0].notes)
