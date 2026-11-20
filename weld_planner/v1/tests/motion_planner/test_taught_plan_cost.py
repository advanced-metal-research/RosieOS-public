"""What a taught program costs to plan, counted, and how long it takes to fly.

The owner's report (2026-10-11): the 1430 cell's `test1` -- five taught moves, four of
them straight lines of 0.42-0.70 m -- took 63 s to plan on the cell's relay CPU
planner and planned a 98.5 s trajectory. The fixture is that program as the
cell's OLP packed it (`weld-plan/export`, planned from the program's start
rather than a measured pose; the 1430 cell's URDF, meshes and limits inside).

Wall time is not asserted: it depends on the runner. The cost is asserted
where it came from -- knots per line (every later stage pays per knot) and
the collision verifier's predicate calls -- with the measured counts beside
each bound. Durations and the path are asserted in absolute units: seconds
against metres over the row's own mm/s, millimetres off the taught line by
the verifier's independent forward kinematics -- 0.01 mm, the robot's quoted
repeatability (owner 2026-10-12), for the planned spline and for what rt-core
commands every servo cycle from the dense export.
"""
from __future__ import annotations

import copy
import dataclasses
from pathlib import Path

import numpy as np
import pytest

from weld_motion_planner.io.wire import request_from_bytes
from weld_motion_planner.planner.planner_main import MotionPlanningOptions, plan_motion
from weld_motion_planner.verifier import verify as verifier

from .committed import committed_plan

FIXTURE = Path(__file__).parent / "fixtures" / "taught_test1.weldplan"
# Straight-line lengths of P1->P2 .. P4->P5, from the taught TCP poses.
LINE_LENGTHS_M = (0.4198, 0.4624, 0.6953, 0.4215)


@pytest.fixture(scope="module")
def request_test1():
    return request_from_bytes(committed_plan(FIXTURE).read_bytes())


def _plan(request, monkeypatch):
    calls = {"collision_predicate": 0}
    predicate = verifier._closer_than

    def counted(*args):
        calls["collision_predicate"] += 1
        return predicate(*args)

    monkeypatch.setattr(verifier, "_closer_than", counted)
    plan = plan_motion(request, MotionPlanningOptions(), log=False)
    assert plan.connecting_trajopt_error is None
    return plan.connecting_trajectories, calls


def _off_line_mm(request, move):
    """Worst distance of the flown TCP from its taught line, 40 points per interval."""
    kin = verifier.kinematics(request.cell)
    tool = request.cell["frames"]["tool"]
    names = move["axis_names"]
    q = np.deg2rad(np.asarray(move["knots_deg"]))
    v = np.deg2rad(np.asarray(move["knot_velocity_deg_s"]))
    times = np.asarray(move["times_s"])

    def tcp(joints):
        full = [joints[names.index(n)] if n in names else np.deg2rad(move["held"][n]) for n in kin.axis_names]
        return kin.transforms(full)[tool][:3, 3]

    start, end = tcp(q[0]), tcp(q[-1])
    direction = (end - start) / np.linalg.norm(end - start)
    worst = 0.0
    for i in range(len(q) - 1):
        h = times[i + 1] - times[i]
        for t in np.linspace(0.0, 1.0, 41)[1:-1]:
            t2, t3 = t * t, t * t * t
            joints = ((2*t3 - 3*t2 + 1) * q[i] + h * (t3 - 2*t2 + t) * v[i]
                      + (-2*t3 + 3*t2) * q[i + 1] + h * (t3 - t2) * v[i + 1])
            offset = tcp(joints) - start
            worst = max(worst, float(np.linalg.norm(offset - (offset @ direction) * direction)))
    return worst * 1000.0, float(np.linalg.norm(end - start))


def _rt_core_cycles(segment, cycle_s):
    """What rt-core commands each cycle from a dense segment: a cubic Hermite
    between rows from their q and qd (`rt-core/engine/trajectory.hpp`
    `sample_trajectory`, `interpolation::Hermite`), held at the last row."""
    t = np.asarray(segment.t_s)
    q, v = np.asarray(segment.q_rad), np.asarray(segment.qd_rad_s)
    cycles = np.arange(0.0, t[-1] + 1e-12, cycle_s)
    k = np.clip(np.searchsorted(t, cycles + 1e-12, side="right") - 1, 0, len(t) - 2)
    span = (t[k + 1] - t[k])[:, None]
    alpha = ((cycles - t[k]) / (t[k + 1] - t[k]))[:, None]
    delta = q[k + 1] - q[k]
    c2 = 3 * delta - span * (2 * v[k] + v[k + 1])
    c3 = -2 * delta + span * (v[k] + v[k + 1])
    return q[k] + alpha * (span * v[k] + alpha * (c2 + alpha * c3))


def _per_cycle_off_segment_mm(request, plan):
    """Worst distance of the commanded TCP, every 1 ms rt-core cycle (the 1430 cell
    `cycle_ns` 1000000) of the dense export, from each taught line SEGMENT:
    beside it, or past either end."""
    from weld_motion_planner.io.native_result import result_document
    from weldplan import dense_joint_trajectory as dtj
    document = result_document(request, plan, request_sha256="0" * 64, source_revision=None)
    identity = dtj.DensePlanIdentity(plan_id="p", program_id="test1", program_digest="sha256:" + "0" * 64,
                                     manifest_revision=1, plan_revision=1, created_at="2026-10-12T00:00:00Z")
    dense = dtj.plan_from_result(document, identity)
    dtj.encode(dense)  # the daemon's own validation
    nodes = {node["id"]: node for node in request.program["nodes"]}
    kin = verifier.kinematics(request.cell)
    tool = request.cell["frames"]["tool"]
    worst = {}
    for move in plan.connecting_trajectories:
        if move["authored_motion"] != "linear":
            continue
        segment = next(s for s in dense.segments if s.source_id == move["node_id"])
        columns = [dtj.JOINT_ORDER.index(name) for name in kin.axis_names]
        points = np.stack([kin.transforms(q[columns])[tool][:3, 3] for q in _rt_core_cycles(segment, 0.001)])
        start = points[0]
        end = np.asarray(nodes[move["node_id"]]["target"]["xyz_m"], dtype=float)
        length = np.linalg.norm(end - start)
        direction = (end - start) / length
        along = np.clip((points - start) @ direction, 0.0, length)
        worst[move["node_id"]] = float(np.linalg.norm(points - start - along[:, None] * direction, axis=1).max()) * 1000.0
    return worst


def test_taught_lines_are_planned_on_few_knots_and_few_collision_queries(request_test1, monkeypatch):
    moves, calls = _plan(request_test1, monkeypatch)
    lines = [m for m in moves if m["authored_motion"] == "linear"]
    assert len(moves) == 5 and len(lines) == 4
    # Measured 67/67/73/67 knots (2026-10-11); one knot per 0.25 mm was
    # 1683/1853/2785/1690 and cost ~330 k predicate calls.
    assert all(len(m["knots_deg"]) <= 120 for m in lines), [len(m["knots_deg"]) for m in lines]
    # Measured 11 063 for the whole program.
    assert calls["collision_predicate"] <= 25_000
    assert all(m["verdict"]["verdict"] == "PASS" for m in moves)


def test_taught_lines_fly_the_row_speed_on_the_taught_line(request_test1, monkeypatch):
    moves, _ = _plan(request_test1, monkeypatch)
    lines = [m for m in moves if m["authored_motion"] == "linear"]
    for move, length in zip(lines, LINE_LENGTHS_M, strict=True):
        off_mm, flown = _off_line_mm(request_test1, move)
        assert flown == pytest.approx(length, abs=5e-4)
        # The certificate's tube is 0.01 mm (taught.POSITION_TOLERANCE_M);
        # measured worst 0.00011 mm (2026-10-12).
        assert off_mm <= 0.01
        # 25 mm/s cruise between short ramps: never faster than the row's
        # speed, and the ramps cost well under a second at these joint limits.
        assert flown / 0.025 <= move["duration_s"] <= flown / 0.025 + 1.0


@pytest.mark.parametrize("euler", [
    [[0., 0., 0.], [.3, -1.2, 2.], [3.1, .1, -.4]],
    [[.2, .1, -.3], [.2, .1, -.3 + 1e-9], [.2, .1 + 2e-7, -.3]],
])
def test_closed_form_slerp_and_angle_are_scipys(euler):
    from scipy.spatial.transform import Rotation, Slerp
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import rotation_angle, slerp
    keys = Rotation.from_euler("xyz", euler)
    fractions = [0., .4, 1.]
    s = np.linspace(0., 1., 1001)
    ours, theirs = slerp(fractions, keys)(s), Slerp(fractions, keys)(s)
    np.testing.assert_allclose(ours, theirs.as_matrix(), rtol=0, atol=1e-12)
    other = Rotation.from_euler("xyz", [.1, .2, .3]) * theirs
    np.testing.assert_allclose(rotation_angle(ours, other.as_matrix()), (theirs.inv() * other).magnitude(), rtol=0, atol=1e-9)


def test_home_waypoint_speed_is_the_one_the_spreadsheet_shows():
    import re
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import HOME_AS_WAYPOINT
    repo = Path(__file__).resolve().parents[4]
    source = (repo / "offline-programming/v1/ui/src/program-v2/move-defaults.ts").read_text()
    shown = float(re.search(r"export const HOME_WAYPOINT_SPEED_MM_S = ([0-9.]+);", source).group(1))
    assert HOME_AS_WAYPOINT["speed_mm_s"] == shown


def test_faster_rows_plan_shorter_moves_on_the_same_line(request_test1, monkeypatch):
    program = copy.deepcopy(request_test1.program)
    for node in program["nodes"]:
        if node.get("op") == "move":
            node["speed_mm_s"] = 250.0
    fast = dataclasses.replace(request_test1, program=program)
    moves, _ = _plan(fast, monkeypatch)
    lines = [m for m in moves if m["authored_motion"] == "linear"]
    for move, length in zip(lines, LINE_LENGTHS_M, strict=True):
        off_mm, flown = _off_line_mm(fast, move)
        # Measured worst 0.0011 mm at 250 mm/s (2026-10-12).
        assert off_mm <= 0.01
        assert move["duration_s"] >= flown / 0.250
        # Joint velocity and acceleration limits may hold a line below
        # 250 mm/s; they must not hold it near the old 25 mm/s.
        assert move["duration_s"] <= flown / 0.100
    assert sum(m["duration_s"] for m in moves) < 0.25 * 98.5


def test_a_line_the_coarse_knots_cannot_prove_is_refined_until_it_is(request_test1, monkeypatch):
    """The coarse knots are a first try, the certificate the guarantee. With
    the tube shrunk to 0.00005 mm, three of test1's lines fail on 67/73 knots
    (0.00006-0.00015 mm measured, 2026-10-12), are refined to 131/143 and then
    pass -- and the flown spline is inside that tube."""
    from weld_motion_planner.planner.trajectory_optimization.engine import taught
    monkeypatch.setattr(taught, "POSITION_TOLERANCE_M", 5e-8)
    moves, _ = _plan(request_test1, monkeypatch)
    lines = [m for m in moves if m["authored_motion"] == "linear"]
    knots = [len(m["knots_deg"]) for m in lines]
    assert sum(k > 120 for k in knots) >= 1, knots
    for move in lines:
        off_mm, _ = _off_line_mm(request_test1, move)
        assert off_mm <= 0.00005


def test_every_commanded_servo_cycle_stays_within_a_hundredth_of_a_millimetre_of_the_taught_line(request_test1):
    """End to end: the planner certifies its spline, but the robot is
    commanded the dense export's 10 ms rows, which rt-core interpolates every
    1 ms cycle. At 400 mm/s each line's last row used to be clamped to the
    move's end, and rt-core's cubic over it overshot the taught end by up to
    0.0189 mm (2026-10-12). Measured after the uniform export clock: 0.0026 mm
    at worst, beside the line, never past its ends."""
    program = copy.deepcopy(request_test1.program)
    for node in program["nodes"]:
        if node.get("op") == "move":
            node["speed_mm_s"] = 400.0
    fast = dataclasses.replace(request_test1, program=program)
    plan = plan_motion(fast, MotionPlanningOptions(), log=False)
    assert plan.connecting_trajopt_error is None
    worst = _per_cycle_off_segment_mm(fast, plan)
    assert len(worst) == 4
    assert max(worst.values()) <= 0.01, worst


@pytest.mark.parametrize("displacement", [1., 1e-3, 1e-6, 1e-8])
@pytest.mark.parametrize("acceleration", [3., 7., 0.17, 1.234567])
def test_short_constant_tcp_clock_has_distinct_finite_times(displacement, acceleration):
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import _constant_tcp_timing
    from scipy.interpolate import CubicHermiteSpline
    fractions = np.linspace(0., 1., 5)
    knots = (fractions * displacement)[:, None]
    limits = {"j": {"velocity_rad_s": 100., "acceleration_rad_s2": acceleration}}
    q, times, rates, progress, _ = _constant_tcp_timing(knots, fractions, displacement, 100., ["j"], limits)
    assert np.isfinite(times).all() and np.all(np.diff(times) > 1e-12)
    assert times[0] == 0 and times[-1] >= .1 - 1e-14
    np.testing.assert_allclose(q[[0,-1]], knots[[0,-1]], rtol=0, atol=1e-15)
    assert np.all(rates[[0,-1]] == 0)
    # Check the generated wire against its intended ramp, not merely that
    # duplicate times were removed. No waypoint or path endpoint is dropped.
    clock = np.linspace(0., 1., 1001)
    flown = CubicHermiteSpline(times, q, rates)(clock*times[-1])[:,0]
    np.testing.assert_allclose(flown, progress(clock)*displacement, rtol=0, atol=1e-12)


def _split_weld_request(request):
    """Split a verified straight taught leg into two collinear destinations."""
    from scipy.spatial.transform import Rotation, Slerp
    program = copy.deepcopy(request.program)
    first, end = program["nodes"][:2]
    first["weld_transition"] = "start"
    end["weld_transition"] = "stop"
    middle = copy.deepcopy(end)
    middle["id"] = "weld-midpoint"
    middle["weld_transition"] = "none"
    for key in ("xyz_m", "joint_values_rad"):
        middle["target"][key] = ((np.asarray(first["target"][key]) + np.asarray(end["target"][key])) / 2).tolist()
    rotations = Rotation.from_euler("xyz", [first["target"]["rpy_rad"], end["target"]["rpy_rad"]])
    middle["target"]["rpy_rad"] = Slerp([0., 1.], rotations)([.5]).as_euler("xyz")[0].tolist()
    program["nodes"] = [first, middle, end]
    return dataclasses.replace(request, program=program)


def test_taught_weld_keeps_speed_and_waypoint_identity_across_dense_boundary(request_test1):
    from weld_motion_planner.io.native_result import result_document
    from weldplan import dense_joint_trajectory as dtj
    request = _split_weld_request(request_test1)
    plan = plan_motion(request, MotionPlanningOptions(), log=False)
    assert plan.connecting_trajopt_error is None, plan.connecting_trajopt_error
    a, b = plan.connecting_trajectories[-2:]
    assert a["continuous_weld"] == b["continuous_weld"]
    assert a["node_id"] == "weld-midpoint"
    assert a["planned_cruise_mm_s"] == pytest.approx(25., rel=1e-6)
    assert np.linalg.norm(a["knot_velocity_deg_s"][-1]) > .1
    np.testing.assert_allclose(a["knot_velocity_deg_s"][-1], b["knot_velocity_deg_s"][0], atol=1e-12)
    document = result_document(request, plan, request_sha256="0"*64, source_revision=None)
    identity = dtj.DensePlanIdentity(plan_id="w", program_id="test", program_digest="sha256:"+"0"*64,
                                     manifest_revision=1, plan_revision=1, created_at="2026-10-18T00:00:00Z")
    dense = dtj.plan_from_result(document, identity, execution_mode="dry_run")
    dtj.encode(dense)
    sa, sb = dense.segments[-2:]
    assert sa.source_id == a["node_id"] and sb.source_id == b["node_id"]
    np.testing.assert_allclose(sa.q_rad[-1], sb.q_rad[0], atol=1e-12)
    np.testing.assert_allclose(sa.qd_rad_s[-1], sb.qd_rad_s[0], atol=1e-12)
    assert sa.t_s[-1] == pytest.approx(a["duration_s"], abs=1e-9)
    assert not any(sa.torch_on + sb.torch_on)
    kin = verifier.kinematics(request.cell)
    tool = request.cell["frames"]["tool"]
    columns = [dtj.JOINT_ORDER.index(name) for name in kin.axis_names]
    # The actual 1 ms controller interpolation around the shared waypoint
    # must retain the authored 25 mm/s, with no hidden rest or export stretch.
    points = []
    for segment, times in [(sa, np.arange(sa.t_s[-1]-.02, sa.t_s[-1], .001)), (sb, np.arange(0., .0201, .001))]:
        from scipy.interpolate import CubicHermiteSpline
        curve = CubicHermiteSpline(segment.t_s, segment.q_rad, segment.qd_rad_s)
        points.extend(kin.transforms(q[columns])[tool][:3,3] for q in curve(times))
    speeds = np.linalg.norm(np.diff(points,axis=0),axis=1) / .001 * 1000
    np.testing.assert_allclose(speeds, 25., rtol=.002)


def test_taught_weld_refuses_sharp_corner_before_execution(request_test1):
    request = _split_weld_request(request_test1)
    request.program["nodes"][1]["target"]["xyz_m"][0] += .02
    plan = plan_motion(request, MotionPlanningOptions(), log=False)
    assert "not tangent" in str(plan.connecting_trajopt_error)
