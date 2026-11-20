from __future__ import annotations

import json

import numpy as np
import pytest
from weld_motion_planner.verifier.hermite_rates import (
    HermiteRateError,
    _interior_roots,
    judge_hermite_rates,
)
from weld_motion_planner.verifier.hermite_wire import hermite_wire_degrees
from weld_motion_planner.verifier.verify import (
    Geometry,
    Verdict,
    VerifyError,
    check_limits,
    kinematics,
    replace_limit_findings,
    verify,
    verify_free_space,
)

LIMITS = {
    "J1": {
        "velocity_rad_s": 2.0,
        "acceleration_rad_s2": 20.0,
    }
}
PROFILE = {"limits": LIMITS}
CELL = {
    "frames": {"base": "world"},
    "links": [{"name": "world"}, {"name": "rod"}],
    "joints": [{
        "name": "J1",
        "parent": "world",
        "child": "rod",
        "type": "revolute",
        "xyz_m": [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
        "axis_xyz": [0.0, 0.0, 1.0],
        "limits_rad": {"lower": -10.0, "upper": 10.0},
    }],
}


@pytest.fixture
def empty_geometry(monkeypatch: pytest.MonkeyPatch) -> Geometry:
    geometry = Geometry(meshes={}, attached_to={}, local={}, pairs=[])
    monkeypatch.setattr(geometry, "objects", dict)
    return geometry


def test_exact_judge_finds_an_interior_velocity_extremum() -> None:
    result = judge_hermite_rates(
        ["J1"],
        [[0.0], [1.0]],
        [[0.0], [0.0]],
        [0.0, 1.0],
        {"J1": {"velocity_rad_s": 2.0, "acceleration_rad_s2": 7.0}},
    )

    velocity = result["per_joint"][0]["velocity"]
    assert velocity["max_abs_rad_s"] == pytest.approx(1.5)
    assert velocity["u"] == pytest.approx(0.5)


def test_position_root_solver_does_not_erase_small_coefficients() -> None:
    assert _interior_roots(1e-15, 0.0, -5e-16) == pytest.approx(
        [np.sqrt(0.5)]
    )


def test_velocity_vertex_does_not_erase_small_coefficients() -> None:
    scale = 1e-31
    result = judge_hermite_rates(
        ["J1"],
        [[0.0], [-scale / 6.0]],
        [[0.0], [0.0]],
        [0.0, 1.0],
        {
            "J1": {
                "velocity_rad_s": scale / 5.0,
                "acceleration_rad_s2": 1.0,
            }
        },
    )
    velocity = result["per_joint"][0]["velocity"]
    assert result["verdict"] == "FAIL"
    assert velocity["u"] == pytest.approx(0.5)


def test_exact_judge_finds_an_endpoint_acceleration_extremum() -> None:
    result = judge_hermite_rates(
        ["J1"],
        [[0.0], [1.0]],
        [[0.0], [0.0]],
        [0.0, 1.0],
        {"J1": {"velocity_rad_s": 2.0, "acceleration_rad_s2": 5.0}},
    )

    acceleration = result["per_joint"][0]["acceleration"]
    assert result["verdict"] == "FAIL"
    assert acceleration["max_abs_rad_s2"] == pytest.approx(6.0)
    assert acceleration["u"] in (0.0, 1.0)


def test_free_space_cannot_leave_a_joint_limit_between_knots(
    empty_geometry: Geometry,
) -> None:
    """Legal knots do not make an overshooting Hermite position-legal."""
    verdict = verify_free_space(
        {
            **CELL,
            "joints": [{
                **CELL["joints"][0],
                "limits_rad": {"lower": -0.5, "upper": 0.5},
            }],
        },
        empty_geometry,
        np.array([[0.0], [0.0]]),
        motion="transit",
        profile={
            "limits": {
                "J1": {
                    "velocity_rad_s": 5.0,
                    "acceleration_rad_s2": 10.0,
                }
            }
        },
        times_s=np.array([0.0, 1.0]),
        velocities=np.array([[4.0], [-4.0]]),
    )

    finding = next(
        item for item in verdict.limit_violations
        if item["kind"] == "position" and item.get("exact_hermite")
    )
    assert verdict.summary()["verdict"] == "FAIL"
    assert finding["value_rad"] == pytest.approx(1.0)
    assert finding["u"] == pytest.approx(0.5)


def test_json_round_trip_preserves_a_near_limit_verdict() -> None:
    """Four-decimal presentation rounding used to turn this PASS into FAIL."""
    rate = 1.9999999
    wire = hermite_wire_degrees(
        [[0.0], [rate]],
        [[rate], [rate]],
        [0.0, 1.0],
    )
    emitted = json.loads(json.dumps(wire, allow_nan=False))
    result = judge_hermite_rates(
        ["J1"],
        np.deg2rad(emitted["knots_deg"]),
        np.deg2rad(emitted["knot_velocity_deg_s"]),
        emitted["times_s"],
        {"J1": {"velocity_rad_s": 2.0, "acceleration_rad_s2": 20.0}},
    )

    legacy = {
        "knots_deg": np.round(wire["knots_deg"], 4),
        "knot_velocity_deg_s": np.round(wire["knot_velocity_deg_s"], 4),
        "times_s": np.round(wire["times_s"], 4),
    }
    old_result = judge_hermite_rates(
        ["J1"],
        np.deg2rad(legacy["knots_deg"]),
        np.deg2rad(legacy["knot_velocity_deg_s"]),
        legacy["times_s"],
        {"J1": {"velocity_rad_s": 2.0, "acceleration_rad_s2": 20.0}},
    )

    assert result["verdict"] == "PASS"
    assert old_result["verdict"] == "FAIL"


def test_free_space_cannot_promote_a_rate_invalid_hermite_curve(
    empty_geometry: Geometry,
) -> None:
    """The exact counterexample that rejected the predecessor repository's PR #338 as written."""
    knots = np.array([[0.0], [1.0]])
    times = np.array([0.0, 1.0])
    velocities = np.array([[3.0], [3.0]])
    secant_findings, missing = check_limits(
        kinematics(CELL), CELL, PROFILE, knots, times
    )
    verdict = verify_free_space(
        CELL,
        empty_geometry,
        knots,
        motion="transit",
        profile=PROFILE,
        times_s=times,
        velocities=velocities,
    )

    assert secant_findings == [] and missing == []
    assert verdict.summary()["verdict"] == "FAIL"
    assert verdict.travel == {"status": "not_applicable", "motion": "transit"}
    assert any(
        finding["kind"] == "velocity" and finding["exact_hermite"]
        for finding in verdict.limit_violations
    )


def test_free_space_can_pass_when_every_applicable_check_passes(
    empty_geometry: Geometry,
) -> None:
    verdict = verify_free_space(
        CELL,
        empty_geometry,
        np.array([[0.0], [1.0]]),
        motion="approach",
        profile=PROFILE,
        times_s=np.array([0.0, 1.0]),
        velocities=np.array([[1.0], [1.0]]),
    )

    assert verdict.summary()["verdict"] == "PASS"
    assert verdict.travel == {"status": "not_applicable", "motion": "approach"}


@pytest.mark.parametrize("rate", [0.5, 3.0])
def test_partial_rate_profile_retains_refusal_and_known_findings(
    empty_geometry: Geometry, rate: float,
) -> None:
    verdict = verify_free_space(
        CELL, empty_geometry, np.array([[0.0], [rate]]),
        motion="transit", profile={"limits": {"J1": {"velocity_rad_s": 2.0}}},
        times_s=np.array([0.0, 1.0]), velocities=np.array([[rate], [rate]]),
    )
    assert "J1: no acceleration limit" in verdict.unverifiable
    assert verdict.summary()["verdict"] == ("REFUSED" if rate < 2.0 else "FAIL")
    assert bool(verdict.limit_violations) == (rate > 2.0)


def test_partial_rate_profile_does_not_hide_invalid_curve() -> None:
    with pytest.raises(VerifyError, match="strictly increasing"):
        check_limits(
            kinematics(CELL), CELL,
            {"limits": {"J1": {"velocity_rad_s": 2.0}}},
            np.array([[0.0], [1.0]]), np.array([0.0, 0.0]),
            knot_velocities=np.array([[1.0], [1.0]]),
        )


def test_missing_axis_rates_preserve_other_axis_findings() -> None:
    kin = kinematics(CELL)
    kin.axis_names.append("J2")
    findings, missing = check_limits(
        kin, CELL, PROFILE, np.array([[0.0, 0.0], [3.0, 0.0]]),
        np.array([0.0, 1.0]),
        knot_velocities=np.array([[3.0, 0.0], [3.0, 0.0]]),
    )
    assert any(f["axis"] == "J1" and f["kind"] == "velocity" for f in findings)
    assert "J2: no velocity limit" in missing
    assert "J2: no acceleration limit" in missing


def test_acceleration_only_profile_still_checks_hermite_extrema() -> None:
    findings, missing = check_limits(
        kinematics(CELL), CELL,
        {"limits": {"J1": {"acceleration_rad_s2": 5.0}}},
        np.array([[0.0], [1.0]]), np.array([0.0, 1.0]),
        knot_velocities=np.array([[0.0], [0.0]]),
    )
    assert missing == ["J1: no velocity limit"]
    assert any(f["kind"] == "acceleration" and f["exact_hermite"] for f in findings)


@pytest.mark.parametrize("limit", [0.0, -1.0, np.nan, np.inf])
def test_partial_profile_does_not_accept_invalid_stated_limits(limit: float) -> None:
    with pytest.raises(VerifyError, match="finite and positive"):
        check_limits(
            kinematics(CELL), CELL, {"limits": {"J1": {"velocity_rad_s": limit}}},
            np.array([[0.0], [1.0]]), np.array([0.0, 1.0]),
            knot_velocities=np.array([[1.0], [1.0]]),
        )


def test_weld_default_still_refuses_missing_travel_inputs(
    empty_geometry: Geometry,
) -> None:
    verdict = verify(
        CELL,
        empty_geometry,
        np.array([[0.0], [1.0]]),
        profile=PROFILE,
        times_s=np.array([0.0, 1.0]),
        velocities=np.array([[1.0], [1.0]]),
    )

    assert verdict.summary()["verdict"] == "REFUSED"
    assert any("travel mobility" in reason for reason in verdict.unverifiable)


def test_exact_seam_rates_replace_secants_in_the_native_verdict() -> None:
    """M5's exact sidecar finding must reach the verdict rendered by the UI."""
    exact = {
        "axis": "J1",
        "kind": "velocity",
        "exact_hermite": True,
    }
    generic = Verdict(
        passed=True,
        collisions=[],
        limit_violations=[
            {"axis": "J1", "kind": "velocity", "value": 1.0},
            {"kind": "travel_speed", "value": 0.0},
        ],
        unverifiable=["generic gap"],
        travel={"status": "checked"},
    )

    composed = replace_limit_findings(generic, [exact], ["exact gap"])

    assert composed.summary()["verdict"] == "FAIL"
    assert composed.limit_violations == [
        exact,
        {"kind": "travel_speed", "value": 0.0},
    ]
    assert composed.unverifiable == ["generic gap", "exact gap"]
    assert composed.travel == {"status": "checked"}


def test_unknown_free_space_motion_fails_closed(empty_geometry: Geometry) -> None:
    with pytest.raises(VerifyError, match="unknown free-space motion"):
        verify_free_space(
            CELL,
            empty_geometry,
            np.array([[0.0], [1.0]]),
            motion="weld",  # type: ignore[arg-type]
            profile=PROFILE,
            times_s=np.array([0.0, 1.0]),
            velocities=np.array([[1.0], [1.0]]),
        )


@pytest.mark.parametrize(
    ("knots", "velocities", "times", "limits", "message"),
    [
        ([[0.0], [1.0]], [[0.0], [0.0]], [0.0, 0.0], LIMITS, "strictly increasing"),
        ([[0.0], [np.nan]], [[0.0], [0.0]], [0.0, 1.0], LIMITS, "nonfinite"),
        ([[0.0], [1.0]], [[0.0], [0.0]], [0.0, 1.0], {}, "missing declared"),
    ],
)
def test_exact_judge_refuses_unjudgeable_inputs(
    knots: list[list[float]],
    velocities: list[list[float]],
    times: list[float],
    limits: dict[str, dict[str, float]],
    message: str,
) -> None:
    with pytest.raises(HermiteRateError, match=message):
        judge_hermite_rates(["J1"], knots, velocities, times, limits)


def test_taught_joint_move_uses_real_rate_and_position_verifier(empty_geometry):
    from types import SimpleNamespace
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import plan_taught
    descriptor = {**CELL, "frames": {"base": "world", "tool": "rod"}}
    cell = SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(limits=LIMITS, held={}),
                           arm=SimpleNamespace(chain=fk.build(descriptor, ["rod"]), device="cpu"))
    node = {"id": "p1", "motion": "joint", "speed_scale": .1, "speed_mm_s": 25.,
            "target": {"joint_names": ["J1"], "joint_values_rad": [1.]}}
    move, end = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert move["verdict"]["verdict"] == "PASS"
    assert end.tolist() == [1.]
    assert move["duration_s"] >= 7.5
    assert move["knot_velocity_deg_s"] == [[0.], [0.]]
    node["capture"] = {"machine_planning_calibration_sha256": "sha256:" + "a"*64}
    with pytest.raises(ValueError, match="calibration changed"):
        plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    del node["capture"]
    node["target"]["joint_values_rad"] = [11.]
    move, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert move["verdict"]["verdict"] == "FAIL"


@pytest.fixture
def taught_connections(monkeypatch, empty_geometry):
    """Isolate node ordering; taught moves still use the real wire verifier."""
    from types import SimpleNamespace
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace, taught
    descriptor = {**CELL, "frames": {"base": "world", "tool": "rod"}}
    cell = SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(
        limits=LIMITS, held={}, reset_pose={"J1": 0.}),
        arm=SimpleNamespace(chain=fk.build(descriptor, ["rod"]), device="cpu"))
    context = SimpleNamespace(device="cpu", planner_collision_model=object(),
                              verifier_collision_model=empty_geometry, progress=None)
    specs = []
    def plan_moves(request, geometry, device, **kwargs):
        assert geometry is context.planner_collision_model
        assert kwargs["verifier_collision_model"] is empty_geometry
        assert kwargs["verify_margin_m"] == .002
        for spec in kwargs["specs"]:
            assert spec.pin_start and spec.pin_end
            np.testing.assert_array_equal(spec.start_rate, [0.])
            np.testing.assert_array_equal(spec.end_rate, [0.])
        specs.extend(kwargs["specs"])
        return kwargs["specs"]
    monkeypatch.setattr(freespace, "plan_moves", plan_moves)
    monkeypatch.setattr(freespace, "move_json", lambda spec: {
        "kind": spec.kind, "knots_deg": np.rad2deg([spec.start, spec.end]).tolist()})
    monkeypatch.setattr(freespace, "standoff_batch", lambda request, ends, device, **kwargs:
                        [[q + .1, q + .2] for q in ends])
    reports = [SimpleNamespace(seam_id="a", knots=np.array([[.5], [.6]])),
               SimpleNamespace(seam_id="b", knots=np.array([[1.5], [1.6]]))]
    def connect(nodes, start_q=None):
        binding = {"model_id": "test", "robot_description_sha256": "sha256:model", "machine_planning_calibration_sha256": "sha256:calibration"}
        start = None if start_q is None else {"schema": "rosie.measured-planning-start.v1", "joint_names": ["J1"], "joint_values_rad": [start_q], "robot_cell": binding, "coordinate_identity": "config:1:32", "observation_identity": "adapter:daemon", "observed_time_ns": "100"}
        return taught.connect_taught_program(
            SimpleNamespace(cell={**descriptor, "axes": ["J1"], "robot_cell": binding}, program={"nodes": nodes}, start_state=start), reports,
            SimpleNamespace(backend=None, verify_margin_m=.002), context, cell)
    return connect, specs


@pytest.mark.parametrize("next_op", ["move", "approach"])
def test_internal_retract_keeps_local_continuous_handoff(taught_connections, next_op):
    connect, specs = taught_connections
    nodes = [{"id": "a-in", "op": "approach"},
             {"id": "a-weld", "op": "weld", "geometry": {"id": "a"}},
             {"id": "a-out", "op": "retract"}, {"id": "pause", "op": "dwell"},
             {"id": "disabled-home", "op": "home", "enabled": False}]
    if next_op == "move":
        nodes.append({"id": "waypoint", "op": "move", "motion": "joint", "speed_scale": 1.,
                      "speed_mm_s": 25., "target": {"joint_names": ["J1"], "joint_values_rad": [1.]}})
    nodes += [{"id": "b-in", "op": "approach"},
              {"id": "b-weld", "op": "weld", "geometry": {"id": "b"}},
              {"id": "b-out", "op": "retract"}]
    entries = connect(nodes)
    emitted = {entry["node_id"]: entry for entry in entries}
    # The retract keeps its own segment and ends at the nearest available
    # standoff, not home. Events and disabled nodes do not change the handoff.
    np.testing.assert_allclose(specs[1].start, [.6])
    np.testing.assert_allclose(specs[1].end, [.7])
    following = emitted["waypoint" if next_op == "move" else "b-in"]
    np.testing.assert_allclose(emitted["a-out"]["knots_deg"][-1], following["knots_deg"][0])
    if next_op == "move":
        assert following["verdict"]["verdict"] == "PASS"
        np.testing.assert_allclose(following["knots_deg"][-1], emitted["b-in"]["knots_deg"][0])
    np.testing.assert_allclose(specs[-1].end, [0.])  # Terminal compatibility.


@pytest.mark.parametrize("tail", [[], [{"id": "home", "op": "home"}],
                                      [{"id": "off", "op": "move", "enabled": False}]])
def test_terminal_retract_still_returns_home(taught_connections, tail):
    connect, specs = taught_connections
    connect([{"id": "in", "op": "approach"},
             {"id": "weld", "op": "weld", "geometry": {"id": "a"}},
             {"id": "out", "op": "retract"}, {"id": "pause", "op": "dwell"}, *tail])
    assert len(specs) == 2
    np.testing.assert_allclose(specs[-1].start, [.6])
    np.testing.assert_allclose(specs[-1].end, [0.])


def test_internal_retract_without_reachable_standoff_refuses(taught_connections, monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace
    connect, specs = taught_connections
    monkeypatch.setattr(freespace, "standoff_batch", lambda request, ends, device, **kwargs:
                        [[] for _ in ends])
    with pytest.raises(ValueError, match="Retract out has no reachable departure standoff"):
        connect([{"id": "in", "op": "approach"},
                 {"id": "weld", "op": "weld", "geometry": {"id": "a"}},
                 {"id": "out", "op": "retract"}, {"id": "next", "op": "move"}])
    assert len(specs) == 1  # No fallback to home or a zero-length retreat.


def test_taught_circular_geometry_hits_via_and_preserves_orientation():
    from scipy.spatial.transform import Rotation
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import cartesian_path
    target = {"xyz_m": [-1., 0., 0.], "rpy_rad": [0., 0., 1.]}
    via = {"xyz_m": [0., 1., 0.], "rpy_rad": [0., 0., .5]}
    position, rotation, length, _ = cartesian_path(np.array([1., 0., 0.]), np.eye(3), target, via)
    np.testing.assert_allclose(position(np.array([0., .5, 1.])), [[1,0,0],[0,1,0],[-1,0,0]], atol=1e-12)
    np.testing.assert_allclose(Rotation.from_matrix(rotation([0., .5, 1.])).as_euler("xyz")[:,2], [0,.5,1])
    assert length == pytest.approx(np.pi)
    with pytest.raises(ValueError, match="non-collinear"):
        cartesian_path(np.array([1.,0.,0.]), np.eye(3), target, {**via, "xyz_m": [0.,0.,0.]})


@pytest.mark.parametrize("motion", ["linear", "circular"])
def test_taught_cartesian_curve_is_ik_solved_and_continuously_verified(empty_geometry, motion):
    from types import SimpleNamespace
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import plan_taught
    radius = 1. if motion == "circular" else 0.
    descriptor = {**CELL, "frames": {"base": "world", "tool": "tip"},
                  "links": CELL["links"] + [{"name": "tip"}],
                  "joints": CELL["joints"] + [{"name": "tool", "type": "fixed", "parent": "rod", "child": "tip", "xyz_m": [radius,0.,0.], "rpy_rad": [0.,0.,0.]}]}
    cell = SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(limits=LIMITS, held={}),
                           arm=SimpleNamespace(chain=fk.build(descriptor, ["tip"]), device="cpu"))
    def pose(q):
        return {"joint_names": ["J1"], "joint_values_rad": [q], "xyz_m": [radius*np.cos(q),radius*np.sin(q),0.], "rpy_rad": [0.,0.,q]}
    node = {"id": "p1", "motion": motion, "speed_scale": .1, "speed_mm_s": 25., "target": pose(.2), "via": pose(.1)}
    move, end = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert move["verdict"]["verdict"] == "PASS"
    assert end[0] == pytest.approx(.2, abs=1e-5)
    assert len(move["knots_deg"]) > 2
    if motion == "circular":
        from scipy.interpolate import CubicHermiteSpline
        curve = CubicHermiteSpline(move["times_s"], np.deg2rad(move["knots_deg"]), np.deg2rad(move["knot_velocity_deg_s"]))
        # One metre radius: angular speed is also TCP speed in metres/s.
        np.testing.assert_allclose(curve(np.array([.3, .5, .7]) * move["duration_s"], 1), .025, rtol=.002)
        np.testing.assert_allclose(curve([0., move["duration_s"]], 1), 0., atol=1e-10)
        node["constant_tcp_speed"] = False
        smooth, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
        assert smooth["duration_s"] > move["duration_s"]
        smooth_curve = CubicHermiteSpline(smooth["times_s"], np.deg2rad(smooth["knots_deg"]), np.deg2rad(smooth["knot_velocity_deg_s"]))
        assert smooth_curve(smooth["duration_s"] * .5, 1)[0] > smooth_curve(smooth["duration_s"] * .25, 1)[0] * 1.2
    node["target"]["xyz_m"][2] = .1
    with pytest.raises(ValueError, match="unreachable"):
        plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)


@pytest.mark.parametrize("where", ["knot", "between_knots"])
def test_taught_certificate_refuses_a_path_it_cannot_prove_within_a_hundredth_of_a_millimetre(where):
    """One revolute joint, 1 m to the TCP: a joint radian is a TCP metre, so a
    perturbation is its own deviation in metres. The exact path is a cubic in
    phase (the spline reproduces it, error 0); a 5 um bulge must pass and a
    20 um one must be refused -- at a knot, and between knots where only the
    continuous bound can see it."""
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine import taught
    descriptor = {**CELL, "frames": {"base": "world", "tool": "tip"},
                  "links": CELL["links"] + [{"name": "tip"}],
                  "joints": CELL["joints"] + [{"name": "tool", "type": "fixed", "parent": "rod", "child": "tip",
                                               "xyz_m": [1., 0., 0.], "rpy_rad": [0., 0., 0.]}]}
    chain = fk.build(descriptor, ["tip"])
    link = chain.link_names.index("tip")
    position, rotation, length, angular = taught.cartesian_path(
        np.array([1., 0., 0.]), np.eye(3), {"xyz_m": [np.cos(.2), np.sin(.2), 0.], "rpy_rad": [0., 0., .2]},
        {"xyz_m": [np.cos(.1), np.sin(.1), 0.], "rpy_rad": [0., 0., .1]})
    assert taught.POSITION_TOLERANCE_M == 1e-5
    u = np.linspace(0., 1., 65)
    h = u[1] - u[0]

    def certify(deviation_m):
        knots = (.2 * u * u * (3 - 2 * u))[:, None]
        tangents = (.2 * 6 * u * (1 - u))[:, None]
        if where == "knot":
            knots[32] += deviation_m
        else:
            # A tangent change of d bulges the interval's middle by h d / 8.
            tangents[32] += 8 * deviation_m / h
        return taught._certify_pose_path(chain, link, [0], knots, tangents, u, position, rotation, length, angular, "cpu")

    assert certify(5e-6) > 0
    with pytest.raises(taught.TrackingRefused, match="0.01 mm"):
        certify(2e-5)


def test_taught_joint_cartesian_edits_and_tcp_speed_limit(empty_geometry):
    from types import SimpleNamespace
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import plan_taught
    descriptor = {**CELL, "frames": {"base": "world", "tool": "tip"},
                  "links": CELL["links"] + [{"name": "tip"}],
                  "joints": CELL["joints"] + [{"name": "tool", "type": "fixed", "parent": "rod", "child": "tip", "xyz_m": [1.,0.,0.], "rpy_rad": [0.,0.,0.]}]}
    cell = SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(limits=LIMITS, held={}),
                           arm=SimpleNamespace(chain=fk.build(descriptor, ["tip"]), device="cpu"))
    node = {"id": "p1", "motion": "joint", "target_space": "cartesian", "speed_scale": 1., "speed_mm_s": 25.,
            "target": {"joint_names": ["J1"], "joint_values_rad": [0.],
                       "xyz_m": [np.cos(.2),np.sin(.2),0.], "rpy_rad": [0.,0.,.2]}}
    move, end = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert move["verdict"]["verdict"] == "PASS"
    assert end[0] == pytest.approx(.2, abs=1e-5)  # Must not execute the unchanged recorded seed.
    assert len(move["knots_deg"]) == 2  # Still MoveJ, not an implicit MoveL.
    assert move["duration_s"] >= 1.5 * abs(end[0]) / .025
    node["speed_mm_s"] = 50.
    faster, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert faster["duration_s"] == pytest.approx(move["duration_s"] / 2)
    node["target"]["xyz_m"][2] = .1
    with pytest.raises(ValueError, match="unreachable"):
        plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    node["motion"] = "circular"
    with pytest.raises(ValueError, match="needs a via pose"):
        plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)


def test_taught_acceleration_is_independent_and_preserves_the_joint_path(empty_geometry):
    from types import SimpleNamespace
    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import plan_taught
    descriptor = {**CELL, "frames": {"base": "world", "tool": "rod"}}
    cell = SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(limits=LIMITS, held={}),
                           arm=SimpleNamespace(chain=fk.build(descriptor, ["rod"]), device="cpu"))
    node = {"id": "p1", "motion": "joint", "speed_scale": 1., "speed_mm_s": 1000.,
            "target": {"joint_names": ["J1"], "joint_values_rad": [1.]}}
    original, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    node["acceleration_scale"] = .01
    slower, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert slower["verdict"]["verdict"] == "PASS"
    assert slower["duration_s"] > original["duration_s"]
    assert slower["knots_deg"] == original["knots_deg"]
    duration = slower["duration_s"]
    assert 6 / duration**2 <= LIMITS["J1"]["acceleration_rad_s2"] * .01
    assert 1.5 / duration <= LIMITS["J1"]["velocity_rad_s"]
    node["acceleration_scale"] = 1.
    restored, _ = plan_taught(SimpleNamespace(cell=descriptor), node, np.array([0.]), cell, empty_geometry, 0.)
    assert restored == original


@pytest.mark.parametrize("distance,speed", [(1., .1), (.001, 10.)])
def test_constant_tcp_clock_cruises_or_uses_short_move_ramps_without_exceeding_limits(distance, speed):
    from scipy.interpolate import CubicHermiteSpline
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import _constant_tcp_timing
    from weld_motion_planner.planner.trajectory_optimization.engine.freespace import retime_rest_to_rest_wire
    fractions = np.linspace(0., 1., 65)
    knots, times, rates, progress, bound = _constant_tcp_timing(
        fractions[:, None] * distance, fractions, distance, speed, ["J1"], LIMITS)
    times, rates, _ = retime_rest_to_rest_wire(["J1"], knots, rates, times, LIMITS)
    assert judge_hermite_rates(["J1"], knots, rates, times, LIMITS)["verdict"] == "PASS"
    curve = CubicHermiteSpline(times, knots, rates)
    u = np.linspace(0., 1., 1001)
    np.testing.assert_allclose(curve(u*times[-1])[:, 0], progress(u)*distance, atol=1e-10)
    assert np.max(curve(u*times[-1], 1)) <= speed * (1+1e-6)
    np.testing.assert_allclose(curve([0., times[-1]], 1), 0., atol=1e-10)
    if distance == 1.:
        np.testing.assert_allclose(curve(np.array([.3, .5, .7])*times[-1], 1), speed, rtol=1e-5)
    else:
        assert np.max(curve(u*times[-1], 1)) < speed


@pytest.mark.parametrize("start_q", [0., .2])
def test_taught_program_starts_at_explicit_measurement_not_reset(taught_connections, start_q):
    # Reuse the real MoveJ + continuous verifier fixture; only the supplied
    # request observation changes. Reset stays zero throughout.
    connect, _ = taught_connections
    node = {"id": "bounded-out", "op": "move", "motion": "joint", "speed_scale": .01,
            "speed_mm_s": 1., "target": {"joint_names": ["J1"], "joint_values_rad": [start_q + .001],
            "tcp_pose": {"xyz_m": [0.,0.,0.], "rpy_rad": [0.,0.,0.]}}}
    result = connect([node], start_q=start_q)
    np.testing.assert_allclose(np.deg2rad(result[0]["knots_deg"][0]), [start_q], atol=1e-12, rtol=0)
    np.testing.assert_allclose(np.deg2rad(result[0]["knots_deg"][-1]), [start_q+.001], atol=1e-12, rtol=0)
    assert result[0]["verdict"]["verdict"] == "PASS"
