"""Every non-I/O node type plans from the robot's measured pose (2026-10-11).

The owner's rule: a general purpose robot is programmed with any waypoint,
Home, weld block or event, and planning from where the robot stands must take
all of them. Before this the measured-start guard accepted taught moves and
dwells only, and a HOME added ahead of four taught waypoints on the 1430 cell's Deck
was refused as "Planning failed".

What is proven here, in absolute radians on a one-joint arm whose TCP sits
1 m from the axis (so 1 rad is 1 m of TCP arc):

- the first emitted leg starts at the measured joint value exactly (1e-12),
  never at the reset pose;
- each leg starts where the previous node ended, and a weld is spliced at
  its own first knot and left at its last (the verified trajectory is not
  touched);
- a Home is a planned transit to the reset pose, from wherever the arm is;
- an `io` node is refused by name on every path, never dropped;
- the module routes a measured-start weld-only program through the in-order
  connector, with the roadmap and the winding walk seeded from the measured
  pose, and leaves the model-reset weld-only path as it was.

The free-space solve is stubbed (it is the batched B-spline solver, covered
elsewhere); taught moves run the real IK, timing and continuous verifier.
"""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from weld_motion_planner.verifier.verify import Geometry

LIMITS = {"J1": {"velocity_rad_s": 2.0, "acceleration_rad_s2": 20.0}}
# The TCP 1 m off the axis, so MoveC's arc is reachable; MoveL runs with the
# TCP on the axis (a pure reorientation), the only straight line one revolute
# joint can follow.
RADIUS_M = 1.0


def _descriptor(radius):
    return {
        "frames": {"base": "world", "tool": "tip"},
        "links": [{"name": "world"}, {"name": "rod"}, {"name": "tip"}],
        "joints": [
            {"name": "J1", "parent": "world", "child": "rod", "type": "revolute", "xyz_m": [0., 0., 0.],
             "rpy_rad": [0., 0., 0.], "axis_xyz": [0., 0., 1.], "limits_rad": {"lower": -10., "upper": 10.}},
            {"name": "tool", "type": "fixed", "parent": "rod", "child": "tip", "xyz_m": [radius, 0., 0.],
             "rpy_rad": [0., 0., 0.]},
        ],
    }
BINDING = {"model_id": "test", "robot_description_sha256": "sha256:model",
           "machine_planning_calibration_sha256": "sha256:calibration"}
HOME = 0.0
MEASURED = 0.3
# Weld reports as M5 hands them over: (first knot, last knot) in radians.
WELDS = {"a": (.5, .6), "b": (1.5, 1.6)}
STANDOFF_RAD = .05


def _pose(q, radius=RADIUS_M):
    return {"joint_names": ["J1"], "joint_values_rad": [q],
            "xyz_m": [radius * np.cos(q), radius * np.sin(q), 0.], "rpy_rad": [0., 0., q]}


def _move(node_id, motion, q, via=None, radius=RADIUS_M):
    node = {"id": node_id, "op": "move", "motion": motion, "speed_scale": .5, "speed_mm_s": 100.,
            "target": _pose(q, radius)}
    if via is not None:
        node["via"] = _pose(via, radius)
    return node


def _start(q):
    return {"schema": "rosie.measured-planning-start.v1", "joint_names": ["J1"], "joint_values_rad": [q],
            "robot_cell": BINDING, "coordinate_identity": "config:1:32", "observation_identity": "adapter:daemon",
            "observed_time_ns": "100"}


@pytest.fixture
def planner(monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace
    from weld_motion_planner.robot_cell.robot import fk
    geometry = Geometry(meshes={}, attached_to={}, local={}, pairs=[])
    monkeypatch.setattr(geometry, "objects", dict)
    def cell_of(radius):
        return SimpleNamespace(moved_axes=["J1"], profile=SimpleNamespace(limits=LIMITS, held={}, reset_pose={"J1": HOME}),
                               arm=SimpleNamespace(chain=fk.build(_descriptor(radius), ["tip"]), device="cpu"))
    cell = cell_of(RADIUS_M)
    context = SimpleNamespace(device="cpu", planner_collision_model=object(), verifier_collision_model=geometry,
                              progress=None, cell=cell)
    specs = []

    def plan_moves(request, model, device, **kwargs):
        for spec in kwargs["specs"]:
            assert spec.pin_start and spec.pin_end
        specs.extend(kwargs["specs"])
        return kwargs["specs"]
    monkeypatch.setattr(freespace, "plan_moves", plan_moves)
    monkeypatch.setattr(freespace, "move_json", lambda spec: {
        "kind": spec.kind, "knots_deg": np.rad2deg([spec.start, spec.end]).tolist(),
        "verdict": {"verdict": "PASS"}})
    monkeypatch.setattr(freespace, "standoff_batch", lambda request, ends, device, **kwargs:
                        [[q + STANDOFF_RAD] for q in ends])
    reports = [SimpleNamespace(seam_id=k, knots=np.array([[a], [b]])) for k, (a, b) in WELDS.items()]

    def request(nodes, start_q, radius=RADIUS_M):
        return SimpleNamespace(cell={**_descriptor(radius), "axes": ["J1"], "robot_cell": BINDING},
                               program={"nodes": nodes}, start_state=None if start_q is None else _start(start_q))

    def connect(nodes, start_q=MEASURED, roadmap=None, radius=RADIUS_M):
        from weld_motion_planner.planner.trajectory_optimization.engine import taught
        return taught.connect_taught_program(request(nodes, start_q, radius), reports,
                                             SimpleNamespace(backend=None, verify_margin_m=.002), context,
                                             cell_of(radius), roadmap=roadmap)
    return SimpleNamespace(connect=connect, specs=specs, request=request, context=context, reports=reports)


def _weld(node_id, seam):
    return {"id": node_id, "op": "weld", "geometry": {"id": seam}}


@pytest.mark.parametrize("entry", [True, False])
def test_unreachable_line_identifies_its_actual_authored_destination(planner, entry):
    # A one-axis arm cannot follow the interior of this chord, although both
    # taught endpoints are reachable. Diagnose the failing leg, not the start
    # of the entire program; disabled/event rows retain their authored indices.
    nodes = [] if entry else [
        _move("first", "joint", .5),
        {"id": "disabled", "op": "move", "enabled": False},
        {"id": "pause", "op": "dwell"},
    ]
    failing = {**_move("failing-line", "linear", .8), "label": "P2"}
    nodes.append(failing)
    with pytest.raises(ValueError) as caught:
        planner.connect(nodes)
    assert str(caught.value) == (
        f"MoveL to P2 (program step {1 if entry else 4}, node failing-line): "
        "Taught Cartesian path has unreachable poses")
    assert isinstance(caught.value.__cause__, ValueError)
    assert str(caught.value.__cause__) == "Taught Cartesian path has unreachable poses"


@pytest.mark.parametrize("motion,expected", [
    ("joint", "MoveJ TCP destination is unreachable"),
    ("circular", "MoveC unlabelled needs a via pose; use Record via"),
])
def test_other_taught_refusals_retain_motion_and_unlabelled_node(planner, motion, expected):
    node = _move("unlabelled", motion, .8)
    if motion == "joint":
        node["target_space"] = "cartesian"
        node["target"]["xyz_m"] = [2., 0., 0.]  # Beyond this one-metre arm.
    with pytest.raises(ValueError) as caught:
        planner.connect([node])
    name = "MoveJ" if motion == "joint" else "MoveC"
    assert str(caught.value) == f"{name} (program step 1, node unlabelled): {expected}"
    assert str(caught.value.__cause__) == expected


def test_tracking_refusal_keeps_reason_after_all_existing_refinement_attempts(planner, monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization.engine import taught
    reason = "Taught Cartesian path exceeds the 0.01 mm / 0.001 rad tracking tolerance: 0.13064 mm, 0.000262 rad"
    refusal = taught.TrackingRefused(reason)
    attempts = []
    def refuse(*args, **kwargs):
        attempts.append(len(args[3]))
        raise refusal
    monkeypatch.setattr(taught, "_certify_pose_path", refuse)
    node = {**_move("tracking-line", "linear", .6, radius=0.), "label": "P7"}
    with pytest.raises(ValueError) as caught:
        planner.connect([node], radius=0.)
    assert attempts == taught.knot_counts(0., .6-MEASURED)
    assert len(attempts) > 1
    assert caught.value.__cause__ is refusal
    assert str(caught.value) == f"MoveL to P7 (program step 1, node tracking-line): {reason}"


MIXES = {
    # The Deck program refused on the 1430 cell at 15:45 (HOME, then waypoints).
    "taught+home": [{"id": "home-in", "op": "home"}, _move("P1", "joint", .5), {"id": "pause", "op": "dwell"},
                    _move("P2", "circular", .8, via=.7), {"id": "home-out", "op": "home"}],
    "MoveL+home": [{"id": "home-in", "op": "home"}, _move("P1", "linear", .5, radius=0.),
                   _move("P2", "joint", .7, radius=0.), {"id": "home-out", "op": "home"}],
    "taught+weld block": [_move("P1", "joint", .4), {"id": "in", "op": "approach"}, _weld("wa", "a"),
                          {"id": "between", "op": "transit"}, _weld("wb", "b"), {"id": "out", "op": "retract"},
                          _move("P2", "joint", 1.0)],
    "weld-only with home": [{"id": "home-in", "op": "home"}, {"id": "in", "op": "approach"}, _weld("wa", "a"),
                            {"id": "between", "op": "transit"}, _weld("wb", "b"), {"id": "out", "op": "retract"},
                            {"id": "home-out", "op": "home"}],
    "weld-only": [{"id": "in", "op": "approach"}, _weld("wa", "a"), {"id": "out", "op": "retract"}],
    "home only": [{"id": "home", "op": "home"}],
}


@pytest.mark.parametrize("mix", MIXES)
def test_every_non_io_node_plans_from_the_measured_pose(planner, mix):
    nodes = MIXES[mix]
    from weldplan.measured_start import validate
    assert validate(_start(MEASURED), {"axes": ["J1"], "robot_cell": BINDING}, {"nodes": nodes}) is not None
    radius = 0. if mix.startswith("MoveL") else RADIUS_M
    entries = {entry["node_id"]: entry for entry in planner.connect(nodes, radius=radius)}
    here = MEASURED
    first = True
    for index, node in enumerate(nodes):
        if node["op"] == "dwell":
            continue
        if node["op"] == "weld":
            a, b = WELDS[node["geometry"]["id"]]
            assert here == pytest.approx(a, abs=1e-12), f"{node['id']} spliced off its first knot"
            here = b
            continue
        entry = entries.get(node["id"])
        if node["op"] == "home" and entry is None:
            assert here == HOME, "a Home is skipped only when the arm is already at reset"
            continue
        knots = np.deg2rad(entry["knots_deg"])[:, 0]
        assert knots[0] == pytest.approx(here, abs=1e-12), f"{node['id']} does not leave where the arm is"
        if first:
            assert abs(knots[0] - MEASURED) <= 1e-12 and abs(knots[0] - HOME) > .1, "first leg left from reset"
            first = False
        assert entry["verdict"]["verdict"] == "PASS"
        here = knots[-1]
        if node["op"] == "home":
            assert here == pytest.approx(HOME, abs=1e-12)
        if node["op"] == "move":
            assert here == pytest.approx(node["target"]["joint_values_rad"][0], abs=1e-5)
        if node["op"] in ("approach", "transit"):
            following = next(n for n in nodes[index + 1:] if n["op"] == "weld")
            assert here == pytest.approx(WELDS[following["geometry"]["id"]][0], abs=1e-12)
    assert not first, "nothing was planned"


def test_measured_start_home_is_a_planned_transit_with_no_weld_standoff(planner):
    planner.connect([{"id": "home", "op": "home"}, _move("P1", "joint", .5)])
    (home,) = planner.specs
    assert home.kind == "transit" and home.via == []
    np.testing.assert_allclose(home.start, [MEASURED], atol=1e-12, rtol=0)
    np.testing.assert_allclose(home.end, [HOME], atol=1e-12, rtol=0)


@pytest.mark.parametrize("mix", ["taught+home", "MoveL+home", "home only"])
def test_without_a_planner_model_home_is_a_verified_waypoint_to_reset(planner, mix):
    # The CPU planner (motion-cpu, example-relay) builds no planner collision model
    # for a weld-free program: Home is a MoveJ to the reset pose at a recorded
    # waypoint's defaults, judged by the real continuous verifier.
    from weld_motion_planner.planner.trajectory_optimization.engine.taught import HOME_AS_WAYPOINT
    planner.context.planner_collision_model = None
    nodes = MIXES[mix]
    entries = {e["node_id"]: e for e in planner.connect(nodes, radius=0. if mix.startswith("MoveL") else RADIUS_M)}
    assert planner.specs == [], "no free-space search was asked for"
    first = entries[nodes[0]["id"]]
    knots = np.deg2rad(first["knots_deg"])[:, 0]
    assert knots[0] == pytest.approx(MEASURED, abs=1e-12) and knots[-1] == pytest.approx(HOME, abs=1e-12)
    assert first["authored_motion"] == "home" and first["verdict"]["verdict"] == "PASS"
    # Rest to rest, and no faster than the Home waypoint's TCP bound: the TCP
    # sits 1 m out, so 250 mm/s is 0.25 rad/s of J1 at most.
    assert first["knot_velocity_deg_s"][0] == [0.] and first["knot_velocity_deg_s"][-1] == [0.]
    assert first["duration_s"] >= MEASURED / (HOME_AS_WAYPOINT["speed_mm_s"] / 1000 / max(RADIUS_M, 1e-9)) - 1e-6 \
        or mix.startswith("MoveL")


def test_approach_from_the_measured_pose_seeds_through_the_weld_standoff(planner):
    planner.connect(MIXES["weld-only"])
    approach, retract = planner.specs
    np.testing.assert_allclose(approach.start, [MEASURED], atol=1e-12, rtol=0)
    np.testing.assert_allclose(approach.end, [WELDS["a"][0]], atol=1e-12, rtol=0)
    np.testing.assert_allclose(approach.via[0], [WELDS["a"][0] + STANDOFF_RAD], atol=1e-12, rtol=0)
    # A terminal retract keeps the return-home behaviour.
    np.testing.assert_allclose(retract.end, [HOME], atol=1e-12, rtol=0)


def test_roadmap_routes_every_free_space_leg_from_where_the_arm_is(planner, monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace
    asked = []
    monkeypatch.setattr(freespace, "roadmap_path", lambda roadmap, model, device, start, end, **kwargs:
                        asked.append((float(start[0]), float(end[0]))) or np.array([start, end]))
    planner.connect(MIXES["weld-only with home"], roadmap=object())
    assert asked == [(MEASURED, HOME), (HOME, WELDS["a"][0]), (WELDS["a"][1], WELDS["b"][0]), (WELDS["b"][1], HOME)]
    assert all(spec.route is not None for spec in planner.specs)


OUTPUTS = {"schema": "rosie.cell-io-outputs.v1", "torch_qualified": False,
           "outputs": [{"name": "gas", "class": "process", "expiry_ns": 200_000_000}]}


@pytest.mark.parametrize("start_q", [None, MEASURED])
def test_io_without_the_cells_outputs_is_refused_by_name_on_every_path(planner, start_q):
    io = {"id": "gas", "op": "io", "label": "Gas on", "channel": "gas", "state": True}
    with pytest.raises(ValueError, match=r"^Gas on \(io\) switches an output: plan with the cell connected"):
        planner.connect([_move("P1", "joint", .5), io], start_q=start_q)
    from weld_motion_planner.planner.trajectory_optimization.connecting_trajopt_main import ConnectingTrajoptModule
    weld_only = [{"id": "in", "op": "approach"}, _weld("wa", "a"), io, {"id": "out", "op": "retract"}]
    with pytest.raises(ValueError, match="switches an output"):
        ConnectingTrajoptModule().plan(planner.request(weld_only, start_q), planner.reports, None, planner.context)
    from weldplan.measured_start import validate
    with pytest.raises(ValueError, match=r"Gas on \(io\) switches an output"):
        validate(_start(MEASURED), {"axes": ["J1"], "robot_cell": BINDING}, {"nodes": weld_only})
    with pytest.raises(ValueError, match=r"switches 'revpi:1', which is not an output of this cell \(outputs: gas\)"):
        validate(_start(MEASURED), {"axes": ["J1"], "robot_cell": BINDING},
                 {"nodes": [_move("P1", "joint", .5), {**io, "channel": "revpi:1"}]}, OUTPUTS)
    assert validate(_start(MEASURED), {"axes": ["J1"], "robot_cell": BINDING},
                    {"nodes": [{**io, "enabled": False}, _move("P1", "joint", .5)]}) is not None


def test_io_rows_with_the_cells_outputs_are_events_the_connector_passes(planner):
    # An output row is planned like a dwell: no leg, the arm at rest; the
    # dense export gives it its segment and markers.
    io = {"id": "gas", "op": "io", "label": "Gas on", "channel": "gas", "state": True}
    request = planner.request([_move("P1", "joint", .5), io, _move("P2", "joint", .6)], MEASURED)
    request.io_outputs = OUTPUTS
    from weld_motion_planner.planner.trajectory_optimization.engine import taught
    entries = taught.connect_taught_program(request, planner.reports, SimpleNamespace(backend=None, verify_margin_m=.002),
                                            planner.context, planner.context.cell)
    assert [e["node_id"] for e in entries] == ["P1", "P2"]
    assert np.deg2rad(entries[1]["knots_deg"][0][0]) == pytest.approx(.5, abs=1e-5)


def test_module_routes_a_measured_weld_only_program_in_order_from_the_measured_pose(planner, monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization import connecting_trajopt_main
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace, roadmap
    seen = {}
    monkeypatch.setattr(roadmap, "build", lambda request, model, device, **kwargs:
                        seen.setdefault("focus", kwargs["focus"]) is not None and "graph")
    monkeypatch.setattr(freespace, "winding_reach", lambda request, reports, start, **kwargs:
                        seen.setdefault("reach_from", float(start[0])) is not None and np.array([start]))
    monkeypatch.setattr(freespace, "rewound", lambda request, reports, start, **kwargs:
                        seen.setdefault("wound_from", (float(start[0]), kwargs["roadmap"])) and reports)
    monkeypatch.setattr(freespace, "roadmap_path", lambda *args, **kwargs: None)
    monkeypatch.setattr(freespace, "connect", lambda *args, **kwargs: pytest.fail("model-reset connector used"))
    options = SimpleNamespace(backend=None, verify_margin_m=.002, order_seams=True)
    out = connecting_trajopt_main.ConnectingTrajoptModule().plan(
        planner.request(MIXES["weld-only with home"], MEASURED), planner.reports, options, planner.context)
    assert seen["reach_from"] == MEASURED and seen["wound_from"] == (MEASURED, "graph")
    first = out.connecting_trajectories[0]
    assert first["node_id"] == "home-in" and np.deg2rad(first["knots_deg"][0][0]) == pytest.approx(MEASURED, abs=1e-12)


def test_model_reset_weld_only_program_keeps_the_reordering_connector(planner, monkeypatch):
    from weld_motion_planner.planner.trajectory_optimization import connecting_trajopt_main
    from weld_motion_planner.planner.trajectory_optimization.engine import freespace, roadmap
    called = {}
    monkeypatch.setattr(roadmap, "build", lambda *args, **kwargs: "graph")
    monkeypatch.setattr(freespace, "winding_reach", lambda request, reports, start, **kwargs: np.array([start]))
    monkeypatch.setattr(freespace, "rewound", lambda request, reports, start, **kwargs: reports)
    monkeypatch.setattr(freespace, "order_seams", lambda reports, start, **kwargs: reports)
    monkeypatch.setattr(freespace, "connect", lambda request, reports, **kwargs: called.setdefault("connect", []))
    options = SimpleNamespace(backend=None, verify_margin_m=.002, order_seams=True)
    connecting_trajopt_main.ConnectingTrajoptModule().plan(
        planner.request(MIXES["weld-only with home"], None), planner.reports, options, planner.context)
    assert "connect" in called


def test_program_order_follows_the_measured_start():
    from weldplan.measured_start import in_program_order
    weld_only = {"nodes": MIXES["weld-only"]}
    assert not in_program_order(weld_only, None)
    assert in_program_order(weld_only, _start(MEASURED))
    assert in_program_order({"nodes": [_move("P1", "joint", .5)]}, None)
    assert not in_program_order({"nodes": [{**_move("P1", "joint", .5), "enabled": False}]}, None)


LOOP_RETURN = {"id": "rosie-loop-return", "op": "home", "extensions": {"rosie.loop_return": True}}


@pytest.mark.parametrize("model", [None, "search"])
def test_loop_return_goes_back_to_the_measured_start_like_a_home(planner, model):
    # OLP loop mode: the last leg ends at the plan's first configuration, not
    # at the reset pose, planned the way a Home is (verified MoveJ without a
    # planner collision model, the free-space search with one).
    if model is None:
        planner.context.planner_collision_model = None
    nodes = [_move("P1", "joint", .5), _move("P2", "joint", .8), {"id": "home-out", "op": "home"}, LOOP_RETURN]
    entries = {e["node_id"]: e for e in planner.connect(nodes)}
    if model is None:
        loop = entries["rosie-loop-return"]
        knots = np.deg2rad(loop["knots_deg"])[:, 0]
        assert loop["authored_motion"] == "loop_return" and loop["verdict"]["verdict"] == "PASS"
        assert knots[0] == pytest.approx(HOME, abs=1e-12) and knots[-1] == pytest.approx(MEASURED, abs=1e-12)
        assert entries["home-out"]["authored_motion"] == "home"
    else:
        home, loop = planner.specs
        np.testing.assert_allclose(home.end, [HOME], atol=1e-12, rtol=0)
        np.testing.assert_allclose(loop.start, [HOME], atol=1e-12, rtol=0)
        np.testing.assert_allclose(loop.end, [MEASURED], atol=1e-12, rtol=0)
        assert loop.kind == "transit" and loop.via == []


def test_loop_return_is_skipped_when_the_program_already_ends_at_its_start(planner):
    planner.context.planner_collision_model = None
    entries = {e["node_id"] for e in planner.connect([_move("P1", "joint", .5), _move("P2", "joint", MEASURED), LOOP_RETURN])}
    assert entries == {"P1", "P2"}, "no extra move when the program ends where it starts"
