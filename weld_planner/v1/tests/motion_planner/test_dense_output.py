"""The dense joint trajectory the motion server emits beside its result.

A real plan needs a GPU and minutes; what this file pins is the part that is
new. The blob is built from the plan's own result with the identity the
daemon fences on, is filed by digest, and comes back byte for byte through
`/api/motion/dense/{digest}` -- and a result the daemon could not play yields
a reason in the daemon's vocabulary, never a failed plan.
"""

from __future__ import annotations

import json

import pytest
from weld_motion_planner.io.dense_output import dense_blob_path, emit_dense
from weldplan import dense_joint_trajectory as dtj

ARM = ["J1", "J2", "J3", "J4", "J5", "J6"]
HELD = {"J7": 12.5, "J8": -3.0, "J9": 0.0}
REQUEST_SHA = "f" * 64


def _smooth_wire(q0, q1, duration_s, samples):
    """A smoothstep pass between two poses: at rest at both ends, as M6 emits."""
    times = [duration_s * i / (samples - 1) for i in range(samples)]
    knots, velocities = [], []
    for t in times:
        u = t / duration_s
        s, ds = u * u * (3 - 2 * u), 6 * u * (1 - u) / duration_s
        knots.append([round(a + (b - a) * s, 4) for a, b in zip(q0, q1)])
        velocities.append([round((b - a) * ds, 4) for a, b in zip(q0, q1)])
    return {"knots_deg": knots, "knot_velocity_deg_s": velocities, "times_s": times}


def native_result():
    home = [10.0, -20.0, 30.0, 0.0, 15.0, -5.0]
    start = [12.0, -21.0, 31.0, 1.0, 16.0, -6.0]
    end = [12.5, -21.2, 31.1, 1.2, 16.1, -6.1]
    # Synthetic native verdicts: the admission gate wants a PASS on every
    # segment; this exercises emission, not the planner's own verification.
    clear = {"verdict": "PASS", "collisions": 0, "penetrating": 0,
             "limit_violations": 0, "unverifiable": []}
    tracking = {"verdict": "PASS", "bound_mm": 0.1, "tolerance_mm": 0.5,
                "segments_out": 0, "undecided": 0, "unverifiable": []}
    return {
        "schema": "amr-weld-planner-v1.motion-plan-result.v1",
        "seams": [{"id": "seam-1", "crossed": True, "axis_names": ARM, "held": HELD,
                   "trajectory": {**_smooth_wire(start, end, 5.0, 26),
                                  "verdict": dict(clear), "tracking": dict(tracking)}}],
        "connecting_trajectories": [
            {"kind": "approach", "from": "home", "to": "seam-1", "axis_names": ARM,
             "held": HELD, **_smooth_wire(home, start, 4.0, 21), "verdict": dict(clear)},
            {"kind": "retract", "from": "seam-1", "to": "home", "axis_names": ARM,
             "held": HELD, **_smooth_wire(end, home, 4.0, 21), "verdict": dict(clear)},
        ],
    }


def test_the_blob_carries_the_fenced_identity_and_is_filed_by_digest(tmp_path):
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path / "dense", request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=3, plan_revision=7)
    assert error is None
    assert summary["plan_id"] == "prog-1:ffffffffffff"
    assert summary["program_digest"] == "sha256:" + REQUEST_SHA
    assert (summary["manifest_revision"], summary["plan_revision"]) == (3, 7)
    assert [s["kind"] for s in summary["segments"]] == [
        "freespace", "dwell", "weld", "dwell", "freespace"]
    assert [s["source_id"] for s in summary["segments"]][2] == "seam-1"
    assert summary["total_sample_count"] == sum(s["sample_count"] for s in summary["segments"])
    blob = dense_blob_path(tmp_path / "dense", summary["trajectory_digest"]).read_bytes()
    assert len(blob) == summary["bytes"]
    header = json.loads(blob[4:4 + int.from_bytes(blob[:4], "big")])
    assert header["trajectory_digest"] == summary["trajectory_digest"]
    assert header["plan_id"] == summary["plan_id"]
    assert header["schema"] == dtj.SCHEMA


def test_the_header_names_the_robot_and_cell_the_plan_was_made_against(tmp_path):
    robot = {"model_id": "rosie_1420_v1", "robot_description_sha256": "sha256:" + "a" * 64,
             "machine_planning_calibration_sha256": "sha256:" + "b" * 64, "machine_planning_calibration_source": "empty"}
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path, request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1, robot=robot)
    assert error is None and summary["robot_cell"] == robot
    blob = dense_blob_path(tmp_path, summary["trajectory_digest"]).read_bytes()
    header = json.loads(blob[4:4 + int.from_bytes(blob[:4], "big")])
    assert header["robot_cell"] == robot
    # Identity, not content: the same trajectory has the same digest either way.
    bare, _ = emit_dense(
        native_result(), store_dir=tmp_path / "bare", request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1)
    assert bare["trajectory_digest"] == summary["trajectory_digest"]
    assert bare["robot_cell"] is None


def test_a_result_the_daemon_could_not_play_is_a_reason_not_a_failure(tmp_path):
    result = native_result()
    del result["connecting_trajectories"]
    result["seams"][0].pop("trajectory")
    summary, error = emit_dense(
        result, store_dir=tmp_path, request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1)
    assert summary is None
    # A seam with no trajectory fails the native admission gate first: it
    # is unverified, and only then would it be empty.
    assert error["reason"] == "motion_not_verified"
    assert error["detail"]


def test_a_program_id_that_is_not_an_identifier_is_refused_before_anything_is_written(tmp_path):
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path / "dense", request_sha256=REQUEST_SHA,
        program_id="../escape", manifest_revision=1, plan_revision=1)
    assert summary is None and error["reason"] == "identity_invalid"
    assert not (tmp_path / "dense").exists()


def test_dry_run_is_filed_as_its_own_motion_only_blob(tmp_path):
    results = {}
    for mode in ("production", "dry_run"):
        summary, error = emit_dense(
            native_result(), store_dir=tmp_path, request_sha256=REQUEST_SHA,
            program_id="prog-1", manifest_revision=1, plan_revision=1,
            execution_mode=mode)
        assert error is None
        assert summary["execution_mode"] == mode
        blob = dense_blob_path(tmp_path, summary["trajectory_digest"]).read_bytes()
        size = int.from_bytes(blob[:4], "big")
        header, samples = json.loads(blob[4:4 + size]), blob[4 + size:]
        # Every record is t, nine q, nine qd and one output flag.
        stride = 8 * 19 + 1
        flags = samples[stride - 1::stride]
        assert bool(any(flags)) == (mode == "production")
        assert header["trajectory_digest"] == summary["trajectory_digest"]
        results[mode] = (summary, samples)
    assert results["production"][0]["trajectory_digest"] != results["dry_run"][0]["trajectory_digest"]
    prod, dry = results["production"][1], results["dry_run"][1]
    assert len(prod) == len(dry)
    for offset in range(0, len(prod), stride):
        assert prod[offset:offset + stride - 1] == dry[offset:offset + stride - 1]


def test_invalid_execution_mode_is_a_refusal_without_a_filed_blob(tmp_path):
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path / "dense", request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1,
        execution_mode="dry-run")
    assert summary is None and error["reason"] == "execution_mode_invalid"
    assert not (tmp_path / "dense").exists()


def test_the_route_returns_the_filed_bytes(tmp_path):
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    from weld_motion_planner.server.motion_planner_server import make_app

    store = tmp_path / "dense"
    summary, _ = emit_dense(
        native_result(), store_dir=store, request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1)
    digest = summary["trajectory_digest"]
    with fastapi_testclient.TestClient(make_app(dense_store_dir=store)) as client:
        response = client.get(f"/api/motion/dense/{digest}")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/octet-stream")
        assert response.content == dense_blob_path(store, digest).read_bytes()
        assert client.get("/api/motion/dense/sha256:" + "0" * 64).status_code == 404
        assert client.get("/api/motion/dense/not-a-digest").status_code == 422


def test_health_names_the_installed_commit(tmp_path, monkeypatch):
    # The deploy tool's `olp install` compares this field with the commit it
    # installed and restores the previous planner unit when they differ.
    fastapi_testclient = pytest.importorskip("fastapi.testclient")
    from weld_motion_planner.server.motion_planner_server import make_app

    commit = "0123456789abcdef0123456789abcdef01234567"
    monkeypatch.setenv("AMR_WELD_PLANNER_SOURCE_REVISION", commit)
    with fastapi_testclient.TestClient(make_app(dense_store_dir=tmp_path)) as client:
        health = client.get("/api/motion/health").json()
        assert health["source_revision"] == commit
        assert health["cuda"] in (True, False)
    monkeypatch.delenv("AMR_WELD_PLANNER_SOURCE_REVISION")
    with fastapi_testclient.TestClient(make_app(dense_store_dir=tmp_path)) as client:
        assert client.get("/api/motion/health").json()["source_revision"] is None


@pytest.mark.parametrize("order_query, expected_order", [
    ("", False), ("&order_seams=false", False), ("&order_seams=true", True),
])
def test_plan_route_forwards_authored_execution_mode_to_dense_export(
        tmp_path, monkeypatch, order_query, expected_order):
    from types import SimpleNamespace
    from fastapi.testclient import TestClient
    from weld_motion_planner.io import native_result as native_module, wire
    from weld_motion_planner.planner import planner_main
    from weld_motion_planner.robot_cell.robot import profile
    from weld_motion_planner.server import motion_planner_server as server

    # Replace the expensive solver, keeping the actual HTTP route, emitter,
    # encoder and filed bytes. No motion or external service participates.
    monkeypatch.setattr(wire, "request_from_bytes", lambda data: SimpleNamespace(program=json.loads(data), cell={}, start_state=None))
    planned_options = []
    def plan_motion(opened, options, **kwargs):
        planned_options.append(options)
        return SimpleNamespace(connecting_trajectories=[], connecting_trajopt_error=None)
    monkeypatch.setattr(planner_main, "plan_motion", plan_motion)
    monkeypatch.setattr(native_module, "result_document", lambda *a, **k: native_result())
    monkeypatch.setattr(native_module, "source_revision_from_environment", lambda: "test")
    monkeypatch.setattr(profile, "for_cell", lambda cell: None)
    monkeypatch.setattr(profile, "moved_axes", lambda cell: ARM)
    monkeypatch.setattr(server, "release_gpu", lambda: None)
    async def run_job(data, parameters, dense_dir, request, progress, spare=None):
        # Exercise the same child pipeline in-process so solver mocks remain
        # local; real process lifecycle is covered by test_server_cancel.py.
        # Preserve the actual JSON worker boundary as well as the endpoint's
        # query parsing: program order must survive process isolation.
        return server._execute_plan_job(
            data, json.loads(json.dumps(parameters)), dense_dir, progress)
    monkeypatch.setattr(server, "_run_plan_job", run_job)
    digests = {}
    with TestClient(server.make_app(dense_store_dir=tmp_path)) as client:
        for mode in ("dry_run", "production", "simulation", None):
            program = {} if mode is None else {"execution_mode": mode}
            response = client.post(
                "/api/motion/plan?dense=true&run_weld_trajopt=true&run_connecting_trajopt=true&program_id=prog-1" + order_query,
                content=json.dumps(program).encode())
            assert response.status_code == 200, response.text
            answer = response.json()
            assert "dense_error" not in answer
            summary = answer["dense"]
            assert summary["execution_mode"] == (mode or "production")
            blob = dense_blob_path(tmp_path, summary["trajectory_digest"]).read_bytes()
            size = int.from_bytes(blob[:4], "big")
            samples = blob[4 + size:]
            stride = 8 * 19 + 1
            assert bool(any(samples[stride - 1::stride])) == (mode != "dry_run")
            digests[mode] = summary["trajectory_digest"]
    assert digests["dry_run"] != digests["production"]
    assert digests["production"] == digests["simulation"] == digests[None]
    assert len(planned_options) == 4
    assert all(options.connecting_trajopt.order_seams is expected_order
               for options in planned_options)


def test_the_cell_profile_lowers_the_header_ceiling_and_refuses_a_faster_plan(tmp_path):
    """The header declares the cell's own limit, and a plan above it is the
    exporter's refusal with the segment named -- not a blob the daemon
    would reject at its gate."""
    from weld_motion_planner.io.dense_output import cell_speed_limits
    limits = cell_speed_limits({"J1": {"velocity_rad_s": 0.09}, "J4": {"velocity_rad_s": 9.0}},
                               ceiling=[3.0] * 9)
    assert limits == [0.09, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]  # never above the default
    # The synthetic wires move J1 by 2 deg over 4 s: under 0.09 rad/s.
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path, request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1,
        max_abs_qd_rad_s=limits)
    assert error is None
    assert summary["max_abs_qd_rad_s"] == limits
    # A cell that cannot reach even the crawl: refused, and the exporter says where.
    summary, error = emit_dense(
        native_result(), store_dir=tmp_path, request_sha256=REQUEST_SHA,
        program_id="prog-1", manifest_revision=1, plan_revision=1,
        max_abs_qd_rad_s=[0.001] * 9)
    assert summary is None
    assert error["reason"] == "qd_limit_exceeded"
    assert error["segment_index"] is not None


def test_dense_output_refuses_implicit_approach_from_another_start(tmp_path):
    import math
    start = {"coordinate_identity": "config:1:32", "observation_identity": "adapter:daemon", "joint_names": ARM, "joint_values_rad": [math.radians(v) for v in [10., -20., 30., 0., 15., -5.]]}
    kwargs = dict(store_dir=tmp_path, request_sha256=REQUEST_SHA, program_id="measured-start", manifest_revision=1, plan_revision=1, start_state=start)
    summary, error = emit_dense(native_result(), **kwargs)
    assert error is None
    blob = dense_blob_path(tmp_path, summary["trajectory_digest"]).read_bytes()
    header = json.loads(blob[4:4+int.from_bytes(blob[:4], "big")])
    assert header["start_state"] == start
    start["joint_values_rad"][5] += .001
    summary, error = emit_dense(native_result(), **kwargs)
    assert summary is None and error["reason"] == "start_state_mismatch"
