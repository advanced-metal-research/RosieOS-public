"""The reusable result keeps native candidate bytes and lineage together."""

from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
import pytest
from weld_motion_planner.io.native_result import (
    RESULT_SCHEMA,
    SOURCE_REVISION_ENV,
    result_document,
    source_revision_from_environment,
)
from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.results import Candidate, SeamPlan, SeamSearchResult

REQUEST_SHA256 = "1" * 64
SOURCE_REVISION = "2" * 40


def _request() -> PlanRequest:
    return PlanRequest(
        manifest={
            "schema": "amr-weld-planner-v1.plan-request.v1",
            "request_id": "case-7",
            "cell": {"id": "rosie_1400_v3", "path": "cell.json"},
            "source": {
                "filename": "part.step",
                "path": "source/part.step",
                "sha256": "3" * 64,
            },
            "contents": [
                {"path": "program.json", "sha256": "4" * 64},
                {"path": "placement.json", "sha256": "5" * 64},
                {"path": "cell.json", "sha256": "6" * 64},
            ],
        },
        program={
            "schema": "amr-weld-planner-v1.weld-program.v1",
            "program_id": "program-7",
        },
        placement={
            "cell_id": "rosie_1400_v3",
            "offset_xyz_m": [0.0, 0.02, 0.0],
        },
        cell={"id": "rosie_1400_v3"},
        step_bytes=b"ISO-10303-21;",
    )


def _result(*, screened: bool = True) -> SeamSearchResult:
    return SeamSearchResult(
        cell_id="rosie_1400_v3",
        screened=screened,
        threshold_rad=np.asarray([0.1, 0.2]),
        threshold_derived_from_limits=True,
        k_best=1,
        seams=[
            SeamPlan(
                id="seam-1",
                length_m=0.1,
                crossed=True,
                axis_names=["J1", "J2", "J3", "J4", "J5", "J6", "J7"],
                fractions=np.asarray([0.0, 1.0]),
                candidates=[
                    Candidate(
                        cost_rad=0.3,
                        joints_rad=np.asarray(
                            [
                                [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, -1.0],
                                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -0.9],
                            ]
                        ),
                        times_s=np.asarray([0.0, 0.25]),
                    )
                ],
            )
        ],
    )


class _Options:
    def as_json(self) -> dict[str, object]:
        return {"k_best": 1, "screen_collisions": True}


def _motion_plan(*, screened: bool = True) -> SimpleNamespace:
    return SimpleNamespace(
        seam_search_result=_result(screened=screened),
        weld_trajectories=[],
        connecting_trajectories=None,
        connecting_trajopt_error=None,
        options=_Options(),
        timings_ms={"dp_seam_search": 12.0, "total": 12.0},
    )


def test_native_result_binds_input_source_and_full_sampled_joint_path():
    document = result_document(
        _request(),
        _motion_plan(),
        request_sha256=REQUEST_SHA256,
        source_revision=SOURCE_REVISION,
    )

    assert document["schema"] == RESULT_SCHEMA
    assert document["request"] == {
        "request_sha256": REQUEST_SHA256,
        "request_id": "case-7",
        "request_schema": "amr-weld-planner-v1.plan-request.v1",
        "program_id": "program-7",
        "program_schema": "amr-weld-planner-v1.weld-program.v1",
        "cell_id": "rosie_1400_v3",
        "cell": {"id": "rosie_1400_v3", "path": "cell.json"},
        "source": {
            "filename": "part.step",
            "path": "source/part.step",
            "sha256": "3" * 64,
        },
        "contents": [
            {"path": "program.json", "sha256": "4" * 64},
            {"path": "placement.json", "sha256": "5" * 64},
            {"path": "cell.json", "sha256": "6" * 64},
        ],
        "placement": {
            "cell_id": "rosie_1400_v3",
            "offset_xyz_m": [0.0, 0.02, 0.0],
        },
    }
    assert document["producer"] == {
        "module": "amr-weld-planner/v1",
        "source_revision": SOURCE_REVISION,
        "source_revision_state": "bound",
    }
    candidate = document["seams"][0]["candidates"][0]
    assert len(candidate["joints_deg"]) == 2
    assert all(len(sample) == 7 for sample in candidate["joints_deg"])
    assert candidate["times_s"] == [0.0, 0.25]
    assert document["coverage"]["candidate_collision_screening"] == (
        "sampled_lattice_nodes"
    )
    assert document["coverage"]["canonical_motion_server"] == "not_evaluated"
    assert document["coverage"]["execution_authority"] == "none"
    json.dumps(document, allow_nan=False, sort_keys=True)


def test_missing_source_and_unscreened_coverage_are_explicit():
    document = result_document(
        _request(),
        _motion_plan(screened=False),
        request_sha256=REQUEST_SHA256,
        source_revision=None,
    )

    assert document["producer"]["source_revision"] is None
    assert document["producer"]["source_revision_state"] == "unavailable"
    assert document["coverage"]["candidate_collision_screening"] == (
        "not_evaluated"
    )
    assert document["coverage"]["weld_trajectory_continuous_verification"] == (
        "not_evaluated"
    )


def test_source_revision_must_be_an_exact_commit(
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv(SOURCE_REVISION_ENV, "b4342a96")
    with pytest.raises(ValueError, match="full 40-character"):
        source_revision_from_environment()

    with pytest.raises(ValueError, match="full 40-character"):
        result_document(
            _request(),
            _motion_plan(),
            request_sha256=REQUEST_SHA256,
            source_revision="origin/js/weld-planning",
        )


def test_whole_plan_speed_scales_all_axes_and_keeps_source_reports(monkeypatch):
    from copy import deepcopy
    from weld_motion_planner.io import native_result

    wire = {"times_s": [0.0, 1.0, 2.0], "duration_s": 2.0,
            "knots_deg": [[0, 0], [10, -20], [20, -40]],
            "knot_velocity_deg_s": [[0, 0], [10, -20], [0, 0]]}
    source = {"seams": [{"trajectory": deepcopy(wire)}]}
    monkeypatch.setattr(native_result, "as_json", lambda *_: deepcopy(source))
    motion = _motion_plan()
    motion.connecting_trajectories = [deepcopy(wire)]
    request = _request()
    request.program["speed_scale"] = 0.1
    request.program["nodes"] = [{"id": "m", "op": "move"},
                                {"id": "d", "op": "dwell", "duration_s": 3}]
    document = result_document(request, motion, request_sha256=REQUEST_SHA256, source_revision=None)
    for scaled in [document["seams"][0]["trajectory"], document["connecting_trajectories"][0]]:
        assert scaled["times_s"] == [0, 10, 20]
        assert scaled["duration_s"] == 20
        assert scaled["knots_deg"] == wire["knots_deg"]
        assert scaled["knot_velocity_deg_s"] == [[0, 0], [1, -2], [0, 0]]
    assert document["program_order"][1]["duration_s"] == 3
    assert motion.connecting_trajectories == [wire]
    assert source["seams"][0]["trajectory"] == wire


@pytest.mark.parametrize("scale", [0, -1, 1.01, float("nan"), float("inf"), True, "0.1"])
def test_invalid_plan_speed_is_refused(scale):
    request = _request()
    request.program["speed_scale"] = scale
    with pytest.raises(ValueError, match="program.speed_scale"):
        result_document(request, _motion_plan(), request_sha256=REQUEST_SHA256, source_revision=None)
