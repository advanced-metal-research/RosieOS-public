"""Reusable native motion-plan artifacts with exact input/source identity.

The planner owns these bytes.  Weld Preflight may retain them unchanged, but
neither this module nor the optional handoff evaluates the candidate or grants
execution authority.
"""

from __future__ import annotations

import os
import re
from typing import Any

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.wire import as_json, finite

RESULT_SCHEMA = "amr-weld-planner-v1.motion-plan-result.v1"
SOURCE_REVISION_ENV = "AMR_WELD_PLANNER_SOURCE_REVISION"
_COMMIT = re.compile(r"^[0-9a-f]{40}$")


def validate_source_revision(value: str) -> str:
    """Require an immutable full Git commit, never a branch or short SHA."""
    if not _COMMIT.fullmatch(value):
        raise ValueError(
            f"{SOURCE_REVISION_ENV} must be a full 40-character lowercase Git commit"
        )
    return value


def source_revision_from_environment() -> str | None:
    """Return the launcher-supplied source pin, or an honest absence."""
    value = os.environ.get(SOURCE_REVISION_ENV)
    if value is None or not value.strip():
        return None
    return validate_source_revision(value.strip())


def _verification_coverage(verdicts: list[Any]) -> str:
    if not verdicts:
        return "not_evaluated"
    checked = sum(verdict is not None for verdict in verdicts)
    if checked == 0:
        return "not_evaluated"
    if checked == len(verdicts):
        return "evaluated_for_all_emitted_trajectories"
    return "partially_evaluated"


def result_document(
    request: PlanRequest,
    motion_plan: Any,
    *,
    request_sha256: str,
    source_revision: str | None,
) -> dict[str, Any]:
    """Return the planner-native result plus exact lineage and claim limits."""
    if not re.fullmatch(r"[0-9a-f]{64}", request_sha256):
        raise ValueError("request_sha256 must be a lowercase SHA-256 digest")
    if source_revision is not None:
        source_revision = validate_source_revision(source_revision)
    speed_scale = request.program.get("speed_scale", 1.0)
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
            or not 0.01 <= speed_scale <= 1.0):
        raise ValueError("program.speed_scale must be in [0.01, 1]")

    found = motion_plan.seam_search_result
    if found is None:
        raise ValueError("a native result document requires a seam-search result")

    weld_trajectories = list(motion_plan.weld_trajectories)
    connecting = motion_plan.connecting_trajectories
    trajectories: dict[str, dict[str, Any]] = {}
    if weld_trajectories:
        from weld_motion_planner.planner.trajectory_optimization.engine.weld_trajopt import (
            trajectory_json,
        )

        trajectories = {
            trajectory.seam_id: trajectory_json(trajectory)
            for trajectory in weld_trajectories
        }
    document = as_json(
        found,
        trajectories,
    )
    document.update(
        {
            "schema": RESULT_SCHEMA,
            "request": {
                "request_sha256": request_sha256,
                "request_id": request.request_id,
                "request_schema": request.manifest.get("schema"),
                "program_id": request.program.get("program_id"),
                "program_schema": request.program.get("schema"),
                "cell_id": request.cell.get("id"),
                "cell": request.manifest.get("cell"),
                "source": request.manifest.get("source"),
                "contents": request.manifest.get("contents"),
                "placement": request.placement,
            },
            "producer": {
                "module": "amr-weld-planner/v1",
                "source_revision": source_revision,
                "source_revision_state": (
                    "bound" if source_revision is not None else "unavailable"
                ),
            },
            "candidate_search": {
                "method": "dynamic_programming_over_discrete_redundancy_lattice",
                "objective": "minimum_sum_absolute_joint_motion_rad",
                "optimality_scope": "exact_within_the_sampled_lattice",
            },
            "coverage": {
                "candidate_collision_screening": (
                    "sampled_lattice_nodes" if found.screened else "not_evaluated"
                ),
                "candidate_limit_checks": "reported_per_candidate",
                "weld_trajectory_continuous_verification": _verification_coverage(
                    [trajectory.verdict for trajectory in weld_trajectories]
                ),
                "connecting_trajectory_continuous_verification": (
                    _verification_coverage(
                        [move.get("verdict") for move in connecting]
                    )
                    if connecting is not None
                    else "not_evaluated"
                ),
                "canonical_motion_server": "not_evaluated",
                "physical_motion": "not_evaluated",
                "execution_authority": "none",
            },
            "options": motion_plan.options.as_json(),
            "timings_ms": motion_plan.timings_ms,
        }
    )
    from weldplan.measured_start import in_program_order
    if in_program_order(request.program, getattr(request, "start_state", None)):
        document["program_order"] = [
            {"id": n["id"], "op": n["op"],
             **({"seam_id": n["geometry"].get("id")} if n["op"] == "weld" else {}),
             **({"duration_s": n["duration_s"]} if n["op"] == "dwell" else {}),
             **({k:n[k] for k in ("weld_transition","weld_start_delay_s","weld_stop_delay_s","motion","speed_mm_s","constant_tcp_speed") if k in n} if n["op"]=="move" else {}),
             **({k:n[k] for k in ("channel","state","action","duration_s","timeout_s","stable_s") if k in n} if n["op"] == "io" else {})}
            for n in request.program["nodes"] if n.get("enabled", True)]
    if connecting is not None:
        document["connecting_trajectories"] = connecting
    if motion_plan.connecting_trajopt_error is not None:
        document["connecting_trajectories_error"] = (
            motion_plan.connecting_trajopt_error
        )
    document = finite(document)
    if speed_scale != 1.0:
        # One clock for every axis: q(s*t), qdot(s*t)*s. The exact Hermite
        # path and its clearance/tracking certificate remain unchanged;
        # original velocity/acceleration bounds stay conservative at s <= 1.
        document["speed_scale"] = speed_scale
        wires = list(document.get("connecting_trajectories") or [])
        wires.extend(seam["trajectory"] for seam in document.get("seams", [])
                     if seam.get("trajectory"))
        for wire in wires:
            wire["times_s"] = [t / speed_scale for t in wire["times_s"]]
            wire["knot_velocity_deg_s"] = [
                [v * speed_scale for v in row] for row in wire["knot_velocity_deg_s"]]
            if "duration_s" in wire:
                wire["duration_s"] /= speed_scale
    return document
