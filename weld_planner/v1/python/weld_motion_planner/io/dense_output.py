"""The joint-space daemon's blob, emitted by the motion server beside its result.

`/api/motion/plan` already holds the 100 Hz joint-space answer -- the exact
Hermite wires of every weld pass and every connecting move. This module turns
that answer into `robot.v4.dense-joint-trajectory.v1` in the same request,
stamps it with the identity the daemon fences on, and files it by content
digest so `/api/motion/dense/{digest}` can hand it out later.

Why here and not in a separate export step: the blob is the plan. A converter
that ran afterwards in another process needed the result shipped to it, a
second store, and an identity handshake between the two -- and it was one
more thing that could be skipped, leaving the operator with rows the daemon
would never accept. The motion server knows the request digest the moment
it answers, so it is the one place the identity can be stamped without a
round trip.

The identity the daemon fences on (`plan_id`, `program_id`, `program_digest`,
`manifest_revision`, `plan_revision`) is fixed at plan time and travels
unchanged into preload and play. `program_digest` is `sha256:<request
digest>`: the daemon compares the string and nothing else, and the request
bytes are what this plan was made from. `plan_id` is
`<program_id>:<request digest[:12]>`, so a replan of the same program is a
new plan and a resend of the same request is the same plan.

A failed emission is returned, never raised: the plan itself succeeded, the
rows compile from it, and the reason arrives in the daemon's own vocabulary
so the UI can say which segment the daemon would have refused.
"""

from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from weldplan import dense_joint_trajectory as dtj

DIGEST_PATTERN = re.compile(r"^sha256:[0-9a-f]{64}$")
PROGRAM_ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,120}$")


def dense_blob_path(store_dir: Path, trajectory_digest: str) -> Path:
    """Where a blob lives, by its content digest and nothing else.

    The digest is what the daemon fences on, so a blob that answers to it is
    the right blob whatever request produced it. The colon is replaced so the
    name is a plain filename on every filesystem the store may sit on.
    """
    if not DIGEST_PATTERN.match(trajectory_digest):
        raise ValueError(f"not a trajectory digest: {trajectory_digest!r}")
    return store_dir / f"{trajectory_digest.replace(':', '-')}.rdt"


def emit_dense(
    result: dict[str, Any],
    *,
    store_dir: Path,
    request_sha256: str,
    program_id: str,
    manifest_revision: int,
    plan_revision: int,
    max_abs_qd_rad_s: list[float] | None = None,
    axis_mask: int | None = None,
    robot: dict[str, str] | None = None,
    execution_mode: str = "production",
    start_state: dict | None = None,
    io_outputs: dict | None = None,
) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    """Build, validate, and file the blob for a finished plan.

    Returns `(summary, None)` on success -- the summary is what the UI shows
    and what the OLP loads and plays by (identity, sampling totals, and one
    entry per segment with its source id so a segment index maps back to a
    node) -- or `(None, error)` with the exporter's `reason`, `detail` and
    `segment_index` when the result cannot be played as-is. With
    `max_abs_qd_rad_s` (the cell's per-axis limits, see `cell_speed_limits`)
    the header declares that ceiling and a faster plan is one of those
    refusals (`qd_limit_exceeded`), not something the daemon finds later. `robot`
    is the descriptor's identity (`weldplan.robot_description.header_identity`),
    written as the header's `robot` block the daemon fences on; without it the
    blob still encodes, and the daemon refuses it. The bytes are
    written atomically: a reader that races the write sees the whole blob or
    none of it, never a truncated one that fails its digest.

    `execution_mode` is the authored program mode, already bound by the
    request digest. Dry run keeps the planned motion but emits no torch-on
    samples; production and simulation preserve the existing output intent.
    """
    if not PROGRAM_ID_PATTERN.match(program_id or ""):
        return None, {"reason": "identity_invalid",
                      "detail": f"program_id {program_id!r} is not a plain identifier",
                      "segment_index": None}
    identity = dtj.DensePlanIdentity(
        plan_id=f"{program_id}:{request_sha256[:12]}",
        program_id=program_id,
        program_digest=f"sha256:{request_sha256}",
        manifest_revision=int(manifest_revision),
        plan_revision=int(plan_revision),
        created_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
    )
    try:
        plan = dtj.plan_from_result(result, identity, max_abs_qd_rad_s=max_abs_qd_rad_s,
                                    axis_mask=axis_mask,
                                    robot=dtj.RobotCell(**robot) if robot else None,
                                    execution_mode=execution_mode, io_outputs=io_outputs)
        if start_state is not None:
            from dataclasses import replace
            first = plan.segments[0].q_rad[0] if plan.segments else None
            if first is None or any(abs(first[int(name[1:])-1] - q) > 1e-10 for name, q in zip(start_state["joint_names"], start_state["joint_values_rad"], strict=True)):
                raise dtj.DenseJointTrajectoryError("start_state_mismatch", "dense first sample differs from captured measured start")
            plan = replace(plan, start_state=start_state)
        blob = dtj.encode(plan)  # the daemon's own validation, run here first
    except dtj.DenseJointTrajectoryError as error:
        return None, {"reason": error.reason, "detail": error.detail,
                      "segment_index": error.segment_index}
    header = json.loads(blob[4:4 + int.from_bytes(blob[:4], "big")])
    digest = header["trajectory_digest"]
    path = dense_blob_path(store_dir, digest)
    store_dir.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_bytes(blob)
    tmp.replace(path)
    return {
        "schema": header["schema"],
        "execution_mode": execution_mode,
        "trajectory_digest": digest,
        "request_sha256": request_sha256,
        "plan_id": identity.plan_id,
        "program_id": identity.program_id,
        "program_digest": identity.program_digest,
        "manifest_revision": identity.manifest_revision,
        "plan_revision": identity.plan_revision,
        "created_at": identity.created_at,
        "dt_s": header["sampling"]["dt_s"],
        "total_sample_count": header["sampling"]["total_sample_count"],
        "total_duration_s": header["sampling"]["total_duration_s"],
        "max_abs_qd_rad_s": header["limits"]["max_abs_qd_rad_s"],
        "robot_cell": header.get("robot_cell"),
        # The output schedule's own digest (outside the trajectory digest);
        # clients keep it and the OLP's Load checks the header against it.
        **({"process_markers_sha256": header["process_markers_sha256"],
            "process_marker_count": len(header["process_markers"])}
           if "process_markers" in header else {}),
        "bytes": len(blob),
        "segments": [
            {"index": s["index"], "kind": s["kind"], "source_id": s["source_id"],
             "sample_count": s["sample_count"], "duration_s": s["duration_s"]}
            for s in header["segments"]
        ],
    }, None


def cell_axis_mask(moved_axes: list[str], held: dict[str, Any] | None = None) -> int:
    """The header's `axis_mask` for a cell: one bit per wire column, J1 is
    bit 0, set for every axis the plan COMMANDS -- the ones it drives and the
    ones it holds at a value. Both are setpoints the daemon sends.

    `rosie_1400_v3` drives J1-J7 and holds J8/J9, so this is 0x1ff, the params
    default, and nothing changes there. `rosie_1420_v1` drives J1-J6 and holds
    nothing, so it is 0x3f -- and that is the whole reason this exists: the
    executor refuses a plan whose mask names an axis the active pair has not
    mapped and homed, the 1420 pair maps 0x3f, and a 0x1ff header was refused
    with "all plan axes must be mapped and homed" while J7-J9 sat at zero.
    The wire stays nine columns wide; the mask is how six is said.
    """
    mask = 0
    for name in [*moved_axes, *(held or {})]:
        name = str(name)
        if not (len(name) >= 2 and name[0] == "J" and name[1:].isdigit()):
            raise ValueError(f"axis {name!r} is not one of J1..J{dtj.AXIS_COUNT}")
        index = int(name[1:]) - 1
        if not 0 <= index < dtj.AXIS_COUNT:
            raise ValueError(f"axis {name!r} is outside the {dtj.AXIS_COUNT}-axis wire")
        mask |= 1 << index
    if mask == 0:
        raise ValueError("a cell with no axes has nothing to send")
    return mask


def cell_speed_limits(profile_limits: dict[str, dict[str, float]] | None,
                      ceiling: list[float] | None = None) -> list[float]:
    """The header ceiling for a cell: per axis J1..J9, the lower of the
    format default and what the CELL enforces.

    `enforced_velocity_rad_s`, not `velocity_rad_s`: the header is the
    ceiling the daemon's follower may command, and it has to be the cell's
    real limit (rosie_1400_v3: 0.09 rad/s on J1-J3, 0.2 elsewhere, read
    from RTCore on 2026-09-07). The planner's own `velocity_rad_s` is
    deliberately lower -- that difference IS the follower's authority to
    close a lag, and writing the planning limit here would clamp the
    correction away exactly when it is needed. A profile that states only
    `velocity_rad_s` (an older one) falls back to it; an axis that states
    neither keeps the default, because absence is a profile problem the
    planner reports rather than a number to invent here.
    """
    default = list(ceiling if ceiling is not None else dtj.load_params()["max_abs_qd_rad_s"])
    out = []
    for index, value in enumerate(default):
        spec = (profile_limits or {}).get(f"J{index + 1}") or {}
        stated = spec.get("enforced_velocity_rad_s") or spec.get("velocity_rad_s")
        out.append(min(float(value), float(stated)) if stated and stated > 0 else float(value))
    return out
