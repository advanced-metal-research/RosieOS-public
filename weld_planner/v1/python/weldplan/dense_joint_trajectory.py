"""Encode a `robot.v4.dense-joint-trajectory.v1` blob for the dense trajectory daemon.

The program-by-reference path `control_solver.WIRE_DT_S` is waiting on: the
planner's per-segment `(t, q, qd)` wires go to the controller as one binary
blob over TCP instead of JSON rows over NATS, so certificate-density sampling
(0.01 s) no longer has to fit through a 1 MiB NATS payload or a browser tab.

Layout: `[u32 BE header_len][UTF-8 JSON header][block 0]..[block S-1]` — one
packed sample block per motion segment, in segment order. Each block keeps the
segment's own clock (`t_s` starts at 0.0): the planner's trajectories are each
"their own clock", and gluing them onto one timeline is exactly the mistake a
player must not make. Records are 153 bytes big-endian:
`f64 t_s, f64 q_rad[9], f64 qd_rad_s[9], u8 flags` (bit0 = torch_on, bits 1-7
reserved zero).

Everything here is stdlib-only on purpose: the C++ daemon and the Go OLP twin
hand-roll the same bytes, and the digest is defined over those bytes — a
serialization library whose output is not canonical (protobuf, pickle) would
force a second layout for the digest anyway.

`encode()` runs the same validation the daemon's ingest runs and raises on
violation, so a malformed plan fails at the producer with the producer's
stack trace instead of a rejection string from the robot.
"""

from __future__ import annotations

import functools
import hashlib
import json
import pathlib
import socket
import struct
from collections.abc import Sequence
from dataclasses import dataclass, field

from weldplan import io_schedule
from weldplan.native_admission import (
    MotionAdmissionError,
    require_native_motion,
)

SCHEMA = "robot.v4.dense-joint-trajectory.v1"
DIGEST_DOMAIN = b"robot.v4.dense-joint-trajectory.v1\x00"
SAMPLE_ENCODING = "f64-be-aos.v1"

AXIS_COUNT = 9  # J1..J6 arm + J7/J8 positioner + J9 side selector; the
# motion server's kLiveAxisCount. A different count is a different robot
# generation and a different schema version, not a parameter.
RECORD_BYTES = 8 + 8 * AXIS_COUNT + 8 * AXIS_COUNT + 1  # 153
_RECORD_STRUCT = struct.Struct(">19dB")

# Matches the OLP simulator's MaxTrajectorySamples resource ceiling. At the
# 0.01 s production grid this is ~42 minutes of motion (2500 s), and 38.25 MB
# of blob -- inside the transport's 64 MiB frame limit with room for growth.
MAX_TOTAL_SAMPLES = 250_000

# The tunable export defaults -- per-axis qd ceiling, grid dt, axis mask --
# live in data/dense_joint_trajectory.params.json (with their measurements), so a
# robot retune is a data edit, not a code change. Only the format's own
# constants (record layout, validation tolerances) stay in code: those are
# enforced identically by the Go and C++ twins and are the contract, not
# tuning.
PARAMS_SCHEMA = "amr-weld-planner-v1.dense-joint-trajectory-params.v1"
_PARAMS_PATH = (
    pathlib.Path(__file__).resolve().parents[2]
    / "data"
    / "dense_joint_trajectory.params.json"
)


@functools.lru_cache(maxsize=1)
def load_params() -> dict:
    """The export defaults. Fails loudly on a missing or mis-schemed file:
    silently falling back to baked-in numbers is how a retune gets lost."""
    document = json.loads(_PARAMS_PATH.read_text())
    if document.get("schema") != PARAMS_SCHEMA:
        raise DenseJointTrajectoryError(
            "params_invalid",
            f"{_PARAMS_PATH} schema {document.get('schema')!r} != {PARAMS_SCHEMA}",
        )
    params = document["params"]
    limits = params["max_abs_qd_rad_s"]
    if len(limits) != AXIS_COUNT:
        raise DenseJointTrajectoryError(
            "params_invalid",
            f"max_abs_qd_rad_s has {len(limits)} values, need {AXIS_COUNT}",
        )
    return params

# A segment's first and last sample must be (numerically) at rest: the
# executor holds between segments with qd = 0, and a nonzero boundary qd
# means the follower would step the commanded velocity at the splice.
BOUNDARY_QD_EPS_RAD_S = 1e-6

# Consecutive segments must hand over in place. A gap larger than this makes
# the follower slew the jump at the qd clamp -- uncommanded motion at up to
# 3 rad/s -- so it is refused at ingest, not smoothed over.
BOUNDARY_Q_EPS_RAD = 1e-3

# The per-sample grid must be the declared dt. 1e-9 s is far below any real
# retiming and only tolerates binary64 accumulation error.
TIME_GRID_EPS_S = 1e-9

# A per-sample position step may exceed limit*dt only by interpolation slack.
Q_STEP_SLACK = 1.25

SEGMENT_KINDS = ("freespace", "weld", "dwell")

FLAG_TORCH_ON = 0x01


@dataclass(frozen=True)
class DenseSegment:
    """One motion segment: the planner's own per-segment wire, radians.

    `t_s` is segment-local and starts at 0.0. `q_rad` / `qd_rad_s` are
    per-sample rows of AXIS_COUNT values. `torch_on` may only be set on
    `kind == "weld"` segments.
    """

    kind: str
    source_id: str
    t_s: Sequence[float]
    q_rad: Sequence[Sequence[float]]
    qd_rad_s: Sequence[Sequence[float]]
    torch_on: Sequence[bool]
    continuous_weld: str = ""


@dataclass(frozen=True)
class DensePlanIdentity:
    """Play-fencing identity. The daemon matches every field exactly at play;
    none of it is part of the content digest."""

    plan_id: str
    program_id: str
    program_digest: str
    manifest_revision: int
    plan_revision: int
    created_at: str


@dataclass(frozen=True)
class RobotCell:
    """What the plan was made against (`docs/ROBOT-DESCRIPTION-CONTRACT.md`):
    the description's git tree hash and the sha256 of the target's `machine_planning_calibration.json`.
    The daemon refuses a plan whose block differs from its own, or that has
    none. Not part of the content digest -- same split as `DensePlanIdentity`."""

    model_id: str
    robot_description_sha256: str
    machine_planning_calibration_sha256: str
    machine_planning_calibration_source: str  # "target" | "empty"
    # "sha256:" + the rt-core configuration_sha256 of the machine the plan was
    # made for. Recorded; the cell never refuses a plan over it. None offline.
    machine_configuration_sha256: str | None = None

    def header(self) -> dict:
        block = {"model_id": self.model_id, "robot_description_sha256": self.robot_description_sha256,
                 "machine_planning_calibration_sha256": self.machine_planning_calibration_sha256,
                 "machine_planning_calibration_source": self.machine_planning_calibration_source}
        if self.machine_configuration_sha256:
            block["machine_configuration_sha256"] = self.machine_configuration_sha256
        return block


@dataclass(frozen=True)
class DensePlan:
    identity: DensePlanIdentity
    segments: Sequence[DenseSegment]
    # None writes no `robot` block, which keeps the reference fixture's bytes;
    # the daemon refuses such a plan at play (`robot_cell_missing`).
    robot: RobotCell | None = None
    start_state: dict | None = None
    # rt-core dense `process_markers` ({segment, sample, action, value}) from
    # the program's `io` rows (weldplan.io_schedule); empty writes none.
    process_markers: Sequence[dict] = ()
    dt_s: float = field(default_factory=lambda: float(load_params()["dt_s"]))
    axis_mask: int = field(default_factory=lambda: int(load_params()["axis_mask"]))
    max_abs_qd_rad_s: Sequence[float] = field(
        default_factory=lambda: tuple(
            float(v) for v in load_params()["max_abs_qd_rad_s"]
        )
    )


class DenseJointTrajectoryError(ValueError):
    """Raised with `reason` (the daemon's rejection vocabulary) and, where a
    segment is at fault, its index -- so producer-side failures read exactly
    like an ingest rejection would."""

    def __init__(self, reason: str, detail: str, segment_index: int | None = None):
        self.reason = reason
        self.detail = detail
        self.segment_index = segment_index
        where = "" if segment_index is None else f" segment {segment_index}:"
        super().__init__(f"{reason}:{where} {detail}")


def _u64(h, value: int) -> None:
    h.update(struct.pack(">Q", value))


def _f64(h, value: float) -> None:
    h.update(struct.pack(">d", value))


def _str(h, value: str) -> None:
    raw = value.encode("utf-8")
    _u64(h, len(raw))
    h.update(raw)


def encode_block(segment: DenseSegment) -> bytes:
    """Pack one segment's samples into the canonical 153-byte records."""
    records = []
    for i in range(len(segment.t_s)):
        flags = FLAG_TORCH_ON if segment.torch_on[i] else 0
        records.append(
            _RECORD_STRUCT.pack(
                float(segment.t_s[i]),
                *(float(v) for v in segment.q_rad[i]),
                *(float(v) for v in segment.qd_rad_s[i]),
                flags,
            )
        )
    return b"".join(records)


def trajectory_digest(plan: DensePlan, blocks: Sequence[bytes]) -> str:
    """Content digest: what the trajectory *is*, not what it is called.

    Hashes the versioned big-endian layout, with each segment's block bytes
    verbatim -- the wire bytes are the digest input, so two runtimes cannot
    disagree by formatting a float differently. Identity strings are excluded;
    they are matched separately at play. After each segment's block comes room
    for optional tagged sections (u8 tag + u64 length + content, hashed only
    when present -- the same domain separation that let qd join the accepted
    plan digest without moving classic digests). v1 defines none; a future
    weld-process table takes tag 0x01 without a version bump.
    """
    h = hashlib.sha256()
    h.update(DIGEST_DOMAIN)
    _u64(h, AXIS_COUNT)
    _u64(h, plan.axis_mask)
    _f64(h, plan.dt_s)
    _u64(h, len(plan.segments))
    for segment, block in zip(plan.segments, blocks):
        _str(h, segment.kind)
        _str(h, segment.source_id)
        _u64(h, len(segment.t_s))
        h.update(block)
        # v1: no optional sections.
    _u64(h, AXIS_COUNT)
    for value in plan.max_abs_qd_rad_s:
        _f64(h, float(value))
    return "sha256:" + h.hexdigest()


def _validate(plan: DensePlan) -> None:
    limits = [float(v) for v in plan.max_abs_qd_rad_s]
    if len(limits) != AXIS_COUNT or not all(
        v > 0 and v == v and v != float("inf") for v in limits  # noqa: PLR0124 -- the NaN test
    ):
        raise DenseJointTrajectoryError(
            "limits_invalid", f"max_abs_qd_rad_s must be {AXIS_COUNT} finite positives"
        )
    if not plan.segments:
        raise DenseJointTrajectoryError("segments_empty", "a plan needs at least one segment")
    if not (plan.dt_s > 0):
        raise DenseJointTrajectoryError("time_grid_invalid", f"dt_s {plan.dt_s} must be > 0")

    total = 0
    previous_last_q: Sequence[float] | None = None
    for index, segment in enumerate(plan.segments):
        if segment.kind not in SEGMENT_KINDS:
            raise DenseJointTrajectoryError(
                "kind_invalid", f"kind {segment.kind!r}", index
            )
        count = len(segment.t_s)
        if count < 2:
            raise DenseJointTrajectoryError(
                "segment_too_short", f"{count} samples; a segment needs at least 2", index
            )
        if not (
            len(segment.q_rad) == count
            and len(segment.qd_rad_s) == count
            and len(segment.torch_on) == count
        ):
            raise DenseJointTrajectoryError(
                "block_layout_invalid",
                f"t/q/qd/torch lengths differ ({count} times)",
                index,
            )
        total += count
        for i in range(count):
            row_q = segment.q_rad[i]
            row_qd = segment.qd_rad_s[i]
            if len(row_q) != AXIS_COUNT or len(row_qd) != AXIS_COUNT:
                raise DenseJointTrajectoryError(
                    "block_layout_invalid", f"sample {i} is not {AXIS_COUNT}-axis", index
                )
            values = [segment.t_s[i], *row_q, *row_qd]
            if any(v != v or v in (float("inf"), float("-inf")) for v in values):  # noqa: PLR0124 -- the NaN test
                raise DenseJointTrajectoryError(
                    "nonfinite_sample", f"sample {i} carries NaN/Inf", index
                )
            if segment.torch_on[i] and segment.kind != "weld":
                raise DenseJointTrajectoryError(
                    "torch_outside_weld",
                    f"torch bit set at sample {i} of a {segment.kind} segment",
                    index,
                )
            for axis in range(AXIS_COUNT):
                if abs(row_qd[axis]) > limits[axis]:
                    raise DenseJointTrajectoryError(
                        "qd_limit_exceeded",
                        f"sample {i} axis {axis}: |{row_qd[axis]:.6f}| rad/s "
                        f"> limit {limits[axis]:.6f} rad/s",
                        index,
                    )
        if abs(segment.t_s[0]) > 0.0:
            raise DenseJointTrajectoryError(
                "time_grid_invalid", f"t_s[0] is {segment.t_s[0]}, not 0.0", index
            )
        for i in range(1, count):
            step = segment.t_s[i] - segment.t_s[i - 1]
            if abs(step - plan.dt_s) > TIME_GRID_EPS_S and not (segment.continuous_weld and 0 < step <= plan.dt_s + TIME_GRID_EPS_S):
                raise DenseJointTrajectoryError(
                    "time_grid_invalid",
                    f"t step {step:.12f} s at sample {i} vs dt_s {plan.dt_s} s",
                    index,
                )
            for axis in range(AXIS_COUNT):
                dq = abs(segment.q_rad[i][axis] - segment.q_rad[i - 1][axis])
                allowed = Q_STEP_SLACK * limits[axis] * plan.dt_s
                if dq > allowed:
                    raise DenseJointTrajectoryError(
                        "q_step_exceeded",
                        f"sample {i} axis {axis}: |dq| {dq:.6f} rad "
                        f"> {allowed:.6f} rad ({Q_STEP_SLACK} * limit * dt)",
                        index,
                    )
        for boundary in (0, count - 1):
            for axis in range(AXIS_COUNT):
                qd = segment.qd_rad_s[boundary][axis]
                neighbor_index = index - 1 if boundary == 0 else index + 1
                connected = bool(segment.continuous_weld) and 0 <= neighbor_index < len(plan.segments) and plan.segments[neighbor_index].continuous_weld == segment.continuous_weld
                if connected:
                    neighbor = plan.segments[neighbor_index]
                    other = -1 if boundary == 0 else 0
                    if abs(qd-neighbor.qd_rad_s[other][axis]) > 1e-12 or abs(segment.q_rad[boundary][axis]-neighbor.q_rad[other][axis]) > 1e-12:
                        raise DenseJointTrajectoryError("continuous_weld_discontinuity", "weld boundary must match position and velocity", index)
                elif abs(qd) > BOUNDARY_QD_EPS_RAD_S:
                    raise DenseJointTrajectoryError(
                        "boundary_qd_nonzero",
                        f"sample {boundary} axis {axis}: |{qd:.9f}| rad/s "
                        f"> {BOUNDARY_QD_EPS_RAD_S} rad/s",
                        index,
                    )
        if previous_last_q is not None:
            for axis in range(AXIS_COUNT):
                gap = abs(segment.q_rad[0][axis] - previous_last_q[axis])
                if gap > BOUNDARY_Q_EPS_RAD:
                    raise DenseJointTrajectoryError(
                        "boundary_q_discontinuity",
                        f"axis {axis}: {gap:.6f} rad gap from previous segment "
                        f"> {BOUNDARY_Q_EPS_RAD} rad",
                        index,
                    )
        previous_last_q = segment.q_rad[count - 1]
    if total > MAX_TOTAL_SAMPLES:
        raise DenseJointTrajectoryError(
            "sample_count_overflow", f"{total} samples > {MAX_TOTAL_SAMPLES}"
        )
    # rt-core's admission of header markers (adapters/rosie/control/program.go,
    # io_marker_invalid), run here first: an exact source sample, a named
    # output, and no two markers for one output on one sample.
    seen = set()
    for marker in plan.process_markers:
        segment, sample, action = marker.get("segment"), marker.get("sample"), marker.get("action")
        if (type(segment) is not int or not 0 <= segment < len(plan.segments) or type(sample) is not int
                or not 0 <= sample < len(plan.segments[segment].t_s) or not isinstance(action, str) or not action
                or type(marker.get("value")) is not bool):
            raise DenseJointTrajectoryError("io_marker_invalid", f"marker {marker!r}")
        if (segment, sample, action) in seen:
            raise DenseJointTrajectoryError("io_marker_invalid", f"two markers for {action!r} on one sample", segment)
        seen.add((segment, sample, action))


def encode(plan: DensePlan) -> bytes:
    """Validate, then emit the blob. Raises DenseJointTrajectoryError on any rule
    the daemon's ingest would refuse."""
    _validate(plan)
    blocks = [encode_block(segment) for segment in plan.segments]
    digest = trajectory_digest(plan, blocks)

    offset = 0
    segment_entries = []
    for index, (segment, block) in enumerate(zip(plan.segments, blocks)):
        entry = {
            "index": index,
            "kind": segment.kind,
            "source_id": segment.source_id,
            **({"continuous_weld":segment.continuous_weld} if segment.continuous_weld else {}),
            "sample_count": len(segment.t_s),
            "duration_s": float(segment.t_s[-1]),
            "block": {
                "byte_offset": offset,
                "byte_length": len(block),
                "sha256": "sha256:" + hashlib.sha256(block).hexdigest(),
            },
        }
        segment_entries.append(entry)
        offset += len(block)

    header = {
        "schema": SCHEMA,
        "plan_id": plan.identity.plan_id,
        "program_id": plan.identity.program_id,
        "program_digest": plan.identity.program_digest,
        "trajectory_digest": digest,
        "manifest_revision": plan.identity.manifest_revision,
        "plan_revision": plan.identity.plan_revision,
        "created_at": plan.identity.created_at,
        "axes": {
            "axis_count": AXIS_COUNT,
            "axis_mask": plan.axis_mask,
            "position_unit": "rad",
            "velocity_unit": "rad_s",
        },
        "sampling": {
            "dt_s": plan.dt_s,
            "total_sample_count": sum(len(s.t_s) for s in plan.segments),
            "total_duration_s": sum(float(s.t_s[-1]) for s in plan.segments),
        },
        "limits": {
            "max_abs_qd_rad_s": [float(v) for v in plan.max_abs_qd_rad_s],
        },
        **({"robot_cell": plan.robot.header()} if plan.robot is not None else {}),
        **({"start_state": plan.start_state} if plan.start_state is not None else {}),
        # Header-only, outside the v1 trajectory digest (the samples' digest
        # rt-core recomputes), so the schedule is bound by its own digest:
        # the OLP refuses a Load whose markers do not hash to it.
        **({"process_markers": [dict(m) for m in plan.process_markers],
            "process_markers_sha256": io_schedule.markers_digest([dict(m) for m in plan.process_markers])}
           if plan.process_markers else {}),
        "segments": segment_entries,
        "sample_encoding": {
            "encoding": SAMPLE_ENCODING,
            "record_bytes": RECORD_BYTES,
        },
    }
    header_bytes = json.dumps(
        header, separators=(",", ":"), sort_keys=False
    ).encode("utf-8")
    return b"".join(
        [struct.pack(">I", len(header_bytes)), header_bytes, *blocks]
    )


# --- TCP client -------------------------------------------------------------

# Framing shared with the collision server: [u64 BE payload size][payload],
# one request frame, one response frame, close. The daemon's payload shape is
# [u32 BE json_len][JSON][optional binary body].
MAX_FRAME_BYTES = 64 * 1024 * 1024


def _send_frame(sock: socket.socket, payload: bytes) -> None:
    if len(payload) > MAX_FRAME_BYTES:
        raise DenseJointTrajectoryError(
            "frame_too_large", f"{len(payload)} bytes > {MAX_FRAME_BYTES}"
        )
    sock.sendall(struct.pack(">Q", len(payload)) + payload)


def _recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks = []
    remaining = count
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise DenseJointTrajectoryError(
                "connection_closed", f"peer closed with {remaining} bytes unread"
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _recv_frame(sock: socket.socket) -> bytes:
    (size,) = struct.unpack(">Q", _recv_exact(sock, 8))
    if size > MAX_FRAME_BYTES:
        raise DenseJointTrajectoryError(
            "frame_too_large", f"response frame {size} bytes > {MAX_FRAME_BYTES}"
        )
    return _recv_exact(sock, size)


def request(host: str, port: int, kind: str, blob: bytes = b"",
            timeout_s: float = 240.0, **fields) -> dict:
    """One request/response exchange with the daemon's TCP ingest."""
    control = json.dumps({"kind": kind, **fields},
                         separators=(",", ":")).encode("utf-8")
    payload = struct.pack(">I", len(control)) + control + blob
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        _send_frame(sock, payload)
        response = _recv_frame(sock)
    (json_len,) = struct.unpack(">I", response[:4])
    return json.loads(response[4:4 + json_len].decode("utf-8"))


def upload(host: str, port: int, blob: bytes) -> dict:
    """Upload a blob. Inert on the daemon side: upload never causes motion."""
    return request(host, port, "upload", blob)


def validate_remote(host: str, port: int, blob: bytes) -> dict:
    """Dry-run: the daemon validates and reports without persisting."""
    return request(host, port, "validate", blob)


# --------------------------------------------------------------------------
# The planner's native result -> a DensePlan.
#
# `amr-weld-planner-v1.motion-plan-result.v1` carries, per segment, the WIRE:
# `(knots_deg, knot_velocity_deg_s, times_s)` -- exact cubic-Hermite data
# (weld_trajopt.trajectory_json / freespace.move_json), so evaluating that
# Hermite reproduces the certified curve to float64. This converter walks the
# joined program in the exporter's own order (each connecting move, then the
# seam it lands on), maps axes to J1..J9 by NAME with the held positioner
# joints filled in, and Hermite-resamples every segment onto the daemon's
# uniform dt grid -- the direct dense path that replaces the lossy
# rows -> playback_step_ms -> re-plan round trip.

JOINT_ORDER = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")
RESULT_SCHEMA = "amr-weld-planner-v1.motion-plan-result.v1"

# The stationary hold framing each weld pass, matching the row exporter's
# WELD_DWELL_MS: the arc strikes and extinguishes at rest, never mid-flight.
DEFAULT_WELD_DWELL_S = 3.0


def _hermite_eval(times, q_knots, qd_knots, s):
    """Cubic Hermite `(q, qd)` at time s over the wire's own knots."""
    import bisect
    k = bisect.bisect_right(times, s) - 1
    k = max(0, min(k, len(times) - 2))
    h = times[k + 1] - times[k]
    if h <= 0.0:
        return list(q_knots[k]), list(qd_knots[k])
    u = (s - times[k]) / h
    u2, u3 = u * u, u * u * u
    b0, b1 = 2 * u3 - 3 * u2 + 1, u3 - 2 * u2 + u
    b2, b3 = -2 * u3 + 3 * u2, u3 - u2
    d0, d1 = 6 * u2 - 6 * u, 3 * u2 - 4 * u + 1
    d2, d3 = -6 * u2 + 6 * u, 3 * u2 - 2 * u
    q = [b0 * q_knots[k][a] + h * b1 * qd_knots[k][a]
         + b2 * q_knots[k + 1][a] + h * b3 * qd_knots[k + 1][a]
         for a in range(AXIS_COUNT)]
    qd = [(d0 * q_knots[k][a] + h * d1 * qd_knots[k][a]
           + d2 * q_knots[k + 1][a] + h * d3 * qd_knots[k + 1][a]) / h
          for a in range(AXIS_COUNT)]
    return q, qd


def _knots_9(entry: dict) -> tuple[list, list, list]:
    """One document segment's wire as J1..J9 radians: `(times, q, qd)` knots.

    Mapped by NAME like the row exporter's `_joints_9`; a held axis rides at
    its held value with zero velocity; an axis nobody names reads zero.
    """
    import math
    names = [str(n) for n in entry.get("axis_names") or []]
    held = {str(k): math.radians(float(v))
            for k, v in (entry.get("held") or {}).items()}
    source = entry.get("trajectory") or entry
    times = [float(t) for t in source.get("times_s") or []]
    knots_deg = source.get("knots_deg") or []
    velocity_deg = source.get("knot_velocity_deg_s") or []
    if len(times) < 2 or len(knots_deg) != len(times) or \
            len(velocity_deg) != len(times):
        raise DenseJointTrajectoryError(
            "result_segment_invalid",
            f"wire needs matching times/knots/velocities, got "
            f"{len(times)}/{len(knots_deg)}/{len(velocity_deg)}")
    index_of = {name: i for i, name in enumerate(names)}
    q_knots, qd_knots = [], []
    for knot, velocity in zip(knots_deg, velocity_deg):
        q_row, qd_row = [], []
        for joint in JOINT_ORDER:
            if joint in index_of:
                q_row.append(math.radians(float(knot[index_of[joint]])))
                qd_row.append(math.radians(float(velocity[index_of[joint]])))
            else:
                q_row.append(held.get(joint, 0.0))
                qd_row.append(0.0)
        q_knots.append(q_row)
        qd_knots.append(qd_row)
    t0 = times[0]
    return [t - t0 for t in times], q_knots, qd_knots


def _time_map(t: float, wire_duration: float, ramp_s: float) -> tuple[float, float]:
    """`(s, ds/dt)`: the wire time played at output time t, with smoothstep
    ramps of `ramp_s` at both ends.

    Outside the ramps the map is identity (the certified curve plays at its
    own speed); inside them the rate rises from 0 to 1 and back with a
    smoothstep, whose integral over a ramp is half its width, so the output
    lasts `wire_duration + ramp_s`. Zero at both ends exactly, which is what
    the daemon's boundary rule wants and what an arc that strikes and stops
    on a still torch wants.
    """
    w = ramp_s
    total = wire_duration + w
    if w <= 0:
        return min(max(t, 0.0), wire_duration), 1.0
    if t <= 0:
        return 0.0, 0.0
    if t >= total:
        return wire_duration, 0.0
    if t < w:
        u = t / w
        return w * (u ** 3 - u ** 4 / 2), 3 * u * u - 2 * u ** 3
    if t <= total - w:
        return w / 2 + (t - w), 1.0
    u = (total - t) / w
    return wire_duration - w * (u ** 3 - u ** 4 / 2), 3 * u * u - 2 * u ** 3


def _resample_segment(kind: str, source_id: str, entry: dict, dt_s: float,
                      torch_on: bool, ramp_s: float = 0.0) -> DenseSegment:
    """Hermite-resample one wire segment onto the uniform dt grid.

    Sample count is ceil(T/dt)+1 and the wire plays on one uniformly
    stretched clock that ends exactly on the last sample: sample i is the
    wire at i dt x `scale` with its velocity x `scale`, scale = T / ((count -
    1) dt) <= 1 (1 when T is a whole number of rows). The joint path is the
    certified one, at most dt/T slower. rt-core interpolates a cubic Hermite
    between rows from their q and qd every cycle (engine/trajectory.hpp), so
    a row inside one wire cubic reproduces it exactly. Until 2026-10-12 the
    last sample was clamped to T instead: rt-core then stretched the wire's
    final sub-dt piece over a whole row, and on a Rosie 1430 arm's `test1` that cubic
    overshot each taught line's end by up to 18.9 um at 250-400 mm/s, past
    the planner's 0.01 mm tube. Boundary velocities are the wire's own pinned
    zeros, written exactly.

    With `ramp_s > 0` the wire is played through `_time_map`: the same
    positions, on the same certified path, but the first and last `ramp_s`
    of the output accelerate from rest and decelerate to rest. This is the
    stand-in for issue 0013 -- a weld pass leaves M5 at travel speed at both
    ends, and the daemon refuses a segment that is not at rest at its ends.
    The wire's own velocity is scaled by the map's rate, so the boundary
    samples are exactly zero and the interior is the wire untouched. It goes
    when M5 consumes an authored run-in/run-out.
    """
    import math
    times, q_knots, qd_knots = _knots_9(entry)
    duration = times[-1]
    ramp = min(max(ramp_s, 0.0), duration)
    total = duration + ramp
    count = max(2, math.ceil(round(total / dt_s, 9)) + 1)
    continuous = entry.get("continuous_weld", "")
    scale = 1.0 if continuous else total / ((count - 1) * dt_s)
    if abs(scale - 1.0) < 1e-12:
        scale = 1.0
    t_out, q_out, qd_out = [], [], []
    for i in range(count):
        t = min(i * dt_s * scale, total)
        s, rate = _time_map(t, duration, ramp)
        q, qd = _hermite_eval(times, q_knots, qd_knots, s)
        t_out.append(round(min(i * dt_s, total) if continuous else i * dt_s, 9))
        q_out.append(q)
        qd_out.append([v * rate * scale for v in qd] if ramp > 0 or scale != 1.0 else qd)
    q_out[0], q_out[-1] = list(q_knots[0]), list(q_knots[-1])
    if ramp > 0:
        qd_out[0] = [0.0] * AXIS_COUNT
        qd_out[-1] = [0.0] * AXIS_COUNT
    else:
        qd_out[0], qd_out[-1] = list(qd_knots[0]), list(qd_knots[-1])
    return DenseSegment(kind=kind, source_id=source_id, t_s=t_out,
                        q_rad=q_out, qd_rad_s=qd_out,
                        torch_on=[torch_on] * count, continuous_weld=continuous)


def _dwell_segment(source_id: str, q: list, dwell_s: float,
                   dt_s: float) -> DenseSegment:
    count = max(2, round(dwell_s / dt_s) + 1)
    return DenseSegment(
        kind="dwell", source_id=source_id,
        t_s=[round(i * dt_s, 9) for i in range(count)],
        q_rad=[list(q)] * count,
        qd_rad_s=[[0.0] * AXIS_COUNT] * count,
        torch_on=[False] * count)


def plan_from_result(result: dict, identity: DensePlanIdentity, *,
                     dt_s: float | None = None,
                     weld_dwell_s: float = DEFAULT_WELD_DWELL_S,
                     weld_ramp_s: float | None = None,
                     max_abs_qd_rad_s: Sequence[float] | None = None,
                     axis_mask: int | None = None,
                     robot: RobotCell | None = None,
                     execution_mode: str = "production",
                     io_outputs: dict | None = None) -> DensePlan:
    """The planner's native result document as a DensePlan.

    `max_abs_qd_rad_s` is the per-axis ceiling written into the header, the
    params default unless the caller knows the cell: the motion server passes
    the cell profile's own velocity limits, so `encode` refuses a plan the
    cell could not play instead of leaving that to the daemon's gate.

    `axis_mask` is the header's claim of WHICH of the nine wire columns this
    plan commands, and the daemon holds the cell to it: the executor refuses a
    plan whose mask names an axis the active pair has not mapped and homed.
    The params default is all nine, which is `rosie_1400_v3` and is wrong for
    a six-axis cell -- on `rosie_1420_v1` the pair maps 0x3f, so a 0x1ff plan
    is refused as "all plan axes must be mapped and homed" although J7..J9
    never move in it. The wire itself is always nine columns wide (`kAxisCount`
    is a constant on every codec); the mask is how six is said. Pass the
    cell's own mask (`dense_output.cell_axis_mask`) when the cell is known.

    Program order is the row exporter's: each connecting move in document
    order, followed by the weld of the seam it lands on; then any crossed
    seam no move reached. Every weld pass is framed by stationary dwell
    segments (torch off) so the arc strikes and extinguishes at rest --
    the dense equivalent of the exporter's WELD_DWELL_MS rows. A seam the
    planner did not cross is refused, not skipped: a dense plan is an
    executable claim, and silently dropping a weld would make it a lie.

    Authored `dry_run` preserves every motion sample and weld segment while
    disabling torch intent. The encoded flags and trajectory digest then
    describe that motion-only artifact. Production, simulation and legacy
    callers retain the existing torch semantics; Load never edits the blob.
    """
    if execution_mode not in ("production", "simulation", "dry_run"):
        raise DenseJointTrajectoryError(
            "execution_mode_invalid", f"unknown execution_mode {execution_mode!r}")
    if result.get("schema") != RESULT_SCHEMA:
        raise DenseJointTrajectoryError(
            "result_schema_mismatch", f"schema {result.get('schema')!r}")
    # A dense plan is an executable claim: only natively verified segments
    # (M5 PASS with a passing tracking certificate, M6 PASS) are resampled.
    try:
        require_native_motion(result)
    except MotionAdmissionError as error:
        raise DenseJointTrajectoryError(error.reason, error.detail) from error
    grid = float(dt_s if dt_s is not None else load_params()["dt_s"])
    # The weld-end ramp (issue 0013 stand-in): from the params file unless
    # the caller says otherwise, so a retune is one number in one place.
    ramp = float(weld_ramp_s if weld_ramp_s is not None
                 else load_params().get("weld_ramp_s", 0.0))
    speed_scale = result.get("speed_scale", 1.0)
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
            or not 0.01 <= speed_scale <= 1.0):
        raise DenseJointTrajectoryError("motion_result_invalid", "Invalid plan speed scale")
    # The wire already carries the scaled clock. Stretch the endpoint ramps
    # by the same factor; keep the controller period and dwell seconds fixed.
    ramp /= speed_scale
    seams = {str(s.get("id")): s for s in result.get("seams") or []}
    segments: list[DenseSegment] = []

    def append_seam(seam: dict) -> None:
        seam_id = str(seam.get("id"))
        if not seam.get("crossed"):
            raise DenseJointTrajectoryError(
                "seam_not_planned", str(seam.get("error") or seam_id))
        weld = _resample_segment("weld", seam_id, seam, grid,
                                 torch_on=execution_mode != "dry_run",
                                 ramp_s=ramp)
        if weld_dwell_s > 0:
            segments.append(_dwell_segment(f"{seam_id}:dwell-in",
                                           weld.q_rad[0], weld_dwell_s, grid))
        segments.append(weld)
        if weld_dwell_s > 0:
            segments.append(_dwell_segment(f"{seam_id}:dwell-out",
                                           weld.q_rad[-1], weld_dwell_s, grid))

    io_events: dict[int, list[tuple[str, bool]]] = {}
    waits, pulse_ends, weld_boundaries = [], {}, {}
    order = result.get("program_order")
    if order is not None:
        from . import taught_weld
        errors=taught_weld.problems(order)
        if errors: raise DenseJointTrajectoryError("weld_program_invalid","; ".join(errors))
        weld_name=None
        if execution_mode!="dry_run" and any(n.get("weld_transition") in ("start","stop") for n in order):
            try: weld_name=taught_weld.assigned_output(io_outputs)
            except ValueError as error: raise DenseJointTrajectoryError("weld_output_unassigned",str(error)) from error
        entries = result.get("connecting_trajectories") or []
        ids = [node.get("id") for node in order]
        move_ids = [move.get("node_id") for move in entries]
        if (not order or any(not isinstance(n, str) or not n for n in ids + move_ids)
                or len(set(ids)) != len(ids) or len(set(move_ids)) != len(move_ids)
                or not set(move_ids).issubset(ids)):
            raise DenseJointTrajectoryError("motion_result_invalid", "Program order and motion node identities disagree")
        moves = {move["node_id"]: move for move in entries}
        for group in taught_weld.spans(order).values():
            if len(group) > 1:
                tags = [moves.get(n["id"],{}).get("continuous_weld") for n in group]
                if not tags[0] or len(set(tags)) != 1:
                    raise DenseJointTrajectoryError("weld_path_not_continuous", "Replan: this weld span was compiled as separate stopped moves")
        if execution_mode != "dry_run" and any(n.get("action") in ("monitor","end_monitor") for n in order):
            issues=io_schedule.io_problems(order,io_outputs)
            if issues: raise DenseJointTrajectoryError("io_monitor_invalid", "; ".join(issues))
        if {n.get("seam_id") for n in order if n["op"] == "weld"} != set(seams):
            raise DenseJointTrajectoryError("motion_result_invalid", "Program order and weld identities disagree")
        def anchor(what: str) -> list:
            if segments:
                return segments[-1].q_rad[-1]
            if not moves:
                raise DenseJointTrajectoryError("result_empty", f"{what} has no established start position")
            first = next(iter(moves.values()))
            return _resample_segment("freespace", "start", first, grid, torch_on=False).q_rad[0]

        for node in order:
            if node["op"] == "weld":
                if node.get("seam_id") not in seams:
                    raise DenseJointTrajectoryError("seam_not_planned", str(node["id"]))
                append_seam(seams[node["seam_id"]])
            elif node["op"] == "dwell":
                if node["duration_s"] > 0:
                    segments.append(_dwell_segment(node["id"], anchor("Dwell"), node["duration_s"], grid))
            elif node["op"] == "io":
                # The row's own short dwell, its output switched on its first
                # sample with the arm at rest (weldplan.io_schedule).
                action=node.get("action","set")
                duration=node.get("duration_s",0) if action=="pulse" else io_schedule.IO_EVENT_S
                if action not in ("set","pulse","wait","monitor","end_monitor") or duration<=0:
                    raise DenseJointTrajectoryError("io_action_invalid",str(node["id"]))
                segments.append(_dwell_segment(node["id"], anchor("An I/O row"), max(duration,2*grid), grid))
                index=len(segments)-1
                if action in ("monitor", "end_monitor"):
                    waits.append({"segment":index,"sample":0,"action":node["channel"],"value":node["state"],
                                  "monitor":"start" if action=="monitor" else "end"})
                elif action=="wait":
                    timeout=round(node.get("timeout_s",0)*1000); stable=round(node.get("stable_s",0)*1000)
                    if timeout<=0 or stable<0 or stable>timeout or timeout>3600000:
                        raise DenseJointTrajectoryError("io_wait_invalid",str(node["id"]))
                    waits.append({"segment":index,"sample":0,"action":node["channel"],"value":node["state"],"timeout_ms":timeout,
                                  **({"stable_ms":stable} if stable else {})})
                else:
                    io_events[index]=[(node["channel"],node["state"])]
                    if action=="pulse": pulse_ends[index]=node["channel"]
            elif node["id"] in moves:
                segments.append(_resample_segment("freespace", node["id"], moves[node["id"]], grid, torch_on=False))
                transition = node.get("weld_transition")
                if transition in ("start", "stop"):
                    delay = node.get("weld_start_delay_s" if transition == "start" else "weld_stop_delay_s", 0)
                    index = len(segments) - 1
                    # Start ON at arrival; stop OFF only after the stationary delay.
                    # Delays live on the immutable plan clock; Stop/fault fencing is unchanged.
                    if weld_name and transition == "start":
                        weld_boundaries[(index, len(segments[index].t_s)-1)] = [(weld_name, True)]
                    if delay > 0:
                        segments.append(_dwell_segment(node["id"] + ":weld-" + transition + "-delay", anchor("Weld delay"), delay, grid))
                    if weld_name and transition == "stop":
                        index = len(segments) - 1
                        weld_boundaries[(index, len(segments[index].t_s)-1)] = [(weld_name, False)]
            elif node["op"] != "home":
                raise DenseJointTrajectoryError("motion_not_planned", str(node["id"]))
    else:
        emitted: set[str] = set()
        for move in result.get("connecting_trajectories") or []:
            # `from`/`to` are seam ids, or absent at the home end of an approach
            # or retract: the segment names that end `home`, which is what the
            # OLP's node mapping reads.
            source_id = f"{move.get('from') or 'home'}->{move.get('to') or 'home'}"
            segments.append(_resample_segment("freespace", source_id, move, grid,
                                              torch_on=False))
            target = str(move.get("to") or "")
            if target in seams and target not in emitted:
                append_seam(seams[target])
                emitted.add(target)
        for seam_id, seam in seams.items():
            if seam_id not in emitted and seam.get("trajectory"):
                append_seam(seam)
                emitted.add(seam_id)
    if not segments:
        raise DenseJointTrajectoryError(
            "result_empty", "the result contains no playable trajectory")
    extra: dict = {}
    if max_abs_qd_rad_s is not None:
        extra["max_abs_qd_rad_s"] = [float(v) for v in max_abs_qd_rad_s]
    if axis_mask is not None:
        mask = int(axis_mask)
        if mask <= 0 or mask >= (1 << AXIS_COUNT):
            raise DenseJointTrajectoryError(
                "axis_mask_invalid",
                f"axis_mask {mask:#x} must name at least one of the {AXIS_COUNT} axes")
        extra["axis_mask"] = mask
    # Output markers, as torch bits: a dry run keeps the motion and switches
    # nothing. The OLP strips header markers for a dry-run Load too.
    if (io_events or waits or weld_boundaries) and execution_mode != "dry_run":
        if io_outputs is None:
            raise DenseJointTrajectoryError("io_outputs_missing", "output rows need the connected cell's outputs")
        try:
            extra["process_markers"] = io_schedule.schedule([len(s.t_s) for s in segments], io_events, io_outputs, grid, pulse_ends, weld_boundaries)
            known_inputs={i.get("name") for i in io_outputs.get("inputs",[])}
            if any(w["action"] not in known_inputs for w in waits): raise ValueError("wait input is not configured")
            extra["process_markers"].extend(waits)
            extra["process_markers"].sort(key=lambda m:(m["segment"],m["sample"],m["action"]))
        except ValueError as error:
            raise DenseJointTrajectoryError("io_schedule_invalid", str(error)) from error
    return DensePlan(identity=identity, segments=segments, dt_s=grid, robot=robot, **extra)
