"""The Python side of the tri-language dense-joint-trajectory contract tests.

The reference fixture in motion-server/joint-trajectory/v1/testdata is the arbiter shared with
the Go twin (offline-programming/v1/internal/densejointtraj) and the C++ daemon
(motion-server/joint-trajectory/v1). This suite proves the encoder reproduces the fixture
byte-for-byte and that every producer-side rejection fires -- in absolute
units, never thresholds derived from the trajectory under test.
"""

import hashlib
import json
import pathlib
import struct
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parents[4]
TESTDATA = REPO / "motion-server" / "joint-trajectory" / "v1" / "testdata"
sys.path.insert(0, str(REPO / "motion-server" / "joint-trajectory" / "v1" / "tools"))

import make_reference_fixture as fixture_tool
from weldplan import dense_joint_trajectory as dtj


def reference_plan():
    delta_free = [0.02, -0.01, 0.005, 0.0, 0.015, -0.02, 0.0, 0.01, 0.0]
    free = fixture_tool.smoothstep_segment(
        "freespace", "move-1", fixture_tool.BASE_Q, delta_free, 5, 0.01
    )
    delta_weld = [0.001, 0.0005, 0.0, -0.001, 0.0, 0.0002, 0.0, 0.0, 0.0]
    weld = fixture_tool.smoothstep_segment(
        "weld",
        "seam-4b1df2",
        free.q_rad[-1],
        delta_weld,
        6,
        0.01,
        torch=[False, True, True, True, True, False],
    )
    return dtj.DensePlan(
        identity=dtj.DensePlanIdentity(
            plan_id="reference-plan-0001",
            program_id="reference-program",
            program_digest="sha256:" + "ab" * 32,
            manifest_revision=7,
            plan_revision=3,
            created_at="2026-09-01T00:00:00Z",
        ),
        segments=[free, weld],
    )


def test_defaults_come_from_the_params_json():
    params = json.loads(
        (REPO / "weld_planner" / "v1" / "data" / "dense_joint_trajectory.params.json")
        .read_text()
    )["params"]
    plan = reference_plan()
    assert plan.dt_s == params["dt_s"]
    assert plan.axis_mask == params["axis_mask"]
    assert list(plan.max_abs_qd_rad_s) == params["max_abs_qd_rad_s"]
    # And they land in the blob header, which is what validators enforce.
    blob = dtj.encode(plan)
    (header_len,) = struct.unpack(">I", blob[:4])
    header = json.loads(blob[4 : 4 + header_len].decode("utf-8"))
    assert header["limits"]["max_abs_qd_rad_s"] == params["max_abs_qd_rad_s"]
    assert header["sampling"]["dt_s"] == params["dt_s"]


def test_encoder_reproduces_reference_fixture_byte_for_byte():
    blob = dtj.encode(reference_plan())
    stored = (TESTDATA / "dense_joint_trajectory_reference.rdt").read_bytes()
    assert blob == stored


def test_digest_matches_expected_sidecar():
    expected = json.loads(
        (TESTDATA / "dense_joint_trajectory_reference.expected.json").read_text()
    )
    plan = reference_plan()
    blocks = [dtj.encode_block(s) for s in plan.segments]
    assert dtj.trajectory_digest(plan, blocks) == expected["trajectory_digest"]
    assert [
        hashlib.sha256(b).hexdigest() for b in blocks
    ] == expected["block_sha256"]


def test_header_is_parseable_and_names_the_blocks():
    blob = dtj.encode(reference_plan())
    (header_len,) = struct.unpack(">I", blob[:4])
    header = json.loads(blob[4 : 4 + header_len].decode("utf-8"))
    assert header["schema"] == dtj.SCHEMA
    assert header["axes"]["axis_count"] == dtj.AXIS_COUNT
    assert header["sample_encoding"]["record_bytes"] == dtj.RECORD_BYTES
    offset = 0
    for entry in header["segments"]:
        assert entry["block"]["byte_offset"] == offset
        assert entry["block"]["byte_length"] == entry["sample_count"] * dtj.RECORD_BYTES
        body = blob[4 + header_len :]
        raw = body[offset : offset + entry["block"]["byte_length"]]
        assert entry["block"]["sha256"] == "sha256:" + hashlib.sha256(raw).hexdigest()
        offset += entry["block"]["byte_length"]
    assert 4 + header_len + offset == len(blob)


def _mutated(mutate):
    plan = reference_plan()
    segments = [
        dtj.DenseSegment(
            kind=s.kind,
            source_id=s.source_id,
            t_s=list(s.t_s),
            q_rad=[list(row) for row in s.q_rad],
            qd_rad_s=[list(row) for row in s.qd_rad_s],
            torch_on=list(s.torch_on),
        )
        for s in plan.segments
    ]
    mutate(segments)
    return dtj.DensePlan(identity=plan.identity, segments=segments)


@pytest.mark.parametrize(
    "reason,mutate",
    [
        # 1e-5 rad/s boundary qd: an order of magnitude over the 1e-6 gate.
        ("boundary_qd_nonzero", lambda s: s[0].qd_rad_s[0].__setitem__(3, 1e-5)),
        # 0.002 rad handover gap: twice the 1e-3 rad tolerance.
        ("boundary_q_discontinuity", lambda s: s[1].q_rad[0].__setitem__(
            0, s[1].q_rad[0][0] + 0.002)),
        # 3.5 rad/s against the 3.0 rad/s ceiling.
        ("qd_limit_exceeded", lambda s: s[0].qd_rad_s[2].__setitem__(0, 3.5)),
        # 0.05 rad in one 0.01 s step: over 1.25 * 3.0 * 0.01 = 0.0375 rad.
        ("q_step_exceeded", lambda s: s[0].q_rad[2].__setitem__(
            0, s[0].q_rad[1][0] + 0.05)),
        ("torch_outside_weld", lambda s: s[0].torch_on.__setitem__(2, True)),
        ("nonfinite_sample", lambda s: s[0].q_rad[2].__setitem__(4, float("nan"))),
        # A 1 ms grid slip: 100x the 1e-9 s tolerance.
        ("time_grid_invalid", lambda s: s[0].t_s.__setitem__(2, s[0].t_s[2] + 0.001)),
    ],
)
def test_encode_refuses(reason, mutate):
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.encode(_mutated(mutate))
    assert raised.value.reason == reason


def test_encode_refuses_single_sample_segment():
    plan = reference_plan()
    stub = dtj.DenseSegment(
        kind="dwell",
        source_id="hold-1",
        t_s=[0.0],
        q_rad=[list(plan.segments[-1].q_rad[-1])],
        qd_rad_s=[[0.0] * dtj.AXIS_COUNT],
        torch_on=[False],
    )
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.encode(dtj.DensePlan(identity=plan.identity,
                                 segments=[*plan.segments, stub]))
    assert raised.value.reason == "segment_too_short"


def test_sample_count_overflow_refused():
    # 250_001 claimed samples must refuse before any per-sample work; build
    # the count check input cheaply by lying about one long dwell.
    plan = reference_plan()
    n = dtj.MAX_TOTAL_SAMPLES + 1
    q = list(plan.segments[-1].q_rad[-1])
    dwell = dtj.DenseSegment(
        kind="dwell",
        source_id="hold-long",
        t_s=[round(i * 0.01, 9) for i in range(n)],
        q_rad=[q] * n,
        qd_rad_s=[[0.0] * dtj.AXIS_COUNT] * n,
        torch_on=[False] * n,
    )
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.encode(dtj.DensePlan(identity=plan.identity, segments=[dwell]))
    assert raised.value.reason == "sample_count_overflow"


# --------------------------------------------------------------------------
# plan_from_result: the planner's native result document -> DensePlan.
# The synthetic document below reproduces the real emission faithfully:
# degrees rounded to 4dp, per-segment linspace grids at the 0.2 s bench wire,
# axis_names naming only the moving axes with the positioner held, and
# boundary velocities pinned to zero (weld_trajopt's own construction).

import math


def _smooth_wire(q0_deg, q1_deg, duration_s, samples):
    """A minimum-jerk hop emitted the way the planner emits wires: the
    Hermite's own exact samples, rounded to 4dp degrees."""
    times = [duration_s * i / (samples - 1) for i in range(samples)]
    knots, velocities = [], []
    for t in times:
        u = t / duration_s
        s = 3 * u * u - 2 * u ** 3
        sd = (6 * u - 6 * u * u) / duration_s
        knots.append([round(a + (b - a) * s, 4) for a, b in zip(q0_deg, q1_deg)])
        velocities.append([round((b - a) * sd, 4) for a, b in zip(q0_deg, q1_deg)])
    velocities[0] = [0.0] * len(q0_deg)
    velocities[-1] = [0.0] * len(q0_deg)
    return {
        "knots_deg": knots,
        "knot_velocity_deg_s": velocities,
        "times_s": [round(t, 4) for t in times],
    }


def _native_result():
    # Synthetic native reports exercise conversion, not planner qualification.
    clear = {"verdict": "PASS", "collisions": 0, "penetrating": 0,
             "limit_violations": 0, "unverifiable": []}
    arm = ["J1", "J2", "J3", "J4", "J5", "J6"]
    held = {"J7": 12.5, "J8": -3.0, "J9": 0.0}
    home = [10.0, -20.0, 30.0, 0.0, 15.0, -5.0]
    weld_start = [12.0, -21.0, 31.0, 1.0, 16.0, -6.0]
    weld_end = [12.5, -21.2, 31.1, 1.2, 16.1, -6.1]
    approach = {
        "kind": "approach", "from": "home", "to": "seam-1",
        "axis_names": arm, "held": held,
        **_smooth_wire(home, weld_start, 4.0, 21),  # the 0.2 s bench wire
        "verdict": dict(clear),
    }
    retract = {
        "kind": "retract", "from": "seam-1", "to": "home",
        "axis_names": arm, "held": held,
        **_smooth_wire(weld_end, home, 4.0, 21),
        "verdict": dict(clear),
    }
    seam = {
        "id": "seam-1", "crossed": True,
        "axis_names": arm, "held": held,
        "trajectory": {
            **_smooth_wire(weld_start, weld_end, 5.0, 26),
            "verdict": dict(clear),
            "tracking": {"verdict": "PASS", "bound_mm": 0.1,
                         "tolerance_mm": 0.5, "segments_out": 0,
                         "undecided": 0, "unverifiable": []},
        },
    }
    return {
        "schema": "amr-weld-planner-v1.motion-plan-result.v1",
        "seams": [seam],
        "connecting_trajectories": [approach, retract],
    }


def _result_identity():
    return dtj.DensePlanIdentity(
        plan_id="native-plan-1", program_id="native-program",
        program_digest="sha256:" + "12" * 32,
        manifest_revision=1, plan_revision=1,
        created_at="2026-09-02T00:00:00Z")


def test_plan_from_result_converts_and_encodes():
    plan = dtj.plan_from_result(_native_result(), _result_identity())
    blob = dtj.encode(plan)  # runs the daemon's full validation
    assert blob[:4]
    kinds = [s.kind for s in plan.segments]
    assert kinds == ["freespace", "dwell", "weld", "dwell", "freespace"]
    # Torch bits only inside the weld, on for the whole pass.
    for segment in plan.segments:
        if segment.kind == "weld":
            assert all(segment.torch_on)
        else:
            assert not any(segment.torch_on)


def test_plan_from_result_maps_axes_by_name_and_holds_positioner():
    plan = dtj.plan_from_result(_native_result(), _result_identity())
    approach = plan.segments[0]
    # First sample is the document's own first knot: J1 = 10 deg.
    assert abs(approach.q_rad[0][0] - math.radians(10.0)) < 1e-9
    # Held positioner axes ride at their held value with zero velocity.
    for sample_q, sample_qd in zip(approach.q_rad, approach.qd_rad_s):
        assert abs(sample_q[6] - math.radians(12.5)) < 1e-9
        assert abs(sample_q[7] - math.radians(-3.0)) < 1e-9
        assert sample_qd[6] == 0.0 and sample_qd[7] == 0.0


def test_dry_run_preserves_every_motion_sample_but_has_distinct_output_identity():
    original = dtj.plan_from_result(_native_result(), _result_identity())
    dry = dtj.plan_from_result(_native_result(), _result_identity(), execution_mode="dry_run")
    assert dry.identity == original.identity
    assert dry.dt_s == original.dt_s
    assert len(dry.segments) == len(original.segments)
    for before, after in zip(original.segments, dry.segments):
        assert (before.kind, before.source_id) == (after.kind, after.source_id)
        assert before.t_s == after.t_s
        assert before.q_rad == after.q_rad
        assert before.qd_rad_s == after.qd_rad_s
        assert not any(after.torch_on)
    assert any(any(segment.torch_on) for segment in original.segments)
    def digest(plan):
        blob = dtj.encode(plan)
        return json.loads(blob[4:4 + int.from_bytes(blob[:4], "big")])["trajectory_digest"]
    assert digest(dry) != digest(original)
    for mode in ("production", "simulation"):
        assert digest(dtj.plan_from_result(_native_result(), _result_identity(), execution_mode=mode)) == digest(original)


@pytest.mark.parametrize("mode", ["", "dry-run", "off", None, 0])
def test_export_refuses_unknown_execution_modes(mode):
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.plan_from_result(_native_result(), _result_identity(), execution_mode=mode)
    assert raised.value.reason == "execution_mode_invalid"


def test_plan_from_result_resamples_wire_to_daemon_grid():
    # Source wire is 0.2 s; the plan grid is the params dt (0.01 s):
    # 4.0 s / 0.01 s + 1 = 401 samples on an exact grid, endpoints the
    # wire's own pinned knots.
    plan = dtj.plan_from_result(_native_result(), _result_identity())
    approach = plan.segments[0]
    assert len(approach.t_s) == 401
    assert approach.t_s[1] - approach.t_s[0] == 0.01
    assert all(abs(v) < 1e-9 for v in approach.qd_rad_s[0])
    assert all(abs(v) < 1e-9 for v in approach.qd_rad_s[-1])
    # Interior velocity is the Hermite's own derivative: the minimum-jerk
    # peak 1.5*dq/T on J1 (2 deg over 4 s -> 0.75 deg/s = 0.0131 rad/s).
    peak = max(s[0] for s in approach.qd_rad_s)
    assert abs(peak - math.radians(1.5 * 2.0 / 4.0)) < 2e-4


def test_rt_core_cycles_reproduce_the_wire_on_one_clock_and_never_pass_its_end():
    """A wire whose duration is not a whole number of rows: rt-core's cycle
    sampler (engine/trajectory.hpp: cubic Hermite between rows from q and qd,
    every 1 ms) must play the certified curve itself, on one uniformly
    stretched clock that ends on the last row. A last row clamped to the
    wire's end made that cubic overshoot the end (18.9 um of TCP on the 1430 cell's
    test1 at 250-400 mm/s, 2026-10-12)."""
    q0, q1, duration = 0.1, 0.6, 1.003  # rad, rad, s: one rest-to-rest cubic
    wire = {"axis_names": ["J1"], "held": {}, "times_s": [0.0, duration],
            "knots_deg": [[math.degrees(q0)], [math.degrees(q1)]], "knot_velocity_deg_s": [[0.0], [0.0]]}
    segment = dtj._resample_segment("freespace", "m", wire, 0.01, torch_on=False)
    assert len(segment.t_s) == 102 and segment.t_s[-1] == pytest.approx(1.01, abs=1e-12)
    assert segment.q_rad[-1][0] == pytest.approx(q1, abs=1e-15)
    scale = duration / 1.01

    def curve(t):
        u = t / duration
        return q0 + (q1 - q0) * u * u * (3 - 2 * u)

    t = [round(i * 0.001, 9) for i in range(1011)]
    for cycle in t:
        k = min(int(round(cycle * 1e9)) // 10_000_000, len(segment.t_s) - 2)
        span = segment.t_s[k + 1] - segment.t_s[k]
        alpha = (cycle - segment.t_s[k]) / span
        a, b = segment.q_rad[k][0], segment.q_rad[k + 1][0]
        va, vb = segment.qd_rad_s[k][0], segment.qd_rad_s[k + 1][0]
        delta = b - a
        c2 = 3 * delta - span * (2 * va + vb)
        c3 = -2 * delta + span * (va + vb)
        commanded = a + alpha * (span * va + alpha * (c2 + alpha * c3))
        assert commanded <= q1 + 1e-12
        assert commanded == pytest.approx(curve(cycle * scale), abs=1e-12)


def test_plan_from_result_refuses_uncrossed_seam():
    result = _native_result()
    result["seams"][0]["crossed"] = False
    result["seams"][0]["error"] = "unreachable"
    result["seams"][0]["frontier"] = 1
    result["seams"][0]["fractions"] = [0, 0.5, 1]
    result["seams"][0]["frontier_causes"] = {"collision": 8}
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.plan_from_result(result, _result_identity())
    assert raised.value.reason == "seam_not_planned"
    assert "unreachable" in raised.value.detail
    assert "sample 2 of 3 (50%" in raised.value.detail
    assert "8 poses rejected by sphere collision screening" in raised.value.detail


def test_plan_from_result_refuses_wrong_schema():
    result = _native_result()
    result["schema"] = "something-else.v1"
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.plan_from_result(result, _result_identity())
    assert raised.value.reason == "result_schema_mismatch"


# --------------------------------------------------------------------------
# The weld-end ramp (issue 0013 stand-in). A real M5 weld leaves and arrives
# at travel speed; the daemon refuses that. The export plays the weld
# through a time map that is at rest at both ends and the wire in between.


def _travel_speed_wire(q0_deg, q1_deg, duration_s, samples):
    """A constant-velocity pass: the shape M5 actually emits at the ends."""
    times = [duration_s * i / (samples - 1) for i in range(samples)]
    velocity = [(b - a) / duration_s for a, b in zip(q0_deg, q1_deg)]
    return {
        "knots_deg": [[round(a + (b - a) * t / duration_s, 4) for a, b in zip(q0_deg, q1_deg)]
                      for t in times],
        "knot_velocity_deg_s": [[round(v, 4) for v in velocity] for _ in times],
        "times_s": [round(t, 4) for t in times],
    }


def _travel_speed_result():
    result = _native_result()
    verified = result["seams"][0]["trajectory"]
    result["seams"][0]["trajectory"] = {
        **_travel_speed_wire(
            [12.0, -21.0, 31.0, 1.0, 16.0, -6.0], [12.5, -21.2, 31.1, 1.2, 16.1, -6.1], 5.0, 26),
        # The native verdicts stay: the ramp is judged on the daemon's
        # boundary rule, not refused earlier by the admission gate.
        "verdict": verified["verdict"], "tracking": verified["tracking"],
    }
    return result


def test_a_weld_at_travel_speed_is_refused_without_the_ramp():
    plan = dtj.plan_from_result(_travel_speed_result(), _result_identity(), weld_ramp_s=0.0)
    with pytest.raises(dtj.DenseJointTrajectoryError) as raised:
        dtj.encode(plan)
    assert raised.value.reason == "boundary_qd_nonzero"


def test_the_ramp_brings_the_weld_to_rest_at_both_ends_on_the_certified_path():
    """Positions are the wire's own, at a shifted time; velocities are the
    wire's scaled by the map's rate; the ends are exactly at rest; the pass
    lasts ramp_s longer; the params file supplies the ramp."""
    assert dtj.load_params()["weld_ramp_s"] == 0.25
    plan = dtj.plan_from_result(_travel_speed_result(), _result_identity())
    dtj.encode(plan)  # the daemon's own ruleset, no rejection
    weld = next(s for s in plan.segments if s.kind == "weld")
    assert abs(weld.t_s[-1] - (5.0 + 0.25)) < 1e-9
    assert all(v == 0.0 for v in weld.qd_rad_s[0])
    assert all(v == 0.0 for v in weld.qd_rad_s[-1])
    # Mid-pass the wire plays untouched: at output time 2.0 + 0.125 the
    # wire is at its own 2.0 s, with its own velocity.
    i = round((2.0 + 0.125) / 0.01)
    wire = _travel_speed_result()["seams"][0]["trajectory"]
    expected_q = math.radians(12.0 + 0.5 * 2.0 / 5.0)
    expected_qd = math.radians(0.5 / 5.0)
    assert abs(weld.q_rad[i][0] - expected_q) < 5e-5  # the wire is 4 dp degrees
    assert abs(weld.qd_rad_s[i][0] - expected_qd) < 5e-5
    assert weld.q_rad[0][0] == math.radians(wire["knots_deg"][0][0])
    # Inside the ramp the position is behind the identity map and the rate
    # is below one, monotone in both.
    j = round(0.1 / 0.01)
    assert weld.q_rad[j][0] < weld.q_rad[i][0]
    assert 0.0 < weld.qd_rad_s[j][0] < expected_qd
    assert all(weld.torch_on)


def test_a_ramp_of_zero_reproduces_the_pinned_fixture_exactly():
    zero = dtj.plan_from_result(_native_result(), _result_identity(), weld_ramp_s=0.0)
    default = dtj.plan_from_result(_native_result(), _result_identity())
    unramped = next(s for s in zero.segments if s.kind == "weld")
    ramped = next(s for s in default.segments if s.kind == "weld")
    assert len(ramped.t_s) == len(unramped.t_s) + 25
    assert ramped.q_rad[0] == unramped.q_rad[0]
    assert ramped.q_rad[-1] == unramped.q_rad[-1]


def test_ten_percent_plan_keeps_path_and_dwell_but_stretches_motion_and_ramps():
    from types import SimpleNamespace
    from unittest.mock import patch
    from weld_motion_planner.io.native_result import result_document

    result = _travel_speed_result()
    motion = SimpleNamespace(seam_search_result=SimpleNamespace(screened=True),
        weld_trajectories=[], connecting_trajectories=result["connecting_trajectories"],
        connecting_trajopt_error=None, options=SimpleNamespace(as_json=lambda: {}), timings_ms={})
    request = SimpleNamespace(program={"speed_scale": 0.1}, request_id="speed-test",
                              manifest={}, cell={}, placement={})
    with patch("weld_motion_planner.io.native_result.as_json", return_value=result):
        slowed = result_document(request, motion, request_sha256="1" * 64, source_revision=None)
    baseline = dtj.plan_from_result(result, _result_identity())
    scaled = dtj.plan_from_result(slowed, _result_identity())
    assert scaled.dt_s == baseline.dt_s
    dtj.encode(scaled)
    for before, after in zip(baseline.segments, scaled.segments):
        if before.kind == "dwell":
            assert after == before
            continue
        assert after.t_s[-1] == pytest.approx(before.t_s[-1] * 10)
        for i in range(len(before.t_s)):
            assert after.q_rad[i * 10] == pytest.approx(before.q_rad[i], abs=1e-12)
            assert after.qd_rad_s[i * 10] == pytest.approx([v * 0.1 for v in before.qd_rad_s[i]], abs=1e-12)


# ---------------------------------------------------------------------------
# The header's axis mask is the CELL's, not the format's default.
#
# The wire is nine columns on every codec; `axis_mask` is the plan's claim of
# which of them it commands, and the daemon refuses a plan whose mask names an
# axis the active pair has not mapped and homed. rosie_1420_v1 maps 0x3f, and
# the params default of 0x1ff had a six-axis plan refused with "all plan axes
# must be mapped and homed" while J7..J9 sat at zero in every sample.


def _header(blob: bytes) -> dict:
    (header_len,) = struct.unpack(">I", blob[:4])
    return json.loads(blob[4 : 4 + header_len].decode("utf-8"))


def test_the_cell_axis_mask_is_driven_plus_held():
    from weld_motion_planner.io.dense_output import cell_axis_mask

    # rosie_1400_v3: J1-J7 driven, J8/J9 held -> every column, the old default.
    assert cell_axis_mask(["J1", "J2", "J3", "J4", "J5", "J6", "J7"],
                          {"J8": 0.0, "J9": 0.0}) == 0x1ff
    # rosie_1420_v1: six driven, nothing held -> the low six bits.
    assert cell_axis_mask(["J1", "J2", "J3", "J4", "J5", "J6"], {}) == 0x3f
    # A held axis is a setpoint too, so it is in the mask even when nothing
    # drives it.
    assert cell_axis_mask(["J1"], {"J9": 1.0}) == (1 << 0) | (1 << 8)
    with pytest.raises(ValueError):
        cell_axis_mask(["J10"], {})
    with pytest.raises(ValueError):
        cell_axis_mask(["positioner"], {})
    with pytest.raises(ValueError):
        cell_axis_mask([], {})


def test_plan_from_result_writes_the_given_axis_mask():
    default = dtj.encode(dtj.plan_from_result(_native_result(), _result_identity()))
    six = dtj.encode(dtj.plan_from_result(_native_result(), _result_identity(),
                                          axis_mask=0x3f))
    assert _header(default)["axes"]["axis_mask"] == 0x1ff
    assert _header(six)["axes"]["axis_mask"] == 0x3f
    # The mask is part of the trajectory digest, so a six-axis plan is a
    # different trajectory from the same samples claimed for nine.
    assert _header(six)["trajectory_digest"] != _header(default)["trajectory_digest"]
    # Nine columns either way: the mask narrows the claim, not the record.
    assert _header(six)["axes"]["axis_count"] == dtj.AXIS_COUNT
    assert _header(six)["sample_encoding"]["record_bytes"] == dtj.RECORD_BYTES


@pytest.mark.parametrize("mask", [0, 1 << dtj.AXIS_COUNT, -1])
def test_plan_from_result_refuses_a_mask_that_names_no_wire_axis(mask):
    with pytest.raises(dtj.DenseJointTrajectoryError) as caught:
        dtj.plan_from_result(_native_result(), _result_identity(), axis_mask=mask)
    assert caught.value.reason == "axis_mask_invalid"


def test_taught_program_keeps_node_order_and_dwell_without_process_outputs():
    result = _native_result()
    first, second = result["connecting_trajectories"]
    first["node_id"], second["node_id"] = "p1", "p2"
    second.update(_smooth_wire(first["knots_deg"][-1], first["knots_deg"][0], 4., 21))
    result["seams"] = []
    result["program_order"] = [{"id": "p1", "op": "move"}, {"id": "pause", "op": "dwell", "duration_s": .25}, {"id": "p2", "op": "move"}]
    plan = dtj.plan_from_result(result, _result_identity())
    assert [s.source_id for s in plan.segments] == ["p1", "pause", "p2"]
    assert [s.kind for s in plan.segments] == ["freespace", "dwell", "freespace"]
    dtj.encode(plan)
    second["node_id"] = "p1"
    with pytest.raises(dtj.DenseJointTrajectoryError, match="identities"):
        dtj.plan_from_result(result, _result_identity())


def test_output_rows_become_header_markers_bound_by_their_own_digest():
    from weldplan import io_schedule
    outputs = {"schema": io_schedule.SCHEMA, "torch_qualified": False,
               "outputs": [{"name": "gas", "class": "process", "expiry_ns": 200_000_000}]}
    result = _native_result()
    first, second = result["connecting_trajectories"]
    first["node_id"], second["node_id"] = "p1", "p2"
    second.update(_smooth_wire(first["knots_deg"][-1], first["knots_deg"][0], 4., 21))
    result["seams"] = []
    result["program_order"] = [{"id": "on", "op": "io", "channel": "gas", "state": True}, {"id": "p1", "op": "move"},
                               {"id": "off", "op": "io", "channel": "gas", "state": False}, {"id": "p2", "op": "move"}]
    plan = dtj.plan_from_result(result, _result_identity(), io_outputs=outputs)
    assert [(s.kind, s.source_id) for s in plan.segments] == [("dwell", "on"), ("freespace", "p1"), ("dwell", "off"), ("freespace", "p2")]
    # The row's dwell is IO_EVENT_S long and holds the pose it starts at.
    assert plan.segments[0].t_s[-1] == pytest.approx(io_schedule.IO_EVENT_S)
    assert plan.segments[0].q_rad[0] == plan.segments[1].q_rad[0]
    header = _header(dtj.encode(plan))
    markers = header["process_markers"]
    assert markers[0] == {"segment": 0, "sample": 0, "action": "gas", "value": True}
    assert {"segment": 2, "sample": 0, "action": "gas", "value": False} in markers
    assert all(m["value"] for m in markers[:-1]) and markers[-1]["value"] is False
    assert header["process_markers_sha256"] == io_schedule.markers_digest(markers)
    # The samples' digest does not move with the schedule (rt-core recomputes it).
    plain = dict(result, program_order=[n if n["op"] != "io" else {"id": n["id"], "op": "dwell", "duration_s": io_schedule.IO_EVENT_S}
                                        for n in result["program_order"]])
    assert _header(dtj.encode(dtj.plan_from_result(plain, _result_identity())))["trajectory_digest"] == header["trajectory_digest"]
    # A dry run switches nothing; with no cell outputs the rows are refused.
    assert "process_markers" not in _header(dtj.encode(dtj.plan_from_result(result, _result_identity(), io_outputs=outputs, execution_mode="dry_run")))
    with pytest.raises(dtj.DenseJointTrajectoryError, match="io_outputs_missing"):
        dtj.plan_from_result(result, _result_identity())


def test_taught_and_weld_segments_keep_authored_order_and_torch_scope():
    import copy
    result = _native_result()
    approach, retract = result["connecting_trajectories"]
    approach["node_id"], retract["node_id"] = "approach", "retract"
    move = copy.deepcopy(retract)
    move["node_id"] = "p1"
    move.update(_smooth_wire(retract["knots_deg"][-1], approach["knots_deg"][-1], 4., 21))
    result["connecting_trajectories"].append(move)
    result["program_order"] = [{"id":"approach", "op":"approach"}, {"id":"w1", "op":"weld", "seam_id":"seam-1"}, {"id":"retract", "op":"retract"}, {"id":"p1", "op":"move"}]
    plan = dtj.plan_from_result(result, _result_identity())
    assert [segment.source_id for segment in plan.segments if segment.kind != "dwell"] == ["approach", "seam-1", "retract", "p1"]
    dtj.encode(plan)


def _taught_pair():
    result = _native_result()
    first, second = result['connecting_trajectories']
    first['node_id'], second['node_id'] = 'p1', 'p2'
    second.update(_smooth_wire(first['knots_deg'][-1], first['knots_deg'][0], 4., 21))
    result['seams'] = []
    return result


def test_pulse_and_wait_keep_authored_order_and_dry_run_strips_both():
    from weldplan import io_schedule
    result = _taught_pair()
    result['program_order'] = [
        {'id':'pulse','op':'io','action':'pulse','channel':'O14','state':True,'duration_s':.2},
        {'id':'p1','op':'move'},
        {'id':'sensor','op':'io','action':'wait','channel':'I14','state':True,'timeout_s':2,'stable_s':.05},
        {'id':'p2','op':'move'}]
    outputs = {'schema':io_schedule.SCHEMA,'torch_qualified':False,
               'outputs':[{'name':'O14','class':'process','expiry_ns':200000000}], 'inputs':[{'name':'I14'}]}
    plan = dtj.plan_from_result(result, _result_identity(), io_outputs=outputs)
    assert [s.source_id for s in plan.segments] == ['pulse','p1','sensor','p2']
    header = _header(dtj.encode(plan))
    markers = header['process_markers']
    pulse = [m for m in markers if m['action']=='O14']
    assert pulse[0]['sample']==0 and pulse[0]['value'] is True
    assert pulse[-1]['sample']==20 and pulse[-1]['value'] is False
    assert all(m['segment']==0 for m in pulse)
    assert {'segment':2,'sample':0,'action':'I14','value':True,'timeout_ms':2000,'stable_ms':50} in markers
    dry = _header(dtj.encode(dtj.plan_from_result(result,_result_identity(),execution_mode='dry_run')))
    assert 'process_markers' not in dry
    assert dry['trajectory_digest']==header['trajectory_digest']
    outputs['inputs']=[]
    with pytest.raises(dtj.DenseJointTrajectoryError,match='wait input is not configured'):
        dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)


def test_weld_arrival_markers_require_qualification_and_leave_motion_unchanged():
    from weldplan import io_schedule
    result = _taught_pair()
    result['program_order'] = [
        {'id':'p1','op':'move','motion':'joint','weld_transition':'start'},
        {'id':'p2','op':'move','motion':'linear','speed_mm_s':8,'constant_tcp_speed':True,'weld_transition':'stop'}]
    outputs = {'schema':io_schedule.SCHEMA,'torch_qualified':False,
               'outputs':[{'name':'welder','class':'torch','expiry_ns':200000000}]}
    with pytest.raises(dtj.DenseJointTrajectoryError,match='weld_output_unassigned'):
        dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)
    # Synthetic qualification for schedule tests only; this is not a machine configuration.
    outputs['torch_qualified']=True
    plan=dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)
    header=_header(dtj.encode(plan)); markers=header['process_markers']
    assert markers[0]=={'segment':0,'sample':len(plan.segments[0].t_s)-1,'action':'welder','value':True}
    assert markers[-1]=={'segment':1,'sample':len(plan.segments[1].t_s)-1,'action':'welder','value':False}
    dry=_header(dtj.encode(dtj.plan_from_result(result,_result_identity(),execution_mode='dry_run')))
    assert 'process_markers' not in dry and dry['trajectory_digest']==header['trajectory_digest']
    result['program_order'][1]['motion']='joint'
    with pytest.raises(dtj.DenseJointTrajectoryError,match='only MoveL/MoveC'):
        dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)


def test_taught_weld_groups_multiple_cartesian_destinations():
    from weldplan.taught_weld import problems
    nodes = [
        {"id": "start", "op": "move", "motion": "joint", "weld_transition": "start"},
        {"id": "middle", "op": "move", "motion": "linear", "speed_mm_s": 8},
        {"id": "end", "op": "move", "motion": "linear", "speed_mm_s": 8, "weld_transition": "stop"},
    ]
    from weldplan.taught_weld import spans
    assert problems(nodes) == []
    assert spans(nodes) == {1: nodes[1:]}
    assert problems([nodes[0], nodes[2]]) == []


def test_scoped_input_monitor_survives_export_and_is_stripped_for_trace():
    from weldplan import io_schedule
    result = _taught_pair()
    result["program_order"] = [
        {"id":"begin","op":"io","action":"monitor","channel":"I14","state":True},
        {"id":"p1","op":"move"},
        {"id":"end","op":"io","action":"end_monitor","channel":"I14","state":True},
        {"id":"p2","op":"move"}]
    doc={"schema":io_schedule.SCHEMA,"torch_qualified":False,"outputs":[],"inputs":[{"name":"I14"}]}
    plan=dtj.plan_from_result(result,_result_identity(),io_outputs=doc)
    header=_header(dtj.encode(plan))
    assert header["process_markers"] == [
        {"segment":0,"sample":0,"action":"I14","value":True,"monitor":"start"},
        {"segment":2,"sample":0,"action":"I14","value":True,"monitor":"end"}]
    dry=_header(dtj.encode(dtj.plan_from_result(result,_result_identity(),execution_mode="dry_run")))
    assert "process_markers" not in dry
    assert dry["trajectory_digest"] == header["trajectory_digest"]


def test_weld_delays_hold_pose_with_output_on_and_dry_run_retains_timing():
    from weldplan import io_schedule
    result=_taught_pair()
    result['program_order']=[
        {'id':'p1','op':'move','motion':'joint','weld_transition':'start','weld_start_delay_s':.3},
        {'id':'p2','op':'move','motion':'linear','speed_mm_s':8,'constant_tcp_speed':True,'weld_transition':'stop','weld_stop_delay_s':.4}]
    outputs={'schema':io_schedule.SCHEMA,'torch_qualified':True,'outputs':[{'name':'welder','class':'torch','expiry_ns':200000000}]}
    plan=dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)
    assert len(plan.segments)==4
    for i,duration in [(1,.3),(3,.4)]:
        hold=plan.segments[i]
        assert hold.t_s[-1]==pytest.approx(duration)
        assert all(q==plan.segments[i-1].q_rad[-1] for q in hold.q_rad)
        assert all(all(v==0 for v in qd) for qd in hold.qd_rad_s)
    markers=_header(dtj.encode(plan))['process_markers']
    assert markers[0]['segment']==0 and markers[0]['value'] is True
    assert markers[-1]['segment']==3 and markers[-1]['sample']==len(plan.segments[3].t_s)-1 and markers[-1]['value'] is False
    assert all(m['value'] for m in markers[:-1])
    dry=dtj.plan_from_result(result,_result_identity(),execution_mode='dry_run')
    assert _header(dtj.encode(dry))['trajectory_digest']==_header(dtj.encode(plan))['trajectory_digest']
    assert 'process_markers' not in _header(dtj.encode(dry))
    result['program_order'][0]['weld_start_delay_s']=-1
    with pytest.raises(dtj.DenseJointTrajectoryError,match='delay'):
        dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)


def test_selected_process_weld_output_preserves_path_and_never_qualifies_torch():
    from weldplan import io_schedule
    result = _taught_pair()
    result['program_order'] = [
        {'id':'p1','op':'move','motion':'joint','weld_transition':'start','weld_start_delay_s':.3},
        {'id':'p2','op':'move','motion':'linear','speed_mm_s':8,'constant_tcp_speed':True,'weld_transition':'stop','weld_stop_delay_s':.4}]
    outputs = {'schema':io_schedule.SCHEMA,'torch_qualified':False,'weld_output':'O1',
               'outputs':[{'name':'weld','class':'torch','expiry_ns':200000000},
                          {'name':'O1','class':'process','expiry_ns':200000000}]}
    plan = dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)
    markers = _header(dtj.encode(plan))['process_markers']
    assert {m['action'] for m in markers} == {'O1'}
    assert markers[0]['value'] is True and markers[-1]['value'] is False
    assert markers[-1]['segment']==3
    dry = dtj.plan_from_result(result,_result_identity(),io_outputs=outputs,execution_mode='dry_run')
    assert 'process_markers' not in _header(dtj.encode(dry))
    assert _header(dtj.encode(dry))['trajectory_digest']==_header(dtj.encode(plan))['trajectory_digest']
    for selected in ('', 'weld', 'O6', 'I1'):
        outputs['weld_output']=selected
        with pytest.raises(dtj.DenseJointTrajectoryError,match='weld_output_unassigned'):
            dtj.plan_from_result(result,_result_identity(),io_outputs=outputs)
        # Trace remains available even without a qualified/assigned trigger.
        assert 'process_markers' not in _header(dtj.encode(dtj.plan_from_result(
            result,_result_identity(),io_outputs=outputs,execution_mode='dry_run')))
    outputs['weld_output']='O1'
    assert io_schedule.io_problems([{'op':'io','channel':'O1','state':True}],outputs)
    assert outputs['torch_qualified'] is False and outputs['outputs'][0]['class']=='torch'
