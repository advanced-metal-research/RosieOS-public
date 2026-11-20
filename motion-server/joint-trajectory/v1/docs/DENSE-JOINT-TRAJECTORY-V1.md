# `robot.v4.dense-joint-trajectory.v1`

The input contract of the dense trajectory daemon: a dense 100 Hz joint
`(q, qd)` program with per-sample torch state, carried as one binary blob
over TCP. This is the program-by-reference path `WIRE_DT_S` (weld planner,
`engine/control_solver.py`) is waiting on: with it, certificate-density
sampling (0.01 s) no longer has to fit through a 1 MiB NATS payload.

Three implementations must stay byte-for-byte agreed:

| language | role | where |
|---|---|---|
| Python | encoder (producer) | `weld_planner/v1/python/weldplan/dense_joint_trajectory.py` |
| Go | codec twin (OLP) | `offline-programming/v1/internal/densejointtraj/` |
| C++ | decoder (the daemon) | `motion-server/joint-trajectory/v1/src/dense_joint_trajectory_format.hpp` |

The reference fixture in `../testdata/dense_joint_trajectory_reference.rdt` is the
arbiter. All three test suites assert the same digest and the same bytes
against it; regenerate it only when the format itself changes
(`tools/make_reference_fixture.py`, deterministic), and expect all three
suites to fail together until every codec agrees again.

## Blob layout

```
[u32 BE header_len][UTF-8 JSON header][sample block 0]..[sample block S-1]
```

One binary sample block **per motion segment**, contiguous, in segment-list
order. The weld planner emits one trajectory per segment ("each its own
clock"); this layout preserves that instead of concatenating onto an
artificial global timeline.

## Sample block

Array of fixed 153-byte records, all fields big-endian:

| offset | field | bytes |
|---|---|---|
| 0 | `t_s` f64 — segment-local, starts at 0.0, monotone | 8 |
| 8 | `q_rad[9]` f64 | 72 |
| 80 | `qd_rad_s[9]` f64 — mandatory in v1 | 72 |
| 152 | `flags` u8 — bit0 = torch_on; bits 1–7 reserved, must be 0 | 1 |

Axis order is J1..J6 arm, J7/J8 positioner, J9 side selector (the motion
server's live axis order). 18,000 samples (3 min at 0.01 s) ≈ 2.75 MB;
the 250,000-sample cap ≈ 38 MB — inside the transport's 64 MiB frame limit.

## JSON header

```jsonc
{
  "schema": "robot.v4.dense-joint-trajectory.v1",
  // Identity, matched exactly at play; not part of the content digest.
  "plan_id": "...", "program_id": "...", "program_digest": "sha256:...",
  "trajectory_digest": "sha256:...",   // self content digest, verified on ingest
  "manifest_revision": 7, "plan_revision": 3, "created_at": "RFC3339",
  "axes": { "axis_count": 9, "axis_mask": 511,
            "position_unit": "rad", "velocity_unit": "rad_s" },
  "sampling": { "dt_s": 0.01, "total_sample_count": 11, "total_duration_s": 0.09 },
  "limits": { "max_abs_qd_rad_s": [3.0, ...] },   // executor clamp ceiling per axis
  "segments": [
    { "index": 0,                       // == position in list == block order
      "kind": "freespace",              // freespace | weld | dwell
      "source_id": "move-1",            // the planner object, for tracing
      "sample_count": 5, "duration_s": 0.04,
      "block": { "byte_offset": 0,      // from end of JSON header
                 "byte_length": 765,    // sample_count * 153
                 "sha256": "sha256:..." } }  // over this block's bytes verbatim
  ],
  "sample_encoding": { "encoding": "f64-be-aos.v1", "record_bytes": 153 }
}
```

Unknown keys are ignored by decoders — that is the forward-compatibility
contract. There is deliberately **no weld-process (WPS) table in v1**; a
future `weld_spans` key per weld segment has a reserved digest section (tag
0x01, below) so adding it will not move existing digests.

Inter-segment holds are explicit `dwell` segments, never implied gaps. The
executor runs a segment to completion, holds the last q with qd = 0, then
starts the next; it never interpolates across segments.

## Validation (shared rejection vocabulary)

Checked in this order; every rejection carries `reason` and, where one
segment is at fault, its index. Block hashes come first, before any sample
parsing, so transport/storage corruption is reported per segment and
distinctly from a logical mismatch.

| reason | rule |
|---|---|
| `header_truncated` / `header_json_invalid` / `schema_mismatch` | framing/schema |
| `axis_count_mismatch` | `axes.axis_count == 9` |
| `sample_encoding_mismatch` | encoding `f64-be-aos.v1`, record_bytes 153 |
| `limits_invalid` | 9 finite positive limits |
| `segments_empty` / `segment_index_out_of_order` / `kind_invalid` | segment list well-formed |
| `segment_too_short` | ≥ 2 samples per segment |
| `block_layout_invalid` | offsets contiguous in order; length = count·153 |
| `blob_length_mismatch` | blocks exactly cover the body |
| `block_sha256_mismatch` | per-block hash, checked before parsing |
| `reserved_flags_set` | flag bits 1–7 zero |
| `total_sample_count_mismatch` / `sample_count_overflow` | header total = Σ; ≤ 250,000 |
| `nonfinite_sample` | no NaN/Inf in t/q/qd |
| `qd_limit_exceeded` | per-sample `\|qd\| ≤ max_abs_qd_rad_s[axis]` |
| `torch_outside_weld` | torch bit only in `weld` segments |
| `time_grid_invalid` | `t[0] == 0`; `\|Δt − dt_s\| ≤ 1e-9 s` |
| `q_step_exceeded` | `\|Δq\| ≤ 1.25 · limit · dt` per axis per step |
| `boundary_qd_nonzero` | `\|qd\| ≤ 1e-6 rad/s` at each block's first/last sample |
| `boundary_q_discontinuity` | handover gap ≤ 1e-3 rad between consecutive segments |
| `duration_mismatch` | `duration_s == t_last` (±1e-9 s) |
| `trajectory_digest_mismatch` | recomputed content digest == header value |

Format validity is necessary but not runtime admission. At `play`, the daemon
also requires a fresh leader lease, fresh matching operation-enabled feedback,
mapped and homed plan axes, and an exact effective per-axis speed ceiling from
RTCore. Sample `qd` and actual finite-difference position-step rates (without
the ingest validator's 1.25 tolerance) above that ceiling are refused as
`downstream_rate_exceeded`. During playback, authority or limit drift fails
closed; feedback error above the configured bound aborts before the plan clock
or torch state advances.

The boundary rules exist because the executor holds with qd = 0 between
segments: a nonzero boundary qd steps the commanded velocity at the splice,
and a position gap would be slewed by the follower at the qd clamp —
uncommanded motion at up to 3 rad/s — so both are refused, never smoothed.

## Content digest

SHA-256 over a versioned big-endian layout (`u64` BE; `f64` as the u64 BE of
its IEEE-754 bits; strings length-delimited `u64 len + UTF-8`), rendered
`"sha256:" + hex`:

```
"robot.v4.dense-joint-trajectory.v1\x00"
u64 axis_count; u64 axis_mask; f64 dt_s; u64 segment_count
per segment, in order:
  str kind; str source_id; u64 sample_count
  <that segment's block bytes verbatim>
  <optional tagged sections: u8 tag, u64 length, content — hashed only when
   present; v1 defines none; a future weld-process table takes tag 0x01>
u64 9; f64 max_abs_qd_rad_s[0..8]
```

The packed block bytes double as the digest input — one canonical layout per
language instead of two, so cross-language digest equality falls out of the
round-trip tests. Identity strings are excluded: this is a content digest;
identity is matched separately at play, same split as the accepted-plan v1
digest. A breaking change is a new schema string and digest domain
(`.v2`); additive metadata rides the optional-section mechanism.

## TCP ingest (phase 2, for orientation)

Framing `[u64 BE size][payload]`, 64 MiB cap (the collision-server pattern);
payload `[u32 BE json_len][JSON][optional binary body]`; one request, one
response, close. Kinds: `upload` (inert — never causes motion), `validate`
(dry-run), `preload` (by reference: digest + identity), `status`. Play/stop
authority stays on NATS with the leader-lease fencing; TCP only solves the
payload-size problem. A `preload` while a plan is playing or paused is
refused with `playback_in_progress` and stages nothing: the executor's tick
indexes the staged plan by the in-flight cursor, so a swap underneath it
would read past the end of a shorter plan or drive a trajectory that never
passed the play identity gate. Stop first.

The `status` reply's `executor` object carries, beyond the cursor
(`state`, `segment_index`, `t_s`, `sample_index`, `torch_on`,
`feedback_age_ms`, `authority_fresh` -- shape frozen, the Go decoder reads
them), the arm and servo view: `armed` (feedback valid and every mapped
axis operation-enabled), `authority` (`valid`, `fresh`, `runtime_mode`
name, `runtime_mode_epoch`, `mapped_axis_mask`, `jetson_id`,
`robot_model_id`) and `feedback` (`valid`, `age_ms`, the ServoFeedbackV4
`armed_mask`/`fault_mask`/`error_mask`/`home_valid_mask`, and nine `axes`
entries of `q_rad`, `q_cmd_rad`, `pos_counts`, `home_counts`,
`statusword`, `mode_display`, `operational`, `fault`, `error`). `q_rad`
and `q_cmd_rad` are `null` when they cannot be known (no feedback or no
authority gear; nothing commanded yet). The field-by-field contract and a
rendered example are in the README's "TCP `status` reply" section.
