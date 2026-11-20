# Implemented application API

2026-10-02 drive-state supervisor (offline patch on `c90b1fe4`; not deployed,
not physically qualified): `axes[i].drive_supervisor` (`severity` arm_ended or
warning, `event_sequence`, `kind` enable_timeout/release_timeout/
left_operation_enabled/arm_cancelled/disarm_motion, `cause`, `message`,
`statusword`, `arm_ended`, `current`, elapsed/bound time, displacement/bound
counts) and one `drive_supervisor` event. An Arm-ending event clears `armed`
only; the grant, generation and enable selection survive, nothing latches, and
Arm is retried. `current` is true until the next accepted Arm request. Example
messages: "J4 did not reach Operation Enabled within 500 ms (statusword
0x0233). Arm ended; retry Arm.", "J5 left Operation Enabled while armed
(statusword 0x0250). Arm ended; retry Arm." `brake_hold_diagnostic` no longer
reports opening_timeout, frame_changed, stale_feedback, observation_time,
drive_state, opening_proof or invalid_configuration. The schema digest changed,
so the OLP UI and Steam Deck client must be rebuilt.

2026-10-01 brake-hold warnings (offline source on `brake-warn-not-fault`;
not deployed, not physically qualified): brake-hold detections no longer fault.
Each is a per-axis warning in `axes[i].brake_hold_diagnostic` (`severity`,
`warning_sequence`, `transition` torque_on/torque_off, `message`, trip sample,
peak displacement/velocity and settled re-anchor) and one `brake_warning` event.
The message names the drive parameter: torque_on "Brake appears to release after
torque on - adjust drive brake release delay (C05.13).", torque_off "Brake appears
to latch after torque off - adjust drive brake hold delay (C05.10)." Bit 10 is
never raised; Stop, drive-managed hold windows, release-delay motion gating and
every other fault are unchanged. The schema digest changed, so the OLP UI must be
rebuilt against the regenerated TypeScript client before it accepts this Describe.

2026-10-01 follow-up (offline working-tree change; not committed, deployed or
physically qualified): movement while the drive still holds torque after the
inhibit (until `hold_delay_ns` elapses) is recorded in
`energized_peak_displacement_counts`/`energized_peak_velocity_counts_per_s` and
never warned; closing is judged from `torque_off_anchor_counts` at
`torque_off_ns`, so a torque_off warning always means take-up after torque off
(C05.10). Three consecutive same-direction warnings within one braked period
(torque off until enable/Arm, Home or frame change; 3 is an unverified initial
choice) make `kind` slip with `same_direction_count`,
`same_direction_displacement_counts` and the message "Brake may be slipping:
J<n> moved <N> times in the same direction (total <counts> counts, <deg> deg)
while the brake was applied - inspect the brake." A reversal restarts the count;
separate stops never accumulate. Slip uses the same `brake_warning` event and is
never a fault. The diagnostic is also published when an energized window closes
(`severity` none until a warning). Digest `9d367b32â€¦` requires rebuilt UI and
Steam Deck clients.

2026-10-02 current-pose reference removal: the software current-pose zero
introduced on this branch (`capture_current_reference`, native command 266,
the `reference_capture` axis policy, `rosie-anchor-v5` receipts and their
compiler, SDK, OLP and test support) is deleted, not stubbed. Home is
established only by drive homing (HM35), which writes 607C; rt-core records the
HM35 position as a v3 anchor for check and backup. That anchor is restored
automatically after a power cycle by direct drive checks, by `RestoreAnchor`,
or by verified-delta restore (command 267) for v3 only. A remaining
`rosie-anchor-v5` file is refused, untouched, as `anchor_source_invalid`:
"current-pose reference anchors are not accepted; establish Home with drive
homing (HM35)". A configuration declaring `reference_capture` no longer
compiles or loads, so trial configurations that opted J6 in cannot be
reproduced byte-for-byte; their entries below remain historical evidence only.
Software never routes around a drive or hardware failure (for example a drive
that fails HM35): the failure is surfaced and fixed at the drive.

The C++ client replenishes consumed streaming reservations through its keeper,
without cold connection setup in BeginJog. An absent optional status `grant`
is not a revocation receipt; clients must still enforce their lease and explicit
authority-loss results. No motion deadlines or admission rules change.

Per-axis calibration applicability, position state/reason and stationary audit
status/reason/time are optional for legacy readers. Missing fields mean unknown,
not successful recovery. Native `home_valid` remains the conservative projection
of applicable calibration and trustworthy position. Position recovery grants no
Reset, Arm or execution authority. The changed capture layout requires matching
readers; older captures fail the existing size/digest check. Hardware release of
the independent multi-turn monitoring change remains blocked on reset/turn-loss
qualification.

OLP's existing joint-move HTTP API accepts `axes[].position_rad` as an absolute
endpoint, exclusive with nonzero `degrees`. Go to plan start uses this before
Load/Play. This changes neither the native protocol nor Home or motion guards.

SDK Reset bookkeeping now retires the matching local fence when a matching
`reset_fault` response has a positive submission sequence, including a native
refusal. Unsubmitted/uncertain responses retain recovery authority, and delayed
old replies cannot clear a newer grant. Subsequent Acquire/Renew uses the fresh
session. This changes no wire schema or native fault-clear requirements.

The Go SDK's opt-in `StatusObservation(ctx)` requests
`GET /v1/status?view=control`. This optional projection returns the existing
`ProcessStatus` payload with `execution.handles: []`; all current native,
logical-axis, authority, clock, execution, fault, I/O and recovery fields retain
their ordinary meaning. The adapter observes the same lifecycle transitions but
skips copying, sorting and serializing its historical handle list. Ordinary
`GET /v1/status` and `Status(ctx)` retain the complete history unchanged.

That history is bounded (2026-11-13). The adapter keeps every prepared and
started handle and the newest 1,024 terminal ones (consumed, discarded,
superseded, retired), counted at each preparation
(`retainedTerminalHandles`, adapters/rosie/control/handles.go). A full read
with that history is about 296 KB, well inside the SDK's 8 MiB response cap;
unbounded, a long loop on a cell grew it past 1.1 MB. Start or
Discard of a forgotten handle answers `unknown_handle` instead of its terminal
reason; both mean upload anew. The schema and capabilities digest are unchanged.

Older adapters ignore the optional query and return full status. The SDK's
existing one-pass projection still discards that history, without issuing a
second request. The operation, required array field, schema, capability digest
and native IPC are unchanged. This is an optional response projection, not a
new motion capability or a change to freshness/recovery policy.

OLP uses current-only observations for Reset/jog startup, ordinary background
observation and the pendant's control-status route before upstream I/O. Joint
moves retain full correlated handle receipts; late compact responses cannot
replace those receipts or refresh their freshness clock. Full diagnostics
remain available.

Go SDK telemetry polling/streaming uses a separate bounded connection pool from
control acquisition, with caller cancellation and idle read deadlines preserved.
This changes no wire API or authority rule; capturing telemetry grants no control.

Retained drive Home can now restore `home_valid_mask` during inhibited startup
after saved-offset, software-anchor, identity and encoder/PDO qualification.
`restored_anchor_home_epoch` identifies the reused record. This grants no control
lease, enable or arm permission; the original explicit Home operation is unchanged.

2026-09-23 OLP telemetry extension: ring windows support up to 100 seconds and
include daemon incarnation/configuration SHA, record configuration/Home epochs,
trajectory ID/submission mask, and per-axis continuous coordinate validity/counts
and Home offset. OLP uses these additive fields to reject mixed identity traces
and compare feedback with the preceding submitted command. Native wire layouts
and motion thresholds are unchanged.

Configuration files are organized by layer under [config/](../config/README.md): drive, robot, machine and cell configs.

Drift note: this document records `protocol/application-v1.schema.json` SHA-256
`0948a57f2ce6c5928e9ecb94cb8725046ec03f29684deb96d3e76a4bd8240662` (exact UTF-8 bytes including final LF).
It adds the optional fault-record `reset_preserves_home` advisory (previous digest `c607bcc6…`).
It removes the retired wrapped-coordinate fields `feedback_wrap`, `command_wrap`
and `anchor_period_counts` (previous digests `f02e9f05…` on the linear-mode branch and `9812b73d…` on main)
and includes per-axis drive health (`AxisLoad`, `AxisMonitor`; docs/decisions/0001; previous digest `2a3b54da…`), the read-only `GET /v1/maintenance` route (`MaintenanceObservation`, `MaintenanceHolder`; previous digest
`b1e28f06…`), the protocol 1.14 path jog lane (Describe `path_jog`, `begin_path_jog`, `GET /v1/jog/path`
`PathJogStatus`, `update_path_jog` chunks and their path reasons), the uncapped HM35 (v3) verified-delta restore policy (no `maximum_delta_counts`; issue #158),
per-cell control timing and lease negotiation, the published control idle timeout, the per-axis bench brake override,
P4-17 collision watchdog state/reason, the per-axis brake-hold warning with its slip kind and energized-window record, the per-axis drive-state supervisor event, the drive alarm auxiliary word,
the P6-05 cell I/O status, operations, reasons and marker contract,
and the P8-34 telemetry V2 payload and retention limits.
Compare this value with Describe's `capabilities_digest` after every schema change.
Source citations below use line numbers at commit `403063dc`.
The capability, field, reason and rule tables below are mechanically rendered from
that schema. Runtime exceptions identified from the HTTP handler are explicit below;
they do not silently change the schema. [Architecture](ARCHITECTURE.md) describes
ownership; [the plan](IMPLEMENTATION_PLAN.md) and tracker own acceptance.

Describe publishes the optional configured `io` block: terminal identity, named
input classes and polarities, output classes, OFF safe states, expiry budgets and
readback wiring. `bus.held_slaves` also includes the declared terminal with
`disposition: "io_terminal"`; it is required under both unknown-slave policies.

Cell I/O (P6-05): `status.core.io` publishes cyclic named inputs (`value`,
`valid`, `observed_ns`, `bit`, `fast`) and outputs (`intent`, `commanded`,
`readback`, `valid`, absolute monotonic `expiry_ns`, `observed_ns`, `changed_ns`,
`marker_sample_ns`, `marker_cycle`, `bit`, `torch`). Its `time_ns` and `cycle`
identify the native observation. Intent and commanded values never prove physical
output; only valid independent input feedback establishes readback. A false input
validity means unavailable; fast inputs also publish not-satisfied on loss.

`io_arm` and `io_disarm` use the ordinary fenced `/v1/control` request and return
a native sequence receipt. I/O permission is separate from motion Arm. Stop writes
process/torch outputs OFF in its fence cycle; their ON requires a fresh grant and explicit
`io_arm`, with current satisfied fast inputs and valid OFF readback observed after
the fence cycle. Disarm clears pending intents. Independent expiry is never
extended by grant renewal. Terminal
failure may prevent physical delivery; the real watchdog/OFF chain still requires
hardware qualification. See [CELL_IO.md](CELL_IO.md).

The optional dense header `process_markers` array contains `{segment, sample,
action, value}` entries. `action` names a configured output and `segment`/`sample`
identify an exact source sample, including dwell and shared endpoints. Duplicate
output markers at the same normalised sample are refused. The full source envelope
and normalised marker list participate in prepared identity. Native raw trajectory
points also accept sixteen-bit `io_mask`/`io_values` in configured output order.
Delivery is bounded to one cycle (`io_marker_delivery_cycles_unverified`); a missed
bound faults instead of replaying a late ON. Two markers for the same output in one
exchange also fault instead of collapsing a physical transition. Each ON gets its
configured independent expiry, at most 1,000,000,000 ns. Readback disagreement after three exchanges latches
execution fault bit 13 (`cell_io`), requiring investigation and restart. These are
simulation-tested bounds, unverified on hardware.

Typed operation reasons include `io_not_configured`, `io_not_armed`,
`io_fast_input_unsatisfied`, `io_readback_disagreement`, `io_torch_unqualified`,
`io_marker_late`, `io_exchange_lost`, and `io_marker_invalid`. The native `io.reason`
codes are respectively 12, 13, 14, 15, 16, 17, 18 (marker-invalid is an adapter
admission reason). Torch-bearing dense programs are refused during Prepare with
`process_io_executor_not_qualified`; torch-class native markers are refused even
when their value is OFF. `torch_qualified: false` is mandatory in configuration.
The generated Go, C++ and TypeScript contracts include this additive surface.

OLP's Run selector chooses process-output suppression separately from planning.
`POST /dense-execution/load` accepts the original plan identity plus optional
`dry_run: true`. After validating that source, Load derives a private execution
copy with torch flags off and explicit process markers removed. Motion bytes,
timing, robot pins and the stored source remain unchanged; no solver is invoked.
The adapter prepares and Start uses the derived execution identity. The session
retains its original `trajectory_digest`, reports `dry_run`, and separately
reports `execution_trajectory_digest` and `execution_normalised_digest`.
Removing torch bits changes the trajectory digest; removing only header markers
can retain it because v1 excludes those markers from that digest, while native
normalisation still identifies the actual prepared outputs.

Changing Run mode requires Load again before Play, but no replan. The alternative
is **Program outputs Â· as planned**: it uses the source unchanged and cannot
restore outputs removed by an older authored `dry_run` export. Process-output
qualification and all normal motion limits, Home, authority, Arm and Play checks
remain in force.

OLP's Planning Cancel aborts the weld-plan HTTP request and its isolated planner
process, discarding unpublished output and preserving the previous saved plan.
It does not acquire robot authority or issue a motion command. Cancellation is
available until the completed result begins committing.

## Transport and response semantics

Local applications connect to HTTP over `/run/rosie-rt-core/control.sock`.
The installed socket resolves to the adapter's `public/control.sock`; its private
native peer is `native/ipc.sock`. Local jog updates use Unix datagrams on `jog.sock`.
[host/rosie-rt-control.service:20](../host/rosie-rt-control.service#L20),
[adapters/rosie/control/jog_linux.go:242](../adapters/rosie/control/jog_linux.go#L242).

Ordinary HTTP replies use `Response`, schema `rosie.rt-control.response.v1`.
`data` carries a named result type; `sequence` and `handle` are top-level fields.
Success is HTTP 200; rejection normally 409, oversized bodies 413. Unknown paths
return `unknown_endpoint`. Native rejections preserve the case-sensitive
`CommandResult` or `JogObservation` fields in `native_result`/`native_jog_result`.
The `error` string can contain an underlying diagnostic; it is not always one
catalogue label. [adapters/rosie/control/http.go:242](../adapters/rosie/control/http.go#L242).

**Documented handler behaviour at this commit:** `restore_anchor` returns a
`RecoveryStatus` read after the restore receipt, with the receipt in
`Response.sequence`; its native admission requires a valid nonzero selected axis
mask. Event cursor errors return HTTP 409 with a catalogue reason, like every other
refusal (both closed under P5-01 at 15f5b353). The schema says unknown prefix
fields are rejected before points, but `readEnvelope` retains unknown metadata until
the full strict decode rejects it. Do not rely on early rejection of every unknown
field. Binding uses `machine_sha256` when supplied, otherwise `configuration_sha256`;
the schema's general exact-Binding wording is broader than this selector. Remote
requests are additionally bound to the mutual-TLS principal and pair that acquired
the session (`session_principal_mismatch`, P6-03).
[adapters/rosie/control/http.go:208](../adapters/rosie/control/http.go#L208),
[adapters/rosie/control/controller.go:882](../adapters/rosie/control/controller.go#L882),
[adapters/rosie/control/events.go:360](../adapters/rosie/control/events.go#L360),
[adapters/rosie/control/admission.go:77](../adapters/rosie/control/admission.go#L77),
[adapters/rosie/control/controller.go:499](../adapters/rosie/control/controller.go#L499).

## Drive digital-input validity

Native axis status and cycle telemetry publish `di_bits` and `di_valid` according
to the selected fixed TxPDO preset. With fresh PDO feedback:

| A6-EC profile / preset | Digital inputs | Drive velocity and following error |
|---|---|---|
| Robot default `a6ec`, bench `a6ec_bench_1to1` / 0x1B04 | `di_bits=0`, `di_valid=false`; 0x60FD is absent from the cyclic map. | `velocity_actual_valid=true`, `following_error_valid=true`; signed 32-bit 0x606C counts/s and 0x60F4 counts in the drive position frame. |
| Explicit `a6ec_1b02` / 0x1B02 | 0x60FD supplies `di_bits` with `di_valid=true`. | Both values are zero and both validity flags are false; these objects are absent from the cyclic map. |

Missing or stale PDO feedback clears validity. A false `di_valid` is unavailable
data, never proof that inputs are inactive. On 0x1B04, mailbox drive-input reads
serve display and supervision and do not make the cyclic DI fields valid.
The hardware stop chain and cyclic cell I/O own safety reactions; see
[the preset decision](drives/a6ec/telemetry-presets-and-safety.md).

## Operations and capability states

`implemented` describes a supplied interface; it is not hardware qualification.
`interim` is the advertised reset API state. `test_only` is dispatchable for the
frozen oracle and fixtures but is not a consumer operation. `unimplemented` performs no operation
and returns `capability_unimplemented` when dispatched as a named JSON capability.
For such entries the schema transport `none` means no supported operation route.
GET capability names and `update_jog` are not POST dispatch aliases.
[adapters/rosie/control/http.go:177](../adapters/rosie/control/http.go#L177).

<!-- BEGIN SCHEMA CAPABILITIES -->
| Operation | State | Transport | Request type; semantic required fields | Schema result |
|---|---|---|---|---|
| `telemetry` | implemented | GET /v1/telemetry?after=<sequence>; GET /v1/telemetry/stream?after=<sequence> | ; none | TelemetryBatch |
| `mark_telemetry` | implemented | POST /v1/control | Request; schema, operation, session, generation, label | sequence |
| `acquire` | implemented | POST /v1/control | Request; schema, operation, controller, binding | Grant |
| `renew` | implemented | POST /v1/control | Request; schema, operation, session, generation | Grant |
| `release` | implemented | POST /v1/control | Request; schema, operation, session, generation | Grant |
| `stop` | implemented | POST /v1/control | Request; schema, operation, session, generation | Grant |
| `enable` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask | sequence |
| `arm` | implemented | POST /v1/control | Request; schema, operation, session, generation | sequence |
| `home` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask | sequence |
| `reset_fault` | interim | POST /v1/control | Request; schema, operation, session, generation | RecoveryStatus |
| `recovery_status` | implemented | POST /v1/control | Request; schema, operation, session, generation | RecoveryStatus |
| `jog` | test_only | POST /v1/control | Request; schema, operation, session, generation, axis_mask, velocity, timeout_ms | sequence |
| `begin_jog` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask, source_origin_host_ns, deadline_host_ns, clock_incarnation | handle |
| `end_jog` | implemented | POST /v1/control | Request; schema, operation, session, generation, jog_generation | handle |
| `prepare_trajectory` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask, points | handle |
| `start_trajectory` | implemented | POST /v1/control | Request; schema, operation, session, generation, handle | sequence |
| `discard_trajectory` | implemented | POST /v1/control | Request; schema, operation, session, generation, handle | sequence |
| `start_program` | implemented | POST /v1/control | Request; schema, operation, session, generation, identity | sequence |
| `describe` | implemented | GET /v1/describe | none; none | Description |
| `status` | implemented | GET /v1/status | none; none | ProcessStatus |
| `jog_clock` | implemented | GET /v1/jog/clock | none; none | LocalJogClock |
| `jog_status` | implemented | GET /v1/jog | none; none | JogObservation |
| `jog_ingress` | implemented | GET /v1/jog/ingress | none; none | JogIngressObservation |
| `maintenance` | implemented | GET /v1/maintenance | none; none | MaintenanceObservation |
| `prepare_program` | implemented | POST /v1/program | binary rdt; X-Control-Session, X-Control-Generation | Program |
| `update_jog` | implemented | jog.sock datagram or WSS /v1/jog; protocol/control.json local_jog_update | local_jog_update; none | JogObservation |
| `begin_path_jog` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask, source_origin_host_ns, deadline_host_ns, clock_incarnation | handle |
| `path_jog_status` | implemented | GET /v1/jog/path | none; none | PathJogStatus |
| `update_path_jog` | implemented | jog.sock datagram or WSS /v1/jog; protocol/control.json local_path_samples | local_path_samples; none | PathJogStatus |
| `halt` | implemented | POST /v1/control | Request; schema, operation, session, generation | Response.sequence/native_result |
| `abort` | unimplemented | none | Request; schema, operation | none |
| `subscribe_events` | implemented | GET /v1/events?after=<sequence>; GET /v1/events/stream?after=<sequence> (SSE) | none; none | EventBatch |
| `readiness` | unimplemented | none | Request; schema, operation | none |
| `restore_anchor` | implemented | POST /v1/control | Request; schema, operation, session, generation, axis_mask | RecoveryStatus |
| `io_arm` | implemented | POST /v1/control | Request; schema, operation, session, generation | sequence |
| `io_disarm` | implemented | POST /v1/control | Request; schema, operation, session, generation | sequence |
| `resource` | implemented | GET /v1/resources/<sha256> | ; none | binary |
| `restore_reference_verified_delta` | interim | POST /v1/control | Request; schema, operation, session, generation, axis_mask | VerifiedDeltaRestoreReceipt |
| `io_test` | implemented | POST /v1/control | Request; schema, operation, session, generation, label, value, source_sequence | sequence |
<!-- END SCHEMA CAPABILITIES -->

## Admission, retries and cancellation

The JSON envelope uses `schema: rosie.rt-control.request.v1`. For a large
`prepare_trajectory`, encode schema, operation, session and nonzero generation
before `points` within 16,384 bytes. The adapter admits the fence and reserves an
upload slot before reading/decoding the rest. Small unordered bodies fitting the
prefix retain the existing behavior. Prefix duplicates are rejected; full decoding
rejects unknown fields and trailing data, and rechecks the admitted envelope.
Do not use a key-sorting encoder that places a large points array before the fence.
[adapters/rosie/control/admission.go:74](../adapters/rosie/control/admission.go#L74),
[adapters/rosie/control/http.go:134](../adapters/rosie/control/http.go#L134).

One JSON and one binary `.rdt` upload may be admitted across all listeners.
Wire caps are 222,516,384 bytes for JSON and 39,298,580 bytes for `.rdt`, including
metadata and whitespace. Another same-kind upload returns `busy`. Cancellation
joins HTTP disconnect and grant revocation and cleans native staging.
`POST /v1/program` uses `X-Control-Session` and decimal `X-Control-Generation`;
it does not provide JSON `request_id` deduplication.
[adapters/rosie/control/admission.go:19](../adapters/rosie/control/admission.go#L19),
[adapters/rosie/control/admission.go:167](../adapters/rosie/control/admission.go#L167),
[adapters/rosie/control/http.go:80](../adapters/rosie/control/http.go#L80).

While the deploy tooling's maintenance notice `/etc/rosie-rt-core/maintenance.json`
exists, `acquire`, `arm`, `enable`, `home`, the anchor and reference restores,
`reset_fault`, `jog`, `begin_jog`, `begin_path_jog`, `start_trajectory`,
`start_program` and `io_arm` are refused with `busy`, wrapping the notice's own
`reason` text when it has one (`busy: rt-core full deploy on <cell>`). Stop,
Release, Halt, EndJog, Renew, IoDisarm, Discard and Prepare always pass; the
notice never stops, disarms or retires a session. A file that is not a notice
is active (fails closed); one past its ETA stays active. Each grant change is journalled as
`rt-control: grant_<event> generation=<n> controller="<name>" principal=<local|remote:<id>>`
(renewals excepted), which the deploy tool reads to name the holder.
`GET /v1/maintenance` (local socket and the mutual-TLS listener, no session or
authority, like `describe`) returns that state as `MaintenanceObservation`:
`active` and `malformed` exactly as the guard decides them, the notice's
`reason`, `phase`, `started_at`, `eta`, `set_by` and `release` (text clipped to
400 bytes; times RFC 3339 UTC, empty when absent or unparseable), and `holder`
while a grant is held: the acquire `controller` name, `remote`, the remote
certificate `identity` (empty locally), `generation` and `since` (adapter wall
clock at acquire). It never carries the session token. OLP merges it with its
own host's notice, so a Steam Deck running OLP locally blocks during a servo
host update too.
[adapters/rosie/control/maintenance.go](../adapters/rosie/control/maintenance.go).

`request_id` is optional on JSON control requests only: 1..64 printable ASCII
bytes, never null or a number. The current session caches at most 256 outcomes.
Identical decoded payload retries join an in-flight request or replay its final
wire reply; changing the payload returns `request_id_conflict`. An executing
oldest entry blocks eviction. This is bounded session-local deduplication, not a
durable guarantee after eviction or restart. Release/expiry detach the cache;
old waiters still receive their original result. After an ambiguous Start, inspect
handle state and incarnation before attempting new motion.
[adapters/rosie/control/idempotence.go:31](../adapters/rosie/control/idempotence.go#L31).

Stop urgently fences motion, retires handles and inhibits outputs without waiting
for a ramp. A valid current grant can return
with a new generation; fresh Enable/Arm are required. An authenticated expired
fence may still cancel through `authorizeStop`, but cannot regain motion permission.
Release and expiry relinquish the grant. Concurrent Stop/Release join the same
cancellation; renewal during Stop reports `stopping: true` as a local keepalive.
Neither a Stop receipt nor a Halt receipt establishes physical standstill or brake engagement.
[adapters/rosie/control/controller.go:1144](../adapters/rosie/control/controller.go#L1144),
[adapters/rosie/control/controller.go:1197](../adapters/rosie/control/controller.go#L1197),
[engine/cycle_machine.hpp:3328](../engine/cycle_machine.hpp#L3328).

Halt requests controlled deceleration to an enabled hold and does not replace Stop.
It retires the active trajectory handle and jog generation while retaining the
grant, Enable and Arm. Trajectory Halt uses the declared trajectory acceleration
when present, otherwise jog acceleration. The final hold retains the last commanded
target; a new Start must begin within one count of that target, and jogging needs
a fresh BeginJog generation. Admission uses current readiness, and every ramp
step, including the final one, checks current feedback against the target-lead
bound before dispatch. Lease expiry, authority loss, bus faults and mapping loss
keep their inhibition paths during Halt. Native Home and service refuse Halt as
`inhibited`. A Halt receipt confirms admission, not completed hold: success returns
`Response.sequence`; a native refusal also carries `native_result`.
An idle Halt changes only its receipt: no motion state, event, motion command
sequence, target or authority generation changes. Ordinary cyclic observation
timestamps continue advancing independently. The receipt sequence is separate
from the motion cursor's `active_command_seq`.
[adapters/rosie/control/halt_linux.go](../adapters/rosie/control/halt_linux.go),
[engine/cycle_machine.hpp](../engine/cycle_machine.hpp),
[tests/cycle_machine_tests.cpp](../tests/cycle_machine_tests.cpp).

## Handle lifecycle

State names are generated from the schema; transitions below follow the sole
handle writer. No terminal state regains Start permission. Definite not-ready or
mode-conflict refusal may restore a prepared plan; ambiguous admission or failed
cleanup retires it. Discard of an active handle is refused; an already discarded
handle is idempotent. [adapters/rosie/control/handles.go:5](../adapters/rosie/control/handles.go#L5),
[adapters/rosie/control/handles.go:151](../adapters/rosie/control/handles.go#L151).

| Schema state | Implemented meaning |
|---|---|
| prepared | Native preparation acknowledged; Start may be attempted under current authority. |
| started | Native Start accepted; observe correlated completion. |
| consumed | Correlated completion or a later accepted execution consumed the handle. |
| discarded | Native Discard acknowledged; no future Start. |
| superseded | Replacement committed and old native storage released. |
| retired | Stop, authority/connection loss or uncertain ownership; no future Start. |

## Recovery and motion semantics

Native jog reserves the next command interval plus the acceleration-bounded
stopping distance before each step. It scales the entire requested vector and
stops within one to two command counts inside a position boundary, preserving
Home and enable while authority and feedback remain valid. `jog_limited`
(reason 14) is an accepted governed state, not a failed session. EndJog and
expiry retain the stopping budget; release and begin an inward jog to move away.
This is command-level simulation qualification, not physical stopping evidence.
The final hard-limit, target-lead and tracking fault guards remain unchanged.

A trusted axis outside its configured position interval may Enable and Arm to
hold its feedback pose. This never starts recovery motion. Configuration, Home
(when required), coordinate, drive, bus and authority gates still apply; limits
validity means that the limits are configured, not that feedback is inside them.
The compiler permits ranges excluding zero. Home may therefore establish a valid
zero outside the range. Changing limits changes configuration identity but keeps
the coordinate identity, so a qualified anchor restore may establish an exterior
hold without Home or motion.

Explicit inward jog and ordinary trajectory requests may reduce the excursion.
Targets may cross the nearest boundary and continue within the inclusive
`[min, max]` interval, but cannot cross the opposite boundary. Exterior trajectory
prefixes must hold or progress inward monotonically; Start checks the first point
against the current held pose and retains its one-count continuity check.
Outward velocity at or beyond a boundary, an increasing exterior excursion, or
a step from outside across the opposite boundary returns `outside_limits_outward`
(native command reason 8, jog reason 15). A trajectory point that leaves an
already valid interval retains the existing `invalid_trajectory` refusal and
inhibition. A rejected fast update preserves the last accepted input
and its original deadline. Position-rate, target-lead and following-error guards
remain active; inhibited outputs retain their existing behavior. Dense program
imports retain their existing requirement that every sample lies inside limits.

`ROSIE_RT_ANCHOR_DIR` configures two distinct process responsibilities. The Go
`rt-control` adapter writes `axis-0.anchor` through `axis-8.anchor` for a nine-axis
Home when that variable is set **in the adapter environment**. The daemon only
reads anchors; setting the variable only for the daemon cannot create files.
Ordinary `home` always runs native Home, including when the directory is empty;
only explicit `restore_anchor` selects the native restoration path.

For brake-equipped independent-source axes, Home delays its final encoder
acquisition until fresh disabled feedback settles after the brake hold delay.
A bounded wait fails Home if it cannot establish the observation; no acceptance
tolerance, grant lifetime or motion permission is expanded.
A refused multi-axis Home may leave a partial volatile Home mask without a
successful capture receipt or saved anchors. Do not present that as full Home
or restart readiness. Individual-axis readiness remains scoped to the requested
mask; trajectory completion alone does not qualify the subsequent brake hold.

The benchdrive source candidate therefore observes the powered cleanup
transition after Stop/Release: output inhibition, an observed inactive grant
and lease, fresh selected-axis PDOs, and two advancing native/axis terminal
publications within the existing observation deadline. Late native faults remain
in the final error alongside any original failure. Missing/frozen observations
cannot produce a successful verdict. Timer APPLIED is not physical brake proof;
RELEASED also represents no-brake profiles, which the current public description
does not distinguish. No public protocol change or physical acceptance is implied.

After an accepted Home receipt, the adapter waits for one observed status with a
new `home_epoch`, `commissioning_phase=0`, every selected bit in `home_valid_mask`,
`armed=0` and `axis_enable_mask=0`. Every selected axis must also publish
`revision_readback_valid=1`, `pdo_fresh=1`, `coordinate_valid=1` and
`coordinate_source_valid=1`, with representable coordinate counts. Home validity
alone is insufficient: capture waits for the qualified coordinate publication,
then syncs and atomically replaces each selected file before reporting Home
success. This capture is part of the grant-owned Home operation, not a general
status-polling side effect. Losing the HTTP response does not cancel it; Stop,
grant expiry or transport loss still remove native permission and drain the job.

For `restore_anchor`, set `ROSIE_RT_ANCHOR_DIR` to the same persistent directory in
**both processes**; give the adapter write access and the daemon read access.
The default store creates `0600` files. To use different service users, provision a
dedicated adapter-owned setgid `2750` directory with a native-reader group; only
that exact directory mode opts new files into `0640`. Group-writable or
world-accessible directories do not opt in. This does not grant restoration
or motion authority, and existing files are not rewritten automatically.
Acquire and renew a fresh grant after restart, remain inhibited and idle, and
wait for verified configuration and current qualified source acquisitions on the
selected axes. For the independent A6 raw source, a matching saved anchor can
establish the command-frame relation before coordinate validity exists.
The stored checksum, adapter-bound `PairID` and `PairRevision`, per-axis `coordinate_identity_sha256`,
observed drive identity and tolerance must
match; native independently checks the saved encoder and command frames against
current evidence. Missing native files return `anchor_missing`, mismatched
identity `anchor_identity_mismatch`, invalid source `anchor_source_invalid`, and
disagreement `anchor_disagrees`. Success returns `RecoveryStatus` after the native
receipt with the selected `home_valid_mask`; per-axis status carries
`restored_anchor_home_epoch` from the saved Home. Restoration never enables or
arms motion. [controller.go](../adapters/rosie/control/controller.go),
[anchor store](../adapters/rosie/control/anchors/store.go).

Battery-backed A6 sampled evidence uses `raw_multi_turn_encoder|3` and bracketed
`0x2040:21/23` raw multi-turn reads. Its origin differs from the command PDO frame.
Per-axis `independent_anchor_source` publishes a separate raw count, completion
timestamp, configured age bound and native validity. This observation can qualify
an explicit restore offer while `coordinate_valid` and `coordinate_source_valid`
remain false. It cannot qualify anchor capture or motion; native repeats the
raw/command agreement, epoch and stationarity checks when Restore completes.
Explicit stationary Home or a matching saved anchor establishes the fixed
relation; startup, Arm and Load cannot silently bind or repair it. Old
`rotation_mode_encoder|1` anchors refuse identity checks and require explicit
Home. Fresh, coherent raw acquisition may permit clearing a coordinate-reference
fault while outputs remain inhibited, but Reset does not establish an origin or
grant Home. Until Home or qualified restore completes, Arm and motion remain
unavailable. Real source disagreement still invalidates evidence, including a
whole-turn raw encoder reset.
When qualified saved-reference recovery has already restored an axis's Home,
explicit Reset preserves that Home only with fresh feedback, trusted coordinates,
a valid coordinate source, known fault provenance, and no remaining untrusted
evidence for that axis. Reset does not create or restore a reference. This avoids
invalidating a successfully recovered reference and immediately re-latching the
same coordinate fault. Other fault types retain their existing reset policy.
Home completion preserves the observed displacement from logical zero; it does
not require the current position to equal the drive's Home offset exactly.
Home and Restore wait for a PDO receipt covering the completed acquisition before
native qualification. The worker uses the existing 250 ms completion deadline;
register operations and configured delays are unchanged. Cancellation, authority
loss, enabling or identity changes reject the pending completion. This changes
no public operation or acceptance threshold. Full-group
physical Home was subsequently confirmed on all six axes of a bench cell.
Internal commissioning diagnostics distinguish binding refusals and record the
associated evidence in the native worker log. They do not change public refusal
codes, Home sequencing, timing or acceptance. The subsequent J2 capture identified
a full-span stationarity refusal. Binding checks every position in the
acquisition interval relative to its fixed starting anchor. A 50-count
stationary margin applies to fresh Home only; Restore and runtime
checks remain 16 counts. Combined implementation 51a6a27a passes WSL coordinate,
worker and policy tests plus ARM coordinate/worker checks and configuration
validation; it is deployed on the bench cell, and the operator subsequently
completed Home on all six axes (native success logs and Home mask 63).
Persistent raw/PDO pairing error beyond 16 counts can still fail a later runtime
check after Home. Returning inside the budget after an excessive excursion does not
qualify. This change adds no public operation and preserves the established Home
timing; software validation passes, but the subsequent physical group attempt
refused J5 after J1-J4 completed. The later 50-count margin and per-axis changes
were followed by a successful six-axis operator Home.
The OLP Home indicators display Homed, Home required, Homing, Status stale or
Status unavailable per joint, plus an aggregate count. They use current native
Home evidence with HTTP and native-observation freshness, not request success.
This frontend presentation adds no Home protocol operation or sequence change;
93 focused UI tests, TypeScript and production build pass.
On a successful new acquisition, the adapter retires the previous cancelled
application execution's error after the native grant is accepted and authorized.
It does not clear native faults or change Home validity, order or timing.
This prevents a retained old error being interpreted as a new Home failure;
the Home race regression and full SDK/control suite pass. The adapter suite's
intermittent torch preparation cancellation reproduces on the unchanged baseline.
Implementation d61f3f73 is deployed through an adapter-only restart; physical
Home acceptance was subsequently confirmed after the later margin/per-axis change.
For a latched coordinate-reference fault, Home first requires the existing
qualified ResetFault path: fresh raw acquisition, disabled idle drives and
healthy CSP. OLP recovery orders ResetFault, observed fault clearance, fresh
acquisition of control, then Home; reset alone leaves Home invalid. Native
admission diagnostics and UI logs expose fault/recovery masks and the blocking
reason. This orchestration changes no native Home order, timing, threshold or
admission exemption. Native diagnostics fe651353 and app recovery f60d6f08 are
active. Ownership/timestamp fences and atomic conditional Stop checks address
delayed renewal, status and event responses crossing reset/reacquire. The final
denseexec race suite, 70 UI tests, TypeScript and production build pass; physical
Home acceptance is now confirmed on all six axes.

A subsequent dry-run start refused `start_discontinuity` with Home still valid.
A separate start-alignment correction, 57a5c81b, requires current Home, verified
configuration, qualified independent encoders and valid coordinates before using
max(existing completion tolerance, 50 counts). First sample time
and velocity must still be zero; position/rate limits and runtime 16-count
pairing checks remain unchanged. Per-axis refusal tracing records first/held
positions, delta, allowance, first velocity/time and caps. WSL alignment/readiness/
cycle-machine checks and the local build pass, as do ARM alignment/readiness and
configuration checks. The change is deployed on the bench cell; no successful
physical trajectory start is claimed. Startup Home is 0 after this restart,
distinct from the successful six-axis Home observed before it.

For battery-less A6 `single_turn_absolute`, raw U40.1C and command 6064 use
different frames after HM35. Native compares U40.1C plus its qualified wrapped
`single_turn_frame_offset` against the paired PDO, retaining the 16-count anchor
allowance. Completed Home establishes that relation; capture stores it in a v4
anchor. Restart startup may reuse the qualified stored relation but does not
grant Home. `restore_anchor` checks the raw encoder residue, saved command and
frame offset independently before granting Home. A record missing the offset
refuses as `anchor_source_invalid`, naming `single_turn_frame_offset` (native
`anchor_identity_changed_field` identifies the missing evidence field); a
contradictory relation refuses. Coordinate identity keys are unchanged.
Daemon restart is covered by simulation; power-cycle persistence after Home on
the battery-less mini-bench drive remains unverified.

Native coordinate-table mismatches publish coordinate reason `IdentityChanged`
(3) and the changed coordinate key. An adapter pair refusal takes precedence over
source validity; subsequent public status carries `anchor_identity_mismatch`,
`anchor_identity_changed_field` of `pair.id` or `pair.revision`, and reason 3 until
a replacement capture or matching pair qualification clears it. These pair fields
are adapter observations; private native status cannot assess adapter pair authority.
Other identity refusals, including unreadable legacy identity records, do not
necessarily supply a changed key or coordinate reason 3.

`recovery_status` requires authorized inhibited idle observation and changes no
fault or Home evidence. `reset_fault` defaults axis mask zero to all configured
axes, requires the same inhibited/idle boundary and reports the correlated
`reset_sequence`, fault bits, home mask and per-bit outcomes. A submitted reset
retires the session, including on a rejected receipt. Missing correlated status
returns `recovery_observation_unavailable`; persistent conditions return
`fault_persists`. Reset never starts motion.
[adapters/rosie/control/reset_linux.go:20](../adapters/rosie/control/reset_linux.go#L20),
[adapters/rosie/control/reset_linux.go:88](../adapters/rosie/control/reset_linux.go#L88).

Current fault records additionally carry optional `reset_preserves_home` and a
dynamic `rehome_axis_mask` computed from the same native decision used by Reset.
The advisory is true only for a complete, fresh, trusted, already-homed configured
axis group, a healthy verified bus without drive faults, and inhibited idle
execution. Missing provenance, stale/future/missing observation timestamps, partial
axis evidence or any persistent fault leave it false. A recovered but still
latched `coordinate_reference` may therefore be explicitly reset without changing
Home. The static recovery class still describes the fault type; it is not current
proof that Home remains lost. Clients must treat a missing advisory as false,
recheck current admission when Reset is pressed, and never use it to auto-reset
from Arm or resume motion. `outcomes` and `reset_sequence` continue to describe
the last attempted reset, separately from this current candidate. The candidate
does not perform recovery, change a reference, issue a drive reset, or clear a
fault. Native independently re-evaluates Reset on receipt.
Both the candidate and actual Reset derive freshness from a complete-exchange
bit and a nonzero PDO observation timestamp no more than two configured cycles
old and not in the future. A previously eligible snapshot cannot clear a
reference fault after its current feedback fails that check.

| Outcome | Meaning |
|---|---|
| `cleared` | The correlated reset cleared this selected fault evidence. |
| `persists` | The condition still prevents clearing; no partial reset is committed. |
| `rehome_required` | After reset, `rehome_axis_mask` needs qualified Home recovery before motion. |

The following native fault matrix is rendered from `include/fault_recovery.hpp`.
Trust loss can require Home even when the unconditional column says false.
Unknown bits persist. [include/fault_recovery.hpp:104](../include/fault_recovery.hpp#L104).

| Bit | Name | Unconditionally invalidates Home | Drive reset | Reverification | Recovery class |
|---|---|---|---|---|---|
| 0 | start_discontinuity | false | false | false | reset_clears |
| 1 | position_rate | true | false | false | rehome_required |
| 2 | cycle_deadline | false | false | false | reset_clears |
| 3 | cycle_sleep | false | false | false | reset_clears |
| 4 | target_lead | false | false | false | reset_after_condition_clears |
| 5 | following_error | false | false | false | reset_after_condition_clears |
| 6 | coordinate_reference | true | false | false | rehome_required |
| 7 | drive_readiness | false | true | true | reset_after_condition_clears |
| 8 | bus_transport | false | false | true | reset_after_condition_clears |
| 9 | completion_timeout | false | false | false | reset_clears |
| 10 | brake_hold | false | false | true | reset_after_condition_clears |
| 11 | pdo_mapping_mismatch | true | false | true | restart_required |
| 12 | collision_watchdog | false | false | false | reset_after_condition_clears |

Bit 4 `target_lead` (changed 2026-10-02, AMR-720) is a command/tracking fault,
not lost position. It invalidates Home only through the conditional trust rule
above (untrusted evidence). Reset needs fresh feedback and a valid coordinate
source. It is never recovered by re-running HM35, which would redefine the
joint's zero at its fault pose.


Bit 10 `brake_hold` is retained wire vocabulary only: the native brake guard no
longer raises it. Every brake-hold detection (opening/release, closing, settling,
held displacement, legacy anchor tolerance) is a per-axis warning in
`axes[i].brake_hold_diagnostic` plus one `brake_warning` event per new
`warning_sequence`. A warning never sets `safety_fault_mask`, revokes outputs,
invalidates Home, disarms or refuses Arm/Enable/jog/trajectory, and needs no
reset or restart. No brake reset proof is supplied, so a latch of this bit, if
ever observed, persists. The guard re-anchors at the settled rest (quiet applied
dwell, re-enable, Home or an Arm request once the drive reports the brake
applied); continued travel beyond its displacement bound warns again.

`collision_watchdog` identifies the affected axes through the native recovery
matrix's `axis_mask`; `execution_fault_reasons & 4096` is its typed bit. Each
native `core.axes[i]` and public `axes[i]` carries `collision_watchdog` with
`armed`, `torque_cycles`, `following_error_cycles`, `peak_torque_raw`,
`peak_following_error_counts`, `last_trip_quantity`, `trip_sustained_cycles`,
`trip_count`, `samples`, `last_trip_peak_torque_raw` and
`last_trip_peak_following_error_counts`. Last-trip peaks survive reset/rearm. Quantities are `none`, `torque`, `following_error`
or `torque_and_following_error`. Independent consecutive counters require
2..1000 strictly exceeding cycles, never a single-sample decision. The Stop fence
cancels execution and inhibits the whole group; recovery is explicit, requires
fresh feedback below both bounds and does not start motion.

Native cycle peaks include enabled hold since the latest arm transition even
when watchdog decisions are disabled, and freeze at trip/disarm. `samples` counts
those fresh cycles; zero cannot establish a tuning measurement. Public status
forwards native values without resampling or estimating peaks. Current
`torque_raw` is signed per-mille of rated torque; use only with `pdo_fresh`.
Mapped valid 0x60F4 supplies following error; otherwise the native evaluator uses
the previous submitted wire target minus feedback in the configured count frame.
On a machine that declares `tracking_lag_allowance`, the following-error bound is
`following_error_counts_max` plus |this cycle's commanded velocity, counts/s| x
the allowance (CONTRACTS.md, "Speed-scaled tracking allowance"); at zero commanded
velocity, or without an allowance, it is the static value. Torque is never scaled.

Cell axes (including robot-definition cells) or their drive profile may declare:

```json
"collision_watchdog": {
  "torque_abs_max_raw": 1000,
  "following_error_counts_max": 8000,
  "sustained_cycles": 10,
  "_unverified": "unverified: starting values; remeasure with robot loads",
  "_source": "tools/rtctl/testdata/collision-watchdog-unverified.json#J1"
}
```

Torque range is 1..1000, following-error range 1..2147483647 counts, persistence
2..1000 cycles. Both provenance strings are required and nonempty. A cell block
replaces the entire profile block, and torque must be mapped. Explicit disable is
`{"disabled":true,"reason":"document why","_unverified":"unverified","_source":"decision reference"}`
without thresholds. An absent block disables with an axis-specific compile
warning. Effective enabled values enter the native profile and machine identity;
threshold changes preserve the coordinate-frame identity. See
[the tuning recipe](BENCH_TESTING.md#collision-threshold-tuning-p4-17) and
[the collision rule](CONTRACTS.md#collision-watchdog-rule-p4-17).

Describe includes protocol/backend, cycle period, native storage capacities,
`configuration_sha256`, verified `machine_sha256`, separate `deployment_sha256`,
contract version/digest and every capability state. Its axis array gives IDs,
order, units, conversion/wrapping, Home capability, effective limits and tracking
and completion budgets. Detached prepared `program` metadata includes both
`source_digest` and `normalised_digest`, alongside the complete Identity.
StartProgram must echo the prepared identity exactly. Torch-bearing programs are
refused during Prepare; configured non-torch output markers require explicit
`io_arm` (P6-05).

The dense axis map is the cell's own description: described rotary axes in native
index order, one column each (six for Rosie 1420; nine for a nine-axis cell).
No start flag or cell config field names it; `--dense-axis-ids` was removed.
Columns beyond this map must be uncommanded in the header mask and still in every
sample, otherwise admission returns `unmapped_dense_axis`. More rotary axes than
the nine-column wire can carry returns `dense_axis_map_requires_nine_ids`.
[adapters/rosie/control/http.go:64](../adapters/rosie/control/http.go#L64),
[adapters/rosie/control/controller.go:1507](../adapters/rosie/control/controller.go#L1507).

Per-axis and top-level interpolation is
`hermite_position_with_velocity, linear_without`, feedforward `qd_optional`, endpoint
`hold_last_sample_then_settle`. Both knots supplying qd select cubic Hermite position
and its derivative as feedforward; either knot lacking qd retains linear position
and segment-slope feedforward. Position deltas use shortest-step command wrapping.
Adjacent Hermite segments meet at the shared position and supplied velocity. At/after the final sample,
position holds and velocity is zero; continuous arrival requires zero final qd.
At Start, the existing one-count command-continuity allowance remains. Larger
held-command offsets may settle only within each axis's existing completion
tolerance, with a stationary sampled first velocity and finite positive velocity
and acceleration bounds. Native execution emits a bounded quintic prefix before
the original first point; the plan identity and samples do not change. Original
plan time and process-I/O markers wait until that endpoint. Stop, lease expiry,
readiness and output guards still apply. An offset outside the tolerance or an
ineligible start remains `start_discontinuity`; Reset clears that fault but does
not reposition the robot.
Prepare checks both position endpoints and every interior velocity zero against
the position limits in the sampler's projected command frame. Wrap seams include
both one-sided limits. A position refusal returns `native_limit_exceeded`;
`SegmentPositionError` retains axis, segment, ending sample, position and limit
in logical units. Native Commit returns `invalid_trajectory`, sets the affected
axis mask and logs `hermite_position_extremum` with axis, segment, position and limit.
Interpolated recovery: a Hermite segment with an exterior endpoint is not held to
the interior extrema bound (that would refuse every recovery from outside the
limits); its interpolated positions are checked as ordered monotone pieces in the
nearest command-circle interval and may never lie further outward than the more
outward endpoint. A bow outward is refused with `outside_limits_outward` (native
command reason 8) and logged as `hermite_recovery_extremum`; segments with both
endpoints inside keep the strict interior bound above; linear segments are
unaffected. Dense program admission mirrors the same rule before upload.
Prepare also checks the quadratic derivative's endpoints and interior
vertex against effective native velocity limits. Dense program admission also
checks the declared per-axis `max_abs_qd_rad_s`; violations return
`native_segment_rate_exceeded` (`SegmentVelocityError` supplies axis, segment,
ending sample, peak and limit in rad/s to typed Go callers). Native Prepare rejects
as `invalid_trajectory` and logs `hermite_velocity_extremum`, axis, segment and peak
in counts/s. No curve is silently clipped or retimed.

Position and Hermite velocity admission refusals also carry optional
`data.limit_violation` with `kind` (`position` or `velocity`), zero-based
`segment` and `sample`, `axis`, `value`, `limit`, and `unit` (`rad` or
`rad/s`). This supplements the stable refusal reason. The Go SDK retains
the original response data in `Rejected.ResponseData`; OLP forwards the
numeric evidence and displays the joint and limit in degrees. For older
adapters, OLP can recover an unambiguous endpoint crossing from the refused
sample and the verified machine description after rejection. It never
changes admission, retries the upload or infers a drive fault from it.

Consumers must keep one count of interior position margin (`1/counts_per_unit`
rad), including interpolated extrema. The adapter preserves the supplied program;
it does not round or clamp it inward. Position/Hermite admission now uses the
shared binary64 arithmetic below, but native acceleration admission still uses
extended count arithmetic and can refuse an exact logical acceleration bound.
Keep one count/s of velocity headroom and one count/sÂ² of acceleration headroom
(`1/counts_per_unit` in the corresponding logical units); native Prepare remains
the final admission authority. These margins cover count-conversion rounding,
not an otherwise out-of-limit curve or an invalid motion frame.

`test_exact_bound_count_divergence` measures binary64 versus extended conversion
at Â±0.1, Â±0.5 and Â±Ï€ rad, for 1:1, 50:1 and 100:1 scales with 131072 wire
counts/revolution and the review's conservative 2086254 counts/rad ceiling.
The maximum difference is 3.5333869163878262e-10 counts before Home subtraction
and 2.1839514374732971e-7 counts with signed-32-bit Home offsets; both are below
0.000001 count. For a representable signed-32-bit CSP target and Home offset,
the two binary64 rounding steps contribute at most 2^-21 + 2^-22 counts
(7.152557373046875e-7 counts): the pre-offset product is bounded by 2^32 and
the result by 2^31. This bounds conversion error, not polynomial extrema.
Exact acceleration fixtures differ by at most
2.3419488570652902e-11 counts/sÂ² and demonstrate native refusals eliminated by
the one-count/sÂ² margin. These are fixture measurements, not a general bound
on arbitrary polynomial evaluation. The described 100:1 scale is
2086075.6700940905 counts/rad, giving a one-count margin of
4.7936899621426291e-7 rad. `TestProgramLargestRatioExactPositionLimit` confirms
that stationary programs at both Â±0.5 rad limits and one count inside them
pass both adapter and native admission on that axis. `busy` covers both an
occupied preparation slot and native capacity refusal; only the latter carries
`native_result.reason=capacity`, as checked by `TestProgramBusyRefusalSources`.

Limit refusals name the first violating sample in time order; among simultaneous
violations the lowest axis index. Axis indices follow native Describe order,
including when a dense program remaps its source axes. An interval violation is
attributed to its ending sample; native `segment` is the zero-based starting point
index and the log's `sample` is the ending point index. Initial knot violations
name sample zero. Format, identity and clock validation still precede limit
diagnostics. A permitted exterior recovery knot is not a position violation.

Producers can reuse `densejointtraj.BoundHermiteSegment(CommandFrame, first,
second HermiteKnot)` in `adapters/rosie/densejointtraj/hermite.go`. It returns
`MinPosition`, `MaxPosition` and guarded `PeakVelocity` in counts and counts/s.
The adapter calls this same helper. `CommandFrame` uses Describe's
`counts_per_unit`, `sign`, and the current status axis's
`native_home_position_offset`. `DecodeProgram` optionally accepts one Home offset
per native axis; omission is the zero-Home frame for offline producers. Live
preparation supplies current offsets. Home/configuration changes require fresh
admission; they do not grant a reusable frame-independent acceptance.

The exact supplied-velocity admission arithmetic is:

1. Round each source time independently: `t_ns = round(t_s * 1e9)`, nearest integer
   with ties away from zero. Reject negative, collapsed or overflowing clocks.
   Use `dt = float64(t1_ns - t0_ns) / 1e9`; do not subtract source seconds or
   convert with a seconds-plus-remainder expression.
2. In unfused IEEE binary64 arithmetic, set `s = counts_per_unit * sign`,
   `p0 = q0*s - home_offset`, `p1 = q1*s - home_offset`,
   `v0 = qd0*s`, `v1 = qd1*s`, and `d = p1-p0`. Round each multiplication before
   its following addition/subtraction. Do not multiply a logical position
   difference by the scale, round positions to integer counts, or apply gearing again.
3. Set `c2 = 3*d - dt*(2*v0+v1)` and `c3 = -2*d + dt*(v0+v1)`.
   For normalized `u` in `[0,1]`, position is
   `p0 + u*(dt*v0 + u*(c2 + u*c3))`, and velocity is
   `v0 + u*(2*c2 + (u*3)*c3)/dt`. Operations round individually to binary64;
   do not use extended precision or fused multiply-add for admission.
4. Position candidates are both endpoints and every root of
   `a*u*u + b*u + c = 0` strictly inside `(0,1)`, with
   `a=3*c3`, `b=2*c2`, `c=dt*v0`. For `a=0, b!=0`, use `u=-c/b`.
   Otherwise let `D=b*b-(4*a)*c`. For `D>=0`, use the stable numerator
   `n=-0.5*(b+copysign(sqrt(D),b))` and roots `n/a` and `c/n`;
   if `n=0`, use `-b/(2*a)`. Compare the range directly to the sorted bounds
   `min_position*s-home_offset`, `max_position*s-home_offset`.
   Equality with a position bound passes; no position slack is added.
5. The raw absolute velocity peak is `max(abs(v0),abs(v1))`, also including
   `abs(v(u))` at `u=-c2/(3*c3)` when `c3!=0` and `0<u<1`.
   Guard every nonzero peak by `nextafter(peak,+infinity)`; zero remains zero.
   Compare this guarded count value directly to
   `min(Describe.max_velocity, source.max_abs_qd_rad_s)*counts_per_unit`.
   Native uses its effective Describe velocity cap. Reject strictly greater
   values; equality passes. A raw exact-bound nonzero peak therefore needs one
   binary64 ULP of headroom. Never divide the peak back into radians before this
   comparison or add an epsilon to the cap. Nonfinite arithmetic rejects.

The raw secant of the unwrapped logical endpoints is not a separate supplied-qd
check: it describes a different path when command wrapping applies. Missing-qd
native segments retain their existing linear point and secant checks. Source knot
limits, format/digest validation and the following sampled acceleration checks
still apply in addition to the continuous Hermite bounds.

Position and velocity checks are `declared`; acceleration is `declared`
only with `max_acceleration`, otherwise `not_declared`; jerk is `unsupported`.
For URDF-mapped robot axes, compiled `max_acceleration` and jog acceleration
come from the registered robot config's per-joint acceleration; travel and
maximum velocity come from its URDF. Machine motion-limit overrides are refused.
The A6 drive profile does not impose a separate acceleration cap.
The sampled acceleration calculation checks qd differences per segment, or
second q differences across segment midpoints when qd is absent. Mixed qd fails
when acceleration is declared. This remains a sampled check; it does not bound
continuous Hermite acceleration, linear corners or endpoint hold. A mapped velocity PDO alone does not qualify
physical feedforward. [include/interpolation_policy.hpp:45](../include/interpolation_policy.hpp#L45).
On the A6-EC in CSP (mode 8) the written 0x60FF is ignored: the drive follows
0x607A only, and its speed feedforward, when configured, is its own derivative
of 0x607A (C01.13 = 1, startup keys of the machine configuration). The
`velocity` a point carries therefore shapes the sampled position (Hermite) and
the speed-scaled tracking bounds, not the drive's speed loop
([CONTRACTS](CONTRACTS.md#drive-speed-feedforward-2026-10-06), P4-19).

## Jog lanes and remote listener

Legacy `jog` uses the JSON/control lane. Independent jogging uses BeginJog,
binary latest-value UpdateJog and EndJog. Obtain `/v1/jog/clock`, supply its
incarnation to BeginJog and retain original monotonic origin/expiry. Sequence,
grant and jog generation checks prevent replay; rejected input never refreshes
expiry. The native source-age ceiling is 250,000,000 ns. End/expiry and public
Halt use the shared bounded ramp; a failed bound inhibits. The arithmetic is
qualified across all 40 supported periods; powered-hardware stopping remains
unverified pending the bench session (P4-05/P5-01; CONTRACTS.md R11).
[adapters/rosie/control/jog_linux.go:257](../adapters/rosie/control/jog_linux.go#L257),
[engine/jog_lane.hpp:240](../engine/jog_lane.hpp#L240),
[protocol/control.json:12](../protocol/control.json#L12).

The optional remote listener requires component-owned mutual TLS 1.3, verified
principal and pair URI SANs, and a current signed CRL. Certificate/key/CRL reload
occurs on file modification for full handshakes; this is not active-connection
revocation. Unauthenticated traffic never reaches the application, including Stop.
Binding the component principal to the real Rosie pair owner is merged under
P6-03 with acceptance pending. Clock uncertainty, drift and calibration-age flags default to zero,
which leaves remote jog clock qualification disabled.
[adapters/rosie/control/remote_tls.go:50](../adapters/rosie/control/remote_tls.go#L50),
[adapters/rosie/control/remote_listener.go:25](../adapters/rosie/control/remote_listener.go#L25),
[cmd/rt-control/main.go:38](../cmd/rt-control/main.go#L38).

Remote `/v1/jog` exclusively upgrades to WSS with the current session,
grant generation and jog generation in the query. Two JSON calibrations bound
source-to-host offset before binary updates. Success updates have no per-update
reply; rejections are observable. Fragmentation is refused and payloads are
bounded at 4,096 bytes. Use generated frames rather than a second hand codec.
[adapters/rosie/control/jog_ws.go:130](../adapters/rosie/control/jog_ws.go#L130),
[adapters/rosie/control/websocket.go:80](../adapters/rosie/control/websocket.go#L80).

Calibration input fields below follow the `jogCalibration` wire tags. This JSON
exchange is implemented in the WSS handler rather than declared as a schema type.
The second source sample must be taken after receiving the first reply and echo
its `host_tx_ns`; the first `calibrated` reply alone does not establish a qualified
clock. Subsequent calibration can refresh the mapping.
[adapters/rosie/control/jog_ws.go:33](../adapters/rosie/control/jog_ws.go#L33),
[adapters/rosie/control/jog_ws.go:46](../adapters/rosie/control/jog_ws.go#L46).

| Calibration field | Wire type | Admission semantics |
|---|---|---|
| `type` | string | Exactly `calibrate`. |
| `source_incarnation` | string | Hex encoding of a nonzero incarnation occupying 16 bytes; must match the established source. |
| `source_ns` | uint64 | Nonzero source monotonic timestamp in ns. |
| `seq` | uint64 | Nonzero, strictly increasing calibration sequence. |
| `host_tx_ns` | uint64, optional initially | Echo the previous reply's host transmit timestamp to qualify the next exchange. |

| Upgrade query field | JSON/query type | Required | Constraint |
|---|---|---|---|
| session | string | True | Exact current session, owned by the authenticated remote principal. |
| generation | integer | True | Uint64 exact application grant generation, encoded as decimal query text. |
| jog_generation | integer | True | Uint64 exact current jog generation, or zero to prepare transport without native motion, encoded as decimal query text. |

| Upgrade rule | Schema contract |
|---|---|
| endpoint | GET /v1/jog on the mutual-TLS listener is exclusively WebSocket upgrade; local GET /v1/jog is the JSON jog_status endpoint. |
| frames | Two calibrate JSON exchanges establish clock bounds before binary local_jog_update frames from protocol/control.json are admitted; successful updates do not get per-update replies. Calibration replies have type, seq, source_ns, host_rx_ns, host_tx_ns; rejection replies have type, seq, reason. With jog_generation=0, send begin with seq=3, axis_mask, a fresh source_ns and the second host_tx_ns after both calibration replies. The server uses the post-calibration Begin sample's earliest host bound as the capture cutoff, independently starts the native first-input allowance at receipt, and replies begun with seq and jog_generation. The native cell age bounds cutoff freshness and the separate wait, and every input retains its original age/deadline checks. Input captured before the Begin source_ns is rejected before clock conversion. Binary input before begin and repeated begin close the stream. Existing nonzero-generation upgrades remain supported. |
| headers | HTTP/1.1 Connection: Upgrade, Upgrade: websocket, Sec-WebSocket-Version: 13 and a base64 Sec-WebSocket-Key decoding to 16 bytes. |
| response | HTTP 101 upgrades to the dedicated jog lane. Failures before upgrade may be text/plain HTTP 400, 403 or 409; unauthenticated remote requests are denied before the application handler. No Response envelope wraps WebSocket frames. |

The shared native schema defines a 224-byte, 8-byte-aligned, little-endian
`local_jog_update`; reserved storage is zero. On the local lane times use the
negotiated host clock; WSS converts source times through its calibrated mapping.
[protocol/control.json:192](../protocol/control.json#L192).

| Binary field | Type | Offset (bytes) | Size (bytes) |
|---|---|---|---|
| session_id | u8[32] | 0 | 32 |
| clock_incarnation | u8[16] | 32 | 16 |
| grant_generation | u64 | 48 | 8 |
| jog_generation | u64 | 56 | 8 |
| source_sequence | u64 | 64 | 8 |
| origin_host_ns | u64 | 72 | 8 |
| deadline_host_ns | u64 | 80 | 8 |
| axis_mask | u32 | 88 | 4 |
| reserved | u32 | 92 | 4 |
| velocity | f64[16] | 96 | 128 |

### Path jog (protocol 1.14)

A path jog plays a joint-space path the application streams (for a Cartesian
hold: seeded IK of the frozen line every sample) along one native path speed,
so all masked axes start, slow and stop together on the sampled curve.
Describe carries `path_jog {max_samples, max_chunk, max_sample_step_s}` only
when the machine configures `jog_jerk`; without it, refuse path holds rather
than falling back to velocity jog. `begin_path_jog` takes the `begin_jog`
fields and returns the jog generation. Samples are `local_path_samples`
frames (2272 bytes, told apart from the 224-byte `local_jog_update` by size)
on `jog.sock` or as WSS `/v1/jog` binary frames: at most `max_chunk` samples
of every native axis in logical joint units (unmasked axes zero), optional
per-sample caps (+Inf none), a `chunk_sequence` that starts at 1 and increases
by one, and an end-of-path flag with an application reason. Chunks carry no
time. They are acknowledged one at a time: locally poll `GET /v1/jog/path`
(`PathJogStatus.accepted` / `refused` / `reason`), remotely read `path_ack`
text frames. Sample 0 must equal the held command target within one count;
read it from a status sample taken at rest. Path speed is a
`local_jog_update` with `flags` bit 0: `velocity[0]` in path units/s, every
other entry zero, the session's whole mask, origin = capture time and deadline
= origin + lifetime; remote speed frames use the stream's clock mapping like
velocity frames. Append geometry before the first speed. End, Halt and input
expiry stop along the path; the session closes when the stop completes.
`PathJogStatus.observation` is the native `path_observation` read from the
fast region (not the 10 Hz status file). The Go SDK wraps both transports in
`Client.NewPathJogSession`; the C++ client does not support path jog. See
[CONTRACTS.md](CONTRACTS.md#path-jog-lane-protocol-114-2026-10-07).

### Native reasons

| Name | Code |
|---|---|
| none | 0 |
| invalid_command | 1 |
| not_ready | 2 |
| mode_conflict | 3 |
| unknown_handle | 4 |
| capacity | 5 |
| invalid_trajectory | 6 |
| outside_limits_outward | 8 |

### Native jog reasons

| Name | Code |
|---|---|
| none | 0 |
| closed | 1 |
| mode_conflict | 2 |
| not_ready | 3 |
| wrong_identity | 4 |
| invalid_generation | 5 |
| stale_sequence | 6 |
| invalid_vector | 7 |
| invalid_timing | 8 |
| expired | 9 |
| authority_lost | 10 |
| clock_reset | 11 |
| generation_exhausted | 12 |
| ramping | 13 |
| limited | 14 |
| outside_limits_outward | 15 |

### Native grant reasons

| Name | Code |
|---|---|
| none | 0 |
| invalid | 1 |
| connection_mismatch | 2 |
| native_generation_mismatch | 3 |
| disconnected | 4 |
| busy | 5 |
| not_inhibited | 6 |
| stale_generation | 7 |
| identity_mismatch | 8 |
| expired | 9 |
| clock_regression | 10 |
| inactive | 11 |
| axis_mask_mismatch | 12 |

## Status and events

Status reports native diagnostics plus logical axes, generation/epoch identities,
plan cursor, buffer health and adapter observation. Invalid logical coordinates
must not be treated as valid positions; inspect validity and timestamp fields.
[adapters/rosie/control/events.go:292](../adapters/rosie/control/events.go#L292).

Poll `/v1/events?after=<uint64>` for a Response containing EventBatch. Stream
`/v1/events/stream?after=<uint64>` as SSE; each message has `id`, `event` and an
Event JSON `data`, not a Response wrapper. Omitted `after` means zero; duplicate,
nondecimal or future cursors return HTTP 409 with `events_cursor_invalid` or
`events_cursor_ahead`. SSE resumes with the query cursor;
the handler does not parse Last-Event-ID. Sequences increase within an adapter
incarnation. A slow reader gets `events_dropped` using the last lost sequence,
then retained events with their original numbers. Restart requires a new cursor.
Events use daemon observation time; native publications may coalesce intermediate
cyclic transitions. They contain no session capability secret. Local filesystem
permissions and remote mutual TLS protect both routes.
[adapters/rosie/control/events.go:48](../adapters/rosie/control/events.go#L48),
[adapters/rosie/control/events.go:330](../adapters/rosie/control/events.go#L330),
[adapters/rosie/control/remote_listener.go:122](../adapters/rosie/control/remote_listener.go#L122).

| Schema event type |
|---|
| grant_acquired |
| grant_renewed |
| grant_released |
| grant_expired |
| grant_revoked |
| enable_changed |
| arm_changed |
| handle_transition |
| execution_started |
| execution_completed |
| execution_faulted |
| execution_aborted |
| jog_begin |
| jog_end |
| jog_expired |
| jog_ramping |
| jog_limited |
| fault_latched |
| fault_cleared |
| home_epoch_changed |
| daemon_incarnation_changed |
| adapter_incarnation_changed |
| publisher_overflow |
| events_dropped |

## Schema rules

These are the schema text, including the handler exceptions identified above.

<!-- BEGIN SCHEMA RULES -->
| Rule | Contract |
|---|---|
| cell_io.fencing | Stop commands OFF in the fence cycle; ON requires a fresh grant, explicit io_arm and valid OFF readback observed after the fence cycle. |
| cell_io.intent_vs_readback | Intent and commanded values never establish independent physical feedback. |
| cell_io.marker_collision | Multiple markers for the same output in one native exchange fault; transitions are never silently collapsed. |
| cell_io.marker_delivery_cycles_unverified | 1 |
| cell_io.qualification | P6-05: torch remains refused. Generic terminal mappings and cycle bounds are unverified on hardware. |
| cell_io.readback_cycles_unverified | 3 |
| constraints.axis_mask | Native 32-bit axis mask constrained to the configured group; reset_fault uses zero for all axes. |
| constraints.binding | controller is 1..63 bytes; pair_id is nonempty; revision nonzero and equal to configured pair revision; configuration_sha256 equals native description. |
| constraints.fence | Current unpredictable session and exact generation; acquire has no fence; restart fencing takes precedence for dispatched operations. |
| constraints.handle | Opaque native uint64 identifier, nonzero for start/discard; terminal handles never regain permission. |
| constraints.identity | start_program requires exact equality with the prepared identity. |
| constraints.jog | Legacy timeout_ms 1..250; independent jog uses negotiated monotonic host origin, original non-extended deadline and clock incarnation. |
| constraints.points | 2..250000 points, time_ns int64 nanoseconds; first time zero, increasing subsequent times; position/velocity in described axis order and logical rad/m units; native limits and continuity enforced. |
| control_timing.cell | control.link_profile is lan (500000000/250000000 ns) or internet (unverified 3000000000/750000000 ns). Explicit max_grant_lease_ns and max_jog_input_age_ns override ceilings; unverified absolute bounds 10000000000 and 2000000000 ns. Lease overrides use whole ms. Compiled configuration and native machine identity bind effective nondefault policy. |
| control_timing.negotiation | Describe publishes max_grant_lease_ns and max_jog_input_age_ns. Acquire requests requested_lease_ms, returns effective lease_ms. Clients size at least three measured round trips plus cadence, capped by cell; renew at most one third of effective lease on a dedicated connection. An insufficient lease fails closed, typed control_session_stale. |
| control_timing.safety | Longer effective leases increase worst-case unattended-stop delay after operator link loss. Longer input-age ceilings prolong stale velocity application before immutable expiry and the existing jog expiry ramp. Clock uncertainty remains part of worst-case age, never subtracted as network latency. |
| envelope_before_points | Large prepare_trajectory requests must put schema, operation, session and nonzero generation before points within 16384 bytes. Unknown envelope fields and duplicate prefix fields are rejected; full decoding disallows unknown fields. Small unordered bodies fitting the prefix retain existing admission behavior. |
| error_patterns | ["unmapped_dense_axis: <axis>","native_limit_exceeded: segment=<index> sample=<index> axis=<id>","discard preparation <handle>: <cause>","jog_native_rejected_<number>","remote CRL signature: <cause>","<underlying diagnostic>","<cause>\\n<cleanup cause>"] |
| errors | The reasons list is the closed catalogue of adapter-owned literal reason labels (including startup/TLS failures). Existing error is also a diagnostic string: JSON, I/O, context, native and dense decoder errors and errors.Join are forwarded unchanged; these open diagnostics are not additional stable reason labels. |
| events.batch_limit | 64 |
| events.capacity | 256 |
| events.ordering | Sequences increase within adapter_incarnation. An events_dropped record uses the last lost sequence as its resumable cursor. Retained events keep their original sequences. A future cursor is rejected; adapter restart requires a new cursor. |
| events.readers | Unix socket filesystem permissions and the existing remote mutual-TLS principal/pair verification apply to both event routes. Events contain no grant session/capability secret. |
| events.timestamp | time_ns is the daemon publication used to observe a change, not adapter wall time. Native snapshots can coalesce transitions between publications; this stream preserves observations, not every cyclic transition. |
| external_enable | A6-EC manual chapter 11, 2004h/C04 and 60FDh: bits 16..20 report DI1..DI5 logic; function 1 selects S-ON, function 4 Emergency stop. Wire the cell stop-chain auxiliary contact to a declared DI and configure/read back its function and polarity to observe it. S-ON observation requires exactly one explicit function-1 startup assignment. A dedicated cyclic STO indication is not documented; S-ON/DI observations are diagnostics and never safety certification or a control decision. |
| home_authority | Home is established only by drive homing (HM35), which writes 607C. There is no software current-pose zero; a persisted current-pose (rosie-anchor-v5) anchor is refused as anchor_source_invalid: current-pose reference anchors are not accepted; establish Home with drive homing (HM35). |
| recovery | reset_fault is interim: requires inhibited idle native observation, reports correlated reset_sequence, per-bit faults/outcomes and home masks, and retires the session after a submitted result; never starts motion. restore_anchor returns RecoveryStatus from a native publication after the restore receipt, with the restore command sequence in Response.sequence; reset_sequence remains the last fault reset sequence. |
| remote_jog_upgrade.endpoint | GET /v1/jog on the mutual-TLS listener is exclusively WebSocket upgrade; local GET /v1/jog is the JSON jog_status endpoint. |
| remote_jog_upgrade.frames | Two calibrate JSON exchanges establish clock bounds before binary local_jog_update frames from protocol/control.json are admitted; successful updates do not get per-update replies. Calibration replies have type, seq, source_ns, host_rx_ns, host_tx_ns; rejection replies have type, seq, reason. With jog_generation=0, send begin with seq=3, axis_mask, a fresh source_ns and the second host_tx_ns after both calibration replies. The server uses the post-calibration Begin sample's earliest host bound as the capture cutoff, independently starts the native first-input allowance at receipt, and replies begun with seq and jog_generation. The native cell age bounds cutoff freshness and the separate wait, and every input retains its original age/deadline checks. Input captured before the Begin source_ns is rejected before clock conversion. Binary input before begin and repeated begin close the stream. Existing nonzero-generation upgrades remain supported. A path session sends begin with "path": true. Binary frames of local_path_samples size carry path chunks (no time); local_jog_update frames with flags bit 0 carry path speed and use the same clock mapping as velocity frames. Each chunk is acknowledged by a path_ack text frame with seq (chunk_sequence) and reason (0 accepted). |
| remote_jog_upgrade.headers | HTTP/1.1 Connection: Upgrade, Upgrade: websocket, Sec-WebSocket-Version: 13 and a base64 Sec-WebSocket-Key decoding to 16 bytes. |
| remote_jog_upgrade.request_fields | [{"constraints":"Exact current session, owned by the authenticated remote principal.","json_type":"string","location":"query","name":"session","required":true},{"constraints":"Uint64 exact application grant generation, encoded as decimal query text.","json_type":"integer","location":"query","name":"generation","required":true},{"constraints":"Uint64 exact current jog generation, or zero to prepare transport without native motion, encoded as decimal query text.","json_type":"integer","location":"query","name":"jog_generation","required":true}] |
| remote_jog_upgrade.response | HTTP 101 upgrades to the dedicated jog lane. Failures before upgrade may be text/plain HTTP 400, 403 or 409; unauthenticated remote requests are denied before the application handler. No Response envelope wraps WebSocket frames. |
| request_id | Optional on JSON control requests only; 1..64 printable ASCII bytes; null and non-string values rejected. The current session caches 256 outcomes; identical decoded payload retries join/replay, changed payload conflicts; release/expiry detaches the cache. |
| required_fields | Type.required describes mandatory response/codec fields. Request fields default when omitted; capability.required_fields lists semantically necessary input for valid admission. Native operation-specific defaults and constraints remain authoritative. Codec parsing does not authorize or execute motion. |
| resources | GET only. Immutable bytes, Content-Length, declared Content-Type and quoted digest ETag. Unknown or malformed digests return resource_unknown with HTTP 404. No repository lookup at request time. Max resource 33554432 bytes, set 134217728 bytes and 1024 entries (unverified bounds). |
| response | Response.data is the capability response type when named; sequence and handle are envelope fields. Errors return HTTP 409 except body size HTTP 413; success HTTP 200. Native receipts use their existing case-sensitive JSON field names. |
| telemetry | application/octet-stream only: generated 312-byte little-endian TelemetryBatchHeaderV1 followed by record_count full 5392-byte CycleCaptureRecordV2 records, never JSON/base64. Maximum bytes per batch: 312 + min(ring capacity,4096)*5392 <= 22085944. after is the last consumed sequence; zero starts at sequence 1 with explicit dropped count for overwritten history. Empty batches have first_sequence=0 and last_sequence=after+dropped. Invalid/future cursors return HTTP 409 with catalogue reasons. Streaming flush cadence is 20 ms (unverified off-cycle budget to amortize HTTP overhead while bounding batching latency); one complete batch per write/flush, including empty batches; HTTP intermediaries may reframe chunks. Slow writes terminate at 5 seconds. No session or authority required; transport authentication still applies. Incarnation changes close a stream; reconcile Describe before resetting the cursor. |
<!-- END SCHEMA RULES -->

## Complete schema field catalogue

`required` is the schema codec/response designation; operation admission uses the
capability table. Embedded types flatten their fields into the containing object,
not a nested property named Fence or Description. Optional fields can be omitted;
`omit_empty` records the generated encoder rule. JSON integers retain their full
signed/unsigned width; do not round uint64 identifiers through floating point.

<!-- BEGIN SCHEMA TYPES -->
### CollisionWatchdogStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `armed` | boolean / bool | true | false | True only while enabled, armed and receiving valid feedback; false after a trip. |
| `torque_cycles` | integer / uint32 | true | false | Consecutive torque breaches; equality resets. Frozen at trip. |
| `following_error_cycles` | integer / uint32 | true | false | Consecutive following-error breaches independent of torque. Frozen at trip. |
| `peak_torque_raw` | integer / uint32 | true | false | Peak absolute 0x6077 raw per-mille since arm; frozen at trip. |
| `peak_following_error_counts` | integer / uint64 | true | false | Peak absolute counts since arm; drive 0x60F4 when valid, otherwise prior wire target minus feedback with configured wrap handling. |
| `last_trip_quantity` | string / string | true | false | none, torque, following_error or torque_and_following_error; retained across explicit reset. |
| `trip_sustained_cycles` | integer / uint32 | true | false | Configured consecutive count reached at last trip, 2..1000. |
| `trip_count` | integer / uint64 | true | false | Monotonic per-axis trip sequence for the daemon incarnation. |
| `samples` | integer / uint64 | true | false | Fresh normal armed/enabled cycles accumulated since arm, even if the watchdog is disabled; peaks include pre-motion hold. Frozen after trip or disarm. |
| `last_trip_peak_torque_raw` | integer / uint32 | true | false | Peak absolute torque at last trip, retained across reset and rearm. |
| `last_trip_peak_following_error_counts` | integer / uint64 | true | false | Peak absolute following error at last trip, retained across reset and rearm. |

### BrakeHoldDiagnostic

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `observed_ns` | integer / uint64 | true | false | Daemon host time of the detecting feedback sample; 0 means no warning in this daemon incarnation. |
| `severity` | string / string | true | false | warning once any brake-hold warning was recorded, otherwise none. Never a fault severity. |
| `warning_sequence` | integer / uint64 | true | false | Monotonic per-axis warning count for the daemon incarnation; 0 means none. A change identifies a new warning. |
| `kind` | string / string | true | false | slip once same_direction_count reaches the native threshold (3, unverified) within one braked period, settle for any other warning, none without a warning. Neither is a fault. |
| `transition` | string / string | true | false | torque_on (detected while the drive was enabling/releasing), torque_off (closing, settling or held) or none. |
| `message` | string / string | true | false | Operator text: for settle, the drive delay to adjust (C05.13 release delay for torque_on, C05.10 hold delay for torque_off); for slip, an instruction to inspect the brake with the count and total displacement; empty when none. |
| `same_direction_count` | integer / uint32 | true | false | Consecutive same-direction motion warnings, including this one, since the braked period began at torque off. Enable, Arm, Home or frame change ends the period and a reversal restarts the run; 0 for a warning without motion evidence. |
| `same_direction_displacement_counts` | integer / int64 | true | false | Net raw displacement from the first anchor of that same-direction run (the torque-off anchor unless the run restarted) to the detecting sample. |
| `cause` | string / string | true | false | Native guard detection name, for example velocity_cap, transition_corridor or disabled_displacement; none when no warning. Not a refusal enum. |
| `phase` | string / string | true | false | Guard phase at detection (opening, opening_enabled, enabled, closing, settling, held) or legacy for profiles without a transition guard. |
| `anchor_counts` | integer / int32 | true | false | Raw feedback anchor the displacement was judged from. |
| `feedback_counts` | integer / int32 | true | false | Raw feedback at the detecting sample. |
| `displacement_counts` | integer / int64 | true | false | Detecting sample minus anchor, with configured wrap handling. |
| `velocity_counts_per_s` | integer / int32 | true | false | Drive velocity at the detecting sample; interpret only when velocity_valid. |
| `velocity_valid` | boolean / bool | true | false | True only when a fresh mapped velocity sample was observed at detection. |
| `peak_displacement_counts` | integer / int64 | true | false | Largest-magnitude displacement from anchor_counts observed until the guard re-anchored. |
| `peak_velocity_counts_per_s` | integer / int32 | true | false | Largest-magnitude valid velocity observed until the guard re-anchored. |
| `settled` | boolean / bool | true | false | True once the guard re-anchored (quiet applied rest, re-enable, Home or Arm request). Not physical brake proof. |
| `settled_displacement_counts` | integer / int64 | true | false | Re-anchor position minus anchor_counts; 0 when the frame changed or not yet settled. |
| `torque_off_ns` | integer / uint64 | true | false | Daemon host time the latest energized hold window closed (hold delay elapsed after the inhibit); 0 when none has closed. Published with or without a warning. |
| `torque_off_anchor_counts` | integer / int32 | true | false | Raw feedback at torque off: the anchor closing, settling and hold are judged from. |
| `energized_peak_displacement_counts` | integer / int64 | true | false | Largest-magnitude displacement from the last enabled anchor while the drive still held torque after the inhibit. Recorded servo settling, never judged or warned. |
| `energized_peak_velocity_counts_per_s` | integer / int32 | true | false | Largest-magnitude valid velocity in that energized window. Recorded, never judged. |
| `transition_caps_valid` | boolean / bool | true | false | True when the effective caps below were evaluated at detection. |
| `effective_velocity_cap_counts_per_s` | integer / uint32 | true | false | Velocity cap evaluated at detection; 0 unless transition_caps_valid. |
| `effective_displacement_cap_counts` | integer / uint32 | true | false | Displacement cap evaluated at detection; 0 unless transition_caps_valid. |
| `tolerance_counts` | integer / uint32 | true | false | Configured held displacement tolerance. |
| `hold_delay_ns` | integer / uint64 | true | false | Configured drive brake hold delay (C05.10) assumed by native timing. |
| `release_delay_ns` | integer / uint64 | true | false | Configured drive brake release delay (C05.13) assumed by native timing. |

### DriveSupervisorDiagnostic

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `observed_ns` | integer / uint64 | true | false | Daemon host time of the deciding feedback sample; 0 means no event in this daemon incarnation. |
| `severity` | string / string | true | false | arm_ended when the event ended an Arm attempt, warning for disarm motion, otherwise none. Never a fault severity; nothing needs reset or restart. |
| `event_sequence` | integer / uint64 | true | false | Monotonic per-axis supervisor event count for the daemon incarnation; 0 means none. A change identifies a new event. |
| `kind` | string / string | true | false | enable_timeout, release_timeout, left_operation_enabled, arm_cancelled, disarm_motion or none. Identical on every axis and independent of brake configuration. |
| `cause` | string / string | true | false | For arm_cancelled: frame_changed, context_lost, mode_changed, target_changed or opening_interrupted; otherwise none. |
| `message` | string / string | true | false | Operator text naming the J1-based joint and statusword, for example 'J4 did not reach Operation Enabled within 500 ms (statusword 0x0233). Arm ended; retry Arm.'; empty when none. |
| `statusword` | integer / uint32 | true | false | Drive statusword at the deciding sample. |
| `arm_ended` | boolean / bool | true | false | True when this event cleared Arm. The grant, generation and enable selection are retained so Arm can be retried at once. |
| `current` | boolean / bool | true | false | True for an arm_ended event until the next accepted Arm request; display it as the reason the last Arm attempt ended. |
| `elapsed_ns` | integer / uint64 | true | false | Time from the supervised start (Arm request, Operation Enabled or inhibit) to the event. |
| `bound_ns` | integer / uint64 | true | false | Timeout applied: the machine drive_supervision enable_timeout_ms for enable_timeout, the axis brake release delay plus release_slack_ms for release_timeout; 0 otherwise. |
| `displacement_counts` | integer / int64 | true | false | For disarm_motion: feedback minus the inhibit position, raw counts. |
| `bound_counts` | integer / uint32 | true | false | For disarm_motion: the machine drive_supervision disarm_motion_tolerance (one joint-unit value for every axis) floored to this axis's counts by gear. |

### Fence

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `session` | string / string | true | false | Current unguessable session token; acquire omits the fence. Tokens issued by the adapter are 64 lowercase hexadecimal characters. |
| `generation` | integer / uint64 | true | false | Nonzero uint64 grant generation. Motion requires exact equality; renew and stop accept an older nonzero generation not above the current one for the same session. |

### Binding

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `pair_id` | string / string | true | false | Exact configured nonempty pair ID. |
| `revision` | integer / uint64 | true | false | Nonzero uint64 equal to configured pair revision. |
| `configuration_sha256` | string / string | true | false | Exact configured native SHA-256 label. |
| `machine_sha256` | string / string | false | true | Machine-semantics digest; omitted by clients that only know the whole-artifact digest. |

### Point

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / int64 | true | false | Signed int64 nanoseconds from plan start; first point zero, later timestamps strictly increasing. |
| `position` | array / []float64 | true | false | Finite logical rad/m positions in described axis order, within native configured bounds. |
| `velocity` | array / []float64 | false | true | Optional finite logical rad/s or m/s velocities in described axis order; nil/empty omits feedforward. |
| `io_mask` | integer / uint32 | false | true | Sixteen-bit mask in configured output order; torch outputs additionally require explicit cell qualification. |
| `io_values` | integer / uint32 | false | true | Sixteen-bit output intents; every set bit must also be set in io_mask. Physical readback is independent. |
| `wait_timeout_ns` | integer / uint64 | false | true | Stationary input barrier metadata; zero timeout means no wait. Native admission verifies channel, value, stability and stationary interval. |
| `wait_stable_ns` | integer / uint64 | false | true | Stationary input barrier metadata; zero timeout means no wait. Native admission verifies channel, value, stability and stationary interval. |
| `wait_input` | integer / uint32 | false | true | Stationary input barrier metadata; zero timeout means no wait. Native admission verifies channel, value, stability and stationary interval. |
| `wait_value` | integer / uint32 | false | true | Stationary input barrier metadata; zero timeout means no wait. Native admission verifies channel, value, stability and stationary interval. |
| `monitor_mask` | integer / uint32 | false | true | Sixteen-bit scoped input requirement; values must be a subset of mask and input validity is checked each cycle. |
| `monitor_values` | integer / uint32 | false | true | Sixteen-bit scoped input requirement; values must be a subset of mask and input validity is checked each cycle. |

### Identity

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `plan_id` | string / string | true | false | Wire field; native admission remains authoritative. |
| `program_id` | string / string | true | false | Wire field; native admission remains authoritative. |
| `program_digest` | string / string | true | false | Wire field; native admission remains authoritative. |
| `trajectory_digest` | string / string | true | false | Wire field; native admission remains authoritative. |
| `source_digest` | string / string | true | false | Verified source trajectory digest; start_program must echo the prepared value exactly. |
| `normalised_digest` | string / string | true | false | Canonical segment-record digest including local clocks and process bits; start_program must echo the prepared value exactly. |
| `manifest_revision` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `plan_revision` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |

### Request

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `request_id` | string / *string | false | true | Optional, non-null string of 1..64 printable ASCII bytes (0x20..0x7e). |
| `schema` | string / string | true | false | Exactly rosie.rt-control.request.v1. |
| `operation` | string / string | true | false | Exact capability name; only POST /v1/control operations dispatch here. Unknown names are rejected. |
| `flattened Fence` | object / Fence | false | false | Wire field; native admission remains authoritative. |
| `label` | string / string | false | false | mark_telemetry requires 1..128 UTF-8 bytes; free-text dump label. |
| `controller` | string / string | false | false | Acquire requires 1..63 bytes; opaque controller name. |
| `binding` | object / Binding | false | false | Acquire requires exact equality with the configured Binding. |
| `axis_mask` | integer / uint32 | false | false | Uint32 configured axis group mask; interpretation is native operation-specific; reset_fault defaults zero to all axes. |
| `velocity` | array / []float64 | false | false | Finite logical velocities in described axis order for legacy jog; native vector and velocity-limit validation applies. |
| `timeout_ms` | integer / uint32 | false | false | Legacy jog requires an integer 1..250 milliseconds. |
| `handle` | integer / uint64 | false | false | Nonzero opaque uint64 handle for start/discard, owned by the current daemon incarnation and grant. |
| `points` | array / []Point | false | false | 2..250000 Point records; declared axis order, native limits and continuity apply. Large bodies require the envelope before this array. |
| `identity` | object / Identity | false | false | Exact prepared program Identity for start_program. |
| `jog_generation` | integer / uint64 | false | false | EndJog requires the exact current nonzero independent jog generation. |
| `source_sequence` | integer / uint64 | false | false | Accepted envelope field retained for compatibility; current /v1/control dispatch does not consume it. Independent updates use the binary lane. |
| `source_origin_host_ns` | integer / uint64 | false | false | BeginJog origin in the negotiated host monotonic clock, uint64 nanoseconds; native freshness validation applies. |
| `deadline_host_ns` | integer / uint64 | false | false | BeginJog producer deadline in host monotonic nanoseconds; after origin and never extended by admission or renewal. |
| `clock_incarnation` | string / string | false | false | BeginJog requires exact equality with GET /v1/jog/clock incarnation. |
| `requested_lease_ms` | integer / int | false | false | Acquire requested lease in whole ms, 1..10000; omitted or zero retains the LAN default capped by the cell. Adapter returns min(request, cell ceiling) in Grant.lease_ms. Clients require at least three measured round trips plus renewal cadence. |
| `value` | boolean / bool | false | false | io_test desired output state. ON requires disarmed axes, an exclusive control grant, and a fresh test-session sample. |
| `begin` | boolean / bool | false | false | Explicit operator ON edge for io_test. Renewals must be false and cannot restore expired or cleared output intents. |

### JogObservation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `Ticket` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConnectionId` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NativeGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `CapabilityId` | array / [16]uint8 | true | false | Wire field; native admission remains authoritative. |
| `AppGrantGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConfigurationEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `HomeEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ClockMappingGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `JogGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `SourceSequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `InputDeadlineHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NowHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ObservedSourceSequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ObservedOriginHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `FirstObservedHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ControlReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `ControlAccepted` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `UpdateReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `StateReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `AxisMask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Open` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `HasInput` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `VelocityScalePpm` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |

### CommandResult

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `Sequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Handle` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Generation` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Operation` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Result` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Reason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `AxisMask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |

### Response

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `native_jog_result` | object / *JogObservation | false | true | Wire field; native admission remains authoritative. |
| `native_result` | object / *CommandResult | false | true | Wire field; native admission remains authoritative. |
| `schema` | string / string | true | false | Wire field; native admission remains authoritative. |
| `operation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `sequence` | integer / uint64 | false | true | Exact uint64 integer; no floating-point conversion. |
| `handle` | integer / uint64 | false | true | Exact uint64 integer; no floating-point conversion. |
| `data` | any / json | false | true | Wire field; native admission remains authoritative. |
| `error` | string / string | false | true | Wire field; native admission remains authoritative. |

### Grant

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `stopping` | boolean / bool | true | false | True means renewal keeps the lease alive while Stop is in flight; it grants no movement permission. |
| `deadline_host_ns` | integer / uint64 | true | false | Uint64 absolute lease deadline in host monotonic nanoseconds. |
| `session` | string / string | true | false | 64-character unpredictable lowercase hexadecimal token while owned; empty after release/expiry. |
| `generation` | integer / uint64 | true | false | Monotonically increasing uint64 application generation, fenced by Stop and acquisition. |
| `controller` | string / string | true | false | Wire field; native admission remains authoritative. |
| `binding` | object / Binding | true | false | Wire field; native admission remains authoritative. |
| `lease_ms` | integer / int | true | false | Effective negotiated lease in whole milliseconds: min(client requested lease, compiled cell ceiling); zero/omitted request retains 500 ms capped by the cell. Renew preserves this value. Longer leases increase worst-case unattended-stop delay after link loss. Unverified absolute bound: 10000 ms. |

### HandleRecord

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `handle` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `state` | string / HandleState | true | false | One of the handle_states labels; terminal states never regain permission. |
| `generation` | integer / uint64 | true | false | Application grant generation that prepared this handle. |
| `execution_generation` | integer / uint64 | true | false | Count of acknowledged native Starts when this handle started; zero before Start. |
| `native_sequence` | integer / uint64 | true | false | Correlated native Start sequence; zero before Start. |
| `identity` | object / Identity | false | false | Immutable raw trajectory identity supplied at Prepare and checked at Start; echoed unchanged through terminal observation. |

### Execution

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `handles` | array / []HandleRecord | true | false | Wire field; native admission remains authoritative. |
| `generation` | integer / uint64 | true | false | Count of acknowledged native Starts; distinct from native control generation. |
| `state` | string / string | true | false | Application execution state: empty before observation, prepared, executing, completed, faulted, cancelled, discarded or released. |
| `identity` | object / Identity | true | false | Wire field; native admission remains authoritative. |
| `segment` | integer / int | true | false | Current zero-based program segment when a program is observed. |
| `native_sequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `error` | string / string | false | true | Wire field; native admission remains authoritative. |

### GrantObservation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `Ticket` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConnectionId` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NativeGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `AppGrantGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `CapabilityId` | array / [16]uint8 | true | false | Wire field; native admission remains authoritative. |
| `DeadlineHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NowHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConfigurationEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `HomeEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `StopAckId` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `GrantReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `GrantActive` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `AxisMask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Reserved` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Reserved1` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Reserved2` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Reserved3` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |

### FaultRecovery

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `bit` | integer / uint32 | true | false | Native execution-fault bit index. |
| `name` | string / string | true | false | Native fault name for that bit. |
| `axis_mask` | integer / uint32 | true | false | Affected axis evidence mask. |
| `recovery` | string / string | true | false | Native recovery-policy label for the fault bit. |
| `outcome` | string / string | true | false | Native per-bit outcome: persists, cleared or rehome_required. |
| `rehome_axis_mask` | integer / uint32 | true | false | Axes requiring qualified re-home: the current full-cell Reset candidate for faults, the actual past decision for outcomes. |
| `reset_preserves_home` | boolean / bool | false | false | Current native full-cell Reset candidate preserves all existing qualified Home references with fresh full-axis feedback, healthy verified bus and inhibited execution. Advisory only, explicit Reset required; absent or false grants no recovery permission. Never reported on historical outcomes. |

### RecoveryStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `home_valid_mask` | integer / uint32 | true | false | Native per-axis home evidence remaining after observed recovery. |
| `faults` | array / []FaultRecovery | true | false | Currently latched fault bits with axis masks and recovery policy. |
| `reset_sequence` | integer / uint64 | true | false | Exact native command sequence of the observed reset, zero before any reset; response sequence must match for reset_fault. |
| `reason` | string / string | true | false | Native recovery summary: empty when no fault persists, otherwise fault_persists; FaultResetRecovery refuses a persistent fault. |
| `outcomes` | array / []FaultRecovery | true | false | Per-bit outcomes of the correlated reset; never inferred from elapsed time. |

### IndependentAnchorSource

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `valid` | boolean / bool | true | false | Native acquisition validity; never motion permission. |
| `encoder_counts` | integer / int64 | true | false | Independent signed raw encoder count. |
| `completed_ns` | integer / uint64 | true | false | Host-monotonic acquisition completion timestamp in nanoseconds. |
| `maximum_age_ns` | integer / uint64 | true | false | Configured acquisition freshness bound in nanoseconds. |

### AxisLoad

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `window_ms` | integer / uint64 | true | false | Length of the summary window, ms: the last 1000 ms of telemetry records ending at the newest one (unverified display choice). Diagnostics only; never a motion or safety input. |
| `torque_samples` | integer / uint32 | true | false | Cycle records in the window with fresh, verified torque feedback; 0 means no torque value below is meaningful. |
| `torque_rms_raw` | number / float64 | true | false | Root mean square of drive 0x6077 over torque_samples, raw per-mille of rated torque. |
| `torque_peak_raw` | integer / uint32 | true | false | Peak absolute drive 0x6077 over torque_samples, raw per-mille of rated torque. |
| `following_error_samples` | integer / uint32 | true | false | Cycle records in the window with valid drive 0x60F4 following error; 0 means the peak below is not meaningful. |
| `following_error_peak_counts` | integer / uint64 | true | false | Peak absolute drive 0x60F4 following error over following_error_samples, drive position counts. |

### AxisMonitor

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `key` | string / string | true | false | The drive profile's monitor key, for example heatsink_temperature. |
| `unit` | string / string | true | false | The drive profile's unit token for value, for example degC. |
| `value` | number / *float64 | false | true | Latest successful mailbox read, raw / 10^decimals from the drive profile; absent until the first successful read. A failed read keeps the last value and its age grows. |
| `age_ms` | integer / *uint64 | false | true | Milliseconds since value was read on the daemon host; absent with value. |

### AxisStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `drive_alarm` | string / string | true | false | A6-EC display code from the 0x603F/0x203F low-half pair; unresolved bus candidates joined by \|; empty when clear, unmapped, or another drive family. |
| `drive_alarm_text` | string / string | true | false | Manual meaning corresponding to drive_alarm; multiple candidates joined by \| in matching order. |
| `drive_alarm_aux_code` | integer / uint32 | true | false | Full raw 0x203F auxiliary SDO word; a two-byte reply occupies the low half. Retained for the current nonzero 0x603F event and configuration epoch, zero when invalid or cleared. Separate from manufacturer_error_code, which carries the cyclic manufacturer_err PDO. |
| `drive_alarm_aux_valid` | boolean / bool | true | false | True only for an admitted auxiliary reply matching the current nonzero 0x603F event and configuration epoch; false when missing, stale, or cleared, preventing zero from being mistaken for a valid reply. |
| `logical_position` | number / float64 | true | false | Wire field; native publication is authoritative. |
| `logical_valid` | boolean / bool | true | false | Wire field; native publication is authoritative. |
| `coordinate_counts` | integer / int64 | true | false | Wire field; native publication is authoritative. |
| `absolute_source_counts` | integer / int32 | true | false | Wire field; native publication is authoritative. |
| `independent_anchor_source` | object / *IndependentAnchorSource | false | true | Fresh independent raw acquisition; does not grant a command-frame binding or Home. |
| `coordinate_valid` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `coordinate_source_valid` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `pdo_fresh` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `native_home_position_offset` | integer / int32 | true | false | Wire field; native publication is authoritative. |
| `observed_revision_no` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `revision_readback_valid` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `serial_no` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `serial_readback_valid` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `anchor_tolerance_counts` | integer / uint64 | true | false | Wire field; native publication is authoritative. |
| `coordinate_reason` | integer / uint32 | true | false | Wire field; native publication is authoritative. |
| `coordinate_identity_sha256` | string / string | true | false | Canonical P0-12 coordinate digest; empty means persisted anchors are unavailable. |
| `coordinate_identity` | string / string | true | false | Canonical P0-12 per-axis fields; empty means persisted anchors are unavailable. |
| `anchor_identity_changed_field` | string / string | true | false | Differing identity component for anchor_identity_mismatch, or empty. |
| `anchor_reason` | string / string | true | false | Wire field; native publication is authoritative. |
| `restored_anchor_home_epoch` | integer / uint64 | true | false | Wire field; native publication is authoritative. |
| `pos_counts` | integer / int32 | true | false | Exact int32 integer; no floating-point conversion. |
| `statusword` | integer / uint16 | true | false | Exact uint16 integer; no floating-point conversion. |
| `error_code` | integer / uint16 | true | false | Exact uint16 integer; no floating-point conversion. |
| `vendor_id` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `product_code` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `slave_position` | integer / uint16 | true | false | Exact uint16 integer; no floating-point conversion. |
| `time_ns` | integer / uint64 | true | false | Native daemon publication; never adapter-estimated motion. |
| `logical_target` | number / float64 | true | false | Native daemon publication; never adapter-estimated motion. |
| `readiness` | string / string | true | false | Native daemon publication; never adapter-estimated motion. |
| `brake_state` | integer / uint32 | true | false | Native daemon publication; never adapter-estimated motion. |
| `home_valid` | boolean / bool | true | false | Native daemon publication; never adapter-estimated motion. |
| `logical_velocity` | number / *float64 | false | true | Native daemon publication; never adapter-estimated motion. |
| `velocity_actual_counts_per_s` | integer / int32 | true | false | Drive 0x606C signed reference counts/s in the position frame, sampled each cycle. Diagnostics only; never a speed-safety certification or an execution guard. |
| `following_error_counts` | integer / int32 | true | false | Drive 0x60F4 signed reference counts in the position frame; preferred input to the configured collision watchdog. |
| `velocity_actual_valid` | boolean / bool | true | false | True only when velocity_actual is mapped and this cycle has fresh PDO feedback. An absent signal is zero with false validity. |
| `following_error_valid` | boolean / bool | true | false | True only when following_error is mapped and this cycle has fresh PDO feedback. An absent signal is zero with false validity. |
| `max_abs_velocity_actual_counts_per_s` | integer / uint32 | true | false | Maximum absolute valid drive velocity in counts/s since the last accepted Arm(true); zero before Arm. Retained through Stop/disarm; INT32_MIN has magnitude 2147483648. |
| `max_abs_following_error_counts` | integer / uint32 | true | false | Maximum absolute valid drive following error in counts since the last accepted Arm(true); zero before Arm. Retained through Stop/disarm; INT32_MIN has magnitude 2147483648. |
| `di_bits` | integer / uint32 | true | false | Current 0x60FD DI logic bits; interpret only when di_valid. |
| `di_valid` | boolean / bool | true | false | True only for a fresh PDO and verified digital-input mapping. |
| `external_enable_active` | boolean / bool | true | false | A6-EC S-ON input active; interpret only when external_enable_valid. Diagnostics only; does not prove STO or torque state. |
| `external_enable_valid` | boolean / bool | true | false | True only with valid DI feedback and exactly one explicitly configured, verified A6-EC S-ON DI function. |
| `torque_raw` | integer / int32 | true | false | Drive 0x6077 raw per-mille of rated torque; use only with fresh PDO feedback. |
| `collision_watchdog` | object / CollisionWatchdogStatus | true | false | Native cycle evaluation and latched trip evidence for this axis. |
| `load` | object / AxisLoad | true | false | The last second of this axis's torque and following error, summarised by the metrics thread from the telemetry ring (docs/decisions/0001). |
| `monitor` | array / []AxisMonitor | true | false | The drive profile's monitor objects for this axis, in profile order; empty when the profile lists none. |
| `brake_hold_diagnostic` | object / BrakeHoldDiagnostic | true | false | Latest native brake-hold warning for this axis. A warning never latches a fault or requires reset, restart, re-Home or refusing Arm. |
| `drive_supervisor` | object / DriveSupervisorDiagnostic | true | false | Latest drive-state supervisor event for this axis: an ended Arm attempt or a disarm-motion warning. Never a latched fault; Arm can be retried. |
| `calibration_valid` | boolean / *bool | false | true | Persistent calibration applicability; null/absent means legacy unknown. |
| `position_state` | string / string | false | true | Native state: unverified, recovering, trusted or lost; empty means legacy unknown. |
| `position_reason` | string / string | false | true | Native CoordinateReason name; none means no coordinate refusal; absent means legacy unknown. |
| `audit_status` | string / string | false | true | Last stationary audit: pending, verified or unavailable; never motion authority. |
| `audit_reason` | string / string | false | true | Native SampleInvalidity name; none means no reported audit failure; absent means legacy unknown. |
| `audit_last_verified_ns` | integer / uint64 | false | true | Monotonic acquisition completion of last accepted independent evidence; zero means never. |
| `anchor_delta_restore` | object / *AnchorDeltaRestoreStatus | false | true | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |

### NativeStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `daemon_incarnation` | string / string | true | false | Wire field; native publication is authoritative. |
| `rt_cpu` | integer / uint32 | true | false | Configured RT CPU index as an exact uint32 integer. |
| `housekeeping_cpus` | string / string | true | false | Effective Linux CPU-list mask for housekeeping workers, excluding the RT CPU. |
| `affinity_applied` | object / map[string]bool | true | false | Worker name to affinity-application success; false is not qualified placement and null means no observations. |
| `recovery` | object / *RecoveryStatus | true | false | Wire field; native admission remains authoritative. |
| `time_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `backend` | string / string | true | false | Wire field; native admission remains authoritative. |
| `configuration_sha256` | string / string | true | false | Wire field; native admission remains authoritative. |
| `control_generation` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `lease_valid` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `config_verified_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `home_valid_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `home_epoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `commissioning_phase` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `safety_fault_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `execution_fault_reasons` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `last_bus_failure_operation` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `last_bus_failure_code` | integer / int64 | true | false | Exact int64 integer; no floating-point conversion. |
| `armed` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `axis_enable_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `native_home_active_axis_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `axes` | array / []AxisStatus | true | false | Wire field; native admission remains authoritative. |
| `configuration_epoch` | integer / uint64 | true | false | Native daemon publication; never adapter-estimated motion. |
| `buffer_health` | object / BufferHealth | true | false | Native daemon publication; never adapter-estimated motion. |
| `plan_cursor` | object / PlanCursor | true | false | Native daemon publication; never adapter-estimated motion. |
| `bus` | object / rawjson | true | false | Native bus policy and held_slaves with retained, observed and expected declared identities, presence, AL state and disposition; unreadable identity fields are null. Dispositions: held for an undeclared device allowed by hold; declared_unused for a matching declaration; refused_unknown for an undeclared device under refuse; refused_identity for a declared identity mismatch; identity_unreadable for an unavailable SII identity; refused_missing for a missing required declaration; identity_changed for a post-admission identity change under hold; disappeared for a missing unused device under hold; refused_state for a held device outside PREOP or INIT, or a configured terminal outside OP after activation. Refuse continuously requires declared identities and presence; post-admission refusal latches a readiness fault and inhibits outputs before submission, requiring public reset_fault (FaultReset), reacquisition of the application session, then explicit Enable and Arm after restoring the device and verified readiness. Enable before reset is rejected with native not_ready (reason 2); reset clears the readiness fault (reason 128) and safety mask while keeping outputs inhibited and retiring the session. Lost Home evidence requires separate qualified recovery before motion. Hold reports post-admission unused-device identity or presence changes and continues axes while axis readiness remains valid. Admission reason and reason_position identify identity_unreadable, unknown_slave, declared_identity_mismatch, declared_slave_missing, unsafe_held_state or capacity_exceeded; none means no observed refusal. No output capability is granted to held devices. A declared cell I/O terminal has disposition io_terminal and contributes one required responding slave. Its declared identity and presence remain mandatory under both hold and refuse; after activation it must be OP or admission reports io_terminal_not_operational. Held devices alone receive no PDOs or output capability. |
| `io` | object / IoState | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |

### MotionState

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `mode` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `state` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `trajectory_id` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `point_index` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `queue_depth` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `last_event` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `done` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `command_sequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `time_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |

### ProcessStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `daemon_incarnation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `adapter_incarnation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `grant` | object / *GrantObservation | true | false | Wire field; native admission remains authoritative. |
| `jog` | object / *JogObservation | true | false | Wire field; native admission remains authoritative. |
| `jog_ingress` | object / JogIngressObservation | true | false | Adapter-lifetime ingress outcomes across datagram, WSS and UpdateJog; counters and latest refusal match GET /v1/jog/ingress. |
| `core` | object / NativeStatus | true | false | Wire field; native admission remains authoritative. |
| `motion` | object / MotionState | true | false | Wire field; native admission remains authoritative. |
| `execution` | object / Execution | true | false | Wire field; native admission remains authoritative. |
| `time_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `axes` | array / []LogicalAxisStatus | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `generations` | object / StatusGenerations | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `plan_cursor` | object / PlanCursor | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `buffer_health` | object / BufferHealth | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `adapter` | object / AdapterStatus | true | false | Adapter runtime observations; not native motion state. |

### ProcessMarker

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / int64 | true | false | Exact int64 integer; no floating-point conversion. |
| `segment` | integer / int | true | false | Exact int integer; no floating-point conversion. |
| `sample` | integer / int | true | false | Exact int integer; no floating-point conversion. |
| `action` | string / string | true | false | Wire field; native admission remains authoritative. |
| `value` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `timeout_ms` | integer / uint32 | false | true | Stationary input wait in milliseconds; zero timeout and empty monitor mean output marker. |
| `stable_ms` | integer / uint32 | false | true | Stationary input wait in milliseconds; zero timeout and empty monitor mean output marker. |
| `monitor` | string / string | false | true | Empty for output/wait; start or end for a paired input monitor scope. |

### Program

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `process_markers` | array / []ProcessMarker | true | false | Wire field; native admission remains authoritative. |
| `identity` | object / Identity | true | false | Wire field; native admission remains authoritative. |
| `requires_process_io` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `segments` | integer / int | true | false | Exact int integer; no floating-point conversion. |
| `samples` | integer / int | true | false | Exact int integer; no floating-point conversion. |
| `normalised_samples` | integer / int | true | false | Exact execution sample count after coincident segment endpoints are shared. |
| `axis_mask` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |

### ProgramLimitData

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `limit_violation` | object / ProgramLimitViolation | true | false | Logical joint-limit diagnostic for native_limit_exceeded or native_segment_rate_exceeded; supplements the unchanged refusal reason and is not a native command receipt. |

### ProgramLimitViolation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `kind` | string / string | true | false | Diagnostic quantity: position or velocity. |
| `segment` | integer / int | true | false | Zero-based source segment index, matching the refusal detail. |
| `sample` | integer / int | true | false | Zero-based source sample index, matching the refusal detail. |
| `axis` | string / string | true | false | Canonical joint identifier reported by admission, for example J6. |
| `value` | number / float64 | true | false | Finite logical joint position or peak velocity in unit; command-count direction and Home offset have been removed. |
| `limit` | number / float64 | true | false | Finite logical admission bound in unit; invalid numeric details are omitted as a whole without discarding the refusal. |
| `unit` | string / string | true | false | rad for position; rad/s for velocity. |

### JogIngressObservation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `source_sequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `reason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `now_host_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `refused` | integer / uint64 | true | false | Cumulative refused ingress count for this adapter process; accepted frames do not increment it. |
| `refused_by_reason` | array / [15]uint64 | true | false | Cumulative refusal counts indexed by protocol/control.json jog reason 0..14; index 0 remains zero. Unverified fixed capacity: 15 reason slots. |
| `latest_refusal` | object / JogIngressRefusal | true | false | Most recent refusal across all ingress transports; zero before any refusal and preserved through acceptance and grant changes. |

### JogIngressRefusal

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `source_sequence` | integer / uint64 | true | false | Refused source sequence; zero when no complete input frame is available. |
| `reason` | integer / uint32 | true | false | Typed native jog reason; zero only before the first refusal. |
| `now_host_ns` | integer / uint64 | true | false | Host monotonic ns when the adapter recorded the refusal; not an RT application timestamp. |

### AxisDescription

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `completion_tolerance` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `feedback_fields` | array / []string | true | false | Wire field; native admission remains authoritative. |
| `index` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `id` | string / string | true | false | Wire field; native admission remains authoritative. |
| `position_unit` | string / string | true | false | Wire field; native admission remains authoritative. |
| `counts_per_unit` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `sign` | integer / int | true | false | Exact int integer; no floating-point conversion. |
| `velocity_command_mapped` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `native_home` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `require_home` | boolean / bool | true | false | Wire field; native admission remains authoritative. |
| `min_position` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `max_position` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `max_velocity` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `jog_acceleration` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `max_target_lead` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `following_error` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `following_error_timeout_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `completion_timeout_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `interpolation` | string / string | true | false | hermite_position_with_velocity, linear_without: cubic Hermite q when both knots supply qd, otherwise linear q; configured shortest-step command wrapping applies. |
| `feedforward` | string / string | true | false | qd_optional: both supplied knot velocities shape Hermite position; feedforward is its derivative. If either knot lacks qd, feedforward is the linear segment slope. Endpoint hold velocity is zero. |
| `checks` | object / rawjson | true | false | Object with exactly position: declared, velocity: declared, acceleration: declared or not_declared, jerk: unsupported. Prepare checks position range and supplied qd/segment velocity. With max_acceleration, check consecutive qd differences per segment or second q differences across segment midpoints when qd is absent; mixed qd presence rejects. Without the profile field acceleration is not_declared. Sampled derivatives give no continuous acceleration guarantee at linear corners or endpoint hold. Jerk is never checked. |
| `endpoint` | string / string | true | false | hold_last_sample_then_settle: hold final q with zero velocity; fresh ready feedback within completion_tolerance by completion_timeout_ns, as evaluated by completion.hpp. |
| `max_acceleration` | number / *float64 | false | true | Positive finite profile bound in position units per s^2; absent only when acceleration is not_declared. |
| `brake_override_reason` | string / string | false | true | Nonempty reason for an explicit bench brake override (effective present=false); absent without an override. Does not claim the physical brake is absent or qualified. |
| `reference_verified_delta_restore` | object / *ReferenceVerifiedDeltaRestorePolicy | false | true | Optional explicit operation admission, separate from immutable coordinate identity; absent disables verified-delta restore. |

### NativeDescription

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `backend` | string / string | true | false | Wire field; native admission remains authoritative. |
| `schema` | string / string | true | false | Wire field; native admission remains authoritative. |
| `protocol_major` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `protocol_minor` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `configuration_sha256` | string / string | true | false | Wire field; native admission remains authoritative. |
| `machine_sha256` | string / string | true | false | Canonical machine-semantics digest recomputed by the daemon at startup. |
| `deployment_sha256` | string / string | true | false | Deployment identity digest (host, NIC, CPUs, sockets, pair, users). |
| `max_trajectory_points` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `resident_plans` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `cycle_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `interpolation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `stop` | string / string | true | false | Wire field; native admission remains authoritative. |
| `axes` | array / []AxisDescription | true | false | Wire field; native admission remains authoritative. |
| `bus` | object / rawjson | true | false | Current native bus policy and held_slaves inventory; the machine digest binds configured policy and declarations, not discovered devices. A declared cell I/O terminal has disposition io_terminal and contributes one required responding slave. Its declared identity and presence remain mandatory under both hold and refuse; after activation it must be OP or admission reports io_terminal_not_operational. Held devices alone receive no PDOs or output capability. |
| `max_grant_lease_ns` | integer / uint64 | true | false | Compiled cell lease ceiling in ns; a longer effective lease increases unattended-stop delay after link loss. |
| `max_jog_input_age_ns` | integer / uint64 | true | false | Compiled cell capture-age and lifetime ceiling in ns; longer ages prolong stale velocity application before the existing expiry ramp. |
| `io` | object / rawjson | false | true | Configured cell I/O terminal identity, torch_qualified=false, named input classes and polarities, and output classes, OFF safe states, expiry_ns budgets and independent readback wiring. Omitted when no terminal is configured; this policy is separate from NativeStatus.io observations. |
| `path_jog` | object / *PathJogCapability | false | true | Path jog lane (protocol 1.14) limits; present only when the machine configures jog_jerk, which bounds the path generator. Absent means path holds must be refused, never replaced by velocity jog. |

### PathJogCapability

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `max_samples` | integer / uint32 | true | false | Native path ring capacity in samples (PATH_JOG_MAX_SAMPLES). |
| `max_chunk` | integer / uint32 | true | false | Samples per path_samples chunk (PATH_JOG_MAX_CHUNK). |
| `max_sample_step_s` | number / float64 | true | false | A sample step above axis max velocity times this many seconds is refused (PATH_JOG_MAX_SAMPLE_STEP_NS, unverified). |

### PathObservation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `ConnId` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NativeGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `CapabilityId` | array / [16]uint8 | true | false | Wire field; native admission remains authoritative. |
| `AppGrantGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConfigEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `HomeEpoch` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `JogGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ClockMappingGeneration` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ChunkSequenceSeen` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ChunkResult` | integer / uint32 | true | false | Jog reason of the chunk named by ChunkSequenceSeen; 0 accepted. |
| `Flags` | integer / uint32 | true | false | Bit 0 open, 1 stopping, 2 ended, 3 has_speed, 4 fault (protocol/control.json path_observation). |
| `Received` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `PlayableEndIndex` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `ConsumedIndex` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `PathS` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `PathSpeed` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `PathAcceleration` | number / float64 | true | false | Wire field; native admission remains authoritative. |
| `Underruns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `SpeedSequence` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `SpeedDeadlineHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `NowHostNs` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `Governing` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `StopReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `EndReason` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |
| `Reserved` | integer / uint32 | true | false | Exact uint32 integer; no floating-point conversion. |

### PathJogStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `observation` | object / PathObservation | true | false | Latest native path_observation read from the fast-control region, not from the 10 Hz status file. |
| `generation` | integer / uint64 | true | false | Jog generation of the adapter path session; zero when no path session is open. |
| `accepted` | integer / uint64 | true | false | Highest chunk_sequence native accepted for that session. |
| `refused` | integer / uint64 | true | false | chunk_sequence of the latest refused chunk for that session; zero before any refusal. |
| `reason` | integer / uint32 | true | false | Jog reason of that refusal (protocol/control.json jog_reasons); a refusal other than path_capacity ends the session along the path. |
| `queued` | integer / uint32 | true | false | Chunks admitted by the adapter and not yet acknowledged by native; unverified bound 64. |

### LocalJogClock

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `domain` | string / string | true | false | Wire field; native admission remains authoritative. |
| `incarnation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `mapping_generation` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |
| `now_host_ns` | integer / uint64 | true | false | Exact uint64 integer; no floating-point conversion. |

### Description

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `flattened NativeDescription` | object / NativeDescription | true | false | Flattened native description fields. |
| `contract_version` | integer / uint32 | true | false | Exactly 1. |
| `capabilities_digest` | string / string | true | false | Lowercase SHA-256 of the exact committed UTF-8 LF schema file bytes, including its final newline; no self-referential digest field. |
| `capabilities` | array / []CapabilityInfo | true | false | Every target capability with its implementation state and transport. |
| `control_idle_timeout_ns` | integer / uint64 | false | true | Compiled link idle timeout in ns; clients keep reserved connections alive at one third of this bound, independently of grants. Unverified LAN and internet policy: 90000000000 ns. An omitted legacy field uses 30000000000 ns. |
| `program` | object / *Program | false | true | Detached prepared program metadata including both identity digests; absent when no program is prepared. |
| `robot` | object / *RobotDescription | false | false | Null when no robot definition is bound or no compiled artefact is loaded; never guessed from axis count. |
| `drives` | array / []DriveDescription | true | false | Per-axis compiled drive configuration and current verified bring-up identity; empty without a compiled artefact. |

### CapabilityInfo

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `name` | string / string | true | false | Canonical capability name. |
| `state` | string / string | true | false | implemented, interim, test_only or unimplemented. test_only operations are retained for oracle ports and fixtures; consumers use implemented operations. |
| `transport` | string / string | true | false | Actual ingress; unimplemented has no transport. |

### RawResponse

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `native_jog_result` | object / *JogObservation | false | true | Wire field; native admission remains authoritative. |
| `native_result` | object / *CommandResult | false | true | Wire field; native admission remains authoritative. |
| `schema` | string / string | true | false | Wire field; native admission remains authoritative. |
| `operation` | string / string | true | false | Wire field; native admission remains authoritative. |
| `sequence` | integer / uint64 | false | true | Exact uint64 integer; no floating-point conversion. |
| `handle` | integer / uint64 | false | true | Exact uint64 integer; no floating-point conversion. |
| `data` | any / rawjson | false | true | Wire field; native admission remains authoritative. |
| `error` | string / string | false | true | Wire field; native admission remains authoritative. |

### LogicalAxisStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `drive_alarm` | string / string | true | false | A6-EC display code from the 0x603F/0x203F low-half pair; unresolved bus candidates joined by \|; empty when clear, unmapped, or another drive family. |
| `drive_alarm_text` | string / string | true | false | Manual meaning corresponding to drive_alarm; multiple candidates joined by \| in matching order. |
| `drive_alarm_aux_code` | integer / uint32 | true | false | Full raw 0x203F auxiliary SDO word; a two-byte reply occupies the low half. Retained for the current nonzero 0x603F event and configuration epoch, zero when invalid or cleared. Separate from manufacturer_error_code, which carries the cyclic manufacturer_err PDO. |
| `drive_alarm_aux_valid` | boolean / bool | true | false | True only for an admitted auxiliary reply matching the current nonzero 0x603F event and configuration epoch; false when missing, stale, or cleared, preventing zero from being mistaken for a valid reply. |
| `time_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `logical_position` | number / float64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `logical_valid` | boolean / bool | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `logical_target` | number / float64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `logical_velocity` | number / *float64 | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `readiness` | string / string | true | false | ready or first failing axis gate in this order: faulted, mode_mismatch, home_required, coordinate_invalid, not_enabled, not_operation_enabled, brake_wait; group authority still applies. |
| `position_counts` | integer / int32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `statusword` | integer / uint16 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `brake_state` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `home_valid` | boolean / bool | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `velocity_actual_counts_per_s` | integer / int32 | true | false | Drive 0x606C signed reference counts/s in the position frame, sampled each cycle. Diagnostics only; never a speed-safety certification or an execution guard. |
| `following_error_counts` | integer / int32 | true | false | Drive 0x60F4 signed reference counts in the position frame; preferred input to the configured collision watchdog. |
| `velocity_actual_valid` | boolean / bool | true | false | True only when velocity_actual is mapped and this cycle has fresh PDO feedback. An absent signal is zero with false validity. |
| `following_error_valid` | boolean / bool | true | false | True only when following_error is mapped and this cycle has fresh PDO feedback. An absent signal is zero with false validity. |
| `max_abs_velocity_actual_counts_per_s` | integer / uint32 | true | false | Maximum absolute valid drive velocity in counts/s since the last accepted Arm(true); zero before Arm. Retained through Stop/disarm; INT32_MIN has magnitude 2147483648. |
| `max_abs_following_error_counts` | integer / uint32 | true | false | Maximum absolute valid drive following error in counts since the last accepted Arm(true); zero before Arm. Retained through Stop/disarm; INT32_MIN has magnitude 2147483648. |
| `di_bits` | integer / uint32 | true | false | Current 0x60FD DI logic bits; interpret only when di_valid. |
| `di_valid` | boolean / bool | true | false | True only for a fresh PDO and verified digital-input mapping. |
| `external_enable_active` | boolean / bool | true | false | A6-EC S-ON input active; interpret only when external_enable_valid. Diagnostics only; does not prove STO or torque state. |
| `external_enable_valid` | boolean / bool | true | false | True only with valid DI feedback and exactly one explicitly configured, verified A6-EC S-ON DI function. |
| `torque_raw` | integer / int32 | true | false | Drive 0x6077 raw per-mille of rated torque; use only with fresh PDO feedback. |
| `collision_watchdog` | object / CollisionWatchdogStatus | true | false | Native cycle evaluation and latched trip evidence for this axis. |
| `brake_hold_diagnostic` | object / BrakeHoldDiagnostic | true | false | Latest native brake-hold warning for this axis. A warning never latches a fault or requires reset, restart, re-Home or refusing Arm. |
| `drive_supervisor` | object / DriveSupervisorDiagnostic | true | false | Latest drive-state supervisor event for this axis: an ended Arm attempt or a disarm-motion warning. Never a latched fault; Arm can be retried. |
| `calibration_valid` | boolean / *bool | false | true | Persistent calibration applicability; null/absent means legacy unknown. |
| `position_state` | string / string | false | true | Native state: unverified, recovering, trusted or lost; empty means legacy unknown. |
| `position_reason` | string / string | false | true | Native CoordinateReason name; none means no coordinate refusal; absent means legacy unknown. |
| `audit_status` | string / string | false | true | Last stationary audit: pending, verified or unavailable; never motion authority. |
| `audit_reason` | string / string | false | true | Native SampleInvalidity name; none means no reported audit failure; absent means legacy unknown. |
| `audit_last_verified_ns` | integer / uint64 | false | true | Monotonic acquisition completion of last accepted independent evidence; zero means never. |
| `anchor_delta_restore` | object / *AnchorDeltaRestoreStatus | false | true | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |

### StatusGenerations

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `native` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `configuration_epoch` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `home_epoch` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `execution` | integer / uint64 | true | false | Adapter handle execution generation; correlated native Start acknowledgement. |
| `jog` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `grant` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `grant_time_ns` | integer / uint64 | true | false | Daemon fast-grant publication timestamp; zero if grant observation is unavailable. Independent of metrics time_ns. |
| `jog_time_ns` | integer / uint64 | true | false | Daemon jog publication timestamp; zero if jog observation is unavailable. Independent of metrics time_ns. |

### PlanCursor

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `active_handle` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `sample_index` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `native_clock_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### BufferHealth

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `slots_free` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `staging_in_progress` | boolean / bool | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `ring_drops` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `native_slot_occupancy` | object / map[string]uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### AdapterStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `heap_inuse` | integer / uint64 | true | false | Go runtime bytes at request observation. |
| `sys` | integer / uint64 | true | false | Go runtime bytes at request observation. |
| `goroutines` | integer / uint32 | true | false | Go runtime goroutine count at request observation. |
| `open_preparation_slots` | integer / uint32 | true | false | Available adapter preparation admissions, from the two atomic reservations. |

### GrantEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `generation` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `stopping` | boolean / bool | true | false | True for a local keepalive while Stop drains; does not confer native authority. |

### HandleTransitionEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `handle` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `from` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `to` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `execution_generation` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `native_sequence` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `identity` | object / Identity | false | false | Immutable raw trajectory identity supplied at Prepare and checked at Start; echoed unchanged through terminal observation. |
| `generation` | integer / uint64 | false | false | Application grant generation that owns this immutable preparation. |

### ExecutionEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `handle` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `generation` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `native_sequence` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `fault_bits` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `recovery` | array / []FaultRecovery | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `identity` | object / Identity | false | false | Immutable raw trajectory identity supplied at Prepare and checked at Start; echoed unchanged through terminal observation. |

### JogEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `generation` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `reason` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `axis_mask` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `velocity_scale_ppm` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### FaultEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `bit` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `axis_mask` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `recovery_class` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### BrakeWarningEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `axis` | integer / uint32 | true | false | Zero-based native axis index; operator text uses J1-based numbering. |
| `diagnostic` | object / BrakeHoldDiagnostic | true | false | The axis brake_hold_diagnostic from the publication that first showed this warning_sequence. |

### DriveSupervisorEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `axis` | integer / uint32 | true | false | Zero-based native axis index; operator text uses J1-based numbering. |
| `diagnostic` | object / DriveSupervisorDiagnostic | true | false | The axis drive_supervisor from the publication that first showed this event_sequence. |

### EnableEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `enabled_mask` | integer / uint32 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `armed` | boolean / bool | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### EpochEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `previous` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `current` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### IncarnationEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `previous` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `current` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### PublisherOverflowEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `previous_drops` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `total_drops` | integer / uint64 | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### EventsDropped

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `first_lost_sequence` | integer / uint64 | true | false | Inclusive first missing event. |
| `last_lost_sequence` | integer / uint64 | true | false | Inclusive last missing event; synthetic drop event sequence equals this cursor, so retained sequences are never renumbered. |

### TelemetryMarkEvent

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `label` | string / string | true | false | Accepted mark label, 1..128 UTF-8 bytes; receipt of queued dump work, not proof of disk output. |
| `native_sequence` | integer / uint64 | true | false | Accepted mark command sequence returned by POST /v1/control; distinct from event and telemetry sample cursors. |
| `generation` | integer / uint64 | true | false | Authorizing application grant generation; session capability is never published. |

### Event

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `sequence` | integer / uint64 | true | false | Strictly increasing within adapter_incarnation; drop records use last_lost_sequence; no renumbering. |
| `time_ns` | integer / uint64 | true | false | Native observation host time, except telemetry_mark uses adapter host monotonic time after native acceptance; never proof of motion or disk output. |
| `type` | string / string | true | false | grant_acquired, grant_renewed, grant_released, grant_expired, grant_revoked, enable_changed, arm_changed, handle_transition, execution_started, execution_completed, execution_faulted, execution_aborted, jog_begin, jog_end, jog_expired, jog_ramping, jog_limited, fault_latched, fault_cleared, brake_warning, drive_supervisor, home_epoch_changed, daemon_incarnation_changed, adapter_incarnation_changed, publisher_overflow, events_dropped, telemetry_mark |
| `daemon_incarnation` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `adapter_incarnation` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `grant` | object / *GrantEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `handle` | object / *HandleTransitionEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `execution` | object / *ExecutionEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `jog` | object / *JogEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `fault` | object / *FaultEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `brake_warning` | object / *BrakeWarningEvent | false | true | Present for brake_warning only: a new per-axis warning_sequence. A warning is not a fault and needs no reset. |
| `drive_supervisor` | object / *DriveSupervisorEvent | false | true | Present for drive_supervisor only: a new per-axis event_sequence. An ended Arm attempt or disarm-motion warning is not a fault and needs no reset. |
| `enable` | object / *EnableEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `epoch` | object / *EpochEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `incarnation` | object / *IncarnationEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `overflow` | object / *PublisherOverflowEvent | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `events_dropped` | object / *EventsDropped | false | true | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |
| `mark` | object / *TelemetryMarkEvent | false | true | Present for telemetry_mark only, once per accepted mark command; rejected commands produce no mark event. |

### EventBatch

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `events` | array / []Event | true | false | At most 64 records, including at most one leading events_dropped record; 256 retained events. |
| `next_sequence` | integer / uint64 | true | false | Resume cursor after the last returned event; unchanged when empty. |
| `latest_sequence` | integer / uint64 | true | false | Newest retained sequence at batch capture. |
| `adapter_incarnation` | string / string | true | false | Native daemon publication; time_ns identifies the observation, never adapter-estimated motion. |

### TelemetryBatch

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `daemon_incarnation` | binary / string | true | false | 32 lowercase hex bytes identifying the daemon owning these sequences; reconnect must reconcile changes. |
| `adapter_incarnation` | binary / string | true | false | 32 lowercase hex bytes identifying the adapter incarnation. |
| `machine_sha256` | binary / string | true | false | 64 lowercase hex machine digest bytes. |
| `deployment_sha256` | binary / string | true | false | 64 lowercase hex deployment digest bytes. |
| `cycle_period_ns` | binary / uint64 | true | false | Cycle period in ns. |
| `axis_count` | binary / uint32 | true | false | Active axes, 1 through 16. |
| `record_layout_digest` | binary / string | true | false | 64 ASCII hex bytes from the generated transitive CycleCaptureRecordV2 layout digest. |
| `first_sequence` | binary / uint64 | true | false | First included sequence; zero when empty. |
| `last_sequence` | binary / uint64 | true | false | Last included sequence or after+dropped when empty; resume cursor. |
| `dropped` | binary / uint64 | true | false | Exact number of records lost after the requested cursor and before this batch. |
| `record_count` | binary / uint32 | true | false | Number of complete records, at most min(ring capacity,4096). |
| `records` | binary / []uint8 | true | false | Opaque binary body; never encoded as JSON or base64. Each CycleCaptureAxisV2 appends velocity_actual_counts_per_s (i32), following_error_counts (i32), velocity_actual_valid (u32 0/1), following_error_valid (u32 0/1). Drive position-frame counts/s and counts; following error also feeds the configured collision watchdog. No speed-safety certification. |

### IoInput

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `name` | string / string | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `value` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `valid` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `observed_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `bit` | integer / uint32 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `fast` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |

### IoOutput

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `name` | string / string | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `intent` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `commanded` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `readback` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `valid` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `expiry_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `observed_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `changed_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `marker_sample_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `marker_cycle` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `bit` | integer / uint32 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `torch` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |

### IoState

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `time_ns` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `cycle` | integer / uint64 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `armed` | boolean / bool | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `reason` | integer / uint32 | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `inputs` | array / []IoInput | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |
| `outputs` | array / []IoOutput | true | false | Native cycle observation or bounded marker intent; physical readback is independent. |

### ResourceInfo

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `path` | string / string | true | false | Description-relative path (robot_description_manifest.json, robot.urdf, meshes/<file>, rtcore_definition.json, ...) or machine_planning_calibration.json. Never a server filesystem path. |
| `sha256` | string / string | true | false | Lowercase SHA-256 of the exact resource bytes. |
| `bytes` | integer / uint64 | true | false | Exact resource byte count; unverified maximum 33554432 bytes. |
| `media_type` | string / string | true | false | application/json, application/xml, model/vnd.collada+xml, model/stl or application/octet-stream. |

### RobotDescription

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `model_id` | string / string | true | false | The robot description's model id: the directory name under robot_description/robots/ the machine was compiled with. |
| `robot_description_sha256` | string / string | true | false | sha256:<64 lowercase hex>, the description's manifest identity over the resources it registers (docs/ROBOT-DESCRIPTION-CONTRACT.md); the identity a plan's header must carry. |
| `machine_planning_calibration_sha256` | string / string | true | false | sha256:<64 lowercase hex> of the served machine_planning_calibration.json bytes, this machine's deviation from the description; the identity a plan's header must carry. |
| `resources` | array / []ResourceInfo | true | false | The description's manifest and every file it registers, by description-relative path, plus machine_planning_calibration.json; unverified maximum 1024 entries and 134217728 distinct bytes. |

### DriveIdentity

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `expected_vendor_id` | integer / uint32 | true | false | Configured vendor expectation used by IgH slave matching; not an independent vendor readback. |
| `expected_product_code` | integer / uint32 | true | false | Configured product expectation used by IgH slave matching; not an independent product readback. |
| `observed_coe_revision` | integer / uint32 | true | false | Observed CoE 0x1018:3 firmware revision; distinct from the configured SII revision and never substituted from configuration. |

### DriveDescription

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `axis` | string / string | true | false | Configured axis ID in native axis order. |
| `config_name` | string / string | true | false | Compiled drive configuration name. |
| `config_sha256` | string / string | true | false | SHA-256 of canonical drive configuration content used in the compiled identity. |
| `slave_position` | integer / uint16 | true | false | Configured EtherCAT slave position. |
| `verified_identity` | object / *DriveIdentity | false | false | Null unless current native configuration verification and CoE revision readback are valid. Vendor/product are configured expectations, revision is observed. Simulation synthesizes vendor/product from the profile and cannot independently inject their mismatch. |

### ReferenceVerifiedDeltaRestorePolicy

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `method` | string / string | true | false | independent_absolute_delta_v1 only. No delta cap: the restored position must lie within the joint's hard limits; encoder/PDO delta agreement, unchanged 607C and fresh inhibited acquisition remain mandatory. |

### AnchorDeltaRestoreStatus

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `method` | string / string | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `anchor_version` | integer / uint32 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `valid` | boolean / bool | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `saved_command_counts` | integer / int32 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `saved_encoder_counts` | integer / int32 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `saved_logical_offset_counts` | integer / int32 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `saved_drive_home_offset_raw` | integer / int32 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `saved_home_epoch` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_restore_command_sequence` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_configuration_epoch` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_home_epoch` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_axis_coordinate_epoch` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_drive_frame_epoch` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_completed_ns` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_pdo_receipt` | integer / uint64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_delta_command_counts` | integer / int64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |
| `bound_delta_encoder_counts` | integer / int64 | true | false | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |

### VerifiedDeltaRestoreReceipt

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `anchor_delta_restore` | object / *AnchorDeltaRestoreStatus | false | true | Native-authored saved-anchor restoration proof; original anchor and logical zero remain unchanged. |

### MaintenanceObservation

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `active` | boolean / bool | true | false | True while the servo host's notice file exists and does not say active false; an unreadable or malformed file is active (fails closed) and a notice past its ETA stays active. Exactly the state that refuses acquire and motion starts with busy. |
| `malformed` | boolean / bool | true | false | The file exists but is not a complete notice (unreadable, not JSON, unknown schema or started_at not RFC 3339); still active. |
| `reason` | string / string | true | false | The notice's own reason text, trimmed and clipped to 400 bytes; the text a busy refusal wraps. Empty when inactive or when the notice has none. |
| `phase` | string / string | true | false | The deploy's current step as written, trimmed and clipped to 400 bytes; empty when absent. |
| `started_at` | string / string | true | false | RFC 3339 UTC time the notice began; empty when inactive or unparseable. |
| `eta` | string / string | true | false | RFC 3339 UTC expected end as written by the deploy; empty when absent or unparseable. Informational: passing it never clears the notice. |
| `set_by` | string / string | true | false | Who or what set the notice, as written, clipped to 400 bytes; empty when absent. |
| `release` | string / string | true | false | The release or commit being installed, as written, clipped to 400 bytes; empty when absent. |
| `holder` | object / *MaintenanceHolder | false | true | The current application grant's holder, present whether or not a notice is active; absent while no grant is held. Never carries the session token. |

### MaintenanceHolder

| JSON field or flattened type | Wire/codec type | Required | Omit empty | Constraint |
|---|---|---|---|---|
| `controller` | string / string | true | false | The caller-chosen acquire controller name (1..63 bytes); a label, not an identity. |
| `remote` | boolean / bool | true | false | True when the grant was acquired over the mutual-TLS listener; false for the local Unix socket. |
| `identity` | string / string | true | false | The remote client certificate identity that owns the grant; empty for the local socket. |
| `generation` | integer / uint64 | true | false | The grant's application generation; not a fence on its own without the session. |
| `since` | string / string | true | false | RFC 3339 UTC time this adapter granted it (adapter wall clock); renewals do not move it. |
<!-- END SCHEMA TYPES -->

## Complete adapter reason catalogue

Literal labels include startup and TLS failures, not only HTTP reasons. Diagnostic
patterns and underlying errors remain open as specified in the rules table.

<!-- BEGIN SCHEMA REASONS -->
| Literal reason | Schema meaning |
|---|---|
| no_grant | Halt requires a current valid application grant. |
| wrong_generation | Halt carries a different application or native generation. |
| inhibited | Halt requires an armed, enabled machine with current readiness; Stop or a fault takes precedence. |
| invalid_telemetry_cursor | Telemetry after must be one unsigned decimal uint64 cursor. |
| telemetry_cursor_ahead | Telemetry cursor exceeds the current ring sequence; reconcile incarnation before restarting. |
| telemetry_busy | Ring header remained torn after the bounded 16 samples; retry the same cursor. Established streams continue with an empty batch and unchanged cursor. |
| telemetry_unavailable | A compatible live telemetry ring could not be observed. |
| telemetry_label_invalid | Telemetry label must contain 1..128 valid UTF-8 bytes. |
| axis_count_mismatch | Dense axis count differs from the nine-axis format. |
| blob_length_mismatch | Dense payload length differs from the declared block extents. |
| block_layout_invalid | Dense block offsets, record lengths or segment metadata counts are inconsistent. |
| block_sha256_mismatch | Dense segment bytes do not match their declared SHA-256 digest. |
| dense_schema_mismatch | Dense header schema does not name the supported dense trajectory format. |
| duration_mismatch | Dense segment duration differs from its final sample clock. |
| header_json_invalid | Dense header cannot be decoded or encoded as JSON. |
| header_truncated | Dense header length or header bytes are incomplete. |
| kind_invalid | Dense segment kind is not freespace, weld or dwell. |
| limits_invalid | Dense velocity limits must contain nine positive finite values. |
| nonfinite_sample | Dense time, position or velocity sample is not finite. |
| q_step_exceeded | Dense position step exceeds the format velocity allowance. |
| qd_limit_exceeded | Dense supplied velocity exceeds its declared format limit. |
| reserved_flags_set | Dense sample sets a reserved process flag bit. |
| sample_count_overflow | Dense source sample count exceeds the format capacity. |
| sample_encoding_mismatch | Dense record encoding or record byte count is unsupported. |
| segment_index_out_of_order | Dense segment indices do not follow payload order. |
| segment_too_short | Dense segment contains fewer than two samples. |
| segments_empty | Dense program contains no motion segments. |
| time_grid_invalid | Dense sample clock is not on its declared positive time grid. |
| torch_outside_weld | Dense torch flag is set outside a weld segment. |
| total_sample_count_mismatch | Dense total sample count differs from its segment counts. |
| trajectory_digest_mismatch | Dense content digest differs from the declared trajectory digest. |
| event_cursor_lost | Event history was lost; reconcile status before resuming from a new cursor. |
| application_generation_exhausted | Rejected: application generation exhausted. |
| authority_binding_mismatch | Rejected: authority binding mismatch. |
| body_too_large | Rejected: body too large. |
| bound path is not a socket | Rejected: bound path is not a socket. |
| boundary_q_discontinuity | Rejected dense segment position gap or accumulated normalisation correction above the boundary tolerance. |
| boundary_qd_nonzero | Rejected dense segment endpoint velocity above the stationary boundary tolerance. |
| busy | Rejected: busy. |
| capability_unimplemented | The named target capability is unavailable and performs no operation. |
| continuous_weld_discontinuity | Rejected continuous weld segments whose adjacent endpoint positions or velocities disagree. |
| control payload too large | The WebSocket transport refused control payload too large. |
| control_already_owned | Rejected: control already owned. |
| control_session_stale | Rejected: control session stale. |
| session_principal_mismatch | The session belongs to a different TLS principal/pair or local transport authority. |
| daemon_restarted | Rejected: daemon restarted. |
| dense_axis_map_requires_nine_ids | Rejected: dense axis map requires nine ids. |
| duplicate_request_field | Rejected: duplicate request field. |
| expired | Rejected: expired. |
| fault_persists | At least one fault condition still prevents clearing. |
| pdo_mapping_mismatch | Assigned PDO mapping differs from the registered layout; corrected bring-up is required. |
| fence | Rejected: fence. |
| fragmentation rejected | The WebSocket transport refused fragmentation rejected. |
| handle_active | Rejected: handle active. |
| handle_consumed | This handle completed or was replaced by a later execution. |
| handle_discarded | This handle was explicitly discarded. |
| handle_retired | Cancellation or uncertain ownership permanently retired this handle. |
| handle_started | This handle has already started. |
| handle_superseded | A committed replacement superseded this handle. |
| hijacking unavailable | Rejected: hijacking unavailable. |
| independent_jog_unavailable | Rejected: independent jog unavailable. |
| invalid UTF-8 | The WebSocket transport refused invalid UTF-8. |
| invalid close code | The WebSocket transport refused invalid close code. |
| invalid close payload | The WebSocket transport refused invalid close payload. |
| invalid close reason | The WebSocket transport refused invalid close reason. |
| invalid frame flags | The WebSocket transport refused invalid frame flags. |
| invalid opcode | The WebSocket transport refused invalid opcode. |
| invalid remote CRL PEM | Rejected: invalid remote CRL PEM. |
| invalid remote pair SAN | Rejected: invalid remote pair SAN. |
| invalid remote principal SAN | Rejected: invalid remote principal SAN. |
| invalid websocket upgrade | Rejected: invalid websocket upgrade. |
| invalid_control_generation | Rejected: invalid control generation. |
| invalid_dense_axis_map | Rejected: invalid dense axis map. |
| invalid_pair_configuration_binding | Rejected: invalid pair configuration binding. |
| invalid_request_envelope | Rejected: invalid request envelope. |
| invalid_request_id | Rejected: invalid request id. |
| jog_clock_arithmetic_range | Rejected: jog clock arithmetic range. |
| jog_clock_calibration_expired | Rejected: jog clock calibration expired. |
| jog_clock_exchange_inconsistent | Rejected: jog clock exchange inconsistent. |
| jog_clock_incarnation_mismatch | Rejected: jog clock incarnation mismatch. |
| jog_clock_invalid_budget_or_exchange | Rejected: jog clock invalid budget or exchange. |
| jog_clock_mapping_generation_mismatch | Rejected: jog clock mapping generation mismatch. |
| jog_clock_moved_backwards | Rejected: jog clock moved backwards. |
| jog_clock_uncertainty_exceeded | Rejected: jog clock uncertainty exceeded. |
| jog_clock_unqualified | Rejected: jog clock unqualified. |
| jog_input_deadline_expired | Rejected: jog input deadline expired. |
| jog_input_entirely_future | Rejected: jog input entirely future. |
| jog_input_too_old | Rejected: jog input too old. |
| jog_invalid_frame | Rejected: jog invalid frame. |
| jog_publisher_busy | Rejected: jog publisher busy. |
| jog_session_or_sequence_stale | Rejected: jog session or sequence stale. |
| jog_session_stale | Rejected: jog session stale. |
| jog_stream_idle | Rejected: no complete jog stream frame within FastControlMaxJogInputAgeNs (250,000,000 ns). |
| path_jog_unavailable | Path jog needs Describe path_jog (machine jog_jerk) and a path session; the cell or session has neither. |
| path_queue_full | The adapter already holds its unverified bound of 64 unacknowledged path chunks; resend after acknowledgements. |
| path_chunk_stale | A path chunk is not the next chunk_sequence of the current path session. |
| manifest_revision_mismatch | Rejected: manifest revision mismatch. |
| mode_conflict | Another active motion mode prevents this operation. |
| native_execution_failed | Native execution ended without successful completion; motion completion is unverified. Inspect status and fault details, correct the cause, and explicitly prepare and start a new execution. |
| native_limit_exceeded | A dense sample exceeds a native position, velocity or declared sampled acceleration limit. |
| outside_limits_outward | A motion request increases an exterior limit excursion or crosses the opposite boundary; native command reason 8 and jog reason 15. |
| native_rejected | Native command admission refused; inspect native_result. |
| native_segment_rate_exceeded | Rejected: native segment rate exceeded. |
| noncanonical length | The WebSocket transport refused noncanonical length. |
| not_ready | The controlled axes do not satisfy native movement readiness. |
| payload too large | The WebSocket transport refused payload too large. |
| process start time unavailable | Rejected: process start time unavailable. |
| process_io_executor_not_qualified | Rejected: process io executor not qualified. |
| program_already_executing | Rejected: program already executing. |
| program_identity_mismatch | Rejected: program identity mismatch. |
| program_identity_missing | Rejected: program identity missing. |
| recovery_observation_unavailable | Rejected: recovery observation unavailable. |
| remote CA must be a self-signed component CA | Rejected: remote CA must be a self-signed component CA. |
| remote CA must contain one CA certificate | Rejected: remote CA must contain one CA certificate. |
| remote CA, certificate, key and CRL are required | Rejected: remote CA, certificate, key and CRL are required. |
| remote CRL not current | Rejected: remote CRL not current. |
| remote client must be issued directly by component CA | Rejected: remote client must be issued directly by component CA. |
| remote client revoked | Rejected: remote client revoked. |
| remote identity SAN must be an opaque token | Rejected: remote identity SAN must be an opaque token. |
| remote pair-id required | Rejected: remote pair-id required. |
| remote principal/pair binding mismatch | Rejected: remote principal/pair binding mismatch. |
| request_envelope_changed | Rejected: request envelope changed. |
| request_id_conflict | Rejected: request id conflict. |
| reset_requires_inhibited | Rejected: reset requires inhibited. |
| schema_mismatch | Rejected: schema mismatch. |
| segment_boundary_discontinuous | Rejected moving segment seam; boundary is the zero-based join index and gap_counts reports second minus first in source-axis counts. |
| socket lock ownership cannot be verified | Rejected: socket lock ownership cannot be verified. |
| socket lock path changed | Rejected: socket lock path changed. |
| socket owned by a live process | Rejected: socket owned by a live process. |
| socket ownership cannot be verified | Rejected: socket ownership cannot be verified. |
| trailing_request_data | Rejected: trailing request data. |
| unknown_endpoint | Rejected: unknown endpoint. |
| unknown_handle | Rejected: unknown handle. |
| unknown_operation | Rejected: unknown operation. |
| unmapped_dense_axis | The configured dense axis ID has no native axis mapping. |
| unsupported critical CRL extension | Rejected: unsupported critical CRL extension. |
| unsupported socket ownership transport | Rejected: unsupported socket ownership transport. |
| anchor_missing | No persisted anchor exists for the axis. |
| anchor_identity_mismatch | The persisted anchor was recorded under a different configuration digest, drive identity or home epoch. |
| anchor_source_invalid | The drive reports no valid absolute source for the axis. |
| anchor_disagrees | The current absolute source disagrees with the persisted anchor beyond the profile tolerance. |
| anchor_store_io | Anchor storage could not be read or persisted; unverified stored evidence cannot authorize restoration. Filesystem details are logged locally and never returned in public feedback. Retry after checking the anchor directory and disk. On restore refusal, the anchor store is not modified. |
| invalid_axis_mask | The requested axis mask is empty or names an axis outside the configured group. |
| event_cursor_ahead | Cursor exceeds this adapter incarnation's newest event; obtain current status/incarnation before resuming. |
| invalid_event_cursor | Event cursor must be a single unsigned decimal uint64. |
| trajectory_identity_mismatch | Raw trajectory Start identity differs from its immutable preparation. |
| invalid_requested_lease | Acquire requested lease is negative or exceeds the unverified absolute 10000 ms bound. |
| io_not_configured | P6-05: named cell I/O is not configured. |
| io_not_armed | Explicit io_arm is required before an ON intent. |
| io_fast_input_unsatisfied | A cyclic fast contact is invalid or not satisfied. |
| io_readback_disagreement | Independent physical feedback disagrees with commanded output. |
| io_torch_unqualified | P6-05: torch-class markers remain refused pending hardware qualification. |
| io_marker_late | A process marker missed its one-cycle delivery bound. |
| io_exchange_lost | Cell I/O has no current complete exchange. |
| io_wait_timeout | The program input did not satisfy its stable interval before the authored timeout; execution stops with outputs cleared. |
| io_wait_input_invalid | The sensor wait has no fresh valid input observation; execution stops with outputs cleared. |
| io_marker_invalid | A marker names an unknown output or has an invalid sample association. |
| resource_unknown | Digest is malformed or absent from the loaded compiled resource set; HTTP 404. |
| resource_artifact_invalid | Startup refuses an incomplete, mismatched, malformed or over-bound compiled resource artefact. |
<!-- END SCHEMA REASONS -->

## Integration profile

This profile uses the public application socket. Source citations in this section
refer to worktree commit `10a5b0351a7bf6eeabe2234f4d2678f3b16e3dbf`.
The Go and C++ clients provide separate lifecycle, urgent and upload connections;
keep renewal running during Home, upload and execution.
[sdk/control/client.go:45](../sdk/control/client.go#L45),
[clients/cpp/include/rosie/rt_control_client.hpp:394](../clients/cpp/include/rosie/rt_control_client.hpp#L394).

The executable examples use the same nine-axis homing machine and immutable `.rdt`
fixtures. `make test-go` runs the Go public-SDK sequence and builds/runs the C++
header-only sequence against the simulation daemon and rt-control. The document
check compares this section's Operation column with actual Go calls and checks
all marked reason references against generated constants. Passing these tests
records current behavior; the conformance gaps found at first publication were closed at 4ad5aad9 (P6-09).
[sdk/control/example_motion_server_test.go:85](../sdk/control/example_motion_server_test.go#L85),
[clients/cpp/examples/motion_server_sequence.cpp:40](../clients/cpp/examples/motion_server_sequence.cpp#L40),
[clients/cpp/cppclient_linux_test.go:392](../clients/cpp/cppclient_linux_test.go#L392),
[sdk/control/profile_document_test.go:100](../sdk/control/profile_document_test.go#L100),
[Makefile:274](../Makefile#L274).

Each control request needs `schema`, `operation`, and, after acquisition, the
returned `session` and current `generation`. The table's J, F and N entries refer
to the shared refusal tables below; they apply in addition to each row's labels.
An unknown diagnostic, transport failure or malformed response means abort the
motion attempt, issue authenticated Stop if possible, and reconcile status before
acquiring new authority. HTTP success acknowledges admission, not physical completion.
[adapters/rosie/control/http.go:177](../adapters/rosie/control/http.go#L177),
[sdk/control/client.go:267](../sdk/control/client.go#L267),
[adapters/rosie/control/handles.go:85](../adapters/rosie/control/handles.go#L85).

| Operation | Required fields / prerequisites | Result and consumer action | Refusals and response |
|---|---|---|---|
| `telemetry` | GET `/v1/telemetry?after=<sequence>` or `/v1/telemetry/stream?after=<sequence>`; no grant. | `TelemetryBatches` / `TelemetryStream`: compare records and exact dropped counts, retain the last consumed sequence and reconcile incarnation changes. | Invalid/future cursors return 409. Busy headers are transient: at most two reads (eight header samples each), then 409 `telemetry_busy` with `Retry-After: 1`; retry the same cursor. Established streams continue with an empty batch and unchanged cursor on busy. Other unavailable-ring refusals require checking native availability. |
| `describe` | GET `/v1/describe`; no grant. Compare `contract_version` and `capabilities_digest` with the supported generated contract. Verify every operation in this column is `implemented`; `test_only`, `interim` and `unimplemented` do not satisfy this profile. Check backend, axis order/units/limits and expected machine **and deployment** SHA-256 against approved deployment artifacts. | `Description`; retain digests and limits. Bind motion authority to `machine_sha256`; `deployment_sha256` is a separately checked deployment identity, not an Acquire field. Abort any mismatch. [controller.go:446](../adapters/rosie/control/controller.go#L446), [description.hpp:9](../include/description.hpp#L9), [api_generated.go:12](../adapters/rosie/control/api_generated.go#L12). | No operation-specific rejection on this GET; common unknown diagnostics abort. [http.go:63](../adapters/rosie/control/http.go#L63). |
| `acquire` | POST `/v1/control`; `controller` is a nonempty identity of at most 63 bytes, not a credential; `binding.pair_id`, `binding.revision`, `binding.machine_sha256`. Local socket permissions or remote TLS authenticate the connection. | `Grant`: save session, generation, binding, `lease_ms`, `deadline_host_ns`, `stopping`. A supplied machine digest wins over legacy `configuration_sha256`; a wrong machine digest never falls back. [controller.go:499](../adapters/rosie/control/controller.go#L499), [remote_tls.go:52](../adapters/rosie/control/remote_tls.go#L52). | J, N; [reason:authority_binding_mismatch](../adapters/rosie/control/api_generated.go#L127): operator corrects identity/pair/revision/digest; [reason:control_already_owned](../adapters/rosie/control/api_generated.go#L143): retry only after owner releases; [reason:application_generation_exhausted](../adapters/rosie/control/api_generated.go#L125): abort and operator restarts/reconciles. [controller.go:509](../adapters/rosie/control/controller.go#L509). |
| `renew` | J envelope plus session and nonzero nonfuture generation. Use a fresh `request_id` for every new renewal; a retained ID replays the old receipt. | `Grant`; update the fence. The default lease ceiling is **500 ms** (`max_grant_lease_ns`, cell-configurable and published by Describe; Acquire negotiates `requested_lease_ms`). Renew at most one third of the effective lease; the example uses an unverified **100 ms** cadence. Observe every renewal failure. `stopping:true` renews only local keepalive while Stop drains; it grants no native motion authority. Expiry inhibits outputs, cancels jog and retires handles; re-acquire, Enable/Arm, and upload anew. [example_motion_server_test.go:28](../sdk/control/example_motion_server_test.go#L28), [protocol/control.json:17](../protocol/control.json#L17), [controller.go:653](../adapters/rosie/control/controller.go#L653), [controller.go:1175](../adapters/rosie/control/controller.go#L1175), [sdk/control/renewal.go:22](../sdk/control/renewal.go#L22). | J, F, N; [reason:expired](../adapters/rosie/control/api_generated.go#L155): stop producing, reconcile and re-acquire. Older generations of the same unexpired session may renew and learn the current fence. [controller.go:731](../adapters/rosie/control/controller.go#L731). |
| `enable` | J, F; `axis_mask` selects configured axes by Describe index (nine-axis example: 511). | `Response.sequence`; poll Status for requested enable bits and drive readiness. Enable and Arm alone do not establish Home or permit Start. [controller.go:775](../adapters/rosie/control/controller.go#L775), [motion_readiness.hpp:102](../include/motion_readiness.hpp#L102). | J, F, N. Resolve the native refusal and per-axis readiness table below before retry. [http.go:242](../adapters/rosie/control/http.go#L242). |
| `arm` | J, F; no axis mask. Current verified configuration, authority and group safety gates still apply at motion admission. | `Response.sequence`; observe `core.armed == 1`. A successful Arm does not mean every axis reports ready. [controller.go:787](../adapters/rosie/control/controller.go#L787), [motion_readiness.hpp:102](../include/motion_readiness.hpp#L102). | J, F, N; inspect native receipt and readiness; operator resolves faults, then retry under live authority. [http.go:242](../adapters/rosie/control/http.go#L242). |
| `home` | J, F; nonzero configured `axis_mask`; axes must advertise `native_home`. Use when required Home evidence is absent and no qualified persisted absolute anchor applies. | `Response.sequence`; Home performs commissioning and leaves selected axes disabled. Observe `home_valid_mask`, `home_epoch` and idle `commissioning_phase`; repeat Enable/Arm and readiness checks before Start. Required Home gates Enable itself: the example observes that refusal, performs Home while inhibited, then enables and arms. [controller.go:787](../adapters/rosie/control/controller.go#L787), [commissioning_worker.hpp:1843](../engine/commissioning_worker.hpp#L1843). | J, F, N; persisted-anchor capture can also return the anchor labels below or storage diagnostics. Abort on Home failure; operator restores trustworthy coordinates. [controller.go:810](../adapters/rosie/control/controller.go#L810), [controller.go:862](../adapters/rosie/control/controller.go#L862). |
| `restore_anchor` | Alternative to Home: J, F, nonzero configured `axis_mask`; inhibited, idle machine and a persisted qualified absolute-source anchor with matching identities. The example deliberately has no anchor and observes refusal. | Returns `RecoveryStatus` read after the restore receipt; `Response.sequence` carries the receipt (15f5b353). [http.go:208](../adapters/rosie/control/http.go#L208), [controller.go:882](../adapters/rosie/control/controller.go#L882). | J, F, N; [reason:invalid_axis_mask](../adapters/rosie/control/api_generated.go#L337): abort/correct mask; [reason:mode_conflict](../adapters/rosie/control/api_generated.go#L247): inhibit and retry; [reason:anchor_missing](../adapters/rosie/control/api_generated.go#L327): operator performs Home; [reason:anchor_identity_mismatch](../adapters/rosie/control/api_generated.go#L329): operator verifies replacement/configuration and re-homes; [reason:anchor_source_invalid](../adapters/rosie/control/api_generated.go#L331): operator restores valid absolute feedback; [reason:anchor_disagrees](../adapters/rosie/control/api_generated.go#L333): abort, operator investigates displacement and re-homes. [controller.go:889](../adapters/rosie/control/controller.go#L889), [anchors/store.go:46](../adapters/rosie/control/anchors/store.go#L46). |
| `recovery_status` | J, F; inhibited, idle observation. | `RecoveryStatus`: `home_valid_mask`, `faults`, `reset_sequence`, `reason`, `outcomes`. Per-bit records carry `bit`, `name`, `axis_mask`, `recovery`, `outcome`, `rehome_axis_mask`. Read the recovery class; this operation clears nothing. Go uses public `Command` and the type reachable through `Status.Core.Recovery`; C++ has `recovery_status`. [reset_linux.go:88](../adapters/rosie/control/reset_linux.go#L88), [client_linux.go:105](../ipcclient/client_linux.go#L105), [rt_control_client.hpp:613](../clients/cpp/include/rosie/rt_control_client.hpp#L613). | J, F; [reason:program_already_executing](../adapters/rosie/control/api_generated.go#L269): Stop then retry; [reason:reset_requires_inhibited](../adapters/rosie/control/api_generated.go#L299): inhibit/wait for idle then retry; [reason:recovery_observation_unavailable](../adapters/rosie/control/api_generated.go#L275): abort recovery claim and operator checks native observations. [reset_linux.go:125](../adapters/rosie/control/reset_linux.go#L125). |
| `prepare_program` | POST `/v1/program` with immutable `.rdt` bytes, `X-Control-Session` and decimal `X-Control-Generation`. SDK also sends Content-Type and X-Request-ID, but uploads have **no deduplication**. Continue renewal. | `Program`: echo its entire `Identity` on Start: plan/program IDs, program/trajectory/source/normalised digests, manifest and plan revisions. Inspect axis mask, samples, normalised samples, segments, process markers and `requires_process_io`. Upload never enables or starts motion. [client.go:303](../sdk/control/client.go#L303), [program.go:23](../adapters/rosie/control/program.go#L23), [controller.go:1474](../adapters/rosie/control/controller.go#L1474). | F, N and upload table below. Never replay an uncertain upload automatically; inspect Describe/Status or Stop before replacing it. [http.go:80](../adapters/rosie/control/http.go#L80), [controller.go:1404](../adapters/rosie/control/controller.go#L1404). A native limit rejection of an upload (reason 6, today surfaced as `native_rejected`) retires every previously prepared public handle and marks execution `faulted`; upload only programs already checked against the declared limits, or expect to re-prepare. [controller.go:1014](../adapters/rosie/control/controller.go#L1014), [handles.go:72](../adapters/rosie/control/handles.go#L72). |
| `start_program` | J, F; complete returned `identity`; prepared handle, fresh Enable/Arm/Home/readiness, initial target continuous with held target, qualified process-I/O if required. | `Response.sequence`; record it and execution generation. Reuse the same retained request object/ID for a lost receipt; a new ID is a new command. Definite pre-admission refusal preserves/restores the prepared handle; ambiguous failure retires it. Inspect handle state before retry. [controller.go:1507](../adapters/rosie/control/controller.go#L1507), [handles.go:128](../adapters/rosie/control/handles.go#L128), [idempotence.go:31](../adapters/rosie/control/idempotence.go#L31). | J, F, N; [reason:program_identity_mismatch](../adapters/rosie/control/api_generated.go#L271): abort/reconcile Describe identity; [reason:program_already_executing](../adapters/rosie/control/api_generated.go#L269) and [reason:mode_conflict](../adapters/rosie/control/api_generated.go#L247): observe active execution or Stop before retry; [reason:process_io_executor_not_qualified](../adapters/rosie/control/api_generated.go#L267): abort, operator qualification; [reason:not_ready](../adapters/rosie/control/api_generated.go#L261): resolve readiness and retry only if handle remains prepared; [reason:unknown_handle](../adapters/rosie/control/api_generated.go#L317): upload anew; [reason:handle_started](../adapters/rosie/control/api_generated.go#L173): observe existing execution; [reason:handle_consumed](../adapters/rosie/control/api_generated.go#L167), [reason:handle_discarded](../adapters/rosie/control/api_generated.go#L169), [reason:handle_superseded](../adapters/rosie/control/api_generated.go#L175), [reason:handle_retired](../adapters/rosie/control/api_generated.go#L171): abort replay, upload anew; [reason:fence](../adapters/rosie/control/api_generated.go#L161): re-acquire and upload anew. [controller.go:1511](../adapters/rosie/control/controller.go#L1511), [handles.go:194](../adapters/rosie/control/handles.go#L194). |
| `subscribe_events` | GET `/v1/events?after=<uint64>` or SSE `/v1/events/stream?after=<uint64>`; no grant required. | `EventBatch` polling or SSE `Event` records. Consume strictly increasing sequence within `adapter_incarnation`, then retain `next_sequence`/last handled SSE ID. Match execution generation and native sequence. Explicit loss or incarnation change requires Status reconciliation before further motion. The C++ client polls with `events(after)` and streams with `events_stream(after, callback, stop)`; `events_dropped` raises `EventCursorLost` carrying the event whose sequence is the resume cursor, `publisher_overflow` is delivered like any event. [events.go:30](../sdk/control/events.go#L30), [events.go:373](../adapters/rosie/control/events.go#L373), [rt_control_client.hpp:641](../clients/cpp/include/rosie/rt_control_client.hpp#L641). | [reason:invalid_event_cursor](../adapters/rosie/control/api_generated.go#L341): abort/correct unsigned decimal cursor; [reason:event_cursor_ahead](../adapters/rosie/control/api_generated.go#L339): read current incarnation/status and resume a valid cursor. Handler returns HTTP 409 with the reason; the Go SDK surfaces it as a typed rejection (15f5b353). [events.go:292](../adapters/rosie/control/events.go#L292), [client.go:372](../sdk/control/client.go#L372). |
| `status` | GET `/v1/status`; no grant. Poll before motion and after terminal/cancellation observations. | `ProcessStatus`; require timestamped native data, correct incarnation/generations and fault-free readiness. A completed execution plus expected measured endpoint establishes example completion. `prepared`, `started`, `consumed`, `discarded`, `superseded`, `retired` are handle states; only prepared can start. [api_generated.go:498](../adapters/rosie/control/api_generated.go#L498), [handles.go:5](../adapters/rosie/control/handles.go#L5). | No closed operation-specific labels; underlying native/status errors abort the motion claim. [controller.go:468](../adapters/rosie/control/controller.go#L468). |
| `stop` | J; previously issued authenticated session, nonzero nonfuture generation. A known expired/revoked session may Stop even though it cannot move. | `Grant`; urgently inhibits outputs and cancels execution, uploads, handles and jog. A live session receives a new generation and needs fresh Enable/Arm. Expired Stop returns no motion session; re-acquire. Stop receipt does not prove physical standstill. [controller.go:1152](../adapters/rosie/control/controller.go#L1152), [controller.go:1195](../adapters/rosie/control/controller.go#L1195), [description.hpp:18](../include/description.hpp#L18). | J, F, N; a stale/unknown session cannot cancel anonymously. Reconcile authenticated identity; abort any assumption of inhibition on failed receipt. [controller.go:1197](../adapters/rosie/control/controller.go#L1197). |
| `release` | J; same authenticated fence rules as Stop. Stop and join the renew producer first. | `Grant` with empty session; discard local authority. Native Stop/inhibition and cancellation happen before completion. SDK Close alone does not release. [operations.go:102](../sdk/control/operations.go#L102), [client.go:445](../sdk/control/client.go#L445), [controller.go:1175](../adapters/rosie/control/controller.go#L1175). | J, F, N; release is not automatically replayed after transport loss. Abort motion, observe inhibition, reconcile before a fresh acquisition. [client.go:198](../sdk/control/client.go#L198). |

`request_id` applies to JSON commands: optional 1..64 printable ASCII bytes,
256 session-local cached outcomes. Identical decoded payload retries join/replay
the same result; payload changes conflict. Release, expiry, eviction and restart
end that replay guarantee. Keep one object/ID for one logical attempt; use a fresh
ID after correcting a rejected request. Re-acquisition requires clearing an old
local SDK fence (or a new client) if Stop/Release has not already cleared it.
[idempotence.go:12](../adapters/rosie/control/idempotence.go#L12),
[idempotence.go:69](../adapters/rosie/control/idempotence.go#L69),
[client.go:258](../sdk/control/client.go#L258),
[rt_control_client.hpp:536](../clients/cpp/include/rosie/rt_control_client.hpp#L536).

| Shared refusal set | Labels and required response |
|---|---|
| J: JSON envelope / replay | [reason:schema_mismatch](../adapters/rosie/control/api_generated.go#L301), [reason:unknown_operation](../adapters/rosie/control/api_generated.go#L319), [reason:unknown_endpoint](../adapters/rosie/control/api_generated.go#L315), [reason:invalid_request_envelope](../adapters/rosie/control/api_generated.go#L207), [reason:duplicate_request_field](../adapters/rosie/control/api_generated.go#L153), [reason:request_envelope_changed](../adapters/rosie/control/api_generated.go#L295), [reason:trailing_request_data](../adapters/rosie/control/api_generated.go#L313), [reason:invalid_request_id](../adapters/rosie/control/api_generated.go#L209), [reason:request_id_conflict](../adapters/rosie/control/api_generated.go#L297): abort the request and correct encoding/contract/ID; do not blindly retry. [reason:body_too_large](../adapters/rosie/control/api_generated.go#L129): reduce request size. [reason:capability_unimplemented](../adapters/rosie/control/api_generated.go#L139): abort this profile and resolve deployment capability. Unknown fields and malformed JSON can instead produce decoder diagnostics. [admission.go:74](../adapters/rosie/control/admission.go#L74), [http.go:27](../adapters/rosie/control/http.go#L27), [idempotence.go:36](../adapters/rosie/control/idempotence.go#L36). |
| F: authority | [reason:control_session_stale](../adapters/rosie/control/api_generated.go#L145): stop producing, reconcile and re-acquire; [reason:daemon_restarted](../adapters/rosie/control/api_generated.go#L149): invalidate handles/old identities, re-Describe and bind before re-acquiring. Repeated anonymous retry is never recovery. [controller.go:643](../adapters/rosie/control/controller.go#L643), [controller.go:136](../adapters/rosie/control/controller.go#L136). |
| N: native refusal | [reason:native_rejected](../adapters/rosie/control/api_generated.go#L255): inspect case-sensitive `native_result` fields. Generated native reason 1 invalid command: abort/correct; 2 not ready: resolve readiness; 3 mode conflict: wait/Stop then retry; 4 unknown handle: re-upload; 5 capacity: free/retire existing preparations then retry; 6 invalid trajectory: abort/replan. Native grant/jog failures and rollback errors can instead carry contextual diagnostics; abort unknown outcomes. [http.go:242](../adapters/rosie/control/http.go#L242), [ipcclient/protocol_generated.go:27](../ipcclient/protocol_generated.go#L27), [handles.go:168](../adapters/rosie/control/handles.go#L168). |

| Upload admission | Labels and required response |
|---|---|
| Capacity and size | One binary and one JSON preparation admission across listeners; [reason:busy](../adapters/rosie/control/api_generated.go#L137): retry after the current upload completes. At most 250000 points and three native resident plans; native reason 5 is capacity. Binary wire cap 39298580 bytes, header cap 1048576 bytes plus four-byte length; [reason:body_too_large](../adapters/rosie/control/api_generated.go#L129): abort/reduce. HTTP 413 currently becomes SDK protocol error, not typed Rejected. [reason:invalid_control_generation](../adapters/rosie/control/api_generated.go#L201): abort/correct decimal header. [admission.go:19](../adapters/rosie/control/admission.go#L19), [description.hpp:13](../include/description.hpp#L13), [client.go:372](../sdk/control/client.go#L372). |
| Identity and mapping | [reason:manifest_revision_mismatch](../adapters/rosie/control/api_generated.go#L245): rebind/replan for current pair revision; [reason:program_identity_missing](../adapters/rosie/control/api_generated.go#L273): abort/correct producer identity; [reason:dense_axis_map_requires_nine_ids](../adapters/rosie/control/api_generated.go#L151), [reason:invalid_dense_axis_map](../adapters/rosie/control/api_generated.go#L203), [reason:unmapped_dense_axis](../adapters/rosie/control/api_generated.go#L321): operator corrects deployment axis mapping; [reason:program_already_executing](../adapters/rosie/control/api_generated.go#L269): await execution or Stop before retry. [program.go:87](../adapters/rosie/control/program.go#L87), [controller.go:1412](../adapters/rosie/control/controller.go#L1412). |
| Continuity and limits | [reason:boundary_q_discontinuity](../adapters/rosie/control/api_generated.go#L133), [reason:boundary_qd_nonzero](../adapters/rosie/control/api_generated.go#L135), [reason:segment_boundary_discontinuous](../adapters/rosie/control/api_generated.go#L303): abort/replan a stationary continuous seam; [reason:native_segment_rate_exceeded](../adapters/rosie/control/api_generated.go#L257): abort/replan rate; [reason:native_limit_exceeded](../adapters/rosie/control/api_generated.go#L251): abort/replan position or velocity. Context can include segment/sample/axis or boundary and count gaps. No silent bridge or retiming. [api_generated.go:423](../adapters/rosie/control/api_generated.go#L423), [program.go:160](../adapters/rosie/control/program.go#L160). |
| Acceleration limit | The adapter pre-checks sampled acceleration against the declared `max_acceleration` with the native derivative form and refuses with [reason:native_limit_exceeded](../adapters/rosie/control/api_generated.go#L251) naming axis and sample before any native upload, so previously prepared handles survive; only an unclassified native result is [reason:native_rejected](../adapters/rosie/control/api_generated.go#L255). [program.go:179](../adapters/rosie/control/program.go#L179). |
| Catalogue completeness | Every dense-format diagnostic is a generated catalogue label (the dense schema label is `dense_schema_mismatch`, distinct from the envelope `schema_mismatch`); a test enumerates the encoder labels and the corpus refusals against the catalogue. Underlying transport and storage diagnostics remain free text in `error`. |

Per-axis `readiness` is an observation, not authority. Group lease, armed state,
configuration verification, limits, current complete bus feedback and Home still
gate Start. These status values are distinct from generated HTTP error labels.
[status_publisher.hpp:1065](../engine/status_publisher.hpp#L1065),
[motion_readiness.hpp:102](../include/motion_readiness.hpp#L102).

Home clears requested bits at reservation. The Home All change
(51a6a27a, deployed on a bench cell) commits each axis after its own
qualified native completion rather than waiting for group Finish. A peer failure
must not discard those qualified successes; failed or incomplete axes remain
invalid, and unselected axes retain their existing evidence. The worker still
stops at the first failure; this change retains success already qualified before
that failure and rejects stale completion. Per-axis `home_valid`
and Home-dependent `logical_valid` follow the resulting mask. Existing faults
and authority gates remain effective: a partial mask is not full-group readiness
or permission to move. Observe the new `home_epoch`, idle `commissioning_phase`
and every required Home bit before explicit Enable/Arm. The preceding deployed
group-Finish-only behavior is documented in the historical Home record.

Jog readiness also requires the native reservation to remain open with the same
connection, grant, configuration epoch, Home epoch, clock mapping and jog
generation. After source loss or an epoch change, ingress refuses old-session
frames with generated jog reason `wrong_identity` (4), including frames with
fresh timestamps and increasing sequences; no refused frame refreshes native
input. Unavailable native observations fail closed with `not_ready` (3).
[Generated jog reasons](../ipcclient/protocol_generated.go#L52) are separate
from HTTP error labels.
`/v1/jog/ingress` reports each observed source sequence and numeric refusal;
the composed probes count 100 `wrong_identity` refusals for 100 serial frames.
A new BeginJog and neutral first input are required to resume.

Each successful public Status snapshot belongs to one daemon incarnation:
`daemon_incarnation` equals `core.daemon_incarnation`, and native grant/jog
observations come from that same connection. During daemon replacement, missing,
stale or predecessor diagnostic files are refused; the adapter never relabels or
retains their Home evidence. A successor starts with `home_valid_mask == 0` and
remains inhibited until explicit recovery through Home or qualified
`restore_anchor`, followed by Enable/Arm. Reconcile both process incarnations,
discard old handles/jog sessions and reset telemetry continuity before resuming.

| Readiness value | Consumer response |
|---|---|
| `faulted` | Abort motion; operator diagnoses drive/safety fault and follows recovery class. [status_publisher.hpp:1068](../engine/status_publisher.hpp#L1068). |
| `warning_tolerated` | A6-EC joint declares `encoder_battery: "absent"` and the raw pair is exactly `0x603F=0x7305`, `0x203F=0x0F90` (ALF9.0). Home and Enable retain their normal authority/configuration/coordinate gates. Inspect Home, coordinate, enabled and group states separately; this observation grants no authority. |
| `mode_mismatch` | Wait only for an expected commissioning transition; otherwise operator restores CSP mode 8. [motion_readiness.hpp:16](../include/motion_readiness.hpp#L16). |
| `home_required` | Home or qualified anchor restore before Start. [motion_readiness.hpp:16](../include/motion_readiness.hpp#L16). |
| `coordinate_invalid` | Abort motion; operator restores trustworthy feedback/configuration/Home evidence. [status_publisher.hpp:1055](../engine/status_publisher.hpp#L1055). |
| `not_enabled` | Enable the intended mask under live authority, then observe again. [motion_readiness.hpp:17](../include/motion_readiness.hpp#L17). |
| `not_operation_enabled` | Poll the DS402 transition; abort/operator action if its observation deadline expires. [motion_readiness.hpp:18](../include/motion_readiness.hpp#L18). |
| `brake_wait` | Poll for qualified brake release; abort/operator action if deadline expires. [motion_readiness.hpp:19](../include/motion_readiness.hpp#L19). |
| `ready` | This axis passes its observed gate; verify the complete selected group and live grant before Start. [motion_readiness.hpp:102](../include/motion_readiness.hpp#L102). |

`warning_tolerated` precedes the remaining mode, Home, coordinate, Enable and
brake observations. DS402 or native latched safety faults still take precedence
as `faulted`. Every other nonzero raw code, including a manufacturer-only code,
is `faulted`; battery-present axes never tolerate the pair. Raw codes remain in
status and telemetry. A marked hook allows the separate `drive_alarm` decoded
name; only the raw pair governs this policy. An unavailable `0x203F` cannot
qualify the exception: current fixed PDO presets need the parallel alarm
acquisition work before this exception can be used on hardware.

Events retain 256 records and return at most 64 per batch. `events_dropped`
explicitly names the inclusive lost sequence interval; its sequence is the last
lost sequence. Native observation coalescing can omit intermediate transitions
without a drop record; events are not a lossless cyclic trace. After any uncertainty,
reconcile timestamped Status, handle state and incarnations rather than replaying
motion. SSE reconnect is explicit at the last handled sequence.
[events.go:19](../adapters/rosie/control/events.go#L19),
[events.go:53](../adapters/rosie/control/events.go#L53),
[events.go:185](../adapters/rosie/control/events.go#L185),
[sdk/control/events.go:64](../sdk/control/events.go#L64).

| Surface outside the motion-server profile | Contract / consumer action |
|---|---|
| Legacy velocity `jog` | Advertised `test_only` in describe; dispatchable for the frozen oracle and fixtures only; pendant/UI producers should use the separate lane below. [operations.go:136](../sdk/control/operations.go#L136), [jog_linux.go:25](../adapters/rosie/control/jog_linux.go#L25). |
| `prepare_trajectory`, `start_trajectory`, `discard_trajectory` | Low-level point/handle API for tests and small moves; production dense programs use `.rdt` and full Program identity. [operations.go:148](../sdk/control/operations.go#L148), [program.go:21](../adapters/rosie/control/program.go#L21). |
| `halt` | Implemented: POST `/v1/control` with `schema`, `operation`, `session`, `generation`. Returns `Response.sequence` for admission; native refusals carry `native_result`. Controlled enabled hold retains the grant, Enable and Arm while retiring active motion. [reason:no_grant](../adapters/rosie/control/halt_linux.go): acquire valid authority; [reason:wrong_generation](../adapters/rosie/control/halt_linux.go): reconcile the current generation; [reason:inhibited](../engine/cycle_machine.hpp): resolve readiness or leave native Home/service before retrying. Observe hold separately; use Stop for urgent inhibition. |
| `abort`, `readiness` | Advertised unimplemented; [reason:capability_unimplemented](../adapters/rosie/control/api_generated.go#L139): abort this route. Use supported Stop and observed Status. [http.go:177](../adapters/rosie/control/http.go#L177), [api_generated.go:669](../adapters/rosie/control/api_generated.go#L669). |
| `reset_fault` | Interim recovery operation, not routine motion startup. Inhibited/idle; zero mask selects all axes. [reason:reset_requires_inhibited](../adapters/rosie/control/api_generated.go#L299), [reason:program_already_executing](../adapters/rosie/control/api_generated.go#L269): inhibit before retry; [reason:fault_persists](../adapters/rosie/control/api_generated.go#L157): operator removes the condition; [reason:recovery_observation_unavailable](../adapters/rosie/control/api_generated.go#L275): abort any reset claim. A submitted reset retires the grant even on rejection; re-acquire after qualified recovery. [reset_linux.go:20](../adapters/rosie/control/reset_linux.go#L20). |

The jog lane accepts joint-space velocity in the configured logical axis units.
Cartesian pendant/UI input must go to the motion server for conversion to joint
velocities before that owner uses this lane. rt-core does not interpret Cartesian
frames, TCP poses or inverse kinematics. A joint-space producer can use the lane
directly after the same authority and clock checks. Legacy NATS command subjects
and joystick UDP packets do not become rt-control jog frames by renaming them.

Pendant/UI producers use a separate jog profile after the same Describe, binding,
renewal, Enable/Arm and readiness gates. Keep only the newest unsent input; never
replay backlog after a stall or reconnect. Source time means monotonic input-capture
time, not send time or wall-clock time. Local producers negotiate the same host
clock incarnation; remote producers negotiate an authenticated source incarnation
and conservative offset interval. The earliest mapped origin bounds age; mapping
uncertainty, calibration age, input age and authority may only shorten expiry.
[jog_linux.go:61](../adapters/rosie/control/jog_linux.go#L61),
[jog_clock.go:24](../adapters/rosie/control/jog_clock.go#L24),
[jog_clock.go:184](../adapters/rosie/control/jog_clock.go#L184).

| Jog surface | Required fields, result and response |
|---|---|
| `jog_clock` / remote clock exchange | Local GET `/v1/jog/clock` returns host monotonic time/incarnation. WSS handshake supplies grant/principal plus source clock calibration. Remote constructors [RtJogRemoteProducer](../clients/cpp/include/rosie/rt_jog_remote_producer.hpp) and [Client.NewRemoteJogSession](../sdk/control/jog_remote.go) perform both exchanges; the second source sample follows receipt of the first reply and echoes its `host_tx_ns`. Default daemon flags disable remote qualification: binary updates receive `jog_clock_unqualified` until the bench supplies `--remote-jog-max-uncertainty-ns`, `--remote-jog-drift-ppb`, and `--remote-jog-calibration-max-age-ns` (P0-10, P8-04). [reason:independent_jog_unavailable](../adapters/rosie/control/api_generated.go#L179): abort deployment; [reason:jog_clock_unqualified](../adapters/rosie/control/api_generated.go#L227), [reason:jog_clock_invalid_budget_or_exchange](../adapters/rosie/control/api_generated.go#L219), [reason:jog_clock_exchange_inconsistent](../adapters/rosie/control/api_generated.go#L215), [reason:jog_clock_uncertainty_exceeded](../adapters/rosie/control/api_generated.go#L225): abort and requalify mapping; [reason:jog_clock_calibration_expired](../adapters/rosie/control/api_generated.go#L213): recalibrate; [reason:jog_clock_incarnation_mismatch](../adapters/rosie/control/api_generated.go#L217), [reason:jog_clock_mapping_generation_mismatch](../adapters/rosie/control/api_generated.go#L221), [reason:jog_clock_moved_backwards](../adapters/rosie/control/api_generated.go#L223): end/reconnect and negotiate fresh identity; [reason:jog_clock_arithmetic_range](../adapters/rosie/control/api_generated.go#L211): abort/correct timestamps. [jog_clock.go:9](../adapters/rosie/control/jog_clock.go#L9), [jog_ws.go:95](../adapters/rosie/control/jog_ws.go#L95). |
| `begin_jog` | JSON session/generation, axis mask, `clock_incarnation`, `source_origin_host_ns`, `deadline_host_ns`; returns `Response.handle` as jog generation. J/F apply; [reason:mode_conflict](../adapters/rosie/control/api_generated.go#L247): await/Stop competing mode; [reason:independent_jog_unavailable](../adapters/rosie/control/api_generated.go#L179): abort; native jog refusal: inspect observation and resolve readiness/identity/timing. A new Begin is required after expiry. The remote constructors prepare an authenticated WSS stream with `jog_generation=0`, complete both calibration exchanges, then send `begin` with `seq=3`, `axis_mask`, a fresh `source_ns` and the second `host_tx_ns`. The server uses the post-calibration Begin sample's earliest host bound as the capture cutoff and samples the first-input deadline at receipt for native BeginJog and replies `begun` with `seq` and `jog_generation`. Setup consumes no native first-input allowance. Existing nonzero-generation upgrades remain supported; new clients fail closed against servers without preparation support. C++ reuses the application-provided TLS `HttpStream` factory via `open_remote_jog_stream`; Go reuses `DialTLS` credentials. [http.go:218](../adapters/rosie/control/http.go#L218), [jog_linux.go:29](../adapters/rosie/control/jog_linux.go#L29). |
| `update_jog` | Generated binary frame over local `jog.sock` datagram or authenticated WSS `/v1/jog`; no per-frame HTTP command. Carry grant and jog identity, strictly increasing nonzero source sequence, source origin/deadline, axis mask and finite vector. Call `RtJogRemoteProducer::update(velocities, source_time)` or `RemoteJogSession.Update(velocities, sourceNS)`; both encode the generated LocalJogUpdate frame. The constructor source clock and each update timestamp must share a monotonic domain. `rejection` / `Rejection` expose typed wire refusals, including generated native codes; `observe` / `Observe` expose the generated jog observation in `/v1/status` and typed expiry. The fixed remote handler reserves `/v1/jog` for WSS upgrades; plain HTTPS GET there returns `jog_session_stale`, while local GET `/v1/jog` remains the observation endpoint. Producers report typed expiry only for native `Expired`; `Closed` and `Ramping` retain the receiver's actual reason. Simulation ramps an aged-out input and then publishes native `Expired` (hardware inhibits and stores `Expired` directly); an application End publishes `Closed`. After admission the receiver closes WSS when no complete frame arrives within the cell input-age ceiling and calls EndJog; neither producer sends automatic keepalive. The receiver answers before it closes: [reason:jog_stream_idle](../adapters/rosie/control/api_generated.go#L243) after the published cell input-age ceiling without a complete frame (LAN default 250,000,000 ns), then EndJog; [reason:control_session_stale](../adapters/rosie/control/api_generated.go#L145) for a frame after Stop, then close. A bare EOF is a transport fault, not a receiver decision. The unverified 5 ms example cadence leaves margin below the 250,000,000 ns source-age ceiling. Observe JogObservation/JogIngressObservation separately. [reason:jog_publisher_busy](../adapters/rosie/control/api_generated.go#L237): replace unsent input with newest; [reason:jog_session_or_sequence_stale](../adapters/rosie/control/api_generated.go#L239), [reason:jog_session_stale](../adapters/rosie/control/api_generated.go#L241): end/rebegin with fresh identity, never replay; [reason:jog_invalid_frame](../adapters/rosie/control/api_generated.go#L235): abort/correct producer; [reason:session_principal_mismatch](../adapters/rosie/control/api_generated.go#L147): the WSS connection principal is not the session owner, abort/re-authenticate; [reason:jog_input_too_old](../adapters/rosie/control/api_generated.go#L233), [reason:jog_input_entirely_future](../adapters/rosie/control/api_generated.go#L231), [reason:jog_input_deadline_expired](../adapters/rosie/control/api_generated.go#L229): discard frame and repair source-time/freshness before fresh Begin. F and clock refusals also apply; rejected frames do not extend expiry. [jog_linux.go:64](../adapters/rosie/control/jog_linux.go#L64), [jog_linux.go:242](../adapters/rosie/control/jog_linux.go#L242), [jog_ws.go:226](../adapters/rosie/control/jog_ws.go#L226), [protocol/control.json:21](../protocol/control.json#L21). |
| `end_jog` | JSON session/generation and `jog_generation`; returns that generation in `Response.handle`. J/F apply; [reason:jog_session_stale](../adapters/rosie/control/api_generated.go#L241): observe the ended/expired session and use a new Begin; [reason:independent_jog_unavailable](../adapters/rosie/control/api_generated.go#L179): abort. End revokes input before its native receipt. Observe ramp/hold or inhibition; End does not release application authority. `RtJogRemoteProducer::end` and `RemoteJogSession.End` send EndJog through the control client using the original fence before closing WSS; call Release separately. End during the expiry ramp may return a typed native `Closed` refusal; after the ramp it can succeed with native reason `None`. The examples preserve the actual receipt and release separately. [jog_linux.go:123](../adapters/rosie/control/jog_linux.go#L123), [http.go:223](../adapters/rosie/control/http.go#L223). |

### Go remote-jog clock maintenance

Go remote-jog clock maintenance (2026-10-01): run
`RemoteJogSession.Rejection(ctx)` alongside `Update` throughout the hold.
The receiver consumes matched calibration replies as well as typed refusals.
Fresh caller updates initiate a two-message refresh after half the requested
input lifetime; the second timestamp is captured after the first reply.
Calibration carries no velocity, does not replay input or extend an accepted
sample's deadline, and does not reopen an expired jog generation. No background
timer manufactures input. A contended SDK update returns `ErrJogWouldBlock`;
consumers discard that unsent sample and wait for new input. OLP preserves the
current generation without claiming admission or renewing its native deadline.
Clock-policy qualification and all configured admission bounds remain required.

## Telemetry publication

| Operation | Transports |
|---|---|
| `telemetry` | `GET /v1/telemetry?after=<sequence>` and `GET /v1/telemetry/stream?after=<sequence>` |

The `telemetry` capability observes every captured cycle without a session or
motion authority. Existing Unix socket protection and remote transport
authentication still apply. `GET /v1/telemetry?after=<sequence>` returns one
`application/octet-stream` batch. Finite batches negotiate gzip when the client
accepts it; decompression preserves the same binary layout and validation.
`GET /v1/telemetry/stream?after=<sequence>`
returns concatenated batches using chunked HTTP. Each server write/flush carries
one batch, including empty batches. The unverified flush cadence is 20 ms to
amortize HTTP overhead while bounding batching delay; network scheduling is not
a realtime guarantee. A blocked write terminates after 5 seconds. Intermediaries
may reframe HTTP chunks, so readers use the batch header to find boundaries.

`protocol/control.json` generates the fixed 312-byte little-endian
`TelemetryBatchHeaderV1`. Its fields are:

| Field | Encoding and meaning |
|---|---|
| `magic`, `version`, `header_bytes` | uint32 `0x31425452` (RTB1), uint16 1, uint16 312 |
| `adapter_incarnation`, `daemon_incarnation` | 32 ASCII lowercase hex bytes each; identity of the publisher and sequence owner |
| `machine_sha256`, `deployment_sha256` | 64 ASCII lowercase hex digest bytes each |
| `record_layout_digest` | 64 ASCII hex bytes equal to generated `CycleCaptureRecordV2LayoutDigest`, including the nested axis layout |
| `cycle_period_ns` | uint64 cycle period in ns |
| `first_sequence`, `last_sequence` | uint64 sequence range; last is the resume cursor |
| `dropped` | uint64 records lost after the requested cursor and before this batch |
| `axis_count`, `record_count`, `record_bytes` | uint32 active axis count, included records, and 5392 (capture V2); field offsets are listed below |
| `reserved` | uint32 zero; nonzero is malformed |

The body is exactly `record_count` complete 4,880-byte `CycleCaptureRecordV2`
records. Active axes retain every captured field; unused axes are zero-filled
when expanding the compact ring slot. No telemetry record is base64 inside JSON.
The schema's logical `TelemetryBatch` is a binary transport type; its generated
Go JSON marshaler refuses JSON and its C++ type has no JSON codec.

Like events, `after` is the last consumed sequence. Omission means zero; zero
requests sequence 1 and reports overwritten history through `dropped`. Each
nonempty batch starts at `after + dropped + 1`; sequences are contiguous within
the batch. An empty batch has first sequence zero and last sequence
`after + dropped`. Resume at last sequence only after handling the batch and its
loss. Invalid decimal, duplicate, empty or overflowing cursors return HTTP 409
`invalid_telemetry_cursor`; future cursors return 409 `telemetry_cursor_ahead`.
Incompatible, corrupt or unavailable rings return `telemetry_unavailable`.
Streams close on native disconnect; clients reject incarnation changes and must
reconcile status/description before resetting a cursor after restart.

The handler reads through the existing read-only ipcclient descriptor/mapping
reader without holding adapter locks. A batch contains at most
`min(ring capacity, 4096)` records, hence its byte bound is
`312 + min(ring capacity,4096)*5392`, at most **22,085,944 bytes**. Streaming has
the same per-batch bound and no per-subscriber record queue.

Go uses generated ipcclient decoders through `sdk/control.TelemetryBatches(ctx,
after)` and `TelemetryStream(ctx, after, callback)`; `TelemetryPeek(ctx, after)`
reads one batch header and its first record and abandons the body, so a consumer
can anchor a (sequence, host time) pair for the cost of one record and estimate
the ring head from the status time instead of paging from the oldest record
(the offline-programming telemetry window does this). The header-only C++ client
adds `telemetry(after)` and `telemetry_stream(after, callback, stop)` using
generated `protocol_generated.hpp` layouts, with no library dependency. The
generated TypeScript files declare the application contract and little-endian
`DataView` decoders for the header, records and axes; uint64 binary fields are
`bigint`. `make check-protocol` and `make check-api` check all three languages.

`tools/natspublisher` publishes the same header/body payload on
`rosie/rt-core.<host>.telemetry.v1` into **ROSIE_RT_CORE_TELEMETRY**, using the
configured host's existing subject-token normalization. Its independent
connection preserves a cursor across reconnects. It splits complete records to
the smaller of the broker's advertised max payload and 1 MiB, reports loss only
on the first split, and advances only after JetStream acknowledges a split.
An absent advertised payload limit is refused for telemetry.
An uncertain acknowledgement can cause duplicate replay; consumers deduplicate
by adapter/daemon incarnation and sequence. Incompatible incarnation changes
fail closed and require reconciliation/restart.

The default telemetry cadence equals the status cadence (500 ms by default) to
amortize publication overhead, configurable with `--telemetry-cadence` or
`ROSIE_RT_NATS_TELEMETRY_CADENCE`. Byte retention defaults to an unverified
512 MiB (536,870,912 bytes): a default 100,000-cycle ring window at 1 kHz needs
`100000*5392 + (100000/500)*312 = 539262400` payload bytes at 500 ms cadence;
the remainder allows split headers and broker overhead. Use
`--telemetry-max-bytes` or `ROSIE_RT_NATS_TELEMETRY_MAX_BYTES` to configure it.
The stream uses file storage, limits retention, discard-old and byte-only
retention (unlimited message count and no age limit). As with the v4 stream,
limits/subjects are repaired in place, incompatible storage/retention is refused,
and archive history is never deleted to repair configuration. The v4-shape
publication remains **compatibility only**, with its sunset under P6-14.


V2 preserves every V1 axis field at its original axis-relative offset and adds
the cycle-held fields below. Sizes and offsets are measured from generated
`protocol/control.json` layouts; both structs are aligned to eight bytes.
The full public record is 4,880 bytes. A compact ring/dump record stores only
`272 + axis_count*320` bytes. Mapping bytes include a 4096-byte header and an
eight-byte seqlock stamp per slot:

| Geometry | Arithmetic | Bytes |
|---|---|---:|
| 9 axes, 1 kHz, 300,000 ms | `4096 + 300000*(8+272+9*320)` | 948,004,096 |
| 2 axes, 1 kHz, 300,000 ms | `4096 + 300000*(8+272+2*320)` | 276,004,096 |
| Default: 9 axes, 1 kHz, 100,000 ms | `4096 + 100000*3160` | 316,004,096 |
| Default: 16 axes, 250 Âµs, 100,000 ms | `4096 + 400000*5400` | 2,160,004,096 (refused above 2 GiB) |

Default retention remains 100,000 ms. This layout's 16-axis, 250 Âµs mapping
exceeds the unchanged 2 GiB ceiling and requires a shorter configured retention.
The minimum dump byte budget is **529,680 bytes**: two 262,144-byte envelopes
plus one 5,392-byte record. Diagnostic layout changes do not alter Home identity.

The old V1 layout digest is
`ae42a2ad68df32eff0d7745b7791a3c8b8e2f629991bbdb600b57d303e77b59c`;
V2 is `1173439681672348f35f9652f2a4821d251638c70b60362ff59b37032a794b6b`.
There is no retained V1 dump decoder: use the matching old `rtctl` for old dumps.
Go ring, batch and dump readers reject a mismatch before interpreting records
with `TelemetryLayoutMismatchError`, naming `StoredDigest` and `ExpectedDigest`.
The batch envelope remains `TelemetryBatchHeaderV1`, version 1. Native IPC minor11,
first introduced for the since-removed current-reference capture, leaves this V2
telemetry layout unchanged; minor numbers are not reused or renumbered.
C++ and LCD readers also expose `TelemetryLayoutMismatchError` with both digests.
NATS treats that error as permanent on both direct batches and SDK observations;
the LCD reports it and stops its telemetry reader. Neither retries a layout refusal.

`torque_raw` is signed 0x6077 per-mille and is zero when `torque_valid` is false.
Torque validity requires a current verified mapping, mapped offset and fresh PDO.
0x1B04 and a6ec_1b02 both map torque; only 0x1B04 maps 0x606C/0x60F4. Existing
velocity/following-error/DI/absolute validity remains authoritative. `pdo_fresh`
also qualifies statusword, mode, DS402 and position; slave state is the cached
diagnostic observation used by this cycle. Coordinate validity and source validity
are separate. `target_velocity_valid` means 0x60FF is mapped and Send was attempted;
`submitted_axis_mask` and the signed Send result distinguish successful submission
from attempted output and do not prove drive receipt. Bus operation 0 means not
attempted, operation 5 is Send; a negative return code is failure.

`ext_valid_mask` bits 0..8 qualify, in order, position error, multi-turn low,
multi-turn high, bus voltage, load rate, IGBT temperature, motor temperature,
drive-not-ready bits and motor-not-rotating code. Each bit requires a mapped,
verified, fresh source; `ext_valid` means at least one bit is set. Invalid extended
values must not be interpreted. Cell I/O words retain the observed input and final
prepared output; `io_input_valid` gates input and bus submission gates output.
`io_configured=false` means no cell I/O. `cycle_jitter_ns` is signed wake minus
the scheduled target; `work_ns` retains its publication measurement contract.

`active_mode`, `state`, `active_traj_id`, `current_point_index` and other motion
names match `StatusMotionStateV1`. `control_generation` is the native fence;
`app_grant_generation` and `jog_generation` retain their existing grant/jog names.
`grant_active` copies the effective published decision, including the native
generation fence and lease validity; a watchdog fault cannot leave it active.
`grant_axis_mask` retains the published grant identity even when inactive.
`armed` and `axis_enable_mask` copy the same final values used by status.
`jog_session_open` is permission to accept input and becomes false during EndJog
or expiry ramping; `jog_ramping` distinguishes retained motion ownership.
`jog_input_deadline_host_ns` preserves the original published input deadline
during that ramp, rather than the internal stopping reservation's sentinel.
Collision fields use the `collision_` prefix followed by status watchdog field
names; quantities are 0 none, 1 torque, 2 following error, 3 both. The trip cycle
retains its final counters/peaks even after authority is revoked. For replay,
advance the evaluator only when `collision_samples` advances, apply the configured
strict thresholds/consecutive-cycle window and compare the trip count/quantity
and group fault masks. When drive following error is unavailable, use the previous
submitted wire target and coordinate/position with the configured wrap policy.

Commissioning JSONL and `dump-to-jsonl` retain `p`, `tp`, `sw`, `er`, `mfr` and
`af` for original fields; added fields use the schema spellings below. Validity,
armed/open/active flags are JSON booleans. Absolute `af[].ok` remains numeric.

`CycleCaptureRecordV2` (5392 bytes):

| Field | Offset | Size | Type |
|---|---:|---:|---|
| `t_ns` | 0 | 8 | `u64` |
| `seq` | 8 | 8 | `u64` |
| `cycle` | 16 | 8 | `u64` |
| `work_ns` | 24 | 8 | `u64` |
| `axes` | 32 | 4 | `u32` |
| `reserved` | 36 | 4 | `u32` |
| `cycle_jitter_ns` | 40 | 8 | `i64` |
| `active_traj_id` | 48 | 8 | `u64` |
| `active_command_seq` | 56 | 8 | `u64` |
| `jog_generation` | 64 | 8 | `u64` |
| `app_grant_generation` | 72 | 8 | `u64` |
| `control_generation` | 80 | 8 | `u64` |
| `configuration_epoch` | 88 | 8 | `u64` |
| `home_epoch` | 96 | 8 | `u64` |
| `grant_deadline_host_ns` | 104 | 8 | `u64` |
| `jog_input_deadline_host_ns` | 112 | 8 | `u64` |
| `jog_source_sequence` | 120 | 8 | `u64` |
| `bus_submission_return_code` | 128 | 8 | `i64` |
| `armed` | 136 | 4 | `u32` |
| `axis_enable_mask` | 140 | 4 | `u32` |
| `active_mode` | 144 | 4 | `u32` |
| `state` | 148 | 4 | `u32` |
| `current_point_index` | 152 | 4 | `u32` |
| `queue_depth` | 156 | 4 | `u32` |
| `last_event_code` | 160 | 4 | `u32` |
| `underrun_count` | 164 | 4 | `u32` |
| `stale_command_flag` | 168 | 4 | `u32` |
| `motion_done` | 172 | 4 | `u32` |
| `capability_flags` | 176 | 4 | `u32` |
| `jog_session_open` | 180 | 4 | `u32` |
| `jog_has_input` | 184 | 4 | `u32` |
| `jog_axis_mask` | 188 | 4 | `u32` |
| `jog_ramping` | 192 | 4 | `u32` |
| `grant_active` | 196 | 4 | `u32` |
| `grant_axis_mask` | 200 | 4 | `u32` |
| `safety_fault_mask` | 204 | 4 | `u32` |
| `execution_fault_reasons` | 208 | 4 | `u32` |
| `submitted_axis_mask` | 212 | 4 | `u32` |
| `bus_submission_operation` | 216 | 4 | `u32` |
| `pdo_fresh_axis_mask` | 220 | 4 | `u32` |
| `config_verified_mask` | 224 | 4 | `u32` |
| `home_valid_mask` | 228 | 4 | `u32` |
| `service_mode_axis_mask` | 232 | 4 | `u32` |
| `wkc_actual` | 236 | 4 | `u32` |
| `wkc_expected` | 240 | 4 | `u32` |
| `master_state` | 244 | 4 | `u32` |
| `io_configured` | 248 | 4 | `u32` |
| `io_input_word` | 252 | 4 | `u32` |
| `io_output_word` | 256 | 4 | `u32` |
| `io_input_valid` | 260 | 4 | `u32` |
| `io_armed` | 264 | 4 | `u32` |
| `io_reason` | 268 | 4 | `u32` |
| `ax` | 272 | 4608 | `CycleCaptureAxisV2[16]` |

`CycleCaptureAxisV2` (320 bytes):

| Field | Offset | Size | Type |
|---|---:|---:|---|
| `position` | 0 | 4 | `i32` |
| `target` | 4 | 4 | `i32` |
| `statusword` | 8 | 2 | `u16` |
| `error_code` | 10 | 2 | `u16` |
| `manufacturer_error_code` | 12 | 4 | `u32` |
| `absolute_value` | 16 | 64 | `i32[16]` |
| `absolute_valid` | 80 | 16 | `u8[16]` |
| `velocity_actual_counts_per_s` | 96 | 4 | `i32` |
| `following_error_counts` | 100 | 4 | `i32` |
| `velocity_actual_valid` | 104 | 4 | `u32` |
| `following_error_valid` | 108 | 4 | `u32` |
| `di_bits` | 112 | 4 | `u32` |
| `di_valid` | 116 | 4 | `u32` |
| `external_enable_active` | 120 | 4 | `u32` |
| `external_enable_valid` | 124 | 4 | `u32` |
| `coordinate_counts` | 128 | 8 | `i64` |
| `coordinate_epoch` | 136 | 8 | `u64` |
| `pdo_observed_time_ns` | 144 | 8 | `u64` |
| `collision_peak_following_error_counts` | 152 | 8 | `u64` |
| `collision_trip_count` | 160 | 8 | `u64` |
| `collision_samples` | 168 | 8 | `u64` |
| `collision_last_trip_peak_following_error_counts` | 176 | 8 | `u64` |
| `absolute_source_counts` | 184 | 4 | `i32` |
| `coordinate_reason` | 188 | 4 | `u32` |
| `target_velocity_counts_per_s` | 192 | 4 | `i32` |
| `collision_armed` | 196 | 4 | `u32` |
| `collision_torque_cycles` | 200 | 4 | `u32` |
| `collision_following_error_cycles` | 204 | 4 | `u32` |
| `collision_peak_torque_raw` | 208 | 4 | `u32` |
| `collision_last_trip_quantity` | 212 | 4 | `u32` |
| `collision_trip_sustained_cycles` | 216 | 4 | `u32` |
| `collision_last_trip_peak_torque_raw` | 220 | 4 | `u32` |
| `brake_state` | 224 | 4 | `u32` |
| `native_home_position_offset` | 228 | 4 | `i32` |
| `native_home_last_abort_code` | 232 | 4 | `u32` |
| `ext_position_error_counts` | 236 | 4 | `i32` |
| `ext_multi_turn_lo` | 240 | 4 | `i32` |
| `ext_multi_turn_hi` | 244 | 4 | `i32` |
| `max_abs_velocity_actual_counts_per_s` | 248 | 4 | `u32` |
| `max_abs_following_error_counts` | 252 | 4 | `u32` |
| `torque_raw` | 256 | 2 | `i16` |
| `ext_bus_voltage_raw` | 258 | 2 | `u16` |
| `ext_load_rate_raw` | 260 | 2 | `u16` |
| `ext_igbt_temp_raw` | 262 | 2 | `i16` |
| `ext_motor_temp_raw` | 264 | 2 | `i16` |
| `ext_drive_not_ready_bits` | 266 | 2 | `u16` |
| `ext_motor_not_rotating_code` | 268 | 2 | `u16` |
| `ext_valid_mask` | 270 | 2 | `u16` |
| `mode_display` | 272 | 1 | `i8` |
| `ds402_state` | 273 | 1 | `u8` |
| `torque_valid` | 274 | 1 | `u8` |
| `coordinate_valid` | 275 | 1 | `u8` |
| `coordinate_source_valid` | 276 | 1 | `u8` |
| `native_home_state` | 277 | 1 | `u8` |
| `slave_al_state` | 278 | 1 | `u8` |
| `slave_online` | 279 | 1 | `u8` |
| `slave_operational` | 280 | 1 | `u8` |
| `pdo_fresh` | 281 | 1 | `u8` |
| `target_velocity_valid` | 282 | 1 | `u8` |
| `ext_valid` | 283 | 1 | `u8` |
| `calibration_valid` | 284 | 1 | `u8` |
| `position_state` | 285 | 1 | `u8`: 0 unverified, 1 recovering, 2 trusted, 3 lost |
| `audit_reason` | 286 | 2 | `u16`: SampleInvalidity |
| `audit_last_verified_ns` | 288 | 8 | `u64` |
| `audit_started_ns` | 296 | 8 | `u64` |
| `audit_completed_ns` | 304 | 8 | `u64` |
| `audit_coherence_error_counts` | 312 | 8 | `u64` |

## Per-cell control link timing

`control.link_profile` is `lan` by default: lease ceiling 500000000 ns and jog
capture-age ceiling 250000000 ns. `internet` selects unverified starting ceilings
3000000000 ns and 750000000 ns. Explicit `control.max_grant_lease_ns` and
`control.max_jog_input_age_ns` override either preset, with unverified absolute
bounds of 10000000000 ns and 2000000000 ns. Lease overrides must be positive whole
milliseconds. Unknown fields, profiles, nulls, fractions and out-of-bound values
fail compilation. See [cell field schema](../protocol/cell-control.schema.json).

Presets set ceilings only. Describe publishes both effective ceilings.
Acquire's `requested_lease_ms` is capped by the cell; `lease_ms` in the grant
is the enforced result and persists through Renew and Stop. Omitted/zero requests
retain a 500 ms lease capped by the cell. Go and C++ clients measure Describe
round trip, request at least three round trips plus renewal cadence, and round
up to whole milliseconds. Their default cadence is at most one third of the
effective lease. A local operator on an internet cell retains a 500 ms request.
A capped lease that cannot cover measured RTT is refused typed
`control_session_stale` before motion. Renewal uses a dedicated connection,
independent of Status, Describe, bulk traffic and Stop.

A longer granted lease increases worst-case time to an unattended stop after
the operator's link dies. A longer capture-age limit permits a stale velocity
command to keep applying longer, bounded by its immutable deadline and the
existing jog expiry ramp. These are safety tradeoffs, not qualified stop-time
claims. Input expiry remains bounded by the current grant; renewal never
extends an already captured command's expiry.

`jog_clock` returns a host monotonic sample and incarnation for BeginJog.
Remote WSS calibration separately establishes a causal offset interval using
an echoed host transmit timestamp and a source sample taken after that receipt.
The earliest possible capture time, including drift, determines worst-case age;
no symmetric-delay assumption or subtraction of measured network latency is
used. Asymmetric calibration and transport jitter can make this conservative
age exceed physical one-way delay. The first-input wait and WSS idle deadline
use the cell input-age ceiling; clock uncertainty/drift qualification remains
required. Insufficient calibration budget is a typed refusal.

`TestDelayedAdapterLeaseRenewal` observes 60 renewals with 300 ms delayed replies
and a 3000 ms lease while Status is blocked; 500 ms is refused typed.
`TestRemoteJogTransportDelay` injects 120/280 ms asymmetric calibration delay,
a source clock 5000000000 ns ahead, and 120â€“400 ms input jitter. Conservative
ages are 240000001â€“520000001 ns and pass a 750000000 ns cell limit; 630/800 ms
input delays are refused as `jog_input_too_old`. Native jog policy tests retain
the mapped capture origin and immutable expiry.

The Deck measures its first Describe with the SDK's unverified 5000 ms bound.
Each later request allows three measured round trips with an unverified 200 ms
floor; WSS startup allows four such request budgets for its multiple exchanges.
Input lifetime is the published cell jog age, and status freshness covers the
receipt budget plus polling cadence, with the cell age as a floor. Begin identity
comes from the native WSS receipt without an extra HTTP observation. The Deck
discards the initiating sample after admission; later samples keep their original capture time.


### Brake transition qualification candidate â€” September 30, 2026

An explicitly configured native brake transition continues publishing existing BRAKE_WAIT_HOLD_DELAY during both Closing and Settling, and reaches BRAKE_APPLIED only after its bounded quiet qualification. Consumers must continue checking fresh advancing inhibited status and faults after Stop/Release; an accepted urgent Stop is not a queued command-result record or a physical brake receipt. No new command, generated type, shared-memory field or public response schema is introduced.

Native `brake_hold_diagnostic` retains the first trip's `cause` and adds `phase` (legacy/dormant/enabled/closing/settling/held). Legacy displacement trips retain `disabled_displacement`. Opt-in trips distinguish stale_feedback, invalid_configuration, observation_time, frame_changed, drive_state, stationary_target_cap, velocity_unavailable, velocity_cap, transition_corridor, transition_displacement_cap, settling_timeout and brake_feedback. These are native diagnostic strings, not new public refusal enums. `observed_ns=0` and cause `none` mean no trip; phase is trip context, not the current guard state. Generated API checks and existing native JSON decoding/legacy brake tests pass. Timer estimates still do not establish physical brake engagement.


### OLP measured-start planning and staging â€” September 30, 2026

This is an OLP/planner contract addition, not a native command or wire-ABI change. Taught planning accepts `require_measured_start`; when true, missing connected native observations reject the request rather than falling back to model reset. Server-owned `start_state.json` uses schema `rosie.measured-planning-start.v1`, exact described `joint_names`/`joint_values_rad`, robot/calibration binding, `coordinate_identity` (`configuration_sha256:configuration_epoch:home_epoch`), `observation_identity` (`adapter_incarnation:daemon_incarnation`) and string `observed_time_ns`. The archive manifest covers its bytes and the dense artifact preserves the capture metadata.

OLP Load rejects `planning_start_stale` before Acquire/Prepare when present capture metadata has a mismatched native identity, unsupported schema, wrong joint order/count or captured position inconsistent with the artifact's first sample. Dry Run preserves this metadata while stripping process outputs. Older offline artifacts without capture metadata remain governed by existing robot/Home/native admission rules. The desktop Play gate also requires fresh native feedback, current coordinate identity, exact program request digest and actual first-sample alignment on every described joint. Load remains nonmotion staging; no automatic connecting move is generated.

Cell manifests may explicitly select repository-relative `rt_core.drive_profiles` for a pinned diagnostic variant. Resolution rejects absolute/traversing/escaping paths and non-directories, requires an explicit configuration SHA for a variant and compares it exactly to compilation. Omitted selection retains `rt-core/config/drives`. This does not authorize changes to installed cell manifests.


### Native Home diagnostic evidence â€” September 30, 2026 (diagnostic launch flag removed October 2, 2026)

Candidate8 adds no public command, generated field or protocol-layout change. Internal terminal logs now distinguish successful submitted CW6/CW7/CW15 prefix frames from the CW31 trigger/completion window, including missing/failed observations. An unavailable stop acknowledgement is unavailable diagnostic evidence, never permission or Home success. A Home that still fails completion after uninterrupted successful CW31/mode6 submissions remains failed; clients must continue honoring the failed receipt and invalid Home.

The native runtime flag `--native-home-diagnostic-axis N` is disabled by default and selects one zero-based axis. It is not a client request or machine-model/configuration identity change. A selected HM35 transaction may read one three-object batch only after actual trigger submission:6062:00 I32,2040:01 I16 and2040:02 I16. Diagnostic receipts have separate timestamps and can be invalid/late; they must not be treated as simultaneous measurements, a safety-speed certificate, or a substitute for Home acceptance. Finite count and unchanged deadline do not make synchronous kernel uploads cancellable. Existing cyclic permission loss/Stop inhibition remains independent. No diagnostic option permits a client to bypass a fault, widen a guard, or claim physical readiness.


### OLP measured-start forwarding and replan recovery â€” 2026-09-30

The application-only seam-worker JSON boundary accepts optional `start_state` and forwards its full immutable measured-start capture to the existing Python packer. Missing/null preserves offline preview behavior; unrelated unknown fields still fail strict decode. This changes no native protocol or motion permission.

A Load rejected as `prepare_program` / `planning_start_stale` prescribes replanning. Fresh measured-start capture therefore admits that exact retained application refusal while continuing to reject unrelated error text, native faults, inhibition, uncertain Stop, invalid coordinates, active motion and nonstationary observations. It neither clears the refusal nor issues Reset, Home, Arm or motion. Load/Play still evaluate the new capture and current native identity.

The Home UI disables and explains a native `restart_required` recovery fault, including brake_hold. Its action handler also refuses the request. Other supported reference-recovery Home policies remain available; the native controller remains authoritative. Jog captions use a valid configured robot model, and inspector program-name Enter commits through the same rename function as normal change.

An acknowledged ordinary `operator_stop` also permits read-only start capture: the app/native reason must be operator_stop, any existing app session must report stopped/operator_stop (a read-only connection can have session:null), native StopAckId must be nonzero, and grant/armed/axis-enable must be explicitly zero. This exception preserves every freshness, stationary-window, coordinate, native-fault, inhibition and uncertain-Stop check. Other stop/error reasons and missing acknowledgement remain refusals. It does not reacquire control or resume motion.

Simulator computer-use validation on appb6e9890f/UIba238d6b passed both sessionless acknowledged Stopâ†’Plan and post-playback acknowledged Stopâ†’Plan with an existing stopped session. The intervening Arm/Load/Play completed. These are application/native-simulation checks; they do not qualify physical Home, brake behavior or robot motion.


### Optional CSP preconditioning diagnostic â€” September 30, 2026 (superseded October 2, 2026: see Native Home transaction and removed launch flags)

Candidate9 adds no public operation, protocol field or generated-client change. The native launch flag `--native-home-csp-precondition-axis N` is default off and selects one zero-based A6 HM35 axis. Its stationary CSP stage executes only inside an already authorized NativeHome request, before the existing Shutdown500ms/HM35 sequence. Configuration/profile identity and public Home semantics are unchanged; completion of the CSP stage is not Home, Arm or motion readiness. Unsupported axes/profiles, missing brakes or a held tolerance other than16 reject rather than silently enabling a different policy.

A fixed captured raw target/frame and native cyclic authority, displacement, velocity, brake, feedback and successful-submission checks bound the stage to at most2s within the original request. CW0 plus observed disabled/held feedback must precede continuation. No mailbox read is added during enabled CSP. A failed stage reports through the existing failed NativeHome receipt; clients must retain ordinary failed-Home handling and must not automatically rearm. The separate optional HM mailbox diagnostic and its non-cancellable-kernel-I/O limitation remain unchanged. Candidate9 is offline-tested source only until separately documented physical deployment and qualification.


Candidate9 selected-J4 follow-up does not extend this interface: axis3 reuses the same explicit launch flag and ordinary Home request, with only the selected Home bit permitted. Composed tests prove disabled unselected axes and no fabricated Home bits; they do not confer readiness on the real robot. CSP stage success followed by an HM35 timeout remains a failed Home result.



### Candidate10 opening diagnostic configuration â€” September 30, 2026

Candidate10 is uncommitted, uninstalled source and introduces no public command, generated-client field or wire-ABI change. Optional machine-axis `brake_opening_guard` contains `max_displacement_counts`, `max_velocity_counts_per_s` and `timeout_ms`. Values are included in compiled configuration identity, so deployment and application bindings must match the new snapshot. Omission preserves the existing policy; malformed values, absent brake/closing policy and invalid timing or wrapping bounds are rejected. The existing selected-axis CSP launch option and ordinary authorized Home request remain the entry points.

The opening phase permits only a bounded fixed-target commissioning transition. Mode6 post-enable monitoring continues from the original anchor for at most 2 s until the actual authorized CW31 submission; opening qualification is never Home success. Stop and permission loss still inhibit cyclic outputs even while the worker is delayed, and failed Home remains failed. Applications must not clear faults, synthesize Home, automatically rearm or treat timer-estimated brake state as measured engagement. The prior Held16 value and 6553 counts/s cap were unverified resting/diagnostic assumptions, not certified enable limits; the new approximately 0.05 degree / 2 degree-per-second envelope is also diagnostic and physically unqualified.

Candidate12, September 30, 2026: native metrics gain a diagnostic `independent_encoder_observation` object with acquisition/configuration/timestamp provenance. It is not Home/Restore authority: `independent_anchor_source.valid` retains its current-frame requirements, and no public protocol, Go adapter or OLP executable change is required. The selected-axis physical test uses the diagnostic observation only during its exact active Home transaction and continues requiring strict anchor validity elsewhere. Home's opt-in closing witness preserves the existing finite caps and settled Held16. Composed qualification is pending.


### Candidate13 ordinary Arm qualification â€” September 30, 2026

Candidate13 changes no public command, native IPC layout or generated client contract. Existing Enable/Arm requests still require valid Home, configuration, coordinates and application authority. Only the explicit machine-axis opening policy gains a distinct, identity-bound ordinary Arm stage: actual successful CW15, immutable target/frame, finite acquisition/release bounds and existing BrakeMachine release qualification must precede motion readiness. Stop/disable/lease loss continues to inhibit immediately. The eventual closure witness remains independent of later normal targets or grant loss, requires causal submitted CW0 and inactive/quiet proof, and ends in unchanged Held16. No automatic retry, Home fabrication or service-based motion bypass is introduced. The diagnostic encoder observation remains read-only evidence, not Home/Restore admission.

Ordinary Enable/Arm/Disable increments audit generation; an asynchronous absolute read crossing that boundary is invalid even when configuration and command-frame epochs remain unchanged. `Epoch` is checked before torn/range conditions, so clients must not relabel that sample as coherent or use its counts as a new anchor.


### Candidate15 diagnostic proof publication — September 30, 2026

Candidate15 now freezes each axisâ€™s sampled absolute proof and its read-success flag before taking the publication clock. Raw observation, anchor diagnostic and the existing Restore offer reuse that immutable per-axis proof. This is not a globally atomic metrics snapshot or simultaneous acquisition of all axes. Existing configuration/frame/fault admission and genuine future, stale, age, torn, range and source-validity rejection remain authoritative; motion policy, thresholds and protocol are unchanged.


### Retained jog mode and measured-start capture — September 30, 2026

Measured-start capture permits retained jog mode 3 only when state is stopped/done, jog input is closed, Stop is acknowledged, grant and lease are released, and outputs are explicitly disabled; existing fault, Home, identity and multi-sample position-window checks remain.


### Candidate21 startup alarm and candidate22 recovery correction â€” September 30, 2026

Candidate22 separates a never-enabled startup alarm from an established brake lifecycle. It retains the original raw anchor, frame and freshness checks and Held16 displacement bound throughout Fault/NotReady recovery, reports brake UNKNOWN, and never permits motion. Any submitted enable/service sequence or observed Operation Enabled permanently ends this startup exception. Established opening, enabled, closing and held failures retain their existing latch and recovery rules. An explicit public ResetFault may produce one selected-axis CW128 receipt only with current startup evidence, inhibited outputs, current verified feedback/configuration and valid caller authority. The command retires that authority; subsequent cycles return CW0. Stop, changed generation, expired original grant deadline, changed configuration/Home epoch or loss of evidence cancels the receipt. No global recovery-table bypass, automatic reset, enable, anchor rewrite or threshold change is introduced. A submitted reset is not alarm clearance: fresh fault-free status, drive error readbacks and readiness must be independently verified before any Home or motion.

### Drive alarm auxiliary validity — September 30, 2026

Healthy status may report `drive_alarm_aux_valid=false`: the native alarm helper reads manufacturer detail for nonzero drive-error events and clears auxiliary validity when cyclic error603F is zero. Admission must require current healthy drive/fault evidence and reject any valid nonzero auxiliary alarm; it must not require a historical auxiliary alarm read to have occurred. No API or wire-schema change is made by candidate22.


### Candidate24 explicit reference recovery â€” September 30, 2026

Native IPC 1.12 command 267 requires a single opted-in axis, current authority, fault-free inhibited state, fresh quiet bracketed independent absolute evidence, unchanged drive offset and full signed command/encoder deltas individually within 9102 counts and agreeing within the existing 16-count tolerance. No modulo alias is accepted. Saved L0 produces the restored logical position, which must remain inside hard limits. The immutable receipt remains historical; new binding fields explicitly report `independent_absolute_delta_v1`, both deltas and the exact native command sequence. Public API success requires fresh (at most 250 ms) matching status, current configuration/Home binding, exact immutable receipt/drive/coordinate identity, retained authority and unchanged anchor storage. Mixed publications are retried within the existing eight-second limit; failure requests Stop. No drive enabling or new zero is part of this operation.

Offline gates passed: six native suites and final composed cycle/worker/simulator tests; focused public API, SDK and anchor-store regressions (stale/future status, old sequence/policy, mixed publication, cancelled/expired authority, missing store/capability, identity and delta refusal); exact compiler scope, all 28 configuration parity cases and template coverage; candidate24 OLP cell/compiler resolution. Actual isolated simulator capture/persist/restart tests passed in 17.323 s: +17 and âˆ’123-count moves restored the original saved zero; over-cap 9103, mismatched and opposite encoder deltas were refused; normal moved-pose restore still refused. Anchor bytes remained unchanged and outputs stayed inhibited. Simulator threads were observed FIFO20/CPU2. Native handshake and remote principal fences passed; the external SDK helper passed pure tests and build only, with no helper execution. Initial broad-regex missing-simulator harness failures and initial test/compiler corrections remain preserved; they are not physical results.


### Candidate25 explicit legacy-anchor recovery â€” September 30, 2026

Candidate25 extends only explicit, opted-in command 267 to unchanged legacy v3 anchors. It never calls Home or captures a new zero. The v3 saved logical offset remains the commissioned drive-derived offset, not negative current position; v5 retains its original capture receipt. Native proof is an exactly-one public `VerifiedDeltaRestoreReceipt`: `anchor_delta_restore` for v3, `reference` for v5. The v3 attestation reports saved C0/E0/offset/607C/Home epoch and new command sequence, binding epochs/time/PDO receipt and full signed deltas. API success requires fresh causal native qualification and unchanged anchor bytes. Earlier axes retain their own bound Home epochs as later serialized restores advance the global epoch. Normal and automatic restoration are unchanged.

Focused Linux API/SDK/store tests, exact compiler scope, 29-machine parity/template coverage and the candidate25 OLP resolver passed. Native minor 1.13 composed qualification passed separately. Full public-contract qualification passed with CGO_ENABLED=1 (`make check-api`, 44.845 seconds). Final isolated process qualification passed in 36.096 seconds against minor-1.13 simulator SHA-256 `4ea20a2f1e98822aac77e398f4290cd9a6aec32bf82079b0aa5e37856e43bd12`: genuine v3 Home saved C0=16 with calibrated L0=0; a daemon power cycle cleared retained Home bits; explicit +17 and in-frame âˆ’8-count restoration preserved that offset and first public Arm held the present target. The âˆ’123-count wrapped-PDO seam, over-cap and contradictory/opposite deltas were refused. V5 positive/negative restoration, normal restoration semantics, handshake and remote-principal fences also passed. All observed simulator realtime threads used FIFO20/CPU2, and cleanup found no remaining simulator. Runtime inputs match checkpoint `8ce1900c1d9dd109516ce5434eb2ea1179a3a5e8`; the final fixture-only refinement retains the wrapped-seam refusal and tests the in-frame negative route directly, before ordinary RestoreAnchor can validly restore a within-tolerance pose. Original failed fixture attempts remain preserved. Final process receipt SHA-256 `c6d9fee601b35e688fb8479d2151812ff64376075ed3d57006e6795ce047c8a0`. Initial compiler fixture errors, omitted resolved battery fact and missing reflected response fixture are preserved as failed offline attempts and corrected forward. Coupled native/API/direct tools and linked OLP rebuilds are required; UI18 remains unchanged. No deployment, restoration, powered motion or parking acceptance is claimed. P8-06 remains Backlog with its existing owner and dependencies.


### Candidate25 UI contract correction — September 30, 2026

Forward correction to the earlier UI18 retention assumption: unchanged UI source does not imply a compatible bundle. `manifestFromRTCore` pins the generated public `CapabilitiesDigest`; UI18 embedded the earlier digest and correctly refused candidate25's description.


### Candidate25 UI display limitation — September 30, 2026

UI display limitation found during candidate25 testing: `REAL OUTPUT: ENABLED` is derived from physical motion authority, not process I/O or Dry run state. The label does not establish that process outputs are active. A truthful copy correction is proposed; no display runtime change is included in the above activation.


### Remote jog reply framing â€” October 1, 2026

WebSocket replies now combine header and payload in one write; partial/error writes close the stream without replay. There is no schema, protocol, SDK framing or clock-budget change. Calibration-only connections still use generation zero and confer no motion admission. Focused frame/deadline/error race tests passed locally; the deployed path and continuous-hold behavior remain pending. The R4 probe's 39/493 intervals above 20 ms are preserved as a failure, not reclassified by this implementation.


### Qualified inhibited brake-fault recovery — September 30, 2026

The hardcoded brake_hold restart requirement is replaced by explicit inhibited reset after continuous fresh, fixed-frame quiet proof using existing brake transition/held limits and hold+settling dwell. The actual latch is reset to Held; Home is preserved only with coordinate trust. No automatic clear, Arm, brake output, new threshold or physical-feedback claim. Exact trip speed/effective caps are now retained.

### Repeated Go SDK hold setup â€” September 30, 2026

`NewRemoteJogSession` consumes one WSS reservation per hold and immediately wakes its connection keeper. If another press arrives before replenishment, setup prepares an authenticated read-only `/v1/jog/ingress` connection within the existing deadline before upgrading or submitting Begin/input. A pre-upgrade preparation/dial failure is `ErrConnectionUnsent`; cancellation remains cancellation. No failed motion request is replayed. This replaces the former missing-reservation failure between successive holds. Implementation `963f1c12`; Linux repeated-session/cancellation/race and OLP integration tests pass. Physical repeated-hold acceptance remains pending.

### Uniform joint-angle brake, hold and restore bounds â€” October 2, 2026 (offline change)

Working tree only; not committed, not deployed, not physically qualified (GitHub issue #158). Nothing is joint-specific: each bound below is one joint-unit value, converted per axis by its gear ratio (131072 Ã— gear / 2Ï€ counts per joint radian) and floored, so no axis admits more than the declared angle. Count and joint-unit forms of the same bound cannot be mixed, and a value that floors below one count on any axis is refused. Each value is the largest previous per-joint equivalent (J4 or J6); none was measured on the loaded arm.

- Held tolerance and encoder/PDO agreement: the A6 profile declares `hold_displacement_tolerance_rad`, `maximum_feedback_error_rad` and `maximum_anchor_error_rad` = 4.2611e-5 rad (the former 16 counts at J4's 18:1), giving 16 counts on J4, 44 on J6 (50:1) and 88 on J1â€“J3/J5 (100:1).
- Start-alignment floor: native `kIndependentEncoderStartAlignmentFloorRad` = 1.332e-4 rad (the former 50 counts at J4).
- Brake guards on all six joints: the Rosie 1430 cell machine configurations declare one machine-level `brake_transition_guard` (`max_displacement_rad` 0.050Â° = 0.0008726646 rad, `max_velocity_rad_per_s` 3.0Â°/s, target stationary 100 ms, settling 100 ms, timeout 500 ms) and one `brake_opening_guard` (0.050Â°, 8.0Â°/s, timeout 500 ms). Natively that is closing 1820 counts / 109226 counts/s on 100:1, 327 / 19660 on J4 and 910 / 54613 on J6; opening 1820 / 291271, 327 / 52428 and 910 / 145635. Robot-bound cells refuse per-axis guard blocks, and a machine-level block must use joint units. Every axis still satisfies the native checks: displacement â‰¥ held tolerance, twice the displacement below the wrap period, and a mapped 32-bit `velocity_actual` (the A6 fixed TxPDO 0x1B04 used by J1â€“J3 already carries it). Brake detections remain warnings.
- Stationary CSP Home precondition (`--native-home-csp-precondition-axis`): its caps are now joint units too (held 4.2611e-5 rad, 0.050Â°, 1.0Â°/s, opening 2.0Â°/s), so any joint qualifies on the same angle instead of only J4 on 16 counts. The same gear-aware held-tolerance check gates the brake-opening contexts, which now apply on all six axes. The canonical opening rate (8.0Â°/s) exceeds this diagnostic's 2.0Â°/s opening cap, so the diagnostic refuses the canonical machine, exactly as it refused the earlier 8Â°/s J4 trial; a commissioning run needs an explicitly narrower opening envelope.
- Manual restore (verified-delta, command 267) has no cap. `maximum_delta_counts` is removed from the compiler (refused by name), native profile, adapter and public schema. An HM35 (v3) anchor restores to any position within the axis hard limits when the full signed command and independent-encoder deltas agree modulo one joint revolution (131072 Ã— gear counts) within the anchor tolerance, 607C is unchanged and the axis is fresh, authorized and inhibited. A wrapped-PDO seam crossing uses the same once-per-revolution reconciliation as the retained-Home restore, and the hard-limit check applies to the reconciled position. Schema/capabilities digest `64dad2af2ad1f3bb721eb28be72c538b0248756a6e2c2f66456e3a57dfc00b8e`; the OLP UI and Steam Deck client must be rebuilt against the regenerated clients.

### Native Home transaction and removed launch flags â€” October 2, 2026

No public operation, generated field or protocol layout changes. Home again submits exactly origin/main's vendor-confirmed HM35 transaction, without the earlier Shutdown or pretrigger dwells, so a successful Home completes without those 500 ms stages; failed-Home receipts and invalid-Home handling are unchanged. The native launch flags `--native-home-csp-precondition-axis` and `--native-home-diagnostic-axis` are removed and refused as unknown arguments; the CSP preconditioning and diagnostic-read sections above are superseded. Cell manifests have no HM35 order variant to select. Offline source only; not deployed.


### Manual I/O and stationary sensor waits (candidate, 2026-10-17)

`io_test` takes `label`, `value`, `source_sequence` and `begin` under the normal
session/generation fence. `begin: true` denotes a new explicit ON, not a
heartbeat; renewals cannot resurrect OFF or expired output intents. It never
arms motion. Since `bd322d01c` (2026-10-19), torch-class outputs are accepted
for bounded manual terminal testing; this does not qualify programmed weld
markers. Motion must remain disarmed. Application-level manual testing uses
an explicit short-lived session; closing the test revokes I/O only and does not
send a motion Stop as an error-recovery side effect.

A process marker's optional `timeout_ms` and `stable_ms` select a configured
input wait instead of an output action. Both fields participate in the marker
digest. Input-only programs retain envelope binding without arming outputs.
Prepared Point wait metadata is independently checked by native admission.
See [CELL_IO.md](CELL_IO.md) for execution and qualification boundaries.


### October 18 candidate: scoped input monitors

Program `monitor` / `end_monitor` events bind named inputs into immutable
`monitor_mask` / `monitor_values` at each trajectory sample (TrajectoryPointV1
320 bytes in protocol minor 15). Pairing, configured names and exact sample
binding are checked before preparation. During execution the native cycle checks
validity and required value, with at most sixteen comparisons and no allocation.
An invalid/mismatched monitored input raises the I/O wait fault and clears output
authority. End monitor removes that requirement. These are operational process
checks, not safety-rated interlocks. Dry-run/Trace strips monitors as well as waits
and output actions. Output authority is requested only for actual output markers.


### Controller-owned indicators, October 18 candidate

Optional configured indicator outputs are native-owned, independent of application
grants. Manual and prepared process writes are refused; Describe exposes class
indicator and its timing policy, while status retains commanded/feedback evidence.
See [Cell I/O](CELL_IO.md#october-18--controller-owned-powerarm-indicators-candidate).
