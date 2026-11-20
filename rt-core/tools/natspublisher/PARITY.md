# RTCore bench publisher parity and operator README

This is the task's single permitted documentation file. The publisher uses only
`sdk/control.Client.Describe` and `Status` (`sdk/control/operations.go:7`). It
observes an existing rt-control instance; it does not acquire, renew, release,
stop, jog, enable, home or execute. Logical observations and execution state are
also preserved under the `status.rt_core` extension. Batch records retain the
legacy reader fields without duplicating that full public observation.

## Deployment and bounded behavior

`make control` builds `build/rt-natspublisher`. Run it with `--socket` pointing at
the public Unix socket, `--nats-url nats://HOST:PORT` and `--host CONCRETE_PEER`.
The equivalent deployment variables are `ROSIE_RT_NATS_URL` and
`ROSIE_RT_NATS_HOST`. `--cadence` / `ROSIE_RT_NATS_CADENCE` sets a positive whole
millisecond interval (for example `500ms`), capped at an unverified 5000 ms to
stay within the existing pair reader's freshness window. Supply URL and host to `host/install.sh` to atomically install
`/etc/rosie-rt-core/natspublisher.env`; `rt-natspublisher --print-env` validates and
emits that environment without contacting either service. Missing configuration
does not invent a host or URL. Invalid configuration preserves the previous file.
The host comes from the selected deployment peer, never `os.Hostname()` or a role
alias. Topic spelling follows `bridge_commands.go:2139`; the concrete-peer and
role checks follow `bridge.go:833` and `bridge.go:849`.

`tools/package-runtime.sh` always includes the unit and includes the binary when
it was built. A missing binary does not fail core packaging or installation;
the unit's executable condition keeps it inactive. If deployment variables are
supplied without the binary, installation reports the missing optional component
and preserves the existing environment. Reinstall with the binary to configure it.
`rosie-rt-natspublisher.service` is disabled by default, including installation
with `--enable`. Its transient service identity belongs only to the public client
group. **Two publishers on one subject are a fault. Stop the v4 NATS bridge before
the operator enables this unit.** The unit conflicts with
`robot-v4-rtcore-nats-bridge.service`; that conflict cannot police a manually
launched bridge or another host. The fleet bridge runs on the RevPi while this
publisher runs on the Jetson. The unit comment and installer environment-write
warning state this explicitly. Stop this publisher before restoring the bridge.
Before publishing, each connection subscribes to this host's status subject,
flushes the subscription and observes one configured cadence. Any live message
on that subject refuses startup with `ErrPublisherActive`, even a malformed one;
archived messages and other hosts' subjects do not count. The subscription is
removed and flushed before publication. This detects an already flowing peer;
it is not a distributed lock and cannot exclude simultaneous starts or peers
whose cadence is longer than the observation window. Operator cutover remains required.

Status and batch cadence defaults to **500 ms**, matching the deployed
`ROBOT_V4_RTCORE_NATS_BATCH_MS=500` of the installed v4 cell configuration,
rather than the bridge's 100 ms code default. The pair reader uses a **5000 ms** status freshness
window (`pair_config.go:48`). Each batch contains one public status sample, not
the v4 high-rate ring history. `interval_ms` describes the configured cadence;
source timestamps and sequence gaps describe actual acquisition. This does not
claim the old ring's sampling rate or preserve every intervening native event.

Unverified resource policies are one pending sample and one in-flight sample,
an **8 KiB** NATS line cap, **1 MiB** NATS message cap, and **500 ms** absolute
network-operation and public-poll deadlines. The SDK separately caps public HTTP
responses at **8 MiB** (`sdk/control/client.go:224`). New samples replace the
pending sample. The cumulative `dropped` count counts replacement, failed poll,
stale sample and incomplete/uncertain publication once per sample, not bytes or
NATS subjects. It appears on both envelopes and their sampled status/record.
It resets on process restart; `sequence` also resets, and native/adapter
incarnations remain explicit under `rt_core`. It does not count timer ticks
coalesced during a slow public poll or replace native `buffer_health.ring_drops`.

Each NATS publication requires a JetStream acknowledgement. A failed/uncertain
sample is discarded, never retried as history. The broker may have retained a
message whose acknowledgement was lost; status and batch are not transactional.
Reconnect uses an unverified **250 ms** initial delay, doubles to **4000 ms**,
and resets after a complete status/batch pair. Connect/provision failures retain
only the latest pending sample. The publisher rechecks source age before each
send, dropping samples older than **500 ms**. `updated_at` is reconstructed from
the native monotonic sample age, so polling frozen data cannot renew its age.
The installed Unix client and daemon share the Linux monotonic clock.

Every poll rechecks the generated public contract version/capabilities digest
and native protocol version (`adapters/rosie/control/api_generated.go:12`,
`ipcclient/protocol_generated.go:10`). A mismatch terminates. Status/describe
configuration and backend disagreement suppress sampled evidence.

Each transition emits one escaped, one-line key=value record to stderr with
`component=rt-natspublisher`, `event`, `transition`, `publisher_state`,
`last_error`, `reconnects` and `drops`. Repeated dial failures or continuous
drops do not emit a record per attempt/sample. Drop recovery requires a complete
status/batch acknowledgement with no concurrent loss. The status envelope adds
the same four diagnostic fields: states include `connecting`, `dial_failed`,
`handshake_refused`, `stream_rejected`, `checking_publisher`,
`publisher_check_failed`, `publishing`, `publish_failed`, `dropping`,
`source_unavailable`, `refused`, and `stopped`. `last_error` retains the most recent
incident after recovery. `reconnects` counts dial attempts after the initial one;
`drops` aliases the existing cumulative sample `dropped` counter. Counters reset
on process restart. A reachable broker receives diagnostic status with `status=null`
when observations are unavailable/stale, without a fabricated batch or refreshed
native evidence. Broker outages themselves remain observable through stderr and
the reader's existing freshness check; recovered publications carry the counters.

The CLI maps wrapped sentinels to the following exit statuses. The unit uses
`Restart=on-failure` and `RestartPreventExitStatus=2 10 11 12 13 14 15`, so
permanent refusals require configuration correction and an explicit restart.

| Exit | Meaning |
|---|---|
| 0 | Clean cancellation/success. |
| 1 | Other process failure; eligible for service restart. |
| 2 | Invalid local flags/deployment configuration. |
| 10 | `ErrContract`: public/native contract mismatch. |
| 11 | `ErrAuthRequired`: broker requires authentication. |
| 12 | `ErrTLSRequired`: broker requires TLS (takes precedence when both are required). |
| 13 | `ErrJetStreamUnavailable`: endpoint does not advertise JetStream, or API has no responder/service. |
| 14 | `ErrStreamConfig`: incompatible immutable settings or refused stream management/configuration. |
| 15 | `ErrPublisherActive`: live traffic on this identity's status subject during startup check. |

The stream is `ROBOT_V4_RTCORE`, subject filter `robot/v4/rtcore.>`, retention
`limits`, storage `file`, discard `old`, maximum **250000 messages**, **268435456
bytes**, age **600000000000 ns**, exactly `bridge.go:341`. Existing limits are
repaired with UPDATE and checked in the response. No stream is ever deleted.
Incompatible name, storage or retention and explicit stream API configuration
refusals return a permanent sentinel without replacing archive history. All
retention mismatches are conservatively refused, including changes some newer
brokers permit. Transport failures retry with bounded backoff. NATS/JetStream must be available
with permissions for these stream APIs, the two publication subjects and a
private `_INBOX.rtcore.*` reply subscription and this host's status subscription.

Neither `robot/v4/go.mod` nor `motion-server/v2/tests/composed/go.mod` requires a
Go NATS client. The implementation therefore uses the small core protocol subset
CONNECT/SUB/UNSUB/PUB/MSG/HMSG/PING/PONG and JetStream JSON request/reply.
Header support requests immediate 503 no-responder replies, so unavailable
account-level JetStream does not silently time out forever. It accepts an
explicit unauthenticated `nats://host:port` endpoint. Credential-bearing URLs,
TLS URLs and servers requiring authentication/TLS are refused; no fallback or
credential inference is attempted. NATS credentials/TLS provisioning is outside
this bench implementation.

`robot/v4/rtcore.<host>.telemetry.pattern` is **not published**. The v4 detector
consumes raw target counts, current/voltage, command history, HM35 home anchors
and command round-trip evidence (`bridge_telemetry.go:1427`). Public status has
no equivalent complete input set. A polled native motion event is not that
history. Stream retention still covers pre-existing pattern messages.

## Field parity

V4 paths below are relative to `robot/v4/realtime-core/natsbridge/` unless given
in full. RT paths are relative to `rt-core/`. `C` means
`GET /v1/status data.core`, defined in `ipcclient/client_linux.go:58`; `M` means
`data.motion` (`ipcclient/client_linux.go:175`); `D` means `GET /v1/describe`
(`ipcclient/description.go:40`). The public envelope and logical axes are in
`adapters/rosie/control/api_generated.go:420`. The mapping implementation is
`payloads` in `tools/natspublisher/publisher/payload.go`.

V4's `StatusMessage.Status` and batch `Records` are maps, not closed structs
(`bridge.go:285`, `bridge.go:303`). The table enumerates all envelope fields,
all fields emitted by its ring decoder/status fallback, bridge annotations, and
the additional concrete status-reader fields. `ReadStatusJSON` (`bridge_telemetry.go:515`)
also passes arbitrary runtime JSON keys through; unspecified extensions have no
closed field contract and are not invented here. A null parent means all its
listed descendants are unavailable, not a zero-valued lifecycle. The batch
record includes the same nulls and validity indicators as status.

| Envelope field | V4 definition | RT source / published meaning |
|---|---|---|
| `schema` | `bridge.go:286,303`; `bridge_telemetry.go:29,60` | Publisher metadata: exact `robot.v4.rtcore.status.v1` or `robot.v4.rtcore.telemetry.batch.v1`. |
| `stream` | `bridge.go:287,304` | Publisher metadata: `ROBOT_V4_RTCORE`. |
| `subject` | `bridge.go:288,305,1114,1118` | Deployment host, v4 token rule, `.status` or `.telemetry.batch`. |
| `host` | `bridge.go:289,306` | Explicit deployment peer, trimmed as in v4. |
| `component` | `bridge.go:290,307` | Publisher metadata: `rtcore`. |
| `test_session_id` | `bridge.go:291,308` | Absent; null. No v4 test-session owner on the public API. |
| `published_at` | `bridge.go:293,309` | Publisher UTC serialization time, RFC3339Nano. |
| `status` | `bridge.go:310` | Mapping in the next table. |
| `recent_commands` | `bridge.go:299,311` | Absent; null, not an empty/completed command log. |
| `sequence` (batch) | `bridge.go:292` | Publisher poll sequence, starts at 1 per process; gaps are retained. |
| `source_monotonic_ms` (batch) | `bridge.go:294` | C.`time_ns` / 1000000 (`ipcclient/client_linux.go:67`). |
| `interval_ms` (batch) | `bridge.go:295` | Configured cadence, default 500 ms policy above. |
| `count`, `records` (batch) | `bridge.go:296,298` | One sampled status record. |
| `dropped` (batch; additive status envelope field) | `bridge.go:297` | Cumulative publisher sample drops, defined above; `Publisher.Dropped` in `publisher.go`. |

| Status / record field | V4 field site | RT source or absence |
|---|---|---|
| `name`, `state` | `bridge_telemetry.go:835` | Compatibility identity `robot-v4-rtcore`, `running` only after a fresh successful public observation. Process liveness, not safety readiness. |
| `mode` | `bridge_telemetry.go:837` | C.`backend` = ethercat gives `ethercat_live`; otherwise null. |
| `rtcore_mode`, `runtime_mode`, `simulation_mode` | `bridge_telemetry.go:644` | C.`backend`: ethercat -> live/false, simulation -> sim/true (`ipcclient/client_linux.go:68`). Unknown backends are refused. |
| `ethercat_live` | `bridge_telemetry.go:849` | C.`backend` = ethercat; identifies backend, not link health. |
| `servo_control_enabled` | `bridge_telemetry.go:631,838` | Absent v4 servo-control state; null. Native enable intent is not proof of this state. |
| `servos_armed_requested` | `bridge_telemetry.go:632,839` | C.`armed` != 0, the native accepted arm latch (`ipcclient/client_linux.go:80`); no v4 request/reply queue is inferred. |
| `active_jog` | `bridge_telemetry.go:630,840` | Fresh M.`mode` = generated `MOTION_MODE_JOG`; null if motion sample stale (`ipcclient/client_linux.go:176`). |
| `arm_disarm_requests` | `bridge_telemetry.go:633,844` | Absent v4 cumulative request counter; null. |
| `axis_error_mask` | `bridge_telemetry.go:634,845` | Nonzero C.`axes[i].error_code` sets bit i, only with fresh per-axis PDO (`ipcclient/client_linux.go:132`). Otherwise null. |
| `axis_fault_mask` | `bridge_telemetry.go:635,846` | C.`axes[i].statusword` fault bit 3; all axes must be fresh (`ipcclient/client_linux.go:131`). Native safety faults remain separate in `rt_core.core`. |
| `axis_enable_mask` | `offline-programming/v1/internal/connected/rtcore.go:30` | C.`axis_enable_mask` unchanged (`ipcclient/client_linux.go:81`). |
| `configured_axis_count` | `offline-programming/v1/internal/connected/rtcore.go:28` | Number of C.`axes`, validated against D.`axes`. |
| `ds402_enabled_axes` | `offline-programming/v1/internal/connected/rtcore.go:29` | Count of fresh C.`axes[].statusword` satisfying CiA402 `(word & 0x006f) == 0x0027`; null if any axis lacks fresh PDO. |
| `home_valid_axis_mask` | `bridge_telemetry.go:847` | C.`home_valid_mask` (`ipcclient/client_linux.go:73`). Does not establish v4 HM35/pair home identity. |
| `axis_pos_counts` | `bridge_telemetry.go:649` | Fresh C.`axes[].pos_counts` (`ipcclient/client_linux.go:130`); stale entries null. |
| `axis_statusword` | `bridge_telemetry.go:652` | Fresh C.`axes[].statusword` (`ipcclient/client_linux.go:131`); stale entries null. |
| `axis_error_code`, `axis_603f_error_code` | `bridge_telemetry.go:661` | Fresh C.`axes[].error_code` (`ipcclient/client_linux.go:132`); stale entries null. |
| `axis_target_counts`, `axis_error_counts` | `bridge_telemetry.go:650` | Absent raw target/difference; null. Logical targets are not raw count targets. |
| `axis_mode_display` | `bridge_telemetry.go:653` | Absent; null. |
| `axis_brake_command`, `axis_braking_command` | `bridge_telemetry.go:654` | Absent commanded output; null. Native `brake_state` is a different state machine, retained under `rt_core`. |
| `axis_voltage_mv`, `axis_current_ma` | `bridge_telemetry.go:657` | Absent; null. |
| `hm35_home_position_counts` | `bridge_telemetry.go:659` | Absent HM35 anchor; null. Native offset is not a v4 HM35 receipt. |
| `active_jog_drop_reason_code`, `active_jog_drop_reason`, `active_jog_drop_count` | `bridge_telemetry.go:667` | Absent legacy jog lifecycle/counter; null. |
| `speed_limit_enabled`, `speed_limit_rad_s`, `speed_limit_last_scale`, `speed_limit_requested_max_rad_s`, `speed_limit_applied_max_rad_s`, `speed_limit_axis_mask`, `speed_limit_command_count`, `speed_limit_sender_id` | `bridge_telemetry.go:677` | Absent v4 global speed limiter and command receipts; all null. D.`axes[].max_velocity` has a different per-axis contract. |
| `safe_min_rad`, `safe_max_rad` | `bridge_commands.go:1838`; `bridge_telemetry.go:880` | D.`axes[].min_position`, `max_position` only when `position_unit=rad`; linear-axis entries null (`ipcclient/description.go:22,30`). These are configured logical limits, not fresh raw encoder limits. |
| `safe_low_raw_counts`, `safe_high_raw_counts` | `bridge_commands.go:1838`; `bridge_telemetry.go:882` | Absent qualified HM35 raw bounds; null. |
| `operational_slaves` | `bridge_telemetry.go:848` | Absent bus topology count; null. |
| `ethercat_cycle_us` | `bridge_telemetry.go:850` | D.`cycle_ns` / 1000 (`ipcclient/description.go:50`), configured cadence. |
| `sync0_configured`, `sync0_cycle_us`, `sync0_cycle_multiple_250us`, `sync0_error_state` | `bridge_telemetry.go:851` | Absent SYNC0 observations; null. |
| `rtcore_loop_jitter_us` | `bridge_telemetry.go:855` | Absent from public status; null. Private metrics are not read. |
| `pdo_fresh` | `bridge_telemetry.go:856` | Every C axis reports `pdo_fresh` and the same sample `time_ns` as C (`ipcclient/client_linux.go:108,120`). False on any unavailable axis. |
| `pdo_fresh_sample` | `bridge_telemetry.go:857` | Absent v4 ring sample identity; null. |
| `commanded_actual_stop_sample`, `commanded_actual_stop_commanded_zero`, `commanded_actual_stop_target_feedback_aligned`, `commanded_actual_stop_max_abs_target_feedback_counts` | `bridge_telemetry.go:858` | Absent complete raw target/feedback stop evidence; null. |
| `axis_max_profile_velocity_counts`, `axis_max_step_counts_per_cycle`, `axis_active_jog_max_lead_counts` | `bridge_telemetry.go:862` | Absent v4 servo-side evidence; null. |
| `active_jog_target_lead_seconds`, `active_jog_target_max_lead_rad` | `bridge_telemetry.go:865` | Absent v4 jog governor settings; null. |
| `updated_at` | `bridge_telemetry.go:867` | Native C.`time_ns` projected to UTC using current same-host monotonic age (`sample` in `publisher.go`). Not the public HTTP receipt time. |
| `event` (record) | `bridge_telemetry.go:627` | `timeseries`: a poll snapshot, never a fabricated transition. |
| `sample` (record) | `bridge_telemetry.go:628` | Publisher poll sequence minus 1; not native cycle sequence. |
| `time_ms` (record) | `bridge_telemetry.go:629` | C.`time_ns` / 1000000. |
| `test_session_id`, `test-session-id` | `bridge_telemetry.go:57` | Absent; null. |
| `motion_output_enabled`, `ethercat_output_enabled` | `offline-programming/v1/internal/connected/rtcore.go:24` | Absent physical output proof; null. `armed`, enable intent and backend are not substitutes. |
| `runtime_mode_epoch` | `offline-programming/v1/internal/execution/nats_controller.go:95` | Absent v4 runtime mode-switch epoch; null. |
| `no_real_movement`, `link_up`, `responding_slaves`, `latest_max_abs_qd_rad_s` | `robot/v4/system_state.go:698,730,735,756` | Absent legacy guard/link/count/commanded-velocity observations; null. |
| `pair_config_jetson_id`, `pair_config_command_id` | `pair_config.go:78,1897` | Absent verified v4 pair identity and receipt; null. Deployment subject identity is not a pair receipt. |
| `pair_config_revision`, `pair_config_sha256`, `pair_config_generated_urdf_sha256`, `home_profile_key` | `pair_config.go:978` | Absent v4 persisted pair/URDF identity; null. D digests have different meanings. |
| `pair_config_gear_numerator`, `pair_config_gear_denominator`, `pair_config_home_position_rad`, `pair_config_activation_epoch`, `pair_config_home_epoch` | `pair_config.go:982` | Absent v4 pair reconciliation/home evidence; null. |
| `pair_config_drive_ratio_converged` | `pair_config.go:987` | Absent measured v4 convergence proof; null. |
| `pair_config_ratio_reconciliation_state`, `pair_config_ratio_reconciliation_simulated`, `pair_config_ratio_reconciliation_abort_code`, `pair_config_ratio_reconciliation_failed_axis_mask`, `pair_config_observed_gear_numerator`, `pair_config_observed_gear_denominator` | `pair_config.go:989` | Absent v4 reconciliation observations; null. |
| `recent_commands` | `bridge_telemetry.go:938` | Absent v4 command log; null. No command records are synthesized from polled execution. |
| `realtime_core_command_log_capacity`, `realtime_core_command_log_captured_count`, `realtime_core_command_log_gap_count`, `realtime_core_command_log_latest_sequence`, `realtime_core_command_log_source` | `bridge_telemetry.go:1232` | Absent v4 command log; null. |
| `realtime_core_command_log_complete` | `bridge_telemetry.go:1236` | Absent measured completeness; null. |
| `recent_commands_full_count`, `recent_commands_payload_count`, `recent_commands_payload_limit`, `recent_commands_payload_truncated` | `bridge_telemetry.go:1244` | Absent v4 command payload accounting; null. |

| Absent group / descendants | V4 field site | Publication |
|---|---|---|
| `tour_start_alarm`: `schema`, `state`, `robot_cell`, `test_session_id`, `arm_command_id`, `requested_beeps`, `verified_on_transitions`, `verified_off_transitions`, `final_o4_off`, `readback_source`, `requested_at`, `started_at`, `completed_at`, `updated_at`, `age_ms`, `sequence`, `error` | `bridge_alarm_lifecycle.go:241` | Parent null; additive `tour_start_alarm_valid=false`. No physical ON/OFF lifecycle is claimed. |
| `revpi_output_readback.schema` | `bridge_revpi_output_readback.go:72` | Compatibility metadata `robot.v4.revpi-output-readback.v1`. |
| `revpi_output_readback.output_count`, `.values_mask`, `.sampled_at`, `.age_ms` | `bridge_revpi_output_readback.go:73` | Null. No sampled physical output or age exists. |
| `revpi_output_readback.valid_mask`, `.fresh`, `.error` | `bridge_revpi_output_readback.go:75` | Existing validity fields 0/false, diagnostic `unavailable from rt-control`. OLP consequently reports weld readback unknown (`offline-programming/v1/internal/connected/rtcore.go:141`). |
| `virtual_simulation_indicator`: `schema`, `label`, `pattern`, `active_color`, `phase_index`, `phase_ms`, `cycle_ms`, `virtual_only`, `physical_output_authority`, `physical_outputs_required_off` | `bridge_telemetry.go:95` | Parent null; additive `virtual_simulation_indicator_valid=false`. No virtual lamp or physical off receipt is invented. |
| Null `recent_commands` record descendants: `event`, `command`, `subject`, `updated_at`, `command_id`, `axis`, `axis_mask`, `test_session_id`, `result`, `error`, `sender_id`, `mode`, `slot`, `on`, `servos_armed_requested`; merged extensions `source`, `latest_command_kind`, `sequence`, `received_monotonic_ns`, `command_log_captured_at` | `bridge_telemetry.go:996,1080,1136,1208` | Entire command-record collection absent; no descendants emitted. |

| Additive extension | RT source / contract |
|---|---|
| `dropped` on status/record | Publisher sample counter described above. |
| `publisher_state`, `last_error`, `reconnects`, `drops` on status envelope | Publisher diagnostics described above. |
| `tour_start_alarm_valid`, `virtual_simulation_indicator_valid` | False availability flags for null parents. |
| `rt_core.core`, `.motion`, `.execution`, `.axes` | The public SDK's typed data, unchanged (`adapters/rosie/control/api_generated.go:420`). Logical position/target/optional velocity retains validity and units, without inventing v4 raw fields. |
| `rt_core.daemon_incarnation`, `.adapter_incarnation` | Public process identity, `api_generated.go:421`. |
| `rt_core.configuration_sha256`, `.machine_sha256`, `.deployment_sha256` | D digests, `ipcclient/description.go:45`. Never used as v4 pair/URDF hashes. |
| `rt_core.capabilities_digest` | Generated public contract digest, `api_generated.go:446`. |

The `rt_core` extension is present only in status. S4/S11 and A1-A3 batch
decoders retain their envelope/record map inputs: event/sample/time, session
identity and the legacy count/mask/current/voltage fields, with explicit nulls
where unavailable (`robot/v4/test/realtime-core/nats_section_test.go:329`,
`preflight_test.go:769`, `telemetry_evidence_test.go:79`). None of those projections
reads `rt_core`; a polled batch still cannot supply missing high-rate evidence.

Legacy decoders can read these envelopes, but safety-dependent readers may
correctly refuse absent output, pair, home or alarm evidence. In particular the
pair-owner probe and system-state mode gate do not gain new authority from this
publisher. This task supplies observation continuity, not P6-03/P6-05 completion.

## Validation contracts

`publisher/publisher_test.go` runs a simulation daemon and rt-control through
`internal/simfixture`, a TCP NATS broker with JetStream API/ack behavior, an
independent OLP-shaped Go decoder and the system-state nested-map decode. Tests
assert the exact stream limits, status/batch subjects and shapes, missing evidence,
stall drops in both payloads, archive-preserving UPDATE, no application grant,
and the contract-specific executable exit. Review regressions cover immutable
UPDATE refusal, permanent sentinels without retries, deduplicated diagnostics,
source-outage status, cadence, duplicate publisher detection and batch projections.
CLI tests check environment/exit-unit parity and real packaging without the optional
binary; the source lifecycle fixture also omits that binary. The broker is a composed
protocol fixture, not a production JetStream durability qualification.
`host/install-test.sh` checks the optional unit, deployment validation and lack of
activation; `host/package-test.sh` checks binary/unit staging and relocation.
