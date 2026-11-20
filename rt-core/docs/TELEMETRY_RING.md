# The 1 kHz telemetry ring: what it carries and how to read it

Entry point for anyone who needs to know what rt-core records every cycle and
how to get at it. State at `458a9236` (P8-34, record V2). The mechanics are in
[ARCHITECTURE.md](ARCHITECTURE.md) (telemetry row and the flow diagram); the
transport contract is in [APPLICATION_API.md](APPLICATION_API.md); the TypeScript
decoder notes are in [clients/ts/README.md](../clients/ts/README.md).

## How it works

- The cyclic thread writes one `CycleCaptureRecordV2` per cycle into a
  single-writer, prefaulted memfd ring (seqlock stamps, monotonic index,
  overwrite counter). Writing is one memcpy of values the cycle already holds;
  no allocation, formatting or bus access happens on the cycle for telemetry.
- Default RAM window is 100,000 ms (`--telemetry-retention-ms`), minimum one
  publisher period, ceiling bounded by 2 GiB for the mapping.
- Dumps to `runtime/telemetry/<utc>-<trigger>.bin` plus a JSON index are
  written by a dedicated SCHED_BATCH writer on Stop, a fault latch, a
  session-authorised `mark_telemetry`, or exit. At most one dump is pending;
  storms are recorded as skipped. Retention is 20 dumps and 536,870,912 bytes
  by default (`--telemetry-dump-count`, `--telemetry-dump-bytes`).
- Live access: `GET /v1/telemetry?after=<sequence>` and
  `/v1/telemetry/stream` on the adapter, decoded with the generated Go, C++
  or TypeScript decoders; a read-only descriptor handoff socket lets a separate
  service user read the ring directly.
- Offline access: `rtctl telemetry dump-to-jsonl <dump.bin>` renders a dump
  with the same field names; keep the `.bin` and its `.json` index together.
- Every reader checks the record size and `CycleCaptureRecordV2LayoutDigest`
  from the ring or batch header and refuses a mismatched layout rather than
  misreading it. Pre-V2 dumps are refused with a typed digest mismatch.
- The 512-record commissioning capture (`cycle-capture.jsonl`) shares the
  record type and field names.

## What is in every record

```
rt-core 1 kHz telemetry ring, record V2 (P8-34, landed 458a9236)
One CycleCaptureRecordV2 per cycle, 5,392 bytes for 16 axes; every field on one
timestamp. Default RAM window 100,000 ms; dumps on Stop, fault latch, mark, exit.

PER RECORD (cycle / group)
  timing        t_ns, seq, cycle, work_ns, cycle_jitter_ns
  bus           wkc_actual, wkc_expected, master_state, pdo_fresh_axis_mask,
                submitted_axis_mask, bus_submission_operation, bus_submission_return_code
  authority     armed, axis_enable_mask, grant_active, grant_axis_mask,
                grant_deadline_host_ns, app_grant_generation, control_generation
  motion        active_mode, state, active_traj_id, active_command_seq,
                current_point_index, queue_depth, underrun_count, stale_command_flag,
                motion_done, last_event_code, capability_flags
  jog           jog_session_open, jog_has_input, jog_axis_mask, jog_ramping,
                jog_generation, jog_source_sequence, jog_input_deadline_host_ns
  faults        safety_fault_mask, execution_fault_reasons
  identity      configuration_epoch, config_verified_mask, home_epoch, home_valid_mask,
                service_mode_axis_mask
  cell I/O      io_configured, io_input_word, io_input_valid, io_output_word, io_armed, io_reason

PER AXIS (x16)
  drive PDO     position (0x6064), statusword (0x6041), error_code (0x603F),
                manufacturer_error_code, mode_display (0x6061), ds402_state,
                torque_raw (0x6077) + torque_valid,
                velocity_actual_counts_per_s (0x606C) + valid,
                following_error_counts (0x60F4) + valid,
                di_bits + di_valid, external_enable_active + valid,
                pdo_fresh, pdo_observed_time_ns
  commands out  target (0x607A wire target), target_velocity_counts_per_s (0x60FF) + valid
  coordinates   coordinate_counts (continuous logical), coordinate_valid,
                coordinate_source_valid, coordinate_reason, coordinate_epoch,
                absolute_source_counts, absolute_value[16] + absolute_valid[16]
  calibration   calibration_valid, position_state (0 unverified, 1 recovering,
                2 trusted, 3 lost); coordinate_reason describes position loss
  audit         audit_reason, audit_last_verified_ns, audit_started_ns,
                audit_completed_ns, audit_coherence_error_counts
  collision wd  armed, torque_cycles, following_error_cycles, peak_torque_raw,
                peak_following_error_counts, samples, trip_count, last_trip_quantity,
                trip_sustained_cycles, last_trip_peak_torque_raw,
                last_trip_peak_following_error_counts
  running peaks max_abs_velocity_actual_counts_per_s, max_abs_following_error_counts
  brake / home  brake_state, native_home_state, native_home_position_offset,
                native_home_last_abort_code
  slave         slave_al_state, slave_online, slave_operational
  A6 extended   ext_position_error_counts, ext_multi_turn_lo/hi, ext_bus_voltage_raw,
  (0x2040, when ext_load_rate_raw, ext_igbt_temp_raw, ext_motor_temp_raw,
   mapped)      ext_drive_not_ready_bits, ext_motor_not_rotating_code,
                ext_valid + ext_valid_mask

Validity rule: every value with a *_valid flag is zero and invalid unless its
mapping was verified for the current configuration epoch and that cycle's
exchange was complete. On the 0x1B04 preset di_valid is false; on a6ec_1b02
velocity/following-error validity is false.
```

## What it is for

- **Tracking**: `target` against `position` (our view) and against
  `following_error_counts` (the drive's view), lag at cruise, catch-up
  overshoot, settling after the last target change. `benchdrive --measure`
  computes these for the feedforward A/B and the Halt stopping measurement.
- **Velocity following**: differentiated `target` against
  `velocity_actual_counts_per_s`; `target_velocity_counts_per_s` shows what
  feedforward was sent (its physical effect on the A6-EC is the open P4-19
  question).
- **Torque and the collision watchdog**: `torque_raw` on the same timestamp as
  the watchdog counters and peaks, so a trip can be replayed from the dump
  alone; `benchdrive --tune-collision` derives thresholds from the peaks.
- **Scheduler health**: `work_ns` and `cycle_jitter_ns` per cycle, alongside
  the status counters.
- **Authority and fault forensics**: grant, generation, fault masks and
  submission result per cycle, so "what did the machine know when it stopped"
  has a cycle-exact answer.

Software observation only: nothing here certifies safe speed, physical
standstill or brake engagement.
