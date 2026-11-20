#pragma once

// The connection owns and joins this worker before reclaiming its IPC regions.
// Mailbox and bulk work is queued; the drainer only publishes bounded records.
void MachineState::run_helper(ConnectionState& connection) {
  request_port = &helper_controls;
  JogCommandRuntime latest_jog_command{};
  auto& cmd_shm = connection.cmd_shm;
  auto& status_shm = connection.status_shm;
  auto& cmd_eventfd = connection.cmd_eventfd;
  auto& status_eventfd = connection.status_eventfd;
  auto& helper_running = connection.helper_running;
  auto& topology_hash = connection.topology_hash;
  auto& build_id_hash = connection.build_id_hash;
      pthread_setname_np(pthread_self(), "ipc-helper");

      if (!apply_housekeeping_affinity("ipc-helper", helper_affinity_applied)) return;

      auto* status_hdr =
          reinterpret_cast<const gradient::ipc::v1::ShmHeaderV1*>(status_shm.base);
      RingView status_ring = make_ring_view(status_shm.base, status_hdr);
      uint64_t status_seq = 1;

      // Command ring + setpoint slot live in cmd_shm.
      auto* cmd_hdr =
          reinterpret_cast<const gradient::ipc::v1::ShmHeaderV1*>(cmd_shm.base);
      RingView cmd_ring = make_ring_view(cmd_shm.base, cmd_hdr);
      rosie::protocol::CommandResult command_result{};
      uint64_t last_command_seq = 0;
      const uint64_t connection_generation = control_policy.generation.load();

      auto clear_motion_intent = [&](uint64_t active_cmd_seq,
                                     uint32_t last_event_code,
                                     uint32_t exec_state) {
        (void)last_event_code;
        (void)exec_state;
        bulk_reset_epoch.fetch_add(1);
        (void)request_control({ControlOperation::AbortMotion});

        latest_jog_command.axis_mask = 0;
        latest_jog_command.flags = gradient::ipc::v1::JOG_FLAG_STOP;
        latest_jog_command.timeout_ns = 0;
        latest_jog_command.cmd_seq = active_cmd_seq;
        for (uint32_t axis_i = 0; axis_i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++axis_i) {
          latest_jog_command.velocity_counts_per_s[axis_i] = 0.0;
        }
        latest_jog_command.seq = active_cmd_seq;
        jog_commands.publish(latest_jog_command);

        command_result.sequence = active_cmd_seq;
        command_result.result = rosie::protocol::result_accepted;
        command_result.reason = rosie::protocol::reason_none;
      };

      // Emit STATUS_HELLO once on connect.
      {
        gradient::ipc::v1::StatusHelloV1 sh{};
        sh.build_id_hash = build_id_hash;
        sh.topology_hash = topology_hash;
        sh.cycle_ns = opt.cycle_ns;
        sh.num_axes = opt.num_axes;
        // Preserved ABI field: mixed per-axis profiles have no global profile ID.
        sh.drive_profile_id = gradient::ipc::v1::DRIVE_PROFILE_UNKNOWN;
        sh.wkc_expected = 2 * opt.num_axes;
        ring_write(status_ring,
                   gradient::ipc::v1::MSG_STATUS_HELLO,
                   &sh,
                   sizeof(sh),
                   &status_seq,
                   now_monotonic_ns());
        eventfd_write_one(status_eventfd);
      }

      // Emit STATUS_AXIS_CONFIG once on connect (bring-up tooling).
      {
        gradient::ipc::v1::StatusAxisConfigV1 cfg{};
        cfg.num_axes = opt.num_axes;
        for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
          cfg.counts_per_rev[i] = opt.axis[i].counts_per_rev;
          cfg.gear_ratio[i] = opt.axis[i].gear_ratio;
          cfg.sign[i] = opt.axis[i].sign;
          cfg.axis_type[i] = opt.axis[i].axis_type;
          cfg.counts_per_unit[i] = opt.axis[i].counts_per_unit;
        }
        ring_write(status_ring,
                   gradient::ipc::v1::MSG_STATUS_AXIS_CONFIG,
                   &cfg,
                   sizeof(cfg),
                   &status_seq,
                   now_monotonic_ns());
        eventfd_write_one(status_eventfd);
      }

      uint64_t next_snapshot_ns = now_monotonic_ns();
      uint64_t next_jog_debug_ns = now_monotonic_ns();

      // Unverified dispatcher budget: at most 64 command copies (32 KiB),
      // 64 receipts per each of three queues (two status records per receipt),
      // and the periodic snapshots per pass. Observation gets a turn after
      // at most 448 command/result records regardless of upload size.
      constexpr unsigned kDispatchBudget = 64;
      while (helper_running.load(std::memory_order_relaxed) &&
             !g_stop.load(std::memory_order_relaxed)) {
        pollfd pfds[1]{};
        pfds[0].fd = cmd_eventfd;
        pfds[0].events = POLLIN;

        // Wake periodically for status publishing even if no commands arrive.
        rosie::protocol::CommandResult completed{};
        auto publish_result = [&](const rosie::protocol::CommandResult& result) {
          if (result.generation < connection_generation) return;
          // Publish the applied motion state before its receipt. Faster
          // dispatch must not expose a result beside an older idle snapshot.
          gradient::ipc::v1::StatusMotionStateV1 motion{};
          if (motion_snapshot.read(motion) && !ring_write(status_ring,
                  gradient::ipc::v1::MSG_STATUS_MOTION_STATE, &motion, sizeof(motion),
                  &status_seq, now_monotonic_ns())) revoke_outputs();
          if (!ring_write(status_ring, rosie::protocol::msg_command_result, &result,
                          sizeof(result), &status_seq, now_monotonic_ns())) revoke_outputs();
          eventfd_write_one(status_eventfd);
        };
        const uint64_t committed_results = rt_results_committed.load(std::memory_order_acquire);
        for (unsigned count = 0; count < kDispatchBudget && rt_result_consumed < committed_results &&
             rt_results.pop(completed); ++count) {
          ++rt_result_consumed;
          publish_result(completed);
        }
        for (unsigned count = 0; count < kDispatchBudget && bulk_results.pop(completed); ++count)
          publish_result(completed);
        CommissioningJobReceipt receipt{};
        for (unsigned count = 0; count < kDispatchBudget && commissioning_job_receipts.pop(receipt); ++count)
          if (receipt.kind != CommissioningJobKind::Readback) publish_result(receipt.result);
        const bool commands_pending = cmd_ring.header &&
            __atomic_load_n(&cmd_ring.header->write_idx, __ATOMIC_ACQUIRE) != cmd_ring.header->read_idx;
        int pr2 = poll(pfds, 1, commands_pending ? 0 : 1);
        if (pr2 > 0 && (pfds[0].revents & POLLIN)) eventfd_drain(cmd_eventfd);
        if (commands_pending || pr2 > 0) {
          // Drain command ring entries and update local state.
          if (cmd_ring.header && cmd_ring.header->magic == gradient::ipc::v1::kMagicRing) {
            uint32_t r = cmd_ring.header->read_idx;
            const uint32_t w = __atomic_load_n(&cmd_ring.header->write_idx, __ATOMIC_ACQUIRE);
            if (uint32_t(w - r) > cmd_ring.capacity) { revoke_outputs(); r = w; }
            bool bulk_backpressure = false;
            for (unsigned count = 0; r != w && count < kDispatchBudget; ++count) {
              const uint32_t slot = r % cmd_ring.capacity;
              uint8_t* slot_ptr =
                  cmd_ring.entries + static_cast<size_t>(slot) * cmd_ring.msg_bytes;

              alignas(8) std::array<uint8_t, gradient::ipc::v1::GRADIENT_RING_MSG_BYTES> local_message{};
              std::memcpy(local_message.data(), slot_ptr, local_message.size());
              auto* mh = reinterpret_cast<const gradient::ipc::v1::MsgHeader*>(local_message.data());
              const uint8_t* payload = local_message.data() + sizeof(*mh);
              const uint64_t received_ns = now_monotonic_ns();
              if (mh->flags != 0 || mh->bytes < sizeof(*mh) || mh->bytes > local_message.size() ||
                  mh->bytes != rosie::rtcore::command_message_size(mh->type) ||
                  mh->time_ns <= revoke_before_ns.load(std::memory_order_acquire) ||
                  mh->seq <= last_command_seq || mh->time_ns > received_ns ||
                  received_ns - mh->time_ns >= control_policy.lease_ns) {
                revoke_outputs();
                rosie::protocol::CommandResult rejected{mh->seq, 0, control_policy.generation.load(), mh->type,
                    rosie::protocol::result_rejected, rosie::protocol::reason_invalid_command, 0};
                ring_write(status_ring, rosie::protocol::msg_command_result, &rejected, sizeof(rejected),
                           &status_seq, received_ns);
                eventfd_write_one(status_eventfd);
                ++r;
                __atomic_store_n(&cmd_ring.header->read_idx, r, __ATOMIC_RELEASE);
                continue;
              }
              command_result = {mh->seq, 0, control_policy.generation.load(), mh->type,
                  rosie::protocol::result_rejected, rosie::protocol::reason_not_ready, 0};
                switch (mh->type) {
                  case rosie::protocol::msg_io_test: {
                    rosie::protocol::IoTest io{};
                    if (!rosie::protocol::decode_io_test(payload, rosie::protocol::io_test_bytes, io) || io.value > 3 || io.value==2) {
                      command_result.reason=rosie::protocol::reason_invalid_command; break;
                    }
                    ControlRequest request{ControlOperation::IoTest};
                    request.value=io.value&1; request.io_begin=(io.value&2)!=0; request.io_index=io.index;
                    request.io_sequence=io.source_sequence; request.io_deadline=io.deadline_ns;
                    request.generation=io.native_generation; request.application_generation=io.application_generation;
                    const auto result=request_control(request); command_result.reason=result.reason;
                    if(result.accepted) command_result.result=rosie::protocol::result_accepted;
                    break;
                  }
                  case rosie::protocol::msg_io_arm:
                  case rosie::protocol::msg_io_disarm: {
                    rosie::protocol::IoRequest io{};
                    if (!rosie::protocol::decode_io_request(payload, rosie::protocol::io_request_bytes, io)) {
                      command_result.reason=rosie::protocol::reason_invalid_command; break;
                    }
                    ControlRequest request{ControlOperation::IoArm};
                    request.value=mh->type==rosie::protocol::msg_io_arm;
                    request.generation=io.native_generation; request.application_generation=io.application_generation;
                    const auto result=request_control(request); command_result.reason=result.reason;
                    if(result.accepted) command_result.result=rosie::protocol::result_accepted;
                    break;
                  }
                case rosie::protocol::msg_halt: {
                  rosie::protocol::HaltRequest halt{};
                  if (!rosie::protocol::decode_halt_request(payload, rosie::protocol::halt_request_bytes, halt)) {
                    command_result.reason = rosie::protocol::reason_invalid_command;
                    break;
                  }
                  ControlRequest request{ControlOperation::Halt};
                  request.generation = halt.native_generation;
                  request.application_generation = halt.application_generation;
                  request.reset_sequence = mh->seq;
                  const auto result = request_control(request);
                  command_result.reason = result.reason;
                  command_result.axis_mask = result.enabled_mask;
                  if (result.accepted) command_result.result = rosie::protocol::result_accepted;
                  break;
                }
                case rosie::protocol::msg_telemetry_mark: {
                  gradient::ipc::v1::TelemetryMarkV1 mark{};
                  if (!gradient::ipc::v1::decode_TelemetryMarkV1(payload, sizeof(mark), mark) ||
                      !mark.length || mark.length > sizeof(mark.label)) {
                    command_result.reason = rosie::protocol::reason_invalid_command;
                    break;
                  }
                  rosie::rtcore::TelemetryTrigger trigger{};
                  trigger.end = telemetry.written();
                  trigger.reason = 4;
                  trigger.length = mark.length;
                  std::memcpy(trigger.label.data(), mark.label, sizeof(mark.label));
                  if (!telemetry_marks.push(trigger)) {
                    telemetry_trigger_drops.fetch_add(1);
                    command_result.reason = rosie::protocol::reason_capacity;
                    break;
                  }
                  command_result.result = rosie::protocol::result_accepted;
                  command_result.reason = rosie::protocol::reason_none;
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_ARM: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdArmV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdArmV1*>(payload);
                    if (cmd->arm > 1 || cmd->reserved) break;
                    const bool arm = (cmd->arm != 0);
                    if (!request_control({ControlOperation::Arm, arm ? 1u : 0u}).accepted) break;
                    command_result.result = rosie::protocol::result_accepted;
                    command_result.reason = rosie::protocol::reason_none;
                    if (!arm) {
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_AXIS_ENABLE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdAxisMaskV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdAxisMaskV1*>(payload);
                    if (cmd->reserved || !request_control({ControlOperation::Enable, cmd->axis_mask}).accepted) break;
                    command_result.result = rosie::protocol::result_accepted;
                    command_result.reason = rosie::protocol::reason_none;
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_AXIS_DISABLE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdAxisMaskV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdAxisMaskV1*>(payload);
                    (void)request_control({ControlOperation::Disable, cmd->axis_mask});
                    if (true) {
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_SET_MODE: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdSetModeV1)) {
                    auto* cmd = reinterpret_cast<const gradient::ipc::v1::CmdSetModeV1*>(payload);
                    if (cmd->mode != 8) { revoke_outputs(); break; }
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    if (mask == 0u) {
                      mask = valid;
                    }
                    mask &= valid;
                    (void)request_control({ControlOperation::SetMode, mask});
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_FAULT_RESET: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdFaultResetV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdFaultResetV1*>(payload);
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    if (mask == 0) {
                      mask = valid;
                    }
                    mask &= valid;
                    if (mask != 0) {
                      ControlRequest reset{ControlOperation::ResetFault, mask};
                      reset.reset_sequence = mh->seq;
                      // Preserve the eight legacy bytes: zero axis mask and
                      // zero reserved flags select the reason-agnostic oracle.
                      reset.legacy_reset = cmd->axis_mask == 0 && cmd->reserved == 0;
                      if (cmd->reserved != 0) {
                        command_result.reason = rosie::protocol::reason_invalid_command;
                        break;
                      }
                      const auto applied = request_control(reset);
                      if (!applied.accepted) {
                        // The JSON extension carries fault_persists and its
                        // sequence; the existing binary not_ready byte stays.
                        command_result.reason = rosie::protocol::reason_not_ready;
                        break;
                      }
                      clear_motion_intent(
                          mh->seq,
                          gradient::ipc::v1::EVT_DISARMED,
                          gradient::ipc::v1::EXEC_STATE_ABORTED);
                    }
                  }
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_RESTORE_REFERENCE_VERIFIED_DELTA:
                case gradient::ipc::v1::MSG_CMD_NATIVE_HOME:
                case gradient::ipc::v1::MSG_CMD_SERVICE_SDO_WRITE: {
                  CommissioningClaim held;
                  if (!try_claim_commissioning(CommissioningClaim::Command, &held)) {
                    if (held == CommissioningClaim::Readback) {
                      // A background readback owns the mailbox for milliseconds; keep
                      // this slot unconsumed and retry on the next dispatcher pass
                      // rather than refusing an operator command it never saw.
                      bulk_backpressure = true;
                      break;
                    }
                    logf("Commissioning busy cmd_seq=%llu operation=0x%x",
                         static_cast<unsigned long long>(mh->seq), static_cast<unsigned>(mh->type));
                    command_result.reason = rosie::protocol::reason_mode_conflict;
                    break;
                  }
                  CommissioningJob job{};
                  job.result = command_result;
                  job.identity = {mh->seq, command_result.generation,
                                  configuration_epoch.load(), home_epoch.load()};
                  job.dirty_epoch = dirty_configuration_epoch.load();
                  if (mh->type == gradient::ipc::v1::MSG_CMD_RESTORE_REFERENCE_VERIFIED_DELTA) {
                    gradient::ipc::v1::CmdRestoreReferenceVerifiedDeltaV1 decoded{};
                    std::memcpy(&decoded, payload, sizeof(decoded));
                    job.kind = CommissioningJobKind::RestoreReferenceVerifiedDelta;
                    job.home.axis_mask = decoded.axis_mask; job.home.flags = decoded.flags;
                  } else if (mh->type == gradient::ipc::v1::MSG_CMD_NATIVE_HOME) {
                    job.kind = CommissioningJobKind::Home;
                    std::memcpy(&job.home, payload, sizeof(job.home));
                  } else {
                    job.kind = CommissioningJobKind::Write;
                    std::memcpy(&job.write, payload, sizeof(job.write));
                  }
                  if (!commissioning_jobs.push(job)) {
                    commissioning_claim.store(CommissioningClaim::Idle, std::memory_order_release);
                    command_result.reason = rosie::protocol::reason_mode_conflict;
                    break;
                  }
                  command_result.result = UINT32_MAX;
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_BEGIN:
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_POINT:
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_COMMIT:
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_START:
                case rosie::protocol::msg_trajectory_discard:
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_ABORT: {
                  BulkJob job{control_policy.generation.load(), bulk_reset_epoch.load()};
                  if(mh->bytes>job.message.size()) { command_result.reason=rosie::protocol::reason_invalid_command; break; }
                  std::memcpy(job.message.data(),local_message.data(),mh->bytes);
                  if (!bulk_jobs.push(job)) {
                    // Keep this slot unconsumed and retry on the next dispatcher
                    // pass; observation and receipts still run during backpressure.
                    bulk_backpressure = true;
                    break;
                  }
                  command_result.result = UINT32_MAX;
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_JOG: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdJogV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdJogV1*>(payload);
                    // QUICK_STOP is only a modifier of legacy STOP. Both
                    // encodings request global inhibition; neither promises
                    // a qualified deceleration or physical standstill.
                    const bool stop = cmd->flags == gradient::ipc::v1::JOG_FLAG_STOP ||
                        cmd->flags == (gradient::ipc::v1::JOG_FLAG_STOP |
                                       gradient::ipc::v1::JOG_FLAG_QUICK_STOP);
                    if (stop) {
                      // Legacy jog STOP shares the global cancellation contract.
                      revoke_outputs(true); bulk_reset_epoch.fetch_add(1);
                      command_result.result = rosie::protocol::result_accepted;
                      command_result.reason = rosie::protocol::reason_none;
                      break;
                    }
                    if (cmd->flags != gradient::ipc::v1::JOG_FLAG_ACTIVE) {
                      command_result.reason = rosie::protocol::reason_invalid_command;
                      revoke_outputs();
                      break;
                    }
                    bool valid = cmd->flags == gradient::ipc::v1::JOG_FLAG_ACTIVE &&
                        armed.load() && axes_ready(cmd->axis_mask) &&
                        cmd->timeout_ns > 0 && cmd->timeout_ns <= control_policy.lease_ns;
                    for (uint32_t i = 0; i < opt.num_axes; ++i)
                      if ((cmd->axis_mask & (1u << i)) && !axis_profiles[i].limits.allows_velocity(cmd->qd[i])) valid = false;
                    if (!valid) { revoke_outputs(); break; }
                    latest_jog_command.generation = control_policy.generation.load();
                    latest_jog_command.axis_mask = cmd->axis_mask;
                    latest_jog_command.flags = cmd->flags;
                    latest_jog_command.timeout_ns = cmd->timeout_ns;
                    latest_jog_command.cmd_seq = mh->seq;
                    for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                      if ((cmd->axis_mask & (1u << i)) == 0u) {
                        latest_jog_command.velocity_counts_per_s[i] = 0.0;
                        continue;
                      }
                      const double cpu = opt.axis[i].counts_per_unit;
                      const int sgn = opt.axis[i].sign;
                      latest_jog_command.velocity_counts_per_s[i] =
                          cmd->qd[i] * cpu * static_cast<double>(sgn);
                    }
                    latest_jog_command.seq = mh->seq;
                    jog_commands.publish(latest_jog_command);

                    command_result.result = UINT32_MAX;
                  }
                  break;
                }
                default:
                  break;
              }

              if (bulk_backpressure) break;
              last_command_seq = mh->seq;
              if (command_result.result != UINT32_MAX) {
                if (mh->type != gradient::ipc::v1::MSG_CMD_NATIVE_HOME)
                  command_result.generation = control_policy.generation.load();
                if (!ring_write(status_ring, rosie::protocol::msg_command_result, &command_result,
                                sizeof(command_result), &status_seq, now_monotonic_ns())) revoke_outputs();
                eventfd_write_one(status_eventfd);
              }
              r += 1;
              __atomic_store_n(&cmd_ring.header->read_idx, r, __ATOMIC_RELEASE);
            }
            __atomic_store_n(&cmd_ring.header->read_idx, r, __ATOMIC_RELEASE);
          }
        }

        if (!commands_pending && readbacks_requested.load() && try_claim_commissioning(CommissioningClaim::Readback)) {
          readbacks_requested.store(false);
          if (!commissioning_jobs.push(readback_job())) {
            commissioning_claim.store(CommissioningClaim::Idle, std::memory_order_release);
          }
        }

        uint64_t now = now_monotonic_ns();
        if (now >= next_snapshot_ns) {
          next_snapshot_ns = now + 100000000; // 100ms (10Hz) for scaffolding

          gradient::ipc::v1::StatusSnapshotV1 snap{};
          (void)io_snapshot.read(snap.io);
          snap.num_axes = opt.num_axes;
          snap.wkc_expected = 2 * opt.num_axes;
          snap.wkc_actual = 0;
          snap.master_state = armed.load(std::memory_order_relaxed)
                                  ? gradient::ipc::v1::MASTER_SAFEOP
                                  : gradient::ipc::v1::MASTER_INIT;
          snap.dc_offset_ns = 0;
          snap.cycle_jitter_ns = rt_last_jitter_ns.load(std::memory_order_relaxed);
          snap.topology_hash = topology_hash;

          for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            snap.axes[i] = AxisStatusV1{};
            snap.axes[i].ds402_state = gradient::ipc::v1::DS402_UNKNOWN;
            snap.axes[i].brake_state = gradient::ipc::v1::BRAKE_UNKNOWN;
          }

          // Prefer real feedback (once EtherCAT is active), else fall back to target counts.
          const uint64_t fb_seq = latest_feedback.seq.load(std::memory_order_acquire);
          if (fb_seq != 0 && !(fb_seq & 1u)) {
            snap.wkc_actual = latest_feedback.wkc_actual;
            if (latest_feedback.wkc_expected != 0) {
              snap.wkc_expected = latest_feedback.wkc_expected;
            }
            snap.master_state = latest_feedback.master_state;
            snap.dc_offset_ns = latest_feedback.dc_offset_ns;
            snap.cycle_jitter_ns = latest_feedback.cycle_jitter_ns;
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              snap.axes[i].pos_counts = latest_feedback.pos_counts[i];
              snap.axes[i].brake_state = latest_feedback.brake_state[i];
              snap.axes[i].torque_raw = latest_feedback.torque_raw[i];
              snap.axes[i].statusword = latest_feedback.statusword[i];
              snap.axes[i].error_code = latest_feedback.error_code[i];
              snap.axes[i].manufacturer_error_code = latest_feedback.manufacturer_error_code[i];
              snap.axes[i].mode_display = latest_feedback.mode_display[i];
              snap.axes[i].ds402_state = latest_feedback.ds402_state[i];
              snap.axes[i].di_bits = latest_feedback.di_bits[i];
              snap.axes[i].di_valid = latest_feedback.di_valid[i];
              snap.axes[i].external_enable_active = latest_feedback.external_enable_active[i];
              snap.axes[i].external_enable_valid = latest_feedback.external_enable_valid[i];
              snap.axes[i].velocity_actual_counts_per_s = latest_feedback.velocity_actual_counts_per_s[i];
              snap.axes[i].following_error_counts = latest_feedback.following_error_counts[i];
              snap.axes[i].velocity_actual_valid = latest_feedback.velocity_actual_valid[i];
              snap.axes[i].following_error_valid = latest_feedback.following_error_valid[i];
              snap.axes[i].max_abs_velocity_actual_counts_per_s = latest_feedback.max_abs_velocity_actual_counts_per_s[i];
              snap.axes[i].max_abs_following_error_counts = latest_feedback.max_abs_following_error_counts[i];
            }
          } else {
            // No live process data yet; leave snapshot at zero/default values.
          }

          ring_write(status_ring,
                     gradient::ipc::v1::MSG_STATUS_SNAPSHOT,
                     &snap,
                     sizeof(snap),
                     &status_seq,
                     now);
          eventfd_write_one(status_eventfd);

          gradient::ipc::v1::StatusMotionStateV1 motion{};
          if (motion_snapshot.read(motion)) {
          ring_write(status_ring,
                     gradient::ipc::v1::MSG_STATUS_MOTION_STATE,
                     &motion,
                     sizeof(motion),
                     &status_seq,
                     now);
          eventfd_write_one(status_eventfd);

          }

          // Validity, host observation time and PDO fields must come from one
          // completed RT cycle. The earlier classic-status sequence cannot
          // cover these later reads across intervening ring publications.
          gradient::ipc::v1::StatusExtendedSnapshotV1 ext_snap{};
          if (rosie::rtcore::read_sequenced_snapshot(latest_feedback.seq, ext_snap,
              [&](auto& ext_snap) noexcept {
            ext_snap.num_axes = opt.num_axes;
            ext_snap.valid_axis_mask =
                latest_feedback.ext_valid_axis_mask.load(std::memory_order_acquire);
            ext_snap.sample_time_ns =
                latest_feedback.ext_sample_time_ns.load(std::memory_order_acquire);
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              ext_snap.axes[i].position_error_counts = latest_feedback.ext_position_error_counts[i];
              ext_snap.axes[i].multi_turn_lo = latest_feedback.ext_multi_turn_lo[i];
              ext_snap.axes[i].multi_turn_hi = latest_feedback.ext_multi_turn_hi[i];
              ext_snap.axes[i].bus_voltage_raw = latest_feedback.ext_bus_voltage_raw[i];
              ext_snap.axes[i].load_rate_raw = latest_feedback.ext_load_rate_raw[i];
              ext_snap.axes[i].igbt_temp_raw = latest_feedback.ext_igbt_temp_raw[i];
              ext_snap.axes[i].motor_temp_raw = latest_feedback.ext_motor_temp_raw[i];
              ext_snap.axes[i].drive_not_ready_bits = latest_feedback.ext_drive_not_ready_bits[i];
              ext_snap.axes[i].motor_not_rotating_code = latest_feedback.ext_motor_not_rotating_code[i];
            }
          })) {
            ring_write(status_ring,
                       gradient::ipc::v1::MSG_STATUS_EXTENDED_SNAPSHOT,
                       &ext_snap,
                       sizeof(ext_snap),
                       &status_seq,
                       now);
            eventfd_write_one(status_eventfd);
          }
        }

        if (now >= next_jog_debug_ns) {
          next_jog_debug_ns = now + 20000000; // 20ms (50Hz) for jog stop instrumentation
          const uint64_t dbg_seq = latest_jog_debug.seq.load(std::memory_order_acquire);
          if (dbg_seq != 0) {
            gradient::ipc::v1::StatusJogDebugV1 jog_debug{};
            jog_debug.num_axes = latest_jog_debug.num_axes;
            jog_debug.active_jog = latest_jog_debug.active_jog;
            jog_debug.active_jog_axis_mask = latest_jog_debug.active_jog_axis_mask;
            jog_debug.command_sp_mask = latest_jog_debug.command_sp_mask;
            jog_debug.have_hold_mask = latest_jog_debug.have_hold_mask;
            jog_debug.have_jog_target_mask = latest_jog_debug.have_jog_target_mask;
            jog_debug.snap_hold_mask = latest_jog_debug.snap_hold_mask;
            jog_debug.stop_arrest_mask = latest_jog_debug.stop_arrest_mask;
            jog_debug.latest_cmd_axis_mask = latest_jog_debug.latest_cmd_axis_mask;
            jog_debug.latest_cmd_flags = latest_jog_debug.latest_cmd_flags;
            jog_debug.last_stop_reason = latest_jog_debug.last_stop_reason;
            jog_debug.last_stop_axis_mask = latest_jog_debug.last_stop_axis_mask;
            jog_debug.sample_time_ns = latest_jog_debug.sample_time_ns;
            jog_debug.active_jog_cmd_seq = latest_jog_debug.active_jog_cmd_seq;
            jog_debug.latest_jog_seq_seen = latest_jog_debug.latest_jog_seq_seen;
            jog_debug.active_jog_deadline_ns = latest_jog_debug.active_jog_deadline_ns;
            jog_debug.latest_cmd_timeout_ns = latest_jog_debug.latest_cmd_timeout_ns;
            jog_debug.last_stop_time_ns = latest_jog_debug.last_stop_time_ns;
            jog_debug.last_stop_cmd_seq = latest_jog_debug.last_stop_cmd_seq;
            std::copy(latest_jog_debug.feedback_pos_counts.begin(),
                      latest_jog_debug.feedback_pos_counts.end(),
                      jog_debug.feedback_pos_counts);
            std::copy(latest_jog_debug.hold_target_counts.begin(),
                      latest_jog_debug.hold_target_counts.end(),
                      jog_debug.hold_target_counts);
            std::copy(latest_jog_debug.output_target_counts.begin(),
                      latest_jog_debug.output_target_counts.end(),
                      jog_debug.output_target_counts);
            std::copy(latest_jog_debug.output_target_velocity_counts_per_s.begin(),
                      latest_jog_debug.output_target_velocity_counts_per_s.end(),
                      jog_debug.output_target_velocity_counts_per_s);

            ring_write(status_ring,
                       gradient::ipc::v1::MSG_STATUS_JOG_DEBUG,
                       &jog_debug,
                       sizeof(jog_debug),
                       &status_seq,
                       now);
            eventfd_write_one(status_eventfd);
          }
        }
      }
    }
