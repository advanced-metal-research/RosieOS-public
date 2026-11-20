#pragma once

// This worker alone mutates trajectory storage. RT acquires immutable slots
// through the existing CAS handoff and retires them without reclaiming memory.
void MachineState::run_bulk_worker() {
  request_port = &bulk_controls;
  pthread_setname_np(pthread_self(), "bulk-worker");
  if (!apply_housekeeping_affinity("bulk-worker", bulk_affinity_applied)) return;
  struct LimitRefusal {
    enum class Kind { None, Point, Direction, Position, Velocity, Acceleration };
    Kind kind = Kind::None;
    uint32_t sample = 0, axis = 0;
    rosie::rtcore::interpolation::Validation detail{};
    // Select by ending sample, then native axis, so validation pass order
    // cannot hide an earlier violation on a higher numbered axis.
    void consider(Kind candidate, uint32_t at, uint32_t on,
        rosie::rtcore::interpolation::Validation check = {}) noexcept {
      if (kind == Kind::None || at < sample || (at == sample && on < axis))
        *this = {candidate, at, on, check};
    }
  };
  struct PendingTrajectoryUpload {
    decltype(trajectory_store)& store;
    int slot = -1;
    bool active = false;
    LimitRefusal refusal{};
    uint64_t traj_id = 0;
    uint32_t axis_mask = 0, expected_points = 0, point_count = 0;
    TrajectoryPointRuntime* points = nullptr;
    // Retain the logical inputs instead of undoing the count conversion.
    // Equal samples are stationary exactly; no tolerance admits outward drift.
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> previous_q{}, previous_qd{};
    // Return abandoned Writing storage even when this connection's worker exits.
    void reset() noexcept {
      store.abandon(slot);
      slot = -1; active = false; traj_id = 0;
      refusal = {};
      axis_mask = 0; expected_points = 0; point_count = 0; points = nullptr;
    }
    ~PendingTrajectoryUpload() { reset(); }
  } pending_upload{trajectory_store};
  uint64_t pending_generation = control_policy.generation.load();
  uint64_t reset_seen = bulk_reset_epoch.load();
  std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> upload_offsets{};
  // Compare all admission passes before reporting. This also runs on a bad
  // point's prefix, preserving early point inhibition without losing an earlier
  // continuous violation that would otherwise only be discovered at Commit.
  auto first_limit_refusal = [&](uint32_t count) {
    auto refusal = pending_upload.refusal;
    if (count < 2) return refusal;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (!(pending_upload.axis_mask & (1u << axis))) continue;
      const auto& profile = axis_profiles[axis];
      const double scale = opt.axis[axis].counts_per_unit * opt.axis[axis].sign;
      const double offset = upload_offsets[axis];
      const double lower = profile.limits.min_q * scale - offset;
      const double upper = profile.limits.max_q * scale - offset;
      const auto position = rosie::rtcore::interpolation::validate_recovery_position(
          pending_upload.points, count, axis, std::min(lower, upper), std::max(lower, upper));
      if (!position) refusal.consider(LimitRefusal::Kind::Position, position.segment + 1, axis, position);
      const auto velocity = rosie::rtcore::interpolation::validate_velocity(
          pending_upload.points, count, axis, profile.limits.max_qd * opt.axis[axis].counts_per_unit);
      if (!velocity) refusal.consider(LimitRefusal::Kind::Velocity, velocity.segment + 1, axis, velocity);
      if (profile.max_acceleration) {
        const auto acceleration = rosie::rtcore::interpolation::validate_acceleration(
            pending_upload.points, count, axis,
            static_cast<long double>(*profile.max_acceleration) * opt.axis[axis].counts_per_unit);
        if (!acceleration) refusal.consider(LimitRefusal::Kind::Acceleration, acceleration.segment + 1, axis, acceleration);
      }
    }
    return refusal;
  };
  auto report_limit_refusal = [&](const LimitRefusal& refusal, rosie::protocol::CommandResult& result) {
    const auto axis = refusal.axis;
    const auto& profile = axis_profiles[axis];
    const auto& check = refusal.detail;
    const double scale = opt.axis[axis].counts_per_unit * opt.axis[axis].sign;
    const double offset = upload_offsets[axis];
    result.axis_mask = 1u << axis;
    result.reason = rosie::protocol::reason_invalid_trajectory;
    const bool recovery = refusal.kind == LimitRefusal::Kind::Direction ||
        check.failure == rosie::rtcore::interpolation::Failure::RecoveryExtremum;
    if (recovery) result.reason = rosie::protocol::reason_outside_limits_outward;
    if (refusal.kind == LimitRefusal::Kind::Point) {
      logf("invalid_trajectory: point_limit axis=%u (%s) sample=%u", axis, profile.name.c_str(), refusal.sample);
    } else if (refusal.kind == LimitRefusal::Kind::Direction) {
      logf("outside_limits_outward: point_direction axis=%u (%s) sample=%u", axis, profile.name.c_str(), refusal.sample);
    } else if (refusal.kind == LimitRefusal::Kind::Position) {
      const char* diagnostic = recovery ? "hermite_recovery_extremum" : "hermite_position_extremum";
      logf("%s: %s axis=%u (%s) segment=%u position=%.17g limit=%.17g sample=%u",
          recovery ? "outside_limits_outward" : "invalid_trajectory",
          diagnostic,
          axis, profile.name.c_str(), check.segment,
          (double(check.value) + offset) / scale, (check.limit + offset) / scale, refusal.sample);
    } else if (refusal.kind == LimitRefusal::Kind::Velocity) {
      logf("invalid_trajectory: hermite_velocity_extremum axis=%u (%s) segment=%u peak_counts_per_s=%.17Lg sample=%u",
          axis, profile.name.c_str(), check.segment, check.value, refusal.sample);
    } else {
      logf("invalid_trajectory: axis=%u (%s) segment=%u acceleration=%.17Lg %s/s^2 limit=%.17g %s/s^2%s sample=%u",
          axis, profile.name.c_str(), check.segment, check.value / scale,
          opt.axis[axis].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY ? "rad" : "m",
          *profile.max_acceleration,
          opt.axis[axis].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY ? "rad" : "m",
          check.failure == rosie::rtcore::interpolation::Failure::MixedFeedforward ? " mixed qd presence" : "", refusal.sample);
    }
  };
  struct PreparedFrame {
    uint64_t id = 0, generation = 0;
    uint32_t mask = 0;
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> offsets{};
  };
  // Measured store geometry: the existing TrajectoryStore has three slots.
  std::array<PreparedFrame, rosie::rtcore::kResidentPlanSlots> prepared_frames{};
  trajectory_store.invalidate(pending_generation);
  auto bulk_axes_ready = [&](uint32_t mask) {
    // Inert preparation may overlap Home; RT still checks Home and the final
    // coordinate/continuity at Start before acquiring permission to execute.
    if (!mask || (mask & ~configured_mask) || !bus_ready.load() ||
        !lease_is_valid_now() || safety_fault_mask.load() ||
        verified_configuration_epoch.load() != configuration_epoch.load() ||
        (control_policy.config_verified_mask.load() & mask) != mask) return false;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
      if ((mask & (1u << axis)) && !axis_profiles[axis].limits.valid) return false;
    return true;
  };
  while (bulk_running.load() && !g_stop.load()) {
    const uint64_t generation = control_policy.generation.load();
    const uint64_t reset = bulk_reset_epoch.load();
    if (generation != pending_generation || reset != reset_seen) {
      pending_upload.reset();
      pending_generation = generation;
      reset_seen = reset;
      trajectory_store.invalidate(generation);
    }
    BulkJob job{};
    if (!bulk_jobs.pop(job)) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    auto* mh = reinterpret_cast<const gradient::ipc::v1::MsgHeader*>(job.message.data());
    const uint8_t* payload = job.message.data() + sizeof(*mh);
    rosie::protocol::CommandResult command_result{mh->seq, 0, job.generation, mh->type,
        rosie::protocol::result_rejected, rosie::protocol::reason_not_ready, 0};
    if (job.generation == pending_generation && job.reset_epoch == reset_seen) {
      switch (mh->type) {
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_BEGIN: {
                  const auto& cmd = *reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryBeginV1*>(payload);
                  pending_upload.reset();
                  command_result.handle = cmd.traj_id; command_result.axis_mask = cmd.axis_mask;
                  command_result.reason = rosie::protocol::reason_invalid_trajectory;
                  if (!cmd.traj_id || !cmd.axis_mask || (cmd.axis_mask & ~configured_mask) ||
                      cmd.expected_points < 2 || cmd.expected_points > kMaxTrajectoryPoints || cmd.flags || cmd.reserved0)
                    break;
                  if (!bulk_axes_ready(cmd.axis_mask)) { command_result.reason = rosie::protocol::reason_not_ready; break; }
                  pending_upload.slot = trajectory_store.begin(cmd.traj_id, pending_generation,
                      cmd.axis_mask, cmd.expected_points);
                  if (pending_upload.slot < 0) { command_result.reason = rosie::protocol::reason_capacity; break; }
                  pending_upload.points = trajectory_store.writing_data(pending_upload.slot);
                  pending_upload.active = true; pending_upload.traj_id = cmd.traj_id;
                  pending_upload.axis_mask = cmd.axis_mask; pending_upload.expected_points = cmd.expected_points;
                  for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
                    upload_offsets[axis] = active_reference_offset(axis);
                  command_result.result = rosie::protocol::result_accepted;
                  command_result.reason = rosie::protocol::reason_none;
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_POINT: {
                  const auto& cmd = *reinterpret_cast<const gradient::ipc::v1::TrajectoryPointV1*>(payload);
                  command_result.handle = cmd.traj_id;
                  command_result.reason = rosie::protocol::reason_invalid_trajectory;
                  bool valid = pending_upload.active && cmd.traj_id == pending_upload.traj_id &&
                      cmd.axis_mask == pending_upload.axis_mask && cmd.point_index == pending_upload.point_count &&
                      cmd.point_index < pending_upload.expected_points && !(cmd.io_high & ~65535u) &&
                      !(cmd.flags & ~(gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY | gradient::ipc::v1::TRAJ_POINTF_LAST_POINT | 0x00ffff00u));
                  if (!valid) { pending_upload.reset(); revoke_outputs(); break; }
                  const uint32_t io_mask=((cmd.flags>>gradient::ipc::v1::IO_MARKER_MASK_SHIFT)&255) | (((cmd.io_high>>0)&255)<<8);
                  const uint32_t io_values=((cmd.flags>>gradient::ipc::v1::IO_MARKER_VALUE_SHIFT)&255) | (((cmd.io_high>>8)&255)<<8);
                  if ((io_values & ~io_mask) || (io_mask >> opt.cell_io.output_count)) {
                    command_result.reason=rosie::protocol::reason_io_not_configured; pending_upload.reset(); break;
                  }
                  if ((cmd.monitor_values & ~cmd.monitor_mask) || (cmd.monitor_mask >> opt.cell_io.input_count)) {
                    command_result.reason=rosie::protocol::reason_io_not_configured; pending_upload.reset(); break;
                  }
                  if ((cmd.wait_timeout_ns && (cmd.wait_input>=opt.cell_io.input_count || cmd.wait_value>1 ||
                      cmd.wait_stable_ns>cmd.wait_timeout_ns || !(cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY))) ||
                      (!cmd.wait_timeout_ns && (cmd.wait_stable_ns || cmd.wait_input || cmd.wait_value))) {
                    pending_upload.reset(); break;
                  }
                  bool torch=false;
                  bool indicator=false;
                  for(unsigned i=0;i<opt.cell_io.output_count;++i) if((io_mask&(1u<<i)) && opt.cell_io.outputs[i].indicator) indicator=true;
                  if(indicator) { command_result.reason=rosie::protocol::reason_io_not_configured; pending_upload.reset(); break; }
                  for(unsigned i=0;i<opt.cell_io.output_count;++i) if((io_mask&(1u<<i)) && opt.cell_io.outputs[i].torch) torch=true;
                  if(torch && !opt.cell_io.torch_qualified) { command_result.reason=rosie::protocol::reason_io_torch_unqualified; pending_upload.reset(); break; }
                  const auto* previous = cmd.point_index ? &pending_upload.points[cmd.point_index - 1] : nullptr;
                  valid = !previous || cmd.t_from_start_ns > previous->t_from_start_ns;
                  if (!valid) { pending_upload.reset(); revoke_outputs(); break; }
                  auto& point = pending_upload.points[pending_upload.point_count];
                  point = {};
                  point.t_from_start_ns = cmd.t_from_start_ns; point.axis_mask = cmd.axis_mask; point.flags = cmd.flags; point.io_high = cmd.io_high;
                  point.wait_timeout_ns=cmd.wait_timeout_ns; point.wait_stable_ns=cmd.wait_stable_ns;
                  point.monitor_mask=cmd.monitor_mask; point.monitor_values=cmd.monitor_values;
                  point.wait_input=cmd.wait_input; point.wait_value=cmd.wait_value;
                  for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                    if (!(cmd.axis_mask & (1u << axis))) continue;
                    bool axis_valid = !cmd.wait_timeout_ns || cmd.qd[axis]==0;
                    const auto& limits = axis_profiles[axis].limits;
                    const double scale = opt.axis[axis].counts_per_unit * opt.axis[axis].sign;
                    const double offset = upload_offsets[axis];
                    const double target = controller_target_to_csp_wire_counts(cmd.q[axis] * scale, offset);
                    const double velocity = cmd.qd[axis] * scale;
                    const auto recovery_limits = limits;
                    // Inert exterior prefixes may hold or approach the nearest
                    // boundary. Start rechecks the first point against the held
                    // pose; the output policy guards every emitted step.
                    const double prior_q = previous ? pending_upload.previous_q[axis] : cmd.q[axis];
                    const double recovery_prior = prior_q;
                    const double recovery_target = cmd.q[axis];
                    const double recovery_velocity = cmd.qd[axis];
                    // Preserve the existing invalid-upload inhibition for a
                    // plan that leaves an already valid interval.
                    if (previous && rosie::rtcore::limit_recovery_contains(recovery_limits, recovery_prior) &&
                        !rosie::rtcore::limit_recovery_contains(recovery_limits, recovery_target)) axis_valid = false;
                    bool direction_valid = rosie::rtcore::limit_recovery_step(recovery_limits, recovery_prior, recovery_target) &&
                        (!(cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) ||
                         rosie::rtcore::limit_recovery_velocity(recovery_limits, recovery_target, recovery_velocity));
                    if (previous && (previous->flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) &&
                        (cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY))
                      direction_valid &= rosie::rtcore::limit_recovery_feedforward(recovery_limits, recovery_prior, recovery_target,
                          pending_upload.previous_qd[axis], recovery_velocity);
                    if (!std::isfinite(target) ||
                        target < INT32_MIN || target > INT32_MAX ||
                        ((cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) &&
                         (!limits.allows_velocity(cmd.qd[axis]) || velocity < INT32_MIN || velocity > INT32_MAX))) axis_valid = false;
                    // Segments with velocity at both ends are bounded by the Hermite
                    // extrema check at commit; the chord check below covers the rest,
                    // in unrounded logical coordinates (or the shortest wrapped step).
                    if (previous && cmd.t_from_start_ns > previous->t_from_start_ns &&
                        !((previous->flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) &&
                          (cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY))) {
                      const double dt = double(cmd.t_from_start_ns - previous->t_from_start_ns) / 1e9;
                      const double step = cmd.q[axis] - prior_q;
                      if (!limits.allows_velocity(step / dt)) axis_valid = false;
                    }
                    point.target_counts[axis] = target;
                    point.velocity_counts_per_s[axis] = (cmd.flags & gradient::ipc::v1::TRAJ_POINTF_HAS_VELOCITY) ? velocity : 0;
                    pending_upload.previous_q[axis] = cmd.q[axis];
                    pending_upload.previous_qd[axis] = cmd.qd[axis];
                    if (!axis_valid)
                      pending_upload.refusal.consider(LimitRefusal::Kind::Point, cmd.point_index, axis);
                    if (!direction_valid)
                      pending_upload.refusal.consider(LimitRefusal::Kind::Direction, cmd.point_index, axis);
                    valid &= axis_valid;
                  }
                  if (!valid) {
                    revoke_outputs();
                    report_limit_refusal(first_limit_refusal(pending_upload.point_count + 1), command_result);
                    pending_upload.reset(); break;
                  }
                  ++pending_upload.point_count;
                  command_result.result = UINT32_MAX; // Only the complete upload needs an acceptance result.
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_COMMIT: {
                  const auto& cmd = *reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryControlV1*>(payload);
                  command_result.handle = cmd.traj_id; command_result.axis_mask = pending_upload.axis_mask;
                  command_result.reason = rosie::protocol::reason_invalid_trajectory;
                  if (!pending_upload.active || cmd.traj_id != pending_upload.traj_id || cmd.flags || cmd.reserved0 ||
                      pending_upload.point_count != pending_upload.expected_points) { pending_upload.reset(); break; }
                  if (!bulk_axes_ready(pending_upload.axis_mask)) { pending_upload.reset(); command_result.reason = rosie::protocol::reason_not_ready; break; }
                  const auto refusal = first_limit_refusal(pending_upload.point_count);
                  if (refusal.kind != LimitRefusal::Kind::None) {
                    report_limit_refusal(refusal, command_result);
                    pending_upload.reset();
                    break;
                  }
                  // A wait may only freeze a stationary interval; reject a
                  // barrier embedded in moving interpolation before publication.
                  bool waits_valid=true;
                  for(uint32_t i=0;i<pending_upload.point_count;++i) {
                    const auto& p=pending_upload.points[i];
                    if(!p.wait_timeout_ns) continue;
                    if(i+1>=pending_upload.point_count) { waits_valid=false; break; }
                    const auto& n=pending_upload.points[i+1];
                    for(uint32_t a=0;a<opt.num_axes;++a) if(p.axis_mask&(1u<<a))
                      if(p.target_counts[a]!=n.target_counts[a] || p.velocity_counts_per_s[a]!=0 || n.velocity_counts_per_s[a]!=0)
                        waits_valid=false;
                  }
                  if(!waits_valid) { pending_upload.reset(); break; }
                  uint32_t next_io=UINT32_MAX, next_wait=UINT32_MAX;
                  for(uint32_t i=pending_upload.point_count;i>0;--i) {
                    auto& point=pending_upload.points[i-1];
                    if(((point.flags>>gradient::ipc::v1::IO_MARKER_MASK_SHIFT)&255) | (((point.io_high>>0)&255)<<8)) next_io=i-1;
                    point.next_io_index=next_io;
                    if(point.wait_timeout_ns) next_wait=i-1;
                    point.next_wait_index=next_wait;
                  }
                  const int slot = trajectory_store.commit(pending_upload.slot) ? pending_upload.slot : -1;
                  for (auto& frame : prepared_frames)
                    if (slot >= 0 && frame.id == cmd.traj_id && frame.generation == pending_generation) frame = {};
                  if (slot >= 0) prepared_frames[slot] = {cmd.traj_id, pending_generation,
                                                        pending_upload.axis_mask, upload_offsets};
                  command_result.result = slot >= 0 ? rosie::protocol::result_prepared : rosie::protocol::result_rejected;
                  command_result.reason = slot >= 0 ? rosie::protocol::reason_none : rosie::protocol::reason_capacity;
                  pending_upload.reset();
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_START: {
                  rosie::protocol::TrajectoryHandle cmd{};
                  command_result.reason = rosie::protocol::reason_invalid_command;
                  if (!rosie::protocol::decode_trajectory_handle(payload, mh->bytes - sizeof(*mh), cmd) || !cmd.handle)
                    break;
                  if (verified_configuration_epoch.load() != configuration_epoch.load() ||
                      (control_policy.config_verified_mask.load() & configured_mask) != configured_mask) {
                    command_result.handle = cmd.handle;
                    command_result.reason = rosie::protocol::reason_not_ready;
                    break;
                  }
                  bool frame_current = true;
                  for (const auto& frame : prepared_frames) {
                    if (frame.id != cmd.handle || frame.generation != pending_generation) continue;
                    for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
                      if ((frame.mask & (1u << axis)) &&
                          frame.offsets[axis] != active_reference_offset(axis))
                        frame_current = false;
                  }
                  if (!frame_current) {
                    // A concurrent Home may establish a different count frame
                    // after this inert upload; never execute its old projection.
                    trajectory_store.discard(pending_generation, cmd.handle);
                    command_result.handle = cmd.handle;
                    command_result.reason = rosie::protocol::reason_not_ready;
                    break;
                  }
                  command_result.handle = cmd.handle; command_result.reason = rosie::protocol::reason_unknown_handle;
                  if (trajectory_store.start(cmd.handle, pending_generation, mh->seq))
                    command_result.result = UINT32_MAX; // The cyclic owner acknowledges actual activation.
                  break;
                }
                case rosie::protocol::msg_trajectory_discard: {
                  rosie::protocol::TrajectoryHandle cmd{};
                  command_result.reason = rosie::protocol::reason_invalid_command;
                  if (!rosie::protocol::decode_trajectory_handle(payload, mh->bytes - sizeof(*mh), cmd) || !cmd.handle)
                    break;
                  command_result.handle = cmd.handle;
                  const auto discarded = trajectory_store.discard(pending_generation, cmd.handle);
                  const bool in_use = discarded == decltype(trajectory_store)::DiscardResult::InUse;
                  command_result.result = in_use ? rosie::protocol::result_rejected : rosie::protocol::result_accepted;
                  command_result.reason = in_use ? rosie::protocol::reason_mode_conflict : rosie::protocol::reason_none;
                  break;
                }
                case gradient::ipc::v1::MSG_CMD_TRAJECTORY_ABORT: {
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdTrajectoryControlV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdTrajectoryControlV1*>(payload);
                    command_result.handle = cmd->traj_id;
                    if (cmd->flags || cmd->reserved0) {
                      command_result.reason = rosie::protocol::reason_invalid_command;
                      break;
                    }
                    // A prepared/ready handle must become unstartable before
                    // acknowledgement. CAS preserves storage if RT activation
                    // wins, in which case only RT may retire it after abort.
                    const auto cancelled = trajectory_store.abort(pending_generation, cmd->traj_id);
                    for (size_t i = 0; i < cancelled.count; ++i) {
                      const auto& start = cancelled.starts[i];
                      const rosie::protocol::CommandResult result{start.sequence, start.id,
                          pending_generation, rosie::protocol::msg_trajectory_start,
                          rosie::protocol::result_rejected, rosie::protocol::reason_not_ready,
                          start.axis_mask};
                      if (!bulk_results.push(result)) revoke_outputs();
                    }
                    if (pending_upload.active &&
                        (cmd->traj_id == 0 || cmd->traj_id == pending_upload.traj_id)) {
                      pending_upload.reset();
                    }
                    command_result.result = rosie::protocol::result_accepted;
                    command_result.reason = rosie::protocol::reason_none;
                  }
                  break;
                }

        default: break;
      }
    }
    if (command_result.result != UINT32_MAX && !bulk_results.push(command_result)) revoke_outputs();
  }
  // Drain abandoned publications before the accept owner creates a successor.
  BulkJob abandoned{};
  while (bulk_jobs.pop(abandoned)) {}
}
