#pragma once

// Operator wording for suspected slip: repeated same-direction movement in one
// braked period points at the brake itself, not a drive delay parameter.
// counts_per_rad is zero for a linear axis, which then reports counts only.
inline std::string brake_slip_message(uint32_t axis, const rosie::rtcore::BrakeHoldEvidence& evidence,
                                      double counts_per_rad) {
  char text[256];
  char angle[48] = "";
  if (counts_per_rad > 0.0)
    std::snprintf(angle, sizeof(angle), ", %.4f deg",
                  double(evidence.same_direction_displacement_counts) / counts_per_rad * 360.0 / kCompletionWrapTwoPi);
  std::snprintf(text, sizeof(text),
      "Brake may be slipping: J%u moved %u times in the same direction (total %lld counts%s) "
      "while the brake was applied - inspect the brake.",
      axis + 1, evidence.same_direction_count,
      static_cast<long long>(evidence.same_direction_displacement_counts), angle);
  return text;
}

inline std::string brake_hold_message(uint32_t axis, const rosie::rtcore::BrakeHoldEvidence& evidence,
                                      double counts_per_rad) {
  if (!evidence.warning_sequence) return "";
  return evidence.slip ? brake_slip_message(axis, evidence, counts_per_rad)
                       : rosie::rtcore::brake_warning_message(evidence.torque_on);
}

// Native metrics retain the latest warning's trip sample; neither a missing
// signal nor a timer derived state is presented as physical brake confirmation.
// A warning is never a fault: it implies no reset, restart or Arm refusal.
// The energized-window fields are recorded movement that was never judged.
inline void append_brake_hold_json(std::ostream& out, const rosie::rtcore::BrakeHoldEvidence& evidence,
                                   const AxisDriveProfile& profile, uint32_t axis, double counts_per_rad) {
  const bool warned = evidence.warning_sequence != 0;
  out << "\"brake_hold_diagnostic\":{\"observed_ns\":" << evidence.observed_ns
      << ",\"severity\":\"" << (warned ? "warning" : "none") << "\""
      << ",\"warning_sequence\":" << evidence.warning_sequence
      << ",\"kind\":\"" << (!warned ? "none" : evidence.slip ? "slip" : "settle") << "\""
      << ",\"transition\":\"" << (!warned ? "none" : evidence.torque_on ? "torque_on" : "torque_off") << "\""
      << ",\"message\":\"" << brake_hold_message(axis, evidence, counts_per_rad) << "\""
      << ",\"same_direction_count\":" << evidence.same_direction_count
      << ",\"same_direction_displacement_counts\":" << evidence.same_direction_displacement_counts
      << ",\"torque_off_ns\":" << evidence.torque_off_ns
      << ",\"torque_off_anchor_counts\":" << evidence.torque_off_anchor_counts
      << ",\"energized_peak_displacement_counts\":" << evidence.energized_peak_displacement_counts
      << ",\"energized_peak_velocity_counts_per_s\":" << evidence.energized_peak_velocity_counts_per_s
      << ",\"peak_displacement_counts\":" << evidence.peak_displacement_counts
      << ",\"peak_velocity_counts_per_s\":" << evidence.peak_velocity_counts_per_s
      << ",\"settled\":" << (evidence.settled ? "true" : "false")
      << ",\"settled_displacement_counts\":" << evidence.settled_displacement_counts
      << ",\"release_delay_ns\":" << profile.brake.release_delay_ns
      << ",\"cause\":\"" << (evidence.observed_ns ? rosie::rtcore::brake_guard_failure_name(evidence.cause) : "none") << "\""
      << ",\"phase\":\"" << (profile.brake.transition.enabled() ? rosie::rtcore::brake_guard_phase_name(evidence.phase) : "legacy") << "\""
      << ",\"anchor_counts\":" << evidence.anchor_counts << ",\"feedback_counts\":" << evidence.feedback_counts
      << ",\"displacement_counts\":" << evidence.displacement_counts
      << ",\"velocity_counts_per_s\":" << evidence.velocity_counts_per_s
      << ",\"velocity_valid\":" << (evidence.velocity_valid ? "true" : "false")
      << ",\"transition_caps_valid\":" << (evidence.transition_caps_valid ? "true" : "false")
      << ",\"effective_velocity_cap_counts_per_s\":" << evidence.effective_velocity_cap_counts_per_s
      << ",\"effective_displacement_cap_counts\":" << evidence.effective_displacement_cap_counts
      << ",\"tolerance_counts\":" << profile.brake.hold_displacement_tolerance_counts
      << ",\"hold_delay_ns\":" << profile.brake.hold_delay_ns
      << ",\"statusword\":" << evidence.statusword << ",\"state\":" << evidence.state
      << ",\"hold_window_active\":" << (evidence.hold_window_active ? "true" : "false")
      << ",\"released_signal_semantic\":\"" << (profile.brake.feedback_required ? profile.brake_released_semantic : "none") << "\""
      << ",\"released_signal_bit\":" << profile.brake_released_bit
      << ",\"released_signal_valid\":" << (evidence.released_valid ? "true" : "false")
      << ",\"released_signal_value\":" << (evidence.released ? "true" : "false") << "},";
}

// Latest supervisor event. An Arm-ending event is current until the next Arm
// request; disarm motion is a warning. Neither latches a fault or needs reset.
inline void append_drive_supervisor_json(std::ostream& out, const rosie::rtcore::DriveSupervisorEvidence& e,
                                         uint32_t axis, double counts_per_unit, bool rotary, uint64_t arm_requests) {
  const bool any = e.sequence != 0;
  out << "\"drive_supervisor\":{\"observed_ns\":" << e.observed_ns
      << ",\"severity\":\"" << (!any ? "none" : e.arm_ended ? "arm_ended" : "warning") << "\""
      << ",\"event_sequence\":" << e.sequence
      << ",\"kind\":\"" << rosie::rtcore::drive_supervisor_kind_name(e.kind) << "\""
      << ",\"cause\":\"" << rosie::rtcore::arm_cancel_cause_name(e.cause) << "\""
      << ",\"message\":\"" << (any ? rosie::rtcore::drive_supervisor_message(e, axis, counts_per_unit, rotary) : std::string()) << "\""
      << ",\"statusword\":" << e.statusword
      << ",\"arm_ended\":" << (e.arm_ended ? "true" : "false")
      << ",\"current\":" << (any && e.arm_ended && e.arm_request_sequence == arm_requests ? "true" : "false")
      << ",\"elapsed_ns\":" << e.elapsed_ns << ",\"bound_ns\":" << e.bound_ns
      << ",\"displacement_counts\":" << e.displacement_counts << ",\"bound_counts\":" << e.bound_counts << "},";
}

// Serialize the worker's correlated first failure, so cleanup cannot replace
// the failed operation or hide a deadline behind generic cancellation.
inline void append_native_home_failure_json(std::ostream& out, const MachineState::NativeHomeFailure& failure) {
  if (!failure.abort_code) { out << "null"; return; }
  out << "{\"job\":" << failure.identity.job
      << ",\"generation\":" << failure.identity.generation
      << ",\"configuration_epoch\":" << failure.identity.configuration_epoch
      << ",\"home_epoch\":" << failure.identity.home_epoch
      << ",\"axis\":" << failure.axis << ",\"step\":" << failure.step
      << ",\"operation\":\"" << failure.operation
      << "\",\"reason\":\"" << failure.reason
      << "\",\"cancellation_reason\":\"" << rosie::rtcore::commissioning_reason_name(failure.cancellation_reason)
      << "\",\"abort_code\":" << failure.abort_code << "}";
}

// Serialize only the cycle's coherent I/O snapshot. Never infer feedback from
// intent or a submitted output byte, including when publication is unavailable.
inline void append_cell_io_json(std::ostream& out, const gradient::ipc::v1::IoStateV1& s) {
  out << "\"io\":{\"time_ns\":" << s.time_ns << ",\"cycle\":" << s.cycle
      << ",\"armed\":" << (s.armed?"true":"false") << ",\"reason\":" << s.reason << ",\"inputs\":[";
  for(unsigned i=0;i<s.input_count && i<16;++i) {
    const auto& p=s.inputs[i]; if(i) out<<',';
    out<<"{\"name\":\""<<reinterpret_cast<const char*>(p.name)<<"\",\"value\":"<<(p.value?"true":"false")
       <<",\"valid\":"<<(p.valid?"true":"false")<<",\"observed_ns\":"<<p.observed_ns
       <<",\"bit\":"<<p.bit<<",\"fast\":"<<(p.fast?"true":"false")<<'}';
  }
  out<<"],\"outputs\":[";
  for(unsigned i=0;i<s.output_count && i<16;++i) {
    const auto& p=s.outputs[i]; if(i) out<<',';
    out<<"{\"name\":\""<<reinterpret_cast<const char*>(p.name)<<"\",\"intent\":"<<(p.intent?"true":"false")
       <<",\"commanded\":"<<(p.commanded?"true":"false")<<",\"readback\":"<<(p.readback?"true":"false")
       <<",\"valid\":"<<(p.valid?"true":"false")<<",\"expiry_ns\":"<<p.expiry_ns
       <<",\"observed_ns\":"<<p.observed_ns<<",\"changed_ns\":"<<p.changed_ns
       <<",\"marker_sample_ns\":"<<p.marker_sample_ns<<",\"marker_cycle\":"<<p.marker_cycle
       <<",\"bit\":"<<p.bit<<",\"torch\":"<<(p.torch?"true":"false")<<'}';
  }
  out<<"]},";
}

// Publish the admitted raw receipt alongside escaped decoded columns so neither
// high bits nor validity disappear between native metrics and public status.
inline void append_drive_alarm_json(std::ostream& out, const rosie::rtcore::DriveAlarmName& alarm,
                                   const rosie::rtcore::AlarmAuxSnapshot& aux) {
  auto quote = [&](const std::string& value) {
    out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << c;
      else if (c >= 0x20) out << c;
      else { char escaped[7]; std::snprintf(escaped, sizeof(escaped), "\\u%04x", c); out << escaped; }
    }
    out << '"';
  };
  out << "\"drive_alarm\":"; quote(alarm.code);
  out << ",\"drive_alarm_text\":"; quote(alarm.text);
  out << ",\"drive_alarm_aux_code\":" << aux.alarm_aux_code
      << ",\"drive_alarm_aux_valid\":" << (aux.alarm_aux_valid ? "true" : "false") << ',';
}

#if GRADIENT_HAVE_ECRT
// IgH ownership: only this publisher queries drive/master diagnostics once per
// 100000000 ns period. ecrt.h (1.6.9) marks master_state master_any,rt_safe and
// slave_config_state master_op,rt_safe; it says "If the state of process data
// exchange shall be monitored in realtime, ecrt_domain_state() should be used."
// Those annotations alone are not a general threading guarantee. In the pinned
// lib/master.c and lib/slave_config.c these const-handle queries only read fd /
// index and issue ioctls with caller-owned output; no userlib object is mutated.
// master/ioctl.c:ec_ioctl_sc_state explicitly needs no master_sem because the
// configuration cannot be deleted meanwhile. Activated handles are immutable;
// this non-RT try-lock prevents teardown and avoids waiting behind mailbox work.
// The cyclic owner retains receive/process/queue/send and readiness. It never
// acquires this lock while cycling; no claim is made for other IgH entry points.
void MachineState::collect_drive_diagnostics() {
  if (!diagnostic_handles_ready.load(std::memory_order_acquire)) return;
  std::unique_lock<std::mutex> lifetime(shared_master_sdo_mutex, std::try_to_lock);
  if (!lifetime.owns_lock()) return;
  auto* handle = shared_master.load(std::memory_order_acquire);
  if (!handle) return;
  DriveDiagnostics sample{};
  const auto topology_generation = topology_monitor.snapshot.generation;
  sample.topology_admitted = topology_monitor.sample(handle, opt, false);
  // A scan in progress has no coherent ring positions yet. Retain the previous
  // complete diagnostic sample; current cyclic WKC still protects every axis.
  // Failed identity reads advance generation and must publish their admission.
  if (topology_monitor.snapshot.generation == topology_generation) return;
  sample.responding_expected = topology_monitor.snapshot.responding_expected;
  if (ecrt_master_state(handle, &sample.master) != 0) sample.master = {};
  // IgH's two diagnostic ioctls are not an atomic topology receipt. If unused
  // devices change between them, wait for a coherent sample instead of revoking
  // healthy axes for an observation race. Cyclic exchange checks remain live.
  if (sample.topology_admitted && sample.master.slaves_responding != sample.responding_expected) return;
  topology_snapshot.publish(topology_monitor.snapshot);
  uint32_t online = 0, operational = 0;
  for (uint32_t i = 0; i < opt.num_axes; ++i) {
    if (diagnostic_sc[i] && ecrt_slave_config_state(diagnostic_sc[i], &sample.slaves[i]) != 0)
      sample.slaves[i] = {};
    online += sample.slaves[i].online ? 1 : 0;
    operational += sample.slaves[i].operational ? 1 : 0;
  }
  sample.sampled_ns = now_monotonic_ns();
  drive_diagnostics.publish(sample);
  drive_diagnostics_generation.fetch_add(1, std::memory_order_release);
  lifetime.unlock();
  const auto phases = diagnostic_phase_requests.exchange(0);
  for (uint32_t bit = 1; bit <= 2; bit <<= 1) {
    if (!(phases & bit)) continue;
    const char* phase = bit == 1 ? "startup_reset" : "startup_timeout";
    logf("EtherCAT phase=%s cycle=%llu elapsed_ms=%llu link_up=%u responding=%u/%u online=%u/%u "
         "operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
         phase, static_cast<unsigned long long>(rt_cycle_counter.load()),
         static_cast<unsigned long long>(latest_feedback.startup_elapsed_ms.load()),
         sample.master.link_up ? 1u : 0u, sample.master.slaves_responding, sample.responding_expected,
         online, opt.num_axes, operational, opt.num_axes, sample.master.al_states,
         static_cast<unsigned>(latest_feedback.domain_wc_state.load()),
         latest_feedback.wkc_actual.load(), latest_feedback.wkc_expected.load());
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      logf("EtherCAT phase=%s axis=%u slave_pos=%u online=%u operational=%u al=%s(0x%x)",
           phase, i, static_cast<unsigned>(opt.slave_position[i]),
           sample.slaves[i].online ? 1u : 0u, sample.slaves[i].operational ? 1u : 0u,
           al_state_label(static_cast<uint8_t>(sample.slaves[i].al_state)),
           static_cast<unsigned>(sample.slaves[i].al_state));
    }
  }
}
#endif

// Drain at most the ring capacity per call; slow storage never delays the RT
// writer. Only this reader formats snapshots, including properly escaped keys.
void MachineState::drain_cycle_capture(FILE* file) {
  if (!file || capture_write_failed.load()) return;
  rosie::rtcore::CycleCaptureRecord sample;
  for (size_t n = 0; n < rosie::rtcore::kCycleCaptureCapacity && cycle_capture.pop(sample); ++n) {
    std::ostringstream line;
    line << "{\"t_ns\":" << sample.t_ns << ",\"seq\":" << sample.seq
         << ",\"cycle\":" << sample.cycle << ",\"work_ns\":" << sample.work_ns;
      line << ",\"cycle_jitter_ns\":" << +sample.cycle_jitter_ns;
      line << ",\"active_traj_id\":" << +sample.active_traj_id;
      line << ",\"active_command_seq\":" << +sample.active_command_seq;
      line << ",\"jog_generation\":" << +sample.jog_generation;
      line << ",\"app_grant_generation\":" << +sample.app_grant_generation;
      line << ",\"control_generation\":" << +sample.control_generation;
      line << ",\"configuration_epoch\":" << +sample.configuration_epoch;
      line << ",\"home_epoch\":" << +sample.home_epoch;
      line << ",\"grant_deadline_host_ns\":" << +sample.grant_deadline_host_ns;
      line << ",\"jog_input_deadline_host_ns\":" << +sample.jog_input_deadline_host_ns;
      line << ",\"jog_source_sequence\":" << +sample.jog_source_sequence;
      line << ",\"bus_submission_return_code\":" << +sample.bus_submission_return_code;
      line << ",\"armed\":" << (sample.armed ? "true" : "false");
      line << ",\"axis_enable_mask\":" << +sample.axis_enable_mask;
      line << ",\"active_mode\":" << +sample.active_mode;
      line << ",\"state\":" << +sample.state;
      line << ",\"current_point_index\":" << +sample.current_point_index;
      line << ",\"queue_depth\":" << +sample.queue_depth;
      line << ",\"last_event_code\":" << +sample.last_event_code;
      line << ",\"underrun_count\":" << +sample.underrun_count;
      line << ",\"stale_command_flag\":" << +sample.stale_command_flag;
      line << ",\"motion_done\":" << (sample.motion_done ? "true" : "false");
      line << ",\"capability_flags\":" << +sample.capability_flags;
      line << ",\"jog_session_open\":" << (sample.jog_session_open ? "true" : "false");
      line << ",\"jog_has_input\":" << (sample.jog_has_input ? "true" : "false");
      line << ",\"jog_axis_mask\":" << +sample.jog_axis_mask;
      line << ",\"jog_ramping\":" << (sample.jog_ramping ? "true" : "false");
      line << ",\"grant_active\":" << (sample.grant_active ? "true" : "false");
      line << ",\"grant_axis_mask\":" << +sample.grant_axis_mask;
      line << ",\"safety_fault_mask\":" << +sample.safety_fault_mask;
      line << ",\"execution_fault_reasons\":" << +sample.execution_fault_reasons;
      line << ",\"submitted_axis_mask\":" << +sample.submitted_axis_mask;
      line << ",\"bus_submission_operation\":" << +sample.bus_submission_operation;
      line << ",\"pdo_fresh_axis_mask\":" << +sample.pdo_fresh_axis_mask;
      line << ",\"config_verified_mask\":" << +sample.config_verified_mask;
      line << ",\"home_valid_mask\":" << +sample.home_valid_mask;
      line << ",\"service_mode_axis_mask\":" << +sample.service_mode_axis_mask;
      line << ",\"wkc_actual\":" << +sample.wkc_actual;
      line << ",\"wkc_expected\":" << +sample.wkc_expected;
      line << ",\"master_state\":" << +sample.master_state;
      line << ",\"io_configured\":" << (sample.io_configured ? "true" : "false");
      line << ",\"io_input_word\":" << +sample.io_input_word;
      line << ",\"io_output_word\":" << +sample.io_output_word;
      line << ",\"io_input_valid\":" << (sample.io_input_valid ? "true" : "false");
      line << ",\"io_armed\":" << (sample.io_armed ? "true" : "false");
      line << ",\"io_reason\":" << +sample.io_reason;
    line << ",\"ax\":[";
    for (uint32_t i = 0; i < sample.axes; ++i) {
      if (i) line << ',';
      const auto& axis = sample.ax[i];
      line << "{\"i\":" << i << ",\"p\":" << axis.position << ",\"tp\":" << axis.target
           << ",\"sw\":" << axis.statusword << ",\"er\":" << axis.error_code
           << ",\"mfr\":" << axis.manufacturer_error_code
           << ",\"velocity_actual_counts_per_s\":" << axis.velocity_actual_counts_per_s
           << ",\"following_error_counts\":" << axis.following_error_counts
           << ",\"velocity_actual_valid\":" << (axis.velocity_actual_valid ? "true" : "false")
           << ",\"following_error_valid\":" << (axis.following_error_valid ? "true" : "false")
           << ",\"di_bits\":" << axis.di_bits << ",\"di_valid\":" << (axis.di_valid ? "true" : "false")
           << ",\"external_enable_active\":" << (axis.external_enable_active ? "true" : "false")
           << ",\"external_enable_valid\":" << (axis.external_enable_valid ? "true" : "false");
      line << ",\"coordinate_counts\":" << +axis.coordinate_counts;
      line << ",\"coordinate_epoch\":" << +axis.coordinate_epoch;
      line << ",\"pdo_observed_time_ns\":" << +axis.pdo_observed_time_ns;
      line << ",\"collision_peak_following_error_counts\":" << +axis.collision_peak_following_error_counts;
      line << ",\"collision_trip_count\":" << +axis.collision_trip_count;
      line << ",\"collision_samples\":" << +axis.collision_samples;
      line << ",\"collision_last_trip_peak_following_error_counts\":" << +axis.collision_last_trip_peak_following_error_counts;
      line << ",\"absolute_source_counts\":" << +axis.absolute_source_counts;
      line << ",\"coordinate_reason\":" << +axis.coordinate_reason;
      line << ",\"calibration_valid\":" << +axis.calibration_valid;
      line << ",\"position_state\":" << +axis.position_state;
      line << ",\"audit_reason\":" << +axis.audit_reason;
      line << ",\"audit_last_verified_ns\":" << +axis.audit_last_verified_ns;
      line << ",\"audit_started_ns\":" << +axis.audit_started_ns;
      line << ",\"audit_completed_ns\":" << +axis.audit_completed_ns;
      line << ",\"audit_coherence_error_counts\":" << +axis.audit_coherence_error_counts;
      line << ",\"target_velocity_counts_per_s\":" << +axis.target_velocity_counts_per_s;
      line << ",\"collision_armed\":" << (axis.collision_armed ? "true" : "false");
      line << ",\"collision_torque_cycles\":" << +axis.collision_torque_cycles;
      line << ",\"collision_following_error_cycles\":" << +axis.collision_following_error_cycles;
      line << ",\"collision_peak_torque_raw\":" << +axis.collision_peak_torque_raw;
      line << ",\"collision_last_trip_quantity\":" << +axis.collision_last_trip_quantity;
      line << ",\"collision_trip_sustained_cycles\":" << +axis.collision_trip_sustained_cycles;
      line << ",\"collision_last_trip_peak_torque_raw\":" << +axis.collision_last_trip_peak_torque_raw;
      line << ",\"brake_state\":" << +axis.brake_state;
      line << ",\"native_home_position_offset\":" << +axis.native_home_position_offset;
      line << ",\"native_home_last_abort_code\":" << +axis.native_home_last_abort_code;
      line << ",\"ext_position_error_counts\":" << +axis.ext_position_error_counts;
      line << ",\"ext_multi_turn_lo\":" << +axis.ext_multi_turn_lo;
      line << ",\"ext_multi_turn_hi\":" << +axis.ext_multi_turn_hi;
      line << ",\"max_abs_velocity_actual_counts_per_s\":" << +axis.max_abs_velocity_actual_counts_per_s;
      line << ",\"max_abs_following_error_counts\":" << +axis.max_abs_following_error_counts;
      line << ",\"torque_raw\":" << +axis.torque_raw;
      line << ",\"ext_bus_voltage_raw\":" << +axis.ext_bus_voltage_raw;
      line << ",\"ext_load_rate_raw\":" << +axis.ext_load_rate_raw;
      line << ",\"ext_igbt_temp_raw\":" << +axis.ext_igbt_temp_raw;
      line << ",\"ext_motor_temp_raw\":" << +axis.ext_motor_temp_raw;
      line << ",\"ext_drive_not_ready_bits\":" << +axis.ext_drive_not_ready_bits;
      line << ",\"ext_motor_not_rotating_code\":" << +axis.ext_motor_not_rotating_code;
      line << ",\"ext_valid_mask\":" << +axis.ext_valid_mask;
      line << ",\"mode_display\":" << +axis.mode_display;
      line << ",\"ds402_state\":" << +axis.ds402_state;
      line << ",\"torque_valid\":" << (axis.torque_valid ? "true" : "false");
      line << ",\"coordinate_valid\":" << (axis.coordinate_valid ? "true" : "false");
      line << ",\"coordinate_source_valid\":" << (axis.coordinate_source_valid ? "true" : "false");
      line << ",\"native_home_state\":" << +axis.native_home_state;
      line << ",\"slave_al_state\":" << +axis.slave_al_state;
      line << ",\"slave_online\":" << (axis.slave_online ? "true" : "false");
      line << ",\"slave_operational\":" << (axis.slave_operational ? "true" : "false");
      line << ",\"pdo_fresh\":" << (axis.pdo_fresh ? "true" : "false");
      line << ",\"target_velocity_valid\":" << (axis.target_velocity_valid ? "true" : "false");
      line << ",\"ext_valid\":" << (axis.ext_valid ? "true" : "false");
      const auto& absolute = axis_profiles[i].absolute;
      if (absolute.valid && absolute.field_count) {
        line << ",\"af\":[";
        bool first = true;
        for (uint32_t f = 0; f < absolute.field_count; ++f) {
          const auto& field = absolute.fields[f];
          if (!field.valid || field.key.empty()) continue;
          if (!first) line << ',';
          first = false;
          line << "{\"k\":\"";
          for (unsigned char c : field.key) {
            if (c == '"' || c == '\\') line << '\\' << c;
            else if (c < 0x20) {
              char escaped[7];
              std::snprintf(escaped, sizeof(escaped), "\\u%04x", c);
              line << escaped;
            } else line << c;
          }
          line << "\",\"v\":" << axis.absolute_value[f]
               << ",\"ok\":" << static_cast<unsigned>(axis.absolute_valid[f]) << '}';
        }
        line << ']';
      }
      line << '}';
    }
    line << "]}\n";
    const auto encoded = line.str();
    if (std::fwrite(encoded.data(), 1, encoded.size(), file) != encoded.size()) {
      capture_write_failed.store(true);
      return;
    }
    capture_records_written.fetch_add(1, std::memory_order_relaxed);
  }
  if (std::fflush(file) != 0) capture_write_failed.store(true);
}

// Serialized retention evicts index-only outcomes first under the shared byte
// budget. Only an incoming real dump may evict a pair; refuse indexes that still
// cannot fit, preventing a skipped/refused storm from deleting binary evidence.
bool retain_telemetry(const std::filesystem::path& directory, uint64_t budget,
                      uint32_t count, uint32_t index_count, uint64_t incoming, bool incoming_pair = true) {
  if (incoming > budget) return false;
  struct Entry { std::filesystem::path index, binary; uint64_t bytes; bool pair; };
  std::vector<Entry> retained;
  uint64_t bytes = 0;
  size_t pairs = 0, indexes = 0;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    const auto path = entry.path();
    const auto name = path.filename().string();
    if ((name.size() >= 8 && name.substr(name.size() - 8) == ".bin.tmp") ||
        (name.size() >= 9 && name.substr(name.size() - 9) == ".json.tmp")) {
      std::filesystem::remove(path);
      continue;
    }
    if (path.extension() == ".bin") {
      auto index = path; index.replace_extension(".json");
      if (!std::filesystem::exists(index)) std::filesystem::remove(path);
      continue;
    }
    if (path.extension() != ".json") continue;
    auto binary = path; binary.replace_extension(".bin");
    uint64_t size = std::filesystem::file_size(path);
    const bool pair = std::filesystem::exists(binary);
    if (pair) { size += std::filesystem::file_size(binary); ++pairs; }
    else ++indexes;
    retained.push_back({path, binary, size, pair}); bytes += size;
  }
  std::sort(retained.begin(), retained.end(), [](const Entry& a, const Entry& b) {
    return a.pair != b.pair ? !a.pair : a.index < b.index;
  });
  for (const auto& entry : retained) {
    const bool over_count = pairs + (incoming_pair ? 1 : 0) > count;
    const bool over_indexes = indexes + (incoming_pair ? 0 : 1) > index_count;
    const bool over_bytes = bytes > budget || incoming > budget - bytes;
    if (!over_count && !over_indexes && !over_bytes) break;
    if (entry.pair && !incoming_pair) continue;
    if (!over_bytes && !(entry.pair ? over_count : over_indexes)) continue;
    std::filesystem::remove(entry.binary);
    std::filesystem::remove(entry.index);
    bytes -= entry.bytes;
    if (entry.pair) --pairs;
    else --indexes;
  }
  return bytes <= budget && incoming <= budget - bytes &&
      pairs + (incoming_pair ? 1 : 0) <= count && indexes + (incoming_pair ? 0 : 1) <= index_count;
}

// Off-cycle only: stream a bounded window with per-slot validation. A writer
// overtaking disk output is reported as missing records, never as coherent data.
// Raw records retain every configured axis/absolute field; JSON keys live in
// the header so reader-side conversion can reproduce commissioning JSONL.
bool MachineState::dump_telemetry(const rosie::rtcore::TelemetryDumpRequest& request, bool skipped) {
  using namespace rosie::rtcore;
  if (!request.count || !request.end) return true;
  const auto& trigger = request.triggers[0];
  const char* reason = trigger.reason == 1 ? "fault" : trigger.reason == 2 ? "stop" :
                       trigger.reason == 4 ? "mark" : "exit";
  const auto directory = opt.telemetry_directory.empty() ?
      std::filesystem::path(opt.socket_path).parent_path() / "telemetry" :
      std::filesystem::path(opt.telemetry_directory);
  std::filesystem::path temporary, index_temporary, published_index;
  try {
    std::filesystem::create_directory(directory);
    if (std::filesystem::is_symlink(directory)) throw std::runtime_error("telemetry directory is a symlink");
    const uint64_t end = request.end;
    const uint64_t begin = end > telemetry.capacity() ? end - telemetry.capacity() + 1 : 1;
    // Unverified reservation includes a full header-sized index, so temporary
    // output plus retained pairs cannot exceed the directory byte budget.
    const uint64_t reservation = 2 * kTelemetryDumpHeaderBytes +
        (end - begin + 1) * TelemetryRing::record_bytes(opt.num_axes);
    const bool refused = !skipped && reservation > opt.telemetry_dump_bytes;
    // A refusal leaves room for its index instead of deleting useful evidence
    // for a binary that can never fit. Actual index admission happens below.
    if (!skipped && !refused &&
        !retain_telemetry(directory, opt.telemetry_dump_bytes, opt.telemetry_dump_count, opt.telemetry_index_count, reservation))
      throw std::runtime_error("telemetry dump exceeds retention budget");
    timespec utc{}; ::clock_gettime(CLOCK_REALTIME, &utc);
    tm broken{}; ::gmtime_r(&utc.tv_sec, &broken);
    char date[32]{}; ::strftime(date, sizeof(date), "%Y%m%dT%H%M%S", &broken);
    char fraction[16]{}; std::snprintf(fraction, sizeof(fraction), "%09ld", utc.tv_nsec);
    const auto basename = std::string(date) + "." + fraction + "Z-" +
        std::to_string(telemetry_index_sequence++) + "-" + reason;
    const auto binary = directory / (basename + ".bin");
    const auto index = directory / (basename + ".json");
    temporary = binary.string() + ".tmp"; index_temporary = index.string() + ".tmp";
    std::unique_ptr<FILE, decltype(&std::fclose)> file(nullptr, &std::fclose);
    if (!skipped && !refused) {
      const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
      if (fd < 0) throw std::runtime_error("create telemetry dump");
      file.reset(::fdopen(fd, "wb"));
      if (!file) { ::close(fd); throw std::runtime_error("open telemetry stream"); }
    }
    // Measured envelope: kTelemetryDumpHeaderBytes includes the binary prefix.
    constexpr size_t header_bytes = kTelemetryDumpHeaderBytes;
    std::array<uint8_t, header_bytes> header{};
    if (file && std::fwrite(header.data(), 1, header.size(), file.get()) != header.size())
      throw std::runtime_error("write telemetry header");
    uint64_t first = 0, last = 0, count = 0, missing = 0;
    CycleCaptureRecord record{};
    for (uint64_t sequence = begin; file && sequence <= end; ++sequence) {
      // Unverified I/O chunk bound: at most 64 records (312320 bytes at 16
      // axes) between flush/yield points; no chunk executes on metrics or RT.
      if ((sequence - begin) % 64 == 0) {
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
        if (telemetry_chunk_observer) telemetry_chunk_observer(telemetry_chunk_context);
#endif
        if (std::fflush(file.get()) != 0) throw std::runtime_error("flush telemetry chunk");
        std::this_thread::yield();
      }
      if (!telemetry.read(sequence, record)) { ++missing; continue; }
      if (std::fwrite(&record, 1, TelemetryRing::record_bytes(opt.num_axes), file.get()) != TelemetryRing::record_bytes(opt.num_axes))
        throw std::runtime_error("write telemetry record");
      if (!first) first = sequence;
      last = sequence; ++count;
    }
    auto quote = [](const std::string& value) {
      std::ostringstream out; out << '"';
      for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20 || c >= 0x80) {
          // Native labels carry UTF-8 bytes; non-ASCII bytes are emitted intact.
          if (c >= 0x80) out << c;
          else { char escaped[7]; std::snprintf(escaped, sizeof(escaped), "\\u%04x", c); out << escaped; }
        } else out << c;
      }
      out << '"'; return out.str();
    };
    std::ostringstream metadata;
    metadata << "{\"protocol_major\":" << rosie::protocol::version_major
      << ",\"protocol_minor\":" << rosie::protocol::version_minor
      << ",\"layout_digest\":\"" << gradient::ipc::v1::CycleCaptureRecordV2LayoutDigest
      << "\",\"cycle_period_ns\":" << opt.cycle_ns << ",\"axis_count\":" << opt.num_axes
      << ",\"record_bytes\":" << TelemetryRing::record_bytes(opt.num_axes)
      << ",\"first_sequence\":" << first << ",\"last_sequence\":" << last
      << ",\"window_first_sequence\":" << begin << ",\"window_last_sequence\":" << end
      << ",\"trigger_sequence\":" << trigger.end
      // Unverified post-trigger tail: zero records. Freeze the last observed
      // trigger boundary without waiting for future cycles or extending a storm.
      << ",\"post_trigger_records\":0,\"file\":" << (file ? quote(binary.filename().string()) : "null")
      << ",\"result\":" << quote(skipped ? "dump_skipped" : refused ? "dump_refused_byte_budget" : "dump_written")
      << ",\"writer_reason\":" << quote(telemetry_scheduling_failed.load() ? "scheduler_failed" : "none")
      << ",\"reserved_bytes\":" << reservation
      << ",\"skipped_trigger_drops\":" << telemetry_skipped_trigger_drops.load()
      << ",\"record_count\":" << count << ",\"missing_records\":" << missing
      << ",\"overwritten_records\":" << telemetry.dropped()
      << ",\"trigger\":\"" << reason << "\",\"label\":"
      << quote(std::string(reinterpret_cast<const char*>(trigger.label.data()), trigger.length))
      << ",\"triggers\":[";
    for (size_t i = 0; i < request.count; ++i) {
      if (i) metadata << ',';
      const auto& item = request.triggers[i];
      metadata << "{\"reason\":" << quote(item.reason == 1 ? "fault" : item.reason == 2 ? "stop" : item.reason == 4 ? "mark" : "exit")
        << ",\"fault_reason\":" << item.fault_reason << ",\"axis\":";
      if (item.axis == UINT32_MAX) metadata << "null"; else metadata << item.axis;
      metadata << ",\"sequence\":" << item.end << ",\"label\":"
        << quote(std::string(reinterpret_cast<const char*>(item.label.data()), item.length)) << '}';
    }
    metadata << "],\"absolute_keys\":[";
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (axis) metadata << ',';
      metadata << '[';
      const auto& absolute = axis_profiles[axis].absolute;
      for (uint32_t field = 0; absolute.valid && field < absolute.field_count; ++field) {
        if (field) metadata << ',';
        metadata << quote(absolute.fields[field].valid ? absolute.fields[field].key : "");
      }
      metadata << ']';
    }
    metadata << "]}\n";
    const auto json = metadata.str();
    if (json.size() > header.size() - 16) throw std::runtime_error("telemetry metadata exceeds bound");
    std::memcpy(header.data(), "RTTDMP1\n", 8);
    const uint32_t size = header_bytes, json_size = json.size();
    std::memcpy(header.data() + 8, &size, 4); std::memcpy(header.data() + 12, &json_size, 4);
    std::memcpy(header.data() + 16, json.data(), json.size());
    if (file && (std::fseek(file.get(), 0, SEEK_SET) != 0 ||
        std::fwrite(header.data(), 1, header.size(), file.get()) != header.size() ||
        std::fflush(file.get()) != 0)) throw std::runtime_error("commit telemetry header");
    const bool binary_written = bool(file);
    if (file && std::fclose(file.release()) != 0) throw std::runtime_error("close telemetry dump");
    if (!binary_written) {
      if (json.size() > opt.telemetry_dump_bytes) throw std::runtime_error("telemetry index exceeds byte budget");
      if (!retain_telemetry(directory, opt.telemetry_dump_bytes, opt.telemetry_dump_count, opt.telemetry_index_count, json.size(), false))
        throw std::runtime_error("telemetry index cannot fit without evicting a dump");
    }
    const int index_fd = ::open(index_temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (index_fd < 0) throw std::runtime_error("create telemetry index");
    const auto written = ::write(index_fd, json.data(), json.size());
    const auto closed = ::close(index_fd);
    if (written != static_cast<ssize_t>(json.size()) || closed != 0) throw std::runtime_error("write telemetry index");
    std::filesystem::rename(index_temporary, index);
    published_index = index;
    if (binary_written) {
      std::filesystem::rename(temporary, binary);
      telemetry_dumps_written.fetch_add(1);
    }
    if (refused) telemetry_dump_failures.fetch_add(1);
    return !refused;
  } catch (const std::exception& error) {
    std::error_code ignored;
    if (!temporary.empty()) std::filesystem::remove(temporary, ignored);
    if (!index_temporary.empty()) std::filesystem::remove(index_temporary, ignored);
    if (!published_index.empty()) std::filesystem::remove(published_index, ignored);
    telemetry_dump_failures.fetch_add(1);
    logf("ERROR: telemetry dump failed: %s", error.what());
    return false;
  }
}

// One bounded pass coalesces every admitted trigger into one immutable window.
// Disk work never executes here, preserving commissioning/metrics cadence.
// Feeds each axis's load window from the telemetry records written since the
// last pass, on the metrics thread (docs/decisions/0001). A record that is
// busy or already overwritten is skipped: fewer samples, never a wrong one.
// With no new record the windows clear, so a stopped ring shows no samples
// rather than its last second for ever.
void MachineState::summarize_drive_load() {
  const uint64_t written = telemetry.active() ? telemetry.written() : 0;
  if (written <= drive_load_cursor) {
    for (auto& window : drive_load) window.clear();
    drive_load_cursor = written;
    return;
  }
  // Unverified bound: 4 s of 1 kHz records per pass, so a late pass costs
  // at most a few window lengths of copying, never the whole ring.
  constexpr uint64_t kMaxRecordsPerPass = 4000;
  uint64_t first = drive_load_cursor + 1;
  if (written - drive_load_cursor > kMaxRecordsPerPass) first = written - kMaxRecordsPerPass + 1;
  rosie::rtcore::CycleCaptureRecord record{};
  for (uint64_t sequence = first; sequence <= written; ++sequence) {
    if (!telemetry.read(sequence, record)) continue;
    for (uint32_t i = 0; i < opt.num_axes && i < record.axes; ++i) {
      const auto& axis = record.ax[i];
      drive_load[i].add(record.t_ns, axis.torque_valid != 0, axis.torque_raw,
          axis.following_error_valid != 0, axis.following_error_counts);
    }
  }
  drive_load_cursor = written;
}

void MachineState::drain_telemetry() {
  rosie::rtcore::TelemetryDumpRequest request{};
  rosie::rtcore::TelemetryTrigger trigger{};
  for (size_t n = 0; n < rosie::rtcore::kTelemetryTriggerCapacity && telemetry_cycle_triggers.pop(trigger); ++n)
    request.triggers[request.count++] = trigger;
  for (size_t n = 0; n < rosie::rtcore::kTelemetryTriggerCapacity && telemetry_marks.pop(trigger); ++n)
    request.triggers[request.count++] = trigger;
  for (size_t i = 0; i < request.count; ++i) request.end = std::max(request.end, request.triggers[i].end);
  queue_telemetry_dump(request);
}

// At most one ordinary binary dump is pending/in flight. Busy storms become index-only
// records in the same FIFO, so the next index preserves their order even if
// completion races a new publisher pass. Overflow never blocks running metrics.
// The sole final exit request queues one additional binary. A failed admission
// leaves an index-only fallback for shutdown after joins, never concurrent I/O.
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
// A deterministic clock exercises queue exhaustion without a wall-clock assertion.
uint64_t (*telemetry_exit_clock_test_hook)() = nullptr;
#endif
void MachineState::queue_telemetry_dump(const rosie::rtcore::TelemetryDumpRequest& request) {
  if (!request.count || !request.end) return;
  if (request.count == 1 && request.triggers[0].reason == 8) {
    const auto clock = [] {
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
      if (telemetry_exit_clock_test_hook) return telemetry_exit_clock_test_hook();
#endif
      return now_monotonic_ns();
    };
    // Unverified admission bound: 100000000 ns lets the worker free a FIFO slot
    // while preventing a lost semaphore/writer from stranding the publisher join.
    constexpr uint64_t admission_ns = 100000000;
    const auto start = clock();
    while (!telemetry_dump_requests.push(request)) {
      if (process_exit_code.load() != 0 || clock() - start >= admission_ns) {
        telemetry_dump_failures.fetch_add(1);
        telemetry_exit_fallback = request;
        telemetry_exit_fallback.skipped = true;
        return;
      }
      std::this_thread::yield();
    }
    if (!telemetry_dump_wake.post()) telemetry_dump_failures.fetch_add(1);
    return;
  }
  if (telemetry_dump_busy.exchange(true, std::memory_order_acq_rel)) {
    telemetry_dumps_skipped.fetch_add(1);
    auto skipped = request; skipped.skipped = true;
    if (!telemetry_dump_requests.push(skipped)) telemetry_skipped_trigger_drops.fetch_add(request.count);
    else if (!telemetry_dump_wake.post()) telemetry_dump_failures.fetch_add(1);
  } else {
    if (!telemetry_dump_requests.push(request)) {
      telemetry_trigger_drops.fetch_add(request.count);
      telemetry_dump_busy.store(false, std::memory_order_release);
    } else if (!telemetry_dump_wake.post()) telemetry_dump_failures.fetch_add(1);
  }
}

// Called after publisher and writer joins; failed exit admission still leaves
// an index without racing the worker's retention scan or filename discriminator.
void MachineState::finish_telemetry() {
  const auto request = telemetry_exit_fallback;
  telemetry_exit_fallback = {};
  if (request.count) dump_telemetry(request, true);
}

// Dedicated housekeeping worker owns all dump/index I/O and retention. Join it
// after the publisher so shutdown cannot destroy mappings beneath disk reads.
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
// Inject syscall return codes before startup, preventing silent worker loss
// from escaping the scheduling-failure regression.
int (*telemetry_scheduling_test_hook)(bool batch) = nullptr;
bool telemetry_live_admission_test_hook = false;
#endif
void MachineState::run_telemetry_writer() {
  if (!apply_housekeeping_affinity("telemetry-dump", telemetry_writer_affinity_applied)) return;
  pthread_setname_np(pthread_self(), "telemetry-dump");
  sched_param scheduling{};
  // Unverified disk-worker priority: SCHED_BATCH, nice 19. Live admission fails
  // closed if either setting fails, preventing inherited FIFO disk writes.
  // Simulation continues at inherited settings for observable dump diagnostics.
  int batch_error = 0, nice_error = 0;
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
  if (telemetry_scheduling_test_hook) {
    batch_error = telemetry_scheduling_test_hook(true);
    nice_error = telemetry_scheduling_test_hook(false);
  } else
#endif
  {
    batch_error = pthread_setschedparam(pthread_self(), SCHED_BATCH, &scheduling);
    if (setpriority(PRIO_PROCESS, 0, 19) != 0) nice_error = errno;
  }
  if (batch_error || nice_error) {
    telemetry_scheduling_failed.store(true);
    telemetry_dump_failures.fetch_add(1);
    bool fatal = false;
#if GRADIENT_ECRT_ENABLED && !defined(ROSIE_RT_SIMULATION)
    fatal = true;
#elif defined(ROSIE_TELEMETRY_TEST_HOOK)
    fatal = telemetry_live_admission_test_hook;
#endif
    logf("%s: telemetry-dump scheduling failed: reason=scheduler_failed SCHED_BATCH=%s nice19=%s",
         fatal ? "ERROR" : "WARNING", std::strerror(batch_error), std::strerror(nice_error));
    if (fatal) {
      telemetry_dump_busy.store(false, std::memory_order_release);
      process_exit_code.store(2); g_stop.store(true);
      return;
    }
  }
  for (;;) {
    if (!telemetry_dump_wake.wait()) {
      telemetry_dump_failures.fetch_add(1);
      return;
    }
    rosie::rtcore::TelemetryDumpRequest request{};
    if (telemetry_dump_requests.pop(request)) {
      dump_telemetry(request, request.skipped);
      if (!request.skipped) telemetry_dump_busy.store(false, std::memory_order_release);
    } else if (telemetry_writer_stop.load(std::memory_order_acquire)) {
      break;
    }
  }
}

// The publisher borrows the runtime until shutdown joins its thread.
// File output and observation never wait for mailbox completion.
void MachineState::publish_status(gid_t ipc_gid) {
    std::string published_description;
    std::array<std::string, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_anchor_difference{};
  if (!apply_housekeeping_affinity("metrics/capture", publisher_affinity_applied)) return;
  // Periodic metrics file for dashboards/monitoring. This intentionally does NOT
  // require an IPC client (RTCore is single-client today).
  const std::filesystem::path sock_parent = std::filesystem::path(opt.socket_path).parent_path();
  const std::filesystem::path metrics_path =
      (sock_parent.empty() ? std::filesystem::path("/tmp")
                           : sock_parent) /
      "metrics.json";
    pthread_setname_np(pthread_self(), "metrics");
    std::unique_ptr<FILE, decltype(&std::fclose)> capture_file(nullptr, &std::fclose);
    if (opt.capture_cycles) {
      // Fixed basename in the socket runtime directory; never follow or replace
      // an existing file/symlink. Open successfully before arming the RT writer.
      const auto directory = sock_parent.empty() ? std::filesystem::path(".") : sock_parent;
      const int dirfd = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
      const int fd = dirfd < 0 ? -1 : openat(dirfd, "cycle-capture.jsonl",
          O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
      if (dirfd >= 0) close(dirfd);
      if (fd >= 0) {
        capture_file.reset(fdopen(fd, "w"));
        if (!capture_file) close(fd);
      }
      if (capture_file) (void)cycle_capture.arm(opt.capture_cycles);
      else {
        capture_write_failed.store(true);
        logf("ERROR: cannot create runtime cycle-capture.jsonl: %s", std::strerror(errno));
      }
    }

    uint64_t last_cycles = rt_cycle_counter.load(std::memory_order_relaxed);
    uint64_t last_time_ns = now_monotonic_ns();
    uint64_t last_warn_ns = 0;
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_collision_trips{};
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_brake_trips{};
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_target_lead{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_target_lead_cleared{};
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_brake_settles{};
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_supervisor_events{};
    while (!cycle_exited.load(std::memory_order_acquire)) {
      rosie::rtcore::drain_diagnostics();
      std::this_thread::sleep_for(std::chrono::nanoseconds(rosie::rtcore::kDiagnosticsPublisherPeriodNs));
#if GRADIENT_HAVE_ECRT
      collect_drive_diagnostics();
#endif
      // Flush evidence before timestamping the associated metrics publication.
      // Observing a later metrics sample must not expose a whole-period-old file.
      drain_cycle_capture(capture_file.get());
      drain_telemetry();
      summarize_drive_load();
      // A mailbox acquisition may finish while the remaining JSON is formatted.
      // Freeze its proof and read result before the clock used for age/future
      // qualification; raw observation and anchor offer must share this copy.
      const auto anchor_delta_publication = rosie::rtcore::snapshot_before_clock(
          anchor_delta_restores, opt.num_axes, [] { return now_monotonic_ns(); });
      const auto absolute_publication = rosie::rtcore::snapshot_before_clock(
          sampled_absolute, opt.num_axes, [] { return now_monotonic_ns(); });
      const uint64_t now_ns = absolute_publication.time_ns;

      const uint64_t cycles = rt_cycle_counter.load(std::memory_order_relaxed);
      const int64_t last_jitter = rt_last_jitter_ns.load(std::memory_order_relaxed);
      const int64_t max_abs_jitter = rt_max_abs_jitter_ns.load(std::memory_order_relaxed);
      const uint64_t overruns = rt_overrun_count.load(std::memory_order_relaxed);

      double hz = 0.0;
      if (now_ns > last_time_ns) {
        const uint64_t dc = cycles - last_cycles;
        const uint64_t dt = now_ns - last_time_ns;
        hz = (static_cast<double>(dc) * 1e9) / static_cast<double>(dt);
      }
      last_cycles = cycles;
      last_time_ns = now_ns;

      // Snapshot EtherCAT feedback if available (double-read seq).
      uint32_t wkc_actual = 0;
      uint64_t dc_reference_sync_attempts = 0, dc_reference_sync_time_ns = 0;
      uint64_t dc_reference_sync_interval_ns = 0, dc_reference_sync_max_interval_ns = 0;
      uint32_t wkc_expected = 2 * opt.num_axes;
      uint32_t master_state = gradient::ipc::v1::MASTER_INIT;
      uint32_t responding_slaves = 0;
      uint32_t online_slaves = 0;
      uint32_t operational_slaves = 0;
      uint32_t startup_elapsed_ms = 0;
      uint32_t startup_reset_count = 0;
      uint8_t link_up = 0;
      uint8_t master_al_states = 0;
      uint8_t domain_wc_state = 0;
      uint8_t startup_ready = 0;
      uint8_t startup_passive_active = 0;
      uint8_t startup_skip_domain_queue_active = 0;
      std::array<StartupSdoFeedback, kMaxStartupSdoDescriptors> startup_drive_config_feedback{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_state{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_position_offset{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> reference_drive_frame{};
      uint64_t reference_configuration = 0;
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_last_abort_code{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};

      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_pos_counts{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> mode_display{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_state{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pdo_observed_time_ns{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_epoch{};
      std::array<int64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_counts{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_source_counts{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_reason{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_valid{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_source_valid{};
      uint32_t pdo_fresh_axis_mask = 0;
      uint64_t snapshot_home_epoch = home_epoch.load();
      uint32_t snapshot_home_valid_mask = control_policy.home_valid_mask.load();
      uint32_t snapshot_enabled_mask = 0, snapshot_fault_mask = 0;
      uint32_t snapshot_config_mask = 0;
      uint64_t snapshot_configuration_epoch = 0, snapshot_verified_epoch = 0;
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
      std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> alarm_events{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> manufacturer_error_code{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_actual_counts_per_s{};
      std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> following_error_counts{};
      std::array<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> torque_raw{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_armed{}, collision_torque_cycles{}, collision_error_cycles{}, collision_peak_torque{}, collision_trip_quantity{}, collision_trip_cycles{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_peak_error{}, collision_trip_count{}, collision_samples{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_trip_peak_torque{};
      std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_trip_peak_error{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_actual_valid{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> following_error_valid{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> di_bits{}, di_valid{}, external_enable_active{}, external_enable_valid{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> max_abs_velocity_actual_counts_per_s{};
      std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> max_abs_following_error_counts{};
      std::array<AbsoluteFeedbackAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_feedback{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_al_state{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_online{};
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_operational{};

      bool have_fb = false;
      std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> calibration_valid{}, position_state{};
      // One collision with the writer must not discard the entire 100 ms
      // publication slot. Retry a bounded number of times in this non-RT worker
      // so phase-aligned sampling cannot hide healthy Arm/Stop observations.
      for (unsigned attempt = 0; attempt < 8 && !have_fb; ++attempt) {
        const uint64_t s1 = latest_feedback.seq.load(std::memory_order_acquire);
        if (s1 != 0 && !(s1 & 1u)) {
          wkc_actual = latest_feedback.wkc_actual;
          dc_reference_sync_attempts = latest_feedback.dc_reference_sync_attempts;
          dc_reference_sync_time_ns = latest_feedback.dc_reference_sync_time_ns;
          dc_reference_sync_interval_ns = latest_feedback.dc_reference_sync_interval_ns;
          dc_reference_sync_max_interval_ns = latest_feedback.dc_reference_sync_max_interval_ns;
          wkc_expected = latest_feedback.wkc_expected;
          master_state = latest_feedback.master_state;
          responding_slaves = latest_feedback.responding_slaves;
          online_slaves = latest_feedback.online_slaves;
          operational_slaves = latest_feedback.operational_slaves;
          startup_elapsed_ms = latest_feedback.startup_elapsed_ms;
          startup_reset_count = latest_feedback.startup_reset_count;
          link_up = latest_feedback.link_up;
          master_al_states = latest_feedback.master_al_states;
          domain_wc_state = latest_feedback.domain_wc_state;
          startup_ready = latest_feedback.startup_ready;
          startup_passive_active = latest_feedback.startup_passive_active;
          startup_skip_domain_queue_active = latest_feedback.startup_skip_domain_queue_active;
          startup_drive_config_feedback = latest_feedback.startup_drive_config_feedback;
          native_home_state = latest_feedback.native_home_state;
          native_home_position_offset = latest_feedback.native_home_position_offset;
          reference_configuration = configuration_epoch.load();
          for (uint32_t a = 0; a < opt.num_axes; ++a) reference_drive_frame[a] = absolute_frame_epoch[a].load();
          // Home identity and offset belong to one cyclic publication. Mixing a
          // later completed epoch with an older offset could persist false trust.
          snapshot_home_epoch = home_epoch.load();
          snapshot_home_valid_mask = control_policy.home_valid_mask.load();
          snapshot_enabled_mask = axis_enable_mask.load();
          snapshot_fault_mask = safety_fault_mask.load();
          snapshot_config_mask = control_policy.config_verified_mask.load();
          snapshot_configuration_epoch = configuration_epoch.load();
          snapshot_verified_epoch = verified_configuration_epoch.load();
          native_home_last_abort_code = latest_feedback.native_home_last_abort_code;
          pos_counts = latest_feedback.pos_counts;
          velocity_actual_counts_per_s = latest_feedback.velocity_actual_counts_per_s;
          following_error_counts = latest_feedback.following_error_counts;
          torque_raw = latest_feedback.torque_raw;
          collision_armed = latest_feedback.collision_armed;
          collision_torque_cycles = latest_feedback.collision_torque_cycles;
          collision_error_cycles = latest_feedback.collision_error_cycles;
          collision_peak_torque = latest_feedback.collision_peak_torque;
          collision_peak_error = latest_feedback.collision_peak_error;
          collision_trip_quantity = latest_feedback.collision_trip_quantity;
          collision_trip_cycles = latest_feedback.collision_trip_cycles;
          collision_trip_count = latest_feedback.collision_trip_count;
          collision_samples = latest_feedback.collision_samples;
          collision_trip_peak_torque = latest_feedback.collision_trip_peak_torque;
          collision_trip_peak_error = latest_feedback.collision_trip_peak_error;
          velocity_actual_valid = latest_feedback.velocity_actual_valid;
          following_error_valid = latest_feedback.following_error_valid;
          di_bits = latest_feedback.di_bits; di_valid = latest_feedback.di_valid;
          external_enable_active = latest_feedback.external_enable_active;
          external_enable_valid = latest_feedback.external_enable_valid;
          max_abs_velocity_actual_counts_per_s = latest_feedback.max_abs_velocity_actual_counts_per_s;
          max_abs_following_error_counts = latest_feedback.max_abs_following_error_counts;
          target_pos_counts = latest_feedback.target_pos_counts;
          mode_display = latest_feedback.mode_display;
          brake_state = latest_feedback.brake_state;
          pdo_observed_time_ns = latest_feedback.pdo_observed_time_ns;
          coordinate_epoch = latest_feedback.coordinate_epoch;
          coordinate_counts = latest_feedback.coordinate_counts;
          absolute_source_counts = latest_feedback.absolute_source_counts;
          coordinate_reason = latest_feedback.coordinate_reason;
          coordinate_valid = latest_feedback.coordinate_valid;
          calibration_valid = latest_feedback.calibration_valid;
          position_state = latest_feedback.position_state;
          coordinate_source_valid = latest_feedback.coordinate_source_valid;
          pdo_fresh_axis_mask = latest_feedback.pdo_fresh_axis_mask;
          statusword = latest_feedback.statusword;
          error_code = latest_feedback.error_code;
          for (uint32_t i = 0; i < opt.num_axes; ++i) alarm_events[i] = drive_alarms[i].event.load();
          manufacturer_error_code = latest_feedback.manufacturer_error_code;
          absolute_feedback = latest_feedback.absolute_feedback;
          slave_al_state = latest_feedback.slave_al_state;
          slave_online = latest_feedback.slave_online;
          slave_operational = latest_feedback.slave_operational;
          const uint64_t s2 = latest_feedback.seq.load(std::memory_order_acquire);
          have_fb = (s1 == s2 && !(s1 & 1u));
        }

        if (!have_fb) std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
#if GRADIENT_HAVE_ECRT
      if (!have_fb) continue;
#endif
      readbacks_requested.store(true, std::memory_order_release);
      for (uint32_t i = 0; have_fb && i < opt.num_axes; ++i) {
        if (collision_trip_count[i] == logged_collision_trips[i]) continue;
        logged_collision_trips[i] = collision_trip_count[i];
        logf("collision_watchdog axis=%u quantity=%s sustained_cycles=%u peak_torque_raw=%u peak_following_error_counts=%llu trip_count=%llu",
            i, CollisionWatchdog::quantity_name(collision_trip_quantity[i]), collision_trip_cycles[i],
            collision_trip_peak_torque[i], static_cast<unsigned long long>(collision_trip_peak_error[i]),
            static_cast<unsigned long long>(collision_trip_count[i]));
      }

      const uint32_t en_mask = axis_enable_mask.load(std::memory_order_relaxed);

      // Only atomic slot states are read: IDs and point storage can be reused
      // concurrently by bulk. Count each slot once to preserve total capacity.
      using SlotState = rosie::rtcore::TrajectoryStore<TrajectoryPointRuntime>::State;
      std::array<uint32_t, static_cast<size_t>(SlotState::Retired) + 1> occupancy{};
      for (size_t i = 0; i < rosie::rtcore::kResidentPlanSlots; ++i)
        ++occupancy[static_cast<size_t>(trajectory_store.view(i).state.load(std::memory_order_acquire))];
      gradient::ipc::v1::StatusMotionStateV1 cursor{};
      const bool have_cursor = motion_snapshot.read(cursor);

      std::ostringstream oss;
      oss << "{";
      oss << "\"time_ns\":" << now_ns << ",";
      oss << "\"buffer_health\":{\"time_ns\":" << now_ns
          << ",\"slots_free\":" << occupancy[static_cast<size_t>(SlotState::Free)] + occupancy[static_cast<size_t>(SlotState::Retired)]
          << ",\"staging_in_progress\":" << (occupancy[static_cast<size_t>(SlotState::Writing)] ? "true" : "false")
          << ",\"ring_drops\":" << rosie::rtcore::diagnostics_dropped.load()
          << ",\"native_slot_occupancy\":{\"free\":" << occupancy[static_cast<size_t>(SlotState::Free)]
          << ",\"writing\":" << occupancy[static_cast<size_t>(SlotState::Writing)]
          << ",\"prepared\":" << occupancy[static_cast<size_t>(SlotState::Prepared)]
          << ",\"ready\":" << occupancy[static_cast<size_t>(SlotState::Ready)]
          << ",\"active\":" << occupancy[static_cast<size_t>(SlotState::Active)]
          << ",\"retired\":" << occupancy[static_cast<size_t>(SlotState::Retired)] << "}},";
      if (have_cursor)
        oss << "\"plan_cursor\":{\"time_ns\":" << cursor.last_update_ns
            << ",\"active_handle\":" << cursor.active_traj_id
            << ",\"sample_index\":" << cursor.current_point_index
            << ",\"native_clock_ns\":" << cursor.last_update_ns << "},";
      oss << "\"cycle_ns\":" << opt.cycle_ns << ",";
      oss << "\"rt_cpu\":" << effective_rt_cpu << ",\"housekeeping_cpus\":\"" << housekeeping.list << "\",";
      oss << "\"affinity_applied\":{\"main\":" << (main_affinity_applied.load() ? "true" : "false")
          << ",\"rt_cycle\":" << (rt_affinity_applied.load() ? "true" : "false")
          << ",\"helper\":" << (helper_affinity_applied.load() ? "true" : "false")
          << ",\"commissioning\":" << (commissioning_affinity_applied.load() ? "true" : "false")
          << ",\"bulk\":" << (bulk_affinity_applied.load() ? "true" : "false")
          << ",\"publisher\":" << (publisher_affinity_applied.load() ? "true" : "false")
          << ",\"capture_drain\":" << (opt.capture_cycles && publisher_affinity_applied.load() ? "true" : "false") << "},";
      oss << "\"num_axes\":" << opt.num_axes << ",";






      oss << "\"dc_enabled\":" << (opt.use_dc ? 1 : 0) << ",";
      oss << "\"dc_reference_sync\":{\"attempts\":" << (have_fb ? dc_reference_sync_attempts : 0)
          << ",\"time_ns\":" << (have_fb ? dc_reference_sync_time_ns : 0)
          << ",\"interval_ns\":" << (have_fb ? dc_reference_sync_interval_ns : 0)
          << ",\"max_interval_ns\":" << (have_fb ? dc_reference_sync_max_interval_ns : 0) << "},";
      oss << "\"output_watchdog_enabled\":" << (opt.disable_output_watchdog ? 0 : 1) << ",";
      oss << "\"split_domains_per_axis\":" << (opt.split_domains_per_axis ? 1 : 0) << ",";
      oss << "\"explicit_pdo_config\":" << (opt.explicit_pdo_config ? 1 : 0) << ",";
      oss << "\"wait_before_safeop_ms\":" << opt.wait_before_safeop_ms << ",";
      oss << "\"preop_to_safeop_timeout_ms\":" << opt.preop_to_safeop_timeout_ms << ",";
      oss << "\"safeop_to_op_timeout_ms\":" << opt.safeop_to_op_timeout_ms << ",";
      oss << "\"startup_passive_ms\":" << opt.startup_passive_ms << ",";
      oss << "\"startup_passive_active\":" << static_cast<unsigned int>(startup_passive_active) << ",";
      oss << "\"startup_skip_domain_queue_ms\":" << opt.startup_skip_domain_queue_ms << ",";
      oss << "\"startup_skip_domain_queue_active\":"
          << static_cast<unsigned int>(startup_skip_domain_queue_active) << ",";
      oss << "\"metrics_startup_readback_enabled\":"
          << (metrics_startup_readback_enabled ? 1 : 0) << ",";
      oss << "\"metrics_native_home_refresh_enabled\":"
          << (metrics_native_home_refresh_enabled ? 1 : 0) << ",";
      oss << "\"metrics_absolute_feedback_poll_enabled\":"
          << (metrics_absolute_feedback_poll_enabled ? 1 : 0) << ",";
      oss << "\"slave_positions\":[";
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (i != 0) {
          oss << ",";
        }
        oss << static_cast<unsigned int>(opt.slave_position[i]);
      }
      oss << "],";
      oss << "\"rt_cycle_counter\":" << cycles << ",";
      oss << "\"rt_hz\":" << hz << ",";
      oss << "\"rt_last_jitter_ns\":" << last_jitter << ",";
      oss << "\"rt_max_abs_jitter_ns\":" << max_abs_jitter << ",";
      oss << "\"rt_max_cycle_work_ns\":" << rt_max_cycle_work_ns.load() << ",";
      oss << "\"telemetry_ring_bytes\":" << telemetry.bytes() << ",";
      oss << "\"telemetry_written\":" << telemetry.written() << ",";
      oss << "\"telemetry_dropped\":" << telemetry.dropped() << ",";
      oss << "\"telemetry_trigger_drops\":" << telemetry_trigger_drops.load() << ",";
      oss << "\"telemetry_dumps_written\":" << telemetry_dumps_written.load() << ",";
      oss << "\"telemetry_dump_failures\":" << telemetry_dump_failures.load() << ",";
      oss << "\"telemetry_writer_reason\":\"" << (telemetry_scheduling_failed.load() ? "scheduler_failed" : "none") << "\",";
      oss << "\"telemetry_dumps_skipped\":" << telemetry_dumps_skipped.load() << ",";
      oss << "\"telemetry_skipped_trigger_drops\":" << telemetry_skipped_trigger_drops.load() << ",";
      oss << "\"capture_records_written\":" << capture_records_written.load() << ",";
      oss << "\"capture_dropped\":" << cycle_capture.dropped() << ",";
      oss << "\"capture_active\":" << (cycle_capture.active() ? 1 : 0) << ",";
      oss << "\"capture_write_failed\":" << (capture_write_failed.load() ? 1 : 0) << ",";
      oss << "\"rt_overrun_count\":" << overruns << ",";
      oss << "\"diagnostics_dropped\":" << rosie::rtcore::diagnostics_dropped.load() << ",";
      oss << "\"diagnostics_produced\":" << rosie::rtcore::diagnostics_produced.load() << ",";
      oss << "\"diagnostics_consumed\":" << rosie::rtcore::diagnostics_consumed.load() << ",";
#if defined(ROSIE_RT_SIMULATION) && ROSIE_RT_SIMULATION
      oss << "\"backend\":\"simulation\",";
#elif GRADIENT_HAVE_ECRT
      oss << "\"backend\":\"ethercat\",";
#else
      oss << "\"backend\":\"ipc_only\",";
#endif
      oss << "\"configuration_sha256\":\"" << opt.configuration_hash << "\",";
      oss << "\"daemon_incarnation\":\"";
      for (auto byte : daemon_incarnation) {
        char encoded[3];
        std::snprintf(encoded, sizeof(encoded), "%02x", byte);
        oss << encoded;
      }
      oss << "\",";
      oss << "\"control_generation\":" << control_policy.generation.load() << ",";
      oss << "\"lease_valid\":" << control_policy.lease_valid_now() << ",";
      oss << "\"config_verified_mask\":" << control_policy.config_verified_mask.load() << ",";
      oss << "\"home_valid_mask\":" << snapshot_home_valid_mask << ",";
      oss << "\"home_epoch\":" << snapshot_home_epoch << ",";
      oss << "\"configuration_epoch\":" << configuration_epoch.load() << ",";
      gradient::ipc::v1::IoStateV1 io{};
      (void)io_snapshot.read(io);
      append_cell_io_json(oss,io);
      oss << "\"dirty_configuration_epoch\":" << dirty_configuration_epoch.load() << ",";
      oss << "\"commissioning_phase\":" << commissioning_phase.load() << ",";
      oss << "\"safety_fault_mask\":" << safety_fault_mask.load() << ",";
      oss << "\"execution_fault_reasons\":" << execution_fault_reasons.load() << ",";
      oss << "\"recovery\":";
      write_recovery_json(oss);
      oss << ',';
      oss << "\"last_bus_failure_operation\":" << last_bus_failure_operation.load() << ",";
      oss << "\"last_bus_failure_code\":" << last_bus_failure_code.load() << ",";
      oss << "\"max_cycle_lateness_ns\":" << opt.max_cycle_lateness_ns << ",";
      oss << "\"armed\":" << (armed.load(std::memory_order_relaxed) ? 1 : 0) << ",";
      oss << "\"axis_enable_mask\":" << en_mask << ",";
      oss << "\"native_home_active_axis_mask\":"
          << native_home_active_axis_mask.load(std::memory_order_relaxed) << ",";
      const uint64_t completion_diag_traj_id =
          trajectory_completion_diag_traj_id.load(std::memory_order_relaxed);
      const uint32_t completion_diag_axis_mask =
          trajectory_completion_diag_axis_mask.load(std::memory_order_relaxed);
      oss << "\"trajectory_completion\":{";
      oss << "\"traj_id\":" << completion_diag_traj_id << ",";
      oss << "\"sample_time_ns\":"
          << trajectory_completion_diag_sample_time_ns.load(std::memory_order_relaxed) << ",";
      oss << "\"axis_mask\":" << completion_diag_axis_mask << ",";
      oss << "\"final_due\":"
          << trajectory_completion_diag_final_due.load(std::memory_order_relaxed) << ",";
      oss << "\"axes\":[";
      bool first_completion_axis = true;
      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        if ((completion_diag_axis_mask & (1u << i)) == 0u) {
          continue;
        }
        if (!first_completion_axis) {
          oss << ",";
        }
        first_completion_axis = false;
        oss << "{";
        oss << "\"axis_index\":" << i << ",";
        oss << "\"target_counts\":"
            << trajectory_completion_diag_target_counts[i].load(std::memory_order_relaxed) << ",";
        oss << "\"feedback_counts\":"
            << trajectory_completion_diag_feedback_counts[i].load(std::memory_order_relaxed) << ",";
        oss << "\"error_counts\":"
            << trajectory_completion_diag_error_counts[i].load(std::memory_order_relaxed);
        oss << "}";
      }
      oss << "]},";
      oss << "\"have_feedback\":" << (have_fb ? 1 : 0) << ",";
      oss << "\"wkc_actual\":" << (have_fb ? wkc_actual : 0) << ",";
      oss << "\"wkc_expected\":" << wkc_expected << ",";
      oss << "\"master_state\":" << master_state << ",";
      oss << "\"responding_slaves\":" << responding_slaves << ",";
      rosie::rtcore::BusTopologySnapshot topology{};
#if GRADIENT_HAVE_ECRT
      topology_snapshot.read(topology);
#endif
      oss << "\"bus\":";
      rosie::rtcore::append_bus_json(oss, opt.bus, topology);
      oss << ',';
      // The publisher owns subsequent description replacements; socket
      // ownership was established before it started. Replace only changed
      // topology; polling alone must not mutate the live owner's description.
      if (topology.generation) {
        const auto encoded = describe_core(opt, axis_profiles, topology);
        if (encoded != published_description) {
          const auto description = std::filesystem::path(opt.socket_path).parent_path() / "description.json";
          const auto temporary = description.string() + ".tmp";
          std::ofstream output(temporary, std::ios::trunc);
          output << encoded; output.close();
          if (output) {
            std::error_code error;
            std::filesystem::rename(temporary, description, error);
            if (error) logf("WARNING: cannot publish bus description: %s", error.message().c_str());
            else published_description = encoded;
          }
        }
      }
      oss << "\"online_slaves\":" << online_slaves << ",";
      oss << "\"operational_slaves\":" << operational_slaves << ",";
      oss << "\"link_up\":" << static_cast<unsigned int>(link_up) << ",";
      oss << "\"master_al_states\":" << static_cast<unsigned int>(master_al_states) << ",";
      oss << "\"domain_wc_state\":" << static_cast<unsigned int>(domain_wc_state) << ",";
      oss << "\"startup_ready\":" << static_cast<unsigned int>(startup_ready) << ",";
      oss << "\"startup_elapsed_ms\":" << startup_elapsed_ms << ",";
      oss << "\"startup_reset_count\":" << startup_reset_count << ",";
      oss << "\"axes\":[";
      auto append_absolute_feedback_json = [&](const AbsoluteFeedbackAxis& axis_feedback, uint32_t axis) {
        const auto& absolute_feedback_cfg = axis_profiles[axis].absolute;
        oss << "\"absolute_feedback\":{";
        bool first = true;
        for (uint32_t field_i = 0; field_i < absolute_feedback_cfg.field_count; ++field_i) {
          const auto& field_cfg = absolute_feedback_cfg.fields[field_i];
          if (!field_cfg.valid || field_cfg.key.empty()) {
            continue;
          }
          if (!first) {
            oss << ",";
          }
          first = false;
          const auto& field = axis_feedback.fields[field_i];
          oss << "\"" << field_cfg.key << "\":{";
          oss << "\"valid\":" << static_cast<unsigned int>(field.valid) << ",";
          oss << "\"value\":" << field.value;
          oss << "}";
        }
        // Phase 1 (2026-04-21) atomic-paired 0x6064 snapshot: the live
        // position counts as they read AT THE MOMENT the SDO uploads
        // above completed. Python's canonical-truth shaft-frame gate
        // reads `paired_pos_counts` instead of the live-now 6064 to
        // avoid the 200 ms poll-skew flicker during motion.
        if (!first) {
          oss << ",";
        }
        oss << "\"paired_pos_counts\":{";
        oss << "\"valid\":" << static_cast<unsigned int>(axis_feedback.paired_valid) << ",";
        oss << "\"value\":" << axis_feedback.paired_pos_counts;
        oss << "},";
        oss << "\"paired_sample_time_ns\":{";
        oss << "\"valid\":" << static_cast<unsigned int>(axis_feedback.paired_valid) << ",";
        oss << "\"value\":" << axis_feedback.paired_sample_time_ns;
        oss << "}";
        oss << "},";
      };
      auto append_startup_drive_config_json = [&](const StartupSdoConfig& descriptor,
                                                  const StartupSdoFeedback& feedback,
                                                  uint32_t axis_index) {
        if (!descriptor.valid) {
          oss << "null";
          return;
        }
        oss << "{";
        oss << "\"setting_key\":\"" << descriptor.key << "\",";
        oss << "\"type\":\"" << startup_sdo_type_name(descriptor.type) << "\",";
        oss << "\"index\":" << static_cast<unsigned int>(descriptor.index) << ",";
        oss << "\"subindex\":" << static_cast<unsigned int>(descriptor.subindex) << ",";
        // Present only for the storage policy: a trial lives in drive RAM and
        // a drive power cycle reverts it; "mode" is the storage-mode object.
        if (descriptor.ram_only) oss << "\"storage\":\"ram_only\",";
        else if (descriptor.storage_mode) oss << "\"storage\":\"mode\",";
        oss << "\"configured\":" << static_cast<unsigned int>(feedback.configured[axis_index]) << ",";
        oss << "\"commanded\":" << feedback.commanded[axis_index] << ",";
        oss << "\"readback_valid\":" << static_cast<unsigned int>(feedback.readback_valid[axis_index]) << ",";
        oss << "\"readback\":" << feedback.readback[axis_index] << ",";
        oss << "\"verified\":" << static_cast<unsigned int>(feedback.verified[axis_index]);
        oss << "}";
      };
      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        if (i != 0) {
          oss << ",";
        }
        oss << "{";
        oss << "\"pdo_profile\":\"" << axis_profiles[i].pdo.label << "\",";
        oss << "\"expected_revision_no\":" << axis_profiles[i].revision_no << ",";
        oss << "\"observed_revision_no\":" << observed_revision[i].load() << ",";
        oss << "\"revision_readback_valid\":" << ((observed_revision_valid_mask.load() >> i) & 1u) << ",";
        oss << "\"vendor_id\":" << axis_profiles[i].vendor_id << ",";
        oss << "\"product_code\":" << axis_profiles[i].product_code << ",";
        oss << "\"slave_position\":" << opt.slave_position[i] << ",";
        oss << "\"pos_counts\":" << pos_counts[i] << ",";
        oss << "\"torque_raw\":" << torque_raw[i] << ",";
        oss << "\"collision_watchdog\":{\"armed\":" << (collision_armed[i] ? "true" : "false")
            << ",\"torque_cycles\":" << collision_torque_cycles[i]
            << ",\"following_error_cycles\":" << collision_error_cycles[i]
            << ",\"peak_torque_raw\":" << collision_peak_torque[i]
            << ",\"peak_following_error_counts\":" << collision_peak_error[i]
            << ",\"last_trip_quantity\":\"" << CollisionWatchdog::quantity_name(collision_trip_quantity[i]) << "\""
            << ",\"trip_sustained_cycles\":" << collision_trip_cycles[i]
            << ",\"samples\":" << collision_samples[i]
            << ",\"last_trip_peak_torque_raw\":" << collision_trip_peak_torque[i]
            << ",\"last_trip_peak_following_error_counts\":" << collision_trip_peak_error[i]
            << ",\"trip_count\":" << collision_trip_count[i] << "},";
        oss << "\"velocity_actual_counts_per_s\":" << velocity_actual_counts_per_s[i] << ",";
        oss << "\"following_error_counts\":" << following_error_counts[i] << ",";
        oss << "\"velocity_actual_valid\":" << (velocity_actual_valid[i] ? "true" : "false") << ",";
        oss << "\"following_error_valid\":" << (following_error_valid[i] ? "true" : "false") << ",";
        oss << "\"di_bits\":" << di_bits[i] << ",\"di_valid\":" << (di_valid[i] ? "true" : "false") << ",";
        oss << "\"external_enable_active\":" << (external_enable_active[i] ? "true" : "false") << ",";
        oss << "\"external_enable_valid\":" << (external_enable_valid[i] ? "true" : "false") << ",";
        oss << "\"max_abs_velocity_actual_counts_per_s\":" << max_abs_velocity_actual_counts_per_s[i] << ",";
        oss << "\"max_abs_following_error_counts\":" << max_abs_following_error_counts[i] << ",";
        // Drive health (docs/decisions/0001): the last second of torque and
        // following error, and the drive profile's monitor objects.
        const auto load = drive_load[i].summary();
        oss << "\"load\":{\"window_ms\":" << load.window_ns / 1000000
            << ",\"torque_samples\":" << load.torque_samples << ",\"torque_rms_raw\":" << load.torque_rms
            << ",\"torque_peak_raw\":" << load.torque_peak
            << ",\"following_error_samples\":" << load.error_samples
            << ",\"following_error_peak_counts\":" << load.error_peak << "},";
        oss << "\"monitor\":[";
        for (size_t m = 0; m < axis_profiles[i].monitor.size(); ++m) {
          const auto& config = axis_profiles[i].monitor[m];
          int32_t raw = 0;
          uint64_t at_ns = 0;
          const bool read = drive_monitors[i].slots[m].snapshot(&raw, &at_ns);
          oss << (m ? "," : "") << "{\"key\":\"" << config.key << "\",\"unit\":\"" << config.unit << "\",\"value\":";
          if (read) oss << rosie::rtcore::monitor_value(raw, config.decimals);
          else oss << "null";
          oss << ",\"age_ms\":";
          if (read) oss << (now_ns > at_ns ? (now_ns - at_ns) / 1000000 : 0);
          else oss << "null";
          oss << "}";
        }
        oss << "],";
        const auto model = coordinate_model(i, native_home_position_offset[i]);
        rosie::rtcore::CoordinateReference turns{};
        turns.valid = coordinate_valid[i] != 0 && coordinate_source_valid[i] != 0;
        turns.continuous_counts = coordinate_counts[i];
        const bool logical_trusted = model.trust((pdo_fresh_axis_mask & (1u << i)) != 0,
            snapshot_verified_epoch == snapshot_configuration_epoch &&
                (snapshot_config_mask & (1u << i)),
            !axis_profiles[i].limits.require_home || (snapshot_home_valid_mask & (1u << i)), turns);
        const auto logical = model.counts_to_logical(pos_counts[i], logical_trusted, turns);
        const auto previous_precision = oss.precision(17);
        oss << "\"logical_position\":" << logical.logical << ",";
        oss << "\"logical_target\":" << model.command_to_logical(target_pos_counts[i]).logical << ",";
        oss.precision(previous_precision);
        oss << "\"logical_valid\":" << (logical.valid ? "true" : "false") << ",";
        // The axis gate uses the native readiness operands and a fixed public
        // priority. Group lease/Arm/commissioning gates remain independently
        // authoritative; ready alone never grants permission to move.
        const auto alarm_aux = drive_alarms[i].snapshot(alarm_events[i], snapshot_configuration_epoch, error_code[i]);
        const char* alarm = axis_profiles[i].alarm_readiness(error_code[i], alarm_aux.alarm_aux_code,
            alarm_aux.alarm_aux_valid, statusword[i],
            (snapshot_fault_mask & (1u << i)) != 0 || manufacturer_error_code[i] != 0);
        const char* readiness = rosie::rtcore::axis_readiness_reason(alarm, mode_display[i] == 8,
            !axis_profiles[i].limits.require_home || (snapshot_home_valid_mask & (1u << i)),
            logical.valid && (!axis_profiles[i].coordinate_evidence.enabled || coordinate_valid[i]),
            (snapshot_enabled_mask & (1u << i)) != 0,
            gradient::ds402::decode_statusword(statusword[i]) == gradient::ds402::State::OperationEnabled,
            brake_state[i] == gradient::ipc::v1::BRAKE_RELEASED);
        oss << "\"readiness\":\"" << readiness << "\",";
        oss << "\"time_ns\":" << now_ns << ",";
        oss << "\"home_valid\":" << ((snapshot_home_valid_mask & (1u << i)) ? "true" : "false") << ",";
        static constexpr const char* position_names[] = {"unverified", "recovering", "trusted", "lost"};
        oss << "\"calibration_valid\":" << (calibration_valid[i] ? "true" : "false") << ",";
        oss << "\"position_state\":\"" << position_names[std::min<unsigned>(position_state[i], 3)] << "\",";
        oss << "\"position_reason\":\"" << rosie::rtcore::coordinate_reason_name(rosie::rtcore::CoordinateReason(coordinate_reason[i])) << "\",";
        rosie::rtcore::SampleConsumption audit{};
        sampled_consumption[i].read(audit);
        oss << "\"audit_status\":\"" << (audit.valid ? "verified" : audit.started_ns ? "unavailable" : "pending") << "\",";
        oss << "\"audit_reason\":\"" << rosie::rtcore::sampled_invalidity_name(audit.reason) << "\",\"audit_last_verified_ns\":" << audit.last_verified_ns << ",";
        oss << "\"serial_no\":" << observed_serial[i].load() << ",";
        oss << "\"serial_readback_valid\":" << ((observed_serial_valid_mask.load() >> i) & 1u) << ",";
        oss << "\"anchor_tolerance_counts\":" << axis_profiles[i].coordinate_evidence.maximum_anchor_error_counts << ",";
        RestoreAnchorEvidence saved{};
        std::string changed_field;
        uint32_t offer = load_restore_anchor(i, saved, &changed_field);
        if (!changed_field.empty() && changed_field != logged_anchor_difference[i])
          logf("anchor_identity_mismatch axis=%u field=%s Home required", i, changed_field.c_str());
        logged_anchor_difference[i] = changed_field;
        const auto& coordinate_identity = axis_profiles[i].coordinate_identity;
        oss << "\"coordinate_identity_sha256\":\"" << (coordinate_identity.empty() ? "" : rosie::rtcore::identity::digest(coordinate_identity)) << "\",";
        // Hex transport is unnecessary here: canonical values are tokens and
        // numeric descriptors, with only LF separators needing JSON escaping.
        std::string coordinate_json;
        for (char c : coordinate_identity) { if (c == '\n') coordinate_json += "\\n"; else coordinate_json += c; }
        oss << "\"coordinate_identity\":\"" << coordinate_json << "\",";
        oss << "\"anchor_identity_changed_field\":\"" << changed_field << "\",";
        if (axis_profiles[i].independent_absolute_origin()) {
          // Independent acquisition is deliberately separate from coordinate/Home
          // validity. It lets the adapter qualify an explicit Restore against its
          // saved record without inventing a command frame before native binding.
          const auto& proof = absolute_publication.values[i];
          const auto maximum_age = axis_profiles[i].coordinate_evidence.maximum_acquisition_age_ns;
          const bool observation_envelope = absolute_publication.available[i] && bus_ready.load() &&
              (pdo_fresh_axis_mask & (1u << i)) && (snapshot_config_mask & (1u << i)) &&
              snapshot_verified_epoch == snapshot_configuration_epoch &&
              configuration_epoch.load() == snapshot_configuration_epoch && !(statusword[i] & 8u) &&
              !manufacturer_error_code[i] && !axis_profiles[i].drive_error_faulted(error_code[i],
                  alarm_aux.alarm_aux_code, alarm_aux.alarm_aux_valid);
          const auto current_command_frame = absolute_frame_epoch[i].load();
          const bool raw_valid = observation_envelope &&
              rosie::rtcore::independent_encoder_observation_current(proof, snapshot_configuration_epoch,
                  now_ns, maximum_age);
          // Diagnostic only. The public adapter may ignore this metrics field;
          // native Home/Restore still require the stricter anchor proof below.
          oss << "\"independent_encoder_observation\":{\"valid\":" << (raw_valid ? "true" : "false")
              << ",\"encoder_counts\":" << proof.encoder_counts
              << ",\"started_ns\":" << proof.started_ns
              << ",\"completed_ns\":" << proof.completed_ns
              << ",\"maximum_age_ns\":" << maximum_age
              << ",\"configuration_epoch\":" << proof.configuration_epoch
              << ",\"command_frame_epoch\":" << proof.home_epoch
              << ",\"current_command_frame_epoch\":" << current_command_frame
              << ",\"acquisition_valid\":" << (proof.valid ? "true" : "false")
              << ",\"acquisition_reason\":" << proof.reason
              << ",\"bracket_completed_ns\":" << proof.bracket_completed_ns
              << ",\"low_completed_ns\":" << proof.low_completed_ns
              << ",\"independent_origin\":" << (proof.independent_origin ? "true" : "false") << "},";
          const bool valid = observation_envelope &&
              rosie::rtcore::independent_acquisition_current(proof, snapshot_configuration_epoch,
                  current_command_frame, now_ns, maximum_age);
          oss << "\"independent_anchor_source\":{\"valid\":" << (valid ? "true" : "false")
              << ",\"encoder_counts\":" << proof.encoder_counts
              << ",\"completed_ns\":" << proof.completed_ns
              << ",\"maximum_age_ns\":" << maximum_age << "},";
          // This is an offer, never a binding. RT repeats the epoch, raw/command
          // agreement and stationary-history checks at actual Restore completion.
          if (!offer && !valid) offer = 3;
          if (!offer && !rosie::rtcore::sampled_anchor_matches(proof, pos_counts[i], saved.counts, saved.turns,
              saved.tolerance_counts)) offer = 4;
        } else {
          if (!offer && (!turns.valid || !(pdo_fresh_axis_mask & (1u << i)))) offer = 3;
          if (!offer && ((!axis_profiles[i].coordinate_evidence.sdo_sampled && saved.counts != saved.turns) ||
              std::abs(int64_t(absolute_source_counts[i]) - int64_t(saved.turns)) > int64_t(saved.tolerance_counts))) offer = 4;
        }
        oss << "\"anchor_reason\":\"" << anchor_reason_text(offer) << "\",";
        oss << "\"restored_anchor_home_epoch\":" << restored_anchor_home_epoch[i].load() << ",";
        oss << "\"brake_state\":" << brake_state[i] << ",";
        rosie::rtcore::BrakeHoldEvidence brake_trip{};
        brake_hold_evidence[i].read(brake_trip);
        const double brake_counts_per_rad = opt.axis[i].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY
            ? opt.axis[i].counts_per_unit : 0.0;
        append_brake_hold_json(oss, brake_trip, axis_profiles[i], i, brake_counts_per_rad);
        rosie::rtcore::TargetLeadEvidence lead{};
        if (target_lead_evidence[i].read(lead) && lead.sequence &&
            (lead.sequence != logged_target_lead[i] || lead.cleared != logged_target_lead_cleared[i])) {
          logged_target_lead[i] = lead.sequence;
          logged_target_lead_cleared[i] = lead.cleared;
          const double deg_per_count = brake_counts_per_rad > 0 ? 180.0 / (3.14159265358979323846 * brake_counts_per_rad) : 0.0;
          if (!lead.cleared) {
            logf("target_lead severity=warning axis=%u sequence=%llu source=%s observed_ns=%llu "
                 "lead_counts=%.0f lead_deg=%.4f limit_counts=%.0f limit_deg=%.4f "
                 "velocity_counts_per_s=%.0f velocity_deg_s=%.3f action=none",
                 i, static_cast<unsigned long long>(lead.sequence), lead.source == 2 ? "halt" : "jog",
                 static_cast<unsigned long long>(lead.observed_ns), lead.lead_counts, lead.lead_counts * deg_per_count,
                 lead.limit_counts, lead.limit_counts * deg_per_count,
                 lead.velocity_counts_per_s, lead.velocity_counts_per_s * deg_per_count);
          } else {
            logf("target_lead_cleared axis=%u sequence=%llu peak_counts=%.0f peak_deg=%.4f duration_ns=%llu",
                 i, static_cast<unsigned long long>(lead.sequence), lead.peak_counts, lead.peak_counts * deg_per_count,
                 static_cast<unsigned long long>(lead.cleared_ns - lead.observed_ns));
          }
        }
        if (brake_trip.warning_sequence && brake_trip.warning_sequence != logged_brake_trips[i]) {
          logged_brake_trips[i] = brake_trip.warning_sequence;
          const auto message = brake_hold_message(i, brake_trip, brake_counts_per_rad);
          logf("brake_hold severity=warning kind=%s axis=%u warning_sequence=%llu message=\"%s\" "
               "cause=%s phase=%s observed_ns=%llu anchor_counts=%d feedback_counts=%d "
               "displacement_counts=%lld tolerance_counts=%u hold_delay_ns=%llu hold_window_active=%u "
               "statusword=0x%04x brake_state=%u released_signal=%s bit=%u valid=%u value=%u "
               "velocity_counts_per_s=%d velocity_valid=%u transition_caps_valid=%u "
               "effective_velocity_cap_counts_per_s=%u effective_displacement_cap_counts=%u command=drive_managed",
               brake_trip.slip ? "slip" : "settle",
               i, static_cast<unsigned long long>(brake_trip.warning_sequence), message.c_str(),
               rosie::rtcore::brake_guard_failure_name(brake_trip.cause),
               axis_profiles[i].brake.transition.enabled() ? rosie::rtcore::brake_guard_phase_name(brake_trip.phase) : "legacy",
               static_cast<unsigned long long>(brake_trip.observed_ns), brake_trip.anchor_counts, brake_trip.feedback_counts,
               static_cast<long long>(brake_trip.displacement_counts), axis_profiles[i].brake.hold_displacement_tolerance_counts,
               static_cast<unsigned long long>(axis_profiles[i].brake.hold_delay_ns), brake_trip.hold_window_active,
               brake_trip.statusword, brake_trip.state,
               axis_profiles[i].brake.feedback_required ? axis_profiles[i].brake_released_semantic.c_str() : "none",
               axis_profiles[i].brake_released_bit, brake_trip.released_valid, brake_trip.released,
               brake_trip.velocity_counts_per_s, brake_trip.velocity_valid, brake_trip.transition_caps_valid,
               brake_trip.effective_velocity_cap_counts_per_s, brake_trip.effective_displacement_cap_counts);
          logf("brake_hold_pattern axis=%u warning_sequence=%llu same_direction_count=%u "
               "same_direction_displacement_counts=%lld torque_off_ns=%llu torque_off_anchor_counts=%d "
               "energized_peak_displacement_counts=%lld energized_peak_velocity_counts_per_s=%d",
               i, static_cast<unsigned long long>(brake_trip.warning_sequence), brake_trip.same_direction_count,
               static_cast<long long>(brake_trip.same_direction_displacement_counts),
               static_cast<unsigned long long>(brake_trip.torque_off_ns), brake_trip.torque_off_anchor_counts,
               static_cast<long long>(brake_trip.energized_peak_displacement_counts),
               brake_trip.energized_peak_velocity_counts_per_s);
        }
        if (brake_trip.settled && brake_trip.warning_sequence != logged_brake_settles[i]) {
          logged_brake_settles[i] = brake_trip.warning_sequence;
          logf("brake_hold_settled axis=%u warning_sequence=%llu settled_displacement_counts=%lld "
               "peak_displacement_counts=%lld peak_velocity_counts_per_s=%d",
               i, static_cast<unsigned long long>(brake_trip.warning_sequence),
               static_cast<long long>(brake_trip.settled_displacement_counts),
               static_cast<long long>(brake_trip.peak_displacement_counts), brake_trip.peak_velocity_counts_per_s);
        }
        rosie::rtcore::DriveSupervisorEvidence supervisor{};
        drive_supervisor_evidence[i].read(supervisor);
        append_drive_supervisor_json(oss, supervisor, i, opt.axis[i].counts_per_unit,
            opt.axis[i].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY, arm_request_sequence.load(std::memory_order_acquire));
        if (supervisor.sequence && supervisor.sequence != logged_supervisor_events[i]) {
          logged_supervisor_events[i] = supervisor.sequence;
          logf("drive_supervisor severity=%s axis=%u event_sequence=%llu kind=%s cause=%s message=\"%s\" "
               "observed_ns=%llu statusword=0x%04x elapsed_ns=%llu bound_ns=%llu displacement_counts=%lld bound_counts=%u",
               supervisor.arm_ended ? "arm_ended" : "warning", i, static_cast<unsigned long long>(supervisor.sequence),
               rosie::rtcore::drive_supervisor_kind_name(supervisor.kind), rosie::rtcore::arm_cancel_cause_name(supervisor.cause),
               rosie::rtcore::drive_supervisor_message(supervisor, i, opt.axis[i].counts_per_unit,
                   opt.axis[i].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY).c_str(),
               static_cast<unsigned long long>(supervisor.observed_ns), supervisor.statusword,
               static_cast<unsigned long long>(supervisor.elapsed_ns), static_cast<unsigned long long>(supervisor.bound_ns),
               static_cast<long long>(supervisor.displacement_counts), supervisor.bound_counts);
        }
        oss << "\"pdo_fresh\":" << ((pdo_fresh_axis_mask >> i) & 1u) << ",";
        oss << "\"pdo_observed_time_ns\":" << pdo_observed_time_ns[i] << ",";
        oss << "\"coordinate_epoch\":" << coordinate_epoch[i] << ",";
        oss << "\"coordinate_valid\":" << static_cast<unsigned int>(coordinate_valid[i]) << ",";
        oss << "\"coordinate_counts\":" << coordinate_counts[i] << ",";
        oss << "\"absolute_source_counts\":" << absolute_source_counts[i] << ",";
        oss << "\"coordinate_reason\":" << (offer == 2 ? uint32_t(rosie::rtcore::CoordinateReason::IdentityChanged) : coordinate_reason[i]) << ",";
        oss << "\"coordinate_source_valid\":" << static_cast<unsigned int>(coordinate_source_valid[i]) << ",";
        if (axis_profiles[i].coordinate_evidence.sdo_sampled) {
          rosie::rtcore::SampleConsumption sample{};
          sampled_consumption[i].read(sample);
          const double samples_per_second = now_ns > sample.first_started_ns && sample.first_started_ns
              ? double(sample.samples) * 1000000000.0 / double(now_ns - sample.first_started_ns) : 0;
          oss << "\"sampled_coordinate\":{\"started_ns\":" << sample.started_ns << ",\"completed_ns\":" << sample.completed_ns
              << ",\"mode\":\"multi_turn\""
              << ",\"startup_attempts\":" << sample.startup_attempts << ",\"startup_verified\":" << sample.startup_verified
              << ",\"startup_reason\":" << sample.startup_reason
              << ",\"acquisition_duration_ns\":" << sample.acquisition_duration_ns << ",\"age_ns\":" << sample.age_ns
              << ",\"low_upload_ns\":" << sample.low_upload_ns << ",\"high_upload_ns\":" << sample.high_upload_ns
              << ",\"bracket_upload_ns\":" << sample.bracket_upload_ns
              << ",\"start_gap_ns\":" << sample.start_gap_ns << ",\"maximum_start_gap_ns\":" << sample.maximum_start_gap_ns
              << ",\"coherence_error_counts\":" << sample.coherence_error_counts << ",\"reason\":" << sample.reason
              << ",\"last_invalidity_reason\":" << sample.last_invalidity_reason << ",\"valid\":" << sample.valid
              << ",\"samples\":" << sample.samples << ",\"samples_per_second\":" << samples_per_second << "},";
        }
        oss << "\"statusword\":" << statusword[i] << ",";
        oss << "\"error_code\":" << error_code[i] << ",";
        append_drive_alarm_json(oss, drive_alarm_name(i, error_code[i], alarm_aux.alarm_aux_code,
            alarm_aux.alarm_aux_valid), alarm_aux);
        {
          // Whether Reset would send this axis's drive its fault reset now,
          // by Reset's own admission table (Reset re-checks every term).
          rosie::rtcore::DriveAlarmResetObservation reset_in{};
          reset_in.statusword = statusword[i];
          reset_in.error_code = error_code[i];
          reset_in.aux_code = alarm_aux.alarm_aux_code;
          reset_in.aux_valid = alarm_aux.alarm_aux_valid;
          reset_in.fresh = (pdo_fresh_axis_mask & (1u << i)) && pdo_observed_time_ns[i];
          reset_in.inhibited = !armed.load() && !snapshot_enabled_mask && !service_mode_axis_mask.load() &&
              !native_home_active_axis_mask.load() && commissioning_phase.load() == 0 && motion_done.load();
          reset_in.now_ns = now_ns;
          reset_in.last_attempt_ns = drive_alarm_reset_last_ns[i].load(std::memory_order_relaxed);
          const auto& reset_config = axis_profiles[i].drive_alarm_reset;
          auto reset_state = rosie::rtcore::classify_drive_alarm_reset(reset_config, reset_in);
          if (drive_alarm_reset_active_mask.load() & (1u << i)) reset_state = rosie::rtcore::DriveAlarmReset::InProgress;
          oss << "\"drive_alarm_reset\":{\"possible\":"
              << (reset_state == rosie::rtcore::DriveAlarmReset::Possible ? "true" : "false")
              << ",\"reason\":\"" << rosie::rtcore::drive_alarm_reset_name(reset_state) << "\",\"retry_after_ns\":"
              << (reset_state == rosie::rtcore::DriveAlarmReset::RateLimited
                      ? rosie::rtcore::drive_alarm_reset_retry_after_ns(reset_config, reset_in.last_attempt_ns, now_ns)
                      : 0) << "},";
        }
        // manufacturer_error_code is the separate cyclic manufacturer_err PDO;
        // the admitted 0x203F SDO receipt is exposed only as drive_alarm_aux_code.
        oss << "\"manufacturer_error_code\":" << manufacturer_error_code[i] << ",";
        oss << "\"startup_drive_config\":";
        size_t first_descriptor = startup_sdos.size();
        for (size_t d = 0; d < startup_sdos.size(); ++d) {
          if (startup_sdos[d].axis_mask & (1u << i)) { first_descriptor = d; break; }
        }
        if (first_descriptor < startup_sdos.size()) {
          append_startup_drive_config_json(startup_sdos[first_descriptor], startup_drive_config_feedback[first_descriptor], i);
        } else { oss << "null"; }
        oss << ",";
        oss << "\"startup_drive_configs\":[";
        bool first_sdo = true;
        for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
          if (!(startup_sdos[descriptor_i].axis_mask & (1u << i))) continue;
          if (!first_sdo) oss << ",";
          first_sdo = false;
          append_startup_drive_config_json(startup_sdos[descriptor_i], startup_drive_config_feedback[descriptor_i], i);
        }
        oss << "],";
        if (anchor_delta_publication.available[i] && anchor_delta_publication.values[i].anchor_version) {
          const auto& b = anchor_delta_publication.values[i];
          const bool valid = rosie::rtcore::anchor_delta_restore_publication_valid(b,
              reference_configuration, coordinate_epoch[i], reference_drive_frame[i], now_ns,
              coordinate_valid[i] && coordinate_source_valid[i] && (snapshot_home_valid_mask & (1u << i)));
          oss << "\"anchor_delta_restore\":{\"method\":\"independent_absolute_delta_v1\",\"anchor_version\":3"
              << ",\"valid\":" << (valid ? "true" : "false")
              << ",\"saved_command_counts\":" << b.saved_command_counts
              << ",\"saved_encoder_counts\":" << b.saved_encoder_counts
              << ",\"saved_logical_offset_counts\":" << b.saved_logical_offset_counts
              << ",\"saved_drive_home_offset_raw\":" << b.saved_drive_home_offset_raw
              << ",\"saved_home_epoch\":" << b.saved_home_epoch
              << ",\"bound_restore_command_sequence\":" << b.bound_restore_command_sequence
              << ",\"bound_configuration_epoch\":" << b.bound_configuration_epoch
              << ",\"bound_home_epoch\":" << b.bound_home_epoch
              << ",\"bound_axis_coordinate_epoch\":" << b.bound_axis_coordinate_epoch
              << ",\"bound_drive_frame_epoch\":" << b.bound_drive_frame_epoch
              << ",\"bound_completed_ns\":" << b.bound_completed_ns
              << ",\"bound_pdo_receipt\":" << b.bound_pdo_receipt
              << ",\"bound_delta_command_counts\":" << b.bound_delta_command_counts
              << ",\"bound_delta_encoder_counts\":" << b.bound_delta_encoder_counts
              << "},";
        }
        oss << "\"native_home_state\":" << static_cast<unsigned int>(native_home_state[i]) << ",";
        oss << "\"native_home_position_offset\":" << native_home_position_offset[i] << ",";
        oss << "\"native_home_last_abort_code\":" << native_home_last_abort_code[i] << ",";
        NativeHomeFailure failure{};
        native_home_failures[i].read(failure);
        oss << "\"native_home_failure\":";
        append_native_home_failure_json(oss, failure);
        oss << ",";
        append_absolute_feedback_json(absolute_feedback[i], i);
        oss << "\"slave_online\":" << static_cast<unsigned int>(slave_online[i]) << ",";
        oss << "\"slave_operational\":" << static_cast<unsigned int>(slave_operational[i]) << ",";
        oss << "\"slave_al_state\":" << static_cast<unsigned int>(slave_al_state[i]);
        oss << "}";
      }
      oss << "]";
      oss << "}\n";

      try {
        const std::filesystem::path tmp = metrics_path.string() + ".tmp";
        std::ofstream f(tmp, std::ios::out | std::ios::trunc);
        f << oss.str();
        f.close();
        std::filesystem::rename(tmp, metrics_path);
        // Keep readable by the controller user/group.
        if (ipc_gid != static_cast<gid_t>(-1)) {
          const int ownership_result = ::chown(metrics_path.c_str(), static_cast<uid_t>(-1), ipc_gid);
          (void)ownership_result;
        }
        (void)::chmod(metrics_path.c_str(), 0644);
      } catch (const std::exception& e) {
        // Don't spam logs if the filesystem is temporarily unhappy.
        if (now_ns - last_warn_ns > 5000000000ULL) { // 5s
          last_warn_ns = now_ns;
          logf("WARNING: failed to write metrics file %s: %s",
               metrics_path.c_str(), e.what());
        }
      }
    }
    drain_cycle_capture(capture_file.get());
    drain_telemetry();
    if (opt.telemetry_dump_on_exit) {
      rosie::rtcore::TelemetryDumpRequest request{};
      request.count = 1; request.end = telemetry.written();
      request.triggers[0].end = request.end; request.triggers[0].reason = 8;
      queue_telemetry_dump(request);
    }
    rosie::rtcore::drain_diagnostics();
  }
