#pragma once

// Configuration and optional inventory description; formatting stays outside RT.
std::string describe_core(const Options& options,
    const std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES>& profiles,
    const rosie::rtcore::BusTopologySnapshot& topology = {}) {
  std::ostringstream out;
  out.precision(17);
  out << "{\"schema\":\"rosie.rt-core.description.v1\",\"protocol_major\":1,\"protocol_minor\":"
      << rosie::protocol::version_minor << ",\"configuration_sha256\":\"" << options.configuration_hash
      << "\",\"machine_sha256\":\"" << options.machine_sha256
      << "\",\"deployment_sha256\":\"" << options.deployment_sha256
      << "\",\"max_trajectory_points\":" << rosie::protocol::max_trajectory_points
      << ",\"resident_plans\":3,\"cycle_ns\":" << options.cycle_ns
      << ",\"max_grant_lease_ns\":" << options.control_max_grant_lease_ns
      << ",\"max_jog_input_age_ns\":" << options.control_max_jog_input_age_ns
      << ",\"interpolation\":\"" << rosie::rtcore::interpolation::capabilities(false).interpolation
      << "\",\"stop\":\"output-inhibition\",\"backend\":\""
#if defined(ROSIE_RT_SIMULATION)
      << "simulation"
#elif GRADIENT_HAVE_ECRT
      << "ethercat"
#else
      << "ipc_only"
#endif
      << "\",\"axes\":[";
  for (uint32_t i = 0; i < options.num_axes; ++i) {
    const auto& axis = options.axis[i];
    const auto& profile = profiles[i];
    const auto& limits = profile.limits;
    const auto policy = rosie::rtcore::interpolation::capabilities(profile.max_acceleration.has_value());
    auto mapped = [&](const char* field) {
      return std::any_of(profile.pdo.rx_entries.begin(), profile.pdo.rx_entries.end(),
          [&](const auto& entry) { return entry.semantic == field; });
    };
    if (i) out << ',';
    out << "{\"index\":" << i << ",\"id\":\"" << profile.name << "\",\"position_unit\":\""
        << (axis.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY ? "rad" : "m")
        << "\",\"counts_per_unit\":" << axis.counts_per_unit << ",\"sign\":" << axis.sign
        << ",\"vendor_id\":" << profile.vendor_id << ",\"product_code\":" << profile.product_code
        << ",\"revision_no\":" << profile.revision_no << ",\"slave_position\":" << options.slave_position[i]
        << ",\"velocity_command_mapped\":" << (mapped("target_vel") ? "true" : "false")
        << ",\"native_home\":" << (profile.home.valid ? "true" : "false")
        << ",\"require_home\":" << (limits.require_home ? "true" : "false")
        << ",\"min_position\":" << limits.min_q << ",\"max_position\":" << limits.max_q
        << ",\"max_velocity\":" << std::min(limits.max_qd, double(axis.max_profile_vel_counts_per_s) / axis.counts_per_unit)
        << ",\"jog_acceleration\":" << limits.jog_acceleration
        << ",\"interpolation\":\"" << policy.interpolation
        << "\",\"feedforward\":\"" << policy.feedforward
        << "\",\"checks\":{\"position\":\"" << policy.checks.position
        << "\",\"velocity\":\"" << policy.checks.velocity
        << "\",\"acceleration\":\"" << policy.checks.acceleration
        << "\",\"jerk\":\"" << policy.checks.jerk
        << "\"},\"endpoint\":\"" << policy.endpoint << "\"";
    if (profile.reference_verified_delta_enabled)
      out << ",\"reference_verified_delta_restore\":{\"method\":\"independent_absolute_delta_v1\"}";
    if (profile.max_acceleration) out << ",\"max_acceleration\":" << *profile.max_acceleration;
    if (!profile.brake_override_reason_hex.empty()) {
      // The startup parser admits only lowercase byte pairs. Escape ASCII as
      // JSON code points and preserve UTF-8 bytes, preventing reason injection.
      out << ",\"brake_override_reason\":\"";
      const auto& hex = profile.brake_override_reason_hex;
      const auto nibble = [](char c) { return c <= '9' ? c - '0' : c - 'a' + 10; };
      for (size_t byte = 0; byte < hex.size(); byte += 2) {
        const auto value = (nibble(hex[byte]) << 4) | nibble(hex[byte + 1]);
        if (value < 128) out << "\\u00" << hex[byte] << hex[byte + 1];
        else out << static_cast<char>(value);
      }
      out << '\"';
    }
    out
        << ",\"max_target_lead\":" << limits.max_target_lead
        << ",\"completion_tolerance\":" << std::max(limits.completion_tolerance, 1.0 / axis.counts_per_unit)
        << ",\"following_error\":" << limits.following_error
        << ",\"following_error_timeout_ns\":" << limits.following_error_timeout_ns
        << ",\"completion_timeout_ns\":" << limits.completion_timeout_ns
        << ",\"feedback_fields\":[";
    for (size_t field = 0; field < profile.pdo.tx_entries.size(); ++field) {
      if (field) out << ',';
      out << '\"' << profile.pdo.tx_entries[field].semantic << '\"';
    }
    out << "]}";
  }
  out << "],\"bus\":";
  rosie::rtcore::append_bus_json(out, options.bus, topology);
  if (options.cell_io.enabled()) {
    out << ",\"io\":";
    rosie::rtcore::append_cell_io_config_json(out, options.cell_io);
  }
  // Path jog needs the machine jog_jerk (one value for every axis): without
  // it the path generator has no jerk bound, so the capability is omitted and
  // clients refuse path holds instead of falling back to velocity jog.
  if (options.num_axes && profiles[0].jog.jerk > 0)
    out << ",\"path_jog\":{\"max_samples\":" << gradient::ipc::v1::PATH_JOG_MAX_SAMPLES
        << ",\"max_chunk\":" << gradient::ipc::v1::PATH_JOG_MAX_CHUNK
        << ",\"max_sample_step_s\":" << double(gradient::ipc::v1::PATH_JOG_MAX_SAMPLE_STEP_NS) / 1e9 << "}";
  out << "}\n";
  return out.str();
}
