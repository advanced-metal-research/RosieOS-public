// Private implementation header included once by robot_v4_cartesian_daemon.cpp.
double max_abs(const std::vector<double>& values) {
  double max_value = 0.0;
  for (double value : values) {
    const double abs_value = value < 0.0 ? -value : value;
    if (abs_value > max_value) {
      max_value = abs_value;
    }
  }
  return max_value;
}

double shortest_angular_distance_rad(double from, double to) {
  double diff = std::fmod(to - from + kPi, 2.0 * kPi);
  if (diff < 0.0) {
    diff += 2.0 * kPi;
  }
  return diff - kPi;
}

bool cartesian_joint_span_guard_allows(const double start_rad[6],
                                       const double candidate_rad[6],
                                       double* max_span_rad,
                                       int* limiting_joint = nullptr) {
  double max_span = 0.0;
  int max_axis = -1;
  for (int axis = 0; axis < 6; ++axis) {
    const double span = std::abs(shortest_angular_distance_rad(start_rad[axis], candidate_rad[axis]));
    if (span > max_span) {
      max_span = span;
      max_axis = axis;
    }
  }
  if (max_span_rad != nullptr) {
    *max_span_rad = max_span;
  }
  if (limiting_joint != nullptr) {
    *limiting_joint = max_axis;
  }
  return max_span <= MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD;
}

void reset_cartesian_joint_span_guard(CartesianJointSpanGuard* guard) {
  guard->initialized = false;
  guard->max_span_rad = 0.0;
  guard->latest_candidate_span_rad = 0.0;
  guard->latest_span_margin_rad = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD;
  guard->latest_limiting_joint = -1;
  guard->latest_reason_code = 4;
  guard->latest_action = "reset";
  guard->latest_reason = "non_cartesian_or_stale_reset";
}

void initialize_cartesian_joint_span_guard(CartesianJointSpanGuard* guard,
                                           const CartesianCommand& command) {
  guard->initialized = true;
  guard->max_span_rad = 0.0;
  for (int axis = 0; axis < 6; ++axis) {
    guard->start_rad[axis] = command.joint_position_rad[axis];
    guard->predicted_rad[axis] = command.joint_position_rad[axis];
  }
}

bool advance_cartesian_joint_span_guard(CartesianJointSpanGuard* guard,
                                        const CartesianCommand& command,
                                        const ResolvedJointVelocity& response,
                                        double dt_seconds) {
  if (!guard->initialized) {
    initialize_cartesian_joint_span_guard(guard, command);
  }
  double candidate[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    candidate[axis] = guard->predicted_rad[axis] + response.qd[axis] * dt_seconds;
  }
  double max_span = 0.0;
  int limiting_joint = -1;
  if (!cartesian_joint_span_guard_allows(guard->start_rad, candidate, &max_span, &limiting_joint)) {
    guard->max_span_rad = std::max(guard->max_span_rad, max_span);
    guard->latest_candidate_span_rad = max_span;
    guard->latest_span_margin_rad = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD - max_span;
    guard->latest_limiting_joint = limiting_joint;
    guard->latest_reason_code = 2;
    guard->latest_action = "telemetry_only";
    guard->latest_reason = "predicted_joint_span_exceeded_telemetry_only";
    for (int axis = 0; axis < 6; ++axis) {
      guard->predicted_rad[axis] = candidate[axis];
    }
    for (int axis = 0; axis < 6; ++axis) {
      guard->start_rad[axis] = guard->predicted_rad[axis];
    }
    return true;
  }
  for (int axis = 0; axis < 6; ++axis) {
    guard->predicted_rad[axis] = candidate[axis];
  }
  guard->max_span_rad = std::max(guard->max_span_rad, max_span);
  guard->latest_candidate_span_rad = max_span;
  guard->latest_span_margin_rad = MAX_CARTESIAN_SESSION_JOINT_SPAN_RAD - max_span;
  guard->latest_limiting_joint = limiting_joint;
  guard->latest_reason_code = 1;
  guard->latest_action = "allow";
  guard->latest_reason = "within_predicted_joint_span";
  if (max_span >= CARTESIAN_SPAN_GUARD_RECENTER_RAD) {
    for (int axis = 0; axis < 6; ++axis) {
      guard->start_rad[axis] = guard->predicted_rad[axis];
    }
    guard->latest_reason_code = 3;
    guard->latest_action = "recenter";
    guard->latest_reason = "rolling_recenter_threshold_reached";
  }
  return true;
}

ResolvedJointVelocity resolve_cartesian_velocity(const CartesianCommand& command,
                             SolverCache* cache = nullptr,
                             RuntimeStatus* safety_status = nullptr) {
  const std::vector<double> qd = damped_least_squares_joint_velocity(command, cache);
  double requested_qd[6]{};
  double joint_limits[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    requested_qd[axis] = qd[static_cast<std::size_t>(axis)];
    joint_limits[axis] = MAX_JOINT_JOG_RAD_S;
  }
  int joint_limiting_index = -1;
  const double joint_limit_scale = robot_v4_common_joint_limit_scale(
      requested_qd, joint_limits, 6, &joint_limiting_index);
  const RobotV4SingularityMetrics singularity =
      robot_v4_singularity_metrics(jacobian_for_command(command, cache));
  const double singularity_scale =
      robot_v4_singularity_velocity_scale(singularity, 0.05, 0.02);
  // One scale is applied to the entire joint vector. Independent per-axis
  // clamping bends a Cartesian jog and is therefore intentionally forbidden.
  const double applied_scale = std::min(joint_limit_scale, singularity_scale);
  if (safety_status != nullptr) {
    safety_status->latest_joint_limit_scale = joint_limit_scale;
    safety_status->latest_singularity_scale = singularity_scale;
    safety_status->latest_applied_velocity_scale = applied_scale;
    safety_status->latest_singularity_score = singularity.score;
    safety_status->latest_singularity_condition_number =
        std::isfinite(singularity.condition_number) ? singularity.condition_number : 0.0;
    safety_status->latest_singularity_limiting_joint =
        singularity_scale < joint_limit_scale
            ? singularity.singular_direction_joint_index
            : joint_limiting_index;
    safety_status->latest_singularity_class =
        !singularity.valid ? "unavailable"
                           : (singularity.score <= 0.02
                                  ? "hard_stop"
                                  : (singularity.score < 0.05 ? "warning" : "clear"));
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      safety_status->latest_requested_qd_rad_s[axis] =
          axis < 6 ? requested_qd[axis] : 0.0;
      safety_status->latest_applied_qd_rad_s[axis] =
          axis < 6 ? requested_qd[axis] * applied_scale : 0.0;
    }
  }
  ResolvedJointVelocity out{};
  for (uint32_t axis = 0; axis < 6; ++axis) {
    out.qd[axis] = qd[axis] * applied_scale;
  }
  return out;
}

double max_abs_qd(const ResolvedJointVelocity& cmd, uint32_t axis_count) {
  double max_value = 0.0;
  for (uint32_t axis = 0; axis < axis_count && axis < GRADIENT_MAX_AXES; ++axis) {
    const double value = cmd.qd[axis];
    const double abs_value = value < 0.0 ? -value : value;
    if (abs_value > max_value) {
      max_value = abs_value;
    }
  }
  return max_value;
}

bool joystick_intent_is_neutral_hold(const SteamDeckJoystickIntent& intent) {
  if (intent.frame != JOYSTICK_INTENT_FRAME_TOOL &&
      intent.frame != JOYSTICK_INTENT_FRAME_JOINT) {
    return false;
  }
  if (!intent.deadman) {
    return false;
  }
  double max_abs_axis = 0.0;
  for (int axis = 0; axis < 6; ++axis) {
    const double normalized_axis = clamp_value(intent.axes[axis], -1.0, 1.0);
    max_abs_axis = std::max(max_abs_axis, std::abs(normalized_axis));
  }
  return max_abs_axis <= 0.02;
}

std::string double_array_json(const double* values, int count) {
  std::ostringstream out;
  out << "[";
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      out << ",";
    }
    out << std::fixed << std::setprecision(6) << values[i];
  }
  out << "]";
  return out.str();
}

std::string render_cartesian_response(const CartesianCommand& command, const std::string& self_test,
                                      const std::string& transport, const std::string& input_source,
                                      const std::string& output_sink) {
  const std::vector<double> requested_qd = damped_least_squares_joint_velocity(command);
  double requested[6]{};
  double limits[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    requested[axis] = requested_qd[static_cast<std::size_t>(axis)];
    limits[axis] = MAX_JOINT_JOG_RAD_S;
  }
  int joint_limiting_index = -1;
  const double joint_limit_scale = robot_v4_common_joint_limit_scale(
      requested, limits, 6, &joint_limiting_index);
  const RobotV4SingularityMetrics singularity =
      robot_v4_singularity_metrics(jacobian_for_command(command, nullptr));
  const double singularity_scale =
      robot_v4_singularity_velocity_scale(singularity, 0.05, 0.02);
  const double applied_scale = std::min(joint_limit_scale, singularity_scale);
  std::vector<double> applied_qd = requested_qd;
  for (double& velocity : applied_qd) {
    velocity *= applied_scale;
  }
  const uint64_t timeout_ns = static_cast<uint64_t>(command.timeout_ms) * 1000000ULL;

  std::ostringstream report;
  report << "{"
         << "\"name\":\"robot-v4-cartesiand\","
         << "\"self_test\":\"" << self_test << "\","
         << "\"transport\":\"" << transport << "\","
         << "\"solver\":\"urdf_chain_damped_least_squares\","
         << "\"urdf_joint_count\":6,"
         << "\"input_source\":\"" << input_source << "\","
         << "\"input_control_space\":\"cartesian_tcp_velocity\","
         << "\"steamdeck_joint_data\":false,"
         << "\"output_sink\":\"" << output_sink << "\","
         << "\"joint_conversion\":\"jetson_urdf_chain_damped_least_squares\","
         << "\"rtcore_control_space\":\"joint_velocity_lease\","
         << "\"seq\":" << command.seq << ","
         << "\"rtcore_msg_type\":\"0x0130\","
         << "\"axis_mask\":\"0x3f\","
         << "\"timeout_ns\":" << timeout_ns << ","
         << "\"motion_safety_evidence_schema\":\"robot.v4.motion-safety-evidence.v1\","
         << "\"requested_joint_velocity_rad_s\":" << json_array(requested_qd) << ","
         << "\"joint_velocity_rad_s\":" << json_array(applied_qd) << ","
         << "\"joint_velocity_limits_rad_s\":" << double_array_json_fixed(limits, 6) << ","
         << "\"joint_limit_scale\":" << std::fixed << std::setprecision(9)
         << joint_limit_scale << ","
         << "\"singularity_scale\":" << std::fixed << std::setprecision(9)
         << singularity_scale << ","
         << "\"applied_velocity_scale\":" << std::fixed << std::setprecision(9)
         << applied_scale << ","
         << "\"singularity_score\":" << std::fixed << std::setprecision(9)
         << singularity.score << ","
         << "\"singularity_condition_number\":" << std::fixed << std::setprecision(6)
         << (std::isfinite(singularity.condition_number) ? singularity.condition_number : 0.0)
         << ",\"singularity_class\":\""
         << (!singularity.valid ? "unavailable"
                                : (singularity.score <= 0.02
                                       ? "hard_stop"
                                       : (singularity.score < 0.05 ? "warning" : "clear")))
         << "\",\"limiting_joint\":\""
         << (singularity_scale < joint_limit_scale &&
                     singularity.singular_direction_joint_index >= 0
                 ? spreadsheet_planner_joint_name(static_cast<std::size_t>(
                       singularity.singular_direction_joint_index))
                 : (joint_limiting_index >= 0
                        ? spreadsheet_planner_joint_name(
                              static_cast<std::size_t>(joint_limiting_index))
                        : "none"))
         << "\","
         << "\"max_abs_qd_rad_s\":" << std::fixed << std::setprecision(6)
         << max_abs(applied_qd) << ","
         << "\"accepted\":" << (applied_scale > 0.0 ? "true" : "false") << ","
         << "\"hardware_access\":false"
         << "}\n";
  return report.str();
}

LatencyStats summarize_latency_us(const std::vector<double>& samples) {
  LatencyStats stats;
  stats.samples = static_cast<int>(samples.size());
  if (samples.empty()) {
    return stats;
  }

  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  double total = 0.0;
  for (double sample : samples) {
    total += sample;
  }
  const size_t p95_index = std::min(sorted.size() - 1,
                                    static_cast<size_t>(std::ceil(sorted.size() * 0.95) - 1.0));
  stats.min_us = sorted.front();
  stats.mean_us = total / static_cast<double>(samples.size());
  stats.p95_us = sorted[p95_index];
  stats.max_us = sorted.back();
  stats.budget_pass = stats.max_us <= END_TO_END_LATENCY_BUDGET_US;
  return stats;
}

std::string latency_stats_json(const LatencyStats& stats) {
  std::ostringstream out;
  out << "\"samples\":" << stats.samples << ","
      << "\"min_us\":" << std::fixed << std::setprecision(3) << stats.min_us << ","
      << "\"mean_us\":" << std::fixed << std::setprecision(3) << stats.mean_us << ","
      << "\"p95_us\":" << std::fixed << std::setprecision(3) << stats.p95_us << ","
      << "\"max_us\":" << std::fixed << std::setprecision(3) << stats.max_us;
  return out.str();
}

std::string dominant_latency_source(const LatencyStats& wait,
                                    const LatencyStats& processing,
                                    const LatencyStats& send_call) {
  if (wait.mean_us >= processing.mean_us && wait.mean_us >= send_call.mean_us) {
    return "jetson_input_wait_before_output_tick";
  }
  if (processing.mean_us >= wait.mean_us && processing.mean_us >= send_call.mean_us) {
    return "jetson_solver_and_packet_processing";
  }
  return "jetson_udp_send_call";
}

std::string render_latency_source_report(const Options& options,
                                         const LatencySourceSamples& samples) {
  const LatencyStats receive_to_output = summarize_latency_us(samples.receive_to_output_us);
  const LatencyStats input_wait = summarize_latency_us(samples.input_wait_us);
  const LatencyStats processing = summarize_latency_us(samples.processing_us);
  const LatencyStats send_call = summarize_latency_us(samples.send_call_us);
  const std::string dominant = dominant_latency_source(input_wait, processing, send_call);
  const int period_ms = static_cast<int>(std::round(1000.0 / static_cast<double>(std::max(1, options.control_frequency_hz))));
  const bool complete = receive_to_output.samples == input_wait.samples &&
                        receive_to_output.samples == processing.samples &&
                        receive_to_output.samples == send_call.samples &&
                        receive_to_output.samples > 0;

  std::ostringstream report;
  report << "{"
         << "\"name\":\"robot-v4-cartesiand\","
         << "\"self_test\":\"latency-source-breakdown\","
         << "\"latency_source_analysis\":\"jetson_receive_to_rtcore_send\","
         << "\"diagnostic_echo_packet\":\"robot_v4_latency_echo_v1\","
         << "\"diagnostic_echo_source\":" << (options.diagnostic_echo_source ? "true" : "false") << ","
         << "\"latency_likely_source\":\"" << dominant << "\","
         << "\"dominant_latency_source\":\"" << dominant << "\","
         << "\"control_frequency_hz\":" << options.control_frequency_hz << ","
         << "\"period_ms\":" << period_ms << ","
         << "\"server_latency_samples\":" << receive_to_output.samples << ","
         << "\"latency_analysis_samples\":" << receive_to_output.samples << ","
         << "\"server_receive_to_output_min_us\":" << std::fixed << std::setprecision(3) << receive_to_output.min_us << ","
         << "\"server_receive_to_output_mean_us\":" << std::fixed << std::setprecision(3) << receive_to_output.mean_us << ","
         << "\"server_receive_to_output_p95_us\":" << std::fixed << std::setprecision(3) << receive_to_output.p95_us << ","
         << "\"server_receive_to_output_max_us\":" << std::fixed << std::setprecision(3) << receive_to_output.max_us << ","
         << "\"server_input_wait_min_us\":" << std::fixed << std::setprecision(3) << input_wait.min_us << ","
         << "\"server_input_wait_mean_us\":" << std::fixed << std::setprecision(3) << input_wait.mean_us << ","
         << "\"server_input_wait_p95_us\":" << std::fixed << std::setprecision(3) << input_wait.p95_us << ","
         << "\"server_input_wait_max_us\":" << std::fixed << std::setprecision(3) << input_wait.max_us << ","
         << "\"server_processing_min_us\":" << std::fixed << std::setprecision(3) << processing.min_us << ","
         << "\"server_processing_mean_us\":" << std::fixed << std::setprecision(3) << processing.mean_us << ","
         << "\"server_processing_p95_us\":" << std::fixed << std::setprecision(3) << processing.p95_us << ","
         << "\"server_processing_max_us\":" << std::fixed << std::setprecision(3) << processing.max_us << ","
         << "\"server_send_call_min_us\":" << std::fixed << std::setprecision(3) << send_call.min_us << ","
         << "\"server_send_call_mean_us\":" << std::fixed << std::setprecision(3) << send_call.mean_us << ","
         << "\"server_send_call_p95_us\":" << std::fixed << std::setprecision(3) << send_call.p95_us << ","
         << "\"server_send_call_max_us\":" << std::fixed << std::setprecision(3) << send_call.max_us << ","
         << "\"solver_cache_enabled\":" << (samples.solver_cache_enabled ? "true" : "false") << ","
         << "\"solver_cache_hits\":" << samples.solver_cache_hits << ","
         << "\"solver_cache_misses\":" << samples.solver_cache_misses << ","
         << "\"accepted_input_count\":" << samples.accepted_input_count << ","
         << "\"immediate_output_count\":" << samples.immediate_output_count << ","
         << "\"scheduled_output_count\":" << samples.scheduled_output_count << ","
         << "\"coalesced_input_count\":" << samples.coalesced_input_count << ","
         << "\"latency_steps\":["
         << "{\"order\":1,\"name\":\"01 jetson udp receive to output tick\"," << latency_stats_json(input_wait) << "},"
         << "{\"order\":2,\"name\":\"02 jetson solver and packet build\"," << latency_stats_json(processing) << "},"
         << "{\"order\":3,\"name\":\"03 jetson udp send call\"," << latency_stats_json(send_call) << "}"
         << "],"
         << "\"latency_analysis_budget_us\":" << std::fixed << std::setprecision(0) << END_TO_END_LATENCY_BUDGET_US << ","
         << "\"latency_budget_pass\":" << (receive_to_output.max_us <= END_TO_END_LATENCY_BUDGET_US ? "true" : "false") << ","
         << "\"source_breakdown_complete\":" << (complete ? "true" : "false") << ","
         << "\"hardware_access\":false"
         << "}\n";
  return report.str();
}

bool write_latency_source_report(const Options& options, const LatencySourceSamples& samples) {
  if (options.latency_report_output.empty()) {
    return true;
  }
  return write_text_file(options.latency_report_output, render_latency_source_report(options, samples));
}

int run_cartesian_io_self_test(const Options& options) {
  if (options.input.empty() || options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: cartesian-io self-test requires --input and --output\n";
    return 2;
  }
  CartesianCommand command{};
  if (!load_cartesian_command(options.input, &command)) {
    return 1;
  }
  const std::string rendered = render_cartesian_response(command, "cartesian-io", "file",
                                                         "cartesian_velocity_vector",
                                                         "rtcore_joint_velocity_lease");
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  std::cout << rendered;
  return 0;
}

struct SolverSpeedCase {
  const char* name;
  double q[6];
  double direction[6];
};

int run_solver_speed_self_test(const Options& options) {
  if (options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: solver-speed self-test requires --output\n";
    return 2;
  }
  const SolverSpeedCase cases[] = {
      {"home_x_positive", {0.0, -0.35, 0.45, 0.0, 0.25, 0.0}, {0.12, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"home_y_negative", {0.0, -0.35, 0.45, 0.0, 0.25, 0.0}, {0.0, -0.10, 0.0, 0.0, 0.0, 0.0}},
      {"home_z_positive", {0.0, -0.35, 0.45, 0.0, 0.25, 0.0}, {0.0, 0.0, 0.08, 0.0, 0.0, 0.0}},
      {"elbow_forward_rz", {0.25, -0.75, 0.85, -0.20, 0.35, 0.12}, {0.05, 0.04, 0.0, 0.0, 0.0, 0.35}},
      {"elbow_back_rx", {-0.45, -0.55, 0.95, 0.30, -0.25, -0.18}, {-0.04, 0.02, 0.04, 0.22, 0.0, 0.0}},
      {"wrist_folded_ry", {0.65, -1.05, 1.20, -0.55, 0.70, -0.30}, {0.0, 0.03, -0.05, 0.0, -0.24, 0.12}},
      {"reach_left_xyz", {-0.80, -0.20, 0.65, 0.40, -0.50, 0.22}, {-0.08, 0.06, 0.04, 0.0, 0.0, -0.18}},
      {"reach_right_combo", {0.90, -0.95, 0.50, -0.45, 0.30, 0.42}, {0.06, -0.05, -0.03, 0.12, -0.10, 0.20}},
  };
  const int case_count = static_cast<int>(sizeof(cases) / sizeof(cases[0]));
  const int iterations = std::max(1, options.iterations);
  double total_us_all = 0.0;
  double max_us_all = 0.0;

  std::ostringstream out;
  out << "{"
      << "\"name\":\"robot-v4-cartesiand\","
      << "\"self_test\":\"solver-speed\","
      << "\"solver\":\"urdf_chain_damped_least_squares\","
      << "\"urdf_joint_count\":6,"
      << "\"case_count\":" << case_count << ","
      << "\"iterations_per_case\":" << iterations << ","
      << "\"cases\":[";
  for (int case_index = 0; case_index < case_count; ++case_index) {
    CartesianCommand command{};
    command.seq = case_index + 1;
    for (int axis = 0; axis < 6; ++axis) {
      command.joint_position_rad[axis] = cases[case_index].q[axis];
      command.velocity_vector[axis] = cases[case_index].direction[axis];
    }
    double case_total_us = 0.0;
    double case_max_us = 0.0;
    std::vector<double> qd;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto start = std::chrono::steady_clock::now();
      qd = damped_least_squares_joint_velocity(command);
      const auto end = std::chrono::steady_clock::now();
      const double elapsed_us = std::chrono::duration<double, std::micro>(end - start).count();
      case_total_us += elapsed_us;
      case_max_us = std::max(case_max_us, elapsed_us);
    }
    total_us_all += case_total_us;
    max_us_all = std::max(max_us_all, case_max_us);
    if (case_index > 0) {
      out << ",";
    }
    out << "{"
        << "\"name\":\"" << cases[case_index].name << "\","
        << "\"starting_joint_position_rad\":" << double_array_json(cases[case_index].q, 6) << ","
        << "\"cartesian_direction\":" << double_array_json(cases[case_index].direction, 6) << ","
        << "\"iterations\":" << iterations << ","
        << "\"mean_solve_us\":" << std::fixed << std::setprecision(3) << (case_total_us / iterations) << ","
        << "\"max_solve_us\":" << std::fixed << std::setprecision(3) << case_max_us << ","
        << "\"joint_velocity_rad_s\":" << json_array(qd) << ","
        << "\"max_abs_qd_rad_s\":" << std::fixed << std::setprecision(6) << max_abs(qd)
        << "}";
  }
  out << "],"
      << "\"mean_solve_us\":" << std::fixed << std::setprecision(3) << (total_us_all / (iterations * case_count)) << ","
      << "\"max_solve_us\":" << std::fixed << std::setprecision(3) << max_us_all << ","
      << "\"hardware_access\":false"
      << "}\n";
  const std::string rendered = out.str();
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  std::cout << rendered;
  return 0;
}

int open_udp_server_socket(const std::string& listen, std::string* ready_address) {
  HostPort host_port;
  if (!split_host_port(listen, &host_port)) {
    std::cerr << "robot-v4-cartesiand: --udp-listen must be HOST:PORT\n";
    return -1;
  }

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  addrinfo* results = nullptr;
  const int gai = ::getaddrinfo(host_port.host.c_str(), host_port.port.c_str(), &hints, &results);
  if (gai != 0) {
    std::cerr << "robot-v4-cartesiand: resolve " << listen << ": " << gai_strerror(gai) << "\n";
    return -1;
  }

  int fd = -1;
  for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
    fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (fd < 0) {
      continue;
    }
    int reuse = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (::bind(fd, item->ai_addr, item->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(results);

  if (fd < 0) {
    std::cerr << "robot-v4-cartesiand: bind " << listen << ": " << std::strerror(errno) << "\n";
    return -1;
  }

  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &bound_len) != 0) {
    std::cerr << "robot-v4-cartesiand: getsockname: " << std::strerror(errno) << "\n";
    ::close(fd);
    return -1;
  }
  char address[INET_ADDRSTRLEN]{};
  const char* rendered_addr = ::inet_ntop(AF_INET, &bound.sin_addr, address, sizeof(address));
  std::string visible_host = rendered_addr == nullptr ? host_port.host : rendered_addr;
  if (visible_host == "0.0.0.0") {
    visible_host = "127.0.0.1";
  }
  std::ostringstream ready;
  ready << visible_host << ":" << ntohs(bound.sin_port);
  *ready_address = ready.str();
  return fd;
}

bool resolve_udp_destination(const std::string& target, sockaddr_storage* out_addr, socklen_t* out_len) {
  HostPort host_port;
  if (!split_host_port(target, &host_port)) {
    return false;
  }
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo* results = nullptr;
  const int gai = ::getaddrinfo(host_port.host.c_str(), host_port.port.c_str(), &hints, &results);
  if (gai != 0 || results == nullptr) {
    if (gai != 0) {
      std::cerr << "robot-v4-cartesiand: resolve " << target << ": " << gai_strerror(gai) << "\n";
    }
    return false;
  }
  std::memset(out_addr, 0, sizeof(*out_addr));
  std::memcpy(out_addr, results->ai_addr, results->ai_addrlen);
  *out_len = static_cast<socklen_t>(results->ai_addrlen);
  ::freeaddrinfo(results);
  return true;
}

// Optional handler invoked for every drained datagram. Returning true means
// the datagram was consumed (plan preload / playback command) and must not be
// collapsed away by the keep-only-the-latest joystick stream behavior.
using UdpSpecialPacketHandler =
    std::function<bool(const unsigned char*, size_t, const sockaddr_storage&, socklen_t)>;

ssize_t receive_latest_udp_datagram(int fd, unsigned char* buffer, size_t buffer_size,
                                    sockaddr_storage* peer, socklen_t* peer_len,
                                    UdpReceiveMetadata* metadata = nullptr,
                                    const UdpSpecialPacketHandler& special_handler = nullptr) {
  ssize_t received = recvfrom_with_metadata(fd, buffer, buffer_size, peer, peer_len, metadata);
  if (received < 0) {
    return received;
  }
  if (special_handler && peer != nullptr && peer_len != nullptr &&
      special_handler(buffer, static_cast<size_t>(received), *peer, *peer_len)) {
    received = -1;
  }

  while (true) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd, &read_set);
    timeval timeout{};
    const int ready = ::select(fd + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(fd, &read_set)) {
      break;
    }
    unsigned char next_buffer[kMaxUdpDatagramBytes]{};
    sockaddr_storage next_peer{};
    socklen_t next_peer_len = sizeof(next_peer);
    UdpReceiveMetadata next_metadata{};
    const ssize_t next_received = recvfrom_with_metadata(fd, next_buffer, sizeof(next_buffer),
                                                         &next_peer, &next_peer_len,
                                                         &next_metadata);
    if (next_received < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (special_handler &&
        special_handler(next_buffer, static_cast<size_t>(next_received), next_peer, next_peer_len)) {
      continue;
    }
    std::memcpy(buffer, next_buffer, static_cast<size_t>(next_received));
    *peer = next_peer;
    *peer_len = next_peer_len;
    if (metadata != nullptr) {
      *metadata = next_metadata;
    }
    received = next_received;
  }
  if (received < 0) {
    errno = EINTR;
  }
  return received;
}
