// Private implementation header included once by robot_v4_cartesian_daemon.cpp.
// Joint error alone does not bound TCP error on a long-reach arm. Keep
// endpoint FK margin inside the pendant's 0.1 mm / 0.05 degree verification
// bounds, including the accepted Cartesian IK residual. Traversal and pure
// MOVEJ retain their existing joint gates.
struct PlanCartesianEndpointConvergence {
  bool required = false;
  bool valid = false;
  bool converged = true;
  double translation_error_m = std::numeric_limits<double>::infinity();
  double orientation_error_rad = std::numeric_limits<double>::infinity();
};

PlanCartesianEndpointConvergence plan_cartesian_endpoint_convergence(
    const PreloadedTesseractPlan& plan, const PlanPlaybackState& playback,
    const double* target_joints, const double* measured_joints) {
  PlanCartesianEndpointConvergence result;
  const bool target_active = playback.active && !playback.operator_paused &&
      (playback.seek_hold
          ? std::abs(playback.plan_elapsed_s - playback.traversal_target_time_s) <= 1e-9
          : playback.plan_elapsed_s >= plan.duration_s);
  result.required = target_active && !plan_playback_joint_ptp_tracking(
      plan, 0, static_cast<int>(plan.samples.size()) - 1);
  if (!result.required) return result;
  result.converged = false;
  if (!plan.valid || !plan.server_validated || plan.samples.size() < 2 ||
      !target_joints || !measured_joints) return result;
  if (!std::isfinite(plan.cartesian_endpoint_translation_tolerance_m) ||
      plan.cartesian_endpoint_translation_tolerance_m <= 0.0 ||
      plan.cartesian_endpoint_translation_tolerance_m > kPlanCartesianEndpointTranslationToleranceM ||
      !std::isfinite(plan.cartesian_endpoint_orientation_tolerance_rad) ||
      plan.cartesian_endpoint_orientation_tolerance_rad <= 0.0 ||
      plan.cartesian_endpoint_orientation_tolerance_rad > kPlanCartesianEndpointOrientationToleranceRad)
    return result;
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    if (!std::isfinite(target_joints[axis]) || !std::isfinite(measured_joints[axis]))
      return result;
  }
  const auto kinematics = motion_server_kinematics_snapshot_for_use();
  if (!kinematics.valid || plan.planner_urdf_path != kinematics.urdf_path ||
      plan.planner_urdf_sha256 != kinematics.urdf_sha256 ||
      (!kinematics.explicit_self_test && !rtcore_pair_config_identity_matches(
          plan.pair_config_binding, kinematics.authority.pair_config))) return result;
  for (const auto& joint : kinematics.arm_joints) {
    double axis_norm_squared = 0.0;
    for (double component : joint.axis) axis_norm_squared += component * component;
    if (!std::isfinite(axis_norm_squared) || axis_norm_squared <= 1e-24) return result;
  }
  // Both FK evaluations use one immutable, authority-bound model snapshot.
  // The scope only pins this thread's FK reads; it does not refresh authority.
  ScopedAsyncPlanningKinematics scope(kinematics);
  double target_pose[6]{}, measured_pose[6]{};
  spreadsheet_fk_pose_from_joints(target_joints, target_pose);
  spreadsheet_fk_pose_from_joints(measured_joints, measured_pose);
  for (int axis = 0; axis < 6; ++axis) {
    if (!std::isfinite(target_pose[axis]) || !std::isfinite(measured_pose[axis])) return result;
  }
  result.translation_error_m = spreadsheet_pose_translation_residual_m(target_pose, measured_pose);
  result.orientation_error_rad = spreadsheet_pose_orientation_residual_rad(target_pose, measured_pose);
  result.valid = std::isfinite(result.translation_error_m) && std::isfinite(result.orientation_error_rad);
  result.converged = result.valid &&
      result.translation_error_m <= plan.cartesian_endpoint_translation_tolerance_m &&
      result.orientation_error_rad <= plan.cartesian_endpoint_orientation_tolerance_rad;
  return result;
}

// One process owns ingress, cancellation and terminal replay for the selected
// output backend. Backend dispatch occurs before any subscriber or output opens.
struct MotionServerExecutionOwner {
  explicit MotionServerExecutionOwner(const Options& options)
      : commands(options.leader_controller_roles) {}
  LeaderAwareRobotCommandQueue commands;
  LeaderAwareCommandIDDeduper history;
  MotionServerQuiescenceBarrier quiescence;
  std::atomic<uint64_t> generation{1};

  void terminal(const RobotCommandMessage& message, const std::string& state,
                const std::string& error, const std::string& native = "") {
    if (!robot_command_has_terminal_receipt(message.kind)) return;
    const auto key = robot_command_deduplication_key(message);
    history.remember_position(key, message.reply_subject);
    auto reply = robot_command_reply_payload(message.kind,
        message.command_id_json.empty() ? message.position.command_id_json : message.command_id_json,
        state == "completed", false, state, error);
    if (!native.empty()) { reply.pop_back(); reply += ",\"native\":" + native + "}"; }
    history.complete_position(key, reply);
  }

  void cancel_queued(bool advance_generation = true) {
    if (advance_generation) generation.fetch_add(1);
    commands.discard_energizing([&](const RobotCommandMessage& message) {
      terminal(message, "cancelled", "motion cancelled before execution");
    });
  }
};

int run_rt_core_cartesian_server(const Options&, MotionServerExecutionOwner&);

// Refuse obsolete selection before constructing ingress or acquiring a grant.
int run_udp_server(const Options& options) {
  if (options.backend != "rt_core") {
    std::cerr << "reason=backend_retired backend=" << options.backend << "\n";
    return 2;
  }
  static MotionServerExecutionOwner owner(options);
  return run_rt_core_cartesian_server(options, owner);
}


int run_spreadsheet_tesseract_plan_self_test(
    const Options& options, bool emit_stdout = true) {
  if (options.output.empty()) {
    std::cerr << "robot-v4-cartesiand: spreadsheet-tesseract-plan self-test requires --output\n";
    return 2;
  }
  std::string raw;
  std::vector<SpreadsheetPlanRow> rows;
  SpreadsheetToolAxisConstraint tool_axis_constraint;
  SpreadsheetHarmoniousAxisPolicy harmonious_axis_policy;
  std::string program_id;
  std::string program_source;
  int program_revision = 0;
  int samples_per_segment = kSpreadsheetPlannerDefaultSamplesPerSegment;
  if (!options.input.empty()) {
    if (!read_text_file(options.input, &raw)) {
      return 1;
    }
    (void)parse_json_int(raw, "samples_per_segment", &samples_per_segment);
    if (!resolve_spreadsheet_plan_request(
            raw, &rows, &tool_axis_constraint, &harmonious_axis_policy,
            &program_id, &program_source, &program_revision)) {
      std::cerr << "robot-v4-cartesiand: spreadsheet-tesseract-plan input must resolve to a motion-server program or include at least two playable rows\n";
      return 1;
    }
  } else {
    fill_default_spreadsheet_plan_rows(&rows);
    program_id = "motion-server-self-test-default";
    program_source = "motion-server-self-test-fixture";
    program_revision = 1;
  }
  samples_per_segment = std::max(1, samples_per_segment);
  const auto planning_started_at = std::chrono::steady_clock::now();
  SpreadsheetMotionServerPlan plan =
      plan_motion_server_tesseract_spreadsheet_path(
          rows, samples_per_segment, tool_axis_constraint, harmonious_axis_policy,
          spreadsheet_plan_joint_speed_limits(raw),
          spreadsheet_plan_singularity_threshold(
              raw, "jacobian_singularity_warning_sigma", 0.05),
          spreadsheet_plan_singularity_threshold(
              raw, "jacobian_singularity_hard_stop_sigma", 0.02));
  plan.program_id = program_id;
  plan.program_source = program_source;
  plan.program_revision = program_revision;
  plan.planning_duration_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - planning_started_at).count();
  if (!plan.planned) {
    std::cerr << "robot-v4-cartesiand: spreadsheet-tesseract-plan failed to generate a dense plan\n";
    return 1;
  }
  const SpreadsheetCollisionCheck collision_check = run_spreadsheet_plan_collision_check(plan);
  const std::string rendered =
      render_spreadsheet_tesseract_plan_report(
          rows, plan, samples_per_segment, collision_check, options.nats_plan_subject);
  if (!write_text_file(options.output, rendered)) {
    return 1;
  }
  if (emit_stdout) {
    std::cout << rendered;
  }
  return 0;
}

// Persistent, isolated planner transport used by Offline Programming. The
// process stays warm for the application session and accepts one request at a
// time over stdin. Request payloads remain bounded by the Go host; this small
// control envelope only names host-owned input/output files.
int run_spreadsheet_tesseract_plan_server(const Options& base_options) {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    std::string request_id;
    std::string input;
    std::string output;
    if (!parse_json_string(line, "request_id", &request_id) ||
        !parse_json_string(line, "input", &input) ||
        !parse_json_string(line, "output", &output) ||
        request_id.empty() || input.empty() || output.empty()) {
      std::cout << "{\"ok\":false,\"error\":\"invalid_request_envelope\"}"
                << std::endl;
      continue;
    }
    Options request_options = base_options;
    request_options.input = input;
    request_options.output = output;
    const int result =
        run_spreadsheet_tesseract_plan_self_test(request_options, false);
    std::cout << "{\"ok\":" << (result == 0 ? "true" : "false")
              << ",\"request_id\":\"" << json_escape(request_id) << "\""
              << ",\"exit_code\":" << result << "}" << std::endl;
  }
  return 0;
}

std::string utc_timestamp() {
  std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buffer[32]{};
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}
