// Private implementation header included once by robot_v4_cartesian_daemon.cpp.
void print_usage(const char* argv0) {
  std::cerr << "usage: " << argv0
            << " [--backend rt_core]"
            << " [--rt-control-socket PATH --rt-jog-socket PATH --rt-pair-id ID"
            << " --rt-pair-revision N --rt-configuration-sha256 HASH"
            << " --rt-axis-ids J1,J2,J3,J4,J5,J6 --rt-urdf PATH --rt-urdf-sha256 HASH]"
            << " [--udp-listen HOST:PORT"
            << " [--udp-ready-file PATH] [--latency-report-output PATH] [--status-output PATH]"
            << " [--daemon-state-url http://127.0.0.1:8787/api/state]"
            << " [--nats-url nats://HOST:PORT|daemon-v1://PEER/ROLE/NAME] [--nats-status-subject SUBJECT]"
            << " [--nats-plan-subject SUBJECT]"
            << " [--diagnostic-echo-source] [--trusted-lan-leader-authority]"
            << " [--oneshot] [--samples N] [--control-frequency-hz N]]"
            << " [--self-test cartesian-io --input PATH --output PATH]"
            << " [--self-test spreadsheet-tesseract-plan --manifest PATH --input PATH --output PATH]"
            << " [--self-test spreadsheet-tesseract-plan-server]"
            << " [--self-test accepted-plan-contract --manifest PATH --input PATH --output PATH]"
            << " [--self-test solver-speed --iterations N --output PATH]"
            << " [--self-test kinematics-authority --input PATH --output PATH]"
            << " [--self-test table-calibration-fit --input PATH --output PATH]"
            << " [--self-test table-calibration-apply --input PATH --output PATH]\n";
}

// NATS subjects live in the deployment manifest, not in code or baked args:
// robot_command_subject (unified per-cell robot command subject) and the
// motion-server node's cold_path_subjects (.status/.plan). Explicit CLI args
// still override for tests and diagnostics.
void apply_manifest_nats_config(Options* options) {
  if (options->rtcore_status_subject.empty()) {
    const char* status_subject =
        std::getenv("ROBOT_V4_RTCORE_STATUS_SUBJECT");
    if (status_subject != nullptr) {
      options->rtcore_status_subject = status_subject;
    }
  }
  std::string path = options->manifest;
  if (path.empty()) {
    const char* env = std::getenv("ROBOT_V4_MANIFEST");
    if (env != nullptr) {
      path = env;
    }
  }
  if (path.empty()) {
    return;
  }
  options->manifest = path;
  std::ifstream in(path);
  if (!in.good()) {
    std::cerr << "robot-v4-cartesiand: manifest not readable: " << path << "\n";
    return;
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  const std::string manifest = buffer.str();
  const auto ends_with = [](const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
  };
  if (options->robot_command_subject.empty()) {
    (void)parse_json_string(manifest, "robot_command_subject", &options->robot_command_subject);
  }
  if (options->robot_cell.empty()) {
    (void)parse_json_string(manifest, "name", &options->robot_cell);
  }
  if (manifest.find("\"leader_controller_roles\"") !=
      std::string::npos) {
    std::string configured_roles_raw;
    std::vector<std::string> configured_roles;
    if (!extract_unique_top_level_json_member(
            manifest, "leader_controller_roles",
            &configured_roles_raw) ||
        !parse_exact_json_string_array_value(
            configured_roles_raw, &configured_roles) ||
        configured_roles.empty()) {
      options->leader_controller_roles_valid = false;
    }
    std::set<std::string> unique_roles;
    for (const std::string& role : configured_roles) {
      if (!ControllerLeaderAuthority::valid_controller_role(role) ||
          !unique_roles.insert(role).second) {
        options->leader_controller_roles_valid = false;
        break;
      }
    }
    if (options->leader_controller_roles_valid) {
      options->leader_controller_roles.assign(
          unique_roles.begin(), unique_roles.end());
    }
  }
  for (const std::string& node : json_object_slices_for_array_key(manifest, "nodes")) {
    std::string role;
    (void)parse_json_string(node, "role", &role);
    if (role == "realtime-core") {
      for (const std::string& subject :
           parse_json_string_array(node, "cold_path_subjects")) {
        if (options->manifest_rtcore_status_subject.empty() &&
            ends_with(subject, ".status")) {
          options->manifest_rtcore_status_subject = subject;
        }
      }
      continue;
    }
    if (role != "motion-server") {
      continue;
    }
    std::string peer;
    if (parse_json_string(node, "peer", &peer) &&
        !peer.empty() &&
        daemon_peer_is_plain_concrete(peer) &&
        !daemon_identity_is_semantic_role(peer)) {
      options->motion_server_peer = peer;
    }
    for (const std::string& subject : parse_json_string_array(node, "cold_path_subjects")) {
      if (options->nats_status_subject.empty() && ends_with(subject, ".status")) {
        options->nats_status_subject = subject;
      }
      if (options->nats_plan_subject.empty() && ends_with(subject, ".plan")) {
        options->nats_plan_subject = subject;
      }
    }
  }
  if (options->rtcore_status_subject.empty()) {
    options->rtcore_status_subject = options->manifest_rtcore_status_subject;
  }
}

std::string robot_v4_subject_host_token(const std::string& subject,
                                        const std::string& prefix) {
  if (subject.rfind(prefix, 0) != 0) {
    return "";
  }
  const std::size_t end = subject.find('.', prefix.size());
  if (end == std::string::npos || end == prefix.size()) {
    return "";
  }
  return subject.substr(prefix.size(), end - prefix.size());
}

bool robot_v4_motion_server_identity_options_valid(const Options& options) {
  if (!options.leader_controller_roles_valid) {
    std::cerr << "robot-v4-cartesiand: leader_controller_roles must be a unique array of canonical controller roles\n";
    return false;
  }
  HostPort daemon_state;
  std::string daemon_path;
  if (!parse_http_url(options.daemon_state_url, &daemon_state, &daemon_path)) {
    std::cerr << "robot-v4-cartesiand: daemon state URL must use a concrete host, got "
              << options.daemon_state_url << "\n";
    return false;
  }
  if (!options.nats_url.empty()) {
    std::string peer;
    std::string role;
    std::string service;
    const bool daemon_ref = options.nats_url.rfind("daemon-v1://", 0) == 0;
    if ((daemon_ref && !daemon_service_ref_parts(options.nats_url, &peer, &role, &service)) ||
        (!daemon_ref && !direct_network_target_has_concrete_host(options.nats_url, "nats://"))) {
      std::cerr << "robot-v4-cartesiand: NATS URL must use peer/role/service or a concrete host, got "
                << options.nats_url << "\n";
      return false;
    }
  }
  if (!options.motion_server_peer.empty() &&
      (!daemon_peer_is_plain_concrete(options.motion_server_peer) ||
       daemon_identity_is_semantic_role(options.motion_server_peer))) {
    std::cerr << "robot-v4-cartesiand: motion-server peer must be a plain concrete peer, got "
              << options.motion_server_peer << "\n";
    return false;
  }
  if (!options.robot_cell.empty() &&
      (!daemon_peer_is_plain_concrete(options.robot_cell) ||
       daemon_identity_is_semantic_role(options.robot_cell))) {
    std::cerr << "robot-v4-cartesiand: robot cell must use a plain concrete peer token, got "
              << options.robot_cell << "\n";
    return false;
  }

  if (!options.udp_listen.empty() &&
      !direct_network_target_has_concrete_host(options.udp_listen, "", true)) {
    std::cerr << "robot-v4-cartesiand: UDP listen must use a concrete HOST:PORT, got "
              << options.udp_listen << "\n";
    return false;
  }
  if (!options.target.empty() &&
      !direct_network_target_has_concrete_host(options.target, "")) {
    std::cerr << "robot-v4-cartesiand: self-test target must use a concrete HOST:PORT, got "
              << options.target << "\n";
    return false;
  }
  for (const std::string* subject : {
           &options.nats_status_subject,
           &options.nats_plan_subject,
           &options.robot_command_subject,
       }) {
    if (subject->empty()) {
      continue;
    }
    std::string host = robot_v4_subject_host_token(*subject, "robot/v4/motion-server.");
    if (host.empty()) {
      host = robot_v4_subject_host_token(*subject, "robot/v4/robot.");
    }
    if (host.empty() || !daemon_peer_is_plain_concrete(host) ||
        daemon_identity_is_semantic_role(host) ||
        (!options.motion_server_peer.empty() && host != options.motion_server_peer)) {
      std::cerr << "robot-v4-cartesiand: subject must use the selected concrete motion-server peer, got "
                << *subject << "\n";
      return false;
      }
  }
  if (!options.rtcore_status_subject.empty()) {
    const std::string host =
        rtcore_status_host_from_subject(options.rtcore_status_subject);
    if (host.empty() || !daemon_peer_is_plain_concrete(host) ||
        daemon_identity_is_semantic_role(host)) {
      std::cerr << "robot-v4-cartesiand: RTCore status subject must use a concrete RTCore host, got "
                << options.rtcore_status_subject << "\n";
      return false;
    }
    if (!options.manifest_rtcore_status_subject.empty() &&
        options.rtcore_status_subject !=
            options.manifest_rtcore_status_subject) {
      std::cerr << "robot-v4-cartesiand: RTCore status subject must match the selected manifest RTCore status subject, got "
                << options.rtcore_status_subject << " expected "
                << options.manifest_rtcore_status_subject << "\n";
      return false;
    }
  }
  return true;
}

bool parse_args(int argc, char** argv, Options* options) {
  bool selected_backend = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--backend" && i + 1 < argc) {
      selected_backend = true;
      options->backend = argv[++i];
      if (options->backend != "rt_core") {
        std::cerr << "reason=backend_retired backend=" << options->backend << "\n";
        return false;
      }
    } else if (arg == "--rt-control-socket" && i + 1 < argc) {
      options->rt_core.socket = argv[++i];
    } else if (arg == "--rt-jog-socket" && i + 1 < argc) {
      options->rt_core.jog_socket = argv[++i];
    } else if (arg == "--rt-pair-id" && i + 1 < argc) {
      options->rt_core.binding.PairID = argv[++i];
    } else if (arg == "--rt-pair-revision" && i + 1 < argc) {
      size_t revision = 0;
      if (!parse_config_size(argv[++i], &revision) || !revision) return false;
      options->rt_core.binding.Revision = revision;
    } else if (arg == "--rt-configuration-sha256" && i + 1 < argc) {
      options->rt_core.binding.ConfigurationSHA256 = argv[++i];
    } else if (arg == "--rt-axis-ids" && i + 1 < argc) {
      options->rt_core.joint_ids = split_config_fields(argv[++i], ',');
    } else if (arg == "--rt-urdf" && i + 1 < argc) {
      options->rt_core.urdf = argv[++i];
    } else if (arg == "--rt-urdf-sha256" && i + 1 < argc) {
      options->rt_core.urdf_sha256 = argv[++i];
    } else if (arg == "--self-test" && i + 1 < argc) {
      options->self_test = argv[++i];
    } else if (arg == "--input" && i + 1 < argc) {
      options->input = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      options->output = argv[++i];
    } else if (arg == "--manifest" && i + 1 < argc) {
      options->manifest = argv[++i];
    } else if (arg == "--udp-listen" && i + 1 < argc) {
      options->udp_listen = argv[++i];
    } else if (arg == "--udp-ready-file" && i + 1 < argc) {
      options->udp_ready_file = argv[++i];
    } else if (arg == "--latency-report-output" && i + 1 < argc) {
      options->latency_report_output = argv[++i];
    } else if (arg == "--status-output" && i + 1 < argc) {
      options->status_output = argv[++i];
    } else if (arg == "--nats-url" && i + 1 < argc) {
      options->nats_url = argv[++i];
    } else if (arg == "--nats-status-subject" && i + 1 < argc) {
      options->nats_status_subject = argv[++i];
    } else if (arg == "--rtcore-status-subject" && i + 1 < argc) {
      options->rtcore_status_subject = argv[++i];
    } else if (arg == "--robot-command-subject" && i + 1 < argc) {
      options->robot_command_subject = argv[++i];
    } else if (arg == "--robot-cell" && i + 1 < argc) {
      options->robot_cell = argv[++i];
    } else if (arg == "--nats-go-home-subject" || arg == "--nats-arm-subject" ||
               arg == "--nats-io-subject" || arg == "--rtcore-target") {
      std::cerr << "reason=backend_retired flag=" << arg << "\n";
      return false;
    } else if (arg == "--nats-plan-subject" && i + 1 < argc) {
      options->nats_plan_subject = argv[++i];
    } else if (arg == "--target" && i + 1 < argc) {
      options->target = argv[++i];
    } else if (arg == "--daemon-state-url" && i + 1 < argc) {
      options->daemon_state_url = argv[++i];
    } else if (arg == "--samples" && i + 1 < argc) {
      if (!parse_positive_int_arg(argv[++i], &options->samples)) {
        return false;
      }
    } else if (arg == "--period-ms" && i + 1 < argc) {
      if (!parse_positive_int_arg(argv[++i], &options->period_ms)) {
        return false;
      }
    } else if (arg == "--control-frequency-hz" && i + 1 < argc) {
      if (!parse_positive_int_arg(argv[++i], &options->control_frequency_hz)) {
        return false;
      }
    } else if (arg == "--iterations" && i + 1 < argc) {
      if (!parse_positive_int_arg(argv[++i], &options->iterations)) {
        return false;
      }
    } else if (arg == "--diagnostic-echo-source") {
      options->diagnostic_echo_source = true;
    } else if (arg == "--trusted-lan-leader-authority") {
      options->trusted_lan_leader_authority = true;
    } else if (arg == "--oneshot") {
      options->oneshot = true;
    } else {
      return false;
    }
  }
  // Unverified repository launch default; offline solvers require no runtime binding.
  if (!selected_backend) {
    const char* environment = std::getenv("MOTION_SERVER_BACKEND");
    if (environment && *environment) options->backend = environment;
    else options->backend = "rt_core";
  }
  if (options->backend != "rt_core") {
    std::cerr << "reason=backend_retired backend=" << options->backend << "\n";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 && std::string(argv[1]) == "--resolve-only") {
    if (argc != 4) return 2;
    return run_cartesian_resolver(argv[2], argv[3]);
  }
  Options options;
  if (!parse_args(argc, argv, &options)) {
    print_usage(argv[0]);
    return 2;
  }
  g_motion_axis_runtime_allow_self_test_defaults =
      !options.self_test.empty();
  apply_manifest_nats_config(&options);
  if (!robot_v4_motion_server_identity_options_valid(options)) {
    return 2;
  }
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif

  if (options.self_test.empty()) {
    const auto& c = options.rt_core;
    const char* missing = c.socket.empty() ? "--rt-control-socket" :
        c.jog_socket.empty() ? "--rt-jog-socket" :
        c.binding.PairID.empty() ? "--rt-pair-id" :
        !c.binding.Revision ? "--rt-pair-revision" :
        c.binding.ConfigurationSHA256.empty() ? "--rt-configuration-sha256" :
        c.joint_ids.empty() ? "--rt-axis-ids" :
        c.urdf.empty() ? "--rt-urdf" :
        c.urdf_sha256.empty() ? "--rt-urdf-sha256" : nullptr;
    if (missing) {
      std::cerr << "robot-v4-cartesiand: rt_core missing binding " << missing << "\n";
      return 2;
    }
    return run_udp_server(options);
  }

  if (!options.self_test.empty()) {
    // Offline solver/report tests may parse the explicitly selected URDF
    // without a live RTCore tracker. Any test that enters run_udp_server has
    // this bypass cleared again at server startup.
    set_motion_server_explicit_self_test_kinematics(true);
    if (options.self_test == "cartesian-io") {
      return run_cartesian_io_self_test(options);
    }

    if (options.self_test == "spreadsheet-tesseract-plan") {
      return run_spreadsheet_tesseract_plan_self_test(options);
    }
    if (options.self_test == "spreadsheet-tesseract-plan-server") {
      return run_spreadsheet_tesseract_plan_server(options);
    }
    if (options.self_test == "accepted-plan-contract") {
      return run_robot_v4_accepted_plan_contract_self_test(options);
    }
    if (options.self_test == "solver-speed") {
      return run_solver_speed_self_test(options);
    }
    if (options.self_test == "kinematics-authority") {
      return run_kinematics_authority_self_test(options);
    }
    if (options.self_test == "table-calibration-fit") {
      return run_table_calibration_fit_self_test(options);
    }
    if (options.self_test == "table-calibration-apply") {
      return run_table_calibration_apply_self_test(options);
    }


    print_usage(argv[0]);
    return 2;
  }

  return 2;
}
