// Private implementation header included after the existing resolver. This
// runtime never starts a v4 status subscriber or sends v4 runtime commands.

// The native scalar contract requires an explicit joint and exactly one finite
// target. Unresolved poses/vectors cannot inherit the legacy codec's zero defaults.
bool rt_core_position_input_resolved(const RobotCommandMessage& message) {
  PositionExecutionRequest parsed;
  if (!parse_position_execution_request(message.raw_payload, &parsed) ||
      parsed.axis < 0 || parsed.axis >= 6 || !std::isfinite(parsed.target_rad)) return false;
  std::string raw;
  bool present = false;
  if (!extract_unique_top_level_json_member(message.raw_payload, "axis", &raw, &present) || !present) return false;
  for (const auto* key : {"tcp_pose", "joint_targets", "joint_positions"}) {
    if (!extract_unique_top_level_json_member(message.raw_payload, key, &raw, &present) || present) return false;
  }
  unsigned targets = 0;
  for (const auto* key : {"target_rad", "target_deg", "relative_jog_rad"}) {
    if (!extract_unique_top_level_json_member(message.raw_payload, key, &raw, &present)) return false;
    targets += present;
  }
  return targets == 1;
}

std::string rt_core_cartesian_status(const cartesian_rt::Backend& backend,
                                     const RuntimeStatus& live,
                                     const LeaderLeaseStatus& leader) {
  namespace api = rosie::rt_control;
  api::Json out = api::Json::object_value();
  const auto add = [&](const std::string& key, const auto& value) {
    out.object.emplace_back(key, api::to_json(value));
  };
  add("name", std::string("robot-v4-cartesiand"));
  add("status_schema", std::string(kStatusSchema));
  add("backend", std::string("rt_core"));
  add("state", backend.reason().empty() ? std::string("running") : std::string("inhibited"));
  add("servos_armed_requested", backend.armed());
  add("last_stop_confirmed", backend.stop_confirmed());
  add("latest_command_kind", live.latest_command_kind);
  add("latest_processed_cold_command_id", live.latest_processed_cold_command_id);
  add("latest_processed_cold_command_kind", live.latest_processed_cold_command_kind);
  add("latest_processed_cold_accepted", live.latest_processed_cold_accepted);
  add("latest_processed_cold_result", live.latest_processed_cold_result);
  add("latest_sequence", live.latest_sequence);
  add("datagrams_received", live.datagrams_received);
  add("rejected_input_count", live.rejected_input_count);
  add("rtcore_outputs_sent", live.rtcore_outputs_sent);
  add("latest_input_fresh", backend.jogging());
  add("stale_zero_hold", !backend.jogging());
  add("latest_qd_units", std::string("rad/s"));
  add("latest_applied_qd_rad_s", backend.applied());
  add("latest_joint_limit_scale", live.latest_joint_limit_scale);
  add("latest_singularity_scale", live.latest_singularity_scale);
  add("latest_singularity_class", live.latest_singularity_class);
  add("leader_id", leader.leader_id);
  add("leader_boot_id", leader.leader_boot_id);
  add("leader_lease_id", leader.leader_lease_id);
  add("leader_fence_epoch", leader.leader_fence_epoch);
  add("leader_campaign_generation", leader.leader_campaign_generation);
  add("leader_lease_fresh", leader.leader_lease_fresh);
  add("leader_expires_in_ms", leader.leader_expires_in_ms);
  add("leader_transition_pending", leader.leader_transition_pending);
  add("latest_leader_result", leader.latest_leader_result);
  add("latest_leader_request_id", leader.latest_leader_request_id);
  add("latest_leader_campaign_generation", leader.latest_leader_campaign_generation);
  add("rt_core_status", backend.status());
  add("axes", backend.status().Core.Axes);
  if (backend.status().Core.Recovery) add("fault_table", backend.status().Core.Recovery->Faults);
  else out.object.emplace_back("fault_table", api::Json{});
  add("refusal", backend.reason());
  out.object.emplace_back("typed_refusal", backend.refusal());
  for (const auto* key : {"runtime_mode", "runtime_mode_epoch", "activation_epoch",
                         "rtcore_pair_authority_runtime_mode_epoch", "plan_playback_active",
                         "rtcore_pair_authority_observed", "home_profile_key"})
    out.object.emplace_back(key, api::Json{});
  add("v4_fields_available", false);
  return api::encode(out);
}

// Bind immutable kinematics to explicit deployment input, never to a synthetic
// v4 mode or Home identity. A changed file stops motion before the next solve.
MotionServerKinematicsSnapshot load_rt_core_kinematics(const cartesian_rt::Configuration& config) {
  MotionServerKinematicsSnapshot kinematics;
  std::string urdf;
  if (config.urdf_sha256.empty() || !read_motion_server_kinematics_urdf(config.urdf, &urdf) ||
      sha256_lower_hex(urdf) != config.urdf_sha256 ||
      !parse_motion_server_kinematics_urdf(urdf, &kinematics) ||
      !motion_server_urdf_file_identity(config.urdf, &kinematics.file_identity))
    throw std::runtime_error("rt_core_urdf_binding_invalid");
  kinematics.valid = true;
  kinematics.urdf_path = config.urdf;
  kinematics.urdf_sha256 = config.urdf_sha256;
  return kinematics;
}

std::string rt_core_native_receipt(const cartesian_rt::Backend& backend,
                                   const RobotCommandMessage& message, std::uint64_t home_epoch = 0) {
  namespace api = rosie::rt_control;
  auto out = api::Json::object_value();
  out.object.emplace_back("binding", api::to_json(backend.binding()));
  out.object.emplace_back("session", api::to_json(backend.session()));
  out.object.emplace_back("generation", api::to_json(backend.generation()));
  out.object.emplace_back("run_id", api::to_json(message.bridge_run.run_id));
  out.object.emplace_back("handle", api::to_json(message.kind == "position" ? backend.handle() : 0));
  out.object.emplace_back("sequence", api::to_json(message.kind == "position" ? backend.native_sequence() : 0));
  if (message.kind == "position") {
    out.object.emplace_back("identity", api::to_json(backend.trajectory_identity()));
    out.object.emplace_back("execution_generation", api::to_json(backend.execution_generation()));
  }
  out.object.emplace_back("home_epoch", api::to_json(home_epoch));
  out.object.emplace_back("result", backend.refusal());
  if (message.kind == "position") out.object.emplace_back("motion", api::to_json(backend.status().Motion));
  return api::encode(out);
}

// Source clocks never become local deadlines directly. An exact challenge
// anchors the bridge's remaining lifetime at send, preventing transit renewal.
struct RTCoreRunAdmission {
  RobotCommandMessage message;
  std::string challenge;
  std::uint64_t sent_ns = 0;
  bool pending = false;
  static std::string key(const BridgeRunCredential& run) {
    return rosie::rt_control::encode(std::vector<std::string>{run.bridge_boot_id, run.run_id,
        run.controller_id, run.controller_boot_id});
  }
  bool verify(const BridgePositionPermit& p, std::uint64_t now, std::uint64_t* deadline) const {
    const auto& r = message.bridge_run;
    LeaderLeaseCredential credential;
    if (!pending || !decode_leader_lease_credential(message.raw_payload, &credential) ||
        p.challenge_id != challenge || p.bridge_boot_id != r.bridge_boot_id || p.run_id != r.run_id ||
        p.controller_id != r.controller_id || p.controller_boot_id != r.controller_boot_id ||
        p.command_id_json != message.command_id_json || p.leader_lease_id != credential.lease_id ||
        p.leader_fence_epoch != credential.fence_epoch || p.deadline_monotonic_ms != r.deadline_monotonic_ms ||
        p.run_deadline_monotonic_ms != r.run_deadline_monotonic_ms ||
        p.bridge_observed_monotonic_ms >= p.deadline_monotonic_ms ||
        p.run_deadline_monotonic_ms < p.deadline_monotonic_ms ||
        p.remaining_ms != p.deadline_monotonic_ms - p.bridge_observed_monotonic_ms ||
        p.run_remaining_ms != p.run_deadline_monotonic_ms - p.bridge_observed_monotonic_ms ||
        p.run_remaining_ms > (std::numeric_limits<std::uint64_t>::max() - sent_ns) / 1000000 ||
        now >= sent_ns + p.remaining_ms * 1000000) return false;
    *deadline = sent_ns + p.run_remaining_ms * 1000000;
    return now < *deadline;
  }
};

// Retain a native Home receipt until a fresh, same-incarnation observation
// proves completion. Queue acceptance cannot manufacture a terminal success.
struct RTCoreHomeReceipt {
  RobotCommandMessage message;
  rosie::rt_control::ProcessStatus before;
  std::uint32_t mask = 0;
  bool pending = false;
  std::string native;

  void cancel(MotionServerExecutionOwner& owner, const std::string& reason) {
    if (pending) owner.terminal(message, "cancelled", reason, native);
    pending = false;
  }
  bool observe(MotionServerExecutionOwner& owner, const rosie::rt_control::ProcessStatus& status) {
    if (!pending) return true;
    if (status.DaemonIncarnation != before.DaemonIncarnation ||
        status.AdapterIncarnation != before.AdapterIncarnation ||
        status.Core.ConfigurationEpoch != before.Core.ConfigurationEpoch ||
        !before.Grant || !status.Grant || !status.Grant->GrantActive ||
        status.Grant->AppGrantGeneration != before.Grant->AppGrantGeneration) {
      owner.terminal(message, "failed", "rt_core_home_identity_changed", native);
      pending = false;
      return false;
    } else if (status.Core.TimeNS > before.Core.TimeNS &&
               status.Core.HomeEpoch > before.Core.HomeEpoch &&
               status.Core.NativeHomeActiveMask == 0 && status.Core.CommissioningPhase == 0) {
      const bool homed = (status.Core.HomeValidMask & mask) == mask;
      auto receipt = native.empty() ? std::string("{}") : native;
      receipt.pop_back();
      receipt += ",\"completed_home_epoch\":" + std::to_string(status.Core.HomeEpoch) + "}";
      owner.terminal(message, homed ? "completed" : "failed", homed ? "" : "rt_core_home_failed", receipt);
      pending = false;
      return homed;
    }
    return true;
  }
};

int run_rt_core_cartesian_server(const Options& options, MotionServerExecutionOwner& owner) {
  namespace api = rosie::rt_control;
  // The existing cooperative leader protocol is required for both simulation
  // and hardware; selecting a backend must never bypass ingress authority.
  if (options.udp_listen.empty() || options.nats_url.empty() || options.robot_command_subject.empty() ||
      !options.trusted_lan_leader_authority || options.rt_core.socket.empty() || options.rt_core.jog_socket.empty()) {
    std::cerr << "rt_core requires local control/jog sockets, UDP input and configured trusted-LAN NATS leadership\n";
    return 2;
  }
  try {
    auto kinematics = load_rt_core_kinematics(options.rt_core);
    cartesian_rt::LocalClient client(options.rt_core);
    cartesian_rt::Backend backend(client, options.rt_core);
    RuntimeStatus live;
    ControllerLeaderAuthority authority(api::host_monotonic_ns(), options.leader_controller_roles);
    auto& queue = owner.commands;
    auto& quiescence = owner.quiescence;
    auto& planning_generation = owner.generation;
    std::string ready_address;
    const int fd = open_udp_server_socket(options.udp_listen, &ready_address);
    if (fd < 0) return 1;
    struct SocketOwner { int fd; ~SocketOwner() { ::close(fd); } } socket_owner{fd};
    if (::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK) < 0)
      throw std::runtime_error("rt_core_input_nonblocking_failed");
    enable_udp_receive_timestamps(fd);
    const auto publish = [&] {
      const auto body = rt_core_cartesian_status(backend, live, authority.status(std::chrono::steady_clock::now()));
      if (!options.status_output.empty()) {
        const auto temporary = options.status_output + ".tmp";
        if (!write_text_file(temporary, body) || ::rename(temporary.c_str(), options.status_output.c_str()) != 0)
          throw std::runtime_error("rt_core_status_publish_failed");
      }
      publish_status_to_nats_async(options, body);
    };
    if (!backend.start()) { publish(); return 1; }
    std::thread(run_robot_command_subscriber, options, &queue, &quiescence,
                &planning_generation, &owner.history).detach();
    struct StopSubscriber { ~StopSubscriber() { keep_running.store(false); } } stop_subscriber;
    if (!options.udp_ready_file.empty() && !write_text_file(options.udp_ready_file, ready_address + "\n")) return 1;
    StreamSmoother smoother;
    SolverCache solver;
    std::string hot_lease;
    uint32_t hot_sequence = 0;
    RTCoreHomeReceipt home_receipt;
    RTCoreRunAdmission admission;
    RobotCommandMessage position_message;
    std::optional<RobotCommandMessage> admitted_position;
    bool position_pending = false;
    std::uint64_t position_deadline = 0, run_deadline = 0;
    std::map<std::string, std::uint64_t> intent_sequences;
    auto observed_generation = planning_generation.load();
    const auto halt = [&](const std::string& reason) {
      smoother = {};
      owner.cancel_queued();
      observed_generation = planning_generation.load();
      if (home_receipt.pending) home_receipt.native = rt_core_native_receipt(backend, home_receipt.message, backend.status().Core.HomeEpoch);
      home_receipt.cancel(owner, "rt_core_home_abandoned");
      if (admission.pending) owner.terminal(admission.message, "cancelled", reason);
      admission.pending = false;
      if (admitted_position) owner.terminal(*admitted_position, "cancelled", reason,
          rt_core_native_receipt(backend, *admitted_position));
      admitted_position.reset();
      if (position_pending) owner.terminal(position_message, "cancelled", reason,
          rt_core_native_receipt(backend, position_message));
      position_pending = false;
      return backend.retire_run(reason);
    };
    const auto transition = [&](const LeaderLeaseAction& action) {
      if (action.transition == LeaderLeaseTransition::kNone) return;
      const bool stopped = halt("leader_transition");
      authority.finish_transition(stopped, std::chrono::steady_clock::now());
      hot_lease.clear(); hot_sequence = 0;
    };
    while (keep_running.load()) {
      // Transport cancellation is authoritative even if the bounded queue
      // could not retain its notification.
      if (planning_generation.load() != observed_generation) halt("ingress_cancelled");
      transition(authority.begin_expiry(std::chrono::steady_clock::now()));
      if (!backend.poll(api::host_monotonic_ns()) &&
          (home_receipt.pending || position_pending || admitted_position || !backend.run_key().empty())) {
        if (backend.reason() != "rt_core_run_expired") {
          if (home_receipt.pending) owner.terminal(home_receipt.message, "failed", backend.reason(),
              rt_core_native_receipt(backend, home_receipt.message, backend.status().Core.HomeEpoch));
          if (position_pending) owner.terminal(position_message, "failed", backend.reason(),
              rt_core_native_receipt(backend, position_message));
          home_receipt.pending = position_pending = false;
        }
        halt(backend.reason());
      }
      if (backend.home_confirmed() && !home_receipt.observe(owner, backend.status())) halt("rt_core_home_observation_failed");
      if (admission.pending && api::host_monotonic_ns() >= admission.sent_ns +
          std::uint64_t(admission.message.position.timeout.count()) * 1000000) halt("rt_core_admission_timeout");
      if (admitted_position) {
        if (api::host_monotonic_ns() >= position_deadline) halt("rt_core_position_timeout");
        else if (backend.motion_ready()) {
          if (!queue.push(*admitted_position)) halt("rt_core_position_queue_full");
          admitted_position.reset();
        }
      }
      if (position_pending) {
        if (api::host_monotonic_ns() >= position_deadline) halt("rt_core_position_timeout");
        else if (backend.position_complete()) {
          owner.terminal(position_message, "completed", "", rt_core_native_receipt(backend, position_message));
          position_pending = false;
        } else if (backend.run_key().empty()) {
          owner.terminal(position_message, "failed", backend.reason(), rt_core_native_receipt(backend, position_message));
          position_pending = false;
          halt(backend.reason());
        }
      }
      RobotCommandMessage message;
      // Drain a bounded snapshot; a busy broker cannot starve status or Stop.
      // Unverified work cap: 64 is the existing command queue capacity.
      for (unsigned count = 0; count < 64 && queue.pop(&message); ++count) {
        bool accepted = false;
        if (message.kind == "bridge_position_permit_reply") {
          BridgePositionPermit permit;
          std::uint64_t deadline = 0;
          LeaderLeaseCredential credential;
          if (admission.pending && decode_bridge_position_permit(message.raw_payload, &permit) &&
              admission.verify(permit, api::host_monotonic_ns(), &deadline) &&
              decode_leader_lease_credential(admission.message.raw_payload, &credential) &&
              authority.authorizes(admission.message.sender_id, credential, std::chrono::steady_clock::now()) &&
              admission.message.planning_generation == planning_generation.load()) {
            message = admission.message;
            admission.pending = false;
            const auto authorized = [&, generation = message.planning_generation] {
              return generation == planning_generation.load();
            };
            if (!backend.bind_run(RTCoreRunAdmission::key(message.bridge_run), deadline) || !backend.arm(authorized)) {
              backend.retire_run(backend.reason());
              owner.terminal(message, backend.reason() == "rt_core_execution_cancelled" ? "cancelled" : "failed",
                  backend.reason(), rt_core_native_receipt(backend, message));
              continue;
            }
            run_deadline = backend.run_deadline();
            message.position_permit_return = true;
            admitted_position = message;
            position_deadline = std::min(run_deadline, admission.sent_ns + std::uint64_t(message.position.timeout.count()) * 1000000);
            continue;
          } else {
            if (admission.pending) halt("rt_core_source_permit_rejected");
            continue;
          }
        }
        if (leader_lease_command_kind(message.kind)) {
          LeaderLeaseRequest request;
          if (decode_leader_lease_request(message.raw_payload, &request)) {
            const auto prior = authority.status(std::chrono::steady_clock::now());
            const auto action = authority.begin_request(request, std::chrono::steady_clock::now());
            transition(action);
            const auto after = authority.status(std::chrono::steady_clock::now());
            if (prior.leader_lease_id != after.leader_lease_id)
              queue.retire_exit_reservations_covered_by(prior.leader_id, prior.leader_boot_id, prior.leader_campaign_generation);
            if (!action.accepted || message.kind == "leader_cancel" || message.kind == "leader_release")
              queue.retire_exit_reservations_covered_by(request.sender_id, request.controller_boot_id, request.campaign_generation);
            accepted = action.accepted && after.latest_leader_result != "revoke_safety_issue_failed";
          }
        } else if (message.kind == "stop" || message.kind == "disarm" ||
                   message.kind == "bridge_permit_transport_lost" || message.kind == "bridge_run_revoke" ||
                   (message.kind == "path_scrub" && message.has_direction && message.direction == 0)) {
          accepted = halt("operator_stop");
        } else {
          LeaderLeaseCredential credential;
          const bool owns = decode_leader_lease_credential(message.raw_payload, &credential) &&
              authority.authorizes(message.sender_id, credential, std::chrono::steady_clock::now());
          if (message.planning_generation != planning_generation.load()) {
            owner.terminal(message, "cancelled", "motion cancelled before execution");
          } else if (!owns) {
            halt("rt_core_command_unowned");
          } else if (message.kind == "go_home" &&
              (!message.bridge_run.valid() || RTCoreRunAdmission::key(message.bridge_run) != backend.run_key())) {
            halt("rt_core_run_not_admitted");
          } else if (message.bridge_run.run_id.empty() && !backend.run_key().empty()) {
            owner.terminal(message, "failed", "rt_core_bound_run_identity_required");
            continue;
          } else if (!message.bridge_run.run_id.empty() && message.kind != "position" &&
              (!message.bridge_run.valid() || RTCoreRunAdmission::key(message.bridge_run) != backend.run_key())) {
            halt("rt_core_run_not_admitted");
          } else if (message.kind == "position") {
            if (!rt_core_position_input_resolved(message)) {
              owner.terminal(message, "failed", "rt_core_position_input_unresolved");
              continue;
            }
            const auto now = api::host_monotonic_ns();
            const auto& run = message.bridge_run;
            if (!run.valid() || run.bridge_boot_id.empty() || run.controller_id != message.sender_id ||
                run.controller_boot_id != credential.controller_boot_id || !message.has_motion_intent_seq ||
                (!message.position_permit_return && message.motion_intent_seq <= intent_sequences[message.sender_id]) ||
                position_pending || home_receipt.pending || admission.pending || admitted_position || !backend.armed()) {
              owner.terminal(message, "failed", "rt_core_position_identity_or_readiness");
              continue;
            }
            intent_sequences[message.sender_id] = message.motion_intent_seq;
            if (!message.position_permit_return) {
              admission.message = message;
              admission.sent_ns = now;
              admission.challenge = "native-" + std::to_string(now);
              admission.pending = true;
              auto raw = message.raw_payload;
              raw.resize(raw.find_last_not_of(" \r\n\t"));
              raw += ",\"challenge_id\":\"" + admission.challenge + "\"}";
              owner.history.request_bridge({options.robot_command_subject, "_INBOX.native." + admission.challenge,
                  "{\"command\":\"position_execution_admit\",\"robot\":\"" + json_escape(options.robot_cell) +
                  "\",\"position\":" + raw + "}"});
              continue;
            }
            MotionServerURDFFileIdentity identity;
            const int joint = message.position.axis;
            if (joint < 0 || joint >= 6 || !motion_server_urdf_file_identity(options.rt_core.urdf, &identity) ||
                !motion_server_urdf_file_identity_equal(identity, kinematics.file_identity)) {
              halt("rt_core_position_axis_or_urdf_invalid");
            } else {
              const double target = message.position.target_rad + (message.position.relative ? backend.positions()[joint] : 0);
              if ((message.position.min_rad != 0 || message.position.max_rad != 0) &&
                  (target < std::min(message.position.min_rad, message.position.max_rad) ||
                   target > std::max(message.position.min_rad, message.position.max_rad))) {
                halt("rt_core_position_request_limit");
              } else {
                position_deadline = std::min(run_deadline, admission.sent_ns + std::uint64_t(message.position.timeout.count()) * 1000000);
                accepted = backend.position(joint, target, message.position.max_speed_rad_s, position_deadline,
                    [&] { return message.planning_generation == planning_generation.load() &&
                        authority.authorizes(message.sender_id, credential, std::chrono::steady_clock::now()); });
                if (accepted) { position_message = message; position_pending = true; }
              }
            }
          } else if (owns && message.kind == "arm") {
            if (home_receipt.pending) halt("home_preempted_by_arm");
            bool armed = true;
            bool present = false;
            std::string raw;
            if (extract_unique_top_level_json_member(message.raw_payload, "armed", &raw, &present) &&
                (!present || parse_exact_json_bool_value(raw, &armed)))
              accepted = armed ? backend.arm([&, generation = message.planning_generation] {
                return generation == planning_generation.load();
              }) : halt("operator_disarm");
          } else if (owns && (message.kind == "home" || message.kind == "go_home")) {
            if (position_pending || admission.pending || admitted_position) {
              owner.terminal(message, "failed", "rt_core_home_requires_idle_run");
              continue;
            }
            owner.cancel_queued(false);
            home_receipt.cancel(owner, "home_replaced");
            home_receipt.message = message;
            home_receipt.before = backend.status();
            home_receipt.mask = 0;
            for (const auto& axis : backend.description().Description.Axes)
              if (std::find(options.rt_core.joint_ids.begin(), options.rt_core.joint_ids.end(), axis.ID) != options.rt_core.joint_ids.end())
                home_receipt.mask |= 1U << axis.Index;
            accepted = backend.home([&, generation = message.planning_generation] {
              return generation == planning_generation.load();
            });
            // Home acquired fresh authority; this is its pre-Home snapshot.
            if (accepted) home_receipt.before = backend.status();
            home_receipt.pending = accepted;
            home_receipt.native = rt_core_native_receipt(backend, message, backend.status().Core.HomeEpoch);
            if (accepted) owner.history.acknowledge_command(robot_command_deduplication_key(message),
                robot_command_reply_payload(message.kind, message.command_id_json, true, false, "accepted", ""));
          } else {
            halt("rt_core_command_unavailable_or_unowned");
          }
        }
        if (!(accepted && (message.kind == "home" || message.kind == "go_home" || message.kind == "position")))
          owner.terminal(message, accepted ? "completed" :
              backend.reason() == "rt_core_execution_cancelled" ? "cancelled" : "failed",
              accepted ? "" : backend.reason(), rt_core_native_receipt(backend, message));
        live.latest_processed_cold_command_id = message.command_id;
        live.latest_processed_cold_command_kind = message.kind;
        live.latest_processed_cold_accepted = accepted;
        live.latest_processed_cold_result = accepted ? "applied" : backend.reason();
      }
      pollfd input{fd, POLLIN, 0};
      const int polled = ::poll(&input, 1, cartesian_rt::poll_ms);
      if (polled < 0 && errno != EINTR) throw std::runtime_error("rt_core_input_poll_failed");
      if (polled > 0 && (input.revents & POLLIN)) {
        std::array<unsigned char, kMaxJoystickIntentPacketBytes + 1> bytes{};
        sockaddr_storage peer{};
        socklen_t peer_length = sizeof(peer);
        SteamDeckJoystickIntent intent;
        bool decoded = false, safety_barrier = false, drained = false;
        std::uint64_t captured = 0;
        // Unverified drain cap: 64 datagrams. An exhausted cap inhibits input;
        // queued motion is never replayed one packet per control iteration.
        for (unsigned count = 0; count < 64; ++count) {
          UdpReceiveMetadata metadata;
          peer_length = sizeof(peer);
          const auto received = recvfrom_with_metadata(fd, bytes.data(), bytes.size(), &peer, &peer_length, &metadata);
          if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { drained = true; break; }
          if (received < 0) { safety_barrier = true; break; }
          ++live.datagrams_received;
          captured = api::host_monotonic_ns();
          // Queue residence may only shorten the daemon's source lifetime.
          const auto queue_ns = std::uint64_t(metadata.kernel_queue_us) * 1000;
          if (!metadata.kernel_timestamp_available || queue_ns >= rosie::protocol::fast_control_max_jog_input_age_ns || queue_ns >= captured)
            safety_barrier = true;
          else captured -= queue_ns;
          decoded = decode_steamdeck_joystick_intent(bytes.data(), received, &intent);
          if (!decoded || !intent.deadman || joystick_intent_is_neutral_hold(intent) ||
              (intent.frame == JOYSTICK_INTENT_FRAME_ARM && intent.tool_tcp[0] <= 0.5)) safety_barrier = true;
        }
        bool accepted = false;
        if (!drained || safety_barrier) {
          accepted = halt("input_safety_barrier");
        } else if (decoded) {
          live.latest_sequence = intent.sequence;
          const bool disarm = intent.frame == JOYSTICK_INTENT_FRAME_ARM && intent.tool_tcp[0] <= 0.5;
          if (!intent.deadman || disarm || joystick_intent_is_neutral_hold(intent)) {
            accepted = halt(disarm ? "operator_disarm" : "deadman_released");
          } else if (authority.authorizes_hot(sender_id_from_intent(intent), intent.leader_lease_id,
                                               intent.leader_fence_epoch, std::chrono::steady_clock::now()) &&
                     (hot_lease != intent.leader_lease_id || intent.sequence > hot_sequence)) {
            hot_lease = intent.leader_lease_id; hot_sequence = intent.sequence;
            if (intent.frame == JOYSTICK_INTENT_FRAME_ARM) {
              if (home_receipt.pending) halt("home_preempted_by_arm");
              accepted = backend.arm();
            } else if (intent.frame == JOYSTICK_INTENT_FRAME_HOME) {
              halt("home_replaced_by_udp");
              accepted = backend.home();
            } else if (intent.frame == JOYSTICK_INTENT_FRAME_BASE || intent.frame == JOYSTICK_INTENT_FRAME_TOOL) {
              try {
                MotionServerURDFFileIdentity identity;
                if (!motion_server_urdf_file_identity(options.rt_core.urdf, &identity) ||
                    !motion_server_urdf_file_identity_equal(identity, kinematics.file_identity))
                  throw std::runtime_error("rt_core_urdf_changed");
                if (backend.poll(captured) && backend.armed()) {
                  ScopedAsyncPlanningKinematics scoped(kinematics);
                  CartesianCommand command;
                  if (!joystick_intent_to_cartesian_command(intent, &command)) throw std::runtime_error("cartesian_input_invalid");
                  const auto q = backend.positions();
                  std::copy(q.begin(), q.end(), command.joint_position_rad);
                  const auto smoothed = smooth_cartesian_command(command, &smoother, options.control_frequency_hz);
                  const auto resolved = resolve_cartesian_velocity(smoothed, &solver, &live);
                  std::array<double, 6> velocity{};
                  std::copy_n(resolved.qd, velocity.size(), velocity.begin());
                  accepted = backend.jog(velocity, captured);
                  if (accepted) ++live.rtcore_outputs_sent;
                }
              } catch (const std::exception& error) { halt(error.what()); }
            } else {
              halt("rt_core_command_unavailable");
            }
          } else {
            halt("leader_fence_or_sequence_rejected");
          }
        } else {
          halt("invalid_cartesian_input");
        }
        if (!accepted) ++live.rejected_input_count;
        live.latest_command_kind = accepted ? "rt_core_intent_applied" : backend.reason();
      }
      publish();
    }
    halt("shutdown");
    backend.shutdown();
    publish();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "rt_core cartesiand: " << error.what() << "\n";
    return 1;
  }
}
