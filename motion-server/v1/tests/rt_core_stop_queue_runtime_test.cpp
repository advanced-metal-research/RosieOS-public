#define main cartesiand_main
#include "robot_v4_cartesian_daemon.cpp"
#undef main
#include <cassert>

namespace queue_api = rosie::rt_control;

struct StopClient final : cartesian_rt::Client {
  cartesian_rt::Configuration config;
  unsigned stops = 0;
  bool native_armed = false;
  StopClient() {
    config.binding = {"queue-test", 1, "config-hash", ""};
    config.joint_ids = {"J1", "J2", "J3", "J4", "J5", "J6"};
  }
  queue_api::Description describe() override {
    queue_api::Description d;
    d.Description.ConfigurationSHA256 = "config-hash";
    for (unsigned i = 0; i < 6; ++i) {
      queue_api::AxisDescription a;
      a.Index = i; a.ID = config.joint_ids[i]; a.PositionUnit = "rad"; a.MaxVelocity = 0.1;
      d.Description.Axes.push_back(a);
    }
    return d;
  }
  queue_api::Grant grant() override { queue_api::Grant g; g.Generation = 1; g.Session = "queue-session"; g.Binding = config.binding; return g; }
  queue_api::Grant acquire(const queue_api::Binding&) override { return grant(); }
  void start_renewal() override {}
  void check_renewal() override {}
  void stop_renewal() override {}
  queue_api::ProcessStatus status() override {
    queue_api::ProcessStatus s;
    s.DaemonIncarnation = "daemon"; s.AdapterIncarnation = "adapter";
    s.Core.TimeNS = queue_api::host_monotonic_ns(); s.Core.ConfigurationSHA256 = "config-hash";
    s.Grant.emplace(); s.Grant->GrantActive = 1; s.Grant->AppGrantGeneration = 1;
    return s;
  }
  queue_api::JogIngressObservation ingress() override { return {}; }
  void enable(std::uint32_t, const cartesian_rt::Authorization& = {}) override {}
  void arm(const cartesian_rt::Authorization& = {}) override { native_armed = true; }
  void home(std::uint32_t, const cartesian_rt::Authorization& = {}) override {}
  void begin(std::uint32_t, std::uint64_t, std::uint64_t) override {}
  bool update(const std::vector<double>&, std::uint64_t, std::uint64_t) override { return false; }
  void end() override {}
  void stop() override { ++stops; native_armed = false; }
  void release() override {}
  queue_api::Response prepare(std::uint32_t, const std::vector<queue_api::Point>&, const queue_api::Identity&, const cartesian_rt::Authorization& = {}) override { assert(false); return {}; }
  queue_api::Response start_trajectory(std::uint64_t, const queue_api::Identity&, const cartesian_rt::Authorization& = {}) override { assert(false); return {}; }
  queue_api::EventBatch events(std::uint64_t) override { assert(false); return {}; }
};

// Exercise the subscriber's normalization and production queue together;
// fail-safe requests must reach the runtime's unconditional Stop branch.
void TestSaturatedDisarmStopPath() {
  for (const auto& kind : {"disarm", "arm"}) {
    StopClient client;
    cartesian_rt::Backend backend(client, client.config);
    assert(backend.start() && backend.arm() && client.native_armed);
    client.stops = 0;
    LeaderAwareRobotCommandQueue queue;
    RobotCommandMessage ordinary;
    ordinary.kind = "home";
    for (unsigned i = 0; i < 48; ++i) assert(queue.push(ordinary));
    assert(!queue.push(ordinary));
    RobotCommandMessage message;
    message.kind = kind;
    message.raw_payload = R"({"armed":false})";
    normalize_rt_core_stop(message);
    assert(message.kind == "stop" && queue.push(message));
    for (unsigned i = 0; i < 48; ++i) {
      assert(queue.pop(&message) && message.kind == "home");
    }
    assert(queue.pop(&message) && message.kind == "stop");
    assert(backend.halt("operator_stop") && backend.stop_confirmed());
    assert(client.stops == 1 && !client.native_armed && !backend.armed());
    assert(queue.empty());
  }
  for (const auto& payload : {R"({})", R"({"armed":true})", R"({"armed":0})",
                              R"({"armed":"false"})", R"({"armed":false,"armed":false})"}) {
    RobotCommandMessage message;
    message.kind = "arm";
    message.raw_payload = payload;
    normalize_rt_core_stop(message);
    assert(message.kind == "arm");
  }
}

// The selected owner must purge native Arm together with main's position/Home,
// retain typed cancellation receipts, and never change a terminal on replay.
void TestSharedOwnerCancellationAndTypedReplay() {
  Options options;
  options.backend = "rt_core";
  MotionServerExecutionOwner owner(options);
  for (const auto& id : {std::string("9007199254740993"), std::string("\"9007199254740993\"")}) {
    RobotCommandMessage message;
    const auto payload = std::string(R"({"schema":"robot.v4.robot-command.v1","command":"home","robot":"other-relay","sender_id":"steamdeck:test","command_id":)") + id + "}";
    assert(parse_robot_command_message(payload, "other-relay", &message));
    message.reply_subject = "_INBOX.test";
    assert(owner.commands.push(message));
  }
  RobotCommandMessage arm;
  arm.kind = "arm";
  assert(owner.commands.push(arm));
  owner.cancel_queued();
  assert(owner.commands.empty() && owner.generation.load() == 2);
  const auto replies = owner.history.pending_replies();
  assert(replies.size() == 2);
  for (const auto& reply : replies) {
    assert(reply.body.find("\"state\":\"cancelled\"") != std::string::npos);
    owner.history.complete_position(reply.key, "incorrect replacement");
    assert(owner.history.position_result(reply.key) == reply.body);
  }
  assert(owner.history.position_result("number:9007199254740993").find("\"command_id\":9007199254740993,") != std::string::npos);
  assert(owner.history.position_result("string:9007199254740993").find("\"command_id\":\"9007199254740993\",") != std::string::npos);
}

void TestNativeHomeTerminalRequiresObservation() {
  Options options;
  MotionServerExecutionOwner owner(options);
  RTCoreHomeReceipt receipt;
  receipt.message.kind = "home";
  receipt.message.command_id = "17";
  receipt.message.command_id_json = "17";
  receipt.message.reply_subject = "_INBOX.home";
  receipt.before.DaemonIncarnation = "daemon";
  receipt.before.AdapterIncarnation = "adapter";
  receipt.before.Core.TimeNS = 100;
  receipt.before.Core.HomeEpoch = 7;
  receipt.before.Grant.emplace();
  receipt.before.Grant->GrantActive = 1;
  receipt.before.Grant->AppGrantGeneration = 5;
  receipt.mask = 63;
  receipt.pending = true;
  auto status = receipt.before;
  receipt.observe(owner, status);
  assert(receipt.pending && owner.history.pending_replies().empty());
  status.Core.TimeNS = 101;
  status.Core.HomeEpoch = 8;
  status.Core.HomeValidMask = 63;
  status.Core.NativeHomeActiveMask = 1;
  receipt.observe(owner, status);
  assert(receipt.pending && owner.history.pending_replies().empty());
  status.Core.NativeHomeActiveMask = 0;
  status.Core.CommissioningPhase = 1;
  receipt.observe(owner, status);
  assert(receipt.pending);
  status.Core.CommissioningPhase = 0;
  receipt.observe(owner, status);
  assert(!receipt.pending && owner.history.pending_replies().size() == 1);
  assert(owner.history.position_result("number:17").find("\"state\":\"completed\"") != std::string::npos);
  receipt.message.command_id = "18";
  receipt.message.command_id_json = "18";
  receipt.pending = true;
  receipt.cancel(owner, "operator_stop");
  receipt.observe(owner, status);
  assert(owner.history.position_result("number:18").find("\"state\":\"cancelled\"") != std::string::npos);
  receipt.message.command_id = "19";
  receipt.message.command_id_json = "19";
  receipt.pending = true;
  status.Grant->AppGrantGeneration = 6;
  assert(!receipt.observe(owner, status) && !receipt.pending);
  assert(owner.history.position_result("number:19").find("\"state\":\"failed\"") != std::string::npos);
}

// Exact source identity and send-time projection prevent replayed permits or
// transport delay from extending a run's grant lifetime.
void TestNativeRunPermitSourceClock() {
  RTCoreRunAdmission admission;
  admission.pending = true;
  admission.sent_ns = 1000000000;
  admission.challenge = "challenge";
  admission.message.command_id_json = "17";
  admission.message.raw_payload = R"({"controller_boot_id":"boot","leader_lease_id":"lease","leader_fence_epoch":4})";
  auto& run = admission.message.bridge_run;
  run = {"bridge", "run", "steamdeck:test", "boot", 150, 200};
  BridgePositionPermit permit;
  permit.challenge_id = "challenge"; permit.bridge_boot_id = "bridge";
  permit.run_id = "run"; permit.controller_id = "steamdeck:test"; permit.controller_boot_id = "boot";
  permit.command_id_json = "17"; permit.leader_lease_id = "lease"; permit.leader_fence_epoch = 4;
  permit.bridge_observed_monotonic_ms = 100; permit.deadline_monotonic_ms = 150;
  permit.remaining_ms = 50; permit.run_deadline_monotonic_ms = 200; permit.run_remaining_ms = 100;
  std::uint64_t deadline = 0;
  assert(admission.verify(permit, 1040000000, &deadline) && deadline == 1100000000);
  assert(!admission.verify(permit, 1050000000, &deadline));
  permit.run_remaining_ms = 101;
  assert(!admission.verify(permit, 1040000000, &deadline));
  permit.run_remaining_ms = 100; permit.command_id_json = "\"17\"";
  assert(!admission.verify(permit, 1040000000, &deadline));
  permit.command_id_json = "17"; permit.bridge_boot_id = "restarted";
  assert(!admission.verify(permit, 1040000000, &deadline));
}

void TestNativePositionRequiresExplicitJointTarget() {
  RobotCommandMessage message;
  for (const auto* input : {R"({})", R"({"tcp_pose":[0.2,0.1,0.3,0,0,0]})",
      R"({"axis":0})", R"({"target_rad":0})", R"({"axis":0,"target_rad":NaN})",
      R"({"axis":0,"target_rad":1e999})", R"({"joint_targets":[0,0]})",
      R"({"joint_targets":[0,0,0,0,0,"NaN"]})", R"({"axis":0,"target_rad":0,"target_deg":1})"}) {
    message.raw_payload = input;
    assert(!rt_core_position_input_resolved(message));
  }
  for (const auto* input : {R"({"axis":0,"target_rad":0})", R"({"axis":5,"target_deg":1})",
                            R"({"axis":2,"relative_jog_rad":-0.01})"}) {
    message.raw_payload = input;
    assert(rt_core_position_input_resolved(message));
  }
}

void TestStopAdmissionInvalidatesGeneration() {
  for (const auto* kind : {"stop", "disarm", "arm"}) {
    std::atomic<uint64_t> generation{7};
    RobotCommandMessage message;
    message.kind = kind; message.raw_payload = R"({"armed":false})";
    admit_rt_core_command(message, generation);
    assert(message.kind == "stop" && generation.load() == 8 && message.planning_generation == 8);
    LeaderAwareRobotCommandQueue queue;
    assert(queue.push(message) && generation.load() == 8);
  }
}

int main() {
  TestSaturatedDisarmStopPath();
  TestSharedOwnerCancellationAndTypedReplay();
  TestNativeHomeTerminalRequiresObservation();
  TestNativeRunPermitSourceClock();
  TestNativePositionRequiresExplicitJointTarget();
  TestStopAdmissionInvalidatesGeneration();
  std::cout << "rt_core_stop_queue_runtime_test: 6 tests PASS\n";
}
