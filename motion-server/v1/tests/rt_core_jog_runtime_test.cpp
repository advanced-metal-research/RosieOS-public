#include "rt_core_jog_backend.hpp"
#include <iostream>

using namespace cartesian_rt;

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

struct FakeClient final : Client {
  std::vector<std::string> calls;
  api::Description description;
  api::Grant current;
  api::ProcessStatus observed;
  api::JogIngressObservation admission;
  std::vector<double> sent;
  std::function<void(const std::string&)> after_call;
  std::string fail;
  std::uint32_t fail_reason = 2;
  bool send_ok = true;
  bool observe_updates = true;
  std::uint64_t sent_origin = 0, sent_deadline = 0;
  std::uint64_t begin_origin = 0;
  std::uint64_t source_sequence = 0;
  Configuration config;
  std::thread::id writer = std::this_thread::get_id();
  bool reuse_session = false;
  bool wrong_binding = false;
  bool consume_trajectory = false;
  bool replay_event = false, repeat_cursor = false;
  std::string corrupt_receipt;
  api::Event terminal_event;
  std::uint64_t execution_counter = 0, event_sequence = 0;
  bool stopped_home_rejection = false, pending_home_rejection = false;
  std::vector<api::Point> points;
  FakeClient() {
    config.binding = {"test-pair", 3, "config-hash", ""};
    description.Description.ConfigurationSHA256 = "config-hash";
    description.Description.Backend = "simulation";
    observed.DaemonIncarnation = "daemon-1";
    observed.AdapterIncarnation = "adapter-1";
    observed.Core.ConfigurationSHA256 = "config-hash";
    observed.Core.ConfigVerifiedMask = 63;
    observed.Core.LeaseValid = 1;
    for (unsigned i = 0; i < 6; ++i) {
      const auto id = "J" + std::to_string(i + 1);
      config.joint_ids.push_back(id);
      api::AxisDescription axis;
      axis.Index = i; axis.ID = id; axis.PositionUnit = "rad"; axis.MaxVelocity = 0.1;
      axis.RequireHome = true;
      axis.MinPosition = -2; axis.MaxPosition = 2; axis.MaxAcceleration = 1;
      description.Description.Axes.push_back(axis);
      api::AxisStatus status;
      status.LogicalValid = true; status.HomeValid = true; status.Readiness = "ready"; status.PDOFresh = 1;
      observed.Core.Axes.push_back(status);
    }
    observed.Grant.emplace();
    observed.Grant->GrantActive = 1;
    observed.Jog.emplace();
  }
  void call(const std::string& name) {
    expect(std::this_thread::get_id() == writer, "native calls escaped the execution owner");
    calls.push_back(name);
    if (after_call) after_call(name);
    if (fail == name) {
      api::Response refusal; refusal.Error = "native_rejected";
      refusal.NativeResult.emplace(); refusal.NativeResult->Reason = fail_reason;
      throw api::Rejected(refusal);
    }
  }
  api::Description describe() override { call("describe"); return description; }
  api::Grant acquire(const api::Binding& binding) override {
    call("acquire"); expect(binding.PairID == "test-pair" && binding.Revision == 3, "configured binding lost");
    ++current.Generation;
    current.Session = reuse_session ? "session-1" : "session-" + std::to_string(current.Generation);
    current.Binding = binding;
    if (wrong_binding) ++current.Binding.Revision;
    observed.Grant->AppGrantGeneration = current.Generation;
    observed.Grant->GrantActive = 1;
    return current;
  }
  api::Grant grant() override { return current; }
  void start_renewal() override { call("start_renewal"); }
  void check_renewal() override { call("renewal"); }
  void stop_renewal() override { call("stop_renewal"); }
  api::ProcessStatus status() override { call("status"); observed.Core.TimeNS = api::host_monotonic_ns(); return observed; }
  api::JogIngressObservation ingress() override { call("ingress"); return admission; }
  void enable(std::uint32_t mask, const cartesian_rt::Authorization& = {}) override { call("enable"); expect(mask == 63, "wrong axis mask"); observed.Core.AxisEnableMask = mask; }
  void arm(const cartesian_rt::Authorization& = {}) override { call("arm"); observed.Core.Armed = 1; }
  void home(std::uint32_t mask, const cartesian_rt::Authorization& = {}) override { call("home"); expect(mask == 63, "wrong Home mask"); }
  bool home_receipt() override {
    if (pending_home_rejection) {
      pending_home_rejection = false;
      api::Response response; response.Error = "native_rejected";
      response.NativeResult.emplace(); response.NativeResult->Reason = 2; response.NativeResult->Operation = 264;
      throw api::Rejected(response);
    }
    return true;
  }
  void begin(std::uint32_t mask, std::uint64_t captured, std::uint64_t deadline) override {
    call("begin"); expect(mask == 63 && deadline > api::host_monotonic_ns(), "wrong Begin");
    begin_origin = captured;
    observed.Jog->Open = 1; ++observed.Jog->JogGeneration;
    source_sequence = 0;
  }
  bool update(const std::vector<double>& velocity, std::uint64_t captured, std::uint64_t deadline) override {
    call("update"); sent = velocity;
    sent_origin = captured; sent_deadline = deadline;
    expect(deadline > api::host_monotonic_ns(), "stale Update");
    ++source_sequence;
    if (observe_updates) {
      observed.Jog->ObservedSourceSequence = source_sequence;
      observed.Jog->ObservedOriginHostNs = captured;
      observed.Jog->FirstObservedHostNs = api::host_monotonic_ns();
      observed.Jog->UpdateReason = 0;
    }
    return send_ok;
  }
  void end() override { call("end"); observed.Jog->Open = 0; }
  void stop() override {
    call("stop"); ++current.Generation; observed.Core.Armed = 0;
    pending_home_rejection = stopped_home_rejection;
  }
  void release() override { call("release"); }
  api::Response prepare(std::uint32_t mask, const std::vector<api::Point>& p, const api::Identity& identity, const Authorization& = {}) override {
    call("prepare"); expect(mask == 63, "wrong trajectory mask"); points = p;
    observed.Execution.Handles = {{42, "prepared", current.Generation, 0, 0, identity}};
    api::Response r; r.Handle = 42;
    auto echo = observed.Execution.Handles[0];
    if (corrupt_receipt == "prepare") echo.Identity.ProgramID = "old-command";
    r.Data = api::to_json(echo); return r;
  }
  api::Response start_trajectory(std::uint64_t handle, const api::Identity& identity, const Authorization& = {}) override {
    call("start_trajectory"); expect(handle == 42, "wrong trajectory handle");
    observed.Motion.TrajectoryID = 42; observed.Motion.CommandSequence = 17;
    auto& h = observed.Execution.Handles[0];
    expect(api::encode(identity) == api::encode(h.Identity), "Start identity mismatch");
    h.State = "started"; h.NativeSequence = 17; h.ExecutionGeneration = ++execution_counter;
    api::Response r; r.Sequence = 17;
    auto echo = h;
    if (corrupt_receipt == "start") echo.Identity.ProgramID = "old-command";
    if (corrupt_receipt == "generation") echo.ExecutionGeneration = 0;
    r.Data = api::to_json(echo); return r;
  }
  api::EventBatch events(std::uint64_t after) override {
    call("events"); api::EventBatch batch; batch.AdapterIncarnation = "adapter-1";
    batch.NextSequence = after; batch.LatestSequence = event_sequence;
    if (consume_trajectory) {
      if (!terminal_event.Sequence) {
        const auto& h = observed.Execution.Handles[0];
        terminal_event.Sequence = ++event_sequence;
        terminal_event.DaemonIncarnation = "daemon-1"; terminal_event.AdapterIncarnation = "adapter-1";
        terminal_event.Handle.emplace();
        terminal_event.Handle->Handle = h.Handle; terminal_event.Handle->NativeSequence = h.NativeSequence;
        terminal_event.Handle->To = "consumed"; terminal_event.Handle->Generation = h.Generation;
        terminal_event.Handle->ExecutionGeneration = h.ExecutionGeneration; terminal_event.Handle->Identity = h.Identity;
      }
      auto e = terminal_event;
      if (repeat_cursor) e.Sequence = after;
      else if (replay_event) e.Sequence = ++event_sequence;
      if (repeat_cursor || replay_event || e.Sequence > after) {
        batch.Events.push_back(e); batch.NextSequence = e.Sequence;
      }
      batch.LatestSequence = event_sequence;
    }
    return batch;
  }
  std::size_t count(const std::string& name) { return std::count(calls.begin(), calls.end(), name); }
};

void arm_jog(FakeClient& client, Backend& backend) {
  expect(backend.start(), "Acquire failed");
  expect(client.count("home") == 0 && client.count("arm") == 0, "startup energized motion");
  expect(backend.arm(), "Arm failed");
  expect(backend.jog({0.4, -0.2, 0, 0, 0, 0}, api::host_monotonic_ns()), "Jog failed");
}

void TestArmJogStopReleaseOrdering() {
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  expect(client.sent[0] > 0.099 && client.sent[0] <= 0.1 && client.sent[1] == -client.sent[0] / 2,
         "Describe cap must scale the entire vector");
  expect(client.count("begin") == 1 && client.count("update") == 1, "wrong publication counts");
  expect(backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "second update refused");
  expect(client.count("begin") == 1 && client.count("update") == 2, "jog generation replaced");
  client.calls.clear(); backend.shutdown();
  expect(client.calls == std::vector<std::string>({"end", "stop", "stop_renewal", "release"}), "shutdown ordering");
  backend.shutdown(); expect(client.count("release") == 1, "double release");
}

void TestRefusalsLatchBeforeAnotherFrame() {
  for (const auto& scenario : {"renewal", "ingress", "native", "external_stop", "fault", "readiness", "home", "missing_axis", "identity", "home_epoch", "pdo"}) {
    FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
    if (std::string(scenario) == "renewal") client.fail = "renewal";
    if (std::string(scenario) == "ingress") { client.admission.SourceSequence = 1; client.admission.Reason = 8; client.admission.NowHostNS = api::host_monotonic_ns(); }
    if (std::string(scenario) == "native") client.observed.Jog->UpdateReason = 7;
    if (std::string(scenario) == "external_stop") ++client.observed.Grant->AppGrantGeneration;
    if (std::string(scenario) == "fault") client.observed.Core.SafetyFaultMask = 1;
    if (std::string(scenario) == "readiness") client.observed.Core.Axes[3].Readiness.clear();
    if (std::string(scenario) == "home") client.observed.Core.Axes[3].HomeValid = false;
    if (std::string(scenario) == "missing_axis") client.observed.Core.Axes.pop_back();
    if (std::string(scenario) == "identity") client.observed.DaemonIncarnation = "daemon-2";
    if (std::string(scenario) == "home_epoch") ++client.observed.Core.HomeEpoch;
    if (std::string(scenario) == "pdo") client.observed.Core.Axes[2].PDOFresh = 0;
    client.calls.clear();
    expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), scenario);
    expect(!backend.armed() && !backend.jogging(), "refusal failed to latch");
    expect(client.count("end") == 1 && client.count("stop") == 1 && client.count("update") == 0, "refusal ordering");
    expect(std::find(client.calls.begin(), client.calls.end(), "end") < std::find(client.calls.begin(), client.calls.end(), "stop"), "Stop preceded End");
    client.fail.clear(); client.calls.clear();
    expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()) && client.calls.empty(), "new input rearmed after refusal");
  }
}

void TestCommandRefusalAndExplicitHome() {
  for (const auto& failure : {"enable", "arm", "begin", "update"}) {
    FakeClient client; Backend backend(client, client.config);
    expect(backend.start(), "start"); client.fail = failure;
    const bool armed = backend.arm();
    if (armed) expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "failed command accepted");
    expect(!backend.armed() && client.count("stop") == 2, "failed command did not Stop before Release and on refusal");
    expect(backend.reason() == "native_rejected" && !backend.refusal().object.empty(), "typed refusal lost");
    client.fail.clear();
  }
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  client.calls.clear(); expect(backend.home(), "explicit Home failed");
  expect(client.calls == std::vector<std::string>({"end", "stop", "stop_renewal", "release", "describe", "acquire", "start_renewal", "status", "enable", "home"}), "Home ordering");
  expect(!backend.armed(), "Home automatically armed");
}

void TestFreshnessAndUnobservedFrame() {
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  client.observed.Jog->ObservedSourceSequence = 0;
  expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "overwrote unobserved frame");
  expect(client.count("update") == 1, "unobserved publication repeated");
  expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns() - 250000000), "aged input accepted");
  expect(!backend.armed(), "aged input not latched");
}

void TestInvalidMappingAndEndFailure() {
  FakeClient bad; bad.config.joint_ids[1] = "J1";
  Backend rejected(bad, bad.config); expect(!rejected.start() && bad.count("acquire") == 0, "duplicate mapping acquired");
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  client.fail = "end"; client.calls.clear();
  expect(backend.halt("operator_disarm"), "Stop should fence a refused End");
  expect(client.calls == std::vector<std::string>({"end", "stop"}), "End refusal skipped Stop");
  expect(!backend.refusal().object.empty(), "End refusal was hidden");
  client.fail.clear();
  client.fail = "stop";
  expect(!backend.halt("operator_disarm") && !backend.stop_confirmed(), "failed Stop reported confirmed");
  expect(!backend.arm() && !backend.armed(), "unconfirmed Stop allowed Arm");
  client.fail.clear();
}

void TestResolverStallPreservesCaptureAndExpiry() {
  FakeClient client; Backend backend(client, client.config);
  expect(backend.start() && backend.arm(), "arm");
  // Inject 100,000,000 ns of input age at the resolver boundary.
  const auto captured = api::host_monotonic_ns() - 100000000;
  const auto resolved = std::array<double, 6>{0.01, 0, 0, 0, 0, 0};
  expect(backend.jog(resolved, captured), "resolved frame rejected");
  expect(client.begin_origin == captured && client.sent_origin == captured && client.sent_deadline == captured + 250000000,
         "resolver refreshed the input lifetime");
  expect(backend.poll(captured + 250000000) && !backend.armed() && !backend.jogging(), "expired velocity retained");
  expect(client.count("update") == 1 && client.count("end") == 1, "expiry publication counts");
  expect(backend.arm(), "rearm");
  expect(!backend.jog(resolved, api::host_monotonic_ns() - 250000000), "expired resolver output published");
  expect(client.count("update") == 1, "stale velocity sent");
}

void TestRearmAfterNativeRefusal() {
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  client.observed.Jog->UpdateReason = 7;
  expect(!backend.poll(api::host_monotonic_ns()) && !backend.armed(), "native refusal not latched");
  expect(backend.arm(), "operator rearm failed");
  client.observe_updates = false;
  expect(backend.jog({0.02, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "retained refusal rejected new first frame");
  expect(client.observed.Jog->JogGeneration == 2 && client.count("update") == 2, "new generation publication counts");
  expect(backend.poll(api::host_monotonic_ns()) && backend.armed(), "old update refusal latched in new generation");
  client.observed.Jog->ObservedSourceSequence = 1;
  client.observed.Jog->ObservedOriginHostNs = client.sent_origin;
  client.observed.Jog->FirstObservedHostNs = api::host_monotonic_ns();
  expect(!backend.poll(api::host_monotonic_ns()) && !backend.armed(), "current frame refusal ignored");
}

void TestRetainedSequenceCannotObserveNewFrame() {
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  const auto old_origin = client.sent_origin;
  const auto old_observation = client.observed.Jog->FirstObservedHostNs;
  expect(backend.halt("operator_stop") && backend.arm(), "rearm");
  client.observe_updates = false;
  expect(backend.jog({0.02, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "first frame");
  expect(client.observed.Jog->ObservedSourceSequence == 1 && client.observed.Jog->ObservedOriginHostNs == old_origin,
         "fixture lost retained observation");
  expect(!backend.jog({0.03, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "old origin acknowledged new frame");
  client.observed.Jog->ObservedOriginHostNs = client.sent_origin;
  client.observed.Jog->FirstObservedHostNs = old_observation;
  expect(!backend.jog({0.03, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "old observation time acknowledged new frame");
  client.observed.Jog->FirstObservedHostNs = api::host_monotonic_ns();
  client.observed.Jog->ObservedSourceSequence = 2;
  expect(!backend.jog({0.03, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "different sequence acknowledged new frame");
  expect(client.count("update") == 2, "unobserved first frame overwritten");
  client.observed.Jog->ObservedSourceSequence = 1;
  expect(backend.jog({0.03, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "matching current observation rejected");
  expect(client.count("update") == 3, "matching observation failed to permit next frame");
}

void TestRoundedCommonScaleStaysWithinDescribeCap() {
  FakeClient client; Backend backend(client, client.config);
  expect(backend.start() && backend.arm(), "arm");
  for (double velocity : {0.155, -0.155}) {
    expect(backend.jog({velocity, -velocity / 2, 0, 0, 0, 0}, api::host_monotonic_ns()), "rounded frame refused");
    expect(std::abs(client.sent[0]) > 0.099 && std::abs(client.sent[0]) <= 0.1, "rounded vector exceeded 0.1 rad/s");
    expect(client.sent[1] == -client.sent[0] / 2, "common scale changed vector direction");
    for (double output : client.sent) expect(std::isfinite(output) && std::abs(output) <= 0.1, "uncapped output published");
  }
  expect(client.count("update") == 2 && backend.armed(), "capped updates latched refusal");
}

void TestRejectedCandidatePreservesAppliedVelocityAndExpiry() {
  FakeClient client; Backend backend(client, client.config); arm_jog(client, backend);
  const auto published = backend.applied();
  const auto deadline = client.sent_deadline;
  client.observed.Jog->ObservedSourceSequence = 0;
  expect(!backend.jog({-0.02, 0.03, 0, 0, 0, 0}, api::host_monotonic_ns()), "unobserved candidate accepted");
  expect(client.count("update") == 1 && backend.applied() == published, "rejected candidate reported as applied");
  expect(backend.poll(deadline) && !backend.armed(), "rejected candidate extended published expiry");
  for (double velocity : backend.applied()) expect(velocity == 0, "Stop retained applied velocity");
}

void TestStartArmAlwaysStopsBeforeRelease() {
  FakeClient client; Backend backend(client, client.config);
  expect(backend.start(), "start"); client.calls.clear();
  expect(backend.arm(), "arm");
  expect(client.calls.size() >= 3 && client.calls[0] == "stop" &&
         client.calls[1] == "stop_renewal" && client.calls[2] == "release", "initial Arm released before Stop");
  client.calls.clear(); backend.shutdown();
  expect(client.calls == std::vector<std::string>({"stop", "stop_renewal", "release"}), "armed shutdown ordering");

  FakeClient idle; Backend idle_backend(idle, idle.config);
  expect(idle_backend.start(), "idle start"); idle.calls.clear();
  idle_backend.shutdown();
  expect(idle.calls == std::vector<std::string>({"stop", "stop_renewal", "release"}), "idle shutdown ordering");

  FakeClient failed; Backend failed_backend(failed, failed.config);
  expect(failed_backend.start(), "failed start"); failed.calls.clear(); failed.fail = "stop";
  expect(!failed_backend.arm() && !failed_backend.armed(), "unconfirmed initial Stop allowed Arm");
  expect(failed.count("stop") == 1 && failed.count("release") == 0 && failed.count("acquire") == 0,
         "unconfirmed Stop replaced authority");
  failed_backend.shutdown();
  expect(failed.count("stop") == 2 && failed.count("release") == 0, "shutdown released after unconfirmed Stop");
}

void TestRunMappingExpiryAndRetiredSession() {
  FakeClient client; Backend backend(client, client.config);
  expect(backend.start(), "start");
  // Unverified fixture lifetime: 10,000,000,000 ns; expiry is injected, never slept.
  const auto expiry = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.bind_run("run-1", expiry), "run acquire");
  const auto session = backend.session(); const auto generation = backend.generation();
  expect(backend.bind_run("run-1", expiry), "same run");
  expect(backend.session() == session && backend.generation() == generation && client.count("acquire") == 2, "run replaced its session");
  expect(!backend.bind_run("run-2", expiry), "second run reused live session");
  expect(backend.arm(), "run arm");
  expect(client.count("acquire") == 2, "run arm replaced session");
  client.calls.clear();
  expect(!backend.poll(expiry), "source expiry ignored");
  expect(client.calls == std::vector<std::string>({"stop", "stop_renewal", "release"}), "expiry failed Stop/join/Release ordering");
  expect(!backend.bind_run("run-1", expiry + 1), "retired run resurrected");
  client.reuse_session = true;
  expect(!backend.bind_run("run-2", expiry), "second run accepted retired native session");
}

void TestNativePositionSingleWriterAndCompletion() {
  FakeClient client; Backend backend(client, client.config);
  // Unverified fixture lifetime: 10,000,000,000 ns.
  const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.start() && backend.bind_run("run", deadline) && backend.arm(), "setup");
  expect(backend.position(0, 0.01, 0.05, deadline), "position");
  expect(client.count("prepare") == 1 && client.count("start_trajectory") == 1 && client.count("update") == 0, "extra native writer");
  expect(client.points.size() == 2 && client.points[0].Time == 0 && client.points[1].Time >= 300000000 &&
      client.points[1].Position[0] == 0.01 && client.points[1].Velocity[0] == 0, "Describe trajectory bounds");
  expect(!backend.jog({0.01, 0, 0, 0, 0, 0}, api::host_monotonic_ns()), "jog overlapped trajectory");
  client.observed.Motion.Done = true; client.observed.Motion.State = 4;
  expect(backend.poll(api::host_monotonic_ns()) && !backend.position_complete(), "status alone completed");
  client.consume_trajectory = true;
  expect(backend.position_complete(), "correlated completion absent");
  expect(backend.home(), "run Home");
  expect(client.count("arm") == 1 && client.count("home") == 1, "Home implicitly armed");
  expect(backend.run_key() == "run", "Home lost source run expiry");
}

void TestNativePositionCancellationBeforeStart() {
  FakeClient client; Backend backend(client, client.config);
  // Unverified fixture lifetime: 10,000,000,000 ns.
  const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.start() && backend.bind_run("run", deadline) && backend.arm(), "setup");
  bool authorized = true;
  client.after_call = [&](const std::string& name) { if (name == "prepare") authorized = false; };
  expect(!backend.position(0, 0.01, 0.05, deadline, [&] { return authorized; }), "cancelled preparation started");
  expect(client.count("prepare") == 1 && client.count("start_trajectory") == 0 && client.count("release") == 2, "cancelled preparation not fenced");
}

void TestNativeStepCancellation() {
  for (const auto operation : {"before_prepare", "enable", "home_enable"}) {
    FakeClient client; Backend backend(client, client.config);
    // Unverified fixture lifetime: 10,000,000,000 ns.
    const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
    expect(backend.start() && backend.bind_run("run", deadline), "setup");
    bool authorized = true;
    if (std::string(operation) == "before_prepare") {
      expect(backend.arm(), "arm"); client.calls.clear();
      expect(!backend.position(0, .01, .05, deadline, [] { return false; }), "cancelled position accepted");
      expect(client.count("prepare") == 0 && client.count("start_trajectory") == 0, "cancelled native preparation");
    } else {
      client.after_call = [&](const std::string& name) { if (name == "enable") authorized = false; };
      const bool result = std::string(operation) == "enable" ? backend.arm([&] { return authorized; }) : backend.home([&] { return authorized; });
      expect(!result && client.count("arm") == 0 && client.count("home") == 0, "native step after cancelled Enable");
    }
    expect(backend.reason() == "rt_core_execution_cancelled" && backend.stop_confirmed(), "cancellation lost confirmed fence");
    client.after_call = {};
  }
}

void TestNativePositionRefusalReasonsAndIdentity() {
  for (const auto operation : {"prepare", "start_trajectory"}) {
    for (const auto reason : {6U, 8U}) {
      FakeClient client; Backend backend(client, client.config);
      // Unverified fixture lifetime: 10,000,000,000 ns.
      const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
      expect(backend.start() && backend.bind_run("run", deadline) && backend.arm(), "setup");
      client.fail = operation; client.fail_reason = reason;
      expect(!backend.position(0, 0.01, 0.05, deadline), "native refusal lost");
      expect(backend.reason() == "native_rejected" && api::encode(backend.refusal()).find("\"Reason\":" + std::to_string(reason)) != std::string::npos,
          "native reason changed");
      expect(backend.run_key().empty() && client.count("update") == 0 && client.count("release") == 2, "refused run remained live");
    }
  }
  FakeClient client; Backend backend(client, client.config);
  // Unverified fixture lifetime: 10,000,000,000 ns.
  const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.start() && backend.bind_run("run", deadline) && backend.arm(), "setup");
  client.observed.AdapterIncarnation = "restarted";
  expect(!backend.position(0, 0.01, 0.05, deadline) && client.count("prepare") == 0 && client.count("start_trajectory") == 0,
      "identity loss reached native Start");
}

void TestHomeRefusalSurvivesNativeFence() {
  FakeClient client; Backend backend(client, client.config);
  expect(backend.start() && backend.home(), "Home setup");
  client.stopped_home_rejection = true;
  ++client.observed.Grant->AppGrantGeneration;
  expect(!backend.poll(api::host_monotonic_ns()), "native Home fence ignored");
  expect(backend.reason() == "native_rejected" && api::encode(backend.refusal()).find("\"Operation\":264") != std::string::npos,
      "native Home receipt hidden by fence observation");
}

void TestGrantEchoBindingMismatch() {
  FakeClient client; client.wrong_binding = true;
  Backend backend(client, client.config);
  expect(!backend.start() && backend.reason() == "rt_core_grant_binding_mismatch", "Acquire accepted another revision");
  expect(client.count("enable") == 0 && client.count("arm") == 0 && client.count("stop") == 1,
      "mismatched native grant energized axes");
}

void TestNativePositionStartsAtHeldTarget() {
  FakeClient client; Backend backend(client, client.config);
  for (auto& axis : client.description.Description.Axes) axis.CountsPerUnit = 1000;
  // Unverified fixture lifetime: 10,000,000,000 ns.
  const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.start() && backend.bind_run("held", deadline) && backend.arm(), "setup");
  client.observed.Core.Axes[0].LogicalPosition = 0;
  client.observed.Core.Axes[0].LogicalTarget = .01;
  client.observed.Core.Axes[1].LogicalPosition = -.02;
  client.observed.Core.Axes[1].LogicalTarget = -.01;
  expect(backend.position(0, .02, .05, deadline), "position");
  expect(std::llround(client.points[0].Position[0] * 1000) == 10, "first knot is not held at ten counts");
  expect(std::llround(client.points[0].Position[1] * 1000) == -10 &&
         std::llround(client.points[1].Position[1] * 1000) == -10, "unselected joint moved from held target");
  // Eleven counts including rounding at 50 counts/s require 330,000,000 ns.
  expect(client.points[1].Time >= 330000000 && client.points[1].Time <= 330000002, "duration did not use held displacement");
}

void TestNativePositionRefusesInvalidHeldTarget() {
  for (const auto scenario : {"nonfinite", "invalid", "outside"}) {
    FakeClient client; Backend backend(client, client.config);
    // Unverified fixture lifetime: 10,000,000,000 ns.
    const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
    expect(backend.start() && backend.bind_run("held", deadline) && backend.arm(), "setup");
    if (std::string(scenario) == "nonfinite") client.observed.Core.Axes[2].LogicalTarget = std::numeric_limits<double>::quiet_NaN();
    if (std::string(scenario) == "invalid") client.observed.Core.Axes[2].LogicalValid = false;
    if (std::string(scenario) == "outside") client.observed.Core.Axes[2].LogicalTarget = 3;
    expect(!backend.position(0, .02, .05, deadline), "invalid held target accepted");
    expect(client.count("prepare") == 0 && client.count("start_trajectory") == 0, "invalid held target reached native");
    expect(backend.reason() == "rt_core_held_target_unavailable", "held target refusal lost");
  }
}

void TestNativePositionRejectsRecycledTuple() {
  FakeClient client; Backend backend(client, client.config);
  // Unverified fixture lifetime: 10,000,000,000 ns.
  const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
  expect(backend.start() && backend.bind_run("old", deadline) && backend.arm() &&
         backend.position(0, .01, .05, deadline), "old setup");
  client.consume_trajectory = true;
  client.observed.Motion.Done = true; client.observed.Motion.State = 4;
  expect(backend.poll(api::host_monotonic_ns()) && backend.position_complete(), "old completion");
  const auto old_identity = api::encode(backend.trajectory_identity());
  client.replay_event = true;
  expect(backend.retire_run("stop") && backend.bind_run("new", deadline) && backend.arm() &&
         backend.position(0, .02, .05, deadline), "new setup");
  expect(backend.handle() == 42 && backend.native_sequence() == 17, "fixture did not recycle tuple");
  expect(api::encode(backend.trajectory_identity()) != old_identity && backend.execution_generation() == 2,
      "replacement reused immutable identity or execution generation");
  expect(backend.poll(api::host_monotonic_ns()) && !backend.position_complete(), "recycled tuple completed a different run");
}

void TestNativePositionRejectsReceiptIdentity() {
  for (const auto* corruption : {"prepare", "start", "generation"}) {
    FakeClient client; Backend backend(client, client.config);
    // Unverified fixture lifetime: 10,000,000,000 ns.
    const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
    expect(backend.start() && backend.bind_run("run", deadline) && backend.arm(), "setup");
    client.corrupt_receipt = corruption;
    expect(!backend.position(0, .01, .05, deadline) && backend.run_key().empty(), "wrong receipt authorized execution");
    if (std::string(corruption) == "prepare")
      expect(client.count("start_trajectory") == 0 && backend.reason() == "rt_core_prepared_identity_changed", "wrong prepare echo reached Start");
    else expect(backend.reason() == "rt_core_started_identity_changed", "wrong start echo was accepted");
  }
}

void TestNativePositionRejectsRestartAndCursorReplay() {
  for (const auto* scenario : {"adapter_restart", "daemon_restart", "cursor", "execution_generation", "grant_generation", "identity", "event_adapter"}) {
    FakeClient client; Backend backend(client, client.config);
    // Unverified fixture lifetime: 10,000,000,000 ns.
    const auto deadline = api::host_monotonic_ns() + 10000000000ULL;
    expect(backend.start() && backend.bind_run("run", deadline) && backend.arm() &&
        backend.position(0, .01, .05, deadline), "setup");
    client.consume_trajectory = true;
    client.observed.Motion.Done = true; client.observed.Motion.State = 4;
    client.events(0);
    const std::string kind = scenario;
    if (kind == "adapter_restart") client.observed.AdapterIncarnation = "adapter-2";
    if (kind == "daemon_restart") client.observed.DaemonIncarnation = "daemon-2";
    if (kind == "cursor") client.repeat_cursor = true;
    if (kind == "execution_generation") ++client.terminal_event.Handle->ExecutionGeneration;
    if (kind == "grant_generation") ++client.terminal_event.Handle->Generation;
    if (kind == "identity") client.terminal_event.Handle->Identity.ProgramID = "previous-command";
    if (kind == "event_adapter") client.terminal_event.AdapterIncarnation = "adapter-2";
    expect(!backend.position_complete(), kind + " completed current execution");
    if (kind == "cursor") expect(backend.reason() == "rt_core_event_cursor_replayed", "replayed cursor not classified");
    if (kind == "adapter_restart" || kind == "daemon_restart")
      expect(backend.reason() == "rt_core_identity_changed", "restart not classified");
  }
}

int main() {
  try {
    TestArmJogStopReleaseOrdering();
    TestRefusalsLatchBeforeAnotherFrame();
    TestCommandRefusalAndExplicitHome();
    TestFreshnessAndUnobservedFrame();
    TestInvalidMappingAndEndFailure();
    TestResolverStallPreservesCaptureAndExpiry();
    TestRearmAfterNativeRefusal();
    TestRetainedSequenceCannotObserveNewFrame();
    TestRoundedCommonScaleStaysWithinDescribeCap();
    TestRejectedCandidatePreservesAppliedVelocityAndExpiry();
    TestStartArmAlwaysStopsBeforeRelease();
    TestRunMappingExpiryAndRetiredSession();
    TestNativePositionSingleWriterAndCompletion();
    TestNativePositionCancellationBeforeStart();
    TestNativeStepCancellation();
    TestNativePositionRefusalReasonsAndIdentity();
    TestHomeRefusalSurvivesNativeFence();
    TestGrantEchoBindingMismatch();
    TestNativePositionStartsAtHeldTarget();
    TestNativePositionRefusesInvalidHeldTarget();
    TestNativePositionRejectsRecycledTuple();
    TestNativePositionRejectsReceiptIdentity();
    TestNativePositionRejectsRestartAndCursorReplay();
    std::cout << "rt_core_jog_runtime_test: 23 tests PASS\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
