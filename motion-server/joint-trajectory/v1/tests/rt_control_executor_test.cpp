#include <future>
#include <iostream>
#include "rt_control_executor.hpp"
#include "executor_status_json.hpp"

using namespace dense_joint_trajectory;

static void check(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
// Unverified synchronization ceiling: 2000 ms; gates, never sleep, establish ordering.
static constexpr auto gate_budget = std::chrono::milliseconds(2000);

struct FakeControl {
  std::mutex mutex;
  std::condition_variable changed;
  bool gate = false, reached = false, released = false, corrupt = false;
  bool expire = false;
  bool home_gate = false, home_started = false, home_observation = false;
  bool describe_gate = false, cleanup_unavailable = false;
  int acquisition_gate = 0, acquisition_read = 0;
  bool hang_renewal = false, renewal_pending = false;
  std::string hang_operation;
  std::vector<rtc::Deadline> shutdown_deadlines;
  int delayed_inhibition = 0, cleanup_samples = 0;
  std::optional<rtc::EventBatch> batch;
  std::string rejection;
  std::vector<std::string> operations;
  std::string uploaded;
  rtc::Grant grant;
  rtc::Program program;
  rtc::ProcessStatus state;
  rtc::Description description;
  FakeControl(const Plan& plan) {
    description.ContractVersion = rtc::ContractVersion;
    description.CapabilitiesDigest = rtc::CapabilitiesDigest;
    auto& d = description.Description;
    d.ConfigurationSHA256 = std::string(64, 'c');
    d.MachineSHA256 = std::string(64, 'a'); d.DeploymentSHA256 = std::string(64, 'b');
    d.Backend = "simulation"; d.MaxTrajectoryPoints = 250000; d.ResidentPlans = 3;
    for (unsigned i = 0; i < 9; ++i) {
      rtc::AxisDescription axis;
      axis.Index = i; axis.ID = "J" + std::to_string(i+1); axis.PositionUnit = "rad";
      axis.MinPosition = -1; axis.MaxPosition = 1; axis.MaxVelocity = .1; axis.NativeHome = true;
      d.Axes.push_back(axis);
      rtc::AxisStatus observed; observed.Readiness = "ready"; state.Core.Axes.push_back(observed);
    }
    for (auto name : {"describe", "acquire", "renew", "enable", "arm", "home", "restore_anchor",
         "recovery_status", "prepare_program", "start_program", "status", "subscribe_events", "stop", "release"}) {
      rtc::CapabilityInfo capability; capability.Name = name; capability.State = "implemented";
      description.Capabilities.push_back(capability);
    }
    state.AdapterIncarnation = "adapter"; state.DaemonIncarnation = "daemon";
    state.Grant = rtc::GrantObservation{};
    state.Core.ConfigurationSHA256 = d.ConfigurationSHA256; state.Core.HomeValidMask = 511;
    program.Identity = native_identity(plan); program.AxisMask = plan.header.axis_mask;
    program.Identity.NormalisedDigest = plan.header.trajectory_digest;
    program.Samples = program.NormalisedSamples = plan.header.total_sample_count;
    program.Segments = plan.segments.size();
    state.Execution.Identity = program.Identity;
  }
  std::string respond(const std::string& path, const std::string& body, rtc::Deadline end) {
    std::unique_lock<std::mutex> lock(mutex);
    rtc::Response response; response.Schema = rtc::ResponseSchema;
    if (path == "/v1/describe") {
      if (describe_gate) {
        reached = true; changed.notify_all();
        check(changed.wait_for(lock, gate_budget, [&] { return released; }), "Describe gate timeout");
      }
      response.Data = rtc::to_json(description);
    }
    else if (path == "/v1/status") {
      if (acquisition_gate != 0 && ++acquisition_read == acquisition_gate) {
        reached = true; changed.notify_all();
        check(changed.wait_for(lock, gate_budget, [&] { return released; }), "acquisition Status gate timeout");
      }
      if (home_gate && home_started && !home_observation) {
        home_observation = true; changed.notify_all();
        check(changed.wait_for(lock, gate_budget, [&] { return released; }), "Home status gate timeout");
        state.Core.HomeValidMask = 511;
      }
      if (grant.Session.empty()) {
        ++cleanup_samples;
        if (hang_operation == "status") {
          shutdown_deadlines.push_back(end);
          changed.wait_until(lock, end, [] { return false; });
          throw rtc::DeadlineExceeded("gated cleanup deadline");
        }
        if (delayed_inhibition > 0) {
          --delayed_inhibition;
          state.Core.Armed = 1;
        } else state.Core.Armed = 0;
        if (cleanup_unavailable) throw rtc::ProtocolError("status transport unavailable");
      }
      state.TimeNS = state.Core.TimeNS = state.Motion.TimeNS = std::uint64_t(monotonic_ms()) * 1000000;
      response.Data = rtc::to_json(state);
    } else if (path.find("/v1/events?") == 0) {
      if (acquisition_gate == 3) {
        reached = true; changed.notify_all();
        check(changed.wait_for(lock, gate_budget, [&] { return released; }), "acquisition Events gate timeout");
      }
      rtc::EventBatch result; result.AdapterIncarnation = state.AdapterIncarnation;
      result.NextSequence = result.LatestSequence = std::stoull(path.substr(path.find('=') + 1));
      if (batch) { result = *batch; batch.reset(); }
      response.Data = rtc::to_json(result);
    } else if (path == "/v1/program") {
      operations.push_back("prepare_program"); uploaded = body;
      auto result = program;
      if (corrupt) result.Identity.SourceDigest = "substituted";
      response.Data = rtc::to_json(result); state.Execution.State = "prepared";
      if (!rejection.empty()) response.Error = rejection;
    } else {
      rtc::Request request; check(rtc::decode(body, request), "request codec");
      const auto& op = request.Operation;
      response.Operation = op; response.Sequence = 1;
      operations.push_back(op);
      if (op == "stop" || op == "release") shutdown_deadlines.push_back(end);
      if ((hang_renewal && op == "renew") || op == hang_operation) {
        if (op == "renew") renewal_pending = true;
        changed.notify_all();
        changed.wait_until(lock, end, [] { return false; });
        throw rtc::DeadlineExceeded("gated control deadline");
      }
      if (op == "acquire") {
        grant.Session = "session"; ++grant.Generation; grant.LeaseMS = 500; grant.Binding = request.Binding;
      } else if (op == "renew") {
        if (expire) response.Error = "expired";
      } else if (op == "enable") state.Core.AxisEnableMask = request.AxisMask;
      else if (op == "home") home_started = true;
      else if (op == "arm") state.Core.Armed = 1;
      else if (op == "stop" || op == "release") {
        state.Core.Armed = 0; state.Core.AxisEnableMask = 0; state.Execution.State = "cancelled";
        ++grant.Generation;
        if (op == "release") grant.Session.clear();
      } else if (op == "recovery_status") {
        rtc::RecoveryStatus recovery; recovery.HomeValidMask = 511; response.Data = rtc::to_json(recovery);
      } else if (op == "start_program") {
        check(same_identity(request.Identity, program.Identity), "echo exact prepared identity");
        reached = true; changed.notify_all();
        if (gate) check(changed.wait_for(lock, gate_budget, [&] { return released; }), "Start gate timeout");
        state.Execution.State = "executing"; state.Execution.NativeSequence = 1; state.Execution.Generation = 1;
      }
      if (op == "acquire" || op == "renew" || op == "stop" || op == "release") {
        ++grant.DeadlineHostNS; response.Data = rtc::to_json(grant);
      }
      changed.notify_all();
    }
    const auto encoded = rtc::encode(response);
    return std::string("HTTP/1.1 ") + (response.Error.empty() ? "200 OK" : "409 Conflict") +
        "\r\nContent-Length: " + std::to_string(encoded.size()) + "\r\n\r\n" + encoded;
  }
};

// A public HttpStream transport double gates actual SDK Start writes. It retains
// the production client codec, request fencing and separate renewal connection.
class FakeStream final : public rtc::HttpStream {
 public:
  explicit FakeStream(FakeControl& fake) : fake_(fake) {}
  std::size_t write(const char* data, std::size_t size, rtc::Deadline end) override {
    request_.append(data, size);
    const auto header = request_.find("\r\n\r\n");
    if (header != std::string::npos) {
      auto length = request_.find("Content-Length: ");
      auto bytes = length == std::string::npos ? 0 : std::stoull(request_.substr(length + 16));
      if (request_.size() == header + 4 + bytes) {
        auto first = request_.find(' '), last = request_.find(' ', first+1);
        response_ = fake_.respond(request_.substr(first+1, last-first-1), request_.substr(header+4), end);
        request_.clear();
      }
    }
    return size;
  }
  std::size_t read(char* data, std::size_t size, rtc::Deadline) override {
    check(!response_.empty(), "response missing");
    auto bytes = std::min(size, response_.size());
    std::memcpy(data, response_.data(), bytes); response_.erase(0, bytes); return bytes;
  }
 private:
  FakeControl& fake_;
  std::string request_, response_;
};

struct Fixture {
  std::vector<unsigned char> bytes;
  Plan plan;
  LeaderAuthority leader{1};
  std::unique_ptr<FakeControl> fake;
  std::unique_ptr<RtControlExecutor> executor;
  std::string record;
  Fixture() {
    std::ifstream file(RT_CORE "/clients/cpp/examples/testdata/valid.rdt", std::ios::binary);
    check(bool(file), "valid fixture readable");
    bytes.assign(std::istreambuf_iterator<char>(file), {});
    check(decode(bytes.data(), bytes.size(), &plan).ok, "valid fixture decoded");
    fake = std::make_unique<FakeControl>(plan);
    DaemonConfig config;
    config.configuration_sha256 = std::string(64, 'c'); config.machine_sha256 = std::string(64, 'a');
    config.deployment_sha256 = std::string(64, 'b'); config.pair_id = "bench"; config.pair_revision = 7;
    executor = std::make_unique<RtControlExecutor>(config, &leader, [&] {
      return std::make_unique<rtc::RtControlClient>(rtc::StreamFactory([&](rtc::Deadline) {
        return std::make_unique<FakeStream>(*fake);
      }));
    });
    record = TEST_BUILD_DIR "/native-unit-" + std::to_string(::getpid());
    check(acquire().ok, "initial acquisition");
  }
  ~Fixture() { executor.reset(); ::unlink((record + ".rt-core.json").c_str()); }
  LeaseGrant acquire() {
    return leader.acquire("offline-programming:test-1", "boot", 2000, monotonic_ms(),
        [&] { executor->revoke_motion(); }, [&](const auto& snapshot) { return executor->leader_acquired(snapshot).reason; });
  }
  void preload() { check(executor->preload(plan, bytes, record).ok, "preload accepted"); }
  PlayIdentity identity() const {
    const auto& h = plan.header;
    return {h.plan_id, h.program_id, h.program_digest, h.trajectory_digest, h.manifest_revision,
            h.plan_revision, leader.snapshot().fence_epoch};
  }
};

// Revoke under the leader mutex must wait for an in-flight Start and cannot
// deadlock with status/tick; a stale play can never resurrect its preparation.
static void ordering(int mode) {
  Fixture f; f.preload(); f.fake->gate = true;
  auto play = std::async(std::launch::async, [&] { return f.executor->play(f.identity()); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->reached; }), "Start reached");
  }
  std::promise<void> entered;
  auto stopped = std::async(std::launch::async, [&] {
    if (mode == 1) {
      return f.leader.acquire("offline-programming:test-2", "next", 2000, monotonic_ms(), [&] {
        entered.set_value(); f.executor->revoke_motion();
      }).ok;
    }
    if (mode == 2) {
      const auto current = f.leader.snapshot();
      return f.leader.release("offline-programming:test-1", "boot", current.lease_id, current.fence_epoch,
          monotonic_ms(), [&] { entered.set_value(); f.executor->revoke_motion(); }).ok;
    }
    entered.set_value(); f.executor->stop("operator_stop"); return true;
  });
  check(entered.get_future().wait_for(gate_budget) == std::future_status::ready, "revoker entered");
  check(stopped.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready, "Stop must join Start");
  auto tick = std::async(std::launch::async, [&] { f.executor->tick(); return f.executor->status(); });
  { std::lock_guard<std::mutex> lock(f.fake->mutex); f.fake->released = true; f.fake->changed.notify_all(); }
  check(play.wait_for(gate_budget) == std::future_status::ready, "Start settled");
  const auto result = play.get();
  check(result.ok || result.reason == "native_operation_cancelled", "Start succeeds or is cancelled by Stop");
  check(stopped.wait_for(gate_budget) == std::future_status::ready && stopped.get(), "Stop returned");
  check(tick.wait_for(gate_budget) == std::future_status::ready, "no leader/executor inversion");
  check(!f.executor->play(f.identity()).ok, "stale preparation cannot restart");
  std::lock_guard<std::mutex> lock(f.fake->mutex);
  const auto& ops = f.fake->operations;
  check(std::count(ops.begin(), ops.end(), "start_program") == 1, "exactly one native Start");
  const auto start = std::find(ops.begin(), ops.end(), "start_program");
  const auto stop = std::find(start, ops.end(), "stop");
  check(stop != ops.end() && std::find(stop, ops.end(), "release") != ops.end(), "Start then Stop then release");
}

// Stop must reach the public control transport and return within 250 ms while
// the separate Home observation remains gated; elapsed time cannot pass the test.
static void stop_during_home(bool revoke) {
  Fixture f; f.preload();
  f.fake->state.Core.HomeValidMask = 0;
  f.fake->home_gate = true;
  auto play = std::async(std::launch::async, [&] { return f.executor->play(f.identity()); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->home_observation; }), "Home observation pending");
  }
  const auto issued = std::chrono::steady_clock::now();
  auto stopped = std::async(std::launch::async, [&] {
    if (revoke) return f.leader.acquire("offline-programming:test-2", "next", 2000, monotonic_ms(),
        [&] { f.executor->revoke_motion(); }).ok;
    f.executor->stop("operator_stop"); return true;
  });
  // Unverified fail-safe regression ceiling: 250 ms, independent of the observation budget.
  constexpr auto stop_bound = std::chrono::milliseconds(250);
  check(stopped.wait_for(stop_bound) == std::future_status::ready && stopped.get(), "Stop returns within 250 ms");
  check(std::chrono::steady_clock::now() - issued <= stop_bound, "absolute Stop bound 250 ms");
  check(play.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready, "observation remains pending after Stop");
  {
    std::lock_guard<std::mutex> lock(f.fake->mutex);
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "stop") == 1, "rt-core received Stop while observation pending");
    f.fake->released = true; f.fake->changed.notify_all();
  }
  check(play.get().reason == "native_operation_cancelled", "old observation cancelled");
  check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "start_program") == 0, "cancelled observation cannot Start");
}

static void acquisition_readers() {
  Fixture f;
  f.fake->describe_gate = true;
  auto acquired = std::async(std::launch::async, [&] { return f.acquire(); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->reached; }), "acquire Describe pending");
  }
  auto readers = std::async(std::launch::async, [&] {
    const auto snapshot = f.leader.snapshot();
    check(!snapshot.active, "leader inactive until acquisition commits");
    check(!f.leader.authorizes("offline-programming:test-1", {}, monotonic_ms()), "pending acquisition cannot authorize");
    check(!f.leader.renew("offline-programming:test-1", "boot", snapshot.lease_id, snapshot.fence_epoch, 2000, monotonic_ms()).ok,
          "pending acquisition cannot renew");
    return f.executor->status().native_state;
  });
  check(readers.wait_for(gate_budget) == std::future_status::ready && readers.get() == "acquiring", "status and leader readers run during acquisition I/O");
  { std::lock_guard<std::mutex> lock(f.fake->mutex); f.fake->released = true; f.fake->changed.notify_all(); }
  check(acquired.get().ok, "acquisition commits after I/O");
}

static void cancel_acquisition_observation() {
  Fixture f;
  f.fake->describe_gate = true;
  auto acquired = std::async(std::launch::async, [&] { return f.acquire(); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->reached; }), "Describe pending before cancel");
  }
  auto stop = std::async(std::launch::async, [&] { f.executor->stop("operator_stop"); });
  check(stop.wait_for(gate_budget) == std::future_status::ready, "Stop does not wait for Describe");
  stop.get();
  { std::lock_guard<std::mutex> lock(f.fake->mutex); f.fake->released = true; f.fake->changed.notify_all(); }
  check(!acquired.get().ok && !f.leader.snapshot().active, "cancelled acquisition cannot commit");
  check(f.executor->status().native_state == "inactive", "cancelled acquisition remains inactive");
}

// Request refusals leave both the captured leader epoch and the gated observation
// intact; only releasing Home may let the original preparation reach Start.
static void preload_during_home() {
  Fixture f; f.preload();
  f.fake->state.Core.HomeValidMask = 0;
  f.fake->home_gate = true;
  const auto lease = f.leader.snapshot();
  const auto grant = f.fake->grant;
  auto play = std::async(std::launch::async, [&] { return f.executor->play(f.identity()); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->home_observation; }), "Home pending before preload");
  }
  check(f.executor->status().commissioning_phase == "home", "Home phase published before observation completes");
  const auto refusal = f.executor->preload(f.plan, f.bytes, f.record);
  check(!refusal.ok && refusal.reason == "native_commissioning_in_progress", "preload carries typed request refusal");
  const auto current = f.leader.snapshot();
  check(current.active && current.lease_id == lease.lease_id && current.fence_epoch == lease.fence_epoch,
        "preload preserves leader lease and epoch");
  check(f.executor->status().native_state == "owned" && f.executor->status().commissioning_phase == "home",
        "preload preserves published observation state");
  check(play.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready, "Home remains gated");
  {
    std::lock_guard<std::mutex> lock(f.fake->mutex);
    check(f.fake->grant.Session == grant.Session && f.fake->grant.Generation == grant.Generation, "native grant unchanged");
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "stop") == 0, "preload sends no Stop");
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "release") == 0, "preload sends no Release");
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "prepare_program") == 1, "preload sends no replacement program");
    f.fake->released = true; f.fake->changed.notify_all();
  }
  check(play.get().ok, "original Home observation reaches Start");
  check(f.executor->status().commissioning_phase.empty(), "completed commissioning clears phase");
  check(f.executor->preload(f.plan, f.bytes, f.record).reason == "native_playback_in_progress", "playing preload refused");
  check(f.executor->status().state == "playing", "playing preload does not abort execution");
}

// Every unlocked acquisition window refuses ingest without cancelling the lease
// being established, including windows after native Acquire has succeeded.
static void preload_during_acquisition(int gate) {
  Fixture f;
  f.executor->stop("setup");
  f.fake->describe_gate = gate == 0;
  // Count Status observations across the observer and the SDK's own Describe;
  // resetting on each Describe hides the post-Acquire observation window.
  f.fake->acquisition_read = 0;
  f.fake->acquisition_gate = gate;
  auto acquired = std::async(std::launch::async, [&] { return f.acquire(); });
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->reached; }), "acquisition observation pending");
  }
  check(f.executor->preload(f.plan, f.bytes, f.record).reason == "native_acquisition_required", "acquisition preload typed refusal");
  check(f.executor->status().native_state == "acquiring", "preload preserves acquisition state");
  {
    std::lock_guard<std::mutex> lock(f.fake->mutex);
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "stop") == 1, "no extra Stop during acquisition");
    check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "release") == 1, "no extra Release during acquisition");
    f.fake->released = true; f.fake->changed.notify_all();
  }
  check(acquired.get().ok, "preload cannot cancel acquisition");
}

// Hung SDK renewal and shutdown commands share the shutdown entry's deadline;
// transport observations, elapsed bounds and typed uncertainty catch lock retention.
static void shutdown_budget(const std::string& operation) {
  Fixture f;
  {
    std::lock_guard<std::mutex> lock(f.fake->mutex);
    f.fake->hang_renewal = true;
    f.fake->hang_operation = operation;
  }
  {
    std::unique_lock<std::mutex> lock(f.fake->mutex);
    check(f.fake->changed.wait_for(lock, gate_budget, [&] { return f.fake->renewal_pending; }), "renewal transport pending");
  }
  const auto issued = std::chrono::steady_clock::now();
  if (operation == "release") {
    const auto lease = f.leader.snapshot();
    check(f.leader.release("offline-programming:test-1", "boot", lease.lease_id, lease.fence_epoch,
        monotonic_ms(), [&] { f.executor->revoke_motion(); }).ok, "leader release finishes uncertain shutdown");
    check(!f.leader.snapshot().active, "uncertain release leaves leader inactive");
  } else f.executor->stop("operator_stop");
  // Unverified scheduling-inclusive regression ceiling: 1250 ms for the 1000 ms I/O budget.
  constexpr auto return_bound = std::chrono::milliseconds(1250);
  check(std::chrono::steady_clock::now() - issued <= return_bound, "shutdown returns within 1250 ms");
  check(f.executor->status().native_state == "cleanup_uncertain", "deadline exhaustion reports uncertainty");
  {
    std::lock_guard<std::mutex> lock(f.fake->mutex);
    check(!f.fake->shutdown_deadlines.empty(), "Stop reached SDK transport");
    // Unverified absolute deadline ceiling: 1001 ms allows millisecond clock quantization.
    check(f.fake->shutdown_deadlines.front() <= issued + std::chrono::milliseconds(1001), "shutdown deadline includes renewal join");
    for (const auto end : f.fake->shutdown_deadlines)
      check(end == f.fake->shutdown_deadlines.front(), "Stop, Release and reconciliation use one deadline");
    f.fake->hang_renewal = false; f.fake->hang_operation.clear();
    // Model native lease expiry after an uncertain Stop/Release.
    f.fake->grant.Session.clear();
  }
  check(f.acquire().ok, "executor and transition locks released for fresh acquisition");
}

static void event_reconciliation(bool loss, bool completed) {
  Fixture f; f.preload(); check(f.executor->play(f.identity()).ok, "play before event");
  rtc::EventBatch batch; batch.AdapterIncarnation = "adapter";
  rtc::Event event; event.AdapterIncarnation = "adapter"; event.TimeNS = 1;
  event.Type = loss ? "events_dropped" : "publisher_overflow";
  event.Sequence = loss ? 10 : 1;
  if (loss) { rtc::EventsDropped dropped; dropped.FirstLostSequence = 1; dropped.LastLostSequence = 10; event.Dropped = dropped; }
  batch.Events.push_back(event); batch.NextSequence = batch.LatestSequence = event.Sequence;
  f.fake->batch = batch;
  if (completed) f.fake->state.Execution.State = "completed";
  f.executor->tick();
  check(f.executor->status().state == (completed ? "done" : "playing"), "event loss reconciles and publisher overflow does not abort");
  std::lock_guard<std::mutex> lock(f.fake->mutex);
  check(std::count(f.fake->operations.begin(), f.fake->operations.end(), "stop") == 0, "event does not Stop");
}

int main() {
  try {
    {
      Plan plan;
      Sample zero, first_end, second_end;
      first_end.time_s = .1; second_end.time_s = .2;
      plan.segments = {{"freespace", "first", {zero, first_end}},
                       {"freespace", "second", {zero, second_end}}};
      auto cursor = native_progress(plan, 1, false);
      check(cursor.segment == 1 && cursor.sample == 0 && cursor.seconds == 0, "shared boundary uses next local clock");
      cursor = native_progress(plan, 2, false);
      check(cursor.segment == 1 && cursor.sample == 1 && cursor.seconds == .2, "cursor reports sample seconds, not host timestamp");
      cursor = native_progress(plan, 0, true);
      check(cursor.segment == 1 && cursor.sample == 1 && cursor.seconds == .2, "completion pins final producer sample");
    }
    ordering(0); ordering(1); ordering(2);
    stop_during_home(false); stop_during_home(true);
    acquisition_readers();
    cancel_acquisition_observation();
    preload_during_home();
    for (int gate = 0; gate < 4; ++gate) preload_during_acquisition(gate);
    shutdown_budget("stop"); shutdown_budget("release"); shutdown_budget("status");
    event_reconciliation(false, false); event_reconciliation(true, false); event_reconciliation(true, true);
    {
      Fixture f; f.preload();
      f.fake->delayed_inhibition = 3;
      f.executor->stop("operator_stop");
      check(f.fake->cleanup_samples >= 5, "cleanup observes delayed inhibition and distinct standstill samples");
      check(f.executor->status().native_state == "inactive", "periodic inhibition reconciles");
      check(!f.executor->status().consumer_action.empty(), "Stop supplies consumer action");
      check(f.acquire().ok, "acquire after successful reconciliation");
      f.fake->cleanup_unavailable = true;
      f.executor->stop("operator_stop");
      check(f.executor->status().native_state == "cleanup_uncertain", "cleanup uncertainty is typed");
      check(!f.acquire().ok, "unreconciled acquisition fails closed");
      f.fake->cleanup_unavailable = false;
      f.fake->state.AdapterIncarnation = "restarted-adapter";
      f.fake->state.Grant->GrantActive = 1;
      check(!f.acquire().ok, "foreign active grant refuses acquisition");
      check(f.executor->status().native_state == "grant_active", "active foreign grant reported as typed state");
      f.fake->state.Grant->GrantActive = 0;
      check(f.acquire().ok, "fresh acquire reconciles without daemon restart");
    }
    {
      Fixture f;
      f.plan.segments[0].samples[1].torch_on = true;
      for (int attempt = 0; attempt < 4; ++attempt)
        check(f.executor->preload(f.plan, f.bytes, f.record).reason == "native_torch_unsupported", "torch refused locally");
      check(f.fake->uploaded.empty(), "torch refusals upload zero bytes");
      f.executor->stop("operator_stop");
      f.plan.segments[0].samples[1].torch_on = false;
      check(f.executor->preload(f.plan, f.bytes, f.record).reason == "native_acquisition_required", "missing session has daemon reason");
    }
    {
      Fixture f; f.preload();
      check(f.fake->uploaded == std::string(f.bytes.begin(), f.bytes.end()), "upload original bytes");
      check(f.executor->pause_command().reason == "pause_unsupported", "pause refused");
      const auto home = f.executor->go_home();
      check(!home.ok && home.reason == "go_home_unsupported", "native Go Home refuses without a legacy planner");
      const auto serialized = executor_status_to_json(f.executor->status());
      JsonValue status;
      JsonParser parser(serialized.data(), serialized.size());
      check(parser.parse(&status), "combined executor status parses");
      std::string backend;
      check(json_str(status.find("backend"), &backend) && backend == "rt_core", "status retains native backend");
      check(status.find("native_result") && status.find("program_identity") &&
            status.find("execution_generation") && status.find("native_sequence"),
            "main status renderer retains native execution identity and result");
      f.fake->corrupt = true;
      check(f.executor->preload(f.plan, f.bytes, f.record).reason == "native_program_identity_mismatch", "digest substitution refused");
      check(!f.executor->play(f.identity()).ok, "mismatch cannot play");
      check(!f.executor->status().torch_on, "native torch intent always false");
    }
    {
      Fixture f;
      f.fake->description.Description.Axes[0].NativeHome = false;
      check(f.acquire().ok, "acquire axis without native Home");
      f.fake->state.Core.HomeValidMask = 0;
      f.preload();
      check(f.executor->play(f.identity()).reason == "native_home_unavailable", "missing native Home uses daemon label");
    }
    {
      Fixture f;
      f.fake->program.Identity.NormalisedDigest = "sha256:" + std::string(64, 'd');
      f.fake->state.Execution.Identity = f.fake->program.Identity;
      f.preload();
      check(f.executor->play(f.identity()).ok, "adapter normalised digest is retained without recomputation");
    }
    {
      Fixture f; f.preload();
      auto stale = f.identity();
      check(f.acquire().ok, "replacement acquisition"); f.preload();
      check(f.executor->play(stale).reason == "leader_lease_expired", "previously authorized command cannot cross epoch");
    }
    {
      Fixture f; f.preload(); check(f.executor->play(f.identity()).ok, "play before expiry");
      { std::lock_guard<std::mutex> lock(f.fake->mutex); f.fake->expire = true; }
      f.executor->start();
      {
        std::unique_lock<std::mutex> lock(f.fake->mutex);
        check(f.fake->changed.wait_for(lock, gate_budget, [&] {
          return std::find(f.fake->operations.begin(), f.fake->operations.end(), "release") != f.fake->operations.end();
        }), "native expiry converges to release");
      }
      check(f.executor->status().abort_reason == "expired", "native expiry label retained");
      check(!f.executor->play(f.identity()).ok, "native expiry invalidates preparation");
      f.executor->stop_thread();
      { std::lock_guard<std::mutex> lock(f.fake->mutex); f.fake->expire = false; }
      check(f.acquire().ok, "fresh authority acquired after expiry");
    }
    {
      Fixture f;
      f.fake->rejection = "native_limit_exceeded: segment=0 sample=0 axis=J3";
      check(f.executor->preload(f.plan, f.bytes, f.record).reason == f.fake->rejection, "typed refusal retained");
      check(f.executor->status().consumer_action == "correct_request", "limit consumer action");
      f.fake->rejection = "future_reason: context";
      check(f.executor->preload(f.plan, f.bytes, f.record).reason == f.fake->rejection, "unknown reason retained");
      check(f.executor->status().state == "aborted", "unknown reason aborts");
    }
    std::cout << "rt_control_executor_test PASS\n";
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
