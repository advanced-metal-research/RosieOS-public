#pragma once

#include <condition_variable>
#include <memory>
#include <set>

#include "execution_backend.hpp"
#include "rt_control_program.hpp"
#include "runtime_config.hpp"

namespace dense_joint_trajectory {

// Start and revocation serialize under mutex_. Read-only observations use an
// independent client outside that mutex, then revalidate cancellation and epoch.
// No pending observation can start motion after Stop/revoke returns.
class RtControlExecutor final : public ExecutionBackend {
 public:
  using ClientFactory = std::function<std::unique_ptr<rtc::RtControlClient>()>;
  RtControlExecutor(DaemonConfig config, LeaderAuthority* leader, ClientFactory factory = {})
      : config_(std::move(config)), leader_(leader), factory_(std::move(factory)) {
    out_.backend = "rt_core";
    out_.state = "idle";
    published_ = out_;
    if (!factory_) factory_ = [this] {
      return std::make_unique<rtc::RtControlClient>(config_.control_socket);
    };
  }
  ~RtControlExecutor() override { stop_thread(); }

  bool start() override {
    running_ = true;
    thread_ = std::thread([this] {
      while (running_) {
        tick();
        std::unique_lock<std::mutex> lock(wait_mutex_);
        changed_.wait_for(lock, poll_interval(), [this] { return !running_; });
      }
    });
    return true;
  }
  void stop_thread() override {
    running_ = false;
    changed_.notify_all();
    if (thread_.joinable()) thread_.join();
    stop("daemon_shutdown");
  }

  Rejection leader_acquired(const LeaderAuthority::Snapshot& snapshot) override {
    std::unique_lock<std::mutex> lock(mutex_);
    PublishOnExit publish{*this};
    const auto cancellation = ++cancellation_;
    out_.native_state = "acquiring";
    publish_status();
    try {
      auto observer = factory_();
      if (cleanup_failed_) {
        reconcile_cleanup(*observer, std::min(shutdown_deadline(), deadline()), true);
      }
      require(!client_, "native_session_already_present");
      const auto described = unlocked_read(lock, snapshot, cancellation,
          [&] { return observer->describe(deadline()); }, false);
      require(described.ContractVersion == rtc::ContractVersion &&
              described.CapabilitiesDigest == rtc::CapabilitiesDigest, "native_contract_mismatch");
      description_ = described.Description;
      require(config_.machine_sha256.size() == 64 && config_.deployment_sha256.size() == 64 &&
              description_.MachineSHA256 == config_.machine_sha256 &&
              description_.DeploymentSHA256 == config_.deployment_sha256 &&
              description_.ConfigurationSHA256 == config_.configuration_sha256 &&
              description_.Backend == config_.expected_backend, "native_deployment_identity_mismatch");
      std::set<std::string> required{"describe", "acquire", "renew", "enable", "arm", "home",
          "restore_anchor", "recovery_status", "prepare_program", "start_program", "status",
          "subscribe_events", "stop", "release"};
      for (const auto& capability : described.Capabilities) {
        if (required.count(capability.Name)) {
          require(capability.State == "implemented", "native_capability_unavailable");
          required.erase(capability.Name);
        }
      }
      require(required.empty(), "native_capability_unavailable");
      require(description_.Axes.size() == kAxisCount && description_.MaxTrajectoryPoints >= 2 &&
              description_.ResidentPlans > 0, "native_axis_map_invalid");
      for (std::size_t i = 0; i < description_.Axes.size(); ++i) {
        const auto& axis = description_.Axes[i];
        require(axis.Index == i && axis.ID == "J" + std::to_string(i + 1) &&
                axis.PositionUnit == "rad" && std::isfinite(axis.MinPosition) &&
                std::isfinite(axis.MaxPosition) && axis.MinPosition < axis.MaxPosition &&
                std::isfinite(axis.MaxVelocity) && axis.MaxVelocity > 0 &&
                (!axis.MaxAcceleration || (std::isfinite(*axis.MaxAcceleration) && *axis.MaxAcceleration > 0)),
                "native_axis_map_invalid");
      }
      require(config_.home_policy == "home" || config_.home_policy == "restore_anchor", "invalid_home_policy");
      rtc::Binding binding;
      binding.PairID = config_.pair_id;
      binding.Revision = config_.pair_revision;
      binding.MachineSHA256 = description_.MachineSHA256;
      const auto initial = unlocked_read(lock, snapshot, cancellation,
          [&] { return observer->status(deadline()); }, false);
      adapter_incarnation_ = initial.AdapterIncarnation;
      daemon_incarnation_ = initial.DaemonIncarnation;
      verify_status(initial);
      client_ = factory_();
      const auto grant = client_->acquire(config_.controller_identity, binding, deadline());
      require(!grant.Session.empty() && grant.Generation > 0 && !grant.Stopping &&
              grant.LeaseMS > renewal_interval().count() &&
              grant.Binding.PairID == binding.PairID && grant.Binding.Revision == binding.Revision &&
              grant.Binding.MachineSHA256 == binding.MachineSHA256, "native_binding_mismatch");
      const auto renewed = client_->renew(deadline());
      require(renewed.Session == grant.Session && renewed.Generation == grant.Generation &&
              renewed.DeadlineHostNS > grant.DeadlineHostNS && !renewed.Stopping, "native_grant_mismatch");
      client_->start_renewal(renewal_interval());
      lease_epoch_ = snapshot.fence_epoch;
      const auto state = unlocked_read(lock, snapshot, cancellation,
          [&] { return observer->status(deadline()); }, false);
      verify_status(state);
      const auto events = unlocked_read(lock, snapshot, cancellation,
          [&] { return observer->events(0, deadline()); }, false);
      require(events.AdapterIncarnation == adapter_incarnation_, "native_incarnation_changed");
      cursor_ = events.LatestSequence;
      completed_event_ = false;
      out_ = ExecutorStatus{};
      out_.backend = "rt_core";
      out_.state = "idle";
      out_.native_state = "owned";
      clear_error();
      return Rejection::accept();
    } catch (const std::exception& error) {
      if (cancellation_.load() != cancellation)
        return Rejection::refuse(operation_cancelled, -1, "acquisition cancelled");
      return failure(error, true);
    }
  }

  Rejection preload(const Plan& plan, const std::vector<unsigned char>& bytes,
                    const std::string& record_path) override {
    const auto leader = leader_->snapshot();
    std::lock_guard<std::mutex> lock(mutex_);
    PublishOnExit publish{*this};
    // Ingest is unauthenticated: request preconditions cannot revoke a session,
    // consume renewal results, or cancel an acquisition/commissioning observation.
    if (out_.native_state == "acquiring" || !client_ || cleanup_failed_)
      return Rejection::refuse("native_acquisition_required", -1, "acquire native authority before preload");
    if (!leader.active || leader_->renewed_deadline(leader) <= monotonic_ms() || leader.fence_epoch != lease_epoch_)
      return Rejection::refuse("leader_lease_expired", -1, "preload requires current leader authority");
    if (observing_)
      return Rejection::refuse("native_commissioning_in_progress", -1, "commissioning is pending");
    if (out_.state == "playing")
      return Rejection::refuse("native_playback_in_progress", -1, "playback is pending");
    try {
      check_authority(leader);
      bool torch = false;
      for (const auto& segment : plan.segments)
        for (const auto& sample : segment.samples) torch = torch || sample.torch_on;
      if (torch) {
        out_.abort_reason = "native_torch_unsupported";
        out_.consumer_action = "remove_torch_samples";
        return Rejection::refuse(out_.abort_reason, -1, "torch samples are unsupported by this daemon");
      }
      ++cancellation_;
      auto identity = native_identity(plan);
      const auto program = client_->prepare_program(std::string(bytes.begin(), bytes.end()), deadline());
      identity.NormalisedDigest = program.Identity.NormalisedDigest;
      require(identity.NormalisedDigest.size() == 71 && identity.NormalisedDigest.rfind("sha256:", 0) == 0 &&
              same_identity(identity, program.Identity) &&
              program.AxisMask == plan.header.axis_mask &&
              program.Segments == std::int64_t(plan.segments.size()) &&
              program.Samples == std::int64_t(plan.header.total_sample_count) &&
              program.NormalisedSamples == program.Samples - program.Segments + 1 &&
              program.RequiresProcessIO == torch, "native_program_identity_mismatch");
      // This record is diagnostic persistence, never authority to replay a handle
      // after restart or lease loss. Start always requires a fresh in-memory preparation.
      const auto path = record_path + ".rt-core.json";
      const auto temporary = path + ".tmp";
      {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        output << rtc::encode(program) << '\n';
        output.close();
        require(bool(output), "store_write_failed");
      }
      require(::rename(temporary.c_str(), path.c_str()) == 0, "store_write_failed");
      plan_ = plan;
      program_ = program;
      prepared_ = true;
      out_.state = "staged";
      out_.trajectory_digest = identity.TrajectoryDigest;
      out_.program_identity_json = rtc::encode(identity);
      out_.execution_state = "prepared";
      out_.execution_generation = 0;
      out_.native_sequence = 0;
      out_.segment_index = 0;
      out_.sample_index = 0;
      out_.t_s = 0;
      clear_error();
      return Rejection::accept();
    } catch (const rtc::Rejected& error) {
        // Admission limit refusals leave any prior prepared program intact.
        return failure(error, error.reason.rfind(rtc::ReasonNativeLimitExceeded, 0) != 0);
      }
      catch (const std::exception& error) { return failure(error, true); }
  }

  Rejection play(const PlayIdentity& identity) override {
    const auto leader = leader_->snapshot();
    std::unique_lock<std::mutex> lock(mutex_);
    PublishOnExit publish{*this};
    if (observing_) return Rejection::refuse("native_commissioning_in_progress", -1, "commissioning is pending");
    const auto cancellation = ++cancellation_;
    observing_ = true;
    ObservationOnExit observing{*this, cancellation};
    auto observer = factory_();
    try {
      check_authority(leader);
      require(prepared_, "native_preparation_required");
      require(identity.leader_fence_epoch == lease_epoch_, "leader_lease_expired");
      const auto& h = plan_.header;
      require(identity.plan_id == h.plan_id && identity.program_id == h.program_id &&
              identity.program_digest == h.program_digest && identity.trajectory_digest == h.trajectory_digest &&
              identity.manifest_revision == h.manifest_revision && identity.plan_revision == h.plan_revision,
              "native_program_identity_mismatch");
      require(out_.state != "playing", "native_playback_in_progress");
      publish_phase("preparation");
      auto state = read_status(lock, *observer, leader, cancellation);
      verify_status(state);
      require(same_identity(state.Execution.Identity, program_.Identity), "native_program_identity_mismatch");
      require(state.Execution.State == "prepared", "native_preparation_retired");
      const auto mask = program_.AxisMask;
      if ((state.Core.HomeValidMask & mask) != mask) {
        publish_phase(config_.home_policy);
        if (config_.home_policy == "restore_anchor") {
          const auto recovered = client_->restore_anchor(mask, deadline());
          require((recovered.HomeValidMask & mask) == mask && recovered.Faults.empty(), "native_home_unconfirmed");
        } else {
          for (const auto& axis : description_.Axes)
            if ((mask & (1u << axis.Index)) && !(state.Core.HomeValidMask & (1u << axis.Index)))
              require(axis.NativeHome, "native_home_unavailable");
          receipt(client_->home(mask, deadline()), rtc::Operation::Home);
        }
        observe(lock, *observer, leader, cancellation, [&](const auto& s) {
          return (s.Core.HomeValidMask & mask) == mask && s.Core.CommissioningPhase == 0 &&
                 s.Core.Armed == 0 && (s.Core.AxisEnableMask & mask) == 0;
        });
      }
      const auto recovery = client_->recovery_status(deadline());
      require(recovery.Faults.empty(), "native_fault_observed");
      publish_phase("enable_arm");
      receipt(client_->enable(mask, deadline()), rtc::Operation::Enable);
      receipt(client_->arm(deadline()), rtc::Operation::Arm);
      publish_phase("readiness");
      observe(lock, *observer, leader, cancellation, [&](const auto& s) {
        bool ready = s.Core.Armed == 1 && (s.Core.AxisEnableMask & mask) == mask;
        for (std::size_t i = 0; i < s.Core.Axes.size(); ++i) {
          if (!(mask & (1u << i))) continue;
          const auto& readiness = s.Core.Axes[i].Readiness;
          if (readiness == "faulted" || readiness == "coordinate_invalid" || readiness == "home_required" ||
              readiness == "mode_mismatch") throw rtc::ProtocolError("native_readiness_" + readiness);
          require(readiness == "ready" || readiness == "not_enabled" ||
                  readiness == "not_operation_enabled" || readiness == "brake_wait", "native_readiness_" + readiness);
          ready = ready && readiness == "ready";
        }
        return ready;
      });
      // Reconcile history and the full prepared identity immediately before Start.
      publish_phase("start");
      consume_events(lock, *observer, leader, cancellation);
      state = read_status(lock, *observer, leader, cancellation);
      verify_status(state);
      require(state.Execution.State == "prepared" && same_identity(state.Execution.Identity, program_.Identity),
              "native_program_identity_mismatch");
      check_authority(leader);
      const auto started = client_->start_program(program_.Identity, deadline());
      receipt(started, rtc::Operation::StartProgram);
      out_.native_sequence = started.Sequence;
      state = read_status(lock, *observer, leader, cancellation);
      verify_status(state);
      require(state.Execution.NativeSequence == started.Sequence &&
              same_identity(state.Execution.Identity, program_.Identity), "native_program_identity_mismatch");
      out_.execution_generation = state.Execution.Generation;
      completed_event_ = false;
      prepared_ = false;
      out_.state = "playing";
      clear_error();
      return Rejection::accept();
    } catch (const rtc::Rejected& error) {
      if (cancellation_.load() != cancellation) return Rejection::refuse(operation_cancelled, -1, "Stop or replacement cancelled the observation");
      if (error.reason == rtc::ReasonHandleStarted) {
        try {
          const auto state = read_status(lock, *observer, leader, cancellation);
          verify_status(state);
          require(same_identity(state.Execution.Identity, program_.Identity) &&
                  state.Execution.NativeSequence != 0, "native_program_identity_mismatch");
          out_.native_sequence = state.Execution.NativeSequence;
          out_.execution_generation = state.Execution.Generation;
          completed_event_ = false;
          out_.state = "playing";
          prepared_ = false;
          return Rejection::accept();
        } catch (const std::exception& nested) {
          if (cancellation_.load() != cancellation) return Rejection::refuse(operation_cancelled, -1, "Stop or replacement cancelled the observation");
          return failure(nested, true);
        }
      }
      if (error.reason == rtc::ReasonNotReady ||
          (error.reason == rtc::ReasonNativeRejected && error.native_result && error.native_result->Reason == 2)) {
        try {
          const auto state = read_status(lock, *observer, leader, cancellation);
          verify_status(state);
          if (state.Execution.State == "prepared" && same_identity(state.Execution.Identity, program_.Identity))
            return failure(error, false);
        } catch (const std::exception& nested) {
          if (cancellation_.load() != cancellation) return Rejection::refuse(operation_cancelled, -1, "Stop or replacement cancelled the observation");
          return failure(nested, true);
        }
      }
      return failure(error, true);
    } catch (const std::exception& error) {
      if (cancellation_.load() != cancellation) return Rejection::refuse(operation_cancelled, -1, "Stop or replacement cancelled the observation");
      return failure(error, true);
    }
  }

  Rejection pause_command() override {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishOnExit publish{*this};
    out_.abort_reason = "pause_unsupported";
    out_.consumer_action = "use_stop";
    return Rejection::refuse(out_.abort_reason, -1, "rt-core profile has no pause operation");
  }
  void stop(const std::string& reason) override {
    ++cancellation_;
    changed_.notify_all();
    std::lock_guard<std::mutex> lock(mutex_);
    PublishOnExit publish{*this};
    observing_ = false;
    clear_error(); out_.state = "aborted"; out_.abort_reason = reason; shutdown_locked();
  }
  void revoke_motion() override { stop("leader_revoked"); }

  ExecutorStatus status() const override {
    const auto leader = leader_->snapshot();
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto out = published_;
    out.leader_fresh = leader.active && leader.deadline_ms > monotonic_ms();
    out.authority_fresh = out.native_state == "owned" && published_epoch_ == leader.fence_epoch && out.leader_fresh;
    return out;
  }

  // Polling never launches motion; it only observes or converges to Stop/release.
  void tick() {
    const auto leader = leader_->snapshot();
    std::unique_lock<std::mutex> lock(mutex_);
    if (!client_ || cleanup_failed_ || observing_ || out_.native_state == "acquiring") return;
    if (!leader.active && leader.fence_epoch == lease_epoch_) return;
    PublishOnExit publish{*this};
    const auto cancellation = cancellation_.load();
    try {
      check_authority(leader);
      const auto now = std::chrono::steady_clock::now();
      if (now < next_poll_) return;
      next_poll_ = now + (out_.state == "playing" ? poll_interval() : idle_interval());
      auto observer = factory_();
      const auto state = read_status(lock, *observer, leader, cancellation);
      verify_status(state);
      consume_events(lock, *observer, leader, cancellation);
      if (out_.state != "playing") return;
      require(state.Execution.Generation == out_.execution_generation &&
              state.Execution.NativeSequence == out_.native_sequence &&
              same_identity(state.Execution.Identity, program_.Identity), "execution_identity_mismatch");
      out_.execution_state = state.Execution.State;
      if (state.Execution.State == "faulted" || state.Execution.State == "cancelled")
        throw rtc::ProtocolError(state.Execution.Error.empty() ? state.Execution.State : state.Execution.Error);
      if (state.PlanCursor.ActiveHandle != 0 || state.Execution.State == "completed") {
        const auto progress = native_progress(plan_, state.PlanCursor.SampleIndex,
                                              state.Execution.State == "completed");
        out_.segment_index = progress.segment;
        out_.sample_index = progress.sample;
        out_.t_s = progress.seconds;
      }
      if (state.Execution.State == "completed" && completed_event_) out_.state = "done";
    } catch (const std::exception& error) {
      if (cancellation_.load() == cancellation) failure(error, true);
    }
  }

 private:
  struct PublishOnExit {
    RtControlExecutor& owner;
    ~PublishOnExit() { owner.publish_status(); }
  };
  struct ObservationOnExit {
    RtControlExecutor& owner;
    std::uint64_t cancellation;
    ~ObservationOnExit() {
      if (owner.cancellation_.load() == cancellation) {
        owner.observing_ = false;
        owner.out_.commissioning_phase.clear();
      }
    }
  };
  // Unverified daemon refusal label shared by all cancelled native operations.
  static constexpr const char* operation_cancelled = "native_operation_cancelled";
  void publish_phase(const std::string& phase) {
    out_.commissioning_phase = phase;
    publish_status();
  }
  void publish_status() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    published_ = out_;
    published_epoch_ = lease_epoch_;
  }
  // Unverified profile cadence: 100 ms, one fifth of the 500 ms native lease.
  static std::chrono::milliseconds renewal_interval() { return std::chrono::milliseconds(100); }
  // Unverified observation cadence: 5 ms; only received status/events establish progress.
  static std::chrono::milliseconds poll_interval() { return std::chrono::milliseconds(5); }
  // Unverified idle cadence: 100 ms limits socket traffic while observing lease health.
  static std::chrono::milliseconds idle_interval() { return std::chrono::milliseconds(100); }
  rtc::Deadline deadline() const {
    return std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.control_timeout_ms);
  }
  // Unverified shutdown I/O budget: 1000 ms, including the SDK's 250 ms renewal
  // join, prevents Stop/Release/reconcile from each occupying a full control timeout.
  static rtc::Deadline shutdown_deadline() {
    return std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  }
  static void require(bool ok, const std::string& reason) { if (!ok) throw rtc::ProtocolError(reason); }
  static void receipt(const rtc::Response& response, rtc::Operation operation) {
    require(response.Schema == rtc::ResponseSchema && response.Operation == rtc::to_json(operation).text &&
            response.Sequence != 0 && response.Error.empty(), "invalid_native_receipt");
  }
  void clear_error() {
    out_.abort_reason.clear(); out_.detail.clear(); out_.consumer_action.clear(); out_.native_result_json = "null";
  }
  Rejection failure(const std::exception& error, bool shutdown) {
    const auto* rejected = dynamic_cast<const rtc::Rejected*>(&error);
    const auto reason = rejected ? rejected->reason : std::string(error.what());
    out_.abort_reason = reason;
    out_.detail = error.what();
    out_.consumer_action = shutdown ? native_consumer_action(reason) : "correct_request";
    out_.native_result_json = rejected && rejected->native_result ? rtc::encode(*rejected->native_result) : "null";
    if (shutdown) { out_.state = "aborted"; shutdown_locked(); }
    return Rejection::refuse(reason, -1, out_.detail);
  }
  void check_authority(const LeaderAuthority::Snapshot& leader) {
    require(client_ && !cleanup_failed_, "native_acquisition_required");
    require(leader.active && leader_->renewed_deadline(leader) > monotonic_ms() &&
            leader.fence_epoch == lease_epoch_, "leader_lease_expired");
    const auto update = client_->take_renewal();
    if (update && update->error) std::rethrow_exception(update->error);
    const auto grant = client_->grant();
    require(!grant.Session.empty() && !grant.Stopping, "native_authority_unavailable");
  }
  void verify_status(const rtc::ProcessStatus& state) {
    require(state.TimeNS != 0 && state.Core.TimeNS != 0 && state.Motion.TimeNS != 0, "native_status_unavailable");
    require(state.AdapterIncarnation == adapter_incarnation_ && state.DaemonIncarnation == daemon_incarnation_, "native_incarnation_changed");
    require(state.Core.ConfigurationSHA256 == description_.ConfigurationSHA256 &&
            state.Core.Axes.size() == description_.Axes.size(), "native_deployment_identity_mismatch");
    require(state.Core.SafetyFaultMask == 0, "native_fault_observed");
    // Unverified native status age ceiling: 200 ms, expressed in host monotonic nanoseconds.
    constexpr std::uint64_t max_status_age_ns = 200000000;
    const auto now = std::uint64_t(monotonic_ms()) * 1000000;
    require(state.Core.TimeNS > now || now - state.Core.TimeNS <= max_status_age_ns, "native_status_stale");
  }
  // Always reacquires the executor lock, including on errors. The independent
  // reader cannot occupy the authenticated client's Stop/Release lanes.
  template<class Read> auto unlocked_read(std::unique_lock<std::mutex>& lock,
      const LeaderAuthority::Snapshot& leader, std::uint64_t cancellation, Read read, bool authorize = true) -> decltype(read()) {
    lock.unlock();
    try {
      auto result = read();
      lock.lock();
      require(cancellation_.load() == cancellation, operation_cancelled);
      if (authorize) check_authority(leader);
      return result;
    } catch (...) {
      if (!lock.owns_lock()) lock.lock();
      throw;
    }
  }
  rtc::ProcessStatus read_status(std::unique_lock<std::mutex>& lock, rtc::RtControlClient& observer,
      const LeaderAuthority::Snapshot& leader, std::uint64_t cancellation, rtc::Deadline end) {
    return unlocked_read(lock, leader, cancellation, [&] { return observer.status(end); });
  }
  rtc::ProcessStatus read_status(std::unique_lock<std::mutex>& lock, rtc::RtControlClient& observer,
      const LeaderAuthority::Snapshot& leader, std::uint64_t cancellation) {
    return read_status(lock, observer, leader, cancellation, deadline());
  }
  template<class Predicate> void observe(std::unique_lock<std::mutex>& lock, rtc::RtControlClient& observer,
      const LeaderAuthority::Snapshot& leader, std::uint64_t cancellation, Predicate predicate) {
    const auto end = deadline();
    for (;;) {
      const auto state = read_status(lock, observer, leader, cancellation, end);
      verify_status(state);
      if (predicate(state)) return;
      require(std::chrono::steady_clock::now() < end, "readiness_observation_expired");
      unlocked_read(lock, leader, cancellation, [&] {
        std::unique_lock<std::mutex> wait(wait_mutex_);
        changed_.wait_for(wait, poll_interval(), [&] { return cancellation_.load() != cancellation; });
        return true;
      });
    }
  }
  void consume_events(std::unique_lock<std::mutex>& lock, rtc::RtControlClient& observer,
      const LeaderAuthority::Snapshot& leader, std::uint64_t cancellation) {
    const auto cursor = cursor_;
    const auto batch = unlocked_read(lock, leader, cancellation, [&] { return observer.events(cursor, deadline()); });
    require(batch.AdapterIncarnation == adapter_incarnation_, "native_incarnation_changed");
    for (const auto& event : batch.Events) {
      if (event.Type == "events_dropped") {
        const auto state = read_status(lock, observer, leader, cancellation);
        verify_status(state);
        if (out_.state == "playing") {
          require(state.Execution.Generation == out_.execution_generation &&
                  state.Execution.NativeSequence == out_.native_sequence &&
                  same_identity(state.Execution.Identity, program_.Identity), "execution_identity_mismatch");
          if (state.Execution.State == "completed") completed_event_ = true;
          require(state.Execution.State != "faulted" && state.Execution.State != "cancelled", "native_execution_aborted");
        }
      }
      cursor_ = event.Sequence;
      if (event.Type == "execution_completed" && event.Execution &&
          event.Execution->Generation == out_.execution_generation &&
          event.Execution->NativeSequence == out_.native_sequence) completed_event_ = true;
    }
  }
  void shutdown_locked() {
    const auto end = shutdown_deadline();
    ++cancellation_;
    observing_ = false;
    out_.commissioning_phase.clear();
    prepared_ = false;
    lease_epoch_ = 0;
    const auto action = out_.consumer_action;
    if (!client_) {
      out_.native_state = cleanup_state();
      out_.consumer_action = cleanup_failed_ ? "reconcile_status_then_acquire" :
          action.empty() ? "acquire_and_preload" : action;
      return;
    }
    out_.native_state = "reconciling";
    out_.consumer_action = "observe_standstill_before_acquire";
    publish_status();
    client_->stop_renewal();
    const auto fence = client_->fence();
    if (!fence.Session.empty()) {
      try {
        require(std::chrono::steady_clock::now() < end, "native_shutdown_expired");
        client_->stop(end);
      }
      catch (const std::exception& error) { out_.detail += std::string("; stop: ") + error.what(); cleanup_failed_ = true; }
      // Keep the authenticated original fence even when expired Stop returns no session.
      try {
        require(std::chrono::steady_clock::now() < end, "native_shutdown_expired");
        rtc::Request request;
        request.Operation = rtc::to_json(rtc::Operation::Release).text;
        request.Fence = fence;
        const auto released = rtc::RtControlClient::data<rtc::Grant>(client_->command(request, end));
        require(released.Session.empty(), "release_uncertain");
      } catch (const std::exception& error) { out_.detail += std::string("; release: ") + error.what(); cleanup_failed_ = true; }
      cleanup_failed_ = true;
      try { reconcile_cleanup(*client_, std::min(end, deadline())); }
      catch (const std::exception& error) { out_.detail += std::string("; reconcile: ") + error.what(); }
    }
    out_.native_state = cleanup_state();
    out_.consumer_action = cleanup_failed_ ? "reconcile_status_then_acquire" :
        action.empty() ? "acquire_and_preload" : action;
    client_.reset();
  }

  std::string cleanup_state() const {
    return !cleanup_failed_ ? "inactive" : cleanup_grant_active_ ? "grant_active" : "cleanup_uncertain";
  }
  // Requires distinct inhibited samples from one current incarnation with no
  // native grant. Restart invalidates old handles, not the ability to reconcile;
  // an active grant (including a foreign owner's) refuses only this acquire.
  void reconcile_cleanup(rtc::RtControlClient& observer, rtc::Deadline end, bool acquiring = false) {
    cleanup_grant_active_ = false;
    const auto after = std::uint64_t(monotonic_ms()) * 1000000;
    rtc::ProcessStatus previous;
    const auto inhibited = [&](const rtc::ProcessStatus& state) {
      return state.TimeNS != 0 && state.Motion.TimeNS != 0 && state.Core.TimeNS > after &&
          !state.AdapterIncarnation.empty() && !state.DaemonIncarnation.empty() &&
          state.Core.Armed == 0 && state.Core.AxisEnableMask == 0 && state.Core.CommissioningPhase == 0 &&
          state.Grant && state.Grant->GrantActive == 0 && state.Core.Axes.size() == kAxisCount &&
          state.Core.ConfigurationSHA256 == config_.configuration_sha256 && state.Execution.State != "executing";
    };
    for (;;) {
      require(std::chrono::steady_clock::now() < end, "native_standstill_unconfirmed");
      const auto state = observer.status(end);
      out_.execution_state = state.Execution.State;
      if (acquiring && state.Grant && state.Grant->GrantActive != 0) {
        cleanup_grant_active_ = true;
        throw rtc::ProtocolError("native_grant_active");
      }
      bool stopped = inhibited(state) && inhibited(previous) && state.Core.TimeNS > previous.Core.TimeNS &&
          state.AdapterIncarnation == previous.AdapterIncarnation && state.DaemonIncarnation == previous.DaemonIncarnation;
      if (stopped) for (std::size_t i = 0; i < kAxisCount; ++i) {
        const auto& axis = state.Core.Axes[i];
        stopped = stopped && axis.PositionCounts == previous.Core.Axes[i].PositionCounts &&
            (!axis.LogicalVelocity || *axis.LogicalVelocity == 0);
      }
      if (stopped) {
        cleanup_failed_ = false;
        return;
      }
      require(std::chrono::steady_clock::now() < end, "native_standstill_unconfirmed");
      previous = state;
      std::this_thread::sleep_until(std::min(end, std::chrono::steady_clock::now() + poll_interval()));
    }
  }

  DaemonConfig config_;
  LeaderAuthority* leader_;
  ClientFactory factory_;
  mutable std::mutex mutex_;
  std::unique_ptr<rtc::RtControlClient> client_;
  rtc::NativeDescription description_;
  Plan plan_;
  rtc::Program program_;
  ExecutorStatus out_;
  mutable std::mutex status_mutex_;
  ExecutorStatus published_;
  std::uint64_t published_epoch_ = 0;
  bool observing_ = false;
  std::atomic<std::uint64_t> cancellation_{0};
  std::chrono::steady_clock::time_point next_poll_{};
  bool prepared_ = false, completed_event_ = false, cleanup_failed_ = false;
  bool cleanup_grant_active_ = false;
  std::uint64_t lease_epoch_ = 0, cursor_ = 0;
  std::string adapter_incarnation_, daemon_incarnation_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex wait_mutex_;
  std::condition_variable changed_;
};
}  // namespace dense_joint_trajectory
