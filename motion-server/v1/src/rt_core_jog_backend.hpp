#pragma once

#include "../../../rt-core/clients/cpp/include/rosie/rt_jog_producer.hpp"
#include <array>
#include <set>
#include <future>

namespace cartesian_rt {
namespace api = rosie::rt_control;
using Authorization = std::function<bool()>;

inline void require_authorized(const Authorization& authorized) {
  if (authorized && !authorized()) throw std::runtime_error("rt_core_execution_cancelled");
}

// Unverified application polling/receipt budgets; source expiry remains bounded
// by the public 250,000,000 ns ceiling. Renewal cadence is the owner contract.
inline constexpr auto renewal_interval = std::chrono::milliseconds(100);
inline constexpr auto receipt_budget = std::chrono::milliseconds(200);
inline constexpr int poll_ms = 10;
inline api::Deadline receipt_deadline() { return std::chrono::steady_clock::now() + receipt_budget; }

struct Configuration {
  std::string socket, jog_socket, urdf, urdf_sha256;
  api::Binding binding;
  std::vector<std::string> joint_ids;
};

// One owner serializes lifecycle and jog calls. Observation must precede each
// publication; a failed operation cannot authorize a replacement frame.
class Client {
 public:
  virtual ~Client() = default;
  virtual api::Description describe() = 0;
  virtual api::Grant acquire(const api::Binding&) = 0;
  virtual api::Grant grant() = 0;
  virtual void start_renewal() = 0;
  virtual void check_renewal() = 0;
  virtual void stop_renewal() = 0;
  virtual api::ProcessStatus status() = 0;
  virtual api::JogIngressObservation ingress() = 0;
  virtual void enable(std::uint32_t, const Authorization& = {}) = 0;
  virtual void arm(const Authorization& = {}) = 0;
  virtual void home(std::uint32_t, const Authorization& = {}) = 0;
  virtual bool home_receipt() { return true; }
  virtual void begin(std::uint32_t, std::uint64_t, std::uint64_t) = 0;
  virtual bool update(const std::vector<double>&, std::uint64_t, std::uint64_t) = 0;
  virtual void end() = 0;
  virtual void stop() = 0;
  virtual void release() = 0;
  virtual api::Response prepare(std::uint32_t, const std::vector<api::Point>&, const api::Identity&, const Authorization& = {}) = 0;
  virtual api::Response start_trajectory(std::uint64_t, const api::Identity&, const Authorization& = {}) = 0;
  virtual api::EventBatch events(std::uint64_t) = 0;
};

class LocalClient final : public Client {
  api::RtControlClient control_;
  api::RtControlClient observer_;
  std::string jog_socket_;
  std::unique_ptr<api::RtJogProducer> producer_;
  std::future<void> home_;
  std::exception_ptr home_failure_;
  std::string control_socket_;
  std::shared_ptr<std::atomic<bool>> home_cancel_;

  // Abandon receipt ownership without waiting for commissioning or its socket.
  // The worker owns its client and cancellation token, preventing owner use-after-free.
  void abandon_home() {
    if (!home_.valid()) return;
    home_cancel_->store(true);
    if (home_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      try { home_.get(); } catch (...) { home_failure_ = std::current_exception(); }
    } else {
      home_failure_ = std::make_exception_ptr(std::runtime_error("rt_core_home_abandoned"));
      home_ = {};
    }
    home_cancel_.reset();
  }
  api::Response invoke(api::Request request, const Authorization& authorized, api::Deadline deadline) {
    require_authorized(authorized);
    try {
      return control_.command(request, deadline, [authorized] { return authorized && !authorized(); });
    } catch (const api::client_detail::StreamStopped&) {
      throw std::runtime_error("rt_core_execution_cancelled");
    }
  }
 public:
  explicit LocalClient(const Configuration& c)
      : control_(c.socket), observer_(c.socket), jog_socket_(c.jog_socket), control_socket_(c.socket) {}
  ~LocalClient() override { abandon_home(); }
  api::Description describe() override { return observer_.describe(receipt_deadline()); }
  api::Grant acquire(const api::Binding& b) override {
    control_.set_fence({});
    return control_.acquire("motion-server:cartesiand", b, receipt_deadline());
  }
  api::Grant grant() override { return control_.grant(); }
  void start_renewal() override { control_.start_renewal(renewal_interval); }
  void check_renewal() override {
    auto renewal = control_.take_renewal();
    if (renewal && renewal->error) std::rethrow_exception(renewal->error);
    const auto g = control_.grant();
    if (g.Stopping || g.Session.empty() || g.DeadlineHostNS <= api::host_monotonic_ns())
      throw api::TransportError("control_session_stale");
  }
  void stop_renewal() override { control_.stop_renewal(); }
  api::ProcessStatus status() override { return observer_.status(receipt_deadline()); }
  api::JogIngressObservation ingress() override { return observer_.jog_ingress(receipt_deadline()); }
  void enable(std::uint32_t mask, const Authorization& authorized = {}) override {
    api::Request r; r.Operation = "enable"; r.AxisMask = mask; invoke(r, authorized, receipt_deadline());
  }
  void arm(const Authorization& authorized = {}) override {
    api::Request r; r.Operation = "arm"; invoke(r, authorized, receipt_deadline());
  }
  // The adapter's Home reply drains commissioning. Keep that receipt off the
  // owner loop so its independent urgent lane can fence a committed Home.
  void home(std::uint32_t mask, const Authorization& authorized = {}) override {
    require_authorized(authorized);
    if (home_.valid()) throw std::runtime_error("rt_core_home_receipt_pending");
    api::Request request;
    request.Operation = "home"; request.AxisMask = mask; request.Fence = control_.fence();
    auto client = std::make_shared<api::RtControlClient>(control_socket_);
    home_cancel_ = std::make_shared<std::atomic<bool>>(false);
    std::packaged_task<void()> work([client, request, authorized, cancelled = home_cancel_]() mutable {
      const auto stop = [authorized, cancelled] { return cancelled->load() || (authorized && !authorized()); };
      if (stop()) throw std::runtime_error("rt_core_home_abandoned");
      // Adapter Home contract: 60,000 ms commissioning deadline, unverified on hardware.
      try { client->command(request, std::chrono::steady_clock::now() + std::chrono::milliseconds(60000), stop); }
      catch (const api::client_detail::StreamStopped&) { throw std::runtime_error("rt_core_home_abandoned"); }
    });
    home_ = work.get_future();
    std::thread(std::move(work)).detach();
  }
  bool home_receipt() override {
    if (home_failure_) {
      auto failure = std::exchange(home_failure_, {});
      std::rethrow_exception(failure);
    }
    if (!home_.valid()) return true;
    if (home_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return false;
    home_.get(); return true;
  }
  void begin(std::uint32_t mask, std::uint64_t captured, std::uint64_t deadline) override {
    producer_ = std::make_unique<api::RtJogProducer>(control_, jog_socket_, mask, captured, deadline, receipt_deadline());
  }
  bool update(const std::vector<double>& velocity, std::uint64_t captured, std::uint64_t deadline) override {
    return producer_ && producer_->update(velocity, captured, deadline);
  }
  void end() override {
    auto producer = std::move(producer_);
    if (producer) producer->end(receipt_deadline());
  }
  void stop() override {
    try { control_.stop(receipt_deadline()); }
    catch (...) { abandon_home(); throw; }
    abandon_home();
  }
  void release() override { control_.release(receipt_deadline()); }
  api::Response prepare(std::uint32_t mask, const std::vector<api::Point>& points, const api::Identity& identity, const Authorization& authorized = {}) override {
    api::Request r; r.Operation = "prepare_trajectory"; r.AxisMask = mask; r.Points = points;
    r.Identity = identity;
    return invoke(r, authorized, receipt_deadline());
  }
  api::Response start_trajectory(std::uint64_t handle, const api::Identity& identity, const Authorization& authorized = {}) override {
    api::Request r; r.Operation = "start_trajectory"; r.Handle = handle;
    r.Identity = identity;
    return invoke(r, authorized, receipt_deadline());
  }
  api::EventBatch events(std::uint64_t after) override { return observer_.events(after, receipt_deadline()); }
};

// A Stop or uncertain receipt latches inhibition. Only a subsequent operator
// arm can establish new authority; neither new input nor renewal rearms motion.
class Backend {
  Client& client_;
  Configuration config_;
  api::Description description_;
  std::array<std::size_t, 6> indices_{};
  std::uint32_t mask_ = 0;
  std::uint64_t generation_ = 0, jog_generation_ = 0, sent_sequence_ = 0;
  std::uint64_t sent_origin_ = 0, sent_not_before_ = 0;
  std::uint64_t input_deadline_ = 0;
  std::uint64_t configuration_epoch_ = 0, home_epoch_ = 0;
  std::string daemon_, adapter_;
  bool acquired_ = false, armed_ = false, jogging_ = false, closed_ = false;
  bool stop_confirmed_ = true;
  api::ProcessStatus status_;
  api::Json refusal_;
  std::string reason_;
  std::vector<double> applied_;
  std::string session_, run_key_;
  std::set<std::string> retired_runs_, retired_sessions_;
  std::uint64_t run_deadline_ = 0, handle_ = 0, native_sequence_ = 0, event_cursor_ = 0;
  api::Identity trajectory_identity_;
  std::uint64_t execution_generation_ = 0;
  bool trajectory_ = false, completion_event_ = false;
  bool home_confirmed_ = true;

  bool same_trajectory(const api::Identity& identity) const {
    return !trajectory_identity_.ProgramID.empty() && api::encode(identity) == api::encode(trajectory_identity_);
  }

  void record(const std::exception& error) {
    reason_ = error.what();
    if (const auto* rejected = dynamic_cast<const api::Rejected*>(&error)) {
      refusal_ = api::Json::object_value();
      reason_ = rejected->reason;
      if (rejected->native_result)
        refusal_.object.emplace_back("native_result", api::to_json(*rejected->native_result));
      if (rejected->native_jog_result)
        refusal_.object.emplace_back("native_jog_result", api::to_json(*rejected->native_jog_result));
    }
  }
  void record_cleanup(const std::string& operation, const std::exception& error) {
    if (refusal_.kind != api::Json::Kind::Object) refusal_ = api::Json::object_value();
    auto detail = api::Json::object_value();
    detail.object.emplace_back("error", api::to_json(std::string(error.what())));
    if (const auto* rejected = dynamic_cast<const api::Rejected*>(&error)) {
      if (rejected->native_result) detail.object.emplace_back("native_result", api::to_json(*rejected->native_result));
      if (rejected->native_jog_result) detail.object.emplace_back("native_jog_result", api::to_json(*rejected->native_jog_result));
    }
    // Replace a previous cleanup receipt rather than growing status on retries.
    auto found = std::find_if(refusal_.object.begin(), refusal_.object.end(), [&](const auto& field) { return field.first == operation; });
    if (found == refusal_.object.end()) refusal_.object.emplace_back(operation, std::move(detail));
    else found->second = std::move(detail);
  }
  void validate_description() {
    description_ = client_.describe();
    const auto& d = description_.Description;
    if (config_.binding.PairID.empty() || !config_.binding.Revision ||
        config_.binding.ConfigurationSHA256.empty() ||
        d.ConfigurationSHA256 != config_.binding.ConfigurationSHA256 ||
        (!config_.binding.MachineSHA256.empty() && d.MachineSHA256 != config_.binding.MachineSHA256))
      throw std::runtime_error("rt_core_binding_mismatch");
    if (config_.joint_ids.size() != indices_.size() || d.Axes.size() > rosie::protocol::max_axes)
      throw std::runtime_error("rt_core_requires_six_joint_ids");
    std::set<std::string> ids;
    mask_ = 0;
    for (std::size_t joint = 0; joint < indices_.size(); ++joint) {
      auto found = std::find_if(d.Axes.begin(), d.Axes.end(), [&](const auto& a) { return a.ID == config_.joint_ids[joint]; });
      if (!ids.insert(config_.joint_ids[joint]).second || found == d.Axes.end() ||
          found->Index != std::size_t(found - d.Axes.begin()) || found->PositionUnit != "rad" ||
          !std::isfinite(found->MaxVelocity) || found->MaxVelocity <= 0)
        throw std::runtime_error("rt_core_axis_mapping_invalid");
      indices_[joint] = found->Index;
      mask_ |= 1U << found->Index;
    }
    applied_.assign(d.Axes.size(), 0);
  }
  void acquire() {
    // Unverified tombstone capacity: refuse replacement after 256 retired sessions.
    if (retired_sessions_.size() >= 256) throw std::runtime_error("rt_core_session_history_full");
    validate_description();
    auto g = client_.acquire(config_.binding);
    acquired_ = true;
    generation_ = g.Generation;
    session_ = g.Session;
    if (!generation_ || g.Binding.PairID != config_.binding.PairID || g.Binding.Revision != config_.binding.Revision ||
        g.Binding.ConfigurationSHA256 != config_.binding.ConfigurationSHA256 ||
        (!config_.binding.MachineSHA256.empty() && g.Binding.MachineSHA256 != config_.binding.MachineSHA256))
      throw std::runtime_error("rt_core_grant_binding_mismatch");
    if (session_.empty() || retired_sessions_.count(session_))
      throw std::runtime_error("rt_core_retired_session");
    client_.start_renewal();
    status_ = client_.status();
    daemon_ = status_.DaemonIncarnation;
    adapter_ = status_.AdapterIncarnation;
    configuration_epoch_ = status_.Core.ConfigurationEpoch;
    home_epoch_ = status_.Core.HomeEpoch;
    if (daemon_.empty() || adapter_.empty()) throw std::runtime_error("rt_core_incarnation_unavailable");
  }
  bool ready() const {
    const auto& core = status_.Core;
    if (!core.Armed || !core.LeaseValid || (core.ConfigVerifiedMask & mask_) != mask_ ||
        (core.AxisEnableMask & mask_) != mask_ || core.Axes.size() != description_.Description.Axes.size()) return false;
    for (auto index : indices_) {
      const auto& axis = core.Axes[index];
      if (axis.Readiness != "ready" || !axis.PDOFresh || !axis.LogicalValid || !std::isfinite(axis.LogicalPosition) ||
          (description_.Description.Axes[index].RequireHome && !axis.HomeValid)) return false;
    }
    return true;
  }
  // Native update fields survive BeginJog. Only the exact submitted frame,
  // first observed after its publication began, can acknowledge this session.
  bool observed_sent_frame() const {
    return sent_sequence_ && status_.Jog && status_.Jog->JogGeneration == jog_generation_ &&
        status_.Jog->ObservedSourceSequence == sent_sequence_ &&
        status_.Jog->ObservedOriginHostNs == sent_origin_ &&
        status_.Jog->FirstObservedHostNs >= sent_not_before_;
  }
  void check() {
    client_.check_renewal();
    status_ = client_.status();
    const auto now = api::host_monotonic_ns();
    if (!status_.Core.TimeNS || status_.Core.TimeNS > now ||
        now - status_.Core.TimeNS >= rosie::protocol::fast_control_max_jog_input_age_ns)
      throw std::runtime_error("rt_core_status_stale");
    if (status_.DaemonIncarnation != daemon_ || status_.AdapterIncarnation != adapter_ ||
        status_.Core.ConfigurationSHA256 != config_.binding.ConfigurationSHA256)
      throw std::runtime_error("rt_core_identity_changed");
    if (armed_ && (configuration_epoch_ != status_.Core.ConfigurationEpoch || home_epoch_ != status_.Core.HomeEpoch))
      throw std::runtime_error("rt_core_coordinate_generation_changed");
    if (status_.Core.SafetyFaultMask || status_.Core.ExecutionFaultReasons || status_.Execution.State == "faulted")
      throw std::runtime_error("rt_core_faulted");
    if (status_.Core.Recovery && !status_.Core.Recovery->Faults.empty())
      throw std::runtime_error("rt_core_faulted");
    for (const auto& a : status_.Core.Axes)
      if (a.ErrorCode || a.Readiness == "faulted") throw std::runtime_error("rt_core_axis_faulted");
    if (!status_.Grant || !status_.Grant->GrantActive || status_.Grant->AppGrantGeneration != generation_ ||
        client_.grant().Generation != generation_ || client_.grant().Session != session_)
      throw std::runtime_error("rt_core_stop_fence");
    if (jogging_) {
      const auto ingress = client_.ingress();
      if (sent_sequence_ && ingress.SourceSequence == sent_sequence_ &&
          ingress.NowHostNS >= sent_not_before_ && ingress.Reason) {
        refusal_ = api::to_json(ingress);
        throw std::runtime_error("rt_core_jog_ingress_refused");
      }
      if (!status_.Jog || status_.Jog->JogGeneration != jog_generation_ || !status_.Jog->Open ||
          (observed_sent_frame() && status_.Jog->UpdateReason) || status_.Jog->StateReason) {
        if (status_.Jog) refusal_ = api::to_json(*status_.Jog);
        throw std::runtime_error("rt_core_jog_refused");
      }
      if (!ready()) throw std::runtime_error("rt_core_readiness_lost");
    }
  }
 public:
  Backend(Client& client, Configuration config) : client_(client), config_(std::move(config)) {}
  ~Backend() { shutdown(); }
  Backend(const Backend&) = delete;
  Backend& operator=(const Backend&) = delete;
  bool start() {
    try { acquire(); return true; }
    catch (const std::exception& e) { record(e); halt(reason_); return false; }
  }
  bool arm(const Authorization& authorized = {}) {
    if (closed_) return false;
    try {
      if (armed_) { check(); return true; }
      if (!stop_confirmed_) throw std::runtime_error("rt_core_stop_unconfirmed");
      if (!run_key_.empty()) {
        check(); require_authorized(authorized); client_.enable(mask_, authorized);
        require_authorized(authorized); client_.arm(authorized); armed_ = true;
        reason_.clear(); refusal_ = {}; return true;
      }
      // Reconcile a stopped/expired grant before a fresh operator request.
      if (acquired_) {
        // Even startup authority must be stopped before Release; an uncertain
        // Stop cannot authorize releasing and replacing the current grant.
        if (!halt("operator_rearm")) return false;
        client_.stop_renewal(); client_.release(); acquired_ = false;
      }
      acquire();
      check();
      require_authorized(authorized); client_.enable(mask_, authorized);
      require_authorized(authorized); client_.arm(authorized);
      armed_ = true;
      reason_.clear(); refusal_ = {};
      return true;
    } catch (const std::exception& e) { record(e); halt(reason_); return false; }
  }
  bool home(const Authorization& authorized = {}) {
    if (closed_) return false;
    try {
      if (!halt("operator_home")) return false;
      // Stop revokes the native capability. Home needs a fresh bound grant,
      // just like Arm, but must never issue Arm as part of commissioning.
      if (acquired_) {
        client_.stop_renewal(); client_.release(); acquired_ = false;
        retired_sessions_.insert(session_);
      }
      acquire();
      require_authorized(authorized); client_.enable(mask_, authorized);
      require_authorized(authorized); client_.home(mask_, authorized);
      home_confirmed_ = false;
      reason_.clear(); refusal_ = {};
      return true;
    } catch (const std::exception& e) { record(e); halt(reason_); return false; }
  }
  bool poll(std::uint64_t now) {
    if (closed_) return false;
    try {
      if (!acquired_) { status_ = client_.status(); return true; }
      if (!run_key_.empty() && now >= run_deadline_) {
        retire_run("rt_core_run_expired"); return false;
      }
      home_confirmed_ = client_.home_receipt();
      check();
      if (jogging_ && now >= input_deadline_) return halt("jog_input_deadline_expired");
      return true;
    } catch (const std::exception& e) { record(e); halt(reason_, true); return false; }
  }
  bool jog(const std::array<double, 6>& resolved, std::uint64_t captured_ns) {
    if (!armed_ || closed_ || trajectory_ || !run_key_.empty()) return false;
    try {
      check();
      if (!ready()) throw std::runtime_error("rt_core_not_ready");
      const auto now = api::host_monotonic_ns();
      if (!captured_ns || captured_ns > now || now - captured_ns >= rosie::protocol::fast_control_max_jog_input_age_ns)
        throw std::runtime_error("jog_input_deadline_expired");
      const auto deadline = captured_ns + rosie::protocol::fast_control_max_jog_input_age_ns;
      std::vector<double> candidate(applied_.size(), 0);
      double scale = 1;
      for (std::size_t joint = 0; joint < indices_.size(); ++joint) {
        if (!std::isfinite(resolved[joint])) throw std::runtime_error("jog_invalid_vector");
        if (resolved[joint] != 0)
          scale = std::min(scale, description_.Description.Axes[indices_[joint]].MaxVelocity / std::abs(resolved[joint]));
      }
      // Round a limiting common scale toward zero so multiplication cannot
      // promote a capped component above Describe's native velocity limit.
      if (scale < 1) scale = std::nextafter(scale, 0.0);
      for (std::size_t joint = 0; joint < indices_.size(); ++joint) {
        const auto index = indices_[joint];
        const auto velocity = resolved[joint] * scale;
        if (!std::isfinite(velocity) || std::abs(velocity) > description_.Description.Axes[index].MaxVelocity)
          throw std::runtime_error("rt_core_velocity_cap_exceeded");
        candidate[index] = velocity;
      }
      if (!jogging_) {
        jog_generation_ = sent_sequence_ = sent_origin_ = sent_not_before_ = 0;
        // Mark the attempted Begin before its receipt: uncertainty still calls End and Stop.
        jogging_ = true;
        client_.begin(mask_, captured_ns, deadline);
        status_ = client_.status();
        if (!status_.Jog || !status_.Jog->Open) throw std::runtime_error("rt_core_jog_unobserved");
        jog_generation_ = status_.Jog->JogGeneration;
        check();
      } else if (!observed_sent_frame()) {
        // Keep at most one unobserved publication. Never hide a rejected frame
        // by overwriting it with a later accepted publication.
        return false;
      }
      const auto publishing = api::host_monotonic_ns();
      if (!client_.update(candidate, captured_ns, deadline)) throw std::runtime_error("jog_publisher_busy");
      applied_ = std::move(candidate);
      input_deadline_ = deadline;
      ++sent_sequence_;
      sent_origin_ = captured_ns;
      sent_not_before_ = publishing;
      return true;
    } catch (const std::exception& e) { record(e); halt(reason_); return false; }
  }
  bool halt(const std::string& reason, bool retain_home_failure = false) {
    reason_ = reason;
    armed_ = false;
    trajectory_ = false;
    if (jogging_) {
      jogging_ = false;
      try { client_.end(); } catch (const std::exception& e) { record_cleanup("end_jog", e); }
    }
    std::fill(applied_.begin(), applied_.end(), 0);
    stop_confirmed_ = !acquired_;
    if (acquired_) {
      try { client_.stop(); stop_confirmed_ = true; }
      catch (const std::exception& e) { record_cleanup("stop", e); }
      try { client_.home_receipt(); }
      catch (const std::exception& e) {
        if (retain_home_failure) record(e);
        else record_cleanup("home", e);
      }
    }
    // Stop fences even a refused End (for example, after an external Stop).
    return stop_confirmed_;
  }
  // Only source-confirmed deadlines may bind a run. Tombstones prevent a
  // delayed command from acquiring fresh authority after the run was stopped.
  bool bind_run(const std::string& key, std::uint64_t deadline) {
    if (key.empty() || deadline <= api::host_monotonic_ns()) { reason_ = "rt_core_run_expired_or_invalid"; return false; }
    if (key == run_key_) { run_deadline_ = std::min(run_deadline_, deadline); return true; }
    // Unverified capacity: fail closed after 256 retired runs per process.
    if (retired_runs_.count(key) || retired_runs_.size() >= 256 || !run_key_.empty()) {
      reason_ = "rt_core_run_retired_or_busy"; return false;
    }
    try {
      if (!retire_run("rt_core_run_acquire")) return false;
      acquire(); run_key_ = key; run_deadline_ = deadline;
      check(); reason_.clear(); refusal_ = {}; return true;
    } catch (const std::exception& e) { record(e); retire_run(reason_); return false; }
  }
  bool retire_run(const std::string& reason) {
    if (!run_key_.empty()) retired_runs_.insert(run_key_);
    run_key_.clear(); run_deadline_ = 0;
    const bool stopped = halt(reason);
    client_.stop_renewal();
    if (acquired_ && stopped) {
      try { client_.release(); acquired_ = false; retired_sessions_.insert(session_); }
      catch (const std::exception& e) { record_cleanup("release", e); return false; }
    }
    return stopped;
  }
  // Start at the valid held target to prevent a feedback offset becoming a
  // discontinuity. Two stationary Hermite knots bound speed and acceleration;
  // native admission remains authoritative for counts.
  bool position(std::size_t joint, double target, double speed, std::uint64_t deadline,
                const Authorization& authorized = {}) {
    if (run_key_.empty() || trajectory_ || jogging_ || !armed_) { reason_ = "rt_core_not_ready"; return false; }
    try {
      deadline = std::min(deadline, run_deadline_);
      check();
      api::Point first, last;
      if (status_.Core.Axes.size() != description_.Description.Axes.size())
        throw std::runtime_error("rt_core_held_target_unavailable");
      for (std::size_t i = 0; i < status_.Core.Axes.size(); ++i) {
        const auto& held = status_.Core.Axes[i];
        const auto& limits = description_.Description.Axes[i];
        if (!held.LogicalValid || !std::isfinite(held.LogicalTarget) ||
            held.LogicalTarget < limits.MinPosition || held.LogicalTarget > limits.MaxPosition)
          throw std::runtime_error("rt_core_held_target_unavailable");
        first.Position.push_back(held.LogicalTarget);
      }
      if (!ready() || joint >= indices_.size()) throw std::runtime_error("rt_core_not_ready");
      const auto index = indices_[joint];
      const auto& axis = description_.Description.Axes[index];
      if (!std::isfinite(target) || target < axis.MinPosition || target > axis.MaxPosition) {
        throw std::runtime_error("native_limit_exceeded");
      }
      if (!std::isfinite(speed) || speed < 0) throw std::runtime_error("rt_core_invalid_speed");
      const double cap = speed > 0 ? std::min(speed, axis.MaxVelocity) : axis.MaxVelocity;
      const double distance = std::abs(target - first.Position[index]);
      // Endpoint rounding can add at most one wire count to the displacement.
      const double bounded_distance = distance + (axis.CountsPerUnit > 0 ? 1 / axis.CountsPerUnit : 0);
      // Exact cubic Hermite bounds: max speed = 1.5*d/T, max acceleration = 6*d/T^2.
      double seconds = 1.5 * bounded_distance / cap;
      if (axis.MaxAcceleration) seconds = std::max(seconds, std::sqrt(6 * bounded_distance / *axis.MaxAcceleration));
      // Unverified minimum knot interval, 10,000,000 ns; native admission still checks the grid.
      const double ns = std::max(10000000.0, std::ceil(seconds * 1e9) + 1);
      const auto now = api::host_monotonic_ns();
      if (!std::isfinite(ns) || deadline <= now || ns >= double(deadline - now))
        throw std::runtime_error("rt_core_position_deadline");
      first.Velocity.assign(first.Position.size(), 0); last = first;
      last.Time = static_cast<std::int64_t>(ns); last.Position[index] = target;
      const auto baseline = client_.events(0);
      if (baseline.AdapterIncarnation != adapter_) throw std::runtime_error("rt_core_event_identity_changed");
      event_cursor_ = baseline.LatestSequence;
      handle_ = native_sequence_ = 0; completion_event_ = false;
      execution_generation_ = 0;
      trajectory_identity_ = {};
      trajectory_identity_.PlanID = run_key_;
      trajectory_identity_.ProgramID = api::new_request_id();
      trajectory_identity_.PlanRevision = generation_;
      trajectory_identity_.ManifestRevision = config_.binding.Revision;
      require_authorized(authorized);
      const auto prepared = client_.prepare(mask_, {first, last}, trajectory_identity_, authorized);
      const auto echo = api::RtControlClient::data<api::HandleRecord>(prepared);
      handle_ = prepared.Handle;
      if (!handle_ || echo.Handle != handle_ || echo.Generation != generation_ ||
          echo.ExecutionGeneration != 0 || echo.State != "prepared" || !same_trajectory(echo.Identity))
        throw std::runtime_error("rt_core_prepared_identity_changed");
      check();
      const auto h = std::find_if(status_.Execution.Handles.begin(), status_.Execution.Handles.end(),
          [&](const auto& item) { return item.Handle == handle_ && item.Generation == generation_ &&
              item.ExecutionGeneration == 0 && item.State == "prepared" && same_trajectory(item.Identity); });
      if (!handle_ || h == status_.Execution.Handles.end() || api::host_monotonic_ns() >= deadline)
        throw std::runtime_error("rt_core_prepared_identity_changed");
      require_authorized(authorized);
      const auto started = client_.start_trajectory(handle_, trajectory_identity_, authorized);
      native_sequence_ = started.Sequence;
      require_authorized(authorized);
      const auto running = api::RtControlClient::data<api::HandleRecord>(started);
      execution_generation_ = running.ExecutionGeneration;
      if (!native_sequence_ || !execution_generation_ || running.Handle != handle_ || running.Generation != generation_ ||
          running.NativeSequence != native_sequence_ || !same_trajectory(running.Identity) ||
          (running.State != "started" && running.State != "consumed"))
        throw std::runtime_error("rt_core_started_identity_changed");
      check();
      trajectory_ = true; return true;
    } catch (const std::exception& e) { record(e); retire_run(reason_); return false; }
  }
  bool position_complete() {
    if (!trajectory_) return false;
    try {
      check();
      auto batch = client_.events(event_cursor_);
      if (batch.AdapterIncarnation != adapter_) throw std::runtime_error("rt_core_event_identity_changed");
      auto cursor = event_cursor_;
      for (const auto& e : batch.Events) {
        if (e.Dropped || e.DaemonIncarnation != daemon_ || e.AdapterIncarnation != adapter_)
          throw std::runtime_error("rt_core_event_history_lost");
        if (cursor == UINT64_MAX || e.Sequence != cursor + 1 || e.Sequence > batch.LatestSequence)
          throw std::runtime_error("rt_core_event_cursor_replayed");
        cursor = e.Sequence;
        if (e.Handle && e.Handle->Handle == handle_ && e.Handle->NativeSequence == native_sequence_ &&
            e.Handle->To == "consumed" && e.Handle->Generation == generation_ &&
            e.Handle->ExecutionGeneration == execution_generation_ && same_trajectory(e.Handle->Identity))
          completion_event_ = true;
      }
      if (batch.NextSequence != cursor || batch.LatestSequence < cursor)
        throw std::runtime_error("rt_core_event_cursor_replayed");
      event_cursor_ = cursor;
      const auto current = std::find_if(status_.Execution.Handles.begin(), status_.Execution.Handles.end(),
          [&](const auto& h) { return h.Handle == handle_ && h.Generation == generation_ &&
              h.ExecutionGeneration == execution_generation_ && h.NativeSequence == native_sequence_ && same_trajectory(h.Identity); });
      if (current == status_.Execution.Handles.end()) throw std::runtime_error("rt_core_started_identity_changed");
      const auto& m = status_.Motion;
      // Protocol completion state 4 is native trajectory Done, not queue acceptance.
      if (completion_event_ && m.Done && m.State == 4 && m.TrajectoryID == handle_ && m.CommandSequence == native_sequence_) {
        trajectory_ = false; return true;
      }
      if (m.Done && m.TrajectoryID == handle_ && m.CommandSequence == native_sequence_ && m.State != 4)
        throw std::runtime_error("rt_core_position_failed");
    } catch (const std::exception& e) { record(e); retire_run(reason_); }
    return false;
  }
  const std::string& run_key() const { return run_key_; }
  std::uint64_t run_deadline() const { return run_deadline_; }
  const std::string& session() const { return session_; }
  std::uint64_t generation() const { return generation_; }
  std::uint64_t handle() const { return handle_; }
  std::uint64_t native_sequence() const { return native_sequence_; }
  std::uint64_t execution_generation() const { return execution_generation_; }
  const api::Identity& trajectory_identity() const { return trajectory_identity_; }
  const api::Binding& binding() const { return config_.binding; }
  void shutdown() {
    if (closed_) return;
    halt("shutdown");
    closed_ = true;
    client_.stop_renewal();
    if (acquired_ && stop_confirmed_) {
      try { client_.release(); acquired_ = false; }
      catch (const std::exception& e) { record_cleanup("release", e); }
    }
  }
  bool armed() const { return armed_; }
  bool motion_ready() const { return armed_ && ready(); }
  bool home_confirmed() const { return home_confirmed_; }
  bool jogging() const { return jogging_; }
  bool stop_confirmed() const { return stop_confirmed_; }
  const api::ProcessStatus& status() const { return status_; }
  const api::Description& description() const { return description_; }
  const std::string& reason() const { return reason_; }
  const api::Json& refusal() const { return refusal_; }
  const std::vector<double>& applied() const { return applied_; }
  std::array<double, 6> positions() const {
    std::array<double, 6> q{};
    for (std::size_t i = 0; i < q.size(); ++i) {
      if (indices_[i] >= status_.Core.Axes.size() || !status_.Core.Axes[indices_[i]].LogicalValid)
        throw std::runtime_error("rt_core_position_unavailable");
      q[i] = status_.Core.Axes[indices_[i]].LogicalPosition;
    }
    return q;
  }
};
}  // namespace cartesian_rt
