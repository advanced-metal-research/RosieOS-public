#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Motion-server-owned controller leader lease. This state is intentionally
// process-local and starts empty on every daemon boot. A lease is only granted
// after the control loop completes the revoke-first transition; clients never
// supply an epoch or lease ID during acquisition. Steam Deck remains the
// compatibility-default controller role. Other roles, including
// offline-programming, must be explicitly admitted by server configuration.
struct LeaderLeaseRequest {
  std::string command;
  std::string sender_id;
  std::string controller_boot_id;
  std::string request_id;
  std::string lease_id;
  std::uint64_t fence_epoch = 0;
  std::uint64_t campaign_generation = 0;
  int ttl_ms = 0;
};

struct LeaderLeaseCredential {
  std::string controller_boot_id;
  std::string lease_id;
  std::uint64_t fence_epoch = 0;
};

// Source-clock permit projected onto an already-issued leader lease. This is
// not a second lease issuer; the bridge remains the source of run admission.
struct BridgePositionPermit {
  std::string challenge_id, bridge_boot_id, run_id, controller_id, controller_boot_id;
  std::string command_id_json, leader_lease_id, pair_config_sha256;
  std::uint64_t leader_fence_epoch = 0, bridge_observed_monotonic_ms = 0;
  std::uint64_t deadline_monotonic_ms = 0, remaining_ms = 0;
  std::uint64_t run_deadline_monotonic_ms = 0, run_remaining_ms = 0;
  std::uint64_t runtime_mode_epoch = 0, activation_epoch = 0, home_epoch = 0, pair_revision = 0;
};

struct LeaderLeaseStatus {
  std::string leader_id = "none";
  std::string leader_boot_id = "none";
  std::string leader_lease_id = "none";
  std::uint64_t leader_fence_epoch = 0;
  std::uint64_t leader_campaign_generation = 0;
  bool leader_lease_fresh = false;
  int leader_expires_in_ms = 0;
  bool leader_transition_pending = false;
  std::string latest_leader_result = "none";
  std::string latest_leader_request_id = "none";
  std::uint64_t latest_leader_campaign_generation = 0;
};

enum class LeaderLeaseTransition {
  kNone,
  kRevoke,
  kRevokeThenAcquire,
};

struct LeaderLeaseAction {
  bool accepted = false;
  LeaderLeaseTransition transition = LeaderLeaseTransition::kNone;
  std::string result = "rejected";
};

class ControllerLeaderAuthority {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit ControllerLeaderAuthority(
      std::uint64_t server_epoch_seed,
      std::uint64_t injected_lease_nonce = 0)
      : ControllerLeaderAuthority(
            server_epoch_seed, {"steamdeck"}, injected_lease_nonce) {}

  ControllerLeaderAuthority(
      std::uint64_t server_epoch_seed,
      std::vector<std::string> allowed_controller_roles,
      std::uint64_t injected_lease_nonce = 0)
      : next_fence_epoch_(server_epoch_seed == 0 ? 1 : server_epoch_seed),
        lease_nonce_(injected_lease_nonce == 0
                         ? random_lease_nonce(server_epoch_seed)
                         : injected_lease_nonce),
        allowed_controller_roles_(
            allowed_controller_roles.begin(),
            allowed_controller_roles.end()) {}

  std::string begin_bridge_position_challenge(BridgePositionPermit expected, TimePoint now) {
    if (expected.bridge_boot_id.empty() || expected.run_id.empty() || expected.command_id_json.empty() ||
        expected.deadline_monotonic_ms == 0 || expected.run_deadline_monotonic_ms < expected.deadline_monotonic_ms ||
        !authorizes(expected.controller_id, {expected.controller_boot_id, expected.leader_lease_id,
                     expected.leader_fence_epoch}, now)) return {};
    expected.challenge_id = "position-" +
        std::to_string(random_lease_nonce(++bridge_challenge_sequence_));
    pending_bridge_permit_ = std::move(expected);
    bridge_challenge_sent_at_ = now;
    return pending_bridge_permit_.challenge_id;
  }

  bool accept_bridge_position_permit(const BridgePositionPermit& reply, TimePoint now) {
    const auto& expected = pending_bridge_permit_;
    if (expected.challenge_id.empty() || reply.challenge_id != expected.challenge_id ||
        reply.bridge_boot_id != expected.bridge_boot_id || reply.run_id != expected.run_id ||
        reply.controller_id != expected.controller_id || reply.controller_boot_id != expected.controller_boot_id ||
        reply.command_id_json != expected.command_id_json || reply.leader_lease_id != expected.leader_lease_id ||
        reply.leader_fence_epoch != expected.leader_fence_epoch ||
        reply.runtime_mode_epoch != expected.runtime_mode_epoch || reply.activation_epoch != expected.activation_epoch ||
        reply.home_epoch != expected.home_epoch || reply.pair_revision != expected.pair_revision ||
        reply.pair_config_sha256 != expected.pair_config_sha256 ||
        reply.deadline_monotonic_ms != expected.deadline_monotonic_ms ||
        reply.run_deadline_monotonic_ms != expected.run_deadline_monotonic_ms ||
        reply.run_deadline_monotonic_ms < reply.deadline_monotonic_ms ||
        reply.run_remaining_ms != reply.run_deadline_monotonic_ms - reply.bridge_observed_monotonic_ms ||
        reply.bridge_observed_monotonic_ms >= reply.deadline_monotonic_ms ||
        reply.remaining_ms != reply.deadline_monotonic_ms - reply.bridge_observed_monotonic_ms ||
        reply.remaining_ms > static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(TimePoint::max() - bridge_challenge_sent_at_).count()) ||
        reply.run_remaining_ms > static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(TimePoint::max() - bridge_challenge_sent_at_).count()) ||
        !authorizes(reply.controller_id, {reply.controller_boot_id, reply.leader_lease_id,
                     reply.leader_fence_epoch}, now)) return false;
    // Send-time anchoring is conservative under transport delay. Receipt-time
    // anchoring would incorrectly add the response's network delay to authority.
    const auto deadline = bridge_challenge_sent_at_ + std::chrono::milliseconds(reply.remaining_ms);
    if (now >= deadline) return false;
    bridge_permit_ = reply;
    bridge_bound_run_ = reply;
    bridge_run_bound_ = true;
    bridge_permit_deadline_ = deadline;
    bridge_run_deadline_ = bridge_challenge_sent_at_ + std::chrono::milliseconds(reply.run_remaining_ms);
    pending_bridge_permit_ = {};
    return true;
  }

  bool authorizes_bridge_position(const std::string& run_id, const std::string& command_id_json, TimePoint now) const {
    return !bridge_permit_.challenge_id.empty() && run_id == bridge_permit_.run_id &&
        command_id_json == bridge_permit_.command_id_json && now < bridge_permit_deadline_ &&
        authorizes(bridge_permit_.controller_id, {bridge_permit_.controller_boot_id,
            bridge_permit_.leader_lease_id, bridge_permit_.leader_fence_epoch}, now);
  }

  bool bridge_position_authority_matches(const std::string& run_id, const std::string& command_id_json,
      TimePoint now, std::uint64_t runtime_epoch, std::uint64_t activation_epoch,
      std::uint64_t home_epoch, std::uint64_t pair_revision, const std::string& pair_digest) const {
    return authorizes_bridge_position(run_id, command_id_json, now) &&
        bridge_permit_.runtime_mode_epoch == runtime_epoch && bridge_permit_.activation_epoch == activation_epoch &&
        bridge_permit_.home_epoch == home_epoch && bridge_permit_.pair_revision == pair_revision &&
        bridge_permit_.pair_config_sha256 == pair_digest;
  }

  LeaderLeaseAction revoke_bridge_run(const std::string& bridge_boot_id, const std::string& run_id, TimePoint now) {
    const bool pending = pending_bridge_permit_.bridge_boot_id == bridge_boot_id && pending_bridge_permit_.run_id == run_id;
    const bool active = bridge_bound_run_.bridge_boot_id == bridge_boot_id && bridge_bound_run_.run_id == run_id;
    if (bridge_boot_id.empty() || run_id.empty() || (!pending && !active)) return {};
    pending_bridge_permit_ = {};
    bridge_permit_ = {};
    return begin_expiry(now, true); // Caller uses the existing revoke/output transition.
  }

  LeaderLeaseAction observe_bridge_boot(const std::string& authoritative_boot_id, TimePoint now) {
    const auto& bound = bridge_bound_run_.run_id.empty() ? pending_bridge_permit_ : bridge_bound_run_;
    if (bound.run_id.empty() || authoritative_boot_id.empty() || bound.bridge_boot_id == authoritative_boot_id) return {};
    const std::string prior_boot = bound.bridge_boot_id, prior_run = bound.run_id;
    return revoke_bridge_run(prior_boot, prior_run, now);
  }

  void cancel_bridge_position() { pending_bridge_permit_ = {}; bridge_permit_ = {}; }
  bool bridge_run_bound() const { return bridge_run_bound_ || !pending_bridge_permit_.challenge_id.empty(); }

  bool matches_bridge_run(const std::string& run_id, const std::string& controller_id,
                          const std::string& controller_boot_id, TimePoint now) const {
    return bridge_run_bound_ && now < bridge_run_deadline_ && bridge_bound_run_.run_id == run_id && bridge_bound_run_.controller_id == controller_id &&
        bridge_bound_run_.controller_boot_id == controller_boot_id &&
        authorizes(controller_id, {controller_boot_id, bridge_bound_run_.leader_lease_id,
                    bridge_bound_run_.leader_fence_epoch}, now);
  }

  bool matches_bridge_jog(const std::string& controller_id, const std::string& lease_id,
                          std::uint64_t fence_epoch, TimePoint now) const {
    return bridge_run_bound_ && now < bridge_run_deadline_ && bridge_bound_run_.controller_id == controller_id &&
        bridge_bound_run_.leader_lease_id == lease_id && bridge_bound_run_.leader_fence_epoch == fence_epoch &&
        authorizes_hot(controller_id, lease_id, fence_epoch, now);
  }

  LeaderLeaseAction revoke_bridge_transport(TimePoint now) {
    const auto bound = bridge_bound_run_.run_id.empty() ? pending_bridge_permit_ : bridge_bound_run_;
    return revoke_bridge_run(bound.bridge_boot_id, bound.run_id, now);
  }

  LeaderLeaseAction observe_bridge_end(const std::string& run_id, const std::string& controller_id,
                                       const std::string& controller_boot_id, TimePoint now) {
    const auto bound = bridge_bound_run_.run_id.empty() ? pending_bridge_permit_ : bridge_bound_run_;
    if (bound.run_id != run_id || bound.controller_id != controller_id || bound.controller_boot_id != controller_boot_id) return {};
    return revoke_bridge_run(bound.bridge_boot_id, bound.run_id, now);
  }

  LeaderLeaseAction begin_request(const LeaderLeaseRequest& request,
                                  TimePoint now) {
    if (!valid_request_identity(request)) {
      return reject(request.request_id, request.campaign_generation,
                    "invalid_leader_identity");
    }
    // Cancel is the safety barrier for a client that stops wanting authority
    // after its acquire PUB/PONG has already entered the broker. It must not
    // wait behind an in-flight revoke/acquire transition. The bounded
    // high-water tombstone also rejects an older acquire that is delivered
    // after this cancel on a different NATS connection.
    if (request.command == "leader_cancel") {
      if (!request.lease_id.empty() || request.fence_epoch != 0 ||
          request.ttl_ms != 0) {
        return reject(request.request_id, request.campaign_generation,
                      "client_fence_not_accepted");
      }
      remember_cancel(request);
      const bool cancels_pending =
          pending_acquire_.command == "leader_acquire" &&
          cancel_covers_campaign(pending_acquire_, request);
      const bool cancels_active =
          active_ && request.sender_id == leader_id_ &&
          request.controller_boot_id == leader_boot_id_ &&
          leader_campaign_generation_ <= request.campaign_generation;
      if (cancels_active) {
        clear_active();
      }
      if (cancels_pending) {
        pending_acquire_ = {};
      }
      latest_request_id_ = request.request_id;
      latest_campaign_generation_ = request.campaign_generation;
      if (cancels_active || cancels_pending) {
        transition_pending_ = true;
        pending_result_ = "cancelled";
        pending_request_id_ = request.request_id;
        pending_campaign_generation_ = request.campaign_generation;
        latest_result_ = "cancel_revoke_pending";
        return {true, LeaderLeaseTransition::kRevoke, latest_result_};
      }
      latest_result_ = "cancelled";
      return {true, LeaderLeaseTransition::kNone, latest_result_};
    }
    if (transition_pending_) {
      return reject_terminal_acquire_if_needed(
          request, "leader_transition_pending");
    }
    if (request.command == "leader_acquire") {
      if (!request.lease_id.empty() || request.fence_epoch != 0 ||
          request.ttl_ms <= 0) {
        return reject_terminal_acquire_if_needed(
            request, "client_fence_not_accepted");
      }
      if (cancelled_by_generation_barrier(request)) {
        return reject(request.request_id, request.campaign_generation,
                      "cancelled");
      }
      // A lost status datagram must not make the current controller revoke
      // itself when it retries the same acquire identity. This is renew-like
      // idempotence; a different sender or controller boot still preempts.
      if (active_ && request.sender_id == leader_id_ &&
          request.controller_boot_id == leader_boot_id_ &&
          request.campaign_generation < leader_campaign_generation_ &&
          now < lease_deadline_) {
        return reject_terminal_acquire_if_needed(
            request, "stale_leader_generation");
      }
      if (active_ && request.sender_id == leader_id_ &&
          request.controller_boot_id == leader_boot_id_ &&
          request.campaign_generation == leader_campaign_generation_ &&
          now < lease_deadline_) {
        lease_deadline_ =
            now + std::chrono::milliseconds(clamp_ttl_ms(request.ttl_ms));
        latest_result_ = "acquired_existing";
        latest_request_id_ = request.request_id;
        latest_campaign_generation_ = request.campaign_generation;
        return {true, LeaderLeaseTransition::kNone, latest_result_};
      }
      const bool preempting = active_;
      if (preempting) {
        remember_cancel_identity(
            leader_id_, leader_boot_id_,
            leader_campaign_generation_);
      }
      clear_active();
      transition_pending_ = true;
      pending_acquire_ = request;
      pending_acquire_.ttl_ms = clamp_ttl_ms(request.ttl_ms);
      pending_result_ =
          preempting ? "preempted_and_acquired" : "acquired";
      pending_request_id_ = request.request_id;
      pending_campaign_generation_ = request.campaign_generation;
      latest_result_ =
          preempting ? "preempt_revoke_pending" : "acquire_revoke_pending";
      latest_request_id_ = request.request_id;
      latest_campaign_generation_ = request.campaign_generation;
      return {true, LeaderLeaseTransition::kRevokeThenAcquire,
              latest_result_};
    }
    if (request.command == "leader_renew") {
      if (request.ttl_ms <= 0 ||
          request.campaign_generation != leader_campaign_generation_ ||
          !authorizes(request.sender_id,
                      {request.controller_boot_id, request.lease_id,
                       request.fence_epoch},
                      now)) {
        return reject(request.request_id, request.campaign_generation,
                      "stale_or_foreign_leader_renew");
      }
      lease_deadline_ =
          now + std::chrono::milliseconds(clamp_ttl_ms(request.ttl_ms));
      latest_result_ = "renewed";
      latest_request_id_ = request.request_id;
      latest_campaign_generation_ = request.campaign_generation;
      return {true, LeaderLeaseTransition::kNone, latest_result_};
    }
    if (request.command == "leader_release") {
      if (request.campaign_generation != leader_campaign_generation_ ||
          !authorizes(request.sender_id,
                      {request.controller_boot_id, request.lease_id,
                       request.fence_epoch},
                      now)) {
        return reject(request.request_id, request.campaign_generation,
                      "stale_or_foreign_leader_release");
      }
      remember_cancel_identity(
          leader_id_, leader_boot_id_,
          leader_campaign_generation_);
      clear_active();
      transition_pending_ = true;
      pending_acquire_ = {};
      pending_result_ = "released";
      pending_request_id_ = request.request_id;
      pending_campaign_generation_ = request.campaign_generation;
      latest_result_ = "release_revoke_pending";
      latest_request_id_ = request.request_id;
      latest_campaign_generation_ = request.campaign_generation;
      return {true, LeaderLeaseTransition::kRevoke, latest_result_};
    }
    return reject(request.request_id, request.campaign_generation,
                  "unknown_leader_command");
  }

  bool expiry_due(TimePoint now) const {
    return active_ && (now >= lease_deadline_ || (bridge_run_bound_ && now >= bridge_run_deadline_));
  }

  LeaderLeaseRequest active_campaign_identity() const {
    LeaderLeaseRequest out;
    if (!active_) {
      return out;
    }
    out.command = "leader_acquire";
    out.sender_id = leader_id_;
    out.controller_boot_id = leader_boot_id_;
    out.campaign_generation = leader_campaign_generation_;
    return out;
  }

  LeaderLeaseAction begin_expiry(TimePoint now, bool force = false) {
    if (transition_pending_ || (!force && !expiry_due(now))) {
      return {false, LeaderLeaseTransition::kNone, "no_expiry_due"};
    }
    const std::uint64_t expired_generation = leader_campaign_generation_;
    remember_cancel_identity(
        leader_id_, leader_boot_id_,
        leader_campaign_generation_);
    clear_active();
    transition_pending_ = true;
    pending_acquire_ = {};
    pending_result_ = "expired";
    pending_request_id_ = "lease-expiry";
    pending_campaign_generation_ = expired_generation;
    latest_result_ = "expiry_revoke_pending";
    latest_request_id_ = "lease-expiry";
    latest_campaign_generation_ = expired_generation;
    return {true, LeaderLeaseTransition::kRevoke, latest_result_};
  }

  void finish_transition(bool safe_actions_issued, TimePoint now) {
    if (!transition_pending_) {
      return;
    }
    if (!safe_actions_issued) {
      if (pending_acquire_.command == "leader_acquire") {
        remember_cancel(pending_acquire_);
      }
      clear_active();
      pending_acquire_ = {};
      transition_pending_ = false;
      latest_result_ = "revoke_safety_issue_failed";
      if (!pending_request_id_.empty()) {
        latest_request_id_ = pending_request_id_;
        latest_campaign_generation_ = pending_campaign_generation_;
      }
      pending_result_.clear();
      pending_request_id_.clear();
      pending_campaign_generation_ = 0;
      return;
    }
    if (pending_acquire_.command == "leader_acquire" &&
        cancelled_by_generation_barrier(pending_acquire_)) {
      pending_result_ = "cancelled";
    } else if (pending_acquire_.command == "leader_acquire") {
      active_ = true;
      leader_id_ = pending_acquire_.sender_id;
      leader_boot_id_ = pending_acquire_.controller_boot_id;
      leader_campaign_generation_ =
          pending_acquire_.campaign_generation;
      leader_fence_epoch_ = allocate_fence_epoch();
      leader_lease_id_ = make_lease_id(leader_fence_epoch_);
      lease_deadline_ =
          now + std::chrono::milliseconds(pending_acquire_.ttl_ms);
    }
    latest_result_ =
        pending_result_.empty() ? "revoked" : pending_result_;
    if (!pending_request_id_.empty()) {
      latest_request_id_ = pending_request_id_;
      latest_campaign_generation_ = pending_campaign_generation_;
    }
    pending_acquire_ = {};
    pending_result_.clear();
    pending_request_id_.clear();
    pending_campaign_generation_ = 0;
    transition_pending_ = false;
  }

  bool authorizes_hot(const std::string& sender_id,
                      const std::string& lease_id,
                      std::uint64_t fence_epoch,
                      TimePoint now) const {
    return active_ && !transition_pending_ && now < lease_deadline_ &&
           (!bridge_run_bound_ || now < bridge_run_deadline_) &&
           sender_id == leader_id_ && !lease_id.empty() &&
           lease_id == leader_lease_id_ && fence_epoch != 0 &&
           fence_epoch == leader_fence_epoch_;
  }

  bool authorizes(const std::string& sender_id,
                  const LeaderLeaseCredential& credential,
                  TimePoint now) const {
    return authorizes_hot(sender_id, credential.lease_id,
                          credential.fence_epoch, now) &&
           !credential.controller_boot_id.empty() &&
           credential.controller_boot_id == leader_boot_id_;
  }

  LeaderLeaseStatus status(TimePoint now) const {
    LeaderLeaseStatus out;
    out.leader_transition_pending = transition_pending_;
    out.latest_leader_result = latest_result_;
    out.latest_leader_request_id = latest_request_id_;
    out.latest_leader_campaign_generation =
        latest_campaign_generation_;
    if (!active_ || transition_pending_ || now >= lease_deadline_ ||
        (bridge_run_bound_ && now >= bridge_run_deadline_)) {
      return out;
    }
    out.leader_id = leader_id_;
    out.leader_boot_id = leader_boot_id_;
    out.leader_lease_id = leader_lease_id_;
    out.leader_fence_epoch = leader_fence_epoch_;
    out.leader_campaign_generation = leader_campaign_generation_;
    out.leader_lease_fresh = true;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        lease_deadline_ - now);
    out.leader_expires_in_ms =
        static_cast<int>(std::max<std::int64_t>(1, remaining.count()));
    return out;
  }

  void record_rejection(const std::string& request_id,
                        const std::string& result,
                        std::uint64_t campaign_generation = 0) {
    (void)reject(request_id, campaign_generation, result);
  }

  void record_terminal_acquire_rejection(
      const LeaderLeaseRequest& request,
      const std::string& result) {
    if (request.command == "leader_acquire" &&
        valid_request_identity(request)) {
      remember_cancel(request);
    }
    (void)reject(request.request_id, request.campaign_generation, result);
  }

  std::size_t cancel_tombstone_count() const noexcept {
    return cancel_tombstones_.size();
  }

  static int clamp_ttl_ms(int ttl_ms) {
    return std::max(500, std::min(2000, ttl_ms));
  }

  static bool valid_controller_role(const std::string& role) {
    if (role.empty() || role.size() > 31 ||
        role.front() < 'a' || role.front() > 'z') {
      return false;
    }
    for (unsigned char ch : role) {
      if ((ch < 'a' || ch > 'z') &&
          (ch < '0' || ch > '9') && ch != '-') {
        return false;
      }
    }
    return true;
  }

  static std::string controller_role_from_sender(
      const std::string& sender_id) {
    const std::size_t separator = sender_id.find(':');
    if (separator == std::string::npos ||
        sender_id.find(':', separator + 1) != std::string::npos) {
      return "";
    }
    const std::string role = sender_id.substr(0, separator);
    return valid_controller_role(role) ? role : "";
  }

  static bool concrete_controller_sender(const std::string& sender_id) {
    if (sender_id.empty() || sender_id.size() > 63) {
      return false;
    }
    const std::string role = controller_role_from_sender(sender_id);
    if (role.empty()) {
      return false;
    }
    const std::string instance =
        sender_id.substr(role.size() + 1);
    if (instance.empty() || instance.size() > 31 ||
        instance.front() < 'a' || instance.front() > 'z') {
      return false;
    }
    for (unsigned char ch : instance) {
      if ((ch < 'a' || ch > 'z') &&
          (ch < '0' || ch > '9') &&
          ch != '-' && ch != '_' && ch != '.') {
        return false;
      }
    }
    if (role == "steamdeck") {
      const std::string prefix = "steamdeck-";
      if (instance.size() <= prefix.size() ||
          instance.compare(0, prefix.size(), prefix) != 0) {
        return false;
      }
    }
    return true;
  }

  // Compatibility helper retained for existing Steam Deck source/tests.
  static bool concrete_steamdeck_sender(const std::string& sender_id) {
    return controller_role_from_sender(sender_id) == "steamdeck" &&
           concrete_controller_sender(sender_id);
  }

  bool controller_sender_allowlisted(
      const std::string& sender_id) const {
    if (!concrete_controller_sender(sender_id)) {
      return false;
    }
    return allowed_controller_roles_.count(
               controller_role_from_sender(sender_id)) != 0;
  }

 private:
  static bool safe_ascii_id(const std::string& value,
                            std::size_t max_size) {
    if (value.empty() || value.size() > max_size) {
      return false;
    }
    for (unsigned char ch : value) {
      if (ch < 0x21 || ch > 0x7e) {
        return false;
      }
    }
    return true;
  }

  bool valid_request_identity(const LeaderLeaseRequest& request) const {
    return controller_sender_allowlisted(request.sender_id) &&
           safe_ascii_id(request.controller_boot_id, 63) &&
           safe_ascii_id(request.request_id, 63) &&
           request.campaign_generation != 0;
  }

  static bool cancel_covers_campaign(
      const LeaderLeaseRequest& candidate,
      const LeaderLeaseRequest& cancel_request) {
    return candidate.sender_id == cancel_request.sender_id &&
           candidate.controller_boot_id ==
               cancel_request.controller_boot_id &&
           candidate.campaign_generation <=
               cancel_request.campaign_generation;
  }

  static std::uint64_t random_lease_nonce(std::uint64_t seed) {
    std::random_device source;
    const std::uint64_t random_high =
        static_cast<std::uint64_t>(source()) << 32u;
    const std::uint64_t random_low =
        static_cast<std::uint64_t>(source());
    const std::uint64_t nonce = random_high ^ random_low ^ seed;
    return nonce == 0 ? 1 : nonce;
  }

  LeaderLeaseAction reject(const std::string& request_id,
                           std::uint64_t campaign_generation,
                           const std::string& result) {
    // A malformed legacy envelope has no generation with which a client can
    // safely correlate the outcome. Keep the last coherent status unchanged;
    // valid authority requests always carry a nonzero generation.
    if (campaign_generation == 0) {
      return {false, LeaderLeaseTransition::kNone, result};
    }
    latest_result_ = result;
    latest_request_id_ =
        request_id.empty() ? "missing-request-id" : request_id;
    latest_campaign_generation_ = campaign_generation;
    return {false, LeaderLeaseTransition::kNone, result};
  }

  LeaderLeaseAction reject_terminal_acquire_if_needed(
      const LeaderLeaseRequest& request,
      const std::string& result) {
    if (request.command == "leader_acquire") {
      remember_cancel(request);
    }
    return reject(request.request_id, request.campaign_generation, result);
  }

  struct LeaderCancelTombstone {
    std::string sender_id;
    std::string controller_boot_id;
    std::uint64_t high_water_generation = 0;
  };

  static constexpr std::size_t kMaxCancelTombstones = 128;

  bool cancelled_by_generation_barrier(
      const LeaderLeaseRequest& request) const {
    for (const LeaderCancelTombstone& tombstone : cancel_tombstones_) {
      if (tombstone.sender_id == request.sender_id &&
          tombstone.controller_boot_id == request.controller_boot_id) {
        return request.campaign_generation <=
               tombstone.high_water_generation;
      }
    }
    return false;
  }

  void remember_cancel(const LeaderLeaseRequest& request) {
    remember_cancel_identity(
        request.sender_id, request.controller_boot_id,
        request.campaign_generation);
  }

  void remember_cancel_identity(
      const std::string& sender_id,
      const std::string& controller_boot_id,
      std::uint64_t campaign_generation) {
    for (auto it = cancel_tombstones_.begin();
         it != cancel_tombstones_.end(); ++it) {
      if (it->sender_id != sender_id ||
          it->controller_boot_id != controller_boot_id) {
        continue;
      }
      LeaderCancelTombstone updated = std::move(*it);
      updated.high_water_generation =
          std::max(updated.high_water_generation,
                   campaign_generation);
      cancel_tombstones_.erase(it);
      cancel_tombstones_.push_back(std::move(updated));
      return;
    }
    if (cancel_tombstones_.size() == kMaxCancelTombstones) {
      cancel_tombstones_.pop_front();
    }
    cancel_tombstones_.push_back(
        {sender_id, controller_boot_id, campaign_generation});
  }

  void clear_active() {
    bridge_run_bound_ = false;
    bridge_bound_run_ = {};
    pending_bridge_permit_ = {};
    bridge_permit_ = {};
    active_ = false;
    leader_id_.clear();
    leader_boot_id_.clear();
    leader_lease_id_.clear();
    leader_fence_epoch_ = 0;
    leader_campaign_generation_ = 0;
    lease_deadline_ = TimePoint{};
  }

  std::uint64_t allocate_fence_epoch() {
    const std::uint64_t allocated = next_fence_epoch_;
    if (next_fence_epoch_ == std::numeric_limits<std::uint64_t>::max()) {
      next_fence_epoch_ = 1;
    } else {
      ++next_fence_epoch_;
    }
    return allocated == 0 ? allocate_fence_epoch() : allocated;
  }

  std::string make_lease_id(std::uint64_t fence_epoch) const {
    std::ostringstream out;
    out << "motion-server-" << std::hex << std::setfill('0')
        << std::setw(16) << lease_nonce_ << "-" << std::setw(16)
        << fence_epoch;
    return out.str();
  }

  bool active_ = false;
  bool transition_pending_ = false;
  std::string leader_id_;
  std::string leader_boot_id_;
  std::string leader_lease_id_;
  std::uint64_t leader_fence_epoch_ = 0;
  std::uint64_t leader_campaign_generation_ = 0;
  TimePoint lease_deadline_{};
  std::uint64_t next_fence_epoch_ = 1;
  std::uint64_t lease_nonce_ = 1;
  std::uint64_t bridge_challenge_sequence_ = 0;
  bool bridge_run_bound_ = false;
  BridgePositionPermit pending_bridge_permit_, bridge_permit_, bridge_bound_run_;
  TimePoint bridge_challenge_sent_at_{}, bridge_permit_deadline_{}, bridge_run_deadline_{};
  LeaderLeaseRequest pending_acquire_;
  std::string pending_result_;
  std::string pending_request_id_;
  std::uint64_t pending_campaign_generation_ = 0;
  std::string latest_result_ = "none";
  std::string latest_request_id_ = "none";
  std::uint64_t latest_campaign_generation_ = 0;
  std::deque<LeaderCancelTombstone> cancel_tombstones_;
  const std::set<std::string> allowed_controller_roles_;
};

using SteamDeckLeaderAuthority = ControllerLeaderAuthority;

inline bool leader_lease_command_kind(const std::string& kind) {
  return kind == "leader_acquire" || kind == "leader_renew" ||
         kind == "leader_release" || kind == "leader_cancel";
}
