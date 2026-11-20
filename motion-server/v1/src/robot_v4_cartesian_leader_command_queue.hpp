#pragma once

// Leader-aware replacement for the legacy queue. Revocation partitions the
// deque while holding the same mutex used by push/pop, so a concurrent NATS
// delivery cannot escape the purge window or reorder a preserved RT+A/STOP.
class LeaderAwareRobotCommandQueue {
 public:
  LeaderAwareRobotCommandQueue()
      : LeaderAwareRobotCommandQueue({"steamdeck"}) {}

  explicit LeaderAwareRobotCommandQueue(
      const std::vector<std::string>& allowed_controller_roles)
      : allowed_controller_roles_(
            allowed_controller_roles.begin(),
            allowed_controller_roles.end()) {}

  // Ordinary work can consume at most 48 slots. The remaining 16 are
  // reserved for fail-safe holds and authority transitions. A valid acquire
  // consumes two additional bounded reservation units, one for its matching
  // cancel and one for its matching release. Either exit command can
  // therefore replace its reservation and enter the queue even when all 64
  // capacity units are occupied. Once accepted, no message is ever evicted:
  // overflow is returned to the subscriber as a rejection.
  bool push(RobotCommandMessage message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool protected_message = preserves_control_safety(message);
    AuthorityCampaignKey authority_key;
    const bool valid_acquire =
        message.kind == "leader_acquire" &&
        parse_authority_campaign_key(message, &authority_key);
    const bool needs_exit_reservation =
        valid_acquire && !has_exit_reservation_locked(authority_key);
    const std::size_t required_capacity =
        1 + (needs_exit_reservation ? kExitSlotsPerAcquire : 0);

    AuthorityCampaignKey exit_key;
    const bool valid_exit =
        (message.kind == "leader_cancel" ||
         message.kind == "leader_release") &&
        parse_authority_campaign_key(message, &exit_key);
    const std::size_t consumed_exit_slots =
        valid_exit
            ? consume_matching_exit_slots_locked(message.kind, exit_key)
            : 0;
    if (consumed_exit_slots != 0) {
      insert_exit_barrier_locked(std::move(message));
      prune_empty_exit_reservations_locked();
      return true;
    }

    if (used_capacity_locked() + required_capacity > kCapacity ||
        (!protected_message && ordinary_count_ >= kOrdinaryCapacity)) {
      return false;
    }
    if (!protected_message) {
      ++ordinary_count_;
    }
    if (needs_exit_reservation) {
      exit_reservations_.push_back(
          {std::move(authority_key), true, true});
    }
    messages_.push_back(std::move(message));
    return true;
  }

  bool pop(RobotCommandMessage* out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (messages_.empty()) {
      return false;
    }
    *out = std::move(messages_.front());
    if (!preserves_control_safety(*out)) {
      --ordinary_count_;
    }
    messages_.pop_front();
    return true;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.empty();
  }

  std::size_t discard_energizing(
      const std::function<void(const RobotCommandMessage&)>& on_discard = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::deque<RobotCommandMessage> survivors;
    std::size_t discarded = 0;
    while (!messages_.empty()) {
      RobotCommandMessage message = std::move(messages_.front());
      messages_.pop_front();
      const bool fail_safe_path_hold =
          message.kind == "path_scrub" && message.has_direction &&
          message.direction == 0;
      const bool leadership_request =
          leader_lease_command_kind(message.kind);
      const bool explicit_fail_safe =
          message.kind == "stop" || fail_safe_path_hold;
      const bool energizing =
          message.kind == "arm" || message.kind == "position" || message.kind == "home" || message.kind == "go_home" ||
          message.kind == "plan_preload" ||
          message.kind == "play" || message.kind == "pause" ||
          message.kind == "move_to_row" ||
          (message.kind == "path_scrub" && !fail_safe_path_hold);
      if (energizing && !leadership_request && !explicit_fail_safe) {
        if (on_discard) on_discard(message);
        --ordinary_count_;
        ++discarded;
      } else {
        survivors.push_back(std::move(message));
      }
    }
    messages_.swap(survivors);
    return discarded;
  }

  // A reservation survives dequeue and a successful grant. The UDP loop
  // retires it only after the authority has installed a generation tombstone
  // for a terminal rejection, release, preemption, expiry, or revoke failure.
  // A queued matching cancel/release consumes its own slot before that point.
  void retire_exit_reservation(const std::string& sender_id,
                               const std::string& controller_boot_id,
                               std::uint64_t campaign_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = exit_reservations_.begin();
         it != exit_reservations_.end();) {
      if (it->key.sender_id == sender_id &&
          it->key.controller_boot_id == controller_boot_id &&
          it->key.campaign_generation == campaign_generation) {
        it = exit_reservations_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void retire_exit_reservations_covered_by(
      const std::string& sender_id,
      const std::string& controller_boot_id,
      std::uint64_t campaign_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = exit_reservations_.begin();
         it != exit_reservations_.end();) {
      if (it->key.sender_id == sender_id &&
          it->key.controller_boot_id == controller_boot_id &&
          it->key.campaign_generation <= campaign_generation) {
        it = exit_reservations_.erase(it);
      } else {
        ++it;
      }
    }
  }

 private:
  static constexpr std::size_t kCapacity = 64;
  static constexpr std::size_t kOrdinaryCapacity = 48;
  static constexpr std::size_t kExitSlotsPerAcquire = 2;

  struct AuthorityCampaignKey {
    std::string sender_id;
    std::string controller_boot_id;
    std::uint64_t campaign_generation = 0;
  };

  struct ExitReservation {
    AuthorityCampaignKey key;
    bool cancel_available = true;
    bool release_available = true;
  };

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

  bool parse_authority_campaign_key(
      const RobotCommandMessage& message,
      AuthorityCampaignKey* out) const {
    if (out == nullptr || message.raw_payload.empty()) {
      return false;
    }
    std::string command;
    std::string sender_id;
    std::string controller_boot_id;
    std::string request_id;
    std::uint64_t campaign_generation = 0;
    if (!parse_json_string(message.raw_payload, "command", &command) ||
        command != message.kind ||
        !parse_json_string(message.raw_payload, "sender_id", &sender_id) ||
        sender_id != message.sender_id ||
        !controller_sender_allowlisted(sender_id) ||
        !parse_json_string(message.raw_payload, "controller_boot_id",
                           &controller_boot_id) ||
        !safe_ascii_id(controller_boot_id, 63) ||
        !parse_json_string(message.raw_payload, "request_id", &request_id) ||
        !safe_ascii_id(request_id, 63) ||
        !parse_json_u64(message.raw_payload, "campaign_generation",
                        &campaign_generation) ||
        campaign_generation == 0) {
      return false;
    }
    if (command == "leader_acquire") {
      // Reserve from the immutable campaign identity even when the remaining
      // acquire envelope is malformed. The application loop will reject,
      // tombstone, and retire this reservation deterministically; accepting
      // the identity here prevents parser drift from recreating the original
      // ACKed-Acquire/blocked-Cancel hole.
    } else if (command == "leader_release") {
      std::string lease_id;
      std::uint64_t fence_epoch = 0;
      int unexpected_ttl_ms = 0;
      if (!parse_json_string(message.raw_payload, "lease_id", &lease_id) ||
          lease_id.empty() ||
          !parse_json_u64(message.raw_payload, "fence_epoch",
                          &fence_epoch) ||
          fence_epoch == 0 ||
          parse_json_int(message.raw_payload, "ttl_ms",
                         &unexpected_ttl_ms)) {
        return false;
      }
    } else if (command == "leader_cancel") {
      std::string unexpected_lease_id;
      std::uint64_t unexpected_fence_epoch = 0;
      int unexpected_ttl_ms = 0;
      if (parse_json_string(message.raw_payload, "lease_id",
                            &unexpected_lease_id) ||
          parse_json_u64(message.raw_payload, "fence_epoch",
                         &unexpected_fence_epoch) ||
          parse_json_int(message.raw_payload, "ttl_ms",
                         &unexpected_ttl_ms)) {
        return false;
      }
    } else {
      return false;
    }
    *out = {std::move(sender_id), std::move(controller_boot_id),
            campaign_generation};
    return true;
  }

  bool controller_sender_allowlisted(
      const std::string& sender_id) const {
    if (!ControllerLeaderAuthority::concrete_controller_sender(sender_id)) {
      return false;
    }
    return allowed_controller_roles_.count(
               ControllerLeaderAuthority::controller_role_from_sender(
                   sender_id)) != 0;
  }

  bool has_exit_reservation_locked(
      const AuthorityCampaignKey& candidate) const {
    for (const ExitReservation& reservation : exit_reservations_) {
      if (same_campaign(reservation.key, candidate)) {
        return true;
      }
    }
    return false;
  }

  static bool same_campaign(const AuthorityCampaignKey& left,
                            const AuthorityCampaignKey& right) {
    return left.sender_id == right.sender_id &&
           left.controller_boot_id == right.controller_boot_id &&
           left.campaign_generation == right.campaign_generation;
  }

  std::size_t consume_matching_exit_slots_locked(
      const std::string& command,
      const AuthorityCampaignKey& exit_key) {
    std::size_t consumed = 0;
    for (ExitReservation& reservation : exit_reservations_) {
      const bool same_controller =
          reservation.key.sender_id == exit_key.sender_id &&
          reservation.key.controller_boot_id ==
              exit_key.controller_boot_id;
      if (!same_controller) {
        continue;
      }
      if (command == "leader_cancel" &&
          reservation.cancel_available &&
          reservation.key.campaign_generation <=
              exit_key.campaign_generation) {
        reservation.cancel_available = false;
        ++consumed;
      } else if (command == "leader_release" &&
                 reservation.release_available &&
                 reservation.key.campaign_generation ==
                     exit_key.campaign_generation) {
        reservation.release_available = false;
        ++consumed;
      }
    }
    return consumed;
  }

  void insert_exit_barrier_locked(RobotCommandMessage message) {
    auto insertion = messages_.begin();
    while (insertion != messages_.end() &&
           (insertion->kind == "leader_cancel" ||
            insertion->kind == "leader_release")) {
      ++insertion;
    }
    messages_.insert(insertion, std::move(message));
  }

  void prune_empty_exit_reservations_locked() {
    for (auto it = exit_reservations_.begin();
         it != exit_reservations_.end();) {
      if (!it->cancel_available && !it->release_available) {
        it = exit_reservations_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::size_t used_capacity_locked() const {
    std::size_t used = messages_.size();
    for (const ExitReservation& reservation : exit_reservations_) {
      used += reservation.cancel_available ? 1 : 0;
      used += reservation.release_available ? 1 : 0;
    }
    return used;
  }

  static bool preserves_control_safety(
      const RobotCommandMessage& message) {
    const bool fail_safe_path_hold =
        message.kind == "path_scrub" && message.has_direction &&
        message.direction == 0;
    return leader_lease_command_kind(message.kind) ||
           message.kind == "stop" || fail_safe_path_hold;
  }

  mutable std::mutex mutex_;
  std::deque<RobotCommandMessage> messages_;
  std::deque<ExitReservation> exit_reservations_;
  std::size_t ordinary_count_ = 0;
  const std::set<std::string> allowed_controller_roles_;
};

// Process-local idempotency with explicit commit: a command ID is only
// remembered after bounded enqueue succeeds. A queue-overflow rejection can
// therefore be retried with the same immutable command ID.
class LeaderAwareCommandIDDeduper {
 public:
  struct PendingReply { std::string key, subject, body; };
  struct BridgeRequest {
    std::string subject, reply_subject, body;
    std::string reply_kind = "bridge_position_permit_reply";
  };

  void request_bridge(BridgeRequest request) {
    std::lock_guard<std::mutex> lock(mutex_);
    bridge_request_ = std::move(request);
  }
  BridgeRequest take_bridge_request() {
    std::lock_guard<std::mutex> lock(mutex_);
    BridgeRequest request = std::move(bridge_request_);
    bridge_request_ = {};
    return request;
  }
  bool contains(const std::string& command_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !command_id.empty() &&
           seen_.count(command_id) != 0;
  }

  void remember(const std::string& command_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    remember_locked(command_id);
  }

  void remember_position(const std::string& key, const std::string& reply_subject) {
    if (key.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    remember_locked(key);
    auto& result = position_results_[key];
    result.subject = reply_subject;
  }

  void complete_position(const std::string& key, const std::string& body) {
    if (key.empty() || body.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    remember_locked(key);
    auto& result = position_results_[key];
    if (result.body.empty()) result.body = body; // Terminal outcomes never mutate.
  }
  void acknowledge_command(const std::string& key, const std::string& body) {
    if (key.empty() || body.empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    remember_locked(key);
    auto& result = position_results_[key];
    if (result.acknowledgement.empty()) result.acknowledgement = body;
  }

  std::string position_result(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = position_results_.find(key);
    return found == position_results_.end() ? "" :
        (found->second.body.empty() ? found->second.acknowledgement : found->second.body);
  }

  std::vector<PendingReply> pending_replies() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PendingReply> replies;
    for (const auto& item : position_results_) {
      const auto& body = item.second.body.empty() ? item.second.acknowledgement : item.second.body;
      if (!item.second.subject.empty() && !body.empty())
        replies.push_back({item.first, item.second.subject, body});
    }
    return replies;
  }

  void delivered(const PendingReply& reply) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = position_results_.find(reply.key);
    if (found != position_results_.end() &&
        (found->second.body == reply.body ||
         (found->second.body.empty() && found->second.acknowledgement == reply.body)) &&
        found->second.subject == reply.subject) found->second.subject.clear();
  }

 private:
  void remember_locked(const std::string& command_id) {
    if (command_id.empty() || seen_.count(command_id) != 0) {
      return;
    }
    seen_.insert(command_id);
    order_.push_back(command_id);
    while (order_.size() > 256) {
      seen_.erase(order_.front());
      position_results_.erase(order_.front());
      order_.pop_front();
    }
  }

  struct PositionResult { std::string subject, body, acknowledgement; };
  mutable std::mutex mutex_;
  std::map<std::string, PositionResult> position_results_;
  BridgeRequest bridge_request_;
  std::set<std::string> seen_;
  std::deque<std::string> order_;
};
