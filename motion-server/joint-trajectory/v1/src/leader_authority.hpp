#pragma once

// Leader-lease authority, reimplemented to v1 semantics without sharing
// code: the daemon MINTS leases itself -- clients never supply an epoch or
// lease id -- and a motion command is authorized only by a "fresh exact"
// lease: active, unexpired, and sender_id / controller_boot_id / lease_id /
// fence_epoch all equal to the current holder's. Acquire is revoke-first:
// motion is stopped before a new fence epoch exists, so two controllers can
// never both believe they hold authority over a moving robot.

#include <cstdint>
#include <chrono>
#include <cstdio>
#include <functional>
#include <atomic>
#include <map>
#include <mutex>
#include <string>

#include "dense_joint_trajectory_format.hpp"

namespace dense_joint_trajectory {

// Default TTL clamp (v1's grant window: long enough to survive scheduling
// jitter, short enough that a vanished controller loses the cell within
// 2 s); deployments set the real values in the daemon config (`leader`).
inline constexpr std::uint64_t kLeaseTtlMinMs = 500;
inline constexpr std::uint64_t kLeaseTtlMaxMs = 2000;

struct LeaderCredential {
  bool valid = false;
  std::string controller_boot_id;
  std::string leader_lease_id;
  std::uint64_t leader_fence_epoch = 0;
};

inline LeaderCredential decode_leader_credential(const JsonValue& payload) {
  LeaderCredential credential;
  credential.valid =
      json_str(payload.find("controller_boot_id"), &credential.controller_boot_id) &&
      !credential.controller_boot_id.empty() &&
      json_str(payload.find("leader_lease_id"), &credential.leader_lease_id) &&
      !credential.leader_lease_id.empty() &&
      json_u64(payload.find("leader_fence_epoch"), &credential.leader_fence_epoch) &&
      credential.leader_fence_epoch != 0;
  return credential;
}

struct LeaseGrant {
  bool ok = false;
  std::string reason;         // rejection reason when !ok
  std::string lease_id;
  std::uint64_t fence_epoch = 0;
  std::uint64_t expires_in_ms = 0;
};

class LeaderAuthority {
 public:
  struct Snapshot {
    bool active = false;
    std::string leader_id;
    std::string lease_id;
    std::uint64_t fence_epoch = 0;
    std::int64_t deadline_ms = 0;
  };
  // role:instance. The only role that may lease this daemon is the pipeline
  // that produced the plan it plays -- offline-programming, the OLP server
  // that compiled the dense trajectory, loaded it, and owns the stop. The
  // pendant is not on this path (it commands cartesiand, motion-server/v1),
  // and a lease that named it would let a plan play under an identity that
  // never saw the plan. Instances are [A-Za-z0-9-]{1,31}; the bench runs as
  // offline-programming:bench-1 so the rule stays exercised.
  static bool sender_identity_valid(const std::string& sender_id) {
    const std::size_t colon = sender_id.find(':');
    if (colon == std::string::npos || colon == 0 ||
        colon + 1 >= sender_id.size()) {
      return false;
    }
    const std::string role = sender_id.substr(0, colon);
    const std::string instance = sender_id.substr(colon + 1);
    if (role != "offline-programming") return false;
    if (instance.empty() || instance.size() > 31) return false;
    for (const char c : instance) {
      const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                      (c >= 'A' && c <= 'Z') || c == '-';
      if (!ok) return false;
    }
    return true;
  }

  // Transition serialization keeps revoke-before-grant ordering. Readers see an
  // inactive lease during I/O; neither executor callbacks nor sockets hold mutex_.
  template <typename RevokeMotion>
  LeaseGrant acquire(const std::string& sender_id,
                     const std::string& controller_boot_id, std::uint64_t ttl_ms,
                     std::int64_t now_ms, RevokeMotion&& revoke_motion,
                     const std::function<std::string(const Snapshot&)>& acquired = {}) {
    const auto begun = std::chrono::steady_clock::now();
    LeaseGrant grant;
    if (!sender_identity_valid(sender_id) || controller_boot_id.empty()) {
      grant.reason = "leader_identity_invalid";
      return grant;
    }
    if (ttl_ms == 0) {
      grant.reason = "ttl_ms_required";
      return grant;
    }
    std::lock_guard<std::mutex> transition(transition_mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    active_ = false;
    published_epoch_ = 0;
    lock.unlock();
    revoke_motion();
    lock.lock();
    ttl_ms = clamp_ttl(ttl_ms);
    leader_id_ = sender_id;
    leader_boot_id_ = controller_boot_id;
    fence_epoch_ = next_fence_epoch_++;
    char lease[80];
    std::snprintf(lease, sizeof(lease), "motion-server-%016llx-%016llx",
                  static_cast<unsigned long long>(lease_nonce_),
                  static_cast<unsigned long long>(fence_epoch_));
    lease_id_ = lease;
    deadline_ms_ = now_ms + std::int64_t(ttl_ms);
    const Snapshot pending{true, leader_id_, lease_id_, fence_epoch_, deadline_ms_};
    lock.unlock();
    if (acquired) {
      grant.reason = acquired(pending);
    }
    lock.lock();
    if (!grant.reason.empty()) return grant;
    // The TTL starts at commit, so acquisition I/O cannot consume the new lease.
    deadline_ms_ = now_ms + std::int64_t(ttl_ms) +
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begun).count();
    active_ = true;
    published_deadline_.store(deadline_ms_);
    published_epoch_.store(fence_epoch_);
    grant.ok = true;
    grant.lease_id = lease_id_;
    grant.fence_epoch = fence_epoch_;
    grant.expires_in_ms = ttl_ms;
    return grant;
  }

  LeaseGrant renew(const std::string& sender_id,
                   const std::string& controller_boot_id,
                   const std::string& lease_id, std::uint64_t fence_epoch,
                   std::uint64_t ttl_ms, std::int64_t now_ms) {
    LeaseGrant grant;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!holder_matches(sender_id, controller_boot_id, lease_id, fence_epoch,
                        now_ms)) {
      grant.reason = "lease_not_held";
      return grant;
    }
    if (ttl_ms == 0) {
      grant.reason = "ttl_ms_required";
      return grant;
    }
    ttl_ms = clamp_ttl(ttl_ms);
    deadline_ms_ = now_ms + std::int64_t(ttl_ms);
    published_deadline_.store(deadline_ms_);
    grant.ok = true;
    grant.lease_id = lease_id_;
    grant.fence_epoch = fence_epoch_;
    grant.expires_in_ms = ttl_ms;
    return grant;
  }

  template <typename RevokeMotion>
  LeaseGrant release(const std::string& sender_id,
                     const std::string& controller_boot_id,
                     const std::string& lease_id, std::uint64_t fence_epoch,
                     std::int64_t now_ms, RevokeMotion&& revoke_motion) {
    LeaseGrant grant;
    std::lock_guard<std::mutex> transition(transition_mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    if (!holder_matches(sender_id, controller_boot_id, lease_id, fence_epoch,
                        now_ms)) {
      grant.reason = "lease_not_held";
      return grant;
    }
    active_ = false;
    published_epoch_ = 0;
    lock.unlock();
    revoke_motion();
    grant.ok = true;
    return grant;
  }

  bool authorizes(const std::string& sender_id,
                  const LeaderCredential& credential, std::int64_t now_ms) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return credential.valid && active_ && now_ms < deadline_ms_ &&
           sender_id == leader_id_ &&
           credential.leader_lease_id == lease_id_ &&
           credential.leader_fence_epoch == fence_epoch_ &&
           credential.controller_boot_id == leader_boot_id_;
  }

  bool lease_fresh(std::int64_t now_ms) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ && now_ms < deadline_ms_;
  }

  // Strictly-increasing motion_intent_seq per sender: a delayed duplicate or
  // reordered motion command can never re-execute.
  bool accept_motion_intent(const std::string& sender_id, std::uint64_t sequence) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& latest = latest_motion_intent_[sender_id];
    if (sequence <= latest) return false;
    latest = sequence;
    return true;
  }

  Snapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Snapshot{active_, leader_id_, lease_id_, fence_epoch_, deadline_ms_};
  }

  // Extends only the already-snapshotted epoch without taking the leader mutex.
  // Long native commissioning can observe renewals without lock inversion.
  std::int64_t renewed_deadline(const Snapshot& snapshot) const {
    if (published_epoch_.load() != snapshot.fence_epoch) return 0;
    const auto deadline = published_deadline_.load();
    return published_epoch_.load() == snapshot.fence_epoch ? deadline : 0;
  }

  explicit LeaderAuthority(std::uint64_t lease_nonce,
                           std::uint64_t ttl_min_ms = kLeaseTtlMinMs,
                           std::uint64_t ttl_max_ms = kLeaseTtlMaxMs)
      : ttl_min_ms_(ttl_min_ms), ttl_max_ms_(ttl_max_ms), lease_nonce_(lease_nonce) {}

 private:
  std::uint64_t clamp_ttl(std::uint64_t ttl_ms) const {
    if (ttl_ms < ttl_min_ms_) return ttl_min_ms_;
    if (ttl_ms > ttl_max_ms_) return ttl_max_ms_;
    return ttl_ms;
  }

  bool holder_matches(const std::string& sender_id,
                      const std::string& controller_boot_id,
                      const std::string& lease_id, std::uint64_t fence_epoch,
                      std::int64_t now_ms) const {
    return active_ && now_ms < deadline_ms_ && sender_id == leader_id_ &&
           controller_boot_id == leader_boot_id_ && lease_id == lease_id_ &&
           fence_epoch == fence_epoch_ && fence_epoch != 0;
  }

  mutable std::mutex mutex_;
  std::mutex transition_mutex_;
  bool active_ = false;
  std::string leader_id_;
  std::string leader_boot_id_;
  std::string lease_id_;
  std::uint64_t fence_epoch_ = 0;
  std::uint64_t next_fence_epoch_ = 1;
  std::int64_t deadline_ms_ = 0;
  std::atomic<std::int64_t> published_deadline_{0};
  std::atomic<std::uint64_t> published_epoch_{0};
  const std::uint64_t ttl_min_ms_;
  const std::uint64_t ttl_max_ms_;
  const std::uint64_t lease_nonce_;
  std::map<std::string, std::uint64_t> latest_motion_intent_;
};

}  // namespace dense_joint_trajectory
