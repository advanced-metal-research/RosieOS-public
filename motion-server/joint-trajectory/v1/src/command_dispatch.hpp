#pragma once

// robot.v4.robot-command.v1 dispatch: the NATS-side control plane. Motion
// authority deliberately stays here (not on the TCP ingest socket): play and
// pause require a fresh exact leader lease plus a strictly-increasing
// motion_intent_seq per sender, while stop is always accepted -- the
// fail-safe command must never be the one a fence can block.

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "execution_backend.hpp"
#include "dense_joint_trajectory_format.hpp"
#include "dense_joint_trajectory_ingest.hpp"
#include "leader_authority.hpp"
#include "nats_client.hpp"

namespace dense_joint_trajectory {

inline constexpr const char* kRobotCommandSchema = "robot.v4.robot-command.v1";
inline constexpr const char* kCommandReplySchema = "robot.v4.command-reply.v1";

// Idempotency: publishers resend for at-least-once delivery over the WiFi
// hop, and a reply can be lost after the command ran. A redelivered
// command_id therefore gets the SAME reply again, and the id is remembered
// only once its handler has run and answered -- never before, so a command
// that never reached its handler is not silently swallowed. `stop` is the
// exception that re-executes on a resend as well: it is idempotent and it
// is the fail-safe, so a resend must always both stop and acknowledge.
// (The first cut recorded the id before any handler ran and dropped the
// duplicate with no reply, so a stop resent after a lost reply did nothing
// and said nothing.)
class CommandDispatcher {
 public:
  // subject, payload: the reply transport. Defaults to a detached NATS
  // publish; a test captures replies instead.
  using Publish = std::function<void(const std::string&, const std::string&)>;

  CommandDispatcher(std::string robot_cell, NatsAddress nats,
                    LeaderAuthority* leader, ExecutionBackend* executor,
                    Publish publish = nullptr)
      : robot_cell_(std::move(robot_cell)),
        nats_(nats),
        leader_(leader),
        executor_(executor),
        publish_(std::move(publish)) {
    if (!publish_) {
      const NatsAddress address = nats_;
      publish_ = [address](const std::string& subject, const std::string& payload) {
        // Detached: a reply must never block the command subscriber.
        std::thread([address, subject, payload] {
          nats_publish(address, subject, payload, "joint_trajectory_daemon-reply");
        }).detach();
      };
    }
  }

  // A full queue refuses explicitly without remembering the ID: no handler ran,
  // so retrying that ID after capacity returns must still reach the handler.
  void refuse_queued(const std::string& payload, const std::string& reply_to) {
    handle(payload, reply_to, true);
  }

  void handle(const std::string& payload, const std::string& reply_to, bool queue_full = false) {
    JsonValue root;
    JsonParser parser(payload.data(), payload.size());
    if (!parser.parse(&root) || root.kind != JsonValue::Kind::Object) return;

    std::string schema, command, robot, command_id, sender_id;
    if (!json_str(root.find("schema"), &schema) || schema != kRobotCommandSchema ||
        !json_str(root.find("command"), &command) || command.empty() ||
        !json_str(root.find("robot"), &robot) || robot != robot_cell_ ||
        !json_str(root.find("command_id"), &command_id) || command_id.empty() ||
        !json_str(root.find("sender_id"), &sender_id) || sender_id.empty()) {
      return;
    }
    if (command == "stop") {
      // Fail-safe: unfenced, no sequence required, always accepted, and
      // re-executed on a resend -- stopping twice is stopping.
      executor_->stop("operator_stop");
    }
    {
      std::string remembered;
      if (remembered_reply(command_id, &remembered)) {
        if (!reply_to.empty()) publish_(reply_to, remembered);
        return;
      }
    }

    if (queue_full) {
      reply(reply_to, command, command_id, false, "command_queue_full", nullptr, false);
      return;
    }
    const std::int64_t now = monotonic_ms();
    if (command == "leader_acquire") {
      std::uint64_t ttl_ms = 0;
      std::string boot_id;
      json_u64(root.find("ttl_ms"), &ttl_ms);
      json_str(root.find("controller_boot_id"), &boot_id);
      // v1 rule: a client-supplied fence is refused outright.
      if (root.find("lease_id") != nullptr || root.find("leader_fence_epoch") != nullptr) {
        reply(reply_to, command, command_id, false, "client_fence_not_accepted", nullptr);
        return;
      }
      const LeaseGrant grant = leader_->acquire(
          sender_id, boot_id, ttl_ms, now, [this] { executor_->revoke_motion(); },
          [this](const LeaderAuthority::Snapshot& snapshot) {
            return executor_->leader_acquired(snapshot).reason;
          });
      reply(reply_to, command, command_id, grant.ok, grant.reason, &grant);
      return;
    }
    if (command == "leader_renew" || command == "leader_release") {
      std::uint64_t ttl_ms = 0, fence_epoch = 0;
      std::string boot_id, lease_id;
      json_u64(root.find("ttl_ms"), &ttl_ms);
      json_u64(root.find("leader_fence_epoch"), &fence_epoch);
      json_str(root.find("controller_boot_id"), &boot_id);
      json_str(root.find("lease_id"), &lease_id);
      const LeaseGrant grant =
          command == "leader_renew"
              ? leader_->renew(sender_id, boot_id, lease_id, fence_epoch, ttl_ms, now)
              : leader_->release(sender_id, boot_id, lease_id, fence_epoch, now,
                                 [this] { executor_->revoke_motion(); });
      reply(reply_to, command, command_id, grant.ok, grant.reason, &grant);
      return;
    }

    if (command != "play" && command != "pause" && command != "stop" &&
        command != "go_home") {
      return;  // bridge-owned kinds (arm, io, ...) are not ours to answer
    }

    if (command == "stop") {
      reply(reply_to, command, command_id, true, "", nullptr);
      return;
    }

    const LeaderCredential credential = decode_leader_credential(root);
    if (!leader_->authorizes(sender_id, credential, now)) {
      reply(reply_to, command, command_id, false,
            "fresh_exact_leader_lease_required", nullptr);
      return;
    }
    std::uint64_t motion_intent_seq = 0;
    if (!json_u64(root.find("motion_intent_seq"), &motion_intent_seq) ||
        motion_intent_seq == 0) {
      reply(reply_to, command, command_id, false,
            "ordered_motion_intent_sequence_required", nullptr);
      return;
    }
    if (!leader_->accept_motion_intent(sender_id, motion_intent_seq)) {
      reply(reply_to, command, command_id, false, "stale_motion_intent_sequence",
            nullptr);
      return;
    }

    if (command == "pause") {
      const auto result = executor_->pause_command();
      reply(reply_to, command, command_id, result.ok, result.reason, nullptr);
      return;
    }
    if (command == "go_home") {
      // Fenced like play, but the plan is the daemon's own: from the pose
      // the servos report to the configured home, so no identity to match.
      const Rejection result = executor_->go_home();
      reply(reply_to, command, command_id, result.ok, result.reason, nullptr);
      return;
    }

    PlayIdentity identity;
    identity.leader_fence_epoch = credential.leader_fence_epoch;
    const bool identity_present =
        json_str(root.find("plan_id"), &identity.plan_id) &&
        json_str(root.find("program_id"), &identity.program_id) &&
        json_str(root.find("program_digest"), &identity.program_digest) &&
        json_str(root.find("trajectory_digest"), &identity.trajectory_digest) &&
        json_u64(root.find("manifest_revision"), &identity.manifest_revision) &&
        json_u64(root.find("plan_revision"), &identity.plan_revision);
    if (!identity_present) {
      reply(reply_to, command, command_id, false,
            "exact_staged_plan_identity_required", nullptr);
      return;
    }
    const Rejection result = executor_->play(identity);
    reply(reply_to, command, command_id, result.ok, result.reason, nullptr);
  }

 private:
  bool remembered_reply(const std::string& command_id, std::string* payload) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = replies_.find(command_id);
    if (it == replies_.end()) return false;
    *payload = it->second;
    return true;
  }

  // Called once the handler has run: the same 256-deep window as v1.
  void remember(const std::string& command_id, const std::string& payload) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (replies_.emplace(command_id, payload).second) order_.push_back(command_id);
    while (order_.size() > 256) {
      replies_.erase(order_.front());
      order_.pop_front();
    }
  }

  void reply(const std::string& reply_to, const std::string& command,
             const std::string& command_id, bool ok, const std::string& reason,
             const LeaseGrant* grant, bool remember_result = true) {
    std::string payload = "{\"schema\":\"" + std::string(kCommandReplySchema) +
                          "\",\"command\":" + json_string(command) +
                          ",\"command_id\":" + json_string(command_id) +
                          ",\"ok\":" + (ok ? "true" : "false");
    if (!reason.empty()) payload += ",\"reason\":" + json_string(reason);
    if (grant != nullptr && grant->ok) {
      payload += ",\"lease_id\":" + json_string(grant->lease_id) +
                 ",\"leader_fence_epoch\":" + std::to_string(grant->fence_epoch) +
                 ",\"expires_in_ms\":" + std::to_string(grant->expires_in_ms);
    }
    payload += "}";
    if (remember_result) remember(command_id, payload);  // handled: a resend gets this again
    if (!reply_to.empty()) publish_(reply_to, payload);
  }

  const std::string robot_cell_;
  const NatsAddress nats_;
  LeaderAuthority* leader_;
  ExecutionBackend* executor_;
  mutable std::mutex mutex_;
  std::map<std::string, std::string> replies_;  // command_id -> reply payload
  std::deque<std::string> order_;
  Publish publish_;
};

}  // namespace dense_joint_trajectory
