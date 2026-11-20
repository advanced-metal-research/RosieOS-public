#pragma once

// Shared immutable-program execution contract. Only the public rt-core client
// implements it; command dispatch cannot acquire a legacy UDP writer.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "dense_joint_trajectory_format.hpp"
#include "leader_authority.hpp"
#include "nats_client.hpp"

namespace dense_joint_trajectory {

// Monotonic timestamps fence NATS authority; wall timestamps identify process leases.

inline std::int64_t monotonic_ms() {
  struct timespec now {};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return std::int64_t(now.tv_sec) * 1000 + now.tv_nsec / 1000000;
}

inline std::int64_t wall_ns() {
  struct timespec now {};
  clock_gettime(CLOCK_REALTIME, &now);
  return std::int64_t(now.tv_sec) * 1000000000 + now.tv_nsec;
}



struct PlayIdentity {
  std::string plan_id;
  std::string program_id;
  std::string program_digest;
  std::string trajectory_digest;
  std::uint64_t manifest_revision = 0;
  std::uint64_t plan_revision = 0;
  std::uint64_t leader_fence_epoch = 0;
};

// One live axis as the TCP status reply shows it: the raw servo words from
// the last accepted ServoFeedbackV4 next to their joint-space reading, so a
// client can see both what the drive says and what the daemon makes of it.
struct AxisStatus {
  bool q_valid = false;       // false => q_rad is unavailable (JSON null)
  double q_rad = 0.0;         // feedback counts -> joint rad (gear, sign applied)
  bool q_cmd_valid = false;   // false until the play commanded a sample
  double q_cmd_rad = 0.0;     // the last commanded sample's q
  std::int32_t pos_counts = 0;
  std::int32_t home_counts = 0;
  std::uint16_t statusword = 0;
  std::uint8_t mode_display = 0;
  bool operational = false;   // statusword_operation_enabled
  bool fault = false;         // bit in axis_fault_mask
  bool error = false;         // bit in axis_error_mask
  // RTCore's home-valid bit. Without it the axis has counts but no angle:
  // home_counts is 0 and the conversion wraps to +-pi, which read as a real
  // position on the live cell (the nine-motor bench, 2026-09-04: J1/J4/J6
  // unhomed) until this flag was carried.
  bool homed = false;
};

struct AuthorityStatus {
  bool valid = false;
  bool fresh = false;
  std::string runtime_mode;   // "" when the status doc carried none
  std::uint64_t runtime_mode_epoch = 0;
  std::uint32_t mapped_axis_mask = 0;
  std::string jetson_id;
  std::string robot_model_id;
};

struct FeedbackStatus {
  bool valid = false;
  std::int64_t age_ms = -1;   // -1 while no feedback has been accepted
  std::uint32_t armed_mask = 0;      // axes whose statusword is operation-enabled
  std::uint32_t fault_mask = 0;      // ServoFeedbackV4 axis_fault_mask
  std::uint32_t error_mask = 0;      // ServoFeedbackV4 axis_error_mask
  std::uint32_t home_valid_mask = 0; // ServoFeedbackV4 home_valid_axis_mask
  std::array<AxisStatus, kAxisCount> axes{};
};

// The executor's view for the TCP status reply. The first block is the
// original contract read by the OLP Go decoder and tools/bench_play.py and
// never changes shape; `armed`, `authority` and `feedback` are the arm and
// servo view a UI needs before it dares to press play, and they are always
// present, never conditionally omitted, so a client can bind to them.
struct ExecutorStatus {
  std::string state;        // idle|staged|playing|paused|done|aborted
  std::string abort_reason;
  std::string trajectory_digest;
  int segment_index = -1;
  std::size_t sample_index = 0;
  double t_s = 0.0;
  bool torch_on = false;
  std::int64_t feedback_age_ms = -1;
  double tracking_error_rad = -1.0;
  int tracking_error_axis = -1;  // 0-based axis behind that error, -1 when none
  bool authority_fresh = false;
  bool leader_fresh = false;
  std::string backend = "rt_core";
  std::string detail;
  std::string consumer_action;
  std::string native_result_json = "null";
  std::string program_identity_json = "null";
  std::string execution_state;
  std::string commissioning_phase;
  std::string native_state = "inactive";  // inactive|acquiring|owned|reconciling|cleanup_uncertain|grant_active
  std::uint64_t execution_generation = 0;
  std::uint64_t native_sequence = 0;

  // A transient plan the daemon synthesised itself ("go_home"), playing or
  // just finished; empty otherwise. While set, segment_index/sample_index/
  // t_s refer to it and trajectory_digest still names the staged program,
  // which is untouched and plays afterwards from its first sample.
  std::string transient;
  std::size_t transient_sample_count = 0;
  double transient_duration_s = 0.0;

  // Feedback is valid AND every axis the authority maps is operation-
  // enabled: the same predicate the playing tick aborts `disarmed` on, so
  // a UI that shows armed==true is showing a play that will not abort on
  // the first tick.
  bool armed = false;
  AuthorityStatus authority;
  FeedbackStatus feedback;
};

// Native execution exposes program identity and fenced lifecycle operations.
class ExecutionBackend {
 public:
  virtual ~ExecutionBackend() = default;
  virtual bool start() = 0;
  virtual void stop_thread() = 0;
  virtual Rejection preload(const Plan&, const std::vector<unsigned char>&,
                            const std::string&) = 0;
  virtual Rejection play(const PlayIdentity&) = 0;
  virtual Rejection pause_command() = 0;
  // Unsupported backends refuse without invoking the legacy follower or changing motion.
  virtual Rejection go_home() {
    return Rejection::refuse("go_home_unsupported", -1, "backend has no Go Home planner");
  }
  virtual void stop(const std::string&) = 0;
  virtual void revoke_motion() = 0;
  virtual Rejection leader_acquired(const LeaderAuthority::Snapshot&) {
    return Rejection::accept();
  }
  virtual ExecutorStatus status() const = 0;
};

}  // namespace dense_joint_trajectory
