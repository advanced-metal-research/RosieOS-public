#pragma once

// The `executor` object of the TCP `status` reply, rendered by hand (the
// daemon has no JSON library; the ingest server builds its replies the
// same way).
//
// Shape contract: the first eight keys (state, abort_reason?,
// trajectory_digest?, segment_index, t_s, sample_index, torch_on,
// feedback_age_ms, authority_fresh) are the original reply, read by the
// OLP Go decoder and tools/bench_play.py -- their names, types and
// presence rules do not change. `armed`, `authority` and `feedback` are
// always present, and every one of the nine `feedback.axes` entries is
// always present, so a client renders the same table whether or not the
// robot is talking; only the per-axis radians go `null` when they cannot
// be computed (no feedback, or no authority to supply the gear).

#include <cstdio>
#include <string>

#include "execution_backend.hpp"
#include "dense_joint_trajectory_ingest.hpp"

namespace dense_joint_trajectory {

namespace status_json_detail {

// Fixed six decimals: 1 urad resolution, plenty for a status display, and
// a stable width so the reply does not change size tick to tick.
inline std::string rad(bool valid, double value) {
  if (!valid) return "null";
  char text[64];
  std::snprintf(text, sizeof(text), "%.6f", value);
  return text;
}

inline const char* boolean(bool value) { return value ? "true" : "false"; }

}  // namespace status_json_detail

inline std::string executor_status_to_json(const ExecutorStatus& s) {
  using namespace status_json_detail;
  std::string out = "{\"state\":" + json_string(s.state);
  out += ",\"backend\":" + json_string(s.backend) +
         ",\"detail\":" + json_string(s.detail) +
         ",\"consumer_action\":" + json_string(s.consumer_action) +
         ",\"native_state\":" + json_string(s.native_state) +
         ",\"native_result\":" + s.native_result_json +
         ",\"program_identity\":" + s.program_identity_json +
         ",\"execution_state\":" + json_string(s.execution_state) +
         ",\"commissioning_phase\":" + json_string(s.commissioning_phase) +
         ",\"execution_generation\":" + std::to_string(s.execution_generation) +
         ",\"native_sequence\":" + std::to_string(s.native_sequence);
  if (!s.abort_reason.empty()) {
    out += ",\"abort_reason\":" + json_string(s.abort_reason);
  }
  if (!s.trajectory_digest.empty()) {
    out += ",\"trajectory_digest\":" + json_string(s.trajectory_digest);
  }
  out += ",\"segment_index\":" + std::to_string(s.segment_index) +
         ",\"t_s\":" + std::to_string(s.t_s) +
         ",\"sample_index\":" + std::to_string(s.sample_index) +
         ",\"torch_on\":" + boolean(s.torch_on) +
         ",\"feedback_age_ms\":" + std::to_string(s.feedback_age_ms) +
         ",\"tracking_error_rad\":" + std::to_string(s.tracking_error_rad) +
         ",\"tracking_error_axis\":" + std::to_string(s.tracking_error_axis) +
         ",\"authority_fresh\":" + boolean(s.authority_fresh) +
         ",\"leader_fresh\":" + boolean(s.leader_fresh);
  // The daemon's own plan, when one is or was under the cursor.
  out += ",\"transient\":" + (s.transient.empty() ? std::string("null") : json_string(s.transient));
  if (!s.transient.empty()) {
    char duration[32];
    std::snprintf(duration, sizeof(duration), "%.6f", s.transient_duration_s);
    out += ",\"transient_plan\":{\"kind\":" + json_string(s.transient) +
           ",\"sample_count\":" + std::to_string(s.transient_sample_count) +
           ",\"duration_s\":" + duration + "}";
  }

  out += std::string(",\"armed\":") + boolean(s.armed);

  const AuthorityStatus& authority = s.authority;
  out += std::string(",\"authority\":{\"valid\":") + boolean(authority.valid) +
         ",\"fresh\":" + boolean(authority.fresh) +
         ",\"runtime_mode\":" + json_string(authority.runtime_mode) +
         ",\"runtime_mode_epoch\":" + std::to_string(authority.runtime_mode_epoch) +
         ",\"mapped_axis_mask\":" + std::to_string(authority.mapped_axis_mask) +
         ",\"jetson_id\":" + json_string(authority.jetson_id) +
         ",\"robot_model_id\":" + json_string(authority.robot_model_id) + "}";

  const FeedbackStatus& feedback = s.feedback;
  out += std::string(",\"feedback\":{\"valid\":") + boolean(feedback.valid) +
         ",\"age_ms\":" + std::to_string(feedback.age_ms) +
         ",\"armed_mask\":" + std::to_string(feedback.armed_mask) +
         ",\"fault_mask\":" + std::to_string(feedback.fault_mask) +
         ",\"error_mask\":" + std::to_string(feedback.error_mask) +
         ",\"home_valid_mask\":" + std::to_string(feedback.home_valid_mask) +
         ",\"axes\":[";
  for (std::size_t a = 0; a < kAxisCount; ++a) {
    const AxisStatus& axis = feedback.axes[a];
    if (a != 0) out += ",";
    out += "{\"index\":" + std::to_string(a) +
           ",\"q_rad\":" + rad(axis.q_valid, axis.q_rad) +
           ",\"q_cmd_rad\":" + rad(axis.q_cmd_valid, axis.q_cmd_rad) +
           ",\"pos_counts\":" + std::to_string(axis.pos_counts) +
           ",\"home_counts\":" + std::to_string(axis.home_counts) +
           ",\"statusword\":" + std::to_string(unsigned(axis.statusword)) +
           ",\"mode_display\":" + std::to_string(unsigned(axis.mode_display)) +
           ",\"operational\":" + boolean(axis.operational) +
           ",\"fault\":" + boolean(axis.fault) +
           ",\"error\":" + boolean(axis.error) +
           ",\"homed\":" + boolean(axis.homed) + "}";
  }
  out += "]}}";
  return out;
}

}  // namespace dense_joint_trajectory
