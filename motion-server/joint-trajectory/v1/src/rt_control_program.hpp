#pragma once

#include "dense_joint_trajectory_format.hpp"
#include "rosie/rt_control_client.hpp"

namespace dense_joint_trajectory {
namespace rtc = rosie::rt_control;

// Computes only producer identity. The adapter owns stationary normalisation;
// its returned digest is retained and checked unchanged at every later boundary.
inline rtc::Identity native_identity(const Plan& plan) {
  const auto& h = plan.header;
  return {h.plan_id, h.program_id, h.program_digest, h.trajectory_digest,
          trajectory_digest(h.axis_mask, h.dt_s, h.max_abs_qd_rad_s, plan.segments), "",
          h.manifest_revision, h.plan_revision};
}

inline bool same_identity(const rtc::Identity& a, const rtc::Identity& b) {
  return rtc::encode(a) == rtc::encode(b);
}

struct NativeProgress {
  int segment = 0;
  std::size_t sample = 0;
  double seconds = 0;
};

// Maps the assembled native sample index back to the producer's segment clock.
// NativeClockNS is a host observation timestamp, never elapsed program time.
// The following segment owns a shared boundary sample, as in native admission.
inline NativeProgress native_progress(const Plan& plan, std::size_t index, bool completed) {
  for (std::size_t segment = 0; segment < plan.segments.size(); ++segment) {
    const auto& samples = plan.segments[segment].samples;
    if (samples.empty()) throw rtc::ProtocolError("native_cursor_invalid");
    if (segment + 1 < plan.segments.size() && (completed || index >= samples.size() - 1)) {
      if (!completed) index -= samples.size() - 1;
      continue;
    }
    if (completed) index = samples.size() - 1;
    if (index >= samples.size()) throw rtc::ProtocolError("native_cursor_invalid");
    return {int(segment), index, samples[index].time_s};
  }
  throw rtc::ProtocolError("native_cursor_invalid");
}

// Only fatal authority outcomes need special actions. Other refusal labels stay
// verbatim in status; unknown outcomes conservatively stop and reconcile.
inline std::string native_consumer_action(const std::string& reason) {
  const auto label = reason.substr(0, reason.find(':'));
  if (label == "expired" || label == "control_session_stale" || label == "fence")
    return "stop_reconcile_reacquire_upload";
  if (label == "daemon_restarted" || label == "native_incarnation_changed")
    return "invalidate_describe_bind_reacquire_upload";
  return "abort_stop_reconcile";
}
}  // namespace dense_joint_trajectory
