#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <limits>
#include <utility>

// A position intent is not a trajectory or a packet-writing controller.
// The existing execution owner supplies configured limits and valid bound
// feedback, serializes cancellation, and remains the sole output writer.
struct PositionExecutionRequest {
  std::string command_id;
  // Validated original JSON token, never a floating-point conversion. Keep
  // numeric legacy IDs distinct from strings when encoding terminal replies.
  std::string command_id_json;
  std::string runtime_mode;
  int axis = 0;
  double target_rad = 0.0;
  bool relative = false;
  double min_rad = 0.0;
  double max_rad = 0.0;
  double tolerance_rad = 0.5 * 3.14159265358979323846 / 180.0;
  double max_speed_rad_s = 0.0;
  std::chrono::milliseconds timeout{15000};
};

enum class PositionExecutionOutcome {
  accepted,
  executing,
  completed,
  cancelled,
  timed_out,
  failed,
};

// Pure lifecycle of one admitted request. No transport, thread, output,
// authority cache, or automatic retry lives here. Terminal outcomes are
// immutable and retain the original identity. These timing/settling defaults
// preserve the bridge baseline; they are not new actuator limits.
class PositionExecution {
 public:
  using Clock = std::chrono::steady_clock;

  PositionExecution(PositionExecutionRequest request,
                    double configured_min_rad, double configured_max_rad,
                    Clock::time_point admitted_at)
      : request_(std::move(request)),
        deadline_(admitted_at + request_.timeout),
        next_sample_(admitted_at),
        min_rad_(configured_min_rad), max_rad_(configured_max_rad) {
    if (request_.min_rad != 0.0 || request_.max_rad != 0.0) {
      min_rad_ = std::max(min_rad_, std::min(request_.min_rad, request_.max_rad));
      max_rad_ = std::min(max_rad_, std::max(request_.min_rad, request_.max_rad));
    }
    if (!std::isfinite(configured_min_rad) ||
        !std::isfinite(configured_max_rad) ||
        !std::isfinite(request_.target_rad) ||
        !std::isfinite(request_.min_rad) || !std::isfinite(request_.max_rad) ||
        !std::isfinite(request_.tolerance_rad) || request_.tolerance_rad <= 0.0 ||
        request_.timeout.count() <= 0 || min_rad_ > max_rad_) {
      outcome_ = PositionExecutionOutcome::failed;
    }
  }

  PositionExecutionOutcome outcome() const { return outcome_; }
  const std::string& command_id() const { return request_.command_id; }
  const std::string& command_id_json() const { return request_.command_id_json; }
  double target_rad() const { return target_rad_; }
  bool target_resolved() const { return target_resolved_; }
  bool in_tolerance() const { return in_tolerance_samples_ != 0; }
  bool terminal() const {
    return outcome_ != PositionExecutionOutcome::accepted &&
           outcome_ != PositionExecutionOutcome::executing;
  }

  void cancel() {
    if (!terminal()) outcome_ = PositionExecutionOutcome::cancelled;
  }
  void fail() {
    if (!terminal()) outcome_ = PositionExecutionOutcome::failed;
  }

  // Poll even when no sample arrives. A stopped feedback source cannot extend
  // an admitted operation's deadline; only a fresh request starts a new one.
  bool feedback_due(Clock::time_point now) {
    if (!terminal() && now >= deadline_) {
      outcome_ = PositionExecutionOutcome::timed_out;
    }
    return !terminal() && now >= next_sample_;
  }

  PositionExecutionOutcome observe(Clock::time_point now, double current_rad) {
    if (!feedback_due(now)) return outcome_;
    if (!std::isfinite(current_rad)) {
      fail();
      return outcome_;
    }
    if (!target_resolved_) {
      const double target = request_.relative
                                ? current_rad + request_.target_rad
                                : request_.target_rad;
      target_rad_ = std::clamp(target, min_rad_, max_rad_);
      target_resolved_ = true;
    }
    constexpr double seam = 3.14159265358979323846 - 2 * 3.14159265358979323846 / 180.0;
    if (std::abs(current_rad) >= seam && std::abs(target_rad_) >= seam &&
        current_rad * target_rad_ < 0) {
      fail();
      return outcome_;
    }
    // Preserve the bridge's three growing-error samples, in joint radians.
    const double error = std::abs(target_rad_ - current_rad);
    runaway_samples_ = error > 0.02 && error > previous_error_ + 0.003 ? runaway_samples_ + 1 : 0;
    previous_error_ = error;
    if (runaway_samples_ >= 3) {
      fail();
      return outcome_;
    }
    outcome_ = PositionExecutionOutcome::executing;
    next_sample_ = now + std::chrono::milliseconds(40);
    if (std::abs(target_rad_ - current_rad) <= request_.tolerance_rad) {
      if (++in_tolerance_samples_ >= 4) {
        outcome_ = PositionExecutionOutcome::completed;
      }
    } else {
      in_tolerance_samples_ = 0;
    }
    return outcome_;
  }

 private:
  PositionExecutionRequest request_;
  Clock::time_point deadline_;
  Clock::time_point next_sample_;
  double min_rad_;
  double max_rad_;
  double target_rad_ = 0.0;
  bool target_resolved_ = false;
  unsigned in_tolerance_samples_ = 0;
  unsigned runaway_samples_ = 0;
  double previous_error_ = std::numeric_limits<double>::infinity();
  PositionExecutionOutcome outcome_ = PositionExecutionOutcome::accepted;
};
