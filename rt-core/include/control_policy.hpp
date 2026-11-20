#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <limits>

namespace rosie::rtcore {
// Inherited transport timeout; distinct from the application grant cap. This
// is not a measured host/drive stopping bound (P0/P7 qualification remains).
inline constexpr uint64_t kTransportHeartbeatLeaseNs = 250000000;
// The RT thread evaluates the output permit every cycle, independently of the
// IPC helper or a blocking commissioning operation. Revoke increments an epoch
// so reconnect/re-arm cannot revive previously queued motion.
struct ControlPolicy {
  std::atomic<uint64_t> heartbeat_ns{0};
  std::atomic<uint64_t> generation{1};
  std::atomic<bool> connected{false};
  // Set before publishing connected. Only the explicit simulation oracle role
  // omits this gate; production adapter heartbeats never grant motion authority.
  std::atomic<bool> application_required{false};
  std::atomic<uint64_t> application_generation{0};
  std::atomic<uint64_t> application_deadline_ns{0};
  std::atomic<uint32_t> config_verified_mask{0};
  std::atomic<uint32_t> home_valid_mask{0};
  uint64_t lease_ns = kTransportHeartbeatLeaseNs;

  bool connection_valid(uint64_t now) const noexcept {
    const uint64_t heartbeat = heartbeat_ns.load(std::memory_order_acquire);
    return connected.load(std::memory_order_acquire) && heartbeat != 0 &&
        now >= heartbeat && now - heartbeat < lease_ns;
  }
  bool application_valid(uint64_t now) const noexcept {
    if (!application_required.load(std::memory_order_acquire)) return true;
    const auto before = generation.load(std::memory_order_acquire);
    const auto admitted = application_generation.load(std::memory_order_acquire);
    const auto deadline = application_deadline_ns.load(std::memory_order_acquire);
    return admitted != 0 && admitted == before && now < deadline &&
        generation.load(std::memory_order_acquire) == before;
  }
  bool lease_valid(uint64_t now) const noexcept {
    return connection_valid(now) && application_valid(now);
  }
  bool lease_valid_now() const noexcept {
    // Load the heartbeat before sampling time. A concurrently refreshed
    // heartbeat can otherwise appear to be in the future relative to the
    // caller's cycle-start timestamp and spuriously revoke a healthy owner.
    const uint64_t heartbeat = heartbeat_ns.load(std::memory_order_acquire);
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return false;
    const uint64_t now = uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
    return connected.load(std::memory_order_acquire) && heartbeat != 0 &&
        now >= heartbeat && now - heartbeat < lease_ns && application_valid(now);
  }
  uint64_t revoke() noexcept {
    application_generation.store(0, std::memory_order_release);
    return generation.fetch_add(1, std::memory_order_acq_rel) + 1;
  }
  // Fence one observed event exactly once. A stale event has no side effects on
  // a successor generation or its application grant. Callers inhibit outputs
  // independently; refusing an exhausted generation never authorizes motion.
  bool revoke(uint64_t expected_generation) noexcept {
    if (!expected_generation || expected_generation == std::numeric_limits<uint64_t>::max())
      return false;
    uint64_t observed = expected_generation;
    if (!generation.compare_exchange_strong(observed, expected_generation + 1,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire)) return false;
    // A new grant may already have been admitted after the generation CAS. Do
    // not erase it when finishing this older event's application invalidation.
    observed = expected_generation;
    (void)application_generation.compare_exchange_strong(observed, 0,
        std::memory_order_acq_rel, std::memory_order_acquire);
    return true;
  }
  bool axis_ready(unsigned axis, bool require_home) const noexcept {
    const uint32_t bit = 1u << axis;
    return (config_verified_mask.load(std::memory_order_acquire) & bit) &&
        (!require_home || (home_valid_mask.load(std::memory_order_acquire) & bit));
  }
};

struct AxisLimits {
  double min_q = 0;
  double max_q = 0;
  double max_qd = 0;
  bool require_home = true;
  bool valid = false;
  double jog_acceleration = 0;
  double max_target_lead = 0;
  double following_error = 0;
  double completion_tolerance = 0;
  uint64_t following_error_timeout_ns = 0;
  uint64_t completion_timeout_ns = 0;
  // Machine-level tracking lag allowance (axes.conf tracking_lag_allowance,
  // identical on every axis): max_target_lead, following_error and the
  // collision watchdog's following_error_counts_max grow by
  // |commanded velocity| x this time (speed_scaled_bound). 0 when the machine
  // omits it: static bounds, as before the allowance existed.
  uint64_t tracking_lag_allowance_ns = 0;
  bool contains(double q) const noexcept {
    return valid && std::isfinite(q) && q >= min_q && q <= max_q;
  }
  bool allows_velocity(double qd) const noexcept {
    return valid && std::isfinite(qd) && std::abs(qd) <= max_qd;
  }
};
}
