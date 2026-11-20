#include "io_wait.hpp"
#pragma once

#include "source_version.hpp"
#include "verified_delta_restore.hpp"
#include "native_home_wire_audit.hpp"
#include "shutdown_drain.hpp"
#include "brake_transition_guard.hpp"
#include "drive_state_supervisor.hpp"
#include "drive_alarm_reset.hpp"

#include "ipc_v1.hpp"

#include <atomic>
#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <limits>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sys/utsname.h>
#include <grp.h>
#include <mutex>
#include <memory>
#include <optional>
#include <pwd.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

using gradient::ipc::v1::AxisStatusV1;

#include "ecrt_detect.hpp"
#include "include/bus_topology.hpp"
#include "ds402.hpp"
#include "control_policy.hpp"
#include "fast_control_region.hpp"
#include "rt_mapping_handoff.hpp"
#include "engine/application_grant.hpp"
#include "connection_failure.hpp"
#include "peer_admission.hpp"
#include "execution_policy.hpp"
#include "fault_recovery.hpp"
#include "checked_bus.hpp"
#include "cycle_bus.hpp"
#include "output_policy.hpp"
#include "coordinate_evidence.hpp"
#include "configuration_identity.hpp"
#include "motion_readiness.hpp"
#include "engine/trajectory_store.hpp"
#include "engine/publication.hpp"
#include "engine/jog.hpp"
#include "engine/jog_lane.hpp"
#include "engine/trajectory.hpp"
#include "engine/completion.hpp"
#include "engine/commissioning.hpp"
#include "engine/diagnostics.hpp"
#include "engine/cycle_capture.hpp"
#include "snapshot_values.hpp"
#include "drive_alarm.hpp"
#include "drive_health.hpp"
#include "command_validation.hpp"
#include "startup_sdo.hpp"
#include "startup_sdo_plan.hpp"
#include "cell_io.hpp"
#include "housekeeping_cpuset.hpp"
#include "rt_cpu.hpp"

#ifndef MFD_CLOEXEC
#define MFD_CLOEXEC 0x0001U
#endif

namespace {

// Keep the engine independent of its wire format while checking every value
// exported in a generated grant receipt at this integration boundary.
#define CHECK_GRANT_REASON(name, wire_name) static_assert( \
    static_cast<uint32_t>(rosie::rtcore::ApplicationGrantReason::name) == rosie::protocol::grant_reason_##wire_name)
CHECK_GRANT_REASON(None, none);
CHECK_GRANT_REASON(Invalid, invalid);
CHECK_GRANT_REASON(ConnectionMismatch, connection_mismatch);
CHECK_GRANT_REASON(NativeGenerationMismatch, native_generation_mismatch);
CHECK_GRANT_REASON(Disconnected, disconnected);
CHECK_GRANT_REASON(Busy, busy);
CHECK_GRANT_REASON(NotInhibited, not_inhibited);
CHECK_GRANT_REASON(StaleGeneration, stale_generation);
CHECK_GRANT_REASON(IdentityMismatch, identity_mismatch);
CHECK_GRANT_REASON(Expired, expired);
CHECK_GRANT_REASON(ClockRegression, clock_regression);
CHECK_GRANT_REASON(Inactive, inactive);
CHECK_GRANT_REASON(AxisMaskMismatch, axis_mask_mismatch);
#undef CHECK_GRANT_REASON
static_assert(static_cast<uint32_t>(rosie::rtcore::ApplicationGrantOperation::Acquire) == rosie::protocol::grant_operation_acquire);
static_assert(static_cast<uint32_t>(rosie::rtcore::ApplicationGrantOperation::Renew) == rosie::protocol::grant_operation_renew);
static_assert(static_cast<uint32_t>(rosie::rtcore::ApplicationGrantOperation::Revoke) == rosie::protocol::grant_operation_revoke);

std::atomic<bool> g_stop{false};

extern "C" void handle_signal(int) {
  g_stop.store(true, std::memory_order_relaxed);
}

uint64_t now_monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

int32_t clamp_i64_to_i32(int64_t value) {
  if (value < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
    return std::numeric_limits<int32_t>::min();
  }
  if (value > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    return std::numeric_limits<int32_t>::max();
  }
  return static_cast<int32_t>(value);
}

// RTCore writes drive-facing CSP targets in the same raw PDO count frame that
// the drive publishes through 0x6064 and expects on 0x607A. Controller-side
// logical targets still include persisted native-home truth, so RTCore must
// subtract that offset exactly once when it converts queued targets into CSP
// wire counts. Mirroring live feedback back to a hold target must stay in raw
// PDO counts, otherwise SAFE_POWER_UP injects a step roughly equal to 0x607C.
int32_t csp_wire_counts_from_feedback(int32_t feedback_pos_counts) {
  return feedback_pos_counts;
}

double controller_target_to_csp_wire_counts(double controller_target_counts,
                                            int32_t native_home_offset_counts) {
  return controller_target_counts - static_cast<double>(native_home_offset_counts);
}

template<class... Args> void logf(const char* fmt, const Args&... args) {
  if (rosie::rtcore::cyclic_diagnostics) {
    rosie::rtcore::defer_diagnostic(fmt, rosie::rtcore::log_value(args)...);
    return;
  }
  std::fprintf(stderr, "[rosie-rt-core] ");
  if constexpr (sizeof...(args) == 0) std::fputs(fmt, stderr);
  else std::fprintf(stderr, fmt, rosie::rtcore::log_value(args)...);
  std::fprintf(stderr, "\n");
}

uint32_t master_state_from_al_states(unsigned int al_states, bool link_up) {
  if (!link_up) {
    return gradient::ipc::v1::MASTER_ERROR;
  }
  if ((al_states & 0x8u) != 0u) {
    return gradient::ipc::v1::MASTER_OP;
  }
  if ((al_states & 0x4u) != 0u) {
    return gradient::ipc::v1::MASTER_SAFEOP;
  }
  if ((al_states & 0x2u) != 0u) {
    return gradient::ipc::v1::MASTER_PREOP;
  }
  if ((al_states & 0x1u) != 0u) {
    return gradient::ipc::v1::MASTER_INIT;
  }
  return gradient::ipc::v1::MASTER_INIT;
}

const char* al_state_label(uint8_t al_state) {
  switch (al_state) {
    case 0x01:
      return "INIT";
    case 0x02:
      return "PREOP";
    case 0x04:
      return "SAFEOP";
    case 0x08:
      return "OP";
    default:
      return "UNKNOWN";
  }
}

size_t align_up(size_t value, size_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

#include "daemon_options.hpp"

#include "engine/ipc_transport.hpp"
#include "engine/bus_bringup.hpp"

// Per-quantity consecutive samples prevent a single spike or alternating
// quantities from deciding motion. A trip freezes evidence until explicit reset;
// only the caller may escalate through the machine's execution-fault fence.
struct CollisionWatchdog {
  bool armed = false, tripped = false, clear = false, observing = false;
  uint32_t torque_cycles = 0, following_error_cycles = 0;
  uint32_t peak_torque_raw = 0;
  uint64_t peak_following_error_counts = 0;
  uint32_t last_trip_quantity = 0, trip_sustained_cycles = 0;
  uint64_t trip_count = 0;
  uint64_t samples = 0;
  uint32_t last_trip_peak_torque_raw = 0;
  uint64_t last_trip_peak_following_error_counts = 0;
  // Wire/status bit vocabulary, unverified on hardware: 1 torque, 2 following error.
  static const char* quantity_name(uint32_t quantity) noexcept {
    return quantity == 1 ? "torque" : quantity == 2 ? "following_error" :
        quantity == 3 ? "torque_and_following_error" : "none";
  }
  // The following-error bound grows with speed like the following_error
  // fault's: following_error_counts_max + |commanded velocity| x lag_s
  // (speed_scaled_bound), the velocity being this cycle's commanded velocity
  // on the axis in counts/s and lag_s the machine's tracking_lag_allowance.
  // A drive trailing a fast jog by its position-loop lag is not a collision:
  // mini bench 2026-11-03, an A6-EC (speed feedforward off) jogging at ~20
  // rad/s (421397 counts/s) read 12897..13593 counts, ~31-32 ms of lag, against
  // the static 12000 and tripped every jog within ~200 ms. lag_s 0 (no allowance) or no commanded velocity
  // keeps the static bound; a NaN bound (nonfinite velocity) counts as over.
  // The torque bound never scales. The defaults (no velocity, no allowance)
  // are the static bound the archived-evidence replay judges against.
  bool observe(const CollisionWatchdogConfig& config, bool arm, bool fresh,
               int16_t torque, int32_t drive_error, bool drive_valid,
               int64_t commanded_error, double commanded_velocity_counts_per_s = 0,
               double lag_s = 0) noexcept {
    const uint32_t torque_abs = torque < 0 ? uint32_t(-int32_t(torque)) : uint32_t(torque);
    const int64_t error = drive_valid ? int64_t(drive_error) : commanded_error;
    const uint64_t error_abs = error < 0 ? uint64_t(-(error + 1)) + 1 : uint64_t(error);
    const double error_bound = rosie::rtcore::speed_scaled_bound(double(config.following_error_counts_max),
        commanded_velocity_counts_per_s, lag_s);
    const bool error_over = !(double(error_abs) <= error_bound);
    clear = config.enabled && config.valid() && fresh &&
        torque_abs <= config.torque_abs_max_raw && !error_over;
    const bool next_observing = arm && !tripped;
    if (next_observing && !observing) {
      torque_cycles = following_error_cycles = peak_torque_raw = 0;
      peak_following_error_counts = 0;
      samples = 0;
    }
    observing = next_observing;
    if (observing && fresh) {
      ++samples;
      peak_torque_raw = std::max(peak_torque_raw, torque_abs);
      peak_following_error_counts = std::max(peak_following_error_counts, error_abs);
    }
    armed = config.enabled && config.valid() && observing && fresh;
    if (!armed) {
      if (!tripped) torque_cycles = following_error_cycles = 0;
      return false;
    }
    torque_cycles = torque_abs > config.torque_abs_max_raw ? torque_cycles + 1 : 0;
    following_error_cycles = error_over ? following_error_cycles + 1 : 0;
    const uint32_t quantity = (torque_cycles >= config.sustained_cycles ? 1u : 0u) |
        (following_error_cycles >= config.sustained_cycles ? 2u : 0u);
    if (!quantity) return false;
    last_trip_quantity = quantity;
    trip_sustained_cycles = config.sustained_cycles;
    last_trip_peak_torque_raw = peak_torque_raw;
    last_trip_peak_following_error_counts = peak_following_error_counts;
    ++trip_count;
    tripped = true;
    armed = false;
    return true;
  }
  void reset() noexcept {
    tripped = armed = observing = false;
    torque_cycles = following_error_cycles = 0;
  }
};

// Included inside the daemon namespace after options, bus and logging helpers.
// Address-stable runtime state: registered bus handles and thread references
// must not outlive this object or be redirected by a copy or move.
enum class CyclePhase { Receive, Readiness, Output, Submit, Publish };

struct CycleOutputs {
  bool continue_running = true;
  uint64_t wake_ns = 0;
  uint32_t motion_ready_mask = 0;
  uint32_t motion_permitted_mask = 0;
  uint32_t submitted_mask = 0;
  std::array<int32_t, rosie::protocol::max_axes> targets{};
};

#if GRADIENT_HAVE_ECRT
template<class Bus>
#endif
struct CycleInputs {
#if GRADIENT_HAVE_ECRT
  Bus& bus;
#endif
  rosie::rtcore::FastControlRegion* fast_region = nullptr;
  uint64_t (*clock)() = now_monotonic_ns;
  void* recorder_context = nullptr;
  void (*recorder)(void*, CyclePhase) = nullptr;
  void observe(CyclePhase phase) const noexcept {
    if (recorder) recorder(recorder_context, phase);
  }
};

struct MachineState {
  const Options& opt;
  rosie::rtcore::CellIO cell_io;
  rosie::rtcore::AtomicSnapshot<gradient::ipc::v1::IoStateV1> io_snapshot;
  OptionsParseResult& parsed;
  decltype(parsed.startup_sdos)& startup_sdos;
  decltype(parsed.axis_profiles)& axis_profiles;
  const bool have_native_home;
  const bool have_absolute_feedback;
  const bool metrics_startup_readback_enabled;
  const bool metrics_native_home_refresh_enabled;
  const bool metrics_absolute_feedback_poll_enabled;
  // RT scaffolding: threads exist even before EtherCAT loop is implemented.
  std::atomic<uint64_t> rt_cycle_counter{0};
  std::atomic<int64_t> rt_last_jitter_ns{0};
  std::atomic<int64_t> rt_max_abs_jitter_ns{0};
  std::atomic<uint64_t> rt_max_cycle_work_ns{0};
  rosie::rtcore::CycleCapture cycle_capture;
  rosie::rtcore::TelemetryRing telemetry;
  rosie::rtcore::SpscQueue<rosie::rtcore::TelemetryTrigger, rosie::rtcore::kTelemetryTriggerCapacity> telemetry_cycle_triggers, telemetry_marks;
  std::atomic<uint64_t> telemetry_trigger_drops{0}, telemetry_dumps_written{0}, telemetry_dump_failures{0};
  // Sticky scheduling outcome keeps the typed publication visible after recovery.
  std::atomic<bool> telemetry_scheduling_failed{false};
  uint32_t telemetry_pending = 0; // Cycle-owned Stop bit, consumed even without capture.
  std::array<uint32_t, std::size(rosie::rtcore::kFaultRecovery)> telemetry_pending_faults{};
  // Unverified FIFO backlog: five requests, with one ordinary pending/in-flight
  // binary; shutdown may queue one exit binary behind it. Others are indexes.
  rosie::rtcore::SpscQueue<rosie::rtcore::TelemetryDumpRequest, 5> telemetry_dump_requests;
  rosie::rtcore::TelemetryDumpWake telemetry_dump_wake;
  // Publisher-owned fallback; shutdown consumes it only after both joins.
  rosie::rtcore::TelemetryDumpRequest telemetry_exit_fallback{};
  std::atomic<bool> telemetry_dump_busy{false}, telemetry_writer_stop{false};
  std::atomic<bool> telemetry_writer_affinity_applied{false};
  std::atomic<uint64_t> telemetry_dumps_skipped{0}, telemetry_skipped_trigger_drops{0};
  uint64_t telemetry_index_sequence = 0; // Dump-worker-owned filename discriminator.
#if defined(ROSIE_TELEMETRY_TEST_HOOK)
  void (*telemetry_chunk_observer)(void*) = nullptr;
  void* telemetry_chunk_context = nullptr;
#endif
  std::atomic<uint64_t> capture_records_written{0};
  std::atomic<bool> capture_write_failed{false};
  rosie::rtcore::HousekeepingCpuset housekeeping;
  uint32_t effective_rt_cpu = opt.rt_cpu;
  std::atomic<bool> main_affinity_applied{false}, rt_affinity_applied{false};
  std::atomic<bool> helper_affinity_applied{false}, commissioning_affinity_applied{false};
  std::atomic<bool> bulk_affinity_applied{false}, publisher_affinity_applied{false};

  // Apply the validated mask before a worker handles requests. Live failures
  // terminate admission; simulation retains an observable warning and false flag.
  bool apply_housekeeping_affinity(const char* name, std::atomic<bool>& applied) {
    const int error = pthread_setaffinity_np(pthread_self(), sizeof(housekeeping.mask), &housekeeping.mask);
    applied.store(error == 0, std::memory_order_release);
    if (!error) return true;
#if GRADIENT_ECRT_ENABLED && !defined(ROSIE_RT_SIMULATION)
    logf("ERROR: %s housekeeping affinity failed: %s", name, std::strerror(error));
    process_exit_code.store(2); g_stop.store(true); return false;
#else
    logf("WARNING: %s housekeeping affinity failed: %s", name, std::strerror(error));
    return true;
#endif
  }

  // Snapshot allowed CPUs before narrowing the main thread. Deriving again from
  // an inherited housekeeping mask would incorrectly reject the RT CPU.
  bool initialize_housekeeping() {
    cpu_set_t allowed;
    if (sched_getaffinity(0, sizeof(allowed), &allowed) != 0) {
      logf("ERROR: cannot read allowed CPUs for RT CPU %u: %s", opt.rt_cpu, std::strerror(errno));
      return false;
    }
    auto selected = rosie::rtcore::select_rt_cpu(allowed, opt.rt_cpu, rosie::rtcore::kSimulationBackend);
    if (!selected.cpu) { logf("ERROR: %s", selected.error.c_str()); return false; }
    effective_rt_cpu = *selected.cpu;
    if (!selected.warning.empty()) logf("%s", selected.warning.c_str());
    housekeeping = rosie::rtcore::housekeeping_cpuset(allowed, effective_rt_cpu, opt.housekeeping_cpus);
    if (!housekeeping) { logf("ERROR: %s", housekeeping.message.c_str()); return false; }
    return apply_housekeeping_affinity("main", main_affinity_applied);
  }
  std::atomic<uint64_t> rt_overrun_count{0};

  // Unverified existing bound; qualification has not established this value.
  static constexpr uint32_t kMaxTrajectoryPoints = rosie::protocol::max_trajectory_points;

  // Shared state (helper thread produces; RT thread consumes).
  using TrajectoryPointRuntime = rosie::rtcore::TrajectoryPoint;

  struct JogCommandRuntime {
    uint64_t generation = 0;
    uint64_t seq = 0;
    uint64_t cmd_seq = 0;
    uint32_t axis_mask = 0;
    uint32_t flags = 0;
    uint64_t timeout_ns = 0;
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_counts_per_s{};
  };

  struct LatestFeedback {
    std::atomic<uint64_t> seq{0};
    rosie::rtcore::SnapshotValue<uint32_t> wkc_actual = 0;
    rosie::rtcore::SnapshotValue<uint32_t> wkc_expected = 0;
    rosie::rtcore::SnapshotValue<uint32_t> master_state = gradient::ipc::v1::MASTER_INIT;
    rosie::rtcore::SnapshotValue<uint32_t> responding_slaves = 0;
    rosie::rtcore::SnapshotValue<uint32_t> online_slaves = 0;
    rosie::rtcore::SnapshotValue<uint32_t> operational_slaves = 0;
    rosie::rtcore::SnapshotValue<int64_t> dc_offset_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> dc_reference_sync_attempts = 0;
    rosie::rtcore::SnapshotValue<uint64_t> dc_reference_sync_time_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> dc_reference_sync_interval_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> dc_reference_sync_max_interval_ns = 0;
    rosie::rtcore::SnapshotValue<int64_t> cycle_jitter_ns = 0;
    rosie::rtcore::SnapshotValue<uint8_t> link_up = 0;
    rosie::rtcore::SnapshotValue<uint8_t> master_al_states = 0;
    rosie::rtcore::SnapshotValue<uint8_t> domain_wc_state = 0;
    rosie::rtcore::SnapshotValue<uint8_t> startup_ready = 0;
    rosie::rtcore::SnapshotValue<uint8_t> startup_passive_active = 0;
    rosie::rtcore::SnapshotValue<uint8_t> startup_skip_domain_queue_active = 0;
    rosie::rtcore::SnapshotValue<uint32_t> startup_elapsed_ms = 0;
    rosie::rtcore::SnapshotValue<uint32_t> startup_reset_count = 0;
    std::array<StartupSdoFeedback, kMaxStartupSdoDescriptors> startup_drive_config_feedback{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_state{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_position_offset{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_last_abort_code{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pos_counts{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_state{};
    rosie::rtcore::SnapshotArray<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pdo_observed_time_ns{};
    rosie::rtcore::SnapshotArray<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_epoch{};
    rosie::rtcore::SnapshotArray<int64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_counts{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_source_counts{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_reason{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_valid{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_source_valid{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> calibration_valid{}, position_state{};
    rosie::rtcore::SnapshotValue<uint32_t> pdo_fresh_axis_mask = 0;
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_actual_counts_per_s{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> following_error_counts{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> velocity_actual_valid{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> following_error_valid{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> max_abs_velocity_actual_counts_per_s{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> max_abs_following_error_counts{};
    rosie::rtcore::SnapshotArray<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> torque_raw{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_armed{}, collision_torque_cycles{}, collision_error_cycles{}, collision_peak_torque{}, collision_trip_quantity{}, collision_trip_cycles{};
    rosie::rtcore::SnapshotArray<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_peak_error{}, collision_trip_count{}, collision_samples{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_trip_peak_torque{};
    rosie::rtcore::SnapshotArray<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_trip_peak_error{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> statusword{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> error_code{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> manufacturer_error_code{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> mode_display{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ds402_state{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> di_bits{};
    rosie::rtcore::SnapshotArray<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> di_valid{}, external_enable_active{}, external_enable_valid{};
    // Value prepared for 0x607A (target_pos) in this cycle's process image.
    // Copied into bounded cycle capture alongside feedback, preserving the
    // command stream at cycle rate without issuing an SDO read.
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_pos_counts{};
    std::array<AbsoluteFeedbackAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_feedback{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_al_state{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_online{};
    rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_operational{};

    // Phase 1 (2026-04-20): extended A6-EC 0x2040 PDO telemetry,
    // populated in the feedback cycle below. ext_valid_axis_mask has
    // bit i set iff axes[i] carries a live PDO read this cycle (the
    // underlying offset was registered by register_axis_offsets_explicit).
    // If the extended PDO registration is missing for a given axis, the
    // corresponding bit stays cleared and the StatusExtendedSnapshotV1
    // emitter leaves that axis as zero so Python consumers see the axis
    // as "not present" rather than "zero-valid".
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_position_error_counts{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_multi_turn_lo{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_multi_turn_hi{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_bus_voltage_raw{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_load_rate_raw{};
    rosie::rtcore::SnapshotArray<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_igbt_temp_raw{};
    rosie::rtcore::SnapshotArray<int16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_motor_temp_raw{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_drive_not_ready_bits{};
    rosie::rtcore::SnapshotArray<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> ext_motor_not_rotating_code{};
    std::atomic<uint32_t> ext_valid_axis_mask{0};
    std::atomic<uint64_t> ext_sample_time_ns{0};
  };

  struct LatestJogDebug {
    std::atomic<uint64_t> seq{0};
    rosie::rtcore::SnapshotValue<uint32_t> num_axes = 0;
    rosie::rtcore::SnapshotValue<uint32_t> active_jog = 0;
    rosie::rtcore::SnapshotValue<uint32_t> active_jog_axis_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> command_sp_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> have_hold_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> have_jog_target_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> snap_hold_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> stop_arrest_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> latest_cmd_axis_mask = 0;
    rosie::rtcore::SnapshotValue<uint32_t> latest_cmd_flags = 0;
    rosie::rtcore::SnapshotValue<uint32_t> last_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_NONE;
    rosie::rtcore::SnapshotValue<uint32_t> last_stop_axis_mask = 0;
    rosie::rtcore::SnapshotValue<uint64_t> sample_time_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> active_jog_cmd_seq = 0;
    rosie::rtcore::SnapshotValue<uint64_t> latest_jog_seq_seen = 0;
    rosie::rtcore::SnapshotValue<uint64_t> active_jog_deadline_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> latest_cmd_timeout_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> last_stop_time_ns = 0;
    rosie::rtcore::SnapshotValue<uint64_t> last_stop_cmd_seq = 0;
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> feedback_pos_counts{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> hold_target_counts{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_counts{};
    rosie::rtcore::SnapshotArray<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_velocity_counts_per_s{};
  };

  // Unverified existing bound; qualification has not established this value.
  static constexpr int32_t kNoModeOverride = std::numeric_limits<int32_t>::min();
  rosie::rtcore::ControlPolicy control_policy;
  rosie::rtcore::ConnectionFailureLatch connection_failure;
  rosie::rtcore::RtMappingHandoff<rosie::rtcore::FastControlRegion> fast_handoff;
  std::array<uint8_t, 16> daemon_incarnation{};
  std::atomic<bool> bus_ready{false};
  // GradientOS main.cpp:4855-4905 waits 500000000 ns after startup_ready
  // before userspace SDOs, leaving the configuration FSM exclusive until OP.
  static constexpr uint64_t kMailboxSettleNs = 500000000;
  std::atomic<uint64_t> mailbox_ready_since_ns{0}, mailbox_ready_epoch{0};
  uint64_t (*mailbox_clock)() = now_monotonic_ns;
  std::array<rosie::rtcore::DriveAlarmState, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_alarms{};
  uint32_t alarm_next_axis = 0; // Commissioning worker only.
  bool alarm_readback_pending() const;
  void refresh_drive_alarm();
  // Drive-profile monitor objects (docs/decisions/0001): the worker writes,
  // the metrics thread publishes. The load windows are metrics-thread only.
  std::array<rosie::rtcore::DriveMonitorState, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_monitors{};
  uint32_t monitor_next = 0; // Commissioning worker only.
  void refresh_monitor(uint64_t (*clock)() = now_monotonic_ns);
  std::array<rosie::rtcore::DriveLoadWindow, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_load{};
  uint64_t drive_load_cursor = 0;
  void summarize_drive_load();

  // Use the event-bound mailbox receipt; a missing receipt keeps the axis
  // faulted. A separate nonzero manufacturer PDO remains an independent veto.
  bool drive_error_faulted(uint32_t axis, uint16_t error, uint32_t manufacturer) const {
    const auto& alarm = drive_alarms[axis];
    const auto aux = alarm.snapshot(alarm.event.load(), configuration_epoch.load(), error);
    return manufacturer || axis_profiles[axis].drive_error_faulted(error,
        aux.alarm_aux_code, aux.alarm_aux_valid);
  }

  // Display code and text of a drive alarm (non-RT publication only).
  // Measured A6-EC identity on the nine-motor bench, 2026-09-14; other vendors'
  // manufacturer codes must never inherit this manual's interpretation.
  rosie::rtcore::DriveAlarmName drive_alarm_name(uint32_t axis, uint16_t error, uint32_t aux, bool valid) const {
    const bool a6ec = axis_profiles[axis].vendor_id == 4194304 && axis_profiles[axis].product_code == 1813;
    return a6ec ? rosie::rtcore::decode_a6ec_alarm(error, aux, valid) : rosie::rtcore::DriveAlarmName{};
  }

  // The cyclic owner resets the interval on every readiness/configuration
  // change. Worker observations alone cannot prove continuous readiness.
  void update_mailbox_readiness(bool ready, uint64_t now) {
    const auto epoch = configuration_epoch.load();
    if (!ready || mailbox_ready_epoch.load() != epoch) {
      mailbox_ready_since_ns.store(0);
      mailbox_ready_epoch.store(epoch);
    }
    if (ready && !mailbox_ready_since_ns.load()) mailbox_ready_since_ns.store(now);
  }
  bool mailbox_admitted(uint32_t axis, uint64_t epoch, uint64_t since, bool master_locked = false);
  std::atomic<uint64_t> configuration_epoch{1};
  std::atomic<uint64_t> home_epoch{1};
  std::atomic<uint64_t> dirty_configuration_epoch{1};
  std::atomic<uint64_t> verified_configuration_epoch{0};
  std::atomic<uint64_t> verification_request{0};
  bool verification_exhausted = false; // Cyclic owner, fail closed on exhaustion.
  uint32_t reverification_coordinate_mask = 0;
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> observed_revision{};
  std::atomic<uint32_t> observed_revision_valid_mask{0};
  std::atomic<uint32_t> safety_fault_mask{0};
  std::atomic<uint32_t> execution_fault_reasons{0};
  std::array<CollisionWatchdog, gradient::ipc::v1::GRADIENT_MAX_AXES> collision_watchdogs{};
  rosie::rtcore::FaultEvidenceArray fault_evidence{}; // Cyclic writer only.
  rosie::rtcore::FaultRecoverySnapshot recovery_state{};
  rosie::rtcore::AtomicSnapshot<rosie::rtcore::FaultRecoverySnapshot> recovery_snapshot;
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::BrakeHoldEvidence>,
      gradient::ipc::v1::GRADIENT_MAX_AXES> brake_hold_evidence;
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::TargetLeadEvidence>,
      gradient::ipc::v1::GRADIENT_MAX_AXES> target_lead_evidence;
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::DriveSupervisorEvidence>,
      gradient::ipc::v1::GRADIENT_MAX_AXES> drive_supervisor_evidence;
  std::atomic<uint64_t> arm_request_sequence{0};
  std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> bus_fault_held{};
  std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> fault_identity_revision{};
  uint32_t fault_identity_valid_mask = 0;
  uint32_t recovery_trusted_mask = 0;
  std::atomic<uint32_t> last_bus_failure_operation{0};
  std::atomic<int64_t> last_bus_failure_code{0};
  std::atomic<uint64_t> revoke_before_ns{0};
  const uint32_t configured_mask = (1u << opt.num_axes) - 1;
  std::atomic<bool> armed{false};
  std::atomic<uint32_t> axis_enable_mask{0};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> desired_mode_of_operation{};
  std::atomic<uint32_t> service_mode_axis_mask{0};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> service_mode_override{};
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> service_controlword_override{};
  std::array<rosie::rtcore::NativeHomeWireAudit, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_wire_audit{};
  // Explicit startup reset grants only a same-cycle CW128 receipt, never motion.
  uint32_t startup_reset_cycle_mask = 0;
  uint64_t startup_reset_generation = 0, startup_reset_deadline_ns = 0;
  uint64_t startup_reset_configuration = 0, startup_reset_home = 0;
  // One operator Reset's drive-alarm reset (drive_alarm_reset.hpp). Cyclic
  // owner only. It is bound to the Reset's generation, configuration, Home
  // epoch and Stop fence: any change ends it as interrupted with CW0.
  struct DriveAlarmResetAttempt {
    uint32_t axis_mask = 0, requested_mask = 0;
    uint64_t reset_sequence = 0, generation = 0, configuration = 0, home = 0, stop_request = 0;
    uint64_t deadline_ns = 0;
    uint32_t cycles = 0;
    std::array<rosie::rtcore::DriveAlarmResetPulse, gradient::ipc::v1::GRADIENT_MAX_AXES> pulses{};
    // The alarm each reset axis held at the press, and the refusals of the
    // same press on other axes; both become the press's per-axis outcome.
    uint32_t evidence_mask = 0;
    std::array<rosie::rtcore::DriveAlarmResetEvidence, gradient::ipc::v1::GRADIENT_MAX_AXES> evidence{};
  };
  static_assert(gradient::ipc::v1::GRADIENT_MAX_AXES <= rosie::rtcore::kRecoveryDriveAxes);
  DriveAlarmResetAttempt drive_alarm_attempt{};
  uint64_t drive_alarm_reset_events = 0; // Cyclic owner only.
  // Start of each axis's last attempt (rate limit); read by the publisher.
  std::array<std::atomic<uint64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_alarm_reset_last_ns{};
  std::atomic<uint32_t> drive_alarm_reset_active_mask{0};
  std::atomic<uint32_t> fault_reset_request{0}; // bitmask; helper thread -> RT thread
  std::atomic<uint64_t> trajectory_abort_request{0}; // 0 means none, UINT64_MAX means any active trajectory
  std::atomic<uint32_t> native_home_active_axis_mask{0};
  std::atomic<uint32_t> commissioning_phase{static_cast<uint32_t>(rosie::rtcore::CommissioningPhase::Idle)};
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::AnchorDeltaRestoreBinding>, gradient::ipc::v1::GRADIENT_MAX_AXES> anchor_delta_restores;
  struct RestoreAnchorEvidence {
    uint32_t version = 0;
    uint64_t home_epoch = 0, tolerance_counts = 0;
    int64_t captured_host_ns = 0;
    int32_t counts = 0, offset = 0, turns = 0;
  };
  struct CommissioningEnvelope {
    rosie::rtcore::CommissioningRequest request{};
    RestoreAnchorEvidence anchor{};
    bool restoring = false;
    bool verified_delta_restore = false;
    rosie::rtcore::SampledAbsolutePosition home_sample{};
    rosie::rtcore::DeltaAcquisitionEvidence reference_evidence{};
  };
  // Private, bounded failure evidence carried to the non-RT worker. It never
  // changes admission and does not add logging or mailbox work to the cycle.
  struct HomeCompletionDiagnostic {
    uint32_t axis = UINT32_MAX, qualification_mask = 0, coordinate_reason = 0;
    rosie::rtcore::IndependentBindRefusal bind_refusal{};
    uint32_t binding_sample_reason = 0;
    uint32_t acquisition_valid = 0, acquisition_reason = 0;
    uint64_t receipt_ns = 0, sample_started_ns = 0, sample_completed_ns = 0;
    uint64_t sample_frame = 0, history_frame = 0;
    uint64_t sample_configuration = 0, history_configuration = 0;
    uint64_t start_generation = 0, latest_generation = 0, stationary_generation = 0;
    uint64_t sample_cycle = 0, stationary_since_cycle = 0;
    int64_t stationary_min = 0, stationary_max = 0;
    uint32_t interval_valid = 0;
    int64_t interval_min = 0, interval_max = 0;
    int64_t anchor_counts = 0;
    uint64_t displacement_counts = 0, displacement_limit = 0;
  };
  struct CommissioningReceipt {
    uint64_t ticket = 0;
    rosie::rtcore::CommissioningIdentity identity{};
    rosie::rtcore::CommissioningReason reason = rosie::rtcore::CommissioningReason::Invalid;
    bool accepted = false;
    bool job_completed = false;
    HomeCompletionDiagnostic home_diagnostic{};
  };
  std::atomic<bool> jog_session_open{false}; // Cyclic observation for Home's close-before-reserve path.
  rosie::rtcore::SpscQueue<CommissioningEnvelope, 8> commissioning_requests;
  rosie::rtcore::SpscQueue<CommissioningReceipt, 8> commissioning_receipts;
  // Capture the cancelled job before revocation changes its generation. A
  // later job must never inherit an earlier cycle deadline as its failure.
  rosie::rtcore::AtomicSnapshot<rosie::rtcore::CommissioningIdentity> commissioning_deadline_identity;
  struct alignas(8) NativeHomeFailure {
    rosie::rtcore::CommissioningIdentity identity{};
    uint32_t axis = 0, step = 0, abort_code = 0;
    rosie::rtcore::CommissioningReason cancellation_reason = rosie::rtcore::CommissioningReason::None;
    // Unverified diagnostic storage caps; all current operation/reason names fit.
    char operation[32]{}, reason[64]{};
  };
  std::array<rosie::rtcore::AtomicSnapshot<NativeHomeFailure>, gradient::ipc::v1::GRADIENT_MAX_AXES> native_home_failures;
  struct HomeTruthReadback {
    uint64_t generation = 0, configuration_epoch = 0, home_epoch = 0, ticket = 0;
    uint32_t axis = 0;
    int32_t offset = 0;
    bool retained_anchor_valid = false;
    RestoreAnchorEvidence anchor{};
    rosie::rtcore::SampledAbsolutePosition sample{};
    uint64_t audit_generation = 0;
    uint32_t calibration_reason = 0;
    // Uploaded in this attempt, after its raw sample; see RetainedHomeDriveHealth.
    bool alarm_objects_read = false;
    uint16_t encoder_fault_state = 0, multi_turn_overflow = 0;
  };
  rosie::rtcore::SpscQueue<HomeTruthReadback, 32> home_truth_readbacks;
  std::array<std::atomic<uint64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> home_truth_processed{};
  std::array<std::atomic<uint64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> home_truth_applied{};
  // Last automatic retained-Home refusal per axis (RetainedHomeRefusal); the
  // commissioning worker logs changes. Diagnostic only, never admission.
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> retained_home_refusal{};
  rosie::rtcore::TrajectoryStore<TrajectoryPointRuntime> trajectory_store;
  rosie::rtcore::SpscQueue<rosie::protocol::CommandResult, 64> rt_results;
  uint64_t rt_result_count = 0; // RT owns the producer count.
  uint64_t rt_result_consumed = 0; // Helper owns the consumer count across connections.
  std::atomic<uint64_t> rt_results_committed{0};
  // Results become externally visible only after the producing cycle publishes
  // its motion snapshot, preventing a fast receipt beside pre-command state.
  bool enqueue_rt_result(const rosie::protocol::CommandResult& result) noexcept {
    if (!rt_results.push(result)) return false;
    ++rt_result_count;
    return true;
  }
  rosie::rtcore::AtomicSnapshot<gradient::ipc::v1::StatusMotionStateV1> motion_snapshot;
  rosie::rtcore::AtomicSnapshot<JogCommandRuntime> jog_commands;
  LatestFeedback latest_feedback{};
  std::array<rosie::rtcore::DriveDiagnosticPeaks, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_diagnostic_peaks{};
  LatestJogDebug latest_jog_debug{};
  std::atomic<uint32_t> motion_active_mode{gradient::ipc::v1::MOTION_MODE_IDLE};
  std::atomic<uint32_t> motion_exec_state{gradient::ipc::v1::EXEC_STATE_IDLE};
  std::atomic<uint64_t> motion_active_traj_id{0};
  std::atomic<uint32_t> motion_current_point_index{std::numeric_limits<uint32_t>::max()};
  std::atomic<uint32_t> motion_queue_depth{0};
  std::atomic<uint32_t> motion_last_event_code{0};
  std::atomic<uint32_t> motion_underrun_count{0};
  std::atomic<uint32_t> motion_stale_command_flag{0};
  std::atomic<uint32_t> motion_done{1};
  std::atomic<uint32_t> motion_capability_flags{
      gradient::ipc::v1::MOTION_CAP_TRAJECTORY_UPLOAD |
      gradient::ipc::v1::MOTION_CAP_JOG_COMMAND};
  std::atomic<uint64_t> motion_active_command_seq{0};
  std::atomic<uint64_t> motion_last_update_ns{0};
  std::atomic<uint64_t> trajectory_completion_diag_traj_id{0};
  std::atomic<uint64_t> trajectory_completion_diag_sample_time_ns{0};
  std::atomic<uint32_t> trajectory_completion_diag_axis_mask{0};
  std::atomic<uint32_t> trajectory_completion_diag_final_due{0};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES>
      trajectory_completion_diag_target_counts{};
  std::array<std::atomic<int32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES>
      trajectory_completion_diag_feedback_counts{};
  std::array<std::atomic<int64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES>
      trajectory_completion_diag_error_counts{};
  // Blocking IgH mailbox calls currently inside the kernel (MailboxCallScope).
  std::atomic<uint32_t> mailbox_calls_in_flight{0};
#if GRADIENT_HAVE_ECRT
  std::atomic<ec_master_t*> shared_master{nullptr};
  // Serialize non-RT SDO traffic against master teardown so the metrics/helper
  // threads cannot race ecrt_release_master() during stop/start cycles.
  std::mutex shared_master_sdo_mutex;
  // The cyclic owner takes this lock (try_lock, while still exchanging) before
  // its loop may end, and keeps it through master teardown.
  std::unique_lock<std::mutex> shutdown_mailbox{shared_master_sdo_mutex, std::defer_lock};
  // Published once after bus activation. The publisher's teardown mutex protects
  // these borrowed handles; no cyclic iteration acquires that mutex.
  std::array<ec_slave_config_t*, rosie::protocol::max_axes> diagnostic_sc{};
  std::atomic<bool> diagnostic_handles_ready{false};
  std::atomic<uint32_t> diagnostic_phase_requests{0};
  BusTopologyMonitor topology_monitor;
  rosie::rtcore::AtomicSnapshot<rosie::rtcore::BusTopologySnapshot> topology_snapshot;
  struct alignas(8) DriveDiagnostics {
    ec_master_state_t master{};
    std::array<ec_slave_config_state_t, rosie::protocol::max_axes> slaves{};
    uint64_t sampled_ns = 0;
    uint32_t responding_expected = 0;
    bool topology_admitted = true;
  };
  rosie::rtcore::AtomicSnapshot<DriveDiagnostics> drive_diagnostics;
  std::atomic<uint64_t> drive_diagnostics_generation{0};
  uint64_t diagnostic_generation_seen = 0;
  void collect_drive_diagnostics();
#endif

  std::atomic<uint64_t> stop_request_id{0}, stop_ack_id{0};
  std::atomic<int> process_exit_code{0};

  MachineState(const Options& options, OptionsParseResult& configuration)
      : opt(options), cell_io(options.cell_io), parsed(configuration), startup_sdos(parsed.startup_sdos),
        axis_profiles(parsed.axis_profiles), have_native_home(parsed.have_native_home),
        have_absolute_feedback(parsed.have_absolute_feedback),
        metrics_startup_readback_enabled(parse_env_flag("GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED", true)),
        metrics_native_home_refresh_enabled(parse_env_flag("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", true)),
        metrics_absolute_feedback_poll_enabled(parse_env_flag("GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED", true)) {
  for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    desired_mode_of_operation[i].store(8, std::memory_order_relaxed);
    service_mode_override[i].store(kNoModeOverride, std::memory_order_relaxed);
    service_controlword_override[i].store(0, std::memory_order_relaxed);
    if (axis_profiles[i].coordinate_evidence.sdo_sampled)
      sampled_bindings[i].configure(axis_profiles[i].coordinate_evidence.maximum_acquisition_age_ns, opt.cycle_ns);
  }
  jog_lane.bind_path(path_jog);

  }
  MachineState(const MachineState&) = delete;
  MachineState& operator=(const MachineState&) = delete;
  MachineState(MachineState&&) = delete;
  MachineState& operator=(MachineState&&) = delete;

  int run();
  void publish_status(gid_t ipc_gid);
  void drain_cycle_capture(FILE* file);
  void drain_telemetry();
  void queue_telemetry_dump(const rosie::rtcore::TelemetryDumpRequest& request);
  void run_telemetry_writer();
  void finish_telemetry();
  bool dump_telemetry(const rosie::rtcore::TelemetryDumpRequest& request, bool skipped = false);
  void run_helper(ConnectionState& connection);
  enum class ControlOperation {
    Inhibit, Revoke, Disconnect, Connect, Arm, IoArm, IoTest, Enable, Disable, SetMode,
    ResetFault, ResetPulse, PrepareHome, BeginService, EndService, AbortMotion, Halt
  };
  struct ControlRequest {
    ControlOperation operation = ControlOperation::Inhibit;
    uint32_t value = 0;
    uint64_t ticket = 0;
    uint64_t generation = 0;
    uint64_t configuration = 0;
    uint64_t home = 0;
    uint64_t job = 0;
    uint64_t reset_sequence = 0;
    bool legacy_reset = false;
    uint64_t application_generation = 0;
    bool io_begin=false;
    uint32_t io_index = 0;
    uint64_t io_sequence = 0, io_deadline = 0;
  };
  struct ControlResult {
    uint64_t ticket = 0;
    uint64_t generation = 0;
    uint32_t previous_mask = 0;
    uint32_t enabled_mask = 0;
    bool accepted = false;
    bool fault_persists = false;
    uint32_t reason = rosie::protocol::reason_not_ready;
  };
  struct ControlPort {
    // Unverified capacity; synchronous producers have at most one outstanding request.
    rosie::rtcore::SpscQueue<ControlRequest, 8> requests;
    rosie::rtcore::SpscQueue<ControlResult, 8> results;
    uint64_t next_ticket = 0; // Producer only.
    std::optional<ControlRequest> pending_halt; // Cyclic owner only.
  };
  struct ConfigurationReadback {
    uint64_t epoch = 0;
    uint32_t verified_mask = 0;
    uint32_t mapping_mismatch_mask = 0;
    uint64_t verification_request = 0;
  };
  ControlPort accept_controls, helper_controls, commissioning_controls, bulk_controls;
  rosie::rtcore::AtomicSnapshot<ConfigurationReadback> configuration_readbacks;
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::SampledAbsolutePosition>,
      gradient::ipc::v1::GRADIENT_MAX_AXES> sampled_absolute;
  std::array<rosie::rtcore::SampledPdoHistory, gradient::ipc::v1::GRADIENT_MAX_AXES> sampled_bindings{};
  std::array<rosie::rtcore::AtomicSnapshot<rosie::rtcore::SampleConsumption>, gradient::ipc::v1::GRADIENT_MAX_AXES> sampled_consumption;
  std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_samples{}, absolute_first_started_ns{};
  uint32_t absolute_next_axis = 0; // Mailbox owner, round-robin fairness.
  std::array<rosie::rtcore::SampledAbsolutePosition, gradient::ipc::v1::GRADIENT_MAX_AXES> sampled_at_receipt{};
  std::atomic<uint32_t> calibration_valid_mask{0}; // RT-owned; independent of tracking/fault recovery.
  std::atomic<uint64_t> audit_generation{1};
  std::atomic<uint32_t> absolute_stationary_mask{0};
  std::array<rosie::rtcore::CoordinateReason, gradient::ipc::v1::GRADIENT_MAX_AXES> recovery_position_reason{};
  std::array<std::atomic<uint64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_frame_epoch{};
  std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> absolute_last_poll_ns{}; // Mailbox owner only.
  std::atomic<bool> cycle_exited{false};
  inline static thread_local MachineState* cycle_owner = nullptr;
  inline static thread_local ControlPort* request_port = nullptr;
  inline static thread_local uint64_t (*cycle_clock)() = now_monotonic_ns;

  bool lease_is_valid_now() const noexcept {
    const uint64_t heartbeat = control_policy.heartbeat_ns.load(std::memory_order_acquire);
    const uint64_t now = cycle_owner == this ? cycle_clock() : now_monotonic_ns();
    return control_policy.connected.load(std::memory_order_acquire) && heartbeat != 0 &&
        now >= heartbeat && now - heartbeat < control_policy.lease_ns && control_policy.application_valid(now);
  }

  auto advance_revoke_floor(uint64_t time_ns) {
    auto before = revoke_before_ns.load(std::memory_order_acquire);
    while (time_ns > before && !revoke_before_ns.compare_exchange_weak(
        before, time_ns, std::memory_order_acq_rel)) {}
  }
  auto inhibit_outputs() {
    if (cycle_owner != this) { (void)request_control({ControlOperation::Inhibit}); return; }
    cell_io.fence(control_policy.generation.load(), application_grant.current.identity.app_grant_generation);
    advance_revoke_floor(cycle_clock());
    armed.store(false, std::memory_order_release);
    axis_enable_mask.store(0, std::memory_order_release);
    service_mode_axis_mask.store(0, std::memory_order_release);
    native_home_active_axis_mask.store(0, std::memory_order_release);
    trajectory_abort_request.store(std::numeric_limits<uint64_t>::max(), std::memory_order_release);
#if GRADIENT_HAVE_ECRT
    active_halt = false;
    halt_axis_mask = 0;
#endif
  }
  // Drive-state supervisor refusal: end only this Arm attempt. The grant,
  // generation and enable selection survive so the same controller can retry
  // Arm; nothing latches and no reset is required.
  void end_arm_attempt() {
    audit_generation.fetch_add(1);
    armed.store(false, std::memory_order_release);
    trajectory_abort_request.store(std::numeric_limits<uint64_t>::max(), std::memory_order_release);
#if GRADIENT_HAVE_ECRT
    active_halt = false;
    halt_axis_mask = 0;
#endif
  }
  auto revoke_outputs(bool telemetry_stop = false) {
    if (cycle_owner != this) { (void)request_control({ControlOperation::Revoke, telemetry_stop ? 1u : 0u}); return; }
    if (telemetry_stop) telemetry_pending |= 2;
    inhibit_outputs();
    control_policy.revoke();
  }
  auto disconnect_outputs() {
    if (cycle_owner != this) { (void)request_control({ControlOperation::Disconnect}); return; }
    // The RT failure path and socket owner can observe the same disconnect.
    // Only its first observer may fence it; both must inhibit immediately.
    const bool was_connected = control_policy.connected.exchange(false, std::memory_order_acq_rel);
    inhibit_outputs();
    if (was_connected) control_policy.revoke();
  }
  // Latch provenance before revoking Enable, so disabled-only bus recovery
  // cannot mistake the post-fault inhibited state for the state at failure.
  void latch_fault_evidence(uint32_t reason, uint32_t mask) {
    uint32_t prior_axes = 0;
    for (const auto& evidence : fault_evidence) prior_axes |= evidence.affected_axis_mask;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      const uint32_t bit = 1u << axis;
      if ((mask & ~prior_axes) & bit) {
        fault_identity_revision[axis] = observed_revision[axis].load();
        fault_identity_valid_mask = (fault_identity_valid_mask & ~bit) |
            (observed_revision_valid_mask.load() & bit);
      }
    }
    for (size_t bit = 0; bit < std::size(rosie::rtcore::kFaultRecovery); ++bit) {
      if (!(reason & (1u << bit))) continue;
      auto& evidence = fault_evidence[bit];
      const uint32_t added = mask & ~evidence.affected_axis_mask;
      telemetry_pending_faults[bit] |= added;
      evidence.affected_axis_mask |= mask;
      uint32_t disabled = 0;
      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
        const uint32_t axis_bit = 1u << axis;
        if ((added & axis_bit) && !(axis_enable_mask.load() & axis_bit) &&
            rosie::rtcore::position_output_inactive(gradient::ds402::decode_statusword(
                latest_feedback.statusword[axis]))) {
          disabled |= axis_bit;
#if GRADIENT_HAVE_ECRT
          if ((1u << bit) == rosie::rtcore::FaultBusTransport && have_last_wire_target[axis])
            bus_fault_held[axis] = last_wire_target[axis];
          else if ((1u << bit) == rosie::rtcore::FaultBusTransport)
            evidence.untrusted_axis_mask |= axis_bit;
#else
          evidence.untrusted_axis_mask |= axis_bit;
#endif
        }
      }
      evidence.disabled_axis_mask |= disabled;
      evidence.untrusted_axis_mask |= mask & ~recovery_trusted_mask;
      if ((1u << bit) == rosie::rtcore::FaultBusTransport)
        evidence.untrusted_axis_mask |= added & ~disabled;
      if (rosie::rtcore::kFaultRecovery[bit].invalidates_home)
        evidence.untrusted_axis_mask |= mask;
      control_policy.home_valid_mask.fetch_and(~evidence.untrusted_axis_mask);
    }
  }
  auto execution_fault(uint32_t reason, uint32_t mask) {
#if GRADIENT_HAVE_ECRT
    if ((reason & rosie::rtcore::FaultCycleDeadline) &&
        commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active)
      commissioning_deadline_identity.publish(commissioning_state.identity);
#endif
    latch_fault_evidence(reason, mask);
    execution_fault_reasons.fetch_or(reason, std::memory_order_acq_rel);
    safety_fault_mask.fetch_or(mask, std::memory_order_acq_rel);
    revoke_outputs();
    motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED);
    motion_last_event_code.store(gradient::ipc::v1::EVT_TRAJECTORY_FAULTED);
    motion_done.store(1);
  }
  auto record_bus_failure(const rosie::rtcore::CycleBusResults& results) {
    if (!results.failed()) return;
    last_bus_failure_operation.store(static_cast<uint32_t>(results.first_failure.operation));
    last_bus_failure_code.store(results.first_failure.return_code);
    if (bus_ready.exchange(false, std::memory_order_acq_rel)) {
      require_reverification();
    }
    if (!(execution_fault_reasons.load() & rosie::rtcore::FaultBusTransport))
      execution_fault(rosie::rtcore::FaultBusTransport, configured_mask);
  }
  auto axes_ready(uint32_t mask) {
    if (!mask || (mask & ~configured_mask) || !bus_ready.load(std::memory_order_acquire) ||
        commissioning_phase.load() != static_cast<uint32_t>(rosie::rtcore::CommissioningPhase::Idle) ||
        !lease_is_valid_now() || safety_fault_mask.load() ||
        verified_configuration_epoch.load() != configuration_epoch.load()) return false;
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      if ((mask & (1u << i)) && (latest_feedback.mode_display[i] != 8 || !axis_profiles[i].limits.valid ||
          (axis_profiles[i].coordinate_evidence.enabled && !latest_feedback.coordinate_valid[i]) ||
          !control_policy.axis_ready(i, axis_profiles[i].limits.require_home))) return false;
    }
    return true;
  }

  // One cyclic writer applies a complete request before publishing its result.
  // The generation/evidence fence applies only to permission-granting requests
  // (Arm on, Enable): delayed permission must not re-arm a successor. Requests
  // that remove permission or change no permission (disarm, PrepareHome,
  // BeginService, SetMode) always apply, exactly as the pre-extraction helper
  // wrote them; a racing epoch change must never leave a disarm unapplied.
  ControlResult apply_control(const ControlRequest& request) {
    ControlResult result{request.ticket, control_policy.generation.load(),
                         axis_enable_mask.load(), axis_enable_mask.load(), false};
    const bool current = request.generation == control_policy.generation.load() &&
        request.configuration == configuration_epoch.load() && request.home == home_epoch.load();
    switch (request.operation) {
      case ControlOperation::IoTest:
        if (!lease_is_valid_now() || !application_grant.active) { result.reason=rosie::protocol::reason_no_grant; return result; }
        if (!current || request.application_generation!=application_grant.current.identity.app_grant_generation) {
          result.reason=rosie::protocol::reason_wrong_generation; return result;
        }
        if (request.value && (armed.load() || axis_enable_mask.load() || !bus_ready.load() || safety_fault_mask.load())) {
          result.reason=rosie::protocol::reason_inhibited; return result;
        }
        if (!motion_done.load() || motion_queue_depth.load()) {
          result.reason=rosie::protocol::reason_mode_conflict; return result;
        }
        result.reason=cell_io.test(request.io_index, request.value, request.io_sequence, request.io_deadline, cycle_clock(), request.generation, request.application_generation, request.io_begin);
        if (result.reason) return result;
        break;
      case ControlOperation::IoArm:
        if (!request.value) { cell_io.disarm(); result.reason=rosie::protocol::reason_none; break; }
        if (!lease_is_valid_now() || !application_grant.active) { result.reason=rosie::protocol::reason_no_grant; return result; }
        if (!current || request.application_generation!=application_grant.current.identity.app_grant_generation) {
          result.reason=rosie::protocol::reason_wrong_generation; return result;
        }
        if (!bus_ready.load() || safety_fault_mask.load()) { result.reason=rosie::protocol::reason_inhibited; return result; }
        result.reason=cell_io.arm(control_policy.generation.load(),request.application_generation);
        if(result.reason) return result;
        break;
      case ControlOperation::Inhibit: inhibit_outputs(); break;
      case ControlOperation::Revoke: revoke_outputs(request.value != 0); break;
      case ControlOperation::Disconnect: disconnect_outputs(); break;
      case ControlOperation::Connect:
        control_policy.application_generation.store(0, std::memory_order_release);
        control_policy.connected.store(true, std::memory_order_release);
        break;
      case ControlOperation::Halt:
        if (!lease_is_valid_now() || !application_grant.active) {
          result.reason = rosie::protocol::reason_no_grant; return result;
        }
        if (!current || request.application_generation != application_grant.current.identity.app_grant_generation) {
          result.reason = rosie::protocol::reason_wrong_generation; return result;
        }
        result.reason = rosie::protocol::reason_inhibited;
#if GRADIENT_HAVE_ECRT
        if (!armed.load() || !axes_ready(axis_enable_mask.load()) ||
            service_mode_axis_mask.load() || native_home_active_axis_mask.load()) return result;
        if (!begin_halt(cycle_clock(), request.reset_sequence)) return result;
        result.reason = rosie::protocol::reason_none;
        break;
#else
        return result;
#endif
      case ControlOperation::Arm:
        if (request.value && cell_io.manual()) cell_io.disarm();
        if (request.value && (!current || !axes_ready(axis_enable_mask.load()))) return result;
        audit_generation.fetch_add(1);
        if (request.value) for (auto& peak : drive_diagnostic_peaks) peak.arm();
        if (request.value) arm_request_sequence.fetch_add(1, std::memory_order_acq_rel);
        armed.store(request.value != 0, std::memory_order_release);
        if (!request.value) revoke_outputs();
        break;
      case ControlOperation::Enable:
        if (!current || !axes_ready(request.value)) return result;
        audit_generation.fetch_add(1);
        axis_enable_mask.store(request.value, std::memory_order_relaxed);
        break;
      case ControlOperation::Disable:
        audit_generation.fetch_add(1);
        axis_enable_mask.store(axis_enable_mask.load() & ~request.value, std::memory_order_relaxed);
        control_policy.revoke();
        break;
      case ControlOperation::SetMode:
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
          if (request.value & (1u << axis)) desired_mode_of_operation[axis].store(8, std::memory_order_relaxed);
        break;
      case ControlOperation::ResetFault: {
        if (!request.legacy_reset && (armed.load() || service_mode_axis_mask.load() ||
            native_home_active_axis_mask.load() || commissioning_phase.load() != 0 ||
            !motion_done.load())) return result;
#if GRADIENT_HAVE_ECRT
        // One drive-alarm reset at a time: its outcome answers its own press.
        if (drive_alarm_attempt.axis_mask) return result;
#endif
        uint32_t startup_reset_mask = 0;
        uint64_t startup_reset_deadline = 0;
#if GRADIENT_HAVE_ECRT
        // A startup drive alarm may exist without a native execution fault.
        // Preserve the ordinary matrix for every latched fault. The additional
        // command is admitted only with the optional guard's never-enabled,
        // fixed-anchor witness and the caller's still-current authority.
        const uint64_t reset_now = cycle_clock();
        if (!request.legacy_reset && current && !armed.load() && !axis_enable_mask.load() &&
            !service_mode_axis_mask.load() && !native_home_active_axis_mask.load() &&
            commissioning_phase.load() == 0 && motion_done.load() &&
            !execution_fault_reasons.load() && !safety_fault_mask.load() &&
            lease_is_valid_now() && application_grant.active &&
            application_grant.current.identity.native_generation == request.generation &&
            (!request.application_generation || request.application_generation ==
                application_grant.current.identity.app_grant_generation) && bus_ready.load() &&
            !verification_exhausted && verified_configuration_epoch.load() == configuration_epoch.load()) {
          startup_reset_deadline = application_grant.current.deadline_host_ns;
          for (uint32_t axis=0; axis<opt.num_axes; ++axis) {
            const uint32_t bit = 1u << axis;
            const auto observed = latest_feedback.pdo_observed_time_ns[axis];
            if ((request.value & bit) && (application_grant.current.axis_mask & bit) &&
                (control_policy.config_verified_mask.load() & bit) &&
                (latest_feedback.pdo_fresh_axis_mask & bit) && observed &&
                observed <= reset_now && reset_now-observed <= 2*opt.cycle_ns &&
                gradient::ds402::decode_statusword(latest_feedback.statusword[axis]) == gradient::ds402::State::Fault &&
                axis_profiles[axis].brake.transition.enabled() &&
                brake_guards[axis].transition.startup_reset_eligible()) startup_reset_mask |= bit;
          }
        }
        // A drive in CiA402 Fault leaves it only on the fault-reset edge, so
        // drive_readiness can never clear "after the condition clears" while
        // the edge waits for that clearance (a Rosie 1430 arm, J5 Er41.1, 2026-10-11).
        // An axis whose alarm the drive profile classifies resettable gets the
        // drive's own reset; the press's outcome follows the drive's answer.
        // Classified before revocation: the press's own authority admits it.
        uint32_t alarm_candidates = 0;
        DriveAlarmResetAttempt alarm_attempt{};
        if (!request.legacy_reset) {
          const uint64_t alarm_now = cycle_clock();
          const bool authority = current && lease_is_valid_now() && application_grant.active &&
              application_grant.current.identity.native_generation == request.generation &&
              (!request.application_generation || request.application_generation ==
                  application_grant.current.identity.app_grant_generation);
          const bool bus = bus_ready.load() && !verification_exhausted &&
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              stop_request_id.load() == stop_ack_id.load() && !g_stop.load();
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            const uint32_t bit = 1u << axis;
            if (!(request.value & bit) || (startup_reset_mask & bit)) continue;
            const auto observation = drive_alarm_observation(axis, alarm_now);
            const auto state = rosie::rtcore::classify_drive_alarm_reset(axis_profiles[axis].drive_alarm_reset, observation);
            if (state == rosie::rtcore::DriveAlarmReset::None) continue;
            auto& evidence = alarm_attempt.evidence[axis];
            evidence.observed_ns = alarm_now;
            evidence.reset_sequence = request.reset_sequence;
            evidence.statusword = observation.statusword;
            evidence.error_code = observation.error_code;
            evidence.aux_code = observation.aux_code;
            evidence.aux_valid = observation.aux_valid;
            alarm_attempt.evidence_mask |= bit;
            if (state == rosie::rtcore::DriveAlarmReset::Possible) {
              // Without current authority, a healthy bus or this axis's
              // verified configuration the press keeps today's refusal.
              if (authority && bus && (control_policy.config_verified_mask.load() & bit)) alarm_candidates |= bit;
              else alarm_attempt.evidence_mask &= ~bit;
              continue;
            }
            evidence.result = state;
            evidence.retry_after_ns = rosie::rtcore::drive_alarm_reset_retry_after_ns(
                axis_profiles[axis].drive_alarm_reset, observation.last_attempt_ns, alarm_now);
          }
        }
#endif
        revoke_outputs();
        if (request.legacy_reset && !(execution_fault_reasons.load() & rosie::rtcore::FaultCollisionWatchdog)) {
          control_policy.home_valid_mask.store(0);
          safety_fault_mask.store(0);
          execution_fault_reasons.store(0);
          fault_evidence = {};
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
            if (request.value & (1u << axis)) collision_watchdogs[axis].reset();
          fault_reset_request.fetch_or(request.value, std::memory_order_relaxed);
          break;
        }
        // The current grant was retired above. Carry no renewed authority:
        // this receipt expires before the next cycle and Stop/new generation
        // invalidates it before submission. Acceptance is not alarm clearance.
        auto arm_startup_reset = [&]() {
          startup_reset_cycle_mask = startup_reset_mask;
          startup_reset_generation = control_policy.generation.load();
          startup_reset_deadline_ns = startup_reset_deadline;
          startup_reset_configuration = request.configuration;
          startup_reset_home = request.home;
        };
        const auto input = fault_recovery_input(request.value);
        const auto decision = rosie::rtcore::decide_fault_reset(input, fault_evidence);
#if GRADIENT_HAVE_ECRT
        if (alarm_candidates) {
          // Only the drives' own Fault may stand in the way; any other
          // persistent condition refuses the press without touching a drive.
          auto relaxed = input;
          relaxed.drive_fault_mask &= ~alarm_candidates;
          if (!rosie::rtcore::decide_fault_reset(relaxed, fault_evidence).persists) {
            arm_startup_reset();
            begin_drive_alarm_reset(alarm_attempt, alarm_candidates, request, cycle_clock());
            break;
          }
          alarm_attempt.evidence_mask &= ~alarm_candidates;
        }
#endif
        recovery_state.reset_sequence = request.reset_sequence;
        recovery_state.attempted = input.faults;
        recovery_state.reset_evidence = fault_evidence;
        for (auto& evidence : recovery_state.reset_evidence)
          evidence.affected_axis_mask &= request.value;
        recovery_state.reset = decision;
#if GRADIENT_HAVE_ECRT
        record_drive_alarm_results(alarm_attempt);
#endif
        if (decision.persists) {
          result.fault_persists = true;
          result.generation = control_policy.generation.load();
          result.enabled_mask = 0;
          publish_recovery();
          return result;
        }
        commit_fault_reset(decision, request.value);
        arm_startup_reset();
        publish_recovery();
        break;
      }
      case ControlOperation::ResetPulse:
        fault_reset_request.fetch_or(request.value, std::memory_order_relaxed);
        break;
      case ControlOperation::PrepareHome:
        if (jog_session_open.load(std::memory_order_acquire)) inhibit_outputs();
        result.previous_mask = axis_enable_mask.load(std::memory_order_relaxed);
        axis_enable_mask.store(result.previous_mask & ~request.value, std::memory_order_relaxed);
        if (!axis_enable_mask.load()) armed.store(false, std::memory_order_relaxed);
        break;
      case ControlOperation::BeginService:
#if GRADIENT_HAVE_ECRT
        if (armed.load() || !lease_is_valid_now() || !current || !request.job ||
            commissioning_state.phase != rosie::rtcore::CommissioningPhase::Active ||
            commissioning_state.identity.job != request.job ||
            configuration_write_job.load(std::memory_order_acquire) != request.job ||
            dirty_configuration_epoch.load() == UINT64_MAX || configuration_epoch.load() >= UINT64_MAX - 1)
          return result;
        dirty_configuration_epoch.fetch_add(1);
        revoke_outputs();
        control_policy.home_valid_mask.store(0);
        control_policy.config_verified_mask.store(0);
        configuration_epoch.fetch_add(1);
        break;
#else
        return result;
#endif
      case ControlOperation::EndService:
        control_policy.config_verified_mask.store(0);
        configuration_epoch.fetch_add(1);
        break;
      case ControlOperation::AbortMotion:
        trajectory_abort_request.store(std::numeric_limits<uint64_t>::max(), std::memory_order_release);
        break;
    }
    result.generation = control_policy.generation.load();
    result.enabled_mask = axis_enable_mask.load();
    result.accepted = true;
    return result;
  }

  // A cached complete-exchange bit is not proof of a current exchange after
  // a delayed cycle. Reset and its advisory use this same bounded observation.
  uint32_t recovery_fresh_feedback_mask() const {
    uint32_t fresh = 0;
    const uint64_t now = cycle_clock();
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      const uint32_t bit = 1u << axis;
      const uint64_t observed = latest_feedback.pdo_observed_time_ns[axis];
      if ((latest_feedback.pdo_fresh_axis_mask & bit) && observed && observed <= now &&
          now - observed <= 2 * opt.cycle_ns)
        fresh |= bit;
    }
    return fresh;
  }

  // The recovery matrix's inputs, from the cyclic owner's latest feedback.
  rosie::rtcore::FaultRecoveryInput fault_recovery_input(uint32_t requested) {
    rosie::rtcore::FaultRecoveryInput input{};
    input.faults = execution_fault_reasons.load();
    input.configured_mask = configured_mask;
    input.requested_mask = requested;
    input.fresh_mask = recovery_fresh_feedback_mask();
    input.trusted_mask = recovery_trusted_mask;
    input.home_valid_mask = control_policy.home_valid_mask.load();
    input.bus_healthy = bus_ready.load() && !verification_exhausted &&
        verified_configuration_epoch.load() == configuration_epoch.load() &&
        (control_policy.config_verified_mask.load() & requested) == requested;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      const uint32_t bit = 1u << axis;
      const auto& profile = axis_profiles[axis];
      if (collision_watchdogs[axis].clear) input.collision_clear_mask |= bit;
      const bool source = !profile.coordinate_evidence.enabled || latest_feedback.coordinate_source_valid[axis];
      const auto position = coordinate_model(axis, active_reference_offset(axis))
          .command_to_logical(latest_feedback.pos_counts[axis]);
      if (source && position.valid && profile.limits.valid)
        input.coordinate_source_mask |= bit;
      if (profile.independent_absolute_origin() && input.bus_healthy && !armed.load() &&
          !axis_enable_mask.load() && motion_done.load() && !(latest_feedback.statusword[axis] & 8u) &&
          !drive_error_faulted(axis, latest_feedback.error_code[axis], latest_feedback.manufacturer_error_code[axis]) &&
          latest_feedback.mode_display[axis] == 8) {
        rosie::rtcore::SampledAbsolutePosition proof{};
        if (sampled_absolute[axis].read(proof) && rosie::rtcore::independent_acquisition_current(proof,
            configuration_epoch.load(), absolute_frame_epoch[axis].load(), latest_feedback.pdo_observed_time_ns[axis],
            profile.coordinate_evidence.maximum_acquisition_age_ns))
          input.reference_recovery_mask |= bit;
      }
      if ((latest_feedback.statusword[axis] & 8u) || drive_error_faulted(axis,
              latest_feedback.error_code[axis], latest_feedback.manufacturer_error_code[axis]) ||
          latest_feedback.mode_display[axis] != 8)
        input.drive_fault_mask |= bit;
      const int64_t feedback = profile.coordinate_evidence.enabled
          ? int64_t(latest_feedback.coordinate_counts[axis]) : int64_t(latest_feedback.pos_counts[axis]);
      // Measured tolerance: one encoder count; compare against the command
      // held at failure, never the target reconciled during the outage.
      if (std::abs(feedback - int64_t(bus_fault_held[axis])) <= 1)
        input.held_match_mask |= bit;
    }
    return input;
  }

  // Apply a non-persisting reset decision (never a partial one).
  void commit_fault_reset(const rosie::rtcore::FaultResetDecision& decision, uint32_t requested) {
    fault_evidence = decision.remaining;
    uint32_t remaining_faults = 0, remaining_axes = 0;
    for (size_t bit = 0; bit < fault_evidence.size(); ++bit) {
      if (fault_evidence[bit].affected_axis_mask) remaining_faults |= 1u << bit;
      remaining_axes |= fault_evidence[bit].affected_axis_mask;
    }
    control_policy.home_valid_mask.fetch_and(~decision.rehome_required);
    safety_fault_mask.store(remaining_axes);
    execution_fault_reasons.store(remaining_faults);
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
      if ((requested & (1u << axis)) && !(remaining_axes & (1u << axis))) collision_watchdogs[axis].reset();
    fault_reset_request.fetch_or(decision.drive_reset_mask, std::memory_order_relaxed);
    if (decision.reverify) require_reverification();
  }

  // The drive-alarm admission inputs of one axis from the cyclic owner's
  // latest feedback (statusword and its age, 0x603F and the admitted 0x203F).
  rosie::rtcore::DriveAlarmResetObservation drive_alarm_observation(uint32_t axis, uint64_t now) {
    const uint32_t bit = 1u << axis;
    const uint64_t observed = latest_feedback.pdo_observed_time_ns[axis];
    const uint16_t error = latest_feedback.error_code[axis];
    const auto& alarm = drive_alarms[axis];
    const auto aux = alarm.snapshot(alarm.event.load(), configuration_epoch.load(), error);
    rosie::rtcore::DriveAlarmResetObservation in{};
    in.statusword = latest_feedback.statusword[axis];
    in.error_code = error;
    in.aux_code = aux.alarm_aux_code;
    in.aux_valid = aux.alarm_aux_valid;
    in.fresh = (latest_feedback.pdo_fresh_axis_mask & bit) && observed && observed <= now &&
        now - observed <= 2 * opt.cycle_ns;
    in.inhibited = !armed.load() && !axis_enable_mask.load() && !service_mode_axis_mask.load() &&
        !native_home_active_axis_mask.load() && commissioning_phase.load() == 0 && motion_done.load();
    in.now_ns = now;
    in.last_attempt_ns = drive_alarm_reset_last_ns[axis].load(std::memory_order_relaxed);
    return in;
  }

  // Put one press's per-axis drive-alarm results into its recovery snapshot.
  void record_drive_alarm_results(const DriveAlarmResetAttempt& attempt) {
    recovery_state.drive_alarm_axis_mask = attempt.evidence_mask;
    recovery_state.drive_alarm = {};
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (!(attempt.evidence_mask & (1u << axis))) continue;
      recovery_state.drive_alarm[axis] = attempt.evidence[axis];
      recovery_state.drive_alarm[axis].sequence = ++drive_alarm_reset_events;
    }
  }

#if GRADIENT_HAVE_ECRT
  // Start the drive's CiA402 fault reset on `mask`. The Reset's own
  // revocation has already advanced the generation; the attempt is bound to
  // that generation and to the current configuration, Home epoch and Stop
  // fence, and to the slowest axis's leave-fault time.
  void begin_drive_alarm_reset(DriveAlarmResetAttempt attempt, uint32_t mask, const ControlRequest& request,
                               uint64_t now) {
    attempt.axis_mask = mask;
    attempt.requested_mask = request.value;
    attempt.reset_sequence = request.reset_sequence;
    attempt.generation = control_policy.generation.load();
    attempt.configuration = configuration_epoch.load();
    attempt.home = home_epoch.load();
    attempt.stop_request = stop_request_id.load();
    attempt.cycles = 0;
    uint64_t timeout = 0;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (!(mask & (1u << axis))) continue;
      timeout = std::max(timeout, axis_profiles[axis].drive_alarm_reset.leave_fault_timeout_ns);
      attempt.pulses[axis].begin(kFaultResetPulseCycles);
      drive_alarm_reset_last_ns[axis].store(now, std::memory_order_relaxed);
    }
    attempt.deadline_ns = now + timeout;
    drive_alarm_attempt = attempt;
    drive_alarm_reset_active_mask.store(mask);
  }

  // End the attempt and answer its press. An axis is cleared only when its
  // drive left Fault on the edge and stayed out; then the ordinary recovery
  // matrix decides the rt-core faults on fresh feedback, exactly as a press
  // with no drive in Fault. Any axis still (or again) in Fault, or any
  // interruption, keeps every fault and reports fault_persists.
  void finish_drive_alarm_reset(bool interrupted, uint64_t now) {
    auto attempt = drive_alarm_attempt;
    drive_alarm_attempt = {};
    drive_alarm_reset_active_mask.store(0);
    bool failed = false;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (!(attempt.axis_mask & (1u << axis))) continue;
      const uint16_t sw = latest_feedback.statusword[axis];
      const auto& pulse = attempt.pulses[axis];
      auto& evidence = attempt.evidence[axis];
      evidence.observed_ns = now;
      evidence.statusword = sw;
      evidence.result = interrupted ? rosie::rtcore::DriveAlarmReset::Interrupted
          : pulse.out_of_fault(sw) ? rosie::rtcore::DriveAlarmReset::Cleared
          : pulse.refaulted ? rosie::rtcore::DriveAlarmReset::Refaulted
          : rosie::rtcore::DriveAlarmReset::PersistsAfterReset;
      failed |= evidence.result != rosie::rtcore::DriveAlarmReset::Cleared;
    }
    const auto input = fault_recovery_input(attempt.requested_mask);
    auto decision = rosie::rtcore::decide_fault_reset(input, fault_evidence);
    if (failed && !decision.persists) {
      decision = {};
      decision.remaining = fault_evidence;
      decision.persists = rosie::rtcore::FaultDriveReadiness;
    }
    recovery_state.reset_sequence = attempt.reset_sequence;
    recovery_state.attempted = input.faults;
    recovery_state.reset_evidence = fault_evidence;
    for (auto& evidence : recovery_state.reset_evidence)
      evidence.affected_axis_mask &= attempt.requested_mask;
    recovery_state.reset = decision;
    record_drive_alarm_results(attempt);
    if (!decision.persists) commit_fault_reset(decision, attempt.requested_mask);
    publish_recovery();
  }
#endif

  void publish_recovery() {
    recovery_state.evidence = fault_evidence;
    recovery_state.faults = execution_fault_reasons.load();
    recovery_state.home_valid_mask = control_policy.home_valid_mask.load();
    recovery_state.configured_mask = configured_mask;
    recovery_state.candidate = {};
    recovery_state.reset_preserves_home = false;
    // No additional coordinate conversion or clock read in the fault-free loop.
    if (recovery_state.faults) {
      const auto input = fault_recovery_input(configured_mask);
      recovery_state.candidate = rosie::rtcore::decide_fault_reset(input, fault_evidence);
      bool inhibited = !armed.load() && !axis_enable_mask.load() && !service_mode_axis_mask.load() &&
          !native_home_active_axis_mask.load() && commissioning_phase.load() == 0 && motion_done.load();
#if GRADIENT_HAVE_ECRT
      inhibited = inhibited && !drive_alarm_attempt.axis_mask;
#endif
      recovery_state.reset_preserves_home = rosie::rtcore::fault_reset_preserves_home(
          input, fault_evidence, recovery_state.candidate, inhibited);
    }
    recovery_snapshot.publish(recovery_state);
  }

  // Verification of unchanged configuration is not a coordinate identity
  // change. Fence cached readbacks with a separate request so trustworthy Home
  // survives recovery while every new permission waits for fresh verification.
  void require_reverification() {
#if GRADIENT_HAVE_ECRT
    // Keep observing already-qualified coordinates through a readback refresh.
    // This cannot establish a new anchor or retain a changed configuration.
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      const uint32_t bit = 1u << axis;
      if (verified_configuration_epoch.load() == configuration_epoch.load() &&
          (control_policy.config_verified_mask.load() & bit) && coordinate_reference[axis].valid &&
          coordinate_reference[axis].identity.configuration_epoch == configuration_epoch.load())
        reverification_coordinate_mask |= bit;
    }
#endif
    verified_configuration_epoch.store(0);
    control_policy.config_verified_mask.store(0);
    if (verification_request.load() == UINT64_MAX) verification_exhausted = true;
    else verification_request.fetch_add(1);
    readbacks_requested.store(true);
  }

  // Serialize only a coherent cyclic observation. The JSON extension keeps
  // reset provenance and recovery classes out of a second binary vocabulary.
  void write_recovery_json(std::ostream& out) {
    rosie::rtcore::FaultRecoverySnapshot snapshot{};
    if (!recovery_snapshot.read(snapshot)) { out << "null"; return; }
    auto faults = [&](uint32_t bits, const rosie::rtcore::FaultEvidenceArray& evidence, bool reset) {
      out << '[';
      bool first = true;
      for (size_t index = 0; index < evidence.size(); ++index) {
        const uint32_t bit = 1u << index;
        const auto mask = evidence[index].affected_axis_mask ? evidence[index].affected_axis_mask :
            (reset ? 0u : snapshot.configured_mask);
        if (!(bits & bit) || !mask) continue;
        if (!first) out << ',';
        first = false;
        const auto& row = rosie::rtcore::kFaultRecovery[index];
        const auto rehome = (reset ? snapshot.reset : snapshot.candidate).rehome_axis_masks[index] & mask;
        const char* outcome = !reset || !(snapshot.reset.cleared & bit) ? "persists" :
            rehome ? "rehome_required" : "cleared";
        out << "{\"bit\":" << index << ",\"name\":\"" << row.name
            << "\",\"axis_mask\":" << mask << ",\"recovery\":\"" << rosie::rtcore::recovery_name(row.recovery)
            << "\",\"outcome\":\"" << outcome << "\",\"rehome_axis_mask\":" << rehome;
        if (!reset) out << ",\"reset_preserves_home\":" << (snapshot.reset_preserves_home ? "true" : "false");
        out << '}';
      }
      out << ']';
    };
    out << "{\"home_valid_mask\":" << snapshot.home_valid_mask << ",\"faults\":";
    faults(snapshot.faults, snapshot.evidence, false);
    out << ",\"reset_sequence\":" << snapshot.reset_sequence << ",\"reason\":\""
        << (snapshot.reset.persists ? "fault_persists" : "") << "\",\"outcomes\":";
    faults(snapshot.attempted, snapshot.reset_evidence, true);
    // The press's drive alarms: one entry per axis whose drive was in Fault,
    // and the operator sentence of every failed one, joined. The adapter adds
    // the message to its fault_persists refusal.
    std::string message;
    out << ",\"drive_alarm_resets\":[";
    bool first = true;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      if (!(snapshot.drive_alarm_axis_mask & (1u << axis))) continue;
      const auto& e = snapshot.drive_alarm[axis];
      const auto name = drive_alarm_name(axis, uint16_t(e.error_code), e.aux_code, e.aux_valid != 0);
      const auto text = rosie::rtcore::drive_alarm_reset_message(e, axis, name.code, name.text);
      if (rosie::rtcore::drive_alarm_reset_failed(e.result) && !text.empty())
        message += (message.empty() ? "" : "; ") + text;
      if (e.sequence > logged_drive_alarm_event) {
        logf("drive_alarm_reset axis=%u joint=J%u reset_sequence=%llu event=%llu result=%s error_code=0x%04x "
             "aux_code=0x%04x aux_valid=%u alarm=\"%s\" statusword=0x%04x retry_after_ns=%llu message=\"%s\"",
             axis, axis + 1, static_cast<unsigned long long>(e.reset_sequence),
             static_cast<unsigned long long>(e.sequence), rosie::rtcore::drive_alarm_reset_name(e.result),
             e.error_code, e.aux_code & 0xffffu, e.aux_valid, name.code.c_str(), e.statusword,
             static_cast<unsigned long long>(e.retry_after_ns), text.c_str());
        logged_drive_alarm_event = e.sequence;
      }
      if (!first) out << ',';
      first = false;
      out << "{\"axis\":" << axis << ",\"result\":\"" << rosie::rtcore::drive_alarm_reset_name(e.result)
          << "\",\"error_code\":" << e.error_code << ",\"aux_code\":" << e.aux_code
          << ",\"aux_valid\":" << (e.aux_valid ? "true" : "false") << ",\"statusword\":" << e.statusword
          << ",\"retry_after_ns\":" << e.retry_after_ns << ",\"drive_alarm\":";
      json_quote(out, name.code);
      out << ",\"message\":";
      json_quote(out, text);
      out << '}';
    }
    out << "],\"message\":";
    json_quote(out, message);
    out << '}';
  }
  uint64_t logged_drive_alarm_event = 0; // Status publisher only.
  static void json_quote(std::ostream& out, const std::string& value) {
    out << '"';
    for (unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << c;
      else if (c >= 0x20) out << c;
      else { char escaped[7]; std::snprintf(escaped, sizeof(escaped), "\\u%04x", c); out << escaped; }
    }
    out << '"';
  }

  // Retain loss of trust throughout the latch, even if feedback later returns
  // to the old anchor. A reset may not erase unobserved motion or replacement.
  void observe_fault_trust() {
    recovery_trusted_mask = 0;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      const uint32_t bit = 1u << axis;
      const bool fresh = (latest_feedback.pdo_fresh_axis_mask & bit) != 0;
      const bool coordinate = axis_profiles[axis].coordinate_evidence.enabled
          ? latest_feedback.coordinate_valid[axis] != 0
          : true;
      if (fresh && coordinate) recovery_trusted_mask |= bit;
      for (size_t index = 0; index < fault_evidence.size(); ++index) {
        auto& evidence = fault_evidence[index];
        if (!(evidence.affected_axis_mask & bit)) continue;
        // An in-progress verification is gated by its request counter. Only
        // a new observed mismatch proves replacement; a cleared readback cache
        // must not itself destroy otherwise continuous position evidence.
        const bool identity = !(fault_identity_valid_mask & bit) ||
            !(observed_revision_valid_mask.load() & bit) ||
            fault_identity_revision[axis] == observed_revision[axis].load();
        // A disabled transport outage may qualify again from the fixed held
        // command. Every observed mismatch still permanently invalidates Home.
        const bool disabled_bus = (1u << index) == rosie::rtcore::FaultBusTransport &&
            (evidence.disabled_axis_mask & bit);
        if (!identity || !coordinate || (!fresh && !disabled_bus))
          evidence.untrusted_axis_mask |= bit;
        if (disabled_bus && fresh && !axis_profiles[axis].independent_absolute_origin()) {
          const int64_t feedback = axis_profiles[axis].coordinate_evidence.enabled
              ? int64_t(latest_feedback.coordinate_counts[axis]) : int64_t(latest_feedback.pos_counts[axis]);
          if (std::abs(feedback - int64_t(bus_fault_held[axis])) > 1)
            evidence.untrusted_axis_mask |= bit;
        }
        control_policy.home_valid_mask.fetch_and(~evidence.untrusted_axis_mask);
      }
    }
  }

  // Non-cyclic callers wait for application, never treating publication as acceptance.
  // Shutdown or exhausted transport fails closed instead of reviving a stale request.
  ControlResult request_control(ControlRequest request) {
    if (request.operation != ControlOperation::Halt && request.operation != ControlOperation::IoArm && request.operation != ControlOperation::IoTest)
      request.generation = control_policy.generation.load();
    request.configuration = configuration_epoch.load();
    request.home = home_epoch.load();
    if (cycle_owner == this) return apply_control(request);
    if (g_stop.load() || cycle_exited.load(std::memory_order_acquire)) return {};
    if (!request_port || request_port->next_ticket == UINT64_MAX) {
      g_stop.store(true);
      return {};
    }
    request.ticket = ++request_port->next_ticket;
    if (!request_port->requests.push(request)) {
      g_stop.store(true);
      return {};
    }
    while (!g_stop.load() && !cycle_exited.load(std::memory_order_acquire)) {
      ControlResult result{};
      if (request_port->results.pop(result)) {
        if (result.ticket == request.ticket) return result;
      } else {
        std::this_thread::yield();
      }
    }
    return {};
  }

  // Bounded input consumption leaves all motion, Home and permit writes on RT.
  // Old readbacks cannot restore verification after a configuration epoch changed.
  void consume_control_requests() {
    ConfigurationReadback readback{};
    if (!verification_exhausted && configuration_readbacks.read(readback) &&
        readback.epoch == configuration_epoch.load() &&
        readback.verification_request == verification_request.load()) {
      if (readback.mapping_mismatch_mask &&
          !(execution_fault_reasons.load() & rosie::rtcore::FaultPdoMappingMismatch))
        execution_fault(rosie::rtcore::FaultPdoMappingMismatch, readback.mapping_mismatch_mask);
      control_policy.config_verified_mask.store(readback.mapping_mismatch_mask ? 0 : readback.verified_mask,
                                                std::memory_order_release);
      verified_configuration_epoch.store(readback.epoch, std::memory_order_release);
      reverification_coordinate_mask &= ~readback.verified_mask;
    }
    for (auto* port : {&accept_controls, &helper_controls, &commissioning_controls, &bulk_controls}) {
      // Unverified bound; match the bounded request port capacity.
      for (unsigned consumed = 0; consumed < 8; ++consumed) {
        ControlRequest request{};
        if (!port->requests.pop(request)) break;
        if (request.operation == ControlOperation::Halt) {
          // Synchronous producers cannot submit a successor before the receipt.
          // Other ports, including urgent Stop, continue before readiness.
          port->pending_halt = request;
          break;
        }
        if (!port->results.push(apply_control(request))) {
          revoke_outputs();
          g_stop.store(true);
          break;
        }
      }
    }
  }

  // Admit only against this exchange. A readiness fence takes precedence over
  // the generation/lease changes it causes, so its receipt reports inhibited.
  void complete_halt_requests(uint32_t ready_mask) {
    for (auto* port : {&accept_controls, &helper_controls, &commissioning_controls, &bulk_controls}) {
      if (!port->pending_halt) continue;
      const auto request = *port->pending_halt;
      port->pending_halt.reset();
      ControlResult result{request.ticket, control_policy.generation.load(),
                           axis_enable_mask.load(), axis_enable_mask.load(), false};
      if (!bus_ready.load() || safety_fault_mask.load() ||
          !rosie::rtcore::motion_group_ready(axis_enable_mask.load(), configured_mask, ready_mask))
        result.reason = rosie::protocol::reason_inhibited;
      else
        result = apply_control(request);
      if (!port->results.push(result)) {
        revoke_outputs();
        g_stop.store(true);
      }
    }
  }

  // Capture motion content at a supplied cyclic observation point. Sampling
  // time stays independent of commands so idle receipts cannot stale clients.
  gradient::ipc::v1::StatusMotionStateV1 motion_status(uint64_t observed_ns) const noexcept {
    gradient::ipc::v1::StatusMotionStateV1 motion{};
    motion.active_mode = motion_active_mode.load(std::memory_order_relaxed);
    motion.state = motion_exec_state.load(std::memory_order_relaxed);
    motion.active_traj_id = motion_active_traj_id.load(std::memory_order_relaxed);
    motion.current_point_index = motion_current_point_index.load(std::memory_order_relaxed);
    motion.queue_depth = motion_queue_depth.load(std::memory_order_relaxed);
    motion.queue_capacity = kMaxTrajectoryPoints;
    motion.last_event_code = motion_last_event_code.load(std::memory_order_relaxed);
    motion.underrun_count = motion_underrun_count.load(std::memory_order_relaxed);
    motion.stale_command_flag = motion_stale_command_flag.load(std::memory_order_relaxed);
    motion.motion_done = motion_done.load(std::memory_order_relaxed);
    motion.capability_flags = motion_capability_flags.load(std::memory_order_relaxed);
    motion.active_command_seq = motion_active_command_seq.load(std::memory_order_relaxed);
    motion.last_update_ns = observed_ns;
    return motion;
  }

  int32_t active_reference_offset(uint32_t axis) const noexcept {
    return int32_t(latest_feedback.native_home_position_offset[axis]);
  }

  rosie::rtcore::AxisCoordinateModel coordinate_model(uint32_t axis, double offset) const noexcept {
    const auto& a = opt.axis[axis];
    return rosie::rtcore::AxisCoordinateModel({a.counts_per_unit, a.gear_ratio, a.sign,
        a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY, a.lead_m_per_rev, offset,
        axis_profiles[axis].coordinate_evidence.enabled});
  }

  enum class CommissioningJobKind { Home, RestoreAnchor, Write, Readback, RestoreReferenceVerifiedDelta };
  // The native 1.6 axis mask has 16 physical bits. Bit 31 selects explicit
  // persisted-anchor restoration; flags remain zero and payload stays 8 bytes.
  static constexpr uint32_t restore_anchor_selector = uint32_t{1} << 31;
  std::array<RestoreAnchorEvidence, gradient::ipc::v1::GRADIENT_MAX_AXES> restore_evidence{};
  std::atomic<uint64_t> restore_anchor_job{0};
  std::atomic<uint64_t> verified_delta_restore_job{0};
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> anchor_reason{};
  std::array<std::atomic<uint64_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> restored_anchor_home_epoch{};
  std::array<std::atomic<uint32_t>, gradient::ipc::v1::GRADIENT_MAX_AXES> observed_serial{};
  std::atomic<uint32_t> observed_serial_valid_mask{0}, observed_serial_attempted_mask{0};
  // Commissioning-worker progress through every axis's startup_observe list;
  // reset with each startup epoch, like the startup readback.
  struct StartupObserveProgress {
    uint32_t axis = 0;
    size_t entry = 0;
    uint32_t attempts = 0;
    uint32_t reads = 0;
    uint32_t failed = 0;
    int64_t last_value = 0;
    bool begun = false;
    bool complete = false;
  };
  StartupObserveProgress startup_observe_progress{};
  // At most one admitted upload per call; true once every entry is journalled.
  bool startup_observe_step();
  // Codes are local metrics values; native 1.6 wire reason values are unchanged.
  static const char* anchor_reason_text(uint32_t reason) noexcept {
    switch (reason) {
      case 1: return "anchor_missing";
      case 2: return "anchor_identity_mismatch";
      case 3: return "anchor_source_invalid";
      case 4: return "anchor_disagrees";
      default: return "anchor_ready";
    }
  }
  uint32_t load_restore_anchor(uint32_t axis, RestoreAnchorEvidence& evidence, std::string* changed_field = nullptr);
  struct CommissioningJob {
    CommissioningJobKind kind = CommissioningJobKind::Readback;
    rosie::rtcore::CommissioningIdentity identity{};
    uint64_t dirty_epoch = 0;
    rosie::protocol::CommandResult result{};
    gradient::ipc::v1::CmdNativeHomeV1 home{};
    gradient::ipc::v1::CmdServiceSdoWriteV1 write{};
  };
  struct CommissioningJobReceipt {
    rosie::rtcore::CommissioningIdentity identity{};
    uint64_t dirty_epoch = 0;
    rosie::protocol::CommandResult result{};
    CommissioningJobKind kind = CommissioningJobKind::Readback;
  };
  struct BulkJob {
    uint64_t generation = 0, reset_epoch = 0;
    // Measured largest bulk command: 288-byte point plus 24-byte header.
    alignas(8) std::array<uint8_t, sizeof(gradient::ipc::v1::TrajectoryPointV1)+sizeof(gradient::ipc::v1::MsgHeader)> message{};
  };
  // Unverified queue capacities. Exclusive admission permits one mailbox job;
  // bulk's 1024 records hold 319488 command bytes plus identity words.
  rosie::rtcore::SpscQueue<CommissioningJob, 1> commissioning_jobs;
  rosie::rtcore::SpscQueue<CommissioningJobReceipt, 64> commissioning_job_receipts;
  rosie::rtcore::SpscQueue<BulkJob, 1024> bulk_jobs;
  rosie::rtcore::SpscQueue<rosie::protocol::CommandResult, 256> bulk_results;
  std::atomic<bool> commissioning_running{false}, bulk_running{false};
  // Distinct from the cancellation request: a blocked SDO still needs cyclic
  // traffic after commissioning_running becomes false.
  std::atomic<bool> commissioning_exited{true};
  // Publish ownership and purpose together. Separate busy/readback flags expose
  // background work as an operator conflict during both claim and release.
  enum class CommissioningClaim { Idle, Command, Readback };
  std::atomic<CommissioningClaim> commissioning_claim{CommissioningClaim::Idle};
  // Release only this worker's ownership, including early exits. Explicit
  // release precedes its receipt; destruction cannot clear a successor claim.
  struct CommissioningClaimRelease {
    std::atomic<CommissioningClaim>* claim;
    void release() noexcept {
      if (claim) {
        claim->store(CommissioningClaim::Idle, std::memory_order_release);
        claim = nullptr;
      }
    }
    ~CommissioningClaimRelease() { release(); }
  };
  std::atomic<bool> readbacks_requested{true};
  std::atomic<uint64_t> bulk_reset_epoch{0};
  std::atomic<uint64_t> configuration_write_job{0};
  uint64_t commissioning_ticket = 0, home_truth_ticket = 0;
  std::thread commissioning_thread, bulk_thread;

  // Claim before publication: queued jobs already exclude every other mailbox
  // producer, preventing a second command from waiting behind a blocked SDO.
  bool try_claim_commissioning(CommissioningClaim kind = CommissioningClaim::Command,
                               CommissioningClaim* held = nullptr) noexcept {
    auto idle = CommissioningClaim::Idle;
    const bool claimed = commissioning_claim.compare_exchange_strong(idle, kind, std::memory_order_acq_rel);
    if (held) *held = idle;
    return claimed;
  }
  // Every captured epoch must still match before a queued job may touch a
  // mailbox. A stale receipt cannot authorize a successor configuration.
  bool commissioning_job_current(const CommissioningJob& job) const noexcept {
    return job.identity.generation == control_policy.generation.load() &&
        job.identity.configuration_epoch == configuration_epoch.load() &&
        job.identity.home_epoch == home_epoch.load() &&
        job.dirty_epoch == dirty_configuration_epoch.load();
  }
  CommissioningJob readback_job() const noexcept {
    CommissioningJob job{};
    job.identity = {now_monotonic_ns(), control_policy.generation.load(),
                    configuration_epoch.load(), home_epoch.load()};
    job.dirty_epoch = dirty_configuration_epoch.load();
    return job;
  }
  void run_commissioning_worker();
  template<class Request>
  bool native_home_axis(uint32_t axis, const rosie::rtcore::CommissioningIdentity& identity,
                        Request&& request);
  void run_bulk_worker();
  void start_commissioning_worker() {
    commissioning_exited.store(false, std::memory_order_release);
    commissioning_running.store(true);
    commissioning_thread = std::thread([this]() {
      struct ExitAcknowledgement {
        std::atomic<bool>& exited;
        ~ExitAcknowledgement() { exited.store(true, std::memory_order_release); }
      } acknowledge{commissioning_exited};
      run_commissioning_worker();
    });
  }
  void join_connection_workers() {
    commissioning_running.store(false);
    bulk_running.store(false);
    if (commissioning_thread.joinable()) commissioning_thread.join();
    if (bulk_thread.joinable()) bulk_thread.join();
    CommissioningJob abandoned{};
    while (commissioning_jobs.pop(abandoned)) {}
    commissioning_claim.store(CommissioningClaim::Idle, std::memory_order_release);
  }

  bool write_sdo_scalar_axis(uint32_t axis,
                                   uint16_t index,
                                   uint8_t subindex,
                                   SdoScalarType type,
                                   int32_t value,
                                   uint32_t* abort_out,
                                   const rosie::rtcore::CommissioningIdentity* home_identity = nullptr);
  bool read_sdo_scalar_axis(uint32_t axis,
                                  uint16_t index,
                                  uint8_t subindex,
                                  SdoScalarType type,
                                  int32_t* value_out,
                                  uint32_t* abort_out,
                                  bool* attempted_out = nullptr);
  bool read_native_home_truth_axis(uint32_t axis, int32_t* truth_out);
  MappingReadback verify_axis_mapping(uint32_t axis);
  bool sample_absolute_axis(uint32_t axis, uint64_t (*clock)() = now_monotonic_ns);
  void sample_absolute_due();
  bool read_absolute_feedback_field_axis(uint32_t axis,
                                               uint16_t index,
                                               uint8_t subindex,
                                               SdoScalarType type,
                                               AbsoluteFeedbackFieldSample* out);

    rosie::rtcore::ApplicationGrantState application_grant{};
    rosie::rtcore::JogLane jog_lane;
    // The one path jog generator (protocol 1.14). Its 1024-sample ring is
    // about 170 KiB, so it lives here, in the heap-allocated machine object,
    // never on the cyclic thread's stack. jog_lane owns its admission.
    rosie::rtcore::PathJog path_jog;
    std::array<double, rosie::rtcore::jog_session_axes> jog_velocity_limits{};
    // Bounded cadence evidence, not a measurement of slave clock offset.
    uint64_t dc_reference_sync_attempts = 0, dc_reference_sync_time_ns = 0;
    uint64_t dc_reference_sync_interval_ns = 0, dc_reference_sync_max_interval_ns = 0;
    uint64_t grant_connection_id = 0;
    uint64_t grant_publication_seen = 0;
    uint64_t grant_request_ticket = 0;
    uint32_t grant_request_reason = 0;
    uint64_t failed_connection_id = 0;
    rosie::rtcore::ConnectionFailureReason failed_connection_reason = rosie::rtcore::ConnectionFailureReason::None;

    const uint64_t period = opt.cycle_ns;
    uint64_t next_ns = 0;
    uint64_t previous_cycle_ns = 0;
    bool shutdown_active = false;
    uint64_t shutdown_started_ns = 0;
    uint64_t shutdown_until_ns = 0;
    bool shutdown_drain_failure_reported = false;
#if GRADIENT_HAVE_ECRT
    // Grace window to push DS402 disable (controlword=0) before dropping the bus.
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kShutdownGraceNs = 250000000ULL; // 250 ms
    uint64_t observed_generation = control_policy.generation.load();
    bool previous_lease_valid = false;
    bool active_trajectory = false;
    bool active_jog = false;
    bool active_halt = false, halt_admitted_this_cycle = false;
    uint32_t halt_axis_mask = 0;
    std::array<rosie::rtcore::JogAxis, rosie::protocol::max_axes> halt_axes{}, trajectory_command_axes{};
    std::array<rosie::rtcore::HaltRamp, rosie::protocol::max_axes> halt_ramps{};
    rosie::rtcore::CommissioningState commissioning_state{};
    uint64_t active_traj_id_rt = 0;
    int active_plan_slot = -1;
    uint64_t active_jog_seq_seen = 0;
    uint64_t active_jog_cmd_seq = 0;
    uint64_t active_jog_deadline_ns = 0;
    uint64_t latest_jog_timeout_ns = 0;
    uint64_t active_traj_start_ns = 0;
    rosie::rtcore::StartAlignment<rosie::protocol::max_axes> start_alignment{};
    rosie::rtcore::CompletionState completion_state{};
    uint64_t active_traj_cmd_seq = 0;
    uint64_t last_jog_stop_time_ns = 0;
    uint64_t last_jog_stop_cmd_seq = 0;
    uint32_t active_traj_point_count = 0;
    uint32_t next_io_point = UINT32_MAX, next_wait_point=UINT32_MAX;
    rosie::rtcore::IoWait input_wait;
    uint32_t active_jog_axis_mask = 0;
    uint32_t latest_jog_axis_mask = 0;
    uint32_t latest_jog_flags = 0;
    uint32_t last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_NONE;
    uint32_t last_jog_stop_axis_mask = 0;
    const TrajectoryPointRuntime* active_traj_points = nullptr;
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> active_jog_velocity_counts_per_s{};
    std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_target_counts_float{};
    std::array<rosie::rtcore::JogAxis, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_axes{};
    // Jerk smoothing state for jog_axes and halt_axes (machine jog_jerk). A
    // Halt that takes over a jog continues the jog's smoother; one that takes
    // over a trajectory seeds the halt smoother from the trajectory's state.
    std::array<rosie::rtcore::JogSmoother, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_smoothers{}, halt_smoothers{};
    std::array<rosie::rtcore::FollowingErrorGuard, gradient::ipc::v1::GRADIENT_MAX_AXES> following_error_guards{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> have_jog_target{};

    // Transfer commanded state before retiring immutable plan storage. Halt
    // never grants enable, resets a guard, or changes the authority generation.
    // Cyclic writer only; target_lead_evidence is the published copy.
    std::array<rosie::rtcore::TargetLeadEvidence, gradient::ipc::v1::GRADIENT_MAX_AXES> target_lead_excursion{};
    // A target leading feedback beyond its speed-scaled max_target_lead is a
    // warning, never a fault (see JogAxis::advance). Publish once when an
    // excursion starts and once when it clears, so the 1 kHz cycle never floods
    // the logger. A gap of more than two cycles without a step starts a new
    // excursion. The bound is the one the step was judged against.
    void note_target_lead(uint32_t axis, const rosie::rtcore::JogAxis& output,
                          uint64_t now, uint32_t source) noexcept {
      auto& e = target_lead_excursion[axis];
      const bool open = e.sequence && !e.cleared && now - e.last_ns <= 2 * opt.cycle_ns;
      if (output.lead_exceeded) {
        const double magnitude = std::abs(output.lead);
        if (!open) {
          e = {e.sequence + 1, now, now, 0, output.lead, magnitude, output.lead_limit, source, 0,
               output.velocity};
          target_lead_evidence[axis].publish(e);
        } else {
          e.last_ns = now;
          e.peak_counts = std::max(e.peak_counts, magnitude);
        }
      } else if (open) {
        e.cleared = 1;
        e.cleared_ns = now;
        target_lead_evidence[axis].publish(e);
      }
    }

    bool begin_halt(uint64_t now, uint64_t sequence) noexcept {
      if (active_halt) {
        // A new receipt correlates to the existing ramp without extending it.
        motion_active_command_seq.store(sequence);
        return true;
      }
      if (active_jog && jog_lane.path_mode()) {
        // Halt on a path session stops along the path within the configured
        // limits (PathJog::halt), never with per-axis ramps that would bend
        // it. The session keeps the axes until that stop completes. Before
        // any geometry nothing moved and the session simply closes.
        jog_lane.halt_path(now);
        if (!jog_lane.state().open) {
          jog_session_open.store(false, std::memory_order_release);
          active_jog = false;
          active_jog_axis_mask = 0; active_jog_deadline_ns = 0;
          have_jog_target.fill(false);
        }
        motion_active_command_seq.store(sequence);
        return true;
      }
      const uint32_t mask = (active_trajectory ? active_traj_points[0].axis_mask : 0) |
          (active_jog ? active_jog_axis_mask : 0);
      // Idle Halt is receipt-only: no motion state/event/command sequence,
      // integrator, hold target or generation changes when no source is active.
      if (!mask) return true;
      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
        if (!(mask & (1u << axis))) continue;
        auto& output = halt_axes[axis];
        output = active_trajectory ? trajectory_command_axes[axis] : jog_axes[axis];
        if (active_trajectory && halt_smoothers[axis].enabled()) {
          output.smoother = &halt_smoothers[axis];
          halt_smoothers[axis].seed(output.position, output.velocity);
        }
        if (!active_trajectory && !have_jog_target[axis])
          output.reset(have_hold[axis] ? hold_target_counts[axis] : int32_t(latest_feedback.pos_counts[axis]));
        const auto& profile = axis_profiles[axis];
        const double acceleration = active_trajectory && profile.max_acceleration
            ? *profile.max_acceleration : profile.limits.jog_acceleration;
        if (!halt_ramps[axis].begin(output,
            std::abs(coordinate_model(axis, 0).logical_delta_to_counts(acceleration)), period, now)) {
          execution_fault(rosie::rtcore::FaultTargetLead, mask);
          return false;
        }
      }
      jog_lane.close_for_halt();
      jog_session_open.store(false, std::memory_order_release);
      active_trajectory = active_jog = false;
      active_jog_axis_mask = 0; active_jog_deadline_ns = 0;
      have_jog_target.fill(false);
      if (active_plan_slot >= 0) trajectory_store.retire(active_plan_slot);
      active_plan_slot = -1; active_traj_points = nullptr; active_traj_point_count = 0;
      halt_axis_mask = mask; active_halt = mask != 0;
      halt_admitted_this_cycle = true;
      motion_active_command_seq.store(sequence);
      motion_active_traj_id.store(0);
      motion_current_point_index.store(UINT32_MAX);
      motion_last_event_code.store(gradient::ipc::v1::EVT_TRAJECTORY_ABORTED);
      motion_queue_depth.store(0);
      motion_exec_state.store(active_halt ? gradient::ipc::v1::EXEC_STATE_EXECUTING : gradient::ipc::v1::EXEC_STATE_ABORTED);
      motion_done.store(!active_halt);
      motion_last_update_ns.store(now);
      return true;
    }
#endif

#if GRADIENT_HAVE_ECRT
    std::atomic<ec_master_t*>& master = shared_master;
#endif

#if GRADIENT_HAVE_ECRT
    // -----------------------------------------------------------------------
    // IgH libecrt setup (descriptor-driven EtherCAT DS402 bring-up)
    // -----------------------------------------------------------------------
    //
    // NOTE: This block compiles only when IgH headers are present. It is
    // intentionally "init-only" work; the cyclic loop below avoids allocation.
    ec_master_state_t master_diag_state{};
    uint32_t topology_responding_expected = opt.num_axes;
    bool topology_admitted = true;
    ec_domain_state_t domain_state{};
    std::array<ec_slave_config_state_t, gradient::ipc::v1::GRADIENT_MAX_AXES> sc_state{};

    std::array<rosie::rtcore::BrakeMachine, gradient::ipc::v1::GRADIENT_MAX_AXES> brakes{};
    std::array<rosie::rtcore::BrakeOutput, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_outputs{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> release_delay_motion_permitted{};
    std::array<rosie::rtcore::DisabledBrakeGuard, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_guards{};
    // Cyclic owner's copy of the latest warning; republished as it settles.
    std::array<rosie::rtcore::BrakeHoldEvidence, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_warning_evidence{};
    std::array<rosie::rtcore::DriveStateSupervisor, gradient::ipc::v1::GRADIENT_MAX_AXES> drive_supervisors{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> hold_target_counts{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> have_hold{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> enable_hold_latched{};
    std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> last_wire_target{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> have_last_wire_target{};
    std::array<rosie::rtcore::BrakeHoldCommand, gradient::ipc::v1::GRADIENT_MAX_AXES> last_brake_hold_command{};
    struct ArmOpeningStage {
      // Native/application/configuration/Home fences; independent of service
      // identities and renewable lease deadlines. One token per enable attempt.
      rosie::rtcore::JogIdentity identity{};
      uint64_t token = 0, frame_epoch = 0, coordinate_epoch = 0;
      int32_t target = 0;
      bool completed = false, cancelled = false;
      rosie::rtcore::ArmCancelCause cancel_cause = rosie::rtcore::ArmCancelCause::None;
    };
    std::array<ArmOpeningStage, gradient::ipc::v1::GRADIENT_MAX_AXES> arm_opening_stages{};
    uint64_t next_brake_opening_token = 0;

    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_coordinate_epoch{};
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_receipt_id{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_receipt_exhausted{};
    std::array<rosie::rtcore::CoordinateReference, gradient::ipc::v1::GRADIENT_MAX_AXES> coordinate_reference{};
    std::array<rosie::rtcore::PositionRateGuard, gradient::ipc::v1::GRADIENT_MAX_AXES> position_rate_guard{};
    std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> position_rate_active{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_stop_quick_stop_cycles_left{};
    std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> jog_stop_arrest_cycles_left{};
    std::array<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> fault_reset_left{};
    struct SlaveDiagSnapshot {
      bool valid = false;
      bool online = false;
      bool operational = false;
      uint8_t al_state = 0;
    };
    struct MasterDiagSnapshot {
      bool valid = false;
      bool link_up = false;
      unsigned int responding = 0;
      unsigned int al_states = 0;
      unsigned int domain_wc_state = 0;
      unsigned int domain_working_counter = 0;
      uint32_t online_slaves = 0;
      uint32_t operational_slaves = 0;
    };
    std::array<SlaveDiagSnapshot, gradient::ipc::v1::GRADIENT_MAX_AXES> prev_slave_diag{};
    MasterDiagSnapshot prev_master_diag{};
    // Unverified reset pulse: 20 cycles; wall-clock width depends on --cycle-ns.
    static constexpr uint8_t kFaultResetPulseCycles = 20;
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kStartupLogIntervalNs = 1000000000ULL; // 1s
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kStartupResetDelayNs = 1000000000ULL;  // 1s
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kStartupDetailedDiagBaseNs = 1500000000ULL;       // 1.5s
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kStartupDetailedDiagPostResumeNs = 1000000000ULL; // 1.0s
    // Unverified existing bound; qualification has not established this value.
    static constexpr uint64_t kStartupTimeoutMinNs = 5000000000ULL;             // 5s
    const uint64_t startup_timeout_ns =
        std::max<uint64_t>(
            kStartupTimeoutMinNs,
            (static_cast<uint64_t>(opt.preop_to_safeop_timeout_ms) +
             static_cast<uint64_t>(opt.safeop_to_op_timeout_ms)) *
                1000000ULL);
    const uint64_t startup_skip_domain_queue_ns =
        static_cast<uint64_t>(opt.startup_skip_domain_queue_ms) * 1000000ULL;
    const uint64_t startup_detailed_diag_ns =
        (startup_skip_domain_queue_ns + kStartupDetailedDiagPostResumeNs >
         kStartupDetailedDiagBaseNs)
            ? (startup_skip_domain_queue_ns + kStartupDetailedDiagPostResumeNs)
            : kStartupDetailedDiagBaseNs;

    bool startup_ready = false;
    bool startup_process_data_live_logged = false;
    bool startup_ready_logged = false;
    bool startup_timeout_logged = false;
    bool startup_wait_logged = false;
    bool startup_reset_issued = false;
    uint32_t startup_reset_count = 0;
    uint64_t last_startup_log_ns = 0;
    bool startup_passive_prev = false;
    bool startup_skip_domain_queue_prev = false;
    // Per-axis safety clamps (max rpm -> counts/s) are computed in finalize_axis_config().

#endif
  auto jog_identity(uint64_t connection_id) {
      return rosie::rtcore::JogIdentity{connection_id, control_policy.generation.load(),
          application_grant.current.identity.capability_id,
          application_grant.current.identity.app_grant_generation,
          configuration_epoch.load(), home_epoch.load(), 1};
  }

  // Advance exactly one cycle. Scheduling and sleeping belong to run_cycle;
  // the supplied clock samples at each original observation point.
  // Cycle-path census: a fault-latching cycle attempts one 152-byte SpscQueue
  // push per (reason bit, axis). The 64-slot queue holds four reasons x 16 axes;
  // further triggers drop and increment the loss count. No allocation, lock or
  // syscall is added by fault provenance, including queue saturation.
#if GRADIENT_HAVE_ECRT
  template<class Bus>
  CycleOutputs step(const CycleInputs<Bus>& inputs) {
    auto& bus = inputs.bus;
    auto& cycle_backend = bus.cycle_backend;
    auto& off = bus.off;
    auto& sc = bus.sc;
    auto& brake_signal_offsets = bus.brake_signal_offsets;
    auto& ecrt_ok = bus.ecrt_ok;
    auto& expected_wkc = bus.expected_wkc;
    auto& startup_begin_ns = bus.startup_begin_ns;
    auto& startup_cycle_counter = bus.startup_cycle_counter;
    auto log_phase_summary = [&](const char* phase, uint64_t, uint64_t) {
      diagnostic_phase_requests.fetch_or(std::strcmp(phase, "startup_reset") == 0 ? 1u : 2u,
                                        std::memory_order_relaxed);
    };
#else
  CycleOutputs step(const CycleInputs& inputs) {
#endif
      CycleOutputs outputs{};
      const uint64_t cycle_work_begin_ns = inputs.clock();
      struct CycleOwnerScope {
        MachineState* previous_owner = cycle_owner;
        uint64_t (*previous_clock)() = cycle_clock;
        ~CycleOwnerScope() { cycle_owner = previous_owner; cycle_clock = previous_clock; }
      } owner_scope;
      cycle_owner = this;
      cycle_clock = inputs.clock;
      cell_io.begin_cycle(cycle_work_begin_ns,rt_cycle_counter.load());
      startup_reset_cycle_mask = 0;
      consume_control_requests();
      auto* fast_cycle_region = inputs.fast_region;
      bool jog_evaluated = false;
      // Measure wakeup jitter for the previous absolute sleep target (`next_ns`).
      // This measures how late/early we were relative to the requested schedule.
      const uint64_t wake_target_ns = next_ns;
      const uint64_t wake_ns = inputs.clock();
      if (fast_cycle_region && fast_cycle_region->connection_id() != failed_connection_id) {
        rosie::protocol::GrantRequest wire_request{};
        const auto publication = fast_cycle_region->read_grant_request(wire_request);
        // Sample after publication/heartbeat reads. A newly produced input or
        // heartbeat must not look future merely because it arrived this cycle.
        const auto heartbeat = control_policy.heartbeat_ns.load(std::memory_order_acquire);
        const auto grant_now = inputs.clock();
        const rosie::rtcore::ApplicationGrantContext context{
            fast_cycle_region->connection_id(), control_policy.generation.load(), grant_now,
            opt.control_max_grant_lease_ns, configured_mask,
            control_policy.connected.load() && heartbeat && grant_now >= heartbeat &&
                grant_now - heartbeat < control_policy.lease_ns && !g_stop.load(),
            !armed.load() && !axis_enable_mask.load() && !service_mode_axis_mask.load()};
        auto apply_grant = [&](const rosie::rtcore::ApplicationGrantDecision& decision) {
          if (decision.revoke_outputs) {
            // Stop may already have advanced the native fence. Clear all
            // permission without advancing it a second time in that case.
            inhibit_outputs();
            if (control_policy.connected.load(std::memory_order_acquire))
              (void)control_policy.revoke(application_grant.current.identity.native_generation);
          }
          application_grant = decision.next;
          if (application_grant.active) {
            control_policy.application_deadline_ns.store(application_grant.current.deadline_host_ns,
                                                          std::memory_order_release);
            control_policy.application_generation.store(application_grant.current.identity.native_generation,
                                                         std::memory_order_release);
          } else {
            control_policy.application_generation.store(0, std::memory_order_release);
          }
        };
        if (grant_connection_id != context.connection_id) {
          const auto binding = rosie::rtcore::bind_application_grant_connection(application_grant, context);
          apply_grant(binding);
          if (binding.accepted) {
            grant_connection_id = context.connection_id;
            grant_publication_seen = 0;
            grant_request_ticket = 0;
            grant_request_reason = 0;
          }
        }
        const auto polled = rosie::rtcore::poll_application_grant(application_grant, context);
        apply_grant(polled);
        if (publication.status == rosie::rtcore::SharedWordsStatus::Ok &&
            publication.sequence > grant_publication_seen) {
          grant_publication_seen = publication.sequence;
          if (wire_request.ticket > grant_request_ticket) {
            grant_request_ticket = wire_request.ticket;
            if (wire_request.reserved || polled.revoke_outputs) {
              grant_request_reason = static_cast<uint32_t>(wire_request.reserved
                  ? rosie::rtcore::ApplicationGrantReason::Invalid : polled.reason);
            } else {
              const rosie::rtcore::ApplicationGrantRequest request{
                  static_cast<rosie::rtcore::ApplicationGrantOperation>(wire_request.operation),
                  {wire_request.connection_id, wire_request.native_generation, wire_request.capability_id,
                   wire_request.app_grant_generation},
                  wire_request.origin_host_ns, wire_request.deadline_host_ns, wire_request.axis_mask};
              const auto decision = rosie::rtcore::evaluate_application_grant(application_grant, context, request);
              apply_grant(decision);
              grant_request_reason = static_cast<uint32_t>(decision.reason);
              if (decision.accepted && request.operation == rosie::rtcore::ApplicationGrantOperation::Acquire)
                advance_revoke_floor(request.origin_host_ns);
            }
          }
        }
      }
      const int64_t jitter_ns =
          static_cast<int64_t>(wake_ns) - static_cast<int64_t>(wake_target_ns);
      rt_last_jitter_ns.store(jitter_ns, std::memory_order_relaxed);
      const int64_t abs_jitter_ns = (jitter_ns >= 0) ? jitter_ns : -jitter_ns;
      int64_t prev_max = rt_max_abs_jitter_ns.load(std::memory_order_relaxed);
      while (abs_jitter_ns > prev_max &&
             !rt_max_abs_jitter_ns.compare_exchange_weak(
                 prev_max, abs_jitter_ns, std::memory_order_relaxed)) {
      }
      if (jitter_ns > static_cast<int64_t>(period)) {
        rt_overrun_count.fetch_add(1, std::memory_order_relaxed);
      }

      if (jitter_ns > static_cast<int64_t>(opt.max_cycle_lateness_ns) &&
          (armed.load() || service_mode_axis_mask.load())) {
        execution_fault(rosie::rtcore::FaultCycleDeadline, configured_mask);
      }
      next_ns = rosie::rtcore::future_deadline(wake_target_ns, wake_ns, period);

#if GRADIENT_HAVE_ECRT
      if (ecrt_ok) {
        // --- EtherCAT cyclic loop (1 kHz) ---
        // IgH needs the current application clock, not the next sleep deadline.
        // Keep its phase anchor at cycle entry; reference drift compensation
        // gets a fresh timestamp near submission below.
        const uint64_t app_time_ns = cycle_work_begin_ns;
        const uint64_t diag_now_ns = wake_ns;
        startup_cycle_counter++;
        const uint64_t startup_elapsed_ns_pre =
            startup_begin_ns != 0 && diag_now_ns >= startup_begin_ns ? (diag_now_ns - startup_begin_ns) : 0;
        const bool startup_passive_active =
            opt.startup_passive_ms > 0 &&
            startup_elapsed_ns_pre < (static_cast<uint64_t>(opt.startup_passive_ms) * 1000000ULL);
        const bool startup_skip_domain_queue_active =
            startup_skip_domain_queue_ns > 0 &&
            startup_elapsed_ns_pre < startup_skip_domain_queue_ns;

        if (startup_passive_active != startup_passive_prev) {
          logf("EtherCAT startup passive_outputs=%u cycle=%llu elapsed_ms=%llu",
               startup_passive_active ? 1u : 0u,
               static_cast<unsigned long long>(startup_cycle_counter),
               static_cast<unsigned long long>(startup_elapsed_ns_pre / 1000000ULL));
          startup_passive_prev = startup_passive_active;
        }
        if (startup_skip_domain_queue_active != startup_skip_domain_queue_prev) {
          logf("EtherCAT startup domain_queue_suppressed=%u cycle=%llu elapsed_ms=%llu",
               startup_skip_domain_queue_active ? 1u : 0u,
               static_cast<unsigned long long>(startup_cycle_counter),
               static_cast<unsigned long long>(startup_elapsed_ns_pre / 1000000ULL));
          startup_skip_domain_queue_prev = startup_skip_domain_queue_active;
        }

        // On SIGINT/SIGTERM inhibit all axes immediately. Keep cyclic communication
        // for at least the grace window AND until the mailbox worker exits; an
        // in-flight IgH request cannot finish after we stop advancing the bus.
        if (g_stop.load(std::memory_order_relaxed) && !shutdown_active) {
          shutdown_active = true;
          shutdown_started_ns = diag_now_ns;
          shutdown_until_ns = diag_now_ns + kShutdownGraceNs;
          logf("shutdown drain begin: outputs inhibited, exchanging until worker_exited=1 and mailbox idle "
               "(worker_exited=%u mailbox_in_flight=%u)",
               commissioning_exited.load(std::memory_order_acquire) ? 1u : 0u,
               mailbox_calls_in_flight.load(std::memory_order_acquire));
          armed.store(false, std::memory_order_relaxed);
          axis_enable_mask.store(0, std::memory_order_relaxed);
          service_mode_axis_mask.store(0, std::memory_order_relaxed);
          native_home_active_axis_mask.store(0, std::memory_order_relaxed);
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            desired_mode_of_operation[axis].store(0, std::memory_order_relaxed);
            service_mode_override[axis].store(kNoModeOverride, std::memory_order_relaxed);
            service_controlword_override[axis].store(0, std::memory_order_relaxed);
          }
        }

        latest_feedback.seq.fetch_add(1, std::memory_order_acq_rel);
        const uint64_t coordinate_configuration_epoch = configuration_epoch.load();
        const uint64_t coordinate_verified_epoch = verified_configuration_epoch.load();
        const uint32_t coordinate_verified_mask = control_policy.config_verified_mask.load();
        inputs.observe(CyclePhase::Receive);
        const bool cycle_feedback_fresh = cycle_backend.begin_cycle(app_time_ns);
        const uint32_t cycle_feedback_mask = cycle_backend.feedback_axis_mask();
        const uint64_t feedback_observed_ns = inputs.clock();
        if (opt.cell_io.enabled()) {
          uint16_t io_inputs = 0;
          const bool io_fresh = inputs.bus.cell_io_wire.read(opt.cell_io, cycle_backend.axis(0), cycle_feedback_fresh, io_inputs);
          const bool process_io_active = cell_io.state().armed && !cell_io.manual();
          cell_io.observe(io_inputs, io_fresh, feedback_observed_ns, rt_cycle_counter.load());
          cell_io.expire(feedback_observed_ns);
          if((cell_io.state().reason==rosie::protocol::reason_io_readback_disagreement ||
              (process_io_active && !cell_io.state().armed)) &&
             !(execution_fault_reasons.load() & rosie::rtcore::FaultCellIO))
            execution_fault(rosie::rtcore::FaultCellIO,configured_mask);
        }
        // Capture each fresh transition before consulting any receipt. A clear
        // followed by the same alarm cannot reuse the preceding upload.
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if (cycle_feedback_mask & (1u << axis))
            drive_alarms[axis].observe(off[axis].err != BusHandles::kInvalidOffset
                ? cycle_backend.axis(axis).read_u16(off[axis].err) : 0);
        }
        const auto cycle_drive_error = [&](uint32_t axis) {
          const auto pd = cycle_backend.axis(axis);
          const uint16_t error = off[axis].err != BusHandles::kInvalidOffset ? pd.read_u16(off[axis].err) : 0;
          const uint32_t manufacturer = off[axis].manufacturer_err != BusHandles::kInvalidOffset
              ? pd.read_u32(off[axis].manufacturer_err) : 0;
          return drive_error_faulted(axis, error, manufacturer);
        };
        latest_feedback.pdo_fresh_axis_mask = cycle_feedback_mask;
        latest_feedback.ext_valid_axis_mask.store(0, std::memory_order_release);
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if (cycle_feedback_mask & (1u << axis)) {
            latest_feedback.pdo_observed_time_ns[axis] = feedback_observed_ns;
            if (coordinate_receipt_id[axis] == UINT64_MAX)
              coordinate_receipt_exhausted[axis] = true;
            else if (!coordinate_receipt_exhausted[axis])
              ++coordinate_receipt_id[axis];
          }
        }
        const auto& cycle_bus = cycle_backend.results();
        domain_state = cycle_backend.domain_state();
        record_bus_failure(cycle_bus);
        // Consume the available topology decision before output. A refusal
        // after admission has the same fault contract as lost axis feedback;
        // publisher diagnostics still cannot establish exchange freshness.
        const auto diagnostic_generation = drive_diagnostics_generation.load(std::memory_order_acquire);
        if (diagnostic_generation != diagnostic_generation_seen) {
          DriveDiagnostics sampled_diagnostics{};
          if (drive_diagnostics.read(sampled_diagnostics)) {
            master_diag_state = sampled_diagnostics.master;
            sc_state = sampled_diagnostics.slaves;
            topology_responding_expected = sampled_diagnostics.responding_expected ? sampled_diagnostics.responding_expected : opt.num_axes;
            topology_admitted = sampled_diagnostics.topology_admitted;
            diagnostic_generation_seen = diagnostic_generation;
          }
        }
        if ((!cycle_feedback_fresh || !topology_admitted) && bus_ready.exchange(false)) {
          control_policy.home_valid_mask.store(0);
          control_policy.config_verified_mask.store(0);
          configuration_epoch.fetch_add(1);
          execution_fault(rosie::rtcore::FaultDriveReadiness, configured_mask);
        }

        inputs.observe(CyclePhase::Readiness);
        auto commissioning_context = [&]() {
          rosie::rtcore::CommissioningIdentity deadline_identity{};
          const bool deadline = commissioning_deadline_identity.read(deadline_identity) &&
              rosie::rtcore::same_commissioning_identity(deadline_identity, commissioning_state.identity);
          return rosie::rtcore::CommissioningContext{
              control_policy.generation.load(), configuration_epoch.load(), home_epoch.load(),
              configured_mask, control_policy.config_verified_mask.load(),
              lease_is_valid_now(), bus_ready.load() && cycle_feedback_fresh && !safety_fault_mask.load() &&
                  verified_configuration_epoch.load() == configuration_epoch.load(),
              armed.load(), active_trajectory || active_jog || active_halt, g_stop.load(),
              deadline ? rosie::rtcore::CommissioningReason::CycleDeadline : rosie::rtcore::CommissioningReason::Cancelled};
        };
        uint32_t cycle_motion_ready_mask = 0;
        const auto arm_opening_identity = [&]() {
          return jog_identity(application_grant.current.identity.connection_id);
        };
        const auto arm_opening_context = [&](uint32_t axis) {
          const uint32_t bit = 1u << axis;
          return axis_profiles[axis].brake.opening.enabled() &&
              a6_brake_opening_profile_supported(axis_profiles[axis]) &&
              armed.load() && (axis_enable_mask.load() & bit) &&
              application_grant.active && (application_grant.current.axis_mask & bit) &&
              arm_opening_identity().valid() && lease_is_valid_now() &&
              application_grant.current.identity.native_generation == control_policy.generation.load() &&
              bus_ready.load() && cycle_feedback_fresh && topology_admitted && !g_stop.load() &&
              !safety_fault_mask.load() && !cycle_drive_error(axis) &&
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              (control_policy.config_verified_mask.load() & bit) &&
              (control_policy.home_valid_mask.load() & bit) &&
              (!axis_profiles[axis].coordinate_evidence.enabled || latest_feedback.coordinate_valid[axis]) &&
              commissioning_state.phase == rosie::rtcore::CommissioningPhase::Idle &&
              !service_mode_axis_mask.load() && !native_home_active_axis_mask.load() &&
              desired_mode_of_operation[axis].load() == 8 &&
              off[axis].mode_disp != BusHandles::kInvalidOffset &&
              cycle_backend.axis(axis).read_u8(off[axis].mode_disp) == 8;
        };
        const auto arm_opening_matches = [&](uint32_t axis) {
          const auto& stage = arm_opening_stages[axis];
          return stage.identity == arm_opening_identity() &&
              stage.frame_epoch == absolute_frame_epoch[axis].load() &&
              stage.coordinate_epoch == axis_coordinate_epoch[axis];
        };
        // Operator-facing classification only; cancellation itself is unchanged.
        const auto arm_cancel_cause = [&](uint32_t axis) {
          if (arm_opening_stages[axis].frame_epoch != absolute_frame_epoch[axis].load())
            return rosie::rtcore::ArmCancelCause::FrameChanged;
          if (desired_mode_of_operation[axis].load() != 8 || off[axis].mode_disp == BusHandles::kInvalidOffset ||
              cycle_backend.axis(axis).read_u8(off[axis].mode_disp) != 8)
            return rosie::rtcore::ArmCancelCause::ModeChanged;
          return rosie::rtcore::ArmCancelCause::ContextLost;
        };
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          const auto pd = cycle_backend.axis(axis);
          const bool fresh = (cycle_feedback_mask & (1u << axis)) != 0;
          const auto drive = gradient::ds402::decode_statusword(pd.read_u16(off[axis].sw));
          const auto& profile = axis_profiles[axis];
          std::optional<bool> released;
          if (fresh && brake_signal_offsets[axis] && *brake_signal_offsets[axis] != BusHandles::kInvalidOffset) {
            const auto offset = *brake_signal_offsets[axis];
            const uint32_t value = profile.brake_released_width == 8 ? pd.read_u8(offset) :
                profile.brake_released_width == 16 ? pd.read_u16(offset) : pd.read_u32(offset);
            released = (value & (uint32_t{1} << profile.brake_released_bit)) != 0;
          }
          brake_outputs[axis] = brakes[axis].step(fresh ? drive : gradient::ds402::State::Unknown,
                                                wake_ns, profile.brake, released);
          if (!fresh && profile.brake.present) {
            brake_outputs[axis].state = gradient::ipc::v1::BRAKE_UNKNOWN;
            brake_outputs[axis].motion_permitted = false;
          }
          rosie::rtcore::BrakeTransitionInput brake_input{};
          brake_input.observed_ns = feedback_observed_ns;
          brake_input.now_ns = inputs.clock();
          brake_input.maximum_age_ns = period;
          brake_input.maximum_gap_ns = 2 * period + opt.max_cycle_lateness_ns;
          brake_input.command = last_brake_hold_command[axis];
          brake_input.maximum_wire_age_ns = 2 * period;
          auto& arm_stage = arm_opening_stages[axis];
          if (!armed.load() || !(axis_enable_mask.load() & (1u << axis))) arm_stage = {};
          else if (arm_stage.token && !arm_stage.cancelled &&
                   (!arm_opening_context(axis) || !arm_opening_matches(axis))) {
            arm_stage.cancelled = true;
            arm_stage.cancel_cause = arm_cancel_cause(axis);
          }
          brake_input.velocity_valid = fresh && off[axis].velocity_actual != BusHandles::kInvalidOffset;
          const bool normal_arm_opening = arm_stage.token && !arm_stage.completed && !arm_stage.cancelled &&
              arm_opening_context(axis) && arm_opening_matches(axis) &&
              !active_trajectory && !active_jog && !active_halt;
          if (normal_arm_opening) {
            brake_input.opening_authorized = brake_input.command.opening_stage &&
                brake_input.command.generation == arm_stage.token;
            brake_input.opening_until_released = true;
          }
          brake_input.velocity = brake_input.velocity_valid ? pd.read_s32(off[axis].velocity_actual) : 0;
          brake_input.enable_requested = armed.load() && (axis_enable_mask.load() & (1u << axis));
          const bool home_service = profile.brake.transition.enabled()
              ? ((native_home_active_axis_mask.load() & (1u << axis)) &&
                 commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active &&
                 rosie::rtcore::commissioning_permission(commissioning_state.identity, commissioning_context()) &&
                 ((service_mode_axis_mask.load() & (1u << axis)) ||
                  absolute_frame_epoch[axis].load() != brake_guards[axis].anchor_epoch))
              : (service_mode_axis_mask.load() & (1u << axis)) != 0;
          // A brake-hold detection is a warning only: no fault, revoke, Home
          // invalidation or Arm refusal. The guard re-anchors on its own.
          const auto position = pd.read_s32(off[axis].pos);
          auto& guard = brake_guards[axis];
          auto& evidence = brake_warning_evidence[axis];
          const bool warned = guard.observe(profile.brake, brake_outputs[axis], drive, fresh, position,
              absolute_frame_epoch[axis].load(), home_service, brake_input);
          if (warned) {
            const auto& w = guard.warning;
            evidence = {feedback_observed_ns,
                static_cast<int64_t>(rosie::rtcore::command_delta(position, w.anchor)),
                w.anchor, position, pd.read_u16(off[axis].sw), brake_outputs[axis].state,
                brake_outputs[axis].hold_window_active, released.has_value(), released.value_or(false),
                w.cause, w.phase, brake_input.velocity, brake_input.velocity_valid,
                w.caps_valid, w.velocity_cap, w.displacement_cap, w.sequence};
          }
          if (warned || guard.warning_updated || guard.energized_updated) {
            const auto& w = guard.warning;
            evidence.peak_displacement_counts = w.peak_displacement;
            evidence.settled_displacement_counts = w.settled_displacement;
            evidence.peak_velocity_counts_per_s = w.peak_velocity;
            evidence.torque_on = w.torque_on;
            evidence.settled = w.settled;
            evidence.slip = w.slip;
            evidence.same_direction_count = w.same_direction_count;
            evidence.same_direction_displacement_counts = w.same_direction_displacement;
            // Recorded for diagnosis only: energized movement is never judged.
            const auto& e = guard.energized;
            evidence.torque_off_ns = e.torque_off_ns;
            evidence.energized_peak_displacement_counts = e.peak_displacement;
            evidence.energized_peak_velocity_counts_per_s = e.peak_velocity;
            evidence.torque_off_anchor_counts = e.torque_off_anchor;
            brake_hold_evidence[axis].publish(evidence);
          }
          // HM35 keeps origin/main's start-bit gating: only the brake's own
          // release delay, never the transition guard's opening qualification.
          release_delay_motion_permitted[axis] = brake_outputs[axis].motion_permitted;
          guard.qualify(profile.brake, brake_outputs[axis]);
          if (normal_arm_opening && brake_guards[axis].transition.phase == rosie::rtcore::BrakeGuardPhase::Enabled &&
              brake_outputs[axis].motion_permitted)
            arm_stage.completed = true;
          if (arm_stage.token && !arm_stage.completed && !arm_stage.cancelled &&
              (brake_guards[axis].transition.phase == rosie::rtcore::BrakeGuardPhase::Closing ||
               brake_guards[axis].transition.phase == rosie::rtcore::BrakeGuardPhase::Settling)) {
            arm_stage.cancelled = true;
            arm_stage.cancel_cause = rosie::rtcore::ArmCancelCause::OpeningInterrupted;
          }
          const int8_t mode = off[axis].mode_disp != BusHandles::kInvalidOffset
              ? static_cast<int8_t>(pd.read_u8(off[axis].mode_disp)) : 0;
          if (rosie::rtcore::motion_axis_ready((cycle_feedback_mask & (1u << axis)) != 0,
                  pd.read_u16(off[axis].sw), mode, cycle_drive_error(axis)) && brake_outputs[axis].motion_permitted)
            cycle_motion_ready_mask |= 1u << axis;
        }
        outputs.motion_ready_mask = cycle_motion_ready_mask;
        complete_halt_requests(cycle_motion_ready_mask);
        const uint32_t executing_mask = active_trajectory && active_traj_point_count
            ? active_traj_points[0].axis_mask : (active_jog ? active_jog_axis_mask : halt_axis_mask);
        // A voluntary Stop has already revoked authority. A drive losing
        // readiness while motion still owns authority is an execution fault,
        // detected before this cycle advances the trajectory or jog target.
        if (executing_mask && armed.load() &&
            !rosie::rtcore::motion_group_ready(executing_mask, configured_mask,
                                              cycle_motion_ready_mask))
          execution_fault(rosie::rtcore::FaultDriveReadiness, executing_mask & ~cycle_motion_ready_mask);

        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> target_counts{};
        std::array<double, gradient::ipc::v1::GRADIENT_MAX_AXES> target_velocity_counts_per_s{};
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_counts{};
        std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_brake_opening_token{};
        std::array<int32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> output_target_velocity_counts_per_s{};
        uint32_t sp_mask = 0;
        uint32_t snap_jog_hold_to_feedback_mask = 0;
        const uint64_t abort_req =
            trajectory_abort_request.exchange(0, std::memory_order_acq_rel);
        if (active_trajectory &&
            (abort_req == std::numeric_limits<uint64_t>::max() ||
             trajectory_store.abort_requested(active_plan_slot))) {
          active_trajectory = false;

          active_traj_point_count = 0;

          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
          motion_exec_state.store(execution_fault_reasons.load()
              ? gradient::ipc::v1::EXEC_STATE_FAULTED : gradient::ipc::v1::EXEC_STATE_ABORTED,
              std::memory_order_relaxed);
          motion_active_traj_id.store(active_traj_id_rt, std::memory_order_relaxed);
          motion_current_point_index.store(std::numeric_limits<uint32_t>::max(), std::memory_order_relaxed);
          motion_queue_depth.store(0, std::memory_order_relaxed);
          motion_last_event_code.store(execution_fault_reasons.load()
              ? gradient::ipc::v1::EVT_TRAJECTORY_FAULTED : gradient::ipc::v1::EVT_TRAJECTORY_ABORTED,
              std::memory_order_relaxed);
          motion_stale_command_flag.store(0, std::memory_order_relaxed);
          motion_done.store(1, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
        }

        const bool lease_valid_now = lease_is_valid_now();
        if (previous_lease_valid && !lease_valid_now) {
          inhibit_outputs();
          // Terminal disconnects have one dedicated fencing owner. The legacy
          // heartbeat transition only fences while that connection stays open.
          if (control_policy.connected.load(std::memory_order_acquire))
            (void)control_policy.revoke(observed_generation);
        }
        previous_lease_valid = lease_valid_now;
        const auto generation_now = control_policy.generation.load(std::memory_order_acquire);
        if (observed_generation != generation_now) {
          observed_generation = generation_now;
          active_trajectory = false; active_jog = false;
          active_halt = false; halt_axis_mask = 0;
           active_jog_axis_mask = 0;
          active_jog_deadline_ns = 0; sp_mask = 0;
          have_jog_target.fill(false); have_hold.fill(false); enable_hold_latched.fill(false);
          for (auto& axis : jog_axes) axis.reset(0);
          motion_exec_state.store(execution_fault_reasons.load()
              ? gradient::ipc::v1::EXEC_STATE_FAULTED : gradient::ipc::v1::EXEC_STATE_ABORTED);
          motion_queue_depth.store(0); motion_done.store(1);
          motion_stale_command_flag.store(lease_valid_now ? 0 : 1);
        }
        if (!active_trajectory) start_alignment.cancel();
        if (!active_trajectory && active_plan_slot >= 0) {
          trajectory_store.retire(active_plan_slot);
          active_plan_slot = -1; active_traj_points = nullptr; active_traj_point_count = 0;
        }
        auto coordinate_policy_for = [&](uint32_t axis) {
          const auto& config = axis_profiles[axis].coordinate_evidence;
          uint64_t tracking_speed = 0;
          if (axis_profiles[axis].independent_absolute_origin()) {
            const auto model = coordinate_model(axis, active_reference_offset(axis));
            tracking_speed = rosie::rtcore::coordinate_tracking_speed(
                config.maximum_physical_speed_counts_per_second,
                std::abs(model.logical_delta_to_counts(axis_profiles[axis].limits.max_qd)),
                double(opt.axis[axis].max_profile_vel_counts_per_s));
          }
          return rosie::rtcore::CoordinatePolicy{
              config.maximum_physical_speed_counts_per_second, config.maximum_acquisition_age_ns,
              config.maximum_feedback_error_counts, config.maximum_anchor_error_counts, config.enabled, tracking_speed};
        };
        auto coordinate_identity_for = [&](uint32_t axis) {
          // This is a qualified host source binding, not a drive-supplied
          // encoder generation. Source validity is checked independently below.
          const uint64_t source_session = coordinate_configuration_epoch;
          return rosie::rtcore::CoordinateIdentity{source_session,
              axis_coordinate_epoch[axis], source_session, axis_profiles[axis].coordinate_digest};
        };
        std::array<rosie::rtcore::SampledAbsolutePosition, gradient::ipc::v1::GRADIENT_MAX_AXES> cycle_absolute{};
        std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> cycle_absolute_valid{};
        // A6-EC fault table (GradientOS docs/resources/a6ec_manual_codes.md):
        // 0x7305 is encoder error. In both sampled modes ordinary statusword
        // faults and nonzero 0x603F inhibit readiness without clearing Home;
        // this encoder-specific code also invalidates coordinate evidence.
        auto encoder_faulted_for = [&](uint32_t axis) {
          return off[axis].err != BusHandles::kInvalidOffset &&
              cycle_backend.axis(axis).read_u16(off[axis].err) == 0x7305 && cycle_drive_error(axis);
        };
        // Per sampled axis: one PDO-history append, at most two sample lookups,
        // at most eight
        // snapshot attempts, bounded int128 arithmetic and one publication.
        // No allocations, mailbox calls, syscalls or age-sized scans in cycle.
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if (!axis_profiles[axis].coordinate_evidence.sdo_sampled) continue;
          auto& absolute = cycle_absolute[axis];
          const auto pd = cycle_backend.axis(axis);
          rosie::rtcore::SampledAbsolutePosition newest{};
          // A publication after this PDO receipt belongs to a later cycle.
          // Retain the previous complete acquisition without renewing its age.
          // A contended snapshot also leaves only the age-bounded prior sample.
          if (!sampled_absolute[axis].read(newest)) newest = {};
          auto& history = sampled_bindings[axis];
          history.observe(coordinate_configuration_epoch, absolute_frame_epoch[axis].load(),
              rt_cycle_counter.load() + 1, feedback_observed_ns, pd.read_s32(off[axis].pos),
              (cycle_feedback_mask & (1u << axis)) != 0, coordinate_policy_for(axis));
          int64_t stationary_low = 0, stationary_high = 0;
          const bool stationary = history.latest.cycle > 1 &&
              history.query_history_interval(history.latest.cycle - 1, history.latest.cycle, stationary_low, stationary_high) &&
              __int128(stationary_high) - stationary_low <= rosie::rtcore::SampledPdoHistory::maximum_home_acquisition_displacement_counts;
          if (stationary) absolute_stationary_mask.fetch_or(1u << axis);
          else absolute_stationary_mask.fetch_and(~(1u << axis));
          const bool faulted = (pd.read_u16(off[axis].sw) & 8u) != 0 ||
              cycle_drive_error(axis);
          cycle_absolute_valid[axis] = axis_profiles[axis].independent_absolute_origin()
              ? history.track_independent(newest, sampled_at_receipt[axis], coordinate_policy_for(axis),
                  feedback_observed_ns, encoder_faulted_for(axis), !armed.load() && !axis_enable_mask.load(), audit_generation.load())
              : history.consume(newest, sampled_at_receipt[axis], coordinate_policy_for(axis), feedback_observed_ns, faulted);
          absolute = sampled_at_receipt[axis];
          if (axis_profiles[axis].independent_absolute_origin() && cycle_absolute_valid[axis]) {
            // Verified startup turn plus current PDO displacement; mailbox
            // metrics retain the actual acquisition age independently.
            absolute = history.startup_sample;
            absolute.encoder_counts = history.command_counts - history.command_offset;
          }
          sampled_consumption[axis].publish(history.metrics);
        }
        auto coordinate_sample_for = [&](uint32_t axis) {
          const auto& config = axis_profiles[axis].coordinate_evidence;
          const auto pd = cycle_backend.axis(axis);
          const bool current = (cycle_feedback_mask & (1u << axis)) != 0 &&
              !coordinate_receipt_exhausted[axis];
          const bool configuration_qualified =
              ((coordinate_verified_mask & (1u << axis)) &&
               coordinate_verified_epoch == coordinate_configuration_epoch) ||
              ((reverification_coordinate_mask & (1u << axis)) && coordinate_reference[axis].valid &&
               coordinate_reference[axis].identity.configuration_epoch == coordinate_configuration_epoch);
          const bool source_valid = current && config.enabled &&
              (config.sdo_sampled ? cycle_absolute_valid[axis] :
              (off[axis].continuous_valid != BusHandles::kInvalidOffset &&
              (pd.read_u16(off[axis].continuous_valid) & config.validity_mask) == config.validity_value)) &&
              (config.sdo_sampled ? !encoder_faulted_for(axis) : !(pd.read_u16(off[axis].sw) & 8u)) &&
              configuration_qualified &&
              configuration_epoch.load() == coordinate_configuration_epoch;
          rosie::rtcore::CoordinateSample sample{};
          sample.raw_counts = pd.read_s32(off[axis].pos);
          sample.evidence = {coordinate_receipt_id[axis], feedback_observed_ns, 0,
              config.sdo_sampled ? opt.cycle_ns : config.maximum_acquisition_age_ns,
              current, config.enabled, source_valid};
          return sample;
        };
        auto coordinate_anchor_for = [&](uint32_t axis,
                                        const rosie::rtcore::CoordinateSample& sample) {
          rosie::rtcore::CoordinateAnchor anchor{};
          anchor.identity = coordinate_identity_for(axis);
          anchor.serial = sample.evidence.receipt_id;
          anchor.source = rosie::rtcore::CoordinateAnchorSource::AbsolutePosition;
          if (off[axis].continuous_pos != BusHandles::kInvalidOffset)
            anchor.continuous_counts = cycle_backend.axis(axis).read_s32(off[axis].continuous_pos);
          anchor.evidence = sample.evidence;
          anchor.verified = sample.evidence.source_valid && off[axis].continuous_pos != BusHandles::kInvalidOffset;
          if (axis_profiles[axis].coordinate_evidence.sdo_sampled) {
            const auto& absolute = cycle_absolute[axis];
            anchor.serial = absolute.started_ns;
            anchor.continuous_counts = sampled_bindings[axis].command_counts;
            anchor.verified = sample.evidence.source_valid && cycle_absolute_valid[axis];
            // The history carries the encoder's turn forward to this PDO
            // receipt. Its acquisition age is the PDO cycle, not mailbox age.
          }
          return anchor;
        };
        auto reference_scope_now = [&](uint32_t mask) {
          return application_grant.active && (application_grant.current.axis_mask & mask) == mask &&
              application_grant.current.identity.native_generation == control_policy.generation.load() &&
              lease_is_valid_now();
        };
        auto reference_inhibited_now = [&]() {
          if (armed.load() || axis_enable_mask.load() || service_mode_axis_mask.load() ||
              active_trajectory || active_jog || jog_lane.state().open || safety_fault_mask.load() ||
              execution_fault_reasons.load()) return false;
          for (uint32_t a = 0; a < opt.num_axes; ++a) {
            const auto& wire = last_brake_hold_command[a];
            if (!(cycle_feedback_mask & (1u << a)) || cycle_drive_error(a) ||
                !rosie::rtcore::position_output_inactive(gradient::ds402::decode_statusword(
                    cycle_backend.axis(a).read_u16(off[a].sw))) ||
                !wire.valid || wire.controlword != 0 || !wire.submitted_ns ||
                wire.submitted_ns >= feedback_observed_ns || feedback_observed_ns - wire.submitted_ns > 2000000ULL)
              return false;
            if (axis_profiles[a].brake.present &&
                (brake_outputs[a].state != gradient::ipc::v1::BRAKE_APPLIED || brake_outputs[a].hold_window_active))
              return false;
          }
          return true;
        };
        uint32_t home_coordinate_committed_mask = 0;
        auto apply_commissioning = [&](const rosie::rtcore::CommissioningDecision& decision,
                                       const rosie::rtcore::CommissioningRequest* request) {
          const bool cancelled = commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active &&
              decision.state.phase == rosie::rtcore::CommissioningPhase::Draining;
          // Each qualified CompleteHome commits that axis independently. A
          // later axis failure or job cancellation does not undo its evidence;
          // Finish still reports success only when the entire group completed.
          const uint32_t completed_home_mask = decision.accepted
              ? (decision.complete_home_mask | decision.complete_reference_mask) : 0;
          commissioning_state = decision.state;
          if (decision.invalidate_home_mask) {
            control_policy.home_valid_mask.fetch_and(~decision.invalidate_home_mask);
            calibration_valid_mask.fetch_and(~decision.invalidate_home_mask);
            audit_generation.fetch_add(1);
            home_epoch.store(commissioning_state.identity.home_epoch);
          }
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            const uint32_t bit = 1u << axis;
            if (decision.invalidate_home_mask & bit) {
              // A Home changes only its selected coordinate systems. The
              // global home epoch continues to identify the complete job.
              if (axis_coordinate_epoch[axis] && axis_coordinate_epoch[axis] != UINT64_MAX)
                ++axis_coordinate_epoch[axis];
              else
                axis_coordinate_epoch[axis] = 0; // Exhaustion cannot wrap trust.
              coordinate_reference[axis] = rosie::rtcore::invalidate_coordinate(
                  coordinate_reference[axis], rosie::rtcore::CoordinateReason::IdentityChanged).next;
              anchor_delta_restores[axis].publish({});
              if (!decision.state.reference_acquisition) {
                latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_IDLE;
                latest_feedback.native_home_last_abort_code[axis] = 0;
              }
            }
            if (cancelled && (commissioning_state.axis_mask & bit) &&
                latest_feedback.native_home_state[axis] == gradient::ipc::v1::NATIVE_HOME_STATE_REQUESTED)
              latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
            if (decision.clear_service_mask & bit) {
              service_controlword_override[axis].store(0);
              service_mode_override[axis].store(kNoModeOverride);
              if (request && decision.accepted &&
                  request->operation == rosie::rtcore::CommissioningOperation::Release)
                desired_mode_of_operation[axis].store(axis_profiles[axis].home.steady_state_mode);
            }
            if (decision.set_service_mask & bit) {
              service_mode_override[axis].store(request->mode);
              service_controlword_override[axis].store(request->controlword);
              latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_REQUESTED;
            }
            if (decision.complete_reference_mask & bit) latest_feedback.native_home_position_offset[axis] = request->home_offset;
            if (decision.complete_home_mask & bit) {
              latest_feedback.native_home_position_offset[axis] = request->home_offset;
              latest_feedback.native_home_last_abort_code[axis] = 0;
              latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_SUCCEEDED;
            }
            if (decision.fail_home_mask & bit) {
              latest_feedback.native_home_last_abort_code[axis] = request->abort_code;
              latest_feedback.native_home_state[axis] = gradient::ipc::v1::NATIVE_HOME_STATE_FAILED;
            }
          }
          if (completed_home_mask) {
            control_policy.home_valid_mask.fetch_or(completed_home_mask);
            calibration_valid_mask.fetch_or(completed_home_mask);
          }
          service_mode_axis_mask.store(commissioning_state.service_mask);
          native_home_active_axis_mask.store(
              commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active && !commissioning_state.reference_acquisition
                  ? commissioning_state.axis_mask : 0);
          commissioning_phase.store(static_cast<uint32_t>(commissioning_state.phase));
        };
        auto current_commissioning_context = commissioning_context();
        if (commissioning_state.reference_acquisition && (!reference_inhibited_now() || !reference_scope_now(commissioning_state.axis_mask)))
          current_commissioning_context.authority_valid = false;
        apply_commissioning(rosie::rtcore::poll_commissioning(
            commissioning_state, current_commissioning_context), nullptr);
        // Bounded request consumption. The mailbox worker waits outside RT for
        // a correlated receipt before using any new mode/controlword permission.
        for (unsigned consumed = 0; consumed < 8; ++consumed) {
          CommissioningEnvelope envelope{};
          if (!commissioning_requests.pop(envelope)) break;
          const auto& request = envelope.request;
          if (request.operation == rosie::rtcore::CommissioningOperation::Reserve &&
              jog_lane.state().open && !envelope.restoring) {
            if (jog_lane.close_for_home()) inhibit_outputs();
            jog_session_open.store(false, std::memory_order_release);
            active_jog = false;
            active_jog_axis_mask = 0;
            active_jog_deadline_ns = 0;
            have_jog_target.fill(false);
          }
          auto decision = rosie::rtcore::evaluate_commissioning(
              commissioning_state, commissioning_context(), request);
          if (decision.accepted && request.operation == rosie::rtcore::CommissioningOperation::ReserveReference) {
            uint32_t axis = 0; while (axis < opt.num_axes && request.axis_mask != (1u << axis)) ++axis;
            // The only reference reservation is HM35 v3 verified-delta
            // restoration, whose anchor version CompleteReference proves.
            if (axis >= opt.num_axes || !envelope.restoring || !envelope.verified_delta_restore ||
                !axis_profiles[axis].reference_verified_delta_enabled || !reference_inhibited_now() || !reference_scope_now(request.axis_mask)) {
              decision = {commissioning_state}; decision.reason = rosie::rtcore::CommissioningReason::NotReady;
            } else {
              // Explicitly discard only this selected origin; fresh post-reservation
              // history and acquisition must establish the replacement.
              sampled_bindings[axis].bound = false; sampled_bindings[axis].failed = false;
              sampled_bindings[axis].metrics.startup_verified = 0;
            }
          }
          if (decision.accepted && request.operation == rosie::rtcore::CommissioningOperation::CompleteReference) {
            uint32_t axis = 0; while (axis < opt.num_axes && request.axis_mask != (1u << axis)) ++axis;
            bool qualified = axis < opt.num_axes && reference_inhibited_now() && reference_scope_now(request.axis_mask);
            rosie::rtcore::DeltaAcquisition reference{};
            if (qualified) {
              auto e = envelope.reference_evidence;
              auto& history = sampled_bindings[axis];
              const auto& policy = coordinate_policy_for(axis);
              e.identity = request.identity; e.authority = commissioning_context(); e.axis_mask = request.axis_mask;
              // The fresh acquisition only witnesses an HM35 v3 delta; it never becomes a zero.
              e.opted_in = envelope.restoring && envelope.verified_delta_restore &&
                  axis_profiles[axis].reference_verified_delta_enabled;
              e.independent_source = axis_profiles[axis].independent_absolute_origin();
              e.all_outputs_inhibited = e.settled_brakes = true;
              e.faulted = cycle_drive_error(axis); e.now_ns = feedback_observed_ns;
              e.axis_coordinate_epoch = axis_coordinate_epoch[axis];
              e.drive_frame_epoch = absolute_frame_epoch[axis].load();
              e.receipt_serial = coordinate_receipt_id[axis]; e.pdo_receipt = coordinate_receipt_id[axis];
              e.pdo_observed_ns = feedback_observed_ns; e.pdo_fresh = (cycle_feedback_mask & request.axis_mask) != 0;
              e.maximum_age_ns = policy.maximum_acquisition_age_ns; e.maximum_pdo_age_ns = period;
              e.tolerance_counts = policy.maximum_anchor_error_counts;
              e.current_command = history.latest.counts;
              e.interval_valid = !history.entries.empty() && e.sample.cycle &&
                  history.entries[e.sample.cycle % history.entries.size()].cycle == e.sample.cycle &&
                  history.query_history_interval(e.sample.cycle, history.latest.cycle, e.interval_min, e.interval_max);
              if (e.interval_valid) e.command_at_sample = history.entries[e.sample.cycle % history.entries.size()].counts;
              qualified = rosie::rtcore::qualify_delta_acquisition(e, reference) == rosie::rtcore::DeltaRestoreReason::None;
              int64_t restore_command_delta = 0, restore_encoder_delta = 0, aligned_command = 0;
              int32_t saved_drive_offset_raw = 0;
              qualified = qualified && envelope.restoring && envelope.verified_delta_restore &&
                  envelope.anchor.version == 3 && envelope.anchor.home_epoch;
              if (qualified)
                qualified = rosie::rtcore::qualify_verified_delta_anchor_v3(envelope.anchor.counts,
                    envelope.anchor.turns, envelope.anchor.offset, axis_profiles[axis].home_truth_sign, reference,
                    e.tolerance_counts,
                    restore_command_delta, restore_encoder_delta, aligned_command, saved_drive_offset_raw) ==
                    rosie::rtcore::DeltaRestoreReason::None &&
                    request.home_offset == envelope.anchor.offset;
              if (qualified) {
                // No delta cap: the restored position, in the saved frame, must lie
                // within the joint's hard limits. The alignment shift is whole turns.
                const int64_t current = e.current_command + (aligned_command - e.command_at_sample);
                const auto position = coordinate_model(axis, envelope.anchor.offset).command_to_logical(current);
                qualified = current >= INT32_MIN && current <= INT32_MAX &&
                    position.valid && axis_profiles[axis].limits.contains(position.logical);
              }
              if (qualified) qualified = history.bind_independent_origin(e.sample, sampled_at_receipt[axis],
                  policy, feedback_observed_ns, aligned_command, false);
              if (qualified) {
                cycle_absolute_valid[axis] = true; cycle_absolute[axis] = sampled_at_receipt[axis];
                const auto sample = coordinate_sample_for(axis);
                const auto candidate = rosie::rtcore::reconcile_coordinate(coordinate_reference[axis],
                    coordinate_identity_for(axis), policy, sample, coordinate_anchor_for(axis, sample));
                int32_t command = 0;
                qualified = candidate.accepted && rosie::rtcore::coordinate_command_counts(candidate.next, command) &&
                    axis_profiles[axis].limits.contains(0.0);
                if (qualified) {
                  coordinate_reference[axis] = candidate.next; home_coordinate_committed_mask |= request.axis_mask;
                  // Keep calibrated v3 offset; C0 is only a historical witness.
                  anchor_delta_restores[axis].publish({true, 3, envelope.anchor.counts, envelope.anchor.turns,
                      envelope.anchor.offset, saved_drive_offset_raw, envelope.anchor.home_epoch,
                      request.identity.job, request.identity.configuration_epoch, request.identity.home_epoch,
                      axis_coordinate_epoch[axis], e.drive_frame_epoch, e.offset_after_completed_ns, e.pdo_receipt,
                      restore_command_delta, restore_encoder_delta});
                  restored_anchor_home_epoch[axis].store(envelope.anchor.home_epoch);
                }
              }
              if (!qualified) { history.bound = false; cycle_absolute_valid[axis] = false; }
            }
            if (!qualified) { decision = {commissioning_state}; decision.reason = rosie::rtcore::CommissioningReason::NotReady; }
          }
          if (envelope.restoring && request.operation == rosie::rtcore::CommissioningOperation::Reserve &&
              (axis_enable_mask.load() || jog_lane.state().open)) {
            decision = {commissioning_state};
            decision.reason = rosie::rtcore::CommissioningReason::NotReady;
          }
          uint32_t new_independent_bindings = 0;
          HomeCompletionDiagnostic home_diagnostic{};
          if (decision.complete_home_mask && envelope.restoring) {
            for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
              if (!(decision.complete_home_mask & (1u << axis))) continue;
              const auto& saved = envelope.anchor;
              auto& history = sampled_bindings[axis];
              // Restoring a versioned independent origin must be possible before
              // coordinate validity exists. The persisted raw count is compared
              // without modulo, and the disabled stationary bind remains explicit.
              if (axis_profiles[axis].independent_absolute_origin() && !history.bound && !axis_enable_mask.load()) {
                const auto before = coordinate_sample_for(axis);
                const auto& proof = envelope.home_sample;
                if (before.evidence.current_receipt && rosie::rtcore::sampled_anchor_matches(proof,
                    before.raw_counts, saved.counts, saved.turns, saved.tolerance_counts)) {
                  const int64_t command = int64_t(saved.counts) + proof.encoder_counts - saved.turns;
                  cycle_absolute_valid[axis] = history.bind_independent_origin(proof, sampled_at_receipt[axis],
                      coordinate_policy_for(axis), feedback_observed_ns, command,
                      cycle_drive_error(axis) || (cycle_backend.axis(axis).read_u16(off[axis].sw) & 8u));
                  if (cycle_absolute_valid[axis]) new_independent_bindings |= 1u << axis;
                  cycle_absolute[axis] = sampled_at_receipt[axis];
                  sampled_consumption[axis].publish(history.metrics);
                }
              }
              const auto sample = coordinate_sample_for(axis);
              const auto absolute = coordinate_anchor_for(axis, sample);
              uint32_t reason = 0;
              if (!absolute.verified || !sample.evidence.current_receipt || axis_enable_mask.load()) reason = 3;
              else if (axis_profiles[axis].coordinate_evidence.sdo_sampled) {
                const auto& proof = cycle_absolute[axis];
                if (!rosie::rtcore::sampled_anchor_matches(proof, sample.raw_counts,
                    saved.counts, saved.turns, saved.tolerance_counts)) reason = 4;
                else {
                  auto& history = sampled_bindings[axis];
                  const int64_t delta = cycle_absolute[axis].encoder_counts - saved.turns;
                  const int64_t rebound = int64_t(saved.counts) + delta;
                  history.command_offset += rebound - history.command_counts;
                  history.command_counts = rebound;
                }
              } else if (saved.counts != saved.turns ||
                  std::abs(absolute.continuous_counts - int64_t(saved.turns)) > int64_t(saved.tolerance_counts)) reason = 4;
              anchor_reason[axis].store(reason);
              if (reason) {
                decision = {commissioning_state};
                decision.reason = rosie::rtcore::CommissioningReason::NotReady;
                break;
              }
            }
          }
          // Home's offset alone cannot establish a continuous turn. A qualified
          // wrapped axis must have a current independent absolute coordinate
          // before CompleteHome can publish success or home validity.
          if (decision.accepted && decision.complete_home_mask) {
            for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
              const uint32_t bit = 1u << axis;
              if (!(decision.complete_home_mask & bit) || !axis_profiles[axis].coordinate_evidence.enabled) continue;
              if (!envelope.restoring && axis_profiles[axis].independent_absolute_origin()) {
                auto& history = sampled_bindings[axis];
                // Undo home_truth_sign in the drive count frame. direction.sign
                // applies only when converting those counts into logical units.
                const int64_t command_home = int64_t(request.home_offset) * axis_profiles[axis].home_truth_sign;
                const bool faulted = cycle_drive_error(axis) || (cycle_backend.axis(axis).read_u16(off[axis].sw) & 8u);
                cycle_absolute_valid[axis] = history.complete_independent_home(envelope.home_sample,
                    sampled_at_receipt[axis], coordinate_policy_for(axis), feedback_observed_ns,
                    command_home, faulted);
                if (cycle_absolute_valid[axis] && axis_profiles[axis].independent_absolute_origin())
                  new_independent_bindings |= bit;
                cycle_absolute[axis] = sampled_at_receipt[axis];
                sampled_consumption[axis].publish(history.metrics);
              }
              const auto sample = coordinate_sample_for(axis);
              const auto candidate = rosie::rtcore::reconcile_coordinate(coordinate_reference[axis],
                  coordinate_identity_for(axis), coordinate_policy_for(axis), sample,
                  coordinate_anchor_for(axis, sample));
              int32_t counts = 0;
              const bool representable = rosie::rtcore::coordinate_command_counts(candidate.next, counts);
              const auto logical = coordinate_model(axis, request.home_offset).command_to_logical(counts);
              const bool configuration_current = configuration_epoch.load() == coordinate_configuration_epoch;
              const bool qualified = candidate.accepted && configuration_current &&
                  representable && logical.valid && axis_profiles[axis].limits.valid;
              if (!qualified) {
                const auto& history = sampled_bindings[axis];
                home_diagnostic.axis = axis;
                home_diagnostic.qualification_mask = (candidate.accepted ? 1u : 0u) |
                    (configuration_current ? 2u : 0u) | (representable ? 4u : 0u) |
                    (logical.valid ? 8u : 0u) | (axis_profiles[axis].limits.valid ? 16u : 0u);
                home_diagnostic.coordinate_reason = uint32_t(candidate.reason);
                if (!envelope.restoring && axis_profiles[axis].independent_absolute_origin()) {
                  home_diagnostic.bind_refusal = history.last_bind_refusal;
                  home_diagnostic.binding_sample_reason = history.last_bind_sample_reason;
                  home_diagnostic.interval_valid = history.last_bind_interval_valid;
                  home_diagnostic.interval_min = history.last_bind_interval_min;
                  home_diagnostic.interval_max = history.last_bind_interval_max;
                  home_diagnostic.anchor_counts = history.last_bind_anchor_counts;
                  home_diagnostic.displacement_counts = history.last_bind_displacement_counts;
                  home_diagnostic.displacement_limit = history.last_bind_displacement_limit;
                }
                home_diagnostic.receipt_ns = feedback_observed_ns;
                home_diagnostic.sample_started_ns = envelope.home_sample.started_ns;
                home_diagnostic.sample_completed_ns = envelope.home_sample.completed_ns;
                home_diagnostic.acquisition_valid = envelope.home_sample.valid;
                home_diagnostic.acquisition_reason = envelope.home_sample.reason;
                home_diagnostic.sample_frame = envelope.home_sample.home_epoch;
                home_diagnostic.history_frame = history.home;
                home_diagnostic.sample_configuration = envelope.home_sample.configuration_epoch;
                home_diagnostic.history_configuration = history.configuration;
                if (!history.entries.empty())
                  home_diagnostic.start_generation = history.entries[envelope.home_sample.cycle % history.entries.size()].generation;
                home_diagnostic.latest_generation = history.latest.generation;
                home_diagnostic.stationary_generation = history.stationary_generation;
                home_diagnostic.sample_cycle = envelope.home_sample.cycle;
                home_diagnostic.stationary_since_cycle = history.stationary_since_cycle;
                home_diagnostic.stationary_min = history.stationary_min;
                home_diagnostic.stationary_max = history.stationary_max;
                coordinate_reference[axis] = rosie::rtcore::invalidate_coordinate(candidate.next,
                    candidate.accepted ? rosie::rtcore::CoordinateReason::InvalidAnchor : candidate.reason).next;
                decision = {commissioning_state};
                decision.reason = rosie::rtcore::CommissioningReason::NotReady;
              } else {
                coordinate_reference[axis] = candidate.next;
                home_coordinate_committed_mask |= bit;
              }
            }
          }
          if (!decision.accepted && new_independent_bindings) {
            // A tentative origin belongs to this CompleteHome transaction. A
            // later qualification failure must not leave it available to normal
            // sampling. A pre-existing qualified origin is never touched here.
            for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
              const uint32_t bit = 1u << axis;
              if (!(new_independent_bindings & bit)) continue;
              auto& history = sampled_bindings[axis];
              history.bound = false; history.failed = true;
              history.metrics.valid = history.metrics.startup_verified = 0;
              history.metrics.reason = history.metrics.last_invalidity_reason = uint32_t(rosie::rtcore::SampleInvalidity::Discontinuity);
              cycle_absolute_valid[axis] = false;
              home_coordinate_committed_mask &= ~bit;
              coordinate_reference[axis] = rosie::rtcore::invalidate_coordinate(coordinate_reference[axis],
                  rosie::rtcore::CoordinateReason::InvalidAnchor).next;
              sampled_consumption[axis].publish(history.metrics);
            }
          }
          apply_commissioning(decision, &request);
          if (decision.accepted && decision.complete_home_mask && envelope.restoring) {
            for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
              if (decision.complete_home_mask & (1u << axis))
                restored_anchor_home_epoch[axis].store(envelope.anchor.home_epoch);
          }
          const auto identity = decision.accepted && (request.operation == rosie::rtcore::CommissioningOperation::Reserve || request.operation == rosie::rtcore::CommissioningOperation::ReserveReference)
              ? decision.state.identity : request.identity;
          if (!commissioning_receipts.push({request.ticket, identity, decision.reason,
                                            decision.accepted, decision.job_completed, home_diagnostic}))
            revoke_outputs();
        }
        for (unsigned consumed = 0; consumed < 16; ++consumed) {
          HomeTruthReadback readback{};
          if (!home_truth_readbacks.pop(readback)) break;
          if (readback.axis >= opt.num_axes || !readback.ticket) continue;
          if (
              commissioning_state.phase == rosie::rtcore::CommissioningPhase::Idle &&
              !armed.load() && !(control_policy.home_valid_mask.load() & (1u << readback.axis)) &&
              readback.generation == control_policy.generation.load() &&
              readback.configuration_epoch == configuration_epoch.load() &&
              readback.home_epoch == home_epoch.load() &&
              readback.audit_generation == audit_generation.load()) {
            latest_feedback.native_home_position_offset[readback.axis] = readback.offset;
            const uint32_t axis = readback.axis, bit = 1u << axis;
            auto& history = sampled_bindings[axis];
            const bool retained_candidate = axis_profiles[axis].independent_absolute_origin();
            bool restored = false;
            if (retained_candidate && readback.calibration_reason == 2) {
              calibration_valid_mask.fetch_and(~bit);
              recovery_position_reason[axis] = rosie::rtcore::CoordinateReason::IdentityChanged;
            }
            // load_restore_anchor admits only drive-homed (v3/v4) anchors; a
            // current-pose v5 file never reaches here (kCurrentReferenceAnchorRejected).
            if (retained_candidate && readback.retained_anchor_valid) {
              if (readback.anchor.offset == readback.offset) calibration_valid_mask.fetch_or(bit);
              else {
                calibration_valid_mask.fetch_and(~bit);
                recovery_position_reason[axis] = rosie::rtcore::CoordinateReason::OffsetChanged;
                retained_home_refusal[axis].store(uint32_t(rosie::rtcore::RetainedHomeRefusal::OffsetChanged));
              }
            }
            if (retained_candidate && readback.retained_anchor_valid &&
                (calibration_valid_mask.load() & bit) && !axis_enable_mask.load() &&
                readback.sample.audit_generation == audit_generation.load() &&
                (coordinate_verified_mask & bit) && coordinate_verified_epoch == coordinate_configuration_epoch &&
                (cycle_feedback_mask & bit)) {
              using Refusal = rosie::rtcore::RetainedHomeRefusal;
              recovery_position_reason[axis] = readback.sample.valid ? rosie::rtcore::CoordinateReason::InvalidAnchor : rosie::rtcore::CoordinateReason::SourceInvalid;
              const auto& saved = readback.anchor;
              const auto pd = cycle_backend.axis(axis);
              const bool error_mapped = off[axis].err != BusHandles::kInvalidOffset;
              const rosie::rtcore::RetainedHomeDriveHealth health{pd.read_u16(off[axis].sw),
                  error_mapped ? pd.read_u16(off[axis].err) : uint16_t{0}, !error_mapped || cycle_drive_error(axis),
                  readback.alarm_objects_read, readback.encoder_fault_state, readback.multi_turn_overflow};
              auto refusal = axis_profiles[axis].retained_home_restore_enabled
                  ? rosie::rtcore::retained_home_drive_refusal(health, axis_profiles[axis].retained_home_requires_homing_attained)
                  : Refusal::Disabled;
              int64_t command = 0;
              const auto margin = std::max(saved.tolerance_counts,
                  rosie::rtcore::SampledPdoHistory::maximum_home_acquisition_displacement_counts);
              // A fresh stationary proof can replace a failed tracking session;
              // it cannot clear the fault latch or reuse pre-loss motion authority.
              int64_t low = 0, high = 0;
              const bool stationary = history.query_history_interval(readback.sample.cycle, history.latest.cycle, low, high) &&
                  __int128(high) - low <= margin;
              if (refusal == Refusal::None && !stationary) refusal = Refusal::Unstationary;
              if (refusal == Refusal::None && !rosie::rtcore::retained_home_anchor_matches(readback.sample,
                  pd.read_s32(off[axis].pos), saved.counts, saved.turns, saved.offset, readback.offset,
                  margin, command)) refusal = Refusal::Disagreement;
              if (refusal == Refusal::None) {
                const auto position = coordinate_model(axis, saved.offset).command_to_logical(command);
                if (!position.valid || !axis_profiles[axis].limits.contains(position.logical)) refusal = Refusal::HardLimit;
              }
              retained_home_refusal[axis].store(uint32_t(refusal));
              if (refusal == Refusal::None) {
                history.bound = false; history.failed = false;
                if (!history.restore_retained_origin(readback.sample, sampled_at_receipt[axis],
                      coordinate_policy_for(axis), feedback_observed_ns, command, false)) {
                  home_truth_processed[axis].store(readback.ticket); continue;
                }
                cycle_absolute_valid[axis] = true;
                cycle_absolute[axis] = sampled_at_receipt[axis];
                const auto sample = coordinate_sample_for(axis);
                const auto candidate = rosie::rtcore::reconcile_coordinate(coordinate_reference[axis],
                    coordinate_identity_for(axis), coordinate_policy_for(axis), sample,
                    coordinate_anchor_for(axis, sample));
                if (candidate.accepted) {
                  coordinate_reference[axis] = candidate.next;
                  home_coordinate_committed_mask |= bit;
                  control_policy.home_valid_mask.fetch_or(bit);
                  restored_anchor_home_epoch[axis].store(saved.home_epoch);
                  for (auto& evidence : fault_evidence) evidence.untrusted_axis_mask &= ~bit;
                  recovery_position_reason[axis] = rosie::rtcore::CoordinateReason::None;
                  restored = true;
                } else {
                  history.bound = false;
                  cycle_absolute_valid[axis] = false;
                }
                sampled_consumption[axis].publish(history.metrics);
              }
            }
            // Retry missing/stale startup proof; never re-home or write drive
            // registers. Once restored, the ordinary coordinate guard owns it.
            if (!retained_candidate || restored)
              home_truth_applied[readback.axis].store(readback.ticket);
          }
          home_truth_processed[readback.axis].store(readback.ticket);
        }
        // Independent local input lane: consume current latest input directly
        // in the cycle, before either motion mode can claim drive targets.
        rosie::rtcore::JogSessionContext jog_context{};
        jog_context.identity = jog_identity(fast_cycle_region ? fast_cycle_region->connection_id() : 0);
        jog_context.now_ns = inputs.clock();
        jog_context.grant_deadline_ns = application_grant.current.deadline_host_ns;
        jog_context.max_input_age_ns = opt.control_max_jog_input_age_ns;
        jog_context.max_first_input_wait_ns = opt.control_max_jog_input_age_ns;
        jog_context.configured_mask = configured_mask;
        jog_context.allowed_mask = application_grant.current.axis_mask;
        jog_context.authority_valid = application_grant.active && lease_is_valid_now() &&
            application_grant.current.identity.native_generation == jog_context.identity.native_generation;
        jog_context.clock_mapping_valid = true; // Local ABI CLOCK_MONOTONIC mapping 1 only.
        jog_context.armed = armed.load();
        const bool was_fast_jog = jog_lane.state().open;
        jog_context.conflicting_motion = active_trajectory || active_halt || halt_admitted_this_cycle || (active_jog && !was_fast_jog);
        jog_context.commissioning_active = commissioning_state.phase != rosie::rtcore::CommissioningPhase::Idle;
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
          if ((cycle_motion_ready_mask & axis_enable_mask.load() & (1u << axis)) && axes_ready(1u << axis))
            jog_context.ready_mask |= 1u << axis;
        // Path jog limits from compiled configuration, in logical joint units:
        // the jog lane's command_limits interval shrunk by one count, the
        // described max velocity, jog_acceleration, a one-count start
        // tolerance and the machine jog_jerk (one value for every axis; 0
        // leaves path jog unavailable). Held targets are the wire targets.
        // Braking margins cover the longest cycle interval that does not
        // already raise FaultCycleDeadline: cycle + max_cycle_lateness_ns.
        jog_lane.set_path_timing(opt.num_axes ? double(axis_profiles[0].jog.jerk) : 0,
            double(gradient::ipc::v1::PATH_JOG_MAX_SAMPLE_STEP_NS) / 1e9, opt.cycle_ns,
            opt.cycle_ns + opt.max_cycle_lateness_ns);
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          const auto model = coordinate_model(axis, active_reference_offset(axis));
          jog_lane.set_limit_feedback(axis, axis_profiles[axis].limits,
              axis_profiles[axis].coordinate_evidence.enabled
                  ? latest_feedback.coordinate_counts[axis] : latest_feedback.pos_counts[axis],
              model.logical_delta_to_counts(1), model.profile.home_offset);
          const auto& limits = axis_profiles[axis].limits;
          rosie::rtcore::PathJogAxisConfig path_axis{};
          if (limits.valid && model.scaling_valid()) {
            const double count = 1.0 / opt.axis[axis].counts_per_unit;
            path_axis.min_q = limits.min_q + count;
            path_axis.max_q = limits.max_q - count;
            path_axis.velocity = std::min(limits.max_qd,
                double(opt.axis[axis].max_profile_vel_counts_per_s) / opt.axis[axis].counts_per_unit);
            path_axis.acceleration = limits.jog_acceleration;
            path_axis.start_tolerance = count;
          }
          const int32_t held = have_last_wire_target[axis] ? last_wire_target[axis]
              : csp_wire_counts_from_feedback(latest_feedback.pos_counts[axis]);
          jog_lane.set_path_axis(axis, path_axis, model.command_to_logical(held).logical);
        }
        const bool jog_inhibit = jog_lane.cycle(
            fast_cycle_region && fast_cycle_region->connection_id() != failed_connection_id ? fast_cycle_region : nullptr,
            jog_context, jog_velocity_limits);
        jog_session_open.store(jog_lane.state().open, std::memory_order_release);
        jog_evaluated = true;
        if (jog_inhibit) inhibit_outputs();
        if (jog_lane.state().open) {
          const auto& input = jog_lane.state();
          active_jog = true;
          active_jog_axis_mask = input.axis_mask;
          active_jog_deadline_ns = input.input_deadline_ns;
          active_jog_cmd_seq = input.source_sequence;
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
            active_jog_velocity_counts_per_s[axis] = coordinate_model(axis, 0).logical_velocity_to_counts(input.velocity[axis]);
          if (!was_fast_jog) have_jog_target.fill(false);
        } else if (was_fast_jog) {
          active_jog = false; active_jog_axis_mask = 0; active_jog_deadline_ns = 0;
          have_jog_target.fill(false);
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_ABORTED);
          motion_done.store(1);
        }
        const int candidate_slot = trajectory_store.acquire_ready();
        if (candidate_slot >= 0) {
          const auto& candidate = trajectory_store.view(candidate_slot);
          rosie::protocol::CommandResult result{candidate.start_sequence, candidate.id,
              candidate.generation, rosie::protocol::msg_trajectory_start,
              rosie::protocol::result_rejected, rosie::protocol::reason_not_ready, candidate.axis_mask};
          rosie::rtcore::MotionStartState start_state{};
          start_state.generation = generation_now;
          start_state.configuration_epoch = configuration_epoch.load();
          start_state.verified_configuration_epoch = verified_configuration_epoch.load();
          start_state.configured_mask = configured_mask;
          start_state.config_verified_mask = control_policy.config_verified_mask.load();
          start_state.home_valid_mask = control_policy.home_valid_mask.load();
          start_state.enabled_mask = axis_enable_mask.load();
          start_state.motion_ready_mask = cycle_motion_ready_mask;
          start_state.safety_fault_mask = safety_fault_mask.load();
          start_state.lease_valid = lease_is_valid_now();
          start_state.armed = armed.load();
          start_state.bus_ready = bus_ready.load();
          start_state.trajectory_active = active_trajectory;
          start_state.jog_active = active_jog || active_halt || halt_admitted_this_cycle;
          start_state.commissioning_active =
              commissioning_state.phase != rosie::rtcore::CommissioningPhase::Idle;
          std::array<rosie::rtcore::MotionStartAxis,
                     gradient::ipc::v1::GRADIENT_MAX_AXES> start_axes{};
          const auto& first = candidate.points[0];
          const auto first_sample = rosie::rtcore::sample_trajectory(candidate.points.data(),
              candidate.points.size(), 0);
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            const uint32_t bit = 1u << axis;
            if (axis_profiles[axis].limits.require_home) start_state.home_required_mask |= bit;
            if (axis_profiles[axis].limits.valid) start_state.limits_valid_mask |= bit;
            if (latest_feedback.mode_display[axis] == 8) start_state.csp_mode_mask |= bit;
            start_axes[axis].first_target = first.target_counts[axis];
            start_axes[axis].held_target = have_last_wire_target[axis] ? last_wire_target[axis]
                : int32_t(latest_feedback.pos_counts[axis]);
            const auto model = coordinate_model(axis, active_reference_offset(axis));
            const auto& profile = axis_profiles[axis];
            start_axes[axis].completion_tolerance_counts =
                rosie::rtcore::independent_encoder_start_alignment_allowance(
                    std::abs(model.logical_delta_to_counts(profile.limits.completion_tolerance)),
                    double(profile.start_alignment_floor_counts),
                    profile.independent_absolute_origin(), coordinate_policy_for(axis).qualified,
                    (start_state.home_valid_mask & bit) != 0,
                    start_state.configuration_epoch == start_state.verified_configuration_epoch &&
                        (start_state.config_verified_mask & bit) != 0,
                    coordinate_reference[axis].valid && cycle_absolute_valid[axis] &&
                        sampled_bindings[axis].bound && !sampled_bindings[axis].failed);
            start_axes[axis].first_velocity = first_sample.velocity[axis];
            start_axes[axis].max_velocity_counts_per_s = profile.limits.max_qd > 0
                ? std::min(std::abs(model.logical_delta_to_counts(profile.limits.max_qd)),
                    double(opt.axis[axis].max_profile_vel_counts_per_s)) : 0;
            const double alignment_acceleration = profile.max_acceleration
                ? *profile.max_acceleration : profile.limits.jog_acceleration;
            start_axes[axis].max_acceleration_counts_per_s2 = alignment_acceleration > 0
                ? std::abs(model.logical_delta_to_counts(alignment_acceleration)) : 0;
            // Prepared counts retain fractions until output projection. Do not
            // truncate an outward sub-count request to the integer held pose.
            const auto limits = rosie::rtcore::command_limits(axis_profiles[axis].limits,
                model.logical_delta_to_counts(1), model.profile.home_offset);
            start_axes[axis].limit_direction_valid = model.scaling_valid() &&
                rosie::rtcore::limit_recovery_step(limits, start_axes[axis].held_target,
                    first.target_counts[axis]) &&
                rosie::rtcore::limit_recovery_velocity(limits, first.target_counts[axis],
                    first.velocity_counts_per_s[axis]);
          }
          const rosie::rtcore::MotionStartRequest start_request{candidate.generation,
              first.t_from_start_ns, candidate.axis_mask, trajectory_store.abort_requested(candidate_slot), period};
          const auto start_decision = rosie::rtcore::evaluate_motion_start(
              start_state, start_request, start_axes);
          if (start_decision.reason == rosie::rtcore::MotionStartReason::ModeConflict) {
            result.reason = rosie::protocol::reason_mode_conflict;
          } else if (start_decision.reason == rosie::rtcore::MotionStartReason::OutsideLimitsOutward) {
            result.reason = rosie::protocol::reason_outside_limits_outward;
          } else if (start_decision.reason == rosie::rtcore::MotionStartReason::InvalidTrajectory) {
            for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
              if (!(candidate.axis_mask & (1u << axis))) continue;
              const auto& start = start_axes[axis];
              logf("Trajectory start refused seq=%llu trajectory=%llu axis=%u first_counts=%.9f "
                   "held_counts=%.9f delta_counts=%.9f alignment_allowance_counts=%.9f "
                   "first_velocity_counts_per_s=%.9f first_time_ns=%llu "
                   "max_velocity_counts_per_s=%.9f max_acceleration_counts_per_s2=%.9f discontinuous=%u",
                   static_cast<unsigned long long>(candidate.start_sequence),
                   static_cast<unsigned long long>(candidate.id), axis,
                   start.first_target, start.held_target,
                   rosie::rtcore::command_delta(start.first_target, start.held_target),
                   start.completion_tolerance_counts, start.first_velocity,
                   static_cast<unsigned long long>(first.t_from_start_ns),
                   start.max_velocity_counts_per_s, start.max_acceleration_counts_per_s2,
                   (start_decision.discontinuous_mask >> axis) & 1u);
            }
            result.reason = rosie::protocol::reason_invalid_trajectory;
            execution_fault(rosie::rtcore::FaultStartDiscontinuity, start_decision.discontinuous_mask);
            active_trajectory = false; active_jog = false;
            active_traj_point_count = 0;
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED);
            motion_last_event_code.store(gradient::ipc::v1::EVT_TRAJECTORY_FAULTED);
            motion_done.store(1); motion_queue_depth.store(0);
          } else if (start_decision.activate && first.next_io_index<candidate.points.size() && !cell_io.state().armed) {
            result.reason=rosie::protocol::reason_io_not_armed;
          } else if (start_decision.activate) {
            active_trajectory = true;
            active_jog = false;
            active_jog_axis_mask = 0;
            active_jog_deadline_ns = 0;
            have_jog_target.fill(false);
            jog_stop_quick_stop_cycles_left.fill(0);
            jog_stop_arrest_cycles_left.fill(0);
            active_traj_id_rt = candidate.id;
            active_traj_cmd_seq = candidate.start_sequence;

            active_traj_point_count = candidate.points.size();

            active_traj_start_ns = wake_target_ns;
            start_alignment.begin(start_axes, candidate.axis_mask, diag_now_ns,
                                  start_decision.alignment_duration_ns);
            completion_state = {};
            active_plan_slot = candidate_slot;
            active_traj_points = candidate.points.data();
            next_io_point = first.next_io_index;
            next_wait_point = first.next_wait_index; input_wait.reset();
            result.result = rosie::protocol::result_accepted;
            result.reason = rosie::protocol::reason_none;
            motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_EXECUTING, std::memory_order_relaxed);
            motion_active_traj_id.store(active_traj_id_rt, std::memory_order_relaxed);
            motion_current_point_index.store(0, std::memory_order_relaxed);
            motion_queue_depth.store(
                active_traj_point_count > 0 ? (active_traj_point_count - 1) : 0,
                std::memory_order_relaxed);
            motion_last_event_code.store(
                gradient::ipc::v1::EVT_TRAJECTORY_EXECUTING, std::memory_order_relaxed);
            motion_stale_command_flag.store(0, std::memory_order_relaxed);
            motion_done.store(0, std::memory_order_relaxed);
            motion_active_command_seq.store(active_traj_cmd_seq, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          }
          if (result.result != rosie::protocol::result_accepted) trajectory_store.retire(candidate_slot);
          if (!enqueue_rt_result(result)) revoke_outputs();
        }
        // Jog snapshots are small and never make the cyclic thread wait.
        JogCommandRuntime latest_jog_command{};
        if (jog_commands.read(latest_jog_command)) {
        const uint64_t jog_seq = latest_jog_command.seq;
        if (jog_seq != 0 && jog_seq != active_jog_seq_seen &&
            latest_jog_command.generation == generation_now) {
          active_jog_seq_seen = jog_seq;
          rosie::protocol::CommandResult jog_result{latest_jog_command.cmd_seq, 0, latest_jog_command.generation,
              gradient::ipc::v1::MSG_CMD_JOG, rosie::protocol::result_rejected,
              jog_lane.state().open || active_trajectory || active_halt || halt_admitted_this_cycle || commissioning_state.phase != rosie::rtcore::CommissioningPhase::Idle
                  ? rosie::protocol::reason_mode_conflict : rosie::protocol::reason_not_ready,
              latest_jog_command.axis_mask};
          bool jog_direction_valid = true;
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            if (!(latest_jog_command.axis_mask & (1u << axis))) continue;
            const auto model = coordinate_model(axis, active_reference_offset(axis));
            const double feedback = axis_profiles[axis].coordinate_evidence.enabled
                ? latest_feedback.coordinate_counts[axis] : latest_feedback.pos_counts[axis];
            jog_direction_valid &= model.scaling_valid() && rosie::rtcore::limit_recovery_velocity(
                rosie::rtcore::command_limits(axis_profiles[axis].limits,
                    model.logical_delta_to_counts(1), model.profile.home_offset), feedback,
                latest_jog_command.velocity_counts_per_s[axis]);
          }
          if (!jog_lane.state().open && !active_trajectory && !active_halt && !halt_admitted_this_cycle && armed.load() && axes_ready(latest_jog_command.axis_mask) &&
              rosie::rtcore::motion_group_ready(latest_jog_command.axis_mask, configured_mask,
                                                cycle_motion_ready_mask) &&
              (axis_enable_mask.load() & latest_jog_command.axis_mask) == latest_jog_command.axis_mask) {
          if (!jog_direction_valid) {
            jog_result.reason = rosie::protocol::reason_outside_limits_outward;
          } else {
          jog_result.result = rosie::protocol::result_accepted;
          jog_result.reason = rosie::protocol::reason_none;
          active_jog_cmd_seq = latest_jog_command.cmd_seq;
          latest_jog_axis_mask = latest_jog_command.axis_mask;
          latest_jog_flags = latest_jog_command.flags;
          latest_jog_timeout_ns = latest_jog_command.timeout_ns;
          active_jog = true;
          active_jog_axis_mask = latest_jog_command.axis_mask;
          active_jog_deadline_ns =
              diag_now_ns + std::max<uint64_t>(latest_jog_command.timeout_ns, period);
          for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((active_jog_axis_mask & (1u << i)) != 0u) {
              jog_stop_quick_stop_cycles_left[i] = 0;
              jog_stop_arrest_cycles_left[i] = 0;
            }
            active_jog_velocity_counts_per_s[i] = latest_jog_command.velocity_counts_per_s[i];
            if ((active_jog_axis_mask & (1u << i)) == 0u) {
              have_jog_target[i] = false;
            }
          }
          }
          }
          if (!enqueue_rt_result(jog_result)) revoke_outputs();
        }

        }

        if (jog_lane.expire_before_dispatch(inputs.clock(), active_jog_velocity_counts_per_s, active_jog_deadline_ns)) {
          inhibit_outputs();
          jog_session_open.store(false, std::memory_order_release);
          active_jog = false;
          active_jog_axis_mask = 0;
          active_jog_deadline_ns = 0;
          have_jog_target.fill(false);
        }
        if (active_halt) {
          bool stopped = true;
          sp_mask = halt_axis_mask;
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            if (!(halt_axis_mask & (1u << axis))) continue;
            const auto model = coordinate_model(axis, 0);
            // Guard the candidate target against this exchange, including the
            // final ramp frame; published feedback still describes the prior cycle.
            const auto feedback = cycle_backend.axis(axis).read_s32(off[axis].pos);
            const auto& tracking = axis_profiles[axis].limits;
            const double lead_limit = std::abs(model.logical_delta_to_counts(tracking.max_target_lead));
            // False means invalid input or an overrun stopping bound, not lead.
            if (!halt_ramps[axis].advance(halt_axes[axis], feedback, lead_limit,
                inputs.clock(),
                double(tracking.tracking_lag_allowance_ns) / 1e9)) {
              execution_fault(rosie::rtcore::FaultTargetLead, 1u << axis);
              sp_mask = 0; break;
            }
            note_target_lead(axis, halt_axes[axis], inputs.clock(), 2);
            const auto projected = model.project_command(halt_axes[axis].position);
            if (!projected.valid) {
              execution_fault(rosie::rtcore::FaultCoordinateReference, 1u << axis);
              sp_mask = 0; break;
            }
            target_counts[axis] = projected.counts;
            target_velocity_counts_per_s[axis] = halt_axes[axis].velocity;
            stopped &= halt_axes[axis].velocity == 0;
          }
          // Keep the motion guards armed through this final zero-velocity frame.
          if (stopped && sp_mask) {
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_ABORTED);
            motion_done.store(1);
          }
        } else if (active_trajectory && active_traj_point_count > 0) {
          rosie::rtcore::TrajectorySample sample{};
          if (start_alignment.active) {
            sample.axis_mask = start_alignment.axis_mask;
            if (start_alignment.sample(diag_now_ns, sample.position, sample.velocity))
              active_traj_start_ns = wake_target_ns;
          } else {
            if(next_wait_point<active_traj_point_count) {
              const auto& barrier=active_traj_points[next_wait_point];
              const uint64_t elapsed=diag_now_ns>=active_traj_start_ns ? diag_now_ns-active_traj_start_ns : 0;
              if(elapsed>=barrier.t_from_start_ns) {
                const auto& input=cell_io.state().inputs[barrier.wait_input];
                const auto wait_result=input_wait.step(diag_now_ns,input.valid,input.value,barrier.wait_value,
                                                       barrier.wait_timeout_ns,barrier.wait_stable_ns);
                active_traj_start_ns=diag_now_ns-barrier.t_from_start_ns;
                if(wait_result==rosie::rtcore::IoWait::Result::satisfied) {
                  active_traj_start_ns=wake_target_ns-barrier.t_from_start_ns;
                  next_wait_point=next_wait_point+1<active_traj_point_count ? active_traj_points[next_wait_point+1].next_wait_index : UINT32_MAX;
                  input_wait.reset();
                } else if(wait_result==rosie::rtcore::IoWait::Result::pending) {
                  cell_io.refresh_program_wait(diag_now_ns);
                } else {
                  cell_io.fault(wait_result==rosie::rtcore::IoWait::Result::timed_out ?
                      rosie::protocol::reason_io_wait_timeout : rosie::protocol::reason_io_wait_input_invalid);
                  execution_fault(rosie::rtcore::FaultIoWait,configured_mask);
                  active_trajectory=false;
                }
              }
            }
            sample = rosie::rtcore::sample_trajectory(active_traj_points, active_traj_point_count,
                diag_now_ns >= active_traj_start_ns ? diag_now_ns - active_traj_start_ns : 0);
          }

          const auto& monitored = active_traj_points[sample.index];
          for (unsigned i=0; i<opt.cell_io.input_count; ++i) {
            if (!(monitored.monitor_mask & (1u<<i))) continue;
            const auto& in = cell_io.state().inputs[i];
            if (!in.valid || bool(in.value) != bool(monitored.monitor_values & (1u<<i))) {
              cell_io.fault(rosie::protocol::reason_io_wait_input_invalid);
              execution_fault(rosie::rtcore::FaultIoWait, configured_mask);
              active_trajectory=false; break;
            }
          }
          sp_mask = active_trajectory ? sample.axis_mask : 0;
          target_velocity_counts_per_s = sample.velocity;
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            const auto projected = coordinate_model(axis, 0).project_command(sample.position[axis]);
            if (!projected.valid) {
              execution_fault(rosie::rtcore::FaultCoordinateReference, 1u << axis);
              active_trajectory = false; sp_mask = 0; break;
            }
            target_counts[axis] = projected.counts;
            trajectory_command_axes[axis] = {sample.position[axis], sample.velocity[axis]};
          }
          motion_current_point_index.store(sample.index);
          motion_queue_depth.store(active_traj_point_count - sample.index - 1);
        } else if (active_jog && jog_lane.path_mode()) {
          // Path jog: every masked axis comes from PathJog's one path scalar,
          // so all axes start, slow and stop together on the sampled curve.
          // No JogSmoother and no per-axis ramp touch these targets; the
          // target-lead warning, position-rate guard, output policy and
          // following-error fault run downstream exactly as for a joint jog.
          const uint32_t mask = active_jog_axis_mask;
          // The path is sampled at this cycle's wake time, as trajectories
          // are, never at a clock read here: that read follows this cycle's
          // intake, whose path_samples append (~4 us per sample on
          // the servo host) varies cycle to cycle, while the drive applies every
          // target on the fixed sync. On a Rosie 1430 arm (2026-10-07/08) the target led
          // by that cycle's work, up to ~70 us at each 25-sample chunk: one-
          // cycle accelerations of 50-146 rad/s^2 against 4*pi.
          const uint64_t now = wake_ns;
          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_JOG, std::memory_order_relaxed);
          motion_active_traj_id.store(0, std::memory_order_relaxed);
          motion_current_point_index.store(std::numeric_limits<uint32_t>::max(), std::memory_order_relaxed);
          motion_queue_depth.store(0, std::memory_order_relaxed);
          motion_last_event_code.store(0, std::memory_order_relaxed);
          motion_stale_command_flag.store(0, std::memory_order_relaxed);
          motion_done.store(0, std::memory_order_relaxed);
          motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          motion_exec_state.store(jog_lane.path_begun() && path_jog.speed() != 0
              ? gradient::ipc::v1::EXEC_STATE_EXECUTING : gradient::ipc::v1::EXEC_STATE_ACCEPTED,
              std::memory_order_relaxed);
          if (jog_lane.path_begun()) {
            auto cycle = path_jog.advance(now);
            bool failed = cycle.fault;
            if (cycle.active && !failed) {
              sp_mask = mask;
              for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                if (!(mask & (1u << i))) continue;
                const auto model = coordinate_model(i, active_reference_offset(i));
                const double counts = model.logical_delta_to_counts(path_jog.position()[i]) - model.profile.home_offset;
                const double velocity = model.logical_velocity_to_counts(path_jog.velocity()[i]);
                const auto& tracking = axis_profiles[i].limits;
                const double lead_limit = std::abs(model.logical_delta_to_counts(tracking.max_target_lead));
                if (!jog_axes[i].follow(counts, velocity, latest_feedback.pos_counts[i], lead_limit,
                        double(tracking.tracking_lag_allowance_ns) / 1e9)) {
                  failed = true; break;
                }
                note_target_lead(i, jog_axes[i], wake_ns, 1);
                have_jog_target[i] = true;
                jog_target_counts_float[i] = counts;
                const auto projected = coordinate_model(i, 0).project_command(counts);
                if (!projected.valid) {
                  execution_fault(rosie::rtcore::FaultCoordinateReference, 1u << i);
                  failed = true; break;
                }
                target_counts[i] = projected.counts;
                target_velocity_counts_per_s[i] = velocity;
              }
            }
            if (failed) {
              // Fail closed: a stop-bound overrun, a passed playable end or an
              // invalid target. FaultTargetLead is the retained bit the
              // per-axis HaltRamp already raises for an overrun stop.
              execution_fault(rosie::rtcore::FaultTargetLead, mask);
              cycle.fault = cycle.closed = true;
              sp_mask = 0;
            }
            if (jog_lane.path_cycle(cycle, now)) inhibit_outputs();
            if (!jog_lane.state().open) {
              // The stop completed (or failed) this cycle; its final target
              // above is still emitted. The next intake sees no open session.
              jog_session_open.store(false, std::memory_order_release);
              active_jog = false; active_jog_axis_mask = 0; active_jog_deadline_ns = 0;
              have_jog_target.fill(false);
              motion_exec_state.store(failed ? gradient::ipc::v1::EXEC_STATE_FAULTED : gradient::ipc::v1::EXEC_STATE_ABORTED);
              motion_done.store(1);
            }
          }
        } else if (active_jog) {
          bool any_nonzero_velocity = jog_lane.ramping();
          for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((active_jog_axis_mask & (1u << i)) == 0u) {
              continue;
            }
            if (std::abs(active_jog_velocity_counts_per_s[i]) > 1e-6 || std::abs(jog_axes[i].velocity) > 1e-6) {
              any_nonzero_velocity = true;
              break;
            }
          }

          if (active_jog_deadline_ns != 0 && diag_now_ns > active_jog_deadline_ns) {
            for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
              if ((active_jog_axis_mask & (1u << i)) != 0u) {
                jog_stop_quick_stop_cycles_left[i] = std::max<uint16_t>(
                    jog_stop_quick_stop_cycles_left[i],
                    axis_profiles[i].jog.quick_stop_cycles);
                jog_stop_arrest_cycles_left[i] = std::max<uint16_t>(
                    jog_stop_arrest_cycles_left[i],
                    axis_profiles[i].jog.arrest_cycles);
              }
            }
            snap_jog_hold_to_feedback_mask |= active_jog_axis_mask;
            last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_TIMEOUT;
            last_jog_stop_time_ns = diag_now_ns;
            last_jog_stop_cmd_seq = active_jog_cmd_seq;
            last_jog_stop_axis_mask = active_jog_axis_mask;
            active_jog = false;
            active_jog_axis_mask = 0;
            active_jog_deadline_ns = 0;
            have_jog_target.fill(false);
            motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_JOG, std::memory_order_relaxed);
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_ABORTED, std::memory_order_relaxed);
            motion_active_traj_id.store(0, std::memory_order_relaxed);
            motion_current_point_index.store(std::numeric_limits<uint32_t>::max(),
                                             std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_last_event_code.store(gradient::ipc::v1::EVT_JOG_TIMEOUT, std::memory_order_relaxed);
            motion_stale_command_flag.store(1, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          } else {
            motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_JOG, std::memory_order_relaxed);
            motion_exec_state.store(
                any_nonzero_velocity ? gradient::ipc::v1::EXEC_STATE_EXECUTING
                                     : gradient::ipc::v1::EXEC_STATE_ACCEPTED,
                std::memory_order_relaxed);
            motion_active_traj_id.store(0, std::memory_order_relaxed);
            motion_current_point_index.store(std::numeric_limits<uint32_t>::max(),
                                             std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_last_event_code.store(0, std::memory_order_relaxed);
            motion_stale_command_flag.store(0, std::memory_order_relaxed);
            motion_done.store(0, std::memory_order_relaxed);
            motion_active_command_seq.store(active_jog_cmd_seq, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);

            if (any_nonzero_velocity) {
              sp_mask = active_jog_axis_mask;
              for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                if ((active_jog_axis_mask & (1u << i)) == 0u) {
                  continue;
                }
                if (!have_jog_target[i]) {
                  if (have_hold[i]) {
                    jog_target_counts_float[i] =
                        static_cast<double>(hold_target_counts[i]);
                  } else {
                    jog_target_counts_float[i] = static_cast<double>(
                        csp_wire_counts_from_feedback(latest_feedback.pos_counts[i]));
                  }
                  jog_axes[i].reset(jog_target_counts_float[i]);
                  have_jog_target[i] = true;
                }
              }
              auto governed_velocity = active_jog_velocity_counts_per_s;
              jog_lane.govern(active_jog_axis_mask, jog_axes, governed_velocity,
                  double(wake_ns - previous_cycle_ns) / 1e9);
              for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
                if (!(active_jog_axis_mask & (1u << i))) continue;
                const auto& tracking = axis_profiles[i].limits;
                const auto model = coordinate_model(i, 0);
                const double lead_limit = std::abs(model.logical_delta_to_counts(tracking.max_target_lead));
                // False means invalid input or an overrun ramp bound, not lead.
                if (!jog_lane.advance(i, jog_axes[i], governed_velocity[i], latest_feedback.pos_counts[i],
                        std::abs(model.logical_delta_to_counts(tracking.jog_acceleration)), lead_limit,
                        double(wake_ns - previous_cycle_ns) / 1e9,
                        double(tracking.tracking_lag_allowance_ns) / 1e9)) {
                  execution_fault(rosie::rtcore::FaultTargetLead, 1u << i);
                  active_jog = false; sp_mask = 0; break;
                }
                note_target_lead(i, jog_axes[i], wake_ns, 1);
                jog_target_counts_float[i] = jog_axes[i].position;
                const auto projected = coordinate_model(i, 0).project_command(jog_target_counts_float[i]);
                if (!projected.valid) {
                  execution_fault(rosie::rtcore::FaultCoordinateReference, 1u << i);
                  active_jog = false; sp_mask = 0; break;
                }
                target_counts[i] = projected.counts;
                target_velocity_counts_per_s[i] = jog_axes[i].velocity;
              }
            }
          }
        }

        if (!lease_valid_now || !bus_ready.load(std::memory_order_acquire) ||
            verified_configuration_epoch.load() != configuration_epoch.load()) {
          armed.store(false); axis_enable_mask.store(0); service_mode_axis_mask.store(0);
          sp_mask = 0;
        }
        // General drive-state supervision: every axis, independent of brake
        // configuration. An enable that never completes, a drop out of
        // Operation Enabled while armed, or a cancelled Arm stage ends this Arm
        // attempt with a reason before any output is written this cycle.
        {
          bool end_arm = false;
          const bool supervised = armed.load() && lease_valid_now && !g_stop.load() && !safety_fault_mask.load();
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            const uint32_t bit = 1u << axis;
            auto& supervisor = drive_supervisors[axis];
            const auto& arm_stage = arm_opening_stages[axis];
            rosie::rtcore::DriveEnableObservation observation{};
            observation.now_ns = feedback_observed_ns;
            observation.fresh = (cycle_feedback_mask & bit) != 0;
            observation.statusword = cycle_backend.axis(axis).read_u16(off[axis].sw);
            observation.drive_error = cycle_drive_error(axis);
            observation.enable_requested = supervised && (axis_enable_mask.load() & bit);
            observation.quick_stop_commanded = jog_stop_quick_stop_cycles_left[axis] > 0 ||
                supervisor.last_controlword() == gradient::ds402::CW_QUICK_STOP;
            observation.released = brake_outputs[axis].motion_permitted;
            observation.release_delay_ns = axis_profiles[axis].brake.present ? axis_profiles[axis].brake.release_delay_ns : 0;
            observation.enable_timeout_ns = axis_profiles[axis].drive_supervision.enable_timeout_ns;
            observation.release_slack_ns = axis_profiles[axis].drive_supervision.release_slack_ns;
            observation.cancelled = arm_stage.cancelled ? arm_stage.cancel_cause : rosie::rtcore::ArmCancelCause::None;
            if (!supervisor.observe_enable(observation)) continue;
            auto evidence = supervisor.evidence();
            evidence.arm_request_sequence = arm_request_sequence.load(std::memory_order_acquire);
            drive_supervisor_evidence[axis].publish(evidence);
            end_arm = true;
          }
          if (end_arm) {
            end_arm_attempt(); // A trajectory retires through the abort request.
            active_jog = false; sp_mask = 0;
          }
        }
        inputs.observe(CyclePhase::Output);
        bool is_armed = armed.load(std::memory_order_relaxed);
        const uint32_t en_mask = axis_enable_mask.load(std::memory_order_relaxed);

        // Latch any new fault reset requests from the helper thread.
        const uint32_t fr_req = fault_reset_request.exchange(0, std::memory_order_acq_rel);
        if (fr_req != 0) {
          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            if ((fr_req & (1u << i)) != 0u) {
              fault_reset_left[i] = kFaultResetPulseCycles;
            }
          }
        }

        // Per-axis DS402 sequencing + CSP targets.
        uint32_t have_hold_mask = 0;
        uint32_t have_jog_target_mask = 0;
        uint32_t stop_arrest_mask = 0;
        uint32_t coordinate_loss_mask = 0;
        for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
          if (jog_stop_arrest_cycles_left[i] > 0) {
            stop_arrest_mask |= (1u << i);
          }
        }
        std::array<rosie::rtcore::InhibitedReference,
                   gradient::ipc::v1::GRADIENT_MAX_AXES> inhibited_references{};
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          const auto axis_pd = cycle_backend.axis(i);
          const uint16_t sw = axis_pd.read_u16(off[i].sw);
          const uint16_t err = off[i].err != BusHandles::kInvalidOffset ? axis_pd.read_u16(off[i].err) : 0;
          const uint32_t manufacturer_err = off[i].manufacturer_err != BusHandles::kInvalidOffset
                                              ? axis_pd.read_u32(off[i].manufacturer_err)
                                              : 0;
          const int32_t pos = axis_pd.read_s32(off[i].pos);
          const int16_t torque = off[i].torque != BusHandles::kInvalidOffset
                                     ? axis_pd.read_s16(off[i].torque)
                                     : 0;
          const uint8_t mode_disp = off[i].mode_disp != BusHandles::kInvalidOffset
                                        ? axis_pd.read_u8(off[i].mode_disp)
                                        : 0;
          const uint32_t di = off[i].di != BusHandles::kInvalidOffset ? axis_pd.read_u32(off[i].di) : 0;

          (void)err; // TODO: publish/report faults.

          const bool service_mode_active =
              commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active &&
              rosie::rtcore::commissioning_permission(commissioning_state.identity, commissioning_context()) &&
              (commissioning_state.service_mask & (1u << i)) != 0u;
          const int32_t commanded_mode = desired_mode_of_operation[i].load(std::memory_order_relaxed);
          const int32_t service_mode = service_mode_override[i].load(std::memory_order_relaxed);
          bool coordinate_valid = false;
          int32_t continuous_feedback = 0;
          rosie::rtcore::CoordinateReason coordinate_reason = rosie::rtcore::CoordinateReason::Unqualified;
          const auto coordinate_sample = coordinate_sample_for(i);
          if (axis_profiles[i].coordinate_evidence.enabled) {
            const bool was_valid = coordinate_reference[i].valid;
            const auto identity = coordinate_identity_for(i);
            const auto policy = coordinate_policy_for(i);
            const bool home_pending = commissioning_state.phase != rosie::rtcore::CommissioningPhase::Idle &&
                (commissioning_state.axis_mask & (1u << i)) &&
                !(commissioning_state.completed_mask & (1u << i));
            if (home_pending) {
              // Home may deliberately change the drive's coordinate frame.
              // Track no ordinary displacement until its qualified completion.
              auto invalid = rosie::rtcore::invalidate_coordinate(coordinate_reference[i],
                  rosie::rtcore::CoordinateReason::Unanchored).next;
              const auto decision = rosie::rtcore::observe_coordinate(invalid, identity, policy, coordinate_sample);
              coordinate_reference[i] = decision.next;
              coordinate_reason = decision.reason;
            } else if (!(home_coordinate_committed_mask & (1u << i))) {
              const auto& state = coordinate_reference[i];
              const auto decision = coordinate_model(i, active_reference_offset(i)).reconcile(
                  state, identity, policy, coordinate_sample, coordinate_anchor_for(i, coordinate_sample));
              coordinate_reference[i] = decision.next;
              coordinate_reason = decision.reason;
            } else {
              coordinate_reason = rosie::rtcore::CoordinateReason::None;
            }
            const bool coordinate_configuration_current =
                configuration_epoch.load() == coordinate_configuration_epoch;
            coordinate_valid = coordinate_sample.evidence.source_valid && coordinate_configuration_current &&
                rosie::rtcore::coordinate_command_counts(coordinate_reference[i], continuous_feedback);
            if (coordinate_reference[i].valid && !coordinate_valid) {
              coordinate_reason = !coordinate_configuration_current
                  ? rosie::rtcore::CoordinateReason::IdentityChanged
                  : !coordinate_sample.evidence.source_valid
                      ? rosie::rtcore::CoordinateReason::SourceInvalid
                      : rosie::rtcore::CoordinateReason::ArithmeticOverflow;
              coordinate_reference[i] = rosie::rtcore::invalidate_coordinate(
                  coordinate_reference[i], coordinate_reason).next;
            }
            if (was_valid && !coordinate_valid && !home_pending) {
              // Loss may mean encoder reset or a changed frame. It retires
              // prepared work even while idle; a later absolute anchor cannot
              // revive pre-loss authority, plans or Home evidence.
              if (axis_coordinate_epoch[i] && axis_coordinate_epoch[i] != UINT64_MAX)
                ++axis_coordinate_epoch[i];
              else
                axis_coordinate_epoch[i] = 0;
              const uint32_t bit = 1u << i;
              control_policy.home_valid_mask.fetch_and(~bit);
              if (!coordinate_loss_mask)
                execution_fault(rosie::rtcore::FaultCoordinateReference, bit);
              else {
                latch_fault_evidence(rosie::rtcore::FaultCoordinateReference, bit);
                safety_fault_mask.fetch_or(bit);
              }
              coordinate_loss_mask |= bit;
              audit_generation.fetch_add(1);
              readbacks_requested.store(true);
              is_armed = false;
              active_trajectory = false;
              active_jog = false;
              sp_mask = 0;
            }
          }
          latest_feedback.coordinate_epoch[i] = axis_coordinate_epoch[i];
          latest_feedback.coordinate_counts[i] = coordinate_reference[i].continuous_counts;
          latest_feedback.absolute_source_counts[i] = axis_profiles[i].coordinate_evidence.sdo_sampled
              ? (cycle_absolute_valid[i] ? static_cast<int32_t>(cycle_absolute[i].encoder_counts) : 0)
              : static_cast<int32_t>(coordinate_anchor_for(i, coordinate_sample).continuous_counts);
          latest_feedback.coordinate_valid[i] = coordinate_valid ? 1u : 0u;
          latest_feedback.coordinate_source_valid[i] = coordinate_sample.evidence.source_valid ? 1u : 0u;
          if (coordinate_valid) recovery_position_reason[i] = rosie::rtcore::CoordinateReason::None;
          else if (recovery_position_reason[i] != rosie::rtcore::CoordinateReason::None)
            coordinate_reason = recovery_position_reason[i];
          latest_feedback.coordinate_reason[i] = static_cast<uint32_t>(coordinate_reason);
          if (!axis_profiles[i].independent_absolute_origin()) {
            if (control_policy.home_valid_mask.load() & (1u << i)) calibration_valid_mask.fetch_or(1u << i);
          }
          latest_feedback.calibration_valid[i] = (calibration_valid_mask.load() >> i) & 1u;
          const bool position_trusted = axis_profiles[i].coordinate_evidence.enabled ? coordinate_valid :
              (cycle_feedback_mask & (1u << i)) && (control_policy.home_valid_mask.load() & (1u << i)) &&
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              (control_policy.config_verified_mask.load() & (1u << i));
          latest_feedback.position_state[i] = position_trusted && latest_feedback.calibration_valid[i] ? 2u :
              latest_feedback.calibration_valid[i] ? (coordinate_loss_mask & (1u << i) ? 3u :
                  !armed.load() && !axis_enable_mask.load() && (cycle_feedback_mask & (1u << i)) && !cycle_drive_error(i) ? 1u : 3u) : 0u;
          rosie::rtcore::AxisOutputConfig output_config{};
          output_config.limits = axis_profiles[i].limits;
          output_config.counts_per_unit = opt.axis[i].counts_per_unit;
          output_config.sign = opt.axis[i].sign;
          output_config.max_profile_velocity = opt.axis[i].max_profile_vel_counts_per_s;
          output_config.rotary = opt.axis[i].axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY;
          output_config.coordinate_evidence_required = axis_profiles[i].coordinate_evidence.enabled;
          rosie::rtcore::AxisOutputInput output_input{};
          output_input.statusword = sw;
          output_input.feedback_position = pos;
          output_input.home_offset = active_reference_offset(i);
          output_input.feedback_fresh = (cycle_feedback_mask & (1u << i)) != 0;
          output_input.continuous_feedback_valid = coordinate_valid;
          output_input.continuous_feedback = continuous_feedback;
          output_input.configuration_current =
              verified_configuration_epoch.load() == configuration_epoch.load();
          output_input.config_verified = (control_policy.config_verified_mask.load() & (1u << i)) != 0;
          output_input.home_valid = (control_policy.home_valid_mask.load() & (1u << i)) != 0;
          output_input.armed = is_armed && !cycle_drive_error(i);
          output_input.enabled = (en_mask & (1u << i)) != 0;
          output_input.startup_passive = startup_passive_active;
          output_input.service_active = service_mode_active && !cycle_drive_error(i);
          output_input.service_controlword = static_cast<uint16_t>(service_controlword_override[i].load());
          output_input.service_mode = static_cast<int8_t>(service_mode == kNoModeOverride ? commanded_mode : service_mode);
          output_input.commanded_mode = static_cast<int8_t>(commanded_mode);
          output_input.setpoint_present = (sp_mask & (1u << i)) != 0;
          output_input.target_position = target_counts[i];
          output_input.target_velocity = target_velocity_counts_per_s[i];
          output_input.snap_hold = (snap_jog_hold_to_feedback_mask & (1u << i)) != 0;
          output_input.jog_active = active_jog || active_halt;
          output_input.trajectory_active = active_trajectory;
          output_input.brake_motion_permitted = output_input.service_active && output_input.service_mode == 6
              ? release_delay_motion_permitted[i] : brake_outputs[i].motion_permitted;
          output_input.previous_target_valid = have_last_wire_target[i];
          output_input.previous_target = last_wire_target[i];
          output_input.now_ns = wake_ns;
          output_input.previous_cycle_ns = previous_cycle_ns;
          rosie::rtcore::AxisOutputState output_state{};
          output_state.held_target = hold_target_counts[i];
          output_state.held_target_valid = have_hold[i];
          output_state.enable_hold_latched = enable_hold_latched[i];
          output_state.position_rate = position_rate_guard[i];
          output_state.position_rate_active = position_rate_active[i];
          output_state.following_error = following_error_guards[i];
          output_state.quick_stop_cycles_left = jog_stop_quick_stop_cycles_left[i];
          output_state.arrest_cycles_left = jog_stop_arrest_cycles_left[i];
          output_state.fault_reset_cycles_left = fault_reset_left[i];
          const auto output = rosie::rtcore::evaluate_axis_output(output_config, output_input, output_state);
          hold_target_counts[i] = output.next.held_target;
          have_hold[i] = output.next.held_target_valid;
          enable_hold_latched[i] = output.next.enable_hold_latched;
          position_rate_guard[i] = output.next.position_rate;
          position_rate_active[i] = output.next.position_rate_active;
          following_error_guards[i] = output.next.following_error;
          jog_stop_quick_stop_cycles_left[i] = output.next.quick_stop_cycles_left;
          jog_stop_arrest_cycles_left[i] = output.next.arrest_cycles_left;
          fault_reset_left[i] = output.next.fault_reset_cycles_left;
          inhibited_references[i] = output.reference;
          const auto st = output.drive_state;
          uint16_t cw = output.controlword;
          int32_t target_pos_out = output.target_position;
          int32_t target_vel_out = output.target_velocity;
          const int16_t target_torque_out = output.target_torque;
          const int8_t mode_out = output.mode;
          const uint16_t tp_func_out = output.touch_probe_function;
          const uint32_t max_profile_vel_out = output.max_profile_velocity;
          auto& arm_hold = arm_opening_stages[i];
          if (axis_profiles[i].brake.opening.enabled() && !output_input.service_active) {
            const bool current = arm_opening_context(i) && output_input.configuration_current &&
                output_input.home_valid && output_input.config_verified &&
                (!output_config.coordinate_evidence_required || output_input.continuous_feedback_valid);
            const bool stationary = !active_trajectory && !active_jog && !active_halt &&
                !output_input.setpoint_present && !output_input.snap_hold &&
                output.next.enable_hold_latched && target_vel_out == 0;
            const auto phase = brake_guards[i].transition.phase;
            if (!arm_hold.token && current && stationary && !output.fault &&
                phase == rosie::rtcore::BrakeGuardPhase::Held && cw == 15 && mode_out == 8 &&
                next_brake_opening_token != UINT64_MAX) {
              arm_hold.identity = arm_opening_identity();
              arm_hold.token = ++next_brake_opening_token;
              arm_hold.frame_epoch = absolute_frame_epoch[i].load();
              arm_hold.coordinate_epoch = axis_coordinate_epoch[i];
              arm_hold.target = target_pos_out;
            }
            if (arm_hold.token && !arm_hold.completed && !arm_hold.cancelled) {
              if (current && stationary && arm_opening_matches(i) && cw == 15 && mode_out == 8 &&
                  target_pos_out == arm_hold.target && !output.fault) {
                output_brake_opening_token[i] = arm_hold.token;
              } else {
                arm_hold.cancelled = true;
                arm_hold.cancel_cause = !arm_opening_matches(i) || !current ? arm_cancel_cause(i)
                                                                             : rosie::rtcore::ArmCancelCause::TargetChanged;
              }
            }
            // A cancelled opening must submit inhibit even if an Arm bit or a
            // conflicting command is still present. Never re-enable during a
            // closing/quiet acquisition; only a new enable lifecycle may retry.
            if (arm_hold.cancelled || phase == rosie::rtcore::BrakeGuardPhase::Closing ||
                phase == rosie::rtcore::BrakeGuardPhase::Settling ||
                (cw == 15 && !arm_hold.token)) {
              cw = 0;
              target_vel_out = 0;
              output_brake_opening_token[i] = 0;
            }
          }
          // A drive fault is not position loss when its current feedback and
          // coordinate identity remain trustworthy. Other invalidations stand.
          const bool trustworthy_drive_fault = (sw & 8u) && output_input.feedback_fresh &&
              (!output_config.coordinate_evidence_required || coordinate_valid) &&
              output.reference.disposition != rosie::rtcore::ReferenceDisposition::Ambiguous;
          if (output.invalidate_home && !trustworthy_drive_fault)
            control_policy.home_valid_mask.fetch_and(~(1u << i));
          if (output.fault) {
            const uint32_t reasons = output.fault_reasons ? output.fault_reasons :
                (sw & 8u) ? rosie::rtcore::FaultDriveReadiness : rosie::rtcore::FaultTargetLead;
            latch_fault_evidence(reasons, 1u << i);
            execution_fault_reasons.fetch_or(reasons);
            safety_fault_mask.fetch_or(1u << i);
            revoke_outputs();
            is_armed = false;
            active_trajectory = false; active_jog = false; sp_mask = 0;
          }
          // Drive-facing outputs. Optional fields are never dereferenced when absent.
          if (!lease_is_valid_now() || safety_fault_mask.load()) cw = 0;
          {
            rosie::rtcore::DriveOutputObservation observation{};
            observation.now_ns = feedback_observed_ns;
            observation.fresh = output_input.feedback_fresh;
            observation.statusword = sw;
            observation.controlword = cw;
            observation.stationary_command = !output_input.service_active && !active_trajectory && !active_jog &&
                !active_halt && !output_input.setpoint_present && !output_input.snap_hold &&
                !output.next.arrest_cycles_left && !output.next.quick_stop_cycles_left;
            observation.feedback = pos;
            observation.frame_epoch = absolute_frame_epoch[i].load();
            observation.bound_counts = axis_profiles[i].drive_supervision.disarm_motion_counts;
            observation.energized_hold_ns = axis_profiles[i].brake.present ? axis_profiles[i].brake.hold_delay_ns : 0;
            if (drive_supervisors[i].observe_output(observation)) {
              auto evidence = drive_supervisors[i].evidence();
              evidence.arm_request_sequence = arm_request_sequence.load(std::memory_order_acquire);
              drive_supervisor_evidence[i].publish(evidence);
            }
          }
          axis_pd.write_u16(off[i].cw, cw);
          axis_pd.write_s32(off[i].target_pos, target_pos_out);
          if (off[i].target_vel != BusHandles::kInvalidOffset) {
            axis_pd.write_s32(off[i].target_vel, target_vel_out);
          }
          if (off[i].target_torque != BusHandles::kInvalidOffset) {
            axis_pd.write_s16(off[i].target_torque, target_torque_out);
          }
          if (off[i].mode != BusHandles::kInvalidOffset) {
            axis_pd.write_s8(off[i].mode, mode_out);
          }
          if (off[i].tp_func != BusHandles::kInvalidOffset) axis_pd.write_u16(off[i].tp_func, tp_func_out);
          if (off[i].max_profile_vel != BusHandles::kInvalidOffset) {
            axis_pd.write_u32(off[i].max_profile_vel, max_profile_vel_out);
          }

          output_target_counts[i] = target_pos_out;
          output_target_velocity_counts_per_s[i] = target_vel_out;
          if (have_hold[i]) {
            have_hold_mask |= (1u << i);
          }
          if (have_jog_target[i]) {
            have_jog_target_mask |= (1u << i);
          }

          // Publish raw feedback (counts + status) for STATUS_SNAPSHOT.
          latest_feedback.pos_counts[i] = pos;
          latest_feedback.brake_state[i] = brake_outputs[i].state;
          latest_feedback.torque_raw[i] = torque;
          latest_feedback.statusword[i] = sw;
          latest_feedback.error_code[i] = err;
          latest_feedback.manufacturer_error_code[i] = manufacturer_err;
          latest_feedback.mode_display[i] = mode_disp;
          latest_feedback.ds402_state[i] = static_cast<uint8_t>(st);
          // Mapped following error also feeds the collision watchdog. These
          // signals do not certify safe speed. Unverified mappings and stale PDOs cannot
          // supply samples or update peaks, even when offsets were registered.
          const bool diagnostic_mapping_verified =
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              (control_policy.config_verified_mask.load() & (1u << i));
          const bool di_valid = diagnostic_mapping_verified &&
              off[i].di != BusHandles::kInvalidOffset && (cycle_feedback_mask & (1u << i));
          latest_feedback.di_bits[i] = di_valid ? di : 0;
          latest_feedback.di_valid[i] = di_valid;
          const auto& external_enable = axis_profiles[i].external_enable_input;
          // A diagnostic assignment never changes the motion permit. Invalid
          // mapping/freshness also clears active-low observations to unknown.
          const bool external_valid = di_valid && external_enable && external_enable->bit_index < 32;
          latest_feedback.external_enable_valid[i] = external_valid;
          latest_feedback.external_enable_active[i] = external_valid &&
              (((di & (uint32_t{1} << external_enable->bit_index)) != 0) == external_enable->active_high);
          const bool velocity_valid = diagnostic_mapping_verified &&
              off[i].velocity_actual != BusHandles::kInvalidOffset && (cycle_feedback_mask & (1u << i));
          const bool following_valid = diagnostic_mapping_verified &&
              off[i].following_error != BusHandles::kInvalidOffset && (cycle_feedback_mask & (1u << i));
          const int32_t velocity_actual = velocity_valid ? axis_pd.read_s32(off[i].velocity_actual) : 0;
          const int32_t following_error = following_valid ? axis_pd.read_s32(off[i].following_error) : 0;
          latest_feedback.velocity_actual_counts_per_s[i] = velocity_actual;
          latest_feedback.following_error_counts[i] = following_error;
          latest_feedback.velocity_actual_valid[i] = velocity_valid;
          latest_feedback.following_error_valid[i] = following_valid;
          auto& collision = collision_watchdogs[i];
          const double command_distance = std::abs(rosie::rtcore::command_delta(
              have_last_wire_target[i] ? last_wire_target[i] : target_pos_out,
              output_config.coordinate_evidence_required ? continuous_feedback : pos));
          // Saturation is fail-closed above every admitted finite count limit;
          // never convert an out-of-range floating value into signed counts.
          const int64_t commanded_error = !std::isfinite(command_distance) || command_distance >= double(INT64_MAX)
              ? INT64_MAX : static_cast<int64_t>(command_distance);
          const bool collision_fresh = diagnostic_mapping_verified && output_input.feedback_fresh &&
              off[i].torque != BusHandles::kInvalidOffset &&
              (!output_config.coordinate_evidence_required || coordinate_valid);
          if (collision.observe(axis_profiles[i].collision_watchdog,
                  output_input.armed && output_input.enabled && !service_mode_active && !output.fault,
                  collision_fresh, torque, following_error, following_valid, commanded_error,
                  double(output.target_velocity),
                  double(axis_profiles[i].limits.tracking_lag_allowance_ns) / 1e9)) {
            execution_fault(rosie::rtcore::FaultCollisionWatchdog, 1u << i);
            is_armed = false;
          }
          latest_feedback.collision_armed[i] = collision.armed;
          latest_feedback.collision_torque_cycles[i] = collision.torque_cycles;
          latest_feedback.collision_error_cycles[i] = collision.following_error_cycles;
          latest_feedback.collision_peak_torque[i] = collision.peak_torque_raw;
          latest_feedback.collision_peak_error[i] = collision.peak_following_error_counts;
          latest_feedback.collision_trip_quantity[i] = collision.last_trip_quantity;
          latest_feedback.collision_trip_cycles[i] = collision.trip_sustained_cycles;
          latest_feedback.collision_trip_count[i] = collision.trip_count;
          latest_feedback.collision_samples[i] = collision.samples;
          latest_feedback.collision_trip_peak_torque[i] = collision.last_trip_peak_torque_raw;
          latest_feedback.collision_trip_peak_error[i] = collision.last_trip_peak_following_error_counts;
          auto& peaks = drive_diagnostic_peaks[i];
          peaks.observe(velocity_actual, velocity_valid, following_error, following_valid);
          latest_feedback.max_abs_velocity_actual_counts_per_s[i] = peaks.velocity;
          latest_feedback.max_abs_following_error_counts[i] = peaks.following_error;
          // Mirror the prepared 0x607A image for diagnostics. Submission and
          // drive receipt are separate evidence, checked after this snapshot.
          latest_feedback.target_pos_counts[i] = target_pos_out;

          // Phase 1 (2026-04-20): extended A6-EC 0x2040 PDO diagnostics.
          // Each read is guarded on a successful offset registration so
          // drives that did not accept the extended mapping do not silently
          // return zeros in the extended snapshot.
          bool ext_valid_any = false;
          if (off[i].ext_position_error != BusHandles::kInvalidOffset) {
            latest_feedback.ext_position_error_counts[i] =
                axis_pd.read_s32(off[i].ext_position_error);
            ext_valid_any = true;
          }
          if (off[i].ext_multi_turn_lo != BusHandles::kInvalidOffset) {
            latest_feedback.ext_multi_turn_lo[i] =
                axis_pd.read_s32(off[i].ext_multi_turn_lo);
            ext_valid_any = true;
          }
          if (off[i].ext_multi_turn_hi != BusHandles::kInvalidOffset) {
            latest_feedback.ext_multi_turn_hi[i] =
                axis_pd.read_s32(off[i].ext_multi_turn_hi);
            ext_valid_any = true;
          }
          if (off[i].ext_bus_voltage != BusHandles::kInvalidOffset) {
            latest_feedback.ext_bus_voltage_raw[i] =
                axis_pd.read_u16(off[i].ext_bus_voltage);
            ext_valid_any = true;
          }
          if (off[i].ext_load_rate != BusHandles::kInvalidOffset) {
            latest_feedback.ext_load_rate_raw[i] =
                axis_pd.read_u16(off[i].ext_load_rate);
            ext_valid_any = true;
          }
          if (off[i].ext_igbt_temp != BusHandles::kInvalidOffset) {
            latest_feedback.ext_igbt_temp_raw[i] =
                static_cast<int16_t>(axis_pd.read_s16(off[i].ext_igbt_temp));
            ext_valid_any = true;
          }
          if (off[i].ext_motor_temp != BusHandles::kInvalidOffset) {
            latest_feedback.ext_motor_temp_raw[i] =
                static_cast<int16_t>(axis_pd.read_s16(off[i].ext_motor_temp));
            ext_valid_any = true;
          }
          if (off[i].ext_drive_not_ready != BusHandles::kInvalidOffset) {
            latest_feedback.ext_drive_not_ready_bits[i] =
                axis_pd.read_u16(off[i].ext_drive_not_ready);
            ext_valid_any = true;
          }
          if (off[i].ext_motor_not_rotating != BusHandles::kInvalidOffset) {
            latest_feedback.ext_motor_not_rotating_code[i] =
                axis_pd.read_u16(off[i].ext_motor_not_rotating);
            ext_valid_any = true;
          }
          if (ext_valid_any && (cycle_feedback_mask & (1u << i))) {
            const uint32_t prev_mask =
                latest_feedback.ext_valid_axis_mask.load(std::memory_order_relaxed);
            latest_feedback.ext_valid_axis_mask.store(
                prev_mask | (1u << i), std::memory_order_release);
          }
        }

        // DC failures also revoke before the final process-image gate. The
        // reference clock is steered every cycle (CycleBus::sync_clocks).
        if (opt.use_dc) {
          // Discipline the reference on every exchange, just like the slaves.
          // The previous 1000-cycle divider left the host/reference phase free
          // to drift between corrections (a Rosie 1430 arm observed ~8 ms DC difference,
          // October 30). Keep the fresh timestamp and existing half-cycle
          // SYNC0 phase; do not relax position-continuity or freshness checks.
          const uint64_t sync_time_ns = inputs.clock();
          dc_reference_sync_interval_ns = dc_reference_sync_time_ns && sync_time_ns >= dc_reference_sync_time_ns
              ? sync_time_ns - dc_reference_sync_time_ns : 0;
          dc_reference_sync_max_interval_ns = std::max(dc_reference_sync_max_interval_ns,
                                                       dc_reference_sync_interval_ns);
          dc_reference_sync_time_ns = sync_time_ns;
          ++dc_reference_sync_attempts;
          cycle_backend.sync_clocks(sync_time_ns);
          record_bus_failure(cycle_bus);
        }

        // Process at most eight marker-bearing samples. The immutable successor
        // skips marker-free spans without a scan or allocation in the cycle.
        if(active_trajectory && !start_alignment.active && next_io_point<active_traj_point_count) {
          const uint64_t elapsed=diag_now_ns-active_traj_start_ns;
          unsigned delivered=0;
          uint32_t delivered_outputs=0;
          while(next_io_point<active_traj_point_count && active_traj_points[next_io_point].t_from_start_ns<=elapsed) {
            const auto& point=active_traj_points[next_io_point];
            uint32_t reason=0;
            if(delivered++>=8 || elapsed-point.t_from_start_ns>opt.cycle_ns*rosie::rtcore::io_marker_delivery_cycles_unverified)
              reason=rosie::protocol::reason_io_marker_late;
            const uint32_t mask=((point.flags>>gradient::ipc::v1::IO_MARKER_MASK_SHIFT)&255) | (((point.io_high>>0)&255)<<8);
            const uint32_t values=((point.flags>>gradient::ipc::v1::IO_MARKER_VALUE_SHIFT)&255) | (((point.io_high>>8)&255)<<8);
            // Two intents for one output cannot both reach the wire this cycle.
            if(mask&delivered_outputs) reason=rosie::protocol::reason_io_marker_late;
            delivered_outputs|=mask;
            for(unsigned i=0;!reason && i<opt.cell_io.output_count;++i) if(mask&(1u<<i))
              reason=cell_io.intent(i,values&(1u<<i),active_traj_start_ns+point.t_from_start_ns,
                                    rt_cycle_counter.load(),diag_now_ns);
            if(reason) {
              logf("I/O marker refused trajectory=%llu point=%u elapsed_ns=%llu scheduled_ns=%llu cycle_ns=%llu reason=%u",
                   static_cast<unsigned long long>(active_traj_id_rt), next_io_point,
                   static_cast<unsigned long long>(elapsed),
                   static_cast<unsigned long long>(point.t_from_start_ns),
                   static_cast<unsigned long long>(opt.cycle_ns), reason);
              cell_io.fault(reason); active_trajectory=false;
              execution_fault(rosie::rtcore::FaultCellIO,configured_mask); break;
            }
            next_io_point=next_io_point+1<active_traj_point_count ? active_traj_points[next_io_point+1].next_io_index : UINT32_MAX;
          }
        }
        // Faults found after sampling (including output checks and DC sync)
        // cancel before completion can overwrite the latched fault this cycle.
        if (execution_fault_reasons.load() || safety_fault_mask.load()) {
          active_trajectory = false;
          active_jog = false;
          active_jog_axis_mask = 0;
          active_jog_deadline_ns = 0;
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED);
          motion_last_event_code.store(gradient::ipc::v1::EVT_TRAJECTORY_FAULTED);
          motion_queue_depth.store(0);
          motion_done.store(1);
        }
        if (active_trajectory && !start_alignment.active && active_traj_point_count > 0) {
          motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_TRAJECTORY, std::memory_order_relaxed);
          motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_EXECUTING, std::memory_order_relaxed);
          motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          const auto& final_point = active_traj_points[active_traj_point_count - 1];
          const uint64_t elapsed_ns =
              (diag_now_ns >= active_traj_start_ns) ? (diag_now_ns - active_traj_start_ns) : 0;
          const bool final_due = elapsed_ns >= final_point.t_from_start_ns;
          bool any_faulted = false;
          uint32_t first_faulted_axis = std::numeric_limits<uint32_t>::max();
          uint16_t first_faulted_error_code = 0;
          uint8_t first_faulted_ds402_state = 0;
          rosie::rtcore::CompletionInput completion_input{};
          completion_input.elapsed_ns = elapsed_ns;
          completion_input.endpoint_ns = final_point.t_from_start_ns;
          completion_input.axis_mask = final_point.axis_mask;
          completion_input.fresh_ready_mask = cycle_motion_ready_mask;
          uint32_t completion_diag_axis_mask = 0;
          for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
            if ((final_point.axis_mask & (1u << i)) == 0u) {
              continue;
            }
            completion_diag_axis_mask |= (1u << i);
            const uint8_t ds402 = latest_feedback.ds402_state[i];
            if (drive_error_faulted(i, latest_feedback.error_code[i], latest_feedback.manufacturer_error_code[i]) ||
                ds402 == gradient::ipc::v1::DS402_FAULT ||
                ds402 == gradient::ipc::v1::DS402_FAULT_REACTION_ACTIVE) {
              any_faulted = true;
              if (first_faulted_axis == std::numeric_limits<uint32_t>::max()) {
                first_faulted_axis = i;
                first_faulted_error_code = latest_feedback.error_code[i];
                first_faulted_ds402_state = ds402;
              }
            }
            const bool qualified_coordinate = axis_profiles[i].coordinate_evidence.enabled;
            completion_input.timeout_ns[i] = axis_profiles[i].limits.completion_timeout_ns;
            if (qualified_coordinate && !latest_feedback.coordinate_valid[i]) {
              completion_input.fresh_ready_mask &= ~(1u << i);
              completion_diag_axis_mask &= ~(1u << i);
              continue; // Invalid retained int64 counts are not a projected sample.
            }
            auto completion_model = coordinate_model(i, 0);
            const auto projected = completion_model.project_command(final_point.target_counts[i]);
            if (!projected.valid) {
              completion_input.fresh_ready_mask &= ~(1u << i);
              completion_diag_axis_mask &= ~(1u << i);
              continue;
            }
            const int32_t final_target_counts = projected.counts;
            const int32_t final_feedback_counts = qualified_coordinate
                ? static_cast<int32_t>(latest_feedback.coordinate_counts[i])
                : csp_wire_counts_from_feedback(latest_feedback.pos_counts[i]);
            const int64_t error_counts =
                static_cast<int64_t>(final_feedback_counts) -
                static_cast<int64_t>(final_target_counts);
            const int64_t comparable_error_counts = error_counts;
            trajectory_completion_diag_target_counts[i].store(final_target_counts,
                                                              std::memory_order_relaxed);
            trajectory_completion_diag_feedback_counts[i].store(final_feedback_counts,
                                                                std::memory_order_relaxed);
            trajectory_completion_diag_error_counts[i].store(comparable_error_counts,
                                                             std::memory_order_relaxed);
            const double tolerance_counts = std::max(1.0, std::abs(completion_model.logical_delta_to_counts(
                axis_profiles[i].limits.completion_tolerance)));
            if (std::abs(double(comparable_error_counts)) <= tolerance_counts)
              completion_input.in_tolerance_mask |= 1u << i;
          }
          trajectory_completion_diag_traj_id.store(active_traj_id_rt, std::memory_order_relaxed);
          trajectory_completion_diag_sample_time_ns.store(diag_now_ns, std::memory_order_relaxed);
          trajectory_completion_diag_axis_mask.store(completion_diag_axis_mask, std::memory_order_relaxed);
          trajectory_completion_diag_final_due.store(final_due ? 1u : 0u, std::memory_order_relaxed);
          const auto completion = rosie::rtcore::evaluate_completion(completion_input, completion_state);
          completion_state = completion.state;
          if (any_faulted) {
            execution_fault(rosie::rtcore::FaultDriveReadiness, completion_input.axis_mask);
            logf("FAULT_EXEC_E2 diag_now_ns=%llu traj_id=%llu final_due=%u axis_index=%u "
                 "error_code=0x%04X ds402_state=%u",
                 static_cast<unsigned long long>(diag_now_ns),
                 static_cast<unsigned long long>(active_traj_id_rt),
                 final_due ? 1u : 0u,
                 static_cast<unsigned int>(first_faulted_axis),
                 static_cast<unsigned int>(first_faulted_error_code),
                 static_cast<unsigned int>(first_faulted_ds402_state));
            active_trajectory = false;
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_FAULTED, std::memory_order_relaxed);
            motion_last_event_code.store(
                gradient::ipc::v1::EVT_TRAJECTORY_FAULTED, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          } else if (completion.fault_mask) {
            execution_fault(rosie::rtcore::FaultCompletionTimeout, completion.fault_mask);
            active_trajectory = false;
            motion_queue_depth.store(0);
          } else if (completion.complete) {
            active_trajectory = false;
            motion_current_point_index.store(active_traj_point_count - 1, std::memory_order_relaxed);
            motion_queue_depth.store(0, std::memory_order_relaxed);
            motion_exec_state.store(gradient::ipc::v1::EXEC_STATE_COMPLETED, std::memory_order_relaxed);
            motion_last_event_code.store(
                gradient::ipc::v1::EVT_TRAJECTORY_COMPLETED, std::memory_order_relaxed);
            motion_done.store(1, std::memory_order_relaxed);
            motion_last_update_ns.store(diag_now_ns, std::memory_order_relaxed);
          }
        }

        // Re-check the final permit across the entire process image. A fault
        // found on a later axis must also inhibit outputs prepared for earlier
        // axes before this cycle's domain is queued.
        const uint64_t frame_stop_request = stop_request_id.load(std::memory_order_acquire);
        const bool final_permit = !cycle_bus.failed() && cycle_feedback_fresh &&
            lease_is_valid_now() && bus_ready.load() &&
            verified_configuration_epoch.load() == configuration_epoch.load();
        const bool final_armed = armed.load();
        const uint32_t final_service_mask =
            commissioning_state.phase == rosie::rtcore::CommissioningPhase::Active &&
            rosie::rtcore::commissioning_permission(commissioning_state.identity, commissioning_context())
                ? commissioning_state.service_mask : 0;
        // A running drive-alarm reset: the edge goes only to its axes, only
        // while the press's context is intact; any change ends it, with CW0.
        uint32_t alarm_reset_cw_mask = 0;
        if (drive_alarm_attempt.axis_mask) {
          auto& attempt = drive_alarm_attempt;
          const uint64_t alarm_now = inputs.clock();
          const bool intact = !final_armed && !axis_enable_mask.load() && !final_service_mask &&
              !native_home_active_axis_mask.load() && commissioning_phase.load() == 0 &&
              control_policy.generation.load() == attempt.generation &&
              configuration_epoch.load() == attempt.configuration && home_epoch.load() == attempt.home &&
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              frame_stop_request == attempt.stop_request && frame_stop_request == stop_ack_id.load() &&
              !g_stop.load() && !cycle_bus.failed() && cycle_feedback_fresh && bus_ready.load() &&
              (cycle_feedback_mask & attempt.axis_mask) == attempt.axis_mask &&
              control_policy.connection_valid(alarm_now);
          ++attempt.cycles;
          if (!intact) {
            finish_drive_alarm_reset(true, alarm_now);
          } else {
            bool all_out = true;
            for (uint32_t i = 0; i < opt.num_axes; ++i) {
              if (!(attempt.axis_mask & (1u << i))) continue;
              const uint16_t sw = cycle_backend.axis(i).read_u16(off[i].sw);
              if (attempt.pulses[i].step(sw)) alarm_reset_cw_mask |= 1u << i;
              all_out &= attempt.pulses[i].out_of_fault(sw);
            }
            // Out of Fault for the whole pulse window, so a drive that
            // re-faults at once is judged faulted, or out of time.
            if ((all_out && attempt.cycles >= kFaultResetPulseCycles) || alarm_now >= attempt.deadline_ns) {
              alarm_reset_cw_mask = 0;
              finish_drive_alarm_reset(false, alarm_now);
            }
          }
        }
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          if (!final_permit || safety_fault_mask.load() ||
              (!final_armed && !(final_service_mask & (1u << i)))) {
            const auto pd = cycle_backend.axis(i);
            const uint64_t reset_submission_now = inputs.clock();
            const bool startup_reset_only = (startup_reset_cycle_mask & (1u << i)) &&
                startup_reset_generation == control_policy.generation.load() &&
                reset_submission_now < startup_reset_deadline_ns &&
                feedback_observed_ns <= reset_submission_now &&
                reset_submission_now - feedback_observed_ns <= period &&
                absolute_frame_epoch[i].load() == brake_guards[i].anchor_epoch &&
                startup_reset_configuration == configuration_epoch.load() &&
                startup_reset_home == home_epoch.load() &&
                frame_stop_request == stop_ack_id.load() && !g_stop.load() &&
                !final_armed && !axis_enable_mask.load() && !final_service_mask &&
                !native_home_active_axis_mask.load() && commissioning_phase.load() == 0 &&
                !safety_fault_mask.load() && !execution_fault_reasons.load() &&
                !cycle_bus.failed() && cycle_feedback_fresh && bus_ready.load() &&
                control_policy.connection_valid(reset_submission_now) &&
                verified_configuration_epoch.load() == configuration_epoch.load() &&
                (control_policy.config_verified_mask.load() & (1u << i)) &&
                axis_profiles[i].brake.transition.enabled() &&
                brake_guards[i].transition.startup_reset_eligible() &&
                gradient::ds402::decode_statusword(pd.read_u16(off[i].sw)) == gradient::ds402::State::Fault;
            const uint16_t reset_only = startup_reset_only || (alarm_reset_cw_mask & (1u << i))
                ? gradient::ds402::CW_FAULT_RESET :
                final_permit && !final_armed && fault_reset_left[i] > 0
                    ? (pd.read_u16(off[i].cw) & 0x80u) : 0;
            pd.write_u16(off[i].cw, reset_only);
            // Preserve active/stale/ambiguous continuous targets. Once fresh
            // CiA402 feedback proves output inactive, preload only the validated
            // coordinate so a later re-arm cannot snap to an old target.
            const int32_t hold = inhibited_references[i].target;
            pd.write_s32(off[i].target_pos, hold);
            if (off[i].target_vel != BusHandles::kInvalidOffset) pd.write_s32(off[i].target_vel, 0);
            output_target_counts[i] = hold;
            output_target_velocity_counts_per_s[i] = 0;
            latest_feedback.target_pos_counts[i] = hold;
          }
        }

        outputs.targets = output_target_counts;
        outputs.motion_permitted_mask = final_permit && final_armed && !safety_fault_mask.load()
            ? cycle_motion_ready_mask & axis_enable_mask.load() : 0;
        latest_jog_debug.num_axes = opt.num_axes;
        latest_jog_debug.active_jog = active_jog ? 1u : 0u;
        latest_jog_debug.active_jog_axis_mask = active_jog_axis_mask;
        latest_jog_debug.command_sp_mask = sp_mask;
        latest_jog_debug.have_hold_mask = have_hold_mask;
        latest_jog_debug.have_jog_target_mask = have_jog_target_mask;
        latest_jog_debug.snap_hold_mask = snap_jog_hold_to_feedback_mask;
        latest_jog_debug.stop_arrest_mask = stop_arrest_mask;
        latest_jog_debug.latest_cmd_axis_mask = latest_jog_axis_mask;
        latest_jog_debug.latest_cmd_flags = latest_jog_flags;
        latest_jog_debug.last_stop_reason = last_jog_stop_reason;
        latest_jog_debug.last_stop_axis_mask = last_jog_stop_axis_mask;
        latest_jog_debug.sample_time_ns = diag_now_ns;
        latest_jog_debug.active_jog_cmd_seq = active_jog_cmd_seq;
        latest_jog_debug.latest_jog_seq_seen = active_jog_seq_seen;
        latest_jog_debug.active_jog_deadline_ns = active_jog_deadline_ns;
        latest_jog_debug.latest_cmd_timeout_ns = latest_jog_timeout_ns;
        latest_jog_debug.last_stop_time_ns = last_jog_stop_time_ns;
        latest_jog_debug.last_stop_cmd_seq = last_jog_stop_cmd_seq;
        latest_jog_debug.feedback_pos_counts = latest_feedback.pos_counts;
        latest_jog_debug.hold_target_counts = hold_target_counts;
        latest_jog_debug.output_target_counts = output_target_counts;
        latest_jog_debug.output_target_velocity_counts_per_s = output_target_velocity_counts_per_s;
        latest_jog_debug.seq.store(diag_now_ns, std::memory_order_release);

        uint32_t submission_mask = 0;
        if (opt.cell_io.enabled()) {
          cell_io.indicators(diag_now_ns, latest_feedback.statusword, opt.num_axes,
              cycle_feedback_mask, cycle_feedback_fresh && !shutdown_active,
              final_armed && final_permit && !safety_fault_mask.load(), !safety_fault_mask.load());
          const bool manual_io_permit = cell_io.manual() && !armed.load() && !axis_enable_mask.load() &&
              lease_is_valid_now() && application_grant.active && bus_ready.load();
          const auto value = cell_io.output(diag_now_ns, (final_permit || manual_io_permit) && !safety_fault_mask.load() && !shutdown_active,
              control_policy.generation.load(), application_grant.current.identity.app_grant_generation);
          inputs.bus.cell_io_wire.write(opt.cell_io, cycle_backend.axis(0), value);
        }
        if (!startup_skip_domain_queue_active) {
          submission_mask = configured_mask;
          (void)cycle_backend.queue(submission_mask);
        }

        inputs.observe(CyclePhase::Submit);
        const auto submission = cycle_backend.submit(submission_mask);
        const uint64_t submission_observed_ns = inputs.clock();
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          const auto pd = cycle_backend.axis(i);
          native_home_wire_audit[i].observe(submission.covers(1u << i),
              {rt_cycle_counter.load(std::memory_order_relaxed), submission_observed_ns,
               pd.read_u16(off[i].cw), pd.read_u16(off[i].sw),
               off[i].mode == BusHandles::kInvalidOffset ? int8_t{0} : static_cast<int8_t>(pd.read_u8(off[i].mode)),
               pd.read_s32(off[i].target_pos), pd.read_s32(off[i].pos)}, period);
        }
        outputs.submitted_mask = submission.submitted_axis_mask;
        record_bus_failure(cycle_bus);
        rosie::rtcore::record_submitted_targets(submission, output_target_counts,
                                                last_wire_target, have_last_wire_target);
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          auto& command = last_brake_hold_command[i];
          command = {};
          if (!submission.covers(1u << i)) continue;
          const auto pd = cycle_backend.axis(i);
          command.valid = true;
          command.enabled = (pd.read_u16(off[i].cw) & 0x0fu) == 0x0fu;
          command.target = output_target_counts[i];
          command.generation = output_brake_opening_token[i] ? output_brake_opening_token[i] : control_policy.generation.load();
          command.opening_stage = output_brake_opening_token[i] != 0;
          command.submitted_ns = submission_observed_ns;
          command.controlword = pd.read_u16(off[i].cw);
          command.mode = off[i].mode == BusHandles::kInvalidOffset ? int8_t{0} :
              static_cast<int8_t>(pd.read_u8(off[i].mode));
          command.stationary = command.enabled && final_permit && final_armed &&
              (axis_enable_mask.load() & (1u << i)) && !(final_service_mask & (1u << i)) &&
              !active_trajectory && !active_jog && !active_halt &&
              output_target_velocity_counts_per_s[i] == 0 &&
              (cycle_feedback_mask & (1u << i)) &&
              (control_policy.home_valid_mask.load() & (1u << i)) &&
              (control_policy.config_verified_mask.load() & (1u << i)) &&
              verified_configuration_epoch.load() == configuration_epoch.load() &&
              !safety_fault_mask.load();
        }
        if (opt.cell_io.enabled()) {
          if (!submission.covers(configured_mask)) {
            cell_io.observe(0, false, diag_now_ns, rt_cycle_counter.load());
            inputs.bus.cell_io_wire.write(opt.cell_io, cycle_backend.axis(0), 0);
          }
          io_snapshot.publish(cell_io.state());
        }
        // Acknowledge only a ticket captured before the final output gate and
        // after all inhibited process images were queued. This is not drive
        // readback, physical standstill or proof of brake engagement.
        if (frame_stop_request && !final_armed && final_service_mask == 0 &&
            submission.covers(configured_mask))
          stop_ack_id.store(frame_stop_request, std::memory_order_release);

        // Publisher-owned IgH diagnostics never establish exchange freshness.
        // Publish immutable activated handles once; teardown uses the non-RT
        // mutex documented by collect_drive_diagnostics().
        if (!diagnostic_handles_ready.load(std::memory_order_relaxed)) {
          diagnostic_sc = sc;
          diagnostic_handles_ready.store(true, std::memory_order_release);
        }
        (void)cycle_backend.query_state();
        domain_state = cycle_backend.domain_state();
        record_bus_failure(cycle_bus);
        uint32_t online_slaves = 0;
        uint32_t operational_slaves = 0;
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          if (sc[i]) {
            if (sc_state[i].online) {
              online_slaves++;
            }
            if (sc_state[i].online && sc_state[i].operational && sc_state[i].al_state == 8) {
              operational_slaves++;
            }
          }
          latest_feedback.slave_online[i] = sc_state[i].online ? 1 : 0;
          latest_feedback.slave_operational[i] = sc_state[i].operational ? 1 : 0;
          latest_feedback.slave_al_state[i] = static_cast<uint8_t>(sc_state[i].al_state);
        }

        const uint64_t startup_elapsed_ns = startup_elapsed_ns_pre;
        if (startup_elapsed_ns <= startup_detailed_diag_ns) {
          MasterDiagSnapshot cur_master_diag{};
          cur_master_diag.valid = true;
          cur_master_diag.link_up = master_diag_state.link_up != 0;
          cur_master_diag.responding = master_diag_state.slaves_responding;
          cur_master_diag.al_states = master_diag_state.al_states;
          cur_master_diag.domain_wc_state = static_cast<unsigned int>(domain_state.wc_state);
          cur_master_diag.domain_working_counter =
              static_cast<unsigned int>(domain_state.working_counter);
          cur_master_diag.online_slaves = online_slaves;
          cur_master_diag.operational_slaves = operational_slaves;

          if (!prev_master_diag.valid ||
              prev_master_diag.link_up != cur_master_diag.link_up ||
              prev_master_diag.responding != cur_master_diag.responding ||
              prev_master_diag.al_states != cur_master_diag.al_states ||
              prev_master_diag.domain_wc_state != cur_master_diag.domain_wc_state ||
              prev_master_diag.domain_working_counter != cur_master_diag.domain_working_counter ||
              prev_master_diag.online_slaves != cur_master_diag.online_slaves ||
              prev_master_diag.operational_slaves != cur_master_diag.operational_slaves) {
            logf("EtherCAT startup transition cycle=%llu elapsed_ms=%llu passive_outputs=%u "
                 "queue_suppressed=%u link_up=%u responding=%u/%u "
                 "online=%u/%u operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
                 static_cast<unsigned long long>(startup_cycle_counter),
                 static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
                 startup_passive_active ? 1u : 0u,
                 startup_skip_domain_queue_active ? 1u : 0u,
                 cur_master_diag.link_up ? 1u : 0u,
                 cur_master_diag.responding,
                 topology_responding_expected,
                 cur_master_diag.online_slaves,
                 opt.num_axes,
                 cur_master_diag.operational_slaves,
                 opt.num_axes,
                 cur_master_diag.al_states,
                 cur_master_diag.domain_wc_state,
                 cur_master_diag.domain_working_counter,
                 expected_wkc);
            prev_master_diag = cur_master_diag;
          }

          for (uint32_t i = 0; i < opt.num_axes; ++i) {
            if (!sc[i]) {
              continue;
            }
            SlaveDiagSnapshot cur_slave_diag{};
            cur_slave_diag.valid = true;
            cur_slave_diag.online = sc_state[i].online != 0;
            cur_slave_diag.operational = sc_state[i].operational != 0;
            cur_slave_diag.al_state = static_cast<uint8_t>(sc_state[i].al_state);
            if (!prev_slave_diag[i].valid ||
                prev_slave_diag[i].online != cur_slave_diag.online ||
                prev_slave_diag[i].operational != cur_slave_diag.operational ||
                prev_slave_diag[i].al_state != cur_slave_diag.al_state) {
              logf("EtherCAT startup slave cycle=%llu elapsed_ms=%llu axis=%u slave_pos=%u online=%u "
                   "operational=%u al=%s(0x%x)",
                   static_cast<unsigned long long>(startup_cycle_counter),
                   static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   cur_slave_diag.online ? 1u : 0u,
                   cur_slave_diag.operational ? 1u : 0u,
                   al_state_label(cur_slave_diag.al_state),
                   static_cast<unsigned int>(cur_slave_diag.al_state));
              prev_slave_diag[i] = cur_slave_diag;
            }
          }
        }
        const bool startup_process_data_live = domain_state.wc_state == EC_WC_COMPLETE;
        const bool startup_all_operational = operational_slaves >= opt.num_axes;
        if (startup_process_data_live && !startup_process_data_live_logged) {
          startup_process_data_live_logged = true;
          logf("EtherCAT startup process_data_live elapsed_ms=%llu passive_outputs=%u "
               "queue_suppressed=%u responding=%u/%u online=%u/%u operational=%u/%u "
               "master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               topology_responding_expected,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
        }
        // Current complete exchanges and all slaves OP are required before
        // userspace may share mailboxes with IgH's configuration FSM.
        startup_ready = startup_process_data_live && startup_all_operational &&
            online_slaves == opt.num_axes && master_diag_state.link_up &&
            topology_admitted && master_diag_state.slaves_responding == topology_responding_expected &&
            cycle_feedback_fresh && !cycle_bus.failed();

        const bool was_ready = bus_ready.exchange(startup_ready, std::memory_order_acq_rel);
        if (was_ready && !startup_ready) {
          control_policy.home_valid_mask.store(0);
          control_policy.config_verified_mask.store(0);
          configuration_epoch.fetch_add(1);
          revoke_outputs();
        }
        update_mailbox_readiness(startup_ready, wake_ns);
        const bool have_no_topology_progress =
            master_diag_state.slaves_responding == 0 || online_slaves == 0;

        if (!startup_ready && !startup_reset_issued &&
            have_no_topology_progress &&
            startup_elapsed_ns >= kStartupResetDelayNs) {
          const int rc = ecrt_master_reset(master);
          startup_reset_issued = true;
          if (rc == 0) {
            startup_reset_count++;
          }
          logf("EtherCAT startup reset attempt rc=%d elapsed_ms=%llu passive_outputs=%u "
               "queue_suppressed=%u responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
               rc,
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               topology_responding_expected,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
          log_phase_summary("startup_reset", startup_elapsed_ns, startup_cycle_counter);
        }

        if (!startup_ready &&
            (!startup_wait_logged || startup_elapsed_ns - last_startup_log_ns >= kStartupLogIntervalNs)) {
          startup_wait_logged = true;
          last_startup_log_ns = startup_elapsed_ns;
          logf("EtherCAT startup waiting elapsed_ms=%llu passive_outputs=%u queue_suppressed=%u "
               "link_up=%u responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG " slave0_al=%s",
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.link_up ? 1u : 0u,
               master_diag_state.slaves_responding,
               topology_responding_expected,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc,
               al_state_label(static_cast<uint8_t>(sc_state[0].al_state)));
        }

        if (startup_ready && !startup_ready_logged) {
          startup_ready_logged = true;
          logf("EtherCAT startup converged elapsed_ms=%llu passive_outputs=%u queue_suppressed=%u "
               "responding=%u/%u online=%u/%u "
               "operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
               static_cast<unsigned long long>(startup_elapsed_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.slaves_responding,
               topology_responding_expected,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
        }

        if (!startup_ready && !startup_timeout_logged && startup_elapsed_ns >= startup_timeout_ns) {
          startup_timeout_logged = true;
          logf("WARNING: EtherCAT startup did not converge within %llu ms: passive_outputs=%u "
               "queue_suppressed=%u link_up=%u responding=%u/%u "
               "online=%u/%u operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
               static_cast<unsigned long long>(startup_timeout_ns / 1000000ULL),
               startup_passive_active ? 1u : 0u,
               startup_skip_domain_queue_active ? 1u : 0u,
               master_diag_state.link_up ? 1u : 0u,
               master_diag_state.slaves_responding,
               topology_responding_expected,
               online_slaves,
               opt.num_axes,
               operational_slaves,
               opt.num_axes,
               master_diag_state.al_states,
               static_cast<unsigned int>(domain_state.wc_state),
               static_cast<unsigned int>(domain_state.working_counter),
               expected_wkc);
          log_phase_summary("startup_timeout", startup_elapsed_ns, startup_cycle_counter);
          if (master_diag_state.slaves_responding > 0 && !startup_process_data_live) {
            logf("WARNING: EtherCAT link is up and slaves respond, but process-data WKC is zero. "
                 "Likely slave state/PDO configuration mismatch; inspect AL states and live PDO assignments.");
          } else if (startup_process_data_live && !startup_all_operational) {
            logf("WARNING: EtherCAT process-data is alive, but only %u/%u selected slaves are "
                 "operational. Startup is still waiting for full OP.",
                 operational_slaves,
                 opt.num_axes);
          }
        }

        latest_feedback.wkc_actual = static_cast<uint32_t>(domain_state.working_counter);
        latest_feedback.wkc_expected = expected_wkc;
        latest_feedback.master_state =
            master_state_from_al_states(master_diag_state.al_states, master_diag_state.link_up != 0);
        latest_feedback.responding_slaves = master_diag_state.slaves_responding;
        latest_feedback.online_slaves = online_slaves;
        latest_feedback.operational_slaves = operational_slaves;
        // TODO: fill dc_offset_ns from IgH reference clock delta.
        latest_feedback.dc_offset_ns = 0;
        latest_feedback.dc_reference_sync_attempts = dc_reference_sync_attempts;
        latest_feedback.dc_reference_sync_time_ns = dc_reference_sync_time_ns;
        latest_feedback.dc_reference_sync_interval_ns = dc_reference_sync_interval_ns;
        latest_feedback.dc_reference_sync_max_interval_ns = dc_reference_sync_max_interval_ns;
        latest_feedback.link_up = master_diag_state.link_up ? 1 : 0;
        latest_feedback.master_al_states = static_cast<uint8_t>(master_diag_state.al_states);
        latest_feedback.domain_wc_state = static_cast<uint8_t>(domain_state.wc_state);
        latest_feedback.startup_ready = startup_ready ? 1 : 0;
        latest_feedback.startup_passive_active = startup_passive_active ? 1 : 0;
        latest_feedback.startup_skip_domain_queue_active = startup_skip_domain_queue_active ? 1 : 0;
        latest_feedback.startup_elapsed_ms = static_cast<uint32_t>(startup_elapsed_ns / 1000000ULL);
        latest_feedback.startup_reset_count = startup_reset_count;
        // Wakeup jitter relative to the requested absolute schedule.
        latest_feedback.cycle_jitter_ns = jitter_ns;
        // The legacy extended record has one timestamp for the full group.
        // Cached/incomplete input must not renew it. This is host observation
        // time, not a claim about the drive encoder's acquisition time.
        if (cycle_feedback_fresh)
          latest_feedback.ext_sample_time_ns.store(feedback_observed_ns, std::memory_order_release);
        inputs.observe(CyclePhase::Publish);
        latest_feedback.seq.fetch_add(1, std::memory_order_release);
      }
#endif  // GRADIENT_HAVE_ECRT

      if (!jog_evaluated) {
        rosie::rtcore::JogSessionContext unavailable{};
        unavailable.identity = jog_identity(fast_cycle_region ? fast_cycle_region->connection_id() : 0);
        unavailable.now_ns = inputs.clock();
        unavailable.grant_deadline_ns = application_grant.current.deadline_host_ns;
        unavailable.authority_valid = application_grant.active && lease_is_valid_now();
        unavailable.clock_mapping_valid = true;
        if (jog_lane.cycle(fast_cycle_region, unavailable, jog_velocity_limits)) inhibit_outputs();
        jog_session_open.store(jog_lane.state().open, std::memory_order_release);
      }
      bool published_grant_active = false;
      if (fast_cycle_region && fast_cycle_region->connection_id() != failed_connection_id) {
        rosie::protocol::GrantObservation observation{};
        observation.ticket = grant_request_ticket;
        observation.connection_id = fast_cycle_region->connection_id();
        observation.native_generation = control_policy.generation.load(std::memory_order_acquire);
        observation.app_grant_generation = application_grant.current.identity.app_grant_generation;
        observation.capability_id = application_grant.current.identity.capability_id;
        observation.deadline_host_ns = application_grant.current.deadline_host_ns;
        observation.now_host_ns = inputs.clock();
        observation.configuration_epoch = configuration_epoch.load();
        observation.home_epoch = home_epoch.load();
        observation.stop_ack_id = stop_ack_id.load(std::memory_order_acquire);
        observation.grant_reason = grant_request_reason;
        observation.grant_active = application_grant.active &&
            application_grant.current.identity.native_generation == observation.native_generation &&
            lease_is_valid_now();
        published_grant_active = observation.grant_active;
        observation.axis_mask = application_grant.current.axis_mask;
        auto published = fast_cycle_region->publish_grant_observation(observation);
        if (published.status == rosie::rtcore::SharedWordsStatus::Ok)
          published = jog_lane.publish(*fast_cycle_region, jog_identity(fast_cycle_region->connection_id()), inputs.clock());
        if (published.status == rosie::rtcore::SharedWordsStatus::Ok)
          published = jog_lane.publish_path(*fast_cycle_region, jog_identity(fast_cycle_region->connection_id()), inputs.clock());
        if (published.status != rosie::rtcore::SharedWordsStatus::Ok) {
          // A stuck/exhausted observation publisher cannot claim admission.
          // End the connection instead of recovering shared state in place.
          failed_connection_id = observation.connection_id;
          failed_connection_reason = published.status == rosie::rtcore::SharedWordsStatus::Busy
              ? rosie::rtcore::ConnectionFailureReason::Busy
              : (published.status == rosie::rtcore::SharedWordsStatus::SequenceExhausted
                  ? rosie::rtcore::ConnectionFailureReason::SequenceExhausted
                  : rosie::rtcore::ConnectionFailureReason::InvalidPublisher);
          disconnect_outputs();
          published_grant_active = false;
        }
      }
      if (fast_cycle_region && fast_cycle_region->connection_id() == failed_connection_id) {
        // One bounded notification; an occupied slot may be retried without
        // re-fencing. Logging, socket teardown and reclamation belong off RT.
        (void)connection_failure.publish(failed_connection_id, failed_connection_reason);
      }

      if (telemetry.active() || cycle_capture.active()) {
        rosie::rtcore::CycleCaptureRecord record{};
        record.t_ns = cycle_work_begin_ns;
        record.seq = latest_feedback.seq.load(std::memory_order_relaxed);
        record.cycle = rt_cycle_counter.load(std::memory_order_relaxed) + 1;
        record.axes = opt.num_axes;
        record.cycle_jitter_ns = jitter_ns;
        record.active_traj_id = motion_active_traj_id.load(std::memory_order_relaxed);
        record.active_command_seq = motion_active_command_seq.load(std::memory_order_relaxed);
        record.jog_generation = jog_lane.state().generation;
        record.app_grant_generation = application_grant.current.identity.app_grant_generation;
        record.control_generation = control_policy.generation.load(std::memory_order_relaxed);
        record.configuration_epoch = configuration_epoch.load(std::memory_order_relaxed);
        record.home_epoch = home_epoch.load(std::memory_order_relaxed);
        record.grant_deadline_host_ns = application_grant.current.deadline_host_ns;
        record.jog_input_deadline_host_ns = jog_lane.input_deadline_ns();
        record.jog_source_sequence = jog_lane.state().source_sequence;
        record.armed = armed.load(std::memory_order_relaxed);
        record.axis_enable_mask = axis_enable_mask.load(std::memory_order_relaxed);
        record.active_mode = motion_active_mode.load(std::memory_order_relaxed);
        record.state = motion_exec_state.load(std::memory_order_relaxed);
        record.current_point_index = motion_current_point_index.load(std::memory_order_relaxed);
        record.queue_depth = motion_queue_depth.load(std::memory_order_relaxed);
        record.last_event_code = motion_last_event_code.load(std::memory_order_relaxed);
        record.underrun_count = motion_underrun_count.load(std::memory_order_relaxed);
        record.stale_command_flag = motion_stale_command_flag.load(std::memory_order_relaxed);
        record.motion_done = motion_done.load(std::memory_order_relaxed);
        record.capability_flags = motion_capability_flags.load(std::memory_order_relaxed);
        record.jog_session_open = jog_lane.input_open();
        record.jog_has_input = jog_lane.state().has_input;
        record.jog_axis_mask = jog_lane.state().axis_mask;
        record.jog_ramping = jog_lane.ramping();
        // Capture the published authority decision, including its generation
        // and lease checks; retained grant identity alone grants no permission.
        record.grant_active = published_grant_active;
        record.grant_axis_mask = application_grant.current.axis_mask;
        record.safety_fault_mask = safety_fault_mask.load(std::memory_order_relaxed);
        record.execution_fault_reasons = execution_fault_reasons.load(std::memory_order_relaxed);
        record.submitted_axis_mask = outputs.submitted_mask;
        record.pdo_fresh_axis_mask = latest_feedback.pdo_fresh_axis_mask;
        record.config_verified_mask = control_policy.config_verified_mask.load(std::memory_order_relaxed);
        record.home_valid_mask = control_policy.home_valid_mask.load(std::memory_order_relaxed);
        record.service_mode_axis_mask = service_mode_axis_mask.load(std::memory_order_relaxed);
        record.wkc_actual = latest_feedback.wkc_actual;
        record.wkc_expected = latest_feedback.wkc_expected;
        record.master_state = latest_feedback.master_state;
        record.io_configured = cell_io.configured();
        record.io_input_word = cell_io.input_word();
        record.io_output_word = cell_io.output_word();
        record.io_input_valid = cell_io.input_valid();
        record.io_armed = cell_io.state().armed;
        record.io_reason = cell_io.state().reason;
#if GRADIENT_HAVE_ECRT
        // Bus results already hold the signed Send result; None is not attempted.
        if (bus.ecrt_ok) {
          record.bus_submission_operation = static_cast<uint32_t>(cycle_backend.results().send.operation);
          record.bus_submission_return_code = cycle_backend.results().send.return_code;
        } else {
          record.pdo_fresh_axis_mask = 0;
          record.io_input_valid = 0;
        }
#endif
        static_assert(kMaxAbsoluteFeedbackFields == rosie::rtcore::kCaptureAbsoluteFields);
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          auto& axis = record.ax[i];
          axis.position = latest_feedback.pos_counts[i];
          axis.target = latest_feedback.target_pos_counts[i];
          axis.statusword = latest_feedback.statusword[i];
          axis.error_code = latest_feedback.error_code[i];
          axis.manufacturer_error_code = latest_feedback.manufacturer_error_code[i];
          axis.velocity_actual_counts_per_s = latest_feedback.velocity_actual_counts_per_s[i];
          axis.following_error_counts = latest_feedback.following_error_counts[i];
          axis.velocity_actual_valid = latest_feedback.velocity_actual_valid[i];
          axis.following_error_valid = latest_feedback.following_error_valid[i];
          axis.di_bits = latest_feedback.di_bits[i];
          axis.di_valid = latest_feedback.di_valid[i];
          axis.external_enable_active = latest_feedback.external_enable_active[i];
          axis.external_enable_valid = latest_feedback.external_enable_valid[i];
          axis.coordinate_counts = latest_feedback.coordinate_counts[i];
          axis.coordinate_epoch = latest_feedback.coordinate_epoch[i];
          axis.pdo_observed_time_ns = latest_feedback.pdo_observed_time_ns[i];
          axis.collision_peak_following_error_counts = latest_feedback.collision_peak_error[i];
          axis.collision_trip_count = latest_feedback.collision_trip_count[i];
          axis.collision_samples = latest_feedback.collision_samples[i];
          axis.collision_last_trip_peak_following_error_counts = latest_feedback.collision_trip_peak_error[i];
          axis.absolute_source_counts = latest_feedback.absolute_source_counts[i];
          axis.coordinate_reason = latest_feedback.coordinate_reason[i];
          axis.collision_armed = latest_feedback.collision_armed[i];
          axis.collision_torque_cycles = latest_feedback.collision_torque_cycles[i];
          axis.collision_following_error_cycles = latest_feedback.collision_error_cycles[i];
          axis.collision_peak_torque_raw = latest_feedback.collision_peak_torque[i];
          axis.collision_last_trip_quantity = latest_feedback.collision_trip_quantity[i];
          axis.collision_trip_sustained_cycles = latest_feedback.collision_trip_cycles[i];
          axis.collision_last_trip_peak_torque_raw = latest_feedback.collision_trip_peak_torque[i];
          axis.brake_state = latest_feedback.brake_state[i];
          axis.native_home_position_offset = latest_feedback.native_home_position_offset[i];
          axis.native_home_last_abort_code = latest_feedback.native_home_last_abort_code[i];
          axis.ext_position_error_counts = latest_feedback.ext_position_error_counts[i];
          axis.ext_multi_turn_lo = latest_feedback.ext_multi_turn_lo[i];
          axis.ext_multi_turn_hi = latest_feedback.ext_multi_turn_hi[i];
          axis.max_abs_velocity_actual_counts_per_s = latest_feedback.max_abs_velocity_actual_counts_per_s[i];
          axis.max_abs_following_error_counts = latest_feedback.max_abs_following_error_counts[i];
          axis.torque_raw = latest_feedback.torque_raw[i];
          axis.ext_bus_voltage_raw = latest_feedback.ext_bus_voltage_raw[i];
          axis.ext_load_rate_raw = latest_feedback.ext_load_rate_raw[i];
          axis.ext_igbt_temp_raw = latest_feedback.ext_igbt_temp_raw[i];
          axis.ext_motor_temp_raw = latest_feedback.ext_motor_temp_raw[i];
          axis.ext_drive_not_ready_bits = latest_feedback.ext_drive_not_ready_bits[i];
          axis.ext_motor_not_rotating_code = latest_feedback.ext_motor_not_rotating_code[i];
          axis.mode_display = static_cast<int8_t>(static_cast<uint8_t>(latest_feedback.mode_display[i]));
          axis.ds402_state = latest_feedback.ds402_state[i];
          axis.coordinate_valid = latest_feedback.coordinate_valid[i];
          axis.coordinate_source_valid = latest_feedback.coordinate_source_valid[i];
          axis.calibration_valid = latest_feedback.calibration_valid[i];
          axis.position_state = latest_feedback.position_state[i];
          const auto& audit = sampled_bindings[i].metrics;
          axis.audit_reason = static_cast<uint16_t>(audit.reason);
          axis.audit_last_verified_ns = audit.last_verified_ns;
          axis.audit_started_ns = audit.started_ns;
          axis.audit_completed_ns = audit.completed_ns;
          axis.audit_coherence_error_counts = audit.coherence_error_counts;
          axis.native_home_state = latest_feedback.native_home_state[i];
          axis.slave_al_state = latest_feedback.slave_al_state[i];
          axis.slave_online = latest_feedback.slave_online[i];
          axis.slave_operational = latest_feedback.slave_operational[i];
          axis.pdo_fresh = (record.pdo_fresh_axis_mask & (1u << i)) != 0;
          axis.target_velocity_counts_per_s = latest_jog_debug.output_target_velocity_counts_per_s[i];
#if GRADIENT_HAVE_ECRT
          const bool verified = verified_configuration_epoch.load(std::memory_order_relaxed) == record.configuration_epoch &&
              (record.config_verified_mask & (1u << i));
          axis.torque_valid = axis.pdo_fresh && verified && off[i].torque != BusHandles::kInvalidOffset;
          axis.target_velocity_valid = bus.ecrt_ok && record.bus_submission_operation != 0 && off[i].target_vel != BusHandles::kInvalidOffset;
          // Measured mask order is the nine extended diagnostic fields below.
          // Each bit requires both a verified mapping and this cycle's fresh PDO.
          if (axis.pdo_fresh && verified) {
            if (off[i].ext_position_error != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 0;
            if (off[i].ext_multi_turn_lo != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 1;
            if (off[i].ext_multi_turn_hi != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 2;
            if (off[i].ext_bus_voltage != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 3;
            if (off[i].ext_load_rate != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 4;
            if (off[i].ext_igbt_temp != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 5;
            if (off[i].ext_motor_temp != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 6;
            if (off[i].ext_drive_not_ready != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 7;
            if (off[i].ext_motor_not_rotating != BusHandles::kInvalidOffset) axis.ext_valid_mask |= 1u << 8;
          }
#endif
          axis.ext_valid = axis.ext_valid_mask != 0;
          if (!axis.torque_valid) axis.torque_raw = 0;
          for (uint32_t f = 0; f < axis_profiles[i].absolute.field_count; ++f) {
            axis.absolute_value[f] = latest_feedback.absolute_feedback[i].fields[f].value;
            axis.absolute_valid[f] = latest_feedback.absolute_feedback[i].fields[f].valid;
          }
        }
        if (!telemetry.push(record, cycle_work_begin_ns, inputs.clock))
          record.work_ns = inputs.clock() - cycle_work_begin_ns;
        // Commissioning retains its historical even feedback publication sequence.
        record.seq = latest_feedback.seq.load(std::memory_order_relaxed);
        (void)cycle_capture.push(record);
      }
      // Consume pending provenance on every path, including disabled capture;
      // an inactive recorder must never replay stale faults when later enabled.
      auto emit_trigger = [&](uint32_t kind, uint32_t fault, uint32_t axis) {
        if (!telemetry.active()) return;
        rosie::rtcore::TelemetryTrigger trigger{};
        trigger.end = telemetry.written(); trigger.reason = kind;
        trigger.fault_reason = fault; trigger.axis = axis;
        if (!telemetry_cycle_triggers.push(trigger)) telemetry_trigger_drops.fetch_add(1);
      };
      for (size_t bit = 0; bit < telemetry_pending_faults.size(); ++bit) {
        if (!telemetry_pending_faults[bit]) continue;
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
          if (telemetry_pending_faults[bit] & (1u << axis)) emit_trigger(1, 1u << bit, axis);
        telemetry_pending_faults[bit] = 0;
      }
      if (telemetry_pending & 2) emit_trigger(2, 0, UINT32_MAX);
      telemetry_pending = 0;
      const uint64_t complete_work_ns = inputs.clock() - cycle_work_begin_ns;
      if (complete_work_ns > rt_max_cycle_work_ns.load(std::memory_order_relaxed))
        rt_max_cycle_work_ns.store(complete_work_ns, std::memory_order_relaxed);

      // Inhibition is immediate, but an in-flight synchronous IgH SDO needs
      // continuing PDO/FSM exchanges to finish. Never stop those exchanges and
      // then wait on the mutex held by that same SDO in master teardown.
      // A stop that arrives after this cycle's inhibit point (shutdown_active
      // still unset) is inhibited and drained from the next cycle. Ending the
      // loop here on such a stop is what stranded the servo host's upload on
      // 2026-10-07 (and AMR-715): no drain ran at all. Only a loop with no
      // cyclic backend, where no mailbox call is ever admitted, ends at once.
#if GRADIENT_HAVE_ECRT
      const bool cyclic_backend = ecrt_ok;
#else
      const bool cyclic_backend = false;
#endif
      if (g_stop.load(std::memory_order_relaxed) && !shutdown_active && !cyclic_backend) return {false, wake_ns};
      if (g_stop.load(std::memory_order_relaxed) && shutdown_active) {
        const bool worker_exited = commissioning_exited.load(std::memory_order_acquire);
        const uint32_t in_flight = mailbox_calls_in_flight.load(std::memory_order_acquire);
        // Every blocking mailbox call runs under shared_master_sdo_mutex. Owning
        // it while still exchanging proves none is in flight and admits none
        // later. try_lock only after the worker exited: a live worker blocked
        // on this lock could never exit. Never a blocking lock on this thread.
#if GRADIENT_HAVE_ECRT
        if (worker_exited && !in_flight && wake_ns >= shutdown_until_ns && !shutdown_mailbox.owns_lock())
          (void)shutdown_mailbox.try_lock();
        const bool mailbox_owned = shutdown_mailbox.owns_lock();
#else
        const bool mailbox_owned = !in_flight;
#endif
        const auto drain = rosie::rtcore::decide_shutdown_drain(wake_ns, shutdown_until_ns,
            worker_exited, mailbox_owned, shutdown_drain_failure_reported);
        if (drain.report_failure) {
          shutdown_drain_failure_reported = true;
          process_exit_code.store(2, std::memory_order_relaxed);
          logf("ERROR: shutdown mailbox drain exceeded 5000 ms worker_exited=%u mailbox_in_flight=%u; "
               "outputs remain inhibited, continuing cyclic exchange; kernel request cancellation is not guaranteed",
               worker_exited ? 1u : 0u, in_flight);
        }
        if (drain.release_bus) {
          logf("shutdown drain complete elapsed_ms=%llu; cyclic exchange ends holding the mailbox lock",
               static_cast<unsigned long long>((wake_ns - shutdown_started_ns) / 1000000ULL));
          return {false, wake_ns};
        }
      }

          motion_snapshot.publish(motion_status(inputs.clock()));
          observe_fault_trust();
          publish_recovery();
          rt_results_committed.store(rt_result_count, std::memory_order_release);

      rt_cycle_counter.fetch_add(1, std::memory_order_relaxed);
      // No usable bus exchange must still produce a refused pending receipt.
      complete_halt_requests(0);

#if GRADIENT_HAVE_ECRT
    if (active_halt && motion_done.load()) {
      active_halt = false; halt_axis_mask = 0;
      motion_active_mode.store(gradient::ipc::v1::MOTION_MODE_IDLE);
    }
    halt_admitted_this_cycle = false;
#endif
    outputs.wake_ns = wake_ns;
    return outputs;
  }

  void run_cycle() {
    cycle_owner = this;
    struct CycleExit {
      MachineState& machine;
      ~CycleExit() { machine.cycle_exited.store(true, std::memory_order_release); }
    } cycle_exit{*this};
    pthread_setname_np(pthread_self(), "rt-cycle");
#if defined(ROSIE_RT_SIMULATION) && !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
    // Simulation measures locking where permitted; denied hosts still prefault
    // plainly. Live run() already fails closed if mlockall is refused.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      logf("Simulation plan prefault: mlockall refused errno=%d", errno);
#endif
    // Live run() has locked memory; finish writable plan residency before bus startup.
    trajectory_store.prefault();
    rosie::rtcore::prefault_trajectory_sampling();
    struct StopFastReader {
      rosie::rtcore::RtMappingHandoff<rosie::rtcore::FastControlRegion>& handoff;
      ~StopFastReader() { handoff.stop_reader(); }
    } stop_fast_reader{fast_handoff};
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(effective_rt_cpu, &cpuset);
    const int affinity_error = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    rt_affinity_applied.store(affinity_error == 0, std::memory_order_release);
#if GRADIENT_ECRT_ENABLED && !defined(ROSIE_RT_SIMULATION)
    // Live admission requires observed host prerequisites, before master access.
    // PREEMPT_RT is proven by /sys/kernel/realtime == 1 where the kernel exposes
    // it, or by the PREEMPT_RT token in the kernel version string: the 6.12
    // PREEMPT_RT kernel on the nine-motor bench (2026-09-13) has no
    // /sys/kernel/realtime file. Same rule as rtctl hostcheck's preempt_rt check.
    std::ifstream realtime_flag("/sys/kernel/realtime");
    int kernel_realtime = 0; realtime_flag >> kernel_realtime;
    if (kernel_realtime != 1) {
      utsname kernel{};
      if (uname(&kernel) == 0 && std::strstr(kernel.version, "PREEMPT_RT") != nullptr) kernel_realtime = 1;
    }
    if (kernel_realtime != 1 || opt.rt_cpu >= CPU_SETSIZE || opt.rt_priority < 1 || opt.rt_priority > 99) {
      logf("ERROR: live runtime requires PREEMPT_RT and explicit CPU/FIFO priority");
      process_exit_code.store(2); g_stop.store(true); return;
    }
    sched_param sp{}; sp.sched_priority = static_cast<int>(opt.rt_priority);
    const int scheduling_error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (affinity_error || scheduling_error) {
      logf("ERROR: RT affinity/FIFO setup failed: affinity=%d scheduling=%d", affinity_error, scheduling_error);
      process_exit_code.store(2); g_stop.store(true); return;
    }
    volatile char stack_reserve[65536];
    for (size_t page = 0; page < sizeof(stack_reserve); page += 4096) stack_reserve[page] = 0;
#else
    if (affinity_error) logf("WARNING: RT CPU %u affinity failed: %s", effective_rt_cpu, std::strerror(affinity_error));
#endif

    for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
      jog_velocity_limits[axis] = axis_profiles[axis].limits.max_qd;
#if GRADIENT_HAVE_ECRT
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      // Admission bounded smoothing_cycles to JogSmoother::max_window.
      const double cycle_s = double(period) / 1e9;
      (void)jog_smoothers[axis].configure(axis_profiles[axis].jog.smoothing_cycles, cycle_s);
      (void)halt_smoothers[axis].configure(axis_profiles[axis].jog.smoothing_cycles, cycle_s);
      jog_axes[axis].smoother = &jog_smoothers[axis];
      jog_lane.configure_axis(axis, jog_axes[axis], axis_profiles[axis].jog,
          std::abs(coordinate_model(axis, 0).logical_delta_to_counts(axis_profiles[axis].limits.jog_acceleration)), period);
    }
    axis_coordinate_epoch.fill(1);
    // Keep commissioning mailbox requests out of the PREOP assignment/proof
    // transaction; the same mutex serializes restoration and master teardown.
    auto bus = [&] {
      std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
      return bring_up_bus(opt, parsed, shared_master, latest_feedback, process_exit_code);
    }();
    if (bus.stop_cycle) return;
    topology_monitor = bus.topology;
    topology_snapshot.publish(topology_monitor.snapshot);
#endif  // GRADIENT_HAVE_ECRT

    // Initialization above can take seconds (EtherCAT config, activation, etc.).
    // Rebase the absolute sleep schedule here so the cyclic loop does not try to
    // "catch up" by blasting thousands of immediate iterations after startup.
    next_ns = now_monotonic_ns();
    previous_cycle_ns = next_ns;

    rosie::rtcore::cyclic_diagnostics = true;
    while (true) {
      rosie::rtcore::RtMappingHandoff<rosie::rtcore::FastControlRegion>::Cycle fast_cycle(fast_handoff);
#if GRADIENT_HAVE_ECRT
      const auto cycle = step(CycleInputs<BusHandles>{bus, fast_cycle.get()});
#else
      const auto cycle = step(CycleInputs{fast_cycle.get()});
#endif
      if (!cycle.continue_running) break;
      timespec ts{};
      ts.tv_sec = static_cast<time_t>(next_ns / 1000000000ULL);
      ts.tv_nsec = static_cast<long>(next_ns % 1000000000ULL);
      const uint64_t end_ns = now_monotonic_ns();
      if (end_ns > next_ns && end_ns - next_ns > opt.max_cycle_lateness_ns &&
          (armed.load() || service_mode_axis_mask.load()))
        execution_fault(rosie::rtcore::FaultCycleDeadline, configured_mask);
      next_ns = rosie::rtcore::future_deadline(next_ns, end_ns, period);
      ts.tv_sec = static_cast<time_t>(next_ns / 1000000000ULL);
      ts.tv_nsec = static_cast<long>(next_ns % 1000000000ULL);
      int sleep_result;
      do { sleep_result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr); }
      while (sleep_result == EINTR);
      if (sleep_result != 0) {
        execution_fault(rosie::rtcore::FaultCycleSleep, configured_mask);
        process_exit_code.store(1);
        g_stop.store(true);
      }
      previous_cycle_ns = cycle.wake_ns;
    }

#if GRADIENT_HAVE_ECRT
    // Best-effort cleanup. This is not RT-critical (we're shutting down), and it
    // allows IgH to deactivate the master configuration cleanly. A drained stop
    // arrives here already holding the mailbox lock. Only a loop that never had
    // a cyclic backend (shutdown_active unset) can arrive without it; no mailbox
    // call is admitted without that exchange, so blocking here cannot wait on one.
    if (master) {
      if (!shutdown_mailbox.owns_lock()) {
        if (const auto in_flight = mailbox_calls_in_flight.load(std::memory_order_acquire))
          logf("ERROR: cyclic exchange ended without the mailbox lock with %u mailbox call(s) in flight", in_flight);
        shutdown_mailbox.lock();
      }
      if (master) {
        bus.restore_assignments();
        ecrt_release_master(master); // deactivates if active
        master = nullptr;
      }
    }
    if (shutdown_mailbox.owns_lock()) shutdown_mailbox.unlock();
#endif
  }

};

} // namespace
