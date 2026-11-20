#pragma once

// Drive health for operators (docs/decisions/0001): drive-profile monitor
// objects read by the commissioning worker, and each axis's recent torque and
// following error summarised by the metrics thread from the telemetry ring.
// Both are diagnostics: nothing here gates motion, and nothing runs on the RT
// thread. They replace a second process that read the drives over the mailbox
// and decoded the ring in Python on the servo host.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace rosie::rtcore {

// Capacity shared with the compiler (tools/rtctl/render.go maxMonitorEntries).
constexpr size_t kMaxMonitorPerAxis = 8;

// One monitor object's latest reading. The worker is the only writer; the
// metrics thread reads. A reader accepts a value only when the same nonzero
// stamp brackets the copy, so it never pairs a value with another read's time.
struct MonitorSlot {
  std::atomic<uint64_t> read_ns{0};
  std::atomic<int32_t> raw{0};
  uint64_t next_due_ns = 0;  // Worker only.

  void publish(int32_t value, uint64_t now_ns) {
    read_ns.store(0);
    raw.store(value);
    read_ns.store(now_ns == 0 ? 1 : now_ns);
  }
  // False when never read or a read is being published.
  bool snapshot(int32_t* value, uint64_t* at_ns) const {
    const auto before = read_ns.load();
    const auto v = raw.load();
    if (!before || read_ns.load() != before) return false;
    *value = v;
    *at_ns = before;
    return true;
  }
};

struct DriveMonitorState {
  std::array<MonitorSlot, kMaxMonitorPerAxis> slots{};
};

// raw / 10^decimals, the profile's display value.
inline double monitor_value(int32_t raw, uint32_t decimals) {
  return raw / std::pow(10.0, static_cast<double>(decimals));
}

// Torque (per-mille of rated torque) and following error (counts) over the
// last kBins x kBinNs, from cycle records. Metrics thread only. Bins are
// keyed by record time, so a gap in the ring shows as fewer samples, never as
// another period's values.
struct DriveLoadWindow {
  // Unverified display choice: 1 s in ten 100 ms bins, one per metrics pass.
  static constexpr uint64_t kBinNs = 100'000'000;
  static constexpr size_t kBins = 10;
  struct Bin {
    uint64_t id = 0;  // record time / kBinNs + 1; 0 is empty
    uint32_t torque_samples = 0, error_samples = 0;
    double torque_sum_squares = 0;
    uint32_t torque_peak = 0;
    uint64_t error_peak = 0;
  };
  struct Summary {
    uint64_t window_ns = kBins * kBinNs;
    uint32_t torque_samples = 0, error_samples = 0;
    double torque_rms = 0;
    uint32_t torque_peak = 0;
    uint64_t error_peak = 0;
  };
  std::array<Bin, kBins> bins{};
  uint64_t newest = 0;

  void add(uint64_t t_ns, bool torque_valid, int32_t torque_raw, bool error_valid, int64_t error_counts) {
    const uint64_t id = t_ns / kBinNs + 1;
    Bin& bin = bins[id % kBins];
    if (bin.id != id) bin = Bin{id};
    newest = std::max(newest, id);
    if (torque_valid) {
      const auto magnitude = static_cast<uint32_t>(std::abs(static_cast<int64_t>(torque_raw)));
      ++bin.torque_samples;
      bin.torque_sum_squares += static_cast<double>(magnitude) * magnitude;
      bin.torque_peak = std::max(bin.torque_peak, magnitude);
    }
    if (error_valid) {
      ++bin.error_samples;
      const auto magnitude = static_cast<uint64_t>(error_counts < 0 ? -error_counts : error_counts);
      bin.error_peak = std::max(bin.error_peak, magnitude);
    }
  }
  void clear() { *this = DriveLoadWindow{}; }
  // The bins of the last window ending at the newest record.
  Summary summary() const {
    Summary s{};
    double sum_squares = 0;
    for (const auto& bin : bins) {
      if (!bin.id || bin.id + kBins <= newest) continue;
      s.torque_samples += bin.torque_samples;
      s.error_samples += bin.error_samples;
      sum_squares += bin.torque_sum_squares;
      s.torque_peak = std::max(s.torque_peak, bin.torque_peak);
      s.error_peak = std::max(s.error_peak, bin.error_peak);
    }
    if (s.torque_samples) s.torque_rms = std::sqrt(sum_squares / s.torque_samples);
    return s;
  }
};

}  // namespace rosie::rtcore
