#pragma once
#include "publication.hpp"
#include <cstdio>
#include <algorithm>
#include <type_traits>
#include <utility>

namespace rosie::rtcore {
// Accumulate only fresh mapped drive samples after Arm, widening before absolute
// value so INT32_MIN cannot overflow. These observations never govern motion.
struct DriveDiagnosticPeaks {
  uint32_t velocity = 0, following_error = 0;
  bool started = false;
  void arm() noexcept { velocity = following_error = 0; started = true; }
  void observe(int32_t speed, bool speed_valid, int32_t error, bool error_valid) noexcept {
    if (!started) return;
    const auto magnitude = [](int32_t v) { return static_cast<uint32_t>(v < 0 ? -int64_t(v) : int64_t(v)); };
    if (speed_valid) velocity = std::max(velocity, magnitude(speed));
    if (error_valid) following_error = std::max(following_error, magnitude(error));
  }
};

// Bounded diagnostic records: the cyclic thread copies scalar arguments and
// short strings; the non-RT worker performs formatting and file I/O.
struct DiagnosticArgument {
  uint64_t integer = 0;
  double real = 0;
  char text[96]{};
  template<class T> void set(T value) noexcept {
    if constexpr (std::is_convertible_v<T, const char*>) {
      const char* source = value;
      if (source) for (size_t i = 0; i + 1 < sizeof(text) && source[i]; ++i) text[i] = source[i];
    } else if constexpr (std::is_floating_point_v<T>) real = value;
    else integer = static_cast<uint64_t>(value);
  }
  template<class T> auto get() const noexcept {
    if constexpr (std::is_convertible_v<T, const char*>) return text;
    else if constexpr (std::is_floating_point_v<T>) return static_cast<T>(real);
    else return static_cast<T>(integer);
  }
};
struct DiagnosticRecord {
  const char* format = nullptr; // Call sites use static format literals.
  std::array<DiagnosticArgument, 24> arguments{};
  void (*write)(const DiagnosticRecord&) = nullptr;
};
// Measured source bound: step has 12 logf -> defer_diagnostic -> push sites:
// 11 scalar sites plus one slave-transition site repeated at most 16 times.
// Treat mutually exclusive/one-shot branches as simultaneous: 27 records/cycle.
// The phase summaries run in the publisher and do not enqueue cyclic records.
inline constexpr size_t kDiagnosticsRecordsPerCycle = 11 + 16;
// Unverified scheduler service bound: one 10 Hz publisher period is 100000000 ns;
// the fastest admitted cycle is 250000 ns, hence 400 * 27 = 10800 records.
// A stalled publisher can still overflow; diagnostics_dropped retains exact loss.
inline constexpr uint64_t kDiagnosticsPublisherPeriodNs = 100'000'000;
inline constexpr size_t kDiagnosticsCapacity =
    kDiagnosticsRecordsPerCycle * (kDiagnosticsPublisherPeriodNs / 250'000);
inline SpscQueue<DiagnosticRecord, kDiagnosticsCapacity> diagnostics;
inline std::atomic<uint64_t> diagnostics_dropped{0};
inline std::atomic<uint64_t> diagnostics_produced{0};
inline std::atomic<uint64_t> diagnostics_consumed{0};
inline thread_local bool cyclic_diagnostics = false;

template<class... T, size_t... I>
void write_diagnostic(const DiagnosticRecord& record, std::index_sequence<I...>) {
  std::fputs("[rosie-rt-core] ", stderr);
  if constexpr (sizeof...(T)) std::fprintf(stderr, record.format, record.arguments[I].template get<T>()...);
  else std::fputs(record.format, stderr);
  std::fputc('\n', stderr);
}
template<class... T> void defer_diagnostic(const char* format, T... values) noexcept {
  static_assert(sizeof...(T) <= 24);
  DiagnosticRecord record;
  record.format = format;
  size_t index = 0;
  (record.arguments[index++].set(values), ...);
  record.write = [](const DiagnosticRecord& value) { write_diagnostic<T...>(value, std::index_sequence_for<T...>{}); };
  diagnostics_produced.fetch_add(1, std::memory_order_relaxed);
  if (!diagnostics.push(record)) diagnostics_dropped.fetch_add(1, std::memory_order_relaxed);
}
inline void drain_diagnostics() {
  DiagnosticRecord record;
  for (size_t n = 0; n < kDiagnosticsCapacity && diagnostics.pop(record); ++n) {
    record.write(record);
    diagnostics_consumed.fetch_add(1, std::memory_order_relaxed);
  }
}
}
