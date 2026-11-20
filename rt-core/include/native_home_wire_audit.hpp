#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

struct NativeHomeWireSample {
  uint64_t cycle = 0, time_ns = 0;
  uint16_t controlword = 0, statusword = 0;
  int8_t mode = 0;
  int32_t target = 0, position = 0;
};

struct NativeHomeWirePrefixBucket {
  uint64_t submitted = 0, mode6 = 0, mode8 = 0, other_mode = 0;
  NativeHomeWireSample first{}, last{};
};

struct NativeHomeWirePrefix {
  uint64_t cycles = 0, submitted = 0, submission_failures = 0;
  uint64_t missing_cycles = 0, nonmonotonic_cycles = 0, time_gaps = 0, maximum_gap_ns = 0;
  // Fixed storage: CW6, CW7, CW15, and every other submitted controlword.
  NativeHomeWirePrefixBucket buckets[4]{};
};

struct NativeHomeWireSummary {
  uint64_t token = 0, requested_ns = 0, ended_ns = 0, stopped_cycle = 0;
  uint64_t cycles = 0, submitted = 0, submission_failures = 0;
  uint64_t prefix_cycles = 0, triggered_submissions = 0, violations = 0;
  uint64_t missing_cycles = 0, nonmonotonic_cycles = 0, time_gaps = 0, maximum_gap_ns = 0;
  NativeHomeWireSample first{}, last{}, first_violation{};
  NativeHomeWirePrefix prefix{};
};

// Read-only diagnostic: one commissioning worker and one cyclic writer.
// The worker reads plain summary fields ONLY after the cyclic stop ack, and
// cannot begin the next audit before consuming the prior result. This release/
// acquire handoff avoids a data race without a hot-loop mutex or logging.
class NativeHomeWireAudit {
 public:
  uint64_t begin(uint64_t now_ns, bool include_prefix = false) noexcept {
    if (next_ > std::numeric_limits<uint64_t>::max() - 2) return 0;
    next_ += 2;
    triggered_token_.store(0, std::memory_order_relaxed);
    requested_ns_.store(now_ns, std::memory_order_relaxed);
    trigger_requested_ns_.store(include_prefix ? 0 : now_ns, std::memory_order_relaxed);
    request_.store(next_ | 1u, std::memory_order_release);
    return next_ | 1u;
  }

  // Mark the existing CW31 observation window without resetting or reading
  // cyclic-owned data. Earlier submitted frames retain their own prefix metrics.
  void trigger_requested(uint64_t token, uint64_t now_ns) noexcept {
    if (token && request_.load(std::memory_order_acquire) == token)
      trigger_requested_ns_.store(now_ns, std::memory_order_release);
  }

  bool trigger_submitted(uint64_t token) const noexcept {
    return token && triggered_token_.load(std::memory_order_acquire) == token;
  }

  void end(uint64_t token, uint64_t now_ns) noexcept {
    if (token) {
      ended_ns_.store(now_ns, std::memory_order_relaxed);
      request_.store(token & ~uint64_t{1}, std::memory_order_release);
    }
  }

  bool read_stopped(uint64_t token, NativeHomeWireSummary& out) const noexcept {
    if (!token || acknowledged_.load(std::memory_order_acquire) != (token & ~uint64_t{1})) return false;
    out = summary_;
    return out.token == token;
  }

  // Called once per attempted bus cycle, including failed/partial submissions.
  // Values are the final process image after all fences, not requested intent.
  void observe(bool submitted, const NativeHomeWireSample& sample, uint64_t period_ns) noexcept {
    const uint64_t request = request_.load(std::memory_order_acquire);
    if (!request || acknowledged_.load(std::memory_order_relaxed) == request) return;
    if (!(request & 1u)) {
      if (summary_.token != (request | 1u)) {
        summary_ = {};
        summary_.token = request | 1u;
        summary_.requested_ns = requested_ns_.load(std::memory_order_relaxed);
      }
      // A submission finished before the worker closed the observation window
      // may reach this hook afterward. Retain it; exclude later cleanup output.
      summary_.ended_ns = ended_ns_.load(std::memory_order_relaxed);
      if (sample.time_ns <= summary_.ended_ns) record(submitted, sample, period_ns);
      summary_.stopped_cycle = sample.cycle;
      acknowledged_.store(request, std::memory_order_release);
      return;
    }
    if (summary_.token != request) {
      summary_ = {};
      summary_.token = request;
      summary_.requested_ns = requested_ns_.load(std::memory_order_relaxed);
      previous_cycle_ = previous_ns_ = prefix_previous_cycle_ = prefix_previous_ns_ = 0;
    }
    record(submitted, sample, period_ns);
  }

 private:
  void record(bool submitted, const NativeHomeWireSample& sample, uint64_t period_ns) noexcept {
    const uint64_t triggered_at = trigger_requested_ns_.load(std::memory_order_acquire);
    if (!triggered_at || sample.time_ns < triggered_at) {
      auto& prefix = summary_.prefix;
      if (prefix.cycles) {
        if (sample.cycle <= prefix_previous_cycle_) ++prefix.nonmonotonic_cycles;
        else if (sample.cycle - prefix_previous_cycle_ > 1)
          prefix.missing_cycles += sample.cycle - prefix_previous_cycle_ - 1;
        if (sample.time_ns <= prefix_previous_ns_) ++prefix.time_gaps;
        else {
          const auto gap = sample.time_ns - prefix_previous_ns_;
          if (gap > prefix.maximum_gap_ns) prefix.maximum_gap_ns = gap;
          if (period_ns && gap > 2 * period_ns) ++prefix.time_gaps;
        }
      }
      prefix_previous_cycle_ = sample.cycle;
      prefix_previous_ns_ = sample.time_ns;
      ++prefix.cycles;
      if (!submitted) { ++prefix.submission_failures; return; }
      ++prefix.submitted;
      const unsigned index = sample.controlword == 6 ? 0 : sample.controlword == 7 ? 1 :
          sample.controlword == 15 ? 2 : 3;
      auto& bucket = prefix.buckets[index];
      if (!bucket.submitted) bucket.first = sample;
      ++bucket.submitted;
      bucket.last = sample;
      if (sample.mode == 6) ++bucket.mode6;
      else if (sample.mode == 8) ++bucket.mode8;
      else ++bucket.other_mode;
      return;
    }
    if (summary_.cycles) {
      if (sample.cycle <= previous_cycle_) ++summary_.nonmonotonic_cycles;
      else if (sample.cycle - previous_cycle_ > 1) summary_.missing_cycles += sample.cycle - previous_cycle_ - 1;
      if (sample.time_ns <= previous_ns_) ++summary_.time_gaps;
      else {
        const auto gap = sample.time_ns - previous_ns_;
        if (gap > summary_.maximum_gap_ns) summary_.maximum_gap_ns = gap;
        if (period_ns && gap > 2 * period_ns) ++summary_.time_gaps;
      }
    }
    previous_cycle_ = sample.cycle;
    previous_ns_ = sample.time_ns;
    ++summary_.cycles;
    if (!submitted) { ++summary_.submission_failures; return; }
    ++summary_.submitted;
    if (!summary_.triggered_submissions) {
      if (sample.controlword != 31 || sample.mode != 6) { ++summary_.prefix_cycles; return; }
      summary_.first = sample;
      triggered_token_.store(summary_.token, std::memory_order_release);
    }
    ++summary_.triggered_submissions;
    summary_.last = sample;
    if (sample.controlword != 31 || sample.mode != 6) {
      if (!summary_.violations) summary_.first_violation = sample;
      ++summary_.violations;
    }
  }

  uint64_t next_ = 0; // Worker-owned.
  std::atomic<uint64_t> request_{0}, acknowledged_{0}, requested_ns_{0}, ended_ns_{0}, trigger_requested_ns_{0};
  std::atomic<uint64_t> triggered_token_{0};
  NativeHomeWireSummary summary_{}; // Cyclic-owned until stop acknowledgment.
  uint64_t previous_cycle_ = 0, previous_ns_ = 0;
  uint64_t prefix_previous_cycle_ = 0, prefix_previous_ns_ = 0;
};

} // namespace rosie::rtcore
