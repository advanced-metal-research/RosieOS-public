#pragma once

#include <cstdint>
#include <array>
#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>

namespace rosie::rtcore {

// One cyclic owner keeps measured coordinates independently of command/hold
// history. A host source session identifies a qualified binding, not an observed
// encoder incarnation. Qualification must require source_valid to become false
// on reset, invalid absolute state, incoherent acquisition or a frame change.
struct CoordinateIdentity {
  uint64_t configuration_epoch = 0;
  uint64_t axis_coordinate_epoch = 0;
  uint64_t source_incarnation = 0;
  // The verified per-axis count frame survives non-coordinate changes; epochs still fence
  // stale evidence. A digest alone never makes a persisted anchor valid.
  std::array<uint8_t, 32> coordinate_identity_sha256{};
};

inline bool operator==(const CoordinateIdentity& a, const CoordinateIdentity& b) noexcept {
  return a.configuration_epoch == b.configuration_epoch &&
      a.axis_coordinate_epoch == b.axis_coordinate_epoch &&
      a.source_incarnation == b.source_incarnation && a.coordinate_identity_sha256 == b.coordinate_identity_sha256;
}

struct CoordinatePolicy {
  // Explicit inhibited/backdrive acquisition bound. Startup and origin binding
  // retain it; independent raw A6 tracking also covers admitted output caps.
  uint64_t maximum_physical_speed_counts_per_second = 0;
  uint64_t maximum_acquisition_age_ns = 0;
  uint64_t maximum_feedback_error_counts = 0;
  uint64_t maximum_anchor_error_counts = 0;
  bool qualified = false;
  // Derived only for the independent raw A6 source. Legacy source contracts
  // retain zero and their existing policy; no qualification is inferred here.
  uint64_t maximum_tracking_speed_counts_per_second = 0;
};

inline bool operator==(const CoordinatePolicy& a, const CoordinatePolicy& b) noexcept {
  return a.maximum_physical_speed_counts_per_second == b.maximum_physical_speed_counts_per_second &&
      a.maximum_acquisition_age_ns == b.maximum_acquisition_age_ns &&
      a.maximum_feedback_error_counts == b.maximum_feedback_error_counts &&
      a.maximum_anchor_error_counts == b.maximum_anchor_error_counts &&
      a.qualified == b.qualified &&
      a.maximum_tracking_speed_counts_per_second == b.maximum_tracking_speed_counts_per_second;
}

// Outward rounding in the same count frame as native position-rate admission.
// Invalid inputs return the explicit bound: their existing configuration gates
// independently prohibit output; they must never grant an unbounded allowance.
inline uint64_t coordinate_tracking_speed(uint64_t inhibited, double native_counts_per_second,
    double drive_counts_per_second) noexcept {
  if (!std::isfinite(native_counts_per_second) || !std::isfinite(drive_counts_per_second) ||
      native_counts_per_second <= 0 || drive_counts_per_second <= 0) return inhibited;
  const auto admitted = std::ceil(std::min(native_counts_per_second, drive_counts_per_second));
  if (admitted >= double(UINT64_MAX)) return inhibited;
  return std::max(inhibited, static_cast<uint64_t>(admitted));
}

inline uint64_t coordinate_tracking_speed(const CoordinatePolicy& policy) noexcept {
  return std::max(policy.maximum_physical_speed_counts_per_second,
      policy.maximum_tracking_speed_counts_per_second);
}

struct AcquisitionEvidence {
  // RT-owned, per-axis, nonzero and monotonically increasing, without reuse or
  // wrap. Only a current successful domain receipt advances either field.
  uint64_t receipt_id = 0;
  uint64_t observation_ns = 0;
  // Encoder acquisition is somewhere in [observation-max, observation-min].
  // Even two PDOs in one domain may have different acquisition instants.
  uint64_t minimum_age_ns = 0;
  uint64_t maximum_age_ns = 0;
  bool current_receipt = false;
  bool acquisition_known = false;
  bool source_valid = false;
};

struct CoordinateSample {
  int32_t raw_counts = 0;
  AcquisitionEvidence evidence{};
};

// Mailbox acquisition is an interval, never a cyclic receipt. Epochs prevent a
// late upload from becoming evidence for a different configuration or Home.
struct SampledAbsolutePosition {
  uint64_t configuration_epoch = 0;
  uint64_t home_epoch = 0;
  uint64_t cycle = 0;
  uint64_t started_ns = 0;
  uint64_t completed_ns = 0;
  int64_t encoder_counts = 0;
  uint32_t valid = 0;
  uint32_t reason = 0;
  uint64_t samples = 0;
  uint64_t first_started_ns = 0;
  uint64_t low_completed_ns = 0;
  uint64_t bracket_completed_ns = 0;
  uint64_t start_gap_ns = 0;
  // Continuous raw displacement shares command count scale, but not its origin.
  // Only explicit stationary Home or a matching persisted anchor may bind it.
  bool independent_origin = false;
  // PDO cycle immediately before the low-word request. The first high upload
  // may span many cycles; it must not enlarge the low-word motion interval.
  uint64_t low_cycle = 0;
  uint64_t audit_generation = 0;
};

// A current bracketed acquisition can make explicit Home/Restore or reference
// fault recovery available, but cannot itself establish a command coordinate.
inline bool independent_acquisition_current(const SampledAbsolutePosition& sample,
    uint64_t configuration, uint64_t home, uint64_t now, uint64_t maximum_age) noexcept {
  return sample.independent_origin && sample.valid && sample.started_ns &&
      sample.configuration_epoch == configuration && sample.home_epoch == home &&
      sample.completed_ns >= sample.started_ns && now >= sample.completed_ns &&
      now - sample.started_ns <= maximum_age && sample.encoder_counts >= INT32_MIN && sample.encoder_counts <= INT32_MAX;
}

// Read-only physical encoder observation, never Home/Restore admission. HM35
// can change the command frame while a coherent independent raw acquisition
// remains current. Omit only command-frame equality; preserve acquisition,
// configuration, age and representable-range checks. The anchor helper above
// remains the stricter authority-bearing contract.
inline bool independent_encoder_observation_current(const SampledAbsolutePosition& sample,
    uint64_t configuration, uint64_t now, uint64_t maximum_age) noexcept {
  return sample.independent_origin && sample.valid && !sample.reason &&
      configuration && sample.configuration_epoch == configuration && sample.started_ns && maximum_age &&
      sample.bracket_completed_ns >= sample.started_ns &&
      sample.low_completed_ns >= sample.bracket_completed_ns &&
      sample.completed_ns >= sample.low_completed_ns && now >= sample.completed_ns &&
      now - sample.started_ns <= maximum_age &&
      sample.encoder_counts >= INT32_MIN && sample.encoder_counts <= INT32_MAX;
}

// Continuous coordinates compare whole counts. A one-turn disagreement is a
// real mismatch and must never be aligned to a nearest representative.
inline uint64_t exact_count_error(int64_t a, int64_t b) noexcept {
  const __int128 delta = __int128(a) - b;
  const auto distance = static_cast<unsigned __int128>(delta < 0 ? -delta : delta);
  return distance > UINT64_MAX ? UINT64_MAX : uint64_t(distance);
}

// SI conversion with outward rounding; saturate instead of wrapping a budget.
inline uint64_t sampled_motion_allowance(const CoordinatePolicy& policy, uint64_t ns, bool tracking = false) noexcept {
  const auto speed = tracking ? coordinate_tracking_speed(policy) : policy.maximum_physical_speed_counts_per_second;
  const unsigned __int128 product = static_cast<unsigned __int128>(speed) * ns;
  const auto bound = product / 1000000000 + (product % 1000000000 != 0) + policy.maximum_feedback_error_counts;
  return bound > UINT64_MAX ? UINT64_MAX : static_cast<uint64_t>(bound);
}

enum class SampleInvalidity : uint32_t { None, Missing, Unavailable, Epoch, Future, Stale,
  History, Incoherent, Discontinuity, DriveFault, Cadence, Range, Torn };

inline const char* sampled_invalidity_name(uint32_t reason) noexcept {
  switch (SampleInvalidity(reason)) {
    case SampleInvalidity::None: return "none";
    case SampleInvalidity::Missing: return "missing";
    case SampleInvalidity::Unavailable: return "unavailable";
    case SampleInvalidity::Epoch: return "epoch";
    case SampleInvalidity::Future: return "future";
    case SampleInvalidity::Stale: return "stale";
    case SampleInvalidity::History: return "history";
    case SampleInvalidity::Incoherent: return "incoherent";
    case SampleInvalidity::Discontinuity: return "discontinuity";
    case SampleInvalidity::DriveFault: return "drive_fault";
    case SampleInvalidity::Cadence: return "cadence";
    case SampleInvalidity::Range: return "range";
    case SampleInvalidity::Torn: return "torn";
  }
  return "unknown";
}

// Acquisition loss is absence of evidence, never permission to rebind an origin.
inline bool sampled_evidence_missing(SampleInvalidity reason) noexcept {
  return reason == SampleInvalidity::Missing || reason == SampleInvalidity::Unavailable ||
      reason == SampleInvalidity::Stale || reason == SampleInvalidity::Cadence || reason == SampleInvalidity::Torn;
}

// Unverified memory admission cap: ceil(age/cycle)+2 <= 131072 entries per axis.
// Compiler and native loader reject oversized histories before cycling starts.
inline bool sampled_history_fits(uint64_t age_ns, uint64_t cycle_ns) noexcept {
  return cycle_ns && age_ns / cycle_ns <= 131070 &&
      (age_ns / cycle_ns < 131070 || age_ns % cycle_ns == 0);
}

// Completion never shifts acquisition start. Cadence marks only a sound
// assembly; preserving upload/bracket failures prevents false contradictions.
inline void finish_sampled_acquisition(SampledAbsolutePosition& sample, uint64_t interval_ns) noexcept {
  if (sample.completed_ns < sample.started_ns) sample.reason = uint32_t(SampleInvalidity::Future);
  else if (!sample.reason && interval_ns && sample.completed_ns - sample.started_ns > interval_ns)
    sample.reason = uint32_t(SampleInvalidity::Cadence);
  sample.valid = sample.reason == 0;
}

struct SampleConsumption {
  uint64_t configuration_epoch = 0, home_epoch = 0;
  uint64_t started_ns = 0, completed_ns = 0, acquisition_duration_ns = 0;
  uint64_t low_upload_ns = 0, high_upload_ns = 0;
  uint64_t age_ns = 0, coherence_error_counts = 0;
  uint64_t samples = 0, first_started_ns = 0;
  uint32_t reason = 0, last_invalidity_reason = 0, valid = 0;
  uint64_t bracket_upload_ns = 0, start_gap_ns = 0, maximum_start_gap_ns = 0;
  uint32_t startup_attempts = 0, startup_reason = 0, startup_verified = 0;
  uint64_t last_verified_ns = 0;
};

// Internal commissioning diagnostics only; not a wire enum or an admission
// policy. Record the first existing failed predicate without changing it.
enum class IndependentBindRefusal : uint32_t {
  None, AlreadyBound, HistoryFailed, NotIndependentSource,
  InvalidSample, MissingSampleStart, UnqualifiedPolicy, DriveFault,
  ConfigurationEpoch, HomeEpoch, SampleTimeOrder, FutureSample, StaleSample,
  MissingHistory, MissingSampleCycle, EncoderRange, CommandRange, HistoryCycle,
  HistoryGeneration, HistoryAfterSample, StationaryGeneration, StationaryWindow,
  CommandPhase, OriginRange, SampleRejected, HistoryInterval
};

inline const char* independent_bind_refusal_name(IndependentBindRefusal reason) noexcept {
  static constexpr const char* names[] = {
    "none", "already_bound", "history_failed", "not_independent_source",
    "invalid_sample", "missing_sample_start", "unqualified_policy", "drive_fault",
    "configuration_epoch", "home_epoch", "sample_time_order", "future_sample", "stale_sample",
    "missing_history", "missing_sample_cycle", "encoder_range", "command_range", "history_cycle",
    "history_generation", "history_after_sample", "stationary_generation", "stationary_window",
    "command_phase", "origin_range", "sample_rejected", "history_interval"
  };
  const auto index = static_cast<uint32_t>(reason);
  return index < sizeof(names) / sizeof(names[0]) ? names[index] : "unknown";
}

// RT-owned PDO history retains observed displacement while mailbox work waits.
// Storage is allocated before cycling. A bounded observation gap starts a new
// audit generation while exact tracking continues; reordered, impossible or
// epoch-changing samples invalidate it. An established encoder origin can only
// rebind after a configuration/Home epoch.
struct SampledPdoHistory {
  // Explicit operator-selected Home commissioning allowance, not a measured
  // hardware qualification. Restore and ongoing source checks keep their policy.
  static constexpr uint64_t maximum_home_acquisition_displacement_counts = 50;
  struct Entry { uint64_t cycle = 0, ns = 0, generation = 0; int64_t counts = 0; int32_t raw = 0; };
  struct Extrema { int64_t minimum = INT64_MAX, maximum = INT64_MIN; };
  std::vector<Entry> entries;
  // Allocated with history before cycling. Each append and range query has a
  // logarithmic worst-case bound: 17 ancestor merges per append and 18 levels
  // per query at the history cap (two queries when the retained interval wraps).
  std::vector<Extrema> extrema_tree;
  size_t extrema_leaf_count = 0;
  Entry latest{};
  uint64_t configuration = 0, home = 0, generation = 1, accepted_started_ns = 0;
  uint64_t accepted_coherence_error_counts = 0;
  int64_t bias = 0, command_offset = 0, command_counts = 0;
  bool bound = false, failed = false;
  uint64_t stationary_since_cycle = 0, stationary_generation = 0;
  int64_t stationary_min = 0, stationary_max = 0;
  SampleConsumption metrics{};
  uint64_t publication_started_ns = 0, observation_cycle_ns = 0;
  SampledAbsolutePosition startup_sample{};
  IndependentBindRefusal last_bind_refusal = IndependentBindRefusal::None;
  uint32_t last_bind_sample_reason = 0;
  int64_t last_bind_interval_min = 0, last_bind_interval_max = 0;
  int64_t last_bind_anchor_counts = 0;
  uint64_t last_bind_displacement_counts = 0, last_bind_displacement_limit = 0;
  bool last_bind_interval_valid = false;

  bool refuse_independent_bind(IndependentBindRefusal reason) noexcept {
    last_bind_refusal = reason;
    return false;
  }

  void configure(uint64_t age_ns, uint64_t cycle_ns) {
    // Unverified memory admission cap: 131072 entries per sampled axis. Reject
    // finer/larger histories instead of allocating on the cyclic path.
    if (!sampled_history_fits(age_ns, cycle_ns)) return;
    observation_cycle_ns = cycle_ns;
    entries.resize(age_ns / cycle_ns + (age_ns % cycle_ns != 0) + 2);
    extrema_leaf_count = 1;
    while (extrema_leaf_count < entries.size()) extrema_leaf_count *= 2;
    extrema_tree.assign(extrema_leaf_count * 2, Extrema{});
  }

  static Extrema merge_extrema(const Extrema& a, const Extrema& b) noexcept {
    return {std::min(a.minimum, b.minimum), std::max(a.maximum, b.maximum)};
  }

  void record_extrema(size_t index, int64_t counts) noexcept {
    size_t node = extrema_leaf_count + index;
    extrema_tree[node] = {counts, counts};
    while (node > 1) {
      node /= 2;
      extrema_tree[node] = merge_extrema(extrema_tree[node * 2], extrema_tree[node * 2 + 1]);
    }
  }

  Extrema query_extrema(size_t first, size_t last) const noexcept {
    Extrema result{};
    size_t left = extrema_leaf_count + first, right = extrema_leaf_count + last + 1;
    while (left < right) {
      if (left & 1u) result = merge_extrema(result, extrema_tree[left++]);
      if (right & 1u) result = merge_extrema(result, extrema_tree[--right]);
      left /= 2; right /= 2;
    }
    return result;
  }

  bool query_history_interval(uint64_t first_cycle, uint64_t last_cycle,
      int64_t& minimum, int64_t& maximum) const noexcept {
    if (!first_cycle || first_cycle > last_cycle || entries.empty() ||
        last_cycle - first_cycle >= entries.size() || !extrema_leaf_count ||
        extrema_tree.size() != extrema_leaf_count * 2 || extrema_leaf_count < entries.size()) return false;
    const size_t first = first_cycle % entries.size(), last = last_cycle % entries.size();
    const auto& begin = entries[first]; const auto& end = entries[last];
    if (begin.cycle != first_cycle || end.cycle != last_cycle ||
        begin.generation != end.generation || end.generation != latest.generation) return false;
    const auto interval = first <= last ? query_extrema(first, last)
        : merge_extrema(query_extrema(first, entries.size() - 1), query_extrema(0, last));
    if (interval.minimum > interval.maximum) return false;
    minimum = interval.minimum; maximum = interval.maximum;
    return true;
  }

  void observe(uint64_t config, uint64_t frame, uint64_t cycle, uint64_t ns,
      int32_t raw, bool current, const CoordinatePolicy& policy) noexcept {
    if (config != configuration || frame != home) {
      configuration = config; home = frame; latest = {}; ++generation;
      bound = failed = false; accepted_started_ns = 0; accepted_coherence_error_counts = 0; command_offset = 0;
      startup_sample = {}; metrics.startup_attempts = metrics.startup_reason = metrics.startup_verified = 0;
      metrics.configuration_epoch = config; metrics.home_epoch = frame;
    }
    if (entries.empty() || !current || !policy.qualified) {
      metrics.valid = 0;
      metrics.reason = metrics.last_invalidity_reason = uint32_t(
          !current ? SampleInvalidity::Unavailable :
          !policy.qualified ? SampleInvalidity::Missing : SampleInvalidity::History);
      latest = {}; ++generation; if (bound) failed = true; return;
    }
    int64_t counts = raw;
    if (latest.cycle) {
      const int64_t delta = int64_t(raw) - latest.raw;
      uint64_t separation = ns >= latest.ns ? ns - latest.ns : UINT64_MAX;
      if (cycle > latest.cycle && observation_cycle_ns) {
        const unsigned __int128 scheduled =
            static_cast<unsigned __int128>(cycle - latest.cycle) * observation_cycle_ns;
        const auto scheduled_ns = scheduled > UINT64_MAX ? UINT64_MAX : static_cast<uint64_t>(scheduled);
        separation = std::max(separation, scheduled_ns);
      }
      const auto allowance = sampled_motion_allowance(policy, separation, true);
      const auto distance = uint64_t(delta < 0 ? -delta : delta);
      const __int128 next = __int128(latest.counts) + delta;
      if (cycle <= latest.cycle || ns <= latest.ns || distance > allowance ||
          next < INT64_MIN || next > INT64_MAX) {
        metrics.valid = 0;
        metrics.reason = metrics.last_invalidity_reason = uint32_t(SampleInvalidity::Discontinuity);
        metrics.age_ns = ns >= latest.ns ? ns - latest.ns : 0;
        metrics.coherence_error_counts = distance;
        ++generation; if (bound) failed = true;
      } else {
        counts = static_cast<int64_t>(next);
        // Missing observations cannot contribute to a bracketed mailbox audit.
        // Fence its interval without discarding exact, speed-bounded PDO counts.
        if (cycle != latest.cycle + 1) ++generation;
      }
    }
    if (!stationary_since_cycle || stationary_generation != generation ||
        __int128(std::max(stationary_max, counts)) - std::min(stationary_min, counts) > policy.maximum_anchor_error_counts) {
      stationary_since_cycle = cycle; stationary_generation = generation;
      stationary_min = stationary_max = counts;
    } else {
      stationary_min = std::min(stationary_min, counts);
      stationary_max = std::max(stationary_max, counts);
    }
    latest = {cycle, ns, generation, counts, raw};
    entries[cycle % entries.size()] = latest;
    record_extrema(cycle % entries.size(), counts);
  }

  bool accept(const SampledAbsolutePosition& sample, const CoordinatePolicy& policy,
      uint64_t now, bool faulted) noexcept {
    metrics.started_ns = sample.started_ns; metrics.completed_ns = sample.completed_ns;
    metrics.acquisition_duration_ns = sample.completed_ns >= sample.started_ns ? sample.completed_ns - sample.started_ns : 0;
    const auto low_started = sample.bracket_completed_ns ? sample.bracket_completed_ns : sample.started_ns;
    metrics.bracket_upload_ns = low_started >= sample.started_ns ? low_started - sample.started_ns : 0;
    metrics.low_upload_ns = sample.low_completed_ns >= low_started ? sample.low_completed_ns - low_started : 0;
    metrics.high_upload_ns = sample.low_completed_ns && sample.completed_ns >= sample.low_completed_ns ? sample.completed_ns - sample.low_completed_ns : 0;
    metrics.age_ns = now >= sample.started_ns ? now - sample.started_ns : 0;
    metrics.coherence_error_counts = sample.started_ns == accepted_started_ns ? accepted_coherence_error_counts : 0;
    metrics.samples = sample.samples; metrics.first_started_ns = sample.first_started_ns;
    metrics.start_gap_ns = sample.start_gap_ns;
    if (sample.start_gap_ns > metrics.maximum_start_gap_ns) metrics.maximum_start_gap_ns = sample.start_gap_ns;
    auto reject = [&](SampleInvalidity reason) {
      metrics.valid = 0; metrics.reason = metrics.last_invalidity_reason = uint32_t(reason); return false;
    };
    if (!policy.qualified || !sample.started_ns) return reject(SampleInvalidity::Missing);
    const bool cadence = !sample.valid && sample.reason == uint32_t(SampleInvalidity::Cadence);
    if (!sample.valid && !cadence) return reject(sample.reason ? SampleInvalidity(sample.reason) : SampleInvalidity::Unavailable);
    if (sample.configuration_epoch != configuration || sample.home_epoch != home) return reject(SampleInvalidity::Epoch);
    if (now < sample.completed_ns || sample.completed_ns < sample.started_ns) return reject(SampleInvalidity::Future);
    // Sound late assemblies still expose contradictions in the receiving cycle.
    // Their age/cadence never authorizes a binding or renews retained evidence.
    const auto loss = cadence ? SampleInvalidity::Cadence :
        now - sample.started_ns > policy.maximum_acquisition_age_ns ? SampleInvalidity::Stale : SampleInvalidity::None;
    if (faulted) return reject(SampleInvalidity::DriveFault);
    if (failed) return reject(SampleInvalidity::Discontinuity);
    const auto no_history = loss != SampleInvalidity::None ? loss : SampleInvalidity::History;
    if (entries.empty() || !latest.cycle) return reject(no_history);
    const auto& acquisition_start = entries[sample.cycle % entries.size()];
    if (!sample.cycle || acquisition_start.cycle != sample.cycle || acquisition_start.generation != latest.generation ||
        acquisition_start.ns > sample.started_ns) return reject(no_history);
    const bool tracked_low = bound && sample.independent_origin && sample.low_cycle;
    const auto& start = tracked_low ? entries[sample.low_cycle % entries.size()] : acquisition_start;
    if (tracked_low && (sample.low_cycle < sample.cycle || start.cycle != sample.low_cycle ||
        start.generation != latest.generation || sample.bracket_completed_ns < sample.started_ns ||
        start.ns > sample.bracket_completed_ns || sample.bracket_completed_ns > sample.low_completed_ns))
      return reject(no_history);
    // The low word was acquired inside this interval; the later high upload
    // supplies signed turns. Only acquisition skew needs a motion allowance.
    const auto acquired = sample.low_completed_ns ? sample.low_completed_ns : sample.completed_ns;
    if (acquired < sample.started_ns || acquired > sample.completed_ns) return reject(SampleInvalidity::Future);
    const auto allowance = sampled_motion_allowance(policy, acquired - start.ns, bound && sample.independent_origin);
    // Acquisition alone cannot establish the independent raw/command origin.
    // This refusal does not poison the history needed for explicit Home/Restore.
    if (sample.independent_origin && !bound) return reject(SampleInvalidity::Missing);
    int64_t encoder_counts = sample.encoder_counts;
    if (!bound && (encoder_counts < INT32_MIN || encoder_counts > INT32_MAX)) {
      return reject(SampleInvalidity::Range);
    }
    if (sample.started_ns != accepted_started_ns) {
      if (bound) {
        const __int128 tracked = __int128(start.counts) + bias;
        const __int128 error = __int128(encoder_counts) - tracked;
        const auto distance = static_cast<unsigned __int128>(error < 0 ? -error : error);
        metrics.coherence_error_counts = distance > UINT64_MAX ? UINT64_MAX : uint64_t(distance);
      } else metrics.coherence_error_counts = exact_count_error(encoder_counts, start.raw);
      if (metrics.coherence_error_counts > allowance) { failed = true; return reject(SampleInvalidity::Incoherent); }
      if (loss != SampleInvalidity::None) return reject(loss);
      if (!bound) {
        const __int128 offset = __int128(encoder_counts) - start.counts;
        if (offset < INT64_MIN || offset > INT64_MAX) return reject(SampleInvalidity::Range);
        bias = int64_t(offset); bound = true;
      }
      accepted_started_ns = sample.started_ns;
      accepted_coherence_error_counts = metrics.coherence_error_counts;
    }
    if (loss != SampleInvalidity::None) return reject(loss);
    const __int128 command = __int128(latest.counts) + bias + command_offset;
    if (command < INT32_MIN || command > INT32_MAX || encoder_counts < INT32_MIN || encoder_counts > INT32_MAX)
      return reject(SampleInvalidity::Range); // v1 persisted anchors carry signed32 frames.
    command_counts = int64_t(command);
    metrics.valid = 1; metrics.reason = 0;
    return true;
  }

  // A transient publication cannot replace age-bounded valid evidence. Check
  // every new acquisition for contradictions before falling back to retention.
  // Metrics describe the retained acquisition; last_invalidity records rejected
  // publications and the gap high-water mark includes failed acquisitions.
  bool consume(const SampledAbsolutePosition& newest, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, bool faulted) noexcept {
    if (!failed && newest.started_ns > publication_started_ns && newest.completed_ns <= now) {
      publication_started_ns = newest.started_ns;
      const bool valid = accept(newest, policy, now, faulted);
      const auto reason = SampleInvalidity(metrics.reason);
      if (valid || !sampled_evidence_missing(reason) || !retained.valid) {
        retained = newest;
        return valid;
      }
    }
    return accept(retained, policy, now, faulted);
  }

  // Home fixes the origin; cyclic PDO displacement carries it forward. A
  // mailbox receipt is an optional stationary audit, never a renewable Home lease.
  bool track_independent(const SampledAbsolutePosition& newest, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, bool encoder_fault, bool audit_allowed,
      uint64_t audit_generation) noexcept {
    if (encoder_fault) { failed = true; metrics.reason = uint32_t(SampleInvalidity::DriveFault); }
    if (newest.started_ns > publication_started_ns && newest.completed_ns <= now) {
      publication_started_ns = newest.started_ns;
      int64_t minimum = 0, maximum = 0;
      const bool stationary = query_history_interval(newest.cycle, latest.cycle, minimum, maximum) &&
          __int128(maximum) - minimum <= maximum_home_acquisition_displacement_counts;
      // Late, moving or fenced publications cannot revoke position trust.
      // Preserve their diagnostics without feeding them into the binding.
      const bool fresh = newest.started_ns <= newest.completed_ns && now >= newest.started_ns &&
          now - newest.started_ns <= policy.maximum_acquisition_age_ns;
      if (audit_allowed && newest.audit_generation == audit_generation && stationary && fresh && bound && !failed) {
        if (accept(newest, policy, now, false)) {
          retained = newest;
          metrics.last_verified_ns = newest.completed_ns;
        }
        // Only accept()'s coherent contradiction can poison this relationship.
        // Torn/incomplete reads remain acquisition failures, never tracking loss.
      } else {
        metrics.started_ns = newest.started_ns; metrics.completed_ns = newest.completed_ns;
        metrics.reason = metrics.last_invalidity_reason = uint32_t(
            newest.audit_generation != audit_generation || !audit_allowed ? SampleInvalidity::Epoch :
            !newest.valid && newest.reason ? SampleInvalidity(newest.reason) :
            !fresh ? SampleInvalidity::Stale : SampleInvalidity::History);
        metrics.valid = 0;
      }
    }
    if (!failed)
      metrics.age_ns = retained.started_ns && now >= retained.started_ns ? now - retained.started_ns : 0;
    if (!bound || failed || !latest.cycle) return false;
    const __int128 command = __int128(latest.counts) + bias + command_offset;
    const __int128 encoder = __int128(latest.counts) + bias;
    if (command < INT32_MIN || command > INT32_MAX || encoder < INT32_MIN || encoder > INT32_MAX) {
      failed = true; metrics.reason = uint32_t(SampleInvalidity::Range); return false;
    }
    command_counts = int64_t(command);
    return true;
  }

  // Only the commissioning boundary calls this, after explicit stationary Home
  // or a matching persisted v2 anchor. The offset is fixed for the frame; normal
  // acceptance still compares full continuous raw counts, including whole turns.
  bool bind_independent_origin(const SampledAbsolutePosition& sample, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, int64_t command_at_sample, bool faulted) noexcept {
    return bind_independent_origin_with_displacement(sample, retained, policy, now,
        command_at_sample, faulted, policy.maximum_anchor_error_counts);
  }

  // Startup recovery uses the saved calibration and the drive's retained Home.
  // This is a stationary acquisition, with the same operator-selected margin as
  // Home; the established running reference keeps its ordinary policy.
  bool restore_retained_origin(const SampledAbsolutePosition& sample, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, int64_t command_at_sample, bool faulted) noexcept {
    auto acquisition = policy;
    acquisition.maximum_anchor_error_counts = std::max(policy.maximum_anchor_error_counts,
        maximum_home_acquisition_displacement_counts);
    return bind_independent_origin_with_displacement(sample, retained, acquisition, now,
        command_at_sample, faulted, acquisition.maximum_anchor_error_counts);
  }

private:
  bool bind_independent_origin_with_displacement(const SampledAbsolutePosition& sample, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, int64_t command_at_sample, bool faulted,
      uint64_t displacement_limit) noexcept {
    using Reason = IndependentBindRefusal;
    last_bind_refusal = Reason::None; last_bind_sample_reason = 0;
    last_bind_interval_min = last_bind_interval_max = 0; last_bind_interval_valid = false;
    last_bind_anchor_counts = 0; last_bind_displacement_counts = 0;
    last_bind_displacement_limit = displacement_limit;
    if (bound) return refuse_independent_bind(Reason::AlreadyBound);
    if (failed) return refuse_independent_bind(Reason::HistoryFailed);
    if (!sample.independent_origin) return refuse_independent_bind(Reason::NotIndependentSource);
    if (!sample.valid) return refuse_independent_bind(Reason::InvalidSample);
    if (!sample.started_ns) return refuse_independent_bind(Reason::MissingSampleStart);
    if (!policy.qualified) return refuse_independent_bind(Reason::UnqualifiedPolicy);
    if (faulted) return refuse_independent_bind(Reason::DriveFault);
    if (sample.configuration_epoch != configuration) return refuse_independent_bind(Reason::ConfigurationEpoch);
    if (sample.home_epoch != home) return refuse_independent_bind(Reason::HomeEpoch);
    if (sample.completed_ns < sample.started_ns) return refuse_independent_bind(Reason::SampleTimeOrder);
    if (now < sample.completed_ns) return refuse_independent_bind(Reason::FutureSample);
    if (now - sample.started_ns > policy.maximum_acquisition_age_ns) return refuse_independent_bind(Reason::StaleSample);
    if (entries.empty()) return refuse_independent_bind(Reason::MissingHistory);
    if (!sample.cycle) return refuse_independent_bind(Reason::MissingSampleCycle);
    if (sample.encoder_counts < INT32_MIN || sample.encoder_counts > INT32_MAX) return refuse_independent_bind(Reason::EncoderRange);
    if (command_at_sample < INT32_MIN || command_at_sample > INT32_MAX) return refuse_independent_bind(Reason::CommandRange);
    const auto& start = entries[sample.cycle % entries.size()];
    if (start.cycle != sample.cycle) return refuse_independent_bind(Reason::HistoryCycle);
    if (start.generation != latest.generation) return refuse_independent_bind(Reason::HistoryGeneration);
    if (start.ns > sample.started_ns) return refuse_independent_bind(Reason::HistoryAfterSample);
    if (stationary_generation != latest.generation) return refuse_independent_bind(Reason::StationaryGeneration);
    if (!query_history_interval(sample.cycle, latest.cycle, last_bind_interval_min, last_bind_interval_max))
      return refuse_independent_bind(Reason::HistoryInterval);
    last_bind_interval_valid = true;
    // The independent raw value was acquired somewhere in this interval. Its
    // pairing error is displacement from the witnessed start anchor, not the
    // interval's total span or extrema from before acquisition. Fresh Home has
    // its own explicit commissioning margin; Restore retains the anchor budget.
    last_bind_anchor_counts = start.counts;
    const auto displacement = std::max(__int128(start.counts) - last_bind_interval_min,
        __int128(last_bind_interval_max) - start.counts);
    last_bind_displacement_counts = displacement > UINT64_MAX ? UINT64_MAX : uint64_t(displacement);
    if (displacement > displacement_limit)
      return refuse_independent_bind(Reason::StationaryWindow);
    if (exact_count_error(start.raw, command_at_sample) > policy.maximum_anchor_error_counts)
      return refuse_independent_bind(Reason::CommandPhase);
    const __int128 origin = __int128(sample.encoder_counts) - start.counts;
    const __int128 command_origin = __int128(command_at_sample) - sample.encoder_counts;
    if (origin < INT64_MIN || origin > INT64_MAX || command_origin < INT64_MIN || command_origin > INT64_MAX)
      return refuse_independent_bind(Reason::OriginRange);
    const auto old_bias = bias, old_offset = command_offset;
    bias = int64_t(origin); command_offset = int64_t(command_origin); bound = true;
    auto binding_policy = policy;
    binding_policy.maximum_tracking_speed_counts_per_second = 0;
    auto binding_sample = sample;
    binding_sample.low_cycle = 0; // full acquisition remains the binding proof
    if (!accept(binding_sample, binding_policy, now, faulted)) {
      bound = false; bias = old_bias; command_offset = old_offset;
      last_bind_sample_reason = metrics.reason;
      return refuse_independent_bind(Reason::SampleRejected);
    }
    retained = startup_sample = sample;
    metrics.last_verified_ns = sample.completed_ns;
    publication_started_ns = sample.started_ns;
    metrics.startup_attempts = metrics.startup_verified = 1; metrics.startup_reason = 0;
    return true;
  }

public:
  // Home truth is the logical-zero offset, not the current PDO position after
  // the drive releases its homing mode. Preserve any real displacement from
  // zero when binding the independent encoder to the witnessed command frame.
  bool complete_independent_home(const SampledAbsolutePosition& sample, SampledAbsolutePosition& retained,
      const CoordinatePolicy& policy, uint64_t now, int64_t command_home, bool faulted) noexcept {
    using Reason = IndependentBindRefusal;
    last_bind_refusal = Reason::None; last_bind_sample_reason = 0;
    last_bind_interval_min = last_bind_interval_max = 0; last_bind_interval_valid = false;
    last_bind_anchor_counts = 0; last_bind_displacement_counts = 0;
    last_bind_displacement_limit = maximum_home_acquisition_displacement_counts;
    if (entries.empty()) return refuse_independent_bind(Reason::MissingHistory);
    if (!sample.cycle) return refuse_independent_bind(Reason::MissingSampleCycle);
    if (command_home < INT32_MIN || command_home > INT32_MAX) return refuse_independent_bind(Reason::CommandRange);
    const auto& start = entries[sample.cycle % entries.size()];
    if (start.cycle != sample.cycle) return refuse_independent_bind(Reason::HistoryCycle);
    // HM35 established command_home at the instant of drive homing. The PDO
    // history is the exact command frame afterward, so preserve any real
    // displacement witnessed before the independent acquisition completed.
    return bind_independent_origin_with_displacement(sample, retained, policy, now,
        start.raw, faulted, maximum_home_acquisition_displacement_counts);
  }
};

// P4-03 stores command counts and independent encoder counts separately. Both
// must agree at restoration in the complete continuous count frame.
inline bool sampled_anchor_matches(const SampledAbsolutePosition& sample, int32_t pdo,
    int32_t saved_command, int32_t saved_encoder, uint64_t tolerance) noexcept {
  const __int128 error = __int128(sample.encoder_counts) - saved_encoder;
  return sample.valid && (error < 0 ? -error : error) <= tolerance &&
      exact_count_error(saved_command, pdo) <= tolerance;
}

// Automatic startup recognition of a persisted Home names its first failed
// precondition. A refusal never commands motion, Home or a drive write; the
// explicit restore operations remain the operator's fallback. Only an HM35 (v3)
// anchor is eligible: a current-pose (v5) file is never a Home.
enum class RetainedHomeRefusal : uint32_t {
  None, Disabled, DriveFault, HomingAttained, DriveError, AlarmUnread, EncoderAlarm,
  MultiTurnOverflow, OffsetChanged, Unstationary, Disagreement, HardLimit, CurrentReferenceAnchor
};

inline const char* retained_home_refusal_name(RetainedHomeRefusal reason) noexcept {
  static constexpr const char* names[] = {"none", "disabled", "drive_fault", "homing_attained_missing",
    "drive_error", "encoder_alarm_unread", "encoder_alarm", "multi_turn_overflow", "offset_changed",
    "unstationary", "anchor_disagrees", "hard_limit", "current_reference_anchor_rejected"};
  const auto index = uint32_t(reason);
  return index < sizeof(names) / sizeof(names[0]) ? names[index] : "unknown";
}

// One attempt's drive-side evidence. The A6-EC clears 6041 bit 15 at every
// power-up by firmware design (vendor reply 2026-04-15, email 4 Q1), so a
// battery-backed multi-turn position cannot depend on it. The same reply (Q2)
// names the direct loss signals instead: Er20.8, Er20.9, ErA0.1 and ALF9.0 all
// raise 603F=0x7305 (a6ec_manual_codes.md); U41.05 (0x2041:06) bits 13-15 are
// multi-turn fault, battery fault and battery warning; C10.15 (0x2010:16) is the
// multi-turn overflow flag. Both uploads belong to the same attempt as the raw
// sample; an unread object is not evidence that the alarm is clear.
struct RetainedHomeDriveHealth {
  uint16_t statusword = 0;
  uint16_t error_code = 0;      // Cyclic 0x603F; zero is required, warnings included.
  bool drive_error = false;     // Admitted 0x203F, manufacturer code or unmapped 0x603F.
  bool alarm_objects_read = false;
  uint16_t encoder_fault_state = 0; // U41.05
  uint16_t multi_turn_overflow = 0; // C10.15
};

inline constexpr uint16_t kA6ecEncoderFaultStateIndex = 0x2041, kA6ecMultiTurnOverflowIndex = 0x2010;
inline constexpr uint8_t kA6ecEncoderFaultStateSubindex = 0x06, kA6ecMultiTurnOverflowSubindex = 0x16;
inline constexpr uint16_t kA6ecEncoderLossBits = 0xe000; // U41.05 bits 13, 14 and 15.

// Statusword bit 3 always refuses. Bits 15/12 (with bit 13 clear) are required
// only when the drive profile declares firmware that retains them.
inline RetainedHomeRefusal retained_home_drive_refusal(const RetainedHomeDriveHealth& health,
    bool require_homing_attained) noexcept {
  using R = RetainedHomeRefusal;
  if (health.statusword & 0x0008u) return R::DriveFault;
  if (require_homing_attained && (health.statusword & 0xb000u) != 0x9000u) return R::HomingAttained;
  if (health.error_code || health.drive_error) return R::DriveError;
  if (!health.alarm_objects_read) return R::AlarmUnread;
  if (health.encoder_fault_state & kA6ecEncoderLossBits) return R::EncoderAlarm;
  if (health.multi_turn_overflow) return R::MultiTurnOverflow;
  return R::None;
}

// A retained drive Home plus the saved raw-to-command relation permits movement
// since calibration. It does not require the machine to be back at Home. The
// full raw and PDO deltas preserve and compare complete turns.
// Drive health and the Home-attained policy are retained_home_drive_refusal's.
inline bool retained_home_anchor_matches(const SampledAbsolutePosition& sample,
    int32_t pdo, int32_t saved_command, int32_t saved_encoder, int32_t saved_offset,
    int32_t drive_offset, uint64_t tolerance, int64_t& command) noexcept {
  if (saved_offset != drive_offset ||
      !sample.valid || !sample.independent_origin) return false;
  const __int128 rebound = __int128(saved_command) + sample.encoder_counts - saved_encoder;
  if (rebound < INT32_MIN || rebound > INT32_MAX) return false;
  if (exact_count_error(int64_t(rebound), pdo) > tolerance) return false;
  command = int64_t(rebound);
  return true;
}

enum class CoordinateAnchorSource : uint8_t { Unspecified, VerifiedHome, AbsolutePosition };

struct CoordinateAnchor {
  CoordinateIdentity identity{};
  uint64_t serial = 0; // Non-renewable RT receipt/event ID, never reused/reset.
  CoordinateAnchorSource source = CoordinateAnchorSource::Unspecified;
  int64_t continuous_counts = 0; // Same count frame as the continuous command.
  AcquisitionEvidence evidence{};
  // Set only after the profile's Home/absolute evidence contract is satisfied.
  // Offsets, target proximity, configuration presence and a bus receipt alone
  // are not verified anchors.
  bool verified = false;
};

enum class CoordinateReason : uint8_t {
  None,
  Unanchored,
  InvalidIdentity,
  IdentityChanged,
  Unqualified,
  InvalidPolicy,
  PolicyChanged,
  MissingReceipt,
  InvalidReceipt,
  NonMonotonicReceipt,
  NonMonotonicTime,
  UnknownAcquisition,
  InvalidAcquisitionWindow,
  AcquisitionExpired,
  SourceInvalid,
  InvalidAnchor,
  StaleAnchor,
  AnchorReceiptMismatch,
  AlreadyAnchored,
  ImpossibleMovement,
  Ambiguous,
  ArithmeticOverflow,
  OffsetChanged,
};

inline const char* coordinate_reason_name(CoordinateReason reason) noexcept {
  static constexpr const char* names[] = {"none", "unanchored", "invalid_identity", "identity_changed",
    "unqualified", "invalid_policy", "policy_changed", "missing_receipt", "invalid_receipt", "non_monotonic_receipt",
    "non_monotonic_time", "unknown_acquisition", "invalid_acquisition_window", "acquisition_expired", "source_invalid",
    "invalid_anchor", "stale_anchor", "anchor_receipt_mismatch", "already_anchored", "impossible_movement",
    "ambiguous", "arithmetic_overflow", "offset_changed"};
  const auto i = static_cast<unsigned>(reason);
  return i < std::size(names) ? names[i] : "unknown";
}

struct CoordinateReference {
  CoordinateIdentity identity{};
  CoordinatePolicy policy{};
  uint64_t highest_anchor_serial = 0; // Includes consumed failed recovery attempts.
  // Invalid samples advance these floors too. An old anchor cannot turn an
  // already-observed loss into a new coordinate reference.
  uint64_t last_seen_receipt_id = 0;
  uint64_t last_seen_observation_ns = 0;
  int64_t continuous_counts = 0;
  int32_t raw_counts = 0;
  AcquisitionEvidence accepted_evidence{};
  bool valid = false;
  CoordinateReason failure = CoordinateReason::Unanchored;
};

struct CoordinateDecision {
  CoordinateReference next{};
  CoordinateReason reason = CoordinateReason::None;
  bool accepted = false;
};

inline CoordinateDecision invalidate_coordinate(CoordinateReference state,
    CoordinateReason reason) noexcept {
  state.valid = false;
  state.failure = reason;
  return {state, reason, false};
}

namespace coordinate_detail {

struct AcquisitionWindow { uint64_t earliest_ns; uint64_t latest_ns; };

inline CoordinateReason validate_policy(const CoordinatePolicy& policy) noexcept {
  if (!policy.qualified) return CoordinateReason::Unqualified;
  if (!policy.maximum_physical_speed_counts_per_second)
    return CoordinateReason::InvalidPolicy;
  return CoordinateReason::None;
}

inline CoordinateReference current_context(CoordinateReference state,
    const CoordinateIdentity& identity, const CoordinatePolicy& policy) noexcept {
  if (!identity.configuration_epoch || !identity.axis_coordinate_epoch || !identity.source_incarnation)
    return invalidate_coordinate(state, CoordinateReason::InvalidIdentity).next;
  const auto policy_reason = validate_policy(policy);
  if (policy_reason != CoordinateReason::None)
    return invalidate_coordinate(state, policy_reason).next;
  if (state.valid && !(state.identity == identity))
    return invalidate_coordinate(state, CoordinateReason::IdentityChanged).next;
  if (state.valid && !(state.policy == policy))
    return invalidate_coordinate(state, CoordinateReason::PolicyChanged).next;
  return state;
}

inline CoordinateReason acquisition_window(const AcquisitionEvidence& evidence,
    const CoordinatePolicy& policy, AcquisitionWindow& out) noexcept {
  if (!evidence.current_receipt) return CoordinateReason::MissingReceipt;
  if (!evidence.receipt_id || !evidence.observation_ns) return CoordinateReason::InvalidReceipt;
  if (!evidence.source_valid) return CoordinateReason::SourceInvalid;
  if (!evidence.acquisition_known) return CoordinateReason::UnknownAcquisition;
  if (evidence.minimum_age_ns > evidence.maximum_age_ns ||
      evidence.maximum_age_ns > evidence.observation_ns)
    return CoordinateReason::InvalidAcquisitionWindow;
  if (evidence.maximum_age_ns > policy.maximum_acquisition_age_ns)
    return CoordinateReason::AcquisitionExpired;
  out = {evidence.observation_ns - evidence.maximum_age_ns,
         evidence.observation_ns - evidence.minimum_age_ns};
  return CoordinateReason::None;
}

inline uint64_t absolute_difference(uint64_t a, uint64_t b) noexcept {
  return a >= b ? a - b : b - a;
}

inline uint64_t maximum_separation(const AcquisitionWindow& a,
    const AcquisitionWindow& b) noexcept {
  const auto first = absolute_difference(a.earliest_ns, b.latest_ns);
  const auto second = absolute_difference(a.latest_ns, b.earliest_ns);
  return first > second ? first : second;
}

inline bool displacement_bound(const CoordinatePolicy& policy,
    const AcquisitionWindow& previous, const AcquisitionWindow& current,
    uint64_t previous_error, uint64_t current_error, uint64_t& result) noexcept {
  // Fixed-width integer arithmetic includes outward rounding before adding the
  // independent measurement errors. Native GCC/Clang support this on x86/aarch64.
  using Wide = unsigned __int128;
  constexpr uint64_t billion = 1000000000;
  const Wide product = Wide(coordinate_tracking_speed(policy)) *
      maximum_separation(previous, current);
  const Wide bound = product / billion + (product % billion != 0) +
      Wide(previous_error) + current_error;
  if (bound > std::numeric_limits<uint64_t>::max()) return false;
  result = static_cast<uint64_t>(bound);
  return true;
}

inline CoordinateReason unique_equivalent(int64_t previous, int32_t raw,
    uint64_t bound, int64_t& result) noexcept {
  const int64_t next = raw;
  const auto distance = exact_count_error(previous, next);
  if (distance > bound) return CoordinateReason::ImpossibleMovement;
  result = next;
  return CoordinateReason::None;
}

inline void note_observation(CoordinateReference& state,
    const AcquisitionEvidence& evidence) noexcept {
  if (evidence.receipt_id > state.last_seen_receipt_id)
    state.last_seen_receipt_id = evidence.receipt_id;
  if (evidence.observation_ns > state.last_seen_observation_ns)
    state.last_seen_observation_ns = evidence.observation_ns;
}

inline CoordinateReason newer_observation(const CoordinateReference& state,
    const AcquisitionEvidence& evidence) noexcept {
  if (evidence.receipt_id <= state.last_seen_receipt_id)
    return CoordinateReason::NonMonotonicReceipt;
  if (evidence.observation_ns <= state.last_seen_observation_ns)
    return CoordinateReason::NonMonotonicTime;
  return CoordinateReason::None;
}

} // namespace coordinate_detail

// A current-domain failure or coordinate-source loss invalidates conservatively;
// later ordinary samples cannot silently restore home/coordinates. Authority,
// drive mode and execution faults are absent: Stop, disabled motion and normal
// following error do not themselves destroy continuously observed coordinates.
inline CoordinateDecision observe_coordinate(CoordinateReference state,
    const CoordinateIdentity& identity, const CoordinatePolicy& policy,
    const CoordinateSample& sample) noexcept {
  using namespace coordinate_detail;
  state = current_context(state, identity, policy);
  const auto before_observation = state;
  note_observation(state, sample.evidence);
  if (!state.valid) return {state, state.failure, false};
  AcquisitionWindow current{}, previous{};
  auto reason = acquisition_window(sample.evidence, policy, current);
  if (reason != CoordinateReason::None) return invalidate_coordinate(state, reason);
  reason = newer_observation(before_observation, sample.evidence);
  if (reason != CoordinateReason::None) return invalidate_coordinate(state, reason);
  reason = acquisition_window(state.accepted_evidence, policy, previous);
  if (reason != CoordinateReason::None) return invalidate_coordinate(state, reason);
  uint64_t bound = 0;
  if (!displacement_bound(policy, previous, current, policy.maximum_feedback_error_counts,
          policy.maximum_feedback_error_counts, bound))
    return invalidate_coordinate(state, CoordinateReason::ArithmeticOverflow);
  int64_t continuous = 0;
  reason = unique_equivalent(state.continuous_counts, sample.raw_counts, bound, continuous);
  if (reason != CoordinateReason::None) return invalidate_coordinate(state, reason);
  state.continuous_counts = continuous;
  state.raw_counts = sample.raw_counts;
  state.accepted_evidence = sample.evidence;
  return {state, CoordinateReason::None, true};
}

// A new anchor and continuous sample must belong to one current domain receipt.
// Their separately qualified acquisition intervals/error bounds determine the
// admissible interval; domain co-location never implies simultaneous sampling.
// Recovery only seeds an invalid reference. A valid same-context reference must
// keep passing observe_coordinate instead of resetting its physical-motion test.
// The caller must not clear home or execution faults merely because this succeeds.
inline CoordinateDecision reconcile_coordinate(CoordinateReference state,
    const CoordinateIdentity& identity, const CoordinatePolicy& policy,
    const CoordinateSample& sample, const CoordinateAnchor& anchor) noexcept {
  using namespace coordinate_detail;
  state = current_context(state, identity, policy);
  if (!identity.configuration_epoch || !identity.axis_coordinate_epoch || !identity.source_incarnation ||
      validate_policy(policy) != CoordinateReason::None)
    return {state, state.failure, false};
  if (!anchor.serial || !anchor.verified ||
      (anchor.source != CoordinateAnchorSource::VerifiedHome &&
       anchor.source != CoordinateAnchorSource::AbsolutePosition))
    return {state, CoordinateReason::InvalidAnchor, false};
  if (anchor.serial <= state.highest_anchor_serial)
    return {state, CoordinateReason::StaleAnchor, false};
  if (!(anchor.identity == identity)) return {state, CoordinateReason::IdentityChanged, false};
  if (anchor.evidence.receipt_id != sample.evidence.receipt_id ||
      anchor.evidence.observation_ns != sample.evidence.observation_ns)
    return {state, CoordinateReason::AnchorReceiptMismatch, false};
  auto reason = newer_observation(state, sample.evidence);
  if (reason != CoordinateReason::None) return {state, reason, false};
  if (!state.valid) {
    // Consume a correctly identified/paired recovery event before testing its
    // acquisition proof. A failed or expired event cannot later be re-stamped.
    state.highest_anchor_serial = anchor.serial;
    note_observation(state, sample.evidence);
  }
  AcquisitionWindow current{}, reference{};
  reason = acquisition_window(sample.evidence, policy, current);
  if (reason != CoordinateReason::None) {
    note_observation(state, sample.evidence);
    return invalidate_coordinate(state, reason);
  }
  reason = acquisition_window(anchor.evidence, policy, reference);
  if (reason != CoordinateReason::None) {
    if (!state.valid) return invalidate_coordinate(state, reason);
    return {state, reason, false};
  }
  if (state.valid) return {state, CoordinateReason::AlreadyAnchored, false};
  uint64_t bound = 0;
  if (!displacement_bound(policy, reference, current, policy.maximum_anchor_error_counts,
          policy.maximum_feedback_error_counts, bound))
    return invalidate_coordinate(state, CoordinateReason::ArithmeticOverflow);
  int64_t continuous = 0;
  reason = unique_equivalent(anchor.continuous_counts, sample.raw_counts, bound, continuous);
  if (reason != CoordinateReason::None) return invalidate_coordinate(state, reason);
  state.identity = identity;
  state.policy = policy;
  state.continuous_counts = continuous;
  state.raw_counts = sample.raw_counts;
  state.accepted_evidence = sample.evidence;
  state.valid = true;
  state.failure = CoordinateReason::None;
  return {state, CoordinateReason::None, true};
}

// Projection does not weaken limits or rate guards. A representable measured
// int64 coordinate can still be outside the continuous signed 607A command range.
inline bool coordinate_command_counts(const CoordinateReference& state, int32_t& result) noexcept {
  if (!state.valid || state.continuous_counts < std::numeric_limits<int32_t>::min() ||
      state.continuous_counts > std::numeric_limits<int32_t>::max()) return false;
  result = static_cast<int32_t>(state.continuous_counts);
  return true;
}

} // namespace rosie::rtcore
