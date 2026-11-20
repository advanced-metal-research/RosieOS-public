#include "engine/publication.hpp"
#include "coordinate_evidence.hpp"

#include <cstdlib>
#include <iostream>

using namespace rosie::rtcore;
#define REQUIRE(condition) do { if (!(condition)) { \
  std::cerr << "publication snapshot check failed at line " << __LINE__ << ": " #condition "\n"; \
  std::abort(); } } while (false)

static constexpr uint64_t ms = 1000000;
static constexpr uint64_t cut = 10000 * ms;
static constexpr uint64_t max_age = 3000 * ms;
static SampledAbsolutePosition sample(uint64_t start, uint64_t completed, int64_t counts) {
  SampledAbsolutePosition out{};
  out.valid = true;
  out.independent_origin = true;
  out.configuration_epoch = 1;
  out.home_epoch = 2;
  out.started_ns = start;
  out.bracket_completed_ns = start + (completed - start) / 3;
  out.low_completed_ns = start + 2 * (completed - start) / 3;
  out.completed_ns = completed;
  out.encoder_counts = counts;
  return out;
}

static void writer_replacement_during_formatting_is_not_future_data() {
  std::array<AtomicSnapshot<SampledAbsolutePosition>, 2> sources;
  auto old = sample(cut - 1595292289, cut - 1503450931, 100);
  auto replacement = sample(cut - 95257652, cut + 612170, 96);
  sources[0].publish(old);
  sources[1].publish(old);
  unsigned clock_calls = 0;
  const auto publication = snapshot_before_clock(sources, 2, [&] {
    // Models the producer replacing the acquisition after the reader freezes
    // its clock. A late read (the former formatter behavior) sees this sample.
    ++clock_calls;
    sources[0].publish(replacement);
    sources[1].publish(replacement);
    return cut;
  });
  REQUIRE(clock_calls == 1 && publication.time_ns == cut);
  for (unsigned i = 0; i < 2; ++i) {
    SampledAbsolutePosition late{};
    REQUIRE(sources[i].read(late));
    REQUIRE(late.valid && !late.reason);
    REQUIRE(!independent_encoder_observation_current(late, 1, cut, max_age));
    REQUIRE(publication.available[i]);
    const auto& proof = publication.values[i];
    REQUIRE(proof.encoder_counts == old.encoder_counts && proof.completed_ns == old.completed_ns);
    REQUIRE(independent_encoder_observation_current(proof, 1, publication.time_ns, max_age));
    REQUIRE(independent_acquisition_current(proof, 1, 2, publication.time_ns, max_age));
    // Repeated raw/anchor formatting uses the same frozen proof, not the source.
    REQUIRE(proof.encoder_counts == 100);
  }
  const auto next = snapshot_before_clock(sources, 2, [] { return cut + ms; });
  REQUIRE(next.values[0].encoder_counts == replacement.encoder_counts);
  REQUIRE(independent_encoder_observation_current(next.values[0], 1, next.time_ns, max_age));
  REQUIRE(independent_acquisition_current(next.values[0], 1, 2, next.time_ns, max_age));
}

static void unchanged_validity_rules_remain_authoritative() {
  std::array<AtomicSnapshot<SampledAbsolutePosition>, 1> sources;
  auto good = sample(cut - 100 * ms, cut - ms, 42);
  for (unsigned failure = 0; failure < 8; ++failure) {
    auto bad = good;
    switch (failure) {
      case 0: bad.completed_ns = cut + 1; break; // A genuinely future sample.
      case 1: bad.started_ns = cut - max_age - 1; break;
      case 2: bad.configuration_epoch = 2; break;
      case 3: bad.valid = false; break;
      case 4: bad.reason = uint32_t(SampleInvalidity::Torn); break;
      case 5: bad.low_completed_ns = bad.completed_ns + 1; break;
      case 6: bad.encoder_counts = int64_t(INT32_MAX) + 1; break;
      case 7: bad.independent_origin = false; break;
    }
    sources[0].publish(bad);
    const auto frozen = snapshot_before_clock(sources, 1, [] { return cut; });
    REQUIRE(frozen.available[0]);
    REQUIRE(!independent_encoder_observation_current(frozen.values[0], 1, frozen.time_ns, max_age));
  }
  sources[0].publish(good);
  const auto frozen = snapshot_before_clock(sources, 1, [] { return cut; });
  // Actual HM frame advancement remains meaningful: raw diagnostic proof does
  // not bind Home, while the anchor offer must match the current command frame.
  REQUIRE(independent_encoder_observation_current(frozen.values[0], 1, frozen.time_ns, max_age));
  REQUIRE(!independent_acquisition_current(frozen.values[0], 1, 3, frozen.time_ns, max_age));
  REQUIRE(!independent_encoder_observation_current(frozen.values[0], 2, frozen.time_ns, max_age));
}

static void failed_or_unselected_reads_never_reuse_a_prior_proof() {
  std::array<AtomicSnapshot<SampledAbsolutePosition>, 2> sources;
  sources[0].publish(sample(cut - 100 * ms, cut - ms, 7));
  const auto before = snapshot_before_clock(sources, 2, [&] {
    sources[1].publish(sample(cut - 100 * ms, cut - ms, 8));
    return cut;
  });
  REQUIRE(before.available[0] && !before.available[1]);
  REQUIRE(!before.values[1].valid && before.values[1].completed_ns == 0);
  const auto next = snapshot_before_clock(sources, 1, [] { return cut; });
  REQUIRE(next.available[0] && !next.available[1]);
  REQUIRE(next.values[1].completed_ns == 0);
  const auto all = snapshot_before_clock(sources, 2, [] { return cut; });
  REQUIRE(all.available[1] && all.values[1].encoder_counts == 8);
}

int main() {
  writer_replacement_during_formatting_is_not_future_data();
  unchanged_validity_rules_remain_authoritative();
  failed_or_unselected_reads_never_reuse_a_prior_proof();
  std::cout << "publication snapshot: all checks passed\n";
}
