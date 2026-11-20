#include "coordinate_evidence.hpp"

#include <cstdio>
#include <cstdlib>

using namespace rosie::rtcore;

namespace {
void expect(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::abort();
  }
}

CoordinatePolicy policy() {
  CoordinatePolicy p{};
  p.maximum_physical_speed_counts_per_second = 2000000;
  p.maximum_tracking_speed_counts_per_second = 2000000;
  p.maximum_acquisition_age_ns = 3000000000;
  p.maximum_feedback_error_counts = 16;
  p.maximum_anchor_error_counts = 16;
  p.qualified = true;
  return p;
}

SampledAbsolutePosition sample(int64_t encoder) {
  SampledAbsolutePosition s{};
  s.configuration_epoch = 1;
  s.home_epoch = 1;
  s.cycle = 1;
  s.started_ns = 100;
  s.bracket_completed_ns = 110;
  s.low_completed_ns = 120;
  s.completed_ns = 130;
  s.encoder_counts = encoder;
  s.valid = 1;
  s.independent_origin = true;
  return s;
}

void history_is_continuous_across_multiple_turns() {
  SampledPdoHistory history;
  history.configure(1000000000, 100000000);
  auto p = policy();
  // Unverified synthetic rate: each 262144-count step in 100 ms remains below
  // the 300016-count allowance, so any generation change is a real regression.
  p.maximum_physical_speed_counts_per_second = 3000000;
  history.observe(1, 1, 1, 100000000, 0, true, p);
  const auto generation = history.latest.generation;
  history.observe(1, 1, 2, 200000000, 131072, true, p);
  history.observe(1, 1, 3, 300000000, 262144, true, p);
  history.observe(1, 1, 4, 400000000, 0, true, p);
  history.observe(1, 1, 5, 500000000, -262144, true, p);
  expect(!history.failed, "continuous +/-2-turn history failed");
  expect(history.latest.generation == generation,
         "continuous +/-2-turn history silently reset its generation");
  expect(history.latest.counts == -262144, "history changed exact continuous counts");
}

void bounded_history_gap_fences_audits_without_losing_counts() {
  SampledPdoHistory close_receipts;
  close_receipts.configure(1000000000, 1000000);
  const auto p = policy();
  close_receipts.observe(1, 1, 1, 100000000, 0, true, p);
  close_receipts.observe(1, 1, 2, 100200000, 1500, true, p);
  expect(!close_receipts.failed && close_receipts.latest.counts == 1500,
         "configured cycle separation did not cover close host observations");

  SampledPdoHistory history;
  history.configure(1000000000, 100000000);
  history.observe(1, 1, 1, 100000000, 0, true, p);
  const auto before = history.latest.generation;
  history.observe(1, 1, 3, 102000000, 3000, true, p);
  expect(!history.failed && history.latest.counts == 3000 &&
             history.latest.generation > before,
         "bounded skipped observation lost exact tracking");
  int64_t minimum = 0, maximum = 0;
  expect(!history.query_history_interval(1, 3, minimum, maximum),
         "audit interval crossed an unobserved cycle");

  SampledPdoHistory impossible;
  impossible.configure(1000000000, 1000000);
  impossible.observe(1, 1, 1, 100000000, 0, true, p);
  impossible.bound = true;
  impossible.observe(1, 1, 3, 102000000, 10000, true, p);
  expect(impossible.failed, "physically impossible skipped observation retained trust");
}

void whole_turn_anchor_disagreements_refuse() {
  const auto s = sample(131072);
  expect(!sampled_anchor_matches(s, 0, 0, 0, 16),
         "restore accepted a one-turn encoder disagreement");

  int64_t command = 0;
  expect(!retained_home_anchor_matches(s, 0, 0, 0, 0, 0, 16, command),
         "retained Home aligned a one-turn disagreement");
  expect(retained_home_anchor_matches(s, 131072, 0, 0, 0, 0, 16, command) &&
             command == 131072,
         "retained Home rejected exact continuous displacement");
}

void home_preserves_continuous_post_hm35_displacement() {
  SampledPdoHistory history;
  history.configure(1000000000, 100000000);
  const auto p = policy();
  history.observe(1, 1, 1, 100, 131072, true, p);
  auto s = sample(500000);
  SampledAbsolutePosition retained{};
  expect(history.complete_independent_home(s, retained, p, 140, 0, false),
         "Home rejected witnessed post-HM35 displacement");
  expect(history.command_counts == 131072, "Home changed the exact command frame");
}

void ordinary_coordinate_samples_are_exact() {
  CoordinateReference state{};
  state.identity = {1, 1, 1, {}};
  state.policy = policy();
  state.continuous_counts = 0;
  state.raw_counts = 0;
  state.valid = true;
  state.accepted_evidence = {1, 100000000, 0, 0, true, true, true};
  state.last_seen_receipt_id = 1;
  state.last_seen_observation_ns = 100000000;
  CoordinateSample next{131072, {2, 300000000, 0, 0, true, true, true}};
  const auto accepted = observe_coordinate(state, state.identity, state.policy, next);
  expect(accepted.accepted && accepted.next.continuous_counts == 131072,
         "continuous PDO sample was folded modulo one turn");
}
}  // namespace

int main() {
  history_is_continuous_across_multiple_turns();
  bounded_history_gap_fences_audits_without_losing_counts();
  whole_turn_anchor_disagreements_refuse();
  home_preserves_continuous_post_hm35_displacement();
  ordinary_coordinate_samples_are_exact();
  std::puts("coordinate evidence tests passed");
}
