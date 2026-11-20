#include "coordinate_model.hpp"

#include <cmath>
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
}  // namespace

int main() {
  AxisCoordinateProfile profile{};
  profile.gear_ratio = 100;
  profile.sign = 1;
  const auto model = AxisCoordinateModel::from_encoder(131072, profile);

  const auto positive = model.logical_to_counts(4 * M_PI);
  const auto negative = model.logical_to_counts(-4 * M_PI);
  expect(positive.valid && positive.counts == 26214400,
         "+720 degrees did not retain its continuous turn count");
  expect(negative.valid && negative.counts == -26214400,
         "-720 degrees did not retain its continuous turn count");

  const auto logical = model.counts_to_logical(positive.counts, true);
  expect(logical.valid && std::abs(logical.logical - 4 * M_PI) < 1e-12,
         "continuous count round trip changed the joint position");

  const auto overflow = model.project_command(double(INT32_MAX) + 1);
  expect(!overflow.valid && overflow.reason == CoordinateMappingReason::Overflow,
         "signed-32 command overflow was admitted");

  AxisLimits limits{};
  limits.valid = true;
  limits.min_q = -4 * M_PI;
  limits.max_q = 4 * M_PI;
  const auto held = model.reconcile(gradient::ds402::State::SwitchOnDisabled,
      true, true, 26214400, true, 26214400, limits);
  expect(held.disposition == ReferenceDisposition::Reconciled &&
             held.target == 26214400,
         "continuous inhibited feedback did not reconcile exactly");

  std::puts("coordinate model tests passed");
}
