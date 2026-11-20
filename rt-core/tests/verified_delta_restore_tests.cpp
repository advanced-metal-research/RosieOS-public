#include "verified_delta_restore.hpp"

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

DeltaAcquisition acquisition(int32_t command, int32_t encoder) {
  DeltaAcquisition a{};
  a.receipt_serial = 1;
  a.configuration_epoch = 1;
  a.home_epoch = 1;
  a.axis_coordinate_epoch = 1;
  a.drive_frame_epoch = 1;
  a.command_counts = command;
  a.encoder_counts = encoder;
  a.drive_home_offset_raw = 0;
  a.started_ns = 10;
  a.completed_ns = 20;
  a.pdo_receipt = 1;
  return a;
}
}  // namespace

int main() {
  int64_t command_delta = 0, encoder_delta = 0, aligned = 0;
  int32_t drive_offset = 0;

  auto exact = acquisition(262144, 262144);
  expect(qualify_verified_delta_anchor_v3(0, 0, 0, 1, exact, 16,
             command_delta, encoder_delta, aligned, drive_offset) ==
             DeltaRestoreReason::None,
         "exact two-turn verified delta was refused");
  expect(command_delta == 262144 && encoder_delta == 262144 && aligned == 262144,
         "verified delta changed exact continuous counts");

  auto disagreement = acquisition(131072, 0);
  expect(qualify_verified_delta_anchor_v3(0, 0, 0, 1, disagreement, 16,
             command_delta, encoder_delta, aligned, drive_offset) ==
             DeltaRestoreReason::DeltaMismatch,
         "verified delta aligned a whole-turn disagreement");

  auto within_tolerance = acquisition(262150, 262144);
  expect(qualify_verified_delta_anchor_v3(0, 0, 0, 1, within_tolerance, 16,
             command_delta, encoder_delta, aligned, drive_offset) ==
             DeltaRestoreReason::None,
         "verified delta rejected an exact-frame tolerance");

  std::puts("verified delta restore tests passed");
}
