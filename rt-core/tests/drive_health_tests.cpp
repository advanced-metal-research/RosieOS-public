// Drive health: monitor publication and the torque / following-error window
// (include/drive_health.hpp, docs/decisions/0001).
#include "../include/drive_health.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using rosie::rtcore::DriveLoadWindow;
using rosie::rtcore::MonitorSlot;
using rosie::rtcore::monitor_value;

static void test_monitor_slot_publishes_value_with_its_time() {
  MonitorSlot slot;
  int32_t value = -1;
  uint64_t at = 0;
  assert(!slot.snapshot(&value, &at));  // never read
  slot.publish(345, 7'000'000'000);
  assert(slot.snapshot(&value, &at) && value == 345 && at == 7'000'000'000);
  slot.publish(-12, 0);  // a zero clock still reads as published
  assert(slot.snapshot(&value, &at) && value == -12 && at == 1);
  // A6-EC heatsink temperature: 0.1 degC per count.
  assert(std::fabs(monitor_value(345, 1) - 34.5) < 1e-9);
  assert(std::fabs(monitor_value(-7, 0) + 7.0) < 1e-9);
}

static void test_window_rms_and_peaks_over_the_last_second() {
  DriveLoadWindow w;
  // 1 kHz for 1 s: torque alternates +-300 per-mille, error peaks at -900.
  for (uint64_t i = 0; i < 1000; ++i)
    w.add(5'000'000'000 + i * 1'000'000, true, i % 2 ? 300 : -300, true, i == 500 ? -900 : 10);
  auto s = w.summary();
  assert(s.torque_samples == 1000 && s.error_samples == 1000);
  assert(std::fabs(s.torque_rms - 300.0) < 1e-9);
  assert(s.torque_peak == 300 && s.error_peak == 900);
  assert(s.window_ns == 1'000'000'000);
}

static void test_window_forgets_bins_older_than_a_second() {
  DriveLoadWindow w;
  w.add(1'000'000'000, true, 900, true, 5000);  // a spike, then 1.2 s of quiet
  for (uint64_t i = 1; i <= 1200; ++i) w.add(1'000'000'000 + i * 1'000'000, true, 100, true, 1);
  const auto s = w.summary();
  assert(s.torque_peak == 100 && s.error_peak == 1);
  assert(s.torque_samples <= 1000 && s.torque_samples >= 900);
}

static void test_invalid_samples_are_not_counted() {
  DriveLoadWindow w;
  w.add(2'000'000'000, false, 999, false, 999);
  w.add(2'000'500'000, true, 50, false, 999);
  const auto s = w.summary();
  assert(s.torque_samples == 1 && s.error_samples == 0 && s.torque_peak == 50 && s.error_peak == 0);
  w.clear();
  assert(w.summary().torque_samples == 0 && w.newest == 0);
}

int main() {
  test_monitor_slot_publishes_value_with_its_time();
  test_window_rms_and_peaks_over_the_last_second();
  test_window_forgets_bins_older_than_a_second();
  test_invalid_samples_are_not_counted();
  std::puts("drive health tests passed");
}
