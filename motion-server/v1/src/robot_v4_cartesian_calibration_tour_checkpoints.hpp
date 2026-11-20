#pragma once

#include <array>
#include <cstddef>

namespace robot_v4::motion_server {

constexpr double kCalibrationTourCheckpointToleranceRad = 0.020;
constexpr std::array<int, 8> kCalibrationTourCheckpointRows = {{
    2, 3, 4, 7, 9, 10, 11, 14,
}};

constexpr bool calibration_tour_checkpoint_row(int row_index) {
  for (const int checkpoint_row : kCalibrationTourCheckpointRows) {
    if (row_index == checkpoint_row) {
      return true;
    }
  }
  return false;
}

constexpr bool calibration_tour_checkpoint_arrival(int previous_row_index,
                                                   int row_index) {
  return previous_row_index != row_index &&
         calibration_tour_checkpoint_row(row_index);
}

constexpr double clamp_forward_plan_time_to_checkpoint(
    double current_time_s,
    double proposed_time_s,
    double checkpoint_time_s) {
  return checkpoint_time_s > current_time_s &&
                 proposed_time_s > checkpoint_time_s
             ? checkpoint_time_s
             : proposed_time_s;
}

constexpr bool calibration_tour_completion_allowed(
    bool checkpoint_holds_enabled,
    bool checkpoint_was_active_at_tick_start,
    bool terminal_checkpoint_was_active_at_tick_start) {
  // A canonical plan may not complete or reloop on the control tick that
  // first advances into its final checkpoint. One subsequent tick must
  // observe the terminal checkpoint under the strict feedback gate. An
  // earlier checkpoint cannot satisfy this guard under a large simulated
  // control step.
  return !checkpoint_holds_enabled ||
         (checkpoint_was_active_at_tick_start &&
          terminal_checkpoint_was_active_at_tick_start);
}

}  // namespace robot_v4::motion_server
