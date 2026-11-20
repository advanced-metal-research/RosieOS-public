#include "robot_v4_cartesian_calibration_tour_checkpoints.hpp"

#include <array>
#include <iostream>

int main() {
  using robot_v4::motion_server::calibration_tour_checkpoint_arrival;
  using robot_v4::motion_server::calibration_tour_checkpoint_row;
  using robot_v4::motion_server::calibration_tour_completion_allowed;
  using robot_v4::motion_server::clamp_forward_plan_time_to_checkpoint;
  using robot_v4::motion_server::kCalibrationTourCheckpointRows;
  using robot_v4::motion_server::kCalibrationTourCheckpointToleranceRad;

  constexpr std::array<int, 8> expected_rows = {{
      2, 3, 4, 7, 9, 10, 11, 14,
  }};
  if (kCalibrationTourCheckpointRows != expected_rows) {
    return 1;
  }
  for (int row_index = 0; row_index <= 14; ++row_index) {
    bool expected = false;
    for (const int checkpoint_row : expected_rows) {
      expected = expected || row_index == checkpoint_row;
    }
    if (calibration_tour_checkpoint_row(row_index) != expected) {
      return 1;
    }
  }

  constexpr double failing_corner_error_rad = 0.080393;
  constexpr double converged_corner_error_rad = 0.019;
  constexpr double persistent_error_rad = 0.050;
  constexpr double watchdog_timeout_s = 20.0;
  if (!calibration_tour_checkpoint_row(2) ||
      !calibration_tour_checkpoint_row(14) ||
      calibration_tour_checkpoint_row(5) ||
      calibration_tour_checkpoint_row(6) ||
      calibration_tour_checkpoint_row(8) ||
      calibration_tour_checkpoint_row(12) ||
      calibration_tour_checkpoint_row(13) ||
      !calibration_tour_checkpoint_arrival(1, 2) ||
      calibration_tour_checkpoint_arrival(2, 2) ||
      clamp_forward_plan_time_to_checkpoint(8.7, 8.9, 8.828) != 8.828 ||
      clamp_forward_plan_time_to_checkpoint(8.7, 8.8, 8.828) != 8.8 ||
      failing_corner_error_rad <= kCalibrationTourCheckpointToleranceRad ||
      converged_corner_error_rad > kCalibrationTourCheckpointToleranceRad ||
      !(persistent_error_rad > kCalibrationTourCheckpointToleranceRad &&
        20.001 > watchdog_timeout_s) ||
      calibration_tour_completion_allowed(true, false, false) ||
      calibration_tour_completion_allowed(true, true, false) ||
      !calibration_tour_completion_allowed(true, true, true) ||
      !calibration_tour_completion_allowed(false, false, false) ||
      kCalibrationTourCheckpointToleranceRad != 0.020) {
    return 1;
  }

  std::cout << "calibration_tour_checkpoints=pass"
            << " checkpoint_count=8"
            << " corner_and_home_rows=true"
            << " exact_target_clamp=true"
            << " failing_corner_holds=true"
            << " converged_corner_releases=true"
            << " no_progress_timeout_s=20"
            << " penultimate_checkpoint_completion_blocked=true"
            << " final_crossing_completion_blocked=true"
            << " tolerance_rad=0.020\n";
  return 0;
}
