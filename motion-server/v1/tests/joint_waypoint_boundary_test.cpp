#define main production_main
#include "../src/robot_v4_cartesian_daemon.cpp"
#undef main
#include <cassert>

int main() {
  const std::string report = "{\"program_id\":\"chunk-test\",\"plan_request_revision\":7,\"padding\":\"" +
      std::string(1100000, 'x') + "\"}";
  std::vector<std::string> frames;
  std::string diagnostic;
  assert(motion_server_plan_transport_frames(report, 65536, &frames, &diagnostic));
  assert(frames.size() > 1);
  SpreadsheetPlanRow from, to;
  from.row_index = 0;
  to.row_index = 1;
  to.move_type = "joint_ptp";
  PreloadedTesseractPlan plan;
  plan.valid = plan.server_validated = true;
  plan.samples.push_back(PreloadedPlanSample{});
  plan.samples.front().row_index = from.row_index;
  // Match append_segment: interior samples retain the source row, arrival
  // takes the destination row. All samples use the target's movement type.
  for (int i = 1; i <= 4; ++i) {
    const auto point = interpolate_spreadsheet_plan_point(
        from, to, i == 4 ? to.row_index : from.row_index, i, i / 4.0);
    PreloadedPlanSample sample;
    sample.row_index = point.row_index;
    sample.joint_ptp = point.move_type == "joint_ptp";
    plan.samples.push_back(sample);
  }
  assert(preloaded_sample_is_joint_waypoint(plan, 0));
  for (int i = 1; i < 4; ++i)
    assert(!preloaded_sample_is_joint_waypoint(plan, i));
  assert(preloaded_sample_is_joint_waypoint(plan, 4));
  PreloadedPlanSample cartesian;
  cartesian.row_index = 1;
  plan.samples.push_back(cartesian);
  assert(preloaded_sample_is_joint_waypoint(plan, 4));
  assert(!preloaded_sample_is_joint_waypoint(plan, 5));
  plan.server_validated = false;
  assert(!preloaded_sample_is_joint_waypoint(plan, 4));

  plan.valid = plan.server_validated = true;
  plan.samples.assign(2, PreloadedPlanSample{});
  plan.samples[1].time_s = 1.0;
  double hold[kLiveAxisCount]{}, lag[kLiveAxisCount]{}, correction[kLiveAxisCount]{};
  lag[0] = -0.01;
  correction[0] = 0.01 / PLAN_PLAYBACK_TRACK_HORIZON_S;
  plan_playback_trajectory_feedforward(plan, 1, 0.2, 0.4, 0.01,
      hold, hold, lag, correction);
  assert(std::abs(correction[0] - 0.01 / PLAN_PLAYBACK_TRACK_HORIZON_S) < 1e-12);
  lag[0] = 0.0;
  plan_playback_trajectory_feedforward(plan, 1, 0.2, 0.4, 0.01,
      hold, hold, lag, correction);
  assert(correction[0] == 0.0);
  lag[0] = -0.01;
  plan_playback_trajectory_feedforward(plan, 0, 0.2, 0.4, 0.01,
      hold, hold, lag, correction);
  assert(correction[0] == 0.0);

  // Non-positive intervals are excluded by the overlap predicate before
  // interval validation; they must not erase a valid overlapping stroke.
  for (double middle_time : {0.0, -0.1}) {
    plan.valid = plan.server_validated = true;
    plan.samples.assign(3, PreloadedPlanSample{});
    plan.samples[0].time_s = 0.0;
    plan.samples[1].time_s = middle_time;
    plan.samples[2].time_s = 1.0;
    plan.samples[2].q_rad[0] = 0.1;
    double now[kLiveAxisCount]{}, horizon[kLiveAxisCount]{}, measured[kLiveAxisCount]{};
    double requested[kLiveAxisCount]{};
    horizon[0] = 0.01;
    plan_playback_trajectory_feedforward(plan, 1, 0.0, 0.1, 0.01,
        now, horizon, measured, requested);
    assert(std::isfinite(requested[0]) && requested[0] > 0.0);
  }
}
