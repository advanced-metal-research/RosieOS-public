#include <cassert>
#include <chrono>
#include <string>

#define main robot_v4_cartesian_daemon_embedded_main
#include "../../../../../../motion-server/v1/src/robot_v4_cartesian_daemon.cpp"
#undef main

int main() {
  MotionServerQuiescenceReceipt receipt;
  receipt.valid = true;
  receipt.pair_authority_revoke_count = 7;
  receipt.authority.runtime_mode = RTCORE_RUNTIME_MODE_LIVE;
  receipt.authority.runtime_mode_epoch = 4242;
  RTCorePairConfigBinding& pair = receipt.authority.pair_config;
  pair.valid = true;
  pair.robot_model_id = "rosie_1420_v1";
  pair.revision = 19;
  pair.snapshot_sha256 = "pair-sha";
  pair.generated_urdf_sha256 = "urdf-sha";
  pair.home_profile_key = "rosie-home";
  pair.activation_epoch = 31;
  pair.home_epoch = 37;
  pair.mapped_axis_mask = 0x3f;
  pair.home_restore_pending_axis_mask_present = true;
  pair.home_restore_pending_axis_mask = 0;
  pair.home_valid_axis_mask = 0x3f;

  const std::string request =
      "{\"runtime_mode\":\"live\","
      "\"runtime_mode_epoch\":4242,"
      "\"robot_model_id\":\"rosie_1420_v1\","
      "\"pair_revision\":19,"
      "\"pair_config_sha256\":\"pair-sha\","
      "\"generated_urdf_sha256\":\"urdf-sha\","
      "\"home_profile_key\":\"rosie-home\","
      "\"activation_epoch\":31,"
      "\"home_epoch\":37,"
      "\"mapped_axis_mask\":63,"
      "\"home_restore_pending_axis_mask\":0,"
      "\"home_valid_axis_mask\":63}";
  std::string error;
  assert(motion_server_quiescence_request_matches(request, receipt, &error));
  assert(error.empty());

  std::string stale_request = request;
  const std::size_t home_epoch = stale_request.find("\"home_epoch\":37");
  assert(home_epoch != std::string::npos);
  stale_request.replace(home_epoch, std::string("\"home_epoch\":37").size(),
                        "\"home_epoch\":38");
  assert(!motion_server_quiescence_request_matches(
      stale_request, receipt, &error));
  assert(!error.empty());

  pair.home_restore_pending_axis_mask = 1;
  assert(!motion_server_quiescence_request_matches(request, receipt, &error));
  pair.home_restore_pending_axis_mask = 0;

  MotionServerQuiescenceBarrier barrier;
  MotionServerQuiescenceReceipt observed;
  std::thread ready_owner([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    barrier.observe(true, receipt.authority,
                    receipt.pair_authority_revoke_count);
  });
  assert(barrier.request(&observed));
  ready_owner.join();
  assert(observed.valid);
  assert(observed.authority.runtime_mode_epoch == 4242);
  assert(observed.pair_authority_revoke_count == 7);

  std::thread blocked_owner([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    barrier.observe(false, receipt.authority,
                    receipt.pair_authority_revoke_count);
  });
  assert(!barrier.request(&observed));
  blocked_owner.join();
  return 0;
}
