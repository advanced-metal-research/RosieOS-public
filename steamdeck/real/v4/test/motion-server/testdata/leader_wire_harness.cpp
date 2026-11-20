#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#define main robot_v4_cartesian_daemon_embedded_main
#include "../../../../../../motion-server/v1/src/robot_v4_cartesian_daemon.cpp"
#undef main

int main() {
  std::vector<std::string> controller_roles;
  assert(parse_exact_json_string_array_value(
      "[\"steamdeck\",\"offline-programming\"]",
      &controller_roles));
  assert(controller_roles.size() == 2);
  assert(!parse_exact_json_string_array_value(
      "[\"steamdeck\",]", &controller_roles));
  assert(!parse_exact_json_string_array_value(
      "[\"steamdeck\",7]", &controller_roles));
  SteamDeckJoystickIntent controller_intent;
  controller_intent.tool_id = "offline-programming:server-1";
  assert(sender_id_from_intent(controller_intent) ==
         "offline-programming:server-1");
  controller_intent.tool_id = "steamdeck-secondary";
  assert(sender_id_from_intent(controller_intent) ==
         "steamdeck:steamdeck-secondary");

  RuntimeStatus status;
  status.motion_owner_status_seq = 77;
  status.leader_id = "steamdeck:steamdeck-secondary";
  status.leader_boot_id = "boot-2";
  status.leader_lease_id = "lease-2";
  status.leader_fence_epoch = 99;
  status.leader_campaign_generation = 7;
  status.leader_lease_fresh = true;
  status.leader_expires_in_ms = 1499;
  status.leader_transition_pending = false;
  status.latest_leader_result = "acquired";
  status.latest_leader_request_id = "request-2";
  status.latest_leader_campaign_generation = 11;

  const std::vector<unsigned char> v4 =
      encode_motion_owner_status_packet(status);
  const std::vector<unsigned char> v5 =
      encode_motion_owner_leader_status_packet_v5(status);
  assert(v4.size() == 576);
  assert(v4[2] == 4);
  assert(v5.size() == 928);
  assert(v5[2] == 5);
  for (std::size_t index = 0; index < v4.size(); ++index) {
    if (index != 2) {
      assert(v5[index] == v4[index]);
    }
  }
  assert(v5[576] == 1);
  assert(read_u32_be(v5.data(), 580) == 1499);
  assert(read_u64_be(v5.data(), 584) == 99);
  assert(std::string(reinterpret_cast<const char*>(v5.data() + 592)) ==
         status.leader_id);
  assert(std::string(reinterpret_cast<const char*>(v5.data() + 656)) ==
         status.leader_boot_id);
  assert(std::string(reinterpret_cast<const char*>(v5.data() + 720)) ==
         status.leader_lease_id);
  assert(std::string(reinterpret_cast<const char*>(v5.data() + 784)) ==
         status.latest_leader_result);
  assert(std::string(reinterpret_cast<const char*>(v5.data() + 848)) ==
         status.latest_leader_request_id);
  assert(read_u64_be(v5.data(), 912) == 11);
  assert(read_u64_be(v5.data(), 920) == 7);

  LeaderLeaseRequest decoded_cancel;
  assert(decode_leader_lease_request(
      "{\"command\":\"leader_cancel\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"boot-2\","
      "\"request_id\":\"cancel-2\","
      "\"campaign_generation\":7}",
      &decoded_cancel));
  assert(decoded_cancel.command == "leader_cancel");
  assert(decoded_cancel.campaign_generation == 7);
  assert(!decode_leader_lease_request(
      "{\"command\":\"leader_cancel\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"boot-2\","
      "\"request_id\":\"cancel-2\","
      "\"campaign_generation\":7,\"ttl_ms\":1500}",
      &decoded_cancel));
  assert(!decode_leader_lease_request(
      "{\"command\":\"leader_acquire\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"boot-2\","
      "\"request_id\":\"acquire-legacy\",\"ttl_ms\":1500}",
      &decoded_cancel));

  const std::string tool_id = "steamdeck-secondary";
  const std::string lease_id = "lease-v2";
  std::vector<unsigned char> joystick(
      kJoystickIntentHeaderBytes + tool_id.size() + 8 + 1 +
          lease_id.size(),
      0);
  put_u16_be(&joystick, 0, JOYSTICK_INTENT_PACKET_MAGIC);
  joystick[2] = JOYSTICK_INTENT_PACKET_VERSION_V2;
  put_u64_be(&joystick, 3, 123);
  put_u32_be(&joystick, 11, 7);
  joystick[15] = JOYSTICK_INTENT_FRAME_JOINT;
  joystick[16] = static_cast<unsigned char>(tool_id.size());
  joystick[45] = 1;
  std::memcpy(joystick.data() + kJoystickIntentHeaderBytes,
              tool_id.data(), tool_id.size());
  std::size_t joystick_fence =
      kJoystickIntentHeaderBytes + tool_id.size();
  put_u64_be(&joystick, joystick_fence, 99);
  joystick_fence += 8;
  joystick[joystick_fence++] =
      static_cast<unsigned char>(lease_id.size());
  std::memcpy(joystick.data() + joystick_fence, lease_id.data(),
              lease_id.size());
  SteamDeckJoystickIntent decoded_joystick;
  assert(decode_steamdeck_joystick_intent(
      joystick.data(), joystick.size(), &decoded_joystick));
  assert(decoded_joystick.tool_id == tool_id);
  assert(decoded_joystick.leader_fence_epoch == 99);
  assert(decoded_joystick.leader_lease_id == lease_id);

  std::vector<unsigned char> malformed = joystick;
  malformed.pop_back();
  assert(!decode_steamdeck_joystick_intent(
      malformed.data(), malformed.size(), &decoded_joystick));

  LeaderAwareRobotCommandQueue queue;
  RobotCommandMessage play;
  play.kind = "play";
  play.command_id = "old-play";
  assert(queue.push(std::move(play)));
  RobotCommandMessage acquire;
  acquire.kind = "leader_acquire";
  acquire.command_id = "queued-acquire";
  assert(queue.push(std::move(acquire)));
  RobotCommandMessage stop;
  stop.kind = "stop";
  stop.command_id = "queued-stop";
  assert(queue.push(std::move(stop)));
  RobotCommandMessage path_hold;
  path_hold.kind = "path_scrub";
  path_hold.command_id = "queued-path-hold";
  path_hold.has_direction = true;
  path_hold.direction = 0;
  assert(queue.push(std::move(path_hold)));
  assert(queue.discard_energizing() == 1);
  RobotCommandMessage survivor;
  assert(queue.pop(&survivor));
  assert(survivor.command_id == "queued-acquire");
  assert(queue.pop(&survivor));
  assert(survivor.command_id == "queued-stop");
  assert(queue.pop(&survivor));
  assert(survivor.command_id == "queued-path-hold");
  assert(!queue.pop(&survivor));

  LeaderAwareRobotCommandQueue overflow_queue;
  for (int index = 0; index < 48; ++index) {
    RobotCommandMessage ordinary;
    ordinary.kind = "plan_request";
    ordinary.command_id = "ordinary-" + std::to_string(index);
    assert(overflow_queue.push(std::move(ordinary)));
  }
  RobotCommandMessage rejected_ordinary;
  rejected_ordinary.kind = "plan_request";
  rejected_ordinary.command_id = "ordinary-rejected";
  assert(!overflow_queue.push(std::move(rejected_ordinary)));

  std::vector<std::string> protected_ids;
  for (int index = 0; index < 14; ++index) {
    RobotCommandMessage authority;
    authority.kind =
        index == 0 ? "leader_cancel" : "leader_renew";
    authority.command_id = "authority-" + std::to_string(index);
    protected_ids.push_back(authority.command_id);
    assert(overflow_queue.push(std::move(authority)));
  }
  RobotCommandMessage overflow_stop;
  overflow_stop.kind = "stop";
  overflow_stop.command_id = "overflow-stop";
  protected_ids.push_back(overflow_stop.command_id);
  assert(overflow_queue.push(std::move(overflow_stop)));
  RobotCommandMessage overflow_hold;
  overflow_hold.kind = "path_scrub";
  overflow_hold.command_id = "overflow-hold";
  overflow_hold.has_direction = true;
  overflow_hold.direction = 0;
  protected_ids.push_back(overflow_hold.command_id);
  assert(overflow_queue.push(std::move(overflow_hold)));

  RobotCommandMessage explicitly_rejected_stop;
  explicitly_rejected_stop.kind = "stop";
  explicitly_rejected_stop.command_id = "full-stop-rejected";
  assert(!overflow_queue.push(std::move(explicitly_rejected_stop)));

  for (int index = 0; index < 48; ++index) {
    assert(overflow_queue.pop(&survivor));
    assert(survivor.command_id ==
           "ordinary-" + std::to_string(index));
  }
  for (const std::string& protected_id : protected_ids) {
    assert(overflow_queue.pop(&survivor));
    assert(survivor.command_id == protected_id);
  }
  assert(!overflow_queue.pop(&survivor));

  // A processed acquire retains two bounded exit reservations. Fill every
  // remaining capacity unit with already-accepted ordinary/protected work;
  // its exact cancel and release must still enter without evicting any ACKed
  // command. Exit barriers are prioritized FIFO ahead of that work.
  LeaderAwareRobotCommandQueue saturated_exit_queue;
  RobotCommandMessage reserved_acquire;
  reserved_acquire.kind = "leader_acquire";
  reserved_acquire.command_id = "reserved-acquire";
  reserved_acquire.sender_id = "steamdeck:steamdeck-secondary";
  reserved_acquire.raw_payload =
      "{\"command\":\"leader_acquire\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"reserved-boot\","
      "\"request_id\":\"reserved-acquire\","
      "\"campaign_generation\":9,\"ttl_ms\":1500}";
  assert(saturated_exit_queue.push(std::move(reserved_acquire)));
  assert(saturated_exit_queue.pop(&survivor));
  LeaderLeaseRequest reserved_request;
  assert(decode_leader_lease_request(
      survivor.raw_payload, &reserved_request));
  SteamDeckLeaderAuthority saturated_authority(
      /*server_epoch_seed=*/1000,
      /*injected_lease_nonce=*/0x1234);
  const auto saturated_at =
      SteamDeckLeaderAuthority::TimePoint{} +
      std::chrono::seconds(1);
  const LeaderLeaseAction reserved_action =
      saturated_authority.begin_request(
          reserved_request, saturated_at);
  assert(reserved_action.accepted);
  saturated_authority.finish_transition(true, saturated_at);
  const LeaderLeaseStatus reserved_status =
      saturated_authority.status(saturated_at);
  assert(reserved_status.leader_lease_fresh);

  for (int index = 0; index < 48; ++index) {
    RobotCommandMessage ordinary;
    ordinary.kind = "plan_request";
    ordinary.command_id =
        "saturated-ordinary-" + std::to_string(index);
    assert(saturated_exit_queue.push(std::move(ordinary)));
  }
  for (int index = 0; index < 14; ++index) {
    RobotCommandMessage protected_stop;
    protected_stop.kind = "stop";
    protected_stop.command_id =
        "saturated-stop-" + std::to_string(index);
    assert(saturated_exit_queue.push(std::move(protected_stop)));
  }
  RobotCommandMessage rejected_when_saturated;
  rejected_when_saturated.kind = "stop";
  rejected_when_saturated.command_id =
      "saturated-stop-rejected";
  assert(!saturated_exit_queue.push(
      std::move(rejected_when_saturated)));

  RobotCommandMessage reserved_cancel;
  reserved_cancel.kind = "leader_cancel";
  reserved_cancel.command_id = "reserved-cancel";
  reserved_cancel.sender_id = "steamdeck:steamdeck-secondary";
  reserved_cancel.raw_payload =
      "{\"command\":\"leader_cancel\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"reserved-boot\","
      "\"request_id\":\"reserved-cancel\","
      "\"campaign_generation\":9}";
  assert(saturated_exit_queue.push(std::move(reserved_cancel)));

  RobotCommandMessage reserved_release;
  reserved_release.kind = "leader_release";
  reserved_release.command_id = "reserved-release";
  reserved_release.sender_id = "steamdeck:steamdeck-secondary";
  reserved_release.raw_payload =
      "{\"command\":\"leader_release\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"reserved-boot\","
      "\"request_id\":\"reserved-release\","
      "\"campaign_generation\":9,\"lease_id\":\"" +
      reserved_status.leader_lease_id +
      "\",\"fence_epoch\":" +
      std::to_string(reserved_status.leader_fence_epoch) + "}";
  assert(saturated_exit_queue.push(std::move(reserved_release)));

  assert(saturated_exit_queue.pop(&survivor));
  assert(survivor.command_id == "reserved-cancel");
  LeaderLeaseRequest queued_cancel;
  assert(decode_leader_lease_request(
      survivor.raw_payload, &queued_cancel));
  const LeaderLeaseAction queued_cancel_action =
      saturated_authority.begin_request(
          queued_cancel, saturated_at);
  assert(queued_cancel_action.accepted);
  saturated_authority.finish_transition(true, saturated_at);
  assert(!saturated_authority.status(saturated_at)
              .leader_lease_fresh);

  assert(saturated_exit_queue.pop(&survivor));
  assert(survivor.command_id == "reserved-release");
  LeaderLeaseRequest queued_release;
  assert(decode_leader_lease_request(
      survivor.raw_payload, &queued_release));
  assert(!saturated_authority.begin_request(
               queued_release, saturated_at)
              .accepted);

  LeaderLeaseRequest delayed_reserved = reserved_request;
  delayed_reserved.request_id = "reserved-acquire-delayed";
  assert(!saturated_authority.begin_request(
               delayed_reserved,
               saturated_at + std::chrono::milliseconds(1))
              .accepted);
  assert(saturated_authority.status(
             saturated_at + std::chrono::milliseconds(1))
             .latest_leader_result == "cancelled");

  for (int index = 0; index < 48; ++index) {
    assert(saturated_exit_queue.pop(&survivor));
    assert(survivor.command_id ==
           "saturated-ordinary-" + std::to_string(index));
  }
  for (int index = 0; index < 14; ++index) {
    assert(saturated_exit_queue.pop(&survivor));
    assert(survivor.command_id ==
           "saturated-stop-" + std::to_string(index));
  }
  assert(!saturated_exit_queue.pop(&survivor));

  // Parser drift cannot leak the two reservation units. This acquire has a
  // coherent campaign identity (so the queue deliberately reserves its exit
  // lane) but an illegal client-supplied fence (so application decode rejects
  // it). The runtime recovery contract tombstones the campaign and retires
  // both slots before accepting another full-capacity batch.
  LeaderAwareRobotCommandQueue malformed_queue;
  RobotCommandMessage malformed_acquire;
  malformed_acquire.kind = "leader_acquire";
  malformed_acquire.command_id = "malformed-acquire";
  malformed_acquire.sender_id = "steamdeck:steamdeck-secondary";
  malformed_acquire.raw_payload =
      "{\"command\":\"leader_acquire\","
      "\"sender_id\":\"steamdeck:steamdeck-secondary\","
      "\"controller_boot_id\":\"malformed-boot\","
      "\"request_id\":\"malformed-acquire\","
      "\"campaign_generation\":12,\"ttl_ms\":1500,"
      "\"lease_id\":\"illegal-client-fence\"}";
  assert(malformed_queue.push(std::move(malformed_acquire)));
  assert(malformed_queue.pop(&survivor));
  LeaderLeaseRequest rejected_malformed;
  assert(!decode_leader_lease_request(
      survivor.raw_payload, &rejected_malformed));
  rejected_malformed.command = "leader_acquire";
  rejected_malformed.sender_id = "steamdeck:steamdeck-secondary";
  rejected_malformed.controller_boot_id = "malformed-boot";
  rejected_malformed.request_id = "malformed-acquire";
  rejected_malformed.campaign_generation = 12;
  SteamDeckLeaderAuthority malformed_authority(
      /*server_epoch_seed=*/1100,
      /*injected_lease_nonce=*/0x4321);
  malformed_authority.record_terminal_acquire_rejection(
      rejected_malformed, "invalid_leader_request_envelope");
  malformed_queue.retire_exit_reservation(
      rejected_malformed.sender_id,
      rejected_malformed.controller_boot_id,
      rejected_malformed.campaign_generation);
  for (int index = 0; index < 48; ++index) {
    RobotCommandMessage ordinary;
    ordinary.kind = "plan_request";
    ordinary.command_id =
        "reclaimed-ordinary-" + std::to_string(index);
    assert(malformed_queue.push(std::move(ordinary)));
  }
  for (int index = 0; index < 16; ++index) {
    RobotCommandMessage protected_stop;
    protected_stop.kind = "stop";
    protected_stop.command_id =
        "reclaimed-stop-" + std::to_string(index);
    assert(malformed_queue.push(std::move(protected_stop)));
  }
  RobotCommandMessage reclaimed_overflow;
  reclaimed_overflow.kind = "stop";
  reclaimed_overflow.command_id = "reclaimed-overflow";
  assert(!malformed_queue.push(std::move(reclaimed_overflow)));
  for (int index = 0; index < 64; ++index) {
    assert(malformed_queue.pop(&survivor));
  }
  assert(!malformed_queue.pop(&survivor));
  LeaderLeaseRequest corrected_delayed = rejected_malformed;
  corrected_delayed.lease_id.clear();
  corrected_delayed.ttl_ms = 1500;
  assert(!malformed_authority.begin_request(
               corrected_delayed,
               saturated_at + std::chrono::milliseconds(2))
              .accepted);

  LeaderAwareCommandIDDeduper committed_deduper;
  assert(!committed_deduper.contains("retry-after-overflow"));
  committed_deduper.remember("accepted-command");
  assert(committed_deduper.contains("accepted-command"));
  assert(!committed_deduper.contains("retry-after-overflow"));
  return 0;
}
