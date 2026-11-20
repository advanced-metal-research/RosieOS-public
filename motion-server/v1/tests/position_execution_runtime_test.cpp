// Compile the actual ingress and queue, not a copied envelope or test parser.
#define main production_main
#include "../src/robot_v4_cartesian_daemon.cpp"
#undef main

namespace {
int failures = 0;
void expect(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}
std::string position_payload(const std::string& fields,
                             const std::string& id = "position-42") {
  return "{\"schema\":\"robot.v4.robot-command.v1\",\"robot\":\"other-relay\","
         "\"command\":\"position\",\"sender_id\":\"steamdeck:steamdeck-secondary\","
         "\"command_id\":\"" + id + "\"" + fields + "}";
}
RobotCommandMessage parse_position(const std::string& fields,
                                   const std::string& id = "position-42") {
  RobotCommandMessage message;
  expect(parse_robot_command_message(position_payload(fields, id), "other-relay", &message),
         "production ingress rejected valid position");
  return message;
}
}

int main() {
  BridgeHomeReply home_reply;
  const std::string home_wire =
      "{\"challenge_id\":\"prepare-1\",\"ticket_id\":\"prepare-1\","
      "\"bridge_boot_id\":\"bridge-1\",\"controller_id\":\"deck-1\","
      "\"controller_boot_id\":\"deck-boot-1\",\"run_id\":\"run-1\","
      "\"kind\":\"go_home\",\"command_id\":9007199254740993,\"state\":\"prepared\"}";
  expect(decode_bridge_home_reply(home_wire, &home_reply) &&
             home_reply.command_id_json == "9007199254740993",
         "Home coordination lost original integer identity");
  expect(!decode_bridge_home_reply(home_wire.substr(0, home_wire.size() - 1) +
             ",\"state\":\"completed\"}", &home_reply),
         "Home coordination accepted duplicate state");
  RTCoreRuntimeModeTracker home_tracker;
  const auto observed = std::chrono::steady_clock::now();
  home_tracker.observe_authority(RTCORE_RUNTIME_MODE_SIM, 1, {}, observed, true);
  RTCoreRuntimeModeBindingSnapshot home_snapshot;
  expect(home_tracker.fresh_snapshot(&home_snapshot, observed) && home_snapshot.home_motion_active,
         "active Home observation was lost across runtime snapshot");
  home_tracker.observe_authority(RTCORE_RUNTIME_MODE_SIM, 1, {}, observed, false);
  expect(home_tracker.fresh_snapshot(&home_snapshot, observed) && !home_snapshot.home_motion_active,
         "terminal Home observation did not clear activity");
  using Outcome = PositionExecutionOutcome;
  using namespace std::chrono;
  const auto now = PositionExecution::Clock::time_point{};
  auto message = parse_position(",\"axis\":5,\"relative_jog_rad\":0.2,\"timeout_ms\":1000");
  expect(parse_position(",\"intent_sequence\":7").motion_intent_seq == 7,
         "legacy bridge intent_sequence did not enter owner ordering");
  expect(message.position.command_id == "position-42" && message.position.axis == 5 &&
             message.position.relative && message.position.target_rad == 0.2,
         "position ingress lost operation identity, axis or relative intent");
  expect(robot_command_kind_is_motion_server_owned("position"), "production position ownership did not cut over");
  expect(robot_command_is_record_only_position(position_payload(",\"servo_v1_action\":\"count-jog\"")),
         "record-only mastering was not kept on its independent path");

  const auto identified_payload = [](const std::string& raw_id) {
    std::string payload = position_payload("");
    const std::string original = "\"position-42\"";
    payload.replace(payload.find(original), original.size(), raw_id);
    return payload;
  };
  for (const std::string& original_id : {
           "42", "0", "18446744073709551615", "\"42\"", "\"deck-42\"",
           "\"deck-\\\"quoted\\\"\"", "\"\\u0034\\u0032\""}) {
    RobotCommandMessage identified;
    expect(parse_robot_command_message(identified_payload(original_id), "other-relay", &identified),
           "numeric/string ID rejected at production ingress");
    for (Outcome terminal : {Outcome::completed, Outcome::cancelled, Outcome::timed_out, Outcome::failed}) {
      PositionExecution execution(identified.position, -1, 1, now);
      expect(position_execution_terminal_reply_payload(execution).empty(),
             "queue acceptance emitted a terminal receipt");
      if (terminal == Outcome::completed) {
        for (int ms : {0, 40, 80, 120}) execution.observe(now + milliseconds(ms), 0);
      } else if (terminal == Outcome::cancelled) execution.cancel();
      else if (terminal == Outcome::failed) execution.fail();
      else execution.feedback_due(now + seconds(60));
      const std::string reply = position_execution_terminal_reply_payload(execution);
      std::string raw_id, state, accepted;
      expect(extract_unique_top_level_json_member(reply, "command_id", &raw_id) && raw_id == original_id,
             "terminal receipt changed original ID type or precision");
      expect(parse_top_level_robot_command_string(reply, "state", &state) &&
                 state == (terminal == Outcome::completed ? "completed" :
                           terminal == Outcome::cancelled ? "cancelled" :
                           terminal == Outcome::timed_out ? "timed_out" : "failed"),
             "terminal receipt lost its execution outcome");
      expect(extract_unique_top_level_json_member(reply, "accepted", &accepted) &&
                 accepted == (terminal == Outcome::completed ? "true" : "false"),
             "unsuccessful execution reported accepted completion");
      int sockets[2];
      if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) != 0) return 1;
      const timeval read_timeout{1, 0};
      ::setsockopt(sockets[1], SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));
      expect(publish_robot_command_reply_body(sockets[0], "_INBOX.position", reply),
             "production reply publisher rejected terminal receipt");
      char wire[4096];
      const ssize_t size = ::recv(sockets[1], wire, sizeof(wire), 0);
      const std::string expected = "PUB _INBOX.position " + std::to_string(reply.size()) + "\r\n" + reply + "\r\n";
      expect(size > 0 && std::string(wire, static_cast<std::size_t>(size)) == expected,
             "production reply wire framing changed terminal identity");
      ::close(sockets[0]); ::close(sockets[1]);
    }
  }
  for (const std::string& invalid_id : {
           "null", "true", "{}", "[]", "\"\"", "-1", "1.5", "1e2", "01",
           "18446744073709551616", "42,\"command_id\":43",
           "42,\"command_\\u0069d\":42"}) {
    RobotCommandMessage rejected;
    expect(!parse_robot_command_message(identified_payload(invalid_id), "other-relay", &rejected),
           "malformed or duplicated position ID admitted");
  }
  RobotCommandMessage numeric_id, string_id, escaped_string_id;
  expect(parse_robot_command_message(identified_payload("42"), "other-relay", &numeric_id) &&
             parse_robot_command_message(identified_payload("\"42\""), "other-relay", &string_id) &&
             parse_robot_command_message(identified_payload("\"\\u0034\\u0032\""), "other-relay", &escaped_string_id),
         "typed deduplication fixture rejected");
  LeaderAwareCommandIDDeduper typed_deduper;
  typed_deduper.remember_position("home-ack", "_INBOX.home");
  typed_deduper.acknowledge_command("home-ack", "accepted");
  auto home_pending = typed_deduper.pending_replies();
  expect(home_pending.size() == 1 && home_pending[0].body == "accepted",
         "source-commit Home acknowledgement was not delivered");
  typed_deduper.delivered(home_pending[0]);
  typed_deduper.remember_position("home-ack", "_INBOX.home");
  typed_deduper.complete_position("home-ack", "completed");
  typed_deduper.complete_position("home-ack", "cancelled");
  expect(typed_deduper.position_result("home-ack") == "completed",
         "Home acknowledgement prevented immutable terminal completion");
  typed_deduper.delivered(typed_deduper.pending_replies()[0]);
  typed_deduper.remember(robot_command_deduplication_key(numeric_id));
  expect(typed_deduper.contains(robot_command_deduplication_key(numeric_id)) &&
             !typed_deduper.contains(robot_command_deduplication_key(string_id)),
         "numeric replay either lost deduplication or swallowed distinct string ID");
  typed_deduper.remember(robot_command_deduplication_key(string_id));
  expect(typed_deduper.contains(robot_command_deduplication_key(escaped_string_id)),
         "equivalent escaped string ID evaded deduplication");
  const std::string retained_key = robot_command_deduplication_key(numeric_id);
  typed_deduper.remember_position(retained_key, "_INBOX.original");
  typed_deduper.complete_position(retained_key, "terminal-original");
  typed_deduper.complete_position(retained_key, "terminal-late");
  auto pending = typed_deduper.pending_replies();
  expect(pending.size() == 1 && pending.front().body == "terminal-original",
         "history overwrote an immutable terminal result");
  typed_deduper.delivered(pending.front());
  expect(typed_deduper.pending_replies().empty() &&
             typed_deduper.position_result(retained_key) == "terminal-original",
         "delivery erased replay evidence");
  for (int i = 0; i < 256; ++i) typed_deduper.remember("later-" + std::to_string(i));
  expect(!typed_deduper.contains(retained_key) && typed_deduper.position_result(retained_key).empty(),
         "terminal cache escaped the existing 256-command history bound");
  auto forged_request = numeric_id.position;
  forged_request.command_id_json = "42,\"accepted\":true";
  PositionExecution forged(forged_request, -1, 1, now);
  forged.cancel();
  expect(position_execution_terminal_reply_payload(forged).empty(),
         "unvalidated stored token injected fields into terminal receipt");
  int legacy_sockets[2];
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, legacy_sockets) != 0) return 1;
  const timeval legacy_read_timeout{1, 0};
  ::setsockopt(legacy_sockets[1], SOL_SOCKET, SO_RCVTIMEO, &legacy_read_timeout, sizeof(legacy_read_timeout));
  expect(publish_robot_command_request_reply(legacy_sockets[0], "_INBOX.legacy", "play", "42", true, false),
         "existing string acceptance reply stopped publishing");
  char legacy_wire[2048];
  const ssize_t legacy_size = ::recv(legacy_sockets[1], legacy_wire, sizeof(legacy_wire), 0);
  expect(legacy_size > 0 && std::string(legacy_wire, static_cast<std::size_t>(legacy_size)).find(
             "\"command_id\":\"42\",\"accepted\":true,\"duplicate\":false,\"state\":\"accepted\"") != std::string::npos,
         "existing string acceptance reply changed type or outcome");
  ::close(legacy_sockets[0]); ::close(legacy_sockets[1]);

  PositionExecution relative(message.position, -1, 1, now);
  expect(relative.outcome() == Outcome::accepted && !relative.target_resolved(),
         "admission incorrectly resolved feedback or completed");
  relative.observe(now, 0.3);
  expect(relative.target_resolved() && relative.target_rad() == 0.5,
         "relative target did not resolve from first feedback");
  relative.observe(now + milliseconds(40), 0.4);
  expect(relative.target_rad() == 0.5, "relative target drifted with feedback");
  relative.observe(now + milliseconds(80), 0.5);
  expect(relative.in_tolerance() && relative.outcome() == Outcome::executing,
         "first tolerance sample did not request settling");
  // Repeated reads at the same control time cannot manufacture four samples.
  for (int i = 0; i < 5; ++i) relative.observe(now + milliseconds(80), 0.5);
  expect(relative.outcome() == Outcome::executing, "early reads completed settling");
  relative.observe(now + milliseconds(120), 0.5);
  relative.observe(now + milliseconds(160), 0.5);
  relative.cancel(); // Stop/replacement/revoke is serialized by the existing owner.
  relative.observe(now + milliseconds(200), 0.5);
  expect(relative.outcome() == Outcome::cancelled && relative.command_id() == "position-42",
         "late fourth sample resurrected cancelled command");
  relative.fail();
  relative.feedback_due(now + seconds(10));
  expect(relative.outcome() == Outcome::cancelled, "terminal cancellation was overwritten");

  auto recovery = parse_position(",\"target_deg\":0", "position-43");
  PositionExecution fresh(recovery.position, -1, 1, now);
  fresh.observe(now, 0);
  fresh.observe(now + milliseconds(40), 0);
  fresh.observe(now + milliseconds(80), 0);
  expect(fresh.outcome() == Outcome::executing, "three samples falsely completed");
  fresh.observe(now + milliseconds(120), 0);
  expect(fresh.outcome() == Outcome::completed && fresh.command_id() == "position-43",
         "fresh request could not complete after cancellation");
  fresh.cancel();
  fresh.fail();
  fresh.feedback_due(now + seconds(20));
  expect(fresh.outcome() == Outcome::completed, "late cancellation rewrote completion");

  auto deadline = parse_position(",\"target_rad\":0,\"timeout_ms\":120");
  PositionExecution timed(deadline.position, -1, 1, now);
  for (int ms : {0, 40, 80, 120}) timed.observe(now + milliseconds(ms), 0);
  expect(timed.outcome() == Outcome::timed_out, "deadline allowed final-sample completion");
  PositionExecution silent(deadline.position, -1, 1, now);
  expect(!silent.feedback_due(now + milliseconds(120)) && silent.outcome() == Outcome::timed_out,
         "missing feedback extended the deadline");
  silent.observe(now + milliseconds(160), 0);
  expect(!silent.target_resolved(), "expired work silently resumed on fresh feedback");

  PositionExecution settling(recovery.position, -1, 1, now);
  settling.observe(now, 0);
  settling.observe(now + milliseconds(40), 0);
  settling.observe(now + milliseconds(80), 0.1);
  for (int ms : {120, 160, 200}) settling.observe(now + milliseconds(ms), 0);
  expect(settling.outcome() == Outcome::executing, "settling count survived an excursion");
  settling.observe(now + milliseconds(240), 0);
  expect(settling.outcome() == Outcome::completed, "settling did not recover");

  auto limited = parse_position(",\"target_deg\":180,\"min_rad\":-0.25,\"max_rad\":0.5");
  PositionExecution clamp(limited.position, -0.1, 0.25, now);
  clamp.observe(now, 0);
  expect(clamp.target_rad() == 0.25, "request widened configured limits");
  auto no_intersection = parse_position(",\"min_rad\":2,\"max_rad\":3");
  PositionExecution invalid_limits(no_intersection.position, -1, 1, now);
  expect(invalid_limits.outcome() == Outcome::failed, "disjoint limits admitted");
  PositionExecution invalid_feedback(recovery.position, -1, 1, now);
  invalid_feedback.observe(now, std::numeric_limits<double>::quiet_NaN());
  invalid_feedback.observe(now + milliseconds(40), 0);
  expect(invalid_feedback.outcome() == Outcome::failed && !invalid_feedback.target_resolved(),
         "invalid feedback resolved or resumed a target");
  auto runaway_request = parse_position(",\"target_rad\":0.2");
  PositionExecution runaway(runaway_request.position, -1, 1, now);
  for (int sample = 0; sample < 4; ++sample)
    runaway.observe(now + milliseconds(40 * sample), -0.004 * sample);
  expect(runaway.outcome() == Outcome::failed, "three growing-error samples did not stop execution");
  auto seam_request = parse_position(",\"target_rad\":-3.13");
  PositionExecution seam(seam_request.position, -kPi, kPi, now);
  seam.observe(now, 3.13);
  expect(seam.outcome() == Outcome::failed, "opposite pi-limit seam was admitted");
  PositionExecution inward(recovery.position, -kPi, kPi, now);
  inward.observe(now, 3.13);
  expect(inward.outcome() == Outcome::executing, "inward recovery from pi limit was denied");

  for (const std::string& fields : {
           ",\"relative_jog_rad\":0.1,\"target_rad\":0.2",
           ",\"axis\":1.5", ",\"axis\":-1", ",\"target_rad\":1e999",
           ",\"target_rad\":0x10", ",\"target_rad\":01", ",\"target_rad\":1.",
           ",\"target_rad\":1,\"target_rad\":2", ",\"timeout_ms\":1.5",
           ",\"intent_sequence\":7,\"motion_intent_seq\":8",
           ",\"control_mode\":\"profile-position\"",
           ",\"servo_v1_action\":\"count-jog\""}) {
    RobotCommandMessage rejected;
    expect(!parse_robot_command_message(position_payload(fields), "other-relay", &rejected),
           "malformed/ambiguous or record-only request admitted as position");
  }
  auto defaults = parse_position(",\"timeout_ms\":-1,\"tolerance_deg\":0");
  auto maximum = parse_position(",\"timeout_ms\":999999");
  expect(defaults.position.timeout == milliseconds(15000) &&
             maximum.position.timeout == milliseconds(45000), "bridge deadline defaults changed");
  auto units = parse_position(",\"target_deg\":90,\"min_deg\":-180,\"max_deg\":180,"
                              "\"tolerance_deg\":1,\"max_speed_rad_s\":0.00001,"
                              "\"control_mode\":\" CSP \"");
  expect(std::abs(units.position.target_rad - kPi / 2) < 1e-12 &&
             std::abs(units.position.min_rad + kPi) < 1e-12 &&
             std::abs(units.position.max_rad - kPi) < 1e-12 &&
             std::abs(units.position.tolerance_rad - kPi / 180) < 1e-12 &&
             units.position.max_speed_rad_s == 0.00001,
         "unit conversion or integrated small speed intent changed");

  // Feed the production queue directly while routing remains disabled. A
  // future cutover must not leave position alive across an existing revoke.
  LeaderAwareRobotCommandQueue queue;
  expect(queue.push(message), "position failed normal queue admission");
  RobotCommandMessage stop; stop.kind = "stop"; stop.command_id = "stop-1";
  expect(queue.push(stop), "stop was lost behind position");
  RobotCommandMessage play; play.kind = "play"; play.command_id = "play-1";
  expect(queue.push(play), "play failed normal queue admission");
  expect(queue.discard_energizing() == 2, "revoke did not discard position with plan work");
  RobotCommandMessage dequeued;
  expect(queue.pop(&dequeued) && dequeued.kind == "stop" && !queue.pop(&dequeued),
         "revoke lost Stop or retained old execution");
  expect(queue.push(recovery) && queue.pop(&dequeued) &&
             dequeued.position.command_id == "position-43", "queue recovery lost fresh identity");

  // Parsing a bridge credential does not turn it into a leader grant. The
  // bridge's active-run store and local monotonic clock remain independent.
  auto bridge = parse_position(",\"run_id\":\"run-1\",\"controller_id\":\"steamdeck:steamdeck-secondary\","
                               "\"controller_boot_id\":\"boot-1\",\"deadline_monotonic_ms\":100,"
                               "\"run_deadline_monotonic_ms\":200");
  expect(bridge.bridge_run.valid(), "production ingress lost bridge-run transport identity");
  ControllerLeaderAuthority authority(1);
  LeaderLeaseCredential credential;
  expect(!decode_leader_lease_credential(position_payload(
             ",\"run_id\":\"run-1\",\"controller_boot_id\":\"boot-1\""), &credential) &&
             !authority.authorizes(bridge.sender_id, credential, now),
         "bridge run silently gained leader authority");
  if (failures) return 1;
  std::cout << "position_execution_runtime_test: passed (no routing or motion qualification)\n";
}
