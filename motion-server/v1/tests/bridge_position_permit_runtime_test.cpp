#include "robot_v4_cartesian_leader_authority.hpp"
#include <iostream>

int main() {
  using namespace std::chrono;
  const auto start = ControllerLeaderAuthority::TimePoint{};
  ControllerLeaderAuthority owner(1, 101);
  LeaderLeaseRequest acquire;
  acquire.command = "leader_acquire"; acquire.sender_id = "steamdeck:steamdeck-secondary";
  acquire.controller_boot_id = "controller-boot"; acquire.request_id = "acquire";
  acquire.campaign_generation = 1; acquire.ttl_ms = 1500;
  int failures = 0;
  const auto check = [&](bool passed, const char* message) {
    if (!passed) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
  };
  check(owner.begin_request(acquire, start).accepted, "real leader acquire failed");
  owner.finish_transition(true, start);
  const auto leader = owner.status(start);
  BridgePositionPermit expected;
  expected.bridge_boot_id = "bridge-boot"; expected.run_id = "run-1";
  expected.controller_id = acquire.sender_id; expected.controller_boot_id = acquire.controller_boot_id;
  expected.command_id_json = "42"; expected.leader_lease_id = leader.leader_lease_id;
  expected.leader_fence_epoch = leader.leader_fence_epoch;
  expected.deadline_monotonic_ms = 10000;
  expected.run_deadline_monotonic_ms = 10500;
  expected.runtime_mode_epoch = 7; expected.activation_epoch = 8; expected.home_epoch = 9;
  expected.pair_revision = 10; expected.pair_config_sha256 = std::string(64, 'a');
  auto reply = expected;
  check(!owner.begin_bridge_position_challenge(expected, start).empty() && owner.bridge_run_bound(),
        "pending challenge did not inhibit conflicting execution");
  owner.cancel_bridge_position();
  check(!owner.bridge_run_bound() && owner.authorizes(acquire.sender_id,
            {acquire.controller_boot_id, leader.leader_lease_id, leader.leader_fence_epoch}, start),
        "rejected first admission poisoned ordinary leader authority");
  reply.challenge_id = owner.begin_bridge_position_challenge(expected, start);
  reply.bridge_observed_monotonic_ms = 9000; reply.remaining_ms = 1000;
  reply.run_remaining_ms = 1500;
  check(!reply.challenge_id.empty(), "valid leader could not start challenge");
  for (int mutation = 0; mutation < 9; ++mutation) {
    auto forged = reply;
    switch (mutation) {
      case 0: forged.bridge_boot_id = "restarted-bridge"; break;
      case 1: forged.run_id = "other-run"; break;
      case 2: forged.challenge_id = "replayed-challenge"; break;
      case 3: ++forged.leader_fence_epoch; break;
      case 4: ++forged.runtime_mode_epoch; break;
      case 5: ++forged.remaining_ms; break;
      case 6: forged.command_id_json = "\"42\""; break;
      case 7: ++forged.run_deadline_monotonic_ms; break;
      case 8: ++forged.run_remaining_ms; break;
    }
    check(!owner.accept_bridge_position_permit(forged, start + milliseconds(100)),
          "mismatched or forged permit admitted");
  }
  check(owner.accept_bridge_position_permit(reply, start + milliseconds(500)), "valid delayed permit rejected");
  check(!owner.accept_bridge_position_permit(reply, start + milliseconds(501)), "consumed challenge replayed");
  check(owner.authorizes_bridge_position("run-1", "42", start + milliseconds(999)), "valid permit lost capability");
  check(owner.bridge_position_authority_matches("run-1", "42", start + milliseconds(600), 7, 8, 9, 10, std::string(64, 'a')) &&
            !owner.bridge_position_authority_matches("run-1", "42", start + milliseconds(600), 8, 8, 9, 10, std::string(64, 'a')),
        "a stale permit borrowed the new runtime generation at admission");
  auto renew = acquire;
  renew.command = "leader_renew"; renew.request_id = "renew";
  renew.lease_id = leader.leader_lease_id; renew.fence_epoch = leader.leader_fence_epoch;
  check(owner.begin_request(renew, start + milliseconds(900)).accepted, "normal leader renewal failed");
  check(!owner.authorizes_bridge_position("run-1", "42", start + milliseconds(1000)),
        "receipt delay or leader renewal extended source permit");
  check(owner.matches_bridge_run("run-1", acquire.sender_id, acquire.controller_boot_id, start + milliseconds(1499)),
        "position deadline incorrectly truncated the still-active run's Play capability");
  check(!owner.matches_bridge_run("run-1", acquire.sender_id, acquire.controller_boot_id, start + milliseconds(1500)),
        "normal leader renewal extended source run authority for Play");
  check(!owner.matches_bridge_jog(acquire.sender_id, leader.leader_lease_id, leader.leader_fence_epoch, start + milliseconds(1500)),
        "normal leader renewal extended source run authority for Jog");
  check(owner.expiry_due(start + milliseconds(1500)), "expired source run did not trigger existing owner revoke");
  reply.challenge_id = owner.begin_bridge_position_challenge(expected, start + milliseconds(1000));
  const auto revoke = owner.revoke_bridge_run("bridge-boot", "run-1", start + milliseconds(1001));
  check(revoke.transition == LeaderLeaseTransition::kRevoke, "End did not enter existing revoke transition");
  check(!owner.accept_bridge_position_permit(reply, start + milliseconds(1002)), "late grant survived End");
  owner.finish_transition(false, start + milliseconds(1003));
  check(!owner.authorizes(acquire.sender_id, {acquire.controller_boot_id, leader.leader_lease_id,
              leader.leader_fence_epoch}, start + milliseconds(1004)), "failed revoke output restored old authority");
  ControllerLeaderAuthority restarted(1, 202);
  restarted.begin_request(acquire, start); restarted.finish_transition(true, start);
  check(!restarted.accept_bridge_position_permit(reply, start + milliseconds(100)), "old grant survived Motion Server restart");
  const auto new_leader = restarted.status(start);
  expected.leader_lease_id = new_leader.leader_lease_id;
  expected.leader_fence_epoch = new_leader.leader_fence_epoch;
  auto fresh = expected;
  fresh.challenge_id = restarted.begin_bridge_position_challenge(expected, start);
  fresh.bridge_observed_monotonic_ms = 9000; fresh.remaining_ms = 1000;
  fresh.run_remaining_ms = 1500;
  check(restarted.accept_bridge_position_permit(fresh, start + milliseconds(100)), "fresh restart grant failed");
  check(restarted.observe_bridge_boot("new-bridge-boot", start + milliseconds(200)).transition == LeaderLeaseTransition::kRevoke &&
            !restarted.authorizes_bridge_position("run-1", "42", start + milliseconds(201)),
        "authoritative bridge restart did not revoke bound execution");
  if (failures) return 1;
  std::cout << "bridge_position_permit_runtime_test: passed (authority binding, not transport authentication)\n";
}
