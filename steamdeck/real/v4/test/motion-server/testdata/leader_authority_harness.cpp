#include <cassert>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "../../../../../../motion-server/v1/src/robot_v4_cartesian_leader_authority.hpp"

namespace {

using Authority = SteamDeckLeaderAuthority;
using Clock = Authority::Clock;

LeaderLeaseRequest acquire(const std::string& deck,
                           const std::string& boot,
                           const std::string& request,
                           std::uint64_t campaign_generation = 1) {
  LeaderLeaseRequest out;
  out.command = "leader_acquire";
  out.sender_id = deck;
  out.controller_boot_id = boot;
  out.request_id = request;
  out.campaign_generation = campaign_generation;
  out.ttl_ms = 1500;
  return out;
}

LeaderLeaseRequest renew_or_release(const std::string& command,
                                    const LeaderLeaseStatus& status,
                                    const std::string& request) {
  LeaderLeaseRequest out;
  out.command = command;
  out.sender_id = status.leader_id;
  out.controller_boot_id = status.leader_boot_id;
  out.request_id = request;
  out.lease_id = status.leader_lease_id;
  out.fence_epoch = status.leader_fence_epoch;
  out.campaign_generation = status.leader_campaign_generation;
  out.ttl_ms = command == "leader_renew" ? 1500 : 0;
  return out;
}

LeaderLeaseRequest cancel(const std::string& deck,
                          const std::string& boot,
                          const std::string& request,
                          std::uint64_t campaign_generation) {
  LeaderLeaseRequest out;
  out.command = "leader_cancel";
  out.sender_id = deck;
  out.controller_boot_id = boot;
  out.request_id = request;
  out.campaign_generation = campaign_generation;
  return out;
}

}  // namespace

int main() {
  const auto start = Clock::time_point{} + std::chrono::seconds(10);

  // Steam Deck remains the compatibility default. Structurally valid future
  // controller roles fail closed until the server allowlist includes them.
  assert(ControllerLeaderAuthority::concrete_controller_sender(
      "offline-programming:server-1"));
  assert(!ControllerLeaderAuthority::concrete_controller_sender(
      "offline_programming:server-1"));
  assert(!ControllerLeaderAuthority::concrete_controller_sender(
      "offline-programming:Server-1"));
  assert(!ControllerLeaderAuthority::concrete_controller_sender(
      "steamdeck:server-1"));
  Authority default_roles(/*server_epoch_seed=*/90,
                          /*injected_lease_nonce=*/0x1000);
  assert(!default_roles.begin_request(
                           acquire("offline-programming:server-1",
                                   "offline-boot", "offline-denied"),
                           start)
              .accepted);

  // Pair/Home authority invalidation is not a TTL timeout. It must be able to
  // revoke a still-fresh lease without terminally shutting down the client so
  // a higher campaign can reacquire after the safety transition.
  Authority forced_expiry(/*server_epoch_seed=*/92,
                          /*injected_lease_nonce=*/0x1002);
  assert(forced_expiry.begin_request(
                           acquire("steamdeck:steamdeck-operator", "forced-boot",
                                   "forced-acquire", 1),
                           start)
             .accepted);
  forced_expiry.finish_transition(true, start);
  assert(forced_expiry.status(start).leader_lease_fresh);
  assert(!forced_expiry.begin_expiry(start).accepted);
  const LeaderLeaseAction forced = forced_expiry.begin_expiry(start, true);
  assert(forced.accepted);
  assert(forced.transition == LeaderLeaseTransition::kRevoke);
  assert(forced_expiry.status(start).leader_transition_pending);
  forced_expiry.finish_transition(true, start);
  const LeaderLeaseStatus forced_follower = forced_expiry.status(start);
  assert(!forced_follower.leader_lease_fresh);
  assert(forced_follower.latest_leader_result == "expired");
  assert(forced_follower.latest_leader_request_id == "lease-expiry");
  assert(forced_expiry.begin_request(
                           acquire("steamdeck:steamdeck-operator", "forced-boot",
                                   "forced-reacquire", 2),
                           start + std::chrono::milliseconds(1))
             .accepted);

  ControllerLeaderAuthority generalized(
      /*server_epoch_seed=*/95,
      std::vector<std::string>{"steamdeck", "offline-programming"},
      /*injected_lease_nonce=*/0x1001);
  assert(!generalized.begin_request(
                         acquire("motion-server:server-1",
                                 "spoof-boot", "spoof-role"),
                         start)
              .accepted);
  const LeaderLeaseAction offline_acquire = generalized.begin_request(
      acquire("offline-programming:server-1", "offline-boot",
              "offline-acquire", 3),
      start);
  assert(offline_acquire.accepted);
  generalized.finish_transition(true, start);
  const LeaderLeaseStatus offline_leader = generalized.status(start);
  assert(offline_leader.leader_id ==
         "offline-programming:server-1");
  assert(offline_leader.leader_campaign_generation == 3);
  assert(generalized.authorizes_hot(
      offline_leader.leader_id, offline_leader.leader_lease_id,
      offline_leader.leader_fence_epoch, start));
  assert(!generalized.authorizes_hot(
      offline_leader.leader_id, offline_leader.leader_lease_id,
      offline_leader.leader_fence_epoch + 1, start));

  Authority authority(/*server_epoch_seed=*/100,
                      /*injected_lease_nonce=*/0x1234);
  assert(!authority.status(start).leader_lease_fresh);
  assert(!authority.begin_request(
                       acquire("steamdeck:steamdeck-secondary", "legacy-boot",
                               "legacy-acquire", 0),
                       start)
              .accepted);
  assert(authority.status(start).latest_leader_result == "none");
  assert(authority.status(start).latest_leader_request_id == "none");
  assert(authority.status(start).latest_leader_campaign_generation == 0);

  const LeaderLeaseAction first =
      authority.begin_request(
          acquire("steamdeck:steamdeck-secondary", "boot-a", "acquire-a"), start);
  assert(first.accepted);
  assert(first.transition == LeaderLeaseTransition::kRevokeThenAcquire);
  LeaderLeaseStatus pending = authority.status(start);
  assert(pending.leader_transition_pending);
  assert(!pending.leader_lease_fresh);
  assert(pending.leader_id == "none");

  authority.finish_transition(true, start);
  const LeaderLeaseStatus leader_a = authority.status(start);
  assert(leader_a.leader_lease_fresh);
  assert(leader_a.leader_id == "steamdeck:steamdeck-secondary");
  assert(leader_a.leader_boot_id == "boot-a");
  assert(leader_a.leader_fence_epoch == 100);
  assert(leader_a.leader_campaign_generation == 1);
  assert(leader_a.leader_lease_id.find("0000000000001234") !=
         std::string::npos);
  assert(authority.authorizes_hot(
      leader_a.leader_id, leader_a.leader_lease_id,
      leader_a.leader_fence_epoch, start));

  // A lost-status retry from the same controller is renew-like: it preserves
  // the exact grant and does not churn through another safety transition.
  const LeaderLeaseAction retry = authority.begin_request(
      acquire("steamdeck:steamdeck-secondary", "boot-a", "acquire-a-retry"),
      start + std::chrono::milliseconds(400));
  assert(retry.accepted);
  assert(retry.transition == LeaderLeaseTransition::kNone);
  const LeaderLeaseStatus retried =
      authority.status(start + std::chrono::milliseconds(400));
  assert(retried.leader_lease_id == leader_a.leader_lease_id);
  assert(retried.leader_fence_epoch == leader_a.leader_fence_epoch);
  assert(retried.latest_leader_campaign_generation == 1);

  // Exact identity, token, and epoch are all required.
  LeaderLeaseRequest wrong = renew_or_release(
      "leader_renew", retried, "renew-wrong");
  wrong.sender_id = "steamdeck:steamdeck-pendant";
  assert(!authority.begin_request(
                        wrong, start + std::chrono::milliseconds(500))
              .accepted);
  wrong = renew_or_release("leader_renew", retried, "renew-wrong-epoch");
  ++wrong.fence_epoch;
  assert(!authority.begin_request(
                        wrong, start + std::chrono::milliseconds(500))
              .accepted);
  wrong = renew_or_release("leader_renew", retried, "renew-wrong-token");
  wrong.lease_id += "-stale";
  assert(!authority.begin_request(
                        wrong, start + std::chrono::milliseconds(500))
              .accepted);

  const LeaderLeaseAction renewed = authority.begin_request(
      renew_or_release("leader_renew", retried, "renew-a"),
      start + std::chrono::milliseconds(500));
  assert(renewed.accepted);
  assert(renewed.transition == LeaderLeaseTransition::kNone);

  // B preempts A only through a no-leader transition. Old A packets remain
  // rejected after B's new epoch and token are installed.
  const LeaderLeaseAction preempt = authority.begin_request(
      acquire("steamdeck:steamdeck-pendant", "boot-b", "acquire-b"),
      start + std::chrono::milliseconds(600));
  assert(preempt.accepted);
  assert(preempt.transition ==
         LeaderLeaseTransition::kRevokeThenAcquire);
  assert(authority.status(start + std::chrono::milliseconds(600))
             .leader_transition_pending);
  assert(!authority.authorizes_hot(
      leader_a.leader_id, leader_a.leader_lease_id,
      leader_a.leader_fence_epoch,
      start + std::chrono::milliseconds(600)));
  authority.finish_transition(
      true, start + std::chrono::milliseconds(600));
  const LeaderLeaseStatus leader_b =
      authority.status(start + std::chrono::milliseconds(600));
  assert(leader_b.leader_id == "steamdeck:steamdeck-pendant");
  assert(leader_b.leader_fence_epoch != leader_a.leader_fence_epoch);
  assert(leader_b.leader_lease_id != leader_a.leader_lease_id);
  assert(!authority.authorizes_hot(
      leader_a.leader_id, leader_a.leader_lease_id,
      leader_a.leader_fence_epoch,
      start + std::chrono::milliseconds(601)));
  const LeaderLeaseAction delayed_preempted_a =
      authority.begin_request(
          acquire("steamdeck:steamdeck-secondary", "boot-a",
                  "delayed-preempted-a"),
          start + std::chrono::milliseconds(602));
  assert(!delayed_preempted_a.accepted);
  assert(delayed_preempted_a.result == "cancelled");
  assert(authority.status(start + std::chrono::milliseconds(602))
             .leader_id == "steamdeck:steamdeck-pendant");

  // A partition expires the lease and first exposes transition_pending. The
  // leader remains absent after the safety actions complete.
  const auto expired_at =
      start + std::chrono::milliseconds(600 + 1501);
  assert(authority.expiry_due(expired_at));
  const LeaderLeaseAction expiry = authority.begin_expiry(expired_at);
  assert(expiry.accepted);
  assert(expiry.transition == LeaderLeaseTransition::kRevoke);
  assert(authority.status(expired_at).leader_transition_pending);
  authority.finish_transition(true, expired_at);
  assert(!authority.status(expired_at).leader_lease_fresh);
  const LeaderLeaseAction delayed_expired_b =
      authority.begin_request(
          acquire("steamdeck:steamdeck-pendant", "boot-b",
                  "delayed-expired-b"),
          expired_at);
  assert(!delayed_expired_b.accepted);
  assert(delayed_expired_b.result == "cancelled");

  // Release is also revoke-first.
  const LeaderLeaseAction reacquire = authority.begin_request(
      acquire("steamdeck:steamdeck-secondary", "boot-c", "acquire-c"),
      expired_at + std::chrono::milliseconds(1));
  assert(reacquire.accepted);
  authority.finish_transition(
      true, expired_at + std::chrono::milliseconds(1));
  const LeaderLeaseStatus leader_c =
      authority.status(expired_at + std::chrono::milliseconds(1));
  const LeaderLeaseAction release = authority.begin_request(
      renew_or_release("leader_release", leader_c, "release-c"),
      expired_at + std::chrono::milliseconds(2));
  assert(release.accepted);
  assert(release.transition == LeaderLeaseTransition::kRevoke);
  authority.finish_transition(
      true, expired_at + std::chrono::milliseconds(2));
  assert(!authority.status(expired_at + std::chrono::milliseconds(2))
              .leader_lease_fresh);
  const LeaderLeaseAction delayed_released_c =
      authority.begin_request(
          acquire("steamdeck:steamdeck-secondary", "boot-c",
                  "delayed-released-c"),
          expired_at + std::chrono::milliseconds(3));
  assert(!delayed_released_c.accepted);
  assert(delayed_released_c.result == "cancelled");

  // A daemon restart never reconstructs a leader from an earlier process.
  Authority restarted(/*server_epoch_seed=*/500,
                      /*injected_lease_nonce=*/0x5678);
  assert(!restarted.status(expired_at).leader_lease_fresh);
  assert(restarted.status(expired_at).leader_id == "none");

  // A foreign cancel may be observed while revoke-first acquisition is in
  // flight, but it cannot replace the pending acquire's terminal outcome
  // identity. The grant/result/request/generation tuple must stay coherent.
  Authority correlated(/*server_epoch_seed=*/650,
                       /*injected_lease_nonce=*/0x8abc);
  const auto correlated_at = expired_at + std::chrono::milliseconds(500);
  const LeaderLeaseAction correlated_acquire = correlated.begin_request(
      acquire("steamdeck:steamdeck-secondary", "correlated-boot",
              "correlated-acquire", 11),
      correlated_at);
  assert(correlated_acquire.accepted);
  const LeaderLeaseAction correlated_foreign_cancel =
      correlated.begin_request(
          cancel("steamdeck:steamdeck-pendant", "foreign-correlated-boot",
                 "foreign-correlated-cancel", 7),
          correlated_at);
  assert(correlated_foreign_cancel.accepted);
  assert(correlated_foreign_cancel.transition ==
         LeaderLeaseTransition::kNone);
  correlated.finish_transition(true, correlated_at);
  const LeaderLeaseStatus correlated_status =
      correlated.status(correlated_at);
  assert(correlated_status.leader_lease_fresh);
  assert(correlated_status.leader_id == "steamdeck:steamdeck-secondary");
  assert(correlated_status.latest_leader_result == "acquired");
  assert(correlated_status.latest_leader_request_id ==
         "correlated-acquire");
  assert(correlated_status.latest_leader_campaign_generation == 11);

  // A terminally rejected duplicate campaign installs its barrier before
  // reservation retirement. Even an already-pending copy must re-check that
  // barrier at transition completion and remain leaderless.
  Authority rejected_pending(/*server_epoch_seed=*/675,
                             /*injected_lease_nonce=*/0x8def);
  const LeaderLeaseAction first_pending =
      rejected_pending.begin_request(
          acquire("steamdeck:steamdeck-secondary", "rejected-pending-boot",
                  "rejected-pending-first", 4),
          correlated_at);
  assert(first_pending.accepted);
  const LeaderLeaseAction terminal_duplicate =
      rejected_pending.begin_request(
          acquire("steamdeck:steamdeck-secondary", "rejected-pending-boot",
                  "rejected-pending-duplicate", 4),
          correlated_at);
  assert(!terminal_duplicate.accepted);
  assert(terminal_duplicate.result == "leader_transition_pending");
  rejected_pending.finish_transition(true, correlated_at);
  assert(!rejected_pending.status(correlated_at)
              .leader_lease_fresh);
  assert(rejected_pending.status(correlated_at)
             .latest_leader_result == "cancelled");

  // An acquire that has entered the application but not completed its
  // revoke-first transition can be cancelled without first finishing that
  // transition. A late completion callback remains harmless.
  Authority raced(/*server_epoch_seed=*/700,
                  /*injected_lease_nonce=*/0x9abc);
  const auto race_at = expired_at + std::chrono::seconds(1);
  const LeaderLeaseAction pending_acquire = raced.begin_request(
      acquire("steamdeck:steamdeck-secondary", "race-boot", "race-acquire", 1),
      race_at);
  assert(pending_acquire.accepted);
  assert(pending_acquire.transition ==
         LeaderLeaseTransition::kRevokeThenAcquire);
  const LeaderLeaseAction cancel_pending = raced.begin_request(
      cancel("steamdeck:steamdeck-secondary", "race-boot", "race-cancel", 1),
      race_at);
  assert(cancel_pending.accepted);
  assert(cancel_pending.transition == LeaderLeaseTransition::kRevoke);
  raced.finish_transition(true, race_at);
  raced.finish_transition(true, race_at);
  LeaderLeaseStatus cancelled = raced.status(race_at);
  assert(!cancelled.leader_lease_fresh);
  assert(cancelled.latest_leader_result == "cancelled");
  assert(cancelled.latest_leader_request_id == "race-cancel");
  assert(cancelled.latest_leader_campaign_generation == 1);

  // The cancel high-water mark closes the cross-connection ordering hole:
  // the delayed old acquire cannot recreate authority.
  const LeaderLeaseAction delayed_old = raced.begin_request(
      acquire("steamdeck:steamdeck-secondary", "race-boot",
              "race-acquire-delayed", 1),
      race_at + std::chrono::milliseconds(1));
  assert(!delayed_old.accepted);
  assert(delayed_old.result == "cancelled");
  assert(!raced.status(race_at + std::chrono::milliseconds(1))
              .leader_lease_fresh);

  // A later physical RT+A uses a higher generation and may acquire. Neither
  // an older cancel from this controller nor a foreign cancel may revoke it.
  const LeaderLeaseAction newer = raced.begin_request(
      acquire("steamdeck:steamdeck-secondary", "race-boot", "race-acquire-2", 2),
      race_at + std::chrono::milliseconds(2));
  assert(newer.accepted);
  raced.finish_transition(
      true, race_at + std::chrono::milliseconds(2));
  const LeaderLeaseStatus generation_two =
      raced.status(race_at + std::chrono::milliseconds(2));
  assert(generation_two.leader_lease_fresh);
  assert(generation_two.leader_campaign_generation == 2);
  const LeaderLeaseAction stale_cancel = raced.begin_request(
      cancel("steamdeck:steamdeck-secondary", "race-boot", "race-cancel-stale", 1),
      race_at + std::chrono::milliseconds(3));
  assert(stale_cancel.accepted);
  assert(stale_cancel.transition == LeaderLeaseTransition::kNone);
  assert(raced.status(race_at + std::chrono::milliseconds(3))
             .leader_campaign_generation == 2);
  const LeaderLeaseAction foreign_cancel = raced.begin_request(
      cancel("steamdeck:steamdeck-pendant", "foreign-boot",
             "race-cancel-foreign", 2),
      race_at + std::chrono::milliseconds(4));
  assert(foreign_cancel.accepted);
  assert(foreign_cancel.transition == LeaderLeaseTransition::kNone);
  assert(raced.status(race_at + std::chrono::milliseconds(4)).leader_id ==
         "steamdeck:steamdeck-secondary");

  const LeaderLeaseAction exact_cancel = raced.begin_request(
      cancel("steamdeck:steamdeck-secondary", "race-boot", "race-cancel-2", 2),
      race_at + std::chrono::milliseconds(5));
  assert(exact_cancel.accepted);
  assert(exact_cancel.transition == LeaderLeaseTransition::kRevoke);
  raced.finish_transition(
      true, race_at + std::chrono::milliseconds(5));
  assert(!raced.status(race_at + std::chrono::milliseconds(5))
              .leader_lease_fresh);
  const LeaderLeaseAction generation_three = raced.begin_request(
      acquire("steamdeck:steamdeck-secondary", "race-boot", "race-acquire-3", 3),
      race_at + std::chrono::milliseconds(6));
  assert(generation_three.accepted);
  raced.finish_transition(
      true, race_at + std::chrono::milliseconds(6));
  assert(raced.status(race_at + std::chrono::milliseconds(6))
             .leader_campaign_generation == 3);

  // High-water cancellation also covers an older same-controller campaign
  // that was already pending when a newer cancel arrived.
  Authority covered(/*server_epoch_seed=*/800,
                    /*injected_lease_nonce=*/0xcdef);
  const LeaderLeaseAction covered_pending = covered.begin_request(
      acquire("steamdeck:steamdeck-secondary", "covered-boot",
              "covered-acquire-1", 1),
      race_at);
  assert(covered_pending.accepted);
  const LeaderLeaseAction covering_cancel = covered.begin_request(
      cancel("steamdeck:steamdeck-secondary", "covered-boot",
             "covered-cancel-2", 2),
      race_at);
  assert(covering_cancel.accepted);
  assert(covering_cancel.transition == LeaderLeaseTransition::kRevoke);
  covered.finish_transition(true, race_at);
  assert(!covered.status(race_at).leader_lease_fresh);
  assert(covered.status(race_at).latest_leader_campaign_generation == 2);
  assert(!covered.begin_request(
                     acquire("steamdeck:steamdeck-secondary", "covered-boot",
                             "covered-delayed-1", 1),
                     race_at + std::chrono::milliseconds(1))
              .accepted);
  const LeaderLeaseAction covered_newer = covered.begin_request(
      acquire("steamdeck:steamdeck-secondary", "covered-boot",
              "covered-acquire-3", 3),
      race_at + std::chrono::milliseconds(2));
  assert(covered_newer.accepted);
  covered.finish_transition(
      true, race_at + std::chrono::milliseconds(2));
  assert(covered.status(race_at + std::chrono::milliseconds(2))
             .leader_campaign_generation == 3);

  // Tombstone memory is process-local and hard bounded even across many
  // controller boots.
  Authority bounded(/*server_epoch_seed=*/900,
                    /*injected_lease_nonce=*/0xdef0);
  for (int index = 0; index < 300; ++index) {
    const LeaderLeaseAction action = bounded.begin_request(
        cancel("steamdeck:steamdeck-secondary",
               "bounded-boot-" + std::to_string(index),
               "bounded-cancel-" + std::to_string(index), 1),
        race_at);
    assert(action.accepted);
  }
  assert(bounded.cancel_tombstone_count() == 128);
  return 0;
}
