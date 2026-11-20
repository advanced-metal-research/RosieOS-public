#include "peer_admission.hpp"

#include <cassert>
#include <cstdio>
#include <limits>

using rosie::rtcore::admit_peer;
using rosie::rtcore::PeerAdmission;
using namespace gradient::ipc::v1;

int main() {
  // unverified synthetic identity fixtures: PID 4321 and three distinct UIDs.
  constexpr pid_t peer_pid = 4321;
  // unverified synthetic daemon UID, independent of the test runner's account.
  constexpr uid_t daemon_uid = 1001;
  // unverified synthetic adapter UID, distinct from the daemon and foreign UIDs.
  constexpr uid_t adapter_uid = 1002;
  // unverified synthetic foreign UID, never authorized by these fixtures.
  constexpr uid_t foreign_uid = 1003;

  ucred peer{peer_pid, daemon_uid, 0};
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, std::nullopt, false) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, adapter_uid, false) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleAdapter, 4322, peer, daemon_uid, std::nullopt, false) ==
         PeerAdmission::PidMismatch);

  peer.uid = adapter_uid;
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, adapter_uid, false) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleAdapter, 4322, peer, daemon_uid, adapter_uid, false) ==
         PeerAdmission::PidMismatch);
  assert(admit_peer(kRoleAdapter, std::numeric_limits<uint64_t>::max(), peer,
                    daemon_uid, adapter_uid, false) == PeerAdmission::PidMismatch);
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, std::nullopt, false) ==
         PeerAdmission::UidMismatch);
  assert(admit_peer(kRoleController, 4321, peer, daemon_uid, adapter_uid, true) ==
         PeerAdmission::UidMismatch);

  peer.uid = foreign_uid;
  peer.gid = adapter_uid;
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, adapter_uid, false) ==
         PeerAdmission::UidMismatch);
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, std::nullopt, false) ==
         PeerAdmission::UidMismatch);

  peer.uid = daemon_uid;
  assert(admit_peer(kRoleController, 4321, peer, daemon_uid, std::nullopt, true) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleController, 4322, peer, daemon_uid, adapter_uid, true) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleController, 4321, peer, daemon_uid, adapter_uid, false) ==
         PeerAdmission::RoleRejected);
  assert(admit_peer(kRoleObserver, 4321, peer, daemon_uid, adapter_uid, true) ==
         PeerAdmission::RoleRejected);
  assert(admit_peer(0, 4321, peer, daemon_uid, adapter_uid, true) ==
         PeerAdmission::RoleRejected);

  peer.uid = 0;
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, uid_t{0}, false) ==
         PeerAdmission::Admitted);
  assert(admit_peer(kRoleAdapter, 4321, peer, daemon_uid, std::nullopt, false) ==
         PeerAdmission::UidMismatch);
  std::puts("peer admission tests passed");
}
