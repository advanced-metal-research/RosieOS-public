#pragma once

#include "ipc_v1.hpp"

#include <cstdint>
#include <optional>
#include <sys/socket.h>
#include <sys/types.h>

namespace rosie::rtcore {

enum class PeerAdmission : uint8_t {
  Admitted,
  RoleRejected,
  UidMismatch,
  PidMismatch,
};

// Admit only a permitted role bound to kernel credentials. The configured UID
// extends adapter access only; it cannot authorize a foreign legacy oracle or
// bypass the adapter PID check. The caller validates the remaining HELLO fields.
inline PeerAdmission admit_peer(uint32_t hello_role, uint64_t hello_pid,
                                const ucred& peer, uid_t daemon_uid,
                                std::optional<uid_t> adapter_uid,
                                bool legacy_role_allowed) noexcept {
  const bool adapter = hello_role == gradient::ipc::v1::kRoleAdapter;
  const bool legacy = legacy_role_allowed &&
      hello_role == gradient::ipc::v1::kRoleController;
  if (!adapter && !legacy) return PeerAdmission::RoleRejected;
  if (peer.uid != daemon_uid &&
      !(adapter && adapter_uid && peer.uid == *adapter_uid))
    return PeerAdmission::UidMismatch;
  if (adapter && hello_pid != static_cast<uint64_t>(peer.pid))
    return PeerAdmission::PidMismatch;
  return PeerAdmission::Admitted;
}

} // namespace rosie::rtcore
