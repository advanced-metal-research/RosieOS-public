#pragma once

namespace socket_owner {

enum class Probe { Missing, Refused, Live, Unknown };
enum class Decision { Bind, Recover, LiveOwner, Refuse };

// Only a refused connection and an exclusively held, verified owner lock permit
// stale removal. Unknown ownership or transport failures must preserve the path.
constexpr Decision decide(Probe probe, bool ownership_verified, bool lock_held) {
  if (!ownership_verified) return Decision::Refuse;
  if (probe == Probe::Live) return Decision::LiveOwner;
  if (probe == Probe::Unknown) return Decision::Refuse;
  if (!lock_held) return Decision::LiveOwner;
  return probe == Probe::Missing ? Decision::Bind : Decision::Recover;
}

} // namespace socket_owner
