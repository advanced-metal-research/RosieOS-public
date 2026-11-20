#include "socket_owner.hpp"
#include <cassert>
#include <initializer_list>

int main() {
  using namespace socket_owner;
  for (const auto probe : {Probe::Missing, Probe::Refused, Probe::Live, Probe::Unknown}) {
    for (const bool locked : {false, true})
      assert(decide(probe, false, locked) == Decision::Refuse);
  }
  assert(decide(Probe::Missing, true, true) == Decision::Bind);
  assert(decide(Probe::Refused, true, true) == Decision::Recover);
  assert(decide(Probe::Missing, true, false) == Decision::LiveOwner);
  assert(decide(Probe::Refused, true, false) == Decision::LiveOwner);
  assert(decide(Probe::Live, true, false) == Decision::LiveOwner);
  assert(decide(Probe::Live, true, true) == Decision::LiveOwner);
  assert(decide(Probe::Unknown, true, false) == Decision::Refuse);
  assert(decide(Probe::Unknown, true, true) == Decision::Refuse);
}
