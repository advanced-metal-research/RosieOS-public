#include "robot_v4_cartesian_leader_authority.hpp"

#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  }
}

}  // namespace

int main() {
  for (const std::string& sender : {
           "steamdeck:steamdeck-primary",
           "steamdeck:steamdeck-secondary",
           "steamdeck:steamdeck-field-a",
       }) {
    expect(ControllerLeaderAuthority::concrete_steamdeck_sender(sender),
           "concrete Steam Deck rejected: " + sender);
  }
  for (const std::string& sender : {
           "steamdeck:steamdeck-",
           "steamdeck:operator",
           "steamdeck:",
       }) {
    expect(!ControllerLeaderAuthority::concrete_steamdeck_sender(sender),
           "non-concrete Steam Deck accepted: " + sender);
  }
  if (failures != 0) {
    return 1;
  }
  std::cout << "controller_identity_runtime_test: ok\n";
  return 0;
}
