// Obsolete selection fails before native startup; argument order cannot revive it.
#include <cassert>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include "runtime_config.hpp"

using namespace dense_joint_trajectory;

int main() {
  const std::string path = TEST_BUILD_DIR "/backend-config-" + std::to_string(::getpid()) + ".json";
  const auto write = [&](const std::string& fields) {
    std::ofstream file(path);
    file << "{\"schema\":\"robot.v4.joint-trajectory-daemon.config.v1\"" << fields << "}";
  };
  DaemonConfig config;
  std::string error;
  for (const std::string fields : {"", ",\"backend\":\"legacy_v4\"", ",\"backend\":\"unknown\""}) {
    write(fields);
    assert(!load_daemon_config(path, &config, &error));
    assert(error == "reason=backend_retired");
    assert(!load_daemon_config(path, &config, &error, "rt_core"));
  }
  write(",\"backend\":\"rt_core\",\"rt_core\":{\"socket\":\"/run/test.sock\",\"pair_id\":\"bench\",\"pair_revision\":7}");
  assert(load_daemon_config(path, &config, &error));
  assert(config.backend == "rt_core" && config.pair_revision == 7);
  assert(!native_binding_configured(config));
  for (bool first : {false, true}) {
    std::vector<std::string> args = {"daemon", "--config", path, "--backend", "legacy_v4"};
    if (first) args = {"daemon", "--backend", "legacy_v4", "--config", path};
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(arg.data());
    assert(!load_daemon_config_arguments(static_cast<int>(argv.size()), argv.data(), &config, &error));
    assert(error == "reason=backend_retired");
  }
  setenv("JOINT_TRAJECTORY_BACKEND", "legacy_v4", 1);
  char name[] = "daemon";
  char* argv[] = {name};
  assert(!load_daemon_config_arguments(1, argv, &config, &error));
  assert(error == "reason=backend_retired");
  unsetenv("JOINT_TRAJECTORY_BACKEND");
  ::unlink(path.c_str());
  std::cout << "runtime_config_test PASS\n";
}
