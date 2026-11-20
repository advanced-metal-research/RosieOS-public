// Production parsing refuses a retired backend before native resource acquisition.
#define main production_main
#include "../src/robot_v4_cartesian_daemon.cpp"
#undef main
#include <cassert>

int main() {
  const auto selected = [](std::vector<std::string> args) {
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(arg.data());
    Options options;
    if (!parse_args(static_cast<int>(argv.size()), argv.data(), &options)) return std::string("refused");
    return options.backend;
  };
  struct SelectionCase {
    const char* flag;
    const char* environment;
    const char* expected;
  };
  // Unverified selection contract: explicit flag, then environment, then rt_core.
  const SelectionCase cases[] = {
      {nullptr, nullptr, "rt_core"},
      {nullptr, "legacy_v4", "refused"},
      {nullptr, "rt_core", "rt_core"},
      {"legacy_v4", nullptr, "refused"},
      {"legacy_v4", "legacy_v4", "refused"},
      {"legacy_v4", "rt_core", "refused"},
      {"rt_core", nullptr, "rt_core"},
      {"rt_core", "legacy_v4", "rt_core"},
      {"rt_core", "rt_core", "rt_core"},
  };
  for (const auto& test : cases) {
    if (test.environment) {
      setenv("MOTION_SERVER_BACKEND", test.environment, 1);
    } else {
      unsetenv("MOTION_SERVER_BACKEND");
    }
    std::vector<std::string> args{"cartesiand"};
    if (test.flag) args.insert(args.end(), {"--backend", test.flag});
    assert(selected(args) == test.expected);
  }
  setenv("MOTION_SERVER_BACKEND", "invalid", 1);
  Options options;
  char name[] = "cartesiand";
  char* argv[] = {name};
  assert(!parse_args(1, argv, &options));
  unsetenv("MOTION_SERVER_BACKEND");
  std::ostringstream error;
  auto* previous = std::cerr.rdbuf(error.rdbuf());
  assert(production_main(1, argv) == 2);
  std::cerr.rdbuf(previous);
  assert(error.str().find("missing binding --rt-control-socket") != std::string::npos);
  char flag[] = "--backend";
  char retired[] = "legacy_v4";
  char* obsolete[] = {name, flag, retired};
  error.str("");
  previous = std::cerr.rdbuf(error.rdbuf());
  assert(production_main(3, obsolete) == 2);
  std::cerr.rdbuf(previous);
  assert(error.str().find("reason=backend_retired") != std::string::npos);
  for (const char* retired_flag : {"--rtcore-target", "--nats-go-home-subject",
                                   "--nats-arm-subject", "--nats-io-subject"}) {
    std::vector<std::string> args{"cartesiand", retired_flag, "unavailable"};
    error.str("");
    previous = std::cerr.rdbuf(error.rdbuf());
    assert(selected(args) == "refused");
    std::cerr.rdbuf(previous);
    assert(error.str().find("reason=backend_retired") != std::string::npos);
  }
}
