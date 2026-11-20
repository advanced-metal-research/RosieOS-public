// joint_trajectory_daemon -- the dense joint trajectory daemon.
//
// Data plane: TCP ingest (upload/validate/preload/status) -- inert, never
// causes motion. Control plane: NATS robot commands (play/pause/stop under
// leader-lease fencing, leases minted here). The rt_core backend prepares
// immutable programs and starts them through the public C++ control client.
//
// Without --nats-url the daemon runs ingest-only (the phase-2 posture):
// nothing can start motion because nothing can deliver a fenced play.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "command_dispatch.hpp"
#include "execution_backend.hpp"
#include "dense_joint_trajectory_format.hpp"
#include "dense_joint_trajectory_ingest.hpp"
#include "executor_status_json.hpp"
#include "leader_authority.hpp"
#include "nats_client.hpp"
#include "runtime_config.hpp"
#include "rt_control_executor.hpp"

namespace {

std::atomic<bool> keep_running{true};

void handle_signal(int) { keep_running.store(false); }

void usage(const char* argv0) {
  std::fprintf(
      stderr,
      "usage: %s [--config PATH] [--dense-ingest-listen HOST:PORT]\n"
      "          [--backend rt_core]\n"
      "          [--plan-store-dir DIR]\n"
      "          [--ready-file PATH]\n"
      "          [--nats-url nats://HOST:PORT] [--robot-cell NAME]\n"
      "          [--robot-command-subject SUBJ]\n"
      "          [--status-publish-subject SUBJ]\n"
      "          store $HOME/.rosie/motion-server/joint-trajectory/v1/dense-plans\n"
      "Limits, timings, and addresses come from the JSON config\n"
      "(config/joint_trajectory_daemon.config.json is canonical; also via\n"
      "JOINT_TRAJECTORY_DAEMON_CONFIG); CLI flags override the file.\n",
      argv0);
}

bool split_host_port(const std::string& value, std::string* host,
                     std::uint16_t* port) {
  const std::size_t colon = value.rfind(':');
  if (colon == std::string::npos) return false;
  *host = value.substr(0, colon);
  *port = std::uint16_t(std::strtoul(value.c_str() + colon + 1, nullptr, 10));
  return !host->empty();
}

}  // namespace

int main(int argc, char** argv) {
  using namespace dense_joint_trajectory;

  IngestOptions ingest_options;
  NatsAddress nats;
  std::string robot_cell, robot_command_subject;
  std::string nats_url;
  std::string status_publish_subject;
  // Config file first (CLI flags override it): --config beats the env var.
  DaemonConfig config;
  std::string config_error;
  if (!load_daemon_config_arguments(argc, argv, &config, &config_error)) {
    std::fprintf(stderr, "config: %s\n", config_error.c_str());
    return 2;
  }
  if (!split_host_port(config.ingest_listen, &ingest_options.listen_host,
                       &ingest_options.listen_port)) {
    std::fprintf(stderr, "reason=ingest_address_invalid\n");
    return 2;
  }
  const char* home = std::getenv("HOME");
  ingest_options.store_dir =
      !config.plan_store_dir.empty()
          ? config.plan_store_dir
          : std::string(home != nullptr ? home : ".") +
                "/.rosie/motion-server/joint-trajectory/v1/dense-plans";

  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    const auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        usage(argv[0]);
        std::exit(2);
      }
      return argv[++i];
    };
    if (argument == "--dense-ingest-listen") {
      if (!split_host_port(next(), &ingest_options.listen_host,
                           &ingest_options.listen_port)) {
        std::fprintf(stderr, "--dense-ingest-listen needs HOST:PORT\n");
        return 2;
      }
    } else if (argument == "--config") {
      next();  // handled in the pre-scan
    } else if (argument == "--backend") {
      config.backend = next();
      if (config.backend != "rt_core") {
        std::fprintf(stderr, "reason=backend_retired\n");
        return 2;
      }
    } else if (argument == "--plan-store-dir") {
      ingest_options.store_dir = next();
    } else if (argument == "--ready-file") {
      ingest_options.ready_file = next();
    } else if (argument == "--joint-target" || argument == "--rtcore-status-subject") {
      std::fprintf(stderr, "reason=option_retired option=%s\n", argument.c_str());
      return 2;
    } else if (argument == "--nats-url") {
      nats_url = next();
    } else if (argument == "--robot-cell") {
      robot_cell = next();
    } else if (argument == "--robot-command-subject") {
      robot_command_subject = next();
    } else if (argument == "--status-publish-subject") {
      status_publish_subject = next();
    } else if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument %s\n", argument.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  if (!native_binding_configured(config)) {
    std::fprintf(stderr, "reason=native_binding_required\n");
    return 2;
  }
  if (!nats_url.empty()) {
    nats = parse_nats_url(nats_url);
    if (!nats.valid) {
      std::fprintf(stderr, "--nats-url %s is not usable\n", nats_url.c_str());
      return 2;
    }
  }

  // Lease nonce: process-start wall time, so restarted daemons can never
  // mint a lease id an old client still holds.
  LeaderAuthority leader(std::uint64_t(wall_ns() / 1000),
                         config.lease_ttl_min_ms, config.lease_ttl_max_ms);
  auto executor = std::make_unique<RtControlExecutor>(config, &leader);
  if (!executor->start()) {
    std::fprintf(stderr, "native executor start failed\n");
    return 1;
  }

  IngestServer ingest(ingest_options);
  ingest.on_preload_bytes = [&executor](const Plan& plan, const std::vector<unsigned char>& bytes,
                                       const std::string& path) {
    return executor->preload(plan, bytes, path);
  };
  ingest.executor_status_json = [&executor] {
    return executor_status_to_json(executor->status());
  };
  if (!ingest.start()) {
    return 1;
  }

  CommandDispatcher dispatcher(robot_cell, nats,
                               &leader, executor.get());
  // Separate owned workers keep authority revocation independent of commissioning.
  // Renew and Stop remain on the subscriber; shutdown joins both workers.
  std::mutex commands_mutex;
  std::condition_variable commands_changed;
  std::deque<std::pair<std::string, std::string>> pending_commands;
  std::deque<std::pair<std::string, std::string>> pending_authority;
  bool commands_running = true;
  auto run_commands = [&](bool authority) {
    auto& pending = authority ? pending_authority : pending_commands;
    for (;;) {
      std::pair<std::string, std::string> command;
      {
        std::unique_lock<std::mutex> lock(commands_mutex);
        commands_changed.wait(lock, [&] { return !commands_running || !pending.empty(); });
        if (!commands_running) return;
        command = std::move(pending.front());
        pending.pop_front();
      }
      dispatcher.handle(command.first, command.second);
    }
  };
  std::thread commands_worker(run_commands, false);
  std::thread authority_worker(run_commands, true);
  std::unique_ptr<NatsSubscriber> command_subscriber;
  if (nats.valid) {
    if (!robot_command_subject.empty()) {
      command_subscriber = std::make_unique<NatsSubscriber>(
          nats, robot_command_subject, 71,
          "joint_trajectory_daemon-command-sub",
          [&](const std::string&, const std::string& reply_to,
                        const std::string& payload) {
            JsonValue root;
            JsonParser parser(payload.data(), payload.size());
            std::string command;
            if (parser.parse(&root)) json_str(root.find("command"), &command);
            if (command == "play" || command == "leader_acquire" || command == "leader_release") {
              std::lock_guard<std::mutex> lock(commands_mutex);
              auto& pending = command == "play" ? pending_commands : pending_authority;
              // Unverified pending command ceiling: 256; overflow cannot authorize motion.
              if (pending.size() < 256) {
                pending.emplace_back(payload, reply_to);
                commands_changed.notify_all();
              } else dispatcher.refuse_queued(payload, reply_to);
              return;
            }
            dispatcher.handle(payload, reply_to);
          });
      command_subscriber->start();
    }

  }

  // The status stream. The OLP is a round trip away from the cell, so it
  // reads the pose from this rather than paying two of them per frame; see
  // status.publish_hz in the config for the measurement.
  std::unique_ptr<NatsTelemetryPublisher> status_publisher;
  if (nats.valid && !status_publish_subject.empty()) {
    const int period_ms =
        int(1000.0 / (config.status_publish_hz > 0 ? config.status_publish_hz : 20.0));
    status_publisher = std::make_unique<NatsTelemetryPublisher>(
        nats, status_publish_subject, period_ms,
        "joint_trajectory_daemon-status-pub",
        [&ingest] { return ingest.status_json(); });
    status_publisher->start();
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  std::printf(
      "joint_trajectory_daemon: ingest %s:%u nats %s store=%s"
      " status-stream=%s\n",
      ingest_options.listen_host.c_str(), unsigned(ingest.port()),
      nats.valid ? nats_url.c_str() : "(disabled)",
      ingest_options.store_dir.c_str(),
      status_publisher ? status_publish_subject.c_str() : "(off)");
  std::fflush(stdout);
  while (keep_running.load()) {
    if (nats.valid) {
      const auto subject = config.status_subject.empty()
          ? robot_command_subject + ".status" : config.status_subject;
      nats_publish(nats, subject, executor_status_to_json(executor->status()),
                   "joint_trajectory_daemon-status");
    }
    struct timespec interval = {0, 200 * 1000 * 1000};
    nanosleep(&interval, nullptr);
  }
  if (status_publisher) status_publisher->stop();
  if (command_subscriber) command_subscriber->stop();
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    commands_running = false;
    pending_commands.clear();
    pending_authority.clear();
  }
  commands_changed.notify_all();
  executor->stop("daemon_shutdown");
  commands_worker.join();
  authority_worker.join();
  ingest.stop();
  executor->stop_thread();
  std::printf("joint_trajectory_daemon: stopped\n");
  return 0;
}
