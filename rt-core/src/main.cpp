#include "engine/cycle_machine.hpp"

namespace {
#include "engine/commissioning_worker.hpp"
#include "engine/bulk_worker.hpp"
#include "engine/helper_worker.hpp"
#include "engine/status_publisher.hpp"

} // namespace

int MachineState::run() {
  if (!initialize_housekeeping()) return 2;
  request_port = &accept_controls;
  const auto& adapter_uid = parsed.adapter_uid;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // Lock memory to avoid page faults in the RT loop.
  // In production, systemd sets LimitMEMLOCK=infinity.
#if !defined(ROSIE_RT_SIMULATION) && !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    logf("ERROR: live runtime requires locked memory: %s", std::strerror(errno));
    return 2;
  }
#endif

  ssize_t random_bytes;
  do { random_bytes = getrandom(daemon_incarnation.data(), daemon_incarnation.size(), 0); }
  while (random_bytes < 0 && errno == EINTR);
  if (random_bytes != static_cast<ssize_t>(daemon_incarnation.size())) {
    logf("ERROR: unable to establish daemon incarnation");
    return 1;
  }

  const auto server = create_ipc_server(opt, axis_profiles);
  if (server.exit_code) return server.exit_code;
  const int server_fd = server.fd;
  const gid_t ipc_gid = server.gid;
  if (!telemetry.create((std::filesystem::path(opt.socket_path).parent_path() / "telemetry.ring").string(),
                        opt.cycle_ns, opt.telemetry_retention_ms, opt.num_axes, ipc_gid, daemon_incarnation)) {
    logf("ERROR: cannot create prefaulted telemetry ring: %s", std::strerror(errno));
    close(server_fd);
    unlink(opt.socket_path.c_str());
    return 2;
  }

  std::thread rt_thread([this]() { run_cycle(); });

  start_commissioning_worker();
  std::thread telemetry_thread([this]() { run_telemetry_writer(); });
  std::thread metrics_thread([this, ipc_gid]() { publish_status(ipc_gid); });

  ConnectionState connection;
  auto& controlling_client_fd = connection.controlling_client_fd;
  auto& cmd_shm = connection.cmd_shm;
  auto& status_shm = connection.status_shm;
  auto& helper_running = connection.helper_running;
  auto& helper_thread = connection.helper_thread;

  auto report_connection_failure = [&]() {
    return connection.report_connection_failure(connection_failure);
  };
  auto reset_connection = [&]() {
    connection.reset(fast_handoff, connection_failure, [this]() { disconnect_outputs(); });
    join_connection_workers();
    if (!g_stop.load()) start_commissioning_worker();
  };

  uint64_t stop_reply_sent = 0;
  // Main accept loop.
  while (!g_stop.load(std::memory_order_relaxed)) {
    telemetry.serve_readers();
    if (report_connection_failure()) reset_connection();
    // Heartbeats use the control socket, independently of the command helper.
    // A blocking mailbox/homing transaction cannot prevent lease expiration.
    if (controlling_client_fd >= 0) {
      char heartbeat[16];
      const ssize_t received = recv(controlling_client_fd, heartbeat, sizeof(heartbeat), MSG_DONTWAIT | MSG_TRUNC);
      if (received == 8 && std::memcmp(heartbeat, "RTHEART1", 8) == 0) {
        control_policy.heartbeat_ns.store(now_monotonic_ns(), std::memory_order_release);
      } else if (received == 8 && std::memcmp(heartbeat, "RTDISARM", 8) == 0) {
        revoke_outputs(true);
      } else if (received == 16 && std::memcmp(heartbeat, "RTSTOP02", 8) == 0) {
        uint64_t ticket = 0; std::memcpy(&ticket, heartbeat + 8, sizeof(ticket));
        revoke_outputs(true);
        stop_request_id.store(ticket, std::memory_order_release);
      } else if (received >= 0) {
        reset_connection();
      }
    }
    if (controlling_client_fd >= 0) {
      const auto ticket = stop_ack_id.load(std::memory_order_acquire);
      if (ticket && ticket != stop_reply_sent) {
        char reply[16]; std::memcpy(reply, "RTSTOPAK", 8); std::memcpy(reply + 8, &ticket, 8);
        if (send(controlling_client_fd, reply, sizeof(reply), MSG_DONTWAIT | MSG_NOSIGNAL) == sizeof(reply))
          stop_reply_sent = ticket;
      }
    }
    // If a controller was connected previously, detect disconnect and free the slot.
    // Without this, a client exiting cleanly can still leave us thinking a controller
    // is connected (because we don't continuously read from the socket).
    if (controlling_client_fd >= 0) {
      pollfd cfd{};
      cfd.fd = controlling_client_fd;
      cfd.events = POLLIN;
#ifdef POLLRDHUP
      cfd.events |= POLLRDHUP;
#endif
      const int cpr = poll(&cfd, 1, 0);
      if (cpr > 0) {
        short hang_mask = static_cast<short>(POLLHUP | POLLERR);
#ifdef POLLRDHUP
        hang_mask = static_cast<short>(hang_mask | POLLRDHUP);
#endif
        if ((cfd.revents & hang_mask) != 0) {
          logf("Controller disconnected; freeing IPC slot.");
          reset_connection();
        }
      }
    }

    const auto accepted = connection.accept_controller(server_fd, adapter_uid, [this](bool adapter_connection) {
      control_policy.application_required.store(adapter_connection, std::memory_order_release);
      control_policy.heartbeat_ns.store(now_monotonic_ns());
      return request_control({ControlOperation::Connect}).accepted;
    });
    if (accepted == AcceptResult::PollFailed) break;
    if (accepted != AcceptResult::Connected) continue;

    if (!connection.initialize(opt, daemon_incarnation, configured_mask, fast_handoff)) {
      reset_connection();
      continue;
    }

    // Start helper thread for status publishing + command ring draining.
    helper_running.store(true, std::memory_order_relaxed);
    bulk_running.store(true);
    bulk_thread = std::thread([this]() { run_bulk_worker(); });
    helper_thread = std::thread([this, &connection]() { run_helper(connection); });

    logf("IPC handshake complete (cmd_shm=%zu bytes, status_shm=%zu bytes)",
         cmd_shm.bytes, status_shm.bytes);
  }

  // Shutdown. Workers join while the cyclic thread still exchanges disabled
  // frames; it ends only after they exited and it owns the mailbox lock.
  g_stop.store(true, std::memory_order_relaxed);
  logf("shutdown: stop requested; joining connection and commissioning workers mailbox_in_flight=%u",
       mailbox_calls_in_flight.load(std::memory_order_acquire));
  reset_connection();
  logf("shutdown: workers joined; waiting for the cyclic thread to release the bus");

  g_stop.store(true, std::memory_order_relaxed);
  if (rt_thread.joinable()) {
    rt_thread.join();
  }
  logf("shutdown: cyclic thread joined; EtherCAT master released");
  if (metrics_thread.joinable()) {
    metrics_thread.join();
  }
  telemetry_writer_stop.store(true, std::memory_order_release);
  (void)telemetry_dump_wake.post();
  telemetry_thread.join();
  finish_telemetry();

  unlink(opt.socket_path.c_str());
  close(server_fd);

  logf("Stopped.");
  return process_exit_code.load(std::memory_order_relaxed);
}

int main(int argc, char** argv) {
  Options opt;
  auto parsed = parse_options(argc, argv, &opt);
  if (parsed.show_usage) print_usage(argv[0]);
  if (parsed.exit_code) return *parsed.exit_code;
  auto machine = std::make_unique<MachineState>(opt, parsed);
  return machine->run();
}
