// The listener belongs to the accept thread until it has joined. Pausing
// the actual accept call makes premature close/descriptor reuse observable
// without relying on ThreadSanitizer scheduling or private member access.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "dense_joint_trajectory_format.hpp"

namespace {

std::mutex gate_mutex;
std::condition_variable gate_changed;
int pending_listener = -1;
bool release_accept = false;

}  // namespace

int ingest_test_accept(int fd, sockaddr* address, socklen_t* size) {
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    pending_listener = fd;
    gate_changed.notify_all();
    gate_changed.wait(lock, [] { return release_accept; });
  }
  return ::accept(fd, address, size);
}

#define accept ingest_test_accept
#include "dense_joint_trajectory_ingest.hpp"
#undef accept

int main() {
  dense_joint_trajectory::IngestOptions options;
  options.listen_host = "127.0.0.1";
  options.listen_port = 0;
  // No requests or ready file: this lifecycle test writes no plan store.
  dense_joint_trajectory::IngestServer server(options);
  if (!server.start()) {
    std::fprintf(stderr, "FAIL: lifecycle server starts\n");
    return 1;
  }
  // Make the listener readable, including when the implementation waits
  // in poll before accepting. EOF prevents an accepted probe from becoming
  // a stalled request after the gate is released.
  const int probe = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(server.port());
  ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  if (probe < 0 || ::connect(probe, reinterpret_cast<sockaddr*>(&address),
                             sizeof(address)) != 0) {
    std::fprintf(stderr, "FAIL: lifecycle probe connects\n");
    std::_Exit(1);
  }
  ::close(probe);
  int listener;
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    if (!gate_changed.wait_for(lock, std::chrono::seconds(2),
                               [] { return pending_listener >= 0; })) {
      std::fprintf(stderr, "FAIL: lifecycle accept thread enters syscall\n");
      std::_Exit(1);
    }
    listener = pending_listener;
  }
  std::atomic<bool> stop_started{false};
  std::atomic<bool> stop_done{false};
  std::thread stopper([&] {
    stop_started.store(true);
    server.stop();
    stop_done.store(true);
  });
  while (!stop_started.load()) std::this_thread::yield();
  // The parent closes immediately, while the fixed stop waits for the
  // gated accept to finish. Give the stopper time to reach either point.
  const auto observation_end =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  bool closed_before_join = false;
  do {
    closed_before_join = ::fcntl(listener, F_GETFD) < 0 && errno == EBADF;
    if (closed_before_join) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < observation_end);
  const bool stopped_before_accept_returned = stop_done.load();
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_accept = true;
    gate_changed.notify_all();
  }
  const auto completion_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!stop_done.load() && std::chrono::steady_clock::now() < completion_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!stop_done.load()) {
    std::fprintf(stderr, "FAIL: shutdown does not unblock accept/stop\n");
    std::_Exit(1);
  }
  stopper.join();
  bool failed = false;
  if (closed_before_join) {
    std::fprintf(stderr, "FAIL: listener closed while accept still owns its descriptor\n");
    failed = true;
  }
  if (stopped_before_accept_returned) {
    std::fprintf(stderr, "FAIL: stop returns before accept thread quiesces\n");
    failed = true;
  }
  if (::fcntl(listener, F_GETFD) >= 0 || errno != EBADF) {
    std::fprintf(stderr, "FAIL: stop leaks the listener after joining\n");
    failed = true;
  }
  // Idempotent stop must preserve unrelated descriptors allocated later.
  const int reused = ::socket(AF_INET, SOCK_STREAM, 0);
  server.stop();
  if (reused < 0 || ::fcntl(reused, F_GETFD) < 0) {
    std::fprintf(stderr, "FAIL: repeated stop preserves unrelated descriptors\n");
    failed = true;
  }
  if (reused >= 0) ::close(reused);
  if (failed) return 1;
  std::printf("dense_joint_trajectory_ingest_lifecycle_test: ok\n");
  return 0;
}
