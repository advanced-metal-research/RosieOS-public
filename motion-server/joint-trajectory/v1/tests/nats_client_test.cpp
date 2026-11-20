// The status stream's transport. NatsTelemetryPublisher is what makes a real
// pose visible at the viewer's rate from a cell a round trip away: a polled
// status pays two RTTs per frame (148 ms each to bench-relay, measured
// 2026-09-07), a stream pays none. This test stands a fake NATS server on
// loopback and asserts the three properties the daemon depends on -- the
// handshake happens once and frames follow at the configured period, each
// frame carries exactly what the producer returned, and a server PING is
// answered so the broker does not drop a publisher that never asks anything.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nats_client.hpp"

namespace {

using dense_joint_trajectory::NatsAddress;
using dense_joint_trajectory::NatsTelemetryPublisher;

int failures = 0;

void expect(bool condition, const std::string& what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
  }
}

// A NATS server that only does what a publisher needs: accept, read the
// protocol, and (once asked) send a PING to see whether the client answers.
class FakeBroker {
 public:
  bool start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;
    const int enable = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
        ::listen(listen_fd_, 4) != 0) {
      return false;
    }
    socklen_t length = sizeof(address);
    if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address), &length) != 0) {
      return false;
    }
    port_ = ntohs(address.sin_port);
    running_.store(true);
    thread_ = std::thread([this] { run(); });
    return true;
  }

  void stop() {
    if (!running_.exchange(false)) return;
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    if (thread_.joinable()) thread_.join();
  }

  std::uint16_t port() const { return port_; }
  // Everything the client has sent, protocol lines and payloads alike.
  std::string transcript() {
    std::lock_guard<std::mutex> lock(mutex_);
    return transcript_;
  }
  void send_ping() { want_ping_.store(true); }

 private:
  void run() {
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) return;
    timeval timeout{0, 50 * 1000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    char chunk[4096];
    while (running_.load()) {
      if (want_ping_.exchange(false)) {
        const std::string ping = "PING\r\n";
        if (::send(fd, ping.data(), ping.size(), MSG_NOSIGNAL) < 0) break;
      }
      const ssize_t received = ::recv(fd, chunk, sizeof(chunk), 0);
      if (received < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        break;
      }
      if (received == 0) break;
      std::lock_guard<std::mutex> lock(mutex_);
      transcript_.append(chunk, std::size_t(received));
    }
    ::close(fd);
  }

  int listen_fd_ = -1;
  std::uint16_t port_ = 0;
  std::atomic<bool> running_{false};
  std::atomic<bool> want_ping_{false};
  std::mutex mutex_;
  std::string transcript_;
  std::thread thread_;
};

std::size_t count(const std::string& haystack, const std::string& needle) {
  std::size_t found = 0;
  for (std::size_t at = haystack.find(needle); at != std::string::npos;
       at = haystack.find(needle, at + needle.size())) {
    ++found;
  }
  return found;
}

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

int main() {
  FakeBroker broker;
  if (!broker.start()) {
    std::fprintf(stderr, "FAIL: cannot bind the fake broker\n");
    return 1;
  }

  NatsAddress address;
  address.host = "127.0.0.1";
  address.port = broker.port();
  address.valid = true;

  std::atomic<int> produced{0};
  NatsTelemetryPublisher publisher(address, "robot/v4/robot.test-cell.status", 40,
                                   "nats-client-test", [&produced] {
                                     return "{\"tick\":" +
                                            std::to_string(produced.fetch_add(1)) + "}";
                                   });
  publisher.start();
  sleep_ms(300);
  broker.send_ping();
  sleep_ms(200);
  publisher.stop();

  const std::string transcript = broker.transcript();
  broker.stop();

  expect(count(transcript, "CONNECT ") == 1,
         "the connection is opened once, not per frame");
  // 500 ms at a 40 ms period is 12 frames; the loop's 20 ms sleep steps and
  // the scheduler make the exact count unstable, so the assertion is that a
  // stream happened at roughly the asked rate, not that it was perfect.
  const std::size_t frames = count(transcript, "PUB robot/v4/robot.test-cell.status ");
  expect(frames >= 6 && frames <= 16,
         "about a frame per period, got " + std::to_string(frames));
  expect(publisher.published() == frames,
         "published() counts what the broker received");
  expect(transcript.find("{\"tick\":0}\r\n") != std::string::npos,
         "the first frame carries the producer's payload verbatim");
  expect(transcript.find("PONG\r\n") != std::string::npos,
         "a server PING is answered, so the broker keeps the publisher");

  if (failures == 0) std::printf("nats_client_test: ok\n");
  return failures == 0 ? 0 : 1;
}
