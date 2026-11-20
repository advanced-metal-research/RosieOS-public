#pragma once

// Minimal hand-rolled NATS client, repo zero-dependency style, speaking the
// same subset cartesiand does: a one-shot connection per publish (CONNECT /
// PUB / PING, success == PONG), and a line-protocol subscriber loop with
// automatic reconnect. The server INFO line is deliberately ignored.
//
// Publishing fails CLOSED: no PONG inside the timeout means the publish did
// not happen. The weld-torch lease is built on that property -- an
// unconfirmed ON must never be assumed delivered, while the bridge-side
// 1 s lease expiry makes an unconfirmed OFF safe by construction.

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace dense_joint_trajectory {

struct NatsAddress {
  std::string host;
  std::uint16_t port = 4222;
  bool valid = false;
};

// Accepts "nats://host:port", "host:port", or "host" (default port 4222).
inline NatsAddress parse_nats_url(const std::string& url) {
  NatsAddress address;
  std::string rest = url;
  const std::string scheme = "nats://";
  if (rest.rfind(scheme, 0) == 0) rest = rest.substr(scheme.size());
  if (rest.empty()) return address;
  const std::size_t colon = rest.rfind(':');
  if (colon == std::string::npos) {
    address.host = rest;
  } else {
    address.host = rest.substr(0, colon);
    address.port = std::uint16_t(std::strtoul(rest.c_str() + colon + 1, nullptr, 10));
  }
  address.valid = !address.host.empty() && address.port != 0;
  return address;
}

namespace nats_detail {

inline int connect_tcp(const NatsAddress& address, int timeout_ms) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* results = nullptr;
  const std::string port = std::to_string(address.port);
  if (::getaddrinfo(address.host.c_str(), port.c_str(), &hints, &results) != 0) {
    return -1;
  }
  int fd = -1;
  for (addrinfo* entry = results; entry != nullptr; entry = entry->ai_next) {
    fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
    if (fd < 0) continue;
    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    if (::connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) break;
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(results);
  return fd;
}

inline bool send_text(int fd, const std::string& text) {
  std::size_t offset = 0;
  while (offset < text.size()) {
    const ssize_t sent =
        ::send(fd, text.data() + offset, text.size() - offset, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (sent == 0) return false;
    offset += std::size_t(sent);
  }
  return true;
}

}  // namespace nats_detail

// One-shot publish; blocks until PONG or timeout. Payloads >= 64 KiB get the
// long timeout, same as cartesiand's publisher.
inline bool nats_publish(const NatsAddress& address, const std::string& subject,
                         const std::string& payload,
                         const std::string& client_name) {
  if (!address.valid || subject.empty()) return false;
  const int timeout_ms = payload.size() >= 64 * 1024 ? 5000 : 400;
  const int fd = nats_detail::connect_tcp(address, timeout_ms);
  if (fd < 0) return false;
  std::string out;
  out += "CONNECT {\"verbose\":false,\"pedantic\":false,\"lang\":\"cpp\","
         "\"version\":\"" + client_name + "\"}\r\n";
  out += "PUB " + subject + " " + std::to_string(payload.size()) + "\r\n";
  out += payload;
  out += "\r\nPING\r\n";
  bool confirmed = false;
  if (nats_detail::send_text(fd, out)) {
    std::string buffer;
    char chunk[512];
    while (true) {
      const ssize_t received = ::recv(fd, chunk, sizeof(chunk), 0);
      if (received <= 0) {
        if (received < 0 && errno == EINTR) continue;
        break;
      }
      buffer.append(chunk, std::size_t(received));
      std::size_t line_start = 0;
      std::size_t line_end;
      bool done = false;
      while ((line_end = buffer.find("\r\n", line_start)) != std::string::npos) {
        const std::string line = buffer.substr(line_start, line_end - line_start);
        line_start = line_end + 2;
        if (line == "PONG") {
          confirmed = true;
          done = true;
          break;
        }
        if (line == "PING") {
          nats_detail::send_text(fd, "PONG\r\n");
        } else if (line.rfind("-ERR", 0) == 0) {
          done = true;
          break;
        }
      }
      buffer.erase(0, line_start);
      if (done) break;
    }
  }
  ::close(fd);
  return confirmed;
}

// Telemetry publisher: its own thread, one held connection, one payload per
// tick, fire-and-forget.
//
// Deliberately NOT the fail-closed nats_publish above. That one waits for a
// PONG because an unconfirmed weld-torch ON must never be assumed delivered.
// A status frame is the opposite: the next one is 50 ms away, so waiting a
// round trip per frame would cost more than a dropped frame does. On this
// bench the OLP workstation is 148 ms of RTT away from the cell (tailscale,
// direct, measured 2026-09-07), so a request/reply status poll costs two
// round trips -- 300 ms, ~3 Hz -- no matter how fast the browser asks. A
// stream turns that RTT into latency instead of a rate ceiling.
class NatsTelemetryPublisher {
 public:
  // Produces the payload for one tick. Called on the publisher thread, so it
  // must take whatever lock the status it reads needs.
  using Producer = std::function<std::string()>;

  NatsTelemetryPublisher(NatsAddress address, std::string subject,
                         int period_ms, std::string client_name,
                         Producer producer)
      : address_(std::move(address)),
        subject_(std::move(subject)),
        period_ms_(period_ms > 0 ? period_ms : 50),
        client_name_(std::move(client_name)),
        producer_(std::move(producer)) {}

  ~NatsTelemetryPublisher() { stop(); }

  void start() {
    if (!address_.valid || subject_.empty() || !producer_) return;
    running_.store(true);
    thread_ = std::thread([this] { run(); });
  }

  void stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
  }

  // Frames written since start; a test reads it to prove the loop ticks.
  std::uint64_t published() const { return published_.load(); }

 private:
  void run() {
    int fd = -1;
    while (running_.load()) {
      if (fd < 0) fd = open_connection();
      if (fd >= 0) {
        // A server PING left unanswered closes us after two intervals, so
        // the socket is drained every tick even though nothing is expected.
        if (!drain(fd) || !publish_once(fd)) {
          ::close(fd);
          fd = -1;
        }
      }
      wait(fd >= 0 ? period_ms_ : 1000);
    }
    if (fd >= 0) ::close(fd);
  }

  // Sleeps in short steps so stop() joins promptly even from the 1 s
  // reconnect wait.
  void wait(int ms) {
    for (int slept = 0; slept < ms && running_.load(); slept += 20) {
      sleep_ms(ms - slept < 20 ? ms - slept : 20);
    }
  }

  int open_connection() {
    const int fd = nats_detail::connect_tcp(address_, 1000);
    if (fd < 0) return -1;
    const std::string handshake =
        "CONNECT {\"verbose\":false,\"pedantic\":false,\"lang\":\"cpp\","
        "\"version\":\"" + client_name_ + "\"}\r\n";
    if (!nats_detail::send_text(fd, handshake)) {
      ::close(fd);
      return -1;
    }
    return fd;
  }

  bool publish_once(int fd) {
    const std::string payload = producer_();
    if (payload.empty()) return true;
    std::string out = "PUB " + subject_ + " " + std::to_string(payload.size()) +
                      "\r\n" + payload + "\r\n";
    if (!nats_detail::send_text(fd, out)) return false;
    published_.fetch_add(1);
    return true;
  }

  // Reads whatever the server has said without blocking. PING is answered;
  // -ERR or a closed socket ends the connection.
  bool drain(int fd) {
    char chunk[512];
    while (true) {
      const ssize_t received = ::recv(fd, chunk, sizeof(chunk), MSG_DONTWAIT);
      if (received < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
        return false;
      }
      if (received == 0) return false;
      buffer_.append(chunk, std::size_t(received));
      std::size_t line_end;
      while ((line_end = buffer_.find("\r\n")) != std::string::npos) {
        const std::string line = buffer_.substr(0, line_end);
        buffer_.erase(0, line_end + 2);
        if (line == "PING") {
          if (!nats_detail::send_text(fd, "PONG\r\n")) return false;
        } else if (line.rfind("-ERR", 0) == 0) {
          return false;
        }
      }
      if (buffer_.size() > 64 * 1024) buffer_.clear();
    }
  }

  static void sleep_ms(int ms) {
    struct timespec interval = {ms / 1000, (ms % 1000) * 1000 * 1000};
    nanosleep(&interval, nullptr);
  }

  NatsAddress address_;
  std::string subject_;
  int period_ms_;
  std::string client_name_;
  Producer producer_;
  std::string buffer_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> published_{0};
  std::thread thread_;
};

// Subscriber loop on its own thread: delivers every MSG on `subject` to the
// callback as (subject, reply_to, payload). Reconnects with a 1 s backoff
// until stop().
class NatsSubscriber {
 public:
  using Callback =
      std::function<void(const std::string& subject, const std::string& reply_to,
                         const std::string& payload)>;

  NatsSubscriber(NatsAddress address, std::string subject, int sid,
                 std::string client_name, Callback callback)
      : address_(std::move(address)),
        subject_(std::move(subject)),
        sid_(sid),
        client_name_(std::move(client_name)),
        callback_(std::move(callback)) {}

  ~NatsSubscriber() { stop(); }

  void start() {
    running_.store(true);
    thread_ = std::thread([this] { run(); });
  }

  void stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
  }

 private:
  void run() {
    while (running_.load()) {
      const int fd = nats_detail::connect_tcp(address_, 1000);
      if (fd < 0) {
        backoff();
        continue;
      }
      std::string handshake =
          "CONNECT {\"verbose\":false,\"pedantic\":false,\"lang\":\"cpp\","
          "\"version\":\"" + client_name_ + "\"}\r\n"
          "SUB " + subject_ + " " + std::to_string(sid_) + "\r\n";
      if (!nats_detail::send_text(fd, handshake)) {
        ::close(fd);
        backoff();
        continue;
      }
      read_loop(fd);
      ::close(fd);
      if (running_.load()) backoff();
    }
  }

  void read_loop(int fd) {
    std::string buffer;
    char chunk[4096];
    while (running_.load()) {
      const ssize_t received = ::recv(fd, chunk, sizeof(chunk), 0);
      if (received < 0) {
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // idle timeout
        return;
      }
      if (received == 0) return;
      buffer.append(chunk, std::size_t(received));
      std::size_t line_end;
      while ((line_end = buffer.find("\r\n")) != std::string::npos) {
        const std::string line = buffer.substr(0, line_end);
        if (line.rfind("MSG ", 0) == 0) {
          // MSG <subject> <sid> [reply-to] <#bytes>
          std::vector<std::string> tokens;
          std::size_t start = 4;
          while (start < line.size()) {
            std::size_t end = line.find(' ', start);
            if (end == std::string::npos) end = line.size();
            tokens.push_back(line.substr(start, end - start));
            start = end + 1;
          }
          if (tokens.size() != 3 && tokens.size() != 4) {
            buffer.erase(0, line_end + 2);
            continue;
          }
          const std::string reply_to = tokens.size() == 4 ? tokens[2] : "";
          const std::size_t bytes =
              std::size_t(std::strtoul(tokens.back().c_str(), nullptr, 10));
          // Wait until the payload + trailing CRLF is buffered.
          while (buffer.size() < line_end + 2 + bytes + 2 && running_.load()) {
            const ssize_t more = ::recv(fd, chunk, sizeof(chunk), 0);
            if (more < 0) {
              if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
              return;
            }
            if (more == 0) return;
            buffer.append(chunk, std::size_t(more));
          }
          const std::string payload = buffer.substr(line_end + 2, bytes);
          buffer.erase(0, line_end + 2 + bytes + 2);
          if (tokens[0] == subject_) {
            callback_(tokens[0], reply_to, payload);
          }
        } else {
          if (line == "PING") nats_detail::send_text(fd, "PONG\r\n");
          buffer.erase(0, line_end + 2);
        }
      }
    }
  }

  void backoff() {
    for (int i = 0; i < 10 && running_.load(); ++i) {
      struct timespec interval = {0, 100 * 1000 * 1000};
      nanosleep(&interval, nullptr);
    }
  }

  NatsAddress address_;
  std::string subject_;
  int sid_;
  std::string client_name_;
  Callback callback_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace dense_joint_trajectory
