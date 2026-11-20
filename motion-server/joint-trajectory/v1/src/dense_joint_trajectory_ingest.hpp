#pragma once

// TCP ingest for the dense trajectory daemon: the data plane that exists so
// multi-MB programs never have to fit through a 1 MiB NATS payload.
//
// Framing is the collision-server pattern: [u64 BE payload_size][payload],
// 64 MiB cap, one request frame, one response frame, connection closed.
// Payload shape: [u32 BE json_len][control JSON][optional binary body].
// Kinds: "upload" (persist a validated blob -- inert, never causes motion),
// "validate" (dry-run), "preload" (stage a stored blob by reference after
// re-verifying it), "status". Motion authority is NOT here: play/stop stay
// on the NATS leader-lease path (phase 3), so the worst a compromised or
// buggy producer can do over this socket is store bytes that the play gate
// will still have to match exactly.

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
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

namespace dense_joint_trajectory {

// Shared with the tesseract collision transport and the Python client. A
// 250,000-sample plan is ~38 MB; 64 MiB leaves headroom without letting a
// runaway producer allocate unbounded memory on the robot.
inline constexpr std::uint64_t kMaxFrameBytes = 64ull * 1024 * 1024;

// One exchange must finish inside this; matches the v1 plan-services TCP
// client budget. A stalled peer must never wedge the accept loop.
inline constexpr int kSocketTimeoutSeconds = 240;

inline constexpr const char* kReadySchema =
    "robot-v4.dense-joint-trajectory-ingest-ready.v1";

// ---------------------------------------------------------------------------
// JSON emission (strings only need escaping; numbers are integral or
// round-trip doubles).

inline std::string json_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 2);
  for (const char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
          out += buffer;
        } else {
          out += c;
        }
    }
  }
  return out;
}

inline std::string json_string(const std::string& value) {
  return "\"" + json_escape(value) + "\"";
}

// ---------------------------------------------------------------------------
// Blocking socket IO with EINTR handling.

inline bool send_all(int fd, const void* data, std::size_t size) {
  const auto* cursor = static_cast<const unsigned char*>(data);
  while (size > 0) {
    const ssize_t sent = ::send(fd, cursor, size, MSG_NOSIGNAL);
    if (sent < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (sent == 0) return false;
    cursor += sent;
    size -= std::size_t(sent);
  }
  return true;
}

inline bool receive_all(int fd, void* data, std::size_t size) {
  auto* cursor = static_cast<unsigned char*>(data);
  while (size > 0) {
    const ssize_t received = ::recv(fd, cursor, size, 0);
    if (received < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    if (received == 0) return false;
    cursor += received;
    size -= std::size_t(received);
  }
  return true;
}

inline bool send_frame(int fd, const std::vector<unsigned char>& payload) {
  unsigned char size_raw[8];
  const std::uint64_t size = payload.size();
  for (int i = 0; i < 8; ++i) {
    size_raw[i] = static_cast<unsigned char>((size >> (8 * (7 - i))) & 0xffu);
  }
  return send_all(fd, size_raw, sizeof(size_raw)) &&
         send_all(fd, payload.data(), payload.size());
}

inline bool receive_frame(int fd, std::vector<unsigned char>* payload) {
  unsigned char size_raw[8];
  if (!receive_all(fd, size_raw, sizeof(size_raw))) return false;
  std::uint64_t size = 0;
  for (int i = 0; i < 8; ++i) size = (size << 8u) | size_raw[i];
  if (size > kMaxFrameBytes) return false;
  payload->resize(std::size_t(size));
  return size == 0 || receive_all(fd, payload->data(), payload->size());
}

// ---------------------------------------------------------------------------
// Plan store: uploaded blobs at <dir>/<digest-hex>.rdt, written atomically
// (tmp + rename) so a crash mid-write can never leave a half blob that a
// later preload would then reject confusingly.

class PlanStore {
 public:
  explicit PlanStore(std::string dir) : dir_(std::move(dir)) {}

  const std::string& dir() const { return dir_; }

  static std::string digest_hex(const std::string& digest) {
    // "sha256:<64 hex>" -> "<64 hex>"; refuse anything else so a digest can
    // never smuggle path separators into a filename.
    if (digest.size() != 71 || digest.rfind("sha256:", 0) != 0) return "";
    for (std::size_t i = 7; i < digest.size(); ++i) {
      const char c = digest[i];
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return "";
    }
    return digest.substr(7);
  }

  std::string path_for(const std::string& digest) const {
    const std::string hex = digest_hex(digest);
    if (hex.empty()) return "";
    return dir_ + "/" + hex + ".rdt";
  }

  bool save(const std::string& digest, const std::vector<unsigned char>& blob) {
    const std::string path = path_for(digest);
    if (path.empty()) return false;
    ::mkdir(dir_.c_str(), 0755);
    const std::string tmp = path + ".tmp";
    {
      std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
      if (!out) return false;
      out.write(reinterpret_cast<const char*>(blob.data()),
                std::streamsize(blob.size()));
      if (!out) {
        ::unlink(tmp.c_str());
        return false;
      }
    }
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
      ::unlink(tmp.c_str());
      return false;
    }
    return true;
  }

  std::optional<std::vector<unsigned char>> load(const std::string& digest) const {
    const std::string path = path_for(digest);
    if (path.empty()) return std::nullopt;
    std::ifstream input(path, std::ios::binary);
    if (!input) return std::nullopt;
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const std::string& data = buffer.str();
    return std::vector<unsigned char>(data.begin(), data.end());
  }

 private:
  std::string dir_;
};

// ---------------------------------------------------------------------------
// Ingest server.

struct IngestOptions {
  std::string listen_host = "127.0.0.1";
  std::uint16_t listen_port = 8797;  // 8794-8796 are taken by the OLP stack.
  std::string store_dir;
  std::string ready_file;  // empty = don't write one
};

class IngestServer {
 public:
  IngestServer(IngestOptions options)
      : options_(std::move(options)), store_(options_.store_dir) {}

  // Phase-3 wiring: offers each verified preload to the executor, which
  // stages its own copy or refuses (playback in progress). A refusal is the
  // client's answer and the ingest-side staged copy is left as it was, so
  // `status` keeps describing the plan actually in flight. Also lets the
  // executor speak for itself in the TCP status document. Both optional;
  // unset means phase-2 behavior.
  std::function<Rejection(const Plan&)> on_preload;
  std::function<Rejection(const Plan&, const std::vector<unsigned char>&,
                          const std::string&)> on_preload_bytes;
  std::function<std::string()> executor_status_json;

  // The status document, as JSON. `status` over TCP answers with it and the
  // telemetry publisher streams the same bytes, so a polled reply and a
  // streamed frame are never two different shapes to keep in step.
  std::string status_json() {
    const StagedSummary staged = staged_summary();
    std::ostringstream out;
    out << "{\"ok\":true,\"kind\":\"status\""
        << ",\"schema\":" << json_string(kSchema)
        << ",\"store_dir\":" << json_string(store_.dir())
        << ",\"staged\":";
    if (staged.present) {
      out << "{\"trajectory_digest\":" << json_string(staged.trajectory_digest)
          << ",\"plan_id\":" << json_string(staged.plan_id)
          << ",\"total_sample_count\":" << staged.total_sample_count << "}";
    } else {
      out << "null";
    }
    out << ",\"executor\":"
        << (executor_status_json ? executor_status_json() : "\"absent\"");
    {
      std::lock_guard<std::mutex> lock(staged_mutex_);
      out << ",\"last_error\":";
      if (last_error_reason_.empty()) {
        out << "null";
      } else {
        out << "{\"reason\":" << json_string(last_error_reason_)
            << ",\"detail\":" << json_string(last_error_detail_) << "}";
      }
    }
    out << "}";
    return out.str();
  }

  ~IngestServer() { stop(); }

  // Bind, write the ready file, and start the accept loop. Returns false
  // with a message on stderr if the socket cannot be bound.
  bool start() {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      std::perror("dense-joint-trajectory ingest: socket");
      return false;
    }
    const int flags = ::fcntl(listen_fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(listen_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      std::perror("dense-joint-trajectory ingest: nonblocking listener");
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    const int enable = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options_.listen_port);
    if (::inet_pton(AF_INET, options_.listen_host.c_str(), &address.sin_addr) != 1) {
      std::fprintf(stderr, "dense-joint-trajectory ingest: bad listen host %s\n",
                   options_.listen_host.c_str());
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) != 0 ||
        ::listen(listen_fd_, 8) != 0) {
      std::perror("dense-joint-trajectory ingest: bind/listen");
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }
    sockaddr_in bound{};
    socklen_t bound_len = sizeof(bound);
    if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&bound),
                      &bound_len) == 0) {
      bound_port_ = ntohs(bound.sin_port);
    }
    if (!options_.ready_file.empty()) {
      write_ready_file();
    }
    running_.store(true);
    accept_thread_ = std::thread([this] { accept_loop(); });
    return true;
  }

  void stop() {
    if (!running_.exchange(false)) {
      return;
    }
    // Wake the listener where supported; poll also bounds the idle wait
    // on platforms whose shutdown() does not wake a listening socket.
    ::shutdown(listen_fd_, SHUT_RDWR);
    // Keep both the member value and the OS descriptor alive until the
    // accept thread exits; closing first permits descriptor reuse while
    // that thread is still about to call accept().
    if (accept_thread_.joinable()) accept_thread_.join();
    ::close(listen_fd_);
    listen_fd_ = -1;
  }

  std::uint16_t port() const { return bound_port_; }

  struct StagedSummary {
    bool present = false;
    std::string trajectory_digest;
    std::string plan_id;
    std::size_t total_sample_count = 0;
  };

  StagedSummary staged_summary() const {
    std::lock_guard<std::mutex> lock(staged_mutex_);
    StagedSummary summary;
    summary.present = staged_present_;
    if (staged_present_) {
      summary.trajectory_digest = staged_.header.trajectory_digest;
      summary.plan_id = staged_.header.plan_id;
      summary.total_sample_count = staged_.header.total_sample_count;
    }
    return summary;
  }

  // Exposed for the executor (phase 3) and for tests: a copy of the staged
  // plan, or nullopt when nothing is staged.
  std::optional<Plan> staged_plan() const {
    std::lock_guard<std::mutex> lock(staged_mutex_);
    if (!staged_present_) return std::nullopt;
    return staged_;
  }

  // Request dispatch, exposed so tests can drive the protocol without a
  // socket as well as through one.
  std::vector<unsigned char> handle_request(const std::vector<unsigned char>& payload) {
    if (payload.size() < 4) {
      return error_response("request_invalid", -1, "payload shorter than json_len");
    }
    const std::size_t json_len = (std::size_t(payload[0]) << 24u) |
                                 (std::size_t(payload[1]) << 16u) |
                                 (std::size_t(payload[2]) << 8u) |
                                 std::size_t(payload[3]);
    if (4 + json_len > payload.size()) {
      return error_response("request_invalid", -1, "json_len exceeds payload");
    }
    JsonValue control;
    JsonParser parser(reinterpret_cast<const char*>(payload.data() + 4), json_len);
    if (!parser.parse(&control) || control.kind != JsonValue::Kind::Object) {
      return error_response("request_invalid", -1, "control JSON parse failed");
    }
    std::string kind;
    if (!json_str(control.find("kind"), &kind)) {
      return error_response("request_invalid", -1, "missing kind");
    }
    const std::vector<unsigned char> body(payload.begin() + 4 + json_len,
                                          payload.end());
    if (kind == "upload") return handle_upload(body, /*persist=*/true);
    if (kind == "validate") return handle_upload(body, /*persist=*/false);
    if (kind == "preload") return handle_preload(control);
    if (kind == "status") return handle_status();
    return error_response("request_invalid", -1, "unknown kind " + kind);
  }

 private:
  void write_ready_file() const {
    std::ostringstream out;
    out << "{\"schema\":" << json_string(kReadySchema)
        << ",\"host\":" << json_string(options_.listen_host)
        << ",\"port\":" << bound_port_ << "}\n";
    const std::string tmp = options_.ready_file + ".tmp";
    {
      std::ofstream file(tmp, std::ios::trunc);
      file << out.str();
    }
    ::rename(tmp.c_str(), options_.ready_file.c_str());
  }

  void accept_loop() {
    while (running_.load()) {
      pollfd listener{listen_fd_, POLLIN, 0};
      const int ready = ::poll(&listener, 1, 100);
      if (!running_.load()) break;
      if (ready == 0) continue;
      if (ready < 0) {
        if (errno == EINTR) continue;
        break;
      }
      if ((listener.revents & POLLIN) == 0) break;
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
        break;
      }
      // BSD may inherit O_NONBLOCK from the listener; requests retain the
      // existing blocking framing and per-connection timeout on all hosts.
      const int flags = ::fcntl(fd, F_GETFL, 0);
      if (flags < 0 || ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0) {
        ::close(fd);
        continue;
      }
      timeval timeout{};
      timeout.tv_sec = kSocketTimeoutSeconds;
      ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
      std::vector<unsigned char> payload;
      if (receive_frame(fd, &payload)) {
        const std::vector<unsigned char> response = handle_request(payload);
        send_frame(fd, response);
      }
      ::close(fd);
    }
  }

  static std::vector<unsigned char> pack_response(const std::string& json) {
    std::vector<unsigned char> out;
    out.reserve(4 + json.size());
    const std::size_t size = json.size();
    out.push_back(static_cast<unsigned char>((size >> 24u) & 0xffu));
    out.push_back(static_cast<unsigned char>((size >> 16u) & 0xffu));
    out.push_back(static_cast<unsigned char>((size >> 8u) & 0xffu));
    out.push_back(static_cast<unsigned char>(size & 0xffu));
    out.insert(out.end(), json.begin(), json.end());
    return out;
  }

  static std::vector<unsigned char> error_response(const std::string& reason,
                                                   int segment_index,
                                                   const std::string& detail) {
    std::ostringstream out;
    out << "{\"ok\":false,\"reason\":" << json_string(reason);
    if (segment_index >= 0) {
      out << ",\"segment_index\":" << segment_index;
    }
    out << ",\"detail\":" << json_string(detail) << "}";
    return pack_response(out.str());
  }

  static std::vector<unsigned char> rejection_response(const Rejection& rejection) {
    return error_response(rejection.reason, rejection.segment_index,
                          rejection.detail);
  }

  std::vector<unsigned char> handle_upload(const std::vector<unsigned char>& body,
                                           bool persist) {
    Plan plan;
    const Rejection result = decode(body.data(), body.size(), &plan);
    if (!result.ok) {
      remember_error(result);
      return rejection_response(result);
    }
    if (persist && !store_.save(plan.header.trajectory_digest, body)) {
      const Rejection failure = Rejection::refuse(
          "store_write_failed", -1, "cannot persist to " + store_.dir());
      remember_error(failure);
      return rejection_response(failure);
    }
    std::ostringstream out;
    out << "{\"ok\":true,\"kind\":" << json_string(persist ? "upload" : "validate")
        << ",\"trajectory_digest\":" << json_string(plan.header.trajectory_digest)
        << ",\"segment_count\":" << plan.segments.size()
        << ",\"total_sample_count\":" << plan.header.total_sample_count
        << ",\"total_duration_s\":" << plan.header.total_duration_s << "}";
    return pack_response(out.str());
  }

  std::vector<unsigned char> handle_preload(const JsonValue& control) {
    std::string digest;
    if (!json_str(control.find("trajectory_digest"), &digest)) {
      return error_response("request_invalid", -1, "missing trajectory_digest");
    }
    const std::optional<std::vector<unsigned char>> blob = store_.load(digest);
    if (!blob.has_value()) {
      return error_response("plan_not_found", -1, digest + " not in store");
    }
    // Full re-verification against bytes at rest: a blob corrupted on disk
    // since upload must fail here, not at play.
    Plan plan;
    const Rejection result = decode(blob->data(), blob->size(), &plan);
    if (!result.ok) {
      remember_error(result);
      return rejection_response(result);
    }
    if (plan.header.trajectory_digest != digest) {
      return error_response("trajectory_digest_mismatch", -1,
                            "stored blob carries " + plan.header.trajectory_digest);
    }
    // Identity fencing: every named field must match the stored header
    // exactly, the same six-way rigor the play gate applies.
    const auto check_str = [&](const char* key, const std::string& actual)
        -> std::optional<std::string> {
      std::string requested;
      if (!json_str(control.find(key), &requested)) {
        return std::string("missing ") + key;
      }
      if (requested != actual) {
        return std::string(key) + " " + requested + " != " + actual;
      }
      return std::nullopt;
    };
    const auto check_u64 = [&](const char* key, std::uint64_t actual)
        -> std::optional<std::string> {
      std::uint64_t requested = 0;
      if (!json_u64(control.find(key), &requested)) {
        return std::string("missing ") + key;
      }
      if (requested != actual) {
        return std::string(key) + " " + std::to_string(requested) + " != " +
               std::to_string(actual);
      }
      return std::nullopt;
    };
    for (const auto& mismatch :
         {check_str("plan_id", plan.header.plan_id),
          check_str("program_id", plan.header.program_id),
          check_str("program_digest", plan.header.program_digest),
          check_u64("manifest_revision", plan.header.manifest_revision),
          check_u64("plan_revision", plan.header.plan_revision)}) {
      if (mismatch.has_value()) {
        return error_response("identity_mismatch", -1, *mismatch);
      }
    }
    if (on_preload || on_preload_bytes) {
      const Rejection accepted = on_preload_bytes
          ? on_preload_bytes(plan, *blob, store_.path_for(digest)) : on_preload(plan);
      if (!accepted.ok) {
        remember_error(accepted);
        return rejection_response(accepted);
      }
    }
    {
      std::lock_guard<std::mutex> lock(staged_mutex_);
      staged_ = plan;
      staged_present_ = true;
    }
    std::ostringstream out;
    out << "{\"ok\":true,\"kind\":\"preload\""
        << ",\"trajectory_digest\":" << json_string(digest)
        << ",\"total_sample_count\":" << staged_summary().total_sample_count << "}";
    return pack_response(out.str());
  }

  std::vector<unsigned char> handle_status() { return pack_response(status_json()); }

  void remember_error(const Rejection& rejection) {
    std::lock_guard<std::mutex> lock(staged_mutex_);
    last_error_reason_ = rejection.reason;
    last_error_detail_ = rejection.detail;
  }

  IngestOptions options_;
  PlanStore store_;
  int listen_fd_ = -1;
  std::uint16_t bound_port_ = 0;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  mutable std::mutex staged_mutex_;
  bool staged_present_ = false;
  Plan staged_;
  std::string last_error_reason_;
  std::string last_error_detail_;
};

}  // namespace dense_joint_trajectory
