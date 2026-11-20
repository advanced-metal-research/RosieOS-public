// Ingest protocol test: drives a real IngestServer over a loopback TCP
// socket with the reference fixture -- upload, validate, preload by
// reference (including across a server restart, proving the store is the
// durable thing), identity fencing, and per-segment corruption rejection.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "dense_joint_trajectory_format.hpp"
#include "dense_joint_trajectory_ingest.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
  }
}

std::vector<unsigned char> read_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string& data = buffer.str();
  return std::vector<unsigned char>(data.begin(), data.end());
}

// Minimal client for the exchange: connect, one request frame, one response
// frame. Mirrors the Python client in weld_programmer/export.
dense_joint_trajectory::JsonValue exchange(std::uint16_t port, const std::string& control,
                                     const std::vector<unsigned char>& body,
                                     bool fragmented = false) {
  dense_joint_trajectory::JsonValue parsed;
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  expect(fd >= 0, "client socket");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  expect(::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
         "client connect");
  std::vector<unsigned char> payload;
  payload.push_back(static_cast<unsigned char>((control.size() >> 24u) & 0xffu));
  payload.push_back(static_cast<unsigned char>((control.size() >> 16u) & 0xffu));
  payload.push_back(static_cast<unsigned char>((control.size() >> 8u) & 0xffu));
  payload.push_back(static_cast<unsigned char>(control.size() & 0xffu));
  payload.insert(payload.end(), control.begin(), control.end());
  payload.insert(payload.end(), body.begin(), body.end());
  if (fragmented) {
    unsigned char size_raw[8];
    for (int i = 0; i < 8; ++i) {
      size_raw[i] = static_cast<unsigned char>(
          (std::uint64_t(payload.size()) >> (8 * (7 - i))) & 0xffu);
    }
    expect(dense_joint_trajectory::send_all(fd, size_raw, sizeof(size_raw)),
           "fragmented client sends frame length");
    ::usleep(20000);
    expect(dense_joint_trajectory::send_all(fd, payload.data(), payload.size()),
           "fragmented client sends delayed frame body");
  } else {
    expect(dense_joint_trajectory::send_frame(fd, payload), "client send");
  }
  std::vector<unsigned char> response;
  expect(dense_joint_trajectory::receive_frame(fd, &response), "client receive");
  ::close(fd);
  if (response.size() >= 4) {
    const std::size_t json_len = (std::size_t(response[0]) << 24u) |
                                 (std::size_t(response[1]) << 16u) |
                                 (std::size_t(response[2]) << 8u) |
                                 std::size_t(response[3]);
    dense_joint_trajectory::JsonParser parser(
        reinterpret_cast<const char*>(response.data() + 4), json_len);
    expect(parser.parse(&parsed), "response JSON parses");
  } else {
    expect(false, "response carries JSON");
  }
  return parsed;
}

bool response_ok(const dense_joint_trajectory::JsonValue& response) {
  const dense_joint_trajectory::JsonValue* ok = response.find("ok");
  return ok != nullptr && ok->kind == dense_joint_trajectory::JsonValue::Kind::Bool &&
         ok->bool_value;
}

std::string response_str(const dense_joint_trajectory::JsonValue& response,
                         const char* key) {
  std::string value;
  dense_joint_trajectory::json_str(response.find(key), &value);
  return value;
}

}  // namespace

int main() {
  const std::string testdata = DENSE_JOINT_TRAJECTORY_TESTDATA_DIR;
  const std::vector<unsigned char> blob =
      read_file(testdata + "/dense_joint_trajectory_reference.rdt");
  expect(!blob.empty(), "reference fixture loads");

  char store_template[] = "/tmp/dense-joint-trajectory-ingest-test-XXXXXX";
  const char* store_dir = ::mkdtemp(store_template);
  expect(store_dir != nullptr, "mkdtemp for plan store");
  const std::string ready_file = std::string(store_dir) + "/ready.json";

  dense_joint_trajectory::Plan reference;
  {
    const auto result = dense_joint_trajectory::decode(blob.data(), blob.size(), &reference);
    expect(result.ok, "reference fixture decodes");
  }
  const std::string digest = reference.header.trajectory_digest;
  const std::string preload_control =
      std::string("{\"kind\":\"preload\",\"trajectory_digest\":\"") + digest +
      "\",\"plan_id\":\"" + reference.header.plan_id +
      "\",\"program_id\":\"" + reference.header.program_id +
      "\",\"program_digest\":\"" + reference.header.program_digest +
      "\",\"manifest_revision\":" + std::to_string(reference.header.manifest_revision) +
      ",\"plan_revision\":" + std::to_string(reference.header.plan_revision) + "}";

  dense_joint_trajectory::IngestOptions options;
  options.listen_host = "127.0.0.1";
  options.listen_port = 0;  // ephemeral: the test must not depend on 8797 being free
  options.store_dir = store_dir;
  options.ready_file = ready_file;

  {
    dense_joint_trajectory::IngestServer server(options);
    expect(server.start(), "server starts");
    expect(server.port() != 0, "server reports its bound port");

    // A nonblocking listener must not turn accepted request framing into
    // nonblocking reads: the body can arrive after its length prefix.
    expect(response_ok(exchange(server.port(), "{\"kind\":\"status\"}", {}, true)),
           "fragmented request retains blocking frame semantics");

    // Ready file announces the actual port.
    {
      const std::vector<unsigned char> ready = read_file(ready_file);
      const std::string text(ready.begin(), ready.end());
      expect(text.find(dense_joint_trajectory::kReadySchema) != std::string::npos,
             "ready file carries the schema id");
      expect(text.find(std::to_string(server.port())) != std::string::npos,
             "ready file carries the bound port");
    }

    // validate: accepted, but nothing persisted.
    {
      const auto response = exchange(server.port(), "{\"kind\":\"validate\"}", blob);
      expect(response_ok(response), "validate accepts the fixture");
      dense_joint_trajectory::PlanStore store(store_dir);
      expect(!store.load(digest).has_value(), "validate persists nothing");
    }

    // preload before upload: not found.
    {
      const auto response = exchange(server.port(), preload_control, {});
      expect(!response_ok(response) &&
                 response_str(response, "reason") == "plan_not_found",
             "preload before upload is plan_not_found");
    }

    // upload: accepted and persisted under the digest.
    {
      const auto response = exchange(server.port(), "{\"kind\":\"upload\"}", blob);
      expect(response_ok(response), "upload accepts the fixture");
      expect(response_str(response, "trajectory_digest") == digest,
             "upload echoes the digest");
      dense_joint_trajectory::PlanStore store(store_dir);
      expect(store.load(digest).has_value(), "upload persists the blob");
    }

    // A corrupted upload is refused, naming the segment.
    {
      std::vector<unsigned char> corrupted = blob;
      corrupted[corrupted.size() - 10] ^= 0x01;
      const auto response = exchange(server.port(), "{\"kind\":\"upload\"}", corrupted);
      expect(!response_ok(response) &&
                 response_str(response, "reason") == "block_sha256_mismatch",
             "corrupted upload is block_sha256_mismatch");
      const dense_joint_trajectory::JsonValue* segment = response.find("segment_index");
      expect(segment != nullptr && segment->number_value == 1,
             "corruption names segment 1");
    }

    // Preload with a wrong identity field is fenced off.
    {
      std::string wrong = preload_control;
      const std::string needle = reference.header.plan_id;
      wrong.replace(wrong.find(needle), needle.size(), "someone-elses-plan");
      const auto response = exchange(server.port(), wrong, {});
      expect(!response_ok(response) &&
                 response_str(response, "reason") == "identity_mismatch",
             "preload with wrong plan_id is identity_mismatch");
    }

    // Correct preload stages the plan.
    {
      const auto response = exchange(server.port(), preload_control, {});
      expect(response_ok(response), "preload stages the uploaded plan");
      const auto staged = server.staged_summary();
      expect(staged.present && staged.trajectory_digest == digest,
             "staged summary carries the digest");
    }

    // The executor's refusal (playback in progress) is the client's answer,
    // and the ingest-side staged copy stays what it was.
    {
      server.on_preload = [](const dense_joint_trajectory::Plan&) {
        return dense_joint_trajectory::Rejection::refuse("playback_in_progress", -1,
                                                          "test executor is playing");
      };
      const auto response = exchange(server.port(), preload_control, {});
      expect(!response_ok(response) &&
                 response_str(response, "reason") == "playback_in_progress",
             "preload refused by the executor reports playback_in_progress");
      const auto staged = server.staged_summary();
      expect(staged.present && staged.trajectory_digest == digest,
             "refused preload leaves the staged summary as it was");
      server.on_preload = nullptr;
    }

    // status reflects the staged plan and the absent executor.
    {
      const auto response = exchange(server.port(), "{\"kind\":\"status\"}", {});
      expect(response_ok(response), "status responds");
      expect(response_str(response, "executor") == "absent",
             "status says the executor is absent in phase 2");
      const dense_joint_trajectory::JsonValue* staged = response.find("staged");
      expect(staged != nullptr &&
                 staged->kind == dense_joint_trajectory::JsonValue::Kind::Object,
             "status names the staged plan");
    }
    server.stop();
  }

  // A fresh server over the same store preloads from disk: the store, not
  // the process, is the durable thing.
  {
    dense_joint_trajectory::IngestServer server(options);
    expect(server.start(), "restarted server starts");
    const auto response = exchange(server.port(), preload_control, {});
    expect(response_ok(response), "preload works after restart from the store");
    server.stop();
  }

  if (failures == 0) {
    std::printf("dense_joint_trajectory_ingest_test: all assertions passed\n");
    return 0;
  }
  std::fprintf(stderr, "dense_joint_trajectory_ingest_test: %d failure(s)\n", failures);
  return 1;
}
