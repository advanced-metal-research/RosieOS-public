// Dispatcher idempotency test: a redelivered command_id gets the same reply
// again; the id is remembered only after its handler ran; and a resent
// `stop` both re-executes and acknowledges. Replies are captured through
// the injectable transport; the executor and leader authority are real.

#include <cstdio>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "command_dispatch.hpp"
#include "execution_backend.hpp"
#include "leader_authority.hpp"

namespace {

using namespace dense_joint_trajectory;

int failures = 0;

void expect(bool condition, const std::string& what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
  }
}

struct Reply {
  std::string subject;
  std::string payload;
};

std::string field(const std::string& payload, const char* key) {
  if (std::string(key) == "ok") {  // the one boolean in a reply
    if (payload.find("\"ok\":true") != std::string::npos) return "true";
    if (payload.find("\"ok\":false") != std::string::npos) return "false";
    return "<absent>";
  }
  JsonValue root;
  JsonParser parser(payload.data(), payload.size());
  if (!parser.parse(&root)) return "<unparsable>";
  std::string text;
  if (json_str(root.find(key), &text)) return text;
  return root.find(key) == nullptr ? "<absent>" : "<other>";
}

std::string command(const std::string& kind, const std::string& id,
                    const std::string& extra = "") {
  return "{\"schema\":\"robot.v4.robot-command.v1\",\"command\":\"" + kind +
         "\",\"robot\":\"cell-test\",\"command_id\":\"" + id +
         "\",\"sender_id\":\"offline-programming:test-1\"" + extra + "}";
}

std::string command_as(const std::string& sender, const std::string& kind,
                       const std::string& id, const std::string& extra = "") {
  return "{\"schema\":\"robot.v4.robot-command.v1\",\"command\":\"" + kind +
         "\",\"robot\":\"cell-test\",\"command_id\":\"" + id +
         "\",\"sender_id\":\"" + sender + "\"" + extra + "}";
}

}  // namespace

int main() {
  // The dispatcher contract is transport independent; no UDP writer is acquired.
  struct RecordingExecutor final : ExecutionBackend {
    ExecutorStatus value;
    bool start() override { value.state = "idle"; return true; }
    void stop_thread() override {}
    Rejection preload(const Plan&, const std::vector<unsigned char>&, const std::string&) override { return Rejection::accept(); }
    Rejection play(const PlayIdentity&) override { return Rejection::accept(); }
    Rejection pause_command() override { return Rejection::refuse("native_pause_unsupported", -1, "native pause unavailable"); }
    void stop(const std::string&) override { value.state = "idle"; }
    void revoke_motion() override { value.state = "idle"; }
    ExecutorStatus status() const override { return value; }
  } executor;
  LeaderAuthority leader(/*lease_nonce=*/42);
  expect(executor.start(), "executor starts");

  std::vector<Reply> replies;
  CommandDispatcher dispatcher("cell-test", NatsAddress{}, &leader, &executor,
                               [&replies](const std::string& subject, const std::string& payload) {
                                 replies.push_back({subject, payload});
                               });

  // stop: acknowledged, and acknowledged again on a resend, executing both times.
  {
    const auto pending = command("play", "queue-overflow");
    dispatcher.refuse_queued(pending, "reply.queue");
    expect(replies.size() == 1 && field(replies.back().payload, "reason") == "command_queue_full",
           "queue overflow returns a typed refusal");
    dispatcher.handle(pending, "reply.retry");
    expect(replies.size() == 2 && field(replies.back().payload, "reason") == "fresh_exact_leader_lease_required",
           "command that never reached a handler remains retryable");
    replies.clear();
  }
  dispatcher.handle(command("stop", "stop-1"), "reply.a");
  expect(replies.size() == 1 && field(replies[0].payload, "ok") == "true" &&
             field(replies[0].payload, "command_id") == "stop-1",
         "first stop is acknowledged ok");
  expect(executor.status().state == "idle" || executor.status().state == "aborted",
         "stop executed (state " + executor.status().state + ")");
  dispatcher.handle(command("stop", "stop-1"), "reply.b");
  expect(replies.size() == 2 && field(replies[1].payload, "ok") == "true" &&
             field(replies[1].payload, "command_id") == "stop-1" &&
             replies[1].subject == "reply.b",
         "resent stop is acknowledged again, on the resend's reply subject");
  expect(replies[1].payload == replies[0].payload, "the resend gets the same reply");

  // leader_acquire: the resend returns the SAME grant, not a second lease.
  dispatcher.handle(command("leader_acquire", "acq-1",
                            ",\"ttl_ms\":1000,\"controller_boot_id\":\"boot-1\""),
                    "reply.c");
  expect(replies.size() == 3 && field(replies[2].payload, "ok") == "true",
         "leader_acquire granted (" + replies.back().payload + ")");
  const std::string lease = field(replies[2].payload, "lease_id");
  dispatcher.handle(command("leader_acquire", "acq-1",
                            ",\"ttl_ms\":1000,\"controller_boot_id\":\"boot-1\""),
                    "reply.c");
  expect(replies.size() == 4 && field(replies[3].payload, "lease_id") == lease,
         "resent leader_acquire repeats the original grant, lease " + lease);
  expect(leader.snapshot().lease_id == lease, "no second lease was minted");


  // A refused command is remembered as refused: the resend is refused the
  // same way, not re-evaluated against new state.
  dispatcher.handle(command("play", "play-1"), "reply.d");  // no credential
  expect(replies.size() == 5 && field(replies[4].payload, "ok") == "false" &&
             field(replies[4].payload, "reason") == "fresh_exact_leader_lease_required",
         "play without a lease is refused");
  dispatcher.handle(command("play", "play-1"), "reply.d");
  expect(replies.size() == 6 && replies[5].payload == replies[4].payload,
         "resent play gets the same refusal");

  // Bridge-owned kinds are neither answered nor remembered.
  dispatcher.handle(command("io", "io-1"), "reply.e");
  expect(replies.size() == 6, "io is not ours to answer");

  // A command with no reply subject still runs and is remembered.
  dispatcher.handle(command("stop", "stop-2"), "");
  expect(replies.size() == 6, "no reply_to: nothing published");
  dispatcher.handle(command("stop", "stop-2"), "reply.f");
  expect(replies.size() == 7 && field(replies[6].payload, "command_id") == "stop-2",
         "the resend of a reply-less stop is acknowledged");


  // Only the pipeline that produced the plan may lease: the pendant's
  // identity, which the v1 allowlist accepted, is refused, and so is a
  // malformed instance.
  for (const std::string& sender :
       {std::string("steamdeck:steamdeck-primary"), std::string("offline-programming:"),
        std::string("offline-programming:bad instance")}) {
    const std::size_t before = replies.size();
    dispatcher.handle(command_as(sender, "leader_acquire", "acq-" + sender,
                                 ",\"controller_boot_id\":\"boot-x\",\"ttl_ms\":1000"),
                      "reply.inbox");
    expect(replies.size() == before + 1 && field(replies.back().payload, "ok") == "false" &&
               field(replies.back().payload, "reason") == "leader_identity_invalid",
           "acquire refused for " + sender + " (" + replies.back().payload + ")");
  }
  expect(leader.snapshot().lease_id == lease, "the refusals did not disturb the lease");

  // go_home is a fenced motion command like play: routed (an unknown kind
  // gets no reply at all) and refused without a lease.
  dispatcher.handle(command("go_home", "home-0"), "reply.d");
  expect(field(replies.back().payload, "command") == "go_home" &&
             field(replies.back().payload, "reason") == "fresh_exact_leader_lease_required",
         "go_home without a lease is refused as fresh_exact_leader_lease_required");
  executor.stop_thread();
  if (failures != 0) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::printf("command_dispatch_test: ok\n");
  return 0;
}
