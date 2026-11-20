#include "rosie/rt_control_client.hpp"
#include "rosie/rt_jog_producer.hpp"
#include "rosie/rt_jog_remote_producer.hpp"
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <future>
#include <iostream>

using namespace rosie::rt_control;
using namespace std::chrono_literals;

struct Incoming { std::string path, body, headers; int connection; };

// A bounded in-process Unix HTTP peer exercises actual framing and reuse rather
// than mocking the parser. Worker failures are rethrown on the owning test thread.
class FakeServer {
    int listener_ = -1;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::exception_ptr error_;
    std::string directory_;
public:
    std::string path;
    std::atomic<int> connections{0}, requests{0};
    // Unverified fragmentation stimulus defaults to three bytes per write.
    explicit FakeServer(std::function<std::string(const Incoming&)> handler, std::size_t fragment_bytes = 3) {
        auto tmp = std::getenv("TMPDIR");
        assert(tmp);
        directory_ = std::string(tmp) + "/c-XXXXXX";
        assert(::mkdtemp(directory_.data()));
        path = directory_ + "/c.sock";
        auto address = client_detail::unix_address(path);
        listener_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        assert(listener_ >= 0);
        assert(::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
        assert(::listen(listener_, 8) == 0);
        thread_ = std::thread([this, handler, fragment_bytes] {
            std::vector<int> peers;
            try {
                while (!stop_) {
                    std::vector<pollfd> polls{{listener_, POLLIN, 0}};
                    for (int fd : peers) polls.push_back({fd, POLLIN, 0});
                    // Unverified test polling budget: 10 ms; assertions use observed requests.
                    if (::poll(polls.data(), polls.size(), 10) <= 0) continue;
                    if (polls[0].revents & POLLIN) {
                        int fd = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC);
                        if (fd < 0) throw std::runtime_error("fake accept");
                        // Unverified fake peer I/O ceiling: 2,000 ms.
                        timeval timeout{2, 0};
                        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
                        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                        peers.push_back(fd); ++connections;
                    }
                    for (std::size_t i = 1; i < polls.size(); ++i) {
                        if (!polls[i].revents || peers[i - 1] < 0) continue;
                        int fd = peers[i - 1];
                        std::string raw;
                        char ch;
                        while (raw.find("\r\n\r\n") == std::string::npos) {
                            auto n = ::recv(fd, &ch, 1, 0);
                            if (n <= 0) break;
                            raw += ch;
                        }
                        if (raw.empty()) { ::close(fd); peers[i - 1] = -1; continue; }
                        auto split = raw.find("\r\n\r\n");
                        if (split == std::string::npos) throw std::runtime_error("fake request header");
                        auto length = raw.find("Content-Length: ");
                        if (length == std::string::npos) throw std::runtime_error("missing request length");
                        auto start = length + std::string("Content-Length: ").size();
                        auto size = client_detail::number(raw.substr(start, raw.find("\r\n", start) - start), 10);
                        Incoming request; request.headers = raw; request.connection = fd;
                        auto space = raw.find(' ');
                        request.path = raw.substr(space + 1, raw.find(' ', space + 1) - space - 1);
                        request.body.resize(size);
                        std::size_t received = 0;
                        while (received < size) {
                            auto n = ::recv(fd, request.body.data() + received, size - received, 0);
                            if (n <= 0) throw std::runtime_error("fake request body");
                            received += static_cast<std::size_t>(n);
                        }
                        ++requests;
                        auto reply = handler(request);
                        if (reply.empty()) { ::close(fd); peers[i - 1] = -1; continue; }
                        for (std::size_t sent = 0; sent < reply.size();) {
                            auto n = ::send(fd, reply.data() + sent, std::min(fragment_bytes, reply.size() - sent), MSG_NOSIGNAL);
                            if (n <= 0) break;
                            sent += static_cast<std::size_t>(n);
                        }
                        if (reply.find("Connection: close\r\n") != std::string::npos) { ::close(fd); peers[i - 1] = -1; }
                    }
                }
            } catch (...) { error_ = std::current_exception(); }
            for (int fd : peers) if (fd >= 0) ::close(fd);
        });
    }
    void finish() { stop_ = true; if (thread_.joinable()) thread_.join(); if (error_) std::rethrow_exception(error_); }
    ~FakeServer() { stop_ = true; if (thread_.joinable()) thread_.join(); ::close(listener_); ::unlink(path.c_str()); ::rmdir(directory_.c_str()); }
};

std::string http(const std::string& body, int status = 200) {
    return "HTTP/1.1 " + std::to_string(status) + " Result\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
}
std::string receipt(const std::string& op, Json data = {}, std::uint64_t handle = 0) {
    Response response; response.Schema = ResponseSchema; response.Operation = op; response.Data = std::move(data); response.Handle = handle;
    return http(encode(response));
}
StreamFactory dial(const std::string& path) { return [path](Deadline d) { return std::make_unique<client_detail::UnixStream>(path, d); }; }

template<class Predicate> void observed(Predicate predicate) {
    // Unverified positive-observation ceiling: 3,000 ms, independent of measured values.
    auto deadline = std::chrono::steady_clock::now() + 3000ms;
    while (!predicate()) { assert(std::chrono::steady_clock::now() < deadline); std::this_thread::yield(); }
}

void framing() {
    FakeServer server([](const Incoming& r) {
        if (r.path == "/chunked") return std::string("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n3\r\nabc\r\n2\r\nde\r\n0\r\nX-Test: yes\r\n\r\n");
        return http("hello");
    });
    client_detail::Lane lane(dial(server.path));
    assert(lane.exchange("GET", "/length", {}, {}, default_deadline()).body == "hello");
    assert(lane.exchange("GET", "/chunked", {}, {}, default_deadline()).body == "abcde");
    assert(lane.exchange("GET", "/length", {}, {}, default_deadline()).body == "hello");
    assert(server.connections == 1 && server.requests == 3);
    server.finish();
    FakeServer closing([](const Incoming&) { return std::string("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Length: 1\r\n\r\nx"); });
    client_detail::Lane reconnect(dial(closing.path));
    assert(reconnect.exchange("GET", "/", {}, {}, default_deadline()).body == "x");
    assert(reconnect.exchange("GET", "/", {}, {}, default_deadline()).body == "x");
    assert(closing.connections == 2); closing.finish();
    FakeServer truncated([](const Incoming&) { return std::string("HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Length: 10\r\n\r\nabc"); });
    client_detail::Lane incomplete(dial(truncated.path));
    bool lost = false;
    try { incomplete.exchange("GET", "/", {}, {}, default_deadline()); } catch (const TransportError&) { lost = true; }
    assert(lost); truncated.finish();
    // Measured V2 wire limit: 22085945 bytes is one above the maximum telemetry response.
    for (const std::string wire : {
            "HTTP/1.0 200 OK\r\nContent-Length: 0\r\n\r\n",
            "HTTP/1.1 2x0 OK\r\nContent-Length: 0\r\n\r\n",
            "HTTP/1.1 200 OK\r\nContent-Length: 0\r\nContent-Length: 0\r\n\r\n",
            "HTTP/1.1 200 OK\r\nContent-Length: 0\r\nTransfer-Encoding: chunked\r\n\r\n",
            "HTTP/1.1 200 OK\r\nContent-Length: -1\r\n\r\n",
            "HTTP/1.1 200 OK\r\nContent-Length: 22085945\r\n\r\n",
            "HTTP/1.1 200 OK\r\nBad Header: x\r\nContent-Length: 0\r\n\r\n",
            "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n1\r\nxzz",
            "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nz\r\n",
            "HTTP/1.1 200 OK\r\n\r\n"}) {
        FakeServer bad([wire](const Incoming&) { return wire; });
        client_detail::Lane connection(dial(bad.path));
        bool rejected = false;
        try { connection.exchange("GET", "/", {}, {}, default_deadline()); }
        catch (const ProtocolError&) { rejected = true; }
        assert(rejected); bad.finish();
    }
}

void retries_and_grants() {
    Grant grant; grant.Session = std::string(64, 'a'); grant.Generation = 1; grant.LeaseMS = 500; grant.DeadlineHostNS = 1;
    std::string retained;
    unsigned arms = 0;
    FakeServer server([&](const Incoming& incoming) {
        if (incoming.path == "/v1/describe") {
            Description d; d.Description.MaxGrantLeaseNS=3000000000; d.Description.MaxJogInputAgeNS=750000000;
            return receipt("describe", to_json(d));
        }
        Request request; assert(decode(incoming.body, request));
        assert(request.RequestID && request.RequestID->size() == 32);
        if (request.Operation == "acquire") { assert(request.RequestedLeaseMS == 500); return receipt(request.Operation, to_json(grant)); }
        assert(request.Fence.Session == grant.Session && request.Fence.Generation == grant.Generation);
        if (request.Operation == "arm") {
            if (++arms == 1) { retained = *request.RequestID; return std::string{}; }
            assert(*request.RequestID == retained);
            return receipt(request.Operation);
        }
        if (request.Operation == "renew") { grant.Stopping = true; ++grant.DeadlineHostNS; return receipt(request.Operation, to_json(grant)); }
        if (request.Operation == "release") return receipt(request.Operation, to_json(grant));
        Response response; response.Schema = ResponseSchema; response.Error = ReasonControlSessionStale;
        response.NativeResult = CommandResult{}; response.NativeResult->Reason = 7;
        return http(encode(response), 409);
    });
    RtControlClient client(server.path);
    client.acquire("test", {});
    assert(client.fence().Generation == 1);
    Request request; request.Operation = "arm"; client.command(request);
    assert(request.RequestID && request.Fence.Generation == 1);
    client.start_renewal(50ms);
    std::optional<Renewal> update;
    observed([&] { update = client.take_renewal(); return bool(update); });
    client.stop_renewal();
    assert(!update->error && update->grant.Stopping && client.grant().Stopping);
    bool rejected = false;
    try { client.home(1); } catch (const Rejected& e) { rejected = e.reason == ReasonControlSessionStale && e.native_result && e.native_result->Reason == 7; }
    assert(rejected);
    client.release(); assert(client.fence().Session.empty());
    server.finish(); assert(arms == 2);
}

void uncertain_upload() {
    FakeServer server([](const Incoming& r) {
        assert(r.path == "/v1/program" && r.body == "immutable rdt");
        assert(r.headers.find("X-Request-ID: ") != std::string::npos);
        return std::string{};
    });
    RtControlClient client(server.path); client.set_fence({"session", 1});
    bool lost = false;
    try { client.prepare_program("immutable rdt"); } catch (const TransportError&) { lost = true; }
    assert(lost && server.requests == 1); server.finish();
}

// TestReviewDeckExpiredRecovery preserves Stop then Release with the expired
// fence; an empty-session Stop cannot strand the Deck controller in inhibition.
void expired_stop_release() {
    unsigned stops = 0, releases = 0, acquires = 0, enables = 0, arms = 0;
    FakeServer server([&](const Incoming& incoming) {
        if (incoming.path == "/v1/describe") {
            Description d; d.Description.MaxGrantLeaseNS = 500000000;
            return receipt("describe", to_json(d));
        }
        Request r; assert(decode(incoming.body, r));
        Grant g;
        if (r.Operation == "stop" || r.Operation == "release") {
            assert(r.Fence.Session == "expired" && r.Fence.Generation == 7);
            if (r.Operation == "stop") ++stops; else ++releases;
            g.Generation = 8;
        } else if (r.Operation == "acquire") {
            ++acquires; g.Session = "fresh"; g.Generation = 9; g.LeaseMS = 500;
        } else {
            assert(r.Fence.Session == "fresh" && r.Fence.Generation == 9);
            if (r.Operation == "enable") ++enables;
            else { assert(r.Operation == "arm"); ++arms; }
        }
        return receipt(r.Operation, to_json(g));
    });
    RtControlClient client(server.path); client.set_fence({"expired", 7});
    assert(client.stop().Session.empty() && client.grant().Session.empty());
    assert(client.grant().Generation == 8);
    client.release();
    assert(client.grant().Session.empty());
    assert(client.acquire("steamdeck-primary", {}).Session == "fresh");
    client.enable(1); client.arm();
    server.finish();
    assert(stops == 1 && releases == 1 && acquires == 1 && enables == 1 && arms == 1);
}

void restore_anchor_recovery() {
    FakeServer server([](const Incoming& incoming) {
        Request request; assert(decode(incoming.body, request));
        assert(incoming.path == "/v1/control" && request.Operation == "restore_anchor" && request.AxisMask == 3);
        assert(request.Fence.Session == "owner" && request.Fence.Generation == 7);
        return http(R"({"schema":"rosie.rt-control.response.v1","operation":"restore_anchor","sequence":9007199254740995,"data":{"home_valid_mask":3,"faults":[],"reset_sequence":9007199254740993,"reason":"","outcomes":[{"bit":8,"name":"bus_transport","axis_mask":3,"recovery":"reset_after_condition_clears","outcome":"cleared","rehome_axis_mask":0}]}})");
    });
    RtControlClient client(server.path); client.set_fence({"owner", 7});
    auto recovery = client.restore_anchor(3);
    assert(recovery.HomeValidMask == 3 && recovery.ResetSequence == 9007199254740993ULL);
    assert(recovery.Faults.empty() && recovery.Reason.empty() && recovery.Outcomes.size() == 1);
    const auto& outcome = recovery.Outcomes[0];
    assert(outcome.Bit == 8 && outcome.Name == "bus_transport" && outcome.AxisMask == 3);
    assert(outcome.Recovery == "reset_after_condition_clears" && outcome.Outcome == "cleared" && outcome.RehomeAxisMask == 0);
    server.finish();

    FakeServer sequence_only([](const Incoming&) {
        return http(R"({"schema":"rosie.rt-control.response.v1","operation":"restore_anchor","sequence":19})");
    });
    RtControlClient incomplete(sequence_only.path);
    bool rejected = false;
    try { incomplete.restore_anchor(1); } catch (const ProtocolError&) { rejected = true; }
    assert(rejected); sequence_only.finish();
}

// Chunk boundaries deliberately split SSE lines, while FakeServer fragments
// socket writes independently; neither boundary may become an event boundary.
std::string sse(const std::string& body, bool finish = true) {
    std::ostringstream wire;
    wire << "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nTransfer-Encoding: chunked\r\n\r\n";
    // Unverified framing stimulus: thirteen bytes in the first HTTP chunk.
    const auto split = std::min<std::size_t>(13, body.size());
    for (const auto& part : {body.substr(0, split), body.substr(split)}) {
        if (!part.empty()) wire << std::hex << part.size() << "\r\n" << part << "\r\n";
    }
    if (finish) wire << "0\r\n\r\n";
    return wire.str();
}

void events() {
    const std::string record = R"({"sequence":9007199254740993,"time_ns":42,"type":"execution_completed","daemon_incarnation":"daemon","adapter_incarnation":"adapter","execution":{"handle":7,"generation":3,"native_sequence":9007199254740995,"fault_bits":0,"recovery":[]}})";
    const auto batch = "{\"events\":[" + record + "],\"next_sequence\":9007199254740993,\"latest_sequence\":9007199254740993,\"adapter_incarnation\":\"adapter\"}";
    FakeServer server([&](const Incoming& r) {
        if (r.path == "/v1/events?after=9007199254740992")
            return http("{\"schema\":\"rosie.rt-control.response.v1\",\"operation\":\"subscribe_events\",\"data\":" + batch + "}");
        if (r.path == "/v1/events/stream?after=9007199254740992") {
            assert(r.headers.find("Accept: text/event-stream") != std::string::npos);
            // Multiline data and CRLF are legal SSE; JSON remains generated.
            return sse(": keepalive\r\n\r\nid: 9007199254740993\r\nevent: execution_completed\r\ndata: \r\ndata: " + record + "\r\n\r\n");
        }
        if (r.path == "/v1/status") return receipt("status", to_json(ProcessStatus{}));
        assert(r.path == "/v1/events?after=18446744073709551615" || r.path == "/v1/events/stream?after=18446744073709551615");
        Response response; response.Schema = ResponseSchema; response.Operation = "subscribe_events"; response.Error = ReasonEventCursorAhead;
        return http(encode(response), 409);
    });
    RtControlClient client(server.path);
    auto result = client.events(9007199254740992ULL);
    assert(result.Events.size() == 1 && result.NextSequence == 9007199254740993ULL && result.LatestSequence == result.NextSequence);
    const auto& event = result.Events[0];
    assert(event.Sequence == 9007199254740993ULL && event.TimeNS == 42 && event.Execution && event.Execution->Generation == 3 && event.Execution->NativeSequence == 9007199254740995ULL);
    unsigned delivered = 0;
    client.events_stream(9007199254740992ULL, [&](const Event& e) {
        assert(e.Sequence == 9007199254740993ULL && e.Execution && e.Execution->Handle == 7);
        client.status(); // An observer's control read cannot deadlock its stream.
        ++delivered;
    }, [&] { return delivered == 1; }, default_deadline());
    assert(delivered == 1);
    for (bool stream : {false, true}) {
        bool refused = false;
        try {
            if (stream) client.events_stream(UINT64_MAX, [](const Event&) { assert(false); }, [] { return false; }, default_deadline());
            else client.events(UINT64_MAX);
        } catch (const Rejected& e) { refused = e.reason == ReasonEventCursorAhead; }
        assert(refused);
    }
    server.finish();

    const std::string dropped = R"({"sequence":3,"time_ns":42,"type":"events_dropped","daemon_incarnation":"daemon","adapter_incarnation":"adapter","events_dropped":{"first_lost_sequence":1,"last_lost_sequence":3}})";
    FakeServer loss([&](const Incoming& r) {
        if (r.path == "/v1/events?after=0") return http("{\"schema\":\"rosie.rt-control.response.v1\",\"operation\":\"subscribe_events\",\"data\":{\"events\":[" + dropped + "],\"next_sequence\":3,\"latest_sequence\":3,\"adapter_incarnation\":\"adapter\"}}");
        return sse("id: 3\nevent: events_dropped\ndata: " + dropped + "\n\n");
    });
    RtControlClient lost(loss.path);
    result = lost.events(0);
    assert(result.Events.size() == 1 && result.Events[0].Dropped && result.Events[0].Dropped->FirstLostSequence == 1 && result.Events[0].Dropped->LastLostSequence == 3);
    bool cursor_lost = false;
    try { lost.events_stream(0, [](const Event&) { assert(false); }, [] { return false; }, default_deadline()); }
    catch (const EventCursorLost& e) { cursor_lost = e.reason == ReasonEventCursorLost && e.event.Dropped && e.event.Dropped->LastLostSequence == 3; }
    assert(cursor_lost); loss.finish();

    for (const auto& body : std::vector<std::string>{
            "id: 9007199254740994\nevent: execution_completed\ndata: " + record + "\n\n",
            "id: 9007199254740993\nevent: wrong_kind\ndata: " + record + "\n\n",
            "id: 9007199254740993\nevent: execution_completed\ndata: {}\n\n",
            "id: 9007199254740993\nevent: execution_completed\ndata: " + record + "\n\n" + "id: 9007199254740993\nevent: execution_completed\ndata: " + record + "\n\n"}) {
        FakeServer bad([&](const Incoming&) { return sse(body); });
        RtControlClient invalid(bad.path);
        bool refused = false;
        try { invalid.events_stream(9007199254740992ULL, [](const Event&) {}, [] { return false; }, default_deadline()); }
        catch (const ProtocolError&) { refused = true; }
        assert(refused); bad.finish();
    }
    for (const auto& bad_batch : {
            R"({"events":[],"next_sequence":1,"latest_sequence":1,"adapter_incarnation":"adapter"})",
            R"({"events":[],"next_sequence":0,"latest_sequence":0})",
            R"({"events":[{"sequence":2,"time_ns":42,"type":"enabled","daemon_incarnation":"daemon","adapter_incarnation":"adapter"}],"next_sequence":2,"latest_sequence":2,"adapter_incarnation":"adapter"})"}) {
        FakeServer bad([&](const Incoming&) { return http(std::string("{\"schema\":\"rosie.rt-control.response.v1\",\"operation\":\"subscribe_events\",\"data\":") + bad_batch + "}"); });
        RtControlClient invalid(bad.path);
        bool refused = false;
        try { invalid.events(0); } catch (const ProtocolError&) { refused = true; }
        assert(refused); bad.finish();
    }
}

void event_stream_cancellation() {
    FakeServer idle([](const Incoming&) { return sse(": connected\n\n", false); });
    RtControlClient client(idle.path);
    std::atomic<bool> stop{false};
    auto stream = std::async(std::launch::async, [&] { client.events_stream(0, [](const Event&) { assert(false); }, [&] { return stop.load(); }); });
    observed([&] { return idle.requests == 1; });
    stop = true;
    // Unverified cancellation ceiling: 1,000 ms; success requires stream return.
    assert(stream.wait_for(1000ms) == std::future_status::ready);
    stream.get();
    client.events_stream(0, [](const Event&) { assert(false); }, [] { return true; });
    assert(idle.requests == 1); idle.finish();
}

void jog_frame() {
    FakeServer server([](const Incoming& r) {
        if (r.path == "/v1/jog/clock") return receipt("jog_clock", to_json(LocalJogClock{"CLOCK_MONOTONIC", std::string(32, 'b'), 1, host_monotonic_ns()}));
        Request request; assert(decode(r.body, request));
        assert(request.Fence.Generation == 7);
        return receipt(request.Operation, {}, 9);
    });
    auto path = server.path + "j";
    auto address = client_detail::unix_address(path);
    int socket = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    assert(socket >= 0 && ::bind(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    RtControlClient client(server.path); client.set_fence({std::string(64, 'a'), 7});
    // Unverified jog test input lifetime: 200,000,000 ns, below generated 250,000,000 ns admission cap.
    RtJogProducer producer(client, path, 1, host_monotonic_ns() + 200000000);
    auto origin = host_monotonic_ns();
    assert(producer.update({0.25}, origin + 200000000));
    std::array<std::uint8_t, rosie::protocol::local_jog_update_bytes> bytes{};
    auto n = ::recv(socket, bytes.data(), bytes.size(), 0);
    rosie::protocol::LocalJogUpdate frame{};
    assert(rosie::protocol::decode_local_jog_update(bytes.data(), static_cast<std::size_t>(n), frame));
    assert(frame.grant_generation == 7 && frame.jog_generation == 9 && frame.source_sequence == 1);
    assert(frame.origin_host_ns >= origin && frame.deadline_host_ns == origin + 200000000 && frame.velocity[0] == 0.25);
    assert(frame.session_id[0] == 0xaa && frame.clock_incarnation[0] == 0xbb);
    bool invalid = false;
    try { producer.update({std::numeric_limits<double>::quiet_NaN()}, host_monotonic_ns() + 200000000); }
    catch (const std::invalid_argument&) { invalid = true; }
    assert(invalid);
    producer.end(); ::close(socket); ::unlink(path.c_str()); server.finish();
}

// RFC 6455's independent accept vector catches SHA-1/base64 drift. A local
// client must fail before issuing BeginJog for the authenticated remote lane.
void remote_jog_contract() {
    assert(remote_jog_detail::accept("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
    JogRejection stale("jog_native_rejected_6", 4);
    assert(stale.native_reason == rosie::protocol::jog_reason_stale_sequence && stale.sequence == 4);
    RtControlClient local("/unused/control.sock");
    local.set_fence({std::string(64, 'a'), 7});
    bool refused = false;
    try { RtJogRemoteProducer producer(local, "localhost", 1, 100000000); }
    catch (const std::invalid_argument&) { refused = true; }
    assert(refused);
}

// This peer decodes the actual masked frames, so a successful update must reach
// the stream even while an independent HTTP observation or WSS read is blocked.
struct RemoteWire {
    std::mutex mutex;
    std::condition_variable changed;
    std::string pending;
    unsigned updates = 0;
    bool reading = false;
    rosie::protocol::LocalJogUpdate last{};
    void reply(const Json& json) {
        auto body = dump(json);
        pending += char(0x81);
        if (body.size() < 126) pending += char(body.size());
        else { pending += char(126); pending += char(body.size() >> 8); pending += char(body.size()); }
        pending += body;
        changed.notify_all();
    }
};

class RemoteStream final : public HttpStream {
    std::shared_ptr<RemoteWire> wire_;
public:
    explicit RemoteStream(std::shared_ptr<RemoteWire> wire) : wire_(std::move(wire)) {}
    std::size_t read(char* data, std::size_t size, Deadline d) override {
        std::unique_lock<std::mutex> lock(wire_->mutex);
        wire_->reading = true;
        wire_->changed.notify_all();
        if (!wire_->changed.wait_until(lock, d, [&] { return !wire_->pending.empty(); })) throw DeadlineExceeded("fake remote read");
        wire_->reading = false;
        auto n = std::min(size, wire_->pending.size());
        std::copy_n(wire_->pending.data(), n, data);
        wire_->pending.erase(0, n);
        return n;
    }
    std::size_t write(const char* data, std::size_t size, Deadline) override {
        std::lock_guard<std::mutex> lock(wire_->mutex);
        std::string packet(data, size);
        if (packet.compare(0, 4, "GET ") == 0) {
            auto start = packet.find("Sec-WebSocket-Key: ") + std::string("Sec-WebSocket-Key: ").size();
            auto key = packet.substr(start, packet.find("\r\n", start) - start);
            wire_->pending = "HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\nUpgrade: websocket\r\nSec-WebSocket-Accept: " + remote_jog_detail::accept(key) + "\r\n\r\n";
        } else {
            assert(size >= 6 && (static_cast<unsigned char>(data[1]) & 0x80));
            std::size_t length = static_cast<unsigned char>(data[1]) & 127, offset = 2;
            if (length == 126) { length = (static_cast<unsigned char>(data[2]) << 8) | static_cast<unsigned char>(data[3]); offset = 4; }
            assert(size == offset + 4 + length);
            std::string payload(length, '\0');
            for (std::size_t i = 0; i < length; ++i) payload[i] = data[offset + 4 + i] ^ data[offset + i % 4];
            if (static_cast<unsigned char>(data[0]) == 0x81) {
                auto json = Parser(payload).parse();
                std::uint64_t source{}, seq{};
                std::string type;
                field(json, "type", type, true); field(json, "seq", seq, true);
                if (type == "begin") {
                    assert(seq == 3);
                    json.object = {{"type", to_json(std::string("begun"))}, {"seq", to_json(seq)},
                                   {"jog_generation", to_json(std::uint64_t(9))}};
                } else {
                    assert(type == "calibrate");
                    field(json, "source_ns", source, true);
                    json.object = {{"type", to_json(std::string("calibrated"))}, {"source_ns", to_json(source)},
                        {"seq", to_json(seq)}, {"host_rx_ns", to_json(std::uint64_t(1))}, {"host_tx_ns", to_json(std::uint64_t(2))}};
                }
                wire_->reply(json);
            } else {
                assert(static_cast<unsigned char>(data[0]) == 0x82);
                assert(rosie::protocol::decode_local_jog_update(reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size(), wire_->last));
                ++wire_->updates;
            }
        }
        wire_->changed.notify_all();
        return size;
    }
};

void remote_jog_independent_reads() {
    std::promise<void> entered, release;
    auto released = release.get_future();
    std::atomic<bool> block{true};
    std::atomic<std::uint32_t> reason{rosie::protocol::jog_reason_none};
    FakeServer server([&](const Incoming& r) {
        if (r.path == "/v1/jog/clock") return receipt("jog_clock", to_json(LocalJogClock{"CLOCK_MONOTONIC", std::string(32, 'b'), 1, host_monotonic_ns()}));
        if (r.path == "/v1/status") {
            if (block.exchange(false)) {
                entered.set_value();
                // Unverified synchronization failure ceiling: 1000 ms.
                assert(released.wait_for(1000ms) == std::future_status::ready);
            }
            ProcessStatus status;
            status.Jog = JogObservation{};
            auto& o = *status.Jog;
            o.JogGeneration = 9; o.AppGrantGeneration = 7; o.ObservedSourceSequence = 1;
            o.StateReason = reason; o.HasInput = o.Open = reason == rosie::protocol::jog_reason_none;
            // Unverified past-deadline stimulus: now 1,000,000,000 ns, deadline 1 ns.
            o.NowHostNs = 1000000000; o.InputDeadlineHostNs = 1;
            return receipt("status", to_json(status));
        }
        Request request; assert(decode(r.body, request));
        return receipt(request.Operation, {}, 9);
    });
    auto wire = std::make_shared<RemoteWire>();
    bool remote = true;
    RtControlClient client(StreamFactory([&](Deadline d) -> std::unique_ptr<HttpStream> {
        if (remote) { remote = false; return std::make_unique<RemoteStream>(wire); }
        return std::make_unique<client_detail::UnixStream>(server.path, d);
    }));
    client.set_fence({std::string(64, 'a'), 7});
    // Unverified test input lifetime: 100,000,000 ns.
    RtJogRemoteProducer producer(client, "localhost", 1, 100000000);
    auto observation = std::async(std::launch::async, [&] { return producer.observe(); });
    // Unverified synchronization failure ceiling: 1000 ms.
    assert(entered.get_future().wait_for(1000ms) == std::future_status::ready);
    assert(producer.update({0.25}, host_monotonic_ns()));
    { std::lock_guard<std::mutex> lock(wire->mutex); assert(wire->updates == 1 && wire->last.source_sequence == 1 && wire->last.velocity[0] == 0.25); }
    release.set_value();
    assert(observation.get().HasInput == 1);

    auto refusal = std::async(std::launch::async, [&] {
        try { producer.rejection(); assert(false); }
        catch (const JogRejection& e) { assert(e.native_reason == rosie::protocol::jog_reason_stale_sequence && e.sequence == 2); }
    });
    { std::unique_lock<std::mutex> lock(wire->mutex); assert(wire->changed.wait_for(lock, 1000ms, [&] { return wire->reading; })); }
    assert(producer.update({0.5}, host_monotonic_ns()));
    {
        std::lock_guard<std::mutex> lock(wire->mutex);
        assert(wire->updates == 2 && wire->last.source_sequence == 2 && wire->last.velocity[0] == 0.5);
        Json json = Json::object_value();
        json.object = {{"type", to_json(std::string("rejected"))}, {"seq", to_json(std::uint64_t(2))}, {"reason", to_json(std::string("jog_native_rejected_6"))}};
        wire->reply(json);
    }
    refusal.get();
    for (auto actual : {rosie::protocol::jog_reason_closed, rosie::protocol::jog_reason_ramping, rosie::protocol::jog_reason_authority_lost}) {
        reason = actual;
        assert(producer.observe().StateReason == actual);
    }
    reason = rosie::protocol::jog_reason_expired;
    bool expired = false;
    try { producer.observe(); }
    catch (const JogRejection& e) { expired = e.reason == ReasonJogInputDeadlineExpired && e.native_reason == 9 && e.native_jog_result && e.native_jog_result->StateReason == 9; }
    assert(expired);
    producer.end();
    reason = rosie::protocol::jog_reason_closed;
    assert(producer.observe().StateReason == rosie::protocol::jog_reason_closed);
    server.finish();
}

// Go integration invokes this driver against the real application boundary.
// Its stdin gates Stop after the proxy has observed actual bulk wire progress.
int integration(const std::string& mode, const std::string& socket, const std::string& hash) {
    RtControlClient client(socket);
    client.acquire("motion-server", {"bench", 7, hash});
    client.start_renewal(50ms);
    client.enable(511); client.arm();
    observed([&] {
        auto s = client.status();
        bool ready = s.Core.Armed == 1 && s.Core.AxisEnableMask == 511;
        for (auto axis : s.Core.Axes) ready = ready && (axis.Statusword & 0x6f) == 0x27;
        return ready;
    });
    if (mode == "--bulk") {
        std::vector<Point> points(250000);
        for (std::size_t i = 0; i < points.size(); ++i) { points[i].Time = static_cast<std::int64_t>(i) * 1000000; points[i].Position.resize(9); }
        auto uploaded = std::async(std::launch::async, [&] {
            try { client.prepare_trajectory(511, points, std::chrono::steady_clock::now() + 30000ms); return std::string("accepted"); }
            catch (const Rejected& e) { return e.reason; }
        });
        assert(std::cin.get() == 's');
        auto begin = std::chrono::steady_clock::now();
        auto stop = client.stop(begin + 1000ms);
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
        assert(elapsed < 1000000000 && stop.Generation == 2);
        assert(uploaded.wait_for(0ms) != std::future_status::ready);
        std::cout << "stopped " << elapsed << std::endl;
        auto result = uploaded.get();
        assert(result == ReasonControlSessionStale);
        std::cout << "upload " << result << std::endl;
    } else {
        auto prepared = client.prepare_trajectory(511, {{0, std::vector<double>(9), {}}, {2000000000, std::vector<double>(9), {}}});
        Request request; request.Operation = "start_trajectory"; request.Handle = prepared.Handle;
        auto reply = client.command(request);
        auto retried = client.command(request);
        assert(reply.Sequence == retried.Sequence && request.RequestID->size() == 32);
        observed([&] { auto s = client.status(); assert(s.Execution.Generation <= 1); return s.Execution.Generation == 1 && s.Motion.CommandSequence == reply.Sequence; });
        std::cout << "generation 1 sequence " << reply.Sequence << std::endl;
        client.stop();
    }
    client.release(); return 0;
}

// Busy responses retain their typed reason on polling and streaming entry points;
// a protocol exception cannot accidentally satisfy either refusal assertion.
void telemetry_busy_rejected() {
    FakeServer server([](const Incoming& request) {
        assert(request.path.find("after=17") != std::string::npos);
        Response response; response.Schema = ResponseSchema;
        response.Operation = "telemetry"; response.Error = ReasonTelemetryBusy;
        return http(encode(response), 409);
    });
    RtControlClient client(server.path);
    unsigned refusals = 0;
    try { client.telemetry(17); assert(false); }
    catch (const Rejected& failure) { assert(failure.reason == "telemetry_busy"); ++refusals; }
    try { client.telemetry_stream(17, [](const TelemetryPublication&) { assert(false); }, [] { return false; }); assert(false); }
    catch (const Rejected& failure) { assert(failure.reason == "telemetry_busy"); ++refusals; }
    assert(refusals == 2);
    server.finish();
}

// Both HTTP entry points refuse V1 exactly once, before delivering a record;
// the typed failure must retain the stored and expected layout identities.
void telemetry_v1_layout_mismatch() {
    using namespace gradient::ipc::v1;
    // Measured pre-P8-34 digest and record geometry, retained only for refusal.
    const std::string old_digest = "ae42a2ad68df32eff0d7745b7791a3c8b8e2f629991bbdb600b57d303e77b59c";
    TelemetryBatchHeaderV1 h{};
    h.magic = 0x31425452; h.version = 1; h.header_bytes = 312;
    h.record_bytes = 2088; h.cycle_period_ns = 1000000; h.axis_count = 1;
    std::fill(std::begin(h.adapter_incarnation), std::end(h.adapter_incarnation), 'a');
    std::fill(std::begin(h.daemon_incarnation), std::end(h.daemon_incarnation), 'b');
    std::fill(std::begin(h.machine_sha256), std::end(h.machine_sha256), 'c');
    std::fill(std::begin(h.deployment_sha256), std::end(h.deployment_sha256), 'd');
    std::memcpy(h.record_layout_digest, old_digest.data(), 64);
    const auto header = encode_TelemetryBatchHeaderV1(h);
    const std::string raw(reinterpret_cast<const char*>(header.data()), header.size());
    std::atomic<unsigned> requests{0};
    FakeServer server([&](const Incoming&) {
        ++requests;
        return "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: " +
            std::to_string(raw.size()) + "\r\n\r\n" + raw;
    });
    RtControlClient client(server.path);
    for (bool stream : {false, true}) {
        try {
            if (stream) client.telemetry_stream(0, [](const TelemetryPublication&) { assert(false); }, [] { return false; });
            else (void)client.telemetry(0);
            assert(false);
        } catch (const TelemetryLayoutMismatchError& error) {
            assert(error.stored_digest == old_digest);
            assert(error.expected_digest == CycleCaptureRecordV2LayoutDigest);
            const std::string message = error.what();
            assert(message.find(old_digest) != std::string::npos);
            assert(message.find(CycleCaptureRecordV2LayoutDigest) != std::string::npos);
        }
        assert(requests.load() == (stream ? 2u : 1u));
    }
    server.finish();
}

// Fragmented binary HTTP and explicit loss retain uint64 precision. Malformed
// generated records fail before callbacks can mistake partial data for evidence.
void telemetry_batches() {
    using namespace gradient::ipc::v1;
    TelemetryBatchHeaderV1 h{};
    h.magic = 0x31425452;
    h.version = 1;
    h.header_bytes = 312;
    h.record_bytes = 5392;
    h.cycle_period_ns = 1000000;
    h.axis_count = 1;
    h.record_count = 1;
    h.dropped = 3;
    h.first_sequence = UINT64_C(9007199254740997);
    h.last_sequence = h.first_sequence;
    std::fill(std::begin(h.adapter_incarnation), std::end(h.adapter_incarnation), 'a');
    std::fill(std::begin(h.daemon_incarnation), std::end(h.daemon_incarnation), 'e');
    std::fill(std::begin(h.machine_sha256), std::end(h.machine_sha256), 'b');
    std::fill(std::begin(h.deployment_sha256), std::end(h.deployment_sha256), 'c');
    std::memcpy(h.record_layout_digest, CycleCaptureRecordV2LayoutDigest, 64);
    CycleCaptureRecordV2 record{};
    record.seq = h.first_sequence;
    record.axes = 1;
    record.t_ns = 123;
    record.ax[0].position = -42;
    auto head = encode_TelemetryBatchHeaderV1(h);
    auto body = encode_CycleCaptureRecordV2(record);
    std::string raw(reinterpret_cast<const char*>(head.data()), head.size());
    raw.append(reinterpret_cast<const char*>(body.data()), body.size());
    FakeServer server([&](const Incoming& request) {
        assert(request.path.find("after=9007199254740993") != std::string::npos);
        if (request.path.find("/stream") == std::string::npos)
            return "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: " + std::to_string(raw.size()) + "\r\n\r\n" + raw;
        std::string reply = "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nTransfer-Encoding: chunked\r\n\r\n";
        for (char ch : raw) {
            reply += "1\r\n";
            reply += ch;
            reply += "\r\n";
        }
        return reply + "0\r\n\r\n";
    });
    RtControlClient client(server.path);
    auto batch = client.telemetry(UINT64_C(9007199254740993));
    assert(batch.Header.dropped == 3 && batch.Records.size() == 1 && batch.Records[0].ax[0].position == -42);
    bool done = false;
    client.telemetry_stream(UINT64_C(9007199254740993), [&](const TelemetryPublication& b) {
        assert(b.Header.last_sequence == h.last_sequence && b.Records.size() == 1);
        done = true;
    }, [&] { return done; });
    assert(done);
    auto bad = raw;
    bad[312+36] = 1;
    try { decode_telemetry(bad, UINT64_C(9007199254740993)); assert(false); } catch (const ProtocolError&) {}
    try { decode_telemetry(raw, UINT64_C(9007199254740994)); assert(false); } catch (const ProtocolError&) {}
    server.finish();

    // TestMaximumTelemetryHTTPBatch: the 22085944-byte publication exceeds the
    // former 8 MiB HTTP cap; all 4096 records must survive framing and decode.
    h.record_count = 4096; h.dropped = 0; h.first_sequence = 1; h.last_sequence = 4096;
    head = encode_TelemetryBatchHeaderV1(h);
    raw.assign(reinterpret_cast<const char*>(head.data()), head.size());
    for (uint64_t sequence = 1; sequence <= 4096; ++sequence) {
        record.seq = sequence;
        body = encode_CycleCaptureRecordV2(record);
        raw.append(reinterpret_cast<const char*>(body.data()), body.size());
    }
    assert(raw.size() == 22085944);
    // Measured fixture chunk size: 65536 bytes; small-fragment coverage is above.
    FakeServer maximum([&](const Incoming&) {
        return "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: 22085944\r\n\r\n" + raw;
    }, 65536);
    RtControlClient maximum_client(maximum.path);
    batch = maximum_client.telemetry(0);
    assert(batch.Records.size() == 4096 && batch.Records.back().seq == 4096);
    maximum.finish();
}

// TestReviewConnectionReopened keeps the peer-close observation permanent:
// connected and acquired reservations both admit Arm with its original identity.
void connection_reopened() {
    for (bool acquired : {false, true}) {
        std::atomic<int> arms{0};
        std::string identity;
        FakeServer server([&](const Incoming& incoming) {
            if (incoming.path == "/v1/describe" || incoming.path == "/v1/jog/ingress") return receipt("describe");
            Request request; assert(decode(incoming.body, request));
            if (request.Operation == "acquire") {
                Grant g; g.Session = "owner"; g.Generation = 7; g.LeaseMS = 500;
                return receipt("acquire", to_json(g));
            }
            assert(request.Operation == "arm" && request.RequestID);
            if (++arms == 1) { identity = incoming.body; return std::string{}; }
            assert(identity == incoming.body);
            return receipt("arm");
        });
        RtControlClient client(server.path);
        if (acquired) { Request r; r.Operation = "acquire"; client.command(r); }
        else client.set_fence({"owner", 7});
        client.maintain_observation(default_deadline(), true);
        client.maintain_connections(default_deadline());
        Request r; r.Operation = "arm";
        client.command(r);
        assert(arms == 2 && client.fence().Generation == 7);
        server.finish();
    }
}

// TestKeepAliveConnectedWithoutGrant observes three sweeps on every reserved
// stream, preventing selected Deck clients from relying on grant renewal.
void keepalive_connected() {
    std::atomic<unsigned> probes{0};
    FakeServer server([&](const Incoming& request) {
        if (request.path == "/v1/describe") {
            Description d; d.ControlIdleTimeoutNS = 300000000;
            return receipt("describe", to_json(d));
        }
        assert(request.path == "/v1/jog/ingress");
        ++probes;
        return receipt("jog_ingress");
    }, 65536);
    {
        RtControlClient client(dial(server.path));
        // Unverified fixture idle policy: 300 ms; Describe derives 100 ms probes.
        client.describe();
        assert(client.connection_keepalive_interval().count() == 100000000);
        observed([&] { return probes >= 24; });
        client.stop_connection_keepalive();
        assert(server.connections == 8 && client.fence().Session.empty());
        // Repeated jogs consume reservations before the idle keepalive interval.
        // The keeper must replenish promptly, without a cold dial in BeginJog.
        client.start_connection_keepalive(std::chrono::seconds(30));
        for (int i=0;i<4;++i) {
            const auto before=probes.load();
            auto stream=client.open_remote_jog_stream(default_deadline());
            assert(stream);
            observed([&] { return probes>=before+8; });
        }
        client.stop_connection_keepalive();
        assert(server.connections == 12 && client.fence().Session.empty());
    }
    server.finish();
}

int main(int argc, char** argv) {
    try {
        if (argc == 4) return integration(argv[1], argv[2], argv[3]);
        // Name the failing contract so an I/O deadline cannot hide its source.
        const std::pair<const char*, void(*)()> checks[] = {
            {"framing", framing}, {"retries_and_grants", retries_and_grants},
            {"TestReviewConnectionReopened", connection_reopened},
            {"TestKeepAliveConnectedWithoutGrant", keepalive_connected},
            {"uncertain_upload", uncertain_upload}, {"jog_frame", jog_frame},
            {"TestReviewDeckExpiredRecovery", expired_stop_release},
            {"remote_jog_contract", remote_jog_contract}, {"remote_jog_independent_reads", remote_jog_independent_reads},
            {"restore_anchor_recovery", restore_anchor_recovery}, {"events", events},
            {"event_stream_cancellation", event_stream_cancellation}, {"telemetry_busy_rejected", telemetry_busy_rejected},
            {"telemetry_batches", telemetry_batches},
            {"telemetry_v1_layout_mismatch", telemetry_v1_layout_mismatch},
        };
        for (const auto& check : checks) {
            try { check.second(); } catch (...) { std::cerr << check.first << ": "; throw; }
        }
        Request r; r.Operation = "start_trajectory"; r.RequestID = new_request_id(); r.Fence = {"token", UINT64_MAX}; r.Handle = UINT64_MAX;
        Request decoded; assert(decode(encode(r), decoded));
        assert(decoded.Handle == UINT64_MAX && decoded.Fence.Generation == UINT64_MAX && decoded.RequestID == r.RequestID);
        // Named fields: the generated AxisStatus grows with the schema, so a
        // positional initialiser would silently shift when fields are added.
        AxisStatus axis{}; axis.PositionCounts = 123; axis.Statusword = 0x27; axis.ErrorCode = 0;
        axis.VendorID = 1; axis.ProductCode = 2; axis.SlavePosition = 3;
        ProcessStatus status; status.Core.Axes.push_back(axis);
        ProcessStatus roundtrip; assert(decode(encode(status), roundtrip)); assert(roundtrip.Core.Axes[0].PositionCounts == 123);
        std::cout << "C++ client tests passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
