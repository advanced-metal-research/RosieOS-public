#pragma once
#include "rt_control_api_generated.hpp"
#include "protocol_generated.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace rosie::rt_control {
// Path jog (protocol 1.14) is not supported by this client: there is no
// begin_path_jog, chunk handshake or path observation helper. Use the Go SDK.

struct TransportError : std::runtime_error { using std::runtime_error::runtime_error; };
struct ProtocolError : TransportError { using TransportError::TransportError; };
// Layout refusal preserves both identities and ends decoding before any record
// is observed; reconnecting cannot repair an incompatible wire schema.
struct TelemetryLayoutMismatchError : ProtocolError {
    std::string stored_digest, expected_digest;
    TelemetryLayoutMismatchError(const std::string& stored, const std::string& expected)
        : ProtocolError("telemetry layout digest mismatch: stored=" + stored + " expected=" + expected),
          stored_digest(stored), expected_digest(expected) {}
};
struct DeadlineExceeded : TransportError { using TransportError::TransportError; };
// Preserve transmission evidence so a lost upload or Release receipt cannot
// be replayed as though the adapter had received no request bytes.
struct ConnectionFailure : TransportError {
    bool sent;
    ConnectionFailure(const std::string& message, bool transmitted)
        : TransportError(transmitted ? message : "rt_core_connection_reopened: request not sent: " + message), sent(transmitted) {}
};
struct Rejected : std::runtime_error {
    std::string reason;
    std::optional<CommandResult> native_result;
    std::optional<JogObservation> native_jog_result;
    explicit Rejected(const Response& r, const std::string& detail = {})
        : std::runtime_error(detail.empty() ? r.Error : detail), reason(r.Error), native_result(r.NativeResult),
          native_jog_result(r.NativeJogResult) {}
};

// Preserve the typed refusal and absolute warm measurement so a cell ceiling
// failure cannot be mistaken for a stale credential or cold TLS delay.
struct LeaseTimingRejected : Rejected {
    std::chrono::nanoseconds round_trip, ceiling;
    LeaseTimingRejected(std::chrono::nanoseconds measured, std::chrono::nanoseconds limit)
        : Rejected(refusal(), std::string(ReasonControlSessionStale) + ": measured_warm_rtt_ns=" +
                   std::to_string(measured.count()) + " lease_ceiling_ns=" + std::to_string(limit.count())),
          round_trip(measured), ceiling(limit) {}
private:
    static Response refusal() { Response r; r.Error = ReasonControlSessionStale; return r; }
};

// Thrown when the stream reports events_dropped: adapter history before
// event.Sequence is gone and streaming must not silently advance over it. The
// generated loss event is retained so recovery can reconcile the exact range from
// Status, then resume with after = event.Sequence; resuming from the previous
// cursor repeats the loss.
struct EventCursorLost : Rejected {
    Event event;
    explicit EventCursorLost(const Event& lost) : Rejected(refusal()), event(lost) {}
private:
    static Response refusal() { Response r; r.Error = ReasonEventCursorLost; return r; }
};

using Deadline = std::chrono::steady_clock::time_point;
// Unverified application receipt budget: 5,000 ms, including lane wait and retry.
inline Deadline default_deadline() { return std::chrono::steady_clock::now() + std::chrono::milliseconds(5000); }

// Remote listeners require a TLS implementation of this hook with authenticated
// server trust and client credentials. Factories must return independent streams;
// read/write/connect must honor the absolute deadline and never replay bytes.
// A read timeout throws DeadlineExceeded without consuming bytes, allowing an
// idle event stream to check its stop predicate without abandoning its cursor.
class HttpStream {
public:
    virtual ~HttpStream() = default;
    virtual std::size_t read(char* data, std::size_t size, Deadline deadline) = 0;
    virtual std::size_t write(const char* data, std::size_t size, Deadline deadline) = 0;
};
using StreamFactory = std::function<std::unique_ptr<HttpStream>(Deadline)>;

namespace client_detail {
inline sockaddr_un unix_address(const std::string& path) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.empty() || path.size() >= sizeof(address.sun_path) || path.find('\0') != std::string::npos)
        throw std::invalid_argument("invalid Unix socket path");
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    return address;
}
inline void wait_fd(int fd, short events, Deadline deadline) {
    for (;;) {
        auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= Deadline::duration::zero()) throw DeadlineExceeded("control deadline; outcome may be unknown");
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
        pollfd p{fd, events, 0};
        int result = ::poll(&p, 1, static_cast<int>(std::min<std::int64_t>(ms + 1, INT32_MAX)));
        if (result > 0) return;
        if (result < 0 && errno != EINTR) throw TransportError(std::strerror(errno));
    }
}
class UnixStream final : public HttpStream {
    int fd_ = -1;
public:
    UnixStream(const std::string& path, Deadline deadline) {
        auto address = unix_address(path);
        fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        if (fd_ < 0) throw TransportError(std::strerror(errno));
        try {
            if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
                if (errno != EINPROGRESS) throw TransportError(std::strerror(errno));
                wait_fd(fd_, POLLOUT, deadline);
                int error = 0;
                socklen_t size = sizeof(error);
                if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error != 0)
                    throw TransportError(std::strerror(error ? error : errno));
            }
        } catch (...) { ::close(fd_); fd_ = -1; throw; }
    }
    ~UnixStream() override { if (fd_ >= 0) ::close(fd_); }
    std::size_t read(char* data, std::size_t size, Deadline deadline) override {
        for (;;) {
            wait_fd(fd_, POLLIN, deadline);
            auto n = ::recv(fd_, data, size, 0);
            if (n >= 0) return static_cast<std::size_t>(n);
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) throw TransportError(std::strerror(errno));
        }
    }
    std::size_t write(const char* data, std::size_t size, Deadline deadline) override {
        for (;;) {
            wait_fd(fd_, POLLOUT, deadline);
            auto n = ::send(fd_, data, size, MSG_NOSIGNAL);
            if (n > 0) return static_cast<std::size_t>(n);
            if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
                throw TransportError("control write failed; outcome may be unknown");
        }
    }
};
inline std::string lower(std::string text) {
    for (char& c : text) if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    return text;
}
inline std::string trim(std::string text) {
    auto start = text.find_first_not_of(" \t");
    if (start == std::string::npos) return {};
    return text.substr(start, text.find_last_not_of(" \t") - start + 1);
}
inline std::size_t number(const std::string& text, int base) {
    std::size_t out = 0;
    auto result = std::from_chars(text.data(), text.data() + text.size(), out, base);
    if (text.empty() || result.ec != std::errc{} || result.ptr != text.data() + text.size())
        throw ProtocolError("invalid HTTP length");
    return out;
}
struct HttpResponse { int status; std::string body; };
struct StreamStopped {};

// Each lane owns its HTTP parser and socket. Malformed or incomplete framing
// destroys that connection, preventing ambiguous bytes from becoming a reply.
class Lane {
    StreamFactory factory_;
    std::unique_ptr<HttpStream> stream_;
    std::string pending_;
    std::timed_mutex mutex_;
    const std::function<bool()>* stop_ = nullptr;
    // Wire bound: 312-byte header plus 4096 records of 5392 bytes.
    static constexpr std::size_t body_limit = 22085944;
    // Unverified resource ceiling: 64 KiB headers/trailers.
    static constexpr std::size_t header_limit = 64 * 1024;
    void receive(Deadline deadline) {
        char buffer[8192];
        std::size_t n;
        for (;;) {
            if (stop_ && (*stop_)()) throw StreamStopped{};
            // Unverified idle cancellation check interval: 50 ms.
            auto read_deadline = stop_ ? std::min(deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(50)) : deadline;
            try { n = stream_->read(buffer, sizeof(buffer), read_deadline); break; }
            catch (const DeadlineExceeded&) {
                if (!stop_ || std::chrono::steady_clock::now() >= deadline) throw;
            }
        }
        if (!n) throw TransportError("control EOF; outcome may be unknown");
        if (n > sizeof(buffer)) throw ProtocolError("invalid stream read");
        pending_.append(buffer, n);
    }
    std::string line(Deadline deadline, std::size_t& budget) {
        for (;;) {
            auto end = pending_.find("\r\n");
            if (end != std::string::npos) {
                if (end + 2 > budget) throw ProtocolError("HTTP headers too large");
                budget -= end + 2;
                auto out = pending_.substr(0, end);
                pending_.erase(0, end + 2);
                return out;
            }
            if (pending_.size() >= budget) throw ProtocolError("HTTP line too large");
            receive(deadline);
        }
    }
    std::string bytes(std::size_t size, Deadline deadline) {
        if (size > body_limit) throw ProtocolError("HTTP body too large");
        while (pending_.size() < size) receive(deadline);
        auto out = pending_.substr(0, size);
        pending_.erase(0, size);
        return out;
    }
    static std::pair<std::string, std::string> header(const std::string& line) {
        auto colon = line.find(':');
        if (colon == 0 || colon == std::string::npos) throw ProtocolError("malformed HTTP header");
        for (std::size_t i = 0; i < colon; ++i) {
            auto c = static_cast<unsigned char>(line[i]);
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                  std::string_view("!#$%&'*+-.^_`|~").find(char(c)) != std::string_view::npos))
                throw ProtocolError("invalid HTTP header name");
        }
        for (std::size_t i = colon + 1; i < line.size(); ++i)
            if ((static_cast<unsigned char>(line[i]) < 32 && line[i] != '\t') || line[i] == 127)
                throw ProtocolError("invalid HTTP header value");
        return {lower(line.substr(0, colon)), trim(line.substr(colon + 1))};
    }
public:
    explicit Lane(StreamFactory factory) : factory_(std::move(factory)) {}
    // Transfer an already warmed HTTP socket for WSS; stale parser bytes cannot
    // cross the upgrade boundary or cause a jog request to open a new socket.
    std::unique_ptr<HttpStream> take_stream() {
        std::lock_guard<std::timed_mutex> lock(mutex_);
        if (!pending_.empty()) throw ProtocolError("pending upgrade bytes");
        return std::move(stream_);
    }
    HttpResponse exchange(const std::string& method, const std::string& path, const std::string& body,
                          const std::vector<std::pair<std::string, std::string>>& headers, Deadline deadline,
                          const std::function<void(const std::string&)>& consume = {}, const std::function<bool()>& stop = {},
                          bool allow_connect = true, bool idle_only = false) {
        std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
        if (idle_only) { if (!lock.try_lock()) return {}; }
        else if (!lock.try_lock_until(deadline)) throw DeadlineExceeded("control lane deadline");
        stop_ = stop ? &stop : nullptr;
        bool request_sent = false;
        try {
            if (stop_ && (*stop_)()) throw StreamStopped{};
            if (!stream_ && !allow_connect) throw TransportError("control warm connection unavailable");
            if (!stream_) stream_ = factory_(deadline);
            if (!stream_) throw TransportError("null HTTP stream");
            std::string request = method + " " + path + " HTTP/1.1\r\nHost: localhost\r\nConnection: keep-alive\r\nContent-Length: " + std::to_string(body.size()) + "\r\n";
            for (const auto& h : headers) {
                if (h.second.find_first_of("\r\n") != std::string::npos || h.second.find('\0') != std::string::npos)
                    throw ProtocolError("invalid request header");
                request += h.first + ": " + h.second + "\r\n";
            }
            request += "\r\n";
            auto send = [&](const std::string& data) {
                for (std::size_t sent = 0; sent < data.size();) {
                    // A stream exception may follow a partial TLS write.
                    request_sent = true;
                    auto n = stream_->write(data.data() + sent, data.size() - sent, deadline);
                    if (!n || n > data.size() - sent) throw TransportError("invalid stream write");
                    sent += n;
                }
            };
            send(request); send(body);
            std::size_t budget = header_limit;
            auto status = line(deadline, budget);
            if (status.size() < 13 || status.substr(0, 9) != "HTTP/1.1 " || status[12] != ' ' ||
                status[9] < '1' || status[9] > '5' || status[10] < '0' || status[10] > '9' || status[11] < '0' || status[11] > '9')
                throw ProtocolError("invalid HTTP status line");
            HttpResponse response{static_cast<int>(number(status.substr(9, 3), 10)), {}};
            bool chunked = false, close = false, have_transfer = false;
            std::string content_type;
            std::optional<std::size_t> length;
            for (;;) {
                auto text = line(deadline, budget);
                if (text.empty()) break;
                auto h = header(text);
                if (h.first == "content-length") {
                    if (length) throw ProtocolError("duplicate HTTP content-length");
                    length = number(h.second, 10);
                } else if (h.first == "transfer-encoding") {
                    if (have_transfer || lower(h.second) != "chunked") throw ProtocolError("unsupported HTTP transfer-encoding");
                    have_transfer = chunked = true;
                } else if (h.first == "connection") {
                    std::istringstream tokens(lower(h.second));
                    std::string token;
                    while (std::getline(tokens, token, ',')) if (trim(token) == "close") close = true;
                } else if (h.first == "content-type") content_type = lower(trim(h.second.substr(0, h.second.find(';'))));
            }
            if (chunked && length) throw ProtocolError("ambiguous HTTP framing");
            const bool streaming = consume && response.status == 200;
            const bool telemetry = path.rfind("/v1/telemetry",0) == 0;
            if (response.status == 200 && ((telemetry && content_type != "application/octet-stream") ||
                (streaming && !telemetry && content_type != "text/event-stream"))) throw ProtocolError("invalid observation content-type");
            if (chunked) {
                for (;;) {
                    if (stop_ && (*stop_)()) throw StreamStopped{};
                    if (streaming) budget = header_limit;
                    // Unverified chunk metadata ceiling: 64 KiB per finite response or SSE chunk.
                    auto size_line = line(deadline, budget);
                    // Chunk extensions are deliberately unsupported; fail closed.
                    auto size = number(size_line, 16);
                    if (!size) break;
                    if (size > body_limit - response.body.size()) throw ProtocolError("HTTP body too large");
                    auto part = bytes(size, deadline);
                    if (bytes(2, deadline) != "\r\n") throw ProtocolError("invalid HTTP chunk terminator");
                    if (streaming) consume(part); else response.body += part;
                }
                for (;;) {
                    auto trailer = line(deadline, budget);
                    if (trailer.empty()) break;
                    auto h = header(trailer);
                    if (h.first == "content-length" || h.first == "transfer-encoding") throw ProtocolError("HTTP framing trailer");
                }
            } else if (length) {
                auto part = bytes(*length, deadline);
                if (streaming) consume(part); else response.body = std::move(part);
            }
            else throw ProtocolError("HTTP response requires an explicit length");
            if (!pending_.empty()) throw ProtocolError("unsolicited HTTP response bytes");
            if (close) stream_.reset();
            stop_ = nullptr;
            return response;
        } catch (const ProtocolError&) { stream_.reset(); pending_.clear(); stop_ = nullptr; throw;
        } catch (const DeadlineExceeded&) { stream_.reset(); pending_.clear(); stop_ = nullptr; throw;
        } catch (const TransportError& error) {
            stream_.reset(); pending_.clear(); stop_ = nullptr;
            throw ConnectionFailure(error.what(), request_sent);
        } catch (...) { stream_.reset(); pending_.clear(); stop_ = nullptr; throw; }
    }
};
} // namespace client_detail

// Sixteen random bytes, encoded as 32 hex digits, match the public retry key
// contract. Failure to obtain OS entropy fails before issuing any mutation.
inline std::string new_request_id() {
    std::array<unsigned char, 16> bytes{};
    int fd = ::open("/dev/urandom", O_RDONLY | O_CLOEXEC);
    if (fd < 0) throw TransportError("request ID entropy unavailable");
    std::size_t done = 0;
    while (done < bytes.size()) {
        auto n = ::read(fd, bytes.data() + done, bytes.size() - done);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { ::close(fd); throw TransportError("request ID entropy read failed"); }
        done += static_cast<std::size_t>(n);
    }
    ::close(fd);
    std::string id;
    for (auto byte : bytes) { id += "0123456789abcdef"[byte >> 4]; id += "0123456789abcdef"[byte & 15]; }
    return id;
}

struct Renewal { Grant grant; std::exception_ptr error; };

struct TelemetryPublication {
    gradient::ipc::v1::TelemetryBatchHeaderV1 Header{};
    std::vector<gradient::ipc::v1::CycleCaptureRecordV2> Records;
};

// Validate generated geometry before allocating a batch; malformed identities,
// sizes or sequence ranges never become consumer observations.
inline gradient::ipc::v1::TelemetryBatchHeaderV1 telemetry_header(const std::string& raw) {
    using namespace gradient::ipc::v1;
    TelemetryBatchHeaderV1 h{};
    if (raw.size() < sizeof(h) || !decode_TelemetryBatchHeaderV1(reinterpret_cast<const uint8_t*>(raw.data()), sizeof(h), h))
        throw ProtocolError("invalid telemetry header");
    if (std::memcmp(h.record_layout_digest, CycleCaptureRecordV2LayoutDigest, 64) != 0)
        throw TelemetryLayoutMismatchError(std::string(reinterpret_cast<const char*>(h.record_layout_digest), 64), CycleCaptureRecordV2LayoutDigest);
    auto hex = [](const auto& bytes) { for (auto c : bytes) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false; return true; };
    if (h.magic != 0x31425452 || h.version != 1 || h.header_bytes != sizeof(h) || h.record_bytes != sizeof(CycleCaptureRecordV2) ||
        h.record_count > 4096 || h.axis_count < 1 || h.axis_count > 16 || h.cycle_period_ns < 250000 || h.cycle_period_ns > 10000000 ||
        !hex(h.adapter_incarnation) || !hex(h.daemon_incarnation) || !hex(h.machine_sha256) || !hex(h.deployment_sha256) ||
        (!h.record_count && h.first_sequence) || (h.record_count && (!h.first_sequence || h.last_sequence < h.first_sequence || h.last_sequence-h.first_sequence != h.record_count-1)))
        throw ProtocolError("incompatible telemetry header");
    return h;
}

// Decode complete generated records and exact loss; uint64 wrap cannot conceal a gap.
inline TelemetryPublication decode_telemetry(const std::string& raw, std::uint64_t after) {
    using namespace gradient::ipc::v1;
    TelemetryPublication b; b.Header = telemetry_header(raw); const auto& h = b.Header;
    if (raw.size() != sizeof(h) + std::size_t(h.record_count)*sizeof(CycleCaptureRecordV2) || h.dropped > UINT64_MAX-after)
        throw ProtocolError("invalid telemetry extent");
    auto next = after + h.dropped;
    if ((!h.record_count && h.last_sequence != next) || (h.record_count && (next == UINT64_MAX || h.first_sequence != next+1)))
        throw ProtocolError("unreported telemetry loss");
    b.Records.reserve(h.record_count);
    for (std::uint32_t i=0; i<h.record_count; ++i) {
        CycleCaptureRecordV2 record{};
        if (!decode_CycleCaptureRecordV2(reinterpret_cast<const uint8_t*>(raw.data()) + sizeof(h) + i*sizeof(record), sizeof(record), record) ||
            record.seq != h.first_sequence+i || record.axes != h.axis_count) throw ProtocolError("invalid telemetry record");
        b.Records.push_back(record);
    }
    return b;
}

// Local state locks never cover socket traffic. Destruction joins renewal and
// closes streams; callers must join other operations first and release explicitly.
class RtControlClient {
    StreamFactory factory_;
    bool remote_stream_ = true;
    client_detail::Lane lifecycle_, urgent_, bulk_, renewal_, observation_, jog_spare_, events_spare_, telemetry_spare_;
    bool warm_ready_ = false;
    Deadline acquire_started_{};
    std::chrono::nanoseconds round_trip_{0};
    std::chrono::nanoseconds lease_ceiling_{0};
    // Unverified legacy idle timeout: 30000 ms, before additive Describe timing.
    std::chrono::nanoseconds idle_timeout_{std::chrono::seconds(30)};
    // Unverified LAN compatibility default: 250 ms source age.
    std::uint64_t jog_input_age_ns_ = 250000000;
    mutable std::mutex state_mutex_;
    Grant grant_{};
    Fence recovery_fence_{};
    std::mutex renewal_lifecycle_, renewal_mutex_;
    std::condition_variable renewal_cv_;
    std::thread renewal_thread_;
    bool renewal_stop_ = false;
    std::optional<Renewal> renewal_update_;
    std::mutex keep_mutex_;
    std::condition_variable keep_cv_;
    std::thread keep_thread_;
    bool keep_stop_ = false;
    bool keep_requested_ = false;

    Response call(client_detail::Lane& lane, const std::string& method, const std::string& path,
                  const std::string& body, const std::vector<std::pair<std::string, std::string>>& headers,
                  bool retry, Deadline deadline, const std::function<bool()>& stop = {}) {
        bool uncertain = false;
        for (unsigned attempt = 0;; ++attempt) {
            client_detail::HttpResponse wire{};
            bool allow_connect;
            { std::lock_guard<std::mutex> lock(state_mutex_);
              allow_connect = !warm_ready_ || &lane == &renewal_ || attempt == 1; }
            try { wire = lane.exchange(method, path, body, headers, deadline, {}, stop, allow_connect); }
            catch (const ProtocolError&) { throw; }
            catch (const ConnectionFailure& error) {
                uncertain = uncertain || error.sent;
                if ((!retry && uncertain) || attempt != 0 || std::chrono::steady_clock::now() >= deadline) throw;
                std::clog << "rt_core_connection_reopened method=" << method << " path=" << path << '\n';
                continue;
            }
            Response response;
            std::string error;
            if (!decode(wire.body, response, &error) || response.Schema != ResponseSchema)
                throw ProtocolError("invalid control response: " + error);
            if (wire.status == 409 && !response.Error.empty()) throw Rejected(response);
            if (wire.status != 200 || !response.Error.empty()) throw ProtocolError("unexpected control HTTP response");
            return response;
        }
    }
    void remember(const Request& request, const Grant& next) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (request.Operation == to_json(Operation::Acquire).text) {
            if (grant_.Session.empty()) { grant_ = next; recovery_fence_ = {}; }
        } else if (request.Operation == to_json(Operation::Release).text &&
                   recovery_fence_.Session == request.Fence.Session && recovery_fence_.Generation == request.Fence.Generation) {
            recovery_fence_ = {}; grant_ = {};
        } else if (grant_.Session == request.Fence.Session) {
            if (request.Operation == to_json(Operation::Release).text) { grant_ = {}; return; }
            // Empty-session Stop retires motion authority. Its old fence is
            // retained only for recovery, never for renewal or motion commands.
            if (request.Operation == to_json(Operation::Stop).text && next.Session.empty()) {
                grant_ = next; recovery_fence_ = request.Fence; return;
            }
            if (next.Generation == grant_.Generation && next.DeadlineHostNS == grant_.DeadlineHostNS && !grant_.Stopping && next.Stopping) return;
            if (next.Generation > grant_.Generation || (next.Generation == grant_.Generation && next.DeadlineHostNS >= grant_.DeadlineHostNS)) grant_ = next;
        }
    }
    template<class T> T get(const std::string& path, Deadline deadline) {
        return data<T>(call(observation_, "GET", path, {}, {}, true, deadline));
    }
    void replenish_reservation() {
        { std::lock_guard<std::mutex> lock(keep_mutex_); keep_requested_ = true; }
        keep_cv_.notify_one();
    }
    // Streaming reads consume a prewarmed socket after acquisition. Only keeper
    // maintenance replenishes it; reconnecting a stream cannot initiate TLS.
    StreamFactory reserved_stream(client_detail::Lane& spare) {
        return [this, &spare](Deadline d) {
            bool warm;
            { std::lock_guard<std::mutex> lock(state_mutex_); warm = warm_ready_; }
            if (!warm) return factory_(d);
            auto stream = spare.take_stream();
            replenish_reservation();
            if (!stream) throw TransportError("control warm stream unavailable");
            return stream;
        };
    }
    // Warm all independent lanes before authority, then touch idle lanes from
    // the keeper. Busy lifecycle work cannot consume the urgent reservation.
    void warm_connections(Deadline deadline, bool initial) {
        std::vector<std::future<void>> work;
        for (auto* lane : {&lifecycle_, &urgent_, &bulk_, &renewal_, &observation_, &jog_spare_, &events_spare_, &telemetry_spare_}) {
            if (lane == &jog_spare_ && !remote_stream_) continue;
            work.push_back(std::async(std::launch::async, [lane, deadline, initial] {
                try {
                    lane->exchange("GET", initial ? "/v1/describe" : "/v1/jog/ingress", {}, {}, deadline, {}, {}, true, !initial);
                } catch (const ConnectionFailure&) {
                    std::clog << "rt_core_connection_reopened method=GET path=/v1/jog/ingress\n";
                    lane->exchange("GET", "/v1/jog/ingress", {}, {}, deadline, {}, {}, true, !initial);
                }
            }));
        }
        std::exception_ptr error;
        for (auto& task : work) { try { task.get(); } catch (...) { error = std::current_exception(); } }
        if (initial && error) std::rethrow_exception(error);
    }
    Response simple(std::string operation, Deadline deadline, std::uint32_t mask = 0, std::uint64_t handle = 0) {
        Request r; r.Operation = std::move(operation); r.AxisMask = mask; r.Handle = handle;
        return command(r, deadline);
    }
public:
    explicit RtControlClient(const std::string& socket)
        : RtControlClient(StreamFactory([socket](Deadline d) { return std::make_unique<client_detail::UnixStream>(socket, d); })) {
        (void)client_detail::unix_address(socket);
        remote_stream_ = false;
    }
    explicit RtControlClient(StreamFactory factory) : factory_(factory), lifecycle_(factory), urgent_(factory), bulk_(factory), renewal_(factory), observation_(factory), jog_spare_(factory), events_spare_(factory), telemetry_spare_(std::move(factory)) {}
    ~RtControlClient() { stop_connection_keepalive(); stop_renewal(); }
    RtControlClient(const RtControlClient&) = delete;
    RtControlClient& operator=(const RtControlClient&) = delete;

    // Open an independent stream using the caller's authenticated TLS factory.
    // Local clients cannot accidentally send remote credentials over plain HTTP.
    std::unique_ptr<HttpStream> open_remote_jog_stream(Deadline d) {
        if (!remote_stream_) throw std::invalid_argument("remote jog requires an authenticated stream factory");
        { std::lock_guard<std::mutex> lock(state_mutex_);
          if (warm_ready_) {
              auto stream = jog_spare_.take_stream();
              replenish_reservation();
              if (!stream) throw TransportError("control warm jog connection unavailable");
              return stream;
          } }
        auto stream = factory_(d);
        if (!stream) throw TransportError("null remote jog stream");
        return stream;
    }

    template<class T> static T data(const Response& r) {
        try { T value{}; from_json(r.Data, value); return value; }
        catch (const std::exception& e) { throw ProtocolError(std::string("invalid response data: ") + e.what()); }
    }
    Grant grant() const { std::lock_guard<std::mutex> lock(state_mutex_); return grant_; }
    Fence fence() const { auto g = grant(); return {g.Session, g.Generation}; }
    // Explicit fence replacement is a setup operation, before concurrent calls.
    void set_fence(const Fence& f) { std::lock_guard<std::mutex> lock(state_mutex_); grant_ = {}; recovery_fence_ = {}; grant_.Session = f.Session; grant_.Generation = f.Generation; }

    // Retains ID and fence in the caller's request for an explicit later retry.
    // Do not alter/share a request between attempts; retries require its live
    // session's retained outcome window. Uncertain results never authorize Start.
    // Cancellation abandons the receipt and closes its lane. The caller must
    // fence uncertain execution; a cancelled exchange is never retried.
    Response command(Request& r, Deadline deadline = default_deadline(), const std::function<bool()>& stop = {}) {
        if (!r.RequestID) r.RequestID = new_request_id();
        if (r.RequestID->empty()) throw std::invalid_argument("empty request ID");
        if (r.Operation != to_json(Operation::Acquire).text && r.Fence.Session.empty() && r.Fence.Generation == 0) {
            r.Fence = fence();
            if (r.Fence.Session.empty() &&
                (r.Operation == to_json(Operation::Stop).text || r.Operation == to_json(Operation::Release).text)) {
                std::lock_guard<std::mutex> lock(state_mutex_); r.Fence = recovery_fence_;
            }
        }
        r.Schema = RequestSchema;
        bool remote_acquire = false;
        if (r.Operation == to_json(Operation::Acquire).text) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // Unverified LAN RTT bound: 20 ms, matching the Go client.
            remote_acquire = round_trip_ > std::chrono::milliseconds(20);
        }
        // Acquire warms the dedicated keeper connection before authority starts.
        auto& lane = r.Operation == to_json(Operation::PrepareTrajectory).text ? bulk_ :
            (r.Operation == to_json(Operation::Renew).text || remote_acquire ? renewal_ :
            (r.Operation == to_json(Operation::Stop).text || r.Operation == to_json(Operation::Release).text ? urgent_ : lifecycle_));
        if (r.Operation == to_json(Operation::Acquire).text) {
            std::lock_guard<std::mutex> lock(state_mutex_); acquire_started_ = std::chrono::steady_clock::now();
        }
        auto response = call(lane, "POST", "/v1/control", encode(r), {{"Content-Type", "application/json"}}, r.Operation != to_json(Operation::Release).text, deadline, stop);
        if (response.Operation != r.Operation) throw ProtocolError("control operation mismatch");
        if (r.Operation == to_json(Operation::Acquire).text || r.Operation == to_json(Operation::Renew).text || r.Operation == to_json(Operation::Stop).text || r.Operation == to_json(Operation::Release).text) remember(r, data<Grant>(response));
        return response;
    }
    Grant acquire(const std::string& controller, const Binding& binding, Deadline d = default_deadline()) {
        bool measured;
        { std::lock_guard<std::mutex> lock(state_mutex_); measured = round_trip_.count() > 0; }
        if (!measured) describe(d);
        std::chrono::nanoseconds rtt, ceiling;
        { std::lock_guard<std::mutex> lock(state_mutex_); rtt = round_trip_; ceiling = lease_ceiling_; }
        // Unverified LAN compatibility floor and ceiling: 500 ms.
        if (ceiling.count() <= 0) ceiling = std::chrono::milliseconds(500);
        if (ceiling < 3*rtt+ceiling/3) throw LeaseTimingRejected(rtt, ceiling);
        auto requested = std::min(ceiling, std::max(std::chrono::nanoseconds(500000000), (9*rtt+std::chrono::nanoseconds(1))/2));
        // Unverified LAN RTT bound: 20 ms; remote links reserve the cell ceiling.
        if (rtt > std::chrono::milliseconds(20)) requested = ceiling;
        auto ms = std::min(ceiling.count()/1000000, (requested.count()+999999)/1000000);
        // Unverified LAN RTT boundary: 20 ms, shared with the Go SDK.
        if (rtt > std::chrono::milliseconds(20)) {
            warm_connections(d, true);
            std::lock_guard<std::mutex> lock(state_mutex_); warm_ready_ = true;
        }
        Request r; r.Operation = to_json(Operation::Acquire).text; r.Controller = controller; r.Binding = binding; r.RequestedLeaseMS = ms;
        auto g = data<Grant>(command(r, d));
        auto lease = std::chrono::milliseconds(g.LeaseMS);
        if (g.LeaseMS <= 0 || g.LeaseMS > ms || lease < 3*rtt+lease/3) {
            try { release(d); } catch (...) {}
            { std::lock_guard<std::mutex> lock(state_mutex_);
              if (grant_.Session == g.Session && grant_.Generation == g.Generation) grant_ = {}; }
            Response rejected; rejected.Error = ReasonControlSessionStale; throw Rejected(rejected);
        }
        return g;
    }
    Grant renew(Deadline d = default_deadline()) { return data<Grant>(simple(to_json(Operation::Renew).text, d)); }
    Grant stop(Deadline d = default_deadline()) { return data<Grant>(simple(to_json(Operation::Stop).text, d)); }
    Grant release(Deadline d = default_deadline()) { stop_renewal(); return data<Grant>(simple(to_json(Operation::Release).text, d)); }
    Response enable(std::uint32_t mask, Deadline d = default_deadline()) { return simple(to_json(Operation::Enable).text, d, mask); }
    Response arm(Deadline d = default_deadline()) { return simple(to_json(Operation::Arm).text, d); }
    // Permission receipt only; independent feedback remains in status.Core.IO.
    Response io_arm(Deadline d = default_deadline()) { return simple(to_json(Operation::IoArm).text, d); }
    Response io_disarm(Deadline d = default_deadline()) { return simple(to_json(Operation::IoDisarm).text, d); }
    Response home(std::uint32_t mask, Deadline d = default_deadline()) { return simple(to_json(Operation::Home).text, d, mask); }
    // Decode the observed recovery contract; a sequence-only reply cannot establish restored Home evidence.
    RecoveryStatus restore_anchor(std::uint32_t mask, Deadline d = default_deadline()) { return data<RecoveryStatus>(simple(to_json(Operation::RestoreAnchor).text, d, mask)); }
    Response reset_fault(std::uint32_t mask, Deadline d = default_deadline()) { return simple(to_json(Operation::ResetFault).text, d, mask); }
    RecoveryStatus recovery_status(Deadline d = default_deadline()) { return data<RecoveryStatus>(simple(to_json(Operation::RecoveryStatus).text, d)); }
    Response jog(std::uint32_t mask, const std::vector<double>& velocity, std::uint32_t timeout_ms, Deadline d = default_deadline()) {
        Request r; r.Operation = to_json(Operation::Jog).text; r.AxisMask = mask; r.Velocity = velocity; r.TimeoutMS = timeout_ms; return command(r, d);
    }
    Response begin_jog(std::uint32_t mask, const std::string& clock, std::uint64_t origin_ns, std::uint64_t deadline_ns, Deadline d = default_deadline()) {
        Request r; r.Operation = to_json(Operation::BeginJog).text; r.AxisMask = mask; r.ClockIncarnation = clock; r.SourceOriginHostNS = origin_ns; r.DeadlineHostNS = deadline_ns; return command(r, d);
    }
    Response end_jog(std::uint64_t generation, Deadline d = default_deadline()) {
        Request r; r.Operation = to_json(Operation::EndJog).text; r.JogGeneration = generation; return command(r, d);
    }
    Response prepare_trajectory(std::uint32_t mask, const std::vector<Point>& points, Deadline d = default_deadline()) {
        Request r; r.Operation = to_json(Operation::PrepareTrajectory).text; r.AxisMask = mask; r.Points = points; return command(r, d);
    }
    Response start_trajectory(std::uint64_t handle, Deadline d = default_deadline()) { return simple(to_json(Operation::StartTrajectory).text, d, 0, handle); }
    Response discard_trajectory(std::uint64_t handle, Deadline d = default_deadline()) { return simple(to_json(Operation::DiscardTrajectory).text, d, 0, handle); }
    Response start_program(const Identity& identity, Deadline d = default_deadline()) {
        Request r; r.Operation = to_json(Operation::StartProgram).text; r.Identity = identity; return command(r, d);
    }
    // The adapter ignores upload retry IDs. Never replay an uncertain .rdt body;
    // inspect status or Stop explicitly before proceeding with new authority.
    Program prepare_program(const std::string& bytes, Deadline d = default_deadline()) {
        auto f = fence();
        return data<Program>(call(bulk_, "POST", "/v1/program", bytes,
            {{"Content-Type", "application/octet-stream"}, {"X-Control-Session", f.Session},
             {"X-Control-Generation", std::to_string(f.Generation)}, {"X-Request-ID", new_request_id()}}, false, d));
    }
    // Warm the lifecycle connection before measuring a second Describe; cold
    // connection/TLS setup and server work cannot inflate lease qualification.
    Description describe(Deadline d = default_deadline()) {
        get<Description>("/v1/describe", d);
        auto started = std::chrono::steady_clock::now();
        auto value = get<Description>("/v1/describe", d);
        // Unverified absolute protocol ceilings: 10 s lease, 2 s input age.
        if (value.Description.MaxGrantLeaseNS > UINT64_C(10000000000) ||
            value.Description.MaxGrantLeaseNS % 1000000 != 0 ||
            value.Description.MaxJogInputAgeNS > UINT64_C(2000000000))
            throw ProtocolError("invalid described control timing");
        // Unverified idle contract bounds: 3 ms through 90000 ms, matching Go.
        if (value.ControlIdleTimeoutNS && (value.ControlIdleTimeoutNS < UINT64_C(3000000) || value.ControlIdleTimeoutNS > UINT64_C(90000000000)))
            throw ProtocolError("invalid described idle timing");
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            round_trip_ = std::chrono::steady_clock::now()-started;
            lease_ceiling_ = std::chrono::nanoseconds(value.Description.MaxGrantLeaseNS);
            if (value.Description.MaxJogInputAgeNS) jog_input_age_ns_ = value.Description.MaxJogInputAgeNS;
            if (value.ControlIdleTimeoutNS) idle_timeout_ = std::chrono::nanoseconds(value.ControlIdleTimeoutNS);
        }
        start_connection_keepalive();
        return value;
    }

    // One binary observation on an independent connection; no grant is needed.
    TelemetryPublication telemetry(std::uint64_t after, Deadline d = default_deadline()) {
        bool warm;
        { std::lock_guard<std::mutex> lock(state_mutex_); warm = warm_ready_; }
        auto wire = telemetry_spare_.exchange("GET", "/v1/telemetry?after="+std::to_string(after), {}, {{"Accept","application/octet-stream"}}, d, {}, {}, !warm);
        if (wire.status == 409) {
            Response response;
            if (!decode(wire.body,response) || response.Schema != ResponseSchema || response.Error.empty()) throw ProtocolError("invalid telemetry refusal");
            throw Rejected(response);
        }
        if (wire.status != 200) throw ProtocolError("unexpected telemetry response");
        return decode_telemetry(wire.body,after);
    }

    // Decode self-framed batches across arbitrary HTTP chunk boundaries. Stops,
    // callback failures and disconnects never silently reset the resume cursor.
    void telemetry_stream(std::uint64_t after, const std::function<void(const TelemetryPublication&)>& callback,
                          const std::function<bool()>& stop, Deadline d = Deadline::max()) {
        if (!callback || !stop) throw std::invalid_argument("telemetry callback and stop required");
        client_detail::Lane lane(reserved_stream(telemetry_spare_));
        std::string pending, incarnation;
        // Measured maximum batch: 312-byte header plus 4096*5392-byte records.
        constexpr std::size_t batch_limit = 22085944;
        auto consume = [&](const std::string& bytes) {
            std::size_t offset = 0;
            while (offset < bytes.size()) {
                if (stop()) throw client_detail::StreamStopped{};
                auto needed = sizeof(gradient::ipc::v1::TelemetryBatchHeaderV1);
                if (pending.size() >= needed) { auto h=telemetry_header(pending); needed += std::size_t(h.record_count)*h.record_bytes; }
                if (needed > batch_limit) throw ProtocolError("telemetry batch too large");
                auto count=std::min(needed-pending.size(),bytes.size()-offset);
                pending.append(bytes,offset,count); offset+=count;
                if (pending.size() < sizeof(gradient::ipc::v1::TelemetryBatchHeaderV1)) continue;
                auto h=telemetry_header(pending);
                if (pending.size() != sizeof(h)+std::size_t(h.record_count)*h.record_bytes) continue;
                auto batch=decode_telemetry(pending,after);
                std::string identity(reinterpret_cast<const char*>(h.adapter_incarnation),sizeof(h.adapter_incarnation));
                identity.append(reinterpret_cast<const char*>(h.daemon_incarnation),sizeof(h.daemon_incarnation));
                if (!incarnation.empty() && incarnation != identity) throw ProtocolError("telemetry incarnation changed");
                incarnation=identity; callback(batch); after=h.last_sequence; pending.clear();
            }
        };
        client_detail::HttpResponse wire;
        try { wire=lane.exchange("GET","/v1/telemetry/stream?after="+std::to_string(after),{},{{"Accept","application/octet-stream"}},d,consume,stop); }
        catch (const client_detail::StreamStopped&) { return; }
        if (wire.status == 409) { Response response; if (!decode(wire.body,response) || response.Schema!=ResponseSchema || response.Error.empty()) throw ProtocolError("invalid telemetry refusal"); throw Rejected(response); }
        if (!stop()) throw TransportError("telemetry stream EOF; resume from last handled cursor");
    }
    ProcessStatus status(Deadline d = default_deadline()) { return get<ProcessStatus>("/v1/status", d); }
    // Validate exact cursors and loss records before consumers advance history.
    // Explicit drop events retain their generated range; no gap is skipped.
    static void validate_event(const Event& event, std::uint64_t after) {
        if (event.Sequence <= after || !event.TimeNS || event.Type.empty()) throw ProtocolError("invalid event cursor");
        if (event.Type == "events_dropped") {
            if (!event.Dropped || event.Dropped->FirstLostSequence != after + 1 || event.Dropped->LastLostSequence != event.Sequence)
                throw ProtocolError("invalid event loss range");
        } else if (event.Sequence != after + 1 || event.Dropped) throw ProtocolError("unreported event loss");
    }
    EventBatch events(std::uint64_t after, Deadline d = default_deadline()) {
        auto batch = get<EventBatch>("/v1/events?after=" + std::to_string(after), d);
        for (const auto& event : batch.Events) {
            validate_event(event, after);
            if (event.AdapterIncarnation != batch.AdapterIncarnation) throw ProtocolError("event incarnation mismatch");
            after = event.Sequence;
        }
        if (batch.NextSequence != after || batch.LatestSequence < after) throw ProtocolError("invalid event batch cursor");
        return batch;
    }
    // Deliver generated events on an independent connection until stop, callback
    // failure or disconnect. Resume explicitly with the last handled sequence;
    // EventCursorLost retains the generated loss event and halts delivery before
    // any later event, requiring explicit status reconciliation by the consumer.
    void events_stream(std::uint64_t after, const std::function<void(const Event&)>& callback,
                       const std::function<bool()>& stop, Deadline d = Deadline::max()) {
        if (!callback || !stop) throw std::invalid_argument("event callback and stop predicate required");
        client_detail::Lane lane(reserved_stream(events_spare_));
        std::string pending, id, kind, body, incarnation;
        // Unverified SSE record ceiling: 65,536 bytes, including field metadata.
        constexpr std::size_t record_limit = 65536;
        std::size_t record_size = 0;
        auto consume = [&](const std::string& bytes) {
            for (char ch : bytes) {
                if (stop()) throw client_detail::StreamStopped{};
                if (++record_size > record_limit) throw ProtocolError("event record too large");
                if (ch != '\n') { pending += ch; continue; }
                if (!pending.empty() && pending.back() == '\r') pending.pop_back();
                auto line = std::move(pending); pending.clear();
                if (line.empty()) {
                    if (!body.empty()) {
                        Event event;
                        if (!decode(body, event) || id != std::to_string(event.Sequence) || kind != event.Type)
                            throw ProtocolError("invalid SSE event");
                        validate_event(event, after);
                        // publisher_overflow is a native-side record and loses no adapter history; it is
                        // delivered like any event, as the Go SDK does. Only events_dropped is cursor loss.
                        if (event.Type == "events_dropped") throw EventCursorLost(event);
                        if (!incarnation.empty() && event.AdapterIncarnation != incarnation) throw ProtocolError("event incarnation changed");
                        incarnation = event.AdapterIncarnation;
                        callback(event);
                        after = event.Sequence;
                    }
                    id.clear(); kind.clear(); body.clear(); record_size = 0;
                } else if (line[0] != ':') {
                    auto colon = line.find(':');
                    if (colon == std::string::npos) continue;
                    auto key = line.substr(0, colon), value = line.substr(colon + 1);
                    if (!value.empty() && value[0] == ' ') value.erase(0, 1);
                    if (key == "id") id = value;
                    else if (key == "event") kind = value;
                    else if (key == "data") { if (!body.empty()) body += '\n'; body += value; }
                }
            }
        };
        client_detail::HttpResponse wire;
        try { wire = lane.exchange("GET", "/v1/events/stream?after=" + std::to_string(after), {}, {{"Accept", "text/event-stream"}}, d, consume, stop); }
        catch (const client_detail::StreamStopped&) { return; }
        if (wire.status == 409) {
            Response response;
            if (!decode(wire.body, response) || response.Schema != ResponseSchema || response.Error.empty()) throw ProtocolError("invalid event refusal");
            throw Rejected(response);
        }
        if (wire.status != 200) throw ProtocolError("unexpected event HTTP response");
        if (!stop()) throw TransportError("event stream EOF; resume from last handled cursor");
    }
    std::uint64_t max_jog_input_age_ns() const { std::lock_guard<std::mutex> lock(state_mutex_); return jog_input_age_ns_; }
    // Describe's complete receipt measures transport setup too, preventing
    // consumers from sizing subsequent requests from an unmeasured link.
    std::chrono::nanoseconds measured_round_trip() const { std::lock_guard<std::mutex> lock(state_mutex_); return round_trip_; }
    // Preflight warming precedes this timestamp; authority age must start at
    // the actual Acquire request, never at the first cold preparation read.
    Deadline acquire_started() const { std::lock_guard<std::mutex> lock(state_mutex_); return acquire_started_; }
    // Custom keepers maintain the same reservations as start_renewal; only the
    // keeper may replace a lost stream after authority has been acquired.
    void maintain_connections(Deadline d) {
        bool warm;
        { std::lock_guard<std::mutex> lock(state_mutex_); warm = warm_ready_; }
        if (warm) warm_connections(d, false);
    }
    // A separate observer reserves only its observation socket before Acquire.
    // Later maintenance skips busy polls and prevents per-poll reconnection.
    void maintain_observation(Deadline d, bool initial = false) {
        try {
            observation_.exchange("GET", initial ? "/v1/describe" : "/v1/jog/ingress", {}, {}, d, {}, {}, true, !initial);
        } catch (const ConnectionFailure&) {
            std::clog << "rt_core_connection_reopened method=GET path=/v1/jog/ingress\n";
            observation_.exchange("GET", "/v1/jog/ingress", {}, {}, d, {}, {}, true, !initial);
        }
        std::lock_guard<std::mutex> lock(state_mutex_); warm_ready_ = true;
    }
    // Derive maintenance from Describe's compiled listener bound so profile
    // changes cannot leave selected reservations idle past the peer timeout.
    std::chrono::nanoseconds connection_keepalive_interval() const {
        std::lock_guard<std::mutex> lock(state_mutex_); return idle_timeout_ / 3;
    }
    // A described/selected client maintains idle reservations until destruction,
    // including between grants. This worker never acquires or renews authority.
    void start_connection_keepalive(std::chrono::nanoseconds interval = std::chrono::nanoseconds(0), bool observation_only = false) {
        if (interval.count() == 0) interval = connection_keepalive_interval();
        if (interval.count() <= 0) throw std::invalid_argument("invalid connection keepalive interval");
        std::lock_guard<std::mutex> lock(keep_mutex_);
        if (keep_thread_.joinable()) return;
        keep_stop_ = false;
        keep_thread_ = std::thread([this, interval, observation_only] {
            std::unique_lock<std::mutex> lock(keep_mutex_);
            while (true) {
                keep_cv_.wait_for(lock, interval, [this] { return keep_stop_ || keep_requested_; });
                if (keep_stop_) break;
                keep_requested_ = false;
                lock.unlock();
                auto d = std::chrono::steady_clock::now() + interval;
                try {
                    if (observation_only) maintain_observation(d);
                    else {
                        warm_connections(d, false);
                        std::lock_guard<std::mutex> state(state_mutex_); warm_ready_ = true;
                    }
                } catch (...) {}
                lock.lock();
            }
        });
    }
    void stop_connection_keepalive() {
        { std::lock_guard<std::mutex> lock(keep_mutex_); keep_stop_ = true; }
        keep_cv_.notify_all();
        if (keep_thread_.joinable()) keep_thread_.join();
    }
    LocalJogClock jog_clock(Deadline d = default_deadline()) { return get<LocalJogClock>("/v1/jog/clock", d); }
    JogObservation jog_status(Deadline d = default_deadline()) { return get<JogObservation>("/v1/jog", d); }
    JogIngressObservation jog_ingress(Deadline d = default_deadline()) { return get<JogIngressObservation>("/v1/jog/ingress", d); }
    // The servo host's maintenance notice as rt-control enforces it, and the grant holder; no session needed.
    MaintenanceObservation maintenance(Deadline d = default_deadline()) { return get<MaintenanceObservation>("/v1/maintenance", d); }

    // Explicitly starts one keeper. The latest result is polled without callbacks
    // so a slow observer cannot stall renewal; an error terminates the keeper.
    void start_renewal(std::chrono::milliseconds interval = std::chrono::milliseconds(0)) {
        std::lock_guard<std::mutex> lifecycle(renewal_lifecycle_);
        auto g = grant();
        // Unverified compatibility lease for an explicit SetFence: 500 ms.
        auto lease = std::chrono::milliseconds(g.LeaseMS > 0 ? g.LeaseMS : 500);
        if (interval.count() == 0) interval = lease/3;
        if (interval.count() <= 0 || g.Session.empty() || interval > lease/3)
            throw std::invalid_argument("renewal requires a fence and cadence at most one third of lease");
        { std::lock_guard<std::mutex> state(state_mutex_);
          if (lease < 3*round_trip_+interval) { Response r; r.Error=ReasonControlSessionStale; throw Rejected(r); } }
        auto budget = lease-interval;
        std::lock_guard<std::mutex> lock(renewal_mutex_);
        if (renewal_thread_.joinable()) throw std::logic_error("stop renewal before restarting");
        renewal_stop_ = false; renewal_update_.reset();
        renewal_thread_ = std::thread([this, interval, budget] {
            std::unique_lock<std::mutex> guard(renewal_mutex_);
            auto next = std::chrono::steady_clock::now()+interval;
            while (!renewal_cv_.wait_until(guard, next, [this] { return renewal_stop_; })) {
                next = std::chrono::steady_clock::now()+interval;
                guard.unlock();
                Renewal update{};
                // The effective lease bounds receipts independently of lifecycle traffic.
                try { update.grant = renew(std::chrono::steady_clock::now() + budget); }
                catch (...) { update.error = std::current_exception(); }
                guard.lock();
                renewal_update_ = update;
                if (update.error) break;
            }
        });
    }
    std::optional<Renewal> take_renewal() {
        std::lock_guard<std::mutex> lock(renewal_mutex_);
        auto result = renewal_update_; renewal_update_.reset(); return result;
    }
    void stop_renewal() {
        std::lock_guard<std::mutex> lifecycle(renewal_lifecycle_);
        { std::lock_guard<std::mutex> lock(renewal_mutex_); renewal_stop_ = true; }
        renewal_cv_.notify_all();
        if (renewal_thread_.joinable()) renewal_thread_.join();
    }
};
} // namespace rosie::rt_control
