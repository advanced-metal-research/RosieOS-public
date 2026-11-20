#pragma once
#include "rt_control_client.hpp"
#include "protocol_generated.hpp"
#include <ctime>

namespace rosie::rt_control {
inline std::uint64_t host_monotonic_ns() {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) throw TransportError("host monotonic clock unavailable");
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

// Fixed grant and jog generations prevent late packets from adopting a Stop
// fence. Update sends one fresh datagram or returns false; it never queues input.
// Destruction closes the producer only. Call end() or Stop to relinquish motion.
class RtJogProducer {
    RtControlClient& client_;
    Fence fence_;
    rosie::protocol::LocalJogUpdate frame_{};
    int fd_ = -1;
    std::mutex mutex_;
    template<std::size_t N> static void unhex(const std::string& text, std::array<std::uint8_t, N>& out) {
        if (text.size() != N * 2) throw std::invalid_argument("invalid jog identity length");
        for (std::size_t i = 0; i < N; ++i) {
            unsigned value{};
            auto begin = text.data() + i * 2;
            auto result = std::from_chars(begin, begin + 2, value, 16);
            if (result.ec != std::errc{} || result.ptr != begin + 2) throw std::invalid_argument("invalid jog identity");
            out[i] = static_cast<std::uint8_t>(value);
        }
    }
    // Unverified absolute lifetime bound: 2 s; native enforces the compiled cell ceiling.
    static void validate_deadline(std::uint64_t origin, std::uint64_t deadline) {
        if (!origin || deadline <= origin || deadline - origin > UINT64_C(2000000000))
            throw std::invalid_argument("invalid jog deadline");
    }
public:
    RtJogProducer(RtControlClient& client, const std::string& jog_socket, std::uint32_t mask,
                  std::uint64_t deadline_ns, Deadline receipt_deadline = default_deadline())
        : RtJogProducer(client, jog_socket, mask, host_monotonic_ns(), deadline_ns, receipt_deadline) {}
    // Begin shares the first input's capture origin; native admission rejects
    // updates older than Begin, so send time would invalidate delayed input.
    RtJogProducer(RtControlClient& client, const std::string& jog_socket, std::uint32_t mask,
                  std::uint64_t captured_ns, std::uint64_t deadline_ns, Deadline receipt_deadline = default_deadline())
        : client_(client), fence_(client.fence()) {
        auto clock = client.jog_clock(receipt_deadline);
        if (clock.Domain != "CLOCK_MONOTONIC" || clock.MappingGeneration != 1)
            throw ProtocolError("unqualified local jog clock");
        unhex(fence_.Session, frame_.session_id);
        unhex(clock.Incarnation, frame_.clock_incarnation);
        if (!fence_.Generation || !mask || (mask >> rosie::protocol::max_axes)) throw std::invalid_argument("invalid jog fence or mask");
        auto address = client_detail::unix_address(jog_socket);
        fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
        if (fd_ < 0) throw TransportError(std::strerror(errno));
        try {
            if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) throw TransportError(std::strerror(errno));
            const auto now = host_monotonic_ns();
            validate_deadline(captured_ns, deadline_ns);
            if (captured_ns > now || deadline_ns <= now)
                throw std::invalid_argument("stale or future jog input");
            Request r; r.Operation = "begin_jog"; r.Fence = fence_; r.AxisMask = mask;
            r.ClockIncarnation = clock.Incarnation; r.SourceOriginHostNS = captured_ns; r.DeadlineHostNS = deadline_ns;
            frame_.jog_generation = client.command(r, receipt_deadline).Handle;
            if (!frame_.jog_generation) throw ProtocolError("missing jog generation");
            frame_.grant_generation = fence_.Generation; frame_.axis_mask = mask;
        } catch (...) { ::close(fd_); fd_ = -1; throw; }
    }
    ~RtJogProducer() { if (fd_ >= 0) ::close(fd_); }
    RtJogProducer(const RtJogProducer&) = delete;
    RtJogProducer& operator=(const RtJogProducer&) = delete;
    bool update(const std::vector<double>& velocity, std::uint64_t deadline_ns) {
        return update(velocity, host_monotonic_ns(), deadline_ns);
    }
    // Preserve input capture on the wire; stale or future capture times cannot
    // gain a fresh lifetime merely because resolution or publication stalled.
    bool update(const std::vector<double>& velocity, std::uint64_t captured_ns, std::uint64_t deadline_ns) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return false;
        if (fd_ < 0) throw TransportError("jog producer closed");
        if (velocity.size() > frame_.velocity.size() || (frame_.axis_mask >> velocity.size()))
            throw std::invalid_argument("invalid jog vector length");
        for (std::size_t i = 0; i < velocity.size(); ++i)
            if (!std::isfinite(velocity[i]) || (!(frame_.axis_mask & (1U << i)) && velocity[i] != 0))
                throw std::invalid_argument("invalid jog velocity");
        const auto now = host_monotonic_ns();
        validate_deadline(captured_ns, deadline_ns);
        if (captured_ns > now || deadline_ns <= now)
            throw std::invalid_argument("stale or future jog input");
        frame_.origin_host_ns = captured_ns;
        if (frame_.source_sequence == UINT64_MAX) throw std::overflow_error("jog sequence exhausted");
        ++frame_.source_sequence;
        frame_.deadline_host_ns = deadline_ns;
        frame_.velocity.fill(0);
        std::copy(velocity.begin(), velocity.end(), frame_.velocity.begin());
        const auto bytes = rosie::protocol::encode_local_jog_update(frame_);
        auto n = ::send(fd_, bytes.data(), bytes.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == EINTR)) return false;
        if (n != static_cast<ssize_t>(bytes.size())) throw TransportError("jog datagram send failed");
        return true;
    }
    void end(Deadline d = default_deadline()) {
        { std::lock_guard<std::mutex> lock(mutex_); if (fd_ < 0) return; ::close(fd_); fd_ = -1; }
        Request r; r.Operation = "end_jog"; r.Fence = fence_; r.JogGeneration = frame_.jog_generation;
        client_.command(r, d);
    }
};
} // namespace rosie::rt_control
