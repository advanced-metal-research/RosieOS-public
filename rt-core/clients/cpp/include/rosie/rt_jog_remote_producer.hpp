#pragma once
#include "rt_jog_producer.hpp"

namespace rosie::rt_control {

// Wire labels remain available even when unknown; native codes use the generated
// catalogue. A transport failure is never represented as successful admission.
struct JogRejection : Rejected {
    std::uint64_t sequence;
    std::uint32_t native_reason;
    JogRejection(const std::string& label, std::uint64_t seq, std::uint32_t native = 0)
        : Rejected(response(label)), sequence(seq), native_reason(native) {
        const std::string prefix = "jog_native_rejected_";
        if (label.compare(0, prefix.size(), prefix) == 0) {
            auto begin = label.data() + prefix.size();
            auto result = std::from_chars(begin, label.data() + label.size(), native_reason);
            if (result.ec != std::errc{} || result.ptr != label.data() + label.size()) native_reason = 0;
        }
    }
private:
    static Response response(const std::string& label) { Response r; r.Error = label; return r; }
};

namespace remote_jog_detail {
struct TestAccess; // Defined only by the conformance test support, never public API.
template<std::size_t N> inline void unhex(const std::string& text, std::array<std::uint8_t, N>& out) {
    if (text.size() != N * 2) throw std::invalid_argument("invalid jog identity length");
    for (std::size_t i = 0; i < N; ++i) {
        unsigned value{};
        auto p = text.data() + 2*i;
        auto r = std::from_chars(p, p+2, value, 16);
        if (r.ec != std::errc{} || r.ptr != p+2) throw std::invalid_argument("invalid jog identity");
        out[i] = static_cast<std::uint8_t>(value);
    }
}
inline std::string base64(const std::uint8_t* bytes, std::size_t size) {
    // RFC 4648 alphabet; measured wire encoding, not an admission parameter.
    constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (std::size_t i = 0; i < size; i += 3) {
        auto n = std::uint32_t(bytes[i]) << 16;
        if (i+1 < size) n |= std::uint32_t(bytes[i+1]) << 8;
        if (i+2 < size) n |= bytes[i+2];
        out += alphabet[n >> 18]; out += alphabet[(n >> 12) & 63];
        out += i+1 < size ? alphabet[(n >> 6) & 63] : '=';
        out += i+2 < size ? alphabet[n & 63] : '=';
    }
    return out;
}
// SHA-1 is used only for the RFC 6455 upgrade accept, never TLS authentication.
// The existing HttpStream factory owns certificate verification and credentials.
inline std::string accept(const std::string& key) {
    auto input = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    auto bits = std::uint64_t(input.size()) * 8;
    input += char(128);
    while (input.size() % 64 != 56) input += char(0);
    for (int shift = 56; shift >= 0; shift -= 8) input += char(bits >> shift);
    std::array<std::uint32_t,5> h{0x67452301,0xefcdab89,0x98badcfe,0x10325476,0xc3d2e1f0};
    auto rot = [](std::uint32_t n, unsigned k) { return (n << k) | (n >> (32-k)); };
    for (std::size_t block = 0; block < input.size(); block += 64) {
        std::array<std::uint32_t,80> w{};
        for (unsigned i=0; i<16; ++i) for (unsigned j=0; j<4; ++j) w[i] = (w[i]<<8) | static_cast<unsigned char>(input[block+4*i+j]);
        for (unsigned i=16; i<80; ++i) w[i] = rot(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);
        auto a=h[0], b=h[1], c=h[2], d=h[3], e=h[4];
        for (unsigned i=0; i<80; ++i) {
            auto f = i<20 ? (b&c)|(~b&d) : i<40 ? b^c^d : i<60 ? (b&c)|(b&d)|(c&d) : b^c^d;
            auto k = i<20 ? 0x5a827999U : i<40 ? 0x6ed9eba1U : i<60 ? 0x8f1bbcdcU : 0xca62c1d6U;
            auto next = rot(a,5)+f+e+k+w[i]; e=d; d=c; c=rot(b,30); b=a; a=next;
        }
        h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e;
    }
    std::array<std::uint8_t,20> digest{};
    for (unsigned i=0; i<20; ++i) digest[i] = static_cast<std::uint8_t>(h[i/4] >> (24-8*(i%4)));
    return base64(digest.data(), digest.size());
}
inline bool token(const std::string& value, const std::string& wanted) {
    std::istringstream in(client_detail::lower(value)); std::string part;
    while (std::getline(in, part, ',')) if (client_detail::trim(part) == wanted) return true;
    return false;
}
} // namespace remote_jog_detail

// Dedicated authenticated WSS producer. source_now and update(source_time) use
// the same monotonic clock; each session generates a fresh clock incarnation.
// It never adopts a Stop fence or queues/replays input after a failed write.
// Destruction closes the stream; end() requests explicit control-plane closure.
// The receiver closes WSS and calls EndJog after 250 ms without a complete frame.
// No automatic keepalive is sent. Join all calls before destroying the producer.
class RtJogRemoteProducer {
    friend struct remote_jog_detail::TestAccess;
    RtControlClient& client_;
    Fence fence_;
    rosie::protocol::LocalJogUpdate frame_{};
    std::uint64_t duration_;
    std::shared_ptr<HttpStream> stream_;
    std::mutex write_mutex_, read_mutex_, observe_mutex_;
    bool ended_ = false;
    // Unverified transport bounds: 4096-byte messages, 8192-byte upgrade headers.
    static constexpr std::size_t payload_limit = 4096, header_limit = 8192;
    // Retire the stream atomically so failed I/O cannot invalidate another call's
    // storage. In-flight I/O retains it only through that call's absolute deadline.
    void close_stream() { std::atomic_store(&stream_, std::shared_ptr<HttpStream>{}); }
    void write(const std::string& bytes, Deadline d) {
        auto stream = std::atomic_load(&stream_);
        if (!stream) throw TransportError("remote jog stream closed");
        for (std::size_t done = 0; done < bytes.size();) {
            auto n = stream->write(bytes.data()+done, bytes.size()-done, d);
            if (!n || n > bytes.size()-done) throw TransportError("invalid remote jog write");
            done += n;
        }
    }
    std::string read(std::size_t size, Deadline d) {
        auto stream = std::atomic_load(&stream_);
        if (!stream) throw TransportError("remote jog stream closed");
        std::string bytes(size, '\0');
        for (std::size_t done = 0; done < size;) {
            auto n = stream->read(bytes.data()+done, size-done, d);
            if (!n || n > size-done) throw TransportError("remote jog EOF");
            done += n;
        }
        return bytes;
    }
    void send(std::uint8_t opcode, const std::string& payload, Deadline d) {
        if (payload.size() > payload_limit) throw ProtocolError("remote jog payload too large");
        std::array<std::uint8_t,16> entropy{};
        remote_jog_detail::unhex(new_request_id(), entropy);
        std::string packet; packet += char(128|opcode);
        if (payload.size() < 126) packet += char(128|payload.size());
        else { packet += char(128|126); packet += char(payload.size()>>8); packet += char(payload.size()); }
        for (unsigned i=0; i<4; ++i) packet += char(entropy[i]);
        for (std::size_t i=0; i<payload.size(); ++i) packet += char(static_cast<unsigned char>(payload[i])^entropy[i%4]);
        try { write(packet, d); } catch (...) { close_stream(); throw; }
    }
    Json message(Deadline d) {
        auto header = read(2,d);
        if (static_cast<unsigned char>(header[0]) != 129 || (static_cast<unsigned char>(header[1])&128)) throw ProtocolError("invalid remote jog frame");
        std::size_t size = static_cast<unsigned char>(header[1]);
        if (size == 127) throw ProtocolError("oversize remote jog frame");
        if (size == 126) {
            auto length=read(2,d); size=(static_cast<unsigned char>(length[0])<<8)|static_cast<unsigned char>(length[1]);
            if (size<126) throw ProtocolError("noncanonical remote jog frame");
        }
        if (size>payload_limit) throw ProtocolError("oversize remote jog frame");
        Json json;
        try { json=Parser(read(size,d)).parse(); require_object(json); }
        catch (const TransportError&) { throw; }
        catch (const std::exception& e) { throw ProtocolError(e.what()); }
        std::string type, reason; std::uint64_t seq{};
        field(json,"type",type,true); field(json,"seq",seq,true);
        if (type=="rejected") { field(json,"reason",reason,true); if (reason.empty()) throw ProtocolError("empty jog refusal"); throw JogRejection(reason,seq); }
        return json;
    }
    void upgrade(const std::string& host, Deadline d) {
        if (host.empty() || host.find_first_of("\r\n /\t") != std::string::npos || host.find('\0') != std::string::npos) throw std::invalid_argument("invalid remote host");
        std::array<std::uint8_t,16> nonce{}; remote_jog_detail::unhex(new_request_id(),nonce);
        auto key=remote_jog_detail::base64(nonce.data(),nonce.size());
        auto path="/v1/jog?session="+fence_.Session+"&generation="+std::to_string(fence_.Generation)+"&jog_generation="+std::to_string(frame_.jog_generation);
        write("GET "+path+" HTTP/1.1\r\nHost: "+host+"\r\nConnection: Upgrade\r\nUpgrade: websocket\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: "+key+"\r\n\r\n",d);
        std::string headers;
        while (headers.size()<4 || headers.substr(headers.size()-4)!="\r\n\r\n") {
            if (headers.size()>=header_limit) throw ProtocolError("oversize upgrade headers");
            headers+=read(1,d); // Leave every post-upgrade byte for the WSS parser.
        }
        std::istringstream in(headers); std::string line;
        std::getline(in,line);
        if (line.compare(0,13,"HTTP/1.1 101 ")!=0) throw ProtocolError("remote jog upgrade rejected: "+line);
        std::map<std::string,std::string> fields;
        while (std::getline(in,line) && line!="\r") {
            if (line.empty() || line.back()!='\r') throw ProtocolError("invalid upgrade header");
            line.pop_back(); auto colon=line.find(':');
            if (!colon || colon==std::string::npos || !fields.emplace(client_detail::lower(line.substr(0,colon)),client_detail::trim(line.substr(colon+1))).second) throw ProtocolError("duplicate or invalid upgrade header");
        }
        if (client_detail::lower(fields["upgrade"])!="websocket" || !remote_jog_detail::token(fields["connection"],"upgrade") || fields["sec-websocket-accept"]!=remote_jog_detail::accept(key)) throw ProtocolError("invalid websocket accept");
    }
public:
    RtJogRemoteProducer(RtControlClient& client, const std::string& host, std::uint32_t mask,
                        std::uint64_t input_duration_ns, std::function<std::uint64_t()> source_now = host_monotonic_ns,
                        Deadline d = default_deadline())
        : client_(client), fence_(client.fence()), duration_(input_duration_ns) {
        if (!source_now || !duration_ || duration_>client.max_jog_input_age_ns() || !mask || (mask>>rosie::protocol::max_axes) || !fence_.Generation) throw std::invalid_argument("invalid remote jog options");
        remote_jog_detail::unhex(fence_.Session,frame_.session_id);
        auto incarnation=new_request_id(); remote_jog_detail::unhex(incarnation,frame_.clock_incarnation);
        stream_=client.open_remote_jog_stream(d);
        frame_.grant_generation=fence_.Generation; frame_.axis_mask=mask;
        try {
            upgrade(host,d);
            std::uint64_t tx=0;
            for (std::uint64_t seq=1; seq<=2; ++seq) {
                auto source=source_now(); // Sample two follows receipt one.
                if (!source) throw std::invalid_argument("zero source clock");
                Json json=Json::object_value();
                json.object={{"type",to_json(std::string("calibrate"))},{"source_incarnation",to_json(incarnation)},{"source_ns",to_json(source)},{"seq",to_json(seq)},{"host_tx_ns",to_json(tx)}};
                send(1,dump(json),d);
                auto reply=message(d); std::string type; std::uint64_t echoed=0, sample=0, rx=0;
                field(reply,"type",type,true); field(reply,"seq",echoed,true); field(reply,"source_ns",sample,true); field(reply,"host_tx_ns",tx,true); field(reply,"host_rx_ns",rx,true);
                if (type!="calibrated" || echoed!=seq || sample!=source || !tx || rx>tx) throw ProtocolError("invalid calibration reply");
            }
            // Reserve native mode only after setup. Input capture timestamps are
            // never rewritten to compensate for transport or calibration.
            Json begin=Json::object_value();
            const auto source=source_now();
            if (!source) throw std::invalid_argument("zero source clock");
            begin.object={{"type",to_json(std::string("begin"))},{"seq",to_json(std::uint64_t(3))},
                          {"host_tx_ns",to_json(tx)},{"axis_mask",to_json(mask)},{"source_ns",to_json(source)}};
            send(1,dump(begin),d);
            auto reply=message(d); std::string type; std::uint64_t seq=0;
            field(reply,"type",type,true); field(reply,"seq",seq,true);
            field(reply,"jog_generation",frame_.jog_generation,true);
            if (type!="begun" || seq!=3 || !frame_.jog_generation) throw ProtocolError("invalid jog begin reply");
        } catch (...) {
            auto failure=std::current_exception();
            if (frame_.jog_generation) { try { end(d); } catch (...) {} }
            close_stream();
            std::rethrow_exception(failure);
        }
    }
    RtJogRemoteProducer(const RtJogRemoteProducer&)=delete;
    RtJogRemoteProducer& operator=(const RtJogRemoteProducer&)=delete;

    // The native Begin receipt fixes this identity; querying it needs no HTTP
    // round trip that could consume the first-input allowance.
    std::uint64_t generation() const { return frame_.jog_generation; }

    // One bounded write per sample; false means contention and no bytes sent.
    // The source deadline never moves with arrival time or authority renewal.
    bool update(const std::vector<double>& velocities, std::uint64_t source_time) {
        std::unique_lock<std::mutex> lock(write_mutex_,std::try_to_lock);
        if (!lock.owns_lock()) return false;
        if (ended_) throw TransportError("remote jog ended");
        auto current=client_.fence();
        if (current.Session!=fence_.Session || current.Generation!=fence_.Generation) throw JogRejection(ReasonControlSessionStale,frame_.source_sequence);
        if (!source_time || source_time>UINT64_MAX-duration_ || frame_.source_sequence==UINT64_MAX) throw std::invalid_argument("invalid source time or exhausted sequence");
        if (velocities.size()>frame_.velocity.size() || (frame_.axis_mask>>velocities.size())) throw std::invalid_argument("invalid jog vector");
        for (std::size_t i=0; i<velocities.size(); ++i) if (!std::isfinite(velocities[i]) || (!(frame_.axis_mask&(1U<<i)) && velocities[i]!=0)) throw std::invalid_argument("invalid jog velocity");
        frame_.velocity.fill(0); std::copy(velocities.begin(),velocities.end(),frame_.velocity.begin());
        frame_.origin_host_ns=source_time; frame_.deadline_host_ns=source_time+duration_; ++frame_.source_sequence;
        auto bytes=rosie::protocol::encode_local_jog_update(frame_);
        // Unverified network write bound: 20 ms; failures close without replay.
        send(2,std::string(reinterpret_cast<const char*>(bytes.data()),bytes.size()),std::chrono::steady_clock::now()+std::chrono::milliseconds(20));
        return true;
    }
    // Blocking refusal observation is explicit; successful updates have no reply.
    // A timeout discards the stream so partial response bytes cannot be reused.
    void rejection(Deadline d=default_deadline()) {
        std::lock_guard<std::mutex> lock(read_mutex_);
        try { message(d); throw ProtocolError("expected jog refusal"); }
        catch (const JogRejection&) { throw; }
        catch (...) { close_stream(); throw; }
    }
    // Preserve the receiver's state reason. Only native Expired establishes
    // expiry; Closed after a deadline can also follow End, Stop or idle cleanup.
    JogObservation observe(Deadline d=default_deadline()) {
        std::lock_guard<std::mutex> lock(observe_mutex_);
        auto status=client_.status(d);
        if (!status.Jog) throw ProtocolError("missing jog observation");
        auto o=*status.Jog;
        if (o.JogGeneration!=frame_.jog_generation || o.AppGrantGeneration!=fence_.Generation) throw JogRejection(ReasonJogSessionStale,o.ObservedSourceSequence);
        if (o.StateReason==rosie::protocol::jog_reason_expired) {
            JogRejection e(ReasonJogInputDeadlineExpired,o.ObservedSourceSequence,o.StateReason); e.native_jog_result=o; throw e;
        }
        return o;
    }
    // Send EndJog with the fixed fence before closing; close-first races the
    // receiver's own cleanup. A lost receipt still requires Stop/expiry.
    void end(Deadline d=default_deadline()) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (ended_) return;
        ended_=true;
        Request r; r.Operation="end_jog"; r.Fence=fence_; r.JogGeneration=frame_.jog_generation;
        try { client_.command(r,d); } catch (...) { close_stream(); throw; }
        close_stream();
    }
};
} // namespace rosie::rt_control
