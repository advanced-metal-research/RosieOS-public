#pragma once
#include "rosie/rt_jog_remote_producer.hpp"

namespace rosie::rt_control::remote_jog_detail {
// Conformance-only wire injection preserves the generated codec while bypassing
// the producer's sequence/fence checks. Applications must never include this file.
struct TestAccess {
    static void frame(RtJogRemoteProducer& producer, std::uint64_t sequence,
                      std::uint64_t source, bool wrong_generation = false) {
        std::lock_guard<std::mutex> lock(producer.write_mutex_);
        auto frame = producer.frame_;
        frame.source_sequence = sequence;
        frame.origin_host_ns = source;
        frame.deadline_host_ns = source + producer.duration_;
        if (wrong_generation) ++frame.jog_generation;
        auto bytes = rosie::protocol::encode_local_jog_update(frame);
        producer.send(2, std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()), default_deadline());
    }
    // RFC 6455 permits unsolicited pong (opcode 10). The receiver consumes it
    // without refreshing jog input or replying, isolating native input expiry.
    static void keepalive(RtJogRemoteProducer& producer) {
        std::lock_guard<std::mutex> lock(producer.write_mutex_);
        producer.send(10, {}, default_deadline());
    }
    // A malformed calibration must receive a refusal even after native closure;
    // its reply proves that the receiver has not taken its idle EndJog path.
    static void probe(RtJogRemoteProducer& producer) {
        std::lock_guard<std::mutex> lock(producer.write_mutex_);
        producer.send(1, "{}", default_deadline());
    }
};
} // namespace rosie::rt_control::remote_jog_detail
