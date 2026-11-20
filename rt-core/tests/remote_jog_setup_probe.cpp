#include "rosie/rt_jog_remote_producer.hpp"
#include <cassert>
#include <iostream>

// The Go fixture supplies verified TLS and observes the native first input;
// waiting for its acknowledgement prevents exit cleanup from hiding admission.
int main(int argc, char** argv) {
    assert(argc == 5);
    using namespace rosie::rt_control;
    try {
        RtControlClient client(StreamFactory([&](Deadline d) {
            return std::make_unique<client_detail::UnixStream>(argv[1], d);
        }));
        client.set_fence({argv[3], std::stoull(argv[4])});
        client.describe();
        RtJogRemoteProducer producer(client, argv[2], 1, client.max_jog_input_age_ns());
        const auto capture = host_monotonic_ns();
        assert(producer.update({0.01}, capture));
        std::cout << "sent " << capture << std::endl;
        assert(std::cin.get() == 'e');
        producer.end();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
