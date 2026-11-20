#include "rosie/rt_control_client.hpp"
#include <fstream>
#include <iostream>
#include <iterator>

using namespace rosie::rt_control;

// Executes only an explicitly supplied program under an acquired public grant.
// Any uncertain outcome triggers best-effort Stop/release and a nonzero exit.
int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: rt-execute CONTROL_SOCKET PAIR_ID PAIR_REVISION CONFIGURATION_SHA256 PROGRAM.rdt\n";
        return 3;
    }
    std::unique_ptr<RtControlClient> client;
    auto cleanup = [&] {
        if (client && !client->fence().Session.empty()) {
            client->stop_renewal();
            try { client->stop(); } catch (...) {}
            try { client->release(); } catch (...) {}
        }
    };
    try {
        Binding binding; binding.PairID = argv[2]; binding.ConfigurationSHA256 = argv[4];
        std::string revision = argv[3];
        auto parsed = std::from_chars(revision.data(), revision.data() + revision.size(), binding.Revision);
        if (parsed.ec != std::errc{} || parsed.ptr != revision.data() + revision.size()) throw std::invalid_argument("invalid pair revision");
        std::ifstream input(argv[5], std::ios::binary);
        if (!input) throw TransportError("cannot open .rdt file");
        std::string bytes{std::istreambuf_iterator<char>(input), {}};
        if (input.bad()) throw TransportError("cannot read .rdt file");
        client = std::make_unique<RtControlClient>(argv[1]);
        client->acquire("motion-server", binding);
        // Unverified example renewal cadence: 50 ms, below the public 500 ms lease.
        client->start_renewal(std::chrono::milliseconds(50));
        const auto description = client->describe();
        const auto count = description.Description.Axes.size();
        if (!count || count > 16) throw ProtocolError("invalid axis count");
        const auto mask = (1U << count) - 1;
        client->enable(mask);
        client->arm();
        auto check_renewal = [&] {
            auto update = client->take_renewal();
            if (update && update->error) std::rethrow_exception(update->error);
            if (client->grant().Stopping) throw TransportError("grant is stopping");
        };
        // Unverified readiness receipt budget: 5,000 ms; success requires observed drive status.
        auto ready_deadline = default_deadline();
        for (;;) {
            check_renewal();
            auto state = client->status(ready_deadline);
            bool ready = state.Core.Armed == 1 && state.Core.AxisEnableMask == mask && state.Core.Axes.size() == count;
            for (const auto& axis : state.Core.Axes) ready = ready && (axis.Statusword & 0x6f) == 0x27;
            if (state.Core.SafetyFaultMask) throw TransportError("runtime fault during enable");
            if (ready) break;
            if (std::chrono::steady_clock::now() >= ready_deadline) throw TransportError("drive readiness deadline");
            // Unverified status polling cadence: 5 ms; readiness is checked above.
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        auto program = client->prepare_program(bytes);
        auto before = client->status();
        client->start_program(program.Identity);
        // Unverified execution observation ceiling: 3,600,000 ms; no inferred completion.
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3600000);
        for (;;) {
            check_renewal();
            auto state = client->status();
            if (state.Core.SafetyFaultMask || !state.Execution.Error.empty() || state.Execution.State == "cancelled" ||
                state.Execution.State == "released" || state.Execution.State == "discarded" || state.Execution.State == "faulted")
                throw TransportError("execution failed: " + state.Execution.Error);
            if (state.Execution.State == "completed" && state.Execution.Generation == before.Execution.Generation + 1) {
                std::cout << encode(state) << '\n';
                break;
            }
            if (std::chrono::steady_clock::now() >= deadline) throw TransportError("execution observation deadline");
            // Unverified status polling cadence: 5 ms; only observed completion succeeds.
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        client->release();
        return 0;
    } catch (const Rejected& e) {
        std::cerr << e.reason << '\n'; cleanup(); return 2;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; cleanup(); return 3;
    }
}
