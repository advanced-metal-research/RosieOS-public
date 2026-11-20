#include <rosie/rt_control_client.hpp>
#include <future>
#include <iostream>

using namespace rosie::rt_control;
using namespace std::chrono_literals;

// Exercise the Deck SDK against the delayed simulation adapter; native state
// and correlated renewal receipts prevent transport success from hiding expiry.
int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) return 2;
    try {
        RtControlClient client(argv[1]);
        RtControlClient observer(argv[1]);
        const bool custom_keeper = argc == 4;
        // Unverified test watchdog: 30000 ms.
        const auto deadline = std::chrono::steady_clock::now() + 30000ms;
        Binding binding; binding.PairID = "bench"; binding.Revision = 7; binding.MachineSHA256 = argv[2];
        if (custom_keeper) observer.maintain_observation(deadline, true);
        auto g = client.acquire("deck:jitter", binding, deadline);
        if (g.LeaseMS != 3000) throw std::runtime_error("remote lease is not ceiling");
        std::future<void> keeper;
        if (custom_keeper) keeper = std::async(std::launch::async, [&] {
            for (unsigned i = 0; i < 3; ++i) {
                auto renewed = client.renew(deadline);
                if (renewed.Session != g.Session || renewed.Generation != g.Generation) throw std::runtime_error("custom keeper changed grant");
                client.maintain_connections(deadline);
                observer.maintain_observation(deadline);
            }
        });
        else client.start_renewal();
        auto arm = std::async(std::launch::async, [&] { client.enable(511, deadline); client.arm(deadline); });
        std::future<ProcessStatus> polls[3];
        for (auto& poll : polls) poll = std::async(std::launch::async, [&] { return (custom_keeper ? observer : client).status(deadline); });
        arm.get();
        for (auto& poll : polls) poll.get();
        unsigned renewals = 0;
        if (custom_keeper) { keeper.get(); renewals = 3; }
        while (renewals < 3) {
            if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("renewal watchdog");
            if (auto r = client.take_renewal()) {
                if (r->error) std::rethrow_exception(r->error);
                if (r->grant.Session != g.Session || r->grant.Generation != g.Generation) throw std::runtime_error("grant changed");
                ++renewals;
            }
            // Unverified observation cadence: 1 ms; only renewal receipts pass.
            std::this_thread::sleep_for(1ms);
        }
        const auto state = client.status(deadline);
        if (!state.Grant || state.Grant->GrantActive != 1 || state.Core.Armed != 1) throw std::runtime_error("native grant expired");
        client.stop_renewal();
        client.telemetry(0, deadline);
        client.telemetry(0, deadline);
        bool received = false;
        client.events_stream(0, [&](const Event&) { received = true; }, [&] { return received; }, deadline);
        if (!received) throw std::runtime_error("warm event stream returned no event");
        client.stop(deadline); client.release(deadline);
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
