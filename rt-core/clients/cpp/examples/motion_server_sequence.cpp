#include "rosie/rt_control_client.hpp"

#include <fstream>
#include <iostream>
#include <iterator>
#include <set>

using namespace rosie::rt_control;

// Explicit checks remain active under NDEBUG and route failures through cleanup.
static void require(bool condition) {
    if (!condition) throw ProtocolError("motion-server profile check failed" );
}

// Unverified observation ceiling: 10000 ms, independent of the motion duration.
static Deadline deadline() { return std::chrono::steady_clock::now() + std::chrono::milliseconds(10000); }

// Retry only typed contention at the same cursor within the caller's deadline;
// a busy ring must not abort conformance or conceal another rejection reason.
static TelemetryPublication telemetry_sample(RtControlClient& client, uint64_t after, Deadline end) {
    for (;;) {
        try { return client.telemetry(after, end); }
        catch (const Rejected& failure) {
            if (failure.reason != ReasonTelemetryBusy) throw;
            if (std::chrono::steady_clock::now() >= end) throw TransportError("telemetry observation deadline");
            std::this_thread::yield();
        }
    }
}

// Reads immutable producer bytes; an unreadable plan cannot become an empty upload.
static std::string read_program(const std::string& directory, const std::string& name) {
    std::ifstream input(directory + "/" + name + ".rdt", std::ios::binary);
    if (!input) throw TransportError("cannot read profile program");
    std::string result{std::istreambuf_iterator<char>(input), {}};
    if (input.bad()) throw TransportError("profile program read failed");
    return result;
}

// Requires a native command receipt; a merely parseable envelope is insufficient.
static void receipt(const Response& response, Operation operation) {
    require(response.Schema == ResponseSchema && response.Operation == to_json(operation).text);
    require(response.Sequence != 0 && response.Error.empty());
}

// Matches the generated reason label and preserves native details, preventing
// transport errors from being counted as deliberate admission refusals.
template<class F> static Rejected rejected(F action, const char* reason) {
    try { action(); }
    catch (const Rejected& failure) {
        require(failure.reason.substr(0, failure.reason.find(':')) == reason);
        return failure;
    }
    throw ProtocolError("expected typed rejection");
}

// Public operations only. Failed checks and uncertain results trigger Stop and
// release, preventing a failed profile run from intentionally retaining motion.
int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: motion-server-sequence SOCKET CONFIG_SHA256 FIXTURES MACHINE_SHA256 DEPLOYMENT_SHA256\n";
        return 2;
    }
    RtControlClient c(argv[1]);
    try {
        const auto description = c.describe(deadline());
        require(description.ContractVersion == ContractVersion && description.CapabilitiesDigest == CapabilitiesDigest);
        const auto& native = description.Description;
        require(native.ConfigurationSHA256 == argv[2] && native.MachineSHA256 == argv[4] && native.DeploymentSHA256 == argv[5]);
        require(native.MachineSHA256.size() == 64 && native.DeploymentSHA256.size() == 64);
        std::set<std::string> required{"describe", "acquire", "renew", "enable", "arm", "home", "restore_anchor", "recovery_status", "prepare_program", "start_program", "status", "subscribe_events", "stop", "release"};
        for (const auto& capability : description.Capabilities) {
            if (required.count(capability.Name)) { require(capability.State == "implemented"); required.erase(capability.Name); }
        }
        require(required.empty() && native.Axes.size() == 9 && native.MaxTrajectoryPoints == 250000 && native.ResidentPlans == 3);
        for (const auto& axis : native.Axes) require(axis.NativeHome && axis.RequireHome && axis.MaxAcceleration && *axis.MaxAcceleration == 1 && axis.MaxVelocity == .1);
        Binding binding; binding.PairID = "bench"; binding.Revision = 7; binding.MachineSHA256 = native.MachineSHA256;
        const auto grant = c.acquire("motion-server-profile", binding, deadline());
        require(!grant.Session.empty() && grant.Generation == 1 && grant.LeaseMS == 500 && !grant.Stopping);
        require(grant.Binding.MachineSHA256 == binding.MachineSHA256);
        const auto renewed = c.renew(deadline());
        require(renewed.Session == grant.Session && renewed.Generation == 1 && renewed.DeadlineHostNS > grant.DeadlineHostNS && !renewed.Stopping);
        // Unverified cadence: 100 ms = one fifth of the declared 500 ms lease.
        c.start_renewal(std::chrono::milliseconds(100));
        auto check_renewal = [&] {
            const auto update = c.take_renewal();
            if (update && update->error) std::rethrow_exception(update->error);
            if (c.grant().Stopping) throw ProtocolError("renewal stopping; motion forbidden");
        };
        auto observe = [&](auto predicate) {
            const auto end = deadline();
            for (;;) {
                check_renewal();
                const auto state = c.status(end);
                require(state.Core.SafetyFaultMask == 0);
                if (predicate(state)) return state;
                if (std::chrono::steady_clock::now() >= end) throw TransportError("observation deadline");
                // Unverified poll cadence: 5 ms; success requires the observation above.
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        };
        const auto unhomed = rejected([&] { c.enable(511, deadline()); }, ReasonNativeRejected);
        require(unhomed.native_result && unhomed.native_result->Reason == 2);
        receipt(c.home(511, deadline()), Operation::Home);
        observe([](const auto& s) { return s.Core.HomeValidMask == 511 && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 && s.Core.CommissioningPhase == 0; });
        Request restore; restore.Operation = to_json(Operation::RestoreAnchor).text; restore.AxisMask = 511;
        rejected([&] { c.command(restore, deadline()); }, ReasonAnchorMissing);
        const auto recovery = c.recovery_status(deadline());
        require(recovery.HomeValidMask == 511 && recovery.Faults.empty());
        const auto program = c.prepare_program(read_program(argv[3], "valid"), deadline());
        // The acceleration fixture is 5 rad/s^2, exceeding the declared 1 rad/s^2.
        const auto acceleration = rejected([&] { c.prepare_program(read_program(argv[3], "acceleration"), deadline()); }, ReasonNativeLimitExceeded);
        require(!acceleration.native_result && acceleration.reason.find("segment=0 sample=1 axis=J1") != std::string::npos);
        rejected([&] { c.prepare_program(read_program(argv[3], "velocity"), deadline()); }, ReasonNativeLimitExceeded);
        const auto retained = c.status(deadline()).Execution;
        require(retained.State == "prepared" && retained.Handles.size() == 1 && retained.Handles[0].State == "prepared" && retained.Generation == 0);
        require(program.Samples == 11 && program.NormalisedSamples == 11 && program.Segments == 1 && program.AxisMask == 511 && !program.RequiresProcessIO);
        require(program.Identity.ManifestRevision == 7 && !program.Identity.SourceDigest.empty() && !program.Identity.NormalisedDigest.empty());
        rejected([&] { c.start_program(program.Identity, deadline()); }, ReasonNotReady);
        const auto pending = c.status(deadline());
        require(pending.Execution.Handles.size() == 1 && pending.Execution.Handles[0].State == "prepared" && pending.Execution.Generation == 0);
        receipt(c.enable(511, deadline()), Operation::Enable);
        receipt(c.arm(deadline()), Operation::Arm);
        observe([](const auto& s) {
            if (s.Core.Armed != 1 || s.Core.AxisEnableMask != 511 || s.Core.Axes.size() != 9) return false;
            for (const auto& axis : s.Core.Axes) if (axis.Readiness != "ready") return false;
            return true;
        });
        const auto initial_events = c.events(0, deadline());
        auto cursor = initial_events.NextSequence;
        const auto incarnation = initial_events.AdapterIncarnation;
        auto telemetry_cursor = telemetry_sample(c, 0, deadline()).Header.last_sequence;
        std::size_t telemetry_count = 0;
        std::ofstream telemetry_capture;
        if (auto path = std::getenv("RTCORE_TELEMETRY_CAPTURE")) {
            telemetry_capture.open(path, std::ios::binary);
            require(bool(telemetry_capture));
        }
        auto capture_telemetry = [&] {
            const auto batch = telemetry_sample(c, telemetry_cursor, deadline());
            if (!batch.Records.empty())
                require(batch.Header.first_sequence == telemetry_cursor + batch.Header.dropped + 1);
            for (const auto& record : batch.Records) {
                require(record.axes == 9 && record.t_ns > 0 && record.seq > telemetry_cursor);
                if (telemetry_capture.is_open()) {
                    auto raw = gradient::ipc::v1::encode_CycleCaptureRecordV2(record);
                    telemetry_capture.write(reinterpret_cast<const char*>(raw.data()), raw.size());
                    require(bool(telemetry_capture));
                }
            }
            telemetry_count += batch.Records.size();
            telemetry_cursor = batch.Header.last_sequence;
        };
        Request start; start.Operation = to_json(Operation::StartProgram).text; start.Identity = program.Identity;
        const auto started = c.command(start, deadline());
        receipt(started, Operation::StartProgram);
        const auto replayed = c.command(start, deadline());
        receipt(replayed, Operation::StartProgram);
        require(start.RequestID && replayed.Sequence == started.Sequence);
        bool event_completed = false;
        auto execution_event = [&](const Event& event) {
            require(event.AdapterIncarnation == incarnation && event.Sequence == cursor + 1);
            require(event.Type != "events_dropped" && event.Type != "publisher_overflow");
            cursor = event.Sequence;
            if (event.Type == "execution_completed" && event.Execution && event.Execution->Generation == 1 && event.Execution->NativeSequence == started.Sequence)
                event_completed = true;
        };
        const auto event_deadline = deadline();
        while (!event_completed) {
            check_renewal();
            capture_telemetry();
            const auto batch = c.events(cursor, event_deadline);
            require(batch.AdapterIncarnation == incarnation);
            for (const auto& event : batch.Events) execution_event(event);
            if (std::chrono::steady_clock::now() >= event_deadline) throw TransportError("event observation deadline");
            std::this_thread::yield();
        }
        // Replay the same execution through SSE and require the same receipt.
        cursor = initial_events.NextSequence;
        event_completed = false;
        c.events_stream(cursor, execution_event, [&] { return event_completed; }, deadline());
        require(event_completed);
        rejected([&] { c.events(UINT64_MAX, deadline()); }, ReasonEventCursorAhead);
        rejected([&] { c.events_stream(UINT64_MAX, [](const Event&) {}, [] { return false; }, deadline()); }, ReasonEventCursorAhead);
        const auto completed = observe([](const auto& s) { return s.Execution.State == "completed" && s.Execution.Generation == 1; });
        capture_telemetry();
        require(telemetry_count >= 2);
        if (telemetry_capture.is_open()) telemetry_capture.close();
        bool telemetry_streamed = false;
        c.telemetry_stream(telemetry_cursor, [&](const TelemetryPublication& b) {
            if (!b.Records.empty()) {
                require(b.Header.first_sequence == telemetry_cursor + b.Header.dropped + 1);
                telemetry_streamed = true;
            }
        }, [&] { return telemetry_streamed; }, deadline());
        require(telemetry_streamed);
        require(completed.Core.Axes[0].PositionCounts >= 19 && completed.Core.Axes[0].PositionCounts <= 23);
        const auto stopped = c.stop(deadline());
        require(stopped.Generation == 2 && stopped.Session == grant.Session && !stopped.Stopping);
        observe([](const auto& s) { return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0; });
        c.prepare_program(read_program(argv[3], "valid"), deadline());
        c.stop_renewal();
        observe([](const auto& s) {
            if (s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 || s.Execution.State != "cancelled") return false;
            for (const auto& handle : s.Execution.Handles) if (handle.State == "prepared" || handle.State == "started") return false;
            return true;
        });
        rejected([&] { c.renew(deadline()); }, ReasonExpired);
        rejected([&] { c.arm(deadline()); }, ReasonControlSessionStale);
        const auto expired_stop = c.stop(deadline());
        require(expired_stop.Session.empty());
        const auto successor = c.acquire("motion-server-profile", binding, deadline());
        require(!successor.Session.empty() && successor.Session != grant.Session);
        require((expired_stop.Generation == 3 && successor.Generation == 4) || (expired_stop.Generation == 4 && successor.Generation == 5));
        const auto released = c.release(deadline());
        require(released.Session.empty() && c.fence().Session.empty());
        std::cout << "sequence complete (polling and SSE verified)\n";
        return 0;
    } catch (const std::exception& error) {
        c.stop_renewal();
        try { c.stop(deadline()); } catch (...) {}
        try { c.release(deadline()); } catch (...) {}
        std::cerr << error.what() << '\n';
        return 1;
    }
}
