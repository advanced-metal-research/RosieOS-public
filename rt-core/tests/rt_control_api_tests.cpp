#include "clients/cpp/include/rosie/rt_control_api_generated.hpp"
#include <cassert>
#include <iostream>
#include <iterator>

using namespace rosie::rt_control;

// Round trips verify all generated fields survive, preventing encoder/decoder omissions.
template<class T> void round_trip(T value = {}) {
    const auto wire = encode(value);
    T decoded{};
    std::string error;
    assert(decode(wire, decoded, &error));
    assert(encode(decoded) == wire);
}

// The Go contract test supplies independent Go-encoded populated fixtures here;
// returning them through the generated codec detects cross-language field drift.
template<class T> int echo(const std::string& wire) {
    T value{};
    std::string error;
    if (!decode(wire, value, &error)) { std::cerr << error << '\n'; return 1; }
    std::cout << encode(value) << '\n';
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 2) {
        const std::string name = argv[1];
        const std::string wire{std::istreambuf_iterator<char>(std::cin), {}};
#define ECHO_TYPE(T) if (name == #T) return echo<T>(wire)
        ECHO_TYPE(Fence); ECHO_TYPE(Binding); ECHO_TYPE(Point); ECHO_TYPE(Identity);
        ECHO_TYPE(Request); ECHO_TYPE(Response); ECHO_TYPE(RawResponse); ECHO_TYPE(Grant);
        ECHO_TYPE(HandleRecord); ECHO_TYPE(Execution); ECHO_TYPE(ProcessStatus);
        ECHO_TYPE(JogObservation); ECHO_TYPE(CommandResult); ECHO_TYPE(GrantObservation);
        ECHO_TYPE(FaultRecovery); ECHO_TYPE(RecoveryStatus); ECHO_TYPE(AxisStatus);
        ECHO_TYPE(CollisionWatchdogStatus); ECHO_TYPE(BrakeHoldDiagnostic); ECHO_TYPE(BrakeWarningEvent);
        ECHO_TYPE(DriveSupervisorDiagnostic); ECHO_TYPE(DriveSupervisorEvent);
        ECHO_TYPE(IoInput); ECHO_TYPE(IoOutput); ECHO_TYPE(IoState);
        ECHO_TYPE(ResourceInfo); ECHO_TYPE(RobotDescription); ECHO_TYPE(DriveIdentity); ECHO_TYPE(DriveDescription);
        ECHO_TYPE(NativeStatus); ECHO_TYPE(MotionState); ECHO_TYPE(ProcessMarker);
        ECHO_TYPE(Program); ECHO_TYPE(JogIngressObservation); ECHO_TYPE(JogIngressRefusal); ECHO_TYPE(AxisDescription);
        ECHO_TYPE(ProgramLimitData); ECHO_TYPE(ProgramLimitViolation);
        ECHO_TYPE(IndependentAnchorSource); ECHO_TYPE(ReferenceVerifiedDeltaRestorePolicy); ECHO_TYPE(AnchorDeltaRestoreStatus); ECHO_TYPE(VerifiedDeltaRestoreReceipt); ECHO_TYPE(PathJogCapability); ECHO_TYPE(PathObservation); ECHO_TYPE(PathJogStatus);
        ECHO_TYPE(MaintenanceObservation); ECHO_TYPE(MaintenanceHolder);
        ECHO_TYPE(AxisLoad); ECHO_TYPE(AxisMonitor);
        ECHO_TYPE(NativeDescription); ECHO_TYPE(LocalJogClock); ECHO_TYPE(Description);
        ECHO_TYPE(CapabilityInfo);
        ECHO_TYPE(LogicalAxisStatus); ECHO_TYPE(StatusGenerations); ECHO_TYPE(PlanCursor);
        ECHO_TYPE(BufferHealth); ECHO_TYPE(AdapterStatus); ECHO_TYPE(GrantEvent);
        ECHO_TYPE(HandleTransitionEvent); ECHO_TYPE(ExecutionEvent); ECHO_TYPE(JogEvent);
        ECHO_TYPE(FaultEvent); ECHO_TYPE(EnableEvent); ECHO_TYPE(EpochEvent);
        ECHO_TYPE(IncarnationEvent); ECHO_TYPE(PublisherOverflowEvent);
        ECHO_TYPE(EventsDropped); ECHO_TYPE(Event); ECHO_TYPE(EventBatch);
        ECHO_TYPE(TelemetryMarkEvent);
#undef ECHO_TYPE
        return 2;
    }
    round_trip<Fence>(); round_trip<Binding>(); round_trip<Point>(); round_trip<Identity>();
    round_trip<Request>(); round_trip<Response>(); round_trip<RawResponse>(); round_trip<Grant>();
    round_trip<HandleRecord>(); round_trip<Execution>(); round_trip<ProcessStatus>();
    round_trip<JogObservation>(); round_trip<CommandResult>(); round_trip<GrantObservation>();
    round_trip<FaultRecovery>(); round_trip<RecoveryStatus>(); round_trip<AxisStatus>();
    round_trip<CollisionWatchdogStatus>(); round_trip<BrakeHoldDiagnostic>(); round_trip<BrakeWarningEvent>();
    round_trip<DriveSupervisorDiagnostic>(); round_trip<DriveSupervisorEvent>();
    round_trip<IoInput>(); round_trip<IoOutput>(); round_trip<IoState>();
    round_trip<ResourceInfo>(); round_trip<RobotDescription>(); round_trip<DriveIdentity>(); round_trip<DriveDescription>();
    round_trip<NativeStatus>(); round_trip<MotionState>(); round_trip<ProcessMarker>();
    round_trip<Program>(); round_trip<JogIngressObservation>(); round_trip<JogIngressRefusal>(); round_trip<AxisDescription>();
    round_trip<ProgramLimitData>(); round_trip<ProgramLimitViolation>();
    round_trip<IndependentAnchorSource>(); round_trip<ReferenceVerifiedDeltaRestorePolicy>(); round_trip<AnchorDeltaRestoreStatus>(); round_trip<VerifiedDeltaRestoreReceipt>(); round_trip<PathJogCapability>(); round_trip<PathObservation>(); round_trip<PathJogStatus>();
    round_trip<MaintenanceObservation>(); round_trip<MaintenanceHolder>();
    round_trip<AxisLoad>(); round_trip<AxisMonitor>();
    round_trip<NativeDescription>(); round_trip<LocalJogClock>(); round_trip<Description>();
    round_trip<CapabilityInfo>();
    CapabilityInfo legacy_jog;
    assert(decode(R"({"name":"jog","state":"test_only","transport":"POST /v1/control"})", legacy_jog));
    assert(legacy_jog.Name == "jog" && legacy_jog.State == "test_only" && legacy_jog.State == CapabilityStateTestOnly && legacy_jog.Transport == "POST /v1/control");
    round_trip(legacy_jog);
    round_trip<Event>(); round_trip<EventBatch>(); round_trip<LogicalAxisStatus>();
    EventsDropped dropped;
    assert(decode(R"({"first_lost_sequence":9007199254740993,"last_lost_sequence":18446744073709551615})", dropped));
    assert(dropped.FirstLostSequence == 9007199254740993ULL);
    assert(dropped.LastLostSequence == 18446744073709551615ULL);
    round_trip(Operation::Acquire);
    round_trip(Operation::SubscribeEvents);
    Operation operation = Operation::Acquire;
    assert(!decode("\"not_in_the_contract\"", operation));
    assert(operation == Operation::Acquire);

    // Decode the independent wire label against the generated reason catalogue;
    // a principal refusal must remain recognizable to application clients.
    RawResponse principal_refusal;
    assert(decode(R"({"schema":"rosie.rt-control.response.v1","operation":"renew","error":"session_principal_mismatch"})", principal_refusal));
    assert(principal_refusal.Error == ReasonSessionPrincipalMismatch);

    Fence fence;
    assert(decode(R"({"session":"a\"b\\c\n\uD83D\uDE80","generation":18446744073709551615})", fence));
    assert(fence.Generation == 18446744073709551615ULL);
    assert(fence.Session == "a\"b\\c\n\xf0\x9f\x9a\x80");
    assert(encode(fence) == "{\"session\":\"a\\\"b\\\\c\\u000a\xf0\x9f\x9a\x80\",\"generation\":18446744073709551615}");
    Point point;
    assert(decode(R"({"time_ns":-9223372036854775808,"position":[1.25,-2.5],"velocity":[0.5,0]})", point));
    assert(point.Time == std::numeric_limits<std::int64_t>::min());
    assert(point.Position.size() == 2 && point.Position[0] == 1.25 && point.Position[1] == -2.5);
    round_trip(point);
    Request request;
    request.Schema = RequestSchema;
    request.Operation = "prepare_trajectory";
    request.Fence = fence;
    request.RequestID = "prepare-1";
    request.Points = {point, point};
    const auto request_wire = encode(request);
    assert(request_wire.find("\"session\"") < request_wire.find("\"points\""));
    assert(request_wire.find("\"generation\"") < request_wire.find("\"points\""));
    round_trip(request);

    CommandResult receipt;
    assert(decode(R"({"Sequence":9007199254740993,"Handle":7,"Generation":8,"Operation":9,"Result":0,"Reason":0,"AxisMask":511})", receipt));
    assert(receipt.Sequence == 9007199254740993ULL && receipt.AxisMask == 511);

    NativeStatus status;
    status.RTCPU = 7;
    status.HousekeepingCPUs = "2-3,5";
    status.AffinityApplied = {{"bulk", false}, {"worker\"\\\n", true}};
    NativeStatus observed;
    assert(decode(encode(status), observed));
    assert(observed.RTCPU == 7 && observed.HousekeepingCPUs == "2-3,5");
    assert(observed.AffinityApplied.size() == 2);
    assert(!observed.AffinityApplied.at("bulk") && observed.AffinityApplied.at("worker\"\\\n"));
    assert(encode(observed.AffinityApplied) == R"({"bulk":false,"worker\"\\\u000a":true})");
    round_trip(status);

    std::map<std::string, std::uint64_t> counts;
    assert(decode(R"({"first":9007199254740993,"last":18446744073709551615})", counts));
    assert(counts.size() == 2 && counts.at("first") == 9007199254740993ULL);
    assert(counts.at("last") == 18446744073709551615ULL);
    assert(encode(counts) == R"({"first":9007199254740993,"last":18446744073709551615})");
    std::map<std::string, std::string> labels;
    assert(decode(R"({"":"empty key","\u03bb":"value\n\"\\"})", labels));
    assert(labels.size() == 2 && labels.at("") == "empty key" && labels.at("\xce\xbb") == "value\n\"\\");
    round_trip(labels);
    for (const auto* invalid : {R"([])", R"({"bulk":1})", R"({"bulk":true,"bulk":false})", R"({"bulk":true,"worker":null})"}) {
        const auto prior = encode(observed.AffinityApplied);
        assert(!decode(invalid, observed.AffinityApplied));
        assert(encode(observed.AffinityApplied) == prior);
    }
    for (const auto* invalid : {R"({"first":-1})", R"({"first":1.5})", R"({"first":18446744073709551616})", R"({"first":"1"})"}) {
        const auto prior = encode(counts);
        assert(!decode(invalid, counts));
        assert(encode(counts) == prior);
    }
    assert(decode("null", counts) && counts.empty());
    assert(decode("{}", labels) && labels.empty());

    for (const auto* malformed : {
        "", "{", "[]", "null", "{}", "{\"session\":\"x\",\"generation\":1} true",
        "{\"session\":\"x\",\"generation\":18446744073709551616}",
        "{\"session\":\"x\",\"generation\":-1}",
        "{\"session\":\"x\",\"generation\":1.5}",
        "{\"session\":\"x\",\"generation\":01}",
        "{\"session\":\"x\",\"generation\":1,}",
        "{\"session\":\"x\",\"generation\":1,\"generation\":2}",
        "{\"session\":\"\\uD800\",\"generation\":1}",
        "{\"session\":\"\\uDC00\",\"generation\":1}",
        "{\"session\":\"\xc0\x80\",\"generation\":1}"
    }) {
        const auto prior = encode(fence);
        assert(!decode(malformed, fence));
        assert(encode(fence) == prior);
    }
    for (const auto* invalid : {"null", "0", "\"\"", "\"\\u001f\""}) {
        assert(!decode(std::string("{\"schema\":\"rosie.rt-control.request.v1\",\"operation\":\"halt\",\"request_id\":")+invalid+"}", request));
    }
    assert(!decode(R"({"schema":"rosie.rt-control.request.v1","operation":"halt","surprise":1})", request));
    assert(decode(R"({"schema":"rosie.rt-control.request.v1","operation":"acquire","controller":"client"})", request));
    assert(!decode(R"({"time_ns":0,"position":[1e999]})", point));
    bool refused = false;
    try { point.Position = {std::numeric_limits<double>::infinity()}; (void)encode(point); }
    catch (const std::exception&) { refused = true; }
    assert(refused);
    std::cout << "rt_control_api_tests: all generated types and malformed JSON passed\n";
}
