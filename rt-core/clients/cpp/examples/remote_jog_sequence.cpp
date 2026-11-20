#include "rosie/rt_jog_remote_producer.hpp"
#include "remote_jog_test_access.hpp"
#include <iostream>

using namespace rosie::rt_control;
using namespace std::chrono_literals;
using WireTest = remote_jog_detail::TestAccess;

static void require(bool condition, const char* failure) { if (!condition) throw std::runtime_error(failure); }
template<class Predicate> void observed(Predicate predicate) {
    // Unverified observation ceiling: 1000 ms. Poll cadence is 1 ms; a returned
    // receipt/state predicate, never elapsed sleep, establishes success.
    auto deadline=std::chrono::steady_clock::now()+1000ms;
    while (!predicate()) {
        require(std::chrono::steady_clock::now()<deadline,"observation bound exceeded");
        std::this_thread::sleep_for(1ms);
    }
}

static void wire_refusal(RtJogRemoteProducer& jog, const std::string& reason,
                         std::uint64_t sequence, std::uint32_t native = 0) {
    try { jog.rejection(); throw std::runtime_error("missing wire refusal"); }
    catch (const JogRejection& e) {
        require(e.reason == reason && e.sequence == sequence && e.native_reason == native, "wrong wire refusal");
    }
}

// Native expiry must throw its typed refusal with the original observation;
// elapsed time or a generic Closed state cannot satisfy the expiry assertion.
static JogObservation observe_expiry(RtJogRemoteProducer& jog, std::uint64_t sequence) {
    try {
        auto o=jog.observe();
        require(o.StateReason!=rosie::protocol::jog_reason_expired,"expiry did not surface a refusal");
        return o;
    } catch (const JogRejection& e) {
        require(e.reason==ReasonJogInputDeadlineExpired && e.native_reason==9 && e.sequence==sequence && e.native_jog_result.has_value(),"wrong native expiry refusal");
        return *e.native_jog_result;
    }
}

// CI supplies an authenticated byte stream through the client's existing
// HttpStream hook. Production supplies its TLS StreamFactory here, verifying
// the listener and presenting the paired principal's client certificate.
int main(int argc,char** argv) {
    const char* stage="acquire and arm";
    try {
        require(argc==5,"usage: remote_jog_sequence authenticated-stream-socket remote-host configuration-sha256 local-observation-socket");
        std::string socket=argv[1];
        RtControlClient client(StreamFactory([socket](Deadline d){return std::make_unique<client_detail::UnixStream>(socket,d);}));
        RtControlClient observer(argv[4]); // CI's read-only /v1/jog oracle.
        client.acquire("cpp-remote-jog",{"bench",7,argv[3]});
        // Unverified renewal cadence: 50 ms within the 500 ms grant lease.
        client.start_renewal(50ms);
        client.enable(511); client.arm();
        observed([&]{ auto o=client.status(); bool ready=o.Core.Armed==1 && o.Core.AxisEnableMask==511;
            for (const auto& axis:o.Core.Axes) ready=ready && (axis.Statusword&0x6f)==0x27;
            return ready; });
        // Unverified source offset stimulus: 1,000,000,000 ns ahead of host.
        auto source=[] {return host_monotonic_ns()+1000000000ULL;};
        // Unverified input lifetime: 100 ms. The unverified 5 ms cadence leaves
        // margin within the 100 ms input lifetime and 250 ms stream idle deadline.
        stage="initial jog setup";
        RtJogRemoteProducer jog(client,argv[2],511,100000000,source);
        std::vector<double> velocity(9); velocity[0]=0.05;
        std::uint64_t input_deadline=0;
        auto next=std::chrono::steady_clock::now();
        stage="initial jog updates";
        for (std::uint64_t sequence=1;sequence<=4;++sequence) {
            next+=5ms; std::this_thread::sleep_until(next);
            require(jog.update(velocity,source()),"producer contended");
            observed([&]{auto o=jog.observe(); auto local=observer.jog_status(); input_deadline=o.InputDeadlineHostNs;
                return o.HasInput==1 && o.ObservedSourceSequence==sequence && local.HasInput==1 && local.ObservedSourceSequence==sequence;});
        }
        stage="stale sequence and identity refusals";
        WireTest::frame(jog,4,source());
        wire_refusal(jog,"jog_native_rejected_6",4,rosie::protocol::jog_reason_stale_sequence);
        WireTest::frame(jog,5,source(),true);
        wire_refusal(jog,ReasonJogSessionStale,5);
        stage="input expiry";
        observed([&]{
            stage="input expiry keepalive";
            WireTest::keepalive(jog);
            stage="input expiry remote observation";
            auto o=observe_expiry(jog,4);
            stage="input expiry local observation";
            auto local=observer.jog_status();
            return o.StateReason==rosie::protocol::jog_reason_expired && o.Open==0 && o.HasInput==0 && o.ObservedSourceSequence==4 && input_deadline!=0 && o.NowHostNs>=input_deadline && !local.Open && !local.HasInput;
        });
        stage="input expiry probe";
        WireTest::probe(jog);
        wire_refusal(jog,ReasonJogClockInvalidBudgetOrExchange,0);
        std::cout<<"live WSS input expiry: jog_input_deadline_expired, native reason 9\n";
        stage="explicit jog end";
        jog.end();
        observed([&]{ auto o=jog.observe(); return o.StateReason==rosie::protocol::jog_reason_none && !o.Open && !o.HasInput; });

        // Idle is a separate transport outcome: require its typed refusal and
        // EndJog ticket, without calling end() or Stop to cause the closure.
        stage="idle jog setup";
        RtJogRemoteProducer idle(client,argv[2],511,100000000,source);
        require(idle.update(std::vector<double>(9),source()),"idle setup update contended");
        observed([&]{ auto o=idle.observe(); return o.HasInput==1 && o.ObservedSourceSequence==1; });
        stage="idle jog expiry and closure";
        observed([&]{ WireTest::keepalive(idle); auto o=observe_expiry(idle,1); return o.StateReason==rosie::protocol::jog_reason_expired && !o.Open; });
        auto ticket=observe_expiry(idle,1).Ticket;
        auto idle_start=std::chrono::steady_clock::now();
        WireTest::probe(idle);
        wire_refusal(idle,ReasonJogClockInvalidBudgetOrExchange,0);
        wire_refusal(idle,ReasonJogStreamIdle,0);
        bool closed=false;
        try { idle.rejection(); }
        catch (const TransportError& e) { closed=std::string(e.what())=="remote jog EOF"; }
        require(closed,"idle stream did not close with EOF");
        require(std::chrono::steady_clock::now()-idle_start>=250ms,"idle stream closed before 250 ms");
        observed([&]{ auto o=observe_expiry(idle,1); return o.Ticket==ticket+1 && o.ControlAccepted==1 && o.ControlReason==rosie::protocol::jog_reason_none && o.StateReason==rosie::protocol::jog_reason_none && !o.Open && !o.HasInput; });
        std::cout<<"idle close: jog_stream_idle before EOF; receiver EndJog ticket +1, accepted=1, control_reason=0, state_reason=0, open=0\n";
        try { idle.end(); }
        catch (const Rejected& e) { require(e.reason==ReasonJogSessionStale,"wrong idle End refusal"); }
        // Start a fresh generation, then deliberately attempt an update after
        // Stop. The producer must reject under its original fence.
        stage="Stop fencing";
        RtJogRemoteProducer stopped(client,argv[2],511,100000000,source);
        auto old=client.fence(); auto grant=client.stop();
        require(grant.Generation==old.Generation+1,"Stop generation mismatch");
        bool rejected=false;
        try { stopped.update(velocity,source()); }
        catch (const JogRejection& e) { rejected=e.reason==ReasonControlSessionStale; }
        require(rejected,"late update adopted Stop fence");
        // Read Stop's terminal refusal before the deliberate retired-frame write:
        // a failed write retires the producer's stream and its unread bytes.
        wire_refusal(stopped,ReasonControlSessionStale,0);
        try { WireTest::frame(stopped,1,source()); }
        catch (const TransportError& e) { std::cout<<"post-Stop write: "<<e.what()<<'\n'; }
        closed=false;
        try { stopped.rejection(); }
        catch (const TransportError& e) {
            auto reason=std::string(e.what());
            closed=reason=="remote jog EOF" || reason=="remote jog stream closed";
        }
        require(closed,"post-Stop stream did not close");
        try { stopped.end(); throw std::runtime_error("End adopted Stop fence"); }
        catch (const Rejected& e) { require(e.reason==ReasonControlSessionStale,"wrong End refusal"); }
        client.release();
        std::cout<<"remote C++ conformance complete: 4 samples at 5 ms; typed stale sequence, out-of-fence, expiry, idle and Stop refusals\n";
        return 0;
    } catch (const std::exception& e) {std::cerr<<stage<<": "<<e.what()<<'\n';return 1;}
}
