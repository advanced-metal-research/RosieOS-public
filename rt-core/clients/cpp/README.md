# C++ application client

Build with `make clients`. C++17 consumers use `-Iclients/cpp/include -pthread`;
both jog helpers also require `-Iinclude` for the generated frame codec.

Run an explicitly prepared configuration and local `rt-control`, then execute:

```sh
build/rt-execute /run/rosie-rt-core/control.sock PAIR_ID PAIR_REVISION CONFIGURATION_SHA256 program.rdt
```

The example acquires, enables, arms, uploads an inert program, explicitly starts,
observes completion and releases. It prints the completed status JSON. Exit codes
are 0 for completion, 2 for a rejection with its schema reason, and 3 for transport
or observation failure. It attempts Stop and release on failure.

Include `rosie/rt_control_client.hpp` and use `rosie::rt_control::RtControlClient`.
Results use the generated public types. Catch `Rejected` for `reason` and optional
`native_result`; catch `TransportError` for uncertain outcomes, including malformed
responses. Pass an absolute steady-clock deadline to bound a call and its retry.
Lifecycle, urgent and bulk operations own independent persistent HTTP connections.

Acquire automatically remembers the fence. Call `renew()` explicitly or start
`start_renewal(std::chrono::milliseconds(50))`, observe `take_renewal()`, and stop
with `stop_renewal()`. A `stopping` grant is a successful receipt. Renewal errors
stop the keeper; they never reacquire authority. Call `release()` explicitly before
destruction and join other calls before destroying the client.

Every mutation carries a random request ID. To retry a lost control reply, keep a
`Request` and pass it to `command()` again without changing its ID, fence or body.
Automatic retries reuse that same request once. Retry only within the original
live session's retained outcome window. Release and `.rdt` uploads are never
automatically replayed. An uncertain upload must not lead to Start; inspect state
or Stop explicitly. `prepare_program()` accepts immutable binary `std::string`
bytes, including NUL bytes.

`RtJogProducer` negotiates `/v1/jog/clock` and begins a fixed jog generation.
Supply `jog.sock`, an axis mask and an absolute host `CLOCK_MONOTONIC` deadline.
Call `update(velocities, deadline_ns)` with fresh input. False means congestion:
discard that input and send a fresh sample later. `end()` closes the producer and
ends its original generation. Stop cancels it; never reuse it after Stop.

Path jog (protocol 1.14: `begin_path_jog`, `local_path_samples` chunks, path
speed frames) is not supported by this client; use the Go SDK's
`NewPathJogSession`. The C++ frame codecs for those records are generated, but
no producer, handshake or observation helper exists here yet.

Unix sockets are the supplied transport. Remote listeners require TLS with server
verification and client credentials. The `HttpStream`/`StreamFactory` constructor
is the transport hook for a separately supplied TLS implementation, with a fresh
stream per connection and deadline-bound I/O. No plaintext TCP transport is
provided. HTTP responses require content-length or chunked framing; unsupported
chunk extensions fail closed.

Remote pendant applications include `rosie/rt_jog_remote_producer.hpp` and use
`RtJogRemoteProducer(client, host, axis_mask, input_duration_ns, source_now)`.
The header-only client does no TLS itself. The pendant must supply a
`StreamFactory` returning a fresh mutual-TLS `HttpStream` that verifies the
listener's certificate and hostname and presents the component's client
certificate and private key for the paired principal. Each stream must support
one concurrent read and write, honor absolute connect/read/write deadlines, and
never replay bytes. Supply that factory to `RtControlClient`; the producer calls
`open_remote_jog_stream(deadline)` to obtain its independent WSS connection.
Keep lifecycle, renewal and jog connections under the same authenticated identity.

After acquisition, renewal, Enable/Arm and observed readiness, the producer
samples `/v1/jog/clock` for BeginJog, upgrades `/v1/jog`, and performs two timestamp
exchanges with a fresh source-clock incarnation. The second source sample must
follow receipt of the first reply and echo its `host_tx_ns`; the constructor does
this exchange. Supply one monotonic clock for `source_now` and every
`update(velocities, source_time_ns)` input-capture timestamp. The receiver's
qualified offset interval conservatively maps source time to host time. Default
receiver qualification flags are zero and binary frames are refused with
`jog_clock_unqualified`; deployment must supply measured uncertainty, drift and
calibration-age limits.

The producer retains its original session, grant generation and jog generation.
Renewal cannot extend an input's source deadline; Stop requires a fresh producer
after renewed authority and readiness. An update returning false sent no frame
because another writer held the send lock: discard it and sample fresh input.
Writes establish transport progress only. `rejection(deadline)` reads typed
`JogRejection` wire labels and generated native codes; `observe(deadline)` returns
the receiver's native jog observation through `/v1/status`. Those reads do not
hold the send lock. Only an explicit native Expired reason is reported as
`jog_input_deadline_expired`; Closed and Ramping remain the receiver's actual
reasons. The simulation currently replaces admitted-input Expired with
Ramping then `Expired`; Stop answers a later frame with `control_session_stale`
before closing WSS.

The receiver closes WSS after **250 ms without a complete frame**, including an
incomplete frame stalled at the read deadline, answering `jog_stream_idle` before
the close and calling EndJog. The idle bound is the same constant as the
**250,000,000 ns native source-age ceiling** (`FastControlMaxJogInputAgeNs`), not
a separate rule; the chosen input duration is the producer's own. The producer
sends no automatic keepalive. The example's 5 ms update cadence is unverified; the
bounds that constrain a cadence are the input lifetime the producer declares and
the 250 ms idle deadline, not the source-age ceiling alone. It is not a qualified
pendant profile. Call `end()` explicitly, including after transport
failure, then release authority separately. Join concurrent calls before
destroying the producer. A failed stream is retired immediately from new calls;
in-flight I/O retains its storage only through its existing absolute deadline.

CI runs `examples/remote_jog_sequence.cpp` through a Go bridge that forwards
plaintext bytes over a Unix socket to the real mutual-TLS listener. C++ performs
the upgrade, calibration and generated frame encoding; Go performs TLS. **C++ TLS
itself is not exercised.** The adjacent `remote_jog_test_access.hpp` is only a
conformance hook for deliberately invalid frames and keepalive stimuli; it is
not an application API. The example checks wire refusals, native `Expired` with
WSS kept alive, the idle close (`jog_stream_idle` before EOF) and the Stop close
(`control_session_stale` before EOF) as typed reasons.

The telemetry decoders include `include/protocol_generated.hpp` by relative path
(`../../../../include/protocol_generated.hpp`), so this directory is no longer
self-contained: a consumer that copies the client headers must also copy that
generated header, or add rt-core's `include/` to its include path.
