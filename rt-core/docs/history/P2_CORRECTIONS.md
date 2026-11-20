# P2 correctness and M1 evidence — 2026-09-12

Initial implementation commit: `1b1353a` on `codex/standalone-rt-core`.

## Completing the M1 implementation

Implementation commit: `b615edc272e9fa534f525537e0048a7cf018bf81`.
[M1 evidence](m1-evidence-2026-09-12.json) binds the tested source to this commit.

The subsequent implementation extracts the full pure Start decision and per-axis
output transition, and routes cyclic PDO access and checked queue/send results
through `CycleBus`. Successful submission receipts identify exactly which axes'
target histories can advance. Failed or partial domain queues cannot produce a
successful group acknowledgement. Broader P4 state ownership and diagnostic
ioctl/trace cleanup remain separate work.

Endpoint settling now has its own required per-axis `completion_timeout_ms`.
Its deadline is anchored to the native planned endpoint and never refreshed by
feedback publication. Fresh ready feedback must meet the existing count-aware
tolerance on time; otherwise fault bit 9 cancels and inhibits before submission.
The six sample configurations provide explicit values. The bench example is an
unqualified placeholder, not measured drive tuning.

The new regression fails on the prior corrected `1b87ecf` engine with documented
simulator/wire-compatibility instrumentation: two selected signed feedback offsets
and a long plan remain Executing after their endpoint deadlines. All three pass
with the fix; an ideal-feedback control completes on both. The baseline endpoint
logic is unchanged. `build/p2-settling-evidence.json` records its source archive,
exact adaptation, schema/header and binary hashes.

Native and public Discard release only inert Prepared handles. Absent/retired is
idempotent success; Ready/Active returns mode conflict without cancelling motion.
Program replacement orders preparation, old-handle disposal and publication with
Start, and reclaims a new handle if publication is cancelled. Uncertain transport
or disposal outcomes cannot leave a handle advertised as definitely prepared.

Protocol **1.4** makes Start/Discard use one generated eight-byte little-endian
record and rejects old minor versions. The native header remains 128 bytes, with
the retired setpoint offset reserved zero. Strict per-axis parsing requires all
critical descriptors; the 14 retired global drive-default options now reject.
All six current machine examples still validate.

The protocol generator is now Go. Its output parity with the original Python
generator was established on frozen 1.3 fixtures before the 1.4 record extension;
independent C++/Go golden bytes and malformed lengths validate the new codec.
The old Python generator is removed. Existing Python test sources are unchanged;
their SDK/compiler receive only the protocol/configuration compatibility needed
to keep running the original oracle. Full Python removal remains P1 after P3.

Combined validation and commit identifiers are recorded in
VALIDATION.md and the tracker. Hardware, brake
engagement, native C++ TSAN, remote-jog timing and actual stack cutover are not
qualified by these software results. Jog still shares the old blocked ingress;
P3 remains required before G0 can close.

## Initial correction checkpoint (1b1353a)

The reviewed drive-readiness, failed-submission, legacy Stop, prepared Abort and
disabled-displacement defects have simulator regressions that fail on `895e249`
and pass on the corrected engine. This is partial M1 implementation evidence.
Jog separation, the remaining P2 extractions/cleanup, brakes, public fault reset
and host/drive qualification still block the release gates.
The [machine-readable evidence](p2-evidence-2026-09-12.json) records baseline
provenance, candidate source hashes, authoritative logs and qualification limits.

## Behavior and evidence

| Finding | Original engine | Corrected engine |
|---|---|---|
| R01: Start before drive enable | Accepts a closed-loop plan and reports Completed while the drives remain disabled. | Rejects `not_ready` without starting the clock; a fresh plan executes after all requested axes reach Operation Enabled/CSP. |
| R02: failed send or domain queue | Reports a successful Stop acknowledgement although the inhibited image was not submitted. | Latches bus failure, revokes authority and withholds submission acknowledgement. A partial queue prevents sending that command group. |
| Related receive/process/DC failure | A stationary active plan completes despite failed input processing or clock synchronization. | Fault state wins over completion in the same cycle. Receive, process and domain-state results participate in feedback freshness. |
| R04: disabled backdrive | After +2,000 observed counts, three reset/re-arm attempts each hit the stale-target position-rate fault. | Reconciles trustworthy disabled feedback, holds the observed position through all three re-arms and executes a fresh plan. +60,000 out-of-limit counts remain inhibited with a coordinate-reference fault. |
| R05: legacy jog STOP | Applies cancellation but returns rejected/not_ready. | STOP and STOP with QUICK_STOP return accepted global cancellation. Unsupported combinations return invalid_command. No physical quick-stop behavior is promised. |
| R06: prepared Abort | A successfully aborted prepared handle can still start; unused preparations strand the bounded slots. | Prepared/Ready handles retire, pending Starts receive rejection, and slots are reclaimed. Active cancellation uses a per-slot flag that cannot cancel a later reused numeric ID. |

The native ABI stays at 1.3 for these corrections. No second motion output path,
new Python functionality, relaxed rate limit or relaxed staleness budget was
introduced. The Go fixture gains a selectable daemon and short temporary paths;
the original Python test sources are unchanged.

The start gate reads current PDOs before activation and sampling; it is separate
from Enable/Prepare admission to avoid an enable-state bootstrap cycle. Submitted
target history advances only for successfully queued and sent axes. Bus errors
remain visible as operation plus signed return code; an ACK is submission evidence,
not readback or standstill.

The re-arm extraction is deliberately limited: an inactive CiA402 state and
fresh, valid coordinates permit preload. Active/QuickStop/stale observations
retain the continuous target. Wrapped feedback requiring an inferred turn stays
ambiguous and requires qualified reconciliation; no absolute position is guessed.
This does not qualify a gravity axis or replace a holding brake.

## Reproduce

Run from `rt-core` on Linux with the documented C++/Go/Python toolchains:

```sh
make test
make test-go
make test-sanitize
RTCORE_TEST_BINARY="$PWD/build/rosie-rt-core-asan" go test -count=1 -race ./ipcclient
```

New coverage lives in `ipcclient/readiness_bus_linux_test.go`,
`ipcclient/rearm_linux_test.go`, `ipcclient/cancellation_linux_test.go` and the
four new C++ policy/boundary test executables registered by `make test`.
The existing Python suite remains the oracle through P2/P3; these additional
regressions are written in Go/C++ to avoid adding to the planned Python port.

For fail-before comparisons, the local `build/p2-baseline` tree preserves the
original `895e249` source and binary, plus an instrumented original binary.
Instrumentation copies the candidate simulator and adds only simulator option
parsing and preactivation configuration. It does not copy the candidate's
readiness, clock, output policy, store or bus-error decisions. Its source archive,
input hashes, patch and simulator hash are recorded in `build/p2-baseline/provenance.json`.
Pass the original binary for cancellation and the instrumented original for
plant/bus tests through `RTCORE_TEST_BINARY`. Baseline failures are intentional.

On this WSL host the Windows filesystem cannot bind Unix sockets. Test runtime
storage was a temporary tmpfs mounted inside `rt-core/build/runtime`; a helper
kept WSL running while tests used it. Sources, build caches and evidence stayed
inside `rt-core`. The initial full Go run overlapped mount setup and is excluded;
the stable-runtime rerun is the authoritative result. No persistent host setup
or physical EtherCAT operation was performed.

## Remaining qualification scope

The later M1 implementation above supersedes the initial checkpoint's open
extraction, Discard, settling and dead-configuration work. The following are
release requirements, not implied by completion of M1:
- Keep P3 separate jog ingress and source-age rejection as a required software
  gate. The shared adapter/client/helper path is still present.
- Preserve Fable's brake, public reset, security, producer/telemetry migration,
  retirement and documentation work. No hardware, C++ TSAN, remote-jog latency,
  actual-stack cutover or final no-Python qualification is claimed here.

Historical baseline probes must use their recorded 1.3 client/test revision,
not the current 1.4 SDK. Their original source/fixture hashes and results remain
in `p2-evidence-2026-09-12.json`; the new endpoint baseline explicitly records
its minimal 1.4 wire bridge separately.
