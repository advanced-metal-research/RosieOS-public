> Historical record: the v4 commands, harnesses and make targets below are retired; current native simulation uses repository-root `./dev-stack.sh` and the motion-server composed tests.

# Position execution ownership

Status: source candidate; public position and Home/GoHome ordering are locally
qualified. No physical-equivalence claim.
Base: PR #96, `620fbff37ac006c51c4313aa207e64c22210b674`.

## Current checkpoint

The sections below retain the migration history. Public feedback-controlled
position now has one subscriber/executor in v1 `run_udp_server`; the bridge's
duplicate writing loop is removed. Record-only Servo mastering remains separate.
Native direct/mesh builds and the feedback probe pass with execution codecs
outside the shared scanner header. Same-run position → Jog/Play is retained;
wrong-run Play is rejected. `make -C motion-server/v1 test-bridge-position-permit`
runs the real Go issuer, production MS reader/owner, loopback RTCore SIM parser,
epoch-change case and independent RTCore contract judge with retained receipts.

The eight-case run in `/tmp/rosie-pr97-resumed-tests-8/tests/bridge-permit.CrnoC8`
passed. A subsequent parent-failing equivalent-unit regression identified the
RTCore SIM count/s mismatch. Reusing the existing conversion (selectively
restored from `719f6d99`) fixes it; the unit regression and composed run
`/tmp/rosie-pr97-unit-qualified-tests-10/tests/bridge-permit.1XcQzs` pass,
including a bounded Jog displacement and valid raw-encoder-frame assertion.
This checkpoint is not a completed composite or physical-equivalence verdict.
The user approved routing Home/GoHome through MS for ordering only. It now uses
one local hold after source preparation, idempotent source commit, and exact
terminal reconciliation. Source validation and hardware algorithms remain
bridge/RTCore-owned. Stop/disarm/revoke/faults remain immediate preemption.
Home retains a source-commit acceptance reply for existing completion-polling
clients; terminal receipts are immutable and separately retained.

`/tmp/rosie-pr97-home-composite-15/tests/bridge-permit.wNpfSD` passes public
GoHome ordering, denied-request non-cancellation, position-during-Home rejection,
lost acknowledgement, original-ID retry, Stop-before-commit, and actual bridge
process restart without creating another run. Reconciliation is read-only and
cannot recreate executable authority. Fresh RTCore Home-active/unknown readback
also rejects new position admission; an MS process restart cannot retain its
old position execution. The existing isolated SIM HM35 helper does not persist
a durable Home reference: the joint test requires the unchanged bridge validator
to reject that result, not claim a completed durable Home. Positive durable HM35
completion is source-validator test evidence only, not physical qualification.
The no-manifest legacy Home/GoHome subjects now explicitly reject requests:
their existing examples lack cell, immutable ID and source authority, so they
cannot be forwarded without inventing credentials. The unified per-cell path
retains those capabilities with current authority; hardware packet helpers
are unchanged.
Run lifetime is retained separately from a position command's deadline. The
source receipt carries both remaining intervals, anchored before the cold RPC.
A completed position does not truncate valid same-run Play/Jog, and ordinary
MS lease renewal cannot extend the source run: expiry enters the existing
owner revoke path. This distinction has a parent-failing regression.
The composite includes main `91a32eaf` (PRs 100, 101 and 103); the seven-receipt
run is `/tmp/rosie-pr97-main-composite-16/tests/bridge-permit.uyLC5R`.
RTCore JogV2 binds pair/Home, not runtime-mode epoch: an old datagram can be
accepted after a fresh Arm. The MS owner rejects stale input and stops on
expiry/revoke, but this PR does not claim RTCore wire-level replay protection
across rearm or silently change that protocol.
Stop after Home commit has a separate parent-failing test: an ordinary zero
Jog cannot abort Home. That Stop now enters the existing serialized revoke/
Disarm path, and RTCore Disarm clears pending Home/GoHome flags so a delayed
completion callback cannot resume them. `/tmp/pr97-stop-committed-home-fixed`
passes; the parent failures are retained beside it. No Home algorithm changed.
Lost position-admission replies now retry the exact pending challenge/request
within the original deadline. The intent watermark remains an ordering fence,
not an execution-success counter. A public same-ID retry regression fails on
`c6ce3537` and passes with this recovery. The real quiescence barrier also now
withholds drained receipts while position admission or Home dispatch remains
outstanding; its regression covers idle, pending Home, and cancellation recovery.

## Change

Route `position` through the shared motion execution owner, preserve its
observable contract, prove equivalent motion, then remove the bridge's
duplicate feedback controller. Do not introduce a second execution service
or keep an automatic fallback to the old controller.

The current controller is `handlePositionCommandPayload` in
`robot/v4/realtime-core/natsbridge/bridge_commands.go`. It resolves relative
targets from feedback once, checks configuration binding and frame safety,
computes error-to-velocity commands, revalidates authority before sending,
and checks completion. This is execution, not transport.

Keep bridge-owned hardware translation and independently enforced RTCore
protections. The migration must use the execution owner's existing command
ordering, cancellation and authority mechanisms rather than copy them into
another adapter. Select the concrete execution entry point against the
composed #96 source before changing routing; v1 and v2 are not interchangeable
merely because both accept motion.

Before implementation, name the selected process/module and command entry
point, its owning lease/fence, and the receipt consumer. This is an unresolved
design choice, not an existing unified owner. Do not route commands until it
is settled against the current code.

The cutover must leave exactly one subscriber responsible for executing and
replying to `position`. Exercise duplicate command IDs, a delayed old request,
and `position` interleaved with jog/trajectory commands: only the selected
owner may write motion. Do not forward the same public subject back onto itself
or leave both implementations responding during rollout. Keep rollback as a
pinned release choice, not a per-command fallback.

## Required equivalence

| Behavior | Migration proof |
| --- | --- |
| Target/frame | Absolute and relative moves preserve home-relative units, axis mapping, signs, effective limits, and single resolution of a relative target. |
| Authority | Valid lease succeeds; expired, superseded or wrong-session requests cannot move. Configuration/Home identity changes cancel admitted work. |
| Cancellation | Stop, Disarm, replacement, disconnect and shutdown cannot leave the old controller sending commands or resurrect a completed result. |
| Completion | Acceptance is not completion. Preserve tolerance, settling and timeout outcomes; retained results identify the original operation. |
| Failure/recovery | Missing/stale/faulted feedback stops execution; recovery does not silently resume the old command. A fresh valid request works afterward. |
| Motion | Compare old and replacement on identical pinned configuration and targets in simulation, then explicitly qualify the relevant hardware paths. Compare measured tracking, stop behavior and completion, not only packet acceptance. |

The existing loop stops on the first in-tolerance sample and requires four
samples, sleeping 40 ms between samples. It sends 180 ms jog leases, checks
growing error, and supports both bound feedback and telemetry-ring sources.
These are baseline behavior, not automatically permanent architecture rules:
any deliberate change needs its own evidence and must be called out.

Equivalence means preserving intended capability, not every accidental result.
Today `positionCommandCanceled` can return `nil` from the handler; that alone
does not prove arrival. The replacement must distinguish accepted, completed,
cancelled and failed outcomes and correlate them to the original command.
Declare how current clients receive those outcomes before changing replies.

Start from the existing regressions in `natsbridge`: `bridge_command_authority_test.go`,
`bridge_listener_shutdown_test.go`, `bridge_position_feedback_test.go`, and the
position/Disarm cases in `bridge_test.go`. Run the same behavioral scenarios
through the replacement entry point; copied helpers passing in isolation do
not establish the routing or single-writer property.

The simulation-only record-only Servo mastering path is separate from
feedback-controlled motion. Preserve that distinction; do not turn its
nonphysical acknowledgement into a claimed completed move.

## Go Home is separate

`handleGoHomeCommandPayload` sends a dedicated RTCore command; it does not run
the bridge's position feedback loop. Leave it unchanged in this slice.
Including it requires inspecting RTCore's command semantics, configured axis
mask, reference frame, limits, cancellation and completion behavior, then a
separate equivalence proof. Drive homing is also outside this migration.

## Removal and naming

Delete the duplicate position controller only after the replacement passes
the sequence and motion comparisons. Keep the baseline in Git and retained
evidence, not as a selectable runtime fallback. Rename components alongside
the actual ownership change; no preliminary mass rename, deployment topology
change, or unrelated client redesign belongs here.

## First implementation slice

Position cancellation now returns `context.Canceled` instead of success;
the existing command-reply envelope reports `accepted: false`,
`state: cancelled`, and the original command ID. Other commands retain their
reply behavior. This is a deliberate outcome correction, not a routing change.
The reply regression fails on the parent (which reports rejected even when
given an explicit cancellation). Full handler-to-client recovery still needs
the migration sequence proof.

The selected integration target is v1's `run_udp_server` command arbitration,
using its leader authority and motion-owner mechanisms. Add position there
without another feedback-writing process. The NATS request/reply consumer
must continue receiving the original command ID; routing is not enabled yet.

No deployment, lease, or physical state was changed.

## Feedback-boundary cancellation proof

The production handler previously returned success if cancellation arrived
while the fourth in-tolerance feedback read was blocked. The post-read path
could send settling holds and return before checking its generation again.
`TestPositionCancellationDuringFinalFeedbackAndRecovery` reproduces this over
loopback UDP with exact-bound feedback: on `d21cb995`, command 42 returns nil
and its reply says `accepted: true`. The handler now checks cancellation after
feedback acquisition, before interpreting the sample or reporting completion.
The same test requires the original cancelled command ID and successful
completion of a fresh command 43. No extra controller, authority, speed bound,
timeout, or routing fallback is introduced.

This is a cancellation regression for the current controller, not the
single-writer migration proof. In particular, a generation check is not an
atomic output fence; a cancellation arriving after the check still requires
the owning execution path's serialization. Verdict: `passed` for the
feedback-read cancellation regression; shared-owner and motion equivalence
remain `not_evaluated`.

### Next integration boundary

The selected process is `motion-server/v1` and the concrete owner is the
`run_udp_server` loop in `robot_v4_cartesian_udp_runtime.hpp`. Its
`LeaderAwareRobotCommandQueue` feeds leader-fenced cold motion application;
`motion_owner_allows`/`acquire_motion_owner` arbitrate the current sender.
The queue classifies position as energizing work for revoke purging. The loop
now admits leader-bound position, advances it in `emit_output`, and cancels it
through shared replacement/Stop/revoke paths; public subscription stays bridge-owned.

The next implementation must admit a typed position request through that
queue, classify it in the existing revoke purge, retain its original receipt
identity through execution, and advance its feedback state in the existing
loop rather than block that loop or start another writer. The existing
`emit_output` path must serialize position against jog/plan output. Its
terminal result must be distinguishable from the cold-motion acceptance
receipt. Bridge transport cutover must occur together with removal of its
position subscription, after tests cover duplicate/delayed requests,
position/jog/plan replacement, stop/disarm/revoke, and completion. The current
regression is an observable baseline for that production-path suite, not
permission to copy the bridge loop into a second controller.

### Typed v1 ingress and lifecycle slice

`parse_robot_command_message` now decodes `PositionExecutionRequest`; public
ownership remains bridge-only. `PositionExecution` is a pure one-request
lifecycle, not another output controller: it resolves relative targets once,
intersects owner-supplied limits, tracks the admitted deadline and four 40 ms
settling samples, and retains immutable terminal outcomes and command identity.
It does not authorize, generate velocities, or run in a second thread.
The production queue purges position alongside other energizing work.

`position_execution_runtime_test.cpp` compiles the actual daemon parser and
queue, tests these sequences and malformed inputs, and checks that public
position ownership has not changed. Verdict: `passed` for typed ingress,
lifecycle and queue revoke/recovery. `position_loop_runtime_test` also passes
synthetic-plant arrival, public UDP jog replacement, position replacing jog,
revoke/reacquire, final-feedback cancellation, and public terminal-status retention.
This is internal-ingress simulation evidence; public cutover remains `not_evaluated`.
The C++ `make test` suite and Go `-run Leader` pass. Broader Go `Leader|Command`
also selects a calibration fixture whose NATS INFO lacks `max_payload`; that
test fails publication and was left unchanged, not counted as passed.

Position ingress now preserves string or uint64 command IDs as original JSON
tokens, including integers above 2^53. Its terminal reply encoder uses the
existing command-reply envelope; completed/cancelled/timed_out/failed outcomes
retain the original type. Deduplication distinguishes numeric 42 from string
"42"; duplicate JSON keys and malformed IDs are rejected. Parser, lifecycle,
terminal wire framing and existing acceptance-reply tests cover this mapping.
The existing subscriber now delivers and replays immutable terminal replies
from the same 256-command history. `make test-position-nats` exercises the real
broker/UDP lane in a test-only subscription build, including fault, raw-frame,
runaway and cancelled retries without re-execution. Production routing is unchanged.

Concrete cutover blockers: v1 uses leader leases;
the bridge owns `BridgeBenchRunAuthority`'s active
run and RevPi-local monotonic deadline under its output mutex. V1's
`BridgeRunCredential` merely transports that identity for downstream weld
authorization; it is not a leader grant or a replica of the active-run store.
Do not compare its deadline with the Jetson clock or invent a leader lease.
The bridge-run authority projection must be settled before public cutover;
terminal retention is bounded and process-local, not durable across restarts. Record-only Servo
mastering is intentionally not decoded as feedback-controlled execution.

The permit core now reuses bridge run admission and the existing leader owner:
bridge-issued source-clock receipts bind challenge, run/boot, original command,
real leader fence and RTCore generation. Local expiry is anchored before the
cold request, not on response arrival; leader renewal cannot extend it.
Cold RPC, exact grant replay and End/disconnect transport now use the existing
NATS readers. `make test-bridge-position-permit` runs the real Go issuer and C++
consumer together. A run-bound leader cannot bypass admission by stripping run
fields from Arm or motion. Locally valid replacement stops A before requesting
B; later B rejection does not silently replace a running A. Lost End delivery
still relies on bridge local purge/readback; failed purge remains failed.
Nonce/field matching is not transport authentication. Production cutover is next.
