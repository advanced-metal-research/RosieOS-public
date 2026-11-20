# Standalone realtime core: implementation plan and completion roadmap

2026-10-19 P8-45 A6-EC absolute linear-mode migration
(`feat/j5-j6-wrist-limits`, Building): every A6-EC axis uses `C00.07 = 1`,
software motor-to-axis gearing and standard drive gear `0x6091:01/02 = 1/1`;
HM35 writes `607C = 0`. Coordinate evidence, retained Home and verified-delta
compare complete signed counts exactly. Rotation mode, wrapped feedback and
commands, batteryless mode 3, and public `feedback_wrap`, `command_wrap` and
`anchor_period_counts` fields are retired. The application schema digest is
`c607bcc67cb5a6cc49eb05b64cecc80492e4799c6a1baa77c24159b1b3b64d46`
(since the 2026-10-31 `reset_preserves_home` advisory,
`0948a57f2ce6c5928e9ecb94cb8725046ec03f29684deb96d3e76a4bd8240662`);
the native, sanitizer, protocol/API, OLP, Deck and one complete aggregate
`make test-go` gate pass. The mini bench proves C00.07=1 activates after a
power cycle and config-verifies inhibited. Its exact declared ALF9.0 warning
can preserve valid live counts while powered, but the renewed HM35 transaction
stays at statusword 0x06A1 after CW15 and never attains Home.
The branch must not merge until a battery box is fitted and the full mini-bench
release proves HM35 zero, ±719-degree travel/reversal and bounded Stop from
servo-rate evidence, and the first cell to migrate is ready. Power-cycle retention and battery
removal are subsequent battery-backed qualifications. After
owner approval and merge, each migrated cell requires a full release and operator HM35 on
every joint. Tracking:
P8-45 and
issue #160 (internal tracker)
for remaining unrelated hard-coded A6-EC literals.

2026-10-06/07 native cutover and tracking work on a Rosie 1430 cell
(`64e76816..fd9af899`): native rt-core is the
only controller (JSLOP removed), J6 is 100:1 with drive readback, and
operator HM35 established Home on all six joints. Target lead is a warning
(`84d28eac`); both tracking bounds grow by the 50 ms machine-level
`tracking_lag_allowance` (`cd8c5b4c`); A6-EC speed feedforward 1/500/318
closes P4-19's question (`6c6f97c0`: 0x60FF is ignored in CSP; lag 27 -> 15 ms
measured); `jog_jerk` 150 rad/s^3 bounds jog, Halt ramps and OLP Move-to, which
now follows one shared progress (`82f88462`, `b2e38b46`, `770675a2`). The
AMR-715 shutdown hang recurred and its root cause is fixed (`5f070e44`,
`fc13479a`; proven live, 0.39 s stop with a mailbox call in flight). The path
jog lane P6-19 (protocol 1.14) is merged and unverified on hardware. Every
tracking value (50 ms, 500, 150 rad/s^3, per-joint `max_target_lead`) is
unverified on the loaded arm (#158). P8-06 stays Backlog with existing
owner/dependencies; P4-19 is Merged, not Verified.

2026-10-02 current-pose reference removal: the software current-pose zero
introduced on this branch (`capture_current_reference`, native command 266,
the `reference_capture` axis policy, `rosie-anchor-v5` receipts and their
compiler, SDK, OLP and test support) is deleted, not stubbed. Home is
established only by drive homing (HM35), which writes 607C; rt-core records the
HM35 position as a v3 anchor for check and backup. That anchor is restored
automatically after a power cycle by direct drive checks, by `RestoreAnchor`,
or by verified-delta restore (command 267) for v3 only. A remaining
`rosie-anchor-v5` file is refused, untouched, as `anchor_source_invalid`:
"current-pose reference anchors are not accepted; establish Home with drive
homing (HM35)". A configuration declaring `reference_capture` no longer
compiles or loads, so trial configurations that opted J6 in cannot be
reproduced byte-for-byte; their entries below remain historical evidence only.
Software never routes around a drive or hardware failure (for example a drive
that fails HM35): the failure is surfaced and fixed at the drive.

2026-09-27 P3-08 follow-up (Codex, client/IPC): consumed C++ jog reservations wake
the keeper; bounded busy-snapshot retry preserves fail-closed observation. Existing
client and v5 checks plus mirrored Deck simulator direction tests pass. Keep the
historical stage; physical qualification and sustained reliability remain open.

2026-09-24 P8-27 follow-up: separate saved Home calibration from cyclic position
confidence; remove the independent multi-turn mailbox-age motion gate and use
read-only, stationary recovery. Existing Home sequence, limits and following-error
protection remain unchanged.
An inhibited bench deployment and automatic six-axis nonzero-position
recovery passed. Motion release/OLP execution acceptance remain blocked on reset/
turn-loss qualification. External Chrome connection/status/reload checks pass;
the stalled capture was fixed with off-cycle HTTP gzip; Chrome captured 3000
records in 15.0 s without motion or calibration changes;
retain P8-27's historical Merged stage without claiming this follow-up Verified.

Owner-operated external Chrome execution subsequently completed: revision 14,
48.071 s, zero faults, six trusted positions. Full 1 kHz run samples were not
preserved before ring rollover; reset/turn-loss and repeat/restart gates remain open.

P8-27 start-position usability follow-up: Go to plan start uses the selected
trajectory's actual first joint positions through the existing bounded move
lane, including offsets below zero-all's 0.02-degree display tolerance. Robot
identity, target freshness, Stop and native admission remain enforced. Existing
Go motion tests and 65 focused UI checks pass. Stage remains Merged.

P8-27 capture contention follow-up: isolate SDK telemetry reads from Arm preflight
reservations and stop interpreting background reconnect notes as unsent operator
commands. Subsequent physical validation completed the unchanged 47.84-second
rev14 path twice, including after restart, using measured J6 internal velocity
feedforward. Fixed SDK Reset retaining the retired renewal session. All six Home
references restored; original motion guards and robot limits remain unchanged.
Stage remains Merged;
broader browser/UI acceptance is still separate.

P8-27 retained-Home follow-up: recover valid drive Home with the existing software
anchor after restart, preserving movement since calibration and checking the
current encoder/PDO frame. Startup offset readback is re-enabled; original Home
sequence and running guards remain unchanged. Stage remains Merged.

2026-09-23 P8-27 tracking follow-up: deployed the last-successfully-submitted
command reference for following-error checks, with existing thresholds and Home
timing preserved. OLP now captures/imports/exports executed TCP traces and compares
them with submitted commands and the displayed plan. Native/ARM, telemetry and
focused UI checks pass; three existing broad UI failures and full 3D/physical
acceptance remain recorded. Stage remains Merged.

2026-09-23 trajectory-start follow-up to P8-27: the operator completed Home on
all six axes (native success logs and Home mask 63). A later dry run refused
start_discontinuity. The exact plan starts at zero time/position/velocity; an
offline replay rejects a 45-count held-position offset but accepts 37 counts.
Add a separate 50-count start-alignment floor for homed, qualified independent
encoders, retaining runtime 16-count checks and trajectory rate/position gates.
The live rejection-time offset was not recorded, so that cause is not proven.
Implementation 57a5c81b adds that floor and per-axis refusal tracing. WSL
start-alignment, motion-readiness and cycle-machine tests and local control/sim
build pass. ARM alignment/readiness and configuration checks pass; 57a5c81b is
deployed on the bench with preserved configuration/anchors. All seven local
services restarted healthy and the app matches the new bench runtime identities.
Physical start acceptance remains pending; the earlier
six-axis Home succeeded before this restart. P8-27 remains Merged.

2026-09-23 Home margin follow-up to P8-27: use a 50-count
stationary encoder margin for fresh Home only, preserving the 16-count Restore
and runtime checks and the established Home sequence/timing. The follow-up also
covers complete local-stack and physical bench teardown/restart, and
qualified per-axis Home success to survive a peer's failure during Home All.
Combined implementation 51a6a27a passes WSL coordinate, worker and commissioning
policy tests and the local control/simulation build. ARM coordinate/worker and
native configuration checks pass; the bench is restarted with 51a6a27a,
unchanged configuration and preserved anchors. All seven local services have
restarted healthy; the app's bench target, both runtime incarnations and configuration
match the deployed bench. Runtime pairing still rejects discrepancies
beyond 16 counts after Home. The subsequent operator Home completed all six axes,
confirmed by native logs and Home mask 63.

2026-09-23 fault-recovery follow-up to P8-27: the next retry reached the new
adapter with the old execution error cleared, but a J3 coordinate-reference
fault blocked Home admission before reservation. Add the qualified existing
ResetFault -> observed clear -> reacquire -> Home flow, plus admission diagnostics
and visible fault/recovery evidence. No native gate exemption or Home sequence,
timing or threshold change. Native diagnostics fe651353 are deployed with
preserved configuration/anchors; ARM worker and config checks pass. Recovery
f60d6f08 adds ownership/timestamp fences and atomic conditional Stop checks for
delayed responses across reset/reacquire. Final denseexec race suite, 70 UI
tests, TypeScript and production build pass; rebuilt app/UI are healthy and
reconnected. Physical six-axis Home and the separate J5 qualification remain open.

2026-09-23 acquisition follow-up to P8-27: a retained error from a previous
cancelled execution was still published after a new grant succeeded. OLP treated
it as current and sent Stop, interrupting the new Home. Retire that old
application execution error only after a new native grant is accepted and
authorized. Preserve native faults and Home sequencing/timing. The Home race
regression and full SDK/control suite pass; the adapter suite's intermittent
torch preparation cancellation also reproduces on the unchanged baseline.
Implementation d61f3f73 is deployed by restarting only the control adapter;
native state and anchors are unchanged. Full-group physical Home and the
distinct earlier J5 stationary-window refusal remain unresolved.

2026-09-23 physical follow-up to P8-27: after bb64f663, operator Home completed
J1-J4 but refused J5 with stationary_window; later attempts lost permission.
The latest observation has Home mask 0 and no active Home. Full-group Home is
not fixed or verified. Frontend work adds per-joint Home states and an aggregate
count, using both HTTP contact age and native observation advancement for
freshness. No Home protocol or sequence changes; 93 focused UI tests, TypeScript and production build pass.

2026-09-23 binding follow-up to P8-27: captured J2 evidence identifies the
stationarity refusal. Its 18-count interval stays within 15 counts of the fixed
acquisition anchor. Check the actual acquisition interval against the existing
16-count anchor-error budget, replacing the global stationary-window/full-span
test with a bounded preallocated range query. No tolerance increase or Home
sequence/timing change. Correction bb64f663 passes WSL/ARM measured-record replay and boundary regressions plus ASan/UBSan; deployed on the bench. Physical six-axis Home remains unverified.

2026-09-23 diagnostic follow-up to P8-27: the same J2 CompleteHome refusal
persisted after the timing rollback. Add internal binding-refusal reasons and a
receipt snapshot logged by the non-realtime worker, without changing Home order,
delays, timeouts, limits or acceptance predicates. WSL/ARM coordinate and commissioning worker
regressions and native configuration validation pass; 09942295 is deployed on the bench. The physical
cause was not yet established at that checkpoint; the subsequent capture and
binding correction are recorded above. Full-group Home remains unverified.

2026-09-23 Home recovery follow-up to P8-27: remove
the fresh-Home receipt wait added in 997f53e9 and restore the preceding worker
sequence. Restore retains its existing receipt wait; the observed-position
binding correction remains. Subsequent physical attempts completed J1 but failed
J2 CompleteHome, then a later attempt timed out at the J6 disarm precondition.
Rollback 7dd340e0 passes WSL/ARM tests and is deployed on the bench; local services are refreshed. Full-group Home remains unverified.

2026-09-23 follow-up to P8-27: robot-bound bench motion limits inherit the
selected robot, including execution and jog acceleration. Remove the legacy
A6 profile acceleration cap and reject machine overrides for URDF-mapped axes.

2026-09-22 follow-up to P8-27: connected OLP planning now includes native
position and rate limits, separates caches when those limits change, and
refuses to plan when the selected target cannot provide its limits. Upload
limit refusals expose the offending value and bound for replanning. Software
regressions cover the reported J6 rejection; physical Load acceptance remains
an operator check after replanning against the deployed bench configuration.

2026-09-22 follow-up to P8-27/P4-15: native jog now reserves stopping distance
before reaching a position limit; OLP keeps accepted Limited sessions alive.
The J9 limit repair has command-level and
simulator acceptance. Machine-specific physical stopping qualification remains
open; genuine tracking/reference faults still require their existing recovery.

Updated 2026-09-12 with merge notes appended through 2026-09-18. Reviewed code
baseline: `895e249`, on `codex/standalone-rt-core`.
The thirteen `history/*.json` evidence records named in this document were untracked on
2026-09-17 by owner decision (bloat review item 5) and remain readable at tag
`archive/standalone-rt-core-2026-09-17`; they are cited by name, not linked.
 This is the authoritative remaining backlog; earlier
checkpoints describe partial implementation, not completion. Checkboxes close
only when the stated acceptance evidence exists; design decisions do not close
their implementation work.

The follow-up review changes execution order to **P0 minimum decisions â†’ P2 â†’
P3 â†’ P1 with test parity â†’ incremental P4/P5 â†’ P6/P7 â†’ P8**. Phase numbers remain
stable for traceability. Keep the existing Python test oracle through P2/P3;
Python absence is required at final G2, not at the earlier safety gates.
[KICKOFF.md](KICKOFF.md) fixes the process/socket topology, remote jog transport,
R21 policy and first milestone. tracker.html breaks the backlog
into owned work packages and records stage, dependencies and completion evidence.
No work-duration estimates are used. The independent review of 2026-09-12 added
R25 to R27, four written decisions in KICKOFF (GradientOS relationship, schema
and generator naming, jog transport fallback, Go for new test coverage), brake
sequencing and an interim fault reset as G1 prerequisites, jog producer and
telemetry successor items in P6, remote listener security in P7, v4 retirement
and documentation consolidation in P8, and visible Deferred items.

Implementation checkpoint `b615edc`: [P2 corrections and evidence](history/P2_CORRECTIONS.md)
records completed M1 implementation: readiness/output/bus boundaries, checked
submission, re-arm, Stop/Abort, explicit Discard/replacement, endpoint settling
and dead configuration/ABI cleanup. The original oracle and new Go/C++ regressions
pass, with race/sanitizer evidence in `history/m1-evidence-2026-09-12.json`.
The M1 changes are committed on the test branch. Independent review REV-01 in
the tracker reopens P2-05 for wrapped-feedback re-arm and requires reproducible
baseline evidence and explicit fault/completion contracts before verification.
The `history/review-response-2026-09-12.json` records confirmed gaps,
the exact historical hash explanation and findings already covered in code.
P3 checkpoint `1c77968`
adds tested atomic shared-memory and generated-frame foundations, source-clock
and jog admission policies, and the RT-owned Home correction for R28. See
`history/p3-evidence-2026-09-12.json`. Checkpoint `214770e` adds the native
application-grant gate and direct request/observation region, with bounded
grant publication, source-fixed expiry and mapping retirement. Its
`history/p3-grant-evidence-2026-09-12.json` includes committed logs and
121 source/configuration hashes. P3-09 is In review; jog transport and bulk/worker
separation are not complete. G0, G1 and G2 remain open.
REV-01 follow-up `1c79f6f` retains the reproducible native 1.3
A/B package, proves the idle fault/recovery contract, protects test assertions,
enforces the IgH build API and replaces the standalone verifier with Go. Its
`history/rev01-followup-evidence-2026-09-12.json` records a complete
canonical copied-source run and a moving native-role application-grant expiry
regression. P2-13 was In review at that checkpoint. P2-05 and the new P2-14 domain-freshness correction
were open at that checkpoint; the later runtime correction is recorded below.
No release gate or full Python port is closed.
All changes in this work are confined to `rt-core`; eventual changes to actual
stack consumers and v4 retirement remain coordinated external work packages.

REV-02 reviewed the native grant checkpoint while the above follow-up was in
progress. Its `history/review-response-rev02-2026-09-12.json` maps every
finding. P3-09 stays In review. P3-10 covers failed-observation connection teardown,
one revocation per event and generated grant geometry/policy constants. Current
adapter/daemon deployment requires the same UID and PID namespace; broader
container credentials remain P7 qualification.

REV-03 reviewed only source `1c79f6f` through documentation `6b24185` and explicitly
excluded the in-progress runtime correction below. It advances P2-13 to Reviewed,
clears the NDEBUG block and closes the idle-bus test finding. Its
`history/review-response-rev03-2026-09-12.json` keeps four remaining requests
visible: automate the unprotected-NDEBUG negative, make one canonical driver
cover Go/sanitizers/historical replay and adopt it in CI, emit command/result
records for otherwise silent replay steps, and expose interim public ResetFault
before bench recovery. Existing pinned logs are retained unchanged; new runner
records must supplement them. CI workflow edits outside `rt-core` remain
coordinated external work under the current folder-only scope.

Checkpoint `ccbde01`: the `history/review-runtime-evidence-2026-09-12.json` records
per-domain current input evidence, full-group cyclic scheduling, truthful PDO
observation timestamps, terminal grant connection cleanup and one fence per
expiry, terminal connection loss or received native cancellation packet. Public
Stop still sends multiple native packets. Shared grant geometry and lease policy are generated
without changing protocol 1.5 bytes. P2-14 and P3-10 enter In review with retained
regressions and evidence; REV-01/REV-02 are preserved and do not automatically
close. Wrapped-coordinate recovery (P2-05), independent jog/bulk/service lanes,
public recovery, remaining review coverage, Python removal and qualification
remain required work. Terminal socket closure precedes helper join, but a bound
behind a genuinely blocked mailbox call still belongs to worker qualification.

REV-04 reviews `6b24185..dd15e34` and advances P3-09, P3-10 and P2-14 to
Reviewed. Its `history/review-response-rev04-2026-09-12.json` keeps P2-05
Building: the optional synthetic coordinate source is not emitted by the normal
compiler or enabled by the real A6 profile. Daemon refusal/recovery, qualified
source conversion and acquisition bounds remain required. The strict missed-
exchange recovery policy now belongs to the R21/bench gate. The neutral R29
replay is distinguished from new-API tests and from a historical whole-daemon
run. Current uncommitted scalar/source-loss/cancellation corrections are outside
this review. P4-01 must reduce orchestration growth before further P4/P5 features.

Wave 3 checkpoint (orchestrated Codex sub-agents, gated per merge): `28234839`
CONTRACTS.md; `abe7dbd1` FaultBrakeHold bit-9 collision fix; `cd5fab9e` P4-01
extractions 1-3 (main.cpp 7,595 to 4,800 lines, independently verified as
behaviour-preserving); `63a45d55` public handle lifecycle, idempotence and
concurrent-caller policy; `8e1e27e7` mutual-TLS remote listener and WSS jog lane.
Each merged tree passed make control, make test, make test-go and make test-sanitize.
Wave 4 checkpoint: `211ebe53` Go review-simulation runner; `44a0ae02` consumer
inventory; `8b9a1ee7` admission before decoding; `fdf1d508` P4-01 complete
(main.cpp 158 lines); `b81fcfc3` Go application SDK and rtctl control; `5b3a8761`
identified reproducible package. Orchestrator fixes: `026d079e` rt-control bootstrap
retry, `1a50f412` disarm gate, `48a71025`/`c5097a01` copied-tree identity. Gate
green at `c5097a01`.
Wave 5 checkpoint: `537aab7a` fault matrix; `5fec966c` commissioning and bulk
workers with fixes `15721cf1`; `81f444f6` restart fencing; `26baaa3b` diagnostics
off-cycle and bounded capture; `498b57b0` fault recovery matrix and public
recovery; `e62d1dd6` complete native protocol generation. Gate at `3fb629a8`:
native, oracle, host and package checks green; every Go package green except one
fault-matrix publication bound exceeded once under three-builder load (passes alone);
27 sanitizer tests green.
Wave 6 checkpoint: `2d1c8e65`/`7f65aa7a` time-based jog policy; `5cebf9f1` housekeeping
cpuset and host checks; `b3b17a7e` prefaulted in-place plan storage; `5ba4fde5` public
API contract generation. Gates on this host now run beside up to eight agents; each
gate had one or more timing-bounded Go tests fail once and pass alone, recorded against
P8-01 and P5-03; native, oracle, host, package and sanitizer steps green at `9bdf2d57`.
Wave 7 checkpoint: `699c71eb` lifecycle tests; `1fb6560f` soak driver; `d7ef560f` C++
client; `a58048d5` planner corpus policy (no real planner outputs exist in the repository);
`45d7c5ed` CI driver; `43e80c84` interpolation declaration; `7f49a4a0`/`0995a778` machine and
deployment identity; `34b497c0`/`8c41f3de` public contract resync after two parallel-merge
drifts. Gate at `8c41f3de`: fifteen Go packages green, one load flake (passes alone), one
deterministic corpus-report drift fixed `1cb1448b`; oracle 38, host scripts, sanitizer 27 green.
Wave 8 checkpoint: `794ffccb` coherent logical status and bounded events (P5-04, rt-core half
of P6-08); `45cb6d31` Go port of all 38 frozen oracle cases with paired parity records (P1-09);
`f0efcae1` documentation consolidation (P8-12) with fix-forward `a3bf38a9`. Orchestrator fixes:
`a3cfd47b` resident-memory budget bound to the production daemon (sanitized runs skip it by
allowlist), `c84ebd7b` review fix-forward on the oracle port (DATA RACE scan, 800 ms revocation
hold, re-captured a6ec golden), `c84ebd7b` parity record on the merged tree. Gate at `a3cfd47b`:
native, oracle, host and package checks green, 18 Go packages green with the one stale golden
failing until `c84ebd7b`, sanitizer 33 green; the four touched Go packages re-run green after the fix.
Current memory-accounting correction: that unlocked WSL checkpoint omitted the
fully resident diagnostic queue and worker stacks when `mlockall` succeeds.
The existing 16 MiB residual allowance remains; named locked components are
accounted separately in the [storage budget](ARCHITECTURE.md#storage-and-memory-budget).
Wave 9 checkpoint: `2590b2fd` Python runtime, frozen oracle and retired tools removed, host
scripts through `rtctl run` and `rtctl control-env`, tracked-source retirement scan (P1-10);
`d41b184d` shared simulation fixture and helper consolidation, net 461 lines removed with a
zero-unmapped assertion census; fix-forwards `b842dbd8`, `73a16846`. Read-only audit of the
whole sub-agent range: on target, no files outside rt-core, bloat was duplication and
caller-less flags. Gate with python3 absent from PATH (a bin directory of symlinks to the build's tools only; command -v python3 returned nothing): on 73a16846 make control, make test-go (19 packages green, 2 allowlisted skips) and make test-sanitize (33) exit 0; on 72a745f0 (comment-only change) make test exit 0 (native, oracle, host, install, PKI, package and lifecycle checks) and make check-python-absent passed with zero hits; the only two failures in the first run were an openssl symlink missing from the orchestrator's restricted PATH and the orchestrator's own provenance comment. rt-core/build/orchestrator-gate.log.

## Executive finding

Keep the standalone `rt-core` boundary, C++ execution engine, per-drive profiles,
logical axis units, native trajectory clock and one final motion authority.
Correct the implementation before calling it ready: jog and bulk currently share
blocking paths, important drive/bus state transitions are incomplete, and the
actual Rosie deployment has not migrated. Passing baseline simulation tests did
not cover all of these defects.

The finished component will use **C++ for cyclic execution and Go for off-cycle
services, configuration, integration and tooling**. It will contain no owned
Python source or Python build/test/runtime dependency. Existing Python remains
transitional until its replacements pass equivalent tests. The P2 corrections
do not remove it or complete the remaining architecture and integration work.

Completion means a qualified, installed core used by the real Rosie stack, with
clear APIs, reproducible builds, tested recovery and rollback. A standalone demo
or an SDK command that executes a plan is an intermediate result.

## Fixed architectural decisions

1. **Separate ingress, shared execution authority.** Jog, bulk transfer, bounded
   control and status/events have independent capacity and scheduling. Stop and
   authority revocation have an urgent path. Inside each RT cycle, one owner
   decides mode, checks authority/coordinates/limits and permits drive output.
2. **C++ owns the cycle.** No allocation, blocking locks, network/file I/O,
   formatting, configuration parsing or sleeping SDO operations in cyclic work.
   Go GC and application latency cannot control cycle scheduling.
3. **Go owns off-cycle work.** Provide Go SDK/tooling and a C++ client for the
   existing C++ motion server. Do not spawn a CLI for every control request.
   The separate Go adapter owns the sole application grant. Both clients use its
   public boundary, never private native IPC. Each wire format has one canonical
   schema generating its C++/Go codecs, extended with each settled contract
   increment; no hand-packed duplicate codecs. KICKOFF names the native, public
   API and jog schemas/generators and the process/socket boundaries.
4. **Plans are inert until Start.** Preserve current motion-server `.rdt` input,
   up to 250,000 points, explicit axis mapping and native timed q/optional qd
   execution. Prepare may run during jog; plan execution and jog cannot both
   control the same axis group. No implicit takeover or per-axis retiming.
5. **Jog is latest-value input.** Preserve source age and a bounded deadline;
   newer sequences supersede older ones. Uploads, queueing or lease renewal must
   never make old human input fresh again. Define remote clock conversion.
6. **Stopping has explicit meanings.** Urgent Stop revokes/inhibits; a qualified
   controlled Halt/EndJog ramp is a separate capability. Accepted command,
   successful bus submission, drive state and physical standstill are distinct.
7. **Profiles describe drives; policy describes axes.** Identity, PDO/SDO/DC,
   home and absolute-feedback differences belong in validated profiles. Future
   Yaskawa EtherCAT support must not require another execution architecture;
   exact model/ESI qualification remains deferred as requested.
8. **The root folder owns the component.** Keep `rt-core`, including the realtime
   application, EtherCAT integration, pinned dependency build/install recipes,
   services, configuration, clients and tests. PREEMPT_RT and the kernel-matched
   IgH master remain explicit host dependencies, not a new in-tree OS/kernel.
   Small Make/shell/systemd plumbing and upstream C dependencies are compatible
   with the Go/C++ requirement; owned application/tooling logic is Go or C++.

## Review disposition and traceability

This table tracks the original findings and their disposition. Reproduced means
a local simulation probe, not a physical-drive result. Implementation evidence
does not close the wider release gates.
`history/review-evidence-895e249.json` records observations and
injection scope. The older [architecture review](history/ARCHITECTURE_REVIEW.md) and
validation history remain historical evidence.

| ID | Finding and evidence status | Required work |
|---|---|---|
| R01 | Reproduced on 895e249; P2 rejects Start before fresh readiness and prevents invalid-feedback completion. The separate endpoint-timeout defect also fails on the prior 1b87ecf checkpoint and passes now. | M1 Start extraction and bounded settling implemented/tested. Physical tracking qualification remains. See history/P2_CORRECTIONS.md. |
| R02 | Reproduced on 895e249 for failed sends and queues; candidate withholds acknowledgement and publishes typed failures. Process/DC and partial-domain tests also pass. | M1 checked cyclic bus interface implemented/tested. Startup/diagnostic restructuring and hardware evidence remain P4/P7. See history/P2_CORRECTIONS.md. |
| R03 | Reproduced: a 50 ms jog waits about 172 ms behind a 250,000-point upload and is then accepted. Adapter `operationMu`, Go `commandMu` and helper FIFO couple the lanes. | P3: independent jog ingress through every layer; preserve input age. |
| R04 | Reproduced for continuous feedback: three reset/re-arm attempts after +2,000 disabled counts fault on 895e249 and pass the correction. REV-01 identifies an unresolved wrapped-feedback/continuous-command case: ordinary disabled displacement is classified ambiguous, invalidates home and blocks re-arm, including the described A6 J6 configuration. | P2-05 reopened. Preserve continuous-feedback evidence; add bounded, observed wrap reconciliation without guessing unobserved whole turns. P4 absolute restoration and physical brake qualification remain distinct requirements. |
| R05 | Reproduced on 895e249; candidate accepts supported STOP combinations and rejects unsupported encodings explicitly. | Legacy result correction implemented. Qualified EndJog/halt behavior remains P3/P4. |
| R06 | Reproduced on 895e249; Abort retires preparations with per-slot active cancellation and correlated pending Start results. Discard and transactional replacement now reclaim unused handles and handle ambiguous replies. | M1 lifecycle implemented/tested. Full public active Abort/retry contracts remain P5. |
| R07 | External renew-during-Stop SDK failure claim remains unconfirmed; the current Python SDK serializes those calls on one instance. | P5: test actual concurrent callers, renewal, expiry and generation changes in replacement SDKs. |
| R08 | Source-confirmed: dense-format boundary tolerance is 1e-3, while adapter continuity is effectively 1e-12. Valid legacy inputs can be incompatible. | P6: versioned producer/consumer boundary policy and real planner corpus; no unvalidated connecting motion. |
| R09 | Source-confirmed: ResetFault clears all home validity; no complete public recovery contract. | P4/P5: reason-specific home/recovery policy and ResetFault API. |
| R10 | The reviewed native parser retained legacy home/absolute defaults; the normal compiler supplied those fields. M1 now rejects omitted descriptors and removes the dead global drive options/default route. | Neutral explicit parsing implemented/tested. Canonical artifact verification remains P1. |
| R11 | Source-confirmed: jog quick-stop/arrest durations are fixed cycle counts despite a configurable cycle period. | P4: time-based qualified policy and tests at supported periods. |
| R12 | Source-confirmed: root services, unprovisioned application access/environment, and stale socket restart behavior leave deployment incomplete. | P7: installation, ownership, restart and least-privilege contracts. |
| R13 | Architecture debt: approximately 6,900-line orchestration file and many cross-thread atomics; existing extracted modules do not constitute complete state ownership. | P2/P4: typed boundaries and one coherent cyclic machine state, not just file splitting. |
| R14 | Static memory estimate: three resident buffers plus staging at 272 bytes x 250,000 samples is about 272 MB before Go bodies/copies. This is not measured RSS. | P4/P5: explicit memory budget, fewer full buffers, bounded admission and measured peaks. |
| R15 | Source-confirmed: native configuration SHA is validated as text and echoed, not verified against the full effective runtime configuration. | P1: canonical artifact verification; separate machine identity from deployment identity. |
| R16 | Source-confirmed: generation covers the 40-byte result and selected constants; much of the wire layout remains manual. | P1: complete C++/Go protocol generation and compatibility fixtures. |
| R17 | Coverage gap: small composed suite, missing controller behavior tests, optional/skipped binary paths and incomplete CI race coverage. Local instrumented composed tests did run; they were not all vacuous. | P1/P8: mandatory instrumented boundaries, fault injection and regression coverage. |
| R18 | API gap: raw drive counts and separate snapshots do not provide a complete coherent logical measured/commanded state or event stream. | P4/P5: units, validity, timestamps, generations and bounded events. |
| R19 | Migration gap: actual C++ motion server, remote authority, cell I/O and active builders/services still need integration. | P6/P7/P8: real consumer handoff and deployment qualification. |
| R20 | New requirement: no Python in the completed component or its new control path. | P1/P8: replace source, tooling, tests, SDK and motion-server shim; enforce absence. |
| R21 | Operator-policy gap: inherited clamp/hold behavior and newer fault/disarm behavior differ, especially at jog limits. | P0/P4/P6: explicit per-condition policy, operator feedback and qualified stopping behavior. |
| R22 | Source-confirmed: helper affinity pins CPUs 0/1 regardless of the configured RT CPU (`src/main.cpp:5805`). | P7: explicit housekeeping cpuset, exclude RT CPU, check allowed CPUs and affinity failure. |
| R23 | Source-confirmed: post-send cyclic work queries `ecrt_slave_config_state` per axis (`src/main.cpp:4333`). | P4/P7: separate diagnostic collection from critical freshness checks, respect IgH ownership and measure worst-case cycle cost. |
| R24 | Cleanup scope needs explicit ownership: legacy drive-config/default path, unused `SetpointSlotV1` (`include/ipc_v1.hpp:137`) and optional fast-trace thread (`src/main.cpp:4664`). | P2: remove dead config path and retire slot with ABI checks; P3: use a new jog payload; P4: replace fast trace with bounded observations before removal. |
| R25 | Independent review 2026-09-12: no brake engage/release sequencing or hold-on-disable policy exists anywhere in the plan or code. `BrakeStateV1` is declared in the IPC header; nothing sequences brakes on disable, fault, Stop, lease loss or re-arm. R04 sag is the symptom this prevents. | P4: per-axis brake policy from the drive profile, implemented and tested before a gravity-loaded axis is enabled on the bench (G1). |
| R26 | Independent review 2026-09-12: the v4 bridge telemetry ring and alarm lifecycle (about ten thousand Go lines) have no successor; existing UIs and supervisors read subjects nothing will publish. | P6: bounded publisher from the status/event stream to existing consumers, or consumer migration, decided by the P6 inventory. |
| R27 | Independent review 2026-09-12: the HTTPS control and WSS jog listener chosen in KICKOFF creates an authentication surface separate from the existing NATS credential model, with no provisioning, rotation or revocation item. | P7: decide reuse of existing principal/pair credentials versus an owned TLS surface, then provision and test it. No unauthenticated path to Stop or jog. |
| R28 | Reproduced on the M1 daemon: Stop during axis 0 Home is acknowledged, but the same helper job adopts the new generation and homes axis 1 afterward. Its overall rejected result hides `home_valid=0x2` and later service-state feedback. Late truth/success publication and cleanup-versus-success reporting share this ownership defect. | P3-05: one original job identity and RT reservation across every axis/step; RT applies service permissions and completion, retains Draining until the worker returns, and distinguishes cleanup from successful Home. Go regression checks post-Stop feedback and final home evidence. |
| R29 | Source-confirmed during REV-01 follow-up: round-robin can leave a domain unqueued while CycleBus calls its cached complete WKC fresh again. Pinned IgH retains received datagram state; the simulator refreshes all axes even without that domain exchange. This can hide stale position/readiness evidence. | P2-14: bind freshness to current per-domain exchange/receive evidence, use full-group fixed-period exchange until multi-rate operation is qualified, and correct simulator input presentation. Required before P2-05 measured-turn integration and final readiness qualification. |
| R30 | REV-02 source-confirmed: failed RT grant-observation publication clears connected and revokes but does not ask the accept owner to tear down the connection. The mapped region/socket can stay occupied, silently refusing future authority. | P3-10: bounded connection-tagged teardown request, off-RT diagnostic, peer close and acknowledged mapping retirement; regress Busy/exhaustion, inhibition, reconnect and stale requests. P3-09 remains In review. |

Effective motor-limit admission was also probed and correctly rejected an
overspeed request. Retain that regression; do not carry it as an established
limit-enforcement defect. Earlier fixes for buffered Stop, discontinuous starts
and missed deadlines likewise need to survive the refactor.

## Salvage map: preserve behavior without copying old ownership

| Existing mechanism | Generalized destination and completion evidence |
|---|---|
| v4 command expiry | P3: source-aged latest-value input, tested through the real producer and transport under bulk load. |
| v4 controller lease and pair fencing | P5/P6: one authenticated grant plus independently enforced native expiry; handoff/restart tests. Pair ownership remains outside the core. |
| v4 configuration readback | P1/P2/P7: verified canonical machine artifact plus actual drive readbacks and explicit mismatch faults. |
| v4 jog velocity slew and target lead | P3/P4: logical-unit axis policies with measured time constants and an explicit clamp/halt/fault decision. |
| v4 home-relative limits | P4: limits in reconciled logical coordinates, with offset/sign/gearing tests and invalid-home rejection. |
| v4 restart invalidation | P4/P5/P7: daemon/session/home generations, conservative recovery and no replay. |
| v4 rotary seam protection | P4: continuous-turn/wrap conversion independent of a specific drive's count representation. |
| v4 physical I/O and weld-output lease | P6/P8: retain the cell I/O owner; qualify independent expiry, stale-ON fencing and physical readback. |
| Gradient absolute anchors and turn alignment | P4/P8: Go persistence with C++ coordinate reconciliation, bound to qualified drive/config/home evidence; explicit re-home when unavailable. |

Each port needs behavior tests in physical units and stated differences from its
source policy. Copying an old bridge or its configuration fields is not evidence
that the underlying behavior survived.

## P0 â€” Freeze contracts, policies and regression evidence

Keep this to the minimum contracts needed for M1/P3. The process/socket and
transport choices and R21 decision table are recorded in [KICKOFF.md](KICKOFF.md).
Complete the remaining transition/measurement details alongside their first tests;
do not hold localized correctness fixes behind a complete API redesign.

- [x] Link each R-ID to its implementation change, a focused regression and the
  result/commit that closes it. Add the successful probes to the existing harness and reproduce R04/R07;
  record a disproved claim explicitly instead of silently dropping it.
- [x] Define the mode/authority transition table, axis-group scope, readiness,
  configuration/home/session generations, plan lifecycle and restart behavior.
  Define accepted versus applied versus completed versus physically observed.
- [x] Specify input-age, Stop-response, cycle-lateness, feedback-age, event-loss
  and memory budgets, with units and measurement points. Host-specific guarantees
  remain unqualified until measured; do not invent latency claims.
- [ ] Choose and document reject/clamp/controlled-halt/fault/disarm behavior for
  invalid input, soft limits, target lead, following error, stale jog, bus/drive
  loss and deadline misses. Keep existing inhibition until any replacement
  stopping behavior is qualified. Normal operator release and emergency action
  must have separate, understandable semantics.
- [x] Name the processes, sockets, sole grant owner, generated-codec rule and
  remote jog transport/clock policy. Decisions are written in KICKOFF and tracked as Built pending review; their
  implementation and measured qualification remain open in P3/P5/P7.
  KICKOFF decisions implemented through P3-08/P7-02/P7-06; docs/CONTRACTS.md records the enforced transitions and budgets (P0-02, P0-03) and the R01-R30 traceability (P0-06).
  docs/CONTRACTS.md section 1 at 663fadaf, every cell with reason string and file:line (P0-02).
  docs/CONTRACTS.md section 2: units, measurement points, enforced constants; every hardware value recorded as not measured (P0-03).
  docs/CONTRACTS.md section 3 maps R01-R30 to commits and named regressions; R07 recorded as disproved for the serialized SDK and resolved for concurrent callers in P5-03 (P0-06).

The policy choice itself is now written: valid overspeed jog is limited with
operator feedback, not disarmed solely for slider overshoot. Its implementation
and qualification remain open; the full condition/recovery table is in KICKOFF.

**Exit:** complete transition/policy tables and regression specifications; no
undefined normal-motion or fault-recovery transition hidden in a default branch.

## P1 â€” Go/C++ foundation and complete Python replacement

Target package/command names below are proposals, not existing functionality.
Run this port after P2/P3 establish corrected behavior using the existing Python
oracle. Compare both harnesses against the same fixed daemon/configuration before
deleting old tests; never change the checker and checked behavior together without
the original regression result. Retaining this oracle does not defer the final
Go/C++-only requirement.

| Existing Python ownership | Go/C++ replacement |
|---|---|
| `python/rtcore/config.py` and run/validate entrypoints | Go configuration package and `rtctl compile/validate/run`; one versioned native configuration artifact. |
| `python/rtcore/client.py`, `protocol.py` | Complete Go native SDK, generated C++/Go codecs and a C++ client where needed. |
| `python/rtcore/control.py`, `control_cli.py` | Go application SDK/`rtctl` and C++ application client; one public control contract. Go SDK `sdk/control` and `rtctl` control commands merged b81fcfc3 (P1-05); Python retained until P1-10; C++ client is P1-06. |
| Original `tools/generate_protocol.py` (removed in M1) | `tools/protocolgen` now provides Go generation/drift checks and the settled Start/Discard codec; complete schemas and generated surfaces remain P1/P3/P5. |
| `tools/commission.py` | Go commissioning/inventory commands; separate read-only inspection from explicit writes. |
| `tools/verify-standalone.py`, `tools/review_simulation.py` | Go standalone/package and composed simulation runners. (both removed 2026-09-17 by the bloat review; CI runs the make targets directly) `tools/standaloneverify` and `tools/reviewsim` (merged 211ebe53, P1-08) exist; Python files retained until P1-10. |
| `tests/test_config.py`, `test_runtime.py`, `test_control.py` and active review probes | Go config/process/transport/controller integration tests and C++ state/policy/fault-injection tests. |
| `motion-server/v2/tools/native_rt_execute.py` | Actual C++ consumer integration plus Go CLI for operator/testing use; remove the shim. |
| Make/CI/host installer references, bytecode and caches | Updated build/install/docs; remove Python launchers, imports, package trees and generated caches. |

- [ ] Inventory all tracked and packaged dependencies, including the new
  motion-server control path. Unrelated Rosie planners are outside this language
  migration; they may still produce the supported plan format.
- [x] Generate all commands, handshake/layout metadata, status, events and
  results from the canonical schema for each wire format. Specify alignment/endian/atomic publication rules,
  reject incompatible versions and test C++/Go golden bytes and malformed input.
  Generate settled slices incrementally with P3/P5; extend the M1 Go generator
  foundation for existing and settled contracts. No hand-packed second codec.
- [x] Have Go compile a strict canonical configuration artifact and C++ verify
  its digest and effective fields before master access. Remove the competing
  command-line/private-line configuration routes and vendor-biased defaults.
  Missing, duplicate or unknown critical fields must not silently change motion.
- [x] Separate machine semantics (axes, units, profiles/PDOs, gearing, limits,
  home policy) from deployment identity (host/kernel/NIC/CPU). Bind handles,
  sessions and home evidence to the identities they actually depend on. A digest
  is an integrity identifier, not authentication.
- [x] Switch build, CI, installation, examples and clients to replacements;
  remove every owned `.py`, Python package/cache and Python invocation in scope.
  Partial at 45cb6d31 (P1-10): the Makefile test, test-go, test-sanitize and test-tsan targets and the rtctl compiler tests no longer invoke Python (compiler outputs are pinned to captured goldens under tools/rtctl/testdata/parity). Twelve owned Python files remain tracked: python/rtcore (7 files), the three frozen oracle files under tests/, tools/commission.py and tools/review_simulation.py; host/install.sh, host/run-core, host/generate-control-env.sh and the copied-tree verifier still reference python3. The Python-absent build proof is not recorded.
  Merged 2590b2fd with fix-forward b842dbd8 (P1-10, agent pyremoval2): the twelve Python files are deleted, protocolgen emits Go and C++ only, host scripts and the installer run through rtctl (rtctl run execs the daemon), and make check-python-absent scans tracked files against one allowlist. Merged d41b184d with fix-forward 73a16846 (agent debloat): the Go test fixtures compile through the built rtctl. Gate with python3 absent from PATH (a bin directory of symlinks to the build's tools only; command -v python3 returned nothing): on 73a16846 make control, make test-go (19 packages green, 2 allowlisted skips) and make test-sanitize (33) exit 0; on 72a745f0 (comment-only change) make test exit 0 (native, oracle, host, install, PKI, package and lifecycle checks) and make check-python-absent passed with zero hits; the only two failures in the first run were an openssl symlink missing from the orchestrator's restricted PATH and the orchestrator's own provenance comment. rt-core/build/orchestrator-gate.log.
- [x] Demonstrate old/new harness parity on the same fixed daemon/configuration
  after P2/P3. Preserve independent assertions and all baseline regressions;
  remove the old oracle only after parity evidence is recorded.
  Merged 45cb6d31 (P1-09, agent pyremoval): every one of the 38 frozen Python oracle cases has a Go counterpart in ipcclient/oracle and adapters/rosie/control/oracle; two paired sessions on the same daemon and configuration are recorded in tests/parity/oracle-parity-e64483e3.json and oracle-parity-92357e11.json; the frozen Python files are unchanged and stay until P1-10.
  Merged e62d1dd6 (P1-01): all 38 native 1.6 layouts and tables generated for C++ (376 size/offset assertions), Go (typed frame decoders) and Python; hand-written structs removed; 38 golden fixtures; pinned 1.3 compatibility retained. The public HTTP/JSON schema and generator remain P5-01.
  Merged 7f49a4a0 with fix 0995a778 (P1-02): rtctl compile emits machine.identity and deployment.identity; the daemon recomputes the machine digest from its effective options and profiles under identical canonical rules and refuses to start naming the first differing field; Python compiler parity retained; fault-matrix digest row un-skipped.
  Merged 7f49a4a0 (P1-03): deployment identity hashed separately; a deployment-only change leaves the machine digest unchanged; grants and coordinate evidence bind to machine_sha256.

**Exit:** old/new harness parity on the same corrected baseline, then copy only
`rt-core` and build, validate configuration, run simulation,
prepare/execute plans, mock commissioning and run all suites with Python absent
from PATH. Source/package/dependency scans find no owned Python requirement.
The Python motion-server shim deletion is coordinated with its P6 replacement.

## P2 â€” Correct execution and bus semantics

First implementation milestone M1: use the existing Python regression harness.
Extract only the start gate, per-axis output policy and checked bus seam needed
by these fixes. Broader P4/P5 restructuring waits for green regressions.

- [x] Require fresh per-axis Operation Enabled, correct mode, verified config,
  valid home where required, authority and healthy bus state before advancing a
  plan clock. Define timeout/rejection and readiness loss during execution.
  Define endpoint settling/completion against fresh feedback and configured
  tolerance/timeout; elapsed plan time alone must not report successful motion.
- [x] Introduce typed results at the bus boundary. Check send, receive and
  relevant DC/queue operations; latch and publish appropriate faults. A Stop
  submission acknowledgement must follow a successful inhibited submission;
  failure/unknown delivery must never be returned as successful delivery.
- [x] Reproduce disabled backdrive/sag, then handle target history and coordinate
  uncertainty on reset/re-arm without a target jump, repeated stale-target fault
  or implicit home restoration. Never resume a pre-fault trajectory.
- [x] Correct supported legacy STOP results and prepared-handle abort behavior;
  preserve urgent cancellation independent of helper work and RT deadlines.
- [x] Extract the readiness/output/transition rules needed for these tests now;
  correctness work must not wait for a cosmetic rewrite of the whole daemon.
- [x] Delete the dead CLI drive-config/default-seeding route after checking
  callers. Retire the unused legacy setpoint payload with an explicit ABI check;
  preserve the latest-value primitive for P3's new jog payload. Keep baseline
  fixtures capable of testing the old ABI separately.
- [x] Add Prepared-only Discard and transactional adapter replacement; reclaim
  unpublished/superseded handles, reject active disposal, and report ambiguous
  cleanup outcomes without advertising invalid preparations.
  Done software-side: continuous-feedback re-arm (M1) and the A6 multi-turn register as the qualified continuous coordinate source with compiler support, merge 4643e70a (P2-05). Bench notes: duplicate 0x2040:0x21 TxPDO mapping and the `_unverified` coordinate placeholders remain to measure.

**REV-01 follow-ups before P2 verification:**

- [x] Resolve P2-05 for wrapped feedback with continuous commands using qualified
  coordinate anchors and physical observation bounds. Cover residual error,
  observed wraps, disabled displacement, ambiguous gaps and overflow. A command
  speed cap is not evidence about unpowered backdrive. Brakes remain separate.
- [ ] Resolve R29/P2-14 before accepting per-axis feedback as fresh or advancing
  measured-turn history: current domain exchange, receive and WKC evidence;
  stale/partial/failed exchange tests and an honest simulated input image.
- [x] Document and prove the existing native idle bus-fault policy and explicit
  reset/Home recovery, including distinct initial incomplete-WKC startup behavior.
- [ ] Expose and test that recovery contract through both public application
  clients. The new tests exercise native simulation access, not a public ResetFault API.
- [ ] Finish endpoint integration cases: mixed-axis excursions around per-axis
  deadlines and warning status with zero mapped error versus a nonzero mapped
  error. The documented contract has no dwell or hysteresis; a nonzero mapped
  error remains fail-closed without qualified profile severity interpretation.
- [x] Retain exact baseline patch/probes/logs and a pinned Go native 1.3 wire
  fixture. Fresh replay reproduces six behavioral failures and six candidate
  passes plus a healthy motion/Stop control on both. The historical six/eight
  test hashes are reconciled; frozen Python assertions are distinguished from
  adapted SDK/compiler code. Independent review remains required.
- [x] Protect all 16 native test executables from caller -DNDEBUG; verify a real
  negative assertion; enforce pinned IgH package/checked-return headers; directly
  test home_truth_sign invalid values in the native loader.
- [x] Replace the standalone Python runner with Go and pass all canonical suites
  from a copied rt-core: control build, native/Python/host tests and all Go race
  packages. Reject inherited settings that could skip command or test execution.
  Broader CI, review_simulation.py and complete Python removal remain P1/P8 work.
- [x] Preserve REV-01 and its stage verdicts. The follow-up checkpoint enters In
  review. P2-05 remains Building; no shorter-main.cpp or gate-completion claim.
  Merged 4643e70a; regressions TestWrappedDisabledDisplacementUsesObservedContinuousCoordinate, TestCompiledCoordinateProfileRefusesBeyondBound, TestWrappedCoordinateRefusalRequiresExplicitRecovery.

**Exit reopened after REV-01:** preserve the existing passing continuous-feedback,
bus, cancellation and endpoint regressions, resolve the wrapped-feedback case,
and make the baseline evidence and client-visible contracts reproducible. The
three Python test files retain their assertions; their SDK and configuration
adapters have changed across the explicit protocol transitions. P3/G0 and
hardware qualification remain separate requirements.

## P3 â€” Separate jog and bulk end to end

REV-02 grant follow-ups (P3-10) must land before P3-09 review can advance:

- [x] Close and diagnose a failed observation publisher through the accept owner;
  retain identity fencing and RT reader retirement before unmapping.
- [x] Make grant expiry revoke once per event; preserve separate Stop requests.
- [x] Generate shared region geometry and grant policy values; document any
  local-only refusal budgets and the same-UID/PID-namespace deployment constraint.
- [x] Align the wire Revoke operation and API text with actual production use.
- [x] Retain NDEBUG negative controls and obtain review of the committed fix.
  REV-04 advanced P3-10 to Reviewed; TestREV02ObservationPublicationFailureRetiresConnection.
  REV-04; TestREV02ApplicationExpiryAdvancesNativeFenceOnce.
  REV-04; protocol/control.json fast_control geometry and grant policy constants; same-UID constraint replaced by `--adapter-user` admission (peer_admission.hpp, merge 8d439090).
  REV-04.
  REV-03/REV-04; TestNativeAssertionProbeRejectsUnprotectedNDEBUG.


Retain the existing harness to prove these changes before P1 test replacement.
Use KICKOFF's public Unix jog socket and dedicated authenticated remote WSS lane;
only the Go adapter writes the private native latest-value slot. Clock conversion
uses a conservative offset interval and an immutable host-monotonic deadline.

- [x] Provide independent jog ingress through the authenticated transport,
  adapter, Go/C++ clients, local IPC and RT reader. No jog update waits on a
  whole upload, bulk allocator, `operationMu`, `commandMu` or helper FIFO.
- [x] Use a versioned latest-value jog slot with bounded publication/reading.
  Reuse the sound shared-memory primitive, not its obsolete payload blindly.
  Specify single-writer ownership, memory ordering and torn-read handling for
  x86-64 and aarch64. The RT thread reads it directly each cycle.
- [x] Define BeginJog/UpdateJog/EndJog, source sequence/time, session/config/home
  and jog generations, axis group, velocity and expiry. Establish source-clock
  conversion/uncertainty and an ingress monotonic deadline; reject old or future
  inputs. Never stamp freshness at delayed publication or RT dispatch. Renewal
  of control authority does not renew a jog input.
- [x] Keep reliable control bounded and independent of bulk transfer. SDO/home
  work uses a worker with explicit exclusive commissioning state. Conflicting
  jog/Start requests reject promptly rather than executing after a long home.
  Stop, renewal and observation remain responsive while that worker is blocked.
- [x] Complete the commissioning boundary (R28): RT allocates a reservation
  bound to connection/application authority, native generation, configuration
  and home epochs. All axes retain that identity. Stop removes permission
  immediately; an in-flight mailbox call keeps a separate Draining state until
  its matching completion. SetService, home truth and successful completion
  require current permission; cleanup alone cannot report success. Background
  readbacks require applied receipts with captured epochs and retry on rejection.
  Home and Restore completion now require a published PDO receipt covering their
  completed acquisition, within the existing completion deadline. This corrects
  the CI `future_sample` handoff race without changing drive operations or delays.
- [x] Give bulk allocation/decoding/disposal its own worker and capacity. Keep
  one owner of trajectory-store mutation; transfer validated immutable storage
  without a second writer or RT reclamation. Budget dispatcher work/results per
  cycle and publish consumed ring entries incrementally.
- [x] Move background SDO reads off the metrics publisher as well as moving
  Home/service writes off dispatch. A blocked mailbox operation must not make
  observation stale or indirectly block Go command admission. Commissioning
  writes need an explicit dirty-configuration epoch; Home's unchanged-config
  reservation cannot be reused unchanged for configuration writes.
- [x] Mirror the adapter's existing application grant into native enforcement;
  connection heartbeats alone confer no application authority. Negotiate normal
  adapter access separately from explicit simulation/test compatibility; never
  infer a privileged role from an unauthenticated client label. Detach and
  quiesce RT mapping readers before unmapping or reconnecting. Implemented in
  `214770e`, tracked separately as P3-09 In review; checkbox awaits that review.
  Normal adapter role requires
  the native grant; legacy access is explicit and simulation-only. The grant
  lane stays responsive while Home blocks dispatch. Host credential/socket
  qualification, full worker separation and the jog lane remain open.
- [x] Allow inert upload while jogging within admitted resource budgets; enforce
  jog-versus-plan execution exclusion in the RT mode owner. Global Stop or
  revocation invalidates every pending lane and prevents delayed replay.
  Local lane merged 256a3636 (jog.sock datagrams, no operationMu/commandMu/ring/helper on the update path; about 1 ms maximum input age under a 250,000-point upload in simulation). Remote WSS lane on the mutual-TLS listener merged 8e1e27e7 (p99 about 1.2 ms under an HTTPS upload, simulation). P3-01 to P3-08.
  protocol 1.6 `jog_control`/`jog_update`/`jog_observation` seqlock slots in the fast-control region; RT reads each cycle in engine/jog_lane.hpp.
  engine/jog_session.hpp and jog_clock.go: identity, generations, immutable deadlines, conservative remote offset interval; renewal never refreshes input.
  P3-09 Reviewed at REV-04; adapter-role admission with `--adapter-user` merged 8d439090.
  TestJogUpdatesAreNotDelayedByBulkUpload and TestJogModeConflictIsNativeDecision; mode exclusion in the RT mode owner.
  Merged 5fec966c with fixes 15721cf1 (P3-05): dedicated commissioning worker owns every mailbox call with typed jobs and receipts, exclusive state, dirty configuration epoch; Enable/Arm/Prepare/Start/Discard receipts within 50,000,000 ns during a 2 s blocked Home in simulation.
  R28 original job identity merged earlier (1c779684); worker separation, Draining retained until the worker returns and configuration-write reservation merged 5fec966c/15721cf1. Blocked-mailbox acquisition bound remains a P7-04 host measurement.
  Merged 5fec966c: engine/bulk_worker.hpp owns staging with bounded per-cycle publication (kDispatchBudget 64); one owner of store mutation; CAS handoff unchanged. In-place staging without a fourth buffer is in progress (P4-07).
  Merged 5fec966c: publisher performs no SDO operation; startup and absolute-feedback readbacks are worker jobs; observation timestamps advance within 100,000,000 ns during a blocked mailbox.

**Exit:** continuously jog while preparing a 250,000-point plan and measure
source-to-RT age (including p50/p99/max) against the declared budget. Prove expired
50 ms input never applies after a longer delay. Exercise reordering, packet loss,
blocked SDO, ownership changes, disconnect and Stop at each layer. Verify bounded
latency under load, not merely the absence of one mutex.
Include multi-axis Home cancellation, Stop before final Home acknowledgement,
lost reservation/completion receipts, stale truth readback, reconnect while the
old worker drains, and stable observation/result delivery during mailbox/bulk
load. New tests are Go/C++; the original Python oracle remains unchanged.

## P4 â€” Finish native ownership, coordinates and bounded storage

Proceed through individually tested extractions after P2/P3 are green. Do not
combine all of P4 and P5 into a single daemon/API rewrite.

- [x] Make `main.cpp` composition/startup code. Extract bus backends, CiA402 drive
  state, coordinate transforms, immutable plan storage and output policy around
  one cyclic `MachineState`. Helpers publish bounded requests/results; they do
  not mutate competing versions of motion/home/permit state through atomics.
- [x] Define and test logical rad/metre to drive-count conversion, drive-side
  gearing, sign, home offsets, continuous turns, seam crossing and overflow.
  Salvage v4 wrap handling and Gradient absolute-anchor/turn reconciliation as
  profile-independent policies, with raw values and validity retained.
- [x] Qualify absolute restoration per drive/axis; persist atomically with
  machine/drive/home identity and corruption/replacement/restart checks. Axes
  without trustworthy absolute evidence explicitly require re-home. Do not
  retain a generic A6 absolute field or infer validity from a zero value.
- [x] Implement the reason-specific fault/reset/home invalidation matrix and
  reconcile held/commanded/measured state on re-arm. Use measured time durations
  for jog expiry/arrest and qualified Halt/EndJog ramps at every supported cycle
  period. Keep brake/STO responsibilities and required readback explicit.
- [x] Specify position interpolation, optional qd feedforward and endpoint
  behavior. Validate derivative/acceleration/jerk constraints that the machine
  actually declares, or expose unsupported capabilities clearly; do not claim
  acceleration qualification from a speed-only test or add unchecked splines.
- [x] Prefer one active plus one staged plan with explicit ownership transfer
  and no additional full-size staging copy. Retain 250,000-point support within
  declared native/adapter memory limits. Justify any extra resident slots.
  Do not add paged streaming until memory measurements require it and underrun
  behavior is specified and tested.
- [x] Remove per-axis diagnostic ioctls from routine cyclic telemetry, preserving
  required fresh readiness/bus checks with explicit ownership and bounded cost.
  Do not move IgH access to a concurrent worker without a supported access model.
- [x] Replace the optional fast-trace writer with bounded snapshot/event capture;
  preserve required commissioning evidence, then remove its flags/thread/path.
- [x] Implement per-axis brake engage/release sequencing and the hold-on-disable
  policy (R25): who commands the brake on disable, fault, Stop, lease loss and
  re-arm; release and engage delays from the drive profile; no sag on gravity
  axes while disabled. Design reference: GradientOS plan section 4.5. This is a
  G1 prerequisite for any gravity-loaded axis, ahead of the rest of P4.
- Deferred within this phase, each recorded in the tracker with its reopen
  condition: the Yaskawa profile and commissioning (reopen on model/ESI), paged
  plan streaming (reopen only if P4 memory measurement fails the budget) and
  CiA402 modes beyond CSP (reopen on a stated drive or application need).
  Software merged 663fadaf (engine/brake.hpp, motion permission gate, published brake state, FaultBrakeHold now bit 10 after the bit-9 collision fix abe7dbd1). Bench: C05.10/C05.13 readback, relay measurement and a profile-valued gravity sag tolerance remain before any gravity axis is enabled (P8-04/P8-05).
  Done: merges cd5fab9e and fdf1d508 (P4-01); src/main.cpp 158 lines, engine/cycle_machine.hpp MachineState::step with typed control requests, helper/publisher/transport/bring-up/options extracted; independent equivalence reviews; disarm gate regression fixed 1a50f412. Verified needs the bring-up connect test.
  Merged 498b57b0 (P4-04): include/fault_recovery.hpp table per fault bit citing the R21 row, per-axis provenance, fault_persists, drive reset and re-verification only where required; tests/fault_recovery_tests.cpp and the Go recovery matrix. Time-based jog expiry/arrest and qualified ramps are in progress (P4-05).
  Merged 26baaa3b (P4-08): publisher owns ecrt_master_state/ecrt_slave_config_state at 100 ms with the IgH rt_safe annotations and pinned sources cited; queue depth derived (10,800 records) with exact drop counting; rt_max_cycle_work_ns published.
  Merged 26baaa3b (P4-09): engine/cycle_capture.hpp 512-slot bounded capture armed by --capture-cycles, drained to runtime-only JSONL; engine/fast_trace.hpp and its flags deleted; commissioning evidence fields preserved.
  Decision and merge b3b17a7e (P4-07): three resident slots retained by justification (the frozen oracle asserts three Prepared plans; adapter replacement needs two beside one active), staging decodes in place into a claimed slot with no fourth buffer, storage allocated once and prefaulted, kResidentPlanBytes 204,000,168 bytes plus 16,777,216-byte overhead, measured VmHWM 211,382,272 bytes, zero RT page faults over twenty transfers. No paged streaming.
  Merged 43e80c84 (P4-06): include/interpolation_policy.hpp declares linear_position interpolation, qd feedforward, hold-then-settle endpoint, checked position/velocity/acceleration and unsupported jerk; describe carries the declarations. Bench input: the unverified A6 acceleration bound refuses the reference program at J3.
  Merged a40ed4f0 (P4-02): include/coordinate_model.hpp is the single conversion and reconciliation model (sign, gearing, counts per unit, offsets, seam crossing, int32 overflow refused, continuous-turn reconciliation) used by the cycle and output policy; raw values and validity retained; logical position published additively; v4 rotary-seam cases cited as tests. Independent review found and the orchestrator reverted a one-count relaxation of the disabled-reference rule (110f1892).
  Merged a40ed4f0 (P4-03): configuration- and identity-bound anchors persisted atomically; restore_anchor admitted only inhibited and idle through the commissioning worker after the driveâ€™s absolute source agrees within tolerance; corruption, replacement and disagreement refused; axes without evidence require Home. Physical qualification of the A6 register and tolerance is a bench step.

**Exit:** pure state/coordinate/policy tests and injected-bus tests cover drive
variation and recovery. Measurements show bounded RT work, allocation-free cycles,
prefaulted memory and storage lifetime safety under replacement/cancellation.

## P5 â€” Complete the public API and resource lifecycle

Freeze each operation's semantics before extending the single schema and
generating both clients. Keep the adapter as grant owner; the C++ application
client does not introduce a second native authority path.

- [x] Publish one versioned capability/operation contract: Describe,
  Acquire/Renew/Release, Enable/Arm/readiness, Prepare/Discard/Start/Abort,
  Begin/Update/EndJog, Stop, qualified Halt, Home, ResetFault and Status/Events.
  Explicitly reject unimplemented capabilities and retire old ambiguity.
- [x] Define handle states, expiry, consumption, replacement and reclamation.
  Specify whether a rejected Start retains its handle; distinguish cancelling
  active motion from discarding inert data. Release replaced preparations.
- [x] Define idempotence, duplicate request behavior and recovery after a lost
  acknowledgement. Clients must resolve unknown outcomes without re-executing
  motion. Test Stop/renew/expiry/release races, multiple callers and stale
  generations; the new SDK must not depend on accidental old lock ordering.
- [x] Provide coherent measured/commanded logical q/qd, timestamps, validity,
  configuration/home/session/execution generations, readiness reasons, plan
  cursor, buffer health and typed faults. Retain raw drive diagnostics separately.
  Events are bounded, sequenced and explicit about overflow and recovery.
  Merged 794ffccb (P5-04, agent events): per-axis logical status with readiness reasons carried in the daemon feedback snapshot and surfaced by the adapter; a bounded sequenced event stream over /v1/events polling and authenticated SSE with explicit cursor loss (events_dropped); generated Go and C++ event types; Go SDK and rtctl readers. Open before Verified: events are derived from coalesced status snapshots, so transitions faster than the publication cadence collapse into one event; coordinate_invalid and faulted readiness parity is unverified.
- [x] Admit request size/concurrency before expensive decoding/allocation; bound
  generic JSON and `.rdt` bodies, SDK encoding and simultaneous preparations.
  Measure total RSS/locked memory and GC effects, not just native buffer size.
- [x] Make public fault recovery complete without requiring raw native IPC.
  Authenticated Stop remains available after motion authority expires; this
  must not become an unauthenticated network stop endpoint.
- [x] Interim, at G1 rather than G2: expose the existing native fault reset
  through `rt-control` as a reason-agnostic `reset_fault` operation so a latched
  fault on the bench does not require a daemon restart. Documented as interim
  and replaced by the reason-specific matrix above.
  Merged 63a45d55: adapters/rosie/control/handles.go (prepared/started/consumed/discarded/superseded/retired; rejected Start retains the handle; Discard of active is `handle_active`). P5-02.
  Merged 63a45d55: `request_id` idempotence table, renew during Stop as keepalive with `stopping:true`, concurrent renew/Stop/Start/expiry/release races under -race. P5-03; R07 resolved for concurrent callers.
  Merged b03039b7: `reset_fault` admitted only inhibited and idle (P5-07).
  Merged 8b9a1ee7 (P5-05): fence admitted from a 16,384-byte envelope prefix, machine-derived caps, one preparation in flight, disconnect cancels staging; adapter peaks measured in simulation (JSON 274,198,528 bytes, dense 210,554,880 bytes; repeat in the P8-03 soak).
  Merged 498b57b0 (P5-06): reset_fault reports per-bit outcomes, recovery_status lists latched faults with recovery class, authenticated Stop is accepted after grant expiry or revocation without restoring motion authority.
  Merged 5ba4fde5 (P5-01): protocol/application-v1.schema.json (contract_version 1, 27 capabilities with state, 95 reason labels, handle states, rules) generates Go and dependency-free C++ codecs through tools/apigen; dispatch rejects unimplemented capabilities (halt, abort, subscribe_events, readiness) with capability_unimplemented; describe reports the contract version and digest. Events are P5-04; halt is P4-05 qualification on the bench.

  November 2 P5-04 follow-up: optional current-only status skips historical
  handle copying/sorting/serialization; existing full history and all current
  evidence remain unchanged. SDK legacy-server fallback and OLP joint-move
  receipt races are covered separately. Offline performance evidence is not
  target qualification.

**Exit:** Go and C++ consumers can perform the whole lifecycle and recover from
faults/restarts using the documented API. No Python bridge, raw-count coordinate
reconstruction or process-per-command adapter is necessary.

## P6 â€” Integrate the real Rosie stack

- [x] Inventory and migrate motion server, pendant/UI/agent jog producers,
  remote authority/pair owner, status consumers, cell I/O and deployment owners.
  Maintain one session authority; do not copy those owners into `rt-core`.
- [x] Integrate the actual C++ motion server with inert native preparation and
  explicit Start; remove its position-to-velocity follower from the new active
  path. Preserve program identity, lock/lifecycle ordering and cancellation.
  Replace the Python tool; do not count a CLI demonstration as this integration.
  Merged 7bb680e7 (P6-02, agent motionserver, two review rounds): motion-server/v2 executes dense programs through rt-core's public surface via the C++ client with identity-verified upload, explicit start, cancellable observations (Stop within 250 ms), acquisition off the leader lock, torch refusal before upload, typed cleanup uncertainty; velocity follower off the active path, legacy backend selectable, native_rt_execute.py deleted (P1-11); composed tests against the simulation daemon.
- [ ] Bind authenticated remote principals and pair/config/home identity to
  local grants; propagate revoke/disconnect/restart across the real transport.
  Connect jog source timestamps at the pendant/UI boundary, not only locally.
- [ ] Test existing planner outputs, not just a synthetic reference fixture.
  Resolve 1e-3 versus 1e-12 segment-boundary behavior through canonical stationary
  shared endpoints or explicit validated normalization/replanning upstream.
  Any changed samples require validation and new digests/version semantics.
  Never loosen tolerance and introduce invisible bridge motion. Preserve axes,
  local clocks, dwell, digest/identity and process-marker associations.
- [ ] Integrate the generic cell I/O owner with native execution timing, bounded
  marker delivery, independent expiry, Stop-before-ON fencing and real readback.
  Torch intent is not physical output. Continue refusing torch-bearing execution
  until this path is qualified; measure any synchronous-I/O capability.
  2026-10-17 candidate: sixteen named channels, manual tap ON/OFF sessions,
  ordered set/pulse/wait actions and guarded weld arrival markers are under local
  simulator/integration test. No hardware qualification or Verified claim.
  [Candidate record](../../steamdeck/real/v5/docs/IO_WELD_IMPLEMENTATION.md).
- [ ] Define operator feedback for rejection, limit approach, home loss and
  reset. Pause/resume, if exposed, needs a qualified stop and newly validated
  continuation; otherwise report it unsupported. Do not relabel Disarm as Pause.
- [ ] Migrate the jog producers: the Steam Deck pendant, the UI and the agent
  path emit Begin/Update/EndJog frames with source time, clock incarnation and
  sequence over the chosen lane, and their v4 UDP jog emitters leave the new
  control path. P3 builds only the receiving side.
- [ ] Provide the telemetry and alarm successor (R26): a bounded publisher from
  the P5 status/event stream to the subjects existing UIs and supervisors read,
  or migrate those consumers. The P6 inventory decides which.
  rt-core half merged 794ffccb (P6-08): the P5-04 status and event stream is the source the existing readers migrate to; the consumer-side publisher or migration for the reader families in APPLICATION_HANDOFF.md is Rosie-stack work outside rt-core and is not built.
  Inventory merged 44a0ae02 (P6-01): APPLICATION_HANDOFF.md lists 31 producers/consumers, three bridge subjects with 11 live and 5 archive reader families, authority handoff and a seven-step cutover order. Migration itself remains P6-02 to P6-08.
- [ ] Publish one consumer integration profile (P6-09): the exact operation sequence for a
  motion server or supervisor, fields, reasons and consumer responses, with Go and C++
  conformance examples that run against the simulation daemon in CI.
- [ ] One jog path for consumers (P6-10): begin/update/end frames over the lane; the legacy
  velocity jog operation is marked test-only in the schema.
- [ ] Simulation consumers on the public path (P6-11): MuJoCo cellsim and OLP jog simulation
  leave the v4 UDP wire so simulation and hardware exercise the same client path.
- [x] Full-rate diagnostic telemetry (P6-12): every cycle's full capture record into a bounded
  shared-memory ring with a rolling RAM window and triggered dumps; no formatting or I/O on the
  cycle. Restores the GradientOS fast-trace capability the P4-09 cleanup removed.
  Merged 01a99ffb (P6-12, agent telemetryring, two review rounds): prefaulted memfd ring written every cycle with seqlock stamps (10,000 writes measured 11,130,865 ns; work_ns now measures wake through publication, 1775 to 2650 ns), coalesced triggers written by a dedicated SCHED_BATCH thread in bounded chunks with count and byte retention, session-authorised mark_telemetry with a catalogue reason, read-only Go reader over a descriptor handoff, rtctl dump-to-jsonl; ring declared separately in the resident budget.
- [x] Full-rate telemetry publication (P6-13): generated binary batches over the public surface
  and an rt-core NATS subject with declared retention; generated TypeScript client.
  Merged a73e1262 (P6-13, agent telemetrypub): GET /v1/telemetry and its stream carry a generated 312-byte header plus full 1,576-byte records with loss-aware cursors and a 6,455,608-byte bound; decoders generated for Go, C++ and TypeScript (node golden comparison); rosie/rt-core.<host>.telemetry.v1 and the ROSIE_RT_CORE_TELEMETRY stream in the publisher.
- [ ] Reader migration and sunset of the v4-shape compatibility publisher (P6-14).
- [ ] NATS command adapter retirement (P6-15): UI, OLP and agent commands reach rt-core through
  the public API or the motion server's control plane; the v4 bridge command adapter goes.
- [ ] Cartesian jog authority (P6-16): cartesiand produces jog-lane frames or is retired.
- [ ] Path jog lane (P6-19): Cartesian holds stream a joint-space path and rt-core plays it
  along one jerk-limited path speed (protocol 1.14 path_begin, path_samples handshake, path
  speed flag, path_observation; Describe path_jog when jog_jerk is configured; adapter
  jog.sock/WSS ingress; Go SDK PathJogSession; C++ client unsupported). Implemented,
  unverified on hardware: 27c3cdc3 (protocol), a92ed9f8 (native lane), 33074198 (adapter,
  ipcclient, Describe), c9cf06e6 (SDK), 95c19798 (minor 14 tripwires); generator d96410e1,
  69b0af69, ba4a4af8, 766fce69; OLP path hold e2dac4de, 8613722a; level-1 harness 5852525b
  (1165/1167, commanded off-axis <= 0.0000008 mm on the 29 recorded holds). Acceptance needs the Rosie 1430 cell procedure in the
  Cartesian jog design (commanded off-axis <= 0.05 mm, joint jerk <= 150 rad/s^3) and a
  RevPi cycle-cost measurement; capabilities digest re-pin and full package deploy first.
  No pathway keeps using robot/v4/realtime-core; rt-core supersedes it
  entirely, so P8-07 installs rt-core and removes the v4 runtime from the deploy closure.

**Exit:** the actual Rosie consumers run against the new core in composed
simulation, including remote authority and I/O faults, without Python in the new
control path. Physical behavior remains an explicit P8 qualification step.

## P7 â€” Package and operate the standalone component

- [x] Pin supported Linux/kernel/architecture, IgH/toolchain dependencies and
  notices; reproduce source build and relocated package use. Record artifact and
  configuration identities. Keep required master/RT host setup under `rt-core`.
- [x] Generate installation configuration and `control.env`; create intended
  users/groups, writable runtime directories and separate native/application
  socket access. Document any native privilege/capability requirement; give the
  application service only the access it needs.
- [x] Handle stale sockets only after verifying ownership and absence of a live
  owner; reject a competing live daemon. Test clean shutdown, fd/thread ordering,
  crash restart, readiness, daemon incarnation changes and restart fencing.
  Never replay a plan or restore output permission on service restart.
- [ ] Qualify memory locking/prefaulting, CPU/FIFO/IRQ/NIC setup, timing and
  watchdogs on the selected host. WSL simulation is not the live guarantee.
- [x] Replace helper CPU0/CPU1 assumptions with an explicit validated housekeeping
  mask excluding the RT CPU and outside forbidden cpusets. Check affinity errors
  and test different RT CPU selections, restricted cpusets and small hosts.
- [x] Secure the remote listener (R27): decide reuse of the existing principal
  and pair credentials versus an owned TLS surface; provision certificates,
  rotation and revocation; test rejected and expired credentials. No
  unauthenticated listener may reach Stop or jog.
- [x] Test install/upgrade/rollback in isolation, including missing/invalid config,
  permission failures and failed startup. Installing must not silently enable
  hardware. Document diagnostics and recovery through the public surface.
  Merged 4ba700f3 and 8d439090: rosie-rt/rosie-ctl/rosie-rt-clients identities, setgid runtime directory, generated control.env, rootless install test (P7-02).
  Decision: owned mutual TLS (KICKOFF). Merged 8e1e27e7: TLS 1.3, client certificates with rosie-principal/rosie-pair SANs, CRL per handshake, reload on change, host/remote-pki.sh. Open: bind HTTPS control operations to the handshake principal (P6-03).
  Merged 5b3a8761 (P7-01): identified relocatable package with rt_package.json, verify-package.sh, reproducible builds (SOURCE_DATE_EPOCH, prefix map, trimpath), versioned install and rollback mechanism; copied-tree build fallback 48a71025. Live RUNPATH and IgH debug-path reproducibility unqualified.
  Merged 81f444f6 (P7-03): socket_owner decision in daemon and adapter (refused connection plus verified exclusive lock, else preserve the path), competing live owner exits 3, crash recovery, daemon and adapter incarnations, restart fences sessions, handles, jog and idempotence; six real-process Go tests. Live same-master exclusion not exercised in WSL.
  Merged 5cebf9f1 (P7-04): include/housekeeping_cpuset.hpp derives allowed-minus-RT or validates --housekeeping-cpus (fail closed on the RT CPU, out-of-set CPUs, single-CPU hosts); every non-RT thread pinned with live-build failure fatal; rt_cpu, housekeeping_cpus and per-thread affinity_applied published; TestHousekeepingThreadsAvoidRtCpu. Different RT CPU selections and restricted cpusets are covered by the pure tests; a small-host run remains a bench check.
  Merged 699c71eb (P7-05): host/lifecycle-test.sh drives install, upgrade, rollback and refused second rollback through the real scripts and binaries with byte-identical fake-root refusal checks, real startup failures, socket ownership and public recovery; no unit is enabled without --enable.

**Exit:** a relocatable Go/C++ package starts and recovers with the intended
service identities and explicit host prerequisites; no source-checkout or Python
runtime is needed. Hardware-specific host checks are recorded at G1.

## P8 â€” Qualification, full cutover and final cleanup

Tests accompany every phase; this is final qualification, not the first test pass.

- [x] Run C++ unit/policy tests, Go client/controller tests and composed tests
  against the actual Go race-instrumented service and ASan/UBSan native binary.
  Missing required binaries/instrumentation must fail CI, not skip coverage.
  Run C++ ThreadSanitizer on a supported host; record the WSL limitation rather
  than presenting it as a pass. Check cross-process atomics on supported targets.
- [x] Exercise delayed enable, external disabled motion, mode mismatch, bus and
  drive faults, bad feedback, blocked SDO, output lag, concurrent Stop/Start,
  stale identities, corrupt configuration and restart. Assert outcomes in
  absolute units and bounded times against independent expected behavior.
- [ ] Stress repeated 250,000-point prepare/discard/execute, continuous jog
  during upload, cancellation, events and I/O. Record latency, missed deadlines,
  page faults and peak memory over a defined soak; no accumulating slots/leaks.
- [ ] Obtain the actual Linux host/NIC, drive models/ESIs, mechanical scaling,
  home/absolute policy, brakes/external stop chain and qualified limits. These
  external inputs block relevant hardware tests, not the software refactor.
  Obtained 2026-09-13/14 on the nine-axis bench host (P8-04): host (Debian 12 arm64 PREEMPT_RT, IgH 1.6.9), nine ANCTL AS715N A6-EC drives (CoE 22033/22034, SII 0x2ef8), fixed PDO mapping 0x1702/0x1B02, rotation-frame count 0x2040:2B/2D, mailbox 24..47 ms, encoder rest spread 1..3 counts, gear ratio in the drives (1/1 on the bare bench motors), HM35 semantics, no brakes on the bench motors. Still open: the robot cell's brakes and external stop chain, mechanical travel and qualified limits.
- [ ] Follow [BENCH_TESTING.md](BENCH_TESTING.md): read-only inventory, inhibited
  bus checks, one-axis motion/recovery, coordinated motion, then full stack/I/O.
  Record exact build/configuration and measured traces for each result.
  Bench 2026-09-13/14 (P8-05, P8-13, evidence under docs/evidence/): read-only inventory, inhibited bus checks, Home on all nine (method 35), Enable/Arm/hold, single-axis jog through the jog lane, single-axis and nine-axis simultaneous trajectory moves (arrow alignment, 180-degree turns), recovery (fault table, reset, anchor restore across restart) and telemetry marks all recorded on the merged daemon 58b51063; 23 of the 26 public operations verified with hardware in the loop, program upload and the legacy JSON jog outstanding. Full stack and cell I/O (P8-06) not started.
  Follow-up 2026-09-30 UTC: Rosie 1430 description, native six-axis
  simulation and OLP model/target admission are built and tested on ARM64.
  P8-06 physical qualification remains open: actual brake/stop/watchdog inputs,
  fieldbus version migration, native Home and Orin
  motion-planner acceptance. No physical controller cutover or cell-I/O test
  occurred.
- [ ] Switch the active builders/manifests/services as one reviewed deployment;
  prove single master ownership and rollback with outputs inhibited. The old
  deployment remains available for rollback until acceptance, without sharing
  command ownership or being a runtime dependency of the new folder.
  Release line (P8-18): the fleet ships as `rosie.robot-stack.release.v1`
  compositions (`releases/robot-stack/v0.1.0/release.json`, tooling under
  `tools/robot-stack-release`); rt-core replaces the `realtime-core` component
  there, so packaging emits a component archive, checksums and descriptor, and
  the cutover selects it into the cell manifest through the release tooling.
  Robot models (P8-19): gear ratios, directions, travel and Home policy per
  robot model (`rosie_1420_v1`, `rosie_1400_v3`) become checked-in model
  definitions with mechanical sources; cell configurations reference a model
  and the compiler and bring-up refuse disagreement with the model or with the
  drives' readback. Today the only authority chain is drive readback = v4 pair
  record = GradientOS command line, with no mechanical source in the repository.
- [ ] Remove obsolete bridges, compatibility entrypoints and caches from the
  new component/control path. Update API/examples/provenance/build recipes and
  evidence to the final code. Scan the packaged and tracked scope for Python.
- [x] Retire `robot/v4/realtime-core` after the rollback window in MIGRATION.md
  closes: sources, systemd units, the composed-E2E workflow references and the
  root README ownership row.
  Merged 01a2ee4b (P8-11, verified): robot/v4 and the legacy realtime-core selections removed; consumers, deployment admission and tests migrated to rt-core; native resources and release rollback provenance preserved.
- [ ] Consolidate documentation into one current architecture document, the
  implemented API and this plan; move the historical review, evidence files
  and dated checkpoints under a history folder and fix inbound links.
  Merged 537aab7a (P8-02): ipcclient/faultmatrix, 25 rows in 11 groups with absolute ns bounds and CONTRACTS.md cell references; 23 pass, 2 explicit skips for unenforced contracts (digest verification, exhaustive fault naming). Publication allowance is load-sensitive on the WSL host; the soak runs it alone.
  Merged 45d7c5ed (P8-01, P8-10): tools/cidriver (make ci) proves race and ASan/UBSan instrumentation, audits skips against a tracker-id allowlist, records measured step durations and build inputs, probes TSan host support and records unsupported_on_host on WSL, checks cross-process memfd atomics on x86-64. The workflow outside rt-core needs the one-line make -C rt-core ci.

**Exit:** the release gates below pass with recorded evidence. No required
integration is relabelled a future enhancement merely to mark the core complete.

## Release gates and sequence

Work in small reviewable changes on `codex/standalone-rt-core`: P0 minimum
decisions, P2 corrections with the original oracle, P3 separate lanes with that
same oracle, P1 port with demonstrated parity, then incremental P4/P5 and P6/P7,
followed by P8 qualification/cutover. P7's bench prerequisites can be completed
after P2/P3 without waiting for P1. Python removal is a G2 product requirement,
not evidence that drives can be operated safely. M1 alone is not powered readiness.

| Gate | Required evidence | Permitted next step |
|---|---|---|
| G0: corrected software | Minimum P0 contracts, P2/M1 regressions and P3 independent lanes pass using the existing oracle; R01-R06 corrected or resolved by evidence; previous limits/Stop/deadline coverage retained. Python is permitted during transition. | Assess host/bench readiness; not final architecture/API completion. |
| G1: powered bench candidate | G0 plus applicable P7 service/access/restart/affinity and host timing checks; verified actual drive/config/home/stop-chain setup and inhibited bench checks. R07 concurrency is tested for the chosen client. Every enabled motion policy has qualified conservative bench values. Brake sequencing (R25) is implemented and tested for every gravity-loaded axis that will be enabled, and an interim fault-reset operation is reachable from bench tooling. Python absence is not a prerequisite. | Controlled one-axis and fault-recovery qualification, then coordinated tests as capabilities qualify. |
| G2: full Rosie cutover | G1 plus P1 parity and complete Python removal, P4/P5 architecture/API completion, P6 real consumer/authority/I/O integration, P7 package completion and P8 physical/soak/restart/rollback evidence. All R-items closed with evidence; zero Python in the new component/control path/tools/tests. Required absolute restoration or documented per-machine re-home is accepted; final cutover passes. | Leave the accepted runtime selected in active deployment and declare this migration complete. |

Complete G2's software and qualification prerequisites before the cutover
transaction; close G2 only after that transaction and its acceptance checks pass.

Deferred, each carried as a flagged tracker item with its reopen condition
(P4-11 to P4-13): the exact Yaskawa profile/commissioning until hardware details
arrive; additional drive modes, dynamic plugins and paged streaming unless an
actual requirement emerges. Remote authority, actual motion-server handoff, process I/O
for welding, deployment recovery and hardware qualification are **not** deferrals
from full Rosie completion.

## Document ownership

- [ARCHITECTURE.md](ARCHITECTURE.md): current implementation, ownership, data and
  control flows, with citations pinned to a named commit.
- [CONTRACTS.md](CONTRACTS.md): enforced rules with their regressions; retained
  `663fadaf` checkpoint tables below its status header.
- [CELL_IO.md](CELL_IO.md), [CONFIGURATION.md](CONFIGURATION.md),
  [config/README.md](../config/README.md): cell I/O rule, configuration fields and
  the layered configuration tree.
- [KICKOFF.md](KICKOFF.md): process/socket topology, transport/clock and R21 policy
  decisions, first milestone and preserved-oracle ordering.
- tracker.html: work-package stages, role owners, dependencies
  and completion evidence. Update its committed data as each item completes;
  browser edits are local drafts until exported and committed.
- [CONTROL_SURFACE.md](history/CONTROL_SURFACE.md): target vocabulary, lanes and ownership.
- [APPLICATION_API.md](APPLICATION_API.md): current implemented API and its known
  limitations; update with each versioned implementation change.
- [APPLICATION_HANDOFF.md](APPLICATION_HANDOFF.md): actual consumer migration map.
- [MIGRATION.md](MIGRATION.md): gate sequence and cutover/rollback procedure.
- [BENCH_TESTING.md](BENCH_TESTING.md): physical qualification sequence/evidence.
- VALIDATION.md: dated results, coverage and limitations.
- [DATAFLOW.md](DATAFLOW.md): one-page processes, connections, cyclic and on-request data flows, configuration loading and consumer status, as diagrams.
- [ARCHITECTURE_REVIEW.md](history/ARCHITECTURE_REVIEW.md),
  [PROVENANCE.md](PROVENANCE.md): historical review and source/dependency lineage.
A checkbox closes only with a code/document reference and applicable test or
qualification evidence. A reproduced defect requires a regression that fails on
the reviewed baseline and passes on the fix. Keep source-only concerns and
unavailable hardware tests visibly open until resolved.

P8-06 (full physical qualification on a Rosie 1430 cell) stays Backlog.


### P8-42 follow-up: authoritative OLP planning start (September 30, 2026)

OLP/planner lane owner: Codex OLP readiness agent under the commissioning orchestrator. The local candidate binds taught move/dwell requests to fresh authoritative joint positions and native coordinate/incarnation identities, prevents an implicit model-zero approach, and rejects stale capture metadata at API Load and stale/misaligned feedback at desktop Play. Per-cell diagnostic profile directories retain exact compiled pins. Existing P8-42 stage and ownership history remain unchanged: these additional offline checks do not qualify physical commissioning or the complete OLP operator workflow. Implementation commit is pending the orchestrator's combined checkpoint.

### P8-42 follow-up: simulator operator path and refusal recovery (September 30, 2026)

Measured-start base implementation was saved as `1f3387bc2465e0e34a8e0a02dc7bff229d5820c0`. Actual isolated Rosie 1430 computer-use tests then exposed an omitted Go seam-worker `start_state` boundary; the follow-up forwards it without weakening strict unknown-field handling. The tested simulator UI completed Plan/Load/Play for three taught MoveJ nodes from measured J1=0.5Â°, ending at0.5Â°; a deliberate0.1Â° mismatch blocked Play until the explicit Go to plan start action. Re-Home invalidated the old plan at Load before Arm. Fresh replan after that refusal now passes computer use on the final app: old-artifact Load after a simulator incarnation change refused, then Plan generated67samples/0.6s from current measured J1zero without Reset, Home or another reconnect, followed by successful Load/Play endingJ1=0.5Â°. Program-name Enter, verified-model jog caption and restart-required Home affordances are corrected with tests. Existing P8-42 stage/owner/dependencies remain unchanged; no physical acceptance follows from these simulator results. See the matching history entry for exact validation/artifacts.

A subsequent simulator Stopâ†’Plan test reproduced refusal on retained operator_stop despite fresh Home63. The narrow fix requires the acknowledged operator_stop receipt (any existing session must be stopped) plus native StopAckId and explicit grant/armed/enable zero; fifteen admission regressions and full affected Go packages pass. The first Stop fix incorrectly required a session and failed the actual read-only connection; the corrected null-session case passes offline and final computer-use retry remains pending; no changes were made to retained browser-history text without reproducible receipt evidence.

Final appb6e9890f/UIba238d6b simulator computer use passed: sessionless acknowledged Stopâ†’Plan67samples/0.6s; Armâ†’Loadâ†’Play terminal done segment3/sample64 endingJ1=0.5Â°; acknowledged Stop/releaseâ†’Plan33samples/0.3s from that nonzero measured pose with an existing stopped session. Simulator remained disarmed after the final plan. This supersedes pending retry language above without transferring acceptance to hardware. Transient selection/capabilities loss and a startup auto-connect race are tracked as UI follow-ups, not additional changes in this checkpoint.


### Remote jog reply framing â€” October 1, 2026 (offline fix)

The inhibited R4 clock probe completed 493 paired calibration samples and verified Release plus two advancing inactive publications, with no motion commands. Its 20 ms interval-width gate failed: 39 samples exceeded the limit, maximum 48.796 ms; the endpoint relative-rate bound was 37,387 ppb. Wider intervals accumulated delay on the preceding server reply path. Delayed ACK/buffering on the SSH-forwarded TLS route is a hypothesis, not a proven cause.

The adapter now writes each WebSocket header and payload as one complete transport write. A partial or failed write closes the stream without retry or appending another frame. The existing 20 ms write deadline, calibration budgets, authority checks and native motion policy remain unchanged. Focused exact-file Go race tests passed for complete frame encoding at 0/2/125/126/4096 bytes, the unchanged deadline, short writes, transport errors and refusal after closure. Linux adapter integration, deployment and an unchanged before/after calibration probe remain pending; continuous-hold motion is not qualified. P8-06 remains Backlog with owner and dependencies unchanged.

### Qualified inhibited brake-fault recovery — September 30, 2026

The hardcoded brake_hold restart requirement is replaced by explicit inhibited reset after continuous fresh, fixed-frame quiet proof using existing brake transition/held limits and hold+settling dwell. The actual latch is reset to Held; Home is preserved only with coordinate trust. No automatic clear, Arm, brake output, new threshold or physical-feedback claim. Exact trip speed/effective caps are now retained.

### P8-33 / P8-06 repeated-hold preparation â€” September 30, 2026

Additional no-start cause on a Rosie 1430 cell reproduced: the Go SDK consumes a reserved jog socket without promptly replenishing it; the composed simulator fails on hold2. Implementation `963f1c129f1785b92eb74db8687807b982f9ee66` adds keeper wakeup and bounded known-unsent preparation before Begin/input. Six four-hold simulator runs, cancellation, five race runs, existing lifecycle and focused OLP tests pass. Physical motion acceptance is open. P8-33 stays Specified, P8-06 Backlog; owners/dependencies unchanged.

### P8-06 / AMR-712 brake-hold warnings â€” October 1, 2026

Offline source on `brake-warn-not-fault`; not deployed and not physically qualified. Brake-hold detections (opening, closing, settling, held, legacy J1â€“J3 anchor tolerance) are non-latching warnings: no bit 10, safety mask, revoke, Home loss, disarm, Arm refusal, reset or restart. The guard re-anchors at the settled rest; continued travel warns again. Existing `brake_hold_evidence` is published as public per-axis `brake_hold_diagnostic` plus a `brake_warning` event with the operator message naming C05.10 (torque off) or C05.13 (torque on); the OLP UI shows it as a non-blocking warning. Regressions cover the cases recorded on a Rosie 1430 cell: J5 opening âˆ’10922 vs 9102 counts/s at âˆ’8 counts, J4 closing 24 counts at 20479 vs 19660 counts/s, and legacy 17 vs 16 counts â€” each warning only. Schema digest changed; UI and Steam Deck rebuilds are required for deployment. P8-06 stays Backlog with existing owner/dependencies; physical acceptance (warning-only behaviour on a physical cell, repeated Arm after settle warnings) remains open. Linear was not synchronized from this change.

Follow-up working-tree change (offline; not committed, deployed or physically qualified): closing is judged only after torque off. Movement from the inhibit until the drive's hold delay elapses (the recorded 13â€“58 ms, ~17-count servo settling on a Rosie 1430 cell) is recorded as energized peaks and never warned; the torque-off sample re-anchors and closing/settling/held are judged from it on J4â€“J6 transition guards and J1â€“J3 legacy guards, so a torque_off warning means brake take-up (the ~102 ms, 22â€“24-count class). Within one braked period, three consecutive same-direction warnings (unverified initial threshold) become a distinct `slip` warning naming the count and total displacement and asking for brake inspection; a reversal restarts the count and separate stops never accumulate. Acceptance remains physical: a cell's stops must show energized settle recorded without warnings, torque-drop take-up as a C05.10 warning, and no slip warning across normal repeated stops. Schema digest `9d367b32â€¦`; Linear not synchronized.

### Uniform joint-angle brake, hold and restore bounds â€” October 2, 2026 (offline change)

Working tree only; not committed, not deployed, not physically qualified (GitHub issue #158). Nothing is joint-specific: each bound below is one joint-unit value, converted per axis by its gear ratio (131072 Ã— gear / 2Ï€ counts per joint radian) and floored, so no axis admits more than the declared angle. Count and joint-unit forms of the same bound cannot be mixed, and a value that floors below one count on any axis is refused. Each value is the largest previous per-joint equivalent (J4 or J6); none was measured on the loaded arm.

- Held tolerance and encoder/PDO agreement: the A6 profile declares `hold_displacement_tolerance_rad`, `maximum_feedback_error_rad` and `maximum_anchor_error_rad` = 4.2611e-5 rad (the former 16 counts at J4's 18:1), giving 16 counts on J4, 44 on J6 (50:1) and 88 on J1â€“J3/J5 (100:1).
- Start-alignment floor: native `kIndependentEncoderStartAlignmentFloorRad` = 1.332e-4 rad (the former 50 counts at J4).
- Brake guards on all six joints: the Rosie 1430 cell machine configurations declare one machine-level `brake_transition_guard` (`max_displacement_rad` 0.050Â° = 0.0008726646 rad, `max_velocity_rad_per_s` 3.0Â°/s, target stationary 100 ms, settling 100 ms, timeout 500 ms) and one `brake_opening_guard` (0.050Â°, 8.0Â°/s, timeout 500 ms). Natively that is closing 1820 counts / 109226 counts/s on 100:1, 327 / 19660 on J4 and 910 / 54613 on J6; opening 1820 / 291271, 327 / 52428 and 910 / 145635. Robot-bound cells refuse per-axis guard blocks, and a machine-level block must use joint units. Every axis still satisfies the native checks: displacement â‰¥ held tolerance, twice the displacement below the wrap period, and a mapped 32-bit `velocity_actual` (the A6 fixed TxPDO 0x1B04 used by J1â€“J3 already carries it). Brake detections remain warnings.
- Stationary CSP Home precondition (`--native-home-csp-precondition-axis`): its caps are now joint units too (held 4.2611e-5 rad, 0.050Â°, 1.0Â°/s, opening 2.0Â°/s), so any joint qualifies on the same angle instead of only J4 on 16 counts. The same gear-aware held-tolerance check gates the brake-opening contexts, which now apply on all six axes. The canonical opening rate (8.0Â°/s) exceeds this diagnostic's 2.0Â°/s opening cap, so the diagnostic refuses the canonical machine, exactly as it refused the earlier 8Â°/s J4 trial; a commissioning run needs an explicitly narrower opening envelope.
- Manual restore (verified-delta, command 267) has no cap. `maximum_delta_counts` is removed from the compiler (refused by name), native profile, adapter and public schema. An HM35 (v3) anchor restores to any position within the axis hard limits when the full signed command and independent-encoder deltas agree modulo one joint revolution (131072 Ã— gear counts) within the anchor tolerance, 607C is unchanged and the axis is fresh, authorized and inhibited. A wrapped-PDO seam crossing uses the same once-per-revolution reconciliation as the retained-Home restore, and the hard-limit check applies to the reconciled position. Schema/capabilities digest `64dad2af2ad1f3bb721eb28be72c538b0248756a6e2c2f66456e3a57dfc00b8e`; the OLP UI and Steam Deck client must be rebuilt against the regenerated clients.

### P8-06 / AMR-713 HM35 restored to origin/main â€” October 2, 2026

Offline source; not deployed and not physically qualified. Native Home again executes exactly origin/main's vendor-confirmed HM35 transaction from unchanged `config/drives/a6ec.json` (begin mode 6/CW0 then 50 ms; set_mode 6; 60E6=0; 607C=RM/2; 6098=35; CW 6, 7, 15; wait 0x0227/0x2048; CW31; wait 0x9000/0x2000; refresh; restore_mode 8; release). Removed: the Shutdown dwell (`7df4185f`), pretrigger dwell, fixed CW15 target and Home opening/trigger witness (`a94209e0` and later candidates), a cell-specific legacy-order drive profile (`b9e82329`), the CSP precondition (`fa6490fa`) and the opt-in diagnostic mailbox reads (`50ab0fc8`). The read-only wire audit and post-release capture settle remain. Acceptance: the commissioning-worker golden event sequence, rtctl a6ec.json golden, cyclic no-pretrigger-hold check and simulator no-dwell trace pass with all C++ policy suites and the six-/nine-axis simulator Home/Arm in WSL. AMR-713 remains open pending supervised physical HM35 evidence; stage, owner and dependencies are unchanged.

### P8-06 / AMR-712 drive-state supervisor — October 2, 2026

Offline working-tree change on `95fb533d`; not committed, deployed or physically qualified. General protections lost with the brake-warning change move to a per-axis drive-state supervisor that is identical on every axis and independent of brake configuration: enable timeout (500 ms, the previous brake-opening bound), release timeout (release delay + 500 ms), armed drop-out from Operation Enabled without a fault bit, and Arm-stage cancellation each end the Arm attempt with a plain reason in `drive_supervisor` (retryable, no latch, grant/enable selection retained); disarm motion beyond 0.02 rad in the energized window warns. Non-brake guard causes no longer produce brake warnings. Every bound is a machine-level `drive_supervision` config field (one value for all axes; joint units floored per axis by gear in rtctl; in machine identity), set in the Rosie 1430 cell machine configurations with provenance (unverified, issue #158); their configuration identities and the cell manifest pins follow. Machines without the block keep their existing identity and behaviour. Home acquisition keeps the joint-unit held tolerance (`joint_unit_tolerances`); no second stationary value is declared. Benchdrive post-disable verdicts no longer pass clean after a warning. Acceptance remaining: physical enable-timeout, STO drop-out and disarm-motion observations on every axis class; per-joint decision on the brake-guard-only startup alarm reset. Schema digest `201fbe81…`; UI and Steam Deck rebuilds required. P8-06 stays Backlog with existing owner/dependencies. Linear was not synchronized from this change.


2026-10-01 P3-08 follow-up (Codex, Go SDK/OLP): caller-driven two-message clock refresh prevents an otherwise fresh held input from outliving the initial calibration. OLP drops transient SDK write contention without admission/native-deadline renewal. Focused race-detected SDK and OLP regressions pass. Historical stage, owner and dependencies are preserved; this is no G1/G2 promotion. Tested implementation b95ec2b39205745e6d107911789d74acc0ae1243 is preserved in branch history; integration with upstream 749e6ab8 changes documentation only.

2026-10-18 P6-05 candidate follow-up: scoped sensor monitors and continuous taught
weld spans implemented locally; 75 planner/export tests passed and the native
weld-marker clock regression passed three runs without changing its deadline.
Full release gates and O6 hardware test pending. No torch qualification claimed.
See steamdeck/real/v5/docs/IO_WELD_IMPLEMENTATION.md for retained failures and scope.

2026-10-18 P6-05 candidate: native-owned drive-power/armed lamp and arm-edge
alarm, config timing and reserved indicator class. Unit tests passed;
Linux gate, deployment and physical acceptance pending. See CELL_IO.md.


## October 19, 2026: Trace timing follow-up

P2-10 follow-up / #177 / AMR-1191: current-time reference synchronization and a mid-cycle SYNC0 phase passed the affected release suites. Physical Trace qualification remains pending; earlier work-package acceptance does not qualify this change. Controller limits and Home semantics are unchanged.

### 2026-10-19 manual weld-terminal setup tests

`bd322d01c` enables bounded manual ON/OFF of the assigned torch-class output
while motion is disarmed. Native CellIO, composed controller manual-session
and production-refusal checks, and pendant `check-io` passed. Production
weld qualification remains unchanged. Physical output acceptance is pending.
See [CELL_IO.md](CELL_IO.md).
