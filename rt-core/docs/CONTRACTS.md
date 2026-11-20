**Status at `403063dc`.** P4-01 provides the cyclic MachineState and worker split; P4-02/P4-03 add coordinate mapping and anchor restore, P4-04/P5-06 add reason-specific recovery, P5-02/P5-03 add handle lifecycle and request deduplication, and P4-05 adds public `halt` with the shared EndJog/expiry/halt ramp on live and simulation builds. The native protocol is 1.10 and the Python oracle is removed (P1-10). The section "1. Mode and authority transitions" and everything after it is the retained `663fadaf` contract checkpoint: its file:line citations, its protocol 1.6 scope, its 272-byte point arithmetic and its "not enforced (P4-05)" rows describe that source, not this commit. Use [ARCHITECTURE.md](ARCHITECTURE.md) and [APPLICATION_API.md](APPLICATION_API.md) for current behavior. Hardware qualification and final release gates remain separate.

Telemetry dump retention (P6-12) applies separate pair and index-only count
ceilings under one directory byte budget. Retention removes the oldest index-only
outcomes before considering any binary/index pair. Only a newer admitted real
dump may displace a pair; skipped and byte-refused outcomes never do. An index
that cannot fit after all index-only records are removed is refused and increments
the dump-failure counter without creating a partial file. The shared protocol
defines inclusive admission bounds of 529680..9007199254740991 bytes: two
262144-byte header/index reservations plus one 5392-byte maximum-axis V2 record,
through the exact JSON integer ceiling. Existing evidence is preserved if a
reconfigured budget is smaller than its stored pair; new indexes fail closed.

The native protocol generator writes the canonical C++ headers to
`clients/cpp/include/rosie/protocol_generated.hpp` and
`clients/cpp/include/rosie/control_protocol.hpp`, plus the TypeScript
`clients/ts/rt_protocol_generated.ts`. The SDK tree is self-contained
for motion servers and the Deck header bundle. The matching paths in `include/`
forward to those headers; shared include guards permit both paths in one
translation unit. `make check-protocol` checks the SDK outputs and forwarding
headers against `protocol/control.json`.

## A6-EC absolute linear coordinate contract (P8-45)

The 2026-10-01 brake-warning change (section at the end) supersedes both the
older bit-10 row below and the 2026-09-16 correction: a brake-hold detection is a
warning that neither inhibits nor invalidates Home, and bit 10 is never raised.
The 2026-09-16 correction made `brake_hold` inhibit and require restart without
itself invalidating Home. Continue coordinate observation regardless; missing or
contradictory position proof still invalidates Home independently. The guard
compares the fixed last-enabled PDO pose against the profile's displacement
tolerance (A6: 16 counts in its unverified 100000000 ns hold window), not the
saved restore pose. Native `brake_hold_diagnostic` and its log retain the first
trip and distinguish displacement from released-signal availability; timer
states are not physical brake confirmation. `anchor_reason` describes whether
the saved pose can be restored now: a completed trajectory can make it
`anchor_disagrees` while live Home remains valid. Restore still refuses a
different pose. Pending revision/serial readbacks make the startup offer
unavailable instead of reporting drive replacement; acquired mismatches retain
the existing identity refusal. See the [A6 diagnosis](drives/a6ec/README.md#brake-hold-and-home-after-stop).

Since 2026-10-19 every A6-EC axis uses absolute linear mode
`C00.07 = 1`. The robot definition's motor-to-axis ratio is applied in software;
the drive's standard `0x6091:01/02` ratio is pinned `1/1`. HM35 writes
`607C = 0`; the remaining vendor-confirmed transaction and its order do not
change.

The PDO position, command and independent raw multi-turn evidence are continuous
signed count values. Startup, Home completion, retained Home, explicit restore
and verified-delta compare the complete values exactly within their configured
count tolerances. They never reduce an error modulo an output revolution or
choose a nearest equivalent. A disagreement of one or more complete output turns
therefore refuses like any other coordinate contradiction.

The drive profile declares the independent source and its acquisition objects;
A6-EC linear mode uses source identity `raw_multi_turn_encoder|3`. The source
version, tracking mode, software ratio and HM35 policy are coordinate identity,
so every rotation-mode or mode-3 anchor is fenced and each migrated axis needs a
fresh operator HM35. Rotation mode, wrapped commands, wrapped feedback,
batteryless single-turn mode 3, `single_turn_frame_offset` and the public
`feedback_wrap`, `command_wrap` and `anchor_period_counts` fields are retired,
not compatibility options.

Battery absence is a separate sourced hardware fact, not a mode selector. A
profile may name one exact `(0x603F, admitted 0x203F)` warning pair for an axis
whose robot definition declares the battery absent. Only that current pair may
remain motion-ready; it still publishes the warning, disables retained-Home
restoration and requires HM35 after every drive power loss. No wildcard,
stale receipt, Er20.8, Er20.9 or ErA0.1 is tolerated.

Software gates establish the contract but not drive behavior. Release remains
blocked until the single-axis mini bench proves continuous mode-1 feedback,
HM35 `6064 = 0`, multi-turn reversals and Stop at servo rate. Battery-backed
retention and battery-removal refusal are a later qualification. Only after that evidence and owner
approval may a migrated cell receive a full release and operator HM35 on every joint.
See [migration](MIGRATION.md) and the
[A6-EC drive record](drives/a6ec/README.md#absolute-linear-mode-migration-2026-10-19).

## Cell I/O rule (P6-05)

The hardware safety chain acts on drive torque independently of rt-core.
Supervisory contacts are observations; configured fast contacts are cyclic
EtherCAT terminal inputs. Drive mailbox digital inputs cannot substitute for
them. The cell owns physical outputs inside the native exchange. A marker is
intent; command and independent wired input readback remain separately observable.
No current complete exchange means invalid/not-satisfied inputs and an OFF process
image. Physical bus-loss OFF response requires a qualified terminal watchdog.

Stop fences every output OFF in the applying cycle and discards earlier intents.
A fresh grant, explicit `io_arm` and valid OFF readback observed after the fence
cycle are required before ON can follow. `io_disarm` also clears intents. Every ON
has an independent configured expiry, unaffected by
motion completion, adapter progress or authority renewal. Markers attach to exact
source samples and missed delivery bounds fault, with no delayed replay. Multiple
markers for the same output in one exchange fault instead of losing a transition.
Exception (2026-10-12): an output declared `readback: none_unverified` (no
input module fitted) has no independent feedback; it keeps fencing, expiry and
gating but is never observed, so neither its OFF proof nor its disagreement exists
([CELL_IO.md](CELL_IO.md), "Outputs without wired feedback").
Readback disagreement latches `cell_io` fault bit 13 and currently requires restart after
investigation. The one-cycle delivery and three-exchange readback settling bounds
are `_unverified` on hardware. Torch-bearing admission remains refused pending
P6-05 hardware qualification; software tests cannot remove that refusal.


### P6-05 compiled digest verification

These are historical migration measurements. The retired nine-axis bench
snapshot is no longer a deployable machine configuration. Current six-motor bench cells
bind a Rosie 1420 bench machine configuration and inherit position limits,
velocity ratings, coordinate evidence mode and Home policy from the pinned Rosie
1420 definition; current compiled pins are recorded in `config/cells/*.json`.

Measured compiles at `c5ea5128` and after cell I/O integration. All twelve
configurations without an `io` block retain identical compiled SHA-256 bytes.

| Configuration | Before | After |
|---|---|---|
| `config/machines/a6ec-bench.example.json` | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` |
| Retired nine-axis bench snapshot | `e7c9015db26100bd3d0895bc8ba58749569c9420bfd2fb480236b947f78fff5b` | `e7c9015db26100bd3d0895bc8ba58749569c9420bfd2fb480236b947f78fff5b` |
| `config/machines/simulation/simulation-home.json` | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` |
| `config/machines/simulation/simulation-linear.json` | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` |
| `config/machines/simulation/simulation-program.json` | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` |
| `config/machines/simulation/simulation-scalars.json` | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` |
| `config/machines/simulation/simulation.json` | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` |
| `config/machines/bench/nine-motors-1to1-ffA.json` | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` |
| `config/machines/bench/nine-motors-1to1-ffB.json` | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` |
| `config/machines/bench/nine-motors-1to1.json` | `1912df7d55763abc8949d31f826c4a86f9ab93b1915ef6b347f5c599aaad3dbe` | `1912df7d55763abc8949d31f826c4a86f9ab93b1915ef6b347f5c599aaad3dbe` |
| `config/machines/bench/one-motor-1to1.json` | `1ec357d237e66c446492d2fcdacdd0b63b6e5f6ae0f7f1cb9246f6841dd8fd89` | `1ec357d237e66c446492d2fcdacdd0b63b6e5f6ae0f7f1cb9246f6841dd8fd89` |
| `config/machines/bench/rosie1420-six-motors.json` | `f51524f0d0cbc0c1cf2fb923c26b5ba276de2c1a09f0a6cc21a4424f0214db40` | `f51524f0d0cbc0c1cf2fb923c26b5ba276de2c1a09f0a6cc21a4424f0214db40` |

| Public contract | Previous accepted digest | Current generated digest |
|---|---|---|
| Capabilities SHA-256 | `d614edae8fa60ee4cccdefcd8f5e3006ae3154ba4d6f4530026600da7a17ad5f` | `5ee9b7377b8dfc746ab83e508dbe7ae03868ab9a15dbec379efe2dac9f61bbf2` |

## Collision watchdog rule (P4-17)

No collision motion decision is made from a single cycle. Each configured axis
counts consecutive strictly over-threshold torque and following-error samples
independently, with `sustained_cycles` in 2..1000. Equality or a lower sample resets
that quantity; stale feedback breaks both runs. Evaluate only normal armed,
enabled operation with fresh verified PDOs, including enabled hold. Prefer mapped
drive 0x60F4; otherwise use the prior wire target minus current feedback in the
continuous signed count frame. The following-error threshold is
`following_error_counts_max` plus |this cycle's commanded velocity on the axis,
counts/s| x the machine's `tracking_lag_allowance` (see "Speed-scaled tracking
allowance"); without an allowance, or at zero commanded velocity, it is the
static value. The torque threshold never scales.
Torque is the absolute signed 0x6077 per-mille value; signed minima must not overflow.

The only collision escalation is the existing execution-fault Stop fence:
`collision_watchdog` (bit 12, value 4096), affected axis mask, cancelled execution
and inhibited group outputs before submission. This does not assert physical
standstill. There is no automatic reset, logging or allocation in the evaluator.
The publisher logs the latched trip; native/public axis status retains quantity,
consecutive counts, peaks and trip sequence. Explicit ResetFault requires fresh
trustworthy feedback with both quantities at or below threshold (the same scaled
following-error threshold) and never starts motion. Home survives only while the
existing coordinate evidence remains trusted.

An absent configuration block disables the watchdog with a compiler warning naming
the axis. An explicit disable requires a reason. Thresholds and provenance are
declared per axis or inherited as a whole profile block; a cell block overrides
the whole profile block. Missing/invalid fields are refused. Starting values are
unverified; replay of archived feedback is not loaded-robot collision qualification.

Commit described by the retained sections below: `663fadaf`.
This document records enforced behaviour, not intent.

## 1. Mode and authority transitions

| Reading rule | Contract |
|---|---|
| Scope | Native adapter role, protocol 1.6 at this checkpoint (1.10 at the current commit; the 1.6 layouts are retained), with public adapter differences identified separately. Protocol reason names are the literal reasons/jog_reasons/grant_reasons keys in `protocol/control.json:25`. |
| State/event cells | The transition tables are a factored matrix: a state list expands to one cell for every named state. `All` means I, E, R, J, P, C, D, F and S. Each cell includes its guard; false guards reject or leave the state unchanged as stated. Authority, fault and service flags overlap; these labels do not invent a single daemon mode enum. |
| Decision order | Evaluate the envelope, authority and event-specific guards in their source order. A state label alone cannot determine a reason. The first failing guard wins. Start gives an existing motion/service reservation precedence over readiness. `include/motion_readiness.hpp:82`. |
| Results | A = accepted; R = rejected; automatic = no request to accept. `none` is an actual wire reason only when explicitly shown. Socket packets, physical changes and fault bits have no command-reason string. No synthetic reason is assigned to them. |
| Polling observations | Automatic grant reasons below are internal policy results. Published `grant_reason` retains the last processed request reason; expiry is observable through `grant_active`, deadline and native generation, without a new request ticket. `src/main.cpp:5001`. |
| Native command envelope, All | R `invalid_command`, revoke and inhibit for wrong size/flags, nonincreasing sequence, timestamp at/below revoke floor, future timestamp, or age at least the transport lease. This precedes all command-ring events. `src/main.cpp:6975`. |
| Dispatch | Command-ring Home runs in the helper and can delay subsequent ring commands. Jog/grant slots and socket Stop are independent. An entry described as rejected at evaluation is not a bounded response-time promise. `src/main.cpp:6939`. |

| Label | Actual state predicate or observation | Decision source |
|---|---|---|
| I: inhibited/disarmed | `armed=false`, `axis_enable_mask=0`, no service permission. Initial state; also the result of inhibition. This does not imply valid authority, home or feedback. | `src/main.cpp:2311` |
| E: enabled-not-ready | Enable intent exists, but Arm, fresh Operation Enabled/CSP/error-free feedback, coordinate readiness or brake release is missing. Enable admission deliberately does not require Operation Enabled. | `include/motion_readiness.hpp:14` |
| R: ready-idle | Armed, requested axes enabled and ready, valid lease/configuration/home/limits, no active trajectory, jog or commissioning reservation. The ready group is evaluated for the request. | `include/motion_readiness.hpp:87` |
| J: jog-session-open | `JogSessionState.open`; Begin owns axes even when `has_input=false`. A zero vector does not release the reservation. | `engine/jog_session.hpp:81` |
| P: plan-executing | `active_trajectory`, `EXEC_STATE_EXECUTING`; includes endpoint settling. Prepared and queued Start are separate storage states. | `src/main.cpp:4022` |
| C: service/commissioning Active | `CommissioningPhase::Active`; original job identity and reserved axis mask retained. Per-axis service permission may be zero between steps. | `engine/commissioning.hpp:141` |
| D: service/commissioning Draining | Cancelled/failed job retains its reservation, with service permission cleared, until matching Finish. | `engine/commissioning.hpp:109` |
| F: faulted | Latched `safety_fault_mask` and/or `execution_fault_reasons`; `EXEC_STATE_FAULTED`, `EVT_TRAJECTORY_FAULTED`, `motion_done=1`. Can coexist with Draining. | `src/main.cpp:2330` |
| S: stopping | No dedicated native stopping enum. Urgent Stop has an outstanding `stop_request_id` until `stop_ack_id`; shutdown additionally has `g_stop`/`shutdown_active`. Inhibited submission is separate from physical stopping. | `src/main.cpp:4727` |

| Event | States | Allowed/rejected, exact reason and transition | Decision source |
|---|---|---|---|
| acquire | All | A `none` only after binding, valid interval/identity/full group, inactive grant, inhibited outputs and a strictly newer application generation. Acquire does not Enable, Arm, clear faults or make feedback ready. Otherwise use the grant guard table. | `engine/application_grant.hpp:220` |
| renew | All | A `none` for the active matching identity and valid source-fixed interval. Does not extend jog input deadlines. Motion/service state is not an independent renew rejection. R `inactive` for an inactive grant after successful polling; other failures follow the grant guards. | `engine/application_grant.hpp:235` |
| application expiry | All | Automatic `expired` grant reason at `now_ns >= deadline_host_ns`; active becomes inactive and emits one revoke effect. Already inactive polls do not repeat that effect. J closes; motion cancels; C drains. | `engine/application_grant.hpp:108` |
| transport expiry | All | Automatic loss of permit once heartbeat age reaches 250,000,000 ns. No command reason. The valid-to-invalid transition inhibits and fences once while connected. | `src/main.cpp:3729` |
| revoke | All | A `none` for matching retained grant identity with zero origin/deadline/mask, including already inactive/expired identity; active permission is removed. R `invalid` for noncanonical payload; R `identity_mismatch` for another identity. | `engine/application_grant.hpp:188` |
| stop | All | Valid native `RTSTOP02` always revokes/inhibits and records its ticket. No command-result reason. An ACK is sent only after inhibited full-group submission; repeated packets are separate cancellation events. | `src/main.cpp:6102` |
| legacy jog STOP | All | A `none` for STOP or STOP\|QUICK_STOP; global cancellation. R `invalid_command` and revoke for other unsupported flag combinations. No qualified quick-stop trajectory is promised. | `src/main.cpp:7363` |
| connection loss | All | Automatic inhibition, clear connected and fence on its first observer. Detach mapping, shut down socket, join helper, wait for RT retirement, then unmap. No command reason. Active service becomes D. | `src/main.cpp:6051` |
| daemon restart | All | Process-local state is discarded; new daemon starts I with configuration/home verification absent and new random incarnation. No command reason or replay/restore operation exists. | `src/main.cpp:2213` |

| Grant guard, in evaluation order | Rejection/automatic reason | Decision source |
|---|---|---|
| Clock regresses, including a previously latched regression | `clock_regression`; active permission removed; reconnect cannot reset the failed host clock | `engine/application_grant.hpp:97` |
| Context connection absent/different; disconnected; invalid native generation/lease cap/configured mask | `connection_mismatch`; `disconnected`; `invalid`, respectively | `engine/application_grant.hpp:102` |
| Active identity uses old native generation, wrong configured group, expired or invalid interval | `native_generation_mismatch`; `axis_mask_mismatch`; `expired`; `invalid`, respectively | `engine/application_grant.hpp:109` |
| Binding a new authenticated connection while outputs are not inhibited | `not_inhibited`; repeated/zero binding identity is `invalid` | `engine/application_grant.hpp:136` |
| Unknown operation or zero connection/native/application generation/capability | `invalid` | `engine/application_grant.hpp:178` |
| Acquire/Renew request connection, native generation or group differs from context | `connection_mismatch`; `native_generation_mismatch`; `axis_mask_mismatch` | `engine/application_grant.hpp:206` |
| Origin zero/future, deadline not after origin, interval exceeds cap; deadline already reached | `invalid`; `expired`, respectively. Receipt does not restart the interval. | `engine/application_grant.hpp:157` |
| Acquire while active; otherwise not inhibited; otherwise generation at/below floor | `busy`; `not_inhibited`; `stale_generation` | `engine/application_grant.hpp:220` |
| Renew inactive; otherwise wrong retained identity | `inactive`; `identity_mismatch` | `engine/application_grant.hpp:235` |
| RT publication has reserved bits, or same-cycle polling already revoked permission | `invalid`, or the polling failure reason; no grant admitted in that evaluation | `src/main.cpp:3539` |

| Event | States | Allowed/rejected, exact reason and transition | Decision source |
|---|---|---|---|
| enable | I, E, R, J, P | A `none` when `axes_ready(mask)` and reserved=0; replace enable mask. Otherwise R `not_ready`. No motion-mode conflict guard; changing masks during motion can cause a later readiness fault. | `src/main.cpp:7017` |
| enable | C, D, F | R `not_ready` while commissioning is nonidle or safety fault is latched. | `src/main.cpp:2350` |
| enable | S | R `not_ready` after native Stop revoked the grant; a later fresh grant can satisfy normal admission. During shutdown the helper exits, without a response guarantee. | `src/main.cpp:6995` |
| arm=true | I, E, R, J, P | A `none` only for arm=1/reserved=0 and `axes_ready(axis_enable_mask)`; zero mask rejects. Does not wait for Operation Enabled/brake release. Otherwise R `not_ready`. | `src/main.cpp:6998` |
| arm=true | C, D, F, S | R `not_ready` while service, fault or revoked lease blocks `axes_ready`. Shutdown can end dispatch. | `src/main.cpp:2350` |
| arm=false | All | A `none` for canonical payload; revoke, clear motion intent, inhibit. A malformed command is rejected by the envelope or leaves `not_ready`. | `src/main.cpp:7007` |
| begin_jog | I, E, F, S | R `authority_lost` if no valid grant; otherwise R `not_ready` for unarmed/unready group. Previously revoked connection/native generation also gives `authority_lost`. No session opens. | `engine/jog_session.hpp:198` |
| begin_jog | R | A `none` with matching identity, fresh native-reserved generation, ready nonempty group and bounded first-input deadline; opens J without input. Detailed failures below. | `engine/jog_session.hpp:191` |
| begin_jog | J, P, C, D | R `mode_conflict` after identity/authority polling succeeds. Existing reservation wins even for a disjoint requested mask. | `engine/jog_session.hpp:195` |
| jog update | I, E, R, P, C, D, F, S | R `closed` when the local jog session is closed. A closure found earlier in the cycle can separately publish its state reason. No update reopens a session. | `engine/jog_lane.hpp:100` |
| jog update | J | A `none` only for matching identity/generation, increasing source sequence, valid vector and immutable timing; newest admitted input replaces the vector. Otherwise R with the detailed jog reason below; rejected input cannot refresh expiry. | `engine/jog_session.hpp:229` |
| end_jog | J | A `none` for matching current identity and nonzero generation covering the open generation; closes and inhibits. A lower generation can be accepted without closing a newer session. Polling failure can remain in the reason alongside accepted cancellation. | `engine/jog_session.hpp:268` |
| end_jog | I, E, R, P, C, D, F, S | With valid authority/current identity, canonical nonzero generation is accepted and advances the closed-generation floor; no motion takeover. If authority is absent and no session is open, R `authority_lost`. | `engine/jog_lane.hpp:67` |
| jog input expiry | J | Automatic `expired`; closes generation and inhibits, both on cycle polling and again before dispatch. `needs_qualified_halt` is only a flag; a halt/hold ramp is not enforced (P4-05). | `engine/jog_lane.hpp:24` |
| jog input expiry | I, E, R, P, C, D, F, S | No open session: no input-expiry transition or new command reason. | `engine/jog_lane.hpp:25` |
| prepare (Begin/Commit) | I, E, R, J, P | A `none` at Begin and `prepared`/`none` at Commit if `axes_ready`, valid complete upload and storage available. Arm/Operation Enabled and idle motion are not Prepare prerequisites. R `not_ready`, `invalid_trajectory` or `capacity` on those failures. | `src/main.cpp:7289` |
| prepare (Begin/Commit) | C, D, F, S | R `not_ready` when evaluated with service/fault/revoked lease; invalid payload can win first. Helper blocking/shutdown can prevent evaluation. | `src/main.cpp:7293` |
| prepare (Point) | All | Valid ordered point has no per-point result. Invalid identity/order/mask/encoding, position/velocity/range or segment slope gives R `invalid_trajectory`, clears staging and revokes outputs. Preserving the current mode on all bad plans is not enforced (P4-04/P5-02). | `src/main.cpp:7255` |
| start | I, E, F, S | For a found Prepared handle, R `not_ready` when generation/lease/Arm/readiness fails; the acquired candidate is retired. Missing/already-consumed handle is `unknown_handle` before RT evaluation. | `src/main.cpp:3974` |
| start | R | A `none` only after full readiness and first-time=0/within-one-count continuity; enters P. Discontinuity gives R `invalid_trajectory` and latches FaultStartDiscontinuity. | `src/main.cpp:4012` |
| start | J, P, C, D | Found Prepared handle is rejected `mode_conflict` by RT and consumed/retired; no implicit takeover. Missing/not-Prepared handle is `unknown_handle`. | `src/main.cpp:4012` |
| discard | All | A `none` for inert Prepared, absent, retired or other-generation handle. R `mode_conflict` for Ready/Active; R `invalid_command` for malformed/zero handle. Independent of the machine's readiness/authority state after envelope admission. | `src/main.cpp:7313` |
| abort | All | A `none` for canonical payload; id=0 cancels all matching-generation plans, unknown is idempotent. Prepared/Ready retire, Active gets a per-slot cancel flag; cancelled queued Start receives R `not_ready`. Malformed flags give `invalid_command`. | `src/main.cpp:7325` |
| completion | P | Automatic only after endpoint and all requested axes remain fresh/ready/in tolerance on time. Sets COMPLETED/event/done and leaves enabled hold subject to permit; no command reason. Faults take precedence. | `src/main.cpp:4642` |
| completion | I, E, R, J, C, D, F, S | No active trajectory: no plan-completion transition. Home completion uses its own identity/Finish contract below. | `src/main.cpp:4552` |
| home | I, E | A `none` only after a nonzero configured/verified mask, valid lease/bus, disarmed admission, successful Reserve and every axis/Finish succeed. Enter C while working. Otherwise R `not_ready`; home validity is not an admission prerequisite. | `src/main.cpp:7090` |
| home | J | Inhibit and close jog before RT Reserve; retain jog generation floor. Home then uses the same disarmed service admission and eventual `none`/`not_ready` result. | `src/main.cpp:3870` |
| home | R, P | R `not_ready` while armed and no open jog session; no automatic plan-to-Home takeover. | `src/main.cpp:7091` |
| home | C, D, F, S | R `not_ready` if Reserve is busy or permission fails; the helper may still be occupied by the existing Home. Internal `Busy`/`Cancelled` are not public protocol reason keys. | `src/main.cpp:7117` |
| reset_fault (native) | All | Canonical mask zero means all; nonzero is clipped to configured axes. Revoke/inhibit, clear all home validity and fault masks, request reset pulse and clear motion intent. A `none` if resulting mask nonzero; otherwise R `not_ready` despite the preceding revocation/clears. Reset does not start motion. | `src/main.cpp:7065` |
| reset_fault (public) | I, E, F, S | Authorized, newly observed unarmed/no-motion/no-service state admits native reset; mask semantics remain native. Mere enable intent is not explicitly rejected. A submitted reset retires the public session and program. | `adapters/rosie/control/reset_linux.go:47` |
| reset_fault (public) | R, J, P, C, D | R `reset_requires_inhibited`, or `program_already_executing` for plan execution. These are adapter error strings, not native schema reasons. Missing fresh observations also rejects. | `adapters/rosie/control/reset_linux.go:86` |

| Jog guard, after applicable session polling | Exact reason/effect | Decision source |
|---|---|---|
| Host time zero/regresses or clock failure already latched | `clock_reset`; close and inhibit | `engine/jog_session.hpp:157` |
| Lost authority, invalid identity, grant deadline reached | `authority_lost`; close; retain revoked connection/native generation | `engine/jog_session.hpp:164` |
| Clock mapping invalid or changes; other identity changes | `clock_reset`; otherwise `wrong_identity`; close | `engine/jog_session.hpp:172` |
| Existing session loses mode/readiness; input deadline/maximum age reached | `mode_conflict`; `not_ready`; `expired`, respectively; close | `engine/jog_session.hpp:180` |
| Begin identity mismatch; existing motion/service/open jog | `wrong_identity`; `mode_conflict`, respectively | `engine/jog_session.hpp:195` |
| Begin group unready; exhausted floor; zero/not-new generation; invalid first-input interval | `not_ready`; `generation_exhausted`; `invalid_generation`; `invalid_timing` | `engine/jog_session.hpp:201` |
| Wire Begin has nonzero requested generation, reserved bits or unknown operation; malformed End | `invalid_generation` default. Native itself reserves Begin's next generation. | `engine/jog_lane.hpp:63` |
| Update reserved/flags nonzero, nonfinite component, nonzero component outside mask or above configured velocity cap | `invalid_vector`; whole update rejected. Valid overspeed scaling is not enforced (P4-05/P6-06). | `engine/jog_lane.hpp:90` |
| Update identity/generation/sequence/group differs | `wrong_identity`; `invalid_generation`; `stale_sequence`; `invalid_vector`, respectively | `engine/jog_session.hpp:233` |
| Origin before Begin, origin zero/future, deadline reached/over budget/after grant, regressing origin, same origin with extended deadline | `invalid_timing`; lane maps deadline-at/before-now to `expired` | `engine/jog_lane.hpp:108` |
| Update admitted | Stores source sequence/origin/deadline and vector; omitted selected-session axes receive zero input | `engine/jog_session.hpp:256` |
| End identity wrong or generation zero | `wrong_identity` or `invalid_generation`, unless an earlier poll reason already exists | `engine/jog_session.hpp:271` |

| Service event | States | Allowed/rejected and observation | Decision source |
|---|---|---|---|
| Reserve | I, E; J after closure | Require original job/generation/configuration/home identity, authority/current bus, disarmed/no motion and verified requested axes. Increment home epoch once and invalidate requested home mask. Internal None; eventual native command `none` only after successful job. | `engine/commissioning.hpp:128` |
| Reserve | R, P, C, D, F, S | Internal NotReady or Busy; native Home maps failed reservation to `not_ready`. | `engine/commissioning.hpp:129` |
| Service/Release/CompleteHome | C | Matching identity, one axis inside reservation and canonical scalar fields required. CompleteHome requires prior service release and not already completed. Invalid/Cancelled are internal; failed job returns native `not_ready`. | `engine/commissioning.hpp:163` |
| Service/Release/CompleteHome | I, E, R, J, P, D, F, S without matching Active reservation | R internally Cancelled; no late worker step can acquire permission from a successor generation. | `engine/commissioning.hpp:147` |
| Stop/expiry/revoke/fault/shutdown while service Active | C, F+C, S+C | Automatic D; clear service permission immediately. No command reason for the transition; worker eventual Home result is `not_ready`. | `engine/commissioning.hpp:109` |
| FailHome | C | Matching single-axis failure clears service permission and enters D; records fail-home mask. | `engine/commissioning.hpp:183` |
| Finish | C, D, including fault/stopping overlays | Matching original identity releases reservation. `job_completed` only if still Active, all axes completed and no service bits. Cleanup acceptance is not successful Home. | `engine/commissioning.hpp:152` |
| Finish | I, E, R, J, P, F, S without retained reservation | R internally Cancelled; cannot release a different job. | `engine/commissioning.hpp:147` |
| Final Home result | All | `accepted`/`none` iff `all_succeeded`; otherwise `rejected`/`not_ready`. | `src/main.cpp:7181` |

| Fault event (All states; predicate selects applicable cells) | Trigger and non-trigger behaviour | Enforced transition/reason evidence | Decision source |
|---|---|---|---|
| FaultStartDiscontinuity, bit 0 | Otherwise-admissible Start with nonzero first time or first target more than one count from held command. No check/fault on a Start already denied readiness/mode. | F, revoke; Start R `invalid_trajectory` | `src/main.cpp:4014` |
| FaultPositionRate, bit 1 | Armed+enabled output violates elapsed-time count rate with at most one count retained quantization credit; inactive cells reset guard. | F; asynchronous fault bit, no command reason | `include/output_policy.hpp:319` |
| FaultCycleDeadline, bit 2 (wake) | Wake lateness exceeds configured ns budget while armed or service mask nonzero. Disarmed idle lateness alone does not latch this fault. | F, revoke; no command reason | `src/main.cpp:3570` |
| FaultCycleDeadline, bit 2 (end) | Cycle end exceeds next deadline by more than configured budget while armed/service active. Missed cycles are skipped. | F, revoke; no command reason | `src/main.cpp:5067` |
| FaultCycleSleep, bit 3 | Absolute monotonic sleep returns nonzero after EINTR retries, independent of motion state. | F then shutdown, exit code 1; no command reason | `src/main.cpp:5074` |
| FaultTargetLead, bit 4 | Active jog integrator reports invalid target lead; cells without jog do not run this check. | F, revoke; no command reason | `src/main.cpp:4212` |
| FaultFollowingError, bit 5 | Armed+enabled, fresh trustworthy feedback exceeds configured logical error for its configured timeout. Invalid feedback resets this guard and follows readiness/coordinate policy. | F; no command reason | `include/output_policy.hpp:324` |
| FaultCoordinateReference, bit 6 (outside ExecutionFault enum) | Invalid/ambiguous reference at output, or loss of required continuous coordinate evidence. Fresh inactive trustworthy coordinates can reconcile; guessed turns cannot. | F and affected home invalidation; no command reason | `src/main.cpp:4335` |
| FaultDriveReadiness, bit 7 (exchange) | Any current full-group exchange missing after bus readiness was established; initial incomplete startup is distinct. Applies to idle as well as motion. | F; clear all config/home verification, advance configuration epoch, revoke | `src/main.cpp:3644` |
| FaultDriveReadiness, bit 7 (motion) | Armed executing group loses fresh Operation Enabled/CSP/error-free/brake-ready status. No executing group means this predicate is false. | F before sampling; no command reason | `src/main.cpp:3689` |
| FaultBusTransport, bit 8 | Checked application-time/receive/process/domain-state/DC/queue/send failure, including idle. | F; first failure operation/signed code retained; config/home cleared if bus had been ready | `src/main.cpp:2338` |
| FaultBrakeHold, bit 10 (bit 9 at 663fadaf; moved in the merge after the collision below was found) | Superseded 2026-10-01: retained wire bit, never raised. Brake-hold detections are warnings (`brake_hold_diagnostic`, `brake_warning` event). Historically: fresh disabled feedback moving beyond the anchor tolerance during the hold window or on a gravity axis. | None (warning only; no inhibit, Home change or command reason) | `engine/cycle_machine.hpp` brake guard |
| FaultCompletionTimeout, bit 9 (outside ExecutionFault enum) | Executing plan reaches endpoint timeout without on-time fresh/ready/in-tolerance evidence. Nonexecuting cells do not evaluate settling. | F, revoke; no command reason | `src/main.cpp:4643` |
| Bit 9 ambiguity (resolved after 663fadaf) | At 663fadaf FaultBrakeHold was declared `1u << 9`, colliding with FaultCompletionTimeout in the same `execution_fault_reasons` mask. | Corrected in the merge following this document: FaultBrakeHold is `1u << 10` and the header names bit 9 as taken. A unique public fault vocabulary remains P4-04/P5-04. | `include/execution_policy.hpp:21` |
| Bit 9, second declaration | FaultCompletionTimeout is `1u << 9` in `engine/completion.hpp:8`, outside the ExecutionFault enum. | After the correction above the two predicates are distinguishable by bit; the split declaration itself remains a P4-04 cleanup. | `engine/completion.hpp:8` |
| Untyped output fault | Hard logical target/velocity limit, statusword fault bit or axis readiness can set `out.fault` without adding a dedicated reason bit. | Safety mask still inhibits; exhaustive unique fault classification is not enforced (P4-04). | `include/output_policy.hpp:344` |
| All detected faults | Fault discovered after sampling cancels active trajectory/jog, clears queue and publishes FAULTED before submission. C subsequently becomes D. | Fault state wins over completion; all group outputs remain gated | `src/main.cpp:4540` |

Native metrics retain the first failed Home transaction in each axis's `native_home_failure`, including the original job/generation/configuration/Home identity, axis, step, operation, abort code and reason. A cycle deadline while that reservation is active records the typed commissioning cancellation `cycle_deadline` before revocation; draining and cleanup cannot replace it with `cancelled`, and a successor reservation cannot inherit it. `set_mode` changes the cyclic service override through a request/receipt, without a mailbox call; CLOCK_MONOTONIC wake/end lateness still faults the controlled group and inhibits outputs during Home. The program-travel simulator fixture uses a separate 200,000,000 ns scheduling budget, reducing its sensitivity to deadline misses below that allowance; it does not qualify live deadline behavior. The live budget and fault ladder are unchanged. The deterministic cyclic harness in `engine/home_deadline_tests.hpp` instead injects a Home `set_mode` deadline miss and verifies typed cancellation, retained original failure identity, generation fencing and inhibited outputs. It also replays the measured 115,525,612 ns wake lateness against fixed 100,000,000 ns and 200,000,000 ns budgets and verifies that 200,000,001 ns still faults and inhibits at the fixture budget.

The SDK and benchdrive held-Home renewal regressions establish the cadence origin before starting renewal, observe three baseline receipts, and require six successful renewals inside the held response window before release. Their absolute receipt/cadence bound remains 150,000,000 ns, separate from the 200,000,000 ns simulator scheduling budget above. `TestHomeCompletedRenewalStallPendingBaseline` queues a successful renewal before baseline observation resumes; signed timestamp differences reject reversed ordering without unsigned wraparound. The benchdrive regressions also require typed renewal loss, keeper termination and inhibited native outputs before releasing the successful Home response.

| Axis/group rule | Enforced scope | Source |
|---|---|---|
| Configured group | Axes are indexed 0 through num_axes-1; configured mask is `(1u << num_axes)-1`, maximum 16. | `src/main.cpp:2233` |
| Application grant | Acquire/Renew requires exactly the full configured mask; subset grants rejected `axis_mask_mismatch`. | `engine/application_grant.hpp:214` |
| Enable/Arm/Prepare | Nonempty configured subset; valid lease/bus/config/home/limits and CSP observation. No mandatory Operation Enabled at this admission gate. | `src/main.cpp:2350` |
| Start | Every requested axis requires enabled intent and current motion readiness; all requested home-required bits must be valid. A nonrequested axis is not in the per-request readiness mask, but global faults/permit still apply. | `include/motion_readiness.hpp:87` |
| Jog | Begin reserves nonempty ready subset of configured/granted group. Update mask can only narrow it; group ownership persists until closure. | `engine/jog_session.hpp:136` |
| Mode exclusion | One trajectory or jog reservation globally; disjoint-axis concurrent motion owners are not supported. Service conflicts with either. | `include/motion_readiness.hpp:82` |
| Service | Reserve subset; each later service step selects exactly one reserved axis. Draining blocks another reservation. | `engine/commissioning.hpp:163` |
| Bus group | Domain bindings cover the configured mask without overlap. A domain is indivisible; requesting one member queues that domain. | `include/cycle_bus.hpp:96` |
| Final output permit | Whole-group lease/current exchange/configuration gate; any safety fault inhibits every axis. Service is allowed only for a currently permitted Active job. | `src/main.cpp:4661` |
| Endpoint tolerance | Completion compares the continuous signed feedback count to the rounded endpoint count exactly. Tolerance is max(1 count, completion_tolerance times counts_per_unit). No periodic equivalent, measured-velocity or standstill criterion is added. | Current output/completion policy |
| Disabled coordinate reconciliation | Fresh inactive CiA402 feedback and valid coordinates/limits can preload the observed continuous position. Qualified exact coordinate evidence is required; contradictory feedback invalidates Home rather than selecting another turn. | Current output/coordinate-evidence policy |
| Brakes | No brake output is commanded by BrakeMachine. Motion waits for configured release delay and required released feedback; no brake means no additional brake gate. Timer-derived APPLIED/RELEASED is not physical confirmation. | `engine/brake.hpp:33` |
| Home brake gate | Native homing mode 6 holds the start-homing bit off until brake motion permission. | `include/output_policy.hpp:255` |
| Urgent inhibition | Stop/fault/lease loss does not wait for brake hold delay. Qualified engage-before-torque-removal and physical stop/brake validation are not enforced (P4-10/P8-04/P8-05). | `src/main.cpp:2311` |

| Identity/generation | Produced or advanced | Checked/used | Limit or missing enforcement |
|---|---|---|---|
| Configuration SHA-256 | Go compiler hashes canonical identity text. `tools/rtctl/compiler.go:269`. | Fast header digest must equal expected digest in client. `ipcclient/application_grant_linux.go:108`. | Daemon only validates supplied hash text, rather than recomputing effective configuration (P1-02/P1-03). `src/main.cpp:1957`. |
| Configuration epoch | Starts 1; verified epoch starts 0. `src/main.cpp:2222`. | Start requires equality with verified epoch. `include/motion_readiness.hpp:89`. | Bus loss clears verification and increments epoch. No persisted epoch. `src/main.cpp:3644`. |
| Home epoch | Starts 1; successful Reserve increments once per complete job. `engine/commissioning.hpp:144`. | Jog identity and commissioning permission require matching epoch. `engine/commissioning.hpp:94`. | Reset clears home bits but does not itself increment home_epoch; it changes native generation. `src/main.cpp:7065`. |
| Per-axis coordinate epoch | Initially 1; affected Home advances it with exhaustion failing closed. `src/main.cpp:3821`. | Coordinate context invalidates retained evidence when configuration/axis/source identity differs. `include/coordinate_evidence.hpp:143`. | Lost evidence advances affected epoch and faults; untouched axes retain their epoch. `src/main.cpp:4335`. |
| Coordinate receipt/source session | Current PDO observation increments receipt id; UINT64_MAX exhausts. `src/main.cpp:3633`. | Source session uses configuration epoch; source validity is checked separately. `src/main.cpp:3763`. | A new timestamp is not manufactured for stale PDOs. `src/main.cpp:3634`. |
| Public session | Acquire mints 32 random bytes, hex encoded. `adapters/rosie/control/controller.go:136`. | `authorize` requires current session, generation, unexpired lease and not stopping. `adapters/rosie/control/controller.go:203`. | HTTP connection lifetime is not the session lifetime; lost application connectivity is handled by expiry. Remote principal binding remains P6-03/P7-06. |
| Native capability | First 16 bytes of SHA-256(public session), paired with public grant generation. `adapters/rosie/control/controller.go:166`. | Exact identity comparison on renewal/revoke. `engine/application_grant.hpp:34`. | Native mirrors adapter authority; it does not independently authenticate a remote principal. |
| Application grant generation | Acquire increments retained controller generation; exhaustion refuses Acquire. `adapters/rosie/control/controller.go:142`. | Native Acquire requires strictly greater floor; Renew exact retained identity. `engine/application_grant.hpp:229`. | Stop/expiry also advance adapter generation; comprehensive exhaustion handling on every increment is not enforced (P5-03). `adapters/rosie/control/controller.go:396`. |
| Native/control generation | Starts 1; revoke increments and clears admitted application generation. `include/control_policy.hpp:17`. | Permit requires admitted generation equal current before/after read. `include/control_policy.hpp:33`. | Expected-generation revoke fences one observed event once. Unconditional revoke has no wrap guard (P5-03). `include/control_policy.hpp:55`. |
| Jog generation | RT reserves highest+1; closed-through floor retained across connections. `engine/jog_lane.hpp:69`. | Begin requires fresh floor; Update exact current; End advances cancellation floor. `engine/jog_session.hpp:202`. | UINT64_MAX exhaustion fails closed; expired/ended input cannot recreate a session. |
| Jog source sequence | Producer-supplied; RT records only admitted increasing sequence. `engine/jog_session.hpp:237`. | Observation separately reports first-seen sequence/origin even for rejection. `engine/jog_lane.hpp:85`. | Renew never restamps source origin/deadline. |
| Execution identity | Native slot carries id, native generation and Start command sequence. `engine/trajectory_store.hpp:23`. | RT Start compares candidate generation to current. `include/motion_readiness.hpp:87`. | A separate execution generation counter is not enforced (P4-01/P5-02); status uses active_traj_id/active_command_seq. `src/main.cpp:5057`. |
| Command sequence/time floor | Client sequence and monotonic time enter MsgHeader; helper tracks last sequence and revoke floor. `src/main.cpp:6975`. | Pending upload/store invalidated when native generation changes. `src/main.cpp:6989`. | Late RT results older than the connection generation are dropped. `src/main.cpp:6951`. |
| Service job/ticket | Home takes command sequence plus original native/configuration/home identities; RT returns incremented Home identity. `src/main.cpp:7114`. | Every step/Finish requires same identity; truth readback requires current generation/config/home. `src/main.cpp:3919`. | A cancelled worker never adopts a successor generation. |
| Daemon incarnation | Fresh 16-byte getrandom result; startup fails if unavailable. `src/main.cpp:2213`. | Header requires nonzero incarnation; local clock exposes it to producer. `ipcclient/jog_lane_linux.go:48`. | Local datagrams reject different clock incarnation; durable crash/restart qualification remains P7-03. `adapters/rosie/control/jog_linux.go:181`. |
| Connection id | Monotonic process-local counter; refuse exhausted counter before increment. `src/main.cpp:6205`. | Grant context comes from authenticated current mapping; request cannot rebind it. `src/main.cpp:3498`. | Counter may restart only with new daemon incarnation/mapping. |
| Clock mapping generation | Local ABI uses host CLOCK_MONOTONIC mapping 1. `src/main.cpp:3942`. | Jog identity equality checks mapping along with native/config/home/grant identities. `engine/jog_session.hpp:21`. | Remote calibration policy helpers exist; a qualified connected remote jog mapping is not enforced (P3-08/P7-06). |
| Publication sequence/ticket | Independent shared-word publication sequence and command ticket; RT processes only newer values. `engine/jog_lane.hpp:58`. | Grant uses newer publication plus newer ticket. `src/main.cpp:3534`. | Failed grant observation retires connection rather than granting unobservable authority. `src/main.cpp:5023`. |

| Plan handle lifecycle | Native enforcement | Source |
|---|---|---|
| Upload staging | One helper-owned staging vector; new Begin clears old staging. Nonzero caller-provided id, exact mask, 2..250,000 points; monotonic point index/time and finite/range/slope validation. | `src/main.cpp:7235` |
| Free/Retired -> Writing -> Prepared | Move validated vector into one of three slots. Same-generation duplicate id or no free/retired slot returns capacity. No RT copy/allocation. | `engine/trajectory_store.hpp:38` |
| Prepared -> Ready | Exact id/generation required. Save Start sequence; publish Ready. A queued Start has no immediate accepted result. | `engine/trajectory_store.hpp:56` |
| Ready -> Active (candidate) | RT atomically claims a slot then evaluates Start. The storage name Active is not yet proof of motion admission. | `engine/trajectory_store.hpp:81` |
| Candidate -> executing or Retired | Accepted Start transfers pointer, starts native clock, resets settling state and returns accepted/none. Any rejected candidate is retired, including mode conflict/not_ready. | `src/main.cpp:4055` |
| Completion/fault/cancellation -> Retired | RT retires active storage when execution ceases; helper later reclaims/reuses it. | `src/main.cpp:3751` |
| Discard | Retire Prepared only. Ready/Active remains in use. Absent/Retired/other generation is idempotent success. | `engine/trajectory_store.hpp:69` |
| Abort | Retire Prepared/Ready with correlated cancelled Starts; per-slot Active flag prevents ID reuse from cancelling a successor. Zero id means every plan in that generation. | `engine/trajectory_store.hpp:102` |
| Native generation change | Old Prepared/Ready become Retired when helper notices; active is cancelled by RT's generation check. A revoked handle cannot Start through the current generation lookup. | `engine/trajectory_store.hpp:124` |
| Handle reuse/expiry | Consumed id can be reused only by a new preparation. No independent time-to-live for inert slots; authority generation supplies invalidation. | `engine/trajectory_store.hpp:134` |
| Home/configuration binding gap | Slot contains no home/configuration epoch/digest. Same-generation Home does not independently expire every Prepared slot: invalidate only retires different generations unless an id is supplied. Explicit per-handle epoch binding is not enforced (P5-02/P4-02). | `engine/trajectory_store.hpp:128` |
| Replacement | Native does not offer atomic replace. Adapter serializes prepare/dispose/publish and withdraws uncertain publication. | `adapters/rosie/control/lifecycle_linux_test.go:267` |

| Restart/connection behaviour | Enforced behaviour or gap | Source |
|---|---|---|
| Process restart | No restored Arm, enables, service reservation, application grant, jog input or native resident plan. These start from zero/default process-local storage. | `src/main.cpp:2221` |
| Anchor identity / Home restart | Home-valid mask starts zero. Explicit inhibited restoration requires a v3 anchor with per-axis coordinate SHA-256, independent adapter pair ID/revision and observed drive identity (serial when valid), matching tolerance and exact independent encoder/command evidence. Configuration SHA-256 is excluded. Pair mismatches return `anchor_identity_mismatch` before native submission. Coordinate changes or v1/v2 records return `anchor_identity_mismatch` / `IdentityChanged`; status and native refusal logs name the changed component. Preset, speed-cap and limit changes preserve qualified anchors. Every compiled axis has a complete coordinate identity, independent of cell pair metadata. | `include/configuration_identity.hpp`, `engine/commissioning_worker.hpp`, `adapters/rosie/control/anchors/store.go` |
| Reconnect | New native connection and mappings; old generation work is invalidated and old mapping waits for RT retirement before destruction. | `src/main.cpp:6075` |
| No replay | Generation change clears active modes, deadlines, targets and queue. Missed scheduling periods are skipped rather than replayed. | `src/main.cpp:3739` |
| Local producer reconnect | Session/grant, daemon clock incarnation and native jog generation all participate in admission; old datagram cannot create Begin. | `adapters/rosie/control/jog_linux.go:168` |
| Public Stop versus Release | Stop retains session but increments generation and explicitly reacquires native grant after inhibited ACK/writer barriers; still requires Enable/Arm. Release empties session. Native Revoke is not this reacquisition operation. | `adapters/rosie/control/controller.go:431` |
| Restart qualification | Stale socket recovery, durable incarnation history and supported crash/upgrade recovery are not established by random identity alone (P7-03/P7-05). | `src/main.cpp:2213` |

| Definition | Concrete evidence | What it establishes |
|---|---|---|
| Accepted | Correlated `CommandResult.sequence/handle/generation/operation/result/reason`; Start result is emitted by RT after activation. Grant/Jog use correlated ticket and accepted/reason observations. `src/main.cpp:4039`. | Admission at that operation's decision point. Prepare's `prepared` result means inert storage only. |
| Applied | RT status fields plus current `control_generation`, jog `source_sequence/open/input_deadline_host_ns`; accepted vector is reflected in JogObservation. `engine/jog_lane.hpp:129`. | Cyclic state adopted the input. A status timestamp or generation alone does not prove drive delivery. |
| Applied to bus | `BusSubmission.covers(configured_mask)` and matching `stop_ack_id`/`RTSTOPAK` ticket; last submitted targets advance only on successful submission. `src/main.cpp:4720`. | Inhibited group queued and submitted. Command acceptance and calculated target fields alone do not establish this. |
| Completed | `EXEC_STATE_COMPLETED`, `EVT_TRAJECTORY_COMPLETED`, `motion_done=1`, after endpoint settling. `src/main.cpp:4646`. | Current fresh ready position evidence met configured tolerance on time. `motion_done=1` alone also occurs for abort/fault and is insufficient. |
| Completed Home | Matching Finish receipt `job_completed` with all reserved axes completed and service released. `engine/commissioning.hpp:154`. | Home success rather than worker cleanup. |
| Physically observed | Received PDO position/status/error and `pdo_fresh_axis_mask`/per-axis `pdo_observed_time_ns`; these timestamps advance only on current exchange. `src/main.cpp:3627`. | Observed feedback, simulated in current software evidence. Independent physical standstill/brake engagement is not inferred from ACK, completion, timer-derived brake state or requested target. |

## 2. Budgets

| Budget | Unit | Start/end measurement points | Enforced constant/policy | Measured simulation value and measuring regression | Qualification |
|---|---|---|---|---|---|
| Jog input age, source to RT observation | ns | Producer `origin_host_ns` to first `first_observed_host_ns`; `observed_source_sequence` identifies the sample. RT stamps observation before admission. `engine/jog_lane.hpp:85`. | LAN default `max_jog_input_age_ns=250000000`, first-input wait cap also 250000000; compiled cell policy supplies both age/wait ceilings; deadline cannot exceed grant deadline. These are unverified policy caps. `protocol/control.json:15`. | No retained p50/p99/max sample for 663fadaf. TestJogUpdatesAreNotDelayedByBulkUpload collects 20 samples during a 250,000-point upload, prints ages[9]/ages[19]/ages[19], asserts max <=20,000,000 ns and upload >=100,000,000 ns. Thresholds are not measured results. `ipcclient/jog_ingress_linux_test.go:329`. | not measured |
| Stop response, request to inhibited submission | ns; ticket count | Native receive records `stop_request_id`; cyclic gate captures `frame_stop_request`, submits group, sets `stop_ack_id`. A dedicated request/submission timestamp pair is not recorded. `src/main.cpp:4727`. | Request-to-submission maximum is not enforced (P0-03/P3-07/P7-04). Failed/partial queue/send cannot ACK; heartbeat 250000000 ns and application 500000000 ns are expiry caps, not Stop latency bounds. `include/control_policy.hpp:11`. | No retained latency distribution. TestFailedBusSendCannotAcknowledgeStop, TestFailedDomainQueueCannotAcknowledgeStop and TestStopSubmitsAllConfiguredDomains test submission correctness, not a measured response bound. `ipcclient/readiness_bus_linux_test.go:127`. | not measured |
| Cycle lateness | ns; overrun count | `wake_ns-wake_target_ns` -> `rt_last_jitter_ns`/`rt_max_abs_jitter_ns`; separate `end_ns-next_ns` at cycle end. `src/main.cpp:3557`. | Fault only while armed/service-active above `max_cycle_lateness_ns`; CLI requires 0 < budget <250000000 ns. Default 1000000 ns; simulation config 20000000 ns, unverified. `src/main.cpp:1743`. | No retained qualified distribution for this checkpoint. Configured 20000000 ns is a budget, not a measurement. `config/simulation.json:50`. | not measured |
| Feedback age, domain exchange to use | ns; current receipt count | Prior submitted domain receipt -> successful receive/process/state -> `pdo_observed_time_ns` -> readiness/coordinate use in current cycle. No independent wire arrival timestamp. `include/cycle_bus.hpp:143`. | Zero tolerated missed current full-group exchanges after established readiness. A numeric domain-exchange-to-use maximum is not enforced (P0-03/P7-04). `src/main.cpp:3644`. | No retained ns age distribution. TestCycleBusReceiptTimesFreezeDuringFiniteInputFailure verifies frozen timestamps during missing input; tests/cycle_bus_receipt_tests.cpp rejects reused receipts. `ipcclient/cycle_bus_contract_linux_test.go:203`. | not measured |
| Event loss | events; dropped records | Producer write attempt versus read/write ring indices and `RingHeaderV1.dropped`; diagnostics separately count `diagnostics_dropped`. `src/main.cpp:1600`. | Full ring drops new entry and increments counter. Critical result publication failure revokes. No complete ordered-loss/recovery budget is enforced (P5-04/P8-03). `src/main.cpp:6952`. | No retained loss-rate/zero-loss measurement under declared load. Queue capacity alone supplies no loss bound. Diagnostics drop policy is explicit. `engine/diagnostics.hpp:50`. | not measured |
| Memory: resident plans and staging | slots; points; bytes | Helper staging resize, slot vector swap, RT retirement, subsequent helper reuse. RSS peak is not sampled. `engine/trajectory_store.hpp:38`. | Three resident slots, each <=250000 points, plus one staging vector. Point has 16-byte time/mask/flags plus two 16-double arrays: 272 bytes on the declared layout; four full payloads =272000000 bytes, arithmetic only, excluding allocator/Go/copies. `engine/trajectory.hpp:9`. | No measured peak RSS. TestDiscardPreparedHandleIsIdempotentAndReclaimsCapacity covers slot reuse; one-active-plus-one-staged and RSS admission remain P4-07/P5-05. `ipcclient/trajectory_lifecycle_linux_test.go:12`. | not measured |
| Memory: fast-control region | bytes; latest-value slots | Per-connection memfd allocation through acknowledged mapping retirement. `src/main.cpp:6252`. | Exactly 4096 bytes; offsets 128 grant, 256 grant observation, 512 jog control, 768 jog update, 1024 jog observation. No queued jog samples in this region. `protocol/control.json:8`. | Geometry checked by TestApplicationGrantRegionValidation and tests/fast_control_region_tests.cpp; 4096 is allocation geometry, not RSS. `ipcclient/application_grant_linux_test.go:57`. | not measured |
| Memory: IPC rings | entries; bytes | Allocate command/status memfd per connection, then release after helper exits. `src/main.cpp:6229`. | 128 command entries and 1024 status entries, 512 bytes each; payload totals 65536/524288 bytes, plus headers and 4096-byte alignment. `include/ipc_v1.hpp:44`. | No measured resident peak. These are static geometry calculations. | not measured |
| Memory: internal rings | entries | SPSC push/pop occupancy; push fails at capacity. `engine/publication.hpp:13`. | RT results 64; commissioning request/receipt 8 each; truth readback 32. `src/main.cpp:2252`. | Diagnostics queue separately has 64 records. No measured peak occupancy or full-system RSS. `engine/diagnostics.hpp:32`. | not measured |
| Endpoint settling (related execution budget) | ms in config; ns in RT | Native execution origin -> planned endpoint -> first and continuing fresh/ready/in-tolerance observations. `engine/completion.hpp:38`. | Required per-axis completion timeout; simulation example 500 ms. Deadline fixed at planned endpoint, exact-deadline settled sample may win; late first evidence cannot. `config/simulation.json:25`. | TestTrajectorySettlingFaultsPersistentTrackingError and TestTrajectorySettlingTimeoutStartsAtEndpoint cover deadline behaviour; no measured hardware settling value. `ipcclient/settling_linux_test.go:143`. | simulation only |
| Following error (related execution budget) | logical rad/m; ms/ns | First out-of-tolerance trustworthy sample through continuous exceedance at RT. `include/output_policy.hpp:324`. | Per-axis following_error and following_error_timeout; simulation example 100 ms. `config/simulation.json:23`. | tests/axis_output_tests.cpp and tests/policy_tests.cpp test policy boundaries; no retained distribution. | simulation only |
| Application grant lease | ns | Captured origin/deadline to RT poll `now_ns`; receipt never restarts origin. `engine/application_grant.hpp:157`. | LAN default maximum 500000000 ns; compiled cell policy supplies the effective ceiling, full configured group. `protocol/control.json:11`. | TestApplicationGrantOriginAndWaitBudget and TestApplicationGrantExpiryInhibitsMovingSubmittedTrajectory exercise expiry; no request-to-physical-stop measurement. `ipcclient/application_grant_motion_linux_test.go:17`. | simulation only |
| Brake release/hold windows | ns; counts | Fresh OperationEnabled transition -> `now-transition_ns`; fixed disabled position anchor -> observed displacement. `engine/brake.hpp:41`. | Default release/hold 100000000 ns each, explicitly unverified and profile-configurable; more than one count disabled displacement triggers guard. `include/output_policy.hpp:23`. | tests/brake_tests.cpp covers exact boundary times; TestBrakeReleaseGatesStartAndPublishesSequence and TestBrakeSagFaultDuringHoldWindow cover simulated observations. No physical relay/holding measurement. `ipcclient/brake_policy_linux_test.go:115`. | simulation only |

The remote adapter's `TestRemoteJogNotDelayedByHttpsUpload` checks that every update is
admitted before its native input deadline and before the concurrent upload finishes.
It records p50/p99/max latency; its former unqualified 20 ms assertion is no longer a
shared-runner CI gate. This is functional concurrency coverage, not hardware latency qualification.

## 3. R-ID traceability

| Record rule | Meaning |
|---|---|
| Baseline | Findings are R01..R30 in the plan disposition table. Hashes below are implementation commits reachable from 663fadaf, checked with git show --stat. No test was executed for this document. |
| closed by evidence | The specific software defect has retained regression evidence; it does not close a wider tracker package or hardware/release gate. |
| open | Partial code and available regressions are named without inventing coverage or verification for the remaining scope. `none` means no implementing commit or exact Go/C++ regression for that remaining claim. |
| Historical records | P2/M1 records retain baseline failure/candidate pass; later review replay preserves selected original native 1.3 tests. R04 continuous evidence remains distinct from wrapped follow-ups. |

| R-ID | Implementing commits (partial where stated) | Exact regression tests / retained record | Status and remaining owner |
|---|---|---|---|
| R01 | `1b1353a3`, `b615edc2` | TestTrajectoryStartWaitsForOperationEnabled; TestTrajectorySettlingFaultsPersistentTrackingError; TestTrajectorySettlingTimeoutStartsAtEndpoint; tests/motion_readiness_tests.cpp; tests/completion_tests.cpp. P2_CORRECTIONS and history/m1-evidence-2026-09-12.json retain before/after. | closed by evidence for readiness/settling predicates. Bit-9 diagnostic ambiguity remains P4-04/P5-04. |
| R02 | `1b1353a3`, `b615edc2` | TestFailedBusSendCannotAcknowledgeStop; TestFailedDomainQueueCannotAcknowledgeStop; TestFailedSplitDomainQueueCannotAcknowledgeStop; TestReceiveFailureFaultsActiveTrajectory; TestDomainProcessFailureFaultsActiveTrajectory; TestDCSyncFailureFaultsActiveTrajectory; tests/cycle_bus_tests.cpp. | closed by evidence for checked submission and fault-before-completion. P2_CORRECTIONS and retained native 1.3 replay distinguish baseline failures from current API tests. |
| R03 | `1c779684`, `95ba76af`, `237d7baf` | TestJogUpdatesAreNotDelayedByBulkUpload; TestLocalJogMovesWhileCommandWriterLocked; TestExpiredJogUpdateNeverApplies; TestJogModeConflictIsNativeDecision. No retained current p50/p99/max log. | open (P3-07/P3-08): independent local lane exists; retained load measurement and qualified remote ingress remain. Legacy ring jog still exists. |
| R04 | `1b1353a3`; wrapped follow-ups `dd15e34f`, `7e098fda`, `4643e70a` | TestDisabledDisplacementRearmsAtObservedPosition reproduces +2000 continuous disabled counts and three re-arms; TestDisabledDisplacementOutsideLimitsCannotRearm refuses +60000 counts. P2 record retains fail on 895e249/pass on correction. Wrapped follow-up: TestWrappedDisabledDisplacementUsesObservedContinuousCoordinate; TestCompiledCoordinateProfileDisabledDisplacement; TestCompiledCoordinateProfileRefusesBeyondBound; TestWrappedCoordinateRefusalRequiresExplicitRecovery; TestIdleCoordinateSourceLossFencesPreparedWorkBeforeRecovery. | open (P2-05/P4-03/P4-10). Continuous case closed by retained evidence; wrapped code now uses observed qualified source/refusal, rather than guessing turns. Physical A6 source/brake qualification and persistent restore remain separate. |
| R05 | `1b1353a3` | TestLegacyJogStopAcknowledgesGlobalCancellation; TestLegacyJogRejectsUnsupportedStopEncodings; history/p2-evidence-2026-09-12.json. | closed by evidence for legacy result/encoding defect. Qualified Halt/EndJog remains P4-05. |
| R06 | `1b1353a3`, `b615edc2` | TestAbortPreparedHandleCannotStartAndReclaimsStorage; TestAbortActivePreservesOtherPreparedPlanAndAllowsIDReuse; TestDiscardPreparedHandleIsIdempotentAndReclaimsCapacity; TestDiscardActiveIsRejectedWithoutCancellingEitherPlan; TestProgramReplacementReclaimsSupersededNativeHandles; tests/trajectory_store_tests.cpp. | closed by evidence for prepared Abort/Discard/replacement correction. Full public identity/expiry/retry scope remains P5-02/P5-03. |
| R07 | No fix required for serialized Python instance; concurrent-call partial changes `214770eb`, `ccbde013` | Python acquire/renew/stop/release share the same lock; renewal cannot overlap Stop on that instance. `python/rtcore/control.py:51`. Concurrent regressions: TestStopReachesNativeWhileRenewReceiptIsPaused; TestApplicationGrantStopPreventsCachingLateReceipt; TestREV02ApplicationExpiryAdvancesNativeFenceOnce. | disproved with record for the single serialized Python SDK instance, as P0-06 scope records; concurrent callers moved to open P5-03. No assertion that all server/SDK races are disproved. (tracker item P0-06). |
| R08 | none | TestReferenceProgramPreservesAxesClocksAndProcessRequirement is related parsing coverage, not producer-corpus boundary parity. | open (P6-04): 1e-3 dense boundary versus approximately 1e-12 adapter continuity policy/corpus reconciliation. |
| R09 | `7b656f94` (interim public reset only) | TestPublicResetFaultRecoversIdleBusFault; TestPublicResetFaultRefusedWhileArmedOrExecuting; TestPublicResetFaultClearsHomeAndRetiresFence. | open (P4-04/P5-06): public reset exists; native still clears all home validity. Reason-specific recovery is not implemented. |
| R10 | `b615edc2` | TestNativeCompiledProfilesRequireEveryField; TestRetiredGlobalDriveOptionsAreRejected; TestNativeCompiledProfileCorpusStillValidates. | closed by evidence for explicit native descriptors and dead defaults. Canonical artifact verification remains P1-02. |
| R11 | `bb75b47e`, `985b60a4`, `d3d6bb70`, `4be2697f`, `b6294b75` | Arithmetic ramp qualification at all 40 supported periods (250000 ns through 10000000 ns in 250000 ns increments): `TestHaltAllPeriods` in tests/cycle_machine_tests.cpp covers trajectory/jog sources and both signs; `ramp(period, false)` and `ramp(period, true)` in tests/jog_policy_tests.cpp cover EndJog/expiry. `TestHaltElapsedClock` covers elapsed monotonic steps and the fixed bound; `TestHaltCurrentFeedbackLead`, `TestHaltFinalFeedbackLead`, `TestHaltAdmissionCurrentReadiness`, `TestHaltAdmissionCurrentBus` and `TestHaltIdleReceiptOnly` cover the review regressions. | open (P4-05) only for powered-hardware stopping qualification, which remains unverified until the bench session. Software arithmetic qualification is complete; it does not establish physical standstill. |
| R12 | `d6d38341`, `bec54cf9` (deployment/admission subset) | tests/peer_admission_tests.cpp; host/install-test.sh is the existing shell deployment check, not a Go/C++ restart regression. | open (P7-02/P7-03/P7-05): identities/env/access implemented in part; crash/stale-socket/upgrade qualification remains. |
| R13 | `b615edc2`, `1c779684`, `214770eb`, `ccbde013` (extractions) | tests/motion_readiness_tests.cpp; tests/axis_output_tests.cpp; tests/commissioning_tests.cpp; tests/application_grant_tests.cpp. | open (P4-01): tested boundaries exist; coherent single MachineState and orchestration reduction remain. |
| R14 | `b615edc2` (reclamation subset) | TestDiscardPreparedHandleIsIdempotentAndReclaimsCapacity; tests/trajectory_store_tests.cpp. No peak-RSS regression. | open (P4-07/P5-05): three slots plus staging remain; arithmetic is not measured resident memory. |
| R15 | `78276229` (compiler subset) | TestParity in tools/rtctl/parity_test.go; TestCanonicalMetadataAndOrder; TestConfigurationAndBackendMismatchRefused checks supplied identities, not native recomputation. | open (P1-02/P1-03): Go canonical compiler exists; C++ effective artifact digest verification absent. |
| R16 | `b615edc2`, `1c779684`, `214770eb`, `ccbde013`, `95ba76af`, `237d7baf` | TestGeneratedHandleCodecs; TestGeneratedJogCodecs; TestProductionGrantLaneCodecs; TestProductionLocalJogCodec. | open (P1-01/P0-07): more records generated; full wire/public/C++ surface and independent compatibility coverage incomplete. |
| R17 | `1c79f6fe`, `dd15e34f`, `7e098fda` | TestNativeAssertionProbeRejectsUnprotectedNDEBUG; TestHealthyDisarmedSendFailureLatchesFaultUntilExplicitRecovery; TestApplicationGrantExpiryInhibitsMovingSubmittedTrajectory; tests/assertions_enabled_tests.cpp. Retained rev01-followup and review-runtime evidence. | open (P8-01/P8-02/P1-09): assertion protection/replay/canonical runner improved; mandatory CI, complete controller/fault matrix and parity remain. |
| R18 | `ccbde013`, `dd15e34f`, `237d7baf` (observation subsets) | TestCycleBusReceiptTimesFreezeDuringFiniteInputFailure; TestHomePreservesUntouchedAxisCoordinateEpoch; tests/sequenced_snapshot_tests.cpp. | open (P5-04/P4-02): fresh raw PDO/coordinate/jog fields exist; coherent complete logical state and ordered bounded events remain. |
| R19 | none for actual consumer cutover | TestReferenceProgramPreservesAxesClocksAndProcessRequirement covers local adapter input only. No actual-stack cutover regression. | open (P6-02/P6-03/P6-05/P7-05/P8-07). |
| R20 | `b615edc2`, `1c79f6fe`, `78276229` (Go generator/verifier/compiler subsets) | TestParity; TestGeneratedHandleCodecs; original Python oracle remains. No final Python-absence acceptance. | open (P1-09/P1-10/P1-11/P8-08): owned Python still present. |
| R21 | `1b1353a3`, `237d7baf`, `7b656f94` (inhibit/reset subsets) | TestExpiredJogUpdateNeverApplies; TestLocalJogExpiryCannotRestartFromLateUpdate; TestPublicResetFaultClearsHomeAndRetiresFence. | open (P0-04/P4-04/P4-05/P6-06): implemented rejection/inhibit differs from intended overspeed/limit governor and qualified halt. |
| R22 | none | No exact Go/C++ regression for housekeeping cpuset exclusion. | open (P7-04): helper still pins CPUs 0/1. `src/main.cpp:6400`. |
| R23 | none for moving diagnostics off cycle | tests/cycle_bus_tests.cpp verifies bus seam, not ioctl removal or cost qualification. Per-axis query remains. `src/main.cpp:4740`. | open (P4-08/P7-04). |
| R24 | `b615edc2`, `95ba76af`, `237d7baf` (dead ABI/config and new jog subsets) | TestRetiredGlobalDriveOptionsAreRejected; TestRetiredSetpointRegionAbsent; TestRetiredSetpointHeaderRejected; TestProductionLocalJogCodec. | open (P4-09): obsolete configuration/slot removed; fast-trace successor/removal remains. `src/main.cpp:5120`. |
| R25 | `a7c7b749`, `2a7ac7e1`, `663fadaf` (merge adds Go compiler brake emission) | tests/brake_tests.cpp; TestBrakeReleaseGatesStartAndPublishesSequence; TestBrakeSagFaultDuringHoldWindow; TestBrakeMappedFeedbackPreservesStatusword; TestBrakeSagWithoutBrakeRetainsCoordinatePolicy; TestBrakeDefaultsKeepExistingMachinesValid; TestParity. | open (P4-10/P8-04/P8-05): drive-managed timing/motion gate/sag diagnosis and compiler emission implemented; physical brake/disable qualification and unique fault bit remain. `tools/rtctl/compiler.go:195`. |
| R26 | none | No telemetry/alarm successor regression. | open (P6-08): existing consumer migration or successor publisher absent. |
| R27 | none for remote TLS surface | No remote credential rotation/revocation/listener regression. Local peer tests do not establish remote security. | open (P7-06/P6-03). |
| R28 | `1c779684`; source/scalar follow-up `dd15e34f`, `7e098fda` | TestStopDuringMultiAxisHomeCannotStartLaterAxis; TestMultiAxisHomeCompletesWithoutCancellation; TestHomeTruthScalarSignCheckedBeforeNarrowing; tests/commissioning_tests.cpp. history/p3-evidence-2026-09-12.json retains Home cancellation evidence. | closed by evidence for original job identity/late-axis defect. Independent worker separation/acquisition bound remains P3-05. |
| R29 | `ccbde013` | TestCycleBusStartupQueueSuppressionCannotCreateFreshInput; TestCycleBusReceiptTimesFreezeDuringFiniteInputFailure; TestCycleBusRejectsUnqualifiedRoundRobinBeforeStartup; tests/cycle_bus_receipt_tests.cpp. history/review-runtime-evidence-2026-09-12.json retains neutral before/after seam replay and current tests. | closed by evidence for current-domain receipt/full-group correction. Neutral seam replay is not a historical whole-daemon run; physical exchange/recovery qualification remains P7-04. |
| R30 | `ccbde013` | TestREV02ObservationPublicationFailureRetiresConnection; TestREV02PendingHomeFailureRetiresOldRegionOnce; TestREV02ApplicationExpiryAdvancesNativeFenceOnce; tests/connection_failure_tests.cpp. Retained rev02-before and review-runtime after records. | closed by evidence for terminal observation-failure teardown and one fence per event. Blocked mailbox return/acquisition bound remains P3-05/P7-04. |


## Cell link timing contract (remotecell)

The compiler validates `control.link_profile` (`lan` default or `internet`)
and optional `control.max_grant_lease_ns` / `control.max_jog_input_age_ns`.
LAN defaults remain 500000000/250000000 ns; internet ceilings are unverified
3000000000/750000000 ns. Absolute unverified bounds are 10000000000/2000000000 ns.
Lease overrides are positive whole milliseconds. Both ceilings are published in
Describe and enforced from compiled native options. Nondefault effective values
enter machine identity; the original control fields enter configuration and
deployment digests. Omitted defaults do not add fields or change any digest.
Explicit default declarations change configuration/deployment identity through
their presence, while native argv and machine identity remain unchanged.

Acquire requests `requested_lease_ms`; the adapter returns the lesser of request
and cell ceiling as `lease_ms`, retained on renewal. Omitted/zero requests use
500 ms capped by the cell. Client sizing requires three measured round trips
plus cadence, with renewal at most one third of the effective lease on a
dedicated connection. A ceiling cannot force a local client to request a long
lease. An insufficient receipt fails closed with typed `control_session_stale`.

Longer granted leases mean a longer worst-case unattended-stop delay after
operator link loss. Longer input ages mean longer stale velocity application
before immutable expiry and the existing jog expiry ramp. Remote clock mapping
uses the earliest possible capture origin from a causal offset interval and
drift; transport delay is never treated as a new capture or removed as offset.
The cell age also bounds first-input wait and WSS idle expiry after admission. Remote producers first upgrade with `jog_generation=0` and complete both calibration replies, then acknowledge the second host timestamp in a WSS `begin` message. The native Begin capture cutoff uses the post-calibration Begin sample's earliest host bound, while its first-input deadline is sampled at receipt; setup has no native jog generation or first-input deadline. Pre-Begin binary input and repeated Begin fail closed. This acknowledgement carries no input sample: capture timestamps and deadlines remain immutable. Preparation remains fenced by grant cancellation and a bounded transport timeout. Older nonzero-generation upgrades retain their existing contract. Independent clock
qualification remains mandatory. These unverified presets do not qualify hardware.

The following configuration SHA-256 values were compiled before changes at
1192d4b8 and after adding internet to the three named cells. All twelve cell
configurations recompile. The four robot definitions those cells pin now live in
`robot_description/robots/<model_id>/rtcore_definition.json` (not standalone
cell configurations); the merge into the descriptions re-pinned every robot-bound
machine (2026-09-20). The nine default cells retain byte-identical
configuration digests and default behavior.

| Cell under config/ | Before | After |
|---|---|---|
| a6ec-bench.example.json | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` |
| Retired nine-axis bench snapshot | `f877ef4d5730af4eecead13c77aeba3c1b1683010810c8e650e6ca197b80cc01` | `e7c9015db26100bd3d0895bc8ba58749569c9420bfd2fb480236b947f78fff5b` |
| simulation-home.json | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` |
| simulation-linear.json | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` |
| simulation-program.json | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` |
| simulation-scalars.json | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` |
| simulation.json | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` |
| bench/nine-motors-1to1-ffA.json | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` |
| bench/nine-motors-1to1-ffB.json | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` |
| bench/nine-motors-1to1.json | `18e94648fda6ee442f769cc2f8b9777bdfa1e0ae41ce7cda31c2b93f24d7c160` | `1912df7d55763abc8949d31f826c4a86f9ab93b1915ef6b347f5c599aaad3dbe` |
| bench/one-motor-1to1.json | `d40d0d3a167bf0ef21c81234bd52ce5c9e2978de9f463e20a6506b7ae4688d6f` | `1ec357d237e66c446492d2fcdacdd0b63b6e5f6ae0f7f1cb9246f6841dd8fd89` (values of 2026-09-17: 3000 rev/min cap, bare-motor watchdog) |
| bench/rosie1420-six-motors.json | `7d196bc776e0904d82f088d411b1d553bf016348444d7c8d09de7058314fdf80` | `f51524f0d0cbc0c1cf2fb923c26b5ba276de2c1a09f0a6cc21a4424f0214db40` (internet preset and the per-axis brake override composed) |

Native Begin validates the capture cutoff against the cell input-age ceiling and
the first-input deadline against admission time plus the cell wait ceiling, each
independently and still capped by the grant. This separates clock uncertainty from
setup allowance. Input admission retains the original origin/deadline validation
and rejects captures before the calibrated cutoff; renewal extends neither input.

P8-28 adds the compiled robot/resource and verified drive description types, the
`resource` observation capability and `resource_unknown` refusal to the public
schema. The thirteen configuration pins in the table above remain unchanged;
resource files and the compiled resource manifest are artefact contents, not
inputs to the machine configuration digest. The Go, C++ and TypeScript contract
identities are emitted by `make generate-api`.


## Opt-in brake transition candidate (AMR-712, 2026-09-30)

The preceding default brake guard remains unchanged when the machine axis omits `brake_transition_guard`. The local opt-in candidate separates closing/settling from permanent held monitoring: a qualified successfully submitted stationary target may define a fixed corridor with the last enabled feedback, padded by the existing held tolerance. Explicit caps never expand that corridor. Inactive Home gets only the conservative held-tolerance corridor; authorized frame reconciliation does not imply a physically held brake. Urgent inhibition remains immediate. Fixed-anchor cumulative sag, unknown/reopened brake feedback, stale/future/frozen data, unexplained frame changes and finite settling timeout fail closed. Held16counts remains unchanged. This candidate has offline native/process qualification only; the assembled robot's brake/relay transfer is not yet accepted.

## Target lead is a warning (2026-10-06)

A jog or Halt/EndJog target leading feedback by more than `max_target_lead`
no longer faults, disarms or revokes the grant. `JogAxis::advance` admits the
step and reports it; the cycle publishes one `TargetLeadEvidence` per excursion
and the status publisher logs `target_lead severity=warning ... action=none`
when it starts and `target_lead_cleared ... peak_deg` when it returns inside
the bound. Reason: the command-to-feedback lag of a loaded arm at its limited
speeds is unmeasured (#158), and the fault disarmed a Rosie 1430 cell mid-jog
(J2, 0.35 degree bound) with nothing wrong at the robot. Tracking
stays bounded by the output policy's FollowingError fault (`following_error`
held for `following_error_timeout_ns`; one value for every joint, e.g.
0.02 rad / 100 ms on a Rosie 1430 cell).
`FaultTargetLead` (bit 4) stays in the wire vocabulary: an invalid integrator
input, an overrun Halt bound and a ramp that cannot begin still raise it.
Regressions: `TestHaltCurrentFeedbackLead`, `TestHaltFinalFeedbackLead`,
`TestHaltGuardStopPrecedence` cause 0, `TestLimitHoldTrackingGuards/target_lead`,
`TestTargetLeadWarnsAndKeepsHome`. No public schema field changed.

## Speed-scaled tracking allowance (2026-10-06)

A servo trails its commanded target by a near-constant time, so its tracking
error is about |commanded velocity| x lag and a static bound is crossed by
ordinary fast motion. Stop dumps of a loaded Rosie 1430 arm measured ~27 ms on every
axis: J3's 0.35 degree `max_target_lead` was crossed at ~13 deg/s and the
0.02 rad `following_error` at ~42 deg/s (J6 tripped at ~60 deg/s, 0.0289 rad).

- Configuration, not source. The machine declares one optional
  `tracking_lag_allowance` block, `{"lag_allowance_ms": N}`, N an integer
  1..1000 (1000 is an unverified sanity cap). rtctl refuses null, zero,
  negative, fractional, exponent, string, over-cap and per-axis declarations
  and emits `tracking_lag_allowance=N` on every axis line; native admission
  repeats the 1..1000 check and requires the same value on every axis. The line
  is part of machine identity and stays out of deployment identity. A machine
  without the block compiles no line and keeps the static bounds; other
  machines' identities are unchanged.
- Effective bound = static bound + |commanded velocity| x allowance
  (`speed_scaled_bound`, `engine/jog.hpp`). The velocity is the one the cycle
  already commands on that axis, in that axis's own frame (counts/s for the
  jog/Halt integrator, rad/s for the output policy), so the time needs no
  per-joint number or gear conversion.
- Target lead (warning): `JogAxis::advance` judges each step against the bound
  scaled by that step's commanded velocity, for jogs and the Halt/EndJog ramp
  (`HaltRamp::advance`, `JogLane::advance`). `TargetLeadEvidence` and the
  `target_lead` journal line now carry the scaled `limit_counts` and the
  commanded `velocity_counts_per_s`/`velocity_deg_s`.
- Following error (fault): `evaluate_axis_output` compares the error against
  `following_error` scaled by this cycle's commanded velocity feedforward.
  `following_error_timeout_ns` is unchanged and an error beyond the scaled
  bound for that long still raises FaultFollowingError. With no commanded
  velocity (holding) the static bound applies.
- Collision watchdog following error (fault, 2026-11-03): `CollisionWatchdog::observe`
  compares the drive's 0x60F4 (or the commanded error) against
  `following_error_counts_max` scaled by the same allowance and by the commanded
  velocity the output policy judged this cycle (`AxisOutputDecision::target_velocity`,
  counts/s, the frame of the counts it compares). `sustained_cycles` is unchanged;
  an error beyond the scaled bound for that many cycles still trips, a NaN bound
  counts as over, ResetFault's clear reads the same bound and the torque
  threshold is unchanged. Measured on the single-axis mini bench (A6-EC, 2^17
  counts/rev, 1:1) before the change: Steam Deck joint jogs at 421397 counts/s
  (~20 rad/s) tripped quantity=following_error after 10 cycles, within ~200 ms,
  with peak errors of 12897, 12937, 13026 and 13593 counts against the static
  12000, the drive trailing ~31-32 ms. The bench machine declares 50 ms
  (unverified there): its bound at that speed is 12000 + 21070 = 33070 counts.
  The archived-evidence replay (`cycle_machine_tests --collision-replay`) still
  judges the static bound.
- Not public: Describe still reports the static `max_target_lead` and
  `following_error`; no application schema or capabilities digest changed.
- A Rosie 1430 cell declares 50 ms, a chosen margin over the measured ~27 ms,
  unverified (issue #158). At 60 deg/s the following-error bound is
  0.02 + 1.0472 x 0.05 = 0.0724 rad; J3's lead bound at 42 deg/s is 2.45 deg.

Regressions: `jog_policy_tests` TestTrackingLagAllowanceLead,
`axis_output_tests` TestTrackingLagAllowanceFollowingError,
`cycle_machine_tests` TestCollisionFollowingErrorSpeedScaled and
TestCollisionFollowingErrorCycleVelocity,
`configuration_identity_tests` (parse, identity, uniformity), rtctl
`TestTrackingLagAllowanceAdmissionAndIdentity`.

## Drive speed feedforward (2026-10-06)

The ~27 ms lag is the drive's own position loop, not rt-core. In CSP (mode 8)
the A6-EC reads only 0x607A from the fixed 0x1702 RxPDO: 0x60FF target velocity
is CSV/PV-only (chapter 11, 60FFh), and the external feedforward object 0x60B1
is in no fixed RxPDO preset (ESI 0x1701..0x1705), so the velocity rt-core
writes never reaches the speed loop. Speed feedforward is selected by C01.13
(0x2001:14; 0 none, factory default; 1 derivative of the position reference;
2 model tracking; 5 0x60B1), scaled by C01.14 (0x2001:15, 0.1 %, factory 0) and
filtered by C01.15 (0x2001:16, Hz, factory 318). With 0/0 the loop is pure P at
C01.00 = 400 (40 rad/s, read back on a bench drive), i.e. a steady lag
of 1/Kp = 25 ms plus transport. A Rosie 1430 stop dump fits tracking
error = 26.5-27.0 ms x commanded velocity on J1/J2/J3/J5/J6, flat across 2-70
deg/s (velocity alone explains >99 % of the variance; the acceleration term is
small and negative, a first-order-lag signature, not feedforward).

- Feedforward is configuration. The A6-EC startup schema gains
  `a6ec_speed_feedforward_filter_hz` (C01.15, 5..16000) beside the existing
  source/percentage keys; robot-bound cells may set all three. The first
  Rosie 1430 trial set C01.13 = 1, C01.14 = 500, C01.15 = 318 on all six axes
  (one dimensionless value; written and read back at every startup). 500 is a
  bench bare-motor J6 value and the manual's tuning example; expected lag
  about 14 ms. Unverified on the loaded arm (#158).
- The feedforward keys are not part of any coordinate identity, so Home
  anchors recorded under the previous configuration stay valid.
- Measured on a Rosie 1430 cell (multi-joint jogs and one Move-to):
  lag 15.1-15.5 ms on J1/J2/J3/J5/J6 (was 26.8-27.1). Unverified as a release
  value until a single-joint check (#158).
- Drives keep what was last written (C13.10 parameter storage, factory 1). A
  configuration that omits these keys does not revert them: turning
  feedforward off needs explicit `source: 0` and `percentage: 0` on every
  axis (`docs/drives/a6ec/README.md`, "Persistence and reverting").

Regressions: rtctl `TestA6FeedforwardStartup` (filter range and readback
descriptor).
Superseded in part on 2026-10-07: these keys are now per-joint tuning keys of
the A6-EC tuning catalog ("Per-joint drive tuning" below).

## Per-joint drive tuning and observe-only reads (2026-10-07)

Since 2026-10-07 every A6-EC servo tuning parameter is
under software control and set per joint; safety bounds (following error,
target lead, tracking lag allowance, timeouts, Home/anchor tolerances, limits)
stay one joint-unit value for every joint.

- Catalog. `config/drives/a6ec.tuning.json` declares, for vendor 0x400000 /
  product 0x715, every tunable object: key, manual code, CoE object, type,
  manual range (raw units), allowed values, factory default, unit and scale,
  modification class, manual citation, class (`tuning` or `uniform`) and
  access (`read_write` or observe-only `read`). Ninety-three entries cover
  C00.04-C00.06, C01 and C02 (`docs/drives/a6ec/README.md`, "Tuning a joint").
  Matched to a profile by EtherCAT identity, so no profile bytes changed. rtctl
  refuses a whole catalog with an entry in the CoE communication area
  (0x1000-0x1FFF: PDO mapping/assignment) or the CiA 402 profile (0x6000-
  0x9FFF: homing, limits, gearing), on an object of its own `forbidden` list
  (C00.00/01/07, C03 limits, C04 I/O, C05 stop, C06 protection, C07/F30
  auto-tuning, C10 homing and 0x2010:19/1A gearing, C13 storage, R/U groups),
  a duplicate object, a label that does not match index/subindex, or a range,
  default or class that does not validate. At compile, a key on an object the
  profile itself owns (startup schema, Home transaction, absolute feedback, PDO
  entries) is refused.
- Writes. Per-axis `startup` accepts the profile's own startup schema keys and
  any `read_write` catalog key, as a raw drive integer or `{"value": x,
  "unit": "<catalog unit>"}` with x / scale whole (6.2 Hz at 0.1 Hz is 62;
  6.25 Hz is refused). Range and allowed values are checked in raw units.
  `tuning` keys may differ between axes or appear on one axis only; a
  `uniform` key must be one value on every axis. Robot-bound cells may set
  catalog keys only; gearing, direction and tracking mode stay derived from the
  robot definition. At most 32 writes per axis and 128 per machine (native
  `kMaxStartupSdosPerAxis`, `kMaxStartupSdoDescriptors`). Rendered exactly as
  before (`key|type|index|sub|value`), written in PREOP with the drive disabled,
  read back after the mailbox settle and required for `config_verified_mask`.
- Observe. `observe` lists catalog keys rt-core only reads: per axis, or once
  at machine level for every axis (an axis's own list replaces it; keys an
  axis writes are skipped there). Up to 64 per axis; an axis's own list may not
  name a key it writes. Compiled to `startup_observe=key|type|index|sub;...`.
  The commissioning worker reads one entry per readback tick, after the startup
  readback, with no mapping verification pending and only while disarmed with
  no axis enabled, through `read_sdo_scalar_axis` (admitted, identity checked,
  counted in flight, so the shutdown drain covers it). Journal:
  `EtherCAT startup observe begin entries=N`, then per entry
  `EtherCAT startup observe axis=.. slave_pos=.. key=.. index=0x.... sub=0x.. value=..`
  or, after three refused attempts, `WARNING: EtherCAT startup observe failed
  axis=.. key=.. abort=0x........ attempts=3`, then `EtherCAT startup observe
  complete reads=R failed=F`. Observe reads never write, are not service-write
  targets and never affect `config_verified_mask`. They restart with every
  startup epoch.
- Identity. Values are machine JSON and so configuration identity; the
  rendered lines are machine identity (native mirrors `startup_observe` only
  when present). A machine that uses a non-`pre_catalog` key or any observe list
  also binds the catalog entries it names into the configuration identity, so a
  catalog edit cannot change compiled output under an unchanged digest. A
  machine that uses only the three speed-feedforward keys keeps its earlier
  configuration identity (earlier bench and cell pins are unchanged). Machine-level `observe` is machine, not deployment, identity.
  Compiled `tuning.json` lists the catalog's `tuning` keys for the
  configuration-only deploy and is in no identity. No protocol or Describe
  change: the capabilities digest is unchanged.
- Configuration-only deploy (`realtime-core config`): a startup key in the new
  compile's `tuning.json` may differ between axes. Since 2026-10-24
  it may also be removed to retain the current drive-held value without future
  startup writes. Other startup-key removals remain refused; missing or malformed
  catalog data grants no exception. The deploy report distinguishes removal
  from writing a value; it restores neither defaults nor earlier tuning, and
  RAM-only trials remain volatile. An older configuration that writes a removed
  key manages it again. All readiness, drift and Home checks remain. A release that
  predates the catalog writes no `tuning.json` and its rtctl refuses `observe`
  and non-feedforward cell tuning keys, so the first deploy is a full rt-core
  package.
- A cell's per-joint values (which keys each joint writes, its observe list)
  are commissioning data in its machine configuration, each with its
  `startup_source` measurement note. Adding an observe list or a tuning write
  changes configuration and machine identity, not coordinate identity, so
  Home anchors stay valid. A machine-level observe list of 64 entries fills
  the native per-axis capacity (`kMaxStartupObservesPerAxis`); a joint that
  writes one of them reads it back instead.

Regressions: rtctl `TestA6TuningCatalogContents`,
`TestTuningCatalogRefusesHomingGearingAndSafetyObjects`,
`TestTuningPerAxisValues`, `TestTuningUniformClassStaysOneValue`,
`TestTuningObserveList`, `TestTuningPerAxisCapacity`,
`TestTuningPreCatalogIdentityAndTuningFile`, `TestRobotCellTuningAndObserve`;
native `configuration_identity_tests` (mixed widths, observe parse, identity,
capacities) and `commissioning_worker_tests`
`TestStartupObserveReadsWithoutVerification`; deploy
`TestRTCoreConfigEvaluateAllowsPerJointTuning` and the refusal table.
Superseded in part on 2026-10-09: startup objects are written only when the
drive holds a different value ("Startup write on change and RAM-only trials").

## Startup write on change and RAM-only trials (2026-10-09)

The A6-EC stores every SDO-written parameter in EEPROM while C13.10
(0x2013:11, "EtherCAT parameter storage") is 1, its factory value: one
drive-wide mode, effective immediately, not a per-write flag (manual chapter
11; ESI "Update function code values written via communication to EEPROM";
vendor notes 2026-04-15 item 10: 607C and the HM35 result persist only with
1). STEPPERONLINE (vendor answer of 2026-10-09): EEPROM endurance about
1e5-1e6 writes, ALF1.3 is the too-frequent-write alarm, whether an identical
write is skipped by the firmware is unconfirmed; read back, write only on
change, write commissioning trials to RAM only and store finalized values
once. Until this change rt-core queued every startup object at every start
(7-10 per axis on a tuned Rosie 1430 cell), and IgH replays the queue at every reconfiguration.

- Write on change (every machine, no configuration change). Before queueing,
  the bring-up reads each of the axis's startup objects in PREOP
  (`upload_sdo_read`). Identical: not queued. Different: queued as before. A
  failed, aborted or wrong-width read: queued (never taken as a match). A
  startup key declared `effective: "power_cycle"` must already read back at
  its configured value; rt-core refuses startup rather than treating a queued
  write as effective before the required drive power cycle. A
  steady-state start writes nothing. Verification is unchanged: every
  descriptor is still read back after startup and gates
  `config_verified_mask`, written or skipped. Journal per object:
  `EtherCAT config phase=slave_config_sdo ... value=V ok prior=P|unread
  storage=stored|ram_only` (written) or `... value=V prior=V skipped=identical`,
  and per axis `EtherCAT startup_sdo_summary axis=.. slave_pos=.. written=N
  skipped=M persisted=K ram_only=R unread=U storage_window=0|1` (axes with no
  startup object log nothing). `persisted` counts writes made outside a
  RAM-only window, i.e. stored by an A6-EC with C13.10 = 1. Cost: one mailbox
  upload per object per start, in PREOP before activation (unmeasured on
  hardware).
- Storage verification (opt-in, machine-level `verify_parameter_storage:
  true`). Requires the drive's tuning catalog to declare `parameter_storage`
  (`config/drives/a6ec.tuning.json`: key `a6ec_parameter_storage`, C13.10
  0x2013:11 u16, store 1, RAM-only 0; a schema field, not a tuning key, so it
  changes no machine that does not opt in). rtctl puts it first in every
  axis's `startup_sdos` at the store value and emits
  `parameter_storage=a6ec_parameter_storage|0`. It is read, written first
  only if it differs (so the stored writes after it are stored), and read
  back like any startup object: a drive found with C13.10 = 0 is set to 1
  and the axis is not configuration-verified until it reads 1. It is never a
  service-write target. Status marks it `"storage":"mode"`.
- RAM-only trials (opt-in, per axis `startup_trial: [keys]`, requires
  `verify_parameter_storage`). Keys must be tuning-catalog keys this axis
  writes; gearing, direction and tracking mode are refused. A trial value that
  differs from the drive's is queued after every stored write as C13.10 = 0,
  the trial writes, C13.10 = 1 (journal `storage_window=open|close`); IgH
  replays that window at every reconfiguration. A trial the drive already
  holds (rt-core restarted, drive powered) is skipped like any identical
  object, with no window. Every start journals
  `WARNING: startup_trial axis=.. keys=.. storage=ram_only: these values live
  in drive RAM only; ...`; status marks the descriptor `"storage":"ram_only"`;
  trial keys are never service-write targets.
- What a power cycle does. A trial lives in drive RAM: a drive power cycle
  reverts it to the value stored in EEPROM. If it was skipped at this start,
  the master does not replay it; the epoch's startup readback then sees the
  stored value (`verified=0 storage=ram_only` plus `WARNING: startup_trial
  ... a RAM-only trial value was lost`), `config_verified_mask` drops the
  axis and arming is refused until rt-core is restarted and writes it again.
  This is the one exception to "the drive keeps the last value written"
  (`docs/drives/a6ec/README.md`, "Persistence and reverting"). Finalize a
  trial by removing its key from `startup_trial`. rt-core alone does not store
  it then: RAM already holds the value, write-on-change skips it and EEPROM
  keeps the pre-trial value until a start that reads something else (after a
  drive power cycle, whose readback mismatch first drops the axis from
  `config_verified_mask` with no trial warning). The configuration-only deploy
  stores it at once (next item).
- Promoting a trial (2026-10-10, `realtime-core config`, Mac side, no native
  change). The deploy compares the running and the new compiled
  configuration: a key in an axis's running `startup_trial`, absent from the
  new one, with the same compiled value is a promotion. Before publishing the
  new configuration the servo host starts a bridge: the new machine JSON with
  each promoted key set one raw step away (raw + 1, else raw - 1, else the
  nearest other allowed value of the drive's tuning catalog) and listed in
  `startup_trial` again, staged as `config-history/machines/<new>.bridge.json`.
  It waits until every axis verifies the bridge and Home is back, stops, then
  publishes and verifies the new configuration as always; its stored write now
  differs from RAM and lands with C13.10 = 1 (journal `prior=<bridge>
  storage=stored`, `persisted` counts it). One extra controller start; the
  robot is disarmed under the cell lock and the maintenance notice the whole
  time, so the bridge value never drives motion. A bridge that fails rolls back
  to the old configuration, whose own trial write restores RAM. `--dry-run`
  lists each promotion and its bridge value; a key with no catalog range is
  refused. The A6-EC has no store object and no stored-value readback (chapter
  11, ESI), so the EEPROM value itself is proven only by a drive power cycle.
- UNCONFIRMED: whether the drive stores the C13.10 writes themselves, so
  whether a window (0, trials, 1) leaves EEPROM holding C13.10 = 0 after a
  drive power cycle. With `verify_parameter_storage` rt-core repairs and
  verifies C13.10 = 1 at every start, before any HM35 can run; keep it on
  after the last trial ends. Bench check before relying on trials on a cell:
  one window, drive power cycle with rt-core stopped, then read C13.10 and the
  trial object over SDO.
- Identity. Write on change changes no configuration, machine, deployment or
  coordinate identity: a cell compiles to the same pin until it opts in. With
  `verify_parameter_storage`, the storage descriptor and the
  `parameter_storage` / `startup_trial` lines are machine identity (native
  mirrors both only when present) and the catalog's storage block joins the
  configuration identity; deployment and coordinate identity are unchanged,
  so Home anchors stay valid. The HM35 transaction is byte-identical
  (`a6ecHM35Transaction`). No protocol, Describe or capabilities change.
- Deploy. A full rt-core release (native runtime change). Opting in later is
  a re-pin; the first opt-in needs this release's rtctl on the cell, and a
  configuration-only deploy refuses removing the storage key from
  `startup_sdos` once present.

Regressions: native `startup_sdo_plan_tests` (identical, different, unread,
window, held trial, storage restored first, order, refusal),
`bus_bringup_tests` `TestStartup*` through the live constructor on all three
A6 profiles (exact queued writes, no HM35 object, verification inputs
unchanged), `configuration_identity_tests` (parse, identity, refusals); rtctl
`TestA6ParameterStorageObject`, `TestParameterStorageOffKeepsCompiledBytes`,
`TestParameterStorageVerifyAndTrial`, `TestParameterStorageRefusals`,
`TestParameterStorageCapacity`, `TestRobotCellStartupTrial`,
`TestParameterStorageNativeIdentity` (Linux, native identity),
`TestMiniBenchSelectedRobotLogParity` (constructor log still the e0c155eb
bytes apart from the write-on-change fields).

## Jog jerk and jerk-limited one-off moves (2026-10-06)

A Rosie 1430 session with 50 % speed feedforward cut the
lag from 27 to 15 ms but raised actual peak acceleration (J6 856 -> 1161
deg/s^2), and J4, commanded nearly still, rang at ~6.1 Hz: worst 1.60 degrees
of following error, above its bound for 33 ms of the 100 ms fault timeout, at
the start of an OLP Move-to (J1/J6 to 60 deg/s); jog starts kicked it up to
0.92 degrees. Both the jog integrator and the Move-to trapezoid stepped the
commanded acceleration (720 deg/s^2) within one 1 ms cycle.

- Jog jerk. Optional machine-level `jog_jerk` `{"jerk_rad_s3": N}` (integer
  1..100000, rotary axes only) compiles to `jog_jerk=N|W` on every axis, W =
  ceil(2 x a_max / N / cycle) cycles (2..1024), a_max the largest
  jog_acceleration/max_acceleration. Natively `JogSmoother` commands the mean
  of the acceleration-limited integrator's last W positions: acceleration stays
  within each axis's limit and jerk within N for any input, reversals included;
  the governor judges the integrator, and the mean cannot pass a limit the
  integrator respects. Halt bounds grow by W cycles; a Halt that takes over a
  trajectory seeds the smoother with that trajectory's steady state. Admission
  requires one value on every axis and 2 x a_axis / (W x cycle) <= N. Machine
  identity, not deployment identity; absent = unsmoothed, identities
  unchanged. Not in the public Describe schema. A Rosie 1430 cell (and its
  simulation): 150 rad/s^3, W = 168 (one J4 ringing period). Stops take
  168 ms longer and travel |v| x 0.084 s further. Unverified (#158).
- One-off moves use the same jerk. `rtctl.Compiled.JogJerk` exposes the
  compiled value; OLP already compiles the selected cell's machine config to
  check its configuration pin, and selection requires live Describe to carry
  that digest, so OLP takes the jerk from that compile (`RTCoreConfig.MoveJerk`,
  never decoded from a catalogue or config file). It is not added to Describe:
  a new Describe field changes the capabilities digest, which every OLP UI
  build and the installed Steam Deck pendant require to match exactly. Local
  simulation and single-config targets have no repository cell manifest and
  keep acceleration-limited moves.
- Move-to (`denseexec.jointMovesProfile`): every joint follows one shared
  progress s(t), q_j = q0_j + dq_j*s, so the commanded joint vector stays on
  the straight joint-space segment and all joints keep their ratios (the TCP
  path rule). s is one symmetric double-S whose speed, acceleration and
  jerk are the minima over moving joints of v_j/|dq_j|, a_j/|dq_j| (fraction
  scaled) and jerk/|dq_j| (jerk is not scaled by the fraction). Superseded
  (b2e38b46): a per-joint double-S stretched to the longest kept the end times
  but not the path; the measured move's joint vector left the segment by
  0.72 degrees (`TestJointMovesProfileStaysOnTheJointSegment`, now <= 1e-9
  rad on the commanded cycles). A move lasts a/j longer than the trapezoid:
  83 ms at fraction 0.99 on a Rosie 1430 cell (the measured move: 518 -> 601 ms; J1 binds
  every limit there, so the shared progress keeps that duration).
  Cartesian steps (`cartesianMoveProfile`) keep their quintic progress clock
  but lengthen it so its own peak jerk, 60 x slope / T^3, stays within the
  jerk on every joint (a 0.1 rad step: 219 -> 342 ms); jerk from the waypoint
  spline's curvature changes is not bounded.
- Move knots (`denseexec/move_knots.go`). Moves upload 20 ms cubic-Hermite
  knots; a segment straddling an acceleration corner reaches up to 4/3 of the
  plan (measured on a Rosie 1430 cell: planned <= 720, commanded 908-912 deg/s^2 on J1/J6).
  Knots now sit on every sample next to a detected acceleration corner and on
  the samples either side of every jerk or acceleration corner the planner
  names, and Move-to knots carry the profile's exact velocity, so the native
  per-cycle samples equal the plan (within 1e-9 rad).
- `jog_jerk` is not part of any coordinate identity, so Home anchors recorded
  before it was added stay valid.

Regressions: `jog_policy_tests` smoothing (start/stop <= 75 rad/s^3, reversals
<= 150 rad/s^3, acceleration <= 4*pi rad/s^2, Halt bound, trajectory seed,
governor at limits, lane EndJog), `configuration_identity_tests` (jog_jerk
parse, consistency, uniformity, identity), rtctl
`TestJogJerkAdmissionAndIdentity`, OLP `TestJointMovesProfileJerkLimited`,
`TestJointMovesProfileJerkRegression2104`, `TestCartesianMoveProfileBoundsJerk`,
`TestKnotPointsKeepPlannedAcceleration`.

## A configuration digest does not identify its compiler (2026-10-06)

`configuration_sha256` is the frozen compatibility identity of the original
machine JSON and profiles (`tools/rtctl/compiler.go`, identity partition), so
the same bytes give the same digest under any rtctl. What the bytes mean is
decided by the compiler that reads them: `deploymentIdentity` files every
top-level key it does not know as machine semantics under deployment identity,
and the native daemon ignores deployment metadata. On 2026-10-06 the
`jog_jerk` machine JSON compiled under the 6c6f97c0 rtctl to the pin a
jog_jerk-aware build also produced, with `jog_jerk` filed as deployment
metadata and no `jog_jerk=` line on any axis: a matching pin, no jerk
limiting. A pin match proves which input was deployed, not that the running
build implements it. Judge a configuration with the installed build's own
rtctl and refuse any non-`_` deployment-identity key that the running
compile lacks; `steamdeck/real/v4` `realtime-core config` does both. Keys
starting with `_` are documentation and bind deployment identity only.

## Path jog lane (protocol 1.14, 2026-10-07)

A Cartesian hold streamed as per-axis joint velocities left the commanded line
by 0.9-75 mm on a Rosie 1430 cell: each axis ramped and stopped on its own
and the source steered from a pose up to 100 ms old. The path jog lane plays
an application's joint-space path instead (OLP: seeded IK of the frozen line
every 1 mm) along one path parameter, so every masked axis is derived from one
jerk-limited path speed and the commanded target never leaves the sampled
curve. Implemented, unverified on hardware.

- Session. `jog_operations.path_begin` (3) opens a jog session in path mode:
  the same identity, generation, authority, configuration/Home epochs and
  first-input deadline as `begin`, and it excludes joint jog, trajectories and
  Halt exactly like a joint jog. Native refuses it with `path_unavailable`
  (22) when the machine has no `jog_jerk`; Describe then omits `path_jog`.
- Geometry. `path_samples` (fast_control offset 1280, 2296 B) is lossless and
  acknowledged, not latest-value: the writer publishes one chunk (at most
  `PATH_JOG_MAX_CHUNK` = 16 samples x 16 axes, logical joint units, unmasked
  axes zero, per-sample caps in path units/s, +Inf none) and waits until
  `path_observation.chunk_sequence_seen` names it. Native consumes only
  `chunk_sequence == last + 1` of the current session; a chunk of another
  session or generation is ignored without acknowledgement (echoing it could
  acknowledge a new writer's chunk of the same number). Chunk 0 begins
  `PathJog` from the held wire targets in logical units; `spacing` and
  `max_speed` are fixed by it. Refusals: `path_start_mismatch` (16, sample 0
  more than one count off the held target: the session closes, no inhibit,
  nothing moved), `path_capacity` (17, ring of `PATH_JOG_MAX_SAMPLES` = 1024
  full; retryable), `path_out_of_order` (18), `path_invalid_sample` (19),
  `path_ended` (20), `path_invalid_config` (23).
- Speed. `jog_update.flags` / `local_jog_update.flags` bit 0: velocity[0] is
  the path speed in path units/s (finite, >= 0), every other entry zero, the
  session's own mask. It passes `evaluate_jog_update` (increasing sequence,
  origin never backwards, an equal origin cannot extend the deadline,
  deadline within the cell input age) against the speed PathJog holds, then
  `PathJog::set_speed`. A plain velocity frame on a path session, or a speed
  frame on a joint session, is `mode_conflict`. A refused speed never closes
  the session.
- Limits, all from compiled configuration in logical units: the jog lane's
  `command_limits` interval shrunk by one count, the described max velocity,
  `jog_acceleration`, a one-count start tolerance, the machine `jog_jerk` (one
  value for every axis), `PATH_JOG_MAX_SAMPLE_STEP_NS` = 20 ms (unverified:
  axis max velocity x 20 ms bounds one sample's joint step; 0.028 rad per 1 mm
  sample at a Rosie 1430 joint's 1.396 rad/s, about 10x the largest normal per-mm step).
  No machine number is in source.
- Stops. In path mode the lane never runs its per-axis expiry or HaltRamp.
  EndJog calls `PathJog::end`, public Halt `PathJog::halt`, and the speed
  deadline triggers PathJog's own stop: all stop along the path within the
  configured limits, the session (and its axes) stays reserved with input
  fenced (observation `open` = 0) until the stop completes, then closes
  without inhibit (`closed`, or `expired` for a deadline). A buffer underrun
  or path end stops at the playable end and keeps the session until its
  speed expires. Authority loss, readiness, identity, clock and Home keep the
  existing inhibit (`PathJog::abandon` at the last target) and close with
  `PathStopReason::Inhibited` (9) without the observation's fault flag
  (ba4a4af8), so a client can tell an inhibit from a generator fault. A
  generator fault (stop bound overrun, playable end passed, invalid target)
  fails closed with `PathStopReason::Fault` (8), `path_fault` (21), execution
  fault `FaultTargetLead` (the retained bit HaltRamp overruns already raise)
  and inhibit.
- Cycle jitter. Braking margins, the speed-limit horizon and the tracker reach
  use max(this interval, `max_cycle_ns`), where the cycle machine passes
  `cycle_ns + max_cycle_lateness_ns` (compiled; 1 ms + 1 ms on a Rosie 1430 cell): the longest
  interval the machine runs without FaultCycleDeadline (766fce69). Sized by
  the current interval alone, a late cycle overshot the playable end by
  ~1e-4 sample and failed closed (1/120 runs at +-2 ms, 7/120 at +-5 ms);
  now 0/60 at +-1 ms and +-5 ms with a matching bound. Per-cycle cost on the
  RevPi is unmeasured (1.5-1.9 us on an M1 / arm64 container).
- Time base. `PathJog::advance` takes the cycle's wake time, the time base
  trajectories sample at, never a clock read after intake: the drive applies
  each target on the fixed sync, and intake work varies by cycle (a
  `path_samples` append costs ~4 us per sample on a RevPi servo host). Until
  2026-10-08 the lane read the clock after intake and the target led by
  that cycle's work: Rosie 1430 dumps showed every joint displaced
  by its own velocity x the lead (97-100 % of the residual), 60-70 us at each
  16-sample frame of a 25-sample chunk (every ~62 ms at 400 mm/s), one-cycle
  accelerations of 50-146 rad/s^2 against 4*pi and J2/J3 target steps up to
  ~0.004 deg. The geometry OLP streamed was not involved. Fixed in
  `1de9e05c`; re-measure the lead on a 400 mm/s hold dump
  (expected within the wake jitter, ~7 us, unverified). Halt ramps still advance on a clock read
  after intake (`halt_ramps[].advance(..., inputs.clock(), ...)` in
  `cycle_machine.hpp`): no chunk-sized intake work precedes them, not yet
  audited.
- Output. Path targets bypass the jog integrator and `JogSmoother`
  (`JogAxis::follow`); joint jogs keep `jog_jerk|W` unchanged. The target-lead
  warning, position-rate guard, output policy and following-error fault run
  on path targets exactly as on jog targets.
- Observation. `path_observation` (offset 3584, 184 B) is published every
  cycle: chunk acknowledgement, received/playable/consumed indices, path s,
  speed and acceleration, underruns, speed sequence and deadline, PathJog's
  `PathGoverning`, `PathStopReason` and end reason (values 1:1), flags bit 0
  open, 1 stopping, 2 ended, 3 has speed, 4 fault. It describes the echoed
  generation including its final state after close.
- Adapter. `begin_path_jog`; `GET /v1/jog/path` (`PathJogStatus`: the native
  observation read from the fast region, never `metrics.json`, plus the
  handshake state). `jog.sock` and WSS `/v1/jog` tell frames apart by exact
  size (local_jog_update 224 B, local_path_samples 2272 B). One writer per
  session owns the handshake behind a bounded queue (unverified 64 chunks);
  `path_capacity` is retried, any other refusal or a lost acknowledgement
  ends the session through EndJog. WSS begin carries `"path": true`, chunks
  get `path_ack` replies, path speed uses the same clock mapping as velocity.
- Transient refusals (2026-10-10). A speed or velocity input that meets
  another writer is refused without being judged. The writers are a second
  producer on the adapter's publication lock, the client's jog or
  fast-region publication lock (Begin/End, a path chunk and a grant request
  each hold it for one publish), and a native observation slot caught
  mid-publication on every bounded read. That refusal is transient, and only
  that one. rt-control repeats the input, rerunning every admission check
  (identity, generation, authority, clock, increasing sequence, open session
  and epochs, vector, deadline, input age), for an unverified 5 ms, never past
  the input's own deadline. If the collision outlasts that, WSS answers
  `jog_publisher_busy` (an existing schema reason; the capabilities digest is
  unchanged), not `jog_native_rejected_3`. The Go SDK resends the same input
  (same payload, origin and deadline, next sequence) only while it is the
  latest one sent, before its source deadline, and at most 8 times
  (unverified), and keeps the session. Native admits the resend only under
  `evaluate_jog_update`'s rules: an equal origin never extends a deadline.
  Every other refusal stays terminal and ends the session, including a
  native `not_ready`. The ingress counters still record a transient refusal
  as `not_ready` (3).
  On a Rosie 1430 cell one such collision, reported as `jog_native_rejected_3` with 195 ms
  of input lifetime left, ended a held +X hold. Native never saw the frame:
  its source sequence stopped at 19.
- Refusal journal. Every ingress refusal is written to rt-control's journal
  (`journalctl -u rosie-rt-control`). Each line carries the transport, the
  full jog reason code and name (path codes are not clamped to 3 as the
  counters are), `transient`, the attempts, the grant and jog generations,
  the sequence, the milliseconds left to the input's deadline and the cause
  text. The budget is an unverified 50 lines per second, and the next line
  carries the suppressed count. `/v1/jog/ingress` counters stay per process;
  the journal survives an rt-control restart.
- Not added: path fields in the stop dump / cycle capture (the V2 record
  layout digest is unchanged); C++ client parity (explicitly unsupported).

Regressions: `path_jog_lane_tests` (admission and identity fences, handshake
gaps, capacity, spacing, unmasked samples, start mismatch, End/expiry/Halt
on the curve within one count with joint jerk <= 150 rad/s^3, underrun at
the playable end, fault fails closed, authority loss), `cycle_machine_tests`
`test_path_jog_through_cycle` (Rosie 1430 units: gear 100, 4*pi rad/s^2,
150 rad/s^3, 1.396 rad/s; emitted counts on the curve within one count,
End/Halt/expiry without inhibit, smoother untouched, trajectory refused),
`test_path_jog_streamed_chunks_on_the_cycle_grid` (two axes, 25-sample
chunks as 16 + 9 frames streamed during playback, 100 ahead and starved to
within 21 samples of the playable end, 4 us of intake per sample: <= 4*pi
rad/s^2 and <= 150 rad/s^3 on the 1 ms grid, on the curve within one count),
protocolgen path goldens and geometry, SDK `TestSDKLocalPathJog`,
`TestSDKRemotePathJog`, `TestSDKPathJogUnavailableWithoutJogJerk`.
Transient refusals: ipcclient `TestLocalJogPublisherCollisionIsTransient`,
`TestObservationBusyIsTransientStaleIsNot`; adapter
`TestRemoteJogPublisherCollisionIsRetried`,
`TestRemoteJogLongCollisionIsTransientAndJournaled`,
`TestUpdateJogPublisherCollisionIsRetried`, `TestJogTransientClassification`,
`TestRetryTransientJogBounds`, `TestJogRefusalJournalBudget`; SDK
`TestPathJogTransientRefusalResendsAndKeepsHold`,
`TestRemoteJogTransientRefusalResendsWithinBound`,
`TestJogRefusalClassification`.

## Brake-hold detections are warnings (2026-10-01, offline, not deployed)

Supersedes "fail closed" above for the brake guard's consequence only; every
detection rule and bound is unchanged. Drives manage their brakes (rt-core
submits CW0; the drive applies its C05.10 hold and C05.13 release delays),
absolute multi-turn feedback retains position, and settle of a few counts under
load at torque-off is normal. Any detection, opt-in or legacy anchor tolerance,
in opening, closing, settling or held phases:

- returns one warning from `DisabledBrakeGuard::observe`, published per axis as
  `brake_hold_diagnostic` with a monotonic `warning_sequence`, `transition`
  (torque_on for opening phases, otherwise torque_off), operator `message`,
  trip sample, caps, and peak/settled displacement and peak velocity;
- never sets bit 10, `safety_fault_mask` or an execution fault, never revokes
  outputs, invalidates Home, disarms, or refuses Arm/Enable/jog/trajectory, and
  needs no reset or restart. `BrakeResetQualification` and the output policy's
  brake-hold input are removed (the stationary CSP Home stage has since been
  removed entirely; see the HM35 section below);
- resettles: no further warning for the same settle. It re-anchors at Operation
  Enabled, Home service or a frame change; when Arm is requested and the drive
  reports the brake applied with no hold window (an opening stage in progress
  keeps its token until Operation Enabled); or after a quiet applied dwell
  (`settling_ns`, legacy `hold_delay_ns`) within the held tolerance. Travel
  beyond the displacement bound (transition cap, legacy tolerance) while
  resettling warns again from a new reference;
- while inactive and resettling, publishes the unfinished transition
  (WAIT_HOLD_DELAY with hold window, or WAIT_RELEASE_DELAY) and denies motion;
  when enabled, BrakeMachine's release-delay gating alone applies;
- ends the startup-alarm exemption, so the startup reset witness is never
  re-established after a warning.

Stop/CW0 timing, drive-managed hold windows, release-delay motion gating and all
other faults (following error, target lead, limits, completion, collision, DS402
Fault/statusword bit 3, coordinate trust) are unchanged.

### Energized hold window and suspected slip (2026-10-01 follow-up, offline)

Working-tree change on top of the section above; not committed, deployed or
physically qualified. It supersedes the AMR-712 stationary-target corridor and
"every sample" closing rule above for closings that began from an observed
Operation Enabled.

- Energized window: from the first closing observation (the CW0 submission for
  a qualified release or HM35 trigger) until `hold_delay_ns` has elapsed and
  BrakeMachine's hold window has ended, the drive still holds torque. Closing
  displacement and velocity there are servo settling, not brake evidence: they
  are recorded (`energized_peak_displacement_counts`,
  `energized_peak_velocity_counts_per_s`) and never warned. Missing velocity,
  stale/aged observations, drive state, frame change, wire proof and the
  closing timeout are still evaluated. Legacy J1â€“J3 guards use BrakeMachine's
  hold window for the same rule.
- Torque off: the first sample after that window re-anchors
  (`torque_off_anchor_counts`, `torque_off_ns`) and closing, settling and held
  checks are judged from it: Â±`hold_displacement_tolerance_counts` corridor plus
  the phase's velocity and displacement caps on every sample, starting with the
  torque-off sample. A warning there is truthfully torque_off, C05.10. The last
  servo target no longer selects a corridor; `target_stationary_ns` is retained
  in configuration identity only. A legacy nongravity axis is judged for one
  `hold_delay_ns` of brake take-up after torque off; a gravity axis keeps its
  permanent hold monitor. An opening aborted before Operation Enabled has no
  energized window and is judged on every sample, as before; the opening path
  and its C05.13 message are unchanged.
- Suspected slip: within one braked period (from torque off until the drive
  reports Operation Enabled, an Arm/enable request, an opening authorization,
  Home service, a frame change or an absent brake; configuration is fixed per
  daemon incarnation), consecutive motion warnings (displacement, corridor,
  caps) with the same displacement sign form a run. The
  `kBrakeSlipSameDirectionWarnings` (3, unverified initial choice) warning and
  any later one in that run is `kind` slip with `same_direction_count` and
  `same_direction_displacement_counts` (net from the run's first anchor) and the
  message "Brake may be slipping: J<n> moved <N> times in the same direction
  (total <counts> counts, <deg> deg) while the brake was applied - inspect the
  brake." A reversal restarts the run at 1; a torque-on warning ends the period;
  a warning without motion evidence neither extends nor breaks it. Slip never
  faults, inhibits or blocks Arm.
- `brake_hold_diagnostic` is also published when an energized window closes,
  with `severity` none until the first warning; `warning_sequence` alone
  identifies a new warning and `brake_warning` event.

### Uniform joint-angle brake, hold and restore bounds â€” October 2, 2026 (offline change)

Working tree only; not committed, not deployed, not physically qualified (GitHub issue #158). Nothing is joint-specific: each bound below is one joint-unit value, converted per axis by its gear ratio (131072 Ã— gear / 2Ï€ counts per joint radian) and floored, so no axis admits more than the declared angle. Count and joint-unit forms of the same bound cannot be mixed, and a value that floors below one count on any axis is refused. Each value is the largest previous per-joint equivalent (J4 or J6); none was measured on the loaded arm.

- Held tolerance and encoder/PDO agreement: the A6 profile declares `hold_displacement_tolerance_rad`, `maximum_feedback_error_rad` and `maximum_anchor_error_rad` = 4.2611e-5 rad (the former 16 counts at J4's 18:1), giving 16 counts on J4, 44 on J6 (50:1) and 88 on J1â€“J3/J5 (100:1).
- Start-alignment floor: native `kIndependentEncoderStartAlignmentFloorRad` = 1.332e-4 rad (the former 50 counts at J4).
- Brake guards on all six joints: the Rosie 1430 cell machine configuration and its trials declare one machine-level `brake_transition_guard` (`max_displacement_rad` 0.050Â° = 0.0008726646 rad, `max_velocity_rad_per_s` 3.0Â°/s, target stationary 100 ms, settling 100 ms, timeout 500 ms) and one `brake_opening_guard` (0.050Â°, 8.0Â°/s, timeout 500 ms). Natively that is closing 1820 counts / 109226 counts/s on 100:1, 327 / 19660 on J4 and 910 / 54613 on J6; opening 1820 / 291271, 327 / 52428 and 910 / 145635. Robot-bound cells refuse per-axis guard blocks, and a machine-level block must use joint units. Every axis still satisfies the native checks: displacement â‰¥ held tolerance and a mapped 32-bit `velocity_actual` (the A6 fixed TxPDO 0x1B04 used by J1â€“J3 already carries it). Brake detections remain warnings.
- Stationary CSP Home precondition (`--native-home-csp-precondition-axis`): its caps are now joint units too (held 4.2611e-5 rad, 0.050Â°, 1.0Â°/s, opening 2.0Â°/s), so any joint qualifies on the same angle instead of only J4 on 16 counts. The same gear-aware held-tolerance check gates the brake-opening contexts, which now apply on all six axes. The canonical opening rate (8.0Â°/s) exceeds this diagnostic's 2.0Â°/s opening cap, so the diagnostic refuses the canonical machine, exactly as it refused the earlier 8Â°/s J4 trial; a commissioning run needs an explicitly narrower opening envelope.
- Manual restore (verified-delta, command 267) has no cap. `maximum_delta_counts` is removed from the compiler (refused by name), native profile, adapter and public schema. An HM35 (v3) anchor restores to any position within the axis hard limits when the full signed command and independent-encoder deltas agree exactly within the anchor tolerance, `607C` is unchanged and the axis is fresh, authorized and inhibited. No periodic equivalent or seam reconciliation is admitted; the hard-limit check applies to the same continuous position. The 2026-10-19 removal of the public wrap fields requires regenerated OLP and Steam Deck clients; record the final capabilities digest only after the implementation tree settles.

## HM35 Home transaction (update 2026-10-19; bench qualification pending)

Native Home submits exactly the descriptor compiled from `config/drives/a6ec.json`: service mode 6/CW0 then 50 ms; `set_mode 6`; 60E6=0; `607C=0`; 6098=35; CW 6, 7, 15; wait set 0x0227 clear 0x2048; CW31; wait set 0x9000 clear 0x2000; refresh truth; `restore_mode 8` (mode 8 with CW31); mode 8/CW0; Release; CompleteHome. The 2026-10-19 change to absolute linear mode alters only the Home offset; the vendor-confirmed order, writes and waits remain fixed. The worker inserts no steady-mode Shutdown dwell, CSP precondition, pretrigger dwell or diagnostic read, and the cyclic owner holds no fixed pretrigger target: Home service targets track feedback. The brake guard never authorizes an opening stage in homing mode. Final test names and generated pins are recorded after the concurrent implementation settles; the mini-bench must prove that the transaction yields `6064 = 0`.

## Drive-state supervisor (2026-10-02, offline, not deployed)

General enable/drop-out protections that were lost when brake detections became
warnings now live in `include/drive_state_supervisor.hpp`, wired by the cyclic
owner. They apply identically to every axis and never read brake-guard state.
Each refusal ends only the current Arm attempt: `armed` clears in the same cycle
(CW0 on every axis), while the grant, control generation and enable selection
are retained. Nothing latches; no reset or restart is needed; Arm is retried.

- Bounds are configuration, not source. The machine declares one
  `drive_supervision` block applied to every axis (`enable_timeout_ms`,
  `release_slack_ms`, and joint-unit `disarm_motion_tolerance`, rad or m).
  rtctl floors the joint-unit value to counts per axis by gear, refuses a
  value below one count on any axis or a per-axis declaration, and emits
  `drive_supervision=enable_ms|slack_ms|disarm_counts` on every axis line;
  native admission requires all three positive and identical timing on every
  axis, and the line is part of machine identity. A Rosie 1430 cell declares 500 ms,
  500 ms and 0.02 rad (41721 counts at 100:1, 20860 at 50:1, 7509 at 18:1;
  previous values from the brake_opening_guard timeout and
  limits.following_error; unverified, issue #158). A machine without the
  block compiles no line and these timing and motion checks are off. There is
  no supervisor stationary value: Home acquisition keeps the machine's
  joint-unit held tolerance (`joint_unit_tolerances`), so one quantity is not
  declared twice.
- Enable timeout: an armed, enabled axis not in Operation Enabled within
  `enable_timeout_ms` of the Arm request ends the attempt: "J4 did not reach
  Operation Enabled within 500 ms (statusword 0x....). Arm ended; retry Arm."
- Release timeout: Operation Enabled but the brake never reports released
  within the axis's release delay plus `release_slack_ms` (0 delay without a
  brake).
- Armed drop-out: an armed axis leaving Operation Enabled with no fault bit
  (STO, external enable, drive quick stop) ends the attempt instead of being
  silently re-stepped CW6/7/15: "J5 left Operation Enabled while armed
  (statusword 0x....). Arm ended; retry Arm." A commanded jog quick stop is
  exempt and its re-enable gets a fresh enable bound; a fault bit stays with
  the existing drive-alarm report.
- Arm-stage cancellation (frame change, context loss, mode change, hold target
  change, interrupted opening) ends the attempt with that cause instead of
  holding CW0 while `armed` stays true.
- Disarm motion: after an inhibit issued from a stationary ordinary command
  (not trajectory, jog, halt or Home/commissioning service), while the
  drive is still energized (Operation Enabled, then the C05.10 hold delay
  mirror; 0 without a brake), displacement from the inhibit position beyond
  the axis's compiled `disarm_motion_tolerance` counts warns once: "J4 moved 1.146 deg while disarming
  (drive still energized); bound 1.146 deg." Warning only; it never blocks the
  next Arm.

Events publish per axis as `drive_supervisor` (`severity` arm_ended/warning,
`event_sequence`, `kind`, `cause`, `message`, `statusword`, `current` until the
next accepted Arm request) and one adapter `drive_supervisor` event. Supervision
is suspended while Stop, lease loss or a latched fault already inhibits.

The brake warning now carries only brake take-up/release/slip evidence. Guard
causes `opening_timeout`, `frame_changed`, `stale_feedback`, `observation_time`,
`drive_state`, `opening_proof` and `invalid_configuration` restart the guard
quietly (no warning; motion withheld until a fresh applied sample anchors it)
and are reported by the supervisor or the existing freshness/readiness checks.
Benchdrive's post-disable verdict records `post_disable_warning_free` NotMet
when a brake warning or supervisor event was published during the run. The
startup drive-alarm one-shot reset still requires the opt-in brake transition
guard's never-enabled, fixed-anchor witness; it is unchanged pending a per-joint
decision.

## rt-control maintenance route (2026-10-07)

`GET /v1/maintenance` (capability `maintenance`, response
`MaintenanceObservation`) serves the servo host's maintenance notice exactly
as the admission guard enforces it (`/etc/rosie-rt-core/maintenance.json`,
`busy` refusals of acquire and motion starts while active) and the current
grant's holder (`MaintenanceHolder`: acquire controller name, remote or local,
remote certificate identity, generation, adapter wall-clock time of the
grant). It needs no session or authority, like `describe`, on the local
socket and the mutual-TLS listener, and never returns the session token or
the deploy's run id. Display text is clipped to 400 bytes on a UTF-8
boundary; the `busy` refusal keeps the notice's whole reason. A malformed or
unreadable notice reads `active` and `malformed`; an ETA never clears it.
OLP merges it with its own host's notice (active if either, both reasons,
later ETA) and keeps the last active cell notice while rt-control is
unreachable. Adding the route changed the capabilities digest from
`b1e28f06…` to `725c5bbb…`; the OLP UI pin and the Steam Deck v5 `olp-core.js`
follow it. Regressions: adapter `TestMaintenanceRoute*`, SDK
`TestSDKMaintenanceReadsTheAdapterNotice`, apigen schema-covers-handlers,
Go/C++ wire interop, OLP `cell_maintenance_test.go`.

## Operator reset of resettable drive alarms (2026-10-11; not yet exercised by an operator)

On a Rosie 1430 cell, J5's drive raised Er41.1 (0x603F 0x7121, 0x203F
0x0411, "Motor over-temperature due to locked-rotor", resettable in the
manual) after a mechanical stall. For more than two hours every Reset answered
`fault_persists` (drive_readiness, axis mask 16, `reset_after_condition_clears`)
until rt-core was restarted. The recovery matrix was deadlocked: an A6-EC leaves
CiA402 Fault only on the fault-reset edge (controlword bit 7 rising, transition
15), but `decide_fault_reset` kept drive_readiness while the statusword fault
bit was set, and the edge (`drive_reset_mask`, the 20-cycle pulse) was issued
only by a transaction that did not persist. Reset's own revocation also left no
output permit for that pulse. A drive that faults while disarmed latches no
native fault at all; Reset then accepted with no effect and the drive stayed in
Fault. Both are fixed here, in `include/drive_alarm_reset.hpp` and the cyclic
owner (`engine/cycle_machine.hpp`).

- **Classification is drive-profile data.** The profile's `fault_reset` block
  lists each resettable alarm by its exact (0x603F, 0x203F) pair; rtctl
  compiles it to the axis line `drive_alarm_reset=<leave_fault_ms>|
  <min_interval_ms>|<603F>:<203F>;...`, part of the machine identity. Source
  classifies nothing. An axis without the line, an alarm the line does not name
  and an alarm whose 0x203F detail is unread are not resettable (a bus code alone
  is ambiguous: 0x3230 is Er40.0, Er41.0 and ALF6.1). `config/drives/a6ec.json`
  and `a6ec_1b02.json` list the 46 manual chapter 10 faults marked resettable
  and exclude, with reasons, ErA0.1 (multi-turn overflow) and Er84.3 (Home
  position setting error), which concern the position reference and stay
  HM35-only, and the reserved Er81.2 (no bus code). The manual's class column is
  printed beside both resettable and non-resettable codes and is not a reset
  criterion. rtctl `TestDriveAlarmResetProfileMatchesManual` holds the list to
  `docs/drives/a6ec/a6ec_manual_codes.json`. The bench profiles
  (`a6ec_bench_1to1*.json`) carry no block and keep the previous behaviour.
- **Admission (one table for Reset and status,
  `classify_drive_alarm_reset`).** Per selected axis: the drive reports CiA402
  Fault; the profile is classified; the statusword is fresh (two cycles); the
  0x203F detail is read; the pair is listed; the robot is disarmed with nothing
  enabled, no motion, Home, service or commissioning; and the axis's last
  attempt is at least `min_interval_ms` old. Reset also requires the press's
  current authority, a healthy verified bus, the axis's verified configuration
  and no pending Stop, and that the ordinary matrix would not persist once the
  admitted drives' own Fault is set aside. Any other persistent fault refuses
  the press without touching a drive. Armed: refused before anything is
  classified (unchanged). Axes eligible for the startup-alarm reset keep that
  path.
- **The edge.** Admitted axes get the drive's own CiA402 fault reset: CW 0x80
  only while the drive reports Fault (not in Fault Reaction Active, where a
  rising bit is ignored), for at most the existing 20-cycle pulse, falling the
  cycle the drive leaves Fault; a drive that faults again gets no second edge.
  It is written in the inhibited output path, so no enable, motion or brake
  release can follow it. The attempt is bound to the generation after Reset's
  own revocation, the configuration and Home epochs, the Stop fence, a fresh
  statusword on its axes, the transport heartbeat and `leave_fault_ms`; any
  change (Stop, a new grant, Arm, Home, service, configuration, bus failure)
  ends it at once with CW0 as `interrupted`. One attempt at a time.
- **The outcome answers the same press.** An axis is `cleared` only when its
  drive left Fault and stayed out for the whole pulse window; then the
  ordinary recovery matrix decides every rt-core fault on fresh feedback, as
  for a press with no drive in Fault (drive_readiness keeps its
  re-verification). Otherwise (`persists_after_reset`, `refaulted`,
  `interrupted`) nothing is cleared and the press reports `fault_persists`.
  The reset's recovery publication, with its `reset_sequence`, appears only at
  the end of the attempt, inside the adapter's 1 s observation (`leave_fault_ms`
  is bounded to 500).
- **Never touched:** Home validity (beyond the matrix's existing rehome
  outcomes), the HM35 transaction, anchors, startup SDOs, tolerances and the
  non-resettable alarms. A cell's coordinate identity and every startup line
  are byte-identical; its compiled configuration differs from the previous one
  only by the `drive_alarm_reset` line.
- **Surfaces.** Native status per axis: `drive_alarm_reset` {`possible`,
  `reason` (none, possible, not_configured, alarm_unread, not_resettable,
  rate_limited, not_inhibited, feedback_stale, in_progress), `retry_after_ns`}.
  Native recovery JSON: `drive_alarm_resets` (axis, result, codes, drive alarm,
  message) and `message`, the joined sentences of the failed axes, for example
  "J5 drive alarm Er41.1 persists after reset: Motor over-temperature due to
  locked- rotor" or "J1 drive alarm Er20.1 is not resettable from rt-core (drive
  profile): Encoder internal fault". The adapter's refusal is `fault_persists:
  <message>` (callers that read the code before the colon are unchanged).
  `ipcclient.RecoveryStatus.Message` is native-to-adapter only (`json:"-"`): the
  public schema and capabilities digest are unchanged. Exposing per-axis
  `reset_possible` on the public status needs a schema field (a capabilities
  digest change) and is not done; native status and the refusal carry it.
- **Journal.** One line per axis per press: `drive_alarm_reset axis=4 joint=J5
  reset_sequence=... event=... result=... error_code=0x7121 aux_code=0x0411
  aux_valid=1 alarm="Er41.1" statusword=... retry_after_ns=... message="..."`.
- **Numbers.** The A6-EC profiles: `leave_fault_ms` 200 and `min_interval_ms`
  1000, both unverified (the A6-EC leave-Fault time is not measured; record it
  from the first live reset's journal line).
- **Identities.** Every machine whose profile carries `fault_reset` gets a new
  configuration identity; cell manifests and their test mirrors follow.
  Capabilities digest unchanged (`725c5bbb…`).
- **Regressions.** Native `tests/drive_alarm_reset_tests.cpp` (parser,
  exact-pair rule, admission order, one-edge pulse, operator text);
  `tests/cycle_machine_tests.cpp` `test_operator_reset_clears_resettable_drive_alarm`
  (resettable and leaves Fault → one press clears, Home kept; stays in Fault →
  20 high cycles then `fault_persists` naming J1 Er41.1; second press inside the
  interval refused `rate_limited`, a new edge after it; re-fault → `refaulted`;
  Er20.1 → refused `not_resettable`, no edge; unread detail; unclassified
  profile; armed → refused; Stop mid-attempt → `interrupted`, CW0; unlatched
  drive Fault → cleared); rtctl `TestDriveAlarmReset*`; adapter
  `TestFaultResetRefusalNamesTheDriveAlarm`; ipcclient
  `TestRecoveryStatusNativeMessageStaysOutOfThePublicForm`. The HM35 goldens
  (`kVendorHm35Transaction`, `TestA6ecHomeTransactionMatchesVendorGolden`) pass
  unchanged.

2026-10-18 candidate: coordinated I/O API identity `2a3b54dac880e1007d20ec690f9ca60450bb9e07b33e3fc19a7a8bff95c44c91`; native protocol minor 15 includes 16-channel state, stationary waits and scoped input requirements. Rebuild every linked consumer; old clients must refuse the new identity.
