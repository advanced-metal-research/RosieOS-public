# Offline programming execution through rt-core

Start with [CELL_CONNECTION.md](CELL_CONNECTION.md) for the one-page picture of how
the app finds, verifies, controls and releases a cell; this document holds the detail.

Plans and their history persist as planning results on the same authored revision.
They do not change authored inputs, which remain protected by revision immutability.
Rails planning waits for persistence before completing, retains robot/calibration
provenance, and LOCAL is shown only after a successful save. A failed write remains
an explicit storage error. Refresh restores saved plans, not native control authority.

## Arm recovery after a stopped run

For a start-position refusal, Arm to recover, then use **Go to plan start**,
**Load to robot**, and **Play**. Go to plan start reads the selected current
trajectory's first sample and uses the existing synchronized joint-move lane at
the selected Jog speed, with process outputs off. It requires matching robot and
calibration identities and armed, fresh joint feedback; STOP cancels. It does
not Home, replan, load, or play automatically. Native joint bounds and motion
admission still apply; this joint move is not a collision-planned approach.

The move API's `axes[]` accepts optional `position_rad`, exclusive with nonzero
`degrees`. Absolute endpoints are resolved from fresh native held targets, so
small offsets are not skipped or calculated from stale browser feedback.

Arm attempts native Reset for both `reset_clears` and
`reset_after_condition_clears` faults before Enable. Native Reset must accept
the current evidence, and fresh status must show the fault mask cleared, before
a new grant is acquired for Enable/Arm. A refused reset prevents enabling;
re-Home and restart faults are not reset by this path. Expected grant retirement
while the Reset reply is pending and its fault-cleared event are not new faults.
Transport errors and newly latched faults retain the independent Stop path.

An explicit **Clear fault** is separate from Arm. Native may advertise
`reset_preserves_home: true` for a latched coordinate-reference fault after
independent saved-anchor recovery has revalidated the reference. Clear fault
then checks fresh inhibited status, rechecks that native proof after acquisition,
and verifies fault clearance, unchanged Home identity/offsets and retired
authority. Missing proof, a changed reference, an additional non-resettable fault
or a refused reset cannot report success. It needs controller identity, not a
robot model, planner, program or new Home. Arm and return-to-zero do not consume
this exception: the operator clears the fault first, then makes a new Arm action.

Reset uses the SDK’s one-pass `StatusObservation` decoder: it retains every
current controller field but skips decoding `execution.handles` history. The
ordinary `Status` API and server-side history remain unchanged. The SDK requests
the native current-only `/v1/status?view=control` view, which avoids copying,
sorting and transporting historical handles. Older adapters ignore the query;
the same decoder still discards history from their full response. No observation
age, request or recovery deadline is relaxed by this optimization.

Reset observation tolerates publication lag within the existing 500 ms window:
an old sample is reread at 25 ms intervals, while every accepted sample still
meets the 150 ms publication-age-plus-request-time ceiling. Invalid clocks,
changed identity/Home and active motion refuse immediately. After the single
native Reset acknowledgement, OLP waits for fresh advancing evidence that both
faults and authority have cleared, rather than treating an intermediate
publication as a failed reset. Timeout or cancellation cannot report success.
This applies to explicit Clear fault and the existing automatic reset during
Arm; it does not change which faults are resettable or enable/re-home the robot.

An explicit **Home** also supports reset-then-Home for a single isolated
`coordinate_reference` fault affecting only the requested joints. That fault
does not by itself prove a new Home is necessary: inspect current native recovery
evidence first. When the reference cannot be recovered and the operator deliberately
establishes a new drive Home (HM35), the joint must be at its mechanical home pose.
The machine must be disarmed
and disabled, with no other commissioning or Home operation active. Recovery
captures the exact fault type, joint mask and native incarnations; another fault
or changed authority cancels it. Native Reset must accept before a fresh grant
runs native drive Home. A failed Reset never reaches Home.

`target_lead` is **not** recovered by Home (2026-10-02, AMR-720). A target-lead
trip is a command/tracking fault: the absolute encoder and the saved HM35
anchor are unaffected. Native classifies it as `reset_after_condition_clears`
and keeps Home valid unless coordinate trust was actually lost. It clears through
Clear faults or the Arm reset above, without changing the reference. The
2026-10-01 change (b91d7da3) that made Home re-zero a target-lead joint at
its current pose was reverted. Running HM35 away from the mechanical home pose
redefines that joint's zero and must never be used to clear a fault.
Since 2026-10-06 a jog or Halt target leading feedback beyond `max_target_lead`
is a native warning (`target_lead severity=warning` in the rt-core journal),
not a fault: it no longer disarms. The following-error fault remains the hard
tracking guard (rt-core/docs/CONTRACTS.md, "Target lead is a warning"). Both
bounds now grow with commanded speed by the machine's `tracking_lag_allowance`
(50 ms on a Rosie 1430 cell), so fast jogs and plans no longer trip on the
arm's ordinary ~27 ms lag; the stationary bounds and the 100 ms following-error
persistence are unchanged (CONTRACTS.md, "Speed-scaled tracking allowance").
Describe still reports the static `max_target_lead` and `following_error`.
A machine configuration may enable the drives' own speed feedforward on every
joint (measured on a Rosie 1430 cell: lag 15 ms instead of 27 ms; CONTRACTS.md,
"Drive speed feedforward"). The machine-level `jog_jerk` (150 rad/s^3 on a
Rosie 1430 cell, unverified) smooths native jog starts, stops and Halt
ramps, so a released jog coasts 168 ms
longer before it holds, and OLP takes the same value from the cell's compiled
configuration (bound to the pin live Describe must match) to jerk-limit
bounded Move-to (one double-S progress shared by every joint, so the joint
path is the straight segment; a/j = 83 ms longer at full fraction) and
Cartesian steps (a longer quintic clock, also shared by every joint). Those moves also upload knots on
both sides of every jerk or acceleration corner with exact velocities, so the
commanded acceleration stays at the planned limit instead of overshooting up
to 4/3 (CONTRACTS.md, "Jog jerk and jerk-limited one-off moves"). Local
simulation and single-config targets keep acceleration-limited moves.

Recovery tests cover conditional reset acceptance and refusal and grant
retirement before the Reset reply. Physical re-Arm remains an operator action.

## Cartesian machine jog

`POST .../dense-execution/cartesian/{start,intent,stop,halt}` takes `target_id`,
`session_id`, `revision`, `frame` (`base` or `tool`), six SI `twist` velocity
components and `fraction`. All POSTs require selected target generation and
cell headers, including release. `GET .../cartesian/state` reports the shared
joint jog revision, generation, sequence, reason and `cadence_ms`. Stale target
identity returns 409 `target_changed`; machine failures never fall back to preview.

A Cartesian **hold** is a path hold (since 2026-10-06; design option D).
The velocity hold it replaces resolved joint velocities from a measured pose
35-100 ms old and left the commanded line by 0.9-75 mm on a Rosie 1430 cell;
it is gone, not kept as a fallback.

Deploying it needs, together: an rt-core package at protocol 1.14 with
`jog_jerk` configured (a configuration-only deploy cannot carry it), the OLP
binary and UI from the same branch, an OLP catalogue re-pinned to the new
capabilities digest (`201fbe81…` -> `b1e28f06…`), a `robot-v4-cartesiand`
with `path_extend`/`fk` (`262ce393` or later; older binaries refuse the
operation with `cartesian_input_invalid`), and a rebuilt Steam Deck v5 package,
whose bundled OLP code refuses any other capabilities digest.

- **Capability.** Holds need the cell's Describe `path_jog` capability (rt-core
  protocol 1.14, advertised only when the machine configures `jog_jerk`).
  Without it Start returns `cartesian_hold_unavailable`; Cartesian **Step**
  keeps working. `GET .../cartesian/state` reports `fraction_updates: true`
  when holds are path holds.
- **Line.** Start requires the arm at rest (`motion.done`) and takes the held
  command (`logical_target`) of J1–J6 as the line origin, never the measured
  position. Exactly one twist component may be nonzero (no diagonals). BASE
  freezes the other base-frame values; TOOL freezes the tool frame at the start;
  rotations turn about the frozen axis through the TCP.
- **Geometry.** `robot-v4-cartesiand --resolve-only` operation `path_extend`
  solves exact seeded IK every 1 mm / 0.25 deg along that line (polished to
  ~1e-11 m so the native third difference stays clean) with joint-limit,
  joint-step (Describe velocity x `path_jog.max_sample_step_s`) and singularity
  gates. The first failing sample ends the path; rt-core stops before it. OLP
  streams 100 samples of look-ahead in chunks of 25 (unverified policy) and
  keeps that look-ahead from the native consumption point. Each sample carries a
  speed cap that carries OLP's 0.75 joint velocity reserve onto the path, and
  the singularity scale between the resolver's gates.
- **Speed.** Every browser or pendant intent is one native speed command:
  `fraction x |twist|` with origin at the intent's receipt and deadline origin +
  the cell's input lifetime. The fraction may change within a hold (joystick,
  speed select); frame, axis and direction may not (`jog_mode_conflict`:
  release, then press again for a fresh line). No timer sends a speed command.
- **Re-press while the previous hold stops** (`internal/denseexec/jog_queue.go`,
  joint jogs too; supersedes design D's refusal). A
  begin that arrives while the same session's released hold is still stopping
  waits for that stop, then begins; `jog/state` reports `queued`. The previous
  stop is never shortened or replaced, and a queued press never moves, stops or
  disarms the robot. The wait is bounded by that stop's own bound after its End
  returned: a joint lane's native HaltRamp bound, `ceil(|v|/a)` + the jog_jerk
  window + one cycle; a path stop publishes none, so the cell's input lifetime
  (250 ms on a Rosie 1430 cell); plus two round trips, one 10 ms poll and one cycle. Past
  the bound the press is refused `jog_mode_conflict` as before. The deadman
  holds: the press keeps receipt + lifetime; intents of its revision refresh it
  (latest wins), including while the released begin sets up; release or Stop
  cancels it; an expired press is dropped (`jog_input_deadline_expired`), never
  started late. A path hold's line still starts at the held command at rest
  (motion done, then every axis published after it, 10 Hz).
- **Intents keep their cadence on a slow link** (browser `jointJog.ts`,
  pendant `hold_pacer.hpp`; a 400 mm/s hold on a cell once expired because
  the client kept one request in flight and skipped ticks). Against a
  server whose `jog/state` reports `pipelined_intents`, a hold keeps sending
  every 50 ms while its press or intents await replies, up to 5 in flight (one
  input lifetime of cadence): intents reach OLP every cadence up to a ~250 ms
  round trip, round trip - 200 ms apart beyond it. Release stops the cadence,
  aborts the in-flight intents and sends End at once. OLP drops an intent
  overtaken at its lock by a newer one (origin not after the last sent;
  latest wins, never a longer deadline); without that, two intents arriving
  together refused the older `jog_invalid_timing`, so clients keep one in
  flight against a server that does not report the flag. An intent refused
  `jog_session_stale` before the press is answered (an ordinary Begin still
  opening its lane) is ignored; the press's reply decides. The browser no
  longer refuses a press locally while a path release (End) is in flight.
  A released hold no longer cancels its in-flight resolver exchange: a
  cancelled exchange restarted the resolver and refused every press for 1 s.
- **Native playback** (`rt-core/engine/path_jog.hpp`): one jerk-limited speed
  profile on the path parameter, limited by every joint's velocity, jog
  acceleration and the machine `jog_jerk` through the chain rule; all joints
  start, slow and stop together. Release, Halt, input expiry (browser closed,
  OLP crashed), a stalled resolver (`path_buffer_underrun`), a path end and
  STOP's fence all stop **along the path**. OLP's End for a lane native
  already stops (its input expired) is refused Closed; that receipt is
  cleanup once native shows the generation fenced under the current grant,
  never a Stop (a cell once disarmed at ~390 mm/s on it).
- **Resolver gate.** Start measures the first 100-sample resolution: it must
  sustain twice the peak consumption (|twist| / spacing) and deliver a chunk
  within half the look-ahead time, else `cartesian_resolver_too_slow`. This
  applies to a local binary, an ssh `OLP_CARTESIAN_RESOLVER` and a Deck-local
  OLP alike. (Measured 62,000 samples/s on an M1 Mac; cell CPUs UNCONFIRMED.)
- **Monitoring.** Every 100 ms the measured pose (10 Hz status) is compared
  with the line through FK; above 2 mm off the line (owner decision, unverified)
  the journal records a warning. It never steers; rt-core following error stays
  the hard fault.
- **Evidence.** The journal has `cartesian path hold: begin` (line, held and
  measured start pose, limits, resolver latency), `speed` (changes > 10 %) and
  `end` (reason, travel, governing limits with durations, samples sent and
  consumed, minimum look-ahead, underruns, resolver p50/p99, monitor maxima).
  `JointJogState` adds `stopping`, `governing_limit`, `stop_reason` and
  `path_speed`.

`OLP_CARTESIAN_RESOLVER` is a command line and may run the cell's own motion
server over ssh (`env -u LD_LIBRARY_PATH ssh -o BatchMode=yes user@<cell>
/path/robot-v4-cartesiand`), with `OLP_CARTESIAN_RESOLVER_REPO` naming the tree
on that host; the persistent per-connection process then holds one ssh session.
Otherwise the existing canonical Motion Server lookup supplies a local binary.
A bounded step uploads Hermite knots (one per 20 ms with velocity) from the
axis's held command (`logical_target`), never from the measured position: the
native Start admits a first target within one count of the held command, and
a measured origin on a 13,107,200-count axis faults `start_discontinuity`.
An upload refusal or timeout is a refusal of that step and keeps the machine
armed; Arm clears a latched `reset_clears` fault before Enable. The preview runtime is independent.
Connect retires that preview process before returning. While a machine is
selected, preview start/intent/home/step/set-zero requests return `target_changed`,
so a second browser cannot reacquire the local preview controller behind it.

Joint and Cartesian holds share one hold slot and exclude each other. Release,
input expiry, disarm, session loss, target switch and STOP retire the same hold;
a path End stops along the path and returns after native closure. Path ends
render `joint_limit`, `jacobian_gate`, `cartesian_reach` or `ik_discontinuity`
beside the pad. Repositioning remains an explicit operator action.

Tests: `TestCartesianPathHold*` and `TestCartesianHoldRefusedWithoutPathJog`
(fake lane), `TestCartesianJogComposed` and
`TestCartesianHoldRefusedWithoutJogJerkComposed` (real resolver, SDK, rt-control
and simulator; `RTCORE_COMPOSED=1`), `TestPickerComposed/jog_target_switch`,
`TestCartesianRoutesIdentityMutationAndTargetSwitch`, and the level-1 harness
`tools/cartesianjog-harness` (real resolver + `path_jog.hpp` on 29 holds
recorded on a Rosie 1430 cell).

## Joint jog and move-by-angle on the selected target

The Jog panel (`MACHINE_FLOW.md`), shown and hidden with JOG in the toolbar
includes one row per described axis, with live public POS,
minus/plus hold buttons, a signed degrees input and GO. Its Joints heading
names the connected cell and its posture (`Drives <cell> (REAL, motion is
physical)`), and the table caption names the cell id (`Jogs the selected
robot: <cell>`). With no machine connected these controls are disabled with
a reason. The Cartesian pad follows the same selected machine; without a
connection it drives the isolated browser preview. These joint
controls use the dense-execution controller selected by the server and its
existing public grant; they never acquire or ARM implicitly. A matching target,
description, current session, held grant, fresh valid feedback and armed drives
are required. All adapter access goes through `rt-core/sdk/control`, over the
configured local socket or HTTPS with the configured client certificate.

The speed selector offers 5%, 10%, 25%, 50%, 75% and 100%, defaulting to 10%.
The server accepts only finite fractions greater than zero and at most 1. Jog
velocity is the signed fraction of the axis's described `max_velocity` times
the measured 0.75 drive reserve (`jointMoveVelocityReserve`, so 100 % never
commands the limit itself); other axes have zero requested velocity. Only one held axis is admitted. A second axis or a
changed direction/fraction is refused with `jog_mode_conflict`; release must
explicitly end the old hold before another begins (a begin during the released
hold's Halt ramp waits for it; see "Re-press while the previous hold stops"). Jog and prepared moves also
exclude other dense operations.
POS comes from `/v1/status` through the panel's existing dense-status poll.

`POST /api/offline-programming/v1/dense-execution/jog/{begin,update,end}` carries
`target_id`, `session_id`, `revision`, zero-based `axis`, `direction` (-1 or +1)
and `fraction`. `GET .../jog/state` returns the browser revision, jog generation,
sample sequence, reason and move state. The server captures input using its
monotonic clock. SDK producers retain the immutable grant, negotiated clock
incarnation and per-generation wire sequence; remote producers use the SDK's
dedicated calibrated WSS lane. Unqualified remote clocks remain typed refusals.
No browser timestamp or adapter grant token is accepted as clock or authority
evidence.

The unverified browser policy is one POST per 50 ms cadence and expiry after
100 ms without intent, including pending Begin. The browser keeps at most one
input request in flight and never replays a queue (superseded 2026-10-07
against a server that reports `pipelined_intents`: up to 5 in flight, see
"Intents keep their cadence on a slow link" above). Hold sends BeginJog, then
UpdateJog sequence 1 and subsequent monotonic samples. Pointer release, leave,
cancel, lost pointer capture, window blur and page hide stop the cadence and
send one EndJog. Missing two cadences ends the generation with
`jog_input_deadline_expired`. Adapter frame refusal or WSS loss also ends it.
End advances the browser revision even if Begin has not arrived, so a late
Begin/Update cannot restart a released hold. Stop, Disarm, lost grant and server
shutdown cancel pending work, revoke authority through urgent Stop, issue EndJog
for the original generation, and Release. A late Begin receipt is ended without
publishing a velocity. Native End/expiry applies the adapter's existing bounded
jog ramp; a failed cleanup requires Stop and uncertain Stop inhibits new work.

`POST .../dense-execution/move` takes the same target/session/revision, `axis`,
signed `degrees` and `fraction`. GO is admitted only for rotary axes described
in radians. The requested delta must not exceed the described range in either
direction, and its endpoint must stay inside the described soft limits. The
server mirrors the non-importable `tools/benchdrive` trapezoid/triangle planner:
it starts at the **held logical target**, uses the selected fraction of velocity
and the lower of described jog/max acceleration, retains benchdrive's 0.75
velocity reserve, and samples on the described daemon cycle. Rounding stretches
the profile; its last point is exactly the requested endpoint. Invalid geometry
or excessive point count is refused before preparation.
Motion and axis packets publish independently: planning waits for an axis
publication after admission, with motion held, before reading that target.

The move sequence is `prepare_trajectory` with the held fence, observed
`prepared` handle, `start_trajectory` with that handle, then public status showing
the matching handle and native command sequence, native Done/state 4, a consumed
or retired handle, and actual position within the described completion tolerance.
The axis publication must also follow the observed native completion packet,
preventing a subsequent GO from starting at a target from the previous motion.
Only that evidence returns `done`; an accepted Start alone does not. POS follows
public feedback throughout. Its completion deadline includes the actual prepared
sample clock, described settling deadline and the existing call budget, so a
valid slow profile is not truncated by the preparation request budget. Stop and
Disarm cancel the request independently of preparation, and request cancellation,
target/session loss or heartbeat/grant
loss also stop it. Failure retires the move through Stop and requests Discard
with the original fence; after revocation the adapter may refuse Discard because
Stop has already retired the handle. No failed or completed handle is replayed.
Unlike a hold, a prepared move does not depend on continued pointer pressure or
window focus.

The shared UI reason mapping preserves adapter codes alongside operator wording,
including `jog_not_ready`, `jog_mode_conflict`, `jog_stale_sequence`, input-age and
clock refusals, `jog_limited`, `outside_limits_outward` and unknown reason codes.
Jog reasons appear under the table; each move row shows its state or typed
reason, including `joint_move_limits`, invalid units, invalid description and
trajectory-capacity refusals. Network failure cannot appear as successful motion.

These controls command the selected robot when its backend is physical. Software
Stop, limits and deadman are not an emergency-stop or proof of physical standstill;
the cell's commissioned interlocks, braking and physical stop chain remain
required. This change was exercised only with fake adapters and local simulation,
never on a bench, Deck or robot host. Remote clock and physical stopping
qualification are not established by the simulation tests.

`internal/denseexec/joint_jog_test.go` covers hold/release sequence, deadman, delayed Begin,
admission, refusal, native feedback, Stop/Disarm and grant loss.
`internal/denseexec/joint_move_test.go` covers absolute profile bounds, preparation/start/completion,
refused preparation/start, late receipts, cancellation, endpoint and sequence
correlation. `TestJointJogComposed` requires bounded J1 position change and a
closed generation with stable counts. `TestJointMoveComposed` observes +30 degrees
and return using a simulation-only configuration copy in `rt-core/build`.
Both reuse the dense controller's composed fixture. UI tests in
`ui/src/execution/jointJog.test.ts` run actual hold/GO handlers, pointer/window
events, timers and HTTP edges, plus dense-panel source integration checks.

## Public backend configuration

The local stack and `start-offline-programming.sh` default to `rt_core`.
Explicit retired backend selections fail with `backend_retired` before startup.
Direct `serve` also defaults to `rt_core` and requires explicit configuration.
Select `serve --execution-backend=rt_core --rt-core-config=<file>` (or
`OFFLINE_PROGRAMMING_EXECUTION_BACKEND` / `OFFLINE_PROGRAMMING_RT_CORE_CONFIG`).
The selected public backend never constructs the legacy Commander, BridgeLease,
connected NATS monitor or connected execution/planning adapters. When enabled, the local simulator uses `OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core`
(the direct-session default as well). That selection retains the
Tesseract planning worker and starts isolated cartesiand with a verified simulation
binding; Cartesian intents use its public jog lane and actual native feedback.
Mixing connected legacy flags with
`rt_core` fails at startup; configuration failure never selects another backend.

The configuration JSON contains `socket` **or** `address` (HTTPS), `binding`
(`pair_id`, `revision`, `machine_sha256`), `deployment_sha256`,
`capabilities_digest`, and `axis_mask`. HTTPS additionally requires `ca_file`,
`certificate_file`, and `key_file`; the SDK verifies both server trust and client
credentials. Values must come from the commissioned deployment. Binding revision
is an explicit acquisition input, not a claim about the v4 pair service. No
socket, private credential or binding is sent to the browser. Description must
match the configured machine, deployment, contract and required capabilities.
The existing `--weld-planner-motion-origin` supplies immutable `.rdt` bytes.
Run the public backend server on Linux/WSL, as required by the current Go SDK.
Other platforms retain the legacy application build and explicitly refuse
`rt_core` selection.

## Choosing a machine

Set `OFFLINE_PROGRAMMING_CELLS` to a server-side JSON catalogue. The normal dev
stack writes `rt-core/build/dev-stack/olp-cells.json` with Local simulation; the
standalone launcher writes it under its local build directory. To append remote
cells, set `DEV_STACK_OLP_CELLS` to another catalogue before `dev-stack.sh start`.
With that variable unset, the existing dev-stack command baselines are unchanged.

```json
{
  "schema": "offline-programming.cell-catalogue.v1",
  "cells": [
    {
      "cell_id": "local-simulation",
      "label": "Local simulation",
      "role": "LOCAL SIMULATION",
      "models": ["rosie_1400_v3"],
      "execution": {"local": true},
      "requested_mode": "simulation"
    },
    {
      "cell_id": "<cell>",
      "label": "Test bench (Rosie 1400, nine motors)",
      "role": "TEST BENCH",
      "models": ["rosie_1400_v3"],
      "execution": {
        "address": "https://127.0.0.1:8443",
        "ca_file": "pki/ca.pem",
        "certificate_file": "pki/olp-dev.pem",
        "key_file": "pki/olp-dev-key.pem",
        "binding": {"pair_id": "<pair>", "revision": 1, "configuration_sha256": "<configuration digest>"},
        "axis_mask": 511,
        "expected_backend": "ethercat"
      },
      "requested_mode": "real"
    }
  ]
}
```

A remote entry resolves `rt-core/config/cells/<cell id>.json`, its cell
manifest. The supported
model ids are exactly `rosie_1400_v3` and `rosie_1420_v1`. Each remote entry must
list the single robot definition bound by its manifest's native `machine_config`.
The server compiles that configuration without contacting hardware, verifies the
definition and factory URDF pins, and requires the resulting configuration digest
to match authenticated live Describe. A supplied configuration digest must also
match this mapping. A nine-motor Rosie 1400 bench binds `rosie_1400_v3`, J1–J9
(mask 511); a six-motor Rosie 1420 bench binds `rosie_1420_v1`, J1–J6 (mask 63).
Use the cell's existing PKI test-root client `olp-dev`: copy only `ca.pem` and the
client certificate/key to the catalogue's `pki/` paths. A cell listed without a
native configuration is unavailable with `cell_configuration_missing` until a
native configuration and its own provisioned OLP listener binding exist.

The local entry uses the dev stack's explicit Rosie 1400 simulation URDF and
`OFFLINE_PROGRAMMING_RT_CORE_CONFIG` Unix-socket binding. It cannot carry an
address or credentials. Remote credentials are relative to the catalogue or
absolute Linux paths and must be the OLP principal's own files, never the Deck's.
Unknown models, duplicate cells, missing credentials and local/remote binding
mixtures fail startup with named `cell_*` reasons. Pair revisions remain exact
uint64 values on the server and are never passed through browser numbers.

The endpoint, pair ID and revision must equal the manifest's `rt_core` node.
Disagreements refuse with `cell_endpoint_mismatch`, `cell_pair_mismatch`,
`cell_pair_revision_mismatch` or `cell_configuration_mismatch`; an incomplete
manifest identity gives `cell_manifest_binding_missing`. Tunnel origin must
remain `https://127.0.0.1:8443`; changing the local tunnel port alone is refused.

`GET /api/offline-programming/v1/targets` returns labels, models, requested mode,
live backend, simulation flag derived from Describe, host or `unreachable` and a typed reason. It returns no credential
paths. **Cells → Connect to** posts the commissioned cell and catalogue model
to `/targets/select`. Describe checks identity, configuration and requested mode.
`simulation` requires the simulation backend; `real` requires EtherCAT.

The dialog facade uses `GET /dense-execution/cells` for authenticated Refresh,
`POST /dense-execution/cells` for matching commissioned address/PKI registration,
and `POST /dense-execution/target` with `{cell}` for selection. An empty cell
calls Picker Clear under selection admission: Stop, acknowledged Release, close
and retire pending holds, then invalidate the generation. Failed required Stop
or Release preserves selection. Unknown endpoints report "not a commissioned
cell"; GUI deletion refuses decommissioning. Registry and Fleet hold no separate
controller, transport or selection.

Pull cell settings copies the native target model and authenticated Describe
backend, host, configuration digest, axes and I/O. Changes invalidate plan
freshness. Models resolve through the existing catalogue function; no daemon
status field supplies workspace identity.

Selection is remembered by the server process, including across browser reloads.
It performs no acquisition, arming, Home or playback. An existing selected
session is stopped and released before replacement. A playing program refuses
switching with `cell_switch_while_playing`; concurrent admission can return
`cell_switch_busy`. An uncertain old Stop preserves the old selection with
`cell_previous_stop_failed`. Model and mode refusals are `cell_model_mismatch`
and `cell_mode_mismatch`; changed deployment identity is
`cell_configuration_mismatch`. Native Home, ARM, Load, Play and STOP remain
explicit actions, and STOP ends control. Cartesian hold-to-jog follows the
selected machine through this same grant and public jog lane. Its local resolver
computes velocities without opening a second controller connection.

Every mutating `/dense-execution/*` request carries `X-RT-Target-Generation`
and `X-RT-Target-Cell` from the rendered status target's `selection_generation`
(a decimal string) and `cell_id`. Each successful selection, including selecting
the same cell again, advances the generation. Missing or mismatched identity
returns HTTP 409 `target_changed` before backend admission; selection stays
locked through the request. This covers ARM, Enable, Home, Go Home, Play, Load
and joint jog/move routes. Stop, disarm, pause, heartbeat and jog stop/end remain
available without matching identity. Fetching a new status alone never retargets
the browser's next action: its displayed target must be updated too.

The title-bar backend and Arm pills follow the
selected description, independently of program mode (`MACHINE_FLOW.md`).
EtherCAT displays **REAL · PHYSICAL OUTPUT** in red before ARM; only a described
simulation backend and simulation flag permit a no-physical-output claim.
Unavailable posture reads **OUTPUT UNKNOWN** and is treated as real. Pending
switches retain the selected machine's posture. The browser simulator's own
arming is a separate `Arm preview` toggle beside the Cartesian pad in the Jog
panel, visible only without a connected machine, and never appears in the header.

| Control | Wording | Colour | Identifies |
| --- | --- | --- | --- |
| Header pill, no machine | `NO MACHINE` | Neutral grey | Browser preview only |
| Header pill, REAL target | `REAL · <cell> · ARMED` / `DISARMED` / `ARMING` / `STOPPING` / `PLAYING` / `STATUS STALE` | Red, filled while the drives are observed energised; amber dashed while stale | Named cell and physical motion; the token is the one armed fact of `MACHINE_FLOW.md` |
| Header pill, simulation target | `SIMULATION · <cell> · ARMED` / `DISARMED` / `ARMING` / `STOPPING` / `STATUS STALE` | Green; amber dashed while stale | Named simulation target |
| Cartesian pad toggle | `Arm preview` / `Disarm preview` | Neutral | Browser simulator jog |

The Arm step's button names its cell (`Arm <cell>`); on a REAL or unknown
posture a second click (`Confirm: energise motors on <cell>`) sends the request.
Unavailable target posture retains the red physical-output treatment; it cannot
imply simulation.

`TestCellCatalogueValidation`, `TestCellManifestModelComesFromConfiguration`,
`TestDevStackCellCatalogueDefaultAndAppend`, `TestPickerSelectionRefusalsAndStopBeforeReplacement`
and `TestCellTargetHTTPSelectionRememberedByServer` cover admission and HTTP state.
`TestPickerComposed` uses two local simulation daemons and two rt-control instances,
including an HTTPS listener with throwaway OLP PKI, to exercise both targets and
all switching refusals. UI coverage is in `ui/src/targets/cells.test.ts` and
`ui/src/execution/densePanel.test.ts`.

## Remote cell

Set `OFFLINE_PROGRAMMING_RT_CORE_REMOTE` to a JSON file on the Linux/WSL
development host. For a six-axis Rosie 1420 binding:

```json
{
  "address": "https://127.0.0.1:8443",
  "ca_file": "pki/ca.pem",
  "certificate_file": "pki/olp-dev.pem",
  "key_file": "pki/olp-dev-key.pem",
  "binding": { "pair_id": "bench-rosie1420", "revision": 1 },
  "axis_mask": 63,
  "expected_backend": "ethercat"
}
```

Credential paths are absolute or relative to this file, never to the launcher's
working directory. `address` is an HTTPS origin without userinfo, a path, query
or fragment. `expected_backend` is optional (`ethercat` or `simulation`); keep
`ethercat` for a bench/robot so discovery cannot silently accept a simulator.
The pair and revision must match the listener's commissioned binding. The
client certificate's `rosie-pair` SAN must name that pair. The mask selects the described axes;
63 selects J1–J6. Selecting a robot model in the UI does not establish authority
or commission its axes.

The launcher authenticates `/v1/describe` using these credentials, then writes a
fresh `rt-core/build/olp-remote/binding.*/olp-rt-core.json` for `serve`.
`machine_sha256` and `configuration_sha256` in `binding`, plus
`deployment_sha256` and `capabilities_digest`, come exclusively from that live
description. Do not copy hashes from a local compile into the input file.
Startup refuses with `remote_certificate_pair_mismatch`, `remote_describe_failed`, `remote_backend_mismatch`,
`remote_credential_missing`, or a named invalid-binding reason. The controller
rechecks the pinned description before acquisition. The launcher restarts an
existing OLP serve process to apply the binding and prints:
`dense backend: rt_core (remote https://127.0.0.1:8443 pair bench-rosie1420 configuration <live digest>)`.

An administrator provisions the listener PKI using
[`rt-core/host/remote-pki.sh`](../../../rt-core/host/remote-pki.sh).
On the cell, in an administrator shell, initialize only an unprovisioned PKI:

```bash
host/remote-pki.sh init /etc/rosie-rt-core/remote-pki bench-rosie1420
host/remote-pki.sh issue-client /etc/rosie-rt-core/remote-pki olp-dev
```

For an already provisioned cell, issue a distinct client from its existing CA;
do not replace the CA or reissue an existing principal. The issued files are
`clients/olp-dev.pem` and `clients/olp-dev-key.pem`. Transfer only those and
`ca.pem` to private storage on the development host. Keep the CA signing key,
server private key and CRL on the cell. The listener uses `--remote-listen`,
`--remote-ca`, `--remote-cert`, `--remote-key` and `--remote-crl`; the installed
service procedure remains the owner of its configuration. Revocation uses
`host/remote-pki.sh revoke /etc/rosie-rt-core/remote-pki olp-dev` and updates the
CRL checked by the listener on each handshake. Wrong-pair clients are refused
with `session_principal_mismatch` from the client certificate's pair SAN check
during TLS verification. Other TLS alerts, including revocation, retain
`rt_core_transport_lost`. Discovery disables curl's default configuration and
always supplies the commissioned CA, so a hostile `.curlrc` cannot disable trust.
No automatic reacquisition or
program replay follows a refusal.

For a bench behind a jump host, keep the cell listener on loopback and open the
tunnel from the development host in a separate terminal:

```bash
ssh -N -J operator@jump-host -L 127.0.0.1:8443:127.0.0.1:8443 operator@<servo-host>
```

The HTTPS origin remains `https://127.0.0.1:8443`; the default issuer's server
SAN includes that address. An SSH tunnel does not replace mutual TLS. For a
direct DNS/IP connection, provision the matching `ROSIE_RT_REMOTE_SERVER_SAN`
when initializing the listener PKI; never disable certificate verification.

Launch on the development host with explicit independent jog selection:

```bash
OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core \
OFFLINE_PROGRAMMING_RT_CORE_REMOTE=/secure/olp/bench.json \
OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core \
bash offline-programming/v1/start-offline-programming.sh
```

`OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core` keeps disconnected preview jog on the
existing local simulation socket and preserves local Tesseract planning. The local
`binding.env` supplies jog identity; the remote file supplies dense execution
identity independently. The remote launcher requires an explicit jog choice. Explicit retired jog
selection is refused with `backend_retired` by both the launcher and direct serve. A selected catalogue cell receives Cartesian jog
through its existing public SDK session, including the calibrated remote lane.
An unbound direct configuration refuses Cartesian jog. The browser sees only
the remote address host and described axes,
never credential paths, private keys or session authority.

With the dev stack, use `DEV_STACK_OLP_REMOTE=/secure/olp/bench.json bash
dev-stack.sh start`. It selects remote dense execution and explicitly retains
the local jog backend selected by `DEV_STACK_BACKEND`. `status` reads the saved
binding and prints the remote origin/pair even from a fresh shell. Stop OLP
before changing bindings. Without `DEV_STACK_OLP_REMOTE`, both existing stack
command baselines are unchanged. Local simulation, pendant and daemon members
continue using their local binding; only the OLP dense execution member changes.

`TestRemoteLauncher` executes this file format against a mock mutual-TLS
Describe server with production test-root PKI. `TestRTCoreComposedHTTPS`
starts the real simulation daemon and remote listener, exercises
Acquire → Enable → Arm → PrepareProgram → StartProgram → completion → Stop →
Release, and verifies revoked/wrong-pair refusals without acquiring authority.
These are development-host tests, not evidence of powered bench execution.

## Execution contract

The route is direct Go SDK `prepare_program` / `start_program`: existing
`internal/denseexec/session.go:463` already fetches, decodes and verifies the
complete `.rdt` and six-field planner identity before upload. This lets that
controller own one public grant without retaining a second NATS leader and arm
bridge. The alternative `motion-server/joint-trajectory/v1` `rt_core` control
plane is admitted, but is not selected by this implementation. The legacy
reviewed `execution.NATSController` consumes a different accepted-plan envelope
and remains disabled under `rt_core`; it is not reinterpreted as an `.rdt` upload.

This historical migration inventory describes retired commands and source paths;
the native replacement column records their disposition, not legacy startup instructions.
Sources below are relative to `offline-programming/v1/`. Line numbers identify
the implementation at the migration checkpoint; repeated metadata is defined once here.
`C` means `robot/v4/robot.<cell>.command`, `M` means
`robot/v4/motion-server.<peer>.status`, `R` means
`robot/v4/rtcore.<host>.status`, and `P` means
`robot/v4/motion-server.<peer>.plan`. Legacy request envelopes contain
`schema:robot.v4.robot-command.v1`, `command`, `command_id`, `sender_id`, `robot`.
Dense credentials contain `controller_boot_id`, `lease_id`, `leader_lease_id`,
`leader_fence_epoch`; motion adds `motion_intent_seq`. The reviewed v1 controller
also carries request/campaign identity. Public commands use the SDK-generated
request, `schema:rosie.rt-control.request.v1`, `operation`, request ID and the
acquired `session`/`generation`; the browser never supplies that fence.

| Legacy source | Subject and command/status payload | Public equivalent and selected behavior |
|---|---|---|
| `internal/denseexec/commands.go:239`, `:258`, `:275`; `internal/denseexec/bridge_lease.go:246`, `:429`, `:394`; `internal/execution/nats_controller.go:387`, `:429`, `:472` | C: `leader_acquire` with boot ID/TTL/campaign; `leader_renew` and `leader_release` with credentials. Bridge observes leader identity, freshness, campaign/fence and transition state on M. | SDK Acquire(controller,binding), StartRenewal/Renew, Stop then Release on `/v1/control`. No NATS leader authority is reused. Stop/renewal generation changes invalidate the prepared session. |
| `internal/execution/nats_controller.go:511`, `:550` | C: `leader_cancel` with request/campaign/boot identity; M: exact held leader check. | Cancel the in-flight operation context and Stop before Release. Public native/session authority is never inferred from M. |
| `internal/denseexec/commands.go:318`; `internal/denseexec/bridge_lease.go:323`; `internal/execution/nats_controller.go:233` | `robot/v4/rtcore.<host>.command.arm` or C: `arm`/`disarm`, `armed`, `runtime_mode`, `runtime_mode_epoch`; reviewed arm includes credential/campaign/intent sequence. R supplies mode epoch and enabled/armed masks. | Explicit Acquire, Enable(axis_mask), Arm; await logical readiness and observed armed state. Disarm is Stop then Release. Existing `/dense-execution/arm` endpoint is retained. |
| `internal/denseexec/session.go:625`; `internal/simulator/jog_authority.go:66` | Runtime mode and epoch are read from the dense daemon's authority or supplied by the isolated simulation fixture. No production OLP `command:mode` emitter was found. | Backend is a deployment property from Describe. No runtime `mode` operation is invented or sent; changing deployment requires explicit reconfiguration. |
| `internal/denseexec/commands.go:355`; `internal/denseexec/session.go:736`; `internal/simulator/jog_session.go:847`, `:928` | C: `go_home` plus credential/sequence; isolated jog GoHome/SetHomeReference mutate simulation state. No production OLP `command:home` or `command:hm35` emitter was found. | Saved-pose Go Home is unavailable, never renamed to homing. New `/dense-execution/home` calls public Home(axis_mask), observes HomeValidMask and commissioning completion while disarmed. UI labels this native Home. HM35 remains a separate consumer migration. |
| `internal/denseexec/session.go:463`; `internal/denseexec/ingest.go:220` | Fetch `/api/motion/dense/<digest>`; TCP length-framed `upload` with blob, then `preload` with the six identity fields. | Fetch/validate the same bytes; SDK PrepareProgram sends `application/octet-stream` to `/v1/program` with session/generation headers. Verify returned planner identity, source digest, normalised digest and axis mask; reject process-I/O programs. No silent sample edits. |
| `internal/execution/nats_controller.go:198` | C: `plan_preload`, plan revision, program ID, accepted-plan envelope and credential; M: exact applied intent and accepted identity. | This legacy gateway is unavailable in the selected backend. Dense execution uses the `.rdt` route above; no conversion from its accepted-plan envelope is fabricated. |
| `internal/denseexec/commands.go:286`; `internal/execution/nats_controller.go:306` | C: `play`, credential/intent sequence, plan/program IDs, program/trajectory digests, manifest/plan revisions; M or dense status: playback active. | SDK `start_program` echoes the entire returned Program.Identity, including both source and normalised digests. Completion requires matching identity and native start sequence plus observed `execution.state=completed`. |
| `internal/denseexec/commands.go:300` | C: `pause`, credential/sequence. | Unavailable public capability: Pause is disabled in the public UI; direct requests Stop and report `capability_unimplemented`. No replay/resume of a cancelled prepared program. |
| `internal/denseexec/commands.go:311`; `internal/execution/nats_controller.go:340`; `internal/denseexec/session.go:1108` | C: unfenced dense `stop`; reviewed stop has credential/plan identity and checks zero-hold application. Reviewed protective `disarm`; `io` with slot 5, on=false, intent sequence. | Public Stop is session-fenced, cancels in-flight work, precedes Release and invalidates the loaded identity. No independent weld OFF acknowledgement exists here. Torch-bearing programs remain refused; no fake RevPi readback. |
| `internal/denseexec/session.go:806`, `:1187`; `internal/denseexec/commands.go:185` | Dense `robot/v4/robot.<cell>.status` stream, or TCP `status` fallback: staged identity, executor state/cursor/error, authority, armed and axes. | SDK Status and EventsContext from `/v1/status` and `/v1/events?after=...`; generated ProcessStatus/Event go to the existing UI status/error path. Native time progression, incarnations, readiness, faults, execution identity and sequence are checked. |
| `connected_monitor.go:100`; `internal/connected/rtcore.go:42`; `internal/execution/nats_controller.go:845`, `:860` | M/P/R subscriptions. R envelope: schema, subject, host, component, published time; body: mode/output/PDO freshness, armed/enable/error/fault masks and RevPi readback. M/P: accepted plans and applied intents. | Connected legacy monitor is disabled. Public dense status uses generated logical validity/Home/readiness and core recovery/fault data. M/P planning semantics and physical I/O readback are not represented as native status. |
| `internal/targetplanning/nats_adapter.go:100`, `:205` | C: `plan_request`, request/command IDs, sender, plan revision, program ID and original program JSON; P: planner result, error, accepted plan. | Planning stays with the planner; rt-core accepts validated `.rdt`, not planner requests. Legacy connected planning is refused when selecting this backend; configured weld-planner blob retrieval remains supported. |
| `internal/simulator/jog_session.go:295`, `:442`, `:414`, `:670`, `:397`; `internal/simulator/jog_authority.go:24`, `:66` | Isolated `robot/v4/robot.olpsim1.command`: acquire/renew; fenced UDP arm/jog intents; synthetic `robot/v4/rtcore.olpsimrt1.status`; local pose/reference mutation and feedback packets. | The local launcher selects isolated cartesiand with `--backend rt_core`: its C++ client owns clock qualification and begin/update/end jog; the existing Cartesian resolver consumes native positions. Legacy feedback synthesis has retired; only native observations are used. |
| `internal/targets/descriptors.go:100`; `ui/src/targets/manifest.ts:39`; `main.go:295` | v4 manifest validates C/M/P/R, peer/host, axes and URDF linkage; startup constructs bridge adapters. | Public backend configuration is explicit. `manifestFromRTCore` uses generated Description and contract constants. Pair revision, factory hash, URDF and Home profile are null/unavailable. Logical positions retain declared units; the viewer poses the URDF only from axes described in radians, mapped by the reported model id (`ui/src/execution/livePose.ts`, "Viewport follows the connected machine" in `MACHINE_FLOW.md`), and fabricates nothing for other units. |

Failure handling is conservative: any rejected command, renewal loss or changed
grant, transport loss, native fault, lost readiness, stale observation, changed
incarnation, event loss, interrupted execution or missing browser heartbeat
cancels motion work and attempts Stop with a fresh bounded context. Release is
sent only after Stop acknowledgement. A failed Stop is reported as unacknowledged;
native grant expiry remains the last fence. SDK rejection labels become the
existing HTTP `error` code and diagnostic `detail`, and asynchronous failure is
visible in session `stop_reason`/`last_error` and `rt_core.reason`. No automatic
reacquire or motion replay follows a failure. Arm/load/native Home are explicit
new operator actions. Status success alone is never authority.

`APPLICATION_API.md` defines the live generated contract, including the
`subscribe_events` capability name and SHA-prefixed Program digests.
`APPLICATION_HANDOFF.md` assigns the pair, I/O, planner and deployment owners;
this controller does not replace them. `history/main-sync-2026-09-14.md` records
main's retained legacy status/leader fields, explicit native Go Home refusal and
inactive deployment selection. This change does not activate a machine or alter
those deployment manifests.

Validation entry points: `TestRTCoreIdentityCompletionAndExternalStop`,
`TestRTCoreLossStopsAndSurfacesReason`, `TestRTCoreRefusalStopsBeforeRelease`,
`TestRTCoreStopPreemptsPreparation`, `TestRTCoreManifestAndIdentityRefuseBeforeMotion`,
`TestRTCoreRenewalLossDuringPreparationPreservesReason`,
`TestRTCoreUnacknowledgedStopDoesNotRelease`, `TestRTCoreHomeAndReturnedIdentity`,
`TestRTCoreBackendSelectionFailsClosed`, and `TestRTCoreComposed`.
The composed test runs with `RTCORE_COMPOSED=1` after `make control sim`, starts
both real processes, and uses only the public SDK for execution. It observes
completion and an external operator Stop through the OLP controller, then checks
armed=0, enabled mask=0, non-executing state and released authority. The fixture
has stationary samples and no torch intent; it is not hardware/process-I/O proof.
`ui/src/targets/rtCore.test.ts` covers generated manifest/status mapping, missing
fields, invalid coordinates/Home, faults, typed refusal and event loss.

Remaining consumers: the Robot V4 UI backend, HM35 runner and example agent are
unchanged and still need their own public-client migrations. The OLP reviewed
connected gateway, saved-pose Go Home, physical weld I/O and
URDF-backed live pose rendering remain unavailable under `rt_core`. Retired backend selections are refused. The removed controller source is not
a rollback input; use the published release procedure in rt-core migration.


Remote cell timing comes from native Describe: `max_grant_lease_ns` and
`max_jog_input_age_ns`. The SDK measures Describe round trip, requests at least
three round trips plus renewal cadence (rounded up to whole milliseconds), and
retains the adapter's effective `lease_ms`. Renewal cadence is at most one third
of that receipt and uses a dedicated connection. A lease that cannot cover the
measured link is refused as `control_session_stale`; there is no reacquisition.

The controller measures Describe as well. Its call budget is effective lease
plus three measured round trips, and its unchanged-native-status age limit is
effective lease plus two measured round trips. Before acquisition the described
ceiling supplies the lease term; before any measurement the existing operation
watchdog bounds bootstrap. Slow Status/Describe cannot queue ahead of renewal.
Stop still precedes Release, and only observed native execution completes a run.

The nine-motor remote bench cell is exercised in simulation by
`TestRTCoreRemoteComposed` with every adapter reply delayed by 300 ms. The fixture changes only the native
cycle-lateness allowance to the existing unverified 20000000 ns simulation bound;
its axes, Home rules and cell control ceilings remain intact. Run with
`RTCORE_COMPOSED=1` after building rt-core. No hardware is contacted. Longer
granted leases increase worst-case unattended-stop delay after link loss; longer
jog ages permit stale velocity for longer before the existing expiry ramp.

## Bench cells

`bench_one_motor_1to1` and `bench_nine_motors_1to1` are bench-class models;
`rosie_1400_v3` and `rosie_1420_v1` remain arm-class models. A bench has no
URDF: the viewport says "Bench cell: no robot model. Live joint table below."
The disabled model selector names the bench. The Cartesian section says
"Not available on a bench without an arm model" and stays disabled. Choose,
Connect, Home, Arm and Drive use the same machine flow; the joint table uses
the described axes, live positions, hold-to-jog and signed move by angle.

Add an entry like this to the `cells` array in the stack's `olp-cells.json`
(for example `rt-core/build/olp-bench/stack/olp-cells.json`). Credential paths
are relative to that catalogue. The app reaches the bench's mutual TLS listener
directly, so the server certificate must name the address in `execution.address`.
`configuration_sha256` is the digest `rtctl compile` prints for the bench's
machine config. Keep the manifest's `activate` false for a manually launched
bench: the entry grants no authority until the operator connects and arms it.

```json
{
  "cell_id": "<bench>",
  "label": "Bench (one motor, 1:1)",
  "role": "BENCH",
  "models": ["bench_one_motor_1to1"],
  "requested_mode": "real",
  "execution": {
    "address": "https://<bench-host>:8443",
    "ca_file": "../pki-bench/ca.pem",
    "certificate_file": "../pki-bench/clients/olp-dev.pem",
    "key_file": "../pki-bench/clients/olp-dev-key.pem",
    "binding": {
      "pair_id": "bench-one-motor-1to1",
      "revision": 1,
      "configuration_sha256": "<sha256 from rtctl compile>"
    },
    "axis_mask": 1,
    "expected_backend": "ethercat"
  }
}
```
