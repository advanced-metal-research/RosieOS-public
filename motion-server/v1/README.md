# Motion Server v1

## Repository launch selection

The normal CLI launch selects `--backend rt_core`. Retired selections return
`backend_retired` before startup. Self-test planner invocations retain their fixtures.
`start-motion-server.sh` and `robot-v4-cartesian.service` select
`MOTION_SERVER_BACKEND=rt_core` through `/etc/robot-v4/motion-server.env`.
Rollback uses published release assets as described in
[rt-core migration](../../rt-core/docs/MIGRATION.md#repository-defaults-and-deployed-cutover). Native launch requires
`ROSIE_RT_CONTROL_SOCKET`, `ROSIE_RT_JOG_SOCKET`, `ROSIE_RT_PAIR_ID`,
`ROSIE_RT_PAIR_REVISION`, `ROSIE_RT_CONFIGURATION_SHA256`, `ROSIE_RT_AXIS_IDS`,
`ROSIE_RT_URDF` and `ROSIE_RT_URDF_SHA256`. The unit expects its helper under
`/opt/rosie/motion-server/v1`; install that helper with the unit.

These defaults do not modify cell manifests, installed units or release
selection. Deployed cells keep their manifest-selected release until cutover.
The dev stack supplies a local simulation binding through `local-rt-core.sh`.
Run `bash motion-server/v1/tests/dev-stack-smoke.sh` for composed jog and dense
execution. `dev-stack.sh` supports only the native simulation backend.

Canonical C++ Motion Server for RosieOS (`robot-v4-cartesiand`).

This tree is a first-class product at the repository root, peer of
`tesseract/v1` and `offline-programming/v1`. It owns catalog, Tesseract
planning, accepted-plan storage, preload/play authority, and the RTCore
motion source.

Build:

```bash
make -C motion-server/v1 all
```

The default binary lands at `$HOME/.rosie/motion-server/v1/bin/robot-v4-cartesiand`.
Offline Programming and `serve-wsl.sh` override `BIN_DIR` to
`motion-server/v1/bin/robot-v4-cartesiand` in the repo so local sim can find it.

NATS subjects stay `robot/v4/motion-server.<peer>.status` and `.plan`. Those
are protocol names, not filesystem paths.

Do not enable physical execution from this tree's OLP launchers.

## rt-core Cartesian backend

`--backend rt_core` selects the public C++ control client and local jog lane;
the normal launch default is `--backend rt_core`. The rt-core backend requires explicit
deployment bindings.

position input requires an explicit `axis` and exactly one finite `target_rad`,
`target_deg` or `relative_jog_rad`. Missing, partial or nonfinite joint input is
refused with `rt_core_position_input_unresolved` before run acquisition or native
preparation. Pose inputs are refused with that reason until an explicit pose
schema, frame and unit validation, bounded kinematic resolution and native
admission exist. Joint vectors have no admitted schema in this backend.

For example:

```bash
robot-v4-cartesiand --backend rt_core \
  --rt-control-socket /run/rosie-rt-core/control.sock \
  --rt-jog-socket /run/rosie-rt-core/jog.sock \
  --rt-pair-id "$PAIR_ID" --rt-pair-revision "$PAIR_REVISION" \
  --rt-configuration-sha256 "$CONFIGURATION_SHA256" \
  --rt-axis-ids "$ARM_AXIS_IDS" \
  --rt-urdf "$URDF_PATH" --rt-urdf-sha256 "$URDF_SHA256" \
  --udp-listen 127.0.0.1:9000 --status-output "$STATUS_PATH" \
  --nats-url "$NATS_URL" --robot-cell "$ROBOT_CELL" \
  --robot-command-subject "$COMMAND_SUBJECT" \
  --nats-status-subject "$STATUS_SUBJECT" --trusted-lan-leader-authority
```

`ARM_AXIS_IDS` is six comma-separated Describe axis IDs in URDF J1..J6 order;
each selected axis must use radians. Other configured axes remain unselected.
The URDF checksum is an independently configured deployment binding. The
existing Cartesian smoother, Jacobian resolver and singularity scaling consume
live logical joint positions from Status. A common vector scale then enforces
Describe's velocity caps. The backend acquires the configured pair, renews every
100 ms, enables and arms only on an operator request, and maps explicit Home to
`home`. Home never implicitly arms. Shutdown orders EndJog, Stop and Release.

The existing trusted-LAN NATS leader lease and v2 joystick lease/epoch/sequence
remain mandatory, including in simulation. Cartesian base/tool UDP intents use
the existing conversion semantics. NATS `arm`, `disarm`, `home` and `stop` are
handled here; applications must inspect `latest_processed_cold_*` for the
applied result rather than treating queue acceptance as completion. UDP arm,
disarm and Home also work. Source-admitted v1 position playback and GoHome use
the same execution owner. Joint jog and weld I/O remain unavailable. No v4
runtime commands or v4 status subscription are started by this backend.

A bridge run is identified by bridge boot ID, run ID, controller ID and
controller boot ID, under the exact live leader lease and fence. Its first
position command obtains an exact `position_execution_admit` challenge reply
over the configured NATS authority lane. The owner validates the original typed
command ID, source deadlines and remaining lifetimes. It projects expiry from
challenge send time; receipt time and leader renewal cannot extend the source
run. Raw source monotonic timestamps never authorize a native grant directly.
The source permit's legacy pair fields describe source admission only; native
authority always uses the explicitly configured pair ID, revision and
configuration SHA256, independently verified through Describe and Acquire.

The owner holds one native grant session at a time for that admitted run.
Startup or prior operator authority is fenced by Stop, renewal join and Release
before fresh bound Acquire. Further positions in the same live run keep its
session and generation. Stop, source expiry, authority loss, NATS disconnect,
native identity loss or uncertain transport retires the run: Stop must be
confirmed before Release. Failed Stop prevents replacement. Retired run and
session identities cannot be reused; after 256 retired runs the process refuses
new runs rather than forgetting tombstones (unverified capacity).

The existing v1 position envelope already carries a resolved joint-axis target
in radians. Relative targets resolve against fresh logical Status; unsupported
axes, changed URDF and requested/native position limits fail closed. The owner
builds two stationary Hermite knots from valid held `logical_target` values and
Describe's limits, then prepares and starts through the public C++ client.
Prepare and Start carry the run's native session/generation fence and an immutable
`Identity` with a fresh opaque command token. Raw receipts and handle events echo
that identity; Start checks it against the preparation. Completion requires the
same identity, grant and execution generations, incarnations, handle/sequence,
native Done and its consumed event. Event cursors must advance without replay or
loss. A recycled handle/sequence cannot complete another command.
Stop advances cancellation at subscriber admission. Prepare, Start, Enable and
Home recheck cancellation before each call; held receipts are cancellable so the
urgent Stop fence does not wait for them.
The owner is the sole writer and excludes jog frames during run playback.

GoHome requires the same previously admitted source run. Missing credentials and
a fresh, unproven bridge run return `rt_core_run_not_admitted`; source admission
must first occur through position. Explicit operator Home remains a separate
commissioning command. Within an admitted run GoHome explicitly
rotates the native session using confirmed Stop, renewal join, Release, fresh
bound Acquire, Enable and Home. This commissioning boundary is the only session
replacement within a live run and preserves its original expiry. It never
implicitly arms. Completion requires a fresh Home epoch, the selected Home mask
and matching daemon, adapter, configuration and grant identity. Stop cancels a
committed Home and retains cancellation even if a later Home observation arrives.
The owner starts one fenced Home receipt worker because the adapter drains
commissioning before replying. Status uses a separate observation client;
Stop uses the urgent lane and confirms from the fence receipt alone. A pending
Home receipt is abandoned with `rt_core_home_abandoned`; its worker owns an
independent client and cancellable socket, so destruction cannot wait for that
reply. The worker captures the original fence and never emits trajectory or jog data.

Typed terminal replay includes `native.binding` (pair ID, revision and
configuration digest), session, grant generation, run ID, native handle/sequence,
trajectory identity and execution generation,
Home epoch and native refusal result. Source expiry, Stop and disconnect retain
cancelled terminals; invalid source identity, retired runs, limit/readiness
refusals, event loss and native identity changes fail closed. Native reason
codes, including trajectory admission reasons 6 and 8, remain in the returned
native result; local preflight errors do not manufacture native receipts.

Each output uses the daemon's monotonic source clock and a deadline no later
than 250,000,000 ns after input reception, shortened by socket queue residence.
Only the newest queued intent is considered; queued safety requests still stop
motion. Unobserved jog publications cannot be overwritten. Renewal loss,
transport/refusal uncertainty, external Stop, faults, stale status, missing
readiness or changed coordinate/URDF identity ends the jog and calls Stop.
Fresh input cannot clear that inhibition; the operator must arm again.

The existing status file and NATS status subject carry `backend: "rt_core"`,
logical `axes` with readiness/Home/statusword, `fault_table`, `refusal`,
`typed_refusal`, and the complete public snapshot in `rt_core_status`.
Cleanup refusals are retained and `last_stop_confirmed` distinguishes a received
Stop acknowledgement from an uncertain outcome.
`v4_fields_available` is false and v4 mode/activation fields are null. For
P6-07, pendant/UI launchers must select this backend, supply the pair/URDF/axis
bindings, direct lifecycle intents to this owner, retain leader renewals and
decode these status fields. Retire the old runtime/bridge command route when
switching; no synthetic v4 feedback packets are provided. Deployment and
physical Home/stop qualification remain consumer work.

Build and test in WSL with Go on PATH and a tmpfs `TMPDIR` for Unix sockets:

```bash
make -C rt-core control sim
make -C motion-server/v1 BIN_DIR="$PWD/rt-core/build/cartesiand" OBJECT_CACHE_DIR= all test test-composed
```

`rt_core_jog_runtime_test` exercises fake-client ordering, refusals, source
freshness, readiness, grant retirement, single-writer trajectory completion and
explicit Home. `rt_core_stop_queue_runtime_test` checks source-clock projection,
typed replay and Home preemption. `TestNativePositionGoHomeThroughOwner` uses a
fixture source-permit responder on NATS and the real native simulator to check
position completion, typed replay, grant reuse, GoHome, committed Home Stop and
retired-run refusal. Composed tests use `TMPDIR` for fixture status and sockets,
retaining compiled binaries under `BIN_DIR`.
`TestCartesianJogThroughRTCoreAndExternalStop`
runs the real cartesiand, simulation daemon and rt-control, sends a fenced
Cartesian intent through a fixture NATS leader, observes joint-count motion,
checks NATS status, then verifies an external Stop and blocked subsequent input.
# Resolve-only Cartesian service

`robot-v4-cartesiand --resolve-only REPO_ROOT MODEL` pins one model at startup,
then reads JSON request lines from stdin (16384 bytes at most each) and writes
one JSON verdict line per request to stdout. This entry point constructs no
control client, acquires no grant, and sends no motion. The offline-programming
app uses it to resolve connected-machine holds through its own public joint jog
session. The existing simulation-driving invocation is unchanged.

Every request contains `model` (`rosie_1420_v1`, `rosie_1400_v3` or a cell's
own robot description id from the resolver's list; it must equal the pinned model), `frame` (`base` or `tool`), six-element `twist`
(m/s, rad/s), `pose` (rad), `lower`/`upper` (rad), `fraction` (greater than zero,
at most 1) and `input_age_ns`, plus an optional `operation`. The model loads from
`REPO_ROOT/robot_description/robots/<model>/robot.urdf`; its limits narrow the
supplied limits. Every response contains `accepted`, `reason`, six `velocities`,
`joint_limit_scale`, `singularity_scale` and `waypoints`.

Without `operation` the request is a velocity hold: the hold's twist is
multiplied by `fraction`, so the twist is the speed at 100 %. TOOL twists rotate
using forward kinematics of the supplied live pose. The existing damped Jacobian
solver and common joint/Jacobian scales remain in force. The response carries
`velocities` in rad/s. A refusal returns zero velocities with `joint_limit`,
`jacobian_gate`, `ik_no_solution` or `cartesian_input_invalid`.

`"operation":"move"` adds `delta` (one nonzero axis: metres or radians) and
returns every sample of that straight segment in `waypoints`, or refuses it whole.

`"operation":"path_extend"` streams the same straight line for a path hold, in
chunks. `pose` is the line origin q0, the held command at hold start; the line is
frozen at FK(q0), so it cannot drift with the measured arm. Extra fields: `axis`
(integer 0-2: X/Y/Z translation, 3-5: rotation about X/Y/Z), `direction`
(integer, exactly -1 or +1), `begin` (unsigned sample index of the first returned
sample), `count` (integer 1-256), `max_joint_step` (six finite values > 0, rad:
the largest per-sample joint step the caller's realtime lane accepts) and `seed`
(six rad, required when `begin` > 0: sample `begin-1`, which the caller already
holds; ignored when `begin` is 0). `twist`, `input_age_ns` and, beyond its
range check, `fraction` are ignored. `begin + count` may not exceed 1000000.
Sample i lies `direction * i * spacing` along the axis, with `spacing` 0.001 m
for translations and 0.25 degree (pi/720 rad) for rotations: the same unverified
sampling bounds as `move`, which shares this line construction and IK. BASE
translations move the TCP along the base axis, TOOL translations along the axis
of the TCP frame at q0; BASE rotations turn the TCP orientation about the base
axis, TOOL rotations about the q0 TCP axis; rotations keep the TCP position
fixed. Sample 0 is `pose` exactly. Each later sample is exact position IK of its
line target seeded from the previous sample on the nearest continuous branch,
then polished to about 1e-11 m / 1e-11 rad so third differences carry no solver
noise (Rosie 1430 fixture: under 11 rad/m^3, against ~6800 rad/m^3 unpolished).
Because sample amounts derive from the absolute index, a chunk requested with
`begin = a` and `seed` = sample a-1 reproduces the one-shot path bit for bit.

Each sample passes these gates in order; the first failing sample ends the
path and nothing past it is returned: `ik_no_solution` (IK failed or nonfinite),
`joint_limit` (outside the narrowed `lower`/`upper`), `ik_discontinuity` (some
|q_i[j] - q_(i-1)[j]| above `max_joint_step[j]`: a branch flip or near-singular
jump), `jacobian_gate` (Jacobian unavailable or at the hold's unverified
0.02 sigma hard gate). An accepted sample carries its singularity speed cap in
(0, 1] (1 above 0.05 sigma). The response adds `sample_scales` (one per returned
waypoint), `end_reason` (`""` when all `count` samples were produced, otherwise
the gate that refused sample `begin + len(waypoints)`) and `spacing`;
`velocities` are zero, `joint_limit_scale` is 1 and `singularity_scale` is the
lowest sample scale (0 with no samples). If sample 0 itself fails a gate the
request is refused: `accepted` false, `reason` that gate, no waypoints. If a
later chunk's first sample fails, the response is accepted with no waypoints
and `end_reason` set. Malformed requests return `cartesian_input_invalid`.
A 256-sample response is about 33 kB. Measured on an Apple M1 Max (macOS, -O2,
2026-10-06): about 62000 samples/s through the JSON service loop; the design
needs 800 samples/s for a 400 mm/s hold. Not yet measured on a cell computer.

`"operation":"fk"` returns `transform`: twelve numbers, the row-major 3x4 TCP
transform of `pose` (rotation, then the translation column in metres). It gates
only on a finite `pose`, never on limits: it monitors measured poses, for
example their distance from the commanded line. A refusal returns an empty
`transform`.

The caller must enforce Describe limits, freshness and motion authority itself;
this service provides computation, never permission to move. Holding a path
does not make it permission either: the caller still checks each streamed
sample against the live machine and stops on its own faults.
