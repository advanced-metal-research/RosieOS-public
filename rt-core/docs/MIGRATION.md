# Migration and cutover

Configuration files are organized by layer under [config/](../config/README.md): drive, robot, machine and cell configs.

## Cell bus topology

The optional cell `bus` block follows `protocol/cell-bus.schema.json`.
`unknown_slaves: "refuse"` is the default. Every responding position must be a
configured axis or an entry in `unused_slaves`. Each unused entry requires
`slave_position`, `vendor_id`, `product_code` and a nonempty `note`; optional
`revision` pins the SII revision, not CoE 0x1018:3. Sibling `_source` and
`_unverified` strings carry provenance. Axis positions and unused positions
cannot overlap or repeat. Missing declarations or mismatched identities refuse
bring-up before activation.

`unknown_slaves: "hold"` requires a nonempty `unknown_slaves_note`. Only definitions
`bench_nine_motors_1to1` and `bench_one_motor_1to1` qualify as benches automatically.
A robot definition, including the nine-motor bench and either Rosie model, requires
`hold_on_robot_acknowledged: true` and a nonempty `hold_on_robot_note` to use hold.
Normal robot cells retain refuse. Hold never relaxes the identity of an explicitly
declared unused slave. `rtctl inventory` reports non-axis SII identities and their
held, declared-unused or refusal disposition without sending those slaves SDOs.

The effective policy and sorted unused identities belong to machine identity;
notes and acknowledgement belong to configuration identity. The block does not
change deployment identity. With no block, compiled bytes and all identity pins
remain unchanged. Explicit empty refuse has the same effective machine identity,
while adding the block changes the configuration digest. Held devices discovered
on the bus are observations and do not alter the configured machine digest.

The daemon leaves non-axis slaves unconfigured in PREOP: no PDO registration,
startup SDOs or outputs. IgH 1.6.9 `master/master.c` requests PREOP when reserving
the master, but also raises its selected DC reference clock to OP. For hold and
declared-unused policies the daemon explicitly selects a configured axis as the
reference clock, including when DC synchronization is disabled. An unused slave
observed in SAFEOP or OP refuses readiness until PREOP or INIT is observed.

The initial inventory reads cached SII once per non-axis device. The non-RT
diagnostics publisher compares IgH's cached SII observations subsequently; it
does not issue CoE identity uploads. Status and describe expose `bus.held_slaves`,
including retained startup identities, current identities, presence and AL state.
An admitted unused slave disappearing or changing identity logs
`policy=log_and_continue_axes`; it contributes neither outputs nor working
counter. Axes continue only while their own online/OP and complete-exchange checks
remain satisfied. A new undeclared position under refuse still blocks readiness.
Physical removal that interrupts an axis's exchange retains the existing fault.

Counter units matter: the simulator's existing model reports two completed
directions per axis (six axes: 12/12). Live IgH LRW increments twice per output FMMU
and once per input FMMU (`master/datagram_pair.c`), giving 18/18 for six axes with
one input and one output mapping. The earlier 18/12 log used an incorrect expected
counter; the three unused drives do not explain the 18. Raw IgH counter values
are preserved, and `EC_WC_COMPLETE` remains authoritative for exchange readiness.

## EtherCAT host profiles

`host/ethercat-foundation.sh` selects the supported tuple from `/etc/os-release`
and `uname -r`, then checks any supplied manifest contract against that tuple.
Unknown OS/kernel combinations, architectures other than arm64 and incorrect
archive digests fail before provisioning. Explicit kernel contracts still refuse
kernel drift. The legacy `rosie-rt-core-revpi-igh-v1` name remains an alias whose
receipt label and recorded hash fields are unchanged.

| Profile | Host match | IgH | Header package | Driver build / selection |
|---|---|---|---|---|
| `revpi-debian12` | Debian 12, `*-revpi*-rpi-v8` | 1.6.9 | `linux-headers-$(uname -r)` | generic only |
| `rpi5-debian13` | Debian 13, `6.18.*-rpi-v8-rt`, PREEMPT_RT build string | 1.6.12 | `linux-headers-rpi-v8-rt` | `--enable-generic --enable-hrtimer`; `DEVICE_MODULES=generic` by default; selecting macb also builds `--enable-macb` |

Set `ROSIE_RT_FOUNDATION_DEVICE_MODULES=macb` only for an explicit Pi driver
qualification. Both profiles compare generated header release and module vermagic
with the exact running kernel; installing a header metapackage for another kernel
does not satisfy this check. Native macb claims its NIC without registering a Linux
netdev, so its attached MAC and link are checked through `ethercat master` when
`eth0` is absent. The Pi PREEMPT_RT check accepts the build string; this host's
missing `/sys/kernel/realtime` is not interpreted as a non-RT kernel.

The [Raspberry Pi trixie arm64 package index](https://archive.raspberrypi.com/debian/dists/trixie/main/binary-arm64/Packages.gz)
confirms the RT-specific header metapackage. The Pi pin matches the provisioned
installation that reached OP. It enables hrtimer and leaves cycles disabled:
`--enable-cycles` fails on arm64 with `cpu_khz` undeclared. Archive selection is:

| IgH | Selection | Archive | SHA-256 |
|---|---|---|---|
| 1.6.9 | Default builder / RevPi | [archive](https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.9/ethercat-1.6.9.tar.gz) | `2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3` |
| 1.6.10 | Explicit builder only; Pi 5 macb / kernel 6.18 support | [archive](https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.10/ethercat-1.6.10.tar.gz) | `b31387b19f8e1d8ced5d4c43f818ffa77a6333c2627782490d1dd1799391d1cc` |
| 1.6.11 | Explicit builder only; receive ordering and oversized datagram fixes | [archive](https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.11/ethercat-1.6.11.tar.gz) | `a68e79603a227ad8f3aca75cdf6abca663a3b7ca50a96faaf1ba020772fde558` |
| 1.6.12 | Pi profile default; retains those fixes and adds CCAT fixes | [archive](https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.12/ethercat-1.6.12.tar.gz) | `6aa49381da3daf08131913c3fade8fd24b650836567459ef1d35991da1549310` |

`ROSIE_RT_FOUNDATION_PROFILE=rpi5-debian13 bash tools/build-igh-userlib.sh`
selects the matching userspace archive. An explicit version argument or
`ROSIE_RT_IGH_VERSION` is also accepted, but a conflicting profile/version pair
fails. With no selection, the existing 1.6.9 archive path, prefix and build flags
are retained. Pi live builds use `ROSIE_RT_IGH_VERSION=1.6.12` and the selected
prefix's pkg-config directory. Packaging records the selected version and archive
digest in `rt_package.json`. Host installation support does not establish G1 timing
or powered-motion qualification; see [Local mini bench](BENCH_TESTING.md#local-mini-bench).

## Repository defaults and deployed cutover

Local development and current consumers select rt-core only. P8-11 removed the
old controller source and selectable backends. Rollback is the published
`robot-stack-v0.2.0` release tarball with its `rtcore-v0.2.0`,
`rtcore-bridge-v0.2.0` and `pair-owner-v0.2.0` component assets, or the recorded
v0.1.0 release set installed on the legacy v4 cells. Preserve their exact
receipts, configurations and inhibited rollback procedure; do not rebuild a v4
release from this branch or reinterpret it as native. Those cells retain their
installed releases pending approved native configurations. The default-flip
table and pendant discussion below record the earlier checkpoint and do not
offer supported legacy selections after retirement.

For release rollback, verify the published checksums against the retained
receipts, keep motion inhibited, and stop the native services before bus
handover. Install the verified release binaries into the versioned slots named
by those receipts using the retained release installation procedure; restore
that release's units and configuration on each recorded host before unmasking
its v4 units. The current deployment CLI refuses v4 manifests with
`reason=backend_retired` and cannot perform this installation.

| Component | Selection | Repository default | Deployed cells until cutover | Flip back |
|---|---|---|---|---|
| Local stack | `DEV_STACK_BACKEND` | `rt_core`: rt-sim, rt-control, dense daemon, virtual bridge, OLP | Not deployed | Stop stack, start with `DEV_STACK_BACKEND=legacy_v4` |
| Stack simulation | `local-rt-core.sh`, `ROSIE_LOCAL_RT_DIR`, `ROSIE_LOCAL_RT_BUILD` | Simulation configuration compiled from `simulation-program.json` with simulation Home recipes; public control/jog sockets | Existing manifest configuration | Legacy switch restores mujoco, velocity controller and sim-cell keeper |
| OLP dense launcher | `OFFLINE_PROGRAMMING_EXECUTION_BACKEND` | `rt_core`, direct Go SDK; generated `OFFLINE_PROGRAMMING_RT_CORE_CONFIG` | Existing release selection | `legacy_v4`; supply legacy dense ingest/NATS binding |
| OLP local jog | `OFFLINE_PROGRAMMING_JOG_BACKEND` | `rt_core`, isolated Cartesian owner on the simulation adapter | Local-only route | `legacy_v4`; `DEV_STACK_BACKEND=legacy_v4` sets both OLP defaults |
| OLP direct serve | `--execution-backend`, `--rt-core-config` | `rt_core`; explicit config required | Existing release selection | `--execution-backend legacy_v4`, omit native config |
| Cartesian daemon | `--backend` | `rt_core`; self-test planner fixtures remain legacy | Existing release/manifest arguments | `--backend legacy_v4` |
| Cartesian helper/unit | `MOTION_SERVER_BACKEND`, `/etc/robot-v4/motion-server.env` | `rt_core`; explicit socket, pair, digest, axis and URDF bindings | Installed release/unit unchanged | `MOTION_SERVER_BACKEND=legacy_v4` |
| Dense daemon | `--backend`, JSON `backend` | `rt_core`; supplied config still requires explicit backend and identity | Existing release/manifest arguments | `--backend legacy_v4` |
| Dense helper/unit | `JOINT_TRAJECTORY_BACKEND`, `/etc/robot-v4/joint-trajectory.env` | `rt_core`; `JOINT_TRAJECTORY_DAEMON_CONFIG` supplies identity | Installed release/unit unchanged | `JOINT_TRAJECTORY_BACKEND=legacy_v4` |
| Virtual bridge direct CLI | `bridge start --backend` | `legacy_v4`, unchanged wire | Local-only | `--backend legacy_v4` |
| Virtual bridge local UI | `v1 dev --backend`, `DEV_STACK_BACKEND` | `rt_core`; fixture remains explicit and command-disabled | Local-only | `--backend legacy_v4` |
| Real pendant profile, both deploy copies | Node `rt_core` configuration and appended profile fields | Complete selected config emits nine fields; absent or explicitly staged config preserves legacy; partial selected config refuses deploy | Existing deployed profile unchanged | Select the approved legacy manifest; `activate: false` keeps staged rt-core inactive |

The stack records its backend and reports native/control/jog socket paths with
`status`. Stop before changing backend. WSL sockets must be on Linux tmpfs:
set `TMPDIR=/dev/shm/rt-core/defaultflips`. Build/config outputs stay under
`rt-core/build`; simulator and adapter have separate logs, ready files and pids.
OLP discovers the running default stack adapter or reuses an explicit
`ROSIE_LOCAL_RT_BUILD` binding. A standalone launch
owns its simulation children and stops them with the launcher. Explicit missing
bindings fail closed. Cartesian input keeps the existing leader/intent fencing;
the local jog launcher first commissions required axes through public Home on
the verified simulator, using its own grant and observing Home before Stop and
release. It refuses an occupied grant. Once coordinates are established,
cartesiand resolves it and sends public jog frames. Feedback is native position,
not legacy UDP integration. A release is a Stop boundary; a new hold requires Arm.
Only one consumer can hold the public grant. Stop local jog before acquiring
dense execution. OLP's dense SDK route and the dense
daemon are alternative execution owners, not two owners of the same program.
The OLP browser remains at port 5189, API 8794, with the same origin storage and
rails/motion environment rules.

### Pendant deploy binding

The cell's servo-authority node supplies `rt_core.pair_id`, `pair_revision` and
`configuration_sha256`, plus `rt_core.pendant` containing `endpoint`, `ca`,
`certificate`, `key`, `crl` and `principal`. The digest is the independently
compiled configuration digest, not the package or machine digest. `endpoint`
must be an HTTPS adapter origin; the four TLS paths are absolute paths on the
Deck. No credential material is copied by this profile change.

Both deploy copies append, in the existing pendant parser's order:
`rt_core_endpoint`, `rt_core_ca`, `rt_core_certificate`, `rt_core_key`,
`rt_core_crl`, `rt_core_principal`, `rt_core_pair`, `rt_core_revision`,
`rt_core_configuration_sha256`. The first ten fields remain byte-identical.
Absent `rt_core`, or explicit `rt_core.activate: false`, produces exactly ten
fields and retains legacy jog. This preserves the existing staged nine-axis bench
manifest until cutover. Multiple selected native configurations, missing fields, invalid digests/revisions and profile
delimiter injection refuse deployment rather than silently selecting legacy.
SteamOS OpenSSL provisioning and installing the approved TLS material remain
follow-ups; no Deck, robot or bench provisioning is performed here.

Validation: `bash motion-server/v1/tests/dev-stack-smoke.sh` runs
`TestLocalRTCoreJog` and `TestDevStackDenseProgram` against stack members, with
public jog sequence/count observations and dense execution generation/endpoint
checks. `TestNativeJogLauncherRequiresCompleteBinding` covers launcher refusal;
`TestLocalLauncherAdapterSelection` covers borrowed adapter reuse, unavailable
bindings and the single legacy switch;
`TestDevStackStatusUsesSavedBinding` covers persisted backend/socket reporting;
`TestSteamDeckDeployRTCoreProfile` covers both deploy encoders. Existing component
and legacy-wire tests remain separate regression checks.

**Status at `403063dc`.** P1-02/P1-03 provide the Go compiler and machine/deployment identities; P1-05/P1-06 provide Go/C++ application clients; P7-01/P7-02/P7-03/P7-05 provide packaging, service configuration, restart fencing and install/rollback checks. P1-09/P1-10 parity and Python removal and P8-11 legacy retirement are verified; P6-02 is verified and P6-03/P6-05/P6-07 are merged with acceptance pending; P8-06 cutover acceptance remains backlog. The sections below this line preserve the dependency and cutover procedure from their named checkpoints; current implementation is in [ARCHITECTURE.md](ARCHITECTURE.md) and [APPLICATION_API.md](APPLICATION_API.md).

Reviewed baseline `895e249`, updated 2026-09-12. [P2_CORRECTIONS.md](history/P2_CORRECTIONS.md)
records the subsequent implementation and evidence. The root `rt-core` is the standalone
source/build owner on `codex/standalone-rt-core`. Its native executor and local
application service are development implementations with open correctness and
integration work. At that baseline active Rosie deployment still selected the
existing runtime; the current selection is in
[Repository defaults and deployed cutover](#repository-defaults-and-deployed-cutover).
No physical master, drive or service was activated by this migration work.

[IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) is the authoritative P0-P8 backlog
and G0-G2 gate definition. [CONTROL_SURFACE.md](history/CONTROL_SURFACE.md) is the target;
[APPLICATION_API.md](APPLICATION_API.md) describes the current transitional API.
Do not interpret old passing software results as permission to skip new gates.

## Implementation sequence

1. Record the minimal process/authority/clock and R21 policy decisions in
   [KICKOFF.md](KICKOFF.md); preserve baseline evidence and existing assertions.
2. Complete P2/M1 localized correctness fixes using the existing Python harness:
   readiness, checked bus/Stop results, re-arm recovery and handle cancellation.
   Extract only the start/output/bus seams those tests require.
3. Complete P3 independent jog/bulk ingress using that same oracle. Preserve
   origin expiry through the selected public/native transports and prove G0.
4. P7 host/service/access/restart/affinity prerequisites, the interim public
   fault-reset path and applicable brake sequencing must accompany inhibited
   drive checks for G1, without waiting for the language port. Then qualify
   only the motion capabilities enabled for that bench configuration.
5. Port P1 against the corrected behavior, prove old/new harness parity and
   replace Python. Extend the generated schema with each settled P3/P5 contract.
   Complete P4/P5 in individually tested extractions, not one broad rewrite.
6. Integrate the real motion server, pendant/UI/agent, authenticated authority and
   cell I/O. Finish package/service operation, physical/soak tests and rollback
   evidence. Preserve per-axis home/absolute qualification.
7. Require complete Python absence and all G2 integration/qualification evidence
   before the cutover transaction. Close G2 only after its acceptance checks.

After the main integration at `1d5db43`, the native consumer is
`motion-server/joint-trajectory/v1/src/rt_control_executor.hpp`, using the public
C++ client in `rt-core/clients/cpp/include/rosie/rt_control_client.hpp`.
The retired Python execution wrapper is not a current deployment entry point.
This software integration does not complete step 6. Existing `.rdt` identity/digest semantics remain
supported, but segment-boundary compatibility must be resolved explicitly before
claiming all current planner outputs execute. The legacy `robot.v4` schema name
is not a source dependency on old robot code.

rt-core replaces `robot/v4/realtime-core`. Delete the old runtime after its
motion-server, pair-owner, pendant, cell-I/O and deployment consumers are cut over;
the main merge does not establish that acceptance. Cartesian pendant input belongs
to the motion server, which must convert it to joint-space velocity and use the
public rt-core jog lane. rt-core remains joint-space only. The selected nine-axis bench
manifest retains `activate: false` and no legacy process intents; the OLP and
legacy v1 motion server still require their explicit consumer migrations.

## Configuration

### A6-EC absolute linear mode (2026-10-19; P8-45)

This migration is in progress on `feat/j5-j6-wrist-limits`; it has not been
released to a cell. Every A6-EC axis moves to absolute linear mode
(`C00.07 = 1`), standard drive gearing `0x6091:01/02 = 1/1`, and the
robot-definition gear ratio applied in software. HM35 writes `607C = 0`; its
other writes, waits and ordering do not change.

The count model becomes continuous end to end. The profile declares
`raw_multi_turn_encoder|3`; startup, Home, retained Home and verified-delta
compare complete signed counts exactly. Rotation mode, wrapped feedback and
commands, batteryless single-turn mode 3, `write_sdo_wrap_fraction`,
`single_turn_frame_offset` and the public `feedback_wrap`, `command_wrap` and
`anchor_period_counts` fields are retired. Old anchors fail source/tracking/Home
identity and are never upgraded.

The mode change is effective only after a drive power cycle. Before merge, a
the single-axis mini bench's powered-only release must prove mode-1 continuity,
HM35 `6064 = 0`, ±719-degree travel and reversal, and Stop from servo-rate
evidence. Its sourced absent battery forbids any power-cycle-retention claim;
after a battery box is fitted, retention and battery-removal refusal are
qualified separately.
After that gate and owner approval, each migrated cell receives its own full
release and operator HM35 on every joint, one cell at a time. Rollback does
not restore the old coordinate evidence or reverse a drive setting merely
because a key is omitted.

Tracking: P8-45 and
issue #160 (internal tracker)
for remaining unrelated hard-coded A6-EC literals.

### A6-EC telemetry presets

Since 2026-09-15 `config/drives/a6ec.json` selects fixed TxPDO 0x1B04
(6916) for robots, matching `a6ec_bench_1to1` at 1000000 ns cycles (1 kHz).
The daemon writes the assignment in PREOP, verifies every entry and rescans IgH
automatically. A nine-drive bench accepted it on 2026-09-15: nine AS715N
drives, all reaching OP with zero unavailable readbacks.

0x1B04 provides signed 32-bit drive velocity and following error; cyclic
`di_bits` is zero and `di_valid` is false. Drive digital inputs are mailbox
display observations. Safety reactions belong to the hardware stop chain and
cyclic cell I/O (P6-05). Select `a6ec_1b02` **when wired drive digital inputs
must be cyclic**: it preserves the previous 0x1B02 layout, with valid fresh DI
and unavailable drive velocity/following-error fields.

For a flat configuration set `axes[i].profile` to `a6ec_1b02`. For a robot-backed
cell, select it in the robot definition's `defaults.drive_profile` or a joint's
`drive_profile`, revise that definition and update the cell's `robot.sha256`
pin before recompiling. A cell cannot override the robot's drive-profile policy.
The preset change updates configuration and machine identities; it does not
change coordinate identity or independently require Home. Existing coordinate,
drive-identity and anchor-restoration checks still apply.

The 1:1 `config/machines/a6ec-bench.example.json` now uses `a6ec_bench_1to1`, matching
`config/machines/bench/nine-motors-1to1.json`. Its declared unverified acquisition allowance
is 33571 counts (`ceil(262144 * 128000000 / 1000000000) + 16`), below the
65536-count half-period. Successful compilation does not qualify powered motion.

### Robot definitions

A robot is a description directory, `robot_description/robots/<model_id>/`
at the repository root (docs/ROBOT-DESCRIPTION-CONTRACT.md): the URDF, the
meshes, the planner's `config.json` and `spheres.json`, and rt-core's own
definition of the robot, `rtcore_definition.json` (schema
`rosie.robot-definition.v1`, with a `robot_id` and a model `revision`
independent of the cell pair revision). `robot_description_manifest.json` registers the files
the robot is made of; the identity is a SHA-256 over the manifest and every
registered file, `robot_description_sha256`, which `go run rosieos/robotdesc/cmd/identity`
prints. A machine selects its robot with `robot.robot_description.path` (the directory,
repository-relative) and pins the identity with `robot.robot_description.sha256`. A changed
description is a changed identity and requires a new pin; the compiler refuses
a mismatched pin before publishing compiled output, and a definition whose
`robot_id` is not the directory's name. Numeric model values carry `_source`
or `_unverified` provenance siblings.

The compiled configuration snapshots the description whole -- the manifest,
every registered file and this machine's `machine_planning_calibration.json` (`robot.machine_planning_calibration`, a file
under the component's `config/` tree, or the model's empty document when the
machine pins none) -- and the adapter serves it under
`GET /v1/resources/<sha256>`, listed in Describe's `robot`. A runtime package
carries every description under its own `robot_description/robots/`
(`tools/package-runtime.sh`), so a cell compiles without a checkout; `rtctl`
reads the description from the component when it carries one and from the
repository otherwise, and the pin decides either way. A missing or tampered
description is `robot_description_unavailable` or `robot_description_mismatch`;
a `machine_planning_calibration.json` that moved under its pin is `robot_hash_mismatch`.

Every A6-EC joint additionally requires `encoder_battery: "present" | "absent"`
and a nonempty `encoder_battery_source` string. Add an
`encoder_battery_unverified` sibling when its battery box is not confirmed.
This includes joints declared absent by a cell; profiles and cells cannot
supply a missing battery fact.

| Definition field | Linear-mode result | Battery warning policy |
|---|---|---|
| `encoder_battery: "present"` | `C00.07 = 1`, absolute linear mode | None tolerated for motion |
| `encoder_battery: "absent"` | `C00.07 = 1`, powered-only exact-count operation; no retained Home after power loss | Only the profile's exact current ALF9.0 pair is a visible tolerated warning; Er20.8/Er20.9/ErA0.1 and unknown/stale pairs fault |

The drive profile pins `startup_defaults.a6ec_encoder_position_tracking_mode`
to 1. The profile's `startup_schema` bounds remain authoritative: equal `min`
and `max` pin the value, and any contradictory mode refuses compilation.
The effective startup value is also `tracking.mode` in coordinate identity.
Every migrated A6 axis changes coordinate and machine identities. Changed
definition hashes and cell bindings also change configuration/deployment digests.

The A6-EC manual,
section 5.1 (printed page 118), specifies 131072 pulses per motor revolution.
Sections 5.1–5.4 (pages 118–123) distinguish fixed-travel absolute linear mode
from the retired rotation and batteryless single-turn modes. A6 linear mode
retains `wire.counts_per_rev=131072`; the robot gear is software scaling, not a
wrap period. rtctl refuses a declared travel whose continuous scaled counts do
not fit signed 32-bit position objects with the configured margin.

The description's `robot.urdf` is authoritative for geometry, mapped joint types
and URDF limits; the definition maps its logical joints to the URDF joint names
(`urdf_joint`) and never repeats a URDF value. A bare-motor bench is a
description with no `robot.urdf` -- only its manifest and definition -- and uses
its own mechanical travel and direction convention. The current compiler admits
independent bounded revolute URDF joints and refuses unsupported joint types or
mimic joints.

| Layer | Owns |
|---|---|
| Robot definition | Joint mapping (`urdf_joint` and logical names), reductions, motor encoder counts, logical signs, Cartesian participation and default policies for drive profile, Home and coordinate evidence. |
| Drive config (`config/drives/<name>.json`) | Object descriptors, position semantics and startup defaults, including the PDO layout, supported startup objects and drive-specific Home/coordinate policies. |
| Machine config (`config/machines/<machine>.json`) | Bus and runtime settings, slave positions, explicit absent-joint declarations in `robot.absent_joints`, `robot_joint`, `slave_position` and narrowing overrides. |
| Cell config (`config/cells/<cell>.json`) | Deployment binding: repository-relative `machine_config`, its compiled `configuration_sha256` pin, pair identity, dense axis order, listener and package selection. Not part of the compiler digest. |

Machine configs cannot override gearing, sign, wire counts or startup objects.
Machine position and velocity limits can narrow the model bounds, and machine
Home/evidence policy can strengthen the defaults. Widening a bound is a typed refusal that includes
the requested value, model bound and logical unit. Every definition joint must
be mapped once or explicitly declared absent. The compiler derives the native
scaling and startup values from the definition and profile. For A6 linear mode,
the drive receives standard gearing 1/1 and software receives the robot
reduction.

The compiled output distinguishes four digests:

| Digest | Meaning |
|---|---|
| `machine_sha256` | Effective native semantics: derived scaling, drive settings, logical limits and policies. |
| `deployment_sha256` | Deployment fields, including the robot reference and its description identity. |
| `configuration_sha256` | Binding of the configuration, profiles and the pinned description. This is the adapter/client configuration identity. |
| `coordinate_identity_sha256` | Per-axis saved Home binding to count semantics, emitted with canonical fields in `coordinate.identity.json` and axis status for every compiled axis. Pair identity is bound separately by the adapter's anchor record. |

Coordinate identity is independent of configuration pair metadata. Robot expansion
takes encoder resolution and gearing from the pinned definition. A6 gearing,
tracking mode and reference direction come from effective startup declarations.
Every axis requires explicit `motor_encoder_counts_per_rev` and
`coordinate_evidence_mode`, including simulation fixtures. Wire scaling never
supplies a missing encoder declaration, and evidence mode has no implicit default.
Other physical profiles also require `gear_ratio` (`numerator`, `denominator`)
and `position_tracking_mode`; synthetic simulation retains internal gearing 1/1
and continuous tracking. Required-home axes require a declared Home procedure;
optional-home axes may retain `home.policy=none`. Deleting a required declaration or setting it to null refuses
compilation with the axis and coordinate key; startup defaults are declarations
and may be overridden explicitly. Configuration pair metadata remains irrelevant.

Coordinate identity continues to cover the profile-declared anchor source,
drive identity, encoder resolution, software gear ratio, logical direction,
Home policy, tracking mode and wire scaling. It no longer carries a
rotation-mode drive-reference ratio or wrap period. SHA-256 covers the
compiler's sorted canonical `key=value` lines; record the final key inventory
and resulting digests only after the concurrent implementation settles.
`anchor.source` comes from the drive profile: A6-EC linear mode uses
`raw_multi_turn_encoder|3` for the continuous `0x2040:21/23` pair. Retired A6
rotation-pair and single-turn anchors must not restore into the raw source's
different contract. Its fixed command-frame offset requires explicit stationary
Home or matching saved evidence, never startup refitting.
See the J5 rotation-wrap abort observed on a bench (2026-09-22)
and pending replacement-source qualification. The suffix is the
source contract version; a changed name or version invalidates stored evidence
with `anchor_identity_changed_field=anchor.source`. It is not a configurable
override. The source-version, tracking, gearing and HM35 changes invalidate all
pre-migration A6 coordinate digests; configuration, machine and deployment
hashes also change with their migrated inputs. `home.policy` includes the native
method, explicit Home offset object/type/value, truth object and truth sign.
Drive revision in the compiled table
is the configured SII revision; the record additionally compares the observed
CoE revision and serial when valid, including serial validity transitions.

TxPDO presets/layouts, speed caps (including 0x607F), limits, jog/following-error
guards, diagnostics, external enable, restore flags and bus timing do not enter
this digest. Anchor tolerance remains an independent restoration check.
The adapter stores its own `PairID` and `PairRevision` alongside the coordinate
digest and drive identity in v3 anchor records. A different adapter pair or revision
refuses restoration with `anchor_identity_mismatch` before native submission.
Native compares only coordinate and drive/count evidence; coordinate mismatches
publish `IdentityChanged` and `anchor_identity_changed_field` and log the field.
Readers recognize v1 configuration-bound and v2 pair-bearing-digest files and
refuse both as identity mismatches. Perform Home to capture v3 evidence; existing
files are never silently upgraded.

Batteryless `rosie-anchor-v4` single-turn records and rotation-mode v3 records
are retired inputs. They refuse with the changed source/tracking/Home identity;
no reader guesses an offset, converts a frame or upgrades them. Perform HM35
after the mode-1 power cycle to capture a new v3 exact-count anchor.

Home calibration is separate, with its own epoch and anchor records. Definitions
and cells contain neither calibration offsets nor saved encoder anchors. An
explicit Home publishes new calibration evidence; subsequent anchor restoration
must satisfy the existing identity, epoch and coordinate checks.

| Change | Update path and acceptance |
|---|---|
| Factory geometry/joints/limits | Update the URDF pin, model revision and cell binding; recompile and qualify the changed model before use. |
| Definition mechanics or policies | Update values and provenance, model revision and cell binding; recompile and verify drive readback. |
| Cell mapping/limits/deployment | Recompile and verify readiness for the selected deployment. |
| Home calibration | Perform explicit Home and persist its anchor record; calibration is never stored in definitions or cells. |

Legacy flat configurations keep their digest with the new compiler. Migrating a
machine config preserves native payloads and machine identity but changes configuration and
deployment identity. Existing v1 anchors require one Home to establish v3
coordinate evidence; subsequent non-coordinate changes preserve it. Recompiling the same inputs remains deterministic;
changing a publication directory does not change any of these identities.

The [copied Rosie mechanical sources](robots/rosie_1400/README.md) map J1..J3
to HSS-40-100-I-D19 harmonic gearboxes (100:1) and J4 to HRP130_D14 with the
owner-supplied purchased HRP130KK-G18 (18:1), consistent with the sheet speeds.
J5 is settled per model by owner confirmation on 2026-09-15: Rosie 1420 v1
uses 100/1; the deployed v4 pair record (`rosie_1420_v1`), drive readback
and the 2026-08-03 team-chat report of pair revision 3 corroborate that model. Rosie 1400 v3
keeps 27:2 from GradientOS gradient05 commit 2945e3ff (10:1 then 27T:20T),
the task-cited DEVLOG 2026-05-01 and the sheet's 1,333 deg/s rated-speed row.
Its CAD L4 belt stage has an A6M60-400 motor, 60-tooth ring gear and belt
tensioner; physical pulley teeth on the current 1400 build remain unverified
until confirmed by the owner or a mechanical document. The 1420 J5 mechanism
differs from that belt stage. The 1400 cell now derives startup 27/2 and a
new machine identity; deployment requires a fresh Home. J6 is
HRP85_D14; its retained 10:1 is supported by the pair record and sheet, but the
HRP85 variant's part-number ratio is unconfirmed. The 1420 export has no
component list; transfer of the 1400 J1..J4 and J6 mechanisms and arm motor
variants is unverified. The sheet's
17-bit motor-side absolute count model and A6M M17 motor suffix support 131072
counts/rev. Robot signs and encoder counts still require independent measurement.
The bare bench uses motors left to right as axes 0..8: its recorded
2026-09-14 axis-0 observation is sign -1, with a positive logical move turning the
shaft arrow clockwise on camera cam1. That observation does not verify the signs
of axes 1..8 or any encoder count.

Packages carry every robot description under `robot_description/robots/`,
exactly the files each manifest registers. `rtctl compile` reads a machine's
`robot.robot_description` from the component when it carries `robot_description/`
and from the repository beside the component otherwise, and recomputes the
manifest identity over the files either way; a pin that does not match is
`robot_description_mismatch`, a missing or unregistered file is
`robot_description_unavailable`. `rt-package manifest` records each packaged
description in `robot_descriptions` with its model id, path and `robot_description_sha256`;
the file inventory also hashes every file. `tools/verify-package.sh` recomputes
each identity against the manifest without repository fallback.
Relocating the complete package therefore preserves compilation and all three
digests.

## Cutover transaction and rollback

Before every cutover step, run `build/rtctl hostcheck --config MACHINE.json --expected-release RELEASE-IDENTITY.json --json` on the target host with permission to read systemd properties and every process's descriptors and loaded image. Supply the independently retained `rosie-rt-core.host-identity.v1` manifest (shape: `tools/rtctl/testdata/hostcheck/identity/release.json`): for each of `rosie-rt-core`, `rosie-rt-control` and `rosie-rt-natspublisher`, record the template SHA-256, ordered installed unit/drop-in paths and SHA-256 values, fully expanded argv, executable SHA-256 from the release package's `rt_package.json`, environment-file paths and relevant nonsecret environment bindings. The check reads systemd's effective command, MainPID/start identity, process argv/environment, `/proc/<pid>/exe` bytes and EtherCAT device holders without changing lifecycle state; arguments and binding values are reported as digests to avoid disclosing credentials. A conflict means the selected release does not exclusively own the observed deployment: keep outputs inhibited, resolve the competing supervisor or mismatched deployment through the cutover procedure, and rerun the check. Unknown evidence, a stopped service, an unreadable process or a pending daemon reload also prevents qualification; proceed only with zero conflicts, zero unknowns and three qualified matches for the intended running deployment, alongside the existing physical and timing gates. Exit codes: 0 when every layer of every service matches the expected release identity, 2 when any layer conflicts (a drop-in, argument, environment binding or loaded image differs, or a foreign process holds the EtherCAT master), 3 when any layer is unknown (unreadable image, absent MainPID, incomplete process or task visibility, a process or executable that changed during the check). Only 0 qualifies a cutover step; 2 and 3 both stop it.

Prepare this transaction and validate it in simulation before physical cutover:

- Record candidate commit/artifact digest, canonical machine configuration,
  host/master versions, calibrated home/absolute policy, service identities and
  every changed manifest/builder. Retain the known previous deployment artifacts.
- Finish or cancel work, inhibit outputs and verify the required physical stop
  state. Revoke old controller grants; stop the old master/service before the
  candidate can own the NIC/devices. Never run both motion owners together.
- Install/select the candidate and its authenticated application boundary.
  Observe readiness and verified identities with output inhibited. Establish a
  fresh session; home or restore qualified position evidence per axis. Arm only
  after the new readiness conditions are observed. No automatic motion replay.
- On failed acceptance, inhibit/revoke the candidate, establish the required
  physical state, stop its master and restore the recorded deployment/config.
  Re-establish fresh authority and valid home/coordinates before any movement.
  Rolling back binaries does not make old handles, home assumptions or sessions
  valid again.
- Record the result and traces, including restart/reconnect and only-one-master
  evidence. Keep rollback artifacts until machine acceptance is complete.

## Robot-stack release components

`rosie-rt-core` replaces `robot-v4-rtcore` in the `realtime-core` role of
`rosie.robot-stack.release.v1`. Ship one runtime component: the daemon,
`rt-control`, `rtctl`, `rt-package`, IgH userspace library and license notices,
host scripts/units and the `config/` tree. `rt-natspublisher` is an
optional process in this same component when its binary was built. It is not an
independently selected release component in this proposal. The daemon and adapter
must share one compiled configuration digest and the adapter/client binding
`--configuration-sha256 --pair-id --pair-revision`; separate process releases
would admit unrecorded combinations. A separate optional publisher component is
possible later, but requires its own tag and qualification. Keeping its current
unit and binary in this install also preserves the existing package installer.
Any payload change, including publisher inclusion, requires a new recorded
composition and consumer qualification. No compatibility range is inferred.

The single version source is `componentVersion` in
`tools/packageinfo/component.go`, initially `0.1.0`. Packaging and descriptors use
it directly; a second VERSION file would duplicate the release identity. This is
an independent component version, with proposed tag `rt-core-v0.1.0`, not the
daemon's diagnostic `git describe` string or the robot-stack composition version.

The public wire family is `control.sock` HTTP/JSON application v1
(`contract_version=2`, `rosie.rt-control.request.v1` and
`rosie.rt-control.response.v1`). Local `jog.sock` datagrams and WSS `/v1/jog` use
the 224-byte `local_jog_update` record in `protocol/control.json`; private native
IPC is major 1, minor 10. The daemon/adapter have no native NATS command subjects.
The optional observer emits compatibility v4-shape
`robot.v4.rtcore.status.v1` and `robot.v4.rtcore.telemetry.batch.v1` on
`robot/v4/rtcore.<host>.status` and
`robot/v4/rtcore.<host>.telemetry.batch`. This observation compatibility does not
make the old v4 command bridge or cartesiand a qualified rt-control client. The
three retained v4 components in the draft remain unchanged for composition
review; their continued operation with rt-core is not qualified. The old bridge
and the optional publisher must not publish concurrently on the same subjects.

Packaging retains the existing install directory and creates a new sibling
`DESTINATION.components/` containing
`rosie-rt-core-<version>-linux-<arch>.tar.gz`, `rosie-rt-core.json`, `SHA256SUMS`,
`build.log`, `rt_package.json` and `verify-package.log`. The archive is the complete
install tree with `bin/rosie-rt-core` as its primary artifact. The descriptor's
`sha256` hashes that binary; `SHA256SUMS` separately hashes the archive, descriptor
and receipts. Archive entries use install permissions, numeric owner/group zero
and `SOURCE_DATE_EPOCH` (zero when omitted), and omit original gzip filenames.
Repacking identical inputs at the same epoch produces identical checksums;
independent IgH rebuild reproducibility remains unqualified as its manifest says.
Existing output and interrupted attempts are retained, never overwritten.

`runtime_requirements` is derived from every packaged ELF's architecture,
interpreter, NEEDED libraries and symbol versions, including Go executables and
the bundled library. It does not infer arm64 requirements from an amd64 producer.
`toolchain` records `go version` and the compiler banner. `source_files` selects
`rt-core/Makefile`, `rt-core/tools/build-igh-userlib.sh`, `rt-core/go.mod`,
`rt-core/go.sum` and the component packaging recipe at the packaged tree's exact
`git rev-parse HEAD`. `rt_package.json` also records dirty state and daemon source
identity. Supply `ROSIE_RT_PACKAGE_BUILD_LOG` to retain the real producer log;
without it, `build.log` explicitly records that producer invocation is unavailable.
The package inspector's output is captured separately in `verify-package.log`.

The archive is also a retained receipt in the descriptor. This deliberately uses
the existing release tool's binary-plus-receipts contract: `bundle.py` preserves
the full install payload under component provenance without adding a second
artifact field or changing the primary-binary hash semantics. The next
specification additionally pins all four receipts in optional top-level
`receipt_sha256`, keyed by component name and receipt path. `bundle.py package`
checks those pins before creating output; `verify`, `archives` and preparation
reject receipt substitution even if a modified bundle has regenerated its own
manifest and checksums. Specifications without this optional map retain their
existing behavior. The complete runtime archive must be pinned when used as a
receipt carrying executable companion files. For `--inputs`,
copy the component asset directory's files into an empty inputs directory and
extract the archive there. This supplies both `bin/rosie-rt-core` and every listed
receipt. Run `package`, `verify`, then `archives` with the same trusted spec.
The v0.1.0 schema describes the generated bundle envelope and hard-codes that
release, version and arm64 platform; the next specification is instead checked
by `bundle.validate_spec`. A local rt-core-only test specification can prove the
whole path without fetching missing v4 sources or pretending their artifacts
were retained on this producer.

For P8-07 preparation the owner supplies a reviewed local manifest and fixture
whose process name and old slot basename are already `rosie-rt-core`, on the node
`realtime-core`. `prepare.py` requires an exact existing name; it does not rename
`robot-v4-rtcore`, invent adapter intents or translate v4 command arguments. Keep
the true previous cell inputs separately during this explicit name migration.
An example selection is:

```json
{
  "rosie-rt-core": {
    "target_role": "realtime-core",
    "path": "/home/pi/.rosie/releases/robot-stack-next/bin/rosie-rt-core"
  }
}
```

Feed that selection, the complete verified bundle, reviewed local manifest and
fixture, and owner-selected cell/Home reference files to `prepare.py`. Its output
is a local packet and exact-primary-hash CLI inputs, not a staged installation.
The owner must also stage the complete runtime archive from the component's
provenance receipt into `/home/pi/.rosie/releases/robot-stack-next/`, verify its
manifest and every companion file, and supply the adapter process intent and
configuration/pair binding. Publishing only the prepared primary binary is
insufficient. The compiled digest is the separately recorded
`external_references.rt_core_configuration_digest`; it must match the deployed
configuration, daemon argv, adapter and clients. Example configuration is not
cell qualification or an owner-selected Home reference.

Package with
`ROSIE_RT_PACKAGE_SLOT_ROOT=/home/pi/.rosie/releases/robot-stack-next` to render
`ExecStart`, `ConditionFileIsExecutable` and `WorkingDirectory` against that root.
The default remains `/opt/rosie-rt-core/current`, used by `host/install.sh`'s
single-directory release store. Custom slot packages are staged by the release
owner, not through that default-store installer. Roots must be normalized
absolute paths with only safe path characters; whitespace, unit expansions and
traversal are refused. For a root under `/home`, `/root` or `/run/user`, the
publisher's rendered `ProtectHome` becomes `read-only` so the selected binary is
accessible while home trees remain non-writable. Other service identity,
capability, device and `/etc/rosie-rt-core/control.env` requirements still apply.
Choose one supervisor: a daemon/CD intent and a systemd unit must not both own the
same process. Neither packaging nor local preparation contacts a robot, edits a
real cell config, enables services or performs an automatic fleet upgrade.

The local producer for this draft was clean commit
`bdc7a73561eaa25adfd932520278f5a172a30337`, WSL Ubuntu-22.04 on Windows 11,
`go version go1.26.2 linux/amd64` and
`g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0`. No arm64 rt-core output was present.
`readelf -h -l -d -V` and `ldd` were run on all five executables and the library.
The live daemon is ELF64 x86-64, interpreter `/lib64/ld-linux-x86-64.so.2`, with
NEEDED `libethercat.so.1`, `libstdc++.so.6`, `libm.so.6`, `libgcc_s.so.1` and
`libc.so.6`; its maximum required GLIBC tag is `GLIBC_2.34`, GLIBCXX tag
`GLIBCXX_3.4.29` and CXXABI tag `CXXABI_1.3.9`. RUNPATH contains
`$ORIGIN/../lib` and the producer's absolute `build/deps/igh-prefix/lib` path.
The adapter, rtctl and publisher require glibc through `GLIBC_2.34`;
`rt-package` has no dynamic dependencies (`ldd` returned 1 with
"not a dynamic executable"). The bundled IgH library requires glibc through
`GLIBC_2.14`. These are measured amd64 requirements, not arm64 qualification.

The primary binary SHA-256 is
`f5c344cadeb83882ac31a201e96075ad8d750e258dff938e29309c270e9b3cdf`;
the runtime archive SHA-256 is
`31e2093dfdb6c988a5fe6f3507e1656f4e4e449deb29a19a1a75a87d8e04e9b5`.
The emitted descriptor is retained byte-for-byte at
`releases/robot-stack/next/rosie-rt-core.json` and as the rt-core component object in
`releases/robot-stack/next/release.json`; the composition explicitly marks these as
host-produced, unpublished artifacts and the composition as mixed architecture.
The local files remain under `rt-core/build/stackrelease-local.components/`,
including the captured build log, manifest and successful verification receipt.
The release owner must retain/publish the real arm64 producer's corresponding
receipts and tag/assets and record their replacement hashes in a new composition.

The live example failed local compilation with
`acquisition_allowance_exceeds_half_period: axis bench_axis allowance_counts=640016 half_period_counts=65536`.
It was not changed. The packaged nine-axis live bench
configuration compiled successfully to
`ecdbacf30d4c0fc46b8227e432f26baad25aece23812e84b3bb397d6b8fe49d4`.
`rtctl control-env` produced that same digest for fixture pair
`stackrelease-local`, revision 1, dense axes J1 through J9. The draft records this
live-backend digest, with its bench source stated explicitly. It does not qualify
the assembled Rosie's configuration. The packaged simulation example also
compiled to `6ba879e6e43c9c20dcdcfbd13166c21aa6ab3e2efbf76f998fe3c70b1000cb35`;
that was a separate compiler check. Final local preparation uses the live bench
reference and explicitly synthetic peer/Home inputs. The owner must
select and qualify the actual live configuration and record its compiled digest,
pair identity/revision and Home reference before cutover.

Commands used inside WSL, from `rt-core/`, with the prescribed Go environment
and `TMPDIR=/dev/shm/rt-core/stackrelease`:

```sh
make control sim
bash tools/build-igh-userlib.sh
go test -race -count=1 ./tools/packageinfo
bash host/package-test.sh
make control
PKG_CONFIG_PATH="$PWD/build/deps/igh-prefix/lib/pkgconfig" make live
CGO_ENABLED=0 go build -trimpath -o build/rt-package ./tools/packageinfo
for binary in build/rosie-rt-core build/rt-control build/rtctl build/rt-package \
    build/rt-natspublisher build/deps/igh-prefix/lib/libethercat.so.1; do
  readelf -h -l -d -V "$binary"
  ldd "$binary" || printf 'ldd exit=%s; ELF tables above are the dependency measurement\n' "$?"
done
SOURCE_DATE_EPOCH=1 \
  ROSIE_RT_PACKAGE_SLOT_ROOT=/home/pi/.rosie/releases/robot-stack-next \
  ROSIE_RT_PACKAGE_BUILD_LOG=build/stackrelease-build.log \
  bash tools/package-runtime.sh build/stackrelease-local
build/stackrelease-local/bin/rtctl compile \
  --config build/stackrelease-local/config/machines/a6ec-bench.example.json \
  --out build/stackrelease-config
build/stackrelease-local/bin/rtctl compile \
  --config build/stackrelease-local/config/machines/simulation/simulation-program.json \
  --out build/stackrelease-simulation-config
build/stackrelease-local/bin/rtctl compile \
  --config build/stackrelease-local/config/machines/<bench machine>.json \
  --out build/stackrelease-bench-config
build/stackrelease-local/bin/rtctl control-env \
  --config build/stackrelease-local/config/machines/<bench machine>.json \
  --pair-id stackrelease-local --pair-revision 1 \
  --dense-axis-ids J1,J2,J3,J4,J5,J6,J7,J8,J9 \
  --out build/stackrelease-binding --installed-out "$PWD/build/stackrelease-binding"
```

The build and package commands passed; the smaller live-example compilation
refused as above, while simulation and live bench compilation passed.
The first control-env invocation omitted required `--installed-out` and refused
without output; the complete invocation above passed with matching digest.
The focused Go tests passed. The first shell comparison was invalidated by an
in-progress packaging source edit; a subsequent fixed-input run passed identical
MANIFEST/SHA256SUMS, checksum checks, archive extraction and relocated verification,
output collision and unsafe-slot refusals, custom slot rendering, tampering,
confined install and rollback. The shell test uses
`ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim` and `SOURCE_DATE_EPOCH=1`.

From the repository root, the draft was parsed with `json.loads`, validated with
`bundle.validate_spec`, and its other three component objects compared equal to
v0.1.0. The final rt-core-only specification under `rt-core/build/stackrelease-pinned-only.json`
selects exactly the emitted descriptor, `platform=linux-amd64`, no historical
qualification/release files, all four trusted receipt pins, the live bench
configuration reference, and release `robot-stack-rt-core-local-test`.
`stackrelease-inputs` contains the install tree plus the component asset files;
the duplicate MANIFEST bytes were compared before copying. No v4 artifacts were
invented and no source commits were fetched. Commands and outcomes:

```sh
export PYTHONDONTWRITEBYTECODE=1 TMPDIR=/dev/shm/rt-core/stackrelease
# Read-only Windows worktree pointer translation for git show in bundle.py.
export GIT_DIR="$(wslpath -u "$(sed -n 's/^gitdir: //p' .git | tr -d '\r')")"
export GIT_WORK_TREE="$PWD"
python3 -m unittest discover -s tools/robot-stack-release -v
python3 tools/robot-stack-release/bundle.py package \
  --spec rt-core/build/stackrelease-pinned-only.json --repo . \
  --inputs rt-core/build/stackrelease-inputs --bundle rt-core/build/stackrelease-pinned-bundle
python3 tools/robot-stack-release/bundle.py verify \
  --spec rt-core/build/stackrelease-pinned-only.json --bundle rt-core/build/stackrelease-pinned-bundle
python3 tools/robot-stack-release/bundle.py archives \
  --spec rt-core/build/stackrelease-pinned-only.json --bundle rt-core/build/stackrelease-pinned-bundle \
  --output rt-core/build/stackrelease-pinned-assets
python3 tools/robot-stack-release/prepare.py \
  --spec rt-core/build/stackrelease-pinned-only.json --bundle rt-core/build/stackrelease-pinned-bundle \
  --manifest rt-core/build/stackrelease-robot_description_manifest.json \
  --fixture rt-core/build/stackrelease-intents.json \
  --selection rt-core/build/stackrelease-selection.json \
  --cell rt-core/config/machines/<bench machine>.json --home rt-core/build/stackrelease-home.json \
  --output rt-core/build/stackrelease-pinned-prepared
(cd rt-core/build/stackrelease-pinned-assets && sha256sum -c SHA256SUMS)
```

All 14 Python tests passed, including the original 11 and new receipt-pin tests
covering input refusal, malformed pins and receipt substitution with regenerated
manifest/checksums. Package, verify, archives and fixture-only preparation passed
with the optional pins enforced. Both resulting archive checksums passed.
The bundled runtime archive was byte-identical to the package archive, and the
prepared primary executable was byte-identical to the packaged daemon. Preparation
used a synthetic `local-test-fixture` peer and a Home file explicitly containing
no qualified position; no real cell config was read or edited. Full four-component
packaging and cross-consumer/hardware qualification remain release-owner work.

The required `make control && make test && make test-go` run passed control and
test, including all C++ checks and confined packaging/lifecycle checks, but
test-go failed in unchanged IPC and benchdrive tests. Each failure was rerun
alone using the same WSL environment:

```sh
go test -race -count=1 -v ./ipcclient -run TestTelemetryLiveContinuityAndNoCycleFaults
go test -race -count=1 -v ./tools/benchdrive -run TestSimulationProgramTravelAndVelocity
go test -race -count=1 -v ./tools/benchdrive -run TestSimulationProgramStartsAtHeldTarget
```

The telemetry rerun passed with zero cycle page faults; the full run had one
additional minor fault (30 to 31), so it is recorded as load-sensitive. Both
benchdrive reruns failed again with `native_segment_rate_exceeded`; their code
and the native execution code are unchanged from `6c87be0f`. Those failures are
not classified as load-sensitive and the full Go gate remains unmet. The scoped
packageinfo tests passed in the full Go run. No unrelated test or runtime limits
were changed to obtain a green result.
`make test-sanitize` passed its ASan/UBSan oracle run with the prescribed WSL
environment. The final commit gate remains blocked by the two reproducible
benchdrive failures above.

## Scope and outstanding external inputs

The folder owns the realtime application, EtherCAT integration, pinned dependency
recipes, host tooling and clients. PREEMPT_RT and the kernel-matched IgH module
remain installed host dependencies. They are not copied into a new kernel.

Required-home axes currently re-home after restart. P4 adds generalized continuous
coordinates and qualified absolute restoration where supported; otherwise retain
an explicit machine-approved re-home policy. Never guess absolute validity or
reuse an anchor after an incompatible drive/config/home change.

The actual Linux host/NIC, drive models/ESIs, mechanical scaling, home method,
brakes/stop chain and conservative test limits are needed for physical gates.
Yaskawa model-specific work is deferred; general profile separation is required
now. Remote authority, actual motion-server integration, cell I/O for welding and
deployment recovery are required for full cutover, not optional future work.
