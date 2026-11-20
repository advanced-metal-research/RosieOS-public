# Bench qualification

Configuration files are organized by layer under [config/](../config/README.md): drive, robot, machine and cell configs.

## Running a subset of a connected bench

A bench cell may select `bus.unknown_slaves: "hold"` with an explicit
`unknown_slaves_note` explaining the connected subset. For example,
`config/machines/bench/rosie1420-six-motors.json` uses positions 0..5 as J1..J6 while the
remaining three motors stay connected. Because it uses a robot definition, it
also carries `hold_on_robot_acknowledged: true` and `hold_on_robot_note`. Only
`bench_nine_motors_1to1` and `bench_one_motor_1to1` bypass that robot acknowledgement.

Held devices receive no slave configuration, PDO registration, startup SDOs or
outputs and remain in PREOP. The DC reference clock is pinned to a configured
axis. Inspect `rtctl inventory --config CELL.json --json` and status/describe's
`bus.held_slaves` for positions, SII identities and presence. A held device in
SAFEOP or OP refuses readiness until it returns to PREOP or INIT.

The default `refuse` policy is appropriate for robot cells. Declare connected
unused devices in `bus.unused_slaves` with position, vendor, product, optional SII
revision and a provenance note. Unknown, missing declared or mismatched unused
devices refuse bring-up. Under `hold`, post-admission disappearance or identity
changes at unused positions log `policy=log_and_continue_axes`; configured axes
still require their own uninterrupted full-group exchange. Under `refuse`, a
post-admission topology refusal latches a readiness fault and inhibits outputs
before the next frame is submitted. After restoring the declared device and
readiness, use public `reset_fault` (FaultReset), reacquire the application
session, then explicitly Enable and Arm. Enable before reset is rejected with
native `not_ready` (reason 2). Reset clears the readiness fault (reason 128) and
safety mask while keeping outputs inhibited; it does not restore lost Home
evidence. Restore qualified Home evidence separately where required for motion.
Restoration alone never re-arms.

| Recovery step under `refuse` | Required observation |
|---|---|
| Restore device and verified readiness | Readiness fault remains latched; enable mask and armed state remain zero. |
| Enable before reset | Typed native rejection `not_ready` (2); outputs remain inhibited. |
| Public `reset_fault` (FaultReset) | Readiness fault and safety mask clear; outputs remain inhibited and the application session retires. |
| Reacquire application session, then Enable | Configured axes become enabled; armed state remains zero. |
| Arm | Armed state becomes one after readiness checks; no motion is replayed. |

Held devices never add to WKC.
Six simulated axes report 12/12; live IgH LRW reports 18/18, reflecting its actual
output/input counter increments. Neither number counts the unused motors.

The bus block changes configuration and effective machine identity; changing only
its notes changes configuration identity. Preserve fresh identity-bound Home
evidence when changing the connected subset. See [migration](MIGRATION.md#cell-bus-topology)
for schema and digest rules and the six-motor setup.

## First cell I/O terminal qualification

For the first cell I/O terminal inventory (P6-05), keep the hardware safety chain
active and physical outputs disconnected/inhibited. With the daemon stopped, use
`build/rtctl inventory --config <machine.json> --json` (drive configs default to `config/drives`) to read the
configured terminal's slave position and SII vendor/product/revision without
writes. Record the actual terminal models, fixed PDO/SM maps, input polarity,
independent readback wiring and watchdog OFF behavior before replacing the generic
`config/drives/dio_terminal.json` placeholders. The current model is one combined
eight-DI/eight-DO slave; separate terminals need separate inventoried mappings and
multi-terminal support. No terminal exists on either bench yet. Keep all
`_unverified` bounds and `torch_qualified: false` until the physical Stop, expiry,
readback and loss checks in [CELL_IO.md](CELL_IO.md) have passed.

Before enabling any output class for cell operation, record matching SII
identity and complete PDO assignment/map readback; one input toggled by hand and
observed with correct polarity within one native cycle; one isolated output ON
with independent wired readback; expiry without another request to the OFF safe
state; and a Stop fence that commands OFF, confirms safe readback, rejects the old
grant and requires a fresh grant plus explicit I/O arm. Use isolated test loads
for those qualification operations. The detailed procedure is in
[First terminal hardware qualification](CELL_IO.md#first-terminal-hardware-qualification).
Torch-class markers remain refused until this qualification is recorded and
reviewed; the current `torch_qualified: false` contract remains enforced.

## Mini bench without an encoder battery

Owner statement 2026-09-15: the mini bench AS715N encoder cable has no battery
box. `robot_description/robots/bench_one_motor_1to1/rtcore_definition.json` declares `encoder_battery: "absent"`;
the 2026-10-20 qualification still uses C00.07=1 absolute linear mode and exact
continuous counts while the drive remains powered. Battery absence is part of
the coordinate identity: retained Home across a drive power loss is unavailable,
and no test may claim it until a battery box is fitted.

The earlier mode change first raised ALF1.0 (`603F=6320`, `203F=0F10`,
re-power required), then Er20.8 (`603F=7305`, `203F=0208`) after power cycle.
Neither is tolerated. With physical motion inhibited and the daemon stopped,
`build/rtctl recover-encoder --slave 0 --backend ethercat` performs the bounded
F31.10/F31.01/F31.00 sequence and verifies raw alarms after each step. Follow the
[A6-EC recovery procedure](drives/a6ec/README.md#encoder-battery-configuration-and-recovery),
including preventing automatic daemon restart. No reset is issued automatically
by bring-up. For a robot joint explicitly declared battery-absent, only the
profile's measured, currently admitted ALF9.0 pair (`7305/0F90`) is a warning:
it disables every power-off retention claim but does not invalidate exact live
counts. Er20.8, Er20.9, ErA0.1, unknown pairs and stale receipts remain faults.

Compile the corrected mini cell and establish a fresh drive Home and software
Home anchor before Enable. A mode change or encoder-data reset invalidates prior
Home evidence: do not restore the previous anchor. Mode 1 presents motor-frame
6064 continuously; software applies the bench's 100/1 ratio (owner direction
2026-11-03, was 1/1: the shaft has no reduction, so one output radian is 100 motor
radians and joint limits read as an arm-like output joint). U40.20/.22 remains the
independent exact-count source (`raw_multi_turn_encoder|3`).

For the powered mini-bench qualification, record U40.20/.22 and `6064` with the
verified C00.07=1, 0x6091=1/1, `607C=0`, configuration/Home epochs and
acquisition validity. Prove monotonic exact-count motion across multiple turns
in both directions and one-shot reversal. Do not power-cycle as a retention
test: without a battery, power loss is expected to invalidate turn retention
and requires operator HM35 after the drive returns.

## Collision threshold tuning (P4-17)

Use `benchdrive --phase move --tune-collision --collision-margin 1.5` or
`benchdrive --phase program --tune-collision --collision-margin 1.5`, together
with the phase's existing binding, evidence, motion and confirmation flags.
Run representative trajectories and loads, read the per-axis
`collision_watchdog_thresholds` in the JSONL summary and the printed table, then
set thresholds above the recorded peaks with margin. No tuning command changes
configuration or arms motion beyond the normal phase contract. On a machine with
`tracking_lag_allowance` the native following-error bound also grows by
|commanded velocity| x the allowance, so a peak recorded at speed includes the
drive's lag that the allowance already admits, and a static suggestion computed
from it is conservative.

The daemon records absolute peak torque (0x6077, raw per-mille of rated) and peak
following error every fresh armed/enabled cycle, even when watchdog decisions are
disabled. Peaks include enabled hold since arm, so status publication cannot hide
a short peak; they may conservatively include motion earlier in the same arm.
Use a fresh arm for each tuning run. The 0x1B04 preset supplies drive 0x60F4;
otherwise following error uses the prior wire target minus feedback with the
configured count-frame/wrap rules. `--tune-collision` reads V2 capture cycles and
retains signed torque, validity, watchdog counters/peaks and group fault masks
on each timestamp. Missing torque validity or lost cycles after the initial
cursor refuse tuning. `--measure` also records capture torque/validity and reports
`peak_torque_raw` alongside its tracking measurements. Suggestions are `ceil(peak * margin)`, minimum 1;
margin must be finite in (1,10]. A result above 1000 torque raw or 2147483647
counts is marked `within_compiler_ranges=false`, never silently clamped.
Choose and qualify `sustained_cycles` separately; the helper does not infer
collision persistence from normal-motion peaks. Preserve `_unverified` until
remeasured on the loaded robot and keep the measurement path in `_source`.

Finite, opt-in starting values are declared in
`tools/rtctl/testdata/collision-watchdog-unverified.json`. Copy the matching block
into each cell axis (including robot-definition cells), or a drive profile.
They are deliberately not inserted for an absent block:

| Axis | torque_abs_max_raw | following_error_counts_max | sustained_cycles | Provenance |
|---|---:|---:|---:|---|
| J1 | 1000 | 8000 | 10 | `_unverified`; owner direction / GradientOS count placeholder |
| J2 | 1000 | 8000 | 10 | `_unverified`; owner direction / GradientOS count placeholder |
| J3 | 1000 | 7000 | 10 | `_unverified`; owner direction / GradientOS count placeholder |
| J4 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |
| J5 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |
| J6 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |
| J7 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |
| J8 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |
| J9 | 1000 | 8000 | 10 | `_unverified`; conservative count placeholder |

Torque uses the finite compiler ceiling, not GradientOS's out-of-range
1800..2500 placeholders or a measured robot safety limit. The 2026-09-15 archived
program/move/halt JSONL replays without a following-error trip (largest recorded
command/feedback discrepancy 103 counts). That historical sparse archive cannot
establish continuous torque behaviour. New V2 ring dumps contain torque and its
validity every cycle; dump replay reproduces the simulator trip cycle and quantity
using captured samples and the configured thresholds. Synthetic signed torque
replay also covers the native evaluator. The old sparse observations remain
insufficient to qualify a continuous 10-cycle window. Bare-motor catch-up
overshoot and later robot loads require remeasurement; these values do not
establish collision detection sensitivity or physical stopping performance.

With a qualified v3 anchor and matching adapter pair ID/revision, switching the surveyed 0x1B02/0x1B04 TxPDO presets no longer requires Home; explicit inhibited `restore_anchor` still checks the drive and count evidence. Every compiled axis publishes its coordinate identity without pair metadata in the cell configuration. The adapter binds its `--pair-id` and `--pair-revision` separately in the anchor store; v1/v2 records require one Home before restoration under this format.

## Local mini bench

The fresh A6-EC on the 2026-09-15 mini bench raised ALF1.0 after the first
absolute-mode write, C00.07 = 4 (0x2000:08). It requires a physical power cycle
for the parameter settings to take effect; no software reset object exists.
Status now names this condition as `drive_alarm: "ALF1.0"` with
`drive_alarm_text: "Re-power-on required for parameter settings to take effect"`
after an admitted 0x203F read resolves 0x603F = 0x6320 and external code 0x0F10.

The owner survey of 2026-09-15 identifies the single-axis mini bench, Raspberry Pi 5 Model B 8 GB,
Raspberry Pi OS Debian 13 arm64, kernel `6.18.34+rpt-rpi-v8-rt` with PREEMPT_RT,
four cores and no isolation. `eth0` is reserved for one A6-EC drive; Wi-Fi
`172.16.11.178` and USB `192.168.55.100` remain management links. The Jetson at
`192.168.55.1` is not an RT host. These scripts run only when the orchestrator
invokes them; implementing and testing them does not provision either host.

Prerequisite packages: `util-linux` (runuser and flock), `jq` (1.6 or newer),
`curl`, `psmisc`, `systemd` and `pkg-config`; install these before invoking the
scripts. Startup checks each required tool by name before host admission and
includes `/usr/sbin` in PATH. Also required: owner-enabled `sudo -n` for the bench login, the existing matching RT kernel
headers, `/home/operator/.rosie/bin/go` (Go 1.26.2), native g++/make and a physically
inhibited bare motor. The [host profile](MIGRATION.md#ethercat-host-profiles) pins
IgH 1.6.12 with `--enable-generic --enable-hrtimer` and without
`--enable-cycles` (arm64 `cpu_khz` failure). It defaults to
`DEVICE_MODULES=generic` at the 1,000,000 ns cycle. `provision.sh` installs
the foundation, obtains eth0's MAC, enables and checks `ethercat.service`, and logs
the master, slave count, verbose SII identity and source/module hashes.
`provision.sh` is the only mini bench script that writes outside `/home/operator/rt-core-bench`, `/dev/shm/rt-core` and the transient units.
It reads
the Wi-Fi identity but never changes `wlan0` or `usb0`; all scripts refuse an
EtherCAT-interface override other than eth0. Before provisioning or startup, the
loaded `ethercat.service` properties (including drop-ins) must resolve to one
readable `ethercat.conf` or `sysconfig/ethercat` beneath `/etc` or `/usr/local/etc`.
The inspector checks merged `Environment`, every `EnvironmentFiles` entry and
the configuration selected by `ethercatctl` or `init.d/ethercat`, including
literal sysconfdir substitutions and delegated `ethercatctl -c` arguments.
Conflicting paths, pending unit reloads, unknown loaders or variables, and missing
optional files fail closed. Only bare or double-quoted literal assignments without
surrounding spaces are accepted; comments and blank lines are ignored. Every input
must be safe even when a later source overrides it. Literal `MASTER0_DEVICE` must
match eth0's MAC; only generic/macb with one
master and no backup NIC are accepted. The attached `ethercat master` MAC must
also match eth0 with link UP before startup. Missing eth0 MAC evidence is refused,
including a native macb setup that has removed the netdev; these commands use generic.

From the reviewed checkout's `rt-core/` directory, the orchestrator runs these
commands in a Linux shell. The archive contains the component at its root; its
SHA-256 is the deployment `<sha>`, independently of the compiled configuration
digest. The bootstrap copy is the reviewed source used to invoke provisioning.

```sh
mkdir -p build
bash tools/packageinfo/git.sh -C .. archive --format=tar.gz --output=rt-core/build/minibench-tree.tgz HEAD -- rt-core robot_description
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> 'mkdir -p /home/operator/rt-core-bench/incoming /home/operator/rt-core-bench/bootstrap'
scp -i ~/.ssh/rosieos_pi build/minibench-tree.tgz operator@<bench-host>:/home/operator/rt-core-bench/incoming/tree.tgz
sha=$(sha256sum build/minibench-tree.tgz | cut -d ' ' -f 1)
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> 'bash -s -- /home/operator/rt-core-bench/incoming/tree.tgz' < host/minibench/bootstrap.sh
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> "sudo -n env NO_REAL_MOVEMENT=1 bash /home/operator/rt-core-bench/bootstrap/$sha/rt-core/host/minibench/provision.sh"
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> "bash /home/operator/rt-core-bench/bootstrap/$sha/rt-core/host/minibench/deploy.sh /home/operator/rt-core-bench/incoming/tree.tgz"
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> "sudo -n env NO_REAL_MOVEMENT=1 bash /home/operator/rt-core-bench/bootstrap/$sha/rt-core/host/minibench/apply-power-cycle-startup.sh /home/operator/rt-core-bench/$sha/rt-core/build/minibench/config/<configuration-sha256>/axes.conf /home/operator/rt-core-bench/$sha/rt-core/build/minibench/config/<configuration-sha256>/argv.json"
# When the command prints drive_power_cycle_required=1, remove drive power,
# restore it, then run up.sh. A software reset cannot activate C00.07.
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> "sudo -n env NO_REAL_MOVEMENT=1 bash /home/operator/rt-core-bench/bootstrap/$sha/rt-core/host/minibench/up.sh $sha"
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> 'curl --fail --unix-socket /dev/shm/rt-core/live/control.sock http://localhost/v1/status'
# Bare unloaded motor only: after the operator confirms its current shaft pose
# is the desired temporary Home, the script proves inhibited/stationary status,
# performs HM35 and leaves the controller disarmed.
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> "env ROSIE_MINIBENCH_OPERATOR_HOME=1 bash /home/operator/rt-core-bench/bootstrap/$sha/rt-core/host/minibench/home-current.sh /home/operator/rt-core-bench/$sha/rt-core/build/rtctl /dev/shm/rt-core/live/control.sock <configuration-sha256>"
```

The reviewed local `bootstrap.sh` is sent over stdin so archive inspection runs
before any archived script. It rejects traversal, links, special files and
embedded build/git paths, then extracts into a fresh staging directory and
publishes `/home/operator/rt-core-bench/bootstrap/<sha>`. It never overwrites an
existing tree: reuse requires matching source file hashes and directory entries
against a fresh extraction of the archive. A differing `Makefile`, extra source
file or forged identity marker is refused. Only the reserved root `build/`
directory is excluded from source comparison so provisioning logs can remain.

`deploy.sh` uses the same extractor to verify `bootstrap/<sha>` and publish or
verify `/home/operator/rt-core-bench/<sha>`, then builds `control live sim` natively
from its `rt-core/` subtree with the matching userlib, compiles
`config/machines/bench/one-motor-1to1-lcdjog.json` and prints
both digests. Repeated completed deployments verify source and retained artifact hashes;
an interrupted build resumes in its existing release. Build caches and logs stay
under the release's `build/`. Provisioning's log and raw SII capture are under
the bootstrap's `build/minibench/`.

`up.sh` runs the daemon as root and the adapter as the bench login in transient
`rosie-minibench-core.service` and `rosie-minibench-control.service` units with
automatic restart disabled. Both inherit `TMPDIR=/dev/shm/rt-core` and
`ROSIE_RT_ANCHOR_DIR=/home/operator/rt-core-bench/anchors`; sockets are under
`/dev/shm/rt-core/live`. It acquires no application grant and issues no Home,
Enable or Arm. Success requires the selected configuration digest, one verified
axis, zero enable/arm/lease and fault fields in public status. Failed startup
stops the units it started. Repeating `up.sh` for the same running release polls
again; another release or a partial pair is refused. If observation exhausts its
budget, the final error includes the last failure once: unavailable runuser,
curl connection/request failure, invalid JSON, or the names of the false status
clauses (for example `config_verified_mask` or `armed`).
Before starting an inactive pair, `up.sh` removes predecessor `description.json`,
`metrics.json` and telemetry dump indexes from its live directory after checking
socket ownership. Locks, sockets and binary dumps keep their existing owners.
The adapter retries predecessor metrics within its bootstrap budget; an
incarnation mismatch on an established connection remains fatal.

The assumed SII vendor/product/revision must match `a6ec_bench_1to1` before the
daemon can issue startup SDOs. A mismatch requires identifying the drive and
reviewing its profile, not bypassing the check. The one-motor derivation
retains 0x1B04, drive gearing 0x6091 = 1/1 with the 100/1 software ratio (since
2026-11-03), no brake, single-turn absolute evidence and the
unverified 2 rev/s startup bound. Physical sign +1, encoder resolution, mechanics,
host CPU/IRQ placement and cycle latency remain unqualified. The single-axis
adapter supports ordinary axis operations; the dense nine-axis plan format does
not apply. Inhibited startup success is not permission for powered motion.
For fixed presets, the daemon owns both PREOP assignment objects: RxPDO 0x1C12
and TxPDO 0x1C13. A factory 0x1701 RxPDO becomes the profile's 0x1702 through
disable/select/count writes, followed by assignment, count and entry readback.
Any mismatch blocks activation. A differing scan cache triggers automatic rescan
and fresh readback. The existing restoration option restores both original
assignments after deactivation; leave it off for a preset hand-off.

Inspect or stop explicitly before replacing a running release:

```sh
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> 'sudo -n journalctl -u rosie-minibench-core.service -u rosie-minibench-control.service --no-pager'
ssh -i ~/.ssh/rosieos_pi operator@<bench-host> 'sudo -n systemctl stop rosie-minibench-control.service rosie-minibench-core.service'
```

Stale/owned sockets are refused, never silently removed. Inspect process owners
and service logs before removing an orphaned socket. The foundation's macb option
is available for separate driver qualification; these local commands use generic.

**Status at `403063dc`.** P1-07 supplies Go rtctl commissioning/inventory tooling; P3-08 supplies local/WSS jog; P4-04/P5-06 supply public recovery; P4-05 supplies public `halt` and the shared ramp (verified in software; powered stopping measurement is the procedure above); P7-04 supplies host checks; P4-10 brake sequencing remains a deferred backlog card. The Python oracle and `tools/commission.py` are removed (P1-10). The sections below this line remain the original qualification procedure, so their claims that Go tooling and these interfaces are unavailable describe that older baseline. P8-04/P8-05 bench acceptance is recorded under `docs/evidence/`; use [APPLICATION_API.md](APPLICATION_API.md) for current commands.

Revised 2026-09-12 against reviewed code `895e249`. Follow the gates in
[IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md). The reviewed build has open
software blockers; it is not a powered-motion candidate merely because older
simulation suites passed. This document is a qualification procedure, not a
record that any drive has been contacted or moved.

## Before powering motion

Close G0 and the G1 host/inhibited checks:
readiness-before-clock, checked bus/Stop results, separate jog/bulk ingress,
explicit handle cancellation, reconciled re-arm, recovery and native authority.
Resolve relevant R04/R07 claims with targeted tests, not assumptions. Preserve
all prior Stop/continuity/limits/deadline regressions. Complete service access,
configuration verification and stale-socket/restart checks.

Use Go `rtctl` commissioning/configuration commands once P1 implements them;
they are target tooling, not available commands at this baseline. The present
`tools/commission.py` and Python configuration launcher are transitional and must
be replaced before final G2. Preserve and use this existing test/tooling oracle
through P2/P3; language removal is not a prerequisite for G0/G1. M1's localized
fixes alone do not qualify powered motion: the jog lane and host checks remain.
Never remove existing checks without demonstrated replacement parity.

Protocol 1.5 restricts the retained direct Python/native oracle to simulation or
IPC-only builds. It cannot obtain the live adapter role. Normal access uses the
Go adapter and its native-enforced application grant; the remaining Go
commissioning/recovery tooling must land before the corresponding bench gate.
REV-01 also reopens wrapped-feedback re-arm (P2-05). Passing the continuous-
feedback regression does not qualify an A6 J6-style wrapped-feedback axis.

For each test record commit/artifact and canonical configuration digests,
host/kernel/master versions, topology/drive firmware, cycle period, configured
limits, operator action, command/feedback trace, expected outcome and pass/fail.
Record failures and recovery; do not count command acceptance as physical success.

## Qualification sequence

For an A6 preset hand-off, inhibit motion with the external stop/STO chain, keep
the application disarmed and stop the prior master owner. Clear stuck slaves by
requesting INIT then PREOP with the EtherCAT CLI; verify their observed states
before launch. Set `ethercat.restore_assignment_on_exit=false` and launch with the
selected preset. The daemon owns the PREOP assignment write and physical mapping
verification. When the master's cached layout differs, automatic rescan releases
the inactive master, queues the rescan ioctl, waits for scan completion and matching
layouts, then reacquires, rebuilds and physically re-verifies before activation.
The scan uses the configured `preop_to_safeop_timeout_ms` bound and one hand-off
per launch; failure retains `rescan_required=1 activation=blocked`.

For nine axes changing to 0x1B04, expect `assignment=written phase=PREOP`, per-axis
`phase=PREOP result=verified`, `phase=pdo_rescan state=released`, `state=queued`,
`state=observed slaves=9/9 al_states=0x2 scan_busy=0 mapping=verified`, and
`state=reacquired master=0 rebuild=required readback=PREOP_required`, followed by
fresh PREOP verification and `IgH libecrt active (num_axes=9, expected_wkc=18)`.
All nine slaves must then be OP: `EtherCAT startup converged` must report
`operational=9/9` and `domain_wc=2 wkc=18/18`. Require `startup_ready` continuously
for **500,000,000 ns** before any userspace SDO after activation. Startup readback
then logs `delay_ms=500` and per-axis `verified=1`; mapping readback logs
`result=verified`. IgH's own startup configuration remains the mailbox owner until
OP. The same runtime admission applies to diagnostics, rotation sampling and native
Home; a readiness or configuration-epoch change discards an in-flight reply and
restarts the settle interval.

Keep restoration off during the preset hand-off and test it separately with
motion inhibited. Its opt-in flag restores the assignment saved at startup after
deactivation and observed PREOP. Require `restore=verified`; record `restore=failed`
as failure. A stale cache with restoration enabled blocks the hand-off. See the
[A6 procedure and exact log lines](drives/a6ec/README.md#fixed-pdo-assignment-ownership)
for the full sequence. These are acceptance observations, not hardware results.

Use `build/benchdrive` for the inhibited, Home, Enable and bounded jog phases with lease renewal and JSONL observations; energising phases require `--confirm-motion`.

1. **Read-only inventory.** Identify the Linux EtherCAT host/dedicated NIC and
   every drive model/ESI, firmware, position and serial/identity where supported.
   `rtctl inventory` uploads through the CLI outside the daemon's mailbox admission and runs only while no daemon owns the master.
   Record actual PDO/DC/startup configuration, drive-side gearing, encoder/frame
   units, home/absolute method, mechanical travel and brake/stop-chain behavior.
   Resolve mismatches before any startup SDO write. Starting the normal bus can
   apply configured startup writes; it is not inherently read-only inspection.
2. **Inhibited bus and host checks.** Run strict config validation, then one
   master with motion inhibited. Verify SDO readbacks, identity/revision,
   observed mode, slave/WKC health, backend and verified config identity. Measure
   locked/prefaulted memory, CPU/FIFO/IRQ/NIC setup, cycle timing and watchdogs.
   Establish conservative machine-specific limits and the external stop/STO
   chain. Software disarm does not replace that chain.
3. **Stationary control and failure checks.** Check session revoke, Stop,
   disconnect, lease/input expiry and restart with one owner. Observe inhibition
   and readbacks; fresh Arm must not revive an old plan/jog. Validate failed-send
   reporting and delayed drive readiness through the proven injection fixture
   before choosing controlled hardware fault tests. Do not advance a plan clock
   while its group is not Operation Enabled.
4. **One-axis home and coordinates.** Use a mechanically appropriate axis.
   Verify direction, units, gearing, offset, seam/wrap/continuous-turn behavior
   and absolute validity. Test disabled displacement only with a mechanically
   suitable setup. Reset/re-arm must neither snap to an old wire target nor get
   stuck in a stale-target fault loop. Verify re-home or qualified absolute
   restore after restart and invalidation after incompatible changes.
5. **Bounded jog and stopping.** With conservative motion limits and clear
   mechanism, verify velocity/ramp, reversal, end/expiry behavior, target lead,
   following error, Stop and external stopping/brakes. Record physical travel
   and stopping distance/time. Run maximum-size preparation concurrently and
   measure source-to-RT age. Expired jog input must never start after upload.
6. **Small prepared trajectory.** Verify inert Prepare, explicit Start, group
   readiness, first-point continuity, actual q/qd behavior, clock/endpoint settling
   and completion. Verify rejected invalid plans, Discard/Abort, cancellation,
   loss of readiness and no replay. A mapped velocity PDO alone does not prove
   a drive consumes CSP feedforward; qualify the actual profile behavior.
7. **All axes and load.** Repeat applicable checks per profile/axis and then
   coordinated motion. Stress repeated 250,000-point upload/execute/discard,
   simultaneous observation, jog during preparation and the agreed soak load.
   Measure memory, deadlines, event loss and feedback freshness. Confirm no
   accumulated slots and no independent retiming of planned axes.
8. **Actual Rosie and cell I/O.** Use the real motion server, pendant/UI sources,
   authenticated controller/pair owner and deployment services. Test real planner
   boundaries/dwell/markers, revocation and reconnect. Qualify cell I/O timing,
   expiry, Stop-before-ON fencing and physical OFF/readback before any welding.
   Do not bypass the present refusal of unqualified torch-bearing execution.
9. **Cutover and rollback.** Complete G2's preceding qualification prerequisites,
   then perform the recorded single-master transaction in
   [MIGRATION.md](MIGRATION.md), including an inhibited rollback drill, fresh
   authority and validated home/coordinates. Close G2 after deployment selection
   and acceptance pass; otherwise restore the recorded previous deployment.

## Benchdrive

Build with `make control`. Each invocation requires `--socket`, `--pair-id`,
`--pair-revision`, exactly one of `--configuration-sha256` or `--machine-sha256`,
and a new `--evidence` JSONL path. Select `--phase` and the axis indices from
Describe. Home, Enable, jog, move, halt, program, sequence and session require
`--confirm-motion`; Home can energise the drives. Complete Home before starting
a move, program, jog or session. The driver checks Home and coordinates before Enable/Arm.

For `--phase move`, use `--axis 0 --delta-rad 0.02` for one axis, or
`--deltas-rad '0:0.02,8:-0.01'` for simultaneous displacements. `--deltas-rad`
replaces `--axis` and `--delta-rad`; any supplied `--axis-mask` must equal the
selected axes. Deltas are finite, nonzero and bounded by +/-3 rad and the
described soft limits. `--velocity-fraction` and `--accel-fraction` are in
`(0, 0.5]`. Every selected axis shares the slowest axis's profile. Completion
requires observed endpoint settling. `--keep-armed` skips the explicit Stop only
after a successful standalone move; terminal Release still inhibits outputs.
An error after acquisition triggers Stop before Release.

For `--phase program`, `--axis-mask` selects the moving axes from J1..J9 in
Describe order. All nine axes must be homed and are Enabled/Armed because the
dense program includes stationary axes. Each first sample equals its published
`logical_target`; unselected axes hold that target. `--delta-rad` defaults to
0.2 rad and must be finite, nonzero, within +/-3 rad and the described soft
limits. The shared trapezoid uses the corpus's 0.01 s grid, with
`--velocity-fraction` and `--accel-fraction` in `(0, 0.5]` and the move phase's
velocity reserve. The repository Go encoder builds the program in memory;
`PrepareProgram` uploads once with session/generation headers while renewal
continues. `StartProgram` echoes all eight returned identity fields. Completion
requires correlated Done, a terminal handle, every endpoint within its described
completion tolerance, measured travel matching each delta, and velocity bounds
checked from recorded positions. JSONL retains identity/digests, held targets,
feedback start/end positions, travel and execution sequence/generation. Stop and
Release follow completion or error; `--confirm-motion` is required.

For the nine bare motors with drive and software ratios 1/1, compile
`config/machines/bench/nine-motors-1to1.json` with `build/rtctl compile --config
config/machines/bench/nine-motors-1to1.json --out build/bench-program-config`. Its compiled
configuration digest is
`e3bcde7a52187a6c8231202d8dc23442f1399955c7d64a7b9c1f051e29ab93ac`.
With the matching live configuration, pair revision 1, Home complete on all nine
axes and adapter dense mapping `J1,J2,J3,J4,J5,J6,J7,J8,J9`, run:

```sh
build/benchdrive --socket /dev/shm/rt-core/live/control.sock \
  --pair-id bench-nine-motors-1to1 --pair-revision 1 \
  --configuration-sha256 e3bcde7a52187a6c8231202d8dc23442f1399955c7d64a7b9c1f051e29ab93ac \
  --phase program --axis-mask 511 --delta-rad 0.2 \
  --velocity-fraction 0.1 --accel-fraction 0.1 \
  --evidence build/bench-nine-program.jsonl --confirm-motion
```

For `--phase jog`, select one `--axis`. `--direction +1` or `--direction -1`
holds that sign for `--jog-duration-ns`, then sends zero and ends jog;
`--direction 0` retains the out-and-back probe. Duration is positive and bounded
by 2,000,000,000 ns, and `--jog-fraction` is in `(0, 0.5]`.
Directional jog and session require `--motor-revs-per-axis-rev` with the verified,
positive motor-to-axis revolution ratio. Evidence records signed measured travel
in radians and converts it to motor revolutions using that ratio.

`--phase session --axis-mask 511 --motor-revs-per-axis-rev 1 --confirm-motion`
keeps one renewed grant while reading one command per line from stdin. Supply the
binding, socket and evidence flags above as well. The grammar is:

```text
move axis:rad[,axis:rad...] [velocity-fraction] [accel-fraction]
jog axis direction fraction duration_ns
stop
status
home mask
quit
```

Move/jog indices and the decimal Home mask must stay within `--axis-mask`.
Session jog direction is `+1` or `-1`; movement bounds match the standalone
phases. `stop` observes inhibition and the resumed native fence; the next motion
command explicitly Enables and Arms again. `home` Stops, Homes the selected
subset and then Enables/Arms the session mask. `status` prints per-axis readiness,
logical position, Home validity and statusword. EOF or `quit` ends the session
with Stop/Release. Invalid input, external Stop, faults and renewal loss end it
with a failure record; native recovery names or execution fault masks accompany
observed faults. Idle stdin still polls status.

`--phase telemetry --label bench-camera-mark` expects decoded native records and
a fresh `telemetry_mark` receipt on `/v1/events`. Its typed label and native
sequence must match the accepted `mark_telemetry` command. JSONL retains the event
sequence, label and native sequence, and the summary must meet
`telemetry_mark_visible_in_events`. A refused mark publishes no mark event. The
receipt establishes command acceptance and event publication; it does not certify
that the telemetry dump reached disk.

## Bare-motor speed feedforward A/B (P8-17, P8-13)

Owner decision 2026-09-14: nine bare ANCTL AS715N A6-EC motors, no load,
gearboxes or brakes, 131072 motor counts/rev, drive/software ratio 1/1.
Use only the matching pinned `bench_nine_motors_1to1` definition. The new cell
`startup` overrides accept only the two speed-feedforward keys; gearing,
tracking mode and direction still come from the model/profile. Existing
configurations omit these optional keys and retain byte-identical argv and their
existing no-write behavior. Factory defaults are source 0 and percentage 0;
omission does not reset or verify a previously changed drive value.

| Variant | Configuration | C01.13 / 0x2001:14 U16 | C01.14 / 0x2001:15 U16 |
| --- | --- | --- | --- |
| A | `config/machines/bench/nine-motors-1to1-ffA.json` | 0, none | 0, 0% |
| B | `config/machines/bench/nine-motors-1to1-ffB.json` | 1, internal reference | 1000, 100% |

Both parameters take effect immediately, but **restart the daemon and adapter
between A and B** so startup writes and readback prove the selected values on
all nine axes. Check configuration verification before Home. C01.15 is
`0x2001:16`, U16, 5..16000 Hz, factory default 318 Hz; retain and record its same
readback for both runs. These variants do not write the filter. The source and
percentage are the only A/B input differences, apart from their resulting
configuration/machine identities. Source values 3 and 4 are undocumented and
the compiler refuses them; supported sources are 0, 1, 2 and 5.

Both variants select `a6ec_bench_1to1_fast`: physical-speed bound 655360 counts/s
(5 rev/s), motor cap 300 rpm, per-axis `max_velocity` 25 rad/s, and
`jog_acceleration` 50 rad/s². Unverified measurement guards are `max_target_lead`
0.75 rad (15645.57 counts) and `following_error` 1 rad (20860.76 counts).
The 47 ms measured acquisition skew at 5 rev/s is 0.235 rev (30801.92 counts),
below half a turn. There is one necessary deviation from a speed-only profile:
`maximum_acquisition_duration_ns` is also tightened to 47000000. The inherited
128000000 declaration gives 83903 counts including the 16-count tolerance and
is rejected by the compiler. Native acquisition checks still reject actual
low-word/PDO ambiguity; full bracket assembly retains its existing age budget.
Validate actual skew and startup success on the bench; this is a measured input
with unverified margin, not a hardware qualification claim.

The unchanged trajectory acceleration cap is **0.5 rad/s²**, while benchdrive
uses at most half that cap. The unchanged position envelope is ±3 rad and each
move is bounded by 3 rad. Consequently the raised velocity cap alone does not
make a few hundred rpm reachable with these prepared moves. The commands below
request a reachable 0.75 rad/s cruise at 0.25 rad/s². Reaching a few hundred rpm
requires a separately approved acceleration/travel change; do not remove these
guards to make the measurement run. The simulator tests cover tooling and count
arithmetic, not A6 feedforward response.

Build with `make control`. Compile on the bench host, then start one native
daemon and one adapter in separate terminals (the orchestrator owns the existing
single-master bring-up and shutdown procedure):

```sh
CONFIG_A=config/machines/bench/nine-motors-1to1-ffA.json
CONFIG_B=config/machines/bench/nine-motors-1to1-ffB.json
DIR_A=$(build/rtctl compile --config "$CONFIG_A" --out build/bench-ff-config)
DIR_B=$(build/rtctl compile --config "$CONFIG_B" --out build/bench-ff-config)
SHA_A=${DIR_A##*/}
SHA_B=${DIR_B##*/}
mkdir -p /dev/shm/rt-core/live

# Native terminal, A (after the previous master has exited):
build/rtctl run --backend live --config config/machines/bench/nine-motors-1to1-ffA.json \
  --out build/bench-ff-config --socket /dev/shm/rt-core/live/ipc.sock
# Adapter terminal, using SHA_A from the compilation above:
build/rt-control --core-socket /dev/shm/rt-core/live/ipc.sock \
  --socket /dev/shm/rt-core/live/control.sock --backend live \
  --pair-id bench-nine-motors-1to1 --pair-revision 1 \
  --configuration-sha256 "$SHA_A"
```

The expected digests for the committed inputs are A
`9299ffc395baf25ea810615d7a0a564dfb6bca5f25ca634cd35f1364071acb2c` and B
`f90a8e601587be0faaf391c4cf1d7a28183584379aa794f6ebfba54f7096e127`.
Use the actual compiler output and matching adapter identity. Evidence paths
must be new; a repeat requires new names. The following are the direct binary
commands in the same form used by `bench-drive.sh`, with no positional phase or
private IPC operation:

```sh
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_A" --evidence build/ffA-home.jsonl --phase home --axis-mask 511 --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_A" --evidence build/ffA-axis0.jsonl --phase move --axis 0 --delta-rad 2.8 --velocity-fraction 0.04 --accel-fraction 0.5 --measure --confirm-motion
# HM35 sets each current shaft position as its new origin before the all-axis move.
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_A" --evidence build/ffA-rehome.jsonl --phase home --axis-mask 511 --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_A" --evidence build/ffA-nine.jsonl --phase move --deltas-rad '0:2.8,1:2.8,2:2.8,3:2.8,4:2.8,5:2.8,6:2.8,7:2.8,8:2.8' --velocity-fraction 0.04 --accel-fraction 0.5 --measure --confirm-motion
```

After A completes and Stop/Release inhibition is observed, shut down both
processes using the bench procedure. Restart with B, recheck every startup
readback, then repeat Home, the one-axis measurement and the all-nine measurement:

```sh
# Native terminal:
build/rtctl run --backend live --config config/machines/bench/nine-motors-1to1-ffB.json \
  --out build/bench-ff-config --socket /dev/shm/rt-core/live/ipc.sock
# Adapter terminal:
build/rt-control --core-socket /dev/shm/rt-core/live/ipc.sock \
  --socket /dev/shm/rt-core/live/control.sock --backend live \
  --pair-id bench-nine-motors-1to1 --pair-revision 1 \
  --configuration-sha256 "$SHA_B"
# Measurement terminal:
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/ffB-home.jsonl --phase home --axis-mask 511 --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/ffB-axis0.jsonl --phase move --axis 0 --delta-rad 2.8 --velocity-fraction 0.04 --accel-fraction 0.5 --measure --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/ffB-rehome.jsonl --phase home --axis-mask 511 --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/ffB-nine.jsonl --phase move --deltas-rad '0:2.8,1:2.8,2:2.8,3:2.8,4:2.8,5:2.8,6:2.8,7:2.8,8:2.8' --velocity-fraction 0.04 --accel-fraction 0.5 --measure --confirm-motion
jq 'select(.kind=="summary") | .response.measurements' build/ffA-axis0.jsonl build/ffB-axis0.jsonl build/ffA-nine.jsonl build/ffB-nine.jsonl
```

`--measure` requires a 1 kHz capture and valid cyclic 0x60F4 following error and
0x606C actual velocity. Each selected axis gets a `position` / `move_cycle`
record per cycle containing the wire target, raw and unwrapped feedback counts,
drive following error and drive actual velocity. `travel` / `move_measurement`
and the final `summary.response.measurements` report absolute peak target/feedback
lag, signed mean lag along the direction of motion at cruise, catch-up overshoot
(feedback ahead of the current commanded target), and settling ns after the
final target stops changing. Cruise requires two consecutive 20 ms target
velocity windows within the explicit two-count quantization allowance. A
triangle without observed cruise fails measurement. Compare the same axes and
command profiles, including overshoot and settling; lower cruise lag alone is
not sufficient. Dropped cycles, identity changes, missing PDO validity or an
observation deadline end with failure and Stop/Release, retaining typed reasons.

## Powered Halt stopping measurement (P4-05 R11)

Keep B running, Home again to leave room inside the soft limits, then run this
single-axis prepared move. Halt is the **public `halt` operation**, sent once
cruise is observed, with the same acquired session and generation as Start:

```sh
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/halt-home.jsonl --phase home --axis-mask 511 --confirm-motion
build/benchdrive --socket /dev/shm/rt-core/live/control.sock --pair-id bench-nine-motors-1to1 --pair-revision 1 --configuration-sha256 "$SHA_B" --evidence build/halt-axis0.jsonl --phase halt --axis 0 --delta-rad 2.8 --cruise-velocity-rad-s 0.5 --accel-fraction 0.5 --confirm-motion
jq 'select(.kind=="summary") | .response.measurements' build/halt-axis0.jsonl
```

The complete unhalted path must itself fit the travel and velocity/acceleration
limits and contain observable cruise; impossible requests receive
`halt_cruise_unreachable_within_travel` or `halt_cruise_exceeds_velocity_cap`.
This trajectory Halt uses the profile's 0.5 rad/s² stopping bound (10430.38
counts/s² at 1/1), not the 50 rad/s² jog bound. To repeat on another axis, Home
and replace `--axis` and evidence names; negative deltas test the other direction.

`halt_cycle`, `halt_receipt`, `halt_measurement`, `completion` and final `summary`
records preserve command/feedback data and the receipt sequence/fence. Zero
commanded velocity is count-resolved: the final wire target remains identical
for 100 ms. Feedback standstill requires 100 ms of consecutive samples within
a 16-count spread and within 16 counts of that target. The timestamps identify
the beginning of these verified windows. Settling is standstill minus the first
final held-target sample. The public API does not expose a native receipt
timestamp: latencies are signed ns from arrival of the public Halt response;
negative values mean the count-resolved stop preceded receipt delivery.
Request-to-zero ns is also recorded as an upper bound from request submission.
Stopping distance is the maximum excursion along motion from the last feedback
sample at or before public receipt; net displacement to final feedback is also
recorded. The reference sample timestamp and its actual ns offset from receipt
preserve capture-alignment uncertainty. Retain request/response timestamps when
assessing that uncertainty.

Commanded deceleration uses consecutive 20 ms position-difference windows and
actual monotonic timestamps. The explicit two-count rounding allowance is
5000 counts/s²; both this allowance and the configured bound appear in the
record. An excess fails with `halt_deceleration_bound_exceeded`. Drive-velocity
deceleration is recorded separately without claiming a firmware acceleration
guarantee. Completion requires correlated native Done, a terminal trajectory
handle, armed idle, unchanged session/generation, and a held target equal to the
last commanded target. These are observed **before** the tool's final Stop and
Release inhibit outputs. Typed public/native refusals remain request records;
there is no automatic retry or change of authority to make Halt succeed.

`TestSimulationMoveMeasurement`, `TestSimulationHaltStopsWithinBound`,
`TestSimulationMeasurementRefusals` and `TestMeasurementRejectsGapsAndUnavailableFields`
exercise these contracts in software. R11 remains open for the orchestrator's
powered stopping measurements and review of the recorded count/ns bounds.
Before returning to an original configuration that omits the new keys, restart
with A and verify source 0 / percentage 0: the original configuration preserves
its no-write behavior and would otherwise retain the last drive values.

## Driving the bench from offline programming

Run OLP only on the development host. The orchestrator must first provision the
cell's public rt-control HTTPS listener with its commissioned pair, OLP principal
certificate and live CRL. Reuse the existing cell CA; never use the Deck's client
certificate or copy the CA/server private keys to OLP. The full PKI and jump-host
procedure is in [Offline programming: Remote cell](../../offline-programming/v1/docs/RT_CORE_EXECUTION.md#remote-cell).

Use this remote-only catalogue as `DEV_STACK_OLP_CELLS`, which appends the entry
to Local simulation. Alternatively include the local entry documented in
[Choosing a machine](../../offline-programming/v1/docs/RT_CORE_EXECUTION.md#choosing-a-machine)
and set `OFFLINE_PROGRAMMING_CELLS` directly.

```json
{
  "schema": "offline-programming.cell-catalogue.v1",
  "cells": [{
    "cell_id": "<cell>",
    "label": "Test bench (Rosie 1420, six motors)",
    "role": "TEST BENCH",
    "models": ["rosie_1420_v1"],
    "requested_mode": "real",
    "execution": {
      "address": "https://127.0.0.1:8443",
      "ca_file": "pki/ca.pem",
      "certificate_file": "pki/olp-dev.pem",
      "key_file": "pki/olp-dev-key.pem",
      "binding": {"pair_id": "bench-rosie1420", "revision": 1, "configuration_sha256": "c1eced1c8f281af244792b5be4b1fbd41c690b9d024c84530d7c681a689aa11d"},
      "axis_mask": 63,
      "expected_backend": "ethercat"
    }
  }]
}
```

The supplied bench binding is `rosie_1420_v1`, pair `bench-rosie1420` revision 1,
J1–J6 (mask 63), configuration
`c1eced1c8f281af244792b5be4b1fbd41c690b9d024c84530d7c681a689aa11d`.
The cell config is a bench cell, `rt-core/config/cells/<cell>.json`; its native
configuration is `rt-core/config/machines/bench/rosie1420-six-motors.json`.
Use the supplied PKI test-root client `olp-dev`; place its CA and client
certificate/key at the catalogue's `pki/` paths. Do not initialize a replacement CA.
The server checks the manifest endpoint, pair, revision and compiled digest before
selection, then requires authenticated Describe to match. Another relay remains listed
with `cell_configuration_missing` and cannot be selected.

Keep the bench listener on loopback. From the development host, the orchestrator
can establish `ssh -N -J operator@jump-host -L 127.0.0.1:8443:127.0.0.1:8443 operator@<servo-host>`.
The catalogue origin remains `https://127.0.0.1:8443`, verified against its server
SAN and CA. SSH does not replace mutual TLS. Confirm Test bench (Rosie 1420, six motors), its bound
model, EtherCAT backend and tunnel host in the connected-target line before any
explicit motion action. Requested `real` refuses a simulation backend.

Selection grants no lease, Home or arming. An existing session receives Stop
then Release before switching; switching during playback is refused. ARM, native
Home, Load and Play remain subject to this document's commissioning and bench
authorization procedure. STOP ends control and invalidates the prepared program.
Cartesian hold-to-jog and Tesseract stay on the independent local simulation;
process-I/O programs remain refused. A lost tunnel or TLS refusal never causes
motion replay.

`TestPickerComposed` exercises both catalogue targets through two local simulation
instances, including an HTTPS listener with throwaway PKI, playback on each and
mid-play/model/mode refusals. `TestRTCoreComposedHTTPS` revokes a client after a
successful Describe and proves the running listener refuses a fresh handshake.
`TestReviewLauncher*` and `TestReviewDisabledJogHasTypedReason` retain the launcher
review probes. These development-host tests do not qualify powered bench motion.

## Limits of the evidence

WSL and the synthetic plant do not qualify actual EtherCAT timing, A6 firmware,
wrap behavior or mechanical stopping. Absolute-anchor restoration is only valid
for explicitly qualified signals/identity; other axes require documented re-home.
Future Yaskawa drives need their actual ESI/profile and this same qualification;
synthetic mixed-profile tests prove architecture behavior, not product support.

## REV-04 frame-loss qualification prerequisite

Before G1, record steady-state missed/incomplete exchange counts under the declared
host load, plus recovery after deliberately lost exchanges on the authorized bench.
The current policy tolerates zero consecutive misses after readiness: outputs are
inhibited, configuration/Home evidence is invalidated and explicit recovery is
required. Record its operational effect; no unmeasured miss tolerance is enabled.
Any proposed tolerance must separately bound output behavior and evidence age,
retain an urgent revoke path and pass fault injection. Do not call cached WKC or
position a new receipt. This checklist records required qualification, not a
completed bench test.
