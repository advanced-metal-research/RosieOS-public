> **Nine-motor bench, dedicated automatic test bench:** the bench is an unattended test
> resource. No human authorization, approval, confirmation, label, or per-run
> permission exists. Invoke automatic tests directly; machine control state is
> coordination and evidence, not human permission.

# A6-EC Drive Notes

This page consolidates the A6-EC drive guidance that matters for `steamdeck/real/v4`, especially servo motion, HM35 homing, native drive frames, and restart trust.

Source notes integrated here:

- the 2026-04-15 A6-EC manufacturer notes (retired `robot/v2` tree)
- the A6-EC frame-semantics and native-Home notes (retired `robot/v2` tree)
- GradientOS `docs/ethercat/bringup.md`
- GradientOS `docs/ethercat/igh.md`
- GradientOS `docs/ethercat/rtcore_jog.md`
- GradientOS `RTOS-ETHERCAT-PLAN/Plan overview & to-do.md`
- GradientOS `RTOS-ETHERCAT-PLAN/Uncertainties.md`

The PDF manual is stored beside this page as `A6-EC_series_servo_drive_manual.pdf` and is tracked through Git LFS.

## Robot v4 Runtime Authority

The selected persisted Jetson `robot.urdf` revision is the sole Robot v4
configuration authority for exact gear ratios, speed limits, lower/upper
limits, and Home positions. Pair records, `current.json`, receipts, and hashes
bind revision identity and integrity; they do not independently authorize
scalar values. Factory defaults are parsed from the embedded seed URDF through
the same strict path.

Manifests supply topology, routing, physical wiring, and sign mapping only.
They do not supply runtime ratios, speeds, limits, or Home values and cannot
authorize a config-dependent fallback.

On the CoE wire, Robot v4 reads and writes the mechanical-ratio numerator at
object `0x2010`, subindex `0x19`, and denominator at subindex `0x1A`.
`C10.18/C10.19` are the vendor UI labels for those parameters. They are not
separate manifest seed fields. `rosie_1400_v3` J5 is exactly `100/11`
(`9.090909090909092`); `rosie_1420_v1` J5 is exactly `100/1`.

## Vendor HM35 And Ratio Guidance

The 2026-04-15 vendor correspondence clarified that "load" and `RM` in rotary mode mean the output shaft after gearbox reduction, not the motor shaft. In rotary mode:

- `RM = Encoder Resolution * Mechanical Gear Ratio Numerator / Mechanical Gear Ratio Denominator`.
- When `C10.1A / C10.1C` are zero, `RM` is determined by `C10.18 / C10.19`.
- The vendor recommends programming the actual mechanical ratio into `C10.18 / C10.19`.
- If `C10.18 / C10.19` remain `1:1`, `6064` and `U40.28` remain motor-side values, and the host must do output-shaft conversion itself.
- `6091` is not the rotary-mode reconstruction source. It is the electronic gear ratio for command-unit conversion.

Changing `C10.18 / C10.19` changes the coordinate system immediately. A fresh homing operation is required after the ratio change, but a power cycle is not required just to make the ratio change take effect.

## HM35 Is Not Just A 607C Write

A direct write to `607C` is not enough to establish a valid homing/reference state.

After successful HM Method 35:

- `6064` immediately reflects the new coordinate system.
- `6041 bit 12 = 1` means homing attained.
- `6041 bit 15 = 1` means the absolute coordinate system has been established.
- `6041 bit 13 = 0` means no homing error.
- No active alarms should be present.

With only a direct `607C` write:

- `6064` does not change immediately.
- Bits 12 and 15 are not set.
- The reference state is not trustworthy by vendor criteria.

The vendor explicitly identified `0x9650` as an expected successful state for HM35 completion. The practical trust signature after a fresh home is:

- `bit12 = 1`
- `bit15 = 1`
- `bit13 = 0`
- no active alarms

## 607C And Persistence

In rotary mode, `607C` should stay in `0 .. RM-1`. Negative values may be accepted by the drive at a low level, but the vendor says they are semantically incorrect in rotary mode. For seam-adjacent homes, prefer a positive value in the valid rotary frame rather than a negative value. Robot v4 now uses a midframe `607C=RM/2` HM35 offset so the current mechanical home becomes joint zero while the CSP `6064/607A` command-frame seam stays far from joystick home.

`607C` persistence depends on `C13.10`. The recommended default is `C13.10 = 1`, which enables automatic EEPROM saving. Homing persistence is affected by this setting as well.

`F31.10` is an encoder-data recovery tool, not a normal commissioning step. It is for cases like encoder battery or multi-turn faults, multi-turn overflow, or catastrophic data loss. After `F31.10 = 4`, physical homing must be performed again.

RTCore restart persistence follows the same ownership model. EtherCAT carries
configuration and feedback, but it is not the persistent configuration store.
A successful HM35 stores a keyed V2 profile at:

```text
/var/lib/robot-v4-rtcore/profiles/<home-profile-key>.v2
```

The key binds the V2 domain, Jetson, model, revision, snapshot hash, generated
URDF hash, mapped mask, all exact ratio fractions, and all Home positions.
Activating a new revision or generated-URDF hash derives a different key,
including after a speed-only or joint-limit-only edit; Apply alone does not
change the active key or epochs. The old unkeyed `hm35-anchors.v1` file is
historical evidence only and cannot authorize Home restoration or a command.

On startup or every EtherCAT reconnect, RTCore first invalidates in-memory
Home. It restores Home only after the selected persisted URDF identity, exact
drive SDO readback, matching keyed profile, fresh complete operational PDO
exchange, vendor HM signature, and encoder consistency checks all pass.
Blocking SDO reads and profile-file writes run outside the realtime cyclic
thread. Missing or mismatched identity, profile, SDO, `607C`, position,
encoder, or modulo-RM evidence fails closed.

Generic EtherCAT startup must not invent or download a ratio, `607C`, `6098`,
or `60E6`. Exact ratio writes are permitted only during explicit V2 Activate:
the owner reopens the selected persisted URDF, RTCore reconciles
`0x2010:0x19/0x1A` in a safe Disarmed posture, and fresh exact readback and
epochs must pass. Downloading `607C` without executing HM35 does not establish
a trustworthy absolute-coordinate state.

## Recommended Vendor Flow

1. Set `C00.07 = 4` for Absolute Rotation Mode.
2. Set the true mechanical-ratio numerator and denominator
   (`0x2010:0x19/0x1A`; vendor labels `C10.18/C10.19`).
3. Keep `C13.10 = 1`.
4. Move the mechanism to the desired zero position.
5. Set `607C` to the desired home value.
6. Switch to HM mode with `6060 = 6`.
7. Set `6098 = 35`.
8. Set `60E6 = 0` for absolute homing-position calculation.
9. Trigger the controlword sequence `6 -> 7 -> 15 -> 31`.
10. Wait for `6041 bit 12 = 1`, `bit 15 = 1`, and `bit 13 = 0`.
11. Return to CSP with `6060 = 8`.

## Frame Semantics

The core project lesson from the v2 A6-EC investigation was that several position frames were being mixed together:

- raw absolute encoder counts
- drive reference/home counts
- stored software anchors

These are related frames, not interchangeable fields.

The A6-EC exposes at least three distinct position families:

- Raw absolute encoder motor frame:
  - `U40.1C`
  - `U40.1E`
  - `U40.20/.22`
- Rotation-mode or mechanical bridge frame:
  - `U40.28`
  - `U40.2A/.2C`
  - `C10.*`
- CSP/reference/home frame:
  - `6064`
  - `607A`
  - `6063`
  - `60FC`
  - `U40.16`

The previous bug class was not random telemetry; it was frame mixing. Future motion, homing, and UI work should name which frame a value belongs to before using it for planning, display, validation, or persistence.

## Multi-Turn Truth Model

The preferred live absolute source for multi-turn reconstruction is:

- first choice: `U40.20 / U40.22`
- second choice: `U40.2A / U40.2C`

The 64-bit combine math is:

- `combined = ((high & 0xffffffff) << 32) | (low & 0xffffffff)`
- if `high` has bit 31 set, then `combined -= 2^64`
- `absolute_counts = signed_i64(U40.22:U40.20)`

The older Chapter 5 reconstruction remains useful as a check:

- `COUNTS_PER_MOTOR_REV = 131072 = 2^17`
- `single_turn_mod = U40.1C mod 131072`
- `signed_multiturn_revs = sign_extend_16(U40.1E)`
- `reconstructed_abs_counts = signed_multiturn_revs * 131072 + single_turn_mod`

For multi-turn-capable joints, pure `6064` is not enough for canonical truth because it wraps at `RM`. The vendor's statement that `6064` is authoritative is interpreted as "authoritative within shaft space." Multi-turn canonical truth remains rooted in `U40.20/.22` plus a persistent home anchor, with `6064` used as a shaft-frame consistency witness.

## Command Frame Rules

`607A` is the target-position command object in CSP mode. Do not issue position commands directly through the `U40.20/U40.22` register family; those objects are read-only feedback sources for multi-turn reconstruction.

For multi-turn joints, the target sent to `607A` must land in the shaft turn the drive is currently observing. Otherwise the drive can chase the wrong turn and raise following-error or excessive-increment faults.

The v2 command-frame rule was:

- compute `target_counts_raw = round((canonical_q + master_offset) * sign * counts_per_unit)`
- fold to the nearest shaft turn of live `6064`:
  - `delta = target_counts_raw - live_6064_counts`
  - `delta -= round(delta / RM) * RM`
  - `target_607A = live_6064_counts + delta`
- reject oversized deltas or seam-straddling point sequences until a validated seam policy exists

For `steamdeck/real/v4`, this reinforces why HM35 home-relative targets and rotary seam guards matter: the command path must not silently cross the `0/RM` seam or mix absolute multi-turn truth with the single-turn CSP command frame.

Robot v4 live J0 evidence on 2026-05-31 showed why the HM35 offset matters:
with `607C=0`, J0 home was latched close to raw `0/RM`; negative safe motion
worked, but positive motion immediately wanted a target below raw zero and the
RTCore seam guard correctly held before the seam. The next RTCore policy is to
run HM35 with `607C=RM/2`, then keep publishing home-relative zero from the
latched HM35 counts. That preserves the operator's joint-zero semantics while
placing the raw command-frame seam roughly half a revolution away from the
normal bidirectional joystick sweep.

## Restart Trust And Persisted Anchors

The v2 workstream found that on the tested A6-EC firmware, `6041 bit 15` could clear after power cycle even when `6064`, `U40.20/.22`, and `607C` restored cleanly. The vendor later confirmed that bit-15 clearing after restart is expected for that firmware and that host-side consistency checks are valid.

The robust restart trust path is:

- an initial HM35 is still required to establish a home anchor;
- after restart, read `U40.20/.22` and paired `6064`;
- verify modulo-`RM` consistency between the persisted anchor and the paired drive frame;
- accept truth only if the multi-turn data is valid and the consistency check passes;
- fail closed if the anchor is missing, multi-turn feedback is invalid, or the mod-`RM` agreement fails.

Signals that multi-turn data is no longer reliable include:

- `Er20.8` encoder battery failure
- `Er20.9` encoder multi-turn abnormality
- `ErA0.1` encoder multi-turn overflow
- `ALF9.0` encoder battery warning
- `0x603F` error code
- `0x203F` extended error code
- `U41.05 bit 13` multi-turn fault
- `U41.05 bit 14` battery fault
- `U41.05 bit 15` battery warning
- `C10.15` multi-turn overflow flag

The vendor considered a 16-count stationary plausibility tolerance reasonable for these consistency checks.

## Native-Home Preconditions

Vendor guidance for HM35 is that the motor must be stationary and inactive. The settled precondition is:

- the axis must have exited `OperationEnabled`;
- motion intent should be cleared before the HM35 transaction starts;
- a conservative `~500 ms` wait for the state transition is reasonable;
- if the drive cannot leave `OperationEnabled`, abort instead of issuing HM35 into an unsafe state.

This supports the `steamdeck/real/v4` design where HM35 is RTCore-owned, active jogs are blocked while home is pending/active, and homing runs as a service-mode transaction rather than a normal motion command.

## Probe And Validation Checks

The v2 probe workflow is still useful because it compares all bridge families in one snapshot:

- `combined(U40.20/.22) ~= sign_extend_16(U40.1E) * 131072 + (U40.1C mod 131072)`
- `6063 ~= 6064 * 6091`
- `60FC ~= 6062 * 6091`
- `combined(U40.2A/.2C) ~= U40.28 * C10_ratio`

These checks may differ by a few counts because SDO reads are live and sequential. A small `0..3` count drift in repeated reads is normal enough that single-sample one-count failures should not drive conclusions. Systematic divergence matters more.

## GradientOS EtherCAT Bring-Up Notes

The local GradientOS checkout was not readable from this agent because macOS denied access to `/Users/operator/Documents/GitHub/GradientOS`. The details below come from the public GradientOS mirror and are still useful for `steamdeck/real/v4` because they describe the same A6-EC EtherCAT/DS402 bring-up surface.

Hardware and master checks:

- Wire the RevPi EtherCAT NIC to the first drive's EtherCAT `CN3 (IN)` port.
- Chain additional drives from `CN4 (OUT)` to the next drive's `CN3 (IN)`.
- Expect `100BASE-TX` full duplex, line topology, CoE/CiA402, and no switch/router in the motion chain.
- The drive supports DC/SYNC0 only. The sync cycle must be an integer multiple of `250 us`; invalid or missing sync can show `Er74.0`, `Er74.1`, or `Er74.2`.
- `ethercat master` showing `Active: no` is normal while no userspace `libecrt` RTCore application has activated the master.
- If `Link: UP` but Rx frames stay at zero and no slaves are discovered, debug wiring, port selection, drive EtherCAT mode, and NIC binding before debugging PDOs or DS402 state.

GradientOS used these fixed A6-EC PDO sets for bring-up:

- RxPDO `0x1702`: `6040` control word, `607A` target position, `60FF` target velocity, `6071` target torque, `6060` mode selection, `60B8` touch probe function, `607F` max speed.
- TxPDO `0x1B02`: `603F` fault code, `6041` status word, `6064` position actual value, `6077` torque feedback, `6061` mode display, `60B9/60BA/60BC` touch probe status/edges, `60FD` digital input status.
- Active assignments are selected through `0x1C12:01` for RPDO and `0x1C13:01` for TPDO.
- PDO mapping edits must happen in Pre-Op and are not EEPROM-persistent; reapply or verify them after drive power cycles.
- `ethercat pdos` can show SII/default mappings, so use CoE reads of `0x1C12:01` and `0x1C13:01` when verifying the active assignments.

These points map directly onto `steamdeck/real/v4` diagnostics: before trusting UI telemetry or a failed HM35 result, verify the physical chain, selected PDOs, WKC/DC health, and drive mode display.

## GradientOS Motion And Safety Lessons

GradientOS' RTCore bring-up notes are not HM35-native, but the motion safety lessons carry over:

- Align target position `607A` to live feedback `6064` before enabling or switching modes, then keep target increments small. This avoids `Er87.1`/`Er87.2` excessive-increment faults.
- Start single-axis motion tests tiny, around `0.005..0.01 rad`, and enable only the axis being mapped.
- `0x607F` max speed is a useful drive-side cap, but host-side per-cycle clamps and stale-command guards are still needed.
- The A6-EC supports DC/SYNC0 and its sync cycle must be an integer multiple of
  `250 us`, so joystick-to-servo latency work should first verify the active
  EtherCAT cycle, WKC/DC health, and SYNC0 error state before changing motion
  gains or command rates. Invalid cycle/sync setup can create drive errors that
  look like joystick latency or command stalls.
- Treat SYNC0 cadence as an EtherCAT timing boundary, not a quick joystick
  latency knob. Before changing it, prove active cycle value, `250 us` multiple
  compliance, WKC/DC health, PDO freshness, RTCore loop jitter, active `607F`,
  host-step/target-lead envelope, quiet faults, and live release-stop timing in
  the same `test-session-id`.
- Keep the low-latency command path small: the PDO set already carries `607A`
  target position, `607F` max speed, mode, status, `6064` feedback, torque, and
  digital inputs. Do not move latency-critical diagnostics into extra SDO reads
  in the joystick path; publish richer evidence from RTCore/NATS side channels.
- Treat `1C12`/`1C13` minimal PDO control-path boundary as part of the release-stop latency gate: active RxPDO `0x1702` and TxPDO `0x1B02` already
  carry the fields needed for the direct Steam Deck UDP -> motion-server UDP ->
  realtime-core `CmdJogV1` -> A6-EC CSP `607A` path. Do not add PDO objects,
  SDO reads, or polling to the joystick path unless active assignment evidence,
  WKC/DC health, PDO freshness, and live release-stop timing prove the current
  mapping is the bottleneck.
- `60FF` target velocity is PDO-mapped, but it is not enough to switch the
  joystick release-stop path away from CSP `607A`/`CmdJogV1` semantics. Treat
  target-velocity use as a separate mode/semantics boundary: before using it for
  latency, prove the active mode, drive interpretation, RTCore command envelope,
  release-stop timing legs, and fault behavior in the same `test-session-id`.
- `6071` target torque is also PDO-mapped, but joystick latency work must treat
  it as a different control mode boundary, not a faster CSP stop. Use `6077`
  torque feedback as diagnostic evidence for load/current behavior, while any
  future torque-command experiment must prove active mode, torque limits,
  RTCore safety envelope, quiet faults, and live release-stop timing before it
  can affect the Steam Deck control path.
- `60B8` touch probe function is PDO-mapped, and `60B9/60BA/60BC` expose
  touch-probe status/edges in the TxPDO. Treat these as diagnostic timing
  evidence candidates only, not a release-stop control path. A future
  touch-probe experiment must avoid polling/SDO in the joystick path and prove
  manual-backed edge semantics, active `60B8` configuration, monotonic
  same-session timestamp correlation, quiet faults, and live release-stop
  timing before it can influence Steam Deck joystick-to-servo latency work.
- TxPDO status fields `603F`, `6041`, `6061`, `6064`, `6077`, and `60FD`
  should explain latency, stalls, drive state, actual position, torque/current,
  and digital input state from the cyclic feedback stream. Treat them as
  diagnostic-only evidence for release-stop timing and fault attribution; do not
  add SDO reads, polling loops, or NATS/UI control dependencies to the joystick
  path just to fetch these values.
- Treat `606C` actual speed as a smoothness/stop diagnostic boundary, not a
  joystick control input. It can help prove a Steam Deck sweep has no hidden
  stops, sign inversions, or seam jumps when correlated with `6064`, `607A`,
  `6062`, `60F4`, active-jog flags, and motion-server receive-to-output
  latency, but it must stay on the cyclic feedback/telemetry side. Do not add
  `606C` polling or velocity-derived command feedback into the Steam Deck
  control path until same-session telemetry proves PDO freshness, monotonic
  sample ordering, smooth actual-speed sign/magnitude through both sweep
  directions, commanded-vs-actual stop evidence, and three live release-stop
  timing samples.
- Treat `607F`, host-side max-step-per-cycle, and active-jog target lead as a
  coordinated latency envelope. Raising `607F` alone can remove a drive-side
  cap, but it must not bypass the RTCore target-step and raw seam-buffer guards
  that prevent `607A` jumps and following/excessive-increment faults.
- Treat `607F` max speed as a drive-side cap boundary, not a standalone
  joystick latency fix. Before changing it, prove the active `607F` value,
  host max-step-per-cycle, active-jog target lead, `607A` increment envelope,
  WKC/DC and PDO freshness, quiet faults, and live release-stop timing in the
  same `test-session-id`.
- Treat host max-step-per-cycle and active-jog target lead as one RTCore
  command-generation boundary. Increasing lead can reduce apparent lag only if
  it stays inside the `607A` increment envelope, seam guards, `607F` cap, WKC/DC
  health, PDO freshness, quiet faults, and same-session live release-stop
  timing evidence; otherwise it can hide stalls or create excessive-increment
  risk.
- Treat `607F`, host max-step-per-cycle, and active-jog target lead as a
  coordinated envelope reconciliation boundary, not independent speed knobs.
  Before changing any one of them for joystick responsiveness, prove active
  `607F`, active host max-step-per-cycle, active-jog target lead, bounded
  `607A` increments, seam guard margins, target-vs-feedback delta, WKC/DC
  health, PDO freshness, quiet following/excessive-increment faults, smooth live
  sweep both directions, commanded-vs-actual stop telemetry, and three live
  release-stop samples under the same test-session-id.
- Treat the `607A` increment/seam boundary as a release-stop safety boundary,
  not a shortcut for apparent joystick responsiveness. Larger target jumps,
  looser seam policies, or stale shaft-turn selection are blocked until live
  `6064` shaft-turn witness data, nearest-turn folded `607A` targets,
  host-step/active-jog lead, `607F` cap, quiet following-error and
  excessive-increment evidence, commanded-vs-actual stop telemetry, and three
  live release-stop timing samples prove the current increment envelope is the
  bottleneck.
- Treat current-feedback-based target generation as a measured target
  generation boundary, not a reason to chase stale commands. It may reduce
  apparent lag only if the same `test-session-id` proves active-jog intent,
  target-vs-feedback delta, bounded `607A` increments, seam safety, PDO
  freshness, quiet faults, commanded-vs-actual stop telemetry, and live
  release-stop timing. Otherwise it can hide "fresh joystick input but no servo
  movement" behind command freshness.
- Treat bus-running disable/disarm stop as the release-stop method boundary:
  stopping the EtherCAT master can produce synchronization-loss alarms such as
  `ErC1.1`, so joystick release-to-stop must keep RTCore and the EtherCAT bus
  alive and apply the fastest direct UDP zero/stop command before any
  master-stop or bus-reset behavior is considered.
- Treat metrics side-channel observability as a latency boundary. RT loop Hz,
  jitter, WKC, lost-frame, PDO freshness, and status diagnostics should be
  published through metrics/NATS/UI side channels, not inserted into the
  Steam Deck release-stop control path. The joystick path stays direct UDP to
  motion-server to realtime-core `CmdJogV1` to A6-EC CSP PDO.
- Treat RTCore-owned safety gates as a latency boundary: fault, stale-command,
  home-valid, limit, WKC/DC, and unexpected-mode checks must remain in RTCore,
  not UI or non-RT layers. Safety should fail fast next to the `CmdJogV1`/PDO
  path without adding UI, file, NATS control, SDO, or polling hops to joystick
  release-stop.
- Treat HM35 as an RTCore-owned home transaction boundary, not a UI-side
  `607C` write or runtime shortcut. Active jog must be blocked while home is
  pending or active, HM35 must restore CSP before joystick motion resumes, and
  home-count changes must be telemetry-visible under the same `test-session-id`
  before any release-stop or sweep evidence can be trusted.
- Treat any drive-native exact-ratio change as a commissioning boundary, not a
  runtime latency knob. Changing `0x2010:0x19/0x1A` changes the `6064`/`607A`
  coordinate frame immediately, so a changed persisted URDF must be explicitly
  Activated, physically reconciled, and paired with a matching V2 HM35 profile,
  frame-specific telemetry, same-session release-stop evidence, and safe-range
  regeneration before joystick motion can trust the new frame.
- Treat `6091` electronic gearing as a command-unit conversion boundary, not a
  Steam Deck latency knob. Vendor guidance says `6091` is not the rotary-mode
  reconstruction source; it is the electronic gear ratio for command-unit
  conversion, while `C10.18`/`C10.19` describe the mechanical ratio used for
  rotary `RM`. Do not reinterpret `6063`/`6064`, `60FC`/`6062`, `607A`, or
  joystick scale through `6091` until same-session telemetry proves active
  `6091`, exact persisted-URDF ratio context, and host wiring/sign mapping,
  direction-correct commanded-vs-actual motion, bounded `607A`, and three live
  release-stop timing samples.
- Treat `6040` halt/quick-stop, `605A` quick-stop option, and `6085`
  quick-stop deceleration as a drive-state transition boundary, not the primary
  Steam Deck release-stop path. The fastest current path remains direct UDP
  `CmdJogV1` to CSP 607A zero/stop while RTCore stays running; halt or
  quick-stop behavior should not replace that path until same-session evidence
  proves controlword timing, drive state/statusword transitions, quiet faults,
  commanded-vs-actual stop telemetry, and three live release-stop samples are
  better and safe.
- Treat torque/current limits as a measured safety envelope boundary, not a
  joystick latency shortcut. The A6-EC manual exposes `6072` max torque,
  `60E0` positive torque limit, `60E1` negative torque limit, and `6077`
  actual torque; current/torque faults can make a "fast stop" look better while
  hiding saturation or insufficient torque. Do not change torque/current limits
  for Steam Deck release-stop until same-session telemetry proves active
  limit values, rated motor/drive current and torque, `6077` actual torque,
  amp-fluctuation/current-spike evidence, quiet current/following faults,
  commanded-vs-actual stop telemetry, and three live release-stop samples.
- Treat gain/filter loop tuning as a stability boundary, not a quick Steam Deck
  latency fix. The A6-EC manual describes position, speed, and current loops,
  including `C01.00` position loop gain, `C01.01` speed loop gain, `R22.01`
  current loop response, axis current low-pass filtering, `C01.18` torque
  feedforward filtering, and `C01.1B` speed loop control behavior. Do not tune
  loop gains or filters for joystick feel until same-session telemetry proves
  active gain/filter values, RTCore loop jitter, target-vs-feedback error,
  `6077` torque/current evidence, quiet current/following faults,
  commanded-vs-actual stop telemetry, and three live release-stop samples.
- Treat PP-mode setpoint/reference semantics as a mode boundary, not a CSP
  joystick latency shortcut. The A6-EC manual says in PP mode `6040` bit 5 can
  wait for the current position reference or interrupt it with a new reference,
  while `6040` bit 6 changes whether `607A` is absolute target position or
  incremental displacement. Steam Deck release-stop must not borrow those PP
  setpoint/reference behaviors unless same-session evidence proves active
  mode, `6040` bit semantics, `607A` absolute/incremental interpretation,
  seam/increment safety, quiet faults, commanded-vs-actual stop telemetry, and
  three live release-stop samples.
- Treat SYNC0 multiple selection as a manual reconciliation boundary. Current
  `steamdeck/real/v4` evidence treats the active `1000 us` cycle as a valid `250 us`
  multiple, while the checked-in A6-EC PDF extraction also found a work-mode
  note saying the synchronization cycle must be an integer multiple of `125 us`
  and that the supported period range varies by motion mode. Do not change
  SYNC0 cadence, RTCore command period, or interpolation-period assumptions
  until the `125 us` versus `250 us` language is reconciled against active
  drive mode, active cycle value, SYNC0 error state, WKC/DC health, RTCore loop
  jitter, PDO freshness, and three live release-stop samples.
- Treat SYNC0/RTCore command-period alignment as a one-turn timing boundary,
  not a blanket reason to raise command rate. The active cycle is currently
  `1000 us`, while manual notes reference `125 us`/`250 us` multiples. Before
  changing SYNC0, RTCore command period, interpolation-period assumptions, send
  cadence, or packet coalescing, prove active SYNC0 cycle, RTCore command
  period, 250 us multiple compliance, manual 125 us reconciliation, WKC/DC
  health, SYNC0 error state none, RTCore loop jitter, PDO freshness, direct UDP
  receive-to-output timing, commanded-vs-actual stop telemetry, and three live
  release-stop samples under the same test-session-id.
- Treat following-error windows and feedforward offsets as a measured evidence
  boundary, not a Steam Deck release-stop shortcut. The A6-EC manual describes
  `6065` following-error window and `6066` following-error timeout: if the
  difference between `6062` position reference and `6064` position actual value
  stays outside `+/-6065` longer than `6066`, `Er47.0` position deviation too
  large occurs. It also exposes `60F4` position deviation plus CSP `60B0`
  position offset, `60B1` speed/feedforward deviation, and `60B2` torque
  offset. Do not relax following-error windows, inject offsets, or use
  feedforward objects as joystick latency fixes until same-session telemetry
  proves active `6065`/`6066` values, `6062`/`6064`/`60F4` commanded-vs-actual
  stop behavior, active `60B0`/`60B1`/`60B2` values, `6077` torque evidence,
  quiet `Er47.0`/following-error faults, bounded `607A` increments, RTCore loop
  jitter, PDO freshness, and three live release-stop samples.
- Treat position/speed reach windows as status diagnostics, not release-stop
  proof. The A6-EC manual says `6067` and `6068` define the PP-mode position
  reach threshold/time window, with `6041` bit 10 set only in PP mode when
  `6062` and `6064` are within the threshold long enough. It also says `606D`
  and `606E` define PV mode speed reach, while `606F` and `6070` define
  PV mode zero-speed threshold/time. Some mode tables mark position reach as
  unsupported or always `1`. For CSP joystick release-stop, do not use `6041`
  bit 10, `6067`/`6068`, or `606D`/`606E`/`606F`/`6070` as stop proof until
  same-session telemetry proves active mode display, active window values,
  `6062`/`6064` commanded-vs-actual stop, `606C` actual speed, `60F4` position
  deviation, bounded `607A` increments, quiet faults, and three live
  release-stop samples.
- Treat `607E` reference polarity as a commissioning/direction boundary, not a
  Steam Deck latency shortcut. The A6-EC manual says `607E` bit 7 multiplies
  the position reference by `-1` and reverses motor direction in
  standard-position or interpolation mode, bit 6 multiplies the speed reference `60FF`
  by `-1` in speed mode, and bit 5 multiplies the torque reference `6071` by
  `-1` in torque mode; the feedforward sections also say bit 6 sets the
  polarity of the `60B1` communication feedforward signal in CSP. Do not change
  `607E` to improve joystick feel, fix sign mistakes, or hide release-stop lag
  until same-session telemetry proves active `607E`, persisted-URDF ratio and
  host wiring/sign mapping,
  active mode display, direction-correct `6064`/`607A` behavior, `60FF`/`6071`
  mode semantics, `60B1` feedforward polarity, quiet direction/following faults,
  bounded `607A` increments, commanded-vs-actual stop telemetry, and three live
  release-stop samples.
- Treat software limits and overtravel status as safety boundaries, not
  release-stop latency tools. The A6-EC manual says `607D.01` and `607D.02`
  define minimum/maximum software position limits relative to mechanical zero,
  `607C` must stay inside those limits or the drive can report `Er84.3`, and
  reversed min/max or invalid rotary limits can report `Er84.1`. It also says
  `6041` bit 11 reports internal software position limit status, `60FD` bits
  expose reverse/forward overtravel inputs, and `ALF2.0`/`ALF2.1` or fault codes
  `0x5443`/`0x5444` report forward/reverse overtravel. Do not adjust `607D`
  limits, rely on overtravel stops, or use limit-status transitions as proof of
  a fast Steam Deck release-stop until same-session telemetry proves active
  `607D.01`/`607D.02`, `607C` inside limits, `6041` bit 11 clear, `60FD`
  overtravel bits clear, no `ALF2.0`/`ALF2.1` or `0x5443`/`0x5444`, host
  safe-range margins, bounded `607A`, commanded-vs-actual stop telemetry, and
  three live release-stop samples.
- Profile-mode objects such as `6083`/`6084` are useful configuration audit
  evidence, but CSP joystick latency should be tuned through SYNC0 cycle
  discipline, PDO freshness, current-feedback-based target generation, and
  bounded `607A` increments rather than assuming profile acceleration settings
  control the cyclic position loop.
- Treat `6083` profile acceleration and `6084` profile deceleration as a
  profile-mode boundary, not direct CSP joystick tuning. Before changing them
  for Steam Deck latency, prove the active drive mode, active object values,
  whether the drive applies them in CSP, RTCore `607A` increment limits,
  quiet faults, and live release-stop timing in the same `test-session-id`.
- Treat `6081` profile velocity and `6086` motion profile type as a
  profile-mode boundary too, not a Steam Deck smoothness shortcut. Profile
  velocity or trapezoid/S-curve profile selection may affect PP/PV style motion,
  but the current joystick path is CSP `607A` over cyclic PDO. Do not change
  `6081`, `6086`, or profile-shape assumptions for joystick smoothness until
  same-session telemetry proves active mode display, active `6081`/`6086`
  values, drive CSP applicability, `607F` cap, host step/target lead, bounded
  `607A`, 606C actual-speed smoothness evidence, commanded-vs-actual stop
  telemetry, and three live release-stop samples.
- Treat `C01.01` speed loop gain, `C01.1B` speed loop behavior, and `60B1`
  speed/feedforward deviation as a speed-loop/feedforward smoothness boundary,
  not a Steam Deck joystick smoothing shortcut. They can help explain rough
  sweeps, target-vs-feedback lag, or torque/current spikes, but they must not
  be changed to mask command latency, seam jumps, unexpected HM35 commands, or
  missing stop proof. Before any speed-loop/feedforward smoothness change,
  prove active `C01.01`, active `C01.1B`, active `60B1`, 606C actual-speed
  smoothness evidence, `6064`/`607A`/`6062`/`60F4` correlation,
  target-vs-feedback delta, `6077` torque/current evidence, RTCore loop jitter,
  PDO freshness, smooth live sweep both directions, commanded-vs-actual stop
  telemetry, and three live release-stop samples under the same test-session-id.
- Treat `R22.01` current loop response, axis current low-pass filtering, and
  `C01.18` torque feedforward filtering as a current-loop/filter stability
  boundary, not a Steam Deck joystick smoothing shortcut. They can explain
  current spikes, buzz, lag, or over-smoothed response, but changing them before
  sweep telemetry proves the bottleneck can hide command latency or make the
  servo look smooth while torque/current evidence is unsafe. Before any
  current-loop/filter smoothness change, prove active `R22.01`, active current
  low-pass filter values, active `C01.18`, `6077` torque/current evidence,
  amp-fluctuation/current-spike evidence, quiet current faults, quiet
  following-error faults, target-vs-feedback delta, RTCore loop jitter, PDO
  freshness, smooth live sweep both directions, commanded-vs-actual stop
  telemetry, and three live release-stop samples under the same test-session-id.
- Interpolation-period objects such as `60C2` are not yet extracted into this
  page from the vendor manual. Treat them as a documentation/evidence gap, not
  as a tuning knob. Before any interpolation-period change is considered, add
  manual-backed notes, capture the active drive value, prove its relationship
  to SYNC0/EtherCAT cycle and RTCore command period, and keep live release-stop
  timing as the authority for whether the bottleneck is servo-side.
- Treat `60C2` interpolation-period active-value capture as a no-motion
  prerequisite, not a Steam Deck joystick latency shortcut. The active value
  must be captured through setup/status evidence outside the joystick command
  path, then reconciled with SYNC0 cycle, RTCore command period, WKC/DC health,
  PDO freshness, and three live release-stop samples before any interpolation
  period, send cadence, or coalescing change is considered.
- For Steam Deck joystick latency work, do not hide stalls by only watching
  command freshness. The live telemetry path now carries an `active_jog` flag
  from RTCore's ring sample flags, and the `active-jog-stall` pattern compares
  active target counts with feedback counts inside the same `test-session-id`.
  This keeps the command path fast while making "fresh joystick input but no
  servo movement" visible in NATS/UI evidence.
- Joystick release-to-stop should stay on the direct UDP command path:
  Steam Deck `steamdeck_joystick_intent_v1` -> motion-server UDP ->
  realtime-core `CmdJogV1`. The current dry-run evidence proves the Deck emits
  one forced-deadman neutral release packet and motion-server forwards the
  release zero-hold immediately, with `0 us` input wait and no scheduled output
  tick. Do not add NATS, UI, file, SDO, or polling hops to the release control
  path.
- The live release-stop measurement gate passed through the CLI runner in
  session `05-latency-20260531T112324.956639000Z`; telemetry captured
  `joystick-release-stop-live-gate` with direct UDP required, single-owner and
  Steam Deck pause/simulate required,
  `release_stop_live_executor_available=false`,
  `release_stop_live_executor_phase=gate-only-no-live-servo-executor`,
  `release_stop_live_executor_armed=false`,
  `release_stop_live_executor_state=locked`, and
  `release_stop_live_measurement_ready=false`. The four required timing legs
  remain Deck release send, motion-server receive/forward, realtime-core zero
  command receive/apply, and first live servo commanded/actual stop sample.
  Keep this as a no-motion gate until a bench live-motion executor can
  publish those measurements for the same `test-session-id`.
- The live release-stop executor contract is now split into two additional
  no-motion proofs before any servo-moving measurement may exist. Session
  `05-latency-20260531T112332.987685000Z` published
  `joystick-release-stop-live-executor-contract-no-servo` with
  `release_stop_live_executor_available=false`,
  `release_stop_live_executor_phase=executor-contract-only-no-joystick-no-servo`,
  `release_stop_live_executor_state=contract-only`, direct UDP, single-owner,
  Steam Deck pause/simulate, and `release_stop_live_measurement_ready=false`.
  Session `05-latency-20260531T112340.811042000Z` published
  `joystick-release-stop-live-execute-request-blocked-no-servo`, proving that
  even when unlock and execute are represented as present, the scaffold remains
  no-motion with
  `release_stop_live_executor_phase=execute-request-blocked-no-live-servo-executor`,
  `release_stop_live_executor_state=execute-request-blocked`, and
  `release_stop_live_measurement_ready=false`. Do not treat an unlock or
  execute request as enough to change SYNC0 cadence, `607F`, active-jog
  target lead, or any drive-side latency knob.
- The release-stop executor preflight row now makes the quiet evidence checklist
  explicit before any future live executor owns the release packet. Session
  `05-latency-20260531T113416.248636000Z` published
  `joystick-release-stop-live-executor-preflight-no-servo` with
  `release_stop_live_preflight_required=true`,
  `release_stop_live_preflight_ready=false`,
  `release_stop_live_executor_phase=preflight-checklist-only-no-joystick-no-servo`,
  and `release_stop_live_owner_required` set to single-owner pause-or-simulate
  control. The required evidence is same-session test logs, telemetry API
  records, quiet Activation & Fault Log, quiet motion-server status, quiet
  realtime-core status, single-owner proof, Steam Deck paused or
  simulate-active, release dry-run pass, motion-server zero-hold dry-run pass,
  and RTCore commanded/actual stop telemetry. Servo-side tuning stays blocked
  while `release_stop_live_preflight_ready=false`.
- The release-stop measurement decision contract now defines what future live
  timing evidence must prove before it can influence drive-side tuning. Session
  `05-latency-20260531T114129.399691000Z` published
  `joystick-release-stop-live-measurement-decision-no-servo` with
  `release_stop_live_executor_phase=measurement-decision-contract-no-joystick-no-servo`,
  `release_stop_live_measurement_required_samples=3`,
  `release_stop_live_measurement_before_after_required=true`, and
  `release_stop_live_measurement_servo_side_unblock_allowed=false`. The
  decision criteria require all four timing legs, monotonic same-session
  timestamps, no stale/ignored/mismatched packets, no NATS/UI/file/SDO/polling
  control hop, no owner conflict, quiet faults, and comparison against
  Deck-side diagnostic echo plus network scheduling evidence. Servo-side tuning
  stays blocked until those criteria are satisfied by real live measurements.
- The release-stop timing schema contract now fixes the payload shape before any
  executor can publish live values. Session
  `05-latency-20260531T114732.757544000Z` published
  `joystick-release-stop-live-timing-schema-no-servo` with
  `release_stop_live_timing_schema_required=true`,
  `release_stop_live_timing_schema_ready=false`,
  `release_stop_live_timing_legs_complete=false`, and
  `release_stop_live_timing_legs_monotonic=false`. The required fields are
  `release_stop_live_deck_release_send_time_ns`,
  `release_stop_live_motion_server_receive_time_ns`,
  `release_stop_live_motion_server_forward_time_ns`,
  `release_stop_live_rtcore_zero_receive_time_ns`,
  `release_stop_live_rtcore_zero_apply_time_ns`,
  `release_stop_live_first_servo_stop_sample_time_ns`,
  `release_stop_live_release_to_motion_server_us`,
  `release_stop_live_motion_server_to_rtcore_us`,
  `release_stop_live_rtcore_to_servo_stop_us`,
  `release_stop_live_release_to_servo_stop_us`,
  `release_stop_live_commanded_actual_stop_sample`,
  `release_stop_live_timing_legs_complete`, and
  `release_stop_live_timing_legs_monotonic`. Do not treat an unpopulated
  schema as servo-side tuning readiness; SYNC0 cadence, `607F` max-speed, and
  active-jog target lead stay blocked until those fields contain monotonic
  same-session live measurements.
- The release-stop clock contract now prevents the future timing executor from
  mixing clock domains or stale status. Session
  `05-latency-20260531T120000.000000000Z` published
  `joystick-release-stop-live-clock-contract-no-servo` with
  `release_stop_live_executor_phase=clock-contract-no-joystick-no-servo`,
  `clock_domain=monotonic_same_session`, capture points
  `deck_release_send|motion_server_receive|motion_server_forward|rtcore_zero_receive|rtcore_zero_apply|first_servo_stop_sample`,
  `sequence_correlation=direct_udp_packet_sequence`, and forbidden timing
  sources
  `wall_clock_delta|stale_status_snapshot|nats_control|ui_control|file_control|sdo_control|polling_control`.
  The future live executor must correlate direct UDP packet sequence with RTCore
  cycle/sample evidence and the first commanded/actual stop sample after the
  zero command before any servo-side latency knob can be considered.
- The timing-math dry-run proves the release-stop schema can carry populated
  monotonic values without pretending they are live servo evidence. Session
  `05-latency-20260531T115826.037903000Z` published
  `joystick-release-stop-live-timing-math-no-servo` with
  `release_stop_live_executor_phase=timing-math-dry-run-no-joystick-no-servo`,
  `release_to_motion_server_us=120`,
  `motion_server_to_rtcore_us=80`, `rtcore_to_servo_stop_us=600`, and
  `release_to_servo_stop_us=900`. The same row keeps
  `release_stop_live_measurement_ready=false` and
  `release_stop_live_measurement_decision=synthetic-timing-math-only-live-servo-evidence-still-required`,
  so these synthetic values only validate duration arithmetic and telemetry
  preservation. They do not authorize SYNC0 cadence, `607F`, or active-jog
  target-lead changes.
- The release-stop sample plan now prevents one-off timing captures from being
  treated as tuning authority. Session
  `05-latency-20260531T120221.406169000Z` published
  `joystick-release-stop-live-sample-plan-no-servo` with
  `release_stop_live_executor_phase=sample-plan-no-joystick-no-servo`,
  `release_stop_live_measurement_required_samples=3`,
  `release_stop_live_measurement_before_after_required=true`, and
  `release_stop_live_measurement_ready=false`. The future live executor must
  collect exactly three active-to-release samples with monotonic timing legs,
  direct UDP sequence correlation, RTCore cycle/sample correlation, first
  commanded/actual stop sample, no owner conflict, quiet faults, and baseline
  plus post-measurement Deck network evidence before timing can be judged.
- The release-stop executor inputs contract now fixes the explicit live-motion
  endpoint inputs before any live packet sender can exist. Session
  `05-latency-20260531T120616.109872000Z` published
  `joystick-release-stop-live-executor-inputs-no-servo` with
  `release_stop_live_executor_phase=executor-inputs-contract-no-joystick-no-servo`,
  `release_stop_live_command_inputs_ready=false`,
  `release_stop_live_executor_available=false`, and
  `release_stop_live_measurement_ready=false`. The required inputs are
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_MEASUREMENT_UNLOCK`,
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_MEASUREMENT_EXECUTE`,
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_MEASUREMENT_SAMPLES`,
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_UDP_HOST`,
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_UDP_PORT`, and
  `ROBOT_V4_STEAMDECK_RELEASE_STOP_LIVE_MAX_STOP_US`. Keep SYNC0, `607F`, and
  active-jog lead blocked until those inputs, preflight readiness, sample-plan
  evidence, and real live timing samples exist together.
- The inputs-ready blocked proof now closes the loophole where complete-looking
  command inputs might be mistaken for an implemented executor. Session
  `05-latency-20260531T121054.610014000Z` published
  `joystick-release-stop-live-inputs-ready-blocked-no-servo` with unlock `1`,
  execute `1`, samples `3`, explicit UDP host/port, and max-stop budget
  represented as present. It still reports
  `release_stop_live_executor_available=false`,
  `release_stop_live_executor_state=inputs-ready-blocked`, and
  `release_stop_live_measurement_ready=false`; no servo-side latency knob can
  move until a dedicated live executor and real timing evidence exist.
- The preflight-ready blocked proof closes the next loophole: represented
  command inputs plus represented preflight readiness still do not become
  measurement authority. Session
  `05-latency-20260531T122549.623754000Z` published
  `joystick-release-stop-live-preflight-ready-blocked-no-servo` with
  `release_stop_live_preflight_ready=true`,
  `release_stop_live_command_inputs_ready=true`,
  `release_stop_live_executor_available=false`,
  `release_stop_live_executor_state=preflight-ready-blocked`, and
  `release_stop_live_measurement_ready=false`. A dedicated live executor must
  still send exactly three direct UDP active-to-release samples and publish
  monotonic timing legs before this evidence can unblock any servo-side tuning.
- The UDP loopback executor proof is the first packet-sending executor slice,
  but it is deliberately local-only. Session
  `05-latency-20260531T123133.314231000Z` published
  `joystick-release-stop-live-udp-loopback-executor-no-servo` after sending one
  active `127.0.0.1` UDP packet followed by one release
  `forced_deadman_zero_hold_udp` packet. It proves `seq=1` then `seq=2`,
  monotonic release-send to loopback-receive timing, and populated timing
  fields, while still reporting `release_stop_live_executor_available=false`,
  `release_stop_live_executor_state=loopback-only`, and
  `release_stop_live_measurement_ready=false` with
  `release_stop_live_measurement_decision=udp-loopback-sequence-only-live-servo-evidence-still-required`.
  Treat it as packet-order and timestamp-capture evidence only; it does not
  touch motion-server,
  realtime-core, EtherCAT, or servo hardware and cannot unblock SYNC0, `607F`,
  or active-jog target-lead tuning.
- The endpoint-authority proof then blocks the dangerous leap from loopback to
  a deployed motion-server endpoint. Session
  `05-latency-20260531T123653.288727000Z` published
  `joystick-release-stop-live-endpoint-authority-no-servo` with
  `loopback_endpoint_allowed=true`, `non_loopback_endpoint_allowed=false`,
  `release_stop_live_executor_state=endpoint-authority-blocked`,
  `release_stop_live_preflight_ready=false`, and
  `release_stop_live_measurement_ready=false`. A future non-loopback endpoint
  needs explicit UDP host/port, bench live-motion unlock/execute, single-owner
  pause-or-simulate proof, quiet faults, release zero-hold dry-run evidence,
  and a dedicated live executor before any packet can target motion-server.
- The motion-server dry-run dependency proof ties that endpoint boundary back
  to the already-passing motion-server row. Session
  `05-latency-20260531T124245.966073000Z` published
  `joystick-release-stop-live-motion-server-dry-run-dependency-no-servo` after
  reading retained NATS/JetStream evidence and confirming the motion-server zero-hold latency proof with
  dependency session `01-20260531T074635130Z-8cf500d0`,
  `release-zero-hold-latency` telemetry,
  `motion_server_release_bypasses_tick=true`, and direct Steam Deck UDP ->
  motion-server UDP -> RTCore path evidence. It still reports
  `release_stop_live_executor_state=dependency-ready-executor-blocked`,
  `release_stop_live_preflight_ready=false`, and
  `release_stop_live_measurement_ready=false`.
- The Deck/network dependency proof ties the live release-stop executor to the
  current Deck-side bottleneck evidence before any non-loopback sample is
  allowed. Session `05-latency-20260531T125138.016755000Z` published
  `joystick-release-stop-live-deck-network-dependency-no-servo` after reading
  retained NATS/JetStream evidence and confirming the Deck-side echo, Deck-side distribution, and
  Deck network scheduling proofs with sessions
  `05-20260531T085316443Z-0fadddaa`,
  `05-20260531T104713777Z-226ff22c`, and
  `05-20260531T104208181Z-19ffa595`. It carries forward
  `steamdeck_echo_kernel_timestamp_available=true`, echo mean/p95/max
  `111144.667/117742/117742 us`, zero sequence mismatches, `wlan0`, and
  `10/20` ping burst samples over `100 ms`, while still reporting
  `release_stop_live_executor_state=deck-network-dependency-ready-executor-blocked`,
  `release_stop_live_preflight_ready=false`, and
  `release_stop_live_measurement_ready=false`.
- The owner/simulate dependency proof ties live release-stop samples to the
  one-owner rule before any non-loopback packet can be sent. Session
  `05-latency-20260531T125146.828891000Z` published
  `joystick-release-stop-live-owner-simulate-dependency-no-servo` after reading
  retained NATS/JetStream evidence and confirming
  the realtime-core single-host motion-owner and Steam Deck local simulation proofs with sessions
  `04-20260530T212305033Z-9ee711c5` and
  `05-20260529T230726.156357000Z`. It carries forward single motion-owner
  lease evidence and proof that simulate mode blocks outbound joystick, HM35,
  telemetry, and arm/disarm UDP commands, while still reporting
  `release_stop_live_executor_state=owner-simulate-dependency-ready-executor-blocked`,
  `release_stop_live_preflight_ready=false`, and
  `release_stop_live_measurement_ready=false`.
- The A6-EC UI page and this markdown file now mirror this gate for operators.
  Source-contract UI-run session `00-20260531T084608602Z-bb71602d` passed with
  non-stale logs and session-scoped test telemetry after guarding both surfaces
  with `TestRobotV4A6ECLatencyTuningGateSourceContract`. The guard blocks
  SYNC0 cadence, `607F` max-speed, and active-jog target lead until Deck/
  network policy and release-stop telemetry for the same `test-session-id`
  prove servo-side tuning is justified.
- The first live no-servo diagnostic echo baseline should stay ahead of real
  motion tuning: it sends one zero-motion Steam Deck packet, drains stale UDP
  datagrams, uses a unique sequence, and requires motion-server latency buckets
  under the same `test-session-id` before changing SYNC0 cadence, `607F`, or
  host-side command limits.
- Earlier no-servo status snapshots showed the dominant delay as
  `jetson_input_wait_before_output_tick`, so the host bridge had to be
  instrumented before changing A6-EC drive parameters. Servo-side tuning such
  as SYNC0 cadence or `607F` should wait until packet-level diagnostics prove
  the host and UDP path are no longer spending most of the budget before the
  response reaches the Deck.
- The first host-side scheduling fix makes diagnostic echo packets with new
  sequences bypass repeated zero-hold coalescing. The local dry-run proved
  `input_wait_max_us=0` at a 10 Hz output cadence, and the deployed no-servo
  Steam Deck baseline first improved to about `98 ms` end-to-end. The latest
  session-scoped probe reported `153.815 ms` end-to-end while the diagnostic
  echo packet itself reported `94 us` receive-to-send, `0 us` input wait,
  `87 us` Jetson kernel UDP queue age, and `250 Hz`. It needed exactly one
  client receive attempt and ignored zero packets. The local client reported
  `steamdeck_echo_kernel_timestamp_available=false`, so its `0 us` client queue
  field is not proof that the client socket queue was empty. The NATS status
  sequence matched the echo sequence, but its `latest_receive_to_output_us` was
  `200.181 ms`, `200.087 ms` above the packet echo timing, so treat status
  latency as a later same-sequence output snapshot, not the one-packet
  diagnostic. The next required proof is `05. steamdeck deck-side live latency
  diagnostic echo no-servo real`: run the same zero-motion diagnostic from the
  actual Steam Deck/Linux runtime and require
  `steamdeck_echo_kernel_timestamp_available=true`, Deck-side UDP queue age,
  receive attempts, ignored packet counts, packet-level motion-server echo
  timing, Jetson UDP queue age, and status-vs-echo sequence/delta evidence in
  the same `test-session-id`. Keep SYNC0 cadence, `607F`, and active-jog target
  lead unchanged until that Deck-side client timing proof explains the remaining
  Deck-observed wait or fails with actionable runtime/build evidence. First
  live attempt `05-20260531T065606.226731000Z` reached the motion-server from
  the Deck at `192.168.50.120:49013` and received a diagnostic echo around
  `121.040 ms` end-to-end, but the deployed Deck binary was stale against the
  current report contract. After redeploy session
  `03-20260531T065914.464263000Z`, the refreshed UI/log/telemetry session
  `05-20260531T085316443Z-0fadddaa` passed with
  `steamdeck_echo_kernel_timestamp_available=true`,
  `steamdeck_echo_kernel_queue_us=71`,
  `motion_server_echo_receive_to_send_us=78`,
  `motion_server_echo_input_wait_us=0`,
  `motion_server_echo_kernel_queue_us=95`, one receive attempt, zero ignored
  packets, matched sequence `522702392`, and
  `steamdeck_latency_end_to_end_max_us=110503`. This says the remaining broad
  wait is not stale packets, client filtering, sequence mismatch, Jetson socket
  queueing, Deck socket queueing, or motion-server userspace echo work. Do not
  tune SYNC0, `607F`, or active-jog target lead from this evidence yet. The
  follow-up distribution row now passes on the current deployed stack in
  UI-run session `05-20260531T104713777Z-226ff22c` after hardening the verifier
  to wait for the final motion-server status sequence and to select the
  session's `latency-diagnostic-echo` record instead of heartbeat records.
  Telemetry seq `12505` captured three independent Deck-side zero-motion echo
  samples with end-to-end mean/p95/max `111144.667/117742/117742 us`, three
  receive attempts, zero ignored packets, zero sequence mismatches,
  `steamdeck_echo_kernel_timestamp_available=true`, Deck kernel queue max
  `107 us`, Jetson kernel queue `108 us`, motion-server receive-to-send
  `66 us`, input wait `0 us`, and matching status sequence. This
  keeps servo-side tuning behind a Deck/network scheduling investigation rather
  than a drive-parameter guess. Deck/network scheduling session
  `05-20260531T102330686Z-7cab336c` published telemetry seq `12384` through the
  UI API with route `192.168.50.120 dev wlan0 src 192.168.50.210 uid 1000`, all
  CPU governors as `powersave`, and ping `10/10` with `0%` loss and
  `min/avg/max/mdev = 99.700/112.046/126.666/9.530 ms`. It confirmed `wlan0`
  Wi-Fi power save is already off, SSID `colonise the moon_5G-2` is on
  `5805 MHz`, signal is `-39 dBm`, RX/TX bitrate is `780.0/866.7 MBit/s`, and
  noninteractive `sudo -n` is unavailable. Follow-up burst session
  `05-20260531T104208181Z-19ffa595` published telemetry seq `12465` with `20/20`
  pings, `0%` loss, `min/avg/max/mdev = 65.053/99.532/119.959/16.346 ms`,
  `20/20` samples over `50 ms`, `10/20` samples over `100 ms`, and `0/20` over
  `150 ms`. That directly matches the diagnostic
  echo wait, so the next performance target is an explicit bench live-motion
  Deck CPU/network policy experiment, such as performance governor or
  wired/alternate Wi-Fi path. Servo-side SYNC0, `607F`, and active-jog target
  lead remain blocked behind a faster Deck-to-motion-server path.
- The no-motion Deck policy gate now makes that block explicit in session
  telemetry before any policy experiment may run. Latest UI session
  `05-20260531T104350274Z-02236242` passed with a fresh log and
  ROBOT_V4_TEST records; seq `12480` on `robot/v4/test.05.telemetry` is the
  `deck-policy-experiment-gate` record. The raw test verifier requires the
  record to name the Deck CPU/network policy experiment as the next latency
  target, summarize the Deck-side evidence, and set
  `deck_servo_side_tuning_blocked=true` for SYNC0 cadence, `607F` max-speed,
  and active-jog target lead. The telemetry API now preserves those custom
  fields on the normalized record as well; the UI API verified seq `12480`
  exposes the next target, evidence summary,
  `servo_blocked=true`, and blocked changes.
- The default no-motion policy experiment scaffold passed through the UI runner
  in session `05-20260531T104350295Z-5939ab85` with a non-stale log and
  ROBOT_V4_TEST records. API seq `12481` captured
  `deck-policy-experiment-no-servo` with unlock/apply/rollback absent, exact
  apply/rollback/execute env vars, baseline/changed sample env vars,
  `deck_policy_experiment_apply_present=false`,
  `deck_policy_experiment_rollback_present=false`,
  `deck_policy_experiment_execute_present=false`,
  `deck_policy_experiment_command_inputs_ready=false`,
  `deck_policy_experiment_executor_available=false`,
  `deck_policy_experiment_executor_phase=scaffold-only-no-apply-measure-rollback`,
  `deck_policy_experiment_executor_armed=false`,
  `deck_policy_experiment_executor_state=locked`,
  `deck_policy_experiment_baseline_samples_valid=false`,
  `deck_policy_experiment_changed_samples_valid=false`,
  `deck_policy_experiment_ready=false`, missing inputs for unlock/apply/
  rollback/baseline samples/changed samples, before/after required, rollback
  required,
  `deck_policy_experiment_locked_by_default=true`,
  `deck_policy_experiment_no_motion=true`,
  `deck_servo_side_tuning_blocked=true`, and blocked changes SYNC0 cadence,
  `607F` max-speed, and active-jog target lead. Latest CLI-run session
  `05-latency-20260531T110129.441153000Z` refreshed this scaffold and proves
  the apply/measure/rollback executor is explicitly unavailable in this row
  until a future dedicated executor exists. This is still a locked no-motion
  scaffold; the actual performance experiment must be a separately authorized,
  reversible Deck CPU/network policy comparison with rollback evidence.
- The Deck CPU/network policy experiment now requires a no-motion dry-run plan
  before any executor exists. That planner must publish
  `deck-policy-experiment-plan-no-servo` telemetry with the ordered sequence:
  pause or simulate the Steam Deck owner, collect baseline Deck-side latency
  distribution samples plus Deck network-scheduling ping bursts, apply one Deck
  CPU/network policy change, collect changed Deck-side distribution plus
  network-scheduling samples, rollback, verify rollback with no-motion samples,
  and compare the same test-session-id logs plus telemetry. It must carry
  forward the current Deck echo and ping-burst evidence and fail if
  `ROBOT_V4_STEAMDECK_POLICY_EXPERIMENT_EXECUTE=1` is present, because the
  planner cannot execute policy changes. Servo-side SYNC0, `607F`, and
  active-jog target lead stay blocked until this reversible Deck/network
  comparison proves the bottleneck moved off the Deck/network path.
- The dedicated Deck CPU/network policy executor contract now exists as a
  no-motion telemetry row before servo-side tuning. Latest CLI-run session
  `05-latency-20260531T110526.424312000Z` published
  `deck-policy-experiment-executor-contract-no-servo` with
  `deck_policy_experiment_executor_available=false`,
  `deck_policy_experiment_executor_phase=executor-contract-only-no-apply-measure-rollback`,
  `deck_policy_experiment_executor_armed=false`, and
  `deck_policy_experiment_executor_state=contract-only`. The future dedicated
  executor must pause or simulate the Steam Deck owner, collect baseline
  Deck-side latency distribution plus Deck network-scheduling burst evidence,
  apply exactly one configured Deck CPU/network policy command, collect
  changed evidence, rollback, verify rollback with no-motion samples, and
  compare session-scoped telemetry before any SYNC0, `607F`, or active-jog
  target-lead tuning.
- A fail-closed execute-request row now proves the scaffold cannot silently act
  as the policy executor. CLI-run session
  `05-latency-20260531T110932.924818000Z` published
  `deck-policy-experiment-execute-request-blocked-no-servo` with synthetic
  unlock/apply/rollback/execute/sample inputs present,
  `deck_policy_experiment_command_inputs_ready=true`,
  `deck_policy_experiment_execute_present=true`,
  `deck_policy_experiment_executor_available=false`,
  `deck_policy_experiment_executor_phase=execute-request-blocked-no-apply-measure-rollback`,
  `deck_policy_experiment_executor_armed=false`,
  `deck_policy_experiment_executor_state=execute-request-blocked`, and
  `deck_policy_experiment_ready=false`. It runs no apply/rollback commands and
  changes no Deck, motion-server, realtime-core, EtherCAT, drive object, or
  servo state.
- A no-motion comparison-decision contract now defines what future before/
  after/rollback evidence must prove before Deck/network policy can be treated
  as improved. CLI-run session `05-latency-20260531T111521.709037000Z`
  published `deck-policy-experiment-comparison-decision-no-servo` requiring
  baseline Deck-side latency distribution, baseline network-scheduling burst,
  changed distribution and burst samples, rollback, rollback verification,
  lower changed Deck-side p95, reduced ping burst over-100ms count, no packet
  loss, no stale or mismatched diagnostic echo packets, low Deck/Jetson queue
  ages, no joystick packets, and no servo motion.
- Servo-side latency work now also requires a no-motion
  `servo-side-latency-pre-tuning-gate` record before any SYNC0 cadence,
  `607F`, or active-jog target-lead change. That gate must keep the control
  path as Steam Deck UDP to motion-server UDP to realtime-core `CmdJogV1` to
  A6-EC CSP PDOs, and require same-session evidence for Deck/network policy,
  release-stop timing legs, EtherCAT cycle period, WKC/DC health, SYNC0 error
  state, RTCore loop jitter, PDO freshness, current `607F` value, host
  max-step-per-cycle, active-jog target lead, and commanded-vs-actual stop
  telemetry. The point is to use the A6-EC knobs only after the telemetry says
  the bottleneck is actually servo-side. Latest UI session
  `05-20260531T103343789Z-02e5afcf` passed with a fresh log, and telemetry seq
  `12429` kept `servo_side_latency_evidence_ready=false`,
  `servo_side_latency_executor_armed=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The follow-on RTCore/EtherCAT evidence audit must publish
  `rtcore-ethercat-evidence-audit` telemetry from live realtime-core NATS
  status before servo-side tuning. It should prove no motion, seven operational
  slaves, WKC actual/expected health, and quiet axis masks, then explicitly list
  any missing evidence for SYNC0/EtherCAT cycle, SYNC0 error state, RTCore loop
  jitter, PDO freshness, `607F`, host step clamp, active-jog lead, and
  commanded-vs-actual stop telemetry. Missing fields are not tuning readiness;
  they are the next RTCore instrumentation target.
- Latest deployed audit evidence now publishes SYNC0/EtherCAT cycle, SYNC0
  error state, RTCore loop jitter, PDO freshness, current `607F`, host
  max-step-per-cycle, active-jog target-lead, and commanded-vs-actual stop
  fields. Session `05-20260531T101134413Z-23f71aa3` reports WKC `21/21`,
  commanded zero-stop aligned to feedback, and
  `rtcore_ethercat_evidence_ready=true`.
- The follow-on WKC/DC health audit must keep WKC separate from the broader
  evidence inventory: publish WKC actual/expected/delta, surplus/deficit,
  responding/online/operational slaves, link state, SYNC0/DC state, PDO
  freshness, and the explicit blocker. A WKC mismatch should be a tuning block,
  not a reason to guess at `607F` or active-jog lead.
- Latest WKC/DC audit evidence reports WKC `21/21` with delta `0` while SYNC0
  is configured at `1000 us` and PDO freshness is true. The same record exposes
  the expected-WKC policy as `domain_expected_wkc=kLiveAxisCount*3_rw_domain`
  (`3` per axis) and the registered PDO shape as `13` entries per axis (`7`
  RX, `6` TX, `91` total). WKC/DC is now evidence-ready; continue to the
  Deck/network policy and live release-stop timing gates before changing SYNC0
  cadence, `607F`, or active-jog lead.
- The servo-doc latency knob map is now executable no-motion evidence. CLI-run
  session `05-latency-20260531T125518.639361000Z` published
  `servo-doc-latency-knob-map-no-servo` after reading this page and retained NATS/JetStream evidence. It names the fastest path as
  `steamdeck_joystick_intent_v1` UDP -> motion-server UDP -> realtime-core
  `CmdJogV1` -> A6-EC CSP PDO, keeps SDO reads and polling out of the
  latency-critical joystick path, and records the servo-doc candidates as SYNC0
  cadence, `607F` max-speed, host max-step-per-cycle, and active-jog target
  lead as one coordinated CSP latency envelope. It treats `6083`/`6084`
  profile acceleration/deceleration as audit-only context, keeps
  `servo_side_latency_executor_state=servo-doc-map-only-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true` until Deck/network before/after
  evidence plus three live release-stop timing samples prove the bottleneck is
  servo-side.
- The interpolation-period evidence gap is now explicit no-motion telemetry:
  `60C2`/interpolation-period settings are blocked until vendor-manual text is
  extracted into this page, the active drive value is captured, and the value is
  correlated with SYNC0/EtherCAT cycle, RTCore command period, PDO freshness,
  WKC/DC health, and live release-stop timing. This prevents treating
  interpolation-period guesses as a shortcut around the faster proven UDP/PDO
  path.
  CLI-run session `05-latency-20260531T130205.610247000Z` published
  `servo-doc-interpolation-period-evidence-gap-no-servo` with
  `60C2_manual_text_extracted=false`,
  `active_60C2_value_captured=false`,
  `servo_side_latency_executor_state=interpolation-period-evidence-gap-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `60FF` target-velocity mode boundary is also explicit no-motion evidence:
  `60FF` is PDO-mapped, but Steam Deck joystick release-stop remains on the CSP
  `607A`/`CmdJogV1` path until same-session evidence proves a target-velocity
  path has known mode semantics, bounded RTCore command behavior, quiet faults,
  and better live release-stop timing. This keeps a tempting PDO field from
  becoming an unmeasured control-path fork.
  CLI-run session `05-latency-20260531T130551.296304000Z` published
  `servo-doc-target-velocity-mode-boundary-no-servo` with
  `60FF_pdo_mapped=true`, `target_velocity_release_stop_allowed=false`,
  `servo_side_latency_executor_state=target-velocity-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6071` target-torque mode boundary is explicit no-motion evidence too:
  `6071` is PDO-mapped, but target torque is blocked as a joystick latency path
  until active mode, torque limits, RTCore safety envelope, quiet faults, and
  live release-stop timing prove it is safe and faster than CSP `607A`/
  `CmdJogV1`. Keep `6077` torque feedback as diagnostic evidence, not a
  command input.
  CLI-run session `05-latency-20260531T130940.506163000Z` published
  `servo-doc-target-torque-mode-boundary-no-servo` with
  `6071_pdo_mapped=true`, `target_torque_release_stop_allowed=false`,
  `torque_feedback_6077_diagnostic_only=true`,
  `servo_side_latency_executor_state=target-torque-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `60B8/60B9/60BA/60BC` touch-probe timing boundary is also no-motion
  evidence: touch-probe PDO fields may become useful diagnostic timing evidence
  for edge correlation, but they are blocked from the joystick release-stop
  control path until manual-backed edge semantics, active `60B8`
  configuration, monotonic same-session timestamp correlation, quiet faults,
  and live release-stop timing prove they help without adding polling/SDO or
  bypassing direct UDP/CSP `607A` stop behavior.
  CLI-run session `05-latency-20260531T131857.016801000Z` published
  `servo-doc-touch-probe-timing-boundary-no-servo` with
  `touch_probe_pdo_mapped=true`,
  `touch_probe_release_stop_control_path_allowed=false`,
  `touch_probe_diagnostic_only=true`,
  `servo_side_latency_executor_state=touch-probe-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The TxPDO status diagnostic boundary is no-motion evidence too: `603F`,
  `6041`, `6061`, `6064`, `6077`, and `60FD` are already in the fast cyclic
  feedback surface, so release-stop and joystick-latency work should use them
  for same-session diagnosis instead of adding control-path SDO reads or
  polling. They remain diagnostic-only until live release-stop timing proves the
  actual bottleneck and same-session records can tie commanded stop, actual
  position, torque feedback, mode/status, fault code, and digital inputs
  together.
  This is the `603F`/`6041`/`6061`/`6064`/`6077`/`60FD` status diagnostic boundary.
  CLI-run session `05-latency-20260531T132411.736453000Z` published
  `servo-doc-status-diagnostics-boundary-no-servo` with
  `status_diagnostics_pdo_feedback=true`,
  `status_diagnostics_control_path_allowed=false`,
  `status_diagnostics_diagnostic_only=true`,
  `servo_side_latency_executor_state=status-diagnostics-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6083/6084` profile accel/decel boundary is no-motion evidence:
  profile acceleration/deceleration values are audit context until same-session
  evidence proves the active mode, active values, CSP applicability, RTCore
  `607A` increment envelope, quiet faults, and live release-stop timing. They
  are not the primary Steam Deck joystick-to-servo latency knob while the
  direct UDP/CSP path still lacks three live release-stop timing samples.
  This is the `6083`/`6084` profile accel/decel boundary.
  CLI-run session `05-latency-20260531T132855.650086000Z` published
  `servo-doc-profile-accel-decel-boundary-no-servo` with
  `profile_accel_decel_audit_only=true`,
  `profile_accel_decel_csp_tuning_allowed=false`,
  `servo_side_latency_executor_state=profile-accel-decel-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `607F` max-speed boundary is no-motion evidence: `607F` is a useful
  drive-side cap, but raising it alone is blocked until the active value,
  RTCore host step clamp, active-jog target lead, `607A` increment envelope,
  WKC/DC health, PDO freshness, quiet faults, and three live release-stop
  timing samples prove the bottleneck has moved to the drive cap.
  This is the `607F` max-speed boundary.
  CLI-run session `05-latency-20260531T133258.420069000Z` published
  `servo-doc-607f-max-speed-boundary-no-servo` with
  `607F_max_speed_change_allowed=false`,
  `servo_side_latency_executor_state=607f-max-speed-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The host-step/active-jog lead boundary is no-motion evidence: RTCore
  max-step-per-cycle and active-jog target lead are command-generation knobs,
  not standalone joystick latency fixes. They remain blocked until same-session
  evidence proves the active values, `607A` increment envelope, seam guards,
  `607F` cap, WKC/DC health, PDO freshness, quiet faults, and three live
  release-stop timing samples.
  This is the host max-step-per-cycle and active-jog target lead boundary.
  CLI-run session `05-latency-20260531T133825.711236000Z` published
  `servo-doc-host-step-active-jog-lead-boundary-no-servo` with
  `host_step_active_jog_lead_change_allowed=false`,
  `servo_side_latency_executor_state=host-step-active-jog-lead-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The SYNC0 cadence boundary is no-motion evidence: the active `1000 us` cycle
  is currently valid and WKC/DC healthy, but changing cadence is blocked until
  same-session evidence proves the new cycle is a `250 us` multiple with clean
  WKC/DC, PDO freshness, RTCore jitter, quiet faults, and three live
  release-stop timing samples.
  This is the SYNC0 cadence boundary.
  CLI-run session `05-latency-20260531T134230.579079000Z` published
  `servo-doc-sync0-cadence-boundary-no-servo` with
  `sync0_cadence_change_allowed=false`,
  `servo_side_latency_executor_state=sync0-cadence-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The minimal PDO control-path boundary is no-motion evidence: active `1C12`/
  `1C13` assignment changes, extra cyclic telemetry objects, SDO reads, and
  polling stay blocked for the joystick release-stop path until registered PDO
  shape, WKC/DC health, PDO freshness, no-SDO/no-polling path evidence, and
  three live release-stop timing samples prove a mapping change is needed.
  This is the minimal PDO control-path boundary.
  CLI-run session `05-latency-20260531T134825.902549000Z` published
  `servo-doc-minimal-pdo-control-path-boundary-no-servo` with
  `minimal_pdo_control_path_change_allowed=false`,
  `servo_side_latency_executor_state=minimal-pdo-control-path-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `607A` increment/seam boundary is no-motion evidence: release-stop target
  generation must stay folded to the live `6064` shaft turn and inside bounded
  target-step/seam guards until same-session commanded-vs-actual stop telemetry,
  quiet following-error/excessive-increment evidence, and three live
  release-stop timing samples prove a larger target envelope is needed.
  This is the `607A` increment/seam boundary.
  CLI-run session `05-latency-20260531T135247.825407000Z` published
  `servo-doc-607a-increment-seam-boundary-no-servo` with
  `607A_increment_seam_change_allowed=false`,
  `servo_side_latency_executor_state=607a-increment-seam-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The current-feedback target-generation boundary is no-motion evidence:
  release-stop and active-jog tuning must compare active target counts with
  feedback counts under the same `test-session-id`, using `active_jog`,
  target-vs-feedback delta, bounded `607A` increments, seam guards, PDO
  freshness, quiet faults, commanded-vs-actual stop telemetry, and three live
  release-stop timing samples before changing target generation.
  This is the current-feedback target-generation boundary.
  CLI-run session `05-latency-20260531T135653.017504000Z` published
  `servo-doc-current-feedback-target-generation-boundary-no-servo` with
  `current_feedback_target_generation_change_allowed=false`,
  `servo_side_latency_executor_state=current-feedback-target-generation-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The bus-running disable/disarm stop boundary is no-motion evidence:
  release-stop must keep RTCore and EtherCAT alive, avoid master-stop/bus-reset
  behavior, publish commanded-vs-actual stop telemetry, and prove quiet
  synchronization-loss/error state under the same `test-session-id` before any
  stop-method change is allowed.
  This is the bus-running disable/disarm stop boundary.
  CLI-run session `05-latency-20260531T140106.140580000Z` published
  `servo-doc-bus-running-disable-disarm-stop-boundary-no-servo` with
  `bus_running_disable_disarm_stop_change_allowed=false`,
  `servo_side_latency_executor_state=bus-running-disable-disarm-stop-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The metrics side-channel observability boundary is no-motion evidence:
  RT loop Hz, jitter, WKC, lost-frame checks, PDO freshness, and status
  diagnostics stay on metrics/NATS/UI side channels and out of the Steam Deck
  release-stop control path until same-session telemetry proves a control-path
  change is necessary.
  This is the metrics side-channel observability boundary.
  CLI-run session `05-latency-20260531T140635.772931000Z` published
  `servo-doc-metrics-side-channel-boundary-no-servo` with
  `metrics_side_channel_change_allowed=false`,
  `servo_side_latency_executor_state=metrics-side-channel-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The RTCore-owned safety-gate boundary is no-motion evidence: fault,
  stale-command, home-valid, limit, WKC/DC, and unexpected-mode gates stay in
  RTCore, not UI or non-RT layers, so the direct release-stop path can fail
  fast without control-hop latency.
  This is the RTCore-owned safety-gate boundary.
  CLI-run session `05-latency-20260531T141106.868643000Z` published
  `servo-doc-rtcore-owned-safety-gate-boundary-no-servo` with
  `rtcore_owned_safety_gate_change_allowed=false`,
  `servo_side_latency_executor_state=rtcore-owned-safety-gate-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The HM35 RTCore-operation boundary is no-motion evidence: HM35 remains a
  first-class RTCore transaction, not a UI-side `607C` write, and active jog
  stays blocked while home is pending or active so joystick release-stop and
  sweep evidence cannot be polluted by home drift.
  This is the HM35 RTCore-operation boundary.
  CLI-run session `05-latency-20260531T141521.038166000Z` published
  `servo-doc-hm35-rtcore-operation-boundary-no-servo` with
  `hm35_rtcore_operation_change_allowed=false`,
  `servo_side_latency_executor_state=hm35-rtcore-operation-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The drive-native exact-ratio frame-change boundary is no-motion evidence:
  `0x2010:0x19/0x1A`, `6064`/`607A`, selected persisted-URDF ratios, host
  wiring/sign mapping, and drive-native frame changes stay blocked as runtime
  latency tweaks until explicit activation, matching V2 HM35, frame-specific
  telemetry, live release-stop timing, and safe-range regeneration prove the
  new frame is safe.
  This is the drive-native gear/sign frame migration boundary.
  CLI-run session `05-latency-20260531T142136.288041000Z` published
  `servo-doc-drive-native-frame-migration-boundary-no-servo` with
  `drive_native_frame_migration_change_allowed=false`,
  `servo_side_latency_executor_state=drive-native-frame-migration-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6091` electronic gearing command-unit boundary is no-motion evidence:
  `6091`, `6063`/`6064`, `60FC`/`6062`, `607A`, and joystick scale stay
  blocked as runtime latency tweaks until active command-unit conversion,
  exact persisted-URDF mechanical-ratio context, host wiring/sign mapping,
  bounded `607A`, direction-correct commanded-vs-actual telemetry, and three
  live release-stop samples prove the conversion is safe and useful.
  This is the `6091` electronic gearing command-unit boundary.
  CLI-run session `05-20260531T155444.341669000Z` published
  `servo-doc-6091-command-unit-boundary-no-servo` with
  `6091_command_unit_change_allowed=false`,
  `servo_side_latency_executor_state=6091-command-unit-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `606C` actual-speed smoothness boundary is no-motion evidence:
  `606C` stays diagnostic-only cyclic telemetry for detecting hidden stops,
  sign inversions, seam jumps, and non-smooth sweeps. It must not become
  polling, feedback-derived joystick control, or a new control hop until
  same-session `6064`/`607A`/`6062`/`60F4` correlation, active-jog flag
  correlation, motion-server receive-to-output latency, PDO freshness,
  monotonic samples, commanded-vs-actual stop telemetry, and three live
  release-stop samples prove a change is necessary.
  This is the `606C` actual-speed smoothness boundary.
  CLI-run session `05-20260531T160016.796392000Z` published
  `servo-doc-606c-actual-speed-smoothness-boundary-no-servo` with
  `606c_actual_speed_smoothness_change_allowed=false`,
  `servo_side_latency_executor_state=606c-actual-speed-smoothness-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6081` profile-velocity / `6086` motion-profile-type boundary is
  no-motion evidence: profile velocity, trapezoid/S-curve profile shape, and
  PP/PV profile assumptions stay blocked as CSP joystick smoothness fixes until
  active mode, active `6081`/`6086`, drive CSP applicability, `607F`, host
  target-step/active-jog lead, bounded `607A`, 606C actual-speed smoothness evidence,
  commanded-vs-actual stop telemetry, and three live release-stop samples prove
  they affect the measured bottleneck safely.
  This is the profile velocity/type boundary. Latest CLI-run session
  `05-20260531T160854.538020000Z` passed in 6.050s with a fresh non-stale log
  and ROBOT_V4_TEST telemetry records for the same test-session-id, publishing
  `servo-doc-profile-velocity-type-boundary-no-servo` with
  `profile_velocity_type_change_allowed=false`,
  `servo_side_latency_executor_state=profile-velocity-type-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The speed-loop/feedforward smoothness boundary is no-motion evidence:
  `C01.01`, `C01.1B`, and `60B1` can explain smoothness, lag, and
  current/torque behavior, but they must not become a Steam Deck joystick
  smoothing shortcut or hide direct UDP/CSP latency. This is the
  speed-loop/feedforward smoothness boundary. Keep changes blocked until active
  `C01.01` speed loop gain, active `C01.1B` speed loop behavior, active `60B1`
  speed/feedforward deviation, 606C actual-speed smoothness evidence,
  `6064`/`607A`/`6062`/`60F4` correlation, target-vs-feedback delta, `6077`
  torque/current evidence, RTCore loop jitter, PDO freshness, smooth live sweep
  both directions, commanded-vs-actual stop telemetry, and three live
  release-stop samples exist.
  CLI-run session `05-20260531T163148.217102000Z` passed in 6.431s with a
  fresh non-stale log and ROBOT_V4_TEST telemetry records for the same
  test-session-id, publishing
  `servo-doc-speed-loop-feedforward-smoothness-boundary-no-servo` with
  `speed_loop_feedforward_smoothness_change_allowed=false`,
  `servo_side_latency_executor_state=speed-loop-feedforward-smoothness-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The current-loop/filter stability boundary is no-motion evidence:
  `R22.01`, axis current low-pass filters, and `C01.18` can explain current
  spikes, buzz, lag, and over-smoothed response, but they must not become a
  Steam Deck joystick smoothing shortcut or hide direct UDP/CSP latency. This is
  the current-loop/filter stability boundary. Keep changes blocked until active
  `R22.01` current loop response, active axis current low-pass filter, active
  `C01.18` torque feedforward filter, `6077` torque/current evidence,
  amp-fluctuation/current-spike evidence, quiet current faults, quiet
  following-error faults, target-vs-feedback delta, RTCore loop jitter, PDO
  freshness, smooth live sweep both directions, commanded-vs-actual stop
  telemetry, and three live release-stop samples exist.
  CLI-run session `05-20260531T163647.692922000Z` passed in 6.822s with a
  fresh non-stale log and ROBOT_V4_TEST telemetry records for the same
  test-session-id, publishing
  `servo-doc-current-loop-filter-stability-boundary-no-servo` with
  `current_loop_filter_stability_change_allowed=false`,
  `servo_side_latency_executor_state=current-loop-filter-stability-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The SYNC0/RTCore command-period alignment boundary is no-motion evidence:
  active `1000 us` SYNC0, RTCore command period, the existing `250 us` multiple
  requirement, and the manual `125 us` work-mode note must be reconciled before
  changing timing or send cadence. This is the SYNC0/RTCore command-period
  alignment boundary. Keep changes blocked until active SYNC0 cycle, RTCore
  command period, 250 us multiple compliance, manual 125 us reconciliation,
  WKC/DC health, SYNC0 error state none, RTCore loop jitter, PDO freshness,
  direct UDP receive-to-output timing, commanded-vs-actual stop telemetry, and
  three live release-stop samples exist.
  CLI-run session `05-20260531T164212.992907000Z` passed in 6.614s with a
  fresh non-stale log and ROBOT_V4_TEST telemetry records for the same
  test-session-id, publishing
  `servo-doc-sync0-rtcore-command-period-alignment-boundary-no-servo` with
  `sync0_rtcore_command_period_alignment_change_allowed=false`,
  `servo_side_latency_executor_state=sync0-rtcore-command-period-alignment-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `60C2` interpolation active-value capture boundary is no-motion evidence:
  active `60C2` is still an evidence gap and must be captured before any
  interpolation-period, send-cadence, or packet-coalescing change. This is the
  `60C2` interpolation active-value capture boundary. Keep changes blocked
  until manual `60C2` text, active `60C2` value, SYNC0 cycle, RTCore command
  period, WKC/DC health, PDO freshness, direct UDP receive-to-output timing,
  commanded-vs-actual stop telemetry, and three live release-stop samples
  exist under the same test-session-id.
  CLI-run session `05-20260531T164736.079955000Z` passed in 6.333s with a
  fresh non-stale log and ROBOT_V4_TEST telemetry records for the same
  test-session-id, publishing
  `servo-doc-60c2-interpolation-active-value-capture-boundary-no-servo` with
  `interpolation_60c2_active_value_capture_change_allowed=false`,
  `servo_side_latency_executor_state=60c2-interpolation-active-value-capture-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The 607F/host-step/active-jog envelope reconciliation boundary is no-motion
  evidence: drive max-speed, host step clamp, and active-jog lead must stay a
  coordinated CSP latency envelope instead of independent speed knobs. This is
  the `607F`/host-step/active-jog envelope reconciliation boundary. Keep changes
  blocked until active `607F`, active host max-step-per-cycle, active-jog target
  lead, bounded `607A` increments, seam guard margins, target-vs-feedback
  delta, WKC/DC health, PDO freshness, quiet following/excessive-increment
  faults, smooth live sweep both directions, commanded-vs-actual stop telemetry,
  and three live release-stop samples exist.
  Pending telemetry must publish
  `servo-doc-607f-host-step-active-jog-envelope-reconciliation-boundary-no-servo` with
  `host_step_active_jog_envelope_reconciliation_change_allowed=false`,
  `servo_side_latency_executor_state=607f-host-step-active-jog-envelope-reconciliation-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6062`/`6064`/`607A`/`60F4` smooth-sweep correlation boundary is
  no-motion evidence: position reference, actual position, active CSP target,
  and position deviation must be interpreted together before a sweep is called
  smooth or a stop is called fast. Do not tune joystick scale, target lead,
  filters, feedforward, or seam limits from any single field until same-session
  telemetry proves monotonic `6062`/`607A` target progression, direction-correct
  `6064`, bounded `60F4` deviation, `606C` actual-speed sign/magnitude,
  no HM35/home hold, no home-relative or raw-seam hold, WKC/DC health, PDO
  freshness, RTCore loop jitter, motion-server receive-to-output latency under
  30 ms, smooth live sweep both directions, commanded-vs-actual stop telemetry,
  and three live release-stop samples.
  Pending telemetry must publish
  `servo-doc-6062-6064-607a-60f4-smooth-sweep-correlation-boundary-no-servo` with
  `smooth_sweep_correlation_change_allowed=false`,
  `servo_side_latency_executor_state=6062-6064-607a-60f4-smooth-sweep-correlation-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6061` CSP mode-display boundary is no-motion evidence: active mode
  display must prove the drive is actually in CSP before `607A`, `6062`,
  `6064`, `60F4`, `606C`, profile objects, reach windows, or feedforward
  objects are used to explain joystick smoothness or release-stop latency.
  Do not change mode selection, borrow PP/PV/PT status, or tune profile-mode
  objects as CSP joystick fixes until same-session telemetry proves active
  `6061` mode display is CSP, commanded `6060` mode selection is CSP,
  HM35 has restored CSP, no unexpected mode transition occurs during the sweep,
  PP/PV/PT shortcuts are blocked, WKC/DC health, PDO freshness, RTCore loop
  jitter, smooth live sweep both directions, commanded-vs-actual stop telemetry,
  and three live release-stop samples exist.
  Pending telemetry must publish
  `servo-doc-6061-csp-mode-display-boundary-no-servo` with
  `csp_mode_display_change_allowed=false`,
  `servo_side_latency_executor_state=6061-csp-mode-display-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `6041` operation-enabled statusword boundary is no-motion evidence:
  statusword state must prove the drive stayed operation-enabled and healthy
  before a joystick sweep pause is attributed to command latency, target math,
  profile tuning, or servo-side gains. Do not change controlword sequencing,
  quick-stop handling, mode-transition handling, or statusword interpretation
  until same-session telemetry proves active `6041` operation-enabled bits stable,
  quick-stop bit healthy, fault bit clear, warning/internal-limit bits quiet,
  no disabled/deactivation transition during sweep, active `6061` CSP,
  WKC/DC health, PDO freshness, RTCore loop jitter, smooth live sweep both
  directions, commanded-vs-actual stop telemetry, and three live release-stop
  samples exist.
  Pending telemetry must publish
  `servo-doc-6041-operation-enabled-statusword-boundary-no-servo` with
  `operation_enabled_statusword_change_allowed=false`,
  `servo_side_latency_executor_state=6041-operation-enabled-statusword-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `603F` fault-code attribution boundary is no-motion evidence: future
  sweep-pause, stall, jank, release-stop, `607F`, target-lead, seam-policy,
  profile-object, gain, or filter attribution must first prove the drive was
  not faulted. Keep `fault_code_603f_attribution_change_allowed=false`,
  `deck_servo_side_tuning_blocked=true`, and
  `servo_side_latency_executor_state=603f-fault-code-attribution-boundary-no-servo-change`
  until `servo-doc-603f-fault-code-attribution-boundary-no-servo` evidence
  shows active `603F` fault code quiet, `6041` fault bit clear,
  no drive fault transition during sweep, operation-enabled `6041` stable,
  active `6061`
  CSP, no following/excessive-increment/current/overtravel fault, WKC/DC
  health, PDO freshness, RTCore loop jitter, smooth live sweep both
  directions, commanded-vs-actual stop telemetry, and three live release-stop
  timing samples under the same `test-session-id`.

- The unexpected HM35 sweep boundary is no-motion evidence: a normal Steam Deck
  joystick range sweep must never issue, trigger, or mask an HM35/home
  transaction. Keep `unexpected_hm35_sweep_change_allowed=false`,
  `deck_servo_side_tuning_blocked=true`, and
  `servo_side_latency_executor_state=unexpected-hm35-sweep-boundary-no-servo-change`
  until `servo-doc-unexpected-hm35-sweep-boundary-no-servo` evidence shows
  HM35 command count zero during sweep, HM35 active false, home counts unchanged,
  home-valid mask stable, no `607C` write, no home-count relatch,
  no home-pending/home-active block while active jog is true, active `6061`
  CSP, operation-enabled `6041` stable, quiet `603F`, WKC/DC health,
  PDO freshness, RTCore loop jitter, motion-server receive-to-output latency under 30ms,
  smooth live sweep both directions, commanded-vs-actual stop telemetry, and
  three live release-stop timing samples under the same `test-session-id`.
- The controlword halt/quick-stop boundary is no-motion evidence: `6040`
  halt/quick-stop, `605A` quick-stop option, and `6085` quick-stop deceleration
  stay blocked as Steam Deck release-stop replacements until drive state and
  statusword transitions, controlword timing, quiet `603F`/`6041` status,
  commanded-vs-actual stop telemetry, and three live release-stop samples prove
  they are safer and faster than direct UDP CSP 607A zero/stop.
  This is the controlword halt/quick-stop boundary.
  CLI-run session `05-latency-20260531T142851.491234000Z` published
  `servo-doc-controlword-quick-stop-boundary-no-servo` with
  `controlword_quick_stop_change_allowed=false`,
  `servo_side_latency_executor_state=controlword-quick-stop-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The torque/current limit boundary is no-motion evidence: `6072`, `60E0`,
  `60E1`, and `6077` stay measured safety-envelope evidence rather than Steam
  Deck release-stop latency knobs until active torque/current limits,
  rated-current/rated-torque context, current-spike telemetry, quiet current
  and following-error status, commanded-vs-actual stop telemetry, and three
  live release-stop samples prove a limit change is necessary and safe.
  This is the torque/current limit boundary.
  CLI-run session `05-latency-20260531T143358.064021000Z` published
  `servo-doc-torque-current-limit-boundary-no-servo` with
  `torque_current_limit_change_allowed=false`,
  `servo_side_latency_executor_state=torque-current-limit-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The gain/filter loop tuning boundary is no-motion evidence: `C01.00`,
  `C01.01`, `R22.01`, axis current low-pass filtering, `C01.18`, and `C01.1B`
  stay stability-tuning evidence rather than Steam Deck release-stop latency
  knobs until active values, RTCore loop jitter, target-vs-feedback error,
  torque/current evidence, quiet current and following-error status,
  commanded-vs-actual stop telemetry, and three live release-stop samples prove
  a gain or filter change is necessary and safe.
  This is the gain/filter loop tuning boundary.
  CLI-run session `05-latency-20260531T143910.715830000Z` published
  `servo-doc-gain-filter-loop-tuning-boundary-no-servo` with
  `gain_filter_loop_tuning_change_allowed=false`,
  `servo_side_latency_executor_state=gain-filter-loop-tuning-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The PP setpoint/reference mode boundary is no-motion evidence: PP-mode
  `6040` bit 5 reference-interrupt behavior and bit 6 `607A` absolute versus
  incremental interpretation stay separate from the Steam Deck CSP release-stop
  path until active mode, controlword bit semantics, `607A` interpretation,
  seam/increment safety, quiet faults, commanded-vs-actual stop telemetry, and
  three live release-stop samples prove a PP-mode change is necessary and safe.
  This is the PP setpoint/reference mode boundary.
  CLI-run session `05-latency-20260531T144441.871966000Z` published
  `servo-doc-pp-setpoint-reference-boundary-no-servo` with
  `pp_setpoint_reference_change_allowed=false`,
  `servo_side_latency_executor_state=pp-setpoint-reference-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The SYNC0 multiple reconciliation boundary is no-motion evidence: the active
  `1000 us` cycle remains accepted, but the manual-backed `125 us` work-mode
  note and existing `250 us` evidence must be reconciled before changing SYNC0,
  RTCore command period, or interpolation-period assumptions.
  This is the SYNC0 multiple reconciliation boundary.
  CLI-run session `05-latency-20260531T145123.175323000Z` published
  `servo-doc-sync0-multiple-reconciliation-boundary-no-servo` with
  `sync0_multiple_reconciliation_change_allowed=false`,
  `servo_side_latency_executor_state=sync0-multiple-reconciliation-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The following-error/feedforward offset boundary is no-motion evidence:
  `6065`/`6066`, `60F4`, and CSP `60B0`/`60B1`/`60B2` can explain release-stop
  behavior, but they must not hide actual lag or fault risk by changing error
  windows or offsets before live stop samples exist. This is the
  following-error feedforward offset boundary.
  CLI-run session `05-latency-20260531T145754.936097000Z` published
  `servo-doc-following-error-feedforward-boundary-no-servo` with
  `following_error_feedforward_change_allowed=false`,
  `servo_side_latency_executor_state=following-error-feedforward-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The position/speed reach-window boundary is no-motion evidence: `6067`/
  `6068`, `606D`/`606E`, `606F`/`6070`, and `6041` bit 10 can explain PP/PV
  status, but they must not replace live CSP commanded-vs-actual stop timing.
  This is the position/speed reach-window boundary.
  CLI-run session `05-latency-20260531T150215.599740000Z` published
  `servo-doc-position-speed-reach-window-boundary-no-servo` with
  `position_speed_reach_window_change_allowed=false`,
  `servo_side_latency_executor_state=position-speed-reach-window-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The `607E` reference-polarity boundary is no-motion evidence: polarity bits
  can reverse position, speed, torque, or communication feedforward semantics,
  but they must not replace persisted-URDF ratio, host wiring/sign correctness,
  or live stop timing.
  This is the `607E` reference-polarity boundary.
  CLI-run session `05-latency-20260531T150752.111998000Z` published
  `servo-doc-607e-reference-polarity-boundary-no-servo` with
  `reference_polarity_607e_change_allowed=false`,
  `servo_side_latency_executor_state=607e-reference-polarity-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- The software-limit/overtravel boundary is no-motion evidence: `607D`,
  `607C`, `6041` bit 11, `60FD`, and overtravel alarm/fault codes can explain
  limit behavior, but they must not replace host safe-range margins or live CSP
  commanded-vs-actual stop timing. This is the software-limit overtravel
  boundary.
  CLI-run session `05-latency-20260531T151249.066892000Z` published
  `servo-doc-software-limit-overtravel-boundary-no-servo` with
  `software_limit_overtravel_change_allowed=false`,
  `servo_side_latency_executor_state=software-limit-overtravel-boundary-no-servo-change`,
  `servo_side_latency_evidence_ready=false`, and
  `deck_servo_side_tuning_blocked=true`.
- A post-deploy WKC/DC audit can fail closed while the EtherCAT bus is still
  settling: session `05-20260531T100333215Z-d6b17eda` had seven
  responding/online slaves but only three operational slaves and WKC `9/14`.
  Session `05-20260531T101134405Z-1a8f0f49` settled to seven operational
  slaves and WKC `21/21`, so steady-state WKC decisions should use the settled
  sample.
- `0x603F`, `6041`, `6061`, `6064`, `6077`, and `60FD` are the core status fields for explaining whether a move failed because of drive state, feedback, torque/current, or digital inputs.
- Stopping the EtherCAT master can produce synchronization-loss alarms such as `ErC1.1`. For operator stops where the bus should stay healthy, prefer keeping RTCore running and disabling/disarming motion.
- Metrics should be observable without consuming the main command path where possible. GradientOS uses `/run/gradient-rt-motion/metrics.json` for RT loop Hz, jitter, WKC, and lost-frame style checks; `steamdeck/real/v4` already has NATS/UI telemetry and should keep the same separation of command and monitoring concerns.

## GradientOS Open Validation Items Relevant To steamdeck/real/v4

The GradientOS uncertainty list is a useful checklist for `steamdeck/real/v4` hardware validation:

- Label physical slave order and map EtherCAT positions to robot joints.
- Confirm the A6-EC identity tuple from `ethercat slaves -v`; GradientOS observed `VendorId=0x00400000`, `ProductCode=0x00000715`, and SII revision `0x00002EF8` on one unit, while CoE identity revision differed.
- Confirm whether `6064`/`607A` are raw encoder counts or scaled by electronic gearing on the tested drive configuration.
- Confirm counts per revolution and sign per axis with real hardware.
- Validate fault reset behavior beyond the standard DS402 `0x0080` pulse.
- Decide which safety checks belong in RTCore rather than UI or non-RT layers. For `steamdeck/real/v4`, fault, stale-command, home-valid, limit, WKC/DC, and unexpected-mode gates should remain RTCore-owned.

## steamdeck/real/v4 Implications

- `steamdeck/real/v4` should keep HM35 as a first-class RTCore operation, not as a UI-side `607C` write.
- The RTCore HM35 transaction should continue to require explicit arm, block jog motion, switch to HM mode, run method `35`, wait for valid statusword bits, restore CSP, and latch home counts.
- Home-relative feedback should keep publishing both raw `axis_pos_counts` and `hm35_home_position_counts`.
- Any future native position command should specify its frame: raw multi-turn truth, `6064/607A` command frame, or home-relative joint frame.
- Seam and wrap handling should stay explicit. A shortest angular delta is not enough if the drive interprets an absolute `607A` sequence across the `0/RM` seam as a long move.
- Robot v4 now reconciles drive-native exact ratios only from the selected
  persisted URDF during explicit V2 Activate. Any changed activation identity
  requires its own matching V2 HM35 profile; a manifest, environment value, or
  host-side fallback cannot substitute for that proof.
- `steamdeck/real/v4/ui/a6ec` should stay an operator-facing summary only. It may explain checks and failure modes, but authority for arming, homing, target limiting, and failure handling belongs in RTCore.
