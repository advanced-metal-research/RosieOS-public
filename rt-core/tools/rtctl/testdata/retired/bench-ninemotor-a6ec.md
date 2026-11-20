# bench-relay / bench-servo A6-EC configuration derivation

## Qualification boundary

This configuration is derived from the superseded v4 runtime's deployed state and
sources. It has not been used to move any axis and does not authorize motion.
Using it requires the bench qualification in `docs/BENCH_TESTING.md:39-90`.
Starting the normal bus can write startup SDOs. The operator must decide the
tracking-mode change **before first bus ownership**, with motion inhibited.
J7-J9 need qualified homing and measured motion limits before motion. A successful
compile or matching inventory is not hardware qualification.

## Coordinate evidence mode and anchor upgrade

J1..J6 use `single_turn_absolute`: drive-side gearing, software ratio 1 and
travel strictly below 2*pi rad. Startup verification binds the encoder turn;
subsequent PDO tracking preserves the coordinate through mailbox absence.
J7..J9 retain `multi_turn` standing proof until their external travel is declared.
For both modes, the drive fault bit or a nonzero 0x603F makes readiness `faulted`
without itself clearing coordinates or Home. Encoder error `0x7305` additionally
invalidates coordinates and Home, per GradientOS
`docs/resources/a6ec_manual_codes.md` (A6-EC fault table).

`fixedpdo-e24dea59.log` is a fixed-PDO/mailbox measurement source under the old
`multi_turn` digests (`aa8468c3...` / `8911771d...`), not startup proof for this
mode. The review reports a bench run at `8e76f404`, configuration digest
`5b4510f497957865a769c8893f8d399cb33a189f06cb278b0c82498e9aa9e380`: J1..J6
verified on attempt 1, all nine axes were valid, and no faults occurred through
a mailbox stall. This is the round-1 bench result, not validation of later fixes.

Persisted anchors from the old digest are refused with `ErrIdentity`. One Home
is required after this upgrade before saving anchors under the new digest.

## Robot definition provenance

The `a6ec` robot profile selects fixed TxPDO 0x1B04 at 1 kHz, matching the bench
layout. Owner decision 2026-09-15: drive velocity 0x606C and following error
0x60F4 are cyclic signed 32-bit diagnostics; drive DI 0x60FD is read by mailbox
for display (`di_bits=0`, `di_valid=false` in cyclic status). The daemon-owned
PREOP write and automatic IgH rescan were accepted on nine AS715N drives on
2026-09-15, all reaching OP with zero unavailable readbacks; see
`docs/evidence/bench-ninemotor-2026-09-15/`. Select `a6ec_1b02` in the pinned
robot definition when wired drive digital inputs must be cyclic. This preset
change preserves gearing, limits, Home policy and coordinate identities.

This cell describes the **1400 arm drives** and its mapped positioner axes.
Bare bench motors use `config/machines/bench/**`, including
`config/machines/bench/nine-motors-1to1.json`, whose 1/1 writes are unchanged; its
configuration digest reflects the corrected profile provenance.

This cell pins `robots/rosie_1400_v3.json` revision 3 with SHA-256
`b9a1282eab3699d60ca852044ce38d2dd7f191fc67b3d3888b1d77f9a177c35a`.
The [copied mechanical sources](../../docs/robots/rosie_1400/README.md) identify
J1..J3 HSS-40-100 gearboxes, J4 HRP130 (18:1), and J6 HRP85 (variant ratio
unconfirmed). The owner decision on 2026-09-15 assigns the deployed 100/1 J5
pair record and readback to the 1420. The 1400 J5 retains GradientOS's 27/2
belt-stage value, with current physical pulley teeth still unconfirmed.
See [J5 history](../../docs/robots/rosie_1400/J5-history.md).

The [A6-EC manual table](../../docs/drives/a6ec/a6ec_manual_chapter_11_parameter_list.md),
Homing Touch Probe Parameters (2010h/C10), specifies `0x2010:19` (C10.18,
numerator) and `0x2010:1A` (C10.19, denominator) as U16 with range **1-65535**,
modifiable at stop and effective immediately. Both **27** and **2** are valid.
The compiler emits these startup writes for J5 (slave 4), with 1,769,472 wire
counts/rev and HM35 `0x607C = 884736` counts. Software gearing remains 1.

This revision changes J5 native writes, scaling, machine and publication
identities. It takes effect only when this 1400 cell is deployed and a fresh
Home follows before motion; startup bus ownership writes the gear objects
before Home. Old anchors cannot qualify the new binding. The angular limits
remain at their existing values; the historical 100/1 count derivations below
are provenance, not current 1400 count values or physical qualification.
`TestRobotMigrationParity` compares against the legacy payload with only the
explicit J5 correction; `TestRobotJ5CompiledCells` checks all three model cases.
The compiled identities below reflect the current compiler and pinned model;
they supersede the older identity values previously shown in this note.

## Source locations and interpretation

All capture names below resolve under the committed evidence folder
`docs/evidence/bench-ninemotor-2026-09-13/`.
These are read-only inputs: `v4-status.json` (keys cited below, updated_at
`2026-09-13T08:49:24Z`), `readback.txt:2` (SDO/PDO capture
`2026-09-13T08:59:23+00:00`), `revpi-deploy.txt`, `hostcheck.json`,
and `inventory.json`. The later drive readback wins over unwritten pair settings
except where superseded by the model-specific J5 owner decision above.

`v4/` in the JSON and this note means the repository directory `robot/v4/`
(`../robot/v4/` relative to `rt-core/`).
For compact table citations:

- **C** = `v4/realtime-core/src/robot_v4_rtcore.cpp`.
- **E** = `v4/realtime-core/src/robot_v4_rtcore_ethercat_runtime.hpp`.
- **T** = `v4/realtime-core/src/robot_v4_rtcore_telemetry_status.hpp`.
- **B** = `v4/realtime-core/natsbridge/bridge_commands.go`.
- **P** = `config/drives/a6ec.json`, addressed by JSON key.
- **X** = `config/machines/a6ec-bench.example.json`, addressed by JSON key.
- **S** = `v4-status.json`, addressed by capture key and zero-based axis index.
- **R** = `readback.txt`, addressed by line.
- Unprefixed repository paths are relative to this `rt-core/` worktree.

`cited` means the value is present in that source; `derived` means the stated
arithmetic or policy follows from it. Neither means mechanically qualified.
`unverified` rows identify the measurement needed before hardware use.

The deployed `ROBOT_V4_ROBOT_AXES` string agrees with
`v4/manifest/bench-relay.json:129-140` on every index, name, URDF joint, kind, sign
and Cartesian participation flag: six arm axes, three external axes,
Cartesian mask 63 and home-all mask 511. All nine slaves were OP, but
`S.axis_enable_mask=0` and `S.ds402_enabled_axes=0`; OP is not motion readiness.
The final service fragment in `revpi-deploy.txt` sets
`ROBOT_V4_SIMULATION_MODE=1`, whereas `S.simulation_mode=false` and
`S.ethercat_live=true`. This discrepancy is preserved, not resolved by guessing:
the captured runtime and SDOs supply the live values here.

## Scaling, home, limits and motion policy

Let `G = vendor numerator / denominator`, `W = 131072 * G` counts per
output-shaft revolution, `K = W/(2*pi)` counts/rad, and `s = sign`.
The motor count constant is `v4/realtime-core/natsbridge/bridge.go:77`
and `C:56`; `B:1749-1782` applies the ratio once.
`P.position_semantics.drive_native_ratio_enabled=true` and
`position_semantics_source=drive_output_shaft` select output-shaft semantics.
Therefore `wire_counts_per_rev=W` and `wire_revs_per_axis_rev=1`.
`include/coordinate_model.hpp:63-68` multiplies these inputs exactly once.
Using both `W` and a second `G` would double the gearing.

Both v4 (`B:1770-1774`) and rt-core
(`include/coordinate_model.hpp:95-119`) use positive logical displacement
times `K*s` for the count displacement. The signs therefore transfer directly.
v4 writes `home + q*K*s`; rt-core writes `q*K*s - home_offset`.
The profile's `home_truth_sign=-1` makes the HM35 truth offset `-H`, with
`H=W/2`. The v4 offset formula is `C:1784-1804`;
`P.native_home.transaction[2]` writes `0x607C = W*1/2`, and transaction[3]
selects method 35. This is a coordinate reference operation, not proof that a
mechanism has reached a mechanically suitable home.

| Axes | G | W counts/rev | W/2 counts | Captured raw low + high + 1 | Cross-check |
|---|---:|---:|---:|---|---|
| J1,J2,J3 | 100/1 | 13107200 | 6553600 | 72818 + 13034381 + 1 = 13107200 | S.hm35_home_position_counts; R:60,135,210 |
| J5 (1400 definition) | 27/2 | 1769472 | 884736 | Historical capture used 100/1; not a current 1400 wrap measurement | Owner decision 2026-09-15; GradientOS gradient05 2945e3ff; J5-history.md |
| J4 | 18/1 | 2359296 | 1179648 | 13107 + 2346188 + 1 = 2359296 | S.hm35_home_position_counts[3]; R:285 |
| J6 | 10/1 | 1310720 | 655360 | 7282 + 1303437 + 1 = 1310720 | S.hm35_home_position_counts[5]; R:435 |
| J7,J8,J9 | 10/1 | 1310720 | 655360 | Captured raw band is 0/0; not a wrap measurement | R:505-506,580-581,655-656 and 510,585,660 |

For J7-J9, `S.pair_config_gear_numerator=[...,1,1,1]`,
denominators 1, observed numerator/denominator 0/0 and
`pair_config_ratio_reconciliation_written_axis_mask=0` do not describe the
drive SDO state. `v4/realtime-core/natsbridge/urdf_runtime_contract.go:167-177`
initializes unmapped contracts to 1/1;
`pair_config_runtime_v2.go:194-197` copies those SDO fractions, while
`pair_config.go:1023-1029` checks observed fractions only for the mapped mask.
A reconciliation that applied these requested v4 external settings would rewrite
10/1 to 1/1. rt-core must preserve the cited drive readback 10/1.
The external `S.hm35_home_position_counts=0` is absence of successful Home,
not evidence against the actual `0x607C=655360`.

Seam distance is `min(wrapped_position, W-wrapped_position)`, with wrapping
and clipping to `[0,W-1]` (`T:62-68`, `C:2738-2747`).
For J1-J6, the captured positions
`[6509175,6553611,6553590,1179628,6553604,655362]` reproduce
`[6509175,6553589,6553590,1179628,6553596,655358]` counts exactly.
For the external raw positions `[-2,30,-32]`, distances `[2,30,32]` fit
both the incorrect v4 W=131072 and readback-derived W=1310720, so those samples
alone cannot prove the external period.

The v4 safe band is an encoder-seam exclusion, **not mechanical travel**.
`C:77` defines 2 degrees; `C:2784-2810` computes
`L=W/180`, `U=W-1-W/180`, transforms both relative to `H` with `K*s`,
sorts them, then intersects with the pair's mechanical joint limits:

```text
band_min = min((L-H)/(K*s), (U-H)/(K*s))
band_max = max((L-H)/(K*s), (U-H)/(K*s))
limits.min = max(band_min, pair_config_lower_limit_rad[i])
limits.max = min(band_max, pair_config_upper_limit_rad[i])
```

`T:86-87` rounds L/U to the captured integer raw counts. The configuration
uses the source's unrounded L/U formula, preserving the intersection rather than
introducing a sub-count outward rounding error (notably J4's L=13107.2).
The resulting limits reproduce `S.safe_min_rad/safe_max_rad` at their six
decimal places. J2, J3, J5 and J6 retain their narrower pair limits. J7-J9
are unmapped (`C:2786-2787`); their zero bands and zero speed caps are
not transferable motion limits. The explicit example placeholders remain
unverified until the external mechanisms, rotary units, travel and speed are measured.

Rotary positions/lead/errors/tolerance are **rad**, velocity **rad/s** and
acceleration **rad/s^2**; timeouts are **ms**.
`tools/rtctl/compiler.go:194-218,553-566` and
`include/coordinate_model.hpp:28,63-68,103-114` define the conversion;
`docs/CONTRACTS.md:222-223` states logical rad/m and ms/ns timing units.
The compiler does not interpret rotary limits as revolutions or degrees.

v4 caps mapped speed at 0.2 rad/s (`C:61,2265,2659-2681`):
`Vcounts=floor(0.2*K)`. Conversion gives:

| Axes | Captured Vcounts/s | Vcounts * 2*pi/W rad/s | Nominal step counts at 1 ms |
|---|---:|---:|---:|
| J1,J2,J3,J5 | 417215 | 0.19999993575553368 | 417.215134 |
| J4 | 75098 | 0.1999980715427706 | 75.098724 |
| J6 | 41721 | 0.1999975389105526 | 41.721513 |

These are the floored representations of the same nominal 0.2 rad/s.
The JSON preserves the angular caps. J5's retained
`0.19999993575553368 rad/s` now corresponds to **56324.025 counts/s** at
27/2, rather than the captured 417215 counts/s at 100/1. Its retained
`0.011344640137963142 rad` lead corresponds to **3194.88 counts**.
`C:2995-2997` computes the unrounded nominal step `0.2*K*0.001`;
`S.axis_max_step_counts_per_cycle` agrees.
`C:64-75,2941-2948` clamps 20 cycles of requested step to the per-axis
angular lead. The `max_target_lead` values for J1-J6 carry those angular caps as
provenance only: in v4 the constant clamps requested lead, while rt-core applies
`max_target_lead` as a fault threshold on jog target-to-feedback error
(`engine/jog.hpp`, `FaultTargetLead`, recovery `rehome_required`). The v4 clamp
was never measured against feedback, so all six rows are unverified until the
tracking error at `max_velocity` and `jog_acceleration` is measured under load.
For J7-J9 the captured 254.862222 counts comes from the incorrect 1/1;
0.70 degrees at the actual 10/1 would be 2548.622222 counts. Neither is a
qualified stopping envelope; retain X's 0.01 rad placeholder pending measurement.

The cell field `drive_speed_limit_motor_rpm` defaults to 3000 motor rpm (owner decision 2026-09-15: A6M rated speed, A6M60-400H2B1-M17 / A6M80-1000H2B1-M17). Each cyclic 0x607F is `round(rpm / 60 * motor_encoder_counts_per_rev)` counts/s for both drive-native and software ratios; the compiler rejects values outside 1..4294967295 counts/s. One motor revolution is 131072 counts in the drive frame regardless of the electronic gear ratio, so a motor speed cap does not scale with the gear. One output-shaft revolution is `131072 * gear_numerator / gear_denominator` counts; with drive-native gearing, `wire_counts_per_rev` describes that output revolution and must not be used directly for the motor-speed conversion. The compiler passes explicit per-axis counts/s through `--drive-speed-limit-counts-per-s`; native admission checks the complete list and binds the resulting caps into machine identity before cyclic emission. The old `max_motor_rpm` retains its wire-frame semantics for one release with a compile warning; specifying both fields fails. Separately, the profile `maximum_physical_speed_counts_per_second` bounds hand-turn or backdrive motion during inhibited single-turn start-up verification: `ceil(speed * mailbox_skew_ns / 1000000000) + feedback_error_counts < half_turn_counts`. It is independent of powered 0x607F and remains subject to that half-turn check. The software `limits.max_velocity` planning and jog envelope is unchanged. Bench move/program observations enforce that axis limit and record the axis, `peak_observed_velocity_rad_s`, planned peak and `overshoot_fraction = max(0, observed_peak / planned_peak - 1)` in evidence and the summary; stationary plans report a null fraction.

The corrected cap is 50 motor rev/s * 131072 = 6553600 counts/s on every
bench-relay, bench, ffA and ffB axis. The previous v4-derived compiler cap of
417215 counts/s on the 100:1 axes was `417215 * 60 / 131072 = 190.98587036132812 motor rpm`,
or about 191 motor rpm, corresponding to roughly 0.2 rad/s at the output shaft.
Round 1 incorrectly multiplied the powered cap by the gear ratio; those values
are superseded by this table. The previous-cap column describes the bench-relay
legacy formula applied to the current definitions, including J5 27/2 and the configured J7..J9 values, rather than
claiming powered v4 operation on those axes.

| Axis | bench-relay gear | Previous v4-derived cap (counts/s) | Corrected cap, all four configurations (counts/s) | Motor cap (rpm) |
|---|---|---|---|---|
| J1 | 100/1 | 417215 | 6553600 | 3000 |
| J2 | 100/1 | 417215 | 6553600 | 3000 |
| J3 | 100/1 | 417215 | 6553600 | 3000 |
| J4 | 18/1 | 75099 | 6553600 | 3000 |
| J5 | 27/2 | 56324 | 6553600 | 3000 |
| J6 | 10/1 | 41722 | 6553600 | 3000 |
| J7 | 100/1 | 417215 | 6553600 | 3000 |
| J8 | 100/1 | 417215 | 6553600 | 3000 |
| J9 | 100/1 | 417215 | 6553600 | 3000 |

The [A6-EC Servo Drive User Guide](https://www.omc-stepperonline.com/download/A6-EC_series_servo_drive_manual.pdf), chapter 11 C03.27/C03.28, specifies 6000 rpm default positive/negative speed limits (the parameter range extends to 8000 rpm); section 12.7.1 specifies A6M rated/maximum motor speeds of 3000/6000 rpm. The compiler conservatively refuses any A6 motor cap above 6000 rpm and warns above 3000 rpm, including legacy values converted to physical motor rpm. The owner's 3000 rpm setting produces no rated-speed warning. Compilation, validation and launch report the warning; it does not authorize operation above rated speed.

v4's acceleration source `C:62-63,3000-3015` is 120 rpm/s = 4*pi rad/s^2
and immediately zeros on a zero request or reversal. rt-core's profile has a
0.5 rad/s^2 unverified bound, so this config keeps X's 0.5, requiring new
ramp/stopping qualification. No v4 value is treated as evidence for rt-core's
following-error or completion policy.

`require_home=true` on all nine retains the safety gate.
`include/control_policy.hpp:76-79` and `include/output_policy.hpp:123`
allow Home admission to be skipped when it is false.
`include/coordinate_model.hpp:79-91` separately requires valid scaling and
continuous source evidence. False is not an axis-disable switch.
The compiler rejects zero-width limits and nonpositive velocity, but accepts
true with the profile's HM35 procedure (`tools/rtctl/compiler.go:194-205`).
J7-J9 need homing before motion; the arm's v4 Home bits are also not imported.
`S.home_restore_anchor_counts=[7528077,6649397,5491485,1320189,7530552,705226,0,0,0]`
are persisted encoder anchors, distinct from HM35 offsets. No captured anchor
is injected into the new runtime. v4's restoration tolerance of 16 counts
(`S.home_restore_consistency_tolerance_counts`) does not qualify the current
profile's unverified 1-count anchor tolerance.

The unchanged profile also carries unverified coordinate acquisition/error and
physical-speed bounds (`P.coordinate_evidence`), jog arrest/quick-stop windows
(`P.jog`), and brake release/hold timing and gravity classification (`P.brake`).
Those inherited defaults require the profile's stated measurements on this cell;
the successful v4 Home signature does not qualify them for rt-core.

## Tracking mode, PDO mapping and host decisions

**Recommended tracking mode: 4; operator decision required before first bus
ownership.** All nine drives read mode 0 at `0x2000:08`
(R:53,128,203,278,353,428,503,578,653). The JSON explicitly writes 4 on every
axis, matching `P.startup_defaults.a6ec_encoder_position_tracking_mode`
and `P.startup_schema` (u16 at 8192:8); this is a hardware-affecting change.

The profile labels this CSP / absolute rotation mode and selects
`canonical_truth_source=encoder_multi_turn_counts`,
`absolute_pair_field=encoder_multi_turn_low` at `0x2040:21`,
wrapped `0x6064` feedback and continuous commands. Mode 4 is the
profile-selected tracking policy intended to support that continuous multi-turn
frame. The model refuses a wrapped sample without independent continuous truth
(`include/coordinate_model.hpp:79-91,141-167`); source loss is a coordinate fault,
not permission to guess a turn (`docs/CONTRACTS.md:126,149`).
The intended benefit is coherent continuous count truth across the raw seam and
disabled displacement, with reset/lost-state detection from `0x2040:43`.
Under v4's captured mode 0, the configured PDOs supplied only `0x6064`
and v4 used its seam-limited raw frame and saved Home anchors.

Neither the profile, the coordinate model nor CONTRACTS supplies a vendor
mode-number behavior specification or proves physical mode-4 retention. They
do **not** establish that mode 0 lacks the multi-turn registers. Equivalence of
mode 0 to the required contract, and mode 4's actual scale, continuity, validity
and retention, are unverified on both firmware revisions. Qualify these during
the inhibited bus/coordinate steps, including a reset and lost absolute state.
Do not silently keep mode 0 merely to avoid the decision, or infer qualification
from the existing startup default. Reference running direction stays explicitly
0 (R:54 plus 75 lines per axis); logical signs are applied in software.
All drives read CSP 8 and homing method 35; DS402 `0x6091:1/2=1/1` is not
the vendor rotation ratio (`R:57-62` and each axis block).

`bus_bringup.explicit_pdo_config=true` is required by the different Tx mapping.
R:69-87 (repeated every 75 lines) shows default RxPDO 0x1702 exactly matching
P.ethercat.rx_layout. Default TxPDO 0x1B02 has
0x603F,0x6041,0x6064,0x6077,0x6061,0x60B9,0x60BA,0x60BC,0x60FD.
P.ethercat.tx_layout instead ends with 0x60FD,0x2040:21,0x2040:43 and omits
the three touch-probe entries. The drives' acceptance of the remapped 0x1B02 is
**unverified until the inhibited bus step**; check all nine and both CoE revisions.
v4's `legacy_ecrt_slave_config_pdos` called that API with the seven Rx and
nine default Tx entries (`E:1024-1078`); it did not establish acceptance of
rt-core's continuous position/validity entries.
R:25 plus 75 lines per axis reports DC support, 64 bit.
P.ethercat.dc_assign_activate=768 (0x0300), matching `E:1085`;
DC support alone does not qualify synchronization.

Inventory binds to SII vendor 4194304 / product 1813 / revision 12024
(`R:19-22` plus 75 lines per axis), with serial 0 throughout.
CoE revisions are `[22033,22033,22033,22034,22034,22034,22033,22033,22034]`
(`R:63` plus 75 lines per axis).
The nine-axis golden is a synthetic read-only scenario using those identities,
the captured statuswords 38480/5712 and `S.axis_pos_counts`; it returns
`match` plus a CoE/SII note for every axis. It is not a new bench capture.
The supplied older `inventory.json` labeled slave 0 mismatched because it
compared CoE 22033 to SII 12024. Current
`tools/rtctl/inventory.go:245-247` retains the distinction.

Retain deployed CPU 0, choose example FIFO priority 90 below the deployed limit
95, and mark both unverified. The four-CPU count and absence of isolcpus are in
`revpi-deploy.txt#===cpu,===cmdline`, not a successful host qualification.
`hostcheck.json#checks` confirms PREEMPT_RT 6.12.56-revpi1-rpi-v8, the
eth0 master MAC, but has no configured rt-core CPU and reports missing isolation/
service prerequisites. Its IgH observation is `1.6.9 unknown` and fails the
exact-version check. Existing v4 affinity is not an rt-core IRQ/housekeeping/
latency measurement. Retain the example's 1000000 ns lateness bound pending
the prescribed host checks; the captured 878.575 us maximum loop jitter is not
a worst-case deadline qualification.

## Per-value derivation

Each non-note scalar configured in the JSON has a row below. SDO notes cannot
be inside `startup`: `tools/rtctl/render.go:121-142` requires each key to
be in the profile schema. They are sibling `startup_source/startup_unverified`
notes and individually cited here. Machine, axis, limits and runtime notes
compile; `TestNineMotorBenchNoteScope` checks that removing annotations leaves
argv, axis descriptors and machine identity unchanged, and rejects an in-startup
note. The frozen configuration digest intentionally includes non-bus metadata
(`tools/rtctl/compiler.go:363-373,435-446`); editing that prose changes this
configuration digest and the deployment identity, not effective machine semantics. Bus notes are elided
by `validateBusBringup`. No compiler defect or identity change was needed.

| Field | Value | Source / derivation / closure measurement | Status |
|---|---|---|---|
| `schema_version` | `1` | tools/rtctl/compiler.go:111 | cited |
| `backend` | `"live"` | v4-status.json#ethercat_live=true; target bench-servo | cited |
| `cycle_ns` | `1000000` | v4-status.json#ethercat_cycle_us=1000; 1000 us * 1000 ns/us | derived |
| `drive_speed_limit_motor_rpm` | `3000` | Owner decision 2026-09-15: 3000/60 * 131072 = 6553600 counts/s on every axis, independent of gear; one motor revolution is 131072 drive-frame counts. A6-EC manual ch. 11 C03.27/C03.28 and sec. 12.7.1: rated 3000 rpm, maximum 6000 rpm; warn above rated, refuse above maximum. The inhibited verification bound remains separate. | owner decision / derived |
| `bus_bringup.startup_passive_ms` | `0` | X.bus_bringup.startup_passive_ms; measure passive startup needed by A6-EC firmware on the bench (ms, 0..4294967295); 0 disables it. Defaults and notes are omitted from legacy artifact identity and argv. | unverified |
| `bus_bringup.explicit_pdo_config` | `true` | R:65-87 and P.ethercat.tx_layout differ; requires explicit mapping. Acceptance of remapped 0x1B02 must be measured during inhibited bus step. | derived |
| `bus_bringup.disable_output_watchdog` | `false` | X.bus_bringup.disable_output_watchdog; measure drive output-watchdog behavior on the bench; false keeps the watchdog enabled. | unverified |
| `bus_bringup.no_dc` | `false` | X.bus_bringup.no_dc; measure distributed-clock startup and synchronization on the bench; false keeps DC enabled. | unverified |
| `bus_bringup.wait_before_safeop_ms` | `250` | X.bus_bringup.wait_before_safeop_ms; measure firmware convergence before SAFEOP on the bench (ms, 0..2147483647); 0 leaves the master default. | unverified |
| `bus_bringup.preop_safeop_timeout_ms` | `5000` | X.bus_bringup.preop_safeop_timeout_ms; measure PREOP to SAFEOP on the bench (ms, 0..4294967295); 0 leaves the master default. | unverified |
| `bus_bringup.safeop_op_timeout_ms` | `5000` | X.bus_bringup.safeop_op_timeout_ms; measure SAFEOP to OP on the bench (ms, 0..4294967295); 0 leaves the master default. | unverified |
| `max_cycle_lateness_ns` | `1000000` | config/machines/a6ec-bench.example.json#max_cycle_lateness_ns; measure worst admitted cycle lateness on the RevPi under intended load; captured jitter is not deadline qualification | unverified |
| `runtime.cpu` | `0` | revpi-deploy.txt#CPUAffinity=0,===cpu=4,===cmdline; hostcheck.json#checks[id=isolated]; retain deployed CPU 0; no isolation; qualify cpuset, housekeeping, IRQ placement and cycle latency | unverified |
| `runtime.priority` | `90` | config/machines/a6ec-bench.example.json#runtime.priority; revpi-deploy.txt#LimitRTPRIO=95; FIFO 90 stays below the deployed RT limit 95; qualify service admission and scheduler interference | unverified |
| `axes[0].name` | `"J1"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:131 | cited |
| `axes[0].slave_position` | `0` | deployed axis index; R:14 | cited |
| `axes[0].profile` | `"a6ec"` | P.id, P.ethercat; R:19-21 SII 4194304/1813/12024 | cited |
| `axes[0].type` | `"rotary"` | P.position_semantics; R:55 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[0].wire_counts_per_rev` | `13107200` | W=131072*100/1=13107200; B:1782, P.position_semantics; 0x607C=6553600=W/2 at R:60 | derived |
| `axes[0].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[0].sign` | `-1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:131; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[0].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:53 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[0].startup.a6ec_rotation_mode_gear_ratio_numerator` | `100` | S.pair_config_gear_numerator[0]; R:55 | cited |
| `axes[0].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | S.pair_config_gear_denominator[0]; R:56 | cited |
| `axes[0].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:54; P.startup_defaults | cited |
| `axes[0].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[0].limits.min` | `-3.1066855891809104` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[0], hm35_home_position_counts[0]; max(band_min, pair_config_lower_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[0].limits.max` | `3.1066860685499065` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[0], hm35_home_position_counts[0]; min(band_max, pair_config_upper_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[0].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[0].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[0].limits.max_target_lead` | `0.006108652381980153` | C:67,2941-2948; S.axis_active_jog_max_lead_counts[0]; 0.35*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[0].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[0].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[0].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[0].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[1].name` | `"J2"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:132 | cited |
| `axes[1].slave_position` | `1` | deployed axis index; R:89 | cited |
| `axes[1].profile` | `"a6ec"` | P.id, P.ethercat; R:94-96 SII 4194304/1813/12024 | cited |
| `axes[1].type` | `"rotary"` | P.position_semantics; R:130 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[1].wire_counts_per_rev` | `13107200` | W=131072*100/1=13107200; B:1782, P.position_semantics; 0x607C=6553600=W/2 at R:135 | derived |
| `axes[1].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[1].sign` | `1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:132; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[1].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:128 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[1].startup.a6ec_rotation_mode_gear_ratio_numerator` | `100` | S.pair_config_gear_numerator[1]; R:130 | cited |
| `axes[1].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | S.pair_config_gear_denominator[1]; R:131 | cited |
| `axes[1].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:129; P.startup_defaults | cited |
| `axes[1].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[1].limits.min` | `-1.884955592` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[1], hm35_home_position_counts[1]; max(band_min, pair_config_lower_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=1. Formula above preserves unrounded band. | derived |
| `axes[1].limits.max` | `1.570796327` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[1], hm35_home_position_counts[1]; min(band_max, pair_config_upper_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=1. Formula above preserves unrounded band. | derived |
| `axes[1].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[1].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[1].limits.max_target_lead` | `0.006108652381980153` | C:68,2941-2948; S.axis_active_jog_max_lead_counts[1]; 0.35*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[1].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[1].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[1].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[1].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[2].name` | `"J3"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:133 | cited |
| `axes[2].slave_position` | `2` | deployed axis index; R:164 | cited |
| `axes[2].profile` | `"a6ec"` | P.id, P.ethercat; R:169-171 SII 4194304/1813/12024 | cited |
| `axes[2].type` | `"rotary"` | P.position_semantics; R:205 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[2].wire_counts_per_rev` | `13107200` | W=131072*100/1=13107200; B:1782, P.position_semantics; 0x607C=6553600=W/2 at R:210 | derived |
| `axes[2].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[2].sign` | `-1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:133; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[2].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:203 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[2].startup.a6ec_rotation_mode_gear_ratio_numerator` | `100` | S.pair_config_gear_numerator[2]; R:205 | cited |
| `axes[2].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | S.pair_config_gear_denominator[2]; R:206 | cited |
| `axes[2].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:204; P.startup_defaults | cited |
| `axes[2].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[2].limits.min` | `-1.396263402` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[2], hm35_home_position_counts[2]; max(band_min, pair_config_lower_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[2].limits.max` | `1.396263402` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[2], hm35_home_position_counts[2]; min(band_max, pair_config_upper_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[2].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[2].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[2].limits.max_target_lead` | `0.006108652381980153` | C:69,2941-2948; S.axis_active_jog_max_lead_counts[2]; 0.35*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[2].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[2].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[2].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[2].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[3].name` | `"J4"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:134 | cited |
| `axes[3].slave_position` | `3` | deployed axis index; R:239 | cited |
| `axes[3].profile` | `"a6ec"` | P.id, P.ethercat; R:244-246 SII 4194304/1813/12024 | cited |
| `axes[3].type` | `"rotary"` | P.position_semantics; R:280 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[3].wire_counts_per_rev` | `2359296` | W=131072*18/1=2359296; B:1782, P.position_semantics; 0x607C=1179648=W/2 at R:285 | derived |
| `axes[3].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[3].sign` | `-1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:134; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[3].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:278 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[3].startup.a6ec_rotation_mode_gear_ratio_numerator` | `18` | S.pair_config_gear_numerator[3]; R:280 | cited |
| `axes[3].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | S.pair_config_gear_denominator[3]; R:281 | cited |
| `axes[3].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:279; P.startup_defaults | cited |
| `axes[3].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[3].limits.min` | `-3.106683405388816` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[3], hm35_home_position_counts[3]; max(band_min, pair_config_lower_limit_rad) using L=2359296/180, U=2359296-1-L, H=1179648, K=2359296/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[3].limits.max` | `3.106686068549907` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[3], hm35_home_position_counts[3]; min(band_max, pair_config_upper_limit_rad) using L=2359296/180, U=2359296-1-L, H=1179648, K=2359296/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[3].limits.max_velocity` | `17.45329252` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 18:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[3].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[3].limits.max_target_lead` | `0.008726646259971648` | C:70,2941-2948; S.axis_active_jog_max_lead_counts[3]; 0.5*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[3].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[3].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[3].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[3].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[4].name` | `"J5"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:135 | cited |
| `axes[4].slave_position` | `4` | deployed axis index; R:314 | cited |
| `axes[4].profile` | `"a6ec"` | P.id, P.ethercat; R:319-321 SII 4194304/1813/12024 | cited |
| `axes[4].type` | `"rotary"` | P.position_semantics; R:355 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[4].wire_counts_per_rev` | `1769472` | W=131072*27/2=1769472; P.position_semantics; HM35 0x607C=884736=W/2. R:360 is historical 100/1 readback assigned to the 1420 by the owner decision 2026-09-15 | derived |
| `axes[4].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[4].sign` | `-1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:135; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[4].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:353 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[4].startup.a6ec_rotation_mode_gear_ratio_numerator` | `27` | robots/rosie_1400_v3.json J5; owner decision 2026-09-15; GradientOS gradient05 2945e3ff, DEVLOG 2026-05-01, sheet rated speed and CAD belt stage (J5-history.md); 0x2010:19 range 1-65535 | derived; physical pulleys unverified |
| `axes[4].startup.a6ec_rotation_mode_gear_ratio_denominator` | `2` | robots/rosie_1400_v3.json J5, same sources; 0x2010:1A range 1-65535 | derived; physical pulleys unverified |
| `axes[4].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:354; P.startup_defaults | cited |
| `axes[4].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[4].limits.min` | `-3.1066855891809104` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[4], hm35_home_position_counts[4]; max(band_min, pair_config_lower_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[4].limits.max` | `1.047197551` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[4], hm35_home_position_counts[4]; min(band_max, pair_config_upper_limit_rad) using L=13107200/180, U=13107200-1-L, H=6553600, K=13107200/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[4].limits.max_velocity` | `23.27105669` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 27/2:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[4].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[4].limits.max_target_lead` | `0.011344640137963142` | C:71,2941-2948; S.axis_active_jog_max_lead_counts[4]; 0.65*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[4].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[4].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[4].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[4].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[5].name` | `"J6"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:136 | cited |
| `axes[5].slave_position` | `5` | deployed axis index; R:389 | cited |
| `axes[5].profile` | `"a6ec"` | P.id, P.ethercat; R:394-396 SII 4194304/1813/12024 | cited |
| `axes[5].type` | `"rotary"` | P.position_semantics; R:430 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[5].wire_counts_per_rev` | `1310720` | W=131072*10/1=1310720; B:1782, P.position_semantics; 0x607C=655360=W/2 at R:435 | derived |
| `axes[5].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[5].sign` | `-1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:136; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[5].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:428 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[5].startup.a6ec_rotation_mode_gear_ratio_numerator` | `10` | S.pair_config_gear_numerator[5]; R:430 | cited |
| `axes[5].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | S.pair_config_gear_denominator[5]; R:431 | cited |
| `axes[5].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:429; P.startup_defaults | cited |
| `axes[5].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); v4 evidence is not imported | derived |
| `axes[5].limits.min` | `-2.094395102` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[5], hm35_home_position_counts[5]; max(band_min, pair_config_lower_limit_rad) using L=1310720/180, U=1310720-1-L, H=655360, K=1310720/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[5].limits.max` | `2.094395102` | C:2784-2810, T:86-87; S.safe_low/high_raw_counts[5], hm35_home_position_counts[5]; min(band_max, pair_config_upper_limit_rad) using L=1310720/180, U=1310720-1-L, H=655360, K=1310720/(2*pi), s=-1. Formula above preserves unrounded band. | derived |
| `axes[5].limits.max_velocity` | `31.41592654` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 10:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[5].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[5].limits.max_target_lead` | `0.013962634015954637` | C:72,2941-2948; S.axis_active_jog_max_lead_counts[5]; 0.8*pi/180 rad; v4 clamp on 20 cycles of step, applied by rt-core as a target-to-feedback fault threshold; measure tracking error at max_velocity under load. | unverified |
| `axes[5].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[5].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[5].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[5].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[6].name` | `"J7"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:137 | cited |
| `axes[6].slave_position` | `6` | deployed axis index; R:464 | cited |
| `axes[6].profile` | `"a6ec"` | P.id, P.ethercat; R:469-471 SII 4194304/1813/12024 | cited |
| `axes[6].type` | `"rotary"` | P.position_semantics; R:505 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[6].wire_counts_per_rev` | `1310720` | W=131072*10/1=1310720; B:1782, P.position_semantics; 0x607C=655360=W/2 at R:510 | derived |
| `axes[6].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[6].sign` | `1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:137; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[6].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:503 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[6].startup.a6ec_rotation_mode_gear_ratio_numerator` | `10` | R:505; v4 requested 1, observed 0, wrote no axes. Preserve actual 10; do not reconcile to 1. | cited |
| `axes[6].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | R:506 | cited |
| `axes[6].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:504; P.startup_defaults | cited |
| `axes[6].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); axis has no v4 Home; fresh qualified homing required before motion | derived |
| `axes[6].limits.min` | `-0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.min; v4-status.json#safe_low_raw_counts[6]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[6].limits.max` | `0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.max; v4-status.json#safe_high_raw_counts[6]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[6].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[6].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[6].limits.max_target_lead` | `0.010000000000000002` | config/machines/a6ec-bench.example.json#axes[0].limits.max_target_lead; rad; measure tracking and stopping with 10/1. Captured 254.862222 counts used v4's incorrect 1/1, so do not reuse it as a qualified lead bound | unverified |
| `axes[6].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[6].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[6].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[6].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[7].name` | `"J8"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:138 | cited |
| `axes[7].slave_position` | `7` | deployed axis index; R:539 | cited |
| `axes[7].profile` | `"a6ec"` | P.id, P.ethercat; R:544-546 SII 4194304/1813/12024 | cited |
| `axes[7].type` | `"rotary"` | P.position_semantics; R:580 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[7].wire_counts_per_rev` | `1310720` | W=131072*10/1=1310720; B:1782, P.position_semantics; 0x607C=655360=W/2 at R:585 | derived |
| `axes[7].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[7].sign` | `1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:138; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[7].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:578 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[7].startup.a6ec_rotation_mode_gear_ratio_numerator` | `10` | R:580; v4 requested 1, observed 0, wrote no axes. Preserve actual 10; do not reconcile to 1. | cited |
| `axes[7].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | R:581 | cited |
| `axes[7].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:579; P.startup_defaults | cited |
| `axes[7].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); axis has no v4 Home; fresh qualified homing required before motion | derived |
| `axes[7].limits.min` | `-0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.min; v4-status.json#safe_low_raw_counts[7]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[7].limits.max` | `0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.max; v4-status.json#safe_high_raw_counts[7]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[7].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[7].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[7].limits.max_target_lead` | `0.010000000000000002` | config/machines/a6ec-bench.example.json#axes[0].limits.max_target_lead; rad; measure tracking and stopping with 10/1. Captured 254.862222 counts used v4's incorrect 1/1, so do not reuse it as a qualified lead bound | unverified |
| `axes[7].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[7].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[7].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[7].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |
| `axes[8].name` | `"J9"` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:139 | cited |
| `axes[8].slave_position` | `8` | deployed axis index; R:614 | cited |
| `axes[8].profile` | `"a6ec"` | P.id, P.ethercat; R:619-621 SII 4194304/1813/12024 | cited |
| `axes[8].type` | `"rotary"` | P.position_semantics; R:655 rotation-mode ratio; external mechanism must still be qualified | cited |
| `axes[8].wire_counts_per_rev` | `1310720` | W=131072*10/1=1310720; B:1782, P.position_semantics; 0x607C=655360=W/2 at R:660 | derived |
| `axes[8].wire_revs_per_axis_rev` | `1` | P.position_semantics drive_output_shaft; one wire output revolution per axis revolution; W already includes G (include/coordinate_model.hpp:63-68) | derived |
| `axes[8].sign` | `1` | revpi-deploy.txt#ROBOT_V4_ROBOT_AXES; v4/manifest/bench-relay.json:139; B:1770; include/coordinate_model.hpp:104-119 | cited |
| `axes[8].startup.a6ec_encoder_position_tracking_mode` | `4` | P.startup_defaults; R:653 currently 0. Operator decision before bus ownership; measure 0x2040:21 continuity/scale/retention and 0x2040:43 invalidation in mode 4. | unverified |
| `axes[8].startup.a6ec_rotation_mode_gear_ratio_numerator` | `10` | R:655; v4 requested 1, observed 0, wrote no axes. Preserve actual 10; do not reconcile to 1. | cited |
| `axes[8].startup.a6ec_rotation_mode_gear_ratio_denominator` | `1` | R:656 | cited |
| `axes[8].startup.a6ec_rotation_mode_reference_running_direction` | `0` | R:654; P.startup_defaults | cited |
| `axes[8].require_home` | `true` | S.home_valid_axis_mask=63; retain gate (include/control_policy.hpp:76-79); axis has no v4 Home; fresh qualified homing required before motion | derived |
| `axes[8].limits.min` | `-0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.min; v4-status.json#safe_low_raw_counts[8]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[8].limits.max` | `0.25` | config/machines/a6ec-bench.example.json#axes[0].limits.max; v4-status.json#safe_high_raw_counts[8]=0 because axis unmapped; measure mechanical radian travel relative to qualified Home before motion. Compiler requires min < max and positive velocity; placeholder is not an axis-disable policy | unverified |
| `axes[8].limits.max_velocity` | `3.14159265` | owner decision 2026-09-21: the 3000 rpm drive cap (drive_speed_limit_motor_rpm) through the 100:1 gear ratio, 314.159265 / ratio rad/s, equal to the URDF velocity rating (P8-44); supersedes the v4 axis_max_profile_velocity_counts value (0.2 rad/s nominal, C:2659-2681). | decided |
| `axes[8].limits.jog_acceleration` | `0.5` | config/machines/a6ec-bench.example.json#axes[0].limits.jog_acceleration; config/drives/a6ec.json#max_acceleration; rad/s^2; measure ramp and stopping under load. v4 uses 120 rpm/s = 4*pi rad/s^2, which is not qualified for rt-core's profile bound | unverified |
| `axes[8].limits.max_target_lead` | `0.010000000000000002` | config/machines/a6ec-bench.example.json#axes[0].limits.max_target_lead; rad; measure tracking and stopping with 10/1. Captured 254.862222 counts used v4's incorrect 1/1, so do not reuse it as a qualified lead bound | unverified |
| `axes[8].limits.following_error` | `0.020000000000000004` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error; rad; measure maximum tracking error under load on this axis | unverified |
| `axes[8].limits.following_error_timeout_ms` | `100` | config/machines/a6ec-bench.example.json#axes[0].limits.following_error_timeout_ms; ms; measure tracking-error persistence and fault response on this axis | unverified |
| `axes[8].limits.completion_tolerance` | `0.0001` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_tolerance; rad; measure stationary endpoint error and encoder noise on this axis | unverified |
| `axes[8].limits.completion_timeout_ms` | `1000` | config/machines/a6ec-bench.example.json#axes[0].limits.completion_timeout_ms; ms; measure endpoint settling and timeout response on this axis | unverified |

## Compilation and regression proof

The executable package is `./tools/rtctl/cmd/rtctl`; `./tools/rtctl`
itself is a library (Makefile control target and `tools/rtctl/cmd/rtctl/main.go`).

```sh
go run ./tools/rtctl/cmd/rtctl compile --config config/machines/bench-ninemotor-a6ec.json --out build/benchconfig
go run ./tools/rtctl/cmd/rtctl validate --config config/machines/bench-ninemotor-a6ec.json --backend ipc-only --out build/benchconfig
```

Both commands run in the prescribed WSL toolchain/environment. Validation uses
the native IPC-only binary's `--validate-config` path and exits before runtime
or bus ownership; the JSON retains `backend=live`. This validates compiled
native descriptors and identities, not the live IgH backend or hardware.

Compiled configuration SHA-256 (updated notes change this compatibility digest):
`651bc03e7805fa222c65a075d80a2795dff378c1aa6ded44a308fdf4a5c93291`.

Effective machine SHA-256 (re-measured 2026-09-21: J7 to J9 at 100:1 and max_velocity at the 3000 rpm cap through each ratio):
`ff8c7404e589cfe8fff768f5de888124f35bd748510c4690b93dfa688d46a7c9`.

Deployment SHA-256 (unchanged by the profile selection):
`87115dd2cc73499d5ae8a3d3767426eaa86374c51b05b3e926793a5d1605419a`.

Fresh-compilation identity changes from the pre-preset head (`7f834034`),
shown as 12-character SHA-256 prefixes:

| Input / identity | Before | After |
|---|---|---|
| bench-relay configuration | `4019e11829d7` | `651bc03e7805` |
| bench-relay machine | `70a7a1041235` | `01aeeb011786` |
| bench-relay deployment | `87115dd2cc73` | unchanged |
| Nine-motor bench configuration | `d11b572df1a5` | `fd989fda44b6` |
| Fast bench A configuration | `560cd5fb595d` | `82b32a9c95f1` |
| Fast bench B configuration | `6b8173a26855` | `ca3daa63d416` |
| Legacy robot fixture / inventory configuration | `9b5910e4d52b` | `f5ba25b735c6` |
| Legacy robot fixture machine | `7ee6dd49eb83` | `6d862297dbfe` |
| Legacy bench fixture configuration | `2d8d72a4fe6a` | `8c5ac16d6375` |
| Synthetic inventory configuration | `2774cab272c56` | `df84802f2b1b` |
| rdtcheck profile bytes (now the 1:1 bench profile) | `2cedd74c0d90` | `ab998517b693` |
| rdtcheck example configuration bytes | `238d818631ff` | `ea1205a69c51` |
| C++ simulation example configuration | `d8010655f880` | unchanged |

All coordinate identities are unchanged. The bench and fast-bench machine and
deployment identities are unchanged; their configuration digests move because
the profile provenance now records accepted assignment. `TestMergedConfigurationIdentityPins`,
`TestA6FixedPDOArtifacts`, `TestA6FeedforwardLegacyArgv` and
`TestInventoryNineMotorMatchGolden` pin the corresponding complete digests.
`TestNineMotorDefaultDriveTelemetryValidity` compiles this robot cell and checks
all nine simulated axes at 1000000 ns: signed velocity/following-error values
are valid, with `di_bits=0` and `di_valid=false`.

The bare-motor cell `config/machines/bench/nine-motors-1to1.json` now has configuration
SHA-256 `fd989fda44b6f7f0923a1d23acd285b89290a17731bfa4e48071dc7f7a00c797`,
measured by fresh compilation with the motor cap and corrected 607Fh provenance, pinned alongside
its machine and deployment digests by `TestRobotJ5CompiledCells`.

Regression coverage includes native corpus validation, profile brake defaults,
all-axis required A6 startup fractions, continuous source profile checks,
bring-up defaults/notes and the capture-derived limits/scaling contract.
`tools/rtctl/testdata/inventory/ninemotor-match.json` pins the complete
55-read synthetic inventory result with nine matching SII identities and nine
mixed-revision notes. WSL results do not close any unverified hardware row.
