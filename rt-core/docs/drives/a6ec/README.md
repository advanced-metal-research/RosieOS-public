# A6-EC servo drive references (ANCTL / STEPPERONLINE A6-EC, AS715N motors)

Drive-specific references kept in RosieOS because the standalone rt-core is the
component that talks to these drives. Copied from GradientOS at commit d47bc53b
(`docs/resources`, `docs/ethercat`) on 2026-09-14; GradientOS proved every control it
had on the robot with these drives, and the manufacturer correspondence here is the
authority behind several rt-core profile values. Keep them here as
drive-specific references and consult them before guessing at drive behaviour.

| File | What it is | Where rt-core uses it |
|---|---|---|
| `a6ec_manual_chapter_11_parameter_list.md`, `.pdf` | Full parameter list (C0x.xx and F objects, CoE 0x2000..0x20xx mapping, defaults, ranges) | `config/drives/a6ec.json` startup schema (linear tracking mode 0x2000:08, standard gear 0x6091:01/02 = 1/1, running direction), feedforward parameters C01.13..C01.18 (P8-17), speed limit defaults |
| `a6ec-manual-chapter-5-absolute-system.pdf` | Absolute encoder system: linear multi-turn and retired rotation/single-turn modes, home offset semantics | exact coordinate evidence from 0x2040:21/23 raw multi-turn and HM35 0x607C = 0; rotation-frame :2B/:2D is historical only |
| `a6ec_manual_codes.md`, `.json` | Fault and alarm codes | `config/drives/a6ec.json` coordinate policy (encoder error 0x7305 invalidates the coordinate; ordinary faults gate readiness only); its `fault_reset` list of the faults an operator Reset may reset (see "Operator reset of a drive alarm") |
| `ethercat/a6ec-manufacturer-notes-2026-04-15.md` | Manufacturer correspondence: HM35 homing sequence 6-7-15-31, statusword bits 12/15/13, 607C behaviour, gear-ratio change needs a fresh Home, no power cycle | `config/drives/a6ec.json` native_home transaction; bench findings P8-04, P8-05 |
| `ethercat/a6ec-manufacturer-notes-2026-10-09.md` | Manufacturer reply (ticket #7821) on gain tuning: C00.06 as the primary lever, dual-acceleration inertia measurement instead of F30.10 at J4, C02.38 low-frequency vibration suppression, notch floor 50 Hz, EEPROM write-on-change; vendor-uncertain points marked | `config/drives/a6ec.tuning.json`; per-joint tuning ("Tuning a joint") |
| `ethercat/a6ec-frame-semantics-and-native-home.md` | Historical position-frame investigation (6064, 607A, U40.16/20/22, 60FC) and native Home | background for the profile-declared raw source and HM35; the 2026-10-19 linear-mode contract in this README supersedes its rotation-mode posture |
| `ethercat/bringup.md` | GradientOS bring-up: PDO presets 0x1702/0x1B02 content, DC, startup SDOs | `config/drives/a6ec.json` ethercat section, `engine/bus_bringup.hpp` |
| `ethercat/igh.md` | IgH master notes (1.6.9 userspace library, kernel module) | `tools/build-igh-userlib.sh`, host qualification |
| `ethercat/rtcore_jog.md` | GradientOS jog behaviour notes | `engine/jog_lane.hpp` lineage |
| `brake-wiring-diagram.{md,mermaid,svg}`, `wiring-diagram.md` | Robot cell brake and drive wiring | P8-04 (brakes and stop chain still to measure on the cell), `config/drives/a6ec.json` brake policy |
| `esi/STEPPERONLINE_A6_Servo_V0.02.xml` | EtherCAT slave information file | `rtctl inventory` identity checks (vendor 0x400000, product 0x715, SII revision 0x2ef8) |

Not copied: the complete `A6-EC_series_servo_drive_manual.pdf` (17 MB) stays in
GradientOS `docs/resources`; the two chapters rt-core relies on are here.

Bench facts established against these drives on a nine-drive bench (2026-09-13/14) are
recorded on tracker items P8-04, P8-05 and P8-13.

## Speed feedforward, gains and built-in tuning (2026-10-06)

Sources: the full manual `A6-EC_series_servo_drive_manual.pdf` in this
directory (chapter 7 "Gain Tuning", printed pp.140-155; chapter 11 tables)
and the ESI.

**What reaches the speed loop in CSP.** Only 0x607A. 0x60FF target velocity is
used in CSV/PV only (chapter 11, 60FFh), so the counts/s rt-core writes there
are ignored in mode 8. External speed feedforward is 0x60B1 (C01.13 = 5), which
no fixed RxPDO preset maps (0x1701..0x1705); only the configurable 0x1600 could,
and rt-core performs no custom mapping writes. Internal feedforward (C01.13 = 1)
differentiates the 0x607A reference inside the drive and needs no PDO change.

| Code | Object | Meaning | Range, factory default | rt-core writes (2026-10-07) |
|---|---|---|---|---|
| C00.04 | 0x2000:05 | Gain auto-tuning mode: 0 manual, 1 standard (gains from stiffness level), 2 positioning | 0..2, 1 | catalog key |
| C00.05 | 0x2000:06 | Stiffness level | 1..31, 12 | catalog key |
| C00.06 | 0x2000:07 | Load inertia ratio, % | 0..12000, 100 | catalog key |
| C01.00..03 | 0x2001:01..04 | 1st position gain (0.1 rad/s), speed gain (0.1 Hz), speed integral (0.01 ms), torque filter (Hz) | 400, 250, 3184, 200 | catalog keys |
| C01.13 | 0x2001:14 | Speed feedforward source (0 none, 1 internal, 2 model tracking, 5 0x60B1); 7.6 says change at stop | 0..5, 0 | catalog key |
| C01.14 | 0x2001:15 | Speed feedforward, 0.1 % | 0..2000, 0 | catalog key |
| C01.15 | 0x2001:16 | Speed feedforward filter cutoff, Hz | 5..16000, 318 | catalog key |
| C01.16..18 | 0x2001:17..19 | Torque feedforward source/percentage/filter | 0, 0, 318 | catalog keys |
| C01.20..23 | 0x2001:21..24 | Position reference moving-average and low-pass filters, 0.1 ms | 0 (off) | catalog keys |
| C01.30 | 0x2001:31 | Adaptive notch (0 off, 1/2 notches, 3 reset, 4 measure only) | 0 | catalog key |
| C02.00..02 | 0x2002:01..03 | Model tracking enable/gain/inertia correction | 0, 500, 1000 | catalog keys |
| C02.38..4E | 0x2002:39.. | Low-frequency vibration suppression sets 1-3 ("Vendor answers 2026-10-09", Q8) | off | catalog keys |

Readback evidence: a bench drive read C01.00 = 400, C01.13 = 0, C01.14 = 0 and
then ran J6 with C01.13 = 1 / C01.14 = 500. The ~27 ms lag of a Rosie 1430 arm
(26.5-27.0 ms on every axis, error proportional to velocity) is what factory
gains with no feedforward predict. Until 2026-10-07 rt-core never wrote the
gains, stiffness, inertia ratio, filters or notches, so whatever each drive
holds stays in force until a configuration writes them ("Tuning a joint"
below).

Steady tracking lag of a P position loop with speed feedforward fraction k is
about (1 - k) / Kp plus transport. At C01.00 = 400 (Kp 40 rad/s): k = 0 gives
25 ms (measured 26.5-27.1 ms on J1/J2/J3/J5/J6 of a Rosie 1430 arm, Stop
dump), k = 0.5 gives 12.5 ms (measured 15.1-15.5 ms on the same arm).
The ~2 ms excess in both is cycle and interpolation delay.

**On-robot verification of a feedforward change.**
Deploy the configuration disarmed, then before Home or Arm confirm the
journal shows `0x2001 sub=0x14/0x15/0x16 commanded=1/500/318 readback=… verified=1`
on all six axes and `config_verified_mask=63`. Arm in OLP with Home valid and
run, at the operator's chosen speed, the same pattern as the baseline session:
single-joint jogs on J1, J2, J3 and J5 at about 10, 30 and 60 deg/s held for at
least 1 s each, then one Move-to of J1/J5/J6. Disarm (Stop) to write the dump
and copy it off the RevPi before any restart (a restart deletes
`/run/rosie-rt-core`, dumps included). Copy at low priority
(`sudo nice -n 19 ionice -c3 cat`, throttled to about 4 MB/s) and decode
elsewhere with `rt-core/ipcclient`: decoding on the servo host once stalled its
EtherCAT exchange for ~1 s and invalidated Home through `bus_transport`.
Fit tracking error against commanded velocity per axis (least squares,
error = lag x velocity; `tools/tuningreport` does this). Expected: velocity coefficient
falls from ~27 ms to about 14 ms on every joint, the remaining error tracks
acceleration more than velocity, peak following error at 60 deg/s drops from
~1.6 deg to under ~0.9 deg, feedback passes the final target by no more than
0.1 deg at any stop (an unverified acceptance number) and settles within about
200 ms, and `velocity_actual` shows no oscillation that was absent before. If any axis rings or overshoots, set C01.14
back to 0 for all axes; do not tune one joint (superseded 2026-10-07: tuning
is per joint, "Tuning a joint" below).

Result on a loaded Rosie 1430 arm with C01.13 = 1 / C01.14 = 500 (multi-joint
jogs and one J1/J6 Move-to; the single-joint jogs were not run): lag
15.1-15.5 ms on J1/J2/J3/J5/J6 (was 26.8-27.1), peak following error at
60 deg/s 0.93-0.96 deg (was 1.62-1.68), stop overshoot <= 0.07 deg, but
actual peak acceleration rose (J6 856 -> 1161 deg/s^2) and J4 rang at
~6.1 Hz to 1.60 deg (was 1.23), 33 ms above its bound, at a Move-to start.
Jerk limiting was added instead of lowering the percentage: the machine-level
`jog_jerk` (150 rad/s^3) also bounds OLP Move-to and Cartesian steps
(`docs/CONTRACTS.md`, "Jog jerk and jerk-limited one-off moves"). Unverified
on the arm (#158). One trajectory per session, with different geometry,
means the J4 1.23 -> 1.60 deg comparison is confounded. Every start of those
configurations logged `0x2001 sub=0x14/0x15/0x16 commanded=1/500/318
readback=1/500/318 verified=1` on all six axes.

**Feedforward off (measured on a Rosie 1430 cell).** With feedforward on, J2
(C00.06 800 %) oscillated at 100 Hz while moving, rising with its own speed to
557 permille 80-500 Hz torque at 30-60 deg/s. J2 alone at 0/0 cut that 70-97
percent in every speed band, J1 (~150 Hz, worst at high J1 speed with the
wrist near the J1 axis) and J3 (200 Hz) kept the same signature with
feedforward on, and the mixed lag (J2 27 ms, the others ~15 ms) put the
measured TCP up to 8.1 mm off the line in 400 mm/s holds. With 0/0 on every
joint (first as `startup_trial` keys, then stored by the deploy's trial
promotion, above), every joint's 80-500 Hz torque median was 3-9 permille in
every own-speed band up to 60 deg/s (J2 had read 557 at 30-60, J1 481), with
no buzz episode; J1 at 30-90 deg/s with the wrist 0.36-0.61 m from its axis
9 (was 356-481); measured TCP at most 1.2 mm off the line in 400 mm/s holds
(was 8.1); lag 26.5-27.1 ms on every joint, following error at most 0.42 of
its bound, no faults. Revert: 1/500 written explicitly on every axis.

**Persistence and reverting.** C13.10 (0x2013:11, "EtherCAT parameter
storage", 0 no storage / 1 storage, factory 1; chapter 11 table in this
directory) makes the drive store SDO-written parameters, and ALF1.3
(`a6ec_manual_codes.md`: "too frequent writing of EEPROM by host controller
SDO", class 3) is the drive's guard against doing it too often. With the
machine's `verify_parameter_storage` rt-core reads C13.10 at every start, writes 1 only if a drive holds
another value and refuses configuration verification until it reads 1;
without it rt-core does not touch C13.10. Consequences:
- A configuration that omits a startup key writes nothing, so the drive keeps
  the last value written. Rolling a cell back to a configuration that writes
  no feedforward keys leaves C01.13/14/15 at the last stored value (expected
  from storage, not read back after a rollback). To turn feedforward off on a
  drive that has it on, deploy a
  configuration that writes `a6ec_speed_feedforward_source: 0` and
  `a6ec_speed_feedforward_percentage: 0` explicitly on every axis, then
  confirm the readback lines. Since 2026-10-24,
  `realtime-core config` allows removal only for keys classified as tuning by
  the new compiled catalog: these become unmanaged and retain current drive
  values. Other startup-key removals remain refused. Omitting a key does not
  undo previously stored values. Deploying an older configuration that writes them manages them again.
  Exception to persistent retention: a `startup_trial`
  key is written with C13.10 = 0 and lives in drive RAM, so a drive power
  cycle reverts it to the stored value and rt-core writes it again at its
  next start.
- Taking a key out of `startup_trial` at the value the trial wrote is a
  promotion: `realtime-core config` starts a bridge configuration first (that
  key RAM-only one raw step away) so the new configuration's stored write
  differs from RAM and is stored (`docs/CONTRACTS.md`, "Promoting a trial").
  Without it EEPROM would keep the pre-trial value. The A6-EC offers no
  stored-value readback and no store object (chapter 11, ESI): only a drive
  power cycle with rt-core stopped proves what EEPROM holds (the bench check
  in `docs/CONTRACTS.md`, "Startup write on change and RAM-only trials").
- Since `913f1244` every rt-core start reads each startup
  object first and writes only those that differ (journal `skipped=identical`
  and `EtherCAT startup_sdo_summary written=.. skipped=..`); readback is
  unchanged. A steady-state start writes 0 objects per axis instead of 7-10.
  The A6-EC's own skip of identical writes and the ALF1.3 threshold stay
  unanswered (vendor Q10 below); `rt-core/docs/CONTRACTS.md`, "Startup write
  on change and RAM-only trials".

**Built-in tuning.** Since 2026-10-07 servo tuning is per-joint
machine configuration, written and read back at startup ("Tuning a joint"
below); the drive's own tuning may only measure. The manual offers:
- Offline inertia identification, F30.10 (0x2030:11), started from the keypad
  (chapter 7.2, 11.3.9). The drive itself moves the motor forward and reverse
  several times using C07.00..04 (0x2007:01..05): 500 rpm, 100 ms ramps, 15 %
  torque target, 2.00 motor revolutions (7.2 degrees of joint travel at 100:1,
  40 degrees at J4's 18:1). It fails with under 0.5 turns, below 3000 rpm/s or
  150 rpm, poor stiffness or "load torque changes dramatically" (gravity-loaded
  J2/J3), and writes C00.06. The object is RW in the dictionary, but the manual
  documents only keypad use; an EtherCAT-triggered run is untested.
- Gain tuning by stiffness level (C00.04 = 1, the factory mode): gains follow
  C00.05 and C00.06 from an internal table the manual does not publish;
  C00.04 = 0 leaves C01.00..03 manual. There is no one-key automatic gain
  search; the manual points to the vendor "backend tool" PC software for
  waveform-based tuning.
- Adaptive notch (C01.30 = 1/2) updates notch parameters online while running;
  C01.30 = 4 only measures the resonance frequency.
Parameters written over SDO are stored in drive EEPROM (ALF1.3 warns against
frequent SDO EEPROM writes), and rt-core's startup writes never touch these
codes, so tuning results would persist across rt-core restarts while staying
outside every configuration identity. Auto-tuning yields per-joint gains, which
conflicted with "nothing joint-specific" in `CLAUDE.md`; this was resolved
on 2026-10-07: tuning is per joint in the machine configuration, safety bounds
stay one joint-unit value, and identification (inertia ratio, C01.30 = 4
resonance frequency) only measures; its results go into the configuration.

## Tuning a joint (2026-10-07)

Every A6-EC tuning object rt-core may touch is declared once in
`config/drives/a6ec.tuning.json`: key, manual code, CoE object (the subindex is
the code's hexadecimal suffix plus one: C02.38 is 0x2002:39), type, manual
range and allowed values in raw drive units, factory default, unit and scale
(raw = value / scale), modification class ("At stop" or "During operation";
rt-core writes with the drive disabled, which satisfies both), manual citation
and class. Ninety-three entries: C00.04-C00.06; C01.00-C01.03 and the 2nd set
C01.08-C01.0B; speed-feedback filters C01.10-C01.12; speed and torque
feedforward C01.13-C01.18; PDFF and damping C01.1B/C01.1C; position-reference
filters C01.20-C01.23 and pre-charge C01.2A; position-reference notches
C01.24-C01.29; adaptive notch C01.30 and C01.31 (observe-only); gain
switchover C01.38-C01.3B; torque notches 1-5 C01.40-C01.4E; model tracking
C02.00-C02.02; speed observer C02.30-C02.32; vibration suppression 1-3
C02.38-C02.4E; disturbance observer C02.60-C02.63; friction compensation
C02.68-C02.6F. Nothing in homing (HM35, C10), gearing (0x6091, 0x2010:19/1A),
PDO mapping, limits (C03) or protection (C06) can be catalogued: rtctl refuses
it (`docs/CONTRACTS.md`, "Per-joint drive tuning").

UNCONFIRMED in the manual (each is recorded on its catalog entry): whether
C00.04 = 1/2 overwrite written C01.00-C01.03 (write C00.04 = 0 to make
configured gains authoritative); C00.05 range (table 1..31, text 0..41);
which notches C01.30 = 1/2 adapt (3rd/4th in 11.3.2, 1st/2nd in 7.14);
C01.31's meaning (table "test times", text "measured resonance", observe-only
here; C01.32/C01.33 are not in the table); C01.24-C01.29, C01.2A and C01.1C
have no description, only table rows; vibration suppression 1-3 have no
procedure and no enable switch, and that compensation ratios 0 (the defaults)
leave them inactive is inferred; C01.13/C01.16/C01.38 modification classes
differ between chapters 7 and 11; the C02.00/C02.01 rows are garbled in the
markdown extraction and were read from the PDF.

The machine configuration sets a key per axis in `startup` (robot cells:
catalog keys only) and lists keys to read in `observe` (machine level for every
axis, or per axis):

```json
"observe": ["a6ec_load_inertia_ratio", "a6ec_vibration_suppression_1_frequency"],
"axes": [
  {
    "robot_joint": "J4",
    "startup": {
      "a6ec_speed_feedforward_source": 1,
      "a6ec_speed_feedforward_percentage": 500,
      "a6ec_speed_feedforward_filter_hz": 318,
      "a6ec_vibration_suppression_1_frequency": {"value": 6.2, "unit": "Hz"}
    }
  }
]
```

The J4 line above is an example, not a cell's configuration: no vibration
suppression is set until it has been measured. `{"value": 6.2, "unit": "Hz"}`
compiles to raw 62 (scale 0.1 Hz); a value that is not a whole multiple of the
scale, a wrong unit, a value outside the manual range or outside the allowed
set is refused. A key written on an axis is dropped from that axis's
inherited observe list (its startup readback journals it).

Procedure, one joint and one variable at a time:

1. Measure. Read the baseline from the `EtherCAT startup observe` journal
   lines of the current start (up to 64 objects per joint, the per-axis
   capacity; a key written on a joint is read back instead). For a
   resonance, use existing Stop dumps, decoded off the servo host, or
   C01.30 = 4 (resonance measurement only) through a configuration. Drive
   auto-tuning (F30.10, the keypad) may measure an inertia ratio; its value
   then goes into the configuration, never stays as drive-held state. In the
   arm, measure C00.06 by the dual-acceleration differential with
   `tools/tuningreport inertia-plan` and `inertia` (README there, "Inertia
   test"; rotor inertias from the manual's chapter 12 in `config/motors`).
2. Set the per-axis value in the machine configuration with its
   `startup_source` measurement note, and commit.
3. Deploy configuration-only (`realtime-core config`,
   `steamdeck/real/v4/README.md`, "Deployment tiers"): it allows per-axis
   differences only for keys the compiled `tuning.json` lists and refuses
   removing a key. The first deploy that carries the catalog is a full rt-core
   package: an older rtctl refuses `observe` and non-feedforward tuning keys.
4. Read back. Before Home or Arm the journal must show
   `EtherCAT startup readback axis=3 ... key=<key> ... commanded=<raw> readback=<raw> verified=1`
   for each written key and `config_verified_mask=63`.
5. Test the same motion as the baseline (the standard short test program:
   still 15 s after arming and XYZ path holds at 400 mm/s; RxRyRz at
   45 deg/s and a Move-to; the tuned joint alone at 10/30/60 deg/s; a Stop
   after each part), disarm, copy the Stop dumps off and compare with
   `tools/tuningreport --baseline` in absolute units: following error,
   `velocity_actual` ringing amplitude and decay on that joint (1-30 Hz), and
   the "Servo loop quality" section for every joint up to 500 Hz (buzz
   episodes, torque lines and which joint carries them, the loop band by own
   speed). An inertia ratio above the real one shows only there (J4 700 %,
   228 Hz). Revert with an explicit value (the catalog's factory default),
   never by removing the key.

Notes: each written key adds one SDO read per axis per start and an EEPROM
write only when its value changes (write on change since `913f1244`; mark
commissioning values `startup_trial` to keep them in RAM). A
drive-side position-reference filter or notch (C01.20-C01.2A) delays that
joint's command relative to the others: the commanded path rt-core sends is
unchanged, but compare all joints' lag after using one (`CLAUDE.md`, TCP path
rule). Vibration suppression and torque notches act inside the drive's loops.

Recommended J4 sequence (Rosie 1430; J4 rang at ~5.9-6.1 Hz, and at
5.6-7 Hz while J4 itself was commanded still, i.e. a
disturbance response of J4's loop, not a command it was following):
(a) full rt-core package deploy carrying this catalog and the observe list,
then save J4's observe lines (C00.04-C00.06, gains, notches, VS1-3) as the
baseline; a C00.06 inertia ratio far below J4's real ratio (18:1 reflects far
more link inertia than 100:1) leaves the speed loop no faster than the
position loop (manual 7.3 wants 3-5x), so read it before anything else;
(b) fit J4's ring frequency from the baseline and earlier Stop dumps
(velocity sign changes are half periods; at least five
periods);
(c) J4 only: `a6ec_vibration_suppression_1_frequency` at the measured value
plus the compensation ratio the vendor confirms (C02.3D/C02.3E are 0 by
default; confirm with ANCTL which ones enable the function and record the
reply on the catalog entries; answered by analogy only, "Vendor answers
2026-10-09" below, Q8); configuration-only deploy; readback; repeat the
baseline motion; (d) only if command-excited ringing remains, a J4
position-reference notch (C01.24-C01.26, 0.1 Hz units: the C01.40 torque
notches start at 10 Hz), checking all joints' lag afterwards. A command
notch filters only J4's own reference, so it cannot damp the ring J4 shows
while commanded still.

### J4 inertia ratio, measured on a loaded Rosie 1430 arm

Steps (a) and the inertia-ratio part of the sequence above, run on the loaded
arm with configuration-only deploys, one value at a time, the same test moves
each time, reports from `tools/tuningreport`. The baseline read every joint's
drive-held C00.06 (442/1143/337/190/35/227 % for J1-J6; never written by
rt-core, origin unrecorded) with C00.04 = 1 and C00.05 = 12 everywhere. J4
rang while commanded still whenever J1-J3 or J6 accelerated.

| J4 C00.06 | Ring (worst stop) | Damping | Worst amplitude | FE / bound | Notes |
|---|---|---|---|---|---|
| 190 % (drive-held) | 6.4 Hz | 0.03 | 0.60 deg | 0.52 | baseline |
| 400 % | 7.4 Hz | 0.18-0.20 | 0.50 deg | 0.43 | start overshoot 39 -> 2 % |
| 700 % | 7.9 Hz | 0.15-0.17 | 0.28 deg | 0.25 | a 228 Hz J4 torque oscillation up to ~20 permille rated for ~9 s after arming, J4 still: the speed loop is at its stability edge |
| **550 %** | 7.2 Hz | 0.17 | 0.38 deg | 0.33 | no 228 Hz line (150-500 Hz torque 3.4-3.9 permille after arming, as at 400 %); stop settle 210 ms; **kept** |

What it shows: raising C00.06 cuts J4's disturbance amplitude, but damping
plateaus at ~0.17 from 400 % up, and 700 % is too stiff. C01.00-C01.03 read
back unchanged (400/250/3184/200) at every step, although J4's lag fell
(16.2 -> 14.6 ms from 190 to 400 %): in standard mode (C00.04 = 1) the drive's
effective gains are not visible in those readbacks (UNCONFIRMED; vendor
question). The 228 Hz line came from a scratch Welch spectrum of `TorqueRaw`
(0x6077) over armed, commanded-still windows; the tuning report's "Servo loop
quality" section now computes it for every joint (it reproduces this
episode: 228 Hz, J4 alone).

### Servo loops across sessions, Rosie 1430

Every recorded session of a Rosie 1430 cell, every joint, up to 500 Hz: J2
(C00.06 1143 %) runs a broad 100-200 Hz oscillation while it moves in seven of
nine sessions, feedforward on or off (up to 1098 permille RMS and +-2805
peak), and carries it into J1, J3 and J4. J1 and J3 have none of their own,
J4 at 550 % is clean, J5 (35 %) has a J5-only 70 Hz line at rest, J6 shares
the wrist ring. With the 2nd gain set C01.08-C01.0B and gain switchover
C01.38-C01.3B on the observe list, both read factory on every joint, and
C01.38 = 0 is "fixed to the 1st gain set", so gain switching is ruled out as
J2's cause. An observe list of 64 per joint is the native per-axis limit
(`kMaxStartupObservesPerAxis`): a further addition needs it raised.

## Vendor answers 2026-10-09

Source: STEPPERONLINE technical support
(technical@omc-stepperonline.com), "[#7821] New messages on this
conversation", sent from support@omc-stepperonline.com to
operator@example.com on 2026-10-09 07:00 UTC (00:00 PDT); Gmail
thread `1a11b7f242409d49`, message `1a11f76c7c2b8dc6`. It answers our eleven
questions of 2026-10-08 01:18 PDT (thread `19d8a4521e3a4cd4`) one by one;
the vendor states it is "upfront about any points where I am not entirely
certain". Numbers below are the email's; "Ours:" marks our own reading
against the manual in this directory (`A6-EC_series_servo_drive_manual.pdf`,
Version A00) and data from a Rosie 1430 cell. Nothing here was measured by the vendor.
The reply verbatim, with each answer marked stated/inferred/unconfirmed:
[`ethercat/a6ec-manufacturer-notes-2026-10-09.md`](ethercat/a6ec-manufacturer-notes-2026-10-09.md).

**Mechanism (preamble).** J4 is the only 18:1 axis, so per unit of load its
reflected inertia is (100/18)^2 ~ 31x that of the 100:1 axes and its gain
margin is the smallest. Reaction torque from J1/J6 acceleration couples
through the base into J4, whose position loop counteracts it and excites a
weakly damped 6.4 Hz mode. Motor/output amplitude 24 deg / 1.3 deg ~ 18.5,
the gear ratio: the arm moves as a unit, a "rigid-body mode with weak damping
from the position loop's restoring force", not a two-inertia flexible mode
(whose motor side would stay nearly still). That explains the C00.06 result
(damping 0.05 -> 0.18, start overshoot 39 -> 2 %, 6.4 -> 7.35 Hz). The
sweep to 700 % is "correct"; back off if the response becomes overdamped or
overshoot reverses. The five notches work at 50-8000 Hz and cannot reach
6.4 Hz; the drive has a "Low-Frequency Vibration Suppression" function that
suppresses two low-frequency vibrations at the end effector, "section 7.14".
S-curve (jerk-limited) J1/J6 commands reduce broadband excitation.
Ours: in manual A00, 7.14 "Vibration Suppression" (printed pp.153-155)
covers only the five notches and the adaptive notch; no section describes
low-frequency suppression, and C02.38-C02.4E appear only in the chapter 11
table. J1/J6 Move-to, jog and Cartesian steps are already jerk-limited
(`jog_jerk` 150 rad/s^3); J4 at 550 % rings at 7.2 Hz with damping
0.12-0.17 (table above).

**Q1, origin of C00.06.** Either an F30.10 identification was run and saved,
or the value was written by hand. Check by running identification on one
axis and reading C00.06 before and after, or by a factory reset (C00.06
should return to 100 %). Not answered: whether U40.34 records the last
identification.

**Q2, F30.10 under EtherCAT.** (a) The manual does not state a bus-state
restriction, but the drive must self-enable while the master owns 0x6040 in
OP: a conflict is "highly likely". Use PREOP, or disconnect the master. (b)
The ESI is a subset of the object dictionary. SDO mapping: index 0x2000 +
group (C07 -> 0x2007, F30 -> 0x2030), subindex = number + 1, so F30.10 is
0x2030:11; list the real objects with `ethercat sdos`. If writing 0x2030:11
starts identification, the motor moves: brake and travel must be safe first.

**Q3, brake.** The brake is not released in step with identification. Release
it first (power the brake by hand, or interlock the BK output with the
identification), or the motor stalls into an overcurrent fault.

**Q4, F30.10 operation.** (a) Not press-and-hold: F30.10 -> [S] -> set the
number of revolutions with up/down -> [S] -> the drive runs forward and
reverse by itself -> displays the inertia -> [M] ends. (b) [M] aborts;
"stop on key release" is the JOG function F30.00. (c) It runs the set number
of revolutions in each direction: travel must allow at least one motor
revolution each way. (d) The manual does not say whether the result is
saved to C00.06; non-default values on all six axes "strongly imply" it is.
(e) C07.00 bits: not answered. (f) C07.02 is the time from 0 to the C07.01
speed ("following standard design conventions", an inference). (g) For the
1000 W 80 mm motor (rated 3000 rpm, max 6000 rpm) use 500-2000 rpm for
identification; the 50-1000 vs 50-8000 rpm range conflict was not resolved.

**Q5, J4 inertia.** (a) F30.10 needs > 150 rpm and > 3000 rpm/s; 3000 rpm/s
at 18:1 is ~1000 deg/s^2 at the output, over J4's 720. These are
prerequisites for accuracy, so F30.10 is unusable for J4 in the arm. Instead
use the dual-acceleration differential method: command J4 in CSP with two
constant-acceleration segments (400 and 700 deg/s^2 at the output) over the
same velocity band; average 0x6077 torque (0.1 % rated, times rated torque
for N m) in each; J_total (motor side) = (T1 - T2) / (alpha1 - alpha2),
friction cancels; C00.06 = (J_total / J_motor - 1) x 100 %. (b) Rotor
inertia is on the motor drawing (not supplied). Ours: the vendor's example
band, 500-1500 rpm at the motor, is 167-500 deg/s at J4's output, above
J4's 80 deg/s limit (240 rpm at the motor); the band must sit inside the
joint limits, and both segments must also cover the same position range
and direction so that gravity and position-dependent friction cancel too.
The measurement tool is separate work (`tools/tuningreport`).

**Q6, standard mode gains.** (a) The internal formulas are not disclosed.
C00.05 selects the base gain group (speed loop Kp/Ki, position loop Kp,
filters); C00.06 scales it to the equivalent inertia; the C01.00-C01.03
readbacks (400/250/3184/200) are "the derived results for Rigidity Level 12
at that specific inertia ratio", and changing C00.05/C00.06 and reading C01
back is the right way to map it. Ours: that contradicts a Rosie 1430 cell, where
C01.00-C01.03 read back unchanged at C00.06 190, 400, 550, 700 and 800 %
while J4's lag fell 16.2 -> 14.6 ms; (b) whether the gains in effect differ
from the readbacks, and how to read them, stays open. Manual-mode procedure
(posted by STEPPERONLINE on the LinuxCNC forum, June 2026): C00.04 = 0 ->
confirm the inertia ratio -> raise C01.01 (speed gain) until slight
noise/vibration, back off 15-20 % -> lower C01.02 (speed integral time)
until steady-state error is gone, not so low that it creeps at low speed ->
raise C01.00 (position gain), back off 10-15 % on overshoot -> then
feedforward and friction compensation. For J4, after the inertia ratio, the
speed loop next and the position loop last.

**Q7, recommended approach.** Short term: standard mode with an accurate
C00.06 per axis (the six axes' reflected inertia differs ~31x; J4 190 ->
400 % proved it). 100:1 axes: verify C00.06 (differential method, or from
load and gear ratio), then sweep C00.05 starting at 10-15, only if tracking
or stability targets are not met. J4: C00.06 accurately, keep C00.05 = 12,
and confirm damping stays above 0.15 before raising stiffness. Switch an
axis to manual mode only if, after C00.06, it tracks poorly without
oscillating, or its position-loop bandwidth must be set independently.
Hybrid allowed (100:1 axes standard, J4 manual).

**Q8, C02.38-C02.4E.** (a, b) A "Class 2" vibration-suppression group,
"low-frequency oscillation/vibration suppression", suppressing two
low-frequency oscillations at the mechanism end at once. Enabled as we
surmised (compensation ratio > 0), "consistent with similar designs": Delta's
low-frequency suppression has two parameters per group, frequency and
response level, active when the level is > 0, faster with a higher level,
and too high "can adversely affect operation". (c) "If this function is
similar to Delta's (range 1.0-100.0 Hz)", 6-7 Hz is in range. Procedure:
write 6.4 Hz by SDO (64 if the unit is 0.1 Hz), read back, run a J4
excitation test comparing amplitude and damping; it is the only relevant
drive-side tool. (d) Model following C02.00 shapes the command; C02.38 is
frequency-specific suppression compensation; C01.13/C01.14 is speed
feedforward; their order is not in the manual (measure the frequency
response). J4's ring comes from other axes' reaction forces, which do not
pass J4's command path, so J4's own model following or feedforward will
barely help; C00.06 (feedback path) and C02.38 ("acts on the torque/feedback
path") can. Ours, from the chapter 11 table (A00 printed pp.206-208; the
chapter 11 excerpt's pp.204-206), all "During operation", "Immediately",
catalogued in `config/drives/a6ec.tuning.json`:

| Set 1 / 2 / 3 | Object (0x2002:) | Meaning | Range, unit | Factory |
|---|---|---|---|---|
| C02.38 / C02.40 / C02.48 | 39 / 41 / 49 | frequency | 10..20000 x 0.1 Hz (1.0-2000.0 Hz), U16 | 1000 (100.0 Hz) |
| C02.39 / C02.41 / C02.49 | 3A / 42 / 4A | inertia correction | 10..8000 x 0.1 % (1.0-800.0 %), U16 | 1000 (100.0 %) |
| C02.3A / C02.42 / C02.4A | 3B / 43 / 4B | low-pass filter correction | -9999..9999 x 0.1 Hz, I16 | 0 |
| C02.3B / C02.43 / C02.4B | 3C / 44 / 4C | high-pass filter 1 correction | -9999..9999 x 0.1 Hz, I16 | 0 |
| C02.3C / C02.44 / C02.4C | 3D / 45 / 4D | high-pass filter 2 frequency | 10..50000 x 0.1 Hz, U16 | 20000 (2000.0 Hz) |
| C02.3D / C02.45 / C02.4D | 3E / 46 / 4E | compensation ratio 1 | 0..20000 x 0.1 %, U16 | 0 |
| C02.3E / C02.46 / C02.4E | 3F / 47 / 4F | compensation ratio 2 | 0..20000 x 0.1 %, U16 | 0 |

The table settles the unit (0.1 Hz: 6.4 Hz is raw 64) and the range
(1.0-2000.0 Hz, not Delta's 1.0-100.0). It has three sets of seven objects,
not two groups of frequency plus response level: which compensation ratio
is the "response level", what 100 % means, what the inertia and filter
corrections do, and whether a set acts on a disturbance J4 sees while
commanded still are UNCONFIRMED (the vendor's answer is an analogy). Every
drive of a Rosie 1430 cell read factory values on all 21 objects (observe
journal). A J4 trial writes C02.38 = 72 and
C02.3D = 100 on J4 only; the catalog entries are unchanged (any byte change
under `config/drives/` makes `release` pick the full tier).

**Q9, notches.** (a) Not answered as asked (manual A00 7.14 says the 1st and
2nd notches adapt; 11.3.2 says the 3rd and 4th). Method offered instead:
seed the four C01.40-series notch frequencies with distinct known values far
from any resonance, enable C01.30 by SDO, read all four back after
identification: the group that changed is the adaptive one. (b) "C01.31 =
0x2001:20, C01.32 = 0x2001:21, C01.33 = 0x2001:22". Ours: wrong. The
chapter 11 table puts C01.30 at 0x2001:31 and C01.31 at 0x2001:32, and a Rosie 1430 cell
reads C01.31 there (value 0); the vendor added 1 to the decimal
suffix (31 + 1 = 32 = 0x20) while its own F30.10 -> 0x2030:11 adds 1 to the
hexadecimal one. By the table's rule C01.32/C01.33 would be 0x2001:33/34
(not in the table; read-only probe only). Never write 0x2001:20-22 on the
vendor's mapping. (c) The specified lower limit is 50 Hz; that C01.40
accepts 10 Hz does not mean it works there (it may clamp internally).

**Q10, EEPROM (C13.10 = 1).** (a) Most general-purpose firmware skips a
write of an identical value, but the A6-EC needs FAE confirmation. (b) ALF1.3
threshold: not answered; EEPROM endurance is typically 100,000-1,000,000
write/erase cycles per sector or byte. (c) Recommended either way: read back
by SDO, compare with the target, write (with C13.10 = 1) only if it changed,
which takes steady-state EEPROM writes per axis from ~10 per start to 0; and
write finalized parameters to EEPROM once while keeping values that change
during commissioning in RAM only (C13.10 not set), committing them when
final. Ours: done in `913f1244` (write on change, `startup_trial`); RAM-only
commissioning changes what a rollback or power cycle reverts (see
"Persistence and reverting", whose "vendor confirmation is open" is now this
answer).

**Q11, 5-10 Hz load oscillation.** In order: (1) calibrate C00.06, "the
fundamental fix" (J4's 190 % is "clearly an underestimate"; measured by the
differential method, damping should rise above 0.05-0.1); (2) C02.38, after
the Q8 check, "highly likely to be directly effective"; (3) disturbance
observer C02.60-C02.63: no manual details, consult an FAE, its bandwidth
could turn the 6-7 Hz disturbance into a new excitation, test incrementally;
(4) command-side smoothing: S-curve or jerk limits on J1/J6, zero cost, do it
first; (5) mechanical review of the 6.4 Hz mode and its low damping (0.05-0.1):
reducer backlash, link connection stiffness, damping material. The vendor
asks for our data back: J4's C00.06 by the differential method and the
results of Q8 (C02.38) and Q9 (notch identification).

Still open after the reply: which C02.3D/C02.3E enables a set and what its
scale means (Q8); effective gains in standard mode (Q6b, Q6c); U40.34 (Q1);
C07.00 bits and the C07.01 range (Q4e, Q4g); which notches adapt (Q9a);
skip-on-identical and the ALF1.3 threshold (Q10a, Q10b, "consult an FAE");
the disturbance observer (Q11). Never asked: standard mode's gain switchover
C01.38 (a Rosie 1430 cell reads C01.38 = 0, "fixed to the 1st gain set", on every
joint, so it cannot explain J2), the J2/J3 379 Hz and J5
70 Hz torque lines at rest, and F30.10 at J5 or under gravity load (J2/J3).

## Tuning report (`tools/tuningreport`)

The before/after comparison of a tuning change is `tools/tuningreport`
(README there): per joint lag, following error and target lead against the
scaled bounds, actual against commanded acceleration, jerk, stop overshoot and
settle, the joint's ring frequency and damping ratio from
actual - commanded(t - lag), and every drive's loop quality up to 500 Hz
(torque, velocity and residual band energies per state and hold, lines and
their coupling, buzz episodes, ripple at rest); `--baseline` marks each metric
better or worse than the previous iteration. Copy and report in one line from
`steamdeck/real/v4` (`steamdeck/real/v4/README.md`, "Analysing a tuning
iteration"):

```sh
go run . realtime-core copy-dumps --manifest rt-core/config/cells/<cell>.json --to mac && (cd ../../../rt-core && go run ./tools/tuningreport report --dumps ~/.rosie/stop-dumps/<cell> --since 45m --manifest config/cells/<cell>.json --label "<what changed>" --out ~/.rosie/tuning-reports/<iteration> --baseline ~/.rosie/tuning-reports/<previous>/metrics.json)
```

## Absolute linear mode migration (2026-10-19)

Every A6-EC axis uses absolute linear mode (`C00.07 = 1`).
The robot definition's motor-to-axis ratio is applied in rt-core, while the
drive's standard gear ratio `0x6091:01/02` is pinned `1/1`. This keeps `6064`
and `607A` on one continuous signed motor-count number line and removes the
rotation-mode seam. Chapter 5 and Chapter 11 require a drive power cycle for
the mode change; a daemon restart is not sufficient.

The A6 profiles declare the independent coordinate source rather than selecting
it from vendor/product identity in C++. The source is the bracketed signed
64-bit raw multi-turn acquisition at `0x2040:23/21/23`, identity
`raw_multi_turn_encoder|3`. Linear U40.24/U40.26 objects are diagnostics, not
an alternate coordinate authority. The source and continuous PDO history are
compared as complete signed values within the configured acquisition and anchor
tolerances. A whole-output-turn difference refuses; software never folds,
unwraps or chooses a nearest representative.

The HM35 descriptor writes `607C = 0`. The other vendor-confirmed writes, waits
and ordering are unchanged. Only validated CompleteHome binds the raw and
command origins, and every migrated axis needs a fresh operator HM35 because
the tracking mode, source version and Home policy change coordinate identity.
Every rotation-mode and mode-3 anchor therefore refuses rather than being
silently upgraded.

Rotation mode, wrapped feedback, wrapped commands, `write_sdo_wrap_fraction`,
batteryless single-turn mode 3, `single_turn_frame_offset` and the public
`feedback_wrap`, `command_wrap` and `anchor_period_counts` fields are retired.
The old 2026-09-15 mode-3 captures and the J5 rotation-wrap abort remain useful
historical evidence of the paths being removed; they are not current
qualification for linear mode.

`encoder_battery` remains a sourced robot hardware fact, but it no longer
selects mode 3 or 4. A declared absent battery may run linear mode while
continuously powered, but it cannot establish power-off turn retention:
coordinate identity records the absence, retained-Home restoration is
unavailable and every power loss requires operator HM35. The single-axis mini
bench uses this powered-only posture. Its drive profile admits only the measured
ALF9.0 `7305/0F90` pair, only with a current 0x203F receipt and only when the
robot declares the battery absent. It remains visible as a warning. Er20.8,
Er20.9, ErA0.1, stale/invalid receipts and every other pair remain faults.

Before merge, the mini bench must prove at servo rate: HM35 yields `6064 = 0`;
single +719 degree to -719 degree moves and the return are monotonic with total
travel equal to net travel and no oversized cycle step; reversal after
accumulated turns is continuous; and Stop is bounded. Power-cycle retention
and battery-removal qualification remain separate work after a battery box is
fitted. None of this
hardware evidence exists yet. See P8-45 and
issue #160 (internal tracker)
for the unrelated remaining hard-coded A6-EC literals.

For Er20.8 (`603F=7305`, `203F=0208`) during linear-mode qualification or
after fitting/replacing a battery, inhibit physical motion, stop the
daemon/master owner and prevent its automatic restart. Keep the selected drive
in PREOP and run:

```sh
build/rtctl recover-encoder --slave 0 --backend ethercat
```

This is an explicit host maintenance command, not a daemon startup action.
It selects master 0, checks A6 identity, idle/inactive master state and disabled
DS402 state, and repeats these guards before each write. The master phase check
follows [IgH's master command](https://gitlab.com/etherlab.org/ethercat/-/raw/stable-1.6/tool/CommandMaster.cpp).
Prevent competing maintenance tools or daemon startup throughout; CLI checks
do not reserve the master atomically.

| Step | Object and value | Required observation before proceeding |
|---|---|---|
| Encoder fault and multi-turn reset, F31.10 | `0x2031:11`, U16 = 4 | Wait 500 ms (unverified settle bound); read both alarms: Er20.8 or ALF9.0 only; reset result must be 0 or 4, never 16 (operation failed). |
| Software reset, F31.01 | `0x2031:02`, U16 = 1 | Poll PREOP and both alarms until ALF9.0 (`7305/0F90`) or clear (`0/0`); retained Er20.8 may converge within the bound. |
| Fault reset, F31.00 | `0x2031:01`, U16 = 1 | Read both alarms until `0/0`; only ALF9.0 may precede clear. |

Polling uses 100 ms cadence and a 10 s convergence limit, both unverified
hardware budgets. Unexpected states, values, unreadable objects or a master
owner stop the procedure before further writes. Unknown post-reset states
require operator diagnosis; writes are not retried. Zero `603F` alone cannot
establish success while `203F` remains nonzero.

**Home is required after encoder data reset, even if a later step fails. Do not
restore an old Home anchor.** Restart with the corrected definition, complete
drive Home and save a fresh software anchor before Enable. Section 5.5 describes
Er208 on first battery connection/replacement and ALF90 below 3.0 V. No
linear-mode axis tolerates either battery warning as motion-ready; Er20.8
remains faulted. Software/mock tests do not qualify these recovery bounds on
hardware.

## Operator reset of a drive alarm (2026-10-11)

Compiled as `drive_alarm_reset=` on every axis whose drive profile lists
`fault_reset`. Not yet exercised by an operator on hardware: the first Er41.1
alarms seen on Rosie 1430 cells (J5 and J2) cleared in a controller restart's
bring-up. OLP's Arm
(`4ed9e029`) sends this Reset first for a resettable alarm.

An A6-EC in CiA402 Fault (statusword bit 3) leaves it only on the CiA402 fault
reset: controlword bit 7 rising (transition 15, to Switch On Disabled). The
fault reset does not enable the drive, release the brake or change 607C, and the
drive accepts it only once the alarm's condition has cleared (a hot motor stays
in Er41.x until its thermal model cools). Before this change rt-core never sent
that edge to a drive whose Fault was latched as `drive_readiness`, so a Rosie 1430 cell's
J5 Er41.1 could be cleared only by restarting rt-core (the restart cleared it
at bring-up by a mechanism not yet confirmed).

Now Reset (OLP Clear faults, the Deck, the one-press recovery) sends the edge
itself, disarmed only, to each axis in Fault whose alarm `config/drives/a6ec.json`
`fault_reset` lists by its exact 0x603F/0x203F pair (46 of the manual's
chapter 10 faults; Er41.1 is `0x7121`/`0x0411`), and judges the press on the
drive's answer: cleared when the drive leaves Fault and stays out, otherwise
`fault_persists: J5 drive alarm Er41.1 persists after reset: Motor
over-temperature due to locked- rotor`. Not reset by rt-core: every fault the
manual marks non-resettable (encoder Er20.x, overcurrent Er10.x, EEPROM/ESI
Er32.x and the rest: power-cycle and service the drive as the manual says),
ErA0.1 multi-turn overflow and Er84.3 Home position setting error (position
reference: HM35 at the mechanical home pose), the reserved Er81.2, any alarm
whose 0x203F detail rt-core has not read, and the ALF warnings, which do not
put the drive in Fault. The manual's class column is not a reset criterion (it
is printed beside both kinds). Attempts on one axis are at least
`min_interval_ms` (1000) apart, and the drive gets `leave_fault_ms` (200) to
leave Fault; both unverified. Read the measured time from the first live
reset's `drive_alarm_reset` journal line and record it in the profile. Contract:
`docs/CONTRACTS.md` "Operator reset of resettable drive alarms".

This is the drive's own recovery through the standard CiA402 path, not a
workaround: nothing widens a tolerance, invalidates or re-establishes Home, or
touches the HM35 transaction, and the mechanical cause of a locked-rotor stall
must still be found before arming.

## Brake hold and Home after Stop

The six Rosie 1420 bench motors **have brakes** (corrected 2026-09-16);
retain the `a6ec` profile. `BrakeMachine` commands no DO: urgent Stop inhibits
6040 immediately, and the drive manages its brake. The profile has no mapped
released signal, so no input/output/statusword bit confirms engagement;
APPLIED/RELEASED are timer estimates. `DisabledBrakeGuard` faults when fresh
6064 moves more than 16 counts from the last enabled PDO during its 100000000 ns
hold window (or on a gravity axis). In `bd-b755b-move-a0-brake-hold-fault.jsonl.gz`,
axis 0 is 5927779 at 284504600322794 ns, 0x9637 (Operation Enabled), then
5927746 at 284504701162410 ns, 0x9650 (Switch On Disabled), and settles at
5927767; bit 10 latches at the intervening Stop. This 33-count sampled excursion
is consistent with the guard, not a missing-bit timeout. The supplied daemon
tail ends after commissioning and contains no Stop trip sample or brake-I/O
readback; it cannot distinguish wiring from physical timing or establish the
exact last-enabled count. Check the [wiring diagram](brake-wiring-diagram.md):
CN1 DO3+ pin 5 to Finder A2, DO3- pin 4 to external 0 V, Finder A1 to external
+24 V, COM11/NO14 switching fused +24 V to BK+, and BK- to 0 V; verify the
DO3 brake assignment/polarity and C05.10..13. [Chapter 11](a6ec_manual_chapter_11_parameter_list.md)
defines C05.10 as brake-close to de-energize delay, C05.11 as closing speed
threshold, C05.12 as S-ON-off closing wait, and C05.13 as release-to-command
delay; these do not prove the host's post-disable window is qualified. No
threshold is relaxed. Native metrics/logs now retain the first trip's timestamp,
anchor/feedback/delta counts, tolerance/window, statusword and released-signal
availability. Brake faults still inhibit and require restart, but coordinate
observation continues and Home survives trustworthy motion; a contradiction or
lost position proof still removes it. `anchor_disagrees` describes availability
of the saved restore pose, which a legitimate trajectory can leave behind,
not validity of live Home. Explicit restore at a different pose still refuses.
The six startup `drive.observed_identity` messages precede mailbox readbacks:
the status publisher compared their initial invalid revision/serial caches
against stored records. Pending identity now makes the restore offer unavailable;
only acquired mismatches diagnose replacement, and explicit restore keeps the
same revision/serial checks. All machine/configuration/deployment digests remain
unchanged; physical brake/wiring qualification remains open.

## Fixed PDO assignment ownership

A6-EC uses fixed presets: RxPDO 0x1702, TxPDO 0x1B02 (digital inputs) or 0x1B04
(velocity and following error). With `explicit_pdo_config=false`, the daemon
observes PREOP, saves the original single assignment in each direction and changes
it only when necessary: `0x1C12:00=0`, `0x1C12:01=0x1702`, `0x1C12:00=1`
for RxPDO and `0x1C13:00=0`, `0x1C13:01=<preset>`, `0x1C13:00=1` for TxPDO.
A factory RxPDO assignment of 0x1701 is replaced by the profile's 0x1702 without
operator SDO writes. Both directions use the same automatic rescan and optional
restoration policy; an already-selected RxPDO adds no log bytes. It verifies
both assignments, counts and every mapped entry before registration or activation.
Unavailable PREOP reads have three bounded attempts; a complete mismatch refuses
bring-up. IgH receives sync-manager direction/watchdog settings without fixed PDO
entry lists. Its configuration FSM can still attempt mapping writes when the SII
advertises support, as recorded by the existing robot mapping notice.

For a preset hand-off, inhibit motion using the external stop/STO chain and keep
the control application disarmed. Stop the prior master owner. If slaves are
stuck, request INIT and then PREOP with the EtherCAT CLI and verify the observed
states before launching the daemon. Keep the profile setting
`ethercat.restore_assignment_on_exit=false` (the default). Launch once with the
selected preset. A physical/cache difference triggers automatic rescan: the daemon
releases the inactive master, queues `EC_IOCTL_MASTER_RESCAN` on `/dev/EtherCAT0`,
observes scan completion and every scanned layout, reacquires the master, rebuilds
configurations and physically re-verifies before activation. It logs slave counts
and each AL state during the hand-off. The existing `preop_to_safeop_timeout_ms`
bounds scan observation; a failed scan, reacquisition or re-verification refuses
with `rescan_required=1 activation=blocked`. There is at most one automatic rescan
per launch; no timeout grants activation.

After activation, every slave must be OP with complete current exchanges:
`operational=9/9` and `domain_wc=2 wkc=18/18` for the nine-axis group.
`startup_ready` must then hold continuously for **500,000,000 ns** before any
userspace SDO, including mapping, startup/diagnostic readbacks, rotation sampling
and native Home. This is the GradientOS startup mailbox settle window; startup
descriptor readbacks retain five bounded attempts. Each request rechecks admission
and discards its reply if readiness or the configuration epoch changes. Either
change restarts the settle interval. The
pre-activation PREOP assignment transaction is a separate mailbox phase.

For a successful 0x1B04 launch from a stale 0x1B02 cache, expect these log fragments
in order (axis lines repeat for each affected slave):

```text
EtherCAT config phase=slave_config_pdos axis=0 slave_pos=0 profile=0x1702/0x1b04 mode=fixed_default readback=PREOP_required
EtherCAT fixed_mapping axis=0 slave_pos=0 tx_pdo=0x1b04 assignment=written phase=PREOP readback=required
EtherCAT mapping_readback axis=0 phase=PREOP result=verified attempts=1
EtherCAT phase=pdo_rescan state=released master=0 restoration=off
EtherCAT phase=pdo_rescan state=queued master=0
EtherCAT phase=pdo_rescan state=observed slaves=9/9 al_states=0x2 scan_busy=0 mapping=verified
EtherCAT phase=pdo_rescan state=reacquired master=0 rebuild=required readback=PREOP_required
EtherCAT mapping_readback axis=0 phase=PREOP result=verified attempts=1
IgH libecrt active (num_axes=9, expected_wkc=18)
```

Next, `EtherCAT startup converged` must show `operational=9/9` and
`domain_wc=2 wkc=18/18`. After the continuous settle window, expect
`EtherCAT startup readback begin key=... delay_ms=500`, per-axis `verified=1`,
and `EtherCAT mapping_readback axis=... result=verified`. A cache already matching
the physical layout skips rescan. The robot 0x1B02 profile retains its original
`readback=pending` and IgH mapping-notice lines; added PREOP phase records describe
the extra verification.

Test restoration separately with motion inhibited. Opting into
`ethercat.restore_assignment_on_exit=true` restores and reads back the saved
assignment only after deactivation and observed PREOP, including failed bring-up.
Expect `phase=PREOP restore=verified preset=... attempts=... rescan_required=1`;
`restore=failed` is not success. This flag must stay off during a preset hand-off:
a cache difference with restoration enabled refuses before releasing the master.
Restoration can leave the scan cache stale; the next normal launch with restoration
off performs the same automatic check/rescan. Bench confirmation remains pending;
follow [bench qualification](../../BENCH_TESTING.md) before motion.
