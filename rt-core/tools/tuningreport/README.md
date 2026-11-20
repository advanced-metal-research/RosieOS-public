# tuningreport: per-joint tuning and acceptance report from Stop dumps

One maintained analysis for every servo-tuning iteration, replacing earlier
ad-hoc scripts (`an`, `j4`, `accel.py`, `holds.py`, `extra.py`, `kin.py`). It reads copied Stop dumps, the cell's
configuration and optional journal text, and writes a README-style report,
`metrics.json` and PNG plots; with `--baseline` it adds a better/worse table
against the previous iteration.

It decodes on the host it runs on (the Mac or the relay). Never run it, or
anything that decodes telemetry, on the servo host: a decode there once
stalled EtherCAT for ~1 s and invalidated Home.

`inertia-plan` and `inertia` measure a joint's load inertia ratio (C00.06)
in the arm by the dual-acceleration differential: see "Inertia test" below.

## Use

```sh
# 1. copy this session's dumps off the servo host (throttled, sha-checked)
cd steamdeck/real/v4
go run . realtime-core copy-dumps --manifest rt-core/config/cells/<cell>.json --to mac
# 2. report on the Mac
cd ../../../rt-core
go run ./tools/tuningreport report \
  --dumps ~/.rosie/stop-dumps/<cell> --since 45m \
  --manifest config/cells/<cell>.json \
  --label "J4 C00.06=250" --out ~/.rosie/tuning-reports/<date>-j4-inertia \
  --baseline ~/.rosie/tuning-reports/<previous>/metrics.json
```

- `--dumps DIR`: `*.bin` dumps and `*.trfx.gz` fixtures; `*.log` files there
  are read as journals. `--since` keeps only dumps the servo host named after
  that time (a duration back from now, or `2026-10-08T10:30` in `--tz`), so one
  iteration is not mixed with older sessions in the same `dump_dir`.
- `--manifest`: the cell manifest; its `rt_core.machine_config` is compiled
  in-process by rtctl's compiler against the checkout's drive profiles and
  robot description, as `realtime-core config` does (rtctl's compiler builds
  on darwin), and the report notes when the result is not the cell's
  `configuration_sha256` pin. `--machine` compiles a machine config directly;
  `--compiled DIR` reads an `rtctl compile --out` directory instead (named by
  its configuration sha256, e.g. the deployed one; the tests use a frozen one).
- `--journal FILE` (repeatable): OLP and RevPi journal text, as
  `journalctl -o short-iso-precise` (zone in the line) or the short format
  (zone found by matching). OLP `cartesian path hold: begin|end` lines name
  each hold's frame, axis and speed; RevPi lines carrying `observed_ns` pin the
  clock and supply `target_lead` warnings and `EtherCAT startup observe`
  values. Without journals the report still runs: holds are inferred and the
  clock comes from the dump file names.
- `--tz` (default the analysing host's zone) for printed times; `--no-plots`.
- `extract --dump X.bin --from-s A --to-s B --out F.trfx.gz` cuts a fixture
  (all columns, raw counts) for tests or to share one event.

Compare iterations only when they ran the same test program (the standard
short test program in `docs/drives/a6ec/README.md`, "Tuning a joint", also
in `steamdeck/real/v4/README.md`, "Tune each joint"): the delta table
cannot tell a gain change from a different motion. A change within 2 % is
"same".

## What it measures

Units: degrees, ms, deg/s, deg/s^2, rad/s^3 (jerk, as `jog_jerk` is
configured), mm for the TCP. Every acceptance bound is the compiled
configuration's, read back with `rtctl.CompiledAxes`: `tracking`
(following error, target lead, completion tolerance), `limits` (velocity),
`max_acceleration`, `tracking_lag_allowance`, `jog_jerk`, and the count
conversion (counts per rev, gear, sign); the drive-settings column is the
compiled startup SDOs. Kinematics come from the robot description the
configuration pins; no machine number is in the code. Detection settings (minimum speeds,
window lengths) are listed under "Method notes" in each report and in
`metrics.json` `params`; the defaults are those of the published analyses.

Per joint (armed and enabled cycles):

- Lag: least-squares slope of following error on commanded speed (with R^2),
  the shift minimising |actual(t) - commanded(t - shift)|, and the median of
  error / speed above 5 deg/s. Following error is commanded[t-1] - actual[t],
  so a pure delay of N cycles fits N - 1 ms (the published 14.9 ms fits vs
  16 ms shifts).
- Following error against the scaled bound following_error + |v_cmd| x
  allowance: peak, max ratio and when, longest and total time over; the
  drive's own 0x60F4 peak.
- Target lead (commanded[t] - actual[t], native's check) against
  max_target_lead + |v_cmd| x allowance: excursions (gaps <= 2 cycles merge),
  peak, longest.
- Acceleration: commanded peak (1-cycle) and its share of the limit; actual
  peak from velocity_actual (20-cycle mean, 10-cycle difference) and the
  actual/commanded ratio (only when commanded >= 0.1 x limit).
- Commanded jerk: 1-cycle second difference and the 20 ms window (centred
  10-cycle second difference, where one-cycle sample-timing spikes cancel),
  against `jog_jerk`.
- Velocity overshoot at starts: smoothed actual beyond a commanded plateau
  (>= 0.25 x the joint's velocity limit, constant 100 ms) within 300 ms.
- Stops (commanded speed to 0 for 300 ms after >= 3 deg/s): residual at the
  stop, overshoot past the target, settle time to completion_tolerance.
- Resonance on the lag-compensated residual actual(t) - commanded(t - lag):
  episodes above 3 x completion_tolerance; per episode peak, settle, frequency
  from the half cycles above the noise floor (median half period), a DFT peak,
  damping ratio from the free-decay half-cycle amplitudes (after the last
  re-excitation), the joint that excited it and peak per unit of its
  acceleration; a Welch spectrum of the whole residual. A joint whose lag fit
  is not credible (R^2 < 0.8: it only rang) takes the other joints' median lag.

Per Cartesian path hold (OLP lines when present, else every multi-joint jog
lane, measured off its start-to-stop chord, a lower bound): commanded TCP off
the line (max/RMS) and orientation drift, actual off the line at steady speed
and over the hold plus 800 ms, stop speed, distance and time from the cycle
the lane closed, joint stop spread, peak acceleration and jerk, worst FE/bound
per joint. Rotational holds report TCP drift and rotation off the axis.
Kinematics are `rt-core/cartesian`: the resolver's
`urdf_forward_kinematics_j1_base` (TCP = the robot's tool frame) in Go,
matching the compiled `robot-v4-cartesiand --resolve-only` to 8.9e-16 on
20,000 random poses (`ROSIE_CARTESIAN_RESOLVER=<binary> go test
./cartesian` re-checks it). Asking the resolver per pose instead would add
3.6-8.7 s to a 2.1 s report (105,225 poses at 15:33). Off-line and
orientation figures are `cartesian.LineDeviation`/`DeviationAlong`, the
definition the OLP hold's monitor and the cartesianjog-harness use.

Per Move-to (active mode 2): duration, joint deltas, peak speed, acceleration
and jerk per joint, joint-space straightness (deviation from the straight
segment, deg and counts), lag-compensated residual and FE/bound per joint
until the next motion, settle after the end.

Cycle cost by state (jog lane, Move-to, armed idle, disarmed): work and
|jitter| percentiles, overruns (work > period), short WKC, sequence gaps,
fault masks.

### Servo loop quality (every joint, up to 500 Hz)

What each drive's loops do above the arm's modes, from the 1 kHz record:
torque 0x6077 (`TorqueRaw`, per mille of rated), velocity 0x606C (deg/s)
and the lag-compensated residual (mdeg); band values are RMS. It exists
because the 1-30 Hz position analysis above cannot see a speed loop
oscillating: on a Rosie 1430 cell J4 at C00.06 = 700 % ran a 228 Hz torque
line after arming and J2 broke into a 100-200 Hz oscillation of +-2000
permille while moving, both found only by scratch spectra. Nyquist is 500 Hz and the PDOs are sampled with no anti-alias
filter: content above 500 Hz folds to 1000 - f.

- States (armed, every joint enabled): armed idle, jog/hold moving, Move-to,
  stop (200 ms before commanded motion ends to `stop_window_ms` after or the
  next motion). Per joint per state: torque, velocity and residual RMS in
  1-30, 30-80, 80-300 and 300-500 Hz, the loop band (80-500 Hz) torque RMS,
  its strongest bin and spread (the width holding the middle half of the
  band's power: a few Hz for a line, tens for a broad oscillation), the
  absolute torque peak. Welch, 512-cycle Hann segments, trend removed;
  Parseval-scaled so a sinusoid of amplitude A reads A / sqrt(2) in any band
  holding it, zero-padded or not.
- Sharp lines >= 30 Hz per state (>= 4 segments): peak power >= 8 x the
  median of the +-25 bins around it, amplitude (sinusoid peak) >= 1 permille,
  Q = f / half-power width (marked `>=` when the 1.95 Hz bins limit it), a
  60 Hz-harmonic flag, and coupling: magnitude-squared coherence with every
  other joint's torque over the same segments (>= 8). A line coherent with
  another joint (>= 0.5) is "carried from" the joint where it is strongest;
  one coherent with none is that joint's own loop.
- Timeline: a 256-cycle Hann window every 100 ms (3.9 Hz bins) where the arm
  is armed throughout; per joint the band RMS, spectrum, own speed, position
  and torque level. Written as `servo-timeline.csv`.
- Buzz episodes: loop-band RMS of a window >= 8 permille with the joint
  commanded still, >= 80 moving (`params.servo`; the measurements behind
  both are in `defaultServoParams`, UNVERIFIED as limits), >= 200 ms, gaps
  <= 2 s merged. Each has onset and offset (window centres, +-128 ms), on
  fraction, peak, dominant frequency, onset state, own speed and position,
  every joint's peak over the span, and the joint furthest over its own
  threshold (the source when several joints show it), and the nearest lane
  or Move-to event.
- Torque ripple at rest (armed idle, >= 1 Hz and >= 30 Hz) and the mean
  holding torque; the loop band by own commanded speed (still, < 5, 5-15,
  15-30, 30-60, >= 60 deg/s: median / p95); Spearman correlation of the loop
  band with own speed, position and torque level.
- Per path hold: band RMS per joint over the commanded motion, the loudest
  joint and its peak frequency.
- Plots: `servo-spectrogram.png` (torque, every joint, the whole session),
  `servo-bands.png` (band timeline, log scale, thresholds and episodes),
  `servo-spectra.png` (per state), `servo-worst-hold.png` (the loudest hold
  against armed idle).
- `--baseline` rows per joint: loop band per state, every band and the
  velocity above 80 Hz while moving, the residual's low band, ripple at rest,
  the worst window, buzz episodes and time, the strongest line.

## Decoding and clock

Dumps are read with `ipcclient.ReadTelemetryDump`, the decoder `rtctl
telemetry` uses, under its one compatibility rule
(`TelemetryDumpIndex.CheckCompatible`): a dump of another protocol minor is
read when its record layout digest equals this build's (1.13 and 1.14 share
it) and refused otherwise. Dumps of one run (shared sequences with identical t_ns)
merge; a sequence gap or a restart starts a new window, and nothing is
computed across one.

Record times map to UTC by, in order: RevPi journal lines with `observed_ns`
(same clock as t_ns; the smallest wall - observed difference, so times may
read late by the journal write latency), OLP hold-begin lines matched to lane
openings (+-60 ms), the dump file name (written 53-63 ms after the trigger:
+-30 ms), or `--clock-offset-ns`.

## Validation

`go test ./tools/tuningreport` runs synthetic signals with known delay
(15 cycles: shift 15 ms, fit 14 ms), jerk (150 rad/s^3), a 6.0 Hz ring with
damping 0.10, a 25 ms excursion over the bound, and for the servo section a
20 permille 228 Hz torque line on a still joint (band RMS 20 / sqrt(2), line
228 Hz at 20 permille, J4 alone, one episode), the same 120 Hz line on two
joints at 10 and 4 permille (coherence > 0.95, carried from the stronger), a
300 permille 100 Hz oscillation on a joint moving at 20 deg/s from 4.0 to
7.0 s (onset and offset within 0.15 s) and a baseline where a 20 -> 5
permille line reads better, all asserted in absolute units; then five
committed fixtures (`testdata/`, 1.1 MB, extracted from a Rosie 1430 cell's
published dumps; the two of 2026-10-07 22:01 and 2026-10-08 carry torque, the
older three were cut before torque was carried and read back without it)
against the published numbers, analysed with the configuration of the day
(its `rtctl compile` output, committed beside the fixtures):

| session | published | this tool (fixture; full dump) |
|---|---|---|
| 2026-10-06 17:40, no FF | lag fit 26.46-27.03 ms; FE/bound J2 0.380, J3 0.411; lead J2 3 (0.4567 deg), J3 3 (0.4998) | fixture 26.32-27.31 ms (12 s window); full dump 26.56/26.46/26.57/26.80/27.03 exactly; FE/bound and lead identical |
| 2026-10-06 21:04, FF 50 % | lag fit 14.87-15.83 ms; J4 1.5976 deg, FE/bound 1.263, 33 ms over; 18 lead excursions, peak 1.6000; actual acc J1 905, J6 1161 deg/s^2 | full dump identical lag fits; J4 1.598 deg, 1.263, 34 cycles over (the published 33 is last - first time); 18, 1.600; 905, 1161 |
| 2026-10-07 15:33, design D | 32 holds, commanded off line 0.0002-0.0009 mm, rev 32 2.43/3.33 mm, stop 401 mm/s 24.2 mm 123 ms; Move-to J4 1.337 deg, FE/bound 1.166, 30 ms over, jerk 150, acc 713/715 | all 32 holds identical to holds.md; Move-to identical; median lag 14.1-15.1 ms identical |
| 2026-10-07 13:06 (3 dumps) | lag fit 14.0/14.2/14.3/17.6/14.8 ms; J4 0.55 deg, FE/bound 0.48 | 14.00/14.17/14.30/17.57/14.82; 0.551, 0.481 (run by hand, not in the tests) |
| 2026-10-07 22:01, J4 C00.06 700 % | scratch `hf-buzz.txt`: J4 228 Hz torque line while still, 200-260 Hz up to 20.4 permille RMS per second, seconds 53-61 of the dump, J4 only | one J4 episode 22:00:49.648-22:00:59.348 (full dump 49.647-59.347), peak 23.0 permille RMS in 80-500 Hz at 22:00:57.248, 228 Hz, J4 alone; line 228 Hz, 14.9 permille amplitude, Q 57 |
| 2026-10-08 11:28 Deck | scratch `hf-analysis.txt`: J2 oscillation ~100 Hz (80-250 Hz hump), onset "end of rev 17, 11:28:21.2", torque to +-2783 permille, J1/J3 couple in | J2 onset 11:28:20.783 (the first window of rev 17, which began 11:28:20.653: 140 permille against <= 39 on the same -X hold 4 s earlier; the burst at its stop, 618, is the published onset), peak 1098 permille RMS at 11:28:23.883, J1 and J3 episodes carried from J2; abs peak 2466 in the fixture, 2805 over the whole dump |

An opt-in `TestFullDumps` re-checks the whole-dump numbers while the
original dump directories exist. Moving to ipcclient's decoder, rtctl's compiled bounds and `rt-core/cartesian`
(2026-10-07) left every scalar and hold of the three full-dump reports'
`metrics.json` bit-identical to the previous build's.

## Limitations

- Inferred holds (no OLP line) measure off the start-to-stop chord, which
  understates a velocity hold's drift against its true axis (21:04: chord
  5.7 mm vs 45 mm off the logged axis on the 498 mm hold).
- The whole-window spectrum needs several 8 s segments; on a short window the
  forced response dominates it (21:04 fixture: 3.8 Hz, full dump 5.2 Hz). The
  episode frequency is the mode estimate.
- The ring section's spectra stop at 30 Hz; above that, read "Servo loop
  quality". Its spectra stop at Nyquist (500 Hz) and are aliased above it.
- Buzz onsets and offsets are timeline window centres (+-128 ms); the
  thresholds are detection settings from three Rosie 1430 sessions, not
  acceptance limits. Moving windows grow with speed even when quiet (J2
  quiet: 11 permille at 5-15 deg/s, 26 at 30-60, 2026-10-07 22:29): compare
  the speed-bin table between iterations, not one number.
- Lines are not reported for states with fewer than four segments (a short
  Move-to's transient reads as lines), nor coupling under eight.
- Tones that move with speed (gear and rotation orders, e.g. Rosie 1430 J6 jogs)
  show in the spectrogram but are not lines of any state.
- Damping is a free-decay estimate; a ring re-excited throughout (no three
  decaying half cycles above the floor) reports none, not zero.
- Residual episodes on the main joints during accelerations are transient
  tracking error, not ringing; they carry no frequency.
- The dump holds the last 100 s: copy right after each test, or the motion is
  gone.

## Inertia test (C00.06): `inertia-plan` and `inertia`

STEPPERONLINE's method (reply of 2026-10-09, `docs/drives/a6ec/ethercat/a6ec-manufacturer-notes-2026-10-09.md`)
for a joint F30.10 cannot identify in the arm (J4 at 18:1 needs >= 3000
rpm/s, about 1000 deg/s^2, above the 720 deg/s^2 limit): constant-acceleration
segments of one joint at two levels over the same speed band, mean 0x6077 in
each, J_total = (T1 - T2) / (alpha1 - alpha2) at the motor (friction cancels),
C00.06 = (J_total / J_rotor - 1) x 100 % (manual PDF p.234: 0 = no load).

```sh
# 1. the program, on the Mac, from the held pose OLP POS shows (every joint, 0.001 deg)
go run ./tools/tuningreport inertia-plan --manifest config/cells/<cell>.json \
  --joint J4 --accels 200,400 --start-deg 0,0,0,0,30,0 [--center-deg 0] \
  --motors config/motors/<cell motors>.json --out ~/.rosie/inertia-plans/<date>-j4
# -> inertia-j4-<digest>.rdt, plan.json, CHECKLIST.md (the operator's page)
# 2. run it from OLP: Machine panel, "Load test program (.rdt)…" (see "Running it" below)
# 3. after the run, Stop, copy-dumps:
go run ./tools/tuningreport inertia --dumps ~/.rosie/stop-dumps/<cell> --since 30m \
  --manifest config/cells/<cell>.json --joint J4 \
  --motors config/motors/<cell motors>.json --plan ~/.rosie/inertia-plans/<date>-j4/plan.json \
  --out ~/.rosie/tuning-reports/<date>-j4-inertia
```

### The program (`inertia-plan`)

A `robot.v4.dense-joint-trajectory.v1` program (10 ms grid, the planner's
WIRE_DT): only the tested joint moves, every other joint holds its start
angle. Lead dwell 1 s, a move to the low end of one position window, then
per level (slowest first) `--repeats` (default 4) there-and-back moves across
the whole window with 0.5 s dwells, the move back to the start, tail dwell
1 s. The window is centred on `--center-deg` (default the start) and is as
wide as the slowest level needs to reach the peak plus 0.2 s of cruise, so
every level sweeps the same positions in both directions and the cruise
segments give friction without acceleration.

Every bound is the compiled configuration's: levels <= `max_acceleration`,
jerk = `jog_jerk` (a higher `--jerk` is refused), peak speed `--peak-speed-frac`
(default 0.75, OLP Move-to's measured drive reserve: A6-EC feedback peaked
12.7 % above plan on 2026-09-14) x `max_velocity`, the window
`--limit-margin-deg` (2) inside the position limits, the program <= 90 s (a
Stop dump holds 100 s). Every jerk piece is a whole number of 10 ms steps:
native execution interpolates the knots with cubic Hermites, which reproduce
the S-curve exactly then (off the grid a 400 deg/s^2 phase read 412.7 between
knots); rounding only lowers acceleration and jerk, and a level whose window
moves run more than 2 % under the request is refused. A level whose constant
phase is under `--min-constant-accel-s` (0.1 s) is refused with the highest
level that keeps it. On a Rosie 1430 cell (80 deg/s, 720 deg/s^2, 150 rad/s^3, peak
60 deg/s) the vendor's example 400/700 is refused: 700 deg/s^2 has no
constant phase at all on the grid (analytic 4 ms: the 81 ms jerk ramps eat
it); 400 deg/s^2 is the highest level with 100 ms, so use 200,400 (J4:
270 ms and 100 ms constant, window +-15.9 deg, 24.4 s at 4 repeats; J2 at
150,300: 380 and 160 ms, 27.2 s). Expected
J4 inertial torque at 550 %: 19 and 39 permille, a 20 permille difference,
hence the repeats.

Before writing, the program passes the checks native preparation makes
(`adapters/rosie/control` `DecodeProgram`, linux-only, repeated on the Mac):
format validation and digest (`densejointtraj.Decode`), stationary seam
normalisation, per-sample speed <= `max_velocity` and dq/dt acceleration <=
`max_acceleration` on the rounded clock, and every Hermite interval's
position extremum and peak speed through `densejointtraj.AdmitHermiteSegment`
in the zero-Home frame. The first sample must be the held command within
`completion_tolerance` (0.034 deg on a Rosie 1430 cell): native settles a smaller offset
with its quintic prefix and refuses a larger one (`start_discontinuity`), so
regenerate from POS if the arm moved.

The header names what the program was checked against, and the cell refuses
it on any difference: `robot_cell` (model, robot description, planning
calibration and `machine_configuration_sha256` = the compiled
configuration's digest; rt-core's codec has no such field, so it is added
to the encoded header JSON, outside the trajectory digest, moving no sample
byte) and `manifest_revision` = the cell's control binding revision, which
native preparation requires: the manifest's `rt_core.pair_revision` (e.g. 1),
or `--manifest-revision` with `--machine`/`--compiled`, refused when
neither names one.

**Running it: OLP's "Load test program (.rdt)…"** (owner approval for the
motion still applies). The program runs through OLP's ordinary dense Load
and Play, so OLP keeps the grant, the heartbeat deadman and STOP, and native
program execution applies every limit, following-error fault and Stop. Do not
use `rtctl program-prepare`/`program-start`: it holds its own grant beside
OLP, where OLP's STOP cannot reach. The cell's OLP must have test programs
on (manifest `cell_deploy.olp.test_programs`; offline-programming
README, "Test programs").

1. Generate from the held pose (OLP POS, every joint) with the cell
   manifest, as above. Copy the output directory's `.rdt` and `plan.json`
   to the Mac running the OLP UI (they are generated there already when the
   Mac ran the command).
2. In OLP: connect, Home valid, Arm. Machine panel, Run row: **Load test
   program (.rdt)…**, choose the `.rdt` and `plan.json` together. The OLP
   checks the robot, configuration, binding revision, axes and that every
   joint's measured position is within its `completion_tolerance` of the
   first sample, then loads it, labelled `TEST PROGRAM, not from the
   planner` with the file, joint, levels, window, trajectory and file
   digests and the generator. A refusal names the reason
   (`test_program_start_mismatch` names the joint and both angles: move the
   arm or regenerate; `test_program_configuration_mismatch`: regenerate
   against the configuration the cell runs). Fix the cause, never the check.
3. **▶ Play test program**, hand on the hardware E-stop. Press STOP within
   10 s of the end (the Stop dump holds 100 s), then Disarm, note the time,
   copy the dumps and run `inertia` (step 3 of the commands above).
4. To repeat, Load it again (a native program is single-use).

The composed test that runs a generated J4 program through all of this on
the Rosie 1430 simulation, including Stop mid-run and the heartbeat deadman, is
`offline-programming/v1/dense_test_program_composed_test.go`.

Gravity (J2, J3, J5 always; J4/J6 when the wrist is not straight): centre the
window where the distal link is horizontal, where gravity torque is at its
extremum and changes least across the window. The analysis fits it anyway:
with the other joints still, one joint's gravity torque is exactly
A cos q + B sin q, fitted as c0 + g1 dq + g2 dq^2 (third-order error in the
window); the differentials subtract that fit (raw values are reported too).

### The analysis (`inertia`)

- Segments: the joint's commanded acceleration (centred difference of the
  1 kHz target velocity) constant within 1 deg/s^2 or 1 % for >= 60 ms,
  other joints commanded under 0.01 deg/s, every joint enabled, torque valid;
  the first 30 ms after each jerk phase dropped (about twice the 14-16 ms
  position-loop lag with feedforward on); acceleration segments >= 20
  deg/s^2, cruise segments >= 2 deg/s. The regressor is the actual motor
  acceleration (least-squares slope of 0x606C over the segment), motor side
  = joint x gear x sign. All settings are printed under "Method notes".
- Fit: weighted least squares over segment means (weight = duration),
  torque = J' alpha_m + Fc mean sign(v) + Bv v_m + c0 + g1 dq + g2 dq^2, with
  95 % intervals from the residuals (Student t). Terms the data cannot
  separate are left out and named (one direction: no Coulomb term; a window
  under 1 deg: no slope; under 5 deg: no curvature). A confidently negative
  slope means 0x6077 reads opposite to the counts on that joint: the torque
  is flipped and the report says so.
- Differentials: `vendor` pairs (same direction and phase, two levels, the
  common speed band) and `up/down` pairs (one level's speed-up against its
  slow-down), each raw and gravity-corrected; the mean of pairs with a t
  interval.
- With fewer than two constant-acceleration levels (ordinary jogs: a Rosie 1430 cell's
  720 deg/s^2 at 150 rad/s^3 has no constant phase below 60 deg/s), whole
  speed changes carry the fit: mean torque over a ramp from steady speed to
  steady speed (+30 ms for the drive to catch up) = J dv/T + mean friction +
  mean gravity, whatever the ramp's shape.
- `--coupled` (opportunistic only) also admits speed changes while other
  joints move and fits their mean accelerations and positions as constant
  nuisance terms (coupling inertia, gravity). It ignores velocity-product
  torques and pose dependence: trust a joint only if it repeats across
  sessions. `--still-deg-s` relaxes "other joints still".
- Output: `README.md` (result table: J_total motor side, load motor and
  joint side, C00.06 with 95 % interval per method; fit terms; pairs;
  segments with residuals), `inertia.json`, `inertia-fit.png` (torque minus
  friction, gravity and offset against motor acceleration, with the slope;
  residuals), `inertia-timeline.png` (torque against the model per window).

Motor data: `--motors` names a `rosie.joint-motors.v1` file in
`rt-core/config/motors` and its `rosie.motor-catalog.v1` catalog. The
catalog's rated torque and rotor inertia are the A6-EC manual's chapter 12
tables (PDF p.264-266; "AS715N" is the drive's EtherCAT name, the motors are
A6M60/A6M80). A cell's joint-to-motor assignment may be UNVERIFIED (Rosie 1400 CAD:
J1-J3 A6M80-1000H2B1-M17, J4-J6 A6M60-400H2B1-M17; replaced motors
whose models were not recorded) and every report says so. The intervals exclude the
nameplate values' own error.

### Validation (`go test ./tools/tuningreport -run 'Inertia|StudentT'`)

The designed program goes through native-style cubic Hermite interpolation
at 1 kHz into a synthetic drive: torque from a known inertia, Coulomb and
viscous friction, an exact cosine gravity torque, white noise and a 50 Hz
hum, quantised to whole permille, actual position 15 cycles behind:

| case | truth | recovered |
|---|---|---|
| J4 plan 200/400, C00.06 550 %, Coulomb 25, viscous 0.3, gravity 40 permille, noise 4 + 2 | 550 % | fit 555.8 +- 7.0 % (J_total within 2 %, truth inside the interval; asserted within 15 points); vendor 551.3 +- 27.5 (within 40); up/down 546.3 +- 7.7 |
| J2 plan 150/300, C00.06 1000 %, gravity 600 permille with the window 40 deg off its extremum, Coulomb 60, noise 8 | 1000 % | fit 999.9 +- 1.6 % (asserted within 25); vendor 998.5 +- 2.9 |
| J4 jogs at one level (720 deg/s^2, 150 rad/s^3) to 10/30/60 deg/s, no designed test | 400 % | ramp mode 390.4 +- 28.6 % (asserted within 40) |

`TestInertiaPlanLimits` asserts the program's peak speed (<= 60 deg/s),
interpolated acceleration (392-400 deg/s^2, equal to the plan's stated level),
the 50 ms / 100 ms grid phases and the refusals (700 deg/s^2: 0 ms constant,
highest 400; 800 deg/s^2 over 720; jerk 151 over 150).

### Retrospective on existing dumps (OPPORTUNISTIC)

Dumps whose last 100 s are Cartesian holds and Move-tos hold no single-joint
motion (the "joint alone" parts of the test program are in dumps that must be
copied right after them), so no J4 number can be had from them. With
`--coupled` (every joint's speed changes during holds, coupling fitted as
constants), pooled over six sessions of a Rosie 1430 cell: J2 repeated
(752 +- 52 %, near its 800 % trial) and J3 roughly repeated (296 +- 49 %); J1
varied with reach (pose-dependent, not one number); J4 and J5 were
uninformative; J6 came out below the rotor inertia (impossible: the coupled
model is biased on the wrist, or the joint's motor is not the assumed one).

None of these replaces the designed test.
