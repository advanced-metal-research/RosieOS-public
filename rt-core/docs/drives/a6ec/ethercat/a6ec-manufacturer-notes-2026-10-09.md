# A6-EC Manufacturer Notes 2026-10-09

Status: vendor correspondence note on gain tuning, vibration suppression and EEPROM writes. Like the [2026-04-15 notes](a6ec-manufacturer-notes-2026-04-15.md) this is source material, not the project's SOP: the procedure is "Tuning a joint" in [`../README.md`](../README.md) and the tuning catalog is `rt-core/config/drives/a6ec.tuning.json`. Read it before changing C00.04/C00.05/C00.06, C02.38-C02.4E, the notches or how startup SDOs reach EEPROM.

## Source

- Reply: STEPPERONLINE technical support, signed by a support engineer, from `support@omc-stepperonline.com` to `operator@example.com`, 2026-10-09 00:00:35 PDT (07:00:35Z), subject "[#7821] New messages on this conversation". Gmail thread `1a11b7f242409d49`, message `1a11f76c7c2b8dc6`. It arrived on the helpdesk ticket #7821 thread, not in the April A6-EC thread.
- Our questions: sent 2026-10-08 01:18:35 PDT (08:18:35Z) from `operator@example.com` to the ticket alias `support@omc-stepperonline.com`, in the April A6-EC thread `19d8a4521e3a4cd4` (message `1a11a97c04dc89fb`). The helpdesk's out-of-office acknowledgement (2026-10-08 05:31 PDT) opened ticket #7821 on thread `1a11b7f242409d49`.
- Both read through Gmail, read-only, 2026-10-09. The reply is reproduced verbatim at the end.

## Our Setup As Stated In The Question

- ANCTL AS715N / A6-EC, ESI `STEPPERONLINE_A6_Servo_V0.02` (vendor 0x00400000, product 0x00000715, revision 0x00002EF8), 17-bit encoders (131072 counts per motor revolution).
- CSP (`6060 = 8`), 1 kHz, DC sync, IgH master. J4 18:1, J1-J3, J5, J6 100:1.
- Read back on every axis: `C00.04 = 1` (standard), `C00.05 = 12`, `C01.00-C01.03 = 400 / 250 / 3184 / 200`; speed feedforward `C01.13 = 1`, `C01.14 = 500` (50 %), `C01.15 = 318 Hz`.
- Drive-held `C00.06` = 442 / 1143 / 337 / 190 / 35 / 227 % on J1-J6.
- Problem: with J4 commanded still, J1/J6 accelerating at about 700 deg/s^2 make J4 oscillate at about 6.4 Hz, damping ratio about 0.05-0.1, peak about 1.3 deg at the output and 24 deg at the motor, settling in 0.6-0.9 s. Only J4.
- J4 `C00.06` 190 -> 400 % (SDO at startup, read back): 6.4 -> 7.35 Hz, damping ratio about 0.05 -> about 0.18, speed overshoot at motion start 39 -> 2 %. `C01.00-C01.03` still read 400 / 250 / 3184 / 200. J4's lag fit went 16.2 -> 14.6 ms.

## Confidence Marks Used Below

- **Stated**: the vendor gives it as fact or as their recommendation.
- **Inferred**: the vendor says it is reasoning, an analogy, a convention or a likelihood; their words are quoted.
- **Unconfirmed**: the vendor says they are not certain or that an FAE must confirm.
- **Not answered**: the question had no answer in the reply.
- **Ours**: our own note against the repo, not the vendor's.

## 0. Vibration Mechanism (Vendor Preface)

- Stated (vendor analysis): J4 is the only 18:1 axis; the load inertia it reflects to the motor is about 31 times that of a 100:1 axis ((100/18)^2), so J4 has the smallest gain margin.
- J1/J6 acceleration reaction torques couple through the base into J4; J4's position loop counters them and a weakly damped 6.4 Hz mode is excited.
- Motor-side 24 deg over output-side 1.3 deg is about 18.5, the reduction ratio: the axis oscillates as a rigid unit. Not an output-side flexible two-inertia mode (the motor side would stay nearly still) but "a rigid-body mode with weak damping from the position loop's restoring force".
- That explains the `C00.06` result: a more accurate inertia ratio matched standard mode's gains better, damping rose (0.05 -> 0.18), overshoot fell (39 % -> 2 %) and higher closed-loop stiffness raised the frequency (6.4 -> 7.35 Hz). "Everything is consistent."
- Sweeping up to 700 % is correct; settle the value by the response and back off if the axis becomes overdamped or overshoot reverses.
- The five notch filters work at 50-8000 Hz and cannot address 6.4 Hz. The drive has a "Low-Frequency Vibration Suppression" function that suppresses two low-frequency vibrations at the end-effector simultaneously; the vendor points to manual section 7.14 "Vibration Suppression".
- S-curve profiling on J1/J6 limits jerk and broadband excitation.
- Ours: the catalog records no vibration-suppression procedure in chapter 7 or 11.3 of our manual copy (7.14 covers the notches), and the reply's item 11 says no dedicated low-frequency section was found. The section pointer is unresolved.

## 1. Origin Of The Drive-Held C00.06 Values

- Asked: shipped values or an earlier `F30.10` identification stored in EEPROM? A record of the last identification (`U40.34`)?
- Inferred ("only two possibilities"): either `F30.10` ran and saved its result, or the value was written by hand. The vendor cites the A6-PN manual (another product) for `C00.06` being "automatically auto-tuned by F30.10".
- Stated (verification): run identification on one axis and read `C00.06` by SDO before and after; or restore factory defaults and see whether it returns to 100 %.
- Not answered: whether drives ship with these values; `U40.34`.

## 2. Offline Inertia Identification Under EtherCAT

- (a) Asked: can `F30.10` run in OP with the master writing `6040 = 0`, or only PREOP/INIT?
  - Inferred ("a conflict is highly likely"): the manual's panel procedure states no bus-state restriction, but panel functions such as S-ON are valid only in non-bus control modes and the drive must self-enable while the master owns `6040` in OP. Safest: PREOP, or disconnect the master.
- (b) Asked: can `F30.10` (`0x2030:11`) and the C07 parameters (`0x2007`) be written by SDO? They are not in the ESI.
  - Stated: the ESI is a subset of the object dictionary; absence from the ESI does not mean absence from the OD.
  - Stated: mapping rule `0x2000 + group` (C07 -> `0x2007`, F30 -> `0x2030`), subindex = parameter number + 1; `F30.10` -> `0x2030:11` ("your deduction was correct").
  - Stated: list the real objects and their access with `ethercat sdos`.
  - Stated (warning): if `0x2030:11` is writable and starts identification, the drive self-enables and moves; make the brake and travel safe first.

## 3. Holding Brake During Identification

- Asked: does keypad `F30.10` release the brake through the brake output (DO3) as a normal enable does?
- Stated: no. The brake does not release in sync with identification. Release it by hand (power the brake) or configure the BK output to interlock with identification; otherwise the motor stalls against the brake, faults on overcurrent and may be damaged.

## 4. F30.10 Operation Details

- (a) Stated: not press-and-hold. Set `F30.10` on the panel -> `[S]` -> `[▲]/[▼]` sets the number of identification revolutions -> `[S]` -> the drive runs multi-revolution forward and reverse by itself -> the inertia value is displayed -> `[M]` finishes.
- (b) Stated: `[M]` aborts. Stop-on-release belongs to JOG (`F30.00`), not identification.
- (c) Stated: revolutions = the set value, in both directions; travel must allow at least one full revolution each way. Ours: the reply does not say motor or load revolution (one motor revolution is 20 deg at J4's output, 3.6 deg on a 100:1 axis).
- (d) Inferred ("strongly implies"): the manual only says the value is displayed; non-default `C00.06` on all six axes implies the result is written to `C00.06` and saved.
- (e) `C07.00` bits (default 769 = 0x301): Not answered (the reply's "(e)" is empty).
- (f) Inferred ("Following standard design conventions"): `C07.02` is the time constant from 0 to the set speed `C07.01`.
- (g) `C07.01` range (50-1000 vs 50-8000 rpm): Not answered directly. Stated instead: for a 1000 W motor (80 mm frame, rated 3000 rpm, max 6000 rpm) identify at 500-2000 rpm. Ours: our question gave no motor power; the reply assumes 1000 W.

## 5. F30.10 Motion Requirements, Or Calculating C00.06

- (a) Asked: can identification run below 150 rpm / 3000 rpm/s (about 1000 deg/s^2 at J4, above our 720 deg/s^2)?
  - Stated: 3000 rpm/s / 18 is about 1000 deg/s^2 at the output, above 720. The manual's conditions (max speed > 150 rpm, accel/decel > 3000 rpm/s) are prerequisites for accuracy, not "slightly lower precision"; J4 as installed can never meet them. Disregard panel `F30.10` for J4.
  - Stated (proposed method), dual-acceleration differential measurement in our own CSP framework:
    - ① Command J4 by CSP through two constant-acceleration segments (output 400 and 700 deg/s^2, both within limits) across the same velocity range (for example 500-1500 rpm at the motor).
    - ② Record each segment's average torque over that range (`0x6077`; CiA402 unit 0.1 % of rated torque, times the motor's rated torque for N·m).
    - ③ `J_total (motor side) = (T1 - T2) / (α1 - α2)`; the difference over the same velocity range cancels friction.
    - ④ `C00.06 = (J_total / J_m - 1) × 100 %`.
    - Needs no mechanical change, stays within limits, and can check our earlier 700 % figure.
- (b) Rotor inertia of the AS715N: Not given; "available in the motor's technical drawings".

## 6. Standard Gain Mode (C00.04 = 1)

- (a) Stated: the internal formulas are not disclosed ("only one parameter needs to be set"). Structure: `C00.05` (rigidity level) picks the base gain group (speed loop Kp/Ki, position loop Kp, filters); `C00.06` scales and matches it to the equivalent inertia.
- Stated: our readback `C01.00-C01.03 = 400 / 250 / 3184 / 200` is "the derived results for Rigidity Level 12 at that specific inertia ratio"; changing `C00.06`/`C00.05` and reading back the C01 group is "the correct engineering approach" to reverse-engineer the mapping.
- Ours: that does not explain what we reported: `C01.00-C01.03` read the same at 190 % and 400 % while J4's lag and damping changed. (b) (are the effective gains hidden from the readbacks, and how to read them) and (c) (does `C00.06` change Kp) are Not answered.
- Stated (official A6 tuning procedure, posted by STEPPERONLINE on the LinuxCNC forum in June 2026): `C00.04 = 0` (manual) -> confirm the inertia ratio -> raise speed loop `C01.01` until slight noise/vibration, back off 15-20 % -> lower `C01.02` to remove steady-state error (not too low: low-speed creep) -> raise position loop `C01.00`, back off 10-15 % on overshoot -> finally feedforward and friction compensation.
- Stated: for a weakly damped, easily excited axis like J4, once the inertia ratio is accurate, tune the speed loop (`C01.01`/`C01.02`) second and the position loop last.

## 7. Recommended Tuning Approach

- Stated (short term): standard mode plus an accurate `C00.06` on every axis; standard mode's inertia scaling suits a 31x spread in reflected inertia, and the J4 190 -> 400 % result shows it works.
- Stated, 100:1 axes: verify `C00.06` (differential method, or calculated from load and ratio), then sweep `C00.05` starting from 10-15; go further only if tracking or stability targets are missed.
- Stated, J4: tune `C00.06` accurately first, keep `C00.05 = 12`, and confirm the damping ratio stays above 0.15 before considering more stiffness.
- Stated, when to go manual (`C00.04 = 0`): an axis with poor tracking error but no oscillation after `C00.06` is right, or a position-loop bandwidth that must be set independently. Then the procedure in item 6.
- Stated: a hybrid is viable: the five 100:1 axes in standard mode, only J4 manual.

## 8. Vibration Suppression C02.38-C02.4E

- (a)(b) Stated: a "Class 2 vibration suppression group", "low-frequency oscillation/vibration suppression", able to suppress two low-frequency oscillations at the mechanism's end simultaneously. Activation follows our guess (compensation ratio > 0), "consistent with similar designs".
- Inferred (Delta analogy, "using a similar architecture"): two parameters per group, frequency and response level; active when the level is > 0; a higher level responds faster, too high a level harms operation.
- Ours: the parameter table has three groups of seven (`C02.38` frequency, `C02.39` inertia correction, `C02.3A` low-pass correction, `C02.3B` high-pass 1 correction, `C02.3C` high-pass 2 frequency, `C02.3D`/`C02.3E` compensation ratios 1/2; groups 2 and 3 at `C02.40` and `C02.48`), not two groups of two. Which object is the "response level" is not said.
- (c) Inferred ("If this function is similar to Delta's (range: 1.0-100.0 Hz)"): 6-7 Hz is in range.
  - Stated (verification): write 6.4 Hz by SDO (64 if the unit is 0.1 Hz) -> read back -> J4 excitation test comparing amplitude and damping ratio.
  - Stated: the notches' 50 Hz floor does not apply here; this is "currently the only relevant built-in tool on the drive side".
  - Ours: the catalog has `C02.38` in 0.1 Hz units, raw 10..20000 (1.0-2000.0 Hz), default 100.0 Hz; 6.4 Hz is raw 64.
- (d) Stated: `C02.00` model following = command shaping (command tracking); the C02.38 series = vibration suppression compensation at one frequency; `C01.13/C01.14` = velocity feedforward. Whether the feedforward is shaped depends on their order, which the manual does not give: "frequency response testing is required".
- Stated (important): J4's 6.4 Hz is excited by reaction forces from other axes that never pass through J4's command path, so J4's own model following or feedforward is "largely ineffective" against it. Effective: `C00.06` (feedback path) and C02.38, "which acts on the torque/feedback path".
- Ours: the same paragraph first says all three "operate in the forward path"; where C02.38 acts is inconsistent within the reply.

## 9. Notches

- (a) Asked: adaptive notches 1st/2nd (7.14) or 3rd/4th (11.3.2)? Not answered directly. Stated (method): set the C01.40-series notch frequencies to distinct known values far from the real resonance -> enable adaptive identification with `C01.30` by SDO -> read the groups back; the one that changed is the adaptive notch. (The reply says "five notch filters" in item 0 and "four notch frequency groups" here; the table has five torque-notch groups `C01.40-C01.4E`.)
- (b) Stated, "Based on verified SDO mapping rules (Index = 0x2000 + Group Number, SubIndex = Sub-number + 1)": `C01.31 = 0x2001:20`, `C01.32 = 0x2001:21`, `C01.33 = 0x2001:22`.
  - Ours, conflict: by the same rule as the catalog applies it (hexadecimal suffix + 1, which gives `F30.10 -> 0x2030:11` and `C13.10 -> 0x2013:11`, i.e. `0x2013:17` decimal in the 2026-04-15 notes), `C01.31-C01.33` are `0x2001:32-0x2001:34`. The vendor's numbers treat "31" as decimal; `0x2001:20` would be `C01.1F`, and `0x2001:21`/`:22` are `C01.20`/`C01.21` (position reference average filters) in the catalog. Do not write those indices; read both candidates with an SDO upload to settle it.
- (c) Stated: the lower limit is 50 Hz. Accepting 10 Hz does not mean it works; low settings may be clamped internally or run outside specification.

## 10. EEPROM Writes (C13.10 = 1)

- (a) Unconfirmed ("confirmation from an FAE is required"): most general-purpose firmware skips writing an identical value; whether the A6-EC does is not known.
- (b) ALF1.3 threshold: Not answered. Stated generally: EEPROM endurance is typically 100,000 to 1,000,000 write/erase cycles (per sector/byte).
- (c) Stated (recommended regardless of (a) and (b)):
  - Startup: read back by SDO -> compare with the target -> write only if it changed, with `C13.10 = 1`. Steady state goes from about 10 EEPROM writes per axis per start to 0.
  - Categorise: write finalised parameters to EEPROM once; write parameters still changing during commissioning to RAM only (without `C13.10 = 1`) and commit them once final.

## 11. Low-Frequency (5-10 Hz) Load Oscillation

- Stated: no dedicated low-frequency section was found in the A6-EC manual (the notch section covers 50 Hz and up). The vendor's list, in order of implementation priority:
  1. Calibrate `C00.06`, "the fundamental fix". "The J4 setting of 190% is clearly an underestimate and the primary cause of your oscillation"; with the measured value (differential method) the damping ratio should rise beyond 0.05-0.1.
  2. C02.38, after the verification in item 8: "highly likely to be directly effective against the 6.4Hz oscillation".
  3. Disturbance observer `C02.60-C02.63`: no manual details. Conceptually it targets load-side disturbances including coupled reaction forces, but a badly chosen observer bandwidth could make the 6-7 Hz disturbance a new excitation source. Unconfirmed: consult an FAE on its applicable range and test incrementally.
  4. Command-side smoothing: S-curve or jerk limits on J1/J6 acceleration to cut excitation energy at 6.4 Hz; zero hardware cost, "should be done first".
  5. Mechanical review of the 6.4 Hz global mode and its low structural damping: reducer backlash, arm joint stiffness, damping materials.
- The vendor asks for: J4's `C00.06` measured by the differential method, and the results of items 8 and 9.

## What This Changes For Us

- `C00.06` is the vendor's first lever too, matching what J4 showed. Measured values replace trial ones; on a Rosie 1430 cell J4 keeps its measured 550 % and J2's 800 % stays a trial until measured.
- Dual-acceleration differential inertia measurement replaces `F30.10` for J4 (and is the preferred measurement for J5); a tool for it from CSP ramps and logged torque is active work. Ours: at 18:1, 500-1500 motor rpm is 167-500 deg/s at J4's output, and reaching 500 deg/s at 400 deg/s^2 takes 1.25 s and about 312 deg before any deceleration, so the velocity window must be fitted to J4's travel and speed limits. `J_m` needs the rotor inertia from the motor drawing (not in the reply).
- The J4 C02.38 trial is the vendor's own verification procedure: write the ring frequency, read it back, excite J4 with J1/J6 accelerations and compare amplitude and damping. The enable/level object and the meaning of the compensation percentage remain our inference from the table and the Delta analogy.
- Do not spend J4 iterations on its own model following (`C02.00`), feedforward or position-reference notch: the ring is driven by coupled reaction torque outside J4's command path (vendor item 8 (d)). Torque notches cannot reach below 50 Hz.
- Command-side smoothing on J1/J6 must be the shared time scaling: the TCP never leaves its commanded path, so no per-joint S-curve. Jog, Move-to and Cartesian steps are already bounded by the machine-level `jog_jerk`.
- EEPROM write-on-change (read, compare, write only on change) is active work. Ours: `607C` and HM35 persistence depend on `C13.10 = 1` (2026-04-15 notes), and a drive keeps the last stored value when a key is omitted, so any RAM-only commissioning mode must leave `C13.10 = 1` in force for Home and revert by explicit values.
- Do not use the vendor's `C01.31-C01.33` indices (item 9 (b)).
- `F30.10` stays out of rt-core (the catalog forbids writes to `0x2007` and `0x2030`). If an axis is ever identified from the keypad: master in PREOP or disconnected, brake released by hand, travel for at least one revolution each way, under the cell lock.
- Standard mode stays; a manual-mode J4 (hybrid) is an option only if J4 tracks poorly without oscillating once `C00.06` is right.

## Still Open / To Ask

- Gain switchover `C01.38` in standard mode (not asked; the observe baseline reads `C01.38 = 0`, fixed 1st gain set, on every joint).
- The J2/J3 379 Hz torque line and J5's 70/140/211 Hz line at rest (not asked).
- `F30.10` at J5's travel and under gravity load (J2/J3) (not asked); whether its "revolutions" are motor or load revolutions.
- C02.38-C02.4E: exact units and range, which object is the response level (`C02.3D` or `C02.3E`), what 100 % means, three groups in the table vs "two" in the reply, whether it acts in standard mode, forward vs torque/feedback path, and the manual section (7.14 vs none).
- Disturbance observer `C02.60-C02.63`: design range and bandwidth guidance (vendor: consult FAE).
- EEPROM: does the A6-EC skip identical writes (FAE), and the ALF1.3 threshold.
- Standard mode: which object reads the gains in effect, and does `C00.06` change Kp (questions 6 (b) and (c)).
- `C07.00` bits, `C07.01` range, the AS715N rotor inertia, `U40.34`.
- `C01.31-C01.33` object indices (vendor's decimal reading vs the catalog's hexadecimal rule); which notches the adaptive notch drives.


---
RAW EMAIL RESPONSE BELOW

Reproduced verbatim from the plain-text body of message `1a11f76c7c2b8dc6`; line breaks added only where the helpdesk's plain text ran two items together, and both personal names replaced with `[name]`.


Dear [name],

Thank you for your patience; here are the answers to the questions you listed, addressed one by one. I will be upfront about any points where I am not entirely certain.

   0. Let's start with the vibration mechanism, as it dictates the parameter tuning strategy.

The phenomenon you observed makes structural sense: J4 is the only axis among the six with an 18:1 ratio. When load inertia is reflected to the motor side, it is approximately 31 times greater—calculated as (100/18)²—than that of the other axes (which have a 100:1 ratio); consequently, J4 inherently has the smallest gain margin. Reaction torques from J1/J6 acceleration couple through the base into J4, forcing the J4 position loop to counteract the disturbance and exciting a weakly damped 6.4 Hz mode. The ratio of motor-side oscillation (24°) to output-side oscillation (1.3°) is approximately 18.5, matching the reduction ratio; this indicates the system is oscillating as a unit based on the rigid transmission ratio. It is not an output-side flexible two-inertia mode (where the motor side would remain nearly stationary), but rather a "rigid-body mode with weak damping from the position loop's restoring force." This also explains why adjusting C00.06 was effective: the inertia ratio became more accurate, gain matching improved in standard mode, closed-loop damping increased (ζ: 0.05 → 0.18), overshoot dropped (39% → 2%), and increased closed-loop stiffness raised the oscillation frequency (6.4 Hz → 7.35 Hz)—everything is consistent. Your approach of sweeping up to 700% is correct; the final value should be determined by the response (you should back off if the system becomes overdamped or if overshoot starts to reverse).

Additionally, note that the five notch filters on the A6-EC operate in the 50 Hz–8000 Hz range, so they cannot address the 6.4 Hz vibration. However, this generation of products features a dedicated "Low-Frequency Vibration Suppression" function capable of simultaneously suppressing two types of low-frequency vibration at the end-effector. Please refer to section 7.14, "Vibration Suppression," in the manual; this feature is designed specifically for scenarios like yours. You can also apply S-curve profiling to J1/J6 to limit jerk and reduce broadband excitation.

1. Source of the C00.06 value

All six axes hold non-default values (100% is the default). There are only two possibilities: either the F30.10 identification process was previously run and the result saved, or the value was manually written. The A6-PN manual confirms that the identification function is specifically designed to determine C00.06 ("automatically auto-tuned by F30.10"). The most direct verification method is to run the identification process on a specific axis and read C00.06 via SDO before and after; a change indicates automatic writing. Additionally, restore the default values (factory reset) and read the parameter again to see if it returns to 100%.

   2. Offline inertia identification under EtherCAT

(a) The manual's panel-based identification procedure begins directly at "Power on" without explicitly stating bus state restrictions. However, based on the A6-EC framework (where panel functions like S-ON are only valid in non-bus control modes) and the fact that the drive must self-enable the motor to perform identification—while the master holds control of object 6040 in the OP state—a conflict is highly likely. The safest approach is to switch to the PREOP state or disconnect the master entirely before performing panel-based identification.

(b) The ESI file is merely a subset description of the object dictionary (OD); the absence of an entry in the ESI does not mean it is missing from the OD. The SDO mapping rule for parameter groups is 0x2000 + Group Number (e.g., Group C07 → 0x2007; Group F30 → 0x2030). For index 10, applying the offset (+1) results in 0x2030:11 (your deduction was correct). Use the ethercat sdos command to list actual objects and verify their existence and read/write permissions. Note: If 0x2030:11 is writable and triggering it causes the drive to self-enable and operate, ensure the brake and travel limits are safe before activation, as the motor will move during identification.

3. Holding brake during the identification process

The brake does not release automatically in sync with the process. Since your J4 drive is equipped with a holding brake, you must ensure the brake is actually released before identification begins (either by manually applying power to the brake or by configuring the BK output logic to interlock with the identification function). Otherwise, attempting to rotate the motor while the brake is engaged will result in a stall-induced overcurrent fault and potential motor damage.

   4. F30.10 operation details

(a) It does not operate via a "press-and-hold" mechanism. Procedure: Set F30.10 on the panel → Press [S] to confirm → Use [▲]/[▼] to set the number of identification revolutions → Press [S] to confirm → The drive automatically performs multi-revolution runs in both forward and reverse directions → The inertia value is displayed → Press [M] to finish.

(b) Use the [M] key to abort the operation; the "stop upon button release" behavior applies to the JOG function (F30.00), not to this identification process, which involves automatic operation based on a preset number of revolutions.

(c) The number of revolutions corresponds to the value you set; the motor runs in both directions, so the mechanical travel range must accommodate at least one full revolution in each direction.

(d) Automatic writing to C00.06: The manual's procedure only mentions "displaying the inertia value" without explicitly stating the save logic; however, the fact that all six axes have non-default values strongly implies that the identification results are written and saved.

(e)

(f) C07.02: Following standard design conventions, this represents the time constant for accelerating from 0 to the set speed (C07.01).

(g) For a 1000W motor (80mm frame size) with a rated speed of 3000 rpm and a maximum speed of 6000 rpm, the recommended identification speed is in the 500–2000 rpm range.

   5. Key issue: The J4 cannot perform panel-based identification, but there is a better method.

(a) 3000 rpm/s ÷ 18 ≈ 1000 deg/s² at the output; this exceeds the 720 limit. This is not merely a matter of "slightly lower precision"—the conditions specified in the manual (max speed > 150 rpm, acceleration/deceleration > 3000 rpm/s) are prerequisites for ensuring identification accuracy; Axis J4, as installed in the complete machine, can never meet these requirements. You can essentially disregard the panel setting F30.10 for Axis J4.

However, the alternative method I propose fits entirely within your existing software framework: since you already have the capability to log position, velocity, and torque data (via CSP, SDO, and PDO), you can directly measure the total inertia using a dual-acceleration differential method:

① Use CSP to command two segments of constant acceleration for J4 (velocity ramps with output-side accelerations of 400 deg/s² and 700 deg/s², both within limits), sweeping across the same velocity range (e.g., 500–1500 rpm at the motor shaft);

② Record the average torque for each segment within that range (Object 0x6077; the CiA402 unit is 0.1% of rated torque, so multiply by the motor's rated torque to convert to N·m);

③ Calculate J_total (motor side) = (T1 − T2) / (α1 − α2); by taking the difference between the two segments over the same velocity range, the friction component is cancelled out;

④ Calculate C00.06 = (J_total / J_m − 1) × 100%.

This method requires no mechanical modifications, does not exceed speed limits, yields directly reliable results, and aligns perfectly with your "all-parameter software control" architecture. You can also use the recorded data to verify the 700% deviation figure you obtained earlier.

(b) Rotor inertia: This is available in the motor's technical drawings; you can look up the data directly.

   6. Derivation of gains for Standard Mode (C00.04 = 1)

(a) The manual states that "only one parameter needs to be set, and the drive automatically provides a matching set of gains"; the manufacturer does not disclose the internal formulas used. Regarding structure: C00.05 (Rigidity Level) determines the base gain group (speed loop Kp/Ki, position loop Kp, filters), while C00.06 (Inertia Ratio) handles scaling and matching based on equivalent inertia. The values you read back—C01.00–C01.03 = 400/250/3184/200—are the derived results for Rigidity Level 12 at that specific inertia ratio; thus, your method of "modifying C00.06/C00.05 and reading back the C01 group" is the correct engineering approach to reverse-engineer this mapping.

In June of this year, we posted the official tuning procedure for the A6 series on the LinuxCNC forum: Set C00.04=0 (Manual Mode) → Confirm inertia ratio → Gradually increase speed loop C01.01 until slight noise/vibration occurs, then back off by 15–20% → Gradually decrease C01.02 to eliminate steady-state error (avoid setting it too low to prevent low-speed creep) → Gradually increase position loop C01.00 (back off by 10–15% if overshoot occurs) → Finally, add feedforward and friction compensation. For a weakly damped, easily excited axis like J4, once the inertia ratio is accurately set, the second step is to adjust the speed loop (C01.01/C01.02), leaving the position loop for last.

   7. Recommended Tuning Method

Short-term recommendation: Standard Mode + accurate C00.06 setting for each axis. Rationale: The reflected inertia of your six-axis loads varies by a factor of 31 ((100/18)²); Standard Mode's scaling based on inertia ratio is designed specifically for this scenario, and the J4 experiment (shifting from 190% to 400%) has already proven the effectiveness of this approach. For axes with a 100:1 ratio: After verifying C00.06 (using the differential method or calculations based on load and gear ratio), perform a sweep for C00.05 starting from a value of 10–15; adjust further only if tracking or stability targets are not met.

J4: First, accurately tune C00.06; keep C00.05 at 12, and confirm the damping ratio remains stable above 0.15 before considering an increase in stiffness.

When to switch to manual mode (C00.04=0): Switch if an axis shows poor tracking error without oscillation after C00.06 is tuned in standard mode, or if the position loop bandwidth needs to be adjusted independently. Follow the official procedure for manual mode: gradually increase C01.01 (speed loop Kp) until slight noise appears, then back off by 15–20% → gradually decrease C01.02 (speed loop Ki) to eliminate steady-state error (avoid setting it too low to prevent creeping) → gradually increase C01.00 (position loop gain) (back off by 10–15% if overshoot occurs).

A hybrid strategy is also viable: keep the five 100:1 axes in standard mode while switching only J4 to manual mode.

   8. Vibration Suppression C02.38 – C02.4E

(a)(b) This set of parameters belongs to the Class 2 vibration suppression group; its function is defined as "low-frequency oscillation/vibration suppression" (capable of suppressing two types of low-frequency oscillation at the mechanism's end simultaneously). Activation follows the logic you surmised (compensation ratio > 0), consistent with similar designs—for instance, Delta’s low-frequency vibration suppression (using a similar architecture) employs two parameters per group: frequency and response level. Activation occurs when the response level is > 0; a higher level yields faster response, though an excessively high level can adversely affect operation.

(c) If this function is similar to Delta's (range: 1.0–100.0 Hz), then 6–7 Hz falls within the operational range. Verification procedure: Write 6.4 Hz via SDO (or 64 if the unit is 0.1 Hz) → read back to confirm acceptance → perform a J4 excitation test to compare amplitude and damping ratio. Note: The 50 Hz lower limit for the notch filter does not apply to 6.4 Hz; this function is currently the only relevant built-in tool on the drive side.

(d) All three operate in the forward path but at different points: C02.00 Model Following = command shaping (improves command tracking); C02.38 series = vibration suppression compensation for a specific frequency; C01.13/C01.14 = velocity feedforward. The order in which feedforward and the shaper are superimposed determines whether the feedforward signal is shaped; since the manual does not specify this, frequency response testing is required. Important note: The 6.4 Hz oscillation in J4 is excited by mechanical coupling reaction forces resulting from the acceleration of other axes—forces that do not pass through J4's command path. Consequently, adjusting J4's own model following or feedforward settings is largely ineffective against this type of oscillation. Effective measures include C00.06 in the feedback path and C02.38, which acts on the torque/feedback path.

   9. Notch Filter

(a) You can manually set the four notch frequency groups (C01.40 series) to distinct, known values far removed from the actual resonance frequency → set C01.30 via SDO to enable adaptive identification → after identification completes, read back the four frequency groups; the group with the automatically modified value indicates the adaptive notch filter.

(b) Based on verified SDO mapping rules (Index = 0x2000 + Group Number, SubIndex = Sub-number + 1; you have verified F30.10 = 0x2030:11 and C07 = 0x2007): C01.31 = 0x2001:20, C01.32 = 0x2001:21, C01.33 = 0x2001:22.

(c) The specified lower limit is 50 Hz. Accepting a parameter setting of 10 Hz does not guarantee effective operation—low-frequency settings may be internally clamped, or performance may fall outside specifications.

   10. EEPROM Write (C13.10=1)

(a) Most general-purpose firmware implementations employ "skip-write-on-identical-value" logic to minimize wear; however, confirmation from an FAE is required to verify if the A6-EC follows this practice.

(b) ALF1.3 threshold. EEPROM write/erase endurance is typically in the range of 100,000 to 1,000,000 cycles (calculated per sector/byte).

(c) Recommended approach (independent of the two unconfirmed factors above):

Modify the startup sequence to: "Read back via SDO → Compare with target value → Write only if the parameter has changed AND set C13.10=1." In steady-state operation, this reduces the number of EEPROM writes per axis from 10 to 0, effectively eliminating wear.

Categorize parameters: Write finalized parameters to EEPROM once; write parameters frequently modified during commissioning only to RAM (without setting C13.10=1), and commit them to non-volatile memory only after finalization.

   11. Low-frequency load oscillation (5–10 Hz) Suppression

I could not find a dedicated low-frequency oscillation section in the A6-EC manual (the notch filter section covers 50Hz+ resonance). Here are the proposed solutions, prioritized by order of implementation:

Correctly calibrate C00.06—this is the fundamental fix. The J4 setting of 190% is clearly an underestimate and the primary cause of your oscillation; after measuring the actual value using the differential method and matching the feedback path gain, the damping ratio should improve beyond the current 0.05–0.1 range.

C02.38 (Low-frequency jitter suppression)—enable this after verifying the settings in point 8; it is highly likely to be directly effective against the 6.4Hz oscillation.

Disturbance Observer (C02.60–C02.63)—no manual details available. Conceptually, this targets load-side disturbances (including reaction forces from coupling with other axes), but improper observer bandwidth design could turn the 6–7Hz disturbance into a new source of excitation; I recommend consulting an FAE regarding the design's applicable range and proceeding with incremental testing.

Command-side smoothing—apply S-curve profiling or jerk limits to the J1/J6 acceleration commands to reduce excitation energy at 6.4Hz. Since your controller calculates trajectories at 1kHz, this step incurs zero hardware cost and should be done first.

Mechanical-side review—address the 6.4Hz global mode and the low structural damping (ζ=0.05–0.1); focus areas include reducer backlash, arm structural connection stiffness, and the potential use of damping materials.

Looking forward to your data: the C00.06 value measured via the J4 differential method and the experimental results from points 8 and 9.

Any questions, please don't hesitate to let me know.

--

Best regards,
[name]

STEPPERONLINE
Tel: +86-25-87156578 x059
technical@omc-stepperonline.com
www.omc-stepperonline.com
