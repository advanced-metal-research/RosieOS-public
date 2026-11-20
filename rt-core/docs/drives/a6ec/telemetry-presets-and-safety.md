# TxPDO presets 0x1B02 and 0x1B04: the case for each, and what the standards say

Written 2026-09-15 for the owner's question whether the drive's cyclic telemetry menu matters for safety.
The A6-EC has fixed TxPDO presets; only the selection (0x1C13:01, in PREOP, lost at power-off) changes.
Both presets carry error code, status word, position, torque, mode and touch-probe data every cycle.
0x1B02 adds the digital-input word 0x60FD; 0x1B04 drops it and adds following error 0x60F4 and actual
velocity 0x606C.

Entry order below is one-based; every subindex is 0. Types follow the
[A6-EC object table](a6ec_manual_chapter_11_parameter_list.md). The robot and
bench 0x1B04 layouts match entry for entry; `a6ec_1b02` retains the previous
robot layout bytes, including the entry source notes.

| Object | Signal | Type / bits | 0x1B02 order | 0x1B04 order |
|---|---|---|---|---|
| 0x603F | Error code | U16 / 16 | 1 | 1 |
| 0x6041 | Status word | U16 / 16 | 2 | 2 |
| 0x6064 | Actual position | I32 / 32 | 3 | 3 |
| 0x6077 | Actual torque | I16 / 16 | 4 | 4 |
| 0x6061 | Mode display | I8 / 8 | 5 | 5 |
| 0x60F4 | Following error | I32 / 32 | absent | 6 |
| 0x60B9 | Touch probe status | U16 / 16 | 6 | 7 |
| 0x60BA | Touch probe position 1 | I32 / 32 | 7 | 8 |
| 0x60BC | Touch probe position 2 | I32 / 32 | 8 | 9 |
| 0x60FD | Digital inputs | U32 / 32 | 9 | absent |
| 0x606C | Actual velocity | I32 / 32 | absent | 10 |

## What the standards require, and where rt-core sits

| Topic | Standard | What it says (for this question) | Consequence for rt-core |
|---|---|---|---|
| Robot safety functions | ISO 10218-1:2025 with ISO 10218-2:2025 | Emergency stop, protective stop, safety-rated monitored speed and stop, speed limit in manual reduced-speed mode, and the other safety functions must be designed and validated as safety-related control per ISO 13849-1 (Performance Level, typically PL d category 3) or IEC 62061 (SIL). | A general-purpose controller reading non-safety process data is not a safety-rated channel. rt-core's guards are protective layers, not the safety function. |
| Safety-related control systems | ISO 13849-1 | Performance Level from architecture (categories), diagnostic coverage, reliability; single-channel software on a general controller does not reach PL d. | Door interlocks, e-stop and light-curtain stops must run through safety-rated hardware (safety relay or safety PLC acting on the drives' power or a certified drive safety input). |
| Drive safety functions | IEC 61800-5-2 | Defines STO, SS1, SS2, SOS, SLS and the rest as functions of a drive that carries a safety rating for them. | Our copy of the A6-EC manual documents no STO input or safety rating; the stop chain therefore acts on the drives' supply or a contactor, outside the drive's own logic, whichever preset we stream. |
| Fieldbus safety data | IEC 61508 with Safety over EtherCAT (FSoE, ETG.5100) | Safety process data is only safety-rated inside a certified safety protocol layer (FSoE, SIL 3); standard PDOs are not. | 0x60FD in a standard PDO is never a safety input. Its 1 ms arrival is a convenience, not a rating. |
| Stop categories | IEC 60204-1 | Category 0 (immediate power removal), 1 (controlled stop then power removal), 2 (controlled stop, power kept). | Stop fence = category 0 behaviour at the software level; Halt = category 2 behaviour; neither is the safety function. |
| Safeguard placement | ISO 13855 | Minimum distance S = K × T + C; T is the whole stopping performance including detection, control response and machine stopping time; K is 1600 or 2000 mm/s. | If a light curtain's response were routed through rt-core (it should not be), a 40 ms mailbox delay adds 64 to 80 mm to the required distance; a 1 ms cyclic bit adds 1.6 to 2 mm. This is the only place the 1 ms versus 40 ms question has a quantitative answer, and the safety-rated chain bypasses rt-core anyway. |
| Collaborative operation | ISO/TS 15066, absorbed into ISO 10218-2:2025 | Power and force limiting uses body-region force and pressure limits; collision detection must itself be part of a validated safety function to count. | A torque or following-error watchdog in rt-core is a protective layer that reduces damage; it cannot make Rosie collaborative on its own. |

Sources: ISO 13855 formula and constants ([Keyence](https://www.keyence.com/ss/products/safetyknowledge/caution/), [Schmersal](https://www.schmersalusa.com/press/technical-articles/calculating-safety-distances)), ISO 10218-1:2025 stopping and functional-safety requirements ([ISO OBP](https://www.iso.org/obp/ui/en/#!iso:std:73933:en), [gt-engineering](https://www.gt-engineering.it/en/technical-standards/en-iso-standards/en-iso-10218-1-safety-requirements-for-industrial-robots/5-5-robot-stopping-functions/), [SGS](https://www.sgs.com.tw/en/news-media-resources-content/page?id=2132)), IEC 61800-5-2 functions ([SEW](https://download.sew-eurodrive.com/download/html/30587239/en-EN/4014183898743621600907.html), [Synapticon](https://www.synapticon.com/en/motion-control-academy/sichere-stopp-funktionen-ss1-ss2-sos)), FSoE ([EtherCAT Technology Group](https://www.ethercat.org/en/safety.html), [Beckhoff](https://www.blog.beckhoffus.com/post/safety-over-ethercat)), ISO/TS 15066 ([FourByThree](https://fourbythree.eu/isots-15066-new-safety-requirements-for-a-new-era-of-industrial-robots/), [comparative analysis](https://arxiv.org/pdf/2602.17822)).

## The case for 0x1B02 (digital inputs cyclic)

- rt-core sees a drive input change in the next cycle. Useful for a fast software reaction that mirrors a hardware event (log, retire motion, refuse re-arm) or for comfort functions (slow zone, consent) where tens of milliseconds would be felt.
- Matches how GradientOS and v4 ran and what the drive powers up with; no assignment write at bring-up.
- Costs: no drive following error or velocity in the cycle; rt-core computes following error itself from command against feedback, but the drive's own view and its measured velocity are unavailable for tuning and for a torque-plus-error watchdog.
- Fact (2026-09-15): no digital input is wired or declared on any cell; the drives carry factory function defaults; v4 mapped the word and never read it.

## The case for 0x1B04 (following error and velocity cyclic)

- The drive's own following error and measured velocity at 1 kHz: what the feedforward tuning, the tracking analysis and a collision watchdog need. Owner decision 2026-09-15: this is the default for bench and robots (`a6ec_bench_1to1` and `a6ec`).
- Digital inputs remain readable by mailbox (24 to 47 ms, best-effort, readiness-gated) for display and supervision; the drive's own reactions to overtravel and home-switch inputs are unchanged.
- Costs: the assignment must be written and verified at every bring-up (the drive forgets it at power-off), which is exactly the bring-up path fixed by the daemon-owned assignment work; no 1 ms view of a drive input.

## Design position

1. Safety functions live on a safety-rated hardware chain acting on the drives' power. rt-core observes and reacts protectively; nothing here is PL-rated.
2. Contacts rt-core must react to within a cycle belong on cell I/O (an EtherCAT I/O terminal or the RevPi inputs, tracker P6-05), read cyclically regardless of the drive preset.
3. Drive inputs are a display and supervisory signal, read by mailbox on 0x1B04.
4. Default 0x1B04 for bench and robots. The daemon-owned PREOP assignment write and automatic IgH rescan were accepted on nine AS715N drives on the nine-motor bench on 2026-09-15: all nine reached OP with zero unavailable readbacks (evidence: the bench record of that day, private). The explicit `a6ec_1b02` alternative preserves the old layout: select when wired drive digital inputs must be cyclic. Robot-backed cells choose this profile through their pinned robot definition; flat configurations use `axes[i].profile`. After the coordinate-identity change (P0-12) switching presets does not independently require Home.

With fresh cyclic feedback, 0x1B04 publishes `di_bits=0`, `di_valid=false`,
`velocity_actual_valid=true` and `following_error_valid=true`; the latter two
values are signed 32-bit drive-frame counts/s and counts. With `a6ec_1b02`, fresh
0x60FD makes DI valid, while drive velocity and following error are unavailable
(zero values with false validity). Mailbox reads do not change cyclic validity.
