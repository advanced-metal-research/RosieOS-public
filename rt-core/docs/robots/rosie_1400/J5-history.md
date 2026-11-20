# J5 ratio history and model-specific owner decision

Historical extraction, 2026-09-14, superseded by the owner decision appended
below on 2026-09-15: the definition then retained deployed **100/1**. CAD shows a belt
stage with a 60-tooth ring (`60ring gear2` under `TG60-G20`); GradientOS and the
specification sheet say **27:2**; the pair record and drives say **100**. Owner
confirmation of the installed gearbox and both pulley tooth counts is required
before either value is treated as physical fact. The mechanism has changed more
than once. A configured drive ratio is not a mechanical measurement.

The table distinguishes source dates from dates inferred in retrospective notes.
`config.py` below means GradientOS
`src/gradient_os/arm_controller/robots/gradient05/config.py`.
`scratchpad` means GradientOS `.cursor/memory/AGENT_SCRATCHPAD.md`.
Current file hashes and numbered excerpts are in [source-excerpts.md](source-excerpts.md).
Git excerpts below preserve older source lines that are absent from the current file.

| Date | J5 value or change | Source line and interpretation |
|---|---|---|
| 2026-03-19 | 20:1 | `config.py:93` at `92918819`; first committed ratio array found |
| 2026-03-20 | 18.1818181818 (20 × 20/22, nominal 200/11) | `config.py:93` at `81f5c27c`: 22T driving 20T after a 20:1 planetary gearbox |
| 2026-04-04 | 31.25 (125/4) | `config.py:93` at `d6606123`: 16T driving 25T after a 20:1 planetary gearbox; also scratchpad:493, dated 2026-04-12 |
| 2026-04-23, committed 2026-04-24 | 11:1 | `config.py:93` at `8088fe85` explicitly dates a gearbox swap to 2026-04-23 |
| Before the 2026-04-25 correction; exact date unverified | 100/11 | Retrospective scratchpad:2484 says the carried-over 100/11 must become 10; no earlier dated source line was found |
| 2026-04-25 | 10:1 | scratchpad:2484, owner correction |
| 2026-04-26 | 11:1, briefly | scratchpad:2558 retrospectively describes a brief correction on this date; preserve separately from the April 23 code comment |
| 2026-04-26, committed 2026-04-27 | 100/11 | scratchpad:2557-2558; `config.py:93-94` at `bf22a964` |
| 2026-04-30, before final correction that day | 18:1 | scratchpad:2787 asks for 1,000 deg/s rated and 2,000 deg/s peak; superseded explicitly by scratchpad:69 |
| Date unverified; reported in robotdef3 task on 2026-09-14 | “18:1 with a 22T:20T belt” | Task's Sources found item 3; not found as a dated line in the supplied current scratchpad or DEVLOG. Do not equate this claim with the March 20 code's 20:1 gearbox × 20/22 |
| 2026-04-30 | 27/2 = 13.5 | scratchpad:67-70; `config.py:93-94` at `2945e3ff`: 10:1 stage followed by 27T:20T |
| CAD generated 2026-05-28 | No settled current total ratio | [component-list.json](component-list.json), L4: `TG60-G20`, `60ring gear2`, belt tensioner and A6M60 motor; the names do not establish both installed pulley tooth counts |
| 2026-06-20 | Ratio changed with different pulleys; value unspecified | Team chat (private): reported a J5 ratio change with different pulleys |
| 2026-08-03 | 99:1 → 100:1 | Team chat (private): reported J5 99:1 → 100:1 on the bench host, pair revision 3 |
| 2026-09-10 | Current ratio disputed; replacement unspecified | Team chat (private): reported the J5 gear ratio as wrong |
| 2026-09-13 | 100:1 | v4 `new-record.json:1`, committed_at `2026-09-13T08:39:56.807469Z`, pair `rosie_1420_v1`; bench drive readback `readback.txt:355-356`, slave 4, numerator 100 and denominator 1 |

The supplied `DEVLOG.md` currently has 17 lines about 2026-02-21 configuration
setup and no J5 ratio history. The task's compressed chronology therefore cannot
be reproduced literally from that file. The dated lines above take precedence
over inventing dates or mechanically combining superseded configurations.
The team-chat entries are paraphrased from quotations in the task received
2026-09-14; message links, export line numbers and independent retrieval were
not supplied.

## Historical GradientOS source lines

Read-only source repository: `C:/Users/operator/Desktop/projects/GradientOS`.
Each excerpt is `git show <commit>:src/gradient_os/arm_controller/robots/gradient05/config.py`.

```text
92918819 (2026-03-19), line 93:
return [100.0, 100.0, 100.0, 18.0, 20.0, 10.0]
81f5c27c (2026-03-20), line 93:
return [100.0, 100.0, 100.0, 18.0, 18.1818181818, 10.0] # the 18.18 ratio is from the pulled setup on the timing belt: 22T driving 20T after the 20:1 planetary gearbox
d6606123 (2026-04-04), line 93:
return [100.0, 100.0, 100.0, 18.0, 31.25, 10.0] # the 31.25 ratio is from the pulled setup on the timing belt: 16T driving 25T after the 20:1 planetary gearbox
8088fe85 (2026-04-24), line 93:
return [100.0, 100.0, 100.0, 18.0, 11.0, 10.0]  # J5 updated 2026-04-23 from 31.25 to 11 (gearbox swap)
bf22a964 (2026-04-27), lines 93-94:
# J5 is commissioned as an exact 100:11 drive-native ratio.
return [100.0, 100.0, 100.0, 18.0, 100.0 / 11.0, 10.0]
2945e3ff (2026-04-30), lines 93-94:
# J5 uses a 10:1 stage followed by 27T:20T, for an exact 27:2 ratio.
return [100.0, 100.0, 100.0, 18.0, 27.0 / 2.0, 10.0]
```

## Owner decision, 2026-09-15

The owner was asked whether the deployed J5 = 100 evidence (the v4 pair
record for `rosie_1420_v1`, drive readback and the 2026-08-03 team-chat report of the
99:1 to 100:1 change on pair revision 3) describes the 1420. The answer was:
"yeah this is rosie 1420; if there's that then we go 1420."

This settles the definition values per model: **Rosie 1420 v1 J5 = 100/1**,
owner-confirmed with pair record and drive readback as corroboration;
**Rosie 1400 v3 J5 = 27/2**, retaining GradientOS gradient05 commit
`2945e3ff` (10:1 stage then 27T:20T), the task-cited
`.cursor/memory/DEVLOG.md` 2026-05-01 and the
[spec sheet](spec-sheet-axis-table.md) rated J5 speed of 1,333 deg/s.
The [CAD L4](component-list.json) A6M60-400 motor, 60-tooth ring gear and
belt tensioner identify the 1400 belt-stage mechanism; the 1420 J5 mechanism
differs. The DEVLOG date is supplied by the 2026-09-15 owner-decision task;
the locally copied DEVLOG described above does not independently reproduce it.
The config commit and sheet remain directly recorded corroboration for 27/2.

The 1400 ratio remains **unverified for the physical build**: physical pulley
teeth on the current 1400 build are unconfirmed by the owner. Confirmation of
the current pulleys by the owner or a mechanical document is still required.
The 1420 ratio no longer carries that qualification; sign and encoder-count
qualifications remain on both models. J6's HRP85 variant ratio is still
unconfirmed. Earlier references to an unresolved conflict or retained 100/1
for the 1400 describe the pre-decision state, not the current definitions.
