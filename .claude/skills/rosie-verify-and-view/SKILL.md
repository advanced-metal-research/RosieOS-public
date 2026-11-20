---
name: rosie-verify-and-view
description: Verify an OLP program and LOOK at it before anything runs — `rosie program check` (pass/fail in absolute units), `rosie view scene` (OLP's own 3D renderer, multi-angle PNGs you open as images) and `rosie report program` (HTML for a person). Use after every authoring change, before every `rosie run`, whenever a weld/part/fixture placement or a path is in question, and when reporting a program to the user.
---

# Check it, then look at it

Agents make geometric mistakes a glance would catch: a part never moved
from the import origin, a part on the floor instead of the stand, a torch
through a fixture, a path through the table, a waypoint taught on the wrong
side. Numbers in a spreadsheet hide these. A picture does not.

## The loop (after every meaningful change, and always before a run)

```bash
rosie program check          # verdict first; exit 1 if anything fails
rosie view scene             # 6 PNGs; then OPEN THEM (Read the files as images)
```

`rosie program check` checks, in absolute units:

| Check | Fails / warns when |
| --- | --- |
| program problems, rows | OLP's document check or a row in error; rows not compiled |
| plan | none, or STALE (edited since planning) |
| robot model | the program's robot ≠ the selected cell's |
| part placement | the part is still at the import origin (warn: was it placed?) |
| trajectory ingest rules | the kept `.rdt` breaks a dense-daemon rule |
| joint speed ceilings | any joint over its ceiling |
| MoveL straightness | a MoveL's planned TCP leaves its line by > 0.5 mm |
| TCP speed | a move peaks > 5 % over its commanded speed (warn) |

## Reading the views

Default shots: `home` (whole cell) at start, `iso`, `front`, `top` at start,
`right` mid-plan, `iso` at the end. Choose others with
`--shots camera@pose`: cameras `home part path front back left right top
iso`; poses `start`, `end`, a fraction `0..1`, `row:<id or number>`.

```bash
rosie view scene --shots iso@row:rails-seam_0001,front@row:rails-seam_0001   # the robot at a weld
rosie view scene --shots part --size 1600x1000                                # close on the part
rosie view scene --no-plan                                                    # authored path only
```

Open every PNG and ask, out loud in your answer:

1. Is the part where the user said, on the table/stand, not in the air or the floor?
2. Does each weld line (yellow) sit on the part's seam?
3. Does the planned path (blue) or the robot pass through the stand, a fixture
   or another part?
4. Is the robot at the weld in a sane pose (no wrist flip, not over-reaching)?
5. Do the waypoint labels (P1, P2, …) appear in the order you intended?

The body also prints scene facts: part triangles (0 = part NOT drawn: its
STEP is not kept on this machine, pass `--step <file>`), fixtures, welds,
path points, planned points, part bounds in mm, and warnings (stale plan,
missing plan bytes). Treat a warning as something to fix or tell the user.

## For a person

```bash
rosie report program --open   # self-contained HTML: verdict, views, checks, spreadsheet, planned path
```

Give the user the file path from `files:`. `rosie report cell` is the
cell-state equivalent.

## Notes

- Rendering takes ~10–20 s cold (headless Chrome + OLP's renderer); `rosie
  doctor` says whether Chrome and the snapshot page are available.
- Named views frame the subject, so the arm can be cropped; `home` shows all.
- Classic welds draw as the row path; the rails overlay, edge overlays and
  J6/TCP panes are not drawn. `row:` poses use taught joints or the first
  planned sample of that row (no IK).
- The check thresholds (0.5 mm, 5 %) are unverified policy; a failure means
  "look", not "the robot will crash".
