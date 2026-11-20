---
name: rosie-programs
description: Author, edit, plan and save OLP weld programs with the `rosie` CLI — new/open, taught MoveJ/MoveL/MoveC waypoints, dwell/IO/Home rows, field edits by OLP path, STEP parts with detected seams, torch-angle search and rails welds, waypoint/line/arc welds, fixtures, part placement, plan, save to the cell or a file, undo. Use for any "make/edit/plan a weld program" task. Nothing here moves the robot.
---

# Programs with rosie

Every `rosie program|part|seam|weld|rails|fixture` command runs OLP's own
engine (the browser's and the Deck's) on the workspace program, so every
field, unit, range, refusal and undo step is OLP's. Authoring and planning
move nothing.

## What to know first

- **One program per workspace**, on disk (`--ws <name>`, default `current`).
  `program new|open` refuse to replace unsaved edits; use another `--ws`.
- **Rows** are referenced by number (`3`), id, or unique id prefix.
- **Units are display units**: mm, °, %, s. `target.xyz_m.0` takes **mm** and
  `target.rpy_rad.2` takes **degrees** despite the names.
- **Every edit makes the plan stale.** Plan last; `program check` says STALE.
- **Torch and angle-search settings** (`rosie rails`) are workspace session
  state, kept between commands; set them **before** searching seams.
- **STEP bytes are never shared between clients.** A program opened from the
  cell whose part this machine never imported needs `--step <file>` once.
- **Rails authoring is chosen at New program**; `program new` always makes a
  rails program. A classic program cannot gain rails welds.

## Recipe: a weld program from a STEP part

```bash
rosie program new bracket-a                       # --model rosie_1420_v1 if no cell is selected
rosie part import ~/parts/bracket_a.step          # ~20 s: topology + seam detection
rosie part place --offset 700,-100,0 --rot 0,0,90 # ASK the user where it sits; mm / ° in the work frame
rosie rails set --torch-preset mig_gas_nozzle_16  # optional, before searching
rosie seam list
rosie seam search --all                           # clear / partial / blocked per seam
rosie seam weld --clear                           # accept every clear seam; partial ones: ask the user
rosie seam weld seam_0003 --runs 0,2              #   …a partial seam's chosen clear runs
rosie weld profile rails-seam_0001 --angle work --points 0:40,0.5:45,1:42
rosie program plan                                # minutes on the CUDA planner
rosie program check && rosie view scene           # then OPEN the PNGs (rosie-verify-and-view)
rosie program save                                # only when the user asks
```

## Recipe: taught moves

```bash
rosie program new teach-test
rosie program add-waypoint --joints 0,-30,45,0,60,0
rosie program add-waypoint --joints 20,-20,40,0,60,20 --motion linear --speed-mm-s 50
rosie program add-event dwell --after 2 && rosie program edit 3 duration_s=1.5
rosie program add-event home
rosie program plan --from-start
```

Joints are degrees in URDF order, every joint of the program's robot.
`--xyz/--rpy` on `add-waypoint` make the target Cartesian (the joints seed
IK). MoveC: `--motion circular`, then `rosie program reteach <row> --joints … --as via`.
From the robot's measured pose: jog it there (browser/Deck or `rosie robot
move`), then `rosie program record` ("Pull machine calibration" →
`rosie program robot pull`; "Adopt the matching robot description" →
`rosie program robot adopt`).

## Editing any field

```bash
rosie program row 3                                 # every field: path, value, unit, range, options
rosie program edit 3 speed_mm_s=40 label="slow approach" target.xyz_m.2=350
```

Values parse as JSON when they are JSON (numbers, true, null, [..]),
otherwise as text. OLP refuses out-of-range values (never clamps); edits apply
in order and the first refusal stops the rest. Rows: `remove`, `move <row>
--by -1`, `enable`, `disable`, `speed <percent>`, `undo [n]`, `redo [n]`.

## Hand-placed welds, parts, fixtures

```bash
rosie weld add --tool line --points '[{"xyz_mm":[0,0,5],"rpy_deg":[180,0,0]},{"xyz_mm":[40,0,5],"rpy_deg":[180,0,0]}]'
rosie weld edit 5 --op move_point --index 1 --xyz 45,0,5
rosie part list | rosie part switch <id> [--step f] | rosie part remove
rosie fixture list | add-stand | edit stand sz=850 x=900 | step clamp.step | replace stand stand.step | remove <id>
```

Waypoint-weld points are in the PART frame; `rpy [180,0,0]` is the torch
straight down. A second `part import` adds a part and makes it active; only the
active part's welds plan, the others are obstacles.

## Plan, save, export

- `rosie program plan [--from-start] [--repeat count|loop]`: with a cell
  selected, a program with motion plans from the measured pose
  (`planning_start_moving` while it moves); `--repeat` adds the return to
  start (needed for Run N times / Loop). `rosie program preview` shows the
  planned TCP path per segment.
- `rosie program save` (cell, revision-checked) or `--file <path>`. A 409
  `program_conflict` means someone saved since you opened: tell the user; reopen
  theirs (`program open <id> --force`) or `save --copy` only with their say.
  423 = the robot has it loaded.
- `rosie program export bracket.weldplan` (the plan container) or
  `bracket.olp.json` (OLP's Export JSON).

## Refusals you will see

| Message | Meaning / next |
| --- | --- |
| workspace "…" has no program | `rosie program new <name>` or `open <id>` |
| has an unsaved program | save it, use `--ws other`, or `--force` (discards it) |
| no part / classic program | `rosie part import` / start a new program for rails |
| no copy of the part's STEP | `--step <same file the program was authored with>` |
| "plans joint moves only" / "needs the CUDA weld planner" | CPU-only planner: welds and MoveL need CUDA (`rosie planner status`) |
| model mismatch | the program's robot ≠ the selected cell's |
| `<path> refused: …` | OLP's range/read-only rule; report it, don't clamp |
