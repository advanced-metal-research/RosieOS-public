# Weld planner v1

The seam search (M4), weld trajectory optimisation (M5), connecting motion
(M6), the continuous verifier and the dense joint-trajectory export behind
offline programming's **Plan**. The OLP packs a `.weldplan`
(`python/weldplan/plan_request.py`) through `seam_worker`, posts it to the
motion server (`python/weld_motion_planner/server/motion_planner_server.py`,
`/api/motion/plan`) and loads the `robot.v4.dense-joint-trajectory.v1` blob it
files. Design and history: `docs/` (`docs/motion/ARCHITECTURE.md`,
`docs/DECISIONS.md`, `docs/issues/`).

## Environments

Install and run through pixi only (the repository `CLAUDE.md` says why):

```bash
cd weld_planner/v1
pixi run -e motion pytest -q tests/motion_planner   # CUDA workstation / Thor
pixi run -e motion-cpu motion-cpu-test              # linux-aarch64 CPU (OLP host)
```

`motion` carries CUDA Torch and NVIDIA Warp: weld search, free-space search and
the planner collision model (ESDF) need it. `motion-cpu` (what a CPU-only
OLP host's planner unit runs) has CPU Torch, SciPy and coal only. It plans
taught moves, dwells and a weld-free Home with the exact continuous verifier;
any program with an approach, weld, transit or retract fails there on
`import warp` until the GPU planner lands (#157).

## Motion server: one process per plan, one warm spare

Every plan still runs in its own process (`--worker-job`), killed when the
requester disconnects, so a plan never shares state with the next. Since
`0ea9ab25` (2026-10-11) the server keeps one spare process that has already
imported what a plan needs (`WARM_MODULES`, `_SpareWorker`, `--worker-spare`)
and hands it the next job; a new spare starts only once that plan's process
is gone, so its imports never compete with a running plan. A dead or missing
spare falls back to a fresh worker. A fresh process paid 2-5 s of imports
per plan with a warm file cache and up to a minute cold on an aarch64 OLP
host. `--no-warm-worker` turns it off; `/health` reports `warm_worker`
(`olp status` shows it). A 5-move taught program (`test1`) on a Rosie 1430
cell's CPU planner plans in 6.4-6.9 s per request measured start, 62 s before
(`0ea9ab25` + `93d1ca79`; `tests/motion_planner/test_server_cancel.py`
covers the spare's lifecycle).

## Where a plan starts, and what each node supports

`start_state.json` in the `.weldplan` is the robot's measured pose, captured by
the OLP from native feedback (`weldplan/measured_start.py`, schema
`rosie.measured-planning-start.v1`). Without it the plan starts at the
description's reset pose (offline preview, or the operator's **Plan from
program start**, after which the apps require Go to plan start before Play).

Since 2026-10-11 every motion node plans from either start (this is a
general purpose robot: it must be programmable from wherever it is):

- **In order.** A measured start, like a taught waypoint, makes the program
  fly node by node in authored order (`measured_start.in_program_order`, used
  by M6, the result's `program_order` and the dense node mapping). Each leg
  leaves where the previous node ended. A weld-only program from the reset
  pose keeps the reordering connector (`freespace.connect`, `order_seams`).
- **MoveJ / MoveL / MoveC** (`engine/taught.py plan_taught`): IK-solved and
  continuously verified from wherever the arm is. A line/arc is first solved
  on one IK knot per 10 mm / 0.02 rad and refined (halving) only when the
  continuous tracking certificate refuses it, down to the 0.25 mm used before
  2026-10-11 (`taught.knot_counts`): `test1` on a Rosie 1430 went from 1683-2785
  knots per line and 63 s of plan to 67-73 knots
  (`tests/motion_planner/test_taught_plan_cost.py`). The certificate's tube
  is `taught.POSITION_TOLERANCE_M` = **0.01 mm** (the arm's quoted
  repeatability is 0.05 mm, or even 0.01 mm; 0.1 mm before) and
  `ORIENTATION_TOLERANCE_RAD` = 0.001 rad (a point 10 mm beyond the TCP moves
  at most 0.01 mm). It bounds the error between samples to second order
  (endpoint errors + sup|e''| w^2/8), so the tighter tube costs fewer FK rows
  than the old first-order one: `test1` ~300 k -> 4-42 k. Measured on `test1`
  at 25 / 250 / 400 mm/s: the spline is 0.11 / 1.1 / 2.6 um off the line,
  and so is what rt-core commands every 1 ms cycle from the dense export
  (0.84 / 1.22 / 2.66 um including whole drive counts) -- after the export
  stopped clamping each segment's last 10 ms row to its end, which made
  rt-core's cubic overshoot each line's end by up to 18.9 um
  (`weldplan.dense_joint_trajectory._resample_segment`: one uniformly
  stretched clock, at most dt/T slower). Weld passes are judged by their
  seam's own authored tolerance (`spec.tolerance.position_mm`, default
  0.5 mm, M5's tracking certificate), not this constant.
  If a taught move refuses after its normal solve/refinement attempts, the
  error identifies the motion type, destination label, authored program step
  and node ID before the original reason. This adds failure context; it does
  not change refinement, feasibility, path tolerances or motion limits.
- **Home**: in a program with no weld block, a MoveJ to the reset pose at a
  recorded waypoint's defaults (`taught.HOME_AS_WAYPOINT`: speed scale 1,
  250 mm/s TCP bound, 25 mm/s before 2026-10-11; the OLP spreadsheet shows it
  on the Home row), judged by the same continuous collision and limit
  verifier, no obstacle search; with welds, a searched free-space transit.
  Skipped when the arm is already exactly at reset.
- **Approach / transit / retract / weld**: the welds keep their M4/M5
  trajectories (spliced only where the arm is at their first knot); windings
  are chosen walking from the real start (`rewound`), the roadmap is built
  around it (`winding_reach`) and seeds every free-space leg; the approach
  leaves the measured pose through the weld's standoff. A retract before a
  waypoint or another approach ends at its departure standoff; a terminal one
  returns home.
- **Dwell**: holds the pose it follows.
- **IO** (`weldplan/io_schedule.py`, design in
  `offline-programming/v1/docs/IO_SCHEDULE.md`): scheduled against the
  connected cell's outputs (`io_outputs.json`, packed by the OLP from rt-core's
  Describe). Each row is a 0.05 s dwell with its marker on the first sample;
  every ON is re-sent after half its expiry and every output still ON gets an
  explicit OFF on the last sample; the header carries rt-core
  `process_markers` and their `process_markers_sha256`. Without the cell's
  outputs, with a channel the cell lacks, or a torch output it has not
  qualified, the row is refused by name (`measured_start.refuse_io`, at pack
  and in M6). Before this the weld-only connector dropped I/O rows without a
  word.

The dense export fails closed (`io/dense_output.py`): `start_state_mismatch`
unless the first sample equals the measured start to 1e-10 rad, and nothing is
exported that is not natively PASS.

Proof: `tests/motion_planner/test_measured_start_any_node.py` (every node mix
from a measured pose, absolute radians, real taught verifier) and
`tools/measured_start_replay.py`, which packs node mixes with the seam
worker's own packer and plans them on the production motion server, judging
the dense blob (first sample, position limits, verdicts, authored order, and
weld samples against the same program from reset). Measured on 2026-10-11 on
this Mac (Docker, 6 CPUs, `motion-cpu`, a Rosie 1430 cell's context from the OLP's
`compile-context`, empty calibration): HOME, MoveJ, dwell, MoveJ, HOME from a
measured pose planned in 3.7 s (planner), first sample 2.8e-17 rad from the
start, J5 0.093 rad inside its +34 degree limit; a waypoint at J5 35.5 degrees
is refused naming the limit. Not a measurement on a cell's OLP host.
