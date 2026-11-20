# The motion planner, and the line it sits on the far side of

**Nothing in this directory is built.** This is a charter for the thing that
will consume what `amr-weld-planner` produces, written now because the shape of
the boundary is already fixed by the code that exists — and because a boundary
nobody wrote down is a boundary that erodes the first time somebody is in a
hurry.

`GOAL.md` stage 4 is the planner. Its non-goals list has always excluded "a
motion planner consuming the weld program" from THIS module, and that stays
true: the planner is a separate program, expected to be its own Python package
in the shape of NVIDIA's cuRobo. What changed is that the seam it attaches to
is now explicit rather than implied.

## The contract is a file, not a function call

Everything crossing the line crosses as one artifact: a `.weldplan` zip, schema
`amr-weld-planner-v1.plan-request.v1`, built by `POST /api/plan-requests` and
read by `weldplan.read_request`. It carries five things — the weld program
verbatim, the placement against the cell's work frame, the cell descriptor
inlined with meshes stripped, the torch, and the source STEP. `plan_request.py`'s
module docstring argues for each one and for the two deliberate omissions (cell
collision meshes, and any joint value at all). Read that before changing this.

The important property is that it is DATA. A planner does not link against this
module, does not import it, and does not need its environment: OCCT, numba and
cadquery are the authoring side's problem. It opens a zip of JSON.

## The one rule

**`weldplan` imports the standard library and nothing else, and nothing in it
may ever import `seam_worker`.** The arrow points one way:

    seam_worker  ──depends on──▶  weldplan  ◀──depends on──  a motion planner

That is what makes the reader cheap and the producer replaceable. It is the
same rule `GOAL.md` states for `cadquery/v1` and `tesseract/v1` — read the
data, do not import the module — applied to a boundary inside this repository
instead of one at its edge. `tests/shared/test_plan_request.py` enforces both
halves, so breaking it fails the suite rather than being discovered later by
whoever tries to install a planner without OCCT.

If a planner turns out to need something from the authoring side that is not in
the plan request — seam frames, span arithmetic, the scalar profiles — the answer is
NOT to import `seam_worker` for it, and not automatically to move it into
`weldplan` either. Decide then, with a real consumer in front of you: the
question is whether the thing is part of the CONTRACT (both sides must agree on
it, forever) or an implementation detail that happens to be useful. Only the
first belongs here. `placement_problems` is the worked example — what makes a
placement record well-formed is contract, so it lives in `weldplan` and
`seam_worker.robot.placement` re-exports it.

## Where it will live

Nothing is created for it yet. When it starts, the two plausible homes are:

- `python/weld_motion_planner/` in this same pixi environment, if it stays small enough
  that sharing the environment is a convenience rather than a constraint; or
- its own `<command>/<version>` directory in the repository, which is the house
  style in `README.md` and the right answer as soon as it needs GPU
  dependencies, a different Python, or its own deployment.

The second is more likely for a cuRobo-shaped package, and the contract above
is deliberately indifferent to the choice — which is the point of making the
seam a file.

## Prior art in this repository

- `tesseract/v1` — TrajOpt cartesian planning and collision checking, pixi
  Python, JSON in and JSON out. Not to be imported (`GOAL.md` forbids it);
  the established way to reach it is to shell out to
  `rosie.sh tesseract v1 plan-cartesian`.
- `motion-server/v1` — the online/realtime cartesian authority, and the
  collision authority when a cell is connected.
- `WEAVE-AND-VERIFICATION.md`, beside this file — why weave is a parameter
  rather than a list of waypoints, the velocity-budget arithmetic, and what
  simulation cannot prove. It is planner-side guidance and always was, which is
  why it sits here rather than with this module's own docs.
