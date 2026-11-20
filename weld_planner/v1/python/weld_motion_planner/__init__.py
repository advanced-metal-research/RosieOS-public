"""The motion planner: a `.weldplan` in, a verified trajectory out.

## Where to start

    verifier/     the T2 verifier and its evidence. The incorruptible judge,
                  built before the contestants and answering to none of them --
                  which is why it is a folder of its own rather than a file
                  beside the work it judges.

## The planner

    planner/      the stages in ORDER, and both searches they are made of.
                  Read `planner/planner_main.py` first: it is the whole planner
                  in order, and everything below is something it calls.

This planner is TWO searches with a hard division of labour, and two folders
inside `planner/` are that division:

    planner/dp_seam_search/
                  the DISCRETE, global half (M4). The redundancy lattice and
                  the Viterbi sweep over it, exact on that discretization,
                  deciding which islands of the feasible set the trajectory
                  lives in. `dp_seam_search_main.py` is the stage.
    planner/trajectory_optimization/
                  the CONTINUOUS, local half. Weld trajectories (M5), the moves
                  connecting them (M6), and the roadmap those are seeded from.

They live under `planner/` because a search is not machinery anyone else uses:
the only caller of either is a stage in that folder, and the folder that owns
the order should own the searches whose order it is.

Neither substitutes for the other: the gradient stage cannot make discrete
decisions -- the feasible set at a seam point is disconnected islands, one per
IK class, and descent cannot cross between them -- and the graph cannot make
millimetre-smooth trajectories. `ARCHITECTURE.md` argues it at length.

## The rest of the folders

Not searches -- the KINDS of thing both halves are written in terms of:

    geometry/     shapes and fields -- the seam, the part, the distance field
    robot_cell/   the station -- `robot/` the machine, `collision/` what it
                  must not hit and the model the planner searches against,
                  `robot_cell.py` the assembler that joins them. (The
                  VERIFIER's model is the judge's and lives in `verifier/`.)
    io/           the boundary -- a request opened (`plan_request`), an answer
                  declared (`results`) and spoken in JSON (`wire`, the only
                  place degrees are), a debug bundle written (`debug`)
    server/       HTTP, and nothing else. The surface, and what the one POST
                  does to build its answer.
    utils/        instruments with no opinion about welding -- the honest
                  stopwatch (`timing`), the device probe and dtype policy
                  (`device`).

Each folder's `__init__` says what belongs in it. The rule for adding a file is
the rule for reading one: **if it plans, it goes in `planner/` -- in the half it
searches in, or beside the order if it IS the order; if it is a KIND of thing a
search is written in terms of, it goes in the folder for that kind; if it is the
boundary or its shapes, it goes in `io/`; if it judges, it goes in `verifier/`;
and an instrument with no opinion about welding goes in `utils/`.** None of that
is by default: `verifier/` in particular must not sit inside the folders whose
work it judges.

A SEPARATE program from `seam_worker`, joined to it only by the plan request.
`docs/motion/MOTION.md` fixes the seam, `docs/motion/ARCHITECTURE.md` the
design, `docs/motion/IMPLEMENTATION.md` the order it is built in.

The one rule, restated here because this is the package it constrains:

    seam_worker  --depends on-->  weldplan  <--depends on--  weld_motion_planner

Nothing here may import `seam_worker`. Not for the curve evaluator, not for
the frame math, not "just for the test". Where this package needs something
the authoring side also computes, it implements it and checks itself against
an oracle EXPORTED by that side -- committed JSON, generated once -- which is
the same pattern `tests/programmer/test_frame_parity.py` uses to hold the
TypeScript client to `seam_worker`'s `frames.py`. `tests/motion_planner/test_boundary.py` enforces
the arrow by reading the source, so breaking it fails the suite rather than
being discovered by whoever tries to install this without OCCT.
"""

SCHEMA_PREFIX = "amr-weld-planner-v1"
