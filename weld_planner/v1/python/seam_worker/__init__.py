"""CAD to weld program: topology, joint detection, and the weld program contract.

A package, and deliberately an empty one -- no re-exports, and the same is true
of every folder below. Importers name the module they want
(`from seam_worker.seam.detect import detect_joints`), which keeps the
import graph readable and, more to the point, keeps it CHEAP: `cad.topology`,
`cad.tessellate`, `cad.occlusion`, `reach.access` and `reach.clearance` pull in
OCCT and numba, and any `__init__` that imported them would make every one of
the 26 CLI entry points -- and every test that only needs `seam.span`
arithmetic -- pay for the whole CAD stack at import.

## Where to start

    program.py    the CONTRACT. An analysis artifact in, a weld program out;
                  what everything upstream is for.
    engine.py     the analyze pipeline, its warm pool, and the job log
                  clients replay from.
    workers.py    what runs in that pool, and why it is a pool at all.
    paths.py      every filesystem anchor this package reads, in ONE place --
                  read its docstring before computing one yourself.
    server/       STEP in, weld program out. The FastAPI app (`server`, the
                  `python -m` entry point) and one module per resource.
                  HTTP and nothing else.

## The folders

The pipeline in order, then the machine, then what is written down:

    cad/          the STEP and what OCCT gives back -- topology (stage 1),
                  the display mesh, the ray intersector. The expensive
                  imports all live here.
    seam/         topology to the seams you would weld (stage 2) and the
                  geometry they are described in -- curves, frames, spans,
                  scalar profiles
    reach/        can this torch weld it (stage 3) -- access, clearance,
                  angle choice, the torch itself, and what to do where it
                  cannot
    robot/        the cell as data, where its links are, where the part sits
    store/        what outlives a session -- the `.weldproj`, settings,
                  presets

The rule for adding a file is the rule for reading one: a STAGE goes in the
folder for its part of the pipeline, HTTP goes in `server/`, and only the
contract, the runner and the path anchors stay at the top.

A SEPARATE program from `weld_motion_planner`, joined to it only by the plan request.
`docs/GOAL.md` fixes the scope, `docs/motion/MOTION.md` the seam between the
two.

    seam_worker  --depends on-->  weldplan  <--depends on--  weld_motion_planner

Nothing in `weldplan` may import this package, and nothing in `weld_motion_planner`
may either. `tests/shared/test_plan_request.py` enforces both halves by reading
the source, so breaking it fails the suite rather than being discovered by
whoever tries to install a planner without OCCT.
"""
