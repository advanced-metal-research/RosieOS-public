"""The planner: the stages in order, and the two searches they are made of.

Read `planner_main.py` first -- it is the whole planner in order, and everything
else here is something it calls.

    planner_main.py        a `.weldplan` in, a runnable plan out. The ORDER, and
                           the `MotionPlanningOptions` and `MotionPlanningResult` that carry one
                           options object and one result per stage.
    base_types.py  what those pairs inherit: `PlannerModuleOptions`, `PlannerModuleResult`.

    dp_seam_search/            the DISCRETE search and the stage that runs it:
                               `dp_seam_search_main.py` over the redundancy
                               lattice (`nodes.py`) and the Viterbi sweep
                               (`graph.py`). M4.
    trajectory_optimization/   the CONTINUOUS search: `weld_trajopt.py` smooths
                               each weld into a curve (M5) and
                               `connecting_trajopt.py` gets the arm between them
                               (M6), over `freespace.py` and `roadmap.py`.

Inside each search folder the same split repeats, and it is worth holding while
reading: a `*_main.py` is the STAGE -- it opens a request, hands the search the
settings for it, and turns what comes back into a result -- and everything
beside it is the search, which knows a lattice or a cost function and nothing
about `.weldplan` files or about how many seams there are. So `graph.search`
answers "how do I cross this lattice" and `DpSeamSearchModule` answers "do
that for every weld in this request, and keep the failures".

Outside this folder is what BOTH searches are written in terms of -- `geometry/`
holds the shapes, `robot_cell/` the station and the model it is asked about,
`io/results` the shapes an answer takes -- plus the judge (`verifier/`), `io/`'s
boundary code and `utils/`'s instruments, none of which makes a plan.

## Why the order needs a file of its own

It used to live in the HTTP endpoint, and before that in three places at once.
The copies drifted -- one screened postures against the cell, one against the
part alone, one not at all -- and the bug that let candidates rest inside the
positioner beam lived in the copy nobody re-read. An order assembled at each
call site is an order with no single answer to "what does the planner do".

The same drift was still here in a quieter form: all three stages built their
own collision model from the request, and only the first honoured a caller's
`mesh_root`. So a run could screen the welds against one set of meshes and the
moves between them against another, and nothing would say so. `planner_main`
builds the world ONCE and hands it down, which is most of why it exists.
"""
