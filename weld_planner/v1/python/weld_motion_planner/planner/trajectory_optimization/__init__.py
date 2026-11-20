"""The CONTINUOUS, local half: a lattice path in, an executable trajectory out.

    weld_trajopt_main        M5's stage: options in, models handed over, the
                             winner certified. `planner_main` calls this.
    connecting_trajopt_main  M6's stage, in order: rewind every weld onto a
                             consistent turn of each wrist axis, then connect
                             them. Also `planner_main`'s.
    options                  both stages' settings, importable without the
                             GPU stack.
    engine/weld_trajopt
               M5: the WELD trajectory. Batched projected gradient over
               cubic-Hermite knots, every seam and every candidate in one
               tensor, with the seam held by PROJECTION rather than by
               penalty -- a penalty leaves millimetre tracking error at
               convergence, which is the scale that matters.
    engine/freespace
               M6's machinery: the moves BETWEEN the welds. The same solver
               with the endpoints clamped in position AND velocity, so the arm
               reaches a weld already moving at that weld's own speed. Timing
               is free here and nowhere else in this planner.
    engine/roadmap
               the deterministic PRM `freespace` seeds from when descent
               cannot reach where it needs to go. It says of itself that it is
               a seed generator: it screens at samples, which this planner
               refuses as proof everywhere else.

Local, and that word is the constraint the whole folder is shaped by. Descent
cannot leave the homotopy class its seed is in -- it can only make that class
tidier -- which is why the seeds are many and shaped deliberately, why
`roadmap` exists at all, and why `freespace.shortcut` is a discrete jump rather
than another gradient term. `docs/issues/0011` is the worked example of
mistaking a frozen optimizer for a converged one.

Nothing here certifies its own output. `verify.py` does, and it sits outside
this folder for that reason.
"""
