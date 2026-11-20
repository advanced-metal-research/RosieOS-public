"""The DISCRETE, global search (M4): which islands the trajectory lives in.

    dp_seam_search_main
             the STAGE, and the only file here that knows what a `.weldplan`
             is: it opens a request, runs the engine for every seam, and
             keeps a seam's failure as a result rather than as an exception.
             `planner_main.py` calls this and nothing else in the folder.
    options  the stage's settings, importable without the GPU stack.
    engine/nodes
             the redundancy lattice. At every seam sample the program pins 5
             DOF and leaves free the torch spin, the positioner, and --
             discretely -- which IK solution the arm uses. A node is one such
             choice, screened, with a cause recorded for every rejection.
    engine/graph
             the sweep over that lattice. Which nodes can follow which (an
             edge is physics, not tuning: 5 mm of travel cannot slew 30
             degrees of spin), the K best ways across in one Viterbi pass, and
             the reachable FRONTIER when there is no way across at all.

**Why this is a separate search and not an initialisation trick.** The feasible
set at one seam point is not a blob but disconnected islands, one per IK class.
A gradient method seeded in one island can never cross to another, and the best
plan may need a different island -- or a different positioner value -- halfway
along the seam. So the discrete decisions are made here, exactly, on a
discretization; `trajectory_optimization` polishes inside the basin this
chose. Neither substitutes for the other.

Failure is a RESULT here, not an exception: when no path crosses all layers the
sweep reports the furthest layer any path reached and, at that frontier, why
every next-layer node died -- as counts, by cause. That is what an operator can
act on, and it is what the split protocol upstream is argued from.
"""
