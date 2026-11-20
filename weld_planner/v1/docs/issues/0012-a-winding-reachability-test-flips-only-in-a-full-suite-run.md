# 0012 — A winding-reachability test flips, and determinism is a stated property

**Status:** open. Seen once in four full-suite runs on 2026-08-16; not
reproduced in isolation.

## What happens

    FAILED tests/motion_planner/test_freespace.py::
           test_the_nearest_winding_is_not_always_the_reachable_one
    1 failed, 516 passed, 1 xfailed in 191.55s

The same test passes when run alone (23 s) and passes with the rest of its own
file (19 passed, 57 s). It has only ever failed inside a full `tests/motion*`
run, and only once — the three full runs before it were 517 passed.

## Why it is not just a flaky test

What the test asserts is DISCRETE and is the finding `docs/issues/0009` closed:
on `bracket_a_2x seam_0001`, `J5 = +165` is the nearer winding and has NO route
from the reset pose, while `-195` — the identical robot pose — routes in five
waypoints. `freespace.rewound` consults `roadmap.path` and takes the first
winding that is actually reachable. The test pins that the nearer one is not the
reachable one.

For that assertion to flip, `roadmap.path` must have returned non-`None` for a
winding it otherwise refuses, or `None` for one it otherwise finds. Either way a
reachability answer changed with nothing about the problem changing — and
`roadmap.py`'s own docstring makes determinism a promise:

> **It is deterministic.** A Sobol sequence, unscrambled, not an RNG: planning
> the same plan request twice must give the same program, and a roadmap drawn from
> `random` would quietly break that.

`tests/motion_planner/test_roadmap.py::test_the_same_request_builds_the_same_roadmap_twice`
exists to hold that, and passes. So whatever moved is downstream of the node
set: edge validation, the attachment screen, or the Dijkstra.

## What is known, and what is not

Known: this session measured run-to-run variation of ~0.06 mm in screened
clearance through the self-collision backward's atomics (`BENCHMARKS.md` 12.5
notes it as the reason not to tune weights below a millimetre). A clearance that
sits within 0.06 mm of a screening threshold would make an edge's validity a
coin flip, and `roadmap._validate` thresholds at exactly `> 0.0`.

Not known, and this is the whole issue: whether that is what happened. The
atomics are in the BACKWARD pass and the roadmap screen is forward-only
(`min` reductions under `torch.no_grad`), which should be deterministic. So
either the forward path has a nondeterministic reduction too, or the trigger is
something else about a full-suite run — GPU memory pressure changing a chunk
boundary in `roadmap.clearances`, or state left by an earlier test module.

## What to do

1. Reproduce with a loop: run the full suite N times recording the verdict, and
   separately loop `roadmap.path` on that pair a few hundred times in one
   process and in a fresh one each time. Determinism is cheap to test and this
   has never been tested at the level of the QUERY, only of the node set.
2. If the forward screen is the source, find the reduction and make it
   deterministic — `torch.use_deterministic_algorithms(True)` in the motion
   environment would surface it as an error rather than as drift.
3. If a clearance genuinely sits on the threshold, the fix is not a tolerance:
   it is that a screen deciding reachability must not be evaluated at exactly
   zero. `roadmap.NODE_MARGIN_M` exists for nodes and edges deliberately have
   none (the endpoints of a link are weld poses, millimetres off the work) —
   which is correct, and is also why the edge test has no margin to spare.

Do not widen the assertion. The test is pinning a real property of this cell
and the flip is the thing to explain.
