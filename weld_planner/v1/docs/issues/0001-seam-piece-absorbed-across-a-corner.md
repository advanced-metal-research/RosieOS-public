# 0001 · A sub-millimetre stub was absorbed across a 90° corner

**Status:** fixed in `weld_programmer.reach.access`, `weld_programmer.seam.seams`,
`weld_programmer.seam.curve` and `weld_motion_planner.geometry.curves`, fixtures regenerated. **8 of 15
seams found a path before; 14 do now.** The one that does not is a different
bug — see 0002. Three faults, not two: the follow-up this issue left open was
real and is written up below as **Three**.
**Found:** 2026-08-12, while checking M4's "every fixture yields a continuous
K-best set" criterion for the first time.
**Blast radius before the fix:** 5 of the 15 seams in the committed corpus
could not be planned at all — `bracket_a` 1 and 3, `bracket_b` 2,
`pipe_bracket` 1 and 3 — plus `dog_box` 2.

## What is wrong

`bracket_a`'s `weld_joint_0001` is a 190 mm three-sided path with three 90°
corners, split into three seams. The splits are correct: every corner is a
split point. But `seam_0001` **starts 27 µm before the first corner instead of
at it**, so its first 27 µm lie on the far side of a 90° turn.

    corner      s = 0.0789375
    seam start  s = 0.0787930      <- 27 µm earlier

A motion planner sampling that seam puts its first sample in the 27 µm stub
and its second past the corner, so between two samples 2.6 mm apart the torch
is asked to swing 60° of electrode direction. No arm does that at the authored
travel speed, so no edge can be formed and the search stops at the first
layer.

Twenty-seven micrometres is not a weld. It is not a length anybody authored,
measured, or intended. It made the seam unplannable.

## How it was found

Not by a test — every test passed. The redundancy graph reported a frontier at
layer 1 on nine of fifteen seams, and "no edge at layer 1" reads as a robot
problem. Three wrong diagnoses came before the right one:

1. **The IK branch identity.** Adjacent samples hold postures 66–229° apart in
   the same lattice cell, which is true and is a real defect, so it looked
   like the cause. Warm-starting from the previous layer's answer was the
   fix — and it did not help: median step 113°, nothing inside 25°.
2. That failure is what pointed at the target. A solver seeded **at** the
   previous answer cannot move 113° unless the target moved, so the target got
   measured instead: the electrode direction turns 60° while the tip travels
   2.55 mm.
3. **"The seam has a corner in it."** True, and reported as such — but the
   operator reading that would go and split the seam, and the seam is already
   split. The corners and the split points had to be compared numerically
   before the 27 µm showed up.

The general lesson is in the ordering: the graph's refusal was correct at
every step, and three layers of correct behaviour sat between the symptom and
a 27 µm number in a data file.

## Why it happens

`python/weld_programmer/seams.py`, `_absorb_short_pieces`.

Its own docstring describes this exact situation and gets the diagnosis right:

> A run boundary is measured by bisection in `access.py` (~0.06 mm) and a
> corner by summing declared lengths, so where a gusset ends exactly at a
> corner the two disagree by a hair… the stub is measurement noise about where
> one boundary is, not a separate stretch of weld, so it is absorbed into the
> piece it touches.

Every clause of that is correct. The action is wrong. Given

- an **exact** boundary (the corner, from summed declared lengths), and
- a **noisy** one (the reachability run edge, bisected to ~0.06 mm),

which disagree by less than the noise, absorption **extends the piece across
the exact boundary**. It should move the noisy boundary onto the exact one.
Absorbing forward is right when the stub's far side is an ordinary junction;
across a C0 corner it manufactures a seam whose first sample is on the wrong
face of a 90° turn.

`_absorb_short_pieces` could not tell the difference because it was not given
the breaks — it saw only a list of intervals.

## What was done

The 27 µm turned out to be two faults stacked, and the second only became
visible once the first was fixed.

**One: the boundary was never snapped.** `access._snap_to_corners` now moves a
blocked-span boundary onto a corner it landed within its own bisection error
of (2 mm sample step, five halvings, ~0.06 mm). It lives in `access.py` and
not in `seams.py` because `blocked_length_m` and `reachable_runs` are both
derived from those spans — moving a boundary there moves both, and
`weldable + blocked == the whole boundary` still holds exactly. Snapped in
intent generation instead, the seam would shrink while the blocked total
stayed put.

**Two: rounding put it straight back.** `access._clear_runs` and
`seams.seams_for_joint` both rounded span boundaries to six decimals. These
are NORMALIZED positions, so a decimal is a decimal of the whole weld line —
six of them quantize a 190 mm curve to 0.19 µm. Twelve decimals was tried and
still landed 5×10⁻¹⁴ short of the join, which is the point: the coarseness is
not the problem, the DIRECTION is. Anything that can round down can round down
past a corner. Both sites now emit the float they computed. Rounding was for
byte-stable plan requests; float64 round-trips exactly through `json.dumps` and
everything upstream is deterministic from the STEP, so the bytes are stable
without it.

Fixed upstream and not in `weld_motion_planner`, though a reconcile there was written
first and thrown away. It made the motion side evaluate the span at the join
while the authoring side still evaluated it at the rounded value, and the
committed seam oracles caught that immediately — which is what they are for.
One number, seen the same way by both sides, is the only version of this fix
that is not two versions of it.

**Three: landing on the corner was not the same as being read on it.** This is
the follow-up this issue left open, and it turned out to be one bug rather than
the two candidates listed. Once a boundary sits exactly on a corner, something
has to decide which SIDE of it to read the tangent from — the two answers
differ by the whole corner angle — and `Curve.locate` decided by comparing
`s × total` against a cumulative sum of quadrature lengths. Both sides of that
comparison are the right number and they are not the same float. `s` is the
authoring side's running sum of the segments' declared `length_m` over their
total; quadrature measures the same arcs independently and answers up to 0.9 nm
differently on `dog_box`'s rounded lobes and one ulp differently on a straight.
So a boundary lying exactly on a corner missed it by a hair, in whichever
direction nothing controls, and the half-open `[start, end)` rule that was
supposed to resolve the tie never saw a tie to resolve.

`Curve.joins_s`, on both sides, now holds the join positions on the scale they
were AUTHORED on — the declared lengths — and a position within 10⁻¹² of one is
on it. Arc length keeps the job only it can do, placing a position that is not
a join. `left_limit` then says which side, and it is a statement rather than an
inference: a seam ARRIVES at its last point and departs every other one, and at
`s = 0` of a closed loop arriving means the last segment. The authoring side
gained the same argument, so `frame_at` and the seam oracle can state the
convention instead of inheriting a rounding, and the parity test in
`tests/motion_planner/test_curves.py` no longer steps around joins — it asserts them.

This is why the split was never the thing to change. Every corner in the corpus
was already a split point, and every seam already started and ended on one. The
seam that reported a 90° swing between its first two samples was correctly cut;
it was being read on the wrong side of its own first corner.

### Measured

Seams that find a path, across the whole committed corpus, at a 25° edge
threshold: **8 of 15 → 10 of 15** for the two faults above, and **10 → 14** for
the third. Verified byte-identical across two fresh processes and pinned by
`tests/motion_planner/test_graph.py::test_the_whole_corpus_crosses_except_the_one_that_is_the_arm`.

The four the third fault held back are exactly the four this issue predicted:
`bracket_a` 2 and `bracket_b` 2, which stopped at their last layer, and
`dog_box` 2 and `pipe_bracket` 1, which stopped at their first. Every one of
them reported 60.0° of electrode swing across 0.32 to 4.84 mm of tip travel,
which was a corner that was not there.

Four values moved and each is pinned with its reason: blocked length
59.75 → 59.87 mm, `pipe_bracket` seam 11 coverage 0.781 → 0.775, pieces
absorbed 4 → 0, and the conservation tolerance from 1 nm to 1 µm — the last
because nanometre agreement between two independently rounded sums was a
coincidence, not an invariant.

## What is still owed

- One seam, `dog_box` 1, and it is not this bug. Split out as **0002**.
- `access._absorb_slivers` solves the same problem one stage earlier and was
  not audited for the same mistake.
- Nothing searched for other six-decimal roundings of normalized parameters.
  Two were found by following one bug; there is no reason to think that is all
  of them.
- Nothing searched for other places a normalized position is compared against
  a quadrature sum either — `joins_s` fixed the two `locate` implementations,
  and `clearance.py` and `access.py` both do their own arithmetic on `s`.

`graph.Graph.frontier_demand` stays regardless. A seam can legitimately
contain a corner when the corner is under the split threshold, and the report
is the only thing that distinguishes that from a robot that cannot reach.
