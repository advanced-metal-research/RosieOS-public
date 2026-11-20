# 0004 · Bin spacing wider than the edge threshold degrades or removes the axis

**Status:** fixed. Defaults resized, `graph.pinned_axes` reports the condition,
`graph.bins_for` sizes it, `graph.escalate` climbs it, and the corpus test sizes
its bins from its own threshold.

**Two corrections to this issue's first version, both from `docs/issues/0005`:**

1. **The headline was confounded.** It read "the corpus goes 14 of 15 to 15 of
   15". That was measured while every seam was being welded at 0°/0° instead of
   the angles it authors. At the authored angles the coarse lattice crosses 15
   of 15 as well, so the corpus does not separate the two fixes. The mechanism
   below is measured directly instead, and does not need a failing seam.
2. **"Spin can never change bin" was wrong**, and so was the first attempt to
   explain why. Spin is not a component of the search state — it is a parameter
   of the TARGET, reaching the state only through inverse kinematics, so `x`
   degrees of spin is not `x` degrees of any joint. The `dog_box` seam 1 path
   crosses **four** spin bins at 45° spacing, and it does it at layer 4 with the
   **branch unchanged** and 21.3° on J6 as its largest move. The "paid jointly
   with a branch change" story in the previous revision was reasoning, not
   measurement, and the measurement says otherwise. See **What spin actually
   costs** below.

What is exact, and is what the test asserts: the **positioner** is a state
component compared against its own threshold, so bins further apart than it
cannot be joined. The path through `dog_box` seam 1 uses **one** positioner bin
at 60° spacing and **six** at 24°.

**Found:** 2026-08-12, while measuring whether the warm start proposed in 0002
was worth its dependency. It was not, and neither was the alternative — the
binding constraint was neither.
**Blast radius:** every search ever run. The redundancy the positioner provides
was not available to the search that exists to exploit it.

## What is wrong

An edge exists between two lattice nodes only where **every** component of the
state moves less than its threshold. The state is the arm's six axes followed by
the positioner. So the POSITIONER, binned further apart than its threshold,
can never be crossed: the search picks a bin at layer 0 and welds the entire
seam in it. Spin is degraded rather than removed — see the correction at the
top.

The shipped numbers:

| | bins | neighbours | edge threshold | can the search move it? |
|---|---:|---:|---:|---|
| positioner `J7`, shipped | 9 | 40.0° | 25° | **no**, exactly |
| positioner `J7`, corpus test | 6 | 60.0° | 25° | **no**, exactly |
| spin, shipped | 12 | 30.0° | 25° | sometimes; see below |
| spin, corpus test | 8 | 45.0° | 25° | sometimes; see below |

The lattice is `samples × spin × positioner × branch` and reports four
dimensions. The positioner was frozen at whatever bin layer 0 landed in; spin
was crossable only where the geometry happened to allow it.

## What spin actually costs

Spin is a parameter of the target pose, not a joint, so the question "can the
search change spin bin" is really "how far apart in JOINT space are two nodes
one spin bin apart". Measured on `dog_box` seam 1:

| spin bin spacing | pairs | min | median | inside a 25° threshold |
|---:|---:|---:|---:|---:|
| 45.0° | 611 | 22.97° | 51.43° | **1.3%** |
| 24.0° | 1541 | 11.82° | 27.51° | **39.3%** |

Not zero and not one-to-one. At 45° spacing the typical neighbour is 51° away —
more than the spin change itself — but the minimum is 22.97°, and the search
takes those. The chosen path's spin bin goes 3 → 4 at layer 4 with the branch
index staying at 0 and per-joint moves of 1.3, 6.4, 4.4, 11.0, 6.5 and 21.3
degrees, all inside the threshold.

Part of why the map is not one-to-one is a fact about this cell: **the electrode
sits exactly 45.000° off J6's axis** at every posture, measured. A roll about the
electrode is therefore not a wrist-roll — it is distributed across J4, J5 and J6,
and how much of it lands on any one of them depends on where the arm is.

So spin is a **degraded** axis at 45° bins rather than an absent one: crossable,
but only where the geometry cooperates, which is 1.3% of the neighbours instead
of 39.3%. `pinned_axes` reports it on that basis and asserts nothing.

## How it was found

By trying to justify a fix for a different issue, and failing.

0002 says adjacent samples in the same lattice cell hold postures far apart, and
proposes warm-starting each layer from the previous one so the branch index
names a consistent sheet. The counter-argument came from the owner and is the
right one: warm-starting makes layer `N`'s node set depend on layer `N-1`'s
answer, and the whole reason there is a dynamic program here is to be free of
that kind of ordering. The alternative proposed alongside it — more random
restarts, no dependency, and an honest refusal when it still fails — was
measured first because it is the cheaper claim to test.

**Both are beside the point, and the measurement is what says so.** On
`dog_box` seam 1, adding random restarts to the eight structured seeds:

| restarts | feasible nodes | frontier |
|---:|---:|---:|
| 0 (shipped) | 11.1% | 28 |
| +8 | 29.2% | 28 |
| +24 | 54.2% | 28 |
| +56 | 64.3% | 28 |
| +120 | 64.9% | 28 |

Six times the nodes and **the wall does not move a layer**. From layer 22 on,
the reachable count per layer is bit-identical between the 0-restart and
56-restart runs: 4, 2, 2, 1, 1, 1, 0.

That is what pointed at the spacing. Tracing the corridor:

    layer  0 ->  1: live  41 ->  23   cheapest step from live   4.72 deg
    layer  2 ->  3: live  18 ->  16   cheapest step from live   3.90 deg
    layer 22 -> 23: live   4 ->   2   cheapest step from live  19.22 deg
    layer 25 -> 26: live   1 ->   1   cheapest step from live  11.94 deg
    layer 27 -> 28: live   1 ->   0   cheapest step from live  52.18 deg

Meanwhile the cheapest **feasible-to-feasible** step at every one of those
layers is 3.3° to 5.1°. Nodes a few degrees apart exist at every layer and the
search cannot use them, because reaching them means changing bin on an axis
binned wider than the step it is allowed.

## Measured

`dog_box` seam 1, 32 samples, 25° threshold, everything else held:

| spin | positioner | spin gap | pos gap | feasible | frontier | peak |
|---:|---:|---:|---:|---:|---:|---:|
| 8 | 6 | 45.0° | 60.0° | 11.14% | **28** | 61 MiB |
| 16 | 6 | 22.5° | 60.0° | 11.55% | none | 96 MiB |
| 8 | 16 | 45.0° | 22.5° | 11.13% | none | 143 MiB |
| 16 | 16 | 22.5° | 22.5° | 11.47% | none | 461 MiB |
| 32 | 32 | 11.2° | 11.2° | 11.47% | none | 6789 MiB |

Loosening **either** axis is enough. The feasible fraction moves by four tenths
of a percentage point across the whole table, which is the point: this was never
about finding more nodes. It is about the nodes being spaced closer than the
step limit so that a path can exist between the ones already there.

The whole corpus, 15 seams, 25° threshold, **at 0°/0° weld angles**: 14 of 15 →
15 of 15. At the angles the seams actually author, the coarse lattice crosses 15
of 15 too — see the correction at the top. The row that carries the mechanism is
the one above: same seam, same threshold, feasible fraction unmoved, frontier
gone when the spacing drops under the threshold.

GPU cost is not the constraint and never was. This machine is a 31.3 GiB
RTX 5090; the shipped 64×12×9 lattice peaks at 168 MiB to build and 129 MiB to
search — half a percent of the card. The fix costs about 1.5%.

## Why it happens

Nothing connected the two numbers. `nodes.build` chose bin counts and
`graph.search` chose a threshold, in different modules, neither aware that the
product of the two decides whether an axis exists. Both were defensible alone:
twelve spin bins is a reasonable sampling of a turn, and 25° per joint between
samples 3 mm apart is a reasonable rate limit.

Nothing reported it either, which is what made it survive. A pinned lattice and
a healthy one produce the same summary — same shape, same node count, same
feasible rate — and differ only in an answer nobody was asking for.

The same shape as `docs/issues/0003` and `0005`: a value chosen by nothing,
reported by nothing.

## What was done

- **`graph.bins_for(threshold_rad, span_rad)`** — the smallest bin count whose
  neighbours are strictly inside the threshold. Strictly, because the edge test
  is `<=` per component and an axis that spends the whole budget alone leaves
  the arm nothing to move with at the same time.
- **`graph.pinned_axes(lattice, threshold_rad)`** — which binned axes the search
  cannot move along, with the gap, the threshold, and the bin count that would
  fix it. **Exact for the positioner**, which is a state component compared
  against its own threshold. **Advisory for spin**, which is not a state
  component — it enters through the arm joints it moves, so a spin change can be
  paid jointly with a branch change and a wide spin bin degrades the axis rather
  than removing it.
- **`Graph.pinned_axes` and `summary()["pinned_axes"]`** — carried whether or
  not the seam crossed. A seam that crossed on a pinned lattice crossed with one
  hand tied, and a seam that did not is not evidence about the robot until this
  is empty.
- **Defaults raised** to 16 spin and 16 positioner bins, 22.5° apart.
- **The corpus test sizes its bins from its own threshold** rather than typing
  in both, and asserts `pinned_axes == {}` alongside 15 of 15.

## What this says about 0002

0002 is not refuted — adjacent samples really do hold postures far apart in the
same lattice cell, and a branch index really does name different sheets at
different layers. What is refuted is that it is **why seams fail**. The seam
0002 was filed against crosses now, with the same eight structured seeds and no
warm start, because the search was given room to roll the torch.

So the warm start stays unbuilt and the objection to it stands on its own
merits: it would trade a property the dynamic program is there to provide for a
fix to something that was not the problem.

## Why restarts and finer bins are not the same lever

They add nodes in different places, and the edge rule only cares about distance.

- A **restart** fills a BRANCH slot in a cell that already exists: another root
  of the *same* pose.
- A **finer bin** creates a NEW cell: the same root of a *nearby* pose.

Measured on `dog_box` seam 1, distance between two feasible postures:

| | pairs | min | median | within 25° |
|---|---:|---:|---:|---:|
| same cell, different branch | 1208 | **180.00°** | 360.00° | **0.0%** |
| neighbouring spin bin, 45° apart | 611 | 22.97° | 51.43° | 1.3% |
| neighbouring spin bin, 24° apart | 1541 | 11.82° | 27.51° | **39.3%** |

Not one of 1208 branch pairs is inside the threshold, and there is no number of
restarts that would change it: distinct roots are separated **by construction**
— that is what "disconnected sheets" means. Bins are separated by a number
somebody chose, and choosing a smaller one moves 1.3% to 39.3%.

Median 27.5° against a 24° bin gap is the second thing worth reading here: a
roll about the electrode does not turn the wrist by the same amount, it turns it
by about that away from a singularity and by more near one. So halving the bin
gap does not halve the posture gap, and the relation between them is the
Jacobian rather than a constant. This is why `pinned_axes` reports spin
approximately and the positioner exactly.

## Adaptive refinement, and the criterion it would need

Refining only where the lattice is thin is the right shape for this, and the
obvious criterion is the wrong one. **The failure is not "no solution found."**
Layer 28 had 47 feasible nodes, its neighbours had 47 and 49, and the per-layer
feasible count across the whole failing region is flat: 43, 46, 57, 47, 47, 49,
47. Refining where solutions are missing would refine nowhere.

What actually degrades is the CORRIDOR, and it degrades with warning:

    reachable  41  23  18  16 ...  4   2   2   1   1   1   0
    cheapest step from the reachable set, deg
              4.7 4.2 3.9 ...     19.2 9.4 10.5 11.9 14.7 52.2

Six layers of the step cost running at half the threshold before the wall. So
the trigger is reachable count and step margin, both of which the search already
computes; `Graph.reachable` is reported per layer and the margin is not.

**It is not worth building yet, and the number that says when is the threshold.**
Uniform refinement costs the square of the number of LIVE nodes per layer —
about 13% of the slots on this corpus, since `search` compacts to feasible nodes
before building anything. Measured on `dog_box` seam 1, peak allocation for the
whole build-and-search:

| bins | gap | build peak | search peak | time |
|---:|---:|---:|---:|---:|
| 15 (today) | 24.0° | 0.1 GiB | 0.02 GiB | 0.7 s |
| 64 | 5.6° | 2.41 GiB | 0.75 GiB | 9.0 s |
| 96 | 3.8° | 5.42 GiB | 0.83 GiB | 20.8 s |
| 128 | 2.8° | 9.63 GiB | 0.94 GiB | 37.6 s |

Two exact changes took the ceiling from 43 bins to past 128, neither of them an
approximation:

**Compaction.** The transition was built over every SLOT and the infeasible ones
masked afterwards. 13% of slots are feasible on this corpus, so 98% of the table
was pairs with a non-existent end. Building it over
`torch.nonzero(feasible)` first makes the layers ragged and drops the table 53×
at 15 bins and 58× at 43.

**Destination chunking.** `topk` runs per destination over all sources, so
slicing the destination axis is an identity — every destination still sees every
source. That caps the transition at `chunk_bytes` regardless of bin count, and
the search's peak stops growing at all: 0.75 GiB at 64 bins, 0.94 at 128. Before
it, the search was the largest thing in the process at 96 bins (9.34 GiB against
the build's 5.42) while costing 3% of the wall time.

Slicing is not free in principle — a slice small enough that the card idles
between launches is catastrophic, 25× at one destination per slice — and the
curve has a bottom rather than a slope. At 96 bins:

| budget | destinations/slice | slices | elements/slice | time | peak |
|---:|---:|---:|---:|---:|---:|
| 1 MiB | 1 | 9,731 | 68,117 | **15.749 s** | 0.15 GiB |
| 16 MiB | 11 | 885 | 749,287 | 1.635 s | 0.15 GiB |
| 64 MiB | 45 | 217 | 3,065,265 | 0.497 s | 0.19 GiB |
| **256 MiB** | 181 | 54 | 12,329,177 | **0.337 s** | 0.32 GiB |
| 1 GiB | 725 | 14 | 49,384,825 | 0.445 s | 0.83 GiB |
| one slice | 9,731 | 1 | 662,846,527 | 0.609 s | 9.34 GiB |

Saturation arrives around three million elements — about 140 per core on this
card — and past it more parallelism buys nothing while the allocation costs
more. The single-slice version is the SLOWEST of the sane options as well as the
largest. The default is 256 MiB, which is the bottom on both axes at once.

What remains is the LATTICE BUILD's inverse-kinematics batch, which grows as
bins² and can be chunked the same way whenever it needs to be, and TIME, which
is the real limit first: 37.6 s a seam at 128 bins.

**Pruning by positioner bin index** is exact and available and is NOT
implemented, because it would help the wrong thing. The positioner is a state
component with its own threshold, so `|turn_i - turn_j| > threshold` is a
guaranteed non-edge computable from bin indices alone — no arm involved. It
removes a fixed 86% of pairs (the surviving fraction converges to
`2 × threshold / 360` = 13.9%, measured at 19.1% for 15 bins down to 13.1% for
96). But chunking already caps memory, and the search is 3% of the wall time, so
a 7× saving on it buys 2.6% overall. It is the right tool if search time ever
becomes the constraint, and today the constraint is the build.

At today's 25° uniform costs a tenth of a gigabyte and 600 ms, and refining
*within* a seam — a ragged lattice, finer bins only near the thin layers — would
buy back a fraction of that for a two-pass search and a much harder invariant.
Below about 10° it becomes worth it and below 7° there is no alternative. That
is not hypothetical: a 90°/s joint with a 1 mm sample step at 8 mm/s travel
gives an 11.2° threshold, and deriving thresholds from real rates is what M7 is
for.

**What is built instead, and it is the cheap 80%: `graph.escalate`.** Refine per
SEAM rather than per layer. Rung one is `bins_for` — the coarsest grid at which
no axis is pinned, hence the coarsest grid whose answer means anything — and a
seam that crosses there never pays for anything more. A seam that fails gets a
grid twice as fine, and then the ladder stops on either of two conditions:

- **The wall did not move.** Same frontier at half the spacing means the
  obstacle is not the spacing. `bracket_a_placed` seams 5 and 7 do exactly this:
  layer 1 at 15 bins, layer 1 at 30 bins, both reporting 60.0° of electrode turn
  across 0.32 mm of tip travel with `is_a_corner` set.
- **The next rung does not fit.** `transition_bytes` predicts the allocation and
  the rung is recorded as not run, with the size it wanted. Predicted rather
  than caught, because an out-of-memory says something was too big and not
  which rung, and a ladder ending in an exception is not evidence about a seam.

Measured on the corpus at both 25° and 15° thresholds: every seam crosses on
rung one, so the ladder costs nothing in the common case. It only ever fires on
the seams that are genuinely stuck, and there it fires twice and stops.

The first version of `transition_bytes` was wrong low by 3.75× because it
counted the named tensors and not the intermediates — `delta` costs two copies,
the compare and the weighting one each, and `step` about three. Wrong low is the
direction that attempts a rung that cannot run, which is the whole point of the
function, so it is fitted against the allocator and pinned by a test.

## Why jitter is not the cheap way out

Proposed and measured: keep a cheap 32×32 grid but offset it by a random
fraction of a bin each run, so the union over runs is dense and a solution that
exists is eventually found. That is how sampling planners earn probabilistic
completeness. It does not transfer, for two reasons, and the second is the one
that generalises.

**An offset moves where the bins are, not how far apart they are.** What fails a
seam is an axis whose neighbours are further apart than the edge threshold, and
that gap is identical at every offset. Twenty seams at a 10° threshold:

| | crosses | time |
|---|---:|---:|
| 32 bins, fixed | 19 of 20 | 3.50 s |
| 32 bins, 8 offset runs unioned | **19 of 20** | 25.85 s |
| **37 bins = `bins_for(10°)`** | **20 of 20** | **3.85 s** |
| 128 bins, fixed | 20 of 20 | 58.82 s |

Twelve percent more bins beats eight times more runs, and beats the brute-force
grid by fifteen times the speed. At 32 bins every one of the twenty seams
reports a pinned axis; at 37 none of them does.

**The union across runs is not a path space.** Each run's dynamic program is
self-contained — layer 5 of one run and layer 6 of another have no edge between
them, in either lattice. A sampling planner accumulates into ONE graph, which is
exactly why more samples densify for it and not for this.

What this planner has instead is **resolution completeness**, and it is
deterministic rather than probabilistic: with the gap under the threshold the
search can track any continuous solution to within one bin, and one bin is by
construction less than the arm can absorb between two samples.

`nodes.build` keeps `spin_offset_bins` and `positioner_offset_bins` anyway,
because a different question is worth asking — whether a result depends on where
the bins happened to land — and answering it needs exactly this.

### The offset is a fraction of one bin, and it used to eat the band prune

They took RADIANS until 2026-08-17, and a radian is not a unit the grid has:
whether a given number was under one bin depended on the bin count, which the
same caller also chose. They are `spin_offset_bins` and
`positioner_offset_bins` now, a fraction of ONE bin in `[0, 1)`, and past a bin
they are refused — on an axis that closes a whole bin is the grid back on
itself, so a run meant to be a different offset silently repeats the last one.

**A capped axis cannot be shifted at all.** `endpoint=True` already puts a bin on
`low` and on `high`, so no shift moves every bin up without taking the top one
past the cap. The old code clipped it back, and that did not move the grid — it
collapsed the MINIMUM gap, which is the number `graph.search` sizes its band
prune from (`bin_gap`, and `reach_bins = floor(threshold / bin_gap)`). Measured
on the real 90-bin capped grid at a 25° threshold:

| offset | min gap, clipped | pairs pruned | min gap, now | pairs pruned |
|---:|---:|---:|---:|---:|
| 0 | 2.0225° | 72.2 % | 2.0225° | 72.2 % |
| 0.5 bin | 1.0112° | 45.6 % | 2.0111° | 72.2 % |
| 0.9 bin | 0.2022° | **0 %** | 2.0020° | 72.2 % |

So an offset of nine tenths of a bin turned the prune off completely, at a
sub-bin offset that looked entirely reasonable. Past a whole bin it was worse
again: two bins land on the limit, which is a duplicate column and a zero gap,
and the band prune is guarded with `bin_gap > 0`.

What it does now is squeeze rather than shift: the start moves, the top stays,
and the width tightens by `offset / (P - 1)` of itself — 1.1 % at the default 90.
Tighter is the direction the edge rule absorbs, and the failure this issue is
about is a gap that is too WIDE. An offset of 0 gives the same grid it always
did, to the bit.

## The wrap, which the closed form made necessary

Closed-form inverse kinematics returns every joint canonically, in `(-pi, pi]`.
Four of this arm's axes have ranges wider than a turn — `J1`, `J4` and `J5` span
722°, `J6` spans 1146° — so a posture at −179° is equally a posture at +181°, and
subtracting two canonical numbers reads a **two degree** step as **358**. A seam
that drifts a joint through the boundary loses its edge there and stops, for a
motion the arm makes without noticing.

The rule now measures the smallest **legal** motion: the destination is offered
the number of turns that brings it nearest the source, and the offer counts only
if it is still inside its limits afterwards. Axes narrower than a turn — `J2` and
`J3` — get zero turns and the plain difference, which is what they always had.

The lattice keeps the canonical values and `Graph.unwound` resolves the winding
**once, on the chosen path**. Storing wound values instead would make a node's
value depend on the path that reached it, which is the ordering dependence the
dynamic program exists to be free of.

**No seam in the corpus uses it.** The closest any best path comes to a boundary
is 17.4° on `J4`, so the tests are constructed rather than found: a two-layer
lattice at ±179° on a wide axis crosses at a 10° threshold and costs 2°, the same
lattice on a 359° axis does not cross at all, and at a threshold that admits the
long way round it costs 358 rather than quietly costing 2.

Reported cost and the wound trajectory's total motion agree to 1.7e-6 rad across
the corpus, which is float32.

## What is still owed

- ~~Two 10 mm seams on `bracket_a_placed`, `seam_0005` and `seam_0007`, still
  stop at layer 1 with 60.0° of electrode turn between their first two
  samples.~~ Both cross now, and neither fix was aimed at them: `seam_0007`
  recovered when 0001 absorbed its corner, and `seam_0005` recovered when the
  tool-frame fix (the origin on the weld instead of a phantom gun-length back)
  moved every pose and its 61°-in-0.3 mm corner became payable — largely as a
  wrist wind the wrap prices honestly. The ladder tests that used `seam_0005`
  as their specimen construct their condition now; no corpus seam needs a
  second rung.
- The `bins_for` relation is available and used at the corpus test, but
  `nodes.build` still takes bin counts as plain arguments and cannot check them,
  since the threshold belongs to the search. A caller can still write 8 and 6.
  `pinned_axes` catches it after the fact rather than before.
