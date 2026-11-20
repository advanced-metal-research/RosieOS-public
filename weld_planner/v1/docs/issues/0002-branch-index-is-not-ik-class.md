# 0002 · A lattice cell is not a continuous track, so a seam with room at every sample can still have no path

**Status:** open as a defect, **closed as an explanation.** The behaviour below
is real and still unfixed: a branch index does name different sheets at
different layers. It is NOT why `dog_box` seam 1 failed. That was
`docs/issues/0004` and `0005`, and either one alone crosses it. 0004: the
positioner was binned 60° apart and searched at a 25° threshold, so no edge could
change positioner bin and the search welded the whole seam with the workpiece
frozen. 0005: the seam was being welded at 0° travel angle when it authors 10°.
Fix either and the seam crosses, with the same eight structured seeds and no warm
start. The corpus is 15 of 15.

**Do not fix this by warm-starting.** The proposal below is superseded on two
counts. It would make layer `N`'s node set depend on layer `N-1`'s answer, which
is the ordering dependence the dynamic program exists to be free of; and it was
measured against the wrong target. Random restarts — the dependency-free version
of the same idea — take `dog_box` seam 1 from 11.1% feasible nodes to 64.9%, six
times as many, and the frontier does not move a single layer. Seed coverage was
never the binding constraint. See 0004 for the numbers.

What remains genuinely owed is below, under **What is still worth fixing**.

**Found:** 2026-08-12, as the one seam left failing after 0001.
**Blast radius:** originally claimed as `dog_box` seam 1. That attribution was
wrong; see above. The mislabelling itself has no measured blast radius yet.

## What is wrong

`dog_box`'s first seam has **47 feasible nodes at layer 28** and stops there
anyway. Nothing about the geometry is remarkable at that point: the electrode
turns 6.2° between samples against a 6.2° median for the whole seam, the tip
travels 3.80 mm, no node is closer than 0.32% of its range to a joint limit,
and the layers on either side hold 47 and 49 nodes.

The frontier's cause histogram says it exactly:

    no ik 165   joint limit 160   duplicate 12   no edge 47

Forty-seven feasible, forty-seven unreachable. Every posture that exists at
layer 28 is further than the edge threshold from every posture reachable at
layer 27.

## The shape this is a defect in

Fix the spin and the positioner and the torch pose is fully determined, so the
arm is six equations in six unknowns — not a further freedom but a finite set
of roots, up to eight. The feasible set at one weld point is **up to eight
disconnected two-dimensional sheets, each coordinatized by
`(spin, positioner)`**. Two continuous freedoms, not three, and the arm's
posture is a LABEL saying which sheet rather than a coordinate on one.

Sheets are what no continuous motion crosses. So a trajectory has to stay on
one, and in the lattice that means: **holding the branch index fixed along the
seam has to mean staying on one sheet.**

## Why it happens

It does not. `nodes.build` solves every layer independently, from one fixed
nominal seed with eight structured branch seeds on top, and **`branch` is the
identity of the SEED, not of the sheet it converged onto.** Nothing makes seed
3 at sample 27 land on the same sheet as seed 3 at sample 28.

So walking the lattice along constant `branch` is not walking a sheet — it is
hopping between sheets, which is the one motion that does not exist.

Measured on `bracket_a`: the same cell holds postures **66° to 229° apart** at
adjacent samples. The cells are not tracks; they are eight independent lotteries
per sample.

Most of the time this costs nothing, because with 8 spins × 6 positioner
angles × 8 branches something lands near something. It costs everything where
the feasible set is thin enough that the lottery misses.

## What has been tried

**Warm-starting layer `l+1` from layer `l`'s answers.** Written and measured on
`bracket_a` seam 1: median step 113°, nothing within 25°, no improvement. That
looked like a refutation and was not — that seam had a 90° corner 27 µm from
its start (issue 0001), so the TARGET had moved and no seeding could help. The
experiment was run against the one seam where it could not work.

It has not been re-run since 0001 was fixed, and `dog_box` seam 1 has no
corner. The refutation does not apply to it.

## What was going to be done, and is not

Seed each layer from the previous layer's answer for the same cell, keeping the
eight structured branches for layer 0 only, so that `branch` becomes a
continuous track by construction rather than by luck. **Abandoned.** Two
reasons, and the first would have been enough:

1. It makes the node set at layer `N` a function of the answer at layer `N-1`.
   The dynamic program below it exists precisely so that no early choice can
   foreclose a later one, and a lattice built by walking forward hands that
   property back at the door — a solve that fails to converge from the carried
   seed deletes a node for a reason that is historical rather than geometric,
   and the search can no longer distinguish the two.
2. It was aimed at a target that was not there. See the status note at the top:
   restarts multiply the node count sixfold and move the frontier zero layers.

## What is still worth fixing

The mislabelling is real and unfixed: `branch` names a seed, and a seed lands
where it lands, so holding `branch` fixed along a seam is not the same as
staying on one sheet. What is now known is that this costs **nothing measurable
in whether a seam crosses** — the search compares joint vectors all-to-all
across adjacent layers and never consults the branch index, so a mislabelled
node is still an ordinary node with ordinary edges.

**A second thing lives in the same eight slots, found while measuring the
first.** `nodes.build` marks a branch a duplicate on a RAW difference —
`(q[i] - q[j]).abs().max() < BRANCH_TOL_RAD` — so one posture held at `J6` and
at `J6 + 360°` occupies two slots. On `dog_box` seam 1 at 15×15 bins, **37.2% of
feasible branch pairs within a cell are the same posture wound differently**
(1132 of 3041 pairs, raw gap median exactly 360.0°), and 965 of 4134 feasible
cells hold at least one such pair.

Whether that is a defect depends on what the slot is for. As SEARCH STATES the
two are genuinely different — the arm has to unwind to get from one to the
other, and the edge rule is right to charge 360° for it. As BRANCH SLOTS they
are one root taking two of eight seats, and the spherical wrist caps the roots
at eight, so every rewound copy displaces a root that has nowhere else to go.
Both readings are in the code at once and neither is written down.

Where the labelling will cost something is anywhere the label is read AS a
sheet:

- **Refinement (M5)** seeding from K-best paths, if it assumes a path that holds
  one branch index stayed on one sheet.
- **Any report** that groups nodes by branch and describes the groups as
  postures. `ui/graph.html` draws the branch axis; a reader taking column 3 for
  "the elbow-down family" is reading something that is not there.
- **Diversity metrics** over K-best, if they treat two paths through different
  branch indices as necessarily different postures.

None of those exist yet in a form that depends on it. The honest fix, when one
is needed, is to LABEL rather than to reseed: cluster each layer's postures
after the fact and assign sheet identities by proximity to the previous layer's
clusters. That is a post-process over an unchanged batched solve, so it buys the
correct label without buying the ordering dependence.

`BENCHMARKS.md` §9 has the numbers if any of this is ever measured.
