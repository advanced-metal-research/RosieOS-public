# 0003 · The positioner has two axes and the lattice freezes one without saying so

**Status:** fixed. The open question below was answered by the machine's owner
on the day it was filed — **J9 is a station index, not a weld axis; the usable
set is the arm's six plus J7** — which settles the dimension count at two
continuous freedoms and closes option 3. Option 1 is done: `nodes.build` takes
`held`, every unswept work axis is named and valued on `Lattice.held` and in
`summary()["held_deg"]`, and three tests in `tests/motion_planner/test_nodes.py` pin it.
**Found:** 2026-08-12, while answering "was the positioner held fixed in that
root-count experiment?" — it was, and the question exposed that "the
positioner" names two joints, not one.
**Blast radius:** every lattice ever built. A whole continuous freedom is
discarded at a value nothing chose, and no output records that it was.

## What is wrong

`rosie_1400_v3` reaches the work through **two** revolute joints:

    J9   floor   -> h_frame                 axis Z (vertical)   ±180°
    J7   h_frame -> positioner_table_a      axis Y (horizontal) ±180°

J9 swings the whole H-frame — both tables — about a vertical axis 2.413 m from
the world origin. J7 rolls table A about a horizontal one. `fk.build(cell,
[work])` reports `num_dof == 2` and `axis_names == ['J9', 'J7']`, correctly.

`nodes.build` takes one of them:

```python
qpos = torch.zeros(P, work_chain.num_dof, dtype=dtype, device=device)
qpos[:, widx[positioner_axis]] = torch.as_tensor(positioner, ...)
```

`positioner_axis` defaults to `"J7"`. The `zeros` pins **J9 at 0** — a value
that is the initialiser of a scratch tensor and not a decision anybody made.
Nothing in `Lattice`, `Lattice.summary()`, the `motion-graph.v0` bundle or
`MOUNT_NOTE` says J9 was held, so a reader of any output sees a screen over
"the positioner" and gets a screen over half of it.

## Why the count matters

The dimension arithmetic for a weld on table A:

    variables   J1..J6, J7                                  7
    pinned      tip position 3, electrode direction 2       5
    free                                                    2

Two continuous freedoms — spin and J7. That is what `nodes.py`'s docstring said
before this issue, and it is right, but it was right by accident: the seven came
from counting the axes the code touches, and the code touched seven because a
`torch.zeros` had silently removed the eighth. Written as eight-minus-five the
answer would have been three, and the gap between the two numbers is exactly the
policy nobody had written down.

The policy is now written down, in the cell's own terms: **J9 is a station
index.** It presents one table to the robot and the other to the operator; a
weld does not run through it. So seven is the usable count and two is the
freedom count, and both are statements rather than side effects.

## How it was found

Not by a test — a test would have had to know the number to check. It came out
of a root-counting experiment run for a different reason: `fk.build(cell,
[tool])` prints `6 axes` and `fk.build(cell, [work])` prints `2`, and 6 + 2 is
not 7.

The general shape is the same one as 0001: `nodes.build` names its parameter
`positioner_axis`, singular, and every reader downstream — including the ones
who wrote the docs — took the name for the count.

## Measured

Feasible fraction of a 24 × 12 × 9 lattice, `bracket_a_placed`, J9 swept and
everything else held:

| J9 | seam_0001 | seam_0002 | seam_0003 |
|---:|---:|---:|---:|
| −90° | 0.00% | 2.69% | 13.71% |
| −60° | 0.00% | 8.97% | **18.65%** |
| −30° | 7.75% | 10.65% | 18.14% |
| **0° (what is used)** | 15.32% | **11.40%** | 15.60% |
| +30° | 17.29% | 10.34% | 6.27% |
| +60° | **18.13%** | 6.76% | 0.00% |
| +90° | 11.73% | 2.43% | 0.00% |
| +180° | 0.00% | 1.04% | 0.00% |

Two things in that table:

1. **J9 = 0 is not the best value for any of the three seams.** The gains are
   modest — 15.3 → 18.1 and 15.6 → 18.7 percentage points of lattice — but they
   are free, and a thin seam is thin at its worst layer, not on average.
2. **The best value points opposite ways on the same part.** seam_0001 wants
   +60° and seam_0003 wants −60°. No single frozen J9 serves the part, which
   is the argument that this belongs in the search rather than in a config
   file.

At the extremes the seam dies outright — seam_0003 has zero feasible nodes at
every J9 ≥ +60°. So J9 is not a nuisance axis with a broad flat optimum; it
moves the part right out of the arm's plan request.

The table above is therefore not an argument for planning through J9. It is the
measurement that says **which station you index to changes what the arm can
reach**, and that is a fixturing fact worth keeping: seam_0001 and seam_0003 of
the same part prefer opposite swings, so no station serves the whole part
equally well. If a seam is ever thin enough to matter, this is a lever that
exists — at setup, not during a weld.

## What was done

1. **Say it.** ✅ `nodes.build` gained `held: dict[str, float] | None`. Every
   axis on the chain to the work that is not `positioner_axis` gets a value
   there, defaulting to 0.0, and the resulting `{name: radians}` rides out on
   `Lattice.held` and in `summary()["held_deg"]` — so the debug bundle and any
   report built from a summary now say which axes were swept and which were
   pinned where. `summary()` also carries `positioner_axis` for the same reason.
2. **Let the caller choose.** ✅ Same argument. A name not on the work chain is
   refused rather than ignored, since a typo in a held axis is this bug again
   in different clothing.
3. **Make it a lattice dimension.** ❌ Closed. J9 is a station index; it is not
   the planner's to move.

Pinned by `tests/motion_planner/test_nodes.py`:
`test_every_work_axis_the_lattice_did_not_sweep_is_named_and_valued` asserts on
the report rather than the value — which axes exist is the cell's business and
what to pin them at is a decision, but that a held axis is named with its number
is neither; `test_a_held_axis_actually_moves_the_part` swings J9 by 90° and
requires the weld point to travel more than 0.5 m, so a `held` that is reported
and then dropped on the floor fails by a mile rather than by a tolerance; and
`test_holding_an_axis_that_is_not_on_the_work_chain_is_refused` covers the typo.
