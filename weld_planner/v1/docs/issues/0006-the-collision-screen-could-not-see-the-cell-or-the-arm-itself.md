# 0006 · The collision screen saw the part, and nothing else in the cell

**Status:** fixed in the screen; the cost of the fix is measured and partly
open. The screen now sees the whole cell and the arm against itself, and **0%
of what it passes is a collision** where 60% used to be. It is now conservative
in the other direction — 19% of what it rejects, the verifier would allow — and that
is the sphere model, not the geometry.
**Found:** 2026-08-12, by looking at a screenshot. The viewer drew the arm
buried in the positioner on a lattice the planner called feasible.
**Blast radius:** every collision-screened lattice ever built, and every
unscreened one by definition. The corpus's "15 of 15 crossing" was measured with
screening off.

## What is wrong

Two holes, and the picture showed the larger one.

**The scene held the workpiece alone.** `make_graph_demo._scene_for` said so and
justified it:

> The PART only, not the whole cell. The h-frame, floor and tables would need
> their .dae meshes placed through the held joints, which is `verify.from_cell`'s
> job and a heavier lift than a screen wants — and the part is what the torch is
> closest to by a wide margin while welding.

Every clause is true except the last, and the last is the one carrying the
argument. The TORCH is closest to the part. The forearm is somewhere else
entirely, and on this cell that somewhere is inside the positioner.

**And the arm could not see itself.** `Scene.self_clearance` has existed as long
as `Scene` has, with its own warp kernel and its own backward pass.
`_screen_collision` never called it.

## Measured

Screened-feasible nodes on `bracket_a_placed` seam 1, re-judged with the verifier's
exact mesh geometry — 128 nodes sampled across 16 layers:

| what the screen looked at | feasible | the verifier rejects |
|---|---:|---:|
| nothing | 10.76% | **60.2%** |
| the part alone (what shipped) | 5.72% | 30.5% |
| part + the cell's static bodies | 4.88% | 13.3% |
| **part + cell, split by frame, + self** | **3.82%** | **0.0%** |

What the 60% were hitting, by pair:

    link_4 <-> link_6                34    the arm folded into itself
    link_4 <-> positioner_table_a    17    the table being welded on
    h_frame <-> link_5               11
    floor <-> link_5                  6
    h_frame <-> link_4                5
    link_5 <-> positioner_table_a     2
    floor <-> link_4                  1
    end_effector <-> link_3           1

**Two thirds of what the screen called feasible was never feasible.**

## Why the frame split is not cosmetic

`scene.build` has taken `env` (world frame) and `fixtures` (work frame) since it
was written, and the difference decides correctness rather than tidiness: a body
that rides the work link turns with the positioner, so its field has to be baked
in the frame that turns with it. Bake the weld table in world at one positioner
angle and every other angle is screened against a table that is not there —
which is this same bug one level down.

Measured: with the table in the world frame the verifier still rejected 13.3%, and
`link_4 <-> positioner_table_a` was 15 of the 17 remaining. Moving it to the work
frame took that to zero. The mesh is twelve triangles.

A body driven by a HELD axis counts as world, because `held` is what the lattice
pins and an axis that never moves makes everything beyond it static. On this cell
that is `J9`, so the h-frame and the far table are world.

## What was done

- **`scene.cell_obstacles(cell, mesh_root, held=...)`** returns
  `(env, fixtures, fixtures_link)` by walking the cell's own joints — which
  bodies move with a free axis, which ride the work link, which are arm. A list
  of names would have gone stale on the next cell.
- **`_screen_collision` takes the minimum with `self_clearance`** where the
  scene has self pairs.
- **`make_graph_demo` passes the cell**, so what the viewer draws is what was
  screened.

## What it costs, and this half is open

The screen is now conservative, which is the right direction for a screen — it
may lose a valid posture, it may not propose an invalid one. The cost is
measured: of 200 sampled nodes the screen rejected for collision, **the verifier
would have allowed 38 (19.0%)**. The sphere model reports penetration those
poses do not have, by a median of 11.2 mm and up to 34.7 mm.

So the corpus number moves, and the honest statement of it is a bound rather
than a fact:

| | crossing |
|---|---:|
| screening off (what BENCHMARKS §10 measured) | 15 of 15 |
| **screening on, whole cell** | **10 of 15** |

The five that stop are `bracket_a` 1 (0.00% feasible), `bracket_b` 1 (0.18%),
`dog_box` 1 (1.73%), `dog_box` 2 (0.00%) and `dog_box` 3 (2.02%). Given the 19%
false-reject rate, some part of that is the sphere model rather than the cell.
**Which part is unmeasured.**

## The tool-frame fix promoted this from a tax to a blocker

Everything above was measured with the planner holding the arm a **phantom
150 mm back from the weld** — `body_length_mm` applied as a mount offset on
top of a `link_6` mesh that already contains the gun and ends at `tool0`
(2026-08-13, the cell's tool asset has the measurement). With the tool origin
correctly ON the weld point:

| offset | weld points with a posture (part-only screen) |
|---|---:|
| 150 mm (the phantom) | 43.1% |
| **0 mm (correct)** | **0.0%** |

Kinematics is fine — collision off, 15 of 15 cross at offset zero. The screen
is the whole story, and it has two defects that the offset was masking:
`link_6`'s spheres run **up to 47 mm of radius on the 15 mm torch tube**, and
the model **stops 64 mm short of the tip** — its farthest sphere sits at
385 mm of a 449 mm link. Fat where the tube is thin, absent where the wire
meets the work. `test_the_sphere_model_cannot_yet_see_a_torch_on_the_weld`
pins the 0.0% so the state cannot be forgotten, and is written to be REVERSED
when the model is refit, not restored.

Every screened number above therefore describes the old geometry. The 19% and
the 10-of-15 are historical context, not the current bill.

## The refit, and what the working screen then said about the data

`tools/fit_spheres.py` (2026-08-13). Two decisions, both traceable to this
issue's measurements:

- **Inscribed spheres for the arm** — radius equals the centre's interior
  depth, so the model cannot report the arm somewhere the arm is not. The
  trade is under-cover between spheres, which is the trade the module's own
  doctrine prices: under-cover costs speed and the CCD verifier re-checks;
  over-cover at the weld turned out to be blindness. `link_2` and the base
  keep the authored fit (open shell, no interior, never near a weld).
- **The analytic torch in place of fitted blobs at the tip.** A surface fit
  sees a 15 mm tube and covers it with 15 mm balls, but the thing near the
  work is a nozzle of half that radius — and the WIRE is left out entirely,
  because it is meant to touch the weld and a screen that models the
  consumable rejects the exact contact welding exists to make. The nozzle at
  a 45° work angle clears a flat plate by ~8 mm: the standoff, doing its job.

Tip on weld, whole-cell screen, `bracket_a_placed` seam 1: **0.0% → 43.1%**
of weld points with a posture, and all three placed seams cross screened
(`test_the_placed_request_crosses_with_the_whole_cell_screen_on`).

**The working screen then indicted the corpus, not the cell.** Every
UNPLACED plan request sits its part inside the positioner's fixture bar — 5% of
`dog_box`'s vertices to 33% of `bracket_b`'s are inside the beam — so their
screened failures at layer 0 are the screen telling the truth about
impossible data. Only `bracket_a_placed` authors a placement, and it is the
one that passes. The demo generators now screen by default (`--no-collide`
to skip), so the dev pages show this state rather than paths the cell
cannot run.

## What is still owed

- **Placements for the rest of the corpus.** The screen is no longer the
  blocker; the data is. Four of five plan requests need the part placed ON the
  fixture rather than inside it before any screened number about them means
  anything.
- **The self-clearance rate deserves a look**: ~23% of kinematically feasible
  postures fail arm-against-itself with the refit model. Plausible for
  elbow-under postures, but it has not been split into real contact vs pair
  margins the way the part screen just was.
- **The corpus assertion still runs with screening off.** It should run both
  ways and say which number is which.
- **`BENCHMARKS.md` §10's "15 of 15" is now labelled**, but the timings there
  are all unscreened. The screened build costs about 10% more (§10 has that),
  so they are close, but they are not the same run.
- Nothing here checks the arm against the TORCH's own cable or the wire feeder,
  which no asset in this repo describes.
