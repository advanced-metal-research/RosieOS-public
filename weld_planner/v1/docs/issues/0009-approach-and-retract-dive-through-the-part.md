# 0009 · Approach and retract dive through the part, and the reason was the winding

**Status:** mostly fixed. `bracket_b_2x` joins end to end with all four links
collision-certified; `bracket_a_2x` gets three of four and `hitch_2x` two of
four. Pinned by `tests/motion_planner/test_freespace.py` and
`tests/motion_planner/test_roadmap.py`, including a counterfactual
(`test_without_rewinding_the_approach_still_dives_through_the_part`) that
**fails the day the fix stops being needed**.
**Found:** 2026-08-13, building M6's first cut. **Diagnosed and mostly
fixed:** 2026-08-13.
**Blast radius:** every approach and every retract, which is to say every
program. The seam trajectories themselves are unaffected — but see *what is
still owed*, because the winding is M4's decision and M6 is currently
correcting it after the fact.

## What was wrong

`freespace.connect` joins a program: reset pose → seam → transit → seam →
… → reset pose. The transits solved and the T2 verifier certified them. The
approach and the retract did not:

    approach  reset     -> seam_0001   FAIL      clearance  -4.4 mm, 5 segments contact
    transit   seam_0001 -> seam_0002   clear     clearance   6.1 mm
    transit   seam_0002 -> seam_0003   clear     clearance   7.1 mm
    retract   seam_0003 -> reset       FAIL      clearance   7.7 mm, 4 segments contact

## How it was found, in the order it actually went

Four wrong or partial answers came first. Each is recorded because each
looked like the answer and the sequence is the lesson.

1. **The margin.** M5 runs its augmented Lagrangian at margin ZERO — correct
   there, since a weld holds the gun millimetres from the work by definition.
   Copying that into free space was this module's first bug: at margin zero
   the term is silent until something is already inside. `cost.py`'s own
   docstring says the default 50 mm margin "exists to give descent a gradient
   before contact when diving into free space". Fixing it cleared both
   transits and left both ends penetrating.
2. **The seeds.** Eight were added — straight, wrist-first and body-first
   staged, five sinusoidal bows — and **all eight penetrated**, between 4.4
   and 37.9 mm. The order of the motion was not what was wrong.
3. **The ranking, and this one was real.** The winner was chosen by
   `(penetrates, cost)`, so when EVERY seed penetrated the tie-break fell to
   smoothness and it picked a 30 mm penetration over a 4 mm one. It ranks by
   clearance now when nothing is clear.
4. **The standoff waypoint, and this one worked — for half the problem.** A
   torch approaches its weld ALONG the electrode, from a standoff above it,
   not sideways through the plate. `standoff_configs` takes the weld pose,
   pushes the tool origin back along `+u`, and asks IK for it **seeded at the
   weld configuration itself** — so the answer is a few degrees away in the
   same branch and the plunge really is a retreat rather than a re-posture.
   Measured on `seam_0001`: 15.5° of largest joint travel for 100 mm of
   retreat, 1 µm of position error, and clearance rising 3.9 → 30.3 → 67.5 →
   141.7 mm at 50/100/150/250 mm of retreat. As a seed it took the approach's
   environment clearance from −29 mm to **+3.9 mm and held it there**.

And then the approach still failed, which is where the real diagnosis
started.

**The verifier was complaining about the arm, not the part.** Its collisions were
`end_effector` against `link_3` and `link_4` against `link_6` — self-pairs.
`CollisionCost` has always priced self-collision (`cost.py` sums
`scene.clearance` and `scene.self_clearance`), but the ranking that picked
the winning seed read only the environment field, so a path that flew nowhere
near the part and folded the forearm into the wrist scored as perfectly clear
and won on smoothness. That is fixed; `Link` now carries `env_clearance_m`
and `self_clearance_m` separately, because the two fail for different reasons
and one combined number hides which one happened.

**With self-collision visible, no seed and no weight escaped it.** Every seed
sat between −70 and −82 mm and 240 optimizer steps moved it by 0.3 mm. The
reason, measured: at the seed, `|∇smoothness|` is 780–4436 and
`|∇self-collision|` is 19–29. A sweep of 32 combinations — rho 50 and 1000,
growth 1.6 and 4.0, 3 and 8 outer steps, 80 and 200 inner — never got past
−49 mm. The constraint was not failing to converge; it was converging inside
a wall.

**So a global method was built, and the first thing it said was that there is
no way through.** `roadmap.py` samples the seven driven axes, screens against
the ESDF *and* the self-pairs, and connects what stands clear. On the
unrewound program it came out in two components of almost exactly equal size,
and that split did not close with density — 4096 → 32 768 samples, 12 → 24
neighbours, still ~50/50. A split invariant to density is a variable, and it
was `J5`: one component held `J5 ∈ [151°, 237°]`, the other
`J5 ∈ [−34°, 44°]`, and the band between them held nothing.

Checked directly, without the graph: of **16 384 configurations with `J5`
between 50° and 145°, not one stands clear of the arm itself.** The best
self-clearance in the band is −3.9 mm. The environment is fine in there (81%
clear) — it is the wrist folding the torch back onto `link_3`.

## Why it happened

`bracket_b_2x seam_0001` starts at `J5 = +199.5°` and the reset pose is
`J5 = 0`. Those are on opposite sides of that band, so **the approach M4 asked
for does not exist** — not "is hard to find", does not exist.

But `J5 = −160.5°` is *the same pose*. A revolute axis at `t` and at `t − 2π`
puts every sphere in the same place; measured across three seams, the
environment and self clearances differ by 0.18 µm and 0.089 µm, which is
float32 forward kinematics. All that differs is the number the controller is
handed. M4's `Graph.unwound` picks a winding per seam with no view of the
program the seam will sit in, and it happened to pick the unreachable one.

## What was done

- `standoff_configs` and the standoff seeds (`freespace._through`).
- Self-collision in the seed ranking, and `env`/`self` clearance reported
  separately on every `Link` and in the CLI.
- `roadmap.py`: a deterministic Sobol PRM over the driven axes, screened
  against both fields, `heapq` for the shortest path. Its node self-margin is
  5 mm and **smaller than its 20 mm environment margin**, which looks like a
  typo and is not: this arm's sphere model never reports more than 17.8 mm
  between the pairs it checks, so 20 mm is not a cautious self margin here,
  it is an impossible one — the first roadmap built with it screened out all
  4096 samples and refused.
- `freespace.rewound`: each seam commanded on the turn that lands it nearest
  where the arm already is, per axis, subject to that axis's limits over the
  whole seam. Where a roadmap is supplied it goes further, because **nearest
  is not the same as reachable**: `bracket_a_2x seam_0001` at `J5 = +165°` is
  nearer to home than `−195°` and has no route, while the other turn routes
  in five waypoints and so does the transit out of it. Candidates are tried
  nearest first and the first one the roadmap can reach wins.
- `freespace.winding_reach`, because the roadmap's box is cut to what is
  being joined and the winding is what it is being asked about — the union of
  candidate windings has to go in up front or the alternative is outside the
  box by construction.
- The roadmap route as a seed (`freespace._along`), laid out along the
  horizon in the time metric with Catmull-Rom velocities at interior
  waypoints.

Result, three seams per plan request, verifier margin 0:

    bracket_b_2x   4 of 4 links certified   env  3.9 … 7.7 mm   self  7.6 … 11.9 mm
    bracket_a_2x   3 of 4                   transit seam_0001->seam_0003: 2 segments
    hitch_2x       2 of 4                   approach, and transit seam_0001->seam_0002

Note what wins once the program is rewound: `straight`, on three of
`bracket_b_2x`'s four links. The roadmap seed is what the two hard links on
the other plan requests fall back to. That is cuRobo's ladder — cheap seed first,
global method only when it fails — arrived at from the other end.

## What is still owed

**`hitch_2x`'s approach.** Environment clearance −21.7 mm, self clearance
+11.9 mm, and the roadmap has a route (5 waypoints at 256 attachments) that
the optimizer does not follow to a clear finish. This one is a local-refinement
failure rather than a homotopy failure, which is a different problem from the
one this issue started as.

**`hitch_2x seam_0002` is nearly self-colliding at the weld itself** — 0.38 mm
of self-clearance at the seam pose. Any transit into it starts and ends at a
pose with no room, so there may be nothing for M6 to fix; this may belong
upstream as a node the lattice should not have offered.

**The winding belongs to M4, not M6.** `rewound` corrects `Graph.unwound`
after the fact, which works and is in the wrong place: M4 chooses a winding
per seam with no view of the program, and the information it needs — where
the arm will be standing when the seam starts — is a program-order fact it
could have. Moving it upstream would also let the DP score a seam's winding
rather than have M6 rewrite it.

**The roadmap is not cached.** It is rebuilt per plan request per run, ~2 s at
8192 samples. It depends only on the cell and the placed part, so it is
exactly the kind of thing M9's content-addressed cache is for.

**Nothing here is proof.** The roadmap screens at samples — nodes, and eight
interior points per edge. That is fine and only fine because what it produces
is a seed, and the T2 verifier certifies the continuous curve afterwards. If a
route from this module ever gets used as evidence rather than as a starting
point, that is a bug.
