# Issues

One file per issue, named `NNNN-short-slug.md`. Newest number wins; numbers are
never reused.

This is for defects and open questions that outlive the conversation they were
found in — the ones where the fix is known but not yet made, or made in one
place and still owed in another. Things that get fixed in the same sitting
belong in the commit message, not here.

Each file opens with a status line and answers four questions in order:

- **What is wrong** — the behaviour, stated so it could be reproduced by
  someone who was not there.
- **How it was found** — because the route to a bug is often the only thing
  that will find its siblings.
- **Why it happens** — the mechanism, down to the file and the line.
- **What was done** — the fix, or the stopgap, and what is still owed.

Numbers over adjectives throughout. "A 27 µm stub" is a fact anyone can check;
"a tiny rounding error" is a feeling.

## Index

| # | Status | Title |
|---|---|---|
| [0001](0001-seam-piece-absorbed-across-a-corner.md) | fixed | A sub-millimetre stub was absorbed across a 90° corner; 8 of 15 seams could not be planned, now 14 |
| [0002](0002-branch-index-is-not-ik-class.md) | open, downgraded | A lattice cell tracks a SEED, not a posture. Real, but NOT why any seam failed — that was 0004. The warm-start fix is abandoned |
| [0003](0003-the-positioner-has-two-axes-and-one-is-silently-frozen.md) | fixed | "The positioner" is two joints; the lattice pinned one at a `torch.zeros` and said nothing. J9 is a station index — held explicitly and reported now |
| [0004](0004-bin-spacing-wider-than-the-edge-threshold-pins-the-axis.md) | fixed | The positioner binned 40° apart and searched at 25° — no edge can join two bins, so the axis was unusable. Two claims in its first version were corrected by 0005 |
| [0006](0006-the-collision-screen-could-not-see-the-cell-or-the-arm-itself.md) | part fixed | The screen held the workpiece alone, so 60% of "feasible" nodes were the arm inside the positioner or itself. Now 0%, at 19% false rejects |
| [0005](0005-the-lattice-welded-at-angles-nobody-authored.md) | part fixed | Every weld was planned at 0°/0° while the seams author a ramp to 34.5°. Nominal now read; the authorized ±5° bands still unused |
| [0007](0007-the-verifier-pad-is-wider-than-a-weld-pass-has-room-for.md) | open, deferred | The verifier's 2 mm pad exceeds the 1.3–2.4 mm a weld pass has, so 3 correct seams report collisions they do not have. Verdict now separates `penetrating`; stopgap is to grow the torch mesh |
| [0008](0008-a-curved-weld-line-cannot-be-tracking-certified.md) | fixed | `lipschitz_f` returned None on a curved weld line, so no curve was ever certified. A curvature bound off Bezier hulls (rational included) and a turning-datum bound close it; `hitch_2x seam_0008` is now decided -- and FAILS at 1.71 mm, which M5's own midpoints (1.60 mm) confirm |
| [0009](0009-approach-and-retract-dive-through-the-part.md) | fixed | The cause was the WINDING: this arm has no clear pose with `J5` in 50–145°, so the approach M4 asked for did not exist. Standoff seeds, self-collision in the ranking, a PRM (`roadmap.py`) and `rewound` gave 4/4 links on `bracket_b_2x`, 3/4 and 2/4 on the others; 0011's work closed the rest — 4/4 on all three at three seams, and 21 of 23 links on `bracket_b_2x`'s full program, up from 20 |
| [0011](0011-the-trust-region-is-a-ratchet-and-the-loop-freezes.md) | fixed in `freespace`, open in `generate` | The per-row trust scale halved on a rejected step and never recovered, so the AL's multiplier growth froze the loop for two thirds of its budget at a 100% accept rate. Every attempt to fix the resulting path shape by reweighting had therefore failed — an optimizer taking no steps cannot hear its objective. M5 has the same ratchet and a seam projection nobody has measured it against |
| [0012](0012-a-winding-reachability-test-flips-only-in-a-full-suite-run.md) | open | A winding-reachability assertion failed once in four full-suite runs and passes alone and in its own file. The claim it pins is discrete — the nearer turn of `J5` has no route and the identical pose one turn away does — so a flip means a reachability answer moved with nothing about the problem moving, and `roadmap.py` promises determinism |
| [0013](0013-the-weld-pass-itself-should-start-and-end-at-rest.md) | open | The weld trajectory's own ends carry travel speed, so the creep handover (2026-08-21) had to break C1 with a stop at every arc start and stop. The right fix is upstream: authored run-in/run-out on the seam's process block, M5 ends at rest, and the rest-to-rest handover is C1 again. Process decision first, planner change second |
| [0015](0015-free-space-is-catmull-rom-where-the-architecture-says-b-spline.md) | open | `ARCHITECTURE.md` says the trajectory is a joint-space B-spline; `freespace.py` optimizes knot positions alone and differences them for tangents, which is Catmull-Rom and `C1`. Acceleration jumps 1.73 rad/s^2 at a knot (0.83 of the cell limit) and `check_limits` samples the knots, so it never sees it. Measured against cuRobo on the same cell: acceleration ratio 3.35 against 0.99 |
| [0017](0017-the-olp-previews-fk-rows-while-the-certified-curve-is-discarded.md) | open, required by D24 | The OLP previews a rails plan by lerping FK'd program.v1 rows, three derivations away from the Hermite `(q, qdot, t)` the verifier certified — which `/api/export/olp/rows` consumes and drops. D24 deletes the row compiler, so this becomes the only preview source rails has |
