# 0011 — The trust region is a ratchet, so the loop freezes and the objective goes unheard

**Status:** fixed in `weld_motion_planner.planner.trajectory_optimization.engine.freespace` (2026-08-16), OPEN in
`weld_motion_planner.planner.trajectory_optimization.engine.weld_trajopt`.

## What happens

Both descent loops in this planner size their step the same way. Each row of
the batch carries a `scale`, applied to whatever step Adam proposes; a step
that makes the objective worse is undone and the row's scale is halved.

    scale.mul_(torch.where(worse, 0.5, 1.0)).clamp_(min=5e-4)

There is no other line that touches `scale`. It starts at 1.0 and the only
value it can ever be multiplied by is 0.5, so it is a **ratchet**: every
rejection is permanent, and a row that has a bad patch early is crippled for
the rest of the run however well it behaves afterwards.

Traced on `bracket_b_2x`'s approach, per 40-step block, `freespace` at 80x3:

    step  accept%  scale min  scale med  scale max  moved deg   best
      40     93.8    0.06250    0.25000    0.25000      23.01  61.89
      80    100.0    0.06250    0.25000    0.25000      35.69  54.20
    -- multipliers grown, Adam restarted, trust scale KEPT
     120     91.6    0.00781    0.01562    0.06250       8.70  60.97
     160    100.0    0.00781    0.01562    0.06250      10.08  59.76
    -- multipliers grown, Adam restarted, trust scale KEPT
     200     99.1    0.00781    0.01562    0.03125       9.77  60.93
     240    100.0    0.00781    0.01562    0.03125      10.00  58.16

The collapse happens at the augmented-Lagrangian multiplier growth, and it is
nobody's mistake that it does: the objective genuinely changes there, so the
first few steps against the new one are correctly rejected. What is wrong is
that the four halvings they cost are never given back. From step 80 onward the
accept rate is 91-100% and the step size is a sixteenth of what it was.

At 320x3 the same trace shows the mechanism from the other side: the scale
holds at 0.25 for the whole first outer iteration, the cost falls 61.89 to
28.82, and the arm moves ~50 degrees per block. Then the first multiplier
growth fires and movement drops to 2.8 degrees per block for the remaining 640
steps. Four times the budget bought one outer iteration's worth of progress,
not four.

## Why it matters more than it looks

**An optimizer that is not taking steps cannot hear its objective.** This is
what made the problem hard to see: every attempt to fix the resulting path
shape by changing what was being minimised failed, and each failure looked
like evidence about the cost function.

- 32 combinations of `rho` and growth and iteration budget moved self-clearance
  from -76 mm to at best -49 mm (`BENCHMARKS.md` 12.1) and the note there
  concludes "the wall was not a weighting problem" -- correct, and it was not a
  wall either.
- An explicit L1 joint-travel term at weight 50, large enough to be comparable
  to the whole rest of the objective, changed total travel by 0.02%.
- Letting the scale grow was tried first at caps of 8 and 64 and made things
  WORSE, which is its own finding: the scale multiplies Adam's own step, so a
  cap above 1 is overshooting a direction Adam has already sized. Accept rate
  fell to 60%.

Recovery at `x1.3` capped at 1.0 -- back to Adam's step, never past it -- took
the approach from 645 to 586 degrees of joint travel on its own, and made the
travel term audible: with both, 560 degrees, which is the geodesic.

## Where it is fixed and where it is not

`freespace.py` recovers now (`TRUST_GROWTH`), and `BENCHMARKS.md` 12.5 has the
before and after across the corpus.

**`weld_trajopt.py` is untouched and has the identical ratchet** at line 620, in a
loop that also runs a hard seam projection every step. The projection moves the
iterate outside the objective's logic in a way free space has nothing
equivalent to, and what a recovering step size does in the presence of it has
not been measured. Fixing it on the strength of a measurement taken on
free-space links would be assuming the answer transfers; M5's numbers are
pinned in `BENCHMARKS.md` 11 and any change has to move them knowingly.

`tests/motion_planner/test_weld_trajopt.py::test_the_trust_region_recovers_after_a_rejected_step`
is marked `xfail` against this and passes when it is fixed.

## The gap this exposed and then closed, kept because it will come back

Part-way through, the T2 verifier was finding a link in contact on `bracket_a_2x`
where the optimizer's own screen said it was clear:

    transit seam_0003   screened env +0.11 mm   verifier worst 0.00 mm
                        link_6 vs workpiece, segment 0, t = 0.75

That link now clears by 2.2 mm and the verifier passes it, so nothing is owed on
this plan request. The failure MODE is worth writing down anyway, because it is
structural and the next tight cell will produce it again: the optimizer samples
knots and their midpoints, so `t = 0.75` of a segment is a point it never looks
at, and spheres against a trilinear field carry up to 1.6 mm of error against
the exact mesh (`BENCHMARKS.md` 5) -- so any link whose screened clearance is
under a millimetre or two is a link the screen cannot speak for. Both halves are
why `ARCHITECTURE.md` says the ESDF serves the search only.

What made the difference was not more sampling but a correct cost: an AL on
`clearance >= 0` with `rho` high enough that a penetration outranks a standoff,
and a buffer whose pull is bounded so it cannot trade the constraint away.
Should the mode return, the untried options are denser sampling on the segments
adjacent to a weld, or the verifier's own certificate inside the accept test.
Neither has been measured, and the millimetre-scale weight tuning that looks
like an alternative is not one: the same run twice differs by ~0.06 mm through
the self-collision backward's atomics.
