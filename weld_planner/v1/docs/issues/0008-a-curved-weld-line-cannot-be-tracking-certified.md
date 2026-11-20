# 0008 · A curved weld line cannot be tracking-certified, so one seam ships with a trajectory nobody can prove

**Status:** fixed 2026-09-17 -- the bound exists, for rational curves too,
and the certificate decides curved weld lines. It decided the one in the
corpus is out of tolerance; see "What was done". Pinned by
`tests/motion_planner/test_tracking.py` (`test_the_curved_bound_is_a_bound_on_the_target_speed`,
`test_the_curved_seam_is_decided_on_the_real_cell_too`).
**Found:** 2026-08-13, the moment the motion corpus was replaced by three real
placed projects. The branch it exercises was written on 2026-08-13 that
morning, unit-tested against a synthetic degree-2 chord, and had never run
against a part.
**Blast radius:** 1 seam of 52 in the corpus — `hitch_2x seam_0008`, a 239 mm
degree-2 curve. Every other seam is a straight weld line and certifies.

## What is wrong

`verify.certify_tracking` proves the electrode stays within
`tolerance.position_mm` of the seam at every instant, not merely at the
samples. It does that by bounding how fast the two points can separate, and
one half of that bound is `SeamTarget.lipschitz_f` — how fast the TARGET can
move along the seam. That function is exact on a degree-1 weld line and
returns `None` on anything else, so the certificate refuses:

    seam_0008: cost 52.82 -> 9.15, acc 0.54 -> 0.23 of limit,
               track: knots 0.0 um, midpoints 0.025 mm, rate violations 0
    tracking certificate: {'verdict': 'REFUSED', 'bound_mm': inf,
      'unverifiable': ['tracking: no derivative bound exists for the seam
       target on segment 0 of the trajectory (fractions 0.0000..0.0083)'],
      'pieces': 0, 'evaluations': 0}

**Read the two lines together, because that is the whole shape of this.** The
synthesis worked: the cost fell by a factor of six, the knots sit on the seam
to a tenth of a micron, and the optimizer's own between-knot measure is
0.025 mm against a 0.5 mm tolerance. The T2 collision verifier passes it, since
that certificate needs no seam derivative. Everything says the trajectory is
good, and the one thing that would PROVE it declines to speak.

That gap is the feature, not the defect. A plausible number beside an honest
"not checked" is exactly what `unverifiable` exists to say — the alternative
is a bound computed from an assumption the geometry does not satisfy, which
would certify a curve using a constant only a straight line has.

## How it was found

By replacing the corpus, not by a test. Five of the six old plan requests were
generated from bare STEP files with `default_placement`, and every seam in
all of them is degree-1; the sixth was hand-placed and also straight. So the
refusal branch had a passing unit test against a synthetic curve and zero
coverage on real geometry, which is the coverage that matters — the synthetic
case proves the `return None` fires, and only a real one proves anything
about what happens downstream of it.

Worth recording because the corpus swap found two other things the same way:
`_seam_oracle` had never read a seam's `direction` (no old fixture had a
reversed seam, so the parity oracle was quietly weaker than it read), and
`test_duplicate_branches...` indexed `plan.seams[1]` and had stopped testing
anything the moment the data moved. **A fixture set that only contains easy
cases does not report that it is easy.**

## Why it happens

`weld_motion_planner/verifier/verify.py`, `SeamTarget.lipschitz_f`. It needs
`sup |dT/df|` over a fraction sub-interval, where `T = P + standoff * u` is
where tool0 belongs. Two terms:

- **The weld point.** `|dP/df|` is exactly the seam's length, at any degree,
  because `s` is normalized ARC LENGTH — no bound is involved at all.
- **The torch direction.** `standoff * |du/df|`, and `u` is built from the
  tangent `t`, the work-angle datum `r`, and the two authored angles. On a
  STRAIGHT segment `t` is constant, so all the turning comes from `r` (affine
  there, so `|dr/df| <= |w'| / min|w|` with both closed-form) and from the
  angle slopes (constant there). Everything is arithmetic.

On a curved segment `t` turns, and its rate is `|dt/df| = kappa * L`. So the
bound needs `sup kappa` over the sub-interval, and nothing computes one.

## What was done

The refusal, and nothing else. `lipschitz_f` returns `None`, `certify_tracking`
puts the reason in `unverifiable`, `bound_m` is infinity, and the verdict is
`REFUSED` rather than `FAIL` — because "I could not check" and "I checked and
it is bad" are different sentences and the verifier's whole job is to keep them
apart.

## What is still owed

**The curvature bound.** `kappa = |C' x C''| / |C'|^3`, and in arc length it
is just `|d^2 P / ds^2|`, so `sup kappa <= sup|C''| / inf|C'|^2` over the
parameter range. Both ends are available without sampling:

- A B-spline's derivative is a B-spline whose control points
  `Segment._d1` and `_d2` already build, and by the CONVEX HULL property the
  derivative's value on a knot span is inside the hull of the local control
  points. `sup|C''|` is therefore the max norm over those, exactly.
- `inf|C'|` is the distance from the origin to that hull — cheap and
  rigorous; a valid lower bound is `max over any unit v of min_i (C'_i . v)`,
  taking `v` as the normalized mean of the derivative control points.

**Non-rational only.** The hull argument is about the control polygon of the
projected curve, and the quotient rule breaks it for a rational spline. Every
weight in the corpus is absent (i.e. 1), so the fast path covers what exists;
a rational curve must keep refusing rather than be bounded by an argument
that does not apply to it.

**Then the rest of `u` follows.** With `sup kappa` in hand the existing chain
closes: `|dt/df| = kappa * L`, and `|du/df| <= (1 + AXIS_SENSITIVITY) *
(|dr/df| + |work'|) + |travel'|` as it already does — but `r`'s own rate
stops being the affine closed form, because `w` is affine only while `t` is
constant. That is the second piece and it is the harder one.

**And a note on where the hard cases live.** `hitch_2x seam_0008` is not only
the corpus's one curved line. It is also the only seam where M4's ruler and
M5's disagree — the sweep's own winner is not the one this stage picks — and
the only path whose best traversal moves the positioner through more than one
bin. A 239 mm degree-2 sweep gives every redundancy room to matter, which is
worth knowing when choosing what to test the next thing against.

## Fixed (2026-09-17)

Written because the OLP's hand-placed welds lower to cubic Bezier seams
(D28): every one with a C1 or C2 waypoint came back `tracking: REFUSED`.

**The curvature bound, rational included.** `verify._curvature_bound`. On a
stretch of one knot span the homogeneous curve `A = (w C, w)` is a
polynomial, and both derivatives of `C = A / w` are polynomials over powers
of `w`:

    C'  = N / w^2,   N = A' w - w' A
    C'' = M / w^3,   M = (A'' w - w'' A) w - 2 w' N

so `kappa = w |N x M| / |N|^3 <= w |M| / |N|^2`. `w`, `N` and `M` are fitted as
Bezier polynomials on the stretch (interpolation at interior Chebyshev
nodes) and bounded by their control points: `w` and `|M|` from above, `|N|`
from below by `min_i N_i . v`. A stretch whose `N` hull reaches the origin is
halved, up to eight times. The rational case the section above said must
refuse is therefore bounded after all: the quotient rule is applied to
polynomials, not to the hull. The bound is 1.13x the true curvature on a
5.6 degree stretch of `hitch_2x`'s 40 mm circle and 4.6x over a whole
120 degree span.

**The datum.** `SeamTarget._curved_datum_rate`: `r = w / |w|` with
`w = D - (D.t) t`, so `|dr/df| <= (|D'| + |D| tau) / |w|` with `tau = kappa L`.
`|D'|` is closed-form per datum (zero for a fixed vector; `L (1 + L_rail_i /
L_i)` for a rail read per segment; `L + span * L_rail` for one read at the
shared `s`), and `|D|`, `|w|` are carried from the piece's middle by those
rates. A piece that could come within `DEGENERATE_DOT` of the tangent is
refused, since the planner substitutes a different datum there. `u` then
gains the tangent's own turning: `|du/df| <= travel' + (1 + A)(work' + A tau
+ |dr/df|) + A tau`. A straight piece keeps the original closed form.

**How it is held.** The derivation is not trusted: on random bent cubic
seams (fixed, rational, rail-per-segment and rail-at-shared-s data, angle
and standoff profiles) no difference quotient of `T` may exceed the bound --
the mean value theorem, 586 bounded stretches (18 refused), none over. Dropping the `tau` term
fails it in three of the four families.

**What it found.** `hitch_2x seam_0008` is no longer refused; it FAILS:
witness 1.707 mm against 0.5 mm, 13 trajectory segments out, and M5's own
midpoint measure agrees at 1.597 mm. The "midpoints 0.025 mm" quoted above
was on the corpus and cell of 2026-08-13; on today's the synthesized
trajectory does not hold a 40 mm circle between knots, and that is now a
planner finding rather than a hidden one. Owed: the weld trajopt's tracking
on tight curves, which is a new issue, not this one.
