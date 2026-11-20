# 0015 — Free space is Catmull-Rom where the architecture says B-spline

**Status:** open, opened 2026-08-26 after a head-to-head against cuRobo on this
cell. The decision taken is to rebuild the free-space stage on cuRobo's
structure and keep this repository's collision model. This document is the
measurement that justified it.

## What is wrong

`docs/motion/ARCHITECTURE.md` opens **Representation** with "Joint-space
B-splines over arm + positioner joints", and free space is covered by the same
section: "Approach, retract and transit phases are ordinary knot-parametrized
joint splines".

`freespace.py` does not do that. Its only optimization variable is the knot
POSITION tensor:

    opt = torch.optim.Adam([self.x], lr=ADAM_LR, capturable=capturable)

and each knot's tangent is a centred difference computed after the fact
(`velocities`). A cubic Hermite whose tangents are centred differences of its
own knots is a **Catmull-Rom spline**. Per axis it carries 16 degrees of
freedom, not 32: the tangent is not free, so the optimizer cannot choose it.

The consequence is the continuity class. Catmull-Rom is `C1`: position and
velocity are continuous, **acceleration is not**. Evaluating the analytic
second derivative either side of each interior knot on one transit:

    knot |  a(u->1-)   a(u->0+)  |  jump (worst axis)
       1 |    0.0000    -1.0472  |    1.7282
       2 |    2.0944     1.0472  |    1.6655
       3 |    1.0472     1.0472  |    0.2390

A `C2` curve would jump by zero. 1.7282 rad/s^2 is **0.83 of this cell's own
acceleration limit** (2.094 on J1..J3), spent on nothing but the seam between
two segments.

## Why it was not caught

`verify.check_limits` differences the waypoints it is handed:

    velocity = np.diff(waypoints, axis=0) / dt[:, None]
    acceleration = np.diff(velocity, axis=0) / dt[:-1, None]

`verify()` does not resample first, so the grid it judges is the knot grid.
The acceleration jump lives BETWEEN knots, and the check steps over it.
Measured across 12 randomly sampled transits, obstacle-free, self-collision on:

    acceleration ratio    knot grid 1.04   |   curve 2.05

So the trajectory exceeds the limit on the planner's own certification grid in
half the cases, and by roughly twice as much where nobody looks.

## How it happened

M5's weld pass IS a real cubic Hermite, for a reason recorded in commit
`4c87e65`: the tip velocity at every knot is solved by DLS and imposed as
DATA, which is a process condition ("the tip's speed stops wobbling between
samples") and drops the between-knot sag two orders, to `h^4/(384R^3)`.

Free space inherited the interpolant and could not inherit the reason -- there
is no tip speed to match -- so the tangents were filled in by differencing.
The representation therefore carries Hermite's cost (`C1` only) without using
Hermite's benefit (a tangent you get to choose). No commit, issue or comment in
this repository records that as a decision.

A torch de Boor evaluator already exists (`geometry/curves.py`), batched and
differentiable, but it is for the seam's rational B-splines -- `FK(q(s_i))`
against `P(s_i)` -- and nothing in the trajectory path imports it.

## What cuRobo does instead

Measured with the same robot, the same 12 problems, the same per-axis limits
(`max_acceleration` set from this cell's profile, not cuRobo's default 10.0),
and both trajectories sampled at the same dt:

| | ours | cuRobo |
|---|---|---|
| duration (s) | 3.53 | 2.76 |
| acceleration ratio | 3.35 | **0.99** |
| jerk RMS | 0.81 | 0.15 |
| speed CV | 0.76 | 0.59 |

cuRobo's acceleration ratio is 0.99 on **all twelve**, which is not luck: it
optimizes B-spline control points (`control_space: BSPLINE_3`, `n_knots: 16`,
a `C2` curve), evaluates costs on the rolled-out states, and shrinks each
bound by `activation_distance: 0.01` so the hinge engages 1% inside the limit.
Its cost pairs a bound hinge with an always-on squared regularization, and
retimes both by `dt^n` -- `n = 1, 2, 3` for velocity, acceleration, jerk -- so
the objective approximates a time integral and is invariant to the horizon.
`freespace.py` has no such dt factor, which `retime`'s own docstring already
notes as a hazard ("a seed's cost on a 3.8-second horizon and an answer's on a
1.4-second one are not the same quantity").

## Caveats on those numbers

Stated so the table is not read as more than it is.

- **Obstacle-free.** Neither side was given the part or the cell meshes; both
  ran with self-collision only. Detour is ~1.0 on both, so no path-finding
  ability was exercised.
- **Self-collision is not symmetric.** Ours uses the structural adjacent-pair
  rule (`collision/pairs.py`); cuRobo was given neighbours AND second
  neighbours, because with neighbours alone it refused every sampled
  configuration.
- **Not verified:** whether cuRobo's `interpolated_trajectory` is retimed to
  respect limits as a post-process. If it is, its 0.99 is partly earned there
  rather than in the optimizer, and the gap in the table is overstated.

## What is owed

Rebuild the free-space stage on cuRobo's structure -- `C2` control-point
representation, bound hinge with activation distance, dt-retimed weights --
while keeping this repository's collision model, its sphere/field split, and
its verifier. Until then `ARCHITECTURE.md` carries a note pointing here.
