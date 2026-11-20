# Weave, manipulability, and what verification can and cannot prove

Planner-side guidance. The contract itself is in `schema/SEAM-MODEL.md`; this
document explains why weave is a *parameter* there rather than geometry, and
what a planner must do to honour it safely.

Read §5 before trusting any simulation result.

---

## 1. Weave is a claim on the velocity budget, not a set of waypoints

The single most misleading thing about weave is that it looks small.

Amplitude is 1.5–6 mm on a robot with ~1400 mm reach. It reads as a rounding
error on the seam. In **displacement** it is. In **velocity** it is the dominant
motion, and that is what the robot has to deliver:

```
 v mm/s  a mm  f Hz  |v_trans| pk   tip pk  tip/travel  path ratio
    8.0   0.0   0.0           0.0      8.0        1.0x        1.00   no weave
    8.0   1.5   3.0          28.3     29.4        3.7x        2.53   light, 3 mm plate
    8.0   3.0   3.0          56.5     57.1        7.1x        4.67   fixture example
    5.0   6.0   2.0          75.4     75.6       15.1x        9.70   heavy, thick plate
   12.0   4.0   4.0         100.5    101.2        8.4x        5.49   fast everything
```

The tip moves 3.7x to 15x faster than the travel speed, over a path 2.5x to 9.7x
longer than the seam. The robot is performing a 3 Hz shake with a slow drift
superimposed, not a smooth traverse with a wiggle.

Two consequences drive everything below:

- Expanding weave into waypoints inflates the problem ~30x (150 centerline
  waypoints becomes 4,500+). That is fatal for *optimization* and merely
  expensive for *verification*. Keep them on opposite sides of the line.
- The velocity and acceleration budget must be **reserved before** the
  centerline is optimized, never checked afterward. See §4.

---

## 2. Geometry

### Axis

Weave displacement is along **`b'`**, the transverse axis of the seam frame
(see `SEAM-MODEL.md` §5 for the frame construction).

`b'` is perpendicular to both the travel direction and the electrode axis —
exactly, for any work and travel angle. Verified numerically to 5.55e-17 over
work -60..60 deg and travel -30..30 deg. It falls out of the construction:
`b' = t x r'` is perpendicular to `r'` by definition, and rotating `r'` about
`b'` to obtain the electrode direction preserves that.

So the frame built for the angles already contains the weave axis. Nothing extra
is needed.

On the reference fixture (90 deg fillet, work 0, travel 10):

```
b' = [0, -0.7071, -0.7071]
  +3 mm -> dy = -2.12, dz = -2.12 mm   (toward the block, down)
  -3 mm -> dy = +2.12, dz = +2.12 mm   (toward the plate, up)
```

That sweeps along the hypotenuse of the fillet triangle, from the toe on one
member to the toe on the other. Which is what a fillet weave is.

**Not along `r`.** `r` points at the torch. Oscillating along it would modulate
standoff distance, which is arc-length control, not weave.

### Shape functions

A weave is a closed 2D curve in the `(b', t)` plane, parametrized by phase:

```
W(s, phi) = a(s) * [ wb(phi) * b'  +  c * wt(phi) * t ]
```

`a(s)` is `amplitude_mm`, `c` is `travel_ratio` (0 for planar shapes), and
`wb`, `wt` are unit-amplitude shape functions.

These are **this module's definitions**, not a standard. Vendor names for weave
shapes disagree, so each is pinned explicitly rather than left to folklore.

| `shape` | `wb(phi)` | `wt(phi)` | notes |
|---|---|---|---|
| `none` | 0 | 0 | stringer bead |
| `sine` | `sin phi` | 0 | planar |
| `zigzag` | triangle wave | 0 | planar, constant transverse speed |
| `trapezoidal` | trapezoid wave | 0 | planar, dwell at the toes |
| `circular` | `sin phi` | `1 - cos phi` | always advances, never reverses along travel |
| `crescent` | `sin phi` | `+/- sin(2 phi) / 2` | bows forward (convex) or back (concave) |
| `triangular` | triangle wave | `(1 + triangle wave)/2` | advances on one leg; vertical-up fillets |

`tilt_deg` rotates the weave plane about `t`. Zero means the plane contains
`b'` and `t`.

### Phase, frequency, and the dwell trap

Frequency is in **Hz**, matching what real controllers accept. Because travel
speed is a scalar function of `s`, a fixed frequency means the weave pitch
tightens automatically wherever travel slows — which is generally desirable at
a corner.

**Dwell extends the period.** `dwell_left_s` and `dwell_right_s` stop the torch
at the toes, so the true cycle time is:

```
T = 1/f + dwell_left_s + dwell_right_s
```

At `f = 3 Hz` with 0.1 s dwell each side, the actual cycle is 0.533 s, i.e.
**1.9 Hz**, not 3 Hz. Anything computing heat input, pitch, or excitation
frequency must use the effective rate, not the nominal one.

Dwell is also the one place where weave is irreducibly a *time* behaviour rather
than a function of arc length. Be explicit about it rather than pretending weave
is purely spatial.

### Discontinuities

At a `C0` corner `b'` flips discontinuously. A torch caught mid-oscillation
jumps several millimetres instantly. `corner_policy` selects the handling:

- `ramp_out` — taper amplitude to zero before the corner and back up after.
  Default, and see §4 for a second reason to prefer it.
- `force_zero_crossing` — retime so phase is at a zero crossing at the corner.
- `continue` — accept the jump. Only sane when amplitude is very small.

`ramp_in_mm` and `ramp_out_mm` do the same at seam ends. You do not strike an
arc mid-weave.

---

## 3. Travel speed means along the seam

`travel_speed_mm_s` is speed along the **seam curve arc length**. It is never
tip speed.

This is the universal welding definition and the physically meaningful one:
heat input is `V * I / travel_speed` and bead cross-section is deposition rate
over travel speed, both per millimetre *of joint*. Given the 7x gap between
travel speed and tip speed, misreading this field is expensive in both
directions — a planner that treats it as tip speed will under-deposit by a
factor of several.

Tip speed is derived, and belongs to the planner. There is deliberately no
`max_tip_speed` field: that is a robot capability, not weld intent.

---

## 4. Planning: weave never enters the optimization

Weave imposes **a floor on transverse mobility**, and nothing else. The planner
solves the centerline subject to staying weave-capable; the weave itself is
applied afterward, inside a region already proven feasible.

### The right index

Not Yoshikawa's `w = sqrt(det(J J^T))`. That measures the *volume* of the
velocity ellipsoid, and it can look perfectly healthy while the ellipsoid is
paper-thin in exactly the direction needed. Weave cares about one direction.

Use **transverse mobility**:

```
mu_b'(q) = 1 / || J_pinv * b' ||_2
```

Maximum Cartesian speed along `b'` per unit joint velocity. The constraint:

```
mu_b'(q)  >=  2 pi f a / (beta * || q_dot_max ||)
```

The per-joint form is stricter and preferable as a hard constraint:

```
| [J_pinv * b']_i | * 2 pi f a  <=  beta_i * q_dot_max_i     for all i
```

The scalar version is the convenient planner cost term. Both need one
pseudoinverse, which the singularity check already computes.

Because `b'` comes from the weld program, this is a seam-frame constraint that
drops straight in alongside collision and joint limits.

### Reserve the budget, do not check it afterward

`beta` is weave's share of the velocity and acceleration budget. Given weave is
80-90% of tip motion, `beta ~ 0.5` is realistic rather than conservative.

**It must be reserved before the centerline is optimized.** A time-optimal
parametrization will consume the entire budget by construction, leaving weave
nothing — and the failure surfaces only after expansion, when replanning is
most expensive.

### Acceleration binds before velocity

```
a=1.5 mm  f=3 Hz  ->  6 reversals/s,  peak accel 0.53 m/s^2
a=3   mm  f=3 Hz  ->  6 reversals/s,  peak accel 1.07 m/s^2
a=4   mm  f=4 Hz  ->  8 reversals/s,  peak accel 2.53 m/s^2
```

Peak transverse acceleration is `(2 pi f)^2 * a`, sustained for the whole weld.
Wrist joint torque and acceleration limits will bind before velocity limits, and
they do so continuously for 90+ seconds, not as a transient.

### The constraint is tightest where the posture is worst

Corner slowdown drops travel speed while frequency holds. On the fixture that
takes tip/travel from 7.1x at 8 mm/s to **11.3x at 5 mm/s** — the peak
manipulability demand lands exactly at the corner, which is also where the arm
is most likely to be mid-reorientation.

This is the second and stronger argument for `corner_policy: ramp_out`: tapering
amplitude through a corner relieves the demand precisely when it peaks.

---

## 5. Verification

### Checking is not optimizing

Collision-checking 11,000 waypoints is a linear sweep. Optimizing over 11,000
waypoints is not. Expansion is affordable for verification and fatal for
planning. Keep the line sharp.

**The final weaved trajectory must pass collision verification.** The
centerline plan and the `mu_b'` floor are necessary but not sufficient — a 3-6 mm
sweep toward the adjacent member is a real clearance risk against clamps,
fixtures, and the other member of the joint.

### Why not let the controller do it

ABB, Fanuc and KUKA all superimpose weave at **servo rate in the motion
controller**: the program carries the seam path plus a weave struct, and the
planner never sees the expanded waypoints. RTCore runs at 1 kHz with the
trajectory preloaded, which would give ~333 samples per weave cycle — ample.

**Rejected**, because nothing would ever verify the poses actually commanded. No
collision check, no limit check, no manipulability check on the motion that
physically happens. In a stack with collision *detection* but no collision
*avoidance*, layering unverified servo-rate motion on top makes the safety story
worse rather than better.

The nuance worth keeping: once the plan request is verified at plan time, executing
via superposition *is* safe, because the plan request is what was proven.
Superposition is an execution optimisation that verification licenses — never a
substitute for it. See `DECISIONS.md` D11.

### Sampling density, and why discrete checking is not enough

Worst-case gap between consecutive tip positions is `peak_tip_speed / (N * f)`
for `N` samples per weave cycle:

```
case                   pk mm/s    N=16    N=32    N=64  N for 0.5mm  total wpts
light, 3 mm plate         29.4    0.61    0.31    0.15           20        5625
fixture example           57.1    1.19    0.59    0.30           39       10969
heavy, thick plate        75.6    2.36    1.18    0.59           76       22800
fast everything          101.2    1.58    0.79    0.40           51       12750
```

At `N=16` the tip jumps 1.2-2.4 mm between checks. On 3 mm sheet and clamp
hardware that is enough to tunnel clean through a fixture between two "clear"
samples. Reaching 0.5 mm gaps needs 39-76 samples per cycle, i.e. 11k-23k
waypoints.

**Use continuous collision detection.** Bullet does swept-volume CCD and
Tesseract+Bullet is already the collision backend here. CCD is sound at far
coarser sampling because it tests the volume swept between poses rather than the
poses themselves. That converts "23,000 samples and still not provably safe"
into "a few thousand and provably safe."

### Making it fast

Discrete checks are embarrassingly parallel — every waypoint is independent, and
so is every CCD segment. Three independent wins, in order of payoff:

1. **Persistent worker.** The current path forks `python3 collision_check.py`
   per call. Process startup dominates; the collision math does not.
   `tesseract/v1/python/persistent_collision_check.py` is a single-process
   server with a result cache and no threading — that is the thing to build on.
2. **Clone-per-thread pool.** Bullet contact managers are not thread-safe on
   shared state; Tesseract's expose `clone()` for precisely this. 24 cores on
   the current dev machine, so ~22 workers. A 45 s serial sweep becomes ~2 s.
3. **CCD**, which cuts the number of checks needed in the first place.

**GPU is the wrong trade for this specific job.** The mature option, cuRobo,
gets its speed by approximating the robot as spheres against a voxel/ESDF world.
That is excellent for reaching and air moves and wrong for millimetre weave
clearance, because the approximation error is the same order as the quantity
being measured. Exact mesh CCD on GPU exists, but the gain over 22 CPU cores
does not justify the engineering cost. Spend the GPU on the planner instead.

One more: the existing verdict path reports
`tesseract_collision_geometry: visual_meshes_as_collision_meshes`. For
millimetre clearance that matters far more than it does for a 50 mm air move.
Confirm what geometry is actually being tested before trusting a clear result.

---

## 6. What no kinematic check can prove

**This is the most important section in the document.**

Every kinematic check can pass — joint limits, velocity, acceleration,
collision, transverse mobility — and the physical TCP can still be somewhere
other than where the trajectory says it is.

```
a=3 mm  f=3 Hz  ->  6 reversals/s,  excitation at 3 Hz
a=4 mm  f=4 Hz  ->  8 reversals/s,  excitation at 4 Hz
typical loaded industrial arm, 1st structural mode: ~5-20 Hz,
lower at full extension
```

A sustained 3-4 Hz oscillation is driven into an arm whose first structural mode
may sit near 5 Hz when extended and loaded. The URDF is rigid. It has no
compliance, no backlash, no drive-train dynamics, and no controller model.
Simulation against it cannot answer whether the arm rings.

Consequences:

- A "verified" weaved trajectory means **kinematically feasible and
  collision-free as modelled**. It does not mean the torch goes where the plan
  says.
- Path deviation from structural excitation is a dynamics and controller
  question and requires hardware measurement.
- Resonance is posture-dependent. An extended arm has a lower first mode than a
  tucked one, so a frequency that is safe in one part of the cell may not be in
  another.

Never report a simulation verifier as proof of weld quality. Report it as what it
is.

---

## 7. Test tiers

Ordered by what can actually answer the question.

**T1 — per plan, cheap.** `mu_b'` floor as a hard constraint at every centerline
waypoint, with `beta` reserved before optimization. Catches the problem where it
is cheapest to fix, by changing posture through the free spin DOF or the
positioner.

**T2 — per plan, final verifier.** Expand weave, then run CCD collision plus joint
position, velocity, and acceleration limits on the expanded trajectory. Fail
closed. Nothing ships without this.

**T3 — once per robot and cell, adversarial.** Sweep a seam across the reachable
workspace and *find* the minimum-`mu_b'` postures rather than sampling typical
ones. Confirm T1's constraint binds strictly before any real limit does, and
measure how much margin `beta` actually buys.

T3 exists because "welding is slow, limits will never bind" is true generically
and false exactly where it matters:

```
peak tip 57.1 mm/s = 2.9% of a 2000 mm/s arm        <- enormous Cartesian headroom
but q_dot = J_pinv * v, and ||J_pinv|| diverges near a singularity
fixture speeds break a 180 deg/s joint limit once mu_b' < 0.0182 m/s per rad/s
```

A test that samples typical postures will pass and prove nothing. T3 must seek
the bad ones.

**T4 — hardware, unavoidable.** Command a known weave and measure the real TCP
path. Sweep frequency to find where the arm rings. Only this tier can answer §6.

T4's output should return as a **frequency ceiling per posture region**, which
T1 then enforces as an additional constraint. That closes the loop: the one
thing simulation cannot determine becomes a parameter simulation can respect.
