# The Weld Program — `amr-weld-planner-v1.weld-program.v1`

The contract between CAD understanding and motion planning.

A weld program says **what weld is required**. It never says how a robot gets
there. It contains no joint values, no TCP poses, no reachability claims, and no
robot. Two different planners must be able to read the same program and produce
two different, both-correct trajectories.

---

## 1. What a weld is, in this model

Four things, and nothing else:

1. **The line the electrode tip follows** — a curve in space.
2. **Work angle at every point on that line** — measured against a reference
   direction supplied by a second curve.
3. **Travel angle at every point** — measured against the same frame.
4. **Process parameters** — travel speed, wire, voltage, gas, weave, everything
   in the WPS.

Rotation of the torch about its own electrode axis is **not** part of the
definition. A weld does not care. That degree of freedom belongs to the planner
and the schema has no field that can accidentally take it away.

---

## 2. Design rules

These are invariants. A field that violates one is a bug in the schema.

**R1 — No motion.** No joint values, poses, quaternions, or robot links.

**R2 — 5 DOF, never 6.** Position (3) plus electrode direction (2). Spin is free
unless a seam sets `spin.mode` explicitly, which exists only as an escape hatch
for tooling that genuinely constrains it (e.g. an off-axis wire feed).

**R3 — No baked sampling.** Anything varying along a seam is a *function of
normalized arc length*, never a per-sample array. Storing arrays is how the
prior implementation destroyed curvature: uniform resampling put one waypoint on
an R ~ 7 mm corner and the torch cut 1.52 mm inside the seam. The planner picks
sampling density, because only the planner knows the tolerance, the dynamics,
and the collision environment.

**R4 — Curves stay analytic.** Tangent needs C1, curvature needs C2. Finite
differences over a polyline are noisiest exactly at tight corners, which is
exactly where they matter.

**R5 — Workpiece frame only.** The planner is free to move the positioner.
Nothing upstream may commit to a world pose.

**R6 — Units live in field names.** `_m`, `_mm`, `_deg`, `_rad`, `_mm_s`,
`_l_min`. No global units block, no unit-carrying siblings. Follows the
convention already used across this repo (`xyz_m`, `positions_rad`,
`tcp_speed_m_s`).

Lengths in **geometry** are metres (`_m`) because they feed a planner.
Lengths in **process parameters** are millimetres (`_mm`) because that is how a
WPS is written. Angles authored by humans are **degrees**. The suffix is always
present, so there is nothing to infer.

---

## 3. Curves

### One representation

Every curve is a **rational B-spline (NURBS)**. There is no second curve type.

| Need | Representation |
|---|---|
| Line segment | degree 1, 2 control points |
| User-supplied sample list | degree 1, N control points |
| Arc, circle, ellipse | degree 2, rational, exact |
| Free-form seam | degree 3+ |

NURBS subsumes all of them *exactly* — conics are represented with zero error,
not tessellated. One evaluator (de Boor), one code path.

A `analytic` block may be attached to any segment as an annotation: the exact
line/circle/arc parameters, for readability, cheap special-case paths, and
round-trip checks against the source STEP. **The B-spline is the source of
truth.** If the two ever disagree, the B-spline wins and the analytic block is
stale.

### Segment definition

```
degree            p
control_points_xyz_m   n+1 points
weights           n+1 values, optional; absent means non-rational (all 1)
knots_u           length n+p+2, non-decreasing, clamped
                  (first p+1 equal, last p+1 equal)
```

A line: `degree 1`, 2 control points, `knots_u [0,0,1,1]`.
A quarter arc: `degree 2`, 3 control points, `weights [1, cos(45°), 1]`,
`knots_u [0,0,0,1,1,1]`.

### Composites

A curve is a list of segments joined end to end. The dogbone seam is 4 arcs plus
4 lines and stays that way — fusing it into one spline would lose exactness and
lose the corners.

`continuity_to_next` (`C0` | `G1` | `C1` | `C2`) marks each junction. `C0`
corners are where a planner should consider slowing, dwelling, or re-seeding
redundancy. It is advisory; the planner decides.

`closed: true` means the last segment's end coincides with the first segment's
start.

### Arc length

**`u` is a dial. `s` is a distance.** Turning the dial at a constant rate does
not move you along the curve at a constant rate. A B-spline is a function
`C(u)`; `u` indexes the control-point blending math and carries no metric
meaning.

Normalized arc length `s ∈ [0,1]` over the whole composite is therefore the
**canonical parameter for everything in this document**. Two independent effects
make `u ≠ s`, and they stack.

**Across segments.** Take a closed rectangle of 250, 125, 250, 125 mm — the
shape the reference fixture was *wrongly assumed* to be, which makes it a useful
worked example. Give each segment an equal share of parameter range and:

```
u=0.250 -> arc 250.0 mm      s=0.250 means arc 187.5 mm     off by 62.5 mm
u=0.750 -> arc 625.0 mm      s=0.750 means arc 562.5 mm     off by 62.5 mm
```

62.5 mm of error on a 750 mm weld. Note the trap: `u=0.5` → 375 mm, exactly half
of 750, a coincidence of this rectangle (250+125). Spot-check the midpoint and
you would conclude it was fine.

**Within a segment.** A degree-1 spline with 3 control points and uniform knots
— the "user-supplied samples" case:

```
|AB|=10  |BC|=90    u=0.500 -> s=0.100     40 mm off, out of 100
|AB|= 1  |BC|=99    u=0.500 -> s=0.010     49 mm off, out of 100
|AB|=50  |BC|=50    u=0.500 -> s=0.500      0 mm off
```

Parameter midpoint is nowhere near curve midpoint unless samples happen to be
evenly spaced — which curvature-adaptive CAD tessellation never is. Even an
exact arc is non-uniform: on a rational quadratic quarter circle, `u=0.25` lands
at 21.60°, not 22.5°.

A rectangle is the easy case — every segment is degree-1 with two control
points, so within-segment `u` is genuinely proportional to arc length. The real
fixture is a dogbone with four 270 degree arcs, and hits both effects at once.

**Why the schema forces `s`.** Every scalar function is defined on it, so
`work_angle_deg` at `s=0.5` means halfway along the *weld*; feed `u` and the
whole profile lands 62.5 mm out of position. Travel speed is mm/s of arc length.
`max_chord_deviation_mm` and waypoint spacing are arc-length statements.

**The conversion.** Forward is `s(u) = ∫|C′(u)| du` by Gauss–Legendre
quadrature. Inverse is a Newton root-find on `s(u) − s_target`, where the
derivative needed, `|C′(u)|`, is the integrand already in hand — 2–3 iterations
at a sane tolerance. Unavoidable, because arc length of a polynomial curve has
no closed form in general; only straight lines give `s ∝ u` free. Needed anyway,
because constant travel speed requires it. In practice, build a `(u, s)` lookup
table once per curve, then binary search plus one Newton polish.

`length_m` is carried on each segment and on the curve as a cached convenience.
It is derived and may be recomputed; it is not authoritative.

---

## 4. Scalar functions

Any quantity varying along a seam uses this shape. `s` is normalized arc length
on the seam curve.

```json
{"kind": "constant", "value": 10.0}

{"kind": "piecewise_linear", "knots": [[0.0, 8.0], [0.4, 14.0], [1.0, 8.0]]}

{"kind": "bspline", "degree": 3, "knots_u": [...], "control_values": [...]}

{"kind": "samples", "s": [...], "values": [...], "interpolation": "linear"}
```

`piecewise_linear` knots must have strictly increasing `s`, with the first at
`0.0` and the last at `1.0`.

`samples` exists for measured or hand-tuned data. It is the one place where a
sampling *is* the data — but it is a sampling of a **parameter**, not of the
geometry, so R3 is intact.

---

## 5. The seam frame and the two angles

This is the core convention. Everything else is bookkeeping.

### The rail

Work angle needs a reference direction, and "relative to what" has no
universally right answer — the bisector, one member's face, and gravity are all
defensible. So the schema does not choose. It carries a second curve, the
**rail**, and the 0° reference direction is simply the direction from the seam
to the rail.

Every conventional answer becomes a choice of where you *put* the rail:

| Convention | Rail generated along |
|---|---|
| Bisector-referenced (geometry-derived) | the dihedral bisector |
| AWS / textbook, "angle to the work surface" | one member's face normal |
| Hand-authored | wherever the operator drew it |

`reference.semantics` records which of these was used, and
`reference.member_ref` names the member when it is `member_face`. Without that
tag the intent is unrecoverable once the rail is baked, and you cannot convert
back to a WPS number.

### Correspondence

Seam point and rail point are matched by **normalized arc length**: `s` on the
seam maps to `s` on the rail.

When seam and rail have the **same segment count**, correspondence is applied
**per segment** — rail segment `i` maps to seam segment `i`, by normalized arc
length within that segment. This matters: a rail generated as a per-segment
offset is discontinuous at corners (the bisector genuinely flips there), and
per-segment matching handles that correctly while whole-curve matching would
smear it.

### Frame construction

At normalized arc length `s`:

```
P(s)   seam curve point
t(s)   unit tangent, in the travel direction (see `direction`)
Q(s)   rail point at the corresponding parameter
d      = Q(s) − P(s)
r(s)   = normalize( d − (d·t) t )        rail direction, orthogonalized
b(s)   = t × r                            right-handed {t, r, b}
```

Orthogonalizing against `t` is required — the raw `Q−P` is generally not
perpendicular to the tangent. The construction fails only when `Q−P` is parallel
to `t`, which is detectable and is a validation error.

### The angles

```
θw = work_angle_deg(s)
θt = travel_angle_deg(s)

r′ = R(t,  θw) · r          work: roll within the cross-section plane
b′ = t × r′
u  = R(b′, −θt) · r′        travel: tilt along the direction of travel
```

`u(s)` is the **torch reference direction**: it points from the weld point
toward where the torch body sits. The electrode axis is `−u(s)`.

`R(axis, angle)` is a right-handed rotation.

Order is work first, then travel. This matches how a welder physically sets up —
roll the torch, then tilt it — and it is what makes travel angle mean "tilt
along travel" regardless of the work angle already applied.

### Sign conventions, verified

**Work angle**, `θw > 0`: rotates `r` toward `+b`, i.e. toward `t × r`. `θw = 0`
means the torch sits exactly on the rail direction.

**Travel angle**, `θt > 0`: **drag / backhand**. The torch leans back over the
completed weld; the electrode points backward against travel.

Sanity check, worked: take `t = +x`, `r = +z`, `θw = 0`, `θt = +15°`.

```
b′ = t × r = x × z = −y
u  = R(−y, −15°) · z = R(+y, +15°) · z = (sin15°, 0, cos15°) ≈ (0.259, 0, 0.966)
```

`u` leans toward `+t`. The torch body sits **ahead** of the weld point, so the
electrode points **back** over completed weld. Drag. Correct.

### What the planner receives

For each `s` the planner may choose to sample:

- **position** — `P(s)`, 3 constraints
- **electrode direction** — `−u(s)`, 2 constraints
- **rotation about the electrode axis** — free, 1 DOF

Five constrained, one free. The free DOF is real and usable: it is available for
reach, collision avoidance, joint-limit avoidance, and singularity avoidance.

---

## 6. The three angles, disambiguated

Easy to conflate. They are unrelated quantities.

| | belongs to | measured | source |
|---|---|---|---|
| **dihedral angle** | the parts | between the two groove faces, through the open side | CAD |
| **work angle** | the torch | rotation within the plane perpendicular to travel | authored |
| **travel angle** | the torch | tilt out of that plane, along travel | authored |

Dihedral is not chosen by anyone — it falls out of how the parts were designed.
Its only role here is that it locates the bisector, and the bisector is the
natural 0° reference for work angle. Travel angle has no relationship to
dihedral at all.

Dihedral is a **scalar function**, not a constant. Weld a straight plate onto a
curved surface and it changes continuously along the seam.

---

## 7. Document structure

```
amr-weld-planner-v1.weld-program.v1
├── schema, program_id, label, created_utc
├── source            STEP path, sha256, unit scale, importer
├── workpiece         frame link, solids with bounds
├── weld_joints[]     objective CAD fact: where parts meet
└── seams[]           weld intent: one seam = one pass
```

`weld_joints` and `seams` are separate because they answer different questions
and change at different rates. A joint is a fact about the assembly. A seam is a
decision about what to do to it. One joint carries many passes; a seam may carry
no joint at all.

### `weld_joints[]`

Named `weld_joints`, not `joints`, because `joint` already means a robot axis
throughout this repository.

```
id                  stable identifier
type                butt | tee | lap | corner | edge | cruciform | plug
weld_symbol         ISO 2553: fillet | square_butt | v_groove | bevel | ...
members[]           solid_id, face_ids, role
contact             kind (coincident | gap | overlap), gap_mm, overlap_area_mm2
weld_line           the curve
groove_faces[]      per weld_line segment: the two faces exposed AT the line
dihedral_angle_deg  scalar function
bisector_rail       generated curve, with offset_m
accessible_sides[]  advisory
confidence          0..1
evidence            detection method and measurements
```

**`members` faces and `groove_faces` are different faces.** This is the part
that is easy to get wrong. On the dog_box lap joint:

- the **contact pair** is the plate's bottom face against the block's top face —
  horizontal on horizontal. It proves the parts are joined and classifies the
  joint as a lap.
- the **groove faces** at the weld line are the plate's *side* face and the
  block's *top* face — vertical meets horizontal, dihedral 90°.

The contact pair classifies the joint. The groove faces determine the torch
angle. Generate the bisector rail from the groove faces.

### `seams[]`

**One seam is one pass.** Root, fill and cap on the same joint are three seams
sharing a `weld_joint_ref`, each with its own offset, parameters, and direction.

```
id
weld_joint_ref      nullable — null means hand-authored
pass                index, role (root | fill | cap | single), offset
curve               inline, or {"from_weld_joint": true} to use the joint's line
span                start_s, end_s — weld part of a joint
direction           forward | reverse
reference           mode, rail, correspondence, semantics, member_ref
orientation         work_angle_deg, travel_angle_deg, spin
weld_geometry       ISO 2553 sizing: leg, throat, intermittent
process             the WPS
lead                lead_in_mm, lead_out_mm along the tangent
tolerance           position_mm, work_angle_deg, travel_angle_deg
sampling_hint       advisory only
```

`reference.mode`:

- `rail_curve` — canonical. Requires `rail`.
- `fixed_vector` — a constant direction in the workpiece frame. For simple
  planar seams.
- `rotation_minimizing` — a rotation-minimizing frame (double-reflection)
  propagated from `seed_direction_xyz`. The fallback when someone hands you a
  bare spline with no joint. **Not Frenet** — the Frenet normal is undefined on
  straight segments and flips at inflection points, so it would fail on the
  dogbone's line segments.

`spin.mode` is `free` by default (R2). `preferred` supplies a soft target that a
planner may ignore. `locked` fully determines orientation and should be used
only when tooling genuinely requires it.

`sampling_hint` carries `max_chord_deviation_mm` and segment length bounds. It
is **advisory**. The planner is not bound by it, and the schema is not permitted
to force a sampling (R3).

### `process`

Decomposed along the lines ABB's arc model and ISO 15609-1 already use, so the
output is legible to a welding engineer.

```
method              GMAW | GTAW | FCAW | SMAW | SAW | ...
iso_4063            numeric process reference, e.g. 135
travel_speed_mm_s   scalar function
arc                 wire_feed_speed_mm_s, voltage_v, current_a, mode,
                    synergic_line, polarity
wire                classification, diameter_mm, stickout_mm
gas                 mixture, flow_l_min, preflow_s, postflow_s
weave               shape, amplitude_mm, frequency_hz, travel_ratio, tilt_deg,
                    dwell_left_s, dwell_right_s, corner_policy, ramp_in/out_mm
start               ignition, run_in_s, hot_start_scale
end                 crater_fill_s, burnback_s, downslope_s
```

Every rate is a scalar function of `s`, so travel speed can slow into a corner
and wire feed can track it.

**`travel_speed_mm_s` is speed along the seam curve arc length, never tip
speed.** Under weave the tip moves 3.7x to 15x faster, over a path 2.5x to 9.7x
longer than the seam. Heat input is `V·I / travel_speed` per millimetre *of
joint*, so a planner that reads this as tip speed under-deposits by a factor of
several. Tip speed is derived and belongs to the planner; there is deliberately
no `max_tip_speed` field, because that is a robot capability rather than weld
intent.

Weave is specified as a *parameter*, not baked into the curve. Baking it would
violate R3 and R4, destroy the seam's own curvature, and inflate the planning
problem roughly 30x. The planner applies weave as a modulation about the seam
frame — displacement along `b′`, which is perpendicular to both the travel
direction and the electrode axis, exactly, for any work and travel angle.

**Weave imposes a floor on transverse mobility, not a set of waypoints.** The
full treatment — shape functions, the `μ_b′` constraint, budget reservation,
verification strategy, and the limits of kinematic simulation — is in
[`../docs/motion/WEAVE-AND-VERIFICATION.md`](../docs/motion/WEAVE-AND-VERIFICATION.md). Read its §6
before reporting any simulation result as proof of weld quality.

---

## 8. What the planner decides

Everything not in this document. Explicitly:

- sampling density and where waypoints land
- torch spin about the electrode axis
- positioner axis values, and whether to move the positioner at all
- approach, retract, and air moves between seams
- seam ordering and sequencing
- redundancy resolution and posture
- collision avoidance
- time parametrization and dynamics

The weld program deliberately does not constrain any of these. If a future field
would, it does not belong here.

Since D23 a producer may state *constraints* on the fourth and fifth of these
— seam order, a standoff direction, via clearance — in `robot.v4.program.v2`,
the pre-compile document one level down (`robot/v4/contracts/programs/`). The
solutions stay the planner's, and the weld program itself still has no such
field.

One clarification, because it is easy to get backwards: **weave is a parameter
here, but it is not the planner's free choice either.** The weld program fixes
weave shape, amplitude and frequency as weld intent; what the planner decides is
how to honour them — which is by maintaining enough transverse mobility that the
weave is executable, and by verifying the expanded trajectory afterward. The
planner may not silently drop or reshape weave to make a plan succeed. If the
weave is infeasible, that is a planning failure to report, not a parameter to
renegotiate.

---

## 9. Reference fixture

`examples/dog_box_lap_perimeter.json` — the 250 × 125 mm top plate of
`cadquery/v1/step/dog_box.step` lapped onto the base block, welded as a closed
fillet around the plate perimeter at z = 150 mm.

**Generated, not authored.** Produced by `python/weld_programmer/topology.py` +
`python/weld_programmer/detect.py`; every number in it was extracted from the STEP.

Eight segments, 821.239 mm total: four 270 degree corner reliefs at R = 25 mm
plus four lines, chained into a closed wire. The arcs stay exact degree-2
rational B-splines. Dihedral is a constant 90 degrees even though four of the
eight groove faces are cylinders -- the relief axes are parallel to the block's
top normal, so the rotating radial normal stays 90 degrees from it all the way
round. Constant dihedral, rotating bisector, which is what makes this a real
test of the rail rather than a trivial one.

The rail is a 20 mm offset that is a concentric arc on the arc segments, not a
translation, and is **intentionally discontinuous** at the corners where the
groove normal flips -- stepping by exactly `offset * |b1 - b2|`, or 20 mm at a
90 degree flip.

This file was previously a hand-authored 750 mm rectangle, inferred from
bounding boxes before the extractor existed and wrongly labelled as extracted.
Bounding boxes cannot show corner reliefs. It hid two real defects, because a
rectangle satisfies both by construction: `detect.py` emitted weld-line segments
unchained (every consecutive gap 35.36 mm, while the count and total length were
correct), and the rail's continuity was asserted rather than measured.

`tests/programmer/test_topology.py::test_top_plate_contact_face_is_a_dogbone` pins the real
numbers so the gap cannot widen unnoticed.

---

## 10. Deliberately deferred

Recorded so they are not silently forgotten.

- **Tack welds** — currently expressible as a seam with `intermittent`, but they
  are sequencing-critical and probably deserve their own type.
- **Multi-robot / multi-torch** — no field claims exclusivity today.
- **Seam tracking and touch sensing** — the SteamDeck program model has both.
  They are a runtime correction to the taught path, so they are arguably planner
  or execution concerns, but the *intent* to use them may belong here.
- **Material** — parent material and thickness affect the whole WPS, and are not
  captured. Probably belongs on `workpiece.solids[]`.
- **Distortion and sequencing intent** — "weld these in this order to control
  distortion" is weld intent, not planning. Order is now expressible as the
  node sequence of `robot.v4.program.v2` (D23), one level down; the weld
  program still has no field for it, and the *reason* for an order (distortion
  control) has no field anywhere yet.
- **Work-angle sign on a closed loop** — `θw > 0` rotates toward `t × r`, which
  is well defined pointwise, but on a closed seam the operator's intuition of
  "toward the plate" may flip relative to the sign depending on traversal
  direction. Worth a validator warning.
