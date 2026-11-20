# Prior art and design decisions

Why this module exists rather than adopting something, and the judgment calls
made while designing the contract. Recorded so they can be revisited on
evidence rather than re-argued from scratch.

---

## 1. Prior art: is there already a "G-code for welding"?

Sort of, in halves, and the halves do not meet. That gap is what this module
fills.

### The process half exists and is mature

| Standard | What it covers |
|---|---|
| **ISO 15609-1** | Welding Procedure Specification for arc welding — process, current, voltage, travel speed, gas, technique, work and travel angle, stringer vs weave |
| **ISO 2553** | Welding symbols on drawings — joint types, fillet leg vs throat, intermittent pitch and length, both-sides, field weld |
| **ISO 6947** | Welding *positions* — PA flat, PB horizontal-vertical, PC horizontal, PF vertical-up, PG vertical-down, PE overhead — defined against the seam and gravity |
| **ISO 9692** | Joint preparation — bevels, root gaps, groove geometry |
| **ISO 4063** | Numeric process references (135 = MAG solid wire, etc.) |

The `process` block borrows its decomposition and its field names from ISO
15609-1, and `weld_geometry` from ISO 2553, so the output is legible to a
welding engineer rather than being invented vocabulary.

ISO 6947 is not currently a field. It is *derivable* from the seam frame and
gravity once the positioner is chosen, which makes it a useful planner-side
validation output — "this seam will be welded in PF" — and a useful input to
positioner selection. See §3.

### The motion half exists and is dumb

- **ISO 6983 (G-code)** — real on gantry welding and cutting systems with
  M-codes for arc on/off. No concept of work or travel angle. Orientation is
  either absent or a baked tool vector.
- **ISO 14649 / ISO 10303-238 (STEP-NC)** — the right *philosophy*: features and
  operations rather than axis moves. Academic extensions to welding exist. It is
  heavyweight and effectively dead in industry.
- **AP242** carries welding PMI annotations, but as drawing metadata rather than
  an executable path description.

### Closest existing thing: vendor arc models

ABB RAPID is the nearest relative — `ArcL`/`ArcC` motion instructions plus:

- `seamdata` — purge, preflow, ignition, burnback, postflow
- `welddata` — voltage, wire feed, speed
- `weavedata` — shape, width, height, dwell

Fronius, Fanuc ArcTool and KUKA ArcTech are structurally similar. **The
parameter decomposition is good and was borrowed deliberately.** What was not
borrowed: orientation is baked into a quaternion per pose, which fixes all six
DOF and forecloses exactly the freedom this contract is built to preserve.

### Conclusion

Nothing standard binds a **parametric 3D seam curve** to **angles as functions
of arc length** with **orientation deliberately underdetermined**. The process
vocabulary is settled and was adopted; the geometry-to-intent binding is the
genuinely missing piece.

So: not reinventing a wheel, but also not inventing new names for parts that
already have them.

---

## 2. Design decisions

Format: the call, what else was considered, and why. Marked **open** where the
alternative is still defensible and evidence could flip it.

### D1 — Rail curve as the work-angle reference

**Call.** Work angle is measured against a direction supplied by a second curve
(the rail), not against a named convention.

**Considered.** A scalar work angle measured from the dihedral bisector; or from
one member's face, per the AWS/textbook definition.

**Why.** Both alternatives are *special cases of where you put the rail* —
generate it along the bisector and you get the first, along a member normal and
you get the second. The rail is the general form, so the schema does not have to
choose a convention that some workpiece will later contradict. It also survives
varying dihedral along a seam without recomputing anything, is authorable by
hand in CAD, and is trivially renderable, which matters a lot during
development.

**Cost.** The semantic label is lost once the rail is baked, so
`reference.semantics` carries it explicitly. Without that tag you cannot convert
back to a WPS work-angle number.

### D2 — Travel angle needs no second reference

**Call.** The rail plus the tangent fully determine the frame: `b = t × r`.

**Considered.** A separate "line normal" for the travel reference.

**Why.** Redundant, and actively worse — a space curve's Frenet normal is
undefined on straight segments and flips sign at inflection points. It would
fail on the dogbone's line segments. The cross product has neither problem.

### D3 — Rotation order is work, then travel — **open**

**Call.** `r′ = R(t, θw)·r`, then `u = R(b′, −θt)·r′` with `b′ = t × r′`.

**Considered.** Travel first, or both about the unrotated frame. These give
different results whenever both angles are nonzero.

**Why.** Matches how a welder physically sets up — roll the torch, then tilt it
— and it is what makes travel angle mean "tilt along travel" independent of the
work angle already applied.

**Open** because no external standard fixes the order, and a WPS that specifies
both angles does not say which is applied first.

### D4 — Positive travel angle is drag — **open**

**Call.** `θt > 0` means backhand: torch leaning back over the completed weld.

**Considered.** Push-positive.

**Why.** Drag is the common default for GMAW on steel. Verified numerically
rather than asserted: `t=+x, r=+z, θw=0, θt=+15°` gives `u=(0.259, 0, 0.966)`,
which leans toward `+t`, putting the torch body ahead and the electrode pointing
back over completed weld.

**Open.** This is a coin flip. Push-positive is equally defensible and some shops
think in it natively. Cheap to flip now, expensive after data exists.

### D5 — One curve type: rational B-spline

**Call.** Every curve is a NURBS. Degree 1 with 2 control points is a line;
degree 1 with N is a user-supplied sample list; degree 2 rational is an exact
conic; degree 3+ is free-form.

**Considered.** A tagged union of `line | arc | polyline | spline`.

**Why.** NURBS subsumes all of them with zero geometric loss — conics are exact,
not tessellated. One evaluator (de Boor), one code path, one set of bugs. And
the decisive reason: **derivatives.** Tangent needs C¹ and curvature needs C²,
and finite differences over a polyline are noisiest exactly at tight corners,
which is what caused the prior implementation's 1.52 mm corner error.

**Cost.** Arc-length reparametrization, which is needed anyway for constant
travel speed. See `SEAM-MODEL.md` §3.

### D6 — Correspondence by normalized arc length, per segment

**Call.** `s` on the seam maps to `s` on the rail; when segment counts match, per
segment.

**Considered.** A shared parameter domain (fragile — parametrization is not
preserved by CAD edits); closest-point projection (robust to mismatched lengths,
but costs a solve per sample and can jitter or become non-unique).

**Why.** Simple and stable, and a rail generated as a per-segment offset has
exactly matching arc lengths by construction. Per-segment matching is what makes
a bisector rail that flips at corners work correctly; whole-curve matching would
smear the discontinuity across the corner.

**Cost.** Breaks if someone hand-authors a rail of wildly different length from
the seam. Detectable, and worth a validator warning.

### D7 — `weld_joints[]` separate from `seams[]`

**Call.** Objective CAD fact and weld intent are separate arrays; a seam
references a joint or references none.

**Considered.** Weld definition only, with joint detection kept as an internal
artifact.

**Why.** Four things are unrecoverable otherwise: regenerating the rail under a
different work-angle convention; multi-pass welds sharing one geometry;
member and dihedral geometry for planner-side accessibility reasoning; and
provenance, so staleness is detectable when the STEP changes.

**Why it does not violate "describe the weld only".** A joint is inert
description, not instruction. Nothing in it constrains motion.

### D8 — One seam is one pass

**Call.** Root, fill and cap are three seams sharing a `weld_joint_ref`,
distinguished by `pass`.

**Considered.** A `passes[]` array inside a single seam.

**Why.** Each pass genuinely has its own offset, parameters, direction and
possibly its own span. A `passes[]` array would have to duplicate most of the
seam anyway. `curve: {from_weld_joint: true}` plus `pass.offset_in_frame_rb_m`
keeps geometry shared without nesting.

**Cost.** Pass ordering is implicit in `pass.index` rather than in array order.
Sequencing across joints has no representation at all yet — see `SEAM-MODEL.md`
§10.

### D9 — Units: metres for geometry, millimetres for process — **open**

**Call.** `_m` on geometry, `_mm` on process parameters, unit always in the
field name.

**Considered.** Metres throughout.

**Why.** Geometry feeds a planner that thinks in metres; process parameters are
read off a WPS where nobody writes 0.0012 m. The suffix convention makes each
unambiguous at the point of use, and matches the repo's existing `xyz_m` /
`positions_rad` / `tcp_speed_m_s` style.

**Open.** It is still a mix, and mixes cause bugs. Defensible to go
metres-throughout and accept unreadable WPS numbers.

### D10 — Weave frequency in Hz, not wavelength

**Call.** `frequency_hz` only.

**Considered.** `wavelength_mm`, or allowing either.

**Why.** Real controllers accept Hz. Because travel speed is a scalar function
here, fixed Hz makes weave pitch tighten automatically where travel slows, which
is generally what you want at a corner.

**Cost.** Bead appearance is governed by pitch, not frequency, so a shop
thinking in pitch has to convert — and the conversion moves when travel speed
does.

### D11 — Rejected: controller-side weave superposition

**Call.** Weave is expanded and verified at plan time. It is not delegated to
servo-rate superposition in the motion controller.

**Considered.** What ABB, Fanuc and KUKA actually do — the program carries the
seam path plus a weave struct, and the controller superimposes the oscillation
at servo rate, so the planner never sees the expanded waypoints.

**Why rejected.** Nothing would ever verify the commanded poses — no collision
check, no limit check, no manipulability check on the motion actually executed.
In a stack that already has collision *detection* but no collision *avoidance*,
adding unverified servo-level motion on top makes the safety story worse, not
better.

**Nuance worth keeping.** Once the plan request is verified at plan time,
*executing* via superposition is safe, because the plan request is what was proven.
Superposition is an execution optimisation that verification licenses — never a
substitute for it.

### D14 — The rate limit is per millimetre of arc, not per unit `s`

**Call.** `max_rate_deg_per_mm`. Convert with
`rate_per_unit_s = max_rate_deg_per_mm × seam_arc_length_mm`.

**Why.** `s` is normalized, so a limit expressed per unit `s` means rates eighty
times apart on a 10 mm seam and an 800 mm one. Worse, merging two pieces of a
split seam into one weld would silently halve what the limit meant, with nothing
on screen to say so. Millimetres of arc are the physical quantity the welder
cares about and the only one that survives a merge.

**Cost.** The validator needs the seam's own length, which means the span and
the joint both have to be in hand. Where they are not, the check reports itself
as *unchecked* rather than passing — R-UI-1 applied to validation.

### D15 — Standoff is an outer loop over one table, and its departure is priced under coverage

**Call.** The optimizer searches work and travel on a lattice, and standoff as
a handful of independent DP runs over slices of that same table. Its departure
from what the weld declares enters the objective at
`settings.optimizer.standoff_cost_deg_per_mm` (default 3), **inside J2** —
below coverage, never beside it.

**Why, computationally.** The clearance oracle keeps every ray's whole sorted
hit list, so moving the torch back only slides the interval `(d, d+L]` along
lines already cast. Seven candidate standoffs are therefore seven scans of a
list, not seven sets of rays. A third lattice axis would have multiplied the
expensive part by seven to answer a question the cheap part already answers.

**Why, physically.** Stickout is held constant along a weld. Varying it varies
the resistive preheat of the wire and therefore the current, so a profile that
ramps it is not a procedure anyone runs.

**Why priced, and why under coverage.** Electrode extension is a *process*
parameter: a longer stickout drops penetration and past roughly 20–25 mm loses
gas coverage. An optimizer that quietly added 6 mm to clear a gusset would have
changed the weld, not just the way the torch got there. But the price sits in
J2, so it can only ever break a **tie** in coverage — it can never trade weld
away to keep a procedure number. That is the property the tests pin, at a cost
of 1e6, because the tempting implementation is a weighted sum where a large
enough constant does exactly that silently.

**Cost.** One tunable constant in the objective, named in settings rather than
buried. Without it there is no way to state whether 1 mm of stickout is worth
2° of work angle, and hiding that choice inside a tier ordering would not make
it go away. The admissible band still defaults to zero width, so none of this
engages unless a procedure opens it.

### D16 — A profile is simplified by vertical deviation in degrees, not perpendicular distance

**Call.** Douglas–Peucker over `(fraction, degrees)` measures the **vertical**
gap to the chord. Not the perpendicular distance the textbook uses.

**Why.** `x` is a dimensionless fraction and `y` is degrees, so a perpendicular
distance in that plane is a length in no unit at all, and dividing it by the
chord's own length makes the effective tolerance depend on the chord's slope.

Found by running the optimizer rather than by reading it. On pipe_bracket's column
fillet the search found 85.4% of the seam and the emitted profile verified at
81.4%: the travel path `[20, 10, 10, 10, 10, 10, 10, 0, 10]` had collapsed to a
straight 20 → 10 ramp under a 2.5° tolerance, because the chord fell 10° over a
unit of `x`, its length was 10.05, and every real deviation was divided by that.
Vertically the first knot is 8.75° off the chord. The same run now verifies at
83.5%.

**Consequence.** The stated contract is checkable and is checked: no sample of
the searched path may sit further than the tolerance from the profile that gets
welded. The rate-limit argument is unaffected — a chord still has the average
slope of the steps it replaced, so it can never be steeper than a limit they all
satisfied.

### D17 — A deviation bound and an absolute band are different claims, and both apply

**Call.** `orientation.limits.<axis>` carries `min`/`max` — where the angle may
be at all — *and* `max_deviation_deg` — how far the search may move it from the
profile that was authored. The admissible set at each knot is the
**intersection**. The settings default (`optimizer.work_deviation_deg`,
`travel_deviation_deg`) is a DEVIATION, not a band.

**Why both.** They coincide only when the profile is constant and centred in
its band, which is the easy case and not the interesting one. On a ramp from
0° to 10°, "stay within ±5° of what I drew" is a window that *travels* with the
ramp; the nearest absolute band expressing it, −5°..15°, also permits 15° at
s=0, which is 15° from the intent. Expressing a deviation as a band therefore
either over-permits or requires drawing two matching curves by hand.

The converse is just as one-sided: a procedure that says "this fillet is never
welded past 6° of work angle" is a fact about the joint, not about anybody's
profile, and it must keep holding when someone redraws the profile.

**Implementation.** It falls out of the mask `orient.optimise` already takes:
`_allowed_bins` reads per-knot `min`/`max` lists, so the deviation window is
just `authored[i] ± d` intersected with the declared band before the lists are
handed over. The DP is untouched.

**Cost, and the one subtlety.** The window is never allowed to close tighter
than half a bin. A grid anchored at zero always has a multiple within half a
step of any value, so this is what guarantees the authored profile is itself
reachable — including at deviation 0, which means "do not move it" and must not
come to mean "no legal cell exists" for a weld whose angle happens to fall
between two bins.

**Measured** on pipe_bracket's column fillet, work pinned at ±2°:

| travel deviation | travel bins | rays | verified |
|---|---|---|---|
| ±40° | 17 | 70,686 | 79.0% → 100% |
| ±4° | 3 | 12,474 | 79.0% → 79.0% |

The bound confines the answer and the cost together, which is the other reason
to prefer it to a band: a tight deviation is a cheap search, where a tight band
around a varying profile is not expressible at all.

---

### D18 — Every boundary loop of a contact is its own weld joint

**Call.** A contact face's boundary is not one wire. A pipe standing on a plate
touches it through an annulus: an outer circle and a bore. Each closed loop
becomes a separate `weld_joint`, labelled `outer` or `inner` in
`evidence.contact_boundary`, and whether a bore can actually be welded is
`access.py`'s answer rather than detection's.

**Why.** `chain_edges` used to return None for anything that was not a single
wire, and `build_joint` then discarded the whole joint — weld line included. On
an exhaust manifold that was all five welds: four runner-to-flange fillets and
the runner into the collector, every one of them a real, exactly coincident
contact. The refusal was deliberate ("silently chaining across them would invent
geometry") and correct as far as it went; what was wrong was throwing the joint
away rather than emitting one joint per loop.

Deciding here that a hole cannot be welded would be this module guessing about
tooling, which is exactly what `access.py` exists to keep out of stage 2. A bore
wide enough to weld inside is a weld; one that is not comes back fully blocked
and produces no candidate seam. Measured on the manifold: the five bores are
55–62% reachable and do become candidates, and it is `clearance.py` — the
tooling-dependent half — that should decide whether a Ø16 nozzle fits a Ø32 bore.

**Consequence.** A pair can yield more than one joint, so ids are assigned per
emitted joint rather than per pair, and `POST /api/steps/{sha}/joints/propose`
returns `weld_joints` plural. The `inner`/`outer` label gates nothing; it exists
so a list of two joints per pipe is readable.

---

### D19 — Accessibility is answered against an indexed tessellation, and the exact intersector is kept runnable

**Call.** `access.py` casts its escape rays at a BVH over triangles
(`occlusion.py`), tessellated for the purpose at a stated deflection and dilated
in plane. The exact-surface path stays selectable as `--engine occt`.

**Why.** The exact path had no spatial index: every ray was intersected against
every surface of the shape, 130 µs each. One pipe bore took 43.6 s; the manifold
took about six minutes. Indexed triangles answer the same question at ~1.5 µs
per ray — 3.7 s for that whole manifold, 25 ms for the bore.

The cost was never the ray count. 142k rays is nothing; a BVH does them in
milliseconds on one core, which is why this needed no GPU despite one being
available. The BVH is the win and a GPU would only multiply it.

Approximation is admissible here because the question is deliberately liberal —
a point is blocked only when *nothing* escapes — but it is not asserted, it is
tested. `tests/programmer/test_occlusion.py` runs the whole stage through both intersectors
on `pipe_bracket.step` and demands the blocked spans match; they agree to four
decimals of a millimetre on every joint, and `min_open_fraction` is identical.

**Two knobs, doing different jobs.** `deflection_mm` buys accuracy and must stay
under `SURFACE_SKIP`; `dilation_mm` buys conservatism, and measurably more than
that — undilated the mesh finds 57.4062 mm blocked where the exact surfaces find
59.7500 mm, and 0.05 mm of margin recovers the exact number. The margin is not a
safety fudge on top of the right answer; it is what reaches it.

**Consequence.** A reachability verdict is now a function of a tessellation, so
`accessibility.intersector` carries the engine and both tolerances. `method`
stays `hemisphere_escape`: the algorithm did not change, only the geometry it
tests against.

---

### D20 — A rail that cannot be offset exactly is approximated and its error stated, not dropped

**Call.** Where the groove face is neither a plane (constant normal) nor a
cylinder whose axis is parallel to the base normal (closed-form concentric arc),
the bisector rail is built by offsetting control points along a varying
bisector, refined by knot insertion until the *measured* deviation is inside
`RAIL_TOLERANCE_DEG`, and the achieved error is reported on the rail.

**Why.** The two exact paths were the only ones, and everything else returned
None — which discarded the entire joint. A cone, torus, sphere, B-spline patch,
or the `extrusion` surface OCCT reports for an elliptical tube produced no weld
at all. There is no exact alternative to reach for: the offset of an ellipse is
not an ellipse, and generally not a rational curve of the same degree.

**The tolerance is an angle.** The rail is the work-angle reference, so an error
`e` at offset `R` matters as `atan(e/R)`; a bare millimetre budget would mean
different things at different offsets. Same argument as D16.

**Refinement is not optional, and the measurement is what proved it.** Pole
offsetting alone put a full ellipse 5.97 mm off a 20 mm offset — a 17° work-angle
error. Uniform refinement roughly halves the error per round and reaches 0.1° at
seven rounds and 388 poles. That pole count is the price of refining every span
equally; splitting only where the bisector turns would be much cheaper and is a
worthwhile optimisation rather than a correctness fix.

**Consequence.** `bisector_rail` carries `approximate`, `max_deviation_mm` and
`within_tolerance`, and they live on the rail rather than its segments because
segments are the shared curve definition and a weld line must not grow
rail-only fields. An exact rail carries none of them. `topology.py` records each
curved face's normal at points along every edge it bounds, because `detect.py`
has no geometry kernel and cannot ask for one itself.

### D21 — The Python is a package, and the hand-off plan request is a package of its own

**Decision.** `python/` holds two packages, `weld_programmer` and `weldplan`, and
is the single import root — put on `PYTHONPATH` by pixi's `[activation.env]`,
so `weld_programmer.seam.detect` resolves identically under `pixi run serve`, under
`python -m weld_programmer.seam.detect`, and under pytest. Data that was three sibling
directories (`fixtures/`, `settings/`, `presets/`) is now under `data/`. The
module's prose docs are under `docs/`. Every filesystem anchor, internal and
external, is in `weld_programmer/paths.py`.

**Why the flat directory could not stay.** It worked only because a script's own
directory lands at `sys.path[0]`, which made two things true at once: `import
settings` in this module and `import settings` in any dependency meant the same
name, and the test suite needed `sys.path.insert` in `conftest.py` to reproduce
what production got for free. Both were documented as hazards in the source
rather than fixed — `tooling.py` is named `tooling` and not `torch` for exactly
this reason, and says so. Packaging retires the whole class; the rename is no
longer needed and was not done.

**Why `weldplan` is separate from `weld_programmer`.** The `.weldplan` plan request is
co-owned: one side writes it, the other reads it, and the reader is a different
program — a motion planner, in the shape of cuRobo, not yet started. A reader
that had to `import weld_programmer` to open one would take on OCCT, numba and
cadquery to parse a zip of JSON. So `weldplan` imports the standard library and
nothing else, and nothing in it may import `weld_programmer`. The arrow points one
way, and `tests/shared/test_plan_request.py` fails if it ever reverses. This is the
rule GOAL.md already states for `cadquery/v1` and `tesseract/v1` — read the
data, do not import the module — applied to a boundary inside this module.

`placement_problems` moved across that line and `weld_programmer.robot.placement`
re-exports it, which is the test of where the line falls: what makes a placement
record well-formed is CONTRACT, because both sides must agree on it, so it
belongs with the plan request. Seam frames and span arithmetic are not, and stayed.

**Why one `paths.py` rather than a `parents[N]` per caller.** There were twenty
of them, including six identical `repo_root()` helpers counting three levels up
to reach `cadquery/v1` and `robot_description/`. Depth counting is not merely fragile, it
fails INVISIBLY: `parents[3]` from a file one level deeper resolves to a real
directory that does not hold what was asked for, and every consumer here treats
absence as normal — `sample_steps` filters on `exists()`, placement offers no
cell and says so. A wrong path did not crash, it edited what the operator was
shown. The move proved the point: the one fixture path missed by the sweep
turned two passing tests into skips, and only a skip count caught it. The three
external anchors take environment overrides (`WELD_REPO_ROOT`, `WELD_CELL_ROOT`,
`WELD_DOG_BOX`); the internal ones do not, because they move only when
`paths.py` itself moves.

**What was deliberately not done.** `server.py` is still 1,900 lines and ~35
routes: splitting it is a refactor with its own risks and no relation to this
one. No motion-planning code was written — the point of this change was to make
the seam explicit, not to fill it. See `docs/motion/MOTION.md`.

### D23 — The pre-compile program is a third document one level down, and it vendors the seam vocabulary

**Call.** `robot.v4.program.v2` (`robot/v4/contracts/programs/`) is the
weld program both authoring flows emit and any planner lowers from: an
ordered list of nodes — `weld`, `approach`, `transit`, `retract`, `home`,
`dwell`, `io` — one spreadsheet row per node. **The weld program of this
module is untouched.** GOAL.md non-negotiable 1 (the program describes the
weld, never the motion) still governs `amr-weld-planner-v1.weld-program.v1`;
program.v2 is the document one level down where sequence and freespace
*constraints* live, and it is still not a solution: no joint values, no
sampled trajectories, no solved paths, enforced by a closed schema and a deny
list of solved-quantity keys checked recursively in Go, TypeScript and Python.

The motion nodes, read in order, must form a single line that alternates weld
and freespace — `home? approach weld (transit weld)* retract home?`. The
planner never inserts a freespace move; a gap the producer did not state is a
validation error. Event nodes sit on the boundaries of that line, execute at
rest, and are transparent to the check. A weld node's geometry is one of two
kinds: `seam`, this module's seam with the curve inline, 5-DOF with the angle
profiles and their bands; or `posed_polyline`, the classic flow's
chord-decimated control points each carrying the TCP orientation as
`torchOrientationRpy` baked it and as the operator may have teach-modified
it. That RPY is authority and is never re-derived. The seam's `process` block
does not travel whole: `travel_speed_mm_s` and `weave` sit on the weld node
because the planner times the weld from one and reserves mobility for the
other, and everything the power source needs goes into `weld_parameters`,
which is reserved and **must be empty** until it is designed. Cell and tool
are named by id and pinned by sha256, never inlined; placement is inline.
`planner_owns[]` carries over from the plan request (D21): the absences are
decisions and are stated.

**Considered.** Extending this module's weld program with a `sequence`
section. `$ref`-ing the seam `$defs` from the new schema instead of copying
them. Inlining the cell and tool as the plan request does. Letting the planner
fill unstated gaps. Re-expressing classic welds through the seam model's
angle vocabulary. Keeping the whole `process` block on the weld node.

**Why.** The weld program is the wrong home because non-negotiable 1 is the
whole reason it can be handed to any planner; a `sequence` inside it would be
the first motion field and the last time that sentence was true. A `$ref`
across documents is a URL that two of the three consumers cannot resolve:
`robot/v4/go.mod` has no JSON Schema library and neither does the OLP UI, and
`robot/v4/contracts/programs` is read by path from four languages and must
stay self-contained. The copy is policed the way `export/robot_v4.py` polices
its copy of OLP's field lists — by a test that goes red on drift. Cell and
tool stay out because program.v2 must be a spreadsheet, and a descriptor is
a large opaque cell; the hash lets a planner refuse a mismatch, which is what
inlining bought. Unstated gaps are refused because the alternation check IS
the validity test: the document says every move the arm makes, and the
planner decides only how. Classic RPY is kept verbatim because the baked
orientation, teach edits included, is what the operator authored, and the
two angle conventions (`weld.ts` world-up vs §5 of SEAM-MODEL.md, D3/D4
still open) never have to be reconciled if neither is lowered into the
other. `process` is split because a half-designed WPS block is worse than an
empty one: consumers start depending on the half.

**Consequence.** Rails and classic remain mutually exclusive per program
(`offline-programming/v1/ui/src/domain/project.ts`), unchanged; program.v2 is
produced by whichever one flow a program uses. A `posed_polyline` weld cannot
be handed to this module's motion server as it stands — the seam search wants
a 5-DOF band, not a 6-DOF pose list — so both planners keep their roles: rails
lowers to a `.weldplan`, classic to program.v1 rows for Tesseract. Neither
lowering is written yet; this decision ships the schema, fixtures, the three
validators and their parity tests only. Run-in/run-out (issue 0013) lands in
`weld_parameters` when designed. Taught waypoints have no node and are
recorded as owed. `tests/shared/test_program_v2.py` is the guard: both layers
on every fixture, `$defs` parity against this module's schema, and the three
deny lists read out of each other's sources.

**Cost.** Twelve `$defs` exist twice and a change to the seam vocabulary here
now fails a test in another module's directory, which is the point. Three
hand-written validators restate one schema, and a rule added to one and not
the others is caught only if a fixture exercises it — so every rule gets a
fixture or an in-test mutation in all three.

---

### D24 — The authoring server is deleted: its computation becomes `seam_worker`, its state moves to Offline Programming

**Call.** `seam_worker/server/` (:8795) is deleted, and with it every route
that held something: `workspace/{steps,projects,programs,presets}`,
`projects/{bundle,open,reattach}`, `settings`, and the `artifacts/<sha>/` store
behind `jobs/analyze`. The OLP project owns all of it. `export/olp`,
`export/olp/replan` and `export/olp/rows` go too — they pushed programs into
OLP's catalog from the era when the weld planner was upstream, and the arrow
now runs the other way.

Everything a computation needs travels **in its request**. STEP bytes, the cell
URDF and spheres, presets and settings are enclosed; nothing is resolved by sha
against a workspace or by path against `CELL_ROOT`. The worker keeps no cache.

`weld_programmer` is renamed **`seam_worker`** and becomes exactly that: a
bounded worker driven over stdin/stdout JSON. `workers.py` is already spawn-safe
and HTTP-free — "Module level, no closures, no app" — and becomes the entry
point. `engine.py` goes with the server rather than keeping its pool: a
`ProcessPoolExecutor` so an event loop is not blocked, a `Job` event log for a
client to replay, and `recycle_pool` because "a running OCCT call cannot be
interrupted cooperatively" are three answers to questions only a long-lived
HTTP server asks. One process per request answers all three, with concurrency
and cancellation owned by the caller -- which is what `internal/cad` already
does. The OLP gets `offline-programming/v1/internal/seam`, mirroring it.

Every decision above this one was written while the package was called
`weld_programmer` and says so; they are history and are left as they were.

`amr-weld-planner-v1.weld-program.v1` and `SEAM-MODEL.md` **retire**.
`robot.v4.program.v2` becomes the single seam definition, and D23's parity arrow
reverses: program.v2 is the original, this module's schema the copy until it is
deleted. program.v2 gains `weld_joint` and `groove_face` in `$defs` and
`workpiece.weld_joints[]`.

`export/` is deleted whole — the two alias modules go, their callers point at
`weldplan` directly, and `export/robot_v4.py` goes with them, so **rails stops
emitting `robot.v4.program.v1` rows at all**. `program.py` is deleted too,
not moved: `robot.v4.program.v2` becomes the motion planner's input format
directly, so **the seam a node carries is authority** and nothing re-resolves it
on the way to the planner. `write_request` stays as a worker op and packs the
program.v2 document the OLP hands it.

**Considered.** Keeping `artifacts/<sha>/` as a content-addressed cache on the
worker side. Keeping `CELL_ROOT` reads, since cells are repository data rather
than server state. Folding the worker into `cadquery/v1`, or lifting it out as a
new `weld-geometry/v1`. Naming it `geom_worker`. Moving `export/robot_v4.py` to
`weldplan` alongside `program.py`, and keeping it in `seam_worker` as a
transitional resident until the executors move. Allowing numpy in `weldplan`.

**Why.** A cache keyed by content hash is not state in the sense that matters,
but it is state in the sense that fails: a program whose geometry lives in
another application's workspace cannot be compiled once that workspace is
cleared, and a rails program is exactly that today. The OLP already holds the
STEP (`RailsSource.bytes`), so enclosing costs a re-upload against a cold worker
and buys a program that is complete on its own.

`CELL_ROOT` is read as data and imported by nothing, so it was never a boundary
violation — but a cell resolved by id on the worker side is the failure the plan
request already refuses: "a request naming a cell by id alone would resolve
differently on another machine, and the placement is measured from that cell's
work frame". Enclosing makes the worker's answer a function of its request and
nothing else.

`cadquery/v1` is the URDF pipeline, with its own UI and deploy commands; moving
there would put the seam vocabulary further from program.v2 and improve nothing
about the server. A new top-level component leaves `weld_planner/v1` a shell.
Once the seam schema retires into program.v2 the placement question dissolves
altogether, so the worker stays where it is and is renamed instead.
`geom_worker` was rejected because it does not distinguish this worker from
`internal/cad`, which is also CAD and also geometry; `seam` names what this one
produces and the other one cannot.

`weldplan` earns its keep by having no dependencies — anything can read a
`.weldplan` or validate a program.v2 without OCCT, CUDA or numpy.

`build_program` is deleted rather than moved because program.v2 removes its job.
Its work is the three-way resolution of a sparse weld list — candidate seam
defaults, then preset, then the operator's overrides, of which only the
overrides are stored. A program.v2 node carries the resolved seam already,
inlined the moment the operator accepted or trimmed it, so resolving again on
the way to the planner would be re-deriving what the document already states.
`program.py` argues that "the client never assembles this document" because two
assemblers drift; deleting one of the two is that argument's other solution, and
the one available once the client's copy is the document rather than a preview
of it.

`export/robot_v4.py` is deleted rather than moved or kept. It runs FK
(`kinematics.link_transforms`, `kinematics.rpy_from_matrix`) over the planner's
joint trajectory to rebuild TCP poses the planner already had, into a document
the rails execution path does not read: the daemon fences on
`sha256:<request sha>` (`weld_motion_planner/io/dense_output.py:86`), not on a
row digest. Keeping it also means keeping a Python reimplementation of OLP's
`computeRowDigest` and `computeProgramDigest` — down to `JSON.stringify`
printing the float 5.0 as `5` and 1e-07 as `1e-7` — policed by a cross-language
harness. That is exactly the drift `program.py` refuses ("a contract assembled
on the client and validated on the server drifts"), running backwards.

program.v2 already vendors twelve `$defs` and D23 took that cost knowingly.
Retiring the original removes the duplication rather than doubling it, and the
parity test survives with its arrow reversed. `weld_joints` belongs under
`workpiece` because its own description says what it is — "Objective CAD fact:
where parts meet" — and `workpiece` is where objective facts about the part
already live. Dropping it was an oversight of the vendoring, not a decision:
program.v2 kept `seam.weld_joint_ref` and kept `curve` optional, and
`seam.curve` is optional precisely because it may be "a reference to the joint's
`weld_line`". Without `weld_joints` that reference is dangling, and the planner
reads it in seven places.

**Consequence.** The weld planner's own UI (`weld_planner/v1/ui`, served at `/`
by the deleted server) has no state to run against. It is removed or demoted to
a dev tool, and the OLP becomes the only authoring surface.

`.weldplan` will carry program.v2 rather than weld-program.v1. That changes the
planner's input representation, which the Weld Preflight handoff explicitly does
not ask for — so the reference run against `data/motion/bracket_b_2x.weldplan`
is taken BEFORE this lands, or the report mixes a v0.2.0-era adapter with a new
representation.

A rails program no longer has a `robot.v4.program.v1` form. It cannot be saved
to the Motion Server catalog, cannot reach the SteamDeck pendant, and cannot be
played by `previewTrajectory(rows)` — so **issue 0017 stops being optional and
becomes part of this cutover**: the OLP previews the planner's certified Hermite
`(q, qdot, t)` directly and takes TCP poses from the viewer's own FK, which is
the better source anyway. Rails then carries program.v2 as its program and the
native result as its plan, and holds no third document. `stampRowNodeIds` and
`fromRobotV4Program` go dead on the rails path; classic keeps both, and
`rows-from-nodes.ts` is untouched. Executor migration to program.v2 is what
restores catalog and pendant reach for rails; until it lands, rails is an
OLP-only flow. With `export/` gone the name `seam_worker` is exact rather than
aspirational.

Two things are deliberately left open rather than decided here. `frame_id` is an
unconstrained string and the OLP producer writes `"world"` into it
(`from-project.ts:141`, `:248`, `:295`) while the schema's only mention of world
is "no world pose is committed to". And classic geometry stays in whatever frame
it is in. Both are real; neither blocks this rework.

**Cost.** Every `optimise` and `clearance` click carries the STEP.
`internal/cad` already runs the classic worker that way — 8 MB STEP, 12 MB
request, 64 MB output, concurrency 1, a 24-entry / 96 MB in-process LRU — so the
shape is proven, but analyze is heavier than an edge sample and a cold worker
re-runs topology extraction, joint detection and rail fitting. The results live
in the OLP; the worker holds nothing.

`weld_joint_ref` is conditional, and with `workpiece.weld_joints[]` present the
condition is expressible inside one document: a seam either carries its own
`curve` or names a `weld_joint_ref` that resolves to an entry of
`workpiece.weld_joints[]` carrying a `weld_line`. It is a cross-reference, so it
belongs in the semantic validators — `program_v2.problems` and its Go and
TypeScript twins — rather than in the schema.

Vendoring `weld_joint` collided once with program.v2's own rule that every
object is closed. v1 leaves `weld_joint.evidence` as
`additionalProperties: true`, and the copy was closed to match the rule --
until the fixture was derived from `dog_box_lap_perimeter.json` and the real
evidence turned out to carry seven keys, five of them unnamed by the schema
(`normal_dot`, `overlap_relation`, `branch_extent_along_normal_m`,
`branch_extent_across_m`, `thickness_ratio`). They are method-specific: a
`coincident_face_pair` joint states different reasons than a boundary-length
match, so closing the object means naming every detection method's vocabulary
in advance. `evidence` stays open, and `test_every_object_is_closed` names it
as the one exception. What closing it was for -- keeping solved quantities out
-- `_forbidden_problems` already provides, because it walks the whole document
from `$` and does not stop at open objects. `workpiece` remains the only
deliberate difference between the two schemas.

Deleting `export/` takes `tests/programmer/test_export_robot_v4.py` and
`tools/olp_validate.mjs` with it, and halves
`tests/programmer/test_native_motion_admission.py`: its `["rows", "dense"]`
parametrization loses the rows arm, and `_mini_fixture()` — which both arms
build from — has to move out of `robot_v4` before it goes. The native-motion
admission contract itself (`weldplan/native_admission.py`) is unaffected; dense
still enforces it.

Deleting `build_program` costs the property `resolve_weld_list` was built for:
"a candidate whose defaults improve — a better bisector rail, a refined span —
improves every weld that did not override that field". With the seam resolved at
accept time, an improved default reaches an existing weld only when the operator
re-accepts it. That is a real loss, taken deliberately: a program that cannot be
compiled without the server's current idea of its own geometry is the larger
problem, and it is the one this decision exists to end.

---

### D25 — The program states every move, and a standoff is a position-only rest the free-space solver plans between

**Call.** M6 stops synthesizing the motion between welds. `robot.v4.program.v2`
states every move the arm makes as a node, a gap the producer did not state is
a validation error, and the planner plans the moves it is handed rather than
the moves it infers. `depart_standoff` and `arrive_standoff` stop being a
planner-side default and become input to the free-space solver.

A standoff is a POINT, derived: the weld end's TCP position plus `direction`
times `distance_mm`, in the workpiece frame. The free-space move runs standoff
to standoff. The straight lift between a weld end and its standoff belongs to
the standoff, not to the free-space move.

At each standoff the solver holds `PIN_CONTROL` (3) coincident control points,
so the motion comes to rest there. That pin constrains **position only** --
`FK(q).xyz` equals the standoff point, orientation free. The three control
points share one configuration, because that is what makes it a rest; which
configuration is the solver's to choose, with the wrist unconstrained. The
weld-touching end keeps its existing hold, where the whole configuration is
fixed.

**Considered.** Leaving M6 to derive the moves and reading the freespace nodes
as advisory. Pinning a standoff in joint space, the way a weld end is pinned.
Constraining the standoff's orientation to the weld end's, or to a lift along
the torch axis.

**Why.** "The document says every move the arm makes, and the planner decides
only HOW" is D23's rule, and the alternation check IS the validity test; a
planner that also inserts moves makes that check decorative, and the operator
loses the thing program.v2 was for -- being able to say what happens between
two welds.

Three control points rather than two, because `control_solver.py` already says
what the count buys: "For a clamped cubic, `P0 = P1` makes velocity zero at the
end and `P0 = P1 = P2` makes acceleration zero too, so three is exactly 'arrive
stopped, and stop smoothly'." `freespace.PIN_KNOTS` is the same 3 for the same
reason, measured: two makes the verifier's first difference zero, three makes
the second difference zero as well, so the acceleration at the handover is zero
rather than an artefact of where a ramp stopped.

Position only, because a standoff is a place to be and not a pose to hold. The
weld end's orientation is DATA -- it is the authored torch angle, and F4's
admissible band is the only latitude anyone granted there. Nothing authored the
standoff's orientation. Fixing it would spend the wrist redundancy exactly
where the free-space move needs it most, at the point furthest from the work,
to satisfy a constraint no one stated.

**Consequence.** A free-space motion gains a pinned band at each end where
today only a weld-touching end has one, and the two pins are different in kind:
a weld end copies a configuration, a standoff solves a 3-DOF position IK with
orientation free. `Motion.pin_start` and `Motion.pin_end` already exist, so
what changes is what a pinned end is constrained to be, not whether an end can
be pinned.

Issue 0013 is untouched: the weld pass starting and ending at rest is about the
weld's own ends and its own clock.

**Cost.** The corpus `.weldplan` fixtures have no freespace nodes at all -- v1
carried seams and nothing between them -- so they cannot be migrated to
program.v2 without synthesizing the very moves this decision says a planner may
not synthesize. Whoever regenerates them states the standoffs, and that is
authoring work, not a conversion script.

### D26 — A rails program is simulated from the trajectory it will run, and the Motion Server leaves that path

**Call.** Plan and Simulate become two buttons in Offline Programming's rails
mode. **Plan** calls the weld planner and keeps the dense joint trajectory;
**Simulate** replays those bytes in the viewer. The Motion Server's
spreadsheet/Tesseract planner is removed from the rails path entirely, and the
Tesseract discrete contact check with it. The classic flow is untouched: it
still plans in the browser, is accepted or rejected by the Motion Server, and
replays what came back.

The split is dispatched on `project.railsAuthoring`, the existing New-program
mode switch, not on a new session flag.

**Considered.** Keeping the Tesseract step for rails as an independent
collision opinion, either on the compiled rows or on the dense blob fed in as
`joint_ptp` rows so no IK would re-run.

**Why.** The rails path was planning the program twice, and the second answer
was the weaker one. `plan_motion_server_tesseract_spreadsheet_path` re-solves:
only a `joint_ptp` row keeps its supplied joints (and derives its TCP by FK
instead); a `line_tcp` row -- which is every row rails ever compiled -- has its
J1..J6 discarded and IK re-run from the interpolated TCP pose, seeded from the
previous solved sample rather than from the row. So the weld planner's
certified motion was flattened to poses, re-solved, and the RESULT of that was
what got watched. The rows that made this possible went with
`export/robot_v4.py`; a rails program has none, so the step could no longer run
at all.

Nothing is lost by dropping the contact check. It samples waypoints, and every
segment in the blob has already passed the T2 verifier -- `weld_plan.go` sends
`verify=true` and `motion_planner_server.py` gives it to both the weld and the
connecting trajopt stages. The verifier proves clearance across whole intervals
with a Lipschitz bound, on the real triangles at fp64 with an independent
forward kinematics, and its own docstring measured why the weaker check is not
worth adding back: 1.2-2.4 mm between consecutive samples on real plans, enough
to pass clean through a 3 mm clamp and report two clear endpoints.

**Consequence.** Simulate is now the only preview a rails program has, and it
shows what the daemon will ingest -- there is no derived second document to
disagree with the first. A trajectory is kept beside `sourceProgramDigest` and
`createdAt`, and the plan panel reports both plus whether the digest is still
the open program's. It is reported rather than hidden: an edited program's last
plan is still worth watching, and the failure to prevent is believing it is the
current one. A project revision cannot answer that question, because it moves
for edits that change no motion and stands still for some that do.

**Cost.** A rails program has no independent collision opinion any more -- the
planner's verifier is the only one, and it shares no code with the thing it
judges but does share an author. Bringing one back means feeding the dense blob
to `run_spreadsheet_plan_collision_check` directly, which is cheap to do (joints
only, so no IK would re-run) and was deliberately not done: adding a sampling
check under an interval proof reports agreement, not evidence.

### D27 — A placement is measured from the middle of the table's top face, on both sides

**Call.** The rosie_1400_v3 work frame is `positioner_table_a_top`: a fixed
child of `positioner_table_a` at `[0, -0.622, +0.100]`, the middle of the bar's
top face. `seam_worker/robot/cell.py` adds it to the cell descriptor
(`WORK_SURFACES`) and names it as the work frame. `offset_xyz_m` now means what
the OLP has always authored — offset from the table-top centre — and the OLP
sends it unconverted.

**Considered.** Converting in the OLP instead (`toProgramV2Document` adding
`[0, -0.622, +0.100]`), which is what `weld_programmer/server/export_olp.py`
did through `placement_from_world` until D24 deleted it. And carrying the
offset beside the frame name rather than as a link.

**Why.** D24 removed the only conversion between the two frames and nothing
replaced it, so every OLP placement reached the planner 622 mm along J7's axis
and 100 mm down: inside the bar. The environment field then refused every pose
(4096 of 4096 at −21.5 mm on the operator's exported bracket_b_2x; 0 of 4096
once corrected), and Plan refused seams that had been accepted and run on the
real motors. The owner's call was to move the planner to the OLP rather than
the reverse: the table top is what an operator measures from, and the J7 axis
is a fact about one Fusion export. A link rather than an offset beside a name,
because FK, the verifier and the positioner sweep all walk joints and so carry
it without a line changed.

**Consequence.** `scene.cell_obstacles` rides the body the work frame is bolted
to (`_carrier`), not the work link itself: the table is the marker's PARENT, and
riding the marker dropped the table's mesh from every screen. The verifier now
checks the part against `h_frame`, which it skipped while both were adjacent to
`positioner_table_a`. A `.weldplan` packed before this carries the old
descriptor and still reads its offsets from the axis; re-plan, which re-packs.
The motion corpus placements are converted and two of three parts turn out to
have been centred on the table.

**Cost.** The top face is measured from `meshes/positioner_table_a.dae`, not
stated by the URDF. A re-exported table mesh with a different bar moves the
surface and not this number.

### D28 — A hand-placed weld is kept as its poses and lowered to a seam before it is planned

**Call.** program.v2 gains a third weld geometry, `manual_pose_wp`: the torch
poses the OLP's Waypoint welds tool places (work point, URDF roll/pitch/yaw,
and C0/C1/C2 per pose), in the workpiece frame, one weld node per weld. The
schema pins what the poses mean -- chord-length cubic Hermite segments whose
tangent is the one-sided chord at C0, the unit-chord bisector at C1 and the
cubic-spline unknown at C2 and the ends; shortest-arc slerp by each segment's
arc-length fraction; every scalar function over normalized arc length of that
path -- so the weld stays editable in the form it was authored. The per-pose
standoff is `standoff_mm` with a knot at each pose. The OLP lowers each one to
a `seam` weld just before packing a plan request (`program-v2/manual-pose-seam.ts`):
the path as one segment per gap -- a degree-1 line where the gap is straight,
the Hermite's cubic Bezier where it is not -- a fixed-vector reference chosen
furthest from the tangent, and work/travel profiles sampled every 2 mm. `write_request`
refuses a `manual_pose_wp` that reached it unlowered.

**Considered.** A new `op` (every line rule, editor and validator keys on
`weld`, and a hand-placed weld is still arc-on to arc-off with speed, standoff
and weave). Carrying the resolved curve beside the poses, so no consumer has to
reproduce the rule; the owner chose the rule alone, because the waypoint
program has to stay editable and a second geometry would be a second
authority. Naming it `waypoints`, which every validator's deny list already
refuses as a solved quantity. Allowing it in classic programs, whose documents
are in the world frame the planner refuses; classic is being retired.

**Why.** The planner already plans seams, and a seam can say everything a
hand-placed weld needs except spin. Lowering in the producer keeps the planner
unchanged and keeps the one-way conversion out of the document.

**Consequence.** A straight weld plans as the seam it came from. bracket_b_2x
`seam_0003` rebuilt as two C0 poses from the planner's own frames and angles
(2026-09-17): 4.43% of poses feasible for both, tracking PASS with a bound of
0.462 mm against the seam's 0.465, 88.02 s of dense trajectory against 87.75.
Emitted as a cubic, the same weld came back `tracking: REFUSED`: the
certificate was exact on degree-1 lines and refused any higher degree
(issue 0008). 0008 was closed the same day with a curvature bound, so a
curved gap is now certified or failed on its merits: the same seam bowed
3 mm off its line at a C2 midpoint (two cubics) planned with tracking PASS,
a bound of 0.500 mm against a witness of 0.0003 mm, no collisions; a straight gap is still
emitted as a line, because the straight bound is exact and the curved one
is not.

Spin about the tool axis is authored and stored, and dropped at plan time: the seam search holds a weld to 5 DOF and never reads
`orientation.spin`. The lowered angles are exact at every knot against
SEAM-MODEL section 5 and within 0.078° of the slerp between knots for a 90°
skew turn over 20 mm. Consecutive coincident poses are refused by all three
validators (chord at most 1 micrometre) and by the tool before that; a new
pose starts as a copy of the one before it and has to be moved.

**Lines, arcs and chains (same day).** The tool makes three kinds of weld:
waypoints, a line (two poses, the same rule) and an arc (`shape: "arc"`, the
circle through three poses, lowered as rational quadratic pieces of at most
90 degrees -- exact, and certified by 0008's rational bound: bracket_b_2x
`seam_0003` redrawn as a 130 mm arc bowed 3 mm planned with tracking PASS,
0.4998 mm bound, 0.0001 mm witness). A weld may start at another's end:
`extensions.manual_pose_wp.start_from`, followed by the OLP after every edit
so a chain moves together. Position only -- the torch at a joint often turns
-- and a link that would close a loop, or whose source is deleted, is
dropped. The planner never reads the link; it sees the resolved poses.

**Cost.** The rule lives in one TS module and in prose; a second producer or a
planner that reads `manual_pose_wp` directly has to reimplement it, and nothing
yet pins a second implementation against the first.

**The robot's assets moved out (2026-09-18).** `data/motion/cells/<cell>.profile.json`,
`.tool.json` and `.spheres.json` are gone. They were a second description of a
robot, kept a directory away from the first -- and the Robot Description
Contract (`docs/ROBOT-DESCRIPTION-CONTRACT.md`, proposed 2026-09-13 after
the nine-motor bench crash-looped on a URDF revision meeting an old pair record) makes one
directory one robot: `robot_description/<model>/` holds `robot.urdf`, `meshes/`,
`spheres.json` and `config.json`, identified by that directory's git tree hash,
and the joint daemon refuses a plan made against any other.

What this package kept is a READER (`robot_cell.robot.description`) and a
MERGE: the cell profile is now computed per plan from the model's `config.json`
and the target cell's `machine_planning_calibration.json`, never stored. The numbers did not change --
`tests/motion_planner/test_description.py` holds the merge to the deleted
files' values to the digit -- but their ownership did. The planning rate (what
the planner parametrizes against, at most the URDF's rating) belongs to the
model; the enforced rate (the ceiling the dense header declares, what the cell
clamps at) belongs to the cell, and falls back to the planning rate when a cell
states none. `spheres.json` carries the sha256 of the URDF and the tree hash of
the meshes it was fitted from, so a refit merged without its meshes -- or meshes
replaced without a refit -- fails a test instead of a weld.

**Cost.** Two of the three sphere provenances are prose from before the hashes
existed, and re-stamping after an unrelated URDF edit is a manual
`tools/fit_spheres.py --restamp`. The OLP does not yet send `config.json` with
a plan request, so the planner's correction caps are zero until it does and a
real jig calibration would be refused on this side.
