# The motion planner: implementation plan

The architecture is `ARCHITECTURE.md`; this is the order it gets built in and
what "done" means at each step. Three rules shape the ordering:

1. **The judge is built before the contestants.** The verifier (M3) exists
   before the graph or the optimizer, so every stage that follows is measured
   by the verifier it must eventually pass, not by its own opinion of itself.
2. **Every milestone ends runnable.** Each one lands a pixi task and tests
   against the two fixture plan requests; there is no milestone whose output is
   only library code.
3. **Oracles are exported, never imported.** `weld_motion_planner` may not import
   `seam_worker`, and its tests may not either — parity against the authoring
   side (curves, frames, FK) is checked against oracle JSON generated once by
   the authoring environment and committed as fixtures, the same pattern
   `test_frame_parity.py` uses for the TypeScript client.

The planner is a file-in, file-out program from day one — `pixi run motion-plan
request.weldplan` writing a response artifact — because the contract is a file
and the CLI is the contract's first consumer. Server and UI integration come
last, and are thin.

## M0 — Bootstrap

`python/weld_motion_planner/` package; its own pixi environment (torch + CUDA, warp,
an OCCT tessellator, scipy; CCD backend added in M3); `pixi run motion-test`.
Plan request loading via `weldplan.read_request`. Fixture `.weldplan` plan requests
for `dog_box` and `pipe_bracket` generated through the authoring server once
and committed (they are deterministic bytes, so they diff).

Done when: the package round-trips both fixture plan requests and the import-arrow
test extends to `weld_motion_planner` (imports `weldplan`, never `seam_worker`).

## M0.5 — The cuRoboV2 spike, timeboxed

Decide adopt-vs-own for the commodity layers before M2 and M5 build them:
can V2's mesh-stamping + PBA+ run under our exact narrow band, and can its
batched L-BFGS drive a loss that includes our projection step? Also lift or
reject its sphere-fit tooling. Output is a decision note appended to
`ARCHITECTURE.md`'s open-items section, not code.

## M1 — Differentiable geometry core

The shared evaluator everything else is built on:

- Rational B-spline evaluation (de Boor) in torch, batched and differentiable;
  arc length and `s → u` inversion to the tolerances `SEAM-MODEL.md` §3 sets.
- Seam frames `{P, t, r, b}` and `apply_angles` per §5, matching the exported
  `frames.py` oracle to float tolerance.
- Batched FK over `cell.json` (arm + positioner), fp32 and fp64 paths, against
  an exported `kinematics.py` oracle.
- The workpiece-frame TCP map: placement ∘ `T_work(q_pos)⁻¹` ∘ `T_tcp(q_arm)`.

Done when: given joint samples, the evaluator reports position error against
`P(s)` and achieved work/travel angles against their bands — the same code
path trajectory generation (M5) will optimize through and the verifier will judge with.

## M2 — Distance fields and the sphere model

The bake per `ARCHITECTURE.md`: OCCT tessellation of the plan request's STEP,
Warp BVH narrow-band exact signed distance, PBA+ far field, dense part grid
(fp32, workpiece frame) and env grid (fp16, world frame), trilinear queries
with autodiff gradients. Torch spheres generated from `tooling.json` at load;
per-link auto-fit for the cell with the validation harness (max uncovered
depth, mean over-coverage — numbers, not vibes) and the per-cell asset format.

Done when: analytic solids (sphere, box) match closed-form distance to
narrow-band accuracy; the part bake meets the 1–2 s budget; the sphere
harness reports on the real cell.

**Status: done.** The world field, the fused query, the sphere model and its
authoring tool, `scene.py` — which assembles the world and workpiece groups
and answers `clearance(q)` differentiably — the J7 feasibility mask
(`positioner.py`), sphere-vs-sphere self collision, and `cost.py` — the hinge,
the augmented Lagrangian and the multiplier update — are all in. The part bake
is comfortably inside budget: 15 ms steady-state for tessellation plus a 1.5 mm
bake (`bracket_a`'s first call costs 859 ms, all of it OCCT import and Warp
kernel load). The whole collision term, forward and backward, is 2.5 ms at
batch 128 on `rosie_1400_v3` (`BENCHMARKS.md` §7).

Named and deliberately not built: a fused multi-field kernel, one launch from
world points to per-sphere cost with the group min and the hinge inside. It
was measured rather than assumed, and it is not on M2's path.

**The generated fixtures cannot be planned against, and one hand-authored
one can.** `tools/make_motion_fixtures.py` uses `default_placement`, which
is an anchor plus a zero offset and rpy — the part lands at the work
frame's ORIGIN, and on `rosie_1400_v3` that origin is inside the H-frame.
The part reads -41.6 mm against it at every positioner angle and the
feasibility mask correctly comes back empty. No default fixes this: where a
part sits on a table is a human judgement made by dragging it in the
authoring UI.

`data/motion/bracket_a_placed.weldplan` is that judgement, made once and
committed. Anchor on the part's own footprint, offset -0.66 m along the
table, yawed -90 degrees. It puts the part at world z 0.651–0.841, standing
on the table plate and clear of the H-frame's 0.63 ceiling, and the M2
stack runs on it end to end: J7 mask 100% allowed with 111.6 mm worst
clearance, reset pose 158.0 mm minimum with nothing in collision. It is the
only committed plan request that can exercise anything past M2, and it is not
produced by the generator.

## M3 — The verifier

Built early, on purpose: it is the incorruptible judge for M4–M7, and it is
independently useful (it can verify any trajectory, whoever produced it).

- CCD backend spike (python-fcl vs Bullet) against the actual fixture meshes;
  pick one, record why in `ARCHITECTURE.md`.
- The T2 verifier: exact-mesh CCD plus joint position/velocity/acceleration
  limits, fp64, fanned across the CPU pool. Fail closed.
- T1 alongside: directional-mobility computation and margins along `s`.

Done when: hand-built trajectories — one clear, one colliding between
samples (the tunnelling case), one limit-violating — are judged correctly,
and verifier throughput is measured against §5's waypoint-count table.

**Status: the backend is chosen and the verifier is in. T1 is not.**

**The spike found that no candidate has a usable mesh CCD**, which changed
what the verifier is. `tools/spike_ccd.py` runs all three against the real cell
meshes; `BENCHMARKS.md` §8 has the table. python-fcl and coal are exact and
agree to 0 mm, within 4% of each other on throughput. pybullet declines every
mesh-vs-mesh pair — convex ones too, so this is not about concavity — while
answering mesh-vs-primitive, so it never gets to be compared. And
python-fcl's `continuousCollide`, the one advertised swept-volume entry point
among them, returns *no collision* for a mesh pair that dense sampling proves
collides at t = 0.133, while getting a sphere-pair control exactly right. A
safety-critical API that silently answers "clear" is worse than no API.

So the verifier does not call a CCD; it **is** one. `verify.py` does conservative
advancement over exact meshes: a Lipschitz bound on how fast any point of a
link can move for a given joint step turns one query into a statement about an
interval, and the interval is bisected only where the statement fails. The
query underneath is coal's `security_margin` predicate — "is anything closer
than m" — which is 16–26× cheaper than the exact distance because the
traversal stops at the first box pair that settles it.

**coal is the backend.** It ties python-fcl on exactness and throughput and
wins on the two things that decide a verifier: it carries that predicate and a
sound `distance_lower_bound` (verified over 4000 poses, 891 of them
penetrating, zero violations), and it is on conda-forge rather than PyPI-only.

Measured on `rosie_1400_v3` with `bracket_a_placed` — 12 bodies, 54 checked
pairs, 19 269 triangles: **1.4 ms per segment** at weld-scale steps, 109
queries. §5's table sized a sampled check at 5 600–22 800 waypoints; the verifier
does not sample, so it is the trajectory the planner actually emits that gets
verified. The CPU pool is a 3.8× win on segments that bisect deeply and a
*loss* on cheap ones — each worker rebuilds the BVH — so serial is the
default and `workers` is opt-in until there is a persistent pool.

**T1 is in, and it is not the constraint the weave document specified.** That
document introduces the index as `μ_b'`, along the weave's transverse axis, to
keep an arm weave-capable — and weave is out of scope, so that floor is not
reinstated. What `mobility.py` computes is the same arithmetic along the
direction that survives: a seam is run at a programmed travel speed along its
tangent, and near a singularity in *that* direction the joints must slew hard
to deliver a modest tip speed. The direction is an argument, so `b'` drops
back in on the day someone takes weave on.

The index is directional and not Yoshikawa's `w = sqrt(det(J Jᵀ))`, because
`w` is the ellipsoid's *volume* and a weld needs one direction at a time. Over
4096 poses of this arm there is one where `w` is **125 % of its median** while
mobility along `+x` is **16 %** of its own — a screen on `w` ranks that posture
above average. The per-joint form is used for the verdict because it is
strictly the stricter: at 4 m/s tip speed the scalar form accepts 2287 poses
of 4096 and the per-joint form accepts 394, so the scalar version would wave
through 1893 poses that break a joint rate limit.

**On this cell the constraint is slack while welding.** At 8 mm/s the arm uses
0.3 % of a 3.14 rad/s budget; it binds on transit moves, not on the weld. That
is the generic-truth trap `WEAVE-AND-VERIFICATION.md` §7 names, and the reason
T3 has to *seek* bad postures rather than sample typical ones.

Left: the response artifact's verdict block.

## M4 — The redundancy graph

Batched IK screening (damped Newton, fp32, seeded per class), node
generation over spin × positioner × IK class, edge rules from the
joint-distance threshold, on-device Viterbi with K-best and the frontier
diagnosis. The rescue pass (band-deviation bins, D17 half-bin rule, rate
limits as edge rules) is part of this milestone, not an afterthought — the
default/rescue tiering only means the second pass runs conditionally.

**Status: inverse kinematics is in; nothing else of M4 is.** `ik.py` is the
piece the graph is mostly made of -- one solve per node -- so it is built and
measured first. Batched damped least squares with per-pose adaptive damping,
joint limits clamped every step so an answer is executable by construction,
and eight structured seeds covering the postures a wrist-partitioned arm can
hold one pose in. `BENCHMARKS.md` §9 has the numbers; the two that shape what
comes next:

- **fp32 at a large batch is the operating point**: 1.88 µs per pose at 65 536,
  against 322 µs at 256. Batch 256 and batch 2 048 cost the same wall time, so
  small batches are paying the driver. The graph must batch across seams, not
  only within one — which `ARCHITECTURE.md` already required for other reasons.
- **The last few percent of solve rate are seeds, not iterations.** Eight seeds
  reach 95.1 %, sixteen random reach 99.2 %, and raising the iteration cap from
  60 to 400 buys 0.24 points. Since seam samples are ~5 mm apart, each layer
  warm-starts from the previous one and the cold rate applies only to layer 0.

**Node generation is in too, with its dev page.** `nodes.py` enumerates the
lattice — samples × spin bins × positioner bins — and screens each cell,
recording a CAUSE rather than a boolean. The solve runs *unclamped* so that
"the arm has no travel for this" stays separable from "no posture reaches
this"; clamping would collapse both into one, and the frontier histogram
`ARCHITECTURE.md` asks for exists precisely to tell them apart, because the
remedies differ.

On `bracket_a_placed`'s first seam at 64 × 12 × 9 = 6912 nodes: 56% feasible,
27% no solution, 17% outside the joint limits, no frontier. The generated
fixtures come in at 20–29% against that 56%, which is the placement problem
already recorded above showing up in a second measurement.

`ui/graph.html` draws it, fed by `tools/make_graph_demo.py` writing
`motion-graph.v0` beside the other debug bundles (same rules: not a contract,
may break at any commit). The page exists because the summary hides the thing
that matters — "56% feasible" describes both a seam with room everywhere and
one surviving a single sample by one node, and the graph's edge rule needs
slack the second does not have. Drawn, they are unmistakable.

**The IK branch is a node dimension, not a collapsed one.** The first version
kept one solution per (sample, spin, positioner) — whichever branch converged
first — and that was measured before it was removed: 0.19% of adjacent
feasible samples jumped by exactly π or 2π, against a 99th percentile of
0.013 rad everywhere else. Branch flips, decided by seed order rather than by
geometry, and the edge rule would have cut the lattice at them. The lattice is
now `samples × spin × positioner × branch`, and a third of its cells have more
than one posture to choose from. A branch that lands where an earlier one
already is gets its own cause code, so the histogram does not overstate the
arm's freedom.

**Branch index is still not the same thing as IK class.** Warm-starting from
the previous sample's answer per branch is what would make it one; without
that, the same seed can converge to different branches at adjacent samples,
and the π jumps are 0.42% rather than gone. Recorded, not fixed — it is the
edge rule's problem to see and the next commit's to remove.

**The tool axis is +X, and it is measured rather than assumed.** `torch_pose`
aligned the electrode with the tool frame's **+Z** until 2026-08-12, taken from
`spheres.torch_spheres`'s docstring — which states the convention of the
TORCH's own local frame and says nothing about how that frame is bolted to
this robot's flange. Two different frames, one measured and one assumed,
ninety degrees apart. `rosie_1400_v3` ships the measurement in
`end_effector_frame_check.json`: *"TCP +X points from the wrist toward the
torch tip / into the workpiece side; the 45-degree work angle is baked into
the fixed link_6→end_effector rotation from CAD."* Correcting it moved the
lattice's feasible share from 14% to 11% and its per-sample count from 116–129
to 88–103, so it was never cosmetic — and nothing downstream had noticed:
inverse kinematics converged onto the wrong pose and the lattice filled with
green.

**And it is read from the cell, not hard-coded.** It was a module constant
for two commits, set to the one value that is right for the one robot in this
repo — which is indistinguishable from a correct implementation until a second
robot arrives, and then wrong silently. It is now
`robot_description/<model>/config.json`'s `torch`, hand-authored beside the sphere model
and the profile, and a cell without one is **refused** rather than defaulted,
on the same argument the profile already makes about velocity limits.

**Not open — a field name misread once, and the arithmetic settles it.** That
file also records `torch_centerline_in_link_6` and
`x_angle_to_torch_centerline_deg = 45.67`, which looked like a competing
candidate for the electrode direction and is not one.
`torch_centerline_in_link_6` equals the normalised `link_6 → end_effector`
origin offset to twelve decimal places: it is which way the WRIST extends —
398 mm along link_6's −Z to where the torch mounts — and says nothing about
which way the gun aims. The 45.67° is therefore the mount's tilt, the same
45° the convention line says is baked in. +X is the torch axis, and
`tool_axis` stays a parameter because it is a per-cell measurement, not
because this is undecided.

`/graph.html` also draws the robot at whatever node is under the cursor, and
that view earned its place twice on the day it was written: it was the only
thing that showed the arm exploded across the cell while the flange checked
out at 150.0 mm from the weld against three independent kinematics.

**The edge rule and the sweep are in** (`graph.py`). 64 layers × 864 nodes
searched in **57 ms**, K-best paths, and a frontier with a cause histogram
when nothing crosses.

The threshold is meant to be physics — seam step ÷ travel speed × each joint's
velocity limit — and **no cell here can supply it**, because the profile
carries position limits and no rates on purpose. So
`threshold_from_limits` returns `None`, the caller states a threshold instead,
and `Graph.derived` records which happened. A plan can never quietly claim its
edges were physical.

**Asking whether K-best was diverse found two state bugs**, neither of which a
feasibility, cost or smoothness test can see:

- *The positioner was not in the search state.* `lattice.q` is the arm alone,
  so two nodes with the same posture and different workpiece angles were the
  same place — zero distance, freely interchangeable, and the sweep would spin
  the part 180° between samples 1.27 mm apart for nothing. Six paths came back
  with identical cost and postures differing by 0.0°.
- *The positioner's bins repeated an angle.* J7 spans a whole turn, so −180°
  and +180° are one orientation and `linspace` binned both. A ninth of the
  lattice was a duplicate column, and one of the K went to it — a path
  differing from the best by 360° of nothing. The spin bins had used
  `endpoint=False` from the start; the positioner now does too, when its range
  closes.

With both fixed the K paths differ by 5–54° of arm and 180° of positioner,
which is what trajectory generation needs to seed different basins.

**`/graph.html` plays the path back.** The K paths and their costs travel in
the bundle as node indices per layer, the page draws the chosen one over the
lattice at display resolution — not into the lattice's own pixels, where at
864 rows a node is a fraction of a pixel and the line would vanish into the
scale-up — and a scrub bar poses the robot at each step with the weld point
marked. Playing is the same code path as scrubbing, on a 50 ms timer.

**14 of 15 seams cross, and the one that does not is the robot.**

Running M4's own done-when across every committed plan request for the first time
found 9 of 15 seams with no path. Five of those were one authoring-side bug
27 micrometres wide — a reachability boundary that landed a hair before a 90°
corner and got absorbed ACROSS it, so the seam's first sample sat on the far
face of the turn and the torch was asked for 60° of swing in 2.6 mm. Two more
followed from six-decimal rounding of NORMALIZED positions, which quantizes a
190 mm curve to 0.19 µm and can round down past a corner. `docs/issues/0001`
has the whole route, including the three wrong diagnoses that came first.

The result is byte-identical across two fresh processes, which settles the
"all deterministic" clause, and both facts are pinned in
`tests/motion_planner/test_graph.py` rather than left as a run I once did.

The remaining seam, `dog_box` 1, has **47 feasible nodes at layer 28 and no
edge into any of them**. Nothing is special about the geometry there — 6.2° of
electrode turn against a 6.2° median, no node within 0.32% of a joint limit —
so it is the cell. The cause is `docs/issues/0002`: a lattice cell is
`(sample, spin, positioner, branch)` and `branch` identifies the SEED, not the
posture it reached, so the cells are not continuous tracks. On `bracket_a` the
same cell holds postures 66–229° apart at adjacent samples. Warm-starting each
layer from the previous one is the fix; it was tried once, on the one seam
where a corner made it impossible, and needs re-running now that the corner is
gone.

Left in M4: the frontier as a reported artifact, and the rescue pass. Corner
handling belongs to the split protocol rather than to the graph.

Done when: the dog_box perimeter yields a continuous K-best set meeting the
edge threshold; a fixture seam engineered to be impossible at the authored
angles is rescued by the band pass; a truly impossible one reports a frontier
with a cause histogram; all deterministic; graph time inside budget.

## M5 — Continuous trajectory generation

Named for what it does: the input is a DISCRETE lattice path — joint rows
250 ms apart, valid only at the samples — and the output is a CONTINUOUS
executable trajectory. The stage was called "refinement" through design; its
owner renamed it because "refine" undersold the job — the real deliverable
is that every guarantee holds BETWEEN the samples too, and a name should
say so.
Requirements settled in conversation, 2026-08-13; the deltas from the
original sketch are the objective (cuRobo's smoothness stack instead of a
bare feasibility polish) and the angle attraction.

**Variables.** Joint spline coefficients, arm axes plus positioner; all
seams × K seeds in one tensor. TIMING IS AN INPUT: `t_i = i × step_s` from
the travel speed, a process parameter. cuRobo dilates dt until limits hold
because its reach tasks may arrive late; a weld may not — dt-dilation is
M6's tool for transit moves, never the seam's.

**Spline resolution: knots at the lattice samples, cubic HERMITE.** One knot
per sample (dt = 0.25 s ≈ 2 mm of arc), each carrying position AND velocity
`(q_i, q̇_i)`. Position-only interpolation is not enough, by arithmetic
rather than taste: between knots the tip sags off the seam by about
`κh²/8`, which crosses the 0.25 mm tolerance exactly at a 2 mm corner
radius — geometry this corpus contains. Matching the DERIVATIVE too was the
owner's call (2026-08-13), and it lands as a projection, not a cost:
`J(q_i)·q̇_i = v_travel·t̂(s_i)` is LINEAR in `q̇_i` once `q_i` stands, so
one DLS least-squares solve makes it exact — and everything hard enough to
project, this stage projects. Two dividends: the between-knot error drops
to `h⁴/(384R³)` (0.005 mm at the same 2 mm radius, fifty-fold margin), and
the tip's speed stops wobbling between samples — constant travel speed is a
PROCESS condition position-only tracking silently lets slip. The electrode
direction's rate is deliberately not projected: direction is pinned at the
knots, the angular tolerances are twenty times looser than the positional
one, and the verifier's between-sample check is the arbiter if that judgement
is wrong.

**Where it is judged, and what leaves (2026-08-30).** Free space's three
numbers, imported so the two stages cannot drift (`control_solver`):

- *The cost grid.* Every knot plus `INTERPOLATION_STEPS − 1` samples of the
  Hermite inside each span (`Synthesizer.grid`, 4 → a read every 62.5 ms).
  The tracking quadrature is the mean over a span's interior samples — at
  two steps a span it is exactly the old midpoint term — and the collision
  term is read there rather than at the knots.
- *Swept.* The AL's environment constraint at a grid sample is the least
  clearance the sphere meets there AND along the chords to its neighbouring
  samples (`CollisionCost.sweep_samples`, `scene.chord_clearance`, capped at
  `SWEEP_STEPS` reads per half chord), so a wall between two samples is
  priced by the optimizer instead of found by T2. Same multipliers, same ρ
  schedule; the constraint is on the segment, not the point.
- *The wire.* What leaves is the same Hermite re-evaluated at `WIRE_DT_S`
  (0.01 s, the controller's clock) in float64 — `(q, q̇, t, f)` per sample,
  `round(T / 0.01)` intervals spread evenly (`resample_wire`). Each sample
  carries the cubic's own derivative, so a Hermite through the wire IS the
  optimized curve; the tracking certificate and T2 walk that wire, at its
  own fractions. `lattice_samples` on the trajectory says how many knots
  the synthesis actually ran on.

**Cost (soft, minimized).** cuRobo's squared-L2 smoothness stack, by finite
differences over the fixed dt — legal here precisely because M5 has no
sweep: the trajectory is one tensor and jerk is three `diff`s, with none of
the Markov-order constraint that keeps jerk out of M4's edge cost.

    w_v · ‖vel‖²  +  w_a · ‖acc‖²  +  w_j · ‖jerk‖²        (cuRobo trajopt stack)
    + w_θ · (θ_achieved − θ_authored)²                       (angle attraction)

The attraction term is what makes the deviation band a BUDGET rather than a
free zone: clip alone lets the optimizer spend 5° of the weld engineer's
intent to buy millimetres of clearance at no cost. Spending must cost.

**Constraints (hard, each by its own mechanism — never uniform penalties).**

    seam tracking (5 DOF)   PROJECTION: 1–3 DLS Newton corrections per sample
                            after each step, then spline refit. Penalties
                            plateau at millimetres; projection is exact and
                            leaves the optimizer exploring only the null
                            space — spin, positioner, posture — which is the
                            real search space anyway.
    deviation band          CLIP to [authored − minus_dev, authored + plus_dev]
                            (the split fields; authoring hands them over
                            pre-intersected with its absolute band).
    velocity limits         CLIP, pairwise: |Δq| ≤ rate × dt.
    joint position limits   CLIP, box.
    collision (env + self)  AL — `collision/cost.py` as written: margins,
                            per-constraint multipliers, ρ growth to zero
                            violation. The only AL in the stage. Read on
                            the cost grid and SWEPT along it (above).
    μ_t' floor              AL alongside collision; expected slack on this
                            cell while welding (measured: 0.3 % of budget).
    acceleration limits     COST PRESSURE (the acc²/jerk² terms) plus the
                            verifier's verdict. The one open knob: escalate to AL
                            if the corner seams do not converge under cost
                            alone. Not clip — a bound on second differences
                            is not a box.

**Loop.**

    seeds    K candidates per seam, RE-RANKED by this stage's own objective
             first — M4 ranks by L1 motion, and the two stages measuring with
             different rulers is how a wrist-flip twin outranks a genuinely
             smoother path.
    outer    AL iterations: step_multipliers, grow ρ
      inner  ADAM steps on the spline coefficients (autodiff end to end:
             de Boor curves → fused FK → ESDF), per-axis step scale
             proportional to the velocity limits; moments reset at AL
             boundaries. Each step: gradient → clip → project → refit.
             Adam and not cuRobo's batched L-BFGS, for a reason and not a
             preference: L-BFGS estimates curvature from gradient HISTORY,
             and our seam projection moves the iterate outside the
             objective's own logic every step, poisoning exactly that
             history — cuRobo can afford L-BFGS because it has no hard
             projection, only costs and clamps. Deferred, not dead: once
             the Adam baseline stands, L-BFGS with sparser projection and
             a history flush is a measurement away (the M0.5 spike's open
             question, finally answerable). Second-order information lives
             where projection is a constraint rather than an interruption:
             the winner-only SQP polish.
    winner   lowest feasible objective per seam, then the fp64 SQP polish
             (sequential quadratic programming — constrained Newton), serial,
             winner only.
    verify   the M3 verifier judges the RESULT: CCD, limits, and between-sample
             tracking against `tolerance.position_mm` — which is the verifier's
             acceptance number, never a budget the optimizer spends (measured:
             the lattice already lands 0.0002 mm median at the samples).

**Tracking is verified CONTINUOUSLY, by the collision verifier's own pattern —
not by sampling.** The owner's requirement (2026-08-13), stated precisely:
the tip path `FK(q(t))` is NOT a spline — a cubic through trigonometry is no
polynomial — so symbolic spline-vs-spline comparison does not exist. What
does exist is a certified bound, the same conservative advancement
`verify.py` already runs for collision: on each knot interval `q̇(t)` is a
quadratic with closed-form extrema (no sampling), the seam side is an exact
spline, and the per-link Lipschitz machinery bounds how far FK can carry the
tip — so one computation certifies the WHOLE interval's maximum tracking
error, bisecting only where the bound fails. The division of trust is the
point: the OPTIMIZER may sample (its mid-interval quadrature points on the
tracking cost are insurance, not proof — it is not trusted anyway); the
VERIFIER may not, because it is.

**Built, and the division of trust paid for itself immediately.**
`verify.certify_tracking`, with `SeamTarget` supplying the seam side. Three
bounds meet at each node: `_velocity_extreme` closes the Hermite's quadratic
derivative in form (checked equal to a dense numerical maximum, not merely
above it); `chain_reach` turns joint rates into tip and target speeds by
walking the chain from a POINT up to a joint, which is the mirror of
`reach_radii`'s walk down from a joint to its meshes; `SeamTarget.lipschitz_f`
bounds `|dT/df|` — exactly on a degree-1 weld line, where `s` being arc length
makes the weld point's term the seam's own length and the torch direction's
term closed-form, and returning None on a curved one so the gap travels as
`unverifiable` instead of as a pass.

Two structural decisions the first cut got wrong and the numbers corrected:

- Intervals are cut at BREAKPOINTS first — segment joins of the weld line and
  knots of the piecewise-linear angle profiles — because the tool0 target
  genuinely JUMPS at a corner, by `2 · standoff · sin(corner/2)`, and a
  Lipschitz constant taken across one would be bounding a function that has
  none.
- Once out of tolerance the recursion switches from a yes/no test to branch
  and bound on the SUPREMUM (`SUP_SLACK`). Stopping at the first witness is
  sound and useless: on `seam_0007` it reported 4.3 mm for a trajectory whose
  worst instant is 20.3 mm, and it is the 20 that M7's slowdown and M8's split
  are sized against.

Measured on `bracket_a_placed`, and this is the result the stage exists for:
the five smooth seams certify CLEAR over their whole length (bound ≤ 0.5 mm,
nothing unverifiable), while the optimizer's own midpoint number understated
the true worst everywhere it was asked — 0.10 against 0.16 mm on `seam_0003`,
and 4.27 against 20.29 mm on `seam_0007`, a factor of 4.75. Sampling was not
merely unproven; it was wrong by the margin that decides the seam.

**The re-ranking was not a formality.** `plan_weld_trajectories` plans all K of
M4's ways across a seam and chooses by this stage's own objective (smoothness
plus tracking, a function of the trajectory alone — deliberately not the
loop's augmented-Lagrangian value, whose multiplier state is not comparable
between two runs). A candidate that penetrates loses to every one that does
not regardless of score, because a negative clearance is a fact and the
smoothness terms are a preference. Only the winner is certified, since the
tracking bisection and the T2 verifier each cost more than the synthesis they
judge; the losers travel with the winner as scores so the choice is arguable.

Measured across `bracket_a_placed`, the two rulers **agree on every seam that
is easy and disagree on exactly the two that are hard** — which is the useful
shape, because it means the re-ranking costs five syntheses where it changes
nothing and earns them where the seam is in trouble:

    seam        M4's pick    M5's pick    peak acc, M4's → M5's
    0001,0003,0004,0006,0010   #0    #0   unchanged
    seam_0005        #0        #1        4.79× → 4.11×  (M4's scored LAST of five)
    seam_0007        #0        #4        3.60× → 3.34×  (M4's scored third)

**And the certificate then refuses to be impressed.** On both corner seams
the sampled midpoint number improves — 7.54 → 7.27 mm and 4.27 → 3.32 mm —
while the CERTIFIED worst tracking error stays at 20.29 mm on both, to four
figures. A different seed moves where the sampled points fall and does not
move the corner. That is the previous section's lesson arriving a second
time, from the other direction: had this stage been ranking on its sampled
tracking number alone it would have reported a 22 % improvement on
`seam_0007` that does not exist.

One honest cost, unresolved: `seam_0007`'s chosen candidate breaks the
acceleration limit at **13 samples against the sweep pick's 9**, at a lower
peak. The objective sums squared normalised rates, so it prefers many small
exceedances to one large one, and it has no term for the count. That is
probably right for a drive that saturates on peak — but it is a preference
nobody stated, and M7 is where it has to be argued rather than inherited.

**And the verifier now judges the curve, which nothing had ever asked it to.**
Until this stage shipped, no part of the pipeline called `verify` at all —
only its helpers — so the T2 certificate had never been run on a planned
trajectory. Two things had to be true before it could be:

- `certify_segment` takes joint VELOCITIES, and with them the motion between
  two waypoints is the cubic Hermite this stage emits rather than the straight
  line through them. That is not a refinement: measured on `bracket_a_placed`
  the curve leaves the chord by 4.4° of joint angle on a corner seam, and the
  two certificates return different verdicts on `seam_0007` — the chord
  reports a collision the real path does not have. The chain walk behind the
  Lipschitz bound became `chain_weights`, a vector, because the speed bound is
  no longer one number for a whole segment.
- `check_travel` was reading `limits[axis]["velocity"]` while the profile
  loader writes `velocity_rad_s` — the identical key mismatch that was found
  and fixed in `check_limits`, still sitting in the copy next door and still
  reporting T1 as unverifiable over a profile that states its limits. Both now
  read one `_rate` helper.

Result on `bracket_a_placed`, at margin 0 (see below): the five smooth seams
verifier **PASS** with nothing unverifiable — no penetration anywhere on the
curve, T1 checked against the stated rates, no singular waypoints. The two
corner seams fail on ACCELERATION alone, with zero collisions: collision-free
and physically infeasible at the programmed speed, which is the honest verdict
and M7's input.

**Deferred by the owner, written up as `docs/issues/0007`:** the verifier's 2 mm
pad exceeds the clearance a weld pass has by design — the gun body runs
1.3–2.4 mm off the part at the measured 20 mm standoff, so at the default
three correct seams report collisions they do not have. Nothing was loosened;
the margin is a parameter defaulting to the verifier's own number, and the verdict
now separates `penetrating` from `collisions`. The stopgap that makes this not
urgent is the owner's: grow the torch geometry and the buffer lives in the
model, where the screen, the AL cost and the verifier all see the same shape
without being told about it separately.

Done when: on both fixtures, between-sample tracking error is within
`tolerance.position_mm` at every point, achieved angles sit inside their
deviation bands with rate limits honoured, acceleration findings from the
corner seams are resolved or reported, the ESDF reports clearance — and the
M3 verifier passes the result. Fixed-seed determinism.

**The corpus was replaced wholesale on 2026-08-13.** The owner placed three
parts in the authoring UI, saved the projects, and directed that motion
planning use only those: `hitch_2x`, `bracket_a_2x`, `bracket_b_2x`.
`tools/project_to_plan_request.py` turns a `.weldproj` into a plan request using the
operator's OWN weld list and placement rather than the defaults
`make_motion_fixtures.py` has to invent. Deterministic bytes; the five
generated plan requests and their oracles are gone.

    plan request   seams   sweep     tracking      T2 verifier
    hitch_2x        19     19/19     18 PASS       19/19 PASS
                                     1 REFUSED
    bracket_a_2x    11     10/11     10/10 PASS    10/10 PASS
    bracket_b_2x    22     22/22     22/22 PASS    22/22 PASS

**Each part now carries a different hard case, which the old corpus could
not.** `bracket_b_2x` certifies end to end — the happy path, 20 s from
plan request to certificates. `bracket_a_2x seam_0002` has no feasible node at
its first sample (5104 collisions, 3254 joint limits, 9640 with no IK): a gun
that cannot fit, reported rather than planned around. And `hitch_2x
seam_0008` is a 239 mm degree-2 curve — the first curved weld line the corpus
has ever held — which the tracking certificate REFUSES because no derivative
bound exists for it yet. That branch had been written, unit-tested on a
synthetic chord, and never once run against a real part.

That one seam turns out to carry three of the corpus's hard cases at once: it
is the only curved line, the only seam where M4's ruler and M5's disagree
(the sweep's winner is not this stage's), and the only path that moves the
positioner through more than one bin.

Prerequisite still open: **deviation bands authored on a placed plan request.**
Neither `bracket_a_placed` nor `bracket_b_2x_placed` carries them, so the
angle-attraction term still has nothing to run against — the requests that
carry bands and the ones that are plannable remain disjoint. (The second-fixture
prerequisite is met; see above.)

## M6 — Free space

Joint-config-to-joint-config L-BFGS planner (batched seeds, clamped
endpoint position and velocity): approach and retract from the per-cell
reset pose, and the same solver reused for transit shortening. These
segments go through the M3 verifier too.

**The reset pose is stated now (2026-08-13), and the cell profile reports no
problems at all for the first time.** It is the URDF's own home: every
revolute joint in `robot.urdf` carries `<calibration rising="0"/>`, so zero
is what the robot's description calls its reference — the owner's decision
was to read that number rather than choose one, which is the difference
between a fact about the cell and a planner's preference.

Checked before it was written down, because a reset pose that collides would
put every approach and every retract in the plan inside something: every
axis's zero is inside its declared range, and the T2 verifier measures 54.9 mm
of worst clearance there on all three plan requests. The closest pair is
`link_4` against `link_6` — the arm folded against itself — which is why the
figure does not move with the part.

Only the DRIVEN axes are listed. J8 and J9 are held and their values come
from `axes.held`; a reset pose that repeated them would be a second place to
keep the station index in step, and the two would drift.

So M6's remaining inputs are all present: the pose to leave from and return
to, the rate limits to move under, and — from M5 — the knot velocity each
seam starts and ends at, which is what the clamped-endpoint-velocity
formulation above needs to hand the arm over to the weld already moving.

**Status: the plan joins, and on `bracket_b_2x` every link of it is
collision-certified.** `freespace.py` builds every move between the welds
from the same cubic-Hermite representation M5 emits, so the T2 verifier certifies
the curve with the same call. Endpoint POSITION and VELOCITY are both
clamped, which is the contract: the arm reaches a seam already moving at that
seam's own speed, so the joined trajectory is C1 across every boundary by
construction rather than by a smoothing pass.

At verifier margin 0, three seams per plan request, links certified — **every link of
every plan request, as of 2026-08-16, which the corpus has never been:**

    bracket_b_2x   4 of 4     env  3.9 … 7.7 mm   self  7.6 … 11.9 mm
    bracket_a_2x   4 of 4     env  2.2 … 6.0 mm   self  4.7 … 11.6 mm
    hitch_2x       4 of 4     env  2.8 … 9.5 mm   self  0.4 … 10.4 mm

It was 4, 3 and 2 of 4 the day before, and what closed the last three links was
the same work that answered a complaint about the SHAPE of the ones that already
passed — a link swinging `J1` through 101 degrees to travel a net 10, and flown
at half the rate the cell allows. Four faults, one symptom, each measured before
it was believed (`BENCHMARKS.md` 12.5, `docs/issues/0011`):

1. **The trust region was a ratchet.** The per-row step scale halved on a
   rejected step and never recovered, so the augmented Lagrangian's multiplier
   growth — where the objective legitimately jumps and a few steps are correctly
   rejected — cost four halvings and froze the loop for the remaining two thirds
   of its budget at a 100% accept rate. This is why every earlier attempt to fix
   the shape by reweighting had failed: an optimizer that is not taking steps
   cannot hear its objective.
2. **Nothing in the objective priced distance.** cuRobo's smoothness stack
   prices how a path is flown, and with acceleration weighted ten times velocity
   a wide graceful swing outscores a direct move with a corner. The direct path
   was in the seed pool at 560 degrees and lost to a bowed one at 590.
3. **Descent cannot jump between paths.** `Linker.shortcut` replaces a span of
   knots with the straight line through it when that line screens clear —
   cuRobo's `PathPruner.prune_path_with_shortcuts` on a trajectory instead of a
   graph path, coarse to fine so it costs `O(L log L)` screens. This is what
   closed `hitch_2x`'s two penetrating links: they were not badly shaped, they
   were in the wrong homotopy class and no gradient leaves one.
4. **The collision term pointed an AL at an unsatisfiable target.** Free space
   had inherited M5's augmented Lagrangian and moved its margin to 50 mm, so the
   multipliers on the clamped weld poses — 3.9 mm off the work, by definition —
   accumulated forever: after six outer steps, nothing in collision, `rho` grown
   281x, 118 entries still under pressure, all of it standoff. Split into an AL
   on `clearance >= 0` and a fixed-weight buffer with a linear tail beyond it,
   `bracket_a_2x`'s stubborn transit went from 2 segments in contact to 2.2 mm
   clear.

Timing is chosen here and nowhere else, and it is now chosen twice: `duration_for`
for the seed and `retime` for the answer. The closed form is a floor for a
rest-to-rest cubic along the STRAIGHT line, so once the optimizer has moved the
interior the knots reach about half the rate limits; `retime` recomputes the
horizon from what the trajectory achieved and re-solves on it, cuRobo's
`finetune_dt_scale` loop. Its accept test asks both of the verifier's questions,
because a shrink that landed over an acceleration limit reached the verifier once,
and one that shaved clearance from 3.9 mm to 0.1 mm reached it twice.

Getting there took three findings, each measured before it was believed
(`docs/issues/0009`):

1. **The standoff waypoint.** A torch approaches its weld along the
   electrode, from above. Seeding with the weld pose pushed back along `+u`
   — an IK solve seeded at the weld configuration itself, so it stays in the
   same branch — took the approach from 29 mm inside the part to 3.9 mm clear
   of it, from the seed onward.
2. **The seed ranking could not see the arm hitting itself.** `CollisionCost`
   has always priced self-collision; the ranking that picks the winning seed
   read only the environment field, so a path that folded `link_4` into
   `link_6` scored as perfectly clear and won on smoothness.
3. **The winding, which was the real one.** A revolute axis at `t` and at
   `t − 2π` is the same robot — 0.18 µm of difference across three seams —
   and M4 picks one per seam with no view of the plan it will sit in.
   `roadmap.py` showed why that matters here: **this arm has no clear pose at
   all with `J5` between 50° and 145°** (0 of 16 384 samples; best
   self-clearance −3.9 mm), so the reset pose and `bracket_b_2x seam_0001` as
   M4 wound it are in different components of the free space and no approach
   exists. `freespace.rewound` commands each seam on the reachable turn
   instead.

`roadmap.py` is new: a deterministic Sobol PRM over the seven driven axes,
screened against both the ESDF and the arm's own self-pairs, `heapq` for the
shortest path. It is a SEED generator and says so — it screens at samples,
which this planner refuses as proof everywhere else, and what it produces is
judged afterwards by the verifier. It earns its place twice: as a seed for the
links that need one, and as the oracle `rewound` consults when the nearest
winding turns out to be unreachable (`bracket_a_2x seam_0001`: `+165°` is
nearer and has no route; `−195°`, the same weld, routes in five waypoints).

The shape is cuRobo's — a cheap seed first, a global method only when it
fails — and it shows in the output: once the plan is rewound, the winning
seed on `bracket_b_2x` is `straight` on three links of four.

Not L-BFGS, which this milestone asks for. The reason M5 could not use it —
a hard seam projection poisoning the curvature estimate — genuinely does not
apply in free space, but the rate CLIP is still a projection, so the case is
now a measurement rather than an argument and the loop that is already proven
here is what runs until someone takes it.

**On the wire and in the drawer (2026-08-13).** `run_weld_trajopt=true` on the motion
server now also joins the plan: the response gains a top-level `connecting_trajectories`
list (`freespace.move_json`, the same Hermite shape as a seam's trajectory
plus `axis_names` and `held` so a link plays on its own), the per-seam
trajectories become the REWOUND ones so every handover matches to the digit,
and a joining failure travels as `connecting_trajectories_error` beside intact seam
trajectories rather than costing them. The full 22-seam `bracket_b_2x`
plan: 23 links, 20 free of contact, worst handover mismatch 0.0000° on the
wire, ~61 s to join (each link solved alone — batching them is the obvious
M9 item). The drawer's list becomes the plan in execution order and the
transport walks stops; `docs/WEBUI.md` D15 has the reader's contract.

## M7 — Timing and end-to-end

Time parametrization against the cell profile's velocity and acceleration
limits, producing the executable trajectory, which then feeds the verifier.
First full run: plan request in, verified trajectories out, for every seam of
both fixtures.

Done when: `pixi run motion-plan fixtures/dog_box.weldplan` completes with
all seams gated green, and the result plays back in the existing viewer via
`urdf-viewer-trajectory-v1`.

## M8 — The response artifact and the split protocol

The response format: per seam a trajectory or a structured infeasibility
record (frontier, cause counts, proposed cuts with certified sub-plans),
plus T1 margins, the T2 verdict with the geometry actually tested, achieved
profiles, timing with reserved β. Deterministic bytes, schema id minted like
the plan request's. Split proposals wired from M4's frontier machinery, firing
only after the rescue pass. A fixture placement engineered to be
half-reachable exercises the whole loop: plan → proposal → operator splits
upstream → new plan request → plan succeeds.

## M9 — Performance

Everything here was designed for but not yet measured: content-addressed
caches (ESDF by STEP/cell digest, plans by seam/placement/cell/tooling
digest), the async per-seam DAG overlapping GPU refine with CPU verify,
CUDA-graph capture on the Viterbi sweep and projection loop if launch-bound.
A benchmark task pins the budget: tens of seconds cold per plan request, 1–2 s
to replan one edited seam. Numbers in the doc get replaced by measured ones.

**The instrument is in, ahead of the optimisations it is for.** `timing.py`'s
`Stopwatch` runs the whole pipeline, at two grains that nest. `plan_motion`
reports `collision_model`, `dp_seam_search`, then `weld_trajopt` and
`connecting_trajopt` when they were asked for and not handed in, and `total`. Inside `dp_seam_search`, `plan_seam_search`
reports `collision_model` — near zero, because the model was handed to it — `seams`
and its own
total, and every seam reports `seam`, `setup`, `targets`, `ik`, `screen`, `grid`,
`search`, `limits` and its own total. They travel three ways — printed
to the planner's stderr as a breakdown, printed by the server as a per-request
line with the queue wait separated from the work, and carried in the response
as `timings_ms` so the Plan tab's drawer draws them (WEBUI.md D13).

Two properties are load-bearing and both are tested. The clock
**synchronises the device at every stage boundary**, because a `perf_counter`
around an async CUDA call measures the launch and not the work — unsynchronised,
the inverse kinematics' time lands under the collision screen and sends
whoever reads it to optimise the wrong stage. And `total` is the SPAN, not the
sum of the stages: the difference is time no stage claimed, and a breakdown
whose bars add up to its own total by construction can never show it.

First measurements, `dog_box` on an RTX 5090, eight seams, k=5, warm process:
`search` 44%, `screen` 27%, `targets` 9%, `seam` 7%, `ik` 7%, and the whole
run 1.4 s against a `collision_model` of 55 ms — a distance field the process had
already baked. The same run on a freshly started server pays 652 ms for that
scene, which is the cold/warm split this milestone's ESDF cache is aimed at,
now visible rather than argued. `ik` at 7% is the closed form's doing; before
it, the same stage was most of the build.

## M10 — Server and UI, thin

The authoring server grows a "plan" action that shells out to the CLI and
serves the response as an artifact; the plan tab renders verdicts, margins
and split proposals. Deliberately last and deliberately thin: everything it
shows exists in the response file, so this milestone adds no planning logic.

## MV — the dev visualizer, growing beside every milestone

A web viewer for a human to *look at* each milestone's output, built as a
dev-only page and grown one panel per milestone rather than as its own late
milestone — a stage whose output can only be judged by reading numbers is a
stage that will be judged wrong.

**Shape.** The planner learns to write a **debug bundle** beside its real
output: `motion-debug.v0`, a zip of JSON (and binary arrays where JSON would
be silly) that any stage can append to — sampled frames, ESDF slices, graph
layers, seed and final trajectories, verifier verdicts, timings. Explicitly
versioned `v0` and explicitly NOT a contract: it may break at any commit,
nothing downstream may depend on it, and the response artifact of M8 remains
the only thing the planner promises. The visualizer is a second entry page in
the existing `ui/` Vite app (`viz.html`, a "Motion (dev)" surface kept out of
the operator flow), because that app already renders the part mesh, weld
lines, rails and frame triads — the scene machinery is reuse, not rebuild.
It loads a bundle by file drop or from the server's artifact directory, and
draws the robot from the cell bundle's own meshes — a dev-machine luxury the
plan request deliberately does not ship.

**What each milestone adds:**

- **M1** — frame triads and torch direction glyphs along `s`, evaluated by
  the new torch core, overlaid on the authoring UI's own rendering of the
  same seam: two implementations, one picture, divergence visible as
  double vision.
- **M2** — ESDF slice planes (drag a plane, see the field), the sphere model
  overlaid on the robot meshes, per-link fit-quality coloring from the
  validation harness.
- **M3** — the trajectory as a polyline you can zoom into, CCD verdicts
  colored per segment, limit-violation traces; the tunnelling test case as
  the demo scene.
- **M4** — the graph made visible: feasible-node count as a heatmap over
  `s` × spin, the K-best paths through it, the frontier marker with its
  cause histogram on a failing seam. **Two pages, because the lattice and
  the search are two different objects.** `/graph.html` is the lattice and
  the answer; `/dp.html` is the sweep's own cost table, which otherwise
  exists nowhere — the dynamic program keeps one cost row and overwrites it
  every layer, on purpose, so the table that makes it a dynamic program is
  gone by the time `search` returns and the one property separating it from
  a greedy walk with a good seed appears in no artefact. `search(trace=True)`
  keeps it; the page draws it as a wave you can run across the seam and a
  back-pointer chain you can follow home with a mouse.
- **M5** — seed vs refined trajectory side by side; tracking error, achieved
  angles against their band plan requests, and clearance, each plotted along `s`.
- **M6/M7** — full playback: time scrubber, robot animated, tip trace,
  per-joint velocity/acceleration against limits.
- **M8** — the response rendered: per-seam verdict list, split proposals
  drawn as cut points and colored sub-spans on the seam.
- **M9** — the stage breakdown per run, cold vs warm, against the budget.

Done-ness of each planner milestone includes its panel: M4 is not done when
the DP returns paths; it is done when you can see them.

## What is not scheduled

**Weave.** The planner does not expand, reserve for, or verify weave. A
weaving seam is planned as its centreline, and the transverse mobility weave
would need is not held open for it -- so a program that weaves is planned
approximately and `request.py` says so. Reinstating it is a timing-stage
budget and a verifier-side expansion, both additive; nothing below is built in a
way that assumes weave is absent.

T3 (adversarial per-cell `μ_b'` sweep) and T4 (hardware weave measurement)
are campaigns against a real cell, not milestones of this package; the verifier
carries their absence as an explicit caveat in every response until they
exist. Sequencing-as-TSP, multi-torch, and seam-tracking intent stay in
`SEAM-MODEL.md` §10's deferred list.
