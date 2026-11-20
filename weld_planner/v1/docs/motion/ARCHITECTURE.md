# The motion planner: architecture

Status: agreed shape, nothing built. This is the overview the code will be
written against. The contract it consumes is fixed by `MOTION.md` (the
`.weldplan` plan request, read by `weldplan.read_request`, stdlib only, never
importing `seam_worker`); what the planner owns is enumerated in
`SEAM-MODEL.md` §8. Nothing here relaxes either of those.

**Weave is out of scope.** The planner plans a weaving seam as its
centreline and does not reserve mobility for the oscillation, expand it for
the verifier, or budget time against it. `WEAVE-AND-VERIFICATION.md` still fixes
those obligations for whoever takes them on; nothing here discharges them,
and `request.py` says so on any program that weaves.

The approach is the cuRobo family's — batched gradient trajectory optimization
over an ESDF. The original plan was to reimplement it because cuRobo v1's
source-available license ruled it out; **cuRoboV2 (April 2026, arXiv
2603.05493) is Apache 2.0**, which reopens the option of depending on its
components (world model, ESDF pipeline, batched L-BFGS) rather than owning
them — a spike decides that, below. What does not change either way: cuRobo
has no notion of planning *along a constrained seam* rather than to a goal
pose, so the seam-specific layers here — the redundancy graph, the projection
step, the split protocol, the exact-CCD verifier — are this planner's own
regardless of whose ESDF sits under them.

## The shape of the problem

At every point `s` on a seam the program pins 5 DOF — position on `P(s)`,
electrode axis within the admissible bands — and leaves free: torch spin about
the electrode, the positioner joints, and, discretely, which IK solution the
arm uses (up to 8 classes for a 6R wrist-partitioned arm). The feasible set at
one seam point is therefore not a blob but disconnected islands, one per IK
class, each parametrized by spin × positioner. A gradient method seeded in one
island can never cross to another, and the best plan may need a different
island — or a different positioner value — halfway along the seam. So the
planner is two searches with a hard division of labour:

- a **discrete, global** search that decides which islands the trajectory
  lives in, done exactly on a discretization (§ redundancy graph);
- a **continuous, local** refinement that polishes within the chosen basin,
  done in parallel from many seeds (§ refinement).

Neither substitutes for the other. The gradient stage cannot make discrete
decisions; the graph cannot make millimetre-smooth trajectories.

## Pipeline

```
.weldplan ──▶ 1 load       plan request, cell FK model, robot sphere model
              2 bake       part ESDF (1–2 mm, workpiece frame)
                           env  ESDF (≥5 mm,  world frame)
  per seam ── 3 graph      redundancy lattice along s → DP → top-K seeds
              4 generate   batched projected gradient on joint splines
              5 attach     approach/retract from reset pose
              6 polish     SQP feasibility pass, winner only
              7 time       parametrization against velocity/accel limits
              8 verify     exact-mesh CCD + limits                     [T2]
  post ────── 9 transit    replan seam-to-seam links, drop reset detours
             10 emit       trajectory artifact + plan report
```

Stages 3–8 run independently per seam: parallel across seams, cacheable per
seam, and a seam that fails does not poison its neighbours.

## Representation

**Joint-space splines over arm + positioner joints.** The trajectory being
optimized is the robot's, and collision, limits and manipulability all live in
joint space; the seam is realized as constraints on it, not by construction.

> **What the free-space stage actually implements, as of 2026-08-26.** This
> section said "B-splines", and for the seam GEOMETRY that is true -- the weld
> program's rational B-splines are evaluated by `geometry/curves.py`'s de Boor,
> differentiably, exactly as described below. The free-space TRAJECTORY is a
> different thing and is not a B-spline.
>
> `freespace.py` optimizes knot POSITIONS only (`Adam([self.x])`) and derives
> each knot's tangent as a centred difference (`velocities`). A cubic Hermite
> whose tangents come from centred differences is a Catmull-Rom spline: `C1`,
> so acceleration is DISCONTINUOUS at every knot. Measured on one transit, the
> jump reaches 1.73 rad/s^2 -- 0.83 of this cell's own acceleration limit, from
> the seam between two segments alone. The verifier does not see it:
> `check_limits` differences the waypoints it is handed, so it samples the
> knots and steps over the jump between them. Measured across 12 transits, the
> acceleration ratio is 1.04 on the knot grid and 2.05 on the curve.
>
> M5's weld pass IS a genuine cubic Hermite and for a stated reason (commit
> `4c87e65`): the tip velocity at each knot is solved for by DLS and imposed as
> DATA, which drops the between-knot sag two orders to `h^4/(384R^3)`. Free
> space inherited the interpolant without inheriting that -- there is no tip
> speed to match, so the tangents were filled in by differencing. The cost of
> Hermite is carried, the benefit is not used.
>
> Nothing in this repository records a decision to use Catmull-Rom here, which
> is why this note exists rather than a rewritten sentence: the gap is drift,
> not a design choice, and it is being closed (see `docs/issues`).

**The weld phase is parametrized by seam arc length `s`, not time.** The
tracking constraint is then pointwise — `FK(q(s_i))` against `P(s_i)` — with
no correspondence search, and it keeps time out of the optimization entirely.
Time enters once, in stage 7, against the cell profile's velocity and
acceleration limits. Approach, retract and transit
phases are ordinary knot-parametrized joint splines; they have no seam to be
exact against.

**No torque model.** Welding is slow and the URDF is rigid; dynamics beyond
joint velocity and acceleration limits buy nothing. Acceleration limits stay:
they bind before velocity does on any motion with sustained transverse
content, and they are what stage 7 parametrizes against.

## Coordinates

The optimization state includes the positioner, and no world-frame seam pose
ever materializes. Seam constraints are evaluated in the workpiece frame by
pulling the TCP through the positioner's forward kinematics:
`T_work(q_pos)⁻¹ · T_tcp(q_arm)`, composed with the placement. The program's
geometry stays where the contract puts it — in the workpiece frame — and the
positioner moving the part costs nothing special. Collision splits the same
way (§ collision): the part's distance field rides the positioner for free
because it is baked in the part's own frame.

## The redundancy graph

The discrete search is Descartes-style ladder-graph planning, and it is the
same machine `orient.py` already is one level up — DP over a masked lattice
(D15, D17) — with the node meaning swapped.

**Layers** are seam samples `s_0 … s_N` (~5 mm apart). **Nodes** are the
discretized redundancy at that sample: spin bin × positioner bin × IK class.
Each node is one IK solve — the torch pose is fully determined by the program
plus the node's choices — screened cheaply: IK converged, joint limits, sphere
model clear of both ESDFs, `μ_t'` above the profile's floor. Screening batches on
GPU; it is the bulk of the graph's cost. **Edges** connect nodes in adjacent
layers whose joint distance is under a threshold scaled to the step. The
threshold is physics, not tuning: 5 mm of travel cannot slew 30° of spin. It
makes node degree a small constant, kills IK-class flips (a wrist flip is a
large joint jump between adjacent samples, and correctly has no edge), and is
what lets rate limits ride along as edge rules.

**Search** is one Viterbi sweep, `O(layers × nodes × degree)` — linear in
seam length, exact on the discretization. Keeping best-K per node yields K
diverse paths for the price of one. Closed seams make the graph cyclic; run
the sweep from the best few start candidates.

**Bands, two-tier.** The admissible band (D17: absolute limits ∩ authored
profile ± deviation) is a pointwise constraint, so it could enter as node
dimensions — but ±5° of angle rarely changes which island is feasible, and the
extra dimensions multiply node count ~25×. So:

- *Default pass:* angles pinned to the authored profile; bands are left to the
  continuous refinement, which treats them as clip boxes.
- *Rescue pass:* only when the default pass finds no path — re-run with coarse
  angle-deviation bins added as node dimensions, masked to the admissible set
  (honouring D17's half-bin guarantee so the authored profile is always a
  node), rate limits as edge rules. This is the search that discovers "this
  seam crosses s = 0.43 *if* work angle gives up 4° through the corner" —
  which is precisely the trade the band exists to permit, found globally.

Every node is admissible by construction, so discretization can never sneak an
out-of-band weld into a seed; the verifier re-checks the achieved angles
exactly regardless.

**Positioner dimensionality.** With one live positioner axis the full spin ×
positioner product fits in the graph (~10³–10⁴ nodes/layer) and mid-seam
coordinated repositioning comes free. With two the product is ~10⁵/layer;
hold the positioner per seam as an outer loop over candidate values — which
also matches shop practice — and admit it into the graph only for seams that
demonstrably need mid-weld repositioning.

**Live is not the same as on the chain**, and `rosie_1400_v3` is the worked
example. Its work frame hangs off two revolute axes — `J9` turns the H-frame,
`J7` tilts table A — and a third, `J8`, tilts table B. But J9 is a *station
index*: it swings the loaded bay to the robot and the empty one to the
operator, who fixtures the next part while this one is welded. During a weld
it is held. So the plannable state is six arm axes plus J7, and the null
space is two-dimensional, which is the case the graph carries whole.

The other axes do not vanish, and this is the trap: J9 and J8 take no
decision, but their VALUES are still needed. J9 because the work frame's
pose depends on it, so a wrong value puts the part somewhere it is not; J8
and everything below it because bay B — with a part clamped to it and an
operator beside it — is real geometry the arm must not hit. They enter the
plan as held values, not as variables.

Which axes are which is partly structural and partly a human fact. The
chains derive it: axes between base and tool are the arm, axes between base
and work are the work chain, and anything on neither belongs to the cell but
not to this weld. What no descriptor can say is that J9 within the work
chain is an index rather than a degree of freedom. That distinction lives in
the cell profile below.

## The cell profile

Not a file: a MERGE, computed per plan from the robot's description and the
target cell's `machine_planning_calibration.json` (`docs/ROBOT-DESCRIPTION-CONTRACT.md`). It used to
be this package's own asset, `data/motion/cells/<cell>.profile.json`, which
made the planner a second place a robot was described. It carries what a URDF
does not and what the weld program must not:

- **Axis roles.** Which axes are the arm, which single axis (or axes) the
  planner may drive as a positioner, which are held, and at what values.
  From `robot_description/<model>/config.json`; a cell may hold a positioner axis of
  its own in `machine_planning_calibration.json`.
- **Velocity and acceleration limits.** A URDF `<limit>` carries position and
  a velocity RATING; stage 7 parametrizes against a planning rate that may be
  lower (`config.json`) and T2 checks acceleration, which the format has no
  field for at all. What the cell ENFORCES — the ceiling the dense header
  declares — is the cell's own number, from its `machine_planning_calibration.json`.
- **The reset pose.** Every seam is planned from it and back to it.
- **The sphere model**, per link, from `robot_description/<model>/spheres.json`, plus
  the torch's, fitted from `tooling.json`'s analytic dimensions.

Structural facts are derived and only overridden here, never restated:
a description that had to list the arm's axes would be a second description of
the URDF, and the two would drift. The same argument moved these files beside
the URDF in the first place.

**Failure is a result.** When no path crosses all layers, the sweep still
reports the reachable frontier — the furthest layer any path attains — and,
at the frontier, why every next-layer node died (collision vs joint limits vs
`μ_t'` vs band), as counts. That diagnosis drives the split protocol below and
is the planner's equivalent of `access.py` reporting blocked spans rather than
failing.

## Continuous trajectory generation (formerly "refinement")

Renamed by the owner, 2026-08-13: the stage's input is a discrete lattice
path and its output is the continuous executable trajectory, and the old
name hid the actual deliverable. Everywhere else in this document the short
form "trajectory generation" means this stage.

Batched gradient descent on the spline coefficients, K seeds per seam from the
graph, all seams and seeds in one tensor. Constraints are handled by kind, not
uniformly:

- **Seam tracking (the 5 pinned DOF): projection, not penalty.** After each
  gradient step, 1–3 damped-least-squares Newton corrections per sample pull
  the tip back onto `P(s)` and the electrode axis back toward legality, then
  the spline is refit through the corrected samples. Each correction is a
  small batched linear solve — parallel in exactly the way a global QP is not.
  Penalties leave millimetre-scale tracking error at convergence, which is the
  scale that matters here; projection makes the constraint effectively hard
  while the optimizer explores only the null-space motion — spin, positioner,
  posture — which is the real search space anyway.
- **Bands and rate limits: clip.** In the space the projection works in, these
  are boxes and pairwise bounds on the achieved angles, evaluated through the
  same FK → seam/rail mapping the verifier uses. Exact and free.
- **Collision and `μ_t'`: augmented Lagrangian on the ESDF.** Not fixed
  penalties — per-constraint multipliers grow until violation actually reaches
  zero instead of plateauing at "small". This is the only place AL is used;
  everything harder is projected or clipped, everything softer is cost.

The winner then gets a small SQP feasibility polish (stage 6) — serial is fine
for one trajectory — so what reaches verification is not "almost feasible".

Nothing in this stage may import `seam_worker`. The program's rational
B-splines are evaluated by the planner's own de Boor implementation, written
once, differentiably, so seam positions, tangents and frames sit inside the
autodiff graph end to end.

## Collision: search vs proof

The rule that decides what is a sphere and what is a field: **spheres are
for the body being checked; fields are for everything it must not hit.**
Only the arm chain and the torch are spheres. Everything else in the cell —
however it moves in world coordinates — is rigid within SOME frame once the
held axes are held, and each such rigid group gets one field in its own
frame:

- **workpiece group** — the part, AND the table, clamps and fixtures bolted
  to the same positioner table: rigid with respect to the part by
  construction, so they bake into workpiece-frame fields and ride the
  positioner for free. Two grids in that one frame, because one resolution
  cannot serve both: fine (1–2 mm, tight box) for the part, coarse (~5 mm)
  for the table and fixtures. Distance to the union of two bodies is the
  min of the two sampled distances, so the pair costs one extra lookup.
- **world group** — floor, pedestal, H-frame, and the second bay with
  whatever is clamped to it (held axes are held for the length of a weld):
  the env field, ≥5 mm, world frame. That the operator-side bay is covered
  without a single sphere is not incidental; it is the group assignment
  doing its job.

Group membership is DERIVED, not configured: a link with no driven axis on
its path to the work-frame link is workpiece-group; none on its path to the
base is world-group; what remains is the arm chain, and that is what gets
spheres. `cell.axis_roles` plus the profile's driven/held split already
carry everything the derivation needs, and a change to which axes are held
re-derives the groups rather than invalidating a hand-made list.

**Workpiece against world is a one-dimensional question, answered once.**
The two field groups can collide with each other -- a tall part swinging
into the H-frame, the table dipping toward the floor -- and neither is
spheres. But their relative pose depends only on the DRIVEN positioner
axes, which during a weld is J7 alone: whether the part hits the world is a
function of one angle, whatever the arm is doing. So it is precomputed at
bake time as a feasibility mask -- sweep J7 over its range (~0.5° steps),
transform a surface sampling of the workpiece group by each angle, query
the env field, keep the angles that clear by a margin sized to the sweep's
own discretization. ~10 ms per plan request at measured query rates. The mask
then enters the pipeline where masks already live: node generation in the
redundancy graph refuses positioner bins outside it (the `_allowed_bins`
pattern), refinement clips J7 to it as a box constraint, a mid-seam
positioner trajectory must stay inside it pointwise, and the held J9 gets
the same check once as a scalar. Two driven positioner axes would make it a
2D cell grid and still cost milliseconds. The CCD verifier re-checks the
winning trajectory, positioner motion included, against exact meshes --
the mask is a search device, not the safety argument.

Sphere centers transform into each field's frame per query, and those
transforms MUST be torch ops on the state tensor, never precomputed
constants. The gradient structure is asymmetric on purpose -- against the
env field `∂d/∂q_pos` is genuinely zero (the arm does not move in world
when the positioner turns), while against the workpiece fields the same
derivative flows through `T_work(q_pos)⁻¹` and is the signal that lets the
optimizer rotate the part out of the way instead of contorting the arm.
A work transform built outside the autograd graph kills exactly that term,
silently: every value stays correct, no error is raised, and the planner
degrades into one that never uses the positioner to dodge. Same failure
family as the fused kernel's crossed y-derivative -- right values, wrong
gradient, invisible without a gradient test -- so the sphere-transform code
lands with a finite-difference check that `∂cost/∂J7` is nonzero through
the workpiece field and zero through the env field.

The honest gap
in all of this is data, not representation: the fixture fields bake the
cell model's NOMINAL clamp poses, and real clamp placement varies per part.
That is a cell-profile input (or a keep-out margin over the fixture zone),
and it would be exactly as unsolved under any other collision scheme.

**The bake.** The scene is known CAD, not sensor data, so none of cuRoboV2's
depth-fusion machinery applies — the bake is offline, from meshes, and can
afford exactness where it matters:

- *Tessellate* the plan request's STEP per solid (this package's own OCCT
  dependency — the plan request deliberately ships geometry, not meshes).
- *Narrow band exact:* within ~±16 voxels of any surface, signed distance by
  closest-point-on-mesh queries against a GPU BVH (Warp `wp.Mesh`,
  winding-number sign so imperfect solids don't flip the field).
- *Far field propagated:* beyond the band, an exact 3D distance transform
  (PBA+, as cuRoboV2 §5.4 uses) from the band's surface voxels — its
  ~voxel-sized quantization error is irrelevant at distance, and the near
  field it would corrupt is owned by the exact band.
- *Storage:* the part grid is dense fp32 — a 300 mm-class part at 1 mm is
  tens of MB, so cuRoboV2's block-sparse hashing (built for room-scale
  mapping) buys nothing here. The env grid at 5 mm is dense fp16, a few
  hundred MB; block-sparse only if a cell ever outgrows that.
- *Queries:* trilinear sample minus sphere radius, gradients by autodiff
  through the interpolation. Sphere-sphere self-collision beside it,
  cuRobo-style; part-vs-environment during positioner motion via a coarse
  sphere cover of the part against the env field.

The ESDF serves the *search only*. Sphere-vs-field error is the same order as
the millimetre clearances a weld runs at -- measured on this cell, 0.023 mm
mean but 1.6 mm worst against the exact mesh -- so the final verifier is
exact-mesh **continuous** collision detection over the plan request's STEP and the
planner's own cell geometry, plus joint position/velocity/acceleration limits.
Fail closed; nothing ships without it. This is T2. The directional-mobility
floor maintained through graph and refinement is T1. T3 and T4 are per-cell campaigns outside
this pipeline.

**The verifier implements that CCD rather than calling one, because none of the
candidates has one that works on meshes.** The spike is `tools/spike_ccd.py`
and the table is `BENCHMARKS.md` §8. Bullet, through pybullet, declines every
mesh-vs-mesh pair -- convex ones too, so it is not a concavity limit -- while
answering mesh-vs-primitive, which is what makes it a hard no rather than a
slow yes. python-fcl and coal are both exact and agree to 0 mm on the real
cell meshes. python-fcl also ships `continuousCollide`, which computes a
correct time of contact for a sphere pair and returns *no collision* for two
meshes that certainly overlap: an API that fails open on the one job it exists
for.

So `verify.py` does conservative advancement itself. For a joint-space segment
it bounds how fast any point of a link can move -- `sum |dq_j| * R_j`, with
`R_j` the reach of joint `j`'s axis over everything distal to it -- and that
Lipschitz constant turns a single query at an interval's midpoint into a
statement about the whole interval. Where the statement fails the interval
halves; where it holds, nothing on it can touch. The primitive underneath is
coal's `security_margin` predicate, "is anything closer than m", which stops
at the first bounding-box pair that settles the question and costs 9-14 us
against 228 for the exact distance. The exact distance is computed once per
verdict, for the report, and never inside the loop.

**coal** is therefore the backend: a tie with python-fcl on exactness and
throughput, a win on the margin predicate and on a `distance_lower_bound`
verified sound over 4000 poses, and on conda-forge rather than PyPI. Measured
on `rosie_1400_v3`: 1.4 ms per segment across 54 checked pairs.

The same argument, and largely the same machinery, then answers a second
question the verifier is asked: does the electrode stay on the seam BETWEEN the
samples. `certify_tracking` swaps the pair of bodies for a pair of points --
the tip and where the tip belongs -- and the mesh query for a distance. The
trajectory between two knots is a cubic Hermite, so its derivative is a
quadratic with closed-form extremes; the reach radii turn those into a speed
bound for the tip and, through the positioner, for the target; the seam side
is an exact curve whose own rate is bounded in closed form on a straight
segment. One distance at an interval's midpoint then bounds the error over
the whole interval, and intervals are cut at the weld line's corners first,
because the target for a torch standing off the work genuinely jumps there.
What this buys is not reassurance but a correction: on the corner seams of
`bracket_a_placed` the optimizer's sampled tracking number understates the
true worst by up to 4.75x.

## Seams the robot cannot weld whole

Splitting one pass into two is an extra strike, a crater, a restart overlap —
a change to the *weld*, not the motion, and D8 says one seam is one pass. The
same rule the seam model states for weave applies: infeasibility is a planning
failure to report, never a parameter to renegotiate silently.

So the planner **proposes and never decides.** The proposal fires only after
it has exhausted everything it legitimately owns — full spin range, positioner
motion, start point on closed seams, and the whole admissible band via the
rescue pass — and it arrives with evidence: the frontier `s`, the cause
histogram at the frontier, proposed cut points (read greedily off the DP:
frontier, restart, repeat — minimal segments, each with a certified-feasible
sub-plan attached). The cause histogram matters because the right fix is often
not a split at all but a placement change, and the operator can only make that
call if told *why* the frontier is where it is. Acceptance happens upstream,
in the authoring tool, with `span.py`'s arithmetic; a revised program comes
back as a new plan request. The loop is diffable because plan requests are
deterministic bytes.

## Between seams

Every seam is planned reset-pose to reset-pose. That makes seams independent —
parallel, cacheable, individually replannable — at the price of detours, which
stage 9 then removes: for each consecutive pair in the executed order, replan
end-of-weld to start-of-weld directly and keep the shorter of {direct, via
reset}. Transit costs computed this way are also the edge weights for
sequencing-as-TSP, if seam ordering ever becomes the planner's to choose;
today order is taken as given.

## The response artifact

`MOTION.md` fixed what enters the planner. What leaves it is the other half of
the contract and is designed with it: one response per request, carrying per
seam either a trajectory or a structured infeasibility record (frontier,
cause counts, proposed cuts with sub-plans), plus the verification verdicts
(T1 margins along `s`, T2 pass with the geometry actually tested, per
`WEAVE-AND-VERIFICATION.md`'s warning), the achieved angle profiles against
their bands, and the timing with the reserved β. Deterministic bytes, same
argument as the plan request. Trajectories target the repo's existing
`urdf-viewer-trajectory-v1` shape for playback. A verified plan is reported as
what it is — kinematically feasible and collision-free as modelled — and
never as evidence of weld quality (§6 of the weave document).

## Speed: parallelism and the GPU

The pipeline is designed for end-to-end latency on one workstation with a
strong consumer GPU (currently an RTX 5090). The design rules, in order of
how much they buy:

**Batch across seams, not just within a seam.** Stages 3–5 are one tensor
program each: every seam × every seed × every sample in a single batch. A
lone seam's refinement cannot saturate 20k cores; a plan request's worth can.
The per-seam independence that made failures isolated is the same property
that makes this legal.

**The CPU/GPU boundary follows the search/proof boundary.** Everything
exploratory is GPU tensor work: ESDF baking (stage 2), the graph's batched
IK + screening (stage 3, ~10⁶ small Newton solves), the projected-gradient
refinement and free-space attach (4–5). Everything certifying is CPU:
the SQP polish, timing, and above all the exact-mesh CCD verifier — the weave
document already argues GPU collision approximations are wrong at weave
clearance scale, and CCD segments are embarrassingly parallel across a
24-core pool instead. This split also enables overlap: the CPU pool
verifies seam A while the GPU refines seam B; the stage graph is a per-seam
DAG executed async, with no global barrier between stages.

**fp32 on the GPU, fp64 where it counts.** Consumer GPUs run fp64 at 1/64
rate, so all GPU stages must tolerate fp32 — which they do, because they
only need to land in the right basin with ~10⁻³ accuracy. The polish and the
verifier run fp64 on CPU, and the verifier's tolerances are the ones that
bind. A seed that fp32 mis-ranks costs a slightly worse start, never a wrong
answer; the verifier is exact regardless.

**Stay resident.** Between stages 3 and 5 nothing leaves the GPU but
scalars and, at the end, the winning trajectories. The DP sweep itself runs
on-device (a per-layer scatter-min; 160 sequential kernels of trivial size,
CUDA-graph-captured to kill launch overhead) purely to avoid the round trip
after screening.

**Cache by digest.** Plan requests are deterministic bytes, so every expensive
artifact is content-addressed: ESDF bakes keyed by (STEP digest,
resolution) and (cell id, resolution); per-seam plans keyed by (seam,
placement, cell, tooling) digests. An operator edit that touches one seam
replans one seam. This is the single biggest end-to-end win in the
authoring loop, and it costs nothing because determinism was already a
contract requirement.

**Frameworks.** PyTorch is the differentiable core — FK, curve evaluation,
projection, AL refinement — chosen over JAX for the cuRobo lineage
(known-good batched-optimization patterns to port from experience, not
code) and easier debugging; over raw CUDA because none of these kernels are
exotic. NVIDIA Warp handles geometry: mesh-BVH closest-point queries for
the ESDF bake and any narrow-phase sphere-mesh checks, zero-copy into torch
via dlpack. If profiling later shows the projection loop is
launch-overhead-bound, the escape hatches are torch.compile, CUDA graphs,
or a Warp kernel — an implementation swap behind a fixed interface, not an
architecture change.

Order-of-magnitude budget, to be held honest by a benchmark fixture: bake
1–2 s cold and free warm; graph well under a second per plan request batched;
refinement a few seconds for all seams; verification minutes → seconds on
the CCD pool. Target: tens of seconds cold for a full plan request, one to two
seconds to replan a single edited seam.

## Home and dependencies

Per `MOTION.md`, first home is `python/weld_motion_planner/` in this repository with
its own pixi environment (torch for batched FK/IK/autodiff; Warp for GPU
mesh queries and ESDF baking; a mesh/CCD library for the verifier; `weldplan`
as its only in-repo import, and the test suite already enforces that
arrow). It moves to its own directory the day the
environment split hurts; the plan request makes that move free, which was the
point of the plan request.

## Ignoring the pairs that always touch

Two links joined by a joint touch. That is what a joint is, and their
sphere covers overlap at the mating face by construction -- a cover that
stopped short of it would leave the real link uncovered. So an adjacent
pair reads as a collision at every pose, including the reset pose, and a
feasibility test that always says infeasible is not a test.

The rule is structural and is derived from the tree in `weld_motion_planner.robot_cell.collision.pairs`,
not written down. A list would be a second description of the tree, and the
one that goes stale.

**Robot against robot** is a sphere-versus-sphere test and the ignore is a
mask over link pairs: adjacent, and each link against itself. Not built
yet; the mask is.

**Robot against the environment** cannot use a pair mask, because the
environment is a distance FIELD and a field does not know which mesh each
voxel came from. "link_1 ignores floor" is not expressible against one grid
that has the floor in it. Two ways out:

- Bake the environment in GROUPS, one per thing a link might need to
  ignore, and mask which groups each link queries. Precise. Memory is a
  wash on this cell -- 121 + 67 MiB against 198 for the union at 5 mm,
  since the union's box is mostly the empty space between a wide flat floor
  and a tall narrow frame -- but every other link then pays two field
  queries where it now pays one, and the field round trip is already the
  largest piece of the collision term.
- Drop the adjacent link from the environment check entirely. Cheap, and
  it costs coverage.

`rosie_1400_v3` takes the second, because the cost is measurable and
measured: `link_1` turns about a vertical axis at the base, and sweeping
J1 x J9 x J8 over 13 824 configurations against the exact meshes puts its
closest approach to the H-frame and to bay B at 355 mm. Nothing is lost by
making it invisible to a field it was never going to reach. A cell where
that number comes out small wants the group split instead, which is why the
adjacency is computed rather than assumed.

The environment is defined as what is NOT on the arm chain -- not as
whatever carries no spheres. `end_effector` and `tool0` are arm links that
nobody has covered yet, and treating "uncovered" as "environment" made
`link_6` adjacent to the environment and silently dropped fifty wrist
spheres from the check.

## Deliberately not decided yet

- **Depend on cuRoboV2 or own the commodity layers.** Now that V2 is Apache
  2.0, its world model, ESDF pipeline and batched L-BFGS are candidates to
  adopt instead of reimplement. A spike decides: adopt if its components
  compose under our seam-projection loop without fighting the API; own them
  if the coupling costs more than the ~2k lines they replace. The
  seam-specific layers are ours either way. (Verify the asset licenses
  shipped beside the code before depending on them.)
- ~~The CCD backend (Bullet direct vs python-fcl)~~ — **settled by
  `tools/spike_ccd.py` against the real cell meshes. coal (hpp-fcl's
  continuation), and no swept-volume CCD from anyone: see below.**
- Whether refinement seeds also get a particle scatter on top of the graph's
  K-best — insurance to add only if basins are demonstrably being missed.
- The response artifact's schema id and field names — designed alongside the
  first consumer, as the plan request was.
- Sphere model authoring for the cell — per-cell asset of this package; format
  chosen when the first real cell is fitted.
