# Parked test claims

Removed while `.weldplan` moved from `amr-weld-planner-v1.weld-program.v1` to
`robot.v4.program.v2` (D24). Every file below loaded a `data/motion/*.weldplan`
fixture, and those fixtures were still v1: `read()` returned no placement and
no seams from them, so the tests failed for the format rather than for the
claim.

**That blocker is gone.** The corpus was rebuilt as program.v2 on 2026-09-11
(`tools/make_motion_fixtures.py`): `bracket_b_2x` carries 22 seams over 6
joints, and `read()` refuses a pre-v2 program outright rather than opening it
empty. So these can be restored -- but restoring is not `git checkout`.
Measured on the first file brought back, `test_request.py`: 22 of its 29
assertions passed unchanged and 7 needed the v1 vocabulary rewritten (the
schema name, `placement.json` as a container member rather than a field
inside the program, and one assertion that deliberately demanded the ABSENCE
of an admissible band). Expect that ratio per file.

They were deleted rather than skipped so nobody reads a long skip list as
coverage. This is the list of what they asserted, to be written again against
program.v2 fixtures. Restore the originals from the commit that removed them
if the wording of a claim matters more than the sentence here.

Not parked: `tests/shared/test_plan_request.py` was migrated in place, because
it tests the container itself and the container is what changed.


**288 tests across 15 files still parked** (test_request.py's 11 and test_tracking.py's 9 are restored; the tracking file came back with `docs/issues/0008`, its curved-seam claims inverted and its fixture seams moved to two that plan on today's cell).


## `motion_planner/test_control_solver.py` — 5

> The control-point solver's contract, on the corpus it will actually fly.

- **no motion exceeds a rate limit anywhere on the curve** — **The contract `docs/issues/0015` exists for.**
- **retime moves a horizon in whichever direction it needs** — `retime` may LENGTHEN as well as shorten, and both are exercised.
- **every pinned end is a real stop and not a declared one** — A weld-side end holds `PIN_CONTROL` control points, and for a clamped
- **the endpoints are exact because the knot vector is clamped** — Position at the ends is DATA -- a weld pose, the reset pose. A clamped
- **acceleration is continuous along every solved motion** — `C2`, on a real answer rather than a random control polygon.

## `motion_planner/test_curves.py` — 18

> Curves and frames, against the authoring side's own answers.

- **the seam lands where the authoring side says** — Position, which is the constraint the optimizer holds to millimetres.
- **the span maps to the same place on the joint** — A seam is a span of its joint's weld line, and on a closed loop the
- **the frame triad matches** — `t`, `r` and `b` together, joins included. `r` is the datum work angle
- **a span end on a corner is the case the fixtures carry** — The parity tests above are only worth their name if the samples they
- **the frame at a corner follows the direction of travel** — The convention both sides hold, on a synthetic L so the claim does not
- **the torch direction matches at every angle pair** — `u` over the whole angle table, including negative work and negative
- **curvature matches** — From the analytic second derivative, never from differences over a
- **the seam length matches**
- **the oracle covers a rail that steps at corners** — A guard on the guard. Per-segment rail correspondence only matters
- **arc length sampling is actually uniform** — Equal `fraction` steps mean equal DISTANCE, which is the defect the
- **gradients flow from the seam point** — The reason this is torch and not numpy: the optimizer differentiates
- **the transverse axis is perpendicular to travel and electrode** — `WEAVE-AND-VERIFICATION.md` §2: the weave axis falls out of the frame
- **float32 lands in the same place to search accuracy** — `ARCHITECTURE.md` puts fp32 on the GPU because a consumer card runs
- **a rational circle has the right circumference** — An oracle can only say two implementations agree. A circle knows its
- **a malformed segment is refused rather than evaluated**
- **an empty curve is refused**
- **a wrapped span maps its ends exactly** — `start + 1 * length` lands an ulp past `end_s`, and a seam's end is
- **self test entry points pass**

## `motion_planner/test_esdf.py` — 13

> The distance field, and the mesh it is baked from.

- **the field matches a closed form**
- **inside is negative** — The convention every margin depends on: a sphere of radius R clears an
- **the field keeps growing outside its box** — Clamping would report the boundary's distance everywhere beyond it --
- **outside the box clearance is understated never overstated** — The extension is a LOWER bound by construction: max of two quantities
- **the interior survives the outside extension** — The extension term is zero inside the box, and a bare max against it
- **the gradient is a unit vector pointing out** — A distance field's gradient has unit length away from the surface, and
- **a grid too large to bake is refused with a number** — Rather than allocating it and dying on an out-of-memory the caller
- **the part tessellates to its declared size** — Metres, not millimetres. A mesh a thousand times too big does not look
- **the part has an inside** — A solid whose winding came out inverted would bake a field that is
- **the seam lies on the surface the field was baked from** — The one test that catches a frame mismatch.
- **fused sample matches the reference** — `sample_fused` is `sample` with the gradient moved into the kernel,
- **fused backward runs no second kernel** — The reason the fused path exists: backward must be a buffer read.
- **self test entry points pass**

## `motion_planner/test_freespace.py` — 20

> M6: the moves between the welds.

- **the visit order is the short way round not the authored one** — `order_seams` reorders the WELDS, not the welding: same reports, each
- **the duration comes from the rate limits and nothing else** — Free space is the ONE place in this planner where timing is a choice,
- **the ruler for wasted motion is time and not radians** — `detour_of` is the number the shortcut pass exists to move.
- **a retime only ever shortens and stops at the limit** — `time_scale` is closed form, one-directional, and floored.
- **the plan is joined end to end** — Approach, then a transit between every consecutive pair, then a
- **every handover is a held stop** — The contract this module exists for: a handover is a STOP.
- **the ends of the plan are the cells reset pose at rest** — The arm leaves home stationary and comes back stationary. Anything
- **every link is collision certified and rate classified exactly** — M6 skips only weld travel mobility; its Hermite rates still bind.
- **a link never leaves slower than it arrived** — The retime is one-directional, and this is the property to pin.
- **no link carries motion nobody asked for** — The complaint this pass was written for, as a number.
- **no link is flown at half the rate the cell allows** — The retime, and the accept test that keeps it honest.
- **the shortcut collapses to the straight line with nothing in the way** — The pass in isolation: no scene, so every span screens clear.
- **the screen that ranks seeds can see the arm hitting itself** — Both clearances are measured, and this was a bug.
- **rewinding a weld changes the command and not the robot** — The premise the whole fix rests on.
- **a winding that leaves an axis out of its limits is not offered** — `J6` runs to +-573 degrees on this cell and `J2` to +-108.9. A turn
- **on the measured rates the unrewound approach clears the part** — The counterfactual this test used to pin -- M4's winding carries `J5`
- **the roadmap keeps the nearest winding when it can reach it** — `bracket_a_2x seam_0001` is the case that forced the roadmap into the
- **the reach a roadmap must cover includes windings not yet chosen** — Chicken and egg: the roadmap's box is cut to what is being joined, and
- **a link on the wire is playable by itself** — `move_json` mirrors `weld_trajopt.trajectory_json`: knots and velocities in
- **a link refuses to plan without a reset pose** — `connect` cannot invent where home is, and a planner that defaulted it

## `motion_planner/test_graph.py` — 42

> Edges and the sweep.

- **the best path is continuous feasible and within its threshold** — Re-derived from the lattice, not read off the sweep's own bookkeeping.
- **the reported cost matches the path it names** — The two are computed by different code -- one accumulates during the
- **a joint wider than a turn may wind rather than unwind** — `J6` spans 1146 degrees on this arm, so a posture at -179 is equally a
- **the wound path is what the arm executes** — The lattice keeps canonical values; the winding is resolved once, on the
- **the k paths are actually different** — The check that found both state bugs. K near-identical paths are worse
- **the positioner is part of the search state** — Two nodes with the same arm posture and different workpiece angles are
- **a positioner slew past the threshold has no edge** — The same trade, refused. If the positioner were absent from the state
- **a closed positioner does not bin the same angle twice** — J7 spans a whole turn, so -180 and +180 are one orientation. Binned
- **a threshold nothing can cross reports a frontier with causes**
- **a corner is reported as the seam s demand not the arm s failure** — The distinction an operator acts on, and the one a cause histogram
- **a straight seam that crosses reports no demand** — `frontier_demand` is `None` when nothing stopped, so a caller cannot
- **a curved seam is not called a corner** — A seam that turns a little at every sample turns a little at the
- **the frontier is where the reachable count hits zero**
- **a wider threshold never reaches less far** — Monotonic by construction -- every edge a tight threshold allows, a
- **a cell without rate limits cannot derive a threshold** — Not `rosie_1400_v3` any more -- its profile states rates since
- **rates and a step give a threshold**
- **the result records whether the threshold was derived**
- **a non positive threshold is refused**
- **one layer is not a graph**
- **the whole corpus crosses except the one that is the arm** — M4's own done-when, asked of every committed plan request rather than of the
- **a positioner binned wider than the threshold is never used** — `docs/issues/0004`, asserted on the mechanism rather than on a symptom.
- **offsetting the grid does not substitute for sizing it** — The proposal was probabilistic completeness: offset the grid a random
- **the edge rule comes from the cell rather than from a caller** — The threshold was STATED for the whole of M4 and `Graph.derived` recorded
- **a derived rule and its bins still cross the corpus** — The cell's measured rates (0.2 rad/s on the positioner, 2026-09-07)
- **bins are sized so neighbours are strictly inside the threshold** — Strictly inside and not equal to. The edge test is `<=` per component,
- **an axis with one bin is not reported as pinned** — A single bin is a choice not to use an axis, not a search that cannot.
- **a seam that crosses does not climb the ladder** — The point of escalating is that the common case never pays for it. The
- **the ladder climbs and rescues a seam the first rung cannot** — The rescue, CONSTRUCTED, because the corpus keeps outgrowing it.
- **the ladder stops when halving the grid does not move the wall** — The stopping rule, CONSTRUCTED rather than hunted for.
- **a rung that will not fit is reported rather than attempted** — Predicted, not caught. An out-of-memory tells the caller that something
- **the predicted transition size matches what the search allocates** — `transition_bytes` decides whether a rung runs, so a prediction that
- **pruning by positioner bin changes no cost** — The positioner is a state component with its own threshold, so
- **pruning survives a band that wraps** — The positioner closes, so a band of bins runs off one end and back on at
- **chunking the transition changes nothing but the peak** — `topk` runs per destination over all sources, so slicing the destination
- **the same seam searched twice gives the same answer** — `ARCHITECTURE.md` asks the whole graph to be deterministic from a seed,
- **the trace is off unless it is asked for** — It costs `L` times the running state, and the sweep was written not to
- **walking the traced back pointers reproduces the reported path** — The claim `/dp.html` is built on, checked rather than trusted.
- **the traced table agrees with the cost that was reported** — The last layer's cheapest slot IS the answer's cost. Anything else means
- **the pair counts nest the way the prunings claim** — `all >= kept >= edges`, per transition.
- **the trace stops where the search stopped** — A seam that fails leaves a partial table, and the page zips the four
- **the placed request crosses with the whole cell screen on** — The corpus test runs unscreened; this is the screened counterpart, on
- **self test entry point passes**

## `motion_planner/test_ik.py` — 20

> Inverse kinematics: joint angles from a torch pose.

- **rotation error is zero for equal rotations**
- **rotation error recovers a known axis and angle** — Constructed, not compared against a second implementation.
- **rotation error stays accurate at tiny angles** — Where a converging solve spends most of its iterations, and where the
- **a solution reproduces its target when run forward** — The claim, checked against forward kinematics and not against the
- **seeding at the answer converges for every pose** — Separates "the solver is broken" from "the seed was in the wrong
- **no solution leaves the joint limits** — Clamped every step, so the answer is executable by construction. A pose
- **more seeds find more solutions** — The measured shape of the seeding problem, asserted so it cannot
- **iterating longer does not rescue a bad basin** — Why the answer to the last few percent is seeds and not patience. If
- **the branch seeds reach genuinely different postures** — Otherwise the graph has no islands to choose between and the whole
- **branch seeds are reproducible** — `ARCHITECTURE.md` asks the graph to be deterministic from a seed, and
- **random seeds are reproducible from a generator**
- **random seeds stay inside the limits**
- **best of takes the first that converged**
- **a cell missing a joint limit is refused** — Defaulting to +-pi would silently give or take travel, and every solve
- **a mismatched seed is refused**
- **a mismatched batch is refused**
- **seeds must be three dimensional**
- **the wrist is spherical which is what caps the branches at eight** — `branch_seeds` returns eight and the lattice's fourth axis is eight long,
- **no pose holds more than eight postures** — The bound, exercised. Many random restarts per pose, and the distinct
- **self test entry point passes**

## `motion_planner/test_ik_closed.py` — 8

> The closed form, against the solver it is meant to replace.

- **the arm meets the premises and says by how much** — `form_of` returns `None` for an arm it cannot solve, so a form coming
- **an arm that breaks a premise gets no closed form** — Each premise, broken one at a time. A closed form that kept working on
- **every closed form solution reproduces its target** — Checked forward, never against the algebra that produced it.
- **the closed form is exact against double precision** — The question that found the bug: **why would a closed form not match
- **the closed form and the solver find the same postures** — The set, not just the members. A branch that quietly duplicated another
- **the branch index names a posture family** — What `docs/issues/0002` asks for. The iterative solver's branch is the
- **the closed form is faster than iterating** — The reason to have it. Asserted as an order of magnitude rather than a
- **self test entry point passes**

## `motion_planner/test_mobility.py` — 16

> Directional mobility, and the Jacobian under it.

- **the jacobian differentiates the kinematics** — Finite differences of `tip`, not a second derivation of the columns.
- **the angular rows are the joint axes** — All three axes are +z here, so every angular column is +z at every
- **the jacobian carries a gradient** — It has to be differentiable to be a cost term. A version that fell back
- **fully extended the arm cannot move along itself** — The textbook singularity, and the one case where the right answers are
- **yoshikawa cannot see what the directional index sees** — The argument the module opens with, as a measurement on the REAL arm.
- **a direction orthogonal to the collapse keeps its mobility** — What `det` cannot express and an SVD can. At the same singular pose
- **mu is the reciprocal of the joint rate norm** — The two outputs are one object seen twice; if they ever disagree, one
- **the joint rates actually achieve the direction** — `J^+ d` is only meaningful if `J (J^+ d) = d`. Checked by pushing the
- **a singular pose does not produce nan** — The blow-up is the signal; `nan` is not. A `nan` here would poison a
- **the per joint form is stricter than the scalar one** — And measured where the limit BITES. At welding speed both accept
- **headroom crosses zero exactly at the limit**
- **missing rate limits are not feasible** — The same rule the verifier follows: not checked is not passed.
- **a singular pose is never feasible**
- **a mismatched direction batch is refused**
- **a cell with an unmodelled moving joint is refused** — `fk.build` folds anything that is not revolute into the fixed origin.
- **self test entry point passes**

## `motion_planner/test_nodes.py` — 51

> The redundancy lattice.

- **the tool axis lands on the electrode** — THE property, and it is a measurement about the robot rather than a
- **any tool axis lands on the electrode** — Not just the default. A cell that measures its torch onto some other
- **the pose is a rotation at every spin** — Orthonormal and right-handed. A drifting frame would make IK chase a
- **spin turns about the electrode and nothing else** — The reason spin is free: it moves the arm and leaves the weld alone.
- **an electrode along the reference axis still gives a frame** — The spin zero has to be picked from some reference, and a fixed choice
- **a feasible node actually reaches its weld** — Re-derived from the stored joints through forward kinematics, not read
- **a feasible node is inside the joint limits** — The lattice solves UNCLAMPED so the causes stay separable, which means
- **every branch is kept rather than collapsed** — The measurement that forced this. With one solution per cell, 0.19% of
- **duplicate branches are named rather than counted as choices** — Eight seeds do not mean eight postures. A branch that landed where an
- **the causes are separable and all appear** — A screen that reported one cause for everything would pass every other
- **a joint limit rejection really is outside the limits** — The cause has to mean what it says. A node labelled `joint limit` must
- **a no ik rejection really did not converge**
- **the shape matches what was asked for** — Four dimensions, not three. The IK branch is a node dimension because
- **the positioner bins span its range without repeating an angle** — Starts at the lower limit, and stops short of the upper one when the
- **an open positioner keeps both of its ends** — The rule is `endpoint=False` only when the axis closes. An axis with a
- **spin covers the circle once without repeating its ends** — `endpoint=False`: -180 and +180 are the same posture, and including
- **a whole bin of offset is refused rather than reduced** — An offset is a fraction of ONE bin, and a bin or more is not a shifted grid.
- **the plan span and the joint limit intersect narrowest first** — Whichever binds. The plan is held to +-90 degrees by default -- the
- **a capped positioner keeps its cap and its spacing at every offset** — The three things clipping broke, out to the last offset allowed.
- **a positioner given the whole turn is shifted rather than squeezed** — The other branch. A whole turn has the same orientation at both ends, so
- **spin stays a canonical turn at every offset** — Spin closes by construction, so its offset has the same bin of slack and
- **the lattice declares the limit it actually enumerated** — `limits_rad`'s last row is the POSITIONER's, and it has to carry the
- **the lattice welds at the angles the program authored** — `docs/issues/0005`, which nothing guarded until now.
- **the lattice takes the offset it was asked for** — `build` wires both offsets through. Cheap to get wrong silently -- an
- **a placement moves the weld point but only turns the electrode** — A point has an origin and a direction does not.
- **no placement leaves the seam where it was** — `None` is a part whose own frame IS the table's, not an error.
- **the axes on the way to the work that are not binned get pinned** — Pinned explicitly rather than left on a fresh tensor's zero. The value is
- **the positioner moves the work frame and stays a rotation** — Forward kinematics on the work chain, not an `Rz` written out.
- **the seam is carried into world in sample spin bin order** — The flat order IS the lattice's, and everything downstream indexes on it.
- **the frontier is the first empty layer** — Constructed rather than hoped for: a real seam that fails everywhere
- **no frontier when every layer survives**
- **the summary counts add up**
- **a cell without the named positioner is refused**
- **a positioner without limits is refused** — Its bins would be invented, and the graph would search angles the
- **an axis that does not move the work frame is refused** — J1 is an arm axis. Binning it as the positioner would enumerate angles
- **the standoff is the programs own** — How far `tool0` -- the tip of the baked gun -- is held off the work is
- **the sphere model and the standoff agree** — At the seam's AUTHORED standoff the screen clears a share; with the tip
- **the collision screen is sliced and the slicing changes nothing** — The screen places 300 spheres for every node, so a 64-bin lattice asks
- **collision screening only marks nodes that passed kinematics** — The cause codes are a funnel: a node that failed IK cannot also be
- **the electrode axis is read from the cell and not defaulted** — It was a module constant for two commits, set to the one value that is
- **a lattice for an unmeasured cell is refused** — No lattice exists for a cell nobody measured.
- **every work axis the lattice did not sweep is named and valued** — `docs/issues/0003`. The chain to the work on this cell has TWO axes --
- **a held axis actually moves the part** — `held` has to reach the kinematics, not just the report. Table A sits
- **holding an axis that is not on the work chain is refused** — A typo in a held axis name would otherwise be silently ignored, which is
- **the iterative fallback never invents and rarely misses** — `analytic=False` is how a lattice gets built for an arm the closed form
- **the screen runs in float32 and agrees with float64** — The screen's precision is a decision about this MACHINE. Consumer cards
- **half precision cannot test its own tolerance** — Why the answer to "would float16 do" is no, and it is not about torch.
- **tensor cores stay off and the reason is two reasons** — TF32 puts float32 matrix multiplies on the tensor cores. Measured on a
- **samples are timed rather than counted** — A fixed count per seam is the wrong unit, and the corpus is the argument.
- **a stated threshold reports the joint rate it assumes** — The cell has no velocity limits, so the threshold is STATED and the
- **self test entry point passes**

## `motion_planner/test_plan.py` — 20

> The pipeline, end to end -- what `planner/` exists to be.

- **the placed request planned screened and crossed**
- **candidates carry the timing the process already fixed** — `times_s` is a declaration, not an invention: the lattice placed its
- **no candidate breaks a velocity limit** — The cross-check the timing makes possible. The edge rule already
- **rate checks actually ran** — No 'unverifiable' left. The profile carries velocity AND acceleration
- **acceleration findings are reported not hidden** — The one-second rule made acceleration checkable, and on the corner
- **the wire shape is the same with and without trajectories** — M5 on the wire is ADDITIVE and this is what says so.
- **one blocked seam cannot take the whole response with it** — `NaN` is not JSON, and it kills the WHOLE encode rather than its field.
- **a stage can be started from instead of run** — M4 is seconds and M5 is minutes, so reaching the minutes must not cost the
- **handed trajectories run m6 alone** — Connecting moves from trajectories somebody else made, with no search.
- **the sweep runs once however far the plan goes** — M5 must be handed the plan M4 already made, not run it again.
- **the server hands the cards memory back** — A process that lives for days must not sit on the peak of its worst
- **the plan accounts for the model it baked** — Baking the distance field is the plan request's COLD cost and the single
- **the drawer knows which stage it is allowed to expand** — The one stage name the UI has to hold as a literal, held on both sides.
- **the endpoint and the planner cannot disagree about a default** — They did. Every default was written twice -- once as a `Query(default=)`
- **the server does not carry a pipeline of its own** — It did twice: a whole copy of the sweep, and later the ORDER of the three
- **one plan screens against one collision model** — The three stages must all be looking at the same obstacles.
- **blocked seams carry their causes**
- **every run says where its time went** — M9's breakdown needs a measurement, not an intention.
- **every seam reports the stages it actually ran** — Per seam, and as LEAVES: `plan_seam` folds the lattice's own stages in
- **the seam stages add up to the run** — The per-seam totals must account for the `seams` stage that contains

## `motion_planner/test_profile.py` — 15

> The cell profile: the axis roles no descriptor can state, and what is

- **the profile agrees with the descriptor it describes** — Every structural check at once: no axis invented, none forgotten, the
- **only the live positioner axis is driven** — The fact this file exists to record. J7 tilts the bay being welded;
- **the null space is two dimensional** — Which is what lets the redundancy graph carry spin and positioner
- **held axes are reattached rather than defaulted** — The trap. A pose over driven axes alone leaves J9 unset, and the
- **what is still missing is named with the milestone that needs it** — Absence is a task, not a fault. Limits and a reset pose are real
- **the reset pose is the urdf home and the arm stands clear there** — The pose M6 will approach from and retract to, checked rather than
- **the placeholder held values are declared as such** — A held J9 of 0.0 is a guess about which bay faces the robot, and the
- **an unmentioned axis is caught** — The dangerous omission: an axis in neither list gets no value at all,
- **driving an axis the cell does not have is caught**
- **a profile for another cell is caught** — Both halves valid, wrong pairing -- the same class of mistake the
- **a pose over the wrong axes is refused**
- **an unwritten profile says what to write**
- **an unknown schema is refused rather than half read**
- **the committed profile is valid json with the right schema**
- **self test entry point passes**

## `motion_planner/test_request.py` — RESTORED 2026-09-11

Brought back with the program.v2 corpus, since it is the file that tests the
request format that changed. The claims below are live again, in that file.

> The planner's front door: what it accepts, and what it says is missing.

- **a fixture request opens with every document present** — All five, because a planner that got four would discover the fifth
- **the geometry is in the workpiece frame** — GOAL.md non-negotiable 5. Everything this planner does with the seam is
- **the cell arrives with kinematics and without meshes** — The split `plan_request.py` argues for: the descriptor pins the joints and
- **no joint value reached the planner** — The positioner is prescribed HERE and nowhere upstream. A request that
- **the fixtures are plannable in principle** — Empty `blocking` is the M0 bar: nothing about these requests is wrong.
- **the per cell assets are named rather than assumed** — A URDF `<limit>` carries position only. Velocity and acceleration are
- **the fixture SET exercises a band** — M4's rescue pass and M5's angle attraction both need a seam with an
- **the planner does not read the tooling document** — The torch collides, but its geometry is the CELL's, not the weld
- **a tampered request is refused** — Digests are checked on read. A request edited after it was built would
- **both fixtures open** — Parametrized over every committed plan request, so a fixture added later is
- **self test entry point passes**

## `motion_planner/test_roadmap.py` — 9

> The probabilistic roadmap, and the wall it found.

- **the box narrows to what is being joined** — A roadmap spread over travel the plan never visits spends its
- **an axis with no stated limit is refused rather than guessed** — The same refusal `ik.limits_of` makes, for the same reason: an axis
- **every node stands clear of the world and of the arm** — Re-screened from the returned nodes rather than trusted, because a
- **the self margin is smaller than the environment one on purpose** — It looks like a typo and it is not.
- **the same request builds the same roadmap twice** — Sobol, unscrambled, not an RNG. Planning the same plan request twice must
- **there is nowhere to stand with the wrist half turned** — The finding that explains `docs/issues/0009`, stated directly.
- **the winding decides whether an approach exists** — The same physical weld, one turn apart on `J5`, on opposite sides of
- **no route is reported as no route** — `None`, not a straight line. A roadmap that cannot connect two poses
- **a route becomes a seed that meets the clamped endpoints** — The route carries no timing and no velocities, so `MoveSolver` lays it out

## `motion_planner/test_verify.py` — 26

> The T2 verifier.

- **the certificate never passes a segment that collides** — 400 random segments, verifier versus brute force.
- **the verifier is not merely refusing everything** — The other direction, allowed to be conservative but not useless.
- **uniform sampling misses what the verifier catches** — The reason the verifier is not a for-loop over waypoints.
- **the verifier certifies the curve and not the line through its ends** — A trajectory of waypoints MEANS a polyline; a trajectory of waypoints
- **velocities equal to the chord slope change nothing** — The compatibility claim, and it is exact rather than close: at the
- **velocities without times are refused** — Half a specification is not a trajectory: the same knots and slopes
- **both rate checks read the keys the loader actually writes** — One list of spellings, because two is how the fix half-took. The
- **the reach radius bounds the actual motion** — `R_j` is the whole proof. If any point moves faster than `R_j` per
- **the reach radius counts bodies that are not their link** — A body's name is not its link's name.
- **a body offset in its link frame enlarges the reach** — `local` is part of the extent, not decoration. A torch mounted 300 mm
- **the verifier kinematics matches the committed oracle** — The verifier writes its own FK so it can catch a kinematics bug in the
- **missing rate limits are reported and not passed** — The distinction the module exists to preserve. This cell has no
- **a position limit violation fails**
- **rate limits are checked when the profile carries them** — And the check has to bite, or the fail-closed path above is the only
- **rate limits are read in the profiles own spelling** — `weld_motion_planner.robot_cell.robot.profile` normalises to `velocity_rad_s` /
- **timing that goes backwards is refused**
- **a trajectory with the wrong axis count is refused**
- **one waypoint is not a trajectory**
- **a disconnected link is named**
- **a held axis leaves the trajectory and stays in the geometry** — J8 and J9 on the real cell: the arm must not hit bay B, but this
- **travel without a tangent is unverifiable not passed**
- **travel without rate limits is unverifiable not passed** — The profile for this cell genuinely has none, so this is the live path
- **welding speed has headroom and a transit speed does not** — Both directions of the same check, so neither is vacuous.
- **the arm axes are matched by name and not by position** — The trajectory spans every axis the verifier models, positioner
- **a verdict carries the travel report**
- **self test entry point passes**

## `motion_planner/test_weld_trajopt.py` — 13

> Trajectory synthesis, judged by its own report.

- **the trust region recovers after a rejected step** — The ratchet `freespace` fixed and this loop still has.
- **the stage ranks the candidates by its own ruler** — M4 orders the ways across by accumulated L1 joint motion; this stage
- **re ranking cannot certify what the bound does not cover** — The finding that keeps this stage honest about its own improvement.
- **the losers travel with the winner** — The choice has to be arguable. Every rejected candidate reports what
- **the emitted curve passes the verifier with nothing unverifiable** — The end of M5's done-when list, and the first time the T2 verifier has
- **synthesis does no harm and clears the rate findings** — The smooth seam: the seed arrived healthy and must leave healthier.
- **a seam nothing can reach is reported and not synthesized** — `bracket_a_2x seam_0002` has no feasible node at its very first sample
- **a short seam is not taxed for sharing a batch** — The risk this whole batching rests on.
- **every seam in the request is one batch** — Cross-seam batching is the point, so it is asserted rather than
- **the second fixture certifies end to end** — `bracket_b_2x`: plan request in, certified trajectories out, whole plan.
- **synthesis is deterministic** — Fixed-seed determinism is in M5's done-when; nothing here draws a
- **the wire is the curve at the controllers clock** — What leaves this stage is `(q, qdot, t)` at `WIRE_DT_S` -- free
- **the pad contributes nothing to a real knots gradient** — The contract behind the pad-tax test, stated where it is decided: the

## `programmer/test_export_robot_v4.py` — 12

> The rail-to-row compile, held to OLP's own contract.

- **js numbers match ecmascript stringify**
- **the digest ignores exactly the transient fields**
- **joints map by name never by position**
- **a mismatched result is refused**
- **a varying travel speed is refused not averaged**
- **the mini program compiles whole**
- **the corpus request compiles**
- **olp own validator accepts the compile**
- **the sidecar rides only when it matches the provenance**
- **regeneration keeps a hand edited seam whole and replaces the rest**
- **regeneration refuses a program it did not export**
- **every certificate density knot is a row and fits the cap** — The wires arrive at 10 ms (control_solver.WIRE_DT_S) and every knot

## `programmer/test_server.py` — 3 (file kept, tests removed)

> These three drove `/api/plan-requests`, which builds a weld-program.v1
> through `build_program`. D24 deletes both that route and that function.

- **a plan request carries the program and the placement** — E. A planner cannot use the program alone -- it does not say where the
- **the program inside a plan request still names no torch** — The grep guard, applied where it would be easiest to lose.
- **a plan request is byte stable** — Same inputs, same bytes. "Did the request change?" is the first question


---

# Parked with the authoring server (D24)

**187 more, across 9 files.** The server, its workspace store, `build_program` and the rail-to-row exporter are gone, so these test code that no
longer exists rather than claims that no longer hold. Where a claim outlived its
module the test moved instead: the dense blob's admission contract is still
asserted in `test_dense_joint_trajectory.py`, against `weldplan`.

## `programmer/test_bundle.py` — 25

> Needed `store/bundle.py` -- the project bundle, moving to the OLP project.

- **a bundle carries the gun and the policy it was authored against** — D13's promise, and until the presets were keyed by kind the bundle could
- **a bundle identifies itself without being parsed** — The ODF trick. `mimetype` stored uncompressed as the FIRST entry sits at
- **a bundle is byte stable** — Same project, same bytes. Without a fixed timestamp this would be
- **a bundle written before presets had kinds still opens** — The archive's member list did not change -- only `presets.json` widened
- **a multi piece weld reattaches to the run it became** — Three pieces welded as one pass, reopened where the run is one seam.
- **a placement names the cell it was authored against** — An offset and an rpy are measured from a work frame, and every cell puts
- **a placement survives the round trip** — The whole point of D. Without it the operator sets the part down again
- **a plain zip is refused**
- **a tampered member is refused** — Truncation, a bad edit, or tampering -- either way the contents are not
- **a trim survives being split across pieces** — The same reattachment, on a weld the operator had trimmed.
- **a weld whose arc is gone is reported not dropped** — R-UI-1 applied to project data. A weld that quietly vanished from a
- **a weld whose coverage survives is matched even when its pieces do not** — The converse, and the reason the previous test has to strand both: a
- **a weld with no anchors is reported**
- **a whole run reattaches to the pieces it became** — The other direction: saved before the split existed, opened after it.
- **analysis parameters are carried because they are inputs** — The candidate set is a function of the STEP AND these knobs. Reopening
- **it round trips**
- **only referenced presets are inlined** — Inlined rather than named: a bundle that referenced a preset by id alone
- **reattach finds everything on an unchanged analysis**
- **reattach survives a different analysis** — The test that earns the design.
- **self test entry point passes**
- **something that is not a zip is refused**
- **the anchor is the midpoint of the arc** — Including across a wrap, where the naive average of the endpoints lands
- **the bundle holds no derived geometry** — The governing rule, asserted rather than trusted. No weld line, no rail,
- **tolerances are stated**
- **two seams never share an anchor key**

## `programmer/test_cell_api.py` — 9

> Needed `server/cells.py` -- the cell routes; cells are enclosed in the request now.

- **a declared mesh is served**
- **a descriptor carries the joints and the meshes**
- **a file that exists but is not a mesh is refused** — The whole point of the whitelist. `calibration.json` is really there,
- **an absent bundle leaves the list empty rather than failing** — A sibling module that is not checked out is not this module's failure.
- **an unknown cell is 404**
- **every mesh a descriptor names can actually be fetched** — A descriptor that names a file the bundle does not have would render as
- **one cells mesh is not reachable through another** — Each cell serves only what its OWN descriptor names. Both of these are
- **the cell list includes the ones it cannot use** — "There are five bundles and three do not say where a workpiece is
- **traversal cannot reach outside the declared meshes**

## `programmer/test_export_olp_dense_request.py` — 3

> Needed `server/export_olp.py` -- the rails compile route.

- **a refined dense compile carries the identity**
- **dense without refine is not asked for**
- **missing identity fields default to the first revision**

## `programmer/test_native_motion_admission.py` — 7

> Needed `export/robot_v4.py` for its fixture and its rows arm; the dense arm's claim survives in `test_dense_joint_trajectory.py`.

- **admitted result preserves rows and dense conversion**
- **candidate authoring requires explicit mode and rejects continuous results**
- **conversion cannot hide findings behind pass**
- **conversion refuses bad or missing certificates**
- **http rejection never writes the catalog**
- **late failure is rejected before any conversion**
- **rows route preserves authoring and gates refinement**

## `programmer/test_presets.py` — 23

> Needed `store/presets.py`.

- **a band that is not a scalar function is rejected** — Plain numbers would be the obvious thing to write and the wrong one: the
- **a broken preset is reported not skipped** — A silently shorter list is the quiet failure this module exists to
- **a kind directory that does not exist is not an error**
- **a preset does not smuggle a marker into the seam** — `additionalProperties: false` has no room for `preset_id`, and that is
- **a preset may carry an admissible band** — A procedure is exactly the right place to say how far a planner may move
- **a preset must actually set something**
- **a preset that is invalid inside a seam is invalid here** — The whole point of validating by $ref into the program schema.
- **a procedure whose own angle is outside its band is refused** — The counterfactual, so the test above is not passing by accident.
- **a search preset may not reopen the stickout band** — It was a settings knob that let the search alter a PROCESS parameter on
- **a search preset must state every field** — A policy is something you pick BETWEEN. A half-stated one compared
- **a torch preset is checked by the function that defines a torch** — One definition per object. `tooling.problems` was the torch record's
- **a torch preset states a range and never a standoff** — How far the gun is HELD is the procedure's to say. A torch that also
- **an unknown kind names the kinds that exist** — D8 for a programmer holding a wrong string: a bare KeyError tells nobody
- **applying a preset replaces whole blocks** — Not a deep merge. This preset's voltage against the previous preset's
- **every kind ships at least one usable preset** — A kind with an empty shipped directory is a tab that opens on nothing,
- **every preset produces a valid program with no warnings** — The real test. A preset is only usable if the program it produces is.
- **every shipped preset carries every field its form draws** — The other half. A form row with nothing behind it renders as an empty
- **every shipped procedure declares the band it will tolerate** — The band `settings.defaults.orientation.limits` used to hold and never
- **presets are where the server looks**
- **self test entry point passes**
- **stickout limits are plain numbers and absent by default** — Deliberately asymmetric with the orientation band. Moving a work angle
- **the module ships usable presets** — One procedure, for the gun this cell has.
- **the ui field table only names paths a seam actually has** — A dead row in the parameters dialog is invisible.

## `programmer/test_program.py` — 18

> Needed `program.py` -- `build_program`, deleted with the resolution it did.

- **a group cannot be welded as one pass** — The pieces of a group touch end to end, and it used to follow from that
- **a program without topology is still valid** — `solids` is optional. Inventing bounding boxes would be worse than
- **a split weld builds and validates** — The whole point of F6: two welds over one candidate now reach a program.
- **a wrapping group is refused like any other** — The column's second run straddles s=0. That used to be the interesting
- **an empty selection refuses with a reason** — minItems: 1. Refusing beats emitting a document that fails validation
- **an unknown id is refused not ignored**
- **dog box builds a valid program**
- **limits authored as overrides reach the program** — What the limits editor writes has to survive resolution.
- **nothing forbidden survives the projection**
- **only referenced joints are carried**
- **pipe bracket builds a valid program**
- **self test entry point passes**
- **source path becomes the uploaded filename** — The artifact's path is where THIS server put the upload. Provenance
- **spin stays free through the projection** — R2 survives one more transform.
- **the dropped keys are named and pinned** — If the artifact grows a key, this fails rather than the key silently not
- **the pieces are still all weldable separately** — The rule costs no arc. Every piece of every group is still a weld, and
- **the program says nothing about a robot** — R1/R5. The same shape as test_weld_program_schema's own check, run over
- **workpiece solids are projected not copied** — Topology solids carry centroid_xyz_m, face_ids and geom_key. The

## `programmer/test_server.py` — 53

> Needed `server/` -- all 42 routes.

- **a bad step fails visibly and leaves nothing behind** — R-UI-1. The most important test here.
- **a cancelled search says cancelled and not failed** — Cancelling recycles the pool -- a running OCCT call cannot be
- **a placement naming an unknown cell is refused** — Rather than resolved against whichever cell happens to be first. The
- **a placement survives save and open over http** — D end to end, through the real routes rather than the writer alone.
- **a plan request without a placement is refused** — With a reason, not with an emitted file the planner discovers is
- **a project brings the gun it was authored against** — D13's promise, on a machine that has never seen the preset. The bundle
- **a project saved without a placement opens without one** — Null, not an invented default. A part that was never placed must not
- **a workspace preset can be deleted and a shipped one cannot** — Deleting a shipped preset would make two checkouts of the same module
- **an invalid program cannot be saved** — WEBUI.md section 11: the UI must not be able to save an invalid program.
- **an opened stickout band becomes a standoff ladder** — Absent limits mean zero freedom -- the default and the safe reading.
- **an operator preset round trips and a bad one cannot be saved**
- **an unknown preset kind names the kinds that exist** — D8 for a URL. "unknown kind" without the list leaves the caller guessing
- **an unknown stage is refused**
- **analyze rejects an unknown sha**
- **artifacts 404 before analysis**
- **artifacts are served and agree with each other**
- **both directions are verified at the fine fan** — Not just the one the lattice preferred.
- **building a program from nothing is refused with a reason**
- **clearance answers for a resolved weld** — The weld arrives as a LIST ENTRY and is resolved server-side.
- **clearance follows the operators overrides** — Tipping the torch over must change the answer, or the resolution is
- **clearance never lands in the tooling independent artifact** — The contract guard, as a test rather than a convention.
- **clearance refuses a torch this machine does not have** — Never the shipped nozzle as a stand-in. A coverage number measured with
- **clearance refuses rather than guessing**
- **clearance uses the torch the settings name** — A Ø32 tube cannot enter what a Ø16 nozzle can. The endpoint has to be
- **empty upload is rejected**
- **every event carries provenance** — R-UI-2. Nothing is shown without saying which file produced it.
- **health reports a warm pool** — The UI disables Load until this says ready, rather than discovering a
- **job reports named stages not a spinner** — R-UI-3. Every stage is separately reported and carries its own numbers.
- **load then analyse is two steps** — The whole lifecycle, in the order an operator does it.
- **optimise never lands in the tooling independent artifact** — The same contract guard the clearance check has. An answer that depends
- **optimise refuses before a job exists** — Everything cheap happens before the pool is touched, so a request that
- **optimise runs to a profile it has verified** — The answer the operator sees must be measured on the profile that will
- **optimise states its cost before it starts** — Rays, not seconds. Seconds depend on the machine; "this will take a
- **presets are served with their problems**
- **propose 404s on an unknown face**
- **propose accepts a real contact pair** — Manual authoring reuses the detector's own path, so the record it
- **propose explains why a bad pair is not a joint** — "Not a joint" is useless. The operator needs to know whether the model is
- **reanalysis recomputes rather than reusing artifacts** — There is no result cache.
- **seam frames endpoint returns an orthonormal frame** — The server owns the frame construction so the client cannot diverge on a
- **seam frames rejects a malformed body**
- **sse events are replayable after a reload** — R-UI-7. A client that reloads reconnects by Last-Event-ID and must not
- **the joints artifact carries seams** — The UI lists seams, not joints, so the seams have to be in the artifact
- **the optimised profile is a document the contract accepts** — It goes into a weld program as an ordinary override, so it has to
- **the reference fixture round trips**
- **the server builds and validates a program** — The client never assembles a weld program. If it built the document in
- **the settings export follows a saved override** — An export that kept reporting the shipped default after an operator
- **the settings export is a download of what the dialog shows** — One builder behind both endpoints, so a copy an operator saves cannot say
- **unknown job is 404**
- **upload is addressed by content**
- **validate checks a rate limit against the seams own length** — Per millimetre of arc, so the seam's span has to reach the check.
- **validate rejects a profile outside its own band** — An ERROR, not a warning.
- **validate warns about locked spin** — Schema-valid but a design mistake: locking spin fixes all 6 DOF and
- **validate warns when rail and weld line counts differ**

## `programmer/test_settings.py` — 37

> Needed `store/settings.py`.

- **a block that moved is named and never ignored** — The whole point. This checkout's workspace had `torch.default_standoff_mm:
- **a bundle with settings is still byte stable**
- **a bundle without settings reads back as none** — Older bundles, and any project saved before this field existed.
- **a default procedure with no stickout refuses rather than guesses** — A coverage number measured at a guessed gap is measured against nothing.
- **a deflection over the surface skip is refused** — The invariant that stops a fast wrong answer.
- **a layer can select the exact intersector** — The oracle stays reachable without editing code. It is 100x slower and
- **a missing workspace file is not an error**
- **a misspelled analysis knob is reported** — The failure this guards against is invisible: a setting that merges but
- **a misspelled intersector field is reported** — Same rule the analysis knobs keep: a typo that merged silently would
- **a partial override keeps the rest of the block** — The reason for a deep merge rather than a block substitution.
- **a project carries the intersector it was authored against** — Same argument as the torch. A blocked span measured against a 0.02 mm
- **a project layer does not disturb the analysis knobs** — A reopened project brings its torch, not the machine's whole config.
- **a project layer travels in the export** — A reopened project's own layer is part of what is in force, so a copy
- **a ray count written as a whole float is accepted** — JSON has no integers, so 256.0 must not be rejected as fractional.
- **a reference this machine cannot resolve is stated and refused** — Never a fallback. Substituting the shipped nozzle for a gun that is not
- **an unknown engine is reported rather than ignored**
- **an unreadable workspace file is an error rather than ignored** — R-UI-1: a settings file that silently did nothing is the worst outcome.
- **analysis defaults drops nothing it understands**
- **changing a default does not move a saved projects parameters** — The case where confusing the two would actually bite.
- **each layer overrides the one below**
- **every shipped reference resolves** — A default naming a preset nobody ships is a first-run dead end, and now
- **layers are reported lowest first**
- **settings survive a bundle round trip**
- **the default procedure is read and owns the fallback standoff** — `defaults.preset_id` was declared, validated and shipped for months with
- **the export carries the layers that made it** — A resolved document says the radius is 9 mm. Only the layers say why, and
- **the export is byte stable** — Two exports of the same settings are the same file, so diffing them is
- **the export reports problems rather than refusing** — Same rule as `load_presets` and the endpoints over it: a broken config is
- **the exported workspace layer stays sparse** — The one that matters for reading an export back.
- **the manifest analysis parameters are not a settings layer** — A project's record of what WAS analysed is not a default for what will be.
- **the resolved document always names its schema**
- **the search policy is a preset and a project may pick another** — How far the search LOOKS is configuration, not a number in a module --
- **the settings hold no torch and no standoff of their own** — The regression test for the bug this refactor exists against: a
- **the shipped defaults are usable** — A module default that does not validate would break every fresh install.
- **the shipped intersector is the indexed one** — The default is `bvh`, and it is the module file that says so.
- **the torch block migration names the preset that matches it** — This checkout's own workspace file is the case: `body_radius_mm: 16` is
- **unusable settings are reported**
- **without the library only the shape of a reference is checked** — Whether a preset is installed is a question about a MACHINE. A settings

## `programmer/test_weld_model.py` — 12

> Needed `store/` -- the weld list resolved against presets.

- **a split weld does not get re merged across the gap** — The half of this that renaming alone would have missed.
- **a weld that shares nothing keeps its id** — A rename that fired on every weld would churn the ids in every saved
- **an unknown seam or preset is refused**
- **merging across groups is refused** — Two groups of one joint are separated by arc no torch can reach. Welding
- **merging within a group is refused for the other reason** — Same refusal, different fact. Pieces of ONE group do touch end to end --
- **overrides do not mutate the candidate** — Only the overrides are persisted, so the candidate has to stay pristine
- **the client does not own ids and the split shows why** — Where the two copies deliberately STOP agreeing, pinned so it does not
- **the fixture exercises every stage** — A parity test over trivial input proves nothing. This asserts the cases
- **the node check script passes** — The TypeScript's own assertions, run as part of the Python suite so a
- **two entries on one seam get their own ids** — Splitting a weld puts two entries on one candidate. They used to be
- **two entries on one seam without ids are refused** — Disambiguation needs something to disambiguate WITH.
- **typescript and python resolve identically**


---

# Parked with the weld planner's own UI (D24)

**86 more.** Some needed the settings store. The rest were LANGUAGE PARITY checks -- they ran a `.mjs` in `ui/scripts/` beside the Python and asserted the two
agreed on placement, span and scalar-function evaluation. Deleting the UI took the
TypeScript half, not the claim: the OLP carries ports of the same code in
`offline-programming/v1/ui/src/rails/` -- `scalar-function.ts`, `span-math.ts`,
`frame-math.ts`, `decisions.ts` -- so parity has somewhere to be re-pointed, and it
matters more now than it did, because that copy is the only one left.

## `programmer/test_pipe_bracket.py` — 13

> Needed `store/settings.py` for the shipped presets.

- **accessibility carries no tooling assumptions** — The reason this belongs in a joint record at all.
- **all five joints classify as tee** — The first case where the lap/tee heuristic must answer 'tee'.
- **all weld lines chain into closed wires** — The chaining fix has to hold on geometry it was not developed against.
- **blocked boundaries are refined not snapped to samples** — REGRESSION. Sampling alone reports spans shifted by a whole step.
- **blocked spans land where the parts actually touch** — Not just 'something is blocked' -- blocked WHERE the gussets are.
- **column weld is a closed rounded square**
- **dog box has no occlusion to test against** — Why pipe_bracket had to be added.
- **every joint has blocked spans** — Was a KNOWN GAP; now the assertion.
- **joint topology matches the weldment**
- **no tooling reaches the whole program either** — The same rule, widened from one block to the whole contract.
- **rail is offset along the groove bisector** — Detection produces a usable rail on a tee, not just on a lap.
- **suggested start is outside every blocked span** — A closed loop starts at an arbitrary edge, so s=0 can land inside a
- **topology inventory**

## `programmer/test_scalar.py` — 30

> Needed `store/` and `ui/scripts/scalar-check.mjs`.

- **a band that varies is checked where it varies** — The admissible window genuinely narrows where a flange rises beside the
- **a broken profile fails validation** — The schema would accept these knots. The validator must not.
- **a constant ignores position**
- **a malformed bound is reported rather than evaluated**
- **a mismatched spline is caught**
- **a profile inside its band is fine**
- **a profile that does not span the seam is caught**
- **a quadratic bspline evaluates**
- **a roll holds flat then turns then holds** — The shape this feature exists for: a work angle that rolls over part of
- **a stepped profile has an infinite rate** — It jumps. Reporting a finite number would be a comfortable lie.
- **a valid function has no problems**
- **a varying angle reaches the program and validates** — End to end: author a rolling work angle, build a program, validate it.
- **a violation names the place and both numbers** — "Work angle out of range" with nowhere to look is not actionable.
- **an uncheckable rate is reported as unchecked** — R-UI-1 applies to validation too: a limit that quietly did not run is
- **crossed limits are reported before anything else** — JSON Schema cannot say min <= max, and a crossed band makes every
- **every scalar in a seam is found** — Validation has to reach all of them, including the ones nested three
- **flatness is recognised**
- **knots out of order are caught** — This validates against the schema and is meaningless. The evaluator
- **limits are walked by scalars in** — They inherit the knot checks, the tagged-union override rule and the
- **non finite values are caught**
- **piecewise linear interpolates between knots**
- **position clamps rather than extrapolating** — Off the end of a seam there is no weld, so there is no value to
- **step samples hold their left value**
- **the extreme between two sample points is still caught** — Both sides are piecewise linear, so their difference is too, and its
- **the node check script passes**
- **the rate is exact for a piecewise linear profile**
- **the rate limit is per millimetre not per unit s** — The whole reason the field is named that way.
- **the rate of a spline is sampled and says so**
- **the worst violation is the one reported** — One message, at the place it is worst -- not one per sample.
- **typescript and python evaluate identically**

## `programmer/test_tooling.py` — 20

> Needed `store/settings.py` -- the torch and its standoff.

- **a fatter torch needs a wider groove**
- **a strided fan is a subset of the full one** — The escalation is only sound if the cheap set is nested in the full one.
- **a torch that cannot exist is rejected**
- **an empty fan is empty rather than an error**
- **every shipped preset gives a usable standoff** — The clearance answer depends on this number, so no preset may omit it.
- **half the fan sits exactly on the rim** — Where a cylinder actually touches a plane.
- **pulling the torch back only ever buys clearance**
- **stickout is the strongest lever there is** — Why the optimizer searches over standoff at all.
- **the fan is deterministic** — An optimizer that is not reproducible cannot be debugged.
- **the fan scales with the radius**
- **the fan spacing is fine enough for a three millimetre plate** — dog_box has 3 mm plates, and a plate thinner than the fan spacing that
- **the fan stays on the barrel**
- **the fit angle matches closed form** — `2 * atan(R / d)`, checked against values that need no calculator.
- **the interior is equal area** — sqrt spacing, so the middle of the barrel is not probed more finely than
- **the interior starts on the axis** — The axis ray is the one that reads as buried when the torch is inside a
- **the numbers the docstring and the ui quote** — These three decide whether the feature looks broken on a 90 degree fillet.
- **the rim makes the radius error second order** — A ring of m points behaves like a barrel of radius R*cos(pi/m).
- **the shipped torch is usable**
- **the weld owns its standoff** — A seam's own stickout wins; the caller's fallback is only a fallback.
- **zero standoff degenerates to nothing fits** — The arithmetic divides by the standoff, so the limit is stated, not hit.

## `programmer/test_placement_parity.py` — 18

> Needed `ui/scripts/placement-check.mjs`.

- **a cell with no work frame is refused rather than measured** — Not silently measured against the world, which would look like a
- **a dropped part touches the table however it is turned** — The reason this is a button and not "set z to zero": once the part is
- **a malformed placement is reported**
- **a part clear of the cell falls back to the work plane** — Rather than being held up by geometry it does not overlap.
- **a part lands on the bar not on the work plane** — The bug this is here for. `positioner_table_a` is a 200 mm square bar
- **a placement rides the positioner** — The claim the module exists for. A part clamped to the table stays
- **dropping measures the part not its bounding box** — A rotated box contains the part but touches it only at its corners.
- **dropping only moves the part down** — Sliding it sideways to find a lower resting place is a different
- **reading a pose back round trips** — A handle that cannot round-trip drifts: every drag would land a little
- **reading back measures against the table not the world** — The bug this is here to catch: forget to divide out `T_work` and the
- **self test entry point passes**
- **the cases actually move the part** — A parity suite over placements that barely differ proves the harness
- **the part stands on the table by default** — The anchor is the footprint centre on the UNDERSIDE, so the default puts
- **the two agree on reading a pose back** — The direction the drag handle runs.
- **the two agree on where the part comes to rest**
- **the two agree on where the part is**
- **turning the part spins it about its own anchor** — Not about the table centre. With the anchor translation outside the
- **with no geometry the part stays where it is** — Rather than being moved to an arbitrary height, which would look like a

## `programmer/test_span_parity.py` — 5

> Needed `ui/scripts/span-check.mjs`.

- **containment agrees with the walk** — Whatever `s_at` produces must be on the span, at every fraction.
- **self test entry point passes**
- **the ends are exact** — A seam's end IS a blocked span's start, so an end computed as
- **the wrap is actually exercised** — A parity test over spans that do not wrap proves nothing about the case
- **typescript and python agree**

## `programmer/test_decisions.py` — 14

> Needed `ui/scripts/decisions-check.mjs`. Parity again: the OLP has `rails/decisions.ts` and `rails/kinematics`-side code.

- **a loop is not split at its own numbering** — A weld all the way round a closed joint has no ends.
- **a run under the process floor is dropped but still counted** — 5 mm is `seams.DEFAULT_MIN_SEAM_LENGTH_M`. Below it nobody strikes an
- **a split of a wrapping span tiles the original arc** — The failure this module exists to prevent: sub-spans that are ordered,
- **a weld with nothing wrong says so rather than offering three buttons**
- **a wrapping run is not measured as negative** — The failure mode if the wrap is missed: `hi - lo` on a joined run is
- **a zero length seam is refused rather than divided by**
- **sealed arc is reported apart from merely blocked** — No angle, no standoff and no optimiser run recovers a sealed stretch, so
- **self test entry point passes**
- **the arc is conserved**
- **the floor comes from the clearance result not a second opinion**
- **the parity cases actually exercise the hard parts** — A parity suite over easy input proves the harness works and nothing
- **the trim of a wrapping span measures back to its own length**
- **trim takes the longest run and split takes them all**
- **typescript and python offer the same thing**

## `programmer/test_kinematics_parity.py` — 5

> Needed `ui/scripts/kinematics-check.mjs`. Parity again: the OLP has `rails/decisions.ts` and `rails/kinematics`-side code.

- **a point fixtured to the table rides with it** — The claim placement rests on. A workpiece 300 mm above the table centre
- **every link agrees between the two** — Full 4x4 per link, not a tool position. A wrist error that happens to
- **self test entry point passes**
- **the grid actually moves the cell** — A parity suite over poses that barely differ proves the harness works
- **the table turns without its origin moving** — J7 tilts the table about an axis through its own origin, so the origin
