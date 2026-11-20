# Measured, on this machine

Numbers replace the estimates in `ARCHITECTURE.md` as they are taken. Rerun
with `pixi run -e bench motion-bench`; the harness is `tools/bench_esdf.py`
and states what it does and does not compare.

Machine: RTX 5090 (sm_120, 31 GiB), torch 2.13 for CUDA 12.9, warp 1.15.
Stable to about 1% across independent runs.

---

## 1. The distance field, against cuRoboV2

The question was whether baking EXACTLY -- one closest-point query per voxel
against a GPU BVH -- is affordable next to the seed-and-propagate pipeline
`ARCHITECTURE.md` originally planned and cuRoboV2 actually ships.

`JumpFloodingEDT` is the real class from the cloned repository. What is timed
against it is construction: seed the grid from the same mesh, then propagate.
Their depth-fusion front end has no analogue here and is not included -- our
scene is known CAD, bounded, baked once.

**This table corrects an earlier version of itself.** The first published
run claimed JFA's near-surface error had a resolution-independent ~2 mm
floor. That was this harness under-seeding large plates (an area-
proportional sample count silently capped at 4096 per triangle), not JFA.
Seeding is now by subdivision -- coverage as a geometric property rather
than a sampling budget -- and with it JFA behaves exactly as its
construction says it must: worst-case near-band error of ~1 voxel, shrinking
with resolution. The conclusion below survives the correction; the earlier
argument for it was wrong and this one is right.

NEAR = voxels within 10 mm of a surface, where the torch actually lives.
Whole-grid averages are dominated by regions nothing ever visits.

| part | voxel | grid | Mvox | MiB | ours | seed + JFA | propagate alone | NEAR err mean / p99 / max |
|---|---|---|---|---|---|---|---|---|
| bracket_a | 3.0 mm | 71×78×98 | 0.54 | 2.1 | 0.8 ms | 4.3 ms | 0.2 ms | 0.73 / 1.69 / 2.74 mm |
| bracket_a | 1.5 mm | 141×155×195 | 4.3 | 16.3 | 2.1 ms | 5.9 ms | 0.9 ms | 0.30 / 0.82 / 1.50 mm |
| bracket_a | 1.0 mm | 211×231×291 | 14.2 | 54.1 | 5.6 ms | 8.3 ms | 3.0 ms | 0.10 / 0.55 / 1.00 mm |
| bracket_a | 0.5 mm | 421×461×581 | 112.8 | 430.1 | 35.4 ms | 35.8 ms | 27.1 ms | 0.03 / 0.29 / 0.50 mm |
| bracket_b | 3.0 mm | 98×68×65 | 0.43 | 1.7 | 0.8 ms | 3.7 ms | 0.2 ms | 0.53 / 1.38 / 3.00 mm |
| bracket_b | 1.5 mm | 195×135×128 | 3.4 | 12.9 | 1.9 ms | 5.2 ms | 0.7 ms | 0.27 / 0.69 / 1.50 mm |
| bracket_b | 1.0 mm | 292×201×191 | 11.2 | 42.8 | 5.5 ms | 8.7 ms | 2.4 ms | 0.01 / 0.38 / 1.00 mm |
| bracket_b | 0.5 mm | 582×401×381 | 88.9 | 339.2 | 33.7 ms | 32.9 ms | 21.3 ms | 0.01 / 0.20 / 0.50 mm |

**Speed.** With seeding costed, exact wins or ties every row (1.0-5.3×).
Against propagation ALONE -- the fair number when a pipeline already has a
seeded grid, as their camera path does -- exact is slower at fine part
grids (35 vs 27 ms) and faster at coarse ones. Either way both are
milliseconds against a budget of seconds: speed does not decide this.

**Accuracy decides it.** JFA's near-band worst case is one voxel -- honest
quantization, shrinking with resolution. But that means matching what the
exact bake gives AT 1.5 mm requires JFA to run a ~0.5 mm grid: 430 MiB and
36 ms in place of 16 MiB and 2 ms, a 26× memory bill for a *worst case*
that exactness provides outright. And the exact query returns the sign in
the same breath, where JFA needs a TSDF alongside to recover it.

### At cell scale, exact wins on speed too

A 2 m box, mostly empty -- propagation's home ground:

| grid | Mvox | MiB | ours | seed + JFA | propagate alone |
|---|---|---|---|---|---|
| 423×427×439 @ 5 mm | 79.3 | 303 | **15.6 ms** | 21.9 ms | 18.1 ms |

JFA pays `log2(longest axis)` passes over the whole volume; a BVH query on
a far-from-anything voxel terminates almost immediately, so exact
throughput RISES with emptiness (5.1 Gvoxel/s here against 3.2 at part
scale). The env field needs no different technique from the part field, and
block-sparse storage stays untriggered: 2 m at 5 mm is 303 MiB on 31 GiB.

### The remaining caveat

JFA exists to update incrementally from a camera at frame rate;
rebuilt-from-scratch on CAD is not its design point, and the seeding cost
charged above is this harness's subdivision pass, not code they ship.

## 2. Cold and warm

| | dog_box @ 1.5 mm |
|---|---|
| tessellate (OCCT, 0.2 mm deflection) | 441 ms |
| first bake, including Warp kernel load | 372 ms |
| every bake after | **5.2 ms** |

**This corrects a number reported earlier.** The M2 commit quoted 0.40 s for
the bake. That was one cold call and is almost entirely Warp compiling and
loading the kernel; the bake is 5 ms. Cold cost is paid once per process, so
it belongs in a startup line rather than a per-part budget -- and it is the
strongest argument yet for the long-lived worker M9 already wanted.

Tessellation is now the slow half of preparing a part, at 441 ms against the
field's 5 ms. If part preparation ever needs to be faster, that is where the
time is.

## 3. Query throughput, in the optimizer's own shape

What matters for M5 is not how fast the field can be sampled -- it is how
fast a whole optimizer iteration's worth of queries goes through the
BACKWARD pass. The refinement needs dcost/dq, and `grid_sample`'s backward
scatters through atomics where its forward only gathers. Forward alone would
flatter the field by exactly that cost.

Shapes are `seeds x waypoints x spheres`, which is cuRoboV2's
`[batch, horizon, num_spheres]` under another name. Part field at 1.5 mm,
fp32, resident on device.

| seeds x waypoints x spheres | points | forward | + backward | Gpts/s | iterations/s |
|---|---|---|---|---|---|
| 1 x 160 x 60 | 0.01M | 0.12 ms | 0.40 ms | 0.02 | 2490 |
| 64 x 160 x 60 | 0.61M | 0.13 ms | 0.36 ms | 1.68 | 2740 |
| 256 x 160 x 60 | 2.46M | 0.36 ms | 0.93 ms | 2.66 | 1080 |
| 64 x 160 x **500** | 5.12M | 0.85 ms | 2.31 ms | 2.22 | **430** |
| 64 x 160 x **1000** | 10.24M | ~2.1 ms | ~5.5 ms | ~1.9 | **~180** |
| 256 x 160 x 500 | 20.48M | ~4.2 ms | ~11.1 ms | ~1.8 | ~90 |

These are ~15% slower than the first published table. That is the price of
making the outside-the-box extension conservative (`sample`'s docstring has
the argument): the sum of boundary value and exit distance is a triangle-
inequality UPPER bound and overstated clearance by up to 88 mm out there,
which for a collision field is the unsafe direction. A `where`+`maximum`
per query buys understated-never-overstated, and 430 iterations per second
with a 500-sphere cell model still leaves the budget untouched.

**Sphere count is not a constraint on how the robot is modelled.** 500
spheres is a whole-cell model -- arm, torch, fixture and the second bay --
against the ~60 an arm alone needs, and 64 seeds of a 160-waypoint seam still
runs 430 optimizer iterations per second, so a 200-iteration refinement is
under half a second of collision queries. At 1000 spheres, past anything a
fitted model should reach, it is ~1.1 s. Fit the model for accuracy and stop
thinking about the budget.

**Batching across seeds is free up to ~1M points.** One seed and 64 seeds
both take 0.3 ms: below a million points the pass is launch-overhead bound,
not memory bound. That is `ARCHITECTURE.md`'s "batch across seams and seeds,
not within one" turned from a principle into a measurement -- the seed count
should be chosen by what the search needs, not by what it costs. Past ~2M
points throughput saturates at 1.9-2.9 Gpts/s and time scales linearly, which
is the memory-bound regime and the expected one.

It also localizes the next speedup: the flat region is kernel launches, so
CUDA graph capture (M9) attacks it and a bigger grid does not.

### The query, head to head after all

An earlier revision of this file claimed standing up cuRoboV2's production
query faithfully was "a larger job than the answer is worth". It was ~150
lines using the recipe out of their own tests, and the claim lasted until
somebody asked why. `tools/bench_query_curobo.py`; bracket_a at 1.0 mm,
same query spheres for all three, gradient included everywhere.

Three contenders, because their checker takes obstacles in two forms:

- **ours** -- exact baked field, `grid_sample`, gradient by autograd.
- **curobo-voxel** -- their production checker over a `VoxelGrid` obstacle
  whose feature tensor is OUR exact field (fp16, their kernel's
  requirement), so the two table paths query identical data and the row
  measures machinery alone: fused kernel with in-pass analytic gradient
  against interpolate-then-autograd.
- **curobo-mesh** -- their checker over the same triangles as a `Mesh`
  obstacle: no table at all, a BVH walk per sphere per evaluation.

| seeds x waypoints x spheres | points | ours-autograd | ours-fused | curobo-voxel | curobo-mesh |
|---|---|---|---|---|---|
| 1 x 160 x 60 | 0.01M | 0.39 ms | 0.29 ms | 0.30 ms | 0.36 ms |
| 64 x 160 x 60 | 0.61M | 0.38 ms | **0.26 ms** | 0.28 ms | 1.35 ms |
| 256 x 160 x 60 | 2.46M | 1.01 ms | 0.52 ms | **0.44 ms** | 4.55 ms |
| 64 x 160 x 500 | 5.12M | 2.39 ms | 0.88 ms | **0.63 ms** | 9.06 ms |
| 64 x 160 x 1000 | 10.24M | 6.00 ms | 1.66 ms | **1.04 ms** | 17.84 ms |

`ours-fused` did not exist when the first version of this table was taken.
The measurement said the gap was the kernel shape, so the shape was ported:
`sample_fused` computes the trilinear value and its analytic gradient in
one pass -- the eight corners are in registers anyway, the derivative is a
handful of extra multiplies -- and its backward is a buffer read behind a
custom autograd Function, exactly cuRobo's arrangement. Same values, same
conservative extension, gradient parity to 5e-3 against autograd. (The
first cut of the kernel crossed two corner indices in the y-derivative and
was wrong by up to 35; values matched perfectly throughout, because the
value path never touches the broken term. Only the gradient parity test
caught it, which is why that test is permanent.)

Findings, updated:

**Fusing the gradient into the kernel was worth 3.6x** (6.00 to 1.66 ms at
the cell-scale sphere count), taken as ~150 lines with no change of data,
semantics, or storage. The optimizer's collision term should use
`sample_fused`; `sample` remains the reference the parity tests hold it to
and the CPU path.

**cuRobo's kernel keeps a 1.6x edge at the largest batches.** Two known
reasons: their table is fp16 (half the memory traffic of our fp32) and
their activation shaping is inside the kernel where ours is torch ops
outside. Both are portable if that last 1.6x is ever worth having; at 1.66
ms per 10M-point iteration it is not.

**A table beats a tree at optimizer batch sizes.** Their own mesh path --
BVH per sphere per evaluation -- trails every table variant by 3-10x on a
mere 548 triangles, and finer tessellation deepens the walk.

## 4. The collision stage end to end

`pixi run -e motion motion-collision`. Everything above measured the field
alone. This is what the optimizer will actually call: forward kinematics
over the arm, the authored sphere model carried along by it, one fused
field query per sphere per timestep, and a gradient all the way back to the
joint angles.

`rosie_1400_v3`, 300 spheres (50 a link over `link_1..link_6`), the
world-static env baked from `floor` + `h_frame` + `positioner_table_b`.
`positioner_table_a` is deliberately not in it: it rides J7 and belongs to
the workpiece frame's field.

    bake, 5 mm, 12159 triangles     380 ms   644 x 381 x 212 = 52.0M voxels, 198 MiB
    128 timesteps x 300 spheres     3.09 ms per iteration, forward + backward
    256 timesteps x 300 spheres     3.03 ms per iteration
    forward only, 38400 points      FK 0.64 ms, field query 0.08 ms

**The field was not the bottleneck; the arm chain was.** The query came out
eight times cheaper than the kinematics feeding it, and doubling the
timestep count did not move the per-iteration cost at all -- launch
overhead over a few dozen small matmuls, not memory traffic over the table.

### Fusing the kinematics, 7.7x

`weld_motion_planner.robot_cell.robot.fk`: one Warp kernel from joint angles straight to world
sphere centres, one thread per (pose, sphere), each walking its own chain.
Threads on the same link redo each other's arithmetic, which is the right
trade -- a handful of 3x3 multiplies is free next to the launch and the
global round-trip that sharing them would need. Backward is analytic,
`dp/dq_j = a_j x (p - o_j)` per revolute joint, written by the forward pass
so the backward kernel is a dot product and an atomic add.

    forward only, 38400 points   FK torch 0.86 ms   FK fused 0.05 ms   field 0.07 ms
    full iteration, fwd + bwd    torch chain 3.12 ms   fused chain 0.40 ms

Seventeen times on the kinematics alone, 7.7x on the whole iteration.
95.1M differentiated sphere-queries a second; 200 iterations of a 128-step
trajectory in 81 ms, from 618. The two halves are now balanced -- 0.05 ms
of kinematics against 0.07 ms of field -- so the next win is not in either.

Correctness against a CPU float64 reference: values agree to 8.3e-5 mm,
gradient to 1.2e-7 relative against autograd through that reference. The
reference runs on the CPU deliberately: torch in this environment cannot
reach triton (no `cuda.h`), and the first GPU autograd op wanting a JIT
aborts the process.

**The fused kernel is exact on real arm points.** Against the reference
`sample` path, on the 300 sphere centres the arm actually produces: value
max difference 0.0001 mm, gradient max difference 0.0000. Worth stating on
*these* points specifically, because 255 of the 300 lie outside the grid
box, where the conservative extension takes over -- a case a synthetic
point cloud inside the box would never exercise.

**The composed gradient is right, and finite differences are not the
judge.** d(sum of clearances)/dq matches central differences to 1e-4 at
eps 1e-3, with the classic U-shape either side of it (trilinear kinks
above, float32 cancellation below). The hinge cost `relu(m - d)^2` shows
much larger disagreement that does not reliably shrink with eps -- and it
is not the extension: measured, zero of the 27 spheres inside the 50 mm
margin were outside the box. It is the field itself. Trilinear
interpolation is C0, so its gradient is piecewise constant with a jump at
every voxel face; over 300 spheres those jumps average out, and the hinge
concentrates the cost on a couple of dozen where they do not. This is
inherent to a trilinear ESDF, cuRobo included, and is not a defect in the
kernel.

**The zero pose had 18 spheres in collision at -42.3 mm** before
`weld_motion_planner.robot_cell.collision.pairs` existed. Correct arithmetic, useless answer: `link_1`
is bolted to `floor` and its cover includes the mating face, so an adjacent
pair reads as a collision at every pose. With the adjacency rule applied
the zero pose is clean -- 183.5 mm minimum clearance, nothing in
collision -- across the 250 spheres on the five links that remain in the
environment check.

### Head to head against cuRoboV2, same robot, same spheres

`tools/bench_fk_curobo.py`. cuRobo gets OUR cell -- the same `robot.urdf`,
the same six axes, and the same 300 authored spheres handed to it as
`collision_spheres` -- rather than their Franka against our arm. Both
answer the same question: batch of joint vectors in, world sphere centres
out, differentiably. The zero-pose centres agree to **0.0000 mm** over all
300, so the timings are of two things that compute the same answer.

Both sides get a CUDA graph, and both are charged the copy of this
iteration's joints into the static buffer a graph has to replay against.
Timing our captured path against their eager one would be measuring the
harness. The eager columns are kept because they isolate the kernels from
the capture.

    batch    ours fwd   cuRobo fwd   ours eager   cuRobo eager   ours f+b   cuRobo f+b
      128    0.009 ms     0.008 ms     0.031 ms       0.085 ms   0.066 ms     0.318 ms
      512    0.012 ms     0.011 ms     0.038 ms       0.077 ms   0.066 ms     0.288 ms
     1024    0.014 ms     0.008 ms     0.039 ms       0.078 ms   0.075 ms     0.299 ms
     4096    0.019 ms     0.012 ms     0.038 ms       0.077 ms   0.107 ms     0.297 ms
    16384    0.045 ms     0.067 ms     0.050 ms       0.078 ms   0.517 ms     0.659 ms
    65536    0.247 ms     0.249 ms     0.240 ms       0.260 ms   2.862 ms     3.447 ms

**Forward and backward together -- what a gradient method actually runs --
we are ahead everywhere by 2.7x to 4.8x.** Eager against eager, forward
alone, ahead everywhere by 1.5x to 2.7x.

**Forward alone with both captured is mixed and they are ahead in the
middle.** Level to about 512 poses, theirs by 1.5-1.7x from 1k to 4k, ours
by 1.5x at 16k, level again at 64k. Their forward does the whole thing in
one graph node; ours needs two above the crossover, and that node is worth
about five microseconds at a size where the kernels take ten.

Getting here took four changes, and the first two matter more than any
arithmetic:

**The driver was the cost.** Batch 1 and batch 16384 both took 0.08 ms on
both sides, which is what an overhead-bound launch looks like. Building the
warp array wrappers once instead of per launch took eager forward from
0.085 ms to 0.038; capturing into a CUDA graph took it to 0.010.

**Two stages above the crossover, one below.** One thread per (pose, link)
for the transforms and one per (pose, sphere) to apply them costs
`poses x (links x depth + spheres)` and wins once there is real work; one
thread per sphere walking its own chain costs `poses x spheres x depth` and
wins while the graph node is the cost. Measured crossover: about a hundred
thousand sphere-updates. The single-kernel path still emits link
transforms, from one designated thread per link -- letting all three
hundred spheres write them was correct, since every writer stores identical
bits, and cost more than the walk it rode along with.

**No Jacobian buffer.** `a x (p - o) = a x p - a x o`, and the second term
is per JOINT, not per sphere. Stage one stores the axis and that moment --
a few kilobytes -- and backward rebuilds each column. The buffer it
replaced was 177 MB at 16k poses, written in forward and read back in
backward. Backward also folds eight spheres per thread before touching
`grad_q`, because one thread per sphere aimed 29M atomic adds at a handful
of slots.

Link poses come out as a first-class result either way (`fn.link_poses(q)`),
which the tool pose and the seam frames want anyway.

Two things still flatter us. Their kernel emits quaternions as well as link
poses, which ours does not. And their forward+backward goes through an
autograd wrapper carrying buffers ours does not.

Reproducing it needs two packages the `bench` environment does not carry,
installed to a scratch prefix rather than into the env: `cuda-core[cu12]`
for cuRobo's kernel backend, and `yourdfpy` for their URDF parser.

### And the collision stage, again

With the fused FK in place, `motion-collision` on the same 128 x 300 shape:

    forward only    FK torch 0.86 ms   FK fused 0.01 ms   field query 0.08 ms
    full iteration  torch chain 3.16 ms   fused chain 0.32 ms   (10.0x)

121.7M differentiated sphere-queries a second. 200 iterations of a 128-step
trajectory in 63 ms, from 618 where this started. The field query is now
eight times the cost of the kinematics -- the opposite of where this began,
and where the next work is if it is ever needed.

## 5. The two kernels together, against the mesh

`pixi run -e motion motion-pipeline`. Random arm poses through the fused FK
into the fused ESDF, producing per-sphere clearance and its derivative in
joint space -- the collision term exactly as the optimizer will call it.

Poses are rejection-sampled to be near something. Uniform random joints put
the arm in the air, where the term is flat and any implementation looks
accurate; these are 256 poses whose nearest sphere is within 100 mm, giving
76 800 clearances of which 4 677 are in collision and 7 082 sit inside the
50 mm margin that the cost can see.

Accuracy is against the MESH, not against another implementation:
`mesh_query_point_sign_winding_number` gives the exact signed distance and
the exact closest point, on the same BVH the field was baked from but with
no grid in between.

### Value

    inside the grid   n=16 647   mean 0.023 mm   p99 0.136 mm   max 1.618 mm
    outside it        max signed overestimate  -0.028 mm

Mean error is half a percent of a 5 mm voxel and the worst is a third of
one. Outside the grid the check is not magnitude but SIGN: the extension is
documented as a lower bound, and the largest overestimate over 60 153
samples is negative, so it never claims more clearance than there is.

### Gradient

Both sides are pushed back through the same FK backward, so what is left is
the field's spatial gradient against the exact one. The active set only --
a sphere the cost cannot see contributes nothing.

    spatial direction   mean cos 0.99956   p1 0.99736   worst 0.75151
    joint-space dd/dq   mean relative 1.8e-4   worst relative 2.1e-2

Eleven of 7 082 active spheres fall below cos 0.99, and their distance to
the surface averages 27.6 mm with a minimum of 2.2 mm -- the direction is
worst where the distance is near zero, which is where the true gradient is
genuinely ambiguous because two surfaces are equally near. No interpolant
picks a side there.

### Cost, and where it now goes

Nested, so each column's increment is that stage's own cost with backward
included:

    batch   FK f+b     +field f+b   +hinge f+b   M clearance/s   200 iters
      128   0.105 ms     0.278 ms     0.345 ms           111.4       69 ms
     1024   0.072 ms     0.274 ms     0.336 ms           913.3       67 ms
     8192   0.188 ms     0.500 ms     0.545 ms          4505.3      109 ms

**The field round trip is now the largest piece**, at about 0.17 ms of the
0.34, having started this exercise as the smallest. The hinge adds another
0.06 as separate torch kernels, each with its own launch and its own
backward node -- cuRobo shapes the activation inside the collision kernel,
which is the same trick as everything else here and the obvious next one.

## 6. Self collision

`rosie_1400_v3`, 300 spheres over six links. The adjacency mask leaves
25 000 pairs of the 44 850 an all-pairs check would take -- two links across
a bearing overlap at the mating face by construction, so checking them
reports a collision at every reachable pose.

Written in plain torch first and measured, which is why it is a kernel now:

    batch    torch fwd   fused fwd    torch f+b   fused f+b   +zero-grad skip
      128     0.129 ms    0.106 ms     0.558 ms    0.403 ms          0.347 ms
     1024     1.591 ms    0.148 ms     6.566 ms    3.790 ms          1.672 ms

The torch version gathers both endpoints into `(B, P, 3)` tensors -- 38 MB
at 128 poses, 307 MB at 1024 -- to do a subtract and a norm. That is
memory-bound and it went superlinear: eight times the batch cost twelve
times the wall clock. The kernel reads two points and writes one float, and
forward is nearly flat.

Backward is atomics: every sphere sits in about 167 pairs, and each pair
scatters into two of them. Under a hinge cost most pairs are far apart and
their incoming gradient is exactly zero, so those return before touching
global memory -- worth 2.3x at batch 1024 and nothing at all to the answer,
which a test asserts.

Correctness: the hand-written backward agrees with the gather-and-norm it
replaced to 0.00e+00 on a pose worked out on paper -- both elbows at pi put
two spheres 50 mm apart against radii summing to 110, and the reported
overlap is -60.0 mm.

## 7. The collision term, assembled

`pixi run -e motion motion-cost`. `rosie_1400_v3` with `bracket_a_placed`:
300 spheres against three fields (world, part, fixtures) plus 25 000
self-collision pairs, forward and backward.

    batch   env only   + self    + hinge   + augmented Lagrangian
      128   1.957 ms   2.329 ms  2.524 ms                2.517 ms
     1024   1.999 ms   4.090 ms  2.842 ms                3.139 ms

Two things in that table are worth reading twice.

**At batch 1024 the hinge is FASTER than the plain sum it sits on top of**
-- 2.842 ms against 4.090. Summing the clearances gives every one of the
25 000 pairs a gradient of exactly 1, which defeats the self-collision
backward's zero-gradient skip; the hinge zeroes most of them and the
atomics go away. The `+ self` column is a pathological case, not a
baseline.

**The multi-field path costs 1.96 ms against `bench_collision`'s 0.34 ms
for one field.** Three field queries account for 0.23 of that. The rest is
the same disease as everywhere else in this package: `clearance` runs about
seventeen small torch launches per call for the group masking and the frame
transform.

One piece of that was a real defect and is fixed. `fk.torch_link_transform`
called `torch.as_tensor` on the chain's numpy constants EVERY CALL -- a
host-to-device copy per invocation -- and cost 0.285 ms against 0.013 ms
for the entire fused kinematics, for a chain three joints long. Caching the
upload and composing the work frame once per `clearance` instead of once
per workpiece group took the term from 2.92 ms to 1.96 at batch 128.

What is left is a fused multi-field kernel: one launch from world points to
per-sphere cost, taking the min across groups and applying the hinge inside.
Measured and named rather than assumed; it is not on M2's critical path.

## 8. The T2 verifier, and the backend spike behind it

`tools/spike_ccd.py`, on the real cell meshes: `link_3` (1 362 tris) against
`h_frame` (9 469). Five questions, ordered so that failing an early one makes
the later numbers moot.

| | python-fcl | coal | pybullet (concave) | pybullet (convex) |
|---|---|---|---|---|
| mesh-vs-mesh answered | 0.2500 m | 0.2500 m | **declined** | **declined** |
| mesh-vs-primitive | 0.2500 m | 0.2500 m | 0.2490 m | 0.2490 m |
| two boxes 250 mm apart | 0.000 mm error | 0.000 mm error | — | — |
| probe in a slot (exact 40 mm) | 40.0 mm | 40.0 mm | — | — |
| distance query | 219 µs | 234 µs | — | — |
| swept CCD, sphere control | **toc 0.400 ✓** | absent | absent | absent |
| swept CCD, mesh pair | **no collision** | absent | absent | absent |

The last two rows are the finding. Dense sampling puts the first contact of
that mesh motion at `t = 0.133`; `continuousCollide` reports the pair never
touches. It gets the sphere control exactly right, so this is FCL's answer and
not a mis-driven API. Separately, a straight tunnelling case — endpoints clear
by 339 and 408 mm, a certain crossing in between — comes back clear too.

pybullet declining every mesh-vs-mesh pair while answering mesh-vs-primitive
is what removes it: not slow, not approximate, absent. The 2.7 µs it posts on
"throughput" is the cost of returning nothing 2 000 times.

**Query cost against separation**, which sets the verifier's cost model:

| gap | python-fcl | coal |
|---|---|---|
| 5 mm | 159 µs | 175 µs |
| 50 mm | 199 µs | 225 µs |
| 500 mm | 489 µs | 527 µs |

Backwards from the intuition: a *far* pair is 3× more expensive than a near
one, because the BVH's running bound prunes hard when something is close and
barely at all when every triangle is at a similar distance.

### The predicate, which is what the verifier actually runs

`tools/spike_lower_bound.py`, 4 000 random poses of `link_3` around
`h_frame`, 891 of them penetrating:

| | cost | vs exact | agrees with `distance()` |
|---|---|---|---|
| exact distance | 228 µs | 1× | — |
| `distance_lower_bound` | 8.4 µs | 27× | sound, 0 violations |
| "closer than 1 mm?" | 8.9 µs | 26× | all 4 000 |
| "closer than 5 mm?" | 10.0 µs | 23× | all 4 000 |
| "closer than 20 mm?" | 14.0 µs | 16× | all 4 000 |
| "closer than 50 mm?" | 26.4 µs | 8.6× | all 4 000 |
| "closer than 200 mm?" | 434 µs | **0.5×** | all 4 000 |

The lower bound is sound but loose — median 7% of the true distance — so
conservative advancement uses the *predicate*, which is exact as a predicate
and stays cheap while the margin stays small. At 200 mm it inverts: inflating
that far makes almost everything "near" and the traversal descends anyway.

### The verifier on the real cell

`tools/bench_verify.py`, `rosie_1400_v3` with `bracket_a_placed`: 12 bodies,
54 checked pairs after adjacency, 19 269 triangles. BVH build 53 ms.

| joint step | ms/segment | queries/segment | workers=8 |
|---|---|---|---|
| 1 mrad | 1.23 | 108.8 | 2.94 |
| 10 mrad | 1.65 | 110.3 | 3.15 |
| 100 mrad | 107.1 | 235.3 | **28.0** |
| 500 mrad | 747.5 | 465.0 | **178.0** |

At weld-scale steps every pair certifies in one query and the segment costs
1.4 ms. The large-step rows are trajectories that actually collide, where the
bisection descends and the clearance report pays for exact distances.

The pool is a 3.8–4.2× win exactly there and a **loss** on cheap segments,
because each worker rebuilds all 12 BVHs. Serial is the default; `workers` is
opt-in until there is a persistent pool. 20 workers buy nothing over 8 on a
64-segment trajectory — there is not enough work to hide the rebuild.

Two defects found by measuring rather than assuming, worth 15× between them:

- The clearance number in the report ran an exact distance per pair *per
  segment* — 54 queries a segment, about half the verifier's cost, for a line of
  output. Screening with the predicate at 50 mm and measuring exactly only
  what trips the screen: 21.3 → 7.4 ms.
- Each query recomputed forward kinematics for all 12 bodies, and the pairs
  are certified independently, so 54 pairs asked for `t = 0.5` and got 54
  identical FK evaluations. Memoizing by `t` and placing only the two bodies
  under test: 7.4 → 1.43 ms.

## 9. Inverse kinematics, which the graph is mostly made of

`weld_motion_planner.robot_cell.robot.ik`, on `rosie_1400_v3`'s six-axis chain. Targets generated by
running random joint values forward, so a solution provably exists and every
miss belongs to the solver.

| batch | fp64 | fp32 | fp32 µs/pose |
|---|---|---|---|
| 256 | 80.7 ms | 82.5 ms | 322 |
| 2 048 | 82.0 ms | 85.2 ms | 41.6 |
| 16 384 | 150.0 ms | 82.3 ms | 5.02 |
| 65 536 | 613.2 ms | 123.3 ms | **1.88** |

**Batch 256 and batch 2 048 cost the same wall time.** That is the signature
this repo has now hit three times — the driver, not the GPU. Sixty iterations,
each walking the chain twice for kinematics and once more for the Jacobian, is
thousands of small launches. Packing the Jacobian's columns into one scatter
instead of `2K` indexed writes was worth 5 ms of 87 and no more; what remains
is the chain walks, and the fix for those is graph capture as `fk.py` already
does. Named with its measurement, not built.

fp64 at 65 536 costs 5× fp32, which is the consumer card's 1/64 double-rate
showing up. `ARCHITECTURE.md` puts screening in fp32 for exactly this reason,
and the verifier is what carries fp64.

### Seeds, not iterations

Solve rate over 2 048 provably-reachable poses:

| seeding | rate |
|---|---|
| one nominal seed | 67.4 % |
| eight structured branches | 95.1 % |
| eight random | 95.8 % |
| sixteen random | 99.2 % |
| twenty-four random | 99.7 % |
| seeded at the answer | 100 % |

Raising the iteration cap from 60 to 400 moves eight-seed coverage from
95.07 % to 95.31 %. The last few percent are basins, not patience. The
structured branches are not better per seed than random ones — they are
**reproducible**, which is what `ARCHITECTURE.md` asks of the graph; random
restarts are the lever for closing the tail.

This matters less than it looks for the graph itself: seam samples are ~5 mm
apart, so each layer warm-starts from the previous one's answer and the cold
rate applies only to the first layer.

This matters less than it looks for the graph itself: seam samples are ~5 mm
apart, and section 10 measures what the graph actually does with that.

## 10. The lattice and the sweep

`tools/bench_graph.py`, on `dog_box` seam 1 unless a row says otherwise. 32
samples, a 25 degree edge threshold, median of three after a warm run.

**These numbers moved twice in one day and the second move inverted them**, so
read the shape rather than memorising the figures. The lattice build used to be
90% of the cost; closed-form inverse kinematics made it ~1%, and the sweep — the
stage nobody was optimising — became everything.

### Bins

| bins | gap | poses | live nodes / layer | build | search | peak | crossed |
|---:|---:|---:|---:|---:|---:|---:|---|
| 15 | 24.0° | 7 200 | 420 | 0.02 s | 0.006 s | 0.06 GiB | yes |
| 32 | 11.2° | 32 768 | 1 888 | 0.02 s | 0.021 s | 0.23 GiB | yes |
| 64 | 5.6° | 131 072 | 7 530 | 0.03 s | 0.270 s | 0.29 GiB | yes |
| 96 | 3.8° | 294 912 | 16 970 | 0.05 s | 1.363 s | 0.55 GiB | yes |
| 128 | 2.8° | 524 288 | 30 146 | 0.08 s | **4.318 s** | 0.97 GiB | yes |

The build is now flat to a rounding error — 20 to 80 ms across a
seventy-fold range of poses — because closed form has no iteration to scale.
The sweep is quadratic in live nodes which are quadratic in bins, so it is
quartic and it now owns the curve.

**Live nodes doubled** when the closed form landed: 17 263 to 30 146 at 128
bins. The eight structured seeds had been converging onto the same roots, and
the duplicate count fell from 10 251 to 34 when they stopped. Twice the lattice
is twice the search's `N`, hence four times its work — the 128-bin sweep went
1.15 s to 4.32. **That is a good trade and it should be read as one:** the extra
nodes are real postures the planner could not previously choose between.

### What each fix bought, at 64 bins

| | build | search | peak | vs shipped |
|---|---:|---:|---:|---:|
| shipped | 0.04 s | 0.273 s | 0.40 GiB | |
| no reach prefilter | 0.03 s | 0.272 s | 0.40 GiB | 0.98× |
| float64 screen | 0.09 s | 0.273 s | | 1.18× |
| one transition slice | 0.04 s | 0.411 s | 5.59 GiB | 1.45× |

**Two of these are now worth nothing, and that is the honest reading.** The
reach prefilter was 1.74× and the fp32 screen 5.24× when the build was 90% of
the cost. The build is 12% of it now, so skipping 28.5% of the poses saves 12%
of 12%. Anything measured against a bottleneck expires when the bottleneck
moves.

The prefilter turned out to be worse than worthless and **no longer runs** under
the closed form. It was costing 3.3 ms of a 90 ms build to compute a mask the
closed-form branch never read — and the closed form SUBSUMES it anyway: the
`arccos` leaves range exactly when the triangle cannot close, which is the wrist
centre being out of reach, and the `arcsin` leaves range exactly when the centre
is inside the shoulder's offset cylinder. Those are the reach conditions,
computed exactly rather than against a sampled bound, as part of solving. Both
paths now measure 90 ms and agree to the float. It stays wired for the iterative
solver, where a skipped pose is thirty-five iterations not run.

Chunking went the other way. It was 1.06× and is now 1.45× and the difference
between running and not.

### Transition chunk size, at 96 bins

| budget | slices | elements / slice | time | peak |
|---:|---:|---:|---:|---:|
| 1 MiB | 16 970 | 118 790 | **42.945 s** | 0.12 GiB |
| 64 MiB | 653 | 3 088 540 | 1.990 s | 0.15 GiB |
| **256 MiB** | 164 | 12 354 160 | **1.358 s** | 0.28 GiB |
| 1 GiB | 41 | 49 416 640 | 1.817 s | 0.80 GiB |
| one slice | 1 | 2 015 866 300 | — | **out of memory** |

The unsliced row no longer runs at all. It was 0.555 s and 9.28 GiB when the
lattice was half this size; the table is quadratic, so doubling the nodes
quadrupled it past the card. Slicing stopped being an optimisation and became
the reason the search works.

Both ends are still bad for their own reasons — too fine and the card idles
between launches, 32× slower at one destination per slice; too coarse and the
allocation costs more than the parallelism buys. Saturation is around three
million elements, about a dozen per resident thread. 256 MiB sits four times
past it.

Worth stating plainly because it is the intuition this contradicts: **slicing
did not make a parallel program sequential.** A two-billion-element tensor is
already processed in thousands of rounds by a card that holds 261 120 threads.
The sequencing is the hardware's; chunking only stops paying memory for the
illusion of doing it all at once.

### Pruning by positioner bin

The positioner is a component of the search state with its own threshold, so
`|bin_i - bin_j| * gap > threshold` is a guaranteed non-edge decided by two
integers — no joints, no distance, no allocation. On `dog_box` seam 1, sweep
only:

| bins | live nodes | pairs it removes | before | after | |
|---:|---:|---:|---:|---:|---:|
| 15 | 420 | 80.0% | 0.006 s | 0.008 s | 0.7× |
| 32 | 1 888 | 84.4% | 0.021 s | 0.026 s | 0.8× |
| 64 | 7 530 | 85.9% | 0.274 s | 0.146 s | 1.9× |
| 96 | 16 970 | 86.5% | 1.368 s | 0.389 s | **3.5×** |
| 128 | 30 146 | 86.7% | 4.317 s | **1.011 s** | **4.3×** |

Exact, not a heuristic: costs are equal at every size and the frontier and the
reachable counts with them. Paths can swap between representatives of the same
cost, because dropping pairs changes the order `topk` sees — the cost is the
answer and the path is one of its equals.

Below 64 bins it LOSES, and that is the shape to remember rather than the
numbers. The saving is proportional to the table; the cost is per chunk, and a
small lattice has more chunks than table. It is left on because the sizes where
it loses are the sizes where the sweep is already six milliseconds.

**4.3× is not the 7.5× the pair count promises**, and the gap is kernel
launches. The first version built an index tensor per chunk and got 1.7×;
slicing a ring laid out twice took it to 2.1×; sizing the chunks for the band
that will actually be seen rather than for all sources took it to 4.3×. Each of
those three was launch overhead, not arithmetic.

### The whole corpus

Fifteen seams, three from each committed plan request, at the shipped configuration
— 15 bins from `bins_for` of the 25 degree threshold, samples timed at 100 ms
of travel apiece, no collision screening.

| | |
|---|---|
| crossed | **15 of 15** — screening OFF; `docs/issues/0006` has the screened number |
| build | 0.24 s |
| search | 0.10 s |
| **per seam** | **22 ms average, 31 ms worst** |

Was 109 ms a seam before the closed form.

### Where the time goes at 128 bins

Profiled per stage with `nvidia-smi` sampled every 20 ms, on the ITERATIVE
path, which is why its total is 12.55 s against the 4.4 s above. Kept because
the shares are what it is here for, and because an arm without a closed form
still runs exactly this:

| stage | time | share | peak | GPU |
|---|---:|---:|---:|---:|
| target poses | 0.02 s | 0.2% | 0.25 GiB | 21% |
| reach prefilter | 0.01 s | 0.1% | 0.16 GiB | – |
| **inverse kinematics** | **11.40 s** | **90.8%** | 7.57 GiB | **99%** |
| causes and duplicates | 0.00 s | 0.0% | 0.62 GiB | – |
| reshape to the grid | 0.00 s | 0.0% | 0.78 GiB | – |
| **the sweep** | **1.12 s** | **8.9%** | 1.20 GiB | **100%** |

The card runs at 97% average and 100% peak through both heavy stages, so there
was never idle to reclaim and every gain has come from doing less work rather
than more of it at once. Closed form is the extreme case of that: it does not
do the work at all.

**Tensor cores are not used and should not be.** Turning on TF32 is 3× slower
and loses 46% of the feasible nodes: the matrices are 3×3 and 6×6 against a
16×16×16 tile, and TF32's ten-bit mantissa resolves 977 µm at this cell's
two-metre scale against a 100 µm tolerance. Two independent reasons, either
sufficient. `tests/motion_planner/test_nodes.py` asserts the flag stays off, because it
is process-wide and a lattice that lost half its nodes looks exactly like a part
that is hard to weld.

### Inverse kinematics, closed form against iterative

`tests/motion_planner/test_ik_closed.py`, and the reason the tables above changed shape.

| poses | iterative | closed form | |
|---:|---:|---:|---:|
| 4 096 | 128.4 ms | 0.49 ms | 261× |
| 65 536 | 1 395.5 ms | 1.62 ms | 863× |
| 524 288 | 11 754.6 ms | 17.07 ms | 689× |

Accuracy is not a straight win and the shape is the point. Residuals over 16 384
targets, both against the same kinematics:

| | median | 90% | 99% | worst |
|---|---:|---:|---:|---:|
| iterative | **0.000 µm** | 7.698 | 80.242 | **99.98 µm** |
| closed form | 0.497 µm | **0.545** | **0.723** | **46.90 µm** |

The iterative solver is exact when it converges cleanly and stops the moment it
is under tolerance, so its worst case IS the tolerance. The closed form has no
tolerance to stop at, so its distribution is narrow. Typical case to one, worst
case and predictability to the other.

That 0.497 µm was then removed. It was not floating point — every measurement
ran in float64 — it was the solver assuming the arm's side offsets cancel when
this arm's two numbers, `-0.125500489` and `+0.125499999`, miss by 490 nm.
Carrying the leftover as a term makes the closed form **exact**: 0.000 nm and
1.8e-13 of rotation over 14 116 solutions. The 60 nm that survives against
`ChainSpec` is `ChainSpec`, whose origins are float32.

### The collision screen

Measured after the fact, and it changed the module. `dog_box` seam 1, one
process per row so fragmentation cannot leak between them:

| bins | without | with | extra | peak without | peak with |
|---:|---:|---:|---:|---:|---:|
| 15 | 0.11 s | 0.11 s | 0.00 s | 0.09 GiB | 0.55 GiB |
| 64 | 0.50 s | 0.56 s | 0.06 s | 1.01 GiB | 1.32 GiB |
| 96 | 1.22 s | 1.35 s | 0.14 s | 2.23 GiB | 2.48 GiB |
| 128 | 2.33 s | 2.56 s | 0.23 s | 3.93 GiB | 4.19 GiB |

**About 10% in time at every size**, which was never the problem. Before it was
sliced the same rows read 24.23 GiB at 64 bins and out-of-memory at 96 and 128 —
so collision-screened planning was capped at half the resolution the rest of the
build manages, on a stage costing a tenth of its time.

The arm carries 300 spheres, so 64 bins asks for 1 048 576 × 300 = **315 million
point positions**, 3.5 GiB of coordinates before the distance field is consulted,
and `fk.py` pairs every point buffer with a gradient buffer it allocates whether
or not anything is differentiating. Sliced at 256 MiB the peak is flat and the
answers are identical to the float.

Baking the field costs 0.5 s once per part, not per seam.

**The screen currently rejects nothing on `dog_box` seam 1** — the arm's closest
approach to the part is 14.3 mm, and the distribution runs 14.3 to 53.9 mm with
a 38.7 mm median. It starts firing at a 20 mm floor. On `bracket_a_placed` seam
1 it fires at any floor: some postures put the arm 50 mm INSIDE the part. Both
are worth stating because a screen that never fires and a screen that is not
wired up produce the same lattice, and only one of those is a result.

## 11. Trajectory generation (M5), and the invariant it was recomputing

`bracket_a_placed`, 7 crossed seams, `k = 2` candidates each, RTX 5090, phases
timed with `Stopwatch`'s CUDA sync so the numbers mean what they say:

    phase          before    after     what it is
    sweep (M4)        2.6 s    2.6 s   unchanged, and never the problem
    synthesis       315.8 s   48.6 s   the Adam loop, per candidate
    tracking cert    17.9 s   18.2 s   conservative advancement on the CPU
    T2 verifier           3.3 s    3.3 s   coal, over the emitted curve
    TOTAL           339.7 s   72.7 s

**The tell was that the cost did not scale with the seam.** A nine-knot seam
took 45 s and a fifty-six-knot one 41 s, which says the work is per ITERATION
and not per sample — so no amount of shrinking the problem would have helped.
`cProfile` then put 61% of a run inside `targets`, and a third of that inside
`Curve.locate`: the seam side of the target — frames, authored angles,
placement — was being rebuilt on every Adam step, Newton-solving `s -> u` for
the same fractions a hundred and thirteen times over, plus an `fk.build` of
the work chain per call.

The fractions are fixed. The lattice chose them and the optimizer moves
joints, not samples, so all of that is loop-invariant; only the positioner's
own rotation depends on a variable. Splitting the composition at that seam
(`seam_local_targets` for the fixed half, `_seam_world_targets` for the half
that must run) is **6.5× on the synthesis and 4.7× end to end**, with the
outputs bit-identical — same cost curve, same certified bound, same 2611
bisection nodes.

### 11.1 Where the time goes now, at the k the operator uses

`k = 5`, the number the drawer asks the sweep for:

    sweep (M4)         2.2 s     1.6%
    synthesis        110.8 s    82.9%
    tracking cert     17.5 s    13.1%
    T2 verifier            3.2 s     2.4%
    TOTAL            133.7 s

Synthesis scales with `k` and nothing else does — the other two run on the
winner alone. So the operator's wait is `k` × ~16 s per seam plus a fixed
~3 s of judging.

### 11.2 Inside the synthesis loop

240 Adam steps on `seam_0001` (41 knots), every timer CUDA-synced. Rows nest,
so they are "time spent anywhere inside this", not a partition:

    project             242 calls   2.36 ms/call   15.1%
    collision.value     485         1.15          14.8%
    knot_velocity       486         1.10          14.2%
    midpoint_tracking   487         0.82          10.6%
    _clip               240         1.53           9.7%
    tool_pose           733         0.47           9.1%
    targets            1219         0.28           9.0%
    smoothness          488         0.12           1.5%
    residual (backward, Adam, Python)             14.7%

**No hotspot, and that is the finding.** Nothing costs more than 2.4 ms and
everything is small, on tensors of 41×7. Note also the call counts: 486
`knot_velocity` for 240 steps. That is what the monotone accept/reject loop
costs — every step evaluates the objective a second time, after clip and
projection, and roughly 20% of the loop is that second evaluation. It buys
the guarantee that a healthy seam cannot be damaged, and the price is now
stated rather than assumed.

### 11.3 The loop is launch-bound, not compute-bound

30 steps under `torch.profiler`:

    self CPU time        546 ms
    self CUDA time        95 ms      <- the actual arithmetic
    cudaLaunchKernel   50 569 calls, 2.55 us each, 22.7% of CPU time
    aten::select       26 083 calls  <- `_clip`'s Python loop over knots

The card does 95 ms of work in a run that takes 546 ms of CPU, and the
average kernel lasts 1–4.5 µs against 2.55 µs to launch it. **The launch
costs as much as the kernel.** Roughly 1 700 launches per Adam step.

That names the fixes and their order, and both were already written down as
deferrals before there was a measurement to justify them:

1. **Batch the candidates and the seams into one tensor** (M5's own
   "correctness first; the batching is a reshape"). Fourteen candidate-runs
   of 41×7 launch fourteen times as many kernels as one run of 14×41×7 and
   ask the GPU for the same arithmetic. This is the `k`-shaped cost in 11.1
   collapsing to nearly nothing.
2. **CUDA-graph capture of the inner step** — M9 lists it conditionally, "if
   launch-bound". It is.
3. `_clip`'s two Python sweeps over the knots, which is where those 26 083
   `select` calls come from.

### 11.3a Batching the candidates: the fix, and what it did

`Synthesizer` now holds a seam's K candidates as one `(K, L, 7)` tensor. They
share the seam, the fractions, the timing, the limits and the target cache and
differ only in where they start, so they are one tensor problem; the chain
functions take a single leading batch dimension, so the candidate and knot
axes fold together for the call and unfold after.

    k = 5, bracket_a_placed        before   candidates   + seams
    synthesis                     110.8 s     ~22 s        ~3 s
    end to end                    133.7 s     45.4 s      26.2 s

Two rounds. First a seam's K candidates, then every seam of the plan request on
top of that — rows are `(seam, candidate)` pairs and share only a cell, with
each seam's curve, clock, fractions and tolerance riding along per row. The
whole plan request now costs what one seam used to.

**Rows are padded to the longest seam, and the pad has to be inert in three
places, not one.** Short rows repeat their last real knot so the pad holds
still, and then: the difference operator has to know each row's own last
index, or a seam's final velocity is computed against the pad and comes back
at half its true value; the acceleration and jerk sums have to be masked, or
the step from the last real velocity down to the pad's zero prices a stop
that is not in the trajectory; and the collision sum has to be masked too,
or a 41-knot row is charged for fifteen extra copies of its last stance.
That third one was harmless on this corpus only because the last stance
happens to be clear — a coincidence, not a guarantee.

Padding to 56 knots costs 2.11× the GPU arithmetic on this plan request. GPU
arithmetic is 18% of the time; the launches it removes are the rest.

**The proof that the diagnosis was right is that nothing got slower.** Thirty
Adam steps cost 546 ms of CPU before and 533 ms after — while doing five times
the work. The per-step cost was never in the arithmetic.

Behaviour is equivalent rather than identical: `seam_0003` lands at cost 1.56
against 1.54 and acceleration 0.16 against 0.17 of limit, because Adam's step
counter now survives a backtrack where recreating the optimizer used to reset
its bias correction. The monotone guarantee is unchanged.

### 11.3b CUDA graph capture: attempted, and BLOCKED

M9 lists graph capture "if launch-bound", and 11.3 established that it is:
after batching, still ~1 900 launches per step, 25% of CPU time in
`cudaLaunchKernel` alone, 190 ms of GPU work inside 533 ms of CPU. The ceiling
is therefore real — a captured replay costs about the GPU time, so roughly 3×
on the loop, taking the plan request from 45 s toward 30.

It does not capture today. Probed directly, two independent blockers:

    torch.linalg.solve   (205 x 6x6)   BLOCKED   CUDA error during capture
    cholesky + solve                   BLOCKED   same
    linalg.inv @ rhs                   BLOCKED   same
    plain autograd forward+backward    captures
    CollisionCost.value                BLOCKED   CPU-to-CUDA copy during capture

1. **Every cuSOLVER path is uncapturable.** The projection and the knot
   velocities each solve a small batched system every step, and no spelling of
   that solve — `solve`, `cholesky_solve`, `inv` — can be captured. The
   systems are `JJᵀ + λI`, symmetric positive definite and 6×6 or 3×3, so an
   unrolled Cholesky written in plain ATen would be capturable arithmetic.
   That means hand-writing a solver into a numerical path that currently
   delegates to LAPACK.
2. **The collision term copies between host and device** inside the warp
   autograd Function, which capture forbids unless the CPU side is pinned.

Note also what is NOT an option here: `torch.compile` with reduce-overhead
would remove dispatch and launch together, and it needs triton, which this
environment cannot compile — the same constraint that dictates the
`(R * v).sum(-1)` spelling in `weld_trajopt.py` and the einsum note in `scene.py`.

So the next step is not "turn on graphs"; it is two pieces of surgery, and
they are written down here rather than done because the batching win was
worth banking and neither piece is small.

### 11.4 Inside the two certificates

`seam_0001`: 3379 bisection nodes for tracking, 40 segments and 5665 queries
for T2.

    tracking: SeamTarget.local    3379 calls   1135.6 us/call   3.84 s
    FK (both certificates)        3876          118.4           0.46 s
    velocity extrema              6315           21.4           0.14 s
    tracking: lipschitz_f           43         1794.0           0.08 s
    verifier: coal margin predicate   5538           11.8           0.07 s
    verifier: coal exact distance      127           66.2           0.01 s

**The tracking certificate is one function.** `SeamTarget.local` is 3.84 s of
a 4.4 s stage, at 1.1 ms to evaluate the seam at ONE scalar fraction —
`Seam.frames` asks `Curve.locate` three times (the point, the rail datum and
the curvature each locate separately) and each solves `s -> u` by Newton with
8-point Gauss-Legendre inside, all as torch calls on batch 1.

The comparison worth keeping: **coal answers a mesh-vs-mesh proximity query
in 11.8 µs, and evaluating the seam at one point costs 96× that.** The
geometry query the verifier exists for is not the expensive part of the verifier.

The fix is the same shape as 11.3's: walk the bisection by LEVEL and evaluate
a whole frontier in one batched call, turning 3379 scalar evaluations into
about a dozen. Not done, deliberately — it restructures the control flow of a
verifier, and `tests/motion_planner/test_tracking.py`'s soundness sweep is what would
have to catch a mistake in it.

**Done, and it was the same fix a third time.** The bisection walks BY LEVEL
now, so a whole frontier's midpoints reach the seam evaluator in one call.
`SeamTarget.local` went from 3379 calls at 1.1 ms to **nine** — nine levels —
and from 3.84 s to 0.08 s, a factor of 48. Nothing about the decision
changed; only the order nodes are visited in, which the supremum search
notices (`seen` climbs on a different schedule, so the node count moves a
little either way and `seam_0005`'s reported bound shifts from 20.59 to
20.68 mm). Any covering of leaves is a valid certificate, so that is a cost
difference and not a soundness one, and the dense sweep in
`test_tracking.py` still holds.

### 11.5 Where it ended up

    k = 5, bracket_a_placed      original    now
    sweep (M4)                      2.2 s    2.3 s
    synthesis                     110.8 s    3.1 s
    tracking certificate           17.5 s    2.7 s
    T2 verifier                         3.2 s    3.3 s
    TOTAL                         133.7 s   11.5 s

**12× end to end, and no stage is the problem any more** — four phases within
a factor of two of each other. The remaining tracking cost is no longer the
seam evaluator either; it is `Kinematics.transforms`, 4442 calls at 95 µs,
the verifier's own independent float64 forward kinematics walking joint by
joint in numpy. Batching that is the next thing, and it is worth ~0.4 s.

The lesson across all three rounds is one lesson: **every time, the code was
asking a batched evaluator for one thing at a time.** The optimizer asked for
one candidate, then one seam; the verifier asked for one bisection node. None
of the three was a slow algorithm.

## 12. Free space (M6), where the measurement was correctness

Nothing in M6 has been optimized yet, so the numbers below are the ones that
decided the DESIGN rather than the runtime. They are here because each was
the thing that settled a question three guesses had failed to.

### 12.1 The constraint that could not be heard

At the approach's seed on `bracket_b_2x`, gradient norms per term:

    seed          smoothness   env collision   self collision
    straight           780.7            56.9             23.8
    wrist-first       4200.4            52.5             19.6
    standoff-1        4437.0            35.5             21.4

Smoothness is 30–200× the collision terms, and the optimizer descends the
sum. This is why 240 steps moved self-clearance by 0.3 mm. A sweep of 32
combinations — rho ∈ {50, 1000}, growth ∈ {1.6, 4.0}, outer ∈ {3, 8}, inner
∈ {80, 200} — improved it from −76 mm to at best −49 mm and never escaped.
**Adam is scale-invariant per parameter, so raising rho changes the DIRECTION
of the summed gradient and not the step size** — which is why it helped at
all, and why it plateaued: the wall was not a weighting problem.

### 12.2 The wall

Over 16 384 quasi-random configurations with `J5` between 50° and 145°:

    environment clear        81.0 %
    self clear                0.0 %       (best self-clearance -3.9 mm)

Outside that band, either side, self-clearance is 7–18 mm. The band is a
property of the arm, not of the part, which is why the environment column is
unbothered by it. This is what makes `bracket_b_2x seam_0001` at
`J5 = +199.5` unreachable from a reset pose at `J5 = 0`, and rewinding it to
`−160.5` — the same pose to 0.18 µm — a fix rather than a preference.

### 12.3 The standoff, per retreat distance

`bracket_b_2x seam_0001`, IK seeded at the weld configuration itself:

    retreat    converged   position error   largest joint move   clearance
      0 mm            —                 —                    —      3.9 mm
     50 mm          yes         0.038 mm               7.35°      16.3 mm
    100 mm          yes         0.001 mm              15.55°      30.3 mm
    150 mm          yes         0.006 mm              24.99°      67.5 mm
    250 mm          yes         0.016 mm              53.45°     141.7 mm

Joint travel rising smoothly with retreat is the number that says the solve
stayed in the same branch — a wrist flip would show as a jump.

### 12.4 What a roadmap costs

8192 samples on `bracket_b_2x`: ~2 s to build, 2978 nodes kept (36%), 16 965
edges, mean degree 11.4. A query is one screen of `2 × attach` edges plus a
`heapq` Dijkstra over the kept nodes — milliseconds. It depends only on the
cell and the placed part, so it is a cache candidate for M9 and is currently
rebuilt every run.

### 12.5 The motion nobody asked for, and the four faults behind it

The complaint was that a link contains needless movement. It was four separate
faults with one symptom, and none of them was the one the code had assumed.
`bracket_b_2x`'s approach, before: 645 degrees of joint travel to make a move
whose geodesic is 560, with `J1` turning 101 degrees to end up 10 from where it
started, flown in 2.9 seconds when the rate limits allow 2.3.

Two rulers are used throughout. **detour** is joint travel divided by the
straight-line distance between the clamped endpoints, in the roadmap's time
metric -- 1.0 is the geodesic. **churn** is a single axis's total travel over
its net travel, which is what catches an excursion that returns.

**The starting point, and why reweighting had never worked.** Per 40-step
block on that approach, tracing the accept rate and the per-row trust scale:

    step  accept%  scale min  scale med  scale max  moved deg   best
      40     93.8    0.06250    0.25000    0.25000      23.01  61.89
      80    100.0    0.06250    0.25000    0.25000      35.69  54.20
    -- multipliers grown, Adam restarted, trust scale KEPT
     120     91.6    0.00781    0.01562    0.06250       8.70  60.97
     240    100.0    0.00781    0.01562    0.03125      10.00  58.16

The scale halves on a rejected step and had no way back. At the multiplier
growth the objective legitimately jumps, a few steps are correctly rejected,
and four halvings later the loop runs at a sixteenth of its step for the rest
of the budget at a 100% accept rate. **An optimizer that is not taking steps
cannot hear its objective**, which is why 12.1's sweep of 32 penalty settings
plateaued and why an explicit joint-travel term at weight 50 changed total
travel by 0.02%. `docs/issues/0011`.

**What each fix bought, on that approach, horizon held so the shapes compare:**

    variant                                   detour  churn      travel
    as it stood                                1.488  10.08 J1    645 deg
    + trust scale recovers to 1.0 on accept    1.245   1.18 J3    586
    + L1 joint travel in the objective         1.245   1.18 J3    586
    + shortcut, one pass                       1.117   7.72 J1    621
    + shortcut, three passes                   1.002   4.61 J1    590
    recovery + travel 25 + shortcut x3         1.006   1.62 J3    560

Read the middle rows together: the travel term alone does nothing measurable
until the loop can move, and the shortcut alone reaches the geodesic in the
time metric while leaving a 46-degree round trip on a fast axis that the metric
prices as cheap. Both are needed and they fix different things. The travel
weight is on a plateau -- at 10 the bowed seed still wins, at 25 and 50 the
answer is the same 560-degree path.

That ablation is under the collision term as it then stood -- the AL at 50 mm --
because the point of it is to isolate one change at a time, and the cost was
still being argued about. It is not the final configuration's number: end to end
below, with the cost split, the same approach comes out at detour 1.03.

Four times the descent budget (320x3, no shortcut) reaches 1.017 and 575
degrees on its own, which is the same answer for twice the wall clock. The
shortcut's advantage is that it is a DISCRETE jump: a span whose straight line
screens clear need not be reachable by deforming through free space.

**The clock.** `duration_for` is exact for a rest-to-rest cubic along the
straight line and padded on top of that, so after the interior moves the knots
reach about half the limits -- worst velocity 0.46-0.54, worst acceleration
0.29-0.48 across the four links. `retime` recomputes the horizon from what the
trajectory achieved, shrinks, and re-solves; refusals bisect rather than give
up, and the accept test asks both of the verifier's questions, because a shrink
that landed over an acceleration limit reached the verifier once and a shrink that
shaved clearance from 3.9 mm to 0.1 mm reached it twice.

**The collision term, which turned out to be the largest fault.** Free space
had inherited M5's augmented Lagrangian and moved its margin to 50 mm, which
points a machine built for satisfiable constraints at a target the arm cannot
reach -- a link begins at a weld pose 3.9 mm off the work, clamped. After six
outer steps:

    outer       rho   lam max    lam>0  in margin  in collision  worst viol mm
        0     128.0      10.1      520        520            55          77.63
        5   14073.7     1257.4     128        118             0          46.91

Nothing in collision and 118 entries still under growing pressure. Split into
an AL on `clearance >= 0` and a fixed-weight buffer beyond it, three things had
to be measured before it worked:

- the AL is not replaceable by a weight: -2.50 mm at 30 000 and still -0.06 mm
  at 3 000 000 on the corpus's tightest link, never crossing zero;
- `rho` must start high enough that a penetration outranks a standoff -- at
  M5's default of 50 the same link sits at -5.24 mm, at 60 000 at -0.18;
- the buffer needs a LINEAR tail. A squared hinge pulls in proportion to the
  deficit, so where there is no room it pulls hardest and trades the constraint
  away: 20 000 with a 10 mm knee clears both the link that needs standoff
  preferred strongly and the link that has none to give, and the same weight
  with the tail removed fails the second again.

**End to end, three plan requests, verifier margin 0, three seams each:**

    plan request        links clear        transit time      worst detour
    bracket_b_2x    4/4  ->  4/4    12.8 s  ->  5.6 s    1.49 -> 1.08
    bracket_a_2x    3/4  ->  4/4    14.1 s  ->  8.5 s    1.14 -> 1.10
    hitch_2x        2/4  ->  4/4    19.9 s  -> 11.1 s    1.20 -> 1.02

Every link of every plan request is now collision-free at three seams, which the
corpus has never been, and the two that were not are the two the shortcut and
the split cost fixed rather than tidied. On the FULL 22-seam program the same
run gives 21 of 23 links free of contact, up from 20, at 44.0 s of transit and
detour 1.00 median / 1.28 worst; the two that remain are screened at -0.1 mm and
+0.5 mm, which is the sub-millimetre regime `docs/issues/0011` describes and not
a shape problem.

**What it costs to solve.** The times above are wall-clock of the flown
trajectory. The solve is the other side of the trade, and it was measured rather
than estimated -- the first estimate written here was "2-3x" and it was wrong.
`bracket_b_2x`, four links, same seed pool and step budget:

    configuration                       total    per link   vs before
    before (no shortcut, no retime)     6.71 s     1.68 s       1.00x
    + shortcut only                    12.75 s     3.19 s       1.90x
    + retime only                      24.69 s     6.17 s       3.68x
    both                               27.59 s     6.90 s       4.11x

The step arithmetic predicted it: 240 descent steps became 480 with the
shortcut's three cycles, plus up to 720 more across three retime attempts.
Shortcut lands on its predicted 2.0x almost exactly, which also confirms its
own screens (~18 scene queries per link) are noise next to the descent. Retime
comes in under its predicted 4.0x because it does not always spend all three
attempts -- it stops early when the rates ask for no shrink, and bisects on a
refusal.

The full 22-seam program takes 3m29s to join, which reads as 3.4x rather than
4.11x only because the sweep, the synthesis and the roadmap build are in that
number too.

**And most of the added cost is spent at batch 1.** Retime is 2.68x of the
4.11x, and its 240 steps re-solve ONE trajectory where the first solve carries
eight to ten seeds in one tensor. This loop is launch-bound, not compute-bound
(11.3: ~1900 kernel launches a step, 190 ms of GPU work inside 533 ms of CPU),
so a batch-1 step costs very nearly what a batch-8 step costs -- the retime is
throwing the batch dimension away. Its three attempts are sequentially
dependent and cannot be batched with each other; the LINKS can, and that is
already M9's item. This measurement is the argument for moving it up.

### 12.6 Choosing the chain: a DP that is not worth building, and the number that says so

M5 synthesizes `k = 5` candidates per seam and takes each seam's own best, with
no view of the program it sits in. The obvious improvement is a second DP —
layers are seams, nodes are that seam's candidates, edges are what it costs to
get from one candidate's last knot to the next's first — because which candidate
a seam ends on decides where the arm stands when the next link starts. The
machine is one this repository already has twice (`graph.py`'s Viterbi,
`orient.py`'s masked lattice), the edge weight needs no trajopt (`duration_for`
in closed form, taken modulo whole turns since a revolute axis at `t` and
`t - 2pi` is the same robot), and it would cost milliseconds.

It was measured before it was built, and it should not be built. Surrogate
transit of the whole program, approach and retract included:

    plan request        seams   today    no quality bar at all   share
    bracket_b_2x       22   72.16 s   -> 71.44 s  (0.72 s)    1.0 %
    bracket_a_2x       10   47.48 s   -> 47.00 s  (0.48 s)    1.0 %
    hitch_2x           19   85.69 s   -> 84.83 s  (0.86 s)    1.0 %

One per cent is the CEILING, taken with every candidate admitted however bad.
At a bar anyone would defend it is nothing:

    band on the objective   nodes/seam   saved      seams changed
    <= 1.25x seam's best          1.00   0.00 s        0 of 22
    <= 2x                         1.18   0.12 s        1
    <= 4x                         1.41   0.14 s        1
    <= 10x                        1.77   0.26 s        3
    any                           4.82   0.72 s       13

Two reasons, and both are numbers rather than opinions.

**The pools are not diverse in the way that matters.** The gap between a seam's
best candidate and its second is typically an order of magnitude of objective —
0.15 against 1.56, 0.12 against 1.30, 0.00 against 6.10 — because M4's K-best
are near-duplicates on the lattice that separate sharply once synthesized. So a
band wide enough to give the DP a choice is a band that accepts a weld path four
to ten times rougher than the one available.

**And a transit's cost is a property of WHERE THE SEAMS ARE.** Per consecutive
pair on `bracket_b_2x`, the surrogate edge over all 25 candidate combinations:

    median spread, as a share of the edge          2.3 %
    best combination beats top-1 -> top-1 by       24 ms (median, per pair)

against edges of 1.6 to 4.4 seconds. Choosing a candidate moves the end pose by
a few degrees of spin and posture; the arm still has to fly across the part.

**What would change the answer**, and it is the one quantity to re-measure on a
new cell: that spread. At 2.3% there is nothing to win. A fixture with widely
separated seams, or a cell whose IK classes put the arm in genuinely different
places at a seam's end, could show 30% — and then the DP is worth its
milliseconds. `generate.plan_weld_candidates` is the seam it plugs into and
exists for that reason; nothing plugs into it today.

## 13. The join, batched and replayed

The target: a 10-seam set's connecting motions -- solve and join -- in 0.5 s
warm, batch as large as memory allows. `bracket_a_2x` (10 crossed seams, 11
connecting motions), RTX 5090, verifier off, `tools/bench_freespace.py`:

    change                                                      connect
    baseline (12.5's per-motion loop)                           71.25 s
    esdf origin `.cpu()` removed, warp wrappers cached           68.19 s
    ONE batch: rows = (motion, seed), M5's row model              9.23 s
    CUDA graph capture of the inner step                          4.56 s
    standoff IK: twenty solves -> one batched call                3.43 s
    one recording per descent (rho a tensor, moments zeroed)      3.34 s
    retime re-solves at RETIME_OUTER_STEPS = 1                    2.73 s

26x, same accept rules, every motion clear, durations equal or shorter --
the retime row ACCEPTED two shrinks the pressured re-solve used to refuse.
The whole 22-seam `bracket_b_2x` joins as one batch in 6.65 s (was 3m29s in
12.6); the self-collision tensor at that size is hundreds of megabytes, so
the batch is nowhere near a memory wall.

### 13.1 What capture surfaced, which eager had been hiding

Three real defects, found because a replay is honest about ordering:

1. **`wp.from_torch` on a differentiable tensor allocates a warp grad buffer
   per call** -- the 11.3b "CPU-to-CUDA copy during capture", and dead weight
   eagerly. Every wrapper now passes `requires_grad=False`.
2. **Warp launches did not follow torch's current stream.** On the default
   stream the two coincide, which is why nothing ever noticed; on a side
   stream the kernels run unordered against the tensors they read, and
   inside a capture they are silently left OUT of the graph. Measured: a
   loss with two fk nodes lost the self-collision branch's gradient
   entirely. `utils.device.warp_stream` now routes every launch
   (`esdf`, `scene`, `fk`), and `scene.clearances` computes both fields from
   ONE fk pass -- which also halves the fk calls per cost eval eagerly.
3. **`fk`'s backward zeroed its accumulator through the warp wrapper** -- a
   warp memset no torch capture can record -- so a replayed backward
   returned exact integer MULTIPLES of the true gradient, one per replay:
   7.7e3, 1.54e4, 2.3e4 for three replays of the same state. The zero is a
   torch op now, and the loop that stalled at cost 326 where eager reached
   72 lands at 71.78 both ways, scale for scale.

### 13.2 Where it now stands, and where the rest of 0.5 s is

The replayed step costs **4.09 ms at production shape** (88 rows, 6,952
poses per eval) and the loop is GPU-arithmetic-bound -- the launch overhead
11.3 measured is gone. ~700 steps x 4 ms is the 2.7 s. The remaining path
to 0.5 s is the product of two factors, both unmeasured as of this table:
the step BUDGET (initial 3x80 and the shortcut re-descents are inherited
numbers, not measured ones) and the step's ARITHMETIC (two full cost
evaluations per step; the AL hinge walks ~12M floats several times per
eval, a candidate for the same kernel fusion §4 measured at 7.7x).

## 14. The 2026-08-27 pass: float32, capture everywhere, and the batched shortcut

One deployment constraint drove all of it: the target is Jetson-class
hardware, where FP64 runs at 1/64 of FP32 with no FP64 tensor cores, and the
CPU that issues kernel launches is a fraction of a desktop's. Everything
below was measured on the 5090; the RATIOS are what transfer.

End to end, the two-weld program (weld + connecting + verification), warm:

    start of the pass (fp64, eager weld loop, greedy pruner)   8.0 s
    weld descent captured as a CUDA graph                      5.2 s
    both loops float32                                         4.35 s
    shortcut screen batched per level                          3.75 s
    re-solve budgets sized to refinement (RESOLVE_STEPS,
    weld inner 60 -> 40 in OPTIONS)                            3.4 s

The 22-seam corpus request over the same span: 20.0 s -> 10.9 s. The
connecting stage STANDALONE on that corpus: 10.5 s -> 0.98 s, of which 8.3 s
was the un-batched pruner screening one segment per call -- a kernel launch
and a device-to-host read each.

Numbers that decided things, kept because they will decide them again:

- The control step, eager, profiled by op: 1.1 ms of GPU per step, and the
  single largest kernel `cutlass d884gemm` -- the FLOAT64 basis GEMM -- at
  13%. The FK and both collision fields were float32 the whole time
  (`q_full` casts on entry), so the float64 above them bought nothing.
- The weld step: 2.9 ms of GPU across ~2,400 kernels, none above 4 us. That
  is a kernel-count problem, not a FLOPs problem; the chain walks
  (`mobility.jacobian`, `pose_error`, `tool_pose`) issue hundreds of
  micro-kernels per step. The named next step is routing the loss-side tool
  positions through the fused warp FK (`fk.py` already carries the
  `dp/dq = a x (p - o)` backward) and giving `project` a fused link-
  transform read -- one launch where there are hundreds. ATTEMPTED and
  pulled back the same day: a zero-offset sphere at the tool link
  reproduces the torch walk's POSITION to 4e-7 m, but its gradient comes
  back wrong by 200x (2.64 against a true magnitude of 0.014, eager and
  internally-captured alike), so `fk_backward`'s identity is not, as used
  there, the tool-point gradient. Diagnose it against
  `torch_link_transform`'s autograd on a one-joint chain before a second
  attempt; a loss is the wrong place to learn a backward is broken.
- Capture refusals are diagnoses. Every one of the four found a per-step
  host synchronise that was ALSO an eager-mode cost: `spec.axis` uploaded
  per jacobian call, the electrode axis per projection, the scatter index
  per call, and `torch.linalg.solve` reading its info flag back. The
  fallback's traceback, gated behind `WELD_PLANNER_CAPTURE_DEBUG`, is how
  they were found.
- What float32 broke, it broke honestly: endpoint exactness (restored from
  the float64 spec, since the ends are data), the pinned-end zero
  velocities (the fp32 GEMM's FMA leaves one product's rounding residual,
  7.5e-7 rad/s, on bitwise-equal control points -- written back as the
  construction guarantees), and the weld loop's batch-invariance to 1e-6
  (a property of the precision, not a contract; the pad-leak test now
  allows the measured 3.5e-2 run-to-run divergence).

## 15. cuRobo's grid, cuRobo's sweep, and the gate under issue 0016 (2026-08-29)

Cost grid 161 -> 61 (`INTERPOLATION_STEPS = 4`, cuRobo's ratio), wire by
time (`WIRE_DT_S`, 0.02 then 0.01 s -- the 0.01 s numbers are in 16), the collision term swept by a warp kernel
(`scene._make_sweep_kernel`, cuRobo's walk with the gradient scattered to
both chord endpoints), the shortcut cycle gated on clearance, verifier
bisection capped at 10. `tools/bench_plan.py`, warm means:

    2-seam                      5090 before   5090 after   Thor before   Thor after
    total                            3,483        3,572         9,592        9,487
    connecting                       1,196        1,501         2,694        3,336
      conn:descent                     422          722         1,128        1,731
    22-seam
    total                           12,753       12,399        45,514       47,242
    connecting                       4,211        5,124         7,379        9,060
    colliding motions (T2)           1 (0016)     0             1 (0016)     0

Speed is flat to slightly worse; every corpus motion certifies for the
first time. What the descent is paying for is COUNT, not cost per step: the
sweep kernel is 0.05 ms of GPU per step and the merge ops around it a few
more, but with the swept clearance in the accept tests the shortcut cycle
finds and keeps more re-solves (kept 4 -> 13 on the 22-seam corpus, solves
11 -> 14). Those are shorter, certified paths -- the cost is real and so is
what it buys. The lever left is the cycle's budget (`SHORTCUT_CYCLES`,
`RESOLVE_STEPS`), to be swept against the detour it recovers.

Numbers that decided things:

- Interpolation steps 4 / 6 / 8 / 10 all held every corpus contract; the
  detour sat at 1.20-1.21 at every grid. Nothing above 4 was buying
  anything measurable.
- A point-sampled hinge at 61 samples let seam_0011 -> seam_0016 penetrate
  between samples (verified); a torch chord read (K = 4 fixed points) closed
  it and cost 5.4 s against 3.8 s on the 22-seam connecting stage; the
  adaptive kernel closes it at 3.8 s. The cap was swept 3 / 6 / 10 with
  identical answers -- it binds only inside a wall's reach.
- The persisting penetration on seam_0013 -> seam_0014 was the shortcut
  cycle accepting a re-solve on rates and length alone: its own clearance
  read -7 mm beside "shortcut kept". `retime`'s no-worse rule, applied to
  the cycle, ends it. Issue 0016 closed on that, not on sampling.
- The sweep kernel's backward against autograd with the argmin held:
  5e-7 on gradients of magnitude 1.45.

## 16. The weld pass at free space's resolution: grid, sweep, wire (2026-08-30)

M5 now judges and emits at the numbers `control_solver` carries -- imported,
not restated. The cost grid is every lattice knot plus `INTERPOLATION_STEPS -
1` samples of the Hermite inside each span (4: a read every 62.5 ms against
the lattice's 0.25 s); the collision AL is read on that grid and SWEPT along
it (`CollisionCost.sweep_samples`, the warp chord walk, `SWEEP_STEPS` 10);
the tracking quadrature is the mean over a span's interior samples; and what
leaves is the same Hermite re-evaluated at `WIRE_DT_S` 0.01 s in float64
(`resample_wire`). `tools/bench_plan.py`, 5090, cold 1 + warm 5 means, ms:

    2-seam                     5090 before   5090 after   Thor before   Thor after
    total                           4,240        4,027        10,119       10,859
    weld_trajopt (M4+M5)            2,130        2,159         5,813        6,811
      weld:descent                    521          690         1,637        2,828
      weld:verify (delta)             400          418         1,321        1,016
    22-seam (bracket_b_2x)
    total                          16,871       16,265        49,556       60,673
    weld_trajopt (M4+M5)            8,165        8,700        37,948       49,683
      weld:descent                    938        2,419         5,808       19,041
      weld:verify (delta)           2,426        1,696         4,006        2,528

(Thor: warm 3, the 0.01 s free-space wire already in "before".)

The descent is the cost, and it is the grid's: at the 22-seam batch (110
rows x 261 grid samples, 28,710 poses a step) one step's collision read is
7.2 ms on the 5090 against ~1.6 at the knots, of which the sweep itself is
0.8; the interior tracking is 2.7 ms. On Thor the same step is ~120 ms
against 36 -- the fused FK and field read at 4x the poses, on a part with
1/50th the FP32 -- and the 22-seam weld stage goes from 38 to 50 s there. What it buys is visible in the report: the
worst clearance over the corpus read at the KNOTS was 1.26 mm, and on the
curve between them it is 0.10 mm -- the AL was never told about the dip.
Grid 2 with the sweep reads the same 0.10 mm, so the chord walk covers what
the extra samples cover -- the 22-seam weld descent by grid and sweep, k=5,
verify off, min of runs, ms:

                              5090      Thor    worst clearance on the grid
    grid 1, no sweep (was)     935     5,142    1.255 mm (knots only: blind)
    grid 2, no sweep             -     9,286    0.096
    grid 2, sweep 10         1,732    10,410    0.096
    grid 4, sweep 10         2,710    19,636    0.105

The grid is kept at free space's 4 because the two stages share the
number; grid 2 is the lever if the weld's descent needs to be cheaper on
Thor, and it costs nothing this corpus can see.

Two things that were measured and reversed on the way:

- **Certifying the wire.** The first cut handed T2 and the tracking
  certificate the 0.01 s wire: 2.4 s -> 42.7 s of `weld:verify` on the
  22-seam corpus, 0.4 -> 3.7 on the 2-seam. Both walk every segment they are
  handed, and the wire is 25x the segments of the SAME cubic. The
  certificates are now computed on the lattice Hermite
  (`SeamTrajectory.lattice_*`), which is the identical curve
  (`test_the_wire_is_the_curve_at_the_controllers_clock`: 1e-9 rad, checked
  against the verifier's own `_hermite`), and the wire is what leaves. Free
  space cannot make the same move -- its wire is a Hermite through samples
  of a B-spline, a different curve -- which is why it pays for its 0.01 s
  and the weld pass does not.
- **A pad tax that was not one.** `hitch_2x`'s 11-knot seam, batched
  against a 151-knot one, landed 2x off its solo acceleration ratio and
  failed the batching test. Gradient parity, taken with the collision term
  live (J2 pushed 0.08 rad into the part, -18 mm): alone against batched
  agree to 2e-6 of the largest entry, with the sweep on. The ratio is
  1-2e-4 of the limit on a converged seam, and at that level the two
  descents differ by float32 noise. The test now asserts the gradient (the
  contract) and gives the ratio an absolute floor of 1e-3 of the limit.
  The pad's first span IS held at the knot on the way (`Synthesizer.grid`):
  a Hermite from a moving knot to its motionless copy bows out, and the
  chord from the last real sample would otherwise walk toward it.

## 17. Free space certified on its pieces, reported on a sample (2026-09-17)

`bracket_b_2x` cut to the eight welds that plan on today's cell (the other 14
are refused by M4), `tools/bench_plan.py`, 5090, cold 1 + warm 2, ms:

    8-weld bracket_b_2x            before       after
    total                          18,829      12,801
    connecting_trajopt              9,189       3,144
      conn:descent                  1,260       1,275
      conn:verify (delta)           7,747       1,719
    weld_trajopt (M4+M5)            9,542       9,562

Free space was paying for its 10 ms wire twice (16 above). Serial, over the
nine motions of that request (wires of 686-1853 samples), with the wire
already certified on its pieces:

    certificate on the kept samples         2.3 s   213 segments
    clearance report on the whole wire     12.6 s   691k queries in all
    clearance report on <= 161 samples      1.7 s   104k queries in all

- **The certificate** now walks the Hermites between the first and last wire
  sample of each cubic piece (`freespace.certified_samples`). The wire's
  `(q, qdot)` are the B-spline's own, so inside a piece that Hermite IS the
  piece; the ~13 wire intervals that straddle a boundary are kept as they
  are. The controller's curve is certified exactly -- 1e-9 rad against the
  wire's Hermite at 4,001 random instants (`test_certified_samples.py`) --
  on 25 segments a motion instead of up to 1,853.
- **The report** reads at most `REPORT_SAMPLES` = 161 wire samples plus the
  certificate's, and names the worst on the wire's own index. It passes and
  fails nothing; 161 is what the wire had before it was sampled by time.
- **Found on the way:** `bspline.basis` returned zero for EVERY derivative at
  `u = 1`. The closing-endpoint rule picked its function by count, and a
  derivative evaluates a basis with one more function. `connect` pins both
  ends of every motion, so the end velocity and acceleration are zero anyway
  and no plan changed; the end jerk was invisible to the cost, and an
  unpinned end (`plan_move`'s default) emitted zero velocity where the curve
  had 51 rad/s. The certificate above is only sound with it fixed, since the
  last piece is now certified from its end velocity.

## What has not been measured

The end-to-end budget in `ARCHITECTURE.md`, which stays an estimate until M7
closes the loop. M6 now joins the program — `bracket_b_2x` runs reset → three
welds → reset with every link certified — but nothing in it has been timed
against a budget, and the roadmap is rebuilt per run rather than cached.

**The sweep is now the cost and nobody has optimised it yet.** Closed form took
the build to 1% and the sweep owns 98% of a 128-bin run. It is already compacted
to feasible nodes and sliced by destination; what has never been tried is the
exact pruning `docs/issues/0004` describes -- the positioner is a state component
compared against its own threshold, so `|turn_i - turn_j| > threshold` is a
guaranteed non-edge computable from bin indices alone, and it removes a fixed 86%
of pairs. That was worth 2.6% when the sweep was 3% of the time. It is worth
most of the run now.

The multi-turn wrap is owed and measured. The closed form returns each joint in
`(-pi, pi]`, and `J1`, `J4`, `J5` and `J6` all have ranges wider than a turn, so
a path needing one of them to wind past 180 degrees sees a 358 degree step and no
edge. It has not bitten -- the corpus is 15 of 15 and the best path on `dog_box`
seam 1 came no closer than 17.4 degrees to the boundary on `J4` -- but that is
margin, not a guarantee.

Whether collision screening should be inside the inverse-kinematics objective
rather than after it. It is a post-hoc screen today, with gradients explicitly
disabled, and `scene.py` carries a full differentiable path that only M5 uses.
The argument for keeping them apart is in `nodes.py`; the cost of being wrong
about it has not been measured.

T1 is built (`mobility.py`) and its numbers are in `IMPLEMENTATION.md` rather
than here, because what it measures is a property of the ARM and not of this
code's speed. The one timing worth stating: the Jacobian and its SVD over
4096 poses of a 6-axis chain is a single batched call, and the whole T1 pass
over a trajectory is dominated by the `svd`, not by the chain walk.
