# AMR Weld Planner v1 Goal

## Goal

Turn CAD assemblies into robot motion for arc welding, as a self-contained
pipeline, with a clean seam between geometry and planning.

The pipeline is deliberately split at one contract — the **weld program**
(`amr-weld-planner-v1.weld-program.v1`). Everything upstream of it answers *what
weld is required*. Everything downstream answers *how the robot achieves it*.
The contract carries no robot state, no joint values, no poses.

```
STEP
  │  1. topology       solids, faces, edges with surface geometry
  │  2. joint detection face pairs across solids -> weld joints
  │  3. weld program    <-- THE CONTRACT: amr-weld-planner-v1.weld-program.v1
  │  4. planning        positioner, redundancy, collision, sequencing
  │  5. trajectory      urdf-viewer-trajectory-v1 -> playback / execution
  v
motion
```

## Why this exists

`cadquery/v1/python/extract_weld_edges.py` already walks CAD to weld paths, and
it works for what it does. The problem is not that it fails — it is that it
bakes decisions belonging to the planner, and its approach does not generalize:

- it emits **dense 6-DOF poses**, fixing torch spin that a weld leaves free.
  This is the main reason for a separate pipeline;
- work angle is hardcoded `45.0` and travel angle does not exist;
- seam detection is **shape recognition** (closed circle >= 75 mm, or a closed
  wire of exactly 4 arcs + 4 lines), not part-to-part adjacency. On this
  workpiece the shapes happen to match — the top plate's contact face genuinely
  is 4 arcs + 4 lines, so it is found. But it is found as a *shape*, with no
  knowledge that it is a joint, no dihedral, and no way to reach a joint whose
  outline is not a dogbone or a large circle;
- it never reads surface geometry at all. `BRepAdaptor_Curve` appears once, as a
  point evaluator; `BRepAdaptor_Surface` is never imported. Joint detection
  needs plane normals and cylinder axes, so it cannot be built on top;
- **two** lossy resamplings, not one. It first samples uniformly in curve
  *parameter* (`param = first + (last - first) * t`, line 348), which is not arc
  length, then arc-length-resamples the resulting polyline. The handoff blamed
  the second; the first is invisible because the damage is already baked in.

This module keeps none of that. Edge and topology handling may be *informed* by
the prior implementation, but the code is self-contained — no imports from
`cadquery/v1/`, no imports from `tesseract/v1/`. External libraries only.

## Context in this repo, as verified 2026-08-02

Facts that shaped the design and are not obvious from a directory listing.

**`path-planner/v1` is not a planner.** It is the Steam Deck spreadsheet
simulator UI. Do not put planning work there, and do not assume this module is
its sibling. The prefix is what keeps the two apart in one repository.

**`cadquery/v1/step/dog_box.step` has 6 solids**, and is a genuine weldment
rather than the synthetic fixture the shape-recognizer treatment implies:

Ids below are the module's own deterministic ordering — volume descending, then
centroid, so they are stable across OCCT versions. Solids 1, 2 and 3 share a
volume, so centroid breaks the tie and the top plate is `solid_2`, not
`solid_1`. (An earlier revision of this table used raw OCCT traversal order and
was wrong about exactly that.)

| solid | bbox (mm) | note |
|---|---|---|
| 0 | 300 x 150 x 150 | base block, z 0-150 |
| 1 | 250 x 3 x 125 | plate at y -78..-75 |
| 2 | 250 x 125 x 3 | top plate, z 150-153 |
| 3 | 250 x 3 x 125 | plate at y +75..+78 |
| 4 | 100 x 100 x 3 | cylinder, x -153..-150 |
| 5 | 100 x 100 x 3 | cylinder, x 150..153 |

All five contacts are exactly coincident: `solid_2`'s bottom face at z=150 meets
`solid_0`'s top face; solids 1 and 3 land on the block's y=+/-75 faces; the
cylinders sit on x=+/-150. Note the bboxes describe extents, not shapes — the
top plate's contact face is a dogbone with four 270 degree reliefs at R=25 mm,
821.239 mm around, which is not something a bounding box can show. Assuming
otherwise is how the reference fixture was wrong for three commits.

**`extract_weld_edges.py` never surfaces face geometry.** Faces are used only to
harvest wires (`build_face_wire_edges`, around line 541); no surface type, no
normals, and no per-solid face ownership reach the topology output. Joint
detection cannot be built on it and has to start from the B-rep.

**`tesseract/v1/python/cartesian_plan.py` already does more than expected**, and
is worth reading before building the planner even though nothing here depends on
it. It supports a 5-DOF tool-axis constraint with spin left free
(`solve_tip_offset_ik_axis`), posture bias through the remaining redundancy, and
a `workpiece_frame` with per-target positioner overrides so targets can be given
in workpiece-local coordinates. Three limits to know about:

- the tool-axis constraint is **global**, one `world_axis_xyz` for all targets,
  so it cannot express a torch direction that rotates along the seam;
- TrajOpt is fed `JointWaypoint` + `FREESPACE` moves, so it smooths and
  collision-avoids *through* numerically IK'd seeds and is free to pull the TCP
  off the seam — `max_tracking_error_m` measures that after the fact;
- positioner axes are **prescribed, not solved** — they are pinned exactly and
  excluded from IK.

**Collision checking** forks `python3 collision_check.py` per call, so process
startup dominates. `persistent_collision_check.py` is a single-process server
with a result cache and no threading. The verdict path reports
`tesseract_collision_geometry: visual_meshes_as_collision_meshes`, which matters
far more for millimetre weave clearance than for air moves. Dev machine has 24
cores. See `motion/WEAVE-AND-VERIFICATION.md` §5.

## Scope

**In scope**

- STEP -> topology with per-solid face ownership and surface geometry.
- Weld joint detection by face-pair adjacency across distinct solids.
- The weld program schema and its validator.
- A motion planner consuming the weld program. Not started, and it is a
  SEPARATE program: the seam it attaches to is the `.weldplan` plan request,
  specified in `motion/MOTION.md`.

**Out of scope**

- Anything that writes to hardware.
- Anything in `robot/v4/`, `steamdeck/`, `realtime-core/`, or the NATS path.
- Modifying `cadquery/v1/` or `tesseract/v1/`. They are references, not
  dependencies.

## Non-negotiables

1. **The weld program describes the weld, never the motion.** No joint values,
   no TCP poses, no reachability assumptions, no robot at all.
2. **Orientation stays underdetermined.** A weld constrains 5 DOF. Torch spin
   about the electrode axis is free unless a seam explicitly says otherwise, and
   the schema has no field capable of accidentally fixing it.
3. **No baked sampling.** Every quantity that varies along a seam is a function
   of normalized arc length, never a per-sample array. The planner chooses
   sampling density.
4. **Curves stay analytic.** Tangent needs C1 and curvature needs C2; both come
   from the curve, never from finite differences over a polyline.
5. **Geometry is expressed in the workpiece frame.** The planner is free to move
   the positioner, so nothing upstream may commit to a world pose.

## Current state

Contract designed. Stages 1-3, the curve evaluator, the authoring server and a
browser viewer are implemented and tested. The planner is not started.

- `python/seam_worker/topology.py` — stage 1 extractor, `amr-weld-planner-v1.topology.v1`
- `python/seam_worker/detect.py` — stage 2 joint detection, `amr-weld-planner-v1.joints.v1`
- `python/seam_worker/tessellate.py` — display mesh, `amr-weld-planner-v1.mesh.v1`
- `python/seam_worker/access.py` — stage 3 reachability, tooling-independent
- `python/seam_worker/curve.py` — rational B-spline evaluation, arc length, curvature
- `python/seam_worker/frames.py` — seam frames `{P, t, r, b}`, SEAM-MODEL.md §5
- `python/seam_worker/server.py` — authoring server: jobs, staged SSE, artifacts (written
  fresh every run, no cache), manual face-pair propose, program save/load,
  validation
- `ui/` — browser viewer: mesh, weld lines, blocked spans, and the seam list
  (one row per weldable run, not per joint)
- `fixtures/pipe_bracket.step` — second workpiece: all tee joints, real occlusion
- `tests/` — 80 pytest cases; `pixi run test`, or `pixi run self-test` for the
  fast worker smoke checks; `pixi run serve` to run the server
- `pixi.toml` — self-contained env, pinned to match `cadquery/v1` so conda
  packages hardlink from the shared rattler cache
- `schema/SEAM-MODEL.md` — conventions, frame math, field semantics
- `schema/amr-weld-planner-v1.weld-program.v1.schema.json` — JSON Schema (2020-12)
- `schema/examples/dog_box_lap_perimeter.json` — reference fixture
- `motion/WEAVE-AND-VERIFICATION.md` — planner-side: weave geometry, the transverse
  mobility constraint, verification strategy, and what kinematic simulation
  cannot prove
- `DECISIONS.md` — prior-art survey (why not adopt an existing standard) and the
  design calls, including D3, D4 and D9, which are still marked open
- `WEBUI.md` — requirements for the STEP-to-weld-program authoring UI: async
  model, display artifacts, picking, and where the frame math lives

## Next steps

1. ~~Schema validator + round-trip tests against the reference fixture.~~
   **Done** (`tests/programmer/test_weld_program_schema.py`)
2. ~~Stage 1 topology extractor.~~ **Done** (`python/seam_worker/topology.py`) — 6 solids,
   42 faces, 90 edges from `dog_box.step`; exact rational B-splines; outward
   normals verified against solid centroids; seam edges flagged.
3. **Regenerate the reference fixture from extracted geometry.** The current one
   is a hand-authored simplification: a 750 mm rectangle where the real face is
   an 821.239 mm dogbone with 270 degree reliefs. The real face is the better
   test of the rail concept, because the groove-face normal rotates through each
   arc and the bisector rail has to follow it.
4. ~~Stage 2 joint detection.~~ **Done** (`python/seam_worker/detect.py`) — 5 joints on
   `dog_box.step` from face adjacency, with weld line, groove faces, dihedral
   and an exact bisector rail.
5. ~~Display mesh and server.~~ **Done** (`python/seam_worker/tessellate.py`,
   `python/seam_worker/server.py`).
6. ~~Curve evaluator.~~ **Done** (`python/seam_worker/curve.py`) — de Boor on rational
   B-splines, panelled Gauss-Legendre arc length, Newton s->u inversion,
   curvature. Validated against OCCT as an independent oracle and against
   closed forms.
7. ~~`POST /api/seams/frames`.~~ **Done** (`python/seam_worker/frames.py`) — the frame
   construction is server-side so the client cannot diverge on a sign
   convention; `apply_angles` is the ten-line reference the client duplicates.
8. ~~Viewer: rail, torch glyph, cross-section inset.~~ **Done** — dashed rail,
   torch glyph with its tip at the weld point, frame triad, and a live
   cross-section showing the groove wedge, the bisector and the torch in it.
   Work and travel sliders act client-side; `tests/programmer/test_frame_parity.py` runs
   the real TypeScript and checks it against `frames.py`.
9. Manual 1D line tool, then selection, parameters and save.
10. Stage 4 planner.

## Where the accessibility and seam-shape work was left

Done, tested and committed. What follows is what a resumer needs, not a summary
of the diff.

**What changed.** `detect.py` emits one joint per boundary loop (D18), so a pipe
yields its outer fillet and its bore instead of nothing. `occlusion.py` is a new
BVH ray intersector and is now what `access.py` uses (D19): the manifold's
accessibility pass went from ~360 s to 3.7 s, and a full analyze in the server
from ~6 minutes to 2.0 s. A general bisector rail (D20) handles groove faces
that are neither planar nor cylindrical, so elliptical and B-spline seams detect
at all. `topology.py` skips degenerate edges, which were segfaulting OCCT inside
`Geom_TrimmedCurve` and taking the pool worker with them — the operator saw
`BrokenProcessPool` on every subsequent job until the server was restarted.

**Known gaps, in the order I would take them.**

1. **The rail refines every span equally.** 388 poles for a 158 mm elliptical
   rail. Splitting only the spans where the bisector actually turns reaches the
   same tolerance far cheaper. Optimisation, not correctness; the measured
   convergence table is in `detect.MAX_RAIL_REFINEMENTS`.
2. **`candidate_pairs` is planar-only.** `detect.candidate_pairs` filters to
   `surface_type == "plane"`, so a curved *contact* — a pipe saddled onto
   another pipe — is never a candidate. Unrelated to the rail work, and the next
   real limit on what can be detected at all.
3. **The UI does not surface the new fields.** `bisector_rail.approximate` and
   `within_tolerance`, and `accessibility.intersector`, are in the artifacts and
   absent from the TypeScript types, so an operator cannot see that a rail is
   approximate or which intersector answered.
4. **Partial overlap is still unsupported.** Pre-existing: `_overlap_relation`
   needs a 2D boolean in the shared plane and reports rather than guesses.

**Two things that are true and easy to trip over.**

- The mechanism behind the coarse-mesh anomaly is **not** established. At 0.25 mm
  deflection one manifold bore read 50 of 53 samples blocked against a true 22.
  The obvious explanation does not reproduce synthetically — see the note in
  `occlusion.py`. The default sits well inside the converged region and
  `test_the_answer_is_stable_under_refinement` pins the property that matters.
- `BRepMesh_IncrementalMesh` keeps any triangulation already on the shape, so a
  deflection asked for after another caller has meshed is accepted and ignored.
  `occlusion.triangles` calls `BRepTools.Clean_s` first;
  `test_asking_for_a_deflection_actually_changes_the_mesh` guards it.

## Definition of done

- A STEP file with no prior configuration yields validated weld joints with
  correct types and dihedral angles, including the five in `dog_box.step`, found
  from face adjacency rather than from recognising an outline shape.
- A weld program round-trips through the schema validator with no loss.
- The same weld program can be consumed by two different planners and produce
  two different, both-valid trajectories — proving the contract does not smuggle
  planning decisions.
- Torch spin is demonstrably free: a planner can exploit it to improve a plan.
- Corner fidelity holds. On the R ~ 7 mm dogbone corners, deviation from the
  analytic seam stays inside the declared position tolerance.

### Weave verification tiers

Weave never enters the optimization, only the constraint set and the final
check. Rationale and numbers in `motion/WEAVE-AND-VERIFICATION.md`.

- **T1, per plan.** Transverse mobility `mu_b'` holds a hard floor at every
  centerline waypoint, with the velocity and acceleration budget share `beta`
  reserved *before* optimization rather than checked after.
- **T2, per plan, the final check.** The expanded weaved trajectory passes continuous
  collision detection plus joint position, velocity, and acceleration limits.
  Fail closed. Nothing ships without it.
- **T3, per robot and cell.** An adversarial sweep *finds* minimum-`mu_b'`
  postures rather than sampling typical ones, and confirms T1's constraint binds
  strictly before any real limit does. Necessary because "welding is slow" is
  true generically and false exactly at near-singular postures.
- **T4, hardware.** Measured TCP path under a commanded weave, with a frequency
  sweep to locate structural resonance. Returns a frequency ceiling per posture
  region that T1 then enforces.

**T1-T3 cannot substitute for T4.** A 3-4 Hz weave excites an arm whose first
structural mode may sit near 5 Hz when extended and loaded, and the URDF is
rigid — no compliance, no backlash, no drive-train dynamics. A passing
simulation check means *kinematically feasible and collision-free as modelled*.
It is never evidence of weld quality, and must not be reported as such.
