# Web UI: authoring weld programs from a STEP file

Load a STEP, review detected weld joints, author seams by hand where detection
misses, set weld parameters, save a `amr-weld-planner-v1.weld-program.v1`.

This tool produces a weld program and stops. No planning, no robot, no
trajectory, no hardware. That is the contract boundary from `GOAL.md`, and the
UI enforces it rather than blurring it — a "simulate" button would be the first
step toward smuggling planning decisions upstream.

---

## 1. The failure mode this is designed against

The existing CAD path in `offline-programming/v1` shells to `pixi run python`
with a 90 s timeout. The first call always times out, because the OCCT
environment is bootstrapping 2.5 GB on first use, and it **silently falls back
to a synthetic fixture**. The user sees geometry. The geometry is fake.

So "async" here is a correctness requirement, not just a responsiveness one.

**R-UI-1 — No silent fallback, ever.** A job succeeds with real geometry, fails
visibly, or is still running. There is no fourth state, and there is no
synthetic fixture anywhere in the codebase to fall back *to*.

**R-UI-2 — Provenance always on screen.** Everything displayed is tagged with
the source STEP's `sha256`. If the viewport shows a shape, the UI can say which
file and which hash produced it. Cheap, and it makes the whole bug class
impossible rather than merely unlikely.

---

## 2. Architecture

```
browser  (Vite + TypeScript + three.js 0.170, no framework)
   |  fetch + EventSource
   v
FastAPI / Starlette  -- in-process --  topology.py, detect.py, frames.py
   |                                   ProcessPoolExecutor, warmed at startup
   v
workspace/
  steps/<sha256>.step
  artifacts/<sha256>/{topology.json, mesh.bin, joints.json}   rewritten every run
  programs/<program_id>.json
```

**Python in-process**, not a Go server driving a Python worker. The extractor is
Python; putting the HTTP layer in the same process removes the subprocess
boundary that produced the original bug, makes SSE trivial, and lets manual
authoring call the detector directly. Cost: it breaks the repo's Go-CLI
convention, so porting to the canonical stack later means writing a Go wrapper.
Accepted — this is a development pipeline, ported later.

**Frontend stack matches `offline-programming/v1/ui`**: plain TypeScript, Vite,
three.js 0.170, no framework. Consistency with the one other 3D UI in this repo
matters more than any framework preference.

### Threading

FastAPI is async but OCCT is CPU-bound and does not reliably release the GIL.
Run it on the event loop and the SSE stream stalls exactly when progress matters
most. So: `ProcessPoolExecutor` with workers warmed at startup, OCCT imported in
the pool initializer.

That reintroduces a process boundary but not the bug. The bug was *cold start,
plus a timeout, plus silent fallback*. Warm pool, no timeout, no fallback.

**Documented consequence:** cancelling a running OCCT call means killing the
worker, which loses its warmth. The next job pays a cold start. This is a known
behaviour, not a surprise, and the UI should say so when cancelling.

**The pool is started with `spawn`, not the Linux default of `fork`.** OCCT runs
its boolean algorithms on an internal thread pool whose threads outlive the
operation. `fork()` copies only the calling thread, so a mutex an OCCT worker
happened to be holding is copied *locked* and never released — the child then
blocks forever on its first OCCT call, with no error and no traceback.

Found by a test, not in production: `pytest tests/programmer/test_clearance.py
tests/programmer/test_server.py` hung where either file alone passed, because the clearance
fixtures cut a wedge out of a block before the server forked. Nothing in the web
process runs a boolean *today*, which is the only reason it had not happened
yet. Spawn costs a fresh interpreter and a cold OCCT import per worker — which
is exactly what the warm pool and the readiness endpoint already exist to pay
for at startup rather than inside a request.

---

## 3. Async model

Job submit -> job id -> **SSE stream**. Not polling, which is either wasteful or
laggy. Not WebSocket, since nothing streams client to server.

**R-UI-3 — Staged progress, never a spinner.** The stages are known and
distinct; each reports separately:

```
topology -> tessellating -> detecting -> accessibility
```

A spinner says nothing. "detecting joints, 340/861 face pairs" tells the user
whether to wait or go for coffee.

**R-UI-4 — First-run environment bootstrap is an explicit state**, not a
timeout: "Preparing CAD environment — 2.5 GB, first run only." This single
change is what kills the original bug.

**R-UI-5 — Cancellable.** Someone will drop a 200 MB assembly in by accident.

**R-UI-6 — ~~Cache by STEP sha256~~. Withdrawn.** Topology, tessellation and
detection are pure functions of the file *for a fixed pipeline*, and the
pipeline is under active development. See "No result cache" below: analysis
always recomputes.

**R-UI-7 — Jobs survive a page reload.** Reconnect to the stream by job id.

---

## 4. API surface

```
GET  /api/health                              worker pool readiness
POST /api/workspace/steps                     upload -> {step_id, sha256}
POST /api/jobs/analyze  {step_id, through?}   -> {job_id, stages}
GET  /api/jobs/{id}/events                    SSE: stages, then result
POST /api/jobs/{id}/cancel
GET  /api/steps/{sha}/topology                artifact from the last run
GET  /api/steps/{sha}/mesh                    display mesh, binary
GET  /api/steps/{sha}/joints                  detected joints
POST /api/steps/{sha}/joints/propose          {face_a, face_b} -> joint | diagnosis
POST /api/seams/frames          {seam}        -> sampled {P, t, r, b}
GET/PUT /api/workspace/programs/{id}
POST /api/validate                            schema + warnings
POST /api/welds/clearance       {weld}        does this torch fit, at these angles
POST /api/welds/optimise        {weld}        search the angles -> job (lattice/search/verify)
GET  /api/settings                            resolved settings + every layer
GET  /api/settings/export.json                the same, as a file to keep
POST /api/settings/resolve      {settings}    the same, with a project's layer on top
PUT  /api/workspace/settings    {settings}    this operator's layer
```

### Loading and analysing are two steps

`through: "tessellating"` truncates the run after the mesh. That is about a
second; the full pipeline is about a minute, almost all of it accessibility.
Choosing a file should not commit you to the minute before you can see whether
it is even the right file — so Load previews, and Analyze is a separate,
deliberate act.

It is the same job, not a second endpoint: same SSE stream, same cancel path,
same stage events. The response carries the stage list this run will emit,
because a progress bar that took its denominator from `/api/health` would get
all four and stop at half on a preview. The `done` event marks a preview as
one, so no client reports "0 seams" for a workpiece nobody has looked for seams
on yet.

A failed or cancelled run no longer deletes the whole artifact directory, only
`joints.json`. Each artifact is written after its own stage finished, so the
mesh is whole or absent — and wiping it would take away the geometry the
operator is looking at, leaving the next request 404ing against a populated
viewport.

### Clearance is on demand, and never a stage

`/api/welds/clearance` takes a weld-list **entry**, not a seam: the angles it
answers for are the product of the candidate's defaults, a preset and the
operator's overrides, and resolving that a second time in the browser would
eventually answer for a weld nobody is going to run.

It is deliberately not a pipeline stage. It depends on the torch and on the
authored profiles, neither of which exists at analyze time, and a stage would
want to write its verdict into `joints.json` — which is the tooling-independent
artifact, and stays that way. `test_server.py` greps it for `torch`, `nozzle`
and `standoff` after a clearance run to keep that a check rather than a habit.

Seconds rather than minutes, so it gets a plain request with no SSE. It runs in
the pool because OCCT does not release the GIL, and it writes nothing.

### The angle search is a job, and its answer is a proposal

`/api/welds/optimise` is the same weld-list entry, taken the same way, but it
is a **job** with stages `lattice → search → verify` on the existing SSE stream
and cancel path. The lattice is 100–300 k rays; on pipe_bracket's tee that is
about 13 s, and one opaque submission would give neither progress nor
parallelism, so it is chunked over the sample axis exactly as accessibility is
chunked over joints.

Three decisions worth keeping:

**Everything cheap happens in the request, before a job exists.** Resolving the
weld, reading its limits, choosing the bins and the standoff ladder are all
milliseconds, so a weld that cannot be searched — a profile outside its own
declared limit, a band with no bin in it, a reversed seam — comes back as a 422
with a reason. The response also states what the run will cost in **rays**, not
seconds: seconds depend on the machine, and "this will take a while" is not a
number anyone can plan around.

**There is no `frames` stage**, though the plan named one. Frames are computed
inside the lattice, once per sample and reused across every cell, and each
chunk computes only its own. A stage for them would mean shipping a frame table
between processes to save nothing — and a stage that is not really a separate
measurement is the same class of lie as a fake fixture, which is why `STAGES`
has no `hashing` either.

**`verify` measures the before AND the after with the fine fan.** The lattice
ranks cells at 33 rays and the emitted profile is the *simplified* one, so the
number the operator sees has to be measured on what will actually be welded —
and so does the number it is compared against. A 70% → 88% built from a coarse
baseline and a fine result is not a comparison.

The answer never edits the weld on its own. It waits behind Apply and Discard,
because a search runs for tens of seconds while the operator is still looking
at the current profile, and a result that appeared under them would be a change
nobody made. Applying writes an ordinary override plus a `provenance` sidecar —
the torch, the knobs and both coverage figures — so a field row can read
`optimizer · 70% → 88%` and can be shown as stale once the torch changes.

Cancelling reports `cancelled`, not `failed`. A running OCCT call cannot be
interrupted cooperatively, so cancelling kills the worker and the pending
future raises; calling that a failure would blame the code for the operator's
own action.

### Settings are layered, and the layers are visible

`GET /api/settings` returns the resolved document *and* the layers that made it
-- module, workspace, project -- because "the torch radius is 8 mm" and "it is
8 mm because this workspace says so" are different answers and only the second
is actionable. R-UI-2 applies to configuration exactly as it applies to
geometry.

**And the same rule one level up.** Settings name a preset rather
than containing it, so the response also carries `references` -- one row per
kind with the id, where the file came from, and the reason if this machine
cannot resolve it -- plus `standoff_mm` and `standoff_source`. The fit angle in
the dialog is computed at the gap the server measures clearance with, and says
which preset the gap came from; a fit angle without its distance is not a
measurement. A reference that resolves to nothing makes `torch_summary` null and
puts the sentence in `problems`, so the command is disabled WITH the reason on
it (D8) rather than drawing a gun nobody owns (R-UI-1).

The project layer travels inside `.weldproj`, so a bundle reopened elsewhere
shows the torch it was authored against rather than whoever-opened-it's. It is
**not** the manifest's `analysis_parameters`: that records what the candidate
set came from, this records what the next analysis and the next weld will use,
and merging them would silently change the candidates under a saved project.

The readiness endpoint exists so the UI can disable Load until the pool is
actually up, rather than discovering it at timeout.

**Behaviour the operator can feel is a setting, not a constant.** The
`playback` block — `autoplay_pause_s`, `fallback_pass_s`, `comb_seconds` —
governs how the trajectory panel plays a weld back, and all three began life as
constants in `seam-preview.ts` and `viewer.ts`. That made "how long does
autoplay sit still between seams" a question you answered by editing code and
rebuilding, when it is really a question about the person watching and the part
they are watching. Anything of that shape belongs here.

Two of them are load-bearing beyond taste. `fallback_pass_s` is only ever the
fallback: a weld that states a travel speed plays in REAL time and no setting
overrides that, because 1× meaning one second of playback per second of welding
is the whole point. And `comb_seconds` shares its axis with playback
deliberately — at the default of 1.0 the torch crosses exactly one comb hair
per second at 1×, so the comb's spacing and the playhead's speed are one
statement rather than two that can drift apart.

The TypeScript fallbacks (`FALLBACK_SECONDS`, `COMB_SECONDS`, and
`HOLD_SECONDS` behind the transport's `holdSeconds`) exist only for the moment
before the settings fetch lands, and must match
`settings/default.settings.json` — if they drift, the app behaves one way for
the first second of a session and another way afterwards.

The block now governs the Plan tab's replay as well, through the same shared
transport — `autoplay_pause_s` is one answer about the person watching,
not one per panel.

### No result cache

`analyze` always recomputes. `artifacts/<sha>/` is only where a run leaves its
output for the GET endpoints; nothing reads it back to decide whether to skip
work.

Artifacts are pure functions of the STEP **for a fixed pipeline**, which is not
the same as pure functions of the STEP, and while this module is being built the
pipeline changes far more often than the fixtures do. A cache keyed on the STEP
alone therefore serves answers a bug has already been fixed in — which happened:
after the blocked-span boundary fix the server kept reporting the old 59.63 mm,
because dog_box.step had not changed. Hashing the pipeline sources into the key
fixed that instance and left the mechanism in place; the mechanism is what is
gone. Recomputing dog_box costs ~13 s and pipe_bracket ~7 s, which is worth paying
to never wonder whether a number is current.

---

## 5. Display artifacts

Derived from topology, stored under the sha, and **explicitly display-only**. Not
collision geometry — `visual_meshes_as_collision_meshes` is already a smell
elsewhere in this repo and must not be repeated here.

### Mesh

Custom binary plus a JSON sidecar: positions, normals, indices, and a **face
range table**. Not per-triangle ids — tessellate face by face and record:

```
[(face_id, tri_start, tri_count), ...]
```

42 entries for `dog_box.step` rather than thousands. This is also
`BufferGeometry.groups` in three.js, so it is idiomatic as well as compact, and
per-face highlighting becomes a material swap rather than a geometry rebuild.

glTF was considered and rejected: per-face provenance in glTF is fiddly, and the
custom format is roughly 50 lines each side and smaller.

### Edge lines

A separate line buffer with an **edge range table**, same structure. Needed for
edge-chain picking, and for drawing the B-rep wireframe over the shaded mesh.

### Tessellation deflection

Tight — 0.01 mm is negligible at these sizes. The tessellation is a pick target
and a display, and precision near face boundaries matters for authoring.

---

## 6. Picking

**GPU colour picking, not raycasting.** Render each CAD face to an offscreen
target in an id-encoded colour, read the single pixel under the cursor, decode
the face id.

Why, for this workpiece specifically:

- O(1) per pick regardless of triangle count. Three.js's raycaster is O(n) over
  triangles without a BVH, and a real weldment is 100k+ triangles.
- Pixel-exact at silhouettes and where faces are nearly coplanar.
- **Handles coincident faces**, which is every joint in this workpiece — contact
  faces sit at exactly zero separation. Raycasting into coincident geometry is
  precisely where z-fighting ambiguity bites, and that is the geometry manual
  authoring cares about most.

Cost is one extra render pass on hover. For a CAD picker that is the standard
trade.

**Edges take priority over faces within a few pixels**, so clicking near a
boundary selects the edge rather than gambling on which side's triangle won.

Selection always speaks **CAD face and edge ids**. Triangles are the pick
mechanism and never leave the renderer.

---

## 7. Making the angles legible

Everything above is plumbing. This is the actual design problem.

The user chooses weld intent over detected joints. File loading is trivial; the
difficulty is that work angle and travel angle are abstract numbers that mean
nothing on a slider. Three views fix that:

**Draw the rail.** A second curve in 3D. It *is* the 0-degree reference, and
seeing it makes "work angle 0" concrete instead of arbitrary.

**Torch glyph on a scrub bar.** Drag along the seam; the glyph sits at `P(s)`
oriented by the current angles. Immediately shows fouling against the workpiece,
awkward orientations, and what the angles actually do.

The bar runs 0..1 along the **seam**, not the parent joint, so every position on
it is weldable and there is nothing to clamp the operator out of. An earlier
version listed joints and clamped the scrub across blocked stretches; splitting
the joint into one seam per reachable run removed the problem instead of
steering around it. A wrapping span (`start_s > end_s`) is traversed the way it
is welded, straight through s=0.

**Live cross-section inset — the highest-value view.** At the scrub point, draw
the plane perpendicular to travel: the two groove faces, the dihedral wedge, the
bisector, and the torch at the current work angle. Cheap to draw, because the
frame is already computed. This is the picture that makes the whole model click.

---

## 8. Where the frame math lives

This has a trap in it.

If the client computes torch orientation in TypeScript and the server computes
it in Python when saving, **they can diverge** — and a sign-convention
divergence is exactly the bug class this design exists to avoid. But
round-tripping on every slider drag is unusable.

**The server sends frames; the client applies angles.**

- Server computes and sends, per sample: `{P, t, r, b}`. The subtle work —
  orthogonalization against the tangent, rail correspondence, per-segment
  matching at corners — happens once, in the tested Python.
- Client applies only the two rotations. About ten lines, safely duplicated.
- Frames do not depend on the angles, so dragging a slider needs no round trip.

---

## 9. Manual authoring

Detection is a heuristic. The UI assumes it both under- and over-produces.

### Face-pair joints

Click two faces, `POST /joints/propose`. **It must reuse the auto-detector's
code path** — a manual joint and an auto joint should be identical records
differing only in how the pair was chosen. Otherwise there are two subtly
different notions of "joint".

**Failure must be diagnostic.** "Not a joint" is useless. "Normals are 47 deg
apart, not antiparallel" and "planes are 2.3 mm apart, beyond the coincidence
threshold" tell the user whether the model is wrong or the threshold is.

This also argues for **surfacing near-misses automatically**. Two faces 0.3 mm
apart are probably a joint with a root gap; they should appear as a
low-confidence candidate rather than something to hunt for by hand. Good
near-miss reporting is the cheapest way to reduce how often manual authoring is
needed at all.

### Arbitrary weld lines: 1D straight lines only

For testing and repair welds. Deliberately restricted to a straight line
segment, which removes most of the complexity:

- no curve fitting, and no interpolating-vs-polyline decision — degree 1, two
  control points
- no self-intersection, no curvature limits, no corner policy
- no rail correspondence subtleties — a straight rail parallel to a straight seam
- **no arc-length reparametrization at all**, since `s` is proportional to `u`
  on a line

What it needs: two endpoints placed by click or gizmo, translate/rotate on the
whole line, draggable endpoint handles, **numeric entry editing the same two
points** (for a testing tool that matters more than dragging — "a 100 mm line at
exactly z=200" is repeatable in a way a drag is not), and endpoint snap onto a
face so repair welds land on material rather than floating near it.

Rail is **always an explicitly prompted fixed direction**. Correct and intuitive
for a planar seam. Note the consequence: the frame construction orthogonalizes
the fixed vector against the tangent at every point, so it always yields a
*valid* frame, but on a seam curving out of plane the effective angle relative to
the local surface drifts even though the field stays constant. On a straight
line that cannot happen. The degeneracy case — tangent parallel to the fixed
vector — is a single check, `|t . fixed|` near 1, and needs a visible warning
rather than a silently bad frame.

**This tool is worth building early.** On a straight line with a fixed rail and
constant angles the torch orientation is identical at every point, so nothing
varies to confound a bug. It is the cheapest end-to-end validation of the frame
pipeline, before any detected geometry is involved.

Verified: such a seam validates against the weld program schema with **no schema
changes** — `weld_joint_ref: null`, `reference.mode: fixed_vector`, degree-1
curve, rail drift 0.00e+00 along the seam.

---

## 10. Parameters, and the preset library

Nobody types wire feed speed from scratch. A **preset library** keyed by
process, material and thickness — a weld preset is just the `process`,
`orientation` and `weld_geometry` blocks from the schema, so this costs nothing
structurally.

Per-seam editing: direction toggle, span trim, passes (root/fill/cap), process
block.

### Three kinds, one mechanism, two surfaces

A preset is a named file you keep several of and pick between, and three things
turned out to be that shape: the **procedure**, the **torch**, and the **search
policy**. The last two were blocks of numbers inside `settings.json`, which made
"this cell's gun" something you retyped per workspace instead of something you
named once — and worse, produced a quantity with two authors and a silent loser.
The preset-kind split has the argument and the measurement, in the commit that made it.

`Tools ▸ Presets…` opens the library, beside `Settings…` because the two split
one question: this is what you weld with, that is how this machine behaves. One
`MenuDef` entry, so it appears in the dropdown and the flat strip without either
renderer knowing about it. `Ctrl+Shift+P` rather than `Ctrl+P`, because
`matchesAccel` calls `preventDefault` and would take the browser's print away.

**The two surfaces share one renderer, and this is load-bearing.** The
weld-parameters dialog and the library both draw a list of numbered fields with
an origin badge and a revert. They used to draw it twice — the weld editor had
one copy and the settings dialog a second that had already drifted, with
different markup, a different revert and no `varies` branch. A third would have
been the one that finally disagreed about what an override looks like.

So `state/field-specs.ts` holds the field table, keyed by kind, and
`panes/prepare/field-form.ts` draws it. The difference between the surfaces is
pushed into one interface, `FormModel`, and it is exactly three things:

| | weld editor | library |
|---|---|---|
| what `origin` names | `candidate` / `preset` / `override` | `module` / `workspace` / `project` |
| what `set` writes to | `entry.overrides` | the sparse draft |
| what revert means | drop the override | back to the shipped file this shadows |

`field-specs.ts` is DOM-free and node-importable, which buys a real check:
`test_the_ui_field_table_only_names_paths_a_seam_actually_has` walks every weld
path against the program schema. A dead row is otherwise invisible — it renders,
it accepts a number, and the number lands at a path nothing reads.

### Its own dialog, and why

The library is `<dialog id="presets">` rather than a fourth owner of `#modal`.

1. **Ownership.** It opens *from* the parameters dialog ("Edit…" beside the
   picker). `#modal` has one owner at a time and `replace()` would destroy the
   weld being edited; native `<dialog>`s stack in the top layer, Escape closes
   only the top one, and closing the library leaves the editor as it was.
2. **D7.** `#modal` is `min(460px, 92vw)` and its contents are tuned to it.
   Widening it to suit a master/detail plus an n-column compare table would
   reflow three other surfaces.
3. **D3.** The compare grid is one flexible column — the label — with every
   value column fixed. At 460px the label collapses and identity truncates,
   which is D6's failure. At 1040 it does not.

### Comparing

Ticking two or more presets swaps the detail pane for a table: one row per
field, one column per preset, and a row whose values disagree carries
`data-differs="true"`. The mark is a **face and a weight, never a hue** —
`--accent`, `--edit`, `--warn` and the rest each already mean one thing (D2),
and "these two numbers are not the same" is not one of them. Units go in the
column head, once (D6).

The point of the comparison is the four rows that differ out of thirty that do
not, so the agreeing rows stay quiet.

### Broken presets stay in the list

`load_presets` returns a file that does not parse or does not validate carrying
its `problems`, rather than dropping it. The library is the surface that exists
to fix those, so filtering here would make the one file worth looking at the one
you cannot see. A broken preset is listed with its problems and its path, is not
tickable for comparison, and its kind's tab says how many there are before the
tab is opened.

The weld editor's picker does the opposite and is right to: a broken procedure
stays OUT of the `<select>`, because it cannot be applied — and the count
appears under it, linked, so the menu is never quietly one item short.

**Units: show millimetres everywhere, including geometry**, because welders
think in millimetres. Convert at the schema boundary. Recorded here so nobody
later "fixes" the schema to match the UI — the geometry/process unit split in
`SEAM-MODEL.md` R6 is deliberate.

**v1 supports constant-valued scalar functions only.** The schema can express
`piecewise_linear` profiles — travel speed slowing into a corner, work angle
tapering — and the reference fixture uses one. Those are not authorable from the
UI in v1. This is a deliberate scope choice, not an oversight: the schema is
allowed to express more than the UI can reach.

---

## 11. Validation

Live schema validation as the user edits. **The UI must not be able to save an
invalid program.** Plus warnings that are not errors:

- rail and seam arc lengths mismatched
- work-angle sign ambiguity on a closed loop (`DECISIONS.md` section 10)
- weave amplitude implausible against plate thickness
- manual seam far from any material — a floating test line is legitimate, but
  the user should know
- rail degenerate: `Q-P` parallel to the tangent

**Clearance is checked for every weld, not for the selected one.**
`clearance-sweep.ts` keeps an answer current for each entry in the list, keyed
on a per-weld fingerprint — the STEP, its seams, its span, its resolved angles,
its direction, its standoff and the torch — so dragging one profile costs one
check while changing the torch correctly costs all of them. The weld list marks
each row, and **no mark means checked and clear**: an unchecked weld shows `?`,
because an unchecked list must never read as a clean bill of health (R-UI-1).

This is what the Validate gate reads. It used to read the single record for
whichever arc the preview happened to be showing, so a program with nine
fouling welds validated as long as the tenth was on screen — a bug you could
only find by clicking every weld, which is the work the check exists to save.

**A sharp corner is a refusal, not a warning.** A weld naming several pieces of
one group welds through the corner they were cut apart at, where the tangent
flips and the torch would have to reorient in place with the arc lit — an
unstable move at the hardest part of the joint. `program.resolve_weld_list`
refuses it, the candidate list does not offer it, and reopening a project under
a finer corner threshold returns one weld per piece rather than one that spans
them. It used to be a warning here (`seam spans a C0 corner with corner_policy:
continue`) and an option in the UI; it is neither now. What counts as sharp is
still a knob — `max_corner_angle_deg`, defaulting to 5° because that is the
work-angle tolerance a seam already declares — but once a corner is over it,
the split stands.

---

## 12. Build order

1. **Stage 2 detector** — overlap boundary, groove faces, dihedral. Library with
   tests, no UI. There is nothing to view without it.
2. **Tessellation** — mesh with face ranges, line buffer with edge ranges.
3. **Server** — health, upload, analyze job, SSE, workspace artifacts.
4. **Viewer** — mesh, joints, rails, frames, cross-section inset.
5. **Manual 1D line tool** — cheap, and validates the frame pipeline in isolation.
6. **Selection, parameters, save.**
7. **Manual face-pair authoring** — picking and `propose`.

Regenerating the reference fixture from real geometry falls out around step 2.

---

## 13. Visual design rules

The R-UI rules above are about honesty. These are about legibility, and they
exist because the pane had drifted into a state where everything on it had the
same weight. Each is written against a measurement, so it can be checked rather
than argued about.

**D1 — A level differs in at least two of {size, weight, colour}.** Never one
alone, never more than three levels in a column. *Measured:* the stylesheet had
three font sizes — 10px (20 uses), 11px (12), 12px (1) — and section headings
were the 10px one, **smaller than the 11px rows beneath them**. A heading now
differs from its rows in four channels: weight, case, tracking and a rule.

**D2 — Colour is a vocabulary: one hue, one meaning.** *Measured:* `--hot`
carried seven meanings across 19 rules — hover, running, failed, error,
selected, edited, broken. A colour that means everything means nothing. The
vocabulary is now `--accent` (the subject: weld line, selection, next action),
`--danger`, `--collide`, `--edit`, `--busy`, `--ok`, `--warn`. **Hover is not a
hue** — it is a lighter face — because a transient state must not borrow a
colour that a persistent one owns.

**D3 — Columns do not move.** A row grid may have exactly one flexible column;
every other width is fixed. *Measured:* the candidate row was
`13px minmax(0,1fr) auto 38px auto auto`. Three `auto`s mean each row
negotiates its own widths, so forty rows never form columns — throwing away
the only thing a table is for.

**D4 — Depth is structural, not indentation.** A group is a band; its children
hang off a visible spine. A 10px indent and a grey name is not a hierarchy.

**D5 — Emphasis goes to the action that advances the task**, not the most
expensive one. At most one primary control visible per pane. *Measured:* the
only saturated element in the left column was *Re-analyze* — the rarest action,
and one that discards completed work — while the primary loop was an 18px grey
`+`.

**D6 — Identity is never truncated; units are named once.** Abbreviating
systematically (`solid_0` → `s0`) is lossless and fine; clipping to `…` is not,
because it promises a way to see the rest. Units belong in a column header, not
on fourteen rows.

**D7 — Space follows density.** The left column carries the longest list in the
app and was clipping joint names at 300px while the right column sat mostly
empty. Panels are sized to what they hold.

Taken to its conclusion, this is why the prepare pane has **two columns and a
drawer** rather than three columns. Everything true of the whole job — the
workpiece, the candidates, the weld list, and the program built from it — lives
in the left column and is always on screen. TRAJECTORY is true of ONE weld, so
it slides in over the right of the viewport when a weld is selected and is gone
otherwise: a panel that can only answer for a selection has nothing to say
without one, and a column that is empty half the time is a column spent on
nothing. It is also the tallest panel in the app and grows with the seam it
draws, so as a column it could only evict its neighbours or be evicted by them.
The drawer is `position: fixed`, so opening it does not resize the viewport —
selecting welds down a list never rebuilds the WebGL drawing buffer.

**D8 — Every state is legible, including disabled and empty.** A disabled
control keeps its label and states its precondition in `title`; 30% opacity
made it impossible to see what was being denied. An empty panel is a sentence,
not a reserved box — and it is *absent*, not empty, when there is nothing to
say.

**D9 — The viewport is the subject, not a fourth panel.** The arc being worked
on is the heaviest line in the scene. Widths come from `LineSegments2`, because
WebGL ignores `linewidth` on ordinary lines — so a "thick" line drawn the
obvious way is one device pixel regardless.

*A line behind metal is drawn dimmer, not equally.* Every weld line runs two
passes: depth-tested normally at full strength, and `GreaterDepth` at
`HIDDEN_OPACITY` of it — which draws only where something occludes it. The
x-ray came first and the dimming did not, and at 0.9–1.0 behind the part the
whole seam network read as a wireframe cage floating in front of the workpiece
rather than as seams on it. *Measured* on `dog_box`, one framed view: 2,080
weld-coloured pixels with the hidden pass off, 7,896 with it at full strength —
so three quarters of what was on screen was line the part should have been in
front of, all of it as bright as the near edges.

Occluding honestly instead was the other option and is wrong here: a tee seam's
root runs along the joint midplane, inside the material, so it is not
occasionally hidden but buried from every angle — which is the "far too
invisible" the x-ray was introduced to fix. Dimming keeps both halves of the
requirement. The collision line is the one exception and keeps no depth test at
all: a foul hidden behind the part causing it is the one to see, and dimming it
would be dimming the alarm.

**D10 — One fact, one mark.** A collision is drawn on the weld line and
nowhere else. Marking the rail as well was tried and removed: the rail is the
*datum* the work angle is measured from, not a thing that collides, and
breaking it made two lines say the same thing while costing the datum its one
job — being continuously visible so the comb has something to be read against.
Redundant marking does not add emphasis, it dilutes what each mark means.

**D11 — A region is separated by a step, not a hairline.** Surfaces come from
one named ramp, and two stacked regions must be far enough apart on it that the
boundary survives without the rule that draws it. *Measured:* every surface in
the stylesheet lived between `#14161a` and `#26` — eighteen levels of grey for
the whole application — so the header (`#1b1e24`) and the menu strip
(`#171a1f`) differed by **four**, and the section headings sat on the same face
as the column they divided. Two bars four levels apart is one bar with a line
in it. The ramp is now `stage → brand → bg → panel → card → raised → chip`,
with the identity bar at `brand` and the menu strip at `raised`, 27 levels
apart; a section is a **card** on the column's darker ground, so a column
decomposes into its sections before you read a word of them.

Two corollaries, both of which cost something and both of which were paid:

- *The column pays for the card, not the content.* A card costs 20px of row
  width — a border and 9px of padding each side — which at 340px clipped every
  joint name back to `solid_0/solid_2…`, losing the `tee` qualifier and
  breaking D6. The left column is 360px to give it back exactly.
- *A list card shrinks but does not grow* (`flex: 0 1 auto`). The list still
  scrolls rather than the column, but a card that also grew stood 599px tall to
  hold one weld — and once a section has a visible border, slack stops being
  empty space and becomes an empty box, which is D8.

**D12 — Chrome uses application vocabulary, not page vocabulary.** The menu bar
is a menu bar: flat titles with no border and no radius, filling the bar's full
height, opening dropdowns. *Measured:* the strip was three bordered, rounded,
25px chips floating in a 34px bar — the visual grammar of buttons on a web page,
which is why it did not read as a menu however it was coloured. A chip also
leaves dead pixels above and below itself that a real menu title does not have.

*And the bar belongs to the SHELL, not to a plane.* It was built by the Prepare
pane and appended to that pane's menu host, so the tab shell hid it along with
the plane: stepping over to Plan took `Open Project…`, `Save Project…` and the
bundle's status off the screen entirely. A project is not a property of the
plane you happen to be standing on — you can place a part, plan its motion,
notice something and want to save from exactly there. `shell/session-bar.ts`
holds the strip; Prepare declares the commands as `MenuDef[]`, because what
they operate (the load panel, the save panel, the settings dialog) lives there
and should not be lifted out to satisfy a menu.

Three consequences, each one a thing that was wrong once the bar outlived the
plane:

- *The bar refreshes itself, from its own store subscription.* The tab shell
  only updates the plane on screen, so a bar refreshed from Prepare's `update`
  went stale the moment you left it — which is the situation it now exists to
  be useful in. It subscribes after the shell does, so a command's `blocked()`
  asks panels that have already seen this state.
- *`blocked()` reads the store, not what the last `update` cached.* Same
  reason, one level down: `saveBlocked` asked a `hasSource` flag that only
  Prepare's `update` writes.
- *`Open Project…` moves you to Prepare first.* It re-analyses the STEP — most
  of a minute — and the stage progress, the candidates it refills and any
  failure all render in Prepare's left column. Invoked from Plan it would leave
  the operator watching a placement pane do nothing, then change under them.
  Saving needs no such move: its status is in the bar, which is now wherever
  they are.

What makes it read as an application rather than an imitation is the behaviour,
not the paint, so all of it is implemented: with a menu open, **hovering another
title switches to it**; Escape closes and returns focus to the title; Up/Down
walk the enabled rows only, wrapping; Left/Right move along the bar; a
`pointerdown` anywhere else dismisses. Accelerators are declared on the same
object that draws the hint, so `Ctrl+O` on the right of a row and the key that
actually fires cannot drift apart.

Two constraints this had to satisfy:

- *The dropdown is `position: fixed`, parented to `<body>`.* `#menubar` is a
  horizontal scroll container and an absolutely-positioned child of one is
  clipped by it — the thing that killed the first popup attempt here and left a
  comment in the stylesheet forbidding vertical lists in the strip. Fixed boxes
  lay out against the viewport, so no ancestor's `overflow` reaches them. The
  constraint was real; "never put a list in the strip" was the wrong conclusion.
- *Only commands that exist.* Two menus — File and Tools — because there are
  three session-level commands. A File menu padded out with a greyed
  `Recent projects` would describe a feature this build does not have, which is
  the same class of lie as R-UI-1's silent fallback. `Load STEP…` and
  `Examples…` stay on the WORKPIECE panel: the left column reads top to bottom
  as the order of work, and moving the first step into a dropdown would break
  that in order to make a menu look fuller.

**D13 — A control that means the same thing on two planes is the same
control.** Not "looks the same": the same module, instantiated twice. The
playback transport — `◀◀ ▶ ▶▶ · speed · repeat · auto` — is `src/transport.ts`,
driven by Prepare's trajectory panel and by Plan's motion drawer.

*Measured:* they had diverged before the module existed. Prepare's row had a
speed multiplier, two independent standing modes and a beat between passes;
Plan's replay was a `play` button, a `start` button and a `setInterval` at
125 ms a sample. Same verb, two meanings — and the fixed tick was the worse
half, because 125 ms was a rendering choice standing in for a fact the plan
already carried. The candidates ship `times_s`, the time each sample is due at
the seam's authored travel speed, so **1× now means the same thing on both
planes**: one second of playback per second of welding.

The split the module enforces is what makes two hosts possible at all. The
transport owns the clock — one float, 0..1 through the pass — and the host owns
what a position MEANS. That is not tidiness: Plan can only pose the arm at the
samples the planner produced, and a host that owned the playhead would round
every frame's advance back to the sample it started on, at which point playback
sits still. The clock is continuous, the pose is not, and the chart draws both
— the cursor slides, the arm steps — because interpolating between two planned
samples would draw a configuration the plan does not contain.

Two rows in one document, so the ids are prefixed per row (`play-toggle`,
`motion-play-toggle`) and the stylesheet addresses classes. Panes are hidden
rather than torn down, so both exist whichever tab is on screen.

*The one thing that is deliberately NOT shared is the speed ladder.* Prepare
stops at 4×; Plan runs to 16×. What 1× means is a fact and must not differ
between planes — but how fast is worth offering is a property of what is being
played, and the two are different activities. A weld is watched: past 4× the
torch crosses more than four comb hairs a second and the profile it is drawing
stops being readable, which is the reason to play it back at all. A trajectory
is reviewed: "did the wrist flip anywhere in this plan" is a question about
eight seams at their own welding durations, and at 1× that is nineteen seconds
for one of them. 16× is the top because of what the plan contains rather than
taste — samples are 250 ms apart, so 16× is a pose every 15.6 ms, already
faster than a frame.

**D14 — What a wait cost is part of what it produced.** The motion drawer
draws the planner's own stage breakdown: every stage the response reports —
`collision_model`, `dp_seam_search`, `weld_trajopt`, `connecting_trajopt` —
biggest first, each with its share, and `dp_seam_search` shown as its parts
summed across the seams rather than as one row, because those parts are the only
ones reported per seam. Every seam row carries its own total, with its breakdown
in the `title`. The numbers are measured in `weld_motion_planner/timing.py` and
travel in the response as `timings_ms` — the UI computes no durations of its
own, so the drawer and the planner's log can never disagree about where the time
went.

The drawer reads the stage names off the response instead of knowing them, with
one exception: it must name the stage it expands. It once knew them all, and a
planner that grew stages above the seam search left the drawer matching none of
them — on a full plan that put 212 s of 214 s into `unattributed`. A stage
added to the planner now appears with no edit here; only a RENAME of the expanded
one needs one, and `tests/motion_planner/test_plan.py` holds the two literals
together.

*Measured:* there was nothing. A plan took as long as it took, and "is forty
seconds normal for this part?" could only be answered by a person with a
stopwatch and access to the server's terminal — which is to say, not by the
operator, who is the one waiting. IMPLEMENTATION.md M9 had asked for the
breakdown since before the tab existed.

Three rules it has to keep, all of which the first draft would have broken:

- *A stage name is jargon until it is explained.* `ik`, `targets` and `screen`
  are the planner's vocabulary; each row's `title` says what the stage IS in a
  sentence. D8 applies to a label nobody outside the pipeline can read exactly
  as it applies to a disabled control.
- *The bar is scaled to the biggest stage, the percentage to the total.* Bars
  scaled to the total make every row but one a sliver, and the ordering — the
  only thing a bar is for here — becomes unreadable at the bottom of the list.
- *The bar wears no hue.* D2 — a duration is a quantity, not a state, and every
  colour in this app already means something else. Length carries it.

The gap between the stages and the total is shown as `unattributed` when it is
a real share of the run, for the reason the planner keeps the two separate:
a breakdown that adds up to its own total by construction is a breakdown that
cannot report the time nobody claimed.

### Typeface

System sans for prose, labels and headings; monospace only for what is code or
measurement — ids, hashes, lengths, angles.

There is **no repo convention** either way: 17 of 40 UIs set a monospace body
font, 23 use sans or system-ui, and `offline-programming/v1/ui` — the UI this
document says the frontend matches — uses Inter. The monospace here was a local
choice whose stated justification, "the numbers line up for free", is already
covered by `font-variant-numeric: tabular-nums`, which works in any face. What
it cost was real: a uniform advance width gives every word the same texture and
blunts bold, which is the opposite of what D1 needs.

**D15 — Continuous motion is an addition, never a replacement.** The motion
drawer can ask the planner for M5's trajectories and replay them, and it does
both behind two separate checkboxes: *generate* asks the server for them,
*play the continuous curve* switches what the playhead drives. Two switches
and not one, because they answer different questions — the sweep's discrete
candidates are what the search DECIDED, and they stay worth looking at after
a trajectory exists for the same seam.

The response is a superset: a seam gains a `trajectory` and loses nothing, so
every reader written before M5 keeps working (`test_plan.py` asserts the two
wire shapes are equal but for that one key). Generation is off by default
because it is a different order of wait — the sweep answers in seconds, one
synthesis per candidate plus a CPU verifier per seam takes minutes — and the
button says so before it is pressed rather than after.

What arrives is the Hermite itself, knots and velocities, not a dense
resampling of it. `hermite()` in the drawer is nine lines, the payload is a
twentieth of the size, and the arm on screen moves along the curve the
planner emitted instead of along a polyline whose resolution the server
picked. That difference is visible: between two knots the discrete replay
steps and the continuous one does not.

Two things the drawer must never say, both of which are one sentence away
from being said wrongly:

- *The certified bound and the optimizer's own number are separate figures.*
  M5 reports the tracking error at midpoints it chose; the verifier bounds every
  instant. On the corner seams of `bracket_a_placed` those differ by a factor
  of nearly five, so merging them into one "tracking" figure would show a
  seam as weldable that is not.
- *"Collision" is not the same as contact.* A weld pass runs 1.3–2.4 mm off
  the part by design, inside the verifier's 2 mm pad (`docs/issues/0007`), so the
  T2 verdict reports `penetrating` separately and the drawer reads that one.
  An operator shown "collision" on every correct seam learns to ignore the
  word.

**The plan joined (M6, 2026-08-13).** Generating now also returns `connecting_trajectories`
— approach, a transit between every consecutive welded pair, retract — and
the drawer's list becomes the PLAN: link rows interleaved with the seam
rows in execution order, indented and quieter, and the arrows and autoplay
walk stops rather than seams. Selecting a link plays its curve with the same
nine-line interpolant; there is no discrete candidate on a link, so the curve
switch rests on and disabled there.

Three shapes of honesty carried over from the seams:

- The additive contract holds: the response gains `connecting_trajectories` (and the per-seam
  trajectories become the REWOUND ones — same poses to the micron, joint
  values chosen so every handover matches to the digit) and loses nothing.
  When joining fails, `connecting_trajectories_error` says why and the seam trajectories still
  stand.
- A link's two clearances stay two numbers — `env` is the torch in the
  plate, `self` is the forearm in the wrist, and they are different failures
  with different fixes (`docs/issues/0009`).
- A link marks weld-only travel mobility `not_applicable`; that omission alone
  no longer forces `REFUSED`. `PASS` still requires continuous collision,
  and exact emitted-Hermite joint-position, velocity, and acceleration checks.
  The drawer renders the native `PASS` / `FAIL` / `REFUSED` result.

The response keeps the Hermite contract arrays (`q`, `qdot`, and `t`) at
binary64 precision. Rounding any of the three after verification would define a
different curve; only labels in the drawer are presentation-rounded. These are
offline review verdicts. OLP export and runtime admission are separate
boundaries and do not acquire this gate merely because the drawer renders it.

## 14. Risks

**Stage 2 does not exist yet.** The UI is a viewer over a detector that has not
been built. Detection comes first, as a tested library, exactly like stage 1.

**O(n^2) face pairs.** `dog_box.step` is 42 faces, 861 pairs, trivial. A real
weldment at 10,000 faces is 50 million. The detector needs an AABB/BVH prefilter
designed in from the start — not because the fixture needs it, but because
retrofitting a spatial index after the API shape is fixed is painful.

**The GPU.** The handoff records a PCIe fault on this machine and advises running
one WebGL surface at a time until confident. This UI is a WebGL surface, and it
competes with the URDF viewer and the existing `cadquery/v1` UIs for the same
device. Currently 1 PCIe error per boot, down from 622,933 before the reseat, so
the reseat held.
