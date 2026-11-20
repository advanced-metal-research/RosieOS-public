# Export to Offline Programming — the rail-to-row compile

> **Historical.** The compile this describes, `weld_programmer/export/robot_v4.py`
> and the `/api/export/olp*` routes that drove it, were deleted in D24: rails
> stops emitting `robot.v4.program.v1` rows. Kept because two other documents
> cite it for how something was found, not as a guide to code that exists.
> `tools/refresh_corpus_cells.py` and issue 0014 are the citations.


How a planned motion result becomes a `robot.v4.program.v1` program in the
Offline Programming V1 catalog. This is a **one-way compile**: rails and the
planner's native result in, spreadsheet rows out. Nothing here reads rows back,
and nothing downstream of the row boundary is changed — the exported program is
an ordinary catalog program that OLP edits, simulates and plans exactly as it
does a hand-authored one.

Verified against `offline-programming/v1` as of 2026-08-22:
`ui/src/contracts/robot-v4-program.ts` (the validator this output must pass),
`ui/src/adapters/robot-v4.ts` (field semantics), `internal/catalog/client.go`
(identity rules), `server.go` (the save envelope), and
`motion-server/v1/src/robot_v4_cartesian_so3.hpp` (rotation convention).

## Inputs

Three documents, all of which the authoring UI already holds after a plan:

1. **The plan request** (`.weldplan`) — the weld program (rails), the cell,
   and the placement. The compile needs it for process parameters, seam
   identity, and FK.
2. **The native result** (`amr-weld-planner-v1.motion-plan-result.v1`) — the
   planner's answer for that exact request. The compile trusts it only if
   `result.request.request_sha256` equals the SHA-256 of the request bytes it
   was handed; a result paired with a different request is refused, not
   patched over.
3. **Program identity** — `program_id` (must match `^[A-Za-z0-9._-]{1,128}$`,
   the catalog's `identityPattern`) and a human `program_name`.

## Native verdict admission

Refined export requires each seam's continuous trajectory to carry a native
`PASS` with zero collision/limit findings and no unverifiable checks, plus a
passing tracking certificate within its declared tolerance. Every supplied
approach, transit, and retract requires the same native movement `PASS`.
Missing, malformed, contradictory, or failing reports and join errors refuse
the entire conversion before rows are generated or the catalog is changed.
Dense conversion applies the same gate before resampling.

The unrefined rails **Compile to rows** action explicitly selects
`candidate_only=True` for M4 authoring. That mode accepts no continuous
trajectory or connecting move; it cannot bypass an M5/M6 failure. The default
compiler, direct Export to OLP, replan, and refined rows route require native
verification. The provenance extension records `source_admission` as
`candidate_only` or `native_pass`; programs remain `saved_unplanned` and in
simulation mode. Raw results remain available for download and review.

This consumes the planner's reports; it does not authenticate them, prove full
request coverage or connection topology, or certify the rows/dense samples
after conversion. Those representations still require their downstream judges.

## Output

The catalog save envelope, ready for
`POST /api/offline-programming/v1/programs`:

```json
{
  "schema": "offline-programming.program-save.v1",
  "catalog_revision": <current catalog revision, caller-supplied>,
  "program": { ... robot.v4.program.v1 ... }
}
```

The Go decoder runs with `DisallowUnknownFields`, so the envelope carries
exactly these three keys and nothing else.

## Program header

| field | value | why |
|---|---|---|
| `schema_version` | `1` | the contract |
| `program_id`, `program_name` | caller-supplied | identity rules above |
| `robot_model` | the request's cell id | validator allows `rosie_1400_v3`, `rosie_1400_h`, `rosie_1420_v1`; any other cell refuses to export |
| `robotic_cell_type` | same as `robot_model` | OLP's own adapter falls back to the model |
| `program_state` | `"saved_unplanned"` | honest: Motion Server has not planned it |
| `execution_mode` | `"simulation"` | export never asks for hardware |
| `program_revision` | `1`, `base_program_revision` `0` | the create-path invariant `server.go` enforces |
| `units` | `{length: "m", angle: "rad", time: "s", joint_order: [J1..J9]}` | byte-for-byte the canonical order |
| `program_digest` | see **Digests** | |

The planner's provenance goes in `program.extensions.amr_weld_planner`:
`request_sha256`, `source_revision` (the full commit the launcher pinned, or
absent with `source_revision_state: "unavailable"`), and the result's
`coverage` block verbatim — the claim limits travel with the program.

## Joint mapping

The 9-vector `target_joints_rad` is J1–J9 in canonical order:

- **J1–J6** — the arm, from the seam's `axis_names` columns in the result
  (wire values are degrees; rows are radians).
- **J7** — positioner side A axis (`positioner_table_a` in Motion Server's
  work-frame policy), **J8** — side B. The result's per-seam `held` map and
  candidate columns name the positioner axes; the compile maps by name, never
  by position.
- **J9** — the station selector, which our planner already holds by name.

Axes the cell does not have are written as `0.0`. `positioner_targets_rad` is
the same 9-vector with the arm entries zeroed — OLP's adapter reads positioner
values from indices 6–8.

Values may legitimately exceed ±180° on joints whose URDF bounds are wider
than a turn (the planner winds rather than unwinds when the bound allows);
they are passed through unchanged, because they are within the limits the
downstream planner checks against.

## TCP pose

`target_tcp_m` is `[x, y, z, roll, pitch, yaw]` in the world frame, radians,
with `R = Rz(yaw) · Ry(pitch) · Rx(roll)` — the same fixed-axis convention on
both sides (`kinematics.rpy_matrix` here, `so3_rpy_rotation_matrix` there).
The two implementations pin different angles at the pitch = ±90° degeneracy
(ours zeroes roll, theirs zeroes yaw); both reconstruct the same rotation, so
the pose is unaffected — only the decomposition differs, and only at exact
gimbal lock.

The pose is computed by our FK (`weld_programmer.robot.kinematics`) at the
row's `target_joints_rad`, through the placement the plan was made with, to
the electrode tip (`tool0`). `tool_frame` is `"tool0"` — the frame the pose
is actually expressed at — with `work_frame` `"world"`, `fixture_side`
`"in_cell"`.

**Verification gate (0.4) — passed, 2026-08-22.** Motion Server's FK
"terminates at the URDF tool0/end_effector transform"
(`plan_services.hpp:615`), and both sides read the same
`robot_description/robots/rosie_1400_v3/robot.urdf`. Measured on the corpus: the simulator's
own sampled `tcp_pose` at our `target_joints_rad` matches our `target_tcp_m`
to four decimals on every row. The frames agree; nothing is translated.

## Row expansion

One seam becomes one row group, in the order the joined program (M6) walks
them. Every generated row carries `source_state: "cad_derived"`, `seam_id`,
`pass_id` (`"root"` until multi-pass exists), `source_project_id`, and
`enabled: true`.

```
joint_ptp rows     the approach/transit move: ONE ROW PER KNOT
comment row        seam header: id, length, waypoint count, candidate rank
joint_ptp rows     the weld: one row per trajectory knot, weld_enable true,
                   arc_state "on"; the LAST weld row is fine_stop true
joint_ptp rows     the retract move, one row per knot
```

- **Every motion row is a joint target.** This was measured, not assumed:
  compiled as `line_tcp`, the spreadsheet planner re-IKs each pose with its
  own branch choice, left our solution at the first seam (J5 off by 3.4 rad)
  and drove the tool through the positioner. Compiled as joint targets, its
  trajectory replays ours with drift under 0.01 rad on every row. The rows
  exist to replay the plan this planner certified; the spin and branch
  decisions ARE the plan.
- **Connecting moves emit every knot, not just the arrival.** The straight
  joint interpolation between a weld's end and the next seam's start is
  exactly the path the free-space planner bent around the positioner —
  measured, the straight line grazes it by 3.7 mm. The knots are the
  certified detour.
- **Weld sampling** — the verified M5 trajectory's knots (the knots ARE
  the planner-chosen sampling; resampling them would re-introduce the
  R3 violation the schema exists to prevent), else the winning candidate's
  `joints_deg` samples only in explicit candidate-only authoring. The
  wires arrive at the certificate density (10 ms) and every knot is a
  row: the OLP's collision coverage judges the rows, so they carry what
  the planner certified. Measured 2026-09-07 on the cell's 0.09 rad/s
  limits, a two-seam bracket program is 8747 rows; the program row cap
  is 65536 for that reason (it was 4096 when knots were 0.2 s apart).
- **Rewind into bounds.** The result's wire values are UNWOUND — continuous
  across the ±180° wrap. A row's value must sit inside the joint's URDF
  bounds, so the compile picks the representative `v + 2πk` inside them
  nearest the previous row (so interpolation never takes the long way). A
  value no turn brings inside is passed through for their checker to flag.
- **Pacing.** Each row's `joint_velocity_rad_s` is the segment's largest
  joint step over its planned duration (the result's `times_s`), clamped to
  the spreadsheet planner's own [0.01, 1.5] rad/s. Each row ALSO carries
  `playback_step_ms` for the segment departing it — that field is the
  authoritative row clock on their side, and a program without it plays at
  a flat 10 s per segment (measured: 9,130 s for a 192 s plan). Weld rows
  also carry the weld's travel speed in `tcp_velocity_m_s`.
- **Arc state** lives on the motion rows (`weld_enable` + `arc_state`),
  matching OLP's adapter; there is no separate arc command row.
- **Process parameters** — `voltage_v`, `amperage_a`, `wire_feed_m_per_min`,
  `weld_process` from the weld's parameters in the request program. Absent
  parameters stay absent; the compile invents no numbers. A varying profile
  (non-constant scalar function) is refused, not averaged.
- **Row caps** — `accel_m_s2`, `joint_accel_rad_s2` carry OLP's own
  authoring defaults (`project.ts`); `blend_radius_m` 0; `fine_stop` false
  except a weld's last row.

In candidate-only authoring, a seam the planner could not cross (`crossed:
false` / an `error`) exports as its comment row, with the error in the text.
Refined export refuses that result instead of creating a partial motion plan.

## Digests

`row_hash` and `program_digest` replicate OLP's own functions exactly
(`computeRowDigest`, `computeProgramDigest`):

1. strip the transient fields (the two `strip*Transient` lists, copied, with
   a test that fails if OLP's lists change),
2. sort keys recursively (`sortCanonical` — arrays keep order, objects sort),
3. serialize as ECMAScript `JSON.stringify` would — **integral floats print
   without a decimal point** (`5`, not `5.0`); this is the one place Python's
   default float formatting differs and the serializer accounts for it,
4. SHA-256, prefixed `sha256:`.

Parity is provable: the cross-validation harness (below) recomputes both
digests with OLP's own TypeScript and fails on any byte difference.

## Cross-validation

The definition of "valid" is **their** validator, not a re-description of it:
`tools/olp_validate.mjs` loads `assertRobotV4Program` and the digest functions
from `offline-programming/v1/ui/src/contracts/robot-v4-program.ts` and runs
them against the compile's output. The pytest suite shells to it when node and
the OLP tree are present, and skips loudly when they are not.

**The 0.4 gate — passed, 2026-08-22.** All three corpus projects
(`bracket_a_2x`, `bracket_b_2x`, `hitch_2x`), planned fresh and compiled,
pass OLP's validator with byte-exact digest parity (925 / 1,671 / 1,533
rows), and the local Tesseract simulator plans each whole: collision, limits
and singularity all **clear**, `validation_state: valid`.

One condition, found the hard way: the committed corpus `.weldplan` bundles
carry a cell captured before `26e61b0` gave the arm its real URDF bounds
(J4 ±6.3 in the bundle, ±π in the URDF; J5 −6.3..6.3 against −3.37..1.3), so
a plan made from the committed bundles emits joints today's cell refuses.
The gate runs against requests re-bundled from the current URDF; the
committed bundles should be refreshed the same way. This is the
cell-identity mismatch stage 3's digest gate exists to catch, met early.

## The sidecar, and regeneration

The exported program is replannable because the request travels WITH it:
`extensions.amr_weld_planner.request_weldplan_b64` is the exact `.weldplan`
(rails, placement, cell, STEP), refused unless it hashes to the provenance's
`request_sha256`. Beside it, `exported_row_hashes` records what this export
wrote, row by row — the honest boundary between generated rows and human
work.

`POST /api/export/olp/replan` (`{program_id}`) reads the sidecar back, plans
it fresh on the motion server, and merges over the catalog copy under one
rule: **a row whose hash still matches the map is the export's to replace;
anything else is somebody's work and is kept.** Conflicts resolve at seam
granularity — one hand-edited row keeps its whole seam, because splicing
fresh rows around an edit would silently change the motion beside it — and
hand-added rows outside any seam are appended and named in the report. Kept
rows stay out of the new map, so a conflict never silently expires. A
re-export of the same program id merges the same way; a program with no map
(exported before sidecars, or authored elsewhere) is refused with 409 rather
than clobbered, `force: true` overriding — and saying so in the response.

## Transport

`POST /api/export/olp` on the authoring server
(`server/export_olp.py`) takes `{request_b64, result}` — the exact
`.weldplan` bytes that were planned and the result they produced — compiles,
and saves to OLP's catalog at `WELD_PLANNER_OLP_ORIGIN` (default
`http://127.0.0.1:8794`). A program id the catalog already holds is revised
on top of its current revision rather than duplicated; a fresh id is created
at revision 1. The plan drawer's **Export to OLP** button is the UI for it,
live once a plan is on screen.

The corpus bundles themselves still carry pre-`26e61b0` cells —
`tools/refresh_corpus_cells.py` rewrites them against the current URDF, and
`docs/issues/0014` records why that rewrite is not yet committed (14 motion
tests pin the old arm's IK branches).

## The dense joint trajectory beside the compile

The joint-space daemon (`motion-server/joint-trajectory/v1`) plays
`robot.v4.dense-joint-trajectory.v1`, a 100 Hz record of every joint. The
motion server emits it itself, in the same request as the plan: `POST
/api/motion/plan?dense=true&program_id=…&manifest_revision=…&plan_revision=…`
(`weld_motion_planner/io/dense_output.py`, built by
`weldplan/dense_joint_trajectory.py`, the exporter the daemon's tests share)
files the blob by content digest under `--dense-store-dir` (default
`~/.cache/rosieos-olp/weld-planner-dense`, or `WELD_PLANNER_DENSE_STORE_DIR`)
and adds a `dense` summary to the result: the identity the daemon fences on
(`plan_id = <program_id>:<request sha256[:12]>`, `program_digest =
sha256:<request sha256>`, the two revisions), sampling totals, and one entry
per segment with its `source_id`. `GET /api/motion/dense/{digest}` returns
the bytes. A result the daemon could not play (no refine stages, a boundary
at speed) yields `dense_error: {reason, detail, segment_index}` in the
daemon's vocabulary instead, and the plan still succeeds.

The rails compile (`POST /api/export/olp/rows` with `dense: true` and
`dense_identity`) only forwards the ask with `refine` and passes `dense` /
`dense_error` through; it builds nothing. The OLP server loads the blob by
digest from the motion server when the operator presses **Load to robot**.

Weld passes end at travel speed (`docs/issues/0013`); the exporter time-warps
the first and last `weld_ramp_s` (0.25 s, `data/dense_joint_trajectory.params.json`)
so the daemon's boundary rule holds. That ramp goes when M5 consumes an
authored run-in and run-out.

## Non-goals

No row is read back. No OLP schema field is added or changed. No accepted
plan is produced (that is stage 3). No execution semantics are implied —
`execution_mode` is `"simulation"` and the authority chain is untouched.
