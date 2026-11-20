# The planning context

Status: IMPLEMENTED 2026-09-19/20 on `agent/robot-description-contract`,
local dev cell only. It follows `ROBOT-DESCRIPTION-CONTRACT.md`
and keeps its two identities and its refusal rules; it changes who derives
the robot from them, how many times, and what a program and a project hold.
The migration steps below are done in order; what each touched is recorded
under it.

## Vocabulary

| | |
|---|---|
| **motion server** | the software that manages a robot cell: `motion-server/joint-trajectory/v1`'s daemon on the cell |
| **real cell** | jetson + RevPi + motors |
| **robot description** | `robot_description/<model>/`: `robot.urdf` (kinematics of the base robot) and `config.json` (what the URDF cannot carry), plus spheres and meshes, identified by `robot_description_sha256` |
| **machine_planning_calibration.json** | one cell's deviation from its model, identified by `machine_planning_calibration_sha256` |
| **program** | one side of the planner's input: the welds and the placement of the part |
| **planning context** | the other side: the cell and the robot, COMPILED from description + config + machine_planning_calibration.json |
| **planner output** | the dense joint trajectory |
| **project** | program + the context's INPUTS (description, machine_planning_calibration.json) + planner output |

## Two principles

**One derivation.** "The corrected robot" is derived from the description and
the cell's deviation exactly once, by the OLP, into a planning context. The
planners and the seam worker read the context and never `machine_planning_calibration.json`,
`config.json` or a description directory. Today the derivation happens twice
(`weldplan.robot_description.apply_settings` onto the descriptor, then
`curobo_backend.urdf_with_cell_velocity` onto a copy of the URDF), and the
second finds its input by globbing `robot_description/` for a name
(`_find_urdf`, `profile.load`, `spheres.load`, `tool.require`). Its own
docstring names what the gap costs: "a smoother planning the uncorrected arm
would hand the verifier a path it never checked."

**The context is ephemeral.** It is compiled right before a plan request is
sent and discarded after. Nothing stores it, nothing hashes it, nothing pins
it. A project stores the inputs and recompiles. This is what keeps it from
becoming what robot/v4's generated URDF became -- a second source of truth
with a lineage of its own. If a context is ever saved, versioned or edited,
this paragraph has been violated.

## The planning context

Self-contained, about 2 MB (measured: `rosie_1400_v3` registers 15 files,
2.23 MB with 11 meshes; `rosie_1420_v1` 14 files, 0.91 MB):

```
identity.json   model_id, robot_description_sha256, machine_planning_calibration_sha256, machine_planning_calibration_source,
                machine_configuration_sha256 when a machine was asked (recorded, never fenced on), and the
                sha256 of the BASE URDF before the cell was applied
robot.urdf      the base URDF with machine_planning_calibration.json APPLIED: joint origins composed
                (T' = T_origin * Trans(xyz_m) * R(rpy_rad)), position limits
                narrowed, velocity the PLANNING rate (cuRobo parametrizes
                against <limit velocity>; the gap up to the cell's enforced
                rate is the daemon's follower authority and rides in the
                header, not the file)
planner.json    config.json merged with machine_planning_calibration.json: per joint the bounds,
                planning velocity, enforced velocity and acceleration; the
                speed ceiling; driven and held axes; reset pose; frames
                (base / tool / work, work_surface); torch
spheres.json    as registered
meshes/         as registered (cuRobo's asset_root_path)
```

A planner machine holds no `robot_description/`. Everything it needs arrived.

The OLP compiles it in Go. `internal/robotconfig` already reads `config.json`
whole (`Caps, Frames, PlanningJoints, Driven, Held, ResetPose, ToolFrame,
ElectrodeAxis`) and resolves `machine_planning_calibration.json` against the description (`Resolve`).
What it does not do yet: write the corrected URDF, merge the rates the way
`profile.for_cell` does (the PLANNING rate never above the cell's ENFORCED
rate, never above the ceiling; a held axis only ever a positioner), and pack.

The URDF is written by ATTRIBUTE SUBSTITUTION -- `origin xyz` / `rpy`,
`limit lower` / `upper` / `velocity` on the joints the cell touched -- with
every other byte preserved. Not an XML round trip: `encoding/xml` drops
comments and reformats, and the planner's parser should see the file the
description's author wrote. The context's bytes are not an identity, so they
need no cross-language reproducibility.

## What the program pins

`robot.v4.program.v2` today pins `cell: {id, sha256}` where the sha256 is of
the DESCRIPTOR bytes in the container -- a derived artifact, with machine_planning_calibration.json
already applied, so a recalibration re-pins every program. That predates the
contract (program.v2 is 09-03, the contract 09-13) and it is the wrong
dependency: the placement is a record in the WORK FRAME, which the URDF
defines; a machine_planning_calibration.json correction moves where that frame lands and the part
goes with it.

So the program pins the ROBOT and not the cell:

```jsonc
"robot": { "model_id": "rosie_1400_v3", "robot_description_sha256": "sha256:<64 hex>" }
"placement": { "cell_id": "rosie_1400_v3", ... }   // must name robot.model_id
```

`placement.cell_id` stays a name: the identity beside it in the same
document is what pins the description, and the validators (schema, Go, TS,
Python) refuse a placement naming another model. `machine_planning_calibration_sha256` is not in the
program. A machine_planning_calibration.json change re-plans; it never invalidates a program.

The all-zero identity is a producer that had no description to pin to; the
packer resolves it to the one it packs. A program pinned to THIS description
packs; one pinned to another is refused by the packer ("the description moved
under the program") until explicit acceptance in the OLP rewrites the pin.

**Strict, with an override.** `robot_description_sha256` covers `config.json`, spheres and
meshes as well as the URDF, so a velocity rating or a re-exported mesh
invalidates programs whose placement is untouched. Decided 2026-09-19: one
identity and exact match. The Workspace's **Accept current settings** action
(`adoptRobotDescription`) is the override -- a person decides the
placement survived, the hash cannot. A second, kinematics-only identity for
programs was considered and rejected: it reopens the canonical-projection
question and gives two answers to "is this the robot". Updated 2026-09-22:
acceptance is one explicit button after reviewing the available settings
differences and revision dates; no hash typing or copying is required. Hashes
remain in technical details and in the machine's exact identity checks.
Missing historical values are stated as unavailable, not inferred from a
different identity. Acceptance records the decision; it does not re-plan or
make an old plan playable.

## Flows

**Connected.** The OLP selects a machine from the cell registry and holds an
rt-core session on it. rt-core's Describe lists the description it was
compiled with (the manifest and every registered file, under
`robot_description/<model>/`) and its `machine_planning_calibration.json`, each as a
content-addressed resource; the OLP reads the machine's identity off that
listing alone (`internal/denseexec/rt_core_robot.go`) and fetches bytes only
when it needs them -- the `machine_planning_calibration.json` at every pack, the description files
when its own copy is another revision. It recomputes the manifest identity
over what arrived and refuses it unless it equals the advertised
`robot_description_sha256` -- the machine is checked, not trusted. It then compiles the
context, sends context + program, and discards the context. The result
carries both hashes in its header; Load compares them with the machine's
before anything is acquired.

Connected planning also reads the effective native position, velocity and
acceleration limits from a fresh Describe response. The compiler intersects
these with the model and calibration before writing both `planner.json` and
`robot.urdf`; a bench's narrower limits must constrain the search, not merely
reject the finished upload. Every driven joint needs a valid native entry.
Virtual held joints may be absent from a six-axis machine. If a selected
machine cannot provide its limits, planning refuses until it reconnects or
the user deliberately disconnects to plan offline.

`execution_limits_sha256` in the context's identity and planner document binds
that limit snapshot to the planner cache key. It does not replace or change
the robot/calibration identities, and it is not a persisted second robot
definition. Two machines with the same description and calibration but
different native limits cannot reuse one another's loaded planning context.
Native admission remains authoritative at Load, including if the machine
changed since planning.

**Offline.** The person picks a model from the robots this OLP has
(`GET /robots`: the workstation's descriptions and the ones fetched from
cells, by identity) and may load a `machine_planning_calibration.json` from a file (Robot menu, only
while no cell is connected). The bytes travel in the weld-plan request
(`machine_planning_calibration_base64`) and are read ONLY when no cell can be asked; the OLP
validates them against the description with the same `Resolve`, compiles,
plans. The header says `machine_planning_calibration_source: "target"` for a file: the gate is the
hash, not the label, and the daemon refuses any value but `target` / `empty`
(`dense_joint_trajectory_format.hpp:875`). The project records
`machinePlanningCalibrationSource: "upload"` for display, so the UI does not have to lie. Execution
needs a cell; offline ends at saving the project and its plans.

**Reopening a project connected.** The UI reads the cell's identity off its
status and, while connected, treats the CELL's description as the current
one (`domain/robots.ts` `setConnectedRobot`): drift and explicit acceptance are
against what the plan will actually be made against, not the workstation's
copy. Equal: the saved plans play as they are (the daemon would accept them
anyway; this tells the person before upload rather than after).
`machine_planning_calibration_sha256` differs: the plan-versus-target notice says so; pull again,
re-plan; the program lives. `robot_description_sha256` differs: the program is stale;
review the available changes, accept the current settings if the placement
survived, then re-plan.

**What the project stores.** Not the context. The program (with its robot
pin), the cell settings it pulled or loaded -- the `machine_planning_calibration.json` BYTES among
them, so a plan can be made offline against the same deviation -- and the
planner's output. The description is not in the project: it is
content-addressed and lives in the OLP's store or its fetched cache; a cell
serves it again on demand.

## What moves, what dies

| Today | Becomes |
|---|---|
| `weldplan/robot_description.py` `apply_settings`, `compose_origin` | deleted; the OLP composed |
| `curobo_backend.py` `urdf_with_cell_velocity`, `_find_urdf` | deleted; the context's URDF |
| `robot_cell/robot/profile.py` `load` + `for_cell` (disk, by id) | reads `planner.json` |
| `robot_cell/collision/spheres.py` `load(cell_id)` (disk) | reads the context's `spheres.json` |
| `robot_cell/robot/tool.py` `require(cell_id)` (disk) | reads `planner.json` |
| `seam_worker/robot/cell.py` `read_urdf` | moves to `weld_motion_planner`; parses the context's URDF into the descriptor. One parse, of one file, that the OLP compiled |
| `seam_worker` `build_plan_request` | packs; holds no robot. `seam_worker` is then the OLP's CAD and packing arm, which is what it does |
| container member `machine_planning_calibration.json` (descriptor) | replaced by the context |
| `program.cell` asset pin, `placement.cell_id` | `program.robot`, `placement.robot_description_sha256` |
| project `CellSettings` (identities only, no bytes) | the inputs: description files, `machine_planning_calibration.json` bytes, and whether they came from a cell or an upload |

## Migration (done)

Each step left the tree working and was useful alone.

1. **Context.** `internal/robotconfig/context.go` (`Compile`: attribute
   substitution, the `for_cell` merge, packing); the seam worker packs it
   under `context/` (`weldplan.plan_request`, members digest-verified like
   every other); `weld_motion_planner.robot_cell.context` reads it, found by
   the descriptor's identity rather than threaded through forty signatures.
   `offline-programming compile-context` exposes the one compiler to the
   weld planner's fixture tools (`tools/olp_context.py`), which is how the
   corpus carries the same context the OLP packs.
2. **The second derivation is gone.** `apply_settings`, `compose_origin`,
   `urdf_with_cell_velocity`, `_find_urdf`, the disk reads in `profile`,
   `spheres`, `tool` and the mesh roots. `read_urdf` is `weldplan.urdf`; the
   packer parses the descriptor off the context's own URDF
   (`descriptor_of`) and is handed no URDF, settings or frames of its own.
   The descriptor member STAYS in the container -- parsed from the same
   corrected URDF, so it cannot disagree -- because every stage walks it. A
   request with no context is refused on the planner side, in words.
3. **The cell serves its description.** rt-core lists it in Describe and
   serves each file under `GET /v1/resources/<sha256>`. The OLP asks the
   target once per pack: local copy at the advertised identity -> use it;
   else the fetched cache
   (`<user cache>/rosieos-olp/robot-descriptions/<hex>/<model>/`); else
   fetch, recompute the identity over what arrived, refuse a mismatch, keep
   by identity. A local copy the cell does not hold is never planned against
   silently; a machine that cannot serve its files makes the pack refuse
   naming both revisions rather than plan what Load will refuse.
4. **The program pins the robot.** `program.robot`, in the schema, the Go
   validator, the TS contract and `weldplan.program_v2`; the UI writes it
   from `robot.model` and its description pin, so there is no second copy
   of which robot to fall behind (the `programV2.cell` reconcile is gone).
5. **Project and offline.** `CellSettings.machinePlanningCalibrationBase64` / `machinePlanningCalibrationSource`;
   the Robot menu's "Load machine_planning_calibration.json…"; `machine_planning_calibration_base64` in the weld-plan
   body, read only without a cell; `/robots` lists fetched descriptions;
   drift and adopt read the connected cell's identity.

## On the standalone rt-core base

The joint daemon is being retired and rt-core serves the robot itself, so
the cell's half of this document moved there (branch
`agent/planning-context-rt-core`, on `codex/standalone-rt-core`):

- **One Go implementation of the contract**: `rosieos/robotdesc`, the module
  at `robot_description/go` beside the data it reads (identity, URDF scan,
  description store, the machine_planning_calibration.json rules), imported by rtctl and by the OLP
  (`internal/robotconfig` re-exports it). The daemon's C++ twin went with
  the daemon.
- **The robot is one directory, and rt-core's definition of it is in there.**
  `robot_description/robots/<id>/rtcore_definition.json` (gearing, encoders,
  drive profiles, joint mapping; decided 2026-09-20) is a registered file, so
  `robot_description_sha256` covers it; rt-core's separate `config/robots/<id>.json` and
  its archived URDF copies are gone. A machine config pins the directory
  (`robot.robot_description.path`) and its identity (`robot.robot_description.sha256`). `rtctl compile`
  recomputes the identity, reads the definition and the description's own
  `robot.urdf` from inside it, refuses a pin that does not match, and
  snapshots the manifest and every registered file into the compiled
  resources by description-relative path. The adapter admits the set only
  when the identity over the served files equals the pin, and serves it under
  `GET /v1/resources/<sha256>`, listed in Describe's `robot.resources`. The
  URDF's `package://<id>/...` meshes must be registered files. A runtime
  package carries every description so a cell compiles without a checkout.
- **The local dev cell can bind the robot** (`simulation-rosie1400.json`,
  selected with `DEV_STACK_MACHINE_CONFIG`): verified 2026-09-20 that the
  simulated cell serves the description and the OLP reads it back. It is
  not the default because the simulated bus cannot Home the A6-EC profile
  the definition selects, so the robot-bound cell neither arms nor jogs;
  the flat `simulation-program.json` stays the default until that is
  solved (docs/ROBOT-DESCRIPTION-CONTRACT.md, "Local dev cell").
- **Describe says which robot, in the contract's words** (application contract
  version 2): `robot: {model_id, robot_description_sha256, machine_planning_calibration_sha256, resources}`. The
  OLP recomputes `robot_description_sha256` over the listing and refuses a machine whose
  listing does not hash to what it advertises.
- **The machine pins its machine_planning_calibration.json.** A machine config's `robot` block may
  carry `machine_planning_calibration: {path, sha256, source}`; rtctl verifies the bytes, judges
  them by the contract's bounded-deviation rules against the pinned
  description (`robotdesc.Resolve`), and serves them as the resource
  `machine_planning_calibration.json`; a machine that pins none serves the model's empty document.
  The pin is part of the machine config, so `configuration_sha256` covers it:
  `machine_planning_calibration_sha256` is the served bytes' identity, as before.
- **Identity is unchanged in kind**: `robot_description_sha256` is still the manifest
  identity (over one more registered file), pinned by the machine and
  advertised in Describe; `machine_planning_calibration_sha256` is still the machine_planning_calibration.json bytes.

- **The OLP asks rt-core.** `RTCoreController` is the `CellTarget`
  (`internal/denseexec/rt_core_robot.go`): `Cell` is the identity read off
  Describe's resource listing (model = the definition id, `robot_description_sha256` =
  the manifest hash over the listed description entries, `machine_planning_calibration_sha256` = the
  `machine_planning_calibration.json` resource's digest) plus the `machine_planning_calibration.json` bytes, `Description`
  is every listed description file fetched by digest. Reads are
  content-addressed and cached for the controller's life. `Load` judges the
  plan's `robot_cell` block against the machine before acquiring it, in the
  contract's words; a machine that binds no robot description (the flat-axis
  simulation cells today) has nothing to judge by and passes the block, as a
  daemon from before the contract did, and its status says so
  (`robot_cell.configured: false`). The picker forwards both calls to the
  selected machine. The joint daemon's `kind: "cell"` / `kind:
  "description"` client paths are gone.

- **The UI, on codex's main.ts.** Its model select and Cells menu stay;
  the program pins the description when it is created, opened, imported or
  moved to another robot, the Workspace's robot card states the pin, the
  drift (against the connected machine's description while one is
  connected, the server's otherwise) and explicit acceptance; Pull cell
  settings fetches the machine's machine_planning_calibration.json bytes; "Load machine_planning_calibration.json…" stands a
  file in while no machine is connected.

**What deploying this to a real cell costs.** The machine and cell configs
of every robot-bound cell changed in the repository (the robot pin, and so
`configuration_sha256` and `deployment_sha256`), so each cell needs the new
package, a recompile and a restart. It does **not** need a new Home: a saved
Home anchor is bound to the pair, the drive's identity and the per-axis
`coordinate_identity_sha256` (gear ratio, encoder counts, sign, tracking mode,
Home policy -- `rt-core/adapters/rosie/control/anchors`), none of which
moved. The J6 limit edit, the file renames and the definition's new home are
not coordinate changes.

## Known costs

- A description edit of any kind invalidates every program for that model
  (strict, above). Explicit acceptance applies to one program and robot version.
- The seam worker's request ceiling was 12 MB (`DefaultMaxRequestBytes`); a
  full STEP (8 MB) plus a 2.2 MB context, base64-encoded, exceeded it. It is
  32 MB now, with the measurement beside it.
- The corpus fixtures (`weld_planner/v1/data/motion/*.weldplan`) carry a
  context and went from ~35 KB to ~360 KB each; `tools/refresh_corpus_cells.py`
  rewrites them from the compiler whenever the description changes.
- A machine_planning_calibration.json that NARROWS a limit leaves the program valid and may make a
  plan fail. That is a planning failure, not a stale program, and the UI must
  say which.
- Spheres carry `provenance` hashes of the base URDF and meshes. Nothing
  verifies them today (`spheres.py:99` stores them). If verification is ever
  added it must compare against the BASE URDF's hash, which `identity.json`
  can carry, not the corrected one.
