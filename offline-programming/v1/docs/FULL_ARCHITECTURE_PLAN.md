# Offline Programming and Robot V4 — Full Architecture Plan

Status: implemented software architecture baseline; physical deployment still
requires the separately reviewed Robot V4 commissioning gate
Scope: Offline Programming V1, Robot V4 Motion Server, Robot V4 SteamDeck,
RTCore, EtherCAT, CadQuery, and local simulation

## 1. Outcome

Offline Programming becomes the main desktop application for creating,
reviewing, simulating, and managing Robot V4 programs.

The program is presented as an editable spreadsheet because rows provide a
clear representation of motion, weld, loop, comment, positioner, and I/O
instructions. The same logical program can be opened, created, and edited on
the Robot V4 SteamDeck teach pendant. Offline Programming does not reproduce
SteamDeck buttons, joystick controls, chords, or its hardware-oriented user
interface.

Path Planner is superseded. Offline Programming may reuse lessons and useful
implementation ideas from it, but Path Planner is not a runtime dependency
and its import/export format is not a compatibility requirement.

The target Robot V4 Motion Server is the final authority for any trajectory
that will run on the physical robot. The physical execution path is:

```text
Offline Programming or SteamDeck
              |
              v
      shared editable program
              |
              v
   target Robot V4 Motion Server
      plans and accepts motion
              |
              v
      immutable accepted plan
              |
              v
   Motion Server preload + play
              |
              v
            RTCore
              |
              v
           EtherCAT
              |
              v
             robot
```

The browser never creates the final physical trajectory and never sends
joint control frames directly to RTCore or EtherCAT.

## 2. Non-negotiable architecture decisions

1. There is one shared, versioned Robot V4 program contract used by Offline
   Programming, SteamDeck, and Motion Server.
2. The editable program and its planned trajectory are different artifacts.
3. Every program edit invalidates its previously accepted plan.
4. The Motion Server owns the robot program catalog and its revision history.
5. The selected physical robot plans the final executable trajectory using
   its deployed planner, robot model, cell configuration, calibration, tools,
   fixtures, limits, and collision world.
6. Motion Server executes only the exact accepted plan identity that the
   operator reviewed.
7. Local planning uses the same Robot V4 Motion Server planner source and
   asset format, but remains simulation-only.
8. Browser preview is visibly approximate and can never be promoted to an
   executable plan.
9. SteamDeck and Offline Programming are peer editors of the same program,
   not import/export islands.
10. Path Planner compatibility and virtual SteamDeck controls are out of
    scope.

## 3. The three persistent artifacts

> A fourth, `robot.v4.program.v2`, is defined in §4 as the successor of the
> Robot V4 program. Since 2026-09-03 the offline project persists its nodes
> (`project.nodes`, with the v2 header in `project.programV2`) as the
> authored program, and the Robot V4 rows in §3.2 are compiled from them.
> This section's text otherwise predates that change.

Keeping these artifacts separate prevents desktop-only authoring data from
leaking into the robot contract and prevents an editable row from silently
changing an already-reviewed trajectory.

### 3.1 Offline project

Schema: `offline-programming.project.v1`

Owned by Offline Programming. It contains:

- imported STEP files and CAD references;
- extracted topology and selected seams;
- seam direction, trimming, ordering, passes, and fixture placement;
- workspace/view state that is useful to authoring;
- the Robot V4 program being authored;
- source and derivation records connecting rows to seams and passes;
- project revision history; and
- cached plan references and evidence for display.

This is the rich workstation document. SteamDeck does not need to understand
CAD topology or desktop view state.

### 3.2 Robot V4 program

Schema: `robot.v4.program.v1`

Owned canonically by the Motion Server program catalog and editable by both
Offline Programming and SteamDeck. It contains:

- stable program ID, name, part metadata, and program revision;
- robot model, cell configuration, execution mode, and settings policy;
- stable row IDs and row order;
- motion, loop, comment, capture, calibration, weld, positioner, and I/O
  commands;
- tool/work frames and target-frame semantics;
- TCP and joint targets using canonical units and joint order;
- speed, acceleration, blending, fine-stop, welding, and process fields;
- tool-axis and harmonious external-axis constraints;
- optional source provenance such as project, seam, and pass IDs;
- edit source and modification state; and
- the base catalog revision used for optimistic concurrency.

This contract is based on the existing native SteamDeck
`SpreadsheetProgramState` and `SpreadsheetProgramRow`; it must not be a
smaller browser-only approximation.

### 3.3 Accepted plan

Schema: `robot.v4.accepted-plan.v1`

Created by a Robot V4 Motion Server and immutable. It contains:

- program ID, program revision, and canonical program digest;
- accepted plan ID, plan revision, and trajectory digest;
- ordered trajectory samples and timing;
- normalized collision, singularity, joint-limit, and runtime evidence;
- planner backend and trajectory method;
- planner build digest;
- robot model and canonical J1–J9 order;
- target/cell identity and manifest revision;
- cell configuration, calibration, URDF, mesh, and collision-world digests;
- tool/work-frame and settings-policy revisions;
- request/correlation ID and creation time; and
- an explicit `simulation_only` or `physical_target` authority classification.

The plan cannot be edited. A row edit creates a new program revision and
requires a new plan.

## 4. Shared program contract

Create authoritative JSON Schemas under a shared Robot V4 contract directory,
for example:

```text
robot/v4/contracts/programs/
  robot.v4.program.v1.schema.json
  robot.v4.program.v2.schema.json
  robot.v4.program-catalog.v1.schema.json
  robot.v4.accepted-plan.v1.schema.json
  fixtures/
```

`robot.v4.program.v2` (added 2026-09-03) is the pre-compile weld program
shared by the classic flow and rails authoring, and the successor of
`robot.v4.program.v1`: one row per node rather than one row per waypoint,
intent only, closed schema, validated by the same Go/TypeScript/Python
fixture discipline. It sits upstream of the v1 rows, which remain the input
to the Motion Server and the Steam Deck until those move; no producer or
consumer is wired yet. The decision record is D23 in
`weld_planner/v1/docs/DECISIONS.md`.

TypeScript and C++ may use native types, but both implementations must be
verified against the same schemas and golden fixtures. No client may maintain
an undocumented reduced copy of the contract.

The existing catalog envelope remains the starting point:

```text
schema = robot.v4.program_catalog.v1
owner  = motion-server
```

The contract work must reconcile every field currently supported by
SteamDeck, including:

- program and part metadata;
- defaults and settings policy;
- program state and execution mode;
- motion, planner, validation, collision, and limit states;
- planned-path revision and sample count;
- playback metadata;
- edit revision and Motion Server base revision;
- all spreadsheet row types and loop fields;
- nominal versus calibrated targets;
- capture and calibration fields;
- positioner, weld/process, and RevPi I/O fields; and
- row plan/sample/hash metadata.

Canonical representation rules:

- joint arrays are ordered J1 through J9 and expressed in radians;
- Cartesian positions are expressed in metres;
- orientations use one explicitly named representation per field;
- IDs are stable opaque strings, not row indexes;
- serialized numbers have a defined finite-number policy;
- object keys, row order, and numeric normalization are defined for hashing;
- hashes exclude transient UI, timestamps, and cache-only evidence; and
- unknown future fields are either preserved or rejected according to an
  explicit schema-version policy.

## 5. Program catalog and multi-editor behaviour

The Motion Server program catalog is the shared source of truth. It provides:

- list and get;
- create;
- update against an explicit base revision;
- immutable revision history;
- delete/archive policy;
- catalog and program schema versions;
- audit metadata; and
- conflict responses containing the current revision.

An update must use optimistic concurrency, such as `If-Match` or
`base_revision`. If Offline Programming and SteamDeck edit the same revision,
the second save receives a conflict rather than overwriting the first.

Required editing behaviour:

- A program created on SteamDeck opens in Offline Programming as a
  spreadsheet-first project with no invented CAD provenance.
- A CAD-derived Offline Programming program opens and remains editable on
  SteamDeck.
- A SteamDeck edit to a CAD-derived row marks that row as teach-modified or
  manually overridden.
- Offline Programming must not silently regenerate a teach-modified row from
  CAD. The operator explicitly keeps the taught override or regenerates it.
- Saving an edit increments the program revision, recalculates its digest,
  clears executable-plan readiness, and marks planning state as stale.
- Stable program and row IDs survive all round trips.

The current SteamDeck catalog parser and writer become conformance
implementations of this shared contract rather than the only specification.

## 6. Planning authority

There are three deliberately different planning levels.

### 6.1 Browser preview

Purpose: instant authoring feedback.

- May interpolate a simple orange path for visualization.
- Must be labelled `PREVIEW · NO MOTION OUTPUT`.
- Is never called canonical, accepted, validated, or executable.
- Cannot satisfy planning, collision, or execution-review evidence.

### 6.2 Local canonical simulation

Purpose: high-fidelity offline validation without robot hardware.

- Invokes a locally built instance of the production Robot V4 Motion Server
  spreadsheet/Tesseract planner, warmed once when the application starts and
  retained for the local simulator session.
- Uses the same planner source, request contract, robot asset format, joint
  order, settings logic, and collision implementation as Robot V4.
- Starts only through explicit local-simulator opt-in.
- Does not start NATS, UDP motion output, RTCore, EtherCAT, RevPi, or other
  physical services.
- Produces an accepted-plan-shaped artifact marked `simulation_only`.
- Fails closed when Tesseract, the correct binary, assets, or containment
  evidence are unavailable.

The Offline Programming backend invokes the Robot V4 Motion Server
`spreadsheet-tesseract-plan-server` persistent entrypoint. The process and its
loopback-only Tesseract/Bullet collision worker are bounded to the application
lifecycle; unchanged exact waypoint checks are cached inside that immutable
collision world. The default binary and probe fixtures are rooted under
`motion-server/v1`.

### 6.3 Physical target planning

Purpose: create the only plan eligible for physical execution.

- Offline Programming submits a specific saved program revision to the
  selected robot through its server-owned target adapter.
- The deployed Robot V4 Motion Server plans it using that robot's actual
  manifest, URDF, calibration, collision assets, tools, limits, settings, and
  planner build.
- The Motion Server stores the accepted plan and returns its immutable
  identity and full review evidence.
- Offline Programming visualizes the returned trajectory rather than
  recalculating it.
- Physical execution references the stored accepted plan by identity and
  digest; it does not upload a desktop trajectory as the final truth.

This avoids claiming that different processor architectures or builds will
produce byte-for-byte identical floating-point output. Local planning is a
faithful simulation; target planning is authoritative for the real robot.

## 7. Physical execution chain

The Offline Programming server, never browser code, performs the following
sequence. Browser-posted plan, evidence, manifest, and observation copies are
selection hints only; the server replaces them with its own stored request,
accepted plan, manifest, and normalized observation records before Review and
again before Start:

1. Fetch fresh target status and capability evidence.
2. Correlate the selected program revision and digest with the target's
   accepted plan.
3. Verify planner, model, cell, configuration, calibration, tool, frame,
   settings, trajectory, start-pose, output, and safety evidence.
4. Issue a short-lived, one-use review token for that exact identity.
5. Acquire fresh motion authority with lease and fencing.
6. Ask Motion Server to preload the accepted plan by plan ID and digest.
7. Confirm that Motion Server reports that exact plan identity as loaded.
8. Perform the internal controlled-start Arm using the RTCore's exact fresh
   live-mode epoch and the same controller lease/fence. There is no standalone
   browser Arm endpoint.
9. Start play with the same lease/fence and idempotency identity.
10. Continuously renew the lease and independently re-observe the Motion
   Server accepted-plan identity, controller authority, and live RTCore
   EtherCAT/servo/weld output read-back.
11. Monitor the live runtime, accepted-plan identity, progress, faults, and
   authority freshness.
12. On stop, fault, disconnect, lease loss, identity mismatch, or process
    shutdown, execute the
    protective tail: stop motion, force weld off, disarm where appropriate,
    release authority, and record the audit result.

Motion Server interpolates the accepted trajectory into RTCore command frames;
RTCore drives EtherCAT. The RTCore NATS bridge is also a final-consumer
controller-role and lease/fence boundary: it independently observes the exact
Motion Server authority status and rejects energizing commands whose
controller boot ID, lease ID, fence epoch, campaign, or intent sequence does
not match. Runtime frames may be interpolated or quantized and therefore need
not equal the sparse trajectory samples byte-for-byte. They must be
demonstrably derived only from the reviewed accepted plan, in order, within
defined interpolation and quantization tolerances.

## 8. Service boundaries

### Offline Programming browser

- Spreadsheet and CAD authoring.
- Project history and local drafts.
- Visualization and playback of returned samples.
- Program catalog client through same-origin APIs.
- Explicit target selection and operator review.
- No NATS subjects, service credentials, raw robot hosts, UDP, RTCore, or
  EtherCAT access.

### Offline Programming server

- CAD worker boundary.
- Local canonical simulation lifecycle and containment.
- Motion Server program-catalog proxy/adapter.
- Connected-target discovery and normalized observation.
- Target planning requests and accepted-plan evidence cache.
- Execution review, identity correlation, lease/fence handling, audit, and
  idempotency.
- Static application serving and browser-safe API projection.

### Robot V4 Motion Server

- Authoritative program catalog.
- Production spreadsheet/Tesseract planner.
- Robot/cell/configuration validation.
- Accepted-plan storage and identity.
- Exact-plan preload and playback.
- Runtime observation and protective motion behaviour.
- The sole normal bridge from a program/plan to RTCore motion commands.

### RTCore and EtherCAT

- Deterministic real-time interpolation/control and hardware I/O.
- No understanding of CAD projects, browser documents, or spreadsheet UI.
- No direct Offline Programming connection.

### SteamDeck

- Physical teach-pendant spreadsheet editor.
- Program create/load/edit/save through the Motion Server catalog.
- Plan status, plan request, and playback controls appropriate to the
  teach-pendant safety model.
- No requirement to understand the Offline Programming CAD document.

## 9. Implemented browser-safe API surface

The Offline Programming server exposes these versioned capabilities:

```text
GET    /api/offline-programming/v1/health
GET    /api/offline-programming/v1/capabilities

GET    /api/offline-programming/v1/programs
POST   /api/offline-programming/v1/programs
GET    /api/offline-programming/v1/programs/{program_id}
PUT    /api/offline-programming/v1/programs/{program_id}
GET    /api/offline-programming/v1/programs/{program_id}/revisions

GET    /api/offline-programming/v1/local-simulator/status
POST   /api/offline-programming/v1/local-simulator/start
POST   /api/offline-programming/v1/local-simulator/stop
POST   /api/offline-programming/v1/local-simulator/plans

GET    /api/offline-programming/v1/connected-targets
GET    /api/offline-programming/v1/connected-targets/{profile}/status
GET    /api/offline-programming/v1/connected-targets/{profile}/capabilities
POST   /api/offline-programming/v1/targets/{profile}/plans

GET    /api/offline-programming/v1/plans/{plan_id}
GET    /api/offline-programming/v1/execution-gateway/capabilities
POST   /api/offline-programming/v1/execution/review
POST   /api/offline-programming/v1/execution/start
POST   /api/offline-programming/v1/execution/{execution_id}/stop
GET    /api/offline-programming/v1/execution/{execution_id}
```

Requirements for every mutating request:

- authenticated server-side target binding;
- request and correlation IDs;
- program/plan revision and digest;
- idempotency key;
- typed success and rejection results;
- bounded body, response, and deadline;
- audit record; and
- no exposure of private transport details to the browser.

## 10. Offline Programming user experience

The workstation has five coordinated areas:

1. Program spreadsheet — the primary editable program representation.
2. CAD and seam tools — optional derivation of motion rows from part
   geometry.
3. 3D viewer — robot, part, fixtures, TCP, planned path, collision markers,
   and playback.
4. Program and plan inspector — revisions, provenance, planner identity,
   validation evidence, warnings, and conflicts.
5. Target and execution area — explicit preview/local/robot target selection,
   plan action, review, and later controlled execution.

The spreadsheet supports:

- clear row type and enabled state;
- stable row IDs with reorder, insert, duplicate, and delete;
- motion targets and frame selection;
- speed, acceleration, blend/fine, positioner, weld/process, and I/O fields;
- loops, comments, capture, and calibration rows;
- CAD seam/pass provenance;
- validation state per row;
- taught/manual override state;
- keyboard-friendly bulk editing; and
- filtered advanced fields without hiding their stored values.

It should not recreate SteamDeck hardware buttons or virtual joystick
actions. Compatibility lives in the data and service contract.

## 11. State and safety model

Suggested program/plan states:

```text
draft
  -> saved_unplanned
  -> planning
  -> accepted_simulation
  -> accepted_physical
  -> execution_ready
  -> running
  -> completed | stopped | faulted
```

Any material edit moves the program back to `saved_unplanned`. A target,
manifest, robot model, calibration, cell configuration, tool, work frame,
settings policy, planner build, or collision-world mismatch also blocks
execution readiness.

Fail-closed cases include:

- missing or unavailable canonical planner;
- legacy or unexpected planner binary;
- stale program or plan revision;
- program or trajectory digest mismatch;
- model, joint order, target, configuration, or calibration mismatch;
- incomplete collision/limit/singularity evidence;
- start-pose disagreement;
- target disconnected or status stale;
- active fault, unsafe output state, or weld state;
- lease/fence loss;
- loaded plan identity different from reviewed identity; and
- runtime departure from the accepted-plan tracking tolerance.

## 12. Delivery phases

### Phase 0 — Freeze the contracts — implemented

- Inventory all Offline Programming and SteamDeck fields.
- Publish the program, catalog, and accepted-plan schemas.
- Define canonical units, orientation, hashing, revisions, unknown-field
  behaviour, and conflict semantics.
- Add valid and invalid golden fixtures.
- Record the planner and execution identity rules as tests.

Exit condition: TypeScript, SteamDeck C++, and Motion Server agree on the same
fixtures and no program data is silently discarded.

### Phase 1 — Shared program catalog — implemented

- Make Motion Server catalog get/create/update/revision operations explicit.
- Add optimistic concurrency and immutable history.
- Complete bidirectional Offline Programming adapters.
- Preserve the existing SteamDeck workflow while making its parser/writer
  schema-conformant.
- Add catalog audit and typed conflicts.

Exit condition: a program can round-trip Offline Programming → Motion Server
catalog → SteamDeck → edited catalog → Offline Programming with stable IDs,
no lost fields, a new revision, and a stale prior plan.

### Phase 2 — Full Offline spreadsheet editor — implemented

- Expand the Offline domain model to the complete shared program.
- Implement row types, metadata, constraints, bulk editing, validation, and
  revision/conflict UI.
- Connect CAD-derived seam/pass rows without making CAD mandatory.
- Implement explicit teach-modified/manual-override handling.
- Keep project history separate from catalog history.

Exit condition: Offline Programming and SteamDeck can each create and edit
programs the other can use.

### Phase 3 — Correct local canonical simulation — implemented

- Local simulation uses the Robot V4 Motion Server planner rooted at
  `motion-server/v1`.
- Remove copied/divergent planner logic from the Offline path.
- Package the correct Robot V4 URDF, meshes, collision assets, Tesseract
  environment, and planner settings.
- Return accepted-plan-shaped simulation evidence and digests.
- Keep the browser preview visibly separate.

Exit condition: local simulation invokes the same Robot V4 planner function
and contract used by live Motion Server planning, with no hardware services
started.

### Phase 4 — Target-authoritative planning — implemented

- Install the server-owned target planning and observation adapter.
- Submit saved program revisions to the selected Motion Server.
- Store and display target-returned accepted plans and evidence.
- Add identity and stale-evidence blockers.
- Compare local and target outcomes diagnostically without requiring
  byte-for-byte floating-point equality.

Exit condition: the physical-target plan shown in Offline Programming is the
same stored plan Motion Server can later preload.

### Phase 5 — Reviewed execution — implemented in software; hardware gated

- Install server-owned transport, manifest allowlists, audit sink,
  idempotency store, and lease/fence authority. Secured NATS credentials and
  subject ACLs remain a mandatory physical-deployment gate.
- Implement one-use review tokens.
- Implement preload-by-identity, loaded-identity confirmation, play,
  monitoring, stop, and protective tail.
- Validate accepted-plan-to-RTCore derivation in simulation before enabling
  hardware.

Exit condition: physical play remains impossible unless every identity and
safety condition is fresh and exact.

### Phase 6 — Cutover and retirement — implemented for Offline Programming

- Remove the Path Planner adapter and compatibility language from Offline
  Programming.
- Do not retain an Offline Path Planner compatibility or reference adapter;
  the old application is outside the supported product path.
- Remove legacy planner defaults and fail startup if an unexpected planner is
  configured.
- Update deployment, service documentation, operator guidance, and recovery
  procedures.

Exit condition: Offline Programming plus SteamDeck are the two supported
editors, Motion Server is the program/plan authority, and no production path
depends on Path Planner or `motion-server/v1`.

## 12.1 Implementation map and operational gates

The phases above are implemented by these boundaries:

- Shared contracts and fixtures:
  `robot/v4/contracts/programs`.
- Offline program model, adapters, spreadsheet, project history, and catalog
  client: `offline-programming/v1/ui/src/contracts`,
  `ui/src/domain`, `ui/src/programs`, and `internal/catalog`.
- SteamDeck peer parsing/edit/save:
  `robot/v4/steamdeck/src/robot_v4_steamdeck_spreadsheet_*`.
- Canonical local simulation:
  `offline-programming/v1/internal/simulator` calling the Robot V4 Motion
  Server persistent planner and warm Tesseract/Bullet collision worker.
- Physical planning and immutable plan storage:
  `offline-programming/v1/internal/targetplanning` and
  `motion-server/v1/src/robot_v4_cartesian_accepted_plan.hpp`.
- Server-owned observation and authority:
  `internal/connected`, `connected_monitor.go`, and
  `connected_registry.go`.
- Reviewed execution, audit, idempotency, and supervision:
  `internal/execution`.
- Final-consumer authority fence:
  `robot/v4/realtime-core/natsbridge/bridge_controller_authority.go` and
  `bridge_controller_observer.go`.

Connected planning requires explicit server flags, an exact daemon-v1 service
resolver, and a Motion Server catalog origin. Connected execution additionally
requires `--enable-connected-execution`, an absolute
`--execution-state-dir`, and a target manifest whose
`leader_controller_roles` includes `offline-programming`. The nine-motor bench is the
current opted-in physical target; a manifest without that role remains
fail-closed.

The Motion Server `trusted-lan` leader option supplies cooperative role,
lease, and fence enforcement; it is not cryptographic transport
authentication. The physical deployment gate must commission secured NATS
access and subject ACLs before connected execution is enabled.

Audit records are append-only JSONL and are fsynced before success is
reported. Applied command identities are persisted. Review tokens and active
sessions are intentionally process-local: after a restart the operator must
obtain fresh evidence and Review again. A graceful shutdown stops every
active session before releasing authority. If Motion Server restarts or
evicts an accepted plan, exact preload fails and replanning is required.

These software gates do not authorize an uncommissioned robot. Physical
enablement still requires the separately reviewed Robot V4 deployment,
on-robot stop/disarm/weld-off checks, start-pose/tracking commissioning, and
recorded hardware acceptance evidence.

## 13. Required acceptance tests

### Contract and editing

- Complete golden program parses and reserializes in TypeScript, SteamDeck
  C++, and Motion Server without field loss.
- Offline-created, SteamDeck-created, CAD-derived, taught, loop, calibration,
  weld, positioner, and I/O programs round-trip.
- Stable IDs survive reorder and edits.
- Every material edit increments revision/digest and invalidates plan state.
- Concurrent edits receive a conflict and never silently overwrite.
- Schema migrations are deterministic and preserve recoverable source data.

### Planning

- Local persistent and live Robot V4 Motion Server routes call the same
  planner implementation.
- A fixed program and fixed assets produce the same canonical structural
  plan digest after excluding explicitly non-canonical runtime metadata.
- Missing Tesseract, URDF, meshes, planner identity, or collision data fails
  closed.
- Browser preview can never satisfy accepted-plan checks.
- Target planning returns all required identity and evidence fields.

### Execution

- Review fails for every single stale or mismatched identity input.
- Review token is one-use, exact-scope, and short-lived.
- Preload confirmation must match the reviewed plan ID and digest.
- Duplicate start requests are idempotent.
- RTCore simulation capture follows the accepted trajectory in order and
  within defined interpolation/quantization tolerances.
- Disconnect, lease loss, fault, plan mismatch, or tracking departure invokes
  the protective tail.
- No browser route can send raw NATS, UDP, RTCore, EtherCAT, jog, or weld
  commands.

### Deployment

- Deployed Motion Server build and asset digests appear in plan evidence.
- Local simulator reports the exact Robot V4 planner build it invokes.
- CI runs without physical hardware and cannot access EtherCAT.
- Hardware enablement requires a separately reviewed deployment gate and
  recorded on-robot commissioning tests.

## 14. Definition of done

The architecture is complete when:

- Offline Programming provides the full spreadsheet/CAD authoring workflow;
- SteamDeck can create, load, edit, and save the same programs;
- both editors use one versioned Motion Server-owned program catalog;
- program revisions, conflicts, provenance, and plan invalidation are
  reliable;
- local simulation uses the Robot V4 production planner implementation;
- the selected robot creates and stores the final physical accepted plan;
- the operator reviews the exact trajectory and evidence that will be
  preloaded;
- Motion Server confirms that exact plan before play;
- RTCore/EtherCAT output is demonstrably derived from that plan;
- all stale, mismatched, unavailable, or unsafe states fail closed; and
- Path Planner and the legacy Motion Server are absent from the supported
  production path.
