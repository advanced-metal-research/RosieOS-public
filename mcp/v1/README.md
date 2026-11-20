# RosieOS MCP server

Since the `rosie` CLI (`cli/v1`) the logic below lives in its library
(`cli/v1/src/lib`, imported here as `rosie-cli/lib/*.js`): program
orchestration, robot admission and actions, bounded jogs, the heartbeat
keeper, the OLP engine bundle (`cli/v1/core/entry.ts`), the `.rdt` reader.
This server is a front end that renders it as MCP tools; build it with
`npm run build` here (it builds `cli/v1` first).

`rosieos-mcp-server` is a [Model Context Protocol](https://modelcontextprotocol.io)
server (stdio, TypeScript SDK v2, spec 2026-07-28 with 2025-era clients still
served) that lets an AI agent read and, within limits, drive a RosieOS cell:

- **rt-core**, the realtime controller, through rt-control's public API
  (observation only);
- **OLP**, the controller of record: machine status, cells, control and
  maintenance, the cell program store, and the robot actions it gates;
- **programs**: OLP's spreadsheet, created, opened, edited row by row and
  field by field, taught waypoints, STEP parts with their detected weld
  seams, part placement, planning, and saving to the cell store or a file;
- **the motion servers**: the dense trajectory daemon's TCP ingest and the
  Cartesian daemon's resolve-only kinematics;
- **the weld planner**: health, planning a `.weldplan`, fetching the dense
  trajectory it filed;
- **this checkout**: cell manifests, the A6-EC alarm table, `.rdt` files and
  the component documentation;
- **the deploy module**'s read-only commands.

Every route it calls already exists and is documented in that component's
README. The one change outside this directory is
`offline-programming/v1/ui/src/rails/authoring.ts`: rails mode's weld search
and accept logic, moved out of the DOM-bound `rails/index.ts` so the browser
and this server run the same code (below).

## Programs: OLP's own engine, not a copy

A program here is edited by OLP's TypeScript, bundled for Node exactly as
`steamdeck/real/v5` bundles `olp-core` for the Deck (`cli/v1/core/entry.ts` ->
`cli/v1/dist/core/olp-core.mjs`, `cli/v1/scripts/build-core.mjs`; the bundle
is the CLI's, shared). The Deck's session
(`olp-core/session.ts`: one open program, undo history, `view`, `fields`,
`edit`, `record`, `addEvent`, `remove`, `move`, `enable`) and run module
(`applyPlan`, `loadBody`) are re-exported unchanged; `core/entry.ts` adds only
what a client with no window needs, each one a call into OLP's modules:

| Step | OLP code it runs |
| --- | --- |
| New program | `createBlankSpreadsheetProject({railsAuthoring: true})`, `pinRobotDescription` (olp-core `newProject`) |
| Every field edit | `groupsForNode` + `applyEdit` + `documentProblems` (olp-core `edit`): the parameter dialog's specs, units and refusals |
| Waypoint from joint angles | `POST /robots/capture-model`, then the TCP by urdf-loader + three.js at the model's TCP frame (`workstation-renderer.ts tcpPoseFor`, without the renderer), `teachCapture`, `recordMove`: the browser's preview capture |
| Import STEP | `POST /cadquery/topology` (64 samples per edge), `normalizeTopology`, `appendCadQueryTopology` (centres the part on the work frame), the first part names the program; `POST /seam detect_joints` |
| Place the part | `setWorkpiecePlacement` (Move part's fields; ±5000 mm, ±360°) |
| Search a seam / add a weld | `rails/authoring.ts`: `searchSeam` (both directions, better coverage, clearance at the chosen angles), `measureRunPiece` (a blocked seam's clear runs), `decideWeld` (`seamWeldNodeFor`, reaccept merge); `insertWeld` |
| Fitted torch, angle search | `rails/parameters.ts` schemas and shipped presets through `parameters/library.ts` + `model.ts` `createParameterSet`/`set` (refused, never clamped), `torchProblems` (refused here: the seam worker's tooling rules would refuse the request), `torchRequest`; session state, as the browser's windows; a change drops the cached searches (`parametersChanged`), accepted welds untouched |
| Work/travel angle profile | `scalar-function.ts fromPoints` (a flat set is a constant; `isEditable` refuses bspline/samples), the profile editor's -90..90° range, then olp-core `edit` (`applyEdit`'s profile kind, `documentProblems`) |
| Workcell fixtures | `workcell.ts` `newBoxFixture`, `newStepFixture`, `editWorkcellFixture` (its clamps 1-5000 mm, ±5000 mm, ±360° refused instead), `defaultWorkcellFixtures`, as main.ts's Stand card; STEP bodies through OLP's own CAD client (`cad/client.ts tessellate`, `normalizeWorkpieceMesh`) |
| Parts | a second Import STEP adds a part (`appendCadQueryTopology` / `beginAddedWorkpiece`); `activateWorkpiece` (main.ts selectWorkpiece); main.ts deletePart (`forgetActiveWorkpiece` + the cleared working copy) |
| Waypoint, line, arc welds | `waypoints/session.ts WaypointSession` hosted as main.ts hosts it (`newWeldProcess` from the program's process defaults), `rpyToQuat`, `withWaypoints`, `resolveStartLinks`; staged and committed only when `documentProblems` finds nothing new |
| Plan | `planRequestBody` with the session's fitted gun, the active part's STEP and `workcellFixturesWire(environmentFixtures)`, each STEP fixture and inactive part as a mesh tessellated from the kept STEP; `POST /weld-plan`; olp-core `applyPlan` |
| Record from the robot | olp-core `teachObservation`, refused first by the Dense panel's editing lock (`lockReason`: loaded, staged, playing, paused), restated in `core/entry.ts editingLockReason` because it is a method of a DOM class |
| Save | the cell store gets the bare project (`cell-sync.ts cellDocument`), revision-checked; a file gets `exportProjectJSON`'s envelope |

`rails/authoring.ts` is the browser's own code moved, not rewritten:
`rails/index.ts` now calls `searchSeam`, `measureRunPiece`, `runIndexAt`,
`runPiecePayload` and `decideWeld`, with its cancellation (`assertCurrent`,
the entry still in the list) passed in as a checkpoint after every await, where
those checks stood before. The UI's rails lifecycle tests run against it,
`test/rails-authoring.test.ts` tests it directly, and the whole UI suite
passed after the move (1043/1043).

STEP bytes are never shared between clients (OLP README, "Cell program
store"): this server keeps every part and STEP fixture it imported under its
fingerprint in `ROSIE_MCP_STATE_DIR/sources` (default `~/.rosie/mcp`,
shared with the `rosie` CLI), CadQuery's tessellation replies in
`.../meshes`, and planned trajectories in `.../dense`, its equivalent of the
browser's IndexedDB. A program opened from the cell whose part this server never
imported needs `step_path` once; the file is checked against the program's
fingerprint. A plan needs a mesh for every STEP fixture and inactive part: it
rebuilds one from the kept CadQuery reply through OLP's own CAD client (no
CadQuery run), tessellates the kept STEP when no reply is kept, and refuses
(posting nothing) when that STEP is not kept either, naming
`fixture_step_paths`.

A plan carries the ACTIVE part only: its welds, its STEP. Every other part
goes to the planner as a mesh obstacle where it is placed
(`environmentFixtures`), beside the workcell fixtures. The classic
`posed_polyline` weld is legacy in OLP (`planRequestBody` refuses it beside
taught moves); this server authors rails and waypoint welds only.

The bundle needs OLP's UI packages: run `npm ci` in `offline-programming/v1/ui`
before `npm run build` here (it builds `cli/v1` first), so this server, the
CLI and the browser run the same three.js, urdf-loader and happy-dom.

## Why robot actions go through OLP only

rt-control grants one controller at a time. OLP holds that grant for the
browser and both Decks, runs the 5 s heartbeat deadman, and applies the
maintenance, control and focus gates (`offline-programming/v1/README.md`,
"Maintenance notice", "Another controller"). A second grant from this server
would sit where OLP's STOP cannot reach, the reason OLP's README gives for not
using `rtctl program-start` beside it. So the rt-core tools are GETs, and
every robot action is an OLP request carrying this server's own
`X-Rosie-Client` label and `X-Rosie-Client-Id` (`mcp-<16 hex>`, one per
process), so OLP's journal and the maintenance overlay name it.

## Robot actions: three modes

`ROSIE_MCP_ROBOT_ACTIONS` decides which robot-action tools exist:

| Mode | Registered | Refused |
| --- | --- | --- |
| `off` (default) | Stop, Disarm, Pause, Finish cycle | — |
| `simulation` | the above, plus Select cell, Arm, Load, Play, Run program, and the bounded jogs (Move joints, Cartesian step, Jog hold) | each of those on a target rt-core does not describe as backend `simulation` |
| `cell` | the same tools | nothing beyond OLP's own gates. Real robots included: the owner's choice, never a default |

Stop, Disarm, Pause and Finish cycle only end motion sooner. OLP passes them
from any client through every gate, so they exist in every mode.

Under `simulation`, each motion tool re-reads `dense-execution/status`
immediately before acting and refuses unless `target.simulation` is true.
OLP derives that flag from rt-core's own Describe backend
(`internal/targets/picker.go`), not from a cell name. The action carries that
read's `X-RT-Target-Generation`/`X-RT-Target-Cell`, so if another client
switches cells between the read and the action, OLP refuses it as
`target_changed`. Tested: a real-robot target gets no POST at all.

**Never offered, in any mode:** Home (HM35 makes the current position Home and
is run only by an operator at the mechanical home pose, never as a recovery),
fault reset (Arm already resets what the drive profile classes resettable;
anything else is the owner's call), recover-to-zero (the browser's All joints
to 0° press, which can arm), open-ended hold-to-run jogging (the jogs below
are bounded per call), test-program upload, cell catalogue edits, program
store save/delete, and every deploy command that builds, installs, restarts,
locks, rolls back or reboots.

### Jogging, bounded per call

An agent has no hand on a trigger, so the jog tools take the browser's own
requests and bound each call here before OLP sees it, in addition to OLP's
and rt-core's limits. They are admitted exactly like Arm and Play (fresh
status, the mode's refusal, the read's selection fence on every request).
`target_id` is the capabilities' `cell` (the rt-core binding's pair id),
which is not `target.cell_id`: the local stack answers `local-dev` and
`local-simulation`. `revision` is the lane's, from `GET jog/state` or
`cartesian/state`, and `session_id` the status's session (Arm first).

| Tool | OLP request | Copied from | Bounds here |
| --- | --- | --- | --- |
| `rosie_olp_move_joints` | `POST dense-execution/move` `{target_id, session_id, revision, axis, degrees, fraction}`; several joints, or any absolute target, add `axes: [{axis, degrees}]` / `[{axis, degrees: 0, position_rad}]` | `ui/src/execution/jointJog.ts` `JointAngleMove` (GO, `beginMany`), olp-core `machine.ts` `zeroMoves`, `absoluteMoves` | ≤ `ROSIE_MCP_JOG_MAX_DEG` (30°) per joint per call, absolute targets measured from the live position; the described range; fraction ≤ `ROSIE_MCP_JOG_MAX_FRACTION` (0.5), default 0.1 |
| `rosie_olp_cartesian_step` | `POST dense-execution/cartesian/move` `{target_id, session_id, revision, frame, axis, distance_mm, angle_degrees, fraction}` | `cartesianMove.ts` `cartesianMoveBody`, olp-core `stepBody` | ≤ 100 mm / 15° per call (`ROSIE_MCP_CARTESIAN_MAX_MM` / `_DEG`, the browser's default step), fraction as above |
| `rosie_olp_jog_hold` | joint: `jog/begin`, `jog/update` every 50 ms, `jog/end` `{target_id, session_id, revision, axis, direction, fraction, end_cause}`; Cartesian: `cartesian/start`, `intent`, `halt` with `{frame, twist, fraction}` | `jointJog.ts` `JointJogHold`, `cartesianJog.ts` `cartesianJogAPI`, `cartesianHoldTwist` | duration ≤ `ROSIE_MCP_JOG_HOLD_MAX_S` (3 s, default 1 s), fraction ≤ `ROSIE_MCP_JOG_HOLD_MAX_FRACTION` (0.25, default 0.1); commanded travel (speed × duration) under the per-call caps |

A move returns when OLP answers `done` with the move's axis and a revision
past the intent's (correlated native completion), as the browser accepts it.
OLP stops a joint move whose request is cancelled, so this server waits the
move's own duration bound (each joint at 0.75 × fraction of its described
velocity, plus ramps, twice, plus 30 s). OLP's 5 s deadman applies during a
joint move, so this server heartbeats while the request is in flight and
restarts its window when the move completes. A Cartesian step is exempt from
the deadman in OLP; it is beaten around the same way.

A hold starts the 50 ms cadence with the press (up to five intents in flight
when OLP reports `pipelined_intents`), ignores a `jog_session_stale` on an
intent sent before the press is answered (the browser's rule), and ends at
the first other refusal or transport failure. End/Halt is sent from a
`finally` block every time, with the browser's `end_cause`
(`button_released`, or `intent_refused:<OLP code>`), without waiting for
intents still in flight (End advances the revision, so OLP drops them).
`dense-execution/stop` follows only a hold that began (OLP accepted the
press, or the press's answer was lost), when End did not land or the hold
then failed. A refused press started nothing: it gets End only, never Stop,
whatever End answers (a Stop there would end the program or loop that
caused the `jog_mode_conflict`).
The answer carries OLP's `last_jog_end` (code, text, re-pull).

OLP refuses every jog while a program plays or a loop runs
(`jog_mode_conflict`); the answer says which, from the status read. OLP's
fence exempts only `jog/stop` and `jog/end` (`dense_execution.go`
`denseTargetRequests`); `cartesian/halt` is exempt from the maintenance gate
but still fenced. Every request here carries the fence headers anyway.

### The heartbeat window

OLP stops the robot when it holds the grant and no client heartbeat has
landed for 5 s (`ui_heartbeat_lost`). One request can arm, take the grant
and move before it answers (Run program waits up to 300 s, a joint move its
own duration bound), so every motion tool (Arm, Load, Play, Run program and
the jogs) starts the beats **before** its request is sent and keeps them up
while it is in flight, whatever the window or OLP's status says; only Stop or
Disarm ends them meanwhile. After a successful answer this server heartbeats
the current session once a second for `ROSIE_MCP_HEARTBEAT_WINDOW_S`
(default 120 s, unverified; 5-3600). Each later motion tool restarts the
window. A refused or failed request does not touch the window: when the last
request in flight ends, the beats run out an earlier success's window or stop,
so refusals keep nothing alive, overlapping ones included. The beats stop
early on Stop or Disarm (also one pressed during an action in flight), or
when OLP no longer holds a session (checked after a 2 s grace from the last
answer, unverified). When the window ends, or this process exits, the beats
stop and OLP's deadman stops the robot unless an operator's browser or Deck
is keeping the session alive. Loops started here are always attended
(`unattended: false`), so they end with the window too.

Each beat is its own heartbeat request with a 1.5 s timeout (unverified:
four beats still fit inside OLP's 5 s deadline); a hung request never holds
up the next beat. The session id is reused from the last status read; status
is read again every 3 s (unverified), at the start of and after every motion
request, and at the next beat after a refused heartbeat
(`dense_session_mismatch`: the session ended or Load replaced it), with the
same timeout and never in front of a beat for a known session.

### Run program is a conversation

`rosie_olp_run_program` mirrors the operator dialog. The first call (confirm
flags false) answers `state: "confirm"` with the action ("Arm and run",
"Move to program start and run") and every joint's from/to/delta, and
nothing moves. The agent shows that to the user. Only on their agreement does
it call again with the confirm flags and the same joints list. OLP refuses
`confirmation_stale` if any joint has moved more than 0.1° since.

## Tools

| Tool | Reads/acts on | Notes |
| --- | --- | --- |
| `rosie_rtcore_describe` | rt-control `GET /v1/describe` | backend, digests, axis limits, capabilities |
| `rosie_rtcore_get_status` | `GET /v1/status` (+ Describe for names) | armed, lease, faults and recovery classes, Home, per-axis alarms |
| `rosie_rtcore_get_maintenance` | `GET /v1/maintenance` | servo-host notice, grant holder |
| `rosie_rtcore_list_events` | `GET /v1/events?after=` | paginated by sequence |
| `rosie_olp_get_health` | OLP `/health`, `/capabilities` | seam worker and motion origin reported separately |
| `rosie_olp_get_machine_status` | `dense-execution/status` | SIMULATION vs REAL ROBOT, session, cursor, loop, axes, heartbeat keeper |
| `rosie_olp_list_cells` | `dense-execution/cells`, `/targets` | |
| `rosie_olp_get_control` | `/maintenance`, `/control` | who controls the robot, connected clients |
| `rosie_olp_list_cell_programs`, `rosie_olp_get_cell_program` | `cell-programs` | read only |
| `rosie_olp_get_local_simulator_status` | `local-simulator/status` | |
| `rosie_olp_stop`, `_disarm`, `_pause`, `_finish_loop` | robot | every mode |
| `rosie_olp_select_cell`, `_arm`, `_load`, `_play`, `_run_program` | robot | `simulation` or `cell` only; Load and Run program take `from_open_program` (the open program's current plan) |
| `rosie_olp_move_joints`, `_cartesian_step`, `_jog_hold` | `dense-execution/move`, `cartesian/move`, `jog/*`, `cartesian/*` | `simulation` or `cell` only; bounded per call ("Jogging, bounded per call") |
| `rosie_program_new`, `_open`, `_save` | engine; cell store `cell-programs` or a file | open/save from the cell are revision-checked; a 409 never overwrites |
| `rosie_program_view`, `_get_row` | engine | the spreadsheet; every field of a row with path, unit, range |
| `rosie_program_edit_row` | engine | any field by OLP path, display units; OLP refuses out-of-range and read-only |
| `rosie_program_add_waypoint`, `_reteach_waypoint` | `/robots/capture-model` + engine FK | MoveJ/MoveL/MoveC from joint angles, optional Cartesian target, via poses |
| `rosie_program_record_robot_pose` | `dense-execution/status`, `/cell`, `/robots/capture-model` | new, reteach or via waypoint from the selected cell's measured pose (OLP's teach rules); reads the robot, moves nothing; refused under the editing lock |
| `rosie_program_robot_settings` | engine; `dense-execution/cell` | show, pull the cell's settings and calibration, adopt the current robot description |
| `rosie_program_add_event`, `_remove_row`, `_move_row`, `_enable_row`, `_set_plan_speed`, `_undo` | engine | dwell, IO, home; OLP's reorder and delete rules |
| `rosie_program_import_step`, `_add_part`, `_place_part` | `/cadquery/topology`, `/seam detect_joints` | the part, centred, then placed in the work frame; a second import adds a part |
| `rosie_program_list_parts`, `_switch_part`, `_remove_part` | engine | which part is active (edited and planned); switching needs its STEP kept or `step_path` |
| `rosie_program_list_fixtures`, `_edit_fixture` | engine; `/cadquery/topology tessellate` | stand box (mm, °), STEP obstacles; add_stand, edit, replace_with_step, add_step_fixture, cell_default, remove |
| `rosie_program_rails_settings` | engine | fitted torch and angle search: show, set (preset and/or values), reset; session state |
| `rosie_program_list_seams`, `_search_seam`, `_add_weld` | `/seam plan_torch_path`, `check_torch_fits` | clear / partial (per-run welds) / blocked; accept or trim; with the session's torch and search |
| `rosie_program_set_weld_profile` | engine | a rails weld's work or travel angle: constant or [s, °] points |
| `rosie_program_add_waypoint_weld`, `_edit_waypoint_weld` | engine | waypoint / line / arc welds from torch poses in the part frame (mm, °, standoff, C0/C1/C2) |
| `rosie_program_plan`, `_export_weldplan` | `POST /weld-plan`, `/weld-plan/export` | measured start when a cell is selected, or `from_program_start`; repeat adds the loop return; fixtures and other parts as meshes |
| `rosie_program_preview_path` | kept `.rdt` + `/robots/capture-model` | the planned TCP path per segment through OLP's FK (mm, chord deviation, peak speed); moves nothing |
| `rosie_dense_daemon_status` | daemon TCP `status` | inert |
| `rosie_dense_daemon_validate` | daemon TCP `validate` | upload's dry run, nothing stored |
| `rosie_kinematics_forward` | `robot-v4-cartesiand --resolve-only`, `fk` | computation only |
| `rosie_kinematics_cartesian_line` | `--resolve-only`, `move` | joint waypoints of a straight TCP move, or the refusal |
| `rosie_planner_get_health` | planner `/api/motion/health`, `/progress` | |
| `rosie_planner_plan_weldplan` | `POST /api/motion/plan` | full result saved to a file, summary returned |
| `rosie_planner_fetch_dense` | `GET /api/motion/dense/{digest}` | saved as `.rdt` and inspected |
| `rosie_list_cells`, `rosie_get_cell_manifest` | `rt-core/config/cells` | committed pins |
| `rosie_lookup_drive_alarm` | A6-EC manual table + `a6ec.json` `fault_reset` | whether rt-core Reset clears it |
| `rosie_inspect_dense_trajectory` | a local `.rdt` | every ingest rule, digest, identity |
| `rosie_search_docs` | 23 component documents | file:line hits with headings |
| `rosie_deploy_inspect` | `go run .` in `steamdeck/real/v4` | `release_plan`, `olp_status`, `precheck`, `verify`, `config_dry_run`, `maintenance_show` |

Resources: `rosieos://docs/<id>` (the same 23 documents),
`rosieos://cells/<cell>` (manifest + summary), `rosieos://mcp/config`
(endpoints, mode, heartbeat state). Prompts: `rosie_diagnose_cell`,
`rosie_author_weld_program`, `rosie_run_program_in_simulation`,
`rosie_review_release`.

`cli/v1/src/lib/rdt.ts` (shared with the CLI) is a fourth reader of `robot.v4.dense-joint-trajectory.v1`
beside the Python encoder, the Go twin and the C++ daemon, ported rule for
rule from the Go twin. Its test holds it to the reference fixture's recorded
digest and block hashes. If the format changes, this reader fails with the
other three.

## Build, test, run

From `mcp/v1`, with Node 20 or later (Windows: `C:\Program Files\nodejs`;
cells and Mac: `~/.rosie/node-22.13.1/bin`; the CLI's snapshot renderer,
which this server does not use, needs 22):

```sh
(cd ../../offline-programming/v1/ui && npm ci)   # the engine bundle's packages
(cd ../../cli/v1 && npm ci)                      # the shared library (rosie-cli)
npm ci
npm test          # build (cli/v1 first, then tsc), then node --test: 41 tests, no network, no robot
npm run build     # cli/v1 (engine bundle, library, snapshot page) + dist/src/index.js
npm run typecheck # cli/v1 and its engine entry against OLP's own sources, then this server
node dist/src/index.js --help
```

The tests drive the server through the SDK's own client and
`createMcpHandler`. Their fakes sit at the process boundary: OLP over HTTP,
rt-control over a Unix socket (a named pipe on Windows), the daemon over TCP.
The offline tools run against this checkout's committed files.

### Configuration

| Variable | Default | Meaning |
| --- | --- | --- |
| `ROSIE_OLP_URL` | `http://127.0.0.1:8794` | OLP API. Against a cell from the Mac: `http://127.0.0.1:18894` (the `olp ui start` tunnel) |
| `ROSIE_PLANNER_URL` | `http://127.0.0.1:8796` | weld planner motion server |
| `ROSIE_DENSE_DAEMON_ADDR` | `127.0.0.1:8797` | dense daemon ingest |
| `ROSIE_RT_CONTROL_SOCKET` | unset | public control socket (dev-stack: `$ROSIE_LOCAL_RT_DIR/control.sock`; cell: `/run/rosie-rt-core/control.sock`) |
| `ROSIE_RT_CONTROL_URL` + `_CA`, `_CERT`, `_KEY` | unset | mutual-TLS listener instead of the socket |
| `ROSIE_CARTESIAND_COMMAND` | `motion-server/v1/bin/robot-v4-cartesiand` if built | path, or JSON argv prefix (e.g. `["wsl","-e","/home/me/RosieOS/motion-server/v1/bin/robot-v4-cartesiand"]`) |
| `ROSIE_CARTESIAND_REPO_ROOT` | repository root | the root as cartesiand sees it (a WSL path under WSL) |
| `ROSIE_MCP_ROBOT_ACTIONS` | `off` | `off`, `simulation` or `cell` |
| `ROSIE_MCP_HEARTBEAT_WINDOW_S` | `120` | heartbeat window after a robot action |
| `ROSIE_MCP_JOG_MAX_DEG`, `ROSIE_MCP_CARTESIAN_MAX_MM`, `ROSIE_MCP_CARTESIAN_MAX_DEG` | `30`, `100`, `15` | per-call travel of a joint move/hold, a Cartesian step/hold (unverified policy) |
| `ROSIE_MCP_JOG_MAX_FRACTION`, `ROSIE_MCP_JOG_HOLD_MAX_S`, `ROSIE_MCP_JOG_HOLD_MAX_FRACTION` | `0.5`, `3`, `0.25` | largest move/step speed fraction, hold duration and hold fraction |
| `ROSIE_MCP_STATE_DIR` (or `ROSIE_STATE_DIR`) | `~/.rosie/mcp` | imported STEP parts (`sources/<fingerprint>.step`), CadQuery mesh replies (`meshes/`) and planned trajectories (`dense/`); shared with the `rosie` CLI |
| `ROSIE_REPO_ROOT`, `ROSIE_GO`, `ROSIE_MCP_CLIENT_LABEL`, `ROSIE_MCP_REQUEST_TIMEOUT_MS` | found / `go` / `rosieos-mcp-server (<host>)` / 15000 | |

### Registering it with a client

Agents are better served by the `rosie` CLI (`cli/v1`): the same library and
rules, state kept on disk instead of in this process, verdict-first output,
3D scene snapshots and reports. The project skills `.claude/skills/rosie-*`
(starting at `rosie`) are written for it.

Cursor (`.cursor/mcp.json`) or any client that takes the same shape:

```json
{
  "mcpServers": {
    "rosieos": {
      "command": "node",
      "args": ["<repo>/mcp/v1/dist/src/index.js"],
      "env": { "ROSIE_MCP_ROBOT_ACTIONS": "off" }
    }
  }
}
```

Claude Code: `claude mcp add rosieos -- node <repo>/mcp/v1/dist/src/index.js`.
Inspector: `npx @modelcontextprotocol/inspector node dist/src/index.js`.

Only stdio is served. A network listener would expose Stop and, in the other
modes, Arm and Play, and this server has no authentication of its own.

## Limits

- The Steam Deck pendant headers (`X-Rosie-Client-Focus`, `-Expect-Cell`,
  `-Pid`) are not sent; OLP then governs this client by the control rule
  alone, as it does a browser.
- JSON numbers above 2^53 (rt-core nanosecond clocks) lose precision; the
  tools use them for display only.
- `rosie_olp_get_machine_status` renders per-axis rows only from the fields
  it recognises in OLP's pass-through `axes`; `response_format: "json"`
  carries all of them.
- Smoke-tested read-only (2026-10-18, Windows, robot actions off) against a
  locally running OLP and weld planner: health, machine status (the local
  cell reported `simulation: true` from rt-core's Describe), cells and
  planner health answered; that OLP build predated `/maintenance`,
  `/control` and the program store, and those tools said so (HTTP 404).
  No robot action, rt-control, daemon, cartesiand or deploy command has run
  against a live service yet; the first such run is the owner's.
- Programs, live against the same local OLP (CadQuery and the seam worker
  real, planner the GPU-less stand-in), 2026-10-18: a 9-axis `rosie_1400_v3`
  program with two waypoints from joint angles, a dwell edited to 1.5 s (a
  negative one refused by OLP), Home, planned in 0.9 s; the kept `.rdt` passed
  every ingest rule, one segment per row. MoveL and welds were refused by the
  stand-in ("plans joint moves only" / "needs the CUDA weld planner"), as it
  must. `bracket_a.step` imported (71 edges, 3 joints, 11 seams detected in
  20 s), `seam_0001` searched partial, 85% clear, one run, accepted as
  `rails-seam_0001-run1` with approach and retract, part turned 90°. A weld
  plan on the CUDA planner and a save to a live cell store have not run yet.
- The jog tools (2026-10-19) are tested against a fake OLP only. A read-only
  probe of the local OLP confirmed the shapes they read (capabilities
  `cell`, `rt_core.description.axes`, `rt_core.status.axes[].logical_position`,
  `jog/state` and `cartesian/state` revisions); that build's status carried
  no `last_jog_end` and its state no `pipelined_intents`, so against it a
  hold keeps one intent in flight and reports no end reason. No jog has
  been sent to a live OLP from here.
- Torch and search settings start at the shipped presets (the browser's
  fallbacks) in every server process, as a fresh browser tab does; a preset
  an operator saved in a browser lives in that browser's store and is not
  offered here (give its values instead).
- A weld's `standoff_mm` has no field in OLP's parameter dialog: a rails
  weld's comes from the angle search, a waypoint weld's is per point
  (`rosie_program_edit_waypoint_weld set_standoff`).
- The editing lock refuses `rosie_program_record_robot_pose` only. The
  browser's `mutate` refuses every edit while the selected cell's session
  is loaded, staged, playing or paused; this server's other edits work on
  its own copy, and a save of a program the robot holds is refused by the
  cell store (423).
- Programs, live against the local OLP (CadQuery and seam worker real),
  2026-10-19: `bracket_a.step` imported, torch set to the MIG Ø16 nozzle (a
  50 mm radius refused in OLP's words), `seam_0001` searched (7.6 s) and
  accepted, its work angle set to a 40-45-42° profile, a 40 mm line weld
  added, a stand and `pipe_bracket.step` as a fixture (tessellated, 5.3 s),
  `bracket_b.step` added as a second part; `/weld-plan/export` packed all
  three `.weldplan`s: only the active part's STEP under `source/`,
  `tooling.json` with the MIG nozzle and its 17 mm default standoff,
  `fixtures.json` with the box, the fixture mesh and the inactive part's
  mesh. No plan was run (the local planner is the GPU-less stand-in).
- `rosie_program_search_seam` searches by seam id; the browser's edge clicks
  (`matchSeam`) are not reproduced.
