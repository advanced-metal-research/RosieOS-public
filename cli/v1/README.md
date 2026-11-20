# rosie — RosieOS from the shell

`rosie` is the agent-first command line for RosieOS: author and plan OLP
programs, **verify** them, **look** at them in OLP's own 3D renderer, diagnose
cells, review releases, and run the (simulated) robot — each a short command
with one verdict line, `--json` for structured output, and an exit code an
agent can branch on. It exists because driving the cell through ~70 MCP tools
was clanky: agents discovered call order by trial, state lived in hidden
process memory, every edit returned the whole spreadsheet, and nothing showed
them the scene, so they made geometric mistakes they would have seen at a
glance.

Agents: start with the skills in `.claude/skills/rosie*` (router: `rosie`),
installed for every Claude Code session on the machine by `install.sh`
(`rosie skills install`). People: `rosie --help`, `rosie <group> --help`,
`rosie <group> <command> --help` (every command has examples).

## Install

```bash
cli/v1/install.sh            # npm ci where needed, builds, installs ~/.rosie/bin/rosie and the agent skills (--no-skills to skip)
export PATH="$HOME/.rosie/bin:$PATH"
rosie doctor
```

Needs Node 22+ (the snapshot driver uses Node's built-in WebSocket) and, for
`rosie view` / `rosie report`, Chrome or Edge (`ROSIE_CHROME` to choose).
`npm run build` builds the OLP engine bundle (`dist/core/olp-core.mjs`, from
OLP's UI packages: `npm ci` in `offline-programming/v1/ui` first), the CLI,
and OLP's snapshot page (`dist/snapshot`). After a pull, re-run `install.sh`;
`rosie doctor` warns when OLP's sources are newer than the bundle.

## Agent skills for every session

The agent skills live in the repository (`.claude/skills/rosie`,
`rosie-local-stack`, `rosie-programs`, `rosie-verify-and-view`,
`rosie-robot`, `rosie-diagnose`, `rosie-trajectories`,
`rosie-deploy-review`), so Claude Code finds them in sessions opened here.
`rosie skills install` (run by `install.sh`; `--no-skills` skips it) links
each into `~/.claude/skills` (or `$CLAUDE_CONFIG_DIR/skills`, or `--dest`)
so every session on the machine finds them; a `git pull` updates them all,
since the links point into the checkout. `--copy` copies instead (marked;
`rosie skills list` says when a copy is stale). It only ever creates,
replaces or removes entries it made, refuses a same-named skill it did not
make, and `rosie skills uninstall` leaves everything else in place. Install
from the checkout you keep (not a temporary worktree): the links point into
it. The skills read documents through `rosie docs show <id>`, so they work
from any directory.

## The contract every command keeps

- **Text** (default): `OK: <verdict>` / `REFUSED: …` / `UNREACHABLE: …` /
  `USAGE: …` first, then a short body, `files:` written, and `next:` commands.
  Mutations print the program line and the rows they changed, never the whole
  spreadsheet (`rosie program show` does that).
- **`--json`**: `{ok, verdict, exit, data, artifacts, next}` on stdout;
  progress goes to stderr (`--quiet` silences it).
- **Exit codes**: 0 ok · 1 refused or a check failed · 2 usage · 3 a service
  unreachable. A refusal quotes OLP's / rt-core's own code and sentence.

## Targets: where rosie points

Named targets live in `~/.rosie/rosie.json` (`rosie target list|show|add|use|remove`).
`local` is built in: OLP :8794, planner :8796, dense daemon :8797, and
dev-stack's rt-control socket when it exists. A cell's OLP from the Mac is the
`olp ui start` tunnel: `rosie target add <cell> --olp-url http://127.0.0.1:18894`.
`--target <name>` selects per command. A named target's values win over the
environment; without `--target`, `ROSIE_OLP_URL`, `ROSIE_PLANNER_URL`,
`ROSIE_DENSE_DAEMON_ADDR`, `ROSIE_RT_CONTROL_SOCKET`/`_URL` and
`ROSIE_ROBOT_ACTIONS` override the default target.

**Robot actions** per target: `off` (default: stop, disarm, pause, finish
only), `simulation` (motion only on a target rt-core describes as backend
`simulation`), `cell` (real robots — the owner's setting; `target add`
refuses it without `--owner-approved`). The CLI reads `ROSIE_ROBOT_ACTIONS`,
never the MCP server's `ROSIE_MCP_ROBOT_ACTIONS`, so one front end's mode never
widens the other's.

## Workspaces: the working program on disk

Every command is a fresh process; OLP's engine keeps its open program (and
undo history) in memory, so rosie keeps what the browser keeps beside a
program in `~/.rosie/workspace/<name>/` (`--ws <name>`, default `current`):
`program.olp.json` (OLP's Export JSON), `session.json` (where it came from and
the cell revision a save is checked against, selection, detected joints,
searched seams, torch and search settings, unsaved flag, the last robot
catalogue), `history/past|future/*.olp.json` (undo/redo through OLP's own
`restoreAsRevision`: an undo is a new revision and drops the plan, as in the
browser) and a writer `lock`. STEP sources, CadQuery mesh replies and planned
trajectories live in the state dir shared with the MCP server
(`~/.rosie/mcp/{sources,meshes,dense}`, `ROSIE_STATE_DIR`). A program with
unsaved edits is never replaced silently (`program new|open` refuse without
`--force`).

## Looking at it: `rosie view scene` and `rosie report`

`rosie view scene` renders the workspace program with **OLP's own three.js
`WorkstationRenderer`** in headless Chrome: robot, part, workcell fixtures,
weld lines, taught waypoints and TCP frames, the planned TCP path, the robot
at chosen poses. Default views: `home` (the whole cell), `iso`, `front`, `top`
at the plan's start, `right` mid-plan, `iso` at the end; choose with
`--shots camera@pose` (cameras `home part path front back left right top iso`;
poses `start end 0..1 row:<id>`). PNGs land in `~/.rosie/views/…`; an agent
opens them as images. `rosie report program` writes one self-contained HTML
page (views embedded, `program check`, spreadsheet, planned path).

The page is `offline-programming/v1/ui/snapshot.html` + `src/snapshot.ts`
(imports viewer/domain modules only, never `main.ts`), built by
`scripts/build-snapshot.mjs` into `dist/snapshot` (never the UI's `dist/`,
the redirect stub); the driver is `src/lib/snapshot` (raw CDP, a static
server). It fails loudly (page exceptions, console errors) and reports what it
could not draw (a part with no kept STEP, plan bytes missing, a stale plan).
Caveats: named views frame the subject, so the arm can be cropped (`home`
shows the cell); classic welds draw as the row path; the rails DOM overlay,
edge overlays and the J6/TCP panes are not drawn; `row:` poses use the taught
joints or the first planned sample of that row (no IK).

## Verifying it: `rosie program check`

One pass/fail report, absolute units: OLP's document problems and row
errors; the plan is current; the program's robot matches the selected cell;
the part was placed (a part still at the import origin is flagged); the kept
trajectory passes every dense-daemon ingest rule and no joint exceeds its
speed ceiling; every MoveL's planned TCP stays within 0.5 mm of its line; no
move's peak TCP speed exceeds its commanded speed by more than 5 % (+1 mm/s).
The 0.5 mm and 5 % bounds are unverified policy (`src/cli/commands/view.ts`).

## Running it: `rosie run`

The operator's Run program press as a two-step conversation: `rosie run --sim`
answers OLP's confirmation (the action and every joint's from/to) and a token,
and **nothing moves**; only after the user agrees, `rosie run --sim --confirm
<token>` sends the same plan with OLP's joints list unchanged (OLP refuses
`confirmation_stale` if a joint moved > 0.1°), then watches until the program
ends. `--sim` refuses unless the selected cell is a simulation, whatever the
target allows. After every successful motion command a detached keepalive
(`rosie robot keepalive`) heartbeats the session for the target's window
(default 120 s, unverified) from the last motion command; `rosie robot
stop|disarm` ends it, and after the window OLP's 5 s deadman stops the robot
unless an operator's client keeps it alive. Jogs (`robot move|step|hold`) are
bounded per call (30° per joint, 100 mm / 15° per step, holds ≤ 3 s at ≤ 0.25;
`ROSIE_JOG_*` to change, unverified policy). **Never offered:** Home (HM35),
fault reset, recover-to-zero, open-ended jogging, and every deploy command
that builds, installs, restarts, locks, rolls back or reboots.

## Commands

| Command | What |
| --- | --- |
| `rosie doctor` | Check every service and the known setup traps in one report |
| `rosie status` | Who has the robot, SIMULATION or REAL, armed, session, maintenance — one screen |
| `rosie target list` | List the named targets (local stack, cell tunnels) and the default |
| `rosie target show` | Show the resolved target: every endpoint and the robot-action mode in effect |
| `rosie target add` | Add or update a named target |
| `rosie target use` | Make a target the default |
| `rosie target remove` | Remove a named target |
| `rosie cell list` | The committed cell manifests (this checkout) and the cells OLP can select |
| `rosie cell show` | One committed manifest: summary, or a section in full |
| `rosie cell diagnose` | Read every layer and say what state the cell is in, why, and what the operator does next |
| `rosie cell alarm` | Explain an A6-EC drive code and whether Arm/Reset may clear it |
| `rosie cell events` | rt-core's event log (grant, faults, jog ends, Home epochs), oldest first |
| `rosie cell describe` | rt-core Describe: backend, configuration pin, axis limits (compare with the manifest) |
| `rosie cell discover` | Find OLP servers on the network (mDNS and the tailnet), as the browser's Search local network |
| `rosie docs search` | Search the component READMEs and contracts (measured numbers, owner rules) |
| `rosie docs show` | Print one document, or the section under a heading |
| `rosie deploy inspect` | Run one read-only deploy command for a cell (plan, precheck, verify, olp status, config dry-run, maintenance) |
| `rosie deploy review` | What a release of <commit> would do to a cell, and what the owner must approve (read-only) |
| `rosie program new` | Start a new program (rails authoring on) for the connected cell's robot or --model |
| `rosie program open` | Open a program from the cell's store (by id) or from an exported JSON file |
| `rosie program list` | The programs in the cell's store, and this machine's workspaces |
| `rosie program show` | The program as OLP's spreadsheet: rows, part, fixtures, plan state, problems |
| `rosie program row` | Every field of one row: path, value, unit, range, options (the paths edit takes) |
| `rosie program edit` | Set fields of a row by OLP path, in display units (mm, °, %, s) |
| `rosie program add-waypoint` | Add a taught MoveJ/MoveL/MoveC waypoint from joint angles (OLP's FK gives the TCP) |
| `rosie program reteach` | Replace a waypoint's target with new joint angles, or set its MoveC via pose |
| `rosie program record` | Teach a waypoint from the selected cell's measured pose (jog there first; moves nothing) |
| `rosie program robot` | The program's robot settings vs the cell: show, pull the cell's calibration, adopt the current description |
| `rosie program add-event` | Add a dwell (0.5 s), IO (revpi:1 ON) or Home row |
| `rosie program remove` | Delete a row (OLP refuses rows that move with their weld) |
| `rosie program move` | Move a row up (negative) or down by N places |
| `rosie program enable` | Enable a row again |
| `rosie program disable` | Disable a row (kept, skipped by planning) |
| `rosie program speed` | Plan speed: 1-100 % of the planner's whole trajectory (dwells stay in seconds) |
| `rosie program undo` | Undo program edits (OLP's restoreAsRevision; survives across commands) |
| `rosie program redo` | Redo undone edits |
| `rosie program save` | Save to the cell's store (revision-checked) or --file |
| `rosie program plan` | Plan through OLP's /weld-plan (seconds for taught moves, minutes for welds on GPU) |
| `rosie program preview` | The planned TCP path per segment: length, chord deviation (≈0 for MoveL), peak TCP speed |
| `rosie program export` | Write the .weldplan container exactly as a plan would pack it (or --json for OLP's Export JSON) |
| `rosie program workspaces` | This machine's workspaces (each holds one working program); --ws <name> selects one |
| `rosie part import` | Import a STEP part (centred on the work frame) and detect its weld seams |
| `rosie part place` | Place the active part in the work frame: base centre at --offset (mm), turned by --rot (°) |
| `rosie part list` | The program's parts: which is active, welds, placement, whether its STEP is kept here |
| `rosie part switch` | Make another part active (edited and planned); the previous one becomes an obstacle |
| `rosie part remove` | Delete the ACTIVE part with its seams and welds (one undo step) |
| `rosie fixture list` | Everything a plan clears besides the robot and active part: stand, STEP fixtures, other parts |
| `rosie fixture add-stand` | Add a 500 × 500 × 1000 mm box stand at 1000, 1000, 0 mm (if none) |
| `rosie fixture edit` | Set a fixture's size (sx sy sz, 1-5000 mm), position (x y z, ±5000 mm) or rotation (rx ry rz, ±360°) |
| `rosie fixture step` | Add a STEP file as a fixture/obstacle (placed by its own origin; move it with fixture edit) |
| `rosie fixture replace` | Replace a fixture's shape with a STEP body (pose and id kept) |
| `rosie fixture default` | Put the cell's placeholder back |
| `rosie fixture remove` | Remove a fixture from collision checking |
| `rosie seam list` | The part's detected weld seams: type, length, weld rows, search verdict |
| `rosie seam search` | Search torch angles on a seam (or --all): clear, partial (clear runs) or blocked |
| `rosie seam weld` | Accept a searched seam as weld rows (or --clear: every seam searched clear) |
| `rosie rails show` | The fitted torch and the angle search (workspace session state, as the browser's windows) |
| `rosie rails set` | Change the torch / search (preset and/or values); drops searched seams, keeps accepted welds |
| `rosie rails reset` | Back to the shipped torch and search presets |
| `rosie weld profile` | A rails weld's work or travel angle: --constant ° or --points s:deg,s:deg,... |
| `rosie weld add` | Add a waypoint / line / arc weld from torch poses in the PART frame (mm, °) |
| `rosie weld edit` | One edit of a waypoint/line/arc weld's poses (points indexed from 0) |
| `rosie program check` | Is this program right? One pass/fail report: problems, plan current, model, trajectory rules, MoveL straightness, speeds, placement |
| `rosie view scene` | PNG snapshots of the program's 3D scene from OLP's own renderer: robot, part, fixtures, welds, waypoints, planned path |
| `rosie report program` | A self-contained HTML report: verdict, 3D views, checks, spreadsheet, planned path — for a person to review |
| `rosie report cell` | A self-contained HTML page of the cell's state: control, maintenance, machine status, rt-core faults |
| `rosie robot stop` | STOP the selected cell through OLP (always available, any client, any gate); ends the keepalive |
| `rosie robot disarm` | Disarm (passes every gate, any client); ends the keepalive |
| `rosie robot pause` | Pause the playing program (a hold on the path, not Stop) |
| `rosie robot finish` | Finish cycle: a running loop ends at the program start after this cycle |
| `rosie robot select` | Connect OLP to a commissioned cell ('' disconnects); stops the previous cell's session |
| `rosie robot arm` | Arm the selected cell (resets only drive-profile-resettable faults; never homes; moves nothing) |
| `rosie robot load` | Load the workspace program's plan (or --trajectory <.rdt>) onto the selected cell; moves nothing |
| `rosie robot play` | Play the loaded session. THE ROBOT MOVES. Needs every joint at the program's first sample (else use rosie run) |
| `rosie run` | Run the planned program (Go to start, Load, Arm, Play) — two steps: preview the moves, then --confirm <token> |
| `rosie robot watch` | Follow the session until it stops playing (progress on stderr), then print the status |
| `rosie robot move` | Move joints BY (relative) or TO (absolute) angles in one synchronised move. THE ROBOT MOVES. Bounded per call |
| `rosie robot step` | One straight TCP step (x y z mm, rx ry rz °) from the held pose. THE ROBOT MOVES. Bounded per call (100 mm / 15°) |
| `rosie robot hold` | A timed hold-to-run jog (joint J3+ or Cartesian z-), ≤ 3 s. THE ROBOT MOVES. End always sent |
| `rosie robot keepalive` | Show (or --stop) the heartbeat keepalive that keeps a session alive between commands |
| `rosie traj inspect` | Validate a .rdt offline with every dense-daemon ingest rule; print its identity, segments, joint ranges and peak speeds |
| `rosie traj validate` | Send a .rdt to the dense daemon's own validate ingest (its decoder; nothing stored) |
| `rosie traj fetch` | Download a planned trajectory from the weld planner by digest, save it and inspect it |
| `rosie traj plan-weldplan` | Plan a .weldplan on the weld planner (seconds to minutes); full result to a file, summary printed |
| `rosie planner status` | The weld planner: CUDA or CPU-only, installed revision, warm worker, the plan in flight |
| `rosie kin fk` | TCP pose of a model at six joint angles (degrees), with the Cartesian jog's own kinematics |
| `rosie kin line` | Would OLP's Cartesian step accept this straight line? (same IK, joint-limit and Jacobian gates; nothing moves) |
| `rosie skills list` | The rosie agent skills in this checkout and whether every Claude Code session on this machine has them |
| `rosie skills install` | Make the rosie skills available to every Claude Code session on this machine (links into ~/.claude/skills) |
| `rosie skills uninstall` | Remove the rosie skills from ~/.claude/skills (only entries rosie made) |

## Architecture

`src/lib` is the RosieOS client library, shared with the MCP server
(`mcp/v1` depends on it as `rosie-cli/lib/*.js`, its tests' fakes are
`rosie-cli/testing`): OLP/rt-control clients, the heartbeat keeper, the
`.rdt` reader, kinematics, ingest, OLP's program engine (`core/entry.ts`, OLP
and olp-core TypeScript bundled for Node), program orchestration
(`programs.ts`), robot admission and actions (`robot.ts`), bounded jogs
(`jog.ts`), planner calls, repo lookups and the status summary. Every rule —
fields, capture, placement, weld nodes, plan body, jog bounds, the
simulation-only admission — lives once, here; the CLI (`src/cli`) and the MCP
tools are front ends that render it.

## Tests

```bash
npm test        # build, then node --test (21 tests): library, workspace, robot, skills, snapshot (real Chrome; skipped without one)
npm run typecheck
```

`test/program-cli.test.ts` drives the real binary one process per command
against a fake OLP (`test/fakes.ts`): a waypoint's TCP against an independent
URDF-chain FK within 1e-6 mm, refusals exit 1, undo as a new revision, seams
kept across processes, the cell revision carried between saves, a 409 never
overwritten. `test/robot-cli.test.ts`: zero POSTs with actions off, a real
robot under `simulation`, `--sim` on a real robot; heartbeats continue after
the command exits and end on Stop; a run is a preview until `--confirm`
carries OLP's joints unchanged. `test/snapshot.test.ts` renders a program
with a dense plan in real Chrome and checks scene counts, part bounds (mm),
plan poses (rad) and the PNGs. `test/skills-cli.test.ts`: links point into
this checkout, re-install is a no-op, a same-named skill rosie did not make is
refused untouched, copies are marked and reported stale, uninstall leaves
other skills. The library's own tests (`rdt`, `ingest`, `heartbeat`) moved
here from `mcp/v1`. Mutations of the session restore, `--sim`, the keepalive
spawn, the jog Stop rule and the skills' foreign-entry refusal each fail a
test.

## Limits

- No robot action, snapshot or plan from this CLI has run against a live
  cell or the CUDA planner yet; tests use fakes at the HTTP boundary and a
  real Chrome. The first live run is the owner's.
- The keepalive window, jog bounds, check thresholds and confirmation TTL
  (10 min) are unverified policy.
- `rosie cell discover` and `rosie deploy` shell out to Go (`go run`); the
  deploy commands are read-only by construction.
