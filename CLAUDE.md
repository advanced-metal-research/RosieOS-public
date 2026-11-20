# RosieOS

For qualification work, read [the qualification pipeline](docs/QUALIFICATION-PIPELINE.md).
Run `node quality/qualification.mjs plan --base origin/main --head HEAD`.
Check the classes, modifiers, claims, and missing scenarios.
If semantic impact extends beyond the path rules, broaden the local plan with
`--add-class` or `--add-modifier`. Update the reviewed rule for future changes.
Never reduce required evidence through PR metadata or claim qualification manually.
When a test changes, update its adjacent `.scenarios.json` declaration.
Convert an important reproducible escaped failure into a deterministic scenario.
Keep robot descriptions, machine configs, and cell configs in their existing owners.
The qualification policy is in shadow mode; review the generated PR plan.

Before work on the nine-motor bench, check the Fleet Board hold and the
selected cell's OLP control view. If another run owns the bench, coordinate
with its owner before a deploy or test. If either status is unavailable,
confirm availability with the bench owner. Use the existing cell lock for
host-heavy work. Direct SSH is not an override of a reservation. The busy
signal is cooperative; it does not prove that host files stayed unchanged.

Robot control stack: offline programming (OLP), the weld planner, the Robot V4
motion servers, and the native `rt-core` runtime. The deploy Go module lives under
`steamdeck/real/v4`; the Steam Deck pendant is `steamdeck/real/v5`. Published
release tarballs preserve rollback for installed v4 cells; branch source is native
only.

Each `*/v1` directory is its own component with its own README. Read the
component README before changing anything in it -- most of them carry measured
numbers next to the constants, and those numbers are the reason the constant is
what it is.

Site notes for the real cells (hosts, manifests, what runs where since when,
per-cell values and standing exceptions, organised under this file's
headings) exist only in the private repository and are imported here:

@docs/site/CLAUDE.md

## Deploying and testing on a robot cell

Use only the repo commands. They are manifest-driven and run from
`steamdeck/real/v4` on the Mac: `go run . <command> --manifest
rt-core/config/cells/<cell>.json ...`.
How to use them -- tiers, fast and full tests, gate reuse, resume, rollback,
troubleshooting, measured timings -- is
[`docs/DEPLOY_AND_TEST_WORKFLOW.md`](docs/DEPLOY_AND_TEST_WORKFLOW.md), the
canonical guide. Each step's detail and refusals: `steamdeck/real/v4/README.md`
"Deployment tiers", "Cell lock" and "Adding a new robot cell". Every command
has `--dry-run` (contacts nothing; `realtime-core config --dry-run` is a
read-only probe of both hosts); `go run . realtime-core|olp|pendant --help`
lists the commands and `<command> --help` a command's flags.

- `realtime-core release --commit <sha> [--plan|--dry-run] [--tier
  config|olp|full] [--test fast|full] [--regate]` reads what the cell runs,
  picks the smallest tier and runs its steps under one cell lock, with
  automatic rollback. Run `--plan` first. Run again for the same commit, it
  resumes a stopped full release and reuses a passed test gate (`--regate`
  runs it anyway). A forced smaller tier is refused when it cannot carry the
  change; `--tier olp` is accepted only while the configuration pin and the
  capabilities digest are unchanged, and lists the rt-core changes it leaves
  undeployed. The fast gate runs only the suites whose inputs changed since
  that suite's last pass recorded on this Mac for the cell (an `olp` or
  `config` tier ignores servo-only inputs); `--plan` prints why each suite
  runs or is skipped.
- The steps, also run alone: `realtime-core config` (configuration-only,
  ~35 s on a Rosie 1430 cell); `olp install|rollback|status` (with
  `cell_deploy.olp.planner` they also install, roll back and report the weld planner unit for the same tree: env built by the
  release's build with pinned pixi, health must answer with the tree's
  commit; never edit the planner unit by hand); `olp ui
 start|status|logs|restart|stop` (this Mac's browser UI against the cell: a
 self-healing tunnel plus Vite on :5189 under launchd; never a hand-made
 tunnel); `realtime-core build`,
  `precheck`, `copy-dumps`, `deploy --package`, `start`, `verify`, `restart`,
  `rollback`, `prune`; `wait-idle`, `timings`, `lock`, `maintenance set|clear|show`;
  `reboot --owner-approved` then `resume-after-reboot`; `pendant
  build|install|rollback|status` for the Steam Deck (separate approval) and
  `pendant tunnel install|status|logs|restart|stop|rollback --host <deck>`
  (the Deck's self-healing tunnel to the cell's OLP under systemd --user, the
  `olp ui` supervisor, from the Deck's `pendant_deploy.decks[].tunnel`
  block; never a hand-made unit).
- `realtime-core restart --reason <text> [--clear-drive-alarms]` is the one
  way to restart a running controller on an unchanged release and
  configuration (it replaces the hand `lock --on servo -- systemctl
  stop ...` + `start` sequences): cell lock, copy-dumps, precheck-like
  refusals, clean systemctl stop (Result=success or nothing starts), start
  on the INSTALLED pin (never needs the manifest's), verify with Home before
  and after (never re-homes), alarms before and after. Any drive alarm
  refuses unless the owner, having inspected its cause, passes
  `--clear-drive-alarms`, and even then only alarms the drive profile's
  `fault_reset` list names resettable; `verify --expect-installed` checks
  the `control.env` pin (DEPLOY_AND_TEST_WORKFLOW.md "Restarting the
  controller").
- OLP cell discovery (Cells -> Search local network) is per cell manifest:
  `cell_deploy.olp.discovery_interfaces` (mDNS `_rosie-olp._tcp` on the named
  operator-LAN interfaces only, never the servo link) and
  `cell_deploy.olp.discovery_tailnet {"listen_port": 8798}` (an
  advertisement-only listener on the host's tailnet address; the API stays on
  loopback). `olp install` writes them into the OLP unit
  (`offline-programming/v1/README.md` "Finding cells on the network").
- The cell's program store (`cell_deploy.olp.program_store {"dir", "label"}`,
  outside deployments_dir): the browser and the cell's Decks open and save the
  cell's programs on its OLP host, revision-checked (409 conflict -> reload
  theirs / keep mine as a copy; 423 while the robot has the program loaded;
  offline -> local draft). `olp install` creates, archives and checks it;
  rollback never touches it (`offline-programming/v1/README.md` "Cell program
  store"). Open program (browser ≡ menu, Deck OPEN PROGRAM) lists the cell's programs.
- Tuning a joint (per-joint commissioning data, see "Home and hardware
  failures"): `rt-core/docs/drives/a6ec/README.md` "Tuning a joint". Run the
  standard short test program, then `realtime-core copy-dumps --to mac` and
  from `rt-core` `go run ./tools/tuningreport report --dumps <dir> --manifest
  <cell> --out <dir> [--baseline <previous>/metrics.json]`
  (`rt-core/tools/tuningreport`). Its "Servo loop quality" section reads
  every joint's torque, velocity and residual up to 500 Hz (buzz episodes,
  lines and their coupling, loop band by speed); the 1-30 Hz ring analysis
  cannot see a speed loop oscillating.

- One pendant per Deck (a hidden pendant on a Deck once armed its cell with
 presses meant for the visible pendant of another cell). Every pendant
 process on a Deck reads the same controller. Only the focused one acts (B
 still Stops), `run.sh` closes any other, the app refuses arm and motion
 while its OLP reports a cell other than its launcher's
 `ROSIE_PENDANT_EXPECT_CELL`, and the OLP refuses them from a `background`
 pendant. Never start a pendant by hand beside its Steam shortcut. To switch
 cell, start the other launcher. One app per Deck, one launcher per cell:
 one cell's manifest installs the app; `pendant install` with another cell's
 manifest (Deck entries with `app_owner`) only writes that cell into its
 launcher (`run-<cell>.sh`, the manifest's `launcher`).

Owner rules:

- A deploy or ops step done more than once becomes a repo command with
  checks, rollback and timing. When a command falls short on the robot, fix
  the command; never work around it by hand (no hand-made scripts, copied
  manifests or one-off edits on a host).
- Robot-specific settings live in the repository: never apply an
  uncommitted configuration to a cell. `release` and `realtime-core config`
  refuse uncommitted changes under `rt-core/config`, `robot_description` or
  the cell manifest (`config --dry-run` only warns, to print an edit's pin).
  When a cell runs a pin no commit holds they warn and list every drive value
  the commit would change, and refuse any change without
  `--accept-config-drift <running pin>` (owner's call). Bring live values
  into the repo instead (DEPLOY_AND_TEST_WORKFLOW.md, "Safety rules").
- Building, testing or updating on a cell's hosts and using its robot are
  mutually exclusive. Any build, test, package, deploy or heavy
  job on a cell's OLP host or servo host runs under the deploy commands or
  `realtime-core lock --manifest <cell> --reason ... [--on relay|servo] --
  <command>`: they wait until the robot is disarmed, unleased and idle, put
  the maintenance notice up on both hosts (arming refused from every client;
  Stop and Disarm still pass) and clear it when the job ends. Never start one
  while the robot is armed or in use. Builds and tests off the cell (Mac, CI,
  a bench) need no lock.
- The servo host runs nothing heavy. Never decode telemetry on it: a
  Stop-dump decode on a servo host stalled EtherCAT for ~1 s and invalidated
  Home. Restarting or redeploying `rosie-rt-core` deletes
  `/run/rosie-rt-core`, Stop dumps included: copy them off first
  (`copy-dumps`, throttled) and decode elsewhere. Never SIGKILL
  `rosie-rt-core`: a kill during a mailbox call leaves the EtherCAT master
  reserved until a reboot. A reboot of the servo host needs the owner's
  approval, even disarmed.
- A servo host whose deploy health gate fails a hostcheck refuses a full
  release until the host is fixed. `--accept-hostcheck <check>` passes it
  only when the owner has accepted that check for that run; accept no other
  hostcheck.
- Loops may run unattended where the cell manifest sets
  `cell_deploy.olp.unattended_loop_allowed` (OLP tier). A Run N times / Loop until stopped keeps running when its starter
  or every client disconnects or goes quiet; it is guarded by STOP from any
  client, the hardware E-stop (keep it within reach), faults, drive alarms
  and a refused reload or Play (all end it at once, never retried); any
  connected client may Finish cycle; a maintenance notice (`maintenance
  set`) ends it at the next cycle boundary; every cycle is journaled with the
  client that started it (`offline-programming/v1/README.md` "Loop mode").
  The deploy lock waits for idle and never stops a loop: Finish cycle or
  STOP it first.

## Running offline programming locally

`./dev-stack.sh start|stop|restart|status|logs` at the repo root supports only
`DEV_STACK_BACKEND=rt_core`. It runs the program catalog (:8787), NATS
(:14222, monitor :18222), `rt-sim`, `rt-control`, the native
`joint_trajectory_daemon` (TCP ingest :8797), and the planner, OLP API and UI below. `motion-server/v1/local-rt-core.sh`
compiles the simulation configuration and supplies the public control/jog
socket and identity bindings. rt-core binaries and the helper's generated
configuration/bindings default to `rt-core/build`. Build the dense daemon with
`make -C motion-server/joint-trajectory/v1 all`; dev-stack reads its output directory
from that Makefile's `print-bin-dir` target (default `rt-core/build/dense`). The Cartesian Makefile defaults
to `ROSIE_HOME/motion-server/v1/bin` (`ROSIE_HOME` defaults to `~/.rosie`).
`status` reports the saved backend and socket paths. Stack pid files and logs
live under `ROSIE_DEV_STACK_DIR`, whose default is a user-specific directory
under `XDG_RUNTIME_DIR` or `TMPDIR`.

In WSL set `TMPDIR` to a Linux filesystem, for example
`/dev/shm/rt-core/olp`; Unix sockets cannot bind on `/mnt/c`. Build with
`make -C rt-core control sim` and the two motion-server Makefiles. The OLP
launcher defaults both execution and local jog to `rt_core`, reuses the stack's
`ROSIE_LOCAL_RT_BUILD/binding.env`, or owns a local simulator and adapter when
no stack binding is selected. When either backend uses `rt_core`, startup is
refused if the adapter selected by an existing
`ROSIE_LOCAL_RT_BUILD/binding.env` fails its Describe probe, or, when
no such binding file is used, an explicit `ROSIE_RT_CONTROL_SOCKET` fails that
probe. An explicit build directory with no `binding.env` still falls back to
the standalone simulator when no explicit control socket is set.
Local Cartesian jog is simulation-only; dense execution uses the public Go
SDK. Stop local jog before acquiring dense execution: the adapter admits one controller at a time.

Independent MuJoCo model and physics tools remain under `mujoco-sim/v1`.
Current launchers refuse obsolete controller selections. Repository retirement
does not change installed releases.

Three processes, on three ports. **The launcher starts one of them.**
Everything below is the whole set; see `offline-programming/v1/README.md`.

```bash
# 1. weld planner motion (CUDA)                  -> 0.0.0.0:8796
cd weld_planner/v1 && pixi run -e motion motion-serve

# 2. OLP API + Tesseract sim + the seam worker   -> 127.0.0.1:8794
export OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=http://127.0.0.1:8796
bash offline-programming/v1/start-offline-programming.sh

# 3. the app itself                              -> 127.0.0.1:5189
cd offline-programming/v1/ui && npm run dev
```

There used to be a fourth, the weld planner's authoring server on `:8795`, and
a proxy in front of it. D24 deleted both: rails authoring is served from inside
the OLP now, by `internal/seam` running `seam_worker` as one subprocess per
request out of `weld_planner/v1`. What can be missing is that pixi environment
rather than another server.

Open **`http://127.0.0.1:5189/offline-programming/v1/ui/`** -- port 5189, not
8794. 8794 is the API.

Three traps, all of which have cost real debugging time:

- **`:8794` in a browser bounces to `:5189`.** `ui/dist/index.html` is a
  deliberate redirect stub, not a broken build. Do not "fix" it by running
  `npm run build` there -- `dist/` is gitignored, so the damage is invisible to
  `git status`.
- **`api health ok` does not mean rails works.** The seam worker needs
  `pixi install -e default` in `weld_planner/v1`; without it every rails action
  ("Define welds from edges") fails in the UI while the API reports healthy.
  And planning needs `OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN`, which
  the launcher does not set. It prints a `rails:` line and a `motion:` line
  either way -- read them.
- **The browser's own copy is per browser ORIGIN; the shared one is the
  cell's.** A server started with `--program-store` (a deployed cell names it
  in `cell_deploy.olp.program_store`) keeps the cell's programs, and the
  browser and the cell's Decks open and save them there, revision-checked
  (offline-programming/v1/README.md "Cell program store"; header pill
  "synced with <label> · rev N" vs "local only"). The
  browser still keeps every program in its origin's IndexedDB as its offline
  copy and draft: a program never uploaded to the cell (all programs authored
  before the store, and every program on a server without one) lives only
  there, invisible from `:8794` or another origin, and "Clear site data"
  destroys it and any unsaved draft. Never suggest clearing site data as a
  debugging step. STEP part bytes are never shared (per browser).

Rails authoring is program state chosen at **New program** (a checkbox), not a
session toggle. An old project does not gain rails by restarting the server.

## Toolchains

None of these are on `PATH` by default; they live under `~/.rosie`
(`~/.rosie/bin` also holds the `rosie` CLI, installed by `cli/v1/install.sh`;
it does not install toolchains):

```bash
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
```

`~/.rosie/bin/pixi` is the project's pixi. Install a component's environment
with `pixi install -e <env>` from that component's directory.

## Tests

Python components are pixi projects. Run them **through pixi** -- the manifest
sets `PYTHONPATH` as an assignment specifically so a system `PYTHONPATH` cannot
reach the environment:

```bash
cd weld_planner/v1
pixi run -e motion pytest -q tests/motion_planner
```

Do not try to assemble the environment by hand with `pip`. This host has
`/opt/ros`- and `/opt/openrobots`-style trees whose pytest plugins and
numpy-ABI-pinned extensions (`coal`, `eigenpy`) break the run in ways that look
like code failures. `pixi.toml` says as much where it sets `PYTHONPATH`.

`pixi run motion-test` is ambiguous across the default/motion/bench
environments -- pass `-e motion`.

Some motion tests are order-dependent (`weld_planner/v1/docs/issues/0012`): a
single-file run and a full-suite run can disagree. Compare like with like
before calling something a regression, and stash your changes to get a real
baseline.

## Home and hardware failures

- Home is established only by drive homing (HM35); the drive owns Home (607C).
 rt-core records the HM35 position as a v3 anchor, a check and backup that is
 restored after a power cycle only through direct drive checks,
 `RestoreAnchor`, or verified-delta for v3. There is no software current-pose
 zero; a `rosie-anchor-v5` file is refused, not restored.
- Software must never route around a drive or hardware failure (for example a
 drive that fails HM35). Surface the failure and fix it at the drive; do not
 add a capture, override, widened tolerance or alternate reference path.
- Never re-home to clear a fault. HM35 makes the current position Home, so it
 is run only by an operator at the joint's mechanical home pose, never as a
 recovery shortcut. A fault that leaves encoder coordinates trusted (for
 example target_lead, AMR-720) must not invalidate Home; it resets once its
 condition clears. Only genuine reference loss (coordinate_reference,
 untrusted evidence, encoder or battery alarm) may require a new HM35.
- Every A6-EC axis uses absolute linear mode
 (`C00.07 = 1`) with the motor-to-axis gear ratio applied in software and
 standard `0x6091:01/02 = 1/1`. The A6-EC HM35 transaction writes
 `607C = 0`; its other vendor-confirmed writes and ordering stay fixed. Do not
 add dwells, preconditioning stages or per-drive variants.
- Rotation mode, wrapped feedback, wrapped commands, batteryless single-turn
 mode 3 and the public `feedback_wrap`, `command_wrap` and
 `anchor_period_counts` fields are retired. Coordinate evidence, retained
 Home and verified-delta compare complete signed counts exactly; a whole-turn
 disagreement must refuse.
- The linear-mode migration stays on its branch until the single-axis mini
 bench passes the servo-rate qualification and the target cell is ready. The
 mode change
 needs a drive power cycle and invalidates old coordinate evidence; after an
 approved full release each migrated cell needs operator HM35 on every joint.
- Nothing is joint-specific. Joints differ only by gear ratio. Every limit,
 tolerance, timeout and check is one joint-unit value (radians, rad/s)
 applied to all axes and converted to counts per axis by gear ratio. If one
 joint needs a longer timeout or wider tolerance, all joints get it.
 Exception (owner decision): drive servo tuning (gains, feed-
 forward, filters, notches, vibration suppression, inertia ratio, stiffness)
 is per-joint machine configuration, limited to the drive's tuning catalog
 (`rt-core/config/drives/a6ec.tuning.json`). rt-core reads it at every start,
 writes only what differs (write on change: each A6-EC SDO write costs an
 EEPROM write, ALF1.3), reads it back and it is part of the configuration
 hash. Owner exception: omitted tuning keys retain drive-held
 values; configuration-only deployment may remove keys classified as tuning
 by the new compiled catalog. This does not restore old tuning, and an older configuration
 that explicitly writes these keys manages them again. Drive auto-tuning
 may measure; this exception does not authorize automatic tuning. Safety
 bounds stay one joint-unit value for every joint.
 rt-core code stays generic for 1..n drives; tuning values are per-robot
 commissioning data set when each robot is commissioned.
- `max_target_lead` is one joint-angle value for every joint. A cell whose
 machine configuration still lists it per joint measures it on the loaded arm
 and sets one value for all joints before release; do not tune it per joint
 meanwhile (issue #158). Exceeding it is a warning, not a fault (owner
 decision); the `target_lead` journal lines are the measurement input for
 that value, and following error remains the hard tracking fault. Both bounds
 also grow with commanded speed by the machine-level `tracking_lag_allowance`
 (a margin over the measured position-loop lag below): bound = static value +
 |commanded velocity| x allowance. It is one time for every joint; do not
 widen the static bounds instead.
- The ~27 ms lag is the A6-EC position loop with speed feedforward off (factory
 C01.13/C01.14 = 0/0, C01.00 = 40 rad/s); in CSP the drive ignores 0x60FF.
 Feedforward is a per-joint tuning key. Measured on a Rosie 1430 arm: with
 1/500, J2 oscillated at 100 Hz up to 557 permille (80-500 Hz torque, 30-60
 deg/s) and J1 near the base up to 481; with 0/0 on every joint every joint
 reads 3-9 in every speed band, the measured TCP stays within 1.2 mm of the
 line at 400 mm/s, lag ~27 ms on all joints. Jog, Halt-ramp and OLP
 Move-to/Cartesian-step jerk is the one machine-level `jog_jerk`; OLP reads it
 from the cell's compiled configuration, never a second number. At startup
 rt-core observes the tuning objects a configuration does not write (64 per
 axis, the native capacity; `rt-core/docs/drives/a6ec/README.md`,
 "Tuning a joint").
- No drive-, robot- or machine-specific number may be hard-coded in rt-core
 source. Limits, tolerances, timeouts and drive transactions live only in
 config files (drive profile, robot definition, machine config) and reach the
 runtime through the rtctl compiler. Code holds only generic mechanics.
 Remaining unrelated legacy A6-EC literals are tracked in
 issue #160 (internal tracker);
 do not add new ones or fold that wider cleanup into the linear-mode migration.
- The TCP never leaves its commanded path (owner rule). Jog,
 Move-to, Cartesian steps, dense trajectories and the weld planner derive
 every joint from one shared time scaling: when any joint reaches a speed,
 acceleration or jerk limit, a singularity or a position limit, all joints
 slow together and keep their ratios, and stops stay on the path. Prove it on
 the commanded path in absolute units. The measured pose (rt-core
 `metrics.json`, 10 Hz, 35-100 ms old) only monitors; nothing steers from it.
 Cartesian holds, Move-to and Cartesian steps comply; a trajectory Halt along
 the path is open.
- A drive keeps the last startup SDO value written to it (A6-EC C13.10
 parameter storage): a configuration that omits a key does not revert it.
 Revert with explicit values, never by rolling back alone. The exception is a
 per-axis `startup_trial` key (requires machine-level
 `verify_parameter_storage`, which also verifies C13.10 = 1 at every start):
 written with C13.10 = 0, it lives in drive RAM and a drive power cycle
 reverts it (`rt-core/docs/CONTRACTS.md`, "Startup write on change").
- Deploying, testing on a cell's hosts and the servo host's rules: see
 "Deploying and testing on a robot cell" above.

## Conventions

- Constants carry their measurement in the comment above them. If you change
  one, change the measurement or say it is now unverified.
- Docstrings state the contract and the failure the code exists to prevent.
  Match that voice; do not replace it with restatements of the code.
- Tests assert the contract in absolute units. A threshold computed from the
  trajectory's own `dt` moves with the thing it is judging and passes when the
  behaviour collapses -- that exact bug hid a 3x regression in the weld
  planner's contact crawl.
- The shared devlog and agent scratchpad are `.cursor/memory/DEVLOG.md` and
  `.cursor/memory/AGENT_SCRATCHPAD.md`, tracked and append-only: mark wrong
  entries superseded, never rewrite them. Project skills live in
  `.claude/skills` (`learning-scratchpad-loop`, `devlog-practice`). Agents
  drive RosieOS with the `rosie` CLI (`cli/v1`, installed by
  `cli/v1/install.sh` as `~/.rosie/bin/rosie`, which also links the rosie
  skills into `~/.claude/skills` for every session: `rosie skills
  install|list|uninstall`): start with the `rosie` skill, which routes to
  `rosie-local-stack`, `rosie-programs`, `rosie-verify-and-view` (check a program and look at its 3D scene before
  anything runs), `rosie-robot`, `rosie-diagnose`, `rosie-trajectories` and
  `rosie-deploy-review`. The `rosieos` MCP server (`mcp/v1`) is a second
  front end over the same library (`cli/v1/src/lib`).
