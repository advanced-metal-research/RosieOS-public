# Deploy, test and tune: developer and operator workflow

How to get a commit onto a robot cell, test it at the right depth, tune a
joint, and update the Steam Deck pendant, with the repo commands that do it.
Every command here runs from `steamdeck/real/v4` on the operator Mac and is
driven by the cell manifest; none is per-cell code. Details of each step
(scripts, refusal lists, exit codes) stay in
[`steamdeck/real/v4/README.md`](../steamdeck/real/v4/README.md#deployment-tiers);
this page is how to use them. Measured times below come from a Rosie 1430
cell. Site notes for the real cells: `docs/site/deploy.md` (private
repository only).

```sh
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
cd steamdeck/real/v4
CELL=rt-core/config/cells/<cell>.json
```

Rule: a deploy or ops step done more than once is a repo
command with checks, rollback and timing. When a command falls short on the
robot, fix the command; never work around it by hand.

`go run . realtime-core --help`, `go run . olp --help` and `go run . pendant
--help` list the commands; `go run . <group> <command> --help` lists one
command's flags.

## Safety rules that every command keeps

- The robot is disarmed with no lease before anything that stops or restarts
  it. Commands wait for that (they never disarm for you) and refuse otherwise.
- Building, testing or updating on a cell's hosts and using its robot are
  mutually exclusive: commands hold the **cell lock** and put up the
  **maintenance notice** (below). Builds and tests off the cell (the Mac, CI)
  need no lock.
- Stop dumps are copied off the servo host before any rt-core restart (a
  restart deletes `/run/rosie-rt-core`, dumps included), throttled, and decoded
  on the Mac, never on the servo host (a decode there stalled EtherCAT for
  ~1 s and invalidated Home).
- Never re-home to clear a fault or a Home that did not come back after a
  restart; report it. Home is set only by an operator's HM35 at the joint's
  mechanical home pose.
- Never SIGKILL `rosie-rt-core`; a stop that hangs is read from
  `/proc/<pid>/task/*/stack` and taken to the owner. A servo host reboot needs
  the owner's approval, even disarmed.
- A drive keeps the last value written to it: revert a drive parameter with an
  explicit value, never by rolling back alone.
- Robot-specific settings live in the repository; a cell never runs a
  configuration git does not hold. `release` and
  `realtime-core config` refuse to apply from a working tree with uncommitted
  changes (tracked or untracked) under `rt-core/config`, `robot_description`
  or the cell manifest, and `config` applies only a committed configuration
  (machine JSON equal to HEAD's, compiling to HEAD's pin). `config --dry-run`
  still probes an edit and prints the pin it compiles to, with a warning.
  When the cell already runs a configuration no commit on the branch pins
  (`git log -S` over `rt-core/config/cells`), `release`, `release --plan` and
  `config` print a WARNING and every startup (drive) value the commit would
  change, running -> commit, from the running compiled `axes.conf` and the
  commit's compile; any such change is refused unless the owner names the
  running pin with `--accept-config-drift <running sha256>`. Why: a cell
  once got speed feedforward 0/0 from an uncommitted edit while the
  repository pinned 1/500, and the next release would have written 1/500
  back into every drive's EEPROM without a word.

## The tiers

| Tier | Carries | Restarts | Typical time (a Rosie 1430 cell) |
|---|---|---|---|
| config | values in the cell's machine config that the **installed** rtctl understands (a drive startup SDO such as a per-joint tuning key, a limit, `tracking_lag_allowance`) | `rosie-rt-core`, `rosie-rt-control`, OLP pin | 35.5-36.9 s per apply (three value changes on one joint: apply 31-33 s, start to verified 19-23 s) |
| olp | OLP server, Cartesian resolver, seam worker, with the rt-core protocol and capabilities digest unchanged | the cell's OLP unit (~1 s outage) | 5.5-7.2 s install (four releases). A whole `release --tier olp` took 17m24s (build 2m02s, gate 15m08s) and 14m03s (build 21 s, gate 13m19s), before the fast gate skipped suites a recorded pass covers; with it, an OLP-only change estimates ~1.5 min (dry-run selection, not yet run) |
| full | anything the servo host's package carries: rt-core native code, protocol, rtctl, drive profiles, robot descriptions, host scripts | both rt-core units, then the OLP | first live full release: full gate ~11.5 min (on lanes, before real-time suites ran alone; reused on a retry), then copy-dumps and precheck ~1 s each, deploy 10 s, start 25 s, verify 1 s, olp install 6 s. First full `release` end to end (`--test fast`): 24m44s, of which 9m22s waiting for the lock behind a stalled release, build and gate 13m56s, deploy 35.7 s, start 22.4 s, verify 7.8 s, olp install 6.6 s |
| pendant | Steam Deck v5 app, its bundled `olp-core.js`, meshes; required by any capabilities digest change | the pendant on one Deck | build 218-225 s from a fresh work tree / 23-24 s incremental (Mac); 54.9 s incremental with the builder image rebuilt from its layer cache; install 5.3 s (app not running) |

The browser UI is not a tier: operators run it from their own checkout
(`offline-programming/v1/ui`, Vite on `:5189`); the relay serves an API-only
stub. Pull the commit; Vite reloads it (`olp ui restart` after a
`package-lock.json` change).

### The browser UI against a cell: `olp ui`

```sh
go run . olp ui start --manifest $CELL     # then open http://127.0.0.1:5189/offline-programming/v1/ui/
go run . olp ui status --manifest $CELL    # exit 1 when anything is down
go run . olp ui logs --manifest $CELL [--follow]
go run . olp ui stop --manifest $CELL
```

`start` runs two launchd agents on this Mac (macOS only), independent of the
shell or agent session that started them: an ssh tunnel
`127.0.0.1:18894 -> <relay> cell_deploy.olp.listen` (relay user and host from
the manifest, `--transport tailscale` by default) and Vite on `127.0.0.1:5189`
with its API proxy on the tunnel. A supervisor loop restarts ssh whenever it
exits (ServerAlive 5 s x 3 notices a dead link; retry 1 s after a healthy run,
doubling to 30 s) and records why in `tunnel.state`. Re-running `start` leaves
healthy processes alone, restarts a dead tunnel at once, runs `npm ci` only
when `node_modules` differs from `package-lock.json`, and refuses a port held
by something it did not start. `stop`, and a `start` that must replace a job
(other ports or transport), refuse while the OLP reports the robot armed or a
held lease (`--force` overrides): the browser's heartbeat would lapse.
`status` prints tunnel up/down with the last restart reason, Vite, OLP health
through the tunnel and through Vite, the OLP build id and whether the cell is
listed. Jobs, state and logs: `~/.rosie/olp-ui/<cell>/`; flags `--port`,
`--tunnel-port`, `--wait`.

Measured on a Rosie 1430 cell: `start` from nothing 2.0 s (tunnel healthy
0.4 s, Vite 1.6 s); the switch from the old hand setup took ~4 s of UI outage.
Killing the tunnel's ssh: OLP health back through the tunnel and Vite in
1.36-1.39 s, with an open UI tab staying connected without a reload. A
SIGKILLed supervisor: launchd respawned it, it ended the orphaned ssh and
health was back in 0.85 s.

### Cells on the network (discovery)

Cells → **Search local network** lists the cells other OLP servers advertise
(mDNS `_rosie-olp._tcp`; design and limits in
[`offline-programming/v1/README.md`](../offline-programming/v1/README.md#finding-cells-on-the-network)).
A cell host's OLP advertises and searches by mDNS only on the interfaces its
manifest names in `cell_deploy.olp.discovery_interfaces` (the operator LAN,
never the link to the servo host), and over Tailscale with
`cell_deploy.olp.discovery_tailnet: {"listen_port": 8798}`: an
advertisement-only listener on the host's tailnet address (the API stays on
loopback) and a search of tailnet peers on that port. An OLP-tier release
(`olp install`) puts them in the unit. From the Mac, in
`offline-programming/v1`: `go run ./tools/olp-discover --tailscale` lists
the tailnet's OLP servers (a cell host was found in 1.3 s; where the Wi-Fi
isolates clients, `--interface en0` hears nothing from it); `curl
http://<OLP host's MagicDNS name>:8798/api/offline-programming/v1/advertisement`
shows one record (on macOS use the full MagicDNS name).

### The cell's program store

`cell_deploy.olp.program_store {"dir", "label"}` gives the cell one program
store on its OLP host: the browser and every Deck open and save the same
programs there, revision-checked
([`offline-programming/v1/README.md`](../offline-programming/v1/README.md#cell-program-store)).
It ships with the OLP tier (`olp install` renders it into the unit, creates
the directory, archives it into `<deployment>/olp-install/program-store-pre-install.tgz`
before the swap, and restores the unit unless the restarted OLP serves it) and
the Decks' sync with a `pendant install`. The directory is outside every
deployment: no install, rollback or prune touches it. The manifest names
it, for example `/home/operator/.rosie/olp/<cell>/programs` with the OLP
host's name as the label. Migration: programs authored before it stay where they
are (each browser origin's IndexedDB, each Deck's `current.olp.json`); the
browser lists the cell's programs and offers "Upload" for the program it has
open; the Deck shows a picker of the cell's saved programs once at start (open
one, upload its own, or a new program).

## One command: `realtime-core release`

`release` reads what the cell runs (the rt-core release on the servo host,
the OLP tree's commit, the live Describe's configuration and capabilities
digest), compares it with the commit, and runs the smallest tier's steps in
order, under the cell lock throughout, with one timing table at the end:

| Tier | Steps (each is the existing command, run in process) |
|---|---|
| config | `build --test` (fast), `copy-dumps`, `config --accept-configuration <commit's pin> --olp restart`, then `olp install` if the OLP tree also changed |
| olp | `build --test` (fast), `olp install` |
| full | `build --test --fetch-package` (full), `copy-dumps`, `precheck`, `deploy --package`, `start`, `verify --expect-capabilities <commit's digest>`, `olp install --expect-capabilities <digest>`, `prune` |

How the tier is chosen (`release_tier.go`, `release_inputs.go`):

- **full** when any path the servo host's package carries changed since the
  installed rt-core release, or the capabilities digest at the commit differs
  from the running one. What the package carries is read from the build, not
  a prefix list: the native sources and generated C++ headers `make live`
  compiles, the host scripts, units, `verify-package.sh`, IgH userlib build,
  `config/` (other cells' and template configs excepted) and robot
  descriptions that `package-runtime.sh` copies, and every Go package the
  shipped binaries (rt-control, rtctl, rt-package, rt-natspublisher) import
  (`go list -deps`, GOOS=linux). An off-cell tool (`tools/tuningreport`,
  `tools/cidriver`) therefore no longer selects full. The import graph is
  read from this checkout, so it is used only when the checkout's `rt-core`,
  `robot_description` and `offline-programming/v1` equal the commit;
  otherwise every rt-core source counts (the plan says why);
- **config** when only the cell pin differs: the commit's
  `rt_core.configuration_sha256` is not the running Describe's. `config`
  itself then proves the installed rtctl understands every key (it refuses
  deployment-identity drift: re-run with `--tier full`);
- **olp** when only the relay's OLP tree changed (`offline-programming/v1`
  outside `ui/`, `motion-server/v1`, `weld_planner/v1`, `cadquery/v1`
  outside `ui/` (part import's scripts and env), and exactly the
  rt-core Go packages the OLP server imports, `tools/rtctl` included);
- nothing to do otherwise. The pendant is flagged with the reason (its own
  sources, `ui/src` `.ts` files bundled into `olp-core.js`, meshes or the
  digest) and the two commands to run; release never touches a Deck.
  `quality/deployment-surface.mjs` over the range adds advisory plan notes
  (cell manifests' declared runtime paths, units, build commands; new or
  removed command and service files).

`--tier config|olp|full` forces a tier. A smaller tier than the change needs
is refused, except `--tier olp` when the pin and digest are unchanged (it
lists the rt-core changes it leaves undeployed). `--tier full` is always
allowed.

Example: `--plan` for a commit carrying an OLP-only check plus a re-pin to
the running configuration chose **full**, although the pin and digest were
unchanged: 18 servo-package paths had changed since the installed release
(the `src/version.cpp` split, `install.sh --prune`, rtctl and ipcclient
refactors, Makefile targets). `--tier olp --plan` was accepted and listed
them. Which one to run is the owner's decision; `git log --oneline
<installed>..<commit> -- rt-core robot_description` shows such commits before
the plan does (`--plan` prints the first six paths).

### Use it

Always look first. `--dry-run` contacts nothing; `--plan` reads the cell
(read-only on the relay and servo host) and prints the tier and the steps.
Unlike every other command's, `realtime-core config --dry-run` is not
contact-free: it probes both hosts read-only and compiles the JSON with the
installed rtctl, on the servo host at nice 19 when the relay has no copy of
the installed package.

```sh
go run . realtime-core release --manifest $CELL --commit <sha> --dry-run
go run . realtime-core release --manifest $CELL --commit <sha> --plan
go run . realtime-core release --manifest $CELL --commit <sha>
```

Examples:

```sh
# OLP-only fix: build, fast tests (olp suites), olp install
go run . realtime-core release --manifest $CELL --commit <sha> --plan

# Tuning value (machine config + re-pin): build, fast tests (rtctl, cells, olp),
# copy-dumps, configuration-only apply, OLP re-pin
go run . realtime-core release --manifest $CELL --commit <tuning commit> --tier config

# Native change: the full gate, package, install, start, verify, OLP.
# A servo host whose health gate fails a required check (for example
# igh_master: its IgH kernel module older than the package's) needs the
# owner's acceptance of that one check, per run.
go run . realtime-core release --manifest $CELL --commit <sha> --tier full [--accept-hostcheck <check>]
```

What you must have:

- the commit on a local ref (the build host receives a git bundle and checks
  out exactly that commit; uncommitted work never ships);
- this checkout's cell manifest and machine config equal to the commit's (the
  pins come from them) and nothing uncommitted under `rt-core/config`,
  `robot_description` or the cell manifest, on every tier and for `--plan`;
  for `full`, a clean checkout at the commit (HEAD = the commit, no changes
  under `rt-core/` or `robot_description/`), because `deploy --package` takes
  only a package built from the checkout's clean HEAD. Use a second clone (or
  `git worktree add --detach /tmp/x <sha>`) for releases when you have work in
  progress;
- a running configuration some commit pins, or the owner's
  `--accept-config-drift <running sha256>` when the release prints drive
  values it would change on a cell running an unpinned configuration (the
  check costs ~0.2 s when the pin is known; an unknown pin adds one read of
  the running `axes.conf` and a compile of the commit with its own rtctl in a
  temporary worktree on the Mac). Bringing the running values into the repo
  is the usual answer: the relay's OLP deployment tree (`git diff` in its
  `source/`) and `/etc/rosie-rt-core/config-history/machines/<pin>.json` on
  the servo host hold what `config` applied.

Other flags: `--test fast|full` (default fast for config/olp, full for full),
`--rollback auto|never` (full tier, below), `--regate` (ignore the gate
records: run the gate even when a pass is recorded, and let fast tests diff
against the installed commits, below), `--rollback-to-configuration <sha256>`
(a resume's rollback target when this Mac has no record), `--clean`
(from-scratch build),
`--install-planner-env` (pixi install when `weld_planner` changed),
`--idle-wait 10m --quiet 2m` (how long to wait for the lock and an idle cell),
`--transport tailscale|ssh`.

Each step keeps its own refusals; `release` adds no override. Each step writes
its own transcript and timings in `~/.rosie/deploy-journal/`; the release run
has one more, whose table has one row per step.

### Retrying, resuming, a stalled start

The first live full release took five runs; each
retry re-ran the 12 min gate and the last steps were finished by hand. Now:

- **A passed gate is reused.** `build --test` records every passed gate in
  `~/.rosie/deploy-journal/gate-passes.jsonl`: cell, commit, git tree, level,
  suites, per-suite results and the gate definition digest (every suite's
  directory and command, the scheduler, the log parser). A later build of the
  same commit skips the gate when a pass ran every suite it would run now (a
  full pass covers fast) under the same definition, and prints `reused: the
  full gate of <sha8> ...`. The recorded results are classified again against
  today's known failures. Since 2026-10-08 each record also carries every
  suite's definition digest and input hash, and fast tests of any commit skip
  the suites a recorded pass already covers (below). `--regate` (build and
  release) ignores the records.
- **A stopped full release resumes.** Run the same `release` again. When the
  commit's rt-core release is already current it reads the units: both
  stopped (installed, the health gate or start failed) -> `start`, `verify`,
  then `build` (reused) and `olp install` if the OLP tree is not the commit;
  both running (started, not verified; or verified, OLP not installed) ->
  `verify`, then `olp install` if needed. One unit up and one down is refused.
  Automatic rollback still works: a full release records the configuration
  that ran before it in `~/.rosie/deploy-journal/release-state/<cell>.json`;
  without that record (another Mac) pass `--rollback-to-configuration
  <sha256>` or `--rollback never`.
- **A stalled bring-up is started once more.** `start` prints the bus
  bring-up whenever it changes (`start bring-up t=13s op=5/6 online=6
  master_al=0xa axes_op=111110 axes_al=0x8,...`, from the core's
  `metrics.json`) and a failed start ends with `start stall kind=bringup`
  (not verified in time), `kind=identity` (another configuration or
  backend) or `kind=busy` (came up armed or granted). Start already stopped
  both units; on `bringup` only, release starts once more, logs the samples
  in its journal, and rolls back only if the second start fails too. No
  other workaround: a bus that does not converge twice is reported.
- **A dead connection fails the step instead of hanging it** (an OLP-tier
  release once held a cell's lock for 64 min on a half-open fast-gate
  session whose gate had finished on the relay 17 min in). Every ssh carries keepalives (5 s x 3, ~15 s to notice a dead
  path). The build and the gate run detached on the relay
  (`~/.rosie/watched/<run>-<label>/`, output and exit status in files); a
  stream that drops or shows no line for 2 min (the relay prints a heartbeat
  every 30 s while a suite is quiet, so a slow suite is not a stall) is
  re-attached once over a fresh connection and continues from the last line
  received. A second stall with nothing new fails the step and releases the
  lock; a gate still running past its limit (full gate 74 min) is stopped on
  the relay. The lock holder gets a heartbeat from the Mac every 10 s and
  releases after 120 s without one; `release` stops before its next step if
  its holder dropped. Thresholds unverified.
- **Old releases are pruned.** After a verified full release, `prune` runs
  the installed `install.sh --prune` on the servo host: it keeps current,
  previous and pinned known-good releases (`realtime-core prune --pin <rev>`
  adds one to `/opt/rosie-rt-core/pinned`), never touches units, and prints
  what it freed. A release whose `install.sh` predates `--prune` reports
  `unsupported`; a failed prune is a warning, never a failed release.

## Restarting the controller: `realtime-core restart`

A controlled restart of what the cell already runs: the installed release
with the installed configuration (the pin `control.env` holds). It never
changes either, and it does not need the manifest's pin: a cell that runs a
configuration no commit holds restarts on that pin, and the manifest's pin
is only reported. To change the release or the configuration, use `release`
or `config` from a commit.

It replaces hand sequences that cleared a latched A6-EC Er41.1 after a
locked-rotor stall (copy-dumps, `lock --on servo -- systemctl stop ...`,
start, verify; on a cell running an unpinned configuration `start` refused
the installed pin and an agent ran start's script by hand).

```sh
go run . realtime-core restart --manifest $CELL --reason "..." --dry-run
go run . realtime-core restart --manifest $CELL --reason "owner freed J5 at the torch; Er41.1 cause inspected" --clear-drive-alarms
```

What it does, under one cell lock (idle wait, then both maintenance notices:
arming refused from every client until it ends):

1. `copy-dumps` to the relay (the restart deletes `/run/rosie-rt-core` and
   its Stop dumps).
2. Read-only facts (the precheck script, which also records the anchor files
   for verify), then the refusals: armed, enabled or leased; an OLP session
   or operator activity in the quiet window; a dump without a verified copy;
   rosie threads stuck in D state; one unit running and the other not; a
   controller running a configuration other than `control.env`'s (a
   half-applied change: use `config` or `rollback`); and any fault or drive
   alarm, see below.
3. One servo-host session under the configuration deploy's flock, which
   ignores hangups: the same checks again just before the stop, `systemctl
   stop` of `rosie-rt-control` then `rosie-rt-core` (the units' SIGTERM drain;
   never a signal by hand), then both must read `ActiveState=inactive
   Result=success`. `Result=timeout` means systemd SIGKILLed a drain that
   outlived `TimeoutStopSec` (15 s): nothing is started (exit 6), take it to
   the owner (a kill during a mailbox call leaves the EtherCAT master reserved
   until a reboot). Then `realtime-core start`'s own script with the installed
   pin: it still refuses another pair, pair revision, or a release that fails
   `rt-package verify`, and stops both units again if the bus does not verify
   in 60 s. Every step is a line in the servo host's
   `/etc/rosie-rt-core/config-history/journal.jsonl` with the reason.
4. Verify against the configuration that ran before and the Home that was
   valid before (`--home-wait`, default 45 s). Restart never homes: an axis
   whose Home does not come back is named (`J5 (axis 4): Home NOT restored`)
   and the run fails. Report it; never re-home or restore anchors.

It prints faults and drive alarms before and after (`before ... drive alarms:
J5 axis 4 Er41.1 0x7121/0x0411 Motor over-temperature ...`, `after ... drive
alarms: none`), the timing table and the journal path
(`~/.rosie/deploy-journal/<run>.log`).

**Drive alarms.** A restart can clear a latched drive alarm at bring-up,
which would hide a hardware failure (CLAUDE.md: never route around one). So:

- any drive alarm refuses unless the owner passes `--clear-drive-alarms`,
  which records that the owner inspected and fixed the cause (the alarms
  present are printed as owner-approved);
- even then, only alarms that the axis's drive profile lists in
  `fault_reset.resettable_alarms` by their exact 0x603F/0x203F pair (the list
  the operator Reset uses; Er41.1 is `0x7121`/`0x0411`). An encoder,
  overcurrent or EEPROM alarm, or one whose 0x203F detail is unread, refuses:
  power-cycle and service the drive as its manual says;
- a fault no resettable alarm explains (anything beyond the alarm's axis bit
  in `safety_fault_mask` and `drive_readiness` in `execution_fault_reasons`)
  always refuses: restart is not a fault-clearing path; the operator Reset
  clears latched faults and never re-homes;
- an alarm still present after the restart fails the run: its cause persists.

**Faults that need a new Home** (`--clear-rehome-faults`, 2026-11-13). A
fault whose recovery re-homes every joint it latched (for example
`coordinate_reference`, and a `drive_readiness` that carries the same
re-home mask) can leave no way out: Reset refuses ("native recovery
requires Home or restart"), the OLP's Home recovery accepts
`coordinate_reference` only alone, and restart refuses it as above. A
bulk SDO scan on a running servo host latched both on all six joints
(issues #177 and #182 describe the same trap).
With `--clear-rehome-faults` the owner states the cause is found, and
restart accepts exactly those faults, only on joints whose Home is already
lost and with no drive alarm present; any other fault still refuses.
Startup re-checks the saved anchors; an operator homes every joint they do
not restore (the done line names them).

The flag also accepts the **stale latch** (`b7c3c9bc`): Home still valid
and every affected joint's position trusted, audit-verified and
coordinate-valid, with `drive_readiness` or `bus_transport` latched on the
same joints. One lost EtherCAT frame leaves exactly that (on an unpinned
RevPi kernel): the latch records the joints untrusted
and never forgets it, although every joint re-verifies, so Arm, Reset and
the OLP's Home all refuse. The restart must then bring that Home back from
the saved anchors, or it fails: a real reference loss is not hidden. On
the cell where this was found it restored all six in 3 s with no fault. Find why the frame was lost
before you use it (`journalctl -k` "datagrams UNMATCHED"; on a RevPi, the
kernel that `rt-core/host/ethercat-foundation.sh kernel` pins).
Never run `ethercat upload` or
`download` in a loop on a servo host while `rosie-rt-core` runs: its own
coordinate sampling shares the mailbox.

**When the start fails** (`START FAILED`, exit 1), both units are stopped,
disarmed, no bus owner. Fix the cause in the printed lines and run `restart`
again: with both units stopped it only starts them (it cannot read the
alarms from before that stop; the report after the start shows them).

`verify` alone takes `--expect-installed` to verify against the `control.env`
pin instead of the manifest's, for the same cells.

## Tests: fast and full

The gate runs on the build host (the relay) in the persistent build tree,
after the build, several suites at a time (lanes = half the host's cores),
longest chain first. Suites that assert real-time behaviour (`alone` below:
clocks, deadlines, daemons they start) run with nothing else running: they
start once every lane is free and nothing starts while they run. Run 5 of the
first live full release failed `sdk_path_jog` `jog_clock_uncertainty_exceeded` on
lanes, on a commit whose gate had passed three times.

| Suite | Directory | Measured (a Rosie 1430 cell's relay, one at a time, nice 15) |
|---|---|---|
| `make_test` | `rt-core` (`make -j test-runs`: every build, the protocol and generated-API checks, the C++ contract binaries in parallel) | 686 s for all of `make test`; ~334 s of it |
| `make_test_serial` (alone) | `rt-core` (`make test-serial`: apigen C++ interop, Go `-race` oracle, host scripts, one at a time) | ~352 s of the 686 s |
| `rtctl` (alone) | `rt-core` (`go test ./tools/rtctl/...`; its daemon tests miss cycle deadlines on a loaded host) | 347 s (before `8185f5ab`, which cut it from 130-150 s to 76 s in the Mac's arm64 container) |
| `motion_test` | `motion-server/v1` | 345 s |
| `ui_test` (after `ui_npm_ci`) | `offline-programming/v1/ui` | 308 s (+6 s) |
| `olp` | `offline-programming/v1` (`go test ./internal/...`) | 39 s |
| `faultmatrix_shutdown`, `sdk_path_jog` (alone) | `rt-core` (after `make_test`) | 22 s, 10 s |
| `olp_composed` (alone) | Cartesian jog composed with rt-core sim and the built resolver | 6 s |
| `pendant_olp_core` | the pendant's bundled OLP logic under node | unmeasured |
| `cells` | `rt-core` (`go test ./config/cells/...`) | 1 s |

- `--test full` runs every suite. One at a time they took 1770 s (29.5 min).
  On three lanes the catalogue estimate is 1417 s, lower bound 1081 s (737 s
  of alone suites with every lane held, 1033 s of lane work around them); it
  was 724 s while `make test`'s real-time tail, `rtctl` and the path-jog
  tests shared lanes with heavy suites, which is what made them flake. The
  catalogue's seconds predate tonight's speed-ups (lifecycle-test 106 s ->
  11 s, rtctl 2x); the journal's measured timings replace them after the
  first gate. A commit whose Makefile predates `test-serial` runs
  plain `make test` as `make_test`. The four full gates of the first live
  full release (every suite on lanes) took 11.4-11.7 min (one failed run
  ended at 7.1 min), with mean suite times
  `motion_test` 6 m 17 s, `rtctl` 6 m 14 s, `make_test` 5 m 45 s, `ui_test`
  5 m 24 s, `olp` 47 s, `faultmatrix_shutdown` 22 s, `olp_composed` and
  `sdk_path_jog` 13 s (`realtime-core timings`; lanes slow each other).
- `--test fast` runs each suite whose inputs changed since **that suite's
  last pass** on this cell, recorded on this Mac.
  Before, it diffed against the installed commits: an OLP-tier release of
  a manifest one-liner spent 13m19s gating 11 suites because rt-core
  commits since the installed release selected `make_test`, `rtctl` and
  the rest, although the previous OLP release's gate had passed them and
  their inputs had not changed since. The rules:
  - a suite's inputs at a commit are the git tree entries its input paths
    select (documentation excluded). It is skipped when a recorded pass of
    it on this cell, under its current definition and with every failure
    still on the known-failures list, saw exactly those entries (any pass,
    not only the newest: a revert is covered by the older pass). A record
    whose commit is no longer in the checkout counts through its input hash;
  - with no covering pass it is skipped only when its inputs equal those at
    every commit the cell runs (release passes the installed rt-core and OLP
    commits; a plain `build` the previous tree's): nothing it covers ships.
    Otherwise it runs, including with no record or an unknown base;
  - an `olp` or `config` release leaves the servo package as installed, so
    servo-only inputs (native sources and tests, host scripts, protocol
    files) do not count for it; every Go file (the OLP server imports
    rt-core packages), a file beside one, `rt-core/config` and
    `rt-core/clients/ts` still do. A full release runs the full gate;
  - every run and skip line says why and names the pass it relies on:
    `olp (...): 1 input path(s) changed since its last pass at <sha8> (...)`,
    `skipped make_test: inputs unchanged since its pass at <sha8> (...); 3
    servo-only path(s) changed since, which the olp tier does not ship: ...`,
    `skipped motion_test: no recorded pass covers its inputs, but they equal
    those at <rt-core sha8>/<olp sha8> (what the cell runs): nothing it covers ships`.
    `release --plan` and `--dry-run` print the same lines from this Mac's
    records before anything runs.

  `--regate` ignores the records (fast diffs against the installed commits
  as before); `--test full` runs every suite. Examples: an OLP server change
  runs `olp` (and `olp_composed` for `denseexec`); a UI change runs
  `ui_npm_ci`, `ui_test`, `pendant_olp_core`; a machine config value or a
  cell manifest runs `rtctl`, `cells`, `olp` (`rtctl`'s tests package
  `config/`); a native header runs `make_test` and the suites after it, but
  not in an `olp` or `config` release.
- Order comes from the journal: suites measured by earlier runs on this Mac
  (`realtime-core timings`) replace the table above.
- Known pre-existing failures are listed in one file,
  `rt-core/tools/cidriver/known_failures.json` (suite, Go test, reason,
  evidence; rt-core's CI driver embeds and validates it), and reported apart;
  any other failure fails the run. Remove an entry when its test is fixed.
  Since 2026-10-07 it lists only `rtctl TestRobotPackagedConfigurations`
  (`cells TestRepositoryCompatibility` was fixed).
- A passed gate is reused for the same commit, tree, suites and gate
  definition (above), and its suites' passes cover later commits with the
  same inputs; `--regate` forces a run. The records live only on the Mac
  that ran the gate: from another Mac, the first fast gate runs every suite
  whose inputs differ from what the cell runs.

Standalone: `go run . realtime-core build --manifest $CELL --commit <sha> --test --test-level fast|full`.

Tests off the cell (no lock) are the component READMEs' commands on the Mac
(for example `go test ./...` here, `pixi run -e motion pytest` in
`weld_planner/v1`).

## The cell lock and the maintenance notice

Every command that builds, tests or changes something on a cell takes the
cell lock, a `flock` on the relay (`~/.rosie/locks/<cell>.lock`). It waits up
to `--idle-wait` for another run's lock, then for the cell to be idle
(disarmed, no lease, no OLP robot request for `--quiet`), printing who is
connected at every poll; then it puts up the maintenance notice and starts.
`release`, `config` and `prune` hold one lock and one notice for their whole
run (`config` used to take it per relay session, so another run could take
the cell between its notice and its apply).

What operators see while the notice is up:

- every OLP browser (the Mac's `olp ui` included) and Steam Deck v5 pendant
  on that cell shows a full-screen "Update in progress" overlay: the reason,
  "Step N/M · <step>" (and "Suite k/m: <suite>" during the test gate), a
  progress bar, the estimate in whole minutes ("About 12 min left"; past it,
  "Taking longer than expected (+N min)", never a negative clock), how long
  it has run, the steps with ticks and times, who started it and the
  release (example below);
- the OLP refuses arm, jog, Move-to, Home, load, play and cell selection
  (503 `maintenance`), and rt-control on the servo host refuses new grants,
  Arm, Enable, Home, jogs and program starts (`busy: <reason>`) for clients
  that bypass the OLP;
- Stop, Disarm, Disconnect and ending a running hold always work. The notice
  never stops or disarms anything itself.

A release's ETA is an estimate from this Mac's journal for the cell, not a
fixed number (it was 75 min for every full release; a cell's overlays once
showed about 1 h 10 min left during a release whose build and gate took
18m47s). `--plan` prints it step by step with its basis:

- each step: the median of its newest 5 runs in the cell's release timings
  (`*-realtime-core-release-*.timings.json`) that the release got past (a
  deploy that failed only on the owner-accepted `igh_master` counts; a start
  that needed a retry does not);
- build+test: the median build part of the cell's builds that compiled, plus
  the gate the build will run: the selected suites' lane plan (journal suite
  means, as `--plan`'s gate lines) times the median measured/planned ratio of
  the last 5 recorded gates; nothing when a recorded pass covers it;
- the notice: the sum + 20 % (at least 1 min), rounded up to a minute,
  counted from when the notice goes up (lock and idle waits come before).
  At every step start and end, and every gate suite start and end, the run
  re-estimates it from the work still to run; the start time stays. Steps
  inside the release update the phase but never the ETA or the progress.
- each step counts once from this Mac's journal: a release's own span, or a
  run of the command alone (`config`, `olp install`, `prune`, ... outside any
  release; machine time, idle wait excluded).

With no run of a step on record its default stands in, unverified and long
(build 10 min, gate 50 min, copy-dumps 5, precheck 2, deploy 10, start 5,
verify 2, olp install 5, prune 2, config 5, rollback 10): a cell never released
from this Mac shows 1 h 50 min for a full release, 1 h 18 min for olp. With
a Rosie 1430 cell's history: full about 24 min, olp about 22 min, config
about 28 min (its config step had no release run yet). The gate's stall limit
and start's ready wait are ceilings, not estimates; they are unchanged.
`realtime-core build`, `config`, `prune` and `olp install` run alone use the
same estimator for their own step (a build with `--test`: the median build
plus the gate of every suite of the level, narrowed once it has chosen).
`pendant build` and `install` take no cell lock and set no notice.

The notice also carries the run's progress (`progress` in the JSON: steps
with state, estimate, elapsed and remaining seconds; current step of total;
the gate's newest suite, its index of the total, suites done and running;
the fraction done). The fraction is weighted by estimated time, not step
count: finished steps count their whole estimate, the running one the share
of its estimate done (at most 99 %); inside the gate the remaining estimate
falls as suites finish, each weighted by its journal mean. The run sends it
as `progress <eta_s> <base64 JSON>` on its lock holder's heartbeat stdin and
the holder writes it into both notices; old clients keep reading `phase`
(the step line) and `eta`. The OLP route serves it; rt-control's route does
not (its notice is part of the generated application API, whose change is a
full rt-core release), so a Deck running OLP locally against a servo-host
notice shows the countdown without the bar. A full release mid-gate:

```text
MAINTENANCE
Update in progress
release de2a0c0e (full tier) on <cell>
Step 1/8 · Build and test gate
Suite 5/12: ui_test · 3 done · 2 running
[██████████░░░░░░░░░░░░░░░ 41%]
Expected finish
About 14 min left
Running for 7:12
▸ Build and test gate                   7:00 of about 18 min
· Copy stop dumps off the servo host    about 1 min
· Pre-checks                            about 1 min
· Install rt-core on the servo host     about 1 min
· Start the controller                  about 1 min
· Verify every axis                     about 1 min
· Install the OLP server                about 1 min
· Prune old releases                    about 1 min
Started by realtime-core release (<user>@<mac>) · release de2a0c0e
Robot actions are refused until it ends. Finish what you are doing, then Stop and disarm; STOP stays available here.
                                                                 [■ STOP]
```

The notice is cleared when the run ends, success or failure, including Ctrl-C
and a dropped connection. To see or end one:

```sh
go run . realtime-core maintenance show --manifest $CELL
go run . realtime-core maintenance clear --manifest $CELL   # any notice, both hosts
```

Clear a notice only after `show` says its run is gone (the owner process is
not running) and no deploy is running from another machine: clients flag a
notice 2 h past its ETA as stale but never clear it themselves.

A one-off job on a cell's hosts that is not a deploy command runs under the
lock too:

```sh
go run . realtime-core lock --manifest $CELL --reason "rebuild planner env" --on relay -- 'cd ~/x && make'
```

## Stop dumps to a tuning report

```sh
go run . realtime-core copy-dumps --manifest $CELL --to mac
(cd ../../../rt-core && go run ./tools/tuningreport report \
  --dumps ~/.rosie/stop-dumps/<cell> --since 45m \
  --manifest config/cells/<cell>.json \
  --label "J4 baseline" --out ~/.rosie/tuning-reports/<iteration>)
```

`copy-dumps` copies every dump to the relay's `dump_dir` with
`nice -n 19 ionice -c3 cat` on the servo host, throttled
(`dump_copy_rate_bytes_per_s`, 4 MiB/s on a Rosie 1430 cell), keeps a copy only when its
sha256 equals the servo host's and records it in `dump_dir/SHA256SUMS`;
`--to mac` then fetches them to `~/.rosie/stop-dumps/<cell>/`, hash-checked.
Nothing is decoded or deleted on the servo host. `precheck`, `config`,
`reboot` and `rollback` count a dump as copied only when a copy carries that
recorded hash (`precheck` also accepts this Mac's copy), so a configuration
release after a session of Stops needs only `copy-dumps`, which `release`
runs. Report options, metrics
and limits: [`rt-core/tools/tuningreport/README.md`](../rt-core/tools/tuningreport/README.md).

## Tuning a joint, configuration-only

Servo tuning is per joint: a joint's gains,
filters and vibration suppression are per-axis `startup` keys in the machine
config, limited to the drive's tuning catalog
(`rt-core/config/drives/a6ec.tuning.json`); safety bounds stay one value for
every joint. The first release carrying the tuning catalog is a full one (an
older rtctl does not know the keys); every iteration after it is a config
release. One joint, one variable, one iteration at a time.

Worked example: J4 load inertia ratio (A6-EC C00.06, `a6ec_load_inertia_ratio`).
**The numbers are examples, not recommendations.**

1. Baseline: run the standard short test program (still 15 s after arming
   and XYZ path holds at 400 mm/s; RxRyRz at 45 deg/s and a Move-to; the
   tuned joint alone at 10/30/60 deg/s; a Stop after each part),
   then `copy-dumps --to mac` and a tuning report without `--baseline`.
2. Edit the cell's machine config, `rt-core/config/machines/<machine>.json`,
   axis index 3 (J4) only, and say where the value came from:

   ```json
   "startup": {
     "a6ec_speed_feedforward_filter_hz": 318,
     "a6ec_speed_feedforward_percentage": 500,
     "a6ec_speed_feedforward_source": 1,
     "a6ec_load_inertia_ratio": 250
   },
   "startup_source": "... C00.06 = 250 (EXAMPLE: from the drive's inertia identification on <date>; unverified)"
   ```

3. Re-pin: compile the edited JSON with the **installed** release's rtctl
   (read-only on both hosts; rtctl itself builds only on Linux) and put the
   digest of its `compile ... -> <sha256>` line into the cell manifest's
   `rt_core.configuration_sha256`:

   ```sh
   go run . realtime-core config --manifest $CELL --dry-run
   ```

   The dry run warns that the edit is not committed; an apply of uncommitted
   bytes is refused, so never skip step 4.

4. Commit both files, then release; `--plan` must say `config`:

   ```sh
   go run . realtime-core release --manifest $CELL --commit HEAD --plan
   go run . realtime-core release --manifest $CELL --commit HEAD
   ```

   It refuses (nothing changed) on a removed startup key, a key outside the
   catalog that differs between axes, a coordinate identity change (that would
   invalidate Home), or any deployment-identity drift.
5. Before Home or Arm, check the readback lines: `realtime-core verify` (or
   the release's verify output) must show `EtherCAT startup readback ...
   verified=1` for the new key on J4 and a full `config_verified_mask`.
6. Run the same test moves, `copy-dumps --to mac`, report with
   `--baseline <previous>/metrics.json`, and keep or revert. Revert with the
   factory value written explicitly, never by deleting the key: the drive
   stores the last value (C13.10) and would keep 250.
7. Record the result in the cell's commissioning record.

The first real run of this loop (one joint's C00.06 on a Rosie 1430 cell,
four values): each `realtime-core config` took 35.5-36.9 s, each
`copy-dumps --to mac` of one dump about 1 min, each report about 2 s on the
Mac. The report's ring analysis covers 1-30 Hz, which cannot see a speed
loop oscillating (one trial value raised a 228 Hz line, found then by a
scratch spectrum). Since 2026-10-08 its "Servo loop quality" section reads
every joint's torque, velocity and residual up to 500 Hz: buzz episodes,
lines and which joint carries them, the loop band by speed
([tuningreport README](../rt-core/tools/tuningreport/README.md#servo-loop-quality-every-joint-up-to-500-hz)).
Run over a cell's recorded sessions it finds a joint oscillating at
100-200 Hz while it moves; a per-joint plan with pass criteria in absolute
units then orders the tuning iterations. A tuning value alone is a config
release; one that rides a commit carrying a native fix goes out with that
full release.

Each restart rewrites every startup SDO on every axis, and the A6-EC raises
ALF1.3 on too-frequent EEPROM writes: iterate deliberately.

## Steam Deck pendant

The cell manifest's `pendant_deploy` block names the builder and the Decks:

```json
"pendant_deploy": {
  "builder": {"kind": "container", "image": "rosie-v5-build:22.04", "platform": "linux/amd64"},
  "decks": [{"name": "<deck>", "transport": "tailscale", "ssh_user": "deck", "host": "<deck>",
             "app_dir": "/home/deck/devkit-game/RosiePendantV5", "steam_app_id": <shortcut app id>}]
}
```

```sh
go run . pendant build --manifest $CELL --commit <sha> --dry-run
go run . pendant build --manifest $CELL --commit <sha>          # add --with-tesseract-env for a first install
go run . pendant status --manifest $CELL --host <deck>
go run . pendant install --manifest $CELL --host <deck> --commit <sha> --dry-run
go run . pendant install --manifest $CELL --host <deck> --commit <sha>
go run . pendant rollback --manifest $CELL --host <deck>
```

One app per Deck, one launcher per cell. A Deck that serves two cells runs
one app from two shortcuts, one launcher script per cell
(`run-<relay>.sh`). One cell's manifest (the app owner) builds and installs
the app. The other cell's Deck entries name `app_owner` (the owning cell)
and its launcher, so `pendant install` with that manifest only writes its
cell into that launcher. It uploads nothing, swaps nothing and stops
nothing; rollback, `--package` and build are refused. Per Deck, run the
owning cell's install first, then the other cell's with the same
`--commit`:

```sh
go run . pendant install --manifest rt-core/config/cells/<owner cell>.json --host <deck> --commit <sha>
go run . pendant install --manifest rt-core/config/cells/<other cell>.json --host <deck> --commit <sha>
```

- **build** packages exactly the commit in a work tree that persists between
  builds: the container's `rosie-v5-work` volume (Ubuntu 22.04 amd64,
  `steamdeck/real/v5/container/`), or `builder.kind: "ssh"` with `ssh_target`
  and `work_dir` for a native linux/amd64 builder. It refuses a work tree
  with changes (`--clean` starts over). The Go OLP server is cross-compiled
  natively on the Mac and passed in. Output:
  `~/.rosie/pendant-packages/RosiePendantV5-<commit>.tar`.
- **install**, **rollback** and **status** send the v5 component's own
  `steamdeck/real/v5/deck-install.sh` to the Deck over stdin (`bash -s`: a
  script passed as an argument matched its own `pgrep -f` on Linux, so
  install would have killed itself mid-swap); `deploy.sh` is retired.
  **install** uploads and stages the package, checks its `SHA256SUMS` and
  that every library resolves before touching the app, refuses while the
  robot behind a running pendant's OLP is armed, enabled, leased or running
  a program (with no pendant, run.sh or Deck-local OLP running it closes
  nothing and checks no robot, so another Deck's loop on that cell does not
  block it; a pendant started mid-install is refused, not closed), stops a
  running pendant, swaps only the files the package owns
  (`pendant.env`, `olp-cells.json`, `private/` and other site files stay),
  carries the tesseract environment over, relaunches the Steam shortcut if
  the app was running and checks the pendant's OLP health. A failed check
  restores the previous package (exit 5). The previous package stays in
  `<app_dir>.previous` for `rollback`.
- Measured on this Mac (Apple silicon, Docker amd64 emulation, 2026-10-07):
  clean build 149 s (source 5 s, npm ci 7 s, make 54 s, make check 22 s,
  package with the Go server built under emulation 60 s); incremental 8 s at
  the same commit with the server cross-compiled natively (9 s cold, 1.2 s
  warm). The C++ app and its checks have no native path on an arm64 Mac (the
  Deck is x86_64): they run emulated, or natively on an amd64 builder.
- Unverified on a Deck: the relaunch (`steam steam://rungameid/<id>` over
  ssh) and the 30 s start bound. Get the owner's approval before touching an
  operator's Deck.

### The Deck's tunnel to the cell's OLP: `pendant tunnel`

```sh
go run . pendant tunnel install --manifest $CELL --host <deck> --dry-run   # rendered unit and supervisor
go run . pendant tunnel install --manifest $CELL --host <deck>
go run . pendant tunnel status  --manifest $CELL --host <deck>   # pendant status includes it
go run . pendant tunnel logs    --manifest $CELL --host <deck> [--follow]
go run . pendant tunnel restart|stop|rollback --manifest $CELL --host <deck>
```

A pendant in remote-OLP mode (`pendant.env`
`ROSIE_PENDANT_OLP_ORIGIN=http://127.0.0.1:18794`) reaches the relay's OLP
through an ssh tunnel on the Deck, over the LAN: the Deck's userspace
Tailscale cannot carry it. The Deck's `tunnel` block in `pendant_deploy`
names the relay's LAN address, the Deck's key and pinned host key (provisioned
by hand), the OLP port, extra forwards and the hand-made units it replaces.
`install` renders a systemd --user unit (`Restart=always`, started at
login/boot, running through sleep) around the `olp ui` supervisor (ServerAlive
5 s x 3; ssh back 1 s after a drop, doubling to 30 s while the relay stays
unreachable; the reason in `tunnel.state`). It replaces the hand-made unit and
keeps its file for `rollback`, waits for OLP health through the new tunnel,
prints how long the pendant had no OLP, and puts the old tunnel back if the
new one loses the health the old one had (exit 5). A current, healthy tunnel
is left alone; a dead one is restarted at once. `install` (when it replaces a
running tunnel), `restart`, `stop` and `rollback` refuse while this Deck's
pendant has the robot armed, leased, playing or jog-held: the pendant's 1 s
heartbeat would lapse against the OLP's 5 s deadman. The pendant itself needs
no restart after a drop: it polls status every 250 ms and shows the robot
reachable again as soon as the tunnel is back. Details: v4 README, "Deck
tunnel". A Deck's first deploy runs `pendant tunnel install` before `pendant
install`: the tunnel install passes on the tunnel's own health and names the
pendant that does not use it yet; `pendant install` then writes
`ROSIE_PENDANT_OLP_ORIGIN` (the tunnel's `olp_port`) into the launcher or
`pendant.env` where neither names an origin, and checks the relaunch through
the tunnel. Not yet run on a Deck when this was written; on a Deck that still
runs a hand-made tunnel unit, the first `install` replaces it.

## Timings

Every command ends with a table (total, machine time, operator wait, then
each session and phase) and keeps `<run>.timings.json` beside its transcript.

```sh
go run . realtime-core timings --last 20
go run . realtime-core timings --command build --cell <cell>
```

## Build host speed

The build host (on a Rosie 1430 cell: 6 cores, 7 GB, Ubuntu 22.04 arm64) builds in
one persistent tree per cell, `<scratch_dir>/tree`, updated by `git fetch` of
the bundle and a checkout of exactly the commit, verified by
`git rev-parse HEAD` = the commit and an empty `git status --porcelain`
before and after (a tree someone edited is refused; `--clean` deletes it and
builds from scratch). The deployment tree `<deployments_dir>/<prefix><sha8>`
receives copies of the artifacts and its own clean clone, as before. Under
the cell lock builds run at normal priority with `-j$(nproc)`; only
`--low-priority` (no lock) uses `build_jobs` at nice 15.

- C++ goes through ccache when the host has it (`/usr/lib/ccache`); on a
  host without it, `sudo apt install ccache` is an owner step.
- The build and the gate share the relay user's persistent caches: Go's
  build cache (`GOCACHE`, the user's default) and the Cartesian resolver's
  content-addressed object cache, the motion-server Makefile's default
  `~/.rosie/cache/motion-server/v1` (the build used a per-cell copy and the
  gate the default, so neither warmed the other). Both print the caches'
  state on their first line; `--clean` keeps them.
- The rt-core package and compiled outputs are reused from the previous tree
  when nothing under `rt-core/` or `robot_description/` changed since it
  (never with `--fetch-package`); the resolver likewise when
  `motion-server/v1` is unchanged. Go uses the user's build cache,
  `node_modules` persists in the tree (npm ci only when `package-lock.json`
  changes), the planner env is linked from the previous tree when
  `weld_planner` is unchanged; with `cell_deploy.olp.planner` it is instead
  content-addressed under the planner's `state_dir` by `pixi.toml`,
  `pixi.lock`, env and pixi version, so a Python-only change reuses it and
  no deployment holds it (steamdeck/real/v4 README, "Build"). Such a cell's
  `olp install` also installs the planner unit for the same tree and checks
  that the planner answers with that tree's commit (`olp status`:
  `planner_revision`, `planner_health`).
- The CadQuery env (Import a part) is provisioned by every build on every
  cell, no flag: linked from the previous tree when `cadquery/v1`'s
  `pixi.toml` and `pixi.lock` are unchanged (same key as the planner's),
  else installed `--locked` by the pinned pixi into the tree (linux-64: 2.5
  GiB, 644 s into an empty package cache; refused below 6 GiB free). Every
  build then imports `cadquery` on it and reads `cadquery/v1/step/dog_box.step`
  as the OLP does, and only then writes `<deployment>/cadquery-env.json`. The
  OLP unit runs that env's python directly (`CADQUERY_TOPOLOGY_PYTHON`): the
  worker's fallback, `pixi run` with the relay's pixi, is what failed every
  part import on the Jetsons until 2026-11-04. `olp install` refuses a tree
  without a passing record and restores the unit unless the running OLP uses
  that python; `olp status` prints `cadquery_env ok|MISSING` and the record
  (`cadquery_check`).
- rt-core compiles `src/main.cpp` once per daemon variant to
  `build/obj/<variant>/main.o` without the source identity; only
  `build/obj/version.o` (`src/version.cpp`) carries `git describe`. A commit
  with no C++ change recompiles that one file and relinks: on that build
  host in a kept tree (`make -j6 live control sim`, under the cell lock)
  98.8 s -> 5.1 s; clean 99.8 s -> 99.1 s. The split compile also makes
  ccache useful (arm64 container, second clean build: 0 hits -> 4/4 hits,
  37.6 s -> 6.0 s with `CXX='ccache g++'`, 66.8 s -> 2.6 s with
  `/usr/lib/ccache` on `PATH`).
- `make test`: the C++ contract binaries run in parallel under `-j` (arm64
  container, -j10: 14.7 s -> 5.0 s); `host/lifecycle-test.sh` reuses the
  outer build's simulator object instead of compiling `main.cpp` twice
  (106 s -> 11 s); `go test ./tools/rtctl/...` compiles each native fixture
  once and overlaps the slow tests (130-150 s -> 76 s), every assertion and
  real timeout kept.
- Measured before these: see `steamdeck/real/v4/README.md`, "Building a deployment tree".

## Rollback per tier

| Tier | Automatic | By hand |
|---|---|---|
| config | a failed check after the stop restores `control.env` and verifies the old configuration (exit 3; exit 4 leaves both units stopped with the steps printed) | `realtime-core config` with the previous machine JSON committed |
| olp | a failed check reinstalls the previous unit (exit 5) | `go run . olp rollback --manifest $CELL` (the program store stays; a tree from before it runs without it) |
| full | `release --rollback auto` (default): when `start` (after one retry of a stalled bring-up) or `verify` fails, `copy-dumps` then `realtime-core rollback --to-configuration <the pin that ran before>`: stop, `install.sh --rollback`, restore that configuration's `control.env` backup, start disarmed, verify. A resumed release uses the configuration recorded when it began | `go run . realtime-core rollback --manifest $CELL --to-configuration <sha256> --dry-run`, then without `--dry-run` |
| pendant | a failed check restores the previous package (exit 5) | `go run . pendant rollback --manifest $CELL --host <deck>` |

A rollback never reverts drive parameters the newer configuration wrote and
the older one omits: write them explicitly.

## Troubleshooting

| You see | It means | Do |
|---|---|---|
| `controller armed=1 ... an operator holds the cell` or `cell stayed in use for 600s` | someone is armed, enabled or holds the lease, or the OLP journal shows robot requests in the quiet window | the idle wait lists who (OLP client label, address, last request, session holder); ask them to finish, Stop, disarm and release. Never disarm for them |
| `the cell is locked by ...` | another run holds the cell lock | wait for it; `maintenance show` names it |
| `uncommitted changes under rt-core/config, robot_description or the cell manifest` (release, config) | this checkout carries configuration edits git does not hold | commit them (machine JSON plus every pin) or run from a clean worktree at the commit; `config --dry-run` still prints an edit's pin |
| `WARNING the cell runs configuration <pin>, which no commit on this branch pins` | someone applied a configuration from an uncommitted or unpushed tree | if the listed drive values say "identical", nothing changes on the drives; otherwise bring the running values into the repo (OLP deployment tree `git diff`, servo host `config-history/machines/<pin>.json`), or the owner accepts with `--accept-config-drift <pin>` |
| `NOT IDLE within 25m` (wait-idle) or an idle-wait timeout | the cell did not go idle in time | re-run later or with a longer `--idle-wait`; the notice was cleared |
| `stop dump(s) without a hash-verified copy` (precheck) or `without a verified copy` (config) | dumps on the servo host with no copy carrying the sha256 `copy-dumps` recorded (`dump_dir/SHA256SUMS`; precheck also counts this Mac's copy) | `realtime-core copy-dumps` (add `--to mac` for analysis); a copy made by other means is copied again |
| `deployment identity changes [...]` (config) | the installed rtctl files a key it does not know as metadata; the binary would ignore it | `release --tier full` |
| `compiled configuration X is not the cell pin Y` (build) | the machine config and the manifest pin disagree at the commit | recompile, re-pin, commit |
| `apply session ended unexpectedly (exit 1, err <nil>)` 0.2 s after `connecting`, no `revpi` lines (config) | before `f6a585b2`: a relay script over Linux's 131072-byte single-argument limit, refused by the relay's exec with no message (seen with a 132852-byte script); the runner now sends scripts over 96 KiB on stdin | `--dry-run` first (nothing was changed if it still shows the old pin), then run again with a build carrying the fix |
| `compiled outputs for the unchanged pin differ` | the compiler's semantics changed under the same digest | review the diff in `scratch_dir/build-<sha8>/compiled.diff`; `--allow-compiled-drift` only with the owner |
| `rt-core health failed on ...` / `AssertionError: {... 'igh_master': 'fail' ...}` (deploy) | `rtctl hostcheck` failed a host prerequisite the gate requires, for example `igh_master` when the host's IgH kernel module is older than the package's. The assertion prints every check: on some hosts `isolated`, `nohz_full` and the `/run/rosie-rt-core` directories also read `fail` on every deploy, but the gate does not require them (the directories exist only while the units run) and `release` ignores them | the package is installed and both units are stopped. Fix the host, or with the owner's acceptance of that one check, per run, `release --accept-hostcheck <check>` / `realtime-core start`. Never skip other required checks |
| `home_valid_mask=N, not 63` (verify) | Home did not come back after the restart | report it with the journal lines verify printed. Do not re-home, restore anchors or verified-delta to clear it |
| `EtherCAT distributed clock NOT LOCKED: slave N system time difference (ESC 0x092C) reads ...` (verify, so also start, restart, release) | the drive's clock does not follow the reference clock. Its sync, and with it the TxPDO update, drifts through the host cycle, and the drive returns stale then doubled feedback that can latch `coordinate_reference`. Before `9bac73cd` rt-core wrote the reference clock once per 1000 cycles: the mini bench read 25.4 ms and IgH logged `Slave did not sync after 5000 ms` at every start. With a write every cycle it reads 1.6-2.4 us. The bound is `cell_deploy.dc_lock` (10 us, IgH's own start check) | do not run the robot. Report it with the `fact DC slave ...` lines. A cell whose installed rt-core predates `9bac73cd` needs a full release that carries it. Do not raise the bound to pass. A `DC lock unreadable` / `not shown` line means a `reg_read` gave no value (sudo, master, timeout): fix that, never skip the check. `Slave did not sync` lines alone are a fact, not a failure |
| `cell_deploy.dc_lock is missing from <manifest>` (verify) | the cell manifest has no DC lock bound | add `dc_lock {"max_system_time_difference_ns", "settle_s"}` with a `_source` note (`steamdeck/real/v4/README.md`, "Adding a new robot cell") and commit it |
| `drive alarm(s) present: ... run again with --clear-drive-alarms` (restart) | a drive alarm is latched; a restart could clear it at bring-up | find and fix the cause at the drive with the owner; only then `--clear-drive-alarms` |
| `drive alarm(s) restart must not clear even with --clear-drive-alarms` (restart) | the alarm is not in the drive profile's `fault_reset` list (encoder, overcurrent, EEPROM ...) or its 0x203F detail is unread | service the drive as its manual says; never restart around it |
| `STOP NOT CLEAN` (restart, exit 6) | a unit did not stop with `Result=success`; `Result=timeout` = systemd SIGKILLed rosie-rt-core after 15 s | nothing was started; read the `shutdown` journal lines and ask the owner. Never SIGKILL by hand; a reboot needs the owner |
| `START FAILED` (restart, exit 1) | the start refused or the bus did not verify after a clean stop; both units are stopped | fix the cause in the lines above, then `restart` again (it only starts the stopped units) |
| `control.env is not the cell pin` (start) on a cell whose installed pin no commit holds | `start` checks the manifest's pin | `realtime-core restart` starts the installed pin; never run start's script by hand |
| `the work tree ... has changes` / `the build tree has changes` | someone edited the persistent tree by hand | look at it; `--clean` only once you know what it was |
| `the robot behind the pendant's OLP is in use` (pendant) | a pendant is open on the Deck and its cell is armed, leased or running a program | finish and disarm first |
| `a pendant started on this Deck during the install` (pendant) | a pendant was opened after the idle check; nothing changed | rerun |
| `deck <name> ... is offline in Tailscale (last seen ...)` (pendant) | Tailscale lists the Deck offline: asleep, powered off or off the network; refused before any upload or session | wake the Deck on its Wi-Fi, check `tailscale status` lists it online, rerun |
| `rosie processes in D/Z state` | a thread is stuck in an IgH call | read `/proc/<pid>/task/*/stack`, ask the owner; never SIGKILL |
| `start stall kind=bringup ...` (start, release) | the bus did not converge within 60 s; the bring-up lines above it show drives in OP per axis over time; release already started once more | report the lines (owner); never re-home or work around it |
| `start stall kind=identity` / `kind=busy` | the controller came up with another configuration or backend, or armed/granted | not retried; read verify's facts |
| `resume refused: the configuration that ran before ... is unknown` | a full release of this commit stopped part-way and this Mac has no record of what ran before | `--rollback-to-configuration <previous Describe configuration_sha256>` or `--rollback never` |
| `realtime-core deploy needs --package` | deploy installs the relay-built package | `realtime-core build --test --fetch-package` (release runs both); the old build-peer path is `--legacy-build-peer` |
| Release stuck at the gate (or build) with nothing running on the relay | before 2026-10-08: the Mac's ssh was half-open (the path died; the relay had finished) and nothing noticed. Now the keepalives end it in ~15 s and the watchdog re-attaches after 2 min of silence | update the checkout. The run prints `stall test gate ...; last line: ...; re-attaching` and continues; if the relay work is gone or the re-attach stalls too, the step fails and the lock is released: run the same release again (a passed gate is reused). Never kill the Mac's ssh by hand |
| `stall ... no output and no relay heartbeat for 2m` / `again with no new output since the re-attach` (build, release) | the connection to the relay is dead, or the relay stopped printing even its heartbeat | `realtime-core maintenance show`; the work's output is in `~/.rosie/watched/<run>-<label>/out` on the relay (`exit` holds its status when it finished). Re-run when the relay is reachable (`tailscale status`) |
| `STALL test gate still running past its limit` | a suite hung (alive, no exit) for longer than twice the gate's recorded one-at-a-time time + 15 min | the gate was stopped on the relay (TERM, KILL after 30 s); read its `out` and the suite logs in `scratch_dir/build-<sha8>/gate-logs/` before re-running |
| `the cell lock's holder session ... ended ...; stopped before step N` (release) | the holder's connection dropped (keepalives) or the relay saw no heartbeat for 120 s and released the lock and notices | `maintenance show`, then run the same release again: it resumes |
| `no heartbeat from the client for 120s` (holder output) | the Mac running the run went away (sleep, dead path) | nothing on the relay; re-run from the Mac |
| `reused: the full gate of <sha8>` | a pass of these suites is recorded for this commit | nothing; `--regate` to run it again |
| `skipped <suite>: inputs unchanged since its pass at <sha8>` / `no recorded pass of this suite on <cell>` (fast gate) | a recorded pass on this Mac covers the suite's inputs at the commit, or none does and its inputs differ from what the cell runs | nothing; `--regate` diffs against the installed commits instead, `--test full` runs every suite |
| UI says `OLP API unreachable` / `Not connected`, or `olp ui status` shows `tunnel DOWN` | the Mac's tunnel to the cell's OLP is down or retrying; `status` prints the last restart reason (`Timeout, server ... not responding` = tailnet path dead) | `olp ui status`; `olp ui start` restarts a dead tunnel at once. Health failing (EOF, connection reset) through a tunnel that is up, with `channel N: open failed: connect failed ... connection refused` in `olp ui logs`, means the relay's OLP itself is down: `olp status` |
| The pendant shows the robot unreachable / `SERVER NOT ANSWERING`, or `pendant tunnel status` shows `tunnel DOWN` | the Deck's tunnel to the relay's OLP is down or retrying; `status` prints the supervisor's phase and last restart reason (`No route to host` / `Connection timed out` = the Deck is off the relay's LAN; `Permission denied (publickey)` = the relay's `authorized_keys` lost the Deck key; `Host key verification failed` = the relay's host key changed) | `pendant tunnel status --host <deck>` (an offline Deck is refused in ~1 s: wake it). `pendant tunnel install` restarts a dead tunnel at once; the pendant reconnects by itself. Health failing through an up tunnel = the relay's OLP is down: `olp status`. `pendant.env ... the tunnel forwards ...` (or `<launcher> ...`) = the pendant's origin does not match `olp_port`: fix `pendant.env` or the Deck's launcher script |
