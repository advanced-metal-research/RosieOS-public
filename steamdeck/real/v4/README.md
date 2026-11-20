# Native runtime and Steam Deck operator runbook

`steamdeck/real/v4` owns the deployment CLI: `realtime-core`, `olp`, `camera`
and `pendant` (`go run . help`). The Steam Deck pendant is
[`steamdeck/real/v5`](../v5/README.md); `pendant` builds and installs it (Tier 4).
The current runtime selection is `rt-core`. Installed v4 cells retain their
published releases until their approved native configuration is supplied; this
checkout refuses retired backend selections with `reason=backend_retired`.

## Native deployment

Cell configs live in `rt-core/config/cells`. A native runtime node must name an
approved `rt_core` package/configuration binding. Do not add a guessed binding
to a v4-only manifest. Manifests of cells that ran the retired runtime remain
readable inventory records, but that runtime cannot be deployed from this
checkout.

To review a native deployment, run from `steamdeck/real/v4`:

```sh
go run . realtime-core deploy --manifest rt-core/config/cells/<cell>.json --dry-run
```

The cell config's `rt_core.configuration_sha256` is checked against the
compiled `control.env` after installation, and the deploy refuses to stop a
running controller that is armed, enabled or granted. A cell whose listener
PKI is already provisioned on the servo host sets `rt_core.remote_pki:
"installed"`; deploy then keeps the host's `ROSIE_RT_REMOTE_ARGS` verbatim
(listener and remote jog clock policy) instead of deriving it from
`remote_listen`, which names the client endpoint. Deploy checks that the
installed control and PKI pair IDs match the manifest before stopping units.
A pair change needs a paired controller and OLP certificate migration; this
command does not rotate credentials. For a reviewed pair change, stage the
new root-owned PKI on the servo host under
`/etc/rosie-rt-core/pki-staged/<new-pair>` and the matching OLP client files
on the relay under `~/.rosie/pki-staged/<new-pair>`. Set
`rt_core.remote_pki: "installed"` in the selected manifest. Then run a full
source release with `--pair-migration-from <old-pair>` and `--rollback auto`.
The release checks both staged CAs before any stop. It backs up the old PKI,
installs the package and new control binding with both controller units
stopped, starts disarmed, then installs the OLP with the new client
certificate. A failed start, verify, or OLP install triggers rollback of the
old package, control binding, and PKI. A failed rollback needs an on-host
state check before recovery. The old and new pair revisions
must match. This does not arm, Home, or jog the bench.
The old CA key remains in root-only `pki-history` for rollback. This path has
no automatic retention or cleanup policy yet; retire it only after the
rollback window closes.
After a completed rollback, a retry keeps the old PKI history in a root-only
`*.retry.*` directory. It does this only when the live PKI and control file
match that backup. A partial or mismatched rollback blocks the retry.
`--pair-migration-from` is not a standalone deploy operation. If the
connection dies during the switch, keep the cell stopped and inspect the
release journal before a resume or paired rollback. A resume from another
computer needs the previous configuration SHA, pair ID, and Describe axis
count as explicit inputs.

Before a full release changes the configuration, `realtime-core release`
compares each power-cycle drive setting with the running controller's last
verified readback. It refuses a mismatch before the build and test gate.
`--plan` runs this read-only check too. The check does not write drive settings
or power-cycle drives. If it refuses, review the stopped-drive migration and
physical power cycle before you retry. The controller reads the drives again
at startup, so a matching preflight does not guarantee activation. See
[`rt-core/docs/MIGRATION.md`](../../../rt-core/docs/MIGRATION.md) for the Home
and coordinate-evidence rules after a drive mode change. If the old release
expects a different drive mode, software rollback alone might not restart it.
Record the old drive values and a stopped-drive reversal plan before migration.

For the reviewed A6-EC tracking-mode change from 4 to 1, use the explicit
two-part release. First, run `release --plan` with `--tier full`,
`--accept-config-drift <live-pin>`, and `--drive-transition prepare`. Review
the cell, old pin, exact commit, selected slave positions, and live readbacks.
The prepare command runs the normal full gate, installs the paired package,
stops both units, writes only the declared mode change, and keeps maintenance
active. Do not start either unit after prepare.

With a person at the bench and the hardwired Stop available, power-cycle the
drives. Then run the same release with `--drive-transition resume`,
`--power-cycled`, and `--rollback never`. Resume checks the installed package
and drive modes before start. It verifies disarmed output and invalid Home.
The maintenance notice remains active after success. Inspect both hosts and
the OLP selection before a separate `maintenance clear` and OLP select. A
failed resume is not permission to clear maintenance or start the old release:
restoring mode 4 requires stopped-drive reversal and another physical cycle.
Neither step authorizes arming, jogging, or physical qualification.

When the manifest build
peer cannot be reached, `--package rosie-rt-core.tar.gz` supplies a live
package built from this clean checkout's exact HEAD; any other commit is
refused. Deploy leaves the controller installed and stopped. Start it
explicitly:

```sh
go run . realtime-core start --manifest rt-core/config/cells/<cell>.json
```

Start refuses unless both units are installed and stopped and `control.env`
carries the cell pin. It then waits for every axis to verify its startup
configuration, with the robot disarmed and no grant, and stops both units
again on any failure. It never enables at boot, arms, Homes or moves.

After satisfying the manifest's package, configuration and host prerequisites,
the same command without `--dry-run` performs the native installation. The
installer retains package verification, bus-owner handover and health checks.
It masks the obsolete runtime/bridge units and closes the old pair-owner route.
Installation leaves motion disarmed; activation follows the explicit manifest
policy and does not authorize a tour, Home or motion automatically.

The v4 commands that drove the retired stack (`manifest`, `deploy`, `control`,
`mode`, `system-state`, `steamdeck`, `test-primitive`, `nats` and the rest), the
native v4 pendant and the local v4 simulation harness are removed. They are not
alternatives for native commissioning.

Build/control peers are selected by the manifest. Keep the exact
source revision, artifact producer, package hash, configuration and manifest
identity together. Target architecture and approved cache identity determine
which artifacts may be installed; a previous test receipt is not permission to
change a cell's native binding.

## Deployment tiers

Pick the smallest tier that carries the change. Every tier starts with the
robot disarmed and no lease held (OLP or Steam Deck session released), and
with Stop dumps copied off the servo host (`realtime-core copy-dumps`): any
rt-core restart deletes `/run/rosie-rt-core`, dumps included. Durations are
these commands' own measurements on a Rosie 1430 cell (`realtime-core
timings`); before them, the hand-made scripts took 30-60 min for tier 3.

Every command below is driven by the cell manifest (`--manifest
rt-core/config/cells/<cell>.json`, run from `steamdeck/real/v4`): hosts and
users from `connections`, the pin and listener from `rt_core`, and the relay's
paths, ports and unit names from its `cell_deploy` block (see [Adding a new
robot cell](#adding-a-new-robot-cell)). They reach the relay like `config`
(`--transport tailscale|ssh`), send every script as the remote command
argument, print `[elapsed +step] phase detail` lines, keep a transcript in
`~/.rosie/deploy-journal/`, and have a `--dry-run` that contacts nothing.

| Tier | Carries | Restarts | Tool | Duration |
|---|---|---|---|---|
| 1. Configuration-only | machine-config values the installed rtctl understands | `rosie-rt-core`, `rosie-rt-control` (and OLP pin) | `realtime-core config` | 35.5-36.9 s (three runs) |
| 2. OLP-only | OLP server, UI, resolver, CadQuery scripts and env; rt-core protocol and capabilities unchanged | the cell's OLP unit | `realtime-core build`, `olp install` | fast gate + 5.5-7.2 s install (`release --tier olp` 17m24s and 14m03s, mostly the gate, before the per-suite fast gate) |
| 3. Full rt-core package | native code, protocol, compiler, drive profiles, descriptions, units | both rt-core units, then OLP | `realtime-core build --test --fetch-package`, `precheck`, `copy-dumps`, `deploy --package`, `start`, `verify`, then tier 2 | full gate (11.4-11.7 min on lanes) + deploy 10 s, start 25 s, verify 1 s, OLP 6 s; the first end-to-end `release` (fast gate) 24m44s incl. a 9m22s lock wait: build+test 13m56s, deploy 35.7 s, start 22.4 s, verify 7.8 s, OLP 6.6 s |
| 4. Steam Deck v5 rebuild | pendant app, bundled `olp-core.js`, meshes; required by any capabilities digest change | the pendant | `pendant build`, `pendant install` | build 218 s fresh, 23 s incremental (54.9 s with the builder image rebuilt); install 5.3 s; needs its own approval |

**Which tier do I use?**

1. Only values in `rt_core.machine_config` change (a drive startup parameter
   such as C01.14, a limit, `tracking_lag_allowance`), and the **installed**
   release's rtctl already knows every key: tier 1.
2. Only OLP Go/UI code or the Cartesian resolver changes, and rt-core's
   protocol minor and capabilities digest stay the same: tier 2.
3. Anything in `rt-core/` source, the protocol, rtctl, a drive profile, a
   robot description, units, or a configuration key the installed rtctl does
   not know: tier 3, then tier 2 for OLP if its pins or code changed.
4. The capabilities digest or protocol minor changed (for example 1.13 -> 1.14:
   `201fbe81…` -> `b1e28f06…`; rt-control's maintenance route: `b1e28f06…` ->
   `725c5bbb…`), or the pendant app, `olp-core.js` or meshes changed: tier 4
   as well, after tier 3.

A robot-description change (tier 3) also invalidates saved OLP programs
("Accept current settings" and a re-plan); a configuration-only change needs
only re-selecting the cell, Pull cell settings and a re-plan.

**One command.** `go run . realtime-core release --manifest <cell> --commit
<sha>` reads what the cell runs, picks the smallest tier by these rules, and
runs that tier's commands below in order under one cell lock, with fast or
full tests, automatic rollback and one timing table; `--plan` shows the tier
and steps first. How to use it, the test levels, the tuning loop and
troubleshooting: [`docs/DEPLOY_AND_TEST_WORKFLOW.md`](../../../docs/DEPLOY_AND_TEST_WORKFLOW.md).

### Maintenance notice (every tier)

`deploy`, `start` and `config` put up a maintenance notice on the relay, which
is the cell's OLP host: `/var/lib/rosie-olp/maintenance.json`. Every OLP browser
and Steam Deck v5 pendant on that OLP then shows a full-screen "Update in
progress" overlay with the reason, the run's step and gate suite, a progress
bar, the minutes left and the step list, and the
OLP refuses arm, jog, Move-to, Home, load, play and cell selection with 503
`maintenance`. Stop, Disarm, Disconnect and ending a running hold always pass;
the notice never disarms or stops anything itself
([OLP maintenance notice](../../../offline-programming/v1/README.md#maintenance-notice)).
Every write and clear also goes to the servo host's copy,
`/etc/rosie-rt-core/maintenance.json`: rt-control (rt-core with the notice)
then refuses new grants, Arm, Enable, Home, jogs and trajectory or program
starts with `busy: <reason>` for clients that bypass the relay's OLP
([rt-control](../../../rt-core/docs/APPLICATION_API.md#admission-retries-and-cancellation)).
If either host's notice cannot be written the command refuses and clears the
other. rt-control with the maintenance route also serves that copy
(`GET /v1/maintenance`, with its grant holder) and every OLP server merges it
from its selected cell into its own notice, so a Steam Deck running OLP
locally shows the overlay during a servo host update too.

- The notice goes up before anything touches the cell; the run then waits up
  to `--idle-wait` (default 2m, unverified) for the controller to be disarmed
  with no lease, reporting the busy state in the notice. The armed/enabled/lease
  gates themselves are unchanged and still refuse if operators have not
  finished. Each poll prints who to wait for: the servo host's armed, enable
  and lease state with the rt-control grant holder (`controller="…"
  principal=…` from its journal; nothing on releases without the notice), and
  every client of the relay's OLP with its label, address, last robot request
  and whether it holds the execution session. `config` sets a notice only when the cell is otherwise ready (any
  other refusal stands at once, with no notice; `--dry-run` never sets one).
  `deploy` sets it when its relay script starts, after the package is built
  and on the relay. `start` does not wait (the controller is stopped).
- `--maintenance-eta` counts from when the notice goes up. Defaults are the
  owner's estimates, unverified: config 2 min (plus the idle wait when the
  cell was busy), deploy 45 min plus the idle wait, start 5 min. Inside a
  cell lock (`release`, `realtime-core lock`) a step's notice write keeps the
  lock's ETA and progress: the lock run owns them. `release` (and `build`,
  `config`, `prune` alone) estimates its own from this Mac's journal and
  sends its progress (steps, gate suites, fraction weighted by estimated
  time) with the re-estimated ETA at every step and suite transition, as
  `progress <eta_s> <base64 JSON>` on its holder's heartbeat stdin;
  `olp install` alone writes a one-step progress with its notice
  (`notice_progress.go`,
  [DEPLOY_AND_TEST_WORKFLOW.md](../../../docs/DEPLOY_AND_TEST_WORKFLOW.md#the-cell-lock-and-the-maintenance-notice)).
- The phase follows the steps: copying the package, installing (stop,
  handover, health), closing the pair owner; restarting the controller with
  the new configuration, then restarting the OLP; starting and verifying.
- Cleared when the command ends, success or failure (an exit trap in the
  relay script; `config` also clears from the Mac). `--keep-maintenance`
  leaves it up for the next step of a session, which takes it over and clears
  it. A dropped connection ends a `deploy` or `start` relay script, whose trap
  clears it; `config`'s apply script finishes on its own and clears it.
- `go run . realtime-core maintenance set --manifest <cell> --reason TEXT
  [--eta 30m]` puts up an operator's notice for a whole session: deploy steps
  update its phase but never clear it. `olp install`/`rollback`, `reboot`,
  `resume-after-reboot` and `wait-idle --notice` set and clear their own.
  `maintenance clear` ends any notice, `maintenance show` prints it. Use
  `clear` for a notice a lost connection left behind; clients flag one 2 h
  past its ETA as stale but never clear it. A notice records the process that
  owns it (host, pid, run id, cell lock); `show` prints `STALE` when that run
  is gone (pid dead or lock free) and `ORPHAN` for a servo-host notice with no
  relay notice. Nothing clears either by itself.
- The relay needs the OLP server with the notice (tier 2) and passwordless
  `sudo -n` (already used for the OLP restart). If the notice cannot be
  written the command refuses with exit 2 and nothing changed.

### Cell lock: building, testing and using the robot are exclusive

Rule: no build, test or update starts on a
cell's hosts (OLP host, servo host) while the robot is in use, and nobody can
arm while one runs. Builds and tests on hosts outside the cell (this Mac, CI,
a bench) need no lock.

- One run per cell: every command that sets the notice first takes
  `~/.rosie/locks/<cell>.lock` on the relay (`flock`), and refuses (exit 2)
  naming the holder from `<cell>.lock.owner` while another run holds it.
- `realtime-core build` (and `--test`) holds the lock for the whole build: it
  waits up to `--idle-wait` (default 10m) for the controller to be disarmed
  with no lease and no OLP robot request in the journal for `--quiet` (2m),
  with **no notice yet**, printing who to wait for every `--interval` (15s).
  Another deploy or lock run's notice counts as in use (`config` between its
  sessions, a step's `--keep-maintenance` notice): it is never adopted or
  cleared; an operator's notice is adopted and left up. It
  refuses (exit 2, nothing started) if the cell stays in use; then puts both
  notices up with reason `build <sha8> in progress on <cell>` (or `build and
  test gate …`), re-checks the cell is still idle under them, and only then
  builds, at normal priority. The ETA is the release estimator's (this Mac's
  journal: the cell's median build, with `--test` plus the gate of every
  suite of the level, narrowed to the chosen suites once the gate starts;
  + 20 %), else 10 min (60 min with `--test`; unverified defaults), or
  `--maintenance-eta` (`build-durations.jsonl` is no longer read or
  written: builds inside releases never filled it). Both notices are cleared when it ends: success,
  failure, Ctrl-C or SIGTERM (the Mac releases the lock and exits 130) or a
  dropped connection (stdin EOF ends the relay holder, whose exit trap
  clears). The relay build ignores hangups, so a released lock keeps the
  notices up until that build or gate finishes; build and gate sessions take
  `<cell>.work` shared and refuse unless the lock is held.
- Dead connections (after a half-open fast-gate session held a cell's lock
  for an hour): every ssh the deploy commands open, Mac to relay
  (plain or `tailscale ssh`, which passes the options to the real ssh) and
  relay to servo host, carries `ServerAliveInterval=5 ServerAliveCountMax=3
  ConnectTimeout` (`remote_ssh.go`, the tunnels' values), so the Mac sees a
  dead path in ~15 s. The relay's sshd may not see it for hours, so the Mac
  also writes a heartbeat line to the lock holder's stdin every 10 s and the
  holder treats 120 s without one like stdin EOF: it releases and clears both
  notices (still waiting for relay work under the lock). Both unverified. A
  healthy idle holder answers both and stays up for the whole run. `release`
  checks before each step that its holder is still connected and otherwise
  stops there ("stopped before step N"; run it again, it resumes).
- `--low-priority` is the old behaviour: no lock, no notice, `nice 15`/
  `ionice` best-effort 7. Only for a relay that serves no robot in use.
- Any other job on a cell's hosts (an agent's test run on the OLP host, a
  benchmark) runs under the generic wrapper, which does the same wait, notice
  and clear around one command:

  ```sh
  go run . realtime-core lock --manifest $CELL --reason "agent test run" --eta 20m \
      --on relay -- 'cd ~/scratch/rosieos && go test ./offline-programming/v1/...'
  ```

  `--on local` (default) runs the command on the Mac with `ROSIE_CELL_LOCK`
  set, so `realtime-core`/`olp` commands of that cell inside it run under the
  lock instead of refusing (they update the notice's phase, never clear it).
  `--on relay` runs it as `bash -c` on the OLP host in its own process group;
  Ctrl-C or a dropped connection sends it TERM before the notices are cleared.
  `--on servo` runs it on the servo host over ssh from the relay; nothing
  heavy runs there, and a dropped connection may leave it running.
- Arming stays refused from every client while the notice is up: browser and
  Deck through the relay's OLP (503 `maintenance`), any client that reaches
  rt-control directly, including a Deck-local OLP, a direct lease and the
  remote jog stream's begin (`busy`). Stop, Disarm, Release and ending a
  running hold always pass.

### Tier 1: configuration-only (`realtime-core config`, `0414a46d`)

Changes only the cell's `rt_core.machine_config` JSON on the running cell
(details: [Configuration-only deploy](#configuration-only-deploy)):

```sh
cd steamdeck/real/v4
go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --dry-run
go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --accept-configuration <new sha256>
```

- An apply refuses uncommitted configuration inputs and a configuration no
  commit holds, and a change to drive values the cell runs from an unpinned
  configuration without `--accept-config-drift <running pin>`
  ([Configuration-only deploy](#configuration-only-deploy)).
- Session 1 (all of `--dry-run`) is read-only: it probes both hosts and
  compiles the JSON with the **installed release's own rtctl**, then refuses
  (exit 2, nothing changed) on a held lock or lease, an armed controller,
  faults or drive alarms, an incomplete `config_verified_mask`, a coordinate
  identity change (would invalidate Home), any non-`_` deployment-identity
  drift, a removed or axis-varying non-tuning startup key, inactive units, a
  wrong anchor layout, Stop dumps present, or a stale OLP pin. Equal digests are a no-op.
- You accept the printed digest. Session 2 stops, publishes and starts only
  `rosie-rt-control` and `rosie-rt-core`, then requires the new Describe digest,
  a full `config_verified_mask`, the pre-deploy `home_valid_mask`, no faults,
  disarmed, no unit restart, unchanged anchors and enablement. Any failure
  after the stop restores `control.env` and verifies the old configuration
  (exit 3); if that fails both units stay stopped with manual steps printed
  (exit 4). With `--olp restart` it then updates the OLP pin; an OLP failure is
  restored and leaves the robot on the new configuration (exit 5).
- It cannot carry protocol or native code changes, or a key the installed
  rtctl does not know: an older rtctl files such a key as deployment metadata
  and the binary silently ignores it (`jog_jerk` on `6c6f97c0`; the tool
  refuses that drift). Those are tier 3.
- Each restart rewrites every startup SDO on every axis; the A6-EC stores
  them (C13.10) and raises ALF1.3 on too-frequent EEPROM writes. Iterate
  deliberately, and turn a parameter off by writing its off value, never by
  omitting the key.
- First-run sequence on a cell that has not used it: `--dry-run`; a no-op
  apply (unchanged JSON); an edit to a `_source` note only (new digest, no
  behaviour change: proves stop, publish, start, verify and the OLP update);
  then a real value change. Set `--home-timeout` from that run's timing.
- **First hardware use** (a Rosie 1430 cell, one joint's inertia-ratio
  trials): three real value changes, each a dry-run, the apply and a
  no-op re-run. Each apply took 35.5-36.9 s (apply session 31-33 s, start to
  verified 19-23 s); each verified the new readback, `config_verified_mask=63`
  and `home_valid_mask=63` with coordinate identity unchanged. The `_source`-only
  step was not run. The core verified the configuration 16-21 s after start
  and Home came back 1-3 s later (`home ok after 1-3s`), well inside
  `--home-timeout`'s 45 s default, which stays unconfirmed until it is set
  from more runs.
- Rollback: run `config` again with the previous machine JSON, or swap
  `control.env` from `/etc/rosie-rt-core/config-history/control-env/` as shown
  under [Configuration-only deploy](#configuration-only-deploy). Drive
  parameters the old configuration does not write keep their new values.

### Building a deployment tree (`realtime-core build`)

```sh
go run . realtime-core build --manifest rt-core/config/cells/<cell>.json [--commit <sha>] [--test [--test-level fast|full] [--test-since <sha>,...] [--test-tier olp|config|full] [--regate]] [--fetch-package] [--clean]
```

Builds one commit into `<deployments_dir>/<deployment_prefix><sha8>` on the
relay; it installs nothing and contacts no controller. The commit must be on a
local ref: the Mac sends a git bundle (incremental over the commit of the tree
the OLP unit runs, else full). The relay builds in the cell's persistent tree,
`<scratch_dir>/tree`: it fetches the bundle and checks out exactly the commit,
and refuses unless `git rev-parse HEAD` is the commit and `git status
--porcelain` is empty before and after (a tree edited by hand is refused;
`--clean` deletes it and builds from scratch), so uncommitted work never
ships. The deployment tree receives copies of the artifacts and its own clean
clone. Under the cell lock at normal priority with every core ([Cell lock](#cell-lock-building-testing-and-using-the-robot-are-exclusive);
`--low-priority`: `nice 15`/`ionice` best-effort 7, `build_jobs`, no lock), logged per step
under `<scratch_dir>/build-<sha8>/`, it
builds rt-core `live control sim`, compiles the cell (must equal the cell pin;
with the same pin the compiled outputs must be byte-identical to the previous
tree's unless `--allow-compiled-drift`), runs `package-runtime.sh` and
`verify-package.sh`, builds the OLP server, writes the API-only UI stub,
builds the Cartesian resolver and checks it answers `fk` for the cell's model,
and links the planner pixi environment. A cell whose manifest has
`cell_deploy.olp.planner` gets a managed env:
`<state_dir>/envs/<env>-<key>`, key = sha256 of `pixi.toml`, `pixi.lock`, the
env name and the pinned pixi version (0.81.0, fetched once into the cell's
cache dir and sha256-checked; the lock is format v7, which a host's pixi
0.67.2 cannot read: it re-solves from the network, and a truncated mapping
download once failed a build that way), installed `--locked` when
no finished one exists (`motion-cpu`: 4.0 GB, 61 s in a linux/arm64
container on the Mac; unmeasured on a cell host), so a Python-only
`weld_planner` change reuses it and needs no `--install-planner-env`. Every
build then runs the env's device probe (`weld_motion_planner.utils.device`,
report `<deployment>/planner-env.json`, which `olp install` requires; with
`cuda: true` torch must run a kernel on the GPU) and loads the tree's planner
server on the env (`make_app`). Without the block the previous tree's env is
linked when `weld_planner` is unchanged since that environment's commit (else
`--install-planner-env` runs `pixi install`). Every build, on every cell and
without a flag, then provisions the CadQuery env part import runs on
(`cadquery/v1`, env `default`): until 2026-11-04 no command did, the OLP's
worker fell back to `pixi run` with the relay's pixi and Import a part on a
Jetson answered 502 `worker_failed` (`unsupported-platform ...
linux-aarch64`) while every deploy check passed. Key = sha256 of
`cadquery/v1`'s `pixi.toml` and `pixi.lock`, the env name and the pinned pixi
version (the managed planner's key); when the previous tree's
`<deployment>/cadquery-env.json` holds the same key the tree links that env
(resolved to the tree that installed it), else the pinned pixi installs it
`--locked` into `<deployment>/source/cadquery/v1/.pixi`, refused below 6 GiB
free (linux-64: 2.5 GiB env, 0.5 GiB package cache, 644 s into an empty
cache on WSL; unmeasured on a Jetson). Then the env's python, run as the OLP
runs it (directly, in `cadquery/v1`), must import `cadquery` and extract the
topology of `cadquery/v1/step/dog_box.step` (128 edges, 10 recommended
seams on linux-64), which also warms it; only then is
`<deployment>/cadquery-env.json` written (cadquery version, import and
topology milliseconds, edges, how the env was provisioned), which `olp
install` requires and the next build reuses by. Timing phase
`cadquery-env`. A flag would only turn every first build on a cell into a
refusal: the env changes only with its lock, and an OLP that cannot import a
part is broken. The IgH userspace library and its notices come from the
cell's `igh_deps_dir` (`PKG_CONFIG_PATH`, `ECRT_LICENSE_DIR`). The rt-core
package and compiled outputs are copied from the previous tree when nothing
under `rt-core/` or `robot_description/` changed since its commit (never with
`--fetch-package`), the resolver likewise when `motion-server/v1` is
unchanged (else it compiles through the relay user's content-addressed
object cache `~/.rosie/cache/motion-server/v1`, which the gate's
`motion_test` shares, as both share `GOCACHE`); C++ goes through ccache when
the relay has it. A dropped connection leaves
the build running; re-running a finished build is a no-op.

`--test` then runs the gate in the same tree, several suites at a time
(rt-core `make test`, `go test ./tools/rtctl/...` and `./config/cells/...`,
the fault-matrix shutdown and SDK path-jog tests, motion-server `make test`,
OLP `go test ./internal/...` and the composed Cartesian jog test, UI `npm ci`
when `package-lock.json` changed and `npm test`, the pendant's `olp-core`
node tests). Suites that assert real-time behaviour (`make_test_serial`,
`make test`'s real-time tail; `rtctl`; `faultmatrix_shutdown`,
`sdk_path_jog`, `olp_composed`) run alone: no other suite runs beside them.
`--test-level fast` runs only the suites whose inputs changed since that
suite's last pass recorded on this Mac for the cell (`release_gate_history.go`;
added after an OLP-tier release spent 13m19s gating rt-core suites whose
inputs had already passed). A suite's inputs at a
commit are the git tree entries its input paths select; a suite is skipped
only when a recorded pass saw exactly those entries (compared through git,
or through the pass's recorded input hash when that commit is gone), or,
with no covering pass, when they equal those at every `--test-since` commit
(release passes the installed rt-core and OLP commits; default the previous
tree's commit), so nothing it covers ships. No record and an unknown base run
it. `--test-tier olp|config` (release passes its tier) leaves out servo-only
inputs (native sources and tests, host scripts, protocol files; never a Go
file, a file beside one, `rt-core/config` or `clients/ts`): those tiers ship
none. Each run and skip prints why, naming the pass it relies on. The default
level is `full` (every suite). Failures listed in
`rt-core/tools/cidriver/known_failures.json` (the
one list; rt-core's CI driver validates it) are reported separately from new
ones; any new or unnamed failure fails the run, and a recorded pass counts
only while today's list still excuses its failures. A passed gate is recorded in
`~/.rosie/deploy-journal/gate-passes.jsonl` (with each suite's definition
digest and input hash) and reused by a later build of the same commit, tree,
suites and gate definition; a change to one suite's command voids only that
suite's passes. `--regate` ignores the records: it runs the gate even for a
recorded commit, and fast then diffs against `--test-since` instead
(docs/DEPLOY_AND_TEST_WORKFLOW.md, "Tests: fast and full"). `--fetch-package` copies
the live package (`runtime-package.components/rosie-rt-core-*.tar.gz`) to
`~/.rosie/rt-core/packages/<sha256>.tar.gz` on the Mac, hash-checked, for
tier 3.

A completed gate with new failures remains failed. Its explicitly successful
selected suites are retained in the same ledger with `partial: true` and
`level: partial` (the original level stays in `attempt_level`), so a
later fast retry can reuse only those suites under the same definition and
input checks. Missing, duplicate, cancelled, unclassified or failing results
are not retained as suite passes. Partial records never satisfy whole-gate
reuse, even if their original level was `full`; the distinct level also prevents
an older CLI from promoting them to a full pass. A full retry still runs the
whole gate. This applies to new attempts only, without importing old logs or
rewriting failed history.

The build and the gate are watched sessions (`remote_watch.go`): the relay
runs the script under `setsid`, detached from the connection, writing its
output to `~/.rosie/watched/<run>-<label>/out` and its exit status to `exit`
there; the session only follows that file, printing a heartbeat every 30 s
while the work is quiet. A stream that ends without the exit status (ssh exit
255 from the keepalives) or shows no line for 2 min is re-attached once over a
fresh connection from the last complete line received: it streams the rest
and the exit status, or reports that the work is gone (killed without a
status). A re-attach that delivers nothing new fails the step, which releases
the lock. A session still running past its limit (build 90 min; gate twice
its suites' recorded one-at-a-time seconds + 15 min, at least 30 min: 74 min
for the full gate) is stopped on the relay (TERM to its process group, KILL
after 30 s) and fails with its last line. All thresholds are unverified.
Before this, an OLP-tier release waited on a half-open gate session whose
gate had ended on the relay 17 min in, holding the lock until the
coordinator sent SIGTERM to the Mac's ssh 64 min after the start.

Measured, before the persistent tree, on a Rosie 1430 cell's relay (a
6-core arm64 Jetson): a build (clean clone, nice 15, `-j4`) took 186 s:
rt-core 100 s, package 5 s, OLP 7 s, resolver 67 s; the gate, one suite at
a time, 1770 s (`make_test` 686 s, `rtctl` 347 s, `motion_test` 345 s,
`ui_test` 308 s). On this Mac in an arm64 Ubuntu 24.04 container (10 cores,
g++ 13, 2026-10-07), rt-core `make control sim`: clean 32 s (24 s with a
warm Go cache), no change 0.4 s, one header or `src/main.cpp` touched
24-25 s, the same at `-j4` and `-j10` (each binary is one translation
unit), and ccache recorded 0 hits and 0 misses: rt-core compiles and links
in one invocation, which ccache does not cache. On that relay in a kept
build directory (deploy timings micro-benchmark, nice 10): `make live
control sim` clean 98 s at `-j4` and `-j6`, one commit to the next 102 s
and a header touch 99 s (every commit recompiles the three `src/main.cpp`
binaries: each embeds the commit through `build/source-version`), same
commit 1 s; resolver 67 s clean, 0 s kept; `make test` 576 s at `-j4`,
520 s at `-j6`. So an OLP or configuration build that reuses rt-core and
the resolver drops ~170 s of the 186 s (measured: an OLP-tier build took
21.1 s, its OLP server 2.9 s); a full build keeps the ~100 s rt-core
compile until rt-core compiles its version string in a separate object; a
full gate on three lanes is bounded by `make_test` then
`faultmatrix_shutdown`, ~708 s of one-at-a-time time (unverified: suites
slow each other). On a relay without ccache, `sudo apt install ccache` is
an owner step.

Since `32f08f57` (2026-10-07) the version is in its own object
(`src/version.cpp`, `build/obj/version.o`) and `main.cpp` compiles once per
variant without it: on that relay a commit with no C++ change in a kept tree
takes 5.1 s instead of 98.8 s (`make -j6 live control sim`, under the cell
lock), a clean build 99.1 s (was 99.8 s), and ccache hits every main.cpp
object on a second clean build in the arm64 container. `lifecycle-test`
reuses the outer simulator object (106 s -> 11 s), `make -j test` runs the
C++ contract binaries in parallel, and the gate runs `make test`'s real-time
tail (`make test-serial`) alone.

### Deploy timings

Every command above (`build [--test]`, `deploy`, `start`, `config`,
`precheck`, `wait-idle`, `copy-dumps`, `verify`, `reboot`,
`resume-after-reboot`, `olp install|rollback`) writes
`~/.rosie/deploy-journal/<run>.timings.json` beside its `<run>.log` and ends
with a table: total, machine time, operator wait (the maintenance idle wait),
then each relay session with its phases. Phases come from the scripts' own
step lines, with the remote clock kept beside the Mac's: build steps
(`rt-core`, `package`, `olp`, `resolver`), gate steps with their slowest
suites (make recipe lines timed through a `SHELL` wrapper, Go packages, node
tests), the servo install (`extract`, `preflight`, `stop`, `install`,
`control-env`, `v4-closure`, `health`), `start-to-verified` and verify's
`home-wait`.

```sh
go run . realtime-core timings [--last 20] [--command build] [--cell <id>]
```

prints the history, oldest first, and the phases and suites that took the
most machine time over those runs. Runs journalled before timings existed
show their total only. Reconstruction of the first native deploys of a
Rosie 1430 cell: the test gate (~16-30 min) and rebuilding
unchanged code dominated machine time: rt-core ~100 s per tree (`src/main.cpp`
embeds the commit, so it recompiles on every commit) and the resolver ~67 s
(each tree starts a fresh object cache).

### Tier 2: OLP-only

For OLP server, UI or resolver changes when the cell's protocol minor and
capabilities digest are unchanged:

```sh
go run . realtime-core build --manifest rt-core/config/cells/<cell>.json --test
go run . olp install --manifest rt-core/config/cells/<cell>.json --deployment <sha> --dry-run
go run . olp install --manifest rt-core/config/cells/<cell>.json --deployment <sha>
```

`olp install` puts up the maintenance notice and waits for the cell to go idle
like `deploy`, then refuses (exit 2, nothing changed) unless the tree is
complete, its build recorded a passing CadQuery check for the tree's
python in `<tree>/cadquery-env.json` (a tree built before 2026-11-04 has
none: build a new one), the planner and control-tunnel units are active and the OLP has no
program playing or paused, no jog hold and no armed, enabled or leased
controller. It reads the **live authenticated Describe** through the cell's
execution route (`https://<rt_core.remote_listen>`) with the current tree's
client certificate and refuses unless it is the cell pin on ethercat with the
machine's axes (`--expect-capabilities` adds the digest). It copies the client
PKI and OLP state from the current tree, writes `private/olp-cells.json` pinned
to that Describe (`binding.configuration_sha256`/`machine_sha256`,
`deployment_sha256`, `capabilities_digest`), backs up the unit as
`<tree>/<unit>.bak-<previous sha8>`, installs the unit rendered from the
manifest (on the first cell it replaced, byte-identical to the hand-made one, but
for `CADQUERY_TOPOLOGY_PYTHON`, the tree's CadQuery env python), restarts
it and checks health, the unit's environment (the running OLP's
`CADQUERY_TOPOLOGY_PYTHON` must be that python) and `/targets`. It then clears
its notice (the OLP refuses selection under one) and selects the cell
(Describe only; no lease, no motion). A failed check reinstalls the backup
(exit 5). Rollback: `go run . olp rollback --manifest <cell>` reinstalls that
backup with the same checks (a tree from before the CadQuery env included);
`olp status` shows the unit, tree, catalogue pins, `cadquery_env
ok|MISSING` (the unit's `CADQUERY_TOPOLOGY_PYTHON`, executable or not) with
the build's `cadquery_check` record, health and `/targets`. A new cell's first OLP unit and client PKI are
provisioned by hand; `--previous <dir>` names a seed tree holding
`private/olp-cells.json`, its PKI and `olp-state/`.

With `cell_deploy.olp.program_store` (`{"dir": ..., "label": ...}`, the
cell's shared program store, `offline-programming/v1/README.md` "Cell program
store"), the unit carries `OFFLINE_PROGRAMMING_PROGRAM_STORE` and its label,
and install creates the directory (relay user, 0700), archives what is in it
into `<tree>/olp-install/program-store-pre-install.tgz` before the swap, and
restores the unit unless the restarted OLP answers `/cell-programs` with the
store. The directory must be outside `deployments_dir`, `scratch_dir` and
`dump_dir` (validated): no install, rollback or prune touches it. `olp
rollback` to a tree from before the store runs without it and says so; the
programs wait in the directory. `olp status` reports whether the running OLP
serves it.

`cell_deploy.olp.unattended_loop_allowed` (bool, absent = false; the
owner's decision per cell) lets the cell's loops run
unattended (`offline-programming/v1/README.md` "Loop mode"). Nothing here
renders it: the OLP reads it from the manifest in its own tree, so `olp
install` (OLP tier) delivers it and `olp rollback` to an older tree takes it
back with that tree's manifest. The rt-core configuration pin does not move.

`olp install` still writes and clears the servo host's notice copy and reads
`rtctl status` there for its idle wait; it restarts nothing on the servo host.
`realtime-core release` picks tier 2 only when nothing the servo host's
package carries changed since the installed rt-core release; `--tier olp`
forces it while the configuration pin and capabilities digest are unchanged
and lists the rt-core changes it leaves undeployed (workflow guide, "One
command").

`olp ui start|status|logs|restart|stop --manifest <cell>` runs the operator
Mac's browser UI against the cell's OLP (`olp_ui.go`): a launchd agent running
a supervised ssh tunnel to `cell_deploy.olp.listen` on the relay
(`127.0.0.1:18894`, `--transport tailscale|ssh`) and a launchd agent running
Vite on `127.0.0.1:5189` proxied to it. It contacts only the relay's OLP API
(health, `/targets`, execution status for its in-use refusal); jobs, state
and logs live in `~/.rosie/olp-ui/<cell>/`. Workflow guide, "The browser UI
against a cell". The tunnel supervisor (`tunnel_supervisor.go`) is shared
with the Steam Deck's `pendant tunnel` ("Deck tunnel" below), which runs it
under systemd --user.

### Tier 3: full rt-core package

For native code, protocol, rtctl, drive profile, description or unit changes:

```sh
go run . realtime-core build --manifest <cell> --commit <sha> --test --fetch-package
go run . realtime-core precheck --manifest <cell>
go run . realtime-core copy-dumps --manifest <cell>            # when precheck lists dumps
go run . realtime-core deploy --manifest <cell> --package ~/.rosie/rt-core/packages/<sha256>.tar.gz
go run . realtime-core start --manifest <cell>
go run . realtime-core verify --manifest <cell>
go run . olp install --manifest <cell> --deployment <sha>
```

1. `build --test` as above; compare any failure with the known list before
   calling it a regression. `deploy --package` takes only a package built from
   this checkout's clean HEAD, so run the remaining steps from a checkout at
   that commit. `deploy` without `--package` refuses: the relay-built package
   is the default. The old path, a package built on the manifest's build peer
   by `tools/build-rt-core-package.sh`, is `--legacy-build-peer` (kept while
   installed cells may depend on it).
2. `precheck` (read-only, exit 2 on any refusal) requires both units active,
   disarmed, no enable or lease, no faults or drive alarms,
   `config_verified_mask` full, no OLP program playing or jog hold and no OLP
   operator activity in `--quiet` (2m), every stop dump copied, and no rosie
   process in D or Z state. It keeps the facts, anchor hashes included, under
   `<scratch_dir>/precheck/` on the relay. `wait-idle` polls the same idle
   facts (`--quiet 10m --limit 25m`); `--notice` puts up the maintenance notice
   while it waits.
3. `copy-dumps` copies every file in `/run/rosie-rt-core/native/telemetry` to
   the relay's `dump_dir` with `nice -n 19 ionice -c3 cat` on the servo host,
   throttled to `dump_copy_rate_bytes_per_s`, and keeps a copy only when its
   sha256 equals the servo host's, recorded in `dump_dir/SHA256SUMS`; `--to
   mac` also brings them to `~/.rosie/stop-dumps/<cell>/`. A dump counts as
   copied (precheck, config, reboot, rollback) only when a copy carries that
   recorded hash. Nothing is decoded or deleted on the servo host; decode
   elsewhere.
4. `deploy` backs up `control.env` into
   `/etc/rosie-rt-core/config-history/control-env/` before any unit stops,
   then installs (`install.sh --from-package --live`, current/previous release
   slots; `generate-control-env.sh --keep-remote-args`). Before the backup it
   refuses, with nothing stopped, a deploy that would lose the host's remote
   listener: `rt_core.remote_pki: "installed"` needs exactly one
   `ROSIE_RT_REMOTE_ARGS="--remote-listen ...` line to keep, and a manifest
   without it is refused when the host's line is not empty (that deploy would
   publish `ROSIE_RT_REMOTE_ARGS=""`, as it once did on a servo host).
   Restore an erased line with `realtime-core rollback`. A health gate that
   fails a required check (for example `igh_master`: the host's IgH kernel
   module older than the package's) leaves the units stopped; `start` then
   starts them disarmed. `release` carries on past it only with the owner's
   per-run `--accept-hostcheck <check>`. `start` prints the drives' bring-up from the core's
   `metrics.json` whenever it changes (operational drives, per-axis OP and AL
   state, master AL) and a failed start ends with `start stall
   kind=bringup|identity|busy`; `release` starts once more after a
   `bringup` stall, then rolls back.
5. `verify` (read-only) requires the cell pin on ethercat with the machine's
   axes, every startup readback `verified=1` since the core started (none
   `verified=0`), `config_verified_mask` and `home_valid_mask` full, no faults,
   disarmed, the anchor layout, anchor files unchanged since the last
   precheck, and nothing in D state; it prints the post-start fault, anchor
   and Home journal lines. A Home that did not come back is reported, never
   re-homed.
   It also requires every DC slave's distributed clock locked
   (`cell_deploy_dc_lock.go`). Once `rosie-rt-core` has been active
   `cell_deploy.dc_lock.settle_s`, the servo host reads ESC register 0x092C
   (system time difference) on each axis's `slave_position` three times, 1 s
   apart. Each read is `sudo -n timeout 3 ethercat reg_read -p<pos> -tuint32
   0x092C`, a register datagram through the running master, not a mailbox
   transfer. The raw text comes back to the Mac. Bit 31 is the sign and bits
   0-30 the magnitude in ns, so `0x8000095c` is 2396 ns. Verify fails when any
   magnitude exceeds `max_system_time_difference_ns`, or when a read is empty
   or unparsable. The `fact` lines carry every value. IgH's `Slave did not
   sync` kernel lines since the start are counted as a fact but never fail
   verify alone, because the clock can converge after IgH's 5 s start check.
   A machine with `bus_bringup.no_dc` has nothing to check, and a manifest
   without `dc_lock` fails verify. Why: until `9bac73cd` the reference clock
   was written once per 1000 cycles and never locked. On the mini bench,
   0x092C read 25.4 ms, IgH warned at every start, and stale-then-doubled
   TxPDO bursts latched `coordinate_reference`, and no deploy step noticed.
   After the fix it reads 1.6-2.4 us. `verify --dry-run` prints the check
   (`dc lock: ...`) and the script that reads it.
6. Tier 2 for OLP (new catalogue pins), and tier 4 if the capabilities digest
   changed.
7. `realtime-core prune` (release runs it last): the installed
   `install.sh --prune` keeps current, previous and pinned releases
   (`--pin <rev>`), deletes the rest under `/opt/rosie-rt-core`, never touches
   units. A failed or unsupported prune changes nothing else.

`realtime-core release` runs these steps under one lock and, run again for the
same commit after a stop part-way, resumes from the cell's state (units
stopped: start; running: verify; then olp install).

Rollback: `go run . realtime-core rollback --manifest <cell>
--to-configuration <the pin that ran before>` (refuses an armed or leased
controller and uncopied dumps; stops both units, `install.sh --rollback` to
the previous slot, restores that configuration's `control.env` backup,
starts disarmed and verifies), then `olp rollback`. `realtime-core release`
runs it after copying the dumps when start or verify fails. If a stopped core does not
exit (thread in an IgH ioctl), do not SIGKILL by hand; read
`/proc/<pid>/task/*/stack` and ask the owner before a reboot (a kill
during a mailbox call leaves the EtherCAT master reserved until a reboot).

### Controlled restart (`realtime-core restart`)

Restarts the controller the cell already runs, release and configuration
unchanged, without the manifest's pin: the start runs the pin `control.env`
holds (on a cell running a configuration no commit pins, `start` refused
it). It replaces hand sequences that cleared a latched A6-EC Er41.1 after a
locked-rotor stall.

```sh
go run . realtime-core restart --manifest <cell> --reason "<why>" [--clear-drive-alarms] [--home-wait 45s] [--dry-run]
```

Under one cell lock (`--idle-wait`, `--quiet`; both notices up): `copy-dumps`;
read-only facts (the precheck script, recording the anchor files for verify)
and its refusals (armed/enabled/leased, OLP session or activity, uncopied
dumps, D state, one unit up and one down, a controller not running
`control.env`'s configuration, faults and drive alarms); one servo-host
session under the configuration deploy's flock that ignores hangups,
re-checks the same facts, stops `rosie-rt-control` then `rosie-rt-core` with
`systemctl` and requires `Result=success` on both (otherwise exit 6, nothing
started: `Result=timeout` is systemd's SIGKILL after `TimeoutStopSec`), then
runs `start`'s script with the installed pin (pair, pair revision and
`rt-package verify` refusals unchanged); verify against that configuration
and the Home that was valid before (restart never homes; a joint whose Home
does not return is named and the run fails). Drive alarms and faults are
printed before and after; the servo host's `config-history/journal.jsonl`
gets `restart-stop`, `restart-stopped`, `restart-started` (or
`restart-stop-unclean`, `restart-start-failed`) with the reason.

`--clear-drive-alarms` is the owner's statement that the cause of the alarms
present was inspected and fixed. Without it any drive alarm refuses; with it
only alarms the axis's drive profile lists in `fault_reset.resettable_alarms`
by their exact 0x603F/0x203F pair pass (read from this checkout: machine
config -> robot definition `drive_profile` -> `rt-core/config/drives/<profile>.json`);
a fault that no such alarm explains (beyond the alarm's axis in
`safety_fault_mask` and `drive_readiness`, bit 7, in
`execution_fault_reasons`) refuses always. With both units already stopped
(a restart whose start failed) it only starts them. `verify
--expect-installed` verifies against the `control.env` pin alone. Exit codes:
0, 1 failed (`START FAILED`: both units stopped; run restart again), 2
refused (nothing changed), 6 stop not clean. Untimed on a cell: the hand
sequence's stop took 2.5 s and start 21.6-21.9 s.

### Servo host reboot (owner approval only)

A reboot of the servo host needs the owner's approval, even disarmed. It is
never automatic and no other command reboots:

```sh
go run . realtime-core reboot --manifest <cell> --owner-approved --reason "<what the owner approved>"
go run . realtime-core resume-after-reboot --manifest <cell> --booted-after <RFC3339 time printed by reboot>
```

`reboot` prints a warning, asks for the servo peer name at a terminal, refuses
an armed, enabled or leased controller or an uncopied stop dump (a reboot
empties `/run`), reboots with `systemctl reboot` under the maintenance notice
and waits up to 600 s for a new boot time. `resume-after-reboot` refuses
unless the host booted after `--booted-after`, both units are inactive, the
EtherCAT master is `Idle` with the cell's slave count (read only while both
units are inactive) and nothing is in D state; it then runs the `start`
script and `verify`.

### Adding a new robot cell

These commands need, in `rt-core/config/cells/<cell>.json`:

- `connections`: the servo host (`transport: ssh-via`, `host`, `ssh_user`,
  `via_peer`) and its relay (`transport: ssh`, `host`, `ssh_user`). The relay
  builds, holds the deployment trees, runs the OLP and receives stop dumps.
- the realtime-core node's `rt_core` block: `machine_config` (its
  `robot.robot_description.path` names the model; its axes are the slave
  count when `bus.unknown_slaves` is `refuse`), `configuration_sha256`,
  `pair_id`/`pair_revision`, and `remote_listen` (the OLP's execution address).
- a top-level `cell_deploy` block (`_`-prefixed keys are notes; any other
  unknown key is refused):
  - `deployments_dir`, `deployment_prefix`: trees are
    `<deployments_dir>/<deployment_prefix><sha8>`;
  - `scratch_dir` (build logs, TMPDIR, bundles, gate clone, precheck records;
    must allow exec) and `dump_dir`, both outside `deployments_dir`;
  - `home` (the relay user's HOME) and `path` (toolchain then system bin
    directories: go, node, pixi, pkg-config);
  - `igh_deps_dir` (holds `igh-prefix/lib/pkgconfig/libethercat.pc` and
    `ethercat-<version>/COPYING*`), `build_jobs`, `dump_copy_rate_bytes_per_s`;
  - `expected_ethercat_slaves`, only when the machine does not refuse unknown
    slaves (a bench with spare drives);
  - `dc_lock: {"max_system_time_difference_ns", "settle_s"}`, verify's bound
    on every DC slave's |ESC 0x092C| and how long `rosie-rt-core` must have
    been active before it samples (tier 3 step 5). It is seeded at 10000 ns,
    IgH's own DC start check, and 15 s, both unverified (the `_source` note in
    each manifest). A DC machine whose manifest lacks it fails verify;
  - `olp`: `unit`, `description` (with `{release}`), `listen`,
    `planner_unit`, `planner_origin`, `planner_pixi_env`, and
    `control_tunnel_unit` when rt-control is reached through a tunnel;
    optional `planner: {"description": "... {release} ...", "state_dir":
    <abs, outside deployments_dir>, "cuda": true|false, "threads": N}` hands
    the planner unit to the deploy commands (`cell_deploy_planner.go`):
    `olp install` (so also `release --tier olp` and a full release) renders
    it for the installed tree (the tree's env python and directory,
    `--dense-store-dir <state_dir>/dense`, Warp/XDG caches under
    `<state_dir>/cache`, OMP/Numba threads, `TORCH_ALLOW_TF32_CUBLAS_OVERRIDE=0`,
    the tree's commit as `AMR_WELD_PLANNER_SOURCE_REVISION`, a first line
    marking it rendered), backs up the current unit beside the OLP's backup
    (a unit without that marker, i.e. made by hand, is also kept for good in
    `<state_dir>/hand-made/`), copies the old dense store's blobs across,
    restarts the planner only when no plan is in flight, and refuses,
    reinstating the backup (exit 5) before the OLP unit is touched, unless
    `/api/motion/health` answers within 180 s from the unit's main process
    running that tree's python in that tree's directory, with
    `source_revision` equal to the tree's commit and `cuda` equal to the
    manifest's; a later failed OLP check reinstates both; `olp rollback`
    reinstates the planner backup too (checking its commit when it is a
    rendered unit). It runs inside `olp install`'s maintenance wrapper, so it
    waits for the cell to be idle like the OLP swap; timing phases `planner`
    and `olp` (install) and `planner-env` (build). `olp status` prints
    `planner_unit managed|hand-made`, `planner_tree`, `planner_revision` (the
    unit's) and `planner_health` (with the running `source_revision`).
    `planner_origin` must then be `http://127.0.0.1:<port>`. Without the
    block the planner unit is provisioned by hand and only has to be active;
    optional OLP cell discovery: `discovery_interfaces` (the operator-LAN
    interfaces for mDNS, never the servo link; renders
    `OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES`) and `discovery_tailnet:
    {"listen_port": N}` (1024-65535, not `listen`'s port; an
    advertisement-only listener on the host's tailnet address, renders
    `OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT`), for example
    `["wlP1p1s0"]` and 8798 ([OLP README](../../../offline-programming/v1/README.md#finding-cells-on-the-network));
    optional `camera_origin`, only for a camera service on the relay that
    this manifest does not install (loopback http URL; renders
    `OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN`). A cell with a `camera` block
    (next item) leaves it out: the origin follows from the block.
  - `camera`, optional: the cell camera service on the relay
    ([cell-camera/v1](../../../cell-camera/v1/README.md)), installed by
    `camera install`: `instance` (the unit is
    `rosie-cell-camera@<instance>.service`), `device` (a `/dev/v4l/by-id/...`
    name), `capture` (`WIDTHxHEIGHT@FPS`), `listen` (loopback) and
    `path_prefix`; optional `pixel_format`, `output`, `jpeg_quality`,
    `encoder`, `label`, `title`, and `replaces` (units holding the same camera
    or port, stopped first and started again if the install fails). The OLP
    unit relays `http://<listen><path_prefix>`
    ([OLP README](../../../offline-programming/v1/README.md#cell-camera)).
    A manifest test checks every committed cell that has the block. A
    RealSense has `"source": "realsense"` and no `device`: a `realsense`
    block with `sdk_dir` (the relay's pyrealsense2 package, built there by
    `cell-camera/v1/host/build-pyrealsense2.sh`), optional `serial` (else
    the only RealSense on the relay) and optional `depth` (`WIDTHxHEIGHT@FPS`:
    raw depth at `<path_prefix>/depth.z16`, else colour only); `pixel_format`
    stays `YUY2`
    ([cell-camera README, "RealSense"](../../../cell-camera/v1/README.md#realsense)).
  - `extra_cameras`, optional: a list of further camera blocks, other
    angles of the same cell (for example `side`). Each has the
    `camera` block's fields and its own `instance`, `device` and `listen`;
    `default` is not an instance name. Needs `camera`. The OLP unit carries
    them as `OFFLINE_PROGRAMMING_CELL_EXTRA_CAMERAS=<instance>=<origin>,...`
    and the OLP lists them after the cell camera.
- for Steam Decks, a top-level `pendant_deploy` block: the `builder` and the
  `decks`, each optionally with a `tunnel` block ([Deck
  tunnel](#deck-tunnel-pendant-tunnel)).

The first cell's block carried the values its hand-made deploy scripts had
hard-coded. The OLP unit, planner unit, control tunnel and the OLP client
PKI are provisioned once by hand.

#### Tune each joint

Servo tuning is per robot and is commissioning configuration: rt-core code stays generic for 1..n drives, a joint's gains,
filters and vibration suppression are per-axis `startup` keys in the cell's
machine configuration (only keys the drive's tuning catalog lists, such as
`rt-core/config/drives/a6ec.tuning.json`), and safety bounds stay one
joint-unit value for every joint. Drive detail, units and journal lines:
`rt-core/docs/drives/a6ec/README.md`, "Tuning a joint"; a worked example:
[workflow](../../../docs/DEPLOY_AND_TEST_WORKFLOW.md#tuning-a-joint-configuration-only).
One joint and one variable at a time:

1. Read the baseline: the start's `EtherCAT startup observe` journal lines for
   every joint (machine-level or per-axis `observe` list), copied off the
   servo host.
2. Run the standard short test program, the same poses and directions every
   iteration, with a Stop after each part (a dump keeps the last ~100 s):
   (A) right after arming, hold still 15 s, then path holds at 400 mm/s
   +-X, +-Y, +-Z; (B) +-Rx, +-Ry, +-Rz at 45 deg/s and one full-speed
   Move-to between two fixed poses; (C) single-joint jogs of the joint being
   tuned at 10, 30 and 60 deg/s each way. Analyse them as
   below (tuning report, decoded on the Mac): each joint's resonance
   frequency, amplitude and decay, whether it rang while commanded still,
   and its servo-loop quality up to 500 Hz (buzz episodes, lines, the loop
   band by speed).
3. Set the joint's tuning value in the machine configuration with its
   measurement note, commit, and update the cell pin.
4. Deploy configuration-only: `realtime-core release --plan` must say
   `config` (or `realtime-core config --dry-run`, then
   `--accept-configuration <new sha256>`). It accepts values that differ
   between axes only for keys the compiled `tuning.json` lists.
5. Check the readback lines before Home or Arm (`realtime-core verify`):
   `EtherCAT startup readback ... verified=1` for every written key and a full
   `config_verified_mask`.
6. Re-run the same moves and report against the previous iteration's
   `metrics.json`; keep or revert.
7. Record the result in the cell's commissioning document.

The first deploy carrying the tuning catalog is tier 3 (an older rtctl
refuses `observe` and the tuning keys); later iterations are tier 1. Revert a
key with an explicit value (its factory default), never by removing it.

### Analysing a tuning iteration (tuning report)

Every tuning iteration is analysed the same way by
`rt-core/tools/tuningreport` (metrics, validation and limits in its README):
right after the test moves, copy the dumps to the Mac and report against the
previous iteration's `metrics.json`, from `steamdeck/real/v4`:

```sh
go run . realtime-core copy-dumps --manifest rt-core/config/cells/<cell>.json --to mac && (cd ../../../rt-core && go run ./tools/tuningreport report --dumps ~/.rosie/stop-dumps/<cell> --since 45m --manifest config/cells/<cell>.json --label "<what changed>" --out ~/.rosie/tuning-reports/<iteration> --baseline ~/.rosie/tuning-reports/<previous>/metrics.json)
```

It decodes on the Mac, never on the servo host, and takes every bound from
the cell's machine configuration. Drop `--baseline` for the first run. Read
both the 1-30 Hz ring section and "Servo loop quality" (every joint up to
500 Hz: buzz episodes, lines, the loop band by speed), and judge them by the
cell's per-joint plan and pass criteria.

### Cell camera (`camera install`, `camera status`)

A cell whose manifest has a `cell_deploy.camera` block runs the camera
service from a built tree. It is not a tier: it touches no controller, takes
no cell lock and sets no maintenance notice.

```sh
go run . camera install --manifest <cell> [--deployment <sha>] --dry-run
go run . camera install --manifest <cell> [--deployment <sha>]
go run . camera status  --manifest <cell>
go run . camera install --manifest <cell> --camera <instance>
go run . camera status  --manifest <cell> --camera <instance>
```

Without `--camera` both act on the cell camera. With it they act on the
camera of that instance, the cell camera's or one in
`cell_deploy.extra_cameras`: one camera a run, so installing a second angle
never restarts the first.

A wrist camera is one of them: an `extra_cameras` entry with instance
`wrist`, so `--camera wrist` installs or inspects it.

`install` uses the tree the OLP unit runs unless `--deployment` names another
(the tree must hold `cell-camera/v1`; `realtime-core build` puts it there).
It stops and disables the units in `camera.replaces`, renders
`/etc/rosie-cell-camera/<instance>.env` from the block, runs the tree's
`cell-camera/v1/host/install.sh <instance> <settings> --start` under sudo, and
then reads the camera's status and one frame. Without a frame it stops the
camera, starts the replaced units again and exits 5. `status` is read-only
apart from asking for one frame. After the first install of a camera on a
cell, run `olp install` so the OLP unit carries the camera's origin.

A RealSense camera's install passes the block's `realsense.sdk_dir` to
`install.sh --realsense-sdk`, which refuses a package the relay's
`/usr/bin/python3` cannot load, copies it beside the service and installs
the udev rule that lets the service open the camera. Build the package on
the relay first, once, under the cell lock (`realtime-core lock --manifest
<cell> --reason "build pyrealsense2" -- bash
<tree>/cell-camera/v1/host/build-pyrealsense2.sh`), or build it on an x86_64
host with `--arch aarch64` and copy the directory to `sdk_dir`.

### Tier 4: Steam Deck v5 pendant rebuild

The pendant's bundled `olp-core.js` refuses any capabilities digest but the
one it was built with, even when it uses the cell host's OLP server
(`pendant.env`). Rebuild whenever the digest or protocol minor changes, or the
pendant, `olp-core.js` or meshes change; a configuration-only change does not
need it in remote-OLP mode (a Deck running its own local OLP server needs a
rebuild and new `olp-cells.json` pins for every configuration digest). Build
and install from the commit the cell runs with the cell manifest's
`pendant_deploy` block (builder, Decks):

```sh
go run . pendant build --manifest <cell> --commit <sha> [--with-tesseract-env]
go run . pendant install --manifest <cell> --host <deck> --commit <sha> [--dry-run]
go run . pendant rollback --manifest <cell> --host <deck>
```

`install` keeps the Deck's site files, refuses while a running pendant's
robot is in use (with no pendant running it checks no robot), verifies
hashes and libraries, relaunches a running
app and restores the previous package on a failed check
([workflow](../../../docs/DEPLOY_AND_TEST_WORKFLOW.md#steam-deck-pendant)).
It touches an operator's Deck: get separate approval. `pendant status --host
<deck>` is read-only and includes the Deck's tunnel when the Deck has one.

The pendant's OLP is resolved as `run.sh` resolves it: `pendant.env`'s
`ROSIE_PENDANT_OLP_ORIGIN`, else the export in the Deck's `launcher` (a site
script in `app_dir` that its `steam_app_id` shortcut runs before `run.sh`,
for example `run-<relay>.sh`), else the local server.
`install` also asks the OLP each running app was started with (`--olp`): a
Deck with a second shortcut to another cell runs the same app. The relaunch
starts the shortcut the running app was started from (Steam's reaper line
`SteamLaunch AppId=<id> -- <app_dir>/<script>`), not `steam_app_id`'s, so an
app open on the other cell's shortcut comes back on that cell; with two
shortcuts open it starts neither.

`install` writes the manifest's cell id into that launcher (after its shebang)
or, without one, into `pendant.env`, as `ROSIE_PENDANT_EXPECT_CELL`; `run.sh`
passes it to the app as `--expect-cell`, and the app refuses to arm or move
while its OLP reports another cell (v5 README, "One pendant per Deck"). A
launcher that already names another cell is refused before anything changes:
the manifest's `launcher` or that file is wrong. A Deck entry with a
`tunnel` block also gets the tunnel's origin: where neither `pendant.env` nor
the launcher names one, `install` writes
`ROSIE_PENDANT_OLP_ORIGIN=http://127.0.0.1:<olp_port>` into the same file,
before the relaunch, so the pendant uses the cell's OLP through the tunnel and
the relaunch is checked through it; a file that names another origin is
refused before anything changes, and a failed check puts the file back with
the old package. On a Deck's first deploy run `pendant tunnel install` first:
it passes on the tunnel's own health and names the pendant that does not use
it yet. `status` prints the cell and
names pendants running from another directory than `app_dir` (a hand-made
copy `install` does not update). Why: a hidden pendant for one cell once
armed that cell with presses meant for another cell's pendant on the same
Deck.

One app per Deck, one launcher per cell. A Deck that serves two cells runs
one app from two shortcuts, one launcher per cell (`run-<relay>.sh`). The
owning cell's manifest installs the app; the other cell's Deck entries name
`"app_owner": "<owner cell>"` and `"launcher": "run-<other relay>.sh"`. On
such a Deck, `pendant install` with the other cell's manifest only writes
that cell into its launcher (`deck-install.sh` `ACTION=cell`). It uploads nothing,
swaps nothing, stops nothing and leaves `pendant.env` alone, because a
shared app's `pendant.env` is read by every shortcut. `--commit` is
optional; when given, it must be the installed app's commit. The command
names any running pendant against that launcher's OLP that started without
`--expect-cell`, since the check applies from its next start. With that
manifest, `rollback`, `--package` and `pendant build` are refused and name
the owner's manifest. A block whose Decks all name `app_owner` needs no
`builder`. Order on a Deck: install the package with the owner's manifest,
then `pendant install --manifest <other cell> --host <deck> --commit <same>`
for each other cell's launcher.

#### Deck tunnel (`pendant tunnel`)

A pendant in remote-OLP mode reaches the cell host's OLP through an ssh
tunnel on the Deck: SteamOS runs Tailscale in userspace-networking mode, so
apps on the Deck cannot reach tailnet addresses, and the Deck uses the relay's
LAN address instead. `pendant_tunnel.go` renders that tunnel from the Deck's
`pendant_deploy.decks[].tunnel` block and keeps it up:

```sh
go run . pendant tunnel install  --manifest <cell> --host <deck> [--wait 30s] [--dry-run]
go run . pendant tunnel status   --manifest <cell> --host <deck>   # exit 1 when unhealthy
go run . pendant tunnel logs     --manifest <cell> --host <deck> [--lines 40] [--follow]
go run . pendant tunnel restart|stop|rollback --manifest <cell> --host <deck>
```

| Block key | Meaning |
|---|---|
| `relay_host` | the relay as the Deck reaches it (its LAN address); the ssh user is `cell_deploy`'s relay user unless `relay_user` names another |
| `relay_auth` | `key` (default): `identity_file` below. `tailscale_ssh`: a Deck whose Tailscale has a kernel tun device reaches the relay's MagicDNS name and the relay's Tailscale SSH server admits it by tailnet identity; no key, `relay_user` required (the account the tailnet SSH policy maps the Deck to), and that policy, not an `authorized_keys` line, bounds the session |
| `tailscale_nc` | with `tailscale_ssh` only: the absolute path of the Deck's `tailscale` CLI when its Tailscale runs in userspace mode (for example `/home/deck/.rosie/bin/tailscale`). Apps there cannot reach a tailnet address, so ssh connects through `<cli> nc %h %p` as its `ProxyCommand` |
| `identity_file`, `known_hosts` | the Deck's own key and the relay's pinned host key, provisioned by hand with the relay's `authorized_keys` restriction (`permitopen`, `from=`); the command only names them. `tailscale_ssh` takes `known_hosts` only |
| `olp_port` | the Deck's local port for `cell_deploy.olp.listen`; the pendant's `ROSIE_PENDANT_OLP_ORIGIN` must be `http://127.0.0.1:<olp_port>` (status checks) |
| `forwards` | further `-L` forwards (for example `18443`, rt-control for the Deck-local OLP) |
| `replaces` | hand-made user units the install takes over |
| `state_dir` | optional; default `/home/<ssh_user>/.rosie/pendant-tunnel/<cell>` |

The unit `rosie-pendant-tunnel-<cell>.service` (systemd --user,
`Restart=always`, `StartLimitIntervalSec=0`, `WantedBy=default.target`: it
starts with the deck user's manager, which SteamOS logs in at boot, and keeps
running through sleep and resume) runs the same supervisor as `olp ui`:
ServerAlive 5 s x 3, ssh restarted 1 s after a dropped link, doubling to 30 s
while the relay stays unreachable, the reason in `tunnel.state`, an orphaned
attempt reaped. The state directory holds `tunnel.sh`, `tunnel.state`,
`tunnel.log`, `rollback/` (the unit files and enabled/active states before the
last install) and `replaced/` (the hand-made units' files, never overwritten).

`install` leaves a current, healthy tunnel alone and restarts a dead one at
once. Otherwise it saves the rollback copy, stops and disables the replaced
units, starts the rendered one and waits for OLP health through it, printing
how long the pendant had no OLP. If the old tunnel had health and the new one
does not get it within `--wait`, the old one is put back (exit 5). A port held
by a process outside these units is refused. `install` (when it would replace
a running tunnel), `restart`, `stop` and `rollback` refuse (exit 2, nothing
changed) while the OLP reports the robot armed, leased, playing or jog-held by
this Deck's pendant (`rosie-pendant-v5 (<hostname>)`) or by a client it does
not name: losing the tunnel lapses the pendant's 1 s heartbeat against the
OLP's 5 s deadman and the OLP Stops the robot. The Mac's browser holding the
robot does not block it. `stop` also disables the unit.

When the link drops, the pendant needs no restart. It polls status every
250 ms with a 2 s timeout and the cell list every 2 s. The first failed status
read shows the robot unreachable and ends any hold: at once when the forward's
port closes, after the 2 s timeout when a dead link still accepts connections.
The pendant picks the robot up again when the tunnel is back. If the outage
outlasts the OLP's 5 s deadman while the pendant has the robot armed, the OLP
Stops it (`steamdeck/real/v5/src/machine.cpp`, code reading). The Deck is reached with `tailscale ssh` like
`pendant install`, and an offline Deck is refused in ~1 s.

State (2026-10-08): tested against a fake systemd --user and on Linux
(ubuntu:24.04, bash 5.2); not yet installed on a Deck. Unverified on SteamOS: `ps -o command=`, user lingering,
systemd --user restart pacing after resume; ServerAlive 5 s x 3 and the
backoff are chosen, not measured.

## Configuration-only deploy

`realtime-core config` changes one input on a running native cell: the cell's
`rt_core.machine_config` JSON. Binaries, drive profiles, robot descriptions,
units, drop-ins, PKI, anchors and enablement stay the installed release's. It
restarts only `rosie-rt-core` and `rosie-rt-control`. It is for values such as a
drive startup SDO, a limit or the tracking allowance. Anything else needs
`realtime-core deploy`, including any change that needs a newer rtctl or
native build (a new configuration key, a protocol minor such as 1.14).
First used on a Rosie 1430 cell for three tuning steps on one joint
([Tier 1](#tier-1-configuration-only-realtime-core-config-0414a46d)).

```sh
cd steamdeck/real/v4
go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --dry-run
go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --accept-configuration <new sha256>
```

The Mac reaches only the relay. `--transport tailscale` (default)
runs `tailscale ssh <relay user>@<relay>`, which needs no Mac key; `--transport ssh`
uses BatchMode ssh to the manifest host. Scripts travel as the remote command
argument, never on stdin. The relay reaches the RevPi over the manifest route
with one multiplexed ssh connection. Other flags: `--olp restart|skip` (default
restart), `--ready-timeout` (default 60s, the measured ~18 s to
`config_verified_mask=63` plus margin), and `--home-timeout` (default 45s,
UNCONFIRMED; the first three runs needed 1-3 s after verification).

The machine bytes are always the worktree file. A run is labelled `committed`
only when those bytes equal `git show HEAD:<machine>` and compile to HEAD's cell
pin; otherwise it is a `trial`. The tool prints the new digest, and only
`--accept-configuration <that digest>` (or typing its first 12 characters at a
terminal) applies it. Since 2026-10-11 an apply takes only a `committed`
configuration (robot-specific settings live in the repository; it replaces
an earlier allowance of uncommitted trials, after a cell ran speed
feedforward 0/0 from an uncommitted edit that the next release would have
reverted): `config` refuses, before contacting anything, any
uncommitted change (tracked or untracked) under `rt-core/config`,
`robot_description` or the cell manifest, and refuses a trial label.
`--dry-run` still evaluates an edited tree, warns that it is not committed and
prints the pin to commit. A RAM-only trial of a drive value is a committed
`startup_trial` key (`rt-core/docs/CONTRACTS.md`, "Startup write on change"),
not an uncommitted file.

Drift guard (`config_drift.go`, also in `release`): when the running
configuration matches no commit reachable from HEAD (`git log -S<pin>` over
`rt-core/config/cells`, ~0.2 s), the run prints `DRIFT WARNING ...` and every
startup value (`startup_sdos`) and stored/RAM choice (`startup_trial`) that
differs between the running compiled `axes.conf` and the new compile, per axis,
running -> new. Any difference is refused unless `--accept-config-drift
<running sha256>` names the running pin; identical values pass with the
warning. `release` reads the running `axes.conf` in its read-only probe
(`sudo -n cat` on the servo host, only that file) and compiles the commit with
the commit's own rtctl in a temporary worktree on the Mac, only when the pin is
unknown; its config tier passes the acceptance to the config step.

Session 1 (the whole of `--dry-run`) is read-only on both hosts. It finds the
OLP deployment from the cell's OLP unit (`cell_deploy.olp`), reads the RevPi release,
`rt_package.json` hash, `control.env`, `rtctl describe/status`, both units'
state, enablement and NRestarts, the anchor directory and file hashes, stop
dumps and the deploy lock, and copies the running compiled identities. It then
compiles the JSON with the installed build's own `rtctl`. That is the relay's
`runtime-package` copy when its `rt_package.json` matches the RevPi's;
otherwise it is the RevPi's `rtctl` at `nice -n 19`.
`configuration_sha256` hashes input bytes, not compiler semantics, so only the
installed compiler can judge the JSON: an older rtctl files a key it does not
know as deployment metadata, and the cell then runs a matching pin without
that feature (`rt-core/docs/CONTRACTS.md`, "A configuration digest does not
identify its compiler").

The Mac refuses (exit 2, nothing changed) when:

- the deploy lock is held, or the probe could not read a fact;
- the installed release cannot compile the JSON (full deploy);
- `coordinate.identity.json` or any axis `coordinate_evidence`/`coordinate_identity`
  line changes: that would invalidate Home, and those joints would need
  operator HM35 at the mechanical home pose;
- any deployment-identity key not starting with `_` is added, removed or
  changed. Keys the installed compiler does not know land there and are
  silently ignored by the binary (for example `jog_jerk` on 6c6f97c0). The
  robot block and runtime, control, telemetry or host-placement edits also
  need a full deploy;
- a startup key outside the new compiled `tuning.json` is removed or varies
  between axes. Catalogued servo-tuning keys may differ per joint or be removed
  to leave current drive values unmanaged. Missing or malformed catalog data
  grants no exception. Removal is reported as "no longer managed; current drive
  value retained": it restores neither defaults nor earlier tuning. RAM-only
  trials remain volatile; an older configuration that writes the key manages
  it again. Other readiness, drift, Home and idle checks still apply;
- the backend is not live/ethercat; the controller is armed, enabled or holds a
  lease (an idle OLP or Steam Deck session holding the lease blocks the deploy:
  release it); there is a safety/execution fault or drive alarm;
  `config_verified_mask` is not full; Describe differs from `control.env`; a
  unit is not active; the anchor directory is not `rosie-ctl:rosie-rt 2750` or a
  file is not 0640;
- a stop dump in `/run/rosie-rt-core/native/telemetry` has no copy in the
  cell's `dump_dir` with the sha256 `copy-dumps` recorded: run `copy-dumps`
  first, the restart deletes `/run/rosie-rt-core` (`RuntimeDirectoryPreserve=no`);
- with `--olp restart`, the OLP tree does not pin the running configuration.

When the new digest equals the running one, the run is a no-op and nothing
restarts. Other `axes.conf` changes (limits, tracking allowance, feedforward)
are printed as the diff.

Session 2 runs one script on the RevPi. It takes
`/run/lock/rosie-rt-config-deploy.lock`, re-checks every probe fact, stages the
JSON at `/etc/rosie-rt-core/config-history/machines/<new>.json` and backs up
`control.env` to `config-history/control-env/<UTC>-<old>.env`. Each phase is
appended to `config-history/journal.jsonl`. It re-checks disarmed, stops
`rosie-rt-control` and then `rosie-rt-core`, and runs the installed
`generate-control-env.sh --keep-remote-args` on the staged file. The published
digest must equal the precompile, and the `ROSIE_RT_REMOTE_ARGS` line must be
unchanged. It then starts both units and polls every second (progress every
2 s) until Describe is the new digest, the backend is ethercat,
`config_verified_mask` is full, `home_valid_mask` is back to its pre-deploy
value, there are no faults and the robot is disarmed. A full mask already means
every startup SDO readback verified. The first unexpected unit restart counts
as a failure, because `Restart=on-failure` with `RestartSec=2` would otherwise
rewrite drive parameters on every attempt. The anchor files and unit enablement
must be unchanged at the end. The script ignores SIGHUP and SIGPIPE, so a dropped
connection cannot leave it between stop and start. It never runs `install.sh`,
hostcheck, `systemctl enable/disable`, `ethercat.service`, or any Home, HM35,
anchor-restore or verified-delta command.

After the robot verifies, `--olp restart` backs up the OLP deployment's machine
JSON, cell manifest and `olp-cells.json` to `<deployment>/config-history/`. It
replaces the machine JSON and the single cell pin, updates the catalogue
binding, restarts the cell's OLP unit, and waits for health and `/targets`. If
that fails, the OLP files are restored and the robot stays on the new
configuration (exit 5). The OLP then refuses the cell on its pin check until it
is updated.

Exit codes: 0 applied and verified, or no-op; 2 refused, nothing changed; 3
failed after the stop, `control.env` restored atomically and the old
configuration verified; 4 rollback failed, both units stopped (bus owner none)
and manual steps printed; 5 robot verified but the OLP update failed and was
restored.

Every line prints as `[  12.3s +1.2s] phase detail`. The transcript, including
both rendered scripts, goes to `~/.rosie/deploy-journal/<run>.log`, and one JSON
line per run to `~/.rosie/deploy-journal/rt-core-config.jsonl`.

Whenever the pins differ (an apply now refuses trials, so this is a stale
committed pin or a hand-made change on the host), the repo cell pin is stale.
`realtime-core start` and `deploy` refuse until the value is committed (machine
JSON plus re-pins of the cell manifest and
`offline-programming/v1/internal/targets/cells_test.go`) or rolled back. The
next `olp install` overwrites the OLP edit, and the Steam Deck v5 pins are
stale. To roll back, run `config` again with the previous machine JSON (the OLP
backup holds it), or swap `control.env` by hand on the RevPi (the tool prints
the exact backup path):

```sh
sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service
sudo -n install -m 0640 -o root -g rosie-ctl /etc/rosie-rt-core/config-history/control-env/<UTC>-<old>.env /etc/rosie-rt-core/.control.env.manual
sudo -n mv -f /etc/rosie-rt-core/.control.env.manual /etc/rosie-rt-core/control.env
sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service
```

Rolling back does not revert a drive parameter that the old configuration does
not write. Every restart makes rt-core rewrite all startup SDOs on every axis
(on a Rosie 1430 cell 7 per axis: 4 drive-profile defaults plus 3
feedforward keys; 8 on a joint that adds C00.06). The
A6-EC stores SDO writes by default (C13.10 = 1, chapter 11; see
`rt-core/docs/drives/a6ec/README.md`, "Persistence and reverting") and raises
ALF1.3 on too-frequent EEPROM writes; the wear per restart is unquantified, so
iterate deliberately. To turn a drive parameter off, write its off value
explicitly. A configuration-only deploy does not
bypass a failing hostcheck (such as an IgH module older than the package's):
the full deploy still fails its health gate until the host is fixed. Old
`/etc/rosie-rt-core/compiled/<sha>` directories are not pruned.

## Local simulation and checks

Use Linux/WSL with Go, Make and a C++17 compiler. In WSL create a tmpfs directory
and set `TMPDIR` there because Unix sockets cannot bind on `/mnt/c`.

```sh
export TMPDIR=/dev/shm/rt-core/deck
mkdir -p "$TMPDIR"
make -C rt-core control sim
make -C motion-server/joint-trajectory/v1 all
make -C motion-server/v1 all
./dev-stack.sh start
./dev-stack.sh status
./dev-stack.sh stop
```

The stack starts `rt-sim`, `rt-control`, the dense daemon and the
OLP/planner services. The dense binary directory comes from its Makefile's
`print-bin-dir` target. See repository `CLAUDE.md` for service prerequisites,
ports, private runtime directories and generated bindings.

```sh
make -C motion-server/v1 all test test-composed
make -C motion-server/joint-trajectory/v1 all test
cd steamdeck/real/v4
go build ./...
go vet ./...
go test -race -count=1 -p=1 ./...
```

On a shared host, pin discovery to a closed loopback daemon endpoint for
offline tests. Runtime, capture and hardware tests retain their
own explicit prerequisites; software checks do not qualify physical motion.

## Health failure and release recovery

A failed native health gate stops the rosie services, leaves v4 units masked
and reports no bus owner. It does not roll back automatically. Correct the
reported prerequisite and repeat native deployment, or use the published
`robot-stack-v0.2.0` tarball with `rtcore-v0.2.0`, `rtcore-bridge-v0.2.0` and
`pair-owner-v0.2.0` assets (or the recorded v0.1.0 set for cells installed from it).
Verify checksums and exact retained receipts/configuration, keep motion
inhibited, and install the verified binaries into the recorded versioned slots
using the retained release installation procedure. Restore the release units
and configuration before unmasking them. Current v4 manifest deployment is
refused and cannot perform this rollback.

See [rt-core migration](../../../rt-core/docs/MIGRATION.md#repository-defaults-and-deployed-cutover)
for the release boundary.
