# Rosie standalone realtime core

`rt-core` is RosieOS's only runtime source. `rosie-rt-core` is the C++ cyclic daemon: it owns drive state and final output permission, using IgH EtherCAT on PREEMPT_RT
Linux. `rosie-rt-core-sim` exercises that state machine without hardware. The separate Go adapter, `rt-control`, owns the public HTTP/JSON API over a Unix socket,
optional remote mutual TLS, one expiring controller lease, an independent jog lane, and prepared trajectory/program handles. Native IPC is private to the adapter.

Applications use the [Go SDK](sdk/control/), [C++ client and generated contracts](clients/cpp/), or [generated TypeScript contracts](clients/ts/). Current consumers are
the [offline-programming app](../offline-programming/v1/) for dense execution and local simulation jog, the
[Cartesian motion server](../motion-server/v1/) through its `rt_core` backend, and [the dense daemon](../motion-server/joint-trajectory/v1/). Only one consumer may
control a core at a time; release local jog before dense execution or pendant control.

`rtctl` handles configuration and one-shot operations; `benchdrive` runs bench procedures; `natspublisher` builds the observation-only `rt-natspublisher`; `soak`
exercises simulation; `lcddash`/`lcdjog` provide the mini bench LCD and hold-to-jog input. Software success does not qualify powered motion. Read
[architecture](docs/ARCHITECTURE.md), [contracts](docs/CONTRACTS.md), [application API](docs/APPLICATION_API.md) and [dataflow](docs/DATAFLOW.md) for the detailed
contracts; [telemetry ring](docs/TELEMETRY_RING.md) lists what every 1 kHz record carries. Drive servo tuning is per-joint machine configuration from
the drive's tuning catalog (`config/drives/a6ec.tuning.json`), written and read back at startup; see [tuning a joint](docs/drives/a6ec/README.md#tuning-a-joint-2026-10-07).

## Directory map

| Directory | Responsibility |
|---|---|
| `adapters/` | Rosie public control service and dense program codec. |
| `clients/` | Public C++ client and generated C++/TypeScript contracts. |
| `cmd/` | `rt-control` executable entry point. |
| `config/` | The one configuration tree: `drives/`, `robots/`, `machines/` (with `bench/` and `simulation/`), `cells/`, `cell-io/`, and `templates/` holding the annotated class definition of each layer; see [config/README.md](config/README.md). |
| `docs/` | Contracts; `docs/drives/` drive references, `docs/evidence/` run records, `docs/history/` checkpoints. |
| `engine/` | Cyclic machine, execution and commissioning policies. |
| `host/` | Install, PKI and systemd; `host/foundation/` NIC/CPU/IRQ services, `host/minibench/` panel setup. |
| `include/` | C++ bus, safety, coordinates, protocol and telemetry primitives. |
| `internal/` | Go Unix HTTP, procfs and test/simulation support. |
| `ipcclient/` | Private native IPC implementation and integration tests. |
| `protocol/` | Native/public schemas and cell-bus schema. |
| `sdk/` | Public Go client in `sdk/control/`. |
| `simfixture/` | Reusable composed simulator fixtures. |
| `src/` | C++ daemon composition entry point. |
| `tests/` | C++ contracts, fixtures and parity records. |
| `tools/` | Commands and build/test utilities listed below. |
| `build/` | Gitignored binaries, generated configurations, caches and local outputs. |

Every `tools/` entry: `rtctl` (compiler/control/host CLI), `benchdrive` (bench procedures), `natspublisher` (NATS observations), `soak` (simulation exerciser), `lcddash`
(LCD panel), `lcdjog` (panel jog producer), `rdtcheck` (offline dense-file diagnostics), `protocolgen` (native generator), `apigen` (public generator), `a6ecgen` (drive
alarm tables), `fixturedelta` (fixture expansion), `packageinfo` (package identity/verifier), `buildcontract` (build regressions), `cidriver` (CI gates),
`build-igh-userlib.sh` (pinned IgH build), `package-runtime.sh` (release staging), `verify-package.sh` (integrity).

## Build and test

Use Linux/WSL, make, Bash, a C++17 compiler and pthread support. [go.mod](go.mod) declares Go 1.24, toolchain Go 1.26.2 and `golang.org/x/sys v0.34.0`. Race/sanitizer
checks need a native compiler; TypeScript conformance needs Node at `$HOME/.rosie/node-22.13.1/bin/node` or `NODE`. Stack helpers need Python 3 and curl. Live builds need
pkg-config and the selected IgH libethercat; [Makefile](Makefile) defaults to 1.6.9, while the Pi host profile selects 1.6.12. From `rt-core/`, keep WSL sockets on Linux
tmpfs: they cannot bind on `/mnt/c`. Host/package checks also use OpenSSL and standard Linux archive/binutils tools.

```sh
export PATH="$HOME/.rosie/go-1.26.2/bin:$PATH"
export GOCACHE=$PWD/build/go-cache GOMODCACHE=$PWD/build/go-mod-cache
export GOTMPDIR=$PWD/build/go-tmp GOFLAGS=-mod=mod TMPDIR=/dev/shm/rt-core/readme
mkdir -p build/go-cache build/go-mod-cache build/go-tmp "$TMPDIR"
make control sim
make test
make test-go
make test-sanitize
```

| Target | Contract |
|---|---|
| `make control sim` | Build adapter, rtctl, benchdrive, observation publisher and simulator. |
| `make test` | C++ contracts, Go oracle parity, generators and confined host/package tests. |
| `make test-go` | All Go packages with race detection and uncached test execution. |
| `make test-sanitize` | ASan/UBSan C++ cell I/O and native oracle checks. |
| `make check-protocol`, `make check-api` | Refuse generated-contract drift. |
| `make generate-protocol`, `make generate-api` | Regenerate contracts; edit schemas, not generated files. |
| `make live` | Build hardware daemon; refuses missing/mismatched IgH metadata. |

All targets were checked with `make -n`. The live binary also built using `bash tools/build-igh-userlib.sh`, then
`PKG_CONFIG_PATH="$PWD/build/deps/igh-prefix/lib/pkgconfig" make live`; it was not started. Host requirements are in [migration](docs/MIGRATION.md).

## Run simulation

At repository root, [dev-stack.sh](../dev-stack.sh) accepts `start`, `status`, `stop`, `restart`, `logs` and optional service names. Without names it selects catalog
(:8787), NATS (:14222, monitor :18222), rt-sim, rt-control, dense daemon (TCP :8797), planner motion (:8796), OLP API (:8794) and UI (:5189).
Open `http://127.0.0.1:5189/offline-programming/v1/ui/`; full-app dependencies are in the [launcher notes](../CLAUDE.md). This verified subset starts only the core and
adapter:

```sh
cd ..
export ROSIE_DEV_STACK_DIR=$TMPDIR/stack ROSIE_LOCAL_RT_DIR=$TMPDIR/stack-rt
export ROSIE_LOCAL_RT_BUILD=$PWD/rt-core/build/readme-stack
export DEV_STACK_UI_HOST=127.0.0.1 DEV_STACK_FIREWALL=0
./dev-stack.sh start rt-sim rt-control
./dev-stack.sh status rt-sim rt-control
./dev-stack.sh stop rt-sim rt-control
cd rt-core
```

Status reported `backend: rt_core`, both services `healthy`, and native/control/jog sockets at `/dev/shm/rt-core/readme/stack-rt/{ipc,control,jog}.sock`. Logs/pids live
in `$ROSIE_DEV_STACK_DIR`; compiled config and consumer bindings in `$ROSIE_LOCAL_RT_BUILD`. Full-stack startup uses
[local-rt-core.sh](../motion-server/v1/local-rt-core.sh); its Home-enabled simulation config has a different identity from the direct example.

For the direct path, compile and launch from `rt-core/`. `rtctl run` recompiles, uses the generated `argv.json` arguments and replaces itself with the selected daemon:

```sh
compiled=$(build/rtctl compile --config config/machines/simulation/simulation-program.json --out build/readme-config)
printf '%s\n' "$compiled"
export SHA=${compiled##*/}
mkdir -p build/readme-runtime
(cd build/readme-runtime && exec ../rtctl run --backend simulation \
  --config ../../config/machines/simulation/simulation-program.json --out ../readme-config \
  --socket "$TMPDIR/ipc.sock") >build/readme-core.log 2>&1 &
core_pid=$!
# Unverified readiness bound: 10 s; success requires the native socket.
deadline=$((SECONDS+10))
until test -S "$TMPDIR/ipc.sock"; do test "$SECONDS" -lt "$deadline" || exit 1; sleep 0.02; done
build/rt-control --core-socket "$TMPDIR/ipc.sock" --socket "$TMPDIR/control.sock" \
  --configuration-sha256 "$SHA" --backend simulation \
  --pair-id readme --pair-revision 1 \
  >build/readme-control.log 2>&1 &
control_pid=$!
until build/rtctl describe --socket "$TMPDIR/control.sock" --json >build/readme-describe.log 2>/dev/null; do test "$SECONDS" -lt "$deadline" || exit 1; sleep 0.02; done
```

Compilation printed this path, nine missing-collision-watchdog warnings and the legacy-flat-axis warning. These simulation defaults are not a hardware recipe.

```text
build/readme-config/835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2
```

The V2 telemetry ring retains 100,000 ms by default. Its measured mapping is
`4096 + ceil(retention_ms*1000000/cycle_ns)*(8 + 272 + 288*axes)` bytes:
nine axes at 1 kHz use **287,204,096 bytes** by default. At 16 axes/250 µs,
the default uses **1,955,204,096 bytes**, below the 2 GiB ceiling; the former
300,000 ms default would require 5,865,604,096 bytes and is refused. Dump files
and public batches identify the V2 layout by digest. V1 dumps require the
matching old `rtctl`; current readers report both digests before decoding.

## Configuration and cell identity

Configuration is one tree, `config/`, with one noun per layer and one annotated template per layer as its class definition ([config/README.md](config/README.md),
[config/templates/](config/templates/)): a **drive config** (`config/drives/<name>.json`) describes one drive model and is selected by name from a machine axis; a
**robot description** (`robot_description/robots/<model_id>/` at the repository root, carried by every runtime package) holds the robot's URDF, meshes, planner facts and rt-core's `rtcore_definition.json`, under one manifest identity; a **machine config** (`config/machines/`, `machines/bench/`, `machines/simulation/`)
binds drives and robot mechanics to bus positions, timing, limits, cell I/O and runtime policy and compiles to one immutable `configuration_sha256`; a **cell config**
(`config/cells/<cell>.json`) binds a deployed cell to a repository-relative `machine_config`, that digest, pair/revision, dense-axis mapping and endpoints such as
`remote_listen`. Compilation emits `axes.conf`, `argv.json` and identity artifacts under the digest directory; configuration, machine and deployment digests are distinct.
[Configuration](docs/CONFIGURATION.md) and [config/cells/README.md](config/cells/README.md) explain the checks. OLP refuses a different compiled/deployed configuration with
`cell_configuration_mismatch`; a different capabilities contract/digest refuses with `rt_core description identity is incompatible`. Reconcile app, cell config and
deployment; do not silently accept a new digest or reuse Home evidence across identities.

| Cell config (`config/cells/*.json`) | Machine config under `rt-core/` | Digest prefix | Installation/role |
|---|---|---|---|
| `linear-rail-1axis` | `config/machines/linear-rail-1axis.json` | `5f2b4e51c7db` | Placeholder one-axis cell: the pattern to copy for a new cell. |

The real cells' and benches' configs are site configuration kept in the private repository. Native listeners pin
`127.0.0.1:8443` unless a cell config names another address; cell config notes define provisioning/tunnel requirements. Six-axis bench bindings retain
missing-watchdog warnings; a matching digest does not qualify them.

## Interact with the public API

These groups summarize [APPLICATION_API.md](docs/APPLICATION_API.md)'s operation table. Remote observations also require authentication; leases fence mutations by
session/generation.

| Surface | Operations/transport | Lease |
|---|---|---|
| Observe | GET `/v1/describe`, `/v1/status`, `/v1/maintenance`, `/v1/jog/clock`, `/v1/jog`, `/v1/jog/ingress`, `/v1/jog/path`, `/v1/events`, `/v1/events/stream`, `/v1/telemetry`, `/v1/telemetry/stream` | No (remote authentication still applies). |
| Session | `acquire`, `renew`, `release`, `stop` via POST `/v1/control`. | Acquire validates binding; others use acquired fence. |
| Machine | `enable`, `arm`, `home`, `halt`, `reset_fault` (interim), `recovery_status`, `restore_anchor`, `io_arm`, `io_disarm`, `mark_telemetry` via POST `/v1/control`. | Yes |
| Jog | `begin_jog`, `end_jog`, `update_jog`; local `jog.sock` datagrams or WSS `/v1/jog` for updates. Path jog (protocol 1.14, Describe `path_jog`, only with machine `jog_jerk`): `begin_path_jog`, then acknowledged path-sample chunks and path-speed updates on the same lanes; `end_jog` stops along the path. | Yes, plus jog generation and source expiry. |
| Trajectory | `prepare_trajectory`, `start_trajectory`, `discard_trajectory` via POST `/v1/control`. | Yes; handle belongs to its generation. |
| Program | `prepare_program`, `start_program`; binary POST `/v1/program`, then JSON POST `/v1/control`. | Yes; upload session/generation headers, then prepared identity. |

JSON `jog` is test-only; `abort` and `readiness` are unimplemented. Stop inhibits outputs and retires handles; its returned generation is the new fence. Halt requests
deceleration to enabled hold. Neither receipt proves physical standstill.

While the deploy tooling's maintenance notice is on the servo host (`/etc/rosie-rt-core/maintenance.json`), rt-control refuses `acquire`, `arm`, `enable`, `home`,
the anchor and reference restores, `reset_fault`, jog begins (also on the WSS jog stream), trajectory and program starts and `io_arm` with `busy: <reason>`; Stop,
Release, Halt, EndJog, Renew, IoDisarm, Discard and Prepare pass. `GET /v1/maintenance`
(capability `maintenance`) reports the notice and the grant holder, never its session; adding it moved the capabilities digest from `b1e28f06…` to `725c5bbb…`
([APPLICATION_API.md](docs/APPLICATION_API.md)).

`build/rtctl --help` prints only compile/validate usage. Dispatch and every subcommand's `--help` were checked for this CLI table (help usually exits 2):

The dump converter treats `--help` as a filename; `rtctl telemetry --help` prints its usage.

| `rtctl` subcommands | Principal inputs/purpose |
|---|---|
| `compile`, `validate`, `run`, `control-env` | `--config`; compile `--out`; validate/run `--backend`; control-env emits deployment binding. |
| `hostcheck`, `inventory`, `recover-encoder` | Host identity `--expected-release`; inventory `--config`; encoder recovery `--backend ethercat --slave` is maintenance. |
| `describe`, `status`, `events` | Observations; events accepts `--after`, `--follow` for SSE. |
| `acquire` | `--controller --pair-id --pair-revision --configuration-sha256`. |
| `release`, `stop`, `halt`, `enable`, `arm`, `home`, `reset-fault` | `--session --generation`; axis operations accept `--axis-mask`. |
| `prepare`, `start`, `discard` | Prepare `--axis-mask --file` points JSON; start/discard `--handle`. |
| `program-prepare`, `program-start` | `--file`: binary `.rdt` or prepared identity JSON respectively. |
| `telemetry dump-to-jsonl` | Positional dump file; offline conversion, no lease. |

Public commands take `--socket` or `--remote` with `--ca --cert --key`, and `--json`. CLI calls do not renew automatically; retain returned fences and use a long-lived
SDK client for motion. There is no CLI `renew` or `jog`. Go starts at `control.Dial(control.UnixPath(...))` or `control.DialTLS` in `sdk/control`; C++ lives in
`clients/cpp/include/rosie/`, TypeScript in `clients/ts/`. Resume SSE at `/v1/events/stream?after=<sequence>` (`rtctl events --follow`); reconcile dropped events and
incarnation changes before acting.

Save this simulation-only acquire/enable/arm/jog/stop/release example as `build/readme-jog.go`. The `ipcclient` helpers only read the local monotonic clock and interpret
public status; authority stays with the SDK.

```go
package main

import (
    "context"
    "fmt"
    "os"
    "time"
    "rosieos/rt-core/ipcclient"
    "rosieos/rt-core/sdk/control"
)

func must[T any](v T, err error) T { if err != nil { panic(err) }; return v }
func check(err error) { if err != nil { panic(err) } }

func main() {
    // Unverified simulation inputs/bounds: nine axes, 0.01 rad/s, 1 s input,
    // 10 ms cadence, 100 ms input expiry, 10 s observation limit, 100 counts minimum travel.
    const mask, velocity, duration, cadence, age, budget, minimumCounts = 511, 0.01, time.Second, 10*time.Millisecond, 100*time.Millisecond, 10*time.Second, 100
    ctx, cancel := context.WithTimeout(context.Background(), budget); defer cancel()
    c := must(control.Dial(control.UnixPath(os.Getenv("TMPDIR")+"/control.sock"))); defer c.Close()
    d := must(c.Describe(ctx))
    if d.Backend != "simulation" || d.ConfigurationSHA256 != os.Getenv("SHA") { panic("simulation identity mismatch") }
    g := must(c.Acquire(ctx, "readme", control.Binding{PairID:"readme", Revision:1, ConfigurationSHA256:d.ConfigurationSHA256}))
    released := false
    defer func() { if !released { c.Stop(context.Background()); c.Release(context.Background()) } }()
    fmt.Printf("acquire generation=%d\n", g.Generation)
    renewals := must(c.StartRenewal(ctx, 0))
    fmt.Printf("enable sequence=%d\n", must(c.Enable(ctx, mask)).Sequence)
    fmt.Printf("arm sequence=%d\n", must(c.Arm(ctx)).Sequence)
    tick := time.NewTicker(cadence); defer tick.Stop()
    for !ipcclient.AllOperationEnabled(must(c.Status(ctx)).Core) {
        select { case <-tick.C: case <-ctx.Done(): panic(ctx.Err()) }
    }
    before := must(c.Status(ctx)).Core.Axes[0].PositionCounts
    jog := must(c.PrepareJogSession(ctx, mask))
    velocities := make([]float64, len(d.Axes)); velocities[0] = velocity
    end := time.Now().Add(duration)
    for time.Now().Before(end) {
        origin := ipcclient.HostMonotonicNS()
        check(jog.UpdateAt(velocities, origin, origin+uint64(age)))
        select {
        case r, ok := <-renewals: if !ok { panic("renewal ended") }; check(r.Err)
        case <-tick.C:
        case <-ctx.Done(): panic(ctx.Err())
        }
    }
    check(jog.End(ctx))
    delta := must(c.Status(ctx)).Core.Axes[0].PositionCounts-before
    if delta < minimumCounts { panic("simulation did not advance 100 counts") }
    fmt.Printf("jog requested_ns=%d delta_counts=%d\n", duration, delta)
    fmt.Printf("stop generation=%d\n", must(c.Stop(ctx)).Generation)
    fmt.Printf("release session_empty=%t\n", must(c.Release(ctx)).Session == "")
    released = true
}
```

Run in the shell that launched the direct simulation, then stop its processes:

```sh
go run build/readme-jog.go
build/rtctl status --socket "$TMPDIR/control.sock" --json > build/readme-status.log
build/rtctl events --socket "$TMPDIR/control.sock" --after 0 --json > build/readme-events.log
kill -TERM "$control_pid" "$core_pid"
wait "$control_pid" "$core_pid"
dump=$(find "$TMPDIR/telemetry" -name '*.bin' -print -quit)
build/rtctl telemetry dump-to-jsonl "$dump" > build/readme-telemetry.jsonl
```

Actual SDK output (values describe this run):

```text
acquire generation=1
enable sequence=1
arm sequence=2
jog requested_ns=1000000000 delta_counts=207
stop generation=2
release session_empty=true
```

Cycle dumps occur on Stop, fault, mark or exit when configured; this run used `/dev/shm/rt-core/readme/telemetry/`. Keep the `.bin` and matching `.json` index together;
`dump-to-jsonl` renders count/ns observations.

## Deploy, qualify and roll back

Deployment is owner-operated. [host/ethercat-foundation.sh](host/ethercat-foundation.sh) provisions the supported host/kernel/IgH tuple;
[host/foundation/](host/foundation/) holds host services; on a RevPi (`revpi-debian12`) the installer installs, enables and verifies them, with the `picontrol` EtherCAT drop-in every
`realtime-core deploy` requires, from `foundation/` beside it (`ROSIE_RT_ETHERCAT_GROUP=picontrol`; first used on a RevPi servo host, 2026-11-13). [host/install.sh](host/install.sh) installs inactive `rosie-rt-core.service`, `rosie-rt-control.service`,
`rosie-rt-natspublisher.service`; activation needs an explicit compiled machine/pair binding. Public names are `/run/rosie-rt-core/control.sock` and
`/run/rosie-rt-core/jog.sock`; `native/ipc.sock` is private. Live admission needs PREEMPT_RT, locked memory, CPU/FIFO admission and a matching IgH kernel master. `rtctl
hostcheck --config` with `--expected-release` checks release/process identity read-only; only a fully matching result qualifies that step.

The native deploy module is [steamdeck/real/v4](../steamdeck/real/v4/). CLI prefix `rosie steamdeck real v4` provides `manifest validate`, `manifest hosts`, `manifest
deploy` (`--manifest`, optional `--dry-run`), `build`, `daemon-bootstrap`, `system-state`, `rollback`. Native manifest deploy is installed-disarmed; automatic legacy
tours/mode-switch requests are refused. Follow [bench testing](docs/BENCH_TESTING.md) for RevPi bench and mini bench recipes, [drive references](docs/drives/) for
diagnostics and mini bench host tools for LCD setup. Powered motion and cell cutover acceptance remain manual, owner-only work.
Deploying to a commissioned cell uses the manifest-driven commands in `steamdeck/real/v4`: `realtime-core release --commit <sha> [--plan]` picks the smallest tier
and runs `realtime-core config|build|precheck|copy-dumps|deploy|start|verify|prune` and `olp install` under the cell lock, with automatic rollback. Usage:
[deploy and test workflow](../docs/DEPLOY_AND_TEST_WORKFLOW.md); step detail: [Deployment tiers](../steamdeck/real/v4/README.md#deployment-tiers).

[Migration](docs/MIGRATION.md) defines packaging, inhibited handover and rollback. `tools/package-runtime.sh` stages the complete runtime and release tarball; retain its
manifest/checksums and previous release. Installer `--from-package` selects inactive artifacts; `--rollback` restores the verified previous selection without starting
units. Published release tarballs preserve installed v4 rollback after source retirement. Re-establish authority and coordinates after rollback; never replay old handles.
Provision mutual TLS with [host/remote-pki.sh](host/remote-pki.sh) (`init`, `issue-client`, `revoke`, `crl`) and installer `--remote-pki`; CA, server/client keys and CRLs belong
outside git. A listener can jog only with a jog-clock policy: give `--remote-pki` all of `ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS`, `ROSIE_RT_REMOTE_JOG_DRIFT_PPB` and
`ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS` (the Rosie 1430 cells run 100000000 / 2000000 / 10000000000); without them it warns and rt-control refuses every remote jog.

The CRL expires. rt-control refuses every client once the CRL's nextUpdate passes ("remote CRL not current"), and `remote-pki.sh` signs it for 7 days
(`default_crl_days`). `rosie-rt-crl.timer` re-signs it daily with `remote-pki.sh crl`. `rosie-rt-control.service` Wants= it, so it runs exactly while the
controller does, and at once when the controller starts after its CRL has lapsed (a lapsed CRL once locked every client out before the timer existed). It reaches a host with
the runtime package; while the controller is stopped the timer is too.

Next: the deploy commands check only that `crl.pem` exists (`steamdeck/real/v4` `rt_core_relay_deploy.go`), not that it is current, and a lapsed CRL
shows only as "live Describe ... failed" (`olp install`, 2026-10-19). Have `precheck` and `status` print the days until nextUpdate and warn under 3.

## Where the truth lives

The tracker owns Backlog, Specified, Building, Built, In review, Reviewed, Merged and Verified stages. Blocked/Deferred/Dropped are flags; merged is
not verified acceptance. Browser drafts require export and commit. [The plan](docs/IMPLEMENTATION_PLAN.md) owns dependencies/criteria. Evidence contains
run records of the benches; their stated
limits apply. [History](docs/history/README.md) preserves checkpoint findings and later removal outcomes.
Read their status labels alongside current code; this README run established no hardware or deployed-cell acceptance.
