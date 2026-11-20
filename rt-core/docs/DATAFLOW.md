# rt-core: how it all connects

One page for the whole picture: the processes, what talks to what, what flows every
millisecond versus on request, and how a robot or bench is described and loaded. The
detailed contracts live in [APPLICATION_API.md](APPLICATION_API.md) (public API),
[ARCHITECTURE.md](ARCHITECTURE.md) (internals), [TELEMETRY_RING.md](TELEMETRY_RING.md)
(what every 1 kHz record carries) and [CONTRACTS.md](CONTRACTS.md) (rules
with their regressions). How the offline-programming app finds and controls a cell
is in [offline-programming CELL_CONNECTION.md](../../offline-programming/v1/docs/CELL_CONNECTION.md). Diagrams are Mermaid; GitHub renders them inline.

## 1. Processes and connections

```mermaid
flowchart LR
  subgraph Drives["EtherCAT bus (1 kHz)"]
    D1["Drive J1<br/>A6-EC"] --- D2["…"] --- D9["Drive J9"]
  end

  subgraph RT["Controller host (PREEMPT_RT), one per cell"]
    IGH["IgH EtherCAT master<br/>(kernel module + userspace lib)"]
    CORE["rosie-rt-core<br/>C++ daemon, 1 ms cycle<br/>engine/*.hpp"]
    CTRL["rt-control<br/>Go adapter<br/>adapters/rosie/control"]
    PUB["rt-natspublisher<br/>diagnostics only"]
    ANCH[("anchors/axis-N.anchor<br/>ROSIE_RT_ANCHOR_DIR")]
    SHM[("shared memory rings<br/>status, telemetry, commands")]
  end

  subgraph Clients["Consumers (any host on the pair's network)"]
    MS["motion-server<br/>joint-trajectory/v1 (programs)<br/>v1 cartesiand (Cartesian jog → joint jog)"]
    OLP["offline-programming<br/>dense execution backend"]
    PEND["pendant (Steam Deck)<br/>local or WSS jog producer"]
    BENCH["tools/benchdrive, rtctl<br/>operator and qualification tooling"]
    READ["status and telemetry readers<br/>(system-state CLI, UI, NATS)"]
  end

  Drives <-- "cyclic PDOs: targets out,<br/>position/status/error in" --> IGH
  IGH <-- "domain process data + mailbox SDOs" --> CORE
  CORE <-- "binary IPC over Unix socket<br/>ipc.sock + shared memory" --> CTRL
  CORE --> SHM
  CTRL --> ANCH
  CTRL -- "public HTTP/JSON over Unix socket<br/>control.sock (+ mutual-TLS remote)" --- MS
  CTRL --- OLP
  CTRL --- BENCH
  CTRL -- "jog.sock datagrams or WSS /v1/jog" --- PEND
  CTRL -- "GET /v1/status, /v1/telemetry, /v1/events" --- READ
  PUB -- "reads telemetry batches" --> CTRL
  PUB -- "NATS: native status/telemetry;<br/>compatibility v4-shape status" --> READ
```

Rules the picture encodes:

- The daemon owns the bus and has exactly one client, the adapter. Nothing else can
  reach the drives.
- The adapter is the only public surface. Every motion-capable request needs a lease
  (acquire, renew, release) bound to the pair identity and the configuration digest;
  Stop is a fence that works even from an expired session.
- The daemon is joint-space only. Cartesian intent is resolved upstream (motion server)
  into per-axis joint velocity before it reaches the jog lane.
- The publisher observes; it never commands.

## 2. Configuration: how a robot or bench is described and loaded

```mermaid
flowchart TD
  subgraph DESC["Robot description<br/>robot_description/robots/&lt;model_id&gt;/ (repository root; a runtime package carries a copy)"]
    URDF["robot.urdf<br/>authoritative geometry, mapped joint types and limits"]
    MODEL["rtcore_definition.json<br/>rosie.robot-definition.v1, model revision<br/>joint mapping (urdf_joint), reductions, encoder counts,<br/>logical signs, Cartesian participation and default policies"]
    MAN["robot_description_manifest.json<br/>registers the files the robot is made of;<br/>robot_description_sha256 = SHA-256 over the manifest and every registered file"]
  end
  subgraph PKG["Runtime package"]
    PROF["Drive config<br/>config/drives/&lt;name&gt;.json<br/>identity, object descriptors, PDO preset and cyclic layout,<br/>position semantics, startup defaults,<br/>native Home transaction, coordinate evidence and brake policy"]
    CFG["Machine config<br/>config/machines/&lt;machine&gt;.json, bench/ and simulation/<br/>robot.robot_description{path, sha256}, explicit absent_joints, robot.machine_planning_calibration{path, sha256}<br/>backend, cycle_ns, bus bring-up, control timing, cell I/O<br/>axes[]: robot_joint, slave_position, narrowing limits and policies"]
    CELL["Cell config<br/>config/cells/&lt;cell&gt;.json<br/>repository-relative machine_config and its configuration_sha256 pin,<br/>pair_id, pair_revision, remote_listen, package, activate"]
  end
  COMP["rtctl compile<br/>recomputes robot_description_sha256 and checks the pin, mappings and narrowed bounds,<br/>judges machine_planning_calibration.json against the description<br/>derives mechanics and startup values using the drive config<br/>checks units, wraps, limits vs 2π rule, evidence policy vs anchor period,<br/>PDO semantics vs preset, allowance vs half period"]
  OUT["compiled/&lt;configuration_sha256&gt;/<br/>argv.json (daemon command line), per-axis compiled profiles, robot.json<br/>resources/: the description whole plus machine_planning_calibration.json, served by the adapter<br/>machine_sha256: effective native semantics<br/>deployment_sha256: deployment including robot reference<br/>configuration_sha256: cell, profiles and pinned description binding"]
  DAEMON["rosie-rt-core started with argv<br/>(systemd unit or rtctl run)"]
  ADAPT["rt-control started with<br/>--configuration-sha256 --pair-id --pair-revision"]
  CLIENT["Client acquire must present<br/>the same pair id, revision and digest"]
  BRING["Bus bring-up:<br/>request master, configure slaves, PDO assignment,<br/>startup SDO writes then readback verified,<br/>DC sync, PREOP→SAFEOP→OP,<br/>mapping readback verified per axis (typed fault on mismatch),<br/>coordinate start-up verification"]

  PROF --> COMP
  URDF --> MODEL
  URDF --> MAN
  MODEL --> MAN
  MAN --> COMP
  MODEL --> COMP
  CFG --> COMP
  COMP --> OUT
  OUT --> DAEMON
  OUT --> ADAPT
  CELL -- "pair, revision, dense axes, listener" --> ADAPT
  OUT -. "config/cells validator recompiles and checks the pin" .-> CELL
  ADAPT --> CLIENT
  DAEMON --> BRING
```

Layer names are drive, robot, machine and cell config ([config/README.md](../config/README.md),
P8-26); the earlier `profiles/` and flat `cells/` layouts are gone. A cell config
pins a machine config's compiled digest and deployment fields; it does not enter
the compiler digest.

The compiler reads the description from the component when it carries
`robot_description/` (a runtime package stages every description, exactly the
files each manifest registers) and from the repository the component sits in
otherwise; the pin is recomputed over the files either way, so the two copies
cannot quietly differ. `verify-package.sh` recomputes every packaged
description's identity against the package manifest.

Machine configs cannot override gearing, sign, wire counts or startup objects. With A6
native gearing, the definition supplies the drive's startup reduction and the
software ratio stays 1. Home calibration remains separate, with its own epoch and
persisted anchors. Migrating a flat machine config preserves native payloads and machine
identity but changes configuration and deployment identity, requiring one Home;
the previous digest directory and its evidence stay untouched. See
[Robot definitions](MIGRATION.md#robot-definitions) for layer ownership, provenance
and the update-path table.

## 3. What flows, and how often

```mermaid
sequenceDiagram
  autonumber
  participant Drv as Drives
  participant Core as rosie-rt-core (1 ms cycle)
  participant Ad as rt-control
  participant Cl as Client (SDK / C++ / TS)

  loop every 1 ms
    Drv->>Core: TxPDO: error, statusword, position, torque, mode,<br/>touch probe, digital inputs (0x1B02) or<br/>following error + actual velocity (0x1B04, bench)
    Core->>Core: readiness, guards (following error, target lead, limits, brake),<br/>coordinate evidence, sample trajectory or jog lane
    Core->>Drv: RxPDO: controlword, target position, target velocity,<br/>target torque, mode, touch probe, max profile velocity
    Core->>Ad: status snapshot + telemetry record (shared memory ring)
  end

  Note over Core,Ad: mailbox SDO reads/writes (24..47 ms each) happen off-cycle:<br/>startup parameters, readback verification, Home transaction,<br/>rotation-frame count for coordinate evidence

  Cl->>Ad: acquire (pair, revision, digest, requested_lease_ms) → grant, lease_ms ≤ cell ceiling (default 500 ms)
  loop at most one third of the lease
    Cl->>Ad: renew
  end
  Cl->>Ad: enable(mask), arm, home(mask) / restore_anchor(mask)
  Ad->>Core: binary commands (IPC ring), native receipts back
  Cl->>Ad: prepare_trajectory / prepare_program (.rdt) → handle + identity
  Cl->>Ad: start_trajectory / start_program (identity echoed)
  Cl-->>Ad: begin_jog → update_jog frames (per-axis rad/s, source origin, expiry) → end_jog
  Ad->>Core: fenced setpoints / jog vector (latest value, expiry enforced)
  Cl->>Ad: stop (fence: cancel motion, ramp to hold, retire handles)
  Cl->>Ad: release
  Ad-->>Cl: GET /v1/status, /v1/events (sequence + incarnation), /v1/telemetry batches
```

Freshness rules: jog frames carry the input-capture time and an expiry; the daemon
scales frames above the per-axis cap and treats a stalled producer as expired within
the cell's input-age ceiling (default 250 ms, published by Describe). A program's first point must equal the held target within
one count. The lease expiring or Stop arriving inhibits outputs before any further
setpoint.

Each cycle publishes one fixed V2 capture after bus submission. The ring copies
the existing feedback and decision state: signed raw torque and mapping/freshness
validity, position/targets/drive velocity/following error, watchdog counters and
peaks, coordinates/epochs/Home/brake/DS402/slave state, extended diagnostics,
motion sample and authority generations, fault masks, signed wake lateness,
submission result and configured cell I/O words. No additional PDO, SDO or ioctl
read is introduced. `work_ns` includes record publication. The ring, commissioning
`cycle-capture.jsonl`, offline `rtctl telemetry dump-to-jsonl`, HTTP batches and
NATS binary batches preserve the same cycle timestamp and values. Optional field
validity must be checked before replay. V1 dumps are refused with stored/expected
digests and require the matching old converter. The measured V2 layout and bounds
are in [telemetry publication](APPLICATION_API.md#telemetry-publication).

## 4. Coordinates, Home and anchors

```mermaid
flowchart LR
  ENC["Drive encoder<br/>fast position 0x6064 (every cycle)<br/>rotation-frame count 0x2040:2B/2D (mailbox)"]
  VER["Start-up verification<br/>(single_turn_absolute mode):<br/>bracketed mailbox sample vs PDO history,<br/>allowance = physical speed × acquisition + tolerance<br/>must stay below half a turn"]
  HOME["Home (method 35)<br/>drive: mode 6, 0x607C, 0x6098=35,<br/>controlword 6→7→15→31,<br/>statusword bits 12 and 15 set, then mode 8"]
  ANCHOR["Adapter writes axis-N.anchor<br/>after qualified coordinate publication<br/>(digest, drive identity, home epoch, counts, offset, period, tolerance)"]
  REST["After restart: restore_anchor(mask)<br/>daemon checks identity, digest, period, tolerance<br/>→ Home valid without rehoming"]
  GUARD["Standing guards:<br/>drive fault flags, following error,<br/>encoder error 0x7305 invalidates the coordinate"]

  ENC --> VER --> HOME --> ANCHOR --> REST
  ENC --> GUARD
```

## 5. Where the consumers stand (2026-09-18)

| Consumer | Path to rt-core | Selection today |
|---|---|---|
| motion-server joint-trajectory/v1 | rt-core backend through the public C++ client (prepare/start program) | `rt_core` only; any other backend is refused `backend_retired` |
| motion-server v1 cartesiand | `--backend rt_core`: Cartesian intent → joint velocity → jog lane (P6-16 verified) | `rt_core` only; legacy selections refused |
| offline-programming | `rt_core` dense execution backend (Go SDK) and local simulation jog | `rt_core` only; `legacy_v4` refused |
| pendant (Steam Deck) | deploy profile `rt_core.pendant` (endpoint, PKI, principal, pair, digest); Cartesian resolution stays in motion-server/v1; jog frames per P6-07 (merged) | native selection; cell acceptance per tracker |
| system-state CLI | native telemetry, status, events | migrated |
| other readers | compatibility v4-shape status/telemetry from rt-natspublisher | v4-shape through the observation-only publisher |
| bench tooling | benchdrive, rtctl, direct on the public API | rt-core |

`robot/v4/realtime-core` and the legacy selections were retired by P8-11 (`01a2ee4b`,
verified). Installed v4 cells keep their published release tarballs
for rollback and await approved native configurations; see
[Repository defaults and deployed cutover](MIGRATION.md#repository-defaults-and-deployed-cutover).
`dev-stack.sh` accepts only `DEV_STACK_BACKEND=rt_core`.
