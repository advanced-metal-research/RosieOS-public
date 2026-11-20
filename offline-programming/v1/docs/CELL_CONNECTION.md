# How offline programming connects to a cell

Entry point for how the OLP finds, verifies, controls and lets go of a machine.
State at `36f4ec9d`. The detailed contracts are in
[RT_CORE_EXECUTION.md](RT_CORE_EXECUTION.md) (backend configuration, remote
cell, execution contract) and [MACHINE_FLOW.md](MACHINE_FLOW.md) (the operator
steps and the armed state); the public API is
[rt-core/docs/APPLICATION_API.md](../../../rt-core/docs/APPLICATION_API.md).

```
How offline programming (OLP) connects to a cell and how that connection is managed

WHO TALKS TO WHOM
  browser (OLP UI)  --HTTP/JSON, same origin-->  OLP Go server (serve, :8794)
  OLP Go server     --rt-core Go SDK (sdk/control)-->  rt-control on the cell
                        local cell:  Unix socket control.sock + jog.sock
                        remote cell: HTTPS mutual TLS 1.3 to https://<host>:8443
                                     (control, bulk, status, events; WSS /v1/jog for jog)
  rt-control        --private IPC-->  rosie-rt-core daemon  -->  EtherCAT drives
  The browser never holds a socket, credential, session token or grant. Only the Go
  server owns the rt-core session. One consumer controls a core at a time.

WHAT THE SERVER KNOWS BEFORE IT CONNECTS
  OFFLINE_PROGRAMMING_CELLS -> a server-side JSON catalogue (schema
  offline-programming.cell-catalogue.v1). One entry per cell: cell_id, label, role,
  the single robot model it binds, requested_mode (simulation | real), and execution:
    local:  {"local": true}  -> the dev stack's Unix-socket binding
    remote: address (HTTPS origin only), ca_file, certificate_file, key_file
            (the OLP principal's own PKI files, relative to the catalogue),
            binding {pair_id, revision, optional configuration_sha256},
            axis_mask, expected_backend (ethercat | simulation)
  The dev stack writes Local simulation into rt-core/build/dev-stack/olp-cells.json;
  DEV_STACK_OLP_CELLS appends remote cells. Startup fails closed on unknown models,
  duplicate cells, missing credentials or local/remote mixtures (cell_* reasons).

  For each remote cell the server also opens rt-core/config/cells/<cell>.json, takes
  its single rt_core node (machine_config, remote_listen, pair_id, pair_revision,
  configuration_sha256), compiles that machine config locally with the rt-core
  compiler (no hardware contact), verifies the robot definition and URDF pins, and
  requires endpoint, pair, revision and compiled digest to agree with the catalogue
  (cell_endpoint_mismatch, cell_pair_mismatch, cell_pair_revision_mismatch,
  cell_configuration_mismatch, cell_manifest_binding_missing). The repository pin is
  the approval to arm. (P8-28 now makes the cell serve its own robot definition,
  drive identities and URDF resources through Describe and /v1/resources; the OLP
  side still resolves the model from its catalogue and repository pin; consuming the
  served description is the follow-up card.)

IDENTITY: WHAT MUST MATCH
  Describe (GET /v1/describe, no grant) returns contract_version, capabilities_digest,
  backend, configuration_sha256, machine_sha256, deployment_sha256, axes, limits,
  max_grant_lease_ns, max_jog_input_age_ns, io.
  The server refuses the cell unless: contract_version == 2; capabilities_digest equals
  the generated client contract; configuration digest equals the compiled/catalogue pin;
  backend matches requested_mode (real -> ethercat, simulation -> simulation);
  the model matches. Refusals: rt_core description identity is incompatible,
  cell_configuration_mismatch, cell_mode_mismatch, cell_model_mismatch.
  Remote TLS: server cert verified against the cell CA; client cert carries URI SANs
  rosie-principal:<id> and rosie-pair:<pair>; wrong pair -> session_principal_mismatch;
  revoked/other TLS failure -> rt_core_transport_lost. CRL is checked every handshake.
  Tunnelled benches keep the listener on loopback: ssh -N -J jump -L 127.0.0.1:8443:...;
  the origin stays https://127.0.0.1:8443 and SSH never replaces mutual TLS.

THE OPERATOR FLOW (Machine panel, five steps, server-derived state)
  1 Choose   GET /targets lists cells with live backend/host/reason; grants nothing.
  2 Connect  POST /targets/select: Describe over mutual TLS, identity checks above.
             Selection is remembered by the server across reloads; performs no
             acquisition, arm, Home or playback. Switching cells: Stop then acknowledged
             Release of the old session first; refused while playing
             (cell_switch_while_playing); uncertain old Stop keeps the old selection.
  3 Home     POST /dense-execution/home: native rt-core Home, drives disarmed, no motion.
  4 Arm      POST /dense-execution/arm {"armed":true}: acquire -> enable -> arm.
             REAL/unknown posture needs a second confirm click. Arm clears a latched
             reset_clears fault (reset_fault, re-acquire) before Enable (P8-32).
  5 Drive    Jog panel (per-axis hold-to-jog, move-by-angle), Cartesian pad, Load/Play.
  STOP is always available and needs no session; DISARM = Stop then Release.

THE GRANT (the one thing that authorises motion)
  acquire(controller_id, binding{pair_id, revision, machine_sha256/configuration_sha256},
  requested_lease_ms) -> Grant{session, generation, lease_ms}. The lease is negotiated:
  SDK measures Describe round trip, asks for >= 3 round trips + cadence, capped by the
  cell's max_grant_lease_ns (default 500 ms); renews on a dedicated connection at most
  every lease/3; a lease that cannot cover the link is refused control_session_stale.
  Every mutating request carries session+generation (the fence). Stop increments the
  generation and retires all handles; Release empties the session. Native rt-core
  enforces the mirrored grant every cycle; expiry inhibits outputs regardless of the OLP.
  Browser heartbeat: the server expects UI heartbeats (5 s deadline); loss during motion
  fails the operation (ui_heartbeat_lost) and Stops; a plain page close does not.

REQUEST FENCING BETWEEN BROWSER AND SERVER
  Every mutating /dense-execution/* request carries X-RT-Target-Generation and
  X-RT-Target-Cell from the rendered status; mismatch -> 409 target_changed before any
  backend call. Each selection advances the generation. Stop, disarm, heartbeat and jog
  end stay available without matching identity.

WHAT FLOWS DURING OPERATION
  status   server polls GET /v1/status (250 ms) and derives the single "armed fact"
           from core.armed + lease held + freshness (STATUS_FRESHNESS_MS 2000);
           stale -> UNKNOWN, never a guessed state.
  events   GET /v1/events(/stream) with sequence + adapter incarnation; loss/incarnation
           change is surfaced, never hidden.
  program  fetch .rdt from the planner (/api/motion/dense/<digest>), verify digest and
           six-field identity, POST /v1/program (prepare_program) with session/generation
           headers, then start_program with the returned identity; completion only on
           observed native Done + terminal handle, never on an accepted Start.
  joint    begin_jog / update_jog (binary frame, jog.sock or calibrated WSS) / end_jog;
  jog      browser cadence 50 ms, input lifetime 100 ms, one axis held at a time;
           move-by-angle = prepare_trajectory (Hermite knots from the held target) +
           start_trajectory + observed completion.
  Cartesian intents go to a bounded resolver subprocess (robot-v4-cartesiand
           --resolve-only, local or over ssh on the cell) that returns joint velocities;
           the OLP then uses the same jog lane. rt-core never sees Cartesian intent.

DISCOVERY (Cells -> Search local network; README "Finding cells on the network")
  An OLP server advertises its catalogue as mDNS _rosie-olp._tcp on the interfaces in
  OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES and, with OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT,
  on an advertisement-only listener at its tailnet address (never the API); it lists
  other servers' cells at GET /discovered-cells, annotated from its own catalogue and manifests. Information
  only: Add fills the form from this server's commissioning and is still refused for an
  uncommissioned cell; Connect still runs the mutual-TLS Describe and pin checks above.

FAILURE POLICY
  Any refusal, renewal loss, changed grant, transport loss, native fault, lost readiness,
  stale observation, changed incarnation, event loss or lost heartbeat cancels motion
  work and issues Stop with a fresh bounded context; Release only after Stop is
  acknowledged; an unacknowledged Stop is reported as such (stop_uncertain) and inhibits
  new work. No automatic re-acquire, no motion replay. Native grant expiry is the last
  fence. Software Stop is not an e-stop; the cell's hardware stop chain remains required.

CELLS IN THE CATALOGUE
  local-simulation (dev stack, Unix socket) and each commissioned remote cell: a
  robot (rosie_1400_v3, rosie_1420_v1) or a bench model, reached over mutual TLS
  through a tunnel or directly. A cell without a native configuration is listed
  unavailable (cell_configuration_missing) until it has one.
```
