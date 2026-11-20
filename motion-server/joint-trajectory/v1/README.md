# motion-server/joint-trajectory/v1 — dense trajectory daemon

## Repository launch selection

The binary, repository config, `start-joint-trajectory.sh` and
`joint-trajectory.service` select `rt_core`. Retired flag, environment and JSON
backend selections fail with `backend_retired` before opening resources.
A supplied JSON file must still contain an explicit backend and complete
approved binding. The helper reads `JOINT_TRAJECTORY_DAEMON_CONFIG`; the unit
reads `/etc/robot-v4/joint-trajectory.env` and expects the helper under
`/opt/rosie/motion-server/joint-trajectory/v1`. Install the helper with the unit.
Neither changes cell manifests, installed services or release selection.
Deployed cells continue using their manifest-selected release until cutover.

`joint_trajectory_daemon` ingests immutable dense `.rdt` programs over TCP and
accepts the existing NATS leader, play and stop commands. The current backend is
`rt_core`: it prepares the complete program through rt-core's public C++ client
and explicitly starts the returned identity. rt-core owns interpolation and
execution. No UDP setpoints, velocity follower or torch publisher runs on this path.

The daemon includes the header-only client from
`../../../rt-core/clients/cpp/include`; it does not copy the client or call native IPC.
The reference contract is rt-core's APPLICATION_API integration profile and
`clients/cpp/examples/motion_server_sequence.cpp`.

## Configuration

Pass `--config config/joint_trajectory_daemon.config.json`, or set
`JOINT_TRAJECTORY_DAEMON_CONFIG`. CLI address flags override the file. Set
`backend` to `rt_core` and supply an `rt_core` object with:

- `socket`: the public Unix control socket.
- `controller`, `pair_id`, `pair_revision`: the controller identity and approved pair binding.
- `machine_sha256`, `deployment_sha256`, `configuration_sha256`: approved artifact digests.
- `expected_backend`: the backend advertised by the approved deployment.
- `home_policy`: `home` or `restore_anchor`. Anchor refusal requires operator correction; it never silently falls back to Home.
- `status_subject`: optional NATS execution status subject, defaulting to the command subject plus `.status`.
- `control_timeout_ms`: unverified receipt/observation ceiling, default 10000 ms.

The backend key is required; an older config without it fails at startup with a
clear message. Missing or mismatched identity refuses acquisition. The supported deployment
maps the dense axes to Describe indices J1 through J9 in radians. Machine and
deployment digests pin the approved axis limits and mapping. Every required
capability must be implemented. Native renewal follows the profile's unverified
100 ms cadence for its 500 ms lease.

TCP `upload` persists the original bytes. `preload` revalidates the six existing
identity fields, uploads those bytes unchanged, and verifies the returned producer
identity and exact source digest. The adapter owns stationary-seam normalisation;
the daemon retains its returned normalised digest and checks the entire returned
identity unchanged before Start. It stores the
returned metadata beside the `.rdt` record as `.rdt.rt-core.json`. That sidecar
never authorizes replay after restart or lease loss. `play` checks prepared
identity, Home or qualified anchor evidence, Enable/Arm and selected-axis
readiness before Start. Completion requires matching native sequence and
execution generation in events and terminal status.
Preload precondition refusals leave acquisition, commissioning, playback and
their leases unchanged; an unauthenticated ingest request cannot cancel them.
Segment progress uses the native sample cursor and original segment clocks;
native host observation timestamps are never displayed as elapsed plan time.

NATS schemas, fencing and command replies retain their existing fields. Status
adds `backend`, `program_identity`, `execution_state`, `execution_generation`,
`native_sequence`, `detail`, `native_result`, `consumer_action`, `commissioning_phase`
and `native_state` (`inactive`, `acquiring`, `owned`, `reconciling`,
`cleanup_uncertain`, `grant_active`). Typed rt-core
refusals, including axis/sample context, remain unchanged in `abort_reason` and
command/preload replies. Unknown diagnostics abort, Stop and release; uncertain
cleanup blocks new authority until a fresh acquire successfully reconciles Status.
Stop observes two distinct timestamped inhibited samples with unchanged position
counts and zero velocity when reported; a receipt alone is insufficient.
Shutdown uses one unverified 1000 ms I/O budget from entry under the executor
mutex, including the SDK's bounded 250 ms renewal join. Stop, Release and
reconciliation share the same absolute deadline to bound executor and leader
transition lock occupancy on an unresponsive adapter. Reconciliation also obeys
`control_timeout_ms`. Exhausting that budget releases the locks with
`native_state=cleanup_uncertain`; it does not establish physical standstill.
Fresh acquisition re-checks Status within the lesser of 1000 ms and
`control_timeout_ms`. A changed adapter incarnation can reconcile using two
inhibited samples from its current incarnation and the approved configuration.
An active grant, including one held by another controller, returns
`native_grant_active`, publishes `native_state=grant_active`, and leaves that
controller untouched. The next acquire re-checks it. Daemon-side conditions use `native_*` labels, including
`native_acquisition_required` and `native_home_unavailable`. Failed admission is
never automatically uploaded or started again.

`pause` returns `pause_unsupported` and leaves native execution running; use
`stop` to cancel. Stop, leader replacement/release/expiry and native lease loss
converge to Stop then release. Fresh motion requires acquisition and preload
again. A torch-bearing program is refused during preload with
`native_torch_unsupported` before any native upload or resident handle allocation.
Torch output remains the external owner's responsibility; `torch_on` is local
intent and is always false on the native path.

Leader transitions mark authority inactive while I/O runs outside the leader
state mutex, then commit the new epoch after revocation and acquisition succeed.
Read-only observations use an independent client outside the executor mutex.
Stop cancels them before locking; each observation rechecks cancellation and
captured epoch/session before Start. Start and Stop remain serialized.
Separate owned workers handle play and authority transitions; renew and Stop
remain on the NATS subscriber. Status publishes a separate snapshot during I/O.
`commissioning_phase` publishes the local phase (`preparation`, `home` or
`restore_anchor`, `enable_arm`, `readiness`, `start`) while Play is pending and
clears when it completes or is stopped. These phases describe work in progress;
native readiness and Start still require their corresponding observations.

Active execution and commissioning poll at an unverified 5 ms cadence. Idle
sessions poll at an unverified 100 ms cadence to limit socket traffic while
checking lease health. Acquisition seeds the event cursor from LatestSequence;
events_dropped reconciles Status and resumes at the loss sequence, while
publisher_overflow is delivered as an ordinary event.

## Build and local stack

From the repository root:

```sh
make -C rt-core control sim
make -C motion-server/joint-trajectory/v1 all
./dev-stack.sh start
./dev-stack.sh status
./dev-stack.sh stop
```

The dense Makefile defines the binary directory and exposes it through
`print-bin-dir`; dev-stack queries that target. Its default is
`rt-core/build/dense`. `DEV_STACK_DENSE_BIN_DIR` can select an explicitly built
alternative. The stack writes the native daemon configuration and verifies its
TCP status before reporting it healthy.

The velocity follower, legacy simulator keeper and their bench harness have
retired. Installed releases are recovered from published tarballs, as described
in [rt-core migration](../../../rt-core/docs/MIGRATION.md#repository-defaults-and-deployed-cutover).

## Status stream

TCP `status` and the configured NATS status subject publish the native executor
state, immutable program identity, execution generation and typed refusals.
The optional stream is telemetry; commands still require authority and receipts.
Native status and behavior are described under Configuration above.

### Format contract

- Contract: [docs/DENSE-JOINT-TRAJECTORY-V1.md](docs/DENSE-JOINT-TRAJECTORY-V1.md) (`robot.v4.dense-joint-trajectory.v1`)
- C++ decoder/digest: `src/dense_joint_trajectory_format.hpp`
- Python encoder: `weld_planner/v1/python/weldplan/dense_joint_trajectory.py`
- Go twin: `offline-programming/v1/internal/densejointtraj/`
- Reference fixture (cross-language arbiter): `testdata/dense_joint_trajectory_reference.rdt`,
  regenerated deterministically by `tools/make_reference_fixture.py`
- [Python encoder test](../../../weld_planner/v1/tests/programmer/test_dense_joint_trajectory.py)

## Tests

Build the sibling rt-core once with `make control sim`. In WSL set Go on PATH
and `TMPDIR=/dev/shm/rt-core/motionserver`, creating that directory before tests.
Run `make test` here for all C++ regressions and the composed Go tests; run
`make test-go` for composed tests and `make test-sanitize` for the native executor
under address/undefined-behavior sanitizers. Binary outputs default to `rt-core/build/dense`; `TEST_BUILD_DIR` selects test scratch.

The composed tests launch simulation rt-core, rt-control, the real daemon and a
NATS protocol broker. They exercise immutable `valid.rdt` execution and measured
endpoint completion, cancellation, and refusal propagation. The original
reference is refused by the local torch scan. Its torch-free derivative exercises
native program refusal using the corpus's declared A6 bench limits repeated across
J1..J9 with simulation commissioning; it must report
`native_limit_exceeded: segment=0 sample=0 axis=J3`. The profile example's wider
simulation position bounds instead first refuse J4, so those are distinct
fixtures. Tests observe native events and NATS status, and assert zero legacy
UDP output and zero torch ON publications.
