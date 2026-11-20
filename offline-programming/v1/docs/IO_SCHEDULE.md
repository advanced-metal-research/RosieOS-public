# Execution I/O schedule: program `io` rows to switched outputs

Status 2026-10-12. The planner half, the OLP's packing and the OLP's output
execution step are implemented and tested (simulation only: no output has
been switched by a program on a cell yet). A production plan that switches
outputs loads bound to its plan record's schedule digest, Play arms the
cell's outputs before Start, and the end of the program disarms them; both
apps show each output's intent, command and readback. No rt-core or protocol
change is needed:
rt-core has executed dense `process_markers`, `io_arm`/`io_disarm` and
per-output expiry since 9f2d3fd4 (`rt-core/docs/CELL_IO.md`,
`APPLICATION_API.md`). The capabilities digest does not change, so no UI or
Deck build has to match a new controller.

## Operator-selected taught-weld output (October 18, 2026)

Cell I/O settings add `weld_output` to the packed `io_outputs.json`. It is an
explicit output name, not a new hardware classification or qualification. Empty
means unassigned; absent preserves the older sole-native-torch behavior. Taught
Weld Start/Stop resolves the selected process or qualified torch channel. Generic
I/O rows cannot also command this reserved weld channel. Existing immutable plans
keep their digest-bound schedule; save setup and replan to use a new assignment.
Trace plans a private `execution_mode: dry_run` copy before preparation, retaining
geometry and timing with no output or sensor-wait markers. It does not overwrite
the production plan. See the [pendant I/O and weld implementation record](../../../steamdeck/real/v5/docs/IO_WELD_IMPLEMENTATION.md).

## The chain

1. **Program.** A program.v2 `io` node is `{channel, state}`: switch one of
   the connected cell's configured outputs on or off, with the arm at rest
   between the motion nodes around it. `channel` is the rt-core output `name`
   (`gas`, `torch`, ...). The authoring default `revpi:1` is a placeholder
   that no cell has; it is refused naming the cell's real outputs.
2. **OLP packing** (`weld_plan.go`, `weld_plan_start.go cellIOOutputs`). For a
   program with an enabled `io` row the OLP reads the connected cell's
   Describe `io` block (carried in the native status) and packs it as
   `io_outputs.json`, schema `rosie.cell-io-outputs.v1`:
   `{torch_qualified, outputs: [{name, class, expiry_ns}]}`, bound by the
   `.weldplan` manifest. Offline: refused,
   `taught_program_unsupported_node`, "Gas on (io) switches an output: plan
   with the cell connected, so the plan knows the cell's outputs and their
   expiry". A connected cell with no `io` block: refused,
   `io_not_configured`.
3. **Planner** (`weld_planner/v1/python/weldplan/io_schedule.py`).
   `io_problems` refuses, by row name, a channel the cell does not have, a
   torch-class output on a cell with `torch_qualified: false`, or a missing
   state, at pack time and again in M6. An `io` row puts the program in
   authored order and builds no planner collision model, so taught programs
   with outputs plan on the CPU planner. The dense export gives each row its
   own dwell segment of `IO_EVENT_S` (0.05 s, five 10 ms samples, unverified
   on hardware) and the marker sits on its first sample. `schedule` then:
   - re-sends every ON after half the output's expiry on the plan's clock
     (`REFRESH_FRACTION` 0.5; never closer than two samples, so an expiry
     under 40 ms is refused at 100 Hz);
   - writes an explicit OFF on the last sample for every output still ON;
   - keeps one marker per output per instant (segment k's first sample is
     segment k-1's last; rt-core faults on two markers for one output in
     one exchange).

   The header carries `process_markers` and `process_markers_sha256`
   (canonical JSON). The markers are outside the v1 trajectory digest, which
   covers samples only and is recomputed by rt-core, so the schedule has its
   own digest; the motion server's dense summary reports it with
   `process_marker_count`. A dry run (authored `execution_mode`) emits no
   markers, as it emits no torch bits.
4. **OLP Load** (`internal/denseexec/rt_core_outputs.go processOutputsGate`,
   run by Load and by Run program's robot-free preflight, before any
   authority is taken). The browser and the Deck keep the dense summary's
   `process_markers_sha256` in the plan record (`denseTrajectoryRecordOf`)
   and send it with Load and Run program (`denseLoadIdentity`). A production
   Load of a plan with header markers is refused (409) when:
   - the header markers do not hash to the header's own
     `process_markers_sha256` (`process_markers_mismatch`);
   - the client sent no digest (`process_markers_unbound`: an old client,
     which cannot show what the plan switches) or another one
     (`process_markers_mismatch`);
   - the cell describes no outputs (`io_not_configured`), a marker names an
     output the cell does not have (`io_marker_invalid`) or a torch-class
     output on a cell whose torch outputs are not qualified
     (`io_torch_unqualified`), each naming the outputs.
   Otherwise the exact planner bytes are prepared and the adapter's identity
   is checked as the envelope it binds: `SourceDigest` = sha256 of the
   uploaded blob, `RequiresProcessIO` true, the trajectory digest unchanged;
   the session records `process_outputs` (sorted output names). The digest is
   the planner's canonical JSON (`densejointtraj.ProcessMarkersDigest`, byte
   for byte Python's `json.dumps(sort_keys, compact, ensure_ascii)`). A
   dry-run Load strips the markers (`densejointtraj.WithoutProcessOutputs`)
   and runs the motion; it needs no digest and is refused only when the
   record names a different schedule.
5. **OLP Play** (`rt_core.go Play`). After the armed and ready check and
   before `start_program`, `io_arm` on the held grant (it arms permission
   only). rt-core's `io_arm` refusals the operator can fix
   (`io_fast_input_unsatisfied` naming the open fast input,
   `io_readback_disagreement` naming the output reading ON,
   `io_exchange_lost`, `io_not_configured`, `io_torch_unqualified`,
   `inhibited`) are answered in words and leave the session loaded and the
   robot armed: nothing started, nothing was stopped, Play again works once
   fixed. Any other failure takes Play's usual path (Stop and release). A
   Start refused `io_not_armed` (the cell disarmed between `io_arm` and
   Start) says so.
6. **Session end** (`rt_core_outputs.go settleOutputs`). Once a session that
   armed outputs is no longer playing (done, or loaded again for a loop's
   next cycle), `io_disarm` on the grant `io_arm` used. It runs under the
   operation lock, so it cannot disarm what a concurrent Play has just
   armed. A Stop, a fault, a drive or bus loss and any grant change need
   nothing: rt-core writes every output OFF and clears permission in that
   cycle, and Stop forgets the armed grant (no `io_disarm` on a released
   grant). The OLP has no program Halt or Pause (Pause answers
   `capability_unimplemented`); a stalled client stops the program through
   the heartbeat deadman's Stop.
7. **rt-core** executes each marker at its source sample, bounded to one
   native cycle; a late marker faults without replay.
8. **Status** (`ui/src/execution/cellOutputs.ts`, the browser's outputs block
   under Run and the Deck's Run page through olp-core
   `cellOutputsFromStatus`). Per described output: commanded ON/off, the
   program's intent when it differs, the wired readback (`readback unknown`
   when invalid, `no wired readback` for a `none_unverified` output, never
   OFF), the time left on an ON before its expiry, and the headline: armed
   or disarmed with rt-core's I/O reason, and what the loaded program
   switches (or that a dry run keeps them off). Hidden for a cell with no
   `io` block; the Deck hides it when the server goes silent.

## Versioning: old consumers refuse

| Consumer | A plan with `process_markers` |
|---|---|
| Planner before this change | Never produced: `io` rows were refused (or, weld-only, dropped). |
| OLP before this change | Load uploads it; native preparation reports `RequiresProcessIO` and the OLP refuses `program_identity_or_process_mismatch` (after taking authority). |
| OLP 2026-10-11 to 2026-10-12 | Refused up front (`process_outputs_not_enabled`) unless dry run. |
| OLP now | Loads bound to the record's digest; a client that sends none (browser or Deck before this change) is refused `process_markers_unbound` and can still dry-run. |
| Browser and Deck now | Send `process_markers_sha256` only for an output plan; an older OLP ignores it (unknown JSON fields are ignored) and keeps refusing production. |
| rt-core adapter (every installed release) | Admits markers naming configured outputs; refuses `io_not_configured` / `io_torch_unqualified` otherwise. |
| Browser and Deck decoders | Preview and Go to plan start read samples only; they execute nothing. |

The format stays `robot.v4.dense-joint-trajectory.v1`: `process_markers` is
the v1 header extension rt-core already defines. A schema bump would make
every installed rt-core refuse every plan.

## Safe behaviour

- **Stop**: rt-core writes every output OFF in its fence cycle; a new ON needs
  a fresh grant and an explicit `io_arm` with OFF readback observed after the
  fence.
- **Fault, permit loss, shutdown, grant change**: `cell_io.output` is gated on
  permit, no safety fault, no shutdown and the current grant generation
  (`engine/cycle_machine.hpp`), so outputs go OFF in that cycle. Readback
  disagreement for three exchanges latches fault bit 13 (`cell_io`).
- **Halt, Pause, a stalled or lost client** (rt-core's view; the OLP has no
  program Halt or Pause and stops on a lost heartbeat): the trajectory stops advancing,
  no refresh marker is delivered, and every ON expires within its
  `expiry_ns` (at most 1 s by rt-core's bound; 200 ms in the simulation
  fixture). Resuming a paused program re-sends ON at its next refresh. Open:
  an rt-core change to write outputs OFF in the cycle that applies Halt/Pause
  would remove the expiry wait (full tier).
- **End of program**: an explicit OFF on the last sample, then expiry.
- **Torch**: torch-class outputs stay refused until the first terminal is
  qualified (`CELL_IO.md` checklist); the planner refuses them by name first.

## Remaining work

The OLP output execution step (OLP tier, no rt-core change) is done
(2026-10-12, branch `olp-io-execution`): items 1-6 of the 2026-10-11 list
(record digest, production Load, Play `io_arm`, session-end `io_disarm`,
status in both apps, composed simulation test) are steps 4-6 and 8 above
and the tests below. Not yet done:

- Run it on a cell: no cell has a configured output (below), so nothing has
  been switched by a program on hardware. The first run is the CELL_IO.md
  first-terminal checklist with the robot attended.
- Add the composed test to the release gate's `olp_composed` suite once it
  is steady on the build host. On a development Mac at load 12-26 (a cell
  release running), 10 runs of the two tests: 3 passed; 6 failed in the
  simulated HM35 setup on a Rosie 1430 simulation (`cycle_deadline` lateness,
  a grant refusal or a lost transport) before any output step; 1 passed
  every output assertion and then lost its 500 ms grant before `done` was
  observed. No output assertion failed.
- An rt-core change to write outputs OFF in the cycle that applies Halt or
  Pause (see Safe behaviour) would remove the expiry wait (full tier).

Cell work (owner decisions, not code):

- A cell with no output hardware configured has no `io` block in its machine
  configuration. rt-core carries a profile for the PLANET IECS-1116-DO (16
  outputs, no mailbox, no inputs;
  `rt-core/config/drives/planet_iecs_1116_do.json`, identity from the ESI).
  Using it means plugging it in after the last drive, an `io` block in the
  cell's `config/machines/<machine>.json` with each output's channel and
  `"readback": "none_unverified"` (no input module), a full-tier release (the
  schema 2 backend is native code), and the first-terminal checks in
  `CELL_IO.md`. Until that configuration is deployed the module must stay off
  the bus: `unknown_slaves: refuse`.
- RevPi digital I/O: rt-core drives a RevPi DIO through the piControl process
  image (`rt-core/config/drives/revpi_dio.json`; machine `io` names
  `picontrol_device` and the PiCtory offsets). A servo host with the expansion
  fitted needs its offsets (`piTest -v O_1`, `I_1`) in the repo. The `revpi:1`
  placeholder is still refused: a program names the output (`gas`, `weld`),
  never a pin.

## Tests

- `weld_planner/v1/tests/shared/test_io_schedule.py`: refusals by name;
  refresh gaps of 0.10 s against a 0.20 s expiry in absolute time; explicit
  final OFF; one marker per output per instant at a shared endpoint; short
  expiry refused; canonical digest.
- `weld_planner/v1/tests/programmer/test_dense_joint_trajectory.py
  test_output_rows_become_header_markers_bound_by_their_own_digest`: row
  dwell segments, header markers and digest, trajectory digest unchanged by
  the schedule, dry run without markers, missing outputs refused.
- `weld_planner/v1/tests/shared/test_plan_request.py`: `io_outputs.json`
  bound and read; unknown channel refused at pack.
- `weld_planner/v1/tools/measured_start_replay.py --io-outputs`: on the
  production motion server with a Rosie 1430 cell's compiled context, P1, gas ON, P2,
  gas OFF, HOME from a measured pose planned on the CPU planner in 2.2 s with
  121 markers, max ON gap 0.10 s, digest matching.
- OLP: `TestWeldPlanRefusesIOThatNoCellOutputCanCarry`,
  `TestPackWeldPlanCarriesTheCellsOutputs`.
- `densejointtraj TestProcessMarkersDigestIsThePlannersCanonicalDigest`:
  digests computed by Python's json.dumps, escapes included.
- `denseexec rt_core_outputs_test.go`: the bound Load (no digest, wrong
  digest, refused before authority; the exact bytes prepared; envelope
  identity at Start; calls `acquire, prepare_program, enable, arm, io_arm,
  start_program, io_disarm`), every robot-free refusal by name, Run
  program's preflight, an `io_arm` refusal that keeps the program loaded and
  Play again after it clears, Stop without `io_disarm`, the dry run arming
  nothing, a record naming a schedule for a motion-only plan.
- `denseexec rt_core_outputs_composed_test.go` (`RTCORE_COMPOSED=1`, after
  `make control sim` in rt-core): a Rosie 1430 simulation with the cell-I/O
  fixture's io block, homed through the OLP; P1, gas ON, P2, gas OFF with two
  refreshes: gas ON within one 1 ms cycle of its sample, held 300 ms (+-21 ms)
  against a 200 ms expiry (so the refreshes reached the terminal), OFF on its
  marker, the envelope identity executed, outputs disarmed after done; with
  the fast input open, Play refused naming `consent`, nothing started, the
  program still loaded.
- UI `execution/cellOutputs.test.ts` and `densePanel.test.ts` (the outputs
  block follows the polled status); Deck `olp-core/outputs.test.ts` and
  `tests/teach_check.cpp` (the Run page shows nothing for a cell without
  outputs, then gas
  ON from the status poll).
