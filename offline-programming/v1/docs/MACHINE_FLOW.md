# The machine path: Choose, Connect, Home, Arm, Drive

One way to turn a machine on, arm it and drive it, in the order the server
enforces. The browser derives every step's state from what the server
reports and never from what it hopes; the pure derivation is
`ui/src/execution/machineFlow.ts`, its tests `machineFlow.test.ts`. The connection
and authority model behind these steps is summarised in [CELL_CONNECTION.md](CELL_CONNECTION.md).

## What the operator sees

Under **Robot** in the setup sidebar, below the robot model, the **Machine**
panel is a vertical rail of five numbered steps. A step is one of:

| Mark | State | Meaning |
| --- | --- | --- |
| green check | done | The server's own status confirms it. |
| orange number | next | The one action to take now; its button is the primary one. |
| amber `!` | blocked | The server refused or cannot proceed; the typed reason is shown in words with the code in brackets. |
| grey number | not yet | A prior step must complete first; the summary says which. |

Across the top a posture banner says what the connected machine is:
**REAL · PHYSICAL OUTPUT** (red), **SIMULATION** (green) or **OUTPUT
UNKNOWN** (red; treated as real). At the foot **STOP** and **DISARM** stay in
view while a machine is connected and are mirrored in the header, next to
the one machine pill, so they are reachable from every sidebar section.

The steps and the requests behind them:

1. **Choose a machine.** `GET /api/offline-programming/v1/targets` fills the
   picker; choosing narrows the robot model and grants nothing. A cell the
   catalogue lists as unavailable blocks this step with its reason
   (`cell_configuration_mismatch`, `session_principal_mismatch`, ...);
   Connect stays offered because the server is the authority, except for
   `cell_configuration_missing`.
2. **Connect.** `POST .../targets/select` describes the cell over mutual TLS
   and checks identity. Done reads `Identity verified over mutual TLS.
   <backend>` and, for EtherCAT, `This is a real machine.` A refusal
   (`cell_mode_mismatch`, `cell_model_mismatch`,
   `rt_core description identity is incompatible`, ...) blocks the step.
3. **Home.** Native rt-core Home (`POST .../dense-execution/home`) runs while
   the drives are disarmed and moves nothing. The step is done when every
   described axis reports `home_valid`; current when some do not; blocked
   when the drives are armed (DISARM, then Home) or the description offers
   no native Home. On the legacy joint daemon this step reads as done and
   saved-pose GO HOME, a motion, is the step's button after Arm.
4. **Arm.** `POST .../dense-execution/arm {"armed": true}` acquires control,
   enables and arms. The step opens with the armed state in large words
   (`ARMED`, `DISARMED`, `ARMING…`, `STOPPING…`, `UNKNOWN: no fresh status
   from the machine`; see *The armed state* below), then the explanation. A
   described simulation arms on one click. A REAL or unknown posture takes
   two: `Arm <cell>` then `Confirm: energise motors on <cell>`, with Cancel
   beside it. Arm waits behind a known Home requirement; before the first
   status the server decides, as it always has.
5. **Drive.** Current once armed and every axis reports ready. Its controls:
   the **Jog panel** button, the run-program row (`Load to robot`, Play,
   Pause on the legacy backend, Stop, readout) and, for the legacy daemon,
   the per-motor table. Machine feedback (rejections, reset availability,
   per-axis Home and limit bands) renders here.

**STOP** (`POST .../dense-execution/stop`) is enabled whenever a machine is
connected and needs no session. **DISARM** is Stop then Release on rt-core.

## The Jog panel

Hold-to-jog and move-by-angle per joint, and the Cartesian pad, live in one
panel over the viewport so the operator jogs while watching the robot. JOG
in the spreadsheet toolbar, the Drive step's button and the panel's own
close button show and hide it; the choice is remembered per browser under
the existing key. Two sections each say what they drive:

- **Joints · machine.** Speed (5, 10, 25, 50, 75, 100 % of each joint's
  described maximum, the note beside it saying what that is in °/s; the
  server keeps its measured 0.75 drive reserve under every fraction), one
  row per described axis with position, hold buttons and a signed-degrees
  GO. The heading names the connected cell and its posture; disabled rows
  say why (no machine, no description, a refusal, no grant, disarmed, not
  ready, busy). Requests carry the target generation and cell headers.
- **Cartesian · SELECTED MACHINE.** While connected, the twelve BASE/TOOL
  hold buttons drive the selected machine through the same controller as
  Joints. The heading names its cell and physical posture and shows the Arm
  step's instruction. Speed fractions run 5% to 100%; in Hold the fraction
  scales the Speed and Rate boxes (100 mm/s at 25% is 25 mm/s) and is also the
  per-joint ceiling; the server checks live pose and limits on each intent. Refusals appear beside the pad in words.
  `Arm preview` is hidden; machine arming remains step 4.
- **Cartesian · BROWSER PREVIEW.** With no machine connected, the pad retains
  its isolated simulator requests and `Arm preview` / `Disarm preview` toggle.
  Production program mode disables that toggle as before. A selected machine
  that loses readiness remains the destination and disables the pad; it never
  silently becomes preview. The Live viewport follows machine status.

Every hold ends once on pointer up, leave, cancel, window blur, page hide,
disarm, session loss or target change; those contracts are unchanged and
covered by `test/main-jog.test.ts`, `execution/jointJog.test.ts` and
`execution/cartesianJog.test.ts`. Cartesian requests pin the same target headers,
session and hold revision; resolver refusals retire the shared joint jog lane.

A failed jog request renders in one of two voices under the joints. A typed
server refusal keeps its code and label (`jog_limited: Jog limited; ...`).
Anything thrown in the browser itself (a DOM `TypeError`, a rejected
`fetch`) renders as `client error: <message>` and is never dressed as
`rt_core_transport_lost`, which is the server's word for losing the machine
and turns the arm state UNKNOWN. The LA bench showed
`rt_core_transport_lost: Illegal invocation` under the joints while the
machine was armed and answering: the hold's default timers invoked
`window.setInterval` as a method of a plain object, which Chrome refuses
with that `TypeError`; `browserJogTimers()` in `jointJog.ts` calls it as a
free function, and `jointJogFailureText` does the mapping
(`jointJog.test.ts`, "failure text ...", "default timers ...").

## Viewport follows the connected machine

While a machine is connected (step 2 done), the 3D model shows that
machine's joint pose from the same `/dense-execution/status` poll the panel
already runs, at the poll rate (250 ms; 25 ms while a program runs on the
machine). The derivation is pure, `execution/livePose.ts`
(`mapLivePose`, `deriveLivePose`; tests in `livePose.test.ts`); the panel
forwards each mapped pose through `DensePanelDeps.onRobotPose` and
exposes `livePoseView()` for the caption. This is what the model is there
for: on the LA bench the joints turned on camera and in the Jog panel's
radians while the model sat at the program preview pose, because the
rt-core backend published positions and nothing forwarded them.

Mapping is by the model the machine reports, `status.target.model_id`,
never by assuming nine wire axes: rosie_1420_v1 describes six axes and its
URDF articulates J1..J6 (J7..J9 stay at home), rosie_1400_v3 describes
nine. Each described axis lands on the model joint with its id (J1..J9); an
id the model does not know falls back to the wire index. Only axes described
in radians are taken as joint angles. An axis without a valid logical
position keeps the value the viewer last showed for it. On the legacy joint
daemon the mapped, homed radians are used as before.

The viewport caption says where the pose comes from:

| State | Caption |
| --- | --- |
| connected, status within `STATUS_FRESHNESS_MS` (2,000 ms) | `LIVE · <cell label> · <model> · status <age> ms` |
| connected, status past the bound or carrying no servo observation | `LIVE (STALE) · <cell label> · <model> · status <age> ms`; the pose stops updating |
| connected, fresh status but no axis in radians with a valid position | `LIVE · <cell label> · <model> · no axis positions · status <age> ms` |
| no machine connected | today's preview caption, `Canonical <model> · home/last-known offline pose` |

The toolbar's **Preview** / **Live** toggle (ids `#pose-source-sim` and
`#pose-source-real`, kept) chooses the pose source explicitly. Connecting a
machine selects Live; disconnecting falls back to Preview. Starting a
browser preview (classic playback or a rails replay) selects Preview so the
program can be previewed in the browser while a machine sits connected, and
the machine takes the viewport back within one status poll of that preview
stopping; an operator's own Preview click is kept until they click Live. A
play or go-home on the machine selects Live and locks both buttons while it
moves, so the real robot wins and the viewer never shows one robot while the
other one moves. A pose for a model other than the loaded one is refused by
the renderer's identity check and reported in the caption as
`Pose rejected: model-mismatch`.

## The armed state

One fact, `MachineFlowView.arm` (`ArmFact` in `machineFlow.ts`), derived
once per render and read by every surface: the header pill, the footer
`MACHINE:` line, the Arm step's headline and summary, the Drive step's
gate, the Jog panel's joints caption and the run row. It has exactly five
states. `UNKNOWN` is reserved for the one case in which the server holds no
fresh observation of the machine; a refusal, a `busy` result or a request
that never left the workstation is rendered beside a known state, never in
place of it. This is the fix for the LA bench split in which the header
read `ARMED` while the Arm step read "arm state is unknown", the Drive step
"Arm first." and the run row "busy".

The observation is the drives' own word: `rt_core.status.core.armed` on
rt-core (carried by the client as `rtCore.coreArmed`, independent of any
reason), the daemon's `armed` on the legacy backend. It is fresh when the
server answered the status poll within `STATUS_FRESHNESS_MS` (2000 ms:
eight 250 ms poll periods, inside the server's 5 s heartbeat deadline;
chosen, not measured) and the reason is not one that says the server cannot
see the machine (`rt_core_transport_lost`, `rt_core_closed`,
`rt_core status unavailable`). The server's own `armed` field stays the
compound it always was (`core.armed && grant held && no reason`) and still
gates program playback; the fact reads it only through `session.lease.held`
to tell an ARMED machine this server controls from one another controller
armed.

| Server fields and panel facts | State | Arm headline | Pill / footer / run-row token | Drive step | Jog caption |
| --- | --- | --- | --- | --- | --- |
| fresh status, `core.armed = 0`, nothing in flight | `disarmed` | `DISARMED` | `DISARMED` | not yet: `Arm first.` | `Arm it first (step 4).` |
| this panel's Arm in flight and `core.armed = 0`; or status reason `busy` / `rt_core_request_slow` with `core.armed = 0` and the last request from here was Arm (or none) | `arming` | `ARMING…` | `ARMING` | not yet: `Arming; Drive opens at ARMED.` | `Arming; wait for ARMED before jogging.` |
| fresh status, `core.armed = 1`, `session.lease.held` | `armed` | `ARMED` | `ARMED` (pill filled red; `PLAYING` / `PAUSED` while a program runs) | current (or blocked by readiness) | empty |
| fresh status, `core.armed = 1`, grant not held | `armed` | `ARMED` | `ARMED` | not yet: `Armed by another controller; Arm here to take control.` | `Armed by another controller; this workstation holds no grant.` |
| this panel's Stop or Disarm in flight and `core.armed = 1`; or reason `busy` with `core.armed = 1` | `disarming` | `STOPPING…` / `DISARMING…` | `STOPPING` / `DISARMING` | not yet: `Stopping; Arm again afterwards.` | `Stopping; wait for DISARMED.` |
| no status yet; status older than the bound; status poll failing; reason `rt_core_transport_lost` / `rt_core_closed` / `rt_core status unavailable` | `unknown` | `UNKNOWN: no fresh status from the machine` | `STATUS STALE` (pill amber, dashed; claims neither side; STOP enabled) | not yet: `Arm state UNKNOWN: no fresh status from the machine.` | `Arm state UNKNOWN: no fresh status from the machine.` |

Beside the state, not instead of it:

- A request the server proved unsent (`rt_core_connection_reopened` as the
  reason, or a failed request from this panel whose
  error reads broken pipe / connection refused / connection reset) adds
  `Request did not reach the machine; retry Arm.` (or Stop, Disarm) to the
  Arm step. The state and the pill do not move.
- A background `rt_core.note` reconnect does not prove the operator's last
  command was unsent and must not produce that retry claim.
- Any other non-blocking reason (`operator_stop`, `control_session_stale`,
  a refused jog) appears as `Last result: ...` on the Arm step, as before.
  `busy` and `rt_core_request_slow` are never shown as a last result; they
  are the transition.
- `rt_core.stop_uncertain` appends `The server reports the last Stop outcome
  unconfirmed.`; `rt_core.recovery_action` is appended to a blocked Arm
  step's summary.

While a request is in flight (`arming`, `disarming`) Arm and Home are not
offered; STOP always is. The Arm button stays available in `unknown`
because the server is the authority on whether Arm is admissible; the
headline says plainly that the state is not known from here.

`machineFlow.test.ts` ("one armed fact: ...") asserts, for each state, that
the pill label, the Arm headline, the Drive step and the Jog caption carry
the same fact; `densePanel.test.ts` ("Arm headline, header pill, run row,
Drive step and Jog caption agree ...") does the same against the rendered
DOM through a disarm, arm, stop, transport-failed arm and stale-status
sequence.

## State mapping

Inputs to `deriveMachineFlow`: the catalogue and the highlighted cell, the
server-remembered selection, capabilities (`backend`, described axes,
`nativeHome`), the last dense status (`armed`, `session`, `daemonError`,
`rtCore.coreArmed`, `rtCoreFlags`, per-axis `homeValid` and `readiness`,
`rtCore.ready`), the age of that status, this panel's request in flight and
its last completed request, and two panel facts (a pending switch, a
pending arm confirmation).

Reasons in `daemonError` split two ways. Blocking reasons mean the machine
will refuse or cannot be reached (`rt_core_inhibited`, `rt_core_closed`,
`rt_core_transport_lost`, `rt_core status unavailable`, `rt_core_fault`,
`rt_core_event`, `control_already_owned`, `session_principal_mismatch`,
`target_changed`) and block Home, Arm and Drive. Every other reason
(`operator_stop`, `control_session_stale`, a refused jog) is the result of
the last action and appears as `Last result: ...` on the Arm step; Arm again
is the recovery and the server stays the authority.

The header pill comes from the same view (`machineStatePill`):

| Situation | Pill |
| --- | --- |
| no machine connected | `NO MACHINE` |
| no machine, Production mode, legacy read-only robot observed | `CONNECTED · READ-ONLY · ARMED` or `DISARMED` |
| simulation connected | `SIMULATION · <cell> · DISARMED` (green), `ARMING`, `ARMED`, `STOPPING` / `DISARMING`, or `STATUS STALE` (amber, dashed) |
| real connected | `REAL · <cell> · DISARMED` or `ARMING` (red edge), `ARMED`, `STOPPING` / `DISARMING` or `PLAYING` (red fill), or `STATUS STALE` (amber, dashed) |
| posture unknown | `OUTPUT UNKNOWN · <cell> · ...` (red), same tokens |

The token after the cell is `ArmFact.token`; the footer shows `MACHINE:`
followed by the same label.

## Where the old controls went

| Before | Now |
| --- | --- |
| Header pills TESSERACT / SIMULATION / SIM ARMED | One machine pill; the planner state stays in Planner > Status. |
| Header SIM ARMED toggle | `Arm preview` in the Jog panel's Cartesian section. |
| Header operator-feedback strip (local jog) | Beneath the Cartesian pad, for a connected machine's refusal; the browser preview shows one line only when a preview hold is refused, never the per-axis fence dump. |
| Robot > Deployment target select, Connect, ARM/DISARM/GO HOME | Steps 1, 2, 4, 3 of the Machine panel. |
| Robot > joint jog table | Jog panel, Joints section. |
| Toolbar JOG Cartesian pad | Jog panel, Cartesian section; JOG still toggles it. |
| Toolbar LOAD TO ROBOT / play / pause / stop / STOP / readout | Machine panel, Drive step (run row) and STOP at the panel foot plus the header. |
| Toolbar REAL · PHYSICAL OUTPUT tag | Machine panel posture banner. |
| Footer `REAL OUTPUT: DISABLED` | `MACHINE: <pill>` from the same view. |

Nothing in the server API changed. Element ids used by tests (`#dense-*`,
`#spreadsheet-jog`, `#toggle-jog`, `#jog-frame`, `#spreadsheet-cartesian-grid`,
`#jog-operator-feedback`) are kept.
