# Native teach pendant v5

One native Qt 5.15 Widgets application, `bin/rosie-pendant-v5`, for the Deck's
1280 × 800 screen. It looks like OLP (grey panels, blue selection, the
spreadsheet) and edits the programs OLP makes. There is no browser and no web
UI on the Deck. v4 remains the installed rollback application.

## Two processes

- **The app** is the only UI. OLP's own program and machine logic -- project
  serialization, node rows, field specs and edits, record/reteach/via, history,
  run gates, machine steps, telemetry series -- is not ported: it is OLP's
  TypeScript from `offline-programming/v1/ui/src` plus `olp-core/*.ts`, bundled
  by esbuild into `olp-core.js` (ES2016 IIFE) and run in QJSEngine. One
  implementation, so the pendant and the desktop cannot disagree. A prelude
  in `src/olp_core.cpp` supplies what Qt 5.15's engine lacks and replaces
  its `WeakMap`/`WeakSet`: they segfault once a key has been collected, and
  esbuild lowers every `#private` class member to one (a Deck crashed twice
  on it). The 3D view
  imports the URDF's meshes from `olp/robot_description/robots` with Assimp.
  On Steam Deck, gamepad input is read directly from the built-in HID report
  stream; other machines use `/dev/input/js0`–`js3`. Both are readable
  regardless of which window is in front: only the focused
  pendant acts on it, and only one pendant runs ("One pendant per Deck").
- **The OLP server** (`offline-programming-linux serve`, started by
  `start-olp.sh`) runs headless on `127.0.0.1:8794` as the backend. It owns
  the machine session -- the rt-core adapter admits one controller -- and
  serves dense execution (connect, arm, stop, heartbeat, jog, move, load,
  play), `capture-model`, the robot catalogue and `/weld-plan`. Its Cartesian
  solver is `bin/robot-v4-cartesiand`.

`run.sh` starts the server as the `rosie-v5-olp` systemd user unit, then the
app in the foreground. Closing the app stops the unit; the server's shutdown
Stops any machine it armed before releasing authority. Nothing arms, homes or
moves without an operator action.

App options: `--olp <origin>` (default `http://127.0.0.1:8794`),
`--expect-cell <cell id>` (the cell the launcher is for; below), `--bundle
<olp-core.js>` (default `<appdir>/olp-core.js`), `--robots <dir>` (default
`<appdir>/../olp/robot_description/robots`), `--programs <dir>` (default
`~/Rosie programs`), `--windowed`, `--snapshot <prefix>` (renders every page at
1280 × 800 to `<prefix>-<n>.png` after 3 s and exits; layout review off the
Deck).

## Pages

The main header keeps STOP; B / R5 / Esc / F4 remain global stop controls.
Cell I/O setup omits its duplicate STOP button.
The header's Layout & appearance menu switches between light and dark themes
(OLP `style.css` tokens; the telemetry lines use OLP's dark palette); the choice
is kept for the next start.
The header shows the program, connected cell and posture, and the Arm pill.
The Jog program pane shows save/sync state and the revision. Hold L2 to arm;
release L2 to disarm. Arm also clears resettable faults first (OLP README
"Arm clears resettable faults first"): the OLP server confirms an
unconfirmed Stop from the robot's state and resets faults whose recovery is a
reset, then arms. So neither an unconfirmed Stop nor a resettable drive alarm
blocks A; the BEFORE ARMING line says what Arm will do first ("Arm will reset
J5 Er41.1 first"), and a refusal is the server's sentence ("J5 drive alarm
Er41.1 (…) still active — let it cool, then Arm again"). A fault that needs a
new Home or a restart is refused by the server before anything is reset. The
gating is olp-core's shared `machineFlow.ts`; no Deck C++ changed for it, so
it reaches a Deck with the next `pendant install`.

- **JOG** -- the main teaching workspace: camera/3D beside the shared OLP
  program editor; there is no separate Program tab. The Speed column always
  shows joint % for MoveJ or programmed TCP mm/s for MoveL/MoveC (not the jog
  speed preset). Tap that cell to edit its speed; program speed scaling still
  applies at planning. Slow / Medium / Fast jog speed presets, joint or
  Cartesian mode, joint selection, Base/Tool frame and Hold/Step mode remain
  above the panes. The program pane shows name, revision, local save and cell
  sync status, an explicit Save button, selection and insertion point. Edit, Delete and Insert sit below the table; Record waypoint, Reteach and Via are below them
  inside the program pane. The program-name menu holds New, Open, Save as, program speed and history.
  Insert holds Dwell, I/O and Home; the ellipsis holds selected-step actions. Collapsing Program hides its touch actions with the pane;
  its controller shortcuts and the fixed status/Stop remain available. The thin divider has a 60 px touch
  target: drag to resize, release within 64 px of an edge to collapse that pane,
  tap for layout choices, or double-tap for equal panels. Tap the edge tab to
  restore the previous split. The width and collapsed side persist locally.
  Touch jog (also in Controller guide) holds the existing joint
  ±/angle/GO controls, Cartesian pads, numeric speed/step settings, Home joints,
  and All joints to 0°. The zero-return action is also directly on the Jog toolbar.
  Its OLP confirmation and motion gates are unchanged.
  **Return-to-zero speed:** all three zero-return buttons use the selected
  preset's continuous TCP travel (mm/s), TCP rotation (°/s) and percentage,
  including while Step mode is selected. The percentage applies once; stick
  deflection is not part of a button press. The existing joint-space return
  path remains synchronized and may sweep the tool through the cell. The
  selected tool speeds are ceilings, with native joint limits still enforced.
  Confirmation shows the effective rates and preserves them through the press.
  This requires OLP capability `recover_zero_jog_speeds`; an older server is
  clearly unavailable for this action rather than silently ignoring the rates.
  Current support bounds the six-joint arm's model-defined TCP. A return that
  moves external axes, lacks suitable geometry or exceeds trajectory capacity
  at an extremely slow preset is refused before arming. L2, joystick jogging,
  mapping, Trace and Run behavior are unchanged.
  The camera panel switches between orbit 3D, Tool view, Wrist cam and Cell cam. Wrist cam is a separate physical camera, not the former J6 rendering.
  The camera slot (`src/camera_view.cpp`) shows the
  cell's camera as the OLP server relays it
  (`GET /api/offline-programming/v1/cell-camera/frame.jpg`, OLP README "Cell
  camera"): the newest JPEG, decoded to the pane's size, at most 15 a second,
  and only while the slot shows. A picture that stops being replaced is marked
  "NO NEW FRAME" after 1 s and removed after 3 s; it is a look at the cell,
  not a safety function. A cell that names no camera, or an OLP server
  without the relay, shows "No camera stream is configured for this cell".
  The pace and the two ages are unverified first values.
  When the cell camera has video, Cell cam is low-data H.264 instead (about
  800 kbit/s, 0.36 GB an hour, against 12.7 GB for JPEG on a cell camera): the OLP
  server relays the camera's stream (`GET .../cell-camera/video.h264`, OLP
  README "Cell camera") through the same tunnel, and the app decodes it with
  OpenH264 (`src/h264_decoder.cpp`, `lib/libopenh264.so.6`). Not WebRTC as in
  the browser: the Deck's Tailscale runs in userspace-networking mode, so it
  has no route for the sender's UDP. The caption
  says VIDEO or JPEG. JPEG is the fallback: a cell without video (404), video
  that never showed a picture after three tries, or
  `ROSIE_PENDANT_CAMERA_VIDEO=off` in the launcher or `pendant.env`. Video
  that worked keeps retrying video. The other cameras (Wrist cam) are JPEG.
  `tests/video_check.cpp` covers the decoder, the stream and the fallbacks.
  The 3D view draws OLP's stage overlays from OLP's own helpers
  (`viewer/workstation.ts`, `viewer/planned-path.ts`): the authored path in
  weld and travel runs with a dot on every target and OLP's sparse row labels,
  faded while a node is selected, and the kept plan's TCP path, every dense
  sample posed through the view's FK at the model's TCP frame.
  Each control draws its Deck binding as the controls themselves (`src/glyphs.cpp`):
  A/B/X/Y, L1/R1, L2/R2 triggers, L4/L5/R4/R5, sticks with the direction that
  moves, the D-pad, View and Menu. The held control is highlighted.
- **RUN** -- plan through the OLP server's `/weld-plan`, preview the kept
  trajectory with a scrubber, Dry run (process outputs off) or Program outputs,
  RUN PROGRAM (RUN AGAIN after a completion) and STOP; GO TO PLAN START, LOAD
  TO ROBOT and PLAY under ADVANCED · STEP BY STEP. The executing node is
  highlighted. **Arm for program** is a separate, explicit Run-page action:
  it stays armed without holding L2, but grants no Jog, Trace or Home permission.
  **Disarm program**, the header arm pill, B/R5 and STOP end that authority.
  It is offered only on a cell whose manifest sets
  `cell_deploy.olp.unattended_loop_allowed` (OLP reports it as capability
  `unattended_loop_allowed`; owner decision 2026-11-13). OLP also refuses an
  arm request carrying `"program": true` on any other cell
  (`program_arm_not_allowed`). There the Run card has no such button, every
  run needs L2 held, and losing focus stops it as before.
  Motion confirmations stay inside the pendant window so confirming a return
  to the program start does not drop a held L2 grant. Releasing L2 still disarms
  an L2-held grant; it does not cancel separate program arming. While Plan is busy,
  Run, Go to plan start, Load and Play
  are unavailable; Stop remains available.
- **TELEMETRY** -- OLP's drive plots on their own page; Capture reads the last
  seconds of the 1 kHz ring.
- **CELL** -- connect one cell from the server's catalogue (connecting verifies
  identity; it never arms), OLP's Machine steps (Home axes and the rest), pull
  the cell's robot settings, accept the current description. Under the cells,
  ON THE NETWORK lists the cells other OLP servers advertise on the OLP
  server's network, read-only, in OLP's words (`olp-core/discovery.ts` over
  OLP's `targets/discoveredCells.ts`): unverified, compatible or incompatible
  with this package's rt-control client, configured on that server or not.
  REFRESH searches again. Adding one is Manage cells… on desktop OLP; the
  Deck has no Add form. In remote-OLP mode the list is the cell host's
  search (its mDNS on the operator-LAN interface and its tailnet search on
  port 8798); the Deck itself may reach the cell host on a different subnet
  than the one it advertises on, and its apps have no tailnet address. A Deck-local OLP searches only when `planner.env`
  sets `OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES` (the Deck's Wi-Fi interface;
  [Finding cells on the network](../../../offline-programming/v1/README.md#finding-cells-on-the-network)).

The current program autosaves to `~/.local/share/Rosie/Rosie Pendant
v5/current.olp.json`, and to the file SAVE AS or OPEN chose, which is
remembered for the next start (QSettings `program/path`, beside the theme
choice) and kept only while it still holds the same program; the header says
SAVED while that file is written, LOCAL when only `current.olp.json` is. A
`current.olp.json` that exists but cannot be opened is never overwritten: it
is moved to `current.olp.json.unreadable-<yyyyMMdd-HHmmss>`, the status line
says where, and only then is a new program started (`src/program_files.cpp`).
Plans' trajectory bytes are kept under `.../trajectories/` (created by the
first plan). Programs are files, not browser storage.

## I/O and taught weld actions

The navigation bar's **I/O setup** button opens live inputs and outputs. With L2
released and the robot idle, open the **Output test** tab, choose **Enable output testing**, then tap an
output's **ON** or **OFF** button. **End output test**, closing the panel,
loss of focus/connection, an expired test heartbeat or robot arming ends test
authority. Reopening the panel does not restore earlier ON commands. An assigned
weld output, including a torch-class terminal, can be tested through this same
bounded manual session; that does not qualify it for programmed welding.
Controller-owned indicator outputs show **Automatic**: ending a test turns off
the tested process outputs, while drive-power/arm indications continue (for
example a green lamp solid while powered and flashing while armed, a sounder
for two seconds after each successful arm).

**Command** reports what the controller requested. **Feedback** is UNKNOWN
unless independently configured feedback is available; an ON command does not
prove that a relay, valve or welder physically operated. Stale inputs also show
UNKNOWN. Test only channels whose connected equipment is understood.

Use the program pane's **Insert → I/O event** for output set/pulse, input wait,
and scoped input monitor/end-monitor rows. Select a configured channel and its
state; waits include stable time and timeout. A continuing monitor covers the
authored section until its matching end, rather than merely checking the input
once. Execution stops on a failed/unknown monitored input or a timed-out wait.

New waypoint opens a 1000 × 540 child panel on **Motion profile**, with move
kind, TCP speed, constant TCP mode and Weld Start/Stop plus the relevant delay.
Waypoint, Pose and Advanced retain labels, geometry, joint limits and capture
provenance. Cancel discards a new draft; double-A still quick-saves. New legs
inside an active weld, and selecting a weld boundary, default to constant TCP
speed. Existing saved programs are not migrated. This is requested cruise speed,
not a guarantee through acceleration ramps or a waiver of planning limits.

Tap an input/output row in Cell I/O to rename it. The compact editor loads the
latest cell settings and saves against that revision, preserving other labels,
weld assignment and delays. Opening it ends any active output test first. The
separate Channel labels page remains available for bulk setup.

Waypoint settings hold weld start/stop boundaries. The welder's own process
settings remain on the power source. **I/O setup → Weld setup → Select output** assigns
the channel for newly planned Weld Start/Stop boundaries. Save, then replan:
existing plans retain their original output schedule. A process output such as
a test lamp uses its ordinary native permission; a torch-class output still
requires hardware qualification. Rosie validates paired boundaries and path/speed
requirements before execution.
Connected tangent weld legs share a path clock and requested TCP cruise speed;
sharp joins or unattainable cruise requests refuse planning. This does not add
automatic corner rounding, remove entry/exit acceleration, or qualify a physical
weld process. Trace and Dry run suppress process outputs. Physical welder
qualification remains pending. Do not substitute an ordinary output row for a
weld start. See [implementation and validation](docs/IO_WELD_IMPLEMENTATION.md).

## Teach and replay

Record waypoints on JOG, then PLAN and RUN PROGRAM on RUN. What each step
needs, and what the pendant says when it is missing:

- **The program's model is the cell's.** Plans do not cross robots. A program
  starts for the last connected cell's model (QSettings `robot/model`, else
  the server's first robot); there is no model name in the pendant. When a
  cell connects, an empty program (no node, no plan) is started again for the
  cell's model under the same name and the status line says so. A program
  with nodes is left exactly as it is: PROGRAM, RUN and Record say "This
  program is for <model>; the connected cell is <model>. New starts a program
  for it, Open picks another" (OLP's `programModelMismatch`).
- **Record** needs every joint homed and trusted, fresh still feedback and no
  held jog: release the sticks and wait for the robot to settle.
- **PLAN** reads the robot's measured pose as the plan's start. It is
  disabled while this pendant holds a jog or a Move-to, step or Go to plan
  start is running ("Release the sticks and wait for the robot to stop, then
  PLAN"). A finished jog needs nothing else: no STOP, no disarm. If the
  server still sees motion it refuses `planning_start_moving` and RUN shows
  the same instruction; the status line carries the server's code and words.
  A late Plan result must match both the program identity and revision before
  it can be kept; opening another program at the same revision cannot adopt
  the old result. Explicit Plan cancels unfinished background Trace planning and prevents
  another background request until Plan finishes. A ready Trace path stays
  cached; an interrupted preparation resumes afterward if still needed. An
  edited program still prepares its new revision. A successful local A Save,
  double-A quick-save, X reteach or Via capture starts that non-moving
  preparation without the ordinary 750 ms edit debounce. The double-A window,
  cell-store sync debounce and transient preparation retry backoff are unchanged.
  A preparation-key error remains a Trace readiness reason; it does not report
  an already saved waypoint as a failed recording.
  A busy motion/foreground-planning lane discards this one-shot scheduling hint;
  ordinary idle preparation resumes later, without a queued motion command.
  If background geometry finishes
  during a jog, its one validated load waits until motion and the acknowledged
  stop are idle; preparing it then does not rerun the planner. Program or
  connection changes discard that deferred load. Trace-only dry-run plans
  never become production Run plans through this scheduling preference.
- **Local trajectory readers** receive an owned native byte buffer for path
  display, plan start and preview, avoiding a base64 decode on the UI thread.
  All three still use OLP's canonical dense-trajectory decoder. One exact-byte
  native buffer of at most 2 MiB and its Qt ArrayBuffer copy may be retained;
  the existing decoded cache also requires at most 8,192 samples and the same
  program, model and live coordinate identity. Larger decoded results are not
  retained; the native buffer obeys its byte cap. Every plan-start check still validates current calibration,
  Home and controller incarnation. `decoded-trajectory-check` compares binary
  and base64 results at 213, 529 and 6,299 samples, including changed/borrowed
  bytes, malformed data and live-identity refusals.
  The planned-path projection now copies only the coordinates and segment
  colour the Deck consumes, avoiding per-sample browser metadata and iterator
  allocations. It retains every sample, segment boundary and exact numeric
  value; the decoder and both cache bounds are unchanged. Local Qt 5.15
  Linux/amd64 measurements on 2026-11-02 (three alternating fresh-process
  trials against `62c35497`) reduced the native-byte `plannedPathRuns` median
  from 100.134 to 35.262 ms for 8,702 samples, and 428.139 to 142.139 ms for
  36,000 samples. These measure decoding/projection/Qt result conversion,
  not file I/O, display FK, rendering or target Deck latency. The expanded
  Qt check includes 8,192/8,193 cache-boundary cases and the two larger plans;
  `run-plan.test.ts` compares every value against the browser projection,
  including six/nine axes, empty/dwell segments, nonfinite/signed-zero values,
  malformed data and output-array independence. Reproduce the bounded timing
  with `./build/decoded-trajectory-check build/olp-core.js tests/fixtures/example-live
  --benchmark`; omit the flag for all correctness/identity checks.
- **Any rt-core restart, re-Home or configuration change** makes every
  earlier plan `planning_start_stale` at Load: plan again.
- **LOAD** needs every axis homed. When the planner restarted or lost its
  store (`dense_blob_not_found`), the pendant loads again with the trajectory
  it kept at plan time; the server checks the copy is exactly this plan.
- **PLAY** needs the robot Armed and at the plan's first point. A staged plan
  on a disarmed robot says "Arm the robot to play"; if the robot is elsewhere,
  Go to plan start, or plan again from where it is.
- **RUN PROGRAM** is LOAD and PLAY in one press, run by
  the OLP server (`POST .../dense-execution/run-program`, OLP README "Run
  program"): when the robot must arm or move to the plan's first point it asks
  once in the All joints to 0° dialog ("Arm and run", "Move to program start
  and run", or both, every joint's angle → start and travel; A confirms, B
  stops and declines), then arms, moves to the start at the Jog speed, loads
  unless staged and plays. After the program completes the button reads RUN
  AGAIN and does the same (native retired the program and the arm is at its
  end). Any refusal stops it with the server's sentence; STOP ends it at any
  point. A planner that lost the plan gets the pendant's kept copy, as LOAD
  does. The words are OLP's (`runProgram.ts` through olp-core `runView`,
  `runBody`, `runAnswer`). After the question closes the pendant asks for
  input focus back and waits for it (`controllerfocus::regainAfterQuestion`,
  bounded 1.5 s) before the confirmed request leaves: without it a confirmed
  Run 5 times could leave as `background` and the server refused it
  `client_not_focused`. A RUN or FINISH CYCLE press that starts
  nothing (refused, or not sent because a press or move is still in flight)
  shows the "RUN 5 TIMES REFUSED" toast under the header with the server's
  sentence and DETAILS (code, step, whether anything moved first), as the
  browser does (`pressNotice.ts`); it goes by itself after 15 s, × dismisses.
- **ONCE / × N / LOOP** (− + set N) choose a repeat before PLAN and RUN
  (OLP README "Loop mode"): PLAN then asks the planner for the return to the
  program's start, RUN asks once (the return move in the dialog), and the OLP
  server runs every cycle by itself. The RUN card shows "Cycle 3 of 5 · last
  cycle 12.3 s"; FINISH CYCLE ends at the program start, STOP at once. On a
  cell whose manifest sets `cell_deploy.olp.unattended_loop_allowed` every
  loop is unattended: any client may FINISH
  CYCLE and closing this pendant does not stop the robot (the question says
  "Unattended: it keeps running if this client disconnects…"; keep the
  E-stop within reach). Elsewhere a loop is attended: only the pendant that
  started it may FINISH CYCLE and it needs this pendant's heartbeat, so
  closing it stops the robot.

For a load test on a cell that allows unattended loops: choose **Dry run**,
**× N** or **Loop**, Plan, release L2 and
centre the sticks, then select **Arm for program** and **Run program**. Confirm
the program-start/return move when asked. An accepted unattended loop is owned
by the server: leaving the Run page, releasing L2, or closing/disconnecting the
Deck does not stop it. **Finish cycle** ends at the program start; **STOP** ends
it immediately and disarms. **Once** also works without holding L2 after program
arming, but still needs Rosie's normal heartbeat: keep the app open for Once.
Before execution is accepted, leaving Run, losing focus/controller/connection,
or maintenance cancels program arming and any pending start through Stop. An
active program never transfers its authority to the sticks; stop/disarm first,
then use a fresh L2 press for Jog/Trace. Program arming is never restored from
saved settings or automatically re-established after a fault/reconnection.

## Controls

From `src/window.cpp` (`button()`, `dpad()`, `eventFilter()`, `showControls()`):

| Input | Action |
|---|---|
| L2 | Hold to arm for Jog/Trace, or to Home when required; release to disarm that held grant (or Stop the Home) |
| Run · Arm for program (touch) | Explicit program-only arm; no L2 hold; Disarm or STOP ends it |
| Joints: left stick (L2 held) ▲▼ | Jog the selected joint |
| Cartesian: left stick (L2 held) | X / Y |
| Cartesian: right stick (L2 held) | Z / RZ |
| Cartesian: R2 + left stick (L2 held) | RX / RY |
| Step mode | One bounded move per push; centre the stick before the next |
| A | Review a measured waypoint before saving (in a dialog: Save/OK); double press within 350 ms saves one point with the last recorded point's settings |
| L4 | Delete the selected waypoint/event on Jog or Program; Undo in the program menu |
| X / L5 | Reteach |
| Y / R4 | Record via |
| B / R5 / Esc / header STOP | Stop, from every page; B also cancels a dialog |
| LB / RB | Previous / next page |
| View / Menu | Cell page / controls guide |
| D-pad ▲▼ (Jog / Program) | Disarmed: highlight previous/next row. Armed with L2: hold to trace backward/forward one waypoint; release before the next |
| D-pad ◀▶ | Jog speed |
| L3 / R3 | Joints ↔ Cartesian / reset camera |
| Right stick without L2 | Orbit the view; R2 + right stick zooms |
| Touch on the 3D view | Drag orbits, pinch zooms toward the fingers, two fingers pan, double-tap resets; + / − / FIT (whole robot and pedestal) buttons |
| Right trackpad / left trackpad | Pointer and click / scroll |
| Steam + X | On-screen keyboard for text fields |

While L2 is held, the sticks jog on the Jog and the Program page, so a program is taught
without changing page: jog, A, jog, A.
Jog and Program use D-pad ▲▼ for rows while disarmed and held waypoint Trace
while armed. A captures a waypoint for review; the program jog strip shows the
mode (L3 or the touch button; ◀ ▶ choose the joint), the speed (D-pad ◀▶),
what jogging does now or why it is refused, and the live path-hold line. A
planned program loaded but not playing never refuses a jog (Run program goes
back to the start and loads it again); a playing program or a running loop,
between cycles too, does: STOP first. The OLP server refuses a jog, move or
step while a loop runs (`jog_mode_conflict`). Other pages never jog
(`tests/teach_check.cpp`).

Trace keeps an editing cursor separate from the highlighted successor after
deleting a taught point. Forward approaches the next surviving motion point;
backward approaches the previous one, including deletion while stopped partway
through a leg (Panasonic OM0105045E29, section 6.15, pages 44–45). A new plan or
speed edit does not imply the TCP has arrived at the highlighted row. Further
deletions and Undo/Redo preserve that relationship and find the surviving
neighbours. When Undo restores the deleted source, backward Trace returns to
that source once before advancing farther backward, even if the TCP may still
be there; Undo alone is not measured arrival. This conservative return differs
from claiming exact known-at-point behavior. Arrival, deliberate row selection,
teaching a new point or reorder ends the temporary cursor. Trace still requires
a fresh held press and validated path;
these cursor checks alone do not establish physical acceptance. Rosie's MoveC
via remains geometry within one waypoint, unlike Panasonic's separately taught
circular intermediate point; a separate stop at that via is not implemented.

Manual joint-angle moves, Cartesian steps and absolute Move-to actions mark
departure from the Trace path just as a joystick jog does. Reverse Trace then
returns to the selected endpoint; leaving a partial Trace leg preserves its
two endpoints. Local validation and lane refusals do not mark departure.
All joints to 0° also marks departure when already armed or after its recovery
confirmation, not while an unarmed confirmation is merely being displayed.
The immutable prepared path remains reusable after these manual moves.

Continuous joystick jogging is proportional in both joint and Cartesian
modes. The selected speed is the ceiling; the selected axis's normalized
deflection raised to 1.5 scales it (the JSLOP Xbox curve). PadInput keeps its
12% hardware dead zone; a new hold starts beyond 16% raw travel and continues
down to the dead zone. Input is sampled every 4 ms and the latest speed is
sent every 20 ms, without time-based smoothing or a queue of old values.
These are software settings, not measured physical response times; the
thresholds are unverified on the Deck. Native acceleration, jerk and
stopping limits still apply. Step mode keeps its 0.35/0.25 thresholds and
one bounded move per push. Touch holds keep their fixed-speed behavior.

Cartesian jog selects one axis at a time. Once acquired, its direction keeps
priority while its own component remains outside the release dead zone; stronger
competing input is ignored and cannot change its speed or end the hold. When the
active direction is released, it ends as an ordinary quiet `stick_neutral`.
After OLP acknowledges End, the latest still-held direction can start without
centring both sticks. Gentle handoffs use the same proportional curve and
existing acquisition threshold; no minimum full-speed pulse is inserted.
The first move still decelerates to a stop before the next axis begins. This is
sequential single-axis jogging, not simultaneous motion or blended corners.

OLP must advertise `proportional_input`; older servers refuse joystick
holds rather than silently interpreting them at a fixed speed. Each hold
pins its ceiling and sends increasing input sequence numbers so delayed
requests cannot restore an older speed. Changing the selected ceiling
ends the current joystick hold; centre before starting again.
Page, mode or frame changes, focus loss, controller loss, faults, refusals,
and an uncertain End still require neutral/fresh input as applicable. L2 release
and Stop cancel motion; a pending handoff never re-arms. Ordinary overlap does
not emit `stick_direction_change`; actual faults retain their messages.
The full offline suite covers this policy; physical handoff acceptance is a
separate step on each installation.
Controllers are rediscovered once a second after a disconnect.

L2 must be released once with the pendant connected and focused, then pressed.
It arms even if a thumb is deflecting a stick; **both sticks must centre after
arming before any joystick jog can begin**. Joystick mappings are unchanged.
A quick release/re-pull waits for all outstanding Arm/Stop replies, up to a
1.5 s UI grace period (not a measured robot stop time), showing WAIT · KEEP L2
HELD. Release, Stop, focus loss, disconnect, a cell change or timeout cancels
that waiting press. A held L2 at startup or after any cancellation/refusal
never automatically re-arms; a new release/press is needed.
Release sends a priority Stop (disarm), including while Arm is pending. Late
and failed Arm replies are fenced with Stop; failed replies can be ambiguous
about whether Arm reached the robot. All Stops must return before a new Arm.
Re-arming also waits for a fresh status requested after the final Stop reply,
so a cached armed observation cannot swallow the next press. Energised
without this pendant’s grant still sends Arm to OLP to acquire or refuse it.
After an acknowledged defensive Stop, the original refusal stays visible.
Manual motion and recovery require L2. Run-page Arm for program grants a
separate permission only for program Arm, Load, Play, Run and Go to plan start;
it cannot authorize manual jog, Trace, Home or All joints to 0°.

Home requires L2 too: it briefly energises the drives, and L2 is its deadman
(owner decision 2026-11-03). Home runs disarmed (rt-core refuses it armed)
and Arm is refused until Home is valid, so while the machine flow's Home
step is current (a joint without a valid Home, native Home offered,
disarmed) an L2 press sends no Arm: it holds the permit for Home only, the
pill reads L2 HELD · PRESS HOME, and HOME JOINTS or the Cell page's Home
then posts `/home`. Without L2 they send nothing ("Hold L2, then press
Home"). Releasing L2 while that Home request is unanswered, or while the
controller reports Home active, sends Stop; with nothing in progress the
release only clears the permit. Once Home is valid L2 arms and disarms as
before.

R2 is only the Cartesian tilt / camera zoom modifier; A always records.
The header identifies held-L2 versus program arming; tapping it can disarm,
but only the Run-page button can latch program arming. These controls require
physical acceptance against the actual deployed build.

Every End says why (`end_cause`, OLP's journal and jog end reason):
`trigger_released`, `stick_neutral`, `stick_direction_change`, `button_released`
(touch keys), `focus_lost`, `controller_disconnected`, `mode_switch`,
`frame_change`, `speed_change`, `ui_navigation`, `cell_change`, `maintenance`,
`operator_stop`, `operator_disarm`, `selection_changed`, `session_changed`,
`state_poll_failed`, `jog_disabled`, `intent_refused:<code>`,
`server_reason:<code>`, `app_closed`
([OLP: end causes from clients](../../../offline-programming/v1/README.md#jog-end-reasons)).
The legacy `stick_direction_change` reason remains supported for older clients.
The new Deck policy ignores overlap and uses a quiet release followed by a
confirmed single-axis handoff instead.

A machine Cartesian hold is OLP's path hold (rt-core protocol 1.14; see
`offline-programming/v1/docs/RT_CORE_EXECUTION.md`, "Cartesian machine jog"):
the TCP stays on a line frozen at the hold's start. Changing the jog speed
(D-pad ◀▶ or the speed buttons) during a Cartesian hold changes that hold's
speed when OLP's hold reply reported `fraction_updates` (`8613722a`); a joint
hold, or a cell without path holds, releases instead. A cell without the
`path_jog` capability refuses Cartesian holds (`cartesian_hold_unavailable`);
Step still works.

## Teaching workspace refinements (2026-10-16)

Jog's table navigates directly from the shared program session, even before
Program has ever been opened. Its program-name menu contains files, history,
program speed and optional table columns; Edit step opens the same full field
editor as Program. Numeric pose editing remains under Destination; Reteach
replaces that destination with a new measured pose. Program remains available
while this migration is reviewed.

The arrow beside Slow / Medium / Fast opens a draft preset editor. Each preset
stores base translation mm/s, rotation degrees/s and percentage. Percentage
scales these Cartesian rates and the native joint limits; native limits still
apply. Effective TCP rates are shown in the editor and button tooltips. The
initial presets retain existing base rates with 10/25/100 percent; settings
persist locally. Effective rates and percentage must remain ordered Slow ≤
Medium ≤ Fast. Hold rates and Step distances remain separate. Touch jog retains
on-screen motion keys, per-joint angle moves and Home; Controller guide is help.

A captures the measured pose immediately, then opens the shared node editor
without adding to the program. Save/A commits exactly one waypoint and one Undo
step; Cancel/B adds nothing. Two A presses within 350 ms skip review and save
one point, including when capture-model is still being fetched. Motion type,
speed, acceleration and constant-TCP preference copy from the last recorded
point (last program move after restart). Target, capture and circular via
geometry never copy; a new MoveC needs its own Record via. Concurrent program
changes invalidate the draft. L4 deletes only the current selection using the
existing edit gates; the native L4 report is used on Deck and its duplicate
Steam F1 event is ignored. Sticks and L2 are unchanged.

## One pendant per Deck

A Deck once ran four pendant processes over three days, two launchers for
each of two cells. Leaving a game in Steam had not stopped them. All read the
same controller, so every Arm pressed on the visible pendant of one cell also
armed the other cell through a hidden one, and each B press sent Stop to both
cells. Four checks now stand between a press and the wrong robot:

1. **The controller acts only on the focused pendant**
   (`src/controller_focus.hpp`). joydev delivers every press to every reader;
   Steam Input and gamescope route only keys, touches and the trackpads to
   the game in front. A press, a D-pad push or a stick acts only while the
   pendant's window or one of its dialogs is the active window, judged when
   the press is read and again when it is handled. A pendant in the
   background still reads the pad every tick, so nothing queues up for
   later, and drops everything except **B**, which still Stops: Stop (and
   Disarm, the same request) can only stop or disarm, never start motion, and
   an operator who presses B means stop, whichever robot is still moving (a
   program left playing behind another app). With one pendant per Deck there
   is normally no background pendant to send it. Losing focus also ends a
   held jog (`focus_lost`, as before) and cancels a pending second Arm press.
2. **One pendant per Deck user.** `run.sh` takes the user's lock
   (`$XDG_RUNTIME_DIR/rosie-pendant-v5.lock`, `flock`) for its whole life, its
   exit Stop included. A launch that finds it held asks that launch to quit:
   SIGTERM to its app (which closes as the operator closing it would) and to
   its `run.sh`, whose EXIT trap then sends its OLP Stop (disarms and
   releases whatever it held) or stops the local server. The sole exception
   is a server-owned unattended loop shown by one fresh status read (native
   time within 1 s of the native host clock, the expected cell, an armed,
   fault-free controller and the loop's own trajectory): that loop keeps
   running. Idle Arm, pending starts, Once, attended loops,
   faults and unverified observations retain the exit Stop. The new launch
   starts once the lock is free (25 s bound, unverified), else it refuses
   with exit 3 and names the holder. Any pendant still running outside the
   lock, such as an older package or another app directory, is asked to quit
   the same way (15 s, unverified) or the launch refuses. The app takes a
   lock of its own (`src/single_instance.cpp`, `rosie-pendant-v5.app.lock`)
   with the same rule for launches that bypass `run.sh`, and its status line
   names the pendant it closed. The newest launch wins because it is the one
   on screen; a pendant nobody can see is the hazard. Switching cell means
   starting the other launcher: one app, one cell at a time.
3. **The launcher's cell** (`src/cell_identity.hpp`). `run.sh` passes
   `ROSIE_PENDANT_EXPECT_CELL` (exported by the launcher, or set in
   `pendant.env`) as `--expect-cell`. Every 2 s the app reads its OLP
   server's `/targets`. Until the first answer, and whenever that server does
   not serve the cell, has another cell selected, or this pendant's Connect
   or the status names another, the header says `WRONG CELL · this pendant
   is for <cell>`, the red line under the pages gives the reason, and arm,
   holds, moves, steps, Home, Load, Play and Connect to any other cell are
   refused with a held jog ended. STOP still goes out. `pendant install`
   writes the manifest's cell into the launcher it manages; on a Deck whose
   app another cell's manifest installs (`app_owner`), it writes only that
   launcher's cell
   ([v4 README](../v4/README.md#tier-4-steam-deck-v5-pendant-rebuild)). The header
   shows the connected cell's label, `CHECKING CELL · <cell>` before the
   first answer, and `NO CELL · for <cell>` while none is connected.
4. **The OLP server checks too** (OLP README, "Pendant identity and focus"):
   every request carries `X-Rosie-Client-Pid`, `X-Rosie-Client-Expect-Cell`
   and `X-Rosie-Client-Focus`. The server refuses arm and motion from a
   `background` pendant (`client_not_focused`) or one launched for another
   cell (`client_cell_mismatch`), and its journal names the host, pid,
   launcher cell and focus of every arm, Stop and Connect.

Operators: one pendant shortcut at a time. To change cell, start the other
launcher; it closes the first, and that first pendant's exit sends its robot
Stop. A Deck with an older package, or with an app directory that `pendant
install` does not manage, has none of this until it is installed or that copy
is retired.

`controller.vdf` is the app's Steam Input layout: the rear buttons send F1–F4
(L4, L5, R4, R5), the right trackpad is an absolute mouse, the left trackpad
scrolls. Its always-on `controller_action ts_n` requests native touchscreen
passthrough; it does not bind a physical button. Steam's default mouse-emulation
mode merges two fingers into alternating mouse coordinates and cannot pinch.
When updating an existing layout, add **Touchscreen Native Support** under its
Default action set's Always-On Command, preserving every physical binding.
Verify `STEAM_TOUCH_CLICK_MODE` is 4 on Gamescope's root while the app is focused
and that a pinch produces distinct XI2 touch contacts, not alternating mouse
motion. Steam and Quick Access remain system controls. On the test Deck it is
installed as `~/.local/share/Steam/controller_config/app_2729270736.vdf` (the
32-bit app ID of the existing `RosiePendantV5` devkit shortcut). Steam reads
that directory only when it starts: until Steam restarts, the shortcut runs the
default template and the trackpads do nothing.

Steam Deck launches use the built-in Valve `28de:1205` input interface,
opened read-only and nonblocking. `src/deck_input.hpp` decodes its 4 ms
reports into the same PadInput axes, signs, trigger threshold and button
mapping. The application applies its existing 12% dead zone and proportional
curve directly. Steam still handles trackpads, the keyboard and L4/L5/R4;
B and R5 Stop also work directly. Steam and Quick Access presses revoke
arming before their overlays appear. No device feature writes, grabs,
Steam layout edits or forced focus are used.

A valid, advancing report is required before controls are ready. A report
stream absent for 100 ms, read failure or disconnect revokes the hold;
reconnection requires released L2 then a new press, with sticks centered
after arming before a jog. Invalid, repeated and backwards reports cannot
extend input freshness. The Deck does not silently switch back to Steam's
virtual pad when the physical stream is unavailable. Other computers retain
the existing joydev backend.

The controls-offline banner and controller hint read the independent reader's
current acquisition health, rather than the last sample consumed by the UI.
A delayed UI refresh must not label a still-advancing input stream offline.
Action admission continues to check the consumed input and its sticky
release/Stop/loss events; display health grants no motion authority. The
bounded `controller_input_health` diagnostic records health transitions and
stale UI-sample episodes with both sample ages, so a real stream loss can be
distinguished from a slow consumer. `deck-reader-check` covers both cases.

This supersedes the unsuccessful startup-query workaround in ba90d2d9:
the operator's physical trigger moved while Steam's virtual trigger stayed
at rest, even after a successful configuration dump and explicit routing
refresh. The raw report protocol is corroborated by SDL's Steam Deck backend:
https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_steamdeck.c
Physical acceptance of each installed version is recorded separately.

## Build

Ubuntu 22.04 / WSL:

```sh
sudo apt install g++ make pkg-config qtbase5-dev libassimp-dev curl
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
(cd offline-programming/v1/ui && npm ci)          # esbuild and OLP's sources
make -C steamdeck/real/v5 -j4 all
```

`make all` builds `build/rosie-pendant-v5`, `build/olp-core.js` and
`build/robot-v4-cartesiand` (from `motion-server/v1`). The bundle depends on
every non-test `.ts` under OLP's `ui/src`, so an OLP change rebuilds it.

The app links Qt5Qml. Without `qtdeclarative5-dev` installed (no sudo), extract
the packages to the sysroot the Makefile looks for:

```sh
mkdir -p ~/.rosie/qt5qml-sysroot/debs && cd ~/.rosie/qt5qml-sysroot/debs
apt-get download qtdeclarative5-dev libqt5qml5 libqt5qmlmodels5 libqt5quick5
for d in *.deb; do dpkg-deb -x "$d" ..; done
```

Run it on the workstation against an OLP server on :8794:

```sh
cd steamdeck/real/v5
LD_LIBRARY_PATH=~/.rosie/qt5qml-sysroot/usr/lib/x86_64-linux-gnu \
  build/rosie-pendant-v5 --windowed --robots ../../../robot_description/robots
```

## Tests

`make -C steamdeck/real/v5 check`:

- `olp-core/session.test.ts` under node, against OLP's seeded robot catalogue
  (including the program following the cell's model);
- `core-check`: the built `olp-core.js` inside Qt's engine, as the app runs it
  (including PLAN's motion and model gates and the plan refusal words);
- `program-check`: an unreadable `current.olp.json` moved aside before a new
  program is written, the remembered SAVE AS file kept only while it holds the
  open program, SAVE AS asking before it replaces another file;
- `capture-check`: the C++ capture TCP against OLP's own urdf-loader/three.js
  poses for both robot descriptions (`build/capture-fk.json`);
- `deck-input-check`: raw report mapping, trigger boundaries, 100 arm cycles,
  initial held trigger, invalid/replayed input, freshness and reconnection;
- `view-check` (offscreen): input interlock, display kinematics, description
  identity, every robot description's mesh set;
- `maintenance-check` (offscreen): the maintenance notice in Qt's engine, its
  overlay, and `Machine` refusing robot actions while STOP still goes out;
- `control-check` (offscreen): another controller using the robot in Qt's
  engine, its card, buttons and placement clear of STOP;
- `jogend-check` (offscreen): the "Jog stopped" view in Qt's engine, its
  overlay's text, DETAILS and ×, that nothing on it takes focus, and its
  placement clear of the header and status line; thousands of views before
  the first poll and trackers replaced by client changes, so a collected
  private-field key would crash it as it crashed the Deck;
  `olp-core/jogend.test.ts` under node;
- `joglive-check` (offscreen): the live hold line in Qt's engine during a
  near-singularity crawl, at the stop and clear; `olp-core/joglive.test.ts`;
- `stick-check`: the stick hold over recorded-like traces (the 21:17:24
  dominance swap, hysteresis, real direction changes, neutral first).
- `focus-check` (offscreen): the real `Window` against a fake OLP server.
  With another window in front (or none of the pendant's), RB, LB, View, A,
  X, Y, L3 and the D-pad do nothing and no POST leaves, B's Stop goes out
  with `X-Rosie-Client-Focus: background`, this pid and the launcher's cell; a
  press read with focus but handled after it moved is dropped; back in front
  the controller acts again. A server serving another cell shows `WRONG
  CELL`, which goes when it serves the launcher's cell.
- `runpress-check` (offscreen): `Machine` against a fake OLP server that
  refuses a background pendant as `client_identity.go` does. A question
  answered by touch leaves no active window; without taking the focus back
  the confirmed Run 5 times leaves `background` and is refused (as a Deck's
  press once was), with it it leaves `focused` and plays. A
  refused press, a refused Finish cycle and a second press while one is in
  flight each reach the toast with the server's words. With hold-to-arm and
  L2 released, Run, recovery, Arm, holds, moves, steps, Play and Home send
  nothing. Connected to a cell (`tests/fixtures/example-live`) whose joints
  need Home, an L2 press posts no Arm and keeps the permit, Home then posts
  `/dense-execution/home`, and releasing L2 posts `/stop` while that request
  is unanswered or the status reports Home active, nothing otherwise; an
  update notice or another cell refuses Home with L2 held. With Home valid
  an L2 press posts Arm and its release `/stop`.
- `steady-check` (offscreen): the real `Window` against a fake OLP server
  answering with bodies recorded from a Rosie 1430 cell (`tests/fixtures/`)
  and a program store listing two browser tabs. The window stays 1280 x 800
  with STOP and RECORD on it while the store label changes (a header wider
  than the screen once made Qt resize the window past it at every label
  change; a Deck read minimum widths 1688 and 1543 and the UI flickered); idle on Jog nothing is re-polished, shown or hidden,
  and the jog strip and three overlays drawn twice from one view change
  nothing; one focused A press makes one capture and one more waypoint on
  the cell; a press during a capture records nothing and says so; in the
  background A records nothing and says why, and B still Stops.
- `identity-check` (offscreen): the launcher-cell decisions, and `Machine`
  refusing arm, holds, moves, steps, Home, Load, Play and Connect to another
  cell (before the first `/targets` answer too) while STOP goes out.
- `planning-priority-check` (offscreen): a deliberately unfinished background
  Trace plan is cancelled when the operator requests Plan; no duplicate is
  submitted while it runs. Preparation resumes after a refusal, a ready path
  remains cached, edits prepare the next revision, and manual joint/Cartesian/
  absolute moves preserve reverse return-to-point behavior without replanning.
  Opening a different program at the same revision refuses an old successful
  response and leaves Plan available for the newly opened program.
- `trace-defer-check` (offscreen): completion during a real held Cartesian jog
  sends no preparation until its delayed End acknowledgement; idle prepares
  the same bytes once, foreground Plan takes priority without duplicate work,
  and edits or a same-cell reconnect discard stale deferred loads. Preparation
  never issues Arm, Trace Begin or Play.
- `instance-check`: the app lock with real processes: a second launch closes
  the first through its SIGTERM handler and names it; a first that ignores
  the request refuses the second; a dead holder's lock is taken.
- `run_lock_check.sh`: `run.sh`'s lock with two cell launchers on
  one app directory, a fake app and a fake curl: the take-over sends the
  closed launch's Stop to its own OLP and passes `--expect-cell`, a stray
  pendant from another directory is closed, and both refusals exit 3.
- `sync-check`: the cell program sync in Qt's engine (the Deck's program
  uploaded as current, a browser revision opened, a stale save turned into a
  conflict kept as the current copy) and `ProgramSync`'s real transport
  against a port nothing listens on (a local draft that says so);
  `olp-core/cellsync.test.ts` under node.

## Package and deploy

```sh
steamdeck/real/v5/package.sh [--with-tesseract-env]     # a local package (WSL / Ubuntu 22.04)
# a Deck: from steamdeck/real/v4, with the cell manifest naming the builder and the Deck
go run . pendant build --manifest $CELL --commit <sha>
go run . pendant install --manifest $CELL --host <deck> --commit <sha>
```

`go run . pendant build|install|rollback|status` in `steamdeck/real/v4`
wraps this component's scripts with the cell manifest's `pendant_deploy`
hosts, the run journal, transport and hash checks
([workflow](../../../docs/DEPLOY_AND_TEST_WORKFLOW.md#steam-deck-pendant)):

- `build` runs `container/build.sh` in a persistent work tree (the
  `rosie-v5-build:22.04` container or a native builder): `make all`,
  `make check`, then `package.sh` with the Go OLP server cross-compiled
  natively on the Mac;
- `install`, `rollback` and `status` send `deck-install.sh` to the Deck on
  stdin. While a pendant runs on the Deck it refuses while the robot behind
  that pendant's OLP is in use (with none running it checks no robot), checks
  the staged package's `SHA256SUMS` and libraries before touching the app,
  stops a running pendant, swaps only the package-owned files (site files
  stay), relaunches the shortcut if the app was running and checks its OLP's
  health; a failed check restores the previous package (exit 5).

`package.sh` runs `make all`, builds the Linux OLP server, and stages
`build/package/RosiePendantV5`:

- `run.sh`, `start-olp.sh`, `controller.vdf`;
- `bin/rosie-pendant-v5`, `bin/olp-core.js`, `bin/robot-v4-cartesiand`;
- `lib/`: `libassimp.so.5`, `libdraco.so.4`, `libminizip.so.1` -- the build
  host's Ubuntu 22.04 copies, dereferenced, with their Debian copyright files.
  SteamOS has Qt 5.15 (`qt5-base`, `qt5-declarative` 5.15.17 on the test Deck)
  but not Assimp. The source directory's `lib/` is no longer read;
- `olp/`: the tracked and untracked-but-not-ignored files of
  `offline-programming/v1`, `motion-server/v1`, `robot_description`,
  `tesseract/v1`, `weld_planner/v1` and `cadquery/v1`, the server binary, and
  a stub `ui/dist/index.html` (`serve` refuses to start without one; the
  pendant never opens it);
- `SHA256SUMS` over everything except the tesseract pixi environment.

`--with-tesseract-env` adds `tesseract/v1/.pixi/envs/default` (~1 GB), needed
on the first deploy and whenever its `pixi.lock` changes.

`deploy.sh` (copy over ssh, check the hashes) was retired on 2026-10-07: it
was a second, unchecked install path beside `pendant install`.

A Deck package built before a cell's capabilities digest changed refuses the
cell. Rebuild from the commit the cell runs, then `pendant install` with the
owner's approval.

## Run on the Deck

Add `run.sh` as a non-Steam (or devkit) shortcut and install `controller.vdf`
as its Steam Input layout. `steam steam://rungameid/<game id>`, with the
shortcut's 64-bit game ID, launches it from the Deck session. Server output: `journalctl --user -u rosie-v5-olp`.

`run.sh` waits up to 30 s (unverified bound; the API answered within 2 s on the
Deck) for `/api/offline-programming/v1/health`, then starts the app with Steam's
`LD_PRELOAD` and `LD_LIBRARY_PATH` removed, `LD_LIBRARY_PATH` set to the app's
`lib/`, and `QT_QPA_PLATFORM=xcb` (gamescope runs it under XWayland) at scale
factor 1.

## Site files

Per Deck, beside `run.sh`; never committed, never packaged:

| File | Read by | What |
|---|---|---|
| `olp-cells.json` | OLP server (`OFFLINE_PROGRAMMING_CELLS`) | The cell catalogue (`offline-programming.cell-catalogue.v1`; see `offline-programming/v1/docs/RT_CORE_EXECUTION.md`, Choosing a machine) |
| `olp-rt-core.json` | OLP server (`OFFLINE_PROGRAMMING_RT_CORE_CONFIG`) | The local rt-core binding a Local simulation entry needs; leave it absent on a Deck whose catalogue lists only remote cells (`start-olp.sh` passes it only when present) |
| `planner.env` | `start-olp.sh`, if present | Weld planner origin and seam worker launch (below) |
| `pendant.env` | `run.sh`, if present | `ROSIE_PENDANT_OLP_ORIGIN`: use a cell host's OLP server instead of the local one (below); `ROSIE_PENDANT_EXPECT_CELL`: the cell this launch is for, when the shortcut runs `run.sh` itself |
| launcher (`run-<cell>.sh`, the manifest's `launcher`) | the Steam shortcut, which execs `run.sh` | exports `ROSIE_PENDANT_OLP_ORIGIN` and `ROSIE_PENDANT_EXPECT_CELL` (written by `pendant install`) |
| credentials (`*.pem`) | OLP server, via the catalogue | Remote cells' client credentials, named by the catalogue |
| `sim/` | nothing in this package | Deck-local simulation fixtures, test only |

`cell.json` was the retired direct-lane build's connection profile. Nothing
reads it now.

## Using the cell host's OLP server

The local server checks a remote cell against the configuration packaged in
`olp/rt-core/config` and against `olp-cells.json`'s pins, so every change to the
cell's configuration digest refuses Connect (`cell_configuration_mismatch`)
until the Deck is rebuilt from the deployed commit. To follow the cell instead,
point the pendant at the OLP server deployed with it, through a tunnel:

```sh
# pendant.env
ROSIE_PENDANT_OLP_ORIGIN=http://127.0.0.1:18794
```

The tunnel runs on the Deck over the LAN (SteamOS runs Tailscale in
userspace networking, so Deck apps cannot reach tailnet addresses). `go run
. pendant tunnel install|status|logs|restart|stop|rollback --manifest <cell>
--host <deck>` in `steamdeck/real/v4` renders it from the Deck's
`pendant_deploy.decks[].tunnel` block as a systemd --user unit around the
`olp ui` supervisor (ssh back 1 s after a drop, doubling to 30 s) and
refuses to drop it while this Deck's pendant has the robot armed, leased,
playing or jog-held ([v4 README, "Deck
tunnel"](../v4/README.md#deck-tunnel-pendant-tunnel)). The pendant
needs no restart after a drop: it shows the robot unreachable and picks it
up when the tunnel is back. `pendant install` writes that origin into
`pendant.env` (or the Deck's launcher) when the Deck's manifest entry has a
tunnel and neither file names one.

`run.sh` then starts no local server and, when the app exits, POSTs
`dense-execution/stop` to that origin (the server's 5 s heartbeat deadman is
the backstop). That server's machine session is shared with any browser on it:
one operator drives at a time. Planning uses that server's planner. The Deck
still needs a rebuild when OLP's program logic (`olp-core.js`) or the robot's
meshes change; a cell configuration change does not. A change to the cell's
capabilities digest does: `olp-core.js` bundles OLP's `denseClient`, which
refuses a Describe whose `capabilities_digest` differs from the generated
client's. rt-core protocol 1.14 (path jog) moved it from `201fbe81…` to
`b1e28f06…`, so a Deck package built before `8613722a` cannot drive a cell
running 1.14, and one built after cannot drive a 1.13 cell. rt-control's
maintenance route (2026-10-07) moved it again to `725c5bbb…`: a Deck built
before that change cannot drive a cell running it, and the reverse. Rebuild
from the commit the cell runs.

## The cell's program

With a cell host's OLP server that keeps a program store (`pendant.env`,
above; `offline-programming/v1/README.md`, "Cell program store"), the
pendant's program **is the cell's current program**, and the browser OLP and
the other Deck edit the same one. `olp-core/cellsync.ts` runs OLP's own sync
state machine (`ui/src/storage/cell-sync.ts`) on this session's project;
`src/program_sync.cpp` only sends what it asks through the OlpClient (GET and
POST) every 250 ms and hands each answer back. No rule is the pendant's own.

- The header beside the program name says where it stands: `<HOST> · R12`
  (synced at the cell's revision 12), `<HOST> · SAVING`, `LOCAL DRAFT` (the
  cell is unreachable; edits are kept and saved when it answers, unless
  someone saved first), `DRAFT · RUNNING` (the robot has the program loaded;
  saved after Stop), `CONFLICT`, `LOCAL` (the server keeps no store), and who
  else has it open (`· BROWSER (CHROME ON MACOS) EDITING`).
- Another client's save opens here whenever this program has nothing
  unsaved, and so does a new current program (a browser opening a program
  from the cell makes it current). A program with unsaved edits is never
  replaced.
- A conflict asks once: **RELOAD THEIRS** drops the edits made here;
  **KEEP MINE AS A COPY** puts them on the cell as a new program and makes it
  current; Later leaves the header at CONFLICT.
- At start, or on a new cell, a Deck program the cell does not have shows
  the **program picker** once (`src/program_picker.cpp`, rows from olp-core
  `cellSyncPicker`): the cell's saved programs newest save first, each with
  model, revision, "saved 3 min ago by <deck>", ★ CURRENT and ● ON THE
  ROBOT; then UPLOAD THIS DECK'S PROGRAM (its name and when it was edited) and
  NEW PROGRAM. D-pad moves, A opens, B closes. Opening one makes it the cell's
  current program; while the robot runs another, A on it says why
  ("The robot is running Bracket; Stop it before opening another program")
  and the picker stays. An empty cell lists only upload and new; nothing is
  uploaded unasked.
- NEW and OPEN of a file put the program on the cell as its current program.
- **OPEN PROGRAM** (Program page, 2026-10-12) is the same picker in its Open
  mode: the cell's programs with OPEN HERE and who else has each open on the
  second line, then FILES ON THE DECK… (the file list it always had). D-pad,
  A opens, B closes; rows elide and set no width, so the screen-sized dialog
  never asks for more than 1280 px (82ff2371). The rules are OLP's
  (`offline-programming/v1/README.md` "Open program"): every row is refused,
  with the reason, while the robot has a program loaded, staged, playing or
  paused, a loop runs, or a Run press is in flight (STOP first; olp-core
  `cellSyncRunLock`), and the cell's own refusals stand. An open program with
  edits the cell has not accepted asks first: SAVE TO <HOST>, THEN OPEN (A;
  opens only once the cell accepted the save) or DISCARD MY EDITS; Cancel (B)
  opens nothing. When saving is impossible (conflict, cell unreachable) A
  does nothing and DISCARD needs a touch.
- `current.olp.json` stays the pendant's own copy, written after every edit
  and every revision opened from the cell; `current.olp.sync.json` beside it
  keeps which cell revision each program was last equal to and whether it has
  unsaved edits, so a draft made offline is reconciled after a restart under
  the same rules.

## Maintenance notice

The app polls its OLP server's `GET /api/offline-programming/v1/maintenance`
every 2 s (`src/window.cpp`). While a deploy has a notice up
([OLP maintenance notice](../../../offline-programming/v1/README.md#maintenance-notice))
a full-screen overlay (`src/maintenance_overlay.cpp`) covers every page: the
reason, the run's step ("Step 4/8 · Install rt-core on the servo host") and
gate suite, a progress bar, the minutes left ("About 12 min left") or, past
the expected finish, "Taking longer than expected (+3 min)", the step list,
elapsed time, who set it, the release and the run (the run id
`realtime-core maintenance show` prints beside the notice's owner), with STOP
on it while a cell is connected. The words and the decision to block are OLP's
own (`olp-core/maintenance.ts` over OLP's `execution/maintenance.ts`). It stays
up if the server stops answering during the notice, and goes when a reachable
server reports none and its health is ok.

A notice 2 h past its ETA (without an ETA, 2 h after it went up) is flagged
STALE: in the header, in red, with how long it has overrun. The server flags
it (`OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER`, default 2 h), and the
pendant applies the 2 h rule on its own clock too, so a notice whose server
stopped answering still turns stale. A stale notice still blocks; nothing on
the pendant clears it. `realtime-core maintenance show` says whether its run
is gone, and only `maintenance clear` ends it.

While it is up only STOP acts: B, R5, Esc, F4 and the overlay's STOP still
Stop (which also disarms and releases). Every other button, the D-pad, the
sticks, keys and touches on the pages go nowhere; an open dialog is cancelled
and a held jog ends, as when a dialog opens over the controls. `Machine`
refuses arm, holds, moves, steps, Home, load, play and Connect itself as
well, as the server does. In remote-OLP mode (`pendant.env`) this is the
cell's notice. A Deck running its own local OLP server shows the servo host's
copy of the notice too: that server reads it from the selected cell's
rt-control and merges it with any notice on the Deck, and keeps the last one
up while a deploy restarts rt-control. Its arm, jog and move requests are
refused by that server, and rt-control itself still refuses them with
`busy: <reason>`.

Under CONNECTED the overlay lists who is using that OLP server (OLP's
`maintenanceSessions` over the route's `clients` and `machine`), the holder of
the execution session or an armed machine first, and who holds the cell
rt-control's grant (controller name and principal, which may be another OLP
server's client). Every request the app sends
carries `X-Rosie-Client: rosie-pendant-v5 (<hostname>)` (`src/olp_client.cpp`),
a label for that list, not an identity.

A Deck built before this change has no overlay: the server still refuses its
robot actions with `maintenance`, shown in the status line. It arrives with
the rebuild the protocol 1.14 deploy needs anyway (tier 4 in
`steamdeck/real/v4/README.md`). `make check` runs `maintenance-check`: the
bundled view in Qt's engine, the overlay offscreen, and the `Machine`
refusals with STOP still sent.

## Another controller using the robot

The OLP server refuses this pendant's arm, hold starts, Move-to, Home, load,
play and Connect while another of its clients (a browser tab, another Deck)
is using the robot, and refuses everything that starts motion while the
cell's rt-control grant is held by another server
([OLP: another controller](../../../offline-programming/v1/README.md#another-controller)).
The pendant names itself with `X-Rosie-Client-Id: deck-<16 hex>`, one random
id per run (`OlpClient::clientId`), so it is its own controller beside a
browser tab on the same host.

From the maintenance poll's `control` member, and from every 409
`in_use_by_other_controller` on any page (`OlpClient::controlRefusal`), a card
(`src/control_overlay.cpp`, decided by `olp-core/control.ts` over OLP's
`execution/controlNotice.ts`) says who has the robot (label and address, or
the other server's controller name and principal), what it is doing (ARMED,
holding the session, the running or last request, program state) and since
when. It is blue, titled ROBOT IN USE, blocks nothing and sits above the
status line, never over the header's STOP; B, R5, Esc and F4 still stop.
WAIT folds it to a one-line bar (DETAILS opens it) until the holder changes
or another refusal arrives; it disappears when the robot is free. STOP ROBOT
is the same Stop (any operator may stop; Stop disarms and frees the robot),
hidden only when another server holds the grant, where the card says to use
that controller's Stop or the hardware stop. TAKE CONTROL appears only when
the server offers it (the holder disarmed, nothing running, idle 30 s) and
moves nothing. The maintenance notice covers the card while it is up.

A Deck built before this change has no card: the server's refusal shows in
the status line, and it is identified by its `X-Rosie-Client` label. `make
check` runs `control-check`: the bundled view in Qt's engine, the card
offscreen, its buttons and its placement clear of the header and status line.

## Jog stopped

When a jog or Cartesian hold ends without the operator letting go -- the
deadman expired, the path ran dry, a singularity, a joint limit, another
client's Stop, an inhibit -- the OLP server classifies the end and publishes
it as `last_jog_end` on the status body
([OLP: jog end reasons](../../../offline-programming/v1/README.md#jog-end-reasons)).
Every dense status poll feeds it to `olp-core/jogend.ts` (OLP's own
`execution/jogEnd.ts`, the browser toast's tracker) with this process's
`X-Rosie-Client-Id`; when the press that began the hold was this pendant's,
an amber card (`src/jogend_overlay.cpp`) appears under the header, centred,
at most 820 px wide: a JOG STOPPED kicker (· DISARMED when it disarmed; JOG
STALLED; JOG COULD NOT START) and the server's sentence at 22 px, readable at
arm's length -- e.g. "Jog stopped: the robot reached a singularity (J5 =
-88.3°, J3 = 40.1°); jog back the way you came, or reposition with joint
jog." Another client's drop-out is a quiet note (plain edge, 18 px,
"JOG NOTE", prefixed with that client's label). Nothing shows for a release
or for this pendant's own STOP or disarm.

It is a child widget, never a window, and neither it nor its buttons take
focus: the trigger, sticks and every controller button (L2 arms and
disarms, B still stops) keep their meaning while it shows. It goes by itself
after 6 s (a note after 4 s); touch × to dismiss it, DETAILS for the
technical detail (whether a re-pull will work, code and cause, the server's
detail, speeds, travel, intent age, samples ahead), which holds it open until
touched again. The maintenance notice covers it. A Deck built before this
change shows only the server's refusal in the status line.

## Live hold line

Live Cartesian hold details appear only when the operator opens **Details**
in the fixed status footer. They no longer add or expand a bar on Jog or Program
(since October 15, 2026), so the workspace stays still during a hold.
The details widget (`src/jog_live.cpp`, drawn from the status poll's `jog_live` through
`olp-core/joglive.ts` and OLP's `execution/jogLive.ts`) shows PATH HOLD · -X with
speed / requested ("21 / 400 mm/s") and a bar, the server's sentence at
18 px ("limited by: singularity (J5 -88.3°, 1.7° from the wrist-singular
-90.0°) — slowing, stops in ~3.2 mm; back out with +X, or joint-jog +J5"),
amber when the singularity slows it, and a SINGULARITY meter while the
resolver's slowing band is near (fill = 1 - its scale; label "J5 1.7° from
-90°"), red at the gate. After a hold that stopped near a singularity it
stays as LAST STOP · -X with the meter until the next hold or move. It has
no controls and takes no focus.

## Planning and Load

Both need the weld planner's CUDA motion server (`weld_planner/v1`, `pixi run
-e motion motion-serve`, `0.0.0.0:8796`) on a workstation the Deck can reach.
Planning posts the packed request to `<origin>/api/motion/plan`; Load has the
OLP server fetch the trajectory bytes from `<origin>/api/motion/dense/<digest>`.
That store is the motion server's `~/.cache` (on a deployed cell a directory
inside a deployment), so after a `dense_blob_not_found` the pendant loads again with
its kept copy, which the server checks and loads
([OLP README](../../../offline-programming/v1/README.md#teach-waypoints-and-moves)).
A plan made on another client and never kept here still needs the planner:
`dense_blob_not_found` then means plan again. Without an origin the Run page
shows the server's refusal, "no weld planner motion origin configured".

Every `/weld-plan` also runs the seam worker once, to pack the `.weldplan`
(`build_plan_request`) -- with or without a workpiece. By default the server
launches it as `pixi run -e default python -m seam_worker.workers` in
`olp/weld_planner/v1`; the package ships that directory's sources but no pixi
environment, so without pixi on the Deck the plan is refused with the seam
worker's "configured Python launcher was not found". Packing imports only the standard library
(`weldplan.plan_request`, `weldplan.urdf`, `weldplan.program_v2`; a STEP is
packed as bytes, not parsed), so the system Python serves it:

```sh
# planner.env
OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=http://<workstation>:8796
SEAM_WORKER_PYTHON=/usr/bin/python3
# Spares preload OCCT, which the system Python does not have.
SEAM_WORKER_SPARES=0
```

Checked 2026-09-28 in WSL: the packaged server, with `/usr/bin/python3`
(3.10.12) and no pixi on `PATH`, ran `build_plan_request` through `/weld-plan`
to the planner's own program validation; no numpy, OCP, cadquery or numba was
imported. A complete taught-moves-only plan from the Deck has not been run, and
the Deck's Python version is unchecked. The alternative is pixi on the Deck and
`pixi install -e default` in `olp/weld_planner/v1` (the OCCT/CadQuery
environment). Rails authoring (detecting welds from a STEP) is not on the
pendant.

## Retired web kiosk

The previous design ran OLP in a flatpak Chrome kiosk. `pendant install`
moves its `run-qt.sh` out of the app directory with the old package (into
`<app>.previous`); its Vite build left with the old package's `olp/`. The
Chrome profile
`~/.var/app/com.google.Chrome/rosie-pendant-v5` is left in place: it may hold
the web pendant's per-origin project store. Export anything wanted from it
before the owner removes the directory by hand; never clear its site data as a
debugging step.

The native build before that linked v4's `RtCoreJogLane` directly. That lane
is not in this app: the OLP server holds the machine session.

## Traps

- **WSL `/tmp` does not persist between `wsl.exe` calls.** Keep scratch files
  on a Linux path that persists, or in the repo's ignored `build/`.
- **`wsl.exe` re-parses its arguments.** Put the commands in a script and run
  `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-22.04 bash /mnt/c/.../x.sh`.
- **Screenshots on the Deck:** `gamescopectl screenshot`; the app draws inside
  gamescope. Off the Deck use `--snapshot`.
- **Steam's `LD_LIBRARY_PATH` and `LD_PRELOAD`** reach everything a Steam
  shortcut starts and select an incompatible libcurl. `run.sh` drops them for
  `curl` and the app; a shell started from a Steam-launched process still has
  them. Preserving the overlay preload was tested on a Deck and did not fix
  the inactive startup binding. A present `/dev/input/js0` alone does not
  prove trigger events arrive.
- **Qt 5.15's engine rejects `method(){ const method = ... }`** (three.js does
  this) and has no BigInt. The bundle is built with `--minify-identifiers`;
  OLP code in the bundle must not need BigInt.
- **One controller per rt-core adapter.** `start-olp.sh` leaves `ROSIE_RT_*`
  unset so OLP's disconnected local jog does not take the only slot; a desktop
  OLP or local jog on the same adapter locks the pendant out.
- **Never touch the checkout's `offline-programming/v1/ui/dist`.** It is the
  deliberate `:5189` redirect stub and gitignored; the package writes its own.

## Not qualified

Physical stick, trigger and rear-button handling, real robot motion, network
fault handling, and a full plan → Load → Play on the Deck against the CUDA
motion server. Mesh display is not collision verification.

Pendant fixes reach a Deck only with a rebuild and `pendant install`; the
server half arrives with an OLP install. Not yet run on a robot: an
unattended loop with no client connected, Arm's reset-then-arm path with a
resettable alarm latched, the 1.5 s focus regain after a Run question
(`regainAfterQuestion`, unmeasured: the next confirmed press's `focus=`
journal field is the check) and Program-page jog (`ecdc815d`).

## Jog workspace revision — 2026-10-15

The compact Jog editor shares the OLP core and editing actions with Program;
there is no second program or save store. It adapts its row count to available
height. The ellipsis menu's File submenu contains New/Open/Save As; Insert
contains Dwell/IO/Home, with enable, delete, undo/redo and plan speed alongside.
Save writes the Deck's current and named files through the existing autosave
path, then ticks the revision-checked cell sync. A local save is not reported as
a confirmed cell save: the adjacent sync label is the source of that status.
The fixed single-row footer shows connection, observed arm and lease state,
and the current notification or refusal. The idle L2 instruction remains in
the arm pill instead of repeating below. Details contains full reasons,
freshness, Home status and the live hold diagnostics. Reduced header padding
and 48-pixel teaching buttons return height to the program; the table keeps
its 40-pixel rows and controller glyphs retain their size. Controller-offline
and arm-blocker warnings still have their own visible line when needed.
The insertion hint's tooltip preserves the selected row's status and TCP speed;
full geometry and status remain in row tooltips and the Edit dialog.
Menus block controller arming, recording and jogging while open; B still stops.
The direct HID input, proportional response and joystick mapping are unchanged.

The steady-check exercises the real window at 1280×800, split extremes, both
collapsed states, touch restore, persisted width, popup capture exclusion and
record/save. Set ROSIE_LAYOUT_SNAPSHOTS to a directory when running it to save
layout review images. Physical operator acceptance of this layout is pending.

## UI clarity revision — October 15, 2026

The controller bindings, direct HID reader, L2 arm/disarm state machine and
proportional response are unchanged. UI changes include neutral surfaces,
sentence-case actions, horizontal separators only on populated program rows,
program/file and step-action menus, direct layout choices, and readable status
rows with technical detail available on demand. The Cell readiness text reflects
hold-L2 arming. The Run panel is ordered Plan → Review → Run; long content scrolls
inside the panel rather than overlapping controls. Dry run explicitly means real
motion with process outputs off.

The node editor keeps a fixed viewport and scrolls long field groups. On Deck,
all dialogs remain screen-sized for gamescope's input coordinates; switching
editor tabs no longer enlarges the native window. Touch scrolling and the right
trackpad can scroll long forms; Done remains outside the scroll area.

Cartesian continuous speed limits retain the existing `jog/linear` and
`jog/angular` settings. Step distances now use separate `jog/linearStep` and
`jog/angularStep` settings, initially 1 mm and 1°. Switching mode never converts
a speed into a distance. The keypad displays mm/s and °/s for Hold, mm and ° for
Step. This changes numeric setting storage, not stick axes or buttons.

Regression coverage includes retained continuous settings across mode changes,
independent initial step distances, fixed editor geometry across field groups,
existing saved-program/sync behavior and the full control/focus suite.


## Charcoal theme, direct actions and 3D gestures — October 15, 2026

The Jog toolbar exposes **All joints to 0°**, using the existing recovery gate
and confirmation. **Delete** is beside Edit/Insert in the resizable program
pane (Delete waypoint when wide), and on the full Program page. It deletes
only the selected item; Undo remains in the program-name menu. Jog controls
is the renamed Touch jog dialog, containing on-screen jog buttons and numeric
speed/step settings. The joystick mapping and L2 arm/disarm are unchanged.

The dark palette uses neutral charcoal surfaces and muted blue selection,
matching the owner's reference. A Rosie 1430 arm whose description matches the detailed CAD from JSLOP uses it, with blue surfaces
and fine CAD edges; the original pedestal and J6/tool remain in place. The pedestal stays neutral;
Nominal joint frames and detailed asset checksums are checked before loading.
URDF transforms, robot-local calibration, live pose, tool/wrist cameras, path
layers, hidden links and picking remain in place. Crease edges are computed
once when meshes load, with faint triangle detail and no bloom render pass.

Two-finger navigation keeps its own contact snapshot instead of Qt's possibly
stale `lastPos`. Contact additions/removals rebase without jumping; stationary
contacts do not replay zoom/pan. Zoom anchors at the previous midpoint before
pan, keeping the scene under the moving fingers. A pinch cannot become a
reset double-tap when a finger lifts. `view-check` covers these transitions and
crease extraction; `steady-check` covers visible actions at three pane widths
and Delete/Undo against the fake program store. Physical touchscreen acceptance
must still be distinguished from these offline checks.

The detailed arm assets are display-only under `assets/robot-visuals/rosie1430`,
packaged into `share/robot-visuals`. They never enter OLP's robot-description
manifest, collision model or planner. The six CAD parts retain their original
triangle counts and source hashes; view-check verifies their source-CAD reference bounds
against JSLOP's provenance (within 0.5 mm, accounting for its J5 frame rotation) and unchanged frame/TCP transforms
through articulation. Missing/corrupt assets or different nominal frames are
reported rather than silently showing a simplified proxy.

FIT always frames the whole robot and pedestal, even after zooming or when a
program is selected. The separate **Layout & appearance → Fit program path**
action retains the focused path view. Neither action moves the robot.

### Cell I/O labels and weld delays

Open **I/O setup → Channel labels** to label configured inputs/outputs; **Weld setup** saves weld-delay
settings to the cell. Labels appear in monitoring and channel pickers; stored
programs keep their original channel IDs. Saves are revision-checked. A different
pendant reads the same cell settings when connected; reopen setup to reload.

Tap **Select output…** (or the currently selected channel) to open the touch
output picker. It lists available process/weld channels by number and label;
reserved channels are excluded. Choose a channel, then **Save changes**. This
saves the requested assignment; if the native controller is configured for a
different weld channel, the screen says controller setup is required and weld
execution remains blocked. Cancel preserves the previous choice.

Set start/stop delays in seconds
(0–30). New waypoint drafts copy those defaults. Selecting Weld start or Weld
stop in the waypoint editor exposes its individual delay. Start delay holds at
the start with output ON before motion; stop delay holds at the endpoint before
turning output OFF. Existing waypoints without these fields use zero delay.
Dry playback retains timing but no process outputs; Trace skips weld delays.

The pendant does not qualify welding or change physical channel polarity, safety
roles, permits or independent feedback. Those remain commissioned controller
configuration. An unqualified trigger is clearly shown and refused for welding;
reserved safety/drive channels are omitted from executable general I/O.
