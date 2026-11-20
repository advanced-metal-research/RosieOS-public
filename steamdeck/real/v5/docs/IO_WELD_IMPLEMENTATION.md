# RevPi I/O and taught welding implementation

Status: implemented and tested offline and on a cell's pendant; physical
output switching and weld qualification are accepted per cell, separately.
Scope: the native RevPi DIO channels, the v5 pendant's I/O panel and setup, and
taught weld boundaries.

## Operator decisions

- Complete execution support before deploying; do not ship a monitor-only or
  output-only subset.
- General-purpose I/O: clamps were an example, not a dedicated machine workflow.
- Manual tests use tap ON / tap OFF. No automatic robot arming or motion.
- Rosie owns path, requested constant TCP weld speed and weld start/stop;
  power-source process parameters remain on the welder. An unassigned weld
  command must refuse before execution.
- Preserve existing joystick mapping, L2 arm/disarm and Trace behavior.
- Physical acceptance remains separate from offline checks. Never change a
  joint reference to make an I/O test possible.

## Implementation boundaries

One native owner writes the output process image. Do not add piTest writes,
a second service, or a UI writer that races rt-core. Inputs, output intent,
submitted command and independent feedback remain distinct; an output image
is not physical relay/arc confirmation. Stale observations display UNKNOWN.

The RevPi DIO has 14 inputs and 14 outputs. Native named-channel capacity is
sixteen in each direction, so all 14 channels map. Protocol minor 15 requires a
coordinated native/adapter/OLP/pendant update. The maximum preallocated
trajectory store is 240,000,168 bytes (320-byte sample layout); no unbounded
allocation is added. Scoped input checks inspect at most sixteen channels each
cycle.

Manual test authority is separate from motion permission: disarmed drives,
explicit test session, bounded heartbeat, newer OFF fencing late ON, expiry
on loss, and OFF on leaving the test session. Existing faults still refuse ON.
No implicit Reset, Home, Enable or Arm is permitted. Transition to robot Arm
ends manual testing.

Program actions must execute in authored order at defined stationary boundaries:
set output, pulse output with duration, wait for input with state, stable time
and timeout; timeout stops execution and reports the row. Stop, disarm, lost
control and faults invalidate authority and clear output intents. A satisfied
input cannot be invented from stale telemetry. Trace never executes process I/O.
Dry-run behavior must be explicit, with no physical process output.

Weld start/stop are semantic boundaries in the taught program, not an alias the
user can accidentally bypass through ordinary I/O. Validate assignment, matching
start/stop, allowed motion types, speed and waits before Load. Preserve existing
programs and saved revisions. Saving a waypoint never changes an output.

Machine channels that are not general-purpose I/O (an output wired to the
E-stop chain, a drive-enable output) are removed from native executable I/O in
the cell's machine configuration. Treat them as reserved machine controls,
never as test channels: switching an output wired to the E-stop once took a
cell's EtherCAT link down and invalidated Home on every axis.

## Gates before deployment

1. Native channel/marker capacity, manual authority, OFF ordering, expiry,
   Stop/fault/connection-loss tests; generated protocol/API checks.
2. Program schema/planner/runner tests for ordered actions, pulses, wait success,
   timeout, stale inputs, cancellation, reverse Trace and dry run.
3. Weld boundary validation and unassigned-output refusal; speed is a requested
   TCP cruise speed, with acceleration and joint limits reported honestly.
4. Pendant UI tests for save/edit/cancel, row summaries and output monitoring;
   full existing L2, mapping, Trace and waypoint-save regression suite.
5. Canonical build/release checks, preserved programs/configuration, disarmed
   device installation, actual screenshots and installed identity verification.
6. Operator physical testing, recorded separately after deployment.

## Execution

Scoped Monitor input / End monitor compile to immutable per-sample masks.
The native loop checks input validity and required state throughout the scope;
a mismatch/stale input stops execution and clears output authority. Trace and
dry-run strip these process actions, including output markers and waits.

Taught weld spans use one path clock across tangent MoveL/MoveC destinations.
Intermediate waypoints retain nonzero matching velocity, their own row identity,
and exact local durations through dense export. Only the full span ramps from/to
rest. Sharp/non-tangent joins and requested speeds beyond the span's limits are
refused explicitly; no hidden corner rounding or lower weld cruise. The requested
speed is a planned cruise target, not measured physical weld-speed acceptance.
Ramp portions at weld start/stop still exist; there is no power-source handshake,
arc sensing or commissioned run-in/run-out in this release.

A marker is delivered on its scheduled cycle. Wake jitter in the trajectory
origin once displaced a 100 ms marker to 110.205190 ms with a 10 ms simulation
cycle; activation, alignment completion and input-wait resume now use the
scheduled cycle boundary for that origin. The one-cycle marker deadline is
unchanged; do not relax the production delivery deadline to pass a test.

## Pendant I/O setup and weld boundary timing

- **I/O setup** stores channel labels, a selected weld output, and 0–30 s
  boundary-delay defaults in the cell's existing shared program store
  (`io-settings/<cell>.json`). Saves use revision checks and atomic
  replacement. Labels do not rename stable program channel IDs. Defaults become
  explicit values on newly recorded points; existing programs are unchanged.
- Waypoint **Weld start** waits stationary with the trigger ON before departing;
  **Weld stop** waits stationary with the trigger ON after arrival, then turns it
  OFF. Timing uses existing sampled dwell segments and expiry-refresh markers.
  Dry playback keeps timing with outputs suppressed; Trace omits these production
  delay segments. Stop/disarm/fault remain immediate inhibition paths.
- The selector cannot turn a general output into a qualified torch. Arbitrary
  reassignment, polarity and permit wiring still require controller
  commissioning. The UI reports this explicitly. An unassigned selection
  additionally prevents production-weld planning.
- Output testing uses a dedicated persistent pendant HTTP connection: a new
  connection per sample failed on the third sample from a Deck, a persistent one
  completed 60 samples. Renewal scheduling does not wait an additional timer
  period after a slow reply. OFF/end requests retain the window-owned transport
  so closing the panel does not destroy their connection. No autonomous daemon
  renewal is introduced.

The pendant uses the existing theme, Monitor / Output test views, explicit
Channel labels / Weld setup entry points, and an on-screen Stop. Delays use the
existing touch number pad. Labels are split into input/output lists. Assignment
and controller qualification are shown as separate facts: choosing an output
does not qualify or energize the welder.

Session expiry: a Deck's replies to empty test sessions measured 125–349 ms over
Wi-Fi, while the controller answered local requests in 0.9–6.8 ms. Disabling the
Deck's Wi-Fi power saving did not eliminate the failure. The setup token expires
after 500 ms and the pendant allows a 450 ms reply. The independent native output
deadline remains **200 ms**. No background service renews ON, no request is
automatically replayed, and a late renewal cannot revive an expired output without
a new ON tap. This does not make long network stalls acceptable for energized
outputs.

The weld-trigger button opens an in-window, touch-scrollable picker of
controller-described process/torch outputs, showing physical channel numbers and
saved labels. Reserved/unknown channels cannot be saved. Selection and
cancellation do not energize outputs; Save changes persists the requested
channel with the existing revision check. This does not reclassify physical
outputs, qualify welding, or change L2/joystick handling.

## Saved channel routing and motion-only Trace

Planning carries an explicit `weld_output` alongside the unchanged native channel
definitions. Weld Start/Stop resolve that exact channel: process outputs can serve
as test indicators; torch-class outputs still require native qualification.
Empty/unknown/reserved selections refuse. Generic I/O rows cannot compete with the
selected weld channel. Changing assignment affects newly planned schedules; saved
trajectories retain exactly the schedule named by their process-marker digest, so
replan after edits. No native configuration, drive parameters or Home references
are changed.

Trace plans a private dry-run copy while retaining weld path geometry,
boundaries, speed and delays. This copy never replaces the saved production
plan. Native Trace continues to suppress outputs.

Intentional Qt request cancellation is `http_cancelled/request_cancelled`,
with network code and detail, rather than a false connection failure. Timeouts
and real connection failures remain errors. Early `jog_session_stale` responses
are the existing pipelined-begin race handled by HoldPacer; cadence, refusal
handling, L2 and joystick mapping are unchanged.

## Automatic status outputs

Controller-owned indicator outputs (for example a green lamp and an alarm
sounder) are reserved: Deck output testing shows Automatic, and neither can be a
programmed or weld output. See [`rt-core/docs/CELL_IO.md`](../../../../rt-core/docs/CELL_IO.md).

## Manual testing of an assigned weld output

The pendant and controller accept the assigned weld output in the existing
manual session. Motion must be disarmed; explicit begin, sequence fencing,
bounded expiry, OFF and Stop behavior remain in force. Expiry of a manual torch
test turns it off without reporting an automatic-weld marker fault. Automatic
indicator outputs and excluded cabinet channels remain outside manual testing.

This does not qualify the output for programmed welding: production torch
qualification and fast-input requirements remain unchanged. Verification is
limited to native CellIO contracts, composed controller I/O simulation and the
pendant I/O test; physical output switching is left to the operator.
