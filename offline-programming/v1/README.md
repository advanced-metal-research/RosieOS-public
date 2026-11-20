# Offline Programming V1

Deployment target provides Cartesian and joint hold-to-jog and signed move-by-angle through one selected rt-core grant, with live positions and STOP/DISARM cancellation; see [Cartesian machine jog](docs/RT_CORE_EXECUTION.md#cartesian-machine-jog).

## Teach waypoints and moves

Use the existing **Jog** controls to position the robot, release jog, then choose
**Teach → Record waypoint** (also in the spreadsheet). Live capture requires fresh,
stationary, verified feedback for every model joint. In Preview pose the action is
labelled **Record preview waypoint**; it records the simulated pose without motion
output. TCP coordinates use the same calibrated robot model as the native planner.

Each spreadsheet waypoint defines its incoming **MoveJ**, **MoveL** or **MoveC**.
Double-click/Edit to name it and set its destination, joint speed scale and Cartesian
speed. MoveJ uses joint angles; MoveL preserves the full TCP pose along a straight
line. For MoveC, select the destination row, jog to the intermediate pose, and choose
**Record via**. The circular path passes through that pose to the destination.
The planner proves a MoveL/MoveC's planned TCP stays within **0.01 mm** and
0.001 rad of the taught line or arc, continuously (the robot's quoted
repeatability; 0.1 mm before), and the dense export keeps what rt-core
commands every cycle inside it (a Rosie 1430 test program: at most 0.0027 mm at
400 mm/s; `weld_planner/v1/README.md`, "MoveJ / MoveL / MoveC"). That is the
commanded path; how closely the drives follow it is separate (measured TCP up
to 1.2 mm off the line in 400 mm/s holds on a Rosie 1430 cell).
Cartesian poses are in world coordinates. Moves stop at each waypoint; blending is
not implemented. **Reteach** replaces a selected destination with a fresh observation.
Ordinary edits preserve its original observation and source identity.

Waypoints reuse program save/reload, undo/redo, enable, reorder and delete. They may
surround Rails/Waypoint weld blocks or form a standalone program without CAD.
Legacy posed-polyline welds remain on the classic planner and must be recreated
with Waypoint welds before mixing them with taught moves. Native planning keeps
authored order, verifies full Cartesian orientation, robot/fixture collision and
robot limits, and stores the resulting trajectory in the existing Plans list.
In mixed programs, a retract followed by a waypoint or another weld approach
ends at a reachable departure standoff, then continues from there. It does not
insert a return home between those rows. A terminal retract or a retract before
an explicit Home retains the existing return-home behavior.
Calibration changes require reteaching captured waypoints. Digital-output nodes are
scheduled against the connected cell's outputs or refused by name, never silently
skipped (table below, [`docs/IO_SCHEDULE.md`](docs/IO_SCHEDULE.md)).
Use **Plan** and preview playback, then the normal **Arm → Go to plan start → Load → Play**
workflow for the selected machine. Recording never arms or moves it.

For a connected program with any motion node, **Plan** captures a stationary start
from at least five advancing native observations over at least 100 ms, with each joint
within 1e-5 rad of the fixed first observation and no active motion, Home or jog.
The 500 ms capture deadline and supported native state gate planning; they do not
certify physical standstill. Native `logical_velocity` is currently absent and is
not replaced with zero. The request
pins those positions, robot/calibration identity and native configuration/Home
generation in `start_state.json`; it never assumes the model reset pose is the
current robot pose. Its dense first sample must equal that captured start.
**Play** requires fresh verified feedback at that sample and the same native
identity. A changed Home/configuration or moved robot requires replanning or an
explicitly reviewed recovery. Offline preview keeps the model reset default.

What each node plans from (2026-10-11: this is a general purpose robot, it
must be programmable from wherever it is). The program is flown in authored
order from its first configuration: the measured pose when connected, the
model reset pose offline or with **Plan from program start**.

| Node | From the measured pose / from the program start |
|---|---|
| MoveJ, MoveL, MoveC | Supported: IK-solved, continuously verified from wherever the previous node ended. |
| Dwell | Supported: holds the pose it follows. |
| Home | Supported. In a program with no weld block it is a MoveJ to the reset pose at a recorded waypoint's defaults (speed scale 1, 250 mm/s since 2026-10-11, shown on the Home row as "250 (default)"), checked by the continuous collision and limit verifier with no obstacle search, so it plans on the CPU planner. A Home already at reset is skipped. With welds it is a searched free-space transit. |
| Approach, transit, retract, weld | Supported in the planner: the weld keeps its verified trajectory, the free-space legs connect from where the arm is (approach from the measured pose through the weld's standoff). A measured-start program keeps authored weld order (the model-reset planner may reorder a weld-only program). Needs the GPU planner: weld search and free-space search use Warp, which the CPU environment (`motion-cpu`, a CPU-only OLP host) does not have (#157). |
| IO | Planned against the connected cell's outputs ([`docs/IO_SCHEDULE.md`](docs/IO_SCHEDULE.md)): the OLP packs the cell's Describe outputs, the planner gives the row a 0.05 s dwell with the output switched on its first sample, refreshes every ON inside its expiry and ends with an explicit OFF. Offline: refused, `taught_program_unsupported_node`, "Gas on (io) switches an output: plan with the cell connected, so the plan knows the cell's outputs and their expiry" (the pendant and the browser say it before posting). A cell with no outputs configured: `io_not_configured`. An unknown channel (the `revpi:1` placeholder) or an unqualified torch output: refused naming the cell's outputs. Production Load and Run program take the plan record's schedule digest (both apps send it): refused `process_markers_unbound` without it (an older browser or Deck; a dry run still loads), `process_markers_mismatch` for another schedule, and by name for an output the cell does not describe; Play arms the cell's outputs (`io_arm`) before Start and the program's end disarms them; an open fast input is refused in words with the program still loaded. The browser (under Run) and the Deck (Run page) show each output's command, the program's intent and the wired readback. Simulation-tested only (2026-10-12). Before 2026-10-11 a weld-only program dropped its I/O rows without a word. |

**Plan from program start** is the alternative when the measured pose cannot be
used (a capture refused, a first leg the planner cannot verify from where the
robot stands). It is the operator's choice, never a fallback: the browser's
plan-failure dialog offers **Plan from program start** when the measured start
could not be verified. Internal path, collision and service failures do not
receive this advice: a different entry pose does not repair a later move. The
Deck's RUN page has a **FROM PROGRAM START** toggle for the next PLAN (cleared
once a plan is kept). The request carries `plan_from_program_start: true`
(`weld_plan.go`; refused together with `require_measured_start`), no capture
is taken, and the plan starts at the model reset pose for every node type.
Both apps then gate Play on the plan start (`runControls.ts`
`gatePlayOnPlanStart`): "Go to plan start first: Play starts only with every
joint at this plan's first position. No automatic approach is performed."
Go to plan start moves at the selected Jog speed with outputs off. Before
2026-10-11 only the browser applied that gate.
Planning and export each capture their own immutable observation, so separate
requests may have different identities even when joint positions are unchanged.

Taught-move planning refusals identify the motion type, destination label (when
named), authored program step and node ID while retaining the planner's original
reason. A tracking-certificate refusal does not by itself prove a joint-limit
or singularity diagnosis. The pendant clears obsolete failure advice when the
planning start mode changes; independent controller faults remain visible.

A finished jog or move is a planning start while the robot stays armed
(2026-10-10, `weld_plan_start.go`): rt-core keeps motion mode JOG after a
released hold, and its terminal state (completed or aborted, done, nothing
queued, no open lane, no held input) is admitted without Stop, disarm or a
released lease. Planning only reads the robot; Load fences the captured
start again (configuration, Home epoch, controller incarnation, first
sample) and native Play checks the robot is at the first sample, so the
earlier "stopped, disarmed and released" rule added an operator step and no
protection. A jog still open or held, a move still running or queued, or a
joint that moves more than 1e-5 rad inside the capture window is refused
409 `planning_start_moving` ("Release the jog and wait for the robot to stop
before planning"); the browser's Plan button and the pendant's PLAN are
disabled while their own jog or move runs, with the same instruction
(`execution/runControls.ts` `planMotionInstruction`). Every other refusal is
400 `planning_start_unverified` and names its cause (the joint and its
state, a fault mask, Home running, commissioning, the controller's own
refusal).

Every `/weld-plan` and `/weld-plan/export` answer is one journal line:
`weld-plan: program <id> from <client> -> <status> <code>: <message> (pack
… ms, plan … ms, fetch … ms, total … ms)`, the client named by its
`X-Rosie-Client` label as in the maintenance notice. Before this a refused
plan left nothing on the server.

**Load** fetches the trajectory from the weld planner by digest. When the
planner cannot serve it (restarted, cache evicted, store directory pruned)
and the client sent the copy it kept at plan time (`dense_blob_base64`; the
browser from its project store and the pendant from its `trajectories/`
send it only on a second Load after `dense_blob_not_found`, because an OLP
server older than this refuses a body that large), that copy is loaded
instead: at most 64 MiB, decodable, its samples hashing to
the header's trajectory digest and its six-field identity equal to the
Load's, then every existing gate (robot, Home, captured start, native
prepare). The planner's own copy wins whenever it answers; the answer's
`blob_source` and a note say which was loaded, and the journal's
`dense-execution: load` line names it.


Verified through external Chrome on 2026-09-25 using Local simulation: live and
preview capture, reteach, circular via capture, destination/speed editing, validation,
undo/redo, row controls, planning and preview. A five-node MoveJ/MoveL/dwell/MoveC/Home
program completed through Go to plan start, Load and Play, then completed again after
refresh (720 samples, 7.15 s; native done at segment 5, sample 715). Hardware
execution is not part of this verification.

## Local rt-core default

`start-offline-programming.sh` defaults `OFFLINE_PROGRAMMING_EXECUTION_BACKEND`
and `OFFLINE_PROGRAMMING_JOG_BACKEND` to `rt_core`. Retired selections fail with
`backend_retired` in both the launcher and direct `serve`, before startup work.
Dense execution uses the public Go SDK.
Disconnected Cartesian hold-to-jog runs an isolated `robot-v4-cartesiand --backend rt_core`
with its own leader broker against the simulation adapter. The launcher prints
`jog backend:` and `dense backend:` alongside `rails:` and `motion:`.

The dev stack supplies `ROSIE_LOCAL_RT_BUILD/binding.env`. Without a stack
binding the launcher owns a simulation core and adapter, with logs and pid files
under `rt-core/build/olp-local` and sockets under `TMPDIR` on Linux. An explicit
but unavailable binding fails startup. Use `TMPDIR=/dev/shm/rt-core/defaultflips`
in WSL. The public adapter admits one controller at a time. Connected Cartesian
jog shares the app's dense-execution grant; its resolver acquires none. Joint steps,
set-zero and saved-pose Go Home remain unavailable on this Cartesian jog route.
Direct `serve` defaults to `rt_core` and requires an explicit `--rt-core-config`
or `OFFLINE_PROGRAMMING_CELLS` catalogue. A catalogue containing Local simulation
also needs the local `--rt-core-config` binding.

The **Cells** menu chooses a commissioned cell through the native Picker.
**Connect to** includes **None (offline)** and Local simulation. Connecting
verifies Describe identity and configuration without arming. **Manage cells…**
keeps the selected radio, Refresh, Disconnect, Pull cell settings and Add cell.
Refresh re-describes every listed cell; Disconnect acknowledges Stop and Release
before retiring the controller and selection generation. A failed required Stop
preserves the selection. Add cell accepts only a matching commissioned address
and PKI reference; it cannot replace endpoint, credential, model or digest pins.
**On the local network** and **Search local network…** list the cells other
OLP servers advertise; see [Finding cells on the network](#finding-cells-on-the-network).

The Deploy dock shows TARGET, the Machine steps with their reasons, and RUN
with Load, Play, Stop and native progress. Home remains explicit while disarmed.
The title-bar Arm pill uses one click for verified simulation and confirmation
for real or unknown posture; Disarm and the adjacent Stop remain reachable with
the dock folded. **Clear faults** in the title bar requests native fault recovery while the robot
is disarmed. It never enables drives, arms, homes, restarts the controller or
resumes motion. The result names faults that remain, including faults requiring
controller restart and recovery. Fresh connected status and disabled drives are
required; Arm remains a separate explicit action after successful recovery.
Clear faults also reaches a drive in Fault when nothing is latched natively
(native then resets it only if the drive profile classes the alarm
resettable). Arm itself also clears faults whose recovery is a reset before it arms; see
[Arm clears resettable faults first](#arm-clears-resettable-faults-first).

The toolbar Jog toggle opens joint and Cartesian controls
beside the spreadsheet. No selected-machine operation falls back to preview.
The viewer defaults to **Live pose** when a cell connects, regardless of the
remembered dock panel. View settings can select **Preview pose**; choosing
Simulate or starting preview playback also selects the preview. Without a
connected cell, the viewer shows the offline preview.
Joint jog slows to a hold before its configured position limit. A native
`jog_limited` observation keeps the session active and the position reference
valid; release and jog inward to move away. It does not require rearming or
rehoming solely because the limit was reached.
See [How offline programming connects to a cell](docs/CELL_CONNECTION.md) for the one-page picture and [Choosing a machine](docs/RT_CORE_EXECUTION.md#choosing-a-machine) for the detail.

For remote dense execution, set `OFFLINE_PROGRAMMING_RT_CORE_REMOTE` to the
HTTPS/CA/client-certificate/key binding described in
[Remote cell](docs/RT_CORE_EXECUTION.md#remote-cell), and explicitly select
`OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core` for the disconnected browser
preview. The launcher obtains identity hashes from authenticated live
Describe and refuses unavailable or mismatched targets. `DEV_STACK_OLP_REMOTE`
passes the same file to the dev-stack OLP member; `status` shows its saved remote
target. The panel displays the remote host and described axes, including the
six-axis Rosie 1420. Credentials stay on the server. The SSH-tunnel and PKI
procedure, backend pinning, and disabled legacy-jog alternative are documented
in that section.

The app remains on **5189**, the API on **8794**. The redirect stub, separate
rails environment and per-origin storage rules in `CLAUDE.md` still apply.
Cell manifests and release selections are unchanged.

STEP import adds to the workspace. **Import → Weld parts from files…** accepts
multiple files; each keeps its own placement and welds, including repeated
imports of the same filename. Select a part in the tree to edit and plan it.
Planning currently targets the active part; the other parts stay visible and
are included as collision geometry. Switching parts invalidates the active plan.
**Fixtures / obstacles from files…** uses the existing stand/fixture system:
geometry with placement controls and collision checking, without weld tools.
The files are retained in the browser's source store with the workspace.

The main view keeps its original orbit sensitivity and mouse gestures. Camera
drags recover after a lost release or window focus change, and viewport text
cannot start a browser drag that interrupts orbiting. Loading models,
changing paths, selecting welds and opening or closing weld tools preserve the
view. Use Home/Frame or Center camera when you want to reposition it.

After planning, TCP path shows the complete route by default: blue travel,
approach and retract moves, and green welds. Hide or show it in View settings
independently of TCP axes and labels. The trace uses the planned samples, stays
in world space during playback, and clears when the plan is stale or removed.
Frame includes the full route when it is visible; planning never reframes it.

If planning refuses changed robot settings, **Review robot settings** opens
Workspace at **Accept current settings**. Review the before/after joint limits,
gearing, rated speeds and other recorded settings, plus a recorded change date
when available. No confirmation code is required;
the version identities and original diagnostic stay under **Technical details**. Accepting
settings remains explicit and does not automatically plan or run motion.

**Accept current settings** appears only when the robot description changes
(its SHA-256, for example a joint's gear ratio 50 -> 100). A change to the
connected cell's configuration digest alone (feedforward, tracking allowance,
`jog_jerk`) does not offer it: re-select the cell, **Pull cell settings**
(the stored cell settings include the configuration digest, so the existing
plan reads stale) and plan again.

Planning for a connected machine uses its current joint limits and rate
limits as well as the robot model and calibration. An unreachable selected
machine blocks planning until it reconnects or you disconnect to plan
offline. After changing a bench's limits, make a new plan before loading it.
If Load rejects a joint limit, the message names the joint, planned value
and allowed limit when numeric evidence is available; the rejected plan
does not call for a drive reset.

Offline Programming V1 is the desktop CAD, spreadsheet-programming,
simulation, target-planning, and reviewed-execution application for Robot V4.
It shares the complete `robot.v4.program.v1` contract with the Robot V4
SteamDeck teach pendant and the Motion Server-owned program catalog.

Path Planner is retired from this application. There is no Path Planner
adapter, compatibility format, runtime dependency, or virtual SteamDeck
control surface.

## Finding cells on the network

Cells ▾ → **On the local network** / **Search local network…**, and the
section of that name in the Cells dialog, list the cells that other OLP
servers advertise on this server's network (design 2026-10-08):

- **What is found: OLP servers.** An OLP already presents its cells, their
  catalogue pins, its build and health. An rt-control listener answers nothing
  without the mutual-TLS client certificate (on a deployed cell it is
  loopback-only behind the OLP host's tunnel), so probing one would prove
  nothing.
- **How: DNS-SD over mDNS, `_rosie-olp._tcp`,** inside the OLP process
  (`internal/discovery`; no avahi service file, nothing to install). The TXT
  record holds `v=1`, a per-process instance id, host, `reach` (`loopback`
  when the HTTP listener is loopback-only, as on every cell host), build id,
  start time, the maintenance flag, and per catalogue cell: id, model, mode,
  configuration pin, the capabilities digest from that server's last
  authenticated Describe (its selected cell only) and whether it is selected.
  At most 8 cells, one packet (TXT at most 1300 bytes). A browser cannot do
  mDNS, so the OLP the UI talks to searches and serves
  `GET /api/offline-programming/v1/discovered-cells` (`?refresh=1` searches
  again; a result is served for 10 s, forced searches at most every 2 s). It
  queries once from an ephemeral port and listens 1.5 s; it never binds 5353
  to search, so avahi and mDNSResponder keep working. The responder binds 5353
  with address reuse beside them, answers only for its own instance and only
  queriers on its interfaces' subnets, announces at start and when its record
  changes, and sends a goodbye on shutdown.
- **Tailscale** carries no multicast, and a Wi-Fi access point may isolate
  its clients (an operator's Mac and the OLP host on the same Wi-Fi cannot even
  ping each other), so the operator's network is the tailnet.
  `OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT=<port>` (fleet port 8798) starts
  a second, separate HTTP listener bound to this host's Tailscale IPv4 only
  (100.64.0.0/10, found on its interfaces; retried every 10 s until Tailscale
  is up). It serves `GET /api/offline-programming/v1/advertisement`, the same
  record as the TXT, and nothing else: every other path is 404, every other
  method 405, and it shares no handler, client list, session or maintenance
  view with the OLP API, which stays on loopback. The same setting makes the
  Cells search ask every online peer in `tailscale status --json` on that
  port (700 ms per peer, 16 at a time, at most 64 peers, the 10 s cache), so
  a search never touches a peer's OLP API or shows up in its client list.
  `OFFLINE_PROGRAMMING_DISCOVERY_ADVERTISE=0` searches without listening.
  A UDP broadcast probe was rejected: it is mDNS without the standard tools
  (`dns-sd`, `avahi-browse`) over the same networks.
- **LAN interfaces are named, never guessed:**
  `OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES=<iface>[,<iface>]` is used for
  advertising and searching. An OLP host's default route can be the wired
  servo link it shares with the servo host while the operator LAN is its
  Wi-Fi (check with `ip -4 route`). Unset, discovery is off and the route and the UI say how to
  turn it on; an unknown interface turns it off with the reason and never stops
  the server. `OFFLINE_PROGRAMMING_DISCOVERY_ADVERTISE=0` searches without advertising.
- **An advertisement is information, never authority.** Every field read from
  the network is validated (ids, models, digests, lengths); malformed cells are
  dropped and a TXT record naming another instance is refused. Each cell is
  annotated from this server's own catalogue and the repository manifests only
  (`here`: configured here, pin `match`/`differs`/`unknown` against the
  catalogue or manifest pin, the commissioned listener). Health is probed only
  for a `reach: network` server, at the answer's source address on a fixed
  path. The UI marks every row **Unverified**, flags **Incompatible** when the
  cell's capabilities digest differs from its generated rt-control client and
  **Pin differs**. **ADD…** only fills the Add cell form with this server's
  commissioned listener and PKI reference for that id, never the advertised
  address; the operator confirms with ADD TO REGISTRY, which still accepts only
  a commissioned catalogue binding (`cell_not_commissioned`), and Connect still
  runs the mutual-TLS Describe and pin checks. Nothing connects, selects or arms
  from discovery, and nothing sends the browser to another OLP's origin (its
  IndexedDB programs would not be there).

From a workstation, `go run ./tools/olp-discover [--interface en0] [--tailscale]
[--json]` lists what that machine hears, and `--advertise-sample [--interface
en0] [--tailnet]` answers as a sample server (cell `sample-cell`, which no
manifest knows) by mDNS and/or on the workstation's tailnet address, to check
that a search reaches it. `dns-sd -B _rosie-olp._tcp` (macOS) and
`avahi-browse -r _rosie-olp._tcp` (Linux) see the same records. Measured on
this Mac's `en0` (2026-10-08): the sample advertiser ran beside mDNSResponder,
`olp-discover` found it and `dns-sd -L` resolved it with the TXT record intact.

Going live: an OLP built from this change carries it (OLP-only install). A
cell host's OLP keeps it off until the cell manifest's
`cell_deploy.olp.discovery_interfaces` names the operator-LAN interface and
`olp install` writes `OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES` into the unit,
and `cell_deploy.olp.discovery_tailnet: {"listen_port": 8798}` writes
`OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT`. On a cell host, the OLP's own
`avahi-browse` resolves the record beside avahi on 5353, and a passive
capture sees OLP answers on the operator-LAN interface only, none on the
servo link. Its Cells list is empty when no other OLP advertises there, and a
Mac cannot hear it when the access point isolates clients. The tailnet
listener serves the advertisement only, at `<tailnet address>:8798`, with the
API still on `127.0.0.1:8794`; `olp-discover --tailscale` from a Mac finds
the host in about a second (curl it by the full MagicDNS name on macOS). A
Deck-local OLP reads them from the
Deck's `planner.env` site file, which `start-olp.sh` exports; SteamOS runs
Tailscale in userspace networking, so a Deck app has no tailnet address and
its list comes from the OLP it talks to (in remote mode the cell OLP host's,
which searches the tailnet). The Deck's Cells page lists ON THE NETWORK, read-only,
from the next pendant rebuild.

Limits of v1: IPv4 only; at most 8 cells per server; Add cannot commission a
cell missing from this server's catalogue (its PKI files and `olp-cells.json`
entry are host changes); a loopback-only OLP is listed but cannot be opened
from elsewhere; the capabilities digest is known only for a server's selected
cell; anyone on the tailnet that the tailnet ACLs let reach port 8798 can read
the advertisement (cell ids, models, pins, build), never more.

## Cell camera

A cell host can run a camera service beside this server. Set
`OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN` to its http URL (a loopback address;
the server fetches what it names, so nothing else is accepted) and the server
relays it, read-only, to its clients (`cell_camera.go`). A client that already
reaches this server through a tunnel then needs no second route.

- `GET /api/offline-programming/v1/cell-camera` answers
  `{"configured": false}`, or `{"configured": true, "status": {...}}` with the
  service's own status document, or `"error"` when the service does not answer.
- `GET /api/offline-programming/v1/cell-camera/frame.jpg?after=<sequence>`
  answers the newest JPEG after `<sequence>` with `X-Frame-Sequence`. It is
  404 `cell_camera_not_configured` without a camera and 503
  `cell_camera_unavailable` while the camera starts, is gone, or answers
  something that is not a JPEG.
- `GET /api/offline-programming/v1/cell-camera/video.h264` relays the camera's
  low-data H.264 stream when the camera service has video configured
  (`cell_video_stream.go`; the stream format is in `cell-camera/v1/README.md`,
  "Low-data video"), about 800 kbit/s instead of the JPEG frames' 28 Mbit/s.
  It is for the Steam Deck pendant, whose userspace Tailscale cannot carry the
  WebRTC sender's UDP, so it shares the tunnel with jog replies. This server
  holds at most about half a second of it: when the client falls further
  behind, units are dropped until the next key frame (the camera is asked for
  one). The SSH tunnel's own buffers are not bounded here (unmeasured; see
  the camera README's "Next steps"). 404 `cell_video_not_configured` without
  video.
  `POST .../cell-camera/video/keyframe` asks the encoder for a key frame.
- `GET /api/offline-programming/v1/cell-camera/depth.z16?after=<sequence>`
  relays a RealSense's newest raw depth frame when the camera service has
  depth configured: 16-bit depth units with `X-Depth-Width`,
  `X-Depth-Height`, `X-Depth-Scale` (metres per unit), `X-Frame-Sequence`,
  `X-Source-Age-Ms`, `X-Camera-Frame-Number` and `X-Camera-Timestamp-Ms`
  (`cell-camera/v1/README.md`, "Raw depth"; the intrinsics are in the status
  document). A body that is not width x height 16-bit units, or a header
  that is not a number, is 503 `cell_camera_unavailable`, never relayed. 404
  `cell_camera_no_depth` when the camera serves no depth.

A cell can have further cameras, other angles of the same cell. Set
`OFFLINE_PROGRAMMING_CELL_EXTRA_CAMERAS` to `<id>=<origin>` entries separated
by commas, for example `side=http://127.0.0.1:8089/camera`; each origin meets
the same loopback rule. The cell camera above stays the one every client
already shows, and its two routes do not change.

- `GET /api/offline-programming/v1/cell-cameras` answers
  `{"cameras": [{"id": "default", "default": true}, {"id": "side", "default": false}]}`:
  the cell camera first, as `default`, then the extras in the order given. It
  asks no camera anything. A cell with no camera answers an empty list.
- `GET /api/offline-programming/v1/cell-cameras/<id>`,
  `.../cell-cameras/<id>/frame.jpg?after=<sequence>` and
  `.../cell-cameras/<id>/depth.z16?after=<sequence>` answer that camera as the
  routes above answer the cell camera; an id the cell does not have is 404
  `cell_camera_unknown`.

The wrist camera is the extra camera `wrist`. Its older
route `/api/offline-programming/v1/wrist-camera` and `.../frame.jpg` still
answer it, for pendants built before the list, and an older OLP unit's
`OFFLINE_PROGRAMMING_WRIST_CAMERA_ORIGIN` still names it. An absent wrist
camera never falls back to the cell camera.

The camera service's contract is `GET <origin>/status` (JSON) and
`GET <origin>/frame.jpg?after=<sequence>` (waits for a newer frame; 503 while
starting). `cell-camera/v1` is that service: one settings file per camera,
installed on the cell host as `rosie-cell-camera@<name>` by `camera install`;
it captures only while frames are asked for. Its predecessor,
`amr-camera.service` (`/opt/amr-camera`), has the same contract. Frames are
relayed as the service encodes them: the server does not rescale or re-encode.
These routes are not robot actions and are answered during a maintenance
notice. In a deployment the origin comes from the cell manifest's
`cell_deploy.camera` block, or `cell_deploy.olp.camera_origin` for a camera
service the manifest does not install (`steamdeck/real/v4/README.md`, "Adding
a new robot cell"). The Steam Deck pendant shows it under Jog > Camera.

## Drive health

`ui/drives.html` (beside `cameras.html`, on the UI's origin) shows the
selected cell's drives per joint: the heatsink temperature with its last two
minutes, and the last second of torque (RMS and peak, % of rated) and
following error (peak). It reads
`GET /api/offline-programming/v1/dense-execution/drive-health` once a second
(`dense_execution.go`, `internal/denseexec/drive_health.go`), which converts
one ordinary status read into display units; 503 `drive_health_unavailable`
when the cell's controller status is not available. Read-only: no robot
request, no grant, answered during a maintenance notice.

rt-core does the reading (docs/decisions/0001): the drive profile's
`monitor` list (A6-EC: U40.30, 0x2040:31, every 5 s) is read by its
commissioning worker over the mailbox, and its metrics thread summarises
torque and following error from its own telemetry ring. No other process
runs on the servo host for this page, and the page sets no temperature
threshold of its own (the vendor's values are not confirmed); it shows the drive's alarms.
Browser only.

## Cell program store

The programs a robot runs live on that robot's OLP host,
and every client of the robot -- the browser OLP on any machine, both Steam
Decks -- opens and saves the same program there (`cell_programs.go`,
`internal/programstore`). Before it, a program lived only where it was edited:
a browser origin's IndexedDB or the Deck's `current.olp.json`. Two clients of
one robot each held a different spreadsheet and nothing told them apart.

The store is on when the server is started with `--program-store <dir>`
(`OFFLINE_PROGRAMMING_PROGRAM_STORE`; the label clients show, "synced with
<host>", is `--program-store-label` / `OFFLINE_PROGRAMMING_PROGRAM_STORE_LABEL`,
else the host name). A deployed cell names the directory in
`cell_deploy.olp.program_store`, outside every deployment tree, so a release,
rollback or prune never takes programs with it (`steamdeck/real/v4/README.md`,
"Tier 2"). Without it every route answers 503 `program_store_disabled` and
clients show "local only".

What it keeps is the client's document verbatim: OLP's project, exactly what
Project > Export writes (identity, robot model and the robot-description pin,
nodes, plan record). Accept current settings and Pull cell settings are edits
like any other and sync too. The server reads only what it files a program by
(`identity.id`, name, robot model, description pin, `program.programId`); OLP's
TypeScript validates the document on open.

**Rules** (one state machine, `ui/src/storage/cell-sync.ts`, run by the
browser and, through olp-core, by the Deck -- they cannot disagree):

- Every save names the store revision the edit was made on. The store's
  revision is its own per-program counter (1, 2, 3...), never the project's
  `identity.revision`, which each client advances locally.
- A save from a stale revision is refused, 409 `program_conflict`, with the
  cell's revision and who saved it. Nothing is merged or overwritten. The
  operator chooses: **Reload theirs** (drop the edits made here, open the
  cell's revision) or **Keep mine as a copy** (the edits become a new program
  on the cell; on the Deck the copy becomes the current program).
- A retried save with the same `save_id` (its first answer lost) is a replay,
  not a conflict with itself.
- An unreachable cell leaves the edit as a **local draft** that says so
  ("local draft · <host> unreachable · rev 7 + local edits"). On reconnect
  the same rule decides it: saved if the cell still has rev 7, a conflict if
  someone saved since. The browser keeps IndexedDB as its copy and draft; the
  Deck keeps the sync links in `current.olp.sync.json` beside
  `current.olp.json`, so a draft survives a restart.
- **Execution lock**: from a dense Load until that session ends (Stop, or the
  daemon leaving loaded/staged/playing/paused), the loaded program's saves are
  refused, 423 `program_executing`; the edit waits as a draft and is saved
  after Stop. The cell's current program cannot be switched away from the one
  the robot runs. The browser's own editing lock (`editingLockReason`) is
  unchanged; this one also covers the other clients.
- A program this client never synced but the cell has (another browser
  origin, cleared storage) is a conflict, not a blind upload.

**Current program.** The cell has one current program: what the Deck shows.
Opening a program from the cell in the browser (≡ > Open program…, also in
Catalogue) makes it current, so the Deck follows; Catalogue > "Make this
<host>'s current program" does it without reopening.

**Open program** (browser ≡ > Open program…, Deck
Program page OPEN PROGRAM). One list, one set of rules for both clients
(`cell-sync.ts` `picker`, `openCheck`, `openAfter`; olp-core
`cellSyncPicker`/`cellSyncOpenCheck`/`cellSyncOpenAfter`): the cell's
programs newest save first with model, revision, when and by whom saved, who
else has each open (presence), the current, on-the-robot, open-here and
draft-here markers. The browser adds search and a "This browser only" section
(IndexedDB programs the cell does not have; "This browser" when the server has
no store); the Deck's last row is its program files. Rules:

- An edit the cell has not accepted is never replaced silently. The operator
  saves it first (the program opens only once the cell accepted it; a 409,
  423 or lost cell on the way opens nothing and keeps the edit), discards it,
  or cancels; the browser can also keep it as a draft in this browser, and
  that program's row then says DRAFT HERE and opens the draft, saved by the
  usual rules. The Deck's A answers save, never discard.
- No switch while the robot holds a program: a session loaded, staged,
  playing or paused, a loop between its cycles, or the client's own Run press
  in flight (`programSwitchLock`, one wording); STOP (or Finish cycle) first.
  So a switch never leaves a stale plan loaded on an armed robot; the cell
  refuses its current-program switch for the same reason.
- Unreachable cell or no store: said at the top, the cell's rows refused with
  it, this browser's programs and the Deck's files still open.
- No delete or rename here (the store's delete is a tombstone API with no
  operator flow yet). The Deck opens the current program
whenever its own has nothing unsaved. A Deck program the cell does not have
(first start, a new cell) shows a picker of the cell's saved programs once:
newest save first, model, revision, when and by whom, current and on-the-robot
markers, plus upload this Deck's program and new program; one the cell would
refuse while the robot runs another says why (`cell-sync.ts` `picker`, shared
with the browser's banner, which lists the same rows for a browser-only
program). Nothing is uploaded unasked, an empty cell included. New program on
the Deck, or Open file, uploads it as current.

**Presence.** Each client's change-feed request says which program it has
open and whether it is editing (unsaved edits or an edit in the last 15 s).
Clients show the others: "synced with <host> · rev 12 · <deck>
editing". A client is listed while its feed request is under 45 s old.

**Browser.** The header pill beside the program name shows the state (synced /
saving / local draft / conflict / local only); the banner under the header asks
only what the operator must answer: a conflict, or "This program is only in
this browser. Upload it to <host>" (migration: every program authored before
the store is local only until uploaded; Not now hides the offer for the
session). A new program made while connected is uploaded from the start. The
Motion Server catalogue below it in the Catalogue menu is the legacy Robot V4
interchange (`--program-catalog-origin`), listed only where a server has one;
a deployed rt-core cell does not.

**API** (GET and POST only, so the Deck's OlpClient needs nothing new):

```text
GET  /api/offline-programming/v1/cell-programs                    store, current, executing, programs, presence
GET  /api/offline-programming/v1/cell-programs/changes?since=<version>&instance=<id>&watch=<program>&editing=0|1&timeout_ms=<=25000
GET  /api/offline-programming/v1/cell-programs/programs/{id}[?revision=N]   {program: head, document}
GET  /api/offline-programming/v1/cell-programs/programs/{id}/revisions
POST /api/offline-programming/v1/cell-programs/programs/{id}/save   {base_revision, save_id, make_current, document}
POST /api/offline-programming/v1/cell-programs/programs/{id}/delete {base_revision}
POST /api/offline-programming/v1/cell-programs/current              {program_id}
```

`changes` is a long poll: it answers at once when the client's version or
server instance is old, else when anything changes (a save, the current
program, presence, what the robot runs) or at the timeout. Refusals carry
`{error, detail}`: 409 `program_conflict` (+ `current` head), 423
`program_executing`, 413 `program_too_large`, 400 `invalid_program`, 404
`program_not_found`, 507 `program_store_full`.

**On disk** (`<dir>`, mode 0700, one server per directory by flock):
`STORE` (schema and the store's persistent id), `state.json` (current
program), `programs/<id>/head.json` and `programs/<id>/rNNNNNNNN.json` (one
immutable file per revision, written by atomic rename; the last 50 are kept),
`audit.jsonl` (every save, create, delete, current switch and refused save:
time, client id, label, address, revision, sha256; rotated at 16 MiB). A
delete is a tombstone revision; a save from it restores the program. Limits
(design bounds, not measured against real programs): 16 MiB per document, 512
programs, 4 GiB of kept revisions. Backups: the kept revisions, plus the whole
store archived by every `olp install` into
`<deployment>/olp-install/program-store-pre-install.tgz`.

**Not shared:** a STEP part's bytes stay in the browser that imported it (the
IndexedDB `sources` store). A browser opening another browser's rails program
gets its edges and welds but must import the part to see its surface or
re-run rails; the Deck never needs it.

## Authority chain

The editable program and executable trajectory are deliberately separate:

```text
Offline Programming or SteamDeck
              |
              v
  Motion Server program catalog
              |
              v
 selected Robot V4 Motion Server
     plans with target assets
              |
              v
 immutable physical accepted plan
              |
              v
 reviewed server session
  lease/fence -> preload -> controlled arm -> play
              |
              v
 Motion Server -> RTCore -> EtherCAT -> robot
```

The browser can select, edit, display, review, and request server actions. It
cannot open a raw NATS/UDP connection, construct a physical trajectory, arm
independently, jog, enable weld output, or write RTCore/EtherCAT frames.

## What is implemented

- A node spreadsheet over the authored program, `robot.v4.program.v2`: one
  preview row per node (weld, approach, transit, retract, home, dwell, io) in
  both authoring modes, with a parameter dialog per node (grouped fields,
  pose tables in mm and degrees, angle profiles, Save/Cancel) and undo/redo
  over whole revisions. The `robot.v4.program.v1` rows the planners read are
  compiled from the nodes -- in the browser for classic welds, on the weld
  planner for rails welds -- and are shown read-only under ADVANCED. A
  classic weld keeps its baked per-point TCP orientation as the authority;
  a project saved before nodes existed builds its classic nodes on load, and
  a rails program saved before nodes shows its compiled rows with a banner
  until its welds are re-authored. Rails welds inline their seam's curve, so
  a many-seam program's revision history is correspondingly larger.
- The shared Robot V4 program, catalog, and accepted-plan schemas and golden
  fixtures used by TypeScript, Go, Motion Server C++, and SteamDeck C++.
- A same-origin program-catalog client with create, load, update, immutable
  revision history, optimistic concurrency, and typed conflict handling.
- CAD topology/seam/pass authoring through a bounded CadQuery worker.
- A visibly non-executable browser preview.
- Explicit-opt-in local canonical simulation using the Robot V4 production
  Motion Server spreadsheet/Tesseract planner in a contained, hardware-free
  persistent process with a warm, loopback-only Tesseract collision world.
- Server-mediated target planning from the exact saved catalog revision. The
  selected robot creates and retains the only physical accepted plan.
- Read-only Motion Server and RTCore observation normalized against exact
  manifest peers and subjects. Private service bindings never enter browser
  JSON.
- A two-stage physical flow: `Review physical start`, then `Start reviewed
  plan`. The server rebuilds all executable input from its stored plan,
  catalog request, manifest, and latest observations; browser-posted copies
  are not authority.
- Manifest-authorized controller roles, short-lived one-use review tokens,
  fresh lease/fence acquisition and revalidation, exact-plan preload,
  controlled arming, play, continuous renewal/read-back supervision, and the
  full Stop/Disarm/Weld-Off/release protective tail.
- Durable JSONL execution audit and command idempotency state.
- An RTCore bridge final-consumer fence. Energizing commands must match the
  exact live Motion Server lease, controller boot ID, fence epoch, campaign,
  and monotonic intent sequence.
- Graceful server shutdown that stops active sessions before releasing
  process-owned authority.

The spreadsheet toolbar's **JOG** toggle shows joint and Cartesian controls.
When connected, the BASE/TOOL pad drives the selected machine through its
existing armed rt-core session, using live J1–J6 feedback and speed fractions
5% to 100% of each joint's described maximum (the server keeps its measured 0.75
drive reserve under every fraction); in Hold the Speed % scales the mm/s and °/s
boxes. With no connection the Jog controls are disabled; **Arm preview** stays in Simulate. A selected but unavailable or disarmed machine
stays disabled; it never falls back to preview.

**All joints to 0°** remains a joint-space move. Steam Deck sends its selected
continuous jog rates as `linear_speed_mm_s` and `angular_speed_deg_s`, together
with `fraction`. The server limits TCP travel and rotation to those base rates
times the fraction, applied once. Local model geometry bounds rates along the
whole joint path, including maxima between endpoints; no IK or resolver round
trips are added. Confirmation retains the original speeds. The capability
`recover_zero_jog_speeds` prevents a new Deck from using this action against an
older server that ignores the fields. Requests omitting both rates (the legacy
browser) keep the percentage-of-native-limits behavior. Partial/invalid rates
are refused before recovery actions. The native joint ceilings and drive reserve
still apply. Like every Move-to it follows
one shared progress along the straight joint-space segment, so the joint that
reaches its speed, acceleration or jerk limit first slows all of them; on a
cell whose configuration sets `jog_jerk` the progress is jerk-limited by that
value (150 rad/s^3 on a Rosie 1430 cell, unverified) ([details](docs/RT_CORE_EXECUTION.md)).
The bounded return currently supports the model-defined six-joint arm TCP;
moving external axes or unavailable geometry are refused before Arm. An
extremely slow return beyond the existing dense-point capacity is also refused
before Arm with an operator instruction, not a robot fault. Feasibility uses
the fresh recovery observation; actual timing is recomputed from all six fresh
held command coordinates under the existing motion fence.

**All joints to 0° is also the one-press recovery**. The button stays usable while the robot is disarmed, stuck in "last
Stop unconfirmed" or faulted; jogging stays refused. One press is
`POST .../dense-execution/recover-to-zero` `{"fraction", "confirm_arm",
"confirmed"}`, and the server (`internal/denseexec/recover_zero_plan.go`) runs,
re-reading a fresh status before every step and stopping at the first that
does not clear:

1. **Confirm Stop.** An unconfirmed Stop (or Release) is settled by the
   51de1050 rule: one fresh native status, with the reset admission's
   freshness and identity proof, shows disarmed, nothing enabled, no lease and
   no grant. Nothing is sent to the robot.
2. **Reset**, only when every latched fault's native recovery is
   `reset_clears` or `reset_after_condition_clears` and no fault's
   `rehome_axis_mask` says the reset would leave a joint's Home invalid
   (untrusted evidence), or when nothing is latched natively but a drive
   reports Fault (for example a J2 Er41.1 with both masks 0, the alarm only
   on the axis). The native reset decides from fresh evidence whether the
   condition has cleared: since rt-core 99fd3f23 it sends the drive its own
   CiA402 fault reset only for alarms the drive profile's `fault_reset` list
   classes resettable, and otherwise refuses `fault_persists`. A latched drive
   alarm such as Er41.1 stays latched until a reset is sent, so OLP cannot
   know beforehand; a refusal is reported in the drive's words.
3. **Arm**, only after the operator confirms "Arm and move all joints to zero"
   in a dialog that lists every joint's angle → 0° and its travel Δ, and warns
   that the joint-space line may sweep the tool. The confirmation sends back
   the angles shown; the server refuses (`confirmation_stale`) if any joint
   now reads more than 0.1° away from them. Already armed, there is no
   question; already at zero, nothing is armed or moved (`at_zero`).
4. **Move**: the unchanged All joints to 0° move (one `JointMove` to exact
   0 rad for every joint off zero, one shared progress, jerk-limited).

It never disarms, never homes, never resets a fault whose recovery is not a
reset, and never retries a step within one press. Refusals (409 `{"error",
"detail", "recovery"}`, `detail` is the sentence the UI shows):

| Step | Code | Text |
|---|---|---|
| start | `motion_active` | A joint move is running; let it finish or press STOP, then press again |
| confirm_stop | `stop_unconfirmed` | The last Stop is still unconfirmed: the servo host still shows a lease or grant. Press STOP, or use the hardware stop |
| control | `control_held_elsewhere` | Another controller holds the robot on the servo host; recovery stops here. Its operator disarms it, or press STOP |
| home | `home_invalid` | Home is not valid on J3. Recovery never re-homes: an operator runs Home with each joint at its mechanical home pose |
| reset | `fault_not_resettable` | J1 coordinate_reference needs a new Home at the mechanical home pose (recovery never re-homes), not a reset; recovery stops here |
| reset | `fault_not_resettable` | J3 bus_transport lost the position reference on J3: it needs a new Home at the mechanical home pose (recovery never re-homes), not a reset; recovery stops here |
| reset | `fault_persists` | J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again |
| arm | `confirmation_stale` | The joints moved since the confirmation (J2 by 1.00°); press again to see the current angles |
| arm / move | Arm's or the move's code | Arm refused: … / Move to zero stopped: … |

The route sits behind the same gates as every robot action: during a
maintenance notice it is refused 503 with the notice's text, and while another
client of this server controls the robot it is refused 409
`in_use_by_other_controller` naming the holder; the client that presses it
becomes the controller. Every step is one journal line,
`dense-execution: recover-to-zero client=<who> step=<step> outcome=<done|persists|needs_confirmation|refused> detail=...`;
a reset's detail names the joints, faults and drive alarms it was sent for
(`axes=J5 faults=drive_readiness(reset_after_condition_clears) alarms=J5 Er41.1 (…); native faults cleared`).
The Steam Deck runs the same flow (A confirms, B stops and declines).

### Arm clears resettable faults first

Re-arming the robot clears faults too if they exist.
`POST .../dense-execution/arm {"armed": true}` runs steps 1 and 2 of
the one-press recovery above, by the same code (`nextArmStep` and
`settleStep` in `internal/denseexec`), before it arms, for every client:

1. From one fresh native status: an unconfirmed Stop (or Release) is
   confirmed by the 51de1050 rule (nothing is sent to the robot), then
2. latched faults whose recovery is a reset, and a drive in Fault with
   nothing latched, are reset (`ResetFault`, the Reset button's own path), and
   the status is read again;
3. then the unchanged Arm: acquire, enable, arm, observe armed.

With no latched Stop, no latched fault and no drive in Fault, Arm is exactly
what it was. Arm never homes, never widens a bound, never resets a fault that
needs a new Home, a restart or an unknown recovery, and never retries a step
within one press. A Stop pressed after the Arm refuses it
(`control_session_stale`) even while Arm is still reading the robot. Refusals
are 409 `{"error", "detail", "steps"}`; `detail` is the sentence the browser
and the Deck show:

| Code | Text |
|---|---|
| `rt_core_inhibited` | The last Stop is still unconfirmed: the servo host still shows a lease or grant. Press STOP, or use the hardware stop. Stop outcome unconfirmed. Retry POST /dense-execution/stop … |
| `fault_not_resettable` | J1 coordinate_reference needs a new Home at the mechanical home pose (Arm never re-homes), not a reset; the robot stays disarmed |
| `fault_not_resettable` | J3 bus_transport lost the position reference on J3: it needs a new Home at the mechanical home pose (Arm never re-homes), not a reset; the robot stays disarmed |
| `fault_persists` | J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again |
| `control_held_elsewhere` | Another controller holds the robot on the servo host; the robot stays disarmed. Its operator disarms it, or press STOP |

The 200 answer carries the same `steps` (`confirm_stop`, `reset`). Each step
is one journal line,
`dense-execution: arm client=<who> step=<confirm_stop|reset> outcome=<done|persists|refused> detail=...`,
beside the existing `dense-execution: arm=true requested by <client identity>`.

The cross-arm checks run first and are unchanged: a `background` pendant
(`client_not_focused`), a pendant launched for another cell
(`client_cell_mismatch`), another controller (`in_use_by_other_controller`)
and the maintenance notice are refused before Arm runs, so nothing is reset
for them.

In the browser and on the Deck (shared `machineFlow.ts`) an unconfirmed Stop
and a resettable drive alarm no longer disable Arm; only maintenance (and the
existing Home and restart cases) does. The Arm step lists what Arm will do
first: "The last Stop is unconfirmed. Arm confirms it from the robot's state
first …" and "J5 drive alarm Er41.1 (…). Arm will reset J5 Er41.1 first; if
the drive still reports it, Arm is refused with the reason." With a fault
that needs a new Home or a restart the line makes no such promise. A refused
Arm shows the server's sentence for `fault_persists`, `fault_not_resettable`,
`motion_active`, `control_held_elsewhere` and `arm_incomplete`. The explicit
**Clear faults** button stays.

### Run program: one press for a planned program

Native rt-core retires a program when it completes, and the arm then stands at
the program's end, so a replay used to need Go to plan start, Load to robot and
Play by hand. **Run program** (after a completion: **Run again**) is now the
default in the browser's RUN row and on the Deck's RUN page; Go to plan start,
Load to robot and Play stay under **Advanced · step by step**. One press is
`POST .../dense-execution/run-program` with the Load body (identity, `dry_run`,
`dense_blob_base64` only when the planner lost the plan) plus `{"fraction",
"confirm_arm", "confirm_move", "confirmed"}`. The server
(`internal/denseexec/run_program_plan.go`) re-reads a fresh status before
every step and stops at the first that does not clear:

1. **Load's checks without the robot**: fetch the bytes, the identity, the
   robot the plan was made for, process outputs in this run mode. A plan Load
   would refuse moves nothing (`Load refused: …`).
2. **Confirm Stop / Reset** while disarmed, by Arm's and recover-to-zero's own
   code (above): a fault that needs a new Home or a restart refuses before the
   question.
3. **One question** when the press must arm or move: "Arm and run" (disarmed at
   the start), "Move to program start and run" (armed elsewhere) or "Arm, move
   to program start and run", in the All joints to 0° dialog with every
   joint's angle → the program's first sample and travel Δ (a joint within its
   `completion_tolerance` of the start stays). The sweep warning shows when it
   moves. The confirmation sends back the angles and the start it showed;
   travel over 0.1° or a changed start refuses `confirmation_stale`. Armed at
   the start there is no question.
4. **Arm** (it still clears resettable faults first, 4ed9e029).
5. **Go to plan start**, when any joint is outside its completion tolerance:
   the existing jerk-limited JointMove to exactly the first sample, at the
   Jog Speed % shown, with the shared progress.
6. **Load**, unless this plan is staged in this run mode and the arm has not
   moved in this press. Native preparation checks limits and identity again.
   Disarmed at the start, Load runs before Arm, so a refused Load leaves the
   robot disarmed.
7. **Play.** The answer is `state: playing` with the session; completion is
   reported by status as before, and the button reads Run again.

A STOP or DISARM at any point ends the press: every operation it runs is
fenced by the Stop epoch the press started under (`run_stopped`, "STOP ended
Run program during move; …"), and a question answered after STOP sends
nothing. A step's own failure keeps its words. It never homes, never widens a
bound and never retries a step within one press. The route sits behind the
same gates as every robot action: the focused pendant of the displayed cell or
the browser only (`client_not_focused`, `client_cell_mismatch`,
`in_use_by_other_controller`), and the maintenance notice. Refusals are 409
`{"error", "detail", "run"}`:

| Step | Code | Text |
|---|---|---|
| load | Load's code | Load refused: dense_identity_mismatch / process_markers_unbound: … / trajectory_limits: … |
| start | `motion_active` | A program is running; let it finish or press STOP, then press Run program again |
| home | `home_invalid` | Home is not valid on J2. Run program never re-homes: … |
| reset | `fault_persists` | J5 drive alarm Er41.1 (…) still active — let it cool, then press Run program again |
| arm / move | `confirmation_stale` | The joints moved since the confirmation (J1 by 0.29°); press Run program again to see the current angles |
| move | `start_not_reached` | The move to the program's start ended outside its completion tolerance; Play stays refused. … |
| any | `run_stopped` | STOP ended Run program during <step>; press Run program to start again (…) |

Every refusal, and a press a client did not send (another request still in
flight), shows a toast in both apps besides the line under the button: "RUN 5
TIMES REFUSED" / "LOOP UNTIL STOPPED REFUSED" / "RUN PROGRAM REFUSED" /
"FINISH CYCLE REFUSED" with the server's sentence and Details (code, step,
what cleared before it or "Nothing was armed, moved or played"); 15 s, ×
dismisses (`ui/src/execution/pressNotice.ts`, drawn by the Jog stopped toast;
the Deck's olp-core `pressnotice.ts`). Before it, a Deck's confirmed Run 5
times presses were refused `client_not_focused` (the Deck sent the confirmed
request before the window manager gave it focus back after the question) and
the Deck said so only in the Run card's line.

Each step is one journal line,
`dense-execution: run-program client=<who> step=<start|confirm|arm|move|load|play> outcome=<done|needs_confirmation|refused> detail=...`
(the move's detail lists each joint `from->to (delta)`), beside Arm's own
`dense-execution: arm …` lines. A press that staged the program marks it loaded
in the cell program store (423 while the robot holds it), as Load does.

### Loop mode: Run N times, Loop until stopped

Beside Run program the operator chooses **Run once**, **Run N
times** (2..10000) or **Loop until stopped** (browser select; Deck ONCE / × N
with − + / LOOP). The press is the same `POST .../run-program` with
`"repeat": "count"|"loop"`, `"count"` and, on the confirmed press,
`"confirm_loop": true`; the loop itself runs in the OLP server
(`internal/denseexec/run_loop.go`).

- **Closing move.** A loop needs a plan that ends where it starts. With a
  repeat selected, Plan sends the program with one extra node, a `home` marked
  `extensions["rosie.loop_return"]` (`ui/src/rails/planRequest.ts`
  `withLoopReturn`, node id `rosie-loop-return`). The weld planner plans it
  exactly like a Home but to the plan's first configuration instead of the
  reset pose (`weld_planner/v1 .../engine/taught.py`): the verified
  waypoint MoveJ on the CPU planner, the free-space search when the program
  has welds; the same continuous collision and limit verifier as every leg.
  When the program already ends within 1e-6 rad of its start no leg is
  planned. Before the first cycle the server checks every axis of the last
  sample is within its `completion_tolerance` of the first; a plan that does
  not close is refused `loop_not_closed` ("plan it again with the repeat
  selected"), nothing sent.
- **One question**, before the first cycle only, even armed at the start:
  "Loop until stopped" / "Run 3 times" (prefixed "Arm, move to program start
  and …" as needed), with the planned return move's duration and each joint's
  travel (the table), or "already ends at its start: no return move".
- **Between cycles** the server stages the same bytes again through Load's
  live checks and native preparation and plays at once (rt-core retires a
  program on completion; nothing else waits). A reload or Play the machine
  refuses ends the loop with its words, never retried.
- **No jog while it runs.** Between cycles the session is `done` or `loaded`
  for moments, not `playing`: a jog begin, a move by angle or a Cartesian
  step is refused `jog_mode_conflict` for as long as the loop runs, and both
  apps' jog gate says "A program loop is running; STOP it (or Finish cycle)
  before jogging" (2026-10-12, with the Deck's jog on the Program page). A
  plan loaded and not playing never refuses a jog: Run program moves back to
  the start and loads it again.
- **Ending.** At the program start after a cycle: N cycles done, **Finish
  cycle** (`POST .../run-program/finish`), a maintenance notice (checked
  before every cycle; a notice never stops a running cycle by itself), a
  refused reload or Play. At once, by the paths that already stop the robot:
  ■/STOP from any client, the hardware E-stop, DISARM, a fault or drive
  alarm (never retried), and, for an attended loop only, the session
  heartbeat deadman and the starter's heartbeat (below).
- **Attended (the default for a cell that does not allow unattended loops).**
  The loop belongs to the client that started it: it needs that client's own
  heartbeat (another client's beat does not keep it running; 5 s of silence
  stops the robot, `ui_heartbeat_lost`), and only that client may Finish
  cycle (`loop_owner_only`); STOP works from anywhere.
- **Unattended (a cell that allows it).** Loops may run unattended, guarded
  by Stop from any client and the hardware E-stop; faults, a refused reload
  and drive alarms stop the loop, and maintenance ends it at the cycle
  boundary. The switch is the cell manifest's
  `cell_deploy.olp.unattended_loop_allowed` (`rt-core/config/cells/<cell>.json`,
  read by OLP from its own tree in `internal/targets/cells.go`, false when
  absent, so an OLP-tier release carries it and the rt-core pin does not
  move; in the machine configuration rt-core's compiler would put it in the
  deployment identity, which the runtime ignores and only a full redeploy
  may change). On such a cell **every** loop is unattended, asked for or not:
  it keeps running when its starter, or every client, disconnects, closes
  the page or goes quiet (no starter heartbeat, no session deadman while the
  loop runs; the session deadman counts again from the loop's end, and the
  browser sends no STOP on page close while an unattended loop runs); **any**
  connected client may Finish cycle; STOP from any client, the E-stop,
  faults, drive alarms and a refused reload or Play end it at once; a
  maintenance notice ends it at the next cycle boundary. The one question
  says so ("Unattended: it keeps running if this client disconnects…";
  run-program's `loop.unattended`), and status `loop.attended` is false.
  Keep the E-stop within reach. A press asking `"unattended": true` on a
  cell without the switch is refused `unattended_loop_not_allowed`.
- **Program arm (Steam Deck, owner decision 2026-11-13).** The same switch
  admits the pendant's Run-page **Arm for program**: an arm that stays
  latched with L2 released, so Once, loops and go-to-start run without the
  dead-man and losing the Deck window's focus does not stop them. The
  switch reaches the pendant as `GET .../dense-execution/capabilities`
  `unattended_loop_allowed`, and `POST .../dense-execution/arm` with
  `{"armed": true, "program": true}` is refused 409
  `program_arm_not_allowed` on a cell without it (`dense_execution.go`).
  The check is against a confused client, like the client headers: OLP
  cannot see L2, so an ordinary arm request is unaffected.
- **On screen**: every status carries `loop` {mode, count, cycles, state
  running|finishing|ended, cycle_elapsed_s, last_cycle_s, owner, attended,
  end_reason}; both apps show "Cycle 5 of 10 · last cycle 12.3 s" and, after,
  "Loop ended after 7 cycles: stopped at once by STOP".
- **Journal**: every line names the client that started the loop:
  `dense-execution: run-program loop client=<who> cycle=1 start mode=…
  attended=<bool> …`, `cycle=<n> start session=…`, `cycle=<n> end
  duration_s=<s> result=done`, `finish requested`, `end cycles=<n>
  reason=<why>`.

Machine requests use `POST .../dense-execution/cartesian/{start,intent,stop,halt}`
and `GET .../dense-execution/cartesian/state`. All POSTs carry
`X-RT-Target-Generation` and `X-RT-Target-Cell`; stale identity returns
409 `target_changed`. They also pin target, session and hold revision.
Disconnected requests retain `.../local-simulator/jog/*`. Release, cancellation,
hiding, disarm, session loss, STOP or refusal ends a hold once. Typed refusals
appear in words beside the pad. Home, set-zero and step are not machine
Cartesian operations.

A machine Cartesian **hold** is a path hold: OLP streams exact IK joint
positions of a line frozen at the held command and rt-core plays them with one
jerk-limited path speed, so the TCP stays on the line through every start, speed
change and stop. It needs the cell's `path_jog` capability (rt-core protocol
1.14 with `jog_jerk` configured); without it holds are refused with
`cartesian_hold_unavailable` and Step mode remains. Speed (the fraction) may
change during a hold; axis, frame or direction need a new hold. A resolver too
slow for the requested speed is refused at Start (`cartesian_resolver_too_slow`).
Details, limits and journal lines: [Cartesian machine jog](docs/RT_CORE_EXECUTION.md#cartesian-machine-jog);
offline acceptance: `tools/cartesianjog-harness`. The velocity hold this
replaced left the commanded line by 0.9-75 mm on a Rosie 1430 cell.
Owner rule: the TCP never leaves its commanded path; when one joint is
limited, every joint slows together.

A press that arrives while the same session's released hold is still
stopping waits for that stop and then begins (`jog/state` reports `queued`)
instead of being refused; past the stop's own bound it is refused
`jog_mode_conflict` as before. Against a server whose `jog/state` reports
`pipelined_intents`, the browser and the pendant keep the 50 ms intent
cadence while replies are outstanding (up to 5 in flight) and the server
drops an intent overtaken by a newer one. Both since 2026-10-07; details in
[Cartesian machine jog](docs/RT_CORE_EXECUTION.md#cartesian-machine-jog).

The `proportional_input` state capability opts clients into proportional
joint and Cartesian holds. Begin pins `max_fraction` (0 < ceiling <= 1);
every input carries a positive, strictly increasing `input_sequence` and
0 < `fraction` <= ceiling. The lane, axis, direction, frame and ceiling
remain unchanged. Old sequence numbers are dropped without renewing the
deadman, including while a press waits for the previous stop. Release still
retires the revision. Clients without these fields keep their previous
contracts (fixed joint fraction, variable Cartesian fraction). The native
Deck uses 20 ms intents for proportional holds, with the same five-request
bound and release priority; other clients retain their existing cadence.
The stop bound uses the proportional ceiling to cover a still-decelerating
joint even when its latest requested speed is low.

The arm pose OLP reads from a cell is rt-core's `metrics.json`, rewritten at
10 Hz and 35-100 ms behind the arm by the time OLP acts on it. It must never
steer motion: lines, Move-to and Step start from the held command
(`logical_target`), and the measured pose is used only to monitor (the hold's
2 mm off-line warning) and to display.

The server runs cartesiand's `--resolve-only REPO_ROOT` entry point (Step:
`move`; hold: `path_extend`, monitoring: `fk`). `path_extend` and `fk` exist
from `262ce393`; an older `robot-v4-cartesiand` still serves Step but
answers `path_extend` with
`cartesian_input_invalid`, so every hold is refused; rebuild it with the OLP
binary.
`OLP_CARTESIAN_RESOLVER` is the resolver command line. For a connected cell it
should run the cell's own motion server, so the kinematics stay on the device:
`env -u LD_LIBRARY_PATH ssh -o BatchMode=yes user@<cell> /path/robot-v4-cartesiand`
with `OLP_CARTESIAN_RESOLVER_REPO` naming the tree on that host (the resolver
reads `urdf/v1/<model>/robot.urdf` under it). One ssh session stays open per
connected machine and answers one request per step. The `env -u LD_LIBRARY_PATH`
matters: the launcher exports the Tesseract pixi library path, and the system
`ssh` refuses to start against that OpenSSL ("OpenSSL version mismatch", exit
255). Unset, the existing canonical local Motion Server lookup applies. This
process has no controller or motion output. See [Cartesian machine jog](docs/RT_CORE_EXECUTION.md#cartesian-machine-jog).

## Safety model

- `offline_preview` is browser-only and always labelled
  `PREVIEW · NO MOTION OUTPUT`.
- Local simulation starts with the offline-programming server by default. It
  is loopback-only and proves
  simulation mode, EtherCAT/motion output off, no hardware access, and no
  physical services.
- Missing Tesseract, the exact canonical binary, URDF/meshes, collision
  assets, or containment proof fails closed.
- Physical target planning uses the target Motion Server’s deployed planner,
  manifest, robot model, calibration, frames, configuration, limits, and
  collision world.
- Every program edit changes the program revision/digest and invalidates its
  previous plan.
- Execution requires an exact program, accepted-plan, trajectory, manifest,
  target, controller, lease, and live-output match.
- A target is eligible only when its manifest explicitly lists
  `offline-programming` in `leader_controller_roles`; targets without that role remain planning/read-only or simulation-only.
- Motion Server's `trusted-lan` leader mode is cooperative lease/fence
  enforcement, not cryptographic transport authentication. Connected
  execution must remain disabled until the deployment gate has commissioned
  secured NATS access and subject ACLs for the robot cell.
- Stale evidence, disconnect, fault, lease loss, fence change, plan change,
  weld-on read-back, or unsafe RTCore output ends the session through the
  protective tail.
- Motion Server restart or accepted-plan eviction invalidates the runtime
  reference; the program must be planned and reviewed again.

## Maintenance notice

While the cell is being updated, built or tested, the deploy tooling in
`steamdeck/real/v4` (`realtime-core release`, `build`, `config`, `deploy`,
`start`, `prune`, `lock` and the rest, and `olp install|rollback`) writes a
notice on the OLP host,
`/var/lib/rosie-olp/maintenance.json` (`OFFLINE_PROGRAMMING_MAINTENANCE_FILE`
overrides the path). The server stats it on every read and re-parses it only
when it changes. `GET /api/offline-programming/v1/maintenance` is always
answered: `active`, `reason`, `phase`, `started_at`, `eta` (RFC 3339 UTC),
`set_by`, `release`, `run_id`, the server's `now`, `overrun`, `stale`,
`malformed`, and `server.build_id`. Health reports `build_id` and
`maintenance.active` too (this host's notice only; it never contacts the cell).

While the notice is active the server answers 503 `maintenance` (with the
reason, phase, ETA and `Retry-After`) to arm, joint and Cartesian hold starts,
Move-to and Cartesian steps, Home, Go Home, fault reset, load, play, cell
selection and catalogue edits, reviewed execution start and any new
dense-execution route. Stop, Pause, Disarm (`arm` with `armed: false`),
Disconnect (`target` with an empty cell), the session heartbeat, execution
Stop, and ending or continuing (`jog/update`, `cartesian/intent`) a hold that
is already running always pass. A notice never disarms, stops or drops a
session by itself: that would be motion policy decided by a file. An armed
operator finishes, Stops and disarms; the deploy tools wait for that and then
apply their own armed/lease gates. Reads, planning, authoring and the local
simulator are not robot actions and are not refused.

Every browser polls the route every 2 s and covers the app with a blocking
"Update in progress" overlay: the reason, the current step, the minutes left
to the expected finish ("About 12 min left"; past it, in red, "Taking longer
than expected (+3 min)", never a negative clock: the ETA is an estimate a
release re-estimates as its steps finish), elapsed time, who started it and
the release. When the notice carries the deploy run's `progress` (passed
through as written, up to 16 KiB; steamdeck/real/v4 `notice_progress.go`) it
also shows "Step N/M · <step>", "Suite k/m: <suite>" during the test gate, a
progress bar weighted by estimated time (advanced between updates by the
running step's clock, at most 99 % of it) and the steps with ticks and times;
the run's step line, which it also writes as the phase for older clients, is
not repeated as "Now:". Everything under it is `inert`; presses, clicks, keys and the
wheel are stopped before the app sees them, but releases still reach it, so a
jog held when the notice appears ends on release. There is no dismiss button.
The overlay carries its own STOP while a cell is connected, because the title
bar's is underneath. If the server stops answering after a notice was seen
(the update restarting it), the overlay stays and says so. It goes away only
when a reachable server reports no notice and `/health` answers ok; if the
server's build changed meanwhile it offers **Reload now** / **Later**. The
Steam Deck v5 pendant shows the same notice from the same code
(`ui/src/execution/maintenance.ts`, bundled into its `olp-core.js`).

The overlay also lists who is connected, whoever holds the machine first:
the selected cell (armed, control lease, program state) and every client that
called this server in the last 60 s (`clients` and `machine` in the route's
answer, present with or without a notice). A client is named only from what its
requests already carry: the Steam Deck's `X-Rosie-Client` label, a browser's
User-Agent and Origin/Referer, `curl` or a Go client, otherwise "Unknown
client"; a loopback address is shown as "this host or a tunnel", because every
SSH-tunnelled client looks the same. The client whose heartbeat is under 5 s
old holds the execution session. The deploy tools print the same list while
they wait for the cell to go idle.

The servo host carries its own copy of the notice for rt-control, which refuses
new grants and motion starts with `busy` for clients that bypass this server.
This server also reads that copy from the selected cell's rt-control
(`GET /v1/maintenance`, `cell_maintenance.go`) with the cell's own socket or
client certificate, at most once per 2 s, and merges it into the route's
answer: `active` while either notice is, both reasons when they differ, the
later ETA and the earlier start, and the robot-action refusals above (a cell
selection is refused only by this host's notice). The route's `cell` object
is that read: `reachable`, `supported` (false for an rt-control older than the
route), the cell notice's own fields, and `holder`, the controller name and
principal holding rt-control's grant (never its session), which the overlay
lists under CONNECTED. While rt-control does not answer after reporting an
active notice (a full deploy restarts it), the last notice stays active with a
note; with no notice seen, an unreachable rt-control is the ordinary
disconnected state. So a Steam Deck running its own OLP server, whose host has
no notice, shows the overlay during a servo host update too. The route changed
the capabilities digest (`b1e28f06…` to `725c5bbb…`): the UI and the Deck
must be built from the commit the cell runs.

A notice that cannot be parsed is active and `malformed` (fails closed). One
whose ETA passed more than 2 h ago (`OFFLINE_PROGRAMMING_MAINTENANCE_STALE_AFTER`)
stays active and is flagged `stale`; nothing clears a notice by itself. To set
or clear one by hand on the OLP host (root: the file is under `/var/lib`):

```text
sudo <olp binary> maintenance set --reason "OLP update" --eta 20m [--phase TEXT]
sudo <olp binary> maintenance show
sudo <olp binary> maintenance clear
```

`<olp binary>` is this module's binary (`go run . maintenance ...` from a
checkout). From the Mac, `go run . realtime-core maintenance set|clear|show
--manifest <cell>` in `steamdeck/real/v4` does the same on the cell's relay.

## Another controller

rt-control grants one controller at a time (a second Acquire is refused
`control_already_owned`), but every client of one OLP server -- browser tabs,
Steam Deck pendants, scripts -- shares the single grant this server holds.
Before this check a second client's **Connect** re-selected the cell, which
Stops the selected backend: it disarmed whoever was jogging, without a word;
its Arm, jog or Move drove a robot someone else had armed (`control.go`).

The client whose control request was last admitted owns the robot. While the
robot is in use -- this server holds the cell's grant, the machine is armed,
or one of the owner's requests (Move, Home, Load) is still running -- the
server refuses 409 `in_use_by_other_controller` to every other client's arm,
joint and Cartesian hold start, Move-to and Cartesian step, Home, fault reset,
load, play, cell selection (Connect, `targets/select` and `target` with a
cell), reviewed execution start and any new dense-execution route. The answer
carries `detail` (who, from where, since when, armed or holding the session,
the running or last request) and `control`, the view below. When nothing is in
use, the next client to act becomes the owner. Stop, Pause, Disarm,
Disconnect, the session heartbeat, execution Stop and ending or continuing a
running hold always pass, from anyone: any operator can stop the robot. Stop
and Disarm release the grant, so the robot is free afterwards.

Clients tell themselves apart with an optional `X-Rosie-Client-Id` header: one
random id per browser tab (kept in `sessionStorage`, so a reload is the same
controller) and one per pendant process. A client that sends none is the
connected-clients identity (address, Origin/Referer, User-Agent, the Deck's
`X-Rosie-Client` label), so today's UI and pendant keep working against this
server, two tabs of one old browser counting as one controller.

**Take control** (`POST /api/offline-programming/v1/control/take`) is the only
handover that does not go through Stop. It succeeds when the robot is free or
already the requester's, or when the owner is disarmed, has no request running
and finished its last one at least 30 s ago (`controlTakeoverIdle`, the
owner's choice, not measured: long enough to cover Home-then-Arm). It never
succeeds against an armed owner or a running request, and it moves nothing.
An owner who leaves armed while another client's heartbeat keeps the session
alive is resolved by Stop, which any client may press.

`GET /api/offline-programming/v1/control` -- and the `control` member of the
maintenance route that every client already polls every 2 s -- is the rule as
the requester sees it: `state` `free`, `yours`, `other` or `other_server`;
`you` and `named`; `owner` (`kind`, `label`, `address`, `loopback`, `since`,
`last_action`, `last_action_at`, `running`, `idle_ms`); `armed`, `lease_held`,
`session_state`; `take_control` with `take_control_note`; `stop_available`;
`detail`. The maintenance route's `clients` entries gain `controller` and
`you`.

`other_server`: the selected cell's rt-control reports (its maintenance route,
at most 2 s old) that its grant is held by a controller name that is not this
server's -- another OLP server, a Deck running OLP locally, `rtctl`. Arm, hold
starts, Home and the rest are refused with that holder named (controller,
remote identity or local socket, since) instead of rt-control's bare
`control_already_owned`; Connect is not refused, because it only observes.
This server cannot stop that holder's motion (rt-control's Stop is fenced to
the grant), so `stop_available` is false and the clients say: use that
controller's Stop or the hardware stop. No rt-control or schema change was
needed; the capabilities digest stays `725c5bbb…`.

### Pendant identity and focus

A Deck once ran four pendant processes, two for each of two cells, all
reading the same controller; each Arm pressed on the visible pendant of one
cell also armed the other cell through a hidden one. This server's journal
said only `arm=true requested by 127.0.0.1:36054`: every Deck comes through an
SSH tunnel.

A pendant now sends on every request `X-Rosie-Client-Pid` (its process id),
`X-Rosie-Client-Expect-Cell` (the cell its launcher was started for, when it
names one) and `X-Rosie-Client-Focus` (`focused` while its window has input
focus, `background` otherwise) (`client_identity.go`). For every request the
control rule above guards (arm, hold starts, Move-to, steps, Home, fault reset,
load, play, Connect, Take control):

- from a client in the `background`: 409 `client_not_focused` ("... is in the
  background ...: a hidden pendant never arms or moves the robot. Bring it to
  the front ... Stop always works.");
- from a client whose expected cell is not the cell this server has selected,
  or, for a Connect, not the cell it asks for: 409 `client_cell_mismatch` with
  `expect_cell` and `cell`.

Stop, Pause, Disarm, Disconnect, the heartbeat and ending a hold pass from
any client, as above. A client that sends none of the headers (a browser, a
script, a pendant built before 2026-10-11) is governed by the control rule
alone: these are a check against a confused client, not authentication. The
journal names the sender everywhere it used to print a peer address:

```
dense-execution: arm=true requested by rosie-pendant-v5 (<deck>) id=deck-0b7e44d1 pid=1376204 expect_cell=<cell-a> focus=focused from 127.0.0.1:36054
control: refused POST .../dense-execution/arm from rosie-pendant-v5 (<deck>) id=... pid=949429 expect_cell=<cell-b> focus=background from 127.0.0.1:40448: client_not_focused: ...
targets: select <cell-b> requested by ...
```

and the maintenance route's `clients` entries carry `pid`, `expect_cell` and
`focus` from each client's latest request. OLP API only: no rt-control,
protocol or capabilities digest change. The pendant's own half (it drops
controller input while unfocused, one pendant per Deck, `--expect-cell`) is
the v5 README's "One pendant per Deck".

## Jog end reasons

A jog or Cartesian hold that ends without the operator releasing -- the robot
stops, stays armed (or not) and the trigger must be pulled again -- used to
leave only an internal code in the failing intent's reply and
`jog hold: end reason=<code>` in the journal (the logs must say why, and
the operator gets a small warning). Every hold end
is now classified once (`internal/denseexec/jog_end.go`) from causes OLP
already has -- its own retirement code, the native path observation
(`PathStopReason`, `PathGoverning`, samples received/consumed) and the
resolver's path-end reason -- into one operator code with the words, the
technical detail and whether pulling the trigger again can work (`repull`:
`yes`, `no` for the same direction, `after_rearm`, `unknown`). No native
field was added; the capabilities digest is unchanged.

| Code | From (OLP, SDK, native causes) | Operator text (abridged) | Re-pull | Pop-up |
|---|---|---|---|---|
| `released` | operator release | "Jog released." | yes | no (yes if another client ended it) |
| `input_deadline_expired` | OLP deadman `jog_input_deadline_expired`; native `expired` / `PathStopReason::Expired`; `jog_invalid_timing`, `jog_clock_moved_backwards`, `jog_input_too_old`, `jog_stream_idle` | "the jog commands arrived late (last one 263 ms ago, limit 250 ms)" | yes | yes |
| `path_underrun` | `path_buffer_underrun` (native `Underrun`, governing `buffer_end`) | "the path was not computed fast enough to keep up (N samples ahead)" | yes | yes |
| `playable_end` | native `PathEnd`, `path_ended`, no resolver reason | "stopped at the end of the computed line" | yes | yes |
| `singularity_gate` | resolver `jacobian_gate` | "reached a singularity (J5 = -88.3°, J3 = 40.1°); jog back the way you came, or reposition with joint jog" | no (reversing works) | yes |
| `ik_discontinuity` | resolver `ik_discontinuity`; native `StepLimit` | "this direction needs a joint to turn faster than it can (near a singularity or an arm flip …)" | no | yes |
| `command_refused` | `jog_native_rejected_<n>` (an rt-control refusal the SDK has no name for), `jog_publisher_busy`, `path_queue_full` | "the robot controller refused one jog command; the robot stayed armed. Pull the trigger again." | yes | yes |
| `joint_limit` | resolver `joint_limit`; native `PositionLimit`; `outside_limits_outward` | "J3 reached its lower limit (-120.3°); jog the other way" | no | yes |
| `workspace_limit` | resolver `cartesian_reach`, `ik_no_solution` | "this line leaves the robot's reach" | no | yes |
| `speed_governed_to_zero` | a release after native held the path at zero speed under `cap`, `velocity`, `curvature` or `jerk` for ≥ 300 ms (`jogStallReportAfter`, unverified); `jog_limited` | "Jog stalled: the speed was held at zero by the per-sample speed cap near a singularity" | no | yes |
| `session_stale` | `jog_session_stale`, `jog_wrong_identity`, `jog_stale_sequence`, `jog_closed`, `path_closed`, `control_session_stale`, `cartesian_resolver_changed`, `server_shutdown`, mid-hold `jog_mode_conflict` | "the jog session was replaced or went stale" | yes | yes |
| `transport_lost` | `rt_core_transport_lost`, `rt_core_closed`, `rt_core_request_slow`, `rt_core_failure`, `jog_transport_congested`, `jog_clock_*`, `ui_heartbeat_lost` | "the link to the robot controller dropped" | yes | yes |
| `halted` | Stop (`operator_stop`), Disarm, `selection_changed`; native public Halt | "Stop was pressed" / "<client> pressed Stop. Arm before jogging again." | after_rearm (Halt: yes) | only when another client (or selection change, or a native Halt) did it |
| `inhibited` | `jog_not_ready`, `no_grant`, `jog_authority_lost`, `jog_clock_reset`, native `Inhibited`, `rt_core_readiness_lost`, `control_renewal_lost`, `rt_core_fault` | "… and the robot disarmed: <why>. Arm again before jogging." | after_rearm | yes |
| `generator_fault` | native `path_fault` (disarms); OLP resolver/stream `cartesian_resolver_unavailable`, `cartesian_resolver_invalid`, `path_capacity`, `path_out_of_order`, `path_invalid_sample` | "the path generator faulted" / "the Cartesian path could not be computed or streamed" | after_rearm / yes | yes |
| `start_mismatch` | `path_start_mismatch` | "Jog could not start: the robot was still settling from the last stop" | yes | yes |
| `refused_at_start` | a refusal while the hold was setting up (`cartesian_pose_stale`, `cartesian_resolver_too_slow`, `cartesian_hold_unavailable`, …) | "Jog could not start (<cause>)." | unknown | yes |
| `unknown` | anything else | "Jog stopped (<cause>). Pull the trigger again; report it if it repeats." | unknown | yes |

The singularity wording gives J5 and J3 at the last streamed sample without
saying which singularity it is: the resolver reports no singular joint, and
a Rosie 1430's wrist singularity is at J5 = ±90°, not 0 (reversing out of
the jam works). `command_refused` is a hold drop-out seen on a cell:
rt-control's WSS stream refused one speed frame
(adapter reason 3, whose catch-all also covers a busy publisher) while native
stayed open and armed; the fix that stops it ending a hold is rt-control's
(full tier). The joint-limit
wording names the joint nearest one of its limits at the last streamed sample
(Cartesian) or the jogged joint (joint jog). An End that turns out uncertain,
so the Stop fence disarms the robot, revises the same end (`disarmed`, "The
robot was disarmed: arm again before jogging.").

**Journal.** The hold's one end line keeps its leading fields and adds the
evidence: `jog hold: end rev=31 kind=cartesian reason=path_buffer_underrun
duration_ms=… updates=… end_code=path_underrun seq=4 client_id=tab-…
client="Browser (…)" ended_by="" generation=7 axis=+X frame=base
requested=400.0 achieved=396.2 speed_unit=mm/s distance=512.30
distance_unit=mm governing=buffer_end samples_ahead=2 last_intent_age_ms=31
lifetime_ms=250 rt_rtt_ms=1.2 at_start=false repull=yes popup=true
disarmed=false text="…" detail="…"`. Joint jogs give the commanded and peak
measured joint speed and the measured travel in deg (10 Hz status);
`samples_ahead` and `governing` are `na`. The `cartesian path hold: end` line
is unchanged.

**API.** The status body (`GET …/dense-execution/status`, which every client
polls) and the jog state (`GET …/jog/state`, `…/cartesian/state`, as
`last_end`) carry `last_jog_end`: `seq` (one per end in this server process),
`code`, `cause`, `text`, `detail`, `repull`, `popup`, `disarmed`, `at_start`,
`kind`, `axis`, `frame`, `client_id` and `client_label` (the client whose press
began the hold: its `X-Rosie-Client-Id`), `ended_by` (another client whose
Stop, Disarm or release ended it), `revision`, `generation`, `at`, `age_ms`
(server clock, at the snapshot), `duration_ms`, the speeds and distance with
their units, `governing`, `samples_ahead`, `last_intent_age_ms`,
`lifetime_ms`, `updates`, `rt_round_trip_ms`.

**Frequent control status.** `GET …/dense-execution/status?view=control`
returns the same fresh observation, target and controller identities, current
session/loop, execution state, faults/recovery, I/O, descriptions and raw/logical
axis data. The selected backend requests native `StatusObservation` before I/O,
omitting historical `execution.handles` during ordinary polling and background
observation. Active joint moves (including return-to-zero and Cartesian steps)
still fetch full native handle receipts; the UI projects those to at most two
records, the current motion handle and newest prepared handle. A late compact
reply cannot replace an active move's retained receipt or refresh its freshness.
Fault and identity checks remain active. Completed-cycle history therefore cannot
enlarge the pendant's frequent response, or the native current-only wire response.
The ordinary route keeps the full diagnostic history, and neither view changes
native handle retention or motion admission. Backends without the optional
current-only reader retain their full-read behavior and UI projection.
Clients requesting the control view may negotiate `Accept-Encoding: gzip`;
the server uses its existing compression middleware, `Vary: Accept-Encoding` and `no-store`.
An explicit `gzip;q=0` is respected. Older servers ignore the query and return
the previous full response, so clients keep the same status-decoding contract.
This changes transfer size, not the communication-loss timeout or arm policy.

**Clients.** `ui/src/execution/jogEnd.ts` (`JogEndTracker`) decides what a
client shows; the browser (`ui/src/ui/jog-end-toast.ts`) and the Deck v5
pendant (`steamdeck/real/v5`, bundled into `olp-core.js`) share it. The client
whose id is `client_id` gets an amber toast, top right under the toolbar,
with the operator text; every other client a quiet note naming whose jog it
was. Nothing for `popup: false` (a release, the owner's own Stop or Disarm).
Each end shows once; one already older than 10 s when first seen (a reload)
does not show. It auto-dismisses after 6 s (a note after 4 s), × dismisses
it, and Details opens the technical detail (what a re-pull will do, code and
cause, the server's detail, the hold's speeds and travel, intent age and
samples ahead, whose jog it was), holding the toast open while open. It takes
no focus and has no scrim, so keyboard jog and STOP keep working. Tests:
`internal/denseexec/jog_end_test.go` (every native, SDK and OLP cause maps;
unknown keeps its code; the words; the journal line and status; Stop
attribution; the joint deadman; the cap stall), `ui/src/execution/jogEnd.test.ts`,
`ui/src/ui/jog-end-toast.test.ts`.

**End causes from clients** (on one cell, 107 of a night's 112 holds were
ended by clients and every one was logged `release`). The End body (`…/jog/end`, `…/cartesian/halt`) carries an
optional `end_cause` (lower-case `[a-z0-9_:.-]`, at most 80 characters, else
`invalid_end_cause`); the end line gains `client_cause=` and
`last_jog_end.client_cause`. A server-side cause always wins (the client's
End came after native or OLP had ended it). An older client sends none: a
release, as before.

| `end_cause` (browser, Deck) | Code | Pop-up |
|---|---|---|
| `trigger_released`, `pointer_up`, `button_released`, `stick_neutral`, `key_up`, none | `released` | no |
| `stick_direction_change` (Deck: the held stick axis let go or another axis took over by the margin, trigger still held) | `stick_direction_change`: "Release RT and centre the stick to jog a new direction." (`repull` `after_neutral`) | yes |
| `pointer_leave`, `pointer_cancel`, `pointer_capture_lost` | `pointer_left` | yes |
| `focus_lost`, `visibility_hidden`, `page_hidden` | `focus_lost` | yes |
| `controller_disconnected` | `controller_disconnected` | yes |
| `mode_switch`, `speed_change`, `frame_change` | `mode_switch` ("Jog ended: the jog mode changed.") | no |
| `ui_navigation`, `cell_change`, `app_closed`, `maintenance` | `ui_navigation` | no |
| `operator_stop`, `operator_disarm` | `halted` | no (yes if another client's) |
| `selection_changed` / `session_changed` / `state_poll_failed` | `halted` / `session_stale` / `transport_lost` | yes |
| `jog_disabled` (the client's own gate: no fresh status, not armed, no session) | `jog_disabled` | yes |
| `intent_refused:<code>`, `server_reason:<code>` | classified as `<code>` | as `<code>` |
| anything else | `client_ended` with the cause | yes |

The browser's words come from `ui/src/execution/jogEndCause.ts`; its jog
pad and joint buttons are pointer holds only (no keyboard or gamepad jog),
so the Deck's stick wobble has no browser equivalent.

**Live hold readout** (fixes A3/A5 of the same analysis: near the wrist
singularity the speed collapsed over the last millimetres and read as a
jam). During a Cartesian path hold the status body carries `jog_live`
(`internal/denseexec/path_live.go`, also `path_live` on the jog state): the
native path speed against the speed asked for, `governing` (native
PathGoverning) and `limit_kind` / `limit` (a native `cap` split into
`singularity` when the binding sample's cap is the resolver's scale x box
speed, else `joint_reserve` naming the joint), `gate_scale` (the resolver's
per-sample singularity scale at the current sample, 1 clear to 0 at its
Jacobian gate) and `min_gate_scale_ahead` over the streamed samples, `near`
(any of them < 1: inside the resolver's own slowing band, sigma_min 0.05 ->
0.02, marked unverified in `cartesian_resolver.hpp`; no threshold of OLP's),
`j5_deg` with `wrist_singular_deg` / `wrist_distance_deg` (the nearest J5 at
which J4 and J6 are collinear, computed from the robot's URDF axes in the
server's robot store: +-90 deg for the Rosie 1430 descriptions,
rosie_1400_v3 and rosie_1420_v1; absent when the store lacks the model),
`line_end` and `distance_to_end` once the resolver's end is streamed,
`escape` (the reverse of the hold's direction: reversing out of a jam works) and
`joint_hint` (J5 away from the singular angle), and `text`, e.g. "limited by:
singularity (J5 -88.3°, 1.7° from the wrist-singular -90.0°) — slowing,
stops in ~3.2 mm; back out with +X, or joint-jog +J5". After the hold ends
the last readout is kept (`active` false; "stopped near a singularity …")
until the next hold or move. Motion is unchanged. The browser's jog pad
(`ui/src/ui/jog-live.ts`) and the Deck's jog page draw it through
`ui/src/execution/jogLive.ts`: speed / requested with a bar, the sentence,
and a SINGULARITY proximity meter (1 - scale) whenever `near`. Elbow and
shoulder measures are not computed. Tests: `jog_followups_test.go`
(client causes, the End through halt, the readout near the singularity and
at a joint-reserve cap), `robot_geometry_test.go` (a Rosie 1430's +-90°
from its URDF), `jogEndCause.test.ts`, `jogLive.test.ts`, `jog-live.test.ts`.

The browser names each tab (`ui/src/execution/controlFetch.ts` wraps `fetch`
before any API client is built, so every same-origin `/api/` request carries
the id). From the maintenance poll's `control` and from any 409
`in_use_by_other_controller`, a card at the bottom of the window
(`ui/src/ui/control-popup.ts`, words from `execution/controlNotice.ts`) says
who has the robot, what it is doing and since when. It is blue and titled
ROBOT IN USE where maintenance is amber, has no scrim and blocks nothing, so
the title bar's STOP stays visible and pressable above it. **Wait** folds it
to a one-line bar (**Details** opens it) until the holder changes or another
refusal arrives; it disappears when the robot is free. **Stop robot** is the
title bar's Stop, hidden only for another server's grant, where the card names
the hardware stop instead. **Take control** appears only when the server
offers it. The Steam Deck shows the same card
([v5 README](../../steamdeck/real/v5/README.md#another-controller-using-the-robot)).
Against a server older than this check nothing is shown.

## Running

The official way to start Offline Programming V1 is the suite launcher. On
Windows + WSL Ubuntu-22.04 that is how Tesseract comes up with the program:
native Windows `go run` cannot exec the Linux planner ELF.

Windows PowerShell:

```text
.\offline-programming\v1\start-offline-programming.ps1
```

WSL (`HOME=/home/operator` for pixi):

```text
./offline-programming/v1/start-offline-programming.sh
```

Open the app at:

```text
http://127.0.0.1:5189/offline-programming/v1/ui/
```

What it starts (or reuses if already healthy):

- Vite UI on `127.0.0.1:5189`
- OLP API + local Tesseract simulator on `127.0.0.1:8794`
  (`serve --listen 127.0.0.1:8794 --enable-local-simulator` under WSL)

The launcher also ensures pixi Tesseract, `robot-v4-cartesiand`, and
`offline-programming-linux`. It probes connected-targets/daemon when
configured and prints `CONNECTED` or `no robot`. A missing robot **does not
block** local Tesseract simulation. Physical arming stays fail-closed: the
launcher never passes `--enable-connected-execution`.

There is no `rosie.sh` dispatcher in this checkout. `./serve-wsl.sh` is the
Linux API helper only (no UI, no robot probe).

On a native Linux desktop the launcher does **not** start Vite -- it only
reuses one that is already up, and prints the `:5189` URL either way. Start it
yourself in a second terminal, or the printed link refuses the connection:

```text
cd offline-programming/v1/ui && npm run dev
```

`ui/dist/index.html` is a redirect stub pointing at `:5189` on purpose, so
opening `:8794` without Vite bounces to a port with nothing on it. That is the
launcher telling you the app lives on Vite; it is not a broken build. (`dist/`
is gitignored, so a stray `npm run build` there is invisible to git and
replaces the stub until the next checkout.)

### Rails authoring needs the weld planner bridge

Rails mode -- "Define welds from edges", the seam model behind the New program
checkbox -- fails every action with

```text
Rails: the seam worker is unavailable
```

when its environment is missing. Rails authoring runs inside this process
(D24). `internal/seam` invokes
`seam_worker` as one subprocess per request out of `weld_planner/v1`, so what
can be missing is that environment:

```text
cd weld_planner/v1 && pixi install -e default
```

**The launcher still reports `api health ok` when it is missing**, so a run
with rails dead looks like a healthy one from the API alone. It prints a
`rails:` line saying which, taken from
`/api/offline-programming/v1/capabilities`.

PLANNING is a separate server and a separate origin, and the launcher does not
set it either:

```text
export OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=http://127.0.0.1:8796
./offline-programming/v1/start-offline-programming.sh
```

```text
cd weld_planner/v1 && pixi run -e motion motion-serve   # 0.0.0.0:8796 (CUDA)
```

That origin is server-only: this process and `denseexec` reach it, and no
browser ever learns it.

Check the capabilities line rather than the launcher's health line -- it
reports the seam worker and the motion origin separately, and rails needs the
first to author and the second to plan:

```text
curl -s http://127.0.0.1:8794/api/offline-programming/v1/capabilities
```

Rails authoring is **program state, chosen at New program** (a checkbox on the
dialog, persisted on the project), not a session toggle: the same seam owned by
both authoring flows would be welded twice. The toolbar toggle edits that state
under a guard and refuses to switch once the other flow has authored anything.
An older project therefore does not gain rails by upgrading the server -- start
a new program with the box ticked.

### Plan and Simulate are two buttons in rails

A classic program's **Plan & simulate** is one action because it is one
question: the browser plans, the Motion Server's spreadsheet/Tesseract planner
accepts or rejects, and what is replayed is the accepted trajectory. A rails
program's is two.

**Plan** calls the weld planner: seam search, the weld and connecting trajopt
stages, and the T2 verifier, with the dense joint trajectory brought back and
kept in this browser's store. Tens of seconds: two welds and three connecting
moves took 23 s on the workstation GPU (measured 2026-09-12). **Simulate** -- the toolbar's ▶ -- plays
those bytes, which are the bytes the daemon ingests and the arm executes.

The Motion Server is not in that path, and the Tesseract discrete contact
check went with it. Two reasons, and neither is a shortcut:

- Rails used to compile rows and hand them back to the spreadsheet planner
  "for the evidence". That planner re-solves: a `line_tcp` row's supplied
  `target_joints_rad` is discarded for J1..J6 and IK is run again from the
  interpolated TCP pose (`plan_motion_server_tesseract_spreadsheet_path`, and
  only a `joint_ptp` row keeps its joints and derives the pose by FK). So what
  was watched was a second answer that merely resembled the certified one.
  The rows that made it possible went with `export/robot_v4.py`.
- The contact check samples waypoints. Every segment in the blob has already
  passed the weld planner's T2 verifier (`verify=true`, both trajopt stages),
  which proves clearance across whole intervals with a Lipschitz bound instead
  of at the samples. That verifier's own measurement is why: 1.2-2.4 mm
  between consecutive samples on real plans, which passes clean through a 3 mm
  clamp and reports two clear endpoints.

The plan panel reports which program the loaded trajectory came from and when
it was planned, and marks it stale when the open program's `program.v2` digest
has moved. A project revision cannot answer that question -- it moves for
edits that change no motion and stands still for some that do.

### The stand is a fixture both tools read

A part on the 1420 sits on something, and the URDF carries the arm and its
pedestal only. Until 2026-09-20 the weld planner's seam worker added a
placeholder stand on its own (a 0.5 m square box 1 m tall at x 1.0, y 1.0,
unmeasured), which nothing in this app drew or could change: parts were placed
in clear air beside a body only the planner believed in, and a failed plan's
collisions could not be seen. The stand is now a **workcell fixture** on the
project (`ui/src/domain/workcell.ts`): listed under Workspace in the tree,
summarised in the inspector, drawn in the viewport at its pose in the cell's
world frame, and edited in the Stand card -- size for a box, position,
rotation, **Replace with STEP** for the real fixture's file (tessellated by
the CAD service, its bytes kept under the fingerprint so the program reopens
with it), **Cell default**, **Remove**.

Every plan request carries the list (`fixtures`, always, an empty list
included). The seam worker packs it as `fixtures.json` in the `.weldplan`,
hashed like every member, and adds no placeholder of its own when the list is
present; the planner's collision screen, cuRobo and the T2 verifier turn it
into world-frame meshes (`weld_motion_planner/robot_cell/collision/fixtures.py`).
The default for the 1420 is the old placeholder, so an untouched program plans
as before; the 1400 has its table in the URDF and defaults to none. The real
stand is still to be measured on the bench.

### Native deployment target

A planned rails program carries an immutable
`robot.v4.dense-joint-trajectory.v1` blob from the weld planner. The native
deployment target fetches and validates those original bytes, prepares them
through the public rt-core SDK, then starts the returned identity explicitly.
It observes the matching native generation and terminal state; NATS publication
or a local viewer cursor cannot establish completion.

Select `rt_core` with an explicit configuration or cell catalogue as described
above. Arm, Home, Stop and grant release use the public API. Pause, saved-pose
Go Home and physical weld I/O remain unavailable on this route. Retired dense
TCP/NATS execution flags are not an alternative runtime selection.

`dev-stack.sh` runs the native simulator, public adapter and dense daemon.
See [RT_CORE_EXECUTION.md](docs/RT_CORE_EXECUTION.md) for the
current target binding, authority and refusal contracts.

To deploy an OLP build to a cell, run from `steamdeck/real/v4`: `go run .
realtime-core build --manifest <cell> --test` builds a tree on the cell's
relay, and `go run . olp install --manifest <cell> --deployment <commit>`
repoints the cell's OLP unit at it (`olp rollback`, `olp status`):
[Tier 2: OLP-only](../../steamdeck/real/v4/README.md#tier-2-olp-only).

To run this UI on a Mac against a cell's OLP, do not hand-build a tunnel and
Vite: from `steamdeck/real/v4`, `go run . olp ui start --manifest
rt-core/config/cells/<cell>.json` starts a self-healing ssh tunnel to the
cell's `cell_deploy.olp.listen` (`127.0.0.1:18894`) and Vite on `:5189` with
`OFFLINE_PROGRAMMING_API_URL` on the tunnel, both as launchd agents
(`status`, `logs`, `restart`, `stop`;
[workflow guide](../../docs/DEPLOY_AND_TEST_WORKFLOW.md#the-browser-ui-against-a-cell-olp-ui)).

### Test programs: a tool-generated `.rdt` through Load and Play

A dense joint program generated by a tool rather than the planner -- today
`rt-core/tools/tuningreport inertia-plan`, the C00.06 inertia test
([its README](../../rt-core/tools/tuningreport/README.md#inertia-test-c0006-inertia-plan-and-inertia))
-- runs through this server's ordinary dense Load and Play, so this server
keeps the grant, the 5 s heartbeat deadman, STOP, Disarm and the control and
maintenance gates exactly as for a planner trajectory. `rtctl program-start`
would hold a second grant beside this server, where its STOP cannot reach;
do not use it for this.

**Off unless the server is started with `OFFLINE_PROGRAMMING_TEST_PROGRAMS=1`**
(`serve --enable-test-programs`). On a cell, the manifest's
`cell_deploy.olp.test_programs: true` makes `olp install` write that line
into the OLP unit. Off, the upload answers 403 `test_programs_disabled`, a
Load naming the source is refused, the dense capabilities answer is
unchanged and the Machine panel shows nothing.

Operator steps, connected to the cell and with Home valid:

1. Arm (the heartbeat starts with the session, as for any program).
2. Machine panel, Run row: **Load test program (.rdt)…**. Choose the `.rdt`
   and its `plan.json` together (the plan is optional; without it the
   program's provenance reads "unknown").
3. The server checks, then loads; a refusal names the reason. On success the
   row reads `TEST PROGRAM, not from the planner · <file> · J4 only, 24.4 s,
   200/400 deg/s², <window> · trajectory <digest> · file <sha256> ·
   configuration <pin> · <generator>, generated <time> · loaded`, and every
   client's status carries `session.source: "test_program"` with the file.
4. **▶ Play test program** (enabled only while that session is loaded, the
   robot armed and ready, and every joint within its `completion_tolerance`
   of the program's first sample). STOP and Disarm stay where they are.
   The program is single-use like any native program: Load it again to
   replay. The planner's own Load and Play are unchanged and do not play it.

What the server checks, all before any authority is acquired or byte sent:

- Upload (`POST .../dense-execution/test-program` `{file_name,
  program_base64, plan_json?, plan_file_name?}`, at most 16 MiB of
  program): the format validator and digest; a header `robot_cell` naming
  the robot and `machine_configuration_sha256` it was checked against; no
  torch bit or process marker; a `plan.json` must name exactly this
  trajectory, plan, program and program digest. Then the selected cell's
  checks, as Load runs them: Home valid, not while any program is playing,
  robot model, description and planning calibration equal to the cell's,
  machine configuration equal to the cell's (a planner plan only gets a note
  here: the tool took its limits, gears and signs from that configuration),
  `manifest_revision` equal to the cell's control binding revision, the
  program's axes equal to the cell's, and every axis's measured position,
  from valid feedback, within that axis's own `completion_tolerance` of the
  first sample. The answer is `{program, checks}`; a refused upload is not
  kept. The upload is a robot route: refused 409
  `in_use_by_other_controller` while another client has the robot and 503
  `maintenance` under a notice, like Load.
- Load (`POST .../load` with `source: "test_program"` and the program's
  identity): the same checks again (the arm may have moved), then native
  preparation's limits, Hermite admission and identity, exactly as for a
  planner artifact. A planner Load (no `source`) never reads uploads.
- `GET .../dense-execution/test-program` lists the uploads (newest four,
  kept in memory until the server restarts).

Evidence (2026-10-09, `dense_test_program_composed_test.go`, Docker on the
operator Mac, `RTCORE_COMPOSED=1`): a J4 program generated by
`tuningreport inertia-plan --compiled` for a Rosie 1430 cell's simulation
configuration, through this HTTP handler, the picker, rt-control and the
native simulator: upload with all four checks, Load, Play to completion
(8.9 s program in 8.9-9.0 s; J4 swept -15.900..15.900 deg, the planned
window; no other joint left its tolerance); a second run Stopped 3 deg in
(native off 151-373 ms after the request); the upload then refused
`test_program_start_mismatch` naming J4; a third run ended by the heartbeat
deadman 4.5-5.0 s after the beats stopped. It passed every phase 4 times;
at Mac load averages of 14-26 beside other agents' containers other runs
failed on simulator timing (`cycle_deadline`, grant renewal lost, a J4
`coordinate_reference` fault while moving), with the simulator's lateness
allowance raised to 200 ms for the container. Re-run it on a quiet Linux
host before relying on its timing. Not yet run on a cell.

### Ports, all of them

| Port | Process | Started by |
|---|---|---|
| `5189` | Vite UI -- **the app**, and the origin its IndexedDB project store lives on | you (`npm run dev`), or `olp ui start` against a cell |
| `18894` | the Mac's tunnel to a cell's OLP API | `olp ui start` (`steamdeck/real/v4`) |
| `8794` | OLP API + local Tesseract simulator + the seam worker | the launcher |
| `8796` | weld planner motion (`weld_motion_planner.server.motion_planner_server`) | you (`pixi run -e motion motion-serve`) |
| ephemeral | NATS, jog `cartesiand` UDP (`--udp-listen 127.0.0.1:0`) | the launcher, fresh each run |
| `8797` | `joint_trajectory_daemon` TCP ingest (`motion-server/joint-trajectory/v1`) | `dev-stack.sh` (daemon) |
| `14222` | nats-server for the robot cell (monitor `18222`) | `dev-stack.sh` (nats) |
| Unix sockets under the selected runtime directory | native simulator and public control/jog | `dev-stack.sh` (`rt-sim`, `rt-control`) |

The project store is per ORIGIN. Saved programs authored on `:5189` are
invisible from `:8794` and clearing site data on `:5189` destroys them.

### Verify

Proved 2026-08-21 after `start-offline-programming.ps1` (reused healthy
Vite `:5189` and WSL serve `:8794`). Expect HTTP 200 and JSON like:

`GET http://127.0.0.1:8794/api/offline-programming/v1/health`

```json
{"ok":true,"local_ready":true,"execution_enabled":false,"connected":false,
 "local_simulator":{"available":true,"code":"available"}}
```

`GET http://127.0.0.1:8794/api/offline-programming/v1/local-simulator/status`

```json
{"status":{"phase":"ready",
  "availability":{"available":true,"code":"available"},
  "readiness":{"ready":true}}}
```

`GET http://127.0.0.1:5189/offline-programming/v1/ui/` → 200, title
`Offline Programming V1`.

Vite proxy (what the UI uses):
`GET http://127.0.0.1:5189/api/offline-programming/v1/local-simulator/status`
→ same `available` / `phase: ready` JSON as the API origin.

WSL loopback often cannot see the Windows Vite port; the browser URL is still
the Windows address above.

Package commands from `offline-programming/v1`:

```text
go run . install
go run . lint
go run . test
go run . build
go run . ui --host 127.0.0.1 --port 5189
```

The fast default test command uses controlled doubles at process and network
boundaries. The explicit live acceptance test invokes the production Robot V4
Motion Server binary and genuine Tesseract collision worker for both readiness
and a collision-clear shared program:

```text
go run . test --filter live
```

Local canonical simulation is enabled by the suite launcher and by `serve`
(default). The explicit flag remains accepted:

```text
go run . serve --listen 127.0.0.1:8794 --enable-local-simulator
```

That `go run . serve` path on native Windows still cannot exec the Linux
planner; use the WSL entry point above for Tesseract.

Connected planning (does not arm the robot):

```text
go run . serve \
  --listen 127.0.0.1:8794 \
  --program-catalog-origin http://127.0.0.1:8787 \
  --enable-connected-planning \
  --daemon-state-url http://127.0.0.1:8788/api/state
```

Reviewed connected execution additionally requires an absolute durable-state
directory and is **not** started by the suite launcher:

```text
go run . serve \
  --listen 127.0.0.1:8794 \
  --program-catalog-origin http://127.0.0.1:8787 \
  --enable-connected-planning \
  --enable-connected-execution \
  --daemon-state-url http://127.0.0.1:8788/api/state \
  --execution-state-dir C:/ProgramData/Gradient/offline-programming
```

The production PWA (after `go run . build`) is also at:

```text
http://127.0.0.1:8794/offline-programming/v1/ui/
```

## Browser-safe API

```text
GET  /api/offline-programming/v1/health
GET  /api/offline-programming/v1/capabilities
GET  /api/offline-programming/v1/maintenance
GET  /api/offline-programming/v1/control
POST /api/offline-programming/v1/control/take

GET  /api/offline-programming/v1/programs
POST /api/offline-programming/v1/programs
GET  /api/offline-programming/v1/programs/{program_id}
PUT  /api/offline-programming/v1/programs/{program_id}
GET  /api/offline-programming/v1/programs/{program_id}/revisions

GET  /api/offline-programming/v1/cell-programs
GET  /api/offline-programming/v1/cell-programs/changes
POST /api/offline-programming/v1/cell-programs/current
GET  /api/offline-programming/v1/cell-programs/programs/{id}
GET  /api/offline-programming/v1/cell-programs/programs/{id}/revisions
POST /api/offline-programming/v1/cell-programs/programs/{id}/save
POST /api/offline-programming/v1/cell-programs/programs/{id}/delete

GET  /api/offline-programming/v1/local-simulator/status
POST /api/offline-programming/v1/local-simulator/start
POST /api/offline-programming/v1/local-simulator/stop
POST /api/offline-programming/v1/local-simulator/plans

GET  /api/offline-programming/v1/connected-targets
GET  /api/offline-programming/v1/connected-targets/{profile}/status
GET  /api/offline-programming/v1/connected-targets/{profile}/capabilities
POST /api/offline-programming/v1/targets/{profile}/plans
GET  /api/offline-programming/v1/plans/{plan_id}

GET  /api/offline-programming/v1/execution-gateway/capabilities
POST /api/offline-programming/v1/execution/review
POST /api/offline-programming/v1/execution/start
GET  /api/offline-programming/v1/execution/{execution_id}
POST /api/offline-programming/v1/execution/{execution_id}/stop
```

Mutation routes require bounded request/correlation/idempotency identities.
Legacy `/connect`, `/execute`, `/teleop`, and unversioned physical-plan routes
remain disabled.

## Ownership

- `steamdeck/real/v4/contracts/programs`: shared schemas, canonicalization rules,
  and cross-runtime fixtures.
- `internal/catalog`: Motion Server catalog proxy and optimistic-concurrency
  boundary.
- `internal/programstore`, `cell_programs.go`: the cell's program store
  ("Cell program store"); `ui/src/storage/cell-sync.ts` is the clients' side.
- `internal/planning`: normalized plan and immutable accepted-plan contracts.
- `internal/simulator`: contained production-planner local simulation.
- `internal/targetplanning`: server-only saved-program-to-target-plan
  promotion and accepted-plan cache.
- `internal/connected`, `connected_registry.go`, and
  `connected_monitor.go`: manifest-bound observation, read-back fusion, and
  server evidence verification.
- `internal/execution`: reviewed sessions, NATS controller, lease/fence
  authority, supervision, protective actions, audit, and idempotency.
- `ui/src/domain` and `ui/src/contracts`: project history and complete shared
  spreadsheet program.
- `ui/src/programs`: same-origin catalog client.
- `ui/src/rails/authoring.ts`: rails weld authoring without a window (the
  angle search both ways, the clearance check, a blocked seam's runs, the
  accepted weld node), called by rails mode (`rails/index.ts`) and by the
  RosieOS MCP server (`mcp/v1`), so a weld accepted in either is the same node.
- `ui/src/execution`: reviewed server workflow only; no raw command sender.
- `steamdeck/real/v5`: the Deck's teach pendant, editing the same programs through OLP's own logic.
- `motion-server/v1`: catalog, production planner, accepted-plan store,
  preload/play authority, and RTCore motion source.
- `rt-core/adapters/rosie/control` and `rt-core/engine`: public session admission
  and native grant/generation enforcement.

The full design, delivery evidence, recovery rules, and acceptance matrix are
in [docs/FULL_ARCHITECTURE_PLAN.md](docs/FULL_ARCHITECTURE_PLAN.md).

Taught moves also support inline spreadsheet edits: select the row, then click
its move type, TCP XYZ/RPY, joint-limit percentage or TCP speed (mm/s). Enter or
leaving the field saves; Escape cancels. Edit still opens the full dialog.
Cartesian edits set `target_space: cartesian`; MoveJ resolves that destination
with the existing IK and retains a joint-space path. Recorded joint targets
remain the default for older points. `speed_mm_s` caps TCP translation for all
three move types, alongside the joint velocity and acceleration limits; it is not a
guarantee of constant TCP speed. MoveC may be authored before recording its via
pose, but planning rejects it until that pose exists. Every edit invalidates the
old plan and retains the original captured observation.

Double-clicking a taught move opens **Motion profile** with TCP speed (mm/s)
and acceleration/deceleration (mm/s²) first. These translation profiles use
the planned samples and the viewer's kinematics on a private robot copy.
Joint velocity and sample-derived acceleration graphs follow further down,
with joint visibility controls and peak values. Joint speed and acceleration limits
are independently editable percentages of the robot's limits, alongside TCP
speed in mm/s. **Save & plan** updates the trajectory and reopens its profile;
an older graph is marked until replanning. Both joint limits default to 100%
for new moves. Older moves without `acceleration_scale` retain their existing
acceleration (inherited from `speed_scale`) until explicitly edited. Recovery,
arming and execution are unchanged; planning does not move the robot.
MoveL and MoveC default to **Constant TCP speed**: a shared path clock ramps
up, cruises at the requested mm/s, then ramps to rest. Short moves use a
triangular profile; joint velocity, acceleration and curvature limits can
reduce the achievable cruise speed. The profile graph shows the resulting
speed. Clearing the checkbox uses the previous smooth speed variation.
MoveJ timing is unchanged. Existing stored trajectories remain unchanged;
replanning applies the selected mode. Pure orientation moves retain smooth
timing because their translational TCP speed is zero.
Authoring and replanning may rewind the browser preview, but cannot overwrite
the live feedback pose. A changed program requires a new plan and explicit
load; moving to its start remains an explicit machine action. Loaded, playing
and paused executions lock authoring until released with Stop.

The spreadsheet's **Plan speed** sets 1–100% of the weld planner's complete
motion trajectory, including taught moves and connecting moves. Set it before
planning, then replan and reload. At 10%, a planned 100 mm/s becomes 10 mm/s:
all axes share one stretched clock, with the same path, reduced derivatives,
and unchanged controller sampling period. Dwell times remain in seconds.
The setting is saved with the program and participates in its digest. It is
separate from the preview playback rate and the per-move joint limit, which
defaults to 100% for new waypoints. Existing saved moves retain their limits.
