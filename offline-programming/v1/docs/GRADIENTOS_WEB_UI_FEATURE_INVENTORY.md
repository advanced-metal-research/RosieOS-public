# GradientOS Web UI — Complete Feature and Functionality Inventory

Status: source-grounded inventory for Offline Programming planning
Inspected: 2026-07-31
Source: `C:\Users\operator\Desktop\projects\GradientOS\web-ui`
Source baseline commit: `d47bc53b40ebd31f01b53526148c554580f2df04`, plus the current modified `web-ui` working tree
Target context: `offline-programming/v1`

## 1. Purpose and scope

This document inventories the functionality implemented in the current GradientOS React web UI so that useful ideas can be evaluated for Offline Programming. It is based on the source code, not just the shorter `web-ui/README.md`.

The inspected UI contains roughly 24,800 lines of TypeScript/TSX application code. The review covered:

- the main operator application and its alternate HTML entry point;
- the application shell, navigation, layout, and persisted view state;
- the Three.js robot/CAD visualizer;
- STEP import and topology handling;
- trajectory and weld authoring, planning, persistence, and execution;
- the Program Tree and bottom timeline;
- robot, drive, jog, gripper, and commissioning controls;
- runtime, robot, tool, and kinematics settings;
- live telemetry, alerts, charts, diagnostics, and lag instrumentation;
- the archived J6 manual-rotation dataset viewer;
- backend API and SSE contracts used by the browser;
- build-time robot/tool asset synchronization; and
- source-level limitations, dormant state, and documentation mismatches.

This is an inventory of the current implementation, not an endorsement of moving every control into Offline Programming. Offline Programming's existing authority model deliberately keeps raw jog, drive enable, weld output, and direct RTCore/EtherCAT control out of the browser. Section 24 classifies the GradientOS features accordingly.

### 1.1 Status terminology

- **Implemented** means the source contains a wired UI and behavior.
- **Backend-dependent** means the browser feature is implemented but requires a specific Gradient API/controller capability or telemetry field.
- **Conditional** means it appears only for a particular runtime, device, setting, or data shape.
- **Dormant** means state or code exists but is not currently presented or consumed by the main rendered application.
- **Mismatch** means visible copy, README documentation, or asset discovery promises more or different behavior than the current execution path provides.

## 2. Application map

The build has two browser entry points.

| Entry point | Purpose | Runtime data |
|---|---|---|
| `index.html` / `src/main.tsx` | Main GradientOS operator, programming, visualization, telemetry, and control application | Live Gradient API, controller, vision stream, local browser files, synced static assets |
| `j6-manual-rotate-dataset.html` / `src/j6-manual-rotate-dataset.tsx` | Archived J6 encoder/manual-rotation analysis page | Static dataset embedded into the TSX bundle; no live API loading |

The main application is not route-based. It is a single, dense workstation shell whose left drawer changes between five workspaces:

1. STEP Import
2. Trajectory
3. Tool Library
4. Weld
5. Live Charts

The center stage, right-side Program Tree and Robot Control docks, bottom timeline, runtime header, and alerts remain shared across those workspaces.

## 3. Main application shell

### 3.1 Header and global status

The header provides a compact operator overview and global actions:

- Gradient Robotics / Control Center identity;
- explicit `LIVE` and `SIM` runtime mode buttons;
- a compact drive status, motion status, safety status, and active fault count supplied by `ControlPanelRuntimeHeader`;
- quick RTCore drive actions where supported: Power Up, Power Down, and Reset All Faults;
- streaming/disconnected state;
- separate API-offline or controller-telemetry-stale badges;
- connect/disconnect;
- robot Home;
- robot Rest;
- a prominent STOP action;
- 3D/vision view toggle;
- reset 3D camera/view;
- show/hide Program Tree; and
- Settings.

The application attempts to connect to the monitor stream once automatically after mounting. Manual connection controls remain available afterward.

The header displays a single-line alert strip for application and vision errors. Longer text is truncated visually but preserved in a tooltip.

### 3.2 LIVE/SIM hot switching

The active controller runtime can be switched in place:

- the operator chooses LIVE or SIM;
- a confirmation explains that active motion will be stopped and the controller will wait for idle;
- `POST /control/runtime-mode` is called with `mode: "live"` or `mode: "simulate"`;
- the controller is expected to swap backends without a process restart; and
- the returned runtime configuration refreshes the displayed active/desired state.

This hot-switch path is distinct from restart-bound runtime policy changes such as robot, backend, profile, tool, or other controller configuration changes.

### 3.3 Resizable workstation layout

The shell is divided into four working regions:

- a left navigation rail and authoring drawer;
- a central stage;
- a right dock split between Program Tree and Robot Control; and
- a bottom timeline.

Pointer-drag resize handles allow the operator to change:

- left authoring pane width;
- right dock width;
- bottom timeline height; and
- the vertical allocation between Program Tree and Robot Control.

The sizes are clamped to usable ranges and to the current viewport. Current defaults and main limits are:

| Region | Default | Limits/constraints |
|---|---:|---|
| Left pane | 368 px | 220–640 px, further constrained by viewport |
| Right pane | 384 px | 220–560 px, further constrained by viewport |
| Timeline | 188 px | 72–520 px, further constrained by viewport |
| Program Tree height | 320 px | minimum 180 px; must leave at least 220 px for Robot Control |

The authoring drawers and settings modal use internal scrolling rather than extending beyond the viewport. The design follows a dark industrial style with dense information hierarchy, cyan/blue authoring accents, orange weld accents, and amber/rose warning states.

The global stylesheet enforces dark color-scheme rendering, square corners, no box shadows, and no backdrop blur even when a component's utility classes request those effects. Slate borders are made more visible globally, and dense panels use a custom thin cyan/slate scrollbar. These are application-wide visual-system rules rather than one-off component styling.

### 3.4 Left rail and drawer behavior

Selecting a rail item opens its drawer. Selecting the active item again closes it and exposes an “Authoring Surface” landing card with buttons for every workspace.

Keyboard shortcuts work unless focus is in an input, textarea, select, or content-editable element, or a Ctrl/Alt/Meta modifier is held:

| Key | Action |
|---|---|
| `1` | Toggle STEP Import |
| `2` | Toggle Trajectory |
| `3` | Toggle Weld |
| `4` | Toggle Live Charts |
| `5` | Toggle Tool Library |
| `T` | Toggle Program Tree |

The Tool Library rail item does not display its `5` shortcut even though the shortcut is implemented.

### 3.5 Shared central-stage overlays

The center stage has persistent context overlays:

- selected program-node label or “Robot Workspace”;
- active authoring panel;
- LIVE/SIM/UNKNOWN runtime badge;
- current program name when meaningful;
- context-sensitive guidance for trajectory point picking, weld edge selection, vision mode, or ordinary inspection;
- controller and application alerts;
- live joint feedback unavailable warning; and
- the 3D lag probe when the visualizer is loaded.

Selecting a Program Tree node or timeline block can automatically focus the camera on the relevant waypoints or path segment.

## 4. Browser persistence and session state

### 4.1 Persisted main settings

The application stores one JSON object under local-storage key `gradient-ui:settings`. It persists:

- show robot bounding box;
- show gripper controls;
- show the software-zero commissioning action;
- collapsed Robot Control state;
- active/closed authoring drawer;
- Program Tree visible/hidden state;
- Program Tree chronological/grouped mode;
- expanded Program Tree node IDs;
- selected Program Tree node ID;
- left and right pane widths;
- timeline height; and
- Program Tree/Robot Control split height.

It also reads and writes four legacy collapse flags—Live Charts, STEP Import, Trajectory, and Weld—but the current drawer rendering does not consume those flags. They are dormant persisted state.

Malformed or unavailable browser storage is handled on a best-effort basis and falls back to defaults.

### 4.2 Persisted telemetry panel visibility

Live-chart panel visibility is separately stored under `gradient.telemetry.panelVisibility.v1`. Each chart or diagnostic block can be hidden, and a “Restore Hidden Panels” action restores defaults.

### 4.3 State that is not persisted

Notable session-only values include:

- API host and vision host;
- whether the heavy 3D visualizer has been loaded;
- current connection state;
- current LIVE/SIM selection while the backend is unavailable;
- selected tool filters and tool draft;
- STEP file and STEP transform;
- current trajectory/weld drafts and unsaved edit history;
- timeline synthetic selection;
- diagnostic pose history; and
- J6 dataset preset, selected series, and sample index.

Saved trajectory and weld programs live behind the Gradient API rather than in browser storage.

## 5. Connection, live state, and resilience

### 5.1 Monitor stream

The UI uses `EventSource` against `GET /monitor`. It tracks open, message, and error states and exposes connect/disconnect in the header.

On a stream error it:

- reports a connection/CORS failure;
- closes the EventSource;
- marks the UI disconnected; and
- clears live programming/runtime state such as current previews, loaded program lists, topology, and weld draft.

This disconnect behavior is broad: a stream disconnect removes much of the current transient authoring state rather than merely freezing the last monitor values.

### 5.2 Parsed monitor data

The client accepts and normalizes:

- controller/source timestamp `t`;
- API receive timestamp and sequence from `monitor_timing`;
- raw joint positions;
- operator/display joint positions;
- gripper value;
- per-servo electrical and status data;
- alerts;
- aggregate drive-fault snapshot;
- per-axis absolute feedback used to synthesize a drive snapshot when the aggregate is absent;
- weld-active flag and weld type;
- controller communications summary; and
- motion/program execution status.

Per-servo data can include voltage, current, temperature, duty/load, position counts, raw torque, statusword, error code, drive mode, DS402 state, digital inputs, fault flags, brake state, logical-joint mapping, axis index, and named status labels.

### 5.3 Pose truth selection

Two pose preferences are deliberately different:

- operator-facing charts and readouts prefer `display_joints`, then fall back to `joints`;
- physical 3D tracking prefers `joints`, then falls back to `display_joints`.

This prevents a display-normalized representation from replacing the raw physical stream in the visualizer while still showing the most useful operator values in charts.

### 5.4 Ordering and spike rejection

To prevent visual snap-backs, the monitor path:

- rejects packets whose source time is not newer than the last accepted source time; and
- rejects a sample if it arrives within 0.25 seconds of the prior sample and any joint jumps more than 0.8 radian.

These guards affect both the 3D handoff and the React status state.

### 5.5 Performance split between 3D and React

Accepted joint samples are sent directly and without React throttling to the visualizer's imperative API. The visualizer applies the newest pose in its animation frame.

The broader React tree—text, charts, panels, and status—is limited to approximately 30 Hz by a 33 ms minimum publish interval. Alert-bearing events always bypass that throttle so safety information remains visible.

### 5.6 Freshness and fallback reads

The UI updates its freshness clock every 250 ms. A monitor sample is considered fresh for 250 ms. When the monitor is disconnected or stale, Robot Control polls detailed joint feedback every 100 ms while the document is visible.

REST feedback is not allowed to overwrite a monitor pose that is still considered fresh. Fallback values are converted from degrees to radians for the shared live state and visualizer.

### 5.7 Drive-fault snapshot recovery

If a monitor event omits the aggregate `drive_faults` block but contains per-axis absolute feedback and servo status, the browser synthesizes a minimum drive snapshot. It derives:

- active/disarmed physical and driver state;
- enable mask and requested/enabled axis counts;
- axis count and fault count;
- DS402 state, statusword, and error code; and
- native/canonical truth and home-verification fields.

The synthesized snapshot is merged with the prior richer snapshot. Live power/gating fields replace stale values while metadata only available in the richer snapshot is retained.

## 6. Alerts and operator error presentation

Monitor alerts are appended to an in-memory list capped at the last 20. The stage renders up to four at once. Individual alerts can be dismissed.

Alerts support:

- error, warning, and informational levels;
- kind and message;
- optional servo IDs;
- optional timestamp; and
- arbitrary details.

Joint-limit alerts receive enhanced presentation with joint labels/servo IDs. Application request failures and vision failures also appear in the header alert strip.

The UI distinguishes:

- API unreachable;
- monitor connected but controller telemetry stale;
- camera stream unavailable;
- planning failures, including structured planner diagnostics; and
- action-specific failures such as save, load, run, power, home, rest, or commissioning failure.

## 7. 3D robot and workcell visualizer

### 7.1 Loading and device-performance behavior

The visualizer is lazy-imported and initially disabled. The rest of the control UI loads first with a “Lightweight startup” card. The user explicitly clicks “Load 3D Workspace” or chooses 3D Stage.

This allows jog, telemetry, settings, and Robot Control to remain usable on resource-constrained devices before WebGL, URDF, CAD, and mesh processing are loaded.

### 7.2 Scene composition

The visualizer uses Three.js directly with OrbitControls and includes:

- Z-up world convention;
- a large XY floor grid;
- world XYZ axes;
- ambient, directional, and hemisphere lighting;
- orbit damping;
- responsive sizing through `ResizeObserver`;
- device pixel ratio capped at 2;
- camera reset;
- camera focus on selected path/waypoint sets; and
- a corner orientation widget with X/Y/Z labels.

The scene grounds the robot from its base geometry and derives reasonable world bounds and a default camera target.

### 7.3 Robot asset resolution

At runtime the visualizer reads `/assets/robots/index.json`, selects the requested robot ID, and falls back to the catalog default if the ID is unknown. The currently synced catalog includes:

- `gradient-05` as the default; and
- `mini-6dof-arm`.

Each robot is loaded from a canonical `robot.urdf` plus referenced STL meshes. The code also has development/public-directory fallback URL resolution to tolerate different Vite serving paths.

### 7.4 Robot pose updates

Live joint values are written directly to the loaded URDF robot. The current implementation intentionally does not interpolate between samples; it tracks accepted telemetry 1:1 to avoid visible lag during fast jogs.

When no fresh physical joint pose exists, the stage warns that its pose may be stale.

### 7.5 Bounding box/workspace walls

An optional bounding-box setting shows translucent work-volume walls, edges, and corner markers. When joint positions change, bounds can be recomputed on a throttled cadence (approximately every 200 ms).

### 7.6 Selection and picking

The visualizer supports two authoring pick modes:

- Shift-click on the ground plane to add trajectory points; and
- raycast hover/click on topology-edge pick tubes for weld selection.

Hover, selected, active, and highlighted topology edges use different colors/line weights.

### 7.7 Path and program overlays

The stage can draw:

- full preview path in yellow;
- highlighted selected path range in orange;
- small path sample markers;
- control-point markers;
- `CP#` and `M#` labels;
- selected waypoint highlighting; and
- camera focus corresponding to Program Tree/timeline selection.

### 7.8 Weld overlays

Weld visualization includes:

- selected edge geometry;
- start marker in green;
- stop marker in red;
- orange weld segment;
- weld-active glow/indicator;
- work/travel/spin angle guide;
- travel direction in cyan;
- up direction in blue;
- surface normal in green;
- torch direction in orange;
- angle arcs;
- optional tool-tip/end-effector frame; and
- a translucent cyan ghost robot from `weld_preview_joint_pose`.

### 7.9 Active tool visualization

The active tool is attached to the J6/flange anchor. The visualizer maps the solver's tool-axis convention to the actual URDF joint axis and separates:

- TCP/tool-tip transform used by kinematics; and
- mesh position/rotation used only to place geometry relative to the flange.

STL tool meshes are loaded, uniformly scaled, positioned in millimetres, and rotated in degrees. If no mesh is configured, loading fails, or the configured file is not STL, an orange/yellow fallback tool marker is displayed.

### 7.10 STEP model rendering

STEP geometry is parsed in the browser with `occt-import-js` and its WASM module. The visualizer:

- accepts `.step` and `.stp` files;
- generates Three.js meshes from local STEP data;
- treats source coordinates as metres;
- recentres the part in X/Y;
- moves its minimum Z to the ground plane;
- applies operator position, rotation, and scale;
- shows a local axis guide; and
- transforms backend topology edges through the same model transform.

Because local STEP parsing lives inside the lazy visualizer, the backend topology can load even while 3D is still paused, but local mesh/status rendering requires the 3D workspace to be loaded.

## 8. Vision workspace

The central stage can be replaced with an MJPEG image stream at:

`{visionHost}/stream.mjpg`

The default host uses the current browser hostname on port 8080. The operator can edit the vision host in Settings for the current session.

Vision behavior includes:

- full-stage object-contain image rendering;
- camera/3D toggle in the header and stage overlay;
- load/error detection;
- “Retry Camera Feed”; and
- direct fallback to “Show 3D Workspace.”

The vision view is display-only. There are no camera controls, overlays, recording, detection results, or vision-driven authoring actions in this UI.

## 9. STEP Import workspace

### 9.1 File handling

The STEP drawer provides:

- browser file picker limited to STEP/STP;
- selected filename and load status;
- clear file; and
- coordinate-frame guidance.

Changing or clearing the STEP file resets topology, weld selection, weld draft, editable weld waypoints, weld ghost pose, cache-readiness state, and any pending saved-program restore.

### 9.2 Transform controls

Operators can edit:

- world position X/Y/Z in metres;
- rotation X/Y/Z in degrees; and
- uniform scale, clamped to a minimum of 0.01.

A reset action returns to the default transform.

### 9.3 Backend topology import

After a file is selected, the browser base64-encodes it and calls `POST /cad/topology/load-step` with filename, STEP data, and `sample_count: 64`.

The response supplies a topology model ID and sampled edges used for weld selection and planning. Topology import and the browser-local mesh parse are separate paths.

### 9.4 Saved weld restore

Saved weld programs embed the original STEP payload and transform. On load the browser reconstructs a `File`, reloads topology, then:

- keeps only saved edge IDs that still exist;
- falls back to the first available edge if none survive;
- clamps saved segment start/end parameters;
- restores the active segment; and
- restores editable waypoints and a saved preview when present.

## 10. Trajectory authoring workspace

### 10.1 Editing workflow

The trajectory drawer has an explicit Start/Stop Editing mode. While editing and not waiting on a plan:

- Shift-clicking the ground plane adds a linear Cartesian control point;
- “Linear Move” captures the live TCP pose as a linear point;
- “Joint Move” captures the live TCP pose but marks the move as joint-space;
- “Move to Home” adds a home instruction;
- Undo Last removes the most recent point; and
- Clear Draft removes the preview and authored waypoints.

For a ground-plane point, orientation is inherited from the preceding control point. If there is no preceding orientation, R/P/Y remains null and the planner/controller inherits pose semantics.

If Home is the first item, the UI captures the current pose and marks it home. Otherwise it clones the last authored pose and changes its move type to home.

### 10.2 Waypoint data model

Each authored waypoint can carry:

- X/Y/Z in metres;
- roll/pitch/yaw in degrees or null/inherited;
- move type: linear, joint, or home;
- optional linear speed in mm/s;
- optional linear acceleration in mm/s²;
- optional rotation speed in deg/s; and
- optional pause-after duration in seconds.

The browser omits optional numeric fields from the API payload when they are null. Pause commands are reconstructed into waypoint metadata when loading a planned trajectory.

### 10.3 Automatic planning

The browser calls `POST /trajectory/plan-points` with encoded waypoints. Different edit paths use either immediate planning or a 220 ms debounce. In-flight preview requests can be cancelled with `AbortController` when a newer edit supersedes them.

The response is converted into a preview containing:

- controller trajectory commands;
- exact Cartesian path samples;
- resolved waypoints and controller-computed motion metadata;
- planning warnings; and
- structured planner diagnostics.

Authored pose/order remains the source of truth; resolved speed/acceleration/rotation/pause metadata is merged back from the planned response.

### 10.4 Preview validity

Before execution, the UI compares the preview against the current authored waypoints, including:

- position;
- orientation;
- move type; and
- motion metadata.

An out-of-sync or explicitly stale preview cannot run. A loaded saved plan is marked stale if it no longer matches its stored waypoints and the UI asks the operator to regenerate.

### 10.5 Planner diagnostics

Planning failures and successful plans can expose detailed diagnostics, including:

- reason code;
- seed used;
- fallback level;
- attempt name;
- smoothing applied;
- profile revision;
- branch-anchor availability;
- maximum joint step;
- joint-limit margin;
- Cartesian and orientation residuals;
- violating pose/joint indices;
- raw versus wrapped jump values;
- jump context and sample source;
- recovery strategy and whether it was accepted;
- solved-point count and elapsed time for attempts; and
- split-recovery segment details.

The UI explicitly surfaces jump-gate failures, raw solver jumps, recovery attempts, and split segments rather than reducing planner failure to one generic message.

### 10.6 Simulation and live execution

The same preview can be submitted to `POST /trajectory/run` with:

- preview name;
- `execution_mode` (`simulate` or `live`);
- `use_cache`; and
- `loop_override`.

Execution gating is explicit:

- “Simulate” is enabled only when the controller is in SIM mode;
- “Run Trajectory” is enabled only in LIVE mode;
- both are blocked for missing/stale/mismatched preview, unknown runtime, active planning, submission in progress, or active controller motion.

Standard trajectory runs always send `use_cache: false`. Weld runs use the cache-backed path described later.

### 10.7 Loop execution

The trajectory drawer has a loop toggle. It is an execution-time override, not part of the saved trajectory authoring record. Controller behavior is expected to move to the initial waypoint and then close/repeat the pass.

### 10.8 Motion-status display

The trajectory and weld drawers show structured status such as:

- request accepted/acknowledged;
- high-level state and detail;
- source of truth and completion scope;
- trajectory ID;
- controller state and active mode;
- queue depth/capacity;
- motion-done flag;
- stale-command flag;
- underrun count;
- program name/state;
- current and completed step counts;
- loop state/iteration; and
- terminal/failure information.

Colors distinguish waiting, active, successful terminal, and failure states.

### 10.9 Saving and loading trajectory programs

The browser uses the Robot Program API rather than the older recorded-trajectory path:

- list: `GET /robot-program/list?kind=trajectory`;
- load: `GET /robot-program/{name}?kind=trajectory`; and
- save/overwrite: `POST /robot-program/save`.

A saved trajectory contains:

- `kind: trajectory`;
- program name;
- authoring waypoints;
- authoring metadata identifying the trajectory editor; and
- the current planned trajectory.

Save first ensures a valid preview exists. Loading hydrates motion metadata, records a saved baseline, disables cache use, and retains a mismatched saved plan only as stale visual evidence.

Regenerating a loaded program can overwrite the saved record when the loaded program name and current trajectory name still align.

There is no trajectory delete action in the current UI.

### 10.10 Draft history

Trajectory edits initiated through the Program Tree maintain a bounded in-memory history of up to 50 states. The draft action card supports:

- Undo Last;
- Undo All/revert to the last saved baseline; and
- Save.

Unsaved changes are determined by comparing the current waypoint list with the saved baseline.

## 11. Program Tree

### 11.1 Purpose and structure

The docked Program Tree is generated from the current planned trajectory and weld selection. It can contain:

- program root;
- setup group;
- operation groups;
- trajectory sequence;
- move groups;
- raw controller commands;
- exact path samples;
- control points; and
- weld segments.

Node badges, subtitles, coordinate summaries, move kinds, weld types, and sample counts provide context.

### 11.2 Chronological and grouped views

The operator can choose:

- **Chronological**: exact execution path samples or controller execution order; or
- **Grouped**: control point → move group → control point, with path samples nested inside each move.

The exact path view does not intentionally down-sample for the tree. Every available planned path sample becomes a node.

### 11.3 Cross-selection

Selecting a node can:

- open the relevant Trajectory or Weld drawer;
- select the matching timeline block;
- highlight waypoint indices;
- highlight a path range;
- focus the 3D camera; and
- make the matching weld edge the active segment.

Selection also works in the opposite direction: choosing an active weld edge or timeline item can select the corresponding Program Tree node.

### 11.4 Control-point editor

When a control point is selected, the tree exposes:

- X/Y/Z numeric editing, step 0.001;
- roll/pitch/yaw numeric editing, step 0.1;
- add joint control point;
- remove control point;
- move earlier; and
- move later.

Trajectory drafts may retain one point; weld drafts must retain at least two.

Trajectory point changes automatically replan. Weld changes remain editable locally until “Apply Waypoint Edits” is used, because the weld source geometry/cache must be rebuilt together.

### 11.5 Move editor

For a selected trajectory move, the tree allows:

- change between linear, joint, and home;
- speed edit and reset to controller default;
- linear acceleration edit and reset to controller default; and
- pause-after edit and reset to no pause.

Speed units adapt to the move:

- linear translation uses mm/s;
- primarily rotational/joint behavior uses deg/s.

Acceleration is offered only for suitable linear translational moves. Pause is unavailable on the final waypoint. Text-entry updates are debounced (approximately 900 ms) and can also commit on Enter/blur.

### 11.6 Motion-profile visualization

For linear moves the Program Tree renders an estimated speed-versus-distance profile:

- trapezoidal when the requested speed is reached;
- triangular when distance is too short to reach it;
- target and peak speed;
- acceleration, cruise, and deceleration distances; and
- estimated move duration.

When no explicit acceleration is authored, the graph uses an approximate default that reaches the requested speed in about one second. The component notes that graph estimation and controller timing are not identical timing authorities.

### 11.7 Editing locks

Waypoint and move editing is disabled while:

- weld planning is active;
- a preview is being submitted; or
- controller motion is active.

### 11.8 Visibility

The tree can be hidden from the header or with `T`. A placeholder remains in the dock with a “Show Program Tree” action so the operator can restore it without opening Settings.

## 12. Bottom program timeline

The timeline is a shared, horizontally scrollable dock with one or more lanes.

Trajectory lanes interleave:

- control-point blocks (`CP1`, `CP2`, and so on); and
- move blocks (`M1`, `M2`, and so on).

Blocks show move kind, coordinates, speed/acceleration/rotation details, and pauses where available. Weld lanes show selected segments and their edge IDs/types.

Selecting a block synchronizes the Program Tree, authoring drawer, stage highlight, camera focus, and the timeline playhead summary. The header shows a playhead label and detail even if a move is represented by a synthetic timeline selection rather than a direct tree node.

When there is no program content the dock retains an empty-state prompt.

## 13. Weld authoring and planning

### 13.1 Prerequisites and edge selection

Weld edge selection requires:

- a STEP topology model;
- a live API connection; and
- edge-selection mode enabled.

In the 3D stage, hover and click use invisible/thick pick geometry while rendered edges communicate hover, selected, and active states. Selection is additive, so a weld draft can contain multiple edges. The drawer lists them, allows one to become active, and supports removal.

### 13.2 Weld draft fields

The weld draft includes:

- primary/default weld type;
- per-segment weld type;
- weld name;
- work angle in degrees;
- travel angle in degrees;
- spin angle in degrees;
- transition clearance in millimetres;
- post action: none, lift, or return to start;
- normalized start/end parameter for each edge; and
- active segment edge ID.

Available weld types in the selector are fillet, butt, lap, tack/spot, and custom.

Start/end sliders display both normalized location and an estimated distance along the sampled edge in millimetres.

### 13.3 Work/travel angle help

The angle fields include an interactive portal-rendered explainer. It:

- shows work and travel angle diagrams;
- allows dragging the illustrated torch tips to see the angles change;
- uses pointer capture for stable dragging;
- clamps itself to the viewport;
- updates on resize/scroll; and
- closes on outside click or Escape.

### 13.4 Section construction

Before calling the backend, the browser samples trimmed spans from each selected topology polyline. It then constructs explicit sections:

- adjacent/contiguous weld spans with the same weld type are merged;
- disconnected spans are separated by lift/travel transition points at the configured clearance; and
- all points are transformed through the current STEP transform.

This client-side section description is sent as both a flattened waypoint override and typed weld/transition sections.

### 13.5 Weld planning request

`POST /trajectory/plan-weld` includes:

- model ID;
- primary edge ID and start/end parameters;
- primary weld type and name;
- `sample_count: 48`;
- computed waypoint override;
- explicit sections with kind, weld type, edge ID, and points; and
- options for work, travel, spin, clearance, and post action.

The result is converted into the common preview model. The returned editable waypoints become the weld-editing source, and an optional `weld_preview_joint_pose` drives the ghost robot.

### 13.6 Automatic preview updates

While the Weld drawer is active, changes to the active segment identity/range or work/travel/spin angles schedule a weld preview after roughly 180 ms. A direct Plan action is also available.

Program Tree waypoint edits are intentionally not treated as ordinary automatic angle updates. They must be applied explicitly and cause the weld preview to be replanned.

Planning failure copy suggests moving the STEP model nearer the robot or adjusting the trimmed range when waypoint planning fails.

### 13.7 Weld simulation and live run

The drawer exposes:

- Simulate Weld in SIM mode; and
- Run Weld in LIVE mode.

Before every weld run the browser replans from the current robot state. This is required so `return_to_start` targets the actual pre-run pose for that invocation rather than a pose captured when the draft was first created.

Weld execution sends `use_cache: true` because the cached high-fidelity path preserves dense weld arcs and transitions. Standard trajectory execution sends `use_cache: false`.

The UI also displays a live `Weld ON` badge and stage glow when monitor telemetry reports weld output active.

### 13.8 Saving and loading weld programs

The browser uses:

- list: `GET /robot-program/list?kind=weld`;
- load: `GET /robot-program/{name}?kind=weld`; and
- save/overwrite: `POST /robot-program/save`.

A saved weld program embeds:

- STEP filename;
- full STEP base64 payload;
- STEP transform;
- complete weld draft and segment data;
- editable waypoints; and
- current planned trajectory.

This makes a weld program self-contained enough to restore its source part and edge selection. There is no weld-program delete action in the UI.

## 14. Tool Library

### 14.1 Quick selection drawer

The Tool Library workspace is optimized for selection and runtime staging. It shows:

- active runtime tool;
- selected/desired tool;
- whether the desired state requires a restart;
- runtime/tool errors;
- refresh tools;
- refresh runtime state;
- stage/apply selected configuration; and
- apply/restart when the returned runtime snapshot reports `restart_required`.

It also links into the full Tool Library editor in Settings.

### 14.2 Filtering

Tools can be filtered by:

- compatible robot (`compatible_robot_ids`, with an empty list meaning all robots);
- tool type; and
- free-text keyword.

Free-text search covers tool ID, display name, description, tool type, and keywords.

### 14.3 Tool schema

The editor supports:

- stable `tool_id`;
- display name;
- description;
- tool type;
- keyword list;
- compatible robot IDs;
- TCP position offset X/Y/Z in millimetres;
- TCP rotation X/Y/Z in degrees;
- optional mesh asset path;
- uniform mesh scale;
- mesh position X/Y/Z in millimetres relative to J6/flange; and
- mesh rotation X/Y/Z in degrees relative to J6/flange.

The tool object may contain a `weld` metadata object, and clone/save behavior preserves it, but the current Settings UI does not expose a weld-metadata editor.

### 14.4 CRUD behavior

The full editor supports:

- new draft with a timestamp-derived `tool-{milliseconds}` ID;
- create via `POST /tools/library`;
- update via `PATCH /tools/library/{tool_id}`;
- delete via `DELETE /tools/library/{tool_id}?actor=web-ui`; and
- refresh/list via `GET /tools/library`.

Create/update bodies identify the actor as `web-ui`. Delete requires confirmation. The built-in `identity` tool cannot be deleted.

### 14.5 Runtime activation

The selected robot, active tool, and RTCore max-RPM override are staged together with `PATCH /info/runtime-config`. The UI treats the backend's returned `restart_required` flag as authoritative and exposes controller restart through `POST /control/restart-controller`.

After requesting restart the UI waits about 1.25 seconds before refreshing runtime configuration.

### 14.6 Asset pipeline and actual renderer support

Tool definitions are discovered at build time from `tools/library/<tool_id>/tool.json`. The sync script copies a configured local mesh, or auto-detects the first `.stl`, `.glb`, or `.gltf` file, into `public/assets/tools` and creates an index.

However, the current Three.js runtime loader only implements STL. GLB/GLTF can be discovered and copied but will produce the fallback marker rather than a rendered tool. The drawer's “STL/GLB/GLTF” drop-in copy therefore overstates current visualizer support.

## 15. Settings dialog

Settings is a modal with fixed header/tab row and an internally scrolling content region. It closes via its close action, outside click, or Escape.

### 15.1 General tab

General settings include:

- API host;
- vision host;
- runtime policy status and selection;
- desired robot;
- RTCore maximum RPM override;
- show gripper panel;
- show software-zero button; and
- show arm bounding box.

The default API host is derived from the current page protocol/hostname on port 4400. The default vision host uses port 8080.

The runtime-policy readout shows active robot/mode, IK solver, servo backend, drive profile, source/default/override information, active tool, and RTCore clamp/default/source. It warns when unsafe/development overrides are active.

An RPM value of 0 is described as disabling the clamp; a blank value asks the backend to use startup/default policy.

### 15.2 Tool Library tab

This tab contains the complete schema editor, filtering, CRUD, runtime selection, refresh, and restart behavior described in Section 14.

### 15.3 Kinematics tab

The Kinematics tab loads `GET /kinematics/profile` and displays:

- profile ID and version;
- robot ID and serial;
- current profile revision;
- base runtime offset; and
- tool runtime offset.

Operators can edit base and tool:

- position X/Y/Z in metres; and
- rotation X/Y/Z in degrees.

The UI also shows translational delta in millimetres for readability.

Apply calls `PATCH /kinematics/runtime-offsets` with `expected_revision`, base, and tool. Reset calls `POST /kinematics/runtime-offsets/reset` with the expected revision. Both require confirmation.

If an optimistic-concurrency/revision error occurs, the UI reports it and refreshes the authoritative profile snapshot.

### 15.4 Runtime configuration discovery

Runtime policy is loaded by fetching `GET /info/robots` and `GET /info/runtime-config` in parallel. The robot catalog provides name, ID, display name, version, default servo backend, default IK solver, and optional drive profile.

Runtime configuration distinguishes:

- active/effective state;
- desired state;
- default and override sources;
- configuration metadata; and
- restart requirement.

## 16. Robot Control dock

Robot Control remains visible on the right regardless of the authoring workspace. Its body can be collapsed, while the compact header status remains visible.

### 16.1 Live status sources

The dock consumes shared monitor telemetry and motion status. If it is rendered outside the shared LiveState provider, it can operate standalone by polling:

- `GET /info/joints-detailed` every 100 ms for joint feedback; and
- `GET /control/motion-status` every 200 ms for execution status.

The main App supplies shared state, so the standalone motion-status section is hidden in favor of the compact header status.

### 16.2 Drive power and fault control

For EtherCAT RTCore backends, the UI provides explicit:

- Power Up;
- Power Down with `wait_for_idle: true`;
- Reset All Faults; and
- per-axis fault reset.

Actions require confirmation. Power Up fails closed until the controller supplies a known safe-for-power-transition result. The UI blocks it for active faults, active motion/unsafe state, or unsynchronized state. A disarmed safe result must remain stable for 600 ms before enable is allowed.

Power state/fault presentation can show:

- servo backend and drive profile;
- physical/driver state;
- EtherCAT master state;
- RTCore state;
- armed flag;
- requested and operation-enabled axes;
- enable/native-home masks;
- online/operational/startup-ready counts;
- working-counter actual/expected;
- master AL state;
- metrics/reference source; and
- per-axis feedback.

### 16.3 Per-axis drive diagnostics

Per-axis cards can show:

- axis index and logical joint;
- DS402 state and statusword;
- standard and manufacturer error codes;
- decoded fault name/class/source;
- resettable status;
- EtherCAT slave online/operational and AL state;
- position counts;
- absolute feedback fields;
- canonical/native truth validity and reason;
- coordinate-system validity;
- native-home state, active state, offset, and abort code; and
- startup drive configuration command/readback verification.

### 16.4 Encoder retention capture

The commissioning flow can capture snapshots around a drive power cycle through `POST /control/encoder-retention/capture`:

- `before_power_down`; and
- `after_power_up`.

The API returns/continues an experiment ID. When the second phase yields comparison data, the UI reports that a before/after comparison artifact was written.

### 16.5 Motion state

The reusable panel contains a Motion State section showing:

- status/detail;
- source and completion scope;
- trajectory ID;
- controller state/mode;
- queue depth/capacity;
- done/stale flags;
- underrun count; and
- safe-for-power-transition result and blockers.

In the main workstation this full section is intentionally hidden because the top header already renders its compact equivalent.

### 16.6 Gripper

The gripper panel is hidden by default and enabled from Settings. It provides:

- 0–180° slider;
- approximately 80 ms command debounce;
- Open preset at 120°; and
- Close preset at 0°.

Commands use `POST /control/set-gripper` with `angle_deg`.

### 16.7 Speed multiplier

The speed slider spans 0–1000 and maps exponentially to 0.1×–10×:

`10^((slider/1000 * 2) - 1)`

The midpoint is 1×. The multiplier affects realtime linear/angular jog and discrete relative linear moves.

### 16.8 Cartesian and rotational jog

The jog surface includes:

- Deadman checkbox, enabled by default;
- Debug checkbox wired to `POST /control/jog/debug`;
- Arm Jog / Disarm Jog;
- editable linear base speed, default 50 mm/s, range shown as 0–1000;
- editable angular base speed, default 15 deg/s, range shown as 0–360;
- live effective base rates after multiplier;
- last backend jog vector;
- ±X, ±Y, ±Z; and
- ±Roll, ±Pitch, ±Yaw.

Multiple held buttons can combine into multi-axis velocity vectors.

### 16.9 Hold-to-jog session protocol

When jog is armed, pointer-down starts or updates a controller jog session:

- each browser instance creates an owner UUID;
- sequence numbers increase monotonically;
- the session lease is 1 second;
- the local tick interval is 50 ms;
- unchanged held input is refreshed at least every 100 ms;
- commands contain linear velocity in m/s and angular velocity in deg/s; and
- releasing the final held control sends a session stop.

The client coalesces queued state so a slow request does not create an unbounded queue. Expired/not-found/inactive session errors can recreate the session while input is still held. Owner-conflict/terminal errors disarm the UI and clear queued state.

### 16.10 Jog failsafe stop

The browser sends a best-effort session stop using `sendBeacon`, falling back to `fetch(..., keepalive: true)`, when:

- the page is hidden;
- `pagehide` fires;
- `beforeunload` fires; or
- the control component unmounts.

It also immediately clears local held axes and disarms jog.

### 16.11 Discrete movement when realtime jog is not armed

The same axis buttons perform a discrete command on pointer release when Arm Jog is off:

- linear buttons call `POST /control/move-line-relative`; and
- rotational buttons call `POST /control/rotate`.

There is an important semantic quirk: the field labelled “Linear Base (mm/s)” is treated as a distance in millimetres for each discrete linear command. The angular base value is likewise used as the discrete angle in degrees, with duration calculated from the effective angular speed.

### 16.12 Joint commissioning

Joint Commissioning is collapsed by default. It includes:

- explicit “RTCore cap 100 RPM” warning/copy;
- refresh feedback;
- step presets 0.25°, 1°, and 5°;
- custom signed step with minimum magnitude handling;
- J1–J6 current angle;
- live/stale/canonical truth state;
- per-joint ± step;
- optional logical Zero;
- drive-native Home; and
- drive/encoder health chips.

Relative joint jog uses `POST /control/joint-jog` and waits for idle. It stops realtime jog before starting. Repeated commands are guarded by action state and a short debounce.

Software Zero calls `POST /control/zero-joint`, writes a persistent logical offset, and is hidden unless explicitly enabled in Settings.

Drive Home calls `POST /control/home-joint-native` after a detailed confirmation. Only one native-home operation may be active at a time. The UI reports acceptance, verification, persistence, refreshed feedback, and failure/abort status. It explicitly warns that the homed axis remains disabled until another Power Up.

### 16.13 Commissioning health chips

Where extended drive telemetry is present, the browser derives UI warning/error chips for:

- bus voltage below 18 V or above 60 V;
- IGBT temperature warning above 70°C, error above 85°C;
- motor temperature warning above 70°C, error above 100°C;
- drive load warning above 80%;
- large position error above 100 display units;
- drive-not-ready text; and
- motor-not-rotating text.

These are presentation thresholds, not controller safety limits.

### 16.14 Global motion buttons

The bottom of Robot Control repeats:

- STOP;
- Home; and
- Rest.

Home/Rest are blocked while another motion action is pending. STOP remains the dominant immediate action.

## 17. Live Charts workspace

### 17.1 Joint charts

The chart tab keeps up to 300 samples for each J1–J6 and plots operator/display joint angles in degrees. Each joint panel shows the latest value and can be hidden.

### 17.2 Per-servo charts

Multi-series charts can display each servo ID for:

- voltage;
- current;
- torque/load percentage;
- temperature;
- position counts;
- raw torque;
- mode display;
- statusword;
- error code; and
- digital-input bits.

The per-servo chart:

- uses a distinct palette by servo ID;
- aligns histories by their newest sample;
- scales dynamically unless a metric supplies fixed limits;
- shows a hover tooltip with every servo's value at the selected historical sample; and
- identifies how many samples ago the hover location is.

The panel is responsive to its container through `ResizeObserver`.

### 17.3 Backend-aware availability message

If the RTCore backend supplies counts/torque/mode/statusword/error/digital inputs but no voltage/current/temperature history, the UI explains that those legacy vitals are not yet mapped into the RTCore snapshot rather than implying the charts are broken.

### 17.4 Drive Feedback cards

When rich servo fields exist, per-axis cards show:

- axis/logical joint;
- DS402 state;
- mode;
- statusword;
- error code;
- counts;
- raw torque;
- digital inputs;
- fault flags;
- brake state;
- named status labels; and
- voltage/current/temperature/load when present.

Faulted axes are rose, OperationEnabled axes are emerald, and other states use a neutral style.

### 17.5 Active alarms

The chart parser prefers named live `status_names`. If absent, it decodes bit strings into fallback labels:

- Overload;
- Overheat;
- Overvoltage;
- Undervoltage;
- Stall;
- Position Fault;
- Comm/Error; and
- Unknown.

The active alarm block groups each alarm name with the affected servo IDs.

The component also computes LED-alarm and unloading-alarm maps, but only the consolidated active-alarm map is currently rendered. Those two additional maps and their summary sets are dormant state.

## 18. Timing diagnostics workspace

Live Charts contains tabs for “Live Charts” and “Diagnostics.” The diagnostics tab can itself be hidden; hiding it returns the active view to Live Charts and stops performance polling.

### 18.1 Adaptive polling

Diagnostics calls `GET /debug/performance`:

- every 250 ms while jogging or controller motion is active; and
- every 1,000 ms while idle or after an error.

It retains the last successful snapshot when later polling fails and labels snapshots live, stale after 2 seconds, offline, waiting, or error.

### 18.2 Summary signals

Four high-level cards show:

- Jog API round-trip time;
- Move acknowledgement time;
- controller UDP dispatch time; and
- RTCore jitter.

Each shows a primary value, contextual maximum/count information, and a proportion bar.

### 18.3 Detailed diagnostics

The panel further separates:

**UI → API timing**

- jog session average/maximum round trip;
- relative move average/maximum round trip;
- wait-for-idle average/maximum round trip;
- jog session state;
- lease remaining; and
- last update age.

**Controller UDP dispatch**

- last command and dispatch duration;
- global average/maximum dispatch;
- interarrival last/average;
- move and jog dispatch average/maximum;
- counts over 5 ms and over 20 ms; and
- stale command-link state and last receive age.

**Jog loop internals**

- loop average/maximum;
- maximum overrun and overrun count;
- velocity update gap average/maximum;
- feedback read average/maximum;
- IK solve average/maximum;
- command send average/maximum;
- last velocity-command age;
- lease expiry count; and
- stale sequence reject count.

**RTCore health**

- current and maximum real-time jitter;
- RT overrun count;
- motion update age;
- motion state;
- jogging state;
- execution/backend policy;
- owner-conflict rejects; and
- last jog-stop reason.

**Controller pose snapshot**

- XYZ metres;
- RPY degrees; and
- J1–J6 degrees.

**Jog IK readout**

- sequence and step duration;
- target pose;
- target-versus-solved error;
- target-versus-applied error;
- clamp flag and clamped joint indices;
- solve-failed flag; and
- current, solved, and applied joint arrays.

### 18.4 Pose history

Each performance snapshot with a controller pose can be appended to an in-memory history, capped at 1,200 unique samples. Duplicate snapshots are suppressed by comparing time, pose, IK debug, motion, jog, and session state.

History provides:

- sample count;
- first/last timestamps;
- total XYZ delta;
- total RPY delta;
- last motion/jog/session state;
- latest joint values;
- latest API/controller command names; and
- a scrollable table of time, state, XYZ, RPY, J1–J3, and J4–J6.

“Save Local JSON” sends the complete history to `POST /debug/pose-history`; despite the button wording, the server writes the local file and returns its path. The UI automatically saves once when an active jog session transitions to stopped, and also supports manual save and clear.

## 19. 3D lag instrumentation

The application contains end-to-end instrumentation specifically for visualizer latency. For up to 120 completed and 240 pending samples it tracks:

- sample source: monitor, REST fallback, or prop;
- controller/source age;
- source-to-API time;
- source cadence;
- API sequence gaps;
- API-to-browser time;
- browser receive cadence;
- parse-to-visual-push time;
- receive-to-visible time;
- push-to-visible time;
- frame interval;
- frame work duration;
- payload bytes;
- joint count; and
- whether display joints were used.

It maintains received, pushed, applied, superseded, dropped, and pending counters and computes current, average, and maximum samples.

The on-stage “3D Lag Probe” refreshes every 500 ms and shows key segments plus a diagnosis. Diagnostic rules distinguish:

- controller/backend sample already old;
- controller monitor cadence below target;
- API/SSE delivery backlog;
- browser receive cadence below target;
- JSON/sample handling delay;
- main-thread/render-loop delay; and
- excessive 3D frame work.

Warnings are written as structured console records with rate limiting; a heartbeat trace is emitted roughly every five seconds. Detailed snapshots are also published to global `__GRADIENT_3D_LAG_TRACE__` for live inspection.

## 20. Archived J6 manual-rotation dataset page

### 20.1 Nature of the page

`/j6-manual-rotate-dataset.html` is a standalone analysis artifact, not a live data browser. The source dataset path, every numeric series, elapsed times, metadata, observations, and statistics are compiled into the TSX file.

It documents a capture from:

`/home/pi/GradientOS/logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl`

### 20.2 Dataset summary

The embedded run contains:

- 1,189 original samples;
- 144 plotted samples;
- 1,045 flat-tail samples removed;
- 227.105 seconds plotted duration;
- 33 numeric series;
- 26 variable series;
- 7 constant series; and
- 22 constant metadata fields.

The flat-tail rule records thresholds of 8 combined counts, 8 absolute counts, and 0.02 degrees.

### 20.3 Analysis observations

The page explains three conclusions from the run:

- absolute A6-EC multi-turn counts remain continuous through large positive/negative excursions;
- controller/API/frontend public truth follows the roughly −570.9° to +535° manual sweep; and
- wrapped behavior remains in the raw/write-reference family rather than the absolute encoder source.

### 20.4 Dual-axis chart

The custom SVG chart uses:

- counts-like fields on the left axis;
- degrees, radians, and status/meta fields on the right axis;
- separate palettes by field group;
- multiple dash patterns;
- dynamic range/padding for each axis;
- a click-selected sample line; and
- a marker for each visible series at the selected sample.

Clicking the chart chooses the nearest elapsed-time sample. A range scrubber provides precise sample selection.

### 20.5 Presets and series selection

Available presets are:

- all non-constant numeric;
- all numeric including constants;
- counts only;
- degrees only;
- radians only;
- counts plus degrees;
- raw versus absolute counts;
- controller/API/UI truth; and
- custom.

Individual series can be searched by field name and toggled. Toggles are grouped into counts, degrees, radians, and status/meta numerics. Choosing an individual toggle changes the active preset to custom.

### 20.6 Sample inspection and tables

The page provides:

- sample number and elapsed time;
- highlighted key encoder/controller/API/frontend fields;
- all visible series values at the selected sample;
- numeric field index with axis, group, min, max, and constant flag; and
- collapsible constant metadata table.

The key spotlight includes absolute counts, combined signed counts, drive fields such as `6064` and `U40.16`, controller axis counts, public degree/radian values, and statusword.

### 20.7 Limitations

- It does not accept a user-selected JSONL file.
- It does not fetch the displayed source path at runtime.
- It does not export chart data or images.
- Preset/selection state is not persisted.
- It represents one archived capture only.

## 21. Backend interface inventory

The following table describes the browser-visible contracts used by the main UI.

| Method and path | Feature | Browser behavior/payload |
|---|---|---|
| `GET /monitor` | Live state | SSE for joints, display joints, servos, alerts, drive faults, weld state, comms, motion status, timing |
| `GET /info/joints` | Header drive fallback | Joint/gripper feedback fallback |
| `GET /info/joints-detailed` | Robot Control fallback | Detailed/canonical joint feedback and per-axis commissioning fields |
| `GET /info/pose` | Trajectory authoring | Current TCP XYZ/RPY capture |
| `GET /info/robots` | Settings/runtime | Available robot policies and defaults |
| `GET /info/runtime-config` | Settings/runtime | Active, desired, sources, overrides, restart requirement |
| `PATCH /info/runtime-config` | Settings/tool | Stage robot, active tool, RT maximum RPM; actor `web-ui` |
| `POST /control/runtime-mode` | Header | Hot switch `live`/`simulate` |
| `POST /control/restart-controller` | Settings/tool | Request restart with reason |
| `GET /control/motion-status` | Standalone Robot Control | Poll execution/power-transition state |
| `POST /control/stop` | Header/Robot Control | Stop motion |
| `POST /control/home` | Header/Robot Control | Move robot home |
| `POST /control/rest` | Header/Robot Control | Move robot to rest pose |
| `POST /control/power-up` | Drive control | Explicit drive enable after client/backend safety gates |
| `POST /control/power-down` | Drive control | Power down, requesting wait for idle |
| `POST /control/reset-faults` | Drive control | All axes or selected joint, depending payload |
| `POST /control/set-gripper` | Gripper | `angle_deg` |
| `POST /control/jog/debug` | Jog | Enable/disable debug mode |
| `POST /control/jog/session/start` | Jog | Owner, sequence, lease, deadman, six-axis velocity |
| `POST /control/jog/session/update` | Jog | Session ID plus next sequence and velocity |
| `POST /control/jog/session/stop` | Jog | Session/owner/reason; also beacon/keepalive failsafe |
| `POST /control/move-line-relative` | Discrete jog | Relative XYZ metres, speed multiplier, open/closed flag |
| `POST /control/rotate` | Discrete jog | Axis, angle degrees, duration seconds |
| `POST /control/joint-jog` | Commissioning | Joint and relative step |
| `POST /control/zero-joint` | Commissioning | Persist logical zero for joint |
| `POST /control/home-joint-native` | Commissioning | Run drive-native home for joint |
| `POST /control/encoder-retention/capture` | Commissioning | Phase and optional experiment ID |
| `POST /cad/topology/load-step` | STEP/weld | Filename, base64 STEP, sample count |
| `POST /trajectory/plan-points` | Trajectory | Authored pose/motion waypoints |
| `POST /trajectory/plan-weld` | Weld | Model/edge/range, sections, points, angles/options |
| `POST /trajectory/run` | Execution | Name, cache flag, execution mode, loop override |
| `GET /robot-program/list?kind=...` | Saved programs | List trajectory or weld names |
| `GET /robot-program/{name}?kind=...` | Saved programs | Load trajectory or weld authoring and plan |
| `POST /robot-program/save` | Saved programs | Create/overwrite trajectory or weld record |
| `GET /tools/library` | Tool Library | List tools/default tool |
| `POST /tools/library` | Tool Library | Create `{actor, tool}` |
| `PATCH /tools/library/{id}` | Tool Library | Update `{actor, tool}` |
| `DELETE /tools/library/{id}?actor=web-ui` | Tool Library | Delete non-identity tool |
| `GET /kinematics/profile` | Kinematics | Profile, revision, base/tool offsets |
| `PATCH /kinematics/runtime-offsets` | Kinematics | Expected revision and new offsets |
| `POST /kinematics/runtime-offsets/reset` | Kinematics | Expected revision, reset to zero |
| `GET /debug/performance` | Diagnostics | API/controller/jog/RTCore timing and pose snapshot |
| `POST /debug/pose-history` | Diagnostics | Persist browser-collected pose history on API host |

No authentication/authorization token is added by the browser code. Requests assume a trusted, reachable Gradient API and appropriate CORS for the page origin.

## 22. Build, assets, and implementation stack

### 22.1 Main technologies

- React 19 and React DOM;
- TypeScript;
- Vite;
- Tailwind CSS 4;
- Three.js;
- URDF Loader;
- OCCT STEP import through `occt-import-js`;
- Lucide icons; and
- Vitest/jsdom/testing-library.

`@react-three/fiber` is installed but the main visualizer uses direct Three.js renderer management.

### 22.2 Development/build behavior

- Vite dev server uses port 8000 and binds `0.0.0.0`.
- Allowed hostnames include `gradientrobotics.local`, `jetson.local`, and `mini-arm.local`.
- Both HTML entry points are production build inputs.
- Robot and tool assets are synchronized before both development and production builds.

### 22.3 Robot asset synchronization

The robot sync script scans `robots/*/robot.json`, validates each manifest, copies its configured URDF and sibling `stl-files`, copies optional USD files, writes a canonical `robot.urdf`, and produces a catalog index.

It requires exactly one default robot. A missing manifest, robot-ID mismatch, invalid asset directory, missing URDF, or multiple/no defaults fails the sync/build.

### 22.4 Tool asset synchronization

The tool sync script scans `tools/library/*/tool.json`, ignores hidden/underscore directories, resolves configured or detected local mesh assets, copies them into public assets, and writes the tool index. Missing optional meshes warn without failing the entire sync.

### 22.5 Automated behavior coverage

The checked-in UI tests are focused rather than comprehensive. They currently exercise:

- raw, display, and live joint-truth selection and fallback rules;
- synthesized/merged drive-power and drive-fault state;
- hold-to-jog session start, refresh, release, expiry recovery, and stop behavior;
- fresh-monitor versus detailed-joint polling behavior;
- partial, stale, unavailable, and untrusted joint feedback presentation;
- controller-owned joint-delta submission without browser-side baselining;
- drive power safety gating, requested-versus-actual state, and wait-for-idle power-down;
- safe/check/blocked runtime-header semantics and stabilization timing;
- software-zero visibility, native-home state/verification, and action interlocks;
- extended A6 EtherCAT PDO health chips and temperature thresholds; and
- the archived J6 page deep link and preset switching.

There are no equivalent component/integration tests for most STEP, trajectory, weld, tool-library, Program Tree, timeline, settings, live-chart, or Three.js interaction flows. Those features are implemented, but a port should not interpret the current test suite as full regression coverage.

## 23. Source-level limitations, dormant features, and mismatches

These details are important when deciding what to reproduce.

### 23.1 Documentation/default-port mismatch

The README still says the Gradient API default is port 4000. Current `resolveDefaultApiHost()` uses port 4400 and explains that Windows IP Helper often reserves 4000. New work should follow the runtime code/configuration contract, not the stale README line.

### 23.2 Tool live-apply wording mismatch

The README says the active tool applies live without restarting. Current browser behavior stages `active_tool_id` in runtime config and follows the backend's `restart_required` response, exposing “Apply & Restart” when required. Ported UX should present active versus desired state explicitly and not promise unconditional hot application.

### 23.3 Tool mesh format mismatch

Build discovery and UI copy mention STL/GLB/GLTF, but the runtime visualizer only loads STL. Other formats display the fallback marker.

### 23.4 Normal trajectory run overlay mismatch

For a normal LIVE trajectory, the UI overlay says “Recalculating Trajectory Path” and “Rebuilding the runnable path from the robot's current pose.” The run handler does not call the point planner again; it validates the existing preview and submits it. Weld runs do explicitly replan. This wording should not be carried over without matching behavior.

### 23.5 Weld cache-readiness state is write-only

`weldPreviewCacheReady` is set/reset throughout import, edit, plan, load, and disconnect flows, but is not read to gate any current button or display. Actual weld execution still replans and uses cache. The state is dormant and should not be copied as an assumed safety gate.

### 23.6 Dormant collapse settings

Four drawer collapse flags are persisted but not used by the current drawer rendering. Only Robot Control collapse is wired.

### 23.7 Dormant compact telemetry component

`TelemetryPanel` can show received time, joint degrees, and gripper, but the main drawer renders `TelemetryWorkspace` instead. It is not part of the visible main application.

### 23.8 Dormant alarm-derived state

Telemetry Charts computes LED alarm and unloading alarm maps and sets, but only active status alarms are rendered.

### 23.9 Host settings are session-only

API and vision hosts are editable but not part of persisted `gradient-ui:settings`. A refresh returns them to hostname-derived port 4400/8080 defaults.

### 23.10 Broad disconnect reset

Disconnecting the SSE stream clears previews, topology, drafts, and saved-program lists. This couples live connectivity to authoring continuity. An offline-first workstation should avoid losing unsaved CAD/program work merely because observation disconnects.

### 23.11 No delete UI for saved robot programs

Trajectory and weld programs can be listed, loaded, saved, and overwritten, but not deleted or archived from this browser.

### 23.12 J6 page is generated/static

The J6 viewer looks like a dataset exploration tool but is a single embedded capture. Generalizing it requires file import or a diagnostic dataset API, schema discovery, and runtime series construction.

### 23.13 Browser is a direct controller client

The main UI directly invokes mode switching, power, jog, zero/home commissioning, trajectory run, and STOP endpoints. This is a materially different authority model from Offline Programming V1, where the browser cannot arm, jog, enable weld output, or communicate directly with RTCore/EtherCAT.

## 24. Recommended interpretation for Offline Programming

This section classifies functionality by how it fits the existing Offline Programming architecture. It is a planning aid, not an implementation status claim.

### 24.1 Strong candidates to bring over directly or adapt closely

| GradientOS capability | Offline Programming interpretation |
|---|---|
| Lazy 3D startup | Retain; large CAD/URDF scenes should not block the editor shell |
| Resizable panes and persisted layout | Retain; map to spreadsheet, viewer, planning evidence, program tree, and timeline |
| STEP transform controls and coordinate guidance | Retain; integrate with Offline Project fixture/workpiece placement |
| Topology edge hover/selection and trimmed start/end | Retain and extend into existing seam/pass authoring |
| Selected-path/control-point highlighting | Retain; synchronize spreadsheet rows, viewer, plan samples, and timeline |
| Chronological/grouped Program Tree | Adapt as an alternate structural view over stable Robot V4 row IDs |
| Shared bottom timeline | Adapt to accepted-plan samples and local simulation; preserve row/sample identity |
| Move speed/acceleration/pause editor | Reuse interaction ideas, but edit authoritative spreadsheet fields rather than a parallel waypoint model |
| Motion-profile graph | Retain as clearly labelled estimate or render target-plan timing evidence |
| Detailed planner diagnostics | Strong candidate; normalize Tesseract/Motion Server collision, limit, singularity, recovery, and residual evidence |
| Ghost robot and weld angle guides | Retain for offline reach/orientation preview |
| Tool TCP versus mesh-placement separation | Retain; it is a useful and correct modeling distinction |
| Robot/tool asset catalogs | Adapt to target manifest, robot model, tool/work-frame revisions, and digest-bound assets |
| Backend-aware telemetry availability copy | Retain for connected target observation so absent fields are explained accurately |
| Hide/restore chart panels | Retain for user-configurable diagnostics workspaces |
| Pose/timing history viewer | Adapt as read-only connected-target evidence or imported diagnostic artifact |
| Lag pipeline instrumentation | Reuse for viewer/stream performance diagnostics, especially on low-power clients |
| Interactive archived dataset viewer | Generalize into a file/API-driven diagnostic dataset explorer if it is a product requirement |

### 24.2 Features to adapt through Offline Programming's authority boundaries

| GradientOS capability | Required adaptation |
|---|---|
| Saved trajectory/weld programs | Use the shared `robot.v4.program.v1` catalog, stable IDs, revisions, provenance, and optimistic concurrency; do not introduce a second `/robot-program` island |
| Planner preview | Keep browser preview visibly non-executable; physical target plans must come from the selected Motion Server |
| Simulate/Run gating | Map to Offline Preview, explicitly opted-in contained local simulation, Target Plan, Review Physical Start, and Start Reviewed Plan |
| Motion status | Consume the existing execution gateway and connected observation; show accepted-plan/program/manifest/lease/fence identities |
| LIVE/SIM display | Represent authority/classification and selected target rather than hot-swapping a controller from the browser |
| Tool activation | Edit program/project tool references and request target planning; active physical runtime policy stays target-owned |
| Runtime/kinematics settings | Present read-only target manifest/profile evidence or create explicitly reviewed configuration workflows outside ordinary program editing |
| Alerts and drive telemetry | Read through connected monitoring and manifest-bound normalization; no raw direct controller client |
| Vision feed | Proxy or authorize through the server if added; do not assume arbitrary cross-origin camera access |
| STOP | Keep the existing server-mediated reviewed execution stop/protective tail; do not copy a raw controller endpoint |

### 24.3 Features intentionally outside the current Offline Programming browser

The following GradientOS controls conflict with Offline Programming V1's documented safety model and should not be copied as direct browser controls:

- Arm/Disarm realtime jog;
- Cartesian/rotational hold-to-jog;
- discrete jog commands;
- gripper actuation;
- drive Power Up/Power Down;
- drive fault reset;
- software joint zero;
- drive-native homing;
- encoder-retention commissioning capture as a motion/control action;
- raw Home/Rest motion commands;
- controller runtime hot switching;
- controller process restart;
- direct weld-output state control; and
- direct raw `/trajectory/run` execution.

These can still inspire read-only status, diagnostic views, or separately commissioned service tools, but they must not bypass Offline Programming's target plan, review token, lease/fence, preload/play, supervision, and protective-tail architecture.

### 24.4 Data-model caution

GradientOS uses a lightweight waypoint list plus a saved plan object. Offline Programming already has a broader shared spreadsheet program with stable row IDs, loops, comments, capture/calibration, positioner, weld/process, RevPi I/O, provenance, revisions, and plan invalidation.

Useful GradientOS UI concepts should therefore be projected onto the existing Robot V4 program and accepted-plan schemas. The waypoint model must not become a competing source of truth.

### 24.5 Offline-first improvements suggested by this review

When borrowing the UI ideas, Offline Programming should improve on several couplings in the source application:

- preserve unsaved authoring state through target/telemetry disconnects;
- persist host/target preferences in the appropriate project or application configuration;
- distinguish approximate browser path, local simulation plan, and physical accepted plan everywhere;
- use stable program/row/sample IDs for Program Tree and timeline selections rather than generated array-index IDs;
- make every stale-plan reason explicit and identity-based;
- expose delete/archive through the canonical catalog if policy permits;
- ensure visible loading/replanning copy reflects actual operations;
- render only mesh formats actually supported or add the missing loaders;
- move diagnostic dataset data out of generated component source; and
- retain direct-control diagnostics only in server-mediated, role-gated tools.

## 25. Feature checklist

For quick planning, the complete visible feature set is:

- [x] auto-connect and manual SSE connection control
- [x] live/disconnected/stale/API-offline status
- [x] LIVE/SIM hot switch with confirmation
- [x] compact drive/motion/safety/fault header
- [x] header Power Up/Power Down/Reset controls for RTCore
- [x] Home, Rest, STOP
- [x] session-selectable API and vision hosts
- [x] resizable/persisted multi-pane workstation
- [x] keyboard workspace shortcuts
- [x] lazy-loaded Three.js/URDF robot stage
- [x] multiple robot asset catalog with default fallback
- [x] raw/display joint truth preference
- [x] packet ordering/spike rejection
- [x] direct 3D pose push plus throttled React telemetry
- [x] robot bounds and orientation widget
- [x] 3D/vision stage switch
- [x] MJPEG vision feed retry/fallback
- [x] local STEP mesh import
- [x] backend STEP topology import
- [x] STEP position/rotation/scale
- [x] ground-plane trajectory point picking
- [x] live linear/joint/home waypoint capture
- [x] speed, acceleration, rotation-speed, and pause metadata
- [x] automatic/debounced planning and cancellation
- [x] exact path visualization
- [x] structured planner warnings/diagnostics/recovery details
- [x] stale/mismatched preview validation
- [x] SIM and LIVE execution gating
- [x] loop execution override
- [x] saved trajectory list/load/save/overwrite
- [x] Program Tree chronological/grouped views
- [x] Program Tree point add/remove/reorder/edit
- [x] Program Tree move-type/speed/acceleration/pause edit
- [x] estimated linear motion-profile chart
- [x] draft undo/revert/save
- [x] shared synchronized timeline
- [x] STEP topology edge hover/select/multi-select/remove
- [x] per-segment weld type and trimming
- [x] weld work/travel/spin angles
- [x] interactive angle help
- [x] clearance and post-action options
- [x] multi-edge weld/transition section construction
- [x] auto and explicit weld planning
- [x] weld ghost pose and angle/TCP overlays
- [x] pre-run weld replan and cache-backed run
- [x] saved weld list/load/save/overwrite with embedded STEP
- [x] Tool Library filtering and quick selection
- [x] full tool CRUD
- [x] TCP and independent mesh placement
- [x] active/desired/restart-required tool state
- [x] robot/runtime/RTCore policy settings
- [x] revision-checked base/tool kinematics offsets
- [x] drive power and decoded per-axis fault status
- [x] encoder retention snapshot capture
- [x] optional gripper controls
- [x] exponential speed multiplier
- [x] hold-to-jog session/deadman/debug/failsafe
- [x] discrete Cartesian/rotation moves
- [x] joint commissioning step/zero/native home
- [x] commissioning health chips
- [x] per-joint and per-servo live charts
- [x] per-axis Drive Feedback cards
- [x] active alarm grouping
- [x] persistent hide/restore chart panels
- [x] adaptive API/controller/jog/RTCore performance diagnostics
- [x] pose/IK history, manual save, and auto-save on jog stop
- [x] end-to-end 3D lag probe and console/global trace
- [x] standalone archived J6 dual-axis dataset analysis
- [x] build-time robot/tool asset synchronization

## 26. Primary implementation files reviewed

| File | Responsibility |
|---|---|
| `src/App.tsx` | Main shell, state, API/SSE, authoring drawers, settings, saved programs, synchronization |
| `src/ArmVisualizer.tsx` | Three.js scene, URDF/tool/STEP rendering, path/weld overlays, picking, camera, live pose |
| `src/ControlPanel.tsx` | Drive power/faults, motion state, gripper, jog, commissioning, Home/Rest/STOP |
| `src/components/ProgramFeatureTree.tsx` | Program Tree rendering and waypoint/move/profile editors |
| `src/components/ProgramTimeline.tsx` | Shared bottom timeline |
| `src/components/SidebarRail.tsx` | Workspace rail |
| `src/components/SidebarDrawer.tsx` | Drawer shell/scroll behavior |
| `src/previewUtils.ts` | Preview normalization, waypoint metadata, planner diagnostics, Program Tree generation |
| `src/TelemetryWorkspace.tsx` | Chart/diagnostic tabs, adaptive performance polling, pose-history persistence |
| `src/TelemetryCharts.tsx` | Joint/servo charts, drive cards, alarms, visibility persistence |
| `src/PerformanceDiagnosticsPanel.tsx` | Timing, RTCore, IK, pose, and pose-history presentation |
| `src/liveState.tsx` | Shared telemetry/motion/drive types and freshness constants |
| `src/poseTelemetry.ts` | Raw/display joint truth selection |
| `src/visualizerLagTelemetry.ts` | End-to-end 3D lag instrumentation |
| `src/J6ManualRotateDatasetPage.tsx` | Standalone archived dataset analysis UI and embedded data |
| `src/useEndpoint.ts` | Default API and vision host resolution |
| `scripts/sync-robot-assets.mjs` | Robot catalog/URDF/mesh synchronization |
| `scripts/sync-tool-assets.mjs` | Tool catalog/mesh synchronization |
| `vite.config.ts` | Two-entry build, dev host/port, public path definition |
| `README.md` and `design.md` | Intended feature summary and UI design conventions, cross-checked against code |

---

This inventory describes the inspected working tree as of 2026-07-31. Because essentially all files under `GradientOS/web-ui` are modified relative to the recorded baseline commit, future transfer work should compare against that working tree or a new committed snapshot rather than relying on commit `d47bc53b` alone.
