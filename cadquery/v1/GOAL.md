# CadQuery v1 Goal

## Goal

Make `cadquery/v1` a focused Geometry workflow for turning the Dog Box CAD model
into weldable seam paths, explaining the generated TCP coordinate frames, and
proving the workflow end to end across both UI tools:

- `cadquery/v1/ui/seam` (`Weld Sequence`)
- `cadquery/v1/ui/edge-geometry` (`Edge Geometry`)

The next implementation pass should treat this file as the source of truth.

## User Experience Target

The UI should feel like a clean operator HUD over a full-screen 3D stage:

- black/dark-gray stage background;
- simple top-left text overlay;
- obvious but minimal buttons;
- no large side-card panels or data-heavy dashboards;
- bash-like text reports instead of dense metric widgets;
- a fading log for status/report output as users click buttons;
- closed-caption tour text at the bottom of the screen;
- 3D labels and leader lines for geometry concepts in the scene.

Use color sparingly. Default UI text should be light gray/cream. Use red or
other colors only for key action highlights, selected seams, coordinate axes,
tour emphasis, and 3D path/frame concepts.

## Current State

### Backend

`cadquery/v1/python/extract_weld_edges.py` owns topology and path generation.

Current Dog Box extraction rules:

- real seam paths are either:
  - standalone closed circles at weld scale;
  - closed dog-bone wires made from exactly `4` arc/circle segments plus `4`
    line segments;
- line-only box loops, small arc fragments, and arbitrary raw STEP fragments
  remain in topology JSON but are not seam candidates;
- current Dog Box topology should expose `10` seam candidates:
  - `6` dog-bone `4x Arc + 4x Line` wires;
  - `4` large circular paths.

Backend TCP frame convention today:

- tool `+X` follows seam tangent/travel;
- tool `+Z` is the torch axis, 45 degrees out of the seam plane;
- tool `+Y` completes the right-handed frame.

Edge Geometry also visualizes an alternate `+X Mode` frame:

- alternate `+Y` follows seam tangent/travel;
- alternate `+X` is the torch axis, chosen to point into the Dog Box/workpiece;
- alternate `+Z` completes the right-handed frame.

### Weld Sequence UI

`cadquery/v1/ui/src/seam.ts` and `seam.css` currently:

- render the seam authoring workflow as a lightweight text overlay;
- auto-start a tour on page load;
- load the Dog Box example;
- select the canonical dog-bone seam `part_3:wire_00034`;
- capture it into `Captured weld seams`;
- end on the requested camera view;
- animate a weld torch along the backend path;
- show a live TCP gizmo at the animated blue contact point;
- use closed captions instead of a tour panel.

### Edge Geometry UI

`cadquery/v1/ui/src/edge-geometry.ts` and `edge-geometry.css` currently:

- render a focused geometry sandbox;
- load Dog Box;
- select real seam candidates;
- plan dense backend TCP poses;
- run a coordinate-frame tour;
- show 3D billboard labels and leader lines for frame concepts;
- show the weld torch animation and live TCP frame gizmo;
- support backend frame mode and alternate `+X Mode`.

## Tesseract Motion Plan On The Dog Box

Bring the `robot/v4/ui` planned-path pattern to the Weld Sequence tour: the tour
must end by playing a Tesseract-style motion plan on the Dog Box, not just a
looping seam trace.

Reference implementation: `robot/v4/ui/src/plannedPath.ts` renders the
motion-server plan (`motion_server_tesseract_trajectory_v1`) as a dense
time-parameterized trajectory where every sample carries `time_s`, a 6DOF
`tcp_pose`, and a `weld_on` flag; weld-on runs draw hot orange, travel runs draw
neutral gray, and TCP frame gizmos mark sampled poses.

### Backend Requirements

`extract_weld_edges.py` must emit a dense time-parameterized `trajectory` inside
`tesseract_plan` for both single-seam (`build_seam`) and multi-seam
(`build_sequence`) operations:

- every trajectory sample has `index`, `seam_id`, `move`
  (`approach` / `weld` / `retract` / `travel`), `weld_on`, `time_s`,
  `target_xyz_mm`, full tool axes, and `orientation_rpy_rad`;
- time parameterization is speed-based, not per-sample constants: weld moves run
  at a weld feed rate (mm/s) and approach/retract/travel moves run at a faster
  travel rate, exactly like the v4 daemon derives `segment_s` from TCP distance
  over `tcp_velocity_m_s`;
- each seam gets `approach -> weld -> retract` moves, and multi-seam sequences
  insert `travel` moves between a retract and the next approach;
- the plan reports motion metrics: `duration_s`, `tool_path_length_mm`,
  `weld_length_mm`, `travel_length_mm`, `sample_count`, `weld_sample_count`,
  `max_tcp_step_mm`, and `max_frame_rotation_deg`.

### Smooth Coordinate Frames

The dog-bone path has line-to-arc corners; per-sample frames must stay smooth so
the torch does not snap while tracing the final path:

- seam tangents are smoothed with a windowed average (closed paths wrap around)
  before torch frames are computed;
- consecutive TCP poses expose a frame-continuity diagnostic
  (`max_frame_rotation_deg` between neighboring poses);
- geometry-method candidates in the Edge Geometry strategy list carry their own
  `max_frame_rotation_deg` smoothness score, and the candidate score penalizes
  frame discontinuity so "optimal" strategies are also smooth strategies;
- the constraints stay intact: tool `+X` tangent alignment near `1.0` and the
  torch `+Z` at 45 degrees out of the seam plane.

### URDF Robot Joint Animation

The rosie_1400_v3 URDF in the Weld Sequence scene must animate through the
motion plan, not sit frozen while a floating torch moves:

- the backend solves IK for every trajectory sample (damped-least-squares over
  the URDF `J1..J6` chain, `end_effector` TCP) and emits `joint_names` +
  `joint_positions_rad` per sample plus an `ik` summary
  (`max_position_error_mm`, `max_rotation_error_deg`, `max_joint_step_rad`);
- the orientation constraint is the torch axis (tool `+Z`) only — spin about a
  symmetric MIG torch is free, which keeps the whole dog-bone seam reachable;
- IK position error stays sub-millimeter and consecutive solutions stay
  continuous (warm-started from the previous sample);
- during playback the UI drives the URDF joints with lerped backend joint
  values so the whole arm plays the plan in step with the torch.

### Weld Sequence Playback Requirements

- The Tesseract path overlay renders weld-on runs and travel runs separately
  (orange weld emphasis vs neutral travel), with TCP gizmos as before.
- The final tour state plays the motion plan through time: torch position lerps
  between trajectory samples by `time_s`, and torch orientation slerps between
  full backend TCP quaternions (no nearest-pose snapping).
- The animated torch follows approach and retract moves as well as the weld
  path, and the live TCP gizmo follows the interpolated backend frame.
- The URDF robot arm follows the backend joint trajectory during playback.
- The shared log reports the motion plan bash-style, e.g.
  `$ tesseract motion` /
  `samples=204 duration=64.2s weld=96 travel=108 frame_rot_max=4.8deg` /
  `ik=damped_least_squares joints=J1,...,J6 pos_err_max=0.020mm`.

## Required Multi-App Onboarding Tour

Create one coherent onboarding story that spans both apps.

The tour should be able to start in `Weld Sequence`, demonstrate the weld seam
workflow, then move into `Edge Geometry` to explain coordinate frames, strategies,
and alternate frame modes. It should not rely on a large tour panel. It should
use:

- in-scene highlights;
- 3D billboard labels;
- leader lines;
- bottom closed captions;
- smooth camera interpolation between tour positions;
- a fading bash-like report log for status output.

### Tour Sequence

1. **Weld Sequence: load Dog Box**
   - Load checked-in `cadquery/v1/step/dog_box.step`.
   - Show a fading report line:
     - `cadquery topology: dog_box.step`
     - `raw_edges=90 composite_wires=38 seam_candidates=10`

2. **Weld Sequence: pick canonical seam**
   - Select `part_3:wire_00034`.
   - Explain this seam is a closed dog-bone wire:
     - `4 arcs + 4 lines`
     - `geom_type=WIRE`
     - `path_method=sampled_edge`

3. **Weld Sequence: show weld path**
   - Show orange seam path and sampled TCP dots.
   - Show animated weld torch.
   - Show live TCP gizmo at torch contact point.
   - End with the seam captured in `Captured weld seams`.

4. **Transition to Edge Geometry**
   - Navigate to `/cadquery/v1/ui/edge-geometry`.
   - Keep context: Dog Box, canonical seam, backend-generated TCP poses.
   - If the app reloads, load Dog Box automatically before continuing.

5. **Edge Geometry: backend frame explanation**
   - Show 3D billboard labels and leader lines:
     - `SEAM TANGENT`
     - `TOOL +X`
     - `SEAM PLANE NORMAL`
     - `SEAM SIDE`
     - `TOOL +Z / TORCH`
     - `TOOL +Y`
   - Closed captions should explain:
     - `+X` follows travel;
     - `+Z` is the torch axis at 45 degrees out of the seam plane;
     - `+Y` completes the right-handed frame.

6. **Edge Geometry: alternate +X Mode explanation**
   - Toggle to `Frame: +X Mode`.
   - Show 3D billboard labels and leader lines:
     - `+Y TRAVEL`
     - `+X TORCH`
     - `+Z RIGHT-HANDED`
   - Closed captions should explain:
     - alternate `+Y` follows travel;
     - alternate `+X` aligns with the torch and points into the Dog Box;
     - alternate `+Z` completes the frame.

7. **Final state**
   - End on the Edge Geometry frame-tour view.
   - The selected seam should be the canonical Dog Box dog-bone seam.
   - The weld torch should be animated.
   - 3D frame labels should be visible.
   - Closed caption should summarize which frame convention is active.

## Shared UI Requirements

Both `Weld Sequence` and `Edge Geometry` should share a visual system:

- title-first overlay in the top-left;
- navigation buttons directly near/under the title;
- simple text and button controls;
- no unnecessary cards or heavy bordered panels;
- default text in light gray/cream;
- red only for action accents, selected/highlighted states, and red coordinate
  axes;
- 3D path/axis colors:
  - red: tool `+X` or alternate `+X`;
  - green: tool `+Y` or alternate `+Y`;
  - blue: tool `+Z` or alternate `+Z`;
  - yellow/orange: weld seam/path emphasis;
  - cyan: CAD topology outlines.

### Bash-Like Reports

Replace dense metric panels with a flowing log:

```text
$ cadquery topology dog_box.step
model=cadquery-step-aac02178 parts=6 raw_edges=90 composite_wires=38 seam_candidates=10

$ seam select part_3:wire_00034
class=dogbone_4arc_4line length_mm=821.129 path_method=sampled_edge

$ tcp plan --poses 96 --frame backend
tool_x_tangent_min=1.000000 angle=45.00deg collision=dog_box_workpiece
```

The report log should fade older entries instead of trying to keep all data on
screen at once.

## Testing And Proof

Use Puppeteer TypeScript proof tests for the multi-app tour.

The proof should:

- launch the Vite UI;
- open `/cadquery/v1/ui/seam`;
- monitor browser `console`, `pageerror`, and failed network requests;
- run the Weld Sequence auto-tour;
- assert Dog Box loaded and `part_3:wire_00034` is selected/captured;
- navigate to `/cadquery/v1/ui/edge-geometry`;
- run the coordinate-frame tour;
- toggle `Frame: +X Mode`;
- assert 3D billboard labels exist for backend and alternate frame concepts;
- assert the weld torch animation exists and moves;
- assert closed captions are visible at the bottom of the screen;
- capture screenshots for each major step;
- record an MP4 for review.

Preferred command:

```shell
./rosie.sh cadquery v1 test --filter ui-tour
```

PowerShell equivalent:

```powershell
.\rosie.ps1 cadquery v1 test --filter ui-tour
```

Artifacts should remain under:

```text
cadquery/v1/ui/test/screenshots/
```

## Acceptance Criteria

- Opening `Weld Sequence` starts the tour automatically.
- The tour ends on a correct Dog Box dog-bone seam and weld torch animation.
- The seam is captured in `Captured weld seams`.
- The tour can transition to `Edge Geometry` and explain coordinate frames.
- Coordinate-frame labels are 3D billboards, not DOM overlays.
- The weld torch and live TCP gizmo are present in both apps where relevant.
- `Frame: +X Mode` shows alternate `+Y` travel, `+X` torch, and `+Z`
  right-handed frame.
- Closed captions are the only tour narration UI.
- UI reports are bash-like fading text, not dense static dashboards.
- Puppeteer proof passes with no browser console errors, no `pageerror`, and no
  failed requests.

## Implementation Notes

- Keep backend geometry authoritative. Do not recompute seam paths in TypeScript.
- TypeScript may transform/visualize backend vectors, but it should not invent
  seam geometry.
- Keep the canonical Dog Box seam and camera view stable unless the user chooses
  a new reference view.
- Prefer small focused tests around the tour proof instead of broad UI rewrites.
- Keep old generated artifacts out of commits unless the user explicitly asks for
  a deploy/build artifact refresh.
