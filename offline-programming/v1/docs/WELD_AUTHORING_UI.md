# Weld Authoring UI — torch angles, weave, arc control, and path visualization

Status: implemented in `offline-programming/v1/ui` (2026-08-04)
Origin: ideas merged from the GradientOS web UI per
`GRADIENTOS_WEB_UI_FEATURE_INVENTORY.md` §24, projected onto the existing
Robot V4 spreadsheet program. The spreadsheet remains the single source of
truth; no parallel waypoint model was introduced.

## What was added

### Row inspector (left panel, ROW tab)
The authoring panel is now tabbed: **SETUP** (project, CAD, robot, target,
placement, weld process defaults) and **ROW** (selected-row detail). Open the
ROW tab from its tab button, the spreadsheet DETAILS button, or by
double-clicking a spreadsheet row. Sections:

- **Motion & speed** — move type, TCP speed/accel, blend, joint speed, fine
  stop, with mm/s and m/min conversions and an estimated speed-vs-distance
  profile graph (trapezoid/triangle). The graph is an authoring aid; the
  accepted Motion Server plan remains the only timing authority. When no
  acceleration is authored the estimate assumes the commanded speed is
  reached in ~1 s, matching the GradientOS controller default.
- **Weld** — explicit ARC ON / ARC OFF segmented control plus process
  (GMAW/GTAW/laser/custom), voltage, current, wire feed, weld/weave profile
  IDs.
- **Torch** — work / travel / spin angles with live cross-section and
  side-view diagrams.
- **Weave** — pattern (none/zigzag/circular/trapezoidal), amplitude,
  frequency, dwell, with the resulting wavelength computed from travel speed.
- **Apply to pass** — copies speed, arc, process, torch, and weave settings
  to every row sharing the seam and phase (or to all arc-on rows for
  non-CAD rows).

### Spreadsheet
- The Weld checkbox column is now an **ARC ON / ARC OFF** toggle that keeps
  `weldEnabled`, `arcState`, and the RevPi weld I/O action consistent
  (`setRowArc` in `src/domain/weld.ts`).
- Advanced rows expose the torch angles and weave fields directly.

### Weld process defaults (SETUP tab)
Edits `processParameters[0]` — travel/approach/retract speeds, voltage,
current, wire feed, torch angles, and weave defaults. Changing them
regenerates CAD-derived rows, exactly like seam/placement edits.

### 3D path visualization (`src/viewer`)
- Direction cones along the path (strided, ≤24) colored travel/weld.
- Weld run start (green) / stop (red) markers.
- Row-number label sprites rendered at constant screen size
  (`sizeAttenuation: false`) and kept deliberately sparse: with no selection
  only weld run boundaries and program endpoints are labelled (all rows for
  programs ≤12 targets); a stage-toolbar **Labels** toggle hides them
  entirely.
- **Focus mode**: selecting a spreadsheet row fades the base path and its
  decorations to ~30% and renders the selected move at full strength — white
  target marker, halo, a tube along the incoming segment, the full weld run
  polyline, labels for prev/self/next and the run boundaries, and only the
  selected row's TCP frame.
- Torch-angle guide at the selected arc-on row: cyan = travel, blue = up,
  green = normal, orange = torch.
- Weave ribbon preview along the selected row's weld run.

## Conventions

### Torch angles (`torchGuideVectors` in `src/domain/weld.ts`)
The rotation sequence matches the GradientOS controller
(`_build_weld_orientations`) and its web UI preview:

1. torch direction starts straight down (work angle 0 = vertical torch);
2. **work angle** rotates about the horizontal-projected travel axis,
   tilting the torch across the seam toward the normal side;
3. **travel angle** rotates about the normal axis (up × travel); positive
   tips the torch direction backward so the torch body leans toward travel —
   physically **positive = drag/backhand, negative = push**, measured along
   the seam travel direction (convention confirmed 2026-08-04);
4. **spin** swings the tilted direction azimuthally about world up.

### Weave
GradientOS has no weave implementation; this is new authoring intent. The
preview ribbon offsets laterally across the seam with wavelength
`travel speed / frequency`. `weaveDwellMs` is stored and round-tripped but
not yet rendered or executed — planner support is pending.

### Persistence and wire compatibility
The new typed row fields (`torchWorkAngleDeg`, `torchTravelAngleDeg`,
`torchSpinAngleDeg`, `weavePattern`, `weaveAmplitudeMm`, `weaveFrequencyHz`,
`weaveDwellMs`) are packed into each motion row's `extensions` under
`gradient_weld_v1` (snake_case) by `toRobotV4Row` and unpacked by
`fromRobotV4Row`. The shared `robot.v4.program.v1` schema is unchanged, so
SteamDeck/Motion Server peers that ignore extensions are unaffected. Legacy
projects hydrate with safe defaults (`serialization.ts`).

**Known limitation:** torch angles and weave are authoring intent +
visualization today. They are not yet consumed by Tesseract planning
(row orientation is still authored via `targetTcpPose` RPY and the program's
`tool_axis_constraint`) or by physical weld execution. Wiring
`gradient_weld_v1` into the Motion Server planner is the follow-up step.

## Tests
`test/weld.test.ts` covers torch vector conventions, weave envelope/disable
behavior, lateral profile bounds, the Robot V4 extension round trip, and the
motion-profile trapezoid/triangle selection. Run with `npm test`.
