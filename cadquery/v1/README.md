# CadQuery v1

CadQuery and OpenCascade in a pixi Python environment, plus the weld-seam
authoring UI built on it. OLP's STEP tessellation worker
(`offline-programming/v1/internal/cad`) runs inside this pixi environment.

## Running it

```shell
cd cadquery/v1 && pixi install          # the CadQuery Python environment
git lfs pull --include "cadquery/v1/step/*"
cd ui && npm ci && npm run dev -- --port 5184
```

Open the seam route at `http://127.0.0.1:5184/cadquery/v1/ui/seam` and the
focused edge-geometry tool at `http://127.0.0.1:5184/cadquery/v1/ui/edge-geometry`.
The end-to-end Dog Box tour proof runs from `cadquery/v1/ui` with
`npm run test:tour` against the running dev server.

## CadQuery Weld Seam UI

`cadquery/v1/ui` includes a dedicated Vite/TypeScript seam extraction page for turning exact CadQuery STEP edges into weld seam candidates.

### Tour Requirements

The weld path planning tour must show seam extraction, path generation with a
45-degree out-of-seam-plane torch constraint, and the `rosie_1400_v3` moving its
TCP along that path, with tour text explaining each step:

- The page title/hero is `WELD SEQUENCE`.
- Both weld pages render the same shared single-log HUD from `cadquery/v1/ui/src/cadquery-shared.ts` and `cadquery-shared.css`: a skewed hero title, one `Tour Test` button, and one fading bash-style report log. No other 2D panels are visible.
- The 2D UI is greyscale end to end. Color is reserved for the 3D coordinate-frame gizmos and billboard labels: red tool `+X`, green tool `+Y`, blue tool `+Z`.
- The fading log owns the left third of the screen: new entries append at the top, older entries fade toward the bottom, the log fills the full remaining vertical space below `Tour Test`, and entries persist across both apps via `sessionStorage` (up to 96 lines).
- The Three.js projection center is offset so the orbit target renders at 2/3 screen width, keeping the 3D subject centered in the right two-thirds beside the log.
- The page uses a full-screen Three.js stage with a Z-up CAD frame and XY floor.
- The tour must load the checked-in `cadquery/v1/step/dog_box.step` example.
- The Dog Box bottom-center must be placed on the center-top of `rosie_1400_v3` `positioner_table_a` / axis `J7a`, not on the table edge or the J7a pivot.
- The CadQuery Python backend owns topology and seam geometry. TypeScript is only the UI/client/visualization layer.
- Backend topology extraction must return exact sampled CAD edges with deterministic IDs such as `part_4:edge_00084`.
- Backend topology extraction must group STEP edge fragments by face-boundary wires, not by whole-solid connected components, so dog-bone outlines become continuous closed `WIRE` edges with mixed `LINE`/`CIRCLE` source geometry. Selectable edges must use spreadsheet-style labels (`A`..`Z`, `AA`...) and list these composite wires ahead of raw fragments.
- Backend topology extraction must classify real Dog Box seam paths explicitly: a real seam is either a standalone closed circle at weld scale or a closed dog-bone `WIRE` made from exactly `4` arc/circle segments plus `4` line segments. Line-only box loops, 2-arc/2-line face fragments, small circle artifacts, and raw STEP fragments stay available in topology JSON but are not seam candidates.
- The current Dog Box STEP should expose `10` real seam candidates from CadQuery: `6` dog-bone `4x Arc + 4x Line` wires plus `4` large circular paths, excluding the plain box edges.
- Backend seam extraction must return JSON with `tool = "rosie cadquery v1 seam backend"` and `workflow.computed_by = "python/cadquery"`.
- Backend path generation must compute a seam-plane normal from the sampled seam geometry and emit 6DOF TCP poses with `seam_plane_normal`, `tool_x_axis`, `tool_y_axis`, `tool_z_axis`, and `angle_from_seam_plane_deg`.
- The welding torch frame convention is: tool `+X` is forward and tangent to the seam/travel direction, tool `+Z` is the torch axis, and tool `+Y` completes the right-handed TCP frame.
- The welding torch orientation must start from the plane-orthogonal seam normal and tilt to a 45-degree tool `+Z` axis out of the seam plane while preserving tool `+X` as the seam tangent.
- Every backend TCP pose must include `tool_x_tangent_alignment`, and the tour proof should require it to stay near `1.0` so linear seams and circular/curved seams keep tool `+X` aligned with the seam tangent.
- Backend geometry planning must evaluate candidate torch frames around the seam plane and expose the selected `geometry_method` plus candidate scores so the edge-geometry tool can iterate on "optimal" path selection before robot kinematics are involved.
- The backend Tesseract environment must include a `dog_box_workpiece` collision object attached to `positioner_table_a`, plus TCP/tool constraints such as `tool_x_axis = locked_to_seam_tangent` and `tool_z_axis = 45deg_out_of_seam_plane_away_from_workpiece`.
- Clicking any seam row in the UI must call the CadQuery backend and regenerate a Tesseract-style TCP path for that seam with the active constraints: tool `+X` tangent to the seam, tool `+Z` 45 degrees out of the seam plane, and Dog Box collision data attached to positioner table A.
- The tour should follow the Tesseract Cartesian planning pattern for dense industrial toolpaths: create dense Cartesian targets, use Descartes-style graph search for waypoint reachability/IK continuity, then TrajOpt-style smoothing/collision refinement.
- Tesseract orientation constraints should be treated as Cartesian constraints/costs over `[x, y, z, rx, ry, rz]`; the weld path should keep position and work-angle/tool-axis constraints tight while allowing only process-safe redundancy such as tool-axis spin or positioner/auxiliary-axis motion when appropriate.
- Dense path generation should target enough waypoints for welding-quality visualization/planning; the current Dog Box tour requests 96 backend-sampled TCP poses per seam.
- The tour must walk the mission end to end: load Dog Box CAD, scan exact edges, pick the seam source, trim direction/endpoints, capture seam JSON, and narrate each section through `$ subtitle` entries in the shared log.
- The final state must leave the Dog Box seam visible on the model with greyscale start/end markers, a travel-direction arrow, and the seam path overlay; axis color is reserved for the TCP coordinate-frame gizmos.
- The final state must animate a weld torch/arrow along the active seam over time, using the backend-generated 6DOF TCP tool axis rather than a frontend-only approximation.
- The scene must load the `rosie_1400_v3` URDF/cell assets as a translucent robot context overlay.
- The backend must produce a longer multi-seam weld sequence for the Dog Box and emit a Tesseract-style motion plan against the `rosie_1400_v3` URDF/world frame.
- The scene must plot Tesseract TCP 6-DOF gizmos/path points as a planning context overlay for that sequence.
- The final state must include a `Captured weld seams` list and keep the final seam row highlighted.
- The final current Dog Box proof captures multi-seam Tesseract TCP poses, while the backend Dog Box topology exposes 10 real seam candidates for the tour and Edge Geometry tool.
- The proof command must generate an MP4 and per-step screenshots under `cadquery/v1/ui/test/screenshots/`.

### Multi-App Tour Loop

The `Tour Test` button on either page runs one continuous cross-app loop:

1. `Weld Sequence` loads Dog Box, selects and captures the canonical `part_3:wire_00034` seam, animates the torch, then navigates to `/edge-geometry?loop=1`.
2. `Edge Geometry` runs the backend coordinate-frame tour, toggles to the alternate `+X Mode` frame and runs that tour, then navigates back to `/seam?loop=1`.
3. The `?loop=1` URL parameter keeps the cycle running unattended; each section pushes a `$ subtitle` entry into the shared log so the narration reads like a bash transcript.

Tour camera moves interpolate on an elliptical path around the orbit target (radius/angle blend in XY with monotonic Z) so arrivals are smooth without bounce. The Puppeteer proof (`seam_tour_test.ts`) clicks `Tour Test` on the seam page and uses `?proof=multi-app` on Edge Geometry to test each page tour deterministically without the loop navigation.

### Edge Geometry Tool

`/cadquery/v1/ui/edge-geometry` is a focused math/debug tool for one job: click a Dog Box edge and verify the backend can return a dense constrained 6DOF path for that edge.

Unlike the Weld Sequence page, this page re-anchors the Dog Box at the world origin (bottom center at `(0, 0, 0)`): backend seam/TCP world coordinates are shifted by the table placement so paths, gizmos, frame labels, and the torch animation follow the box. The Weld Sequence page keeps the box on `positioner_table_a` in the robot cell.

The tool must:

- Load Dog Box topology from the same CadQuery backend as the main seam tour.
- Let the user click/select an edge from the ranked edge list.
- Request a dense backend path with `seam_sample_count = 128`.
- Let the user pick a returned geometry strategy/candidate and replan the same edge with that strategy.
- Provide a strategy-tour loop that cycles through multiple edges and multiple geometry strategies so their scores, tangent alignment, and 45-degree angle behavior can be compared.
- Validate every returned TCP pose for:
  - tool `+X` tangent alignment near `1.0`;
  - torch angle between `35` and `55` degrees out of the seam plane;
  - selected backend `geometry_method` and its candidate score list;
  - `dog_box_workpiece` collision object present in the Tesseract environment;
  - `tool_x_axis = locked_to_seam_tangent` in the backend constraint metadata.
- Draw the edge/path and TCP 6DOF gizmos where red is tool `+X`, green is tool `+Y`, and blue is tool `+Z`.
- Report an obvious `PASS`/`FAIL` before this geometry is trusted by the larger weld-sequence tour.

### Runtime Route

The seam page is intentionally separate from the existing object library and parts image pages. It is a clean operator HUD over a full-screen Three.js stage: hero title top-left, one `Tour Test` button, and a single fading bash-style log carrying every status, validation, and subtitle line. The legacy panels (edge lists, sliders, seam cards, JSON output) still exist in hidden runtime DOM so the tours and Puppeteer proof can drive them, but they are not visible to the operator.

The stage is Z-up end to end: raw CadQuery coordinates keep `Z` as vertical, the floor grid is the XY plane, and the Three.js camera/orbit controls use `camera.up = (0, 0, 1)`.

The selected seam renders with greyscale operator cues: start/end markers, a travel-direction arrow, and a tube/line path overlay. The animated torch and TCP gizmos carry the only color (red `+X`, green `+Y`, blue `+Z`), so screenshots stay readable while the coordinate frames stay obvious.

The manual workflow is:

1. Choose a `.step` or `.stp` file in the seam page.
2. Set the edge sample count, defaulting to 64 samples per edge.
3. Click `Extract CadQuery Edges`.
4. The Vite dev server handles `POST /api/cadquery/topology`, runs `pixi run python python/extract_weld_edges.py --stdin` from `cadquery/v1`, and returns sampled exact edge topology from CadQuery/OpenCascade.
5. Pick an edge directly in the full-screen Three.js stage or from the sorted edge list.
6. Trim the seam with normalized start/end arclength sliders.
7. Use `Reverse Direction` when travel direction needs to flip.
8. Click `Add Seam` to store the orange segment in the seam set.
9. Copy or download the generated seam JSON.

The Dog Box tour uses the checked-in `cadquery/v1/step/dog_box.step` asset as a ready-made example. Click `Tour Test`, or open the looping variant directly:

```text
http://127.0.0.1:5184/cadquery/v1/ui/seam?loop=1
```

The tour loads Dog Box through the same CadQuery backend, extracts exact topology, ranks only real seam paths, captures the canonical seam, animates the torch, and then hands off to the Edge Geometry frame tours before looping back.

The Python backend script is `cadquery/v1/python/extract_weld_edges.py`. It imports the STEP with CadQuery, walks solids and edges, samples each edge through OpenCascade curve adaptors when available, deduplicates repeated topology edges, and returns JSON with:

- `model_id`, `filename`, `fingerprint`, `unit`, and `sample_count`
- `parts[]` with per-part edge counts
- `edges[]` with deterministic IDs like `part_0:edge_00000`
- each edge's `geom_type`, `samples`, `length_mm`, and bounds

The TypeScript frontend is `cadquery/v1/ui/src/seam.ts` with styling in `cadquery/v1/ui/src/seam.css`; the HTML entrypoint is `cadquery/v1/ui/seam.html`. The dev API and `/seam` route are registered in `cadquery/v1/ui/vite.config.ts`.

Code shared by both weld pages lives in `cadquery/v1/ui/src/cadquery-shared.ts` and `cadquery-shared.css`:

- `renderSingleLogHud` - the hero title / `Tour Test` / fading-log shell, plus hidden runtime DOM.
- `appendReport` - top-appending bash-style log entries persisted across app navigations.
- `createCadqueryStage` - scene, camera (with the 2/3 view offset), renderer, controls, grid, lights, and resize.
- `animateCameraElliptical` - smooth elliptical camera interpolation for tour steps.
- `createFrameConceptGroup` / `createBillboardLabel` - the shared 3D coordinate-frame label system.
- `loadDogBoxTopology` / `buildDogBoxSeamPayload` - the Dog Box backend API calls.
- `rankedSeamEdges` / `compactEdgeTitle` / `compactEdgeMeta` - shared edge-data ranking and labels.
- `createConfiguredUrdfLoader` - the URDF/Collada mesh loading helper used for the `rosie_1400_v3` overlay.

The seam JSON output is measured in millimeters and contains the selected source edge, normalized `start_s` / `end_s`, direction, source model fingerprint, `workflow.source = "cadquery"`, and the trimmed weld points. The frontend does not compute the weld seam geometry itself; it sends the selected edge and trim values back to the CadQuery Python backend, which reloads the STEP, samples the selected edge segment, and returns `tool = "rosie cadquery v1 seam backend"` with `workflow.computed_by = "python/cadquery"`. The TypeScript UI is a client for loading, choosing, and visualizing backend-computed topology and seam data. This UI does not plan robot motion yet; it is a CAD edge selection and weld seam authoring surface on top of CadQuery.

The end-to-end browser proof lives in `cadquery/v1/ui/scripts/seam_tour_test.ts` and uses Puppeteer TypeScript against the live UI. The preferred command is:

```shell
cd cadquery/v1/ui && npm run test:tour
```

The proof opens `/cadquery/v1/ui/seam`, records an MP4 while it clicks `Tour Test`, waits for the canonical seam to be selected/captured/animated, then drives both Edge Geometry frame tours in `?proof=multi-app` mode and asserts seam state, TCP constraints, billboard label counts, torch motion, and a clean browser console. It writes screenshots, state, and video:

- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour-step-01-weld-sequence-final.png`
- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour-step-02-edge-geometry-backend-frame.png`
- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour-step-03-edge-geometry-x-mode-frame.png`
- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour-step-04-multi-app-tour-final.png`
- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour.mp4`
- `cadquery/v1/ui/test/screenshots/cadquery-seam-tour-state.json`


Current Dog Box topology result: 90 raw CadQuery edges, 38 composite face wires, and 10 real seam candidates: 6 closed dog-bone `4x Arc + 4x Line` wires plus 4 large standalone circular paths.

## Python tools

Run from `cadquery/v1`:

| Task | Purpose |
| --- | --- |
| `pixi run python python/extract_weld_edges.py --stdin` | The seam page's topology backend (the dev server spawns it) |
| `pixi run convert --step <file> --out <dir> [--split]` | STEP to STL meshes plus a fixed-joint URDF |
| `pixi run parts-image` | STEP-derived technical line-drawing PNGs and `parts/index.json` for the `/parts` page |
| `pixi run analyze-mount-faces` | Likely bolt-pattern mounting faces in a STEP |
| `pixi run smoke` | Converter argument and sanitizer smoke checks |
| `pixi run lint` | `ruff check python` |

STEP files do not carry a robot joint graph, so `convert` produces fixed-joint
URDFs only; inspect mesh scale and frames before using its output.

## STEP assets

Reusable STEP/STP assets live in `cadquery/v1/step/` and are tracked with Git LFS. The current checked-in asset is `dog_box.step`, with LFS oid:

```text
sha256:aac02178a17f60d410445fe2049ea80a5d838359f226572a7fe5edfcd2062309
size 30194
```

`.gitattributes` marks `*.step`, `*.stp`, `*.STEP`, and `*.STP` as LFS files so large CAD inputs do not bloat normal repo syncs.
