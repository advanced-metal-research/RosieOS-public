import { hasWorkpieceGeometry } from "../domain/placement.ts";
import { effectiveNodes } from "../program-v2/nodes.ts";
import { hasMotionNodes, measuredStartUnsupported, planRequestBody as packPlanRequest, plannedNodes } from "./planRequest.ts";
import { PLANNED_FROM_PROGRAM_START } from "../execution/runControls.ts";
/**
 * Rails mode: the weld planner's seam model, inside this window.
 *
 * A PROGRAM state, not a session toggle (`project.railsAuthoring`, chosen at
 * New program): a seam owned by both authoring flows would be welded twice,
 * so switching mid-program is allowed only while the other flow has authored
 * nothing. With rails on, clicking a part edge (or clicking near a
 * joint line) resolves it to a detected seam and runs the angle search on it
 * automatically — the result lands in the WELDS panel as a list entry, not
 * as a popup. Clicking the entry opens its detail: the weld planner's own
 * spline display for work and travel angle (ported verbatim), the clearance
 * verdict as a coverage strip, and the two decisions — trim to the longest
 * clear run, or accept. The rail and the comb draw over the part in the
 * viewport, posed by the workpiece placement.
 */

import * as THREE from "three";

import { catalogEdgeById } from "../domain/edge-catalog";
import type { OfflineProgrammingProject } from "../domain/project";
import type { WorkcellFixtureWire } from "../domain/workcell.ts";
import { projectWorkpiecePlacement } from "../viewer/workstation";
import * as client from "./client";
import { createPresetLibrary, type PresetStore } from "../parameters/library.ts";
import { createParameterView, type ParameterView } from "../parameters/view.ts";
import { createParameterDialog, type ParameterDialog } from "../parameters/dialog.ts";
import type { ParameterPreset, ParameterValue } from "../parameters/model.ts";
import { format as formatParameter } from "../parameters/view.ts";
import { NumberField } from "../ui/fields.ts";
import { iconElement } from "../ui/icons.ts";
import { planRefusalFrom, planRefusalText } from "../ui/progress-dialog.ts";
import { X } from "lucide";
import {
  SEARCH_SCHEMA,
  SEARCH_SCOPE,
  SHIPPED_SEARCH_PRESETS,
  SHIPPED_TORCH_PRESETS,
  TORCH_SCHEMA,
  TORCH_SCOPE,
  torchProblems,
  torchRequest,
} from "./parameters.ts";
import { toProgramV2Document } from "../program-v2/from-project.ts";
import { lowerManualPoseWelds } from "../program-v2/manual-pose-seam.ts";
import { isManualPoseWeld } from "../waypoints/node.ts";
import { computeProgramV2Digest } from "../contracts/robot-v4-program-v2.ts";
import type {
  RailsClearance,
  RailsJoint,
  RailsJointsDoc,
  RailsOptimisePlan,
  RailsSeam,
  RailsWeldPayload,
} from "./client";
import type { FreeSpaceBackend, RailsDenseSummary } from "./client";
import { narrowsToRun } from "./decisions";
import { decideWeld, measureRunPiece, runIndexAt, runPiecePayload, searchSeam, type SeamPlan } from "./authoring.ts";
import { el, replace } from "./dom";
import type { WeldNode } from "../contracts/robot-v4-program-v2.ts";
import { isRailsWeld, railsPayloadOf, railsProvenance } from "../program-v2/rails-node.ts";
import { buildRailsOverlay, disposeOverlay, RAILS_AMBER, tickOverlayFlow } from "./overlay";
import type { AnglePath } from "./overlay";
import { evaluate } from "./scalar-function";
import type { ScalarFunction } from "./scalar-function";
import { createSplineEditor } from "./spline-editor";
import { fractionAt, summarise } from "./span-math";
import type { CoverageSummary, Span } from "./span-math";

/** How near an edge must pass a joint's curve to mean it, metres. */
const MATCH_TOLERANCE_M = 0.003;
/** A surface point is a coarser pointer than an edge, so it gets more
 *  slack: a click beside the line is an explicit "search the seam near here", landing
 *  on a FACE beside the seam — a plate face point sits a plate-width from
 *  the junction line, so the radius is generous and the nearest joint wins. */
const POINT_TOLERANCE_M = 0.025;

export type RailsEntryState =
  | "searching"
  | "proposed"
  | "failed"
  | "accepted"
  | "trimmed";

/** One rails weld as the panel lists it: a seam, the search's answer for it,
 *  and what the operator decided. */
export interface RailsEntry {
  id: string;
  seam: RailsSeam;
  joint: RailsJoint;
  state: RailsEntryState;
  /** The searched stage or the failure, for the list row. */
  note: string;
  span: Span;
  plan: RailsOptimisePlan | null;
  clearance: RailsClearance | null;
  summary: CoverageSummary | null;
  payload: RailsWeldPayload | null;
  overlay: THREE.Group | null;
}

export interface RailsSource {
  kind: string;
  filename?: string;
  bytes?: Uint8Array;
}

export interface RailsDeps {
  requiresMeasuredStart?(): boolean;
  /** The operator plans for a repeat (Run N times / Loop): the plan ends
   *  with the planner's return to its start (planRequest.ts withLoopReturn). */
  closeLoop?(): boolean;
  /** The overlays' parent — the viewer's work-frame follower, so guides,
   *  rails and combs turn with the part on the positioner. */
  parent: THREE.Object3D;
  getProject: () => OfflineProgrammingProject;
  getSource: () => RailsSource | null;
  /** The workcell fixtures as the planner receives them; throws
   *  `WorkcellFixtureUnavailable` for a STEP fixture not yet tessellated. */
  getFixtures: () => WorkcellFixtureWire[];
  /** One line of narration — lands wherever the host puts CAD status.
   *  `detail` is kept on the Logs pane entry (a refused plan's raw object). */
  status: (text: string, isError?: boolean, detail?: unknown) => void;
  /** The welds panel wants re-rendering — entry state changed. */
  requestPanelRender: () => void;
  /** A compiled robot.v4 program (wire form) — install its rows into the
   *  open project. The host owns the project, so it owns the merge. */
  /** Keep what a plan produced. There are no rows to install any more: what
   *  comes back is the planner's own document and the trajectory the robot
   *  plays, and the OLP holds both because the motion server's dense store is
   *  a cache that owes nobody a copy later. */
  applyPlan: (result: client.RailsPlanResult) => Promise<void>;
  /** A decided weld became (or changed as) a program node. The host puts it
   *  in the program's node line, where it is the authority from then on. */
  upsertWeldNode: (node: WeldNode) => void;
  /** A weld left the list; its node goes with it. */
  removeWeldNode: (nodeId: string) => void;
  /** Where the parameter windows keep the operator's saved presets. */
  presetStore: PresetStore;
}

/** One seam's search result, cached under its seam id and parameters'
 *  digest; see `seamPlans` inside `createRailsMode`. */
export type { SeamPlan };

/** What the Rails welds card's ✕ puts back: every entry's decision and
 *  search, the cached seam plans, and both parameter sets. Taken when the
 *  card opens, restored on cancel. */
export interface RailsSnapshot {
  entries: Map<string, Pick<RailsEntry, "state" | "note" | "span" | "plan" | "clearance" | "summary" | "payload">>;
  seamPlans: Map<string, SeamPlan>;
  search: { preset: ParameterPreset; values: Record<string, ParameterValue> };
  torch: { preset: ParameterPreset; values: Record<string, ParameterValue> };
}

export interface RailsMode {
  readonly enabled: boolean;
  /** Active part analysis/search is in flight. Reset releases it immediately. */
  readonly busy: boolean;
  readonly entries: readonly RailsEntry[];
  /** Reconcile decided entries with the program, including deletion and undo. */
  syncProgramWelds(): void;
  snapshot(): RailsSnapshot;
  restore(snapshot: RailsSnapshot): void;
  /** Open the detail sheet of the entry for this seam (a weld node's id is
   *  its seam's id). False when no entry matches. */
  openEntryForSeam(seamId: string): boolean;
  toggle(): Promise<void>;
  /** Turn rails on (idempotent) — the auto path when a rails-authoring
   *  program imports its STEP. */
  enable(): Promise<void>;
  /** Turn rails off silently — the sync path when a classic program loads. */
  disable(): void;
  /** The CAD fingerprint of the part whose seams are analysed and drawn, or
   *  null when nothing is. The sync path compares it with the program's. */
  readonly fingerprint: string | null;
  /** Forget the analysed part: dispose every seam guide and weld overlay,
   *  drop the joints document, frames, searches and entries, turn the mode
   *  off. The sync path calls this when the program's part is not the one
   *  analysed (a New program, another program, a program without a part),
   *  so the previous part's seams never stay drawn over the next one. */
  reset(): void;
  /** True when rails mode consumed the click (mode on and part analysed).
   *  `pointM` is the pick-ray's hit on the edge tube (world), so a blocked
   *  seam resolves to the clear run UNDER the click, not the edge's middle. */
  onEdgeClicked(edgeId: string, pointM?: [number, number, number]): boolean;
  /** A plain click anywhere near a seam works too — an edge's pick tube is
   *  4 mm wide and easy to miss; a surface point beside the seam is not. */
  onScenePointClicked(pointM: [number, number, number]): boolean;
  /** Re-pose the overlays after the workpiece placement changes. */
  syncPlacement(): void;
  /** Render the rails weld list into the welds panel host. */
  /** Draw the seam list into `host`. With `head: false` the caller frames
   *  it (a ToolCard) and gets the "n of m decided" line back to show; with
   *  `parameters: false` the parameter buttons are left out, for a caller
   *  that shows the parameters its own way (`renderParameterSections`). */
  renderPanel(host: HTMLElement, options?: { head?: boolean; parameters?: boolean }): string;
  /** Draw the quick view of each parameter set -- the values worth
   *  changing without opening the sheet, and a way to the sheet -- and
   *  return each set's header line: its preset and how much differs. */
  renderParameterSections(hosts: { search: HTMLElement; torch: HTMLElement }): { search: string; torch: string };
  /** Which solver plans the connecting motions. The Planner selector owns
   *  this — a rails program's "planner" IS the weld planner, and the choice
   *  between its free-space solvers is the only thing left to choose there. */
  freeSpaceBackend: FreeSpaceBackend;
  /** What the Plan button does in a rails program: run the weld planner's
   *  search, both trajopt stages and the verifier, and keep the dense
   *  trajectory. Resolves true when a plan was installed, false when there
   *  was nothing decided; throws on failure so the caller does not go on to
   *  simulate a stale trajectory. */
  /** Plan every decided weld on the weld planner. `report` hears each
   *  stage as it starts (the request, the planner, the install), so a
   *  progress dialog can say where the wait is. */
  /** `fromProgramStart`: plan from the program's own start instead of the
   *  connected robot's measured pose (Go to plan start before Load/Play). */
  planWeldTrajectory(report?: (stage: { index: number; count: number; label: string }) => void, signal?: AbortSignal,
    options?: { fromProgramStart?: boolean }): Promise<boolean>;
  /** The `.weldplan` this program would be planned from, as bytes — the one
   *  artifact that carries a plan to another machine. Same packing as Plan,
   *  so what is exported is what would be planned. */
  exportWeldPlan(): Promise<{ bytes: Uint8Array; filename: string; sha256: string }>;
}

export function createRailsMode(deps: RailsDeps): RailsMode {
  let enabled = false;
  let busy = false;
  // Every reset/disable invalidates pending work, even when two parts have
  // identical STEP bytes and therefore the same fingerprint.
  let generation = 0;
  let preparation: Promise<void> | null = null;
  const snapshotGenerations = new WeakMap<RailsSnapshot, number>();
  const assertCurrent = (started: number): void => {
    if (started !== generation) throw new Error("The active part changed; this operation was cancelled.");
  };
  let sha: string | null = null;
  let doc: RailsJointsDoc | null = null;
  /** The default weld preset from the weld planner's settings. A weld
   *  searched bare — no preset — starts from candidate defaults (work 0°)
   *  and fouls on most fillets; their UI never does that, and neither does
   *  this mode. */
  const framesByJoint = new Map<string, client.RailsFrames>();
  /** One search per seam: the whole-arc plan, its payload and its clearance
   *  verdict, kept so clicking the OTHER clear run of a blocked seam adds
   *  that piece from the cache instead of searching again. */
  const seamPlans = new Map<string, SeamPlan>();
  const entries: RailsEntry[] = [];
  const authoredNodes = new Map<string, WeldNode>();
  let syncedProgramNodes: OfflineProgrammingProject["nodes"] | null = null;

  const root = new THREE.Group();
  root.name = "rails-overlay";
  root.visible = false;
  deps.parent.add(root);
  /** The detected seams themselves, drawn as guide lines the moment the
   *  analysis lands — the operator must SEE what is clickable, and the CAD
   *  edge overlay (cyan) shows every edge, not the weldable ones. */
  let guides: THREE.Group | null = null;

  const detail = buildDetailDialog();
  document.body.appendChild(detail.element);

  /** The two parameter windows this mode owns: how hard to look, and which
   *  gun is fitted.
   *
   *  Built ONCE, here, and re-parented into the panel on every render.
   *  `renderPanel` replaces the host's children, so a window rebuilt per
   *  render would lose focus, the caret and a half-typed number every time
   *  the weld list changed under it. Moving a live node keeps all three.
   *
   *  Changing either invalidates the searches already in the cache: a
   *  coverage number measured with the old gun, or the old deviation, is a
   *  number about a search nobody asked for. The already-ACCEPTED welds are
   *  untouched -- their nodes carry the values they were searched with. */
  const searchParameters = createParameterView({
    schema: SEARCH_SCHEMA,
    library: createPresetLibrary({
      scope: SEARCH_SCOPE,
      schema: SEARCH_SCHEMA,
      shipped: SHIPPED_SEARCH_PRESETS,
      store: deps.presetStore,
    }),
    status: (text, isError) => status(text, isError),
    onChange: () => parametersChanged("the angle search parameters changed"),
  });
  const torchParameters = createParameterView({
    schema: TORCH_SCHEMA,
    library: createPresetLibrary({
      scope: TORCH_SCOPE,
      schema: TORCH_SCHEMA,
      shipped: SHIPPED_TORCH_PRESETS,
      store: deps.presetStore,
    }),
    constraints: torchProblems,
    status: (text, isError) => status(text, isError),
    onChange: () => parametersChanged("the fitted torch changed"),
  });
  void searchParameters.ready();
  void torchParameters.ready();

  /** Each sheet in a modal of its own. Built here and never re-parented, so
   *  a dialog reopened is the sheet as it was left -- the loaded preset, the
   *  edits, all of it. Closing is not a cancel: an edit applies when it is
   *  committed, and there is no OK to forget to press. */
  const searchDialog = createParameterDialog({
    title: "Angle search",
    description:
      "How hard the search looks, and what looking costs. Reaches the worker "
      + "only while a seam is being searched — a weld that has been accepted "
      + "keeps the values it was searched with.",
    view: searchParameters,
  });
  const torchDialog = createParameterDialog({
    title: "Fitted torch",
    description:
      "The gun that is actually on the robot. It travels with every "
      + "measurement: coverage measured with the shipped nozzle in place of "
      + "the fitted one is a number about hardware nobody has.",
    view: torchParameters,
  });
  document.body.append(searchDialog.element, torchDialog.element);

  /** The fitted gun, as the seam worker reads one. Read at each call rather
   *  than captured: the window is live, and a request built from a snapshot
   *  taken at start-up would measure with the gun that WAS fitted. */
  const torch = () => torchRequest(torchParameters.values(), torchParameters.set.preset);
  /** The search knobs, likewise. Values, never the preset's id -- a preset
   *  edited later must not reach back and change a weld already accepted. */
  const search = () => ({
    work_deviation_deg: Number(searchParameters.values().work_deviation_deg),
    travel_deviation_deg: Number(searchParameters.values().travel_deviation_deg),
    standoff_cost_deg_per_mm: Number(searchParameters.values().standoff_cost_deg_per_mm),
  });
  const defaultStandoffMm = () => Number(torchParameters.values().default_standoff_mm);

  /** A parameter changed: redraw the panel so its button says how far off
   *  the preset the values now are, and drop anything measured with the old
   *  ones. Two jobs because they have different conditions -- the panel
   *  always needs redrawing, and there is often nothing cached to drop. */
  function parametersChanged(why: string): void {
    invalidateSearches(why);
    deps.requestPanelRender();
  }

  /** Drop the cached searches, because they were measured with numbers that
   *  are no longer the numbers. Proposed entries go back to unsearched; a
   *  weld that was accepted keeps what it was accepted with. */
  function invalidateSearches(why: string): void {
    if (seamPlans.size === 0) return;
    seamPlans.clear();
    for (const entry of entries) {
      if (entry.state !== "proposed") continue;
      entry.plan = null;
      entry.summary = null;
      // The clearance goes too: it is a measurement made with the gun and the
      // angles that are no longer the gun and the angles.
      entry.clearance = null;
      entry.note = "click again to search with the new parameters";
      redrawOverlay(entry);
    }
    status(`${why} — click a seam to search it again`);
  }

  // The direction arrows march as long as the mode is on — the motion is the
  // direction readout, and a paused arrow still points the right way.
  const animateFlows = (now: number): void => {
    if (enabled) {
      for (const entry of entries) {
        if (entry.overlay) tickOverlayFlow(entry.overlay, now / 1000);
      }
    }
    requestAnimationFrame(animateFlows);
  };
  requestAnimationFrame(animateFlows);

  const status = (text: string, isError = false, detail?: unknown) =>
    deps.status(`Rails: ${text}`, isError, detail);

  function syncPlacement(): void {
    const placement = projectWorkpiecePlacement(deps.getProject());
    if (!placement) return;
    root.position.set(placement.originM[0], placement.originM[1], placement.originM[2]);
    // The same three numbers the part mesh takes, in the same order: they
    // are a "ZYX" decomposition (workpieceWorldRotationRpy), and reading
    // them back as "XYZ" put the seams up to tens of degrees off the part
    // once the placement had rotation about more than one axis.
    root.rotation.set(
      placement.rotationRpyRad[0],
      placement.rotationRpyRad[1],
      placement.rotationRpyRad[2],
      "ZYX",
    );
  }

  /** Quietly push an imported STEP to the weld planner, so rails can enable
   *  later — this session or the next — from the project's fingerprint alone.
   *  Fire-and-forget: an unreachable weld planner must not break the import. */
  async function prepare(started: number): Promise<void> {
    const source = deps.getSource();
    const fingerprint = deps.getProject().cad.fingerprint;
    if (!source || source.kind !== "bytes" || !source.bytes) {
      // The STEP is kept under its fingerprint when it is imported and
      // restored when a project is opened, so this is a project saved before
      // it was kept -- or one whose site data was cleared.
      throw new Error(
        "this session has no copy of the part's STEP — import it again " +
          "(Import STEP), and it will be there next time",
      );
    }
    const step = source.bytes;
    if (sha === fingerprint && doc) return;
    status("detecting weld joints — topology, joints, default seams…");
    const detected = (await client.detectJoints(step)).joints;
    assertCurrent(started);
    doc = detected;
    framesByJoint.clear();
    seamPlans.clear();
    await drawSeamGuides(started);
    assertCurrent(started);
    sha = fingerprint;
    status(
      `${doc.seams.length} seams over ${doc.weld_joints.length} joints — ` +
        "click an orange seam line to search its angles",
    );
  }

  /** One orange line per detected joint, on the seam curve itself. Also
   *  warms the frames cache, so the first click matches without a fetch. */
  async function drawSeamGuides(started: number): Promise<void> {
    if (guides) {
      disposeOverlay(guides);
      guides = null;
    }
    if (!doc) return;
    const group = new THREE.Group();
    group.name = "rails-seam-guides";
    for (const joint of doc.weld_joints) {
      try {
        const frames = await jointFrames(joint, started);
        assertCurrent(started);
        const positions: number[] = [];
        for (const f of frames.frames) {
          positions.push(f.point_xyz_m[0]!, f.point_xyz_m[1]!, f.point_xyz_m[2]!);
        }
        if (positions.length < 6) continue;
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute(
          "position",
          new THREE.BufferAttribute(new Float32Array(positions), 3),
        );
        const line = new THREE.Line(
          geometry,
          new THREE.LineBasicMaterial({ color: RAILS_AMBER }),
        );
        line.renderOrder = 2;
        group.add(line);
      } catch (error) {
        if (started !== generation) {
          disposeOverlay(group);
          throw error;
        }
        console.warn(`rails: no guide for ${joint.id}:`, error);
      }
    }
    assertCurrent(started);
    guides = group;
    root.add(group);
    syncPlacement();
  }

  async function jointFrames(joint: RailsJoint, started = generation): Promise<client.RailsFrames> {
    const cached = framesByJoint.get(joint.id);
    if (cached) return cached;
    const seam = doc?.seams.find((s) => s.weld_joint_ref === joint.id);
    const frames = await client.sampleSeamFrames(joint.weld_line, {
      mode: "rail_curve",
      rail: seam?.reference.rail ?? joint.bisector_rail?.curve,
    });
    assertCurrent(started);
    framesByJoint.set(joint.id, frames);
    return frames;
  }

  /** The joint whose curve this edge runs along, and where along it. */
  async function matchSeam(
    samplesM: [number, number, number][],
    toleranceM = MATCH_TOLERANCE_M,
  ): Promise<{ seam: RailsSeam; joint: RailsJoint; s: number } | null> {
    const started = generation;
    if (!doc) return null;
    const probes: [number, number, number][] = [];
    const step = Math.max(1, Math.floor(samplesM.length / 9));
    for (let i = 0; i < samplesM.length; i += step) probes.push(samplesM[i]!);

    let best: { joint: RailsJoint; score: number; midS: number } | null = null;
    for (const joint of doc.weld_joints) {
      const frames = (await jointFrames(joint, started)).frames;
      assertCurrent(started);
      if (frames.length < 2) continue;
      const nearest = (p: [number, number, number]) => {
        let distance = Infinity;
        let s = 0;
        for (const f of frames) {
          const d = Math.hypot(
            p[0] - f.point_xyz_m[0]!,
            p[1] - f.point_xyz_m[1]!,
            p[2] - f.point_xyz_m[2]!,
          );
          if (d < distance) {
            distance = d;
            s = f.s;
          }
        }
        return { distance, s };
      };
      const hits = probes.map(nearest);
      const score = hits.reduce((sum, h) => sum + h.distance, 0) / hits.length;
      if (!best || score < best.score) {
        const mid = nearest(samplesM[Math.floor(samplesM.length / 2)]!);
        best = { joint, score, midS: mid.s };
      }
    }
    if (!best || best.score > toleranceM) return null;
    const seam = seamAtS(best.joint, best.midS);
    return seam ? { seam, joint: best.joint, s: best.midS } : null;
  }

  /** The seam of this joint whose span holds `s`, wrap-aware. */
  function seamAtS(joint: RailsJoint, s: number): RailsSeam | null {
    if (!doc) return null;
    const onJoint = doc.seams.filter((x) => x.weld_joint_ref === joint.id);
    return (
      onJoint.find((x) => {
        const span = x.span;
        return span.end_s < span.start_s
          ? s >= span.start_s || s <= span.end_s
          : s >= span.start_s && s <= span.end_s;
      }) ?? onJoint[0] ?? null
    );
  }

  /** The joint's `s` nearest a part-frame point — where along the seam a
   *  click actually landed. */
  async function nearestSOnJoint(
    joint: RailsJoint,
    p: [number, number, number],
  ): Promise<number | null> {
    const started = generation;
    const frames = (await jointFrames(joint, started)).frames;
    assertCurrent(started);
    let s: number | null = null;
    let distance = Infinity;
    for (const f of frames) {
      const d = Math.hypot(
        p[0] - f.point_xyz_m[0]!,
        p[1] - f.point_xyz_m[1]!,
        p[2] - f.point_xyz_m[2]!,
      );
      if (d < distance) {
        distance = d;
        s = f.s;
      }
    }
    return s;
  }

  /** The STEP this session imported. Every measuring request carries it: the
   *  worker resolves nothing from a hash, so there is nowhere else it could
   *  come from. `prepare` has already refused if it is missing. */
  function stepBytes(): Uint8Array {
    const source = deps.getSource();
    if (!source || source.kind !== "bytes" || !source.bytes) {
      throw new Error("this session has no copy of the part's STEP — import it again");
    }
    return source.bytes;
  }

  function upsertEntry(seam: RailsSeam, joint: RailsJoint): RailsEntry {
    const existing = entries.find((entry) => entry.id === `rails-${seam.id}`);
    if (existing) {
      if (existing.overlay) disposeOverlay(existing.overlay);
      existing.state = "searching";
      existing.note = "queued…";
      existing.plan = null;
      existing.clearance = null;
      existing.summary = null;
      existing.payload = null;
      existing.overlay = null;
      existing.span = { ...seam.span };
      return existing;
    }
    const entry: RailsEntry = {
      id: `rails-${seam.id}`,
      seam,
      joint,
      state: "searching",
      note: "queued…",
      span: { ...seam.span },
      plan: null,
      clearance: null,
      summary: null,
      payload: null,
      overlay: null,
    };
    entries.push(entry);
    return entry;
  }

  /** The comb's angles: the payload's CURRENT profiles evaluated at the
   *  plan's knot fractions — so an edit in the spline display moves the comb,
   *  not just the numbers. */
  function anglePath(entry: RailsEntry): AnglePath | null {
    if (!entry.plan || !entry.payload) return null;
    const orientation = (entry.payload.overrides.orientation ?? {}) as {
      work_angle_deg?: ScalarFunction;
      travel_angle_deg?: ScalarFunction;
    };
    const knots = entry.plan.knot_fractions;
    const work = orientation.work_angle_deg;
    const travel = orientation.travel_angle_deg;
    return {
      knotFractions: knots,
      workDeg: knots.map((f, i) =>
        work ? evaluate(work, f) : entry.plan!.work_path_deg[i] ?? 0),
      travelDeg: knots.map((f, i) =>
        travel ? evaluate(travel, f) : entry.plan!.travel_path_deg[i] ?? 0),
      reverse: (entry.plan.direction ?? entry.seam.direction) === "reverse",
    };
  }

  function redrawOverlay(entry: RailsEntry): void {
    if (entry.overlay) disposeOverlay(entry.overlay);
    entry.overlay = null;
    const frames = framesByJoint.get(entry.joint.id);
    if (!frames) return;
    entry.overlay = buildRailsOverlay(frames, entry.span, anglePath(entry));
    root.add(entry.overlay);
    root.visible = enabled;
    syncPlacement();
    // First orientation pass under the real placement — the build-time tick
    // ran before the group was parented, so the arrows' lookAt saw no
    // workpiece rotation yet.
    tickOverlayFlow(entry.overlay, 0);
  }

  /**
   * One clear run of a blocked seam becomes its own entry — the weld
   * planner's SPLIT, one piece at a time: the operator clicked THIS side, so
   * only this side enters the list; the other side enters when they click it.
   * The optimised profiles carry over narrowed onto the piece (a split
   * decides WHERE to weld, not how), and only the clearance re-check runs.
   */
  async function addRunPiece(
    seam: RailsSeam,
    joint: RailsJoint,
    cached: SeamPlan,
    k: number,
  ): Promise<void> {
    const started = generation;
    const count = cached.report.split.length;
    const { id, span, payload } = runPiecePayload(seam, cached, k);
    const stale = entries.find((e) => e.id === id);
    if (stale) removeEntry(stale);
    const entry: RailsEntry = {
      id,
      seam,
      joint,
      state: "searching",
      note: `run ${k + 1}/${count} — checking clearance…`,
      span,
      plan: null,
      clearance: null,
      summary: null,
      payload,
      overlay: null,
    };
    entries.push(entry);
    deps.requestPanelRender();
    const piece = await measureRunPiece(client, stepBytes(), seam, joint, cached, k, torch(), () => {
      assertCurrent(started);
      return entries.includes(entry);
    });
    if (!piece) return;
    entry.payload = piece.payload;
    entry.clearance = piece.clearance;
    entry.summary = piece.summary;
    entry.plan = piece.plan;
    entry.state = "proposed";
    entry.note = piece.note;
    redrawOverlay(entry);
  }

  async function optimiseSeam(
    seam: RailsSeam,
    joint: RailsJoint,
    clickS: number,
  ): Promise<void> {
    if (!sha) return;
    const started = generation;
    busy = true;
    let progress: RailsEntry | null = null;
    deps.requestPanelRender();
    try {
      let cached = seamPlans.get(seam.id);
      if (!cached) {
        progress = upsertEntry(seam, joint);
        deps.requestPanelRender();
        status(`${seam.id}: searching angles…`);
        const searched = await searchSeam(
          client,
          stepBytes(),
          seam,
          joint,
          { torch: torch(), search: search(), defaultStandoffMm: defaultStandoffMm() },
          () => {
            assertCurrent(started);
            return entries.includes(progress!);
          },
          (direction, plan) => {
            progress!.note = `${direction}, ${Math.round(100 * (plan.clear_fraction ?? 0))}% clear`;
            status(`${seam.id}: checking clearance at the chosen angles…`);
          },
        );
        if (!searched) return;
        cached = searched;
        seamPlans.set(seam.id, cached);
      }

      const { report } = cached;
      if (narrowsToRun(report)) {
        // Blocked somewhere. Only the run the operator clicked enters the
        // list; the guides keep showing the rest of the seam for the next
        // click.
        if (progress) removeEntry(progress);
        const k = runIndexAt(report, fractionAt(seam.span, clickS));
        await addRunPiece(seam, joint, cached, k);
        assertCurrent(started);
        status(
          report.split.length === 1
            ? `${seam.id}: narrowed to its one clear run — the blocked metal is left out`
            : `${seam.id}: blocked mid-way — run ${k + 1} of ${report.split.length} ` +
              "added; click the other clear stretches to add them",
        );
      } else {
        const entry = progress ?? upsertEntry(seam, joint);
        entry.plan = cached.plan;
        entry.payload = cached.payload;
        entry.clearance = cached.clearance;
        entry.summary = summarise(cached.clearance, seam.span);
        entry.state = "proposed";
        entry.note = `${Math.round(cached.plan.clear_fraction * 100)}% weldable`;
        redrawOverlay(entry);
        status(
          `${seam.id}: ${Math.round(cached.plan.clear_fraction * 100)}% weldable at ` +
            "the proposed angles — open it in the welds panel to decide",
        );
      }
    } catch (error) {
      if (started !== generation) return;
      if (progress && !entries.includes(progress)) return;
      const note = error instanceof Error ? error.message : String(error);
      if (progress) {
        progress.state = "failed";
        progress.note = note;
      }
      status(note, true);
    } finally {
      if (started === generation) {
        busy = false;
        deps.requestPanelRender();
      }
    }
  }

  /** The solver the next Plan hands the connecting motions to.
   *  Session state, not program state: it changes how a plan is COMPUTED and
   *  nothing about what was authored, so it does not belong on the project. */
  let freeSpaceBackend: FreeSpaceBackend = "bspline";

  /** Plan the decided welds: the seam search, then the weld and connecting
   *  trajopt stages, then the verifier. Minutes, not seconds.
   *
   *  There used to be a shallower mode that stopped after the search and
   *  compiled its winning candidates straight to `robot.v4.program.v1` rows,
   *  so the spreadsheet filled quickly. Accepting a weld writes its node now,
   *  so the spreadsheet is already full before anything is planned, and the
   *  rows are gone with `export/robot_v4.py`. What is left is planning. */
  /** Everything `build_plan_request` needs, in one place, so Plan and Export
   *  cannot drift into packing two different containers. */
  async function planRequestBody(fromProgramStart = false): Promise<client.PlanRequestBody> {
    const started = generation;
    const project = deps.getProject();
    const body = await packPlanRequest(project, {
      tooling: { ...torch(), default_standoff_mm: defaultStandoffMm() },
      stepBase64: hasWorkpieceGeometry(project) ? client.encodeSTEP(stepBytes()) : "",
      stepFilename: deps.getSource()?.filename ?? "part.step",
      fixtures: deps.getFixtures(),
      freeSpaceBackend,
      requireMeasuredStart: hasMotionNodes(project) && deps.requiresMeasuredStart?.() === true,
      planFromProgramStart: fromProgramStart,
      closeLoop: deps.closeLoop?.() === true,
    });
    assertCurrent(started);
    return body;
  }

  async function exportWeldPlan(): Promise<{ bytes: Uint8Array; filename: string; sha256: string }> {
    if (!sha && hasWorkpieceGeometry(deps.getProject())) {
      throw new Error(
        "this program's STEP is not loaded in this session, so there is no geometry "
          + "to pack — import it again (Import STEP)",
      );
    }
    const started = generation;
    const body = await planRequestBody();
    assertCurrent(started);
    const result = await client.exportWeldPlan(body);
    assertCurrent(started);
    return result;
  }

  async function planWeldTrajectory(
    report: (stage: { index: number; count: number; label: string }) => void = () => {},
    signal?: AbortSignal,
    options: { fromProgramStart?: boolean } = {},
  ): Promise<boolean> {
    signal?.throwIfAborted();
    // Every refusal says why. A Plan button that does nothing is
    // indistinguishable from a broken one, and these two used to return
    // silently -- which is exactly what "I press Plan and nothing happens"
    // looks like from the outside.
    if (busy) {
      status("still working — wait for the seam search to finish, then plan");
      return false;
    }
    if (!sha && hasWorkpieceGeometry(deps.getProject())) {
      // The welds are project state and survive a reload; the STEP is kept
      // separately and may not have. Planning needs the geometry, so this is
      // reachable with a full weld list on screen.
      status(
        "this program's STEP is not loaded in this session, so there is no "
          + "geometry to plan against — import it again (Import STEP)",
        true,
      );
      return false;
    }
    // The program's seam welds are what gets planned: the nodes carry the
    // decided span, direction, profiles and speed, edited or not, and they
    // survive a reload where the session's entries do not.
    const decided = plannedNodes(deps.getProject());
    if (!decided.length) {
      status("Nothing to plan — record a waypoint or add a weld");
      return false;
    }
    const unsupported = measuredStartUnsupported(deps.getProject(), deps.requiresMeasuredStart?.() === true);
    if (unsupported) {
      status(unsupported, true);
      return false;
    }
    const started = generation;
    busy = true;
    deps.requestPanelRender();
    try {
      const project = deps.getProject();
      const placement = projectWorkpiecePlacement(project);
      if (hasWorkpieceGeometry(project) && !placement) throw new Error("the part has no placement to plan against");
      status(
        `planning ${decided.length} move/weld node(s) — weld + connecting trajopt, ` +
          "verify…",
      );
      // The digest is stamped inside `planRequestBody`, before the document
      // travels, so it is in the `.weldplan` and in the request hash that
      // covers it -- and so a trajectory kept beside it can say which program
      // it came from rather than which one is open now. Export packs the same
      // body, so what is exported is what would be planned.
      report({ index: 1, count: 4, label: "Packing the program for the weld planner" });
      const fromProgramStart = options.fromProgramStart === true;
      const body = await planRequestBody(fromProgramStart);
      assertCurrent(started);
      signal?.throwIfAborted();
      report({
        index: 2,
        count: 4,
        label: `Planning ${decided.length} move/weld node${decided.length === 1 ? "" : "s"} · seam search, weld and connecting trajopt, verify`,
      });
      const out = await client.planWeldTrajectory(body, signal);
      assertCurrent(started);
      signal?.throwIfAborted();
      report({ index: 3, count: 4, label: "Keeping the plan and its trajectory" });
      await deps.applyPlan(out);
      assertCurrent(started);
      const denseNote = out.dense
        ? ` · trajectory ${out.dense.total_sample_count} samples, ` +
          `${out.dense.total_duration_s.toFixed(1)} s, kept here`
        : ` · no trajectory (${out.dense_error?.reason ?? "not built"})`;
      status(`planned ${decided.length} move/weld node(s)${denseNote} — ` +
        (fromProgramStart ? PLANNED_FROM_PROGRAM_START : "Simulate replays it"));
      return true;
    } catch (error) {
      if (started !== generation) return false;
      if (signal?.aborted) {
        status("Planning cancelled");
        throw error;
      }
      const refusal = planRefusalFrom(error);
      status(planRefusalText(refusal), true, typeof refusal === "object" ? refusal : error);
      // Thrown, not swallowed: the caller goes on to the canonical simulator
      // and must not check a stale plan.
      throw error;
    } finally {
      if (started === generation) {
        busy = false;
        deps.requestPanelRender();
      }
    }
  }

  function removeEntry(entry: RailsEntry): void {
    if (!entries.includes(entry)) return;
    if (entry.overlay) disposeOverlay(entry.overlay);
    const index = entries.indexOf(entry);
    if (index >= 0) entries.splice(index, 1);
    entry.overlay = null;
    if (deps.getProject().nodes.some((node) => node.id === entry.id)) deps.removeWeldNode(entry.id);
    deps.requestPanelRender();
  }

  function authoredMotionKey(node: WeldNode): string {
    const provenance = railsProvenance(node);
    return JSON.stringify([node.geometry, node.travel_speed_mm_s, node.standoff_mm, node.weave,
      provenance?.joint_id, provenance?.joint, provenance?.sha256]);
  }

  function syncProgramWelds(): void {
    const project = deps.getProject();
    if (!doc || sha !== project.cad.fingerprint) return;
    if (project.nodes === syncedProgramNodes) return;
    const current = new Map(project.nodes.filter(isRailsWeld).map((node) => [node.id, node]));
    for (const entry of [...entries]) {
      if (!current.has(entry.id) && (authoredNodes.has(entry.id) || entry.state === "accepted" || entry.state === "trimmed")) {
        if (entry.overlay) disposeOverlay(entry.overlay);
        entry.overlay = null;
        entries.splice(entries.indexOf(entry), 1);
        detail.close();
      }
    }
    for (const [id, node] of current) {
      const existing = entries.find((entry) => entry.id === id);
      const previous = authoredNodes.get(id);
      // Ordinary project edits clone every node; identity alone is not a change.
      if (existing && previous && JSON.stringify(previous) === JSON.stringify(node)) continue;
      if (node.geometry.kind !== "seam") continue;
      const geometry = node.geometry;
      const provenance = railsProvenance(node);
      if (provenance?.sha256 && provenance.sha256 !== sha) continue;
      const detected = doc.seams.find((seam) => seam.id === geometry.id);
      const joint = provenance?.joint as RailsJoint | undefined
        ?? doc.weld_joints.find((joint) => joint.id === provenance?.joint_id) ?? existing?.joint;
      if (!joint) continue;
      const payload = railsPayloadOf(node)!;
      const span = payload.span ?? detected?.span ?? { start_s: 0, end_s: 1 };
      const seam = { ...detected, ...geometry, label: node.label, span,
        weld_joint_ref: joint.id, process: { travel_speed_mm_s: node.travel_speed_mm_s },
      } as unknown as RailsSeam;
      const changedMotion = existing && (!previous || authoredMotionKey(previous) !== authoredMotionKey(node));
      // Replacement invalidates outstanding searches by object identity as well
      // as clearing measurements made against the earlier authored geometry.
      if (changedMotion) {
        if (existing.overlay) disposeOverlay(existing.overlay);
        existing.overlay = null;
        seamPlans.delete(existing.seam.id);
        detail.close();
      }
      const entry: RailsEntry = existing && !changedMotion ? existing : { id, seam, joint, span, state: "accepted", note: "",
        plan: null, clearance: null, summary: null, payload: null, overlay: null };
      if (existing && changedMotion) entries[entries.indexOf(existing)] = entry;
      entry.joint = joint;
      entry.seam = seam;
      entry.span = { ...span };
      entry.payload = payload;
      entry.note = node.comment || "accepted";
      if (entry.state !== "trimmed") entry.state = "accepted";
      if (!existing) entries.push(entry);
      redrawOverlay(entry);
    }
    authoredNodes.clear();
    for (const [id, node] of current) authoredNodes.set(id, structuredClone(node));
    syncedProgramNodes = project.nodes;
  }

  function decide(entry: RailsEntry, choice: "trim" | "accept"): void {
    if (!entries.includes(entry)) return;
    const previous = deps.getProject().nodes.find((node): node is WeldNode => node.id === entry.id && isRailsWeld(node));
    const decision = decideWeld(entry, choice, sha ?? "", previous);
    if (!decision) return;
    entry.span = decision.span;
    entry.state = decision.state;
    entry.note = decision.note;
    if (decision.payload) entry.payload = decision.payload;
    redrawOverlay(entry);
    detail.close();
    const node = decision.node;
    // The synchronous program render must retain the just-accepted measurements.
    authoredNodes.set(node.id, structuredClone(node));
    deps.upsertWeldNode(node);
    deps.requestPanelRender();
    status(`${entry.seam.id} ${entry.note}`);
  }

  function openDetail(entry: RailsEntry): void {
    detail.show(entry, {
      onDecide: (choice) => decide(entry, choice),
      onRemove: () => {
        removeEntry(entry);
        detail.close();
      },
      onProfileChange: (field, fn) => {
        if (!entries.includes(entry) || !entry.payload) return;
        const orientation = {
          ...(entry.payload.overrides.orientation as Record<string, unknown>),
          [field]: fn,
        };
        entry.payload = {
          ...entry.payload,
          overrides: { ...entry.payload.overrides, orientation },
        };
        redrawOverlay(entry);
      },
    });
  }

  function renderPanel(host: HTMLElement, options: { head?: boolean; parameters?: boolean } = {}): string {
    const decided = deps.getProject().nodes.filter(isRailsWeld).length;
    const summary = `${decided} of ${entries.length} decided`;
    const rows = entries.map((entry) => {
      const children: (HTMLElement | string)[] = [
        el("span", { class: "rails-entry-label" }, entry.seam.label || entry.seam.id),
        el("span", { class: entry.note.trimEnd().endsWith("…") ? "rails-entry-note is-busy" : "rails-entry-note" }, entry.note),
      ];
      if (entry.state !== "searching") {
        // Remove, right on the row: the same as the dialog's "Remove weld".
        children.push(
          el(
            "button",
            {
              type: "button",
              class: "rails-entry-remove",
              title: "Remove this weld",
              "aria-label": "Remove weld",
              on: {
                click: (event: Event) => {
                  event.stopPropagation();
                  removeEntry(entry);
                },
              },
            },
            iconElement(X, 14),
          ),
        );
      }
      if (entry.state === "proposed") {
        // The two decisions right on the row — the detail dialog stays for
        // reading the profiles, but deciding must not require a detour.
        // Trim is always offered once a search has found a clear run: on a
        // seam reachable end to end it trims to the whole of it, which is
        // still the operator's call to make rather than the tool's.
        const canTrim = Boolean(entry.summary?.trimSpan);
        children.push(
          el(
            "div",
            { class: "rails-entry-actions" },
            el(
              "button",
              {
                type: "button",
                disabled: !canTrim,
                title: canTrim
                  ? "narrow the weld to the longest stretch the torch can reach"
                  : "no clear stretch was found to trim to",
                on: {
                  click: (event: Event) => {
                    event.stopPropagation();
                    decide(entry, "trim");
                  },
                },
              },
              canTrim ? `Trim (${entry.summary!.trimMm.toFixed(0)} mm)` : "Trim",
            ),
            el(
              "button",
              {
                type: "button",
                on: {
                  click: (event: Event) => {
                    event.stopPropagation();
                    decide(entry, "accept");
                  },
                },
              },
              "Accept",
            ),
          ),
        );
      }
      return el(
        "div",
        {
          class: `rails-entry is-${entry.state}`,
          on: {
            click: () => {
              if (entry.state !== "searching") openDetail(entry);
            },
          },
        },
        ...children,
      );
    });
    replace(
      host,
      el(
        "div",
        { class: "weld-drawer-panel rails-panel" },
        options.head === false
          ? ""
          : el(
              "div",
              { class: "rails-panel-head" },
              el("b", {}, "Rails welds"),
              el("span", { class: "rails-entry-note" }, summary),
            ),
        rows.length
          ? rows
          : el(
              "p",
              { class: "rails-panel-hint" },
              doc
                ? "Click an orange seam line to search its angles."
                : "Import a STEP to analyse its seams.",
            ),
        // Below the welds, because the welds are what the operator came for.
        // Each opens a sheet in a modal; what stays here is the one line an
        // operator needs to notice a wrong preset without opening anything.
        options.parameters === false
          ? ""
          : el(
              "div",
              { class: "rails-parameter-buttons" },
              parameterButton("Angle search", searchParameters, searchDialog),
              parameterButton("Fitted torch", torchParameters, torchDialog),
            ),
      ),
    );
    return summary;
  }

  /** The fields shown inline in the Rails welds card. The rest of each
   *  schema stays behind "All parameters…": a sheet of twenty values in a
   *  viewport card would be the sheet, only cramped. */
  const QUICK_SEARCH_KEYS = ["work_deviation_deg", "travel_deviation_deg"];
  const QUICK_TORCH_KEYS = ["default_standoff_mm", "standoff_min_mm", "standoff_max_mm"];

  function snapshot(): RailsSnapshot {
    const saved = new Map<string, RailsSnapshot["entries"] extends Map<string, infer V> ? V : never>();
    for (const entry of entries) {
      saved.set(entry.id, {
        state: entry.state,
        note: entry.note,
        span: { ...entry.span },
        plan: entry.plan,
        clearance: entry.clearance,
        summary: entry.summary,
        payload: entry.payload,
      });
    }
    const result: RailsSnapshot = {
      entries: saved,
      seamPlans: new Map(seamPlans),
      search: { preset: searchParameters.set.preset, values: searchParameters.values() },
      torch: { preset: torchParameters.set.preset, values: torchParameters.values() },
    };
    snapshotGenerations.set(result, generation);
    return result;
  }

  function restore(saved: RailsSnapshot): void {
    if (snapshotGenerations.get(saved) !== generation) return;
    for (const entry of entries) {
      const before = saved.entries.get(entry.id);
      if (!before) continue;
      Object.assign(entry, { ...before, span: { ...before.span } });
      redrawOverlay(entry);
    }
    seamPlans.clear();
    for (const [key, plan] of saved.seamPlans) seamPlans.set(key, plan);
    for (const [view, state] of [[searchParameters, saved.search], [torchParameters, saved.torch]] as const) {
      view.set.loadPreset(state.preset);
      for (const [key, value] of Object.entries(state.values)) view.set.set(key, value);
      view.refresh();
    }
    deps.requestPanelRender();
  }

  function renderParameterSections(
    hosts: { search: HTMLElement; torch: HTMLElement },
  ): { search: string; torch: string } {
    return {
      search: renderQuickParameters(hosts.search, searchParameters, searchDialog, QUICK_SEARCH_KEYS, "the angle search parameters changed"),
      torch: renderQuickParameters(hosts.torch, torchParameters, torchDialog, QUICK_TORCH_KEYS, "the fitted torch changed"),
    };
  }

  function renderQuickParameters(
    host: HTMLElement,
    view: ParameterView,
    dialog: ParameterDialog,
    keys: readonly string[],
    why: string,
  ): string {
    host.replaceChildren();
    for (const key of keys) {
      const field = view.set.schema.find((candidate) => candidate.key === key);
      if (!field || field.kind !== "number") continue;
      const state = view.set.state(key);
      const number = new NumberField({
        label: field.label,
        unit: field.unit,
        step: field.step ?? "any",
        min: field.min,
        max: field.max,
        onChange: (value) => {
          const outcome = view.set.set(key, value);
          if (!outcome.ok) {
            status(`${field.label} ${outcome.reason}`, true);
            number.setValue(formatParameter(field, view.set.get(key), false));
            return;
          }
          view.refresh();
          parametersChanged(why);
        },
      });
      number.el.title = field.help ?? "";
      number.el.classList.toggle("is-modified", state.modified);
      number.setValue(formatParameter(field, state.value, false));
      number.mount(host);
    }
    const more = el(
      "button",
      {
        type: "button",
        class: "tool-card-more",
        title: "Open the full parameter sheet: presets, every value, save",
        on: { click: () => dialog.open() },
      },
      "All parameters…",
    );
    host.append(more);
    const count = view.set.modifiedKeys().length;
    return count === 0
      ? view.set.preset.label
      : `${view.set.preset.label} · ${count} changed`;
  }

  /** The panel's handle on one parameter sheet: what is loaded, how far off
   *  it the values are, and a way in. */
  function parameterButton(
    label: string, view: ParameterView, dialog: ParameterDialog,
  ): HTMLElement {
    const count = view.set.modifiedKeys().length;
    return el(
      "button",
      {
        type: "button",
        class: `rails-parameter-button${count > 0 ? " is-modified" : ""}`,
        title: `Open the ${label.toLowerCase()} parameters`,
        on: { click: () => dialog.open() },
      },
      el("span", { class: "rails-entry-label" }, label),
      el(
        "span",
        { class: "rails-entry-note" },
        count === 0
          ? view.set.preset.label
          : `${view.set.preset.label} · ${count} changed`,
      ),
    );
  }

  return {
    get enabled() {
      return enabled;
    },
    get busy() {
      return busy;
    },
    get entries() {
      return entries;
    },
    enable() {
      if (preparation) return preparation;
      if (enabled) return Promise.resolve();
      const started = generation;
      busy = true;
      preparation = (async () => {
        try {
          await prepare(started);
          assertCurrent(started);
          enabled = true;
          root.visible = true;
          syncProgramWelds();
          syncPlacement();
        } catch (error) {
          if (started === generation) status(error instanceof Error ? error.message : String(error), true);
        } finally {
          if (started === generation) {
            busy = false;
            preparation = null;
            deps.requestPanelRender();
          }
        }
      })();
      return preparation;
    },
    get freeSpaceBackend() {
      return freeSpaceBackend;
    },
    set freeSpaceBackend(name: FreeSpaceBackend) {
      freeSpaceBackend = name;
    },
    disable() {
      generation += 1;
      preparation = null;
      busy = false;
      enabled = false;
      root.visible = false;
      detail.close();
      deps.requestPanelRender();
    },
    get fingerprint() {
      return doc ? sha : null;
    },
    reset() {
      generation += 1;
      preparation = null;
      busy = false;
      for (const entry of entries) {
        if (entry.overlay) disposeOverlay(entry.overlay);
        entry.overlay = null;
      }
      entries.length = 0;
      authoredNodes.clear();
      syncedProgramNodes = null;
      seamPlans.clear();
      framesByJoint.clear();
      if (guides) {
        disposeOverlay(guides);
        guides = null;
      }
      doc = null;
      sha = null;
      enabled = false;
      root.visible = false;
      detail.close();
      status("off — import a part and its seams are detected");
      deps.requestPanelRender();
    },
    async toggle() {
      if (busy) return;
      if (enabled) {
        this.disable();
        status("off — edge clicks draft welds the usual way");
        return;
      }
      await this.enable();
    },
    onScenePointClicked(pointM: [number, number, number]): boolean {
      if (!enabled) return false;
      if (busy) {
        status("still working — one search at a time");
        return true;
      }
      syncPlacement();
      root.updateMatrixWorld(true);
      const local = root.worldToLocal(
        new THREE.Vector3(pointM[0], pointM[1], pointM[2]));
      const started = generation;
      void (async () => {
        busy = true;
        try {
          status("matching the point to a detected seam…");
          const match = await matchSeam(
            [[local.x, local.y, local.z]], POINT_TOLERANCE_M);
          assertCurrent(started);
          busy = false;
          if (!match) {
            // A click that found no seam is not a failure of anything; it
            // is told where it landed and left as information.
            status(
              "no detected seam within 25 mm of " +
                `[${local.x.toFixed(3)}, ${local.y.toFixed(3)}, ${local.z.toFixed(3)}] ` +
                "(part frame) — click closer to a joint line",
            );
            return;
          }
          await optimiseSeam(match.seam, match.joint, match.s);
        } catch (error) {
          if (started !== generation) return;
          busy = false;
          status(error instanceof Error ? error.message : String(error), true);
        }
      })();
      return true;
    },
    onEdgeClicked(edgeId: string, pointM?: [number, number, number]): boolean {
      if (!enabled) return false;
      if (busy) {
        status("still working — one search at a time");
        return true;
      }
      const edge = catalogEdgeById(deps.getProject(), edgeId);
      if (!edge || edge.samplesM.length < 2) {
        status(`edge ${edgeId} has no geometry to match`, true);
        return true;
      }
      // The hit point in the part's frame — an edge is matched as a whole,
      // but WHERE along it the click landed decides which clear run of a
      // blocked seam is meant.
      let clickedLocal: [number, number, number] | null = null;
      if (pointM) {
        syncPlacement();
        root.updateMatrixWorld(true);
        const local = root.worldToLocal(
          new THREE.Vector3(pointM[0], pointM[1], pointM[2]));
        clickedLocal = [local.x, local.y, local.z];
      }
      const started = generation;
      void (async () => {
        busy = true;
        try {
          status("matching the edge to a detected seam…");
          const samples = edge.samplesM as [number, number, number][];
          let match = await matchSeam(samples);
          assertCurrent(started);
          let snapped = false;
          if (!match) {
            // The clicked edge is an outline, not a joint line — the usual
            // near-miss (a rib's outer edge sits one plate-thickness from
            // its junction seam). Snap from the click itself when we have
            // it, the edge's midpoint otherwise, instead of answering with
            // silence-shaped precision.
            const probe =
              clickedLocal ?? samples[Math.floor(samples.length / 2)]!;
            match = await matchSeam([probe], POINT_TOLERANCE_M);
            assertCurrent(started);
            snapped = match !== null;
          }
          if (match && clickedLocal) {
            // The edge matched as a whole; the click decides both WHICH
            // seam piece of the joint is meant and where along it.
            const nearest = await nearestSOnJoint(match.joint, clickedLocal);
            assertCurrent(started);
            if (nearest !== null) {
              const seam = seamAtS(match.joint, nearest) ?? match.seam;
              match = { ...match, seam, s: nearest };
            }
          }
          busy = false;
          if (!match) {
            status(
              "no detected seam within 25 mm of that edge — click a joint " +
                "line where two parts meet",
            );
            return;
          }
          if (snapped) status(`that edge is not a seam — snapped to ${match.seam.id}`);
          await optimiseSeam(match.seam, match.joint, match.s);
        } catch (error) {
          if (started !== generation) return;
          busy = false;
          status(error instanceof Error ? error.message : String(error), true);
        }
      })();
      return true;
    },
    syncPlacement,
    syncProgramWelds,
    renderPanel,
    renderParameterSections,
    snapshot,
    restore,
    openEntryForSeam(seamId: string): boolean {
      const entry = entries.find((candidate) => candidate.seam.id === seamId);
      if (!entry || entry.state === "searching") return false;
      openDetail(entry);
      return true;
    },
    planWeldTrajectory,
    exportWeldPlan,
  };
}

// ---------------------------------------------------------------------------
// the detail dialog — opened from the list, never by itself
// ---------------------------------------------------------------------------

interface DetailCallbacks {
  onDecide(choice: "trim" | "accept"): void;
  onRemove(): void;
  onProfileChange(
    field: "work_angle_deg" | "travel_angle_deg",
    fn: ScalarFunction,
  ): void;
}

interface DetailDialog {
  element: HTMLDialogElement;
  show(entry: RailsEntry, callbacks: DetailCallbacks): void;
  close(): void;
}

const STRIP_W = 520;

function coverageStrip(summary: CoverageSummary): SVGElement {
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", `0 0 ${STRIP_W} 10`);
  svg.setAttribute("class", "rails-dialog-coverage");
  svg.setAttribute("role", "img");
  svg.setAttribute("aria-label", "Clear and blocked stretches");
  const rect = (x: number, width: number, fill: string) => {
    const node = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    node.setAttribute("x", String(10 + x * (STRIP_W - 20)));
    node.setAttribute("y", "0");
    node.setAttribute("width", String(Math.max(1, width * (STRIP_W - 20))));
    node.setAttribute("height", "10");
    node.setAttribute("fill", fill);
    svg.appendChild(node);
  };
  rect(0, 1, "#3a4250");
  const paint = (spans: number[][], colour: string) => {
    for (const [a, b] of spans) {
      if (b! < a!) {
        rect(a!, 1 - a!, colour);
        rect(0, b!, colour);
      } else {
        rect(a!, b! - a!, colour);
      }
    }
  };
  paint(summary.clearRuns, "#2f9e6e");
  paint(summary.blockedSpans, "#c8442c");
  return svg;
}

function buildDetailDialog(): DetailDialog {
  const element = document.createElement("dialog");
  element.className = "rails-dialog";
  element.addEventListener("click", (event) => {
    if (event.target === element) element.close();
  });

  return {
    element,
    close: () => {
      if (element.open) element.close();
    },
    show(entry, callbacks) {
      const { plan, summary, seam } = entry;
      const body = el("div", { class: "rails-dialog-body" });
      body.appendChild(el("h2", {}, seam.label || seam.id));
      if (plan) {
        body.appendChild(
          el(
            "p",
            { class: "rails-dialog-stats" },
            `${summary?.seamMm.toFixed(0) ?? "?"} mm seam · ` +
              `${Math.round(plan.clear_fraction * 100)}% weldable at the proposed angles ` +
              `(${Math.round(plan.authored_clear_fraction * 100)}% as authored) · ` +
              `standoff ${plan.standoff_mm.toFixed(1)} mm` +
              (plan.direction_changed ? " · direction REVERSED by the search" : ""),
          ),
        );
      }

      // The weld planner's own spline display, one per angle. Editing a
      // profile updates the entry's overrides and the comb in the viewport.
      const orientation = ((entry.payload?.overrides.orientation ?? {}) as {
        work_angle_deg?: ScalarFunction;
        travel_angle_deg?: ScalarFunction;
      });
      for (const [field, label] of [
        ["work_angle_deg", "Work angle"],
        ["travel_angle_deg", "Travel angle"],
      ] as const) {
        const editor = createSplineEditor({
          label,
          unit: "°",
          min: -90,
          max: 90,
          zero: 0,
          onChange: (fn) => callbacks.onProfileChange(field, fn),
        });
        editor.set(orientation[field], "search");
        body.appendChild(editor.root);
      }

      if (summary) body.appendChild(coverageStrip(summary));

      const canTrim = Boolean(summary?.trimSpan);
      body.appendChild(
        el(
          "div",
          { class: "rails-dialog-actions" },
          el(
            "button",
            {
              type: "button",
              disabled: !canTrim,
              title: canTrim
                ? "narrow the weld to the longest stretch the torch can reach"
                : "no clear stretch was found to trim to",
              on: { click: () => callbacks.onDecide("trim") },
            },
            `Trim to longest clear run (${summary?.trimMm.toFixed(0) ?? 0} mm)`,
          ),
          el(
            "button",
            { type: "button", on: { click: () => callbacks.onDecide("accept") } },
            "Accept",
          ),
          el(
            "button",
            {
              type: "button",
              class: "secondary",
              on: { click: () => callbacks.onRemove() },
            },
            "Remove weld",
          ),
          el(
            "button",
            { type: "button", class: "secondary", on: { click: () => element.close() } },
            "Close",
          ),
        ),
      );
      replace(element, body);
      if (!element.open) element.showModal();
    },
  };
}
