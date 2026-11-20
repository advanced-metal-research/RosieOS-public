import "./edge-geometry.css";

import * as THREE from "three";
import {
  appendReport,
  animateCameraElliptical,
  buildDogBoxSeamPayload,
  compactEdgeMeta,
  compactEdgeTitle,
  createCadqueryStage,
  createFrameConceptGroup,
  type FrameConcept,
  loadDogBoxTopology,
  rankedSeamEdges,
  renderSingleLogHud,
  query,
} from "./cadquery-shared";

type PointTuple = [number, number, number];
type Point3 = { x: number; y: number; z: number };
type ToolFrame = { x: Point3; y: Point3; z: Point3 };
type FrameMode = "backend" | "alternate";

type TopologyEdge = {
  id: string;
  label?: string;
  part_id?: string;
  geom_type?: string;
  length_mm?: number;
  samples: PointTuple[];
  closed?: boolean;
  source_edge_ids?: string[];
  source_geom_types?: string[];
  source_geom_counts?: Record<string, number>;
  path_class?: "circle" | "dogbone_4arc_4line" | "wire_non_seam" | "raw_fragment" | string;
  is_seam_candidate?: boolean;
};

type TopologyModel = {
  model_id: string;
  filename: string;
  fingerprint: string;
  edges: TopologyEdge[];
  seam_candidate_count?: number;
};

type TCPPose = {
  index: number;
  target_xyz_mm?: PointTuple;
  seam_tangent?: PointTuple;
  seam_plane_normal?: PointTuple;
  seam_side_axis?: PointTuple;
  tool_x_axis?: PointTuple;
  tool_y_axis?: PointTuple;
  tool_z_axis?: PointTuple;
  angle_from_seam_plane_deg?: number;
  tool_x_tangent_alignment?: number;
};

type SeamPayload = {
  tool: string;
  placement?: {
    origin_mm?: PointTuple;
    table?: string;
    axis?: string;
  };
  seam: {
    edge_id: string;
    point_count: number;
    start_s: number;
    end_s: number;
    points?: PointTuple[];
    world_points_mm?: PointTuple[];
  };
  tesseract_plan?: {
    tcp_poses?: TCPPose[];
    geometry_method?: {
      selected?: string;
      candidates?: GeometryCandidate[];
      selection_reason?: string;
    };
    path_method?: {
      selected?: string;
      control_pose_count?: number;
      control_points?: PointTuple[];
    };
    environment?: {
      collision_objects?: unknown[];
      constraints?: Record<string, unknown>;
    };
  };
};

type GeometryCandidate = {
  id: string;
  score?: number;
  away_from_part?: number;
  upward?: number;
  tool_x_tangent_alignment?: number;
  angle_from_seam_plane_deg?: number;
  max_frame_rotation_deg?: number;
};

declare global {
  interface Window {
    __edgeGeometryRuntime?: {
      scene: THREE.Scene;
      camera: THREE.PerspectiveCamera;
      controls: ReturnType<typeof createCadqueryStage>["controls"];
      runFrameTour?: () => Promise<void>;
    };
  }
}

const EDGE_DEFAULT = 0x8a8a8a;
const EDGE_SELECTED = 0xffffff;
const PATH_COLOR = 0xd8d8d8;
const PASS_LIMIT_DEG = 10;
const DOG_BOX_TOP_CENTER_MM: Point3 = { x: 1913.0, y: 685.80026, z: 650.8788 };

const app = document.querySelector<HTMLElement>("#edge-geometry-app");
if (!app) throw new Error("missing #edge-geometry-app");

const hud = renderSingleLogHud({
  app,
  shellClass: "edge-geometry-shell",
  stageSectionClass: "stage-wrap",
  stageId: "stage",
  stageLabel: "Full screen edge geometry stage",
  hudClass: "edge-hud-grid",
  title: "Edge Geometry",
  activeApp: "edge-geometry",
  tourButtonId: "frame-tour",
  logId: "edge-report-log",
  extraHiddenButtonsHTML: `<button id="frame-mode-toggle" type="button">Frame: Backend</button>`,
  hiddenHTML: `
    <aside class="panel panel-controls">
      <p class="section-kicker">Commands</p>
      <h2>Edge Geometry Sandbox</h2>
      <div class="actions">
        <button id="load-dog-box" type="button">Load Dog Box</button>
        <button id="plan-selected" type="button" disabled>Plan Selected Edge</button>
        <button id="strategy-tour" type="button">Run Strategy Tour</button>
      </div>
      <div class="actions">
        <button id="sampled-path" type="button">Sampled Edge</button>
        <button id="three-point-path" type="button">3 Point Arc</button>
      </div>
      <dl class="metric-grid">
        <div class="metric"><dt>Model</dt><dd id="model-metric">none</dd></div>
        <div class="metric"><dt>Edges</dt><dd id="edge-count">0</dd></div>
      </dl>
      <div id="edge-list" class="edge-list"></div>
    </aside>
    <aside class="panel panel-validation">
      <p class="section-kicker">Validation</p>
      <h2 id="selected-title">No edge selected</h2>
      <dl class="metric-grid">
        <div class="metric"><dt>TCP poses</dt><dd id="pose-count">0</dd></div>
        <div class="metric"><dt>+X tangent min</dt><dd id="x-align">-</dd></div>
        <div class="metric"><dt>Angle range</dt><dd id="angle-range">-</dd></div>
        <div class="metric"><dt>Collision object</dt><dd id="collision-state">-</dd></div>
        <div class="metric"><dt>Constraint</dt><dd id="constraint-state">-</dd></div>
        <div class="metric"><dt>Path Method</dt><dd id="path-method-state">sampled_edge</dd></div>
        <div class="metric"><dt>Method</dt><dd id="method-state">-</dd></div>
        <div class="metric"><dt>Result</dt><dd id="result-state">waiting</dd></div>
      </dl>
      <section>
        <p class="section-kicker">Strategies</p>
        <div id="strategy-list" class="strategy-list"></div>
      </section>
      <textarea id="plan-json" readonly spellcheck="false"></textarea>
    </aside>
    <ol id="strategy-tour-log" class="strategy-tour-log" hidden></ol>
    <div id="frame-tour-caption" class="tour-caption frame-tour-caption" role="status" aria-live="polite" hidden></div>
  `,
});

const stageEl = hud.stageEl;
const edgeListEl = query<HTMLElement>("#edge-list");
const loadButton = query<HTMLButtonElement>("#load-dog-box");
const planButton = query<HTMLButtonElement>("#plan-selected");
const strategyTourButton = query<HTMLButtonElement>("#strategy-tour");
const frameTourButton = hud.tourButton;
const frameModeButton = query<HTMLButtonElement>("#frame-mode-toggle");
const sampledPathButton = query<HTMLButtonElement>("#sampled-path");
const threePointPathButton = query<HTMLButtonElement>("#three-point-path");
const modelMetric = query<HTMLElement>("#model-metric");
const edgeCountEl = query<HTMLElement>("#edge-count");
const reportLogEl = hud.reportLogEl;
const selectedTitle = query<HTMLElement>("#selected-title");
const poseCountEl = query<HTMLElement>("#pose-count");
const xAlignEl = query<HTMLElement>("#x-align");
const angleRangeEl = query<HTMLElement>("#angle-range");
const collisionStateEl = query<HTMLElement>("#collision-state");
const constraintStateEl = query<HTMLElement>("#constraint-state");
const pathMethodStateEl = query<HTMLElement>("#path-method-state");
const methodStateEl = query<HTMLElement>("#method-state");
const resultStateEl = query<HTMLElement>("#result-state");
const strategyListEl = query<HTMLElement>("#strategy-list");
const strategyTourLogEl = query<HTMLElement>("#strategy-tour-log");
const planJSONEl = query<HTMLTextAreaElement>("#plan-json");
const frameTourCaptionEl = query<HTMLElement>("#frame-tour-caption");

const stage = createCadqueryStage(stageEl, {
  cameraPosition: { x: 430, y: -1300, z: 620 },
  target: { x: 0, y: 0, z: 120 },
  gridSize: 1600,
  gridDivisions: 32,
});
const { scene, camera, renderer, controls } = stage;
window.__edgeGeometryRuntime = { scene, camera, controls, runFrameTour: () => runFrameTour() };

let topology: TopologyModel | null = null;
let selectedEdge: TopologyEdge | null = null;
let modelOffset: Point3 = { x: 0, y: 0, z: 0 };
let dogBoxOffset: Point3 = { x: 0, y: 0, z: 0 };
let dogBoxWorldCenter: Point3 = { x: 0, y: 0, z: 0 };
let edgeGroup: THREE.Group | null = null;
let pathGroup: THREE.Group | null = null;
let frameTourGroup: THREE.Group | null = null;
let weldAnimationGroup: THREE.Group | null = null;
let weldTraceLine: THREE.Line | null = null;
let weldTorch: THREE.Group | null = null;
let weldTorchLiveGizmo: THREE.Group | null = null;
let weldAnimationStartedMS = performance.now();
let activeWeldPath: Point3[] = [];
let activeWeldToolFrames: ToolFrame[] = [];
let selectedStrategyId: string | null = null;
let lastPayload: SeamPayload | null = null;
let selectedPathMethod: "sampled_edge" | "three_point_arc" = "sampled_edge";
let frameMode: FrameMode = "backend";
let activeFrameTourPose: TCPPose | null = null;

const FRAME_CONCEPTS: FrameConcept<TCPPose>[] = [
  {
    label: "SEAM TANGENT",
    caption: "The seam tangent is the local travel direction sampled from the CAD dog-bone path.",
    color: 0xff2020,
    scale: 1.35,
    vector: (pose) => tupleVector(pose.seam_tangent),
  },
  {
    label: "TOOL +X",
    caption: "Tool +X is locked to the seam tangent, so the TCP frame points forward along travel.",
    color: 0xff2020,
    scale: 1.55,
    vector: (pose) => tupleVector(pose.tool_x_axis),
  },
  {
    label: "SEAM PLANE NORMAL",
    caption: "The seam plane normal comes from the sampled seam points. It defines what out-of-plane means.",
    color: 0xf7f0e4,
    scale: 1.2,
    vector: (pose) => tupleVector(pose.seam_plane_normal),
  },
  {
    label: "SEAM SIDE",
    caption: "The side axis is in the seam plane, perpendicular to travel. The backend can flip this side.",
    color: 0xffd166,
    scale: 1.2,
    vector: (pose) => tupleVector(pose.seam_side_axis),
  },
  {
    label: "TOOL +Z / TORCH",
    caption: "Tool +Z is the torch axis. It is tilted 45 degrees out of the seam plane.",
    color: 0x38bdf8,
    scale: 1.45,
    vector: (pose) => tupleVector(pose.tool_z_axis),
  },
  {
    label: "TOOL +Y",
    caption: "Tool +Y completes the right-handed TCP frame from tool +Z and tool +X.",
    color: 0x72d6a4,
    scale: 1.25,
    vector: (pose) => tupleVector(pose.tool_y_axis),
  },
];

const ALT_FRAME_CONCEPTS: FrameConcept<TCPPose>[] = [
  {
    label: "+Y TRAVEL",
    caption: "+X mode: +Y follows the seam tangent, so travel is the green axis instead of backend +X.",
    color: 0x72d6a4,
    scale: 1.55,
    vector: (pose) => frameAxisVector(pose, "alternate", "y"),
  },
  {
    label: "+X TORCH",
    caption: "+X mode: +X is the torch direction, selected to point into the Dog Box/workpiece.",
    color: 0xff2020,
    scale: 1.55,
    vector: (pose) => frameAxisVector(pose, "alternate", "x"),
  },
  {
    label: "+Z RIGHT-HANDED",
    caption: "+X mode: +Z is recomputed from +X and +Y so the alternate TCP frame remains right-handed.",
    color: 0x38bdf8,
    scale: 1.3,
    vector: (pose) => frameAxisVector(pose, "alternate", "z"),
  },
];

function point(sample: PointTuple): Point3 {
  return { x: Number(sample[0]), y: Number(sample[1]), z: Number(sample[2]) };
}

function toScene(pointValue: Point3): THREE.Vector3 {
  return new THREE.Vector3(pointValue.x + dogBoxOffset.x, pointValue.y + dogBoxOffset.y, pointValue.z + dogBoxOffset.z);
}

function worldToScene(value: PointTuple): THREE.Vector3 {
  // Backend world coordinates keep the Dog Box on the positioner table; this
  // page shows the same geometry re-anchored at the world origin.
  return new THREE.Vector3(
    Number(value[0]) - DOG_BOX_TOP_CENTER_MM.x,
    Number(value[1]) - DOG_BOX_TOP_CENTER_MM.y,
    Number(value[2]) - DOG_BOX_TOP_CENTER_MM.z,
  );
}

function format(value: number, digits = 3): string {
  return Number.isFinite(value) ? value.toFixed(digits) : "-";
}

function pushReport(command: string, lines: string[] = []): void {
  appendReport(reportLogEl, command, lines);
}

function setFrameTourCaption(text: string): void {
  frameTourCaptionEl.textContent = text;
  pushReport("subtitle", [text]);
}

async function loadDogBox(): Promise<void> {
  loadButton.disabled = true;
  resultStateEl.textContent = "loading Dog Box";
  pushReport("cadquery topology dog_box.step", ["status=loading"]);
  topology = await loadDogBoxTopology(64);
  modelMetric.textContent = topology.model_id.slice(0, 20);
  edgeCountEl.textContent = String(topology.seam_candidate_count ?? topology.edges.length);
  pushReport("cadquery topology dog_box.step", [
    `model=${topology.model_id.slice(0, 20)} edges=${topology.edges.length} seam_candidates=${topology.seam_candidate_count ?? topology.edges.length}`,
  ]);
  computePlacement(topology);
  renderEdges(topology);
  renderEdgeList(topology.edges);
  resultStateEl.textContent = "pick an edge";
  loadButton.disabled = false;
}

function computePlacement(model: TopologyModel): void {
  const points = model.edges.flatMap((edge) => edge.samples.map(point));
  const min = {
    x: Math.min(...points.map((p) => p.x)),
    y: Math.min(...points.map((p) => p.y)),
    z: Math.min(...points.map((p) => p.z)),
  };
  const max = {
    x: Math.max(...points.map((p) => p.x)),
    y: Math.max(...points.map((p) => p.y)),
    z: Math.max(...points.map((p) => p.z)),
  };
  modelOffset = { x: (min.x + max.x) * 0.5, y: (min.y + max.y) * 0.5, z: min.z };
  // Anchor the Dog Box bottom center at the world origin on this page.
  dogBoxOffset = {
    x: -modelOffset.x,
    y: -modelOffset.y,
    z: -modelOffset.z,
  };
  dogBoxWorldCenter = {
    x: (min.x + max.x) * 0.5 + dogBoxOffset.x,
    y: (min.y + max.y) * 0.5 + dogBoxOffset.y,
    z: (min.z + max.z) * 0.5 + dogBoxOffset.z,
  };
}

function renderEdges(model: TopologyModel): void {
  if (edgeGroup) {
    scene.remove(edgeGroup);
    dispose(edgeGroup);
  }
  edgeGroup = new THREE.Group();
  model.edges.forEach((edge) => {
    if (edge.samples.length < 2) return;
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(edge.samples.map((sample) => toScene(point(sample)))),
      new THREE.LineBasicMaterial({ color: edge.id === selectedEdge?.id ? EDGE_SELECTED : EDGE_DEFAULT, transparent: true, opacity: 0.78, depthTest: false }),
    );
    line.renderOrder = 10;
    edgeGroup!.add(line);
  });
  scene.add(edgeGroup);
}

function renderEdgeList(edges: TopologyEdge[]): void {
  const ranked = rankedSeamEdges(edges, 80);
  edgeListEl.replaceChildren(...ranked.map((edge) => {
    const row = document.createElement("button");
    row.type = "button";
    row.className = "edge-row";
    row.classList.toggle("is-selected", edge.id === selectedEdge?.id);
    row.innerHTML = `<strong>${compactEdgeTitle(edge)}</strong><span>${compactEdgeMeta(edge)}</span>`;
    row.addEventListener("click", () => {
      selectedEdge = edge;
      ensureSelectedPathMethod(edge);
      selectedTitle.textContent = edge.id;
      planButton.disabled = false;
      renderEdges(topology!);
      renderEdgeList(topology!.edges);
      void planEdge(edge);
    });
    return row;
  }));
}

async function planEdge(edge: TopologyEdge): Promise<void> {
  if (!topology) return;
  ensureSelectedPathMethod(edge);
  resultStateEl.textContent = "planning";
  pushReport(`tcp plan ${edge.id}`, [
    `frame=${frameMode} path_method=${selectedPathMethod} samples=128`,
  ]);
  const payload = await buildDogBoxSeamPayload<SeamPayload>({
    edge_id: edge.id,
    start_s: 0.12,
    end_s: 0.88,
    seam_sample_count: 128,
    geometry_method: selectedStrategyId,
    path_method: selectedPathMethod,
  });
  renderPlan(payload);
}

function renderPlan(payload: SeamPayload): void {
  lastPayload = payload;
  const poses = payload.tesseract_plan?.tcp_poses ?? [];
  poseCountEl.textContent = String(poses.length);
  const alignments = poses.map((pose) => Number(pose.tool_x_tangent_alignment ?? NaN)).filter(Number.isFinite);
  const angles = poses.map((pose) => Number(pose.angle_from_seam_plane_deg ?? NaN)).filter(Number.isFinite);
  const minAlignment = Math.min(...alignments);
  const minAngle = Math.min(...angles);
  const maxAngle = Math.max(...angles);
  const collisionOK = Boolean(payload.tesseract_plan?.environment?.collision_objects?.length);
  const constraintOK = payload.tesseract_plan?.environment?.constraints?.tool_x_axis === "locked_to_seam_tangent";
  const methodName = payload.tesseract_plan?.geometry_method?.selected ?? "missing";
  const candidateCount = payload.tesseract_plan?.geometry_method?.candidates?.length ?? 0;
  const backendPathMethod = payload.tesseract_plan?.path_method?.selected ?? selectedPathMethod;
  pushReport(`tcp result ${payload.seam.edge_id}`, [
    `poses=${poses.length} +x_tangent_min=${format(minAlignment, 6)} angle=${format(minAngle, 2)}-${format(maxAngle, 2)}deg result=${minAlignment >= 0.999 && minAngle >= 35 && maxAngle <= 55 ? "PASS" : "FAIL"}`,
  ]);
  xAlignEl.textContent = format(minAlignment, 6);
  angleRangeEl.textContent = `${format(minAngle, 2)}-${format(maxAngle, 2)} deg`;
  collisionStateEl.textContent = collisionOK ? "dog_box_workpiece" : "missing";
  constraintStateEl.textContent = constraintOK ? "+X tangent / 45deg" : "missing";
  pathMethodStateEl.textContent = backendPathMethod;
  methodStateEl.textContent = `${methodName} (${candidateCount})`;
  xAlignEl.className = minAlignment >= 0.999 ? "pass" : "fail";
  angleRangeEl.className = minAngle >= 35 && maxAngle <= 55 ? "pass" : "fail";
  collisionStateEl.className = collisionOK ? "pass" : "fail";
  constraintStateEl.className = constraintOK ? "pass" : "fail";
  pathMethodStateEl.className = backendPathMethod === selectedPathMethod ? "pass" : "warn";
  methodStateEl.className = methodName !== "missing" && candidateCount > 0 ? "pass" : "fail";
  resultStateEl.textContent = minAlignment >= 0.999 && minAngle >= 35 && maxAngle <= 55 && collisionOK && constraintOK ? "PASS" : "FAIL";
  resultStateEl.className = resultStateEl.textContent === "PASS" ? "pass" : "fail";
  planJSONEl.value = JSON.stringify(payload, null, 2);
  renderStrategyList(payload.tesseract_plan?.geometry_method?.candidates ?? [], methodName);
  renderPlanGeometry(payload);
}

function renderStrategyList(candidates: GeometryCandidate[], selected: string): void {
  if (candidates.length === 0) {
    strategyListEl.replaceChildren(empty("No strategies returned yet."));
    return;
  }
  strategyListEl.replaceChildren(
    ...candidates.map((candidate) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "strategy-row";
      button.classList.toggle("is-selected", candidate.id === selected);
      button.innerHTML = `
        <strong>${candidate.id}</strong>
        <span>score ${format(Number(candidate.score ?? Number.NaN), 3)} / +X ${format(Number(candidate.tool_x_tangent_alignment ?? Number.NaN), 4)} / angle ${format(Number(candidate.angle_from_seam_plane_deg ?? Number.NaN), 2)} / frame_rot ${format(Number(candidate.max_frame_rotation_deg ?? Number.NaN), 1)}deg</span>
      `;
      button.addEventListener("click", () => {
        selectedStrategyId = candidate.id;
        if (selectedEdge) void planEdge(selectedEdge);
      });
      return button;
    }),
  );
}

function empty(text: string): HTMLElement {
  const div = document.createElement("div");
  div.className = "empty-state";
  div.textContent = text;
  return div;
}

async function runStrategyTour(): Promise<void> {
  strategyTourButton.disabled = true;
  strategyTourLogEl.replaceChildren();
  try {
    if (!topology) {
      await loadDogBox();
    }
    const edges = [...(topology?.edges ?? [])]
      .filter((edge) => edge.is_seam_candidate)
      .sort((a, b) => (b.length_mm ?? 0) - (a.length_mm ?? 0))
      .slice(0, 2);
    for (const edge of edges) {
      selectedStrategyId = null;
      selectedEdge = edge;
      selectedTitle.textContent = edge.id;
      renderEdges(topology!);
      renderEdgeList(topology!.edges);
      await planEdge(edge);
      addTourLog(`Edge ${edge.id}: backend selected ${lastPayload?.tesseract_plan?.geometry_method?.selected ?? "none"}`);
      const candidates = (lastPayload?.tesseract_plan?.geometry_method?.candidates ?? []).slice(0, 4);
      for (const pathMethod of supportedPathMethods(edge)) {
        selectedPathMethod = pathMethod;
        syncPathMethodButtons();
        for (const candidate of candidates) {
          selectedStrategyId = candidate.id;
          await planEdge(edge);
          addTourLog(`${pathMethod} / ${candidate.id}: ${resultStateEl.textContent}, +X ${xAlignEl.textContent}, angle ${angleRangeEl.textContent}, frame_rot ${format(Number(candidate.max_frame_rotation_deg ?? Number.NaN), 1)}deg`);
          await sleep(250);
        }
      }
    }
  } finally {
    strategyTourButton.disabled = false;
  }
}

async function runFrameTour(): Promise<void> {
  frameTourButton.disabled = true;
  try {
    frameTourCaptionEl.hidden = false;
    setFrameTourCaption("Loading Dog Box and planning a dog-bone seam frame.");
    pushReport(`tour frame start`, [`frame=${frameMode}`]);
    if (!topology) {
      await loadDogBox();
    }
    const edge = [...(topology?.edges ?? [])]
      .filter((item) => item.path_class === "dogbone_4arc_4line")
      .sort((a, b) => (b.length_mm ?? 0) - (a.length_mm ?? 0))[0];
    if (!edge) throw new Error("No dog-bone seam candidate found.");
    selectedPathMethod = "sampled_edge";
    selectedStrategyId = null;
    selectedEdge = edge;
    selectedTitle.textContent = edge.id;
    renderEdges(topology!);
    renderEdgeList(topology!.edges);
    await planEdge(edge);
    const poses = lastPayload?.tesseract_plan?.tcp_poses ?? [];
    const pose = poses[Math.max(0, Math.min(poses.length - 1, Math.floor(poses.length * 0.38)))];
    if (!pose?.target_xyz_mm) throw new Error("No TCP pose returned for frame tour.");
    await framePoseCamera(pose);
    for (const concept of frameConceptsForMode()) {
      renderFrameConcept(pose, concept);
      setFrameTourCaption(concept.caption);
      await sleep(1800);
    }
    renderFrameConcepts(pose, frameConceptsForMode());
    setFrameTourCaption(frameMode === "alternate"
      ? "+X mode tour complete: red +X is the into-box torch axis, green +Y follows travel, and blue +Z completes the frame."
      : "Frame tour complete: red +X follows travel, blue +Z is the 45 degree torch axis, and green +Y completes the TCP frame.");
    resultStateEl.textContent = frameMode === "alternate" ? "ALT FRAME TOUR PASS" : "FRAME TOUR PASS";
    resultStateEl.className = "pass";
    pushReport("tour frame complete", [
      `frame=${frameMode} seam=${edge.id} labels=${frameConceptsForMode().length} result=PASS`,
    ]);
  } catch (error) {
    frameTourCaptionEl.hidden = false;
    setFrameTourCaption(`Frame tour failed: ${error instanceof Error ? error.message : String(error)}`);
    pushReport("tour frame error", [error instanceof Error ? error.message : String(error)]);
  } finally {
    frameTourButton.disabled = false;
  }
}

async function runLoopingMultiAppTour(): Promise<void> {
  if (frameTourButton.disabled) return;
  try {
    frameMode = "backend";
    syncFrameModeButton();
    await runFrameTour();
    await sleep(900);
    frameMode = "alternate";
    syncFrameModeButton();
    await runFrameTour();
    await sleep(2200);
  } finally {
    // Always hand back to Weld Sequence so the multi-app tour loops forever,
    // even when a frame tour fails.
    const nextURL = new URL(`${import.meta.env.BASE_URL}seam`, window.location.origin);
    nextURL.searchParams.set("loop", "1");
    window.location.href = nextURL.toString();
  }
}

function frameConceptsForMode(): FrameConcept<TCPPose>[] {
  return frameMode === "alternate" ? ALT_FRAME_CONCEPTS : FRAME_CONCEPTS;
}

function framePoseCamera(pose: TCPPose): Promise<void> {
  if (!pose.target_xyz_mm) return Promise.resolve();
  const target = worldToScene(pose.target_xyz_mm);
  return animateCameraElliptical({
    camera,
    controls,
    pose: {
      target,
      position: target.clone().add(new THREE.Vector3(430, -520, 320)),
      durationMs: 900,
    },
  });
}

function renderFrameConcept(pose: TCPPose, concept: FrameConcept<TCPPose>): void {
  renderFrameConcepts(pose, [concept]);
}

function renderFrameConcepts(pose: TCPPose, concepts: FrameConcept<TCPPose>[]): void {
  clearFrameTour();
  if (!pose.target_xyz_mm) return;
  const origin = worldToScene(pose.target_xyz_mm);
  frameTourGroup = createFrameConceptGroup({ pose, origin, concepts, groupName: "edge-geometry-frame-tour" });
  scene.add(frameTourGroup);
}

function clearFrameTour(): void {
  if (frameTourGroup) {
    scene.remove(frameTourGroup);
    dispose(frameTourGroup);
    frameTourGroup = null;
  }
}

function addTourLog(text: string): void {
  const item = document.createElement("li");
  item.textContent = text;
  strategyTourLogEl.appendChild(item);
}

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function renderPlanGeometry(payload: SeamPayload): void {
  if (pathGroup) {
    scene.remove(pathGroup);
    dispose(pathGroup);
  }
  pathGroup = new THREE.Group();
  const seamPoints = payload.seam.world_points_mm ?? payload.seam.points ?? [];
  const scenePath = seamPoints.map((sample) => {
    const vector = worldToScene(sample);
    return { x: vector.x, y: vector.y, z: vector.z };
  });
  if (seamPoints.length >= 2) {
    pathGroup.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(seamPoints.map(worldToScene)),
      new THREE.LineBasicMaterial({ color: PATH_COLOR, depthTest: false }),
    ));
  }
  const poses = payload.tesseract_plan?.tcp_poses ?? [];
  activeWeldToolFrames = poses.map((pose) => toolFrameFromPose(pose, frameMode)).filter((frame): frame is ToolFrame => frame !== null);
  poses.forEach((pose, index) => {
    if (index % Math.max(1, Math.floor(poses.length / 24)) !== 0 && index !== poses.length - 1) return;
    if (!pose.target_xyz_mm || !pose.tool_x_axis || !pose.tool_y_axis || !pose.tool_z_axis) return;
    const frame = toolFrameFromPose(pose, frameMode);
    if (!frame) return;
    const gizmo = createAxisGizmo(worldToScene(pose.target_xyz_mm), frame, 34);
    pathGroup!.add(gizmo);
  });
  scene.add(pathGroup);
  setupWeldAnimation(scenePath);
}

function createAxisGizmo(position: THREE.Vector3, frame: ToolFrame, size: number): THREE.Group {
  const group = new THREE.Group();
  group.position.copy(position);
  const axes = [
    { axis: frame.x, color: 0xff4b4b, scale: 1.25 },
    { axis: frame.y, color: 0x72d6a4, scale: 1.0 },
    { axis: frame.z, color: 0x38bdf8, scale: 1.0 },
  ];
  axes.forEach((entry) => {
    const end = new THREE.Vector3(entry.axis.x, entry.axis.y, entry.axis.z).normalize().multiplyScalar(size * entry.scale);
    group.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), end]),
      new THREE.LineBasicMaterial({ color: entry.color, depthTest: false }),
    ));
  });
  return group;
}

function setupWeldAnimation(points: Point3[]): void {
  activeWeldPath = points.map((point) => ({ ...point }));
  weldAnimationStartedMS = performance.now();
  if (weldAnimationGroup) {
    scene.remove(weldAnimationGroup);
    dispose(weldAnimationGroup);
  }
  weldAnimationGroup = new THREE.Group();
  weldAnimationGroup.name = "edge-geometry-weld-torch-animation";
  weldTraceLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([]),
    new THREE.LineBasicMaterial({ color: 0xf2f2f2, depthTest: false }),
  );
  weldTraceLine.name = "edge-geometry-weld-trace";
  weldTraceLine.renderOrder = 86;
  weldAnimationGroup.add(weldTraceLine);
  weldTorch = createWeldTorch();
  weldAnimationGroup.add(weldTorch);
  const initialFrame = activeWeldToolFrames[0] ?? {
    x: { x: 1, y: 0, z: 0 },
    y: { x: 0, y: 1, z: 0 },
    z: { x: 0, y: 0, z: 1 },
  };
  weldTorchLiveGizmo = createToolFrameGizmo(activeWeldPath[0] ?? { x: 0, y: 0, z: 0 }, initialFrame, 42);
  weldTorchLiveGizmo.name = "edge-geometry-live-weld-tcp-gizmo";
  weldAnimationGroup.add(weldTorchLiveGizmo);
  scene.add(weldAnimationGroup);
}

function createWeldTorch(): THREE.Group {
  const group = new THREE.Group();
  group.name = "edge-geometry-animated-weld-torch";
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(2.2, 2.2, 58, 14),
    new THREE.MeshBasicMaterial({ color: 0xf7f0e4, depthTest: false, depthWrite: false }),
  );
  shaft.position.y = -48;
  shaft.renderOrder = 88;
  group.add(shaft);
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(8.5, 24, 20),
    new THREE.MeshBasicMaterial({ color: 0xcfcfcf, depthTest: false, depthWrite: false }),
  );
  cone.position.y = -12;
  cone.rotation.x = Math.PI;
  cone.renderOrder = 89;
  group.add(cone);
  const contact = new THREE.Mesh(
    new THREE.SphereGeometry(5.2, 18, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.88, depthTest: false, depthWrite: false }),
  );
  contact.name = "edge-geometry-weld-contact-blue-dot";
  contact.renderOrder = 90;
  group.add(contact);
  return group;
}

function createToolFrameGizmo(position: Point3, frame: ToolFrame, size: number): THREE.Group {
  const group = new THREE.Group();
  group.position.set(position.x, position.y, position.z);
  group.renderOrder = 91;
  [
    { axis: frame.x, color: 0xff4b4b, name: "tcp-x" },
    { axis: frame.y, color: 0x72d6a4, name: "tcp-y" },
    { axis: frame.z, color: 0x38bdf8, name: "tcp-z" },
  ].forEach((entry) => {
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)]),
      new THREE.LineBasicMaterial({ color: entry.color, depthTest: false }),
    );
    line.name = entry.name;
    line.renderOrder = 92;
    group.add(line);
  });
  updateToolFrameGizmo(group, position, frame, size);
  return group;
}

function updateToolFrameGizmo(group: THREE.Group, position: Point3 | THREE.Vector3, frame: ToolFrame, size: number): void {
  group.position.set(position.x, position.y, position.z);
  [
    { axis: frame.x, name: "tcp-x" },
    { axis: frame.y, name: "tcp-y" },
    { axis: frame.z, name: "tcp-z" },
  ].forEach((entry) => {
    const line = group.getObjectByName(entry.name) as THREE.Line | undefined;
    if (!line) return;
    const axis = new THREE.Vector3(entry.axis.x, entry.axis.y, entry.axis.z).normalize().multiplyScalar(size);
    line.geometry.dispose();
    line.geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), axis]);
  });
}

function updateWeldAnimation(nowMS: number): void {
  if (!weldAnimationGroup || !weldTorch || !weldTraceLine || !weldTorchLiveGizmo || activeWeldPath.length < 2) {
    return;
  }
  const durationMS = Math.max(2400, activeWeldPath.length * 80);
  const progress = ((nowMS - weldAnimationStartedMS) % durationMS) / durationMS;
  const sample = sampleScenePath(activeWeldPath, progress);
  weldTorch.position.copy(sample.position);
  const frameIndex = Math.max(0, Math.min(activeWeldToolFrames.length - 1, Math.round(progress * Math.max(0, activeWeldToolFrames.length - 1))));
  const frame = activeWeldToolFrames[frameIndex] ?? toolFrameFromTangentAndZ(sample.tangent, new THREE.Vector3(0, 0, 1));
  const torchAxis = frameMode === "alternate"
    ? new THREE.Vector3(frame.x.x, frame.x.y, frame.x.z).normalize()
    : new THREE.Vector3(frame.z.x, frame.z.y, frame.z.z).normalize();
  weldTorch.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), torchAxis);
  updateToolFrameGizmo(weldTorchLiveGizmo, sample.position, frame, 42);
  const trace = sampledTrace(activeWeldPath, progress, 64);
  weldTraceLine.geometry.dispose();
  weldTraceLine.geometry = new THREE.BufferGeometry().setFromPoints(trace);
}

function sampleScenePath(points: Point3[], progress: number): { position: THREE.Vector3; tangent: THREE.Vector3 } {
  const vectors = points.map((point) => new THREE.Vector3(point.x, point.y, point.z));
  const lengths = [0];
  for (let index = 1; index < vectors.length; index += 1) {
    lengths.push(lengths[index - 1] + vectors[index - 1].distanceTo(vectors[index]));
  }
  const total = lengths[lengths.length - 1] || 1;
  const target = Math.max(0, Math.min(1, progress)) * total;
  for (let index = 1; index < lengths.length; index += 1) {
    if (lengths[index] < target) continue;
    const start = lengths[index - 1];
    const segment = lengths[index] - start || 1;
    const t = (target - start) / segment;
    const position = vectors[index - 1].clone().lerp(vectors[index], t);
    const tangent = vectors[index].clone().sub(vectors[index - 1]).normalize();
    return { position, tangent };
  }
  return {
    position: vectors[vectors.length - 1].clone(),
    tangent: vectors[vectors.length - 1].clone().sub(vectors[vectors.length - 2]).normalize(),
  };
}

function sampledTrace(points: Point3[], progress: number, count: number): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  const steps = Math.max(2, Math.floor(count * Math.max(0.02, progress)));
  for (let index = 0; index < steps; index += 1) {
    const t = progress <= 0 ? 0 : (index / (steps - 1)) * progress;
    out.push(sampleScenePath(points, t).position);
  }
  return out;
}

function toolFrameFromPose(pose: TCPPose, mode: FrameMode = "backend"): ToolFrame | null {
  if (!pose.tool_x_axis || !pose.tool_y_axis || !pose.tool_z_axis) return null;
  const backend = {
    x: point(pose.tool_x_axis),
    y: point(pose.tool_y_axis),
    z: point(pose.tool_z_axis),
  };
  if (mode === "backend") {
    return backend;
  }
  const tangent = tupleVector(pose.seam_tangent) ?? pointVector(backend.x);
  const altY = tangent.normalize();
  const torchCandidate = pointVector(backend.z).normalize();
  const posePoint = pose.target_xyz_mm ? worldToScene(pose.target_xyz_mm) : null;
  const intoBox = posePoint
    ? new THREE.Vector3(dogBoxWorldCenter.x - posePoint.x, dogBoxWorldCenter.y - posePoint.y, dogBoxWorldCenter.z - posePoint.z).normalize()
    : null;
  const altX = intoBox && torchCandidate.dot(intoBox) < 0
    ? torchCandidate.clone().multiplyScalar(-1)
    : torchCandidate;
  const altZ = new THREE.Vector3().crossVectors(altX, altY).normalize();
  if (altZ.lengthSq() <= 1e-9) return backend;
  altY.crossVectors(altZ, altX).normalize();
  return {
    x: vectorPoint(altX),
    y: vectorPoint(altY),
    z: vectorPoint(altZ),
  };
}

function tupleVector(tuple: PointTuple | undefined): THREE.Vector3 | null {
  return Array.isArray(tuple) ? new THREE.Vector3(tuple[0], tuple[1], tuple[2]).normalize() : null;
}

function pointVector(value: Point3): THREE.Vector3 {
  return new THREE.Vector3(value.x, value.y, value.z);
}

function vectorPoint(value: THREE.Vector3): Point3 {
  return { x: value.x, y: value.y, z: value.z };
}

function frameAxisVector(pose: TCPPose, mode: FrameMode, axis: keyof ToolFrame): THREE.Vector3 | null {
  const frame = toolFrameFromPose(pose, mode);
  return frame ? pointVector(frame[axis]).normalize() : null;
}

function toolFrameFromTangentAndZ(tangent: THREE.Vector3, toolZ: THREE.Vector3): ToolFrame {
  const z = toolZ.clone().normalize();
  const x = tangent.clone().sub(z.clone().multiplyScalar(tangent.dot(z)));
  if (x.lengthSq() <= 1e-9) x.copy(tangent);
  x.normalize();
  const y = new THREE.Vector3().crossVectors(z, x).normalize();
  x.crossVectors(y, z).normalize();
  return {
    x: { x: x.x, y: x.y, z: x.z },
    y: { x: y.x, y: y.y, z: y.z },
    z: { x: z.x, y: z.y, z: z.z },
  };
}

function dispose(object: THREE.Object3D): void {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.isMesh) {
      mesh.geometry.dispose();
      const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      materials.forEach((material) => material.dispose());
      return;
    }
    const line = child as THREE.Line;
    if (line.isLine) {
      line.geometry.dispose();
      const materials = Array.isArray(line.material) ? line.material : [line.material];
      materials.forEach((material) => material.dispose());
      return;
    }
    const sprite = child as THREE.Sprite;
    if (sprite.isSprite) {
      const materials = Array.isArray(sprite.material) ? sprite.material : [sprite.material];
      materials.forEach((material) => {
        if (material.map) material.map.dispose();
        material.dispose();
      });
    }
  });
}

function resize(): void {
  stage.resize();
}

function tick(): void {
  requestAnimationFrame(tick);
  updateWeldAnimation(performance.now());
  controls.update();
  renderer.render(scene, camera);
}

loadButton.addEventListener("click", () => void loadDogBox());
planButton.addEventListener("click", () => selectedEdge && void planEdge(selectedEdge));
strategyTourButton.addEventListener("click", () => void runStrategyTour());
frameTourButton.addEventListener("click", () => void runLoopingMultiAppTour());
frameModeButton.addEventListener("click", () => {
  frameMode = frameMode === "backend" ? "alternate" : "backend";
  syncFrameModeButton();
  pushReport("frame mode", [frameMode === "alternate" ? "mode=+X" : "mode=backend"]);
  if (lastPayload) {
    renderPlan(lastPayload);
    const poses = lastPayload.tesseract_plan?.tcp_poses ?? [];
    const pose = poses[Math.max(0, Math.min(poses.length - 1, Math.floor(poses.length * 0.38)))];
    if (pose?.target_xyz_mm) {
      renderFrameConcepts(pose, frameConceptsForMode());
      frameTourCaptionEl.hidden = false;
      setFrameTourCaption(frameMode === "alternate"
        ? "+X mode: red +X is the into-box torch axis, green +Y follows travel, +Z completes the right-handed frame."
        : "Backend frame: +X is travel, +Z is the 45 degree torch axis, +Y completes the right-handed frame.");
    }
  }
});
sampledPathButton.addEventListener("click", () => {
  selectedPathMethod = "sampled_edge";
  syncPathMethodButtons();
  if (selectedEdge) void planEdge(selectedEdge);
});
threePointPathButton.addEventListener("click", () => {
  if (selectedEdge && !supportedPathMethods(selectedEdge).includes("three_point_arc")) {
    selectedPathMethod = "sampled_edge";
    syncPathMethodButtons();
    addTourLog("3 Point Arc is only valid for standalone circular paths; dog-bone wires use exact sampled CAD paths.");
    void planEdge(selectedEdge);
    return;
  }
  selectedPathMethod = "three_point_arc";
  syncPathMethodButtons();
  if (selectedEdge) void planEdge(selectedEdge);
});
window.addEventListener("resize", resize);
new ResizeObserver(resize).observe(stageEl.parentElement!);
resize();
tick();

function syncPathMethodButtons(): void {
  const threePointSupported = !selectedEdge || supportedPathMethods(selectedEdge).includes("three_point_arc");
  if (!threePointSupported && selectedPathMethod === "three_point_arc") {
    selectedPathMethod = "sampled_edge";
  }
  sampledPathButton.classList.toggle("is-selected", selectedPathMethod === "sampled_edge");
  threePointPathButton.classList.toggle("is-selected", selectedPathMethod === "three_point_arc");
  threePointPathButton.disabled = !threePointSupported;
  threePointPathButton.title = threePointSupported
    ? "Fit an arc through start/mid/end points on circular paths."
    : "Dog-bone wires are exact 4-arc + 4-line CAD paths and cannot be replaced by one 3-point arc.";
}

function supportedPathMethods(edge: TopologyEdge): Array<"sampled_edge" | "three_point_arc"> {
  return edge.path_class === "circle" ? ["sampled_edge", "three_point_arc"] : ["sampled_edge"];
}

function ensureSelectedPathMethod(edge: TopologyEdge): void {
  if (!supportedPathMethods(edge).includes(selectedPathMethod)) {
    selectedPathMethod = "sampled_edge";
  }
  syncPathMethodButtons();
}

syncPathMethodButtons();
syncFrameModeButton();

if (new URLSearchParams(window.location.search).get("loop") === "1") {
  window.setTimeout(() => {
    void runLoopingMultiAppTour();
  }, 500);
}

function syncFrameModeButton(): void {
  frameModeButton.textContent = frameMode === "alternate" ? "Frame: +X Mode" : "Frame: Backend +X Travel";
  frameModeButton.classList.toggle("is-selected", frameMode === "alternate");
}
