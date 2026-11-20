import "./cadquery-shared.css";

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import URDFLoader from "urdf-loader";

export type PointTuple = [number, number, number];
export type Point3 = { x: number; y: number; z: number };
export type ToolFrame = { x: Point3; y: Point3; z: Point3 };

export type TopologyEdge = {
  id: string;
  label?: string;
  part_id?: string;
  geom_type?: string;
  samples: PointTuple[];
  length_mm?: number;
  bounds_min?: PointTuple;
  bounds_max?: PointTuple;
  closed?: boolean;
  source_edge_ids?: string[];
  source_geom_types?: string[];
  source_geom_counts?: Record<string, number>;
  path_class?: "circle" | "dogbone_4arc_4line" | "wire_non_seam" | "raw_fragment" | string;
  is_seam_candidate?: boolean;
};

export type RecommendedSeam = {
  edge_id: string;
  part_id?: string;
  geom_type?: string;
  path_class?: string;
  start_s: number;
  end_s: number;
  reversed?: boolean;
  reason?: string;
};

export type TopologyModel = {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: "mm";
  sample_count: number;
  parts: Array<{ id: string; edge_count: number }>;
  edges: TopologyEdge[];
  seam_candidate_count?: number;
  recommended_seams?: RecommendedSeam[];
};

export type SingleLogHud = {
  stageEl: HTMLElement;
  reportLogEl: HTMLElement;
  tourButton: HTMLButtonElement;
  hiddenRoot: HTMLElement;
};

export type CadqueryStage = {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  controls: OrbitControls;
  resize: () => void;
};

export type CameraTourPose = {
  position: THREE.Vector3;
  target: THREE.Vector3;
  durationMs?: number;
};

export type FrameConcept<TPose> = {
  label: string;
  caption: string;
  color: number;
  scale?: number;
  vector: (pose: TPose) => THREE.Vector3 | null;
};

export type LoadedRobot = THREE.Object3D & {
  joints?: Record<string, { jointType?: string; angle?: number; limit?: { lower?: number; upper?: number } }>;
  robotName?: string;
  setJointValue?: (name: string, ...values: number[]) => boolean;
};

const SHARED_LOG_STORAGE_KEY = "cadquery.v1.sharedTourLog";
const SHARED_LOG_MAX_LINES = 96;

export function renderSingleLogHud(options: {
  app: HTMLElement;
  shellClass: string;
  stageSectionClass: string;
  stageId: string;
  stageClass?: string;
  stageLabel: string;
  hudClass: string;
  title: string;
  activeApp: "seam" | "edge-geometry";
  tourButtonId: string;
  logId: string;
  hiddenHTML: string;
  extraHiddenButtonsHTML?: string;
}): SingleLogHud {
  const seamHref = `${import.meta.env.BASE_URL}seam`;
  const edgeHref = `${import.meta.env.BASE_URL}edge-geometry`;
  options.app.innerHTML = `
    <main class="${options.shellClass}">
      <section class="${options.stageSectionClass}" aria-label="${options.stageLabel}">
        <div id="${options.stageId}" class="${options.stageClass ?? ""}"></div>
        <div class="stage-vignette" aria-hidden="true"></div>
      </section>
      <section class="panel bottom-panel edge-hud-grid cadquery-single-log-hud ${options.hudClass}" aria-label="${options.title} single log HUD">
        <header class="seam-topbar hero cadquery-hero">
          <div>
            <p class="eyebrow cadquery-hero-eyebrow">Rosie Geometry / v1</p>
            <h1 class="cadquery-hero-title">${options.title}</h1>
            <nav class="tool-switcher" aria-label="Geometry tool switcher">
              <a class="${options.activeApp === "seam" ? "is-active" : ""}" href="${seamHref}">Weld Sequence</a>
              <a class="${options.activeApp === "edge-geometry" ? "is-active" : ""}" href="${edgeHref}">Edge Geometry</a>
              <button id="${options.tourButtonId}" class="cadquery-tour-test-button" type="button">Tour Test</button>
              ${options.extraHiddenButtonsHTML ?? ""}
            </nav>
            <pre id="${options.logId}" class="fading-log cadquery-fading-log" aria-live="polite"></pre>
          </div>
        </header>
        <div class="runtime-only cadquery-runtime-only" aria-hidden="true" hidden>
          ${options.hiddenHTML}
        </div>
      </section>
    </main>
  `;
  const reportLogEl = query<HTMLElement>(`#${options.logId}`);
  reportLogEl.textContent = readSharedReportLog().join("\n");

  return {
    stageEl: query<HTMLElement>(`#${options.stageId}`),
    reportLogEl,
    tourButton: query<HTMLButtonElement>(`#${options.tourButtonId}`),
    hiddenRoot: query<HTMLElement>(".cadquery-runtime-only"),
  };
}

export function query<T extends Element>(selector: string): T {
  const el = document.querySelector<T>(selector);
  if (!el) {
    throw new Error(`missing ${selector}`);
  }
  return el;
}

export function appendReport(logEl: HTMLElement, command: string, lines: string[] = [], maxLines = SHARED_LOG_MAX_LINES): void {
  const entries = readSharedReportLog(logEl);
  const nextEntries = [`$ ${command}`, ...lines, ...entries].slice(0, maxLines);
  logEl.textContent = nextEntries.join("\n");
  writeSharedReportLog(nextEntries);
}

export function animateCameraElliptical(options: {
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  pose: CameraTourPose;
  isCancelled?: () => boolean;
}): Promise<void> {
  const { camera, controls, pose, isCancelled } = options;
  const startPosition = camera.position.clone();
  const startTarget = controls.target.clone();
  const startOffset = startPosition.clone().sub(startTarget);
  const endOffset = pose.position.clone().sub(pose.target);
  const startRadius = Math.max(1, Math.hypot(startOffset.x, startOffset.y));
  const endRadius = Math.max(1, Math.hypot(endOffset.x, endOffset.y));
  const startAngle = Math.atan2(startOffset.y, startOffset.x);
  const endAngle = startAngle + shortestAngleDelta(startAngle, Math.atan2(endOffset.y, endOffset.x));
  const durationMs = Math.max(120, pose.durationMs ?? 1000);
  const startedAt = performance.now();
  camera.near = 0.1;
  camera.far = 100000;
  camera.zoom = 1;
  camera.updateProjectionMatrix();

  return new Promise((resolve) => {
    const step = (now: number) => {
      if (isCancelled?.()) {
        resolve();
        return;
      }
      const progress = Math.min(1, Math.max(0, (now - startedAt) / durationMs));
      const eased = easeInOutCubic(progress);
      const target = startTarget.clone().lerp(pose.target, eased);
      const radius = THREE.MathUtils.lerp(startRadius, endRadius, eased);
      const angle = THREE.MathUtils.lerp(startAngle, endAngle, eased);
      const z = THREE.MathUtils.lerp(startOffset.z, endOffset.z, eased);
      camera.position.set(target.x + Math.cos(angle) * radius, target.y + Math.sin(angle) * radius, target.z + z);
      controls.target.copy(target);
      controls.update();
      if (progress < 1) {
        window.requestAnimationFrame(step);
        return;
      }
      resolve();
    };
    window.requestAnimationFrame(step);
  });
}

export function createCadqueryStage(stageEl: HTMLElement, options: {
  cameraPosition: Point3;
  target: Point3;
  gridSize: number;
  gridDivisions: number;
  fog?: { near: number; far: number };
}): CadqueryStage {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050505);
  if (options.fog) {
    scene.fog = new THREE.Fog(0x050505, options.fog.near, options.fog.far);
  }

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100000);
  camera.position.set(options.cameraPosition.x, options.cameraPosition.y, options.cameraPosition.z);
  camera.up.set(0, 0, 1);

  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  stageEl.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(options.target.x, options.target.y, options.target.z);
  controls.object.up.set(0, 0, 1);
  controls.update();

  const grid = new THREE.GridHelper(options.gridSize, options.gridDivisions, 0x4a4a4a, 0x1f1f1f);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.HemisphereLight(0xf0f0f0, 0x151515, 1.6));
  const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
  keyLight.position.set(500, -800, 1200);
  scene.add(keyLight);
  const fillLight = new THREE.DirectionalLight(0xb8b8b8, 0.9);
  fillLight.position.set(-900, 480, 500);
  scene.add(fillLight);

  const resize = () => {
    const host = stageEl.parentElement ?? stageEl;
    const rect = host.getBoundingClientRect();
    const width = Math.max(1, rect.width);
    const height = Math.max(1, rect.height);
    camera.aspect = width / height;
    // Shift the projection center to 2/3 screen width so the fading log owns
    // the left third and the scene focus sits centered in the right two-thirds.
    camera.setViewOffset(width, height, -width / 6, 0, width, height);
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };

  return { scene, camera, renderer, controls, resize };
}

export function createFrameConceptGroup<TPose>(options: {
  pose: TPose;
  origin: THREE.Vector3;
  concepts: FrameConcept<TPose>[];
  groupName: string;
  baseLength?: number;
}): THREE.Group {
  const group = new THREE.Group();
  group.name = options.groupName;
  options.concepts.forEach((concept) => {
    const axis = concept.vector(options.pose);
    if (!axis) return;
    const length = (options.baseLength ?? 84) * (concept.scale ?? 1);
    const end = options.origin.clone().add(axis.normalize().multiplyScalar(length));
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([options.origin, end]),
      new THREE.LineBasicMaterial({ color: concept.color, depthTest: false }),
    );
    line.renderOrder = 80;
    group.add(line);
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(4.6, 18, 12),
      new THREE.MeshBasicMaterial({ color: concept.color, depthTest: false }),
    );
    dot.position.copy(end);
    dot.renderOrder = 81;
    group.add(dot);
    const labelOffset = end.clone().sub(options.origin).normalize().multiplyScalar(28);
    const label = createBillboardLabel(concept.label, concept.color);
    label.position.copy(end.clone().add(labelOffset));
    group.add(label);
  });
  return group;
}

export function createBillboardLabel(text: string, color: number): THREE.Sprite {
  const canvas = document.createElement("canvas");
  const scale = window.devicePixelRatio || 1;
  canvas.width = 320 * scale;
  canvas.height = 78 * scale;
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return new THREE.Sprite();
  }
  ctx.scale(scale, scale);
  ctx.clearRect(0, 0, 320, 78);
  ctx.font = "900 22px SFMono-Regular, Consolas, Liberation Mono, monospace";
  ctx.textBaseline = "middle";
  const colorStyle = `#${color.toString(16).padStart(6, "0")}`;
  ctx.fillStyle = "rgba(0,0,0,0.62)";
  ctx.strokeStyle = colorStyle;
  ctx.lineWidth = 2;
  ctx.fillRect(8, 12, 304, 54);
  ctx.strokeRect(8, 12, 304, 54);
  ctx.fillStyle = colorStyle;
  ctx.fillText(text, 20, 39);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });
  const sprite = new THREE.Sprite(material);
  sprite.name = `frame-billboard-${text.toLowerCase().replace(/[^a-z0-9]+/g, "-")}`;
  sprite.scale.set(112, 27, 1);
  sprite.renderOrder = 90;
  return sprite;
}

export async function loadDogBoxTopology(sampleCount = 64): Promise<TopologyModel> {
  const response = await fetch("/api/cadquery/topology", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ example: "dog_box", sample_count: sampleCount }),
  });
  if (!response.ok) {
    throw new Error(`Dog Box topology HTTP ${response.status}`);
  }
  return (await response.json()) as TopologyModel;
}

export async function buildDogBoxSeamPayload<TPayload>(body: Record<string, unknown>): Promise<TPayload> {
  const response = await fetch("/api/cadquery/topology", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ example: "dog_box", operation: "build_seam", ...body }),
  });
  if (!response.ok) {
    throw new Error(`seam HTTP ${response.status}`);
  }
  return (await response.json()) as TPayload;
}

export function formatLength(length: number | undefined): string {
  if (!Number.isFinite(length)) {
    return "-";
  }
  if ((length as number) >= 1000) {
    return `${((length as number) / 1000).toFixed(3)} m`;
  }
  return `${(length as number).toFixed((length as number) >= 100 ? 1 : 2)} mm`;
}

export function rankedSeamEdges(edges: TopologyEdge[], limit: number): TopologyEdge[] {
  const seamCandidates = edges.filter((edge) => edge.is_seam_candidate);
  const visibleEdges = seamCandidates.length > 0 ? seamCandidates : edges;
  return [...visibleEdges]
    .sort((a, b) => edgeRank(b) - edgeRank(a) || (b.length_mm ?? 0) - (a.length_mm ?? 0))
    .slice(0, limit);
}

export function compactEdgeTitle(edge: TopologyEdge): string {
  return `${edge.label ?? "?"} ${edge.id.replace(/^part_/, "p")}`;
}

export function compactEdgeMeta(edge: TopologyEdge): string {
  const counts = edge.source_geom_counts
    ? ` ${edge.source_geom_counts.ARC ?? 0}a+${edge.source_geom_counts.LINE ?? 0}l`
    : "";
  return `${edge.path_class ?? edge.geom_type ?? "EDGE"} ${formatLength(edge.length_mm)}${edge.part_id ? ` ${edge.part_id}` : ""}${edge.closed ? " closed" : ""}${counts}`;
}

export function point(sample: PointTuple): Point3 {
  return { x: Number(sample[0]), y: Number(sample[1]), z: Number(sample[2]) };
}

export function computeModelOffset(edges: TopologyEdge[]): Point3 {
  const points = edges.flatMap((edge) => edge.samples.map(point));
  if (points.length === 0) {
    return { x: 0, y: 0, z: 0 };
  }
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
  return { x: (min.x + max.x) * 0.5, y: (min.y + max.y) * 0.5, z: min.z };
}

export function createConfiguredUrdfLoader(options: {
  packageName: string;
  packageBaseURL: string;
  onMeshLoaded?: () => void;
  prepareCollada?: (scene: THREE.Object3D) => void;
}): URDFLoader {
  const loader = new URDFLoader();
  (loader as URDFLoader & { packages?: Record<string, string> }).packages = {
    [options.packageName]: options.packageBaseURL,
  };
  const typedLoader = loader as URDFLoader & {
    loadMeshCb: (path: string, manager: THREE.LoadingManager, material: THREE.Material | null, done: (mesh: THREE.Object3D, err?: Error) => void) => void;
  };
  const defaultMeshLoader = typedLoader.loadMeshCb.bind(loader);
  typedLoader.loadMeshCb = ((meshPath: string, manager: THREE.LoadingManager, material: THREE.Material | null, done: (mesh: THREE.Object3D, err?: Error) => void): void => {
    if (!meshPath.toLowerCase().endsWith(".dae")) {
      defaultMeshLoader(meshPath, manager, material, done);
      return;
    }
    new ColladaLoader(manager).load(
      meshPath,
      (collada) => {
        options.prepareCollada?.(collada.scene);
        options.onMeshLoaded?.();
        done(collada.scene);
      },
      undefined,
      (error: unknown) => done(new THREE.Object3D(), error instanceof Error ? error : new Error(String(error))),
    );
  }) as typeof typedLoader.loadMeshCb;
  return loader;
}

function edgeRank(edge: TopologyEdge): number {
  return edge.path_class === "dogbone_4arc_4line" ? 4 : edge.path_class === "circle" ? 3 : 0;
}

function easeInOutCubic(value: number): number {
  return value < 0.5 ? 4 * value * value * value : 1 - Math.pow(-2 * value + 2, 3) / 2;
}

function shortestAngleDelta(from: number, to: number): number {
  return Math.atan2(Math.sin(to - from), Math.cos(to - from));
}

function readSharedReportLog(fallbackEl?: HTMLElement): string[] {
  try {
    const stored = window.sessionStorage.getItem(SHARED_LOG_STORAGE_KEY);
    if (stored) {
      const parsed = JSON.parse(stored);
      if (Array.isArray(parsed)) {
        return parsed.filter((line): line is string => typeof line === "string").slice(0, SHARED_LOG_MAX_LINES);
      }
    }
  } catch {
    // Ignore storage errors; the visible log is still the source of truth.
  }
  return fallbackEl?.textContent ? fallbackEl.textContent.split("\n").filter(Boolean) : [];
}

function writeSharedReportLog(entries: string[]): void {
  try {
    window.sessionStorage.setItem(SHARED_LOG_STORAGE_KEY, JSON.stringify(entries.slice(0, SHARED_LOG_MAX_LINES)));
  } catch {
    // Storage can fail in private contexts; keep the in-memory DOM log only.
  }
}
