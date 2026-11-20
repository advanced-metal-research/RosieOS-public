import "./seam.css";

import * as THREE from "three";
import {
  appendReport,
  animateCameraElliptical,
  buildDogBoxSeamPayload,
  compactEdgeMeta,
  compactEdgeTitle,
  computeModelOffset as computeTopologyModelOffset,
  createCadqueryStage,
  createConfiguredUrdfLoader,
  loadDogBoxTopology,
  rankedSeamEdges,
  renderSingleLogHud,
} from "./cadquery-shared";

type Point3 = { x: number; y: number; z: number };
type PointTuple = [number, number, number];
type ToolFrame = { x: Point3; y: Point3; z: Point3 };
type TourCameraPose = { position: THREE.Vector3; target: THREE.Vector3; durationMs?: number };
type LoadedRobot = THREE.Object3D & {
  joints?: Record<string, { jointType?: string; angle?: number; limit?: { lower?: number; upper?: number } }>;
  robotName?: string;
  setJointValue?: (name: string, ...values: number[]) => boolean;
};

type TopologyEdge = {
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

type RecommendedSeam = {
  edge_id: string;
  part_id?: string;
  geom_type?: string;
  path_class?: string;
  start_s: number;
  end_s: number;
  reversed?: boolean;
  reason?: string;
};

type TopologyModel = {
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

type BackendSeamPayload = {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: "mm";
  workflow: Record<string, unknown>;
  placement?: {
    robot?: string;
    axis?: string;
    table?: string;
    frame?: string;
    origin_mm?: PointTuple;
  };
  seam: {
    edge_id: string;
    part_id: string | null;
    geom_type: string;
    start_s: number;
    end_s: number;
    reversed: boolean;
    length_mm: number | null;
    point_count: number;
    points: PointTuple[];
    world_points_mm?: PointTuple[];
    start_point_mm?: PointTuple;
    stop_point_mm?: PointTuple;
    positioner_axis?: string;
  };
  weld_sequence?: {
    schema?: string;
    robot?: string;
    frame?: string;
    positioner_axis?: string;
    seams?: Array<{
      id: string;
      edge_id: string;
      start_point_mm: PointTuple;
      stop_point_mm: PointTuple;
      point_count: number;
    }>;
  };
  tesseract_plan?: {
    schema?: string;
    planner?: string;
    robot?: string;
    edge_id?: string;
    path_method?: {
      selected?: string;
    };
    tcp_poses?: Array<{
      index: number;
      time_s: number;
      target_xyz_m?: PointTuple;
      target_xyz_mm?: PointTuple;
      seam_tangent?: PointTuple;
      tool_x_axis?: PointTuple;
      tool_y_axis?: PointTuple;
      tool_z_axis?: PointTuple;
      orientation_rpy_rad?: PointTuple;
      work_angle_deg?: number;
    }>;
    motion_plan?: MotionPlan;
  };
};

type CollisionContact = {
  link_a: string;
  link_b: string;
  depth_mm: number;
};

type MotionSample = {
  index: number;
  seam_id: string;
  move: "approach" | "weld" | "retract" | "travel";
  weld_on: boolean;
  time_s: number;
  joint_names?: string[];
  joint_positions_rad?: number[];
  ik_position_error_mm?: number;
  ik_rotation_error_deg?: number;
  collision_contacts?: CollisionContact[];
  target_xyz_mm?: PointTuple;
  seam_tangent?: PointTuple;
  tool_x_axis?: PointTuple;
  tool_y_axis?: PointTuple;
  tool_z_axis?: PointTuple;
  tool_quat_xyzw?: [number, number, number, number];
};

type MotionPlan = {
  trajectory: MotionSample[];
  sample_count: number;
  weld_sample_count: number;
  duration_s: number;
  tool_path_length_mm: number;
  weld_length_mm: number;
  travel_length_mm: number;
  max_tcp_step_mm: number;
  max_frame_rotation_deg: number;
  per_seam_frame_rotation_deg?: number[];
  weld_feed_mm_s?: number;
  travel_feed_mm_s?: number;
  ik?: {
    solver: string;
    robot: string;
    tcp_link: string;
    joint_names: string[];
    max_position_error_mm: number;
    mean_position_error_mm: number;
    max_rotation_error_deg: number;
    max_joint_step_rad: number;
  };
  collision?: {
    status: "checked" | "unavailable" | "skipped";
    reason?: string;
    checker?: string;
    checked_waypoint_count?: number;
    contact_count?: number;
    contact_sample_count?: number;
    max_depth_mm?: number;
    contact_pairs?: string[];
    check_duration_s?: number;
  };
};

type BackendTcpPose = NonNullable<NonNullable<BackendSeamPayload["tesseract_plan"]>["tcp_poses"]>[number];

type MotionSceneSample = {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  timeS: number;
  weldOn: boolean;
  move: MotionSample["move"];
  jointPositionsRad: number[] | null;
  collisionContacts: CollisionContact[] | null;
};

type BackendSequencePayload = {
  schema_version: number;
  tool: string;
  model_id: string;
  filename: string;
  fingerprint: string;
  unit: "mm";
  workflow: Record<string, unknown>;
  placement?: BackendSeamPayload["placement"];
  weld_sequence?: BackendSeamPayload["weld_sequence"];
  tesseract_plan?: BackendSeamPayload["tesseract_plan"];
};

type SavedSeam = {
  id: string;
  edgeId: string;
  partId: string | null;
  geomType: string;
  startS: number;
  endS: number;
  reversed: boolean;
  lengthMm: number | null;
  points: Point3[];
};

type TesseractProgram = {
  command_pose_hash?: string;
  active_tcp_installation_name?: string;
  command_poses?: Array<{
    index: number;
    time_s: number;
    target_xyz_m?: PointTuple;
    ur_base_pose?: [number, number, number, number, number, number];
    orientation_policy?: string;
  }>;
  trajectory?: Array<{
    index: number;
    time_s: number;
    target_xyz_m?: PointTuple;
    active_tcp_xyz_m?: PointTuple;
    orientation_rpy_rad?: PointTuple;
  }>;
};

type SeamRuntime = {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  controls: ReturnType<typeof createCadqueryStage>["controls"];
  renderer: THREE.WebGLRenderer;
  model: TopologyModel | null;
  selectedEdgeId: string | null;
  savedSeams: SavedSeam[];
  tourStep: number;
  weldAnimationProgress: number;
  weldTorchDistanceToPathMm: number | null;
  robotJointAnglesRad: number[] | null;
  robotJointsAnimating: boolean;
  collisionActive: boolean;
  collisionLinks: string[];
  collisionSampleCount: number;
  collisionOutlinedLinks: string[];
};

declare global {
  interface Window {
    __cadquerySeamRuntime?: SeamRuntime;
    __cadqueryRunDogBoxTour?: () => Promise<void>;
  }
}

const EDGE_DEFAULT = 0x8a8a8a;
const EDGE_HOVER = 0xd8d8d8;
const EDGE_SELECTED = 0xffffff;
const SEAM_ORANGE = 0xe6e6e6;
const PICK_RADIUS_MM = 1.8;
const SEAM_RADIUS_MM = 1.2;
const WELD_TCP_SAMPLE_COUNT = 96;
const POSITIONER_TABLE_A_TOP_CENTER_MM: Point3 = { x: 1913.0, y: 685.80026, z: 650.8788 };
const ROBOT_ARM_JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"];
const TOUR_FINAL_EDGE_ID = "part_3:wire_00034";
const TOUR_FINAL_CAMERA_POSITION = new THREE.Vector3(3236.1403281478748, 2793.08152044188, 1337.1877029928213);
const TOUR_FINAL_CONTROLS_TARGET = new THREE.Vector3(1912.9999908447266, 685.8002593994141, 727.2647289550781);
const TOUR_CAMERA_POSES = {
  intro: {
    position: new THREE.Vector3(520, -860, 520),
    target: new THREE.Vector3(120, -80, 120),
    durationMs: 950,
  },
  load: {
    position: new THREE.Vector3(2415, -294, 815),
    target: TOUR_FINAL_CONTROLS_TARGET.clone(),
    durationMs: 1150,
  },
  topology: {
    position: new THREE.Vector3(2515, 226, 855),
    target: TOUR_FINAL_CONTROLS_TARGET.clone(),
    durationMs: 1100,
  },
  seam: {
    position: new THREE.Vector3(2615, 1566, 975),
    target: TOUR_FINAL_CONTROLS_TARGET.clone(),
    durationMs: 1200,
  },
  captured: {
    position: new THREE.Vector3(2855, 2066, 1075),
    target: TOUR_FINAL_CONTROLS_TARGET.clone(),
    durationMs: 1200,
  },
  final: {
    position: TOUR_FINAL_CAMERA_POSITION.clone(),
    target: TOUR_FINAL_CONTROLS_TARGET.clone(),
    durationMs: 1450,
  },
} satisfies Record<string, TourCameraPose>;

const app = document.querySelector<HTMLElement>("#seam-app");
if (!app) {
  throw new Error("missing #seam-app");
}

const hud = renderSingleLogHud({
  app,
  shellClass: "seam-shell",
  stageSectionClass: "seam-stage",
  stageId: "stage",
  stageClass: "stage",
  stageLabel: "Full screen CAD edge stage",
  hudClass: "bottom-panel",
  title: "Weld Sequence",
  activeApp: "seam",
  tourButtonId: "quick-tour-button",
  logId: "seam-report-log",
  hiddenHTML: `
    <div id="status" class="status" aria-live="polite">Load a STEP file to extract exact Geometry edges.</div>
    <div class="title-stats" aria-label="Model metrics">
      <span>Model <strong id="model-id">none</strong></span>
      <span>Parts <strong id="part-count">0</strong></span>
      <span>Edges <strong id="edge-count">0</strong></span>
      <span>Frame <strong>Z-up / mm</strong></span>
    </div>
    <section class="panel-section edge-browser">
      <div class="section-heading">
        <div>
          <p class="section-kicker">Topology Edges</p>
          <h2>Pick a seam source</h2>
        </div>
        <button id="frame-button" class="secondary-action" type="button" disabled>Frame</button>
      </div>
      <div id="edge-list" class="edge-list" role="list"></div>
    </section>
    <section class="panel-section">
      <p class="section-kicker">Selected Edge</p>
      <h2 id="selected-edge">No edge selected</h2>
      <dl class="edge-meta">
        <div><dt>Geometry</dt><dd id="edge-type">-</dd></div>
        <div><dt>Length</dt><dd id="edge-length">-</dd></div>
        <div><dt>Part</dt><dd id="edge-part">-</dd></div>
      </dl>
    </section>
    <section class="panel-section">
      <div class="section-heading">
        <div>
          <p class="section-kicker">Seam Window</p>
          <h2>Trim by arclength</h2>
        </div>
        <span id="seam-point-count" class="pill">0 points</span>
      </div>
      <div class="range-control">
        <label for="start-range">Start <output id="start-output">0%</output></label>
        <input id="start-range" type="range" min="0" max="100" step="1" value="0" disabled />
      </div>
      <div class="range-control">
        <label for="end-range">End <output id="end-output">100%</output></label>
        <input id="end-range" type="range" min="0" max="100" step="1" value="100" disabled />
      </div>
      <div class="button-grid">
        <button id="reverse-button" class="secondary-action" type="button" disabled data-testid="reverse-button">Reverse Direction</button>
        <button id="add-seam-button" class="primary-action" type="button" disabled data-testid="add-seam-button">Add Seam</button>
      </div>
    </section>
    <section class="panel-section">
      <div class="section-heading">
        <div>
          <p class="section-kicker">Seam Set</p>
          <h2>Captured weld seams</h2>
        </div>
        <span id="seam-count" class="pill" data-testid="seam-count">0 seams</span>
      </div>
      <div id="saved-seams" class="saved-seams" data-testid="saved-seams"></div>
    </section>
    <span id="robot-status">loading rosie_1400_v3</span>
    <span id="path-status">loading path plan</span>
    <span id="weld-status">waiting for seam</span>
    <span id="collision-status">collision: no plan</span>
    <span id="file-name">dog_box.step</span>
    <input id="step-file" type="file" accept=".step,.stp,.STEP,.STP" />
    <input id="sample-count" type="number" min="8" max="256" step="8" value="64" />
    <button id="dog-box-button" type="button" data-testid="dog-box-button">Load Dog Box Example</button>
    <button id="extract-button" type="button">Extract Geometry Edges</button>
    <textarea id="seam-json" spellcheck="false" readonly></textarea>
    <button id="copy-button" type="button" disabled>Copy</button>
    <button id="download-button" type="button" disabled>Download Seam JSON</button>
    <div id="tour-caption" class="tour-caption" role="status" aria-live="polite" hidden></div>
  `,
});

const stageEl = hud.stageEl;
const statusEl = mustQuery<HTMLElement>("#status");
const stepFileInput = mustQuery<HTMLInputElement>("#step-file");
const fileNameEl = mustQuery<HTMLElement>("#file-name");
const sampleCountInput = mustQuery<HTMLInputElement>("#sample-count");
const extractButton = mustQuery<HTMLButtonElement>("#extract-button");
const dogBoxButton = mustQuery<HTMLButtonElement>("#dog-box-button");
const modelIdEl = mustQuery<HTMLElement>("#model-id");
const partCountEl = mustQuery<HTMLElement>("#part-count");
const edgeCountEl = mustQuery<HTMLElement>("#edge-count");
const quickTourButton = hud.tourButton;
const reportLogEl = hud.reportLogEl;
const edgeListEl = mustQuery<HTMLElement>("#edge-list");
const frameButton = mustQuery<HTMLButtonElement>("#frame-button");
const selectedEdgeEl = mustQuery<HTMLElement>("#selected-edge");
const edgeTypeEl = mustQuery<HTMLElement>("#edge-type");
const edgeLengthEl = mustQuery<HTMLElement>("#edge-length");
const edgePartEl = mustQuery<HTMLElement>("#edge-part");
const startRange = mustQuery<HTMLInputElement>("#start-range");
const endRange = mustQuery<HTMLInputElement>("#end-range");
const startOutput = mustQuery<HTMLOutputElement>("#start-output");
const endOutput = mustQuery<HTMLOutputElement>("#end-output");
const reverseButton = mustQuery<HTMLButtonElement>("#reverse-button");
const addSeamButton = mustQuery<HTMLButtonElement>("#add-seam-button");
const seamPointCountEl = mustQuery<HTMLElement>("#seam-point-count");
const seamCountEl = mustQuery<HTMLElement>("#seam-count");
const savedSeamsEl = mustQuery<HTMLElement>("#saved-seams");
const robotStatusEl = mustQuery<HTMLElement>("#robot-status");
const pathStatusEl = mustQuery<HTMLElement>("#path-status");
const weldStatusEl = mustQuery<HTMLElement>("#weld-status");
const collisionStatusEl = mustQuery<HTMLElement>("#collision-status");
const seamJSONEl = mustQuery<HTMLTextAreaElement>("#seam-json");
const copyButton = mustQuery<HTMLButtonElement>("#copy-button");
const downloadButton = mustQuery<HTMLButtonElement>("#download-button");
const tourCaptionEl = mustQuery<HTMLElement>("#tour-caption");

const stage = createCadqueryStage(stageEl, {
  cameraPosition: { x: 420, y: -760, z: 420 },
  target: { x: 0, y: 0, z: 0 },
  gridSize: 1200,
  gridDivisions: 24,
  fog: { near: 1400, far: 5400 },
});
const { scene, camera, renderer, controls } = stage;

const axis = new THREE.AxesHelper(140);
scene.add(axis);

let activeModel: TopologyModel | null = null;
let modelGroup: THREE.Group | null = null;
let seamGroup: THREE.Group | null = null;
let robotGroup: THREE.Group | null = null;
let loadedRobot: LoadedRobot | null = null;
let tesseractPathGroup: THREE.Group | null = null;
let weldAnimationGroup: THREE.Group | null = null;
let weldTraceLine: THREE.Line | null = null;
let weldTorch: THREE.Group | null = null;
let weldTorchLiveGizmo: THREE.Group | null = null;
let weldAnimationStartedMS = performance.now();
let activeWeldPath: Point3[] = [];
let activeWeldToolZAxes: Point3[] = [];
let activeWeldToolFrames: ToolFrame[] = [];
let motionPlanGroup: THREE.Group | null = null;
let activeMotionSamples: MotionSceneSample[] = [];
let activeMotionDurationS = 0;
let selectedEdgeId: string | null = null;
let hoveredEdgeId: string | null = null;
let cameraTourAnimationSerial = 0;
let inlineTourRunning = false;
let modelOffset: Point3 = { x: 0, y: 0, z: 0 };
let dogBoxSceneOffset: Point3 = { x: 0, y: 0, z: 0 };
let currentSeamPoints: Point3[] = [];
let currentBackendSeam: BackendSeamPayload | null = null;
let currentBackendSequence: BackendSequencePayload | null = null;
let seamDirectionReversed = false;
let savedSeams: SavedSeam[] = [];
let activeSavedSeamId: string | null = null;
let tourStep = 0;
let activeStepRequest: Record<string, unknown> | null = null;
let seamPreviewRequestSerial = 0;

const edgeLines = new Map<string, THREE.Line>();
const edgePickMeshes = new Map<string, THREE.Mesh>();
const edgeById = new Map<string, TopologyEdge>();
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

window.__cadquerySeamRuntime = {
  scene,
  camera,
  controls,
  renderer,
  model: null,
  selectedEdgeId: null,
  savedSeams,
  tourStep,
  weldAnimationProgress: 0,
  weldTorchDistanceToPathMm: null,
  robotJointAnglesRad: null,
  robotJointsAnimating: false,
  collisionActive: false,
  collisionLinks: [],
  collisionSampleCount: 0,
  collisionOutlinedLinks: [],
};

function mustQuery<T extends Element>(selector: string): T {
  const el = document.querySelector<T>(selector);
  if (!el) {
    throw new Error(`missing ${selector}`);
  }
  return el;
}

function uiAssetURL(pathname: string): string {
  return `${import.meta.env.BASE_URL}${pathname.replace(/^\/+/, "")}`;
}

function metersTupleToScenePoint(value: PointTuple): Point3 {
  return {
    x: value[0] * 1000,
    y: value[1] * 1000,
    z: value[2] * 1000,
  };
}

function worldMmTupleToScenePoint(value: PointTuple): Point3 {
  return {
    x: Number(value[0]),
    y: Number(value[1]),
    z: Number(value[2]),
  };
}

async function loadRobotAndPlannerOverlays(): Promise<void> {
  await Promise.allSettled([loadRosie1400V3Robot(), loadTesseractPathPlan()]);
}

function loadRosie1400V3Robot(): Promise<void> {
  return new Promise((resolve) => {
    let renderedMeshCount = 0;
    const loader = createConfiguredUrdfLoader({
      packageName: "rosie_1400_v3",
      packageBaseURL: uiAssetURL("robot/rosie_1400_v3/"),
      prepareCollada: applyRobotV3Materials,
      onMeshLoaded: () => {
        renderedMeshCount += 1;
        robotStatusEl.textContent = `rosie_1400_v3 URDF loaded (${renderedMeshCount} meshes)`;
      },
    });
    loader.load(
      uiAssetURL("robot/rosie_1400_v3/robot.urdf"),
      (robot: LoadedRobot) => {
        if (robotGroup) {
          scene.remove(robotGroup);
          disposeObject(robotGroup);
        }
        loadedRobot = robot;
        collisionOutlineObjects = [];
        robotGroup = new THREE.Group();
        robotGroup.name = "rosie_1400_v3_urdf_overlay";
        robot.name = robot.robotName ?? "rosie_1400_v3";
        robot.scale.setScalar(1000);
        robot.position.set(0, 0, 0);
        robot.rotation.set(0, 0, 0);
        applyRobotV3Materials(robot);
        robotGroup.add(robot);
        scene.add(robotGroup);
        robotStatusEl.textContent = `rosie_1400_v3 URDF loaded (${renderedMeshCount} meshes)`;
        // A plan may have arrived before the robot finished loading.
        installCollisionOutlines();
        resolve();
      },
      undefined,
      (error: unknown) => {
        robotStatusEl.textContent = `URDF unavailable: ${error instanceof Error ? error.message : String(error)}`;
        resolve();
      },
    );
  });
}

async function loadTesseractPathPlan(): Promise<void> {
  try {
    const response = await fetch(uiAssetURL("tesseract/robot_v7_program.json"), { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const program = (await response.json()) as TesseractProgram;
    renderTesseractPathPlan(program);
  } catch (error) {
    pathStatusEl.textContent = `path unavailable: ${error instanceof Error ? error.message : String(error)}`;
  }
}

function renderTesseractPathPlan(program: TesseractProgram): void {
  if (tesseractPathGroup) {
    scene.remove(tesseractPathGroup);
    disposeObject(tesseractPathGroup);
  }
  tesseractPathGroup = new THREE.Group();
  tesseractPathGroup.name = "robot_v7_tesseract_tcp_path";
  const poses = Array.isArray(program.command_poses) ? program.command_poses : [];
  const points = poses
    .map((pose) => (Array.isArray(pose.target_xyz_m) ? metersTupleToScenePoint(pose.target_xyz_m) : null))
    .filter((point): point is Point3 => point !== null);
  if (points.length >= 2) {
    const line = createPolyline(points, 0xd8d8d8, 0.95);
    line.name = "tesseract-command-pose-path";
    line.renderOrder = 22;
    tesseractPathGroup.add(line);
  }
  poses.forEach((pose, index) => {
    if (!Array.isArray(pose.target_xyz_m)) return;
    if (index % Math.max(1, Math.floor(poses.length / 16)) !== 0 && index !== poses.length - 1) return;
    const position = metersTupleToScenePoint(pose.target_xyz_m);
    const quaternion = Array.isArray(pose.ur_base_pose)
      ? quaternionFromRotationVector(pose.ur_base_pose[3], pose.ur_base_pose[4], pose.ur_base_pose[5])
      : new THREE.Quaternion();
    const gizmo = createTCPFrameGizmo(position, quaternion, 34);
    gizmo.name = `tcp-gizmo-${pose.index}`;
    tesseractPathGroup!.add(gizmo);
  });
  scene.add(tesseractPathGroup);
  pathStatusEl.textContent = `${points.length} Tesseract TCP poses`;
}

function renderBackendTesseractPlan(plan: NonNullable<BackendSeamPayload["tesseract_plan"]>, seamCount?: number): void {
  if (tesseractPathGroup) {
    scene.remove(tesseractPathGroup);
    disposeObject(tesseractPathGroup);
  }
  tesseractPathGroup = new THREE.Group();
  tesseractPathGroup.name = "cadquery_backend_tesseract_weld_plan";
  const poses = Array.isArray(plan.tcp_poses) ? plan.tcp_poses : [];
  const points = poses
    .map((pose) => {
      if (Array.isArray(pose.target_xyz_mm)) return worldMmTupleToScenePoint(pose.target_xyz_mm);
      if (Array.isArray(pose.target_xyz_m)) return metersTupleToScenePoint(pose.target_xyz_m);
      return null;
    })
    .filter((point): point is Point3 => point !== null);
  if (points.length >= 2) {
    const line = createPolyline(points, 0xd8d8d8, 1);
    line.name = "backend-tesseract-weld-tcp-path";
    line.renderOrder = 35;
    tesseractPathGroup.add(line);
  }
  points.forEach((point, index) => {
    const marker = createPointMarker(point, 0xf2f2f2, 2.2, true);
    marker.name = `backend-tcp-target-dot-${index}`;
    marker.renderOrder = 37;
    tesseractPathGroup!.add(marker);
  });
  poses.forEach((pose, index) => {
    if (index % Math.max(1, Math.floor(poses.length / 18)) !== 0 && index !== poses.length - 1) return;
    const source = pose.target_xyz_mm
      ? worldMmTupleToScenePoint(pose.target_xyz_mm)
      : pose.target_xyz_m
        ? metersTupleToScenePoint(pose.target_xyz_m)
        : null;
    if (!source) return;
    const frame = toolFrameFromPose(pose);
    const gizmo = frame
      ? createToolFrameGizmo(source, frame, 42, 0xfff0c2)
      : createTCPFrameGizmo(
          source,
          Array.isArray(pose.orientation_rpy_rad)
            ? new THREE.Quaternion().setFromEuler(new THREE.Euler(pose.orientation_rpy_rad[0], pose.orientation_rpy_rad[1], pose.orientation_rpy_rad[2], "XYZ"))
            : new THREE.Quaternion(),
          42,
        );
    gizmo.name = `backend-tcp-gizmo-${pose.index}`;
    tesseractPathGroup!.add(gizmo);
  });
  scene.add(tesseractPathGroup);
  pathStatusEl.textContent =
    typeof seamCount === "number"
      ? `${points.length} Tesseract TCP poses / ${seamCount} seams / +X tangent / 45deg plane`
      : `${points.length} Tesseract TCP poses on ${plan.robot ?? "rosie_1400_v3"} / +X tangent / 45deg plane`;
}

function robotV3MaterialForName(name: string): THREE.Material {
  const lower = name.toLowerCase();
  const color = lower.includes("floor")
    ? 0x3b4048
    : lower.includes("h_frame")
      ? 0xa6adb8
      : lower.includes("positioner")
        ? 0x757c86
        : ["link_1", "link_3", "link_5"].some((token) => lower.includes(token))
          ? 0xe2e5e9
          : 0x9aa1aa;
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: lower.includes("floor") ? 0.22 : 0.58,
    depthTest: true,
    depthWrite: false,
    side: THREE.DoubleSide,
  });
}

function applyRobotV3Materials(rootObject: THREE.Object3D): void {
  rootObject.traverse((object) => {
    const mesh = object as THREE.Mesh & { isMesh?: boolean; material?: THREE.Material | THREE.Material[] };
    if (!mesh.isMesh) {
      return;
    }
    mesh.material = robotV3MaterialForName(mesh.name || rootObject.name || "");
    mesh.castShadow = true;
    mesh.receiveShadow = true;
  });
}

function toPoint(sample: PointTuple): Point3 {
  return { x: Number(sample[0]), y: Number(sample[1]), z: Number(sample[2]) };
}

function toSceneVector(point: Point3): THREE.Vector3 {
  return new THREE.Vector3(point.x + dogBoxSceneOffset.x, point.y + dogBoxSceneOffset.y, point.z + dogBoxSceneOffset.z);
}

function formatLength(length: number | undefined): string {
  if (typeof length !== "number" || !Number.isFinite(length)) {
    return "-";
  }
  if (length >= 1000) {
    return `${(length / 1000).toFixed(3)} m`;
  }
  return `${length.toFixed(length >= 100 ? 1 : 2)} mm`;
}

function status(message: string): void {
  statusEl.textContent = message;
  pushReport("status", [message]);
}

function pushReport(command: string, lines: string[] = []): void {
  appendReport(reportLogEl, command, lines);
}

function setTourStep(nextStep: number): void {
  tourStep = Math.max(0, Math.min(5, nextStep));
  window.__cadquerySeamRuntime!.tourStep = tourStep;
  document.documentElement.dataset.tourStep = String(tourStep);
}

function maybeAdvanceTour(minStep: number): void {
  if (tourStep < minStep) {
    setTourStep(minStep);
  }
}

function resetTourLog(): void {
  // Tour UI is intentionally hidden; keep this as a compatibility no-op.
}

function addTourLog(text: string): void {
  void text;
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function fileToBase64(file: File): Promise<string> {
  const buffer = await file.arrayBuffer();
  let binary = "";
  const bytes = new Uint8Array(buffer);
  const chunkSize = 0x8000;
  for (let offset = 0; offset < bytes.length; offset += chunkSize) {
    binary += String.fromCharCode(...bytes.subarray(offset, offset + chunkSize));
  }
  return btoa(binary);
}

async function postTopologyRequest(body: Record<string, unknown>, label: string): Promise<void> {
  extractButton.disabled = true;
  dogBoxButton.disabled = true;
  status(`Extracting exact edges from ${label} with Geometry backend...`);
  try {
    activeStepRequest = { ...body };
    const response = await fetch("/api/cadquery/topology", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = (await response.json()) as TopologyModel | { error?: string };
    if (!response.ok) {
      throw new Error("error" in data && data.error ? data.error : `HTTP ${response.status}`);
    }
    loadTopology(data as TopologyModel);
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    status(`Extraction failed: ${message}`);
  } finally {
    extractButton.disabled = false;
    dogBoxButton.disabled = false;
  }
}

async function extractTopology(): Promise<void> {
  const file = stepFileInput.files?.[0] ?? null;
  if (!file) {
    status("Choose a STEP file first or load the Dog Box example.");
    return;
  }
  const stepBase64 = await fileToBase64(file);
  const sampleCount = Math.max(8, Math.min(256, Number(sampleCountInput.value) || 64));
  await postTopologyRequest(
    {
      filename: file.name,
      step_base64: stepBase64,
      sample_count: sampleCount,
    },
    file.name,
  );
}

async function loadDogBoxExample(): Promise<void> {
  fileNameEl.textContent = "dog_box.step";
  const sampleCount = Math.max(8, Math.min(256, Number(sampleCountInput.value) || 64));
  extractButton.disabled = true;
  dogBoxButton.disabled = true;
  status("Extracting exact edges from Dog Box example with Geometry backend...");
  try {
    activeStepRequest = { example: "dog_box", sample_count: sampleCount };
    loadTopology(await loadDogBoxTopology(sampleCount));
  } catch (error) {
    status(`Extraction failed: ${error instanceof Error ? error.message : String(error)}`);
  } finally {
    extractButton.disabled = false;
    dogBoxButton.disabled = false;
  }
}

function loadTopology(model: TopologyModel): void {
  clearTopology();
  activeModel = model;
  window.__cadquerySeamRuntime!.model = model;
  savedSeams = [];
  activeSavedSeamId = null;
  window.__cadquerySeamRuntime!.savedSeams = savedSeams;
  renderSavedSeams();
  maybeAdvanceTour(1);
  modelOffset = computeTopologyModelOffset(model.edges);
  dogBoxSceneOffset = {
    x: POSITIONER_TABLE_A_TOP_CENTER_MM.x - modelOffset.x,
    y: POSITIONER_TABLE_A_TOP_CENTER_MM.y - modelOffset.y,
    z: POSITIONER_TABLE_A_TOP_CENTER_MM.z - modelOffset.z,
  };
  modelIdEl.textContent = model.model_id.slice(0, 20);
  modelIdEl.title = model.model_id;
  partCountEl.textContent = String(model.parts.length);
  edgeCountEl.textContent = String(model.edges.length);
  frameButton.disabled = false;
  void loadRobotAndPlannerOverlays();

  const group = new THREE.Group();
  group.name = "cadquery-topology-edges";
  scene.add(group);
  modelGroup = group;

  model.edges.forEach((edge) => {
    edgeById.set(edge.id, edge);
    const points = edge.samples.map(toPoint);
    if (points.length < 2) {
      return;
    }
    const line = createEdgeLine(points, EDGE_DEFAULT, 0.68);
    line.userData.edgeId = edge.id;
    group.add(line);
    edgeLines.set(edge.id, line);

    const pickMesh = createTube(points, PICK_RADIUS_MM, EDGE_DEFAULT, 0);
    if (pickMesh) {
      pickMesh.userData.edgeId = edge.id;
      group.add(pickMesh);
      edgePickMeshes.set(edge.id, pickMesh);
    }
  });

  renderEdgeList(model.edges);
  if (!inlineTourRunning) {
    frameTopology();
  }
  pushReport("cadquery topology dog_box.step", [
    `model=${model.model_id.slice(0, 20)} parts=${model.parts.length} edges=${model.edges.length}`,
  ]);
  status(`${model.filename}: extracted ${model.edges.length} edges from ${model.parts.length} part(s).`);
  addTourLog(`Scanned ${model.edges.length} exact edges from ${model.filename}.`);
  maybeAdvanceTour(2);
}

function clearTopology(): void {
  if (modelGroup) {
    scene.remove(modelGroup);
    disposeObject(modelGroup);
    modelGroup = null;
  }
  if (seamGroup) {
    scene.remove(seamGroup);
    disposeObject(seamGroup);
    seamGroup = null;
  }
  if (weldAnimationGroup) {
    scene.remove(weldAnimationGroup);
    disposeObject(weldAnimationGroup);
    weldAnimationGroup = null;
    weldTraceLine = null;
    weldTorch = null;
    weldTorchLiveGizmo = null;
  }
  if (motionPlanGroup) {
    scene.remove(motionPlanGroup);
    disposeObject(motionPlanGroup);
    motionPlanGroup = null;
  }
  activeMotionSamples = [];
  activeMotionDurationS = 0;
  window.__cadquerySeamRuntime!.robotJointsAnimating = false;
  window.__cadquerySeamRuntime!.robotJointAnglesRad = null;
  window.__cadquerySeamRuntime!.collisionSampleCount = 0;
  applyCollisionFeedback(null);
  pendingCollisionOutlineLinks = [];
  clearCollisionOutlines();
  collisionStatusEl.textContent = "collision: no plan";
  activeModel = null;
  selectedEdgeId = null;
  hoveredEdgeId = null;
  edgeById.clear();
  edgeLines.clear();
  edgePickMeshes.clear();
  edgeListEl.replaceChildren();
  selectedEdgeEl.textContent = "No edge selected";
  edgeTypeEl.textContent = "-";
  edgeLengthEl.textContent = "-";
  edgePartEl.textContent = "-";
  seamJSONEl.value = "";
  currentSeamPoints = [];
  activeWeldPath = [];
  activeWeldToolZAxes = [];
  activeWeldToolFrames = [];
  currentBackendSeam = null;
  dogBoxSceneOffset = { x: 0, y: 0, z: 0 };
  seamDirectionReversed = false;
  savedSeams = [];
  activeSavedSeamId = null;
  renderSavedSeams();
  copyButton.disabled = true;
  downloadButton.disabled = true;
  reverseButton.disabled = true;
  addSeamButton.disabled = true;
  startRange.disabled = true;
  endRange.disabled = true;
  window.__cadquerySeamRuntime!.model = null;
  window.__cadquerySeamRuntime!.selectedEdgeId = null;
}

function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) {
      const line = child as THREE.Line;
      if (line.isLine) {
        line.geometry.dispose();
        const material = line.material;
        if (Array.isArray(material)) {
          material.forEach((item) => item.dispose());
        } else {
          material.dispose();
        }
      }
      return;
    }
    mesh.geometry.dispose();
    const material = mesh.material;
    if (Array.isArray(material)) {
      material.forEach((item) => item.dispose());
    } else {
      material.dispose();
    }
  });
}

function computeModelOffset(edges: TopologyEdge[]): Point3 {
  const points = edges.flatMap((edge) => edge.samples.map(toPoint));
  if (points.length === 0) {
    return { x: 0, y: 0, z: 0 };
  }
  const min = {
    x: Math.min(...points.map((point) => point.x)),
    y: Math.min(...points.map((point) => point.y)),
    z: Math.min(...points.map((point) => point.z)),
  };
  const max = {
    x: Math.max(...points.map((point) => point.x)),
    y: Math.max(...points.map((point) => point.y)),
    z: Math.max(...points.map((point) => point.z)),
  };
  return { x: (min.x + max.x) * 0.5, y: (min.y + max.y) * 0.5, z: min.z };
}

function createEdgeLine(points: Point3[], color: number, opacity: number): THREE.Line {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map(toSceneVector));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    depthTest: false,
    depthWrite: false,
  });
  const line = new THREE.Line(geometry, material);
  line.renderOrder = 10;
  return line;
}

function createTube(points: Point3[], radius: number, color: number, opacity: number): THREE.Mesh | null {
  if (points.length < 2) {
    return null;
  }
  const vectors = points.map(toSceneVector);
  return createTubeFromVectors(vectors, radius, color, opacity);
}

function createTubeScene(points: Point3[], radius: number, color: number, opacity: number): THREE.Mesh | null {
  if (points.length < 2) {
    return null;
  }
  const vectors = points.map((point) => new THREE.Vector3(point.x, point.y, point.z));
  return createTubeFromVectors(vectors, radius, color, opacity);
}

function createTubeFromVectors(vectors: THREE.Vector3[], radius: number, color: number, opacity: number): THREE.Mesh | null {
  const curve = new THREE.CatmullRomCurve3(vectors, false, "centripetal");
  const geometry = new THREE.TubeGeometry(curve, Math.max(4, vectors.length * 2), radius, 8, false);
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    depthTest: false,
    depthWrite: false,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.renderOrder = 14;
  return mesh;
}

function createPolyline(points: Point3[], color: number, opacity: number): THREE.Line {
  const geometry = new THREE.BufferGeometry().setFromPoints(points.map((point) => new THREE.Vector3(point.x, point.y, point.z)));
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    depthTest: false,
    depthWrite: false,
  });
  return new THREE.Line(geometry, material);
}

function quaternionFromRotationVector(rx: number, ry: number, rz: number): THREE.Quaternion {
  const axis = new THREE.Vector3(rx, ry, rz);
  const angle = axis.length();
  if (angle <= 1e-9) {
    return new THREE.Quaternion();
  }
  axis.normalize();
  return new THREE.Quaternion().setFromAxisAngle(axis, angle);
}

function createTCPFrameGizmo(position: Point3, quaternion: THREE.Quaternion, size: number): THREE.Group {
  const group = new THREE.Group();
  group.position.set(position.x, position.y, position.z);
  group.quaternion.copy(quaternion);
  group.renderOrder = 30;
  const axes: Array<{ direction: THREE.Vector3; color: number; name: string }> = [
    { direction: new THREE.Vector3(1, 0, 0), color: 0xff4b4b, name: "tcp-x" },
    { direction: new THREE.Vector3(0, 1, 0), color: 0x72d6a4, name: "tcp-y" },
    { direction: new THREE.Vector3(0, 0, 1), color: 0x38bdf8, name: "tcp-z" },
  ];
  axes.forEach((axis) => {
    const line = createPolyline(
      [
        { x: 0, y: 0, z: 0 },
        { x: axis.direction.x * size, y: axis.direction.y * size, z: axis.direction.z * size },
      ],
      axis.color,
      0.95,
    );
    line.name = axis.name;
    group.add(line);
  });
  const dot = createPointMarker({ x: 0, y: 0, z: 0 }, 0xffd166, Math.max(2.5, size * 0.08), true);
  dot.name = "tcp-origin";
  group.add(dot);
  return group;
}

function createToolFrameGizmo(position: Point3, frame: ToolFrame, size: number, originColor = 0xffd166): THREE.Group {
  const group = new THREE.Group();
  group.position.set(position.x, position.y, position.z);
  group.renderOrder = 38;
  [
    { axis: frame.x, color: 0xff4b4b, name: "tcp-x" },
    { axis: frame.y, color: 0x72d6a4, name: "tcp-y" },
    { axis: frame.z, color: 0x38bdf8, name: "tcp-z" },
  ].forEach((entry) => {
    const line = createAxisLine(entry.name, entry.color);
    group.add(line);
  });
  const dot = createPointMarker({ x: 0, y: 0, z: 0 }, originColor, Math.max(2.4, size * 0.08), true);
  dot.name = "tcp-origin";
  group.add(dot);
  updateToolFrameGizmo(group, position, frame, size);
  return group;
}

function createAxisLine(name: string, color: number): THREE.Line {
  const line = createPolyline([{ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }], color, 0.98);
  line.name = name;
  line.renderOrder = 43;
  return line;
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

function toolFrameFromPose(pose: BackendTcpPose): ToolFrame | null {
  if (!Array.isArray(pose.tool_x_axis) || !Array.isArray(pose.tool_y_axis) || !Array.isArray(pose.tool_z_axis)) {
    return null;
  }
  return {
    x: toPoint(pose.tool_x_axis),
    y: toPoint(pose.tool_y_axis),
    z: toPoint(pose.tool_z_axis),
  };
}

function toolFrameFromTangentAndZ(tangent: THREE.Vector3, toolZ: THREE.Vector3): ToolFrame {
  const z = toolZ.clone().normalize();
  const x = tangent.clone().sub(z.clone().multiplyScalar(tangent.dot(z)));
  if (x.lengthSq() <= 1e-9) {
    x.copy(tangent);
  }
  x.normalize();
  const y = new THREE.Vector3().crossVectors(z, x).normalize();
  x.crossVectors(y, z).normalize();
  return {
    x: { x: x.x, y: x.y, z: x.z },
    y: { x: y.x, y: y.y, z: y.z },
    z: { x: z.x, y: z.y, z: z.z },
  };
}

function createPointMarker(point: Point3, color: number, radius: number, sceneSpace = false): THREE.Mesh {
  const geometry = new THREE.SphereGeometry(radius, 18, 12);
  const material = new THREE.MeshBasicMaterial({
    color,
    depthTest: false,
    depthWrite: false,
  });
  const marker = new THREE.Mesh(geometry, material);
  marker.position.copy(sceneSpace ? new THREE.Vector3(point.x, point.y, point.z) : toSceneVector(point));
  marker.renderOrder = 32;
  return marker;
}

function createDirectionArrow(points: Point3[], sceneSpace = false): THREE.Mesh | null {
  if (points.length < 3) {
    return null;
  }
  const midIndex = Math.max(1, Math.min(points.length - 2, Math.floor(points.length * 0.62)));
  const toVector = (point: Point3) => (sceneSpace ? new THREE.Vector3(point.x, point.y, point.z) : toSceneVector(point));
  const previous = toVector(points[midIndex - 1]);
  const current = toVector(points[midIndex]);
  const next = toVector(points[midIndex + 1]);
  const direction = new THREE.Vector3().subVectors(next, previous);
  if (direction.lengthSq() <= 1e-9) {
    return null;
  }
  direction.normalize();
  const geometry = new THREE.ConeGeometry(5.6, 16, 18);
  const material = new THREE.MeshBasicMaterial({
    color: 0xd8d8d8,
    depthTest: false,
    depthWrite: false,
  });
  const arrow = new THREE.Mesh(geometry, material);
  arrow.position.copy(current);
  arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  arrow.renderOrder = 34;
  arrow.name = "seam-travel-direction";
  return arrow;
}

function quaternionFromToolQuat(quat: [number, number, number, number] | undefined, fallbackZ: Point3): THREE.Quaternion {
  if (quat && quat.every((value) => Number.isFinite(value))) {
    return new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]).normalize();
  }
  // Fallback: orient a default frame so local +Z points along the tool axis.
  const z = new THREE.Vector3(fallbackZ.x, fallbackZ.y, fallbackZ.z).normalize();
  return new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), z);
}

function toolFrameFromQuaternion(q: THREE.Quaternion): ToolFrame {
  const x = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
  const y = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
  const z = new THREE.Vector3(0, 0, 1).applyQuaternion(q);
  return { x: { x: x.x, y: x.y, z: x.z }, y: { x: y.x, y: y.y, z: y.z }, z: { x: z.x, y: z.y, z: z.z } };
}

function buildMotionSceneSamples(plan: MotionPlan): MotionSceneSample[] {
  return plan.trajectory
    .map((sample) => {
      if (!sample.target_xyz_mm) {
        return null;
      }
      const position = worldMmTupleToScenePoint(sample.target_xyz_mm);
      const fallbackZ = sample.tool_z_axis ? toPoint(sample.tool_z_axis) : { x: 0, y: 0, z: 1 };
      const quaternion = quaternionFromToolQuat(sample.tool_quat_xyzw, fallbackZ);
      return {
        position: new THREE.Vector3(position.x, position.y, position.z),
        quaternion,
        timeS: sample.time_s,
        weldOn: sample.weld_on,
        move: sample.move,
        jointPositionsRad:
          Array.isArray(sample.joint_positions_rad) && sample.joint_positions_rad.length === 6
            ? sample.joint_positions_rad.map(Number)
            : null,
        collisionContacts:
          Array.isArray(sample.collision_contacts) && sample.collision_contacts.length > 0
            ? sample.collision_contacts
            : null,
      } satisfies MotionSceneSample;
    })
    .filter((sample): sample is MotionSceneSample => sample !== null);
}

function renderMotionPlan(plan: MotionPlan): void {
  if (motionPlanGroup) {
    scene.remove(motionPlanGroup);
    disposeObject(motionPlanGroup);
    motionPlanGroup = null;
  }
  const samples = buildMotionSceneSamples(plan);
  activeMotionSamples = samples;
  activeMotionDurationS = plan.duration_s || 0;
  if (samples.length < 2) {
    return;
  }
  const group = new THREE.Group();
  group.name = "cadquery-tesseract-motion-plan";
  group.renderOrder = 38;

  // Split the trajectory into weld-on and travel runs so the arc-live portion
  // reads hot orange while approach/retract/travel stay neutral gray.
  let runStart = 0;
  for (let index = 1; index <= samples.length; index += 1) {
    const runEnded = index === samples.length || samples[index].weldOn !== samples[runStart].weldOn;
    if (!runEnded) {
      continue;
    }
    const runEndInclusive = Math.min(index, samples.length - 1);
    const points = samples.slice(runStart, runEndInclusive + 1).map((sample) => sample.position);
    if (points.length >= 2) {
      const weldOn = samples[runStart].weldOn;
      const line = createPolyline(points, weldOn ? 0xffa53d : 0x9a9a9a, weldOn ? 1 : 0.7);
      line.name = weldOn ? "motion-weld-run" : "motion-travel-run";
      line.renderOrder = weldOn ? 41 : 39;
      group.add(line);
    }
    runStart = index;
  }

  // TCP frame gizmos along the path; keep the count bounded for a dense plan.
  const stride = Math.max(1, Math.ceil(samples.length / 26));
  samples.forEach((sample, index) => {
    const boundary = index === 0 || index === samples.length - 1 || sample.weldOn !== samples[index - 1].weldOn;
    if (!boundary && index % stride !== 0) {
      return;
    }
    const gizmo = createToolFrameGizmo(
      { x: sample.position.x, y: sample.position.y, z: sample.position.z },
      toolFrameFromQuaternion(sample.quaternion),
      boundary ? 44 : 30,
      sample.weldOn ? 0xffb066 : 0xf2f2f2,
    );
    gizmo.name = `motion-gizmo-${index}`;
    group.add(gizmo);
  });

  // Robot/v4 pattern: contact waypoints read RED in the overlay.
  samples.forEach((sample, index) => {
    if (!sample.collisionContacts) {
      return;
    }
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(6.5, 14, 10),
      new THREE.MeshBasicMaterial({ color: 0xff4444, transparent: true, opacity: 0.9, depthTest: false, depthWrite: false }),
    );
    marker.position.copy(sample.position);
    marker.renderOrder = 44;
    marker.name = `motion-collision-contact-${index}`;
    group.add(marker);
  });

  scene.add(group);
  motionPlanGroup = group;
}

type MotionPlaybackSample = {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
  weldOn: boolean;
  jointPositionsRad: number[] | null;
  collisionContacts: CollisionContact[] | null;
};

function lerpJointPositions(a: number[] | null, b: number[] | null, t: number): number[] | null {
  if (!a || !b || a.length !== b.length) {
    return a ?? b;
  }
  return a.map((value, index) => value + (b[index] - value) * t);
}

function sampleMotionPlan(timeS: number): MotionPlaybackSample | null {
  if (activeMotionSamples.length === 0) {
    return null;
  }
  if (activeMotionSamples.length === 1) {
    const only = activeMotionSamples[0];
    return {
      position: only.position.clone(),
      quaternion: only.quaternion.clone(),
      weldOn: only.weldOn,
      jointPositionsRad: only.jointPositionsRad ? [...only.jointPositionsRad] : null,
      collisionContacts: only.collisionContacts,
    };
  }
  const clamped = Math.max(0, Math.min(activeMotionDurationS, timeS));
  for (let index = 1; index < activeMotionSamples.length; index += 1) {
    const prev = activeMotionSamples[index - 1];
    const next = activeMotionSamples[index];
    if (next.timeS < clamped) {
      continue;
    }
    const span = next.timeS - prev.timeS;
    const t = span > 1e-6 ? (clamped - prev.timeS) / span : 0;
    const position = prev.position.clone().lerp(next.position, t);
    const quaternion = prev.quaternion.clone().slerp(next.quaternion, t);
    const nearest = t < 0.5 ? prev : next;
    return {
      position,
      quaternion,
      weldOn: nearest.weldOn,
      jointPositionsRad: lerpJointPositions(prev.jointPositionsRad, next.jointPositionsRad, t),
      collisionContacts: nearest.collisionContacts,
    };
  }
  const last = activeMotionSamples[activeMotionSamples.length - 1];
  return {
    position: last.position.clone(),
    quaternion: last.quaternion.clone(),
    weldOn: last.weldOn,
    jointPositionsRad: last.jointPositionsRad ? [...last.jointPositionsRad] : null,
    collisionContacts: last.collisionContacts,
  };
}

// Robot/v4 pattern: links in contact render RED during plan playback. The
// original material of every tinted mesh is kept so clearing the contact
// restores the exact previous appearance.
const collisionTintOriginalMaterials = new Map<THREE.Mesh, THREE.Material | THREE.Material[]>();
const collisionTintMaterial = new THREE.MeshBasicMaterial({
  color: 0xff3b30,
  transparent: true,
  opacity: 0.85,
  side: THREE.DoubleSide,
});
let activeCollisionLinkKey = "";

// Persistent red edge-line outlines on BOTH members of every colliding link
// pair, installed when a checked plan arrives so the contacts are visible for
// the whole tour, not only in the instant playback crosses a contact
// waypoint. Lines are parented to the link meshes so they follow the joint
// animation.
const collisionOutlineMaterial = new THREE.LineBasicMaterial({
  color: 0xff2d20,
  transparent: true,
  opacity: 0.95,
  depthTest: false,
});
let collisionOutlineObjects: THREE.LineSegments[] = [];
let pendingCollisionOutlineLinks: string[] = [];

function clearCollisionOutlines(): void {
  for (const line of collisionOutlineObjects) {
    line.parent?.remove(line);
    line.geometry.dispose();
  }
  collisionOutlineObjects = [];
  window.__cadquerySeamRuntime!.collisionOutlinedLinks = [];
}

function installCollisionOutlines(): void {
  clearCollisionOutlines();
  if (!loadedRobot || pendingCollisionOutlineLinks.length === 0) {
    return;
  }
  const outlined: string[] = [];
  for (const link of pendingCollisionOutlineLinks) {
    const linkObject = loadedRobot.getObjectByName(link);
    if (!linkObject) {
      continue;
    }
    let added = false;
    linkObject.traverse((child) => {
      const mesh = child as THREE.Mesh;
      if (!mesh.isMesh || !mesh.geometry) {
        return;
      }
      const line = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 24), collisionOutlineMaterial);
      line.name = `collision-outline-${link}`;
      line.renderOrder = 46;
      mesh.add(line);
      collisionOutlineObjects.push(line);
      added = true;
    });
    if (added) {
      outlined.push(link);
    }
  }
  window.__cadquerySeamRuntime!.collisionOutlinedLinks = outlined;
}

function collisionLinkNames(contacts: CollisionContact[] | null): string[] {
  if (!contacts) {
    return [];
  }
  const names = new Set<string>();
  for (const contact of contacts) {
    names.add(contact.link_a);
    names.add(contact.link_b);
  }
  return [...names].sort();
}

function applyCollisionFeedback(contacts: CollisionContact[] | null): void {
  const runtime = window.__cadquerySeamRuntime!;
  const links = collisionLinkNames(contacts);
  const key = links.join("|");
  runtime.collisionActive = links.length > 0;
  runtime.collisionLinks = links;
  if (key === activeCollisionLinkKey) {
    return;
  }
  activeCollisionLinkKey = key;

  // Restore previously tinted meshes before tinting the new contact set.
  for (const [mesh, material] of collisionTintOriginalMaterials) {
    mesh.material = material;
  }
  collisionTintOriginalMaterials.clear();

  if (links.length === 0) {
    collisionStatusEl.textContent = "collision: clear";
    return;
  }
  const deepest = contacts!.reduce((max, contact) => Math.max(max, contact.depth_mm), 0);
  collisionStatusEl.textContent = `COLLISION ${contacts!.map((c) => `${c.link_a}|${c.link_b}`).join(" ")} depth=${deepest.toFixed(1)}mm`;
  if (!loadedRobot) {
    return;
  }
  for (const link of links) {
    const linkObject = loadedRobot.getObjectByName(link);
    if (!linkObject) {
      continue;
    }
    linkObject.traverse((child) => {
      const mesh = child as THREE.Mesh;
      if (mesh.isMesh && !collisionTintOriginalMaterials.has(mesh)) {
        collisionTintOriginalMaterials.set(mesh, mesh.material);
        mesh.material = collisionTintMaterial;
      }
    });
  }
}

function applyRobotJointPositions(jointPositionsRad: number[] | null): boolean {
  const runtime = window.__cadquerySeamRuntime!;
  if (!loadedRobot?.setJointValue || !jointPositionsRad || jointPositionsRad.length !== ROBOT_ARM_JOINT_NAMES.length) {
    runtime.robotJointsAnimating = false;
    return false;
  }
  ROBOT_ARM_JOINT_NAMES.forEach((jointName, index) => {
    loadedRobot!.setJointValue!(jointName, jointPositionsRad[index]);
  });
  runtime.robotJointAnglesRad = [...jointPositionsRad];
  if (!runtime.robotJointsAnimating) {
    runtime.robotJointsAnimating = true;
    robotStatusEl.textContent = "rosie_1400_v3 animating backend joint trajectory (J1-J6)";
  }
  return true;
}

function setupWeldAnimation(points: Point3[]): void {
  activeWeldPath = points.map((point) => ({ ...point }));
  weldAnimationStartedMS = performance.now();
  if (weldAnimationGroup) {
    scene.remove(weldAnimationGroup);
    disposeObject(weldAnimationGroup);
  }
  weldAnimationGroup = new THREE.Group();
  weldAnimationGroup.name = "weld-arrow-trace-animation";
  weldTraceLine = createPolyline([], 0xf2f2f2, 1);
  weldTraceLine.name = "weld-simulation-trace";
  weldTraceLine.renderOrder = 36;
  weldAnimationGroup.add(weldTraceLine);
  weldTorch = createWeldTorch();
  weldAnimationGroup.add(weldTorch);
  const initialGizmoPoint = activeWeldPath[0] ?? { x: 0, y: 0, z: 0 };
  weldTorchLiveGizmo = createToolFrameGizmo(
    initialGizmoPoint,
    activeWeldToolFrames[0] ?? { x: { x: 1, y: 0, z: 0 }, y: { x: 0, y: 1, z: 0 }, z: { x: 0, y: 0, z: 1 } },
    54,
    0x38bdf8,
  );
  weldTorchLiveGizmo.name = "animated-torch-live-tcp-gizmo";
  weldAnimationGroup.add(weldTorchLiveGizmo);
  scene.add(weldAnimationGroup);
  weldStatusEl.textContent = "animating 45deg weld torch";
}

function createWeldTorch(): THREE.Group {
  const group = new THREE.Group();
  group.name = "weld-45deg-torch";
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(2.2, 2.2, 58, 14),
    new THREE.MeshBasicMaterial({ color: 0xf7f0e4, depthTest: false, depthWrite: false }),
  );
  shaft.position.y = -48;
  shaft.renderOrder = 40;
  group.add(shaft);
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(8.5, 24, 20),
    new THREE.MeshBasicMaterial({ color: 0xcfcfcf, depthTest: false, depthWrite: false }),
  );
  cone.position.y = -12;
  cone.rotation.x = Math.PI;
  cone.renderOrder = 41;
  group.add(cone);
  const arc = new THREE.Mesh(
    new THREE.SphereGeometry(5.2, 18, 12),
    new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.85, depthTest: false, depthWrite: false }),
  );
  arc.position.y = 0;
  arc.renderOrder = 42;
  group.add(arc);
  return group;
}

function updateWeldAnimation(nowMS: number): void {
  if (!weldAnimationGroup || !weldTorch || !weldTraceLine || !weldTorchLiveGizmo || activeWeldPath.length < 2) {
    return;
  }

  // Preferred playback: play the backend Tesseract motion plan through time so
  // the torch follows approach -> weld -> retract with slerped TCP orientation
  // instead of a frontend seam-only loop.
  if (activeMotionSamples.length >= 2 && activeMotionDurationS > 0) {
    const playbackDurationMS = Math.max(4000, activeMotionDurationS * 220);
    const progress = ((nowMS - weldAnimationStartedMS) % playbackDurationMS) / playbackDurationMS;
    window.__cadquerySeamRuntime!.weldAnimationProgress = progress;
    const motion = sampleMotionPlan(progress * activeMotionDurationS);
    if (motion) {
      weldTorch.position.copy(motion.position);
      window.__cadquerySeamRuntime!.weldTorchDistanceToPathMm = minDistanceToPath(motion.position, activeWeldPath);
      const torchAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(motion.quaternion);
      weldTorch.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), torchAxis);
      updateToolFrameGizmo(weldTorchLiveGizmo, motion.position, toolFrameFromQuaternion(motion.quaternion), 54);
      const trace = motionTraceUpTo(progress);
      weldTraceLine.geometry.dispose();
      weldTraceLine.geometry = new THREE.BufferGeometry().setFromPoints(trace);
      // Drive the URDF robot arm through the backend joint trajectory so the
      // whole cell plays the Tesseract plan, not just the floating torch.
      applyRobotJointPositions(motion.jointPositionsRad);
      // Robot/v4 pattern: contact links tint red while the plan passes
      // through a colliding waypoint.
      applyCollisionFeedback(motion.collisionContacts);
    }
    return;
  }

  const durationMS = Math.max(2400, activeWeldPath.length * 80);
  const progress = ((nowMS - weldAnimationStartedMS) % durationMS) / durationMS;
  window.__cadquerySeamRuntime!.weldAnimationProgress = progress;
  const sample = sampleScenePath(activeWeldPath, progress);
  weldTorch.position.copy(sample.position);
  window.__cadquerySeamRuntime!.weldTorchDistanceToPathMm = minDistanceToPath(weldTorch.position, activeWeldPath);
  const axisIndex = Math.max(0, Math.min(activeWeldToolZAxes.length - 1, Math.round(progress * Math.max(0, activeWeldToolZAxes.length - 1))));
  const backendFrame = activeWeldToolFrames[axisIndex];
  const backendAxis = backendFrame?.z ?? activeWeldToolZAxes[axisIndex];
  const torchAxis = backendAxis
    ? new THREE.Vector3(backendAxis.x, backendAxis.y, backendAxis.z).normalize()
    : sample.tangent.clone().multiplyScalar(0.70710678).add(new THREE.Vector3(0, 0, 0.70710678)).normalize();
  weldTorch.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), torchAxis);
  updateToolFrameGizmo(
    weldTorchLiveGizmo,
    sample.position,
    backendFrame ?? toolFrameFromTangentAndZ(sample.tangent, torchAxis),
    54,
  );

  const tracePoints = sampledTrace(activeWeldPath, progress, 64);
  weldTraceLine.geometry.dispose();
  weldTraceLine.geometry = new THREE.BufferGeometry().setFromPoints(tracePoints);
}

function motionTraceUpTo(progress: number): THREE.Vector3[] {
  const cutoff = Math.max(0, Math.min(1, progress)) * activeMotionDurationS;
  const out: THREE.Vector3[] = [];
  for (const sample of activeMotionSamples) {
    if (sample.timeS > cutoff) {
      break;
    }
    out.push(sample.position.clone());
  }
  if (out.length < 2) {
    return activeMotionSamples.slice(0, 2).map((sample) => sample.position.clone());
  }
  return out;
}

function minDistanceToPath(point: THREE.Vector3, path: Point3[]): number {
  if (path.length === 0) {
    return Number.POSITIVE_INFINITY;
  }
  return Math.min(
    ...path.map((pathPoint) => point.distanceTo(new THREE.Vector3(pathPoint.x, pathPoint.y, pathPoint.z))),
  );
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
    const startDistance = lengths[index - 1];
    const segmentLength = lengths[index] - startDistance || 1;
    const t = (target - startDistance) / segmentLength;
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

function renderEdgeList(edges: TopologyEdge[]): void {
  const rows = rankedSeamEdges(edges, 120)
    .map((edge) => {
      const row = document.createElement("button");
      row.type = "button";
      row.className = "edge-row";
      row.dataset.edgeId = edge.id;
      row.dataset.testid = "edge-row";
      row.addEventListener("click", () => void selectEdge(edge.id));

      const title = document.createElement("strong");
      title.textContent = compactEdgeTitle(edge);
      const meta = document.createElement("span");
      meta.textContent = compactEdgeMeta(edge);
      const part = document.createElement("em");
      part.textContent = edge.part_id ?? "part";
      row.append(title, meta, part);
      return row;
    });
  edgeListEl.replaceChildren(...rows);
}

async function selectEdge(edgeId: string): Promise<void> {
  const edge = edgeById.get(edgeId);
  if (!edge) {
    return;
  }
  selectedEdgeId = edgeId;
  window.__cadquerySeamRuntime!.selectedEdgeId = edgeId;
  startRange.disabled = false;
  endRange.disabled = false;
  reverseButton.disabled = false;
  addSeamButton.disabled = false;
  copyButton.disabled = false;
  downloadButton.disabled = false;
  selectedEdgeEl.textContent = edge.id;
  selectedEdgeEl.title = edge.id;
  edgeTypeEl.textContent = edge.geom_type ?? "EDGE";
  edgeLengthEl.textContent = formatLength(edge.length_mm);
  edgePartEl.textContent = edge.part_id ?? "-";
  edgeListEl.querySelectorAll<HTMLButtonElement>(".edge-row").forEach((row) => {
    row.classList.toggle("is-selected", row.dataset.edgeId === edgeId);
  });
  refreshEdgeVisuals();
  await renderSeamSegment();
  addTourLog(`Selected ${edge.id} (${edge.geom_type ?? "EDGE"}, ${formatLength(edge.length_mm)}).`);
  maybeAdvanceTour(3);
}

function refreshEdgeVisuals(): void {
  edgeLines.forEach((line, edgeId) => {
    const material = line.material as THREE.LineBasicMaterial;
    if (edgeId === selectedEdgeId) {
      material.color.setHex(EDGE_SELECTED);
      material.opacity = 1;
      material.transparent = false;
      line.renderOrder = 20;
    } else if (edgeId === hoveredEdgeId) {
      material.color.setHex(EDGE_HOVER);
      material.opacity = 1;
      material.transparent = false;
      line.renderOrder = 18;
    } else {
      material.color.setHex(EDGE_DEFAULT);
      material.opacity = 0.62;
      material.transparent = true;
      line.renderOrder = 10;
    }
  });
}

async function renderSeamSegment(): Promise<void> {
  if (seamGroup) {
    scene.remove(seamGroup);
    disposeObject(seamGroup);
    seamGroup = null;
  }
  const edge = selectedEdgeId ? edgeById.get(selectedEdgeId) : null;
  if (!edge) {
    return;
  }
  normalizeRanges();
  const startS = Number(startRange.value) / 100;
  const endS = Number(endRange.value) / 100;
  const requestSerial = ++seamPreviewRequestSerial;
  const backendPayload = await requestBackendSeam(edge.id, startS, endS, seamDirectionReversed);
  if (requestSerial !== seamPreviewRequestSerial) {
    return;
  }
  currentBackendSeam = backendPayload;
  const points = (backendPayload.seam.world_points_mm ?? backendPayload.seam.points).map(worldMmTupleToScenePoint);
  currentSeamPoints = points;
  pushReport(`tcp plan ${edge.id}`, [
    `poses=${backendPayload.tesseract_plan?.tcp_poses?.length ?? points.length} path_method=${backendPayload.tesseract_plan?.path_method?.selected ?? "sampled_edge"}`,
  ]);
  activeWeldToolFrames = (backendPayload.tesseract_plan?.tcp_poses ?? [])
    .map(toolFrameFromPose)
    .filter((frame): frame is ToolFrame => frame !== null);
  activeWeldToolZAxes = (backendPayload.tesseract_plan?.tcp_poses ?? [])
    .map((pose) => (Array.isArray(pose.tool_z_axis) ? toPoint(pose.tool_z_axis) : null))
    .filter((axis): axis is Point3 => axis !== null);
  const motionPlan = backendPayload.tesseract_plan?.motion_plan;
  if (motionPlan) {
    renderMotionPlan(motionPlan);
    const reportLines = [
      `samples=${motionPlan.sample_count} duration=${motionPlan.duration_s.toFixed(1)}s weld=${motionPlan.weld_sample_count} travel=${motionPlan.sample_count - motionPlan.weld_sample_count} frame_rot_max=${motionPlan.max_frame_rotation_deg.toFixed(1)}deg`,
    ];
    if (motionPlan.ik) {
      reportLines.push(
        `ik=${motionPlan.ik.solver} joints=${motionPlan.ik.joint_names.join(",")} pos_err_max=${motionPlan.ik.max_position_error_mm.toFixed(3)}mm rot_err_max=${motionPlan.ik.max_rotation_error_deg.toFixed(3)}deg`,
      );
    }
    pushReport("tesseract motion", reportLines);
    const collision = motionPlan.collision;
    const contactSamples = motionPlan.trajectory.filter((sample) => (sample.collision_contacts?.length ?? 0) > 0).length;
    window.__cadquerySeamRuntime!.collisionSampleCount = contactSamples;
    // Outline BOTH sides of every colliding pair for the whole tour.
    const outlineLinks = new Set<string>();
    for (const sample of motionPlan.trajectory) {
      for (const contact of sample.collision_contacts ?? []) {
        outlineLinks.add(contact.link_a);
        outlineLinks.add(contact.link_b);
      }
    }
    pendingCollisionOutlineLinks = [...outlineLinks].sort();
    installCollisionOutlines();
    if (collision?.status === "checked") {
      collisionStatusEl.textContent =
        contactSamples > 0
          ? `collision: ${contactSamples}/${motionPlan.sample_count} contact waypoints (report-only)`
          : "collision: clear";
      pushReport("tesseract collision", [
        `checker=${collision.checker} waypoints=${collision.checked_waypoint_count} contacts=${collision.contact_count} contact_samples=${collision.contact_sample_count} max_depth=${(collision.max_depth_mm ?? 0).toFixed(1)}mm`,
        ...(collision.contact_pairs && collision.contact_pairs.length > 0
          ? [`pairs=${collision.contact_pairs.join(" ")} (report-only, robot/v4 pattern)`]
          : []),
      ]);
    } else {
      collisionStatusEl.textContent = `collision: ${collision?.status ?? "not checked"}${collision?.reason ? ` (${collision.reason})` : ""}`;
      pushReport("tesseract collision", [collisionStatusEl.textContent]);
    }
  } else {
    activeMotionSamples = [];
    activeMotionDurationS = 0;
  }
  setupWeldAnimation(points);
  if (backendPayload.tesseract_plan) {
    renderBackendTesseractPlan(backendPayload.tesseract_plan);
  }
  const group = new THREE.Group();
  group.name = "weld-seam-candidate";
  const line = createPolyline(points, SEAM_ORANGE, 1);
  line.renderOrder = 28;
  group.add(line);
  const tube = createTubeScene(points, SEAM_RADIUS_MM, SEAM_ORANGE, 0.95);
  if (tube) {
    tube.renderOrder = 27;
    group.add(tube);
  }
  const startMarker = createPointMarker(points[0], 0xffffff, 4.8, true);
  startMarker.name = "seam-start-marker";
  group.add(startMarker);
  const endMarker = createPointMarker(points[points.length - 1], SEAM_ORANGE, 4.8, true);
  endMarker.name = "seam-end-marker";
  group.add(endMarker);
  const arrow = createDirectionArrow(points, true);
  if (arrow) {
    group.add(arrow);
  }
  scene.add(group);
  seamGroup = group;

  seamPointCountEl.textContent = `${points.length} points`;
  seamJSONEl.value = JSON.stringify(backendPayload, null, 2);
}

async function requestBackendSeam(
  edgeId: string,
  startS: number,
  endS: number,
  reversed: boolean,
): Promise<BackendSeamPayload> {
  if (!activeStepRequest) {
    throw new Error("No active STEP request is available for backend seam computation.");
  }
  const response = await fetch("/api/cadquery/topology", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      ...activeStepRequest,
      operation: "build_seam",
      edge_id: edgeId,
      start_s: startS,
      end_s: endS,
      reversed,
      seam_sample_count: WELD_TCP_SAMPLE_COUNT,
    }),
  });
  const data = (await response.json()) as BackendSeamPayload | { error?: string };
  if (!response.ok) {
    throw new Error("error" in data && data.error ? data.error : `HTTP ${response.status}`);
  }
  return data as BackendSeamPayload;
}

async function requestBackendSequence(): Promise<BackendSequencePayload | null> {
  if (!activeStepRequest || savedSeams.length === 0) {
    return null;
  }
  const response = await fetch("/api/cadquery/topology", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      ...activeStepRequest,
      operation: "build_sequence",
      seam_sample_count: WELD_TCP_SAMPLE_COUNT,
      segments: savedSeams.map((seam) => ({
        id: seam.id,
        edge_id: seam.edgeId,
        start_s: seam.startS,
        end_s: seam.endS,
        reversed: seam.reversed,
      })),
    }),
  });
  const data = (await response.json()) as BackendSequencePayload | { error?: string };
  if (!response.ok) {
    throw new Error("error" in data && data.error ? data.error : `HTTP ${response.status}`);
  }
  return data as BackendSequencePayload;
}

async function refreshBackendSequence(): Promise<void> {
  try {
    const sequence = await requestBackendSequence();
    if (!sequence) {
      return;
    }
    currentBackendSequence = sequence;
    const seamCount = sequence.weld_sequence?.seams?.length ?? 0;
    if (sequence.tesseract_plan) {
      renderBackendTesseractPlan(sequence.tesseract_plan, seamCount);
    }
    pathStatusEl.textContent = `${sequence.tesseract_plan?.tcp_poses?.length ?? 0} Tesseract TCP poses / ${seamCount} seams / +X tangent / 45deg plane`;
  } catch (error) {
    pathStatusEl.textContent = `sequence unavailable: ${error instanceof Error ? error.message : String(error)}`;
  }
}

function normalizeRanges(): void {
  let startValue = Number(startRange.value);
  let endValue = Number(endRange.value);
  if (startValue === endValue) {
    endValue = Math.min(100, startValue + 1);
    if (endValue === startValue) {
      startValue = Math.max(0, endValue - 1);
    }
  }
  if (startValue > endValue) {
    [startValue, endValue] = [endValue, startValue];
  }
  startRange.value = String(startValue);
  endRange.value = String(endValue);
  startOutput.textContent = `${startValue}%`;
  endOutput.textContent = `${endValue}%`;
}

async function reverseSeamDirection(): Promise<void> {
  if (!selectedEdgeId) {
    return;
  }
  seamDirectionReversed = !seamDirectionReversed;
  reverseButton.textContent = seamDirectionReversed ? "Forward Direction" : "Reverse Direction";
  await renderSeamSegment();
  addTourLog(seamDirectionReversed ? "Reversed seam travel direction." : "Restored forward seam travel direction.");
  status(seamDirectionReversed ? "Seam direction reversed." : "Seam direction restored.");
}

async function addCurrentSeam(): Promise<void> {
  await saveCurrentSeam({ refreshSequence: true, updateStatus: true });
}

async function saveCurrentSeam(options: { refreshSequence: boolean; updateStatus: boolean }): Promise<void> {
  const edge = selectedEdgeId ? edgeById.get(selectedEdgeId) : null;
  if (!edge || !currentBackendSeam || currentSeamPoints.length < 2) {
    status("Pick an edge and trim a non-empty seam before adding it.");
    return;
  }
  const backendSeam = currentBackendSeam.seam;
  const existingSeam = savedSeams.find((item) => item.edgeId === edge.id);
  const seam: SavedSeam = {
    id: existingSeam?.id ?? `seam_${String(savedSeams.length + 1).padStart(2, "0")}`,
    edgeId: backendSeam.edge_id,
    partId: backendSeam.part_id,
    geomType: backendSeam.geom_type,
    startS: backendSeam.start_s,
    endS: backendSeam.end_s,
    reversed: backendSeam.reversed,
    lengthMm: backendSeam.length_mm,
    points: currentSeamPoints.map((point) => ({ ...point })),
  };
  savedSeams = [...savedSeams.filter((item) => item.edgeId !== edge.id), seam];
  activeSavedSeamId = seam.id;
  window.__cadquerySeamRuntime!.savedSeams = savedSeams;
  renderSavedSeams();
  if (options.refreshSequence) {
    await refreshBackendSequence();
  }
  maybeAdvanceTour(5);
  addTourLog(`Captured ${seam.id} from ${edge.id} with ${seam.points.length} points.`);
  pushReport(`seam capture ${seam.edgeId}`, [
    `id=${seam.id} range=${Math.round(seam.startS * 100)}-${Math.round(seam.endS * 100)} points=${seam.points.length}`,
  ]);
  if (options.updateStatus) {
    status(`Tour complete: Dog Box weld sequence captured through ${edge.id}.`);
  }
}

function renderSavedSeams(): void {
  seamCountEl.textContent = `${savedSeams.length} seam${savedSeams.length === 1 ? "" : "s"}`;
  if (savedSeams.length === 0) {
    savedSeamsEl.replaceChildren(emptyState("No seams captured yet. Add the current orange segment when it looks right."));
    return;
  }
  savedSeamsEl.replaceChildren(
    ...savedSeams.map((seam) => {
      const row = document.createElement("button");
      row.type = "button";
      row.className = "saved-seam-row";
      row.classList.toggle("is-selected", seam.id === activeSavedSeamId);
      row.dataset.edgeId = seam.edgeId;
      row.addEventListener("click", () => {
        activeSavedSeamId = seam.id;
        renderSavedSeams();
        void selectEdge(seam.edgeId);
      });
      row.innerHTML = `
        <strong>${seam.id}</strong>
        <span>${seam.edgeId}</span>
        <em>${Math.round(seam.startS * 100)}-${Math.round(seam.endS * 100)}%${seam.reversed ? " / reversed" : ""}</em>
      `;
      return row;
    }),
  );
}

function emptyState(text: string): HTMLElement {
  const div = document.createElement("div");
  div.className = "empty-state";
  div.textContent = text;
  return div;
}

async function runInlineTour(loopAfter = new URLSearchParams(window.location.search).get("loop") === "1"): Promise<void> {
  if (quickTourButton.disabled) {
    return;
  }
  quickTourButton.disabled = true;
  inlineTourRunning = true;
  try {
    const steps: Array<{ selector: string; caption: string; camera: TourCameraPose; before?: () => Promise<void> | void }> = [
      {
        selector: ".seam-topbar",
        caption: "Weld Sequence is the main seam authoring overlay. Everything here stays over the live 3D CAD stage.",
        camera: TOUR_CAMERA_POSES.intro,
      },
      {
        selector: ".tool-switcher",
        caption: "Use these navigation buttons to switch between the weld sequence page and the focused edge geometry sandbox.",
        camera: TOUR_CAMERA_POSES.intro,
      },
      {
        selector: ".title-stats",
        caption: "These stats summarize the currently loaded model without taking space in the control stack.",
        camera: TOUR_CAMERA_POSES.intro,
      },
      {
        selector: ".edge-browser",
        caption: "Loading Dog Box now. The edge list will show real seam candidates only.",
        camera: TOUR_CAMERA_POSES.load,
        before: async () => {
          if (!activeModel) {
            await loadDogBoxExample();
          }
        },
      },
      {
        selector: ".edge-browser",
        caption: "Topology Edges lists closed circles and 4-arc + 4-line dog-bone wires from the CAD backend.",
        camera: TOUR_CAMERA_POSES.topology,
      },
      {
        selector: "[data-testid='edge-row']",
        caption: "This dog-bone wire is the example seam. Selecting it asks the backend for a sampled CAD path and TCP poses.",
        camera: TOUR_CAMERA_POSES.seam,
        before: async () => {
          await selectExampleTourSeam();
        },
      },
      {
        selector: ".panel-section:nth-of-type(2)",
        caption: "Selected Edge shows the active CAD source path and its length/type/part.",
        camera: TOUR_CAMERA_POSES.seam,
      },
      {
        selector: ".panel-section:nth-of-type(3)",
        caption: "Seam Window trims the selected path by arclength. The orange path and weld torch update from backend geometry.",
        camera: TOUR_CAMERA_POSES.seam,
      },
      {
        selector: ".panel-section:nth-of-type(4)",
        caption: "Seam Set stores captured seams. Capturing the example seam now so it remains in the list at the end.",
        camera: TOUR_CAMERA_POSES.captured,
        before: async () => {
          await ensureExampleTourSeamCaptured();
        },
      },
      {
        selector: ".stage",
        caption: `The tour ends here: ${TOUR_FINAL_EDGE_ID} is selected, captured, and animated with backend +X tangent and 45 degree tool angle.`,
        camera: TOUR_CAMERA_POSES.final,
      },
    ];

    tourCaptionEl.hidden = false;
    for (const step of steps) {
      await step.before?.();
      const target = document.querySelector<HTMLElement>(step.selector);
      if (!target) {
        continue;
      }
      await showTourHighlight(target, step.caption, step.camera);
    }
    status("Tour complete: example seam selected, captured, and animating.");
  } catch (error) {
    pushReport("tour error", [error instanceof Error ? error.message : String(error)]);
  } finally {
    inlineTourRunning = false;
    clearTourHighlights();
    window.setTimeout(() => {
      tourCaptionEl.hidden = true;
    }, 1800);
    quickTourButton.disabled = false;
    if (loopAfter) {
      // Always hand off to Edge Geometry so the multi-app tour loops forever,
      // even when a tour step fails.
      const nextURL = new URL(`${import.meta.env.BASE_URL}edge-geometry`, window.location.origin);
      nextURL.searchParams.set("loop", "1");
      window.setTimeout(() => {
        window.location.href = nextURL.toString();
      }, 2200);
    }
  }
}

async function selectExampleTourSeam(): Promise<void> {
  if (!activeModel) {
    await loadDogBoxExample();
  }
  const edge =
    edgeById.get(TOUR_FINAL_EDGE_ID) ??
    [...edgeById.values()].find((item) => item.path_class === "dogbone_4arc_4line") ??
    [...edgeById.values()][0];
  if (!edge) {
    return;
  }
  startRange.value = "0";
  endRange.value = "100";
  seamDirectionReversed = false;
  reverseButton.textContent = "Reverse Direction";
  await selectEdge(edge.id);
}

async function ensureExampleTourSeamCaptured(): Promise<void> {
  if (selectedEdgeId !== TOUR_FINAL_EDGE_ID) {
    await selectExampleTourSeam();
  }
  const existing = savedSeams.find((item) => item.edgeId === TOUR_FINAL_EDGE_ID);
  if (existing) {
    activeSavedSeamId = existing.id;
    renderSavedSeams();
    return;
  }
  await saveCurrentSeam({ refreshSequence: false, updateStatus: false });
}

function applyTourFinalCameraView(): void {
  controls.target.copy(TOUR_FINAL_CONTROLS_TARGET);
  camera.position.copy(TOUR_FINAL_CAMERA_POSITION);
  camera.near = 0.1;
  camera.far = 20000;
  camera.zoom = 1;
  camera.updateProjectionMatrix();
  controls.update();
}

async function showTourHighlight(target: HTMLElement, caption: string, cameraPose: TourCameraPose): Promise<void> {
  void target;
  tourCaptionEl.textContent = caption;
  pushReport("subtitle", [caption]);
  await Promise.all([animateCameraTo(cameraPose), delay(1550)]);
}

function animateCameraTo(pose: TourCameraPose): Promise<void> {
  const serial = ++cameraTourAnimationSerial;
  return animateCameraElliptical({
    camera,
    controls,
    pose,
    isCancelled: () => serial !== cameraTourAnimationSerial,
  });
}

function clearTourHighlights(): void {
  // 2D tour highlights are intentionally disabled; the log and camera carry the tour.
}

function frameTopology(): void {
  if (!modelGroup) {
    return;
  }
  modelGroup.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(modelGroup);
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z, 50);
  controls.target.copy(center);
  camera.near = Math.max(maxDim / 1000, 0.1);
  camera.far = Math.max(maxDim * 24, 4000);
  camera.position.copy(center).add(new THREE.Vector3(maxDim * 1.2, -maxDim * 1.65, maxDim * 1.05));
  camera.updateProjectionMatrix();
  controls.update();
}

function updatePointer(event: PointerEvent): void {
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
}

function pickEdge(): string | null {
  if (edgePickMeshes.size === 0) {
    return null;
  }
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObjects([...edgePickMeshes.values()], false)[0];
  return hit ? String(hit.object.userData.edgeId ?? "") || null : null;
}

function handlePointerMove(event: PointerEvent): void {
  updatePointer(event);
  const nextHover = pickEdge();
  if (nextHover !== hoveredEdgeId) {
    hoveredEdgeId = nextHover;
    refreshEdgeVisuals();
  }
}

function handleClick(event: PointerEvent): void {
  updatePointer(event);
  const edgeId = pickEdge();
  if (edgeId) {
    void selectEdge(edgeId);
  }
}

async function copySeamJSON(): Promise<void> {
  if (!seamJSONEl.value) {
    return;
  }
  await navigator.clipboard.writeText(seamJSONEl.value);
  status("Copied seam JSON to clipboard.");
}

function downloadSeamJSON(): void {
  if (!seamJSONEl.value) {
    return;
  }
  const edgeSafe = (selectedEdgeId ?? "edge").replace(/[^A-Za-z0-9_-]+/g, "_");
  const blob = new Blob([seamJSONEl.value, "\n"], { type: "application/json" });
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `cadquery-weld-seam-${edgeSafe}.json`;
  link.click();
  URL.revokeObjectURL(link.href);
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

stepFileInput.addEventListener("change", () => {
  const file = stepFileInput.files?.[0] ?? null;
  fileNameEl.textContent = file ? file.name : "Choose .step or .stp";
  if (file) {
    status(`${file.name} ready. Extract Geometry edges to begin seam selection.`);
  }
});
quickTourButton.addEventListener("click", () => void runInlineTour(true));
dogBoxButton.addEventListener("click", () => void loadDogBoxExample());
extractButton.addEventListener("click", () => void extractTopology());
frameButton.addEventListener("click", frameTopology);
startRange.addEventListener("input", () => {
  void renderSeamSegment().then(() => maybeAdvanceTour(4));
  addTourLog(`Adjusted seam window to ${startRange.value}-${endRange.value}%.`);
});
endRange.addEventListener("input", () => {
  void renderSeamSegment().then(() => maybeAdvanceTour(4));
  addTourLog(`Adjusted seam window to ${startRange.value}-${endRange.value}%.`);
});
reverseButton.addEventListener("click", () => void reverseSeamDirection());
addSeamButton.addEventListener("click", () => void addCurrentSeam());
copyButton.addEventListener("click", () => void copySeamJSON());
downloadButton.addEventListener("click", downloadSeamJSON);
renderer.domElement.addEventListener("pointermove", handlePointerMove);
renderer.domElement.addEventListener("click", handleClick);
renderer.domElement.addEventListener("pointerleave", () => {
  hoveredEdgeId = null;
  refreshEdgeVisuals();
});

window.addEventListener("resize", resize);
new ResizeObserver(resize).observe(stageEl);
resize();
tick();

setTourStep(0);
if (new URLSearchParams(window.location.search).get("loop") === "1") {
  window.setTimeout(() => {
    void runInlineTour(true);
  }, 400);
}
