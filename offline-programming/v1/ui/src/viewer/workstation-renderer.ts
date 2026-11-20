import * as THREE from "three";
import { themeToken } from "../ui/theme.ts";
import { WorkstationOrbitControls } from "./orbit-controls.ts";
import { tracePlannedTcpPath } from "./planned-path.ts";
import { PartGizmo } from "./part-gizmo.ts";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import type { WorkcellFixture } from "../domain/workcell.ts";
import { createWorkcellFixtureGroup } from "./workcellFixtures.ts";
import type {
  RobotPoseSnapshot,
  SimulationSample,
  TcpPose,
} from "../simulation/contracts.ts";
import {
  PoseLatch,
  type IdentityValidation,
  type ResolvedPoseMode,
  validateLoadedUrdfIdentity,
} from "./identity.ts";
import { boundsOfPoints, type MotionPathPoint } from "./geometry.ts";
import {
  CANONICAL_ROBOT_MODELS,
  robotModelFor,
  type RobotModelDescriptor,
} from "./models.ts";
import {
  applyTcpPose,
  createCollisionMarkerGroup,
  createCurveCenterGroup,
  createMotionPathGroup,
  createPlannedPathGroup,
  createPathDecorationGroup,
  createSeamOverlayGroup,
  createSelectionOverlayGroup,
  createTcpFrameGroup,
  type CollisionMarker,
  type CurveCenterMarker,
  type OverlayPalette,
  type PathLabel,
  type SeamOverlay,
  type SelectionOverlayInput,
  type TcpFrameMarker,
} from "./overlays.ts";
import { disposeObjectTree } from "./resource-disposal.ts";
import { loadUrdfRobot } from "./urdf.ts";
import {
  applyWorkpiecePlacement,
  createWorkpieceMeshGroup,
  workpieceMeshIdentity,
  type WorkpieceMeshOptions,
  type WorkpieceMeshPlacement,
} from "./workpiece.ts";
import {
  validateScissorPaneLayout,
  rendererSize,
  viewportFromRects,
  type PaneLayoutValidation,
  type RectLike,
} from "./viewport.ts";
import { WorldAxisIndicator } from "./world-axis.ts";

export interface WorkstationElements {
  stage: HTMLElement;
  canvas: HTMLCanvasElement;
  orbitPane: HTMLElement;
  worldAxisCanvas: HTMLCanvasElement;
  j6Pane: HTMLElement;
  tcpPane: HTMLElement;
}

export type WorkstationModelRegistry = Readonly<
  Record<string, RobotModelDescriptor<string>>
>;

/** How faint everything but the part is drawn under part focus. Enough to
 *  keep the robot and table as a reference, too faint to compete with a
 *  seam line. */
export const PART_FOCUS_OPACITY = 0.12;

function isUnder(node: THREE.Object3D, ancestor: THREE.Object3D): boolean {
  for (let cursor = node.parent; cursor; cursor = cursor.parent) {
    if (cursor === ancestor) return true;
  }
  return false;
}

export type WorkstationEvent =
  | { type: "model-loading"; modelId: string }
  | { type: "model-loaded"; modelId: string; assetErrors: readonly string[] }
  | { type: "model-error"; modelId: string; message: string }
  | { type: "pose"; modelId: string; mode: ResolvedPoseMode }
  | { type: "pose-rejected"; modelId: string; validation: IdentityValidation }
  | { type: "layout-warning"; validation: PaneLayoutValidation }
  /** The operator grabbed or released a part-gizmo handle. While true the
   *  orbit camera is frozen and edge picking must ignore the pointer. */
  | { type: "workpiece-drag"; dragging: boolean }
  /** A gizmo drag ended: the anchor's new world position and the part's
   *  world rotation (xyzw), for placementFromWorldAnchorPose. */
  | { type: "workpiece-placed"; anchorM: [number, number, number]; rotationXyzw: [number, number, number, number] }
  /** The operator grabbed or released the waypoint handle; as for the part. */
  | { type: "waypoint-drag"; dragging: boolean }
  /** The waypoint handle moved the object, once per pointer move while held:
   *  its pose so far, for a live preview. Nothing is committed. */
  | ({ type: "waypoint-moving" } & WaypointPose)
  /** A waypoint handle drag ended: the pose to keep. */
  | ({ type: "waypoint-moved" } & WaypointPose);

/** A dragged waypoint object and its pose in `waypointLayer`'s frame (the
 *  part's), quaternion xyzw. */
export interface WaypointPose {
  target: THREE.Object3D;
  positionM: [number, number, number];
  quaternionXyzw: [number, number, number, number];
}


export interface WorkstationRendererOptions {
  elements: WorkstationElements;
  models?: WorkstationModelRegistry;
  /** The robot to draw. The caller's, always: a default here would draw one
   *  robot over another one's program. */
  initialModelId: string;
  baseURL?: string;
  pixelRatioCap?: number;
  splitView?: boolean;
  autoStart?: boolean;
  onEvent?: (event: WorkstationEvent) => void;
}

export interface ModelLoadOutcome {
  ok: boolean;
  modelId: string;
  assetErrors: readonly string[];
  error?: string;
}

export interface WorkstationViewportDiagnostics {
  validation: PaneLayoutValidation;
  orbit: ReturnType<typeof viewportFromRects>;
  j6: ReturnType<typeof viewportFromRects>;
  tcp: ReturnType<typeof viewportFromRects>;
  pixelRatio: number;
}

/**
 * Framework-free Three.js workstation. A single WebGLRenderer owns orbit, J6,
 * and TCP views; right-side panes are rendered with CSS-pixel scissors.
 */
export class WorkstationRenderer {
  readonly elements: WorkstationElements;
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly orbitCamera = new THREE.PerspectiveCamera(46, 1, 0.01, 200);
  readonly j6Camera = new THREE.PerspectiveCamera(62, 1, 0.02, 40);
  readonly tcpCamera = new THREE.PerspectiveCamera(68, 1, 0.012, 40);
  readonly controls: WorkstationOrbitControls;
  readonly ready: Promise<ModelLoadOutcome>;

  private readonly models: WorkstationModelRegistry;
  private readonly baseURL: string;
  private readonly pixelRatioCap: number;
  private readonly onEvent?: (event: WorkstationEvent) => void;
  private descriptor: RobotModelDescriptor<string>;
  private poseLatch: PoseLatch<string>;
  private robot: URDFRobot | null = null;
  private splitView: boolean;
  private j6PaneVisible = true;
  private tcpPaneVisible = true;
  private resizeObserver: ResizeObserver | null = null;
  private animationFrame: number | null = null;
  private loadGeneration = 0;
  private disposed = false;
  private pathLayer = new THREE.Group();
  private plannedPathLayer = new THREE.Group();
  private executionPathLayer = new THREE.Group();
  private executionPathVisible = true;
  private plannedPathVisible = true;
  private plannedPathPoints: MotionPathPoint[] = [];
  private pathDecorationLayer = new THREE.Group();
  private selectionLayer = new THREE.Group();
  /** Rides the positioner's table link: the workpiece, its seams and the
   *  rails overlays are its children, so a turning positioner turns the part
   *  and everything drawn on it. Identity when the model has no work frame. */
  readonly workFrameLayer = new THREE.Group();
  private workFrameZeroInverse: THREE.Matrix4 | null = null;
  private workpieceLayer = new THREE.Group();
  /** Identity of the installed workpiece geometry; "" when the layer is empty. */
  private workpieceIdentity = "";
  /** The workcell fixtures (the stand), at the scene root: they are the
   *  cell's, in the world frame, and do not ride the work frame. */
  private fixtureLayer = new THREE.Group();
  private fixtureIdentity = "";
  /** The part gizmo's handle: sits at the placement anchor with the part's
   *  rotation, under the work-frame follower like the mesh, so a drag reads
   *  back in the datum frame every placement is measured in. */
  private readonly gizmoPivot = new THREE.Object3D();
  private gizmo: PartGizmo | null = null;
  private gizmoEnabled = false;
  private gizmoHasAnchor = false;
  /** Set while a handle is held: the mesh follows the pivot rigidly from
   *  these start matrices instead of waiting for the project round trip. */
  private gizmoDrag: { pivotInverse: THREE.Matrix4; mesh: THREE.Matrix4 } | null = null;
  /** Waypoint welds draw here: posed with the part (the same placement the
   *  mesh takes), so what is drawn in it is in the part's own frame. */
  readonly waypointLayer = new THREE.Group();
  /** The same triad as the part's, on whichever waypoint torch is selected.
   *  A second instance rather than a retargeted first: the part gizmo's
   *  drag moves the mesh, this one moves only its object. */
  private waypointGizmo: PartGizmo | null = null;
  private waypointGizmoTarget: THREE.Object3D | null = null;
  /** Surrounding materials to restore when part focus goes off. */
  private partFocus: {
    materials: Map<THREE.Material, { opacity: number; transparent: boolean; depthWrite: boolean }>;
    dim: number;
  } | null = null;
  private seamLayer = new THREE.Group();
  private collisionLayer = new THREE.Group();
  private tcpFrameLayer = new THREE.Group();
  private curveCenterLayer = new THREE.Group();
  private currentTcp = createCurrentTcpFrame();
  private tcpFrameEnabled = true;
  private tcpPoseKnown = false;
  private readonly worldAxisIndicator: WorldAxisIndicator;
  /** In-flight camera pan toward a focused selection, advanced in renderNow. */
  private focusTween: {
    fromTarget: THREE.Vector3;
    toTarget: THREE.Vector3;
    fromPosition: THREE.Vector3;
    toPosition: THREE.Vector3;
    startedAtMs: number;
    durationMs: number;
  } | null = null;
  private rendererWidth = 0;
  private rendererHeight = 0;
  private rendererPixelRatio = 0;

  private readonly scratchOrigin = new THREE.Vector3();
  private readonly scratchTarget = new THREE.Vector3();
  private readonly scratchPosition = new THREE.Vector3();
  private readonly scratchForward = new THREE.Vector3();
  private readonly scratchUp = new THREE.Vector3();
  private readonly scratchRight = new THREE.Vector3();
  private readonly scratchNegForward = new THREE.Vector3();
  private readonly scratchQuaternion = new THREE.Quaternion();
  private readonly scratchBasis = new THREE.Matrix4();

  constructor(options: WorkstationRendererOptions) {
    this.elements = options.elements;
    this.models = options.models ?? CANONICAL_ROBOT_MODELS;
    // The caller names the robot: it is the project's, and a default here
    // would draw one robot over another one's program.
    const initialModelId = options.initialModelId;
    // A model this file has no chrome for still draws: the server holds its
    // description, and refusing to show a robot that exists is worse than
    // showing it with a generic camera (`robotModelFor`).
    const descriptor = this.models[initialModelId] ?? robotModelFor(initialModelId);
    this.descriptor = descriptor;
    this.poseLatch = new PoseLatch(descriptor);
    this.baseURL = options.baseURL ?? document.baseURI;
    this.pixelRatioCap = Math.max(1, options.pixelRatioCap ?? 1.5);
    this.onEvent = options.onEvent;
    this.splitView = options.splitView ?? true;
    this.worldAxisIndicator = new WorldAxisIndicator(
      this.elements.worldAxisCanvas,
    );

    this.scene.up.set(...descriptor.worldUp);
    this.scene.background = new THREE.Color(0xcfd2d5);
    this.scene.fog = new THREE.Fog(0xcfd2d5, 8, 30);
    this.configureScene();

    // One context only: every pane uses this renderer through scissor rectangles.
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.elements.canvas,
      antialias: true,
      alpha: false,
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.25;
    this.renderer.shadowMap.enabled = true;
    // PCF (not soft) is much cheaper per fragment, and autoUpdate=false lets us
    // rebuild the shadow map exactly once per frame instead of once per pane
    // render() below — the split view was paying for it up to 3x/frame.
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.shadowMap.autoUpdate = false;
    this.renderer.autoClear = false;
    this.applyTheme();

    this.orbitCamera.up.set(...descriptor.worldUp);
    this.controls = new WorkstationOrbitControls(this.orbitCamera, this.elements.orbitPane);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    // The operator grabbing the camera always wins over an automated pan.
    this.controls.addEventListener("start", () => {
      this.focusTween = null;
    });
    this.applyCameraDescriptor(descriptor);
    this.resetOrbitCamera();
    this.setSplitView(this.splitView);

    if (typeof ResizeObserver === "function") {
      this.resizeObserver = new ResizeObserver(() => this.resize());
      this.resizeObserver.observe(this.elements.stage);
    }
    this.resize();
    this.ready = this.loadModel(initialModelId);
    if (options.autoStart ?? true) this.start();
  }

  get modelId(): string {
    return this.descriptor.id;
  }

  get poseMode(): ResolvedPoseMode {
    return this.poseLatch.resolve().mode;
  }

  currentPoseSnapshot(): RobotPoseSnapshot<string> {
    const pose = this.poseLatch.resolve().pose;
    return {
      ...pose,
      jointOrder: [...pose.jointOrder],
      jointsRad: [...pose.jointsRad],
    };
  }

  start(): void {
    this.assertActive();
    if (this.animationFrame !== null) return;
    this.animationFrame = requestAnimationFrame(this.animate);
  }

  stop(): void {
    if (this.animationFrame === null) return;
    cancelAnimationFrame(this.animationFrame);
    this.animationFrame = null;
  }

  async loadModel(modelId: string): Promise<ModelLoadOutcome> {
    this.assertActive();
    const descriptor = this.models[modelId] ?? robotModelFor(modelId);
    if (this.robot && this.descriptor.id === modelId) {
      return { ok: true, modelId, assetErrors: [] };
    }

    const generation = ++this.loadGeneration;
    this.emit({ type: "model-loading", modelId });
    try {
      const loaded = await loadUrdfRobot(descriptor, this.baseURL);
      if (this.disposed || generation !== this.loadGeneration) {
        disposeObjectTree(loaded.robot);
        return {
          ok: false,
          modelId,
          assetErrors: loaded.assetErrors,
          error: "model load superseded",
        };
      }
      const identity = validateLoadedUrdfIdentity(descriptor, loaded.robot);
      if (!identity.ok) {
        disposeObjectTree(loaded.robot);
        const error = identity.issues.map((issue) => issue.message).join("; ");
        this.emit({ type: "model-error", modelId, message: error });
        return { ok: false, modelId, assetErrors: loaded.assetErrors, error };
      }

      if (this.robot) {
        this.scene.remove(this.robot);
        disposeObjectTree(this.robot);
      }
      this.robot = loaded.robot;
      this.robot.name = `${modelId}-workstation`;
      this.scene.add(this.robot);
      // The work frame's pose at the zero pose is the datum every placement
      // in the project is measured against; the follower applies only the
      // positioner's departure from it.
      this.workFrameZeroInverse = null;
      const workLink = descriptor.workFrameLink
        ? loaded.robot.links[descriptor.workFrameLink] ?? null
        : null;
      if (workLink) {
        for (const jointName of descriptor.urdfJointNames) {
          loaded.robot.setJointValue(jointName, 0);
        }
        loaded.robot.updateMatrixWorld(true);
        this.workFrameZeroInverse = workLink.matrixWorld.clone().invert();
      }
      if (descriptor.id !== this.descriptor.id) {
        this.poseLatch = new PoseLatch(descriptor);
      }
      this.descriptor = descriptor;
      this.applyCameraDescriptor(descriptor);
      this.applyResolvedPose(this.poseLatch.resolve());
      this.emit({ type: "model-loaded", modelId, assetErrors: loaded.assetErrors });
      return { ok: true, modelId, assetErrors: loaded.assetErrors };
    } catch (error) {
      if (generation !== this.loadGeneration) {
        return { ok: false, modelId, assetErrors: [], error: "model load superseded" };
      }
      const message = error instanceof Error ? error.message : String(error);
      this.emit({ type: "model-error", modelId, message });
      return { ok: false, modelId, assetErrors: [], error: message };
    }
  }

  setPose(pose: RobotPoseSnapshot<string>, online = true): IdentityValidation {
    this.assertActive();
    const validation = this.poseLatch.accept(pose, online);
    if (!validation.ok) {
      this.emit({ type: "pose-rejected", modelId: pose.modelId, validation });
      return validation;
    }
    this.applyResolvedPose(this.poseLatch.resolve(pose.observedAtMs));
    return validation;
  }

  setOffline(): void {
    this.assertActive();
    this.poseLatch.markOffline();
    this.applyResolvedPose(this.poseLatch.resolve());
  }

  applySimulationSample(sample: SimulationSample<string>): IdentityValidation {
    const validation = this.setPose(sample, true);
    if (validation.ok && sample.tcpPose) this.setCurrentTcpPose(sample.tcpPose);
    return validation;
  }

  setPath(
    points: readonly MotionPathPoint[],
    palette?: OverlayPalette,
    opacity = 1,
  ): void {
    this.elements.stage.dataset.pathPointCount = String(points.length);
    this.elements.stage.dataset.pathPointsM = points
      .map((point) => point.positionM.join(","))
      .join(";");
    this.pathLayer = this.replaceLayer(
      this.pathLayer,
      createMotionPathGroup(points, palette, "workstation-path", opacity),
    );
  }

  /** The complete planned route stays in world space while the positioner moves. */
  setPlannedPath(segments: readonly (readonly MotionPathPoint[])[]): void {
    this.assertActive();
    this.plannedPathPoints = segments.flatMap(points => [...points]);
    this.plannedPathLayer = this.replaceLayer(this.plannedPathLayer, createPlannedPathGroup(segments));
    this.plannedPathLayer.visible = this.plannedPathVisible;
    this.elements.stage.dataset.plannedPathPointCount = String(this.plannedPathPoints.length);
  }

  setPlannedPathVisible(visible: boolean): void {
    this.plannedPathVisible = visible;
    this.plannedPathLayer.visible = visible;
    this.elements.stage.dataset.plannedPathVisible = String(visible);
  }

  setExecutionPaths(actual: MotionPathPoint[][], commanded: MotionPathPoint[][]): void {
    this.assertActive();
    const group = new THREE.Group();
    group.name = "recorded-tcp-paths";
    const reference = createPlannedPathGroup(commanded, 0x9938a3);
    reference.name = "recorded-commanded-tcp";
    const measured = createPlannedPathGroup(actual, 0xd66a00);
    measured.name = "encoder-derived-tcp";
    group.add(reference, measured);
    this.executionPathLayer = this.replaceLayer(this.executionPathLayer, group);
    this.executionPathLayer.visible = this.executionPathVisible;
  }

  setExecutionPathVisible(visible: boolean): void {
    this.executionPathVisible = visible;
    this.executionPathLayer.visible = visible;
  }

  async tracePlannedPath(
    segments: readonly (readonly SimulationSample[])[],
    isCurrent: () => boolean,
  ): Promise<MotionPathPoint[][]> {
    this.assertActive();
    if (!this.robot) throw new Error("Robot geometry is not loaded yet");
    return tracePlannedTcpPath(this.robot, this.descriptor.cameras.tcp.frame,
      this.descriptor.urdfJointNames, segments, { isCurrent });
  }

  /**
   * Weld start/stop markers and row-number labels for the
   * authored spreadsheet path.
   */
  setPathDecorations(
    points: readonly MotionPathPoint[],
    labels: readonly PathLabel[] = [],
    palette?: OverlayPalette,
    opacity = 1,
  ): void {
    this.pathDecorationLayer = this.replaceLayer(
      this.pathDecorationLayer,
      createPathDecorationGroup(points, labels, palette, opacity),
    );
  }

  /** Highlight for the selected spreadsheet row, or null to clear it. */
  setSelectionOverlay(
    input: SelectionOverlayInput | null,
    palette?: OverlayPalette,
  ): void {
    this.elements.stage.dataset.selectedTarget = input
      ? `${input.id}:${input.targetM.map((value) => value.toFixed(4)).join(",")}`
      : "";
    this.selectionLayer = this.replaceLayer(
      this.selectionLayer,
      createSelectionOverlayGroup(input, palette),
    );
  }

  /**
   * The imported part itself, placed by the fixture placement. Passing null
   * clears the layer, which is the correct state for the browser-fallback
   * example project and whenever the CAD service has no mesh for a fingerprint.
   *
   * The mesh is the largest GPU allocation in the app, so it goes through the
   * same replaceLayer path as every other layer to guarantee disposal.
   */
  setWorkpiece(
    mesh: CadWorkpieceMesh | null,
    placement: WorkpieceMeshPlacement | null,
    options?: WorkpieceMeshOptions,
  ): void {
    const identity = mesh ? workpieceMeshIdentity(mesh, options) : "";
    if (identity !== "" && identity === this.workpieceIdentity) {
      // Same part, same display options: this is a placement edit, so move the
      // existing layer instead of re-uploading its buffers.
      this.setWorkpiecePlacement(placement);
      return;
    }
    if (identity === "" && this.workpieceIdentity === "") {
      // Saved CAD edges can be placed before surface tessellation arrives.
      // Keep their handle anchor current even when there is no mesh to upload.
      this.setWorkpiecePlacement(placement);
      return;
    }
    this.workpieceIdentity = identity;
    this.elements.stage.dataset.workpieceTriangleCount = mesh
      ? String(mesh.triangleCount)
      : "";
    this.elements.stage.dataset.workpieceFingerprint = mesh?.fingerprint ?? "";
    this.workpieceLayer = this.replaceLayer(
      this.workpieceLayer,
      createWorkpieceMeshGroup(mesh, placement, options),
    );
    this.setWorkpiecePlacement(placement);
  }

  /**
   * The workcell fixtures, drawn where the plan request will say they are.
   * Rebuilt only when the list, a pose, a size or an arrived STEP mesh
   * changes; a Stand card keystroke rebuilds a box, which is twelve triangles.
   */
  setWorkcellFixtures(
    fixtures: readonly WorkcellFixture[],
    meshes: ReadonlyMap<string, CadWorkpieceMesh>,
  ): void {
    const identity = fixtures.map((fixture) => JSON.stringify([
      fixture.id,
      fixture.shape,
      fixture.translationM,
      fixture.rotationRpyRad,
      fixture.shape.kind === "step" ? meshes.has(fixture.shape.fingerprint.toLowerCase()) : true,
    ])).join("|");
    if (identity === this.fixtureIdentity) return;
    this.fixtureIdentity = identity;
    this.fixtureLayer = this.replaceLayer(this.fixtureLayer, createWorkcellFixtureGroup(fixtures, meshes));
  }

  /**
   * Move an already-loaded workpiece without rebuilding its geometry. Placement
   * edits fire on every keystroke in the offset fields, so re-uploading a
   * multi-megabyte buffer per edit is not acceptable.
   */
  setWorkpiecePlacement(placement: WorkpieceMeshPlacement | null): void {
    applyWorkpiecePlacement(this.workpieceLayer, placement);
    applyWorkpiecePlacement(this.waypointLayer, placement);
    // Mid-drag the pivot is the operator's; the project catches up on release.
    if (this.gizmoDrag) return;
    this.gizmoHasAnchor = placement?.anchorM !== undefined;
    if (placement?.anchorM) {
      this.gizmoPivot.position.set(...placement.anchorM);
      this.gizmoPivot.quaternion.copy(this.workpieceLayer.quaternion);
    }
    this.syncGizmoAttachment();
  }

  /**
   * Show the part's placement triad (arrows to move it, arc knobs to turn
   * it), or hide it. The handles pivot on the placement anchor (the
   * part's base centre), drag in world axes, move the mesh live, and report
   * the released pose once as a "workpiece-placed" event -- one project edit
   * per drag, not one per frame, because a placement edit regenerates every
   * CAD-derived row.
   */
  setWorkpieceGizmo(enabled: boolean): void {
    this.gizmoEnabled = enabled;
    if (enabled && !this.gizmo && !this.disposed) this.gizmo = this.createGizmo();
    this.syncGizmoAttachment();
  }

  get workpieceGizmoEnabled(): boolean {
    return this.gizmoEnabled;
  }

  private syncGizmoAttachment(): void {
    const gizmo = this.gizmo;
    if (!gizmo) return;
    const wanted = this.gizmoEnabled && this.gizmoHasAnchor;
    if (wanted && gizmo.object !== this.gizmoPivot) gizmo.attach(this.gizmoPivot);
    if (!wanted && gizmo.object) gizmo.detach();
    gizmo.helper.visible = wanted;
  }

  /**
   * Put the handle on a waypoint torch (a descendant of `waypointLayer`), or
   * take it off with null. The handle moves the object live; the pose is
   * reported once per drag as a "waypoint-moved" event, as the part's is.
   */
  setWaypointGizmoTarget(target: THREE.Object3D | null): void {
    this.waypointGizmoTarget = target;
    if (target && !this.waypointGizmo && !this.disposed) this.waypointGizmo = this.createWaypointGizmo();
    const gizmo = this.waypointGizmo;
    if (!gizmo || gizmo.object === target) return;
    if (target) gizmo.attach(target);
    else gizmo.detach();
  }

  private createWaypointGizmo(): PartGizmo {
    const gizmo = new PartGizmo({
      camera: this.orbitCamera,
      domElement: this.elements.orbitPane,
      callbacks: {
        onDraggingChanged: (dragging) => {
          this.controls.enabled = !dragging;
          this.emit({ type: "waypoint-drag", dragging });
        },
        onObjectChange: () => {
          const pose = this.waypointPose();
          if (pose) this.emit({ type: "waypoint-moving", ...pose });
        },
        onMouseUp: () => {
          const pose = this.waypointPose();
          if (pose) this.emit({ type: "waypoint-moved", ...pose });
        },
      },
    });
    this.scene.add(gizmo.helper);
    return gizmo;
  }

  /** The handle's object, in the layer's frame -- the one waypoints are
   *  stored in; the object may sit under intermediate groups. */
  private waypointPose(): WaypointPose | null {
    const target = this.waypointGizmoTarget;
    if (!target) return null;
    this.waypointLayer.updateWorldMatrix(true, false);
    target.updateWorldMatrix(true, false);
    const local = this.waypointLayer.matrixWorld.clone().invert().multiply(target.matrixWorld);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    local.decompose(p, q, new THREE.Vector3());
    return { target, positionM: [p.x, p.y, p.z], quaternionXyzw: [q.x, q.y, q.z, q.w] };
  }

  private createGizmo(): PartGizmo {
    const gizmo = new PartGizmo({
      camera: this.orbitCamera,
      domElement: this.elements.orbitPane,
      callbacks: {
        onDraggingChanged: (dragging) => {
          // The orbit camera and the gizmo share the pane; one of them owns a drag.
          this.controls.enabled = !dragging;
          if (dragging) {
            this.gizmoPivot.updateMatrix();
            this.workpieceLayer.updateMatrix();
            this.gizmoDrag = {
              pivotInverse: this.gizmoPivot.matrix.clone().invert(),
              mesh: this.workpieceLayer.matrix.clone(),
            };
          } else {
            this.gizmoDrag = null;
          }
          this.emit({ type: "workpiece-drag", dragging });
        },
        onObjectChange: () => {
          const drag = this.gizmoDrag;
          if (!drag) return;
          // mesh = pivot · pivot0⁻¹ · mesh0: the part rides the handle rigidly.
          this.gizmoPivot.updateMatrix();
          const next = this.gizmoPivot.matrix.clone().multiply(drag.pivotInverse).multiply(drag.mesh);
          next.decompose(this.workpieceLayer.position, this.workpieceLayer.quaternion, this.workpieceLayer.scale);
        },
        onMouseUp: () => {
          const p = this.gizmoPivot.position;
          const q = this.gizmoPivot.quaternion;
          this.emit({
            type: "workpiece-placed",
            anchorM: [p.x, p.y, p.z],
            rotationXyzw: [q.x, q.y, q.z, q.w],
          });
        },
      },
    });
    this.scene.add(gizmo.helper);
    return gizmo;
  }

  setSeams(
    seams: readonly SeamOverlay[],
    palette?: OverlayPalette,
  ): void {
    this.seamLayer = this.replaceLayer(
      this.seamLayer,
      createSeamOverlayGroup(seams, palette),
    );
  }

  setCollisionMarkers(markers: readonly CollisionMarker[]): void {
    this.collisionLayer = this.replaceLayer(
      this.collisionLayer,
      createCollisionMarkerGroup(markers),
    );
  }

  setTcpFrames(frames: readonly TcpFrameMarker[]): void {
    this.tcpFrameLayer = this.replaceLayer(
      this.tcpFrameLayer,
      createTcpFrameGroup(frames),
    );
  }

  setCurveCenters(centers: readonly CurveCenterMarker[]): void {
    this.curveCenterLayer = this.replaceLayer(
      this.curveCenterLayer,
      createCurveCenterGroup(centers),
    );
  }

  setCurrentTcpPose(pose: TcpPose | null): void {
    if (pose) {
      applyTcpPose(this.currentTcp, pose);
      this.tcpPoseKnown = true;
    }
    // A null sample no longer hides the frame — the static pose derived in
    // applyResolvedPose keeps showing where the TCP actually is.
    this.refreshTcpFrameVisibility();
  }

  /**
   * Jog support: the current joint state in the descriptor's pose joint
   * order, with URDF limits where the model defines them. Feed edited values
   * back through setPose with this same order.
   */
  jogJointStates(): {
    name: string;
    valueRad: number;
    lowerRad: number;
    upperRad: number;
  }[] {
    const resolved = this.poseLatch.resolve();
    return resolved.pose.jointOrder.map((name, index) => {
      const joint = this.robot?.joints?.[name];
      const limit = joint?.limit as { lower?: number | null; upper?: number | null } | undefined;
      const lower = Number(limit?.lower);
      const upper = Number(limit?.upper);
      const hasLimits = Number.isFinite(lower) && Number.isFinite(upper) && upper > lower;
      return {
        name,
        valueRad: resolved.pose.jointsRad[index] ?? 0,
        lowerRad: hasLimits ? lower : -Math.PI,
        upperRad: hasLimits ? upper : Math.PI,
      };
    });
  }

  /** The descriptor's home pose in its pose joint order, for a jog reset. */
  homeJogPose(): { jointOrder: string[]; jointsRad: number[] } {
    return {
      jointOrder: [...this.descriptor.poseJointOrder],
      jointsRad: [...this.descriptor.homePoseRad],
    };
  }

  /**
   * World-space pose of the TCP marker, URDF fixed-axis RPY components
   * (three.js Euler order "ZYX" — the same convention applyTcpPose decodes).
   * Null until a robot pose has been applied.
   */
  currentTcpPoseWorld(): [number, number, number, number, number, number] | null {
    if (!this.tcpPoseKnown) return null;
    const euler = new THREE.Euler().setFromQuaternion(this.currentTcp.quaternion, "ZYX");
    const position = this.currentTcp.position;
    return [position.x, position.y, position.z, euler.x, euler.y, euler.z];
  }

  /** FK of an explicit captured pose; does not change the displayed robot. */
  tcpPoseFor(pose: RobotPoseSnapshot<string>, correctedUrdf: string): [number, number, number, number, number, number] {
    if (!this.robot || pose.modelId !== this.descriptor.id) throw new Error("Robot model is not ready for this capture");
    const loader = new URDFLoader();
    loader.parseVisual = false; loader.parseCollision = false;
    const copy = loader.parse(correctedUrdf);
    for (const name of this.descriptor.urdfJointNames) {
      const index = pose.jointOrder.indexOf(name), value = pose.jointsRad[index];
      if (index < 0 || value === undefined || !Number.isFinite(value) || !copy.joints[name]) throw new Error(`Capture is missing ${name}`);
      copy.joints[name]!.ignoreLimits = true;
      copy.setJointValue(name, value);
    }
    const frame = copy.links[this.descriptor.cameras.tcp.frame] ?? copy.joints[this.descriptor.cameras.tcp.frame];
    if (!frame) throw new Error("The robot description has no TCP frame");
    frame.updateWorldMatrix(true, false);
    const position = new THREE.Vector3().setFromMatrixPosition(frame.matrixWorld);
    const rotation = new THREE.Euler().setFromRotationMatrix(frame.matrixWorld, "ZYX");
    return [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z];
  }

  /**
   * Rotate a tool-frame vector into the world/base frame using the current
   * TCP orientation (tool-frame jogging: the daemon consumes base-frame
   * twists, so the pendant rotates the stick by the live tool pose).
   */
  rotateTcpVectorToWorld(vector: [number, number, number]): [number, number, number] {
    const rotated = new THREE.Vector3(vector[0], vector[1], vector[2])
      .applyQuaternion(this.currentTcp.quaternion);
    return [rotated.x, rotated.y, rotated.z];
  }

  /** Toolbar toggle: show or hide the TCP position marker and axes. */
  setTcpFrameEnabled(enabled: boolean): void {
    this.tcpFrameEnabled = enabled;
    this.refreshTcpFrameVisibility();
  }

  private refreshTcpFrameVisibility(): void {
    this.currentTcp.visible = this.tcpFrameEnabled && this.tcpPoseKnown;
  }

  setSplitView(enabled: boolean): void {
    this.splitView = enabled;
    this.elements.stage.classList.toggle("workstation-split", enabled);
    this.elements.stage.dataset.workstationLayout = enabled ? "orbit-j6-tcp" : "orbit";
    this.elements.j6Pane.hidden = !enabled;
    this.elements.tcpPane.hidden = !enabled;
    this.resize();
  }

  setPaneVisibility(visibility: { j6: boolean; tcp: boolean }): void {
    this.j6PaneVisible = visibility.j6;
    this.tcpPaneVisible = visibility.tcp;
    this.elements.j6Pane.hidden = !visibility.j6;
    this.elements.tcpPane.hidden = !visibility.tcp;
    const anyCamVisible = visibility.j6 || visibility.tcp;
    this.elements.stage.dataset.j6Cam = visibility.j6 ? "visible" : "hidden";
    this.elements.stage.dataset.tcpCam = visibility.tcp ? "visible" : "hidden";
    // Keep the split class in sync — orbit-only when both cams are off.
    this.elements.stage.classList.toggle("workstation-split", this.splitView && anyCamVisible);
    this.resize();
  }

  resetOrbitCamera(): void {
    this.focusTween = null;
    this.orbitCamera.position.set(...this.descriptor.orbitHome.positionM);
    this.controls.target.set(...this.descriptor.orbitHome.targetM);
    this.controls.update();
  }

  /**
   * Gently pan the orbit camera so the given points are centred, preserving
   * the current viewing direction and distance. Used for click-to-focus on a
   * spreadsheet move; `framePath` remains the zoom-to-fit action.
   */
  focusOn(pointsM: readonly [number, number, number][], durationMs = 350): void {
    this.assertActive();
    if (this.controls.interacting || !this.controls.enabled) return;
    const bounds = boundsOfPoints(pointsM as [number, number, number][]);
    if (!bounds) return;
    const toTarget = new THREE.Vector3(...bounds.center);
    const offset = this.orbitCamera.position.clone().sub(this.controls.target);
    this.elements.stage.dataset.focusTarget = bounds.center
      .map((value) => value.toFixed(4))
      .join(",");
    if (durationMs <= 0) {
      this.controls.target.copy(toTarget);
      this.orbitCamera.position.copy(toTarget).add(offset);
      this.controls.update();
      this.focusTween = null;
      return;
    }
    this.focusTween = {
      fromTarget: this.controls.target.clone(),
      toTarget,
      fromPosition: this.orbitCamera.position.clone(),
      toPosition: toTarget.clone().add(offset),
      startedAtMs: performance.now(),
      durationMs,
    };
  }

  /**
   * Part focus: everything that is not the part -- robot, cell, grid, path
   * -- goes translucent. Opening or closing a weld tool never moves the
   * camera; Center camera and the view Home/Frame button remain explicit.
   * Off restores the surrounding materials.
   *
   * Materials are dimmed in place and their prior state remembered per
   * material object, because a mesh may share its material with others
   * and cloning would leave the robot's own material updates unseen.
   */
  setPartFocus(enabled: boolean): void {
    if (enabled === (this.partFocus !== null)) return;
    this.focusTween = null;
    if (!enabled) {
      const focus = this.partFocus!;
      for (const [material, before] of focus.materials) {
        material.opacity = before.opacity;
        material.transparent = before.transparent;
        material.depthWrite = before.depthWrite;
        material.needsUpdate = true;
      }
      this.partFocus = null;
      return;
    }
    const materials = new Map<THREE.Material, { opacity: number; transparent: boolean; depthWrite: boolean }>();
    this.scene.traverse((node) => {
      if (node === this.workFrameLayer || isUnder(node, this.workFrameLayer)) return;
      if (node === this.plannedPathLayer || isUnder(node, this.plannedPathLayer)) return;
      if (node === this.executionPathLayer || isUnder(node, this.executionPathLayer)) return;
      if (this.gizmo && (node === this.gizmo.helper || isUnder(node, this.gizmo.helper))) return;
      if (this.waypointGizmo && (node === this.waypointGizmo.helper || isUnder(node, this.waypointGizmo.helper))) return;
      const owned = (node as THREE.Mesh).material as THREE.Material | THREE.Material[] | undefined;
      if (!owned) return;
      for (const material of Array.isArray(owned) ? owned : [owned]) {
        if (materials.has(material)) continue;
        materials.set(material, {
          opacity: material.opacity,
          transparent: material.transparent,
          depthWrite: material.depthWrite,
        });
        material.transparent = true;
        material.opacity = Math.min(material.opacity, PART_FOCUS_OPACITY);
        material.depthWrite = false;
        material.needsUpdate = true;
      }
    });
    this.partFocus = {
      materials,
      dim: PART_FOCUS_OPACITY,
    };
  }

  get partFocused(): boolean {
    return this.partFocus !== null;
  }

  /** Under part focus, how faint the rest of the cell is: `opacity` 1 is
   *  solid, 0 is gone. The camera stays on the part either way. A no-op
   *  outside focus. */
  setPartDim(opacity: number): void {
    const focus = this.partFocus;
    if (!focus) return;
    const level = Math.min(1, Math.max(0, opacity));
    for (const [material, before] of focus.materials) {
      if (level >= 1) {
        material.opacity = before.opacity;
        material.transparent = before.transparent;
        material.depthWrite = before.depthWrite;
      } else {
        material.transparent = true;
        material.opacity = Math.min(before.opacity, level);
        material.depthWrite = false;
      }
      material.needsUpdate = true;
    }
    focus.dim = level;
  }

  get partDim(): number {
    return this.partFocus?.dim ?? 1;
  }

  /** The centre of the part's top face, in the part's own frame (the frame
   *  `waypointLayer` draws in): where a first waypoint's work point goes, so
   *  its torch starts on the part rather than inside it. Null with no part
   *  loaded. */
  partTopCentre(): [number, number, number] | null {
    const layer = this.workpieceLayer;
    layer.updateWorldMatrix(true, true);
    const toLayer = layer.matrixWorld.clone().invert();
    const box = new THREE.Box3();
    const part = new THREE.Box3();
    layer.traverse((node) => {
      const geometry = (node as THREE.Mesh).geometry as THREE.BufferGeometry | undefined;
      if (!(node as THREE.Mesh).isMesh || !geometry) return;
      if (!geometry.boundingBox) geometry.computeBoundingBox();
      part.copy(geometry.boundingBox!).applyMatrix4(toLayer.clone().multiply(node.matrixWorld));
      box.union(part);
    });
    if (box.isEmpty()) return null;
    const centre = box.getCenter(new THREE.Vector3());
    return [centre.x, centre.y, box.max.z];
  }

  /** Orbit about the part's centre, close enough that it fills the view. */
  framePart(): void {
    this.focusTween = null;
    const box = new THREE.Box3().setFromObject(this.workpieceLayer);
    if (box.isEmpty()) return;
    const center = box.getCenter(new THREE.Vector3());
    const radiusM = Math.max(0.05, box.getSize(new THREE.Vector3()).length() / 2);
    const stage = this.elements.stage.getBoundingClientRect();
    const aspect = Math.max(0.25, stage.width / Math.max(1, stage.height));
    const verticalFov = THREE.MathUtils.degToRad(this.orbitCamera.fov);
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * aspect);
    const fitFov = Math.max(0.1, Math.min(verticalFov, horizontalFov));
    const distanceM = radiusM / Math.sin(fitFov / 2) * 1.25;
    const direction = this.orbitCamera.position.clone().sub(this.controls.target);
    if (direction.lengthSq() < 1e-9) direction.set(1, -1, 0.75);
    direction.normalize();
    this.controls.target.copy(center);
    this.orbitCamera.position.copy(center).addScaledVector(direction, distanceM);
    this.controls.update();
  }

  framePath(points: readonly MotionPathPoint[]): void {
    this.focusTween = null;
    if (this.plannedPathVisible && this.plannedPathPoints.length > 0) points = this.plannedPathPoints;
    const bounds = boundsOfPoints(points.map(({ positionM }) => positionM));
    if (!bounds) {
      this.resetOrbitCamera();
      return;
    }
    const center = new THREE.Vector3(...bounds.center);
    const radiusM = Math.max(0.3, Math.hypot(...bounds.size) / 2);
    const stage = this.elements.stage.getBoundingClientRect();
    const aspect = Math.max(0.25, stage.width / Math.max(1, stage.height));
    const verticalFov = THREE.MathUtils.degToRad(this.orbitCamera.fov);
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * aspect);
    const fitFov = Math.max(0.1, Math.min(verticalFov, horizontalFov));
    const distanceM = Math.max(
      1.4,
      Math.min(12, radiusM / Math.sin(fitFov / 2) * 1.6),
    );
    const direction = this.orbitCamera.position.clone().sub(this.controls.target);
    if (direction.lengthSq() < 1e-9) direction.set(1, -1, 0.75);
    direction.normalize();
    this.controls.target.copy(center);
    this.orbitCamera.position.copy(center).addScaledVector(direction, distanceM);
    this.controls.update();
  }

  resize(): void {
    if (this.disposed) return;
    this.syncRendererSize();
    const diagnostics = this.viewportDiagnostics();
    if (this.splitView && !diagnostics.validation.ok) {
      this.emit({ type: "layout-warning", validation: diagnostics.validation });
    }
    if (this.animationFrame === null) this.renderNow();
  }

  viewportDiagnostics(): WorkstationViewportDiagnostics {
    const stage = rect(this.elements.stage.getBoundingClientRect());
    const canvas = rect(this.elements.canvas.getBoundingClientRect());
    const orbit = rect(this.elements.orbitPane.getBoundingClientRect());
    const j6 = rect(this.elements.j6Pane.getBoundingClientRect());
    const tcp = rect(this.elements.tcpPane.getBoundingClientRect());
    return {
      validation: validateScissorPaneLayout(stage, canvas, orbit, j6, tcp),
      orbit: viewportFromRects(canvas, orbit),
      j6: viewportFromRects(canvas, j6),
      tcp: viewportFromRects(canvas, tcp),
      pixelRatio: this.renderer.getPixelRatio(),
    };
  }

  renderNow(): void {
    if (this.disposed) return;
    // Browser startup/maximise and display-scale changes are not guaranteed to
    // emit a useful window resize event. The running viewport repairs itself
    // from the live stage dimensions on the next rendered frame.
    this.syncRendererSize();
    const tween = this.focusTween;
    if (tween) {
      const raw = (performance.now() - tween.startedAtMs) / tween.durationMs;
      const t = Math.max(0, Math.min(1, raw));
      const eased = t * t * (3 - 2 * t);
      this.controls.target.lerpVectors(tween.fromTarget, tween.toTarget, eased);
      this.orbitCamera.position.lerpVectors(
        tween.fromPosition,
        tween.toPosition,
        eased,
      );
      if (t >= 1) this.focusTween = null;
    }
    this.controls.update();
    this.robot?.updateMatrixWorld(true);
    // Rebuild the shadow map once for this frame; the per-pane render() calls
    // below reuse it (autoUpdate is off) instead of each rebuilding it.
    this.renderer.shadowMap.needsUpdate = true;
    const hasJ6 = this.mountTargetCamera(this.j6Camera, this.descriptor.cameras.j6);
    const hasTcp = this.mountForwardCamera(this.tcpCamera, this.descriptor.cameras.tcp);

    this.renderer.setScissorTest(false);
    this.renderer.clear(true, true, true);
    // The gizmo answers to the orbit camera's pointer only; drawn in the
    // J6/TCP panes it would be a handle nothing can grab.
    const helpers = [this.gizmo, this.waypointGizmo].flatMap((gizmo) =>
      gizmo ? [{ helper: gizmo.helper, shown: gizmo.helper.visible }] : []
    );
    this.gizmo?.update(this.rendererPixelRatio);
    this.waypointGizmo?.update(this.rendererPixelRatio);
    this.renderPane(this.orbitCamera, this.elements.orbitPane);
    this.worldAxisIndicator.draw(this.orbitCamera);
    for (const { helper } of helpers) helper.visible = false;
    if (this.splitView) {
      if (!this.elements.j6Pane.hidden) {
        this.renderPane(hasJ6 ? this.j6Camera : this.orbitCamera, this.elements.j6Pane);
      }
      if (!this.elements.tcpPane.hidden) {
        this.renderPane(
          hasTcp ? this.tcpCamera : this.orbitCamera,
          this.elements.tcpPane,
          this.descriptor.cameras.tcp.hideFrames,
        );
      }
    }
    for (const { helper, shown } of helpers) helper.visible = shown;
    this.renderer.setScissorTest(false);
  }

  dispose(): void {
    if (this.disposed) return;
    this.stop();
    this.disposed = true;
    this.loadGeneration += 1;
    this.resizeObserver?.disconnect();
    this.resizeObserver = null;
    this.controls.dispose();
    this.gizmo?.dispose();
    this.gizmo = null;
    this.waypointGizmo?.dispose();
    this.waypointGizmo = null;
    disposeObjectTree(this.scene);
    this.scene.clear();
    this.renderer.renderLists.dispose();
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.robot = null;
    this.workFrameZeroInverse = null;
    this.followWorkFrame();
  }

  private animate = (): void => {
    this.animationFrame = null;
    if (this.disposed) return;
    this.renderNow();
    this.animationFrame = requestAnimationFrame(this.animate);
  };

  /**
   * Repaint the scene's ground, fog, floor grid and sky light from the
   * theme's viewport tokens (style.css `--viewport-*`). Called once at
   * construction and again whenever the window's theme changes; geometry,
   * paths and the robot keep their own colours.
   */
  applyTheme(): void {
    const color = (name: string, fallback: string) => new THREE.Color(themeToken(name, fallback));
    const ground = color("--viewport-bg", "#cfd2d5");
    (this.scene.background as THREE.Color).copy(ground);
    this.scene.fog?.color.copy(ground);
    this.renderer.setClearColor(ground, 1);
    this.hemisphere.color.copy(color("--viewport-sky", "#f6f7f8"));
    this.hemisphere.groundColor.copy(color("--viewport-ground", "#8f979e"));
    // A GridHelper's two weights are baked into its vertex colours, so a
    // new theme is a new grid.
    if (this.floorGrid) {
      this.scene.remove(this.floorGrid);
      this.floorGrid.geometry.dispose();
      (this.floorGrid.material as THREE.Material).dispose();
    }
    this.floorGrid = new THREE.GridHelper(12, 48, color("--viewport-grid", "#8f979e"), color("--viewport-grid-minor", "#b4b9be"));
    this.floorGrid.rotation.x = Math.PI / 2;
    this.scene.add(this.floorGrid);
    this.worldAxisIndicator.invalidate();
  }

  private readonly hemisphere = new THREE.HemisphereLight(0xf6f7f8, 0x8f979e, 1.4);
  private floorGrid: THREE.GridHelper | null = null;

  private configureScene(): void {
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.3));
    this.scene.add(this.hemisphere);
    const key = new THREE.DirectionalLight(0xffffff, 2.6);
    key.position.set(5, -4, 7);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xdcdfe2, 1.1);
    fill.position.set(-4, 3, 2);
    this.scene.add(fill);
    this.pathLayer.name = "workstation-path";
    this.plannedPathLayer.name = "workstation-planned-tcp-path";
    this.pathDecorationLayer.name = "workstation-path-decorations";
    this.selectionLayer.name = "workstation-selection";
    this.workFrameLayer.name = "workstation-work-frame";
    this.workpieceLayer.name = "workstation-workpiece";
    this.seamLayer.name = "workstation-seams";
    this.gizmoPivot.name = "workstation-workpiece-gizmo-pivot";
    this.waypointLayer.name = "workstation-waypoints";
    this.workFrameLayer.add(this.workpieceLayer, this.seamLayer, this.gizmoPivot, this.waypointLayer);
    this.fixtureLayer.name = "workstation-fixtures";
    this.scene.add(this.fixtureLayer);
    this.collisionLayer.name = "workstation-collisions";
    this.tcpFrameLayer.name = "workstation-tcp-frames";
    this.curveCenterLayer.name = "workstation-curve-centers";
    this.scene.add(
      this.workFrameLayer,
      this.pathLayer,
      this.plannedPathLayer,
      this.executionPathLayer,
      this.pathDecorationLayer,
      this.selectionLayer,
      this.collisionLayer,
      this.tcpFrameLayer,
      this.curveCenterLayer,
      this.currentTcp,
    );
  }

  private applyCameraDescriptor(descriptor: RobotModelDescriptor<string>): void {
    this.scene.up.set(...descriptor.worldUp);
    this.orbitCamera.up.set(...descriptor.worldUp);
    configureCamera(this.j6Camera, descriptor.cameras.j6);
    configureCamera(this.tcpCamera, descriptor.cameras.tcp);
    this.j6Camera.matrixAutoUpdate = false;
    this.tcpCamera.matrixAutoUpdate = false;
  }

  private applyResolvedPose(resolved: ReturnType<PoseLatch<string>["resolve"]>): void {
    this.elements.stage.dataset.poseJointOrder =
      resolved.pose.jointOrder.join(",");
    this.elements.stage.dataset.poseJointsRad =
      resolved.pose.jointsRad.join(",");
    this.elements.stage.dataset.poseSource = resolved.pose.source;
    this.elements.stage.dataset.poseMode = resolved.mode;
    if (this.robot) {
      const values = new Map(
        resolved.pose.jointOrder.map((name, index) => [name, resolved.pose.jointsRad[index]!] as const),
      );
      for (const jointName of this.descriptor.urdfJointNames) {
        const value = values.get(jointName);
        if (value !== undefined) this.robot.setJointValue(jointName, value);
      }
      this.robot.updateMatrixWorld(true);
      this.followWorkFrame();
      // Keep the TCP marker on the tool frame at every pose, not just during
      // playback samples: the operator sees where the TCP actually is.
      const tcpFrame = this.frame(this.descriptor.cameras.tcp.frame);
      if (tcpFrame) {
        this.currentTcp.position.setFromMatrixPosition(tcpFrame.matrixWorld);
        this.currentTcp.quaternion.setFromRotationMatrix(tcpFrame.matrixWorld);
        this.tcpPoseKnown = true;
        this.refreshTcpFrameVisibility();
      }
    }
    this.emit({ type: "pose", modelId: this.descriptor.id, mode: resolved.mode });
  }

  private mountTargetCamera(
    camera: THREE.PerspectiveCamera,
    mount: RobotModelDescriptor<string>["cameras"]["j6"],
  ): boolean {
    const frame = this.frame(mount.frame);
    const target = this.frame(mount.targetFrame);
    if (!frame || !target) return false;
    frame.updateWorldMatrix(true, false);
    target.updateWorldMatrix(true, false);
    frame.getWorldPosition(this.scratchOrigin);
    frame.getWorldQuaternion(this.scratchQuaternion);
    target.getWorldPosition(this.scratchTarget);
    this.scratchPosition.copy(this.scratchOrigin).add(
      vector(mount.lensOffsetLocalM).applyQuaternion(this.scratchQuaternion),
    );
    this.scratchForward.copy(this.scratchTarget).sub(this.scratchPosition);
    if (this.scratchForward.lengthSq() < 1e-10) return false;
    this.scratchForward.normalize();
    this.scratchUp.copy(vector(mount.upLocal)).applyQuaternion(this.scratchQuaternion).normalize();
    return this.commitCameraMatrix(camera);
  }

  private mountForwardCamera(
    camera: THREE.PerspectiveCamera,
    mount: RobotModelDescriptor<string>["cameras"]["tcp"],
  ): boolean {
    const frame = this.frame(mount.frame);
    if (!frame) return false;
    frame.updateWorldMatrix(true, false);
    frame.getWorldPosition(this.scratchOrigin);
    frame.getWorldQuaternion(this.scratchQuaternion);
    this.scratchPosition.copy(this.scratchOrigin).add(
      vector(mount.lensOffsetLocalM).applyQuaternion(this.scratchQuaternion),
    );
    this.scratchForward.copy(vector(mount.forwardLocal))
      .applyQuaternion(this.scratchQuaternion)
      .normalize();
    this.scratchUp.copy(vector(mount.upLocal)).applyQuaternion(this.scratchQuaternion).normalize();
    return this.commitCameraMatrix(camera);
  }

  private commitCameraMatrix(camera: THREE.PerspectiveCamera): boolean {
    this.scratchRight.crossVectors(this.scratchForward, this.scratchUp);
    if (this.scratchRight.lengthSq() < 1e-8) {
      this.scratchUp.set(0, 1, 0).applyQuaternion(this.scratchQuaternion).normalize();
      this.scratchRight.crossVectors(this.scratchForward, this.scratchUp);
    }
    if (this.scratchRight.lengthSq() < 1e-8) return false;
    this.scratchRight.normalize();
    this.scratchUp.crossVectors(this.scratchRight, this.scratchForward).normalize();
    this.scratchNegForward.copy(this.scratchForward).negate();
    this.scratchBasis.makeBasis(
      this.scratchRight,
      this.scratchUp,
      this.scratchNegForward,
    );
    this.scratchBasis.setPosition(this.scratchPosition);
    camera.matrix.copy(this.scratchBasis);
    camera.matrixWorld.copy(this.scratchBasis);
    camera.matrixWorldInverse.copy(this.scratchBasis).invert();
    camera.position.copy(this.scratchPosition);
    return true;
  }

  private frame(name: string): THREE.Object3D | null {
    return this.robot?.frames[name] ?? this.robot?.links[name] ?? null;
  }

  /** Pose the work-frame follower at the table's departure from its zero:
   *  T_link_now · T_link_zero⁻¹, so at the zero pose it is the identity and
   *  placements keep meaning exactly what they meant. */
  private followWorkFrame(): void {
    const link = this.descriptor.workFrameLink
      ? this.frame(this.descriptor.workFrameLink)
      : null;
    if (!link || !this.workFrameZeroInverse) {
      this.workFrameLayer.position.set(0, 0, 0);
      this.workFrameLayer.quaternion.identity();
      this.workFrameLayer.scale.set(1, 1, 1);
      return;
    }
    const follow = new THREE.Matrix4().multiplyMatrices(
      link.matrixWorld,
      this.workFrameZeroInverse,
    );
    follow.decompose(
      this.workFrameLayer.position,
      this.workFrameLayer.quaternion,
      this.workFrameLayer.scale,
    );
    this.workFrameLayer.updateMatrixWorld(true);
  }

  private renderPane(
    camera: THREE.PerspectiveCamera,
    pane: HTMLElement,
    hiddenFrameNames: readonly string[] = [],
  ): void {
    if (pane.hidden) return;
    const canvasRect = rect(this.elements.canvas.getBoundingClientRect());
    const paneRect = rect(pane.getBoundingClientRect());
    const view = viewportFromRects(canvasRect, paneRect);
    if (view.width < 2 || view.height < 2) return;
    camera.aspect = view.aspect;
    camera.updateProjectionMatrix();
    this.renderer.setViewport(view.x, view.y, view.width, view.height);
    this.renderer.setScissor(view.x, view.y, view.width, view.height);
    this.renderer.setScissorTest(true);

    const hidden = hiddenFrameNames
      .map((name) => this.frame(name))
      .filter((frame): frame is THREE.Object3D => frame !== null)
      .map((frame) => ({ frame, visible: frame.visible }));
    hidden.forEach(({ frame }) => {
      frame.visible = false;
    });
    try {
      this.renderer.clear(true, true, true);
      this.renderer.render(this.scene, camera);
    } finally {
      hidden.forEach(({ frame, visible }) => {
        frame.visible = visible;
      });
    }
  }

  private syncRendererSize(): void {
    const box = this.elements.stage.getBoundingClientRect();
    const size = rendererSize(
      box.width,
      box.height,
      window.devicePixelRatio || 1,
      this.pixelRatioCap,
    );
    if (
      size.width === this.rendererWidth &&
      size.height === this.rendererHeight &&
      size.pixelRatio === this.rendererPixelRatio
    ) return;
    this.rendererWidth = size.width;
    this.rendererHeight = size.height;
    this.rendererPixelRatio = size.pixelRatio;
    this.renderer.setPixelRatio(size.pixelRatio);
    this.renderer.setSize(size.width, size.height, false);
  }

  private replaceLayer(current: THREE.Group, next: THREE.Group): THREE.Group {
    // A layer is replaced where it lives — the workpiece and seam layers sit
    // under the work-frame follower, the rest at the scene root.
    const parent = current.parent ?? this.scene;
    parent.remove(current);
    disposeObjectTree(current);
    current.clear();
    parent.add(next);
    return next;
  }

  private emit(event: WorkstationEvent): void {
    this.onEvent?.(event);
  }

  private assertActive(): void {
    if (this.disposed) throw new Error("workstation renderer has been disposed");
  }
}

function createCurrentTcpFrame(): THREE.Group {
  const group = new THREE.Group();
  group.name = "workstation-current-tcp";
  group.visible = false;
  const axes = new THREE.AxesHelper(0.12);
  const axisMaterials = Array.isArray(axes.material) ? axes.material : [axes.material];
  axisMaterials.forEach((material) => {
    material.depthTest = false;
    material.depthWrite = false;
  });
  axes.renderOrder = 130;
  group.add(axes);
  const center = createTcpDotSprite();
  center.renderOrder = 131;
  group.add(center);
  return group;
}

/**
 * Fixed-screen-size TCP dot — a billboard sprite so it never blocks the
 * workpiece regardless of zoom. About 1/5th the on-screen size of the old
 * 0.012 r sphere.
 */
function createTcpDotSprite(): THREE.Sprite {
  const canvas = document.createElement("canvas");
  const size = 32;
  canvas.width = size;
  canvas.height = size;
  const context = canvas.getContext("2d");
  if (context) {
    context.fillStyle = "#ffffff";
    context.beginPath();
    context.arc(size / 2, size / 2, size / 2 - 1, 0, Math.PI * 2);
    context.fill();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: texture,
      sizeAttenuation: false,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
      transparent: true,
    }),
  );
  sprite.name = "workstation-tcp-dot";
  // With sizeAttenuation off, scale is in view units at distance 1 —
  // roughly scaleY * 1.18 of the viewport height for the 46 deg orbit camera.
  // 0.004 ≈ 1/5th of the old 0.012 r sphere's on-screen size.
  sprite.scale.set(0.004, 0.004, 1);
  return sprite;
}

function configureCamera(
  camera: THREE.PerspectiveCamera,
  descriptor: { fovDeg: number; nearM: number; farM: number },
): void {
  camera.fov = descriptor.fovDeg;
  camera.near = descriptor.nearM;
  camera.far = descriptor.farM;
  camera.updateProjectionMatrix();
}

function vector(value: readonly [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(value[0], value[1], value[2]);
}

function rect(value: DOMRect | DOMRectReadOnly): RectLike {
  return {
    left: value.left,
    top: value.top,
    right: value.right,
    bottom: value.bottom,
    width: value.width,
    height: value.height,
  };
}
