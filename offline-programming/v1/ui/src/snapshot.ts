/**
 * Scene snapshots of one program, drawn by the workstation's own renderer.
 *
 * An agent that edits a program cannot see it. A part placed a metre from
 * the arm, a weld on the wrong face or a plan that sweeps through the stand
 * reads as plausible numbers in a JSON document and is obvious in a picture.
 * This page makes the picture: `WorkstationRenderer`, `syncProjectToWorkstation`,
 * the planned-path tracer and the weld overlays are the ones the app draws
 * with, so what a snapshot shows is what the operator would see -- a second
 * renderer would be a second opinion about where things are.
 *
 * It imports only viewer and domain modules, never `main.ts`: the app boots
 * IndexedDB, polls its API and never settles, which hangs a headless
 * browser's capture (AGENT_SCRATCHPAD 2026-10-07). Driven over CDP by
 * `cli/v1/src/lib/snapshot`; built by `vite.snapshot.config.ts`.
 *
 * Nothing here repairs a program. A document OLP cannot import, a robot it
 * cannot load or a mesh it cannot read is an error in `load()`'s answer, and
 * anything not drawn (a part with no mesh, a plan whose bytes were not given,
 * a plan for another revision) is said in its warnings: a snapshot that
 * silently leaves something out shows a cell that does not exist.
 */

import * as THREE from "three";

import { normalizeWorkpieceMesh } from "./cad/normalize.ts";
import type { CadWorkpieceMesh } from "./cad/contracts.ts";
import type { OfflineProgrammingProject, PlanTrajectorySample, Vec3 } from "./domain/project.ts";
import { planIsFresh } from "./domain/project.ts";
import { knownRobotModel, parseRobotCatalogue, setRobotCatalogue } from "./domain/robots.ts";
import { importProjectJSON } from "./domain/serialization.ts";
import { environmentFixtures } from "./domain/workpieces.ts";
import { decodeDenseTrajectory, type DenseTrajectory } from "./execution/denseTrajectoryBlob.ts";
import { rowNodeId } from "./program-v2/rows-from-nodes.ts";
import { weldPathPoints } from "./program-v2/weld-path.ts";
import type { SimulationSample } from "./simulation/contracts.ts";
import type { MotionPathPoint } from "./viewer/geometry.ts";
import { robotModelFor } from "./viewer/models.ts";
import { densePlannedPathRuns, projectPlannedPath } from "./viewer/planned-path.ts";
import { WeldHighlight } from "./viewer/weld-highlight.ts";
import { applyDenseSample, applyProjectSample, projectPath, syncProjectToWorkstation } from "./viewer/workstation.ts";
import { WorkstationRenderer } from "./viewer/workstation-renderer.ts";
import { WaypointOverlay } from "./waypoints/overlay.ts";
import { isManualPoseWeld, waypointWeldOf } from "./waypoints/node.ts";

export interface SnapshotLoadInput {
  /** `GET /api/offline-programming/v1/robots`, as the server answered. */
  catalogue: unknown;
  /** The program: a bare project (a cell store document) or an export
   *  envelope `{schema, exportedAt, project}`. */
  document: unknown;
  /** Lowercase STEP fingerprint -> the CadQuery tessellate response body. */
  meshes?: Record<string, unknown>;
  /** The program's dense plan, robot.v4.dense-joint-trajectory.v1 bytes. */
  denseBase64?: string;
  width?: number;
  height?: number;
  showLabels?: boolean;
}

export type MmBounds = [number[], number[]];

export interface SnapshotScene {
  modelId: string;
  partTriangles: number;
  fixtures: number;
  pathPoints: number;
  plannedPathPoints: number;
  plannedStale: boolean;
  welds: number;
  boundsMm: { part?: MmBounds; path?: MmBounds };
}

export interface SnapshotLoadResult {
  ok: boolean;
  errors: string[];
  warnings: string[];
  scene: SnapshotScene;
}

export type NamedCamera = "home" | "part" | "path" | "front" | "back" | "left" | "right" | "top" | "iso";

export interface SnapshotShotSpec {
  /** Joints in the model's pose order (J1..J9), radians; a shorter vector
   *  covering the URDF's articulated joints is padded with zeros. */
  pose?: number[] | { sample: "start" | "end" | number } | { row: string };
  camera: NamedCamera | { positionM: number[]; targetM: number[]; fovDeg?: number };
  tcpFrame?: boolean;
  partFocus?: boolean;
  plannedPath?: boolean;
}

export interface SnapshotShotResult {
  dataUrl: string;
  pose: { jointOrder: string[]; jointsRad: number[] };
  camera: { positionM: number[]; targetM: number[]; fovDeg: number };
  notes: string[];
}

/** The orbit camera's own field of view (`WorkstationRenderer.orbitCamera`). */
const ORBIT_FOV_DEG = 46;
/** How much of the frame the subject may fill: the fit radius is drawn this
 *  much larger than the bounds' sphere, so nothing touches the edge. */
const FRAME_MARGIN = 1.12;
/** Smallest framed radius: a single taught point still gets 30 cm of context. */
const MIN_FRAME_RADIUS_M = 0.15;

/** Named views in the cell's world frame (Z up). front looks back at the arm
 *  from +X, the side it reaches toward; left is the arm's left (+Y). top is
 *  a hair off vertical: OrbitControls has no defined azimuth straight down. */
const VIEW_DIRECTIONS: Record<"front" | "back" | "left" | "right" | "top" | "iso", Vec3> = {
  front: [1, 0, 0.18],
  back: [-1, 0, 0.18],
  left: [0, 1, 0.18],
  right: [0, -1, 0.18],
  top: [0, -0.002, 1],
  iso: [1, -1, 0.75],
};

interface Loaded {
  viewer: WorkstationRenderer;
  project: OfflineProgrammingProject;
  dense: DenseTrajectory | null;
  /** The world points the named views frame: part, path, plan and welds. */
  subject: THREE.Box3;
  width: number;
  height: number;
}

let loaded: Loaded | null = null;
const stage = document.getElementById("workstation-stage") as HTMLElement;

function emptyScene(modelId = ""): SnapshotScene {
  return { modelId, partTriangles: 0, fixtures: 0, pathPoints: 0, plannedPathPoints: 0, plannedStale: false, welds: 0, boundsMm: {} };
}

function message(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

function mmBounds(box: THREE.Box3): MmBounds | undefined {
  if (box.isEmpty()) return undefined;
  const round = (v: THREE.Vector3) => [v.x, v.y, v.z].map((value) => Math.round(value * 10_000) / 10);
  return [round(box.min), round(box.max)];
}

function boxOfPoints(points: readonly (readonly number[])[]): THREE.Box3 {
  const box = new THREE.Box3();
  for (const point of points) box.expandByPoint(new THREE.Vector3(point[0], point[1], point[2]));
  return box;
}

function base64Bytes(text: string): Uint8Array {
  const binary = atob(text);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) bytes[index] = binary.charCodeAt(index);
  return bytes;
}

async function load(input: SnapshotLoadInput): Promise<SnapshotLoadResult> {
  const errors: string[] = [];
  const warnings: string[] = [];
  stage.dataset.snapshotState = "loading";
  loaded?.viewer.dispose();
  loaded = null;
  // The renderer reports what it drew on the stage; a second load must not
  // inherit the first program's counts where it draws nothing.
  for (const key of Object.keys(stage.dataset)) if (key !== "snapshotState") delete stage.dataset[key];

  const width = Math.round(input.width ?? 1280);
  const height = Math.round(input.height ?? 800);
  stage.style.width = `${width}px`;
  stage.style.height = `${height}px`;

  // The catalogue first: placement reads the cell's work frame from it, so a
  // project parsed before it would hang its part off the robot's base.
  const catalogue = parseRobotCatalogue(input.catalogue);
  if (catalogue.length === 0) errors.push("the robot catalogue lists no robot (expected GET /api/offline-programming/v1/robots)");
  setRobotCatalogue(catalogue);

  let project: OfflineProgrammingProject;
  try {
    project = importProjectJSON(JSON.stringify(input.document));
  } catch (error) {
    errors.push(`OLP cannot import the program: ${message(error)}`);
    stage.dataset.snapshotState = "failed";
    return { ok: false, errors, warnings, scene: emptyScene() };
  }
  const modelId = project.robot.model;
  if (!knownRobotModel(modelId)) {
    warnings.push(`the catalogue has no description for ${modelId}: placement uses the robot's base as the work frame`);
  }

  // Meshes are the CAD service's wire; OLP's own normaliser reads them, keyed
  // by the fingerprint the mesh says it is (the renderer drops a mismatch).
  const meshes = new Map<string, CadWorkpieceMesh>();
  for (const [key, wire] of Object.entries(input.meshes ?? {})) {
    try {
      const mesh = normalizeWorkpieceMesh(wire);
      if (mesh.fingerprint !== key.toLowerCase()) {
        warnings.push(`mesh given under ${key} is for ${mesh.fingerprint}; used under its own fingerprint`);
      }
      meshes.set(mesh.fingerprint, mesh);
    } catch (error) {
      errors.push(`mesh ${key} is not a CadQuery tessellation: ${message(error)}`);
    }
  }

  const viewerEvents: string[] = [];
  const viewer = new WorkstationRenderer({
    elements: {
      stage,
      canvas: document.getElementById("workstation-canvas") as HTMLCanvasElement,
      orbitPane: document.getElementById("orbit-pane") as HTMLElement,
      worldAxisCanvas: document.getElementById("world-axis-indicator") as HTMLCanvasElement,
      j6Pane: document.getElementById("j6-pane") as HTMLElement,
      tcpPane: document.getElementById("tcp-pane") as HTMLElement,
    },
    initialModelId: modelId,
    // An absolute DIRECTORY: the default, document.baseURI, is this page's
    // file URL, and resolveURL would put the robot under snapshot.html/.
    baseURL: new URL("./", location.href).href,
    pixelRatioCap: 1,
    splitView: false,
    autoStart: false,
    onEvent: (event) => {
      if (event.type === "model-error") viewerEvents.push(`robot ${event.modelId} did not load: ${event.message}`);
      if (event.type === "pose-rejected") viewerEvents.push(`pose rejected: ${event.validation.issues.map((issue) => issue.message).join("; ")}`);
    },
  });
  // A snapshot is one frame: damping would leave the camera short of where
  // it was put until frames that never come.
  viewer.controls.enableDamping = false;
  // The constructor's own load first. Starting a second load of the same
  // model while it is in flight (as `syncProjectToWorkstation` would) lets
  // three's FileLoader merge the duplicate mesh requests without telling
  // the second LoadingManager, which then reports the robot loaded with
  // links still missing: the first shot drew no robot (measured 2026-10-29,
  // the first of five cold runs). Once loaded, the sync below reuses it.
  await viewer.ready;

  const fingerprint = project.cad.fingerprint.toLowerCase();
  const outcome = await syncProjectToWorkstation(viewer, project, {
    workpieceMesh: meshes.get(fingerprint) ?? null,
    fixtureMeshes: meshes,
    showLabels: input.showLabels ?? true,
  });
  errors.push(...viewerEvents);
  if (!outcome.ok) errors.push(`robot ${modelId} did not load: ${outcome.error ?? "unknown error"}`);
  // A robot missing a link is a robot drawn wrong, which is worse than none.
  for (const assetError of outcome.assetErrors) errors.push(`robot asset: ${assetError}`);
  // Every visual the URDF names must have arrived as a mesh: a link drawn as
  // nothing is a robot with a hole in it, not a smaller robot.
  const robot = viewer.scene.getObjectByName(`${modelId}-workstation`);
  robot?.traverse((node) => {
    if (node.type !== "URDFVisual") return;
    let meshes = 0;
    node.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) meshes += 1;
    });
    if (meshes === 0) errors.push(`robot visual ${node.name || node.parent?.name || "?"} has no mesh`);
  });
  if (/^[a-f0-9]{64}$/.test(fingerprint) && !meshes.has(fingerprint)) {
    warnings.push(`part ${project.cad.filename} is not drawn: no mesh for its fingerprint ${fingerprint}`);
  } else if (!/^[a-f0-9]{64}$/.test(fingerprint)) {
    warnings.push(`part ${project.cad.filename} has no STEP fingerprint (${project.cad.fingerprint}); its surface is not drawn`);
  }
  const fixtures = environmentFixtures(project);
  for (const fixture of fixtures) {
    if (fixture.shape.kind === "step" && !meshes.has(fixture.shape.fingerprint.toLowerCase())) {
      warnings.push(`fixture ${fixture.name} is not drawn: no mesh for its fingerprint ${fixture.shape.fingerprint}`);
    }
  }

  // Welds in the part's frame: every weld's line, lit as a selected weld is
  // (over the part, so a weld inside a corner is still seen), and the
  // hand-placed ones' torches, which carry the orientation.
  const weldNodes = project.nodes.filter((node) => node.op === "weld" && node.enabled);
  let welds = 0;
  const weldWorld: Vec3[] = [];
  viewer.waypointLayer.updateWorldMatrix(true, false);
  for (const node of weldNodes) {
    const points = weldPathPoints(node);
    if (!points || points.length < 2) continue;
    const highlight = new WeldHighlight();
    highlight.show(points, false);
    viewer.waypointLayer.add(highlight.root);
    welds += 1;
    for (const point of points) {
      const world = new THREE.Vector3(...point).applyMatrix4(viewer.waypointLayer.matrixWorld);
      weldWorld.push([world.x, world.y, world.z]);
    }
  }
  const manual = project.nodes.filter(isManualPoseWeld).filter((node) => node.enabled).map(waypointWeldOf);
  if (manual.length > 0) {
    const overlay = new WaypointOverlay();
    overlay.sync(manual, { weldId: null, waypointId: null });
    overlay.torchesVisible = true;
    viewer.waypointLayer.add(overlay.root);
  }

  // The planned path: the dense plan when its bytes are given, else the
  // classic plan's own TCP samples. Drawn even when it is not this
  // revision's plan (the app would hide it), and said: an agent asking what
  // the plan does needs to see it, and to know it is old.
  let dense: DenseTrajectory | null = null;
  let plannedStale = false;
  if (input.denseBase64) {
    try {
      dense = decodeDenseTrajectory(base64Bytes(input.denseBase64));
    } catch (error) {
      errors.push(`the dense plan does not decode: ${message(error)}`);
    }
  }
  if (dense) {
    const record = project.denseTrajectory;
    const stale: string[] = [];
    if (!record) stale.push("the program records no dense plan");
    else {
      if (record.trajectoryDigest !== dense.trajectoryDigest) stale.push(`its digest ${dense.trajectoryDigest} is not the program's ${record.trajectoryDigest}`);
      if (record.sourceProjectRevision !== project.identity.revision) stale.push(`it was planned at revision ${record.sourceProjectRevision}, the program is at ${project.identity.revision}`);
    }
    if (dense.robot && dense.robot.modelId !== modelId) stale.push(`it is for ${dense.robot.modelId}, not ${modelId}`);
    if (dense.robot && record?.robot && dense.robot.robotDescriptionSha256 !== project.robot.description?.robotDescriptionSha256) {
      stale.push("its robot description is not the one the program is pinned to");
    }
    plannedStale = stale.length > 0;
    if (plannedStale) warnings.push(`planned path drawn but stale: ${stale.join("; ")}`);
    if (outcome.ok) {
      try {
        viewer.setPlannedPath(await viewer.tracePlannedPath(densePlannedPathRuns(project, dense), () => true));
      } catch (error) {
        errors.push(`the dense plan cannot be traced on ${modelId}: ${message(error)}`);
      }
    }
  } else if (project.plan.trajectory.length > 0) {
    plannedStale = !planIsFresh(project);
    if (plannedStale) warnings.push(`planned path drawn but stale: plan ${project.plan.state} from revision ${project.plan.sourceProjectRevision}, program at ${project.identity.revision}`);
    try {
      viewer.setPlannedPath([projectPlannedPath(project)]);
    } catch (error) {
      errors.push(`the classic plan cannot be drawn: ${message(error)}`);
    }
  } else if (project.denseTrajectory && !input.denseBase64) {
    warnings.push(`the program has a dense plan (${project.denseTrajectory.trajectoryDigest}) but its bytes were not given: planned path not drawn`);
  }

  // What the named views frame: part, authored path, plan and welds, in
  // the world. With none of them, the robot.
  viewer.scene.updateMatrixWorld(true);
  const partLayer = viewer.scene.getObjectByName("workstation-workpiece");
  const partBox = partLayer ? new THREE.Box3().setFromObject(partLayer) : new THREE.Box3();
  const authored = safePath(project, errors);
  const plannedPoints = Number(stage.dataset.plannedPathPointCount ?? 0) > 0 ? plannedPathPoints(viewer) : [];
  const pathBox = boxOfPoints([...authored.map((p) => p.positionM), ...plannedPoints, ...weldWorld]);
  const subject = partBox.clone().union(pathBox);
  if (subject.isEmpty()) {
    const robot = viewer.scene.getObjectByName(`${modelId}-workstation`);
    if (robot) subject.setFromObject(robot);
  }

  loaded = { viewer, project, dense, subject, width, height };
  viewer.resetOrbitCamera();
  viewer.renderNow();
  const ok = errors.length === 0;
  stage.dataset.snapshotState = ok ? "ready" : "failed";
  return {
    ok,
    errors,
    warnings,
    scene: {
      modelId,
      partTriangles: Number(stage.dataset.workpieceTriangleCount || 0),
      fixtures: fixtures.length,
      pathPoints: Number(stage.dataset.pathPointCount ?? 0),
      plannedPathPoints: Number(stage.dataset.plannedPathPointCount ?? 0),
      plannedStale,
      welds,
      boundsMm: { part: mmBounds(partBox), path: mmBounds(pathBox) },
    },
  };
}

function safePath(project: OfflineProgrammingProject, errors: string[]): MotionPathPoint[] {
  try {
    return projectPath(project);
  } catch (error) {
    errors.push(`the program's path cannot be resolved: ${message(error)}`);
    return [];
  }
}

/** The planned path as drawn, read back from its layer (world frame). */
function plannedPathPoints(viewer: WorkstationRenderer): Vec3[] {
  const layer = viewer.scene.getObjectByName("workstation-planned-tcp-path");
  const box = layer ? new THREE.Box3().setFromObject(layer) : new THREE.Box3();
  return box.isEmpty() ? [] : [[box.min.x, box.min.y, box.min.z], [box.max.x, box.max.y, box.max.z]];
}

function requireLoaded(): Loaded {
  if (!loaded) throw new Error("olpSnapshot.shot: load() a program first");
  return loaded;
}

/** Every planned sample on one timeline, in segment order. */
function plannedSamples(state: Loaded): SimulationSample[] {
  if (state.dense) return densePlannedPathRuns(state.project, state.dense).flat();
  return [];
}

function applyPose(state: Loaded, pose: SnapshotShotSpec["pose"], notes: string[]): void {
  const { viewer, project } = state;
  if (pose === undefined) return;
  const descriptor = robotModelFor(project.robot.model);
  if (Array.isArray(pose)) {
    const order = descriptor.poseJointOrder;
    if (pose.length !== order.length && pose.length !== descriptor.urdfJointNames.length) {
      throw new Error(`pose has ${pose.length} joints; ${project.robot.model} takes ${order.join(",")}`);
    }
    const jointsRad = order.map((_, index) => pose[index] ?? 0);
    const validation = viewer.setPose({ modelId: project.robot.model, jointOrder: [...order], jointsRad, source: "offline" }, false);
    if (!validation.ok) throw new Error(`pose rejected: ${validation.issues.map((issue) => issue.message).join("; ")}`);
    return;
  }
  if ("sample" in pose) {
    const samples = plannedSamples(state);
    const classic = project.plan.trajectory;
    const total = samples.at(-1)?.timeS ?? classic.at(-1)?.timeS ?? 0;
    const fraction = pose.sample === "start" ? 0 : pose.sample === "end" ? 1 : pose.sample;
    if (!(fraction >= 0 && fraction <= 1)) throw new Error(`sample ${String(pose.sample)} is not start, end or a fraction 0..1`);
    const at = fraction * total;
    if (samples.length > 0) {
      applyDenseSample(viewer, nearest(samples, at, (sample) => sample.timeS));
    } else if (classic.length > 0) {
      applyProjectSample(viewer, project, nearest(classic, at, (sample) => sample.timeS));
    } else {
      throw new Error("the program has no planned samples to pose the robot from (give the dense plan, or plan it)");
    }
    return;
  }
  const joints = rowJoints(state, pose.row);
  if (!joints) throw new Error(`no joint solution is known for ${pose.row}: it is not a taught move, and no plan sample belongs to it`);
  notes.push(joints.note);
  const validation = viewer.setPose({ modelId: project.robot.model, jointOrder: [...descriptor.poseJointOrder], jointsRad: joints.jointsRad, source: "offline" }, false);
  if (!validation.ok) throw new Error(`pose rejected: ${validation.issues.map((issue) => issue.message).join("; ")}`);
}

function nearest<T>(items: readonly T[], at: number, time: (item: T) => number): T {
  let best = items[0] as T;
  for (const item of items) if (Math.abs(time(item) - at) < Math.abs(time(best) - at)) best = item;
  return best;
}

/** Joints for a program row or node: a taught move's recorded joints, else
 *  the first planned sample that belongs to it. Never IK here -- a solution
 *  this page invented would be drawn as if the planner had chosen it. */
function rowJoints(state: Loaded, id: string): { jointsRad: number[]; note: string } | null {
  const { project } = state;
  const order = robotModelFor(project.robot.model).poseJointOrder;
  const rowIndex = project.rows.findIndex((row) => row.id === id);
  const nodeId = rowIndex >= 0 ? rowNodeId(project.rows[rowIndex]!) ?? id : id;
  const node = project.nodes.find((candidate) => candidate.id === nodeId);
  if (node?.op === "move") {
    const jointsRad = order.map((name) => node.target.joint_values_rad[node.target.joint_names.indexOf(name)] ?? 0);
    return { jointsRad, note: `${id}: the taught joints of move ${node.label}` };
  }
  const dense = state.dense;
  if (dense) {
    const runs = densePlannedPathRuns(project, dense);
    const index = dense.segments.findIndex((segment) => segment.sourceId === nodeId || segment.sourceId === id);
    const sample = index >= 0 ? runs[index]?.[0] : undefined;
    if (sample) return { jointsRad: [...sample.jointsRad], note: `${id}: first sample of planned segment ${index} (${dense.segments[index]!.kind})` };
  }
  if (rowIndex >= 0) {
    const sample = project.plan.trajectory.find((candidate: PlanTrajectorySample) => candidate.rowIndex === rowIndex);
    if (sample) return { jointsRad: order.map((_, i) => sample.jointPositionsRad[i] ?? 0), note: `${id}: the classic plan's sample at t=${sample.timeS.toFixed(3)} s` };
  }
  return null;
}

function applyCamera(state: Loaded, camera: SnapshotShotSpec["camera"], notes: string[]): void {
  const { viewer, project, subject } = state;
  const orbit = viewer.orbitCamera;
  const fovDeg = typeof camera === "object" ? camera.fovDeg ?? ORBIT_FOV_DEG : ORBIT_FOV_DEG;
  orbit.fov = fovDeg;
  orbit.updateProjectionMatrix();
  if (typeof camera === "object") {
    const [px, py, pz] = camera.positionM;
    const [tx, ty, tz] = camera.targetM;
    if (![px, py, pz, tx, ty, tz].every((value) => Number.isFinite(value))) throw new Error("camera positionM and targetM must be three finite numbers each");
    orbit.position.set(px!, py!, pz!);
    viewer.controls.target.set(tx!, ty!, tz!);
    viewer.controls.update();
    return;
  }
  switch (camera) {
    case "home":
      viewer.resetOrbitCamera();
      return;
    case "part": {
      viewer.resetOrbitCamera();
      const part = viewer.scene.getObjectByName("workstation-workpiece");
      if (!part || new THREE.Box3().setFromObject(part).isEmpty()) notes.push("part: no part surface is drawn; showing home");
      viewer.framePart();
      return;
    }
    case "path":
      viewer.resetOrbitCamera();
      viewer.framePath(projectPath(project));
      return;
    default: {
      const direction = VIEW_DIRECTIONS[camera];
      if (!direction) throw new Error(`unknown camera ${String(camera)}`);
      frameBox(state, subject, new THREE.Vector3(...direction).normalize());
    }
  }
}

/** Put the camera on `direction` from the box's centre, far enough that the
 *  box's bounding sphere fits the narrower field of view with a margin. */
function frameBox(state: Loaded, box: THREE.Box3, direction: THREE.Vector3): void {
  const { viewer, width, height } = state;
  const center = box.getCenter(new THREE.Vector3());
  const radius = Math.max(MIN_FRAME_RADIUS_M, box.getSize(new THREE.Vector3()).length() / 2) * FRAME_MARGIN;
  const vertical = THREE.MathUtils.degToRad(viewer.orbitCamera.fov);
  const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * (width / height));
  const distance = radius / Math.sin(Math.min(vertical, horizontal) / 2);
  viewer.controls.target.copy(center);
  viewer.orbitCamera.position.copy(center).addScaledVector(direction, distance);
  viewer.controls.update();
}

function shot(spec: SnapshotShotSpec): SnapshotShotResult {
  const state = requireLoaded();
  const { viewer } = state;
  const notes: string[] = [];
  applyPose(state, spec.pose, notes);
  viewer.setTcpFrameEnabled(spec.tcpFrame ?? true);
  viewer.setPlannedPathVisible(spec.plannedPath ?? true);
  viewer.setPartFocus(spec.partFocus ?? false);
  applyCamera(state, spec.camera, notes);

  // Read back in the same task as the draw: the context does not preserve
  // its drawing buffer, so a later read sees a cleared canvas.
  viewer.renderNow();
  const output = document.createElement("canvas");
  output.width = state.width;
  output.height = state.height;
  const context = output.getContext("2d");
  if (!context) throw new Error("no 2D context to compose the snapshot");
  context.drawImage(viewer.elements.canvas, 0, 0, state.width, state.height);
  const axis = viewer.elements.worldAxisCanvas;
  const stageRect = stage.getBoundingClientRect();
  const axisRect = axis.getBoundingClientRect();
  context.drawImage(axis, axisRect.left - stageRect.left, axisRect.top - stageRect.top, axisRect.width, axisRect.height);
  const pose = viewer.currentPoseSnapshot();
  return {
    dataUrl: output.toDataURL("image/png"),
    pose: { jointOrder: [...pose.jointOrder], jointsRad: [...pose.jointsRad] },
    camera: {
      positionM: viewer.orbitCamera.position.toArray(),
      targetM: viewer.controls.target.toArray(),
      fovDeg: viewer.orbitCamera.fov,
    },
    notes,
  };
}

declare global {
  interface Window {
    olpSnapshot?: { load: typeof load; shot: typeof shot };
  }
}

window.olpSnapshot = { load, shot };
stage.dataset.snapshotState = "idle";
