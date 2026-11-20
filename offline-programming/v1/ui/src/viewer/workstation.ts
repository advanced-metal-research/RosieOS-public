import type {
  AuthoredProgramRow,
  OfflineProgrammingProject,
  PlanTrajectorySample,
  TcpPose,
  Vec3,
} from "../domain/project.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import type { CadWorkpieceMesh } from "../cad/contracts.ts";
import { environmentFixtures } from "../domain/workpieces.ts";
import {
  plannerPoseToWorld,
  worldPointToPlanner,
  transformWorkpiecePoint,
  workpieceAnchorWorldM,
  workpieceWorldRotationRpy,
} from "../domain/placement.ts";
import {
  rowTorchSettings,
  rowWeaveSettings,
  torchGuideVectors,
  weavePreviewPoints,
} from "../domain/weld.ts";
import { rowNodeId } from "../program-v2/rows-from-nodes.ts";
import { buildProgramView } from "../domain/points-moves.ts";
import {
  isSelectableTopologyEdge,
  projectEdgeCatalog,
} from "../domain/edge-catalog.ts";
import type { EdgeOverlayEdge } from "./edge-picking.ts";
import type { MotionPathPoint } from "./geometry.ts";
import type {
  CollisionMarker,
  CurveCenterMarker,
  PathLabel,
  SelectionOverlayInput,
  TcpFrameMarker,
} from "./overlays.ts";
import {
  type ModelLoadOutcome,
  WorkstationRenderer,
} from "./workstation-renderer.ts";
import type { WorkpieceMeshPlacement } from "./workpiece.ts";

/**
 * Domain adapter for the full one-context workstation. It deliberately owns no
 * Three.js renderer, scene, animation loop, or resource lifecycle.
 */
export interface AuthoringOverlayOptions {
  showLabels?: boolean;
  /**
   * Tessellated surface for this project's CAD fingerprint, or null when there
   * is none (browser-fallback example, CAD service unavailable, tessellation
   * failed). Held by the caller, never by the project.
   */
  workpieceMesh?: CadWorkpieceMesh | null;
  showWorkpieceEdges?: boolean;
  /** Tessellated surfaces of the STEP fixtures, by lowercase fingerprint;
   *  held by the caller like the workpiece mesh. */
  fixtureMeshes?: ReadonlyMap<string, CadWorkpieceMesh>;
}

export async function syncProjectToWorkstation(
  renderer: WorkstationRenderer,
  project: OfflineProgrammingProject,
  options: AuthoringOverlayOptions = {},
): Promise<ModelLoadOutcome> {
  const outcome = await renderer.loadModel(project.robot.model);
  if (!outcome.ok) return outcome;
  // Authoring refreshes geometry, not pose ownership or live feedback state.
  // The spreadsheet is the program. Do not leave a CAD seam or a stale plan
  // overlay in the viewport when the loaded/edited rows say something else.
  renderer.setSeams([]);
  syncWorkpieceMesh(renderer, project, options);
  renderer.setWorkcellFixtures(environmentFixtures(project), options.fixtureMeshes ?? new Map());
  syncAuthoringOverlays(renderer, project, options);
  renderer.setCollisionMarkers(projectCollisionMarkers(project));
  return outcome;
}

/**
 * Install or clear the workpiece mesh and put it where the fixture placement
 * says it is.
 *
 * A mesh belongs to a project only if its fingerprint matches the project's CAD
 * fingerprint — otherwise a stale mesh from a previously imported part would sit
 * under the new part's weld paths.
 */
export function syncWorkpieceMesh(
  renderer: WorkstationRenderer,
  project: OfflineProgrammingProject,
  options: AuthoringOverlayOptions = {},
): void {
  const mesh = workpieceMeshForProject(project, options.workpieceMesh ?? null);
  renderer.setWorkpiece(mesh, projectWorkpiecePlacement(project), {
    showEdges: options.showWorkpieceEdges === true,
  });
}

/** The mesh only applies to the part currently loaded in the project. */
export function workpieceMeshForProject(
  project: OfflineProgrammingProject,
  mesh: CadWorkpieceMesh | null,
): CadWorkpieceMesh | null {
  if (!mesh) return null;
  return mesh.fingerprint === project.cad.fingerprint.toLowerCase() ? mesh : null;
}

/**
 * The rigid CAD-local → world transform for the workpiece, taken from the same
 * placement functions that position the seam geometry. Reusing them is what
 * keeps the mesh and the weld paths from ever disagreeing.
 */
export function projectWorkpiecePlacement(
  project: OfflineProgrammingProject,
): WorkpieceMeshPlacement | null {
  if (project.fixtures.length === 0) return null;
  return {
    originM: transformWorkpiecePoint(project, [0, 0, 0]),
    rotationRpyRad: workpieceWorldRotationRpy(project),
    anchorM: workpieceAnchorWorldM(project) ?? undefined,
  };
}

/**
 * World-space points describing the selected move (its target plus the
 * incoming segment start) for camera auto-centering. Null when the selection
 * is not a visible motion target.
 */
export function selectionFocusPointsM(
  project: OfflineProgrammingProject,
): Vec3[] | null {
  const selection = projectSelectionOverlay(project);
  if (!selection) return null;
  const points: Vec3[] = [[...selection.targetM] as Vec3];
  if (selection.previousM) points.push([...selection.previousM] as Vec3);
  return points;
}

/**
 * Path, decoration, selection, and TCP-frame overlays. Selecting a row turns
 * this into a focus mode: the base path and its decorations fade so the
 * selected move, its run, and its guides carry the visual weight.
 */
export function syncAuthoringOverlays(
  renderer: WorkstationRenderer,
  project: OfflineProgrammingProject,
  options: AuthoringOverlayOptions = {},
): void {
  const path = projectPath(project);
  const selection = projectSelectionOverlay(project);
  const dim = selection ? 0.3 : 1;
  renderer.setPath(path, undefined, dim);
  renderer.setPathDecorations(
    path,
    options.showLabels === false ? [] : projectRowLabels(project),
    undefined,
    selection ? 0.25 : 1,
  );
  renderer.setSelectionOverlay(selection);
  renderer.setTcpFrames(projectTcpFrames(project));
  renderer.setCurveCenters(projectCurveCenters(project));
}

/**
 * World-space pickable edges from the persisted catalog, placed through the
 * same fixture transform as the seam geometry so edges, part surface, and
 * weld paths never disagree.
 */
export function projectCadEdgeOverlays(
  project: OfflineProgrammingProject,
): EdgeOverlayEdge[] {
  return projectEdgeCatalog(project)
    .filter(isSelectableTopologyEdge)
    .map((edge) => ({
      id: edge.id,
      pointsM: edge.samplesM.map((point) => transformWorkpiecePoint(project, point)),
      closed: edge.closed,
    }));
}

export function applyProjectSample(
  renderer: WorkstationRenderer,
  project: OfflineProgrammingProject,
  sample: PlanTrajectorySample | null,
): void {
  if (!sample) {
    renderer.setCurrentTcpPose(null);
    return;
  }
  const simulation = toSimulationSample(project, sample);
  renderer.setPose(simulation, false);
  renderer.setCurrentTcpPose(
    simulation.tcpPose ? plannerPoseToWorld(project, simulation.tcpPose) : null,
  );
}

/** Show one dense-trajectory sample.
 *
 *  The rails counterpart of `applyProjectSample`, and separate for the same
 *  reason the samples are: a dense sample is joints, rates and a torch flag,
 *  with no pose to place a marker at. The robot is posed from the joints,
 *  which is what the renderer does with a project sample too -- what is
 *  missing here is only the TCP frame, and it is missing because nothing
 *  computed a second answer for where the tool already is. */
export function applyDenseSample(
  renderer: WorkstationRenderer,
  sample: SimulationSample | null,
): void {
  if (!sample) {
    renderer.setCurrentTcpPose(null);
    return;
  }
  renderer.setPose(sample, false);
  renderer.setCurrentTcpPose(null);
}

export function toSimulationSample(
  project: OfflineProgrammingProject,
  sample: PlanTrajectorySample,
): SimulationSample {
  return {
    modelId: project.robot.model,
    jointOrder: [...project.robot.jointOrder],
    jointsRad: [...sample.jointPositionsRad],
    source: project.plan.planner === "browser_preview" ? "preview" : "playback",
    timeS: sample.timeS,
    tcpPose: [...sample.tcpPose],
    motion: sample.weldOn ? "weld" : "travel",
    seamId: sample.seamId,
    rowIndex: sample.rowIndex,
  };
}

/**
 * Orbit framing points: the program path when it exists, otherwise the CAD
 * topology edges so Frame program still zooms to the imported part.
 */
export function projectFramePoints(
  project: OfflineProgrammingProject,
): MotionPathPoint[] {
  const path = projectPath(project);
  if (path.length > 0) return path;
  return projectCadEdgeOverlays(project).flatMap((edge) =>
    edge.pointsM.map((positionM) => ({
      positionM,
      motion: "travel" as const,
    })),
  );
}

export function projectPath(project: OfflineProgrammingProject): MotionPathPoint[] {
  // The path line keeps every wire waypoint so CURVE geometry remains exact;
  // labels, markers, selection, and frames use the compact CP/move view.
  return spreadsheetMotionTargets(project, false).map(({ pose, weldOn }) => ({
    positionM: plannerPoseToWorld(project, pose).slice(0, 3) as [number, number, number],
    motion: weldOn ? "weld" : "travel",
  }));
}

export function projectCollisionMarkers(
  project: OfflineProgrammingProject,
): CollisionMarker[] {
  return project.plan.evidence.collisionPoints.flatMap((collision, index) => {
    const sample = project.plan.trajectory[collision.waypointIndex];
    if (!sample) return [];
    return [{
      id: `collision-${index}-${collision.waypointIndex}`,
      positionM: plannerPoseToWorld(project, sample.tcpPose).slice(0, 3) as [number, number, number],
      severity: "collision" as const,
      linkA: collision.linkA,
      linkB: collision.linkB,
    }];
  });
}

export function projectTcpFrames(project: OfflineProgrammingProject): TcpFrameMarker[] {
  const targets = spreadsheetMotionTargets(project);
  if (targets.length === 0) return [];
  // Focus mode: a selected row keeps its own frame only; otherwise a sparse
  // stride avoids the axis-triad thicket the frames used to draw.
  const selected = targets.find(
    (target) => target.rowId === project.simulation.selectedRowId,
  );
  if (selected) {
    return [{
      id: `tcp-row-${selected.rowId}`,
      pose: plannerPoseToWorld(project, selected.pose),
      sizeM: 0.055,
    }];
  }
  const stride = Math.max(1, Math.ceil(targets.length / 10));
  return targets.flatMap((target, index) => {
    const terminal = index === targets.length - 1;
    if (index % stride !== 0 && !terminal) return [];
    return [{
      id: `tcp-row-${target.rowId}`,
      pose: plannerPoseToWorld(project, target.pose),
      sizeM: 0.04,
    }];
  });
}

export function projectCurveCenters(project: OfflineProgrammingProject): CurveCenterMarker[] {
  const selected = project.simulation.selectedRowId;
  return buildProgramView(project).moves.flatMap((move) =>
    move.kind === "CURVE" && move.centerM && !move.modifiedExternally
      ? [{
        id: move.id,
        positionM: plannerPoseToWorld(project, [
          move.centerM[0],
          move.centerM[1],
          move.centerM[2],
          0,
          0,
          0,
        ]).slice(0, 3) as Vec3,
        selected: move.end.row.id === selected,
      }]
      : []
  );
}

/**
 * Spreadsheet row numbers on the drawn path, kept deliberately sparse so the
 * stage stays readable:
 * - with a selected row: only that row, its immediate neighbours, and the
 *   boundaries of its run are labelled;
 * - with no selection: weld run boundaries and the program endpoints only
 *   (every target for short programs).
 */
export function projectRowLabels(project: OfflineProgrammingProject): PathLabel[] {
  const targets = spreadsheetMotionTargets(project);
  if (targets.length === 0) return [];
  const selectedIndex = targets.findIndex(
    (target) => target.rowId === project.simulation.selectedRowId,
  );

  const wanted = new Set<number>();
  if (selectedIndex >= 0) {
    wanted.add(selectedIndex);
    if (selectedIndex > 0) wanted.add(selectedIndex - 1);
    if (selectedIndex < targets.length - 1) wanted.add(selectedIndex + 1);
    const selected = targets[selectedIndex]!;
    if (selected.weldOn) {
      const inRun = (candidate: SpreadsheetMotionTarget): boolean =>
        candidate.weldOn && candidate.seamId === selected.seamId;
      let start = selectedIndex;
      while (start > 0 && inRun(targets[start - 1]!)) start -= 1;
      let end = selectedIndex;
      while (end < targets.length - 1 && inRun(targets[end + 1]!)) end += 1;
      wanted.add(start);
      wanted.add(end);
    }
  } else if (targets.length <= 12) {
    targets.forEach((_, index) => wanted.add(index));
  } else {
    wanted.add(0);
    wanted.add(targets.length - 1);
    for (let index = 0; index < targets.length; index += 1) {
      const previous = targets[index - 1];
      const next = targets[index + 1];
      const weldBoundary = targets[index]!.weldOn &&
        (targets[index]!.weldOn !== (previous?.weldOn ?? false) ||
          targets[index]!.weldOn !== (next?.weldOn ?? false));
      if (weldBoundary) wanted.add(index);
    }
  }

  return [...wanted].sort((left, right) => left - right).map((index) => {
    const target = targets[index]!;
    return {
      id: target.rowId,
      text: target.label ?? String(target.rowIndex + 1).padStart(2, "0"),
      positionM: worldPoint(project, target.pose),
      motion: target.weldOn ? "weld" as const : "travel" as const,
      emphasis: index === selectedIndex,
    };
  });
}

/**
 * Selected-row emphasis for the 3D stage: target marker, incoming segment,
 * torch-angle guide on arc-on rows, and a weave ribbon along the weld run.
 */
export function projectSelectionOverlay(
  project: OfflineProgrammingProject,
): SelectionOverlayInput | null {
  const selectedRowId = project.simulation.selectedRowId;
  if (!selectedRowId) return null;
  const targets = spreadsheetMotionTargets(project);
  const index = targets.findIndex((target) => target.rowId === selectedRowId);
  if (index < 0) return null;
  const target = targets[index]!;
  const row = project.rows.find((candidate) => candidate.id === selectedRowId)!;
  const targetM = worldPoint(project, target.pose);
  const previousM = index > 0 ? worldPoint(project, targets[index - 1]!.pose) : null;
  const nextM = index < targets.length - 1
    ? worldPoint(project, targets[index + 1]!.pose)
    : null;

  let torchGuide: SelectionOverlayInput["torchGuide"] = null;
  let weavePointsM: Vec3[] = [];
  let runPointsM: Vec3[] = [];
  if (target.weldOn) {
    const travelDir = direction(targetM, nextM && targets[index + 1]!.weldOn ? nextM : null) ??
      direction(previousM, targetM);
    if (travelDir) {
      const vectors = torchGuideVectors(travelDir, rowTorchSettings(row));
      if (vectors) {
        torchGuide = {
          originM: targetM,
          travelDir: vectors.travel,
          sideDir: vectors.side,
          torchUpDir: vectors.torchUp,
        };
      }
    }
    runPointsM = weldRunWorldPoints(project, targets, index);
    weavePointsM = weavePreviewPoints({
      pathPointsM: runPointsM,
      travelSpeedMps: row.tcpVelocityMps,
      weave: rowWeaveSettings(row),
    });
  }
  return {
    id: target.rowId,
    targetM,
    previousM,
    motion: target.weldOn ? "weld" : "travel",
    torchGuide,
    weavePointsM,
    runPointsM,
  };
}

/** World points of the contiguous weld run containing the given target. */
function weldRunWorldPoints(
  project: OfflineProgrammingProject,
  targets: readonly SpreadsheetMotionTarget[],
  index: number,
): Vec3[] {
  const seamId = targets[index]!.seamId;
  const inRun = (candidate: SpreadsheetMotionTarget): boolean =>
    candidate.weldOn && candidate.seamId === seamId;
  let start = index;
  while (start > 0 && inRun(targets[start - 1]!)) start -= 1;
  let end = index;
  while (end < targets.length - 1 && inRun(targets[end + 1]!)) end += 1;
  return targets.slice(start, end + 1)
    .map((target) => worldPoint(project, target.pose));
}

function worldPoint(project: OfflineProgrammingProject, pose: TcpPose): Vec3 {
  return plannerPoseToWorld(project, pose).slice(0, 3) as Vec3;
}

function direction(from: Vec3 | null, to: Vec3 | null): Vec3 | null {
  if (!from || !to) return null;
  const delta: Vec3 = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
  return Math.hypot(...delta) > 1e-9 ? delta : null;
}

interface SpreadsheetMotionTarget {
  rowId: string;
  label?: string;
  rowIndex: number;
  row: AuthoredProgramRow;
  seamId: string | null;
  pose: TcpPose;
  weldOn: boolean;
}

/**
 * Resolve the visible program in spreadsheet order. Cartesian rows use their
 * authored XYZ/RPY directly. Joint rows use the canonical planner's FK result
 * for that row when available; inventing a Cartesian point for an unplanned
 * joint target would be misleading.
 */
function spreadsheetMotionTargets(
  project: OfflineProgrammingProject,
  visibleOnly = true,
): SpreadsheetMotionTarget[] {
  const plannedTcpByRow = new Map<number, TcpPose>();
  for (const sample of project.plan.trajectory) {
    plannedTcpByRow.set(sample.rowIndex, [...sample.tcpPose]);
  }

  const nodes = new Map(project.nodes.map(node => [node.id, node]));
  const visibleRowIds = new Set(buildProgramView(project).visibleRowIds);
  return project.rows.flatMap((row, rowIndex) => {
    if (!row.enabled || (row.rowType ?? "motion") !== "motion") return [];
    const node = nodes.get(rowNodeId(row) ?? "");
    // Taught targets already contain the captured world TCP, including MoveJ.
    // Use the authored pose even before planning; an old plan must not move it.
    if (node?.op === "move") {
      if (!node.enabled) return [];
      const points = node.motion === "circular" && node.via
        ? [{ target: node.via, via: true }, { target: node.target, via: false }]
        : [{ target: node.target, via: false }];
      return points.flatMap(({ target, via }) => {
        const pose: TcpPose = [...worldPointToPlanner(project, target.xyz_m), ...target.rpy_rad];
        if (!pose.every(Number.isFinite)) return [];
        return [{ rowId: via ? `${row.id}:via` : row.id,
          label: `${node.label}${via ? " via" : ""}`, rowIndex, row,
          seamId: null, pose, weldOn: false }];
      });
    }
    if (visibleOnly && !visibleRowIds.has(row.id)) return [];
    const jointTarget = row.moveType === "JOINT" ||
      row.targetFrameType === "joint_positions";
    const pose = jointTarget
      ? plannedTcpByRow.get(rowIndex)
      : row.targetTcpPose;
    if (!pose || pose.some((value) => !Number.isFinite(value))) return [];
    return [{
      rowId: row.id,
      rowIndex,
      row,
      seamId: row.seamId,
      pose: [...pose] as TcpPose,
      weldOn: row.weldEnabled,
    }];
  });
}
