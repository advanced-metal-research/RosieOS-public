/**
 * Between the project and its program.v2 document.
 *
 * Two directions. `syncClassicNodes` builds weld nodes for the classic flow
 * out of what the project already authors -- a seam, its sequence step, and
 * the rows it compiled to -- so a program saved before nodes existed opens
 * with nodes and a new weld gets one the moment it is created. The rows are
 * READ here, never written: a row's TCP pose is the classic flow's baked
 * orientation, teach edits included, and that is what the node's points
 * are. `toProgramV2Document` assembles the wire document a planner would
 * receive, which is also what the validators judge.
 *
 * DOM-free; `from-project.test.ts` migrates the bundled dog box.
 */

import { railsJointOf } from "./rails-node.ts";
import {
  ROBOT_V4_PROGRAM_V2_SCHEMA,
  type PosedPoint,
  type ProgramV2Node,
  type RobotV4ProgramV2,
  type WeldNode,
  programV2Problems,
} from "../contracts/robot-v4-program-v2.ts";
import { deliberatelyRowlessSeamIDs, moveToleranceM, rowsForSeam } from "../adapters/cadquery.ts";
import { moveMetadata } from "../domain/points-moves.ts";
import {
  plannerPointToWorld,
  workpieceAnchorM,
  workpiecePlacement,
} from "../domain/placement.ts";
import {
  DEFAULT_TORCH_STANDOFF_MM,
  JOINT_ORDER,
  PLACEHOLDER_ROBOT_DESCRIPTION_SHA256,
  PLACEHOLDER_SHA256,
  type AuthoredSeam,
  type OfflineProgrammingProject,
  type SequenceStep,
  type Vec3,
  type WeldProcessParameters,
  projectId,
} from "../domain/project.ts";
import { processTorchDefaults } from "../domain/weld.ts";
import { insertWeld, isWeld, removeNode } from "./nodes.ts";

/** The provenance a classic weld node carries under `extensions.classic`. */
export interface ClassicProvenance {
  seam_id: string;
  process_id: string;
  /** True while the points are what the seam geometry + placement give;
   *  false once the operator edited a point, after which a placement or
   *  trim change leaves the points alone. */
  cad_derived: boolean;
  /** For a curved CAD edge, the via handle of each segment between two
   *  control points (world frame; null for a straight segment), so the
   *  compiled rows keep their CURVE moves. Dropped when a point is edited. */
  curve_centers_m?: (Vec3 | null)[];
}

export const CLASSIC_EXTENSION = "classic" as const;
export const DEFAULT_STANDOFF_MM = 50;

export function classicProvenance(node: ProgramV2Node): ClassicProvenance | null {
  const value = node.extensions?.[CLASSIC_EXTENSION];
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const record = value as Record<string, unknown>;
  if (typeof record.seam_id !== "string") return null;
  const centers = Array.isArray(record.curve_centers_m)
    ? record.curve_centers_m.map((center) =>
      Array.isArray(center) && center.length === 3 ? [Number(center[0]), Number(center[1]), Number(center[2])] as Vec3 : null)
    : undefined;
  return {
    seam_id: record.seam_id,
    process_id: typeof record.process_id === "string" ? record.process_id : "",
    cad_derived: record.cad_derived !== false,
    ...(centers ? { curve_centers_m: centers } : {}),
  };
}

export function setClassicProvenance(node: ProgramV2Node, provenance: ClassicProvenance): void {
  node.extensions = { ...(node.extensions ?? {}), [CLASSIC_EXTENSION]: { ...provenance } };
}

/** The classic weld's poses, world frame, computed the way `rowsForSeam`
 *  does: trim, reverse, chord-decimate, place, orient. */
export function classicWeldPoses(
  project: OfflineProgrammingProject,
  seam: AuthoredSeam,
  step: SequenceStep,
): PosedPoint[] {
  return classicWeldGeometry(project, seam, step).points;
}

/** The poses plus the via handles of curved segments, both world frame. */
export function classicWeldGeometry(
  project: OfflineProgrammingProject,
  seam: AuthoredSeam,
  step: SequenceStep,
): { points: PosedPoint[]; curveCenters: (Vec3 | null)[] } {
  const rows = rowsForSeam(project, seam, step, moveToleranceM(project), null).rows;
  const weldRows = rows.filter((row) => row.phase === "weld");
  const points = weldRows.map((row) => posedPointFromPlannerPose(project, row.targetTcpPose));
  const curveCenters = weldRows.slice(1).map((row) => {
    const center = moveMetadata(row)?.centerM;
    return center ? plannerPointToWorld(project, center) : null;
  });
  return { points, curveCenters };
}

function posedPointFromPlannerPose(
  project: OfflineProgrammingProject,
  pose: readonly number[],
): PosedPoint {
  const xyz = plannerPointToWorld(project, [pose[0]!, pose[1]!, pose[2]!]);
  return { xyz_m: xyz, rpy_rad: [pose[3]!, pose[4]!, pose[5]!] };
}

function processOf(project: OfflineProgrammingProject, seam: AuthoredSeam): WeldProcessParameters {
  return project.processParameters.find((candidate) => candidate.id === seam.processParameterId)
    ?? project.processParameters[0]!;
}

/** A weld node for a classic seam. */
export function classicWeldNode(
  project: OfflineProgrammingProject,
  seam: AuthoredSeam,
  points: PosedPoint[],
  cadDerived: boolean,
  curveCenters: (Vec3 | null)[] = [],
): WeldNode {
  const process = processOf(project, seam);
  const torch = processTorchDefaults(process);
  const id = projectId();
  const node: WeldNode = {
    id,
    op: "weld",
    enabled: seam.enabled,
    label: seam.label,
    comment: "",
    row_hash: `pending:${id}`,
    geometry: {
      kind: "posed_polyline",
      frame_id: "world",
      closed: false,
      points,
      torch_angles_deg: { work: torch.workAngleDeg, travel: torch.travelAngleDeg, spin: torch.spinAngleDeg },
    },
    travel_speed_mm_s: { kind: "constant", value: process.travelSpeedMps * 1000 },
    standoff_mm: { kind: "constant", value: process.torchStandoffMm ?? DEFAULT_TORCH_STANDOFF_MM },
    weave: weaveOf(torch),
    weld_parameters: {},
  };
  setClassicProvenance(node, {
    seam_id: seam.id,
    process_id: process.id,
    cad_derived: cadDerived,
    ...(curveCenters.some((center) => center !== null) ? { curve_centers_m: curveCenters } : {}),
  });
  return node;
}

function weaveOf(torch: ReturnType<typeof processTorchDefaults>): WeldNode["weave"] {
  if (torch.pattern === "none") return { shape: "none" };
  return {
    shape: torch.pattern,
    amplitude_mm: { kind: "constant", value: torch.amplitudeMm },
    frequency_hz: { kind: "constant", value: torch.frequencyHz },
  };
}

function standoffOf(offsetM: number): { direction: "workpiece_z"; distance_mm: number } {
  return { direction: "workpiece_z", distance_mm: Math.max(0, offsetM * 1000) };
}

/**
 * Make the classic nodes match the seams: a node for every sequence step
 * that has none, no node for a seam that is gone. Existing rows of a new
 * seam supply its points (baked RPY and teach edits preserved); otherwise
 * the points are computed. Returns whether anything changed.
 */
export function syncClassicNodes(project: OfflineProgrammingProject): boolean {
  let nodes = project.nodes;
  let changed = false;
  const seamIds = new Set(project.seams.map((seam) => seam.id));
  // A seam the row budget disabled and left rowless has no node either: the
  // budget is a fact about the wire contract, and a node that compiled to
  // rows would put the seam back over it.
  const rowless = deliberatelyRowlessSeamIDs(project);
  for (const node of [...nodes]) {
    const provenance = isWeld(node) ? classicProvenance(node) : null;
    if (provenance && (!seamIds.has(provenance.seam_id) || rowless.has(provenance.seam_id))) {
      nodes = removeNode(nodes, node.id);
      changed = true;
    }
  }
  const nodeBySeam = new Map<string, WeldNode>();
  for (const node of nodes) {
    if (!isWeld(node)) continue;
    const provenance = classicProvenance(node);
    if (provenance) nodeBySeam.set(provenance.seam_id, node);
  }
  for (const step of [...project.sequence].sort((left, right) => left.order - right.order)) {
    if (nodeBySeam.has(step.seamId) || rowless.has(step.seamId)) continue;
    const seam = project.seams.find((candidate) => candidate.id === step.seamId);
    if (!seam || seam.geometry.pointsM.length < 2) continue;
    const weldRows = project.rows.filter((row) => row.seamId === seam.id && row.phase === "weld");
    let points: PosedPoint[];
    let curveCenters: (Vec3 | null)[];
    if (weldRows.length >= 2) {
      points = weldRows.map((row) => posedPointFromPlannerPose(project, row.targetTcpPose));
      curveCenters = weldRows.slice(1).map((row) => {
        const center = moveMetadata(row)?.centerM;
        return center ? plannerPointToWorld(project, center) : null;
      });
    } else {
      ({ points, curveCenters } = classicWeldGeometry(project, seam, step));
    }
    if (points.length < 2) continue;
    const cadDerived = !weldRows.some((row) => row.sourceState === "teach_modified");
    const weld = classicWeldNode(project, seam, points, cadDerived, curveCenters);
    nodes = insertWeld(nodes, weld, {
      arrive: standoffOf(step.approachOffsetM),
      depart: standoffOf(step.retractOffsetM),
    });
    changed = true;
  }
  if (changed) project.nodes = nodes;
  return changed;
}

/**
 * Recompute the points of every classic weld that is still CAD-derived,
 * after a placement, trim or process change. A weld whose points the
 * operator edited keeps them. Returns whether any node changed.
 */
export function refreshClassicNodes(project: OfflineProgrammingProject): boolean {
  let changed = false;
  for (const node of project.nodes) {
    if (!isWeld(node)) continue;
    const provenance = classicProvenance(node);
    if (!provenance || !provenance.cad_derived) continue;
    const seam = project.seams.find((candidate) => candidate.id === provenance.seam_id);
    const step = project.sequence.find((candidate) => candidate.seamId === provenance.seam_id);
    if (!seam || !step || seam.geometry.pointsM.length < 2) continue;
    const { points, curveCenters } = classicWeldGeometry(project, seam, step);
    if (points.length < 2) continue;
    const process = processOf(project, seam);
    const torch = processTorchDefaults(process);
    node.geometry = {
      kind: "posed_polyline",
      frame_id: "world",
      closed: false,
      points,
      torch_angles_deg: { work: torch.workAngleDeg, travel: torch.travelAngleDeg, spin: torch.spinAngleDeg },
    };
    setClassicProvenance(node, {
      ...provenance,
      ...(curveCenters.some((center) => center !== null) ? { curve_centers_m: curveCenters } : {}),
    });
    if (!curveCenters.some((center) => center !== null)) {
      delete (node.extensions!.classic as Record<string, unknown>).curve_centers_m;
    }
    node.enabled = seam.enabled;
    changed = true;
  }
  return changed;
}

/** `workpiece.weld_joints[]`, or nothing when no node carries a joint. Absent
 *  rather than empty on a classic program: the field is optional, and an empty
 *  list would claim the part has no joints rather than that none were named. */
function weldJointsOf(nodes: readonly ProgramV2Node[]): { weld_joints?: Record<string, unknown>[] } {
  const byId = new Map<string, Record<string, unknown>>();
  for (const node of nodes) {
    const joint = railsJointOf(node);
    const id = joint?.id;
    if (!joint || typeof id !== "string" || id === "" || byId.has(id)) continue;
    byId.set(id, joint);
  }
  return byId.size ? { weld_joints: [...byId.values()] } : {};
}

/** The wire document a planner would receive. Assembled, never stored. */
export function toProgramV2Document(project: OfflineProgrammingProject): RobotV4ProgramV2 {
  const placement = workpiecePlacement(project);
  const anchor = workpieceAnchorM(project) ?? [0, 0, 0];
  const fingerprint = /^[0-9a-f]{64}$/.test(project.cad.fingerprint)
    ? project.cad.fingerprint
    : PLACEHOLDER_SHA256;
  return {
    schema: ROBOT_V4_PROGRAM_V2_SCHEMA,
    program_id: project.program.programId,
    program_name: project.identity.name,
    program_revision: project.program.programRevision,
    base_program_revision: project.program.baseProgramRevision,
    program_state: project.program.programState,
    execution_mode: project.program.executionMode,
    created_utc: project.identity.updatedAt,
    units: { length: "m", angle: "rad", time: "s", joint_order: JOINT_ORDER },
    producer: {
      component: "offline-programming-v1",
      source_revision: "offline-programming/v1",
      inputs: [
        {
          kind: "project",
          ref: `${project.identity.id}@${project.identity.revision}`,
          sha256: PLACEHOLDER_SHA256,
        },
        { kind: "cad", ref: project.cad.filename, sha256: fingerprint },
      ],
    },
    workpiece: {
      frame_id: project.railsAuthoring ? "workpiece" : "world",
      // Every rails seam names the joint it came from, so the list it names
      // has to be here or the reference dangles. Deduplicated: two welds cut
      // from the same joint at a corner name one joint, not two.
      ...weldJointsOf(project.nodes),
    },
    // Offsets from the middle of the table's top face, as authored. The weld
    // planner's work frame is that same point (seam_worker/robot/cell.py
    // WORK_SURFACES), so nothing is converted here.
    placement: {
      cell_id: project.robot.model,
      anchor_xyz_m: [anchor[0], anchor[1], anchor[2]],
      offset_xyz_m: placement ? [...placement.offsetFromWorkFrameM] : [0, 0, 0],
      rpy_rad: placement ? [...placement.rotationRpyRad] : [0, 0, 0],
    },
    // The robot and the description it was authored against, from the one
    // place the project keeps them (docs/PLANNING-CONTEXT.md). The pin is
    // strict: a description that moved makes this program stale until
    // somebody adopts the new one, typed, in the Robot menu.
    robot: {
      model_id: project.robot.model,
      robot_description_sha256: project.robot.description?.robotDescriptionSha256 || PLACEHOLDER_ROBOT_DESCRIPTION_SHA256,
    },
    ...(project.programV2.tool ? { tool: { ...project.programV2.tool } } : {}),
    defaults: { ...project.programV2.defaults },
    ...((project.programV2.speedScale ?? 1) !== 1 ? { speed_scale: project.programV2.speedScale } : {}),
    nodes: structuredClone(project.nodes),
    planner_owns: [...project.programV2.plannerOwns],
    extensions: {},
  };
}

/** Every problem with the project's program.v2 document; none when it has
 *  no nodes yet, because an empty program is not a wrong one. */
export function documentProblems(project: OfflineProgrammingProject): string[] {
  if (project.nodes.length === 0) return [];
  return programV2Problems(toProgramV2Document(project));
}
