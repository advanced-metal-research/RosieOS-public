import { robotModelFor } from "../viewer/models.ts";
/**
 * The classic lowering: nodes to `robot.v4.program.v1` rows.
 *
 * Rows are what the Motion Server's Tesseract planner reads, so a program
 * with posed welds compiles here, in the browser, every time its nodes
 * change. A posed weld's points become one motion row each with the RPY
 * carried verbatim -- no re-decimation, no re-orientation, the node is the
 * authority. A freespace node becomes the lift it states: an approach's
 * arrival row before the weld's first point, a retract's departure row
 * after its last, a transit's departure and arrival rows between two welds.
 * A stated lift is honoured; an unstated one on an approach or retract
 * lowers as the classic default (`DEFAULT_STANDOFF_MM` along workpiece +Z),
 * and an unstated one on a transit lowers as nothing, which is what the
 * classic program did between seams. Events lower to the nearest v1 row
 * kinds: a home command, a comment for a dwell, an io row.
 *
 * A `seam`-kind weld cannot be lowered here -- its curve is analytic and its
 * torch is a band -- so it emits nothing and the weld planner's compile owns
 * its rows. Every row carries `extensions.node_id`, which is how selection,
 * playback and status find their way back to the node.
 */

import type { AuthoredProgramRow, AuthoredSeam, OfflineProgrammingProject, Rpy, RowPhase, Vec3 } from "../domain/project.ts";
import { DEFAULT_TRAVEL_SPEED_MPS } from "../domain/project.ts";
import type {
  FreespaceNode,
  PosedPoint,
  ProgramV2Node,
  ScalarFunction,
  Standoff,
  WeldNode,
} from "../contracts/robot-v4-program-v2.ts";
import { annotateCadSeamMoves, makeRow, moveToleranceM } from "../adapters/cadquery.ts";
import { createSpreadsheetRow } from "../domain/spreadsheet.ts";
import { transformWorkpiecePoint, worldPointToPlanner } from "../domain/placement.ts";
import { attachMoveMetadata, type MoveSegmentInput } from "../domain/points-moves.ts";
import { processTorchDefaults } from "../domain/weld.ts";
import { DEFAULT_STANDOFF_MM, classicProvenance } from "./from-project.ts";
import { effectiveNodes, isFreespace, isWeld } from "./nodes.ts";

export const NODE_ID_EXTENSION = "node_id" as const;

export function rowNodeId(row: AuthoredProgramRow): string | null {
  const value = row.extensions?.[NODE_ID_EXTENSION];
  return typeof value === "string" ? value : null;
}

/**
 * The rows compiled from a node. Rows written since nodes existed carry the
 * node id; rows of a project saved before that (or compiled by the weld
 * planner before the ids were stamped) are matched by the seam they name
 * and their phase, read-only, so a migrated program shows its status
 * without its saved rows being rewritten on load.
 */
export function rowsOfNode(project: OfflineProgrammingProject, nodeId: string): AuthoredProgramRow[] {
  const stamped = project.rows.filter((row) => rowNodeId(row) === nodeId);
  if (stamped.length > 0) return stamped;
  const index = project.nodes.findIndex((node) => node.id === nodeId);
  const node = project.nodes[index];
  if (!node) return [];
  const seamOf = (candidate: ProgramV2Node | undefined): string | null => {
    if (!candidate || !isWeld(candidate)) return null;
    if (candidate.geometry.kind === "seam") return candidate.geometry.id;
    return classicProvenance(candidate)?.seam_id ?? null;
  };
  // A row whose node id names no node (a save whose nodes were rebuilt)
  // is as unstamped as one with no id.
  const known = new Set(project.nodes.map((candidate) => candidate.id));
  const unstamped = (row: AuthoredProgramRow): boolean => {
    const id = rowNodeId(row);
    return id === null || !known.has(id);
  };
  if (isWeld(node)) {
    const seamId = seamOf(node);
    return seamId ? project.rows.filter((row) => unstamped(row) && row.seamId === seamId && row.phase === "weld") : [];
  }
  if (!isFreespace(node)) return [];
  const neighbour = (step: -1 | 1): ProgramV2Node | undefined => {
    for (let at = index + step; at >= 0 && at < project.nodes.length; at += step) {
      const candidate = project.nodes[at]!;
      if (isWeld(candidate) || isFreespace(candidate)) return candidate;
    }
    return undefined;
  };
  const before = seamOf(neighbour(-1));
  const after = seamOf(neighbour(1));
  return project.rows.filter((row) =>
    unstamped(row) &&
    ((row.phase === "retract" && row.seamId === before) || (row.phase === "approach" && row.seamId === after))
  );
}

/** The value of a scalar profile at its start, for a row that needs one number. */
export function scalarStart(fn: ScalarFunction | undefined, fallback: number): number {
  if (!fn) return fallback;
  switch (fn.kind) {
    case "constant": return fn.value;
    case "piecewise_linear": return fn.knots[0]?.[1] ?? fallback;
    case "bspline": return fn.control_values[0] ?? fallback;
    case "samples": return fn.values[0] ?? fallback;
  }
}

export function rowsFromNodes(project: OfflineProgrammingProject): AuthoredProgramRow[] {
  const rows: AuthoredProgramRow[] = [];
  const weldRowSpans: { node: WeldNode; seamId: string; arrive: number | null; start: number; end: number; depart: number | null; curveCenters: (Vec3 | null)[] }[] = [];
  let pending: FreespaceNode | null = null;
  let last: { node: WeldNode; poses: PosedPoint[]; seam: AuthoredSeam | null } | null = null;
  const toleranceM = moveToleranceM(project);

  const push = (node: ProgramV2Node, suffix: string, row: AuthoredProgramRow): void => {
    row.id = `${node.id}:${suffix}`;
    row.rowHash = `offline:${row.id}:${project.program.programRevision + 1}`;
    // Only enabled nodes reach this point; a disabled weld and the moves
    // it needed are already gone from the effective line.
    row.extensions = { ...(row.extensions ?? {}), [NODE_ID_EXTENSION]: node.id, node_op: node.op };
    rows.push(row);
  };

  for (const node of effectiveNodes(project.nodes)) {
    if (node.op === "move") {
      const row = createSpreadsheetRow(project, "motion");
      row.label = node.label; row.comment = node.comment;
      row.moveType = node.motion === "joint" ? "JOINT" : node.motion === "linear" ? "LINE" : "CURVE";
      row.targetTcpPose = [...worldPointToPlanner(project, node.target.xyz_m), ...node.target.rpy_rad];
      const names = robotModelFor(project.robot.model).urdfJointNames;
      names.forEach((name, i) => {
        const value = node.target.joint_values_rad[node.target.joint_names.indexOf(name)];
        if (value === undefined || !Number.isFinite(value)) throw new Error(`Waypoint ${node.label} is missing ${name}`);
        row.targetJointsRad[i] = value;
      });
      row.tcpVelocityMps = node.speed_mm_s / 1000;
      row.fineStop = true; row.blendRadiusM = 0; row.weldEnabled = false;
      push(node, "target", row);
      pending = null; last = null;
      continue;
    }
    if (isWeld(node)) {
      if (node.geometry.kind !== "posed_polyline") {
        // The weld planner compiles this one; nothing to lower here.
        pending = null;
        last = null;
        continue;
      }
      const provenance = classicProvenance(node);
      const seam = provenance
        ? project.seams.find((candidate) => candidate.id === provenance.seam_id) ?? null
        : null;
      const poses = node.geometry.points;
      const first = poses[0]!;
      const process = seam
        ? project.processParameters.find((candidate) => candidate.id === seam.processParameterId)
        : undefined;
      const defaults = processTorchDefaults(process ?? project.processParameters[0]);
      const torch = {
        ...defaults,
        ...(node.geometry.torch_angles_deg
          ? {
              workAngleDeg: node.geometry.torch_angles_deg.work,
              travelAngleDeg: node.geometry.torch_angles_deg.travel,
              spinAngleDeg: node.geometry.torch_angles_deg.spin,
            }
          : {}),
        pattern: weavePattern(node),
        amplitudeMm: scalarStart(node.weave.amplitude_mm as ScalarFunction | undefined, defaults.amplitudeMm),
        frequencyHz: scalarStart(node.weave.frequency_hz as ScalarFunction | undefined, defaults.frequencyHz),
      };
      let arrive: number | null = null;
      if (pending && (pending.op === "approach" || pending.op === "transit")) {
        const lift = pending.arrive_standoff ?? (pending.op === "approach" ? defaultStandoff() : null);
        if (lift) {
          arrive = rows.length;
          const point = liftedPoint(project, first, lift);
          const speed = (process ?? project.processParameters[0])?.approachSpeedMps ?? 0.1;
          // A transit's arrival lowers as the classic approach row: the phase
          // is what torch synchronisation and the overlays key on.
          push(pending, "arrive", makeRow(project, seam, "approach", worldPointToPlanner(project, point), false, speed, 0, first.rpy_rad, torch));
        }
      }
      pending = null;
      const speed = scalarStart(node.travel_speed_mm_s, DEFAULT_TRAVEL_SPEED_MPS * 1000) / 1000;
      const start = rows.length;
      poses.forEach((pose, index) => {
        const row = makeRow(project, seam, "weld", worldPointToPlanner(project, pose.xyz_m), true, speed, index, pose.rpy_rad, torch);
        if (!seam) {
          row.label = `${node.label} · WELD ${index + 1}`;
          row.seamId = node.id;
        }
        push(node, `w${index}`, row);
      });
      weldRowSpans.push({ node, seamId: seam?.id ?? node.id, arrive, start, end: rows.length - 1, depart: null, curveCenters: provenance?.curve_centers_m ?? [] });
      last = { node, poses, seam };
      continue;
    }
    if (isFreespace(node)) {
      if ((node.op === "retract" || node.op === "transit") && last) {
        const lift = node.depart_standoff ?? (node.op === "retract" ? defaultStandoff() : null);
        if (lift) {
          const end = last.poses.at(-1)!;
          const point = liftedPoint(project, end, lift);
          const process = last.seam
            ? project.processParameters.find((candidate) => candidate.id === last!.seam!.processParameterId)
            : undefined;
          const speed = (process ?? project.processParameters[0])?.retractSpeedMps ?? 0.1;
          // A transit's departure lowers as the classic retract row.
          const phase: RowPhase = "retract";
          const torch = processTorchDefaults(process ?? project.processParameters[0]);
          const span = weldRowSpans.at(-1);
          if (span && span.node === last.node) span.depart = rows.length;
          push(node, "depart", makeRow(project, last.seam, phase, worldPointToPlanner(project, point), false, speed, 0, end.rpy_rad, torch));
        }
      }
      if (node.op === "home") {
        const row = createSpreadsheetRow(project, "command");
        row.command = "HOME";
        row.label = node.label;
        row.comment = node.comment;
        row.loopTargetRowId = null;
        row.loopIterations = 0;
        row.phase = "home";
        push(node, "home", row);
        pending = null;
        last = null;
        continue;
      }
      pending = node;
      continue;
    }
    if (node.op === "dwell") {
      const row = createSpreadsheetRow(project, "comment");
      row.label = node.label;
      row.comment = `DWELL ${node.duration_s} s${node.comment ? ` · ${node.comment}` : ""}`;
      push(node, "dwell", row);
      continue;
    }
    if (node.op === "io") {
      const row = createSpreadsheetRow(project, "io");
      row.label = node.label;
      row.comment = `${node.action === "monitor" ? "MONITOR " : node.action === "end_monitor" ? "END MONITOR " : node.action === "wait" ? "WAIT " : node.action === "pulse" ? "PULSE " : ""}${node.channel} ${node.state ? "ON" : "OFF"}${node.action === "pulse" ? ` · ${node.duration_s} s` : node.action === "wait" ? ` · timeout ${node.timeout_s} s, stable ${node.stable_s ?? 0} s` : ""}${node.comment ? ` · ${node.comment}` : ""}`;
      const slot = /(\d+)\s*$/.exec(node.channel);
      row.revPiIOSlot = slot ? Number(slot[1]) : null;
      row.revPiIOAction = node.state ? "weld_on" : "weld_off";
      push(node, "io", row);
    }
  }

  for (const span of weldRowSpans) {
    const centers = span.curveCenters.map((center) => (center ? worldPointToPlanner(project, center) : null));
    if (span.arrive !== null && span.depart !== null) {
      // The seam compile's shape: approach, control points, retract, one
      // move per adjacent pair, so the compiled view groups them the same.
      annotateCadSeamMoves(rows, span.arrive, span.depart, span.seamId, toleranceM, centers);
      continue;
    }
    const segments: MoveSegmentInput[] = [];
    for (let index = span.start; index < span.end; index += 1) {
      const centerM = centers[index - span.start] ?? null;
      segments.push(centerM
        ? { startIndex: index, endIndex: index + 1, kind: "CURVE", centerM, toleranceM }
        : { startIndex: index, endIndex: index + 1, kind: "LINE", toleranceM });
    }
    if (segments.length) attachMoveMetadata(rows, segments, `cad:${span.seamId}`);
  }
  return reconcileRows(project.rows, rows);
}

/** The fields the planner writes on a row, which a recompile must not reset. */
const ROW_STATUS_KEYS = [
  "rowRevision",
  "rowHash",
  "plannerStatus",
  "validationState",
  "collisionCheckState",
  "limitCheckState",
  "plannedPathRevision",
  "plannedSampleCount",
] as const;

/**
 * Two things a recompile owes the rows it replaces. A row that comes back
 * with the same id keeps its planner status and hash: it is the same row,
 * and `reviseProject` stales it only if its content changed -- a fresh
 * `not_planned` on an untouched row would hide that it was already planned.
 * And rows the operator authored by hand -- no node, no seam -- survive at
 * the index they had; they are simply not what nodes own.
 */
export function reconcileRows(previous: readonly AuthoredProgramRow[], generated: AuthoredProgramRow[]): AuthoredProgramRow[] {
  const prior = new Map(previous.map((row) => [row.id, row]));
  for (const row of generated) {
    const before = prior.get(row.id);
    if (!before) continue;
    for (const key of ROW_STATUS_KEYS) {
      if (before[key] !== undefined) (row as unknown as Record<string, unknown>)[key] = before[key];
    }
  }
  const manual = previous
    .map((row, index) => ({ row, index }))
    .filter(({ row }) => rowNodeId(row) === null && row.seamId === null);
  if (manual.length === 0) return generated;
  const out = [...generated];
  for (const { row, index } of manual) out.splice(Math.min(index, out.length), 0, row);
  return out;
}

function weavePattern(node: WeldNode): "none" | "zigzag" | "circular" | "trapezoidal" {
  const shape = node.weave.shape;
  return shape === "zigzag" || shape === "circular" || shape === "trapezoidal" ? shape : "none";
}

function defaultStandoff(): Standoff {
  return { direction: "workpiece_z", distance_mm: DEFAULT_STANDOFF_MM };
}

/** A pose lifted by a standoff, world frame. */
export function liftedPoint(
  project: OfflineProgrammingProject,
  pose: PosedPoint,
  standoff: Standoff,
): Vec3 {
  const distance = standoff.distance_mm / 1000;
  let direction: Vec3;
  if (standoff.direction === "workpiece_z") {
    direction = workpieceAxisInWorld(project, [0, 0, 1]);
  } else if (standoff.direction === "torch_axis") {
    // The electrode points along tool +X; backing off is along -X.
    const axis = toolXAxis(pose.rpy_rad);
    direction = [-axis[0], -axis[1], -axis[2]];
  } else {
    direction = workpieceAxisInWorld(project, standoff.direction);
  }
  return [
    pose.xyz_m[0] + direction[0] * distance,
    pose.xyz_m[1] + direction[1] * distance,
    pose.xyz_m[2] + direction[2] * distance,
  ];
}

function workpieceAxisInWorld(project: OfflineProgrammingProject, axis: Vec3): Vec3 {
  const origin = transformWorkpiecePoint(project, [0, 0, 0]);
  const tip = transformWorkpiecePoint(project, axis);
  const vector: Vec3 = [tip[0] - origin[0], tip[1] - origin[1], tip[2] - origin[2]];
  const length = Math.hypot(...vector) || 1;
  return [vector[0] / length, vector[1] / length, vector[2] / length];
}

/** Tool +X in the world, for URDF fixed-axis RPY (Rz · Ry · Rx). */
export function toolXAxis(rpy: Rpy): Vec3 {
  const [roll, pitch, yaw] = rpy;
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll), sr = Math.sin(roll);
  // First column of Rz(yaw)·Ry(pitch)·Rx(roll).
  void cr; void sr;
  return [cy * cp, sy * cp, -sp];
}

/**
 * Give rows the weld planner compiled their node ids: a weld row by the
 * seam it names, an approach row the freespace node before that weld, a
 * retract or transit row the one after. Rows that name no seam stay
 * unstamped.
 */
export function stampRowNodeIds(rows: AuthoredProgramRow[], nodes: readonly ProgramV2Node[]): void {
  const weldBySeam = new Map<string, number>();
  nodes.forEach((node, index) => {
    if (isWeld(node) && node.geometry.kind === "seam") weldBySeam.set(node.geometry.id, index);
  });
  const motionNeighbour = (from: number, step: -1 | 1): ProgramV2Node | null => {
    for (let index = from + step; index >= 0 && index < nodes.length; index += step) {
      const node = nodes[index]!;
      if (isFreespace(node) || isWeld(node)) return node;
    }
    return null;
  };
  for (const row of rows) {
    if (!row.seamId) continue;
    const weldIndex = weldBySeam.get(row.seamId);
    if (weldIndex === undefined) continue;
    const owner = row.phase === "weld"
      ? nodes[weldIndex]!
      : row.phase === "approach"
        ? motionNeighbour(weldIndex, -1)
        : motionNeighbour(weldIndex, 1);
    if (owner) row.extensions = { ...(row.extensions ?? {}), [NODE_ID_EXTENSION]: owner.id, node_op: owner.op };
  }
}
