/**
 * A waypoint weld as a program.v2 node, and back.
 *
 * One weld is one `op: "weld"` node with `manual_pose_wp` geometry: each
 * waypoint's work point and orientation (as URDF roll, pitch, yaw) and its
 * continuity, in the workpiece frame. The per-waypoint standoff is the
 * node's `standoff_mm`, a piecewise-linear function with a knot at each
 * waypoint's normalized arc length -- the schema keeps scalars as functions
 * of s, never per-point arrays. The waypoint ids the tool selects by ride in
 * `extensions.manual_pose_wp`, because the points themselves are closed.
 *
 * The node is the authority; the tool reads it back with `waypointWeldOf`
 * and writes every edit with `withWaypoints`, so undo, autosave and the
 * catalogue all see the weld as the program does.
 *
 * Three tools make these welds, and the node says which in
 * `extensions.manual_pose_wp.tool`: a waypoint weld (any number of points,
 * the spline rule), a line (two points, the same rule) and an arc (three
 * points, `shape: "arc"` in the geometry, because that one changes what the
 * points mean). `start_from` names another such weld whose END this weld's
 * first point sits on; `resolveStartLinks` keeps it there.
 */

import type {
  ManualPoseContinuity,
  ManualPoseGeometry,
  ProgramV2Node,
  ScalarFunction,
  WeldNode,
} from "../contracts/robot-v4-program-v2.ts";
import { projectId } from "../domain/project.ts";
import {
  DEFAULT_WAYPOINT_STANDOFF_MM,
  clampStandoffMm,
  waypointArcFractions,
  type Continuity,
  type Quat,
  type V3,
  type Waypoint,
  type WeldShape,
} from "./model.ts";

/** The frame a rails program's geometry is in (`toProgramV2Document`); the
 *  document refuses a manual_pose_wp weld in any other. */
export const MANUAL_POSE_FRAME = "workpiece";
export const MANUAL_POSE_EXTENSION = "manual_pose_wp" as const;

export type WeldTool = "waypoint" | "line" | "arc";

export interface WaypointWeld {
  id: string;
  label: string;
  enabled: boolean;
  tool: WeldTool;
  /** The weld whose last point this weld's first point is tied to. */
  startFrom: string | null;
  waypoints: Waypoint[];
}

/** The path rule a tool's points follow. */
export const shapeOf = (tool: WeldTool): WeldShape => (tool === "arc" ? "arc" : "spline");

export const isManualPoseWeld = (node: ProgramV2Node): node is WeldNode & { geometry: ManualPoseGeometry } =>
  node.op === "weld" && node.geometry.kind === "manual_pose_wp";

export function waypointWeldOf(node: WeldNode & { geometry: ManualPoseGeometry }): WaypointWeld {
  const ids = pointIds(node);
  const extension = manualExtension(node);
  const points = node.geometry.points;
  const skeleton = points.map((point) => ({
    positionM: [...point.xyz_m] as V3,
    continuity: fromWire(point.continuity),
  }));
  const tool = node.geometry.shape === "arc" ? "arc" : extension.tool === "line" ? "line" : "waypoint";
  const fractions = waypointArcFractions(skeleton, shapeOf(tool));
  const waypoints = points.map((point, index): Waypoint => ({
    // A point written by another producer has no id of ours; a derived one
    // is stable for as long as the point keeps its place.
    id: ids[index] ?? `${node.id}:${index}`,
    positionM: skeleton[index]!.positionM,
    quaternion: rpyToQuat(point.rpy_rad as V3),
    continuity: skeleton[index]!.continuity,
    standoffMm: clampStandoffMm(evaluate(node.standoff_mm, fractions[index] ?? 0)),
  }));
  return {
    id: node.id,
    label: node.label,
    enabled: node.enabled,
    tool,
    startFrom: extension.start_from,
    waypoints,
  };
}

/** The node with its geometry and standoff rewritten from `waypoints`,
 *  and its tool and start link from `meta` where given (else kept). */
export function withWaypoints(
  node: WeldNode,
  waypoints: readonly Waypoint[],
  meta: { tool?: WeldTool; startFrom?: string | null } = {},
): WeldNode {
  const next = structuredClone(node);
  const extension = isManualPoseWeld(node) ? manualExtension(node) : { tool: undefined, start_from: null };
  const tool: WeldTool = meta.tool ?? (node.geometry.kind === "manual_pose_wp" && node.geometry.shape === "arc"
    ? "arc"
    : extension.tool === "line" ? "line" : "waypoint");
  const startFrom = meta.startFrom !== undefined ? meta.startFrom : extension.start_from;
  next.geometry = {
    kind: "manual_pose_wp",
    frame_id: MANUAL_POSE_FRAME,
    ...(tool === "arc" ? { shape: "arc" as const } : {}),
    points: waypoints.map((point) => ({
      xyz_m: [...point.positionM],
      rpy_rad: quatToRpy(point.quaternion),
      continuity: toWire(point.continuity),
    })),
  };
  next.standoff_mm = standoffFunction(waypoints, shapeOf(tool));
  next.extensions = {
    ...(next.extensions ?? {}),
    [MANUAL_POSE_EXTENSION]: {
      point_ids: waypoints.map((point) => point.id),
      tool,
      ...(startFrom ? { start_from: startFrom } : {}),
    },
  };
  return next;
}

/**
 * Every start link, followed: a weld whose `start_from` names another
 * manual weld gets its first point moved onto that weld's last point --
 * position only; the torch there keeps its own orientation, since a weld
 * that starts where another ended often starts at a different angle. In
 * dependency order, so a chain settles in one call. A link to a weld that is
 * gone, or that would close a loop, is dropped rather than followed.
 */
export function resolveStartLinks(nodes: readonly ProgramV2Node[]): ProgramV2Node[] {
  const welds = new Map(nodes.filter(isManualPoseWeld).map((node) => [node.id, node]));
  const out = new Map<string, WeldNode>();
  const settling = new Set<string>();
  const settle = (id: string): WeldNode => {
    const done = out.get(id);
    if (done) return done;
    const node = welds.get(id)!;
    const weld = waypointWeldOf(node);
    let result: WeldNode = node;
    settling.add(id);
    const source = weld.startFrom ? welds.get(weld.startFrom) : undefined;
    if (weld.startFrom && (!source || settling.has(weld.startFrom))) {
      result = withWaypoints(node, weld.waypoints, { startFrom: null });
    } else if (source) {
      const end = waypointWeldOf(settle(source.id) as typeof source).waypoints.at(-1);
      const first = weld.waypoints[0];
      if (end && first && !samePosition(first.positionM, end.positionM)) {
        const moved = [{ ...first, positionM: [...end.positionM] as V3 }, ...weld.waypoints.slice(1)];
        result = withWaypoints(node, moved);
      }
    }
    settling.delete(id);
    out.set(id, result);
    return result;
  };
  let changed = false;
  const resolved = nodes.map((node) => {
    if (!isManualPoseWeld(node)) return node;
    const next = settle(node.id);
    if (next !== node) changed = true;
    return next;
  });
  return changed ? resolved : [...nodes];
}

/** The welds a weld could start from: every other manual weld that does
 *  not, through its own links, start from this one. */
export function startCandidates(nodes: readonly ProgramV2Node[], weldId: string): WeldNode[] {
  const welds = nodes.filter(isManualPoseWeld);
  const startOf = new Map(welds.map((node) => [node.id, manualExtension(node).start_from]));
  const dependsOn = (id: string): boolean => {
    const seen = new Set<string>();
    for (let at: string | null | undefined = id; at; at = startOf.get(at)) {
      if (at === weldId) return true;
      if (seen.has(at)) return false;
      seen.add(at);
    }
    return false;
  };
  return welds.filter((node) => node.id !== weldId && !dependsOn(node.id));
}

function samePosition(a: readonly number[], b: readonly number[]): boolean {
  return a[0] === b[0] && a[1] === b[1] && a[2] === b[2];
}

function manualExtension(node: WeldNode): { tool?: string; start_from: string | null } {
  const value = node.extensions?.[MANUAL_POSE_EXTENSION] as { tool?: unknown; start_from?: unknown } | undefined;
  return {
    tool: typeof value?.tool === "string" ? value.tool : undefined,
    start_from: typeof value?.start_from === "string" ? value.start_from : null,
  };
}

export interface ManualWeldProcess {
  travelSpeedMmS: number;
  weave: WeldNode["weave"];
}

export function manualPoseWeldNode(
  label: string,
  waypoints: readonly Waypoint[],
  process: ManualWeldProcess,
  meta: { tool?: WeldTool; startFrom?: string | null } = {},
): WeldNode {
  const id = projectId();
  const node: WeldNode = {
    id,
    op: "weld",
    enabled: true,
    label,
    comment: "",
    row_hash: `pending:${id}`,
    geometry: { kind: "manual_pose_wp", frame_id: MANUAL_POSE_FRAME, points: [] },
    travel_speed_mm_s: { kind: "constant", value: process.travelSpeedMmS },
    standoff_mm: { kind: "constant", value: DEFAULT_WAYPOINT_STANDOFF_MM },
    weave: structuredClone(process.weave),
    weld_parameters: {},
  };
  return withWaypoints(node, waypoints, { tool: meta.tool ?? "waypoint", startFrom: meta.startFrom ?? null });
}

/** The first label of the form "Waypoint weld N" no node already uses. */
export function nextWeldLabel(nodes: readonly ProgramV2Node[], tool: WeldTool = "waypoint"): string {
  const stem = { waypoint: "Waypoint weld", line: "Line", arc: "Arc" }[tool];
  const used = new Set(nodes.map((node) => node.label));
  let index = 1;
  while (used.has(`${stem} ${index}`)) index += 1;
  return `${stem} ${index}`;
}

/** Pairs of consecutive waypoints that sit on top of each other, as
 *  indexes of the second -- what the document refuses, said early. */
export function coincidentWaypoints(waypoints: readonly Waypoint[], minChordM: number): number[] {
  const out: number[] = [];
  for (let i = 1; i < waypoints.length; i += 1) {
    const a = waypoints[i - 1]!.positionM;
    const b = waypoints[i]!.positionM;
    if (Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) <= minChordM) out.push(i);
  }
  return out;
}

/**
 * Knots at each waypoint's arc length. A waypoint that shares its s with the
 * one before (coincident, which the document refuses anyway) contributes no
 * knot, so the function stays well-formed while the tool shows the problem.
 */
function standoffFunction(waypoints: readonly Waypoint[], shape: WeldShape): ScalarFunction {
  const values = waypoints.map((point) => point.standoffMm);
  if (values.length === 0) return { kind: "constant", value: DEFAULT_WAYPOINT_STANDOFF_MM };
  if (values.every((value) => value === values[0])) return { kind: "constant", value: values[0]! };
  const fractions = waypointArcFractions(waypoints, shape);
  const knots: [number, number][] = [];
  fractions.forEach((s, index) => {
    const last = knots.at(-1);
    if (last && s <= last[0]) return;
    knots.push([s, values[index]!]);
  });
  if (knots.length < 2) return { kind: "constant", value: values[0]! };
  knots[knots.length - 1]![0] = 1;
  return { kind: "piecewise_linear", knots };
}

function evaluate(fn: ScalarFunction, s: number): number {
  if (fn.kind === "constant") return fn.value;
  if (fn.kind === "piecewise_linear") {
    const knots = fn.knots;
    if (s <= knots[0]![0]) return knots[0]![1];
    for (let i = 1; i < knots.length; i += 1) {
      const [s1, v1] = knots[i]!;
      if (s <= s1) {
        const [s0, v0] = knots[i - 1]!;
        return v0 + ((s - s0) / (s1 - s0 || 1)) * (v1 - v0);
      }
    }
    return knots[knots.length - 1]![1];
  }
  return DEFAULT_WAYPOINT_STANDOFF_MM;
}

function pointIds(node: WeldNode): string[] {
  const value = node.extensions?.[MANUAL_POSE_EXTENSION] as { point_ids?: unknown } | undefined;
  const ids = value?.point_ids;
  return Array.isArray(ids) && ids.every((id) => typeof id === "string") ? ids as string[] : [];
}

function toWire(continuity: Continuity): ManualPoseContinuity {
  return continuity.toUpperCase() as ManualPoseContinuity;
}

function fromWire(continuity: ManualPoseContinuity): Continuity {
  return continuity.toLowerCase() as Continuity;
}

/** URDF fixed-axis roll, pitch, yaw -- R = Rz(yaw) Ry(pitch) Rx(roll) -- to a
 *  unit quaternion, x y z w. */
export function rpyToQuat([roll, pitch, yaw]: V3): Quat {
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2);
  const sy = Math.sin(yaw / 2);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

/** The inverse of `rpyToQuat`. Pitch is clamped at ±90°, where roll and yaw
 *  share one axis and any split of them is the same rotation. */
export function quatToRpy([x, y, z, w]: Quat): V3 {
  const length = Math.hypot(x, y, z, w) || 1;
  [x, y, z, w] = [x / length, y / length, z / length, w / length];
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.min(1, Math.max(-1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}
