/** Node values and status for the spreadsheet. UI units are mm and degrees. */

import type { AuthoredProgramRow } from "../domain/project.ts";
import type {
  Curve,
  FreespaceNode,
  FreespacePolicy,
  ProgramV2Node,
  ScalarFunction,
  Standoff,
  WeldNode,
} from "../contracts/robot-v4-program-v2.ts";
import { isFreespace, isWeld } from "./nodes.ts";

export interface NodeSummary {
  node: ProgramV2Node;
  id: string;
  number: number;
  op: ProgramV2Node["op"];
  /** weld | freespace | event -- one hue per class in the grid. */
  family: "weld" | "freespace" | "event";
  enabled: boolean;
  label: string;
  geometry: string;
  process: string;
  freespace: string;
  status: string;
  statusClass: "ok" | "stale" | "none" | "error";
  comment: string;
}

export interface SummaryContext {
  defaults: FreespacePolicy;
  /** The rows compiled from this node, for the status column. */
  rowsOf(nodeId: string): AuthoredProgramRow[];
  /** True when the node is left out of the compiled line because a weld
   *  it belongs to is disabled. */
  skipped?(nodeId: string): boolean;
  planned?(nodeId: string): boolean;
}

export function nodeSummary(node: ProgramV2Node, number: number, context: SummaryContext): NodeSummary {
  const base = {
    node,
    id: node.id,
    number,
    op: node.op,
    enabled: node.enabled,
    label: node.label,
    comment: node.comment,
  };
  const skipped = context.skipped?.(node.id) ?? false;
  if (node.op === "move") {
    return { ...base, family: "freespace", geometry: [
      ...node.target.xyz_m.map((v, i) => `${["X", "Y", "Z"][i]} ${(v * 1000).toFixed(1)} mm`),
      ...node.target.rpy_rad.map((v, i) => `${["Rx", "Ry", "Rz"][i]} ${(v * 180 / Math.PI).toFixed(1)}°`),
    ].join(" · "),
      process: `${Math.round(node.speed_scale * 100)}%`,
      freespace: { joint: "MoveJ", linear: "MoveL", circular: "MoveC" }[node.motion],
      status: node.enabled && node.motion === "circular" && !node.via ? "Record via required" : node.enabled ? `${node.capture.source === "machine" ? "Measured" : "Preview"} waypoint · ${context.planned?.(node.id) ? "planned" : "needs plan"}` : "disabled",
      statusClass: context.planned?.(node.id) ? "ok" : "none" };
  }
  if (isWeld(node)) {
    return {
      ...base,
      family: "weld",
      geometry: weldGeometry(node),
      process: weldProcess(node),
      freespace: "",
      ...(node.enabled ? weldStatus(node, context.rowsOf(node.id)) : { status: "disabled · not compiled", statusClass: "none" as const }),
    };
  }
  if (isFreespace(node)) {
    return {
      ...base,
      family: "freespace",
      geometry: "",
      process: "",
      freespace: freespaceText(node, context.defaults),
      ...(skipped
        ? { status: "skipped · its weld is disabled", statusClass: "none" as const }
        : context.planned?.(node.id) ? { status: "planned", statusClass: "ok" as const }
        : freespaceStatus(node, context.rowsOf(node.id))),
    };
  }
  if (node.op === "dwell") {
    return { ...base, family: "event", geometry: "", process: `${node.duration_s} s`, freespace: "", status: "event", statusClass: "none" };
  }
  return {
    ...base,
    family: "event",
    geometry: "",
    process: `${node.channel} ${node.state ? "ON" : "OFF"}`,
    freespace: "",
    status: "event",
    statusClass: "none",
  };
}

export function curveLengthM(curve: Curve): number {
  if (typeof curve.length_m === "number") return curve.length_m;
  return curve.segments.reduce((sum, segment) => sum + (segment.length_m ?? polylineLength(segment.control_points_xyz_m)), 0);
}

function polylineLength(points: readonly (readonly number[])[]): number {
  let total = 0;
  for (let index = 1; index < points.length; index += 1) {
    const a = points[index - 1]!;
    const b = points[index]!;
    total += Math.hypot(b[0]! - a[0]!, b[1]! - a[1]!, b[2]! - a[2]!);
  }
  return total;
}

export const mm = (metres: number): string =>
  `${(metres * 1000).toFixed(metres * 1000 >= 100 ? 0 : 1)} mm`;

function weldGeometry(node: WeldNode): string {
  const geometry = node.geometry;
  if (geometry.kind === "seam") {
    const span = geometry.span ?? {};
    const start = span.start_s ?? 0;
    const end = span.end_s ?? 1;
    const partial = start !== 0 || end !== 1;
    return [
      "seam",
      mm(curveLengthM(geometry.curve)),
      geometry.curve.closed ? "closed" : "open",
      geometry.direction === "reverse" ? "reverse" : "forward",
      partial ? `span ${start.toFixed(2)}–${end.toFixed(2)}` : null,
    ].filter(Boolean).join(" · ");
  }
  const length = polylineLength(geometry.points.map((point) => point.xyz_m));
  const first = geometry.points[0]!;
  return [
    geometry.kind === "manual_pose_wp" ? `${geometry.points.length} hand-placed poses` : `${geometry.points.length} points`,
    mm(length),
    `[${first.xyz_m.map((value) => (value * 1000).toFixed(1)).join(", ")}] mm…`,
  ].join(" · ");
}

export function scalarText(fn: ScalarFunction | undefined, unit: string): string {
  if (!fn) return "—";
  if (fn.kind === "constant") return `${trim(fn.value)} ${unit}`;
  return `profile (${fn.kind})`;
}

function trim(value: number): string {
  return String(Number(value.toFixed(3)));
}

function weldProcess(node: WeldNode): string {
  const parts = [scalarText(node.travel_speed_mm_s, "mm/s")];
  if (node.weave.shape !== "none") parts.push(`weave ${node.weave.shape}`);
  if (node.geometry.kind === "posed_polyline" && node.geometry.torch_angles_deg) {
    const angles = node.geometry.torch_angles_deg;
    parts.push(`torch ${trim(angles.work)}°/${trim(angles.travel)}°`);
  }
  return parts.join(" · ");
}

export function standoffText(standoff: Standoff | undefined, arrow: string): string | null {
  if (!standoff) return null;
  const direction = standoff.direction === "workpiece_z"
    ? "Z"
    : standoff.direction === "torch_axis"
      ? "torch"
      : `[${standoff.direction.map((value) => value.toFixed(2)).join(",")}]`;
  return `${arrow}${direction} ${trim(standoff.distance_mm)} mm`;
}

function freespaceText(node: FreespaceNode, defaults: FreespacePolicy): string {
  const parts = [
    standoffText(node.depart_standoff, "↑"),
    standoffText(node.arrive_standoff, "↓"),
  ].filter((part): part is string => part !== null);
  const overrides = node.overrides ?? {};
  if (overrides.clearance_min_mm !== undefined) parts.push(`clearance ${trim(overrides.clearance_min_mm)} mm`);
  if (overrides.speed_scale !== undefined) parts.push(`speed ×${trim(overrides.speed_scale)}`);
  if (overrides.torch_policy !== undefined) parts.push(`torch ${overrides.torch_policy}`);
  if (overrides.positioner_policy !== undefined) parts.push(`positioner ${overrides.positioner_policy}`);
  if (parts.length === 0) parts.push(`planner decides · clearance ${trim(defaults.clearance_min_mm)} mm`);
  return parts.join(" · ");
}

function weldStatus(node: WeldNode, rows: AuthoredProgramRow[]): Pick<NodeSummary, "status" | "statusClass"> {
  if (rows.length === 0) {
    // Seams and hand-placed poses are the weld planner's to compile; only a
    // posed polyline lowers here, so only its missing rows are an error.
    return node.geometry.kind !== "posed_polyline"
      ? { status: "not compiled", statusClass: "stale" }
      : { status: "no rows", statusClass: "error" };
  }
  return rowStatus(rows);
}

function freespaceStatus(node: FreespaceNode, rows: AuthoredProgramRow[]): Pick<NodeSummary, "status" | "statusClass"> {
  if (rows.length === 0) return { status: "planner decides", statusClass: "none" };
  return rowStatus(rows);
}

function rowStatus(rows: AuthoredProgramRow[]): Pick<NodeSummary, "status" | "statusClass"> {
  const states = new Set(rows.map((row) => row.plannerStatus ?? "not_planned"));
  const validation = new Set(rows.map((row) => row.validationState ?? "unknown"));
  if (validation.has("invalid") || validation.has("failed")) return { status: `${rows.length} rows · invalid`, statusClass: "error" };
  if (states.has("stale") || states.has("not_planned")) return { status: `${rows.length} rows · unplanned`, statusClass: "stale" };
  return { status: `${rows.length} rows · ${[...states].join("/")}`, statusClass: "ok" };
}
