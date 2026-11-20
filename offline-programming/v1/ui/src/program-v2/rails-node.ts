/**
 * A rails weld as a node, and back.
 *
 * When the operator accepts or trims a rails entry, the weld planner's seam
 * becomes a `seam`-kind weld node: the seam JSON with the curve inlined
 * from its joint, the decided span, the direction and angle profiles the
 * search proposed folded in as the node's own orientation. From then on the
 * node is the authority -- the dialog edits its span, direction, profiles
 * and speed -- and the compile builds the weld planner's payload back out
 * of it with `railsPayloadOf`, so a reloaded program compiles without the
 * session's search state.
 *
 * `extensions.rails` keeps the payload the search produced (preset id,
 * stickout override, ids) and the STEP sha the seam belongs to.
 */

import type { ProgramV2Node, ScalarFunction, SeamGeometry, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import type { RailsJoint, RailsSeam, RailsWeldPayload } from "../rails/client.ts";
import { DEFAULT_TORCH_STANDOFF_MM } from "../domain/project.ts";
import { isWeld } from "./nodes.ts";

export const RAILS_EXTENSION = "rails" as const;

/** Fields the planner never reads and the document does not need: the
 *  accessibility study is hundreds of samples per joint and both are authoring
 *  aids, optional in the schema. Everything else rides along, because a weld
 *  program that says "tee, fillet, 90 degrees, solid_0 to solid_2" is legible
 *  where a bare curve is not. */
const JOINT_AIDS = ["accessibility", "accessible_sides"] as const;

/** The joint a seam names, as program.v2 carries it. */
export function jointForDocument(joint: Record<string, unknown>): Record<string, unknown> {
  const out = structuredClone(joint);
  for (const key of JOINT_AIDS) delete out[key];
  return out;
}

export interface RailsProvenance {
  payload: RailsWeldPayload;
  joint_id: string;
  sha256: string;
  joint: Record<string, unknown> | null;
}

export interface RailsEntryLike {
  id: string;
  seam: RailsSeam;
  joint: RailsJoint;
  span: { start_s: number; end_s: number };
  payload: RailsWeldPayload | null;
  note: string;
}

export function railsProvenance(node: ProgramV2Node): RailsProvenance | null {
  const value = node.extensions?.[RAILS_EXTENSION];
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const record = value as Record<string, unknown>;
  if (!record.payload || typeof record.payload !== "object") return null;
  return {
    payload: record.payload as RailsWeldPayload,
    joint_id: String(record.joint_id ?? ""),
    sha256: String(record.sha256 ?? ""),
    joint: isRecord(record.joint) ? record.joint : null,
  };
}

const SEAM_KEYS = ["label", "weld_joint_ref", "origin", "pass", "weld_geometry", "lead", "tolerance", "sampling_hint"] as const;

/** What this weld is welded at, in millimetres.
 *
 *  Three sources, most specific first: the standoff the ANGLE SEARCH chose
 *  for this seam (it lands in the payload's process override as the wire's
 *  stickout), then the stickout the detected seam's own procedure states,
 *  then the default for a gun that is held somewhere.
 *
 *  Never zero and never absent. The node schema requires this, and the
 *  planner refuses a weld that authors none -- zero would put the tip in the
 *  part. It used to be absent on every rails weld: the detected seam carries
 *  `process.wire.stickout_mm`, but `process` is not one of the keys copied
 *  onto the geometry, so it never reached the node and every seam came back
 *  from the planner refused. */
function standoffOf(
  overrideProcess: Record<string, unknown>,
  process: Record<string, unknown>,
): ScalarFunction {
  for (const source of [overrideProcess, process]) {
    const authored = source.standoff_mm;
    if (isRecord(authored)) return structuredClone(authored) as unknown as ScalarFunction;
    const wire = isRecord(source.wire) ? source.wire : null;
    const stickout = wire?.stickout_mm;
    if (typeof stickout === "number" && Number.isFinite(stickout) && stickout > 0) {
      return { kind: "constant", value: stickout } as unknown as ScalarFunction;
    }
  }
  return { kind: "constant", value: DEFAULT_TORCH_STANDOFF_MM } as unknown as ScalarFunction;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

export function seamWeldNodeFor(entry: RailsEntryLike, sha256: string): WeldNode {
  const seam = entry.seam as Record<string, unknown>;
  const payload: RailsWeldPayload = entry.payload ?? { id: entry.id, seam_ids: [entry.seam.id], preset_id: null, overrides: {} };
  const overrides = payload.overrides ?? {};
  const seamCurve = seam.curve;
  const curve = isRecord(seamCurve) && !("from_weld_joint" in seamCurve)
    ? seamCurve
    : (entry.joint.weld_line as unknown as Record<string, unknown>);
  const seamOrientation = isRecord(seam.orientation) ? seam.orientation : {};
  const overrideOrientation = isRecord(overrides.orientation) ? overrides.orientation : {};
  const orientation = {
    work_angle_deg: { kind: "constant", value: 0 },
    travel_angle_deg: { kind: "constant", value: 10 },
    ...seamOrientation,
    ...overrideOrientation,
  } as SeamGeometry["orientation"];
  const geometry: SeamGeometry = {
    kind: "seam",
    id: entry.seam.id,
    curve: structuredClone(curve) as unknown as SeamGeometry["curve"],
    span: { ...entry.span },
    direction: (overrides.direction as "forward" | "reverse" | undefined) ?? (entry.seam.direction as "forward" | "reverse"),
    reference: structuredClone(entry.seam.reference) as Record<string, unknown>,
    orientation: structuredClone(orientation),
  };
  for (const key of SEAM_KEYS) {
    if (seam[key] !== undefined) (geometry as unknown as Record<string, unknown>)[key] = structuredClone(seam[key]);
  }
  const process = isRecord(seam.process) ? seam.process : {};
  const overrideProcess = isRecord(overrides.process) ? overrides.process : {};
  const speed = (overrideProcess.travel_speed_mm_s ?? process.travel_speed_mm_s) as ScalarFunction | undefined;
  const weave = isRecord(overrideProcess.weave) ? overrideProcess.weave : isRecord(process.weave) ? process.weave : { shape: "none" };
  return {
    id: entry.id,
    op: "weld",
    enabled: true,
    label: entry.seam.label || entry.seam.id,
    comment: entry.note,
    row_hash: `pending:${entry.id}`,
    geometry,
    // The fallback mirrors the weld planner's DEFAULT_PROCESS (4 mm/s since
    // 2026-09-07: the cell's measured wrist rate binds at 8); a seam normally
    // carries its own.
    travel_speed_mm_s: speed ? structuredClone(speed) : { kind: "constant", value: 4 },
    standoff_mm: standoffOf(overrideProcess, process),
    weave: structuredClone(weave) as WeldNode["weave"],
    weld_parameters: {},
    extensions: {
      [RAILS_EXTENSION]: {
        payload: structuredClone(payload),
        joint_id: entry.joint.id,
        sha256,
        // Inlined here for the same reason the curve is: the document has to
        // resolve its own `weld_joint_ref`, and the session's joints doc is
        // gone by the time a saved program is reopened.
        joint: jointForDocument(entry.joint as unknown as Record<string, unknown>),
      },
    },
  };
}

/** The weld planner's weld-list entry, rebuilt from the node the operator
 *  edits: span, direction, orientation and speed come from the node. */
export function railsPayloadOf(node: ProgramV2Node): RailsWeldPayload | null {
  if (!isWeld(node) || node.geometry.kind !== "seam") return null;
  const provenance = railsProvenance(node);
  const base: RailsWeldPayload = provenance?.payload ?? { id: node.id, seam_ids: [node.geometry.id], preset_id: null, overrides: {} };
  const overrides: Record<string, unknown> = structuredClone(base.overrides ?? {});
  overrides.orientation = {
    ...(isRecord(overrides.orientation) ? overrides.orientation : {}),
    work_angle_deg: node.geometry.orientation.work_angle_deg,
    travel_angle_deg: node.geometry.orientation.travel_angle_deg,
  };
  if (node.geometry.direction) overrides.direction = node.geometry.direction;
  const process = isRecord(overrides.process) ? overrides.process : {};
  overrides.process = { ...process, travel_speed_mm_s: node.travel_speed_mm_s, weave: node.weave };
  return {
    ...base,
    id: node.id,
    seam_ids: [node.geometry.id],
    label: node.label,
    span: node.geometry.span ? { start_s: node.geometry.span.start_s ?? 0, end_s: node.geometry.span.end_s ?? 1 } : null,
    overrides,
  };
}

export const isRailsWeld = (node: ProgramV2Node): node is WeldNode => isWeld(node) && node.geometry.kind === "seam";

/** The joint a rails weld carries, or null. `workpiece.weld_joints[]` is
 *  assembled from these, so a seam's `weld_joint_ref` resolves inside the
 *  document it is written in. */
export function railsJointOf(node: ProgramV2Node): Record<string, unknown> | null {
  if (!isRailsWeld(node)) return null;
  return railsProvenance(node)?.joint ?? null;
}
