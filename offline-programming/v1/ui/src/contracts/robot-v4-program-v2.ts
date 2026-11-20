/**
 * `robot.v4.program.v2`: the authored robot program shared by the classic
 * flow and the rails flow, and the successor of `robot.v4.program.v1`.
 *
 * One row per authored node. Taught moves carry a destination and its original
 * observation; solved/sample trajectories remain separate plan artifacts.
 * Weld blocks retain explicit approach/transit/retract nodes. Taught moves may
 * appear before, after or between complete weld blocks. Events do not move.
 *
 * Schema: steamdeck/real/v4/contracts/programs/robot.v4.program.v2.schema.json.
 * This file is the TypeScript half of the contract, hand-rolled in the idiom
 * of robot-v4-program.ts because the UI has no JSON Schema library: every
 * rule the schema states is restated here, closed objects included. Twins:
 * steamdeck/real/v4/contracts/programs/program_v2_test.go and
 * weld_planner/v1/python/weldplan/program_v2.py. The spreadsheet, persistence and planners share this document.
 */

import {
  ROBOT_V4_JOINT_ORDER,
  sortCanonical,
  type RobotV4ProgramState,
  type RobotV4Units,
  type RobotV4Vec3,
} from "./robot-v4-program.ts";

export const ROBOT_V4_PROGRAM_V2_SCHEMA = "robot.v4.program.v2" as const;
export const ROBOT_V4_PROGRAM_V2_MAX_NODES = 4096 as const;

export const PROGRAM_V2_MOTION_OPS = ["weld", "approach", "transit", "retract", "home", "move"] as const;
export const PROGRAM_V2_FREESPACE_OPS = ["approach", "transit", "retract", "home"] as const;
export const PROGRAM_V2_EVENT_OPS = ["io", "dwell"] as const;

/** Solved quantities, banned anywhere. Exact key match, recursive. */
export const PROGRAM_V2_FORBIDDEN_KEYS = new Set([
  "joints", "joint_positions_rad", "target_joints_rad", "q_rad", "qd_rad_s",
  "target_tcp_m", "tcp_pose_m", "trajectory", "planned_path", "accepted_plan",
  "preloaded_plan", "waypoints", "rows",
]);

/** Seam and classic vocabulary on a weld; a path under a freespace node. */
export const PROGRAM_V2_FREESPACE_FORBIDDEN_KEYS = new Set([
  "knots", "knots_u", "times_s", "control_points_xyz_m", "segments", "samples",
  "points", "poses", "rpy_rad", "path",
]);

const POLICY_KEYS = ["clearance_min_mm", "speed_scale", "torch_policy", "positioner_policy"] as const;
const COMMON_KEYS = ["id", "op", "enabled", "label", "comment", "row_hash", "extensions"];
const FREESPACE_KEYS = [...COMMON_KEYS, "depart_standoff", "arrive_standoff", "overrides"];
const NODE_KEYS: Record<ProgramV2Op, Set<string>> = {
  weld: new Set([...COMMON_KEYS, "geometry", "travel_speed_mm_s", "standoff_mm", "weave", "weld_preset_id", "weld_parameters"]),
  approach: new Set(FREESPACE_KEYS),
  transit: new Set(FREESPACE_KEYS),
  retract: new Set(FREESPACE_KEYS),
  home: new Set(FREESPACE_KEYS),
  dwell: new Set([...COMMON_KEYS, "duration_s"]),
  io: new Set([...COMMON_KEYS, "channel", "state", "action", "duration_s", "timeout_s", "stable_s"]),
  move: new Set([...COMMON_KEYS, "motion", "target_space", "target", "via", "speed_scale", "speed_mm_s", "acceleration_scale", "constant_tcp_speed", "capture", "weld_transition", "weld_start_delay_s", "weld_stop_delay_s"]),
};
const SHA256_HEX = /^[0-9a-f]{64}$/;
const DIGEST = /^sha256:[0-9a-f]{64}$/;
const RFC3339 = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?(Z|[+-]\d{2}:\d{2})$/;

export type ProgramV2Op = "weld" | "approach" | "transit" | "retract" | "home" | "dwell" | "io" | "move";
export type ProgramV2TorchPolicy = "free" | "hold_last" | "torch_down";
export type ProgramV2PositionerPolicy = "free" | "hold";

export type ScalarFunction =
  | { kind: "constant"; value: number }
  | { kind: "piecewise_linear"; knots: [number, number][] }
  | { kind: "bspline"; degree: number; knots_u: number[]; control_values: number[] }
  | { kind: "samples"; s: number[]; values: number[] };

export interface CurveSegment {
  id?: string;
  degree: number;
  control_points_xyz_m: RobotV4Vec3[];
  weights?: number[];
  knots_u: number[];
  analytic?: Record<string, unknown>;
  length_m?: number;
  continuity_to_next?: "C0" | "G1" | "C1" | "C2";
  source_edge_id?: string;
}

export interface Curve {
  segments: CurveSegment[];
  closed?: boolean;
  length_m?: number;
}

/** The weld planner's seam, minus `process`; see the schema's vendored $defs/seam. */
export interface SeamGeometry {
  kind: "seam";
  id: string;
  label?: string;
  weld_joint_ref?: string | null;
  origin?: Record<string, unknown>;
  pass?: Record<string, unknown>;
  curve: Curve;
  span?: { start_s?: number; end_s?: number };
  direction?: "forward" | "reverse";
  reference: Record<string, unknown>;
  orientation: {
    work_angle_deg: ScalarFunction;
    travel_angle_deg: ScalarFunction;
    spin?: Record<string, unknown>;
    limits?: Record<string, unknown>;
  };
  /** What the angle search was given, beside the profiles it chose.
   *  Provenance: `orientation` is the authority. Absent on a seam nobody
   *  searched. */
  search_provenance?: SearchProvenance;
  weld_geometry?: Record<string, unknown>;
  lead?: { lead_in_mm?: number; lead_out_mm?: number };
  tolerance?: Record<string, unknown>;
  sampling_hint?: Record<string, unknown>;
}

export interface SearchProvenance {
  /** The preset these values are compared against in the editor, as a label.
   *  The values are the authority, so a preset edited later changes nothing. */
  preset_id?: string;
  work_deviation_deg: number;
  travel_deviation_deg: number;
  standoff_cost_deg_per_mm: number;
  work_bin_deg: number;
  travel_bin_deg: number;
  knot_spacing_mm: number;
  sample_step_mm: number;
  standoff_mm: number;
  intersector: { engine: "bvh" | "occt"; deflection_mm?: number; dilation_mm?: number };
}

export interface PosedPoint {
  xyz_m: RobotV4Vec3;
  rpy_rad: RobotV4Vec3;
}

/**
 * The classic flow's weld: chord-decimated control points, each carrying the
 * TCP orientation as the authoring tool baked it. The RPY is AUTHORITY; a
 * planner never re-derives it from `torch_angles_deg`, which is provenance.
 */
export interface PosedPolylineGeometry {
  kind: "posed_polyline";
  frame_id: string;
  closed?: boolean;
  points: PosedPoint[];
  torch_angles_deg?: { work: number; travel: number; spin: number };
}

/** How the path passes through a hand-placed pose (see ManualPoseGeometry). */
export type ManualPoseContinuity = "C0" | "C1" | "C2";

export interface ManualPosePoint {
  /** The work point the torch aims at; the path runs through it. */
  xyz_m: RobotV4Vec3;
  /** Torch orientation, URDF fixed-axis. Tool +Z points from the gun into
   *  the work; the contact tip sits `standoff_mm` back along -Z. */
  rpy_rad: RobotV4Vec3;
  /** Ignored on the first and last point. */
  continuity: ManualPoseContinuity;
}

/**
 * A weld authored as torch poses placed by hand (the Waypoint weld tool),
 * kept in that form so it stays editable. The points and the schema's rule
 * are the weld: chord-length cubic Hermite segments whose tangent is the
 * one-sided chord at a C0 point, the unit-chord bisector at a C1 point and
 * the cubic-spline unknown at a C2 point and the ends; orientation is the
 * shortest-arc slerp by each segment's arc-length fraction; every scalar
 * function on the node is over normalized arc length of that path. In the
 * workpiece frame, with consecutive points distinct.
 */
export interface ManualPoseGeometry {
  kind: "manual_pose_wp";
  frame_id: string;
  /** `spline` (the default) is the waypoint rule above; `arc` is the circle
   *  through exactly three non-collinear points, first through second to
   *  third, with every point's continuity ignored. */
  shape?: "spline" | "arc";
  points: ManualPosePoint[];
}

/** The shortest chord two consecutive manual_pose_wp points may have: a
 *  micrometre. Below it the chord has no direction to be continuous with and
 *  the arc-length parameter collapses (the schema's rule). */
export const MANUAL_POSE_MIN_CHORD_M = 1e-6;

export interface Standoff {
  direction: "torch_axis" | "workpiece_z" | RobotV4Vec3;
  distance_mm: number;
}

export interface FreespacePolicy {
  clearance_min_mm: number;
  speed_scale: number;
  torch_policy: ProgramV2TorchPolicy;
  positioner_policy: ProgramV2PositionerPolicy;
}

interface NodeCommon {
  id: string;
  enabled: boolean;
  label: string;
  comment: string;
  row_hash: string;
  extensions?: Record<string, unknown>;
}

export interface WeldNode extends NodeCommon {
  op: "weld";
  geometry: SeamGeometry | PosedPolylineGeometry | ManualPoseGeometry;
  travel_speed_mm_s: ScalarFunction;
  standoff_mm: ScalarFunction;
  weave: { shape: string; [key: string]: unknown };
  /** The weld preset this node's process and orientation started from, as a
   *  label the editor compares against. The values here are the authority. */
  weld_preset_id?: string;
  /** Reserved. Must be empty in program.v2. */
  weld_parameters: Record<string, never>;
}

export interface FreespaceNode extends NodeCommon {
  op: "approach" | "transit" | "retract" | "home";
  depart_standoff?: Standoff;
  arrive_standoff?: Standoff;
  overrides?: Partial<FreespacePolicy>;
}

export interface DwellNode extends NodeCommon {
  op: "dwell";
  duration_s: number;
}

export interface IoNode extends NodeCommon {
  action?: "set" | "pulse" | "wait" | "monitor" | "end_monitor";
  duration_s?: number;
  timeout_s?: number;
  stable_s?: number;
  op: "io";
  channel: string;
  state: boolean;
}

/** An authored destination, not a sampled trajectory. Joint values retain the
 * taught configuration; Cartesian moves retain all six components of TCP pose. */
export interface TeachPose extends PosedPoint {
  frame_id: "world";
  joint_names: string[];
  joint_values_rad: number[];
}

export interface MoveNode extends NodeCommon {
  weld_transition?: "none" | "start" | "stop";
  weld_start_delay_s?: number;
  weld_stop_delay_s?: number;
  op: "move";
  motion: "joint" | "linear" | "circular";
  /** Cartesian edits resolve IK at planning; omitted keeps recorded joint authority. */
  target_space?: "joint" | "cartesian";
  target: TeachPose;
  via?: TeachPose;
  speed_scale: number;
  speed_mm_s: number;
  /** Independent joint acceleration limit; legacy nodes inherit speed_scale. */
  acceleration_scale?: number;
  /** MoveL/MoveC cruise at TCP speed, with ramps; defaults to true. */
  constant_tcp_speed?: boolean;
  capture: {
    source: "machine" | "preview";
    observed_at: string;
    model_id: string;
    robot_description_sha256: string;
    machine_planning_calibration_sha256?: string;
    cell_id?: string;
    pose: TeachPose;
  };
}

export type ProgramV2Node = WeldNode | FreespaceNode | DwellNode | IoNode | MoveNode;

export interface ProgramV2Input {
  kind: "project" | "weld_program" | "cad" | "cell" | "tool";
  ref: string;
  sha256: string;
}

export interface AssetRef {
  id: string;
  sha256: string;
}

/** The robot a program is for: the model, and the identity of its
 *  description (`sha256:<hex>`, docs/ROBOT-DESCRIPTION-CONTRACT.md). The
 *  all-zero identity is a producer that had no description to pin to. */
export interface RobotRef {
  model_id: string;
  robot_description_sha256: string;
}

export interface RobotV4ProgramV2 {
  schema: typeof ROBOT_V4_PROGRAM_V2_SCHEMA;
  program_id: string;
  program_name: string;
  program_revision: number;
  base_program_revision: number;
  program_digest?: string;
  program_state: RobotV4ProgramState;
  execution_mode: "simulation" | "dry_run" | "production";
  created_utc: string;
  units: RobotV4Units;
  producer: {
    component: "offline-programming-v1" | "weld-planner-v1";
    source_revision: string;
    inputs: ProgramV2Input[];
  };
  source?: Record<string, unknown>;
  workpiece: {
    frame_id: string;
    solids?: Record<string, unknown>[];
    /** The joints a seam's `weld_joint_ref` resolves against. */
    weld_joints?: Record<string, unknown>[];
  };
  placement: {
    cell_id: string;
    anchor_xyz_m: RobotV4Vec3;
    offset_xyz_m: RobotV4Vec3;
    rpy_rad: RobotV4Vec3;
  };
  robot: RobotRef;
  tool?: AssetRef;
  /** Whole-program motion speed multiplier, applied after planning. Default 1. */
  speed_scale?: number;
  defaults: FreespacePolicy;
  nodes: ProgramV2Node[];
  planner_owns: string[];
  extensions?: Record<string, unknown>;
}

export function parseRobotV4ProgramV2(value: unknown): RobotV4ProgramV2 {
  assertRobotV4ProgramV2(value);
  return structuredClone(value);
}

/**
 * Every problem, as sentences, in the style of the Python twin's
 * `problems()`. Empty means the document is accepted.
 */
export function programV2Problems(value: unknown): string[] {
  try {
    assertRobotV4ProgramV2(value);
    return [];
  } catch (error) {
    return [error instanceof Error ? error.message : String(error)];
  }
}

export function assertRobotV4ProgramV2(value: unknown): asserts value is RobotV4ProgramV2 {
  const doc = record(value, "program");
  equal(doc.schema, ROBOT_V4_PROGRAM_V2_SCHEMA, "program.schema");
  nonEmptyString(doc.program_id, "program.program_id");
  nonEmptyString(doc.program_name, "program.program_name");
  if (doc.speed_scale !== undefined && (typeof doc.speed_scale !== "number" ||
      !(doc.speed_scale >= 0.01 && doc.speed_scale <= 1))) {
    throw new Error("program.speed_scale must be in [0.01, 1]");
  }
  positiveInteger(doc.program_revision, "program.program_revision");
  nonNegativeInteger(doc.base_program_revision, "program.base_program_revision");
  if (doc.program_digest !== undefined && !(typeof doc.program_digest === "string" && DIGEST.test(doc.program_digest))) {
    throw new Error("program.program_digest must be a sha256 digest");
  }
  oneOf(doc.program_state, [
    "draft", "saved_unplanned", "needs_plan", "planning", "planned", "accepted_simulation",
    "accepted_physical", "execution_ready", "running", "completed", "stopped", "faulted",
  ], "program.program_state");
  oneOf(doc.execution_mode, ["simulation", "dry_run", "production"], "program.execution_mode");
  if (typeof doc.created_utc !== "string" || !RFC3339.test(doc.created_utc) || Number.isNaN(Date.parse(doc.created_utc))) {
    throw new Error("program.created_utc must be an RFC 3339 timestamp");
  }
  const units = record(doc.units, "program.units");
  equal(units.length, "m", "program.units.length");
  equal(units.angle, "rad", "program.units.angle");
  equal(units.time, "s", "program.units.time");
  if (!Array.isArray(units.joint_order) || units.joint_order.length !== ROBOT_V4_JOINT_ORDER.length ||
      units.joint_order.some((joint, index) => joint !== ROBOT_V4_JOINT_ORDER[index])) {
    throw new Error("program.units.joint_order must be canonical J1-J9");
  }
  assertProducer(doc.producer);
  const workpiece = record(doc.workpiece, "program.workpiece");
  nonEmptyString(workpiece.frame_id, "program.workpiece.frame_id");
  const placement = record(doc.placement, "program.placement");
  nonEmptyString(placement.cell_id, "program.placement.cell_id");
  for (const key of ["anchor_xyz_m", "offset_xyz_m", "rpy_rad"]) {
    tuple(placement[key], 3, `program.placement.${key}`);
  }
  assertRobotRef(doc.robot, "program.robot");
  if ((doc.robot as RobotRef).model_id !== placement.cell_id) {
    throw new Error(`program.placement.cell_id names ${String(placement.cell_id)} and the program is for `
      + `${(doc.robot as RobotRef).model_id}; the offsets are a record in one description's work frame`);
  }
  if (doc.tool !== undefined) assertAssetRef(doc.tool, "program.tool");
  const defaults = record(doc.defaults, "program.defaults");
  for (const key of POLICY_KEYS) {
    if (!(key in defaults)) throw new Error(`program.defaults.${key} is required`);
  }
  assertPolicy(defaults, "program.defaults");
  if (!Array.isArray(doc.planner_owns) || doc.planner_owns.length === 0) {
    throw new Error("program.planner_owns must state at least one absence");
  }
  doc.planner_owns.forEach((sentence, index) => nonEmptyString(sentence, `program.planner_owns[${index}]`));
  forbiddenKeys(doc, "program", false);
  forbiddenKeys(defaults, "program.defaults", true);

  if (!Array.isArray(doc.nodes) || doc.nodes.length < 1 || doc.nodes.length > ROBOT_V4_PROGRAM_V2_MAX_NODES) {
    throw new Error(`program.nodes must contain 1-${ROBOT_V4_PROGRAM_V2_MAX_NODES} entries`);
  }
  const ids = new Set<string>();
  // The joint index a seam's weld_joint_ref resolves against. A duplicated id
  // is reported rather than silently shadowed.
  const joints = new Map<string, Record<string, unknown>>();
  const jointHolder = isRecord(doc.workpiece) ? doc.workpiece : {};
  if (Array.isArray(jointHolder.weld_joints)) {
    for (const raw of jointHolder.weld_joints) {
      if (!isRecord(raw)) continue;
      const jid = raw.id;
      if (typeof jid !== "string" || jid === "") continue;
      if (joints.has(jid)) throw new Error(`workpiece.weld_joints has two joints with id ${JSON.stringify(jid)}`);
      joints.set(jid, raw);
    }
  }
  const motion: { id: string; op: ProgramV2Op }[] = [];
  doc.nodes.forEach((raw, index) => {
    const node = record(raw, `program.nodes[${index}]`);
    nonEmptyString(node.id, `program.nodes[${index}].id`);
    const id = String(node.id);
    if (ids.has(id)) throw new Error(`node id ${JSON.stringify(id)} is used twice; one row, one id`);
    ids.add(id);
    const op = node.op as ProgramV2Op;
    const allowed = NODE_KEYS[op];
    if (!allowed) throw new Error(`node ${JSON.stringify(id)} has unknown op ${String(node.op)}`);
    for (const key of Object.keys(node)) {
      if (!allowed.has(key)) {
        throw new Error(`node ${JSON.stringify(id)} carries ${JSON.stringify(key)}, which is not a field of a ${op} node`);
      }
    }
    if (typeof node.enabled !== "boolean") throw new Error(`node ${JSON.stringify(id)} enabled must be boolean`);
    for (const key of ["label", "comment"]) {
      if (typeof node[key] !== "string") throw new Error(`node ${JSON.stringify(id)} ${key} must be a string`);
    }
    nonEmptyString(node.row_hash, `node ${JSON.stringify(id)} row_hash`);
    switch (op) {
      case "move":
        assertMoveNode(node, id);
        if (node.weld_transition !== undefined) oneOf(node.weld_transition,["none","start","stop"],`move ${id} weld transition`);
        for (const key of ["weld_start_delay_s", "weld_stop_delay_s"] as const) {
          const v = node[key];
          if (v !== undefined && (typeof v !== "number" || !Number.isFinite(v) || v < 0 || v > 30)) throw new Error(`move ${id}: ${key} must be 0–30 seconds`);
        }
        break;
      case "weld":
        assertWeldNode(node, id, joints, workpiece.frame_id);
        break;
      case "approach": case "transit": case "retract": case "home":
        assertFreespaceNode(node, id, op);
        break;
      case "dwell":
        if (typeof node.duration_s !== "number" || !Number.isFinite(node.duration_s) || node.duration_s < 0) {
          throw new Error(`dwell ${JSON.stringify(id)} needs a non-negative duration_s`);
        }
        break;
      case "io":
        nonEmptyString(node.channel, `io ${JSON.stringify(id)} channel`);
        if (typeof node.state !== "boolean") throw new Error(`io ${JSON.stringify(id)} state must be boolean`);
        if (node.action !== undefined) oneOf(node.action,["set","pulse","wait","monitor","end_monitor"],`io ${id} action`);
        for (const key of ["duration_s","timeout_s","stable_s"]) if (node[key] !== undefined &&
          (typeof node[key] !== "number" || !Number.isFinite(node[key]) || Number(node[key]) < (key === "stable_s" ? 0 : 0.001) || Number(node[key]) > 3600))
            throw new Error(`io ${id} ${key} is outside its allowed range`);
        if (node.action === "pulse" && (node.state !== true || node.duration_s === undefined)) throw new Error(`io ${id} pulse needs ON and a duration`);
        if (node.action === "wait" && (node.timeout_s === undefined || Number(node.stable_s ?? 0) > Number(node.timeout_s))) throw new Error(`io ${id} wait needs a timeout at least as long as stability`);
        break;
    }
    if (op !== "dwell" && op !== "io") motion.push({ id, op });
  });
  assertSingleLine(motion);
}

/** The single line: home? approach weld (transit weld)* retract home?. */
function assertSingleLine(motion: { id: string; op: ProgramV2Op }[]): void {
  const problems = programV2LineProblems(motion);
  if (problems.length) throw new Error(problems[0]);
}

/**
 * Every way the motion nodes fail to form the single line, as sentences.
 * Exported so an editor can show all of them at once and disable the
 * control that would create one, instead of learning about it on Save.
 */
export function programV2LineProblems(motion: { id: string; op: ProgramV2Op }[]): string[] {
  const problems: string[] = [];
  if (motion.length === 0) return ["the program has no motion nodes"];
  motion.forEach((current, k) => {
    if (current.op === "home" && k !== 0 && k !== motion.length - 1) {
      problems.push(`home ${JSON.stringify(current.id)} sits in the middle of the program; home may only start or end it`);
    }
  });
  const fail = (text: string): void => { problems.push(text); };
  motion.forEach((current, k) => {
    const prev = k > 0 ? motion[k - 1]?.op : undefined;
    const next = motion[k + 1]?.op;
    const id = JSON.stringify(current.id);
    switch (current.op) {
      case "weld":
        if (prev !== "approach" && prev !== "transit") {
          fail(`weld ${id} is not preceded by an approach or transit${
            prev === undefined || prev === "home" ? " (an approach is missing)" : ""}`);
        }
        if (next !== "retract" && next !== "transit") {
          fail(`weld ${id} is not followed by a retract or transit${
            next === "weld" ? " (a transit is missing before the next weld)" : ""}`);
        }
        break;
      case "approach":
        if (next !== "weld") fail(`approach ${id} is not followed by a weld`);
        if (prev !== undefined && prev !== "home" && prev !== "move" && prev !== "retract") {
          fail(`approach ${id} follows ${prev}; only a home or nothing may precede it`);
        }
        break;
      case "retract":
        if (prev !== "weld") fail(`retract ${id} is not preceded by a weld`);
        if (next !== undefined && next !== "home" && next !== "move" && next !== "approach") {
          fail(`retract ${id} is followed by ${next}; only a home or nothing may follow it`);
        }
        break;
      case "transit":
        if (prev !== "weld" || next !== "weld") fail(`transit ${id} must sit between two welds`);
        break;
      case "home":
        break;
    }
  });
  return problems;
}

function assertTeachPose(value: unknown, path: string): void {
  const pose = record(value, path);
  equal(pose.frame_id, "world", `${path}.frame_id`);
  for (const key of ["xyz_m", "rpy_rad"]) {
    const vector = pose[key];
    if (!Array.isArray(vector) || vector.length !== 3 || !vector.every(v => typeof v === "number" && Number.isFinite(v))) {
      throw new Error(`${path}.${key} needs three finite coordinates`);
    }
  }
  const names = pose.joint_names, values = pose.joint_values_rad;
  if (!Array.isArray(names) || !names.length || names.length > 9 || names.some(n => typeof n !== "string" || !n) || new Set(names).size !== names.length ||
      !Array.isArray(values) || values.length !== names.length || values.some(v => typeof v !== "number" || !Number.isFinite(v))) {
    throw new Error(`${path} needs unique joint names and matching finite values`);
  }
  if (Object.keys(pose).some(key => !["frame_id", "xyz_m", "rpy_rad", "joint_names", "joint_values_rad"].includes(key))) throw new Error(`${path} has an unknown field`);
}

function assertMoveNode(node: Record<string, unknown>, id: string): void {
  const path = `move ${JSON.stringify(id)}`;
  oneOf(node.motion, ["joint", "linear", "circular"], `${path}.motion`);
  assertTeachPose(node.target, `${path}.target`);
  if (node.via !== undefined) assertTeachPose(node.via, `${path}.via`);
  if (node.target_space !== undefined) oneOf(node.target_space, ["joint", "cartesian"], `${path}.target_space`);
  if (typeof node.speed_scale !== "number" || !(node.speed_scale > 0 && node.speed_scale <= 1)) throw new Error(`${path}.speed_scale must be in (0, 1]`);
  if (typeof node.speed_mm_s !== "number" || !Number.isFinite(node.speed_mm_s) || node.speed_mm_s <= 0) throw new Error(`${path}.speed_mm_s must be positive`);
  if (node.constant_tcp_speed !== undefined && typeof node.constant_tcp_speed !== "boolean") throw new Error(`${path}.constant_tcp_speed must be boolean`);
  if (node.acceleration_scale !== undefined && (typeof node.acceleration_scale !== "number" ||
      !(node.acceleration_scale > 0 && node.acceleration_scale <= 1))) throw new Error(`${path}.acceleration_scale must be in (0, 1]`);
  const capture = record(node.capture, `${path}.capture`);
  oneOf(capture.source, ["machine", "preview"], `${path}.capture.source`);
  if (typeof capture.observed_at !== "string" || !RFC3339.test(capture.observed_at) || Number.isNaN(Date.parse(capture.observed_at))) throw new Error(`${path}.capture.observed_at must be an RFC 3339 timestamp`);
  assertRobotRef(capture, `${path}.capture`);
  if (capture.machine_planning_calibration_sha256 !== undefined && !/^sha256:[0-9a-f]{64}$/.test(String(capture.machine_planning_calibration_sha256))) throw new Error(`${path}.capture has an invalid calibration identity`);
  assertTeachPose(capture.pose, `${path}.capture.pose`);
  if (capture.source === "machine") nonEmptyString(capture.cell_id, `${path}.capture.cell_id`);
  if (Object.keys(capture).some(key => !["source", "observed_at", "model_id", "robot_description_sha256", "machine_planning_calibration_sha256", "cell_id", "pose"].includes(key))) throw new Error(`${path}.capture has an unknown field`);
}

function assertProducer(value: unknown): void {
  const producer = record(value, "program.producer");
  oneOf(producer.component, ["offline-programming-v1", "weld-planner-v1"], "program.producer.component");
  nonEmptyString(producer.source_revision, "program.producer.source_revision");
  if (!Array.isArray(producer.inputs) || producer.inputs.length === 0) {
    throw new Error("program.producer.inputs must name at least one input");
  }
  producer.inputs.forEach((raw, index) => {
    const input = record(raw, `program.producer.inputs[${index}]`);
    oneOf(input.kind, ["project", "weld_program", "cad", "cell", "tool"], `program.producer.inputs[${index}].kind`);
    nonEmptyString(input.ref, `program.producer.inputs[${index}].ref`);
    sha256Hex(input.sha256, `program.producer.inputs[${index}].sha256`);
  });
}

function assertRobotRef(value: unknown, path: string): void {
  const ref = record(value, path);
  nonEmptyString(ref.model_id, `${path}.model_id`);
  if (typeof ref.robot_description_sha256 !== "string" || !/^sha256:[0-9a-f]{64}$/.test(ref.robot_description_sha256)) {
    throw new Error(`${path}.robot_description_sha256 must be a description identity (sha256:<64 hex>)`);
  }
}

function assertAssetRef(value: unknown, path: string): void {
  const ref = record(value, path);
  nonEmptyString(ref.id, `${path}.id`);
  sha256Hex(ref.sha256, `${path}.sha256`);
}

function assertPolicy(policy: Record<string, unknown>, path: string): void {
  for (const key of Object.keys(policy)) {
    if (!(POLICY_KEYS as readonly string[]).includes(key)) throw new Error(`${path}.${key} is not a defaults key`);
  }
  if ("clearance_min_mm" in policy) finiteNonNegative(policy.clearance_min_mm, `${path}.clearance_min_mm`);
  if ("speed_scale" in policy) {
    const scale = policy.speed_scale;
    if (typeof scale !== "number" || !(scale > 0 && scale <= 1)) throw new Error(`${path}.speed_scale must be in (0, 1]`);
  }
  if ("torch_policy" in policy) oneOf(policy.torch_policy, ["free", "hold_last", "torch_down"], `${path}.torch_policy`);
  if ("positioner_policy" in policy) oneOf(policy.positioner_policy, ["free", "hold"], `${path}.positioner_policy`);
}

function assertWeldNode(
  node: Record<string, unknown>,
  id: string,
  joints: Map<string, Record<string, unknown>>,
  workpieceFrame: unknown,
): void {
  const label = `weld ${JSON.stringify(id)}`;
  if (!("weld_parameters" in node)) throw new Error(`${label} has no weld_parameters; the reserved block must be present`);
  const params = record(node.weld_parameters, `${label} weld_parameters`);
  if (Object.keys(params).length !== 0) {
    throw new Error(`${label} fills weld_parameters (${Object.keys(params).join(", ")}); the block is reserved and must be empty in program.v2`);
  }
  assertScalarFunction(node.travel_speed_mm_s, `${label} travel_speed_mm_s`);
  // Never defaulted here. An absent standoff is not zero -- zero puts the tip
  // in the part -- and a validator that filled one in would hand the planner
  // a number nobody authored.
  assertScalarFunction(node.standoff_mm, `${label} standoff_mm`);
  assertPositiveScalar(node.standoff_mm, `${label} standoff_mm`);
  const weave = record(node.weave, `${label} weave`);
  nonEmptyString(weave.shape, `${label} weave.shape`);
  const geometry = record(node.geometry, `${label} geometry`);
  if (geometry.kind === "seam") {
    assertSeamGeometry(geometry, label, joints);
  } else if (geometry.kind === "posed_polyline") {
    nonEmptyString(geometry.frame_id, `${label} posed_polyline frame_id`);
    if (!Array.isArray(geometry.points) || geometry.points.length < 2) {
      throw new Error(`${label} posed_polyline needs at least two points`);
    }
    geometry.points.forEach((raw, index) => {
      const point = record(raw, `${label} point ${index}`);
      tuple(point.xyz_m, 3, `${label} point ${index} xyz_m`);
      tuple(point.rpy_rad, 3, `${label} point ${index} rpy_rad`);
    });
  } else if (geometry.kind === "manual_pose_wp") {
    assertManualPoseGeometry(geometry, label, workpieceFrame);
  } else {
    throw new Error(`${label} geometry kind ${String(geometry.kind)} is not seam, posed_polyline or manual_pose_wp`);
  }
}

function assertManualPoseGeometry(geometry: Record<string, unknown>, label: string, workpieceFrame: unknown): void {
  // The workpiece frame by definition: a hand-placed pose rides with the part.
  if (typeof geometry.frame_id !== "string" || geometry.frame_id === "" || geometry.frame_id !== workpieceFrame) {
    throw new Error(
      `${label} manual_pose_wp frame_id ${JSON.stringify(geometry.frame_id)} is not the workpiece frame ${JSON.stringify(workpieceFrame)}`,
    );
  }
  if (geometry.shape !== undefined && geometry.shape !== "spline" && geometry.shape !== "arc") {
    throw new Error(`${label} manual_pose_wp shape must be spline or arc`);
  }
  if (!Array.isArray(geometry.points) || geometry.points.length < 2) {
    throw new Error(`${label} manual_pose_wp needs at least two points`);
  }
  if (geometry.shape === "arc" && geometry.points.length !== 3) {
    throw new Error(`${label} manual_pose_wp arc needs exactly three points`);
  }
  let previous: number[] | null = null;
  geometry.points.forEach((raw, index) => {
    const point = record(raw, `${label} point ${index}`);
    tuple(point.xyz_m, 3, `${label} point ${index} xyz_m`);
    tuple(point.rpy_rad, 3, `${label} point ${index} rpy_rad`);
    if (!["C0", "C1", "C2"].includes(point.continuity as string)) {
      throw new Error(`${label} point ${index} continuity must be C0, C1 or C2`);
    }
    const xyz = point.xyz_m as number[];
    if (previous && Math.hypot(xyz[0]! - previous[0]!, xyz[1]! - previous[1]!, xyz[2]! - previous[2]!) <= MANUAL_POSE_MIN_CHORD_M) {
      throw new Error(`${label} points ${index - 1} and ${index} coincide; consecutive points must be distinct`);
    }
    previous = xyz;
  });
  if (geometry.shape === "arc") {
    const [a, b, c] = (geometry.points as Record<string, unknown>[]).map((point) => point.xyz_m as number[]);
    const ab = [b![0]! - a![0]!, b![1]! - a![1]!, b![2]! - a![2]!];
    const ac = [c![0]! - a![0]!, c![1]! - a![1]!, c![2]! - a![2]!];
    const chord = Math.hypot(ac[0]!, ac[1]!, ac[2]!);
    const area = Math.hypot(
      ab[1]! * ac[2]! - ab[2]! * ac[1]!,
      ab[2]! * ac[0]! - ab[0]! * ac[2]!,
      ab[0]! * ac[1]! - ab[1]! * ac[0]!,
    );
    if (chord <= MANUAL_POSE_MIN_CHORD_M || area / chord <= MANUAL_POSE_MIN_CHORD_M) {
      throw new Error(`${label} manual_pose_wp arc points lie on a line; no circle passes through them in order`);
    }
  }
}

/** A search that could not have moved what it says it was free to move. The
 *  window is the authored profile plus or minus the deviation and the bin is
 *  how finely it was walked, so a bin wider than the window leaves one value
 *  in it. Each number is fine alone; only the pair is wrong, which is why the
 *  schema cannot say it. */
function assertSearchProvenance(value: unknown, label: string): void {
  if (value === undefined) return;
  const block = record(value, `${label} search_provenance`);
  for (const axis of ["work", "travel"]) {
    const deviation = block[`${axis}_deviation_deg`];
    const binned = block[`${axis}_bin_deg`];
    if (typeof deviation !== "number" || typeof binned !== "number") continue;
    if (deviation > 0 && binned > 2 * deviation) {
      throw new Error(
        `${label} searched ${axis} over +/-${deviation} degrees in ${binned}-degree bins, ` +
          "which is one bin: the deviation names a freedom the bin spacing did not allow",
      );
    }
  }
}

function assertSeamGeometry(geometry: Record<string, unknown>, label: string, joints: Map<string, Record<string, unknown>>): void {
  for (const key of ["id", "reference", "orientation"]) {
    if (!(key in geometry)) throw new Error(`${label} seam geometry needs ${key}`);
  }
  assertSearchProvenance(geometry.search_provenance, label);
  // A named joint has to EXIST, curve or no curve. The reference resolves
  // inside this document because workpiece.weld_joints[] is here (D24).
  const ref = geometry.weld_joint_ref;
  const named = typeof ref === "string" && ref !== "";
  if (named && !joints.has(ref as string)) {
    throw new Error(`${label} names weld_joint_ref ${JSON.stringify(ref)}, which is not in workpiece.weld_joints`);
  }
  if (!isRecord(geometry.curve)) {
    // A seam may take its line from its joint instead of carrying one, which
    // is what the planner does: Seam.__init__ reads seam["curve"] or
    // joint["weld_line"] and refuses when neither is there.
    if (!named) {
      throw new Error(`${label} seam geometry has no inline curve and no weld_joint_ref to take one from`);
    }
    if (!isRecord(joints.get(ref as string)?.weld_line)) {
      throw new Error(`${label} has no curve and joint ${JSON.stringify(ref)} has no weld_line, so the seam has no line at all`);
    }
    return;
  }
  const curve = record(geometry.curve, `${label} curve (must be inline)`);
  if (!Array.isArray(curve.segments) || curve.segments.length === 0) throw new Error(`${label} curve has no segments`);
  curve.segments.forEach((raw, index) => {
    const segment = record(raw, `${label} curve segment ${index}`);
    const poles = segment.control_points_xyz_m;
    const knots = segment.knots_u;
    const degree = segment.degree;
    if (!Array.isArray(poles) || !Array.isArray(knots) || !Number.isInteger(degree)) {
      throw new Error(`${label} curve segment ${index} needs control_points_xyz_m, degree and knots_u`);
    }
    const n = poles.length;
    const p = Number(degree);
    if (knots.length !== n + p + 1) {
      throw new Error(`${label} curve segment ${index} knots_u has ${knots.length} entries, needs ${n} + ${p} + 1`);
    }
    const values = knots.map((knot) => Number(knot));
    for (let k = 1; k < values.length; k += 1) {
      if ((values[k] ?? NaN) < (values[k - 1] ?? NaN)) throw new Error(`${label} curve segment ${index} knots_u is not non-decreasing`);
    }
    for (let k = 0; k <= p; k += 1) {
      if (values[k] !== values[0] || values[values.length - 1 - k] !== values[values.length - 1]) {
        throw new Error(`${label} curve segment ${index} knots_u is not clamped`);
      }
    }
    if (Array.isArray(segment.weights) && segment.weights.length !== n) {
      throw new Error(`${label} curve segment ${index} has ${segment.weights.length} weights for ${n} poles`);
    }
  });
  const span = geometry.span;
  if (isRecord(span)) {
    const start = typeof span.start_s === "number" ? span.start_s : 0;
    const end = typeof span.end_s === "number" ? span.end_s : 1;
    if (start > end && curve.closed !== true) {
      throw new Error(`${label} span wraps through s=0, which is only valid on a closed curve`);
    }
  }
  const orientation = record(geometry.orientation, `${label} orientation`);
  for (const axis of ["work_angle_deg", "travel_angle_deg"]) {
    if (axis in orientation) assertScalarFunction(orientation[axis], `${label} ${axis}`);
  }
}

function assertScalarFunction(value: unknown, where: string): void {
  const fn = record(value, where);
  switch (fn.kind) {
    case "constant":
      if (typeof fn.value !== "number" || !Number.isFinite(fn.value)) throw new Error(`${where} constant needs a value`);
      break;
    case "piecewise_linear": {
      const knots = fn.knots;
      if (!Array.isArray(knots) || knots.length < 2) throw new Error(`${where} piecewise_linear needs at least two knots`);
      let previous = -1;
      knots.forEach((raw, index) => {
        if (!Array.isArray(raw) || raw.length !== 2) throw new Error(`${where} piecewise_linear knots must be [s, value] pairs`);
        const s = Number(raw[0]);
        if ((index === 0 && s !== 0) || (index === knots.length - 1 && s !== 1)) {
          throw new Error(`${where} piecewise_linear knots must run from s=0 to s=1`);
        }
        if (s <= previous) throw new Error(`${where} piecewise_linear knots must have strictly increasing s`);
        previous = s;
      });
      break;
    }
    case "samples":
      if (!Array.isArray(fn.s) || !Array.isArray(fn.values) || fn.s.length < 2 || fn.s.length !== fn.values.length) {
        throw new Error(`${where} samples needs matching s and values`);
      }
      break;
    case "bspline":
      if (!Array.isArray(fn.knots_u) || !Array.isArray(fn.control_values) || !Number.isInteger(fn.degree) ||
          fn.knots_u.length !== fn.control_values.length + Number(fn.degree) + 1) {
        throw new Error(`${where} bspline knot count does not match control_values + degree + 1`);
      }
      break;
    default:
      throw new Error(`${where} has unknown kind ${String(fn.kind)}`);
  }
}

/** Every value a standoff profile can take has to be positive: the number is a distance from the work, and zero or less is the tip inside the part. Checked here rather than left to the planner, because a document that says it is welding at 0 mm is wrong before anything is planned. */
function assertPositiveScalar(value: unknown, where: string): void {
  const fn = record(value, where);
  const values: unknown[] = fn.kind === "constant"
    ? [fn.value]
    : fn.kind === "piecewise_linear"
      ? (fn.knots as unknown[][]).map((knot) => knot[1])
      : fn.kind === "samples"
        ? (fn.values as unknown[])
        : (fn.control_values as unknown[]);
  for (const candidate of values) {
    if (typeof candidate !== "number" || !(candidate > 0)) {
      throw new Error(`${where} must be positive everywhere, got ${String(candidate)}`);
    }
  }
}

function assertFreespaceNode(node: Record<string, unknown>, id: string, op: string): void {
  const label = `node ${JSON.stringify(id)}`;
  forbiddenKeys(node, label, true);
  if ("overrides" in node) assertPolicy(record(node.overrides, `${label} overrides`), `${label} overrides`);
  for (const side of ["depart_standoff", "arrive_standoff"] as const) {
    if (!(side in node)) continue;
    if (side === "depart_standoff" && (op === "approach" || op === "home")) {
      throw new Error(`${op} ${JSON.stringify(id)} carries depart_standoff, but nothing is welded before it`);
    }
    if (side === "arrive_standoff" && (op === "retract" || op === "home")) {
      throw new Error(`${op} ${JSON.stringify(id)} carries arrive_standoff, but nothing is welded after it`);
    }
    const lift = record(node[side], `${label} ${side}`);
    finiteNonNegative(lift.distance_mm, `${label} ${side} distance_mm`);
    if (typeof lift.direction === "string") {
      oneOf(lift.direction, ["torch_axis", "workpiece_z"], `${label} ${side} direction`);
    } else {
      tuple(lift.direction, 3, `${label} ${side} direction`);
    }
  }
}

/**
 * Refuses any key that names a solved quantity. Exact match, recursive: a
 * substring match would trip on weld_joint_ref, a top-level-only match would
 * miss a path one object down, which is where a smuggled solution would sit.
 */
function forbiddenKeys(value: unknown, path: string, scoped: boolean): void {
  if (Array.isArray(value)) {
    value.forEach((child, index) => forbiddenKeys(child, `${path}[${index}]`, scoped));
    return;
  }
  if (!isRecord(value)) return;
  for (const [key, child] of Object.entries(value)) {
    if (PROGRAM_V2_FORBIDDEN_KEYS.has(key) || (scoped && PROGRAM_V2_FREESPACE_FORBIDDEN_KEYS.has(key))) {
      throw new Error(`${path} carries ${JSON.stringify(key)}, which is a solved quantity; program.v2 states intent only`);
    }
    forbiddenKeys(child, `${path}.${key}`, scoped);
  }
}

// ---------------------------------------------------------------- digests

/**
 * The same discipline as robot.v4.program.v1: a node's hash is over its
 * canonical JSON minus the hash itself, and the program digest is over the
 * canonical document minus the digest. Both use the v1 `sortCanonical`.
 */
export async function computeProgramV2NodeDigest(node: ProgramV2Node): Promise<string> {
  const value = structuredClone(node) as unknown as Record<string, unknown>;
  delete value.row_hash;
  return sha256(JSON.stringify(sortCanonical(value)));
}

export function canonicalProgramV2JSON(program: RobotV4ProgramV2): string {
  assertRobotV4ProgramV2(program);
  const value = structuredClone(program) as unknown as Record<string, unknown>;
  delete value.program_digest;
  return JSON.stringify(sortCanonical(value));
}

export async function computeProgramV2Digest(program: RobotV4ProgramV2): Promise<string> {
  return sha256(canonicalProgramV2JSON(program));
}

async function sha256(text: string): Promise<string> {
  const result = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return `sha256:${[...new Uint8Array(result)].map((byte) => byte.toString(16).padStart(2, "0")).join("")}`;
}

// ---------------------------------------------------------------- helpers

function record(value: unknown, path: string): Record<string, unknown> {
  if (!isRecord(value)) throw new Error(`${path} must be an object`);
  return value;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function nonEmptyString(value: unknown, path: string): void {
  if (typeof value !== "string" || !value.trim()) throw new Error(`${path} is required`);
}

function equal(actual: unknown, expected: unknown, path: string): void {
  if (actual !== expected) throw new Error(`${path} must be ${String(expected)}`);
}

function oneOf(value: unknown, options: readonly string[], path: string): void {
  if (typeof value !== "string" || !options.includes(value)) {
    throw new Error(`${path} is invalid: ${String(value)}`);
  }
}

function positiveInteger(value: unknown, path: string): void {
  if (!Number.isInteger(value) || Number(value) < 1) throw new Error(`${path} must be a positive integer`);
}

function nonNegativeInteger(value: unknown, path: string): void {
  if (!Number.isInteger(value) || Number(value) < 0) throw new Error(`${path} must be a non-negative integer`);
}

function finiteNonNegative(value: unknown, path: string): void {
  if (typeof value !== "number" || !Number.isFinite(value) || value < 0) {
    throw new Error(`${path} must be a non-negative finite number`);
  }
}

function sha256Hex(value: unknown, path: string): void {
  if (typeof value !== "string" || !SHA256_HEX.test(value)) {
    throw new Error(`${path} must be 64 hex characters; the contents stay out, the hash pins them`);
  }
}

function tuple(value: unknown, length: number, path: string): void {
  if (!Array.isArray(value) || value.length !== length ||
      value.some((item) => typeof item !== "number" || !Number.isFinite(item))) {
    throw new Error(`${path} must contain ${length} finite numbers`);
  }
}
