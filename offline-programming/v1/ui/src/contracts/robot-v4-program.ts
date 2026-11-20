export const ROBOT_V4_PROGRAM_SCHEMA_VERSION = 1 as const;
// The row bound is a memory-safety backstop for a controller-facing wire
// contract (P2.11, ledger L27), not an authoring budget. Sized on 2026-09-07
// for the planner's wires at the 10 ms certificate density on a cell limited
// to 0.09 rad/s: a two-seam bracket program is 8747 rows (one per knot), so
// 65536 rows is ~11 minutes of motion, about 7x that program; a full-fidelity
// wire document at 65536 rows measures ~85 MB. The former 4096 was >5x the
// worst real part at 0.2 s knots (exact Dog Box, 730 rows); the historical 64
// was a Steam Deck demo-fixture constant, never a controller limit. Mirrors
// maxItems in robot/v4/contracts/programs/robot.v4.program.v1.schema.json and
// maxRobotV4ProgramRows in offline-programming/v1/internal/planning.
export const ROBOT_V4_PROGRAM_MAX_ROWS = 65536 as const;
export const ROBOT_V4_CATALOG_SCHEMA = "robot.v4.program_catalog.v1" as const;
export const ROBOT_V4_ACCEPTED_PLAN_SCHEMA = "robot.v4.accepted-plan.v1" as const;
export const ROBOT_V4_JOINT_ORDER = [
  "J1",
  "J2",
  "J3",
  "J4",
  "J5",
  "J6",
  "J7",
  "J8",
  "J9",
] as const;

export type RobotV4JointVector = [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
];
export type RobotV4TcpPose = [number, number, number, number, number, number];
export type RobotV4Vec3 = [number, number, number];

export interface RobotV4Units {
  length: "m";
  angle: "rad";
  time: "s";
  joint_order: typeof ROBOT_V4_JOINT_ORDER;
}

export interface RobotV4ProgramDefaults {
  default_tool_frame?: string;
  default_work_frame?: string;
  default_move_type?: "line_tcp" | "curve_tcp" | "joint_ptp";
  default_tcp_velocity_m_s?: number;
  default_joint_velocity_rad_s?: number;
  default_accel_m_s2?: number;
  default_joint_accel_rad_s2?: number;
  default_blend_radius_m?: number;
  [key: string]: unknown;
}

export interface RobotV4SettingsPolicy {
  global_joint_speed_limits_rad_s?: RobotV4JointVector;
  global_joint_position_limits_rad?: RobotV4JointVector;
  singularity_protection_enabled?: boolean;
  singularity_behavior?: "avoid" | "warn" | "stop";
  singularity_warning_threshold?: number;
  singularity_hard_stop_threshold?: number;
  singularity_speed_limit_rad_s?: number;
  revision?: number;
  [key: string]: unknown;
}

export interface RobotV4ToolAxisConstraint {
  tool_frame: string;
  tip_link: string;
  world_frame: string;
  tool_axis_xyz: RobotV4Vec3;
  world_axis_xyz: RobotV4Vec3;
  offset_reference_axis_xyz: RobotV4Vec3;
  offset_deg: number;
  tolerance_minus_deg: number;
  tolerance_plus_deg: number;
  weight: number;
  [key: string]: unknown;
}

export interface RobotV4HarmoniousExternalAxis {
  joint: string;
  active_side: string;
  selector_joint: string;
  selector_rad: number;
  selector_tolerance_rad: number;
  center_rad: number;
  amplitude_rad: number;
  phase_deg: number;
  trajectory_source: string;
  work_frame: string;
  workplane_relative: boolean;
  workplane_pivot_m: RobotV4Vec3;
  workplane_axis_xyz: RobotV4Vec3;
  reference_rad: number;
  tool_axis_follows_workplane: boolean;
  [key: string]: unknown;
}

export interface RobotV4ProgramConstraints {
  tip_link?: string;
  tool_axis_constraint?: RobotV4ToolAxisConstraint;
  harmonious_external_axis?: RobotV4HarmoniousExternalAxis;
  [key: string]: unknown;
}

export type RobotV4ProgramState =
  | "draft"
  | "saved_unplanned"
  | "needs_plan"
  | "planning"
  | "planned"
  | "accepted_simulation"
  | "accepted_physical"
  | "execution_ready"
  | "running"
  | "completed"
  | "stopped"
  | "faulted";

export type RobotV4ProgramRowType =
  | "motion"
  | "command"
  | "comment"
  | "capture"
  | "calibration"
  | "io";

export interface RobotV4ProgramRow {
  row_id: string;
  row_index?: number;
  row_number?: number;
  revision: number;
  row_hash: string;
  enabled: boolean;
  label?: string;
  row_type: RobotV4ProgramRowType;
  command?: string;
  comment?: string;
  loop_target_row_id?: string;
  loop_iterations?: number;
  return_home_before_repeat?: boolean;
  move_type?: "line_tcp" | "curve_tcp" | "joint_ptp" | "none";
  target_frame_type?: string;
  tool_frame?: string;
  work_frame?: string;
  fixture_side?: string;
  motion_phase?: string;
  calibration_section?: number;
  harmonious_phase_deg?: number;
  capture_source?: string;
  speed_source?: string;
  seam_id?: string;
  pass_id?: string;
  pass_role?: string;
  source_project_id?: string;
  source_project_revision?: number;
  source_state?: "cad_derived" | "teach_modified" | "manual" | "imported";
  target_tcp_m?: RobotV4TcpPose;
  nominal_tcp_m?: RobotV4Vec3;
  target_joints_rad?: RobotV4JointVector;
  current_tcp_pose?: RobotV4TcpPose;
  current_joints_rad?: RobotV4JointVector;
  positioner_targets_rad?: RobotV4JointVector;
  segment_meters?: number;
  tcp_velocity_m_s?: number;
  joint_velocity_rad_s?: number;
  accel_m_s2?: number;
  joint_accel_rad_s2?: number;
  blend_radius_m?: number;
  fine_stop?: boolean;
  playback_step_ms?: number;
  weld_enable?: boolean;
  arc_state?: "on" | "off";
  wire_feed_m_per_min?: number;
  voltage_v?: number;
  amperage_a?: number;
  weld_profile_id?: number;
  weave_profile_id?: number;
  weld_process?: string;
  revpi_io_slot?: number;
  revpi_io_action?: string;
  revpi_io_pending?: boolean;
  planner_status?: string;
  validation_state?: string;
  collision_check_state?: string;
  limit_check_state?: string;
  planned_path_revision?: number;
  planned_sample_count?: number;
  extensions?: Record<string, unknown>;
  [key: string]: unknown;
}

export interface RobotV4Program {
  schema_version: typeof ROBOT_V4_PROGRAM_SCHEMA_VERSION;
  program_id: string;
  program_name: string;
  part_name?: string;
  part_number?: string;
  part_revision?: string;
  "robotic-cell-type"?: string;
  robotic_cell_type?: string;
  /** The robot description this program is written against, by directory
   *  name. WHICH ones exist is the server's answer (`domain/robots.ts`), so
   *  this contract checks the shape of the name and nothing more: a wire
   *  format that listed the robots would have to be edited to add one. */
  robot_model: string;
  cell_config_revision?: string;
  program_state: RobotV4ProgramState;
  execution_mode: "simulation" | "dry_run" | "production";
  units: RobotV4Units;
  program_revision: number;
  base_program_revision: number;
  program_digest?: string;
  source_project_id?: string;
  source_project_revision?: number;
  edit_source?: "offline_programming" | "steamdeck" | "motion_server" | "import";
  edited_at?: string;
  defaults?: RobotV4ProgramDefaults;
  settings?: RobotV4SettingsPolicy;
  constraints?: RobotV4ProgramConstraints;
  planner_status?: "not_planned" | "needs_plan" | "planning" | "planned" | "stale" | "rejected";
  validation_state?: "unknown" | "valid" | "invalid";
  collision_check_state?: string;
  limit_check_state?: string;
  planned_path_revision?: number;
  planned_sample_count?: number;
  rows: RobotV4ProgramRow[];
  extensions?: Record<string, unknown>;
  [key: string]: unknown;
}

export interface RobotV4ProgramCatalog {
  schema: typeof ROBOT_V4_CATALOG_SCHEMA;
  owner: "motion-server";
  client?: "offline-programming" | "steamdeck" | "motion-server";
  autosave?: boolean;
  catalog_revision: number;
  updated_at?: string;
  updated_at_unix?: number;
  programs: RobotV4Program[];
  [key: string]: unknown;
}

export interface RobotV4AcceptedPlanSample {
  sample_index: number;
  row_id: string;
  row_index: number;
  time_s: number;
  tcp_pose_m: RobotV4TcpPose;
  joint_positions_rad: RobotV4JointVector;
  weld_on: boolean;
  path_parent?: string;
  path_segment?: string;
  [key: string]: unknown;
}

export interface RobotV4AcceptedPlan {
  schema: typeof ROBOT_V4_ACCEPTED_PLAN_SCHEMA;
  authority_classification: "simulation_only" | "physical_target";
  plan_id: string;
  plan_revision: number;
  trajectory_digest: string;
  program_id: string;
  program_revision: number;
  program_digest: string;
  planner: {
    owner: "motion-server";
    backend: string;
    trajectory_method: string;
    build_digest: string;
    request_subject?: string;
    [key: string]: unknown;
  };
  target: {
    target_id: string;
    robot_model: string;
    joint_order: typeof ROBOT_V4_JOINT_ORDER;
    manifest_revision: string;
    cell_config_revision: string;
    calibration_digest: string;
    urdf_digest: string;
    mesh_digest?: string;
    collision_world_digest: string;
    settings_revision: number;
    tool_frame?: string;
    work_frame?: string;
    [key: string]: unknown;
  };
  evidence: {
    planning_contract_accepted: boolean;
    collision: "clear" | "collision" | "unavailable";
    singularity: "clear" | "warning" | "hard_stop" | "unavailable";
    limits: "clear" | "outside_limits" | "unavailable";
    start_pose?: "converged" | "mismatch" | "unavailable";
    runtime_playable: boolean;
    collision_points?: Record<string, unknown>[];
    singularity_points?: Record<string, unknown>[];
    [key: string]: unknown;
  };
  request_id?: string;
  trajectory: RobotV4AcceptedPlanSample[];
  created_at: string;
  [key: string]: unknown;
}

const PROGRAM_STATES = new Set<RobotV4ProgramState>([
  "draft",
  "saved_unplanned",
  "needs_plan",
  "planning",
  "planned",
  "accepted_simulation",
  "accepted_physical",
  "execution_ready",
  "running",
  "completed",
  "stopped",
  "faulted",
]);
const ROW_TYPES = new Set<RobotV4ProgramRowType>([
  "motion",
  "command",
  "comment",
  "capture",
  "calibration",
  "io",
]);
const DIGEST_PATTERN = /^sha256:[0-9a-f]{64}$/;

export function parseRobotV4Program(value: unknown): RobotV4Program {
  assertRobotV4Program(value);
  return structuredClone(value);
}

/** A description directory name: what `robot_description/<model>/` is called.
 *  Checked as a name, never against a list -- whether the robot EXISTS is
 *  answered by the server that holds the directory. */
function robotModelName(value: unknown, path: string): void {
  if (typeof value !== "string" || !/^[A-Za-z0-9][A-Za-z0-9_-]*$/.test(value)) {
    throw new Error(`${path} is not a robot description name: ${String(value)}`);
  }
}

export function assertRobotV4Program(value: unknown): asserts value is RobotV4Program {
  const program = record(value, "program");
  equal(program.schema_version, ROBOT_V4_PROGRAM_SCHEMA_VERSION, "program.schema_version");
  nonEmptyString(program.program_id, "program.program_id");
  nonEmptyString(program.program_name, "program.program_name");
  robotModelName(program.robot_model, "program.robot_model");
  if (!PROGRAM_STATES.has(program.program_state as RobotV4ProgramState)) {
    throw new Error(`program.program_state is unsupported: ${String(program.program_state)}`);
  }
  if (!["simulation", "dry_run", "production"].includes(String(program.execution_mode))) {
    throw new Error(`program.execution_mode is unsupported: ${String(program.execution_mode)}`);
  }
  if (typeof program["robotic-cell-type"] !== "string" &&
      typeof program.robotic_cell_type !== "string") {
    throw new Error("program robotic cell type is required");
  }
  positiveInteger(program.program_revision, "program.program_revision");
  nonNegativeInteger(program.base_program_revision, "program.base_program_revision");
  if (program.program_digest !== undefined) digest(program.program_digest, "program.program_digest");
  assertUnits(program.units);
  if (
    !Array.isArray(program.rows) ||
    program.rows.length < 1 ||
    program.rows.length > ROBOT_V4_PROGRAM_MAX_ROWS
  ) {
    throw new Error(`program.rows must contain 1-${ROBOT_V4_PROGRAM_MAX_ROWS} rows`);
  }
  const ids = new Set<string>();
  program.rows.forEach((row, index) => {
    assertRobotV4ProgramRow(row, index);
    const rowId = (row as RobotV4ProgramRow).row_id;
    if (ids.has(rowId)) throw new Error(`program.rows contains duplicate row_id ${rowId}`);
    ids.add(rowId);
  });
  for (const row of program.rows as RobotV4ProgramRow[]) {
    if (row.row_type === "command" && row.command === "LOOP" &&
        !ids.has(row.loop_target_row_id ?? "")) {
      throw new Error(`loop row ${row.row_id} targets missing row ${row.loop_target_row_id ?? ""}`);
    }
  }
}

export function parseRobotV4Catalog(value: unknown): RobotV4ProgramCatalog {
  assertRobotV4Catalog(value);
  return structuredClone(value);
}

export function assertRobotV4Catalog(value: unknown): asserts value is RobotV4ProgramCatalog {
  const catalog = record(value, "catalog");
  equal(catalog.schema, ROBOT_V4_CATALOG_SCHEMA, "catalog.schema");
  equal(catalog.owner, "motion-server", "catalog.owner");
  nonNegativeInteger(catalog.catalog_revision, "catalog.catalog_revision");
  if (!Array.isArray(catalog.programs)) throw new Error("catalog.programs must be an array");
  const ids = new Set<string>();
  catalog.programs.forEach((program) => {
    assertRobotV4Program(program);
    const id = (program as RobotV4Program).program_id;
    if (ids.has(id)) throw new Error(`catalog contains duplicate program_id ${id}`);
    ids.add(id);
  });
}

export function parseRobotV4AcceptedPlan(value: unknown): RobotV4AcceptedPlan {
  assertRobotV4AcceptedPlan(value);
  return structuredClone(value);
}

export function assertRobotV4AcceptedPlan(value: unknown): asserts value is RobotV4AcceptedPlan {
  const plan = record(value, "accepted plan");
  equal(plan.schema, ROBOT_V4_ACCEPTED_PLAN_SCHEMA, "accepted plan.schema");
  if (!["simulation_only", "physical_target"].includes(String(plan.authority_classification))) {
    throw new Error("accepted plan.authority_classification is invalid");
  }
  nonEmptyString(plan.plan_id, "accepted plan.plan_id");
  positiveInteger(plan.plan_revision, "accepted plan.plan_revision");
  digest(plan.trajectory_digest, "accepted plan.trajectory_digest");
  nonEmptyString(plan.program_id, "accepted plan.program_id");
  positiveInteger(plan.program_revision, "accepted plan.program_revision");
  digest(plan.program_digest, "accepted plan.program_digest");
  const planner = record(plan.planner, "accepted plan.planner");
  equal(planner.owner, "motion-server", "accepted plan.planner.owner");
  nonEmptyString(planner.backend, "accepted plan.planner.backend");
  nonEmptyString(planner.trajectory_method, "accepted plan.planner.trajectory_method");
  digest(planner.build_digest, "accepted plan.planner.build_digest");
  const target = record(plan.target, "accepted plan.target");
  nonEmptyString(target.target_id, "accepted plan.target.target_id");
  nonEmptyString(target.robot_model, "accepted plan.target.robot_model");
  assertJointOrder(target.joint_order, "accepted plan.target.joint_order");
  for (const key of [
    "manifest_revision",
    "cell_config_revision",
  ]) nonEmptyString(target[key], `accepted plan.target.${key}`);
  for (const key of [
    "calibration_digest",
    "urdf_digest",
    "collision_world_digest",
  ]) digest(target[key], `accepted plan.target.${key}`);
  positiveInteger(target.settings_revision, "accepted plan.target.settings_revision");
  const evidence = record(plan.evidence, "accepted plan.evidence");
  if (typeof evidence.planning_contract_accepted !== "boolean" ||
      typeof evidence.runtime_playable !== "boolean") {
    throw new Error("accepted plan evidence booleans are required");
  }
  if (!["clear", "collision", "unavailable"].includes(String(evidence.collision)) ||
      !["clear", "warning", "hard_stop", "unavailable"].includes(String(evidence.singularity)) ||
      !["clear", "outside_limits", "unavailable"].includes(String(evidence.limits))) {
    throw new Error("accepted plan evidence state is invalid");
  }
  if (!Array.isArray(plan.trajectory) || plan.trajectory.length < 2) {
    throw new Error("accepted plan.trajectory must contain at least two samples");
  }
  let priorTime = -1;
  plan.trajectory.forEach((sample, index) => {
    const point = record(sample, `accepted plan.trajectory[${index}]`);
    nonNegativeInteger(point.sample_index, `accepted plan.trajectory[${index}].sample_index`);
    equal(point.sample_index, index, `accepted plan.trajectory[${index}].sample_index`);
    nonEmptyString(point.row_id, `accepted plan.trajectory[${index}].row_id`);
    nonNegativeInteger(point.row_index, `accepted plan.trajectory[${index}].row_index`);
    finiteNonNegative(point.time_s, `accepted plan.trajectory[${index}].time_s`);
    if (Number(point.time_s) < priorTime) throw new Error("accepted plan trajectory time must be monotonic");
    priorTime = Number(point.time_s);
    tuple(point.tcp_pose_m, 6, `accepted plan.trajectory[${index}].tcp_pose_m`);
    tuple(point.joint_positions_rad, 9, `accepted plan.trajectory[${index}].joint_positions_rad`);
    if (typeof point.weld_on !== "boolean") {
      throw new Error(`accepted plan.trajectory[${index}].weld_on must be boolean`);
    }
  });
  nonEmptyString(plan.created_at, "accepted plan.created_at");
}

export function canonicalProgramJSON(program: RobotV4Program): string {
  assertRobotV4Program(program);
  return JSON.stringify(sortCanonical(stripProgramTransient(program)));
}

export async function computeProgramDigest(program: RobotV4Program): Promise<string> {
  const data = new TextEncoder().encode(canonicalProgramJSON(program));
  const result = await crypto.subtle.digest("SHA-256", data);
  return `sha256:${[...new Uint8Array(result)].map((byte) => byte.toString(16).padStart(2, "0")).join("")}`;
}

export async function finalizeRobotV4Program(
  value: RobotV4Program,
): Promise<RobotV4Program> {
  const program = parseRobotV4Program(value);
  delete program.program_digest;
  program.rows = await Promise.all(program.rows.map(async (row) => ({
    ...row,
    row_hash: await computeRowDigest(row),
  })));
  program.program_digest = await computeProgramDigest(program);
  return parseRobotV4Program(program);
}

export async function reviseRobotV4Program(
  current: RobotV4Program,
  editSource: NonNullable<RobotV4Program["edit_source"]>,
  mutate: (draft: RobotV4Program) => void,
  editedAt = new Date().toISOString(),
): Promise<RobotV4Program> {
  assertRobotV4Program(current);
  const beforeRows = new Map(current.rows.map((row) => [
    row.row_id,
    JSON.stringify(sortCanonical(stripRowTransient(row))),
  ]));
  const next = structuredClone(current);
  mutate(next);
  next.base_program_revision = current.program_revision;
  next.program_revision = current.program_revision + 1;
  next.edit_source = editSource;
  next.edited_at = editedAt;
  next.program_state = "needs_plan";
  next.planner_status = "stale";
  next.validation_state = "unknown";
  next.collision_check_state = "not_run";
  next.limit_check_state = "not_run";
  next.planned_path_revision = 0;
  next.planned_sample_count = 0;
  delete next.program_digest;
  next.rows = await Promise.all(next.rows.map(async (row, index) => {
    const updated = structuredClone(row);
    updated.row_index = index;
    updated.row_number = index + 1;
    const after = JSON.stringify(sortCanonical(stripRowTransient(updated)));
    if (beforeRows.get(updated.row_id) !== after) {
      updated.revision = next.program_revision;
      updated.planner_status = "stale";
      updated.validation_state = "unknown";
      updated.collision_check_state = "not_run";
      updated.limit_check_state = "not_run";
      updated.planned_path_revision = 0;
      updated.planned_sample_count = 0;
      updated.row_hash = await computeRowDigest(updated);
    }
    return updated;
  }));
  assertRobotV4Program(next);
  next.program_digest = await computeProgramDigest(next);
  return next;
}

async function computeRowDigest(row: RobotV4ProgramRow): Promise<string> {
  const data = new TextEncoder().encode(JSON.stringify(sortCanonical(stripRowTransient(row))));
  const result = await crypto.subtle.digest("SHA-256", data);
  return `sha256:${[...new Uint8Array(result)].map((byte) => byte.toString(16).padStart(2, "0")).join("")}`;
}

function assertRobotV4ProgramRow(value: unknown, index: number): asserts value is RobotV4ProgramRow {
  const row = record(value, `program.rows[${index}]`);
  nonEmptyString(row.row_id, `program.rows[${index}].row_id`);
  nonNegativeInteger(row.revision, `program.rows[${index}].revision`);
  nonEmptyString(row.row_hash, `program.rows[${index}].row_hash`);
  if (typeof row.enabled !== "boolean") throw new Error(`program.rows[${index}].enabled must be boolean`);
  if (!ROW_TYPES.has(row.row_type as RobotV4ProgramRowType)) {
    throw new Error(`program.rows[${index}].row_type is unsupported: ${String(row.row_type)}`);
  }
  if (row.row_type === "motion") {
    if (!["line_tcp", "curve_tcp", "joint_ptp"].includes(String(row.move_type))) {
      throw new Error(`motion row ${String(row.row_id)} has invalid move_type`);
    }
    for (const key of ["target_frame_type", "tool_frame", "work_frame"]) {
      nonEmptyString(row[key], `program.rows[${index}].${key}`);
    }
    tuple(row.target_tcp_m, 6, `program.rows[${index}].target_tcp_m`);
    tuple(row.target_joints_rad, 9, `program.rows[${index}].target_joints_rad`);
    for (const key of [
      "tcp_velocity_m_s",
      "joint_velocity_rad_s",
      "accel_m_s2",
      "joint_accel_rad_s2",
      "blend_radius_m",
    ]) finiteNonNegative(row[key], `program.rows[${index}].${key}`);
    if (typeof row.fine_stop !== "boolean") {
      throw new Error(`program.rows[${index}].fine_stop must be boolean`);
    }
  }
  if (row.row_type === "command" && row.command === "LOOP") {
    nonEmptyString(row.loop_target_row_id, `program.rows[${index}].loop_target_row_id`);
    nonNegativeInteger(row.loop_iterations, `program.rows[${index}].loop_iterations`);
    if (typeof row.return_home_before_repeat !== "boolean") {
      throw new Error(`program.rows[${index}].return_home_before_repeat must be boolean`);
    }
  }
}

function assertUnits(value: unknown): asserts value is RobotV4Units {
  const units = record(value, "program.units");
  equal(units.length, "m", "program.units.length");
  equal(units.angle, "rad", "program.units.angle");
  equal(units.time, "s", "program.units.time");
  assertJointOrder(units.joint_order, "program.units.joint_order");
}

function assertJointOrder(value: unknown, path: string): void {
  if (!Array.isArray(value) || value.length !== ROBOT_V4_JOINT_ORDER.length ||
      value.some((joint, index) => joint !== ROBOT_V4_JOINT_ORDER[index])) {
    throw new Error(`${path} must be canonical J1-J9`);
  }
}

function stripProgramTransient(program: RobotV4Program): Record<string, unknown> {
  const value = structuredClone(program) as Record<string, unknown>;
  for (const key of [
    "program_digest",
    "base_program_revision",
    "edited_at",
    "planner_status",
    "validation_state",
    "collision_check_state",
    "limit_check_state",
    "planned_path_revision",
    "planned_sample_count",
    "planned_path",
    "audit",
  ]) delete value[key];
  value.rows = program.rows.map(stripRowTransient);
  return value;
}

function stripRowTransient(row: RobotV4ProgramRow): Record<string, unknown> {
  const value = structuredClone(row) as Record<string, unknown>;
  for (const key of [
    "row_index",
    "row_number",
    "row_hash",
    "planner_status",
    "validation_state",
    "collision_check_state",
    "limit_check_state",
    "planned_path_revision",
    "planned_sample_count",
    "current_tcp_pose",
    "current_joints_rad",
    "revpi_io_pending",
  ]) delete value[key];
  return value;
}

/**
 * Key-sorted deep copy that refuses non-finite numbers, so a digest is over
 * one canonical byte string. Exported because robot.v4.program.v2 digests the
 * same way: one definition, or the two digests drift.
 */
export function sortCanonical(value: unknown): unknown {
  if (Array.isArray(value)) return value.map(sortCanonical);
  if (!isRecord(value)) {
    if (typeof value === "number" && !Number.isFinite(value)) {
      throw new Error("canonical program JSON cannot contain non-finite numbers");
    }
    return value;
  }
  return Object.fromEntries(
    Object.keys(value).sort().map((key) => [key, sortCanonical(value[key])]),
  );
}

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

function positiveInteger(value: unknown, path: string): void {
  if (!Number.isInteger(value) || Number(value) < 1) throw new Error(`${path} must be a positive integer`);
}

function nonNegativeInteger(value: unknown, path: string): void {
  if (!Number.isInteger(value) || Number(value) < 0) throw new Error(`${path} must be a non-negative integer`);
}

function finiteNonNegative(value: unknown, path: string): void {
  if (!Number.isFinite(value) || Number(value) < 0) throw new Error(`${path} must be a non-negative finite number`);
}

function digest(value: unknown, path: string): void {
  if (typeof value !== "string" || !DIGEST_PATTERN.test(value)) {
    throw new Error(`${path} must be a sha256 digest`);
  }
}

function tuple(value: unknown, length: number, path: string): void {
  if (!Array.isArray(value) || value.length !== length ||
      value.some((item) => typeof item !== "number" || !Number.isFinite(item))) {
    throw new Error(`${path} must contain ${length} finite numbers`);
  }
}
