import {
  ROBOT_V4_JOINT_ORDER,
  parseRobotV4Catalog,
  parseRobotV4Program,
  type RobotV4AcceptedPlan,
  type RobotV4Program,
  type RobotV4ProgramCatalog,
  type RobotV4ProgramRow,
} from "../contracts/robot-v4-program.ts";
import {
  JOINT_ORDER,
  cloneProject,
  createProject,
  missingPlan,
  zeroJoints,
  type AuthoredProgramRow,
  type OfflineProgrammingProject,
  type PlanTrajectorySample,
  type RobotModel,
  type RowMove,
  type RowPhase,
  type TcpPose,
} from "../domain/project.ts";
import {
  normalizeWeavePattern,
  rowTorchSettings,
  rowWeaveSettings,
} from "../domain/weld.ts";

const WELD_EXTENSION_KEY = "gradient_weld_v1";

export type RobotV4PlanRequest = RobotV4Program & {
  request_source: "offline-programming-v1";
  project_revision: number;
  joint_order: string[];
  target_profile_ref: string | null;
  execution_mode: "simulation";
};

export type RobotV4PlanRequestRow = RobotV4ProgramRow;

export interface RobotV4ProgramOptions {
  executionMode?: RobotV4Program["execution_mode"];
  editSource?: NonNullable<RobotV4Program["edit_source"]>;
}

export interface RobotV4ProgramImportOptions {
  project?: OfflineProgrammingProject;
  catalogRevision?: number | null;
  timestamp?: string;
}

export function toRobotV4Program(
  project: OfflineProgrammingProject,
  options: RobotV4ProgramOptions = {},
): RobotV4Program {
  const wireExtensions = wireRecord(project.program.extensions);
  const program: RobotV4Program = {
    ...wireExtensions,
    schema_version: 1,
    program_id: project.program.programId || project.identity.id,
    program_name: project.identity.name,
    part_name: project.program.partName,
    part_number: project.program.partNumber,
    part_revision: project.program.partRevision,
    "robotic-cell-type": project.program.roboticCellType || project.robot.model,
    robot_model: project.robot.model,
    cell_config_revision: project.program.cellConfigRevision,
    program_state: project.program.programState,
    execution_mode: options.executionMode ?? project.program.executionMode,
    units: {
      length: "m",
      angle: "rad",
      time: "s",
      joint_order: ROBOT_V4_JOINT_ORDER,
    },
    program_revision: project.program.programRevision,
    base_program_revision: project.program.baseProgramRevision,
    source_project_id: project.identity.id,
    source_project_revision: project.identity.revision,
    edit_source: options.editSource ?? project.program.editSource,
    edited_at: project.identity.updatedAt,
    defaults: structuredClone(project.program.defaults),
    settings: structuredClone(project.program.settings),
    constraints: structuredClone(project.program.constraints),
    planner_status: planStatus(project),
    validation_state: project.plan.state === "server_validated" ? "valid" : "unknown",
    collision_check_state: project.plan.evidence.collision === "clear" ? "clear" : "not_run",
    limit_check_state: project.plan.evidence.limits === "clear" ? "clear" : "not_run",
    planned_path_revision: project.plan.state === "server_validated"
      ? project.program.programRevision
      : 0,
    planned_sample_count: project.plan.state === "server_validated"
      ? project.plan.trajectory.length
      : 0,
    rows: project.rows.map((row, index) => toRobotV4Row(project, row, index)),
    extensions: explicitExtensions(project.program.extensions),
  };
  if (project.program.programDigest) program.program_digest = project.program.programDigest;
  return parseRobotV4Program(program);
}

export function toRobotV4PlanRequest(project: OfflineProgrammingProject): RobotV4PlanRequest {
  return {
    ...toRobotV4Program(project, { executionMode: "simulation" }),
    request_source: "offline-programming-v1",
    project_revision: project.identity.revision,
    joint_order: [...JOINT_ORDER],
    target_profile_ref: project.target.profileId,
    execution_mode: "simulation",
  };
}

/**
 * Convert the immutable Motion Server accepted-plan trajectory into the
 * workstation playback contract. The renderer must consume this artifact,
 * rather than rebuilding or interpolating a second browser-owned trajectory.
 */
export function acceptedPlanTrajectory(
  project: OfflineProgrammingProject,
  accepted: RobotV4AcceptedPlan,
): PlanTrajectorySample[] {
  if (accepted.target.robot_model !== project.robot.model) {
    throw new Error("accepted plan robot model does not match the open program");
  }
  return accepted.trajectory.map((sample) => {
    const row = project.rows[sample.row_index];
    if (!row || row.id !== sample.row_id) {
      throw new Error(
        `accepted sample ${sample.sample_index} does not match spreadsheet row identity`,
      );
    }
    return {
      index: sample.sample_index,
      rowIndex: sample.row_index,
      seamId: row.seamId || null,
      phase: row.phase,
      weldOn: sample.weld_on,
      timeS: sample.time_s,
      tcpPose: [...sample.tcp_pose_m],
      jointPositionsRad: [...sample.joint_positions_rad],
    };
  });
}

export function fromRobotV4Program(
  value: unknown,
  options: RobotV4ProgramImportOptions = {},
): OfflineProgrammingProject {
  const program = parseRobotV4Program(value);
  const timestamp = options.timestamp ?? new Date().toISOString();
  const sourceProjectId = nonempty(program.source_project_id)
    ? program.source_project_id
    : program.program_id;
  const project = options.project
    ? cloneProject(options.project)
    : createProject({
      id: sourceProjectId,
      name: program.program_name,
      robotModel: projectRobotModel(program.robot_model),
      now: timestamp,
      cad: {
        kind: "example",
        filename: program.part_name || "steamdeck-program",
        fingerprint: `catalog:${program.program_id}`,
        topologyModelId: `catalog:${program.program_id}`,
        sourceUnit: "m",
      },
    });

  if (options.project) {
    project.identity.revision += 1;
    project.identity.updatedAt = timestamp;
  }
  project.identity.name = program.program_name;
  project.robot.model = projectRobotModel(program.robot_model);
  project.robot.jointOrder = JOINT_ORDER;
  if (!options.project) {
    project.target = { mode: "offline_preview", profileId: null, manifestRef: null };
  }
  project.program = {
    schema: "robot.v4.program.v1",
    programId: program.program_id,
    programRevision: program.program_revision,
    // The project keeps the revision observed from the Motion Server as its
    // optimistic-concurrency baseline. The wire field records the base used
    // by the prior save and is therefore not the next editor token.
    baseProgramRevision: program.program_revision,
    programDigest: program.program_digest ?? null,
    catalogRevision: options.catalogRevision ?? null,
    partName: program.part_name ?? "",
    partNumber: program.part_number ?? "",
    partRevision: program.part_revision ?? "",
    roboticCellType: program["robotic-cell-type"] ?? program.robotic_cell_type ?? program.robot_model,
    cellConfigRevision: program.cell_config_revision ?? "",
    programState: program.program_state,
    executionMode: program.execution_mode,
    editSource: program.edit_source ?? "import",
    sourceProjectRevision: program.source_project_revision ?? project.identity.revision,
    defaults: structuredClone(program.defaults ?? {}),
    settings: structuredClone(program.settings ?? {}),
    constraints: structuredClone(program.constraints ?? {}),
    extensions: {
      ...structuredClone(program.extensions ?? {}),
      __wire: unknownProperties(program, PROGRAM_KEYS),
    },
  };
  project.rows = program.rows.map(fromRobotV4Row);
  project.plan = missingPlan();
  project.simulation.selectedRowId = project.rows[0]?.id ?? null;
  return project;
}

export function fromRobotV4Catalog(
  value: unknown,
  programId: string,
  options: Omit<RobotV4ProgramImportOptions, "catalogRevision"> = {},
): OfflineProgrammingProject {
  const catalog = parseRobotV4Catalog(value);
  const program = catalog.programs.find((candidate) => candidate.program_id === programId);
  if (!program) throw new Error(`Robot V4 catalog does not contain program ${programId}`);
  return fromRobotV4Program(program, {
    ...options,
    catalogRevision: catalog.catalog_revision,
  });
}

export function toRobotV4Catalog(
  projects: readonly OfflineProgrammingProject[],
  catalogRevision: number,
): RobotV4ProgramCatalog {
  return parseRobotV4Catalog({
    schema: "robot.v4.program_catalog.v1",
    owner: "motion-server",
    client: "offline-programming",
    autosave: false,
    catalog_revision: catalogRevision,
    programs: projects.map((project) => toRobotV4Program(project)),
  });
}

function toRobotV4Row(
  project: OfflineProgrammingProject,
  row: AuthoredProgramRow,
  index: number,
): RobotV4ProgramRow {
  const rowType = row.rowType ?? "motion";
  const result: RobotV4ProgramRow = {
    ...wireRecord(row.extensions),
    row_id: row.id,
    row_index: index,
    row_number: index + 1,
    revision: row.rowRevision ?? project.program.programRevision,
    row_hash: row.rowHash || `offline:${row.id}:${row.rowRevision ?? project.program.programRevision}`,
    enabled: row.enabled,
    label: row.label,
    row_type: rowType,
    command: row.command,
    extensions: explicitExtensions(row.extensions),
  };
  if (row.comment) result.comment = row.comment;
  if (rowType === "command" && row.command === "LOOP") {
    result.move_type = "none";
    result.loop_target_row_id = row.loopTargetRowId ?? "";
    result.loop_iterations = row.loopIterations ?? 1;
    result.return_home_before_repeat = row.returnHomeBeforeRepeat === true;
    return result;
  }
  if (rowType !== "motion") {
    if (row.captureSource) result.capture_source = row.captureSource;
    if (row.nominalTcpM) result.nominal_tcp_m = [...row.nominalTcpM];
    if (row.calibrationSection !== undefined) result.calibration_section = row.calibrationSection;
    if (row.revPiIOSlot !== null) result.revpi_io_slot = row.revPiIOSlot;
    result.revpi_io_action = row.revPiIOAction;
    result.revpi_io_pending = row.revPiIOPending === true;
    return result;
  }

  Object.assign(result, {
    move_type: databaseMove(row.moveType),
    target_frame_type: row.targetFrameType ??
      (row.moveType === "JOINT" ? "joint_positions" : "tcp_cartesian"),
    tool_frame: row.toolFrameId,
    work_frame: row.workFrameId,
    fixture_side: row.fixtureSide ?? "in_cell",
    motion_phase: row.phase,
    calibration_section: row.calibrationSection ?? 0,
    harmonious_phase_deg: row.harmoniousPhaseDeg ?? 0,
    capture_source: row.captureSource ?? row.source,
    speed_source: row.speedSource ?? "programmed",
    seam_id: row.seamId ?? "",
    pass_id: row.passId,
    pass_role: row.passRole,
    source_project_id: project.identity.id,
    source_project_revision: project.identity.revision,
    source_state: row.sourceState ?? (row.source === "cadquery" ? "cad_derived" : "manual"),
    target_tcp_m: [...row.targetTcpPose],
    target_joints_rad: [...row.targetJointsRad],
    current_tcp_pose: [...(row.currentTcpPose ?? row.targetTcpPose)],
    current_joints_rad: [...(row.currentJointsRad ?? row.targetJointsRad)],
    positioner_targets_rad: [...(row.positionerJointTargetsRad ?? zeroJoints(row.positionerTargetsRad))],
    tcp_velocity_m_s: row.tcpVelocityMps,
    joint_velocity_rad_s: row.jointVelocityRadS,
    accel_m_s2: row.accelerationMps2,
    joint_accel_rad_s2: row.jointAccelerationRadS2,
    blend_radius_m: row.blendRadiusM,
    fine_stop: row.fineStop,
    weld_enable: row.weldEnabled,
    arc_state: row.arcState,
    wire_feed_m_per_min: row.wireFeedMPerMin,
    voltage_v: row.voltageV,
    amperage_a: row.amperageA ?? 0,
    weld_profile_id: row.weldProfileId,
    weave_profile_id: row.weaveProfileId,
    weld_process: row.weldProcess,
    revpi_io_slot: row.revPiIOSlot ?? 0,
    revpi_io_action: row.revPiIOAction,
    revpi_io_pending: row.revPiIOPending === true,
    planner_status: row.plannerStatus ?? "not_planned",
    validation_state: row.validationState ?? "unknown",
    collision_check_state: row.collisionCheckState ?? "not_run",
    limit_check_state: row.limitCheckState ?? "not_run",
    planned_path_revision: row.plannedPathRevision ?? 0,
    planned_sample_count: row.plannedSampleCount ?? 0,
  } satisfies Partial<RobotV4ProgramRow>);
  if (row.nominalTcpM) result.nominal_tcp_m = [...row.nominalTcpM];
  // The Motion Server treats an absent playback_step_ms as 10 000 ms and
  // ignores the row velocities, so the imported clock must survive the
  // round-trip. Written only when present: rows authored here never had it,
  // and adding a default would change their digests.
  if (row.playbackStepMs !== undefined) {
    result.playback_step_ms = row.playbackStepMs;
  }
  // Torch orientation and weave strategy travel inside the row extensions so
  // the shared robot.v4.program.v1 schema stays byte-compatible for peers
  // that do not read them yet.
  const torch = rowTorchSettings(row);
  const weave = rowWeaveSettings(row);
  result.extensions = {
    ...(result.extensions ?? {}),
    [WELD_EXTENSION_KEY]: {
      torch_work_angle_deg: torch.workAngleDeg,
      torch_travel_angle_deg: torch.travelAngleDeg,
      torch_spin_angle_deg: torch.spinAngleDeg,
      weave_pattern: weave.pattern,
      weave_amplitude_mm: weave.amplitudeMm,
      weave_frequency_hz: weave.frequencyHz,
      weave_dwell_ms: weave.dwellMs,
    },
  };
  return result;
}

function fromRobotV4Row(row: RobotV4ProgramRow): AuthoredProgramRow {
  const targetTcp = tuple6(row.target_tcp_m);
  const targetJoints = zeroJoints(row.target_joints_rad);
  const positioner = zeroJoints(row.positioner_targets_rad);
  const storedExtensions = structuredClone(row.extensions ?? {});
  const weldExtension = isRecordValue(storedExtensions[WELD_EXTENSION_KEY])
    ? storedExtensions[WELD_EXTENSION_KEY] as Record<string, unknown>
    : {};
  // The typed row fields are the single source of truth; keep the packed
  // block out of the retained extensions so edits never diverge from it.
  delete storedExtensions[WELD_EXTENSION_KEY];
  return {
    id: row.row_id,
    enabled: row.enabled,
    label: row.label ?? row.row_id,
    rowType: row.row_type,
    rowRevision: row.revision,
    rowHash: row.row_hash,
    command: programCommand(row.command, row.row_type),
    comment: stringValue(row.comment),
    loopTargetRowId: nonempty(row.loop_target_row_id) ? row.loop_target_row_id : null,
    loopIterations: integer(row.loop_iterations, 0),
    returnHomeBeforeRepeat: row.return_home_before_repeat === true,
    moveType: authoredMove(row.move_type),
    phase: authoredPhase(row.motion_phase),
    targetFrameType: stringValue(row.target_frame_type, row.move_type === "joint_ptp" ? "joint" : "tcp_cartesian"),
    fixtureSide: stringValue(row.fixture_side, "in_cell"),
    calibrationSection: integer(row.calibration_section, 0),
    harmoniousPhaseDeg: finite(row.harmonious_phase_deg, 0),
    captureSource: stringValue(row.capture_source, "robot_v4"),
    speedSource: stringValue(row.speed_source, "programmed"),
    seamId: nonempty(row.seam_id) ? row.seam_id : null,
    passId: stringValue(row.pass_id),
    passRole: stringValue(row.pass_role, "none"),
    toolFrameId: stringValue(row.tool_frame, "end_effector"),
    workFrameId: stringValue(row.work_frame, "world"),
    fixtureId: null,
    targetTcpPose: targetTcp,
    targetJointsRad: targetJoints,
    positionerTargetsRad: [positioner[6], positioner[7], positioner[8]],
    positionerJointTargetsRad: positioner,
    nominalTcpM: row.nominal_tcp_m ? [...row.nominal_tcp_m] : null,
    currentTcpPose: tuple6(row.current_tcp_pose ?? row.target_tcp_m),
    currentJointsRad: zeroJoints(row.current_joints_rad ?? row.target_joints_rad),
    tcpVelocityMps: finite(row.tcp_velocity_m_s, 0.05),
    jointVelocityRadS: finite(row.joint_velocity_rad_s, 0.25),
    accelerationMps2: finite(row.accel_m_s2, 0.75),
    jointAccelerationRadS2: finite(row.joint_accel_rad_s2, 1),
    blendRadiusM: finite(row.blend_radius_m, 0),
    fineStop: row.fine_stop !== false,
    ...(typeof row.playback_step_ms === "number" &&
        Number.isFinite(row.playback_step_ms) && row.playback_step_ms > 0
      ? { playbackStepMs: row.playback_step_ms }
      : {}),
    weldEnabled: row.weld_enable === true,
    arcState: row.arc_state === "on" ? "on" : "off",
    wireFeedMPerMin: finite(row.wire_feed_m_per_min, 0),
    voltageV: finite(row.voltage_v, 0),
    amperageA: finite(row.amperage_a, 0),
    weldProfileId: integer(row.weld_profile_id, 0),
    weaveProfileId: integer(row.weave_profile_id, 0),
    weldProcess: stringValue(row.weld_process, "none"),
    torchWorkAngleDeg: finite(weldExtension.torch_work_angle_deg, 0),
    torchTravelAngleDeg: finite(weldExtension.torch_travel_angle_deg, 0),
    torchSpinAngleDeg: finite(weldExtension.torch_spin_angle_deg, 0),
    weavePattern: normalizeWeavePattern(weldExtension.weave_pattern),
    weaveAmplitudeMm: finite(weldExtension.weave_amplitude_mm, 2),
    weaveFrequencyHz: finite(weldExtension.weave_frequency_hz, 1),
    weaveDwellMs: finite(weldExtension.weave_dwell_ms, 0),
    revPiIOSlot: row.revpi_io_slot === undefined ? null : integer(row.revpi_io_slot, 0),
    revPiIOAction: row.revpi_io_action === "weld_on" || row.revpi_io_action === "weld_off"
      ? row.revpi_io_action
      : "none",
    revPiIOPending: row.revpi_io_pending === true,
    plannerStatus: stringValue(row.planner_status, "not_planned"),
    validationState: stringValue(row.validation_state, "unknown"),
    collisionCheckState: stringValue(row.collision_check_state, "not_run"),
    limitCheckState: stringValue(row.limit_check_state, "not_run"),
    plannedPathRevision: integer(row.planned_path_revision, 0),
    plannedSampleCount: integer(row.planned_sample_count, 0),
    sourceState: sourceState(row.source_state),
    source: "robot_v4",
    extensions: {
      ...storedExtensions,
      __wire: unknownProperties(row, ROW_KEYS),
    },
  };
}

function planStatus(project: OfflineProgrammingProject): RobotV4Program["planner_status"] {
  if (project.plan.state === "server_validated") return "planned";
  if (project.plan.state === "rejected") return "rejected";
  if (project.plan.state === "stale") return "stale";
  return "not_planned";
}

function databaseMove(move: RowMove): NonNullable<RobotV4ProgramRow["move_type"]> {
  if (move === "JOINT") return "joint_ptp";
  if (move === "CURVE") return "curve_tcp";
  return "line_tcp";
}

function authoredMove(move: unknown): RowMove {
  if (move === "joint_ptp") return "JOINT";
  if (move === "curve_tcp") return "CURVE";
  return "LINE";
}

function authoredPhase(phase: unknown): RowPhase {
  return ["approach", "weld", "retract", "travel", "home"].includes(String(phase))
    ? phase as RowPhase
    : "travel";
}

function programCommand(
  command: unknown,
  rowType: RobotV4ProgramRow["row_type"],
): AuthoredProgramRow["command"] {
  const known: AuthoredProgramRow["command"][] = [
    "MOVE",
    "WELD_ON",
    "WELD_OFF",
    "HOME",
    "LOOP",
    "COMMENT",
    "CAPTURE",
    "CALIBRATE",
    "IO",
  ];
  if (known.includes(command as AuthoredProgramRow["command"])) {
    return command as AuthoredProgramRow["command"];
  }
  if (rowType === "comment") return "COMMENT";
  if (rowType === "capture") return "CAPTURE";
  if (rowType === "calibration") return "CALIBRATE";
  if (rowType === "io") return "IO";
  return "MOVE";
}

/** The program's own robot, kept as it was written. An imported program that
 *  names a robot this server does not have is shown as such by the Robot
 *  card; rewriting it to another robot here would silently re-aim the whole
 *  program at a different arm. */
function projectRobotModel(model: RobotV4Program["robot_model"]): RobotModel {
  return String(model ?? "");
}

function sourceState(value: unknown): NonNullable<AuthoredProgramRow["sourceState"]> {
  return ["cad_derived", "teach_modified", "manual", "imported"].includes(String(value))
    ? value as NonNullable<AuthoredProgramRow["sourceState"]>
    : "imported";
}

function tuple6(value: unknown): TcpPose {
  const source = Array.isArray(value) ? value : [];
  return [0, 1, 2, 3, 4, 5].map((index) => finite(source[index], 0)) as TcpPose;
}

function finite(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function integer(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isInteger(parsed) ? parsed : fallback;
}

function stringValue(value: unknown, fallback = ""): string {
  return typeof value === "string" ? value : fallback;
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}

function isRecordValue(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function explicitExtensions(value: Record<string, unknown> | undefined): Record<string, unknown> {
  if (!value) return {};
  const copy = structuredClone(value);
  delete copy.__wire;
  return copy;
}

function wireRecord(value: Record<string, unknown> | undefined): Record<string, unknown> {
  const wire = value?.__wire;
  return typeof wire === "object" && wire !== null && !Array.isArray(wire)
    ? structuredClone(wire) as Record<string, unknown>
    : {};
}

function unknownProperties(
  value: Record<string, unknown>,
  known: ReadonlySet<string>,
): Record<string, unknown> {
  return Object.fromEntries(
    Object.entries(value)
      .filter(([key]) => !known.has(key))
      .map(([key, field]) => [key, structuredClone(field)]),
  );
}

const PROGRAM_KEYS = new Set([
  "schema_version",
  "program_id",
  "program_name",
  "part_name",
  "part_number",
  "part_revision",
  "robotic-cell-type",
  "robotic_cell_type",
  "robot_model",
  "cell_config_revision",
  "program_state",
  "execution_mode",
  "units",
  "program_revision",
  "base_program_revision",
  "program_digest",
  "source_project_id",
  "source_project_revision",
  "edit_source",
  "edited_at",
  "defaults",
  "settings",
  "constraints",
  "planner_status",
  "validation_state",
  "collision_check_state",
  "limit_check_state",
  "planned_path_revision",
  "planned_sample_count",
  "rows",
  "extensions",
]);

const ROW_KEYS = new Set([
  "row_id",
  "row_index",
  "row_number",
  "revision",
  "row_hash",
  "enabled",
  "label",
  "row_type",
  "command",
  "comment",
  "loop_target_row_id",
  "loop_iterations",
  "return_home_before_repeat",
  "move_type",
  "target_frame_type",
  "tool_frame",
  "work_frame",
  "fixture_side",
  "motion_phase",
  "calibration_section",
  "harmonious_phase_deg",
  "capture_source",
  "speed_source",
  "seam_id",
  "pass_id",
  "pass_role",
  "source_project_id",
  "source_project_revision",
  "source_state",
  "target_tcp_m",
  "nominal_tcp_m",
  "target_joints_rad",
  "current_tcp_pose",
  "current_joints_rad",
  "positioner_targets_rad",
  "segment_meters",
  "tcp_velocity_m_s",
  "joint_velocity_rad_s",
  "accel_m_s2",
  "joint_accel_rad_s2",
  "blend_radius_m",
  "fine_stop",
  "playback_step_ms",
  "weld_enable",
  "arc_state",
  "wire_feed_m_per_min",
  "voltage_v",
  "amperage_a",
  "weld_profile_id",
  "weave_profile_id",
  "weld_process",
  "revpi_io_slot",
  "revpi_io_action",
  "revpi_io_pending",
  "planner_status",
  "validation_state",
  "collision_check_state",
  "limit_check_state",
  "planned_path_revision",
  "planned_sample_count",
  "extensions",
]);
