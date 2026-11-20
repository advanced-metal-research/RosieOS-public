import { defaultWorkcellFixtures } from "./workcell.ts";
import {
  DEFAULT_TORCH_STANDOFF_MM,
  DEFAULT_TRAVEL_SPEED_MPS,
  JOINT_ORDER,
  PROJECT_SCHEMA,
  type AuthoredProgramRow,
  type OfflineProgrammingProject,
  assertProject,
  cloneProject,
  createProject,
  defaultProgramV2Header,
  isRecord,
  missingPlan,
  zeroJoints,
} from "./project.ts";
import { hydrateEdgeCatalog } from "./edge-catalog.ts";
import { migrateLegacySeamTrims } from "./seam.ts";
import { normalizeWeavePattern } from "./weld.ts";
import { syncClassicNodes } from "../program-v2/from-project.ts";

export interface ProjectExportEnvelope {
  schema: "offline-programming.project-export.v1";
  exportedAt: string;
  project: OfflineProgrammingProject;
}

export function exportProjectJSON(project: OfflineProgrammingProject, exportedAt = new Date().toISOString()): string {
  assertProject(project);
  const envelope: ProjectExportEnvelope = {
    schema: "offline-programming.project-export.v1",
    exportedAt,
    project: cloneProject(project),
  };
  return `${JSON.stringify(envelope, null, 2)}\n`;
}

export function importProjectJSON(raw: string, importedAt = new Date().toISOString()): OfflineProgrammingProject {
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch (error) {
    throw new Error(`invalid project JSON: ${error instanceof Error ? error.message : String(error)}`);
  }
  const candidate = isRecord(parsed) && parsed.schema === "offline-programming.project-export.v1"
    ? parsed.project
    : parsed;
  const migrated = migrateProject(candidate, importedAt);
  assertProject(migrated);
  return migrated;
}

export function migrateProject(value: unknown, migratedAt = new Date().toISOString()): OfflineProgrammingProject {
  if (isRecord(value) && value.schema === PROJECT_SCHEMA) {
    return hydrateV1(value as unknown as OfflineProgrammingProject);
  }
  if (isRecord(value) && Number(value.schema_version) === 1) {
    return migrateLegacyV1(value, migratedAt);
  }
  const schema = isRecord(value) ? String(value.schema ?? value.schema_version ?? "missing") : "missing";
  throw new Error(`unsupported offline programming project schema ${schema}`);
}

function hydrateV1(project: OfflineProgrammingProject): OfflineProgrammingProject {
  const hydrated = cloneProject(project);
  migrateLegacySeamTrims(hydrated);
  // Projects saved before P2.12 have no pickable-edge catalog.
  hydrated.edgeCatalog = hydrateEdgeCatalog(hydrated.edgeCatalog);
  hydrated.railsAuthoring ??= false;
  // Projects saved before the stand was the operator's: the cell's placeholder,
  // which is what the planner cleared for them all along.
  hydrated.workcellFixtures ??= defaultWorkcellFixtures(hydrated.robot.model);
  hydrated.planHistory ??= [];
  hydrated.robot.jointOrder = JOINT_ORDER;
  hydrated.program ??= createProject({
    id: hydrated.identity.id,
    name: hydrated.identity.name,
    robotModel: hydrated.robot.model,
    now: hydrated.identity.createdAt,
    cad: hydrated.cad,
  }).program;
  hydrated.program.programId ||= hydrated.identity.id;
  hydrated.program.programRevision = Math.max(1, integer(hydrated.program.programRevision, hydrated.identity.revision));
  hydrated.program.baseProgramRevision = Math.max(
    0,
    integer(hydrated.program.baseProgramRevision, Math.max(0, hydrated.program.programRevision - 1)),
  );
  hydrated.program.sourceProjectRevision = Math.max(
    1,
    integer(hydrated.program.sourceProjectRevision, hydrated.identity.revision),
  );
  hydrated.program.defaults ??= {};
  hydrated.program.settings ??= {};
  hydrated.program.constraints ??= {};
  hydrated.program.extensions ??= {};
  hydrated.rows = hydrated.rows.map((row) => hydrateRow(row, hydrated.program.programRevision));
  hydrated.plan ??= missingPlan();
  hydrated.plan.evidence ??= missingPlan().evidence;
  hydrated.plan.trajectory ??= [];
  hydrated.plan.acceptedPlan ??= null;
  hydrated.simulation ??= {
    cursorS: 0,
    playbackRate: 1,
    loop: false,
    selectedRowId: null,
    camera: "orbit",
  };
  hydrated.nodes ??= [];
  hydrated.denseTrajectory ??= null;
  hydrated.programV2 ??= defaultProgramV2Header();
  // A classic program saved before nodes existed: build its nodes from the
  // seams, sequence and rows it already has. Rows are read, not rewritten,
  // so a baked or teach-modified RPY survives as the weld node's points. A
  // legacy rails program has no seams here and stays rows-only.
  if (hydrated.nodes.length === 0 && hydrated.seams.length > 0 && !hydrated.railsAuthoring) {
    syncClassicNodes(hydrated);
  }
  retireFiftyMillimetreDefault(hydrated);
  return hydrated;
}

/** The travel speed nobody chose. Until 2026-09-21 a new project's process was
 *  made at 0.05 m/s, and a hand-placed weld copied that when it was placed, so
 *  saved programs carry welds at exactly 50 mm/s -- ten times a real travel
 *  speed, and fast enough that the planner's 250 ms lattice put 12.5 mm between
 *  knots and refused the plan on tracking. Those, and only those, become the
 *  default on load (owner decision 2026-09-21): a hand-placed weld at exactly
 *  50, and the untouched default process at exactly 0.05. Any other number was
 *  typed by a person and stays. Exact equality is safe here because both values
 *  were written as literals, never computed. */
const RETIRED_DEFAULT_TRAVEL_SPEED_MPS = 0.05;

function retireFiftyMillimetreDefault(project: OfflineProgrammingProject): void {
  for (const process of project.processParameters ?? []) {
    if (process.id === "process-default" && process.travelSpeedMps === RETIRED_DEFAULT_TRAVEL_SPEED_MPS) {
      process.travelSpeedMps = DEFAULT_TRAVEL_SPEED_MPS;
    }
  }
  for (const node of project.nodes ?? []) {
    if (node.op !== "weld" || node.geometry?.kind !== "manual_pose_wp") continue;
    const speed = node.travel_speed_mm_s;
    if (speed?.kind === "constant" && speed.value === RETIRED_DEFAULT_TRAVEL_SPEED_MPS * 1000) {
      speed.value = DEFAULT_TRAVEL_SPEED_MPS * 1000;
    }
  }
}

function migrateLegacyV1(value: Record<string, unknown>, migratedAt: string): OfflineProgrammingProject {
  const legacyIdentity = isRecord(value.identity) ? value.identity : {};
  const legacyRobot = isRecord(value.robot) ? value.robot : {};
  const legacyCad = isRecord(value.cad) ? value.cad : {};
  const project = createProject({
    id: stringValue(legacyIdentity.id, stringValue(value.project_id, undefined)),
    name: stringValue(legacyIdentity.name, stringValue(value.project_name, "Imported offline program")),
    // A legacy project names its robot; keep the name it named. Whether this
    // server has that description is said in the Robot card, not decided here
    // by falling back to whichever robot the browser used to know about.
    robotModel: stringValue(legacyRobot.model, stringValue(value.robot_model, undefined)) || undefined,
    now: stringValue(legacyIdentity.createdAt, migratedAt),
    cad: {
      kind: legacyCad.kind === "step" ? "step" : "example",
      filename: stringValue(legacyCad.filename, "imported.step"),
      fingerprint: stringValue(legacyCad.fingerprint, "legacy:unknown"),
      topologyModelId: stringValue(legacyCad.topologyModelId, "legacy"),
      sourceUnit: legacyCad.sourceUnit === "m" ? "m" : "mm",
    },
  });
  project.identity.description = stringValue(legacyIdentity.description, "");
  project.identity.updatedAt = migratedAt;
  project.identity.revision = Math.max(1, integer(legacyIdentity.revision, integer(value.revision, 1)));
  project.program.programRevision = project.identity.revision;
  project.program.baseProgramRevision = Math.max(0, project.identity.revision - 1);
  project.program.sourceProjectRevision = project.identity.revision;
  project.rows = Array.isArray(value.rows) ? value.rows.flatMap(migrateLegacyRow) : [];
  project.plan = missingPlan();
  return project;
}

function migrateLegacyRow(value: unknown, index: number): AuthoredProgramRow[] {
  if (!isRecord(value)) return [];
  const pose = numberArray(value.targetTcpPose ?? value.target_tcp_pose, 6);
  const targetTcpPose = [
    pose[0] ?? 0,
    pose[1] ?? 0,
    pose[2] ?? 0,
    pose[3] ?? 0,
    pose[4] ?? 0,
    pose[5] ?? 0,
  ] as AuthoredProgramRow["targetTcpPose"];
  return [{
    id: stringValue(value.id, stringValue(value.row_id, `row-${index + 1}`)),
    enabled: value.enabled !== false,
    label: stringValue(value.label, `ROW ${index + 1}`),
    rowType: normalizeRowType(value.rowType ?? value.row_type),
    rowRevision: Math.max(0, integer(value.rowRevision ?? value.revision, 0)),
    rowHash: stringValue(value.rowHash ?? value.row_hash, `legacy-row-${index + 1}`),
    command: normalizeCommand(value.command),
    comment: stringValue(value.comment, ""),
    loopTargetRowId: nullableString(value.loopTargetRowId ?? value.loop_target_row_id),
    loopIterations: Math.max(0, integer(value.loopIterations ?? value.loop_iterations, 0)),
    returnHomeBeforeRepeat:
      value.returnHomeBeforeRepeat === true || value.return_home_before_repeat === true,
    moveType: normalizeMove(value.moveType ?? value.move_type),
    phase: normalizePhase(value.phase ?? value.motion_phase),
    targetFrameType: stringValue(value.targetFrameType ?? value.target_frame_type, "tcp_cartesian"),
    fixtureSide: stringValue(value.fixtureSide ?? value.fixture_side, "in_cell"),
    calibrationSection: Math.max(0, integer(value.calibrationSection ?? value.calibration_section, 0)),
    harmoniousPhaseDeg: finite(value.harmoniousPhaseDeg ?? value.harmonious_phase_deg, 0),
    captureSource: stringValue(value.captureSource ?? value.capture_source, "offline_programming"),
    speedSource: stringValue(value.speedSource ?? value.speed_source, "programmed"),
    seamId: nullableString(value.seamId ?? value.seam_id),
    passId: stringValue(value.passId ?? value.pass_id, ""),
    passRole: stringValue(value.passRole ?? value.pass_role, "none"),
    toolFrameId: stringValue(value.toolFrameId ?? value.tool_frame, "end_effector"),
    workFrameId: stringValue(value.workFrameId ?? value.work_frame, "world"),
    fixtureId: nullableString(value.fixtureId),
    targetTcpPose,
    targetJointsRad: zeroJoints(numberArray(value.targetJointsRad ?? value.target_joints_rad, 9)),
    positionerTargetsRad: tuple3(value.positionerTargetsRad ?? value.positioner_targets_rad),
    positionerJointTargetsRad: zeroJoints(
      numberArray(value.positionerJointTargetsRad ?? value.positioner_targets_rad, 9),
    ),
    nominalTcpM: nullableTuple3(value.nominalTcpM ?? value.nominal_tcp_m),
    currentTcpPose: numberArray(
      value.currentTcpPose ?? value.current_tcp_pose ?? value.targetTcpPose ?? value.target_tcp_pose,
      6,
    ) as AuthoredProgramRow["targetTcpPose"],
    currentJointsRad: zeroJoints(
      numberArray(
        value.currentJointsRad ?? value.current_joints_rad ??
          value.targetJointsRad ?? value.target_joints_rad,
        9,
      ),
    ),
    tcpVelocityMps: finite(value.tcpVelocityMps ?? value.tcp_velocity_m_s, 0.05),
    jointVelocityRadS: finite(value.jointVelocityRadS ?? value.joint_velocity_rad_s, 0.25),
    accelerationMps2: finite(value.accelerationMps2 ?? value.accel_m_s2, 0.75),
    jointAccelerationRadS2: finite(value.jointAccelerationRadS2 ?? value.joint_accel_rad_s2, 1),
    blendRadiusM: finite(value.blendRadiusM ?? value.blend_radius_m, 0),
    fineStop: value.fineStop !== false && value.fine_stop !== false,
    weldEnabled: value.weldEnabled === true || value.weld_enable === true,
    arcState: value.arcState === "on" || value.arc_state === "on" ? "on" : "off",
    wireFeedMPerMin: finite(value.wireFeedMPerMin ?? value.wire_feed_m_per_min, 0),
    voltageV: finite(value.voltageV ?? value.voltage_v, 0),
    amperageA: finite(value.amperageA ?? value.amperage_a, 0),
    weldProfileId: integer(value.weldProfileId ?? value.weld_profile_id, 0),
    weaveProfileId: integer(value.weaveProfileId ?? value.weave_profile_id, 0),
    weldProcess: stringValue(value.weldProcess ?? value.weld_process, "none"),
    revPiIOSlot: nullableInteger(value.revPiIOSlot ?? value.revpi_io_slot),
    revPiIOAction: normalizeIOAction(value.revPiIOAction ?? value.revpi_io_action),
    revPiIOPending: value.revPiIOPending === true || value.revpi_io_pending === true,
    plannerStatus: stringValue(value.plannerStatus ?? value.planner_status, "not_planned"),
    validationState: stringValue(value.validationState ?? value.validation_state, "unknown"),
    collisionCheckState: stringValue(
      value.collisionCheckState ?? value.collision_check_state,
      "not_run",
    ),
    limitCheckState: stringValue(value.limitCheckState ?? value.limit_check_state, "not_run"),
    plannedPathRevision: Math.max(
      0,
      integer(value.plannedPathRevision ?? value.planned_path_revision, 0),
    ),
    plannedSampleCount: Math.max(
      0,
      integer(value.plannedSampleCount ?? value.planned_sample_count, 0),
    ),
    sourceState: normalizeSourceState(value.sourceState ?? value.source_state),
    source: "offline_programming",
  }];
}

function hydrateRow(row: AuthoredProgramRow, programRevision: number): AuthoredProgramRow {
  return {
    ...row,
    toolFrameId: stringValue(row.toolFrameId, "end_effector"),
    rowType: row.rowType ?? "motion",
    rowRevision: Math.max(0, integer(row.rowRevision, programRevision)),
    rowHash: stringValue(row.rowHash, `offline:${row.id}:${programRevision}`),
    comment: row.comment ?? "",
    loopTargetRowId: row.loopTargetRowId ?? null,
    loopIterations: Math.max(0, integer(row.loopIterations, 0)),
    returnHomeBeforeRepeat: row.returnHomeBeforeRepeat === true,
    targetFrameType: row.targetFrameType ?? (row.moveType === "JOINT" ? "joint" : "tcp_cartesian"),
    fixtureSide: row.fixtureSide ?? "in_cell",
    calibrationSection: Math.max(0, integer(row.calibrationSection, 0)),
    harmoniousPhaseDeg: finite(row.harmoniousPhaseDeg, 0),
    captureSource: row.captureSource ?? row.source,
    speedSource: row.speedSource ?? "programmed",
    targetJointsRad: zeroJoints(row.targetJointsRad),
    targetTcpPose: numberArray(row.targetTcpPose, 6) as AuthoredProgramRow["targetTcpPose"],
    positionerTargetsRad: tuple3(row.positionerTargetsRad),
    positionerJointTargetsRad: zeroJoints(
      row.positionerJointTargetsRad ?? row.positionerTargetsRad,
    ),
    nominalTcpM: row.nominalTcpM ? tuple3(row.nominalTcpM) : null,
    currentTcpPose: numberArray(
      row.currentTcpPose ?? row.targetTcpPose,
      6,
    ) as AuthoredProgramRow["targetTcpPose"],
    currentJointsRad: zeroJoints(row.currentJointsRad ?? row.targetJointsRad),
    amperageA: finite(row.amperageA, 0),
    torchWorkAngleDeg: finite(row.torchWorkAngleDeg, 0),
    torchTravelAngleDeg: finite(row.torchTravelAngleDeg, 0),
    torchSpinAngleDeg: finite(row.torchSpinAngleDeg, 0),
    weavePattern: normalizeWeavePattern(row.weavePattern),
    weaveAmplitudeMm: finite(row.weaveAmplitudeMm, 2),
    weaveFrequencyHz: finite(row.weaveFrequencyHz, 1),
    weaveDwellMs: finite(row.weaveDwellMs, 0),
    revPiIOPending: row.revPiIOPending === true,
    plannerStatus: row.plannerStatus ?? "not_planned",
    validationState: row.validationState ?? "unknown",
    collisionCheckState: row.collisionCheckState ?? "not_run",
    limitCheckState: row.limitCheckState ?? "not_run",
    plannedPathRevision: Math.max(0, integer(row.plannedPathRevision, 0)),
    plannedSampleCount: Math.max(0, integer(row.plannedSampleCount, 0)),
    sourceState: row.sourceState ?? (row.source === "cadquery" ? "cad_derived" : "manual"),
    extensions: row.extensions ?? {},
  };
}

function normalizeCommand(value: unknown): AuthoredProgramRow["command"] {
  return [
    "MOVE",
    "WELD_ON",
    "WELD_OFF",
    "HOME",
    "LOOP",
    "COMMENT",
    "CAPTURE",
    "CALIBRATE",
    "IO",
  ].includes(String(value))
    ? String(value) as AuthoredProgramRow["command"]
    : "MOVE";
}

function normalizeRowType(value: unknown): NonNullable<AuthoredProgramRow["rowType"]> {
  return ["motion", "command", "comment", "capture", "calibration", "io"].includes(String(value))
    ? String(value) as NonNullable<AuthoredProgramRow["rowType"]>
    : "motion";
}

function normalizeMove(value: unknown): AuthoredProgramRow["moveType"] {
  return ["LINE", "CURVE", "JOINT"].includes(String(value).toUpperCase())
    ? String(value).toUpperCase() as AuthoredProgramRow["moveType"]
    : "LINE";
}

function normalizePhase(value: unknown): AuthoredProgramRow["phase"] {
  return ["approach", "weld", "retract", "travel", "home"].includes(String(value))
    ? String(value) as AuthoredProgramRow["phase"]
    : "travel";
}

function normalizeIOAction(value: unknown): AuthoredProgramRow["revPiIOAction"] {
  return ["weld_on", "weld_off"].includes(String(value))
    ? String(value) as AuthoredProgramRow["revPiIOAction"]
    : "none";
}

function normalizeSourceState(value: unknown): NonNullable<AuthoredProgramRow["sourceState"]> {
  return ["cad_derived", "teach_modified", "manual", "imported"].includes(String(value))
    ? String(value) as NonNullable<AuthoredProgramRow["sourceState"]>
    : "imported";
}

function stringValue(value: unknown, fallback: string | undefined): string {
  return typeof value === "string" && value.trim() ? value.trim() : (fallback ?? "");
}

function nullableString(value: unknown): string | null {
  return typeof value === "string" && value.trim() ? value.trim() : null;
}

function numberArray(value: unknown, length: number): number[] {
  const source = Array.isArray(value) ? value : [];
  return Array.from({ length }, (_, index) => finite(source[index], 0));
}

function tuple3(value: unknown): [number, number, number] {
  const values = numberArray(value, 3);
  return [values[0]!, values[1]!, values[2]!];
}

function nullableTuple3(value: unknown): [number, number, number] | null {
  return Array.isArray(value) && value.length >= 3 ? tuple3(value) : null;
}

function finite(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function integer(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isInteger(parsed) ? parsed : fallback;
}

function nullableInteger(value: unknown): number | null {
  const parsed = Number(value);
  return Number.isInteger(parsed) && parsed >= 0 ? parsed : null;
}
