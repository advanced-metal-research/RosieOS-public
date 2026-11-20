import {
  JOINT_ORDER,
  createProject,
  projectId,
  reviseProject,
  zeroJoints,
  type AuthoredProgramRow,
  type OfflineProgrammingProject,
  type RowType,
} from "./project.ts";
import { synchronizeTorchOrientations } from "./weld.ts";
import {
  MOVE_EXTENSION_KEY,
  buildProgramView,
  ensureManualCurveMoves,
  refreshAllMoveHashes,
} from "./points-moves.ts";

export type TeachOverrideDecision = "keep_taught" | "regenerate_from_cad";

/**
 * A new program is EMPTY — no rows, no points, no trajectory. Program content
 * only ever comes from a user action (add row, create weld, shift-click a
 * point). Owner directive, 2026-08-05.
 */
export function createBlankSpreadsheetProject(input: {
  name?: string;
  robotModel?: OfflineProgrammingProject["robot"]["model"];
  railsAuthoring?: boolean;
} = {}): OfflineProgrammingProject {
  const project = createProject(input);
  project.rows = [];
  project.simulation.selectedRowId = null;
  project.railsAuthoring = input.railsAuthoring ?? false;
  return project;
}

export function createSpreadsheetRow(
  project: OfflineProgrammingProject,
  rowType: RowType = "motion",
  id = projectId(),
): AuthoredProgramRow {
  const firstRow = project.rows[0];
  const command = commandForType(rowType);
  const defaults = project.program.defaults;
  return {
    id,
    enabled: true,
    label: defaultLabel(rowType, project.rows.length + 1),
    rowType,
    command,
    comment: rowType === "comment" ? "New comment" : "",
    loopTargetRowId: rowType === "command" ? firstRow?.id ?? null : null,
    loopIterations: rowType === "command" ? 1 : 0,
    returnHomeBeforeRepeat: false,
    moveType: "LINE",
    phase: "travel",
    targetFrameType: "tcp_cartesian",
    fixtureSide: "in_cell",
    calibrationSection: 0,
    harmoniousPhaseDeg: 0,
    captureSource: rowType === "capture" ? "tcp_pose" : "offline_programming",
    speedSource: "programmed",
    seamId: null,
    passId: "",
    passRole: "none",
    toolFrameId: text(defaults.default_tool_frame, "end_effector"),
    workFrameId: text(defaults.default_work_frame, "world"),
    fixtureId: null,
    targetTcpPose: [0, 0, 0, 0, 0, 0],
    targetJointsRad: zeroJoints(),
    positionerTargetsRad: [0, 0, 0],
    positionerJointTargetsRad: zeroJoints(),
    nominalTcpM: null,
    currentTcpPose: [0, 0, 0, 0, 0, 0],
    currentJointsRad: zeroJoints(),
    tcpVelocityMps: finite(defaults.default_tcp_velocity_m_s, 0.05),
    jointVelocityRadS: finite(defaults.default_joint_velocity_rad_s, 0.25),
    accelerationMps2: finite(defaults.default_accel_m_s2, 0.75),
    jointAccelerationRadS2: finite(defaults.default_joint_accel_rad_s2, 1),
    blendRadiusM: finite(defaults.default_blend_radius_m, 0),
    fineStop: true,
    weldEnabled: false,
    arcState: "off",
    wireFeedMPerMin: 0,
    voltageV: 0,
    amperageA: 0,
    weldProfileId: 0,
    weaveProfileId: 0,
    weldProcess: "none",
    torchWorkAngleDeg: 0,
    torchTravelAngleDeg: 0,
    torchSpinAngleDeg: 0,
    weavePattern: "none",
    weaveAmplitudeMm: 2,
    weaveFrequencyHz: 1,
    weaveDwellMs: 0,
    revPiIOSlot: null,
    revPiIOAction: "none",
    revPiIOPending: false,
    plannerStatus: "not_planned",
    validationState: "unknown",
    collisionCheckState: "not_run",
    limitCheckState: "not_run",
    plannedPathRevision: 0,
    plannedSampleCount: 0,
    sourceState: "manual",
    source: "offline_programming",
    extensions: {},
  };
}

/**
 * Shift-click add-point flow (P2.12): append one motion control point at the
 * picked position. The row carries no gradient_move_v1 metadata, so the
 * points & moves view exposes it as a bare control point joined to its
 * neighbour by a synthetic legacy move.
 */
export function appendControlPointRow(
  project: OfflineProgrammingProject,
  targetTcpPose: AuthoredProgramRow["targetTcpPose"],
): OfflineProgrammingProject {
  return reviseProject(project, "rows", (draft) => {
    const row = createSpreadsheetRow(draft, "motion");
    row.label = `Point ${String(draft.rows.length + 1).padStart(2, "0")}`;
    row.targetTcpPose = [...targetTcpPose];
    row.currentTcpPose = [...targetTcpPose];
    draft.rows.push(row);
    draft.simulation.selectedRowId = row.id;
  });
}

export function insertSpreadsheetRow(
  project: OfflineProgrammingProject,
  index: number,
  rowType: RowType,
): OfflineProgrammingProject {
  const boundedIndex = Math.max(0, Math.min(project.rows.length, Math.trunc(index)));
  return reviseProject(project, "rows", (draft) => {
    draft.rows.splice(boundedIndex, 0, createSpreadsheetRow(draft, rowType));
    draft.simulation.selectedRowId = draft.rows[boundedIndex]!.id;
  });
}

export function duplicateSpreadsheetRow(
  project: OfflineProgrammingProject,
  rowID: string,
): OfflineProgrammingProject {
  const index = project.rows.findIndex((row) => row.id === rowID);
  if (index < 0) throw new Error(`spreadsheet row ${rowID} was not found`);
  return reviseProject(project, "rows", (draft) => {
    const source = draft.rows[index]!;
    const duplicate = structuredClone(source);
    duplicate.id = projectId();
    duplicate.label = `${source.label} copy`;
    duplicate.rowRevision = undefined;
    duplicate.rowHash = undefined;
    duplicate.sourceState = source.sourceState === "teach_modified" ? "teach_modified" : "manual";
    // A duplicated row is a NEW control point: cloning gradient_move_v1
    // verbatim would carry a duplicate control_id and stale samples_hash
    // into any future control_id lookup (review F4, 2026-08-04).
    if (duplicate.extensions) delete duplicate.extensions[MOVE_EXTENSION_KEY];
    duplicate.source = "offline_programming";
    draft.rows.splice(index + 1, 0, duplicate);
    draft.simulation.selectedRowId = duplicate.id;
  });
}

export function deleteSpreadsheetRow(
  project: OfflineProgrammingProject,
  rowID: string,
): OfflineProgrammingProject {
  // An empty authored program is legal (2026-08-05): the wire
  // contract's ≥1-row rule applies at export/plan time, not while editing.
  const index = project.rows.findIndex((row) => row.id === rowID);
  if (index < 0) throw new Error(`spreadsheet row ${rowID} was not found`);
  return reviseProject(project, "rows", (draft) => {
    draft.rows.splice(index, 1);
    draft.simulation.selectedRowId =
      draft.rows[Math.min(index, draft.rows.length - 1)]?.id ?? null;
    for (const row of draft.rows) {
      if (row.loopTargetRowId === rowID) row.loopTargetRowId = draft.rows[0]?.id ?? null;
    }
  });
}

export function moveSpreadsheetRow(
  project: OfflineProgrammingProject,
  rowID: string,
  delta: -1 | 1,
): OfflineProgrammingProject {
  const index = project.rows.findIndex((row) => row.id === rowID);
  const destination = index + delta;
  if (index < 0) throw new Error(`spreadsheet row ${rowID} was not found`);
  if (destination < 0 || destination >= project.rows.length) return project;
  return reviseProject(project, "rows", (draft) => {
    const [row] = draft.rows.splice(index, 1);
    draft.rows.splice(destination, 0, row!);
    draft.simulation.selectedRowId = rowID;
  });
}

export function updateSpreadsheetRows(
  project: OfflineProgrammingProject,
  rowIDs: readonly string[],
  edit: (row: AuthoredProgramRow) => void,
): OfflineProgrammingProject {
  const selected = new Set(rowIDs);
  if (selected.size === 0) return project;
  const found = project.rows.filter((row) => selected.has(row.id)).length;
  if (found !== selected.size) throw new Error("one or more spreadsheet rows were not found");
  // Only moves that reconciled cleanly BEFORE this edit may have their sample
  // hash refreshed. Rehashing an externally modified move would silently heal
  // the mismatch and re-hide the peer's rows behind a collapsed move, which is
  // exactly the claim the reconciliation rule forbids.
  const intactMoveIDs = new Set(
    buildProgramView(project).moves
      .filter((move) => !move.modifiedExternally)
      .map((move) => move.id),
  );
  return reviseProject(project, "rows", (draft) => {
    const orientationSeamIDs = new Set<string>();
    for (const row of draft.rows) {
      if (!selected.has(row.id)) continue;
      const before = orientationInputs(row);
      edit(row);
      if (
        row.seamId &&
        row.phase === "weld" &&
        before !== orientationInputs(row)
      ) {
        orientationSeamIDs.add(row.seamId);
      }
      if (row.sourceState === "cad_derived") row.sourceState = "manual";
      row.source = "offline_programming";
    }
    synchronizeTorchOrientations(draft.rows, orientationSeamIDs);
    ensureManualCurveMoves(draft.rows);
    refreshAllMoveHashes(draft.rows, intactMoveIDs);
  });
}

export function resolveTeachOverride(
  project: OfflineProgrammingProject,
  rowID: string,
  decision: TeachOverrideDecision,
  regenerated?: AuthoredProgramRow,
): OfflineProgrammingProject {
  const index = project.rows.findIndex((row) => row.id === rowID);
  if (index < 0) throw new Error(`spreadsheet row ${rowID} was not found`);
  if (project.rows[index]!.sourceState !== "teach_modified") {
    throw new Error("row is not a teach-modified override");
  }
  if (decision === "regenerate_from_cad" && !regenerated) {
    throw new Error("a regenerated CAD row is required");
  }
  return reviseProject(project, "rows", (draft) => {
    const current = draft.rows[index]!;
    if (decision === "keep_taught") {
      current.sourceState = "teach_modified";
      current.extensions = {
        ...(current.extensions ?? {}),
        teach_override_resolution: "keep_taught",
      };
      return;
    }
    const replacement = structuredClone(regenerated!);
    replacement.id = current.id;
    replacement.rowRevision = undefined;
    replacement.rowHash = undefined;
    replacement.sourceState = "cad_derived";
    replacement.source = "cadquery";
    replacement.extensions = {
      ...(replacement.extensions ?? {}),
      teach_override_resolution: "regenerate_from_cad",
    };
    draft.rows[index] = replacement;
  });
}

export function canonicalJointName(index: number): typeof JOINT_ORDER[number] {
  const joint = JOINT_ORDER[index];
  if (!joint) throw new Error("joint index is outside canonical J1-J9 order");
  return joint;
}

function commandForType(rowType: RowType): AuthoredProgramRow["command"] {
  switch (rowType) {
    case "motion": return "MOVE";
    case "command": return "LOOP";
    case "comment": return "COMMENT";
    case "capture": return "CAPTURE";
    case "calibration": return "CALIBRATE";
    case "io": return "IO";
  }
}

function defaultLabel(rowType: RowType, number: number): string {
  // Motion rows are the user's waypoints — label them as points.
  if (rowType === "motion") return `Point ${String(number).padStart(2, "0")}`;
  return `${rowType[0]!.toUpperCase()}${rowType.slice(1)} ${String(number).padStart(2, "0")}`;
}

function finite(value: unknown, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function text(value: unknown, fallback: string): string {
  return typeof value === "string" && value.trim() ? value.trim() : fallback;
}

function orientationInputs(row: AuthoredProgramRow): string {
  return [
    row.targetTcpPose[0],
    row.targetTcpPose[1],
    row.targetTcpPose[2],
    row.torchWorkAngleDeg,
    row.torchTravelAngleDeg,
    row.torchSpinAngleDeg,
  ].join("|");
}
