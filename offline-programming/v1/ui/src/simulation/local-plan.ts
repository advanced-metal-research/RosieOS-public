import type {
  NormalizedPlanRequest,
  NormalizedPlanRow,
} from "../execution/contracts.ts";
import { NORMALIZED_PLAN_REQUEST_SCHEMA } from "../execution/contracts.ts";
import type { OfflineProgrammingProject } from "../domain/project.ts";
import { toRobotV4Program } from "../adapters/robot-v4.ts";

export interface BuildLocalPlanOptions {
  requestId: string;
  submittedAt?: string;
}

export function buildLocalPlanRequest(
  project: OfflineProgrammingProject,
  options: BuildLocalPlanOptions,
): NormalizedPlanRequest {
  if (project.target.mode !== "local_simulator" || !nonempty(project.target.profileId)) {
    throw new Error("local planning requires an explicit local_simulator target");
  }
  if (!safeIdentity(options.requestId)) {
    throw new Error("local planning requestId is invalid");
  }
  if (!Number.isSafeInteger(project.identity.revision) || project.identity.revision < 1) {
    throw new Error("local planning requires a positive project revision");
  }
  const submittedAt = options.submittedAt ?? new Date().toISOString();
  if (!Number.isFinite(Date.parse(submittedAt))) {
    throw new Error("local planning submittedAt is invalid");
  }
  const rows = project.rows.map((row): NormalizedPlanRow => ({
    rowId: row.id,
    revision: row.rowRevision ?? project.program.programRevision,
    enabled: row.enabled,
    rowType: row.rowType ?? "motion",
    command: row.command ?? "",
    moveType: row.moveType,
    motionPhase: row.phase,
    toolFrame: row.toolFrameId,
    workFrame: row.workFrameId,
    targetTcpM: [...row.targetTcpPose],
    targetJointsRad: [...row.targetJointsRad],
    tcpVelocityMS: row.tcpVelocityMps,
    jointVelocityRadS: row.jointVelocityRadS,
    weldEnable: row.weldEnabled,
    arcState: row.arcState,
  }));
  if (rows.filter(({ enabled, rowType }) => enabled && rowType === "motion").length < 2) {
    throw new Error("local planning requires at least two enabled motion rows");
  }
  validateRows(rows);
  const program = toRobotV4Program(project);
  return {
    schema: NORMALIZED_PLAN_REQUEST_SCHEMA,
    requestId: options.requestId,
    projectId: project.identity.id,
    projectRevision: project.identity.revision,
    programId: project.program.programId,
    programRevision: project.program.programRevision,
    baseProgramRevision: project.program.baseProgramRevision,
    robotModel: project.robot.model,
    target: {
      profileId: project.target.profileId,
      manifestRef: project.target.manifestRef,
    },
    submittedAt,
    program,
    rows,
  };
}

export function localPlanRequestWire(request: NormalizedPlanRequest): unknown {
  return {
    schema: request.schema,
    request_id: request.requestId,
    project_id: request.projectId,
    project_revision: request.projectRevision,
    target: {
      profile_id: request.target.profileId,
      manifest_ref: request.target.manifestRef,
    },
    submitted_at: request.submittedAt,
    program: structuredClone(request.program),
  };
}

function validateRows(rows: NormalizedPlanRow[]): void {
  const ids = new Set<string>();
  for (const [index, row] of rows.entries()) {
    if (!safeIdentity(row.rowId) || ids.has(row.rowId)) {
      throw new Error(`local planning row ${index} has invalid or duplicate identity`);
    }
    ids.add(row.rowId);
    if (row.rowType === "motion" &&
        (!nonempty(row.toolFrame) || !nonempty(row.workFrame))) {
      throw new Error(`local planning row ${index} has incomplete frame identity`);
    }
    for (const value of [
      ...row.targetTcpM,
      ...row.targetJointsRad,
      row.tcpVelocityMS,
      row.jointVelocityRadS,
    ]) {
      if (!Number.isFinite(value)) {
        throw new Error(`local planning row ${index} contains a non-finite value`);
      }
    }
    if (row.tcpVelocityMS < 0 || row.jointVelocityRadS < 0) {
      throw new Error(`local planning row ${index} contains a negative velocity`);
    }
  }
}

function safeIdentity(value: string): boolean {
  return nonempty(value) &&
    value.length <= 128 &&
    /^[A-Za-z0-9._:/-]+$/.test(value);
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}
