import {
  cloneProject,
  missingPlan,
  zeroJoints,
  type JointVector,
  type OfflineProgrammingProject,
  type PlanTrajectorySample,
} from "../domain/project.ts";
import { createSpreadsheetRow } from "../domain/spreadsheet.ts";

export type ViewerJointMovePurpose =
  | "home"
  | "program_start"
  | "timeline_seek"
  | "loop_return";

export interface ViewerJointMoveRequest {
  purpose: ViewerJointMovePurpose;
  startJointsRad: JointVector;
  targetJointsRad: JointVector;
  localProfileId: string;
}

const JOINT_TOLERANCE_RAD = 1e-6;

/**
 * Creates a transient two-waypoint Robot V4 program for the canonical Motion
 * Server/Tesseract planner. The browser never creates the trajectory itself.
 */
export function createViewerJointMoveProject(
  source: OfflineProgrammingProject,
  request: ViewerJointMoveRequest,
): OfflineProgrammingProject {
  assertJointVector(request.startJointsRad, "start");
  assertJointVector(request.targetJointsRad, "target");
  if (!request.localProfileId.trim()) {
    throw new Error("viewer joint move requires a local planner profile");
  }

  const transient = cloneProject(source);
  transient.identity.id = `viewer-motion-${source.identity.id}`;
  transient.identity.name = `${source.identity.name} · ${request.purpose} MOVEJ`;
  transient.target = {
    mode: "local_simulator",
    profileId: request.localProfileId,
    manifestRef: null,
  };
  transient.program.programId = `viewer-motion-${source.identity.id}`;
  transient.program.programRevision = 1;
  transient.program.baseProgramRevision = 0;
  transient.program.programDigest = null;
  transient.program.catalogRevision = null;
  transient.program.programState = "needs_plan";
  transient.program.executionMode = "simulation";
  transient.program.editSource = "offline_programming";
  transient.program.sourceProjectRevision = transient.identity.revision;
  transient.program.defaults = {
    ...transient.program.defaults,
    default_move_type: "joint_ptp",
  };
  transient.plan = missingPlan();
  transient.simulation.cursorS = 0;
  transient.simulation.loop = false;

  const start = createSpreadsheetRow(
    transient,
    "motion",
    `viewer-${request.purpose}-start`,
  );
  const target = createSpreadsheetRow(
    transient,
    "motion",
    `viewer-${request.purpose}-target`,
  );
  configureJointRow(
    start,
    `Current J1–J9 · ${request.purpose}`,
    request.startJointsRad,
    request.startJointsRad,
    transient.program.programRevision,
  );
  configureJointRow(
    target,
    request.purpose === "home" ? "Home · J1–J9 zero" : `Target · ${request.purpose}`,
    request.targetJointsRad,
    request.startJointsRad,
    transient.program.programRevision,
  );
  transient.rows = [start, target];
  transient.simulation.selectedRowId = target.id;
  return transient;
}

export function validateViewerJointMoveTrajectory(
  trajectory: readonly PlanTrajectorySample[],
  startJointsRad: JointVector,
  targetJointsRad: JointVector,
): void {
  if (trajectory.length < 2) {
    throw new Error("Tesseract joint move must contain at least two timed samples");
  }
  const durationS = trajectory.at(-1)?.timeS ?? 0;
  if (!Number.isFinite(durationS) || durationS <= 0) {
    throw new Error("Tesseract joint move must have a positive duration");
  }
  if (!jointsMatch(trajectory[0]!.jointPositionsRad, startJointsRad)) {
    throw new Error("Tesseract joint move does not start at the rendered robot pose");
  }
  if (!jointsMatch(trajectory.at(-1)!.jointPositionsRad, targetJointsRad)) {
    throw new Error("Tesseract joint move does not end at the requested joint target");
  }
  let previousTimeS = -1;
  for (const [index, sample] of trajectory.entries()) {
    if (
      sample.index !== index ||
      !Number.isFinite(sample.timeS) ||
      sample.timeS < previousTimeS ||
      sample.jointPositionsRad.length !== startJointsRad.length ||
      sample.jointPositionsRad.some((value) => !Number.isFinite(value))
    ) {
      throw new Error(`Tesseract joint move sample ${index} is invalid`);
    }
    previousTimeS = sample.timeS;
  }
}

export function jointsMatch(
  left: readonly number[],
  right: readonly number[],
  toleranceRad = JOINT_TOLERANCE_RAD,
): boolean {
  return left.length === right.length &&
    left.every((value, index) =>
      Math.abs(value - (right[index] ?? Number.NaN)) <= toleranceRad
    );
}

export function homeJointTarget(): JointVector {
  return zeroJoints();
}

function configureJointRow(
  row: ReturnType<typeof createSpreadsheetRow>,
  label: string,
  targetJointsRad: JointVector,
  currentJointsRad: JointVector,
  revision: number,
): void {
  row.enabled = true;
  row.label = label;
  row.rowType = "motion";
  row.command = "MOVE";
  row.moveType = "JOINT";
  row.phase = "travel";
  row.targetFrameType = "joint_positions";
  row.speedSource = "programmed";
  row.targetTcpPose = [0, 0, 0, 0, 0, 0];
  row.currentTcpPose = [0, 0, 0, 0, 0, 0];
  row.targetJointsRad = [...targetJointsRad];
  row.currentJointsRad = [...currentJointsRad];
  row.positionerTargetsRad = [0, 0, 0];
  row.positionerJointTargetsRad = zeroJoints();
  row.fineStop = true;
  row.weldEnabled = false;
  row.arcState = "off";
  row.extensions = {
    ...row.extensions,
    __wire: {
      playback_step_ms: 0,
    },
  };
  row.rowRevision = revision;
  row.rowHash = `viewer-motion:${row.id}:${revision}`;
  row.plannerStatus = "not_planned";
  row.validationState = "unknown";
  row.collisionCheckState = "not_run";
  row.limitCheckState = "not_run";
  row.plannedPathRevision = 0;
  row.plannedSampleCount = 0;
}

function assertJointVector(value: readonly number[], label: string): asserts value is JointVector {
  if (value.length !== 9 || value.some((joint) => !Number.isFinite(joint))) {
    throw new Error(`viewer joint move ${label} must be finite J1-J9`);
  }
}
