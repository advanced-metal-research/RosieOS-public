import type {
  JointVector,
  RobotModel,
  TcpPose,
} from "../domain/project.ts";
import type { RobotV4Program } from "../contracts/robot-v4-program.ts";

export type {
  JointVector,
  RobotModel,
  TcpPose,
} from "../domain/project.ts";

export const NORMALIZED_PLAN_REQUEST_SCHEMA =
  "offline-programming.normalized-plan-request.v1" as const;
export const NORMALIZED_PLAN_EVIDENCE_SCHEMA =
  "offline-programming.normalized-plan-evidence.v1" as const;

export type PlanValidationState =
  | "unvalidated"
  | "server_validated"
  | "rejected"
  | "stale";
export type CheckState = "unavailable" | "clear" | "blocked";
export interface PlanTargetIdentity {
  profileId: string;
  manifestRef: string | null;
}

export interface PlanIdentity {
  projectId: string;
  projectRevision: number;
  programId: string;
  programRevision: number;
  planId: string;
  planRevision: number;
  programHash: string;
  robotModel: RobotModel;
  target: PlanTargetIdentity;
}

export interface NormalizedPlanRow {
  rowId: string;
  revision: number;
  enabled: boolean;
  rowType: RobotV4Program["rows"][number]["row_type"];
  command: string;
  moveType: string;
  motionPhase: string;
  toolFrame: string;
  workFrame: string;
  targetTcpM: TcpPose;
  targetJointsRad: JointVector;
  tcpVelocityMS: number;
  jointVelocityRadS: number;
  weldEnable: boolean;
  arcState: string;
}

export interface NormalizedPlanRequest {
  schema: typeof NORMALIZED_PLAN_REQUEST_SCHEMA;
  requestId: string;
  projectId: string;
  projectRevision: number;
  programId: string;
  programRevision: number;
  baseProgramRevision: number;
  robotModel: RobotModel;
  target: PlanTargetIdentity;
  submittedAt: string;
  program: RobotV4Program;
  // Validated correlation index for report row references. It is never sent
  // as a second planner input; the complete program above is authoritative.
  rows: NormalizedPlanRow[];
}

export interface CollisionPoint {
  rowIndex: number;
  waypointIndex: number;
  linkA: string;
  linkB: string;
  depthM: number;
  affectedJoints: string[];
  positionsRad?: number[];
}

export interface SingularityPoint {
  rowIndex: number;
  waypointIndex: number;
  state: string;
  appliedVelocityScale: number;
  limitingJoint?: string;
  affectedJoints: string[];
  positionsRad?: number[];
}

export interface UnreachablePoint {
  rowIndex: number;
  waypointIndex: number;
  reason: string;
  joint?: string;
  positionsRad?: number[];
}

export interface NormalizedPlanEvidence {
  schema: typeof NORMALIZED_PLAN_EVIDENCE_SCHEMA;
  requestId: string;
  identity: PlanIdentity;
  generatedAt: string;
  plannerOwner: string;
  plannerBackend: string;
  state: PlanValidationState;
  accepted: boolean;
  validationState: string;
  sampleCount: number;
  collision: {
    state: CheckState;
    checkState: string;
    points: CollisionPoint[];
  };
  singularity: {
    state: CheckState;
    checkState: string;
    points: SingularityPoint[];
  };
  limits: {
    state: CheckState;
    checkState: string;
    points: UnreachablePoint[];
  };
  runtime: {
    state: CheckState;
    playable: boolean;
  };
  correlationReasons: string[];
}
