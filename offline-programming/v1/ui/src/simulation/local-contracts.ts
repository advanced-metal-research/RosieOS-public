import type {
  CheckState,
  CollisionPoint,
  NormalizedPlanEvidence,
  NormalizedPlanRequest,
  PlanIdentity,
  PlanTargetIdentity,
  SingularityPoint,
  UnreachablePoint,
} from "../execution/contracts.ts";
import type {
  JointVector,
  PlanTrajectorySample,
  RobotModel,
  RowPhase,
  TcpPose,
} from "../domain/project.ts";
import {
  parseRobotV4AcceptedPlan,
  type RobotV4AcceptedPlan,
} from "../contracts/robot-v4-program.ts";

export const LOCAL_SIMULATOR_STATUS_SCHEMA =
  "offline-programming.local-simulator-status.v1" as const;
export const LOCAL_SIMULATOR_SAFETY_SCHEMA =
  "offline-programming.local-simulator-safety-proof.v1" as const;
export const LOCAL_PLAN_RESULT_SCHEMA =
  "offline-programming.normalized-plan-result.v1" as const;
export const MOTION_SERVER_PLANNER_BACKEND =
  "motion_server_tesseract_trajectory_v1" as const;
const MAX_LOCAL_PROOF_WINDOW_MS = 30_000;
const MAX_LOCAL_FUTURE_SKEW_MS = 1_000;

export type LocalLifecyclePhase =
  | "stopped"
  | "starting"
  | "ready"
  | "stopping"
  | "failed";

export interface LocalSafetyProof {
  schema: string;
  profileId: string;
  runtimeBootId: string;
  statusSequence: number;
  observedAtMs: number;
  freshUntilMs: number;
  simulation: boolean;
  noRealMovement: boolean;
  loopbackOnly: boolean;
  ethercatOutputEnabled: boolean;
  motionOutputEnabled: boolean;
  hardwareAccess: boolean;
  physicalServicesStarted: boolean;
  commandSubmissionEnabled: boolean;
}

export interface LocalBackendAvailability {
  schema: string;
  available: boolean;
  code: string;
  reason: string;
  canonicalPlanner: boolean;
  processBounded: boolean;
  persistentProcess: boolean;
  explicitOptIn: boolean;
}

export interface LocalSimulatorStatus {
  schema: string;
  serviceId: string;
  statusSequence: number;
  phase: LocalLifecyclePhase;
  observedAtMs: number;
  availability: LocalBackendAvailability;
  profileId: string;
  robotModel: RobotModel;
  platform: string;
  loopbackHost: string;
  planTransport: string;
  physicalServiceStart: boolean;
  commandTransport: boolean;
  ready: boolean;
  readinessReasons: string[];
  safety: LocalSafetyProof;
  planInFlight: boolean;
  lastErrorCode: string;
  lastError: string;
}

export interface LocalPlanResult {
  schema: typeof LOCAL_PLAN_RESULT_SCHEMA;
  evidence: NormalizedPlanEvidence;
  trajectory: PlanTrajectorySample[];
  reportReasons: string[];
  acceptedPlan: RobotV4AcceptedPlan | null;
}

export function localSafetyReasons(
  status: LocalSimulatorStatus,
  nowMs = Date.now(),
): string[] {
  const proof = status.safety;
  const reasons: string[] = [];
  if (status.schema !== LOCAL_SIMULATOR_STATUS_SCHEMA) reasons.push("status_schema_invalid");
  if (!status.availability.available) reasons.push(status.availability.code || "backend_unavailable");
  if (
    status.availability.schema !==
      "offline-programming.local-simulator-availability.v1"
  ) {
    reasons.push("availability_schema_invalid");
  }
  if (!status.availability.canonicalPlanner) reasons.push("canonical_planner_not_proven");
  if (!status.availability.processBounded) reasons.push("process_bound_not_proven");
  if (!status.availability.persistentProcess) reasons.push("persistent_process_not_proven");
  if (!status.availability.explicitOptIn) reasons.push("explicit_opt_in_not_proven");
  if (status.phase !== "ready" || !status.ready) reasons.push("runtime_not_ready");
  if (status.readinessReasons.length > 0) reasons.push("readiness_reasons_present");
  if (!positiveInteger(status.statusSequence)) reasons.push("status_sequence_invalid");
  if (status.platform !== "pc_native") reasons.push("platform_not_pc_native");
  if (!loopbackHost(status.loopbackHost)) reasons.push("host_not_loopback");
  if (status.planTransport !== "in_process") reasons.push("plan_transport_not_in_process");
  if (status.physicalServiceStart) reasons.push("physical_service_start_enabled");
  if (status.commandTransport) reasons.push("command_transport_enabled");
  if (proof.schema !== LOCAL_SIMULATOR_SAFETY_SCHEMA) reasons.push("safety_schema_invalid");
  if (proof.profileId !== status.profileId) reasons.push("safety_profile_mismatch");
  if (!nonempty(proof.runtimeBootId)) reasons.push("runtime_boot_id_missing");
  if (!positiveInteger(proof.statusSequence)) reasons.push("safety_sequence_invalid");
  if (
    !validTime(nowMs) ||
    !validTime(proof.observedAtMs) ||
    !validTime(proof.freshUntilMs) ||
    proof.observedAtMs > nowMs + MAX_LOCAL_FUTURE_SKEW_MS ||
    nowMs >= proof.freshUntilMs ||
    nowMs - proof.observedAtMs > MAX_LOCAL_PROOF_WINDOW_MS
  ) {
    reasons.push("safety_proof_stale");
  }
  if (
    proof.freshUntilMs - proof.observedAtMs > MAX_LOCAL_PROOF_WINDOW_MS ||
    proof.freshUntilMs - nowMs > MAX_LOCAL_PROOF_WINDOW_MS
  ) {
    reasons.push("safety_proof_window_unbounded");
  }
  if (!proof.simulation) reasons.push("simulation_not_proven");
  if (!proof.noRealMovement) reasons.push("no_real_movement_not_proven");
  if (!proof.loopbackOnly) reasons.push("loopback_only_not_proven");
  if (proof.ethercatOutputEnabled) reasons.push("ethercat_output_enabled");
  if (proof.motionOutputEnabled) reasons.push("motion_output_enabled");
  if (proof.hardwareAccess) reasons.push("hardware_access_enabled");
  if (proof.physicalServicesStarted) reasons.push("physical_services_started");
  if (proof.commandSubmissionEnabled) reasons.push("command_submission_enabled");
  return [...new Set(reasons)];
}

export function localSimulatorReady(
  status: LocalSimulatorStatus | null,
  nowMs = Date.now(),
): boolean {
  return status !== null && localSafetyReasons(status, nowMs).length === 0;
}

export function parseLocalSimulatorStatus(value: unknown): LocalSimulatorStatus {
  const root = record(value, "local simulator status");
  const topology = record(root.topology, "local simulator topology");
  const readiness = record(root.readiness, "local simulator readiness");
  const safety = record(root.safety, "local simulator safety");
  const availability = record(root.availability, "local simulator availability");
  const phase = string(root.phase, "phase");
  if (!["stopped", "starting", "ready", "stopping", "failed"].includes(phase)) {
    throw new Error(`invalid local simulator phase ${JSON.stringify(phase)}`);
  }
  const robotModel = robotModelName(topology.robot_model, "robot_model");
  const status: LocalSimulatorStatus = {
    schema: string(root.schema, "schema"),
    serviceId: string(root.service_id, "service_id"),
    statusSequence: integer(root.status_sequence, "status_sequence"),
    phase: phase as LocalLifecyclePhase,
    observedAtMs: timestamp(root.observed_at, "observed_at"),
    availability: {
      schema: string(availability.schema, "availability.schema"),
      available: bool(availability.available, "availability.available"),
      code: string(availability.code, "availability.code"),
      reason: optionalString(availability.reason),
      canonicalPlanner: bool(
        availability.canonical_planner,
        "availability.canonical_planner",
      ),
      processBounded: bool(
        availability.process_bounded,
        "availability.process_bounded",
      ),
      persistentProcess: bool(
        availability.persistent_process,
        "availability.persistent_process",
      ),
      explicitOptIn: bool(
        availability.explicit_opt_in,
        "availability.explicit_opt_in",
      ),
    },
    profileId: string(topology.profile_id, "profile_id"),
    robotModel,
    platform: string(topology.platform, "platform"),
    loopbackHost: string(topology.loopback_host, "loopback_host"),
    planTransport: string(topology.plan_transport, "plan_transport"),
    physicalServiceStart: bool(topology.physical_service_start, "physical_service_start"),
    commandTransport: bool(topology.command_transport, "command_transport"),
    ready: bool(readiness.ready, "ready"),
    readinessReasons: stringArray(readiness.reasons, "readiness reasons"),
    safety: {
      schema: optionalString(safety.schema),
      profileId: optionalString(safety.profile_id),
      runtimeBootId: optionalString(safety.runtime_boot_id),
      statusSequence: integer(safety.status_sequence, "safety.status_sequence"),
      observedAtMs: timestamp(safety.observed_at, "safety.observed_at"),
      freshUntilMs: timestamp(safety.fresh_until, "safety.fresh_until"),
      simulation: bool(safety.simulation, "safety.simulation"),
      noRealMovement: bool(safety.no_real_movement, "safety.no_real_movement"),
      loopbackOnly: bool(safety.loopback_only, "safety.loopback_only"),
      ethercatOutputEnabled: bool(
        safety.ethercat_output_enabled,
        "safety.ethercat_output_enabled",
      ),
      motionOutputEnabled: bool(
        safety.motion_output_enabled,
        "safety.motion_output_enabled",
      ),
      hardwareAccess: bool(safety.hardware_access, "safety.hardware_access"),
      physicalServicesStarted: bool(
        safety.physical_services_started,
        "safety.physical_services_started",
      ),
      commandSubmissionEnabled: bool(
        safety.command_submission_enabled,
        "safety.command_submission_enabled",
      ),
    },
    planInFlight: bool(root.plan_in_flight, "plan_in_flight"),
    lastErrorCode: optionalString(root.last_error_code),
    lastError: optionalString(root.last_error),
  };
  return status;
}

export function parseLocalPlanResult(
  value: unknown,
  request: NormalizedPlanRequest,
): LocalPlanResult {
  const root = record(value, "local plan result");
  const evidence = parseEvidence(root.evidence);
  const trajectoryRaw = array(root.trajectory, "trajectory");
  const reportReasons = stringArray(root.report_reasons, "report_reasons");
  const trajectory = trajectoryRaw.map((entry, index) => {
    const point = record(entry, `trajectory ${index}`);
    const waypointIndex = integer(point.waypoint_index, `trajectory ${index} waypoint`);
    const rowIndex = integer(point.row_index, `trajectory ${index} row`);
    const moveType = string(point.move_type, `trajectory ${index} move_type`);
    if (!["line_tcp", "curve_tcp", "joint_ptp"].includes(moveType)) {
      throw new Error(`trajectory ${index} move_type is not canonical`);
    }
    const tcpPose = numberTuple(point.tcp_pose, 6, `trajectory ${index} TCP`) as TcpPose;
    const joints = numberTuple(
      point.positions_rad,
      9,
      `trajectory ${index} joints`,
    ) as JointVector;
    return {
      index: waypointIndex,
      rowIndex,
      seamId: null,
      phase: rowPhase(point.motion_phase),
      weldOn: bool(point.weld_on, `trajectory ${index} weld_on`),
      timeS: finiteNumber(point.time_s, `trajectory ${index} time_s`),
      tcpPose,
      jointPositionsRad: joints,
    };
  });
  const result: LocalPlanResult = {
    schema: string(root.schema, "schema") as typeof LOCAL_PLAN_RESULT_SCHEMA,
    evidence,
    trajectory,
    reportReasons,
    acceptedPlan: root.accepted_plan === undefined
      ? null
      : parseRobotV4AcceptedPlan(root.accepted_plan),
  };
  const correlationReasons = localPlanCorrelationReasons(result, request);
  if (correlationReasons.length > 0) {
    throw new Error(
      `local plan correlation failed closed: ${correlationReasons.join(",")}`,
    );
  }
  return result;
}

export function localPlanCorrelationReasons(
  result: LocalPlanResult,
  request: NormalizedPlanRequest,
): string[] {
  const reasons: string[] = [];
  const plan = result.evidence.identity;
  if (result.schema !== LOCAL_PLAN_RESULT_SCHEMA) reasons.push("result_schema_invalid");
  if (
    result.evidence.schema !== "offline-programming.normalized-plan-evidence.v1"
  ) {
    reasons.push("evidence_schema_invalid");
  }
  if (result.evidence.requestId !== request.requestId) reasons.push("request_id_mismatch");
  if (
    plan.projectId !== request.projectId ||
    plan.projectRevision !== request.projectRevision
  ) {
    reasons.push("project_revision_mismatch");
  }
  if (
    plan.programId !== request.programId ||
    plan.programRevision !== request.programRevision
  ) {
    reasons.push("program_revision_mismatch");
  }
  if (plan.robotModel !== request.robotModel) reasons.push("robot_model_mismatch");
  if (
    plan.target.profileId !== request.target.profileId ||
    plan.target.manifestRef !== request.target.manifestRef
  ) {
    reasons.push("target_mismatch");
  }
  if (!positiveInteger(plan.planRevision) || !nonempty(plan.planId)) {
    reasons.push("plan_identity_incomplete");
  }
  if (
    result.evidence.plannerOwner !== "motion-server" ||
    result.evidence.plannerBackend !== MOTION_SERVER_PLANNER_BACKEND
  ) {
    reasons.push("canonical_planner_identity_invalid");
  }
  if (result.evidence.correlationReasons.length > 0) {
    reasons.push("canonical_report_uncorrelated");
  }
  return [...new Set(reasons)];
}

export function localPlanPlayableReasons(
  result: LocalPlanResult,
  request: NormalizedPlanRequest,
): string[] {
  const reasons: string[] = [];
  if (
    result.evidence.state !== "server_validated" ||
    !result.evidence.accepted ||
    result.evidence.validationState !== "valid" ||
    result.evidence.correlationReasons.length > 0
  ) {
    reasons.push("canonical_plan_evidence_invalid");
  }
  if (
    result.evidence.collision.state !== "clear" ||
    result.evidence.collision.checkState !== "tesseract_discrete_contact_checked" ||
    result.evidence.collision.points.length > 0
  ) {
    reasons.push("collision_evidence_not_clear");
  }
  if (
    result.evidence.singularity.state !== "clear" ||
    result.evidence.singularity.checkState !== "jacobian_svd_checked" ||
    result.evidence.singularity.points.some((point) =>
      point.rowIndex < 0 ||
      point.rowIndex >= request.rows.length ||
      point.waypointIndex < 0 ||
      point.waypointIndex >= result.evidence.sampleCount ||
      point.state !== "warning" ||
      point.appliedVelocityScale < 0 ||
      point.appliedVelocityScale > 1 ||
      point.positionsRad?.length !== 9
    )
  ) {
    reasons.push("singularity_evidence_not_clear");
  }
  if (
    result.evidence.limits.state !== "clear" ||
    result.evidence.limits.checkState !== "clear" ||
    result.evidence.limits.points.length > 0
  ) {
    reasons.push("limit_evidence_not_clear");
  }
  if (
    result.evidence.runtime.state !== "clear" ||
    !result.evidence.runtime.playable
  ) {
    reasons.push("runtime_not_playable");
  }
  if (
    result.reportReasons.length > 0 ||
    result.evidence.sampleCount < 1 ||
    result.trajectory.length !== result.evidence.sampleCount
  ) {
    reasons.push("trajectory_evidence_incomplete");
  }
  const acceptedPlan = result.acceptedPlan;
  if (
    acceptedPlan === null ||
    acceptedPlan.authority_classification !== "simulation_only" ||
    acceptedPlan.plan_id !== result.evidence.identity.planId ||
    acceptedPlan.plan_revision !== result.evidence.identity.planRevision ||
    acceptedPlan.program_id !== request.programId ||
    acceptedPlan.program_revision !== request.programRevision ||
    acceptedPlan.planner.owner !== "motion-server" ||
    acceptedPlan.planner.backend !== MOTION_SERVER_PLANNER_BACKEND ||
    acceptedPlan.target.target_id !== "local-simulator" ||
    acceptedPlan.target.robot_model !== request.robotModel ||
    acceptedPlan.evidence.runtime_playable ||
    acceptedPlan.trajectory.length !== result.trajectory.length
  ) {
    reasons.push("simulation_accepted_plan_invalid");
  }
  if (acceptedPlan !== null && acceptedPlan.trajectory.length === result.trajectory.length) {
    for (let index = 0; index < result.trajectory.length; index += 1) {
      const normalized = result.trajectory[index]!;
      const accepted = acceptedPlan.trajectory[index]!;
      const requestRow = request.rows[normalized.rowIndex];
      if (
        accepted.sample_index !== normalized.index ||
        accepted.row_index !== normalized.rowIndex ||
        accepted.row_id !== requestRow?.rowId ||
        accepted.time_s !== normalized.timeS ||
        accepted.weld_on !== normalized.weldOn ||
        !sameNumbers(accepted.tcp_pose_m, normalized.tcpPose) ||
        !sameNumbers(accepted.joint_positions_rad, normalized.jointPositionsRad)
      ) {
        reasons.push("accepted_trajectory_mismatch");
        break;
      }
    }
  }
  let previousTime = -1;
  for (let index = 0; index < result.trajectory.length; index += 1) {
    const point = result.trajectory[index]!;
    if (
      point.index !== index ||
      point.rowIndex < 0 ||
      point.rowIndex >= request.rows.length ||
      point.timeS < previousTime
    ) {
      reasons.push("trajectory_identity_invalid");
      break;
    }
    previousTime = point.timeS;
  }
  return [...new Set(reasons)];
}

function sameNumbers(left: readonly number[], right: readonly number[]): boolean {
  return left.length === right.length &&
    left.every((value, index) => value === right[index]);
}

export function assertLocalPlanResult(
  result: LocalPlanResult,
  request: NormalizedPlanRequest,
): void {
  const reasons = [
    ...localPlanCorrelationReasons(result, request),
    ...localPlanPlayableReasons(result, request),
  ];
  if (reasons.length > 0) {
    throw new Error(`local plan failed closed: ${[...new Set(reasons)].join(",")}`);
  }
}

function parseEvidence(value: unknown): NormalizedPlanEvidence {
  const root = record(value, "plan evidence");
  const identity = parsePlanIdentity(root.identity);
  const collision = record(root.collision, "collision evidence");
  const singularity = record(root.singularity, "singularity evidence");
  const limits = record(root.limits, "limit evidence");
  const runtime = record(root.runtime, "runtime evidence");
  const state = string(root.state, "evidence.state");
  if (!["unvalidated", "server_validated", "rejected", "stale"].includes(state)) {
    throw new Error(`invalid plan validation state ${JSON.stringify(state)}`);
  }
  return {
    schema: string(root.schema, "evidence.schema") as NormalizedPlanEvidence["schema"],
    requestId: string(root.request_id, "evidence.request_id"),
    identity,
    generatedAt: new Date(timestamp(root.generated_at, "evidence.generated_at")).toISOString(),
    plannerOwner: string(root.planner_owner, "evidence.planner_owner"),
    plannerBackend: string(root.planner_backend, "evidence.planner_backend"),
    state: state as NormalizedPlanEvidence["state"],
    accepted: bool(root.accepted, "evidence.accepted"),
    validationState: string(root.validation_state, "evidence.validation_state"),
    sampleCount: integer(root.sample_count, "evidence.sample_count"),
    collision: {
      state: checkState(collision.state, "collision.state"),
      checkState: string(collision.check_state, "collision.check_state"),
      points: array(collision.points, "collision.points").map(parseCollisionPoint),
    },
    singularity: {
      state: checkState(singularity.state, "singularity.state"),
      checkState: string(singularity.check_state, "singularity.check_state"),
      points: array(singularity.points, "singularity.points").map(parseSingularityPoint),
    },
    limits: {
      state: checkState(limits.state, "limits.state"),
      checkState: string(limits.check_state, "limits.check_state"),
      points: array(limits.points, "limits.points").map(parseUnreachablePoint),
    },
    runtime: {
      state: checkState(runtime.state, "runtime.state"),
      playable: bool(runtime.playable, "runtime.playable"),
    },
    correlationReasons: stringArray(
      root.correlation_reasons,
      "evidence.correlation_reasons",
    ),
  };
}

/** A robot description directory name. This layer checks the NAME; which
 *  robots exist is the server's answer (`domain/robots.ts`), so a simulator
 *  status naming a new robot is read rather than refused. */
function robotModelName(value: unknown, path: string): string {
  const name = string(value, path);
  if (!/^[A-Za-z0-9][A-Za-z0-9_-]*$/.test(name)) {
    throw new Error(`${path} is not a robot description name: ${JSON.stringify(name)}`);
  }
  return name;
}

function parsePlanIdentity(value: unknown): PlanIdentity {
  const root = record(value, "plan identity");
  const target = record(root.target, "plan target");
  const robotModel = robotModelName(root.robot_model, "plan.robot_model");
  const planTarget: PlanTargetIdentity = {
    profileId: string(target.profile_id, "plan.target.profile_id"),
    manifestRef: nullableString(target.manifest_ref, "plan.target.manifest_ref"),
  };
  return {
    projectId: string(root.project_id, "plan.project_id"),
    projectRevision: integer(root.project_revision, "plan.project_revision"),
    programId: string(root.program_id, "plan.program_id"),
    programRevision: integer(root.program_revision, "plan.program_revision"),
    planId: string(root.plan_id, "plan.plan_id"),
    planRevision: integer(root.plan_revision, "plan.plan_revision"),
    programHash: string(root.program_hash, "plan.program_hash"),
    robotModel,
    target: planTarget,
  };
}

function parseCollisionPoint(value: unknown): CollisionPoint {
  const root = record(value, "collision point");
  return {
    rowIndex: integer(root.row_index, "collision row"),
    waypointIndex: integer(root.waypoint_index, "collision waypoint"),
    linkA: string(root.link_a, "collision link_a"),
    linkB: string(root.link_b, "collision link_b"),
    depthM: finiteNumber(root.depth_m, "collision depth"),
    affectedJoints: stringArray(root.affected_joints, "collision joints"),
    positionsRad: optionalNumberArray(root.positions_rad, "collision positions"),
  };
}

function parseSingularityPoint(value: unknown): SingularityPoint {
  const root = record(value, "singularity point");
  return {
    rowIndex: integer(root.row_index, "singularity row"),
    waypointIndex: integer(root.waypoint_index, "singularity waypoint"),
    state: string(root.state, "singularity point state"),
    appliedVelocityScale: finiteNumber(
      root.applied_velocity_scale,
      "singularity velocity scale",
    ),
    limitingJoint: optionalString(root.limiting_joint) || undefined,
    affectedJoints: stringArray(root.affected_joints, "singularity joints"),
    positionsRad: optionalNumberArray(root.positions_rad, "singularity positions"),
  };
}

function parseUnreachablePoint(value: unknown): UnreachablePoint {
  const root = record(value, "unreachable point");
  return {
    rowIndex: integer(root.row_index, "unreachable row"),
    waypointIndex: integer(root.waypoint_index, "unreachable waypoint"),
    reason: string(root.reason, "unreachable reason"),
    joint: optionalString(root.joint) || undefined,
    positionsRad: optionalNumberArray(root.positions_rad, "unreachable positions"),
  };
}

function rowPhase(value: unknown): RowPhase {
  const phase = string(value, "motion_phase");
  return ["approach", "weld", "retract", "travel", "home"].includes(phase)
    ? phase as RowPhase
    : "travel";
}

function checkState(value: unknown, name: string): CheckState {
  const state = string(value, name);
  if (state !== "unavailable" && state !== "clear" && state !== "blocked") {
    throw new Error(`${name} is invalid`);
  }
  return state;
}

function record(value: unknown, name: string): Record<string, unknown> {
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    throw new Error(`${name} must be an object`);
  }
  return value as Record<string, unknown>;
}

function array(value: unknown, name: string): unknown[] {
  if (!Array.isArray(value)) throw new Error(`${name} must be an array`);
  return value;
}

function string(value: unknown, name: string): string {
  if (typeof value !== "string" || value.trim() === "") {
    throw new Error(`${name} must be a non-empty string`);
  }
  return value;
}

function optionalString(value: unknown): string {
  return typeof value === "string" ? value : "";
}

function nullableString(value: unknown, name: string): string | null {
  if (value === null || value === undefined || value === "") return null;
  if (typeof value !== "string") throw new Error(`${name} must be a string or null`);
  return value;
}

function bool(value: unknown, name: string): boolean {
  if (typeof value !== "boolean") throw new Error(`${name} must be boolean`);
  return value;
}

function integer(value: unknown, name: string): number {
  if (!Number.isSafeInteger(value)) throw new Error(`${name} must be a safe integer`);
  return value as number;
}

function finiteNumber(value: unknown, name: string): number {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(`${name} must be finite`);
  }
  return value;
}

function timestamp(value: unknown, name: string): number {
  if (typeof value !== "string") throw new Error(`${name} must be an ISO timestamp`);
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${name} must be an ISO timestamp`);
  return parsed;
}

function stringArray(value: unknown, name: string): string[] {
  return array(value, name).map((entry, index) => string(entry, `${name} ${index}`));
}

function numberTuple(value: unknown, length: number, name: string): number[] {
  const values = array(value, name);
  if (values.length !== length) throw new Error(`${name} must contain ${length} values`);
  return values.map((entry, index) => finiteNumber(entry, `${name} ${index}`));
}

function optionalNumberArray(value: unknown, name: string): number[] | undefined {
  if (value === undefined || value === null) return undefined;
  return array(value, name).map((entry, index) => finiteNumber(entry, `${name} ${index}`));
}

function loopbackHost(host: string): boolean {
  const normalized = host.toLowerCase().replace(/^\[|\]$/g, "");
  return normalized === "localhost" ||
    normalized === "::1" ||
    normalized.startsWith("127.");
}

function positiveInteger(value: number): boolean {
  return Number.isSafeInteger(value) && value > 0;
}

function validTime(value: number): boolean {
  return Number.isFinite(value) && value >= 0;
}

function nonempty(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0;
}
