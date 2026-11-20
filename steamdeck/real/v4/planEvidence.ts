// Version-tolerant thin-client view of the motion-server plan report.  The
// motion server remains the authority; clients only normalize evidence that
// is actually present and keep missing evidence explicitly unavailable.

export type RobotV4PlanUnreachablePoint = {
  joint?: string;
  joint_value_rad?: number;
  positions_rad?: number[];
  reason: string;
  row_index: number;
  tcp_pose: number[];
  time_s: number;
  waypoint_index: number;
};

export type RobotV4PlanCollisionPoint = {
  affected_joints: string[];
  depth_m: number;
  link_a: string;
  link_b: string;
  positions_rad?: number[];
  row_index: number;
  tcp_pose: number[];
  time_s: number;
  waypoint_index: number;
};

export type RobotV4PlanSingularityPoint = {
  affected_joints: string[];
  applied_velocity_scale: number;
  condition_number?: number;
  limiting_joint?: string;
  manipulability?: number;
  positions_rad?: number[];
  row_index: number;
  sigma_min?: number;
  state?: string;
  tcp_pose: number[];
  time_s: number;
  waypoint_index: number;
};

export type RobotV4PlanEvidenceSummary = {
  affectedJoints: string[];
  collisionEvidenceAvailable: boolean;
  collisionPoints: RobotV4PlanCollisionPoint[];
  jointSpeedLimitsApplied: boolean | null;
  jointSpeedLimitsRadS: number[];
  maxJointSpeedsRadS: number[];
  minAppliedVelocityScale: number | null;
  playEnabled: boolean | null;
  programName: string;
  reachabilityEvidenceAvailable: boolean;
  sampleCount: number;
  singularityEvidenceAvailable: boolean;
  singularityPoints: RobotV4PlanSingularityPoint[];
  unreachablePoints: RobotV4PlanUnreachablePoint[];
};

export function summarizeRobotV4PlanEvidence(plan: unknown): RobotV4PlanEvidenceSummary {
  const record = isRecord(plan) ? plan : {};
  const rawCollisionPoints = Array.isArray(record.collision_points) ? record.collision_points : null;
  const rawSingularityPoints = Array.isArray(record.singularity_points) ? record.singularity_points : null;
  const rawUnreachablePoints = Array.isArray(record.unreachable_points) ? record.unreachable_points : null;
  const collisionEvidenceAvailable = rawCollisionPoints !== null;
  const singularityEvidenceAvailable = rawSingularityPoints !== null;
  const unreachableEvidenceAvailable = rawUnreachablePoints !== null;
  const collisionPoints: RobotV4PlanCollisionPoint[] = rawCollisionPoints
    ? rawCollisionPoints.flatMap(normalizeCollisionPoint)
    : [];
  const unreachablePoints: RobotV4PlanUnreachablePoint[] = rawUnreachablePoints
    ? rawUnreachablePoints.flatMap(normalizeUnreachablePoint)
    : [];
  const singularityPoints: RobotV4PlanSingularityPoint[] = rawSingularityPoints
    ? rawSingularityPoints.flatMap(normalizeSingularityPoint)
    : [];
  const affectedJoints = uniqueSorted([
    ...collisionPoints.flatMap((point) => point.affected_joints),
    ...unreachablePoints.flatMap((point) => point.joint ? [point.joint] : []),
    ...singularityPoints.flatMap((point) => [
      ...point.affected_joints,
      ...(point.limiting_joint ? [point.limiting_joint] : []),
    ]),
  ]);
  const scales = singularityPoints
    .map((point) => point.applied_velocity_scale)
    .filter(Number.isFinite);
  const reportedMinimumScale = optionalFiniteNumber(record.minimum_applied_velocity_scale);
  return {
    affectedJoints,
    collisionEvidenceAvailable,
    collisionPoints,
    jointSpeedLimitsApplied: typeof record.joint_speed_limits_applied === "boolean"
      ? record.joint_speed_limits_applied
      : null,
    jointSpeedLimitsRadS: numberArray(record.joint_speed_limits_rad_s),
    maxJointSpeedsRadS: numberArray(record.max_joint_speeds_rad_s),
    minAppliedVelocityScale: scales.length > 0
      ? Math.min(...scales)
      : reportedMinimumScale === undefined
        ? null
        : Math.max(0, Math.min(1, reportedMinimumScale)),
    playEnabled: typeof record.play_enabled === "boolean" ? record.play_enabled : null,
    programName: String(record.program_name || record.program_id || "diagnostic plan"),
    reachabilityEvidenceAvailable: collisionEvidenceAvailable
      && unreachableEvidenceAvailable
      && typeof record.collision_check_state === "string",
    sampleCount: Number(record.planned_sample_count) || (Array.isArray(record.trajectory) ? record.trajectory.length : 0),
    singularityEvidenceAvailable,
    singularityPoints,
    unreachablePoints,
  };
}

function normalizeCollisionPoint(value: unknown): RobotV4PlanCollisionPoint[] {
  if (!isRecord(value)) {
    return [];
  }
  return [{
    affected_joints: stringArray(value.affected_joints ?? value.collision_joints),
    depth_m: finiteNumber(value.depth_m),
    link_a: String(value.link_a || ""),
    link_b: String(value.link_b || ""),
    positions_rad: optionalNumberArray(value.positions_rad),
    row_index: integer(value.row_index),
    tcp_pose: numberArray(value.tcp_pose),
    time_s: finiteNumber(value.time_s),
    waypoint_index: integer(value.waypoint_index),
  }];
}

function normalizeUnreachablePoint(value: unknown): RobotV4PlanUnreachablePoint[] {
  if (!isRecord(value)) {
    return [];
  }
  const jointValue = Number(value.joint_value_rad);
  return [{
    joint: typeof value.joint === "string" && value.joint.trim() ? value.joint.trim() : undefined,
    joint_value_rad: Number.isFinite(jointValue) ? jointValue : undefined,
    positions_rad: optionalNumberArray(value.positions_rad),
    reason: String(value.reason || "unknown"),
    row_index: integer(value.row_index),
    tcp_pose: numberArray(value.tcp_pose),
    time_s: finiteNumber(value.time_s),
    waypoint_index: integer(value.waypoint_index),
  }];
}

function normalizeSingularityPoint(value: unknown): RobotV4PlanSingularityPoint[] {
  if (!isRecord(value)) {
    return [];
  }
  const scale = firstFiniteNumber(
    value.applied_velocity_scale,
    value.proportional_velocity_scale,
    value.velocity_scale,
  );
  if (scale === null) {
    return [];
  }
  const limitingJoint = normalizedJointName(value.limiting_joint)
    || normalizedJointName(value.singular_direction_joint);
  const affectedJoints = stringArray(value.affected_joints);
  if (limitingJoint && !affectedJoints.includes(limitingJoint)) {
    affectedJoints.push(limitingJoint);
    affectedJoints.sort((left, right) => left.localeCompare(right, undefined, { numeric: true }));
  }
  const singularValues = numberArray(value.jacobian_singular_values);
  return [{
    affected_joints: affectedJoints,
    applied_velocity_scale: Math.max(0, Math.min(1, scale)),
    condition_number: optionalFiniteNumber(value.condition_number)
      ?? optionalFiniteNumber(value.jacobian_condition_number),
    limiting_joint: limitingJoint,
    manipulability: optionalFiniteNumber(value.manipulability)
      ?? optionalFiniteNumber(value.jacobian_manipulability),
    positions_rad: optionalNumberArray(value.positions_rad),
    row_index: integer(value.row_index),
    sigma_min: optionalFiniteNumber(value.sigma_min)
      ?? (singularValues.length > 0 ? Math.min(...singularValues) : undefined),
    state: typeof value.state === "string"
      ? value.state
      : typeof value.singularity_class === "string"
        ? value.singularity_class
        : undefined,
    tcp_pose: numberArray(value.tcp_pose),
    time_s: finiteNumber(value.time_s),
    waypoint_index: integer(value.waypoint_index),
  }];
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function numberArray(value: unknown): number[] {
  return Array.isArray(value) ? value.map(Number).filter(Number.isFinite) : [];
}

function optionalNumberArray(value: unknown): number[] | undefined {
  const values = numberArray(value);
  return values.length > 0 ? values : undefined;
}

function stringArray(value: unknown): string[] {
  return Array.isArray(value)
    ? uniqueSorted(value.map(String).map((entry) => entry.trim()).filter(Boolean))
    : [];
}

function normalizedJointName(value: unknown): string | undefined {
  if (typeof value !== "string") {
    return undefined;
  }
  const normalized = value.trim();
  return normalized && normalized.toLowerCase() !== "none" ? normalized : undefined;
}

function finiteNumber(value: unknown): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
}

function optionalFiniteNumber(value: unknown): number | undefined {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : undefined;
}

function firstFiniteNumber(...values: unknown[]): number | null {
  for (const value of values) {
    const parsed = Number(value);
    if (Number.isFinite(parsed)) {
      return parsed;
    }
  }
  return null;
}

function integer(value: unknown): number {
  const parsed = Number(value);
  return Number.isInteger(parsed) ? parsed : -1;
}

function uniqueSorted(values: string[]): string[] {
  return [...new Set(values)].sort((left, right) => left.localeCompare(right, undefined, { numeric: true }));
}
