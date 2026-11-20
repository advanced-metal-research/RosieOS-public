import { interpolateTcpPose } from "./so3.ts";
import type {
  AuthoredProgramRow,
  JointVector,
  PlanTrajectorySample,
  TcpPose,
} from "./project.ts";

const MIN_SEGMENT_DURATION_S = 0.25;
const MOTION_EPSILON = 1e-9;

export interface TrajectoryMotionSummary {
  durationS: number;
  tcpTravelM: number;
  jointTravelRad: number;
  robotMoves: boolean;
}

/**
 * Browser-preview trajectory built straight from the authored rows.
 *
 * Orientation is interpolated on the SO(3) geodesic (domain/so3.ts), the same
 * path the Motion Server plans, so what the operator previews is what the robot
 * executes. Translation and joints stay componentwise lerps.
 */
export function previewTrajectory(rows: AuthoredProgramRow[]): PlanTrajectorySample[] {
  const enabled = rows.filter((row) => row.enabled);
  if (enabled.length === 0) return [];
  if (enabled.length === 1) return [sampleFromRow(enabled[0]!, 0, 0, 0)];
  const samples: PlanTrajectorySample[] = [];
  let timeS = 0;
  for (let index = 0; index < enabled.length - 1; index += 1) {
    const from = enabled[index]!;
    const to = enabled[index + 1]!;
    const distance = xyzDistance(from.targetTcpPose, to.targetTcpPose);
    const velocity = Math.max(0.01, to.tcpVelocityMps || from.tcpVelocityMps || 0.05);
    const duration = Math.max(MIN_SEGMENT_DURATION_S, distance / velocity);
    const steps = Math.max(3, Math.ceil(distance * 50));
    for (let step = 0; step < steps; step += 1) {
      if (index > 0 && step === 0) continue;
      const alpha = step / (steps - 1);
      samples.push({
        index: samples.length,
        rowIndex: alpha < 0.5 ? index : index + 1,
        seamId: alpha < 0.5 ? from.seamId : to.seamId,
        phase: alpha < 0.5 ? from.phase : to.phase,
        weldOn: alpha < 0.5 ? from.weldEnabled : to.weldEnabled,
        timeS: timeS + alpha * duration,
        tcpPose: interpolateTcpPose(from.targetTcpPose, to.targetTcpPose, alpha) as TcpPose,
        jointPositionsRad: lerpTuple(from.targetJointsRad, to.targetJointsRad, alpha) as JointVector,
      });
    }
    timeS += duration;
  }
  return samples;
}

export function sampleAtTime(
  trajectory: PlanTrajectorySample[],
  timeS: number,
): { sample: PlanTrajectorySample | null; done: boolean } {
  if (trajectory.length === 0) return { sample: null, done: true };
  const last = trajectory.at(-1)!;
  if (timeS >= last.timeS) return { sample: last, done: true };
  // Binary search for the first sample at or after the clock (lower_bound).
  // The clock is monotonic per frame; this replaces an O(n) findIndex scan.
  // The guards above already handled empty trajectories and timeS >= last.
  const target = Math.max(0, timeS);
  let lo = 0;
  let hi = trajectory.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (trajectory[mid]!.timeS >= target) hi = mid;
    else lo = mid + 1;
  }
  const rightIndex = lo;
  if (rightIndex <= 0) return { sample: trajectory[0]!, done: false };
  const left = trajectory[rightIndex - 1]!;
  const right = trajectory[rightIndex]!;
  const span = right.timeS - left.timeS;
  const alpha = span > 0 ? (timeS - left.timeS) / span : 0;
  return {
    sample: {
      ...left,
      index: left.index,
      rowIndex: alpha < 0.5 ? left.rowIndex : right.rowIndex,
      seamId: alpha < 0.5 ? left.seamId : right.seamId,
      phase: alpha < 0.5 ? left.phase : right.phase,
      weldOn: alpha < 0.5 ? left.weldOn : right.weldOn,
      timeS,
      tcpPose: interpolateTcpPose(left.tcpPose, right.tcpPose, alpha) as TcpPose,
      jointPositionsRad: lerpTuple(left.jointPositionsRad, right.jointPositionsRad, alpha) as JointVector,
    },
    done: false,
  };
}

/** Spreadsheet row id for the move a trajectory sample is currently on. */
export function rowIdForTrajectorySample(
  rows: readonly Pick<AuthoredProgramRow, "id">[],
  sample: PlanTrajectorySample | null | undefined,
): string | null {
  if (!sample) return null;
  if (!Number.isSafeInteger(sample.rowIndex) || sample.rowIndex < 0) return null;
  return rows[sample.rowIndex]?.id ?? null;
}

/** Spreadsheet row id under the playback clock at `timeS`. */
export function rowIdAtPlaybackTime(
  rows: readonly Pick<AuthoredProgramRow, "id">[],
  trajectory: PlanTrajectorySample[],
  timeS: number,
): string | null {
  return rowIdForTrajectorySample(rows, sampleAtTime(trajectory, timeS).sample);
}

export type PlaybackStepDirection = "backward" | "forward";

const STEP_TIME_EPSILON_S = 1e-9;

/**
 * Start times (s) of each distinct `rowIndex` run in trajectory order.
 * One boundary = one spreadsheet/trajectory move.
 */
export function moveBoundaryTimes(
  trajectory: readonly PlanTrajectorySample[],
): number[] {
  if (trajectory.length === 0) return [];
  const times: number[] = [];
  let lastRowIndex: number | null = null;
  for (const sample of trajectory) {
    if (sample.rowIndex !== lastRowIndex) {
      times.push(sample.timeS);
      lastRowIndex = sample.rowIndex;
    }
  }
  return times;
}

/**
 * Destination playback clock after stepping one move forward/back from `timeS`.
 * Returns null when the step is a no-op (empty trajectory, already at start/end).
 *
 * The returned time is the *end* of the step segment. Callers must animate along
 * the accepted trajectory from `timeS` to this target (forward in time, or reverse
 * for backward) — never teleport — so step controls stay transferable to a robot.
 *
 * Forward: complete the current move (next boundary, or trajectory end on the last move).
 * Backward: if mid-move, return to the current move start; if already there, reverse
 * through the previous move to its start.
 */
export function stepPlaybackTime(
  trajectory: readonly PlanTrajectorySample[],
  timeS: number,
  direction: PlaybackStepDirection,
): number | null {
  const boundaries = moveBoundaryTimes(trajectory);
  if (boundaries.length === 0) return null;
  const t = Math.max(0, timeS);
  const endTime = trajectory.at(-1)!.timeS;

  let currentIdx = 0;
  for (let index = 0; index < boundaries.length; index += 1) {
    if (boundaries[index]! <= t + STEP_TIME_EPSILON_S) currentIdx = index;
    else break;
  }

  if (direction === "forward") {
    if (currentIdx + 1 < boundaries.length) {
      const next = boundaries[currentIdx + 1]!;
      return next > t + STEP_TIME_EPSILON_S ? next : null;
    }
    // Last move: step through to the planned trajectory end, not a no-op at the start.
    return endTime > t + STEP_TIME_EPSILON_S ? endTime : null;
  }

  const currentStart = boundaries[currentIdx]!;
  if (t > currentStart + STEP_TIME_EPSILON_S) return currentStart;
  if (currentIdx <= 0) return null;
  return boundaries[currentIdx - 1]!;
}

export function canStepPlayback(
  trajectory: readonly PlanTrajectorySample[],
  timeS: number,
  direction: PlaybackStepDirection,
): boolean {
  return stepPlaybackTime(trajectory, timeS, direction) !== null;
}

export function trajectoryMotionSummary(
  trajectory: readonly PlanTrajectorySample[],
): TrajectoryMotionSummary {
  let tcpTravelM = 0;
  let jointTravelRad = 0;
  for (let index = 1; index < trajectory.length; index += 1) {
    const previous = trajectory[index - 1]!;
    const current = trajectory[index]!;
    tcpTravelM += xyzDistance(previous.tcpPose, current.tcpPose);
    jointTravelRad += tupleDistance(
      previous.jointPositionsRad,
      current.jointPositionsRad,
    );
  }
  return {
    durationS: trajectory.at(-1)?.timeS ?? 0,
    tcpTravelM,
    jointTravelRad,
    robotMoves: jointTravelRad > MOTION_EPSILON,
  };
}

function sampleFromRow(row: AuthoredProgramRow, index: number, rowIndex: number, timeS: number): PlanTrajectorySample {
  return {
    index,
    rowIndex,
    seamId: row.seamId,
    phase: row.phase,
    weldOn: row.weldEnabled,
    timeS,
    tcpPose: [...row.targetTcpPose],
    jointPositionsRad: [...row.targetJointsRad],
  };
}

function xyzDistance(left: TcpPose, right: TcpPose): number {
  return Math.hypot(right[0] - left[0], right[1] - left[1], right[2] - left[2]);
}

function tupleDistance(left: readonly number[], right: readonly number[]): number {
  return Math.hypot(...left.map((value, index) => (right[index] ?? value) - value));
}

function lerpTuple(left: readonly number[], right: readonly number[], alpha: number): number[] {
  const t = Math.max(0, Math.min(1, alpha));
  return left.map((value, index) => value + ((right[index] ?? value) - value) * t);
}
