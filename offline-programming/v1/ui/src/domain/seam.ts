import type {
  AuthoredSeam,
  OfflineProgrammingProject,
  Vec3,
} from "./project.ts";

export const MIN_REMAINING_SEAM_M = 0.0001;

export interface SeamTrimLimits {
  totalLengthM: number;
  startOffsetM: number;
  endOffsetM: number;
  remainingLengthM: number;
  maxStartOffsetM: number;
  maxEndOffsetM: number;
}

/**
 * Point at a normalized 0..1 arc-length parameter. Copied from GradientOS
 * `samplePointOnPolyline` in `web-ui/src/App.tsx`.
 */
export function pointOnPolylineNormalizedS(
  points: readonly Vec3[],
  s: number,
): Vec3 | null {
  if (points.length === 0) return null;
  if (points.length === 1) return copyPoint(points[0]!);
  const cumulativeM = [0];
  for (let index = 1; index < points.length; index += 1) {
    const left = points[index - 1]!;
    const right = points[index]!;
    cumulativeM.push(
      cumulativeM[index - 1]! +
        Math.hypot(right[0] - left[0], right[1] - left[1], right[2] - left[2]),
    );
  }
  const totalLengthM = cumulativeM.at(-1) ?? 0;
  if (totalLengthM <= 1e-9) return copyPoint(points[0]!);
  return pointAtDistance(points, cumulativeM, clamp(finite(s), 0, 1) * totalLengthM);
}

/**
 * Even samples along a normalized window. GradientOS `sampleSegmentOnPolyline`.
 */
export function samplePolylineNormalizedWindow(
  points: readonly Vec3[],
  startS: number,
  endS: number,
  sampleCount = 28,
): Vec3[] {
  const a = clamp(finite(startS), 0, 1);
  const b = clamp(finite(endS), 0, 1);
  const s0 = Math.min(a, b);
  const s1 = Math.max(a, b);
  const count = Math.max(2, sampleCount);
  const sampled: Vec3[] = [];
  for (let index = 0; index < count; index += 1) {
    const point = pointOnPolylineNormalizedS(
      points,
      s0 + (s1 - s0) * (index / (count - 1)),
    );
    if (point) sampled.push(point);
  }
  return sampled;
}

export function polylineLengthM(points: readonly Vec3[]): number {
  let lengthM = 0;
  for (let index = 1; index < points.length; index += 1) {
    const left = points[index - 1]!;
    const right = points[index]!;
    lengthM += Math.hypot(
      right[0] - left[0],
      right[1] - left[1],
      right[2] - left[2],
    );
  }
  return lengthM;
}

export function seamTrimLimits(seam: AuthoredSeam): SeamTrimLimits {
  const totalLengthM = polylineLengthM(seam.geometry.pointsM);
  const minimumRemainingM = Math.min(MIN_REMAINING_SEAM_M, totalLengthM);
  const maximumCombinedTrimM = Math.max(0, totalLengthM - minimumRemainingM);
  const startOffsetM = clamp(
    finite(seam.trim.startOffsetM),
    0,
    maximumCombinedTrimM,
  );
  const endOffsetM = clamp(
    finite(seam.trim.endOffsetM),
    0,
    Math.max(0, maximumCombinedTrimM - startOffsetM),
  );
  return {
    totalLengthM,
    startOffsetM,
    endOffsetM,
    remainingLengthM: Math.max(0, totalLengthM - startOffsetM - endOffsetM),
    maxStartOffsetM: Math.max(0, maximumCombinedTrimM - endOffsetM),
    maxEndOffsetM: Math.max(0, maximumCombinedTrimM - startOffsetM),
  };
}

export function setSeamTrimOffsetM(
  seam: AuthoredSeam,
  side: "start" | "end",
  requestedOffsetM: number,
): void {
  const limits = seamTrimLimits(seam);
  const value = Math.max(0, finite(requestedOffsetM));
  if (side === "start") {
    seam.trim.startOffsetM = Math.min(value, limits.maxStartOffsetM);
    seam.trim.endOffsetM = limits.endOffsetM;
  } else {
    seam.trim.startOffsetM = limits.startOffsetM;
    seam.trim.endOffsetM = Math.min(value, limits.maxEndOffsetM);
  }
}

export function trimPolylineByOffsetsM(
  points: readonly Vec3[],
  requestedStartOffsetM: number,
  requestedEndOffsetM: number,
): Vec3[] {
  if (points.length < 2) return points.map(copyPoint);
  const totalLengthM = polylineLengthM(points);
  if (totalLengthM <= 0) return points.map(copyPoint);

  const minimumRemainingM = Math.min(MIN_REMAINING_SEAM_M, totalLengthM);
  const startOffsetM = clamp(
    finite(requestedStartOffsetM),
    0,
    Math.max(0, totalLengthM - minimumRemainingM),
  );
  const endOffsetM = clamp(
    finite(requestedEndOffsetM),
    0,
    Math.max(0, totalLengthM - minimumRemainingM - startOffsetM),
  );
  const endDistanceM = totalLengthM - endOffsetM;
  const cumulativeM = [0];
  for (let index = 1; index < points.length; index += 1) {
    const left = points[index - 1]!;
    const right = points[index]!;
    cumulativeM.push(
      cumulativeM[index - 1]! +
      Math.hypot(
        right[0] - left[0],
        right[1] - left[1],
        right[2] - left[2],
      ),
    );
  }

  const trimmed: Vec3[] = [pointAtDistance(points, cumulativeM, startOffsetM)];
  for (let index = 1; index < points.length - 1; index += 1) {
    const distanceM = cumulativeM[index]!;
    if (distanceM > startOffsetM && distanceM < endDistanceM) {
      trimmed.push(copyPoint(points[index]!));
    }
  }
  trimmed.push(pointAtDistance(points, cumulativeM, endDistanceM));
  return trimmed;
}

export function needsPhysicalSeamTrimMigration(
  project: OfflineProgrammingProject,
): boolean {
  return project.seams.some((seam) => {
    const trim = seam.trim as unknown as Record<string, unknown>;
    return !Number.isFinite(Number(trim.startOffsetM)) ||
      !Number.isFinite(Number(trim.endOffsetM));
  });
}

export function migrateLegacySeamTrims(
  project: OfflineProgrammingProject,
): boolean {
  let migrated = false;
  for (const seam of project.seams) {
    const trim = seam.trim as unknown as Record<string, unknown>;
    if (
      Number.isFinite(Number(trim.startOffsetM)) &&
      Number.isFinite(Number(trim.endOffsetM))
    ) {
      const limits = seamTrimLimits(seam);
      seam.trim = {
        startOffsetM: limits.startOffsetM,
        endOffsetM: limits.endOffsetM,
      };
      continue;
    }
    const totalLengthM = polylineLengthM(seam.geometry.pointsM);
    const legacyStart = clamp(finite(trim.start), 0, 1);
    const legacyEnd = clamp(finite(trim.end, 1), legacyStart, 1);
    seam.trim = {
      startOffsetM: legacyStart * totalLengthM,
      endOffsetM: (1 - legacyEnd) * totalLengthM,
    };
    const limits = seamTrimLimits(seam);
    seam.trim = {
      startOffsetM: limits.startOffsetM,
      endOffsetM: limits.endOffsetM,
    };
    migrated = true;
  }
  return migrated;
}

function pointAtDistance(
  points: readonly Vec3[],
  cumulativeM: readonly number[],
  distanceM: number,
): Vec3 {
  if (distanceM <= 0) return copyPoint(points[0]!);
  const totalLengthM = cumulativeM.at(-1) ?? 0;
  if (distanceM >= totalLengthM) return copyPoint(points.at(-1)!);
  for (let index = 1; index < points.length; index += 1) {
    const segmentEndM = cumulativeM[index]!;
    if (distanceM > segmentEndM) continue;
    const segmentStartM = cumulativeM[index - 1]!;
    const segmentLengthM = segmentEndM - segmentStartM;
    const ratio = segmentLengthM > 0
      ? (distanceM - segmentStartM) / segmentLengthM
      : 0;
    const left = points[index - 1]!;
    const right = points[index]!;
    return [
      left[0] + (right[0] - left[0]) * ratio,
      left[1] + (right[1] - left[1]) * ratio,
      left[2] + (right[2] - left[2]) * ratio,
    ];
  }
  return copyPoint(points.at(-1)!);
}

function copyPoint(point: readonly [number, number, number]): Vec3 {
  return [point[0], point[1], point[2]];
}

function finite(value: unknown, fallback = 0): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.max(minimum, Math.min(maximum, value));
}
