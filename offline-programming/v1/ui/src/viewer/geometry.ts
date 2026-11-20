import type { MotionClass, Vec3 } from "../simulation/contracts.ts";

export interface MotionPathPoint {
  positionM: Vec3;
  /** Classifies the segment beginning at this point. */
  motion: MotionClass;
}

export interface MotionPathRun {
  motion: MotionClass;
  pointsM: Vec3[];
}

export interface Bounds3 {
  minimum: Vec3;
  maximum: Vec3;
  center: Vec3;
  size: Vec3;
}

/** Split a polyline into colorable runs while retaining transition vertices. */
export function buildMotionRuns(points: readonly MotionPathPoint[]): MotionPathRun[] {
  if (points.length < 2) return [];
  assertFinitePoints(points);
  const runs: MotionPathRun[] = [];
  let motion = points[0]!.motion;
  let run: Vec3[] = [copyVec3(points[0]!.positionM)];

  for (let index = 0; index < points.length - 1; index += 1) {
    const from = points[index]!;
    const to = points[index + 1]!;
    if (from.motion !== motion) {
      if (run.length >= 2) runs.push({ motion, pointsM: run });
      motion = from.motion;
      run = [copyVec3(from.positionM)];
    }
    run.push(copyVec3(to.positionM));
  }
  if (run.length >= 2) runs.push({ motion, pointsM: run });
  return runs;
}

export function polylineLengthM(points: readonly Vec3[]): number {
  let length = 0;
  for (let index = 1; index < points.length; index += 1) {
    length += distance3(points[index - 1]!, points[index]!);
  }
  return length;
}

export function boundsOfPoints(points: readonly Vec3[]): Bounds3 | null {
  if (points.length === 0) return null;
  points.forEach(assertFiniteVec3);
  const first = points[0]!;
  let minX = first[0];
  let minY = first[1];
  let minZ = first[2];
  let maxX = first[0];
  let maxY = first[1];
  let maxZ = first[2];
  for (const point of points.slice(1)) {
    minX = Math.min(minX, point[0]);
    minY = Math.min(minY, point[1]);
    minZ = Math.min(minZ, point[2]);
    maxX = Math.max(maxX, point[0]);
    maxY = Math.max(maxY, point[1]);
    maxZ = Math.max(maxZ, point[2]);
  }
  const minimum: Vec3 = [minX, minY, minZ];
  const maximum: Vec3 = [maxX, maxY, maxZ];
  const size: Vec3 = [maxX - minX, maxY - minY, maxZ - minZ];
  const center: Vec3 = [
    minX + size[0] / 2,
    minY + size[1] / 2,
    minZ + size[2] / 2,
  ];
  return { minimum, maximum, center, size };
}

export function distance3(left: Vec3, right: Vec3): number {
  return Math.hypot(
    right[0] - left[0],
    right[1] - left[1],
    right[2] - left[2],
  );
}

function assertFinitePoints(points: readonly MotionPathPoint[]): void {
  points.forEach((point) => assertFiniteVec3(point.positionM));
}

function assertFiniteVec3(point: Vec3): void {
  if (point.some((value) => !Number.isFinite(value))) {
    throw new Error("geometry point contains a non-finite coordinate");
  }
}

function copyVec3(point: Vec3): Vec3 {
  return [point[0], point[1], point[2]];
}
