import { JOINT_ORDER, type JointVector } from "../domain/project.ts";

export function normalizeJointName(value: string): string | null {
  const match = value.trim().toUpperCase().match(/^J(?:OINT[_ -]?)?(\d)(?:[AB])?$/);
  if (!match) return null;
  const index = Number(match[1]);
  return index >= 1 && index <= JOINT_ORDER.length ? `J${index}` : null;
}

export function canonicalJoints(
  positions: readonly number[] | undefined,
  names?: readonly string[],
): JointVector {
  const result = JOINT_ORDER.map(() => 0) as JointVector;
  if (!positions) return result;
  if (!names?.length) {
    positions.slice(0, JOINT_ORDER.length).forEach((value, index) => {
      result[index] = finite(value);
    });
    return result;
  }
  names.forEach((name, sourceIndex) => {
    const canonical = normalizeJointName(name);
    if (!canonical) return;
    const targetIndex = JOINT_ORDER.indexOf(canonical as (typeof JOINT_ORDER)[number]);
    if (targetIndex >= 0) result[targetIndex] = finite(positions[sourceIndex]);
  });
  return result;
}

export function canonicalJointRecord(positions: readonly number[]): Record<(typeof JOINT_ORDER)[number], number> {
  return Object.fromEntries(JOINT_ORDER.map((name, index) => [name, finite(positions[index])])) as
    Record<(typeof JOINT_ORDER)[number], number>;
}

function finite(value: unknown): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
}

