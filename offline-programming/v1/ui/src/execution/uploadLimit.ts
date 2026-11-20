/** The upload preflight's exact offending sample. This is a rejected plan,
 * not evidence that a drive has faulted or needs a reset. */
export interface UploadLimitViolation {
  kind: "position" | "velocity";
  segment: number;
  sample: number;
  axis: string;
  value: number;
  limit: number;
  unit: "rad" | "rad/s";
}

export function parseUploadLimitViolation(value: unknown): UploadLimitViolation | null {
  if (typeof value !== "object" || value === null || Array.isArray(value)) return null;
  const record = value as Record<string, unknown>;
  if (record.kind !== "position" && record.kind !== "velocity") return null;
  if (record.unit !== (record.kind === "position" ? "rad" : "rad/s")) return null;
  if (typeof record.axis !== "string" || !/^[A-Za-z][A-Za-z0-9_-]{0,63}$/.test(record.axis)) return null;
  if (typeof record.segment !== "number" || !Number.isSafeInteger(record.segment) || record.segment < 0 ||
    typeof record.sample !== "number" || !Number.isSafeInteger(record.sample) || record.sample < 0) return null;
  if (typeof record.value !== "number" || !Number.isFinite(record.value) ||
    typeof record.limit !== "number" || !Number.isFinite(record.limit)) return null;
  if (record.kind === "velocity" && record.limit < 0) return null;
  return { kind: record.kind, segment: record.segment, sample: record.sample, axis: record.axis,
    value: record.value, limit: record.limit, unit: record.kind === "position" ? "rad" : "rad/s" };
}

export function uploadLimitText(violation: UploadLimitViolation): string {
  // Six decimal places retain the small overshoot that would disappear if
  // both the offending value and bound were rounded to whole degrees.
  const degrees = (radians: number) => String(Number((radians * 180 / Math.PI).toFixed(6)));
  const unit = violation.kind === "position" ? "°" : "°/s";
  const subject = violation.kind === "position" ? "position" : "speed";
  const value = violation.kind === "velocity" ? Math.abs(violation.value) : violation.value;
  return `${violation.axis} planned ${subject} ${degrees(value)}${unit} is outside the connected machine's ${degrees(violation.limit)}${unit} limit. ` +
    "The trajectory was not loaded. Replan for the connected machine, then load again.";
}
