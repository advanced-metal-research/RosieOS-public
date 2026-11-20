/**
 * Span arithmetic for rails mode, ported from the weld planner
 * (`weld_planner/v1/ui/src/span.ts` and `state/decisions.ts`). A span lives
 * in joint `s` (0..1 around the joint); fraction 0 and 1 are the same point
 * on a closed joint, so a run with start > end wraps through the join.
 * Copied, not imported: the two trees typecheck under different tsconfigs.
 */

export interface Span {
  start_s: number;
  end_s: number;
}

const clamp01 = (v: number) => Math.min(1, Math.max(0, v));

export const runFraction = (lo: number, hi: number): number =>
  hi < lo ? 1 - lo + hi : hi - lo;

export const spanFraction = (span: Span): number => runFraction(span.start_s, span.end_s);

/** Joint position at `fraction` along the span; the ends return exactly. */
export function sAt(span: Span, fraction: number): number {
  const f = clamp01(fraction);
  if (f <= 0) return span.start_s;
  if (f >= 1) return span.end_s;
  const s = span.start_s + f * spanFraction(span);
  return s > 1 ? s - 1 : s;
}

export const wrapsThroughZero = (span: Span): boolean => span.end_s < span.start_s;

export function containsS(span: Span, s: number): boolean {
  return wrapsThroughZero(span)
    ? s >= span.start_s || s <= span.end_s
    : s >= span.start_s && s <= span.end_s;
}

/** Inverse of `sAt`; positions outside the span clamp to its ends. */
export function fractionAt(span: Span, s: number): number {
  const length = spanFraction(span);
  if (length <= 0) return 0;
  let offset = s - span.start_s;
  if (offset < 0) offset += 1;
  return clamp01(offset / length);
}

export const subSpan = (span: Span, lo: number, hi: number): Span => ({
  start_s: sAt(span, lo),
  end_s: sAt(span, hi),
});

/** The clearance verdict, reduced to what the popup offers. */
export interface CoverageSummary {
  /** In weld fraction, for drawing on the profiles' axis. */
  clearRuns: number[][];
  blockedSpans: number[][];
  /** The longest clear run as an `entry.span`, ready to trim to. */
  trimSpan: Span | null;
  trimMm: number;
  seamMm: number;
  clearMm: number;
}

export function summarise(
  clearance: {
    seam_length_m: number;
    clear_runs?: number[][];
    blocked_spans?: number[][];
  },
  span: Span,
): CoverageSummary {
  const lengthM = clearance.seam_length_m;
  const runs = clearance.clear_runs ?? [];
  let longest: number[] | null = null;
  for (const run of runs) {
    if (!longest || runFraction(run[0]!, run[1]!) > runFraction(longest[0]!, longest[1]!)) {
      longest = run;
    }
  }
  const clearMm = runs.reduce((sum, [lo, hi]) => sum + runFraction(lo!, hi!), 0) * lengthM * 1000;
  return {
    clearRuns: runs,
    blockedSpans: clearance.blocked_spans ?? [],
    trimSpan: longest ? subSpan(span, longest[0]!, longest[1]!) : null,
    trimMm: longest ? runFraction(longest[0]!, longest[1]!) * lengthM * 1000 : 0,
    seamMm: lengthM * 1000,
    clearMm,
  };
}
