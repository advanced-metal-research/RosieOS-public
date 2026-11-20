/**
 * What to do about the stretches the torch cannot reach — the weld planner's
 * coverage arithmetic (`weld_planner/v1/ui/src/state/decisions.ts`, itself the
 * twin of `python/weld_programmer/decisions.py`), ported for rails mode.
 *
 * Clearance reports in **fraction along the seam**; a weld's span is in **`s`
 * along the joint**, which may wrap through zero. Confuse the two and you get
 * spans that are plausible, ordered, inside [0, 1] and weld the wrong metal.
 *
 * Three choices, and they are not degrees of the same thing:
 *
 *   TRIM     weld the longest clear run and give up the rest
 *   SPLIT    weld every run worth striking an arc for, as separate entries
 *   ACCEPT   weld it as authored, fouling and all — a real answer, since the
 *            check is a model and the operator is looking at the part, but one
 *            that has to be RECORDED so reopening shows a decision rather than
 *            an unnoticed collision
 */

import type { RailsClearance } from "./client.ts";
import { restrict } from "./scalar-function.ts";
import type { ScalarFunction } from "./scalar-function.ts";
import { runFraction, subSpan } from "./span-math.ts";
import type { Span } from "./span-math.ts";

/** Matches the weld planner's `clearance.MIN_CLEAR_RUN_M`. Only a fallback:
 *  a clearance result states the floor it used, and that one wins. */
export const MIN_RUN_M = 0.005;

export interface CoverageOffer {
  state: "clear" | "partial" | "blocked";
  seamLengthM: number;
  usableLengthM: number;
  droppedLengthM: number;
  blockedLengthM: number;
  minRunM: number;
  /** In seam fraction, for narrowing the profiles onto each piece. */
  usableRuns: number[][];
  droppedRuns: number[][];
  /** In joint `s`, ready to become `entry.span`. */
  trim: Span | null;
  trimLengthM: number;
  split: Span[];
  splitLengthM: number;
}

// Wrap-aware. On a CLOSED weld a run may straddle the join and is reported
// with `start > end`; subtracting naively makes it negative, which then
// cancels part of another run in the sum — a coverage figure too low by
// however much wrapped, with nothing on screen to say so.
const sum = (runs: number[][]) =>
  runs.reduce((total, [lo, hi]) => total + runFraction(lo!, hi!), 0);

export function offer(
  clearance: RailsClearance,
  span: Span,
  minRunM?: number,
): CoverageOffer {
  const lengthM = clearance.seam_length_m;
  if (!(lengthM > 0)) throw new Error("a seam of zero length has nothing to decide about");
  const floor = minRunM ?? clearance.min_clear_run_m ?? MIN_RUN_M;

  const runs = clearance.clear_runs ?? [];
  const usableRuns = runs.filter(([lo, hi]) => runFraction(lo!, hi!) * lengthM >= floor);
  const droppedRuns = runs.filter(([lo, hi]) => runFraction(lo!, hi!) * lengthM < floor);

  const usableLengthM = sum(usableRuns) * lengthM;
  const droppedLengthM = sum(droppedRuns) * lengthM;
  const blockedLengthM = sum(clearance.blocked_spans ?? []) * lengthM;

  let longest: number[] | null = null;
  for (const run of usableRuns) {
    if (!longest || runFraction(run[0]!, run[1]!) > runFraction(longest[0]!, longest[1]!)) {
      longest = run;
    }
  }

  // `clear` means there is nothing to decide, and is distinguished from
  // `partial` so the flow can stay silent rather than splitting a weld with
  // no problem.
  const state =
    !runs.length || usableLengthM <= 0
      ? "blocked"
      : blockedLengthM + droppedLengthM <= 0
        ? "clear"
        : "partial";

  return {
    state,
    seamLengthM: lengthM,
    usableLengthM,
    droppedLengthM,
    blockedLengthM,
    minRunM: floor,
    usableRuns,
    droppedRuns,
    trim: longest ? subSpan(span, longest[0]!, longest[1]!) : null,
    trimLengthM: longest ? runFraction(longest[0]!, longest[1]!) * lengthM : 0,
    split: usableRuns.map(([lo, hi]) => subSpan(span, lo!, hi!)),
    splitLengthM: usableLengthM,
  };
}

/**
 * Whether a click on this seam should enter the RUN it landed on rather than
 * the seam as a whole.
 *
 * Yes whenever the seam is partial and there is a usable run to narrow to --
 * one is enough. This asked for two, on the reading that narrowing is only
 * interesting when there is a choice of pieces; but a seam with a single
 * clear stretch and blocked metal at each end is exactly the case where the
 * whole seam must not enter the list. It was entering it as "12% weldable",
 * fouling included, waiting for someone to notice and press Trim.
 *
 * A `clear` seam narrows to nothing, because there is nothing to leave out.
 */
export const narrowsToRun = (o: CoverageOffer): boolean =>
  o.state === "partial" && o.split.length >= 1;

/**
 * usable + dropped + blocked == the whole seam.
 *
 * The tripwire. Clear runs and blocked spans partition the seam by
 * construction, and usable/dropped partition the clear runs, so this holds
 * exactly — and it is what a fraction-versus-`s` mix-up cannot survive.
 */
export const conserves = (o: CoverageOffer): boolean =>
  Math.abs(o.usableLengthM + o.droppedLengthM + o.blockedLengthM - o.seamLengthM) < 1e-9;

/**
 * Every scalar function in a document, narrowed to a sub-range of the arc —
 * the weld planner's `restrictScalars`. A split carries the optimised
 * profiles onto each piece by narrowing them, not by searching again: a split
 * decides WHERE to weld, not how.
 */
export function restrictScalars<T>(node: T, lo: number, hi: number): T {
  if (Array.isArray(node)) {
    return node.map((item) => restrictScalars(item, lo, hi)) as unknown as T;
  }
  if (node && typeof node === "object") {
    const record = node as Record<string, unknown>;
    if (typeof record.kind === "string") {
      return restrict(record as unknown as ScalarFunction, lo, hi) as unknown as T;
    }
    return Object.fromEntries(
      Object.entries(record).map(([key, value]) => [key, restrictScalars(value, lo, hi)]),
    ) as unknown as T;
  }
  return node;
}
