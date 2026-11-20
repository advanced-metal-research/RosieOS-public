/**
 * Rails weld authoring without a window: the angle search, the clearance
 * check, the narrowing of a blocked seam to one clear run, and the accepted
 * weld node. The browser's rails mode (index.ts) and any headless client (the
 * RosieOS MCP server) both call these, so a weld accepted in either is the
 * same node: one engine, as cell-sync.ts is for opening programs.
 *
 * Nothing here holds state or draws. Cancellation stays the caller's: every
 * await is followed by its `checkpoint`, which may throw (a stale
 * generation) or answer false (the entry is gone), exactly where the
 * browser's own `assertCurrent` and `entries.includes` checks stood.
 */
import type { WeldNode } from "../contracts/robot-v4-program-v2.ts";
import { seamWeldNodeFor } from "../program-v2/rails-node.ts";
import type {
  RailsClearance,
  RailsJoint,
  RailsOptimisePlan,
  RailsSearchKnobs,
  RailsSeam,
  RailsWeldPayload,
} from "./client.ts";
import { offer, restrictScalars, type CoverageOffer } from "./decisions.ts";
import { SHIPPED_SEARCH_PRESETS } from "./parameters.ts";
import { summarise, type CoverageSummary, type Span } from "./span-math.ts";

/** One seam's search result, cached under its seam id and parameters' digest. */
export interface SeamPlan {
  plan: RailsOptimisePlan;
  payload: RailsWeldPayload;
  clearance: RailsClearance;
  report: CoverageOffer;
}

/** The two seam-worker measurements authoring makes (client.ts). */
export interface SeamMeasurer {
  planTorchPath(step: Uint8Array, body: {
    joint: RailsJoint;
    seam: RailsSeam;
    torch: unknown;
    direction?: "forward" | "reverse";
    search: RailsSearchKnobs;
    default_standoff_mm: number;
  }): Promise<{ direction: string; plan: RailsOptimisePlan }>;
  checkTorchFits(step: Uint8Array, body: { joint: RailsJoint; seam: RailsSeam; torch: unknown; options?: unknown }): Promise<{ clearance: RailsClearance }>;
}

/** The fitted gun and the search knobs a measurement is made with. */
export interface SearchInputs {
  torch: unknown;
  search: RailsSearchKnobs;
  defaultStandoffMm: number;
}

/** After each await: throw to abandon, false to drop the result quietly. */
export type Checkpoint = () => boolean;

/** The search window's fallback knobs: the first shipped preset, which the
 *  parameter window itself falls back to (parameters/library.ts). */
export function shippedSearchKnobs(): RailsSearchKnobs {
  const values = SHIPPED_SEARCH_PRESETS[0]!.values;
  return {
    work_deviation_deg: Number(values.work_deviation_deg),
    travel_deviation_deg: Number(values.travel_deviation_deg),
    standoff_cost_deg_per_mm: Number(values.standoff_cost_deg_per_mm),
  };
}

/** The seam as the search left it: the chosen profiles, the chosen
 *  direction, the chosen span. Clearance has to be measured against the
 *  angles that were chosen rather than the ones that were drawn. */
export function posedSeam(seam: RailsSeam, payload: RailsWeldPayload): RailsSeam {
  const overrides: Record<string, unknown> = payload.overrides ?? {};
  const chosen = overrides.orientation;
  const orientation = chosen && typeof chosen === "object" && !Array.isArray(chosen)
    ? (chosen as Record<string, unknown>)
    : {};
  const authored = (seam.orientation ?? {}) as Record<string, unknown>;
  return {
    ...seam,
    orientation: { ...authored, ...orientation },
    ...(typeof overrides.direction === "string" ? { direction: overrides.direction } : {}),
    ...(payload.span ? { span: payload.span } : {}),
  };
}

/** The search's answer folded into a weld-list entry: profiles replace,
 *  direction only when it turned, stickout only when it moved. */
export function optimisedPayload(seam: RailsSeam, plan: RailsOptimisePlan): RailsWeldPayload {
  const overrides: Record<string, unknown> = {
    orientation: {
      work_angle_deg: plan.work_angle_deg,
      travel_angle_deg: plan.travel_angle_deg,
    },
  };
  if (plan.direction_changed && plan.direction) overrides.direction = plan.direction;
  if (plan.standoff_departure_mm > 0) {
    overrides.process = { wire: { stickout_mm: plan.standoff_mm } };
  }
  return {
    id: `rails-${seam.id}`,
    seam_ids: [seam.id],
    preset_id: null,
    overrides,
    span: null,
  };
}

/** The usable run a fraction along the seam lands in — or the nearest one, wrap-aware. */
export function runIndexAt(report: CoverageOffer, fraction: number): number {
  const circular = (a: number, b: number) => {
    const d = Math.abs(a - b) % 1;
    return Math.min(d, 1 - d);
  };
  let best = 0;
  let bestDistance = Infinity;
  report.usableRuns.forEach(([lo, hi], k) => {
    const inside = lo! <= hi!
      ? fraction >= lo! && fraction <= hi!
      : fraction >= lo! || fraction <= hi!;
    const distance = inside ? 0 : Math.min(circular(fraction, lo!), circular(fraction, hi!));
    if (distance < bestDistance) {
      bestDistance = distance;
      best = k;
    }
  });
  return best;
}

/**
 * Search one seam both ways round and keep the better coverage, then check
 * the torch fits at the chosen angles. The same metal answers differently
 * run the other way -- measured on `bracket_b`, three times the weld for
 * asking backwards -- and nothing but measuring both says which. Two calls
 * because one lattice answering both directions only pays while the
 * authored angle sits inside the deviation. Null when a checkpoint dropped it.
 */
export async function searchSeam(
  measurer: SeamMeasurer,
  step: Uint8Array,
  seam: RailsSeam,
  joint: RailsJoint,
  inputs: SearchInputs,
  checkpoint: Checkpoint = () => true,
  onSearched?: (direction: string, plan: RailsOptimisePlan) => void,
): Promise<SeamPlan | null> {
  const request = {
    joint,
    seam,
    torch: inputs.torch,
    search: inputs.search,
    default_standoff_mm: inputs.defaultStandoffMm,
  };
  const tried = await Promise.all(
    (["forward", "reverse"] as const).map((direction) => measurer.planTorchPath(step, { ...request, direction })),
  );
  if (!checkpoint()) return null;
  const best = tried.reduce((a, b) => ((b.plan.clear_fraction ?? 0) > (a.plan.clear_fraction ?? 0) ? b : a));
  const plan = best.plan;
  if (!plan) throw new Error("the search finished without a plan");
  onSearched?.(best.direction, plan);
  const payload = optimisedPayload(seam, plan);
  const checked = await measurer.checkTorchFits(step, {
    joint,
    seam: posedSeam(seam, payload),
    torch: inputs.torch,
    options: { standoff_mm: plan.standoff_mm },
  });
  if (!checkpoint()) return null;
  return { plan, payload, clearance: checked.clearance, report: offer(checked.clearance, seam.span) };
}

/** The id, span and narrowed payload of run k of a blocked seam (the weld
 *  planner's SPLIT): the optimised profiles carry over narrowed onto the
 *  piece -- a split decides WHERE to weld, not how. */
export function runPiecePayload(seam: RailsSeam, cached: SeamPlan, k: number): { id: string; span: Span; payload: RailsWeldPayload } {
  const [lo, hi] = cached.report.usableRuns[k]!;
  const span = cached.report.split[k]!;
  const id = `rails-${seam.id}-run${k + 1}`;
  return {
    id,
    span: { ...span },
    payload: {
      ...cached.payload,
      id,
      overrides: restrictScalars(structuredClone(cached.payload.overrides), lo!, hi!),
      span: { ...span },
    },
  };
}

export interface RunPiece {
  id: string;
  span: Span;
  payload: RailsWeldPayload;
  clearance: RailsClearance;
  summary: CoverageSummary;
  plan: RailsOptimisePlan;
  note: string;
}

/** Run k's clearance re-check, and the parent plan re-axed onto the piece:
 *  the narrowed profiles are the truth now, so the knots are just sample
 *  positions and the path arrays are left to the profiles' evaluation. */
export async function measureRunPiece(
  measurer: SeamMeasurer,
  step: Uint8Array,
  seam: RailsSeam,
  joint: RailsJoint,
  cached: SeamPlan,
  k: number,
  torch: unknown,
  checkpoint: Checkpoint = () => true,
): Promise<RunPiece | null> {
  const count = cached.report.split.length;
  const { id, span, payload } = runPiecePayload(seam, cached, k);
  const checked = await measurer.checkTorchFits(step, {
    joint,
    seam: posedSeam(seam, payload),
    torch,
    options: { standoff_mm: cached.plan.standoff_mm },
  });
  if (!checkpoint()) return null;
  const summary = summarise(checked.clearance, span);
  const pct = summary.seamMm > 0 ? Math.round((100 * summary.clearMm) / summary.seamMm) : 0;
  return {
    id,
    span,
    payload,
    clearance: checked.clearance,
    summary,
    plan: {
      ...cached.plan,
      knot_fractions: Array.from({ length: 9 }, (_, i) => i / 8),
      work_path_deg: [],
      travel_path_deg: [],
      clear_fraction: pct / 100,
      seam_length_mm: summary.seamMm,
    },
    note: `run ${k + 1}/${count} · ${summary.seamMm.toFixed(0)} mm · ${pct}% clear`,
  };
}

/** What a decision reads of an entry. */
export interface DecidableEntry {
  id: string;
  seam: RailsSeam;
  joint: RailsJoint;
  span: Span;
  plan: RailsOptimisePlan | null;
  summary: CoverageSummary | null;
  payload: RailsWeldPayload | null;
}

export interface Decision {
  state: "accepted" | "trimmed";
  span: Span;
  note: string;
  payload: RailsWeldPayload | null;
  node: WeldNode;
}

/**
 * Accept the proposal, or trim it to its longest clear run, and the weld
 * node that decision writes. Null when there is nothing to trim to.
 * Reaccepting edits Rails geometry and profiles; it must not re-enable a
 * disabled weld or discard authored standoff, process parameters or other
 * extensions, so `previous` (the node already in the program) is merged.
 */
export function decideWeld(entry: DecidableEntry, choice: "trim" | "accept", sha256: string, previous?: WeldNode): Decision | null {
  let span = entry.span;
  let note: string;
  let state: Decision["state"];
  if (choice === "trim") {
    if (!entry.summary?.trimSpan) return null;
    span = { ...entry.summary.trimSpan };
    state = "trimmed";
    note = `trimmed to ${entry.summary.trimMm.toFixed(0)} mm`;
  } else {
    state = "accepted";
    note = entry.plan ? `accepted · ${Math.round(entry.plan.clear_fraction * 100)}% weldable` : "accepted";
  }
  const payload = entry.payload ? { ...entry.payload, span } : null;
  const generated = seamWeldNodeFor({ ...entry, span, payload, note }, sha256);
  const node: WeldNode = previous ? {
    ...previous,
    geometry: { ...previous.geometry, ...generated.geometry },
    travel_speed_mm_s: generated.travel_speed_mm_s,
    weave: generated.weave,
    comment: generated.comment,
    extensions: { ...previous.extensions, ...generated.extensions },
  } : generated;
  return { state, span, note, payload, node };
}
