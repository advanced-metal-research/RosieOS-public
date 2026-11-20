import type { WorkcellFixtureWire } from "../domain/workcell.ts";

/**
 * Rails mode's calls into the seam worker.
 *
 * One route, `POST /api/offline-programming/v1/seam`, with the operation named
 * in the body: the OLP forwards the worker's own request shape rather than
 * restating it as an API, the way it already drives the classic CAD worker.
 * There is no second application behind this any more -- D24 deleted the
 * authoring server and the bridge to it, and `internal/seam` runs
 * `seam_worker` as one subprocess per request out of `weld_planner/v1`.
 *
 * The STEP travels in the request. Nothing is looked up by hash, because there
 * is no workspace on the other side to look anything up in.
 */

const SEAM = "/api/offline-programming/v1/seam";

/** Base64 without a data: prefix, for the `step_base64` every measuring
 *  request carries. Chunked because `String.fromCharCode(...bytes)` on a
 *  multi-megabyte STEP overflows the argument list. */
function base64(bytes: Uint8Array): string {
  let binary = "";
  const stride = 0x8000;
  for (let index = 0; index < bytes.length; index += stride) {
    binary += String.fromCharCode(...bytes.subarray(index, index + stride));
  }
  return btoa(binary);
}

async function call<T>(body: Record<string, unknown>): Promise<T> {
  const response = await fetch(SEAM, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!response.ok) {
    let detail = `${response.status}`;
    try {
      const failure = (await response.json()) as { error?: string; code?: string };
      detail = failure.error ?? failure.code ?? detail;
    } catch {
      /* the status alone, then */
    }
    throw new Error(detail);
  }
  return (await response.json()) as T;
}

export interface RailsUpload {
  sha256: string;
  filename: string;
  bytes: number;
}

/** A seam as the joints doc lists it — the unit an edge pick resolves to. */
export interface RailsSeam {
  id: string;
  label: string;
  weld_joint_ref: string;
  span: { start_s: number; end_s: number };
  direction: string;
  reference: { mode: string; rail?: unknown };
  process: Record<string, unknown>;
  [key: string]: unknown;
}

export interface RailsJoint {
  id: string;
  weld_line: { segments: { source_edge_id?: string }[]; closed: boolean; length_m: number };
  bisector_rail?: { curve: unknown };
  [key: string]: unknown;
}

export interface RailsJointsDoc {
  weld_joints: RailsJoint[];
  seams: RailsSeam[];
}

export interface RailsFrame {
  s: number;
  point_xyz_m: number[];
  tangent_xyz: number[];
  reference_xyz: number[];
  binormal_xyz: number[];
}

export interface RailsFrames {
  length_m: number;
  closed: boolean;
  frames: RailsFrame[];
  /** Sampling notes from the worker -- a rail that ran out before the seam,
   *  a curvature limit that capped the grid. Worth keeping: a guide drawn
   *  from a truncated frame set looks like a short seam, not a warning. */
  warnings?: string[];
}

/** What the angle search returns — the proposal the popup charts. */
export interface RailsOptimisePlan {
  work_angle_deg: Record<string, unknown>;
  travel_angle_deg: Record<string, unknown>;
  direction?: "forward" | "reverse";
  direction_changed?: boolean;
  standoff_mm: number;
  standoff_departure_mm: number;
  clear_fraction: number;
  authored_clear_fraction: number;
  seam_length_mm: number;
  knot_fractions: number[];
  work_path_deg: number[];
  travel_path_deg: number[];
}

export interface RailsClearance {
  seam_length_m: number;
  clear_runs?: number[][];
  blocked_spans?: number[][];
  sealed_spans?: number[][];
  min_clear_run_m?: number;
  [key: string]: unknown;
}

/** The wire form of one rails weld — the weld planner's weld-list entry. */
export interface RailsWeldPayload {
  id: string;
  seam_ids: string[];
  preset_id: string | null;
  label?: string;
  span?: { start_s: number; end_s: number } | null;
  overrides: Record<string, unknown>;
}

/** The active settings document — rails mode honours its default weld preset,
 *  the same rule the weld planner's own UI applies at weld creation. */
/** Where the parts meet, and the seams each joint defaults to. One call: the
 *  server ran topology, detection and the default seams as separate stages of
 *  a job because it streamed progress between them, and a caller that waits
 *  for one answer has no use for the seam. */
export const detectJoints = (step: Uint8Array): Promise<{ topology: unknown; joints: RailsJointsDoc }> =>
  call({ operation: "detect_joints", step_base64: base64(step) });

/** The `{P, t, r, b}` at each point along a seam. No STEP: a curve and the
 *  rail its work angle is measured from are the whole input.
 *
 *  Unwrapped once on the way out. Every operation answers under a named key
 *  -- `detect_joints` under `joints`, this one under `frames` -- and what
 *  `seam_frames()` returns is itself an object with a `frames` array in it.
 *  So the wire carries `{frames: {length_m, closed, frames: [...]}}`, and a
 *  caller that read the envelope as the document got the inner OBJECT where
 *  it wanted the array. That is not a type error anywhere: it fails at the
 *  `for...of`, one call later, as "frames.frames is not iterable". */
export const sampleSeamFrames = async (
  curve: unknown, reference: unknown, samples = 160,
): Promise<RailsFrames> =>
  (await call<{ frames: RailsFrames }>(
    { operation: "sample_seam_frames", curve, reference, samples })).frames;

/** How hard the angle search looks. Values, never a preset id: a preset is a
 *  starting point and a comparison, and one edited later must not reach back
 *  and change a weld that was already accepted. */
export interface RailsSearchKnobs {
  work_deviation_deg: number;
  travel_deviation_deg: number;
  standoff_cost_deg_per_mm: number;
  work_bin_deg?: number;
  travel_bin_deg?: number;
  knot_spacing_mm?: number;
  sample_step_mm?: number;
}

/** Where the torch goes along one seam and how it is held there. ONE
 *  direction: the same metal answers differently run the other way, and
 *  nothing but measuring both says which is better, so a caller that wants
 *  both asks twice and keeps the better coverage. */
export const planTorchPath = (
  step: Uint8Array,
  body: {
    joint: RailsJoint;
    seam: RailsSeam;
    torch: unknown;
    direction?: "forward" | "reverse";
    search: RailsSearchKnobs;
    default_standoff_mm: number;
    intersector?: unknown;
  },
): Promise<{ direction: string; plan: RailsOptimisePlan; rays_cast: number; warnings: string[] }> =>
  call({ operation: "plan_torch_path", step_base64: base64(step), ...body });

/** Does THIS torch, at THESE angles, actually fit -- and where does it not.
 *  The coverage strip comes from here. */
export const checkTorchFits = (
  step: Uint8Array,
  body: { joint: RailsJoint; seam: RailsSeam; torch: unknown; options?: unknown },
): Promise<{ clearance: RailsClearance }> =>
  call({ operation: "check_torch_fits", step_base64: base64(step), ...body });

export interface RailsDenseSummary {
  schema: string;
  trajectory_digest: string;
  request_sha256: string;
  plan_id: string;
  program_id: string;
  program_digest: string;
  manifest_revision: number;
  plan_revision: number;
  created_at: string;
  dt_s: number;
  total_sample_count: number;
  total_duration_s: number;
  bytes: number;
  segments: {
    index: number;
    kind: "freespace" | "weld" | "dwell";
    source_id: string;
    sample_count: number;
    duration_s: number;
  }[];
  /** The trajectory header's robot identity, as returned by the motion server. */
  robot_cell?: RailsDenseRobot | null;
  /** The output schedule's digest and size, present only when the plan
   *  switches outputs (weldplan/io_schedule.py markers_digest). */
  process_markers_sha256?: string;
  process_marker_count?: number;
}

export interface RailsDenseRobot {
  model_id: string;
  robot_description_sha256: string;
  machine_planning_calibration_sha256: string;
  machine_planning_calibration_source: "target" | "empty";
}

/** What the rails compile returns: a whole robot.v4 program, rows included —
 *  the caller merges it into the open project; the catalog is not involved. */
export interface RailsPlan {
  ok: boolean;
  program: Record<string, unknown>;
  rows: number;
  motion_rows: number;
  cell_id: string;
  /** Per-seam verdicts — an uncrossed seam compiled to a comment row only. */
  /** The joint-space daemon's trajectory, when the compile asked for it
   *  (`dense` with `refine`) and the motion server could build one. The
   *  motion server emits and files the bytes itself; this is its summary. */
  dense?: RailsDenseSummary | null;
  /** Why there is no `dense`: the daemon's own rejection vocabulary. */
  dense_error?: { reason: string; detail: string; segment_index: number | null } | null;
  seams: {
    id: string;
    crossed: boolean;
    candidates: number;
    /** Why the first unreachable layer was unreachable, as a count per cause
     *  from the search's own `_frontier_causes`. Null on a crossed seam. */
    frontier_causes?: Record<string, number> | null;
    length_mm?: number | null;
    feasible_rate?: number | null;
  }[];
}

/**
 * Compile decided rails welds into robot.v4 rows.
 *
 * One request: packing, the motion search and the rail-to-row compile run
 * server-side and the browser waits. Without `refine` that is seconds per
 * weld; with it the trajopt stages run too, and two welds with three connecting moves took 23 s (measured 2026-09-12).
 */
export type FreeSpaceBackend = "bspline" | "curobo" | "legacy";

/** What the plan route answers with. The planner's own document, the digest
 *  of the request it answers, and the trajectory the robot will play --
 *  fetched at plan time rather than left on the motion server, whose dense
 *  store is a cache under `~/.cache` and owes nobody a copy later. */
export interface RailsPlanResult {
  result: unknown;
  weldplan_sha256: string;
  /** Echoed back: the program this plan answers for. */
  program_digest?: string;
  dense_blob_base64?: string;
  dense?: RailsDenseSummary;
  dense_error?: { reason: string; detail: string; segment_index: number | null };
}

/** Pack, plan, and bring the trajectory back, in one request.
 *
 *  The `.weldplan` is built by the seam worker and posted to the motion server
 *  by the OLP: the browser never learns either address, and the program.v2
 *  document it sends is the one it already holds rather than a weld list the
 *  other side would rebuild a program from. */
/** The same `.weldplan` a plan is made from, as bytes.
 *
 *  Packed by the same code path as `planWeldTrajectory`, deliberately: an
 *  export that differed from what was planned would reproduce a different
 *  question. The container is the one artifact that carries a plan somewhere
 *  else -- program.v2, the placement inside it, the cell descriptor, the
 *  tooling and the STEP, hashed together -- which the project JSON cannot do,
 *  because it is what the OLP holds rather than what a planner is handed. */
export const exportWeldPlan = async (body: PlanRequestBody): Promise<{
  bytes: Uint8Array;
  filename: string;
  sha256: string;
}> => {
  const response = await fetch("/api/offline-programming/v1/weld-plan/export", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!response.ok) {
    let detail = `${response.status}`;
    try {
      detail = ((await response.json()) as { error?: string }).error ?? detail;
    } catch {
      /* the status alone, then */
    }
    throw new Error(detail);
  }
  const disposition = response.headers.get("Content-Disposition") ?? "";
  const named = /filename="([^"]+)"/.exec(disposition);
  return {
    bytes: new Uint8Array(await response.arrayBuffer()),
    filename: named?.[1] ?? `${body.program_id || "program"}.weldplan`,
    sha256: response.headers.get("X-Weldplan-SHA256") ?? "",
  };
};

export interface PlanRequestBody {
  require_measured_start?: boolean;
  /** Plan from the program's start (model reset) with no measured capture;
   *  Go to plan start before Load and Play (weld_plan.go). */
  plan_from_program_start?: boolean;
  program: unknown;
  tooling: unknown;
  step_base64: string;
  step_filename: string;
  workpiece_absent?: boolean;
  cell_id: string;
  /** The machine_planning_calibration.json a plan is made against when NO cell is connected --
   *  pulled earlier or loaded from a file. A connected cell's own file wins;
   *  the server reads this only without one (docs/PLANNING-CONTEXT.md). */
  machine_planning_calibration_base64?: string;
  program_id: string;
  /** The operator's workcell fixtures (the stand), in the cell's world frame:
   *  what the planner clears instead of its own placeholder. Always sent, an
   *  empty list included, so the planner never adds a stand the OLP does not
   *  draw. */
  fixtures: WorkcellFixtureWire[];
  /** The digest of the document being planned, so what comes back can name
   *  the program it answers rather than the one that happens to be open. */
  program_digest?: string;
  free_space_backend?: FreeSpaceBackend;
  manifest_revision?: number;
  plan_revision?: number;
}

export const planWeldTrajectory = (body: PlanRequestBody, signal?: AbortSignal): Promise<RailsPlanResult> =>
  fetch("/api/offline-programming/v1/weld-plan", {
    signal,
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then(async (response) => {
    if (!response.ok) {
      let detail = `${response.status}`;
      let denseError: RailsPlanResult["dense_error"];
      let code: string | undefined;
      try {
        const failure = (await response.json()) as {
          code?: string;
          error?: string;
          dense_error?: RailsPlanResult["dense_error"];
        };
        detail = failure.error ?? detail;
        denseError = failure.dense_error;
        code = failure.code;
      } catch {
        /* the status alone, then */
      }
      throw Object.assign(new Error(detail), { dense_error: denseError, code });
    }
    return (await response.json()) as RailsPlanResult;
  });

export { base64 as encodeSTEP };
