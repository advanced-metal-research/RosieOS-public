// The weld planner motion server (weld_planner/v1, :8796), shared by the CLI
// and the MCP server: plan a .weldplan (the full result is megabytes of joint
// samples, so it is written to a file and summarised) and fetch a filed dense
// trajectory by digest. Nothing here moves a robot.
import { mkdir, readFile, stat, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";

import type { Config } from "./config.js";
import { fetchJSON, refusalText } from "./http.js";
import { summarize, type TrajectorySummary } from "./rdt.js";

export function plannerOutputDir(): string {
  return path.join(tmpdir(), "rosieos-mcp");
}

export interface WeldplanOptions {
  dense?: boolean;
  programId?: string;
  manifestRevision?: number;
  planRevision?: number;
  verify?: boolean;
  verifyMarginMm?: number;
  orderSeams?: boolean;
  freeSpaceBackend?: "bspline" | "curobo" | "legacy";
  timeoutS?: number;
  outputPath?: string;
}

export interface WeldplanSummary {
  result_path: string; elapsed_s: number; seams: number; seams_crossed: number; weld_trajectories: number;
  connecting_moves: number | null; connecting_error: unknown; dense: unknown; dense_error: unknown; coverage: unknown;
  source_revision: unknown; request_sha256: unknown; program_order: unknown; timings_ms: unknown;
}

/** POST /api/motion/plan with a .weldplan; the full result goes to a file, a summary comes back. Throws with the planner's words. */
export async function planWeldplan(config: Config, file: string, options: WeldplanOptions = {}): Promise<WeldplanSummary> {
  const info = await stat(file);
  if (!info.isFile()) throw new Error(`${file} is not a file`);
  const dense = options.dense ?? true;
  const query = new URLSearchParams({
    dense: String(dense), program_id: options.programId ?? "", manifest_revision: String(options.manifestRevision ?? 1), plan_revision: String(options.planRevision ?? 1),
  });
  if (dense) {
    query.set("run_weld_trajopt", "true");
    query.set("run_connecting_trajopt", "true");
  }
  if (options.verify !== undefined) query.set("verify", String(options.verify));
  if (options.verifyMarginMm !== undefined) query.set("verify_margin_mm", String(options.verifyMarginMm));
  if (options.orderSeams !== undefined) query.set("order_seams", String(options.orderSeams));
  if (options.freeSpaceBackend !== undefined) query.set("free_space_backend", options.freeSpaceBackend);
  const started = Date.now();
  const reply = await fetchJSON("weld planner", `${config.plannerUrl}/api/motion/plan?${query}`, { method: "POST", rawBody: await readFile(file), timeoutMs: (options.timeoutS ?? 600) * 1000 });
  const elapsedS = (Date.now() - started) / 1000;
  if (!reply.ok) throw new Error(`the planner refused the plan after ${elapsedS.toFixed(1)} s (HTTP ${reply.status}): ${refusalText(reply)}`);
  const out = (reply.body ?? {}) as Record<string, unknown>;
  const target = options.outputPath ?? path.join(plannerOutputDir(), `plan-${path.basename(file, path.extname(file))}-${started}.json`);
  await mkdir(path.dirname(target), { recursive: true });
  await writeFile(target, JSON.stringify(out));
  // Keys from weld_motion_planner/io/native_result.py result_document.
  const seams = Array.isArray(out.seams) ? (out.seams as Array<Record<string, unknown>>) : [];
  const producer = (out.producer ?? {}) as Record<string, unknown>;
  const request = (out.request ?? {}) as Record<string, unknown>;
  return {
    result_path: target, elapsed_s: elapsedS, seams: seams.length, seams_crossed: seams.filter((s) => s.crossed).length,
    weld_trajectories: seams.filter((s) => s.trajectory).length,
    connecting_moves: Array.isArray(out.connecting_trajectories) ? out.connecting_trajectories.length : null,
    connecting_error: out.connecting_trajectories_error ?? null, dense: out.dense ?? null, dense_error: out.dense_error ?? null,
    coverage: out.coverage ?? null, source_revision: producer.source_revision ?? null, request_sha256: request.request_sha256 ?? null,
    program_order: out.program_order ?? null, timings_ms: out.timings_ms ?? null,
  };
}

/** GET /api/motion/dense/{digest}: save the blob as .rdt and inspect it offline with every ingest rule. */
export async function fetchDense(config: Config, digest: string, outputPath?: string): Promise<TrajectorySummary & { saved_to: string }> {
  let response: Response;
  try {
    response = await fetch(`${config.plannerUrl}/api/motion/dense/${digest}`, { signal: AbortSignal.timeout(Math.max(config.requestTimeoutMs, 60_000)) });
  } catch (error) {
    throw new Error(`weld planner at ${config.plannerUrl} is unreachable: ${error instanceof Error ? error.message : String(error)}`);
  }
  if (!response.ok) throw new Error(`the planner has no dense trajectory ${digest} (HTTP ${response.status}): ${(await response.text()).slice(0, 300)}`);
  const blob = new Uint8Array(await response.arrayBuffer());
  const target = outputPath ?? path.join(plannerOutputDir(), `${digest.slice(7, 23)}.rdt`);
  await mkdir(path.dirname(target), { recursive: true });
  await writeFile(target, blob);
  return { saved_to: target, ...summarize(blob) };
}
