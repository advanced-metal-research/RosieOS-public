// The weld planner motion server (weld_planner/v1, FastAPI on :8796): seam
// search, weld and connecting trajopt, the continuous verifier and the dense
// export. Planning moves nothing; it is CPU/GPU work on the planner host, one
// plan at a time (a second request queues behind the lock).
import path from "node:path";

import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { failure, guarded, json, responseFormat, result } from "../format.js";
import { fetchJSON, refusalText } from "rosie-cli/lib/http.js";
import { fetchDense, planWeldplan } from "rosie-cli/lib/planner.js";
import { renderTrajectory } from "./repo.js";

const READ_ONLY = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true } as const;
const DIGEST = /^sha256:[0-9a-f]{64}$/;

export function registerPlannerTools(server: McpServer, ctx: Context): void {
  const { config } = ctx;
  const base = config.plannerUrl;

  server.registerTool(
    "rosie_planner_get_health",
    {
      title: "Check the weld planner motion server",
      description: `Read the weld planner's health and current stage (GET /api/motion/health and /api/motion/progress on ${base}): CUDA availability and device, the source revision the deploy tool installed (null when run by hand), whether a warm spare worker is ready, and the stage of the plan in flight (null when idle).

Read-only. A 'motion-cpu' planner (on the relay) plans taught moves, dwells and Home only; approach/weld/transit need the CUDA environment.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const [health, progress] = await Promise.all([
        fetchJSON("weld planner", `${base}/api/motion/health`, { timeoutMs: config.requestTimeoutMs }),
        fetchJSON("weld planner", `${base}/api/motion/progress`, { timeoutMs: config.requestTimeoutMs }),
      ]);
      const data = { url: base, health: health.body, progress: progress.body };
      return result(response_format, data, () => `# Weld planner at ${base}\n\n\`\`\`json\n${json(data)}\n\`\`\``);
    }),
  );

  server.registerTool(
    "rosie_planner_plan_weldplan",
    {
      title: "Plan a .weldplan with the weld planner",
      description: `Post a .weldplan request file (packed by OLP's seam worker; OLP's 'weld-plan/export' writes one) to the weld planner (POST /api/motion/plan) and wait for the plan. Can take seconds (taught moves on CPU) to minutes (welds on GPU). Moves nothing.

The full result is megabytes of joint samples: it is written to output_path (default: a file under the OS temp dir) and this tool returns a summary: seams crossed, trajectories made, connecting moves, dense export identity or refusal, and the planner's stage timings.

Args:
  - path: absolute path of the .weldplan file.
  - dense (default true): also export the robot.v4.dense-joint-trajectory.v1 blob the cell executes; its identity (trajectory_digest, plan_id, ...) is what rosie_olp_load / rosie_olp_run_program take.
  - program_id, manifest_revision, plan_revision: the identity fields the daemon fences on.
  - verify (default true), verify_margin_mm, order_seams, free_space_backend: planner options (defaults are the planner's).
  - timeout_s: give up waiting after this long (default 600); the planner cancels a request whose client disconnected.`,
      inputSchema: z
        .object({
          path: z.string().min(1),
          dense: z.boolean().default(true),
          program_id: z.string().max(120).default(""),
          manifest_revision: z.number().int().min(1).default(1),
          plan_revision: z.number().int().min(1).default(1),
          verify: z.boolean().optional(),
          verify_margin_mm: z.number().min(0).max(50).optional(),
          order_seams: z.boolean().optional(),
          free_space_backend: z.enum(["bspline", "curobo", "legacy"]).optional(),
          timeout_s: z.number().int().min(10).max(3600).default(600),
          output_path: z.string().optional().describe("Where to write the full JSON result"),
          response_format: responseFormat,
        })
        .strict(),
      annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: true },
    },
    guarded(async (input) => {
      let summary: Awaited<ReturnType<typeof planWeldplan>>;
      try {
        summary = await planWeldplan(config, input.path, {
          dense: input.dense, programId: input.program_id, manifestRevision: input.manifest_revision, planRevision: input.plan_revision,
          verify: input.verify, verifyMarginMm: input.verify_margin_mm, orderSeams: input.order_seams, freeSpaceBackend: input.free_space_backend,
          timeoutS: input.timeout_s, outputPath: input.output_path,
        });
      } catch (error) {
        return failure(error instanceof Error ? error.message : String(error));
      }
      const elapsedS = summary.elapsed_s;
      const target = summary.result_path;
      return result(input.response_format, summary, () =>
        [
          `# Plan of ${path.basename(input.path)} (${elapsedS.toFixed(1)} s)`,
          `- **Seams**: ${summary.seams} (${summary.seams_crossed} crossed, ${summary.weld_trajectories} weld trajectories) · **connecting moves**: ${summary.connecting_moves ?? "n/a"}`,
          summary.connecting_error ? `- **Connecting moves FAILED**: ${String(summary.connecting_error)}` : "",
          summary.dense ? `- **Dense export**:\n\`\`\`json\n${json(summary.dense)}\n\`\`\`` : "",
          summary.dense_error ? `- **Dense export REFUSED**: ${json(summary.dense_error)}` : "",
          summary.coverage ? `- **Coverage**: ${json(summary.coverage)}` : "",
          summary.timings_ms ? `- **Stage timings (ms)**: ${json(summary.timings_ms)}` : "",
          `- **Planner revision**: ${String(summary.source_revision ?? "unbound")} · **Full result**: \`${target}\``,
        ]
          .filter(Boolean)
          .join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_planner_fetch_dense",
    {
      title: "Fetch a planned dense trajectory by digest",
      description: `Download the dense trajectory blob a plan filed on the weld planner (GET /api/motion/dense/{trajectory_digest}), save it as an .rdt file and inspect it offline (format rules, digest, segments, per-axis range and peak speed against the header's ceiling). Moves nothing.`,
      inputSchema: z
        .object({
          trajectory_digest: z.string().regex(DIGEST).describe("sha256:<64 hex>"),
          output_path: z.string().optional().describe("Where to save the .rdt (default under the OS temp dir)"),
          response_format: responseFormat,
        })
        .strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ trajectory_digest, output_path, response_format }) => {
      let summary: Awaited<ReturnType<typeof fetchDense>>;
      try {
        summary = await fetchDense(config, trajectory_digest, output_path);
      } catch (error) {
        return failure(error instanceof Error ? error.message : String(error));
      }
      const target = summary.saved_to;
      return result(response_format, summary, () => `Saved to \`${target}\`.\n\n${renderTrajectory(summary)}`);
    }),
  );
}
