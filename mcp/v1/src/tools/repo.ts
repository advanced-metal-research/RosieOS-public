// Answers from this checkout alone: committed cell manifests, the A6-EC alarm
// tables, dense trajectory files and the component documentation. Nothing
// here contacts a host.
import { readFile, stat } from "node:fs/promises";

import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import * as repo from "rosie-cli/lib/repo.js";
import { DOCS } from "rosie-cli/lib/repo.js";
import { deg, failure, guarded, json, responseFormat, result, table, yesNo } from "../format.js";
import { summarize, type TrajectorySummary } from "rosie-cli/lib/rdt.js";

const LOCAL = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: false } as const;
const CELL_NAME = /^[A-Za-z0-9._-]+$/;

export { DOCS, docsFor } from "rosie-cli/lib/repo.js";

export function cellsDir(ctx: Context): string {
  return repo.cellsDir(ctx.config.repoRoot);
}

export function listCellNames(ctx: Context): Promise<string[]> {
  return repo.listCellNames(ctx.config.repoRoot);
}

export function readCell(ctx: Context, name: string): Promise<Record<string, unknown>> {
  return repo.readCell(ctx.config.repoRoot, name);
}

export const cellSummary = repo.cellSummary;

export function renderTrajectory(summary: TrajectorySummary & { saved_to?: string }): string {
  if (!summary.valid) {
    const r = summary.rejection!;
    return `# Dense trajectory REJECTED\n- **Reason**: \`${r.reason}\`${r.segment_index !== null ? ` (segment ${r.segment_index})` : ""}\n- **Detail**: ${r.detail}\n\nThe daemon's ingest would refuse this blob with the same reason.`;
  }
  const id = summary.identity!;
  return [
    "# Dense trajectory (valid)",
    `- **Identity**: trajectory_digest \`${id.trajectory_digest}\` · plan_id \`${id.plan_id}\` · program_id \`${id.program_id}\` · program_digest \`${id.program_digest}\` · manifest_revision ${id.manifest_revision} · plan_revision ${id.plan_revision}`,
    `- **Created**: ${id.created_at} · **dt**: ${summary.dt_s} s · **samples**: ${summary.total_samples} · **duration**: ${summary.total_duration_s?.toFixed(3)} s · **axis mask**: 0x${summary.axis_mask?.toString(16)}`,
    summary.robot_cell ? `- **Robot**: ${json(summary.robot_cell)}` : "- **Robot**: none (the daemon requires one at preload)",
    `- **Measured start**: ${summary.start_state ? "present" : "none"} · **process markers**: ${summary.process_markers}`,
    "",
    table(["#", "Kind", "Source", "Samples", "Duration (s)", "Torch samples"], summary.segments.map((s) => [s.index, s.kind, s.source_id, s.samples, s.duration_s.toFixed(3), s.torch_samples])),
    "",
    table(
      ["Axis", "In mask", "Min", "Max", "Peak speed", "Ceiling", "Use"],
      summary.axes.filter((a) => a.in_mask).map((a) => [a.axis, yesNo(a.in_mask), deg(a.min_q_rad), deg(a.max_q_rad), `${deg(a.max_abs_qd_rad_s)}/s`, `${deg(a.limit_qd_rad_s)}/s`, `${(a.qd_utilisation * 100).toFixed(1)} %`]),
    ),
  ].join("\n");
}


export function registerRepoTools(server: McpServer, ctx: Context): void {
  const root = ctx.config.repoRoot;

  server.registerTool(
    "rosie_list_cells",
    {
      title: "List the committed robot cells",
      description: `List every cell manifest committed under rt-core/config/cells (this checkout): its name, servo host, the machine config and configuration pin it binds, the rt-control remote listener, the OLP unit and listen address, and whether unattended loops are allowed.

Local, read-only. The pin here is what the repository approves; the controller's running pin comes from rosie_rtcore_describe. Never apply an uncommitted configuration to a cell.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: LOCAL,
    },
    guarded(async ({ response_format }) => {
      const names = await listCellNames(ctx);
      const cells = await Promise.all(names.map(async (name) => cellSummary(name, await readCell(ctx, name))));
      return result(response_format, { count: cells.length, cells }, () =>
        [
          `# ${cells.length} committed cells (rt-core/config/cells)`,
          "",
          table(
            ["Cell", "Machine config", "Pin", "rt-control", "OLP", "Deployable"],
            cells.map((c) => [c.file, c.machine_config ?? "", c.configuration_sha256 ? `${String(c.configuration_sha256).slice(0, 12)}…` : "", c.remote_listen ?? "", c.olp_listen ?? "", yesNo(c.has_cell_deploy)]),
          ),
        ].join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_get_cell_manifest",
    {
      title: "Get a cell manifest",
      description: `Read one committed cell manifest (rt-core/config/cells/<cell>.json). Without 'section' it returns the summary (as rosie_list_cells) plus the top-level keys; with section = a top-level key (e.g. 'cell_deploy', 'nodes', 'robot_axes', 'connections', 'pendant_deploy') it returns that part in full.

Local, read-only. Deploy commands run from steamdeck/real/v4 with --manifest rt-core/config/cells/<cell>.json; a cell's name is its manifest's file name.`,
      inputSchema: z
        .object({
          cell: z.string().regex(CELL_NAME).describe("Manifest file name without .json, for example example-arm-1420"),
          section: z.string().max(100).optional(),
          response_format: responseFormat,
        })
        .strict(),
      annotations: LOCAL,
    },
    guarded(async ({ cell, section, response_format }) => {
      let manifest: Record<string, unknown>;
      try {
        manifest = await readCell(ctx, cell);
      } catch {
        return failure(`no cell manifest ${cell}; known cells: ${(await listCellNames(ctx)).join(", ")}`);
      }
      if (section !== undefined) {
        if (!(section in manifest)) return failure(`${cell} has no section '${section}'; sections: ${Object.keys(manifest).join(", ")}`);
        const data = { cell, section, value: manifest[section] };
        return result(response_format, data, () => `# ${cell} · ${section}\n\n\`\`\`json\n${json(manifest[section])}\n\`\`\``);
      }
      const data = { ...cellSummary(cell, manifest), sections: Object.keys(manifest) };
      return result(response_format, data, () => `# Cell ${cell}\n\n\`\`\`json\n${json(data)}\n\`\`\``);
    }),
  );

  server.registerTool(
    "rosie_lookup_drive_alarm",
    {
      title: "Look up an A6-EC drive alarm or fault",
      description: `Explain an A6-EC servo drive code from the vendor manual table (rt-core/docs/drives/a6ec/a6ec_manual_codes.json) and say whether rt-core's drive profile (rt-core/config/drives/a6ec.json fault_reset.resettable_alarms) lets an operator Reset/Arm clear it.

Args:
  - query: a fault code (Er41.1), alarm code (ALF1.3), 0x603F bus fault code (0x3230), or words from the name ("over-temperature").
Local, read-only. A code the profile does not list is not resettable by software: it needs the drive or the hardware fixed, and Home is never re-run to clear a fault.`,
      inputSchema: z.object({ query: z.string().min(2).max(100), response_format: responseFormat }).strict(),
      annotations: LOCAL,
    },
    guarded(async ({ query, response_format }) => {
      const matches = await repo.lookupAlarm(root, query);
      const page = matches.slice(0, 25);
      if (page.length === 0) return failure(`no A6-EC code or name matches '${query}'. Codes look like Er41.1, ALF1.3 or 0x3230.`);
      return result(response_format, { query, total: matches.length, matches: page }, () =>
        [
          `# A6-EC codes matching "${query}" (${matches.length}${matches.length > page.length ? `, first ${page.length}` : ""})`,
          "",
          table(
            ["Code", "Name", "Manual: resettable", "rt-core Reset clears it", "0x603F", "0x203F"],
            page.map((m) => [m.code ?? m.key, m.name, yesNo(m.resettable), m.rt_core_profile_resettable === null ? "n/a" : yesNo(m.rt_core_profile_resettable), m.bus_fault_code_603f ?? "", m.fault_code_203f ?? m.alarm_code_203f ?? ""]),
          ),
          "",
          "Source: vendor manual table (best-effort PDF extraction; the PDF is authoritative). 'rt-core Reset clears it' is the drive profile's fault_reset list: Arm and recover-to-zero send the drive's own CiA402 reset only for those.",
        ].join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_inspect_dense_trajectory",
    {
      title: "Inspect a dense trajectory (.rdt) file",
      description: `Decode and validate a robot.v4.dense-joint-trajectory.v1 file offline with every rule the dense daemon's ingest enforces (block hashes, time grid, speed ceilings, boundary rest, torch only in welds, content digest), then summarise it: the six-field identity that Load and Run program take, robot and measured start binding, segments, and per joint the angle range and peak speed against the header's ceiling.

Local, read-only. A rejection names the same reason the daemon would answer.`,
      inputSchema: z.object({ path: z.string().min(1).describe("Absolute path to the .rdt file"), response_format: responseFormat }).strict(),
      annotations: LOCAL,
    },
    guarded(async ({ path: file, response_format }) => {
      const info = await stat(file);
      if (info.size > 64 * 1024 * 1024) return failure(`${file} is ${info.size} bytes, beyond the 64 MiB frame limit`);
      const summary = summarize(await readFile(file));
      return { ...result(response_format, summary, () => renderTrajectory(summary)), isError: !summary.valid };
    }),
  );

  server.registerTool(
    "rosie_search_docs",
    {
      title: "Search the RosieOS documentation",
      description: `Search the component READMEs and contracts (${DOCS.length} documents: repo rules, deploy workflow, rt-core API and contracts, drive reference, motion servers, dense format, OLP, weld planner, pendant, deploy tool) for a phrase, case-insensitive. Each hit carries the document id, file, line, the nearest heading and the line. Read a whole document as the MCP resource rosieos://docs/<id>.

Args:
  - query: text to find (plain text, not a regex).
  - docs: optional list of document ids to restrict the search.
  - limit / offset: pagination (default 20 from 0).
Local, read-only. These documents carry measured numbers next to their constants and the owner's rules; prefer them over guesses.`,
      inputSchema: z
        .object({
          query: z.string().min(2).max(200),
          docs: z.array(z.string().regex(/^[a-z0-9-]+$/)).optional(),
          limit: z.number().int().min(1).max(100).default(20),
          offset: z.number().int().min(0).default(0),
          response_format: responseFormat,
        })
        .strict(),
      annotations: LOCAL,
    },
    guarded(async ({ query, docs, limit, offset, response_format }) => {
      const hits = await repo.searchDocs(root, query, docs);
      const page = hits.slice(offset, offset + limit);
      const data = { query, total: hits.length, count: page.length, offset, has_more: offset + page.length < hits.length, next_offset: offset + page.length < hits.length ? offset + page.length : null, hits: page };
      return result(response_format, data, () =>
        page.length === 0
          ? `No documentation line contains "${query}".`
          : [`# "${query}": ${hits.length} hits${data.has_more ? ` (showing ${offset + 1}-${offset + page.length}; next offset ${data.next_offset})` : ""}`, "", ...page.map((h) => `- **${h.file}:${h.line}** (${h.heading || "top"}): ${h.text}`)].join("\n"),
      );
    }),
  );
}
