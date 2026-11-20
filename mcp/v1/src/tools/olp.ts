// Offline programming (OLP): reads. None of these acquire a grant, change the
// selected cell or edit a program; the cell program store's save and delete
// routes are deliberately absent (they write operators' programs and are
// refused 423 while the robot holds one).
import type { McpServer } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import type { Context } from "rosie-cli/lib/context.js";
import { failure, guarded, json, responseFormat, result, table, yesNo } from "../format.js";
import { refusalText, type HttpReply } from "rosie-cli/lib/http.js";
import { DENSE, OLP_API, OlpRefusal } from "rosie-cli/lib/olp.js";
import { describeStatus } from "rosie-cli/lib/status.js";

const READ_ONLY = { readOnlyHint: true, destructiveHint: false, idempotentHint: true, openWorldHint: true } as const;

export { describeStatus } from "rosie-cli/lib/status.js";

function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? (value as Record<string, unknown>) : {};
}

export function registerOlpTools(server: McpServer, ctx: Context): void {
  const { olp } = ctx;

  server.registerTool(
    "rosie_olp_get_health",
    {
      title: "Check the OLP server and its planners",
      description: `Read the OLP server's health and capabilities (GET /health and /capabilities on ${ctx.config.olpUrl}).

"health ok" does NOT mean planning works: rails authoring needs the seam worker (weld_planner/v1 'pixi install -e default') and planning needs OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN pointing at the weld planner (:8796). This tool reports both separately, plus the maintenance flag and build id.

Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const [health, capabilities] = await Promise.all([olp.get(`${OLP_API}/health`), olp.get(`${OLP_API}/capabilities`)]);
      const data = { url: olp.baseUrl, health: health.body, health_status: health.status, capabilities: capabilities.body };
      return result(response_format, data, () => [`# OLP at ${olp.baseUrl}`, "", `## Health (HTTP ${health.status})`, "```json", json(health.body), "```", "", `## Capabilities (HTTP ${capabilities.status})`, "```json", json(capabilities.body), "```"].join("\n"));
    }),
  );

  server.registerTool(
    "rosie_olp_get_machine_status",
    {
      title: "Get the selected cell's machine status from OLP",
      description: `Read OLP's machine view of the selected cell (GET dense-execution/status): which cell is selected and whether it is a SIMULATION or a REAL ROBOT, armed, the execution session (id, kind, state, lease, heartbeat age vs its 5 s deadline, stop reason, last error), the playback cursor, a running loop, the last jog end reason and per-axis readiness/Home/drive alarms.

This is what every browser and Steam Deck polls; it is the first thing to read before any robot action. Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const status = await olp.status();
      return result(response_format, status, () => `# OLP machine status\n${describeStatus(status)}\n\nHeartbeat keeper: ${json(ctx.keeper.state())}`);
    }),
  );

  server.registerTool(
    "rosie_olp_list_cells",
    {
      title: "List the cells OLP can select",
      description: `List OLP's commissioned cells and targets (GET dense-execution/cells and GET /targets): which is active, each cell's model, host and backend, and whether it is reachable. Use rosie_olp_select_cell (robot actions enabled) to connect one.

Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const [cells, targets] = await Promise.all([olp.getOk(`${DENSE}/cells`, "cells"), olp.getOk(`${OLP_API}/targets`, "targets")]);
      const data = { cells, targets };
      return result(response_format, data, () => {
        const c = record(cells);
        const list = (Array.isArray(c.cells) ? c.cells : []).map(record);
        const targetList = (Array.isArray(record(targets).targets) ? (record(targets).targets as unknown[]) : []).map(record);
        return [
          `# OLP cells (active: ${String(c.active_cell ?? c.active ?? "none")})`,
          "",
          list.length ? table(["Cell", "Label", "Model", "Host", "Backend"], list.map((x) => [x.cell_id ?? x.id, x.label, x.model_id, x.host, x.backend])) : "No commissioned cells.",
          "",
          "## Targets",
          targetList.length
            ? table(["Cell", "Label", "Backend", "Simulation", "Reason"], targetList.map((x) => [x.cell_id ?? x.id, x.label, x.backend, yesNo(x.simulation), x.reason ?? ""]))
            : "```json\n" + json(targets) + "\n```",
        ].join("\n");
      });
    }),
  );

  server.registerTool(
    "rosie_olp_get_control",
    {
      title: "Who controls the robot, and is an update running",
      description: `Read OLP's maintenance notice and control view (GET /maintenance and /control): whether a deploy/build/test holds the cell (robot actions are then refused 503 'maintenance'), its reason, phase and ETA; which client owns the robot (state free / yours / other / other_server), whether it is armed, and every client that called this server in the last 60 s.

"yours" means this MCP server's own client id. Read-only. Check it before any robot action and when an action is refused 409 'in_use_by_other_controller'.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      // Each route answers on its own: an OLP built before one of them
      // (404) still reports the other.
      const [maintenanceReply, controlReply] = await Promise.all([olp.get(`${OLP_API}/maintenance`), olp.get(`${OLP_API}/control`)]);
      const missing = (name: string, reply: HttpReply) => (reply.ok ? null : `${name}: HTTP ${reply.status} ${refusalText(reply)}${reply.status === 404 ? " (this OLP build predates the route)" : ""}`);
      const problems = [missing("maintenance", maintenanceReply), missing("control", controlReply)].filter((p): p is string => p !== null);
      if (problems.length === 2) return failure(`OLP answered neither route: ${problems.join("; ")}`);
      const maintenance = maintenanceReply.ok ? maintenanceReply.body : null;
      const control = controlReply.ok ? controlReply.body : null;
      const data = { client_id: olp.clientId, maintenance, control, unavailable: problems };
      return result(response_format, data, () => {
        const m = record(maintenance);
        const c = record(control);
        const owner = record(c.owner);
        const clients = (Array.isArray(m.clients) ? m.clients : []).map(record);
        return [
          "# OLP maintenance and control",
          ...problems.map((p) => `- **Unavailable**: ${p}`),
          maintenance ? `- **Maintenance notice**: ${m.active ? `ACTIVE — ${String(m.reason ?? "")} (${String(m.phase ?? "")}, ETA ${String(m.eta ?? "?")})${m.stale ? " [stale]" : ""}` : "none"}` : "",
          `- **Control**: ${String(c.state ?? "?")}${owner.label ? ` — owner ${String(owner.label)} (${String(owner.address ?? "")}) since ${String(owner.since ?? "?")}, last action ${String(owner.last_action ?? "?")}` : ""}`,
          `- **Armed**: ${yesNo(c.armed)} · **lease held**: ${yesNo(c.lease_held)} · **Stop available**: ${yesNo(c.stop_available)}`,
          c.detail ? `- **Detail**: ${String(c.detail)}` : "",
          `- **This MCP server's client id**: \`${olp.clientId}\``,
          "",
          clients.length ? table(["Client", "Kind", "Address", "Controller", "Last seen"], clients.map((x) => [x.label, x.kind, x.address, yesNo(x.controller), x.last_seen ?? x.seen_at ?? ""])) : "No clients in the last 60 s.",
        ]
          .filter((line) => line !== "")
          .join("\n");
      });
    }),
  );

  server.registerTool(
    "rosie_olp_list_cell_programs",
    {
      title: "List the programs stored on the cell",
      description: `List the programs in the cell's program store (GET cell-programs): the programs the browser and both Steam Decks open and save, revision-checked, with who saved them and when, and which one is current.

Args:
  - query (optional): case-insensitive substring filter on program id or name.
  - limit / offset: pagination (default 25 from 0).
Read-only. A server started without --program-store answers that it has no store.`,
      inputSchema: z
        .object({
          query: z.string().max(200).optional().describe("Substring to match against program id or name"),
          limit: z.number().int().min(1).max(100).default(25),
          offset: z.number().int().min(0).default(0),
          response_format: responseFormat,
        })
        .strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ query, limit, offset, response_format }) => {
      const reply = await olp.get(`${OLP_API}/cell-programs`);
      if (!reply.ok) return failure(`OLP cell program store: ${refusalText(reply)}`, { status: reply.status, body: reply.body });
      const body = record(reply.body);
      const all = (Array.isArray(body.programs) ? body.programs : Array.isArray(reply.body) ? (reply.body as unknown[]) : []).map(record);
      const needle = query?.toLowerCase();
      const matched = needle ? all.filter((p) => `${String(p.id ?? "")} ${String(p.name ?? "")}`.toLowerCase().includes(needle)) : all;
      const page = matched.slice(offset, offset + limit);
      const data = {
        store: Object.fromEntries(Object.entries(body).filter(([k]) => k !== "programs")),
        total: matched.length,
        count: page.length,
        offset,
        has_more: offset + page.length < matched.length,
        next_offset: offset + page.length < matched.length ? offset + page.length : null,
        programs: page,
      };
      return result(response_format, data, () =>
        [
          `# Cell programs (${matched.length}${needle ? ` matching "${query}"` : ""}, showing ${offset + 1}-${offset + page.length})`,
          "",
          page.length
            ? table(["Id", "Name", "Revision", "Saved", "By", "Current"], page.map((p) => [p.id, p.name, p.revision, p.saved_at ?? p.updated_at, p.saved_by ?? p.by, yesNo(p.current)]))
            : "No programs.",
          data.has_more ? `\nMore: offset=${data.next_offset}` : "",
        ].join("\n"),
      );
    }),
  );

  server.registerTool(
    "rosie_olp_get_cell_program",
    {
      title: "Get one program from the cell's store",
      description: `Read one stored program (GET cell-programs/programs/{id}), or its revision history with include='revisions'. The document is robot.v4.program.v2 nodes (welds, taught MoveJ/MoveL/MoveC waypoints, dwells, IO, home). Large documents are truncated in markdown; use response_format='json' for the full body.

Read-only.`,
      inputSchema: z
        .object({
          program_id: z.string().min(1).max(200).describe("Program id from rosie_olp_list_cell_programs"),
          include: z.enum(["document", "revisions"]).default("document"),
          response_format: responseFormat,
        })
        .strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ program_id, include, response_format }) => {
      const path = `${OLP_API}/cell-programs/programs/${encodeURIComponent(program_id)}${include === "revisions" ? "/revisions" : ""}`;
      const reply = await olp.get(path);
      if (!reply.ok) return failure(`OLP refused ${include} of ${program_id}: ${refusalText(reply)}`);
      return result(response_format, reply.body, () => `# Cell program ${program_id} (${include})\n\n\`\`\`json\n${json(reply.body)}\n\`\`\``);
    }),
  );

  server.registerTool(
    "rosie_olp_get_local_simulator_status",
    {
      title: "Get OLP's local Tesseract simulator status",
      description: `Read the local canonical simulator's status (GET local-simulator/status): phase, availability and readiness of the contained Motion Server + Tesseract planner process. It is loopback-only, hardware-free and proves simulation mode with EtherCAT and motion output off.

Read-only.`,
      inputSchema: z.object({ response_format: responseFormat }).strict(),
      annotations: READ_ONLY,
    },
    guarded(async ({ response_format }) => {
      const reply = await olp.get(`${OLP_API}/local-simulator/status`);
      if (!reply.ok) throw new OlpRefusal("local simulator status", reply);
      return result(response_format, reply.body, () => `# Local simulator\n\n\`\`\`json\n${json(reply.body)}\n\`\`\``);
    }),
  );
}
