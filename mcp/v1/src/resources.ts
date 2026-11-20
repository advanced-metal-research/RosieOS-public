// Resources: the documents and cell manifests a client can attach as context
// without a tool call, and this server's own effective configuration.
import { access, readFile } from "node:fs/promises";
import path from "node:path";

import { ResourceTemplate, type McpServer } from "@modelcontextprotocol/server";

import type { Context } from "rosie-cli/lib/context.js";
import { cellSummary, DOCS, docsFor, listCellNames, readCell } from "./tools/repo.js";

function single(value: string | string[] | undefined): string {
  return Array.isArray(value) ? (value[0] ?? "") : (value ?? "");
}

async function exists(file: string): Promise<boolean> {
  try {
    await access(file);
    return true;
  } catch {
    return false;
  }
}

async function presentDocs(repoRoot: string): Promise<typeof DOCS[number][]> {
  const docs = docsFor(repoRoot);
  const present = await Promise.all(docs.map((doc) => exists(path.join(repoRoot, doc.path))));
  return docs.filter((_, i) => present[i]);
}

export function registerResources(server: McpServer, ctx: Context): void {
  server.registerResource(
    "rosieos-doc",
    new ResourceTemplate("rosieos://docs/{id}", {
      // A listed document can be absent: the real cells' records stay in the
      // private repository and the public mirror omits them. List only what exists.
      list: async () => ({
        resources: (await presentDocs(ctx.config.repoRoot)).map((doc) => ({ uri: `rosieos://docs/${doc.id}`, name: doc.id, title: doc.title, description: doc.path, mimeType: "text/markdown" })),
      }),
    }),
    { title: "RosieOS documentation", description: "Component READMEs, contracts and the repository's working rules", mimeType: "text/markdown" },
    async (uri, variables) => {
      // Only ids from the public and site lists resolve, so no client string
      // reaches the filesystem.
      const docs = docsFor(ctx.config.repoRoot);
      const doc = docs.find((d) => d.id === single(variables.id));
      if (!doc) throw new Error(`unknown document ${uri.href}; known: ${docs.map((d) => d.id).join(", ")}`);
      const file = path.join(ctx.config.repoRoot, doc.path);
      if (!(await exists(file))) throw new Error(`document ${doc.id} (${doc.path}) is not in this checkout`);
      return { contents: [{ uri: uri.href, mimeType: "text/markdown", text: await readFile(file, "utf8") }] };
    },
  );

  server.registerResource(
    "rosieos-cell",
    new ResourceTemplate("rosieos://cells/{cell}", {
      list: async () => ({
        resources: (await listCellNames(ctx)).map((cell) => ({ uri: `rosieos://cells/${cell}`, name: cell, description: `rt-core/config/cells/${cell}.json`, mimeType: "application/json" })),
      }),
    }),
    { title: "RosieOS cell manifest", description: "A committed cell manifest from rt-core/config/cells", mimeType: "application/json" },
    async (uri, variables) => {
      const cell = single(variables.cell);
      if (!(await listCellNames(ctx)).includes(cell)) throw new Error(`no committed cell manifest ${cell}`);
      const manifest = await readCell(ctx, cell);
      return { contents: [{ uri: uri.href, mimeType: "application/json", text: JSON.stringify({ summary: cellSummary(cell, manifest), manifest }, null, 2) }] };
    },
  );

  server.registerResource(
    "rosieos-mcp-config",
    "rosieos://mcp/config",
    { title: "This MCP server's configuration", description: "Endpoints, robot-action mode and heartbeat window in effect", mimeType: "application/json" },
    async (uri) => {
      const { config } = ctx;
      const view = {
        repo_root: config.repoRoot,
        olp_url: config.olpUrl,
        planner_url: config.plannerUrl,
        dense_daemon: `${config.denseDaemonAddr.host}:${config.denseDaemonAddr.port}`,
        rt_control: ctx.rt.configured ? ctx.rt.target : "not configured",
        cartesiand: config.cartesiandCommand ?? "not available",
        robot_actions: config.robotActions,
        heartbeat_window_s: config.heartbeatWindowS,
        jog_limits: config.jog,
        olp_client_id: ctx.olp.clientId,
        client_label: config.clientLabel,
        heartbeat: ctx.keeper.state(),
      };
      return { contents: [{ uri: uri.href, mimeType: "application/json", text: JSON.stringify(view, null, 2) }] };
    },
  );
}
