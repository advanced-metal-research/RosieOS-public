#!/usr/bin/env node
// stdio entry point. stdout is the protocol channel: everything human goes to
// stderr. Only stdio is served: robot actions from a network listener would
// need authentication this server does not have.
import { serveStdio } from "@modelcontextprotocol/server/stdio";

import { loadConfig } from "rosie-cli/lib/config.js";
import { createContext, type Context } from "rosie-cli/lib/context.js";
import { createServer, SERVER_NAME, SERVER_VERSION } from "./server.js";

if (process.argv.includes("--help") || process.argv.includes("-h")) {
  console.error(`${SERVER_NAME} ${SERVER_VERSION}: MCP server (stdio) for RosieOS.
Environment:
  ROSIE_OLP_URL                 OLP API (default http://127.0.0.1:8794)
  ROSIE_PLANNER_URL             weld planner motion server (default http://127.0.0.1:8796)
  ROSIE_DENSE_DAEMON_ADDR       dense daemon TCP ingest (default 127.0.0.1:8797)
  ROSIE_RT_CONTROL_SOCKET       rt-control public control socket, or
  ROSIE_RT_CONTROL_URL          rt-control mTLS listener with ROSIE_RT_CONTROL_CA/CERT/KEY
  ROSIE_CARTESIAND_COMMAND      robot-v4-cartesiand path or JSON argv prefix
  ROSIE_CARTESIAND_REPO_ROOT    repository root as cartesiand sees it
  ROSIE_MCP_ROBOT_ACTIONS       off (default) | simulation | cell
  ROSIE_MCP_HEARTBEAT_WINDOW_S  heartbeat window after a robot action (default 120)
  ROSIE_REPO_ROOT, ROSIE_GO, ROSIE_MCP_CLIENT_LABEL, ROSIE_MCP_REQUEST_TIMEOUT_MS
See mcp/v1/README.md.`);
  process.exit(0);
}

let context: Context;
try {
  context = createContext(loadConfig());
} catch (error) {
  console.error(`${SERVER_NAME}: ${error instanceof Error ? error.message : String(error)}`);
  process.exit(2);
}
serveStdio(() => createServer(context), { onerror: (error) => console.error(`${SERVER_NAME}: ${error.message}`) });
console.error(
  `${SERVER_NAME} ${SERVER_VERSION} on stdio · OLP ${context.config.olpUrl} · planner ${context.config.plannerUrl} · rt-control ${context.rt.configured ? context.rt.target : "not configured"} · robot actions ${context.config.robotActions}`,
);
