// An in-process MCP client through the SDK's own handler, so tests drive the
// server exactly as a host does; the process-boundary fakes are the CLI's
// (rosie-cli/testing), shared by both front ends.
import { Client, StreamableHTTPClientTransport } from "@modelcontextprotocol/client";
import { createMcpHandler } from "@modelcontextprotocol/server";

import { loadConfig } from "rosie-cli/lib/config.js";
import { createContext, type Context } from "rosie-cli/lib/context.js";
import { HeartbeatKeeper, type KeeperTiming } from "rosie-cli/lib/heartbeat.js";
import { REPO_ROOT } from "rosie-cli/testing";

import { createServer } from "../src/server.js";

export { REPO_ROOT, simTarget, startFakeOlp, startFakeRtControl, urdfTcpMmOf, type FakeOlp, type FakeRoute, type Recorded } from "rosie-cli/testing";

/** `keeper` swaps in a keeper with test timing (ms cadence instead of 1 s) before any server is built. */
export async function connect(env: Record<string, string>, options: { keeper?: KeeperTiming } = {}): Promise<{ client: Client; ctx: Context; close(): Promise<void> }> {
  const ctx = createContext(loadConfig({ ROSIE_REPO_ROOT: REPO_ROOT, ...env }));
  if (options.keeper) ctx.keeper = new HeartbeatKeeper(ctx.olp, ctx.config.heartbeatWindowS * 1000, options.keeper);
  const handler = createMcpHandler(() => createServer(ctx));
  const transport = new StreamableHTTPClientTransport(new URL("http://test.local/mcp"), {
    fetch: (url, init) => handler.fetch(new Request(url, init)),
  });
  const client = new Client({ name: "rosieos-mcp-test", version: "1.0.0" }, { versionNegotiation: { mode: "auto" } });
  await client.connect(transport);
  return {
    client,
    ctx,
    close: async () => {
      ctx.keeper.stop("test ended");
      await client.close();
      await handler.close();
    },
  };
}

export function textOf(result: { content?: unknown }): string {
  const content = (result.content ?? []) as Array<{ type: string; text?: string }>;
  return content.map((block) => block.text ?? "").join("\n");
}

