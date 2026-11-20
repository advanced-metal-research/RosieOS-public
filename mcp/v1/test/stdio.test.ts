// The built entry point over real stdio, spawned the way an MCP host spawns it.
import assert from "node:assert/strict";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { test } from "node:test";

import { Client } from "@modelcontextprotocol/client";
import { StdioClientTransport } from "@modelcontextprotocol/client/stdio";

const entry = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../src/index.js");

test("stdio: the entry point serves annotated tools and names an unreachable OLP", async () => {
  const client = new Client({ name: "rosieos-mcp-stdio-test", version: "1.0.0" });
  await client.connect(new StdioClientTransport({ command: process.execPath, args: [entry], env: { ...process.env, ROSIE_MCP_ROBOT_ACTIONS: "off", ROSIE_OLP_URL: "http://127.0.0.1:9" } as Record<string, string> }));
  try {
    const tools = (await client.listTools()).tools;
    assert.ok(tools.some((tool) => tool.name === "rosie_olp_stop"));
    assert.ok(tools.every((tool) => tool.annotations !== undefined && typeof tool.description === "string" && tool.description.length > 40), "every tool is annotated and described");
    const unreachable = await client.callTool({ name: "rosie_olp_get_machine_status", arguments: {} });
    assert.equal(unreachable.isError, true);
    assert.match(String((unreachable.content as Array<{ text: string }>)[0]?.text), /OLP at http:\/\/127\.0\.0\.1:9.* is unreachable/);
  } finally {
    await client.close();
  }
});
