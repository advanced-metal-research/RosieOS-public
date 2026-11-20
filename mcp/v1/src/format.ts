// Tool results: one text rendering for the model plus the same data as
// structuredContent. Every tool shares these so a refusal from any service
// reads the same way.
import type { CallToolResult } from "@modelcontextprotocol/server";
import * as z from "zod/v4";

import { errorMessage, json, truncate } from "rosie-cli/lib/text.js";

export { CHARACTER_LIMIT, deg, errorMessage, json, maskAxes, table, truncate, yesNo } from "rosie-cli/lib/text.js";

export const responseFormat = z
  .enum(["markdown", "json"])
  .default("markdown")
  .describe("Output format: 'markdown' for a human-readable summary or 'json' for the full structured data");

export type ResponseFormat = z.infer<typeof responseFormat>;

function asStructured(data: unknown): Record<string, unknown> {
  if (data !== null && typeof data === "object" && !Array.isArray(data)) return data as Record<string, unknown>;
  return { value: data };
}

/** A successful result: markdown or JSON text, with the data as structuredContent. */
export function result(format: ResponseFormat, data: unknown, markdown: () => string): CallToolResult {
  const text = format === "json" ? json(data) : markdown();
  return { content: [{ type: "text", text: truncate(text) }], structuredContent: asStructured(data) };
}

export function text(message: string, data?: unknown): CallToolResult {
  return data === undefined
    ? { content: [{ type: "text", text: truncate(message) }] }
    : { content: [{ type: "text", text: truncate(message) }], structuredContent: asStructured(data) };
}

/** A refusal or failure the model can read and act on; never thrown. */
export function failure(message: string, data?: unknown): CallToolResult {
  return { ...text(`Error: ${message}`, data), isError: true };
}

/** Wraps a handler so an unexpected throw becomes an isError result with the cause. */
export function guarded<A>(handler: (args: A) => Promise<CallToolResult>): (args: A) => Promise<CallToolResult> {
  return async (args: A) => {
    try {
      return await handler(args);
    } catch (error) {
      return failure(errorMessage(error));
    }
  };
}

