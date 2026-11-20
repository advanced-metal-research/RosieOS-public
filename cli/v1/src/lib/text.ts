// Plain-text helpers every front end renders with (the CLI's text output, the
// MCP server's markdown). No protocol types here: an answer reads the same
// whichever front end asked.

/** Largest text rendering returned; the full data stays in --json / structuredContent or a saved file. */
export const CHARACTER_LIMIT = 25_000;

export function truncate(text: string, limit = CHARACTER_LIMIT): string {
  if (text.length <= limit) return text;
  return `${text.slice(0, limit)}\n\n[truncated ${text.length - limit} characters; use the JSON output with narrower parameters, or the saved file]`;
}

export function json(value: unknown): string {
  return JSON.stringify(value, null, 2);
}

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error);
}

export function deg(rad: number | null | undefined): string {
  return typeof rad === "number" && Number.isFinite(rad) ? `${((rad * 180) / Math.PI).toFixed(3)}°` : "n/a";
}

export function yesNo(value: unknown): string {
  return value === true || value === 1 ? "yes" : value === false || value === 0 ? "no" : "n/a";
}

export function maskAxes(mask: number | null | undefined, count = 16): string {
  if (typeof mask !== "number" || mask === 0) return "none";
  const axes: string[] = [];
  for (let bit = 0; bit < count; bit++) if (mask & (1 << bit)) axes.push(`J${bit + 1}`);
  return axes.join(", ");
}

/** A markdown table: what both an agent and a terminal read without help. */
export function table(headers: string[], rows: unknown[][]): string {
  const cell = (value: unknown) => String(value ?? "").replace(/\|/g, "\\|").replace(/\n/g, " ");
  return [
    `| ${headers.join(" | ")} |`,
    `| ${headers.map(() => "---").join(" | ")} |`,
    ...rows.map((row) => `| ${row.map(cell).join(" | ")} |`),
  ].join("\n");
}

export function record(value: unknown): Record<string, unknown> {
  return value && typeof value === "object" && !Array.isArray(value) ? (value as Record<string, unknown>) : {};
}
