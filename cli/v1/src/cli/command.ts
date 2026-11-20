// The command model and the output contract every `rosie` command keeps, so
// an agent can call any command the same way it calls a web tool:
//
// - text (default): one verdict line first, then a short body, the files it
//   wrote, and the next commands worth running;
// - --json: one envelope {ok, verdict, data, artifacts, next} on stdout;
// - exit 0 ok, 1 refused or a check failed, 2 usage error, 3 a service was
//   unreachable. A refusal quotes OLP's or rt-core's own code and sentence.
import { ServiceUnreachable } from "../lib/http.js";
import { OlpRefusal } from "../lib/olp.js";
import { errorMessage, truncate } from "../lib/text.js";

export type OptionType = "string" | "number" | "boolean" | "numbers" | "strings" | "json";

export interface OptionSpec {
  name: string;
  type: OptionType;
  description: string;
  required?: boolean;
  default?: unknown;
  choices?: readonly string[];
  /** Taken from the n-th positional argument when not given as a flag. */
  positional?: number;
  /** One-letter alias, e.g. "o" for --out. */
  short?: string;
  /** With `positional`: every positional from that index on, as a string list. */
  rest?: boolean;
}

export interface Outcome {
  ok: boolean;
  /** One line an agent can act on without reading further. */
  verdict: string;
  body?: string;
  data?: unknown;
  /** Files written (PNG, HTML, JSON, .rdt). */
  artifacts?: string[];
  /** Follow-up commands worth running, exactly as typed. */
  next?: string[];
  /** Overrides the exit code (default 0 when ok, else 1). */
  exit?: number;
}

export interface Command<E = unknown> {
  path: readonly string[];
  summary: string;
  description?: string;
  options?: readonly OptionSpec[];
  examples?: readonly string[];
  run(env: E, args: Record<string, unknown>): Promise<Outcome>;
}

export class UsageError extends Error {}

export const EXIT = { ok: 0, refused: 1, usage: 2, unreachable: 3 } as const;

export function ok(verdict: string, extra: Omit<Outcome, "ok" | "verdict"> = {}): Outcome {
  return { ok: true, verdict, ...extra };
}

export function refused(verdict: string, extra: Omit<Outcome, "ok" | "verdict"> = {}): Outcome {
  return { ok: false, verdict, ...extra };
}

/** An exception as an outcome: unreachable services exit 3, usage errors 2, everything else is a refusal. */
export function outcomeOf(error: unknown): Outcome {
  if (error instanceof UsageError) return { ok: false, verdict: `usage: ${error.message}`, exit: EXIT.usage };
  if (error instanceof ServiceUnreachable) return { ok: false, verdict: error.message, exit: EXIT.unreachable, next: ["rosie doctor"] };
  if (error instanceof OlpRefusal) return { ok: false, verdict: error.message };
  return { ok: false, verdict: errorMessage(error) };
}

export function exitCode(outcome: Outcome): number {
  return outcome.exit ?? (outcome.ok ? EXIT.ok : EXIT.refused);
}

/** Text: verdict first (prefixed OK/REFUSED), then body, files and next steps. */
export function renderText(outcome: Outcome): string {
  const lines = [`${outcome.ok ? "OK" : outcome.exit === EXIT.unreachable ? "UNREACHABLE" : outcome.exit === EXIT.usage ? "USAGE" : "REFUSED"}: ${outcome.verdict}`];
  if (outcome.body?.trim()) lines.push("", truncate(outcome.body.trimEnd()));
  if (outcome.artifacts?.length) lines.push("", "files:", ...outcome.artifacts.map((file) => `  ${file}`));
  if (outcome.next?.length) lines.push("", "next:", ...outcome.next.map((command) => `  ${command}`));
  return lines.join("\n");
}

export function renderJson(outcome: Outcome): string {
  return JSON.stringify({
    ok: outcome.ok,
    verdict: outcome.verdict,
    exit: exitCode(outcome),
    data: outcome.data ?? null,
    artifacts: outcome.artifacts ?? [],
    next: outcome.next ?? [],
  }, null, 2);
}

// ---------------------------------------------------------------------------
// Arguments

function kebab(name: string): string {
  return name.replace(/_/g, "-");
}

function convert(spec: OptionSpec, raw: string | true): unknown {
  const name = `--${kebab(spec.name)}`;
  if (spec.type === "boolean") {
    if (raw === true) return true;
    if (/^(true|yes|1|on)$/i.test(raw)) return true;
    if (/^(false|no|0|off)$/i.test(raw)) return false;
    throw new UsageError(`${name} takes true or false, got ${JSON.stringify(raw)}`);
  }
  if (raw === true) throw new UsageError(`${name} needs a value`);
  switch (spec.type) {
    case "number": {
      const value = Number(raw);
      if (!Number.isFinite(value)) throw new UsageError(`${name} takes a number, got ${JSON.stringify(raw)}`);
      return value;
    }
    case "numbers": {
      const values = raw.split(/[,\s]+/).filter(Boolean).map(Number);
      if (!values.length || values.some((v) => !Number.isFinite(v))) throw new UsageError(`${name} takes comma-separated numbers, got ${JSON.stringify(raw)}`);
      return values;
    }
    case "strings":
      return raw.split(",").map((s) => s.trim()).filter(Boolean);
    case "json":
      try {
        return JSON.parse(raw);
      } catch {
        throw new UsageError(`${name} takes JSON, got ${JSON.stringify(raw)}`);
      }
    default:
      if (spec.choices && !spec.choices.includes(raw)) throw new UsageError(`${name} is one of ${spec.choices.join(", ")}, got ${JSON.stringify(raw)}`);
      return raw;
  }
}

/** `--key value`, `--key=value`, `--flag`, `--no-flag`, `-k value`, positionals; `--` ends options. */
export function parseArgs(specs: readonly OptionSpec[], argv: readonly string[]): Record<string, unknown> {
  const byName = new Map(specs.map((spec) => [kebab(spec.name), spec]));
  const byShort = new Map(specs.filter((spec) => spec.short).map((spec) => [spec.short!, spec]));
  const values: Record<string, unknown> = {};
  const positionals: string[] = [];
  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i]!;
    if (arg === "--") {
      positionals.push(...argv.slice(i + 1));
      break;
    }
    let spec: OptionSpec | undefined;
    let raw: string | true | undefined;
    if (arg.startsWith("--")) {
      const eq = arg.indexOf("=");
      let key = eq >= 0 ? arg.slice(2, eq) : arg.slice(2);
      if (eq >= 0) raw = arg.slice(eq + 1);
      spec = byName.get(key);
      if (!spec && key.startsWith("no-") && byName.get(key.slice(3))?.type === "boolean") {
        spec = byName.get(key.slice(3));
        key = key.slice(3);
        raw = "false";
      }
      if (!spec) throw new UsageError(`unknown option --${key}`);
    } else if (/^-[A-Za-z]$/.test(arg)) {
      spec = byShort.get(arg.slice(1));
      if (!spec) throw new UsageError(`unknown option ${arg}`);
    } else {
      positionals.push(arg);
      continue;
    }
    if (raw === undefined) {
      const next = argv[i + 1];
      // A negative number is a value, not an option.
      if (spec.type !== "boolean" && next !== undefined && (!next.startsWith("-") || /^-\d|^-\.\d/.test(next))) {
        raw = next;
        i++;
      } else {
        raw = true;
      }
    }
    values[spec.name] = convert(spec, raw);
  }
  const restSpec = specs.find((spec) => spec.rest && spec.positional !== undefined);
  for (const spec of specs) {
    if (spec === restSpec) {
      const tail = positionals.slice(spec.positional);
      if (values[spec.name] === undefined && tail.length) values[spec.name] = tail;
      continue;
    }
    if (values[spec.name] === undefined && spec.positional !== undefined && positionals[spec.positional] !== undefined) {
      values[spec.name] = convert(spec, positionals[spec.positional]!);
    }
  }
  const taken = new Set(specs.filter((s) => s.positional !== undefined).map((s) => s.positional));
  const extra = positionals.filter((_, index) => !taken.has(index) && !(restSpec && index >= restSpec.positional!));
  if (extra.length) throw new UsageError(`unexpected argument${extra.length > 1 ? "s" : ""}: ${extra.join(" ")}`);
  for (const spec of specs) {
    if (values[spec.name] === undefined && spec.default !== undefined) values[spec.name] = spec.default;
    if (values[spec.name] === undefined && spec.required) throw new UsageError(`--${kebab(spec.name)} is required (${spec.description})`);
  }
  return values;
}

export function commandHelp(command: Command<never>): string {
  const specs = command.options ?? [];
  const positional = specs.filter((s) => s.positional !== undefined).sort((a, b) => a.positional! - b.positional!);
  const usage = `rosie ${command.path.join(" ")}${positional.map((s) => (s.required ? ` <${kebab(s.name)}>` : ` [${kebab(s.name)}]`)).join("")}${specs.some((s) => s.positional === undefined) ? " [options]" : ""}`;
  const lines = [usage, "", command.summary];
  if (command.description) lines.push("", command.description.trim());
  if (specs.length) {
    lines.push("", "options:");
    for (const spec of specs) {
      const flag = `${spec.short ? `-${spec.short}, ` : ""}--${kebab(spec.name)}${spec.type === "boolean" ? "" : ` <${spec.choices ? spec.choices.join("|") : spec.type}>`}`;
      const notes = [spec.required ? "required" : "", spec.default !== undefined ? `default ${JSON.stringify(spec.default)}` : ""].filter(Boolean).join(", ");
      lines.push(`  ${flag.padEnd(34)} ${spec.description}${notes ? ` (${notes})` : ""}`);
    }
  }
  lines.push("", "global: --json (envelope on stdout), --target <name>, --ws <workspace>, --help");
  if (command.examples?.length) lines.push("", "examples:", ...command.examples.map((example) => `  ${example}`));
  return lines.join("\n");
}
