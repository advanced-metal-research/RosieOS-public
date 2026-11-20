// Shared by the program and authoring commands: run a body against the
// workspace's program inside OLP's engine, and the compact renderings that
// keep a mutation's answer to the lines that changed (the MCP's full
// spreadsheet on every edit was the largest cost in an agent's context).
import type { Context } from "../../lib/context.js";
import type { Engine, Row } from "../../lib/engine.js";
import { refused, UsageError, type Outcome } from "../command.js";
import type { Env } from "../env.js";
import { Workspace } from "../workspace.js";

export interface Program {
  ctx: Context;
  engine: Engine;
  ws: Workspace;
}

export interface ProgramRun {
  /** Writes the program back when it changed (also when the body refuses after a partial edit, as OLP keeps what applied). */
  mutates?: boolean;
  /** Refuse unless the workspace has a program. Default true. */
  needsOpen?: boolean;
  /** new/open: a new history. */
  fresh?: boolean;
  /** save: marks the workspace saved. */
  saved?: boolean;
}

export async function onProgram(env: Env, options: ProgramRun, body: (p: Program) => Promise<Outcome>): Promise<Outcome> {
  const ctx = await env.ctx();
  const ws = await Workspace.open(env.home, env.globals.workspace);
  try {
    return await ctx.program.run(async (engine) => {
      const open = await ws.restore(ctx, engine);
      if ((options.needsOpen ?? true) && !open) {
        return refused(`workspace "${ws.name}" has no program`, { next: ["rosie program new <name>", "rosie program list", "rosie program open <program-id>"] });
      }
      try {
        const outcome = await body({ ctx, engine, ws });
        if (options.mutates) await ws.persist(ctx, engine, { fresh: options.fresh, saved: options.saved && outcome.ok });
        return outcome;
      } catch (error) {
        if (options.mutates && !options.fresh) await ws.persist(ctx, engine).catch(() => undefined);
        throw error;
      }
    });
  } finally {
    await ws.release();
  }
}

/** A row by id, by number in the spreadsheet (3 or #3), or by a unique id prefix. */
export function resolveRow(engine: Engine, ref: unknown): string {
  const text = String(ref ?? "").trim();
  if (!text) throw new UsageError("a row is required (an id or a number from rosie program show)");
  const rows = engine.view().rows;
  const number = /^#?(\d+)$/.exec(text);
  if (number) {
    const row = rows.find((r) => r.number === Number(number[1]));
    if (!row) throw new UsageError(`no row ${number[1]}; the program has ${rows.length} rows`);
    return row.id;
  }
  if (rows.some((r) => r.id === text)) return text;
  const prefixed = rows.filter((r) => r.id.startsWith(text));
  if (prefixed.length === 1) return prefixed[0]!.id;
  throw new UsageError(prefixed.length ? `"${text}" matches rows ${prefixed.map((r) => r.id).join(", ")}` : `no row "${text}"; rosie program show lists them`);
}

/** One spreadsheet row, as the table shows it. */
export function rowLine(row: Row): string {
  const c = row.cells;
  const what = row.op === "move"
    ? `${String(c.motion)} xyz [${[0, 1, 2].map((i) => Number(c[`target.xyz_m.${i}`]).toFixed(1)).join(", ")}] mm rpy [${[0, 1, 2].map((i) => Number(c[`target.rpy_rad.${i}`]).toFixed(1)).join(", ")}]° ${String(c.speed_mm_s)} mm/s ${Number(c.speed_scale).toFixed(0)}%`
    : [row.geometry, row.process, row.freespace].filter(Boolean).join(" · ");
  return `#${row.number} ${row.id} ${row.op} "${row.label}" ${what}${row.status ? ` [${row.status}]` : ""}${row.enabled ? "" : " (disabled)"}`;
}

export function rowById(engine: Engine, id: string): Row | undefined {
  return engine.view().rows.find((row) => row.id === id);
}

/** One line: name, model, rows, part, plan state, problems, saved state. */
export function programLine(p: Program): string {
  const view = p.engine.view();
  const plan = p.engine.planRecord();
  const part = p.engine.partView();
  return `${view.name} (${view.model}) · ${view.rows.length} rows${part.cad ? ` · part ${part.cad.filename}` : ""} · plan ${plan ? (plan.current ? "CURRENT" : "STALE") : "none"}${view.problems.length ? ` · ${view.problems.length} problem(s)` : ""}${p.ws.isUnsaved ? " · unsaved" : ""}`;
}

/** key=value pairs; values parsed as JSON when they are JSON (numbers, true, null, [..]), else strings. */
export function assignments(items: unknown): { path: string; value: unknown }[] {
  const list = Array.isArray(items) ? items.map(String) : [];
  if (!list.length) throw new UsageError("give at least one path=value (paths from rosie program row <row>)");
  return list.map((item) => {
    const eq = item.indexOf("=");
    if (eq <= 0) throw new UsageError(`"${item}" is not path=value`);
    const raw = item.slice(eq + 1);
    let value: unknown = raw;
    try {
      value = JSON.parse(raw);
    } catch {
      value = raw;
    }
    return { path: item.slice(0, eq), value };
  });
}

export function vector3(value: unknown, name: string): number[] | undefined {
  if (value === undefined) return undefined;
  const list = value as number[];
  if (!Array.isArray(list) || list.length !== 3) throw new UsageError(`${name} takes three numbers, e.g. 100,0,250`);
  return list;
}
