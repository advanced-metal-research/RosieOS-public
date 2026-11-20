// The working program on disk. Every `rosie` command is a fresh process, and
// OLP's engine holds its open program (and its undo history) in memory, so
// the CLI keeps everything the browser keeps beside a program in a
// workspace directory instead -- inspectable, diffable, and never lost to a
// restart:
//
//   ~/.rosie/workspace/<name>/
//     program.olp.json      OLP's Export JSON of the open program
//     session.json          where it came from (cell revision for a checked save),
//                           selection, detected joints, searched seams, torch and
//                           search settings, unsaved flag, the last robot catalogue
//     history/past/*.olp.json, history/future/*.olp.json   undo / redo revisions
//     lock                  one writer at a time
//
// STEP sources, CadQuery mesh replies and planned trajectories live in the
// shared state dir (~/.rosie/mcp by default), keyed by fingerprint/digest.
import { open as openFile, mkdir, readdir, readFile, rename, rm, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Context } from "../lib/context.js";
import type { Engine } from "../lib/engine.js";
import { ServiceUnreachable } from "../lib/http.js";
import type { SessionState } from "../lib/program.js";

/** OLP's ProjectHistory limit (domain/history.ts). */
const HISTORY_LIMIT = 80;
const NAME = /^[A-Za-z0-9._-]{1,64}$/;

interface SessionFile {
  state: SessionState;
  selected: string | null;
  /** Edited since the last save to the cell or a file. */
  unsaved: boolean;
  catalogue: unknown;
  updated_at: string;
}

function alive(pid: number): boolean {
  try {
    process.kill(pid, 0);
    return true;
  } catch {
    return false;
  }
}

/** The project inside an export envelope, without the export timestamp (which changes on every export). */
function projectOf(text: string): string {
  try {
    return JSON.stringify((JSON.parse(text) as { project?: unknown }).project ?? null);
  } catch {
    return text;
  }
}

async function writeAtomic(file: string, content: string): Promise<void> {
  await mkdir(path.dirname(file), { recursive: true });
  const temporary = `${file}.${process.pid}.tmp`;
  await writeFile(temporary, content);
  await rename(temporary, file);
}

export class Workspace {
  private loadedText: string | null = null;
  private unsaved = false;

  private constructor(readonly name: string, readonly dir: string) {}

  static root(home: string): string {
    return path.join(home, "workspace");
  }

  /** Open (creating) a workspace and take its lock; a lock held by a live process refuses. */
  static async open(home: string, name: string): Promise<Workspace> {
    if (!NAME.test(name)) throw new Error(`workspace name ${JSON.stringify(name)}: use letters, digits, . _ -`);
    const dir = path.join(Workspace.root(home), name);
    await mkdir(path.join(dir, "history", "past"), { recursive: true });
    await mkdir(path.join(dir, "history", "future"), { recursive: true });
    const lock = path.join(dir, "lock");
    for (let attempt = 0; attempt < 2; attempt++) {
      try {
        const handle = await openFile(lock, "wx");
        await handle.writeFile(String(process.pid));
        await handle.close();
        return new Workspace(name, dir);
      } catch (error) {
        if ((error as NodeJS.ErrnoException).code !== "EEXIST") throw error;
        const holder = Number((await readFile(lock, "utf8").catch(() => "")).trim());
        if (holder && alive(holder) && holder !== process.pid) {
          throw new Error(`workspace ${name} is in use by process ${holder} (another rosie command); wait for it, or use --ws <other>`);
        }
        await rm(lock, { force: true });
      }
    }
    throw new Error(`could not lock workspace ${name}`);
  }

  static async list(home: string): Promise<{ name: string; program: string | null; unsaved: boolean; updated: string | null }[]> {
    const root = Workspace.root(home);
    const names = await readdir(root).catch(() => [] as string[]);
    const out = [];
    for (const name of names.sort()) {
      const text = await readFile(path.join(root, name, "program.olp.json"), "utf8").catch(() => null);
      const session = JSON.parse(await readFile(path.join(root, name, "session.json"), "utf8").catch(() => "null")) as SessionFile | null;
      let program: string | null = null;
      try {
        program = text ? String((JSON.parse(text) as { project?: { identity?: { name?: string } } }).project?.identity?.name ?? "?") : null;
      } catch {
        program = "?";
      }
      out.push({ name, program, unsaved: session?.unsaved ?? false, updated: session?.updated_at ?? null });
    }
    return out;
  }

  async release(): Promise<void> {
    await rm(path.join(this.dir, "lock"), { force: true });
  }

  private get programFile(): string {
    return path.join(this.dir, "program.olp.json");
  }

  private get sessionFile(): string {
    return path.join(this.dir, "session.json");
  }

  get isUnsaved(): boolean {
    return this.unsaved;
  }

  /**
   * Put the workspace's program and session back into this process's engine:
   * the robot catalogue first (placement hangs off the model's work frame),
   * then the program, then the selection. With OLP unreachable the last
   * catalogue kept here stands in, so reading a program never needs OLP.
   * Answers whether a program is open.
   */
  async restore(ctx: Context, engine: Engine): Promise<boolean> {
    const session = JSON.parse(await readFile(this.sessionFile, "utf8").catch(() => "null")) as SessionFile | null;
    const text = await readFile(this.programFile, "utf8").catch(() => null);
    try {
      await ctx.program.installRobots(engine);
    } catch (error) {
      if (!(error instanceof ServiceUnreachable) || !session?.catalogue) throw error;
      engine.setRobots(session.catalogue, null);
      ctx.program.catalogue = session.catalogue;
    }
    ctx.program.importState(session?.state ?? null);
    this.unsaved = session?.unsaved ?? false;
    if (!text) {
      ctx.program.link = null;
      return false;
    }
    engine.openProject(text);
    if (session?.selected) engine.select(session.selected);
    this.loadedText = text;
    return true;
  }

  /**
   * Write the engine's program and session back. A changed program pushes
   * the revision this command started from onto the undo stack and clears
   * redo, as OLP's ProjectHistory.push does. `saved` marks it saved (to the
   * cell or a file); `fresh` (new/open) starts a new history.
   */
  async persist(ctx: Context, engine: Engine, options: { saved?: boolean; fresh?: boolean; history?: boolean } = {}): Promise<{ changed: boolean }> {
    let text: string | null = null;
    try {
      text = engine.exportProject();
    } catch {
      text = null;
    }
    const changed = text !== null && (this.loadedText === null || projectOf(text) !== projectOf(this.loadedText));
    if (options.fresh) {
      await this.clearHistory();
    } else if (changed && this.loadedText && options.history !== false) {
      await this.push("past", this.loadedText);
      await this.clearStack("future");
    }
    if (text !== null) await writeAtomic(this.programFile, text);
    if (options.saved) this.unsaved = false;
    else if (changed && !options.fresh) this.unsaved = true;
    else if (options.fresh) this.unsaved = ctx.program.link?.source === "new";
    const view = text !== null ? engine.view() : null;
    const session: SessionFile = {
      state: ctx.program.exportState(), selected: view?.selected ?? null, unsaved: this.unsaved,
      catalogue: ctx.program.catalogue, updated_at: new Date().toISOString(),
    };
    await writeAtomic(this.sessionFile, JSON.stringify(session));
    this.loadedText = text;
    return { changed };
  }

  private async stack(which: "past" | "future"): Promise<string[]> {
    return (await readdir(path.join(this.dir, "history", which)).catch(() => [] as string[])).filter((f) => f.endsWith(".olp.json")).sort();
  }

  private async push(which: "past" | "future", text: string): Promise<void> {
    const entries = await this.stack(which);
    const next = entries.length ? Number(entries.at(-1)!.slice(0, 6)) + 1 : 1;
    await writeAtomic(path.join(this.dir, "history", which, `${String(next).padStart(6, "0")}.olp.json`), text);
    for (const old of entries.slice(0, Math.max(0, entries.length + 1 - HISTORY_LIMIT))) await rm(path.join(this.dir, "history", which, old), { force: true });
  }

  private async pop(which: "past" | "future"): Promise<string | null> {
    const entries = await this.stack(which);
    const last = entries.at(-1);
    if (!last) return null;
    const file = path.join(this.dir, "history", which, last);
    const text = await readFile(file, "utf8");
    await rm(file, { force: true });
    return text;
  }

  private async clearStack(which: "past" | "future"): Promise<void> {
    for (const entry of await this.stack(which)) await rm(path.join(this.dir, "history", which, entry), { force: true });
  }

  async clearHistory(): Promise<void> {
    await this.clearStack("past");
    await this.clearStack("future");
  }

  async depth(): Promise<{ undo: number; redo: number }> {
    return { undo: (await this.stack("past")).length, redo: (await this.stack("future")).length };
  }

  /**
   * Undo or redo `steps` revisions through OLP's restoreAsRevision
   * (revision bumped, plan dropped), moving each replaced revision onto the
   * other stack. Answers how many steps were taken.
   */
  async step(ctx: Context, engine: Engine, kind: "undo" | "redo", steps: number): Promise<number> {
    let done = 0;
    for (; done < steps; done++) {
      const snapshot = await this.pop(kind === "undo" ? "past" : "future");
      if (!snapshot) break;
      await this.push(kind === "undo" ? "future" : "past", engine.exportProject());
      engine.restoreSnapshot(snapshot, kind);
    }
    if (done) {
      await writeAtomic(this.programFile, engine.exportProject());
      this.loadedText = engine.exportProject();
      this.unsaved = true;
      const session: SessionFile = {
        state: ctx.program.exportState(), selected: engine.view().selected, unsaved: true, catalogue: ctx.program.catalogue, updated_at: new Date().toISOString(),
      };
      await writeAtomic(this.sessionFile, JSON.stringify(session));
    }
    return done;
  }

  /** Forget the program (new/open replaces it). */
  async reset(): Promise<void> {
    await rm(this.programFile, { force: true });
    await this.clearHistory();
    this.loadedText = null;
    this.unsaved = false;
  }
}
