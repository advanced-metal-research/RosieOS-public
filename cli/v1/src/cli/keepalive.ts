// OLP stops the robot when the grant holder sends no heartbeat for 5 s. A
// `rosie` command is one process, so a command that arms, loads or moves
// would leave the session to that deadman the moment it exits. After every
// successful motion command, a detached `rosie robot keepalive --serve`
// process (the library's HeartbeatKeeper, the MCP server's rules) beats the
// session for the target's window (default 120 s) from the LAST motion
// command; later commands extend it through the state file. It ends on the
// window, on `rosie robot stop|disarm`, or when OLP no longer holds a session.
// Nothing started from an agent keeps moving because the agent forgot it.
import { spawn } from "node:child_process";
import { mkdir, readFile, rename, rm, writeFile } from "node:fs/promises";
import path from "node:path";

import type { Context } from "../lib/context.js";
import type { Env } from "./env.js";

export interface KeepaliveFile {
  pid: number;
  target: string;
  olp_url: string;
  /** When the last motion command succeeded (ms since epoch): the window runs from here. */
  extend_at: number;
  window_s: number;
  session: string | null;
  stop?: string;
  started_at: number;
  beats?: number;
  last_error?: string | null;
}

export interface KeepaliveView {
  running: boolean;
  pid?: number;
  target?: string;
  session?: string | null;
  until?: string;
  beats?: number;
  last_error?: string | null;
}

function stateFile(home: string): string {
  return path.join(home, "cli", "keepalive.json");
}

async function readState(home: string): Promise<KeepaliveFile | null> {
  try {
    return JSON.parse(await readFile(stateFile(home), "utf8")) as KeepaliveFile;
  } catch {
    return null;
  }
}

async function writeState(home: string, state: KeepaliveFile): Promise<void> {
  const file = stateFile(home);
  await mkdir(path.dirname(file), { recursive: true });
  const temporary = `${file}.${process.pid}.tmp`;
  await writeFile(temporary, JSON.stringify(state, null, 2));
  await rename(temporary, file);
}

function alive(pid: number): boolean {
  try {
    process.kill(pid, 0);
    return true;
  } catch {
    return false;
  }
}

export async function keepaliveState(home: string): Promise<KeepaliveView> {
  const state = await readState(home);
  if (!state || !alive(state.pid) || state.stop) return { running: false };
  return {
    running: true, pid: state.pid, target: state.target, session: state.session,
    until: new Date(state.extend_at + state.window_s * 1000).toISOString(), beats: state.beats, last_error: state.last_error ?? null,
  };
}

/**
 * After a motion command succeeded: extend the running keepalive for this
 * OLP, or start one. A keepalive for another OLP is stopped first (one
 * robot session per CLI user at a time).
 */
export async function extendKeepalive(env: Env, ctx: Context, session: string | null): Promise<KeepaliveView> {
  const resolved = await env.resolved();
  const state = await readState(env.home);
  const now = Date.now();
  if (state && alive(state.pid) && !state.stop && state.olp_url === ctx.config.olpUrl) {
    await writeState(env.home, { ...state, extend_at: now, window_s: ctx.config.heartbeatWindowS, session: session ?? state.session });
    return keepaliveState(env.home);
  }
  if (state && alive(state.pid)) await stopKeepalive(env.home, `superseded by a robot action on ${resolved.name}`);
  const main = process.argv[1]!;
  const args = [main, "robot", "keepalive", "--serve", ...(env.globals.target ? ["--target", env.globals.target] : [])];
  const child = spawn(process.execPath, args, { detached: true, stdio: "ignore", env: process.env });
  child.unref();
  await writeState(env.home, {
    pid: child.pid!, target: resolved.name, olp_url: ctx.config.olpUrl, extend_at: now, window_s: ctx.config.heartbeatWindowS,
    session, started_at: now, beats: 0, last_error: null,
  });
  return keepaliveState(env.home);
}

/** Stop or Disarm pressed: the keepalive ends now (Stop itself already went to OLP). */
export async function stopKeepalive(home: string, reason: string): Promise<boolean> {
  const state = await readState(home);
  if (!state) return false;
  await writeState(home, { ...state, stop: reason });
  if (alive(state.pid)) {
    try {
      process.kill(state.pid, "SIGTERM");
    } catch {
      // already gone
    }
  }
  await rm(stateFile(home), { force: true });
  return true;
}

/**
 * The keepalive process body. Polls the state file every 250 ms: a newer
 * `extend_at` restarts the keeper's window; `stop`, a vanished file, the
 * window's end or OLP's session ending stops it, and the process exits.
 */
export async function serveKeepalive(env: Env): Promise<void> {
  const ctx = await env.ctx();
  let state = await readState(env.home);
  if (!state || state.pid !== process.pid) {
    // Started by hand or raced: adopt the file only if it names no live process.
    if (state && alive(state.pid) && state.pid !== process.pid) return;
    state = { pid: process.pid, target: (await env.resolved()).name, olp_url: ctx.config.olpUrl, extend_at: Date.now(), window_s: ctx.config.heartbeatWindowS, session: null, started_at: Date.now() };
    await writeState(env.home, state);
  }
  let seen = state.extend_at;
  ctx.keeper.extend();
  let ending = false;
  const finish = async (reason: string) => {
    if (ending) return;
    ending = true;
    ctx.keeper.stop(reason);
    const current = await readState(env.home);
    if (current?.pid === process.pid) await rm(stateFile(env.home), { force: true });
    process.exit(0);
  };
  process.on("SIGTERM", () => void finish("stopped"));
  process.on("SIGINT", () => void finish("interrupted"));
  for (;;) {
    await new Promise((resolve) => setTimeout(resolve, 250));
    const current = await readState(env.home);
    if (!current || current.pid !== process.pid || current.stop) return finish(current?.stop ?? "state file removed");
    if (current.extend_at > seen) {
      seen = current.extend_at;
      ctx.keeper.extend();
    }
    const keeper = ctx.keeper.state();
    if (!keeper.active) return finish(keeper.ended_reason ?? "window ended");
    if (keeper.beats !== current.beats || keeper.session_id !== current.session || keeper.last_error !== (current.last_error ?? null)) {
      await writeState(env.home, { ...current, beats: keeper.beats, session: keeper.session_id ?? current.session, last_error: keeper.last_error });
    }
  }
}
