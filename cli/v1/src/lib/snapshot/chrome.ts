// Headless Chrome and a minimal Chrome DevTools Protocol client, over Node's
// global WebSocket (Node >= 22): no puppeteer, no playwright, no npm package
// that pins a browser download. The browser is the one already installed.
//
// A Chrome left running holds a GPU process and a profile directory; one
// left per snapshot run fills a laptop by the afternoon. `withChrome` kills
// the process and removes its profile on every path out, failure included.
import { spawn, type ChildProcess } from "node:child_process";
import { existsSync } from "node:fs";
import { mkdtemp, readFile, rm } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { setTimeout as sleep } from "node:timers/promises";

/** Where a browser is looked for, after ROSIE_CHROME. Edge is Chromium and
 *  speaks the same protocol (AGENT_SCRATCHPAD 2026-10-07 rendered this scene
 *  in headless Edge). */
const MAC_BROWSERS = [
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
  "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
  "/Applications/Chromium.app/Contents/MacOS/Chromium",
];
const PATH_BROWSERS = ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "microsoft-edge"];

/** WebGL with no GPU: SwiftShader through ANGLE renders the three.js scene
 *  headless (the flags the scratchpad's working Edge run used). */
const HEADLESS_FLAGS = [
  "--headless=new",
  "--remote-debugging-port=0",
  "--enable-unsafe-swiftshader",
  "--use-angle=swiftshader",
  "--hide-scrollbars",
  "--no-first-run",
  "--no-default-browser-check",
  "--disable-background-networking",
  "--disable-extensions",
  "--disable-sync",
  "--mute-audio",
];

/** The browser to run: ROSIE_CHROME, else a known install, else null. A set
 *  ROSIE_CHROME that does not exist is an error, not a fallback: the caller
 *  asked for that browser. */
export function findChrome(env: NodeJS.ProcessEnv = process.env): string | null {
  const chosen = env.ROSIE_CHROME?.trim();
  if (chosen) {
    if (!existsSync(chosen)) throw new Error(`ROSIE_CHROME=${chosen} does not exist`);
    return chosen;
  }
  for (const candidate of MAC_BROWSERS) if (existsSync(candidate)) return candidate;
  for (const dir of (env.PATH ?? "").split(path.delimiter)) {
    for (const name of PATH_BROWSERS) {
      const candidate = path.join(dir, name);
      if (dir && existsSync(candidate)) return candidate;
    }
  }
  return null;
}

type Handler = (params: Record<string, unknown>, sessionId: string | undefined) => void;

interface Pending {
  method: string;
  resolve: (value: Record<string, unknown>) => void;
  reject: (error: Error) => void;
  timer: NodeJS.Timeout;
}

/** One WebSocket to the browser; page commands ride it with a flattened
 *  `sessionId` (Target.attachToTarget {flatten: true}). */
export class CdpClient {
  readonly #ws: WebSocket;
  #nextId = 1;
  readonly #pending = new Map<number, Pending>();
  readonly #handlers = new Map<string, Set<Handler>>();
  #closed: Error | null = null;

  private constructor(ws: WebSocket) {
    this.#ws = ws;
    ws.addEventListener("message", (event) => this.#receive(String(event.data)));
    ws.addEventListener("close", () => this.#fail(new Error("the DevTools connection closed")));
    ws.addEventListener("error", () => this.#fail(new Error("the DevTools connection failed")));
  }

  static connect(url: string, timeoutMs = 10_000): Promise<CdpClient> {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(url);
      const timer = setTimeout(() => {
        ws.close();
        reject(new Error(`DevTools did not accept a connection at ${url} within ${timeoutMs} ms`));
      }, timeoutMs);
      ws.addEventListener("open", () => {
        clearTimeout(timer);
        resolve(new CdpClient(ws));
      }, { once: true });
      ws.addEventListener("error", () => {
        clearTimeout(timer);
        reject(new Error(`cannot connect to DevTools at ${url}`));
      }, { once: true });
    });
  }

  /** Send a command and wait for its answer; a protocol error rejects with
   *  the method named, so a failed snapshot says which step failed. */
  send<T extends Record<string, unknown> = Record<string, unknown>>(
    method: string,
    params: Record<string, unknown> = {},
    sessionId?: string,
    timeoutMs = 30_000,
  ): Promise<T> {
    if (this.#closed) return Promise.reject(this.#closed);
    const id = this.#nextId++;
    return new Promise<T>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.#pending.delete(id);
        reject(new Error(`${method} got no answer within ${timeoutMs} ms`));
      }, timeoutMs);
      this.#pending.set(id, { method, resolve: resolve as (value: Record<string, unknown>) => void, reject, timer });
      this.#ws.send(JSON.stringify({ id, method, params, ...(sessionId ? { sessionId } : {}) }));
    });
  }

  /** Listen for an event; returns the unsubscribe. */
  on(method: string, handler: Handler): () => void {
    let set = this.#handlers.get(method);
    if (!set) this.#handlers.set(method, (set = new Set()));
    set.add(handler);
    return () => set.delete(handler);
  }

  close(): void {
    this.#fail(new Error("the DevTools connection was closed"));
    try {
      this.#ws.close();
    } catch {
      // Already closing.
    }
  }

  #receive(raw: string): void {
    let message: { id?: number; method?: string; params?: Record<string, unknown>; result?: Record<string, unknown>; error?: { message?: string; data?: string }; sessionId?: string };
    try {
      message = JSON.parse(raw);
    } catch {
      return;
    }
    if (typeof message.id === "number") {
      const pending = this.#pending.get(message.id);
      if (!pending) return;
      this.#pending.delete(message.id);
      clearTimeout(pending.timer);
      if (message.error) {
        pending.reject(new Error(`${pending.method}: ${message.error.message ?? "protocol error"}${message.error.data ? ` (${message.error.data})` : ""}`));
      } else {
        pending.resolve(message.result ?? {});
      }
      return;
    }
    if (message.method) {
      for (const handler of this.#handlers.get(message.method) ?? []) handler(message.params ?? {}, message.sessionId);
    }
  }

  #fail(error: Error): void {
    if (this.#closed) return;
    this.#closed = error;
    for (const [id, pending] of this.#pending) {
      clearTimeout(pending.timer);
      pending.reject(new Error(`${pending.method}: ${error.message}`));
      this.#pending.delete(id);
    }
  }
}

export interface ChromeSession {
  executable: string;
  cdp: CdpClient;
  /** The browser's DevTools endpoint, ws://127.0.0.1:<port>/devtools/browser/<id>. */
  browserUrl: string;
}

/** The DevTools endpoint Chrome writes into its profile once it listens:
 *  line 1 the port, line 2 the browser target's path. */
async function devToolsUrl(profile: string, child: ChildProcess, stderr: () => string, timeoutMs: number): Promise<string> {
  const deadline = Date.now() + timeoutMs;
  const file = path.join(profile, "DevToolsActivePort");
  while (Date.now() < deadline) {
    if (child.exitCode !== null || child.signalCode !== null) {
      throw new Error(`the browser exited (${child.exitCode ?? child.signalCode}) before DevTools listened: ${stderr().trim().slice(-800)}`);
    }
    try {
      const [port, browserPath] = (await readFile(file, "utf8")).split("\n").map((line) => line.trim());
      if (port && browserPath) return `ws://127.0.0.1:${port}${browserPath}`;
    } catch {
      // Not written yet.
    }
    const match = /DevTools listening on (ws:\/\/\S+)/.exec(stderr());
    if (match) return match[1]!;
    await sleep(50);
  }
  throw new Error(`the browser did not open DevTools within ${timeoutMs} ms: ${stderr().trim().slice(-800)}`);
}

async function stop(child: ChildProcess, graceMs: number): Promise<void> {
  if (child.exitCode !== null || child.signalCode !== null) return;
  const exited = new Promise<void>((resolve) => child.once("exit", () => resolve()));
  child.kill("SIGTERM");
  const waitExit = async (): Promise<boolean> => {
    const abort = new AbortController();
    try {
      return await Promise.race([exited.then(() => true), sleep(graceMs, false, { signal: abort.signal }).catch(() => false)]);
    } finally {
      abort.abort();
    }
  };
  if (!(await waitExit())) {
    child.kill("SIGKILL");
    await waitExit();
  }
}

/**
 * Run `use` against a fresh headless browser with its own throwaway profile.
 * The browser is closed, killed if it will not close, and its profile
 * removed whatever `use` does.
 */
export async function withChrome<T>(
  use: (session: ChromeSession) => Promise<T>,
  options: { executable?: string; timeoutMs?: number; extraArgs?: string[] } = {},
): Promise<T> {
  const executable = options.executable ?? findChrome();
  if (!executable) {
    throw new Error("no Chrome found: install Google Chrome or Microsoft Edge, or set ROSIE_CHROME to a Chromium executable");
  }
  const timeoutMs = options.timeoutMs ?? 20_000;
  const profile = await mkdtemp(path.join(os.tmpdir(), "rosie-snapshot-chrome-"));
  let child: ChildProcess | null = null;
  let cdp: CdpClient | null = null;
  try {
    let stderr = "";
    child = spawn(executable, [...HEADLESS_FLAGS, `--user-data-dir=${profile}`, ...(options.extraArgs ?? []), "about:blank"], {
      stdio: ["ignore", "ignore", "pipe"],
    });
    child.stderr?.setEncoding("utf8");
    child.stderr?.on("data", (chunk: string) => {
      stderr = (stderr + chunk).slice(-16_384);
    });
    const spawned = child;
    const spawnError = new Promise<never>((_, reject) => spawned.once("error", (error) => reject(new Error(`cannot start ${executable}: ${error.message}`))));
    const browserUrl = await Promise.race([devToolsUrl(profile, spawned, () => stderr, timeoutMs), spawnError]);
    cdp = await CdpClient.connect(browserUrl, timeoutMs);
    return await use({ executable, cdp, browserUrl });
  } finally {
    if (cdp) {
      await cdp.send("Browser.close", {}, undefined, 2_000).catch(() => undefined);
      cdp.close();
    }
    if (child) await stop(child, 3_000);
    // Chrome may still be flushing its profile for a moment after exit.
    await rm(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 }).catch(() => undefined);
  }
}
