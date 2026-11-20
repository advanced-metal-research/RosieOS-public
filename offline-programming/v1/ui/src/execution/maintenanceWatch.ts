import {
  MAINTENANCE_API, MAINTENANCE_HEALTH_API, MAINTENANCE_POLL_MS, MaintenanceTracker, maintenanceObservationOf,
  type MaintenanceDisplay, type MaintenanceObservation,
} from "./maintenance.ts";
import { maintenanceSessionLines, type MaintenanceSessionLine } from "./maintenanceSessions.ts";

/**
 * Polls the OLP server's maintenance notice and keeps a renderer current: the
 * notice every 2 s, the countdown twice a second while it blocks, and the
 * server's health once the notice clears. Same-origin fetches, like every
 * other API call the app makes (Vite proxies them to :8794 in development).
 */
export interface MaintenanceWatchOptions {
  /** `sessions`: who is connected, from the last answer (maintenanceSessions.ts). */
  render: (display: MaintenanceDisplay, sessions: MaintenanceSessionLine[]) => void;
  /** Every answered body, for what else rides on the poll (its `control`). */
  onBody?: (body: unknown) => void;
  fetch?: (input: string, init?: RequestInit) => Promise<Response>;
  now?: () => number;
  pollMs?: number;
  /** A poll that takes longer counts as unreachable. Unverified bound. */
  timeoutMs?: number;
}

export class MaintenanceWatch {
  readonly tracker = new MaintenanceTracker();
  readonly #options: MaintenanceWatchOptions;
  readonly #fetch: (input: string, init?: RequestInit) => Promise<Response>;
  readonly #now: () => number;
  #pollTimer: ReturnType<typeof setTimeout> | null = null;
  #tickTimer: ReturnType<typeof setInterval> | null = null;
  #stopped = false;
  #lastBody: unknown = null;
  #offsetMs = 0;

  constructor(options: MaintenanceWatchOptions) {
    this.#options = options;
    this.#fetch = options.fetch ?? ((input, init) => fetch(input, init));
    this.#now = options.now ?? (() => Date.now());
  }

  start(): this {
    this.#stopped = false;
    void this.#loop();
    return this;
  }

  stop(): void {
    this.#stopped = true;
    if (this.#pollTimer !== null) clearTimeout(this.#pollTimer);
    if (this.#tickTimer !== null) clearInterval(this.#tickTimer);
    this.#pollTimer = this.#tickTimer = null;
  }

  /** One poll: the notice, then health when the notice has just cleared. */
  async poll(): Promise<MaintenanceDisplay> {
    this.tracker.observe(await this.#observe(), this.#now());
    if (this.tracker.needsHealth) {
      const health = await this.#json(MAINTENANCE_HEALTH_API);
      const ok = !!health && health.ok === true;
      this.tracker.confirmHealth(ok, ok && typeof health!.build_id === "string" ? health!.build_id : "");
    }
    return this.render();
  }

  render(): MaintenanceDisplay {
    const now = this.#now();
    const display = this.tracker.display(now);
    this.#options.render(display, maintenanceSessionLines(this.#lastBody, now + this.#offsetMs));
    this.#tick(display.blocking);
    return display;
  }

  dismissReload(): void {
    this.tracker.dismissReload();
    this.render();
  }

  async #loop(): Promise<void> {
    if (this.#stopped) return;
    try {
      await this.poll();
    } finally {
      if (!this.#stopped) this.#pollTimer = setTimeout(() => void this.#loop(), this.#options.pollMs ?? MAINTENANCE_POLL_MS);
    }
  }

  #tick(blocking: boolean): void {
    if (blocking && this.#tickTimer === null && !this.#stopped) {
      this.#tickTimer = setInterval(() => this.render(), 500);
    } else if (!blocking && this.#tickTimer !== null) {
      clearInterval(this.#tickTimer);
      this.#tickTimer = null;
    }
  }

  async #observe(): Promise<MaintenanceObservation> {
    try {
      const response = await this.#request(MAINTENANCE_API);
      const body = response.ok ? await response.json() : null;
      if (response.ok && body && typeof body === "object") {
        this.#lastBody = body;
        const serverNow = Date.parse(String((body as Record<string, unknown>).now));
        if (Number.isFinite(serverNow)) this.#offsetMs = serverNow - this.#now();
        this.#options.onBody?.(body);
      }
      return maintenanceObservationOf(response.status, body);
    } catch (error) {
      return { kind: "unreachable", error: error instanceof Error ? error.message : String(error) };
    }
  }

  async #json(path: string): Promise<Record<string, unknown> | null> {
    try {
      const response = await this.#request(path);
      if (!response.ok) return null;
      const body = await response.json();
      return body && typeof body === "object" ? body as Record<string, unknown> : null;
    } catch {
      return null;
    }
  }

  async #request(path: string): Promise<Response> {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.#options.timeoutMs ?? 1500);
    try {
      return await this.#fetch(path, { cache: "no-store", headers: { Accept: "application/json" }, signal: controller.signal });
    } finally {
      clearTimeout(timer);
    }
  }
}
