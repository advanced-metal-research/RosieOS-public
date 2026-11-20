// OLP stops the robot when the grant is held and no client heartbeat has
// landed for 5 s (denseexec rt_core.go, `ui_heartbeat_lost`). A browser tab
// beats while it is open; this server beats while one of its own robot
// actions is in flight and for a bounded window after the last one that
// succeeded. When the window ends, or this process dies, the beats stop and
// OLP's deadman stops the robot: nothing started from a chat keeps moving
// because a model forgot about it.
import type { OlpClient } from "./olp.js";
import { DENSE } from "./olp.js";

// Unverified cadence: 1 s leaves four missed beats inside OLP's 5 s deadline.
const BEAT_INTERVAL_MS = 1000;
// Unverified bound: a beat's request that has not answered in 1.5 s is
// aborted. Every tick sends its own heartbeat whatever earlier ones are
// doing, so a hung request never delays the next beat; the bound keeps at
// most two open at once, and four beats still fit inside OLP's 5 s deadline.
const BEAT_REQUEST_TIMEOUT_MS = 1500;
// Unverified: how often the session is re-read while heartbeats succeed, to
// notice that OLP released the lease (a refused heartbeat re-reads at once).
const STATUS_REFRESH_MS = 3000;
// Unverified: after an action's answer OLP's status may trail it by a poll;
// for 2 s an unheld session does not stop the beats.
const ACTION_GRACE_MS = 2000;

export interface HeartbeatState {
  active: boolean;
  window_s: number;
  /** Milliseconds until beats stop unless another robot action extends the window. */
  remaining_ms: number;
  /** Robot actions whose request is in flight: the window cannot end and an unheld session does not stop the beats. */
  in_flight: number;
  session_id: string | null;
  beats: number;
  last_error: string | null;
  ended_reason: string | null;
}

export interface KeeperTiming {
  intervalMs?: number;
  requestTimeoutMs?: number;
  statusRefreshMs?: number;
  graceMs?: number;
  now?: () => number;
}

export class HeartbeatKeeper {
  private timer: NodeJS.Timeout | null = null;
  /** End of the window granted by the last success (or extend()); an action in flight never moves it. */
  private until = 0;
  private pins = 0;
  private graceUntil = 0;
  /** Bumped by every stop(): an action in flight across a STOP or Disarm must not restart the beats. */
  private epoch = 0;
  private refreshing = false;
  private statusAt = -Infinity;
  private beats = 0;
  private sessionId: string | null = null;
  private lastError: string | null = null;
  private endedReason: string | null = null;
  private readonly intervalMs: number;
  private readonly requestTimeoutMs: number;
  private readonly statusRefreshMs: number;
  private readonly graceMs: number;
  private readonly now: () => number;

  constructor(
    private readonly olp: OlpClient,
    private readonly windowMs: number,
    timing: KeeperTiming = {},
  ) {
    this.intervalMs = timing.intervalMs ?? BEAT_INTERVAL_MS;
    this.requestTimeoutMs = timing.requestTimeoutMs ?? BEAT_REQUEST_TIMEOUT_MS;
    this.statusRefreshMs = timing.statusRefreshMs ?? STATUS_REFRESH_MS;
    this.graceMs = timing.graceMs ?? ACTION_GRACE_MS;
    this.now = timing.now ?? Date.now;
  }

  /** Starts beating, or pushes the end of the window out to now + window. */
  extend(): void {
    this.until = this.now() + this.windowMs;
    this.start();
  }

  private start(): void {
    this.endedReason = null;
    if (this.timer) return;
    this.timer = setInterval(() => void this.beat(), this.intervalMs);
    this.timer.unref();
    void this.beat();
  }

  /**
   * Runs one robot action with beats around it. An action may arm, take the
   * grant or move before its single request answers (Run program waits up
   * to 300 s), so the beats start before it is sent and nothing ends them
   * while it is in flight but stop(). Success restarts the window from its
   * answer. A failed action does not touch the window, so when the last
   * action in flight ends the keeper is as it was before them: beating out
   * the window of an earlier success, or stopped. Overlapping actions that
   * all refuse keep nothing alive; an action in flight is never mistaken
   * for an earlier success. A STOP or Disarm during the action wins: no
   * restart afterwards.
   */
  async around<T>(action: () => Promise<T>, succeeded: (result: T) => boolean): Promise<T> {
    const epoch = this.epoch;
    this.pins++;
    this.statusAt = -Infinity;
    this.start();
    let ok = false;
    try {
      const result = await action();
      ok = succeeded(result);
      return result;
    } finally {
      this.pins--;
      this.graceUntil = this.now() + this.graceMs;
      this.statusAt = -Infinity;
      if (ok && this.epoch === epoch) this.extend();
      if (this.pins === 0 && this.timer && this.now() >= this.until) {
        this.stop("the action did not succeed: nothing of this server's to keep alive");
      }
    }
  }

  stop(reason: string): void {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
    this.until = 0;
    this.epoch++;
    this.endedReason = reason;
  }

  state(): HeartbeatState {
    return {
      active: this.timer !== null,
      window_s: this.windowMs / 1000,
      remaining_ms: this.timer ? Math.max(0, this.until - this.now()) : 0,
      in_flight: this.pins,
      session_id: this.sessionId,
      beats: this.beats,
      last_error: this.lastError,
      ended_reason: this.endedReason,
    };
  }

  /**
   * One beat: a heartbeat for the last session read, beside (never behind)
   * a status read when one is due. With no session known yet, the status
   * read comes first and the heartbeat follows it.
   */
  async beat(): Promise<void> {
    if (!this.timer) return;
    if (this.pins === 0 && this.now() >= this.until) {
      this.stop("window ended; OLP's deadman stops the robot if it is still held");
      return;
    }
    const known = this.sessionId;
    const refresh = known === null || this.now() - this.statusAt >= this.statusRefreshMs ? this.refresh() : null;
    if (known) {
      await Promise.all([this.heartbeat(known), refresh]);
      return;
    }
    await refresh;
    if (this.timer && this.sessionId) await this.heartbeat(this.sessionId);
  }

  /** The session to beat, from a fresh status; stops when OLP holds none and no action of ours is in flight. */
  private async refresh(): Promise<void> {
    if (this.refreshing) return;
    this.refreshing = true;
    try {
      const status = await this.olp.status(this.requestTimeoutMs);
      this.statusAt = this.now();
      if (!this.timer) return;
      const session = status.session;
      const held = !!session?.id && (session.lease?.held === true || status.armed === true);
      if (!held && this.pins === 0 && this.now() >= this.graceUntil) {
        this.stop("no held session: nothing to keep alive");
        return;
      }
      this.sessionId = session?.id ?? null;
    } catch (error) {
      this.lastError = error instanceof Error ? error.message : String(error);
    } finally {
      this.refreshing = false;
    }
  }

  private async heartbeat(id: string): Promise<void> {
    try {
      const reply = await this.olp.post(`${DENSE}/heartbeat`, { session_id: id }, { timeoutMs: this.requestTimeoutMs });
      if (reply.ok) {
        this.beats++;
        this.lastError = null;
        return;
      }
      this.lastError = `HTTP ${reply.status}: ${JSON.stringify(reply.body).slice(0, 300)}`;
      // dense_session_mismatch: the session ended, or Load / Run program replaced it; the next beat reads status first.
      if (this.sessionId === id) this.sessionId = null;
    } catch (error) {
      this.lastError = error instanceof Error ? error.message : String(error);
    }
  }
}
