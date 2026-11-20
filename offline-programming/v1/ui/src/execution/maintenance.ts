/**
 * The update notice as every client shows it: the browser's blocking overlay
 * (ui/maintenance-overlay.ts) and the Steam Deck pendant (bundled into
 * olp-core.js), so the two cannot disagree about when to block or what to say.
 *
 * The OLP server serves the notice the deploy tooling writes on its host,
 * merged with the selected cell's own notice from its rt-control on the servo
 * host (GET .../maintenance, server maintenance.go and cell_maintenance.go),
 * and refuses robot actions while either is active. A Steam Deck running OLP
 * locally has only the cell's. A client blocks while the server says so, and keeps blocking
 * while the server is unreachable after it said so -- the update may be
 * restarting that server, and a blank-then-normal screen in between would
 * invite a click on a half-updated cell. It stops blocking only when a
 * reachable server reports no notice and its health answers ok.
 *
 * Bundled for Qt 5.15's engine (ES2016): no padStart, no BigInt.
 */

export const MAINTENANCE_API = "/api/offline-programming/v1/maintenance";
export const MAINTENANCE_HEALTH_API = "/api/offline-programming/v1/health";
/** The owner's cadence for noticing a notice (2026-10-07). */
export const MAINTENANCE_POLL_MS = 2000;

/** The selected cell's own notice, as its rt-control last answered. */
export interface MaintenanceCellNotice {
  cell: string;
  active: boolean;
  /** rt-control answered the last read; false while a deploy restarts it. */
  reachable: boolean;
  reason: string;
  error: string;
}

export type MaintenanceStepState = "pending" | "running" | "done" | "failed";

/** One step of the deploy run (steamdeck/real/v4 notice_progress.go). */
export interface MaintenanceProgressStep {
  name: string;
  label: string;
  state: MaintenanceStepState;
  estimateS: number;
  elapsedS: number;
  /** The running step's estimate left when the progress was written. */
  remainingS: number;
}

/** The test gate inside the running step: the newest suite started (its
 *  start order of the total), the suites done and the ones running. */
export interface MaintenanceProgressSuite {
  name: string;
  index: number;
  total: number;
  done: string[];
  running: string[];
}

/** What a deploy run is doing: its steps, the current one, the gate's suites
 *  and the fraction done weighted by estimated time. Absent from older
 *  deploy tools and from a notice that only the servo host carries. */
export interface MaintenanceProgress {
  steps: MaintenanceProgressStep[];
  current: number;
  total: number;
  suite: MaintenanceProgressSuite | null;
  fraction: number;
  /** When the deploy last wrote it (the OLP host's clock). */
  updatedAtMs: number | null;
}

export interface MaintenanceNotice {
  active: boolean;
  reason: string;
  phase: string;
  startedAtMs: number | null;
  etaMs: number | null;
  setBy: string;
  release: string;
  runId: string;
  stale: boolean;
  malformed: boolean;
  error: string;
  /** The server's clock when it answered. */
  serverNowMs: number | null;
  buildId: string;
  cell: MaintenanceCellNotice | null;
  progress: MaintenanceProgress | null;
}

/** One poll's outcome. `absent`: the server answered but has no maintenance
 *  route (404, a server older than the notice). Any other failure, a proxy's
 *  502 included, is `unreachable`. */
export type MaintenanceObservation =
  | { kind: "notice"; payload: unknown }
  | { kind: "absent" }
  | { kind: "unreachable"; error: string };

/** One HTTP answer as an observation: `status` 0 means no answer at all. */
export function maintenanceObservationOf(status: number, body: unknown, error = ""): MaintenanceObservation {
  if (status === 404) return { kind: "absent" };
  if (status >= 200 && status < 300) return { kind: "notice", payload: body };
  return { kind: "unreachable", error: error || (status ? `HTTP ${status}` : "no answer") };
}

export interface MaintenanceDisplay {
  /** Block every input but Stop. */
  blocking: boolean;
  title: string;
  reason: string;
  phase: string;
  /** "Expected finish", "Past the expected finish" or "No expected finish time". */
  countdownLabel: string;
  /** "About 12 min left", "Less than a minute left" or, past the ETA,
   *  "Taking longer than expected (+3 min)"; empty without an ETA. The ETA is an
   *  estimate the deploy re-estimates as its steps finish, so it is shown in
   *  whole minutes, never as a precise or negative clock. */
  countdown: string;
  overrun: boolean;
  /** "Running for 12:34"; empty when the start is unknown. */
  elapsed: string;
  /** 0..1 while the run reports progress, else null: weighted by estimated
   *  time and advanced between updates by the running step's clock. */
  progress: number | null;
  /** "47%"; empty without progress. */
  progressText: string;
  /** "Step 4/8 · Install rt-core on the servo host"; empty without progress. */
  stepLine: string;
  /** "Suite 3/12: ui_test · 2 done" while the test gate runs, else empty. */
  suiteLine: string;
  /** Every step with its mark and time, in run order. */
  steps: MaintenanceStepLine[];
  /** Who started it and what is being installed. */
  source: string;
  notes: string[];
  unreachable: boolean;
  stale: boolean;
  /** Waiting for the server's health after the notice cleared. */
  confirming: boolean;
  /** The notice cleared and the server is a different build than before it. */
  reloadSuggested: boolean;
  buildBefore: string;
  buildAfter: string;
}

/** One row of the compact step list: "✓ Pre-checks 0:02". */
export interface MaintenanceStepLine {
  label: string;
  state: MaintenanceStepState;
  /** ✓ done, ▸ running, ✗ failed, · pending. */
  mark: string;
  /** Done: how long it took; running: "3:10 of about 15 min"; pending: "about 15 min". */
  time: string;
}

function text(value: unknown): string {
  return typeof value === "string" ? value.trim() : "";
}

function count(value: unknown): number {
  return typeof value === "number" && Number.isFinite(value) && value > 0 ? value : 0;
}

function names(value: unknown): string[] {
  return Array.isArray(value) ? value.filter((v): v is string => typeof v === "string") : [];
}

const STEP_STATES: MaintenanceStepState[] = ["pending", "running", "done", "failed"];

/** The notice's progress; null when absent or not one. */
export function normalizeMaintenanceProgress(value: unknown): MaintenanceProgress | null {
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const record = value as Record<string, unknown>;
  if (!Array.isArray(record.steps) || record.steps.length === 0) return null;
  const steps: MaintenanceProgressStep[] = [];
  for (const raw of record.steps) {
    if (!raw || typeof raw !== "object") return null;
    const step = raw as Record<string, unknown>;
    const state = STEP_STATES.indexOf(step.state as MaintenanceStepState) >= 0 ? step.state as MaintenanceStepState : "pending";
    steps.push({ name: text(step.name), label: text(step.label) || text(step.name), state,
      estimateS: count(step.estimate_s), elapsedS: count(step.elapsed_s), remainingS: count(step.remaining_s) });
  }
  const suite = record.suite && typeof record.suite === "object" && !Array.isArray(record.suite) ? record.suite as Record<string, unknown> : null;
  return {
    steps,
    current: Math.floor(count(record.current)),
    total: Math.floor(count(record.total)) || steps.length,
    suite: suite ? { name: text(suite.name), index: Math.floor(count(suite.index)), total: Math.floor(count(suite.total)), done: names(suite.done), running: names(suite.running) } : null,
    fraction: Math.min(1, count(record.fraction)),
    updatedAtMs: time(record.updated_at),
  };
}

/** The fraction done at serverNow: each step weighted by its estimate,
 *  finished steps whole, the running one by the share of its estimate done,
 *  its remaining time counted down from updated_at (at most 99 %, as the
 *  deploy tool computes it). */
export function maintenanceProgressFraction(progress: MaintenanceProgress, serverNow: number): number {
  const since = progress.updatedAtMs === null ? 0 : Math.max(0, serverNow - progress.updatedAtMs) / 1000;
  let weighted = 0;
  let total = 0;
  for (const step of progress.steps) {
    let fraction = 0;
    if (step.state === "done" || step.state === "failed") fraction = 1;
    else if (step.state === "running" && step.estimateS > 0) {
      const left = Math.max(step.remainingS - since, step.estimateS / 20);
      fraction = Math.min(Math.max(1 - left / step.estimateS, 0), 0.99);
    }
    weighted += step.estimateS * fraction;
    total += step.estimateS;
  }
  return total > 0 ? weighted / total : progress.fraction;
}

function stepLines(progress: MaintenanceProgress, serverNow: number): MaintenanceStepLine[] {
  const since = progress.updatedAtMs === null ? 0 : Math.max(0, serverNow - progress.updatedAtMs);
  return progress.steps.map((step) => {
    const mark = step.state === "done" ? "✓" : step.state === "running" ? "▸" : step.state === "failed" ? "✗" : "·";
    let time = `about ${formatMaintenanceMinutes(step.estimateS * 1000)}`;
    if (step.state === "done" || step.state === "failed") time = formatMaintenanceDuration(step.elapsedS * 1000);
    else if (step.state === "running") time = `${formatMaintenanceDuration(step.elapsedS * 1000 + since)} of ${time}`;
    return { label: step.label, state: step.state, mark, time };
  });
}

function time(value: unknown): number | null {
  if (typeof value !== "string" || !value.trim()) return null;
  const ms = Date.parse(value);
  return Number.isFinite(ms) ? ms : null;
}

/** The server's JSON as a notice; null when it is not one. */
export function normalizeMaintenance(payload: unknown): MaintenanceNotice | null {
  if (!payload || typeof payload !== "object" || Array.isArray(payload)) return null;
  const record = payload as Record<string, unknown>;
  if (typeof record.active !== "boolean") return null;
  const server = record.server && typeof record.server === "object" ? record.server as Record<string, unknown> : {};
  const cell = record.cell && typeof record.cell === "object" && !Array.isArray(record.cell) ? record.cell as Record<string, unknown> : null;
  return {
    active: record.active,
    reason: text(record.reason),
    phase: text(record.phase),
    startedAtMs: time(record.started_at),
    etaMs: time(record.eta),
    setBy: text(record.set_by),
    release: text(record.release),
    runId: text(record.run_id),
    stale: record.stale === true,
    malformed: record.malformed === true,
    error: text(record.error),
    serverNowMs: time(record.now),
    buildId: text(server.build_id),
    cell: cell ? { cell: text(cell.cell), active: cell.active === true, reachable: cell.reachable === true, reason: text(cell.reason), error: text(cell.error) } : null,
    progress: normalizeMaintenanceProgress(record.progress),
  };
}

function two(n: number): string {
  return n < 10 ? "0" + n : String(n);
}

/** 754 s -> "12:34"; 3723 s -> "1:02:03". */
export function formatMaintenanceDuration(ms: number): string {
  const total = Math.max(0, Math.floor(ms / 1000));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const seconds = total % 60;
  return hours > 0 ? `${hours}:${two(minutes)}:${two(seconds)}` : `${minutes}:${two(seconds)}`;
}

/** Whole minutes for an estimate: 754 s -> "13 min" (rounded up); 3900 s -> "1 h 5 min". */
export function formatMaintenanceMinutes(ms: number): string {
  const total = Math.max(1, Math.ceil(ms / 60000));
  const hours = Math.floor(total / 60);
  const minutes = total % 60;
  if (hours === 0) return `${minutes} min`;
  return minutes === 0 ? `${hours} h` : `${hours} h ${minutes} min`;
}

/**
 * The notice state across polls. `observe` each poll, `confirmHealth` when
 * `needsHealth` asks for it, and `display` as often as the countdown ticks.
 */
export class MaintenanceTracker {
  private notice: MaintenanceNotice | null = null;
  /** server clock minus this client's clock, from the last answer. */
  private offsetMs = 0;
  private blocking = false;
  private unreachable = false;
  private unreachableError = "";
  private confirming = false;
  private lastBuild = "";
  private buildBefore = "";
  private buildAfter = "";
  private reload = false;

  observe(observation: MaintenanceObservation, nowMs: number): void {
    if (observation.kind === "unreachable") {
      // Never seen a notice: an unreachable server is the app's ordinary
      // disconnected state, not maintenance.
      this.unreachable = this.blocking;
      this.unreachableError = observation.error;
      return;
    }
    this.unreachable = false;
    this.unreachableError = "";
    const notice = observation.kind === "notice" ? normalizeMaintenance(observation.payload) : null;
    if (observation.kind === "notice" && !notice) {
      // A reachable server answering garbage on the notice route: keep the
      // last decision rather than unblocking on noise.
      return;
    }
    if (notice && notice.serverNowMs !== null) this.offsetMs = notice.serverNowMs - nowMs;
    if (notice && notice.active) {
      if (!this.blocking) {
        this.buildBefore = this.lastBuild || notice.buildId;
        this.reload = false;
      }
      this.notice = notice;
      this.blocking = true;
      this.confirming = false;
      return;
    }
    // Inactive or absent: a blocking client first confirms the server is healthy.
    if (this.blocking) {
      this.confirming = true;
      this.buildAfter = notice ? notice.buildId : "";
      return;
    }
    if (notice && notice.buildId) this.lastBuild = notice.buildId;
  }

  /** Whether the host should fetch health now (the notice has cleared). */
  get needsHealth(): boolean {
    return this.blocking && this.confirming && !this.unreachable;
  }

  /** Health after the notice cleared: ok unblocks; a changed build suggests a reload. */
  confirmHealth(ok: boolean, buildId = ""): void {
    if (!this.needsHealth || !ok) return;
    const after = buildId || this.buildAfter;
    this.reload = !!this.buildBefore && !!after && after !== this.buildBefore;
    this.buildAfter = after;
    if (after) this.lastBuild = after;
    this.blocking = false;
    this.confirming = false;
    this.notice = null;
  }

  /** The reload suggestion has been shown and answered. */
  dismissReload(): void {
    this.reload = false;
  }

  get active(): boolean {
    return this.blocking;
  }

  display(nowMs: number): MaintenanceDisplay {
    const notice = this.notice;
    const serverNow = nowMs + this.offsetMs;
    const display: MaintenanceDisplay = {
      blocking: this.blocking,
      title: "Update in progress",
      reason: notice ? notice.reason || "Update in progress" : "",
      phase: notice ? notice.phase : "",
      countdownLabel: "No expected finish time",
      countdown: "",
      overrun: false,
      elapsed: "",
      progress: null,
      progressText: "",
      stepLine: "",
      suiteLine: "",
      steps: [],
      source: "",
      notes: [],
      unreachable: this.unreachable,
      stale: !!notice && notice.stale,
      confirming: this.confirming,
      reloadSuggested: !this.blocking && this.reload,
      buildBefore: this.buildBefore,
      buildAfter: this.buildAfter,
    };
    if (!notice) return display;
    if (notice.etaMs !== null) {
      const remaining = notice.etaMs - serverNow;
      display.overrun = remaining < 0;
      if (display.overrun) {
        const overMinutes = Math.floor(-remaining / 60000);
        display.countdownLabel = "Past the expected finish";
        display.countdown = overMinutes < 1 ? "Taking longer than expected" : `Taking longer than expected (+${formatMaintenanceMinutes(overMinutes * 60000)})`;
      } else {
        display.countdownLabel = "Expected finish";
        display.countdown = remaining < 60000 ? "Less than a minute left" : `About ${formatMaintenanceMinutes(remaining)} left`;
      }
    }
    if (notice.startedAtMs !== null) display.elapsed = `Running for ${formatMaintenanceDuration(serverNow - notice.startedAtMs)}`;
    const progress = notice.progress;
    if (progress) {
      const fraction = maintenanceProgressFraction(progress, serverNow);
      display.progress = fraction;
      display.progressText = `${Math.floor(fraction * 100)}%`;
      display.steps = stepLines(progress, serverNow);
      const current = progress.current > 0 ? progress.steps[progress.current - 1] : undefined;
      if (current) {
        display.stepLine = `Step ${progress.current}/${progress.total} · ${current.label}`;
        // The run writes its step line as the phase for older clients; shown here once.
        if (display.phase.indexOf("Step ") === 0) display.phase = "";
      }
      const suite = progress.suite;
      if (suite && suite.name) {
        const parts = [`Suite ${suite.index}/${suite.total}: ${suite.name}`];
        if (suite.done.length) parts.push(`${suite.done.length} done`);
        if (suite.running.length > 1) parts.push(`${suite.running.length} running`);
        display.suiteLine = parts.join(" · ");
      }
    }
    const source: string[] = [];
    if (notice.setBy) source.push(`Started by ${notice.setBy}`);
    if (notice.release) source.push(`release ${notice.release}`);
    display.source = source.join(" · ");
    if (this.unreachable) {
      display.notes.push("The OLP server is not answering; the update may be restarting it. This screen stays until the server is back and the update has finished.");
    }
    const cell = notice.cell && notice.cell.active ? notice.cell : null;
    const servo = cell && cell.cell ? `the servo host (${cell.cell})` : "the servo host";
    if (cell && !cell.reachable) {
      display.notes.push(`rt-control on ${servo} is not answering; the update may be restarting it. Its last notice is shown, and this screen stays until it answers without one.`);
    }
    if (this.confirming) display.notes.push("The update has finished; checking that the OLP server is healthy…");
    if (notice.malformed) {
      display.notes.push(`The maintenance notice could not be fully read${notice.error ? ` (${notice.error})` : ""}. Robot actions stay refused until it is corrected or cleared${cell ? ` (on ${servo} too)` : " on the OLP host"}.`);
    }
    if (notice.stale) {
      display.notes.push(cell
        ? `This notice is long past its expected finish. The update may have stopped without clearing it: check the deploy, then clear it with \`realtime-core maintenance clear\` (both hosts).`
        : "This notice is long past its expected finish. The update may have stopped without clearing it: check the deploy, then clear it on the OLP host with `offline-programming maintenance clear`.");
    }
    display.notes.push("Robot actions are refused until it ends. Finish what you are doing, then Stop and disarm; STOP stays available here.");
    return display;
  }
}
