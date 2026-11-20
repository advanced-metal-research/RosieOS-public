// The update notice on the pendant, decided by OLP's own tracker (the one the
// browser overlay uses): when to block, the countdown, the overrun, the
// unreachable-while-updating rule and the words. The Qt side polls the OLP
// server's maintenance route and draws what this returns, including who is
// connected (OLP's maintenanceSessions).
import {
  MaintenanceTracker, maintenanceObservationOf, normalizeMaintenance, formatMaintenanceDuration,
  type MaintenanceDisplay, type MaintenanceNotice,
} from "../../../../offline-programming/v1/ui/src/execution/maintenance.ts";
import {
  maintenanceSessionLines, type MaintenanceSessionLine,
} from "../../../../offline-programming/v1/ui/src/execution/maintenanceSessions.ts";

/** A notice this long past its expected finish (without one: this long up) is
 *  stale: the deploy that set it probably died without clearing it. The OLP
 *  server's default (maintenance.go defaultMaintenanceStaleAfter, 2 h) and the
 *  documented client rule; clients flag it and never clear it. The pendant
 *  applies it itself as well, so a notice whose server stopped answering
 *  still turns stale on the Deck's screen. */
export const MAINTENANCE_STALE_AFTER_MS = 2 * 60 * 60 * 1000;

/** OLP's display plus the pendant's stale flag. `source` also names the run,
 *  which `realtime-core maintenance show` prints beside its owner. */
export interface PendantMaintenanceView extends MaintenanceDisplay {
  /** "STALE: 2:13:00 past its expected finish"; empty unless stale. */
  staleLine: string;
}

let tracker = new MaintenanceTracker();
// The last answered body (for its client list) and the server clock offset.
let lastBody: unknown = null;
let offsetMs = 0;
// The last active notice answered, while the tracker blocks on it.
let notice: MaintenanceNotice | null = null;

/** One poll: `status` is the HTTP status (0: the server never answered).
 *  Returns whether the server's health must be read before unblocking. */
export function maintenanceObserve(status: number, body: unknown, error: string, nowMs: number): boolean {
  tracker.observe(maintenanceObservationOf(status, body, error), nowMs);
  if (status >= 200 && status < 300 && body && typeof body === "object") {
    lastBody = body;
    const now = Date.parse(String((body as Record<string, unknown>).now));
    if (Number.isFinite(now)) offsetMs = now - nowMs;
    const answered = normalizeMaintenance(body);
    if (answered && answered.active) notice = answered;
  }
  if (!tracker.active) notice = null;
  return tracker.needsHealth;
}

/** The health reply once the notice has cleared. */
export function maintenanceHealth(ok: boolean, buildId: string): void {
  tracker.confirmHealth(ok, buildId);
  if (!tracker.active) notice = null;
}

export function maintenanceView(nowMs: number): PendantMaintenanceView {
  const view: PendantMaintenanceView = { ...tracker.display(nowMs), staleLine: "" };
  if (!view.blocking || !notice) return view;
  if (notice.runId) view.source = view.source ? `${view.source} · run ${notice.runId}` : `run ${notice.runId}`;
  const since = notice.etaMs !== null ? notice.etaMs : notice.startedAtMs;
  const over = since === null ? 0 : nowMs + offsetMs - since;
  view.stale = view.stale || over > MAINTENANCE_STALE_AFTER_MS;
  if (view.stale) {
    view.staleLine = notice.etaMs !== null
      ? `STALE: ${formatMaintenanceDuration(over)} past its expected finish`
      : `STALE: up for ${formatMaintenanceDuration(over)} with no expected finish`;
  }
  return view;
}

/** Who is connected, as of the last answer (empty from an older server). */
export function maintenanceSessions(nowMs: number): MaintenanceSessionLine[] {
  return maintenanceSessionLines(lastBody, nowMs + offsetMs);
}

export function maintenanceDismissReload(): void {
  tracker.dismissReload();
}

/** Tests start from a client that has seen nothing. */
export function maintenanceForget(): void {
  tracker = new MaintenanceTracker();
  lastBody = null;
  offsetMs = 0;
  notice = null;
}
