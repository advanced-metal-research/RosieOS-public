// "Jog stopped" on the pendant, decided by OLP's own tracker (the one the
// browser toast uses): the server's last classified hold end (status body
// last_jog_end), shown as a toast when this pendant's press began the hold
// and as a quiet note otherwise. The Qt side feeds it every dense status
// poll with this process's X-Rosie-Client-Id and draws what this returns.
import {
  JogEndTracker, type JogEndNotice,
} from "../../../../offline-programming/v1/ui/src/execution/jogEnd.ts";

let tracker: JogEndTracker | null = null;
let trackerClient = "";

/** One dense status body; true when the overlay must redraw. */
export function jogEndObserve(body: unknown, nowMs: number, clientId: string): boolean {
  if (!tracker || trackerClient !== clientId) {
    tracker = new JogEndTracker(clientId);
    trackerClient = clientId;
  }
  return tracker.observe(body, nowMs);
}

/** The × (or the A button while it shows). */
export function jogEndDismiss(): void {
  tracker?.dismiss();
}

/** DETAILS: the technical detail, holding the toast open; again closes it. */
export function jogEndToggleDetails(nowMs: number): void {
  tracker?.toggleDetails(nowMs);
}

export function jogEndView(nowMs: number): JogEndNotice {
  const notice = tracker ? tracker.notice(nowMs) : new JogEndTracker("").notice(nowMs);
  // Older OLP servers name the former R2 jog trigger. Only adapt that
  // instruction; preserve the server's diagnosis and evidence.
  return { ...notice, text: notice.text.replace(
    "Release RT and centre the stick to jog a new direction.",
    "Centre the sticks before jogging a new direction; keep L2 held to stay armed.",
  ) };
}

/** Tests start from a pendant that has seen nothing. */
export function jogEndForget(): void {
  tracker = null;
  trackerClient = "";
}
