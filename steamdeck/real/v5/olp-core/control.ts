// "Another controller is using the robot" on the pendant, decided by OLP's own
// tracker (the one the browser pop-up uses): who holds the robot, what it is
// doing, since when, whether Take control is offered and whether this
// server's Stop reaches it. The Qt side feeds it the maintenance poll's body
// and every 409 in_use_by_other_controller, and draws what this returns.
import {
  ControlTracker, type ControlNotice,
} from "../../../../offline-programming/v1/ui/src/execution/controlNotice.ts";

let tracker = new ControlTracker();

/** The maintenance route's body (its `control` member). */
export function controlObserve(body: unknown, nowMs: number): void {
  tracker.observe(body, nowMs);
}

/** A refusal's body; true when another controller was the reason. */
export function controlRefused(body: unknown, nowMs: number): boolean {
  return tracker.refused(body, nowMs);
}

/** POST .../control/take answered with this pendant's view. */
export function controlTook(body: unknown, nowMs: number): void {
  tracker.took(body, nowMs);
}

/** WAIT: the slim bar until the situation changes. */
export function controlWait(nowMs: number): void {
  tracker.wait(nowMs);
}

/** DETAILS: the full card again. */
export function controlExpand(): void {
  tracker.expand();
}

export function controlView(nowMs: number): ControlNotice {
  return tracker.notice(nowMs);
}

/** Tests start from a pendant that has seen nothing. */
export function controlForget(): void {
  tracker = new ControlTracker();
}
