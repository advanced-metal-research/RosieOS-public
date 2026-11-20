// "Run program refused" on the pendant, decided by OLP's own tracker (the one
// the browser's toast uses, execution/pressNotice.ts): a Run program, loop or
// Finish cycle press that started nothing says why in the Jog stopped
// overlay's place. The Qt side shows what runAnswer / finishAnswer return as
// `refusal` and draws pressNoticeView.
import { PressNoticeTracker, type PressRefusal } from "../../../../offline-programming/v1/ui/src/execution/pressNotice.ts";
import type { JogEndNotice } from "../../../../offline-programming/v1/ui/src/execution/jogEnd.ts";
import { finishRefusal, repeatOf, runNotSent } from "../../../../offline-programming/v1/ui/src/execution/runProgram.ts";
import { runRepeat } from "./run.ts";

let tracker = new PressNoticeTracker();

/** A refused or unsent press: replaces any notice on screen. */
export function pressNoticeShow(refusal: PressRefusal, nowMs: number): void {
  tracker.show(refusal, nowMs);
}

/** The ×, and a later press that went through. */
export function pressNoticeDismiss(): void {
  tracker.dismiss();
}

export function pressNoticeToggleDetails(nowMs: number): void {
  tracker.toggleDetails(nowMs);
}

export function pressNoticeView(nowMs: number): JogEndNotice {
  return tracker.notice(nowMs);
}

/** A Run press this pendant did not send, and why. */
export function runNotSentRefusal(why: string): PressRefusal {
  const repeat = runRepeat();
  return runNotSent(why, repeatOf(repeat.mode, repeat.count));
}

/** A refused Finish cycle (its 409's code and sentence). */
export function finishRefusalOf(code: string, detail: string): PressRefusal {
  return finishRefusal(code, detail);
}

/** Tests start from a pendant that has shown nothing. */
export function pressNoticeForget(): void {
  tracker = new PressNoticeTracker();
}
