/**
 * "Run program refused": a press that started nothing says why where the
 * operator is looking, in a toast, not only in the line under the button.
 *
 * 2026-10-12 00:22 PDT, a Steam Deck on a Rosie 1430 arm: the owner chose Run 5 times,
 * pressed RUN and confirmed the question five times; the OLP server refused
 * every confirmed request (`client_not_focused`) and the only trace on the
 * Deck was the Run card's result line. "Selecting Run 5 loops did nothing."
 *
 * One notice at a time, the newest press's. It is drawn by the same toast as
 * "Jog stopped" (the browser's `ui/jog-end-toast.ts`, the Deck's
 * JogEndOverlay), so it has that toast's shape: kicker, the server's
 * sentence, details. It takes no focus and blocks nothing: STOP and the
 * controller keep working under it.
 *
 * Bundled for Qt 5.15's engine (ES2016) by the Deck's olp-core.
 */
import type { JogEndNotice } from "./jogEnd.ts";

/** A refusal stays up this long unless dismissed; Details holds it open.
 *  Longer than Jog stopped (6 s): the operator may be watching the robot,
 *  not the screen, when the answer comes. Unverified. */
export const PRESS_NOTICE_MS = 15000;

/** What a refused press says: the kicker names the press, `text` is the
 *  server's sentence, `details` the code and step. */
export interface PressRefusal {
  kicker: string;
  text: string;
  details: string[];
  code: string;
}

const INACTIVE: JogEndNotice = {
  active: false, tone: "toast", seq: 0, code: "", kicker: "", text: "", details: [], expanded: false, remainingMs: 0,
};

export class PressNoticeTracker {
  #shown: { refusal: PressRefusal; seq: number; untilMs: number } | null = null;
  #seq = 0;
  #expanded = false;

  /** A press was refused or ignored: show it, replacing any older notice. */
  show(refusal: PressRefusal, nowMs: number): void {
    this.#shown = { refusal, seq: ++this.#seq, untilMs: nowMs + PRESS_NOTICE_MS };
    this.#expanded = false;
  }

  /** The × button, and a later press that went through. */
  dismiss(): void {
    this.#shown = null;
    this.#expanded = false;
  }

  /** Details: open holds the notice; closing it restarts the timer. */
  toggleDetails(nowMs: number): void {
    if (!this.#shown) return;
    this.#expanded = !this.#expanded;
    if (!this.#expanded) this.#shown.untilMs = nowMs + PRESS_NOTICE_MS;
  }

  notice(nowMs: number): JogEndNotice {
    const shown = this.#shown;
    if (!shown) return INACTIVE;
    if (!this.#expanded && nowMs >= shown.untilMs) {
      this.#shown = null;
      return INACTIVE;
    }
    const { refusal } = shown;
    return {
      active: true, tone: "toast", seq: shown.seq, code: refusal.code, kicker: refusal.kicker, text: refusal.text,
      details: refusal.details, expanded: this.#expanded,
      remainingMs: this.#expanded ? PRESS_NOTICE_MS : Math.max(0, shown.untilMs - nowMs),
    };
  }
}
