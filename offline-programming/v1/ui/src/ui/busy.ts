/**
 * One busy indicator for every wait in the app: a hairline trace with an
 * accent segment sweeping it, drawn by `.is-busy::before` (style.css) before
 * the element's own words. Static "…" text left the operator unsure whether
 * anything was happening; the trace moves while the work does.
 *
 * The app's convention already is that a status ending in an ellipsis is a
 * wait in progress, so the status setters mark busy from the text and the
 * words stay the single source of truth.
 */

/** Whether a status line describes work in progress: it ends in an ellipsis. */
export function isBusyText(text: string): boolean {
  return text.trimEnd().endsWith("…");
}

/** Show or hide the trace on `el`, and say so to assistive technology. */
export function markBusy(el: Element, busy: boolean): void {
  // Optional calls: the unit tests hand in bare stubs without a classList.
  el.classList?.toggle("is-busy", busy);
  if (busy) el.setAttribute?.("aria-busy", "true");
  else el.removeAttribute?.("aria-busy");
}

/** Set a status line's words and its trace in one step. */
export function setBusyText(el: HTMLElement, text: string): void {
  el.textContent = text;
  markBusy(el, isBusyText(text));
}
