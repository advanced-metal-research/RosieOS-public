import { Widget, element } from "./widget.ts";

/**
 * A popup for a long job: what it is doing, which stage of how many, a
 * bar, and how long it has been. Planning takes seconds to a minute and
 * a greyed button says nothing about whether anything is happening.
 *
 * Stages are sequential and counted ("Planning on the weld planner
 * (2/4)"); the bar is that completed fraction (2/4 is half) and steps
 * when a stage is reported. The sweep gradient runs only before any
 * stage has produced a fraction; it never sits over a known fill.
 * `finish` shows the result and closes after a moment; `fail` keeps the
 * fill it reached, shows the reason as a sentence, and waits to be closed.
 */
export interface ProgressStage {
  /** 1-based current stage; the bar fills `index / count`. */
  index: number;
  count: number;
  label: string;
  /**
   * Finer progress inside the current stage (seams planned of seams
   * total). When both are set, the fill is
   * `(index - 1 + done / total) / count` instead of `index / count`.
   */
  done?: number;
  total?: number;
}

/** A planner refusal as the motion server states it. */
export interface PlanRefusal {
  reason: string;
  detail: string;
  segment_index: number | null;
}

export interface ProgressFailureOptions {
  /** Keep diagnostics available without making them the primary explanation. */
  details?: string;
  action?: { label: string; run: () => void };
}

/**
 * A refused plan as a sentence: the planner's `detail`, then the reason
 * code, then the segment when the refusal names one. The JSON object
 * must never be the dialog text — that is what the operator saw.
 */
export function planRefusalText(reason: string | PlanRefusal): string {
  const refusal = typeof reason === "string" ? parsePlanRefusal(reason) : reason;
  if (!refusal) return typeof reason === "string" ? reason : "";
  const parts: string[] = [];
  const detail = refusal.detail.trim();
  const code = refusal.reason.trim();
  if (detail) parts.push(detail);
  if (code) parts.push(code);
  if (refusal.segment_index != null) parts.push(`segment ${refusal.segment_index}`);
  return parts.join(" · ");
}

/** Pull a `{reason, detail, segment_index}` object out of an error or a
 *  string that still has the JSON glued on. */
export function planRefusalFrom(error: unknown): string | PlanRefusal {
  if (error && typeof error === "object" && "dense_error" in error) {
    const parsed = asPlanRefusal((error as { dense_error?: unknown }).dense_error);
    if (parsed) return parsed;
  }
  const text = error instanceof Error ? error.message : String(error);
  return parsePlanRefusal(text) ?? text;
}

export function parsePlanRefusal(source: string): PlanRefusal | null {
  const start = source.indexOf("{");
  if (start < 0) return null;
  try {
    return asPlanRefusal(JSON.parse(source.slice(start)));
  } catch {
    return null;
  }
}

function asPlanRefusal(value: unknown): PlanRefusal | null {
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  const record = value as Record<string, unknown>;
  if (typeof record.reason !== "string") return null;
  return {
    reason: record.reason,
    detail: typeof record.detail === "string" ? record.detail : "",
    segment_index: typeof record.segment_index === "number" ? record.segment_index : null,
  };
}

export class ProgressDialog extends Widget<HTMLElement> {
  readonly #title: HTMLHeadingElement;
  readonly #stage: HTMLParagraphElement;
  readonly #bar: HTMLDivElement;
  readonly #fill: HTMLDivElement;
  readonly #elapsed: HTMLSpanElement;
  readonly #note: HTMLParagraphElement;
  readonly #close: HTMLButtonElement;
  readonly #cancel: HTMLButtonElement;
  #cancelAction: (() => void) | null = null;
  readonly #action: HTMLButtonElement;
  readonly #details: HTMLDetailsElement;
  readonly #detailsText: HTMLParagraphElement;
  #failureAction: (() => void) | null = null;
  #startedMs = 0;
  #timer: number | null = null;
  #closeTimer: number | null = null;
  #currentFill = 0;

  constructor() {
    super(element("section", "progress-dialog"));
    this.el.setAttribute("role", "dialog");
    this.el.setAttribute("aria-live", "polite");
    this.el.hidden = true;
    const header = element("div", "progress-dialog-header");
    this.#title = element("h3", "", "Working…");
    this.#elapsed = element("span", "progress-dialog-elapsed", "0.0 s");
    header.append(this.#title, this.#elapsed);
    this.#stage = element("p", "progress-dialog-stage", "");
    this.#bar = element("div", "progress-dialog-bar");
    this.#bar.setAttribute("role", "progressbar");
    this.#bar.setAttribute("aria-valuemin", "0");
    this.#bar.setAttribute("aria-valuemax", "100");
    this.#bar.setAttribute("aria-valuenow", "0");
    this.#fill = element("div", "progress-dialog-fill");
    this.#bar.append(this.#fill);
    this.#note = element("p", "progress-dialog-note", "");
    this.#note.hidden = true;
    this.#close = element("button", "mini-button progress-dialog-close", "CLOSE");
    this.#close.type = "button";
    this.#close.hidden = true;
    this.listen(this.#close, "click", () => this.hide());
    this.#cancel = element("button", "mini-button progress-dialog-cancel", "Cancel");
    this.#cancel.type = "button";
    this.#cancel.hidden = true;
    this.listen(this.#cancel, "click", () => {
      const cancel = this.#cancelAction;
      if (!cancel) return;
      this.#cancelAction = null;
      this.#cancel.disabled = true;
      this.#cancel.textContent = "Cancelling…";
      this.#stage.textContent = "Cancelling planning…";
      cancel();
    });
    this.#action = element("button", "mini-button progress-dialog-action");
    this.#action.type = "button";
    this.#action.hidden = true;
    this.listen(this.#action, "click", () => {
      const run = this.#failureAction;
      if (!run) return;
      this.hide();
      run();
    });
    this.#details = element("details", "progress-dialog-details");
    this.#details.hidden = true;
    this.#detailsText = element("p", "");
    this.#details.append(element("summary", "", "Technical details"), this.#detailsText);
    this.el.append(header, this.#stage, this.#bar, this.#note, this.#details, this.#action, this.#cancel, this.#close);
  }

  get shown(): boolean {
    return !this.el.hidden;
  }

  /** Open for a new job. */
  start(title: string, onCancel?: () => void): void {
    if (this.#closeTimer !== null) window.clearTimeout(this.#closeTimer);
    this.#closeTimer = null;
    this.#title.textContent = title;
    this.#stage.textContent = "";
    this.#note.hidden = true;
    this.#close.hidden = true;
    this.#clearFailure();
    this.setCancelAction(onCancel ?? null);
    this.el.classList.remove("is-done", "is-failed");
    this.el.classList.add("is-running");
    this.#setFill(0, true);
    this.#startedMs = performance.now();
    this.#tick();
    if (this.#timer === null) this.#timer = window.setInterval(() => this.#tick(), 100);
    this.el.hidden = false;
  }

  /** Cancellation ends before committing the completed result. */
  setCancelAction(action: (() => void) | null): void {
    this.#cancelAction = action;
    this.#cancel.hidden = action === null;
    this.#cancel.disabled = false;
    this.#cancel.textContent = "Cancel";
  }

  cancelled(): void {
    this.#stop();
    this.#clearFailure();
    this.el.classList.remove("is-running", "is-failed", "is-done");
    this.#setFill(this.#currentFill, false);
    this.#stage.textContent = "Cancelled";
    this.#note.textContent = "No new plan was saved. You can plan again when ready.";
    this.#note.hidden = false;
    this.#close.hidden = false;
  }

  /** Now on this stage of that many. The fill is the completed fraction. */
  stage(stage: ProgressStage): void {
    this.#stage.textContent = `${stage.label} (${stage.index}/${stage.count})`;
    this.#setFill(stageFraction(stage), true);
  }

  /** Done: show the outcome, then close on its own. */
  finish(note: string, closeAfterMs = 900): void {
    this.#stop();
    this.#clearFailure();
    this.el.classList.remove("is-running");
    this.el.classList.add("is-done");
    this.#setFill(1, false);
    this.#stage.textContent = "Done";
    this.#note.textContent = note;
    this.#note.hidden = note === "";
    this.#closeTimer = window.setTimeout(() => this.hide(), closeAfterMs);
  }

  /** Failed: keep the fill it reached, show why, wait to be closed. */
  fail(reason: string | PlanRefusal, options: ProgressFailureOptions = {}): void {
    this.#stop();
    this.#clearFailure();
    if (options.action) {
      this.#failureAction = options.action.run;
      this.#action.textContent = options.action.label;
      this.#action.hidden = false;
    }
    if (options.details) {
      this.#detailsText.textContent = options.details;
      this.#details.hidden = false;
    }
    this.el.classList.remove("is-running");
    this.el.classList.add("is-failed");
    this.#setFill(this.#currentFill, false);
    this.#stage.textContent = "Failed";
    this.#note.textContent = planRefusalText(reason);
    this.#note.hidden = false;
    this.#close.hidden = false;
  }

  hide(): void {
    this.#stop();
    if (this.#closeTimer !== null) window.clearTimeout(this.#closeTimer);
    this.#closeTimer = null;
    this.el.hidden = true;
    this.#clearFailure();
  }

  #clearFailure(): void {
    this.#failureAction = null;
    this.#action.hidden = true;
    this.#details.hidden = true;
    this.#details.open = false;
    this.#detailsText.textContent = "";
  }

  #setFill(fraction: number, running: boolean): void {
    const clamped = Math.min(1, Math.max(0, fraction));
    this.#currentFill = clamped;
    const percent = Math.round(clamped * 100);
    this.#fill.style.width = `${percent}%`;
    this.#bar.setAttribute("aria-valuenow", String(percent));
    this.#bar.setAttribute("aria-valuemax", "100");
    // A known fraction never sits under the sweep; the gradient is only
    // while the first stage has produced nothing yet.
    this.#bar.classList.toggle("is-indeterminate", running && percent === 0);
  }

  #tick(): void {
    const seconds = (performance.now() - this.#startedMs) / 1000;
    this.#elapsed.textContent = `${seconds.toFixed(1)} s`;
  }

  #stop(): void {
    this.setCancelAction(null);
    if (this.#timer !== null) window.clearInterval(this.#timer);
    this.#timer = null;
    this.#tick();
  }

  override dispose(): void {
    this.hide();
    super.dispose();
  }
}

function stageFraction(stage: ProgressStage): number {
  const count = Math.max(1, stage.count);
  const index = Math.min(count, Math.max(0, stage.index));
  if (stage.total != null && stage.total > 0 && stage.done != null) {
    const intra = Math.min(1, Math.max(0, stage.done / stage.total));
    return (index - 1 + intra) / count;
  }
  return index / count;
}
