import { CircleCheck, CircleX, TriangleAlert } from "lucide";

import { iconElement } from "./icons.ts";
import type { LogBus, LogEntry, LogLevel } from "./log.ts";
import { Widget, element } from "./widget.ts";

/**
 * The status bar's log indicator and the panel behind it.
 *
 * The indicator is one icon: a red cross while an unseen error stands, an
 * orange triangle while only warnings do, a green check otherwise, with
 * the count of what is unseen. Clicking it -- or the status bar -- opens
 * the panel: every entry, newest last, with time, level, module and
 * message, a filter per level, and Clear. Opening marks everything seen,
 * which is what returns the indicator to green.
 */
export class LogIndicator extends Widget<HTMLButtonElement> {
  readonly #errors: HTMLSpanElement;
  readonly #warnings: HTMLSpanElement;
  readonly #ok: HTMLSpanElement;

  constructor(private readonly bus: LogBus, onClick: () => void) {
    super(element("button", "log-indicator"));
    this.el.type = "button";
    this.el.title = "Open the log";
    // Errors and warnings each with their own count, so "2 errors and 5
    // warnings" is not read as seven of something.
    this.#errors = element("span", "log-indicator-part is-error");
    this.#warnings = element("span", "log-indicator-part is-warn");
    this.#ok = element("span", "log-indicator-part is-ok");
    this.#ok.append(iconElement(CircleCheck, 14));
    this.el.append(this.#errors, this.#warnings, this.#ok);
    this.listen(this.el, "click", (event) => {
      event.stopPropagation();
      onClick();
    });
    this.own(bus.changed.connect(() => this.sync()));
    this.sync();
  }

  sync(): void {
    const errors = this.bus.unseenCount("error");
    const warnings = this.bus.unseenCount("warn") - errors;
    this.#errors.replaceChildren(iconElement(CircleX, 14), element("span", "", String(errors)));
    this.#errors.hidden = errors === 0;
    this.#warnings.replaceChildren(iconElement(TriangleAlert, 14), element("span", "", String(warnings)));
    this.#warnings.hidden = warnings === 0;
    this.#ok.hidden = errors > 0 || warnings > 0;
    this.el.dataset.level = errors > 0 ? "error" : warnings > 0 ? "warn" : "ok";
    this.el.setAttribute("aria-label", errors > 0 || warnings > 0
      ? `${errors} unread error${errors === 1 ? "" : "s"}, ${warnings} unread warning${warnings === 1 ? "" : "s"}`
      : "No unread problems");
  }
}

const LEVELS: readonly LogLevel[] = ["debug", "info", "warn", "error"];

export class LogPanel extends Widget<HTMLElement> {
  readonly #rows: HTMLDivElement;
  readonly #filters = new Map<LogLevel, HTMLButtonElement>();
  readonly #enabled = new Set<LogLevel>(["info", "warn", "error"]);
  #release: (() => void) | null = null;

  constructor(private readonly bus: LogBus) {
    super(element("section", "log-panel"));
    this.el.setAttribute("role", "dialog");
    this.el.setAttribute("aria-label", "Logs");
    this.el.hidden = true;
    const header = element("div", "log-panel-header");
    header.append(element("h3", "", "Logs"));
    const filters = element("div", "log-panel-filters");
    for (const level of LEVELS) {
      // A checkbox in the button: on/off is the box, not only a tint.
      const button = element("button", "log-panel-filter");
      button.append(element("span", "", level), element("span", "log-panel-filter-box"));
      button.type = "button";
      button.dataset.level = level;
      button.setAttribute("aria-pressed", String(this.#enabled.has(level)));
      this.listen(button, "click", () => {
        if (this.#enabled.has(level)) this.#enabled.delete(level);
        else this.#enabled.add(level);
        button.setAttribute("aria-pressed", String(this.#enabled.has(level)));
        this.render();
      });
      filters.append(button);
      this.#filters.set(level, button);
    }
    const clear = element("button", "mini-button log-panel-clear", "CLEAR");
    clear.type = "button";
    this.listen(clear, "click", () => bus.clear());
    const close = element("button", "log-panel-close", "×");
    close.type = "button";
    close.setAttribute("aria-label", "Close");
    this.listen(close, "click", () => this.close());
    header.append(filters, clear, close);
    this.#rows = element("div", "log-panel-rows");
    this.#rows.setAttribute("role", "log");
    this.el.append(header, this.#rows);
    this.own(bus.changed.connect(() => {
      if (this.shown) this.render();
    }));
  }

  get shown(): boolean {
    return !this.el.hidden;
  }

  open(): void {
    if (this.shown) return;
    this.el.hidden = false;
    this.render();
    this.#rows.scrollTop = this.#rows.scrollHeight;
    this.bus.markSeen();
    const onPointerDown = (event: PointerEvent): void => {
      const target = event.target as Node | null;
      if (target && this.el.contains(target)) return;
      this.close();
    };
    const onKeyDown = (event: KeyboardEvent): void => {
      if (event.key === "Escape") this.close();
    };
    queueMicrotask(() => {
      if (!this.shown) return;
      document.addEventListener("pointerdown", onPointerDown, true);
      document.addEventListener("keydown", onKeyDown, true);
    });
    this.#release = () => {
      document.removeEventListener("pointerdown", onPointerDown, true);
      document.removeEventListener("keydown", onKeyDown, true);
    };
  }

  close(): void {
    if (!this.shown) return;
    this.el.hidden = true;
    this.#release?.();
    this.#release = null;
    // Whatever arrived while it was open has been seen too.
    this.bus.markSeen();
  }

  toggle(): void {
    if (this.shown) this.close();
    else this.open();
  }

  render(): void {
    const atBottom = this.#rows.scrollTop + this.#rows.clientHeight >= this.#rows.scrollHeight - 4;
    this.#rows.replaceChildren();
    const entries = this.bus.entries.filter((entry) => this.#enabled.has(entry.level));
    if (entries.length === 0) {
      this.#rows.append(element("p", "log-panel-empty", "Nothing logged."));
      return;
    }
    for (const entry of entries) this.#rows.append(renderRow(entry));
    if (atBottom) this.#rows.scrollTop = this.#rows.scrollHeight;
  }
}

function renderRow(entry: LogEntry): HTMLElement {
  const row = element("div", "log-row");
  row.dataset.level = entry.level;
  row.append(
    element("span", "log-row-time", formatTime(entry.timeMs)),
    element("span", "log-row-level", entry.level),
    element("span", "log-row-module", entry.module),
    element("span", "log-row-message", entry.message),
  );
  if (entry.detail) {
    const details = document.createElement("details");
    details.className = "log-row-detail";
    const summary = document.createElement("summary");
    summary.textContent = "detail";
    const pre = document.createElement("pre");
    pre.textContent = entry.detail;
    details.append(summary, pre);
    row.append(details);
  }
  return row;
}

export function formatTime(timeMs: number): string {
  const date = new Date(timeMs);
  const pad = (value: number, width = 2): string => String(value).padStart(width, "0");
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}.${pad(date.getMilliseconds(), 3)}`;
}
