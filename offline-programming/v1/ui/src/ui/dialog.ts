import { X } from "lucide";

import { iconElement } from "./icons.ts";
import { Widget, element } from "./widget.ts";

/**
 * A modal popup: centred over the whole page on a dimmed backdrop, a
 * title bar with ✕, sections below, an optional footer. Escape, ✕ and a
 * click on the backdrop close it.
 *
 * Against the ToolCard: a tool card sits in the viewport's corner beside
 * the thing it edits and the viewport stays live under it; a dialog
 * takes the page for something that is not about the viewport at all
 * (the cell registry, a log). One at a time is the backdrop's doing.
 */
export interface DialogOptions {
  title: string;
  /** Extra class on the panel (width, layout). */
  className?: string;
  /** Called after the dialog closed, however it closed. */
  onClose?: () => void;
}

export interface DialogSectionOptions {
  title?: string;
  /** A collapsible section (a <details>); `open` is its initial state. */
  collapsible?: boolean;
  open?: boolean;
  /** Right-aligned text in the header, for a summary; set with
   *  `setSectionSummary`. */
  summary?: string;
}

export class Dialog extends Widget<HTMLDivElement> {
  readonly panel: HTMLElement;
  readonly #title: HTMLHeadingElement;
  readonly #body: HTMLDivElement;
  readonly #footer: HTMLDivElement;
  readonly #summaries = new Map<HTMLDivElement, HTMLSpanElement>();
  readonly #onClose: (() => void) | null;
  #release: (() => void) | null = null;

  constructor(options: DialogOptions) {
    super(element("div", "dialog-backdrop"));
    this.el.hidden = true;
    this.#onClose = options.onClose ?? null;
    this.panel = element("section", "dialog");
    if (options.className) this.panel.classList.add(options.className);
    this.panel.setAttribute("role", "dialog");
    this.panel.setAttribute("aria-modal", "true");
    this.panel.setAttribute("aria-label", options.title);
    this.panel.tabIndex = -1;
    const header = element("div", "dialog-header");
    this.#title = element("h3", "", options.title);
    const close = element("button", "dialog-close");
    close.type = "button";
    close.title = "Close";
    close.setAttribute("aria-label", "Close");
    close.append(iconElement(X, 16));
    this.listen(close, "click", () => this.close());
    header.append(this.#title, close);
    this.#body = element("div", "dialog-body");
    this.#footer = element("div", "dialog-footer");
    this.#footer.hidden = true;
    this.panel.append(header, this.#body, this.#footer);
    this.el.append(this.panel);
    // A click on the dimmed page, not on the panel, is a close.
    this.listen(this.el, "pointerdown", (event) => {
      if (event.target === this.el) this.close();
    });
  }

  get shown(): boolean {
    return !this.el.hidden;
  }

  get footer(): HTMLDivElement {
    this.#footer.hidden = false;
    return this.#footer;
  }

  setTitle(text: string): void {
    this.#title.textContent = text;
  }

  /** A section of the body; returns the element to put content in. */
  addSection(options: DialogSectionOptions = {}): HTMLDivElement {
    const content = element("div", "dialog-section-body");
    if (options.collapsible && options.title) {
      const details = document.createElement("details");
      details.className = "dialog-section is-collapsible";
      details.open = options.open !== false;
      const summary = document.createElement("summary");
      summary.className = "dialog-section-header";
      summary.append(
        element("span", "dialog-section-chevron", "▸"),
        element("span", "dialog-section-title", options.title),
      );
      const note = element("span", "dialog-section-summary", options.summary ?? "");
      summary.append(note);
      this.#summaries.set(content, note);
      details.append(summary, content);
      this.#body.append(details);
      return content;
    }
    const section = element("section", "dialog-section");
    if (options.title) {
      const header = element("div", "dialog-section-header");
      header.append(element("span", "dialog-section-title", options.title));
      const note = element("span", "dialog-section-summary", options.summary ?? "");
      header.append(note);
      this.#summaries.set(content, note);
      section.append(header);
    }
    section.append(content);
    this.#body.append(section);
    return content;
  }

  setSectionSummary(section: HTMLDivElement, text: string): void {
    const note = this.#summaries.get(section);
    if (note) note.textContent = text;
  }

  open(): void {
    if (this.shown) return;
    this.el.hidden = false;
    const onKeyDown = (event: KeyboardEvent): void => {
      if (event.key === "Escape") {
        event.preventDefault();
        this.close();
      }
    };
    document.addEventListener("keydown", onKeyDown, true);
    this.#release = () => document.removeEventListener("keydown", onKeyDown, true);
    this.panel.focus();
  }

  close(): void {
    if (!this.shown) return;
    this.el.hidden = true;
    this.#release?.();
    this.#release = null;
    this.#onClose?.();
  }

  toggle(): void {
    if (this.shown) this.close();
    else this.open();
  }

  override dispose(): void {
    this.#release?.();
    this.#release = null;
    super.dispose();
  }
}
