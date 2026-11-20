import { Check, X } from "lucide";

import { iconElement } from "./icons.ts";
import { Widget, element } from "./widget.ts";

/**
 * A tool's dialog: the floating card a CAD feature shows while it is
 * active (Onshape's feature dialog) -- a title, ✓ to keep and close, ✕ to
 * put things back and close, an optional note, and sections of fields.
 *
 * The card knows nothing about what it edits. A tool composes it: adds
 * fields to sections, feeds them values on each render, and answers the
 * accept/cancel callbacks. What the card guarantees is the shape every
 * tool dialog shares, so Move part, a future Transform seam, and a jog
 * dialog all read the same way and dispose the same way.
 *
 * A section with a title is collapsible (a CAD feature dialog's "Grid
 * definition ▸"): its header carries a one-line summary so the collapsed
 * state still says what is inside, and whether it is open is remembered
 * per card in the browser, so a dialog reopens the way it was left.
 */
export interface ToolCardSectionOptions {
  /** A header makes the section collapsible; without one it is a plain
   *  block, as the Move part rows were. */
  title?: string;
  /** Right-aligned text in the header: what the section holds, in one
   *  line, for when it is collapsed. Updated with `setSectionSummary`. */
  summary?: string;
  /** Open on first use (default true). Remembered thereafter. */
  open?: boolean;
  /** Storage key; defaults to the title. */
  key?: string;
}

export interface ToolCardOptions {
  title: string;
  onAccept?: () => void;
  onCancel?: () => void;
  /** ✕ that only puts the card away -- for a tool with nothing to undo.
   *  Ignored when `onCancel` is given. */
  onClose?: () => void;
  acceptTitle?: string;
  cancelTitle?: string;
  /** Extra class for the card element (width, placement). */
  className?: string;
}

export class ToolCard extends Widget<HTMLElement> {
  readonly #title: HTMLHeadingElement;
  readonly #note: HTMLParagraphElement;
  readonly #sections: HTMLDivElement[] = [];
  readonly #footer: HTMLDivElement;
  readonly #summaries = new Map<HTMLDivElement, HTMLSpanElement>();
  readonly #details = new Map<HTMLDivElement, HTMLDetailsElement>();
  readonly #storageKey: string;
  #tools: HTMLDivElement | null = null;
  #accept: HTMLButtonElement | null = null;
  #acceptTitle = "";

  constructor(options: ToolCardOptions) {
    super(element("section", "tool-card"));
    this.#storageKey = `olp.tool-card.${options.title}`;
    if (options.className) this.el.classList.add(options.className);
    this.el.setAttribute("aria-label", options.title);
    this.el.hidden = true;

    const heading = element("div", "tool-card-heading");
    this.#title = element("h3", "", options.title);
    heading.append(this.#title);
    const buttons = element("div", "tool-card-buttons");
    if (options.onAccept) {
      const accept = element("button", "tool-card-accept");
      accept.type = "button";
      accept.title = options.acceptTitle ?? "Keep and close";
      accept.setAttribute("aria-label", "Accept");
      accept.append(iconElement(Check, 16));
      const onAccept = options.onAccept;
      this.listen(accept, "click", () => onAccept());
      buttons.append(accept);
      this.#accept = accept;
      this.#acceptTitle = accept.title;
    }
    if (options.onCancel) {
      const cancel = element("button", "tool-card-cancel");
      cancel.type = "button";
      cancel.title = options.cancelTitle ?? "Put back and close";
      cancel.setAttribute("aria-label", "Cancel");
      cancel.append(iconElement(X, 16));
      const onCancel = options.onCancel;
      this.listen(cancel, "click", () => onCancel());
      // Escape anywhere in the card is cancel, as in every CAD dialog.
      this.listen(this.el, "keydown", (event) => {
        if (event.key === "Escape") {
          event.preventDefault();
          onCancel();
        }
      });
      buttons.append(cancel);
    } else if (options.onClose) {
      const close = element("button", "tool-card-close");
      close.type = "button";
      close.title = "Close";
      close.setAttribute("aria-label", "Close");
      close.append(iconElement(X, 16));
      const onClose = options.onClose;
      this.listen(close, "click", () => onClose());
      this.listen(this.el, "keydown", (event) => {
        if (event.key === "Escape") {
          event.preventDefault();
          onClose();
        }
      });
      buttons.append(close);
    }
    heading.append(buttons);
    this.el.append(heading);

    this.#note = element("p", "tool-card-note");
    this.#note.hidden = true;
    this.el.append(this.#note);
    this.#footer = element("div", "tool-card-footer");
    this.#footer.hidden = true;
    this.el.append(this.#footer);
  }

  /** The row under every section, for a control that belongs to the card
   *  as a whole (a view slider) rather than to a section. */
  get footer(): HTMLDivElement {
    this.#footer.hidden = false;
    return this.#footer;
  }

  /** Refuse ✓ while `reason` stands, saying why on the button; null keeps
   *  it available again. ✕ is never refused. */
  setAcceptBlocked(reason: string | null): void {
    if (!this.#accept) return;
    this.#accept.disabled = reason !== null;
    this.#accept.title = reason ?? this.#acceptTitle;
  }

  setTitle(text: string): void {
    this.#title.textContent = text;
    this.el.setAttribute("aria-label", text);
  }

  /** A line under the title; empty hides it. */
  setNote(text: string): void {
    this.#note.textContent = text;
    this.#note.hidden = text === "";
  }

  /** A small button in the row under the note: a tool's own quick
   *  commands ("Centre", "Reset"), as against the ✓/✕ that close it. */
  addTool(label: string, onClick: () => void, title = ""): HTMLButtonElement {
    if (!this.#tools) {
      this.#tools = element("div", "tool-card-tools");
      this.#note.after(this.#tools);
    }
    const button = element("button", "mini-button tool-card-tool", label);
    button.type = "button";
    if (title) button.title = title;
    this.listen(button, "click", () => onClick());
    this.#tools.append(button);
    return button;
  }

  /** Start a new section; sections are separated by a rule. Returns the
   *  element to put fields in. */
  addSection(options: ToolCardSectionOptions = {}): HTMLDivElement {
    if (!options.title) {
      const section = element("div", "tool-card-section");
      this.#sections.push(section);
      this.el.insertBefore(section, this.#footer);
      return section;
    }
    const details = document.createElement("details");
    details.className = "tool-card-section is-collapsible";
    const key = `${this.#storageKey}.${options.key ?? options.title}`;
    details.open = readOpen(key) ?? options.open !== false;
    const header = document.createElement("summary");
    header.className = "tool-card-section-header";
    header.append(element("span", "tool-card-section-chevron", "▸"));
    header.append(element("span", "tool-card-section-title", options.title));
    const summary = element("span", "tool-card-section-summary", options.summary ?? "");
    header.append(summary);
    const body = element("div", "tool-card-section-body");
    details.append(header, body);
    this.listen(details, "toggle", () => writeOpen(key, details.open));
    this.#sections.push(body);
    this.#summaries.set(body, summary);
    this.#details.set(body, details);
    this.el.insertBefore(details, this.#footer);
    return body;
  }

  /** The one-line summary in a collapsible section's header. */
  setSectionSummary(section: HTMLDivElement, text: string): void {
    const summary = this.#summaries.get(section);
    if (summary) summary.textContent = text;
  }

  setSectionOpen(section: HTMLDivElement, open: boolean): void {
    const details = this.#details.get(section);
    if (details) details.open = open;
  }

  isSectionOpen(section: HTMLDivElement): boolean {
    return this.#details.get(section)?.open ?? true;
  }

  /** Add a widget to a section (the last one, or a new one if none). */
  add<W extends Widget>(widget: W, section: HTMLDivElement | null = null): W {
    const host = section ?? this.#sections.at(-1) ?? this.addSection();
    widget.mount(host);
    this.own(() => widget.dispose());
    return widget;
  }

  show(): void {
    this.el.hidden = false;
  }

  hide(): void {
    this.el.hidden = true;
  }

  get shown(): boolean {
    return !this.el.hidden;
  }
}

function readOpen(key: string): boolean | null {
  try {
    const stored = localStorage.getItem(key);
    return stored === null ? null : stored === "1";
  } catch {
    return null;
  }
}

function writeOpen(key: string, open: boolean): void {
  try {
    localStorage.setItem(key, open ? "1" : "0");
  } catch {
    // Private windows and blocked storage: the section still works, it
    // just opens the default way next time.
  }
}
