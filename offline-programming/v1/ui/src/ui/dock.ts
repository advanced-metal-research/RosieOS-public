import { Widget, element } from "./widget.ts";

/**
 * A side dock: a strip of vertical tabs on the window's edge that stays
 * folded until a tab is clicked, then snaps open to show that tab's pane.
 * Clicking the open tab folds it again. Which tab was open is remembered
 * per browser, so the dock comes back the way it was left.
 *
 * The dock frames panes and nothing more: the app moves its own elements
 * in (a transport, a deployment panel) and their handlers come with them.
 */
export interface DockTab {
  id: string;
  label: string;
}

export interface SideDockOptions {
  tabs: readonly DockTab[];
  /** Storage key for the open tab; omit to forget on reload. */
  storageKey?: string;
  onChange?: (openTab: string | null) => void;
}

export class SideDock extends Widget<HTMLElement> {
  readonly panes = new Map<string, HTMLDivElement>();
  readonly #buttons = new Map<string, HTMLButtonElement>();
  readonly #storageKey: string | null;
  readonly #onChange: ((openTab: string | null) => void) | null;
  #open: string | null = null;

  constructor(options: SideDockOptions) {
    super(element("aside", "side-dock"));
    this.#storageKey = options.storageKey ?? null;
    this.#onChange = options.onChange ?? null;
    const strip = element("div", "side-dock-strip");
    strip.setAttribute("role", "tablist");
    strip.setAttribute("aria-orientation", "vertical");
    const body = element("div", "side-dock-body");
    for (const tab of options.tabs) {
      const button = element("button", "side-dock-tab", tab.label);
      button.type = "button";
      button.setAttribute("role", "tab");
      button.setAttribute("aria-selected", "false");
      button.dataset.tab = tab.id;
      this.listen(button, "click", () => this.toggle(tab.id));
      strip.append(button);
      this.#buttons.set(tab.id, button);
      const pane = element("div", "side-dock-pane");
      pane.setAttribute("role", "tabpanel");
      pane.dataset.tab = tab.id;
      pane.hidden = true;
      body.append(pane);
      this.panes.set(tab.id, pane);
    }
    this.el.append(strip, body);
    this.#apply(this.#restore());
  }

  get openTab(): string | null {
    return this.#open;
  }

  open(id: string): void {
    if (!this.panes.has(id)) return;
    this.#apply(id);
    this.#persist();
    this.#onChange?.(this.#open);
  }

  collapse(): void {
    this.#apply(null);
    this.#persist();
    this.#onChange?.(this.#open);
  }

  toggle(id: string): void {
    if (this.#open === id) this.collapse();
    else this.open(id);
  }

  #apply(id: string | null): void {
    this.#open = id;
    this.el.classList.toggle("is-open", id !== null);
    for (const [tab, button] of this.#buttons) {
      const active = tab === id;
      button.setAttribute("aria-selected", String(active));
      button.classList.toggle("is-active", active);
    }
    for (const [tab, pane] of this.panes) pane.hidden = tab !== id;
  }

  #restore(): string | null {
    if (!this.#storageKey) return null;
    try {
      const stored = localStorage.getItem(this.#storageKey);
      return stored && this.panes.has(stored) ? stored : null;
    } catch {
      return null;
    }
  }

  #persist(): void {
    if (!this.#storageKey) return;
    try {
      if (this.#open) localStorage.setItem(this.#storageKey, this.#open);
      else localStorage.removeItem(this.#storageKey);
    } catch {
      // Storage blocked: the dock forgets on reload.
    }
  }
}
