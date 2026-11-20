import type { Action } from "./action.ts";
import { iconElement } from "./icons.ts";
import { Widget, element } from "./widget.ts";

/**
 * Close on the next pointerdown outside (the anchor counts as inside, so
 * its own click can toggle), or on Escape. Registered after the current
 * event has finished dispatching so that event is not it. Returns a
 * function that drops the listeners.
 */
export function armDismiss(el: HTMLElement, anchor: HTMLElement | null, close: () => void): () => void {
  let armed = true;
  const onPointerDown = (event: PointerEvent): void => {
    const target = event.target as Node | null;
    if (target && (el.contains(target) || anchor?.contains(target))) return;
    close();
  };
  const onKeyDown = (event: KeyboardEvent): void => {
    if (event.key === "Escape") close();
  };
  queueMicrotask(() => {
    if (!armed) return;
    document.addEventListener("pointerdown", onPointerDown, true);
    document.addEventListener("keydown", onKeyDown, true);
  });
  return () => {
    armed = false;
    document.removeEventListener("pointerdown", onPointerDown, true);
    document.removeEventListener("keydown", onKeyDown, true);
  };
}

/**
 * A popup menu of actions, and a toolbar button that opens one.
 *
 * The menu is a plain positioned list, opened under its anchor, closed by
 * a click anywhere else, Escape, or choosing an item -- except an item
 * added with `keepOpen`, a setting the operator may want to change again
 * while reading its effect in the same menu. Items render the
 * same action state as toolbar buttons (checked mark, disabled with its
 * reason), so a command shown in both places never disagrees with itself.
 */
export class MenuItem extends Widget<HTMLButtonElement> {
  readonly action: Action;
  readonly #check: HTMLSpanElement;
  readonly #label: HTMLSpanElement;
  readonly #hint: HTMLSpanElement;

  constructor(action: Action, onChosen: () => void) {
    super(element("button", "menu-item"));
    this.action = action;
    this.el.type = "button";
    this.el.setAttribute("role", action.checkable ? "menuitemcheckbox" : "menuitem");
    this.el.dataset.action = action.id;
    this.#check = element("span", "menu-item-check");
    this.#label = element("span", "menu-item-label");
    this.#hint = element("span", "menu-item-hint");
    this.el.append(this.#check);
    if (action.icon) this.el.append(iconElement(action.icon, 16));
    this.el.append(this.#label, this.#hint);
    this.listen(this.el, "click", () => {
      if (this.action.trigger()) onChosen();
    });
    this.own(action.changed.connect(() => this.sync()));
    this.sync();
  }

  sync(): void {
    const action = this.action;
    this.#label.textContent = action.label;
    this.#hint.textContent = action.shortcut ?? "";
    this.el.title = action.disabledReason ?? "";
    this.el.disabled = !action.enabled;
    if (action.checkable) {
      this.el.setAttribute("aria-checked", String(action.checked));
      this.#check.textContent = action.checked ? "✓" : "";
    }
    this.el.classList.toggle("is-active", action.checkable && action.checked);
  }
}

export class Menu extends Widget<HTMLDivElement> {
  readonly items: MenuItem[] = [];
  #open = false;
  readonly #submenus: Menu[] = [];

  constructor(label = "Menu") {
    super(element("div", "menu"));
    this.el.setAttribute("role", "menu");
    this.el.setAttribute("aria-label", label);
    this.el.hidden = true;
    // The pointer on one of this menu's own items, other than a submenu's,
    // folds any open submenu away, as a desktop menu does.
    this.listen(this.el, "pointerover", (event) => {
      const item = (event.target as HTMLElement).closest(".menu-item");
      if (!item || item.parentElement !== this.el || item.classList.contains("menu-item-submenu")) return;
      this.#closeSubmenus(null);
    });
  }

  get open(): boolean {
    return this.#open;
  }

  addAction(action: Action, options: { keepOpen?: boolean } = {}): MenuItem {
    const onChosen = options.keepOpen ? () => {} : () => this.close();
    const item = new MenuItem(action, onChosen).mount(this.el);
    this.items.push(item);
    this.own(() => item.dispose());
    // One gutter for the whole menu: once any item can carry a check,
    // every item in this menu leaves room for one, so labels line up.
    if (action.checkable) this.el.classList.add("has-checks");
    return item;
  }

  addActions(actions: readonly Action[]): MenuItem[] {
    return actions.map((action) => this.addAction(action));
  }

  addSeparator(): void {
    const separator = element("div", "menu-separator");
    separator.setAttribute("role", "separator");
    this.el.append(separator);
  }

  /** A non-interactive heading over a run of items ("Open demo file"). */
  addHeading(text: string): void {
    const heading = element("div", "menu-heading", text);
    heading.setAttribute("role", "presentation");
    this.el.append(heading);
  }

  /** A line of muted text: a status the menu's commands act on. */
  addNote(text: string): HTMLDivElement {
    const note = element("div", "menu-note", text);
    note.setAttribute("role", "presentation");
    this.el.append(note);
    return note;
  }

  /** The caller's own markup, for a control no menu item can be: a field
   *  and the button that acts on what was typed into it. Pointer events
   *  inside the menu do not close it, so such a block is usable in place. */
  addBlock(content: HTMLElement): HTMLDivElement {
    const block = element("div", "menu-block");
    block.setAttribute("role", "presentation");
    block.append(content);
    this.el.append(block);
    return block;
  }

  /**
   * An item that opens a menu of its own beside it ("Theme ›"): a group of
   * settings kept out of this menu's list, so the list can grow without
   * crowding. It opens on hover, click, or → / Enter from the keyboard.
   * The submenu's element lives inside this menu's, so a click in it is a
   * click inside this menu: dismissal stays this menu's, and closing this
   * menu closes it. The submenu is the caller's and survives `clear()`.
   */
  addSubmenu(label: string, submenu: Menu): HTMLButtonElement {
    const item = element("button", "menu-item menu-item-submenu");
    item.type = "button";
    item.setAttribute("role", "menuitem");
    item.setAttribute("aria-haspopup", "menu");
    item.setAttribute("aria-expanded", "false");
    item.append(
      element("span", "menu-item-check"),
      element("span", "menu-item-label", label),
      element("span", "menu-item-chevron", "›"),
    );
    this.el.append(item, submenu.el);
    const open = (): void => {
      this.#closeSubmenus(submenu);
      submenu.openBeside(item);
      item.setAttribute("aria-expanded", "true");
    };
    item.addEventListener("pointerenter", open);
    item.addEventListener("click", open);
    item.addEventListener("keydown", (event) => {
      if (event.key !== "ArrowRight" && event.key !== "Enter") return;
      event.preventDefault();
      open();
      submenu.items[0]?.el.focus();
    });
    if (!this.#submenus.includes(submenu)) {
      this.#submenus.push(submenu);
      submenu.el.addEventListener("keydown", (event) => {
        if (event.key !== "ArrowLeft") return;
        submenu.close();
        (submenu.el.previousElementSibling as HTMLElement | null)?.focus();
      });
      submenu.closed.add(() => submenu.el.previousElementSibling?.setAttribute("aria-expanded", "false"));
    }
    return item;
  }

  #closeSubmenus(except: Menu | null): void {
    for (const submenu of this.#submenus) if (submenu !== except) submenu.close();
  }

  /** Empty the menu, for one whose items depend on state at open time (a
   *  recent-files list). Items are disposed; the actions are the caller's. */
  clear(): void {
    this.#closeSubmenus(null);
    for (const item of this.items) item.dispose();
    this.items.length = 0;
    this.el.replaceChildren();
    this.el.classList.remove("has-checks");
  }

  /** Called every time the menu is about to open; rebuild here. */
  beforeOpen: (() => void) | null = null;

  /** Show the menu at a point: a context menu at the pointer. Kept inside
   *  the viewport, flipping above or left of the point when it must. */
  openAt(x: number, y: number): void {
    if (this.#open) return;
    this.beforeOpen?.();
    this.#open = true;
    this.el.hidden = false;
    const width = this.el.offsetWidth;
    const height = this.el.offsetHeight;
    const viewportW = document.documentElement.clientWidth || Number.POSITIVE_INFINITY;
    const viewportH = document.documentElement.clientHeight || Number.POSITIVE_INFINITY;
    const left = x + width > viewportW - 8 ? Math.max(8, x - width) : x;
    const top = y + height > viewportH - 8 ? Math.max(8, y - height) : y;
    this.el.style.left = `${left}px`;
    this.el.style.top = `${top}px`;
    this.#armClose(null);
    this.items[0]?.el.focus();
  }

  /** Show the menu under an anchor element. "start" lines the menu's left
   *  edge up with the anchor's; "end" its right edge (an anchor in a pane's
   *  top-right corner, like the view menu under the axis gizmo, must open
   *  inwards). Either way it is kept inside the viewport. */
  openBelow(anchor: HTMLElement, align: MenuAlign = "start"): void {
    if (this.#open) return;
    this.beforeOpen?.();
    this.#open = true;
    this.el.hidden = false;
    const rect = anchor.getBoundingClientRect();
    const width = this.el.offsetWidth;
    const viewport = document.documentElement.clientWidth || Number.POSITIVE_INFINITY;
    const wanted = align === "end" ? rect.right - width : rect.left;
    const left = Math.max(8, Math.min(wanted, viewport - width - 8));
    this.el.style.left = `${left}px`;
    this.el.style.top = `${rect.bottom + 4}px`;
    this.#armClose(anchor);
    this.items[0]?.el.focus();
  }

  /** Show the menu as a submenu: beside its item, its first row level with
   *  the item's, flipping to the item's left when the right has no room.
   *  Its parent owns dismissal, so no listeners are armed here. */
  openBeside(item: HTMLElement): void {
    if (this.#open) return;
    this.beforeOpen?.();
    this.#open = true;
    this.el.hidden = false;
    const rect = item.getBoundingClientRect();
    const width = this.el.offsetWidth;
    const height = this.el.offsetHeight;
    const viewportW = document.documentElement.clientWidth || Number.POSITIVE_INFINITY;
    const viewportH = document.documentElement.clientHeight || Number.POSITIVE_INFINITY;
    // Up by the menu's 3 px padding and its hairline, so the rows line up.
    const top = Math.max(8, Math.min(rect.top - 4, viewportH - height - 8));
    const left = rect.right + 4 + width > viewportW - 8 ? Math.max(8, rect.left - width - 4) : rect.right + 4;
    this.el.style.left = `${left}px`;
    this.el.style.top = `${top}px`;
  }

  /** Close on the next pointerdown outside (the anchor counts as inside, so
   *  its own click can toggle), or on Escape. Registered after the opening
   *  event has finished dispatching so that event is not it. */
  #armClose(anchor: HTMLElement | null): void {
    this.#release = armDismiss(this.el, anchor, () => this.close());
  }

  #release: (() => void) | null = null;

  close(): void {
    if (!this.#open) return;
    this.#closeSubmenus(null);
    this.#open = false;
    this.el.hidden = true;
    this.#release?.();
    this.#release = null;
    this.closed.forEach((slot) => slot());
  }

  /** Called after the menu closed, for the anchor to drop its pressed look. */
  readonly closed = new Set<() => void>();

  override dispose(): void {
    this.close();
    super.dispose();
  }
}

export type MenuAlign = "start" | "end";

export interface MenuButtonOptions {
  label: string;
  /** Which edge of the button the menu lines up with (default "start"). */
  align?: MenuAlign;
  icon?: Parameters<typeof iconElement>[0] | null;
  /** Show the label text (default true); false keeps it as the accessible
   *  name only, for a gear in a corner. */
  showLabel?: boolean;
  /** Show the ▾ caret (default true). */
  caret?: boolean;
  iconSize?: number;
  tooltip?: string;
}

/** A toolbar button that opens a menu instead of triggering an action. */
export class MenuButton extends Widget<HTMLButtonElement> {
  readonly menu: Menu;

  constructor(options: MenuButtonOptions, menu: Menu) {
    super(element("button", "tool-button menu-button"));
    this.menu = menu;
    this.el.type = "button";
    this.el.setAttribute("aria-haspopup", "menu");
    this.el.setAttribute("aria-expanded", "false");
    this.el.setAttribute("aria-label", options.label);
    this.el.title = options.tooltip ?? options.label;
    if (options.icon) {
      this.el.append(iconElement(options.icon, options.iconSize ?? 18));
      this.el.classList.add("has-icon");
    }
    if (options.showLabel !== false) {
      this.el.append(element("span", "tool-button-label", options.label));
    } else {
      this.el.classList.add("icon-only");
    }
    if (options.caret !== false) this.el.append(element("span", "menu-button-caret", "▾"));
    this.listen(this.el, "click", () => {
      if (menu.open) {
        menu.close();
      } else {
        menu.openBelow(this.el, options.align ?? "start");
        this.el.setAttribute("aria-expanded", "true");
      }
    });
    const onClosed = (): void => this.el.setAttribute("aria-expanded", "false");
    menu.closed.add(onClosed);
    this.own(() => menu.closed.delete(onClosed));
  }
}
