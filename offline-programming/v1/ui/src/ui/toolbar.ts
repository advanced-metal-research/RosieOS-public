import type { Action } from "./action.ts";
import { iconElement } from "./icons.ts";
import { Widget, element } from "./widget.ts";

/**
 * A CAD-style toolbar: groups of icon buttons with a short label under
 * each, separators between groups, every button driven by an `Action`.
 *
 * A `ToolButton` owns no state. It reads its action on every `changed`
 * and shows: `disabled`, `aria-pressed` (checkable only), `is-active`, and
 * the action's `title` -- which is the reason for being disabled when
 * there is one, so the operator learns why a button is grey by hovering
 * it, as the old pose-source buttons did by hand.
 */
export interface ToolButtonOptions {
  /** Show the label under the icon (default true). An icon-only button
   *  still has the label as its accessible name. */
  showLabel?: boolean;
  iconSize?: number;
}

export class ToolButton extends Widget<HTMLButtonElement> {
  readonly action: Action;
  #labelEl: HTMLSpanElement | null = null;

  constructor(action: Action, options: ToolButtonOptions = {}) {
    super(element("button", "tool-button"));
    this.action = action;
    this.el.type = "button";
    this.el.dataset.action = action.id;
    if (action.icon) {
      this.el.append(iconElement(action.icon, options.iconSize ?? 18));
      this.el.classList.add("has-icon");
    }
    if (options.showLabel !== false) {
      this.#labelEl = element("span", "tool-button-label");
      this.el.append(this.#labelEl);
    } else {
      this.el.classList.add("icon-only");
    }
    this.listen(this.el, "click", () => {
      this.action.trigger();
    });
    this.own(action.changed.connect(() => this.sync()));
    this.sync();
  }

  sync(): void {
    const action = this.action;
    const label = action.label;
    if (this.#labelEl) this.#labelEl.textContent = label;
    this.el.setAttribute("aria-label", label);
    this.el.title = action.title;
    this.el.disabled = !action.enabled;
    if (action.checkable) {
      this.el.setAttribute("aria-pressed", String(action.checked));
    } else {
      this.el.removeAttribute("aria-pressed");
    }
    this.el.classList.toggle("is-active", action.checkable && action.checked);
  }
}

export class ToolSeparator extends Widget<HTMLSpanElement> {
  constructor() {
    super(element("span", "tool-separator"));
    this.el.setAttribute("role", "separator");
    this.el.setAttribute("aria-orientation", "vertical");
  }
}

/** A named cluster of buttons: "View", "Part", "Cameras". */
export class ToolGroup extends Widget<HTMLDivElement> {
  readonly items: Widget[] = [];
  readonly #body: HTMLDivElement;

  constructor(label: string | null = null) {
    super(element("div", "tool-group"));
    this.el.setAttribute("role", "group");
    this.#body = element("div", "tool-group-body");
    this.el.append(this.#body);
    if (label) {
      this.el.setAttribute("aria-label", label);
      this.el.append(element("span", "tool-group-label", label));
    }
  }

  add<W extends Widget>(widget: W): W {
    this.items.push(widget);
    widget.mount(this.#body);
    this.own(() => widget.dispose());
    return widget;
  }

  addAction(action: Action, options?: ToolButtonOptions): ToolButton {
    return this.add(new ToolButton(action, options));
  }

  addActions(actions: readonly Action[], options?: ToolButtonOptions): ToolButton[] {
    return actions.map((action) => this.addAction(action, options));
  }
}

export class ToolBar extends Widget<HTMLDivElement> {
  readonly groups: ToolGroup[] = [];

  constructor(label = "Toolbar") {
    super(element("div", "toolbar"));
    this.el.setAttribute("role", "toolbar");
    this.el.setAttribute("aria-label", label);
  }

  /** Start a group; a separator goes before every group but the first. */
  addGroup(label: string | null = null): ToolGroup {
    if (this.groups.length > 0) {
      const separator = new ToolSeparator().mount(this.el);
      this.own(() => separator.dispose());
    }
    const group = new ToolGroup(label).mount(this.el);
    this.groups.push(group);
    this.own(() => group.dispose());
    return group;
  }

  /** Every button on the bar, for tests and for the shortcut hint pass. */
  buttons(): ToolButton[] {
    return this.groups.flatMap((group) =>
      group.items.filter((item): item is ToolButton => item instanceof ToolButton));
  }

  /** A slot for something that is not a button (a status pill, a select). */
  addWidget<W extends Widget>(widget: W): W {
    widget.mount(this.el);
    this.own(() => widget.dispose());
    return widget;
  }
}
