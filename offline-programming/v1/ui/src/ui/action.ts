import type { IconNode } from "lucide";

import { Signal } from "./signal.ts";

/**
 * An `Action` is a command the operator can issue, with everything a view
 * needs to show it: label, icon, tooltip, shortcut, and whether it is
 * enabled or checked right now. Qt's `QAction`, and the reason to have it:
 * one command is shown in several places (a toolbar button, a menu item,
 * a keyboard shortcut) and its state is set ONCE, here, instead of in
 * every view. The alternative is what main.ts had -- each render pass
 * finding each button by id and poking `disabled`, `aria-pressed` and
 * `title` by hand, so a command's state lived in as many places as it had
 * buttons.
 *
 * Views subscribe to `changed` and re-read the action; they never keep
 * their own copy of its state. `trigger()` is the single entry point for
 * "the operator did it": it refuses when disabled, flips `checked` for a
 * checkable action (or selects it in its exclusive group), then emits
 * `triggered`. Code that owns the underlying state may instead set
 * `checked` directly to mirror it; that emits `changed` but not
 * `triggered`, so a view update never loops back into a command.
 */
export interface ActionOptions {
  id: string;
  label: string;
  icon?: IconNode | null;
  tooltip?: string;
  /** Display and binding form, e.g. "F", "Shift+E", "Ctrl+Z". */
  shortcut?: string | null;
  checkable?: boolean;
  checked?: boolean;
  enabled?: boolean;
  group?: ActionGroup;
}

export class Action {
  readonly id: string;
  readonly icon: IconNode | null;
  readonly checkable: boolean;
  readonly shortcut: string | null;
  /** Emitted after `trigger()` ran (the action was enabled). */
  readonly triggered = new Signal<Action>();
  /** Emitted after `checked` changed by any route, with the new value. */
  readonly toggled = new Signal<boolean>();
  /** Emitted after any displayed property changed. */
  readonly changed = new Signal<Action>();
  readonly group: ActionGroup | null;
  #label: string;
  #tooltip: string;
  #checked: boolean;
  #enabled: boolean;
  #disabledReason: string | null = null;

  constructor(options: ActionOptions) {
    this.id = options.id;
    this.#label = options.label;
    this.icon = options.icon ?? null;
    this.#tooltip = options.tooltip ?? options.label;
    this.shortcut = options.shortcut ?? null;
    this.checkable = options.checkable === true || options.group !== undefined;
    this.#checked = options.checked === true;
    this.#enabled = options.enabled !== false;
    this.group = options.group ?? null;
    this.group?.add(this);
  }

  get label(): string {
    return this.#label;
  }

  set label(value: string) {
    if (value === this.#label) return;
    this.#label = value;
    this.changed.emit(this);
  }

  get tooltip(): string {
    return this.#tooltip;
  }

  set tooltip(value: string) {
    if (value === this.#tooltip) return;
    this.#tooltip = value;
    this.changed.emit(this);
  }

  get checked(): boolean {
    return this.#checked;
  }

  /** Mirror an externally owned state. Emits `toggled` and `changed`, not
   *  `triggered`. */
  set checked(value: boolean) {
    if (!this.checkable || value === this.#checked) return;
    this.#checked = value;
    if (value && this.group) this.group.uncheckOthers(this);
    this.toggled.emit(value);
    this.changed.emit(this);
  }

  get enabled(): boolean {
    return this.#enabled;
  }

  set enabled(value: boolean) {
    this.setEnabled(value, null);
  }

  /** Why the action is disabled, for the view to show in place of the
   *  tooltip; null when enabled or when no reason was given. */
  get disabledReason(): string | null {
    return this.#enabled ? null : this.#disabledReason;
  }

  /** Disable with a reason the operator can read ("Locked while a robot is
   *  moving"), or enable. A reason with `enabled = true` is discarded. */
  setEnabled(enabled: boolean, reason: string | null = null): void {
    const nextReason = enabled ? null : reason;
    if (enabled === this.#enabled && nextReason === this.#disabledReason) return;
    this.#enabled = enabled;
    this.#disabledReason = nextReason;
    this.changed.emit(this);
  }

  /** The title a view shows: the tooltip, the shortcut in brackets, or the
   *  reason it is disabled. */
  get title(): string {
    const reason = this.disabledReason;
    if (reason) return reason;
    return this.shortcut ? `${this.#tooltip} [${this.shortcut}]` : this.#tooltip;
  }

  /** The operator issued the command. Refused while disabled. */
  trigger(): boolean {
    if (!this.#enabled) return false;
    if (this.group) {
      // Exclusive: re-triggering the checked one keeps it checked.
      if (!this.#checked) this.checked = true;
    } else if (this.checkable) {
      this.checked = !this.#checked;
    }
    this.triggered.emit(this);
    return true;
  }

  dispose(): void {
    this.triggered.clear();
    this.toggled.clear();
    this.changed.clear();
    this.group?.remove(this);
  }
}

/**
 * Exclusive actions: checking one unchecks the rest (Qt's `QActionGroup`).
 * The pose source (Sim / Real) is one: the viewer shows one pose at a time.
 */
export class ActionGroup {
  readonly #actions = new Set<Action>();
  /** Emitted with the newly checked action. */
  readonly selected = new Signal<Action>();

  get actions(): readonly Action[] {
    return [...this.#actions];
  }

  get checked(): Action | null {
    for (const action of this.#actions) if (action.checked) return action;
    return null;
  }

  add(action: Action): void {
    this.#actions.add(action);
    action.toggled.connect((checked) => {
      if (checked) this.selected.emit(action);
    });
  }

  remove(action: Action): void {
    this.#actions.delete(action);
  }

  uncheckOthers(keep: Action): void {
    for (const action of this.#actions) {
      if (action !== keep && action.checked) action.checked = false;
    }
  }
}
