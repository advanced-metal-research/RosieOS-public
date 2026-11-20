import type { Action } from "./action.ts";

/**
 * Keyboard shortcuts for actions. An action declares its shortcut once
 * ("F", "Shift+E", "Ctrl+Z"); binding it here makes the key trigger the
 * action, and the toolbar shows the same string in its tooltip, so what
 * the operator reads is what works.
 *
 * Keys typed into a field are never shortcuts: the spreadsheet and the
 * placement inputs take letters and digits, and "M" while editing an
 * offset must not start moving the part.
 */
export interface ShortcutChord {
  key: string;
  ctrl: boolean;
  shift: boolean;
  alt: boolean;
}

export function parseShortcut(shortcut: string): ShortcutChord {
  const parts = shortcut.split("+").map((part) => part.trim()).filter(Boolean);
  const chord: ShortcutChord = { key: "", ctrl: false, shift: false, alt: false };
  for (const part of parts) {
    const lower = part.toLowerCase();
    if (lower === "ctrl" || lower === "control" || lower === "cmd" || lower === "meta") chord.ctrl = true;
    else if (lower === "shift") chord.shift = true;
    else if (lower === "alt" || lower === "option") chord.alt = true;
    else chord.key = normaliseKey(part);
  }
  return chord;
}

export function chordOfEvent(event: KeyboardEvent): ShortcutChord {
  return {
    key: normaliseKey(event.key),
    ctrl: event.ctrlKey || event.metaKey,
    shift: event.shiftKey,
    alt: event.altKey,
  };
}

export function sameChord(a: ShortcutChord, b: ShortcutChord): boolean {
  return a.key === b.key && a.ctrl === b.ctrl && a.shift === b.shift && a.alt === b.alt;
}

function normaliseKey(key: string): string {
  const named: Record<string, string> = { esc: "escape", del: "delete", space: " " };
  const lower = key.toLowerCase();
  return named[lower] ?? lower;
}

/** True when the key press belongs to a text field, not to the app. */
export function isTypingTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  if (target.isContentEditable) return true;
  const tag = target.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
}

export class Shortcuts {
  readonly #bindings = new Map<Action, ShortcutChord>();
  readonly #target: Window | Document | HTMLElement;
  readonly #onKeyDown = (event: KeyboardEvent): void => {
    if (event.defaultPrevented || event.repeat) return;
    if (isTypingTarget(event.target)) return;
    const chord = chordOfEvent(event);
    for (const [action, bound] of this.#bindings) {
      if (!sameChord(chord, bound)) continue;
      if (action.trigger()) event.preventDefault();
      return;
    }
  };

  constructor(target: Window | Document | HTMLElement) {
    this.#target = target;
    target.addEventListener("keydown", this.#onKeyDown as EventListener);
  }

  /** Bind an action's declared shortcut; an action without one is skipped. */
  bind(...actions: readonly Action[]): void {
    for (const action of actions) {
      if (!action.shortcut) continue;
      this.#bindings.set(action, parseShortcut(action.shortcut));
    }
  }

  unbind(action: Action): void {
    this.#bindings.delete(action);
  }

  /** The action a chord would trigger, for tests and for conflict checks. */
  lookup(chord: ShortcutChord): Action | null {
    for (const [action, bound] of this.#bindings) {
      if (sameChord(chord, bound)) return action;
    }
    return null;
  }

  dispose(): void {
    this.#target.removeEventListener("keydown", this.#onKeyDown as EventListener);
    this.#bindings.clear();
  }
}
