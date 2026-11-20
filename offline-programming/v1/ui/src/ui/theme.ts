import { Signal } from "./signal.ts";

/**
 * The window's colour theme: the shop-grey palette with the lights up
 * (light) or down (dark). The operator picks System, Light or Dark; the
 * page always carries the RESOLVED theme as `<html data-theme>`, so the
 * stylesheet has one dark block keyed on that attribute rather than a
 * second copy inside a `prefers-color-scheme` query, and canvas code (the
 * 3D scene, the axis gizmo, the telemetry plot) reads the same tokens the
 * CSS does.
 *
 * index.html sets the attribute before the stylesheet paints, from the
 * same storage key, so a dark-mode operator never sees a light flash on
 * load. The choice is per browser: it is a display preference, not program
 * state, and nothing planned or sent to a robot reads it.
 */
export type ThemePreference = "system" | "light" | "dark";
export type Theme = "light" | "dark";

export const THEME_STORAGE_KEY = "olp.theme";

/** Emitted with the resolved theme whenever it changes, by either route:
 *  the operator's choice or, under System, the OS setting. */
export const themeChanged = new Signal<Theme>();

let preference: ThemePreference = readThemePreference();
const systemDark = typeof matchMedia === "function" ? matchMedia("(prefers-color-scheme: dark)") : null;
systemDark?.addEventListener("change", () => {
  if (preference === "system") applyResolved();
});

export function readThemePreference(): ThemePreference {
  try {
    const stored = localStorage.getItem(THEME_STORAGE_KEY);
    return stored === "light" || stored === "dark" ? stored : "system";
  } catch {
    return "system";
  }
}

export function themePreference(): ThemePreference {
  return preference;
}

export function currentTheme(): Theme {
  if (typeof document === "undefined") return "light";
  return document.documentElement.dataset.theme === "dark" ? "dark" : "light";
}

export function setThemePreference(next: ThemePreference): void {
  preference = next;
  try {
    if (next === "system") localStorage.removeItem(THEME_STORAGE_KEY);
    else localStorage.setItem(THEME_STORAGE_KEY, next);
  } catch { /* the choice lasts the session */ }
  applyResolved();
}

/** Put the resolved theme on the page (again): idempotent, and the one
 *  place `data-theme` is written after load. */
export function applyResolved(): Theme {
  const theme: Theme = preference === "system" ? (systemDark?.matches ? "dark" : "light") : preference;
  const root = document.documentElement;
  const changed = root.dataset.theme !== theme;
  root.dataset.theme = theme;
  // The browser chrome's colour follows the window's field.
  const meta = document.querySelector<HTMLMetaElement>('meta[name="theme-color"]');
  if (meta) meta.content = getComputedStyle(root).getPropertyValue("--bg").trim() || meta.content;
  if (changed) themeChanged.emit(theme);
  return theme;
}

/** A token's current value, for code that paints outside CSS. */
export function themeToken(name: string, fallback: string): string {
  if (typeof document === "undefined" || typeof getComputedStyle !== "function") return fallback;
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}
