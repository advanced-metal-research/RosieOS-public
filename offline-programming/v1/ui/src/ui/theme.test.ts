import assert from "node:assert/strict";
import test from "node:test";
import { readFile } from "node:fs/promises";

// The theme module reads the page (storage, the OS query, <html>) when it
// loads, so the page is installed first and the module imported after.
async function loadTheme(stored: string | null) {
  const { Window } = await import("happy-dom");
  const window = new Window();
  if (stored !== null) window.localStorage.setItem("olp.theme", stored);
  Object.assign(globalThis, {
    document: window.document,
    localStorage: window.localStorage,
    matchMedia: window.matchMedia.bind(window),
    getComputedStyle: window.getComputedStyle.bind(window),
  });
  return import(`./theme.ts?case=${stored}-${Math.random()}`) as Promise<typeof import("./theme.ts")>;
}

test("a stored choice is the page's theme, and System follows the OS query", async () => {
  const theme = await loadTheme("dark");
  assert.equal(theme.themePreference(), "dark");
  assert.equal(theme.applyResolved(), "dark");
  assert.equal(document.documentElement.dataset.theme, "dark");

  const seen: string[] = [];
  theme.themeChanged.connect((value) => seen.push(value));
  theme.setThemePreference("light");
  assert.equal(localStorage.getItem(theme.THEME_STORAGE_KEY), "light");
  assert.equal(theme.currentTheme(), "light");
  // System forgets the choice; the stub OS reports no dark preference.
  theme.setThemePreference("system");
  assert.equal(localStorage.getItem(theme.THEME_STORAGE_KEY), null);
  assert.equal(theme.currentTheme(), "light");
  assert.deepEqual(seen, ["light"], "only a change of the resolved theme is announced");
});

test("an unknown stored value reads as System", async () => {
  const theme = await loadTheme("sepia");
  assert.equal(theme.themePreference(), "system");
});

test("index.html resolves the theme from the same key before the stylesheet paints", async () => {
  const html = await readFile(new URL("../../index.html", import.meta.url), "utf8");
  const theme = await loadTheme(null);
  assert.ok(html.includes(`localStorage.getItem("${theme.THEME_STORAGE_KEY}")`));
  assert.ok(html.indexOf("dataset.theme") < html.indexOf("/src/main.ts"));
  const css = await readFile(new URL("../style.css", import.meta.url), "utf8");
  assert.match(css, /:root\[data-theme="dark"\] \{\n  color-scheme: dark;/);
});
