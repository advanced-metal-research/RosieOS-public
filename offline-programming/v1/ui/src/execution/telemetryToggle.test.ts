import assert from "node:assert/strict";
import test from "node:test";
import { readFile } from "node:fs/promises";

// The telemetry panel is hideable from the toolbar, next to Jog, and the
// choice is remembered per browser (owner, 2026-09-20). Source pins: the
// toggle, what it hides, and the dock floor that follows it.
test("the toolbar toggle beside Jog hides the telemetry panel and the pane closes up", async () => {
  const main = await readFile(new URL("../main.ts", import.meta.url), "utf8");
  assert.match(main, /const TELEMETRY_VISIBLE_KEY = "olp\.telemetry\.shown";/);
  assert.match(main, /new Action\(\{ id: "toggle-telemetry", label: "Telemetry", icon: Activity, checkable: true, checked: telemetryVisible/);
  // The bottom dock's views share one group: the spreadsheet, then Jog and
  // the telemetry beside it (owner, 2026-09-26).
  assert.match(main, /appToolbar\.addGroup\(\)\.addActions\(\[stageActions\.spreadsheet, jogAction, telemetryAction\]\);/);
  const apply = main.slice(main.indexOf("function applyTelemetryVisible(): void {"), main.indexOf("\n}\n", main.indexOf("function applyTelemetryVisible(): void {")));
  for (const step of [
    'query("#telemetry-strip").hidden = !shown;',
    'query("#jog-column-resizer").hidden = !shown || !jogPadVisible;',
    'query("#jog-panel").classList.toggle("is-strip-hidden", !shown);',
    'query("#jog-panel").classList.toggle("is-controls-hidden", !jogPadVisible);',
    'query("#jog-panel").hidden = !dockPaneShown();',
    'query("#bottom-workspace").classList.toggle("strip-hidden", !shown);',
    "resizableLayout.apply();",
  ]) assert.ok(apply.includes(step), `applyTelemetryVisible must ${step}`);
  // Telemetry shows without the Jog controls (owner, 2026-09-27): the toggle
  // is never disabled for a closed Jog, and either view keeps the pane open.
  assert.ok(!apply.includes("telemetryAction.setEnabled("), "telemetry must not wait for Jog");
  assert.match(main, /function dockPaneShown\(\): boolean \{\n  return jogPadVisible \|\| telemetryVisible;\n\}/);
  assert.match(main, /dockMinHeight: \(\) => \(dockPaneShown\(\) \? \(telemetryVisible \? JOG_DOCK_MIN_HEIGHT_PX : JOG_DOCK_MIN_HEIGHT_JOINTS_PX\) : 0\)/);
  assert.match(main, /if \(panel\) panel\.hidden = !dockPaneShown\(\);/);
  // Hiding the Jog pane also re-applies which of the pane's views remain.
  const toggleJog = main.slice(main.indexOf("function toggleJogPad(): void {"), main.indexOf("\n}\n", main.indexOf("function toggleJogPad(): void {")));
  assert.ok(toggleJog.includes("applyTelemetryVisible();"));
  const css = await readFile(new URL("../style.css", import.meta.url), "utf8");
  assert.match(css, /\.jog-panel\.is-strip-hidden \{ grid-template-columns: minmax\(0, 1fr\); \}/);
  assert.match(css, /\.jog-panel\.is-controls-hidden > \.jog-column \{ display: none; \}/);
  assert.match(css, /\.bottom-workspace\.has-jog\.has-sheet\.strip-hidden \{ grid-template-columns: minmax\(0, 1fr\) 10px minmax\(min\(380px, 100%\), var\(--jog-share, 40%\)\); \}/);
});
