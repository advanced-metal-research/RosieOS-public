import assert from "node:assert/strict";
import test from "node:test";
import { readFile } from "node:fs/promises";

// A New program showed the previous part's orange seam guides (owner,
// 2026-09-20): disable() only hid the overlay group and the sync path never
// disabled a rails program without a part. The rails mode now forgets the
// analysed part when the program's part is not the analysed one.
test("rails mode forgets the analysed part when the program's part differs or is missing", async () => {
  const rails = await readFile(new URL("./index.ts", import.meta.url), "utf8");
  assert.match(rails, /readonly fingerprint: string \| null;/);
  assert.match(rails, /reset\(\): void;/);
  const reset = rails.slice(rails.indexOf("    reset() {"), rails.indexOf("    async toggle()"));
  for (const step of ["disposeOverlay(entry.overlay)", "entries.length = 0", "seamPlans.clear()", "framesByJoint.clear()", "disposeOverlay(guides)", "doc = null", "sha = null", "enabled = false", "root.visible = false"]) {
    assert.ok(reset.includes(step), `reset() must ${step}`);
  }
  const main = await readFile(new URL("../main.ts", import.meta.url), "utf8");
  const sync = main.slice(main.indexOf("function syncRailsModeToProject(): void {"), main.indexOf("  syncRailsButton();\n  syncEdgeSelectionState();\n  updateTargetOptions();\n  renderTargetAndReview();\n}"));
  assert.match(sync, /const analysed = railsMode\.fingerprint;/);
  assert.match(sync, /analysed !== null && \(!hasWorkpieceGeometry\(project\) \|\| analysed !== project\.cad\.fingerprint\)\) railsMode\.reset\(\);/);
  // The reset comes before the enable gate, so a changed part is analysed afresh.
  assert.ok(sync.indexOf("railsMode.reset()") < sync.indexOf("railsMode.enable()"));
  // Every program switch goes through the sync path.
  for (const site of ["async function createNewProgram", "async function importProject"]) {
    const body = main.slice(main.indexOf(site), main.indexOf("\n}\n", main.indexOf(site)));
    assert.ok(body.includes("syncRailsModeToProject();"), `${site} must sync rails`);
  }
});
