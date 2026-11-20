import { readFile } from "node:fs/promises";
import assert from "node:assert/strict";
import { access } from "node:fs/promises";
import test from "node:test";

import { STEP_EXAMPLES } from "../scripts/step-examples.mjs";

const repoRoot = new URL("../../../../", import.meta.url);

const exists = (path: string | URL) => access(path).then(() => true, () => false);

// The full tree carries every part; the public mirror leaves out the private
// ones together with tools/public-mirror/private-paths.txt, and the stage
// offers only the parts present. The bundled example is public in both.
test("every example STEP the UI offers exists where the stage copies it from", async () => {
  assert.ok(STEP_EXAMPLES.length >= 3);
  const fullTree = await exists(new URL("tools/public-mirror/private-paths.txt", repoRoot));
  const bundled = STEP_EXAMPLES[0];
  assert.ok(bundled, "no bundled example");
  const ids = new Set<string>();
  for (const example of STEP_EXAMPLES) {
    assert.ok(!ids.has(example.id), `duplicate example id ${example.id}`);
    ids.add(example.id);
    assert.match(example.source, /\.step$/);
    if (fullTree || example.id === bundled.id) {
      assert.ok(await exists(new URL(example.source, repoRoot)), `missing ${example.source}`);
    }
    assert.ok(example.label && example.description);
  }
});

test("the demo parts are not offered in the menus", async () => {
  const source = await readFile(new URL("../src/main.ts", import.meta.url), "utf8");
  assert.doesNotMatch(source, /addHeading\("Open demo file"\)/);
  assert.doesNotMatch(source, /Demo: \$\{example\.label\}/);
});
