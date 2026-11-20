import assert from "node:assert/strict";
import test from "node:test";

import { viewportShellHeight } from "./viewport-shell.ts";

test("viewport shell uses the browser's current root height rather than a startup viewport unit", () => {
  assert.equal(viewportShellHeight(840), 840);
  assert.equal(viewportShellHeight(1080), 1080);
  assert.equal(viewportShellHeight(1079.6), 1080);
  assert.equal(viewportShellHeight(0), 1);
});
