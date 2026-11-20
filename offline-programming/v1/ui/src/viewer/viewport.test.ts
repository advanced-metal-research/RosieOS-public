import test from "node:test";
import assert from "node:assert/strict";
import {
  expectedBufferScissor,
  rendererSize,
  validateScissorPaneLayout,
  viewportFromRects,
  type RectLike,
} from "./viewport.ts";

function rect(left: number, top: number, width: number, height: number): RectLike {
  return {
    left,
    top,
    right: left + width,
    bottom: top + height,
    width,
    height,
  };
}

test("viewport is canvas-relative CSS pixels with WebGL bottom origin", () => {
  const canvas = rect(100, 50, 1000, 600);
  const pane = rect(720, 50, 380, 300);
  assert.deepEqual(viewportFromRects(canvas, pane), {
    x: 620,
    y: 300,
    width: 380,
    height: 300,
    aspect: 380 / 300,
  });
});

test("device pixel ratio is applied exactly once to the expected GL scissor", () => {
  const viewport = viewportFromRects(
    rect(0, 0, 1000, 600),
    rect(620, 300, 380, 300),
  );
  assert.deepEqual(expectedBufferScissor(viewport, 2.25), {
    x: 1395,
    y: 0,
    width: 855,
    height: 675,
  });
  // The viewport itself remains CSS-sized regardless of DPR.
  assert.equal(viewport.x, 620);
  assert.equal(viewport.width, 380);
});

test("renderer sizing follows viewport and DPR changes after browser startup", () => {
  assert.deepEqual(rendererSize(631.2, 720.1, 1, 2), {
    width: 632,
    height: 721,
    pixelRatio: 1,
  });
  assert.deepEqual(rendererSize(946.01, 814, 1.5, 2), {
    width: 947,
    height: 814,
    pixelRatio: 1.5,
  });
  assert.equal(rendererSize(946, 814, 3, 2).pixelRatio, 2);
});

test("split pane validation proves disjoint shared edges and stage coverage", () => {
  const stage = rect(10, 20, 1000, 600);
  const canvas = rect(10, 20, 1000, 600);
  const orbit = rect(10, 20, 620, 600);
  const j6 = rect(630, 20, 380, 300);
  const tcp = rect(630, 320, 380, 300);
  assert.equal(validateScissorPaneLayout(stage, canvas, orbit, j6, tcp).ok, true);

  const shiftedTcp = rect(640, 320, 370, 300);
  const invalid = validateScissorPaneLayout(stage, canvas, orbit, j6, shiftedTcp);
  assert.equal(invalid.ok, false);
  assert.match(invalid.issues.join("\n"), /camera column/);
});
