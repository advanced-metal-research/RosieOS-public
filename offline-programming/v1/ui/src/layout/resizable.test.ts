import assert from "node:assert/strict";
import test from "node:test";
import {
  DEFAULT_RESIZABLE_LAYOUT,
  normalizeResizableLayout,
} from "./resizable.ts";

test("resizable layout preserves valid desktop dimensions", () => {
  assert.deepEqual(
    normalizeResizableLayout({ ...DEFAULT_RESIZABLE_LAYOUT }, 1_920, 1_080),
    DEFAULT_RESIZABLE_LAYOUT,
  );
});

test("resizable layout clamps panels and camera splits", () => {
  assert.deepEqual(
    normalizeResizableLayout({
      authorWidthPx: 9_000,
      programWidthPx: -5,
      spreadsheetHeightPx: 9_000,
      cameraColumnPercent: 99,
      cameraRowPercent: 2,
    }, 1_280, 1_080),
    {
      authorWidthPx: 560,
      programWidthPx: 260,
      spreadsheetHeightPx: 750,
      cameraColumnPercent: 82,
      cameraRowPercent: 20,
    },
  );
});

test("resizable layout protects a usable center stage on narrow desktops", () => {
  const layout = normalizeResizableLayout({
    authorWidthPx: 500,
    programWidthPx: 500,
    spreadsheetHeightPx: 360,
    cameraColumnPercent: 60,
    cameraRowPercent: 70,
  }, 1_120, 900);
  assert.equal(layout.authorWidthPx, 420);
  assert.equal(layout.programWidthPx, 260);
  assert.equal(layout.spreadsheetHeightPx, 360);
  assert.equal(layout.cameraColumnPercent, 60);
  assert.equal(layout.cameraRowPercent, 70);
});

test("resizable layout preserves the author panel content minimum", () => {
  const layout = normalizeResizableLayout({
    ...DEFAULT_RESIZABLE_LAYOUT,
    authorWidthPx: 100,
  }, 1_920, 1_080);
  assert.equal(layout.authorWidthPx, 380);
});

test("resizable layout keeps the spreadsheet usable without hiding the stage", () => {
  const tooSmall = normalizeResizableLayout({
    ...DEFAULT_RESIZABLE_LAYOUT,
    spreadsheetHeightPx: 40,
  }, 1_920, 900);
  const tooLarge = normalizeResizableLayout({
    ...DEFAULT_RESIZABLE_LAYOUT,
    spreadsheetHeightPx: 2_000,
  }, 1_920, 900);

  assert.equal(tooSmall.spreadsheetHeightPx, 220);
  assert.equal(tooLarge.spreadsheetHeightPx, 570);
});

// The boundary between the Jog column and the plot is a resizer like the
// others (owner, 2026-09-20): dragged, arrowed, reset by double click, and
// remembered per browser as a share of the Jog pane.
test("the Jog column boundary is wired like the other resizers", async () => {
  const { readFile } = await import("node:fs/promises");
  const source = await readFile(new URL("./resizable.ts", import.meta.url), "utf8");
  assert.match(source, /jogColumnResizer\?: HTMLElement;/);
  assert.match(source, /localStorage\.getItem\("olp\.jog\.column"\)/);
  assert.match(source, /localStorage\.setItem\("olp\.jog\.column", JSON\.stringify\(jogColumnShare\)\)/);
  assert.match(source, /jogColumnShare = clamp\(jogColumnShare, 25, 75\);/);
  assert.match(source, /style\.setProperty\("--jog-column", jogColumnShare \+ "%"\)/);
  assert.match(source, /jogColumnShare = \(event\.clientX - bounds\.left\) \/ bounds\.width \* 100;/);
  assert.match(source, /adjust\(key, "ArrowLeft", "ArrowRight", delta => \{ jogColumnShare \+= delta; \}\)/);
  const main = await readFile(new URL("../main.ts", import.meta.url), "utf8");
  assert.match(main, /id="jog-column-resizer" class="workspace-resizer jog-column-resizer" role="separator"/);
  assert.match(main, /jogColumnResizer: query<HTMLElement>\("#jog-column-resizer"\),\n\s+jogPanel: query<HTMLElement>\("#jog-panel"\),/);
  const css = await readFile(new URL("../style.css", import.meta.url), "utf8");
  assert.match(css, /grid-template-columns: minmax\(340px, var\(--jog-column, 42%\)\) 10px minmax\(0, 1fr\);/);
  assert.match(css, /\.jog-panel > \.jog-column-resizer \{ grid-row: 1 \/ 3; grid-column: 2; \}/);
});
