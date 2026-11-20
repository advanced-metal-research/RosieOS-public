import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";

const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  HTMLElement: browser.HTMLElement,
  Node: browser.Node,
  KeyboardEvent: browser.KeyboardEvent,
  MouseEvent: browser.MouseEvent,
  PointerEvent: browser.MouseEvent,
  Event: browser.Event,
  localStorage: browser.localStorage,
});

const { Dialog } = await import("./dialog.ts");

test("a dialog opens over the page and closes on ✕, Escape and the backdrop", () => {
  let closed = 0;
  const dialog = new Dialog({ title: "Robot", onClose: () => { closed += 1; } });
  dialog.mount(document.body);
  const body = dialog.addSection({ title: "Cells", summary: "none" });
  body.append(document.createTextNode("list"));
  assert.equal(dialog.shown, false);
  dialog.open();
  assert.equal(dialog.shown, true);
  assert.equal(dialog.panel.getAttribute("aria-modal"), "true");
  assert.equal(dialog.el.querySelector(".dialog-section-summary")?.textContent, "none");
  dialog.setSectionSummary(body, "dev-cell");
  assert.equal(dialog.el.querySelector(".dialog-section-summary")?.textContent, "dev-cell");
  dialog.el.querySelector<HTMLButtonElement>(".dialog-close")!.dispatchEvent(new browser.MouseEvent("click", { bubbles: true }) as unknown as MouseEvent);
  assert.equal(dialog.shown, false);
  assert.equal(closed, 1);
  dialog.open();
  document.dispatchEvent(new browser.KeyboardEvent("keydown", { key: "Escape", bubbles: true }) as unknown as KeyboardEvent);
  assert.equal(dialog.shown, false);
  assert.equal(closed, 2);
  dialog.open();
  // A click on the panel stays; a click on the dimmed page closes.
  dialog.panel.dispatchEvent(new browser.MouseEvent("pointerdown", { bubbles: true }) as unknown as MouseEvent);
  assert.equal(dialog.shown, true);
  dialog.el.dispatchEvent(new browser.MouseEvent("pointerdown") as unknown as MouseEvent);
  assert.equal(dialog.shown, false);
  assert.equal(closed, 3);
  // A collapsible section is a <details>.
  dialog.addSection({ title: "Add cell", collapsible: true, open: false });
  const details = dialog.el.querySelector<HTMLDetailsElement>("details.dialog-section")!;
  assert.equal(details.open, false);
});
