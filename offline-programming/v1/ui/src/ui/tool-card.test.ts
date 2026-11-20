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
  Event: browser.Event,
  localStorage: browser.localStorage,
});

const { ToolCard } = await import("./tool-card.ts");
const { NumberField } = await import("./fields.ts");

function click(el: HTMLElement): void {
  el.dispatchEvent(new browser.MouseEvent("click", { bubbles: true }) as unknown as MouseEvent);
}

test("a tool card's ✓ and ✕ reach the tool, and Escape is ✕", () => {
  const calls: string[] = [];
  const card = new ToolCard({
    title: "Move part",
    onAccept: () => calls.push("accept"),
    onCancel: () => calls.push("cancel"),
  });
  document.body.append(card.el);
  click(card.el.querySelector(".tool-card-accept") as HTMLElement);
  click(card.el.querySelector(".tool-card-cancel") as HTMLElement);
  card.el.dispatchEvent(new browser.KeyboardEvent("keydown", { key: "Escape", bubbles: true }) as unknown as KeyboardEvent);
  assert.deepEqual(calls, ["accept", "cancel", "cancel"]);
  card.dispose();
});

test("a number field reports finished edits and is not overwritten mid-edit", () => {
  const seen: number[] = [];
  const field = new NumberField({ label: "X", unit: "mm", accent: "x", onChange: (v) => seen.push(v) });
  document.body.append(field.el);
  assert.equal(field.el.dataset.accent, "x");
  field.setValue("12.5");
  assert.equal(field.input.value, "12.5");
  field.input.focus();
  field.input.value = "40";
  field.setValue("99");
  assert.equal(field.input.value, "40", "a render pass must not eat a half-typed value");
  field.input.dispatchEvent(new browser.Event("change", { bubbles: true }) as unknown as Event);
  assert.deepEqual(seen, [40]);
  field.input.blur();
  field.setValue("99");
  assert.equal(field.input.value, "99");
  field.dispose();
});

test("fields added to a card are disposed with it", () => {
  const card = new ToolCard({ title: "T" });
  const field = card.add(new NumberField({ label: "Z", onChange: () => {} }));
  assert.equal(card.el.querySelectorAll(".number-field").length, 1);
  card.dispose();
  assert.equal(field.disposed, true);
});

test("a titled section is collapsible, shows a summary, and remembers its state", () => {
  const card = new ToolCard({ title: "Rails welds" });
  const search = card.addSection({ title: "Angle search", key: "search", open: false, summary: "Steady" });
  document.body.append(card.el);
  const details = search.closest("details") as HTMLDetailsElement;
  assert.equal(details.open, false);
  assert.match(details.querySelector(".tool-card-section-summary")!.textContent ?? "", /Steady/);
  card.setSectionSummary(search, "Steady · 2 changed");
  assert.equal(details.querySelector(".tool-card-section-summary")!.textContent, "Steady · 2 changed");
  card.setSectionOpen(search, true);
  details.dispatchEvent(new browser.Event("toggle") as unknown as Event);
  card.dispose();
  // A second card of the same name opens the section the way it was left.
  const again = new ToolCard({ title: "Rails welds" });
  const section = again.addSection({ title: "Angle search", key: "search", open: false });
  assert.equal((section.closest("details") as HTMLDetailsElement).open, true);
  again.dispose();
  try {
    localStorage.removeItem("olp.tool-card.Rails welds.search");
  } catch {
    // no storage in this runtime
  }
});
