import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";
import { newBoxFixture, newStepFixture } from "../domain/workcell.ts";
import { StandCard } from "./standCard.ts";

test("fixture tools preserve stand reset while imported fixtures have independent placement and removal", () => {
  const window = new Window();
  const before = Object.getOwnPropertyDescriptor(globalThis, "document");
  Object.defineProperty(globalThis, "document", { configurable: true, value: window.document });
  const calls: unknown[] = [];
  const card = new StandCard({
    onChange: (field, value) => calls.push([field, value]),
    onReplaceWithStep: () => calls.push("replace"),
    onCellDefault: () => calls.push("default"),
    onRemove: () => calls.push("remove"),
    onAccept: () => calls.push("accept"),
    onCancel: () => calls.push("cancel"),
  }).mount(window.document.body as unknown as HTMLElement);
  try {
    const button = (text: string) => Array.from(card.el.querySelectorAll("button"))
      .find((candidate) => candidate.textContent === text)!;
    const reset = button("Cell default");
    const stand = newBoxFixture();
    card.setFixture(stand);
    assert.equal(reset.hidden, false);
    reset.click();
    assert.deepEqual(calls, ["default"]);
    const sizeSection = card.el.querySelector("details")!;
    assert.equal(sizeSection.hidden, false);

    const fixture = newStepFixture([stand], { filename: "clamp.step", fingerprint: "clamp" });
    card.setFixture(fixture);
    assert.equal(card.el.getAttribute("aria-label"), "clamp.step");
    assert.equal(reset.hidden, true);
    assert.equal(reset.disabled, true);
    reset.dispatchEvent(new window.MouseEvent("click") as unknown as MouseEvent);
    assert.deepEqual(calls, ["default"], "generic fixtures cannot restore the stand default");
    assert.equal(sizeSection.hidden, true, "STEP fixtures do not expose box dimensions");
    const x = card.el.querySelector<HTMLInputElement>('input[aria-label="X"]')!;
    assert.equal(x.value, "0.0");
    x.value = "250";
    x.dispatchEvent(new window.Event("change") as unknown as Event);
    button("Replace with STEP").click();
    button("Remove").click();
    assert.deepEqual(calls, ["default", ["x", 250], "replace", "remove"]);
    card.setFixture(stand);
    assert.equal(reset.hidden, false);
    assert.equal(sizeSection.hidden, false);
  } finally {
    card.dispose();
    if (before) Object.defineProperty(globalThis, "document", before);
    else Reflect.deleteProperty(globalThis, "document");
    void window.happyDOM.close();
  }
});
