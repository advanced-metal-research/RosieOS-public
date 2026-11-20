import assert from "node:assert/strict";
import test from "node:test";
import { isBusyText, markBusy, setBusyText } from "./busy.ts";

class Stub {
  textContent = "";
  #classes = new Set<string>();
  #attrs = new Map<string, string>();
  classList = {
    toggle: (name: string, force: boolean) => { if (force) this.#classes.add(name); else this.#classes.delete(name); return force; },
    contains: (name: string) => this.#classes.has(name),
  };
  setAttribute(name: string, value: string): void { this.#attrs.set(name, value); }
  removeAttribute(name: string): void { this.#attrs.delete(name); }
  getAttribute(name: string): string | null { return this.#attrs.get(name) ?? null; }
}

test("a status is busy when it ends in an ellipsis, with or without trailing space", () => {
  assert.equal(isBusyText("Planning…"), true);
  assert.equal(isBusyText("reading the newest 3 s of the ring… "), true);
  assert.equal(isBusyText("Planned 8 welds"), false);
  // A trailing ellipsis means in progress, so a truncated digest must end in words, not dots.
  assert.equal(isBusyText("Exported plan · sha256 0123abcd (first 12)"), false);
  assert.equal(isBusyText(""), false);
});

test("markBusy toggles the trace class and tells assistive technology", () => {
  const el = new Stub();
  markBusy(el as unknown as Element, true);
  assert.equal(el.classList.contains("is-busy"), true);
  assert.equal(el.getAttribute("aria-busy"), "true");
  markBusy(el as unknown as Element, false);
  assert.equal(el.classList.contains("is-busy"), false);
  assert.equal(el.getAttribute("aria-busy"), null);
});

test("setBusyText writes the words and marks busy from them", () => {
  const el = new Stub();
  setBusyText(el as unknown as HTMLElement, "Importing dog_box.step through local CadQuery…");
  assert.equal(el.textContent, "Importing dog_box.step through local CadQuery…");
  assert.equal(el.classList.contains("is-busy"), true);
  setBusyText(el as unknown as HTMLElement, "128 topology edges · select edges to define welds");
  assert.equal(el.classList.contains("is-busy"), false);
});
