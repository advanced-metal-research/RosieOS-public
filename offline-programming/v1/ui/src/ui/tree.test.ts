import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";
import { TreeView, type TreeNode } from "./tree.ts";

test("unchanged status refresh preserves an import press, focus and latest callback", () => {
  const window = new Window();
  const before = Object.getOwnPropertyDescriptor(globalThis, "document");
  Object.defineProperty(globalThis, "document", { configurable: true, value: window.document });
  const tree = new TreeView().mount(window.document.body as unknown as HTMLElement);
  try {
    let imported = "";
    const nodes = (value: string): TreeNode[] => [{ id: "part", label: "No part", muted: true,
      hint: { label: "Import a part", run: () => { imported = value; } } }];
    tree.render(nodes("old"));
    const hint = tree.el.querySelector<HTMLButtonElement>(".tree-hint")!;
    hint.focus();
    hint.dispatchEvent(new window.MouseEvent("mousedown", { bubbles: true }) as unknown as MouseEvent);
    for (let poll = 0; poll < 10; ++poll) tree.render(nodes("latest"));
    assert.ok(tree.el.querySelector(".tree-hint") === hint, "status refresh replaced the pressed import button");
    assert.equal(hint.isConnected, true);
    assert.equal(window.document.activeElement, hint);
    hint.click();
    assert.equal(imported, "latest");
    assert.equal(tree.selectedId, null);
    for (const key of ["Enter", " "]) {
      const event = new window.KeyboardEvent("keydown", { key, bubbles: true, cancelable: true });
      hint.dispatchEvent(event as unknown as KeyboardEvent);
      assert.equal(event.defaultPrevented, false);
      assert.equal(tree.selectedId, null);
    }
    tree.render([{ id: "part", label: "bracket.step", busy: true, selectable: false }]);
    const busyRow = tree.el.querySelector(".tree-row")!;
    assert.equal(busyRow.getAttribute("aria-busy"), "true");
    const spinner = busyRow.querySelector(".tree-spinner")!;
    assert.equal(spinner.getAttribute("aria-label"), "Loading bracket.step");
    assert.equal(spinner.previousElementSibling?.textContent, "bracket.step");
    tree.render([{ id: "part", label: "bracket.step", busy: true, selectable: false }]);
    assert.ok(tree.el.querySelector(".tree-spinner") === spinner, "poll restarted the loading circle");
    tree.render([{ id: "part", label: "bracket.step" }]);
    assert.equal(tree.el.querySelector(".tree-spinner"), null);
    assert.equal(tree.el.querySelector(".tree-row")!.hasAttribute("aria-busy"), false);
    assert.equal(tree.el.querySelector(".tree-hint"), null);
    assert.match(tree.el.textContent!, /bracket.step/);
    const row = tree.el.querySelector<HTMLElement>(".tree-row")!;
    row.dispatchEvent(new window.KeyboardEvent("keydown", { key: "Enter", bubbles: true }) as unknown as KeyboardEvent);
    assert.equal(tree.selectedId, "part");
  } finally {
    tree.dispose();
    if (before) Object.defineProperty(globalThis, "document", before);
    else Reflect.deleteProperty(globalThis, "document");
    void window.happyDOM.close();
  }
});
