import assert from "node:assert/strict";
import test from "node:test";

import { Action, ActionGroup } from "./action.ts";
import { Signal } from "./signal.ts";
import { parseShortcut, sameChord } from "./shortcuts.ts";

test("a signal runs every slot even when one throws, then rethrows", () => {
  const signal = new Signal<number>();
  const seen: number[] = [];
  signal.connect(() => {
    throw new Error("first slot broke");
  });
  signal.connect((value) => seen.push(value));
  assert.throws(() => signal.emit(7), /first slot broke/);
  assert.deepEqual(seen, [7]);
});

test("a disconnect returned by connect stops the slot", () => {
  const signal = new Signal<void>();
  let calls = 0;
  const off = signal.connect(() => {
    calls += 1;
  });
  signal.emit();
  off();
  signal.emit();
  assert.equal(calls, 1);
});

test("trigger refuses while disabled and says so", () => {
  const action = new Action({ id: "frame", label: "Frame" });
  let fired = 0;
  action.triggered.connect(() => {
    fired += 1;
  });
  action.setEnabled(false, "Locked while a robot is moving");
  assert.equal(action.trigger(), false);
  assert.equal(fired, 0);
  assert.equal(action.title, "Locked while a robot is moving");
  action.enabled = true;
  assert.equal(action.trigger(), true);
  assert.equal(fired, 1);
});

test("a checkable action flips on trigger and mirrors on set, without retriggering", () => {
  const action = new Action({ id: "labels", label: "Labels", checkable: true, checked: true });
  const toggles: boolean[] = [];
  let fired = 0;
  action.toggled.connect((value) => toggles.push(value));
  action.triggered.connect(() => {
    fired += 1;
  });
  action.trigger();
  assert.equal(action.checked, false);
  // The owner of the state pushes it back: a view update, not a command.
  action.checked = true;
  assert.deepEqual(toggles, [false, true]);
  assert.equal(fired, 1);
});

test("the title carries the shortcut, and the disabled reason replaces it", () => {
  const action = new Action({ id: "move", label: "Move part", tooltip: "Drag the part", shortcut: "M" });
  assert.equal(action.title, "Drag the part [M]");
  action.setEnabled(false, "Import a part first");
  assert.equal(action.title, "Import a part first");
  action.setEnabled(false);
  assert.equal(action.title, "Drag the part [M]");
});

test("an exclusive group checks one and unchecks the others", () => {
  const group = new ActionGroup();
  const sim = new Action({ id: "sim", label: "Sim", group, checked: true });
  const real = new Action({ id: "real", label: "Real", group });
  const picked: string[] = [];
  group.selected.connect((action) => picked.push(action.id));
  real.trigger();
  assert.equal(sim.checked, false);
  assert.equal(real.checked, true);
  // Re-triggering the checked one keeps it checked (a radio, not a toggle).
  real.trigger();
  assert.equal(real.checked, true);
  assert.deepEqual(picked, ["real"]);
  assert.equal(group.checked, real);
});

test("shortcuts parse modifiers in any order and case", () => {
  assert.ok(sameChord(parseShortcut("Ctrl+Shift+Z"), parseShortcut("shift+ctrl+z")));
  assert.ok(!sameChord(parseShortcut("Z"), parseShortcut("Shift+Z")));
  assert.deepEqual(parseShortcut("Esc"), { key: "escape", ctrl: false, shift: false, alt: false });
});
