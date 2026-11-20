import assert from "node:assert/strict";
import test from "node:test";
import { Window } from "happy-dom";

// A browser for the widgets: happy-dom supplies document, HTMLElement and
// events. Installed once for this file; the pure ui logic (Action, Signal,
// shortcuts parsing) is tested without it in action.test.ts.
const browser = new Window();
Object.assign(globalThis, {
  window: browser,
  document: browser.document,
  HTMLElement: browser.HTMLElement,
  Node: browser.Node,
  KeyboardEvent: browser.KeyboardEvent,
  PointerEvent: browser.PointerEvent,
  MouseEvent: browser.MouseEvent,
  Event: browser.Event,
});

const { Action, ActionGroup } = await import("./action.ts");
const { ToolBar } = await import("./toolbar.ts");
const { Menu, MenuButton } = await import("./menu.ts");
const { Shortcuts } = await import("./shortcuts.ts");

function click(el: HTMLElement): void {
  el.dispatchEvent(new browser.MouseEvent("click", { bubbles: true }) as unknown as MouseEvent);
}

test("a tool button shows its action and triggers it", () => {
  const action = new Action({ id: "frame", label: "Frame", tooltip: "Frame the program", shortcut: "F" });
  let fired = 0;
  action.triggered.connect(() => {
    fired += 1;
  });
  const bar = new ToolBar();
  const [button] = bar.addGroup("View").addActions([action]);
  assert.equal(button!.el.textContent, "Frame");
  assert.equal(button!.el.title, "Frame the program [F]");
  click(button!.el);
  assert.equal(fired, 1);
  action.setEnabled(false, "Nothing to frame");
  assert.equal(button!.el.disabled, true);
  assert.equal(button!.el.title, "Nothing to frame");
  click(button!.el);
  assert.equal(fired, 1);
});

test("a checkable button follows the action, whichever side changed", () => {
  const action = new Action({ id: "labels", label: "Labels", checkable: true, checked: true });
  const bar = new ToolBar();
  const [button] = bar.addGroup().addActions([action]);
  assert.equal(button!.el.getAttribute("aria-pressed"), "true");
  assert.ok(button!.el.classList.contains("is-active"));
  click(button!.el);
  assert.equal(action.checked, false);
  assert.equal(button!.el.getAttribute("aria-pressed"), "false");
  action.checked = true;
  assert.ok(button!.el.classList.contains("is-active"));
});

test("an exclusive group renders as a radio pair", () => {
  const group = new ActionGroup();
  const sim = new Action({ id: "sim", label: "Sim", group, checked: true });
  const real = new Action({ id: "real", label: "Real", group });
  const bar = new ToolBar();
  const [simButton, realButton] = bar.addGroup("Pose").addActions([sim, real]);
  click(realButton!.el);
  assert.equal(simButton!.el.getAttribute("aria-pressed"), "false");
  assert.equal(realButton!.el.getAttribute("aria-pressed"), "true");
});

test("separators go between groups, not before the first", () => {
  const bar = new ToolBar();
  bar.addGroup("A");
  bar.addGroup("B");
  bar.addGroup("C");
  const kinds = [...bar.el.children].map((child) => child.className);
  assert.deepEqual(kinds, ["tool-group", "tool-separator", "tool-group", "tool-separator", "tool-group"]);
});

test("disposing the bar disconnects its buttons from their actions", () => {
  const action = new Action({ id: "x", label: "X", checkable: true });
  const bar = new ToolBar();
  bar.addGroup().addActions([action]);
  assert.equal(action.changed.connections, 1);
  bar.dispose();
  assert.equal(action.changed.connections, 0);
});

test("a menu opens under its button, chooses an action and closes", async () => {
  const j6 = new Action({ id: "j6", label: "J6 cam", checkable: true, checked: true });
  const menu = new Menu("Cameras");
  menu.addActions([j6]);
  const button = new MenuButton({ label: "Cameras" }, menu);
  document.body.append(button.el, menu.el);
  click(button.el);
  assert.equal(menu.open, true);
  assert.equal(button.el.getAttribute("aria-expanded"), "true");
  click(menu.items[0]!.el);
  assert.equal(j6.checked, false);
  assert.equal(menu.open, false);
  assert.equal(button.el.getAttribute("aria-expanded"), "false");
  menu.dispose();
  button.dispose();
});

// Choosing the robot model is this: the picker and the cell it has to agree
// with are in one menu, so a choice reads back where it was made.
test("a keepOpen item leaves the menu up, and a block in it takes typing", () => {
  const rosie = new Action({ id: "robot-model:rosie_1400_v3", label: "rosie_1400_v3", checkable: true });
  const menu = new Menu("Robot");
  menu.addAction(rosie, { keepOpen: true });
  const field = document.createElement("input");
  const adopt = document.createElement("button");
  adopt.disabled = true;
  field.addEventListener("input", () => {
    adopt.disabled = field.value !== "bcc393bae0fe";
  });
  const block = document.createElement("div");
  block.append(field, adopt);
  const button = new MenuButton({ label: "Robot" }, menu);
  document.body.append(button.el, menu.el);
  click(button.el);
  menu.addBlock(block);
  click(menu.items[0]!.el);
  assert.equal(rosie.checked, true);
  assert.equal(menu.open, true, "the menu stays up so the choice can be read back");
  // The block is the menu's own markup, and typing into it is not a choice.
  field.value = "bcc393bae0fe";
  field.dispatchEvent(new browser.Event("input", { bubbles: true }) as unknown as Event);
  assert.equal(adopt.disabled, false);
  assert.equal(menu.open, true);
  assert.ok(menu.el.querySelector(".menu-block")!.contains(field));
  menu.dispose();
  button.dispose();
});

test("shortcuts trigger actions except while typing in a field", () => {
  const action = new Action({ id: "move", label: "Move", shortcut: "M", checkable: true });
  const shortcuts = new Shortcuts(document as unknown as Document);
  shortcuts.bind(action);
  const press = (target: HTMLElement): void => {
    target.dispatchEvent(new browser.KeyboardEvent("keydown", { key: "m", bubbles: true }) as unknown as KeyboardEvent);
  };
  press(document.body as unknown as HTMLElement);
  assert.equal(action.checked, true);
  const input = document.createElement("input");
  document.body.append(input);
  press(input as unknown as HTMLElement);
  assert.equal(action.checked, true, "a letter typed into a field is not a shortcut");
  shortcuts.dispose();
});

test("a submenu opens beside its item, keeps its radios, and closes with its parent", () => {
  const parent = new Menu("Program").mount(document.body);
  const sub = new Menu("Theme");
  const group = new ActionGroup();
  const light = new Action({ id: "light", label: "Light", group, checked: true });
  const dark = new Action({ id: "dark", label: "Dark", group });
  sub.addAction(light, { keepOpen: true });
  sub.addAction(dark, { keepOpen: true });
  const other = new Action({ id: "export", label: "Export" });
  const build = (): HTMLButtonElement => {
    parent.clear();
    parent.addAction(other);
    return parent.addSubmenu("Theme", sub);
  };
  parent.beforeOpen = () => { build(); };
  parent.openAt(10, 10);
  const opener = parent.el.querySelector<HTMLButtonElement>(".menu-item-submenu")!;
  assert.equal(opener.querySelector(".menu-item-label")!.textContent, "Theme");
  assert.equal(opener.querySelector(".menu-item-chevron")!.textContent, "›");
  // The submenu lives inside the parent, so a click in it is inside the parent.
  assert.equal(sub.el.parentElement, parent.el);
  opener.dispatchEvent(new browser.Event("pointerenter") as unknown as Event);
  assert.equal(sub.open, true);
  assert.equal(opener.getAttribute("aria-expanded"), "true");
  click(sub.items[1]!.el);
  assert.equal(dark.checked, true);
  assert.equal(sub.open, true, "a keepOpen radio leaves the submenu open");
  // The pointer on another of the parent's items folds the submenu.
  parent.items[0]!.el.dispatchEvent(new browser.Event("pointerover", { bubbles: true }) as unknown as Event);
  assert.equal(sub.open, false);
  assert.equal(opener.getAttribute("aria-expanded"), "false");
  click(opener);
  assert.equal(sub.open, true);
  parent.close();
  assert.equal(sub.open, false, "closing the parent closes the submenu");
  // Rebuilt on the next open, the same submenu is reused.
  parent.openAt(10, 10);
  assert.equal(parent.el.querySelectorAll(".menu-item-submenu").length, 1);
  assert.equal(sub.el.parentElement, parent.el);
  parent.dispose();
});
