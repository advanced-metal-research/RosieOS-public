import assert from "node:assert/strict";
import test from "node:test";
import { bindProgramNameInput } from "./programName.ts";

test("inspector program name commits on Enter without blur, and normal change still commits", () => {
  const input = Object.assign(new EventTarget(), { value: "Taught simulation" });
  const names: string[] = [];
  bindProgramNameInput(input as HTMLInputElement, name => names.push(name));
  const key = (key: string, isComposing = false) => Object.assign(new Event("keydown", { cancelable: true }), { key, isComposing });
  const enter = key("Enter"); input.dispatchEvent(enter);
  assert.equal(enter.defaultPrevented, true);
  assert.deepEqual(names, ["Taught simulation"]);
  input.value = "Second name"; input.dispatchEvent(new Event("change"));
  assert.deepEqual(names, ["Taught simulation", "Second name"]);
  input.dispatchEvent(key("Escape")); input.dispatchEvent(key("Enter", true));
  assert.equal(names.length, 2);
});
