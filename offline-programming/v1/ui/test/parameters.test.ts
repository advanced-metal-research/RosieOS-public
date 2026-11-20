import assert from "node:assert/strict";
import test from "node:test";

import {
  completeValues,
  createParameterSet,
  validate,
  type ParameterPreset,
  type ParameterSchema,
} from "../src/parameters/model.ts";
import {
  createPresetLibrary,
  memoryPresetStore,
  type PresetStore,
} from "../src/parameters/library.ts";

const SCHEMA: ParameterSchema = [
  { key: "work_deg", label: "Work", kind: "number", unit: "°", min: 0, max: 90, step: 1 },
  { key: "cost", label: "Cost", kind: "number", unit: "°/mm", min: 0.1, max: 30, step: 0.1 },
  {
    key: "mode",
    label: "Mode",
    kind: "choice",
    options: [{ value: "fast", label: "Fast" }, { value: "thorough", label: "Thorough" }],
  },
  { key: "reverse", label: "Also reverse", kind: "toggle" },
];

const STEADY: ParameterPreset = {
  id: "steady",
  label: "Steady",
  origin: "shipped",
  values: { work_deg: 15, cost: 3, mode: "fast", reverse: true },
};

function set(initial?: Record<string, number | string | boolean>) {
  return createParameterSet(SCHEMA, STEADY, { initial });
}

test("a fresh set holds the preset's values and nothing reads as modified", () => {
  const parameters = set();
  assert.deepEqual(parameters.values(), { work_deg: 15, cost: 3, mode: "fast", reverse: true });
  assert.equal(parameters.modified, false);
  assert.deepEqual(parameters.modifiedKeys(), []);
});

test("modified is a comparison against the preset, not a record of edits", () => {
  // Typing the preset's own number in by hand is not a modification: the
  // question the window answers is how these differ from the preset.
  const parameters = set();
  parameters.set("work_deg", 20);
  assert.deepEqual(parameters.modifiedKeys(), ["work_deg"]);
  parameters.set("work_deg", 15);
  assert.equal(parameters.modified, false);
});

test("modifiedKeys comes back in schema order, not edit order", () => {
  const parameters = set();
  parameters.set("mode", "thorough");
  parameters.set("work_deg", 20);
  assert.deepEqual(parameters.modifiedKeys(), ["work_deg", "mode"]);
});

test("an out-of-range number is refused and the old value stands", () => {
  // Clamping would leave the operator believing they set one number while
  // the request carried another.
  const parameters = set();
  const outcome = parameters.set("work_deg", 200);
  assert.equal(outcome.ok, false);
  assert.match(outcome.ok === false ? outcome.reason : "", /above the maximum, 90 °/);
  assert.equal(parameters.get("work_deg"), 15);
  assert.equal(parameters.modified, false);
});

test("a value that is not a number at all is refused", () => {
  const parameters = set();
  assert.equal(parameters.set("cost", Number.NaN).ok, false);
  assert.equal(parameters.set("cost", "3" as unknown as number).ok, false);
  assert.equal(parameters.get("cost"), 3);
});

test("a choice outside the options and a non-boolean toggle are refused", () => {
  const parameters = set();
  assert.equal(parameters.set("mode", "sideways").ok, false);
  assert.equal(parameters.set("reverse", 1 as unknown as boolean).ok, false);
  assert.equal(parameters.get("mode"), "fast");
  assert.equal(parameters.get("reverse"), true);
});

test("revert puts one field back and leaves the others", () => {
  const parameters = set();
  parameters.set("work_deg", 40);
  parameters.set("cost", 9);
  parameters.revert("work_deg");
  assert.equal(parameters.get("work_deg"), 15);
  assert.equal(parameters.get("cost"), 9);
  assert.deepEqual(parameters.modifiedKeys(), ["cost"]);
});

test("revertAll puts every field back", () => {
  const parameters = set();
  parameters.set("work_deg", 40);
  parameters.set("mode", "thorough");
  parameters.set("reverse", false);
  parameters.revertAll();
  assert.deepEqual(parameters.values(), STEADY.values);
  assert.equal(parameters.modified, false);
});

test("loading another preset takes its values whole", () => {
  const parameters = set();
  parameters.set("work_deg", 40);
  parameters.loadPreset({
    id: "wide", label: "Wide", origin: "user",
    values: { work_deg: 30, cost: 1, mode: "thorough", reverse: false },
  });
  assert.equal(parameters.preset.id, "wide");
  assert.equal(parameters.get("work_deg"), 30);
  assert.equal(parameters.modified, false);
});

test("rebase moves the comparison without moving the values", () => {
  // What Save does: the numbers on screen are now the preset's, so nothing
  // reads as modified, but nothing on screen changed either.
  const parameters = set();
  parameters.set("work_deg", 40);
  const values = parameters.values();
  parameters.rebase({ id: "mine", label: "Mine", origin: "user", values });
  assert.deepEqual(parameters.values(), values);
  assert.equal(parameters.modified, false);
  assert.equal(parameters.preset.label, "Mine");
});

test("a preset saved before a field existed does not read as a hole", () => {
  const parameters = createParameterSet(SCHEMA, {
    id: "old", label: "Old", origin: "user", values: { work_deg: 22 },
  });
  // The field the old preset never heard of falls back to the schema's own
  // default rather than to undefined.
  assert.equal(parameters.get("work_deg"), 22);
  assert.equal(parameters.get("mode"), "fast");
  assert.equal(parameters.get("reverse"), false);
  assert.equal(parameters.get("cost"), 0.1);
});

test("completeValues prefers the values, then the fallback, then the schema", () => {
  const filled = completeValues(SCHEMA, { work_deg: 5 }, { cost: 7 });
  assert.equal(filled.work_deg, 5);
  assert.equal(filled.cost, 7);
  assert.equal(filled.mode, "fast");
});

test("constraints answer across fields and are advisory", () => {
  // Refusing the first edit of a pair would make the second unreachable.
  const parameters = createParameterSet(SCHEMA, STEADY, {
    constraints: (values) =>
      Number(values.work_deg) > Number(values.cost) * 10 ? ["work angle outruns the cost"] : [],
  });
  assert.deepEqual(parameters.problems(), []);
  assert.equal(parameters.set("cost", 0.5).ok, true, "the edit is still applied");
  assert.deepEqual(parameters.problems(), ["work angle outruns the cost"]);
});

test("validate is the same answer the set gives", () => {
  const field = SCHEMA[0]!;
  assert.equal(validate(field, 15).ok, true);
  assert.equal(validate(field, -1).ok, false);
  assert.equal(set().set("work_deg", -1).ok, false);
});

// ---- the library --------------------------------------------------------

function library(store: PresetStore = memoryPresetStore()) {
  return createPresetLibrary({
    scope: "test.scope", schema: SCHEMA, shipped: [STEADY], store,
  });
}

test("a library opens against what it ships", async () => {
  const presets = library();
  assert.equal(presets.fallback.id, "steady");
  assert.deepEqual(presets.all().map((preset) => preset.id), ["steady"]);
});

test("a shipped preset refuses Save, and says to use Save as", async () => {
  // It is the thing "revert to preset" reverts TO. Overwriting it would
  // remove the only fixed point in the window.
  const presets = library();
  const outcome = await presets.save("steady", { work_deg: 40 });
  assert.equal(outcome.ok, false);
  assert.match(outcome.ok === false ? outcome.reason : "", /shipped preset.*Save as/);
  assert.equal(presets.get("steady")!.values.work_deg, 15);
});

test("Save as keeps the values and the new preset is the operator's", async () => {
  const presets = library();
  const outcome = await presets.saveAs("Thin plate", { work_deg: 40, cost: 8, mode: "thorough", reverse: false });
  assert.equal(outcome.ok, true);
  const saved = outcome.ok ? outcome.preset : null;
  assert.equal(saved!.id, "thin_plate");
  assert.equal(saved!.origin, "user");
  assert.equal(saved!.values.work_deg, 40);
  assert.deepEqual(presets.all().map((preset) => preset.id), ["steady", "thin_plate"]);
});

test("shipped presets sort first, the operator's by label after them", async () => {
  const presets = library();
  await presets.saveAs("Zinc", { work_deg: 1 });
  await presets.saveAs("Aluminium", { work_deg: 2 });
  assert.deepEqual(presets.all().map((preset) => preset.label), ["Steady", "Aluminium", "Zinc"]);
});

test("Save as under an existing name of the operator's own overwrites it", async () => {
  const presets = library();
  await presets.saveAs("Thin plate", { work_deg: 40 });
  await presets.saveAs("thin PLATE", { work_deg: 41 });
  assert.equal(presets.all().length, 2, "not a second preset with the same name");
  assert.equal(presets.get("thin_plate")!.values.work_deg, 41);
});

test("Save as cannot take a shipped preset's name", async () => {
  const presets = library();
  const outcome = await presets.saveAs("Steady", { work_deg: 40 });
  assert.equal(outcome.ok, false);
  assert.equal(presets.get("steady")!.values.work_deg, 15);
});

test("a preset needs a name", async () => {
  const outcome = await library().saveAs("   ", { work_deg: 40 });
  assert.equal(outcome.ok, false);
  assert.match(outcome.ok === false ? outcome.reason : "", /needs a name/);
});

test("Save over the operator's own preset rewrites it", async () => {
  const presets = library();
  const created = await presets.saveAs("Thin plate", { work_deg: 40 });
  assert.equal(created.ok, true);
  const saved = await presets.save("thin_plate", { work_deg: 44, cost: 3, mode: "fast", reverse: true });
  assert.equal(saved.ok, true);
  assert.equal(presets.get("thin_plate")!.values.work_deg, 44);
});

test("a store that refuses is reported, not swallowed", async () => {
  // A Save that reported success and kept nothing is how an operator loses a
  // procedure they believe is written down.
  const presets = library({
    list: async () => [],
    put: async () => {
      throw new Error("quota exceeded");
    },
  });
  const outcome = await presets.saveAs("Thin plate", { work_deg: 40 });
  assert.equal(outcome.ok, false);
  assert.match(outcome.ok === false ? outcome.reason : "", /quota exceeded/);
  assert.equal(presets.all().length, 1, "nothing was added on a failed write");
});

test("an unreadable store leaves a working window", async () => {
  const presets = library({
    list: async () => {
      throw new Error("IndexedDB is gone");
    },
    put: async () => {},
  });
  await presets.load();
  assert.deepEqual(presets.all().map((preset) => preset.id), ["steady"]);
});

test("saved presets come back on the next load, scoped", async () => {
  const store = memoryPresetStore();
  await library(store).saveAs("Thin plate", { work_deg: 40 });
  const reopened = library(store);
  await reopened.load();
  assert.deepEqual(reopened.all().map((preset) => preset.id), ["steady", "thin_plate"]);

  const other = createPresetLibrary({
    scope: "other.scope", schema: SCHEMA, shipped: [STEADY], store,
  });
  await other.load();
  assert.deepEqual(other.all().map((preset) => preset.id), ["steady"], "scopes do not see each other");
});

test("saving over a preset fills in fields it predates", async () => {
  const store = memoryPresetStore();
  await store.put("test.scope", {
    id: "old", label: "Old", origin: "user", values: { work_deg: 22 },
  });
  const presets = library(store);
  await presets.load();
  const saved = await presets.save("old", { work_deg: 30, cost: 2, mode: "thorough", reverse: false });
  assert.equal(saved.ok, true);
  assert.deepEqual(presets.get("old")!.values, {
    work_deg: 30, cost: 2, mode: "thorough", reverse: false,
  });
});
