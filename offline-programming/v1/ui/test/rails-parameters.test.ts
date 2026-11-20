import assert from "node:assert/strict";
import test from "node:test";

import {
  SEARCH_SCHEMA,
  SHIPPED_SEARCH_PRESETS,
  SHIPPED_TORCH_PRESETS,
  TORCH_SCHEMA,
  torchProblems,
  torchRequest,
} from "../src/rails/parameters.ts";
import { MIG_GAS_NOZZLE_16, ROSIE_LINK6_GUN, STEADY_SEARCH } from "../src/rails/presets.ts";
import { createParameterSet, validate } from "../src/parameters/model.ts";

test("the shipped presets are exactly the values presets.ts holds", () => {
  // Two files, one set of numbers: the schema describes the form, presets.ts
  // holds what goes in it, and a drift between them would ship a window whose
  // defaults are not the defaults.
  const search = SHIPPED_SEARCH_PRESETS.find((preset) => preset.id === STEADY_SEARCH.id)!;
  assert.equal(search.values.work_deviation_deg, STEADY_SEARCH.work_deviation_deg);
  assert.equal(search.values.travel_deviation_deg, STEADY_SEARCH.travel_deviation_deg);
  assert.equal(search.values.standoff_cost_deg_per_mm, STEADY_SEARCH.standoff_cost_deg_per_mm);

  for (const gun of [ROSIE_LINK6_GUN, MIG_GAS_NOZZLE_16]) {
    const torch = SHIPPED_TORCH_PRESETS.find((preset) => preset.id === gun.id)!;
    assert.equal(torch.values.body_radius_mm, gun.body_radius_mm);
    assert.equal(torch.values.body_length_mm, gun.body_length_mm);
    assert.equal(torch.values.standoff_min_mm, gun.standoff_range_mm.min);
    assert.equal(torch.values.standoff_max_mm, gun.standoff_range_mm.max);
  }
});

test("rails authors against the gun on the cell, not the nozzle", () => {
  // Measured on bracket_b_2x: authored against the Ø16 nozzle the seam search
  // kept full spans and bent the travel angle into the corners, and the motion
  // planner crossed neither seam; against this Ø32 gun it reproduced the seams
  // that ran on the motors and both crossed. The first shipped preset is the
  // one a window opens on.
  assert.equal(SHIPPED_TORCH_PRESETS[0]!.id, ROSIE_LINK6_GUN.id);
  assert.equal(SHIPPED_TORCH_PRESETS[0]!.values.body_radius_mm, 16);
});

test("every shipped value passes its own field's bounds", () => {
  for (const [schema, presets] of [
    [SEARCH_SCHEMA, SHIPPED_SEARCH_PRESETS],
    [TORCH_SCHEMA, SHIPPED_TORCH_PRESETS],
  ] as const) {
    for (const preset of presets) {
      for (const field of schema) {
        const outcome = validate(field, preset.values[field.key]!);
        assert.equal(
          outcome.ok, true,
          `${preset.id}.${field.key}: ${outcome.ok ? "" : outcome.reason}`,
        );
      }
    }
  }
});

test("the shipped torch is coherent across its fields", () => {
  const torch = SHIPPED_TORCH_PRESETS[0]!;
  assert.deepEqual(torchProblems({ ...torch.values }), []);
});

test("a barrel shorter than the furthest standoff is named as the problem", () => {
  // What `reach/tooling.py` refuses on the other side, said before the
  // request rather than after it.
  const problems = torchProblems({
    body_radius_mm: 8, body_length_mm: 20,
    standoff_min_mm: 6, standoff_max_mm: 25, default_standoff_mm: 10,
  });
  assert.equal(problems.length, 1);
  assert.match(problems[0]!, /barrel has to be longer/);
});

test("a crossed range and a default outside it are both named", () => {
  const problems = torchProblems({
    body_radius_mm: 8, body_length_mm: 150,
    standoff_min_mm: 30, standoff_max_mm: 10, default_standoff_mm: 50,
  });
  assert.equal(problems.length, 2);
  assert.match(problems.join(" "), /range crosses/);
  assert.match(problems.join(" "), /outside the range/);
});

test("constraints do not block the first edit of a pair", () => {
  // Widening a range means editing two numbers; refusing the first would
  // make the second unreachable.
  const parameters = createParameterSet(TORCH_SCHEMA, SHIPPED_TORCH_PRESETS[0]!, {
    constraints: torchProblems,
  });
  // Fitting a stubby gun: the barrel goes in first and is briefly shorter
  // than the standoff it is held at.
  assert.equal(parameters.set("body_length_mm", 20).ok, true);
  assert.match(parameters.problems().join(" "), /barrel has to be longer/);
  // Still above the 17 mm default standoff, so only the barrel was wrong.
  assert.equal(parameters.set("standoff_max_mm", 18).ok, true);
  assert.deepEqual(parameters.problems(), []);
});

test("the torch request is the shape the seam worker reads", () => {
  const parameters = createParameterSet(TORCH_SCHEMA, SHIPPED_TORCH_PRESETS[0]!);
  const request = torchRequest(parameters.values(), { id: "gun", label: "Gun" });
  assert.deepEqual(request, {
    id: "gun",
    label: "Gun",
    body_radius_mm: ROSIE_LINK6_GUN.body_radius_mm,
    body_length_mm: ROSIE_LINK6_GUN.body_length_mm,
    standoff_range_mm: {
      min: ROSIE_LINK6_GUN.standoff_range_mm.min,
      max: ROSIE_LINK6_GUN.standoff_range_mm.max,
    },
  });
  // `default_standoff_mm` is deliberately NOT in here: the worker takes it as
  // its own request field, beside the torch rather than inside it.
  assert.equal("default_standoff_mm" in request, false);
});

test("every field carries a label and a help line", () => {
  // These reach a torch. A number with no sentence saying what it does is a
  // number an operator will guess at.
  for (const field of [...SEARCH_SCHEMA, ...TORCH_SCHEMA]) {
    assert.ok(field.label.length > 0, `${field.key} has no label`);
    assert.ok((field.help ?? "").length > 20, `${field.key} has no help`);
    if (field.kind === "number") {
      assert.ok(field.unit, `${field.key} has no unit`);
      assert.ok(field.min !== undefined && field.max !== undefined, `${field.key} is unbounded`);
    }
  }
});
