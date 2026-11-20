import assert from "node:assert/strict";
import test from "node:test";

import {
  createDogBoxProject,
  migrateDogBoxPlannerDefaults,
  needsDogBoxPlannerDefaultsMigration,
} from "./dog-box.ts";

test("Dog Box defaults keep the tool in the authored downward hemisphere", () => {
  const project = createDogBoxProject("2026-07-31T00:00:00.000Z");
  assert.deepEqual(project.program.constraints.tool_axis_constraint, {
    tool_frame: "end_effector",
    tip_link: "tool0",
    world_frame: "world",
    tool_axis_xyz: [1, 0, 0],
    world_axis_xyz: [0, 0, -1],
    offset_reference_axis_xyz: [1, 0, 0],
    offset_deg: 0,
    tolerance_minus_deg: 90,
    tolerance_plus_deg: 90,
    weight: 0.5,
  });
});

test("persisted bundled Dog Box projects migrate from the rejected 45-degree fixture", () => {
  const project = createDogBoxProject("2026-07-31T00:00:00.000Z");
  const constraint = project.program.constraints.tool_axis_constraint as Record<string, unknown>;
  constraint.offset_deg = 45;
  constraint.tolerance_minus_deg = 20;
  constraint.tolerance_plus_deg = 20;

  assert.equal(needsDogBoxPlannerDefaultsMigration(project), true);
  assert.equal(migrateDogBoxPlannerDefaults(project), true);
  assert.equal(constraint.offset_deg, 45);
  assert.deepEqual(project.program.constraints.tool_axis_constraint, {
    ...constraint,
    offset_deg: 0,
    tolerance_minus_deg: 90,
    tolerance_plus_deg: 90,
  });
  assert.equal(needsDogBoxPlannerDefaultsMigration(project), false);
});

test("persisted bundled Dog Box projects migrate from the obsolete one-degree gate", () => {
  const project = createDogBoxProject("2026-07-31T00:00:00.000Z");
  const constraint = project.program.constraints.tool_axis_constraint as Record<string, unknown>;
  constraint.tolerance_minus_deg = 1;
  constraint.tolerance_plus_deg = 1;

  assert.equal(needsDogBoxPlannerDefaultsMigration(project), true);
  assert.equal(migrateDogBoxPlannerDefaults(project), true);
  assert.equal(
    (project.program.constraints.tool_axis_constraint as Record<string, unknown>)
      .tolerance_minus_deg,
    90,
  );
});

test("Dog Box migration never rewrites an imported customer program", () => {
  const project = createDogBoxProject("2026-07-31T00:00:00.000Z");
  project.cad.kind = "step";
  const constraint = project.program.constraints.tool_axis_constraint as Record<string, unknown>;
  constraint.offset_deg = 45;
  constraint.tolerance_minus_deg = 20;
  constraint.tolerance_plus_deg = 20;

  assert.equal(migrateDogBoxPlannerDefaults(project), false);
  assert.equal(constraint.offset_deg, 45);
});
