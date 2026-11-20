import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { seedRobotCatalogue } from "../../../../offline-programming/v1/ui/test/support/robot-catalogue.ts";
import { documentProblems } from "../../../../offline-programming/v1/ui/src/program-v2/from-project.ts";
import * as session from "./session.ts";

const pose = {
  frame_id: "world" as const, xyz_m: [1.6, .7, .9] as [number, number, number],
  rpy_rad: [Math.PI, .6, 3.1] as [number, number, number],
  joint_names: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"],
  joint_values_rad: [0, .1, .2, .3, .4, .5, 0, 0, 0],
};
function setup() {
  seedRobotCatalogue();
  session.newProject("View cache", "rosie_1400_v3");
  const capture = {
    source: "machine" as const, observed_at: "2026-10-27T00:00:00Z", model_id: "rosie_1400_v3",
    robot_description_sha256: session.view().description!, cell_id: "local-simulation", pose,
  };
  session.record("new", pose, capture, "first");
  session.record("new", pose, capture, "second");
}

test("status/selection reads reuse derived rows; edits, undo and redo refresh values and validation", () => {
  setup();
  const original = session.view();
  for (let i = 0; i < 1000; ++i) {
    session.select(i % 2 ? "first" : "second");
    const next = session.view();
    assert.equal(next.rows, original.rows);
    assert.equal(next.problems, original.problems);
    assert.equal(next.selected, i % 2 ? "first" : "second");
  }
  session.edit("second", "motion", "circular");
  const invalid = session.view();
  assert.notEqual(invalid.rows, original.rows);
  assert.match(invalid.rows[1]!.status, /Record via required/);
  assert.deepEqual(invalid.problems, documentProblems(session.currentProject()));
  assert.notEqual(invalid.problems, original.problems);
  session.undo();
  const undone = session.view();
  assert.deepEqual(undone.rows, original.rows);
  assert.deepEqual(undone.problems, original.problems);
  assert.equal(undone.canRedo, true);
  session.redo();
  assert.deepEqual(session.view().rows, invalid.rows);
  assert.equal(session.view().canRedo, false);
  const malformed = structuredClone(session.currentProject());
  assert.equal(malformed.nodes[0]!.op, "move");
  if (malformed.nodes[0]!.op === "move") malformed.nodes[0]!.speed_mm_s = -5;
  session.commitProject(malformed);
  assert.notDeepEqual(session.view().problems, invalid.problems);
  assert.deepEqual(session.view().problems, documentProblems(malformed));
});

test("same-revision replacements, catalogue refreshes and new/open programs cannot retain old derived rows", () => {
  setup();
  const original = session.view();
  const replacement = structuredClone(session.currentProject());
  replacement.nodes[0]!.label = "Replacement";
  // Plans can replace a project without incrementing its authoring revision.
  session.commitProject(replacement);
  assert.equal(session.view().revision, original.revision);
  assert.equal(session.view().rows[0]!.label, "Replacement");
  const kept = session.view();
  session.setRobots(JSON.parse(readFileSync(new URL("../tests/fixtures/robots.json", import.meta.url), "utf8")), null);
  assert.notEqual(session.view().rows, kept.rows);
  assert.deepEqual(session.view().problems, documentProblems(session.currentProject()));
  const saved = session.exportProject();
  session.newProject("Empty", "rosie_1400_v3");
  assert.equal(session.view().rows.length, 0);
  assert.equal(session.view().canUndo, false);
  session.openProject(saved);
  assert.equal(session.view().rows[0]!.label, "Replacement");
  assert.equal(session.view().canUndo, false);
});
