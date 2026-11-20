import assert from "node:assert/strict";
import test from "node:test";
import { createDogBoxProject } from "../src/examples/dog-box.ts";
import {
  createBlankSpreadsheetProject,
  createSpreadsheetRow,
  deleteSpreadsheetRow,
  duplicateSpreadsheetRow,
  insertSpreadsheetRow,
  moveSpreadsheetRow,
  resolveTeachOverride,
  updateSpreadsheetRows,
} from "../src/domain/spreadsheet.ts";

test("new blank program is EMPTY — no rows, no points, no selection", () => {
  const project = createBlankSpreadsheetProject({
    name: "Bracket weld",
    robotModel: "rosie_1400_v3",
  });
  assert.equal(project.identity.name, "Bracket weld");
  assert.equal(project.program.programId, project.identity.id);
  assert.equal(project.program.programRevision, 1);
  assert.equal(project.program.programState, "draft");
  assert.equal(project.rows.length, 0);
  assert.equal(project.seams.length, 0);
  assert.equal(project.sequence.length, 0);
  assert.deepEqual(project.program.constraints, {
    tip_link: "tool0",
    tool_axis_constraint: {
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
    },
  });
  assert.equal(project.simulation.selectedRowId, null);
  assert.equal(project.plan.state, "missing");
});

test("spreadsheet insert, duplicate, edit, move, and delete preserve stable identities", () => {
  let project = createDogBoxProject();
  const initialCount = project.rows.length;
  const previousProgramRevision = project.program.programRevision;
  project = insertSpreadsheetRow(project, 1, "comment");
  const inserted = project.rows[1]!;
  assert.equal(project.rows.length, initialCount + 1);
  assert.equal(project.program.baseProgramRevision, 0);
  assert.equal(project.program.programRevision, previousProgramRevision + 1);

  project = duplicateSpreadsheetRow(project, inserted.id);
  const duplicate = project.rows[2]!;
  assert.notEqual(duplicate.id, inserted.id);
  assert.equal(duplicate.command, inserted.command);

  const stableID = duplicate.id;
  project = updateSpreadsheetRows(project, [stableID], (row) => {
    row.comment = "Edited as a spreadsheet cell";
  });
  assert.equal(project.rows.find((row) => row.id === stableID)?.comment, "Edited as a spreadsheet cell");
  assert.equal(project.rows.find((row) => row.id === stableID)?.rowRevision, project.program.programRevision);

  project = moveSpreadsheetRow(project, stableID, 1);
  assert.equal(project.rows[3]?.id, stableID);
  project = deleteSpreadsheetRow(project, stableID);
  assert.equal(project.rows.some((row) => row.id === stableID), false);
});

test("editing a weld row torch angle recomputes the outgoing TCP RPY", () => {
  let project = createDogBoxProject();
  const weld = project.rows.find((row) => row.phase === "weld");
  assert.ok(weld);
  const before = weld.targetTcpPose.slice(3);

  project = updateSpreadsheetRows(project, [weld.id], (row) => {
    row.torchWorkAngleDeg = 5;
    row.torchTravelAngleDeg = -12;
    row.torchSpinAngleDeg = 30;
  });

  const updated = project.rows.find((row) => row.id === weld.id);
  assert.ok(updated);
  assert.notDeepEqual(updated.targetTcpPose.slice(3), before);
  assert.deepEqual(updated.currentTcpPose?.slice(3), updated.targetTcpPose.slice(3));
  const approach = project.rows.find((row) =>
    row.seamId === updated.seamId && row.phase === "approach"
  );
  assert.ok(approach);
  assert.equal(approach.torchWorkAngleDeg, 5);
  assert.equal(approach.torchTravelAngleDeg, -12);
  assert.equal(approach.torchSpinAngleDeg, 30);
});

test("teach-modified CAD rows require an explicit keep or regenerate decision", () => {
  let project = createDogBoxProject();
  const rowID = project.rows[0]!.id;
  project.rows[0]!.sourceState = "teach_modified";
  const kept = resolveTeachOverride(project, rowID, "keep_taught");
  assert.equal(kept.rows[0]!.sourceState, "teach_modified");
  assert.equal(kept.rows[0]!.extensions?.teach_override_resolution, "keep_taught");

  const regenerated = createSpreadsheetRow(project, "motion", "replacement");
  regenerated.targetTcpPose[0] = 0.75;
  const replaced = resolveTeachOverride(project, rowID, "regenerate_from_cad", regenerated);
  assert.equal(replaced.rows[0]!.id, rowID);
  assert.equal(replaced.rows[0]!.sourceState, "cad_derived");
  assert.equal(replaced.rows[0]!.targetTcpPose[0], 0.75);
});

test("editing a seam neighbor never rewrites a teach-modified row pose", () => {
  let project = createDogBoxProject();
  const weldRows = project.rows.filter((row) => row.phase === "weld");
  const edited = weldRows[0]!;
  const taught = weldRows[1]!;
  project = updateSpreadsheetRows(project, [taught.id], (row) => {
    row.sourceState = "teach_modified";
    row.targetTcpPose = [0.71, -0.22, 0.63, 1.111, 2.222, 3.333];
    row.currentTcpPose = [0.7, -0.2, 0.6, -1.111, -2.222, -3.333];
  });
  const before = structuredClone(project.rows.find((row) => row.id === taught.id)!);

  project = updateSpreadsheetRows(project, [edited.id], (row) => {
    row.torchSpinAngleDeg = 37;
  });
  const after = project.rows.find((row) => row.id === taught.id)!;
  assert.deepEqual(after.targetTcpPose, before.targetTcpPose);
  assert.deepEqual(after.currentTcpPose, before.currentTcpPose);
});

test("deleting the final program row leaves a legal empty program", () => {
  const project = createDogBoxProject();
  project.rows = [project.rows[0]!];
  const emptied = deleteSpreadsheetRow(project, project.rows[0]!.id);
  assert.equal(emptied.rows.length, 0);
  assert.equal(emptied.simulation.selectedRowId, null);
});
