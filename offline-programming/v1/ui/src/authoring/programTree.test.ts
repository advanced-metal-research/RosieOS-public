import assert from "node:assert/strict";
import test from "node:test";

import { createProject } from "../domain/project.ts";
import { appendCadQueryTopology } from "../adapters/cadquery.ts";
import { createWeldsFromRecommendations } from "../adapters/weld-authoring.ts";
import { activeWorkpieceId, projectWorkpieces } from "../domain/workpieces.ts";
import { workpieceTopology } from "../../test/support/workpiece-fixtures.ts";
import { programTreeNodes, TREE_FIXTURE_PREFIX, TREE_PART, TREE_WORKSPACE, TREE_WORKPIECE_PREFIX, TREE_WORKPIECE_WELD_PREFIX } from "./programTree.ts";

test("an importing part shows its filename and busy state without changing the document", () => {
  const project = createProject({ robotModel: "rosie_1400_v3" });
  const before = JSON.stringify(project);
  const options = { onImportPart: () => undefined };
  const part = programTreeNodes(project, { ...options, importingFilename: "bracket.step" })[0]!
    .children!.find(node => node.id === "importing-part")!;
  assert.equal(part.label, "bracket.step");
  assert.equal(part.busy, true);
  assert.equal(part.selectable, false);
  assert.equal(part.hint, undefined);
  assert.equal(JSON.stringify(project), before);
  const restored = programTreeNodes(project, options)[0]!.children!.find(node => node.id === TREE_PART)!;
  assert.equal(restored.busy, undefined);
  assert.equal(restored.label, "No part");
  assert.equal(restored.hint?.label, "Import a part");
});

// The stand is the cell's, so it sits under Workspace, whatever part is
// loaded; a cell with none offers to add one (owner, 2026-09-20: the planner's
// invisible placeholder must be visible here and editable).
test("the workspace row lists the cell's stand, or offers one", () => {
  const options = { onImportPart: () => undefined, onAddStand: () => undefined };
  const tree = programTreeNodes(createProject({ robotModel: "rosie_1420_v1" }), options);
  const workspace = tree[0]!.children!.find((node) => node.id === TREE_WORKSPACE)!;
  assert.equal(workspace.children!.length, 1);
  assert.equal(workspace.children![0]!.id, TREE_FIXTURE_PREFIX + "stand");
  assert.equal(workspace.children![0]!.label, "Stand");
  assert.equal(workspace.children![0]!.sublabel, "fixture · 500 × 500 × 1000 mm · at 1000, 1000, 0 mm · placeholder");

  const bare = programTreeNodes(createProject({ robotModel: "rosie_1400_v3" }), options);
  const slot = bare[0]!.children!.find((node) => node.id === TREE_WORKSPACE)!.children![0]!;
  assert.equal(slot.label, "No stand");
  assert.equal(slot.muted, true);
  assert.equal(slot.hint?.label, "Add a stand");
});

test("each imported weld part stays in the tree with its own welds and unique selections", () => {
  const first = createWeldsFromRecommendations(appendCadQueryTopology(createProject(), workpieceTopology()));
  const firstId = activeWorkpieceId(first);
  const project = createWeldsFromRecommendations(appendCadQueryTopology(first, workpieceTopology()));
  const tree = programTreeNodes(project, { onImportPart: () => undefined })[0]!.children!;
  const inactive = tree.find(node => node.id === TREE_WORKPIECE_PREFIX + firstId)!;
  const active = tree.find(node => node.id === TREE_PART)!;
  assert.equal(inactive.label, "bracket.step");
  assert.equal(active.label, "bracket.step");
  assert.match(active.sublabel!, /active weld part/);
  assert.ok(inactive.children!.length > 0);
  assert.ok(active.children!.length > 0);
  assert.ok(inactive.children!.every(node => node.id.startsWith(TREE_WORKPIECE_WELD_PREFIX)));
  const ids = [...inactive.children!, ...active.children!].map(node => node.id);
  assert.equal(new Set(ids).size, ids.length);
});

test("an added part's pending row preserves existing parts and fixture imports appear under Workspace", () => {
  const project = appendCadQueryTopology(createProject({ robotModel: "rosie_1420_v1" }), workpieceTopology());
  const options = { onImportPart: () => undefined, importingFilename: "added.step" };
  const parts = programTreeNodes(project, options)[0]!.children!;
  assert.equal(parts.find(node => node.id === TREE_PART)!.label, "bracket.step");
  assert.equal(parts.find(node => node.id === "importing-part")!.busy, true);
  const fixtureTree = programTreeNodes(project, { ...options, importingKind: "fixture" })[0]!.children!;
  assert.equal(fixtureTree.some(node => node.id === "importing-part"), false);
  const fixtures = fixtureTree.find(node => node.id === TREE_WORKSPACE)!.children!;
  assert.equal(fixtures[0]!.label, "Stand");
  assert.equal(fixtures.find(node => node.id === "importing-part")!.label, "added.step");
  const completing = programTreeNodes(project, {
    ...options, importingWorkpieceId: projectWorkpieces(project)[0]!.id,
  })[0]!.children!;
  assert.equal(completing.find(node => node.id === TREE_PART)!.busy, true);
  assert.equal(completing.some(node => node.id === "importing-part"), false);
});
