import assert from "node:assert/strict";
import test from "node:test";

import { ProjectHistory } from "../src/domain/history.ts";
import { installPreviewPlan, reviseProject } from "../src/domain/project.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

test("undo and redo restore content as new monotonic revisions and stale the plan", () => {
  const history = new ProjectHistory();
  const base = createDogBoxProject("2026-07-30T00:00:00.000Z");
  const planned = installPreviewPlan(base, [], "2026-07-30T00:00:10.000Z");
  history.push(planned);
  const edited = reviseProject(planned, "seams", (draft) => {
    draft.seams[0]!.label = "Changed";
  }, "2026-07-30T00:01:00.000Z");

  const undone = history.undo(edited, "2026-07-30T00:02:00.000Z");
  assert.equal(undone.seams[0]!.label, planned.seams[0]!.label);
  assert.equal(undone.identity.revision, edited.identity.revision + 1);
  assert.equal(undone.plan.state, "stale");
  assert.equal(history.canRedo, true);

  const redone = history.redo(undone, "2026-07-30T00:03:00.000Z");
  assert.equal(redone.seams[0]!.label, "Changed");
  assert.equal(redone.identity.revision, undone.identity.revision + 1);
  assert.equal(redone.plan.state, "stale");
});

test("new edits clear the redo branch", () => {
  const history = new ProjectHistory();
  const base = createDogBoxProject();
  history.push(base);
  const changed = reviseProject(base, "metadata", (draft) => { draft.identity.name = "A"; });
  history.undo(changed);
  assert.equal(history.canRedo, true);
  history.push(base);
  assert.equal(history.canRedo, false);
});

