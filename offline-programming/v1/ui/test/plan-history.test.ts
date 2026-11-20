import assert from "node:assert/strict";
import test from "node:test";

import { previewTrajectory } from "../src/domain/playback.ts";
import { activatePlan, activePlanEntryId, deletePlan, PLAN_HISTORY_LIMIT, recordPlan } from "../src/domain/planHistory.ts";
import { installPreviewPlan, reviseProject, type DenseTrajectoryRecord, type OfflineProgrammingProject } from "../src/domain/project.ts";
import { exportProjectJSON, importProjectJSON } from "../src/domain/serialization.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

const NOW = "2026-09-16T12:00:00.000Z";

function denseRecord(digest: string, revision: number): DenseTrajectoryRecord {
  return {
    schema: "dense.v1", trajectoryDigest: `sha256:${digest}`, requestSha256: "r", planId: `plan-${digest}`,
    programId: "p", programDigest: "sha256:req", manifestRevision: 1, planRevision: 1, createdAt: NOW,
    dtS: 0.01, totalSampleCount: 1200, totalDurationS: 12, bytes: 100,
    segments: [{ index: 0, kind: "weld", sourceId: "s", sampleCount: 1200, durationS: 12 }],
    sourceProjectRevision: revision, sourceProgramDigest: "sha256:prog",
  };
}

test("every plan is recorded newest first, the active one is known, and an old one can be made active again", () => {
  let project: OfflineProgrammingProject = { ...createDogBoxProject(NOW), railsAuthoring: true };
  project.denseTrajectory = denseRecord("aaa", project.identity.revision);
  project = recordPlan(project);
  assert.equal(project.planHistory?.length, 1);
  assert.equal(activePlanEntryId(project), "plan:sha256:aaa");
  // A recompile with the same digest does not double.
  project = recordPlan(project);
  assert.equal(project.planHistory?.length, 1);
  // An edit stales the active plan; the next plan goes on top.
  project = reviseProject(project, "nodes", () => undefined, NOW);
  assert.equal(project.denseTrajectory, null);
  project.denseTrajectory = denseRecord("bbb", project.identity.revision);
  project = recordPlan(project);
  assert.deepEqual(project.planHistory?.map((entry) => entry.id), ["plan:sha256:bbb", "plan:sha256:aaa"]);
  assert.equal(activePlanEntryId(project), "plan:sha256:bbb");
  // Activating the old one puts it back in the slot; it reads as stale by revision.
  const back = activatePlan(project, "plan:sha256:aaa");
  assert.equal(back.denseTrajectory?.trajectoryDigest, "sha256:aaa");
  assert.equal(activePlanEntryId(back), "plan:sha256:aaa");
  assert.notEqual(back.denseTrajectory?.sourceProjectRevision, back.identity.revision);
  assert.equal(back.planHistory?.length, 2, "activating is not a new record");
  assert.equal(activatePlan(project, "plan:nope"), project);
  // The history survives export and import.
  const round = importProjectJSON(exportProjectJSON(back));
  assert.deepEqual(round.planHistory?.map((entry) => entry.id), ["plan:sha256:bbb", "plan:sha256:aaa"]);
});

test("forgetting the active plan leaves the program with none; forgetting another leaves it alone", () => {
  let project: OfflineProgrammingProject = { ...createDogBoxProject(NOW), railsAuthoring: true };
  project.denseTrajectory = denseRecord("aaa", project.identity.revision);
  project = recordPlan(project);
  project.denseTrajectory = denseRecord("bbb", project.identity.revision);
  project = recordPlan(project);
  const other = deletePlan(project, "plan:sha256:aaa");
  assert.equal(other.denseTrajectory?.trajectoryDigest, "sha256:bbb");
  assert.deepEqual(other.planHistory?.map((entry) => entry.id), ["plan:sha256:bbb"]);
  const active = deletePlan(project, "plan:sha256:bbb");
  assert.equal(active.denseTrajectory, null);
  assert.equal(active.plan.state, "missing");
  assert.deepEqual(active.planHistory?.map((entry) => entry.id), ["plan:sha256:aaa"]);
  assert.equal(deletePlan(project, "plan:nope"), project);
});

test("classic plans are recorded too, and the history is capped", () => {
  let project = createDogBoxProject(NOW);
  for (let index = 0; index < PLAN_HISTORY_LIMIT + 2; index += 1) {
    project = installPreviewPlan(project, previewTrajectory(project.rows), NOW);
    project = recordPlan(project);
    project = reviseProject(project, "seams", (draft) => {
      draft.seams[0]!.reversed = !draft.seams[0]!.reversed;
    }, NOW);
  }
  assert.equal(project.planHistory?.length, PLAN_HISTORY_LIMIT);
  assert.equal(project.planHistory?.[0]?.kind, "classic");
  assert.equal(project.planHistory?.[0]?.planner, "browser preview");
  assert.ok((project.planHistory?.[0]?.summary?.sampleCount ?? 0) > 0);
});
