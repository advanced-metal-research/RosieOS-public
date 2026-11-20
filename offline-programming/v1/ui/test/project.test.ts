import assert from "node:assert/strict";
import test from "node:test";

import { previewTrajectory } from "../src/domain/playback.ts";
import {
  cellSettingsKey,
  emptyEvidence,
  executionReadiness,
  installPreviewPlan,
  installServerValidatedPlan,
  planIsFresh,
  reviseProject,
} from "../src/domain/project.ts";
import {
  exportProjectJSON,
  importProjectJSON,
  migrateProject,
} from "../src/domain/serialization.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

const NOW = "2026-07-30T00:00:00.000Z";

test("authoring revisions are monotonic and invalidate preview plans", () => {
  const base = createDogBoxProject(NOW);
  const preview = installPreviewPlan(base, previewTrajectory(base.rows), NOW);
  assert.equal(preview.plan.state, "preview");
  assert.equal(planIsFresh(preview), true);

  const edited = reviseProject(preview, "seams", (draft) => {
    draft.seams[0]!.reversed = true;
  }, "2026-07-30T00:01:00.000Z");
  assert.equal(edited.identity.revision, preview.identity.revision + 1);
  assert.equal(edited.plan.state, "stale");
  assert.equal(edited.plan.evidence.runtimePlayable, false);
  assert.equal(planIsFresh(edited), false);
});

test("a plan records the cell settings it was made for and is stale once they change", () => {
  const base = createDogBoxProject(NOW);
  const benchCell = {
    cell: "bench-relay-dense", jetsonId: "bench-relay", robotModelId: "rosie_1400_v3",
    runtimeMode: "sim", mappedAxisMask: 0x3f, rtcoreHost: "bench-servo", pulledAt: NOW,
  };
  const pulled = reviseProject(base, "robot", (draft) => {
    draft.robot.cell = benchCell;
  }, NOW);
  const planned = installPreviewPlan(pulled, previewTrajectory(pulled.rows), NOW);
  assert.equal(planned.plan.sourceCell, cellSettingsKey(benchCell));
  assert.equal(planIsFresh(planned), true);
  // A plan from before the Robot menu carries no key and stays fresh on
  // a program that never pulled settings.
  const legacy = installPreviewPlan(base, previewTrajectory(base.rows), NOW);
  delete legacy.plan.sourceCell;
  assert.equal(planIsFresh(legacy), true);
  // The same cell pulled again with the same settings is not a change.
  assert.equal(cellSettingsKey({ ...benchCell, pulledAt: "2026-07-31T00:00:00.000Z", runtimeMode: "live" }), cellSettingsKey(benchCell));
  // Native configuration and backend pins participate in the same freshness key.
  assert.notEqual(cellSettingsKey({ ...benchCell, configurationDigest: "a".repeat(64) }), cellSettingsKey({ ...benchCell, configurationDigest: "b".repeat(64) }));
  assert.notEqual(cellSettingsKey({ ...benchCell, backend: "simulation" }), cellSettingsKey({ ...benchCell, backend: "ethercat" }));
  // Another cell, or another axis map, is.
  assert.notEqual(cellSettingsKey({ ...benchCell, cell: "dev-cell" }), cellSettingsKey(benchCell));
  assert.notEqual(cellSettingsKey({ ...benchCell, mappedAxisMask: 0x1ff }), cellSettingsKey(benchCell));
  const repointed = reviseProject(planned, "robot", (draft) => {
    draft.robot.cell = { ...benchCell, cell: "dev-cell" };
  }, "2026-07-30T00:01:00.000Z");
  assert.equal(repointed.plan.state, "stale");
  assert.equal(planIsFresh(repointed), false);
});

test("metadata and simulation edits preserve plan state", () => {
  const base = createDogBoxProject(NOW);
  const preview = installPreviewPlan(base, previewTrajectory(base.rows), NOW);
  const metadata = reviseProject(preview, "metadata", (draft) => {
    draft.identity.name = "Renamed Dog Box";
  }, "2026-07-30T00:01:00.000Z");
  assert.equal(metadata.plan.state, "preview");
  assert.equal(metadata.plan.sourceProjectRevision, metadata.identity.revision);
  assert.equal(planIsFresh(metadata), true);
});

test("execution readiness fails closed until every server-owned proof is present", () => {
  const base = createDogBoxProject(NOW);
  assert.equal(executionReadiness(base).ready, false);
  const evidence = {
    ...emptyEvidence(),
    collision: "clear" as const,
    singularity: "clear" as const,
    limits: "clear" as const,
    startPose: "converged" as const,
    runtimePlayable: true,
  };
  let validated = installServerValidatedPlan(base, {
    id: "plan-1",
    revision: 1,
    createdAt: NOW,
    evidence,
    trajectory: previewTrajectory(base.rows),
  });
  assert.equal(executionReadiness(validated).ready, false);
  validated = reviseProject(validated, "target", (draft) => {
    draft.target = {
      mode: "real_connected",
      profileId: "cell-1",
      manifestRef: "rt-core/config/cells/cell-1.json",
    };
  });
  assert.equal(validated.plan.state, "stale");
  assert.equal(executionReadiness(validated).ready, false);
});

test("project JSON export/import round-trips and rejects unknown schemas", () => {
  const project = createDogBoxProject(NOW);
  const raw = exportProjectJSON(project, NOW);
  const imported = importProjectJSON(raw, NOW);
  assert.deepEqual(imported, project);
  assert.throws(
    () => importProjectJSON(`{"schema":"offline-programming.project.v99"}`),
    /unsupported offline programming project schema/,
  );
});

test("a stored v1 project saved before the Robot V4 program block hydrates instead of throwing", () => {
  // Projects persisted by early builds carry the current schema string but no
  // `program` object at all. Reading one back raw crashed boot on the first
  // `project.program.constraints` access.
  const legacy = structuredClone(createDogBoxProject(NOW)) as unknown as Record<string, unknown>;
  delete legacy.program;
  assert.equal("program" in legacy, false);

  const hydrated = migrateProject(legacy, NOW);
  assert.equal(hydrated.program.programId, hydrated.identity.id);
  assert.ok(hydrated.program.constraints);
  assert.equal(hydrated.rows.length, createDogBoxProject(NOW).rows.length);
});

test("legacy schema_version 1 migrates joint arrays to canonical J1-J9", () => {
  const imported = importProjectJSON(JSON.stringify({
    schema_version: 1,
    project_id: "legacy-1",
    project_name: "Legacy",
    robot_model: "rosie_1400_v3",
    revision: 4,
    rows: [{
      row_id: "row-1",
      target_tcp_pose: [0.5, 0, 0.7, 0, 0, 0],
      target_joints_rad: [1, 2, 3, 4, 5, 6, 7],
    }],
  }), NOW);
  assert.equal(imported.identity.revision, 4);
  assert.deepEqual(imported.rows[0]!.targetJointsRad, [1, 2, 3, 4, 5, 6, 7, 0, 0]);
  assert.equal(imported.plan.state, "missing");
});
