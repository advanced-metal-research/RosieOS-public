import assert from "node:assert/strict";
import test from "node:test";

import {
  installPreviewPlan,
  pinRobotDescription,
  reviseProject,
} from "../src/domain/project.ts";
import { previewTrajectory } from "../src/domain/playback.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";
import { activatePlan, deletePlan, recordPlan } from "../src/domain/planHistory.ts";
import { migrateProject } from "../src/domain/serialization.ts";
import { AutosaveController } from "../src/storage/autosave.ts";
import {
  IndexedDbProjectRepository,
  MemoryProjectRepository,
} from "../src/storage/repository.ts";

test("memory repository stores current project and immutable revision snapshots", async () => {
  const repository = new MemoryProjectRepository();
  await repository.init();
  const first = createDogBoxProject("2026-07-30T00:00:00.000Z");
  const second = reviseProject(first, "metadata", (draft) => {
    draft.identity.name = "Second";
  }, "2026-07-30T00:01:00.000Z");
  await repository.save(first);
  const plannedFirst = installPreviewPlan(first, previewTrajectory(first.rows));
  await repository.save(plannedFirst);
  assert.equal(
    (await repository.getRevision(first.identity.id, first.identity.revision))!
      .plan.state,
    "preview",
  );
  await repository.save(second);
  assert.deepEqual(await repository.listRevisions(first.identity.id), [first.identity.revision, second.identity.revision]);
  assert.equal((await repository.get(first.identity.id))!.identity.name, "Second");
  assert.equal((await repository.getRevision(first.identity.id, first.identity.revision))!.identity.name, first.identity.name);

  const conflicting = structuredClone(second);
  conflicting.identity.name = "Conflicting same revision";
  await assert.rejects(repository.save(conflicting), /immutable/);
});

test("IndexedDB repository explicitly falls back to memory when no factory exists", async () => {
  const repository = new IndexedDbProjectRepository({ factory: null, databaseName: "test-offline" });
  await repository.init();
  assert.equal(repository.usingMemoryFallback, true);
  const project = createDogBoxProject("2026-07-30T00:00:00.000Z");
  await repository.save(project);
  assert.deepEqual(await repository.listRevisions(project.identity.id), [project.identity.revision]);
  const loaded = await repository.get(project.identity.id);
  assert.deepEqual(loaded, project);
});

test("repository delete removes both current and revision history", async () => {
  const repository = new MemoryProjectRepository();
  const project = createDogBoxProject();
  await repository.save(project);
  await repository.delete(project.identity.id);
  assert.equal(await repository.get(project.identity.id), null);
  assert.deepEqual(await repository.listRevisions(project.identity.id), []);
});

test("an imported STEP is kept under its fingerprint so a later session has it", async () => {
  // The only copy. The weld planner's workspace used to hold one and rails
  // asked for it by hash on reopen; the seam worker resolves nothing from a
  // hash, so the bytes have to be here or the project cannot be worked on.
  const repository = new MemoryProjectRepository();
  await repository.init();
  const bytes = new TextEncoder().encode("ISO-10303-21;\nHEADER;");
  const fingerprint = "a".repeat(64);

  assert.equal(await repository.getSource(fingerprint), null, "nothing kept yet");
  await repository.putSource({ fingerprint, filename: "bracket.step", bytes });

  const stored = await repository.getSource(fingerprint);
  assert.ok(stored, "the STEP was not kept");
  assert.equal(stored.filename, "bracket.step");
  assert.deepEqual([...stored.bytes], [...bytes]);
  assert.match(stored.storedAt, /^\d{4}-\d{2}-\d{2}T/);
});

test("a kept STEP is a copy, so writing the array afterwards cannot reach it", async () => {
  const repository = new MemoryProjectRepository();
  await repository.init();
  const bytes = new TextEncoder().encode("ISO-10303-21;");
  const fingerprint = "b".repeat(64);
  await repository.putSource({ fingerprint, filename: "part.step", bytes });

  bytes.fill(0);
  const stored = await repository.getSource(fingerprint);
  assert.ok(stored);
  assert.equal(new TextDecoder().decode(stored.bytes), "ISO-10303-21;");

  // And the other way: what a caller gets back is its own.
  stored.bytes.fill(0);
  const again = await repository.getSource(fingerprint);
  assert.equal(new TextDecoder().decode(again!.bytes), "ISO-10303-21;");
});

test("two projects cut from one part share one kept copy", async () => {
  // Keyed by fingerprint, which for an imported STEP is its sha256.
  const repository = new MemoryProjectRepository();
  await repository.init();
  const fingerprint = "c".repeat(64);
  await repository.putSource({
    fingerprint, filename: "first-import.step",
    bytes: new TextEncoder().encode("ISO-10303-21;"),
  });
  await repository.putSource({
    fingerprint, filename: "same-part-again.step",
    bytes: new TextEncoder().encode("ISO-10303-21;"),
  });
  assert.equal((await repository.getSource(fingerprint))!.filename, "same-part-again.step");
});

test("the trajectory a plan produced is kept where the plan can be replayed from", async () => {
  // The motion server files it under ~/.cache and owes nobody a copy later.
  // Keyed by the digest, which is the only name a blob has: the daemon fences
  // on it and the motion server serves it by it.
  const repository = new MemoryProjectRepository();
  await repository.init();
  const digest = "sha256:" + "d".repeat(64);
  const bytes = new Uint8Array([0, 0, 0, 8, 123, 125]);

  assert.equal(await repository.getTrajectory(digest), null);
  await repository.putTrajectory({ digest, bytes });

  const stored = await repository.getTrajectory(digest);
  assert.ok(stored);
  assert.deepEqual([...stored.bytes], [...bytes]);

  bytes.fill(9);
  assert.deepEqual([...(await repository.getTrajectory(digest))!.bytes], [0, 0, 0, 8, 123, 125]);
});

// Opening a program written before the description contract gives it a pin.
// That is provenance, not an edit, so it saves under the revision it already
// has: the store used to refuse it and the app never finished loading.
test("a description pin is not an edit, and saves under the same revision", async () => {
  const repository = new MemoryProjectRepository();
  await repository.init();
  const stored = createDogBoxProject("2026-09-18T00:00:00.000Z");
  delete stored.robot.description;
  await repository.save(stored);
  const pinned = pinRobotDescription(stored, "2026-09-18T01:00:00.000Z");
  assert.notEqual(pinned, stored, "the project should have gained a pin");
  await repository.save(pinned);
  const back = await repository.get(stored.identity.id);
  assert.equal(back!.robot.description?.robotDescriptionSha256, pinned.robot.description?.robotDescriptionSha256);
  assert.equal(back!.identity.revision, stored.identity.revision);
  // An actual edit under the same revision is still refused.
  const edited = { ...pinned, rows: [] };
  await assert.rejects(() => repository.save(edited), /immutable/);
});

test("planning, switching and deleting plans persist on the same authored revision", async () => {
  const repository = new MemoryProjectRepository();
  let project = createDogBoxProject();
  await repository.save(project);
  const revision = project.identity.revision;
  for (const suffix of ["first", "second"]) {
    project.denseTrajectory = { schema: "robot.v4.dense-joint-trajectory.v1", trajectoryDigest: `sha256:${suffix}`, requestSha256: suffix,
      planId: suffix, programId: project.identity.id, programDigest: suffix, manifestRevision: 1, planRevision: revision,
      createdAt: new Date().toISOString(), dtS: .01, totalSampleCount: 1, totalDurationS: .01, bytes: 153, segments: [], sourceProjectRevision: revision };
    project = recordPlan(project);
    await repository.save(project);
  }
  const reopened = migrateProject(await repository.get(project.identity.id));
  assert.equal(reopened.denseTrajectory?.trajectoryDigest, "sha256:second");
  assert.equal(reopened.planHistory?.length, 2);
  // A user selects another waypoint before planning/replaying the same revision.
  reopened.simulation.selectedRowId = reopened.rows.at(-1)!.id;
  reopened.simulation.cursorS = 1;
  const active = activatePlan(reopened, "plan:sha256:first");
  await repository.save(active);
  assert.equal((await repository.get(project.identity.id))!.denseTrajectory?.trajectoryDigest, "sha256:first");
  await repository.save(deletePlan(active, "plan:sha256:first"));
  assert.equal((await repository.get(project.identity.id))!.denseTrajectory, null);
  const edited = structuredClone(active); edited.identity.name = "unsaved author edit";
  await assert.rejects(repository.save(edited), /immutable/);
});

test("autosave reports saved only after the write succeeds", async () => {
  const repository = new MemoryProjectRepository();
  let saved = 0;
  let failed = 0;
  const autosave = new AutosaveController(repository, 1000, () => failed++, () => saved++);
  const project = createDogBoxProject();
  autosave.schedule(project);
  assert.equal(saved, 0);
  await autosave.flush();
  assert.equal(saved, 1);
  const conflict = structuredClone(project); conflict.identity.name = "conflict";
  autosave.schedule(conflict);
  await assert.rejects(autosave.flush(), /immutable/);
  assert.equal(saved, 1, "a failed save must not report LOCAL");
  assert.equal(failed, 1, "explicit planning saves must show storage failures too");
});
