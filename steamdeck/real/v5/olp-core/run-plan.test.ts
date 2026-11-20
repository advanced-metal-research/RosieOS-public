import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../../../offline-programming/v1/ui/test/support/robot-catalogue.ts";
import * as session from "./session.ts";
import * as run from "./run.ts";
import { decodeDenseTrajectory, RECORD_BYTES } from "../../../../offline-programming/v1/ui/src/execution/denseTrajectoryBlob.ts";
import { densePlannedPathRuns } from "../../../../offline-programming/v1/ui/src/viewer/planned-path.ts";
import { robotModelFor } from "../../../../offline-programming/v1/ui/src/viewer/models.ts";

seedRobotCatalogue();
function program(name: string) {
  session.newProject(name, "rosie_1400_v3");
  session.addEvent("home");
  const source = session.currentProject();
  const result = {
    program_digest: "sha256:source",
    dense: {
      schema: "dense", trajectory_digest: "sha256:trajectory", request_sha256: "request", plan_id: "plan",
      program_id: source.program.programId, program_digest: "sha256:source", manifest_revision: 1, plan_revision: 1,
      created_at: "2026-10-27T00:00:00Z", dt_s: .001, total_sample_count: 3000, total_duration_s: 3, bytes: 100,
      segments: [{ index: 0, kind: "freespace", source_id: source.nodes[0]!.id, sample_count: 3000, duration_s: 3 }],
    },
  } as Parameters<typeof run.applyPlan>[0];
  return { id: source.identity.id, revision: source.identity.revision, result, saved: session.exportProject() };
}

test("a late foreground plan cannot attach to another program at the same revision", () => {
  const a = program("A");
  const b = program("B");
  assert.equal(a.revision, b.revision);
  assert.notEqual(a.id, b.id);
  const before = structuredClone(session.currentProject());
  const undoBefore = session.view().canUndo;
  assert.throws(() => run.applyPlan(a.result, a.revision, a.id), /changed while it was planning/);
  assert.deepEqual(session.currentProject(), before);
  assert.equal(session.view().canUndo, undoBefore);
  assert.equal(session.currentProject().denseTrajectory, null);
  // The active program can still accept its own result after that refusal.
  assert.equal(run.applyPlan(b.result, b.revision, b.id), b.result.dense!.trajectory_digest);
  assert.equal(session.currentProject().denseTrajectory!.programId, b.result.dense!.program_id);
});

test("opening a same-revision program rejects a stale result; edits still reject the correct identity", () => {
  const a = program("Saved A");
  const b = program("Saved B");
  session.openProject(a.saved);
  assert.throws(() => run.applyPlan(b.result, b.revision, b.id), /changed while it was planning/);
  assert.equal(session.projectIdentity(), a.id);
  session.addEvent("dwell");
  assert.throws(() => run.applyPlan(a.result, a.revision, a.id), /changed while it was planning/);
  assert.equal(session.currentProject().denseTrajectory, null);
});

function pathBlob(samples: number) {
  const segments = [{ kind: "weld", sample_count: 0 }, { kind: "freespace", sample_count: Math.floor(samples / 3) },
    { kind: "weld", sample_count: Math.floor(samples / 3) }, { kind: "dwell", sample_count: samples - 2 * Math.floor(samples / 3) }];
  const header = Buffer.from(JSON.stringify({ schema: "dense", trajectory_digest: "sha256:same", segments }));
  const bytes = Buffer.alloc(4 + header.length + samples * RECORD_BYTES);
  bytes.writeUInt32BE(header.length); header.copy(bytes, 4);
  const special = [-0, Number.MIN_VALUE, -Number.MIN_VALUE, Number.MAX_VALUE, NaN, Infinity, -Infinity];
  for (let index = 0; index < samples; ++index) {
    const at = 4 + header.length + index * RECORD_BYTES;
    bytes.writeDoubleBE(index * .001, at);
    for (let axis = 0; axis < 9; ++axis) {
      bytes.writeDoubleBE(index < special.length ? special[index]! : (index * 9 + axis) * .000001, at + 8 + axis * 8);
    }
    bytes[at + RECORD_BYTES - 1] = index % 2;
  }
  return bytes;
}

test("pendant path projection equals the canonical browser projection for every sample and segment", () => {
  for (const model of ["rosie_1420_v1", "rosie_1400_v3"]) {
    session.newProject("path projection", model);
    const project = session.currentProject();
    const before = structuredClone(project);
    for (const count of [0, 2, 8192, 8193, 8702, 36000]) {
      const bytes = pathBlob(count), trajectory = decodeDenseTrajectory(bytes);
      const expected = {
        jointOrder: [...project.robot.jointOrder], frame: robotModelFor(model).cameras.tcp.frame,
        runs: densePlannedPathRuns(project, trajectory).map(samples => ({
          weld: samples[0]?.motion === "weld", q: samples.map(sample => sample.jointsRad),
        })),
      };
      const native = Uint8Array.from(bytes).buffer;
      assert.deepEqual(run.plannedPathRuns(native), expected);
      assert.deepEqual(run.plannedPathRuns(bytes.toString("base64")), expected);
      const changed = run.plannedPathRuns(native);
      if (count > 0) changed.runs.find(segment => segment.q.length)!.q[0]![0] = 97;
      assert.deepEqual(run.plannedPathRuns(native), expected); // result never aliases decoded input
    }
    assert.deepEqual(session.currentProject(), before);
  }
});

test("path projection keeps canonical malformed dense refusals", () => {
  session.newProject("path refusals", "rosie_1420_v1");
  for (const bytes of [Buffer.from([0, 0, 0, 99]), pathBlob(2).subarray(0, 20),
                       Buffer.concat([pathBlob(2), Buffer.from([0])])]) {
    let expected = "";
    try { decodeDenseTrajectory(bytes); } catch (error) { expected = (error as Error).message; }
    assert.ok(expected);
    assert.throws(() => run.plannedPathRuns(Uint8Array.from(bytes).buffer), error => (error as Error).message === expected);
  }
});
