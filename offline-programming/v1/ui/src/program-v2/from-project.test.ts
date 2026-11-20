import assert from "node:assert/strict";
import test from "node:test";

import { computeProgramV2Digest, programV2Problems } from "../contracts/robot-v4-program-v2.ts";
import { plannerPointToWorld } from "../domain/placement.ts";
import { exportProjectJSON, importProjectJSON } from "../domain/serialization.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import {
  classicProvenance,
  documentProblems,
  refreshClassicNodes,
  syncClassicNodes,
  toProgramV2Document,
} from "./from-project.ts";
import { isWeld, lineProblems, weldNodes } from "./nodes.ts";

test("whole-plan speed persists, binds the digest, and leaves individual moves unchanged", async () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  const original = toProgramV2Document(project);
  assert.equal(original.speed_scale, undefined, "old programs retain their full-speed digest");
  project.programV2.speedScale = 0.1;
  const restored = importProjectJSON(exportProjectJSON(project));
  const slowed = toProgramV2Document(restored);
  assert.equal(slowed.speed_scale, 0.1);
  assert.deepEqual(slowed.nodes, original.nodes);
  assert.deepEqual(programV2Problems(slowed), []);
  assert.notEqual(await computeProgramV2Digest(slowed), await computeProgramV2Digest(original));
  for (const value of [0, -1, 1.01, NaN, Infinity]) {
    assert.match(programV2Problems({ ...slowed, speed_scale: value })[0]!, /program.speed_scale/);
  }
  project.programV2.speedScale = 1;
  assert.deepEqual(toProgramV2Document(project), original);
});

test("a classic program's nodes come from its seams, in sequence order, with the rows' poses", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  // Weld creation already synced the nodes; a sync now changes nothing.
  assert.ok(project.nodes.length > 0);
  assert.equal(syncClassicNodes(project), false);
  const welds = weldNodes(project.nodes);
  assert.equal(welds.length, project.sequence.length);
  assert.deepEqual(lineProblems(project.nodes), []);
  const ordered = [...project.sequence].sort((a, b) => a.order - b.order);
  welds.forEach((weld, index) => {
    const provenance = classicProvenance(weld);
    assert.ok(provenance);
    assert.equal(provenance.seam_id, ordered[index]!.seamId);
    assert.equal(provenance.cad_derived, true);
    assert.equal(weld.geometry.kind, "posed_polyline");
    const rows = project.rows.filter((row) => row.seamId === provenance.seam_id && row.phase === "weld");
    assert.equal(weld.geometry.points.length, rows.length);
    weld.geometry.points.forEach((point, k) => {
      const pose = rows[k]!.targetTcpPose;
      assert.deepEqual(point.xyz_m, plannerPointToWorld(project, [pose[0], pose[1], pose[2]]));
      assert.deepEqual(point.rpy_rad, [pose[3], pose[4], pose[5]]);
    });
  });
  // Approach and retract carry the sequence step's lifts.
  const approach = project.nodes[0]!;
  assert.equal(approach.op, "approach");
  assert.deepEqual((approach as { arrive_standoff?: unknown }).arrive_standoff, { direction: "workpiece_z", distance_mm: 50 });
  assert.equal(syncClassicNodes(project), false, "a second sync changes nothing");
});

test("the assembled program.v2 document is accepted by the contract", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  syncClassicNodes(project);
  assert.deepEqual(documentProblems(project), []);
  const document = toProgramV2Document(project);
  assert.deepEqual(programV2Problems(document), []);
  assert.equal(document.nodes.length, project.nodes.length);
  assert.equal(document.placement.cell_id, project.robot.model);
  // The robot and the description it was authored against, from `robot`.
  assert.equal(document.robot.model_id, project.robot.model);
  assert.equal(document.robot.robot_description_sha256, project.robot.description?.robotDescriptionSha256 ?? "sha256:" + "0".repeat(64));
});

test("a project saved before nodes existed opens with nodes; a teach-modified row survives as points", () => {
  const legacy = createDogBoxProject("2026-09-03T00:00:00.000Z");
  const taught = legacy.rows.find((row) => row.phase === "weld")!;
  taught.sourceState = "teach_modified";
  taught.targetTcpPose = [0.5, -0.1, 0.3, 0.1, 0.2, 0.3];
  const json = exportProjectJSON(legacy);
  const stripped = JSON.parse(json) as { project: Record<string, unknown> };
  delete stripped.project.nodes;
  delete stripped.project.programV2;
  const opened = importProjectJSON(JSON.stringify(stripped));
  assert.ok(opened.nodes.length > 0);
  const weld = opened.nodes.find((node) => isWeld(node) && classicProvenance(node)?.seam_id === taught.seamId);
  assert.ok(weld && isWeld(weld) && weld.geometry.kind === "posed_polyline");
  assert.equal(classicProvenance(weld)!.cad_derived, false);
  const point = weld.geometry.points.find((candidate) => candidate.rpy_rad[0] === 0.1);
  assert.ok(point, "the taught pose is one of the node's points");
  assert.deepEqual(point.xyz_m, plannerPointToWorld(opened, [0.5, -0.1, 0.3]));
});

test("refresh recomputes CAD-derived welds after a placement change and leaves hand-edited ones alone", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  syncClassicNodes(project);
  const [first, second] = weldNodes(project.nodes);
  assert.ok(first && second && first.geometry.kind === "posed_polyline" && second.geometry.kind === "posed_polyline");
  const before = structuredClone(first.geometry.points);
  second.geometry.points[0]!.xyz_m = [9, 9, 9];
  const provenance = classicProvenance(second)!;
  second.extensions = { classic: { ...provenance, cad_derived: false } };
  project.fixtures[0]!.translationM = [
    project.fixtures[0]!.translationM[0] + 0.1,
    project.fixtures[0]!.translationM[1],
    project.fixtures[0]!.translationM[2],
  ];
  assert.ok(refreshClassicNodes(project));
  assert.notDeepEqual(first.geometry.points, before, "a CAD-derived weld follows the part");
  assert.deepEqual(second.geometry.points[0]!.xyz_m, [9, 9, 9], "a hand-edited weld keeps its points");
});
