import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import { fileURLToPath } from "node:url";

import { programV2Problems } from "../contracts/robot-v4-program-v2.ts";
import type { ProgramV2Node, RobotV4ProgramV2 } from "../contracts/robot-v4-program-v2.ts";
import type { RailsJoint, RailsSeam } from "../rails/client.ts";
import {
  isRailsWeld,
  jointForDocument,
  railsJointOf,
  railsPayloadOf,
  seamWeldNodeFor,
} from "./rails-node.ts";
import type { RailsEntryLike } from "./rails-node.ts";

const SHA = "b".repeat(64);

/** The weld planner's own example, so the seam and the joint under test are
 *  the ones the detector really emits rather than a shape invented here. */
async function example(): Promise<{ seams: RailsSeam[]; weld_joints: RailsJoint[] }> {
  const path = fileURLToPath(
    new URL("../../../../../weld_planner/v1/schema/examples/dog_box_lap_perimeter.json", import.meta.url),
  );
  return JSON.parse(await readFile(path, "utf8"));
}

async function entry(): Promise<RailsEntryLike> {
  const doc = await example();
  const seam = doc.seams[0]!;
  const joint = doc.weld_joints.find((candidate) => candidate.id === seam.weld_joint_ref)!;
  return { id: "rails_weld_1", seam, joint, span: { start_s: 0, end_s: 1 }, payload: null, note: "accepted" };
}

test("a rails weld carries the joint its seam names, minus the authoring aids", async () => {
  const node = seamWeldNodeFor(await entry(), SHA);
  assert.ok(isRailsWeld(node));
  const joint = railsJointOf(node);
  assert.ok(joint, "the node carries no joint");
  // The reference resolves against what the node itself carries.
  assert.equal(joint.id, (node.geometry as { weld_joint_ref?: string }).weld_joint_ref);
  assert.ok(joint.weld_line, "the joint keeps its line");
  assert.equal("accessibility" in joint, false);
  assert.equal("accessible_sides" in joint, false);
});

test("jointForDocument drops only the aids and copies the rest", async () => {
  const { joint } = await entry();
  const trimmed = jointForDocument(joint as unknown as Record<string, unknown>);
  const dropped = ["accessibility", "accessible_sides"];
  for (const key of Object.keys(joint)) {
    if (dropped.includes(key)) continue;
    assert.deepEqual(trimmed[key], (joint as Record<string, unknown>)[key], key);
  }
  // A copy, not the original: editing the node must not edit the session doc.
  assert.notEqual(trimmed, joint as unknown);
});

test("a rails weld validates inside a document, and dangles without its joint", async () => {
  const node = seamWeldNodeFor(await entry(), SHA);
  const path = fileURLToPath(
    new URL("../../../../../steamdeck/real/v4/contracts/programs/fixtures/complete-program-v2.json", import.meta.url),
  );
  const doc = JSON.parse(await readFile(path, "utf8")) as RobotV4ProgramV2;
  // Node 3 is the fixture's rails weld. Swapping in one this module built puts
  // the producer under the same validator the contract tests use, which is
  // what let `weld_joint_ref` point at nothing for as long as it did.
  doc.nodes[3] = node as ProgramV2Node;
  doc.workpiece.weld_joints = [railsJointOf(node)!];
  assert.deepEqual(programV2Problems(doc), []);

  doc.workpiece.weld_joints = [];
  const problems = programV2Problems(doc);
  assert.match(problems[0] ?? "", /not in workpiece\.weld_joints/);
});

test("the compile payload still names the seam, not the joint", async () => {
  const node = seamWeldNodeFor(await entry(), SHA);
  const payload = railsPayloadOf(node)!;
  assert.deepEqual(payload.seam_ids, [(node.geometry as { id: string }).id]);
});

test("a rails weld states the standoff its detected seam already carried", async () => {
  // The detected seam's own procedure states `process.wire.stickout_mm`, and
  // it used to be dropped: `process` is not one of the keys copied onto the
  // geometry, so the node authored no standoff at all and the planner
  // refused every seam for it -- inside a 200, with no error anywhere.
  const node = seamWeldNodeFor(await entry(), SHA);
  assert.deepEqual(node.standoff_mm, { kind: "constant", value: 17 });
});

test("the standoff the search chose beats the seam's own", async () => {
  // The search moves the gun to keep an angle; what it settled on is what
  // this weld is welded at.
  const base = await entry();
  const node = seamWeldNodeFor({
    ...base,
    payload: {
      id: base.id, seam_ids: [base.seam.id], preset_id: null, span: null,
      overrides: { process: { wire: { stickout_mm: 7.5 } } },
    },
  }, SHA);
  assert.deepEqual(node.standoff_mm, { kind: "constant", value: 7.5 });
});

test("a seam that states no stickout still authors a standoff", async () => {
  // Never absent and never zero: the schema requires it and zero puts the tip
  // in the part, so the fitted gun's default stands in.
  const base = await entry();
  const seam = { ...base.seam, process: { travel_speed_mm_s: { kind: "constant", value: 4 } } };
  const node = seamWeldNodeFor({ ...base, seam: seam as typeof base.seam }, SHA);
  assert.equal((node.standoff_mm as { kind: string; value: number }).kind, "constant");
  assert.ok((node.standoff_mm as { kind: string; value: number }).value > 0);
});

