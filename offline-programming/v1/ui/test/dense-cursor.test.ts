import assert from "node:assert/strict";
import test from "node:test";

import type { ProgramV2Node } from "../src/contracts/robot-v4-program-v2.ts";
import { denseSegmentNodeId } from "../src/program-v2/dense-cursor.ts";

const common = { enabled: true, label: "", comment: "", row_hash: "" };
const weld = (id: string, seam: string): ProgramV2Node => ({
  ...common, id, op: "weld",
  geometry: { kind: "seam", id: seam } as never,
  travel_speed_mm_s: 8 as never, standoff_mm: 12 as never,
  weave: { shape: "none" }, weld_parameters: {},
});
const free = (id: string, op: "approach" | "transit" | "retract"): ProgramV2Node => ({ ...common, id, op });
const io = (id: string): ProgramV2Node => ({ ...common, id, op: "io" } as never);

const nodes = [
  free("n-approach", "approach"),
  weld("n-weld-a", "seam-a"),
  io("n-io"),
  free("n-transit", "transit"),
  weld("n-weld-b", "seam-b"),
  free("n-retract", "retract"),
];

test("a weld segment and its dwells light the weld node", () => {
  assert.equal(denseSegmentNodeId(nodes, "weld", "seam-b"), "n-weld-b");
  assert.equal(denseSegmentNodeId(nodes, "dwell", "seam-a:dwell-in"), "n-weld-a");
  assert.equal(denseSegmentNodeId(nodes, "dwell", "seam-a:dwell-out"), "n-weld-a");
});

test("a connecting move lights the freespace node between its welds, events skipped", () => {
  assert.equal(denseSegmentNodeId(nodes, "freespace", "home->seam-a"), "n-approach");
  assert.equal(denseSegmentNodeId(nodes, "freespace", "seam-a->seam-b"), "n-transit");
  assert.equal(denseSegmentNodeId(nodes, "freespace", "seam-b->home"), "n-retract");
});

test("an unknown seam or a malformed source is no node, never a wrong one", () => {
  assert.equal(denseSegmentNodeId(nodes, "weld", "seam-z"), null);
  assert.equal(denseSegmentNodeId(nodes, "freespace", "seam-a"), null);
  assert.equal(denseSegmentNodeId(nodes, "freespace", "seam-z->seam-y"), null);
  // Two welds back to back (an invalid line) have no freespace between them.
  const adjacent = [weld("a", "seam-a"), weld("b", "seam-b")];
  assert.equal(denseSegmentNodeId(adjacent, "freespace", "seam-a->seam-b"), null);
});
