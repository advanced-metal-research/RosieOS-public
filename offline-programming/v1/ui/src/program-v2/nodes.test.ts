import assert from "node:assert/strict";
import test from "node:test";

import type { ProgramV2Node, WeldNode } from "../contracts/robot-v4-program-v2.ts";
import {
  dwellNode,
  effectiveNodes,
  freespaceNode,
  insertWeld,
  ioNode,
  lineProblems,
  moveBlocked,
  moveNode,
  removeNode,
} from "./nodes.ts";

function weld(id: string): WeldNode {
  return {
    id, op: "weld", enabled: true, label: id, comment: "", row_hash: `pending:${id}`,
    geometry: {
      kind: "posed_polyline", frame_id: "world",
      points: [{ xyz_m: [0, 0, 0], rpy_rad: [0, 0, 0] }, { xyz_m: [0.1, 0, 0], rpy_rad: [0, 0, 0] }],
    },
    travel_speed_mm_s: { kind: "constant", value: 8 },
    standoff_mm: { kind: "constant", value: 12 },
    weave: { shape: "none" },
    weld_parameters: {},
  };
}

const ops = (nodes: readonly ProgramV2Node[]): string => nodes.map((node) => node.op).join(" ");
const lift = { direction: "workpiece_z" as const, distance_mm: 10 };

test("the first weld gets an approach and a retract; the next turns the retract into a transit", () => {
  let nodes = insertWeld([], weld("w1"), { arrive: lift, depart: lift });
  assert.equal(ops(nodes), "approach weld retract");
  assert.deepEqual(lineProblems(nodes), []);
  nodes = insertWeld(nodes, weld("w2"), { arrive: { ...lift, distance_mm: 20 }, depart: lift });
  assert.equal(ops(nodes), "approach weld transit weld retract");
  const transit = nodes[2]!;
  assert.equal(transit.op, "transit");
  assert.deepEqual((transit as { depart_standoff?: unknown }).depart_standoff, lift);
  assert.deepEqual((transit as { arrive_standoff?: unknown }).arrive_standoff, { ...lift, distance_mm: 20 });
  assert.deepEqual(lineProblems(nodes), []);
});

test("a trailing home (and the events that go with it) stays last when a weld is added", () => {
  const nodes = [
    freespaceNode("home", { id: "h0" }),
    ...insertWeld([], weld("w1")),
    ioNode("revpi:3", false, { id: "gas-off" }),
    freespaceNode("home", { id: "h1" }),
  ];
  const next = insertWeld(nodes, weld("w2"));
  assert.equal(ops(next), "home approach weld transit weld retract io home");
  assert.deepEqual(lineProblems(next), []);
});

test("removing a weld merges the freespace around it and never breaks the line", () => {
  let nodes = insertWeld(insertWeld(insertWeld([], weld("w1")), weld("w2")), weld("w3"));
  assert.equal(ops(nodes), "approach weld transit weld transit weld retract");
  nodes = removeNode(nodes, "w2");
  assert.equal(ops(nodes), "approach weld transit weld retract");
  nodes = removeNode(nodes, "w3");
  assert.equal(ops(nodes), "approach weld retract");
  assert.deepEqual(lineProblems(nodes), []);
  nodes = removeNode(nodes, "w1");
  assert.deepEqual(nodes, []);
});

test("removing the first of two welds turns the transit into the approach", () => {
  const nodes = removeNode(insertWeld(insertWeld([], weld("w1")), weld("w2")), "w1");
  assert.equal(ops(nodes), "approach weld retract");
  assert.equal(nodes[1]!.id, "w2");
});

test("a freespace node cannot be removed on its own; events and homes can", () => {
  const nodes = [freespaceNode("home", { id: "h" }), ...insertWeld([], weld("w1")), dwellNode(0.5, { id: "d" })];
  assert.throws(() => removeNode(nodes, "w1-approach"), /delete the weld instead/);
  assert.equal(ops(removeNode(nodes, "h")), "approach weld retract dwell");
  assert.equal(ops(removeNode(nodes, "d")), "home approach weld retract");
});

test("a weld moves by swapping with its neighbouring weld; freespace stays put", () => {
  const nodes = insertWeld(insertWeld([], weld("w1")), weld("w2"));
  const moved = moveNode(nodes, "w2", -1);
  assert.equal(moved.map((node) => node.id).join(" "), "w1-approach w2 w1-retract w1 w2-retract");
  assert.deepEqual(lineProblems(moved), []);
  assert.equal(moveBlocked(nodes, "w1", -1), "already the first weld");
  assert.match(moveBlocked(nodes, "w1-retract", 1) ?? "", /moves with its weld/);
});

test("an event slides one slot at a time and the line does not care", () => {
  const nodes = [...insertWeld([], weld("w1")), ioNode("revpi:3", true, { id: "io" })];
  const moved = moveNode(moveNode(nodes, "io", -1), "io", -1);
  assert.equal(ops(moved), "approach io weld retract");
  assert.deepEqual(lineProblems(moved), []);
});

test("two adjacent welds are named as the missing transit", () => {
  const problems = lineProblems([freespaceNode("approach"), weld("a"), weld("b"), freespaceNode("retract")]);
  assert.ok(problems.some((line) => line.includes("transit is missing")), problems.join("; "));
});

test("a disabled weld compiles as if deleted: its moves merge and the line stays whole", () => {
  const nodes = insertWeld(insertWeld(insertWeld([], weld("w1")), weld("w2")), weld("w3"));
  nodes[3]!.enabled = false; // w2
  const effective = effectiveNodes(nodes);
  assert.equal(ops(effective), "approach weld transit weld retract");
  assert.deepEqual(effective.map((node) => node.id).filter((id) => id.startsWith("w") && !id.includes("-")), ["w1", "w3"]);
  assert.deepEqual(lineProblems(effective), []);
  assert.equal(nodes.length, 7, "the authored list is untouched");
});


test("taught moves reorder around complete weld blocks and disable without removing welds", () => {
  const pose = { frame_id: "world" as const, xyz_m: [0, 0, 0] as [number, number, number], rpy_rad: [0, 0, 0] as [number, number, number], joint_names: ["J1"], joint_values_rad: [0] };
  const move: import("../contracts/robot-v4-program-v2.ts").MoveNode = {
    id: "p1", op: "move", label: "P1", comment: "", row_hash: "pending:p1", enabled: true,
    motion: "joint", target: pose, speed_scale: .1, speed_mm_s: 25,
    capture: { source: "preview", observed_at: "2026-09-25T00:00:00Z", model_id: "rosie_1420_v1", robot_description_sha256: "sha256:" + "1".repeat(64), pose },
  };
  const nodes = insertWeld([move], weld("w1"));
  assert.equal(ops(nodes), "move approach weld retract");
  assert.deepEqual(lineProblems(nodes), []);
  assert.equal(ops(moveNode(nodes, "p1", 1)), "approach weld retract move");
  assert.equal(ops(effectiveNodes([{ ...move, enabled: false }, ...nodes.slice(1)])), "approach weld retract");
  assert.equal(ops(removeNode(nodes, "p1")), "approach weld retract");
});
