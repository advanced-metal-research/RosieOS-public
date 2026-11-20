import assert from "node:assert/strict";
import test from "node:test";
import { LOOP_RETURN_NODE_ID, withLoopReturn } from "./planRequest.ts";

const move = { id: "P1", op: "move", enabled: true } as never;
const io = { id: "gas", op: "io", enabled: true } as never;

test("a repeat plans the return to the start as one marked Home, once", () => {
  const looped = withLoopReturn({ nodes: [move] }, true);
  assert.equal(looped.nodes.length, 2);
  const last = looped.nodes[1] as unknown as Record<string, unknown>;
  assert.deepEqual([last.id, last.op, last.enabled, last.extensions], [LOOP_RETURN_NODE_ID, "home", true, { "rosie.loop_return": true }]);
  assert.equal(withLoopReturn(looped, true).nodes.length, 2, "never twice");
  assert.equal(withLoopReturn({ nodes: [io] }, true).nodes.length, 1, "nothing moves: no return");
  assert.equal(withLoopReturn({ nodes: [move] }, false).nodes.length, 1, "a model-reset weld plan starts and ends at reset");
});
