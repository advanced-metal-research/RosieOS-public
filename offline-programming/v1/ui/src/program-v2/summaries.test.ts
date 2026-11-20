import assert from "node:assert/strict";
import test from "node:test";

import { createDogBoxProject } from "../examples/dog-box.ts";
import { syncClassicNodes } from "./from-project.ts";
import { dwellNode, freespaceNode, weldNodes } from "./nodes.ts";
import { rowsFromNodes, rowsOfNode } from "./rows-from-nodes.ts";
import { nodeSummary } from "./summaries.ts";

const defaults = { clearance_min_mm: 20, speed_scale: 1, torch_policy: "free" as const, positioner_policy: "free" as const };

test("summaries preview the node in millimetres and degrees and report the compiled rows", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  syncClassicNodes(project);
  project.rows = rowsFromNodes(project);
  const context = { defaults, rowsOf: (id: string) => rowsOfNode(project, id) };
  const weld = nodeSummary(weldNodes(project.nodes)[0]!, 2, context);
  assert.equal(weld.family, "weld");
  assert.match(weld.geometry, /\d+ points · [\d.]+ mm/);
  // The project's own default travel speed (DEFAULT_TRAVEL_SPEED_MPS), in mm/s.
  assert.match(weld.process, /^5 mm\/s/);
  assert.match(weld.process, /torch 45°\/10°/);
  assert.match(weld.status, /rows · unplanned/);
  const approach = nodeSummary(project.nodes[0]!, 1, context);
  assert.equal(approach.family, "freespace");
  assert.match(approach.freespace, /↓Z 50 mm/);
  const bare = nodeSummary(freespaceNode("transit"), 3, { defaults, rowsOf: () => [] });
  assert.match(bare.freespace, /planner decides · clearance 20 mm/);
  const dwell = nodeSummary(dwellNode(0.5), 4, { defaults, rowsOf: () => [] });
  assert.equal(dwell.family, "event");
  assert.equal(dwell.process, "0.5 s");
});
