import assert from "node:assert/strict";
import test from "node:test";

import type { FreespaceNode } from "../contracts/robot-v4-program-v2.ts";
import { buildProgramView } from "../domain/points-moves.ts";
import { createDogBoxProject } from "../examples/dog-box.ts";
import { syncClassicNodes } from "./from-project.ts";
import { insertWeld, weldNodes } from "./nodes.ts";
import { rowNodeId, rowsFromNodes, rowsOfNode, stampRowNodeIds } from "./rows-from-nodes.ts";

test("rows compiled from classic nodes reproduce the rows the seams compiled to", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  const original = project.rows;
  syncClassicNodes(project);
  const rows = rowsFromNodes(project);
  const weldRows = rows.filter((row) => row.phase === "weld");
  assert.equal(weldRows.length, original.filter((row) => row.phase === "weld").length);
  weldRows.forEach((row, index) => {
    const source = original.filter((candidate) => candidate.phase === "weld")[index]!;
    row.targetTcpPose.forEach((value, k) => assert.ok(Math.abs(value - source.targetTcpPose[k]!) < 1e-9));
    assert.equal(row.seamId, source.seamId);
    assert.equal(row.weldEnabled, true);
  });
  // An approach row before every weld and a retract row after it, each
  // lifted the sequence step's 50 mm along the part's +Z -- a transit's two
  // lifts lower as the classic retract/approach pair.
  const welds = weldNodes(project.nodes);
  assert.equal(rows.filter((row) => row.phase === "approach").length, welds.length);
  assert.equal(rows.filter((row) => row.phase === "retract").length, welds.length);
  assert.equal(rows.filter((row) => row.phase === "travel").length, 0);
  const first = rows[0]!;
  assert.equal(first.phase, "approach");
  const firstWeld = rows[1]!;
  assert.ok(Math.abs(first.targetTcpPose[2] - (firstWeld.targetTcpPose[2] + 0.05)) < 1e-9);
  for (const row of rows) assert.ok(rowNodeId(row), `${row.id} names its node`);
  assert.equal(rowsOfNode({ ...project, rows }, welds[0]!.id).length, weldRows.filter((row) => row.seamId === weldRows[0]!.seamId).length);
  // The compiled view still groups the weld into moves.
  const view = buildProgramView({ ...project, rows });
  assert.ok(view.moves.length > 0);
});

test("a transit lowers only the lifts it states; a posed weld without a classic seam still compiles", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  syncClassicNodes(project);
  const welds = weldNodes(project.nodes);
  const transit = project.nodes.find((node) => node.op === "transit") as FreespaceNode;
  delete transit.arrive_standoff;
  delete transit.depart_standoff;
  const rows = rowsFromNodes(project);
  assert.equal(rows.filter((row) => rowNodeId(row) === transit.id).length, 0);
  // A hand-made posed weld with no seam provenance.
  const loose = structuredClone(welds[0]!);
  loose.id = "loose";
  delete loose.extensions;
  project.nodes = insertWeld(project.nodes, loose);
  const withLoose = rowsFromNodes(project);
  const looseRows = withLoose.filter((row) => rowNodeId(row) === "loose");
  assert.ok(looseRows.length >= 2);
  assert.equal(looseRows[0]!.seamId, "loose");
});

test("rows the weld planner compiled get their node ids by seam", () => {
  const nodes = insertWeld([], {
    id: "seam-weld", op: "weld", enabled: true, label: "s", comment: "", row_hash: "pending",
    geometry: {
      kind: "seam", id: "seam_0001", reference: { mode: "fixed_vector" },
      curve: { segments: [{ degree: 1, control_points_xyz_m: [[0, 0, 0], [1, 0, 0]], knots_u: [0, 0, 1, 1] }] },
      orientation: { work_angle_deg: { kind: "constant", value: 0 }, travel_angle_deg: { kind: "constant", value: 0 } },
    },
    travel_speed_mm_s: { kind: "constant", value: 8 },
    standoff_mm: { kind: "constant", value: 12 },
    weave: { shape: "none" },
    weld_parameters: {},
  });
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  const rows = project.rows.slice(0, 3).map((row, index) => ({
    ...row,
    seamId: "seam_0001",
    phase: (["approach", "weld", "retract"] as const)[index]!,
    extensions: {},
  }));
  stampRowNodeIds(rows, nodes);
  assert.equal(rowNodeId(rows[0]!), "seam-weld-approach");
  assert.equal(rowNodeId(rows[1]!), "seam-weld");
  assert.equal(rowNodeId(rows[2]!), "seam-weld-retract");
});

test("a migrated program's unstamped rows are found by seam and phase, read-only", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  // As a project saved before nodes: rows without node ids.
  project.rows = project.rows.map((row) => ({ ...row, extensions: {} }));
  const welds = weldNodes(project.nodes);
  const first = welds[0]!;
  const weldRows = rowsOfNode(project, first.id);
  assert.ok(weldRows.length >= 2);
  assert.ok(weldRows.every((row) => row.phase === "weld"));
  const approach = project.nodes[0]!;
  assert.equal(approach.op, "approach");
  assert.equal(rowsOfNode(project, approach.id).length, 1);
  const transit = project.nodes.find((node) => node.op === "transit")!;
  assert.equal(rowsOfNode(project, transit.id).length, 2);
  assert.deepEqual(project.rows.map((row) => row.extensions), project.rows.map(() => ({})), "nothing was rewritten");
});

test("a disabled weld and the moves around it produce no rows", () => {
  const project = createDogBoxProject("2026-09-03T00:00:00.000Z");
  const welds = weldNodes(project.nodes);
  const second = welds[1]!;
  second.enabled = false;
  const rows = rowsFromNodes(project);
  assert.equal(rows.filter((row) => rowNodeId(row) === second.id).length, 0);
  const transitBefore = project.nodes[project.nodes.findIndex((node) => node.id === second.id) - 1]!;
  assert.equal(transitBefore.op, "transit");
  // One transit survives between the neighbouring welds, the other is merged away.
  assert.equal(rows.filter((row) => rowNodeId(row) === transitBefore.id).length, 2);
  assert.equal(rows.filter((row) => row.phase === "weld").length, project.rows.filter((row) => row.phase === "weld").length - 2);
  assert.ok(rows.every((row) => row.enabled));
});
