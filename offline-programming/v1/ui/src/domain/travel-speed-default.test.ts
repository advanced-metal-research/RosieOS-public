import assert from "node:assert/strict";
import test from "node:test";

import type { ProgramV2Node } from "../contracts/robot-v4-program-v2.ts";
import { DEFAULT_TRAVEL_SPEED_MPS, createProject } from "./project.ts";
import { migrateProject } from "./serialization.ts";

/** A saved hand-placed weld, as `manualPoseWeldNode` wrote it, at `speed` mm/s. */
function placedWeld(id: string, speed: number): ProgramV2Node {
  return {
    id, op: "weld", enabled: true, label: id, comment: "", row_hash: `pending:${id}`,
    geometry: { kind: "manual_pose_wp", frame_id: "workpiece", points: [] },
    travel_speed_mm_s: { kind: "constant", value: speed },
    standoff_mm: { kind: "constant", value: 15 },
    weave: { shape: "none" },
    weld_parameters: {},
  } as unknown as ProgramV2Node;
}

const speedOf = (node: ProgramV2Node): number =>
  (node as unknown as { travel_speed_mm_s: { value: number } }).travel_speed_mm_s.value;

test("a new project's process walks at the default, 5 mm/s", () => {
  assert.equal(DEFAULT_TRAVEL_SPEED_MPS, 0.005);
  assert.equal(createProject({}).processParameters[0]!.travelSpeedMps, 0.005);
});

test("a saved program loses the 50 mm/s nobody chose, and keeps every speed somebody did", () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  project.processParameters[0]!.travelSpeedMps = 0.05; // as projects were made before 2026-09-21
  project.nodes = [placedWeld("old-default", 50), placedWeld("typed", 12), placedWeld("typed-slow", 2)];
  const loaded = migrateProject(JSON.parse(JSON.stringify(project)));
  assert.deepEqual(loaded.nodes.map(speedOf), [5, 12, 2]);
  assert.equal(loaded.processParameters[0]!.travelSpeedMps, 0.005);
  // Loading what was just loaded changes nothing further.
  const again = migrateProject(JSON.parse(JSON.stringify(loaded)));
  assert.deepEqual(again.nodes.map(speedOf), [5, 12, 2]);
});

test("a process somebody retimed is left alone, as is a weld that is not hand-placed", () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  project.processParameters[0]!.travelSpeedMps = 0.02;
  const rails = placedWeld("rails", 50) as unknown as { geometry: { kind: string } };
  rails.geometry.kind = "seam";
  project.nodes = [rails as unknown as ProgramV2Node];
  const loaded = migrateProject(JSON.parse(JSON.stringify(project)));
  assert.equal(loaded.processParameters[0]!.travelSpeedMps, 0.02);
  assert.deepEqual(loaded.nodes.map(speedOf), [50]);
});
