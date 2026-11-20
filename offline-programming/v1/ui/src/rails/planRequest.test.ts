import test from "node:test";
import assert from "node:assert/strict";
import type { TeachPose } from "../contracts/robot-v4-program-v2.ts";
import { createProject } from "../domain/project.ts";
import { dwellNode, freespaceNode, ioNode } from "../program-v2/nodes.ts";
import { hasMotionNodes, measuredStartUnsupported, planRequestBody, plannedNodes, shippedTooling } from "./planRequest.ts";

// The Deck's program on the 1430 cell at 15:45 2026-10-11 (HOME, then taught
// waypoints) was refused as "Planning failed". Every motion node now plans
// from the robot's measured pose; only an output row is refused, by name and
// with the OLP's own words (weld_plan_start.go taughtProgramRefusal), on any
// path, because nothing would switch it.
test("only an output row is refused before posting; Home and taught moves plan from the robot's pose", () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  const pose: TeachPose = { frame_id: "world", xyz_m: [0,0,0], rpy_rad: [0,0,0], joint_names: ["J1","J2","J3","J4","J5","J6"], joint_values_rad: [0,0,0,0,0,0] };
  const move = (id: string) => ({ id, op: "move" as const, enabled: true, label: id, comment: "", row_hash: `pending:${id}`, motion: "joint" as const, target: pose, speed_scale: 1, speed_mm_s: 25,
    capture: { source: "preview" as const, observed_at: "2026-10-11T22:45:00Z", model_id: "rosie_1420_v1", robot_description_sha256: "sha256:"+"1".repeat(64), pose } });
  project.nodes = [freespaceNode("home"), move("P1"), dwellNode(0.5), move("P2"), freespaceNode("home")];
  assert.equal(measuredStartUnsupported(project), "");
  assert.equal(hasMotionNodes(project), true);
  project.nodes.splice(2, 0, ioNode("gas", true, { label: "Gas on" }));
  // Offline the plan cannot know the cell's outputs; connected, the cell
  // decides and the server names what it refuses.
  assert.equal(measuredStartUnsupported(project),
    "Gas on (io) switches an output: plan with the cell connected, so the plan knows the cell's outputs and their expiry");
  assert.equal(measuredStartUnsupported(project, true), "");
  project.nodes[2]!.enabled = false;
  assert.equal(measuredStartUnsupported(project), "", "a disabled output is not planned");
  project.nodes = [freespaceNode("home")];
  assert.equal(hasMotionNodes(project), true, "a Home alone is a plan from the robot's pose");
  assert.equal(plannedNodes(project).length, 1);
  project.nodes = [dwellNode(0.5), ioNode("revpi:1", false)];
  assert.equal(hasMotionNodes(project), false);
});

test("plan from program start is never sent with a required measured start", async () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  project.nodes = [freespaceNode("home")];
  project.cad.filename = "No workpiece";
  const body = await planRequestBody(project, { tooling: shippedTooling(), stepBase64: "", stepFilename: "No workpiece", fixtures: [], freeSpaceBackend: "bspline",
    requireMeasuredStart: true, planFromProgramStart: true });
  assert.equal(body.require_measured_start, false);
  assert.equal(body.plan_from_program_start, true);
  const measured = await planRequestBody(project, { tooling: shippedTooling(), stepBase64: "", stepFilename: "No workpiece", fixtures: [], freeSpaceBackend: "bspline",
    requireMeasuredStart: true });
  assert.equal(measured.require_measured_start, true);
  assert.equal("plan_from_program_start" in measured, false, "older OLP servers never see the new field unless it is chosen");
});

for (const connected of [true, false]) test(`planning request requires native capture only when selected=${connected}`, async () => {
  const project = createProject({ robotModel: "rosie_1420_v1" });
  const pose: TeachPose = { frame_id: "world", xyz_m: [0,0,0], rpy_rad: [0,0,0], joint_names: ["J1","J2","J3","J4","J5","J6"], joint_values_rad: [0,0,0,0,0,0] };
  project.nodes = [{ id: "point", op: "move", enabled: true, label: "Point", comment: "", row_hash: "pending:point", motion: "joint", target: pose, speed_scale: .01, speed_mm_s: 1,
    capture: { source: "preview", observed_at: "2026-09-30T00:00:00Z", model_id: "rosie_1420_v1", robot_description_sha256: "sha256:"+"1".repeat(64), pose } }];
  project.cad.filename = "No workpiece";
  const before = JSON.stringify(project);
  const body = await planRequestBody(project, { tooling: shippedTooling(), stepBase64: "", stepFilename: "No workpiece", fixtures: [], freeSpaceBackend: "bspline", requireMeasuredStart: connected });
  assert.equal(body.require_measured_start, connected);
  assert.equal("start_state" in body, false, "browser never supplies an authoritative measured pose");
  assert.equal(JSON.stringify(project), before, "capturing a planning request does not rewrite the program or Home");
});
