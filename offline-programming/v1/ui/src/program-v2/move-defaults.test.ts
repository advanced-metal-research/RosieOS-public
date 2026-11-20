import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../test/support/robot-catalogue.ts";
import type { MoveNode, TeachPose } from "../contracts/robot-v4-program-v2.ts";
import { pinRobotDescription } from "../domain/project.ts";
import { createBlankSpreadsheetProject } from "../domain/spreadsheet.ts";
import { recordMove } from "./edits.ts";
import { HOME_WAYPOINT_SPEED_MM_S, RECORDED_MOVE_SPEED_MM_S } from "./move-defaults.ts";

seedRobotCatalogue();

function pose(x: number): TeachPose {
  return { frame_id: "world", joint_names: ["J1", "J2", "J3", "J4", "J5", "J6"], joint_values_rad: [x, 0, 0, 0, 0, 0],
    xyz_m: [1, x, 1], rpy_rad: [0, 0, 0] };
}

// 25 mm/s made the 1430 cell's five-move `test1` a 98.5 s trajectory (2026-10-11).
// The default changes what a NEW row gets; a saved row keeps the speed it
// was recorded or edited with.
test("a recorded waypoint gets the default speed and an existing row keeps its own", () => {
  const draft = pinRobotDescription(createBlankSpreadsheetProject({ name: "teach", robotModel: "rosie_1400_v3", railsAuthoring: true }));
  const capture = { source: "preview", observed_at: "2026-10-11T22:00:00Z", model_id: draft.robot.model,
    robot_description_sha256: draft.robot.description!.robotDescriptionSha256, pose: pose(0) } as MoveNode["capture"];
  const first = recordMove(draft, "new", null, pose(0), capture, "p1");
  const recorded = draft.nodes.find((node) => node.id === first) as MoveNode;
  assert.equal(recorded.speed_mm_s, RECORDED_MOVE_SPEED_MM_S);
  assert.ok(RECORDED_MOVE_SPEED_MM_S >= 100 && RECORDED_MOVE_SPEED_MM_S <= 250, "the owner's MoveL range");
  recorded.speed_mm_s = 25;
  recordMove(draft, "new", first, pose(0.1), capture, "p2");
  assert.equal((draft.nodes.find((node) => node.id === "p1") as MoveNode).speed_mm_s, 25);
  assert.equal((draft.nodes.find((node) => node.id === "p2") as MoveNode).speed_mm_s, RECORDED_MOVE_SPEED_MM_S);
  recordMove(draft, "reteach", "p1", pose(0.05), capture, "unused");
  assert.equal((draft.nodes.find((node) => node.id === "p1") as MoveNode).speed_mm_s, 25, "reteach moves the target, not the speed");
  assert.equal(HOME_WAYPOINT_SPEED_MM_S, RECORDED_MOVE_SPEED_MM_S);
});
