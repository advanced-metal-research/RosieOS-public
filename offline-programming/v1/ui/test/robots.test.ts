/**
 * Which robots the browser thinks exist, and what that costs when it is
 * wrong.
 *
 * The list used to be a TypeScript union with a work frame constant beside
 * it. It is the server's answer now, fetched once; these hold the fetched
 * catalogue to the numbers the constants carried, because the offset written
 * into a `.weldplan` is measured from this frame and read from the planner's,
 * and the two being 1.913 m apart is the bug the whole contract exists to end.
 */

import assert from "node:assert/strict";
import test from "node:test";

import {
  cellWorkFrame,
  defaultRobotModel,
  knownRobotModel,
  parseRobotCatalogue,
  robotCatalogue,
  robotDescriptor,
  setRobotCatalogue,
} from "../src/domain/robots.ts";
import { fetchRobotCatalogue, ROBOTS_PATH } from "../src/targets/robotsClient.ts";
import { createProject, type OfflineProgrammingProject } from "../src/domain/project.ts";
import { seedRobotCatalogue, TEST_ROBOTS } from "./support/robot-catalogue.ts";

// What the browser carried before the server answered: CELL_WORK_FRAME, and
// DEFAULT_TABLE_CENTRE_WORLD_M with it, rounded to 0.1 mm.
const WAS = {
  rosie_1400_v3: { id: "positioner_table_a", worldM: [1.913, 0.6858, 0.6509] },
  rosie_1420_v1: { id: "world", worldM: [0, 0, 0] },
};
const TOLERANCE_M = 1e-4;

const PAYLOAD = {
  robots: [
    {
      model_id: "rosie_1400_v3",
      robot_description_sha256: `sha256:${"1".repeat(64)}`,
      frames: {
        base: "world",
        tool: "tool0",
        work: "positioner_table_a_top",
        work_world_m: [1.9129999999999998, 0.68580026, 0.6508788],
      },
      axes: { driven: ["J1", "J7"], held: { J8: 0, J9: 0 } },
      reset_pose: { J1: 0 },
    },
    {
      model_id: "rosie_1420_v1",
      robot_description_sha256: `sha256:${"2".repeat(64)}`,
      frames: { base: "world", tool: "tool0", work: "world", work_world_m: [0, 0, 0] },
      axes: { driven: ["J1"], held: {} },
      reset_pose: {},
    },
  ],
};

function response(body: unknown, status = 200): Response {
  return new Response(JSON.stringify(body), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}

test("the route's answer is read whole, and a robot it cannot describe is left out", () => {
  const parsed = parseRobotCatalogue(PAYLOAD);
  assert.deepEqual(parsed.map((entry) => entry.modelId), ["rosie_1400_v3", "rosie_1420_v1"]);
  assert.equal(parsed[0]!.frames.work, "positioner_table_a_top");
  assert.deepEqual(parsed[0]!.held, { J8: 0, J9: 0 });
  assert.deepEqual(parsed[0]!.driven, ["J1", "J7"]);
  assert.equal(parsed[1]!.robotDescriptionSha256, `sha256:${"2".repeat(64)}`);
  // No id, or no work frame to place a part against: not a robot this can offer.
  assert.deepEqual(parseRobotCatalogue({ robots: [{ frames: PAYLOAD.robots[0]!.frames }] }), []);
  assert.deepEqual(parseRobotCatalogue({ robots: [{ model_id: "x" }] }), []);
  assert.deepEqual(
    parseRobotCatalogue({ robots: [{ model_id: "x", frames: { work: "world", work_world_m: [0, 0] } }] }),
    [],
  );
  assert.deepEqual(parseRobotCatalogue({}), []);
  assert.deepEqual(parseRobotCatalogue(null), []);
});

test("the client asks the one route, and passes the server's words on when it cannot", async () => {
  const asked: string[] = [];
  const robots = await fetchRobotCatalogue((path) => {
    asked.push(path);
    return Promise.resolve(response(PAYLOAD));
  });
  assert.deepEqual(asked, [ROBOTS_PATH]);
  assert.deepEqual(robots.map((entry) => entry.modelId), ["rosie_1400_v3", "rosie_1420_v1"]);

  await assert.rejects(
    () => fetchRobotCatalogue(() => Promise.resolve(response({ error: "no robot descriptions at /nowhere" }, 500))),
    /no robot descriptions at \/nowhere/,
  );
});

test("the fetched work frame is the constant it replaced, to 0.1 mm", () => {
  setRobotCatalogue(parseRobotCatalogue(PAYLOAD));
  for (const [model, was] of Object.entries(WAS)) {
    const now = cellWorkFrame(model);
    was.worldM.forEach((value, axis) => {
      assert.ok(
        Math.abs(now.worldM[axis]! - value) < TOLERANCE_M,
        `${model} axis ${axis}: ${now.worldM[axis]} != ${value}`,
      );
    });
  }
  // The id is the planner's own: the 1400's table SURFACE, which is the point
  // those coordinates always named, rather than the link it hangs off.
  assert.equal(cellWorkFrame("rosie_1400_v3").id, "positioner_table_a_top");
  assert.notEqual(cellWorkFrame("rosie_1400_v3").id, WAS.rosie_1400_v3.id);
  assert.equal(cellWorkFrame("rosie_1420_v1").id, WAS.rosie_1420_v1.id);
  seedRobotCatalogue();
});

test("a project naming a robot this server does not have is not quietly given another one's", () => {
  seedRobotCatalogue();
  assert.equal(knownRobotModel("rosie_1400_v3"), true);
  assert.equal(knownRobotModel("rosie_9999_x1"), false);
  // The base origin, NOT the 1400's table: a part hung off another robot's
  // positioner is a part 1.913 m from where the planner reads it.
  const unknown = cellWorkFrame("rosie_9999_x1");
  assert.deepEqual(unknown, { id: "world", worldM: [0, 0, 0] });
  assert.notEqual(unknown.id, cellWorkFrame("rosie_1400_v3").id);

  const stored: OfflineProgrammingProject = createProject({ robotModel: "rosie_9999_x1" });
  // The name it was authored with survives; what is missing is said, not fixed.
  assert.equal(stored.robot.model, "rosie_9999_x1");
  assert.equal(knownRobotModel(stored.robot.model), false);
});

test("a new program is for the first robot the server listed, and nothing when it listed none", () => {
  seedRobotCatalogue();
  assert.equal(defaultRobotModel(), TEST_ROBOTS[0]!.modelId);
  assert.equal(createProject().robot.model, TEST_ROBOTS[0]!.modelId);
  assert.equal(robotDescriptor(TEST_ROBOTS[0]!.modelId)?.frames.tool, "tool0");

  setRobotCatalogue([]);
  assert.equal(defaultRobotModel(), "");
  assert.deepEqual(robotCatalogue(), []);
  assert.equal(robotDescriptor("rosie_1400_v3"), null);
  seedRobotCatalogue();
});
