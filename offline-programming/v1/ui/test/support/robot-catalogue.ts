/**
 * The robots a test runs against.
 *
 * In the app the catalogue arrives from `GET /robots` before the first
 * project is read, so every project has a description behind it; a test
 * process starts with an empty one, and placement arithmetic with no work
 * frame is not the thing any of these tests are about. `scripts/test.mjs`
 * preloads `seed.ts`, which installs these two.
 *
 * The numbers are the SERVER'S: `work_world_m` is computed from the URDF at
 * the zero pose, which is why 1400's is the unrounded twin of the constant
 * the browser used to carry (`DEFAULT_TABLE_CENTRE_WORLD_M`, [1.913, 0.6858,
 * 0.6509]) and its work frame is `positioner_table_a_top`, the surface point
 * the planner adds, rather than `positioner_table_a`, the link it hangs off.
 * `test/robots.test.ts` holds the two to each other.
 */

import { setRobotCatalogue, type RobotDescriptorSummary } from "../../src/domain/robots.ts";

export const ROSIE_1400_V3: RobotDescriptorSummary = {
  modelId: "rosie_1400_v3",
  robotDescriptionSha256: `sha256:${"1".repeat(64)}`,
  frames: {
    base: "world",
    tool: "tool0",
    work: "positioner_table_a_top",
    workWorldM: [1.9129999999999998, 0.68580026, 0.6508788],
    armBaseWorldM: [0.7874, 0.6858, 0.4808375],
  },
  driven: ["J1", "J2", "J3", "J4", "J5", "J6", "J7"],
  held: { J8: 0, J9: 0 },
  resetPose: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0, J7: 0 },
};

export const ROSIE_1420_V1: RobotDescriptorSummary = {
  modelId: "rosie_1420_v1",
  robotDescriptionSha256: `sha256:${"2".repeat(64)}`,
  frames: {
    base: "world", tool: "tool0", work: "world", workWorldM: [0, 0, 0],
    armBaseWorldM: [0.4127, 0.4, 0.893],
  },
  driven: ["J1", "J2", "J3", "J4", "J5", "J6"],
  held: {},
  resetPose: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
};

export const TEST_ROBOTS: RobotDescriptorSummary[] = [ROSIE_1400_V3, ROSIE_1420_V1];

/** Put the session back to "the server listed these two". */
export function seedRobotCatalogue(): void {
  setRobotCatalogue(TEST_ROBOTS);
}
