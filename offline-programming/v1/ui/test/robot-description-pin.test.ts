/**
 * Which version of its robot a program was authored against.
 *
 * The name is stored and everything resolves by it; the hash is what tells
 * a program written against last month's description from one written
 * against today's. A robot's geometry, limits or torch can change under a
 * name that does not, and the cell answers that by refusing the plan -- it
 * cannot answer whether the person meant it. These hold the one place that
 * can: the pin, the drift it makes visible, and what adopting does and does
 * not do.
 */

import assert from "node:assert/strict";
import test from "node:test";

import {
  adoptRobotDescription,
  createProject,
  pinRobotDescription,
  robotDescriptionDrift,
  robotDescriptionPinFor,
  setRobotDescriptionPin,
  type OfflineProgrammingProject,
} from "../src/domain/project.ts";
import { setRobotCatalogue } from "../src/domain/robots.ts";
import { ROSIE_1400_V3, ROSIE_1420_V1, seedRobotCatalogue } from "./support/robot-catalogue.ts";

const MOVED = `sha256:${"9".repeat(64)}`;

/** The same catalogue, with the 1400's description changed under its name --
 *  a refit mesh, a corrected limit, another torch. */
function describeMovedRobot(): void {
  setRobotCatalogue([{ ...ROSIE_1400_V3, robotDescriptionSha256: MOVED }, ROSIE_1420_V1]);
}

test("a program is pinned to the description it was authored against", () => {
  seedRobotCatalogue();
  const project = createProject({ robotModel: "rosie_1400_v3" });
  assert.equal(project.robot.description?.robotDescriptionSha256, ROSIE_1400_V3.robotDescriptionSha256);
  assert.ok(project.robot.description?.pinnedAt);
  assert.equal(project.robot.description?.adopted, undefined);
  assert.equal(robotDescriptionDrift(project), null);

  // Choosing another robot pins that robot's description, and keeps none of
  // the history: an adoption was a decision about a different robot.
  const draft = structuredClone(project) as OfflineProgrammingProject;
  draft.robot.description = { robotDescriptionSha256: "sha256:old", pinnedAt: "2026-01-01T00:00:00Z", adopted: [
    { from: "sha256:a", to: "sha256:old", at: "2026-01-01T00:00:00Z" },
  ] };
  draft.robot.model = "rosie_1420_v1";
  setRobotDescriptionPin(draft, "2026-09-18T10:00:00Z");
  assert.deepEqual(draft.robot.description, {
    robotDescriptionSha256: ROSIE_1420_V1.robotDescriptionSha256,
    pinnedAt: "2026-09-18T10:00:00Z",
  });
});

test("a description that moved under the robot's name is drift, and it is named", () => {
  seedRobotCatalogue();
  const project = createProject({ robotModel: "rosie_1400_v3" });
  assert.equal(robotDescriptionDrift(project), null);

  describeMovedRobot();
  assert.deepEqual(robotDescriptionDrift(project), {
    from: ROSIE_1400_V3.robotDescriptionSha256,
    to: MOVED,
  });
  // The other robot did not move.
  assert.equal(robotDescriptionDrift(createProject({ robotModel: "rosie_1420_v1" })), null);
  seedRobotCatalogue();
});

test("adopting re-pins, records what was left behind, and claims nothing else", () => {
  seedRobotCatalogue();
  const project = createProject({ robotModel: "rosie_1400_v3" });
  const before = structuredClone(project) as OfflineProgrammingProject;
  describeMovedRobot();

  adoptRobotDescription(project, "2026-09-18T12:00:00Z");
  assert.equal(project.robot.description?.robotDescriptionSha256, MOVED);
  assert.deepEqual(project.robot.description?.adopted, [
    { from: ROSIE_1400_V3.robotDescriptionSha256, to: MOVED, at: "2026-09-18T12:00:00Z" },
  ]);
  assert.equal(robotDescriptionDrift(project), null);

  // What it did NOT do: the program, its rows, its plan and its cell settings
  // are what they were. Adopting is a note about provenance, not an edit, and
  // the plan it leaves behind is one the cell still refuses.
  assert.deepEqual(project.nodes, before.nodes);
  assert.deepEqual(project.rows, before.rows);
  assert.deepEqual(project.plan, before.plan);
  assert.equal(project.identity.revision, before.identity.revision);
  assert.equal(project.denseTrajectory, before.denseTrajectory);

  // Adopting again when nothing moved re-pins without inventing a record.
  adoptRobotDescription(project, "2026-09-18T13:00:00Z");
  assert.equal(project.robot.description?.adopted?.length, 1);
  seedRobotCatalogue();
});

test("a program saved before the pin takes the current description, once", () => {
  seedRobotCatalogue();
  const stored = createProject({ robotModel: "rosie_1400_v3" });
  delete stored.robot.description;

  const pinned = pinRobotDescription(stored, "2026-09-18T09:00:00Z");
  assert.notEqual(pinned, stored, "pinning returns the pinned copy");
  assert.deepEqual(pinned.robot.description, {
    robotDescriptionSha256: ROSIE_1400_V3.robotDescriptionSha256,
    pinnedAt: "2026-09-18T09:00:00Z",
  });
  // Opening is not editing: the revision and the plan are untouched.
  assert.equal(pinned.identity.revision, stored.identity.revision);
  assert.deepEqual(pinned.plan, stored.plan);
  // It is the FIRST open only. A program pinned to a description that has
  // since moved keeps its pin, because that difference is the whole point.
  describeMovedRobot();
  assert.equal(pinRobotDescription(pinned), pinned);
  assert.deepEqual(robotDescriptionDrift(pinned), {
    from: ROSIE_1400_V3.robotDescriptionSha256,
    to: MOVED,
  });
  seedRobotCatalogue();
});

test("a robot this server has no description for is said, not drifted", () => {
  seedRobotCatalogue();
  const stored = createProject({ robotModel: "rosie_9999_x1" });
  assert.equal(stored.robot.description, null);
  assert.equal(robotDescriptionPinFor("rosie_9999_x1"), null);
  // Nothing to pin to and nothing to compare: the card says the server has no
  // description for this robot, which is a different sentence from drift.
  assert.equal(pinRobotDescription(stored), stored);
  assert.equal(robotDescriptionDrift(stored), null);
  adoptRobotDescription(stored);
  assert.equal(stored.robot.description, null);
});
