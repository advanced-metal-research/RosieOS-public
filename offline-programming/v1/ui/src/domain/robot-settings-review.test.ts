import assert from "node:assert/strict";
import test from "node:test";
import { adoptRobotDescription, createProject, pinRobotDescription } from "./project.ts";
import { robotSettingsReview } from "./robot-settings-review.ts";
import { parseRobotCatalogue, setConnectedRobot, setRobotCatalogue,
  type RobotDescriptorSummary, type RobotSettings } from "./robots.ts";

const FROM = `sha256:${"a".repeat(64)}`, TO = `sha256:${"b".repeat(64)}`;
const settings: RobotSettings = {
  revision: 1, joints: [{ name: "J1", type: "revolute", lower: -Math.PI, upper: Math.PI, rated_velocity: Math.PI, gear_ratio: 100 }],
  planning: { speed_ceiling_rad_s: 1, joints: { J1: { velocity_rad_s: 1, acceleration_rad_s2: 2 } } },
  torch: { tool_frame: "torch", electrode_axis: [1, 0, 0] }, resources: { "robot.urdf": FROM },
};
function entry(hash = FROM, overrides: Partial<RobotSettings> = {}): RobotDescriptorSummary {
  return {
    modelId: "test_robot", robotDescriptionSha256: hash,
    frames: { base: "world", tool: "torch", work: "floor", workWorldM: [0, 0, 0], armBaseWorldM: [0, 0, 0] },
    driven: ["J1"], held: {}, resetPose: {}, settings: structuredClone({ ...settings, ...overrides }),
  };
}
test.afterEach(() => { setConnectedRobot(null); });

test("new and adopted pins save settings without sharing the catalogue's mutable objects", () => {
  const descriptor = entry();
  setRobotCatalogue([descriptor]);
  const project = createProject({ robotModel: "test_robot" });
  assert.equal(project.robot.description!.settingsSnapshot!.robotDescriptionSha256, FROM);
  descriptor.settings!.joints[0]!.gear_ratio = 50;
  assert.equal(project.robot.description!.settingsSnapshot!.settings.joints[0]!.gear_ratio, 100);
  setRobotCatalogue([entry(TO, { revision: 2 })]);
  adoptRobotDescription(project, "2026-09-22T00:00:00Z");
  assert.equal(project.robot.description!.settingsSnapshot!.settings.revision, 2);
  assert.equal(project.robot.description!.settingsSnapshot!.robotDescriptionSha256, TO);
  assert.deepEqual(project.robot.description!.adopted, [{ from: FROM, to: TO, at: "2026-09-22T00:00:00Z" }]);
});

test("new pins never attach local settings to an unavailable connected robot identity", () => {
  setRobotCatalogue([entry()]);
  setConnectedRobot({ modelId: "test_robot", robotDescriptionSha256: TO });
  const project = createProject({ robotModel: "test_robot" });
  assert.equal(project.robot.description!.robotDescriptionSha256, TO);
  assert.equal(project.robot.description!.settingsSnapshot, undefined);
  delete project.robot.description;
  const pinned = pinRobotDescription(project);
  assert.equal(pinned.robot.description!.robotDescriptionSha256, TO);
  assert.equal(pinned.robot.description!.settingsSnapshot, undefined);
});

test("saved snapshots produce human units, resource notices and no invented change dates", () => {
  setRobotCatalogue([entry()]);
  const project = createProject({ robotModel: "test_robot" });
  const next = entry(TO, { revision: 2, joints: [{ ...settings.joints[0]!, lower: -Math.PI / 2, rated_velocity: Math.PI / 2, gear_ratio: 50 }], resources: { "robot.urdf": TO } });
  setRobotCatalogue([next]);
  const review = robotSettingsReview(project);
  assert.equal(review.date, null);
  assert.deepEqual(review.rows.find(row => row.label === "J1 lower position limit"), { label: "J1 lower position limit", before: "-180 °", after: "-90 °" });
  assert.deepEqual(review.rows.find(row => row.label === "J1 rated joint speed"), { label: "J1 rated joint speed", before: "180 °/s", after: "90 °/s" });
  assert.deepEqual(review.rows.find(row => row.label === "J1 gear ratio"), { label: "J1 gear ratio", before: "100:1", after: "50:1" });
  assert.ok(review.rows.some(row => row.label === "Description file: robot.urdf"));
  assert.ok(review.unchanged.includes("Recorded torch configuration"));
  assert.ok(!JSON.stringify(review.rows).includes("sha256:"));
  assert.ok(!review.unchanged.some(note => note.includes("geometry")));
});

test("an exact historical catalogue entry can explain a pin with no saved snapshot", () => {
  setRobotCatalogue([entry()]);
  const project = createProject({ robotModel: "test_robot" });
  delete project.robot.description!.settingsSnapshot;
  setRobotCatalogue([entry(TO, { revision: 2 }), entry()]);
  const review = robotSettingsReview(project);
  assert.match(review.provenance, /catalogue/);
  assert.deepEqual(review.rows, [{ label: "Robot definition revision", before: "1", after: "2" }]);
});

test("wrong identity snapshots and missing exact current descriptions are not compared", () => {
  setRobotCatalogue([entry()]);
  const project = createProject({ robotModel: "test_robot" });
  project.robot.description!.settingsSnapshot!.modelId = "other_robot";
  setRobotCatalogue([entry(TO)]);
  assert.match(robotSettingsReview(project).explanation!, /cannot be reconstructed/);
  project.robot.description!.settingsSnapshot!.modelId = "test_robot";
  setConnectedRobot({ modelId: "test_robot", robotDescriptionSha256: `sha256:${"c".repeat(64)}` });
  const unavailable = robotSettingsReview(project);
  assert.match(unavailable.explanation!, /current connected robot's exact settings are not available/);
  assert.deepEqual(unavailable.rows, []);
});

test("the audited positioner correction only applies to its exact model and full identities", () => {
  const old = "sha256:803ab172c4e2e7f93f33162d3060a51cc86014747b7617864090f09aaeb9ecd7";
  const current = "sha256:686029493a224a51a506d7ae0a056f9080d941167e64911cc03c7d6ac1ed24a9";
  setRobotCatalogue([{ ...entry(old), modelId: "rosie_1400_v3" }]);
  const project = createProject({ robotModel: "rosie_1400_v3" });
  delete project.robot.description!.settingsSnapshot;
  setRobotCatalogue([{ ...entry(current), modelId: "rosie_1400_v3" }]);
  const review = robotSettingsReview(project);
  assert.equal(review.date, "2026-09-21T23:32:45+10:00");
  assert.match(review.provenance, /commit date, not the deployment date/);
  assert.equal(review.technical.commit, "29658345a222864ab9eece859061cce44b10834f");
  assert.deepEqual(review.rows[1], { label: "J7, J8 and J9 gear ratio", before: "10:1", after: "100:1" });
  assert.deepEqual(review.rows[2], { label: "J7, J8 and J9 rated joint speed", before: "1,800 °/s", after: "180 °/s" });
  project.robot.description!.robotDescriptionSha256 = old.slice(0, -1) + "0";
  assert.equal(robotSettingsReview(project).date, null);
  assert.ok(robotSettingsReview(project).rows.every(row => row.before === "Not recorded"));
  assert.match(robotSettingsReview(project).title, /Current settings/);
});

test("a malformed imported snapshot falls back to current values without inventing a comparison", () => {
  setRobotCatalogue([entry()]);
  const project = createProject({ robotModel: "test_robot" });
  (project.robot.description!.settingsSnapshot as unknown as { frames: unknown }).frames = { workWorldM: "bad" };
  setRobotCatalogue([entry(TO, { revision: 2 })]);
  const review = robotSettingsReview(project);
  assert.match(review.title, /previous values unavailable/);
  assert.match(review.explanation!, /not confirmed changes/);
  assert.ok(review.rows.some(row => row.label === "J1 rated joint speed" && row.after === "180 °/s"));
  assert.ok(review.rows.every(row => row.before === "Not recorded"));
  assert.deepEqual(review.unchanged, []);
});

test("prismatic ratings use millimetres while explicitly angular planning fields keep angular units", () => {
  setRobotCatalogue([entry(FROM, { joints: [{ name: "slide", type: "prismatic", lower: 0, upper: 1, rated_velocity: 0.1 }] })]);
  const project = createProject({ robotModel: "test_robot" });
  setRobotCatalogue([entry(TO, { joints: [{ name: "slide", type: "prismatic", lower: 0, upper: 2, rated_velocity: 0.2 }],
    planning: { joints: { slide: { velocity_rad_s: Math.PI } } } })]);
  const rows = robotSettingsReview(project).rows;
  assert.equal(rows.find(row => row.label === "slide upper position limit")!.after, "2000 mm");
  assert.equal(rows.find(row => row.label === "slide rated joint speed")!.after, "200 mm/s");
  assert.equal(rows.find(row => row.label === "slide planning speed")!.after, "180 °/s");
});

test("optional settings parsing preserves old catalogue support and removes non-finite values", () => {
  const wire = {
    model_id: "test_robot", robot_description_sha256: FROM,
    frames: { work: "floor", work_world_m: [0, 0, 0] },
  };
  assert.equal(parseRobotCatalogue({ robots: [wire] })[0]!.settings, undefined);
  const parsed = parseRobotCatalogue({ robots: [{ ...wire, settings: {
    revision: Infinity, joints: [{ name: "J1", type: "revolute", lower: NaN, upper: 1, gear_ratio: "100" }],
    torch: { tool_frame: "torch", electrode_axis: [1, Infinity, 0] },
  } }] })[0]!.settings!;
  assert.equal(parsed.revision, undefined);
  assert.deepEqual(parsed.joints[0], { name: "J1", type: "revolute", upper: 1 });
  assert.equal(parsed.torch!.electrode_axis, undefined);
});
