import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import {
  ROBOT_V4_PROGRAM_MAX_ROWS,
  canonicalProgramJSON,
  computeProgramDigest,
  parseRobotV4AcceptedPlan,
  parseRobotV4Catalog,
  parseRobotV4Program,
  reviseRobotV4Program,
  type RobotV4Program,
} from "../src/contracts/robot-v4-program.ts";
import {
  acceptedPlanTrajectory,
  fromRobotV4Catalog,
  toRobotV4Program,
} from "../src/adapters/robot-v4.ts";
import { createProject } from "../src/domain/project.ts";
import { migrateProject } from "../src/domain/serialization.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

const fixtures = new URL("../../../../steamdeck/real/v4/contracts/programs/fixtures/", import.meta.url);
const schemas = new URL("../../../../steamdeck/real/v4/contracts/programs/", import.meta.url);

async function jsonFile(base: URL, name: string): Promise<unknown> {
  return JSON.parse(await readFile(new URL(name, base), "utf8"));
}

test("Robot V4 contract schemas and valid fixtures are parseable JSON", async () => {
  for (const name of [
    "robot.v4.program.v1.schema.json",
    "robot.v4.program-catalog.v1.schema.json",
    "robot.v4.accepted-plan.v1.schema.json",
  ]) {
    const schema = await jsonFile(schemas, name) as Record<string, unknown>;
    assert.equal(schema.$schema, "https://json-schema.org/draft/2020-12/schema");
    assert.equal(typeof schema.$id, "string");
  }
  parseRobotV4Program(await jsonFile(fixtures, "complete-program.json"));
  parseRobotV4Catalog(await jsonFile(fixtures, "complete-catalog.json"));
  parseRobotV4AcceptedPlan(await jsonFile(fixtures, "accepted-simulation-plan.json"));
});

test("Robot V4 program rejects a missing stable row identity", async () => {
  const invalid = await jsonFile(fixtures, "invalid-program-missing-row-id.json");
  assert.throws(
    () => parseRobotV4Program(invalid),
    /row_id is required/,
  );
});

test("Robot V4 program row bound is the 65536 contract backstop (4096 before 10 ms rows), not the old 64", async () => {
  const program = parseRobotV4Program(await jsonFile(fixtures, "complete-program.json"));
  const template = program.rows.find((row) => row.row_type === "motion")!;
  const expand = (rowCount: number): RobotV4Program => ({
    ...structuredClone(program),
    rows: Array.from({ length: rowCount }, (_, index) => ({
      ...structuredClone(template),
      row_id: `row-${index + 1}`,
      row_index: index,
      row_number: index + 1,
      row_hash: `rowhash-${index + 1}`,
    })),
  });
  // The exact Dog Box needs 730 rows at 0.1 mm weld tolerance (ledger L21).
  assert.equal(parseRobotV4Program(expand(730)).rows.length, 730);
  assert.equal(
    parseRobotV4Program(expand(ROBOT_V4_PROGRAM_MAX_ROWS)).rows.length,
    ROBOT_V4_PROGRAM_MAX_ROWS,
  );
  assert.throws(
    () => parseRobotV4Program(expand(ROBOT_V4_PROGRAM_MAX_ROWS + 1)),
    /program\.rows must contain/,
  );
});

test("canonical digest ignores planning evidence but covers editable inputs", async () => {
  const program = parseRobotV4Program(await jsonFile(fixtures, "complete-program.json"));
  const first = await computeProgramDigest(program);
  const evidenceOnly = structuredClone(program);
  evidenceOnly.planner_status = "planned";
  evidenceOnly.validation_state = "valid";
  evidenceOnly.planned_path_revision = 7;
  evidenceOnly.planned_sample_count = 100;
  evidenceOnly.rows[0]!.planner_status = "planned";
  evidenceOnly.rows[0]!.planned_sample_count = 100;
  assert.equal(await computeProgramDigest(evidenceOnly), first);

  const edited = structuredClone(program);
  edited.rows[1]!.tcp_velocity_m_s = 0.06;
  assert.notEqual(await computeProgramDigest(edited), first);
  assert.notEqual(canonicalProgramJSON(edited), canonicalProgramJSON(program));
});

test("editing a program advances revision and invalidates only changed row planning identity", async () => {
  const program = parseRobotV4Program(await jsonFile(fixtures, "complete-program.json"));
  const unchangedHash = program.rows[0]!.row_hash;
  const changedHash = program.rows[1]!.row_hash;
  const revised = await reviseRobotV4Program(
    program,
    "steamdeck",
    (draft) => {
      draft.rows[1]!.target_tcp_m![0] += 0.001;
      draft.rows[1]!.source_state = "teach_modified";
    },
    "2026-07-30T10:01:00Z",
  );

  assert.equal(revised.program_revision, program.program_revision + 1);
  assert.equal(revised.base_program_revision, program.program_revision);
  assert.equal(revised.program_state, "needs_plan");
  assert.equal(revised.planner_status, "stale");
  assert.equal(revised.planned_path_revision, 0);
  assert.equal(revised.rows[0]!.row_hash, unchangedHash);
  assert.notEqual(revised.rows[1]!.row_hash, changedHash);
  assert.equal(revised.rows[1]!.revision, revised.program_revision);
  assert.match(revised.program_digest!, /^sha256:[0-9a-f]{64}$/);
});

test("Motion Server catalog round-trips through the Offline project without losing SteamDeck fields", async () => {
  const catalog = await jsonFile(fixtures, "complete-catalog.json");
  const project = fromRobotV4Catalog(
    catalog,
    "catalog-round-trip-program",
    { timestamp: "2026-07-30T10:02:00Z" },
  );
  assert.equal(project.program.programRevision, 2);
  assert.equal(project.program.baseProgramRevision, 2);
  assert.equal(project.program.catalogRevision, 11);
  assert.equal(project.program.editSource, "steamdeck");
  assert.equal(project.rows[0]!.sourceState, "teach_modified");
  assert.equal(project.cad.fingerprint, "catalog:catalog-round-trip-program");

  const roundTrip = toRobotV4Program(project);
  assert.deepEqual(roundTrip.server_private_hint, { preserve: "top-level" });
  assert.deepEqual(roundTrip.rows[0]!.steamdeck_extra, { preserve: "row-level" });
  assert.deepEqual(roundTrip.extensions, {
    unknown_field_round_trip: { keep: true },
  });
  assert.equal(roundTrip.rows[0]!.row_id, "row-1");
  assert.equal(roundTrip.rows[0]!.revision, 2);
});

test("CAD rows, catalog imports, serialized rows, and program defaults agree on end_effector", async () => {
  const cad = createProject();
  assert.equal(cad.program.defaults.default_tool_frame, "end_effector");
  // The frame every row references must be declared in the project's own
  // frame table (closure condition 3, 2026-08-04).
  assert.ok(cad.frames.some((frame) => frame.id === "end_effector"));
  assert.ok(
    createDogBoxProject().rows.every((row) => row.toolFrameId === "end_effector"),
  );

  const catalog = structuredClone(
    parseRobotV4Catalog(await jsonFile(fixtures, "complete-catalog.json")),
  );
  catalog.programs[0]!.rows[0]!.tool_frame = "end_effector";
  assert.equal(
    fromRobotV4Catalog(catalog, catalog.programs[0]!.program_id).rows[0]!.toolFrameId,
    "end_effector",
  );

  const stored = structuredClone(createDogBoxProject()) as unknown as Record<string, unknown>;
  const storedRows = (stored.rows as Array<Record<string, unknown>>);
  delete storedRows[0]!.toolFrameId;
  assert.equal(migrateProject(stored).rows[0]!.toolFrameId, "end_effector");
});

test("workstation playback is derived directly from the accepted-plan joint trajectory", async () => {
  const project = fromRobotV4Catalog(
    await jsonFile(fixtures, "complete-catalog.json"),
    "catalog-round-trip-program",
    { timestamp: "2026-07-30T10:02:00Z" },
  );
  const accepted = parseRobotV4AcceptedPlan(
    await jsonFile(fixtures, "accepted-simulation-plan.json"),
  );

  const trajectory = acceptedPlanTrajectory(project, accepted);

  assert.equal(trajectory.length, accepted.trajectory.length);
  assert.deepEqual(
    trajectory.map(({ timeS, jointPositionsRad }) => [timeS, jointPositionsRad]),
    accepted.trajectory.map(({ time_s, joint_positions_rad }) => [
      time_s,
      joint_positions_rad,
    ]),
  );
  const mismatched = structuredClone(accepted);
  mismatched.trajectory[0]!.row_id = "different-row";
  assert.throws(
    () => acceptedPlanTrajectory(project, mismatched),
    /does not match spreadsheet row identity/,
  );
});
