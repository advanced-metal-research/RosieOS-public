import assert from "node:assert/strict";
import test from "node:test";

import {
  DENSE_EXECUTION_PATH,
  DenseExecutionClient,
  DenseExecutionError,
  normalizeDenseStatus,
  type DenseRobotCell,
} from "../src/execution/denseClient.ts";
import {
  describeRobotCell,
  plannedWithoutACell,
  machineConfigurationNote,
  robotMismatch,
  robotRefFrom,
  robotRefFromBlob,
  robotRefusalSentence,
} from "../src/execution/robotCell.ts";
import { cellSettingsKey, type CellSettings, type DenseRobotRef } from "../src/domain/project.ts";

const TREE = `sha256:${"a".repeat(64)}`;
const CELL = `sha256:${"c".repeat(64)}`;

const target = (overrides: Partial<DenseRobotCell> = {}): DenseRobotCell => ({
  configured: true, valid: true, modelId: "rosie_1420_v1", robotDescriptionSha256: TREE,
  machinePlanningCalibrationSha256: CELL, machinePlanningCalibrationPresent: true, machineConfigurationSha256: "", error: null, ...overrides,
});
const plan = (overrides: Partial<DenseRobotRef> = {}): DenseRobotRef => ({
  modelId: "rosie_1420_v1", robotDescriptionSha256: TREE, machinePlanningCalibrationSha256: CELL, machinePlanningCalibrationSource: "target", ...overrides,
});

const jsonResponse = (status: number, body: unknown): Response =>
  new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });

test("the daemon's robot identity and its {reason, detail} last error normalise", () => {
  const status = normalizeDenseStatus({
    available: true,
    daemon: {
      executor: { state: "idle", robot_cell: { configured: true, valid: false, model_id: "rosie_1420_v1",
        robot_description_sha256: TREE, machine_planning_calibration_sha256: "", machine_planning_calibration_present: true, error: "machine_planning_calibration.json widens J2" } },
      last_error: { reason: "robot_cell_mismatch", detail: "machine_planning_calibration_sha256: plan a, cell b" },
    },
  });
  assert.deepEqual(status.daemon?.robotCell, {
    configured: true, valid: false, modelId: "rosie_1420_v1", robotDescriptionSha256: TREE,
    machinePlanningCalibrationSha256: "", machinePlanningCalibrationPresent: true, machineConfigurationSha256: "",
    error: "machine_planning_calibration.json widens J2",
  });
  assert.deepEqual(status.daemon?.lastError, { reason: "robot_cell_mismatch", detail: "machine_planning_calibration_sha256: plan a, cell b" });
  // A daemon from before the contract: no robot, and a plain string error.
  const older = normalizeDenseStatus({ available: true, daemon: { executor: { state: "idle" }, last_error: "boom" } });
  assert.equal(older.daemon?.robotCell, null);
  assert.deepEqual(older.daemon?.lastError, { reason: "boom", detail: "" });
  assert.equal(normalizeDenseStatus({ daemon: { executor: "absent", last_error: null } }).daemon?.lastError, null);
});

test("cell() fetches the target's bytes and identity", async () => {
  const calls: string[] = [];
  const client = new DenseExecutionClient(async (input) => {
    calls.push(String(input));
    return jsonResponse(200, { robot_cell: { configured: true, valid: true, model_id: "rosie_1420_v1",
      robot_description_sha256: TREE, machine_planning_calibration_sha256: CELL, machine_planning_calibration_present: false, error: "" }, machine_planning_calibration_base64: "e30K" });
  });
  const body = await client.cell();
  assert.deepEqual(calls, [`${DENSE_EXECUTION_PATH}/cell`]);
  assert.equal(body.machinePlanningCalibrationBase64, "e30K");
  assert.equal(body.robotCell.robotDescriptionSha256, TREE);
  assert.equal(body.robotCell.machinePlanningCalibrationPresent, false);

  const refusing = new DenseExecutionClient(async () =>
    jsonResponse(409, { error: "robot_cell_unavailable", detail: "no robot.urdf" }));
  await assert.rejects(refusing.cell(), (error: unknown) =>
    error instanceof DenseExecutionError && error.code === "robot_cell_unavailable");
});

test("a plan's robot block reads from the header or the summary, whole or not at all", () => {
  const block = { model_id: "rosie_1420_v1", robot_description_sha256: TREE, machine_planning_calibration_sha256: CELL, machine_planning_calibration_source: "empty" };
  assert.deepEqual(robotRefFrom(block), plan({ machinePlanningCalibrationSource: "empty" }));
  assert.equal(robotRefFrom({ ...block, machine_planning_calibration_source: "cache" }), null);
  assert.equal(robotRefFrom({ ...block, robot_description_sha256: undefined }), null);
  assert.equal(robotRefFrom(null), null);

  const header = new TextEncoder().encode(JSON.stringify({ schema: "x", robot_cell: block }));
  const blob = new Uint8Array(4 + header.length + 3);
  new DataView(blob.buffer).setUint32(0, header.length, false);
  blob.set(header, 4);
  assert.deepEqual(robotRefFromBlob(blob), plan({ machinePlanningCalibrationSource: "empty" }));
  assert.equal(robotRefFromBlob(blob.subarray(0, 10)), null);
  assert.equal(robotRefFromBlob(new Uint8Array([0, 0, 0, 2, 123, 125])), null);
});

test("the panel says, as the daemon would, why a plan will be refused", () => {
  assert.equal(robotMismatch(plan(), target()), null);
  // A daemon from before the contract judges by its own rules.
  assert.equal(robotMismatch(undefined, null), null);
  assert.match(robotMismatch(undefined, target())!, /names no robot/);
  assert.match(robotMismatch(plan(), target({ configured: false, valid: false }))!, /no robot description/);
  assert.match(robotMismatch(plan(), target({ valid: false, error: "bad cap" }))!, /invalid: bad cap/);
  const other = robotMismatch(plan({ modelId: "rosie_1400_v3", machinePlanningCalibrationSha256: `sha256:${"d".repeat(64)}` }), target())!;
  assert.match(other, /re-fetch cell settings and re-plan/);
  assert.match(other, /model rosie_1400_v3 ≠ rosie_1420_v1/);
  assert.match(other, /machine_planning_calibration\.json dddddddddddd ≠ cccccccccccc/);
  assert.doesNotMatch(other, /description/);
  assert.match(robotMismatch(plan({ robotDescriptionSha256: `sha256:${"b".repeat(64)}` }), target())!, /description bbbbbbbbbbbb ≠ aaaaaaaaaaaa/);
});

test("robot refusals read as sentences, and carry the daemon's detail", () => {
  for (const code of ["robot_cell_missing", "robot_cell_unavailable", "robot_cell_mismatch", "robot_cell_changed"]) {
    assert.ok(robotRefusalSentence(code), code);
  }
  assert.equal(robotRefusalSentence("axes_not_ready"), null);
  assert.match(describeRobotCell(target()), /rosie_1420_v1 · description aaaaaaaaaaaa · machine_planning_calibration\.json cccccccccccc/);
  assert.match(describeRobotCell(target({ machinePlanningCalibrationPresent: false })), /machine_planning_calibration\.json none/);
  assert.match(describeRobotCell(target({ valid: false, error: "x" })), /INVALID: x/);
});

test("the cell settings key moves when the target's identity does", () => {
  const settings: CellSettings = {
    cell: "dev-cell", jetsonId: null, robotModelId: "rosie_1420_v1", runtimeMode: "sim",
    mappedAxisMask: 0x3f, rtcoreHost: null, pulledAt: "2026-09-18T00:00:00.000Z",
  };
  // Settings from before the contract keep their key; a pulled identity
  // joins it, and a new machine_planning_calibration.json moves it, so the plan reads stale.
  assert.equal(cellSettingsKey(settings), "dev-cell:rosie_1420_v1:63::");
  const pulled = { ...settings, robotDescriptionSha256: TREE, machinePlanningCalibrationSha256: CELL };
  assert.notEqual(cellSettingsKey(pulled), cellSettingsKey(settings));
  assert.notEqual(cellSettingsKey({ ...pulled, machinePlanningCalibrationSha256: `sha256:${"e".repeat(64)}` }), cellSettingsKey(pulled));
  // The bytes are never kept here -- the server fetches them when it packs --
  // so only when they were fetched changes, and that is not part of the key.
  assert.equal(cellSettingsKey({ ...pulled, cellFetchedAt: "later" }), cellSettingsKey(pulled));
});

test("a plan made with no target says so, and it is not a refusal", () => {
  const offline = { modelId: "rosie_1420_v1", robotDescriptionSha256: `sha256:${"a".repeat(64)}`,
    machinePlanningCalibrationSha256: `sha256:${"b".repeat(64)}`, machinePlanningCalibrationSource: "empty" as const };
  assert.match(plannedWithoutACell(offline)!, /planned without a cell/);
  assert.match(plannedWithoutACell(offline)!, /pull its settings and plan again/);
  assert.equal(plannedWithoutACell({ ...offline, machinePlanningCalibrationSource: "target" }), null);
  assert.equal(plannedWithoutACell(null), null);
  // Offline is about what the plan read, not about what a target thinks: with
  // no target there is nothing to refuse it yet.
  assert.equal(robotMismatch(offline, null), null);
});

test("the machine's configuration is recorded in a plan and never refuses it", () => {
  const was = `sha256:${"1".repeat(64)}`;
  const now = `sha256:${"2".repeat(64)}`;
  // The header's block carries it when the plan was made for a machine, and
  // reads whole without it (a plan made offline).
  const block = { model_id: "rosie_1420_v1", robot_description_sha256: TREE, machine_planning_calibration_sha256: CELL,
    machine_planning_calibration_source: "target" };
  assert.equal(robotRefFrom({ ...block, machine_configuration_sha256: was })?.machineConfigurationSha256, was);
  assert.equal(robotRefFrom(block)?.machineConfigurationSha256, undefined);
  // Another configuration than the target's is a sentence, not a mismatch.
  const recorded = plan({ machineConfigurationSha256: was });
  assert.equal(robotMismatch(recorded, target({ machineConfigurationSha256: now })), null);
  assert.match(machineConfigurationNote(recorded, target({ machineConfigurationSha256: now })) ?? "", /configuration changed since this plan was made \(111111111111 → 222222222222\)/);
  assert.equal(machineConfigurationNote(recorded, target({ machineConfigurationSha256: was })), null);
  // Nothing to compare: an offline plan, or a target that does not say.
  assert.equal(machineConfigurationNote(plan(), target({ machineConfigurationSha256: now })), null);
  assert.equal(machineConfigurationNote(recorded, target()), null);
});
