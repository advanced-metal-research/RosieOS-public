import assert from "node:assert/strict";
import test from "node:test";

import {
  adaptCadQueryTopology,
  installBrowserFallbackPreview,
  topologyToSeams,
  type CadTopology,
} from "../src/adapters/cadquery.ts";
import { createWeldFromEdges } from "../src/adapters/weld-authoring.ts";
import { canonicalJoints, normalizeJointName } from "../src/adapters/joints.ts";
import {
  toRobotV4PlanRequest,
} from "../src/adapters/robot-v4.ts";
import { createProject } from "../src/domain/project.ts";
import {
  createBlankSpreadsheetProject,
  createSpreadsheetRow,
  insertSpreadsheetRow,
} from "../src/domain/spreadsheet.ts";
import { createDogBoxProject } from "../src/examples/dog-box.ts";

const topology: CadTopology = {
  schemaVersion: 1,
  unit: "m",
  sourceUnit: "mm",
  tool: "test",
  modelId: "cad-1",
  filename: "part.step",
  fingerprint: "sha256",
  sampleCount: 2,
  parts: [{ id: "part-1", edgeCount: 1 }],
  edges: [{
    id: "edge-1",
    label: null,
    partId: "part-1",
    geometryType: "LINE",
    sampleCount: 2,
    samplesM: [[1, 0.2, 0.3], [1.5, 0.2, 0.3]],
    lengthM: 0.5,
    boundsMinM: [1, 0.2, 0.3],
    boundsMaxM: [1.5, 0.2, 0.3],
    closed: false,
    sourceEdgeIds: [],
    sourceGeometryTypes: [],
    pathClass: "raw_fragment",
    isSeamCandidate: true,
  }],
  rawEdgeCount: 1,
  compositeEdgeCount: 0,
  seamCandidateCount: 1,
  recommendedSeams: [{
    edgeId: "edge-1",
    partId: "part-1",
    geometryType: "LINE",
    pathClass: "raw_fragment",
    window: { start: 0, end: 1 },
    reversed: false,
    reason: "test",
  }],
};

test("CadQuery adapter normalizes millimetres to metres and imports without a program", () => {
  const seams = topologyToSeams(topology);
  assert.deepEqual(seams[0]!.geometry.pointsM, [[1, 0.2, 0.3], [1.5, 0.2, 0.3]]);
  const project = adaptCadQueryTopology(createProject({ id: "project-1" }), topology);
  assert.equal(project.cad.fingerprint, "sha256");
  // P2.12: import carries the CAD block, fixture, and edge catalog only.
  assert.equal(project.rows.length, 0);
  assert.equal(project.seams.length, 0);
  assert.deepEqual(project.edgeCatalog.map((edge) => edge.id), ["edge-1"]);
  assert.deepEqual(project.fixtures[0]!.translationM, [-1.25, -0.2, -0.3]);
  // The program appears when the user creates a weld from the selected edge.
  const authored = createWeldFromEdges(project, {
    weldType: "fillet",
    segments: [{ edgeId: "edge-1", startS: 0, endS: 1, reversed: false }],
  });
  assert.ok(Math.abs(authored.rows[0]!.targetTcpPose[0] - 0.8756) < 1e-9);
});

test("browser fallback preview preserves a manually authored spreadsheet", () => {
  let project = createBlankSpreadsheetProject({
    name: "manual",
    robotModel: "rosie_1400_v3",
  });
  project.rows.push(createSpreadsheetRow(project, "motion"));
  project = insertSpreadsheetRow(project, 1, "motion");
  const rowIDs = project.rows.map(({ id }) => id);

  const preview = installBrowserFallbackPreview(project);

  assert.deepEqual(preview.rows.map(({ id }) => id), rowIDs);
  assert.equal(preview.rows.length, 2);
  assert.equal(preview.plan.state, "preview");
  assert.ok(preview.plan.trajectory.length > 0);
  assert.ok(preview.plan.trajectory.every(({ rowIndex }) => rowIndex < 2));
});

test("joint adapter always emits J1-J9 and maps external axis aliases", () => {
  const joints = canonicalJoints([90, 10, 70], ["J9", "J1", "J7a"]);
  assert.deepEqual(joints, [10, 0, 0, 0, 0, 0, 70, 0, 90]);
  assert.equal(normalizeJointName("joint_6"), "J6");
  assert.equal(normalizeJointName("bad"), null);
});

test("Robot V4 request is simulation-only and carries exact project revision", () => {
  const project = createDogBoxProject("2026-07-30T00:00:00.000Z");
  const request = toRobotV4PlanRequest(project);
  assert.equal(request.execution_mode, "simulation");
  assert.equal(request.project_revision, project.identity.revision);
  assert.deepEqual(request.joint_order, ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]);
  assert.equal(request.rows[0]!.target_joints_rad!.length, 9);
  assert.equal("armed" in request, false);
});

test("Robot V4 request sends authored row RPY with a compatible hemisphere constraint", () => {
  const project = createDogBoxProject("2026-07-30T00:00:00.000Z");
  const request = toRobotV4PlanRequest(project);
  const weldRow = request.rows.find((row) => row.motion_phase === "weld");
  assert.ok(weldRow?.target_tcp_m);
  assert.notDeepEqual(weldRow.target_tcp_m.slice(3), [0, 0, 0]);
  assert.deepEqual(request.constraints?.tool_axis_constraint, {
    tool_frame: "end_effector",
    tip_link: "tool0",
    world_frame: "world",
    tool_axis_xyz: [1, 0, 0],
    world_axis_xyz: [0, 0, -1],
    offset_reference_axis_xyz: [1, 0, 0],
    offset_deg: 0,
    tolerance_minus_deg: 90,
    tolerance_plus_deg: 90,
    weight: 0.5,
  });
});

test("boot repair leaves disabled rowless seams alone (OF-1: reload must not mutate saved programs)", async () => {
  const { ensureCadPathRows } = await import("../src/adapters/cadquery.ts");
  const { createDogBoxProject } = await import("../src/examples/dog-box.ts");
  const project = createDogBoxProject();
  // Simulate an over-budget import: one seam disabled, rows omitted, and the
  // omission RECORDED in the import's over-budget marker.
  const victim = project.seams[0]!;
  victim.enabled = false;
  project.rows = project.rows.filter((row) => row.seamId !== victim.id);
  project.program.settings.gradient_move_v1 = {
    over_budget_seam_ids: [victim.id],
  };
  const rowCount = project.rows.length;
  assert.equal(ensureCadPathRows(project), false);
  assert.equal(project.rows.length, rowCount);
  // Without the marker the same rowless seam is legacy damage — restored.
  project.program.settings.gradient_move_v1 = { over_budget_seam_ids: [] };
  assert.equal(ensureCadPathRows(project), true);
  assert.ok(project.rows.some((row) => row.seamId === victim.id));
});
